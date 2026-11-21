"""The sim node's 3D view data (viz/model, viz/meshes.bin, viz/frame), the markers API and the --sim world
socket resolution. docs/sim.md, "3D view and markers"."""
import numpy as np
import pytest
from fastapi.testclient import TestClient
from sim_helpers import make_world

from robot_utils.nodes.sim import viz
from robot_utils.nodes.sim.node import SimNode
from robot_utils.nodes.sim.server import create_app


@pytest.fixture(scope="module")
def world():
    return make_world()


@pytest.fixture
def api(world, tmp_path):
    node = SimNode(world, "vizt", str(tmp_path / "scenes"))
    with TestClient(create_app(node, start=False)) as c:
        yield c, node


def test_cluster_simplifies_and_keeps_valid_triangles():
    g = np.linspace(0, 0.01, 21)                                   # a 21 x 21 vertex grid, 0.5 mm apart
    xs, ys = np.meshgrid(g, g)
    v = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], 1)
    f = []
    for i in range(20):
        for j in range(20):
            a = i * 21 + j
            f += [[a, a + 1, a + 22], [a, a + 22, a + 21]]
    f = np.array(f)
    nv, nf = viz.cluster(v, f, 0.002)
    assert len(nv) < len(v) / 4 and 0 < len(nf) < len(f)
    assert nf.max() < len(nv) and np.all((nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]))
    assert np.allclose(nv.min(0)[:2], 0, atol=1e-3) and np.allclose(nv.max(0)[:2], 0.01, atol=1.1e-3)
    same_v, same_f = viz.cluster(v, f, 0)
    assert same_v is v and same_f is f


def test_viz_model_draws_every_robot_body_with_its_visual_geoms(world):
    info, blob = viz.model(world)
    names = [b["body"] for b in info["bodies"]]
    assert names[0] == "world" and "base_link" in names and "arm_end_link" in names
    drawn = {g["body"] for g in info["geoms"]}
    for i, b in enumerate(info["bodies"]):                         # every body with geometry is drawn ...
        if b["kind"] == "robot" and any(world.m.geom_bodyid[k] == i for k in range(world.m.ngeom)):
            assert i in drawn, b["body"]
    robot = [g for g in info["geoms"] if info["bodies"][g["body"]]["kind"] == "robot"]
    assert robot and all(g["visual"] for g in robot)               # ... with its visual geoms only
    assert {g["type"] for g in robot} >= {"box", "cylinder", "mesh"}
    assert len(blob) == info["blob_bytes"]
    for m in info["meshes"]:                                       # the blob holds every mesh as indexed
        dt = np.uint16 if m["index"] == "u16" else np.uint32
        v = np.frombuffer(blob, "<f4", m["nvert"] * 3, m["vert_offset"])
        f = np.frombuffer(blob, dt, m["nface"] * 3, m["face_offset"])
        assert np.isfinite(v).all() and f.max() < m["nvert"]
    assert sum(m["nvert"] for m in info["meshes"]) < 80_000         # simplified (as compiled: ~216 k)
    assert viz.model(world)[1] is blob                             # cached


def test_viz_api_model_meshes_frame_and_recompile(api):
    c, node = api
    info = c.get("/viz/model").json()
    r = c.get("/viz/meshes.bin", params={"generation": info["generation"]})
    assert r.status_code == 200 and len(r.content) == info["blob_bytes"]
    assert c.get("/viz/meshes.bin", params={"generation": info["generation"] + 1}).status_code == 409
    f = c.get("/viz/frame").json()
    nb = len(info["bodies"])
    assert len(f["pos"]) == 3 * nb and len(f["quat"]) == 4 * nb and f["generation"] == info["generation"]
    i = [b["body"] for b in info["bodies"]].index("base_link")
    assert f["pos"][3 * i:3 * i + 3] == pytest.approx(f["base"]["xyz"], abs=1e-4)
    c.post("/scene/objects", json={"name": "crate", "size": [0.2, 0.2, 0.2], "xyz": [1.0, 0.0, 0.1]})
    try:
        info2 = c.get("/viz/model").json()
        assert info2["generation"] > info["generation"]
        obj = next(k for k, b in enumerate(info2["bodies"]) if b["kind"] == "object")
        assert info2["bodies"][obj]["name"] == "crate"
        assert [g["type"] for g in info2["geoms"] if g["body"] == obj] == ["box"]
        assert c.get("/viz/frame").json()["generation"] == info2["generation"]
    finally:
        c.delete("/scene/objects/crate")


def test_markers_post_list_replace_delete(api):
    c, node = api
    r = c.post("/markers", json={"source": "wbc", "markers": [
        {"name": "wbc/arm/target", "kind": "frame", "frame": "world", "pos": [0.5, 0, 0.6], "quat": [0, 0, 0, 2],
         "color": "#e0a030", "label": "target"},
        {"name": "wbc/arm/cmd", "frame": "arm_end_link", "pos": [0, 0, 0]},
        {"name": "path", "kind": "line", "frame": "base", "points": [[0, 0, 0], [1, 0, 0]]}]})
    assert r.status_code == 200 and r.json() == {"names": ["wbc/arm/target", "wbc/arm/cmd", "path"], "count": 3}
    mk = {m["name"]: m for m in c.get("/markers").json()["markers"]}
    assert mk["wbc/arm/target"]["quat"] == [0, 0, 0, 1.0] and mk["wbc/arm/cmd"]["kind"] == "frame"
    assert mk["path"]["source"] == "wbc"
    # atomic: one bad marker refuses the whole post
    bad = c.post("/markers", json={"markers": [{"name": "ok", "pos": [0, 0, 0]}, {"name": "x", "frame": "nope", "pos": [0, 0, 0]}]})
    assert bad.status_code == 400 and "nope" in bad.json()["detail"]
    assert c.post("/markers", json={"markers": [{"name": "p", "kind": "point"}]}).status_code == 400   # no pos
    assert c.post("/markers", json={"markers": [{"name": "l", "kind": "line", "points": [[0, 0]]}]}).status_code == 400
    assert len(c.get("/markers").json()["markers"]) == 3
    # the frame shows them, then only when they changed
    f = node.viz_frame()
    assert len(f["markers"]) == 3 and "markers" not in node.viz_frame(f["markers_version"])
    # replace: this source's others go
    c.post("/markers", json={"source": "wbc", "replace": True, "markers": [{"name": "wbc/arm/target", "pos": [0, 0, 1]}]})
    assert [m["name"] for m in c.get("/markers").json()["markers"]] == ["wbc/arm/target"]
    c.post("/markers", json={"source": "other", "markers": [{"name": "o", "kind": "text", "pos": [0, 0, 1], "label": "hi"}]})
    assert c.delete("/markers/wbc/arm/target").json() == {"removed": 1}
    assert c.delete("/markers/wbc/arm/target").status_code == 404
    assert c.delete("/markers", params={"source": "other"}).json() == {"removed": 1}
    assert c.delete("/markers").json() == {"removed": 0}


def test_marker_ttl_expires():
    now = [100.0]
    m = viz.Markers(clock=lambda: now[0])
    m.put([{"name": "a", "pos": [0, 0, 0], "ttl": 0.5}, {"name": "b", "pos": [0, 0, 0]}], set())
    v = m.version
    assert {x["name"] for x in m.list()} == {"a", "b"}
    now[0] += 0.6
    assert [x["name"] for x in m.list()] == ["b"] and m.version > v
    with pytest.raises(ValueError):
        m.put([{"name": f"n{i}", "pos": [0, 0, 0]} for i in range(viz.MAX_MARKERS + 1)], set())
    assert [x["name"] for x in m.list()] == ["b"]                   # refused whole


def test_world_socket_resolution(tmp_path):
    from robot_utils.nodes.sim.backends import is_world, wait_for_world, world_socket
    from robot_utils.nodes.sim.protocol import backend_socket
    assert world_socket("world", "arm") == backend_socket("sim", "arm")
    assert world_socket("world=sim2", "camera-camera") == backend_socket("sim2", "camera-camera")
    assert world_socket(f"world={tmp_path}", "swerve") == tmp_path / "swerve.sock"
    assert is_world("world") and is_world("world=x") and not is_world("toy") and not is_world(None)
    with pytest.raises(ValueError):
        world_socket("toy", "arm")
    assert wait_for_world(tmp_path / "none.sock", timeout=0.3) is False
