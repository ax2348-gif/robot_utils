"""Rigid scenes (docs/scene3d.md): the rigid_scene.v1 format (scene3d/format.py), the adapter for today's Quest AR view
(scene3d/compat.py), the sim's MuJoCo exporter (nodes/sim/rigid.py) and its HTTP endpoints, and articulated scene
objects (the cabinet)."""
import json

import numpy as np
import pytest
from fastapi.testclient import TestClient
from sim_helpers import make_world

from robot_utils.scene3d import Frame, SceneBuilder, compat, decode_frame, encode_frame, outline_edges
from robot_utils.scene3d.format import frame_bytes, mesh_arrays


def cube_mesh():
    v = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    f = []
    for ax in range(3):
        for s in (-1, 1):
            q = [i for i in range(8) if v[i, ax] == s]
            a, b, c, d = q[0], q[1], q[3], q[2]          # a quad, as two triangles
            f += [[a, b, c], [a, c, d]]
    return v, np.array(f)


# -- format ----------------------------------------------------------------------------------------------------

def test_outline_edges_keep_creases_and_boundaries_not_flat_diagonals():
    v, f = cube_mesh()
    e = outline_edges(v, f, 30)
    assert len(e) == 12                                  # the cube's edges, not its 6 face diagonals
    assert all(np.sum(np.abs(v[a] - v[b]) > 0) == 1 for a, b in e)
    assert len(outline_edges(v, f[:2], 30)) == 4         # one face alone: its boundary
    assert len(outline_edges(v, f, 100)) == 0            # 90 deg creases under a 100 deg threshold


def test_scene_builder_blob_round_trips_and_ids_follow_content():
    v, f = cube_mesh()

    def make(rgba):
        sb = SceneBuilder("test")
        mi = sb.add_mesh("cube", v * 0.1, f, error=0.001)
        b = sb.add_body("a", "object", [0, 0, 0, 0, 0, 0, 1], True)
        sb.add_geom(b, "mesh", mesh=mi, rgba=rgba)
        sb.add_body("floor", "world", [0, 0, 0, 0, 0, 0, 1], False,
                    geoms=[{"type": "box", "half_extents": [1, 1, 0.01], "role": "collision"}])
        return sb.build()
    idx, blob = make([1, 0, 0, 1])
    assert idx["dynamic"] == [0] and idx["meshes"][0]["nedge"] == 12 and idx["meshes"][0]["error"] == 0.001
    mv, mt, me = mesh_arrays(idx, blob, 0)
    assert np.allclose(mv, v * 0.1) and (mt == f).all() and len(me) == 12
    assert idx["meshes"][0]["bounds"] == [[-0.1] * 3, [0.1] * 3]
    assert make([1, 0, 0, 1])[0]["scene_id"] == idx["scene_id"] != make([0, 1, 0, 1])[0]["scene_id"]
    with pytest.raises(ValueError):
        SceneBuilder("t").add_body("x", "object", [0] * 7, True, geoms=[{"type": "box"}])


def test_frames_encode_compactly_and_decode():
    p = np.random.default_rng(0).normal(size=(30, 7))
    f = Frame(7, 12.5, 3, "0123456789abcdef", p)
    data = encode_frame(f) + encode_frame(Frame(8, 12.6, 3, "0123456789abcdef", p[:2], {1: [1, 0.5, 0, 0.4]}))
    assert len(data) == frame_bytes(30) + frame_bytes(2, 1) and frame_bytes(30) == 872
    g, used = decode_frame(data)
    assert (g.seq, g.t, g.reset_seq, g.scene_id, g.colors) == (7, 12.5, 3, "0123456789abcdef", {})
    assert np.allclose(g.poses, p, atol=1e-5) and used == frame_bytes(30)
    h, _ = decode_frame(data[used:])
    assert h.poses.shape == (2, 7) and list(h.colors) == [1]
    assert np.allclose(h.colors[1], [1, 0.5, 0, 0.4], atol=1 / 255)
    assert h.to_json()["colors"] == {"1": [1.0, 0.502, 0.0, 0.4]}
    with pytest.raises(ValueError):
        decode_frame(data[:40])


# -- compat ----------------------------------------------------------------------------------------------------

def test_compat_lines_put_every_body_as_primitives_in_its_frame():
    v, f = cube_mesh()
    sb = SceneBuilder("t")
    mi = sb.add_mesh("m", v * [0.1, 0.2, 0.3] + [1, 0, 0], f)
    b = sb.add_body("arm", "robot", [0] * 6 + [1], True)
    sb.add_geom(b, "mesh", mesh=mi, role="visual")
    sb.add_geom(b, "capsule", radius=0.05, half_length=0.2, role="collision", pos=[0, 0, 1])
    sb.add_body("table", "object", [0.5, 0, 0.3, 0, 0, 0, 1], False,
                geoms=[{"type": "box", "half_extents": [0.3, 0.8, 0.015], "role": "both"}])
    sb.add_body("floor", "world", [0] * 6 + [1], False, geoms=[{"type": "plane", "half_extents": [0, 0]}])
    idx, _ = sb.build()
    m = compat.model(idx)
    assert m["root"] == "world" and m["links"]["world"]["visuals"] == [] and m["model_version"] == idx["scene_id"]
    pose = np.array([[1, 2, 0, 0, 0, 0, 1.0]])
    line = compat.StateLines(idx).line(Frame(1, 0.5, 0, idx["scene_id"], pose), base=pose[0], overlays={"x": 1})
    o = line["objects"]
    assert set(o) == {"arm", "table"}                    # the plane is left out
    assert o["arm"]["pose"] == [1, 2, 0, 0, 0, 0, 1] and o["table"]["pose"] == [0.5, 0, 0.3, 0, 0, 0, 1]
    (cap,) = o["arm"]["geom"]                            # collision role by default
    assert cap["type"] == "capsule" and cap["length"] == 0.4 and cap["origin"]["xyz"] == [0, 0, 1]
    (box,) = compat.StateLines(idx, "visual").line(Frame(1, 0, 0, "", pose))["objects"]["arm"]["geom"]
    assert box["half_size"] == [0.1, 0.2, 0.3] and box["origin"]["xyz"] == [1, 0, 0]   # the mesh's bounds
    assert line["base"] == [1, 2, 0, 0, 0, 0, 1] and line["overlays"] == {"x": 1} and line["model_version"]


def test_compat_colour_overrides_recolour_and_hide():
    sb = SceneBuilder("t")
    sb.add_body("a", "ghost", [0] * 6 + [1], True, geoms=[{"type": "sphere", "radius": 0.1, "rgba": [1, 1, 1, 1]}])
    sb.add_body("b", "ghost", [0] * 6 + [1], True, geoms=[{"type": "sphere", "radius": 0.1}])
    idx, _ = sb.build()
    pose = np.tile([0, 0, 0, 0, 0, 0, 1.0], (2, 1))
    o = compat.StateLines(idx).line(Frame(1, 0, 0, "", pose, {0: [1, 0, 0, 0.5], 1: [0, 0, 0, 0]}))["objects"]
    assert list(o) == ["a"] and o["a"]["geom"][0]["rgba"] == [1, 0, 0, 0.5]


def test_compose_matches_rotation_then_translation():
    q = [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]     # 90 deg about z
    p = compat.compose([1, 0, 0, *q], [1, 0, 0, 0, 0, 0, 1])
    assert np.allclose(p[:3], [1, 1, 0]) and np.allclose(p[3:], q)


# -- the sim -------------------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def world():
    w = make_world(scene={"tabletop": True, "cabinet": True})
    yield w
    w.close()


def test_sim_scene_classifies_static_and_dynamic_bodies_and_poses_match(world):
    from robot_utils.nodes.sim import rigid
    idx, blob = rigid.build(world)
    by = {b["name"]: b for b in idx["bodies"]}
    assert not by["world"]["dynamic"] and not by["table"]["dynamic"] and not by["cabinet"]["dynamic"]
    for n in ("base_link", "cube", "cabinet/drawer_high", "cabinet/door"):
        assert by[n]["dynamic"], n
    assert by["cabinet/drawer_high"]["joint"]["type"] == "slide"
    assert by["cabinet/door"]["joint"] == {"name": "obj_cabinet/door", "type": "hinge", "range": [0.0, 1.9]}
    roles = {g["role"] for g in by["cube"]["geoms"]}
    assert roles == {"both"}                              # an object: what is drawn is what collides
    hulls = [m for m in idx["meshes"] if m["name"].endswith(".hull")]
    assert hulls and all(m["error"] == 0 for m in hulls)  # colliding meshes as MuJoCo's convex hulls
    assert all(m["error"] > 0 for m in idx["meshes"] if not m["name"].endswith(".hull"))
    assert len(blob) == idx["blob_bytes"] and rigid.build(world)[0] is idx           # cached
    f = rigid.Poses(idx).frame(world)
    i = idx["dynamic"].index(by["cube"]["id"])
    b = world.m.body("obj_cube").id
    assert np.allclose(f.poses[i, :3], world.d.xpos[b]) and np.allclose(f.poses[i, 6], world.d.xquat[b][0])
    world.move_object("table", [0.7, 0, 0.287])           # a static body moved: a new scene
    assert rigid.build(world)[0]["scene_id"] != idx["scene_id"]


def test_cabinet_drawer_slides_door_swings_and_reset_closes_them(world):
    world.reset(start_scene=True)
    joints = lambda: next(o for o in world.scene()["objects"] if o["name"] == "cabinet")["joints"]  # noqa: E731
    assert joints() == {"drawer_low": 0.0, "drawer_high": 0.0, "door": 0.0}
    world.step(300)
    assert max(abs(v) for v in joints().values()) < 1e-3          # it stays shut on its own
    cube0 = next(o for o in world.scene()["objects"] if o["name"] == "drawer_cube")["xyz"]
    dr, door = world.m.body("obj_cabinet/drawer_high").id, world.m.body("obj_cabinet/door").id
    for _ in range(600):
        world.d.xfrc_applied[dr] = [-6, 0, 0, 0, 0, 0]
        world.d.xfrc_applied[door] = [0, 0, 0, 0, 0, -0.8]       # about -z: the door's opening direction
        world.step(1)
    world.d.xfrc_applied[:] = 0
    world.step(200)
    q = joints()
    assert 0.1 < q["drawer_high"] <= 0.28 + 1e-3 and q["door"] > 0.5 and abs(q["drawer_low"]) < 1e-3
    cube = next(o for o in world.scene()["objects"] if o["name"] == "drawer_cube")["xyz"]
    assert cube[0] < cube0[0] - 0.08 and abs(cube[2] - cube0[2]) < 0.01      # carried out by the drawer
    world.reset()
    assert joints() == {"drawer_low": 0.0, "drawer_high": 0.0, "door": 0.0}


def test_articulated_object_validation(world):
    from robot_utils.nodes.sim.world import SimError
    bad = [
        [],
        [{"name": "a", "joint": {"type": "slide"}, "geoms": [{"shape": "box"}]}],      # a root with a joint
        [{"name": "a", "geoms": [{"shape": "box"}]}, {"name": "b", "parent": "zz", "geoms": [{"shape": "box"}]}],
        [{"name": "a", "geoms": []}],
        [{"name": "a", "geoms": [{"shape": "box"}]},
         {"name": "b", "parent": "a", "joint": {"type": "ball"}, "geoms": [{"shape": "box"}]}],
    ]
    for links in bad:
        with pytest.raises(SimError):
            world.add_object({"name": "bad", "shape": "articulated", "links": links})


def test_ghosts_copy_a_subtree_at_its_goal_and_hide_when_stale(world):
    from robot_utils.nodes.sim import rigid
    from robot_utils.nodes.sim.viz import Ghosts
    from robot_utils.scene3d.pose import compose
    t = [0.0]
    gh = Ghosts(clock=lambda: t[0])
    names = {world.m.body(i).name for i in range(world.m.nbody)}
    with pytest.raises(ValueError):
        gh.put([{"name": "g", "body": "nope", "relative": [0] * 6 + [1]}], names)
    with pytest.raises(ValueError):
        gh.put([{"name": "g", "body": "arm_end_link"}], names)          # no pose
    delta = [0.05, 0, 0, 0, 0, np.sin(0.2), np.cos(0.2)]               # 5 cm along the link's x, turned 0.4 rad
    gh.put([{"name": "goal", "body": "arm_end_link", "relative": delta, "rgba": [1, 0.8, 0, 0.4], "ttl": 0.5}],
           names)
    idx, _ = rigid.build(world, ghosts=gh.declared())
    (g,) = idx["ghosts"]
    assert len(g["bodies"]) >= 3                                       # the end link and the fingers
    kinds = {idx["bodies"][i]["kind"] for i in g["bodies"]}
    assert kinds == {"ghost"} and all(i in idx["dynamic"] for i in g["bodies"])
    poses = rigid.Poses(idx)
    f = poses.frame(world, gh.live())
    m, d = world.m, world.d
    x = lambda b: np.concatenate([d.xpos[b], d.xquat[b][[1, 2, 3, 0]]])  # noqa: E731
    root = x(g["sources"][0])
    for gid, src in zip(g["bodies"], g["sources"]):
        slot = idx["dynamic"].index(gid)
        # root * delta * inv(root) * link: inv(root) * link = (R^T (p - p_root), q_root^-1 q)
        exp = compose(compose(root, delta), compose(np.concatenate([[0, 0, 0], root[3:] * [-1, -1, -1, 1]]),
                                                    np.concatenate([x(src)[:3] - root[:3], x(src)[3:]])))
        assert np.allclose(f.poses[slot][:3], exp[:3], atol=1e-6), m.body(src).name
        assert f.colors[slot] == [1, 0.8, 0, 0.4]
    t[0] = 1.0                                                         # not posted again within its ttl: hidden
    f = poses.frame(world, gh.live())
    assert all(f.colors[idx["dynamic"].index(i)][3] == 0 for i in g["bodies"])
    assert rigid.build(world)[0]["scene_id"] != idx["scene_id"]       # without the ghost: another scene


# -- HTTP ------------------------------------------------------------------------------------------------------

@pytest.fixture
def api(world, tmp_path):
    from robot_utils.nodes.sim.node import SimNode
    from robot_utils.nodes.sim.server import create_app
    world.reset(start_scene=True)
    node = SimNode(world, "s3", str(tmp_path / "scenes"))
    with TestClient(create_app(node, start=False)) as c:
        yield c, node


def read_stream(c, path, n, binary):
    out, buf = [], b""
    with c.stream("GET", path) as r:
        assert r.status_code == 200
        for chunk in r.iter_bytes():
            buf += chunk
            if binary:
                while True:
                    try:
                        f, used = decode_frame(buf)
                    except ValueError:
                        break
                    out.append(f)
                    buf = buf[used:]
            else:
                *lines, buf = buf.split(b"\n")
                out += [json.loads(x) for x in lines if x]
            if len(out) >= n:
                return out


def test_http_scene_blob_frames_and_compat(api):
    c, node = api
    idx = c.get("/scene3d").json()
    assert idx["schema"] == "rigid_scene.v1" and idx["bodies"][0]["name"] == "world"
    blob = c.get("/scene3d/blob", params={"scene_id": idx["scene_id"]})
    assert blob.status_code == 200 and len(blob.content) == idx["blob_bytes"]
    assert c.get("/scene3d/blob", params={"scene_id": "0" * 16}).status_code == 409
    f = c.get("/scene3d/frame").json()
    assert f["scene_id"] == idx["scene_id"] and len(f["poses"]) == len(idx["dynamic"])
    frames = read_stream(c, "/scene3d/stream?hz=100", 2, True)
    assert frames[1].seq == frames[0].seq + 1 and frames[0].poses.shape == (len(idx["dynamic"]), 7)
    assert frames[0].scene_id == idx["scene_id"]
    jl = read_stream(c, "/scene3d/stream?hz=100&format=jsonl", 1, False)
    assert len(jl[0]["poses"]) == len(idx["dynamic"])
    m = c.get("/scene3d/compat/model").json()
    assert m["model_version"] == idx["scene_id"] and m["root"] == "world"
    (line,) = read_stream(c, "/scene3d/compat/stream?hz=100", 1, False)
    assert line["model_version"] == idx["scene_id"] and len(line["base"]) == 7
    assert {"cabinet/drawer_high", "table", "base_link", "cube"} <= set(line["objects"])
    assert "markers" in line["overlays"]


def test_http_ghosts_in_scene_frames_compat_and_robot_state(api):
    c, node = api
    assert c.post("/ghosts", json={"ghosts": [{"name": "g", "body": "nope", "relative": [0] * 6 + [1]}]}).status_code == 400
    r = c.post("/ghosts", json={"source": "t", "ghosts": [{"name": "goal", "body": "arm_end_link",
                                                          "relative": [0.02, 0, 0, 0, 0, 0, 1], "rgba": [1, 0, 0, 0.3],
                                                          "ttl": 30}]})
    assert r.json() == {"names": ["goal"], "count": 1}
    idx = c.get("/scene3d").json()
    gb = idx["ghosts"][0]["bodies"]
    assert all(idx["bodies"][i]["kind"] == "ghost" for i in gb)
    f = c.get("/scene3d/frame").json()
    slot = idx["dynamic"].index(gb[0])
    assert f["colors"][str(slot)] == [1, 0, 0, 0.3]
    (fb,) = read_stream(c, "/scene3d/stream?hz=100", 1, True)
    assert np.allclose(fb.colors[slot], [1, 0, 0, 0.3], atol=1 / 255)
    (line,) = read_stream(c, "/scene3d/compat/stream?hz=100", 1, False)
    ghost = [k for k in line["objects"] if k.startswith("ghost:goal/")]
    assert ghost and line["objects"][ghost[0]]["geom"][0]["rgba"] == [1, 0, 0, 0.3]
    rs = c.get("/robot_state").json()
    assert set(ghost) <= set(rs["objects"]) and rs["objects"][ghost[0]]["fixed"] is False
    assert c.get("/ghosts").json()["ghosts"][0]["live"] is True
    assert c.delete("/ghosts", params={"source": "t"}).json() == {"removed": 1}
    assert not any(k.startswith("ghost:") for k in c.get("/robot_state").json()["objects"])


def test_http_articulated_object_and_robot_state_links(api):
    c, node = api
    r = c.post("/scene/objects", json={"name": "box2", "shape": "articulated", "fixed": True, "xyz": [2, 2, 0],
                                       "links": [{"name": "base", "geoms": [{"shape": "box", "size": [0.4, 0.4, 0.1],
                                                                             "pos": [0, 0, 0.05]}]},
                                                 {"name": "lid", "parent": "base", "pos": [0, -0.2, 0.1],
                                                  "joint": {"type": "hinge", "axis": [1, 0, 0], "range": [0, 1.5]},
                                                  "geoms": [{"shape": "box", "size": [0.4, 0.4, 0.02],
                                                             "pos": [0, 0.2, 0.01]}]}]})
    assert r.status_code == 200, r.text
    assert r.json()["joints"] == {"lid": 0.0} and r.json()["links"][1]["joint"]["type"] == "hinge"
    rs = c.get("/robot_state").json()
    lid = rs["objects"]["box2/lid"]
    assert isinstance(lid["geom"], list) and lid["geom"][0]["origin"]["xyz"] == [0, 0.2, 0.01]
    assert rs["objects"]["box2/base"]["fixed"] and not lid["fixed"]
    saved = c.post("/scene/save", json={"name": "lidbox"}).json()
    assert "joints" not in next(o for o in saved["objects"] if o["name"] == "box2")
    assert c.post("/scene/load", json={"name": "lidbox"}).status_code == 200
    bad = c.post("/scene/objects", json={"name": "x", "shape": "articulated",
                                         "links": [{"name": "a", "geoms": [{"shape": "sphere"}]}]})
    assert bad.status_code == 422


def test_http_headset_view_setting_cycles_and_validates(api):
    c, node = api
    assert c.get("/scene3d/view").json() == {"style": "hidden_line", "role": "auto"}
    assert c.post("/scene3d/view/next").json()["style"] == "wire"
    assert c.post("/scene3d/view", json={"role": "collision"}).json() == {"style": "wire", "role": "collision"}
    assert c.post("/scene3d/view", json={"style": "sparkly"}).status_code == 422
    assert c.get("/state").json()["view"] == {"style": "wire", "role": "collision"}
    for _ in range(3):
        c.post("/scene3d/view/next")
    assert c.get("/scene3d/view").json()["style"] == "hidden_line"
    node.set_view("hidden_line", "auto")
