"""The rings scene (docs/sim.md, "The rings scene"): two counter-rotating driven rings with red and blue objects, and
the joint drive of articulated objects it is built on."""
import math

import numpy as np
import pytest
from fastapi.testclient import TestClient
from sim_helpers import make_world

from robot_utils.nodes.sim import scene as S


@pytest.fixture(scope="module")
def world():
    w = make_world(scene={"tabletop": False, "rings": True})
    yield w
    w.close()


def objs(world):
    return {o["name"]: o for o in world.scene()["objects"]}


def polar(xyz):
    (cx, cy), p = S.ring_geometry()["centre"], np.asarray(xyz[:2])
    return float(np.hypot(p[0] - cx, p[1] - cy)), math.atan2(p[1] - cy, p[0] - cx)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def test_layout_seeded_graspable_and_in_reach_of_the_start_pose():
    from robot_utils.nodes.arm.config import GAP_MAX
    from robot_utils.nodes.swerve.constants import FRAME
    a, b = S.ring_objects(seed=3), S.ring_objects(seed=3)
    assert a == b and a != S.ring_objects(seed=4)
    assert len(a) == 16 and sum(o["rgba"] == S.RED for o in a) == 8
    g = S.ring_geometry()
    for o in a:
        widths = o["size"][:2] if o["shape"] == "box" else [2 * o["radius"]] * 2
        assert max(widths) < 0.8 * GAP_MAX, o
        r, _ = polar(o["xyz"])
        band = g["outer"] if "_outer" in o["name"] else g["inner"]
        assert band[0] + 0.03 < r < band[1] - 0.03, o
    pts = np.array([o["xyz"][:2] for o in a])
    d = np.linalg.norm(pts[:, None] - pts[None], axis=-1) + np.eye(len(a))
    assert d.min() > 0.06                                       # none touching
    # the outer edge EDGE_GAP ahead of the base's front, on the arm mount's line
    assert g["centre"][0] - g["outer"][1] == pytest.approx(FRAME / 2 + S.EDGE_GAP)
    assert (g["outer"][1] - g["outer"][0], g["inner"][1] - g["inner"][0]) == pytest.approx((0.20, 0.30))


def test_drives_hold_their_speed_under_load_and_objects_ride_along(world):
    world.reset(start_scene=True)
    jo = world.obj_joints["rings"]
    start = {n: polar(o["xyz"]) for n, o in objs(world).items() if n != "rings"}
    q0 = {k: float(world.d.qpos[jo[k + "0"][0]]) for k in ("outer", "inner")}     # turned during the reset's settle
    world.step(1000)
    w_out, w_in = (float(world.d.qvel[jo[k + "0"][1]]) for k in ("outer", "inner"))
    assert w_out == pytest.approx(0.04 / 1.0, rel=0.01) and w_in == pytest.approx(-0.04 / 0.79, rel=0.01)
    # a 10 N m brake on the outer ring (an arm leaning on it): the 1000 N m s/rad servo gives up 0.01 rad/s
    b = world.m.body("obj_rings/outer5").id                 # on one sector: the rest help through the mimic
    for _ in range(500):
        world.d.xfrc_applied[b] = [0, 0, 0, 0, 0, -10.0]
        world.step(1)
    w_loaded = float(world.d.qvel[jo["outer0"][1]])
    world.d.xfrc_applied[b] = 0
    assert abs(w_loaded - 0.04) < 0.012
    world.step(1500)
    q = {k: float(world.d.qpos[jo[k + "0"][0]]) - q0[k] for k in ("outer", "inner")}
    for n, o in objs(world).items():
        if n == "rings":
            continue
        r, a = polar(o["xyz"])
        ring = "outer" if "_outer" in n else "inner"
        assert abs(r - start[n][0]) < 0.01, n                      # no slip outward
        assert abs(wrap(a - start[n][1] - q[ring])) < 0.01, n      # turned with its ring
        assert o["xyz"][2] > S.ring_geometry()["top"], n          # still on it


def test_object_pushed_off_the_inner_edge_falls_to_the_floor_and_reset_restores(world):
    world.reset(start_scene=True)
    start = objs(world)
    g = S.ring_geometry()
    (cx, cy), r_in = g["centre"], g["inner"][0]
    name = "red_inner1" if "red_inner1" in start else next(n for n in start if "_inner" in n)
    _, a = polar(start[name]["xyz"])
    world.move_object(name, [cx + (r_in + 0.03) * math.cos(a), cy + (r_in + 0.03) * math.sin(a),
                             start[name]["xyz"][2]])
    f = 15 * start[name]["mass"]                                 # N: a 1.5 g shove toward the centre, 0.12 s
    world.apply_wrench(name, [-f * math.cos(a), -f * math.sin(a), 0], [0, 0, 0], 0.12)
    world.step(1500)
    o = objs(world)[name]
    r, _ = polar(o["xyz"])
    assert r < r_in and o["xyz"][2] < 0.05, o                  # through the hollow, on the floor
    world.reset()
    after = objs(world)
    jo = world.obj_joints["rings"]
    assert after["rings"]["joints"]["outer0"] == pytest.approx(0.0, abs=0.02)    # back (0.3 s settle at 0.04 rad/s)
    for n, o in after.items():
        if n != "rings":
            assert np.allclose(o["xyz"][:2], start[n]["xyz"][:2], atol=0.02), n
    assert abs(float(world.d.qvel[jo["outer0"][1]]) - 0.04) < 1e-3          # and turning again


def test_drive_validation(world):
    from robot_utils.nodes.sim.world import SimError
    base = {"name": "a", "geoms": [{"shape": "box"}]}
    for drive in ({"velocity": 1.0}, {"max_torque": 1.0}, {"velocity": 1, "max_torque": 0},
                  {"velocity": float("nan"), "max_torque": 1}, {"velocity": 1, "max_torque": 1, "gain": -1}, 3.0):
        with pytest.raises(SimError):
            world.add_object({"name": "bad", "shape": "articulated", "links": [
                base, {"name": "b", "parent": "a", "joint": {"type": "hinge", "axis": [0, 0, 1], "drive": drive},
                       "geoms": [{"shape": "box"}]}]})
    for mimic in ("zz", "a", "b"):                    # unknown; the root (no joint); itself (not earlier)
        with pytest.raises(SimError):
            world.add_object({"name": "bad", "shape": "articulated", "links": [
                base, {"name": "b", "parent": "a", "joint": {"type": "hinge", "mimic": mimic},
                       "geoms": [{"shape": "box"}]}]})
    with pytest.raises(SimError):                     # another joint type
        world.add_object({"name": "bad", "shape": "articulated", "links": [
            base, {"name": "b", "parent": "a", "joint": {"type": "slide"}, "geoms": [{"shape": "box"}]},
            {"name": "c", "parent": "a", "joint": {"type": "hinge", "mimic": "b"}, "geoms": [{"shape": "box"}]}]})
    assert "bad" not in world.objects


def test_rings_are_dynamic_bodies_in_the_rigid_scene(world):
    from robot_utils.nodes.sim import rigid
    world.reset(start_scene=True)
    idx, _ = rigid.build(world)
    by = {b["name"]: b for b in idx["bodies"]}
    assert by["rings/outer0"]["dynamic"] and by["rings/inner7"]["dynamic"] and not by["rings"]["dynamic"]
    assert by["rings/outer0"]["joint"]["type"] == "hinge"


def test_http_drive_round_trips_through_save_and_load(world, tmp_path):
    from robot_utils.nodes.sim.node import SimNode
    from robot_utils.nodes.sim.server import create_app
    world.reset(start_scene=True)
    nu, neq = world.m.nu, world.m.neq
    node = SimNode(world, "rg", str(tmp_path / "scenes"))
    spin = {"name": "spin", "shape": "articulated", "fixed": True, "xyz": [-2, -2, 0],
            "links": [{"name": "base", "geoms": [{"shape": "box", "size": [0.1, 0.1, 0.1], "pos": [0, 0, 0.05]}]},
                      {"name": "top", "parent": "base", "pos": [0, 0, 0.12],
                       "joint": {"type": "hinge", "axis": [0, 0, 1], "drive": {"velocity": 0.5, "max_torque": 2.0}},
                       "geoms": [{"shape": "box", "size": [0.4, 0.05, 0.02]}]},
                      {"name": "top2", "parent": "base", "pos": [0, 0, 0.16],
                       "joint": {"type": "hinge", "axis": [0, 0, 1], "mimic": "top", "damping": 0.0},
                       "geoms": [{"shape": "box", "size": [0.05, 0.4, 0.02]}]}]}
    with TestClient(create_app(node, start=False)) as c:
        r = c.post("/scene/objects", json=spin)
        assert r.status_code == 200, r.text
        assert r.json()["links"][1]["joint"]["drive"] == {"velocity": 0.5, "max_torque": 2.0, "gain": 1000.0}
        bad = {**spin, "name": "spin2"}
        bad["links"] = [spin["links"][0], {**spin["links"][1], "joint": {"type": "hinge", "drive": {"velocity": 1}}}]
        assert c.post("/scene/objects", json=bad).status_code == 422
        c.post("/scene/save", json={"name": "spun"})
        assert c.post("/scene/load", json={"name": "spun"}).status_code == 200
        world.step(500)
        top, top2 = world.obj_joints["spin"]["top"], world.obj_joints["spin"]["top2"]
        assert float(world.d.qvel[top[1]]) == pytest.approx(0.5, rel=0.01)
        assert float(world.d.qpos[top2[0]]) == pytest.approx(float(world.d.qpos[top[0]]), abs=0.01)   # mimics it
        assert c.delete("/scene/objects/spin").status_code == 200       # its drive and mimic go with it
    assert "spin" not in world.objects and (world.m.nu, world.m.neq) == (nu, neq)
