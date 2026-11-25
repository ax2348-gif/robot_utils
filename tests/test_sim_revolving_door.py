"""The revolving door scene (docs/sim.md, "The revolving door scene"): a passive one-way rotor that spins down from its
start speed, the robot-sized compartments, the passage that takes most of a turn, and the joint fields it is built on
(``qvel0``, ``ratchet``)."""
import math

import numpy as np
import pytest
from fastapi.testclient import TestClient
from sim_helpers import make_world

from robot_utils.nodes.sim import scene as S

pytest.importorskip("scipy")


@pytest.fixture(scope="module")
def world():
    w = make_world(scene={"tabletop": False, "revolving_door": True})
    yield w
    w.close()


def rotor(world):
    return world.obj_joints["revolving_door"]["rotor"]          # (qpos adr, dof adr)


def robot_footprint(world) -> tuple:
    """(x0, x1, y0, y1): the robot's extent seen from above at its start pose (base at the origin facing +x, the arm
    at its start pose), from every robot geom's box."""
    m, d = world.m, world.d
    pts = []
    for g in range(m.ngeom):
        name = m.body(m.geom_bodyid[g]).name
        if name == "world" or name.startswith("obj_"):
            continue
        c, half = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + c
        pts.append(corners @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    p = np.concatenate(pts)
    return float(p[:, 0].min()), float(p[:, 0].max()), float(p[:, 1].min()), float(p[:, 1].max())


def test_preset_puts_the_door_ahead_of_the_robot_facing_it():
    import pathlib
    from robot_utils.nodes.sim.world import World, load_config
    from robot_utils.nodes.swerve.constants import FRAME
    path = pathlib.Path(__file__).parents[1] / "deploy/sim/sim_revolving_door_dynamic.yaml"
    cfg = load_config(str(path))
    assert cfg["components"]["arm"]["fidelity"] == "dynamic" and cfg["scene"]["view_anchor"] == [0.6, 0.0, 0.0]
    assert "timing" not in __import__("yaml").safe_load(path.read_text())          # the defaults, as the others
    objs = World.start_objects(type("W", (), {"cfg": cfg})())
    assert [o["name"] for o in objs] == ["revolving_door"]
    x, y, _ = objs[0]["xyz"]
    assert y == 0.0 and cfg["base"]["start"]["yaw"] == 0.0     # straight ahead of the robot, which faces it
    assert x - S.DOOR_R - S.DOOR_WALL_T - FRAME / 2 == pytest.approx(S.DOOR_GAP)    # the drum 0.6 m off its front
    g = S.door_geometry()
    a_e = (g["entrance"][0] + g["entrance"][1]) / 2
    assert math.cos(a_e) < 0 < math.sin(a_e)                   # the entrance is on the robot's side, front-left


def test_spin_lasts_over_a_minute_then_stops(world):
    world.reset()
    qa, va = rotor(world)
    assert world.t == 0.0 and float(world.d.qvel[va]) == pytest.approx(S.DOOR_SPIN)
    speeds = [float(world.d.qvel[va])]
    for _ in range(100):                                     # 100 s of physics, 1 s at a time
        world.step(1000)
        speeds.append(float(world.d.qvel[va]))
    speeds = np.array(speeds)
    assert speeds[60] > 0.05                                 # still turning a minute in
    stopped = int(np.argmax(np.abs(speeds) < 1e-3))
    assert 60 < stopped < 100, speeds.round(3)              # then stops (81 s measured)
    assert np.all(np.diff(speeds) <= 1e-6)                   # slowing all the way, never reversing
    assert np.all(np.abs(speeds[stopped:]) < 1e-3)
    assert float(world.d.qpos[qa]) == pytest.approx(13.46, abs=0.3)   # ~2.1 turns from the start


def test_reset_restores_the_spin(world):
    qa, va = rotor(world)
    world.step(3000)
    world.reset()
    assert world.t == 0.0
    assert float(world.d.qpos[qa]) == pytest.approx(0.0, abs=1e-9)
    assert float(world.d.qvel[va]) == pytest.approx(S.DOOR_SPIN)
    world.step(10_000)
    assert 0.29 < float(world.d.qvel[va]) < 0.31               # the same decay as from the start (0.301 at 10 s)
    world.reset(start_scene=True)                             # a reload of the start scene spins it too
    qa, va = rotor(world)
    assert float(world.d.qvel[va]) == pytest.approx(S.DOOR_SPIN)


def test_ratchet_holds_the_rotor_against_reverse_and_frees_it_forward(world):
    world.reset()
    qa, va = rotor(world)
    world.d.qvel[va] = 0.0                                    # at rest
    b = world.m.body("obj_revolving_door/rotor").id
    q0 = float(world.d.qpos[qa])
    for _ in range(2000):                                     # 150 N m backward (more than the robot pushes) for 2 s
        world.d.xfrc_applied[b] = [0, 0, 0, 0, 0, -150.0]
        world.step(1)
    assert float(world.d.qpos[qa]) - q0 > -0.005              # 1.5 mrad/s creep at most
    for _ in range(2000):                                     # 20 N m forward: it turns
        world.d.xfrc_applied[b] = [0, 0, 0, 0, 0, 20.0]
        world.step(1)
    world.d.xfrc_applied[b] = 0
    # 5 wings: (20 - 0.75 N m friction) / ~205 kg m^2 = 0.094 rad/s^2, less the damping: ~0.19 rad in 2 s
    assert 0.14 < float(world.d.qpos[qa]) - q0 < 0.23


def test_compartments_fit_the_robot_with_margins(world):
    """The robot (base and the arm at its start pose) in a compartment, front outward, centred: every side clears the
    wings and the drum by >= 5 cm; a doorway is >= 10 cm wider than it each side."""
    from door_sweep import _dist, _parts
    x0, x1, y0, y1 = robot_footprint(world)
    assert (x1 - x0, y1 - y0) == pytest.approx((0.576, 0.454), abs=0.005)
    door = S.revolving_door()
    links = {ln["name"]: ln for ln in door["links"]}
    g = S.door_geometry()
    beta = -math.pi / 2                                       # a compartment on the closed side, away from the doorways
    q = beta - (g["entrance"][0] + g["entrance"][1]) / 2      # turns the entrance compartment there
    u = np.linspace(0, 1, 60)
    edge = np.concatenate([np.stack([x0 + (x1 - x0) * u, np.full_like(u, y)], 1) for y in (y0, y1)]
                          + [np.stack([np.full_like(u, x), y0 + (y1 - y0) * u], 1) for x in (x0, x1)])
    best = 0.0
    for rho in np.arange(0.5, 1.3, 0.005):                   # the base's centre along the compartment's middle
        local = edge - [0.0, (y0 + y1) / 2]
        c, s = math.cos(beta), math.sin(beta)
        pts = local @ np.array([[c, s], [-s, c]]) + rho * np.array([c, s])
        clear = min(_dist(_parts(links["drum"]), pts).min(), _dist(_parts(links["rotor"]), pts, q).min())
        best = max(best, clear)
    assert best >= 0.05, best                                 # 8.3 cm measured
    assert (S.DOOR_DOORWAY - (y1 - y0)) / 2 >= 0.10           # 12.3 cm


def test_passage_takes_most_of_a_turn():
    """A body the size of the base's inscribed disc (anything the robot does, it does too) cannot get from the near
    side to the far side unless the rotor turns >= 210 deg (5 wings); without the ratchet it could go back the short way."""
    from door_sweep import sweep
    from robot_utils.nodes.swerve.constants import FRAME
    door = S.revolving_door()
    ccw = sweep(door, FRAME / 2, step_deg=3)
    turns = [t for t in ccw["turns"] if t is not None]        # None: no compartment open to the entrance at q0
    assert len(turns) >= 0.8 * len(ccw["turns"]) and math.inf not in turns     # in at most q0, and out again
    assert ccw["min"] >= 210 and max(turns) <= 330, ccw       # 5 wings: 232-292 deg measured (3 deg steps)
    cw = sweep(door, FRAME / 2, step_deg=3, reverse=True)
    assert cw["min"] < 30, cw                                 # why the ratchet: 4 deg the other way


def test_body_pushed_at_the_exit_from_the_entrance_compartment_stays_in(world):
    """Physics: a robot-sized box (0.43 m, 20 kg, slippery) shoved from just inside the entrance straight at the exit
    for 3 s, the rotor at rest: the wings and the ratchet keep it in the drum."""
    world.reset()
    qa, va = rotor(world)
    world.d.qvel[va] = 0.0
    g = S.door_geometry()
    (cx, cy, _), a_e = S.revolving_door()["xyz"], (g["entrance"][0] + g["entrance"][1]) / 2
    a_x = (g["exit"][0] + g["exit"][1]) / 2
    start = [cx + 0.95 * S.DOOR_R * math.cos(a_e), cy + 0.95 * S.DOOR_R * math.sin(a_e), 0.16]
    world.add_object({"name": "pusher", "shape": "box", "size": [0.43, 0.43, 0.3], "mass": 20.0, "xyz": start,
                      "yaw": a_e, "friction": [0.1, 0.005, 0.0001]})
    try:
        qa, va = rotor(world)
        q0 = float(world.d.qpos[qa])
        target = np.array([cx + 2.0 * S.DOOR_R * math.cos(a_x), cy + 2.0 * S.DOOR_R * math.sin(a_x)])
        for _ in range(3000):
            p = np.array(world.object_state("pusher")["xyz"][:2])
            f = 150.0 * (target - p) / np.linalg.norm(target - p)
            world.apply_wrench("pusher", [f[0], f[1], 0.0], [0, 0, 0], 0.002)
            world.step(1)
        p = np.array(world.object_state("pusher")["xyz"][:2])
        assert np.hypot(*(p - [cx, cy])) < S.DOOR_R, p        # still in the drum
        assert float(world.d.qpos[qa]) - q0 > -0.01           # the rotor did not turn back
    finally:
        world.remove_object("pusher")


def test_qvel0_and_ratchet_validation(world):
    from robot_utils.nodes.sim.world import SimError
    base = {"name": "a", "geoms": [{"shape": "box"}]}

    def obj(**joint):
        return {"name": "bad", "shape": "articulated", "links": [
            base, {"name": "b", "parent": "a", "joint": {"type": "hinge", "axis": [0, 0, 1], **joint},
                   "geoms": [{"shape": "box"}]}]}

    for v in ("fast", float("nan"), float("inf"), True, [1.0]):
        with pytest.raises(SimError, match="qvel0"):
            world.add_object(obj(qvel0=v))
    for rt in ({"direction": 0, "max_torque": 1}, {"direction": 2, "max_torque": 1}, {"direction": 1},
               {"direction": 1, "max_torque": 0}, {"direction": 1, "max_torque": 1, "gain": -1},
               {"direction": True, "max_torque": 1}, 1):
        with pytest.raises(SimError, match="ratchet"):
            world.add_object(obj(ratchet=rt))
    with pytest.raises(SimError, match="not both"):
        world.add_object(obj(ratchet={"direction": 1, "max_torque": 1},
                             drive={"velocity": 1.0, "max_torque": 1.0}))
    assert "bad" not in world.objects


def test_rotor_is_a_dynamic_body_in_the_rigid_scene(world):
    from robot_utils.nodes.sim import rigid
    world.reset()
    idx, _ = rigid.build(world)
    by = {b["name"]: b for b in idx["bodies"]}
    assert by["revolving_door/rotor"]["dynamic"] and not by["revolving_door"]["dynamic"]
    assert by["revolving_door/rotor"]["joint"]["type"] == "hinge"
    assert len(by["revolving_door/rotor"]["geoms"]) == S.DOOR_WINGS + 1       # the wings and the post
    assert by["revolving_door/rotor"]["geoms"][1]["rgba"][3] < 1              # glass: see-through


def test_http_qvel0_and_ratchet_round_trip_through_save_and_load(world, tmp_path):
    from robot_utils.nodes.sim.node import SimNode
    from robot_utils.nodes.sim.server import create_app
    world.reset()
    nu = world.m.nu
    node = SimNode(world, "rd", str(tmp_path / "scenes"))
    top = {"name": "top", "parent": "base", "pos": [0, 0, 0.12],
           "joint": {"type": "hinge", "axis": [0, 0, 1], "damping": 0.0, "qvel0": 0.7,
                     "ratchet": {"direction": 1, "max_torque": 50.0}},
           "geoms": [{"shape": "box", "size": [0.4, 0.05, 0.02]}]}
    spin = {"name": "spin", "shape": "articulated", "fixed": True, "xyz": [-2, -2, 0],
            "links": [{"name": "base", "geoms": [{"shape": "box", "size": [0.1, 0.1, 0.1], "pos": [0, 0, 0.05]}]},
                      top]}
    with TestClient(create_app(node, start=False)) as c:
        r = c.post("/scene/objects", json=spin)
        assert r.status_code == 200, r.text
        j = r.json()["links"][1]["joint"]
        assert j["qvel0"] == 0.7 and j["ratchet"] == {"direction": 1, "max_torque": 50.0, "gain": 1e5}
        assert float(world.d.qvel[world.obj_joints["spin"]["top"][1]]) == pytest.approx(0.7)   # spinning on load
        for bad in ({"qvel0": "fast"}, {"ratchet": {"direction": 2, "max_torque": 1}}):
            obj = {**spin, "name": "spin2", "links": [spin["links"][0], {**top, "joint": {**top["joint"], **bad}}]}
            assert c.post("/scene/objects", json=obj).status_code == 422
        assert c.post("/scene/save", json={"name": "spun"}).status_code == 200
        saved = (tmp_path / "scenes" / "spun.yaml").read_text()
        assert "qvel0: 0.7" in saved and "ratchet:" in saved
        world.step(500)
        assert c.post("/scene/load", json={"name": "spun"}).status_code == 200
        top_j = world.obj_joints["spin"]["top"]
        assert float(world.d.qvel[top_j[1]]) == pytest.approx(0.7)              # loaded: spinning again
        assert c.post("/scene/reset", json={}).status_code == 200
        assert float(world.d.qvel[world.obj_joints["spin"]["top"][1]]) == pytest.approx(0.7)   # and after a reset
        b = world.m.body("obj_spin/top").id
        for _ in range(300):                                   # its ratchet holds against reverse
            world.d.xfrc_applied[b] = [0, 0, 0, 0, 0, -5.0]
            world.step(1)
        world.d.xfrc_applied[b] = 0
        assert float(world.d.qvel[world.obj_joints["spin"]["top"][1]]) > -1e-3
        assert c.delete("/scene/objects/spin").status_code == 200      # its ratchet goes with it
    assert "spin" not in world.objects and world.m.nu == nu
