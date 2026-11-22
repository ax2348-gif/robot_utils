"""The default tabletop scene (nodes/sim/scene.py; docs/sim.md "The default scene") and fixed objects: the table
is welded to the world, the graspable objects fit the gripper and rest on it, the arm reaches them pointing down
from the start pose (and the far one after driving sideways), and everything appears in /scene, the 3D view data,
robot_state.v1 and the camera renders."""
import math

import mujoco
import numpy as np
import pytest
from sim_helpers import make_world, needs_gl

from robot_utils.nodes.sim import viz
from robot_utils.nodes.sim.scene import tabletop
from robot_utils.nodes.sim.world import SimError

GRASP = np.array([-0.05, 0.0, 0.0])     # between the finger pads, in arm_end_link (its +x points out of the gripper)


@pytest.fixture(scope="module")
def w():
    return make_world(scene={"tabletop": True})


def reach_down(world, p, base_xy=(0.0, 0.0), tries=12, offset=GRASP, jaw=None, seed=None, pos_tol=0.004,
               ang_tol_deg=10.0, iters=300):
    """Is world point ``p`` reachable by the grasp point (``offset`` in arm_end_link) with the gripper pointing
    straight down, the base at ``base_xy`` (start yaw)? ``jaw``: a horizontal world direction the jaws must close
    along (end_link y, either sign). Accepted within ``pos_tol`` m and ``ang_tol_deg``. Damped least squares on the model's arm joints from ``seed`` (else the start
    pose), then random seeds. Returns the joint angles, or None."""
    m = world.m
    d = mujoco.MjData(m)
    d.qpos[:] = world.d.qpos
    d.qpos[world.base_q:world.base_q + 2] = base_xy
    e = m.body("arm_end_link").id
    ids = [m.joint(f"arm_joint{i}").id for i in range(1, 7)]
    lo, hi = m.jnt_range[ids, 0], m.jnt_range[ids, 1]
    dofs = world.arm_v
    rng = np.random.default_rng(0)
    for t in range(tries):
        q = (seed if seed is not None else world.cfg["components"]["arm"]["start_q"]) if t == 0 else rng.uniform(lo, hi)
        q = np.array(q, float)
        for _ in range(iters):
            d.qpos[world.arm_q] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            R = d.xmat[e].reshape(3, 3)
            g = d.xpos[e] + R @ offset
            ep = p - g
            er = np.cross(R[:, 0], [0.0, 0.0, -1.0])            # tool axis onto straight down
            if jaw is not None:                                  # jaw travel onto the given direction
                j = np.asarray(jaw, float) * (1.0 if R[:, 1] @ jaw >= 0 else -1.0)
                er = er + np.cross(R[:, 1], j)
            if np.linalg.norm(ep) < pos_tol and np.linalg.norm(er) < math.sin(math.radians(ang_tol_deg)):
                return q
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, jr, g, e)
            J = np.vstack([jp[:, dofs], 0.3 * jr[:, dofs]])
            err = np.concatenate([ep, 0.3 * er])
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), err), lo, hi)
    return None


def test_fixed_objects_stay_and_are_reported(w):
    o = {o["name"]: o for o in w.scene()["objects"]}
    assert o["table"]["fixed"] and "fixed" not in o["cube"]
    top0 = o["table"]["xyz"]
    w.apply_wrench("table", [500, 0, 0], [0, 0, 0], 0.2)
    w.step(300)
    assert w.object_state("table")["xyz"] == top0                   # welded: pushes do nothing
    w.move_object("table", [top0[0] + 0.1, 0, top0[2]])             # a privileged move still works
    assert w.object_state("table")["xyz"][0] == pytest.approx(top0[0] + 0.1)
    w.reset()                                                       # and reset puts it back
    assert w.object_state("table")["xyz"] == pytest.approx(top0)
    rs = viz.robot_state(w)["objects"]
    assert rs["table"]["fixed"] and not rs["cube"]["fixed"] and rs["table"]["geom"]["size"] == [0.6, 1.6, 0.03]
    assert rs["can"]["geom"]["type"] == "cylinder" and rs["can"]["geom"]["rgba"]
    info, _ = viz.model(w)
    names = {b["name"] for b in info["bodies"] if b["kind"] == "object"}
    assert {"table", "table_leg1", "cube", "can", "block", "cube_far", "crate", "bin"} <= names


def test_default_scene_geometry_rests_and_fits_the_gripper(w):
    from robot_utils.nodes.arm.config import GAP_MAX
    from robot_utils.nodes.swerve.constants import FRAME, TOP_Z
    st = {o["name"]: o for o in w.scene()["objects"]}
    table = st["table"]
    top = table["xyz"][2] + 0.015
    assert top == pytest.approx(TOP_Z + 0.08, abs=1e-6)            # 8 cm above the arm's mount
    assert table["xyz"][0] - 0.3 == pytest.approx(FRAME / 2 + 0.04)  # front edge 4 cm in front of the base
    w.step(1000)
    for name in ("cube", "can", "block", "cube_far", "cube_view"):
        o = w.object_state(name)
        h = o["size"][2] if o["shape"] == "box" else o["length"]
        assert o["xyz"][2] == pytest.approx(top + h / 2, abs=0.004), name     # resting on the table
        width = min(o["size"][:2]) if o["shape"] == "box" else 2 * o["radius"]
        assert width < GAP_MAX, name                                          # fits the open gripper
    assert w.object_state("crate")["xyz"][2] == pytest.approx(0.15, abs=0.004)
    w.reset()


def test_the_arm_reaches_the_table_objects_from_the_start_pose(w):
    st = {o["name"]: o for o in w.scene()["objects"]}

    def grasp_point(o):
        h = o["size"][2] if o["shape"] == "box" else o["length"]
        return np.array(o["xyz"]) + [0, 0, h / 2 - 0.015]            # 1.5 cm below the top
    for name in ("cube", "can", "block"):
        assert reach_down(w, grasp_point(st[name])) is not None, name
    far = grasp_point(st["cube_far"])
    assert reach_down(w, far) is None                                  # needs the base to move ...
    assert reach_down(w, far, base_xy=(0.0, 0.55)) is not None         # ... sideways along the table


def test_reset_to_the_start_scene_and_no_tabletop():
    w = make_world(scene={"tabletop": True})
    w.remove_object("cube")
    w.add_object({"name": "extra", "xyz": [2, 2, 0.05]})
    w.reset()
    names = {o["name"] for o in w.scene()["objects"]}
    assert "cube" not in names and "extra" in names                    # plain reset: the objects there are
    w.reset(start_scene=True)
    names = {o["name"] for o in w.scene()["objects"]}
    assert "cube" in names and "extra" not in names and names == {o["name"] for o in tabletop()}
    assert make_world().scene()["objects"] == []                       # tests: empty unless asked
    with pytest.raises(SimError):
        w.add_object({"name": "table", "fixed": True})                # names stay unique


@needs_gl
def test_cameras_see_the_table_and_objects():
    from robot_utils.nodes.sim.render import Cameras
    full = make_world(scene={"tabletop": True})
    empty = make_world()
    can = full.object_state("can")
    q = reach_down(full, np.array(can["xyz"]) + [0, 0, 0.12])          # the gripper 7 cm above the can, looking down
    assert q is not None
    for world in (full, empty):
        world.d.qpos[world.arm_q] = q
        mujoco.mj_forward(world.m, world.d)
    out = {}
    for key, world in (("full", full), ("empty", empty)):
        c = Cameras(world, world.cfg["cameras"])
        try:
            out[key] = {cam: c._render(c.cam(cam), "depth") for cam in ("camera", "wrist_camera")}
        finally:
            c.close()
    for cam in ("camera", "wrist_camera"):
        (mf, df), (me, de) = out["full"][cam], out["empty"][cam]
        a = np.frombuffer(df, "<u2").reshape(mf["height"], mf["width"]).astype(float)
        b = np.frombuffer(de, "<u2").reshape(me["height"], me["width"]).astype(float)
        nearer = (a > 0) & ((b == 0) | (a < b - 20))                   # the scene in front of the floor
        assert nearer.mean() > 0.05, (cam, nearer.mean())
