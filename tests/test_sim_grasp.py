"""Grasping in the sim world (docs/sim.md, "Grasping"): top-down grasps of the default scene's cube, can and block
from the start pose, in every model variant. The jaws collide as the jaw boxes (not the CAD hull), the grasp height
keeps the fingertips off the table, the jaws close along the object's narrow side; each object is lifted 10 cm and
held 2 s. The arm's tracking (did the gripper reach the lift pose) is checked apart from the object's slip in the
jaws (measured in the gripper's frame)."""
import numpy as np
import pytest
from sim_helpers import make_world
from test_sim_scene import reach_down

from robot_utils.nodes.arm.geometry import ARM_BOXES
from robot_utils.nodes.sim.world import GRIPPER_OPEN_RAD

# The jaw geometry (arm/geometry.py, end_link frame, x out of the gripper, origin at the fingertips): the rail is
# the part of the gripper an object's top would hit first when the tips go down beside it.
RAIL = next(b for b in ARM_BOXES if b[0] == "end" and b[5] > 150)      # the gripper's linear rail, full width
RAIL_FACE = -(RAIL[1] + RAIL[4] / 2) / 1000                             # m behind the tips (~73 mm)
CLEAR = 0.006                                                           # m, fingertips above the table
APPROACH, LIFT, HOLD = 0.08, 0.10, 2.0


def grasp(w, obj: str) -> dict:
    m, d, a = w.m, w.d, w.arm
    bid, e = m.body("obj_" + obj).id, m.body("arm_end_link").id
    top = w.object_state("table")["xyz"][2] + w.objects["table"]["size"][2] / 2
    o = w.object_state(obj)
    h = o["size"][2] if o["shape"] == "box" else o["length"]
    tip = np.array(o["xyz"][:2] + [max(top + CLEAR, top + h - RAIL_FACE + CLEAR)])    # tips off the table, rail
    yaw = o["rpy"][2]                                                                 # above the object's top
    jaw = None                                         # a box: close along its narrow side; a cylinder: any
    if o["shape"] == "box":
        jaw = [np.cos(yaw), np.sin(yaw), 0.0] if o["size"][0] <= o["size"][1] else [-np.sin(yaw), np.cos(yaw), 0.0]
    kw = dict(offset=np.zeros(3), pos_tol=0.001, iters=800)          # the fingertips (end_link origin)
    down, q = [], None                                 # straight down in 1 cm steps (IK warm-started)
    for k in range(int(round(APPROACH / 0.01)) + 1):
        q = reach_down(w, tip + [0, 0, APPROACH - 0.01 * k], jaw=jaw, seed=q, ang_tol_deg=1.5,
                       tries=1 if q is not None else 12, **kw)
        assert q is not None, (obj, "approach", k)
        down.append(q)
    up = []                                            # up 10 cm (the wrist may tilt: the top-down reach ends
    for k in range(1, int(round(LIFT / 0.01)) + 1):     # ~10 cm above the table)
        q = reach_down(w, tip + [0, 0, 0.01 * k], seed=q, ang_tol_deg=35, tries=3, **kw)
        assert q is not None, (obj, "lift", k)
        up.append(q)
    a.enabled = a.g_enabled = True

    def run(q, g, s):
        for _ in range(int(round(s / m.opt.timestep / 10))):
            a.cmd = (np.asarray(q), np.zeros(6), np.full(6, 50.0), np.ones(6), np.zeros(6))
            a.gcmd = (g, 0.0, 5.0, 0.5, 0.0)
            a.last_frame = d.time
            w.swerve_op("modules", {"steer": {n: [True, 0.0] for n in w.module_names},
                                    "drive": {n: [True, 0.0] for n in w.module_names}})   # base held: zero twist
            w.step(10)
    run(down[0], GRIPPER_OPEN_RAD, 2.0)
    for q in down[1:]:
        run(q, GRIPPER_OPEN_RAD, 0.15)
    run(down[-1], GRIPPER_OPEN_RAD, 0.5)
    z0 = float(d.xpos[bid][2])
    run(down[-1], 0.0, 1.0)                            # close
    gap, force = 2 * float(d.qpos[w.finger_q]), float(d.actuator_force[w.finger_a])
    for q in up:
        run(q, 0.0, 0.15)
    run(up[-1], 0.0, 0.5)
    R = d.xmat[e].reshape(3, 3).copy()
    rel0 = R.T @ (d.xpos[bid] - d.xpos[e])
    out = {"arm_lift": float(d.xpos[e][2] - tip[2]), "lift": float(d.xpos[bid][2] - z0), "gap": gap,
           "force": force, "width": min(o["size"][:2]) if o["shape"] == "box" else 2 * o["radius"]}
    run(up[-1], 0.0, HOLD)
    R = d.xmat[e].reshape(3, 3)
    out["slip"] = float(np.linalg.norm(R.T @ (d.xpos[bid] - d.xpos[e]) - rel0))
    out["fault"] = a.fault
    return out


@pytest.mark.parametrize("variant", ["full", "boxes", "simple"])
@pytest.mark.parametrize("obj", ["cube", "can", "block"])
def test_top_down_grasp_lift_and_hold(variant, obj):
    w = make_world(variant, scene={"tabletop": True})
    r = grasp(w, obj)
    assert r["fault"] == 0
    assert r["arm_lift"] == pytest.approx(LIFT, abs=0.005), r          # the arm reached the lift pose ...
    assert r["lift"] > LIFT - 0.01, r                                   # ... and the object came with it
    assert r["slip"] < 0.005, r                                         # held 2 s without slipping
    assert r["width"] - 0.003 < r["gap"] < r["width"] + 0.001, r        # jaws on the object, not through it
    assert abs(r["force"]) <= w.limits["finger_force_max"] + 1e-6, r    # the gripper's force cap


def test_jaws_collide_as_the_jaw_boxes_in_every_variant():
    from robot_utils.nodes.description.model import arm_box_geometry
    want = sorted(tuple(np.round(s, 4)) for _, _, s in arm_box_geometry()["finger_left_link"])
    for variant in ("full", "boxes", "simple"):
        w = make_world(variant)
        m = w.m
        for link in ("arm_finger_left_link", "arm_finger_right_link"):
            b = m.body(link).id
            coll = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and (m.geom_contype[g] or m.geom_conaffinity[g])]
            assert sorted(tuple(np.round(2 * m.geom_size[g], 4)) for g in coll) == want, (variant, link)
            assert all(m.geom_condim[g] == w.cfg["contact"]["fingers"]["condim"] for g in coll)
            vis = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and not m.geom_contype[g]]
            assert vis, (variant, link)                                 # the visual geometry stays
        assert m.opt.noslip_iterations == w.cfg["contact"]["noslip_iterations"]
