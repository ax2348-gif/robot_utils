"""Grasp-hold run for the OpenArm's gripper (Anvil's, from Anvil's URDF), as its arm node would command it. Run in a
process of its own (tests/test_openarm.py starts it): the arm type goes in the environment before arm/config.py is
imported, so the gripper law, its gap and force constants are the arm's, as in its arm node.

    python openarm_grasp_run.py WIDTH_MM [WIDTH_MM ...]   -> one JSON line per width

The dynamic sim (MIT law per motor, the gripper motor through its equivalent pinion), the right arm: a box of the given
width across the jaws on the 0.70 m table, grasped from above and in front (IK on the arm's URDF), close at the arm node's default
squeeze through ``gripper.holding`` (its contact detector on the previous tick's reading), lift 10 cm at 0.1 m/s, hold,
shake +-2 cm at 2 Hz, hold. Slip = the object's displacement in end_link since the jaws closed (mm)."""
import json
import math
import os
import sys

import numpy as np
import pinocchio as pin

from robot_utils.robots import load
from robot_utils.robots.arm_types import ENV

ROBOT, ARM = "openarm_bimanual", "arm_right"
prof = load(ROBOT)
spec = prof.arm(ARM)
atype = spec.arm_type()
os.environ[ENV] = json.dumps(atype.public())
from robot_utils.nodes.arm import config as C  # noqa: E402
from robot_utils.nodes.arm import gripper as G  # noqa: E402
from robot_utils.nodes.sim.node import fetch_bundle  # noqa: E402
from robot_utils.nodes.sim.scene import tabletop  # noqa: E402
from robot_utils.nodes.sim.world import World, load_config, unzip_bundle  # noqa: E402

DT = 0.01
assert abs(C.GAP_MAX - atype.gripper_type().gap_max) < 1e-9          # the arm's gripper constants, not the reBot's
model = pin.buildModelFromUrdf(spec.urdf_path())
data = model.createData()
END = model.getFrameId("end_link")
TOP = prof.sim["scene"]["work_top"]


def ik(T_goal, q0):
    q = pin.neutral(model)
    q[:7] = q0
    lo, hi = model.lowerPositionLimit[:7], model.upperPositionLimit[:7]
    for _ in range(300):
        pin.framesForwardKinematics(model, data, q)
        T = data.oMf[END]
        e = np.concatenate([T_goal.translation - T.translation, pin.log3(T_goal.rotation @ T.rotation.T)])
        if np.linalg.norm(e) < 1e-5:
            break
        pin.computeJointJacobians(model, data, q)
        J = pin.getFrameJacobian(model, data, END, pin.LOCAL_WORLD_ALIGNED)[:, :7]
        q[:7] = np.clip(q[:7] + J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e), lo, hi)
    return q[:7].copy(), float(np.linalg.norm(e))


def gravity(q7, gap):
    q = pin.neutral(model)
    q[:7] = q7
    return pin.computeGeneralizedGravity(model, data, q)[:7] * np.asarray(atype.tau_scale)


def run(width_mm: float) -> dict:
    data_, how = fetch_bundle("local", "full", ROBOT)
    urdf, assets, info = unzip_bundle(data_)
    w_ = width_mm / 1000
    mount = np.array(info["frames_in_base"][f"{spec.prefix}base_link"])[:3, 3]
    p_obj = np.array([0.44, -0.22, TOP + 0.03 + 0.001])                # box 60 mm high on the table
    box = {"name": "box", "shape": "box", "size": [0.04, w_, 0.06], "mass": 0.25, "xyz": p_obj.tolist(),
           "rgba": [0.85, 0.2, 0.15, 1]}
    over = {"time": {"run": False}, "timing": {"lenient": False}, "components": {"arm": {"fidelity": "dynamic"}},
            "scene": {"tabletop": False, "work_top": TOP, "objects": tabletop(TOP)[:5] + [box]}, "cameras": {}}
    cfg = load_config(None, over, robot=prof.sim)
    # the table and its legs only (the tabletop's small objects out of the way), and the box
    w = World(urdf, assets, cfg, {"source": how, "arms": info.get("arms"), "frames_in_base": info.get("frames_in_base")})
    m, d = w.m, w.d
    bid, e = m.body("obj_box").id, m.body(f"{spec.prefix}end_link").id
    # the tool 60 deg from straight down, tilted forward (straight down at the table's height this close to the body is
    # beyond the wrist's range: no solution within the joint limits; with Anvil's J6 range, -70..+45 deg on the right,
    # 50 deg leaves the wrist within 0.03 rad of its limit), the jaws along world y (the wrist camera up); fingertips
    # 30 mm over the table (the pads' corners reach lower at this tilt)
    a = math.radians(60)
    xt = np.array([math.sin(a), 0.0, -math.cos(a)])
    R_down = np.column_stack([xt, [0.0, 1.0, 0.0], np.cross(xt, [0.0, 1.0, 0.0])])
    tips_at = p_obj - np.array([0, 0, 0.03]) + np.array([0, 0, 0.03])
    q_nom = np.array(spec.posture, float)
    path = []
    rng = np.random.default_rng(0)
    lo, hi = model.lowerPositionLimit[:7], model.upperPositionLimit[:7]
    best = None
    for s_ in range(40):                                                # the first waypoint: multi-start IK
        q, err = ik(pin.SE3(R_down, tips_at + [0, 0, 0.12] - mount), q_nom if s_ == 0 else lo + rng.random(7) * (hi - lo))
        if err < 1e-4 and np.min(np.minimum(q - lo, hi - q)) > 0.05:
            best = q if best is None or np.sum(np.abs(q - q_nom)) < np.sum(np.abs(best - q_nom)) else best
    assert best is not None, "no grasp solution"
    q = best
    for z in np.linspace(0.12, 0.0, 120):                               # descend 12 cm over 1.2 s
        q, err = ik(pin.SE3(R_down, tips_at + [0, 0, z] - mount), q)
        assert err < 1e-3, (width_mm, z, err)
        path.append(("approach", q))
    path += [("settle", q)] * 40 + [("close", q)] * 120
    for z in np.linspace(0.0, 0.10, 100):                               # lift 10 cm at 0.1 m/s
        q, _ = ik(pin.SE3(R_down, tips_at + [0, 0, z] - mount), q)
        path.append(("lift", q))
    q_up = q
    path += [("hold", q_up)] * 50
    for i in range(1, 151):                                             # shake +-2 cm at 2 Hz, 1.5 s
        dy = 0.02 * math.sin(2 * math.pi * 2 * i * DT)
        q, _ = ik(pin.SE3(R_down, tips_at + [0, dy, 0.10] - mount), q)
        path.append(("shake", q))
    path += [("end", q_up)] * 100
    # the other arm holds its posture
    other = next(a for a in prof.arms if a.name != ARM)
    ot = other.arm_type()
    om = pin.buildModelFromUrdf(other.urdf_path())
    od = om.createData()
    oq = pin.neutral(om)
    oq[:7] = other.posture
    o_tau = pin.computeGeneralizedGravity(om, od, oq)[:7]
    # start the arm at the first waypoint (a reset settles it there)
    for a, qq in ((ARM, path[0][1]), (other.name, np.array(other.posture))):
        w.cfg["components"].setdefault("arms", {}).setdefault(a, {})["start_q"] = list(qq)
    w.__dict__.pop("_arm_cfgs", None)
    w.reset()
    w.arm_op("enable", {}, ARM)
    w.arm_op("enable", {}, other.name)
    st = G.Contact()
    prev = w.arm_op("read", {}, ARM)
    n = int(round(DT / m.opt.timestep))
    t, rel0, out = 0.0, None, {"width_mm": width_mm, "slip": {}}
    goal = C.GAP_MAX
    for phase, qq in path:
        if phase == "close":
            goal = 0.0
        g = G.holding(goal, C.DEFAULT_MAX_FORCE, prev["g_q"], prev["g_q"], prev["g_qd"], atype.g_kp, atype.g_kd, t, st)
        w.arm_op("send", {"q": list(qq), "qd": [0.0] * 7, "kp": list(atype.kp), "kd": list(atype.kd),
                          "tau": list(gravity(prev["q"], 0.0)), "gripper": True, "g_q": g.q, "g_qd": 0.0,
                          "g_kp": g.kp, "g_kd": g.kd, "g_tau": g.ff}, ARM)
        w.arm_op("send", {"q": list(other.posture), "qd": [0.0] * 7, "kp": list(ot.kp), "kd": list(ot.kd),
                          "tau": list(o_tau), "gripper": False}, other.name)
        prev = w.arm_op("read", {}, ARM)
        w.step(n)
        t += DT
        R = d.xmat[e].reshape(3, 3)
        rel = R.T @ (d.xpos[bid] - d.xpos[e])
        if phase == "close":
            rel0 = rel.copy()
            out["gap_mm"] = round(-prev["g_q"] * C.GAP_PER_RAD * 1000, 1)
            out["squeeze_n"] = round(g.applied, 1)
            out["contact"] = bool(st.on)
        elif rel0 is not None:
            out["slip"][phase] = max(out["slip"].get(phase, 0.0), round(float(np.linalg.norm(rel - rel0)) * 1000, 2))
    out["lifted_mm"] = round((float(d.xpos[bid][2]) - p_obj[2]) * 1000, 1)
    out["faults"] = [w.arm_op("read", {}, a)["status"] for a in (ARM, other.name)]
    return out


if __name__ == "__main__":
    for wmm in sys.argv[1:]:
        print(json.dumps(run(float(wmm))), flush=True)
