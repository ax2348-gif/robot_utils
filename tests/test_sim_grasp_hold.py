"""Holding a grasp while the arm moves at teleop speeds, in both arm fidelities (docs/sim.md, "Grasping").

The arm node's own commands, tick by tick (10 ms): the joints at the soft profile's kp / kd with the node's gravity
feedforward (pinocchio x tau_scale), the gripper through the node's force-limited law (``gripper.holding``: gap goal
0, ``DEFAULT_MAX_FORCE`` squeeze, its contact detector on the previous tick's reading, as the pipelined bus gives
it). Top-down grasp of the tabletop cube and can, lift 10 cm, carry 10 cm sideways at 0.3 m/s and back, shake
+-2 cm at 2 Hz (0.25 m/s, 3.2 m/s^2), hold. Slip = the object's displacement in the gripper frame since the jaws
closed (mm)."""
import math

import numpy as np
import pytest

from sim_helpers import make_world
from test_sim_grasp import CLEAR, RAIL_FACE
from test_sim_scene import reach_down

from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm import gripper as G

DT = 0.01
TRACE = False
P = C.load_profile("soft")
_KIN = []


def node_gravity(q):
    if not _KIN:
        from robot_utils.nodes.arm.kin import ArmKinematics
        _KIN.append(ArmKinematics({}))
    return _KIN[0].gravity(np.asarray(q, float)) * P.tau_scale


def pad_forces(w, bid):
    """{finger: (contacts, total normal force N)} between the object and each finger."""
    import mujoco
    m, d = w.m, w.d
    out = {}
    for i in range(d.ncon):
        c = d.contact[i]
        b = {int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])}
        if bid not in b:
            continue
        other = m.body((b - {bid}).pop()).name
        f = np.zeros(6)
        mujoco.mj_contactForce(m, d, i, f)
        n, tot = out.get(other, (0, 0.0))
        out[other] = (n + 1, round(tot + f[0], 1))
    return out


def path(w, obj):
    """Joint waypoints at the 10 ms tick: (phase, q) list."""
    o = w.object_state(obj)
    top = w.object_state("table")["xyz"][2] + w.objects["table"]["size"][2] / 2
    h = o["size"][2] if o["shape"] == "box" else o["length"]
    tip = np.array(o["xyz"][:2] + [max(top + CLEAR, top + h - RAIL_FACE + CLEAR)])
    jaw = None
    if o["shape"] == "box":
        yaw = o["rpy"][2]
        jaw = [np.cos(yaw), np.sin(yaw), 0.0] if o["size"][0] <= o["size"][1] else [-np.sin(yaw), np.cos(yaw), 0.0]
    kw = dict(offset=np.zeros(3), pos_tol=0.001, iters=800)
    out, q = [], None

    def to(p, phase, tol=1.5):
        nonlocal q
        r = reach_down(w, p, jaw=jaw, seed=q, ang_tol_deg=tol, tries=1 if q is not None else 12, **kw)
        if r is None:
            r = reach_down(w, p, seed=q, ang_tol_deg=35, tries=3, **kw)
        assert r is not None, (obj, phase, p)
        q = r
        out.append((phase, r))
    for k in range(9):                                   # down 8 cm in 1 cm steps, 0.15 s each
        for _ in range(15 if k else 150):
            to(tip + [0, 0, 0.08 - 0.01 * k], "approach") if _ == 0 else out.append(("approach", q))
    out += [("settle", q)] * 50
    out += [("close", q)] * 100
    lift = tip + [0, 0, 0.10]
    for i in range(1, 101):                              # lift 10 cm at 0.1 m/s
        to(tip + [0, 0, 0.001 * i], "lift", tol=35)
    out += [("hold", q)] * 30
    for s in list(range(1, 34)) + list(range(32, 15, -1)):      # 10 cm sideways at 0.3 m/s, half back
        to(lift + [0, -0.1 * s / 33, 0], "carry", tol=35)
    for i in range(1, 201):                              # shake +-2 cm at 2 Hz diagonally (x and y), 2 s
        to(lift + [-0.02 * math.sin(2 * math.pi * 2 * i * DT), -0.05 + 0.02 * math.sin(2 * math.pi * 2 * i * DT), 0], "shake", tol=35)
    out += [("hold", q)] * 50
    for i in range(1, 201):                              # wrist pitch +-0.5 rad at 1 Hz (3 rad/s peak), 2 s
        out.append(("pitch", q + np.array([0, 0, 0, 0, 0.5 * math.sin(2 * math.pi * i * DT), 0])))
    for i in range(1, 201):                              # wrist roll +-0.8 rad at 1 Hz (5 rad/s peak), 2 s
        out.append(("roll", q + np.array([0, 0, 0, 0, 0, 0.8 * math.sin(2 * math.pi * i * DT)])))
    out += [("end", q)] * 100
    return out


def run_grasp(fidelity, obj, variant="full", cfg=None, max_force=C.DEFAULT_MAX_FORCE):
    over = {"scene": {"tabletop": True}, "components": {"arm": {"fidelity": fidelity}}}
    for k, v in (cfg or {}).items():
        over[k] = v if not isinstance(v, dict) or k not in over else {**over[k], **v}
    w = make_world(variant, **over)
    m, d = w.m, w.d
    bid, e = m.body("obj_" + obj).id, m.body("arm_end_link").id
    steps = path(w, obj)
    w.arm_op("enable", {})
    st = G.Contact()
    goal = C.GAP_MAX
    prev = w.arm_op("read", {})
    rel0, out, z0 = None, {"slip": {}, "turn": {}}, float(d.xpos[bid][2])
    n = int(round(DT / m.opt.timestep))
    t = 0.0
    last = [None]
    for phase, q in steps:
        w.swerve_op("modules", {"steer": {k: [True, 0.0] for k in w.module_names},
                                "drive": {k: [True, 0.0] for k in w.module_names}})
        if phase == "close":
            goal = 0.0
        g = G.holding(goal, max_force, prev["g_q"], prev["g_q"], prev["g_qd"], P.g_kp, P.g_kd, t, st)
        w.arm_op("send", {"q": list(q), "qd": [0.0] * 6, "kp": list(P.kp), "kd": list(P.kd),
                          "tau": list(node_gravity(prev["q"])), "gripper": True, "g_q": g.q, "g_qd": 0.0,
                          "g_kp": g.kp, "g_kd": g.kd, "g_tau": g.ff})
        prev = w.arm_op("read", {})                       # what the node sees next tick (pipelined)
        w.step(n)
        t += DT
        R = d.xmat[e].reshape(3, 3)
        rel = R.T @ (d.xpos[bid] - d.xpos[e])
        Rrel = R.T @ d.xmat[bid].reshape(3, 3)
        if phase == "close":
            rel0, Rrel0 = rel.copy(), Rrel.copy()
            out["gap_mm"] = 2 * float(d.qpos[w.finger_q]) * 1000
            out["applied_n"] = g.applied
            out["contact"] = st.on
        elif rel0 is not None:
            s = float(np.linalg.norm(rel - rel0)) * 1000
            a = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Rrel0.T @ Rrel) - 1) / 2))))
            out["slip"][phase] = max(out["slip"].get(phase, 0.0), round(s, 2))
            if TRACE and (phase != last[0]):
                print("   ", last[0], "->", phase, "d(mm, end_link xyz)", np.round((rel - rel0) * 1000, 2),
                      "gap", round(2 * float(d.qpos[w.finger_q]) * 1000, 2), "g_q", round(prev["g_q"], 3),
                      "contact", st.on, "applied", round(g.applied, 1), "ff", round(g.ff, 3),
                      "normals N", pad_forces(w, bid), "act", round(float(d.actuator_force[w.finger_a]), 1))
            out["turn"][phase] = max(out["turn"].get(phase, 0.0), round(a, 1))
        last[0] = phase
    out["slip_max"] = max(out["slip"].values())
    out["slip_end"] = float(np.linalg.norm(rel - rel0)) * 1000
    out["lift_mm"] = (float(d.xpos[bid][2]) - z0) * 1000
    out["fault"] = w.arm.fault
    w.close()
    return out


# Measured 2026-11-24 (slip mm / turn deg, max over the run; cube, can):
#   before (jaw contacts mixed with the objects' solref; kinematic finger capped by the URDF's 8 N effort):
#     dynamic 5.1 / 1.5, 5.2 / 1.5;  kinematic 215 / 174 (dropped), 19 / 64
#   after: dynamic 0.46 / 0.3, 0.46 / 0.3;  kinematic 4.6 / 12.5, 0.7 / 9.7 (its squeeze is the sim servo's, not the
#   node's force limit, and the wrist roll at 5 rad/s turns the object in the jaws: docs/sim.md "Grasping")
LIMITS = {"dynamic": (1.0, 1.0), "kinematic": (6.0, 15.0)}


@pytest.mark.parametrize("fidelity", ["dynamic", "kinematic"])
@pytest.mark.parametrize("obj", ["cube", "can"])
def test_grasp_holds_through_teleop_motion(fidelity, obj):
    r = run_grasp(fidelity, obj)
    slip, turn = LIMITS[fidelity]
    assert r["fault"] == 0, r
    assert r["lift_mm"] > 90, r                         # still in the hand, 10 cm up
    assert r["slip_max"] < slip and r["slip_end"] < slip, r
    assert max(r["turn"].values()) < turn, r
    assert r["contact"] and r["applied_n"] == pytest.approx(C.DEFAULT_MAX_FORCE, abs=1.0), r   # the node's limit


if __name__ == "__main__":
    import sys
    TRACE = True
    for fid in sys.argv[1:] or ["dynamic", "kinematic"]:
        for obj in ("cube", "can"):
            print(fid, obj, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in run_grasp(fid, obj).items()})
