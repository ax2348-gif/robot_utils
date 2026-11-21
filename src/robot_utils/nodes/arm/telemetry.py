"""The arm node's telemetry: the tick row's schema (FIELDS, UNITS, COL, FIELD_DOC; the jacobian channel's
JAC_FIELDS) and the pure functions that build a row and read one back.

The row schema is append-only: recordings and pages read columns by name, but old recordings were
written in this order, so a new column goes at the end (``_JAM`` was appended last for that reason).
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import pinocchio as pin

from . import config as C
from .config import N
from .kin import pose_error

STATES = ["connecting", "uninitialized", "initializing", "off", "disabled", "floating", "servoing", "fault"]
SOURCES = ["none", "hold", "track", "servo", "pose", "home", "stow", "chunk"]
PHASES = ["none", "executing", "settling", "reached", "not_reached", "done", "aborted", "preempted", "jammed"]
JOINT_COLS = [("q", "rad"), ("q_mot", "rad"), ("qd", "rad/s"), ("tau", "Nm"), ("q_cmd", "rad"),
              ("qd_ff", "rad/s"), ("tau_g", "Nm"), ("e_mot", "rad"), ("tau_est", "Nm"), ("acc_lim", "bool"),
              ("t_mos", "C"), ("status", "enum")]
_BASE = [("t_tick", "s"), ("tick", "count"), ("state", "enum"), ("enabled", "bool"), ("motors_on", "bool"),
         ("servo", "bool"), ("g_servo", "bool"), ("estop", "bool"), ("lease_id", "id"), ("source", "enum"),
         ("seq", "count"), ("overruns", "count"), ("tick_dt", "s"), ("limited", "bool"), ("twist_age", "s"),
         ("read_age", "s"), ("read_misses", "count")]
_TCP = [("x", "m"), ("y", "m"), ("z", "m"), ("qx", ""), ("qy", ""), ("qz", ""), ("qw", ""),
        ("cmd_x", "m"), ("cmd_y", "m"), ("cmd_z", "m"),
        ("err_pos", "m"), ("err_rot", "rad"), ("goal_err_pos", "m"), ("goal_err_rot", "rad"),
        ("w", ""), ("w_lin", ""), ("sigma_min", "")]
_GRIP = [("g_gap", "m"), ("g_gap_cmd", "m"), ("g_gap_goal", "m"), ("g_qd", "rad/s"), ("g_tau", "Nm"),
         ("g_tau_est", "Nm"), ("g_force", "N"), ("g_force_demand", "N"), ("g_tau_ff", "Nm"), ("g_force_max", "N"), ("g_limiting", "bool"),
         ("g_status", "enum")]
_MOTION = [("motion_id", "id"), ("phase", "enum"), ("progress", "")]
_JAM = [("jam", "bool")]   # appended last: existing columns keep their positions
_ALL = (_BASE + [(f"j{i}_{n}", u) for i in range(1, N + 1) for n, u in JOINT_COLS] + _TCP + _GRIP + _MOTION + _JAM)
FIELDS = [n for n, _ in _ALL]
UNITS = [u for _, u in _ALL]
COL = {n: i for i, n in enumerate(FIELDS)}
JAC_FIELDS = ["t_tick"] + [f"J{r}{c}" for r in range(6) for c in range(1, N + 1)]
JAC_UNITS = ["s"] + [""] * 36
FIELD_DOC = {
    "state": "index into " + ", ".join(STATES),
    "servo / g_servo": "arm / gripper closed loop engaged",
    "source": "arm motion source in force: index into " + ", ".join(SOURCES) + " (none = not servoing)",
    "phase": "of the planned motion (servo / pose / home / stow / chunk): index into " + ", ".join(PHASES),
    "jam": "1 while a jam is in force: the arm static but still pushing (residual over its band; GET /jam)",
    "j<i>_q": "output encoder (xout): the arm's measured pose",
    "j<i>_q_mot / qd / tau": "motor-side feedback frame",
    "j<i>_q_cmd / qd_ff": "position command and the MIT velocity reference (track: the twist's joint velocity while fresh; planned motions: the trajectory's) (NaN when not servoing)",
    "j<i>_acc_lim": "1 if the acceleration limit held back an increase of this joint's commanded speed this tick",
    "j<i>_e_mot": "q_cmd - q_mot: the error the motor's MIT loop sees",
    "j<i>_tau_est": "kp*e_mot + kd*(qd_ff - qd) + tau_g: torque the MIT law applies",
    "j<i>_tau_g": "gravity feedforward (whenever the motors are on)",
    "j<i>_status": "Damiao status code (1 enabled, 0 disabled, >= 8 fault)",
    "x..qw": "TCP pose from the output encoders, arm frame (quaternion x, y, z, w)",
    "cmd_x..z": "TCP position of q_cmd",
    "err_pos / err_rot": "measured TCP vs commanded TCP (output side)",
    "goal_err_pos / rot": "commanded TCP vs the goal of the source",
    "w / w_lin / sigma_min": "manipulability sqrt(det(J J^T)) of the 6x6 TCP Jacobian, of its translational rows, and its smallest singular value, at the measured pose",
    "limited": "the joint speed limit held q_cmd back this tick (the step limit)",
    "read_age": "age of the reading this tick used: 0 normally; > 0 when replies were late (still awaited, one tick) or missed and the latest complete reading was kept (fault beyond 50 ms with the motors on)",
    "read_misses": "ticks with missed replies (none within two ticks) since the node started (per motor in /state)",
    "twist_age": "s since the track twist feedforward was written (NaN if none)",
    "g_gap / g_gap_cmd / g_gap_goal": "jaw gap measured / sent this tick (after the force limit) / goal (m)",
    "g_force": "squeeze force applied, estimated: the gripper's MIT law plus the force-limit feedforward, over the pinion (N)",
    "g_force_demand": "squeeze force the MIT law alone asks for, before the force limit (N); above g_force_max while limiting",
    "g_tau_ff": "force-limit feedforward (opposes closing; |g_tau_ff| <= g_tau_est)",
    "J<r><c>": "jacobian channel: TCP Jacobian at the measured pose, arm-frame axes (LOCAL_WORLD_ALIGNED), row r of [vx vy vz wx wy wz], column = joint",
}


def tick_row(now: float, base: list, r, kin, profile, q_cmd: Optional[np.ndarray], v_cmd: np.ndarray,
             acc_lim: np.ndarray, tau_g, goal: Optional[pin.SE3], gripper, g_est: dict, g_live: bool,
             motion, jam: bool) -> tuple[np.ndarray, np.ndarray]:
    """One tick's telemetry row (FIELDS) and the TCP Jacobian at the measured pose (for the jacobian channel).

    ``base``: the _BASE columns' values, in order. ``r``: the reading. ``q_cmd``: the command while servoing
    (None otherwise), with ``v_cmd`` its MIT velocity reference and ``acc_lim`` the acceleration-limit flags.
    ``tau_g``: the gravity feedforward (None with the motors off). ``goal``: the source's goal pose.
    ``gripper``: the Gripper, ``g_est`` the force-limit estimates (``tau_est``, ``force``, ``demand``, ``ff``,
    ``limiting``), ``g_live``: the gripper holding with the motors on. ``motion``: the motion in force or
    the last one (or None). ``jam``: a jam in force."""
    nan = math.nan
    row = np.full(len(FIELDS), nan)
    row[:len(_BASE)] = base
    p = profile
    for i in range(N):
        b = len(_BASE) + i * len(JOINT_COLS)
        tg = tau_g[i] if tau_g is not None else nan
        if q_cmd is not None:
            qc, vc = q_cmd[i], v_cmd[i]
            e = qc - r.q_mot[i]
            te = p.kp[i] * e + p.kd[i] * (vc - r.qd[i]) + tg
            al = float(acc_lim[i])
        else:
            qc = vc = e = al = nan
            te = tg
        row[b:b + len(JOINT_COLS)] = [r.q[i], r.q_mot[i], r.qd[i], r.tau[i], qc, vc, tg, e, te, al,
                                      r.t_mos[i], r.status[i]]
    T = kin.fk(r.q)
    J = kin.jacobian(r.q, "arm")
    mn = kin.manipulability(J)
    quat = pin.Quaternion(T.rotation)
    b = COL["x"]
    row[b:b + 7] = [*T.translation, quat.x, quat.y, quat.z, quat.w]
    if q_cmd is not None:
        Tc = kin.fk(q_cmd)
        row[COL["cmd_x"]:COL["cmd_x"] + 3] = Tc.translation
        row[COL["err_pos"]], row[COL["err_rot"]] = pose_error(T, Tc)
        if goal is not None:
            row[COL["goal_err_pos"]], row[COL["goal_err_rot"]] = pose_error(Tc, goal)
    row[COL["w"]], row[COL["w_lin"]], row[COL["sigma_min"]] = mn["w"], mn["w_lin"], mn["sigma_min"]
    g, gd, gl = gripper, g_est, g_live
    row[COL["g_gap"]:COL["g_gap"] + len(_GRIP)] = [
        C.rad_to_gap(r.g_q), g.sent if (gl and g.sent is not None) else nan,
        g.goal if (gl and g.goal is not None) else nan, r.g_qd, r.g_tau,
        gd["tau_est"] if gl else nan, gd["force"] if gl else nan, gd["demand"] if gl else nan,
        gd["ff"] if gl else nan, g.max_force,
        float(gd["limiting"] and gl), r.status[N]]
    if motion is not None:
        mp = motion.public(now)
        row[COL["motion_id"]], row[COL["phase"]], row[COL["progress"]] = motion.traj.id, PHASES.index(motion.phase), mp["progress"]
    else:
        row[COL["phase"]] = 0
    row[COL["jam"]] = float(jam)
    return row, J


def row_dict(row) -> dict:
    """A tick row by field name, non-finite values as None (``latest`` in /state)."""
    return {k: (v if math.isfinite(v) else None) for k, v in zip(FIELDS, row.tolist())}


def tracking(row) -> dict:
    """The tracking errors of a tick row (GET /tracking)."""
    g = lambda n: None if not math.isfinite(row[COL[n]]) else float(row[COL[n]])   # noqa: E731
    return {"servo": bool(row[COL["servo"]] == 1.0),
            "e_mot": [g(f"j{i}_e_mot") for i in range(1, N + 1)],
            "tau_est": [g(f"j{i}_tau_est") for i in range(1, N + 1)],
            "tau_g": [g(f"j{i}_tau_g") for i in range(1, N + 1)],
            "err_pos_m": g("err_pos"), "err_rot_rad": g("err_rot"),
            "goal_err_pos_m": g("goal_err_pos"), "goal_err_rot_rad": g("goal_err_rot"),
            "limited": bool(row[COL["limited"]] == 1.0), "t": float(row[0])}
