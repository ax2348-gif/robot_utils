"""The arm node's gripper: its command state, request validation and the force-limit law (pure functions)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from . import config as C


@dataclass
class Gripper:
    goal: Optional[float] = None       # m gap: the MIT position command (None while free)
    sent: Optional[float] = None       # m gap actually sent (the force limit may pull it back)
    max_force: float = C.DEFAULT_MAX_FORCE


def check_gripper(gap_mm: Optional[float] = None, max_force_n: Optional[float] = None):
    """Validated (gap m or None, max force N or None). Raises ValueError before anything changes."""
    f = None
    if max_force_n is not None:
        f = float(max_force_n)
        if not (math.isfinite(f) and 0.0 <= f <= C.HARD_MAX_FORCE):
            raise ValueError(f"max_force_n must be within 0..{C.HARD_MAX_FORCE:g} N")
    gap = None
    if gap_mm is not None:
        gap = float(gap_mm) / 1000.0
        if not (math.isfinite(gap) and 0.0 <= gap <= C.GAP_MAX + 1e-9):
            raise ValueError(f"gap_mm must be within 0..{C.GAP_MAX * 1000:.1f}")
    return gap, f


@dataclass
class GripCommand:
    """One tick's gripper MIT command and what the force limit made of it."""
    q: float              # rad position sent (motor side)
    kp: float
    kd: float
    ff: float             # Nm force-limit feedforward (opposes closing)
    tau_est: float        # Nm the MIT law alone applies at the goal (NaN while free)
    demand: float         # N squeeze force the MIT law alone asks for (NaN while free)
    applied: float        # N squeeze force applied, estimated (NaN while free)
    limiting: bool


def holding(goal_gap: float, max_force: float, g_q: float, g_q_mot: float, g_qd: float,
            g_kp: float, g_kd: float) -> GripCommand:
    """Gripper holding ``goal_gap``: MIT position at the goal, velocity reference 0 (the damping term never adds
    grip force). Force limit from the torque its MIT law applies (motor side): away from fully open an opposing
    feedforward, never larger than that torque (the net torque never opens); within FF_OPEN_MARGIN of fully
    open no feedforward, the position command is pulled back toward the jaws instead. ``g_q``: output encoder,
    ``g_q_mot`` / ``g_qd``: motor feedback."""
    q = C.gap_to_rad(goal_gap)
    tau_est = g_kp * (q - g_q_mot) + g_kd * (0.0 - g_qd)
    force = C.force_of_torque(max(tau_est, 0.0))
    tau_lim = C.torque_of_force(max_force)
    ff, limiting = 0.0, False
    if force > max_force:
        limiting = True
        if C.rad_to_gap(g_q) > C.GAP_MAX - C.FF_OPEN_MARGIN:
            q = min(q, g_q_mot + tau_lim / g_kp)          # closing is +rad
        else:
            ff = -(tau_est - tau_lim)                      # opposes closing; |ff| < tau_est
    # what is applied: the MIT law with the position actually sent, plus the feedforward
    applied = C.force_of_torque(max(g_kp * (q - g_q_mot) + g_kd * (0.0 - g_qd) + ff, 0.0))
    return GripCommand(q, g_kp, g_kd, ff, tau_est, force, applied, limiting)


def free(g_q_mot: float) -> GripCommand:
    """Gripper free (gainless): the command follows the motor."""
    return GripCommand(g_q_mot, 0.0, 0.0, 0.0, math.nan, math.nan, math.nan, False)
