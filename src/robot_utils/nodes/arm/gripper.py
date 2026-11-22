"""The arm node's gripper: its command state, request validation and the contact force-limit law.

The force limit bounds the SQUEEZE (the jaws on something), not free motion. Free, the gripper runs the plain MIT
position law toward its goal with the profile's gains, uncapped (as fast as the raw interface). A contact detector
on the bus feedback (motor-side position and velocity) switches the limit on: the jaws stalled (motor speed below
``stall_vel``) for ``stall_s`` with the goal still past them by more than the limit's own error (``max_force``
worth of kp) and more than ``contact_err``. In contact the closing torque is capped at ``max_force``; it stays in
contact until the command stops squeezing past the limit (hysteresis ``release``). The position where the jaws
stalled is remembered: closing back toward it (within ``approach``) is capped too, so a re-grasp of the same object
never hits it with more than the limit. The cap acts on the law's position term (what squeezes at rest); its
damping term -kd * qd is left alone. Opening is never limited (the cap only ever removes closing torque).

The motor's torque reading (``g_tau``) is not used: under the MIT law the motor delivers the torque it is commanded
whether the jaws move or not, so the reading equals the law's output in free motion as in contact and carries no
contact information that position and velocity do not."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from . import config as C


@dataclass
class Contact:
    """The contact detector's state (control thread)."""
    on: bool = False                    # in contact: the squeeze is capped at max_force
    stall_since: Optional[float] = None   # s: the jaws stalled against a closing goal since (not yet in contact)
    at: Optional[float] = None          # rad motor side: where the jaws last stalled on something (None: forgotten)

    def reset(self) -> None:
        self.on, self.stall_since, self.at = False, None, None


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
    limiting: bool        # the cap binds this tick (in contact, or closing back onto a remembered contact)
    contact: bool = False   # the detector's state: the jaws are on something


def detect(st: Contact, now: float, e: float, e_lim: float, g_q_mot: float, g_qd: float,
           cfg: C.GripperContact) -> bool:
    """Update the contact detector; True where the cap applies (in contact, or within ``approach`` of the
    remembered contact position). ``e``: closing position error (rad, goal minus motor position), ``e_lim``: the
    error at which the law alone reaches the limit (rad)."""
    rad = 1.0 / C.GAP_PER_RAD
    if not st.on:
        stalled = abs(g_qd) < cfg.stall_vel and e > max(e_lim, cfg.contact_err * rad)
        if not stalled:
            st.stall_since = None
        else:
            if st.stall_since is None:
                st.stall_since = now
            if now - st.stall_since >= cfg.stall_s - 1e-9:
                st.on, st.stall_since, st.at = True, None, g_q_mot
    elif e < e_lim - cfg.release * rad:          # the command no longer squeezes past the limit: free again
        st.on = False
    elif abs(g_qd) < cfg.stall_vel:              # still on it (a soft object: where it is now)
        st.at = g_q_mot
    if not st.on and st.at is not None:          # forget it: closed through it (gone), or opened well away
        if g_q_mot > st.at + cfg.release * rad or g_q_mot < st.at - cfg.forget * rad:
            st.at = None
    return st.on or (st.at is not None and g_q_mot >= st.at - cfg.approach * rad)   # closing is +rad


def holding(goal_gap: float, max_force: float, g_q: float, g_q_mot: float, g_qd: float,
            g_kp: float, g_kd: float, now: float = 0.0, st: Optional[Contact] = None,
            cfg: Optional[C.GripperContact] = None) -> GripCommand:
    """Gripper holding ``goal_gap``: MIT position at the goal, velocity reference 0 (the damping term never adds
    grip force). Free, that is all (uncapped). Where the contact detector (``st``, updated here; None: a fresh one,
    so never in contact) applies the cap, the law's position term (motor side; the squeeze at rest) is limited to
    ``max_force``: away from fully open an opposing feedforward, never larger than that term (it never drives the
    jaws open); within FF_OPEN_MARGIN of fully open no feedforward, the position command is pulled back toward the jaws
    instead. ``g_q``: output encoder, ``g_q_mot`` / ``g_qd``: motor feedback."""
    cfg = cfg or C.GripperContact()
    st = st if st is not None else Contact()
    q = C.gap_to_rad(goal_gap)
    tau_est = g_kp * (q - g_q_mot) + g_kd * (0.0 - g_qd)
    force = C.force_of_torque(max(tau_est, 0.0))
    tau_lim = C.torque_of_force(max_force)
    e_lim = tau_lim / g_kp if g_kp > 0 else math.inf
    tau_pos = g_kp * (q - g_q_mot)                         # the position term: what squeezes at rest
    capped = detect(st, now, q - g_q_mot, e_lim, g_q_mot, g_qd, cfg)
    ff, limiting = 0.0, False
    if capped and tau_pos > tau_lim:
        # the cap acts on the position term; the damping term stays physical (-kd * qd: it brakes the jaws
        # running into the object, and a feedforward cancelling it would come back as a spike when they stop)
        limiting = True
        if C.rad_to_gap(g_q) > C.GAP_MAX - C.FF_OPEN_MARGIN:
            q = min(q, g_q_mot + tau_lim / g_kp)          # closing is +rad
        else:
            ff = -(tau_pos - tau_lim)                      # opposes closing; |ff| < tau_pos
    # what is applied: the MIT law with the position actually sent, plus the feedforward
    applied = C.force_of_torque(max(g_kp * (q - g_q_mot) + g_kd * (0.0 - g_qd) + ff, 0.0))
    return GripCommand(q, g_kp, g_kd, ff, tau_est, force, applied, limiting, st.on)


def free(g_q_mot: float) -> GripCommand:
    """Gripper free (gainless): the command follows the motor."""
    return GripCommand(g_q_mot, 0.0, 0.0, 0.0, math.nan, math.nan, math.nan, False)
