"""The operator's elbow swivel, the teleop half of the relative elbow-swivel hint: input only. Pure.

Per engaged hand, from Meta body tracking (``body.py``, quest_env BODY_TRACKING.md), for the arm the hand is paired
to (mirror pairing, ``body_frozen.py``), the datagram's EE carries an optional ``swivel`` object::

    "swivel": {"seq": 17000123, "dpsi": 0.31, "elbow": 2.05, "conf": 0.9}

- ``seq``: new on every engagement of the hand paired to this EE (``SeqCounter``): the WBC re-anchors the robot's
  swivel when it changes. Not the headset's ``engagement_id``: that counts per hand from 0 and restarts with each
  XR session, so after a mirror swap or a new session an EE could see an id it has already seen.
- ``dpsi``: rad, the operator's swivel change since that engagement, **positive = the elbow moving out** (away from
  the body), for either arm: the WBC's convention (``wbc/swivel_geom.py``: +1 for the left arm, -1 for the right),
  so out maps to out whatever the pairing (mirror needs no sign: the operator's left elbow out drives the robot's
  right elbow out, its mirror image).
- ``elbow``: rad, the operator's upper-arm–forearm angle (π = straight).
- ``conf``: 0..1, Meta's body confidence (``arm_sample``). The joints' position-tracked flags are not folded in:
  with controllers in hand Meta infers the arms and never marks them tracked (a VR session, 2026-11-25: confidence
  1.0, shoulder, elbow and wrist valid and inferred in every one of 430 samples), so halving per inferred joint gave
  0.125 always, under the WBC's 0.3 fade: the hint never acted. ``tracked`` is still reported.

Omitted when the frame has no body block (or an inactive one) or the arm's joints are not valid. The WBC fades,
smooths and anchors the hint to the robot: nothing of that here.

**The joints** (XR_FB body joint set, ids 0–17 and the hand joints, the same in both sets):

- the shoulder: ``<side>_arm_upper`` (ids 10 / 15), the upper arm bone's root, i.e. the glenohumeral joint (the
  shoulder centre). ``<side>_shoulder`` and ``<side>_scapula`` are the clavicle's and the scapula's roots, on the
  trunk: they move with shrugs, not with the arm;
- the elbow: ``<side>_arm_lower`` (11 / 16, BODY_TRACKING.md "the elbow");
- the wrist: ``<side>_hand_wrist`` (19 / 45; in the ``core`` subset). The body's own wrist, not the controller
  shifted to the wrist: the three points then come from one solver and one sample time (``body.time_ns``, which can
  lag the frame's controller sample), so a body lag cannot read as a swivel; and the controller-to-wrist offset
  depends on how the controller is held. Meta's solver places the wrist from the controller anyway.

**The swivel change** is the elbow's rotation about the shoulder → wrist axis, measured right-handed and then signed
by the operator's arm (``SIDE_SIGN``: +1 left, -1 right, so positive is out for both), accumulated frame by frame
by parallel transport: the previous
elbow direction (the elbow's offset from the axis, normalised) is carried to the new axis by the minimal rotation
between the two axes, and the step is the signed angle from it to the new elbow direction about the new axis. No
fixed reference direction (a "down" reference is singular with the arm hanging or raised vertically): the hanging
arm swivels like any other. Each step is in (-π, π]; the sum is unbounded.

Undefined steps are bridged, not reset: while the elbow is within ``MIN_PERP_M`` of the axis (the arm nearly
straight: no swivel to measure) or the joints are not valid, the last defined direction is kept and the next
defined sample steps from it, so ``dpsi`` holds meanwhile and then continues by the rotation actually made (a short
body dropout, ``--body-every``, an arm straightened and bent again). An axis reversed between two defined samples
(> ``MAX_AXIS_STEP``) has no minimal rotation: the reference restarts there (``dpsi`` holds; counted in
``restarts``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import body as B

JOINTS = {side: (f"{side}_arm_upper", f"{side}_arm_lower", f"{side}_hand_wrist") for side in ("left", "right")}
MIN_AXIS_M = 0.05          # shoulder–wrist distance below this: no axis
MIN_PERP_M = 0.02          # the elbow this close to the axis (elbow angle within ~8° of straight): no direction
MAX_AXIS_STEP = math.radians(150)   # the axis turned more than this since the last defined sample: restart
SIDE_SIGN = {"left": 1.0, "right": -1.0}   # right-handed angle x this = positive out (the WBC's swivel_geom sign)


@dataclass
class ArmSample:
    """One frame's operator arm, in LOCAL (metres)."""
    valid: bool
    reason: str = ""
    shoulder: Optional[np.ndarray] = None
    elbow: Optional[np.ndarray] = None
    wrist: Optional[np.ndarray] = None
    elbow_angle: Optional[float] = None   # rad, π straight
    conf: float = 0.0
    tracked: int = 0                      # of the three joints, how many are position-tracked

    def summary(self) -> dict:
        return {"valid": self.valid, "reason": self.reason or None,
                "elbow": None if self.elbow_angle is None else round(self.elbow_angle, 4),
                "conf": round(self.conf, 3) if self.valid else None, "tracked": self.tracked if self.valid else None}


def arm_sample(body: Optional["B.Body"], side: str) -> ArmSample:
    """The operator's ``side`` arm from a parsed body block: valid when the block is available and active and the
    shoulder, elbow and wrist positions are valid (flags) and finite, with a shoulder–wrist distance of at least
    ``MIN_AXIS_M``. ``conf`` = the body's confidence (the tracked flags are reported, not used: module docstring)."""
    if body is None:
        return ArmSample(False, "no body block")
    if not body.available:
        return ArmSample(False, f"body unavailable: {body.reason}" if body.reason else "body unavailable")
    if not body.active:
        return ArmSample(False, "body inactive")
    pts, tracked = [], 0
    for name in JOINTS[side]:
        j = body.joint(name)
        if j is None:
            return ArmSample(False, f"{name} not sent")
        p, _, flags = j
        if not flags & B.POSITION_VALID or not np.all(np.isfinite(p)):
            return ArmSample(False, f"{name} not valid")
        tracked += bool(flags & B.POSITION_TRACKED)
        pts.append(np.asarray(p, float).copy())
    s, e, w = pts
    if np.linalg.norm(w - s) < MIN_AXIS_M:
        return ArmSample(False, "wrist at the shoulder")
    u, f = s - e, w - e
    nu, nf = np.linalg.norm(u), np.linalg.norm(f)
    if nu < 1e-4 or nf < 1e-4:
        return ArmSample(False, "degenerate arm")
    angle = math.acos(max(-1.0, min(1.0, float(u @ f) / (nu * nf))))
    conf = max(0.0, min(1.0, float(body.confidence)))
    return ArmSample(True, "", s, e, w, angle, conf, tracked)


def _axis_dir(a: ArmSample):
    """(the unit shoulder → wrist axis, the unit elbow direction off it, or None when too close to the axis)."""
    ax = a.wrist - a.shoulder
    ax = ax / np.linalg.norm(ax)
    off = (a.elbow - a.shoulder) - float((a.elbow - a.shoulder) @ ax) * ax
    n = float(np.linalg.norm(off))
    return ax, (off / n if n >= MIN_PERP_M else None)


def transport(e0: np.ndarray, a0: np.ndarray, a1: np.ndarray) -> np.ndarray:
    """``e0`` (a unit vector ⟂ ``a0``) carried to ``a1`` by the minimal rotation taking ``a0`` to ``a1``
    (``1 + a0·a1`` must not vanish)."""
    v = e0 - float(e0 @ a1) / (1.0 + float(a0 @ a1)) * (a0 + a1)
    v = v - float(v @ a1) * a1
    return v / np.linalg.norm(v)


class SwivelTracker:
    """One hand's swivel change since its engagement (module docstring). ``start(seq)`` at an engagement,
    ``update(sample)`` every frame while engaged, ``stop()`` at the release."""

    def __init__(self):
        self.seq: Optional[int] = None
        self.dpsi = 0.0
        self.ref: Optional[tuple] = None      # the last defined (axis, elbow direction)
        self.straight = False                 # the newest valid sample had no elbow direction
        self.restarts = 0

    @property
    def active(self) -> bool:
        return self.seq is not None

    def start(self, seq: int) -> None:
        self.seq, self.dpsi, self.ref, self.straight = int(seq), 0.0, None, False

    def stop(self) -> None:
        self.seq, self.ref, self.straight = None, None, False

    def update(self, a: ArmSample) -> None:
        if self.seq is None or not a.valid:
            return
        ax, e = _axis_dir(a)
        self.straight = e is None
        if e is None:
            return
        if self.ref is not None:
            a0, e0 = self.ref
            if float(a0 @ ax) < math.cos(MAX_AXIS_STEP):
                self.restarts += 1
            else:
                et = transport(e0, a0, ax)
                self.dpsi += math.atan2(float(ax @ np.cross(et, e)), float(et @ e))
        self.ref = (ax, e)


class SeqCounter:
    """Engagement sequence numbers for the swivel object: strictly increasing, and seeded from the clock (``now``
    in ms) so a restarted client does not reuse the previous run's numbers. One per client (MultiCore shares one
    between its hands)."""

    def __init__(self):
        self.last = 0

    def next(self, now: float) -> int:
        ms = int(now * 1e3) if math.isfinite(now) else 0
        self.last = max(self.last + 1, ms)
        return self.last
