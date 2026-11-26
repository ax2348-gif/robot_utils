"""The base hint's source candidates: the headset (``head``), the upper body (``chest``) and the pelvis (``hips``),
each as a planar pose of the operator in the headset's room, raw and smoothed (README "The base hint's source").

Link 1 of the body-frozen chain (``body_frozen.py``) moves the base reference by the operator's horizontal motion and
yaw since engagement. Historically that is the headset's (worn on the neck: a torso proxy that still nods, shifts and
turns with the head and neck). With body tracking (quest_env BODY_TRACKING.md) the chest or the pelvis can drive it
instead: the pelvis is where the operator walks, the chest where the upper body faces.

Per frame, for each candidate:

- ``p``: its position in OpenXR LOCAL (metres); only the horizontal part is used.
- ``yaw``: its gravity-aligned heading, **unwrapped** (continuous). The headset's is ``frames.head_yaw`` (its forward
  axis). A body joint's is the heading of one of its own axes: the joint frame's most horizontal axis, chosen the
  first time the joint is seen (and again if it ever tips near vertical). No Meta joint-axis convention is assumed:
  the heading's absolute value is meaningless, only its changes are used (link 1 is relative to an anchor). Height
  is never used, so the same code serves a headset worn on the neck or on the head.
- ``ok``: usable this frame: the head tracked, or the body active, confident (``min_confidence``) and the joint's
  position and orientation valid (and tracked, with ``require_tracked``), its chosen axis not near vertical.
- ``f_p`` / ``f_yaw``: smoothed. A first-order low-pass (time constant ``tau``, on the frames' ``sample_time_ns``),
  then a deadband: the output moves only once the low-passed value is more than ``deadband_m`` (position, in the
  plane) / ``deadband_rad`` (yaw) away from it, and then trails it by that distance (no step: a backlash). A
  candidate that was not ok restarts its filter at its next ok value.

The smoothing applies to the body candidates only when they drive the hint; the head drives it raw (the existing
behaviour), but all three are computed and logged (raw and smoothed) whichever is selected, for offline analysis
(``hint_analysis.py``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import body as B
from . import frames as F

SOURCES = ("head", "chest", "hips")
JOINTS = {"chest": "chest", "hips": "hips"}
UP_LOCAL = np.array([0.0, 1.0, 0.0])


@dataclass
class HintFilterConfig:
    tau: float = 0.15                 # s, low-pass time constant (position and yaw)
    deadband_m: float = 0.02          # m, in the plane
    deadband_rad: float = 0.03        # rad (1.7 degrees)
    min_confidence: float = 0.5       # body confidence below this: the candidate is not ok
    require_tracked: bool = True      # the joint's position and orientation must be tracked, not only valid


class Smoother:
    """Low-pass then backlash deadband on (x, y in the plane of LOCAL, yaw unwrapped)."""

    def __init__(self, cfg: HintFilterConfig):
        self.cfg = cfg
        self.lp: Optional[np.ndarray] = None       # x, z (LOCAL horizontal), yaw
        self.out: Optional[np.ndarray] = None
        self.t: Optional[float] = None

    def reset(self) -> None:
        self.lp = self.out = self.t = None

    def update(self, t: Optional[float], p_local, yaw: float) -> np.ndarray:
        x = np.array([p_local[0], p_local[2], yaw], float)
        if self.lp is None:
            self.lp, self.out, self.t = x.copy(), x.copy(), t
            return self.out.copy()
        dt = 0.0 if t is None or self.t is None else max(0.0, t - self.t)
        self.t = t if t is not None else self.t
        a = 1.0 if self.cfg.tau <= 0 else 1.0 - math.exp(-dt / self.cfg.tau)
        self.lp = self.lp + a * (x - self.lp)
        d = self.lp[:2] - self.out[:2]
        n = float(np.linalg.norm(d))
        if n > self.cfg.deadband_m:
            self.out[:2] += d * (1.0 - self.cfg.deadband_m / n)
        e = self.lp[2] - self.out[2]
        if abs(e) > self.cfg.deadband_rad:
            self.out[2] += e - math.copysign(self.cfg.deadband_rad, e)
        return self.out.copy()


@dataclass
class Sample:
    ok: bool
    reason: str = ""
    confidence: Optional[float] = None
    p: Optional[np.ndarray] = None          # LOCAL
    yaw: Optional[float] = None             # unwrapped
    f_p: Optional[np.ndarray] = None        # LOCAL (height from the raw sample)
    f_yaw: Optional[float] = None

    def record(self) -> dict:
        r = {"ok": self.ok}
        if self.reason:
            r["reason"] = self.reason
        if self.confidence is not None:
            r["confidence"] = round(self.confidence, 3)
        if self.p is not None:
            r["p"] = [round(float(v), 5) for v in self.p]
            r["yaw"] = round(float(self.yaw), 5)
            r["f_p"] = [round(float(v), 5) for v in self.f_p]
            r["f_yaw"] = round(float(self.f_yaw), 5)
        return r


class _Unwrap:
    def __init__(self):
        self.last: Optional[float] = None

    def __call__(self, yaw: float) -> float:
        self.last = yaw if self.last is None else self.last + F.wrap(yaw - self.last)
        return self.last


def _heading(v_local) -> Optional[float]:
    """The gravity-aligned heading of a LOCAL vector in operator FLU (as ``frames.head_yaw``), None if near
    vertical."""
    f = F.C @ np.asarray(v_local, float)
    h = math.hypot(f[0], f[1])
    return None if h < F.MIN_HORIZONTAL * float(np.linalg.norm(f)) else math.atan2(f[1], f[0])


class Candidate:
    def __init__(self, name: str, cfg: HintFilterConfig):
        self.name, self.cfg = name, cfg
        self.smoother = Smoother(cfg)
        self.unwrap = _Unwrap()
        self.axis: Optional[int] = None            # body joints: the column of the joint frame whose heading is used
        self.was_ok = False

    def _raw(self, frame: dict, body: Optional[B.Body]):
        if self.name == "head":
            head = frame.get("head") or {}
            if not (head.get("position_tracked") and head.get("orientation_tracked")):
                return "head not tracked", None, None, None
            pose = F.pose_from_matrix16(head.get("pose"))
            yaw = F.head_yaw(pose[1]) if pose is not None else None
            if yaw is None:
                return "head heading undefined", None, None, None
            return "", None, pose[0], yaw
        if body is None:
            return "no body", None, None, None
        if not body.available:
            return "body unavailable", None, None, None
        if not body.active:
            return "body not active", body.confidence, None, None
        j = body.joint(JOINTS[self.name])
        if j is None:
            return "joint not sent", body.confidence, None, None
        p, q, flags = j
        need = B.VALID | (B.TRACKED if self.cfg.require_tracked else 0)
        if flags & need != need:
            return f"joint flags {flags}", body.confidence, None, None
        if body.confidence < self.cfg.min_confidence:
            return f"confidence {body.confidence:.2f}", body.confidence, None, None
        r = F.matrix_from_quat(q)
        if self.axis is not None and _heading(r[:, self.axis]) is None:
            self.axis = None
        if self.axis is None:
            self.axis = int(np.argmin([abs(float(UP_LOCAL @ r[:, k])) for k in range(3)]))
            self.unwrap = _Unwrap()                # a new axis: a new (meaningless) zero
            self.smoother.reset()
        yaw = _heading(r[:, self.axis])
        if yaw is None:
            return "joint axis vertical", body.confidence, None, None
        return "", body.confidence, np.asarray(p, float), yaw

    def update(self, frame: dict, body: Optional[B.Body], t: Optional[float]) -> Sample:
        reason, conf, p, yaw = self._raw(frame, body)
        if p is None:
            self.was_ok = False
            return Sample(False, reason, conf)
        if not self.was_ok:
            self.smoother.reset()
        self.was_ok = True
        yu = self.unwrap(yaw)
        f = self.smoother.update(t, p, yu)
        return Sample(True, "", conf, np.asarray(p, float), yu, np.array([f[0], p[1], f[1]]), float(f[2]))


class HintSources:
    """All three candidates, updated per frame (``update`` returns name -> ``Sample``)."""

    def __init__(self, cfg: Optional[HintFilterConfig] = None):
        self.cfg = cfg or HintFilterConfig()
        self.cands = {n: Candidate(n, self.cfg) for n in SOURCES}
        self.latest: dict = {}

    def update(self, frame: dict, body: Optional[B.Body] = None) -> dict:
        if body is None and isinstance(frame.get("body"), dict):
            try:
                body = B.parse_body(frame)
            except B.BodyError:
                body = None
        ns = frame.get("sample_time_ns")
        t = ns * 1e-9 if isinstance(ns, int) and not isinstance(ns, bool) else None
        self.latest = {n: c.update(frame, body, t) for n, c in self.cands.items()}
        return self.latest
