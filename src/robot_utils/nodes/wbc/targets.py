"""Per end effector target buffers: a timestamped world-frame trajectory, reactive or chunked
(docs/plans/wbc_v1.md "Targets").

- **Reactive**: one sample; the newest wins; its twist is the feedforward while fresh.
- **Chunk**: a sequence that replaces the buffer; samples already in the past (the stale prefix) are dropped;
  it is blended in with a decaying, critically damped offset anchored at the *commanded* EE state (pose and
  velocity) at the moment it takes over, so the target is continuous in pose and velocity at every seam:
  ``B(t) = N(t) (+) (d0 + (dv + d0 / tau) s) exp(-s / tau)``, ``s = t - t_r``, ``d0 = P_cmd (-) N(t_r)``,
  ``dv = V_cmd - N'(t_r)``.
- **Staleness, two stages**: the feedforward fades from ``ff_timeout`` to 0 at twice that (the pose holds); the
  node's ``hold_timeout`` (no datagram from the client) stops the base and holds the arm.

Everything here runs on the control thread (the receiver hands parsed messages over through the node's inbox),
so no locking. Poses: position (3) + rotation matrix; twists [v; w] in the world axes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pinocchio as pin


def quat_to_R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not (n > 1e-9 and math.isfinite(n)):
        raise ValueError("quaternion must be finite and not zero")
    return pin.Quaternion(w / n, x / n, y / n, z / n).toRotationMatrix()


def R_to_quat(R) -> np.ndarray:
    q = pin.Quaternion(np.asarray(R))
    q.normalize()
    v = np.array([q.x, q.y, q.z, q.w])
    return v if v[3] >= 0 else -v


def pose_diff(pa, Ra, pb, Rb) -> np.ndarray:
    """a (-) b: [pa - pb; log3(Ra Rb^T)] (world axes)."""
    return np.concatenate([pa - pb, pin.log3(Ra @ Rb.T)])


def pose_add(p, R, d) -> tuple[np.ndarray, np.ndarray]:
    """(p, R) (+) d: shifted by d[:3], rotated by exp3(d[3:]) on the left (world axes)."""
    return p + d[:3], pin.exp3(d[3:]) @ R


@dataclass
class Target:
    """The target of one tick."""
    p: np.ndarray
    R: np.ndarray
    v: np.ndarray            # 6, world axes (feedforward, already scaled by ff_scale)
    ff_scale: float
    gripper: Optional[float]
    age: float               # s since the sample in force (reactive) / past the chunk's end (0 inside)
    kind: str
    id: int


class TargetBuffer:
    def __init__(self, name: str, ff_timeout: float = 0.1, blend_tau: float = 0.15):
        self.name = name
        self.ff_timeout = ff_timeout
        self.blend_tau = blend_tau
        self.kind: Optional[str] = None       # None | reactive | chunk
        self.id = 0
        self.reset_id = None
        self.gripper: Optional[float] = None  # the latest gripper value given (persists across targets)
        # reactive
        self._t = 0.0
        self._p = self._R = self._v = None
        # chunk
        self._ct: Optional[np.ndarray] = None
        self._cp = self._cR = self._cv = self._cg = None
        self._fd = False
        self._t_r = 0.0
        self._d0 = np.zeros(6)
        self._dv = np.zeros(6)

    # -- replace -----------------------------------------------------------------------------------------

    def clear(self) -> None:
        self.kind = None

    def set_reactive(self, t: float, p, R, v=None, gripper=None, reset_id=None) -> int:
        self.kind, self.reset_id = "reactive", reset_id
        self._t, self._p, self._R = float(t), np.asarray(p, float), np.asarray(R, float)
        self._v = np.zeros(6) if v is None else np.asarray(v, float)
        if gripper is not None:
            self.gripper = float(gripper)
        self.id += 1
        return self.id

    def set_chunk(self, now: float, t, P, Rs, V=None, G=None, *, anchor_p=None, anchor_R=None, anchor_v=None,
                  reset_id=None) -> int:
        """Replace with a chunk. ``t`` strictly increasing; samples before ``now`` are the stale prefix (dropped:
        the trajectory is evaluated from ``now``). ``anchor_*``: the commanded EE state now (None: no blend)."""
        t = np.asarray(t, float)
        P = np.asarray(P, float).reshape(len(t), 3)
        Rs = np.asarray(Rs, float).reshape(len(t), 3, 3)
        self._fd = V is None
        if V is None:                       # finite differences, constant per segment (the last sample: zero)
            V = np.zeros((len(t), 6))
            for i in range(len(t) - 1):
                V[i] = pose_diff(P[i + 1], Rs[i + 1], P[i], Rs[i]) / (t[i + 1] - t[i])
        keep = max(0, int(np.searchsorted(t, now, side="right")) - 1)     # the sample at or before now stays
        self._ct, self._cp, self._cR = t[keep:], P[keep:], Rs[keep:]
        self._cv = np.asarray(V, float).reshape(len(t), 6)[keep:]
        self._cg = None if G is None else np.asarray(G, float)[keep:]
        self.kind, self.reset_id = "chunk", reset_id
        self._t_r = float(now)
        self._d0, self._dv = np.zeros(6), np.zeros(6)
        if anchor_p is not None:
            pN, RN, vN = self._eval_raw(now)
            self._d0 = pose_diff(np.asarray(anchor_p, float), np.asarray(anchor_R, float), pN, RN)
            self._dv = (np.zeros(6) if anchor_v is None else np.asarray(anchor_v, float)) - vN
        self.id += 1
        return self.id

    # -- evaluate -----------------------------------------------------------------------------------------

    def _eval_raw(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        ts = self._ct
        if t <= ts[0]:
            return self._cp[0], self._cR[0], np.zeros(6) if len(ts) == 1 or t < ts[0] else self._cv[0]
        if t >= ts[-1]:
            return self._cp[-1], self._cR[-1], np.zeros(6)
        i = int(np.searchsorted(ts, t, side="right")) - 1
        a = (t - ts[i]) / (ts[i + 1] - ts[i])
        p = self._cp[i] + a * (self._cp[i + 1] - self._cp[i])
        R = self._cR[i] @ pin.exp3(a * pin.log3(self._cR[i].T @ self._cR[i + 1]))
        # velocity: the given samples' (linear in between) or the segment's finite difference
        v = self._cv[i] if self._fd else self._cv[i] + a * (self._cv[i + 1] - self._cv[i])
        return p, R, v

    def ff_scale(self, age: float) -> float:
        if age <= self.ff_timeout:
            return 1.0
        return max(0.0, 1.0 - (age - self.ff_timeout) / self.ff_timeout)

    def sample(self, now: float) -> Optional[Target]:
        if self.kind is None:
            return None
        if self.kind == "reactive":
            age = max(0.0, now - self._t)
            k = self.ff_scale(age)
            return Target(self._p, self._R, self._v * k, k, self.gripper, age, "reactive", self.id)
        p, R, v = self._eval_raw(now)
        s = now - self._t_r
        if s >= 0 and (np.any(self._d0) or np.any(self._dv)):
            tau = self.blend_tau
            e = math.exp(-s / tau)
            c1 = self._dv + self._d0 / tau
            off = (self._d0 + c1 * s) * e
            doff = (c1 - (self._d0 + c1 * s) / tau) * e
            p, R = pose_add(p, R, off)
            v = v + doff
        age = max(0.0, now - float(self._ct[-1]))
        k = self.ff_scale(age)
        g = self.gripper
        if self._cg is not None and len(self._cg):
            i = int(np.clip(np.searchsorted(self._ct, now, side="right") - 1, 0, len(self._cg) - 1))
            if math.isfinite(self._cg[i]):
                g = float(self._cg[i])
                self.gripper = g
        return Target(p, R, v * k, k, g, age, "chunk", self.id)
