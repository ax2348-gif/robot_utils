"""Camera motion between the depth and the colour capture times: pure numpy, no I/O.

The registration needs ``M`` = the pose change of the **depth camera frame** from the depth
frame's time t_d to the colour frame's time t_c:

    p(t_c) = M · p(t_d),   M = T_wd(t_c)^-1 · T_wd(t_d)

(T_wd = pose of the depth camera in some world frame). Sources, best first:

- **rotation** from the camera's own IMU attitude filter: orientation q(t) (IMU -> gravity-aligned
  world, ~100 Hz, ``QuatTrack``, slerp-interpolated). With R_di = IMU -> depth rotation (the
  camera's factory ``imu.accel_to_depth``), R_wd = R_wi · R_di^T and
  R_rel = R_di · R_wi(t_c)^T · R_wi(t_d) · R_di^T. Needs no base calibration.
- **translation** from the estimator's planar base pose (x, y, theta, ~100 Hz, ``PoseTrack``,
  linear interpolation) and the depth camera's pose in the base T_bd (description node):
  T_wd = T_wb · T_bd, so M_base = T_bd^-1 · T_wb(t_c)^-1 · T_wb(t_d) · T_bd.

Modes (``Compensator.relative``):

| mode | rotation | translation | needs |
|---|---|---|---|
| ``full`` | IMU | base (M_base's) | IMU, estimator, calibrated T_bd |
| ``base`` | base (planar) | base | estimator, T_bd (no IMU) |
| ``rotation`` | IMU | none | IMU |
| ``none`` | identity | none | nothing |
"""
from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np

from .register import inv, rigid


# -- quaternions (w, x, y, z), Hamilton, active rotation -------------------------------------

def quat_normalize(q) -> np.ndarray:
    q = np.asarray(q, np.float64)
    return q / np.linalg.norm(q)


def quat_to_R(q) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_from_R(R) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z), w >= 0."""
    R = np.asarray(R, np.float64)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = quat_normalize(q)
    return -q if q[0] < 0 else q


def quat_from_axis_angle(axis, angle) -> np.ndarray:
    a = np.asarray(axis, np.float64)
    a = a / np.linalg.norm(a)
    return np.concatenate([[math.cos(angle / 2)], a * math.sin(angle / 2)])


def quat_mul(a, b) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def slerp(q0, q1, a: float) -> np.ndarray:
    """Spherical interpolation (shortest arc); a outside [0, 1] extrapolates at constant rate."""
    q0, q1 = quat_normalize(q0), quat_normalize(q1)
    if float(np.dot(q0, q1)) < 0:
        q1 = -q1
    dq = quat_mul(q0 * [1, -1, -1, -1], q1)          # q0^-1 q1
    n = float(np.linalg.norm(dq[1:]))
    if n < 1e-15:
        return q0
    angle = 2.0 * math.atan2(n, dq[0])
    return quat_normalize(quat_mul(q0, quat_from_axis_angle(dq[1:] / n, a * angle)))


def rotation_angle(R) -> float:
    return math.acos(max(-1.0, min(1.0, (np.trace(R[:3, :3]) - 1) / 2)))


def rz(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def base_T(x: float, y: float, theta: float) -> np.ndarray:
    return rigid(rz(theta), [x, y, 0.0])


# -- time tracks ----------------------------------------------------------------------------

class _Track:
    """Samples sorted by time (appends in order; out-of-order samples are dropped)."""

    def __init__(self, seconds: float = 5.0, max_gap: float = 0.1, max_extrapolate: float = 0.05,
                 restart_gap: float = 1.0):
        self.seconds, self.max_gap, self.max_extrapolate = seconds, max_gap, max_extrapolate
        self.restart_gap = restart_gap
        self.t: deque = deque()
        self.v: deque = deque()

    def clear(self):
        self.t.clear()
        self.v.clear()

    @property
    def start(self) -> Optional[float]:
        return self.t[0] if self.t else None

    @property
    def end(self) -> Optional[float]:
        return self.t[-1] if self.t else None

    def covers(self, t: float) -> bool:
        return bool(self.t) and self.t[-1] >= t

    def _add(self, t: float, v) -> None:
        if self.t and t <= self.t[-1]:
            return
        if self.t and t - self.t[-1] > self.restart_gap:     # e.g. reconnected after idling: start afresh
            self.clear()
        self.t.append(t)
        self.v.append(v)
        while self.t and self.t[0] < t - self.seconds:
            self.t.popleft()
            self.v.popleft()

    def _bracket(self, t: float):
        """(i0, i1, a) with value = interp(v[i0], v[i1], a), or None if t is not covered.
        Inside: the samples around t (a gap > max_gap is not bridged). After the end: the last
        two samples extrapolated, up to max_extrapolate. Before the start: None."""
        n = len(self.t)
        if n == 0:
            return None
        ts = self.t
        if t > ts[-1]:
            if t - ts[-1] > self.max_extrapolate or n < 2:
                return (n - 1, n - 1, 0.0) if t - ts[-1] <= 1e-6 else None
            t0, t1 = ts[-2], ts[-1]
            if t1 - t0 > self.max_gap or t1 <= t0:
                return None
            return n - 2, n - 1, (t - t0) / (t1 - t0)
        if t < ts[0]:
            return None
        # binary search on the deque (indexing a deque is O(n) from the far end; n is ~500)
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        t0, t1 = ts[lo], ts[hi]
        if t1 - t0 > self.max_gap:
            return None
        return lo, hi, 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)


class QuatTrack(_Track):
    """IMU orientation (IMU -> world) over time; ``at(t)`` slerps."""

    def add(self, t: float, q) -> None:
        q = quat_normalize(q)
        if self.v and float(np.dot(self.v[-1], q)) < 0:
            q = -q                               # keep the sign continuous
        self._add(t, q)

    def at(self, t: float) -> Optional[np.ndarray]:
        b = self._bracket(t)
        if b is None:
            return None
        i0, i1, a = b
        return self.v[i0] if i0 == i1 else slerp(self.v[i0], self.v[i1], a)


class PoseTrack(_Track):
    """Planar base pose (x, y, theta continuous) over time; ``at(t)`` interpolates linearly."""

    def add(self, t: float, x: float, y: float, theta: float) -> None:
        self._add(t, np.array([x, y, theta], np.float64))

    def at(self, t: float) -> Optional[np.ndarray]:
        b = self._bracket(t)
        if b is None:
            return None
        i0, i1, a = b
        return self.v[i0] if i0 == i1 else self.v[i0] + a * (self.v[i1] - self.v[i0])


# -- the compensator -------------------------------------------------------------------------

class Compensator:
    """Holds the motion sources and computes M (depth frame at t_d -> depth frame at t_c)."""

    MODES = ("full", "base", "rotation", "none")

    def __init__(self, imu_seconds: float = 5.0, pose_seconds: float = 5.0):
        self.imu = QuatTrack(imu_seconds, max_gap=0.1, max_extrapolate=0.05)
        self.pose = PoseTrack(pose_seconds, max_gap=0.1, max_extrapolate=0.1)
        self.R_di = np.eye(3)                       # IMU -> depth rotation (camera /calibration)
        self.T_bd: Optional[np.ndarray] = None      # depth camera in base (description); None = unknown
        self.use_imu = True
        self.use_pose = True

    def set_imu_to_depth(self, R) -> None:
        self.R_di = np.eye(3) if R is None else np.asarray(R, np.float64).reshape(3, 3)

    def set_base_to_depth(self, T) -> None:
        self.T_bd = None if T is None else np.asarray(T, np.float64).reshape(4, 4)

    def imu_rotation(self, t_d: float, t_c: float) -> Optional[np.ndarray]:
        """R_rel (depth frame): p(t_c) = R_rel p(t_d) for a static point, rotation only."""
        if not self.use_imu:
            return None
        qd, qc = self.imu.at(t_d), self.imu.at(t_c)
        if qd is None or qc is None:
            return None
        R_rel_imu = quat_to_R(qc).T @ quat_to_R(qd)       # R_wi(t_c)^T R_wi(t_d)
        return self.R_di @ R_rel_imu @ self.R_di.T

    def base_motion(self, t_d: float, t_c: float) -> Optional[np.ndarray]:
        """M_base (4x4) from the planar base poses and T_bd, or None."""
        if not self.use_pose or self.T_bd is None:
            return None
        pd, pc = self.pose.at(t_d), self.pose.at(t_c)
        if pd is None or pc is None:
            return None
        return inv(self.T_bd) @ inv(base_T(*pc)) @ base_T(*pd) @ self.T_bd

    def relative(self, t_d: float, t_c: float) -> tuple[np.ndarray, str]:
        """(M, mode). M: 4x4, p_depth(t_c) = M · p_depth(t_d)."""
        R = self.imu_rotation(t_d, t_c)
        Mb = self.base_motion(t_d, t_c)
        if R is not None and Mb is not None:
            return rigid(R, Mb[:3, 3]), "full"
        if Mb is not None:
            return Mb, "base"
        if R is not None:
            return rigid(R, np.zeros(3)), "rotation"
        return np.eye(4), "none"

    def summary(self, M: np.ndarray) -> dict:
        return {"rotation_deg": round(math.degrees(rotation_angle(M)), 4),
                "translation_mm": round(float(np.linalg.norm(M[:3, 3])) * 1000.0, 3)}
