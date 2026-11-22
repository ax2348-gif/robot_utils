"""Frame math for the Quest client (numpy only).

Conventions (quest_env DEMO.md, "Frame mapping"):

- The headset reports poses in OpenXR LOCAL (X right, Y up, -Z forward) as 16 row-major numbers of a 4x4
  transform (translation in elements 3, 7, 11; quest_env native/telemetry.cpp ``appendMatrixJson``).
- Operator and robot frames are FLU (X forward, Y left, Z up). LOCAL maps to operator FLU through
  ``C = [[0,0,-1],[-1,0,0],[0,1,0]]``: a LOCAL vector v is ``C @ v`` in operator axes, a LOCAL orientation R is
  ``C @ R`` (the native ``teleop::operatorPose``).
- The gravity-aligned head yaw: the head's forward axis (LOCAL -Z) mapped through C, projected on the floor;
  ``atan2(f_y, f_x)``. Undefined when looking almost straight up or down (horizontal part < 0.15, as native).

A pose is ``(p, R)``: a 3-vector and a 3x3 rotation. Quaternions are ``[x, y, z, w]`` (the WBC contract's order).
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

C = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # OpenXR LOCAL -> operator FLU
MIN_HORIZONTAL = 0.15   # native: |forward_xy| below this does not define a heading


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_axis(axis: Sequence[float], angle: float) -> np.ndarray:
    """Rodrigues: rotation by ``angle`` about ``axis``."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def wrap(angle: float) -> float:
    """Angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def pose_from_matrix16(m: Optional[Sequence[float]]):
    """``(p, R)`` from the headset's 16 row-major numbers; None if missing, the wrong size, non-finite or not a
    rotation (|R^T R - I| > 1e-2; the values are printed with a few decimals)."""
    if m is None:
        return None
    try:
        a = np.asarray(m, float)
    except (TypeError, ValueError):
        return None
    if a.shape != (16,) or not np.all(np.isfinite(a)):
        return None
    t = a.reshape(4, 4)
    r = t[:3, :3]
    if np.abs(r.T @ r - np.eye(3)).max() > 1e-2:
        return None
    return t[:3, 3].copy(), orthonormalize(r)


def matrix16_from_pose(p, r) -> list[float]:
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = p
    return [float(x) for x in t.reshape(-1)]


def orthonormalize(r) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.asarray(r, float))
    q = u @ vt
    if np.linalg.det(q) < 0:
        u[:, -1] *= -1
        q = u @ vt
    return q


def head_yaw(head_r_local) -> Optional[float]:
    """Gravity-aligned yaw of the head (operator FLU axes about Z up), from its LOCAL orientation; None when
    looking almost straight up or down."""
    f = C @ (np.asarray(head_r_local, float) @ np.array([0.0, 0.0, -1.0]))
    if math.hypot(f[0], f[1]) < MIN_HORIZONTAL:
        return None
    return math.atan2(f[1], f[0])


class OperatorFrame:
    """An upright operator frame captured at one engagement: origin = the head position, X = the head's
    gravity-aligned forward, Z = up. ``mapping = Rz(-yaw) @ C`` takes LOCAL vectors into it (as native
    ``capturedFromQuest``)."""

    def __init__(self, head_p_local, yaw: float):
        self.origin = np.asarray(head_p_local, float).copy()
        self.yaw = float(yaw)
        self.mapping = rot_z(-self.yaw) @ C

    @classmethod
    def from_head(cls, head_p_local, head_r_local) -> Optional["OperatorFrame"]:
        yaw = head_yaw(head_r_local)
        return None if yaw is None else cls(head_p_local, yaw)

    def pose(self, p_local, r_local):
        """A LOCAL pose expressed in this frame."""
        return self.mapping @ (np.asarray(p_local, float) - self.origin), self.mapping @ np.asarray(r_local, float)

    def vector(self, v_local) -> np.ndarray:
        return self.mapping @ np.asarray(v_local, float)


class PlanarFrame:
    """An upright robot frame in the world: position (x, y, z) and yaw (the platform's heading)."""

    def __init__(self, x: float, y: float, yaw: float, z: float = 0.0):
        self.p = np.array([x, y, z], float)
        self.yaw = float(yaw)
        self.r = rot_z(self.yaw)

    def to_local(self, p_world, r_world):
        return self.r.T @ (np.asarray(p_world, float) - self.p), self.r.T @ np.asarray(r_world, float)

    def to_world(self, p_local, r_local):
        return self.p + self.r @ np.asarray(p_local, float), self.r @ np.asarray(r_local, float)


def delta(anchor, current):
    """``(dp, dR)`` from an anchor pose to the current pose in one frame: ``dp = p - p0`` and the spatial
    rotation ``dR = R R0^T``."""
    return current[0] - anchor[0], current[1] @ anchor[1].T


def apply_delta(anchor, dp, dr):
    """The body-frozen target in the anchor's frame: ``(p0 + dp, dR R0)``."""
    return anchor[0] + dp, dr @ anchor[1]


# -- quaternions ([x, y, z, w]) ------------------------------------------------------------------

def quat_from_matrix(r) -> np.ndarray:
    r = np.asarray(r, float)
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = [0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def matrix_from_quat(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotation_angle(r) -> float:
    return math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
