"""Frame tree: a flat map of frames, each naming its parent. Pure numpy, no I/O.

A frame's pose is given in its parent: ``p_parent = R p_child + t``. Rotations are written as
``rpy`` (radians, fixed-axis X then Y then Z, as URDF: ``R = Rz(yaw) Ry(pitch) Rx(roll)``) or
as a 3x3 ``rotation``; translations ``xyz`` in metres.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Inverse of rpy_to_matrix (pitch in [-pi/2, pi/2])."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 1 - 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:   # gimbal lock: put everything in yaw
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def orthonormalize(R: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(R)
    M = u @ vt
    if np.linalg.det(M) < 0:
        u[:, -1] *= -1
        M = u @ vt
    return M


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def frame_T(f: dict) -> np.ndarray:
    """4x4 pose of a frame in its parent from a frame dict (rpy or rotation, xyz or translation)."""
    if "rotation" in f and f["rotation"] is not None:
        R = np.asarray(f["rotation"], dtype=float)
    else:
        R = rpy_to_matrix(*(f.get("rpy") or (0.0, 0.0, 0.0)))
    t = f.get("xyz", f.get("translation")) or (0.0, 0.0, 0.0)
    return make_T(R, t)


def wahba(body: list, ref: list, weights: Optional[list] = None) -> np.ndarray:
    """Rotation R minimising sum w |ref_i - R body_i|^2 (unit vectors): maps body -> ref."""
    w = np.ones(len(body)) if weights is None else np.asarray(weights, dtype=float)
    B = sum(wi * np.outer(np.asarray(r, float), np.asarray(b, float)) for wi, b, r in zip(w, body, ref))
    u, _, vt = np.linalg.svd(B)
    d = np.sign(np.linalg.det(u @ vt)) or 1.0
    return u @ np.diag([1.0, 1.0, d]) @ vt


class FrameTree:
    """Frames by name; ``T(a, b)`` is the 4x4 pose of frame b in frame a (``p_a = T p_b``)."""

    def __init__(self, frames: dict[str, dict]):
        self.frames = frames
        for name in frames:
            self._chain(name)   # validates: known parents, no cycles

    def _chain(self, name: str) -> list[str]:
        out, seen = [], set()
        while name is not None:
            if name not in self.frames:
                raise KeyError(f"unknown frame {name!r}")
            if name in seen:
                raise ValueError(f"cycle at frame {name!r}")
            seen.add(name)
            out.append(name)
            name = self.frames[name].get("parent")
        return out

    def to_root(self, name: str) -> np.ndarray:
        """Pose of ``name`` in its tree's root."""
        T = np.eye(4)
        for n in self._chain(name):
            if self.frames[n].get("parent") is not None:
                T = frame_T(self.frames[n]) @ T
        return T

    def root(self, name: str) -> str:
        return self._chain(name)[-1]

    def T(self, a: str, b: str) -> np.ndarray:
        if self.root(a) != self.root(b):
            raise ValueError(f"{a!r} and {b!r} are not in the same tree")
        return np.linalg.inv(self.to_root(a)) @ self.to_root(b)


def describe_T(T: np.ndarray) -> dict:
    r, p, y = matrix_to_rpy(T[:3, :3])
    return {"xyz": np.round(T[:3, 3], 6).tolist(), "rpy": [round(r, 7), round(p, 7), round(y, 7)],
            "rpy_deg": [round(math.degrees(v), 4) for v in (r, p, y)],
            "rotation": np.round(T[:3, :3], 9).tolist(), "matrix": np.round(T, 9).tolist()}
