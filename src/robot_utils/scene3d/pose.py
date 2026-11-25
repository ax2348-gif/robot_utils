"""Pose arithmetic for rigid scenes: [x, y, z, qx, qy, qz, qw] (quaternions x, y, z, w)."""
from __future__ import annotations

import numpy as np


def qmul(a, b) -> np.ndarray:
    """Quaternion product a * b, both x, y, z, w."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def qrot(q, v) -> np.ndarray:
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def compose(a, b) -> np.ndarray:
    """Pose a then b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return np.concatenate([a[:3] + qrot(a[3:], b[:3]), qmul(a[3:], b[3:])])


def inverse(a) -> np.ndarray:
    a = np.asarray(a, float)
    qi = np.array([-a[3], -a[4], -a[5], a[6]]) / max(float(np.dot(a[3:], a[3:])), 1e-12)
    return np.concatenate([-qrot(qi, a[:3]), qi])


def angle(q) -> float:
    """The rotation angle (rad) of a quaternion."""
    q = np.asarray(q, float)
    q = q / max(float(np.linalg.norm(q)), 1e-12)
    return float(2 * np.arctan2(np.linalg.norm(q[:3]), abs(q[3])))
