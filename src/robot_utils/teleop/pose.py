"""Turn an engaged hand's relative motion into an absolute pose target for something the
robot can measure (an end effector, a tool frame, a camera, a mobile base...).

    clutch = PoseClutch(operator_to_robot=R_base_from_operator, max_offset_m=0.5)
    ...
    target = clutch.update(inputs.hands["right"], measure=lambda: my_fk())
    if target is not None:
        drive_toward(target)

On each new engagement the anchor is measured once (``measure`` is only called then), and
the target is ``(anchor.position + dp, dR @ anchor.rotation)`` with the operator's deltas
rotated into the robot's frame. While the hand is not engaged the target is None: what
"hold" means is up to the robot.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Union

import numpy as np

from .clutch import HandState


@dataclass(frozen=True)
class Pose:
    position: np.ndarray   # (3,)
    rotation: np.ndarray   # (3, 3)

    @staticmethod
    def identity() -> "Pose":
        return Pose(np.zeros(3), np.eye(3))

    @staticmethod
    def from_matrix(t: np.ndarray) -> "Pose":
        t = np.asarray(t, dtype=float)
        return Pose(t[:3, 3].copy(), t[:3, :3].copy())

    def matrix(self) -> np.ndarray:
        t = np.eye(4)
        t[:3, :3], t[:3, 3] = self.rotation, self.position
        return t


Measure = Union[Pose, Callable[[], Pose]]


class PoseClutch:
    """Anchor-and-delta pose target for one hand.

    ``operator_to_robot``: rotation taking operator-frame vectors (x forward, y left, z up)
    into the frame ``measure`` reports in. ``max_offset_m`` / ``max_angle_rad`` clamp how
    far the target may move from the anchor (None: no clamp beyond the app's own limit).
    """

    def __init__(self, operator_to_robot: Optional[np.ndarray] = None, *,
                 max_offset_m: Optional[float] = None, max_angle_rad: Optional[float] = None):
        r = np.eye(3) if operator_to_robot is None else np.asarray(operator_to_robot, dtype=float)
        if r.shape != (3, 3) or not np.allclose(r.T @ r, np.eye(3), atol=1e-6) \
                or not np.isclose(np.linalg.det(r), 1.0, atol=1e-6):
            raise ValueError("operator_to_robot must be a 3x3 rotation matrix")
        self.operator_to_robot = r
        self.max_offset_m = max_offset_m
        self.max_angle_rad = max_angle_rad
        self.anchor: Optional[Pose] = None
        self.target: Optional[Pose] = None
        self._engagement: Optional[int] = None

    def reset(self) -> None:
        self.anchor = self.target = self._engagement = None

    def update(self, hand: HandState, measure: Measure) -> Optional[Pose]:
        if not hand.engaged:
            self.reset()
            return None
        if hand.engagement != self._engagement:
            anchor = measure() if callable(measure) else measure
            if anchor is None:
                raise ValueError("measure returned None at engagement")
            _check_pose(anchor)
            self.anchor, self._engagement = anchor, hand.engagement
        r = self.operator_to_robot
        dp = r @ hand.delta_position
        dr = r @ hand.delta_rotation @ r.T
        if self.max_offset_m is not None:
            n = float(np.linalg.norm(dp))
            if n > self.max_offset_m:
                dp = dp * (self.max_offset_m / n)
        if self.max_angle_rad is not None:
            dr = clamp_rotation(dr, self.max_angle_rad)
        self.target = Pose(self.anchor.position + dp, dr @ self.anchor.rotation)
        return self.target


def _check_pose(p: Pose) -> None:
    pos, rot = np.asarray(p.position), np.asarray(p.rotation)
    if pos.shape != (3,) or rot.shape != (3, 3) or not (np.isfinite(pos).all() and np.isfinite(rot).all()):
        raise ValueError(f"measured anchor is not a finite pose: {p!r}")


def rotation_vector(r: np.ndarray) -> np.ndarray:
    """Axis * angle (rad, in [0, pi]) of a rotation matrix."""
    q = _quaternion(r)
    w, v = q[0], q[1:]
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        return np.zeros(3)
    return v / s * 2.0 * np.arctan2(s, w)


def rotation_from_vector(rv: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return np.eye(3)
    k = np.asarray(rv, dtype=float) / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * kx @ kx


def clamp_rotation(r: np.ndarray, max_angle_rad: float) -> np.ndarray:
    rv = rotation_vector(r)
    a = float(np.linalg.norm(rv))
    return r if a <= max_angle_rad else rotation_from_vector(rv * (max_angle_rad / a))


def _quaternion(r: np.ndarray) -> np.ndarray:
    """(w, x, y, z), w >= 0."""
    m = np.asarray(r, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q
