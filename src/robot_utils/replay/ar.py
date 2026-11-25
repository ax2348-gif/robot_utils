"""The headset's frames, and where its AR view put the sim world (quest_env ``ROBOT_AR.md``, ``native/robot_ar.h``).

Frames:

- **LOCAL** (OpenXR): X right, Y up, -Z forward. A telemetry record's ``head.pose`` is the VIEW space (the centre
  eye: camera looking along its -Z, up its +Y) located in LOCAL; ``hands.<side>.pose`` the grip spaces in LOCAL;
  16 row-major numbers each (translation in elements 3, 7, 11).
- **LOCAL-FLU**: the same space in X forward, Y left, Z up axes: ``C @ v`` for a LOCAL vector (``teleop::questToBase``);
  a head's orientation is ``C @ R`` (the native ``teleop::operatorPose``).
- **The sim world** (z up, metres) is drawn in LOCAL at ``x_local = R (scale * x_sim) + p``: ``(p, R)`` is the
  ``sim_in_local`` of the placement (``C^T`` times the placed frame ``world()``: the origin 1.6 m ahead of the head
  along its gravity-aligned yaw, at the floor, plus ``--ar-offset`` / ``--ar-yaw``; ``scale`` is ``--ar-scale``).

Where the placement comes from:

- **logged** (new headsets): every telemetry record carries a ``robot_ar`` block (``placementJson`` in robot_ar.h,
  documented in ROBOT_AR.md "Logged placement") with ``sim_in_local``, ``scale``, the floor source and the eyes;
- **reconstructed** (older logs, ``Reconstruction``): ``Placement`` is a port of the native one, fed the logged head
  poses and the sim's ``reset_seq`` as recorded (the scene frames' receipt): placed at the **first tracked head in
  the log**, again at each ``reset_seq`` change (the first value is a baseline) and each XR session change, with
  the floor **estimated** 1.6 m below the head and offset / yaw / scale as given (the headset's config is not in the
  log). Exact only if the recording saw the first placement and the headset had no floor space; see docs/collect.md.

``Placement.block()`` writes the same block as the headset, so the fake headset (tools/quest_teleop/fake_headset.py)
emits what a real one logs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

C = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # LOCAL -> FLU (questToBase)
AHEAD_M = 1.6
FLOOR_BELOW_HEAD_M = 1.6

# Per-eye fields of view (rad, OpenXR XrFovf: left, right, up, down) for logs without the headset's own:
# approximately a Quest 3 / 3S eye (the left eye; the right one mirrored). Only used when no record has `views`.
DEFAULT_EYE_FOV = {"left": (-0.942, 0.698, 0.768, -0.873), "right": (-0.698, 0.942, 0.768, -0.873)}
DEFAULT_IPD_M = 0.063


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def matrix16(p, r) -> list:
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = p
    return [round(float(x), 6) for x in t.reshape(-1)]


def pose16(m) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """``(p, R)`` from 16 row-major numbers (rotation re-orthonormalised: the headset prints a few decimals); None
    if missing or malformed."""
    if m is None:
        return None
    try:
        a = np.asarray(m, float)
    except (TypeError, ValueError):
        return None
    if a.shape != (16,) or not np.all(np.isfinite(a)):
        return None
    t = a.reshape(4, 4)
    u, _, vt = np.linalg.svd(t[:3, :3])
    r = u @ vt
    if np.linalg.det(r) < 0:
        return None
    return t[:3, 3].copy(), r


def head_pose(frame: dict) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """The record's head (VIEW in LOCAL) when tracked, else None."""
    h = frame.get("head") if isinstance(frame.get("head"), dict) else None
    if not h or not (h.get("position_tracked") and h.get("orientation_tracked")):
        return None
    return pose16(h.get("pose"))


def hand_pose(frame: dict, side: str) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """A controller's grip pose in LOCAL when tracked (active, position and orientation tracked), else None."""
    hands = frame.get("hands") if isinstance(frame.get("hands"), dict) else {}
    h = hands.get(side) if isinstance(hands.get(side), dict) else None
    if not h or not h.get("active") or not (h.get("position_tracked") and h.get("orientation_tracked")):
        return None
    return pose16(h.get("pose"))


def flu(p, r) -> tuple[np.ndarray, np.ndarray]:
    """A LOCAL pose in LOCAL-FLU axes (``teleop::operatorPose``)."""
    return C @ np.asarray(p, float), C @ np.asarray(r, float)


# -- the native placement, ported ---------------------------------------------------------------------------------

@dataclass
class Placement:
    placed: bool = False
    session: Optional[str] = None
    origin_p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    origin_r: np.ndarray = field(default_factory=lambda: np.eye(3))
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.0
    scale: float = 1.0
    seq_seen: bool = False
    pending: bool = False
    pending_why: str = "A"
    last_seq: int = 0
    floor_estimated: bool = False
    placements: int = 0
    reason: str = ""
    placed_seq: Optional[int] = None
    floor_z: float = 0.0

    @staticmethod
    def head_yaw(head_flu_r) -> np.ndarray:
        """Gravity-aligned yaw of a head orientation in LOCAL-FLU (``Placement::headYaw``)."""
        f = head_flu_r @ np.array([0.0, 0.0, -1.0])
        if math.hypot(f[0], f[1]) < 0.15:
            f = head_flu_r @ np.array([0.0, 1.0, 0.0]) * (1.0 if f[2] < 0 else -1.0)
        return rot_z(math.atan2(f[1], f[0]))

    def reset(self, head_p, heading, floor: Optional[float] = None, why: str = "reset") -> None:
        ahead = np.asarray(head_p, float) + heading @ np.array([AHEAD_M, 0.0, 0.0])
        self.floor_z = float(floor) if floor is not None else float(head_p[2]) - FLOOR_BELOW_HEAD_M
        self.origin_p = np.array([ahead[0], ahead[1], self.floor_z])
        self.origin_r = np.asarray(heading, float)
        self.floor_estimated = floor is None
        self.placed = True
        self.placements += 1
        self.reason = why
        self.placed_seq = self.last_seq if self.seq_seen else None

    def update(self, session, head_flu: Optional[tuple], reset_seq: Optional[int] = None, a_press: bool = False,
               floor: Optional[float] = None) -> bool:
        """One headset frame (``Placement::update``): ``head_flu`` the head in LOCAL-FLU, None when untracked. True
        when it re-anchored on a reset."""
        valid = head_flu is not None
        if not self.placed or session != self.session:
            self.placed = False
            if valid:
                self.reset(head_flu[0], self.head_yaw(head_flu[1]), floor, "session" if self.placements else "first")
            self.session = session
        if reset_seq is not None:
            if not self.seq_seen:
                self.seq_seen, self.last_seq = True, int(reset_seq)
                if self.placed:
                    self.placed_seq = self.last_seq
            elif int(reset_seq) != self.last_seq:
                self.last_seq, self.pending, self.pending_why = int(reset_seq), True, "reset_seq"
        elif a_press and valid:
            self.pending, self.pending_why = True, "A"
        if not self.pending or not valid:
            return False
        self.pending = False
        self.reset(head_flu[0], self.head_yaw(head_flu[1]), floor, self.pending_why)
        return True

    def world(self) -> tuple[np.ndarray, np.ndarray]:
        return self.origin_p + self.origin_r @ self.offset, self.origin_r @ rot_z(self.yaw)

    def sim_in_local(self) -> tuple[np.ndarray, np.ndarray]:
        p, r = self.world()
        return C.T @ p, C.T @ r

    def block(self, floor_space: str = "LOCAL_FLOOR", views: Optional[list] = None) -> dict:
        """The telemetry record's ``robot_ar`` block, as the headset writes it."""
        p, r = self.sim_in_local()
        return {"placed": self.placed, "placements": self.placements, "reason": self.reason,
                "reset_seq": self.last_seq if self.seq_seen else None, "placed_reset_seq": self.placed_seq,
                "reset_pending": self.pending,
                "floor": "none" if not self.placements else "estimated" if self.floor_estimated else floor_space,
                "floor_z": round(self.floor_z, 6), "offset": [round(float(x), 6) for x in self.offset],
                "yaw": round(self.yaw, 6), "scale": round(self.scale, 6),
                "origin_flu": matrix16(self.origin_p, self.origin_r), "sim_in_local": matrix16(p, r),
                "views": views}


@dataclass
class SimInLocal:
    """Where the sim world was drawn: ``x_local = r @ (scale * x_sim) + p``."""
    p: np.ndarray
    r: np.ndarray
    scale: float
    source: str                  # "logged" | "reconstructed"
    placements: int = 0

    def to_local(self, x_sim) -> np.ndarray:
        return self.r @ (self.scale * np.asarray(x_sim, float)) + self.p

    def camera(self, eye_p, eye_r) -> tuple[np.ndarray, np.ndarray]:
        """An eye pose in LOCAL -> the same camera in the sim world (position divided by the scale: a uniformly
        scaled world seen from a scaled-down camera is the same picture)."""
        return self.r.T @ (np.asarray(eye_p, float) - self.p) / self.scale, self.r.T @ np.asarray(eye_r, float)


def logged_placement(frame: dict) -> Optional[SimInLocal]:
    """The placement a record carries (``robot_ar``), or None (an older headset, or the AR view off / not placed)."""
    b = frame.get("robot_ar") if isinstance(frame.get("robot_ar"), dict) else None
    if not b or not b.get("placed"):
        return None
    pr = pose16(b.get("sim_in_local"))
    if pr is None:
        return None
    scale = b.get("scale")
    scale = float(scale) if isinstance(scale, (int, float)) and scale > 0 else 1.0
    return SimInLocal(pr[0], pr[1], scale, "logged", int(b.get("placements") or 0))


class Reconstruction:
    """The placement of an older log (no ``robot_ar`` block), rebuilt from its head poses and the sim's reset_seq
    (``Placement`` above). Feed it the records in time order."""

    def __init__(self, offset=(0.0, 0.0, 0.0), yaw: float = 0.0, scale: float = 1.0):
        self.pl = Placement(offset=np.asarray(offset, float), yaw=float(yaw), scale=float(scale))

    def update(self, frame: dict, reset_seq: Optional[int]) -> Optional[SimInLocal]:
        h = head_pose(frame)
        self.pl.update(frame.get("session_id"), flu(*h) if h is not None else None, reset_seq)
        if not self.pl.placed:
            return None
        p, r = self.pl.sim_in_local()
        return SimInLocal(p, r, self.pl.scale, "reconstructed", self.pl.placements)


# -- eyes ---------------------------------------------------------------------------------------------------------

@dataclass
class Eye:
    """A camera relative to the head (VIEW space) and its field of view (tangent extents, OpenXR's sign
    convention: left and down negative)."""
    p: np.ndarray
    r: np.ndarray
    fov: tuple                   # (angle_left, angle_right, angle_up, angle_down), rad
    source: str                  # "logged" | "default"

    def tangents(self) -> tuple[float, float, float, float]:
        return tuple(math.tan(a) for a in self.fov)


def logged_views(frame: dict) -> Optional[dict]:
    """``{"left": Eye, "right": Eye}`` from a record's ``robot_ar.views``, or None."""
    b = frame.get("robot_ar") if isinstance(frame.get("robot_ar"), dict) else None
    views = b.get("views") if b else None
    if not isinstance(views, list) or len(views) != 2:
        return None
    out = {}
    for name, v in zip(("left", "right"), views):
        pr = pose16(v.get("pose")) if isinstance(v, dict) else None
        fov = v.get("fov") if isinstance(v, dict) else None
        if pr is None or not isinstance(fov, list) or len(fov) != 4:
            return None
        out[v.get("eye") or name] = Eye(pr[0], pr[1], tuple(float(x) for x in fov), "logged")
    return out


def default_views() -> dict:
    return {k: Eye(np.array([(-1 if k == "left" else 1) * DEFAULT_IPD_M / 2, 0.0, 0.0]), np.eye(3),
                   DEFAULT_EYE_FOV[k], "default") for k in ("left", "right")}


def make_eye(views: dict, which: str) -> Eye:
    """``left`` / ``right``: that eye as it was; ``center``: the VIEW space (between the eyes) with the binocular
    field (the left eye's left edge, the right eye's right edge, the wider of up and down): everything either eye
    saw, from one viewpoint."""
    if which in ("left", "right"):
        return views[which]
    lv, rv = views["left"], views["right"]
    fov = (min(lv.fov[0], rv.fov[0]), max(lv.fov[1], rv.fov[1]), max(lv.fov[2], rv.fov[2]),
           min(lv.fov[3], rv.fov[3]))
    return Eye(np.zeros(3), np.eye(3), fov, lv.source)


def eye_in_local(head: tuple, eye: Eye) -> tuple[np.ndarray, np.ndarray]:
    hp, hr = head
    return hp + hr @ eye.p, hr @ eye.r


def project(eye_local: tuple, eye: Eye, x_local: Sequence[float], width: int, height: int) -> Optional[np.ndarray]:
    """Pixel (u right, v down; pixel centres at +0.5) of a LOCAL point seen from ``eye_local`` with ``eye``'s field
    of view on a ``width`` x ``height`` image; None behind the eye. The reference the renderer is tested against."""
    p, r = eye_local
    c = r.T @ (np.asarray(x_local, float) - p)            # camera frame: x right, y up, -z forward
    if c[2] >= -1e-9:
        return None
    x, y = c[0] / -c[2], c[1] / -c[2]
    tl, tr, tu, td = eye.tangents()
    return np.array([(x - tl) / (tr - tl) * width, (tu - y) / (tu - td) * height])
