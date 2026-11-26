"""The headset's body tracking block (quest_env BODY_TRACKING.md, schema ``teleop.quest_body.v1``): parse and
validate it, summarise it, and build one (the fake headset, tests).

The headset forwards Meta body tracking (``XR_FB_body_tracking``; the full-body joint set and the fidelity when
the runtime has them) as a ``body`` block in a ``teleop.quest_state.v1`` record, only when telemetry runs with
``--body``. Poses are in OpenXR LOCAL, the space of ``head`` and ``hands``, as 7 numbers per joint: position
(metres), quaternion ``[x, y, z, w]``. Per-joint ``flags`` are ``XrSpaceLocationFlags``: 1 orientation valid,
2 position valid, 4 orientation tracked, 8 position tracked.

The reader parses it (summarised on the node's ``/state``; recordings keep it with the rest of each frame). Teleop
uses it only when asked: the base hint's chest / hips source (``hint_source.py``) and the elbow-swivel hint
(``swivel.py``).

``parse_body(frame)`` returns None without a block (older headsets, ``--body`` off) or with a newer schema version
(forward compatible), and raises ``BodyError`` on a malformed v1 block.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Union

import numpy as np

SCHEMA = "teleop.quest_body.v1"
JOINT_SET_IDS = {"default": 0, "full_body": 1000274000}
JOINT_COUNTS = {"default": 70, "full_body": 84}
ORIENTATION_VALID, POSITION_VALID, ORIENTATION_TRACKED, POSITION_TRACKED = 1, 2, 4, 8
VALID = ORIENTATION_VALID | POSITION_VALID
TRACKED = ORIENTATION_TRACKED | POSITION_TRACKED

# XrFullBodyJointMETA (ids 0..69 are XrBodyJointFB's too).
JOINT_NAMES = (
    "root", "hips", "spine_lower", "spine_middle", "spine_upper", "chest", "neck", "head",
    "left_shoulder", "left_scapula", "left_arm_upper", "left_arm_lower", "left_hand_wrist_twist",
    "right_shoulder", "right_scapula", "right_arm_upper", "right_arm_lower", "right_hand_wrist_twist",
    *[f"{side}_hand_{j}" for side in ("left",) for j in (
        "palm", "wrist",
        *[f"{f}_{s}" for f in ("thumb",) for s in ("metacarpal", "proximal", "distal", "tip")],
        *[f"{f}_{s}" for f in ("index", "middle", "ring", "little")
          for s in ("metacarpal", "proximal", "intermediate", "distal", "tip")])],
    *[f"{side}_hand_{j}" for side in ("right",) for j in (
        "palm", "wrist",
        *[f"{f}_{s}" for f in ("thumb",) for s in ("metacarpal", "proximal", "distal", "tip")],
        *[f"{f}_{s}" for f in ("index", "middle", "ring", "little")
          for s in ("metacarpal", "proximal", "intermediate", "distal", "tip")])],
    *[f"{side}_{j}" for side in ("left", "right") for j in (
        "upper_leg", "lower_leg", "foot_ankle_twist", "foot_ankle", "foot_subtalar", "foot_transverse",
        "foot_ball")],
)
JOINT_IDS = {n: i for i, n in enumerate(JOINT_NAMES)}
assert len(JOINT_NAMES) == 84 and JOINT_IDS["right_hand_little_tip"] == 69 and JOINT_IDS["left_upper_leg"] == 70
# Readable aliases for the joints teleop work cares about (the elbow is OpenXR's "arm lower").
ALIASES = {"left_elbow": "left_arm_lower", "right_elbow": "right_arm_lower",
           "left_upper_arm": "left_arm_upper", "right_upper_arm": "right_arm_upper"}
FINGERS = frozenset(range(20, 44)) | frozenset(range(46, 70))


def core_ids(joint_count: int) -> list[int]:
    """The joints a ``core`` block carries: all but the 48 finger joints (BODY_TRACKING.md)."""
    return [i for i in range(joint_count) if i not in FINGERS]


class BodyError(ValueError):
    pass


@dataclass
class Body:
    available: bool
    reason: str = ""
    active: bool = False
    joint_set: str = "default"
    joint_count: int = 70
    confidence: float = 0.0
    fidelity: Optional[str] = None
    fidelity_requested: Optional[str] = None
    skeleton_changed_count: int = 0
    time_ns: Optional[int] = None
    subset: str = "core"
    ids: np.ndarray = field(default_factory=lambda: np.zeros(0, int))
    flags: np.ndarray = field(default_factory=lambda: np.zeros(0, int))
    poses: np.ndarray = field(default_factory=lambda: np.zeros((0, 7)))   # LOCAL: x y z qx qy qz qw

    def _row(self, joint: Union[int, str]) -> Optional[int]:
        j = JOINT_IDS.get(ALIASES.get(joint, joint)) if isinstance(joint, str) else int(joint)
        if j is None:
            raise KeyError(joint)
        rows = np.flatnonzero(self.ids == j)
        return int(rows[0]) if rows.size else None

    def joint(self, joint: Union[int, str]) -> Optional[tuple[np.ndarray, np.ndarray, int]]:
        """``(position, quaternion xyzw, flags)`` of a joint (id or name; ``left_elbow`` etc. too) in LOCAL, or
        None when it was not sent."""
        r = self._row(joint)
        return None if r is None else (self.poses[r, :3], self.poses[r, 3:], int(self.flags[r]))

    @property
    def valid_count(self) -> int:
        """Joints with a valid position and orientation."""
        return int(np.count_nonzero((self.flags & VALID) == VALID))

    @property
    def tracked_count(self) -> int:
        """Joints whose position and orientation are both tracked (not inferred)."""
        return int(np.count_nonzero((self.flags & TRACKED) == TRACKED))

    def summary(self) -> dict:
        if not self.available:
            return {"available": False, "reason": self.reason}
        return {"available": True, "active": self.active, "joint_set": self.joint_set,
                "joint_count": self.joint_count, "sent": int(self.ids.size), "subset": self.subset,
                "valid": self.valid_count, "tracked": self.tracked_count,
                "confidence": round(self.confidence, 3), "fidelity": self.fidelity,
                "fidelity_requested": self.fidelity_requested,
                "skeleton_changed_count": self.skeleton_changed_count}


def _int(v, what: str) -> int:
    if not isinstance(v, int) or isinstance(v, bool):
        raise BodyError(f"body: {what} is not an integer")
    return v


def _num(v, what: str) -> float:
    if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v):
        raise BodyError(f"body: {what} is not a finite number")
    return float(v)


def parse_block(b) -> Optional[Body]:
    """Validate a ``body`` block (module docstring)."""
    if not isinstance(b, dict):
        raise BodyError("body: not an object")
    if b.get("schema") != SCHEMA:
        if isinstance(b.get("schema"), str) and b["schema"].startswith("teleop.quest_body."):
            return None                      # a newer version: ignored, not an error
        raise BodyError(f"body: schema {b.get('schema')!r}")
    if not isinstance(b.get("available"), bool):
        raise BodyError("body: available is not a boolean")
    if not b["available"]:
        reason = b.get("reason", "")
        return Body(False, reason if isinstance(reason, str) else str(reason))
    if not isinstance(b.get("active"), bool):
        raise BodyError("body: active is not a boolean")
    js = b.get("joint_set")
    if js not in JOINT_COUNTS:
        raise BodyError(f"body: joint_set {js!r}")
    count = _int(b.get("joint_count"), "joint_count")
    if count != JOINT_COUNTS[js]:
        raise BodyError(f"body: joint_count {count} for joint set {js}")
    if "joint_set_id" in b and _int(b["joint_set_id"], "joint_set_id") != JOINT_SET_IDS[js]:
        raise BodyError(f"body: joint_set_id {b['joint_set_id']} for joint set {js}")
    fid = b.get("fidelity")
    req = b.get("fidelity_requested")
    for name, v in (("fidelity", fid), ("fidelity_requested", req)):
        if v not in (None, "high", "low"):
            raise BodyError(f"body: {name} {v!r}")
    ids, flags, poses = b.get("joints"), b.get("flags"), b.get("poses")
    if not (isinstance(ids, list) and isinstance(flags, list) and isinstance(poses, list)):
        raise BodyError("body: joints, flags and poses must be lists")
    if len(flags) != len(ids) or len(poses) != 7 * len(ids):
        raise BodyError(f"body: {len(ids)} joints, {len(flags)} flags, {len(poses)} pose numbers")
    ids = [_int(i, "a joint id") for i in ids]
    if any(not 0 <= i < count for i in ids) or any(a >= c for a, c in zip(ids, ids[1:])):
        raise BodyError("body: joint ids must be ascending, unique and within the joint set")
    flags = [_int(f, "a joint's flags") for f in flags]
    poses = [_num(x, "a pose number") for x in poses]
    tns = b.get("time_ns")
    return Body(True, "", b["active"], js, count, _num(b.get("confidence", 0.0), "confidence"), fid, req,
                _int(b.get("skeleton_changed_count", 0), "skeleton_changed_count"),
                None if tns is None else _int(tns, "time_ns"), str(b.get("subset", "core")),
                np.asarray(ids, int), np.asarray(flags, int), np.asarray(poses, float).reshape(-1, 7))


def parse_body(frame: dict) -> Optional[Body]:
    """The frame's body block, parsed; None when absent or a newer version; ``BodyError`` when malformed."""
    if not isinstance(frame, dict) or "body" not in frame:
        return None
    return parse_block(frame["body"])


def _q(r: np.ndarray) -> list:
    """Rotation matrix -> quaternion [x, y, z, w] (w >= 0)."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
    if q[3] < 0:
        q = [-x for x in q]
    return q


def encode(joints: dict, *, joint_set: str = "default", active: bool = True, confidence: float = 0.9,
           fidelity: Optional[str] = "high", fidelity_requested: Optional[str] = "high",
           skeleton_changed_count: int = 1, time_ns: Optional[int] = 0, subset: str = "core") -> dict:
    """A v1 block as the headset writes it. ``joints``: id -> ``(p, R or quaternion xyzw, flags)`` in LOCAL; ids
    outside the subset are dropped, missing ones are sent with flags 0 (as the runtime reports them)."""
    count = JOINT_COUNTS[joint_set]
    ids = list(range(count)) if subset == "all" else core_ids(count)
    flags, poses = [], []
    for i in ids:
        if i in joints:
            p, r, f = joints[i]
            q = _q(np.asarray(r, float)) if np.shape(r) == (3, 3) else list(r)
            flags.append(int(f))
            poses.extend(round(float(x), 4) + 0.0 for x in (*p, *q))
        else:
            flags.append(0)
            poses.extend([0.0] * 6 + [1.0])
    if not active:
        ids, flags, poses = [], [], []
    return {"schema": SCHEMA, "available": True, "active": active, "joint_set": joint_set,
            "joint_set_id": JOINT_SET_IDS[joint_set], "joint_count": count, "confidence": round(confidence, 3),
            "fidelity": fidelity if active else None, "fidelity_requested": fidelity_requested,
            "skeleton_changed_count": skeleton_changed_count, "time_ns": time_ns, "subset": subset,
            "joints": ids, "flags": flags, "poses": poses}


def unavailable(reason: str) -> dict:
    return {"schema": SCHEMA, "available": False, "reason": reason}


# -- a plausible skeleton (the fake headset) ------------------------------------------------------------------

UPPER_ARM, FOREARM = 0.30, 0.27      # metres


def _frame_along(d: np.ndarray, up: np.ndarray) -> np.ndarray:
    """A rotation whose x axis is ``d`` (z as close to ``up`` as possible)."""
    x = d / (np.linalg.norm(d) or 1.0)
    z = up - np.dot(up, x) * x
    if np.linalg.norm(z) < 1e-6:
        z = np.array([1.0, 0.0, 0.0]) - x[0] * x
    z /= np.linalg.norm(z)
    return np.column_stack([x, np.cross(z, x), z])


def _elbow(shoulder: np.ndarray, wrist: np.ndarray, side: float) -> np.ndarray:
    """Two-link IK in operator FLU: the elbow on the circle of solutions, hanging down and slightly outward."""
    d = wrist - shoulder
    n = float(np.linalg.norm(d))
    n = min(max(n, 1e-3), UPPER_ARM + FOREARM - 1e-4)
    u = d / (np.linalg.norm(d) or 1.0)
    a = (UPPER_ARM ** 2 - FOREARM ** 2 + n ** 2) / (2 * n)
    h = math.sqrt(max(UPPER_ARM ** 2 - a ** 2, 0.0))
    pref = np.array([0.0, 0.35 * side, -1.0])            # down, outward (side +1 left)
    perp = pref - np.dot(pref, u) * u
    if np.linalg.norm(perp) < 1e-6:
        perp = np.array([0.0, side, 0.0]) - np.dot(np.array([0.0, side, 0.0]), u) * u
    perp /= np.linalg.norm(perp)
    return shoulder + a * u + h * perp


def plausible_skeleton(head_p_op: Sequence[float], yaw: float, hands_op: dict, *,
                       neck_worn: bool = True, chest_yaw: Optional[float] = None, hips_yaw: Optional[float] = None,
                       hips_shift: Sequence[float] = (0.0, 0.0), swivel: Optional[dict] = None) -> dict:
    """A plausible upper body in operator FLU (metres), from the headset pose and the controller positions:
    joint id -> ``(p, R)`` (FLU). ``neck_worn``: the headset hangs on the neck (our operators), so the chest is just
    below it and the head above; otherwise the headset is the head. ``hands_op``: side -> wrist position or None
    (the arm then hangs). Orientations are the torso's yaw for the trunk and segment-aligned frames for the arms:
    nominal, not Meta's joint conventions. ``yaw`` is the headset's; ``chest_yaw`` (upper body, shoulders) and
    ``hips_yaw`` (pelvis, lower spine) default to it; ``hips_shift`` moves the pelvis in the plane (FLU).
    ``swivel``: side -> rad, the elbow turned about the shoulder → wrist axis (right-handed) from the IK's choice."""
    def rot(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    rz, rc = rot(yaw), rot(yaw if chest_yaw is None else chest_yaw)
    rh = rot(yaw if hips_yaw is None else hips_yaw)
    fwd, left, up = rc[:, 0], rc[:, 1], np.array([0.0, 0.0, 1.0])
    hp = np.asarray(head_p_op, float)
    neck = hp - (0.0 if neck_worn else 0.15) * up - 0.03 * rz[:, 0]
    head = neck + 0.20 * up if neck_worn else hp
    chest = neck - 0.15 * up
    hips = chest - 0.45 * up + np.array([hips_shift[0], hips_shift[1], 0.0])
    out = {JOINT_IDS["root"]: (hips - 0.95 * up, rh), JOINT_IDS["hips"]: (hips, rh),
           JOINT_IDS["spine_lower"]: (hips + 0.10 * up, rh), JOINT_IDS["spine_middle"]: (hips + 0.22 * up, rh),
           JOINT_IDS["spine_upper"]: (hips + 0.34 * up, rc), JOINT_IDS["chest"]: (chest, rc),
           JOINT_IDS["neck"]: (neck, rc), JOINT_IDS["head"]: (head, rz)}
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        scap = chest + 0.07 * sgn * left + 0.04 * up
        shoulder = chest + 0.18 * sgn * left + 0.02 * up
        wrist = hands_op.get(side)
        wrist = shoulder - (UPPER_ARM + FOREARM - 0.01) * up if wrist is None else np.asarray(wrist, float)
        elbow = _elbow(shoulder, wrist, sgn)
        psi = (swivel or {}).get(side, 0.0)
        if psi:
            ax = (wrist - shoulder) / (np.linalg.norm(wrist - shoulder) or 1.0)
            v = elbow - shoulder
            c, s_ = math.cos(psi), math.sin(psi)
            elbow = shoulder + v * c + np.cross(ax, v) * s_ + ax * float(ax @ v) * (1 - c)
        wrist = elbow + (wrist - elbow) / (np.linalg.norm(wrist - elbow) or 1.0) * min(
            FOREARM, float(np.linalg.norm(wrist - elbow)))
        ru, rf = _frame_along(elbow - shoulder, up), _frame_along(wrist - elbow, up)
        p = f"{side}_"
        out[JOINT_IDS[p + "shoulder"]] = (shoulder, rc)
        out[JOINT_IDS[p + "scapula"]] = (scap, rc)
        out[JOINT_IDS[p + "arm_upper"]] = (shoulder, ru)
        out[JOINT_IDS[p + "arm_lower"]] = (elbow, rf)
        out[JOINT_IDS[p + "hand_wrist_twist"]] = (elbow + 0.8 * (wrist - elbow), rf)
        out[JOINT_IDS[p + "hand_wrist"]] = (wrist, rf)
        out[JOINT_IDS[p + "hand_palm"]] = (wrist + 0.08 * rf[:, 0], rf)
    return out
