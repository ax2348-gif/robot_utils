"""The operator stream (``quest_teleop.oplog``) as tables, and the body-frozen clutch anchors reconstructed from it.

The operator stream's records are JSON (``headset``: the raw ``teleop.quest_state.v1`` frame; ``sent``: each
datagram to the WBC; ``event``). The browser needs a few numbers per record, so ``OplogTables.open`` parses each
record once into column tables (cached as memory-mapped ``.npy`` next to the stream index) which then read like
any rows stream (``TableStream``: ``rows`` / ``fields`` / ``envelope``):

- ``oplog.headset``: per Quest frame, in **operator FLU axes** (``frames.C`` applied to OpenXR LOCAL; origin =
  LOCAL's, the head where tracking started): the head and both controllers' poses (position, quaternion xyzw of
  ``C R_local``: the device's own OpenXR axes, -z forward, expressed in FLU), validity, grip, trigger, sticks,
  buttons, and the native intent's clutch state per hand (``engaged``, ``engagement_id``, ``needs_release``).
- ``oplog.sent``: per datagram: ``armed``, ``arm_seq``, the EE target (world), gripper, the base mode
  (``BASE_MODES``) and the base hint target (``hint_x``, ``hint_y``, ``hint_yaw``) or commanded velocity.

**Clutch anchors** (``anchors``). The Pi's body-frozen client (tools/quest_teleop/body_frozen.py) captures, on the
frame an engagement starts: the operator frame (origin = the head position, X = its gravity-aligned yaw), the
controller pose in it (the hand anchor), the platform frame (the WBC row's base x, y, yaw) and the commanded EE
pose (the EE anchor). None of these are sent as such, but:

- the capture frame is the first frame of the engagement: the native intent's ``engagement_id`` changes with
  ``engaged`` true; the head and hand poses of that frame give the operator frame and the hand anchor exactly;
- the client then sends ``base.mode = "hint"``; on its first hint datagram the hint pose is the platform frame
  (head motion and stick offset are still zero) and the EE target is the EE anchor (``target_raw`` = the
  command at capture), exactly when that step handled one frame (the usual case at 72 Hz), within one frame's
  hand motion otherwise;
- an engagement the client refused (no fresh WBC row, head not tracked, needs a release) sends no hint: it is
  listed with ``accepted: false``.

What is NOT recorded: which of a multi-frame step's frames produced a datagram (``sent`` has no ``frame_seq``),
and the anchors themselves; docs/browse.md "Anchors" proposes the smallest addition.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from .index import KINDS, StreamIndex

C = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # OpenXR LOCAL -> operator FLU
BASE_MODES = {"free": 0, "locked": 1, "commanded": 2, "hint": 3}
TABLE_VERSION = 2

_POSE = ("x", "y", "z", "qx", "qy", "qz", "qw")
HEADSET_FIELDS = (["t_rx", "frame_seq", "focused", "head_valid"] + [f"head_{k}" for k in _POSE]
                  + [f"{h}_{k}" for h in ("left", "right") for k in (
                      ["valid"] + list(_POSE) + ["grip", "trigger", "stick_x", "stick_y", "btn_primary",
                                                  "btn_secondary", "btn_stick", "engaged", "engagement_id",
                                                  "needs_release", "intent_valid"])]
                  + ["drive_vx", "drive_vy", "drive_w"])
SENT_FIELDS = (["t_send", "seq", "is_target", "armed", "arm_seq"] + [f"ee_{k}" for k in _POSE]
               + ["ee_gripper", "base_mode", "hint_x", "hint_y", "hint_yaw", "cmd_vx", "cmd_vy", "cmd_w", "est_reset"])


def quat_from_matrix(r: np.ndarray) -> np.ndarray:
    """[x, y, z, w] of a rotation matrix (Shepperd)."""
    m = r
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def matrix_from_quat(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _pose16(m) -> Optional[tuple]:
    """(p, R) in LOCAL from the 16 row-major numbers, or None (frames.pose_from_matrix16's rule)."""
    if not isinstance(m, (list, tuple)) or len(m) != 16:
        return None
    try:
        a = np.asarray(m, float).reshape(4, 4)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(a)):
        return None
    r = a[:3, :3]
    if np.abs(r.T @ r - np.eye(3)).max() > 1e-2:
        return None
    u, _, vt = np.linalg.svd(r)
    q = u @ vt
    if np.linalg.det(q) < 0:
        u[:, -1] *= -1
        q = u @ vt
    return a[:3, 3].copy(), q


def _flu_pose(m) -> list:
    """7 numbers (FLU position, quaternion of C R) or NaNs."""
    p = _pose16(m)
    if p is None:
        return [np.nan] * 7
    return list(C @ p[0]) + list(quat_from_matrix(C @ p[1]))


def _num(v) -> float:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else np.nan


def _b(v) -> float:
    return 1.0 if v is True else 0.0


def headset_row(t: float, fr: dict) -> list:
    head = fr.get("head") or {}
    hv = bool(head.get("position_tracked") and head.get("orientation_tracked"))
    row = [t, _num(fr.get("frame_seq")), _b(fr.get("input_focused")), float(hv)]
    row += _flu_pose(head.get("pose")) if hv else [np.nan] * 7
    intent = (fr.get("control_intent") or {}).get("hands") or {}
    for name in ("left", "right"):
        h = (fr.get("hands") or {}).get(name) or {}
        ok = bool(h.get("active") and h.get("position_valid") and h.get("orientation_valid"))
        js = h.get("joystick") or [np.nan, np.nan]
        bt = h.get("buttons") or {}
        ih = intent.get(name) or {}
        row += [float(ok)] + (_flu_pose(h.get("pose")) if ok else [np.nan] * 7)
        row += [_num(h.get("grip")), _num(h.get("trigger")), _num(js[0] if len(js) > 0 else None),
                _num(js[1] if len(js) > 1 else None), _b(bt.get("primary")), _b(bt.get("secondary")),
                _b(bt.get("stick")), _b(ih.get("engaged")), _num(ih.get("engagement_id")), _b(ih.get("needs_release")),
                _b(ih.get("valid"))]
    bv = (fr.get("control_intent") or {}).get("base_velocity") or [np.nan] * 3
    row += [_num(x) for x in (list(bv) + [None] * 3)[:3]]
    return row


def sent_row(t: float, d: dict) -> list:
    ees = d.get("ees") or {}
    ee = next(iter(ees.values()), {}) if ees else {}
    pos = ee.get("pos") or [np.nan] * 3
    quat = ee.get("quat") or [np.nan] * 4
    base = d.get("base") or {}
    mode = BASE_MODES.get(base.get("mode"), -1) if base else np.nan
    tgt = base.get("target") or {}
    pose = tgt.get("pose") or {}
    heading = tgt.get("heading") or {}
    vel = base.get("velocity") or {}
    return ([t, _num(d.get("seq")), float(d.get("type") == "target"), _b(d.get("armed")), _num(d.get("arm_seq"))]
            + [_num(x) for x in (list(pos) + [None] * 3)[:3]] + [_num(x) for x in (list(quat) + [None] * 4)[:4]]
            + [_num(ee.get("gripper")), mode, _num(pose.get("x")), _num(pose.get("y")), _num(heading.get("yaw")),
               _num(vel.get("vx")), _num(vel.get("vy")), _num(vel.get("omega")), _num(d.get("est_reset"))])


class TableStream:
    """A column table read like a rows stream: ``rows(a, b, fields)``, ``fields()``, ``t_range``, ``envelope``."""

    kind = "rows"

    def __init__(self, name: str, fields: Sequence[str], data: np.ndarray, units: Optional[dict] = None):
        self.name = name
        self._fields = list(fields)
        self.col = {f: i for i, f in enumerate(self._fields)}
        self.data = data                       # (n, k), column 0 = time
        self.units = units or {}
        self.n_records = len(data)
        self.dropped = 0

    def fields(self) -> list[tuple[str, str]]:
        return [(f, self.units.get(f, "")) for f in self._fields]

    @property
    def t_range(self) -> tuple[float, float]:
        if not self.n_records:
            return (np.nan, np.nan)
        return float(self.data[0, 0]), float(self.data[-1, 0])

    def rows(self, a: float, b: float, fields: Sequence[str], stride: int = 1):
        t = self.data[:, 0]
        lo, hi = int(np.searchsorted(t, a, "left")), int(np.searchsorted(t, b, "right"))
        blk = np.asarray(self.data[lo:hi:max(1, stride)])
        v = np.full((len(blk), len(fields)), np.nan)
        for k, f in enumerate(fields):
            if f in self.col:
                v[:, k] = blk[:, self.col[f]]
        return blk[:, 0].astype(np.float64), v

    def envelope(self, a: float, b: float, field_name: str, buckets: int) -> dict:
        t, v = self.rows(a, b, [field_name])
        edges = np.linspace(a, b, buckets + 1)
        k = np.clip(np.searchsorted(edges, t, "right") - 1, 0, buckets - 1)
        lo, hi = np.full(buckets, np.nan), np.full(buckets, np.nan)
        ok = np.isfinite(v[:, 0])
        if ok.any():
            np.fmin.at(lo, k[ok], v[ok, 0])
            np.fmax.at(hi, k[ok], v[ok, 0])
        return {"t0": a, "t1": b, "min": lo, "max": hi}


class OplogTables:
    def __init__(self, headset: TableStream, sent: TableStream, events: list[dict]):
        self.headset, self.sent, self.events = headset, sent, events
        self._anchors: Optional[list] = None

    @classmethod
    def open(cls, idx: StreamIndex, cache: Optional[Path] = None) -> "OplogTables":
        from .index import cache_root, files_key
        root = cache_root() if cache is None else Path(cache)
        d = root / f"{idx.name}-{files_key(idx.files, f'oplog{TABLE_VERSION}')}"
        try:
            hs = np.load(d / "headset.npy", mmap_mode="r")
            se = np.load(d / "sent.npy", mmap_mode="r")
            ev = json.loads((d / "events.json").read_text())
            return cls(TableStream("oplog.headset", HEADSET_FIELDS, hs), TableStream("oplog.sent", SENT_FIELDS, se), ev)
        except (OSError, ValueError):
            pass
        hrows, srows, events = [], [], []
        kinds = idx.a["kind"]
        for i in range(idx.n_records):
            k = int(kinds[i])
            if k not in (KINDS["headset"], KINDS["sent"], KINDS["event"]):
                continue
            h = idx.header(i)
            if k == KINDS["headset"]:
                fr = h.get("frame")
                if isinstance(fr, dict):
                    hrows.append(headset_row(float(idx.a["t0"][i]), fr))
            elif k == KINDS["sent"]:
                dg = h.get("datagram")
                if isinstance(dg, dict):
                    srows.append(sent_row(float(idx.a["t0"][i]), dg))
            else:
                from ..records import event_line
                events.append(event_line(h))
        hs = np.asarray(hrows, np.float64).reshape(-1, len(HEADSET_FIELDS))
        se = np.asarray(srows, np.float64).reshape(-1, len(SENT_FIELDS))
        for a in (hs, se):
            if len(a) and np.any(np.diff(a[:, 0]) < 0):
                a[:] = a[np.argsort(a[:, 0], kind="stable")]
        try:
            d.mkdir(parents=True, exist_ok=True)
            np.save(d / "headset.npy", hs)
            np.save(d / "sent.npy", se)
            tmp = d / "events.json.tmp"
            tmp.write_text(json.dumps(events))
            os.replace(tmp, d / "events.json")
        except OSError:
            pass
        return cls(TableStream("oplog.headset", HEADSET_FIELDS, hs), TableStream("oplog.sent", SENT_FIELDS, se), events)

    # -- anchors ---------------------------------------------------------------------------------

    def anchors(self) -> list[dict]:
        if self._anchors is None:
            self._anchors = reconstruct_anchors(self.headset, self.sent)
        return self._anchors


def head_yaw_flu(q) -> Optional[float]:
    """Gravity-aligned yaw of the head from its FLU quaternion (frames.head_yaw: forward = R (0, 0, -1))."""
    f = matrix_from_quat(q) @ np.array([0.0, 0.0, -1.0])
    if math.hypot(f[0], f[1]) < 0.2:
        return None
    return math.atan2(f[1], f[0])


def reconstruct_anchors(hs: TableStream, sent: TableStream) -> list[dict]:
    """One entry per engagement of either hand (see the module docstring)."""
    H = np.asarray(hs.data)
    S = np.asarray(sent.data)
    out = []
    if not len(H):
        return out
    c = hs.col
    st, smode = (S[:, 0], S[:, sent.col["base_mode"]]) if len(S) else (np.zeros(0), np.zeros(0))
    for hand in ("left", "right"):
        eng = H[:, c[f"{hand}_engaged"]] > 0.5
        eid = H[:, c[f"{hand}_engagement_id"]]
        start = np.flatnonzero(eng & ~np.concatenate([[False], eng[:-1]]))
        # a new engagement id while engaged (a release the stream did not show) also starts one
        changed = np.flatnonzero(eng[1:] & eng[:-1] & (eid[1:] != eid[:-1])) + 1
        for i in sorted(set(start.tolist()) | set(changed.tolist())):
            j = i
            while j + 1 < len(H) and eng[j + 1] and eid[j + 1] == eid[i]:
                j += 1
            t0, t1 = float(H[i, 0]), float(H[j, 0])
            hp = H[i, [c[f"head_{k}"] for k in _POSE]]
            pp = H[i, [c[f"{hand}_{k}"] for k in _POSE]]
            a = {"hand": hand, "engagement_id": _num(eid[i]), "t_start": t0, "t_end": t1, "frame": int(i),
                 "accepted": False, "head": None, "operator": None, "hand_world": None, "hand_anchor": None,
                 "robot_frame": None, "ee_anchor": None, "t_first_hint": None}
            if np.all(np.isfinite(hp)):
                yaw = head_yaw_flu(hp[3:])
                a["head"] = [float(v) for v in hp]
                if yaw is not None:
                    a["operator"] = {"origin": [float(v) for v in hp[:3]], "yaw": yaw}
            if np.all(np.isfinite(pp)):
                a["hand_world"] = [float(v) for v in pp]
                if a["operator"] is not None:
                    rz = np.array([[math.cos(-a["operator"]["yaw"]), -math.sin(-a["operator"]["yaw"]), 0],
                                   [math.sin(-a["operator"]["yaw"]), math.cos(-a["operator"]["yaw"]), 0], [0, 0, 1]])
                    p = rz @ (pp[:3] - hp[:3])
                    r = rz @ matrix_from_quat(pp[3:])
                    a["hand_anchor"] = [float(v) for v in list(p) + list(quat_from_matrix(r))]
            # the first hint datagram during the engagement: the robot-side anchors
            if len(st):
                k0 = int(np.searchsorted(st, t0, "left"))
                k1 = int(np.searchsorted(st, t1 + 0.1, "right"))
                hint = np.flatnonzero(smode[k0:k1] == BASE_MODES["hint"])
                if len(hint):
                    k = k0 + int(hint[0])
                    prev_hint = k > 0 and smode[k - 1] == BASE_MODES["hint"] and st[k - 1] >= t0
                    if not prev_hint:
                        sc = sent.col
                        a["accepted"] = True
                        a["t_first_hint"] = float(st[k])
                        a["robot_frame"] = [float(S[k, sc["hint_x"]]), float(S[k, sc["hint_y"]]),
                                            float(S[k, sc["hint_yaw"]])]
                        a["ee_anchor"] = [float(S[k, sc[f"ee_{q}"]]) for q in _POSE]
            out.append(a)
    out.sort(key=lambda a: a["t_start"])
    return out
