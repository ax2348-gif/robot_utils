#!/usr/bin/env python3
"""A fake Quest headset: serves ``teleop.quest_state.v1`` JSON lines on TCP (default port 9391) with scripted head
and hand motion, for tests and for a dry run of the client without the headset.

    python tools/quest_teleop/fake_headset.py [--port 9391] [--rate 72] [--scenario demo] [--loop] [--ar URL]

The frames have the native app's shape (quest_env native/telemetry.cpp): ``session_id``, ``frame_seq``,
``input_focused``, ``action_sync_result``, ``head`` (tracking flags and a 16-number row-major LOCAL pose),
``hands.left/right`` (active, flags, pose, trigger, grip, joystick, buttons) and ``control_intent``, computed by
``NativeIntent``, a Python port of the native ``teleop::Intent`` (grip hysteresis 0.6/0.4, release required
after start, focus loss, gaps and session changes, the 18 % stick dead zone, body-frozen deltas).

Scenarios are lists of keyframes ``(t, state)``: poses are given in operator FLU (metres, radians) and converted
to OpenXR LOCAL with ``C``; values between keyframes are interpolated linearly (yaw too), booleans and
the session step. ``demo`` engages the right hand, moves it, walks forward, turns the head, uses the stick,
releases, loses focus and changes session. ``record`` arms (Y), reaches, looks around, starts an episode (A: the
sim resets), arms again, reaches and stops it (B tap): a sim teleop recording to replay.

``--ar URL`` (the sim stack's frontend) emulates the headset's robot AR view for the log: ``ArView`` places the sim
world as the native view does (robot_utils ``replay.ar.Placement``, a port of quest_env's: the first tracked head,
each change of the sim's ``reset_seq``, polled from ``URL/nodes/sim/scene3d/frame``) with a floor 1.6 m below the
first head, and every frame carries the ``robot_ar`` block a real headset logs (placement and eyes; quest_env
ROBOT_AR.md "Logged placement"). The eyes are a nominal Quest 3 pair (``replay.ar.default_views``).

``--body core|all`` adds the ``body`` block a headset sends with ``quest run --body`` (quest_env BODY_TRACKING.md):
a plausible upper body (``body.plausible_skeleton``: the headset worn on the neck, shoulders from the torso, elbows
from two-link IK to the controllers), the default joint set, fully tracked, inactive while the head is untracked.
``HeadsetState.elbow_swivel`` turns each elbow about its shoulder → wrist axis (the ``elbow`` scenario).

Haptics (quest_env HAPTICS.md): ``--haptics URL`` (the node's ``GET /haptics?stream=1``, e.g.
``http://127.0.0.1:8080/nodes/quest_teleop/haptics?stream=1``) reads the node's ``teleop.quest_haptics.v1`` lines
like the headset (``HapticsStream``) and plays them with ``HapticsPlayer``, a port of the headset's player, ticked
once per frame; every frame then carries the ``haptics`` block (the ack). Without it: an older build, no block.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Callable, Optional

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from quest_teleop import body as B     # noqa: E402
    from quest_teleop import frames as F   # noqa: E402
else:
    from . import body as B
    from . import frames as F

# -- native intent, ported ------------------------------------------------------------------------


def _stick(x: float) -> float:
    return max(-1.0, min(1.0, x)) if math.isfinite(x) and abs(x) > 0.18 else 0.0


@dataclass
class HandInput:
    valid: bool = False
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))      # LOCAL
    r: np.ndarray = field(default_factory=lambda: np.eye(3))        # LOCAL
    grip: float = 0.0
    trigger: float = 0.0
    sx: float = 0.0
    sy: float = 0.0


class NativeClutch:
    def __init__(self):
        self.engaged, self.needs_release, self.valid = False, True, False
        self.id = 0
        self.anchor = (np.zeros(3), np.eye(3))
        self.op_origin = np.zeros(3)
        self.op_rot = np.eye(3)          # operator base orientation in LOCAL (transpose of capturedFromQuest)
        self.captured = F.C.copy()
        self.delta = np.zeros(3)
        self.delta_r = np.eye(3)
        self.gripper = 0.0

    def invalidate(self):
        self.engaged, self.needs_release, self.valid = False, True, False
        self.delta, self.delta_r = np.zeros(3), np.eye(3)

    def update(self, h: HandInput, focused: bool, op_from_quest, body_frozen: bool, head):
        self.valid = focused and h.valid and np.all(np.isfinite(h.p)) and math.isfinite(h.grip)
        if not self.valid:
            self.invalidate()
            return
        self.gripper = max(0.0, min(1.0, h.trigger))
        if h.grip <= 0.4:
            self.engaged, self.needs_release = False, False
            self.delta, self.delta_r = np.zeros(3), np.eye(3)
            return
        if self.needs_release:
            return
        if body_frozen and not self.engaged and h.grip >= 0.6:
            if head is None:
                self.invalidate()
                return
            yaw = F.head_yaw(head[1])
            if yaw is None:
                self.invalidate()
                return
            self.captured = F.rot_z(-yaw) @ F.C
            self.op_origin, self.op_rot = head[0].copy(), self.captured.T
        mapping = self.captured if body_frozen else op_from_quest
        cur = (mapping @ (h.p - (self.op_origin if body_frozen else 0)), mapping @ h.r)
        if not self.engaged and h.grip >= 0.6:
            self.anchor, self.engaged = cur, True
            self.id += 1
        if self.engaged:
            d = cur[0] - self.anchor[0]
            n = np.linalg.norm(d)
            if not body_frozen and n > 0.45:
                d = d * (0.45 / n)
            self.delta, self.delta_r = d, cur[1] @ self.anchor[1].T


class NativeIntent:
    def __init__(self, body_frozen: bool = True):
        self.hands = [NativeClutch(), NativeClutch()]
        self.drive = np.zeros(3)
        self.session = 0
        self.initialized = False
        self.body_frozen = body_frozen
        self.op_from_quest = F.C.copy()
        self.revision = 0

    def realign(self, head) -> bool:
        if any(h.engaged for h in self.hands) or head is None:
            return False
        yaw = F.head_yaw(head[1])
        if yaw is None:
            return False
        self.op_from_quest = F.rot_z(-yaw) @ F.C
        self.revision += 1
        for h in self.hands:
            h.invalidate()
        self.drive = np.zeros(3)
        return True

    def update(self, session: int, focused: bool, left: HandInput, right: HandInput, gap=False, head=None,
               align=False):
        disc = not self.initialized or session != self.session
        if disc or gap:
            for h in self.hands:
                h.invalidate()
        if disc:
            self.op_from_quest, self.revision = F.C.copy(), 0
        self.initialized, self.session = True, session
        # A realigns in the standard scheme only (native teleop::Intent: body-frozen takes its frame at each grip)
        aligned = (align and not self.body_frozen and focused and not gap and left.grip <= 0.4 and right.grip <= 0.4
                   and head is not None and self.realign(head))
        ok = focused and not gap and not aligned
        self.hands[0].update(left, ok, self.op_from_quest, self.body_frozen, head)
        self.hands[1].update(right, ok, self.op_from_quest, self.body_frozen, head)
        self.drive = np.zeros(3)
        if ok:
            if self.hands[0].valid:
                self.drive[0] = 0.45 * _stick(left.sy)
                self.drive[1] = -0.45 * _stick(left.sx)
            if self.hands[1].valid:
                self.drive[2] = -0.8 * _stick(right.sx)

    def json(self) -> dict:
        out = {"schema": "teleop.operator_intent.v1", "session_id": str(self.session), "frame": "operator_base_flu",
               "scheme": "body-frozen" if self.body_frozen else "standard",
               "translation_limit_m": None if self.body_frozen else 0.45,
               "base_velocity": _r(self.drive), "operator_frame_revision": self.revision,
               "operator_from_quest": _r(self.op_from_quest.reshape(-1)), "hands": {}}
        for name, c in zip(("left", "right"), self.hands):
            h = {"valid": c.valid, "engaged": c.engaged, "needs_release": c.needs_release, "engagement_id": c.id,
                 "delta_position": _r(c.delta), "delta_rotation": _r(c.delta_r.reshape(-1))}
            if self.body_frozen:
                h["operator_base_in_quest_position"] = _r(c.op_origin)
                h["operator_base_in_quest_rotation"] = _r(c.op_rot.reshape(-1))
            h["gripper"] = c.gripper
            out["hands"][name] = h
        return out


def _r(a, nd: int = 6) -> list:
    return [round(float(x), nd) for x in np.asarray(a).reshape(-1)]


# -- the headset's state, in operator terms --------------------------------------------------------


@dataclass
class HandState:
    pos: tuple = (0.35, -0.2, -0.35)     # operator FLU, relative to the LOCAL origin
    rpy: tuple = (0.0, 0.0, 0.0)         # operator orientation (roll, pitch, yaw)
    grip: float = 0.0
    trigger: float = 0.0
    stick: tuple = (0.0, 0.0)            # raw joystick (x right, y up)
    tracked: bool = True
    click: bool = False                  # thumbstick click
    primary: bool = False                # X (left) / A (right; also HeadsetState.align)
    secondary: bool = False              # Y (left) / B (right)


@dataclass
class HeadsetState:
    head_pos: tuple = (0.0, 0.0, 0.0)    # operator FLU (LOCAL origin = the first head position)
    head_yaw: float = 0.0
    head_pitch: float = 0.0              # positive = looking down
    head_tracked: bool = True
    left: HandState = field(default_factory=lambda: HandState(pos=(0.35, 0.2, -0.35)))
    right: HandState = field(default_factory=HandState)
    focused: bool = True
    session: int = 1
    align: bool = False                  # the A button
    # the body block (Headset(body=...)): the upper body's and the pelvis's yaw (None: the head's), the pelvis
    # shifted in the plane (FLU), the confidence and whether the body is tracked at all
    chest_yaw: Optional[float] = None
    hips_yaw: Optional[float] = None
    hips_shift: tuple = (0.0, 0.0)
    body_confidence: float = 0.9
    body_active: bool = True
    elbow_swivel: tuple = (0.0, 0.0)     # (left, right) rad: each elbow turned about its shoulder → wrist axis


def _rpy(r, p, y):
    return F.rot_z(y) @ F.rot_axis((0, 1, 0), p) @ F.rot_axis((1, 0, 0), r)


def to_local(pos_op, r_op):
    """Operator FLU pose -> OpenXR LOCAL (the inverse of C)."""
    return F.C.T @ np.asarray(pos_op, float), F.C.T @ r_op


def head_r_op(yaw: float, pitch: float = 0.0):
    """The head's orientation in operator axes for a level head facing ``yaw`` (identity head = ``C``)."""
    return F.rot_z(yaw) @ F.rot_axis((0, 1, 0), pitch) @ F.C


def _body_joints(s: HeadsetState, neck_worn: bool = True) -> dict:
    """The plausible skeleton for a state, in LOCAL: id -> (p, R, flags)."""
    hands = {k: (np.asarray(h.pos, float) if h.tracked else None) for k, h in (("left", s.left), ("right", s.right))}
    out = {}
    for i, (p, r) in B.plausible_skeleton(s.head_pos, s.head_yaw, hands, neck_worn=neck_worn,
                                          chest_yaw=s.chest_yaw, hips_yaw=s.hips_yaw,
                                          hips_shift=s.hips_shift,
                                          swivel={"left": s.elbow_swivel[0], "right": s.elbow_swivel[1]}).items():
        pl, rl = to_local(p, r)
        out[i] = (pl, rl, B.VALID | B.TRACKED)
    return out


class Headset:
    """Builds native-shaped frames from a HeadsetState."""

    def __init__(self, body_frozen: bool = True, session_base: int = 1_700_000_000_000_000_000,
                 ar: Optional["ArView"] = None, body: Optional[str] = None, neck_worn: bool = True):
        self.ar = ar
        self.neck_worn = neck_worn                                   # the body's headset: on the neck or the head
        self.body = body if body in ("core", "all") else None      # the body block's joint subset; None: no block
        self.intent = NativeIntent(body_frozen)
        self.session_base = session_base
        self.seq = 0
        self.session = None
        self.last_t: Optional[float] = None

    def frame(self, s: HeadsetState, t: float) -> dict:
        sid = self.session_base + s.session
        if sid != self.session:
            self.session, self.seq = sid, 0
        gap = self.last_t is not None and t - self.last_t > 0.2
        self.last_t = t
        hp, hr = to_local(s.head_pos, head_r_op(s.head_yaw, s.head_pitch))
        head = (hp, hr) if s.head_tracked else None
        hands_in = []
        for hs in (s.left, s.right):
            p, r = to_local(hs.pos, _rpy(*hs.rpy))
            hands_in.append(HandInput(hs.tracked, p, r, hs.grip, hs.trigger, hs.stick[0], hs.stick[1]))
        self.intent.update(sid, s.focused, hands_in[0], hands_in[1], gap, head, s.align)
        frame = {
            "schema": "teleop.quest_state.v1", "session_id": str(sid), "frame_seq": self.seq,
            "sample_time_ns": int(t * 1e9), "xr_session_state": "FOCUSED" if s.focused else "VISIBLE",
            "action_sync_result": 0 if s.focused else 8, "xr_frame_hz": 72.0, "input_poll_ms": 0.1,
            "input_focused": s.focused, "reference_space": "local",
            "head": {"position_valid": s.head_tracked, "orientation_valid": s.head_tracked,
                     "position_tracked": s.head_tracked, "orientation_tracked": s.head_tracked,
                     "pose": F.matrix16_from_pose(hp, hr) if s.head_tracked else None},
            "hands": {}, "control_intent": self.intent.json(), "demo": {"enabled": False, "base_frame": "live"},
        }
        if self.ar is not None:
            frame["robot_ar"] = self.ar.block(sid, head, s.align and s.focused)
        if self.body is not None:
            frame["body"] = self.body_block(s, int(t * 1e9))
        for name, hs, hi in (("left", s.left, hands_in[0]), ("right", s.right, hands_in[1])):
            frame["hands"][name] = {
                "pose_source": "tracked_remote", "active": hs.tracked, "position_valid": hs.tracked,
                "orientation_valid": hs.tracked, "position_tracked": hs.tracked, "orientation_tracked": hs.tracked,
                "pose": F.matrix16_from_pose(hi.p, hi.r) if hs.tracked else None,
                "trigger": hs.trigger, "grip": hs.grip, "joystick": list(hs.stick),
                "buttons": {"primary": (name == "right" and s.align) or hs.primary, "secondary": hs.secondary,
                            "thumb_rest": False, "stick": hs.click}}
        self.seq += 1
        return frame

    def body_block(self, s: HeadsetState, time_ns: int) -> dict:
        """The body tracking block for a state (inactive while the head is untracked)."""
        if not s.head_tracked or not s.body_active:
            return B.encode({}, active=False, confidence=0.0, time_ns=time_ns, subset=self.body)
        return B.encode(_body_joints(s, self.neck_worn), confidence=s.body_confidence, time_ns=time_ns,
                        subset=self.body)


class ArView:
    """The headset's AR placement, emulated (see the module doc). ``reset_seq()`` returns the sim's newest
    ``reset_seq`` or None; ``floor_z``: the floor in LOCAL-FLU (the first head is LOCAL's origin)."""

    def __init__(self, reset_seq: Callable[[], Optional[int]] = lambda: None, floor_z: float = -1.6,
                 floor_space: str = "LOCAL_FLOOR"):
        from robot_utils.replay import ar as AR      # noqa: PLC0415 (only with --ar: needs robot_utils on the path)
        self.AR = AR
        self.placement = AR.Placement()
        self.reset_seq, self.floor_z, self.floor_space = reset_seq, floor_z, floor_space
        self.views = [{"eye": k, "fov": list(v.fov), "pose": AR.matrix16(v.p, v.r)}
                      for k, v in AR.default_views().items()]
        self._a_prev = False

    def block(self, session, head_local, a_down: bool) -> dict:
        a_press, self._a_prev = a_down and not self._a_prev, a_down
        head_flu = self.AR.flu(*head_local) if head_local is not None else None
        self.placement.update(str(session), head_flu, self.reset_seq(), a_press, self.floor_z)
        return self.placement.block(self.floor_space, self.views)


class SimResetSeq:
    """The sim's reset_seq, polled in the background from ``<url>/nodes/sim/scene3d/frame`` (the real headset reads
    it from the state stream)."""

    def __init__(self, url: str, period: float = 0.2):
        self.url = url.rstrip("/") + "/nodes/sim/scene3d/frame"
        self.value: Optional[int] = None
        self.period = period
        threading.Thread(target=self._run, name="fake-headset-reset-seq", daemon=True).start()

    def _run(self):
        import urllib.request     # noqa: PLC0415
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=2) as r:
                    self.value = int(json.loads(r.read()).get("reset_seq"))
            except Exception:     # noqa: BLE001 (the sim not up yet: keep polling)
                pass
            time.sleep(self.period)

    def __call__(self) -> Optional[int]:
        return self.value


# -- scenarios -------------------------------------------------------------------------------------


def _lerp(a, b, u):
    if a is None or b is None:                       # optional fields (the body's yaws): step
        return a if u < 1 else b
    if isinstance(a, bool) or isinstance(a, int) and isinstance(b, int):
        return a if u < 1 else b
    if isinstance(a, tuple):
        return tuple(_lerp(x, y, u) for x, y in zip(a, b))
    if isinstance(a, HandState):
        return HandState(*(_lerp(getattr(a, k), getattr(b, k), u) for k in a.__dataclass_fields__))
    return a + (b - a) * u


def sample(keyframes: list, t: float) -> HeadsetState:
    """The state at ``t`` from ``[(t0, state0), (t1, state1), ...]``; numbers interpolate, flags step."""
    if t <= keyframes[0][0]:
        return keyframes[0][1]
    for (t0, s0), (t1, s1) in zip(keyframes, keyframes[1:]):
        if t < t1:
            u = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            return HeadsetState(*(_lerp(getattr(s0, k), getattr(s1, k), u) for k in s0.__dataclass_fields__))
    return keyframes[-1][1]


def demo_scenario() -> list:
    """Engage, reach forward, walk 0.5 m, turn the head, drive with the stick, release, lose focus, new session."""
    s = HeadsetState()
    grip = replace(s, right=replace(s.right, grip=1.0))
    reach = replace(grip, right=replace(grip.right, pos=(0.45, -0.2, -0.30), trigger=0.8))
    walk = replace(reach, head_pos=(0.5, 0.0, 0.0), right=replace(reach.right, pos=(0.95, -0.2, -0.30)))
    turn = replace(walk, head_yaw=0.5)
    stick = replace(turn, left=replace(turn.left, stick=(0.0, 1.0)))
    unstick = replace(turn)
    release = replace(turn, right=replace(turn.right, grip=0.0))
    unfocus = replace(release, focused=False)
    refocus = replace(release, focused=True)
    newsession = replace(release, session=2)
    return [(0.0, s), (1.0, s), (1.01, grip), (2.0, grip), (3.0, reach), (5.0, walk), (6.0, turn), (6.01, stick),
            (7.0, stick), (7.01, unstick), (7.5, unstick), (7.51, release), (8.0, release), (8.01, unfocus),
            (8.5, unfocus), (8.51, refocus), (9.0, refocus), (9.01, newsession), (10.0, newsession)]


def _press(keys: list, t: float, s: HeadsetState, hand: str, button: str, hold: float = 0.3) -> HeadsetState:
    """Append a press of ``hand``'s ``button`` (``primary`` / ``secondary``) at ``t`` held ``hold`` s; returns ``s``."""
    down = replace(s, **{hand: replace(getattr(s, hand), **{button: True})})
    keys += [(t, s), (t + 0.01, down), (t + hold, down), (t + hold + 0.01, s)]
    return s


def episodes_scenario() -> list:
    """Armed motion and episodes for recording tests (docs/collect.md): per episode Y (arm), A (start; the sim
    resets, which disarms), Y again, engage the right grip, reach / twist / squeeze, walk 0.4 m, turn the head,
    release, drive with the left stick, B tap (stop, kept). Three episodes, the third aborted with X mid-reach."""
    keys: list = []
    s = HeadsetState()
    t = 0.0
    keys.append((t, s))
    for k in range(3):
        sign = 1.0 if k % 2 == 0 else -1.0
        t += 1.5
        _press(keys, t, s, "left", "secondary")                 # Y: arm
        t += 1.5
        _press(keys, t, s, "right", "primary")                  # A: start the episode (the sim resets: disarmed)
        t += 5.0
        _press(keys, t, s, "left", "secondary")                 # Y: arm again after the reset
        t += 1.2
        grip = replace(s, right=replace(s.right, grip=1.0))
        keys += [(t, s), (t + 0.01, grip)]
        t += 1.0
        reach = replace(grip, right=replace(grip.right, pos=(0.50, -0.2 + 0.10 * sign, -0.22),
                                            rpy=(0.3 * sign, -0.2, 0.25 * sign), trigger=0.9))
        keys.append((t, grip))
        t += 3.0
        keys.append((t, reach))
        if k == 2:                                              # X mid-episode: disable, the episode is aborted
            t += 0.8
            _press(keys, t, reach, "left", "primary", hold=0.5)
            t += 1.5
            keys.append((t, replace(reach, right=replace(reach.right, grip=0.0))))
            s = HeadsetState()
            t += 0.5
            keys.append((t, s))
            continue
        walk = replace(reach, head_pos=(0.4, 0.0, 0.0), right=replace(reach.right, pos=(0.90, -0.2 + 0.1 * sign, -0.22)))
        t += 3.0
        keys.append((t, walk))
        turn = replace(walk, head_yaw=0.35 * sign)
        t += 2.0
        keys.append((t, turn))
        t += 0.8
        release = replace(turn, right=replace(turn.right, grip=0.0, trigger=0.0))
        keys += [(t, turn), (t + 0.01, release)]
        t += 0.8
        drive = replace(release, left=replace(release.left, stick=(0.0, 0.8)))
        keys += [(t, release), (t + 0.01, drive)]
        t += 1.5
        keys += [(t, drive), (t + 0.01, release)]
        t += 0.8
        _press(keys, t, release, "right", "secondary")          # B tap: stop, kept
        t += 1.0
        s = replace(release, head_pos=(0.0, 0.0, 0.0), head_yaw=0.0,
                    right=replace(release.right, pos=HandState().pos, rpy=(0.0, 0.0, 0.0)))
        keys += [(t, release), (t + 1.0, s)]
        t += 1.0
    keys.append((t + 2.0, s))
    return keys


def record_scenario() -> list:
    """A sim teleop session to record and replay: arm (Y), reach with the right hand while looking around, release;
    A (an episode: the sim resets, the AR view re-anchors, teleop is left disarmed), arm again, reach, B tap (the
    episode kept), idle."""
    s = HeadsetState()
    y = replace(s, left=replace(s.left, secondary=True))
    grip = replace(s, right=replace(s.right, grip=1.0))
    reach = replace(grip, right=replace(grip.right, pos=(0.55, -0.15, -0.15), trigger=0.9), head_yaw=0.35,
                    head_pitch=0.25)
    look = replace(reach, head_yaw=-0.3, head_pos=(0.2, 0.1, 0.0), right=replace(reach.right, pos=(0.7, -0.05, -0.2)))
    release = replace(look, right=replace(look.right, grip=0.0, trigger=0.0))
    back = replace(release, head_yaw=0.0, head_pitch=0.1, head_pos=(0.0, 0.0, 0.0),
                   right=replace(release.right, pos=(0.35, -0.2, -0.35)))
    a = replace(back, align=True)
    y2 = replace(back, left=replace(back.left, secondary=True))
    grip2 = replace(back, right=replace(back.right, grip=1.0))
    reach2 = replace(grip2, right=replace(grip2.right, pos=(0.5, -0.3, -0.1), rpy=(0.0, 0.3, -0.4), trigger=1.0),
                     head_yaw=-0.25, head_pitch=0.3)
    release2 = replace(reach2, right=replace(reach2.right, grip=0.0))
    b = replace(release2, right=replace(release2.right, secondary=True))
    return [(0.0, s), (1.5, s), (1.51, y), (2.0, y), (2.01, s), (2.5, s), (2.51, grip), (3.0, grip), (5.0, reach),
            (7.0, look), (7.5, look), (7.51, release), (8.5, back), (9.0, back), (9.01, a), (9.3, a), (9.31, back),
            (11.5, back), (11.51, y2), (12.0, y2), (12.01, back), (12.5, back), (12.51, grip2), (13.0, grip2),
            (15.0, reach2), (15.5, reach2), (15.51, release2), (16.0, release2), (16.01, b), (16.3, b),
            (16.31, release2), (18.0, release2)]


def two_hands_scenario(lead: float = 10.0) -> list:
    """Two hands, two arms (a bimanual robot, ``--hands``): idle for ``lead`` s (time to start the teleop), Y arms,
    the right grip engages and reaches 10 cm forward, the left engages too and lifts 10 cm while squeezing its
    trigger, the right releases, then the left; idle until 40 s after the lead."""
    idle = HeadsetState()
    y = replace(idle, left=replace(idle.left, secondary=True))
    r = replace(idle, right=replace(idle.right, grip=1.0))
    r_move = replace(r, right=replace(r.right, pos=(0.45, -0.2, -0.35)))
    rl = replace(r_move, left=replace(r_move.left, grip=1.0))
    rl_move = replace(rl, left=replace(rl.left, pos=(0.35, 0.2, -0.25), trigger=0.6))
    l_only = replace(rl_move, right=replace(rl_move.right, grip=0.0))
    none = replace(l_only, left=replace(l_only.left, grip=0.0))
    t = lead
    return [(0.0, idle), (t, idle), (t + 0.01, y), (t + 0.2, y), (t + 0.21, idle), (t + 1.0, idle), (t + 1.01, r),
            (t + 1.5, r), (t + 3.0, r_move), (t + 4.0, r_move), (t + 4.01, rl), (t + 4.5, rl), (t + 6.0, rl_move),
            (t + 8.0, rl_move), (t + 8.01, l_only), (t + 9.0, l_only), (t + 9.01, none), (t + 40.0, none)]


def hands_close_scenario(lead: float = 10.0) -> list:
    """Two hands brought close (a bimanual robot: the arm-proximity term, the elbows' swivel): idle for ``lead`` s,
    Y arms, both grips engage; the hands meet in front (each 16 cm in, 5 cm forward: a handover), hold; the left
    reaches across under the right arm (8 cm back, 26 cm in, 10 cm up) while the right reaches forward and in, hold;
    both return, hold, release; idle until 40 s after the lead."""
    idle = HeadsetState()
    y = replace(idle, left=replace(idle.left, secondary=True))
    lp, rp = idle.left.pos, idle.right.pos

    def at(dl, dr, grip=1.0):
        return replace(idle, left=replace(idle.left, grip=grip, pos=tuple(a + b for a, b in zip(lp, dl))),
                       right=replace(idle.right, grip=grip, pos=tuple(a + b for a, b in zip(rp, dr))))
    both = at((0, 0, 0), (0, 0, 0))
    meet = at((0.05, -0.16, 0.0), (0.05, 0.16, 0.0))
    across = at((-0.08, -0.26, 0.10), (0.10, 0.16, 0.08))
    none = at((0, 0, 0), (0, 0, 0), grip=0.0)
    t = lead
    return [(0.0, idle), (t, idle), (t + 0.01, y), (t + 0.2, y), (t + 0.21, idle), (t + 1.0, idle), (t + 1.01, both),
            (t + 2.0, both), (t + 5.0, meet), (t + 8.0, meet), (t + 11.0, across), (t + 14.0, across),
            (t + 17.0, both), (t + 19.0, both), (t + 19.01, none), (t + 40.0, none)]



def turned(s: HeadsetState, yaw: float) -> HeadsetState:
    """The whole operator turned by ``yaw`` about the LOCAL origin's vertical: the head and both hands (positions
    and orientations) rotate together."""
    rz = F.rot_z(yaw)

    def hand(h: HandState) -> HandState:
        return replace(h, pos=tuple(float(x) for x in rz @ np.asarray(h.pos, float)),
                       rpy=(h.rpy[0], h.rpy[1], h.rpy[2] + yaw))
    return replace(s, head_pos=tuple(float(x) for x in rz @ np.asarray(s.head_pos, float)),
                   head_yaw=s.head_yaw + yaw, left=hand(s.left), right=hand(s.right))


def mirror_scenario(lead: float = 10.0) -> list:
    """Mirror pairing (two hands, ``--mirror auto``): idle for ``lead`` s, Y arms (registers: the operator faces
    where the robot faces), the operator turns round to face the robot, the right grip engages (mirrored: it drives
    the robot's left arm) and reaches 10 cm forward with the trigger squeezed, the operator turns back halfway while
    still engaged (the pairing holds), releases, turns back fully and the right grip engages again (straight: the
    right arm) and reaches; idle until 40 s after the lead."""
    idle = HeadsetState()
    y = replace(idle, left=replace(idle.left, secondary=True))
    back = turned(idle, math.pi)
    g = replace(back, right=replace(back.right, grip=1.0))
    reach = replace(g, right=replace(g.right, pos=tuple(np.add(g.right.pos, (-0.1, 0.0, 0.0))), trigger=0.7))
    half = turned(replace(reach, head_yaw=0.0, head_pos=(0.0, 0.0, 0.0), left=idle.left,
                          right=replace(idle.right, grip=1.0, trigger=0.7, pos=(0.45, -0.2, -0.35))), math.pi / 2)
    released = replace(half, right=replace(half.right, grip=0.0, trigger=0.0))
    front = idle
    g2 = replace(front, right=replace(front.right, grip=1.0))
    reach2 = replace(g2, right=replace(g2.right, pos=(0.45, -0.2, -0.35)))
    t = lead
    return [(0.0, idle), (t, idle), (t + 0.01, y), (t + 0.2, y), (t + 0.21, idle), (t + 0.5, idle), (t + 1.5, back),
            (t + 1.8, back), (t + 1.81, g), (t + 2.0, g), (t + 2.8, reach), (t + 3.0, reach), (t + 3.8, half),
            (t + 4.0, half), (t + 4.01, released), (t + 4.3, released), (t + 5.0, front), (t + 5.2, front),
            (t + 5.21, g2), (t + 5.4, g2), (t + 6.2, reach2), (t + 40.0, reach2)]


def elbow_scenario(lead: float = 10.0) -> list:
    """The elbow-swivel hint (``--body``, the node's ``--elbow-hint``): idle for ``lead`` s, Y arms, the right grip
    engages, the right elbow swivels +0.6 rad about the shoulder → wrist axis (hand still), back to +0.2, releases;
    engages again (a new seq: the change restarts at zero) and swivels -0.4 rad; idle until 40 s after the lead."""
    idle = HeadsetState()
    y = replace(idle, left=replace(idle.left, secondary=True))
    g = replace(idle, right=replace(idle.right, grip=1.0))
    out = replace(g, elbow_swivel=(0.0, 0.6))
    back = replace(g, elbow_swivel=(0.0, 0.2))
    rel = replace(back, right=replace(back.right, grip=0.0))
    g2 = replace(back, right=replace(back.right, grip=1.0))
    in2 = replace(g2, elbow_swivel=(0.0, -0.2))
    t = lead
    return [(0.0, idle), (t, idle), (t + 0.01, y), (t + 0.2, y), (t + 0.21, idle), (t + 1.0, idle), (t + 1.01, g),
            (t + 1.5, g), (t + 2.5, out), (t + 3.0, out), (t + 3.5, back), (t + 4.0, back), (t + 4.01, rel),
            (t + 4.5, rel), (t + 4.51, g2), (t + 5.0, g2), (t + 6.0, in2), (t + 40.0, in2)]



SCENARIOS: dict[str, Callable[[], list]] = {"demo": demo_scenario, "record": record_scenario,
                                            "idle": lambda: [(0.0, HeadsetState())], "episodes": episodes_scenario,
                                            "two_hands": two_hands_scenario, "hands_close": hands_close_scenario,
                                            "mirror": mirror_scenario, "elbow": elbow_scenario}


# -- haptics: the headset's player, ported -----------------------------------------------------------------------

HAPTICS_SCHEMA = "teleop.quest_haptics.v1"


class HapticsPlayer:
    """A port of quest_env native/haptics.h (``haptics::Player``): the node's ``teleop.quest_haptics.v1`` lines in,
    vibration commands out, one ``tick`` per XR frame. Pulses play once (by id, per epoch), ``delay_s`` after they
    are first seen; a pulse older than ``MAX_AGE_S`` on arrival is skipped; a newer group on a hand replaces its
    pulses still waiting. A level plays (re-applied in ``LEVEL_CHUNK_S`` pieces) until its line's ``ttl_s`` runs out;
    pulses play over it. ``played`` logs every command (``t``, ``hand``, ``action`` apply / stop, ``amplitude``,
    ``duration_s``, ``frequency_hz``, ``id``: the pulse's, None for a level)."""

    MAX_AGE_S = 0.5
    MAX_TTL_S = 1.0
    LEVEL_CHUNK_S = 0.1
    LEVEL_REFRESH_S = 0.05

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.epoch: Optional[str] = None
        self.last_id = 0
        self.lines = 0
        self.bad = 0
        self.connected = False             # the stream is up (HapticsStream)
        self.played: list[dict] = []
        self._reset_hands()

    def _reset_hands(self) -> None:
        self.pending = {"left": [], "right": []}          # [(start, pulse)]
        self.level = {"left": (0.0, 0.0, 0.0), "right": (0.0, 0.0, 0.0)}   # (amplitude, frequency, until)
        self.playing = {"left": None, "right": None}       # ("pulse", end) | ("level", amplitude, refresh)

    def block(self) -> dict:
        """The ``haptics`` block of each frame (the node's capability check and ack)."""
        with self.lock:
            return {"schema": HAPTICS_SCHEMA, "epoch": self.epoch, "last_id": self.last_id,
                    "connected": self.connected}

    def receive(self, line: str, now: Optional[float] = None) -> bool:
        now = self.clock() if now is None else now
        try:
            m = json.loads(line)
            ok = isinstance(m, dict) and m.get("schema") == HAPTICS_SCHEMA and isinstance(m.get("hands"), dict)
        except ValueError:
            ok = False
        with self.lock:
            if not ok:
                self.bad += 1
                return False
            self.lines += 1
            epoch = str(m.get("epoch"))
            if epoch != self.epoch:
                self.epoch, self.last_id = epoch, 0
                self._reset_hands()
            ttl = min(max(float(m.get("ttl_s") or 0.0), 0.0), self.MAX_TTL_S)
            top = self.last_id
            for hand in ("left", "right"):
                h = m["hands"].get(hand)
                if not isinstance(h, dict):
                    continue
                lv = min(max(float(h.get("level") or 0.0), 0.0), 1.0)
                self.level[hand] = (lv, float(h.get("frequency_hz") or 0.0), now + ttl)
                for p in sorted(h.get("pulses") or [], key=lambda p: p["id"]):
                    if p["id"] <= self.last_id:
                        continue
                    top = max(top, p["id"])
                    if float(p.get("age_s") or 0.0) > self.MAX_AGE_S:
                        continue
                    q = self.pending[hand]
                    q[:] = [x for x in q if x[1]["group"] == p["group"]]     # a newer group replaces the waiting
                    q.append((now + float(p.get("delay_s") or 0.0), p))
            self.last_id = top
            return True

    def tick(self, now: Optional[float] = None) -> list[dict]:
        now = self.clock() if now is None else now
        out = []
        with self.lock:
            for hand in ("left", "right"):
                q, cur = self.pending[hand], self.playing[hand]
                due = [x for x in q if x[0] <= now]
                if due:
                    q[:] = [x for x in q if x[0] > now]
                    p = due[-1][1]
                    out.append({"t": now, "hand": hand, "action": "apply", "amplitude": p["amplitude"],
                                "duration_s": p["duration_s"], "frequency_hz": p.get("frequency_hz", 0.0),
                                "id": p["id"]})
                    self.playing[hand] = ("pulse", now + p["duration_s"])
                    continue
                if cur is not None and cur[0] == "pulse" and now < cur[1]:
                    continue
                amp, freq, until = self.level[hand]
                if amp > 0 and now < until:
                    if cur is None or cur[0] != "level" or cur[1] != amp or now >= cur[2]:
                        out.append({"t": now, "hand": hand, "action": "apply", "amplitude": amp,
                                    "duration_s": self.LEVEL_CHUNK_S, "frequency_hz": freq, "id": None})
                        self.playing[hand] = ("level", amp, now + self.LEVEL_REFRESH_S)
                elif cur is not None:
                    if cur[0] == "level":
                        out.append({"t": now, "hand": hand, "action": "stop", "amplitude": 0.0, "duration_s": 0.0,
                                    "frequency_hz": 0.0, "id": None})
                    self.playing[hand] = None
            self.played += out
        return out

    def disconnected(self) -> None:
        """The stream lost: waiting pulses and levels dropped; a level playing gets a stop at the next tick (a pulse
        ends by itself), as the native player."""
        with self.lock:
            self.connected = False
            self.pending = {"left": [], "right": []}
            self.level = {"left": (0.0, 0.0, 0.0), "right": (0.0, 0.0, 0.0)}


class HapticsStream:
    """The headset's haptics client, like quest_env native/haptics.h's: one long ``GET <url>`` (the node's
    ``/haptics?stream=1``), each line into ``player.receive``; reconnects (250 ms, then up to 2 s) and drops the
    connection after ``STALL_S`` without a byte (the node writes a line at least every second)."""

    STALL_S = 2.0

    def __init__(self, url: str, player: HapticsPlayer):
        self.url, self.player = url, player
        self.error: Optional[str] = None
        self.connects = 0
        self._stop = threading.Event()
        self._resp = None
        self._thread = threading.Thread(target=self._run, name="fake-headset-haptics", daemon=True)

    def start(self) -> "HapticsStream":
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        r = self._resp
        if r is not None:
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        self._thread.join(3)

    def _run(self) -> None:
        import urllib.request     # noqa: PLC0415
        backoff = 0.25
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=self.STALL_S) as r:
                    self._resp = r
                    self.connects += 1
                    with self.player.lock:
                        self.player.connected = True
                    backoff = 0.25
                    for raw in r:
                        if self._stop.is_set():
                            break
                        self.player.receive(raw.decode(errors="replace"))
            except Exception as e:  # noqa: BLE001 (any transport error: reconnect)
                self.error = f"{type(e).__name__}: {e}"[:200]
            finally:
                self._resp = None
            self.player.disconnected()
            self._stop.wait(backoff)
            backoff = min(2.0, backoff * 2)


# -- the server ------------------------------------------------------------------------------------


class FakeHeadsetServer:
    """TCP server sending one JSON line per frame to every connected client (like the native app). With
    ``haptics_url`` (the node's ``/haptics?stream=1``) it plays the node's haptics like the headset: ``self.haptics``
    (a ``HapticsPlayer``) fed by ``self.haptics_stream``, ticked once per frame sent, and every frame carries the
    ``haptics`` block. Without: an older build (no block)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 0, haptics_url: Optional[str] = None):
        self.sock = socket.create_server((host, port))
        self.port = self.sock.getsockname()[1]
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()
        self.sent = 0
        self.haptics: Optional[HapticsPlayer] = None
        self.haptics_stream: Optional[HapticsStream] = None
        if haptics_url:
            self.start_haptics(haptics_url)
        self._stop = threading.Event()
        threading.Thread(target=self._accept, name="fake-headset-accept", daemon=True).start()

    def start_haptics(self, url: str) -> HapticsPlayer:
        """Play the node's haptics from its stream at ``url`` from now on (the frames carry the block)."""
        self.haptics = HapticsPlayer()
        self.haptics_stream = HapticsStream(url, self.haptics).start()
        return self.haptics

    def _accept(self):
        while not self._stop.is_set():
            try:
                c, _ = self.sock.accept()
            except OSError:
                return
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                self.clients.append(c)

    @property
    def n_clients(self) -> int:
        with self.lock:
            return len(self.clients)

    def send(self, frame: dict) -> None:
        if self.haptics is not None:
            self.haptics.tick()
            frame = {**frame, "haptics": self.haptics.block()}
        line = (json.dumps(frame, separators=(",", ":")) + "\n").encode()
        with self.lock:
            for c in list(self.clients):
                try:
                    c.sendall(line)
                except OSError:
                    self.clients.remove(c)
                    c.close()
        self.sent += 1

    def drop_clients(self) -> None:
        with self.lock:
            for c in self.clients:
                try:
                    c.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                c.close()
            self.clients.clear()

    def close(self) -> None:
        self._stop.set()
        if self.haptics_stream is not None:
            self.haptics_stream.close()
        self.drop_clients()
        self.sock.close()


def play(server: FakeHeadsetServer, keyframes: list, rate: float = 72.0, *, loop: bool = False,
         headset: Optional[Headset] = None, stop: Optional[threading.Event] = None,
         pause: Callable[[float], bool] = lambda t: False,
         on_tick: Optional[Callable[[float], None]] = None) -> Headset:
    """Send the scenario in real time at ``rate``; ``pause(t)`` True withholds the frame (a network outage: the
    headset itself sees no gap). ``on_tick(t)`` runs before each frame (tests: drop the clients, ...)."""
    hs = headset or Headset()
    stop = stop or threading.Event()
    end = keyframes[-1][0]
    t0 = time.monotonic()
    k = 0
    while not stop.is_set():
        t = k / rate
        if t > end:
            if not loop:
                break
            t0, k, t = time.monotonic(), 0, 0.0
        if on_tick is not None:
            on_tick(t)
        frame = hs.frame(sample(keyframes, t), time.monotonic())   # the headset runs on during an outage
        if not pause(t):
            server.send(frame)
        k += 1
        delay = t0 + k / rate - time.monotonic()
        if delay > 0:
            stop.wait(delay)
    return hs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9391)
    ap.add_argument("--rate", type=float, default=72.0)
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="demo")
    ap.add_argument("--loop", action="store_true", help="repeat the scenario")
    ap.add_argument("--standard", action="store_true", help="intent in the standard scheme, not body-frozen")
    ap.add_argument("--ar", metavar="URL", help="emulate the robot AR view against this sim stack's frontend (the "
                                                "robot_ar block of each record)")
    ap.add_argument("--body", choices=("off", "core", "all"), default="off",
                    help="add the body tracking block (quest_env BODY_TRACKING.md) with these joints")
    ap.add_argument("--haptics", metavar="URL",
                    help="play the node's haptics like the headset: its GET /haptics?stream=1 URL (e.g. "
                         "http://127.0.0.1:8080/nodes/quest_teleop/haptics?stream=1)")
    ap.add_argument("--wait-client", action="store_true",
                    help="start the scenario only once a client is connected")
    args = ap.parse_args()
    server = FakeHeadsetServer(args.host, args.port, haptics_url=args.haptics)
    print(f"fake headset on {args.host}:{server.port}, scenario {args.scenario} at {args.rate} Hz", flush=True)
    ar = ArView(SimResetSeq(args.ar)) if args.ar else None
    try:
        while args.wait_client and server.n_clients == 0:
            time.sleep(0.05)
        play(server, SCENARIOS[args.scenario](), args.rate, loop=args.loop,
             headset=Headset(not args.standard, ar=ar, body=None if args.body == "off" else args.body))
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == "__main__":
    main()
