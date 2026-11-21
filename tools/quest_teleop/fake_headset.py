#!/usr/bin/env python3
"""A fake Quest headset: serves ``teleop.quest_state.v1`` JSON lines on TCP (default port 9391) with scripted head
and hand motion, for tests and for a dry run of the client without the headset.

    python tools/quest_teleop/fake_headset.py [--port 9391] [--rate 72] [--scenario demo] [--loop]

The frames have the native app's shape (quest_env native/telemetry.cpp): ``session_id``, ``frame_seq``,
``input_focused``, ``action_sync_result``, ``head`` (tracking flags and a 16-number row-major LOCAL pose),
``hands.left/right`` (active, flags, pose, trigger, grip, joystick, buttons) and ``control_intent``, computed by
``NativeIntent``, a Python port of the native ``teleop::Intent`` (grip hysteresis 0.6/0.4, release required
after start, focus loss, gaps and session changes, the 18 % stick dead zone, body-frozen deltas).

Scenarios are lists of keyframes ``(t, state)``: poses are given in operator FLU (metres, radians) and converted
to OpenXR LOCAL with ``C``; values between keyframes are interpolated linearly (yaw too), booleans and
the session step. ``demo`` engages the right hand, moves it, walks forward, turns the head, uses the stick,
releases, loses focus and changes session.
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
    from quest_teleop import frames as F   # noqa: E402
else:
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
        aligned = (align and focused and not gap and left.grip <= 0.4 and right.grip <= 0.4 and head is not None
                   and self.realign(head))
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


def _rpy(r, p, y):
    return F.rot_z(y) @ F.rot_axis((0, 1, 0), p) @ F.rot_axis((1, 0, 0), r)


def to_local(pos_op, r_op):
    """Operator FLU pose -> OpenXR LOCAL (the inverse of C)."""
    return F.C.T @ np.asarray(pos_op, float), F.C.T @ r_op


def head_r_op(yaw: float, pitch: float = 0.0):
    """The head's orientation in operator axes for a level head facing ``yaw`` (identity head = ``C``)."""
    return F.rot_z(yaw) @ F.rot_axis((0, 1, 0), pitch) @ F.C


class Headset:
    """Builds native-shaped frames from a HeadsetState."""

    def __init__(self, body_frozen: bool = True, session_base: int = 1_700_000_000_000_000_000):
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
        for name, hs, hi in (("left", s.left, hands_in[0]), ("right", s.right, hands_in[1])):
            frame["hands"][name] = {
                "pose_source": "tracked_remote", "active": hs.tracked, "position_valid": hs.tracked,
                "orientation_valid": hs.tracked, "position_tracked": hs.tracked, "orientation_tracked": hs.tracked,
                "pose": F.matrix16_from_pose(hi.p, hi.r) if hs.tracked else None,
                "trigger": hs.trigger, "grip": hs.grip, "joystick": list(hs.stick),
                "buttons": {"primary": name == "right" and s.align, "secondary": False, "thumb_rest": False,
                            "stick": hs.click}}
        self.seq += 1
        return frame


# -- scenarios -------------------------------------------------------------------------------------


def _lerp(a, b, u):
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


SCENARIOS: dict[str, Callable[[], list]] = {"demo": demo_scenario, "idle": lambda: [(0.0, HeadsetState())]}


# -- the server ------------------------------------------------------------------------------------


class FakeHeadsetServer:
    """TCP server sending one JSON line per frame to every connected client (like the native app)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        self.sock = socket.create_server((host, port))
        self.port = self.sock.getsockname()[1]
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()
        self.sent = 0
        self._stop = threading.Event()
        threading.Thread(target=self._accept, name="fake-headset-accept", daemon=True).start()

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
    args = ap.parse_args()
    server = FakeHeadsetServer(args.host, args.port)
    print(f"fake headset on {args.host}:{server.port}, scenario {args.scenario} at {args.rate} Hz", flush=True)
    try:
        play(server, SCENARIOS[args.scenario](), args.rate, loop=args.loop, headset=Headset(not args.standard))
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == "__main__":
    main()
