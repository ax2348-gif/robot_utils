"""The client's side of the WBC's armed latch: the left controller's X and Y (README "Arming: Y and X"). Pure: no
sockets, the clock is passed in; ``teleop.TeleopClient`` runs the HTTP requests it asks for and sends the datagrams.

- **X** (``hands.left.buttons.primary``) is LEVEL-triggered and means **disable**: any accepted frame with X down
  disarms at once (the client sends ``armed: false``) and asks for ``POST /disable`` (again every
  ``disable_retry_s`` while X is held and the WBC row still shows it enabled). The teleop session stays open.
- **Y** (``hands.left.buttons.secondary``) **arms** (enabling first through ``POST /enable`` when the WBC is
  disabled) only on a deliberate press: Y seen released in a valid frame since the last disarm / disable /
  discontinuity, then held for ``y_frames`` (3) consecutive fresh valid frames with X up, both grips below
  ``grip_max`` (0.4), both sticks inside the native dead zone (0.18), and a fresh WBC row showing it enabled (or our
  enable just succeeded) without a fault. One arm per press: ``arm_seq`` += 1; Y must be released before the next.
  Anything failing during the press spoils it (release Y and press again).
- **Disarm** (``armed: false`` at once, then in every datagram): X; any discontinuity (``QuestGate``: reconnect,
  session, revision, gap); an invalid frame (unfocused, ...); stale, late or slow input (``QuestGate.fresh``); the
  LEFT controller inactive (the stop button gone); no command to send; the WBC's rows stale (newest older than
  ``wbc_stale_s``, 0.5 s); the WBC's own row showing it disarmed after it processed our arming datagram, or not
  enabled. Nothing re-arms on its own: every arm is a new Y press.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

WBC_DISABLED, WBC_ENABLING, WBC_FAULT = 0, 1, 5
WBC_ENABLED = (2, 3, 4)          # idle, tracking, holding (the WBC stream's state enum)
DEAD_ZONE = 0.18                 # the native stick dead zone (quest_env teleop::Intent)


def _hand(frame: dict, name: str) -> dict:
    h = (frame.get("hands") or {}).get(name)
    return h if isinstance(h, dict) else {}


def button(frame: dict, hand: str, name: str) -> bool:
    b = _hand(frame, hand).get("buttons")
    return isinstance(b, dict) and b.get(name) is True


def x_pressed(frame: dict) -> bool:
    return button(frame, "left", "primary")


def y_pressed(frame: dict) -> bool:
    return button(frame, "left", "secondary")


def _num(x) -> Optional[float]:
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) else None


def sticks_neutral(frame: dict, dead_zone: float = DEAD_ZONE) -> bool:
    """Both controllers' raw joysticks inside the dead zone (a missing or malformed stick is not neutral)."""
    for name in ("left", "right"):
        js = _hand(frame, name).get("joystick")
        if not isinstance(js, (list, tuple)) or len(js) < 2:
            return False
        v = [_num(x) for x in js[:2]]
        if any(x is None or abs(x) > dead_zone for x in v):
            return False
    return True


def grips_released(frame: dict, grip_max: float) -> bool:
    for name in ("left", "right"):
        g = _num(_hand(frame, name).get("grip"))
        if g is None or g >= grip_max:
            return False
    return True


@dataclass
class LatchConfig:
    y_frames: int = 3                 # Y held this many consecutive fresh valid frames
    grip_max: float = 0.4             # both grips below this to arm
    dead_zone: float = DEAD_ZONE
    robot_stale_s: float = 0.3        # the WBC row must be younger than this to arm
    wbc_stale_s: float = 0.5          # armed: the newest WBC row older than this disarms (0.3 tripped on WBC process
                                      # stalls on the Pi, 2026-11-21)
    disable_retry_s: float = 1.0      # X held: POST /disable again this often while the WBC still shows enabled
    enable_timeout_s: float = 2.0     # a press on a disabled WBC: cancelled if not enabled (and armed) by then


class ArmLatch:
    """``on_frame`` per accepted frame, ``tick`` per control step; read ``armed`` / ``arm_seq``; drain ``events``
    (("armed", arm_seq) / ("disarmed", reason)) and ``requests`` ("enable" / "disable") after each call; report the
    HTTP results with ``enable_done`` / ``disable_done``; ``sent_arm`` with the seq of the arming datagram."""

    def __init__(self, cfg: LatchConfig = LatchConfig(), arm_seq: int = 0):
        self.cfg = cfg
        self.armed = False
        self.arm_seq = arm_seq            # the newest press's number (the WBC arms only above its floor)
        self.reason = "not armed: press Y"  # the current state: armed, enabling, why not armed / why cancelled
        self.y_ready = False              # Y seen released in a valid frame since the last disarm / discontinuity
        self.y_count = 0
        # a deliberate press on a disabled WBC: {"t": when, "enabled_seq": None | the datagram seq when our enable
        # was reported done}; it waits through enabling and completes on a fresh enabled row after that seq
        self.pending: Optional[dict] = None
        self.just_enabled = False         # our POST /enable succeeded (until a disable / fault / disarm)
        self.x_down = False
        self.disabled_by_client = False   # the WBC is disabled because of X (the teleop node keeps running)
        self.arm_dgram_seq: Optional[int] = None   # the datagram seq that carried the arm
        self.events: list = []
        self.requests: list = []
        self.presses = 0                  # arms (one per press)
        self._last_disable = -math.inf

    # -- transitions ----------------------------------------------------------------------------------------

    def set_floor(self, floor: int) -> None:
        """The WBC's arm_seq floor (``POST /session``): presses are numbered above it."""
        self.arm_seq = max(self.arm_seq, int(floor or 0))

    @property
    def pending_arm(self) -> bool:
        return self.pending is not None

    def _cancel(self, why: str) -> None:
        if self.pending is not None:
            self.pending = None
            self.reason = f"arm cancelled: {why}"

    def _spoil(self, why: str = "") -> None:
        """The current press is spoiled (Y must be released and pressed again); a pending arm is cancelled."""
        self.y_ready, self.y_count = False, 0
        self._cancel(why or "the press was interrupted")

    def disarm(self, reason: str) -> None:
        """Disarm now (if armed: one ("disarmed", reason) event); every disarm needs a fresh Y release and press. A
        pending arm is cancelled with this reason."""
        self._spoil(reason)
        if self.armed:
            self.armed = False
            self.arm_dgram_seq = None
            self.reason = reason
            self.events.append(("disarmed", reason))

    def _arm(self) -> None:
        self.arm_seq += 1
        self.presses += 1
        self.armed, self.reason = True, "armed"
        self.arm_dgram_seq = None
        self.pending = None
        self._spoil()                          # one arm per press
        self.events.append(("armed", self.arm_seq))

    def sent_arm(self, seq: int) -> None:
        """The datagram ``seq`` that first carried this arm (the WBC row's ``seq`` tells when it was processed)."""
        if self.armed and self.arm_dgram_seq is None:
            self.arm_dgram_seq = seq

    def enable_done(self, ok: bool, detail: str = "", seq: Optional[int] = None) -> None:
        """Our ``POST /enable`` answered. ``seq``: the newest datagram seq sent when the answer was seen: a pending arm
        completes only on a WBC row that processed a later datagram (so it reflects the enable)."""
        if ok:
            self.just_enabled, self.disabled_by_client = True, False
            if self.pending is not None:
                self.pending["enabled_seq"] = -1 if seq is None else int(seq)
        elif self.pending is not None:
            self._cancel(f"enable refused: {detail}")
        else:
            self.reason = f"enable refused: {detail}"

    def disable_done(self, ok: bool, detail: str = "") -> None:
        if not ok:
            self.reason = f"disable failed: {detail}"

    def _request(self, what: str) -> None:
        if what not in self.requests:
            self.requests.append(what)

    # -- inputs ---------------------------------------------------------------------------------------------

    def _wbc(self, row: Optional[dict], row_t: Optional[float], now: float) -> tuple[Optional[int], Optional[str]]:
        """(the WBC state from a fresh row, None) or (None, why not usable)."""
        if row is None or row_t is None:
            return None, "no WBC state"
        if now - row_t > self.cfg.robot_stale_s:
            return None, f"WBC state stale ({now - row_t:.2f} s)"
        s = _num(row.get("state"))
        if s is None:
            return None, "no WBC state"
        return int(s), None

    def _preconditions(self, frame: dict, row, row_t, now: float, fresh: bool) -> Optional[str]:
        if not fresh:
            return "headset input not fresh"
        if not grips_released(frame, self.cfg.grip_max):
            return "a grip is pressed"
        if not sticks_neutral(frame, self.cfg.dead_zone):
            return "a stick is deflected"
        st, why = self._wbc(row, row_t, now)
        if why:
            return why
        if st == WBC_FAULT:
            return "the WBC is in a fault (X disables and clears it)"
        if st == WBC_ENABLING:
            return "the WBC is enabling"
        return None

    def on_frame(self, frame: dict, verdict, row: Optional[dict], row_t: Optional[float], now: float,
                 fresh: bool) -> None:
        """One accepted-or-not frame from ``QuestGate.check`` (in order)."""
        if not verdict.accept:
            return
        if x_pressed(frame):                       # level-triggered: any accepted frame, valid or not
            self.disarm("X: disabled")             # (a pending arm: "arm cancelled: X: disabled")
            if not self.reason.startswith("arm cancelled"):
                self.reason = "X: disabled (Y re-enables and arms)"
            self.just_enabled = False
            st, _ = self._wbc(row, row_t, now)
            if not self.x_down or (now - self._last_disable >= self.cfg.disable_retry_s and st != WBC_DISABLED):
                self._request("disable")
                self._last_disable = now
            self.x_down = True
            self.disabled_by_client = True
            return
        self.x_down = False
        if verdict.discontinuity:
            self.disarm(f"headset discontinuity ({verdict.reason})")
        if not verdict.valid:
            self.disarm(f"headset input invalid ({verdict.reason})")
            return
        if not _hand(frame, "left").get("active"):
            self.disarm("the left controller is not active (X / Y unavailable)")
            return
        if self.pending is not None:               # a press already made: Y released or held, it waits
            if not y_pressed(frame):
                self.y_ready, self.y_count = True, 0
            self._pending_step(frame, row, row_t, now, fresh)
            return
        if not y_pressed(frame):
            self.y_ready, self.y_count = True, 0
            return
        if not self.y_ready:                       # held since before a disarm, or already used for an arm
            return
        why = self._preconditions(frame, row, row_t, now, fresh)
        if why:
            self.reason = f"not armed: {why} (release Y and press again)"
            self._spoil()
            return
        self.y_count += 1
        if self.y_count < self.cfg.y_frames:
            return
        st, _ = self._wbc(row, row_t, now)
        self.y_ready, self.y_count = False, 0     # this press is used, whatever follows
        if st in WBC_ENABLED:
            self._arm()
        elif st == WBC_DISABLED:                  # enable, then arm when the WBC shows it enabled
            self.pending = {"t": now, "enabled_seq": None}
            self.reason = "enabling the WBC…"
            self._request("enable")

    def _pending_step(self, frame: Optional[dict], row, row_t, now: float, fresh: bool) -> None:
        """A pending arm: cancelled on a timeout or a fault row; completed on the first fresh row that processed a
        datagram sent after our enable succeeded and shows the WBC enabled, if the other preconditions hold then."""
        p = self.pending
        if now - p["t"] > self.cfg.enable_timeout_s:
            self._cancel(f"the WBC did not enable within {self.cfg.enable_timeout_s:g} s")
            return
        st, why = self._wbc(row, row_t, now)
        if st == WBC_FAULT:
            self._cancel("the WBC is in a fault (X disables and clears it)")
            return
        if p["enabled_seq"] is None or why is not None or st not in WBC_ENABLED:
            return                                 # still enabling (or the row is not usable yet): wait
        seq = _num(row.get("seq"))
        if seq is not None and seq <= p["enabled_seq"]:
            return                                 # this row predates our enable
        if frame is None:
            return                                 # complete on a frame: its input is checked
        if not fresh:
            self._cancel("headset input not fresh")
        elif not grips_released(frame, self.cfg.grip_max):
            self._cancel("a grip is pressed")
        elif not sticks_neutral(frame, self.cfg.dead_zone):
            self._cancel("a stick is deflected")
        else:
            self._arm()

    def tick(self, now: float, fresh: bool, why_stale: str, row: Optional[dict], row_t: Optional[float]) -> None:
        """Per control step: stale / late / slow input disarms; so does the WBC's own row disagreeing."""
        if not fresh:
            if self.armed:
                self.disarm(why_stale or "headset input stale")
            else:
                self._spoil(why_stale or "headset input stale")
            return
        if self.pending is not None:               # the timeout and a fault row, between frames too
            self._pending_step(None, row, row_t, now, fresh)
        if row is None or row_t is None or now - row_t > self.cfg.wbc_stale_s:
            if self.armed:                         # the WBC stream is gone or late: we cannot see what it does
                self.disarm("WBC state stale" if row is None or row_t is None else
                            f"WBC state stale ({now - row_t:.2f} s)")
            return
        st = _num(row.get("state"))
        if st is not None and int(st) == WBC_FAULT:
            self.just_enabled = False
        if not self.armed or self.arm_dgram_seq is None:
            return
        # only rows that reflect our arming datagram (their seq: the newest datagram the WBC processed) count: an
        # older row may predate our enable
        a, seq = _num(row.get("armed")), _num(row.get("seq"))
        if seq is None or seq < self.arm_dgram_seq:
            return
        if st is not None and int(st) not in WBC_ENABLED:
            self.disarm("the WBC is " + {WBC_DISABLED: "disabled", WBC_FAULT: "in a fault"}.get(int(st), "not enabled"))
            return
        if a is not None and a == 0.0:
            self.disarm("the WBC disarmed (its /state disarm_reason)")
