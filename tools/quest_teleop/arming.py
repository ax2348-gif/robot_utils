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
  LEFT controller inactive (the stop button gone); no command to send; the WBC's own row showing it disarmed after
  it processed our arming datagram, or not enabled. Nothing re-arms on its own: every arm is a new Y press.
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
    disable_retry_s: float = 1.0      # X held: POST /disable again this often while the WBC still shows enabled


class ArmLatch:
    """``on_frame`` per accepted frame, ``tick`` per control step; read ``armed`` / ``arm_seq``; drain ``events``
    (("armed", arm_seq) / ("disarmed", reason)) and ``requests`` ("enable" / "disable") after each call; report the
    HTTP results with ``enable_done`` / ``disable_done``; ``sent_arm`` with the seq of the arming datagram."""

    def __init__(self, cfg: LatchConfig = LatchConfig(), arm_seq: int = 0):
        self.cfg = cfg
        self.armed = False
        self.arm_seq = arm_seq            # the newest press's number (the WBC arms only above its floor)
        self.reason = "not armed yet"     # why not armed (the last disarm, or why a press was refused)
        self.y_ready = False              # Y seen released in a valid frame since the last disarm / discontinuity
        self.y_count = 0
        self.pending_arm = False          # a qualified press waits for our POST /enable
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

    def _spoil(self) -> None:
        self.y_ready, self.y_count, self.pending_arm = False, 0, False

    def disarm(self, reason: str) -> None:
        """Disarm now (if armed: one ("disarmed", reason) event); every disarm needs a fresh Y release and press."""
        self._spoil()
        if self.armed:
            self.armed = False
            self.arm_dgram_seq = None
            self.reason = reason
            self.events.append(("disarmed", reason))

    def _arm(self) -> None:
        self.arm_seq += 1
        self.presses += 1
        self.armed, self.reason = True, ""
        self.arm_dgram_seq = None
        self._spoil()                          # one arm per press
        self.events.append(("armed", self.arm_seq))

    def sent_arm(self, seq: int) -> None:
        """The datagram ``seq`` that first carried this arm (the WBC row's ``seq`` tells when it was processed)."""
        if self.armed and self.arm_dgram_seq is None:
            self.arm_dgram_seq = seq

    def enable_done(self, ok: bool, detail: str = "") -> None:
        if ok:
            self.just_enabled, self.disabled_by_client = True, False
        else:
            self.pending_arm = False
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
            self.disarm("X: disabled")
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
        if not y_pressed(frame):
            self.y_ready, self.y_count, self.pending_arm = True, 0, False
            return
        if not self.y_ready:                       # held since before a disarm, or already used for an arm
            if self.pending_arm:                   # waiting for our enable with Y held
                why = self._preconditions(frame, row, row_t, now, fresh)
                if why:
                    self.reason = f"arm refused: {why}"
                    self._spoil()
                elif self.just_enabled:
                    self._arm()
            return
        why = self._preconditions(frame, row, row_t, now, fresh)
        if why:
            self.reason = f"arm refused: {why}"
            self._spoil()
            return
        self.y_count += 1
        if self.y_count < self.cfg.y_frames:
            return
        st, _ = self._wbc(row, row_t, now)
        self.y_ready, self.y_count = False, 0     # this press is used, whatever follows
        if st in WBC_ENABLED or self.just_enabled:
            self._arm()
        elif st == WBC_DISABLED:
            self.pending_arm = True               # enable, then arm if Y is still held
            self.reason = "enabling the WBC"
            self._request("enable")

    def tick(self, now: float, fresh: bool, why_stale: str, row: Optional[dict], row_t: Optional[float]) -> None:
        """Per control step: stale / late / slow input disarms; so does the WBC's own row disagreeing."""
        if not fresh:
            if self.armed:
                self.disarm(why_stale or "headset input stale")
            else:
                self._spoil()
            return
        if row is None:
            return
        st = _num(row.get("state"))
        if st is not None and int(st) == WBC_FAULT:
            self.just_enabled = False
        if not self.armed or self.arm_dgram_seq is None:
            return
        if st is not None and int(st) not in WBC_ENABLED:
            self.disarm("the WBC is " + {WBC_DISABLED: "disabled", WBC_FAULT: "in a fault"}.get(int(st), "not enabled"))
            return
        a, seq = _num(row.get("armed")), _num(row.get("seq"))
        if a is not None and seq is not None and seq >= self.arm_dgram_seq and a == 0.0:
            self.disarm("the WBC disarmed (its /state disarm_reason)")
