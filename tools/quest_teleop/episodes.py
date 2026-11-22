"""The right controller's A and B as episode buttons (docs/plans/data_collection_v1.md, "Decided"; the teleop
node acts on them, docs/quest_teleop_node.md "Episodes"). Pure: no sockets, the clock is the frames' arrival time.

- **A** (``hands.right.buttons.primary``) = start an episode: a deliberate press only, with the Y button's rigor
  (arming.py): A seen released in a valid frame first, then held for ``press_frames`` (3) consecutive fresh valid
  frames; one action per press (released before the next). A frame that is not valid (unfocused, a discontinuity,
  the right controller inactive) spoils the press, so a press made or held during focus loss never counts.
- **B** (``hands.right.buttons.secondary``) tap = stop the episode and keep it: B seen released first, then held
  for at least ``press_frames`` fresh valid frames and released (in a valid frame) less than ``tap_max_s`` (2 s)
  after it went down. Held for ``tap_max_s`` instead: one ``B_hold`` (the headset stops its telemetry after a 2 s
  hold; the episode ends with it). Any frame that is not fresh and valid while B is down spoils the tap.

``on_frame`` per frame from ``QuestGate.check``; read and clear ``events`` ("A" / "B" / "B_hold") after each call.
``edges`` lists the raw edges of all four buttons (``(name, pressed)``, valid frames only) for the event log.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

BUTTONS = (("A", "right", "primary"), ("B", "right", "secondary"), ("X", "left", "primary"),
           ("Y", "left", "secondary"))


def _pressed(frame: dict, hand: str, name: str) -> bool:
    h = (frame.get("hands") or {}).get(hand)
    b = h.get("buttons") if isinstance(h, dict) else None
    return isinstance(b, dict) and b.get(name) is True


def _active(frame: dict, hand: str) -> bool:
    h = (frame.get("hands") or {}).get(hand)
    return isinstance(h, dict) and bool(h.get("active"))


@dataclass
class ButtonConfig:
    press_frames: int = 3       # A / B held this many consecutive fresh valid frames
    tap_max_s: float = 2.0      # B released before this = a tap; held this long = a hold


class EpisodeButtons:
    def __init__(self, cfg: ButtonConfig = ButtonConfig()):
        self.cfg = cfg
        self.events: list = []
        self.edges: list = []
        self.state = {name: False for name, _, _ in BUTTONS}   # the last valid frame's buttons
        self.a_ready = False
        self.a_count = 0
        self.b_ready = False
        self.b_down: Optional[float] = None    # arrival time of the first frame of this B press
        self.b_count = 0
        self.b_held = False                    # this press already reported B_hold

    def spoil(self) -> None:
        """Both presses void: each needs a release seen in a valid frame first."""
        self.a_ready, self.a_count = False, 0
        self.b_ready, self.b_down, self.b_count = False, None, 0

    def on_frame(self, frame: dict, verdict, now: float, fresh: bool) -> None:
        if not verdict.accept:
            return
        if not verdict.valid:
            self.spoil()
            return
        for name, hand, button in BUTTONS:
            p = _pressed(frame, hand, button)
            if p != self.state[name]:
                self.state[name] = p
                self.edges.append((name, p))
        if verdict.discontinuity or not _active(frame, "right"):
            self.spoil()
            if not _active(frame, "right"):
                return
        self._a(_pressed(frame, "right", "primary"), fresh)
        self._b(_pressed(frame, "right", "secondary"), now, fresh)

    def _a(self, pressed: bool, fresh: bool) -> None:
        if not pressed:
            self.a_ready, self.a_count = True, 0
            return
        if not self.a_ready:
            return
        if not fresh:
            self.a_ready, self.a_count = False, 0
            return
        self.a_count += 1
        if self.a_count >= self.cfg.press_frames:
            self.a_ready, self.a_count = False, 0     # one action per press
            self.events.append("A")

    def _b(self, pressed: bool, now: float, fresh: bool) -> None:
        if not pressed:
            if (self.b_down is not None and not self.b_held and self.b_count >= self.cfg.press_frames
                    and now - self.b_down < self.cfg.tap_max_s):
                self.events.append("B")
            self.b_ready, self.b_down, self.b_count, self.b_held = True, None, 0, False
            return
        if self.b_down is None:
            if not self.b_ready or not fresh:
                self.b_ready = False
                return
            self.b_ready, self.b_down, self.b_count, self.b_held = False, now, 1, False
            return
        if not fresh:
            self.b_down, self.b_count = None, 0      # spoiled: release first
            return
        self.b_count += 1
        if not self.b_held and now - self.b_down >= self.cfg.tap_max_s:
            self.b_held = True
            self.events.append("B_hold")
