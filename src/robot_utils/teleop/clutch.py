"""Per-controller clutch: decides, from the raw intent, whether a hand is driving the robot.

The Quest app already clutches (grip press/release, ``engagement_id``); this is the robot
side of the consumer contract, which the app cannot enforce for a remote robot:

- no input, stale input, an unfocused app or ``valid`` False all release the hand;
- after any such gap, a session change or an operator-frame realignment, the operator must
  release and grip again before the hand drives anything (``needs-release``) — an
  engagement already in progress is never adopted, since its delta was not taken from a
  pose this robot measured;
- the runner can also force that (``disarm()``), e.g. after a fault.

A hand that passes all of that is ``engaged``; ``engagement`` is a local counter that
changes with every accepted engagement, which is what pose anchors key on.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Hashable, Optional

import numpy as np

from ..quest.messages import HandIntent

# HandState.status values
NO_INPUT = "no-input"            # no fresh, focused intent at all
INVALID = "invalid"              # intent present, this hand not valid (tracking lost etc.)
RELEASED = "released"            # valid, not gripping; ready to engage
NEEDS_RELEASE = "needs-release"  # gripping, but must release first
ENGAGED = "engaged"

_ZERO3 = np.zeros(3)
_ZERO3.flags.writeable = False
_EYE3 = np.eye(3)
_EYE3.flags.writeable = False


@dataclass(frozen=True)
class HandState:
    """What one controller asks for this tick, after the clutch.

    ``delta_position``/``delta_rotation`` are the motion since this engagement began, in the
    operator frame; identity unless ``engaged``. ``trigger`` (0..1) is None unless the hand
    is valid, so a stale value can never be used.
    """

    side: str
    status: str
    engaged: bool = False
    just_engaged: bool = False
    just_released: bool = False
    engagement: int = 0
    delta_position: np.ndarray = field(default_factory=lambda: _ZERO3, repr=False)
    delta_rotation: np.ndarray = field(default_factory=lambda: _EYE3, repr=False)
    trigger: Optional[float] = None
    intent: Optional[HandIntent] = field(default=None, repr=False)

    @property
    def valid(self) -> bool:
        return self.status not in (NO_INPUT, INVALID)


class HandClutch:
    def __init__(self, side: str):
        self.side = side
        self._key: Optional[Hashable] = None
        self._armed = False
        self._accepted: Optional[int] = None   # Quest engagement_id being followed
        self._engaged = False
        self._counter = 0

    def disarm(self) -> None:
        """Drop any engagement and require a release before the next one."""
        self._armed = False
        self._accepted = None
        self._engaged = False

    def update(self, hand: Optional[HandIntent], key: Hashable = None) -> HandState:
        """``hand`` is None when there is no fresh, focused intent. ``key`` identifies the
        frame the deltas live in (session, operator-frame revision); a change disarms."""
        was = self._engaged
        state = self._update(hand, key)
        if was and not state.engaged:
            state = replace(state, just_released=True)
        return state

    def _update(self, hand: Optional[HandIntent], key: Hashable) -> HandState:
        if hand is None:
            self.disarm()
            return HandState(self.side, NO_INPUT)
        if not hand.valid:
            self.disarm()
            return HandState(self.side, INVALID, intent=hand)
        if key != self._key:
            self.disarm()
            self._key = key
        if not hand.engaged:
            self._accepted, self._engaged = None, False
            if hand.needs_release:
                self._armed = False
                return HandState(self.side, NEEDS_RELEASE, trigger=hand.trigger, intent=hand)
            self._armed = True
            return HandState(self.side, RELEASED, trigger=hand.trigger, intent=hand)
        just = False
        if hand.engagement_id != self._accepted:
            if not self._armed:
                self._engaged = False
                return HandState(self.side, NEEDS_RELEASE, trigger=hand.trigger, intent=hand)
            self._accepted, self._armed = hand.engagement_id, False
            self._counter += 1
            just = True
        self._engaged = True
        return HandState(self.side, ENGAGED, engaged=True, just_engaged=just,
                         engagement=self._counter, delta_position=hand.delta_position,
                         delta_rotation=hand.delta_rotation, trigger=hand.trigger, intent=hand)

