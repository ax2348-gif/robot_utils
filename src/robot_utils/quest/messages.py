"""Typed, validated view of the Quest app's ``teleop.quest_state.v1`` JSON lines.

Only the operator intent (``control_intent``, schema ``teleop.operator_intent.v1``) is parsed
into types; everything else in the record (raw head and controller poses, buttons, the
on-headset demo) stays available as ``QuestFrame.raw``.

Validation is strict about what a robot would act on and lenient about the rest:

- a record without the expected top-level schema, ``frame_seq`` or ``session_id`` is
  rejected (``ProtocolError``);
- a missing or malformed ``control_intent`` leaves ``QuestFrame.intent`` as ``None``;
- a hand whose numbers are missing or non-finite (the app sends ``null`` for NaN), or whose
  rotation is not a rotation, is marked invalid (``HandIntent.valid`` False, with
  ``invalid_reason``) even if the app said valid;
- a malformed ``base_velocity`` is reported as zero with ``twist_valid`` False.

Rotations are re-orthonormalised after the check: the app prints six significant digits.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

import numpy as np

STATE_SCHEMA = "teleop.quest_state.v1"
INTENT_SCHEMA = "teleop.operator_intent.v1"
SIDES = ("left", "right")

ROTATION_TOLERANCE = 1e-3  # max |R^T R - I| entry and |det R - 1| accepted before projecting
_ZERO3 = np.zeros(3)
_ZERO3.flags.writeable = False
_EYE3 = np.eye(3)
_EYE3.flags.writeable = False


class ProtocolError(ValueError):
    """A record that cannot be used at all (wrong schema, missing sequencing fields)."""


@dataclass(frozen=True)
class HandIntent:
    """One controller's clutched motion request, in the operator frame (``OperatorIntent.frame``).

    ``delta_position`` (m) and ``delta_rotation`` are the controller's motion since the current
    engagement began; a consumer applies them to a pose it measured when it saw that engagement
    start. ``trigger`` is the analog trigger, 0 released .. 1 pressed (the wire calls it
    ``gripper``); like everything else here it means nothing unless ``valid``.
    """

    valid: bool
    engaged: bool
    needs_release: bool
    engagement_id: int
    delta_position: np.ndarray = field(repr=False)
    delta_rotation: np.ndarray = field(repr=False)
    trigger: float
    invalid_reason: Optional[str] = None
    # body-frozen scheme only: the operator frame this engagement captured, in Quest LOCAL
    operator_base_position: Optional[np.ndarray] = field(default=None, repr=False)
    operator_base_rotation: Optional[np.ndarray] = field(default=None, repr=False)


@dataclass(frozen=True)
class OperatorIntent:
    session_id: str
    frame: str                    # "operator_base_flu": x forward, y left, z up
    scheme: str                   # "standard" or "body-frozen" (see the Quest app's DEMO.md)
    translation_limit_m: Optional[float]
    twist: np.ndarray = field(repr=False)    # wire "base_velocity": [forward m/s, left m/s, yaw rad/s]
    twist_valid: bool
    operator_frame_revision: int
    operator_from_quest: np.ndarray = field(repr=False)
    hands: Mapping[str, HandIntent]


@dataclass(frozen=True)
class QuestFrame:
    frame_seq: int
    session_id: str
    input_focused: bool
    intent: Optional[OperatorIntent]
    received_at: float            # time.monotonic() when the line arrived (consumer's clock)
    raw: Mapping[str, Any] = field(repr=False)
    intent_error: Optional[str] = None


def parse_frame(record: Any, received_at: float) -> QuestFrame:
    if not isinstance(record, dict):
        raise ProtocolError("record is not a JSON object")
    if record.get("schema") != STATE_SCHEMA:
        raise ProtocolError(f"schema {record.get('schema')!r}, expected {STATE_SCHEMA!r}")
    seq = record.get("frame_seq")
    session = record.get("session_id")
    if not _is_int(seq) or seq < 0:
        raise ProtocolError(f"bad frame_seq {seq!r}")
    if not isinstance(session, str):
        raise ProtocolError(f"bad session_id {session!r}")
    intent, error = None, None
    if "control_intent" not in record:
        error = "no control_intent"
    else:
        try:
            intent = parse_intent(record["control_intent"])
        except ProtocolError as e:
            error = str(e)
    return QuestFrame(frame_seq=seq, session_id=session,
                      input_focused=record.get("input_focused") is True,
                      intent=intent, received_at=received_at, raw=record, intent_error=error)


def parse_intent(obj: Any) -> OperatorIntent:
    if not isinstance(obj, dict):
        raise ProtocolError("control_intent is not an object")
    if obj.get("schema") != INTENT_SCHEMA:
        raise ProtocolError(f"control_intent schema {obj.get('schema')!r}, expected {INTENT_SCHEMA!r}")
    session = obj.get("session_id")
    revision = obj.get("operator_frame_revision")
    if not isinstance(session, str):
        raise ProtocolError(f"bad control_intent.session_id {session!r}")
    if not _is_int(revision):
        raise ProtocolError(f"bad operator_frame_revision {revision!r}")
    hands_obj = obj.get("hands")
    if not isinstance(hands_obj, dict):
        raise ProtocolError("control_intent.hands is not an object")
    frame_rot = _rotation(obj.get("operator_from_quest"))
    if frame_rot is None:
        raise ProtocolError("bad operator_from_quest")
    twist = _vector(obj.get("base_velocity"))
    limit = obj.get("translation_limit_m")
    return OperatorIntent(
        session_id=session,
        frame=str(obj.get("frame", "")),
        scheme=str(obj.get("scheme", "")),
        translation_limit_m=float(limit) if _is_finite(limit) else None,
        twist=twist if twist is not None else _ZERO3,
        twist_valid=twist is not None,
        operator_frame_revision=revision,
        operator_from_quest=frame_rot,
        hands={side: parse_hand(hands_obj.get(side)) for side in SIDES},
    )


def parse_hand(obj: Any) -> HandIntent:
    if not isinstance(obj, dict):
        return _invalid_hand("missing")
    eid = obj.get("engagement_id")
    eid = eid if _is_int(eid) else -1
    flags = dict(engaged=obj.get("engaged") is True, needs_release=obj.get("needs_release") is True,
                 engagement_id=eid)
    reason = None
    dp = _vector(obj.get("delta_position"))
    dr = _rotation(obj.get("delta_rotation"))
    trigger = obj.get("gripper")
    if obj.get("valid") is not True:
        reason = "reported invalid"
    elif eid < 0:
        reason = "bad engagement_id"
    elif dp is None:
        reason = "bad delta_position"
    elif dr is None:
        reason = "bad delta_rotation"
    elif not _is_finite(trigger) or not -0.01 <= trigger <= 1.01:
        reason = "bad gripper/trigger"
    if reason is not None:
        return HandIntent(valid=False, delta_position=_ZERO3, delta_rotation=_EYE3, trigger=0.0,
                          invalid_reason=reason, **flags)
    return HandIntent(valid=True, delta_position=dp, delta_rotation=dr,
                      trigger=min(max(float(trigger), 0.0), 1.0),
                      operator_base_position=_vector(obj.get("operator_base_in_quest_position")),
                      operator_base_rotation=_rotation(obj.get("operator_base_in_quest_rotation")),
                      **flags)


def _invalid_hand(reason: str) -> HandIntent:
    return HandIntent(valid=False, engaged=False, needs_release=False, engagement_id=-1,
                      delta_position=_ZERO3, delta_rotation=_EYE3, trigger=0.0, invalid_reason=reason)


def _is_int(x: Any) -> bool:
    return isinstance(x, int) and not isinstance(x, bool)


def _is_finite(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _numbers(obj: Any, n: int) -> Optional[np.ndarray]:
    if not isinstance(obj, list) or len(obj) != n or not all(_is_finite(v) for v in obj):
        return None
    a = np.array(obj, dtype=float)
    a.flags.writeable = False
    return a


def _vector(obj: Any) -> Optional[np.ndarray]:
    return _numbers(obj, 3)


def _rotation(obj: Any) -> Optional[np.ndarray]:
    """A 9-element row-major rotation, checked and projected onto SO(3); None if not one."""
    a = _numbers(obj, 9)
    if a is None:
        return None
    m = a.reshape(3, 3)
    if (np.abs(m.T @ m - np.eye(3)).max() > ROTATION_TOLERANCE
            or abs(np.linalg.det(m) - 1.0) > ROTATION_TOLERANCE):
        return None
    u, _, vt = np.linalg.svd(m)
    r = u @ vt
    r.flags.writeable = False
    return r
