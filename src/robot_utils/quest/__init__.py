"""Client for the Quest teleop app's telemetry stream (``teleop.quest_state.v1`` JSON lines).

    from robot_utils.quest import QuestClient

    with QuestClient("100.72.77.83")      # port defaults to 9391 as quest:
        frame = quest.latest()            # newest parsed QuestFrame, or None
        if frame is not None and quest.age(frame) < 0.2 and frame.intent is not None:
            hand = frame.intent.hands["right"]

The app on the headset serves the stream on every interface, port 9391 (quest_env 447e80f
and later), so connect to the headset's LAN or tailnet address directly.

For driving a robot, use ``robot_utils.teleop``, which applies the consumer contract
(freshness, focus, validity, session and operator-frame changes, release-before-engage).
"""
from .client import DEFAULT_PORT, ClientStats, QuestClient
from .messages import (INTENT_SCHEMA, SIDES, STATE_SCHEMA, HandIntent, OperatorIntent,
                       ProtocolError, QuestFrame, parse_frame)

__all__ = ["QuestClient", "DEFAULT_PORT", "ClientStats", "QuestFrame", "OperatorIntent", "HandIntent",
           "ProtocolError", "parse_frame", "STATE_SCHEMA", "INTENT_SCHEMA", "SIDES"]
