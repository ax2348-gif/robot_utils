"""Client for the Quest teleop app's telemetry stream (``teleop.quest_state.v1`` JSON lines).

    from robot_utils.quest import QuestClient

    with QuestClient("100.72.77.83", 9392) as quest:
        frame = quest.latest()            # newest parsed QuestFrame, or None
        if frame is not None and quest.age(frame) < 0.2 and frame.intent is not None:
            hand = frame.intent.hands["right"]

The app on the headset serves the stream on 127.0.0.1:9391; reach it over the network with a
relay on the headset, e.g. ``nc -s <quest-ip> -L -p 9392 nc 127.0.0.1 9391``.

For driving a robot, use ``robot_utils.teleop``, which applies the consumer contract
(freshness, focus, validity, session and operator-frame changes, release-before-engage).
"""
from .client import ClientStats, QuestClient
from .messages import (INTENT_SCHEMA, SIDES, STATE_SCHEMA, HandIntent, OperatorIntent,
                       ProtocolError, QuestFrame, parse_frame)

__all__ = ["QuestClient", "ClientStats", "QuestFrame", "OperatorIntent", "HandIntent",
           "ProtocolError", "parse_frame", "STATE_SCHEMA", "INTENT_SCHEMA", "SIDES"]
