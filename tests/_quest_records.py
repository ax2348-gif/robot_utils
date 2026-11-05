"""Builders for teleop.quest_state.v1 records as the Quest app emits them."""
import math

EYE9 = [1, 0, 0, 0, 1, 0, 0, 0, 1]


def rot_z9(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c, -s, 0, s, c, 0, 0, 0, 1]


def hand(valid=True, engaged=False, needs_release=False, engagement_id=0,
         dp=(0.0, 0.0, 0.0), dr=EYE9, gripper=0.0):
    return {"valid": valid, "engaged": engaged, "needs_release": needs_release,
            "engagement_id": engagement_id, "delta_position": list(dp),
            "delta_rotation": list(dr), "gripper": gripper}


def record(seq, session="s1", focused=True, revision=0, left=None, right=None,
           base_velocity=(0.0, 0.0, 0.0), intent=True):
    r = {"schema": "teleop.quest_state.v1", "session_id": session, "frame_seq": seq,
         "sample_time_ns": 0, "input_focused": focused, "head": {}, "hands": {}}
    if intent:
        r["control_intent"] = {
            "schema": "teleop.operator_intent.v1", "session_id": session,
            "frame": "operator_base_flu", "scheme": "standard", "translation_limit_m": 0.45,
            "base_velocity": list(base_velocity), "operator_frame_revision": revision,
            "operator_from_quest": EYE9,
            "hands": {"left": left if left is not None else hand(),
                      "right": right if right is not None else hand()},
        }
    return r
