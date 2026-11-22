"""The Quest client's armed latch (tools/quest_teleop/arming.py, README "Arming: Y and X"): Start is not live; Y arms
only on a deliberate press (released first, 3 fresh valid frames, X up, grips released, sticks neutral, a fresh
enabled WBC without a fault; one arm per press; enables first when disabled); X is level-triggered DISABLE from any
frame; every disarm trigger sends armed:false at once; nothing recovers into armed; after arming the sticks need
neutral and the gripper waits for the trigger; the riding pose is captured afresh. Pure (tests/quest_latch_fakes.py)."""
from dataclasses import replace

import numpy as np
import pytest

from quest_latch_fakes import GRIP, IDLE, Rig, X, Y, stick, trigger, with_left


def targets(msgs):
    return [m for m in msgs if m["type"] == "target"]


# -- Start is not live ---------------------------------------------------------------------------------------

def test_start_is_not_live_nothing_but_disarmed_keepalives():
    rig = Rig()
    for s in (IDLE, GRIP, stick(IDLE, (0.0, 1.0), 1.0), trigger(IDLE, 1.0), replace(GRIP, right=replace(
            GRIP.right, pos=(0.6, -0.2, -0.2)))):
        rig.run(s, 10)
    assert not rig.latch.armed and rig.wbc.msgs
    assert all(m["type"] == "alive" and m["armed"] is False for m in rig.wbc.msgs)
    assert rig.wbc.calls == []                                  # no enable / disable on its own


# -- Y: a deliberate press ----------------------------------------------------------------------------------

def test_y_arms_after_release_and_three_fresh_frames_one_arm_per_press():
    rig = Rig()
    rig.run(Y(), 10)                                            # held since start: never seen released
    assert not rig.latch.armed
    rig.run(IDLE, 1)
    rig.run(Y(), 2)
    assert not rig.latch.armed                                  # 2 frames: not yet
    rig.frame(Y())
    assert rig.latch.armed and rig.latch.arm_seq == 1
    m = rig.last()
    assert m["type"] == "target" and m["armed"] is True and m["arm_seq"] == 1   # the arming datagram is a target
    assert rig.stream.row["armed"] == 1.0
    rig.run(Y(), 20)                                            # still held: no second arm
    assert rig.latch.arm_seq == 1 and rig.latch.presses == 1
    assert all(t["armed"] and t["arm_seq"] == 1 for t in targets(rig.since(0))[1:])


def test_the_arm_seq_starts_above_the_wbc_floor():
    rig = Rig()
    rig.client.session_opened({"session": 1, "arm_seq": 41})
    rig.wbc.arm_seq_floor = 41
    rig.arm()
    assert rig.latch.arm_seq == 42 and rig.stream.row["armed"] == 1.0


@pytest.mark.parametrize("spoiler,why", [
    (GRIP, "grip"), (stick(IDLE, (0.0, 0.5)), "stick"), (stick(IDLE, (0.0, 0.0), 0.3), "stick"),
    (with_left(IDLE, grip=0.5), "grip")])
def test_y_refused_while_a_grip_or_a_stick_is_used_and_the_press_is_spoiled(spoiler, why):
    rig = Rig()
    rig.run(IDLE, 2)
    rig.run(Y(spoiler), 5)
    assert not rig.latch.armed and why in rig.latch.reason
    rig.run(Y(), 5)                                             # the grip / stick back to rest, Y still held
    assert not rig.latch.armed                                  # spoiled: release Y first
    rig.arm()


def test_y_refused_on_a_stale_or_faulted_wbc_row():
    rig = Rig()
    rig.stream.row["state"] = 5.0                               # a WBC fault
    rig.run(IDLE, 2)
    rig.run(Y(), 5)
    assert not rig.latch.armed and "fault" in rig.latch.reason
    rig.stream.row["state"] = 2.0
    rig.stream.frozen = True                                    # the WBC rows stop arriving
    rig.run(IDLE, 40)
    rig.run(Y(), 10)
    assert not rig.latch.armed and "stale" in rig.latch.reason
    rig.stream.frozen = False
    rig.arm()


def test_focus_loss_mid_press_needs_a_new_press():
    rig = Rig()
    rig.run(IDLE, 2)
    rig.run(Y(), 2)
    rig.frame(replace(Y(), focused=False))                      # unfocused mid-press
    rig.run(Y(), 10)
    assert not rig.latch.armed
    rig.arm()


def test_y_on_a_disabled_wbc_enables_then_arms_if_still_held():
    rig = Rig()
    rig.stream.row["state"] = 0.0                               # disabled (by X earlier, say)
    rig.run(IDLE, 2)
    rig.run(Y(), 3)
    assert rig.wbc.calls == ["enable"] and not rig.latch.armed  # the press asked for the enable
    rig.frame(Y())                                              # enabled; Y still held: armed
    assert rig.latch.armed and rig.stream.row["armed"] == 1.0
    # enabled and released before the arm: no arm
    rig2 = Rig()
    rig2.stream.row["state"] = 0.0
    rig2.client.actions.sync = True
    rig2.run(IDLE, 2)
    rig2.run(Y(), 3)
    rig2.run(IDLE, 3)
    assert rig2.wbc.calls == ["enable"] and not rig2.latch.armed
    # a refused enable: no arm, the reason shown
    rig3 = Rig()
    rig3.stream.row["state"] = 0.0
    rig3.wbc.refuse_enable = "e-stop is latched"
    rig3.run(IDLE, 2)
    rig3.run(Y(), 6)
    assert not rig3.latch.armed and "e-stop is latched" in rig3.latch.reason


# -- X: level-triggered disable ------------------------------------------------------------------------------

def test_x_disarms_at_once_and_disables_from_any_frame():
    rig = Rig()
    rig.arm()
    n = len(rig.wbc.msgs)
    rig.frame(X())
    assert not rig.latch.armed and rig.wbc.calls == ["disable"]
    after = rig.since(n)
    assert after and after[0]["armed"] is False                 # armed:false went out in the same step
    rig.run(X(), 50)                                            # held: no request storm (the WBC shows disabled)
    assert rig.wbc.calls == ["disable"]
    assert rig.latch.disabled_by_client
    rig.run(IDLE, 2)
    rig.run(Y(), 4)                                             # Y re-enables and arms
    assert rig.wbc.calls == ["disable", "enable"] and rig.latch.armed
    rig.frame(replace(X(), focused=False))                      # X in an unfocused (invalid) frame still disables
    assert not rig.latch.armed and rig.wbc.calls[-1] == "disable"


def test_x_with_y_never_arms():
    rig = Rig()
    rig.run(IDLE, 2)
    rig.run(with_left(IDLE, primary=True, secondary=True), 10)
    assert not rig.latch.armed


def test_x_held_retries_the_disable_while_the_wbc_still_shows_enabled():
    rig = Rig()
    rig.arm()
    rig.wbc.disable = lambda: rig.wbc.calls.append("disable")    # the WBC never actually disables
    rig.run(X(), 250)                                           # 2.5 s
    assert 2 <= rig.wbc.calls.count("disable") <= 4


# -- every client disarm trigger: armed:false at once ---------------------------------------------------------

@pytest.mark.parametrize("trigger_", ["session", "unfocused", "left_inactive", "stale", "reconnect", "wbc_disarmed",
                                      "wbc_disabled", "hand_invalid"])
def test_client_disarm_triggers_send_armed_false(trigger_):
    rig = Rig()
    rig.arm()
    rig.run(IDLE, 5)
    assert rig.latch.armed
    n = len(rig.wbc.msgs)
    if trigger_ == "session":
        rig.frame(replace(IDLE, session=2))
    elif trigger_ == "unfocused":
        rig.frame(replace(IDLE, focused=False))
    elif trigger_ == "left_inactive":
        rig.frame(with_left(IDLE, tracked=False))
    elif trigger_ == "stale":
        rig.t += 0.25
        rig.client.step(wait=0)                                  # no frame for 250 ms
    elif trigger_ == "reconnect":
        rig.conn = 2
        rig.frame(IDLE)
    elif trigger_ == "wbc_disarmed":
        rig.wbc.wbc_disarm()
        rig.frame(IDLE)
    elif trigger_ == "wbc_disabled":
        rig.stream.row["state"] = 0.0
        rig.frame(IDLE)
    elif trigger_ == "hand_invalid":
        rig.frame(replace(IDLE, right=replace(IDLE.right, tracked=False)))
    assert not rig.latch.armed, trigger_
    assert any(m["armed"] is False for m in rig.since(n)), rig.since(n)
    assert all(m["armed"] is False for m in rig.since(n))
    assert rig.latch.reason


def test_no_recovery_rearms_with_y_held_through_it():
    rig = Rig()
    rig.run(IDLE, 2)
    rig.run(Y(), 3)
    assert rig.latch.armed
    for disrupt in (lambda: rig.frame(replace(Y(), session=2)), lambda: setattr(rig, "conn", rig.conn + 1),
                    lambda: rig.frame(replace(Y(), focused=False))):
        disrupt()
        rig.run(Y(), 60)                                        # Y held all along, the input fine again
        assert not rig.latch.armed
    rig.t += 0.5                                                # stale, then fresh with Y held
    rig.run(Y(), 60)
    assert not rig.latch.armed
    assert all(not m["armed"] for m in rig.wbc.msgs[-100:])


def test_a_wbc_row_older_than_the_arming_datagram_never_disarms():
    """Enable-then-arm: the newest WBC row may predate the enable (disabled, not armed); only rows that processed
    our arming datagram (seq) count."""
    rig = Rig()
    rig.stream.row["state"] = 0.0
    rig.run(IDLE, 2)
    rig.run(Y(), 4)
    assert rig.latch.armed
    old = dict(rig.stream.row, state=0.0, armed=0.0, seq=rig.stream.row["seq"] - 1)
    rig.stream.latest = lambda: (dict(old), rig.t)             # the stream lags: an old row, fresh in age
    rig.run(IDLE, 5)
    assert rig.latch.armed
    old["seq"] = float(rig.wbc.seq)                             # a row after our datagrams, disabled: disarm
    rig.frame(IDLE)
    assert not rig.latch.armed and "disabled" in rig.latch.reason


def test_alive_and_the_old_press_never_rearm_after_a_wbc_side_disarm():
    rig = Rig()
    rig.arm()
    rig.wbc.wbc_disarm()                                        # e.g. a control-tick overrun on the WBC
    rig.run(IDLE, 10)
    assert not rig.latch.armed and rig.stream.row["armed"] == 0.0
    rig.arm()                                                   # a new press: arm_seq 2 above the floor
    assert rig.latch.arm_seq == 2 and rig.stream.row["armed"] == 1.0


# -- after arming: sticks need neutral, the gripper waits for the trigger, the riding pose is fresh -------------

def test_sticks_need_neutral_after_arming():
    rig = Rig()
    rig.arm()                                                   # (ends with a neutral frame: the sticks are live)
    c = rig.frame(stick(IDLE, (0.0, 1.0)))
    assert c["base"]["mode"] == "commanded"
    # disarm, re-arm, and push the stick in the very next frame: no velocity until seen neutral
    rig.frame(X())
    rig.run(IDLE, 2)
    rig.run(Y(), 4)
    assert rig.latch.armed
    for _ in range(5):
        c = rig.frame(stick(IDLE, (0.0, 1.0), 1.0))
        assert c["base"] == {"mode": "locked"}, c["base"]
    rig.frame(IDLE)                                             # seen neutral
    c = rig.frame(stick(IDLE, (0.0, 1.0)))
    assert c["base"]["mode"] == "commanded" and c["base"]["velocity"]["vx"] > 0.3


def test_gripper_untouched_until_the_trigger_moves_after_arming():
    rig = Rig()
    s = trigger(IDLE, 0.3)
    rig.run(s, 2)
    rig.run(Y(s), 3)
    assert rig.latch.armed
    for _ in range(10):
        c = rig.frame(trigger(IDLE, 0.32))                      # within 0.05 of where it was at arming
        assert "gripper" not in c["ees"]["arm"]
    c = rig.frame(trigger(IDLE, 0.6))
    assert abs(c["ees"]["arm"]["gripper"] - 0.4) < 1e-9          # moved: commanded (1 - trigger)
    c = rig.frame(trigger(IDLE, 0.3))
    assert abs(c["ees"]["arm"]["gripper"] - 0.7) < 1e-9          # and from then on follows it


def test_a_disarm_ends_the_engagement_and_the_next_needs_a_fresh_squeeze():
    rig = Rig()
    rig.arm()
    c = rig.run(GRIP, 5)                                        # a squeeze after arming engages (at the command)
    assert rig.core.status.state == "engaged" and c["base"]["mode"] == "hint"
    np.testing.assert_allclose(targets(rig.wbc.msgs)[-5]["ees"]["arm"]["pos"], [1.5, 2.1, 0.8], atol=1e-9)
    rig.frame(GRIP)
    rig.wbc.wbc_disarm()                                        # disarmed while engaged (the WBC's side)
    rig.run(GRIP, 3)
    assert not rig.latch.armed and rig.core.eng is None and rig.core.needs_release


def test_the_riding_pose_is_captured_afresh_after_a_disarm():
    rig = Rig()
    c = rig.arm()
    np.testing.assert_allclose(c["ees"]["arm"]["pos"], [1.5, 2.1, 0.8], atol=1e-9)
    rig.frame(X())                                              # disarmed; the arm is then moved by something else
    rig.stream.row.update(arm_cmd_x=1.2, arm_cmd_y=2.4, arm_cmd_z=0.5)
    rig.run(IDLE, 2)
    rig.run(Y(), 4)                                             # (disabled by X: the press enables, then arms)
    assert rig.latch.armed and rig.latch.arm_seq == 2
    c = rig.last("target")
    np.testing.assert_allclose(c["ees"]["arm"]["pos"], [1.2, 2.4, 0.5], atol=1e-9)   # not the stale riding pose
