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


def slow_enable(rig, frames_enabling=10):
    """The real WBC: POST /enable answers, but the rows show 'enabling' (1) for a while before 'idle' (2)."""
    state = {"left": None}

    def enable():
        rig.wbc.calls.append("enable")
        rig.stream.row["state"] = 1.0
        state["left"] = frames_enabling
        return {"enabled": True}
    rig.wbc.enable = enable
    orig = rig.frame

    def frame(s, **kw):
        if state["left"] is not None:
            state["left"] -= 1
            if state["left"] <= 0:
                rig.stream.row["state"], state["left"] = 2.0, None
        return orig(s, **kw)
    rig.frame = frame


def x_then_y(rig, y_frames):
    """Armed, then X (disabled), then one deliberate Y press of ``y_frames`` frames, then Y released."""
    rig.arm()
    rig.frame(X())
    assert rig.stream.row["state"] == 0.0 and not rig.latch.armed
    rig.run(IDLE, 2)
    rig.run(Y(), y_frames)


@pytest.mark.parametrize("held", [True, False])
def test_x_then_one_y_press_waits_through_enabling_and_arms(held):
    """The real-robot bug: after X, Y left the client 'stuck on the WBC is enabling'. One press now waits through
    the enabling rows and arms on the first enabled row after our enable, whether Y is still held or released."""
    rig = Rig()
    slow_enable(rig, 10)
    x_then_y(rig, 3)
    assert rig.wbc.calls == ["disable", "enable"] and rig.latch.pending_arm
    assert rig.latch.reason == "enabling the WBC…"
    for _ in range(5):                                          # rows: enabling
        rig.frame(Y() if held else IDLE)
        assert not rig.latch.armed and rig.latch.pending_arm and rig.stream.row["state"] == 1.0
    rig.run(Y() if held else IDLE, 8)                           # the row turns enabled: armed (one press)
    assert rig.latch.armed and rig.latch.reason == "armed" and rig.latch.presses == 2
    assert rig.stream.row["armed"] == 1.0 and rig.wbc.calls == ["disable", "enable"]


def test_a_pending_arm_times_out():
    rig = Rig()
    slow_enable(rig, 10_000)                                    # never enabled
    x_then_y(rig, 3)
    rig.run(IDLE, 150)                                          # 1.5 s: still waiting
    assert rig.latch.pending_arm and not rig.latch.armed
    rig.run(IDLE, 60)                                           # past 2 s
    assert not rig.latch.pending_arm and not rig.latch.armed
    assert rig.latch.reason.startswith("arm cancelled: the WBC did not enable within 2")


def test_x_during_a_pending_enable_cancels_and_disables():
    rig = Rig()
    slow_enable(rig, 30)
    x_then_y(rig, 3)
    rig.run(IDLE, 5)
    rig.frame(X())
    assert not rig.latch.pending_arm and rig.latch.reason == "arm cancelled: X: disabled"
    assert rig.wbc.calls[-1] == "disable"
    rig.run(IDLE, 40)
    assert not rig.latch.armed


def test_a_fault_row_cancels_a_pending_arm():
    rig = Rig()
    slow_enable(rig, 30)
    x_then_y(rig, 3)
    rig.run(IDLE, 3)
    rig.stream.row["state"] = 5.0
    rig.frame(IDLE)
    assert not rig.latch.pending_arm and "fault" in rig.latch.reason and not rig.latch.armed


@pytest.mark.parametrize("spoiler,why", [(GRIP, "grip"), (stick(IDLE, (0.0, 0.6)), "stick")])
def test_preconditions_failing_at_completion_cancel(spoiler, why):
    rig = Rig()
    slow_enable(rig, 10)
    x_then_y(rig, 3)
    rig.run(spoiler, 15)                                        # enabled while a grip / stick is used
    assert not rig.latch.armed and not rig.latch.pending_arm
    assert rig.latch.reason.startswith("arm cancelled: ") and why in rig.latch.reason


def test_a_disarm_condition_cancels_a_pending_arm():
    rig = Rig()
    slow_enable(rig, 30)
    x_then_y(rig, 3)
    rig.frame(replace(IDLE, session=2))                         # a discontinuity
    assert not rig.latch.pending_arm and rig.latch.reason.startswith("arm cancelled: headset discontinuity")
    rig.run(IDLE, 40)
    assert not rig.latch.armed


def test_a_refused_enable_cancels_with_the_reason():
    rig = Rig()
    rig.stream.row["state"] = 0.0
    rig.wbc.refuse_enable = "e-stop is latched"
    rig.run(IDLE, 2)
    rig.run(Y(), 6)
    assert not rig.latch.armed and rig.latch.reason == "arm cancelled: enable refused: e-stop is latched"


def test_the_reason_reflects_the_state():
    rig = Rig()
    assert rig.latch.reason == "not armed: press Y"
    rig.arm()
    assert rig.latch.reason == "armed"
    rig.frame(X())
    assert rig.latch.reason == "X: disabled (Y re-enables and arms)"
    rig.run(IDLE, 2)
    rig.run(Y(), 3)
    assert rig.latch.reason == "enabling the WBC…"


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


def test_a_stale_wbc_row_after_arming_disarms():
    """Armed, the WBC rows stop arriving (the stream died): disarm once the newest row is older than 0.3 s; the
    rows coming back never re-arm."""
    rig = Rig()
    rig.arm()
    rig.stream.frozen = True
    rig.run(IDLE, 25)                                           # 0.25 s: still within
    assert rig.latch.armed
    n = len(rig.wbc.msgs)
    rig.run(IDLE, 10)                                           # 0.35 s
    assert not rig.latch.armed and "WBC state stale" in rig.latch.reason
    after = rig.since(n)
    k = next(i for i, m in enumerate(after) if m["armed"] is False)      # sent at the disarm ...
    assert all(m["armed"] is False for m in after[k:])                    # ... and from then on
    rig.stream.frozen = False
    rig.run(IDLE, 20)
    assert not rig.latch.armed
    from quest_teleop.arming import LatchConfig                 # configurable
    rig2 = Rig(latch=LatchConfig(wbc_stale_s=0.1))
    rig2.arm()
    rig2.stream.frozen = True
    rig2.run(IDLE, 15)
    assert not rig2.latch.armed


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


def test_gripper_follows_the_trigger_from_the_arming_frame_and_never_while_disarmed():
    """The owner's rule (rebot_ctl): no gripper command before arming (Start, after any disarm / disable: the gripper
    stays where it is); from the arming frame on, 1 - trigger in every target (a released trigger opens)."""
    rig = Rig()
    rig.run(trigger(IDLE, 0.8), 10)                             # after Start, the trigger squeezed: nothing
    assert all("ees" not in m for m in rig.wbc.msgs)
    rig.run(IDLE, 2)
    n = len(rig.wbc.msgs)
    rig.run(Y(), 3)                                             # armed with the trigger released
    assert rig.latch.armed
    first = targets(rig.since(n))[0]
    assert first["armed"] and first["ees"]["arm"]["gripper"] == 1.0          # open at once
    c = rig.frame(trigger(IDLE, 0.6))
    assert abs(c["ees"]["arm"]["gripper"] - 0.4) < 1e-9
    c = rig.frame(trigger(IDLE, 0.3))
    assert abs(c["ees"]["arm"]["gripper"] - 0.7) < 1e-9
    rig.frame(X())                                              # disarmed and disabled: no gripper command
    n = len(rig.wbc.msgs)
    rig.run(trigger(IDLE, 0.0), 10)
    assert all("ees" not in m for m in rig.since(n))
    s = trigger(IDLE, 0.9)                                      # re-armed with the trigger squeezed: closes at once
    rig.run(s, 2)
    n = len(rig.wbc.msgs)
    rig.run(Y(s), 4)
    assert rig.latch.armed
    assert abs(targets(rig.since(n))[0]["ees"]["arm"]["gripper"] - 0.1) < 1e-9


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


def test_an_inhibit_cancels_a_pending_arm():
    """The teleop node's sim reset sets the latch's inhibit and re-enables the WBC itself: a Y press made just
    before must never complete during the reset (the merge of the pending-arm fix with the episodes' inhibit)."""
    rig = Rig()
    slow_enable(rig, 10)
    x_then_y(rig, 3)
    assert rig.latch.pending_arm
    rig.latch.inhibit = "sim reset in progress"
    rig.run(IDLE, 20)                                           # the row turns enabled meanwhile
    assert rig.stream.row["state"] == 2.0
    assert not rig.latch.armed and not rig.latch.pending_arm
    assert rig.latch.reason == "arm cancelled: sim reset in progress"
