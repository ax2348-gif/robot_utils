"""The tiered recovery of the Quest client's armed latch (tools/quest_teleop/arming.py, README "Hold and re-grip"):
a transient dropout holds and a release + re-grip resumes with no Y (a new arm_seq above the WBC's floor, anchored to
the commanded pose: no jump); a cause longer than the grace window, a serious cause (estimator reset, the WBC's
non-transient disarm) or a deliberate one (X) disarms and needs Y; a grip held through the dropout never resumes;
a resume the WBC refuses returns to the hold. Pure (tests/quest_latch_fakes.py), plus the node's events and /state."""
from dataclasses import replace

import numpy as np
import pytest

from quest_latch_fakes import GRIP, IDLE, Rig, X, Y, with_left
from quest_teleop.arming import TRANSIENT_CLASSES, ResumeConfig, wbc_reason_transient

CMD = [1.5, 2.1, 0.8]                       # the fake WBC row's commanded EE (world)


def kinds(rig, since=0):
    return [e[0] for e in rig.client_events[since:]]


def make_rig(**kw):
    rig = Rig(**kw)
    rig.client_events = []
    rig.client.on_event = rig.client_events.append
    return rig


def engaged_rig(**kw):
    """Armed by Y, then the right grip engaged and the hand moved 10 cm (the target is off the command)."""
    rig = make_rig(**kw)
    rig.arm()
    rig.run(GRIP, 3)
    moved = replace(GRIP, right=replace(GRIP.right, pos=tuple(np.add(GRIP.right.pos, (0.1, 0.0, 0.0)))))
    rig.run(moved, 3)
    assert rig.core.eng is not None and rig.latch.armed
    return rig, moved


def gap(rig, seconds, s=IDLE, step=0.05):
    """No headset frame for ``seconds`` (one client step every ``step``)."""
    n = max(1, int(round(seconds / step)))
    for _ in range(n):
        rig.frame(s, dt=step, deliver=False)


def last_target(rig):
    return rig.last("target")


# -- defaults -------------------------------------------------------------------------------------------------

def test_default_config_holds_every_transient_class_for_two_seconds():
    r = ResumeConfig()
    assert r.enabled and r.grace_s == 2.0 and r.hold_max_s == 60.0
    assert all(r.classes[c] for c in TRANSIENT_CLASSES) and set(r.classes) == set(TRANSIENT_CLASSES)
    rig = make_rig()
    assert rig.latch.resume.enabled and rig.latch.tier == "disarmed"
    assert rig.latch.clutch_hands == ("right",)


@pytest.mark.parametrize("reason,transient", [
    ("control tick overrun: 120 ms", True), ("input stale (0.30 s without a datagram)", True),
    ("estimator: estimator pose is 0.31 s old", True), ("estimator: estimator stream stale (0.41 s)", True),
    ("estimator reset (world frame changed)", False), ("pose jumped 0.200 m / 3.0°", False),
    ("e-stop (button)", False), ("POST /disarm (page)", False), ("fault: solver", False),
    ("disabled: X", False), ("new session", False), ("client lease expired", False), (None, False), ("", False)])
def test_wbc_disarm_reasons_are_classified(reason, transient):
    assert wbc_reason_transient(reason) is transient


# -- a short input gap: hold, a re-grip resumes ---------------------------------------------------------------

def test_a_300_ms_gap_holds_then_a_regrip_resumes_without_y_and_without_a_jump():
    rig, moved = engaged_rig()
    seq0, n0 = rig.latch.arm_seq, len(rig.client_events)
    gap(rig, 0.3, moved)
    lt = rig.latch
    assert not lt.armed and lt.tier == "hold" and "re-grip" in lt.reason
    assert rig.last()["armed"] is False and rig.stream.row["armed"] == 0.0     # held on the wire, as a disarm
    assert kinds(rig, n0) == ["hold"]
    ev = rig.client_events[n0]
    assert ev[2]["class"] == "input_stale"
    rig.run(IDLE, 3)                                     # input back, the hand released
    assert lt.tier == "hold" and lt.hold["bad_since"] is None and not lt.armed
    n1 = len(rig.wbc.msgs)
    rig.frame(moved)                                     # re-grip (the hand is wherever the operator left it)
    assert lt.armed and lt.tier == "armed" and lt.arm_seq == seq0 + 1
    assert "Y" not in [c for c in rig.wbc.calls] and lt.presses == 2           # no Y, no enable
    t = [m for m in rig.since(n1) if m["type"] == "target"][0]
    assert t["armed"] is True and t["arm_seq"] == seq0 + 1
    np.testing.assert_allclose(t["ees"]["arm"]["pos"], CMD, atol=1e-9)        # anchored to the command: no jump
    assert rig.stream.row["armed"] == 1.0
    rig.run(moved, 2)
    assert lt.resumes == 1 and lt.hold is None and kinds(rig, n0)[-1] == "resumed"
    info = [e for e in rig.client_events if e[0] == "resumed"][0][2]
    assert info["hand"] == "right" and info["class"] in ("input_stale", "input_gap")


def test_a_grip_held_through_the_dropout_never_resumes_until_released_and_regripped():
    rig, moved = engaged_rig()
    gap(rig, 0.3, moved)
    rig.run(moved, 30)                                   # the grip never released
    assert not rig.latch.armed and rig.latch.tier == "hold"
    assert all(not m["armed"] for m in rig.wbc.msgs[-30:])
    rig.run(IDLE, 2)
    rig.frame(moved)
    assert rig.latch.armed


def test_the_left_grip_does_not_resume_a_one_hand_session():
    rig, _ = engaged_rig()
    gap(rig, 0.3)
    rig.run(IDLE, 2)
    rig.run(with_left(IDLE, grip=1.0), 3)
    assert not rig.latch.armed and rig.latch.tier == "hold"


def test_a_regrip_while_the_cause_is_still_present_is_refused_and_needs_a_new_one():
    rig, moved = engaged_rig()
    gap(rig, 0.3)
    rig.run(IDLE, 2)
    rig.client.gate.max_latency = 0.05
    rig.run(IDLE, 2, late=0.2)                           # late input: the cause is back
    rig.frame(moved, late=0.2)
    assert not rig.latch.armed and rig.latch.tier == "hold"
    rig.client.gate.max_latency = 10.0
    rig.run(moved, 3)                                    # fresh again but still gripped: that edge was used
    assert not rig.latch.armed
    rig.run(IDLE, 2)
    rig.frame(moved)
    assert rig.latch.armed


# -- escalation -------------------------------------------------------------------------------------------------

def test_a_gap_longer_than_the_window_escalates_to_a_disarm_that_needs_y():
    rig, moved = engaged_rig()
    n0 = len(rig.client_events)
    gap(rig, 2.5)
    lt = rig.latch
    assert lt.tier == "disarmed" and lt.hold is None and "press Y" in lt.reason
    assert kinds(rig, n0) == ["hold", "escalated"]
    rig.run(IDLE, 3)
    rig.run(moved, 3)
    assert not lt.armed                                  # a re-grip never resumes a disarm
    rig.run(IDLE, 2)
    rig.run(Y(), 3)
    assert lt.armed and lt.tier == "armed"


def test_a_window_is_configurable_and_off_restores_every_disarm_needs_y():
    rig, moved = engaged_rig(resume=ResumeConfig(grace_s=0.5))
    gap(rig, 0.8)
    assert rig.latch.tier == "disarmed"
    rig2, moved2 = engaged_rig(resume=ResumeConfig(enabled=False))
    n0 = len(rig2.client_events)
    gap(rig2, 0.3)
    assert rig2.latch.tier == "disarmed" and kinds(rig2, n0) == ["disarmed"]
    rig2.run(IDLE, 2)
    rig2.run(moved2, 2)
    assert not rig2.latch.armed


def test_a_class_turned_off_disarms_as_before():
    r = ResumeConfig()
    r.classes["input_stale"] = False
    rig, _ = engaged_rig(resume=r)
    gap(rig, 0.3)
    assert rig.latch.tier == "disarmed"


def test_a_hold_older_than_hold_max_escalates():
    rig, moved = engaged_rig(resume=ResumeConfig(hold_max_s=1.0))
    gap(rig, 0.3)
    rig.run(IDLE, 120)                                   # 1.2 s released, never re-gripped
    assert rig.latch.tier == "disarmed"


# -- deliberate and serious causes ------------------------------------------------------------------------------

def test_x_is_never_resumed_by_a_regrip():
    rig, moved = engaged_rig()
    rig.run(X(), 2)
    assert rig.latch.tier == "disarmed" and "disable" in rig.wbc.calls
    rig.run(IDLE, 3)
    rig.run(moved, 5)
    assert not rig.latch.armed


def test_x_during_a_hold_escalates_it():
    rig, moved = engaged_rig()
    gap(rig, 0.3)
    n0 = len(rig.client_events)
    rig.run(X(), 2)
    assert rig.latch.tier == "disarmed" and "escalated" in kinds(rig, n0)
    rig.run(IDLE, 3)
    rig.run(moved, 3)
    assert not rig.latch.armed


def test_an_estimator_reset_is_never_resumed():
    rig, moved = engaged_rig()
    n0 = len(rig.client_events)
    rig.stream.row["est_reset"] = 1.0
    rig.wbc.wbc_disarm("estimator reset (world frame changed)")
    rig.run(moved, 3)
    lt = rig.latch
    assert lt.tier == "disarmed" and kinds(rig, n0)[:2] == ["hold", "escalated"]
    assert "why" in rig.wbc.calls
    rig.run(IDLE, 3)
    rig.run(moved, 3)
    assert not lt.armed


def test_a_reconnect_of_the_same_headset_session_holds_and_a_regrip_resumes():
    """The reader reopens a connection silent for 1 s: a Wi-Fi blackout, the same headset session: a gap."""
    rig, moved = engaged_rig()
    rig.conn += 1                                        # the TCP stream reconnected
    rig.run(IDLE, 3)
    assert rig.latch.tier == "hold" and rig.latch.hold["class"] == "input_gap"
    rig.run(IDLE, 60)                                    # the gate's window and recovery after the reconnect
    rig.run(moved, 3)
    assert rig.latch.armed


def test_a_new_headset_session_is_serious():
    rig, moved = engaged_rig()
    rig.hs.session_base += 1                                  # the headset app restarted
    rig.run(IDLE, 3)
    assert rig.latch.tier == "disarmed"
    rig.run(IDLE, 60)
    rig.run(moved, 3)
    assert not rig.latch.armed


def test_a_refused_y_press_is_an_event_with_its_reason():
    rig, moved = engaged_rig()
    rig.hs.session_base += 1                                  # serious: Y needed
    rig.run(IDLE, 3)
    n0 = len(rig.client_events)
    rig.run(Y(GRIP), 4)                                  # Y with a grip pressed: refused
    ev = [e for e in rig.client_events[n0:] if e[0] == "y_refused"]
    assert len(ev) == 1 and "grip" in ev[0][1]
    assert not rig.latch.armed


def test_the_wbcs_own_short_dropout_holds_and_a_regrip_resumes():
    rig, moved = engaged_rig()
    rig.wbc.wbc_disarm("control tick overrun: 120 ms")
    rig.run(moved, 3)
    lt = rig.latch
    assert lt.tier == "hold" and not lt.hold["classifying"] and "tick overrun" in lt.hold["cause"]
    rig.run(IDLE, 2)
    rig.frame(moved)
    assert lt.armed and rig.stream.row["armed"] == 1.0


def test_the_wbc_disabled_during_a_hold_escalates():
    rig, moved = engaged_rig()
    gap(rig, 0.3)
    rig.stream.row["state"] = 0.0                        # disabled (the page, an e-stop)
    rig.run(IDLE, 2)
    assert rig.latch.tier == "disarmed"
    rig.stream.row["state"] = 2.0
    rig.run(moved, 3)
    assert not rig.latch.armed


def test_a_sim_reset_inhibit_escalates_a_hold():
    rig, moved = engaged_rig()
    gap(rig, 0.3)
    rig.latch.inhibit = "sim reset in progress"
    rig.run(IDLE, 2)
    assert rig.latch.tier == "disarmed"


# -- the WBC's arm_target check still applies -------------------------------------------------------------------

def test_a_resume_the_wbc_refuses_returns_to_the_hold_and_the_next_regrip_resumes():
    rig, moved = engaged_rig()
    gap(rig, 0.3)
    rig.run(IDLE, 2)
    rig.wbc.refuse_arm = "arm_far"                       # the WBC's arm_target_pos / rot check refuses it
    n0 = len(rig.client_events)
    rig.run(moved, 3)
    lt = rig.latch
    assert not lt.armed and lt.tier == "hold" and rig.stream.row["armed"] == 0.0
    assert kinds(rig, n0) == ["resumed", "resume_refused"]
    assert rig.last()["armed"] is False
    rig.wbc.refuse_arm = None
    rig.run(moved, 3)                                    # still held: no new edge
    assert not lt.armed
    rig.run(IDLE, 2)
    rig.frame(moved)
    assert lt.armed and lt.arm_seq == 3 and rig.stream.row["armed"] == 1.0   # a new arm_seq above the floor


def test_a_refused_y_arm_is_a_disarm_not_a_hold():
    rig = make_rig()
    rig.wbc.refuse_arm = "arm_far"
    rig.run(IDLE, 2)
    rig.run(Y(), 3)
    rig.run(IDLE, 3)
    assert rig.latch.tier == "disarmed" and "refused" in rig.latch.reason


# -- state ----------------------------------------------------------------------------------------------------

def test_tier_info_reports_the_tier_the_cause_and_resumability():
    rig, moved = engaged_rig()
    assert rig.latch.tier_info(rig.t)["tier"] == "armed"
    gap(rig, 0.3)
    i = rig.latch.tier_info(rig.t)
    assert i["tier"] == "hold" and i["hold"]["class"] == "input_stale" and not i["hold"]["resumable"]
    rig.run(IDLE, 2)
    i = rig.latch.tier_info(rig.t)
    assert i["hold"]["resumable"] and i["hold"]["released"] == ["right"] and i["config"]["grace_s"] == 2.0


# -- the node: events, /state, config -----------------------------------------------------------------------------

def test_node_events_state_and_config():
    from test_teleop_episodes import Env
    env = Env()
    rig = env.rig
    rig.latch.resume = env.node.resume      # start_teleop shares the node's config with its client (the rig's is its own)
    rig.arm()
    rig.run(GRIP, 3)
    gap(rig, 0.3, GRIP)
    held = env.events("held")
    assert len(held) == 1 and held[0]["detail"]["class"] == "input_stale"
    assert held[0]["detail"]["cue"] == "hold: re-grip to resume"
    T = env.node.state()["teleop"]
    assert T["tier"]["tier"] == "hold" and T["tier"]["hold"]["cause"]
    rig.run(IDLE, 3)
    rig.run(GRIP, 3)
    r = env.events("resumed")
    assert len(r) == 1 and r[0]["detail"]["arm_seq"] == 2 and r[0]["detail"]["hand"] == "right"
    assert env.node.state()["teleop"]["tier"]["tier"] == "armed"
    env.node._publish(rig.t, rig.client)
    assert env.node.latest["tier"] == 2.0
    # config: the window and the classes, live
    out = env.node.set_resume(grace_s=0.5, classes={"input_late": False})
    assert out["grace_s"] == 0.5 and out["classes"]["input_late"] is False
    assert env.node.state()["config"]["resume"]["grace_s"] == 0.5 and rig.latch.resume.grace_s == 0.5
    with pytest.raises(ValueError):
        env.node.set_resume(classes={"bogus": True})
    rig.run(IDLE, 3)
    gap(rig, 1.0)
    e = env.events("escalated")
    assert len(e) == 1 and e[0]["detail"]["cue"] == "disarmed: press Y"
    assert env.node.state()["teleop"]["tier"]["tier"] == "disarmed"


def test_with_the_real_latency_gate_a_regrip_after_recovery_resumes():
    """The real gate (0.5 s of good frames after late ones): a re-grip before the input is fresh again is refused
    (that edge is used); one after it resumes."""
    rig = make_rig(latency=True)
    rig.run(IDLE, 80)                                    # 0.8 s: the rate window judged, fresh
    rig.arm()
    rig.run(GRIP, 5)
    assert rig.core.eng is not None
    rig.run(GRIP, 3, late=0.3)                           # 300 ms late: input_late
    assert rig.latch.tier == "hold" and rig.latch.hold["class"] == "input_late"
    rig.run(IDLE, 5)                                     # 50 ms of good frames: still recovering
    rig.frame(GRIP)
    assert not rig.latch.armed and rig.latch.hold_info(rig.t)["last_refused"] == "the cause has not cleared yet"
    rig.run(IDLE, 80)
    rig.frame(GRIP)
    assert rig.latch.armed and rig.stream.row["armed"] == 1.0
    np.testing.assert_allclose(rig.last("target")["ees"]["arm"]["pos"], CMD, atol=1e-9)


# -- causes the tick cannot see between frames, serious causes during a hold ----------------------------------------

UNFOCUSED = replace(IDLE, focused=False)


def test_a_brief_unfocused_spell_holds_and_a_regrip_resumes():
    rig, moved = engaged_rig()
    rig.run(UNFOCUSED, 10)                               # 0.1 s of unfocused frames
    lt = rig.latch
    assert lt.tier == "hold" and lt.hold["class"] == "input_invalid"
    rig.run(IDLE, 3)
    rig.frame(moved)
    assert lt.armed and rig.stream.row["armed"] == 1.0


def test_unfocused_longer_than_the_window_escalates_even_though_frames_keep_arriving():
    rig, moved = engaged_rig()
    rig.run(UNFOCUSED, 250)                              # 2.5 s of unfocused frames (the headset taken off)
    lt = rig.latch
    assert lt.tier == "disarmed" and "escalated" in kinds(rig)
    rig.run(IDLE, 3)
    rig.run(moved, 3)
    assert not lt.armed


def test_an_estimator_reset_during_a_hold_escalates():
    rig, moved = engaged_rig()
    gap(rig, 0.3)                                        # held (the client disarmed first: the WBC's reason is ours)
    assert rig.latch.tier == "hold"
    rig.stream.row["est_reset"] = 7.0                    # the world frame changed meanwhile
    rig.run(IDLE, 3)
    assert rig.latch.tier == "disarmed" and "estimator reset" in rig.latch.reason
    rig.run(moved, 3)
    assert not rig.latch.armed


def test_repeated_wbc_refusals_escalate():
    rig, moved = engaged_rig()
    gap(rig, 0.3)
    rig.run(IDLE, 2)
    rig.wbc.refuse_arm = "arm_far"
    for _ in range(3):
        rig.run(moved, 3)
        rig.run(IDLE, 2)
    lt = rig.latch
    assert lt.tier == "disarmed" and "refused 3 resumes" in lt.reason
    assert kinds(rig).count("resume_refused") == 3 and kinds(rig)[-1] == "escalated"


def test_a_serious_cause_before_the_wbc_confirms_a_resume_escalates():
    rig, moved = engaged_rig()
    gap(rig, 0.3)
    rig.run(IDLE, 2)
    rig.wbc.refuse_arm = "arm_far"                       # the resume is sent but not taken yet
    rig.frame(moved)
    assert rig.latch.armed and rig.latch.resuming
    rig.hs.session_base += 1                                  # a new headset session before the WBC row answers
    rig.frame(moved)
    assert rig.latch.tier == "disarmed" and kinds(rig)[-1] == "escalated"


def test_a_wbc_disarm_with_no_reported_reason_holds_and_a_named_serious_one_escalates():
    """The WBC's disarm_reason is None when it already moved on (or its /state didn't answer): a hold, not a Y."""
    lt = make_rig().latch
    lt._enter_hold("the WBC disarmed (its /state disarm_reason)", "wbc_disarm", 0.0)
    lt.hold["classifying"] = True
    lt.wbc_disarm_reason(None, 0.1)
    assert lt.tier == "hold" and "no reason reported" in lt.hold["cause"]
    lt.hold["classifying"] = True
    lt.wbc_disarm_reason("estimator reset (world frame changed)", 0.2)
    assert lt.tier == "disarmed"
