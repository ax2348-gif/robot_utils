"""Headset latency detection (tools/quest_teleop/quest_stream.QuestGate): latency = arrival (our clock) - the frame's
sample_time_ns (the headset's), over the session's running minimum (the clock offset cancels); late (> 150 ms over
it) or slow (< 30 Hz over 0.5 s) input is stale, and disarms; recovery needs 0.5 s of good frames and never re-arms.
Pure (tests/quest_latch_fakes.py)."""
from quest_latch_fakes import IDLE, Rig, Y   # (puts tools/ on sys.path)

from quest_teleop.fake_headset import Headset  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402
from quest_teleop.teleop import status_line  # noqa: E402


def feed(gate, hs, t, n, dt=1 / 72, late=0.0, offset=0.0):
    for _ in range(n):
        t += dt
        gate.check(Frame(hs.frame(IDLE, t - late + offset), t, 1))
    return t


def test_steady_frames_are_fresh_whatever_the_clock_offset():
    for offset in (0.0, -12345.678, 3.5e6):                   # the headset's clock is not ours
        gate, hs = QuestGate(), Headset()
        t = feed(gate, hs, 100.0, 80, late=0.03, offset=offset)
        assert gate.fresh(t) and abs(gate.latency) < 1e-6


def test_steady_but_late_frames_are_stale_and_recover_only_after_half_a_second():
    gate, hs = QuestGate(), Headset()
    t = feed(gate, hs, 100.0, 72, late=0.02)
    assert gate.fresh(t)
    t = feed(gate, hs, t, 20, late=0.2)                       # a steady stream, 180 ms later than its best
    assert not gate.fresh(t) and "latency" in gate.why_stale(t) and abs(gate.latency - 0.18) < 1e-6
    assert gate.counts["late"] == 20
    t = feed(gate, hs, t, 30, late=0.02)                      # good again for 0.42 s: still recovering
    assert not gate.fresh(t)
    t = feed(gate, hs, t, 8, late=0.02)                       # 0.53 s of good frames
    assert gate.fresh(t)


def test_a_threshold_is_configurable():
    gate, hs = QuestGate(max_latency=0.3), Headset()
    t = feed(gate, hs, 100.0, 72)
    t = feed(gate, hs, t, 20, late=0.2)
    assert gate.fresh(t)


def test_a_low_frame_rate_is_stale():
    gate, hs = QuestGate(), Headset()
    t = feed(gate, hs, 100.0, 72)
    assert gate.fresh(t)
    t = feed(gate, hs, t, 20, dt=1 / 20)                      # 20 Hz: each frame inside the 200 ms freshness
    assert not gate.fresh(t) and "rate" in gate.why_stale(t) and gate.counts["slow"] > 0
    assert 15 <= gate.rate(t) <= 25
    gate2 = QuestGate(min_rate=15.0)
    t = feed(gate2, hs, 200.0, 72)
    t = feed(gate2, hs, t, 20, dt=1 / 20)
    assert gate2.fresh(t)


def test_frames_without_a_sample_time_are_stale():
    gate, hs = QuestGate(), Headset()
    t = 100.0
    for _ in range(40):
        t += 0.01
        d = hs.frame(IDLE, t)
        del d["sample_time_ns"]
        gate.check(Frame(d, t, 1))
    assert not gate.fresh(t) and "sample_time_ns" in gate.why_stale(t)


def test_late_input_disarms_the_client_and_recovery_never_rearms():
    rig = Rig()
    rig.arm()
    rig.run(IDLE, 20, late=0.01)
    assert rig.latch.armed
    n = len(rig.wbc.msgs)
    rig.run(Y(), 3, late=0.3)                                 # late frames (Y held all along from here)
    assert not rig.latch.armed and "latency" in rig.latch.reason
    assert rig.since(n)[0]["armed"] is False
    rig.run(Y(), 100, late=0.01)                              # good again for 1 s, Y still held: not armed
    assert not rig.latch.armed and rig.client.gate.fresh(rig.t)
    rig.arm()                                                 # a new press arms
    assert rig.latch.arm_seq == 2


def test_slow_input_disarms_the_client():
    rig = Rig()
    rig.arm()
    rig.run(IDLE, 60)
    rig.run(IDLE, 15, dt=0.06)                                # 16 Hz
    assert not rig.latch.armed and "rate" in rig.latch.reason


def test_latency_and_rate_are_in_the_status_line():
    rig = Rig()
    rig.run(IDLE, 60, late=0.0)
    line = status_line(rig.client)
    assert "lat=" in line and "rate=" in line and "disarmed" in line
    rig.arm()
    assert "ARMED#1" in status_line(rig.client)
