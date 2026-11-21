"""The arm bus's per-tick requests (HardwareBus over a fake RebotArm and motorbridge): the output
encoder (xout) read every tick for every motor, the MIT frame (answered with a feedback frame), and
a feedback refresh only for motors that got no MIT frame the tick before or whose MIT frames went
unanswered (bus.FeedbackRefresh). docs/arm.md."""
import logging
from collections import Counter
from types import SimpleNamespace

import numpy as np
import pytest

from robot_utils.nodes.arm import bus as busmod
from robot_utils.nodes.arm.bus import HardwareBus, ReadLate

NAMES = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
DT = 0.01


class Clock:
    """time.monotonic for the bus: advances 1 µs per call (strictly increasing), ticks by hand."""

    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        self.t += 1e-6
        return self.t


class Wire:
    """The bus: frames sent since the last poll, answered at the next poll (one controller)."""

    def __init__(self, clock):
        self.clock = clock
        self.pending = []           # (motor, kind)
        self.sent = Counter()       # (motor, kind) -> frames sent since clear()
        self.unanswered = set()     # motors whose MIT frames get no feedback frame
        self.drop_xout = set()      # motors whose next xout request gets no reply
        self.motors = {}

    def send(self, name, kind):
        self.pending.append((name, kind))
        self.sent[(name, kind)] += 1

    def poll(self):
        now = self.clock.monotonic()
        pending, self.pending = self.pending, []
        for name, kind in pending:
            m = self.motors[name]
            if kind == "xout":
                if name in self.drop_xout:
                    self.drop_xout.discard(name)
                    continue
                m.xout, m.xout_fresh = (0.1, now), True
            elif kind == "mit" and name in self.unanswered:
                continue
            else:                   # a refresh or an MIT frame: a feedback frame
                m.state = (SimpleNamespace(pos=0.1, vel=0.0, torq=0.0, status_code=1, t_mos=35.0, t_rotor=38.0), now)
                m.seq += 1

    def per_motor(self, kind):
        return {n: self.sent[(n, kind)] for n in NAMES}


class OldMotor:
    """motorbridge.core.Motor before the feedback's age (fork 0.5.6+2ec.1)."""

    def __init__(self, name, wire):
        self.name, self.wire = name, wire
        self.xout = None            # (value, arrived)
        self.xout_fresh = False     # the cached reply answers the latest request
        self.state = None           # (state, arrived)
        self.seq = 0
        wire.motors[name] = self

    def request_register(self, rid):
        assert rid == 81
        self.xout_fresh = False
        self.wire.send(self.name, "xout")

    def get_cached_register_f32(self, rid, fresh_only=True):
        if self.xout is None or not self.xout_fresh:
            return None
        return self.xout[0], self.wire.clock.monotonic() - self.xout[1]

    def request_feedback(self):
        self.wire.send(self.name, "refresh")

    def get_state(self):
        return None if self.state is None else self.state[0]


class Motor(OldMotor):
    """With get_state_with_age (the fork's next release)."""

    def get_state_with_age(self):
        if self.state is None:
            return None
        return self.state[0], self.wire.clock.monotonic() - self.state[1], self.seq


class Group:
    def __init__(self, name, joint_names, wire):
        self.name, self.joint_names, self.wire = name, joint_names, wire
        self._q_refused = np.zeros(len(joint_names), dtype=bool)
        self._q_last = np.zeros(len(joint_names))

    def send_mit(self, q, vel, kp, kd, tau):
        for n in self.joint_names:
            self.wire.send(n, "mit")

    def enable(self): ...
    def disable(self): ...
    def clear_error(self): ...
    def mode_mit(self): return True


class FakeArm:
    """The parts of rebot_ctl's RebotArm the HardwareBus uses."""

    def __init__(self, wire, motor_cls=Motor):
        self.wire = wire
        self.setpoint_limits = {}
        self._all_joints = [SimpleNamespace(name=n) for n in NAMES]
        self._motor_map = {n: motor_cls(n, wire) for n in NAMES}
        self._ctrl_map = {"bridge": SimpleNamespace(poll_feedback_once=wire.poll)}
        self.arm = Group("arm", NAMES[:6], wire)
        self.gripper = Group("gripper", ["gripper"], wire)

    def _setup_motors(self): ...
    def check_can_timeout(self): ...
    def check_joint_limits(self): ...
    def disable_all(self): ...


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(busmod, "time", c)
    return c


def make_bus(clock, motor_cls=Motor):
    wire = Wire(clock)
    b = HardwareBus("unused.yaml", arm=FakeArm(wire, motor_cls))
    b.connect()
    return b, wire


def tick(b, wire, send=True, gripper=True):
    """One control tick (read, then the MIT frames); the frames it sent, per kind and motor."""
    wire.clock.t += DT
    wire.sent.clear()
    try:
        b.read()
        late = False
    except ReadLate:
        late = True
    if send:
        z = np.zeros(6)
        b.send(z, z, z, z, z, 0.0, 0.0, 0.0, 0.0, 0.0, gripper=gripper)
    return SimpleNamespace(xout=wire.per_motor("xout"), refresh=wire.per_motor("refresh"),
                           mit=wire.per_motor("mit"), late=late)


ALL = dict.fromkeys(NAMES, 1)
NONE = dict.fromkeys(NAMES, 0)
TWICE = dict.fromkeys(NAMES, 2)


def test_motors_sent_mit_need_no_refresh_two_frames_per_motor_per_tick(clock):
    b, wire = make_bus(clock)
    assert b.refresh.skip
    first = tick(b, wire)                   # the first read after connecting: no MIT frames yet
    assert first.refresh == TWICE and first.xout == TWICE   # (requests, waits, requests for the next tick)
    for _ in range(50):
        f = tick(b, wire)
        assert f.xout == ALL                # the output encoder, every tick
        assert f.mit == ALL
        assert f.refresh == NONE            # MIT replies keep the feedback fresh
        assert all(f.xout[n] + f.mit[n] + f.refresh[n] == 2 for n in NAMES)
    assert b.refresh.fallbacks == {} and b.refresh.refreshes == 2 * len(NAMES)
    assert b.reply_latency == pytest.approx(DT, abs=1e-4)   # still timed (xout replies: the next poll here)


def test_motors_off_refresh_every_motor_every_tick(clock):
    b, wire = make_bus(clock)
    tick(b, wire, send=False)
    for _ in range(10):
        f = tick(b, wire, send=False)
        assert f.refresh == ALL and f.xout == ALL and f.mit == NONE
    assert b.refresh.fallbacks == {}


def test_gripper_in_damping_is_refreshed_the_arm_is_not(clock):
    b, wire = make_bus(clock)
    tick(b, wire, gripper=True)
    tick(b, wire, gripper=True)
    b.gripper_motor(False)                  # damping: no frames from the next send
    f = tick(b, wire, gripper=False)        # last tick's MIT frame to the gripper was forgotten
    assert f.refresh == {**NONE, "gripper": 1}
    for _ in range(10):
        f = tick(b, wire, gripper=False)
        assert f.refresh == {**NONE, "gripper": 1} and f.xout == ALL
        assert f.mit == {**ALL, "gripper": 0}
    b.gripper_motor(True)
    f = tick(b, wire, gripper=True)         # its first MIT frame goes out after this read
    assert f.refresh == {**NONE, "gripper": 1}
    f = tick(b, wire, gripper=True)
    assert f.refresh == NONE
    assert b.refresh.fallbacks == {}


def test_disable_and_enable_refresh_the_first_reads(clock):
    b, wire = make_bus(clock)
    for _ in range(5):
        tick(b, wire)
    b.disable()                             # after this tick's MIT frames: they promise no more
    f = tick(b, wire, send=False)
    assert f.refresh == ALL
    f = tick(b, wire, send=False)
    assert f.refresh == ALL
    b.enable()
    f = tick(b, wire)                       # the first read after enabling: no MIT frame yet
    assert f.refresh == ALL
    f = tick(b, wire)
    assert f.refresh == NONE and f.xout == ALL
    assert b.refresh.fallbacks == {}


def test_unanswered_mit_frames_fall_back_to_refreshes_and_count(clock):
    b, wire = make_bus(clock)
    wire.unanswered = {"joint3"}
    tick(b, wire)
    since = {n: 0 for n in NAMES}            # ticks since each motor's feedback was refreshed
    worst = 0
    for _ in range(60):
        f = tick(b, wire)
        assert f.xout == ALL
        for n in NAMES:
            since[n] = 0 if f.refresh[n] else since[n] + 1
        worst = max(worst, since["joint3"])
        assert all(f.refresh[n] == 0 for n in NAMES if n != "joint3")
    assert set(b.refresh.fallbacks) == {"joint3"}
    assert 15 <= b.refresh.fallbacks["joint3"] <= 30   # about every third tick
    assert worst <= 2                        # never more than two ticks without a refresh
    age = b.arm._motor_map["joint3"].get_state_with_age()[1]
    assert age < 3.5 * DT


def test_one_late_mit_reply_is_no_fallback(clock):
    b, wire = make_bus(clock)
    for _ in range(3):
        tick(b, wire)
    wire.unanswered = {"joint5"}             # one MIT frame unanswered...
    tick(b, wire)
    wire.unanswered = set()                  # ...the next one answered
    for _ in range(5):
        f = tick(b, wire)
        assert f.refresh == NONE
    assert b.refresh.fallbacks == {}


def test_a_late_xout_reply_sends_nothing_that_tick_and_no_fallback(clock):
    b, wire = make_bus(clock)
    for _ in range(3):
        tick(b, wire)
    wire.drop_xout = {"joint2"}
    f = tick(b, wire)                        # its reply is still due: wait a tick, nothing requested
    assert f.late and f.xout == NONE and f.refresh == NONE and f.mit == ALL   # the MIT frames go out
    clock.t += DT
    wire.sent.clear()
    with pytest.raises(busmod.ReadMiss):     # missed after the extra tick; the next requests are sent
        b.read()
    assert wire.per_motor("xout") == ALL and wire.per_motor("refresh") == NONE
    z = np.zeros(6)
    b.send(z, z, z, z, z, 0.0, 0.0, 0.0, 0.0, 0.0)
    for _ in range(5):
        f = tick(b, wire)
        assert f.refresh == NONE and not f.late and f.xout == ALL
    assert b.refresh.fallbacks == {}


def test_a_motorbridge_without_the_feedback_age_refreshes_everything(clock, monkeypatch, caplog):
    monkeypatch.setattr(HardwareBus, "_warned_no_state_age", False)
    with caplog.at_level(logging.WARNING, logger=busmod.__name__):
        b, wire = make_bus(clock, OldMotor)
        make_bus(clock, OldMotor)             # a reconnect: logged once per process
    assert not b.refresh.skip
    assert len([r for r in caplog.records if "get_state_with_age" in r.getMessage()]) == 1
    tick(b, wire)
    for _ in range(10):
        f = tick(b, wire)
        assert f.refresh == ALL and f.xout == ALL and f.mit == ALL   # today's three per motor
    assert b.refresh.fallbacks == {}


def test_a_motor_without_the_feedback_age_refreshes_everything(clock, monkeypatch):
    from motorbridge import CallError

    class NonDamiao(Motor):
        def get_state_with_age(self):
            raise CallError("get_state_with_age failed: state age is only available for Damiao motors")
    monkeypatch.setattr(HardwareBus, "_warned_no_state_age", False)
    b, wire = make_bus(clock, NonDamiao)
    assert not b.refresh.skip
    tick(b, wire)
    assert tick(b, wire).refresh == ALL


def test_mit_frames_dropped_silently_by_the_driver_fall_back_too(clock):
    """rebot_ctl swallows a failed MIT write (except CallError: pass): the bus counts it as sent."""
    b, wire = make_bus(clock)
    orig = wire.send
    wire.send = lambda name, kind: None if (name, kind) == ("joint6", "mit") else orig(name, kind)
    tick(b, wire)
    refreshed = 0
    for _ in range(30):
        f = tick(b, wire)
        refreshed += f.refresh["joint6"]
        assert all(f.refresh[n] == 0 for n in NAMES if n != "joint6")
    assert refreshed >= 8 and b.refresh.fallbacks["joint6"] == refreshed


def test_reading_carries_its_measurement_times(clock):
    """The pipelined bus: tick k's output encoders answer requests sent at the end of tick k-1's read (t_meas about
    one tick before tick k starts); the motor feedback answers tick k-1's MIT frames and arrives during tick k
    (t_meas_mot newer than t_meas)."""
    b, wire = make_bus(clock)
    tick(b, wire)
    starts, readings = [], []
    for _ in range(5):
        clock.t += DT
        starts.append(clock.t)
        readings.append(b.read())
        z = np.zeros(6)
        b.send(z, z, z, z, z, 0.0, 0.0, 0.0, 0.0, 0.0)
    for k in range(1, 5):
        r = readings[k]
        assert starts[k - 1] < r.t_meas < starts[k - 1] + 1e-3        # sent at the end of the previous read
        assert r.t_meas < r.t_meas_mot <= starts[k] + 1e-3            # arrived during this tick's read
        assert starts[k] - r.t_meas == pytest.approx(DT, abs=1e-3)


def test_feedback_arrival_unknown_without_its_age(clock):
    b, wire = make_bus(clock, OldMotor)
    tick(b, wire)
    clock.t += DT
    r = b.read()
    assert np.isfinite(r.t_meas) and np.isnan(r.t_meas_mot)
