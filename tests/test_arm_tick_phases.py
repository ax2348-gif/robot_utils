"""Tick-phase timing (arm/phases.py, ``--tick-phases``): what it records per tick, its statistics,
the hardware bus's read split, the node wiring and the CLI options."""
import numpy as np
import pytest

from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.__main__ import parse_cpus
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.node import DT, ArmNode
from robot_utils.nodes.arm.phases import PHASES, TickPhases
from robot_utils.nodes.common.locks import LockStore

from test_arm_feedback_refresh import clock, make_bus, tick   # noqa: F401 (fixture)

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)


class FakeClocks:
    def __init__(self):
        self.t = 0.0
        self.cpu = 0.0

    def wall(self):
        return self.t

    def thread(self):
        return self.cpu


def one_tick(ph, c, wake, read, control, send, publish, cpu):
    ph.begin(wake)
    for name, dt in (("read", read), ("control", control), ("send", send), ("control", 0.0), ("publish", publish)):
        c.t += dt
        ph.mark(name)
    c.cpu += cpu
    ph.end()


def test_records_each_phase_and_work_and_cpu():
    c = FakeClocks()
    ph = TickPhases(capacity=10, clock=c.wall, cpu_clock=c.thread)
    one_tick(ph, c, wake=0.0004, read=0.002, control=0.001, send=0.004, publish=0.0015, cpu=0.003)
    row = dict(zip(PHASES, ph.rows[0]))
    assert row["wake"] == pytest.approx(0.0004) and row["read"] == pytest.approx(0.002)
    assert row["control"] == pytest.approx(0.001) and row["send"] == pytest.approx(0.004)
    assert row["publish"] == pytest.approx(0.0015) and row["work"] == pytest.approx(0.0085)
    assert row["cpu"] == pytest.approx(0.003)
    assert row["read_collect"] == 0.0 and row["read_request"] == 0.0     # no split from this bus
    assert ph.n == 1


def test_unmarked_time_is_charged_to_control_and_early_ticks_are_dropped():
    c = FakeClocks()
    ph = TickPhases(capacity=10, clock=c.wall, cpu_clock=c.thread)
    ph.begin(0.0)
    c.t += 0.003
    ph.mark("read")
    c.t += 0.002                     # nothing marked before end: computation
    ph.end()
    assert dict(zip(PHASES, ph.rows[0]))["control"] == pytest.approx(0.002)
    ph.begin(0.0)                    # a tick that returns early (a fault) never ends ...
    ph.begin(0.0)                    # ... and the next begin discards it
    ph.end()
    assert ph.n == 2
    ph.mark("read")                  # outside a tick: ignored
    ph.end()
    assert ph.n == 2


def test_stats_percentiles_worst_and_overruns_from_wake_plus_work():
    c = FakeClocks()
    ph = TickPhases(capacity=100, clock=c.wall, cpu_clock=c.thread)
    for i in range(98):
        c.t = i * DT
        one_tick(ph, c, wake=0.0001, read=0.002, control=0.001, send=0.004, publish=0.001, cpu=0.002)
    c.t = 98 * DT
    one_tick(ph, c, wake=0.003, read=0.002, control=0.001, send=0.006, publish=0.001, cpu=0.002)    # 13 ms
    c.t = 99 * DT
    one_tick(ph, c, wake=0.0001, read=0.002, control=0.001, send=0.013, publish=0.001, cpu=0.002)  # 17.1 ms
    st = ph.stats()
    assert st["ticks"] == 100 and st["recorded"] == 100
    assert st["phases"]["send"]["p50"] == pytest.approx(4.0) and st["phases"]["send"]["max"] == pytest.approx(13.0)
    assert st["over_10ms"] == 2 and st["over_15ms"] == 1
    worst = st["worst"]
    assert worst[0]["send"] == pytest.approx(13.0) and worst[0]["age_s"] == 0.0
    assert worst[1]["wake"] == pytest.approx(3.0) and worst[1]["age_s"] == pytest.approx(DT)
    assert len(worst) == 10


def test_ring_keeps_the_last_capacity_ticks():
    c = FakeClocks()
    ph = TickPhases(capacity=5, clock=c.wall, cpu_clock=c.thread)
    for i in range(12):
        one_tick(ph, c, wake=0.0, read=0.001 * (i + 1), control=0.0, send=0.0, publish=0.0, cpu=0.0)
    st = ph.stats()
    assert st["ticks"] == 5 and st["recorded"] == 12
    assert st["phases"]["read"]["max"] == pytest.approx(12.0) and st["phases"]["read"]["p50"] == pytest.approx(10.0)


def test_empty_stats():
    assert TickPhases().stats() == {"ticks": 0, "recorded": 0, "phases": {}, "worst": []}


def test_hardware_bus_splits_its_read(clock):   # noqa: F811 (fixture)
    b, wire = make_bus(clock)
    assert b.read_split is None
    tick(b, wire)
    collect, request = b.read_split
    assert collect >= 0.0 and request > 0.0
    tick(b, wire)
    assert b.read_split is not None


def test_node_records_phases_on_the_sim(tmp_path):
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
    t = [1000.0]
    limits = joint_limits(load_cfg(C.PROFILES["soft"]))
    node = ArmNode(lambda: SimBus(Q0, clock=lambda: t[0], limits=limits), locks=LockStore(tmp_path / "locks"),
                   clock=lambda: t[0], tick_phases=True)
    for _ in range(20):
        t[0] += DT
        node.step()
    assert node.state == "uninitialized"
    ph = node.phases
    assert ph.n == 19                              # the first step connects
    rows = ph.rows[:ph.n]
    col = {n: i for i, n in enumerate(PHASES)}
    assert np.all(rows[:, col["read"]] > 0) and np.all(rows[:, col["publish"]] > 0)
    assert np.all(rows[:, col["send"]] == 0)       # motors off: no MIT frames
    assert np.all(rows[:, col["work"]] >= rows[:, col["read"]] + rows[:, col["publish"]])
    assert ph.stats()["ticks"] == 19


def test_node_without_tick_phases_records_nothing(tmp_path):
    node = ArmNode(lambda: SimBus(Q0), locks=LockStore(tmp_path / "locks"))
    assert node.phases is None and node.rt_cpus is None


def test_parse_cpus():
    assert parse_cpus(None) is None and parse_cpus("") is None
    assert parse_cpus("3") == {3}
    assert parse_cpus("1-3") == {1, 2, 3}
    assert parse_cpus("0, 2-3") == {0, 2, 3}
