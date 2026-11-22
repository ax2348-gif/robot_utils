"""The bus driver's native reader thread raised to real time at every connect (common/realtime.py
``raise_native_threads``, arm/node.py ``_raise_bus_threads``; docs/arm.md "Processes"): motorbridge's polling worker
starts with the bus and, created by the real-time control thread, would otherwise be a normal thread."""
import _thread
import os
import threading
import time

import pytest
from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg

from helpers import Clock
from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.node import DT, ArmNode
from robot_utils.nodes.common import realtime
from robot_utils.nodes.common.locks import LockStore

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)
LIMITS = joint_limits(load_cfg(C.PROFILES["soft"]))
FIFO = 1


class NativeBus(SimBus):
    """A 'hardware' bus whose connect starts a thread outside ``threading`` (as libmotor_abi's polling worker)."""
    sim = False
    started: list = []

    def connect(self) -> None:
        super().connect()
        ev = threading.Event()
        _thread.start_new_thread(lambda: (NativeBus.started.append(threading.get_native_id()), ev.set(),
                                          time.sleep(1.0)), ())
        ev.wait(1)


@pytest.fixture
def fake_os(monkeypatch):
    """The scheduler calls recorded (and the task listing faked where there is no /proc)."""
    calls, pol = [], {}
    monkeypatch.setattr(os, "SCHED_FIFO", FIFO, raising=False)
    monkeypatch.setattr(os, "SCHED_OTHER", 0, raising=False)
    monkeypatch.setattr(os, "sched_param", lambda p: p, raising=False)

    def setsched(tid, policy, prio):
        calls.append((tid, policy & ~getattr(os, "SCHED_RESET_ON_FORK", 0), prio))
        if pol.get("deny"):
            raise PermissionError(1, "Operation not permitted")
        pol[tid] = FIFO
    monkeypatch.setattr(os, "sched_setscheduler", setsched, raising=False)
    monkeypatch.setattr(os, "sched_getscheduler", lambda tid: pol.get(tid, 0), raising=False)
    return calls, pol


def node_with(bus_cls, tmp_path):
    clock = Clock()
    n = ArmNode(lambda: bus_cls(Q0, clock=clock, limits=LIMITS), locks=LockStore(tmp_path / "locks"), clock=clock)
    return n, clock


def connect(n, clock):
    clock.t += DT
    n.step()
    assert n.state == "uninitialized", n.error


def test_the_one_new_native_thread_is_raised_below_the_loop_on_every_connect(tmp_path, fake_os, monkeypatch):
    calls, _ = fake_os
    tids = iter([{1, 2}, {1, 2, 101}, {1, 2}, {1, 2, 102}])          # before / after each connect
    monkeypatch.setattr(realtime, "task_ids", lambda: next(tids))
    n, clock = node_with(NativeBus, tmp_path)
    connect(n, clock)
    assert calls == [(101, FIFO, realtime.PRIO_BUS_IO)] and realtime.PRIO_BUS_IO == 39
    bt = n.state_dict()["realtime"]["bus_threads"]
    assert bt == [{"tid": 101, "name": "?", "policy": "SCHED_FIFO", "priority": 39, "error": None}]
    assert any(e["msg"].startswith("bus reader thread is real time: tid 101") for e in n.events.list())
    n._reconnect = True                                              # a reconnect: a new worker, raised again
    clock.t += DT
    n.step()
    assert calls[-1] == (102, FIFO, 39) and n.state_dict()["realtime"]["bus_threads"][0]["tid"] == 102


@pytest.mark.parametrize("case", ["none", "two", "denied", "no_proc"])
def test_otherwise_a_warning_and_the_connect_goes_on(tmp_path, fake_os, monkeypatch, case):
    calls, pol = fake_os
    after = {"none": {1, 2}, "two": {1, 2, 101, 102}, "denied": {1, 2, 101}, "no_proc": None}[case]
    tids = iter([None if case == "no_proc" else {1, 2}, after])
    monkeypatch.setattr(realtime, "task_ids", lambda: next(tids))
    pol["deny"] = case == "denied"
    n, clock = node_with(NativeBus, tmp_path)
    connect(n, clock)
    rt = n.state_dict()["realtime"]
    if case == "no_proc":                                            # macOS: nothing to do, nothing said
        assert calls == [] and rt["bus_threads"] is None
        return
    warn = [e for e in n.events.list() if "NOT made real time" in e["msg"]]
    assert len(warn) == 1 and warn[0]["level"] == "warn", n.events.list()
    if case == "denied":
        assert calls == [(101, FIFO, 39)] and "RLIMIT_RTPRIO" in rt["bus_threads"][0]["error"]
    else:
        assert calls == [] and len(rt["bus_threads"]) == (0 if case == "none" else 2)


def test_a_simulated_bus_is_left_alone(tmp_path, fake_os, monkeypatch):
    calls, _ = fake_os
    monkeypatch.setattr(realtime, "task_ids", lambda: {1, 2, 3})
    n, clock = node_with(SimBus, tmp_path)
    connect(n, clock)
    assert calls == [] and n.state_dict()["realtime"]["bus_threads"] is None


@pytest.mark.skipif(not os.path.isdir("/proc/self/task"), reason="needs Linux /proc")
def test_the_real_task_listing_finds_the_native_thread(tmp_path, fake_os):
    calls, _ = fake_os
    NativeBus.started.clear()
    n, clock = node_with(NativeBus, tmp_path)
    connect(n, clock)
    assert [c[0] for c in calls] == NativeBus.started


def test_switched_off(monkeypatch):
    monkeypatch.setenv("ROBOT_UTILS_RT", "0")
    rec = realtime.raise_native_threads({1}, 39)
    assert rec["threads"] == [] and "switched off" in rec["error"]


def test_never_raises(monkeypatch):
    monkeypatch.setattr(realtime, "task_ids", lambda: (_ for _ in ()).throw(RuntimeError("boom")))
    rec = realtime.raise_native_threads({1}, 39)
    assert "boom" in rec["error"]
