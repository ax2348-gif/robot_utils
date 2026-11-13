"""common.realtime: the request for SCHED_FIFO never raises and reports what happened."""
import os

import pytest

from robot_utils.nodes.common import realtime
from robot_utils.nodes.common.realtime import RealtimeOut


def fake_os(monkeypatch, setter):
    for name, value in (("SCHED_FIFO", 1), ("SCHED_RESET_ON_FORK", 0x40000000), ("sched_param", lambda p: p),
                        ("sched_setscheduler", setter)):
        monkeypatch.setattr(os, name, value, raising=False)


def test_granted(monkeypatch):
    calls = []
    fake_os(monkeypatch, lambda pid, policy, param: calls.append((pid, policy, param)))
    rt = realtime.make_realtime(40, "test")
    assert rt == {"policy": "SCHED_FIFO", "priority": 40, "requested": 40, "error": None}
    assert calls == [(0, 1 | 0x40000000, 40)]          # the calling thread, reset in children
    assert "SCHED_FIFO 40" in realtime.describe(rt)
    RealtimeOut(**rt)


def test_refused_is_reported_not_raised(monkeypatch):
    def refuse(*a):
        raise PermissionError(1, "Operation not permitted")
    fake_os(monkeypatch, refuse)
    rt = realtime.make_realtime(40, "test")
    assert rt["policy"] == "SCHED_OTHER" and rt["priority"] == 0 and rt["requested"] == 40
    assert "RLIMIT_RTPRIO" in rt["error"] and "NOT real time" in realtime.describe(rt)
    RealtimeOut(**rt)


def test_switched_off(monkeypatch):
    fake_os(monkeypatch, lambda *a: pytest.fail("must not be called"))
    monkeypatch.setenv("ROBOT_UTILS_RT", "0")
    rt = realtime.make_realtime(40)
    assert rt["policy"] == "SCHED_OTHER" and rt["requested"] == 0 and "ROBOT_UTILS_RT" in rt["error"]


def test_other_os(monkeypatch):
    monkeypatch.delattr(os, "sched_setscheduler", raising=False)
    rt = realtime.make_realtime(40)
    assert rt["policy"] == "SCHED_OTHER" and "not available" in rt["error"]


def test_not_started_matches_the_model():
    RealtimeOut(**realtime.not_started(20))
