"""The arm row's measurement times (t_meas, t_meas_mot): appended last, never after the tick, and on a pipelined
bus about a tick older than t_tick (docs/arm.md, "When the data was measured")."""
import dataclasses

import numpy as np
import pytest

from helpers import Clock
from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.node import COL, DT, FIELDS, UNITS, ArmNode
from robot_utils.nodes.common.locks import LockStore

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)


class PipelinedBus(SimBus):
    """A SimBus whose replies answer the previous tick's requests: the output encoders of a read are the previous
    read's sample (requested then), the motor feedback arrived 5 ms into the tick (the previous MIT frames')."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._prev = None

    def read(self):
        now = super().read()
        prev, self._prev = self._prev, now
        if prev is None:          # the first read waits for its own replies
            return now
        return dataclasses.replace(now, q=prev.q, g_q=prev.g_q, t_meas=prev.t, t_meas_mot=prev.t + 0.005)


def run(tmp_path, bus_cls):
    clock = Clock()

    def factory():
        from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
        return bus_cls(Q0, clock=clock, limits=joint_limits(load_cfg(C.PROFILES["soft"])))

    n = ArmNode(factory, locks=LockStore(tmp_path / "locks"), clock=clock)
    for _ in range(20):
        clock.t += DT
        n.step()
    return n.hub.recent(10, "tick")


def test_measurement_columns_are_appended():
    assert FIELDS[-5:-3] == ["t_meas", "t_meas_mot"] and UNITS[-5:-3] == ["s", "s"]
    assert COL["jam"] == len(FIELDS) - 6                # the columns before keep their positions
    assert FIELDS[-3:] == ["stream_seq", "stream_age", "stream_lat"]    # then the joint channel's, last


@pytest.mark.parametrize("bus_cls", [SimBus, PipelinedBus])
def test_measurement_times_never_after_the_tick(tmp_path, bus_cls):
    rows = run(tmp_path, bus_cls)
    t, tm, tmm = rows[:, COL["t_tick"]], rows[:, COL["t_meas"]], rows[:, COL["t_meas_mot"]]
    assert np.all(np.isfinite(tm)) and np.all(np.isfinite(tmm))
    assert np.all(tm <= t) and np.all(tmm <= t)
    if bus_cls is SimBus:                               # sampled at the read: no pipeline delay
        assert np.all(tm == t) and np.all(tmm == t)


def test_pipelined_bus_data_is_about_a_tick_old(tmp_path):
    rows = run(tmp_path, PipelinedBus)
    t, tm, tmm = rows[:, COL["t_tick"]], rows[:, COL["t_meas"]], rows[:, COL["t_meas_mot"]]
    assert t - tm == pytest.approx(np.full(len(t), DT), abs=1e-9)
    assert np.all(tmm > tm)                             # the motor feedback is newer than q in the same row
