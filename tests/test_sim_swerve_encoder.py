"""The sim's drive SPARK (sim/backends/swerve.py WorldSpark): an encoder set (SET_PRIMARY_ENCODER_POSITION, the
swerve node's rebase) holds. The world rewrote the reading every step, so every rebase timed out (0.5 s with the
odometry dropped and the base held still, retried every 10 s)."""
import math
from collections import defaultdict
from types import SimpleNamespace

import pytest

from robot_utils.nodes.sim.backends import swerve as B


class StubCoupler:
    def __init__(self):
        self.wheel = 10.0                         # rad of wheel rotation in the world
        self.radius = {"fl": 0.05}

    def set_drive(self, *a):
        pass

    def set_steer(self, *a):
        pass

    def state(self, module):
        return {"wheel": self.wheel, "speed": 0.0, "steer": 0.0, "steer_rate": 0.0}


def drive_spark(coupler):
    s = object.__new__(B.WorldSpark)              # no virtual bus: just the step
    s.coupler, s.module, s.role = coupler, SimpleNamespace(name="fl", steer_offset=0.0), "drive"
    s._pos_offset, s._pos_written = 0.0, None
    s.setpoint, s._last_enable, s.heartbeat_timeout, s.params = None, -math.inf, 0.1, defaultdict(float)
    s.position = s.velocity = s.applied_output = 0.0
    return s


def test_an_encoder_set_holds_and_the_reading_then_follows_the_wheel():
    c = StubCoupler()
    s = drive_spark(c)
    s._step(0.01)
    per_rad = c.radius["fl"] / (B.DRIVE_MPS_PER_RPM * 60)
    assert s.position == pytest.approx(10.0 * per_rad)
    s.position = 0.0                              # SET_PRIMARY_ENCODER_POSITION 0 (the rebase)
    s._step(0.01)
    assert abs(s.position) < 1e-12                # held (it was overwritten with the world's 10 rad before)
    c.wheel = 12.0
    s._step(0.01)
    assert s.position == pytest.approx(2.0 * per_rad)    # moves with the wheel from the new zero
    s.position = 5.0                              # a second set
    c.wheel = 13.0
    s._step(0.01)
    assert s.position == pytest.approx(5.0 + 1.0 * per_rad)
