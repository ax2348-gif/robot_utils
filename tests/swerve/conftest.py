"""Fixtures for the swerve tests: a fresh sparkcan virtual bus per test (from rev_ctl tests/conftest.py)."""
import itertools

import pytest

from sparkcan.bus import CANBus

_names = itertools.count()


@pytest.fixture
def iface():
    """A fresh virtual interface name per test, closed afterwards."""
    name = f"virtual:swerve{next(_names)}"
    yield name
    bus = CANBus._instances.get(name)
    if bus is not None:
        bus.close()
