"""A simulated base: 8 SPARKs on a sparkcan virtual bus, configured as setup_sparks would.

    sims, interface = start_sim()      # e.g. interface "virtual:swerve-sim-1"
    node = SwerveNode(interface) ...
    stop_sim(sims, interface)

Used by ``python -m robot_utils.nodes.swerve --sim`` and the tests. No hardware.
"""
from __future__ import annotations

import itertools

from sparkcan.bus import CANBus
from sparkcan.sim import SimulatedSpark

from . import setup_sparks
from .constants import MODULES

_names = itertools.count(1)


def start_sim(interface: str | None = None) -> tuple[dict, str]:
    interface = interface or f"virtual:swerve-sim-{next(_names)}"
    sims = {}
    for m in MODULES:
        sims[m.steer_id] = SimulatedSpark(interface, m.steer_id).start()
        sims[m.drive_id] = SimulatedSpark(interface, m.drive_id).start()
    for _, can_id, config in setup_sparks.targets():
        sims[can_id].params.update({int(p): v for p, v in config.flatten()})   # as if applied
    return sims, interface


def stop_sim(sims: dict, interface: str) -> None:
    for s in sims.values():
        s.close()
    bus = CANBus._instances.get(interface)
    if bus is not None:
        bus.close()
