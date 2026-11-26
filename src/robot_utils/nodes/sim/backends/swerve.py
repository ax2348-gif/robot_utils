"""The swerve node's base on the sim world: world-coupled SPARKs (docs/sim.md, "Swerve backend").

    sims, interface, coupler = start_world_sim("sim")      # 8 WorldSparks on "virtual:swerve-world-1"
    node = SwerveNode(interface) ...
    stop_world_sim(sims, interface, coupler)

The eight ``WorldSpark``s are sparkcan ``SimulatedSpark``s, so the swerve node keeps its whole CAN path
(heartbeats, parameters, status frames, setup_sparks' configuration); only their motion comes from the
world. Every 2 ms one ``Coupler`` thread sends the world each module's latest setpoints and enable state
(``swerve.sock`` op ``modules``) and keeps the module states it answers:

- steering SPARK: a POSITION setpoint on the absolute encoder (0..1, wrapping) -> the module angle
  ``(setpoint - steer_offset) x 2 pi``; it reports the world's module angle back as that encoder reading
  (status 5, honouring the encoder offset / inversion parameters);
- drive SPARK: a VELOCITY setpoint in m/s (setup_sparks' conversion factors) -> the wheel speed; it reports
  the wheel's speed and travel (status 2, motor RPM and rotations through the same factors).

A SPARK counts as commanding only while the node's heartbeat enables it (``SimulatedSpark.enabled``); if
the coupler stops, the world disables every module after 0.1 s (as a lost heartbeat).
"""
from __future__ import annotations

import itertools
import math
import threading
import time
from typing import Optional

from sparkcan.bus import CANBus
from sparkcan.sim import SimulatedSpark
from sparkcan.types import ControlType

from ...swerve import setup_sparks
from ...swerve.constants import DRIVE_MPS_PER_RPM, MODULES
from ..protocol import IpcClient, IpcError, backend_socket

_P = __import__("sparkcan._params", fromlist=["SparkParameter"]).SparkParameter
_names = itertools.count(1)
PERIOD = 0.002


class Coupler:
    """Exchanges every module's setpoints and states with the world (one thread, one connection)."""

    def __init__(self, sim: str = "sim", socket_path=None):
        self.client = IpcClient(socket_path or backend_socket(sim, "swerve"), timeout=2.0)
        self.steer: dict = {}       # module -> (enabled, angle rad)
        self.drive: dict = {}       # module -> (enabled, speed m/s)
        self.states: dict = {}      # module -> {steer, steer_rate, wheel, speed}
        self.t_world: Optional[float] = None
        self.error: Optional[str] = None
        self.exchanges = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        hello = self.client.call("hello")[0]
        self.modules = hello["modules"]
        self.radius = dict(zip(self.modules, hello["wheel_radius"]))     # m
        self._exchange()
        self._thread = threading.Thread(target=self._loop, name="swerve-world-coupler", daemon=True)
        self._thread.start()

    def set_steer(self, module: str, enabled: bool, angle: float) -> None:
        with self._lock:
            self.steer[module] = (bool(enabled), float(angle))

    def set_drive(self, module: str, enabled: bool, speed: float) -> None:
        with self._lock:
            self.drive[module] = (bool(enabled), float(speed))

    def state(self, module: str) -> Optional[dict]:
        with self._lock:
            return self.states.get(module)

    def _exchange(self) -> None:
        with self._lock:
            steer, drive = dict(self.steer), dict(self.drive)
        r = self.client.call("modules", steer=steer, drive=drive)[0]
        with self._lock:
            self.states = r["modules"]
            self.t_world = r["t"]
        self.exchanges += 1

    def _loop(self) -> None:
        nxt = time.monotonic()
        while not self._stop.is_set():
            try:
                self._exchange()
                self.error = None
            except IpcError as e:
                self.error = str(e)
            nxt += PERIOD
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)
        self.client.close()


class WorldSpark(SimulatedSpark):
    """A SimulatedSpark whose motion is a swerve module's in the world (steer or drive)."""

    def __init__(self, interface: str, device_id: int, coupler: Coupler, module, role: str):
        self.coupler, self.module, self.role = coupler, module, role
        self._pos_offset = 0.0              # drive: what SET_PRIMARY_ENCODER_POSITION moved the reading by
        self._pos_written: Optional[float] = None   # drive: the reading this class last wrote
        super().__init__(interface, device_id)

    def _step(self, dt: float) -> None:
        sp = self.setpoint if self.enabled else None
        p = self.params
        if self.role == "steer":
            on = sp is not None and sp[0] in (ControlType.POSITION, ControlType.MAX_MOTION_POSITION_CONTROL)
            angle = (sp[1] - self.module.steer_offset) * 2 * math.pi if on else 0.0
            self.coupler.set_steer(self.module.name, on, (angle + math.pi) % (2 * math.pi) - math.pi)
        else:
            on = sp is not None and sp[0] in (ControlType.VELOCITY, ControlType.MAX_MOTION_VELOCITY_CONTROL)
            factor = p[int(_P.VELOCITY_CONVERSION_FACTOR)] or 1.0
            self.coupler.set_drive(self.module.name, on, sp[1] / factor * DRIVE_MPS_PER_RPM if on else 0.0)
        st = self.coupler.state(self.module.name)
        if st is None:
            return
        if self.role == "steer":
            # the encoder reads (angle / 2 pi + offset) mod 1; position is set so that status 5 reports it
            # (status 5 = (position - DUTY_CYCLE_OFFSET) mod 1, inverted if DUTY_CYCLE_INVERTED)
            reading = (st["steer"] / (2 * math.pi) + self.module.steer_offset) % 1.0
            inverted = bool(p[int(_P.DUTY_CYCLE_INVERTED)])
            self.position = ((1.0 - reading) % 1.0 if inverted else reading) + p[int(_P.DUTY_CYCLE_OFFSET)]
            rps = st["steer_rate"] / (2 * math.pi)
            self.velocity = (-rps if inverted else rps) * 60.0
            self.applied_output = max(-0.3, min(0.3, 0.5 * (st["steer_rate"] / 12.566)))
        else:
            rpm = st["speed"] / DRIVE_MPS_PER_RPM                     # motor RPM for the wheel speed
            self.velocity = rpm
            # wheel travel (m) / m per motor rotation = motor rotations, plus the offset of an encoder set: a
            # SET_PRIMARY_ENCODER_POSITION (the swerve node's rebase) wrote self.position since the last step (both
            # under the SPARK's lock). Without it the next step overwrote the set and every rebase timed out: 0.5 s
            # with the odometry dropped and the base held still, retried every 10 s (2026-11-25)
            if self._pos_written is not None and self.position != self._pos_written:
                self._pos_offset += self.position - self._pos_written
            raw = st["wheel"] * self.coupler.radius[self.module.name] / (DRIVE_MPS_PER_RPM * 60)
            self.position = self._pos_written = raw + self._pos_offset
            self.applied_output = max(-0.99, min(0.99, rpm / 6784.0))


def start_world_sim(sim: str = "sim", interface: Optional[str] = None, socket_path=None) -> tuple[dict, str, Coupler]:
    """Eight world-coupled SPARKs on a virtual bus, configured as setup_sparks would (as swerve.sim.start_sim)."""
    interface = interface or f"virtual:swerve-world-{next(_names)}"
    coupler = Coupler(sim, socket_path)
    sims = {}
    for m in MODULES:
        if m.name not in coupler.modules:
            coupler.close()
            raise RuntimeError(f"the sim world's model has no module {m.name} (has {coupler.modules})")
        sims[m.steer_id] = WorldSpark(interface, m.steer_id, coupler, m, "steer").start()
        sims[m.drive_id] = WorldSpark(interface, m.drive_id, coupler, m, "drive").start()
    for _, can_id, config in setup_sparks.targets():
        sims[can_id].params.update({int(p): v for p, v in config.flatten()})   # as if applied
    return sims, interface, coupler


def stop_world_sim(sims: dict, interface: str, coupler: Coupler) -> None:
    for s in sims.values():
        s.close()
    coupler.close()
    bus = CANBus._instances.get(interface)
    if bus is not None:
        bus.close()
