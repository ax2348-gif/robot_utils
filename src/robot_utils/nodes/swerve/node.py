"""The swerve node's core: commands in, one telemetry row per control tick out (no web code).

Threads: ``SwerveDrive`` runs the 100 Hz control loop on its own thread (the hardware owner).
This class plugs into it through two hooks that run on that thread every tick:

``_pre_tick``  safety gating and the command handoff, in this order:
               0. chocks (common/locks.py): while any chock covers this node it stops and
                  disables, every tick, even if the web server stalls;
               1. ``lease.expire_if_due()``: a lapsed lease ends here (stops, disables, resets
                  the soft limits), so a lapse is caught at 100 Hz even if the web server stalls;
               2. e-stop latched, or not enabled -> ``drive.stop()``;
               3. read the CommandSlot; no command or older than COMMAND_TIMEOUT (0.25 s) ->
                  ``drive.stop()``; otherwise clamp to the soft limits (which are always within
                  the hard limits) and ``drive.drive(...)``.
``_on_tick``   builds the tick's telemetry row (FIELDS) and publishes it to the RowHub.

Everything else (web requests) only writes small pieces of state under locks: the CommandSlot,
the Lease, the EStop and a few flags. Requests never touch the CAN bus.

Fast path (not implemented yet; see docs/nodes.md): a streaming command endpoint would call
``command(token, twist, seq, source="fast")`` for each received command, i.e. write the same
CommandSlot; the tick reads whatever was written last, from either path.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from sparkcan import PeriodicFrame

from ..common.estop import EStop, EStopInfo
from ..common.hub import RowHub
from ..common.lease import Lease, RequestError
from ..common.locks import LockStore, describe as describe_locks
from ..common.server import EventLog
from .constants import COMMAND_TIMEOUT, MODULES
from .drive import SwerveDrive

HARD_MAX_SPEED = 2.0      # m/s, translation (norm of vx, vy); fixed
HARD_MAX_OMEGA = 4.0      # rad/s; fixed
LEASE_HEARTBEAT = 1.0     # s
SOURCES = {"": 0, "slow": 1, "fast": 2}

_BASE_FIELDS = [
    ("t_tick", "s"), ("tick", "count"), ("motors_enabled", "bool"), ("enabled", "bool"),
    ("estop", "bool"), ("lease_id", "id"), ("cmd_seq", "count"), ("cmd_source", "enum"),
    ("cmd_age", "s"), ("req_vx", "m/s"), ("req_vy", "m/s"), ("req_omega", "rad/s"),
    ("cmd_vx", "m/s"), ("cmd_vy", "m/s"), ("cmd_omega", "rad/s"),
    ("soft_max_speed", "m/s"), ("soft_max_omega", "rad/s"), ("loop_ok", "bool"),
]
_MODULE_FIELDS = [
    ("steer_angle", "rad"), ("steer_vel", "rad/s"), ("t_steer", "s"),
    ("drive_vel", "m/s"), ("drive_pos", "m"), ("t_drive", "s"),
    ("steer_sp", "rad"), ("drive_sp", "m/s"),
    ("steer_out", "duty"), ("steer_current", "A"), ("t_steer_s0", "s"),
    ("drive_out", "duty"), ("drive_current", "A"), ("t_drive_s0", "s"),
]
FIELDS: list[str] = [n for n, _ in _BASE_FIELDS] + [f"{m.name}_{n}" for m in MODULES for n, _ in _MODULE_FIELDS]
UNITS: list[str] = [u for _, u in _BASE_FIELDS] + [u for _ in MODULES for _, u in _MODULE_FIELDS]
FIELD_DOC = {
    "t_tick": "server monotonic time at the start of the control tick",
    "motors_enabled": "1 if the tick sent setpoints and an enabling heartbeat",
    "enabled": "the node's enable flag (POST /enable)",
    "lease_id": "id of the lease in force (0 = none)",
    "cmd_source": "writer of the command in force: 0 none, 1 slow (POST /command), 2 fast",
    "cmd_age": "seconds since that command was written (NaN if none)",
    "req_*": "the command as requested; cmd_* = after the soft-limit clamp, as sent (NaN when stopped)",
    "<module>_steer_angle": "wheel angle from the absolute encoder (0 = forward, CCW +), STATUS_5",
    "<module>_steer_vel": "from the encoder's velocity (REV default unit RPM, converted)",
    "<module>_t_steer / t_drive / t_*_s0": "server monotonic arrival time of that status frame",
    "<module>_drive_pos": "accumulated wheel travel since the SPARK booted (float32 on the wire)",
    "<module>_steer_sp / drive_sp": "setpoints sent this tick (NaN if none)",
}


@dataclass(frozen=True)
class Command:
    twist: tuple[float, float, float]
    seq: int
    source: str
    t: float


class CommandSlot:
    """Latest-command register: any number of writers, the control tick reads the newest."""

    def __init__(self):
        self._lock = threading.Lock()
        self._cmd: Optional[Command] = None

    def write(self, twist, seq: int = 0, source: str = "slow") -> Command:
        cmd = Command(tuple(float(v) for v in twist), int(seq), source, time.monotonic())
        with self._lock:
            self._cmd = cmd
        return cmd

    def read(self) -> Optional[Command]:
        with self._lock:
            return self._cmd

    def clear(self) -> None:
        with self._lock:
            self._cmd = None


def clamp(twist, max_speed: float, max_omega: float) -> tuple[tuple[float, float, float], bool]:
    """Translation scaled (direction kept) to max_speed; omega clipped to max_omega."""
    vx, vy, w = twist
    v = math.hypot(vx, vy)
    clamped = False
    if v > max_speed:
        k = max_speed / v
        vx, vy, clamped = vx * k, vy * k, True
    if abs(w) > max_omega:
        w, clamped = math.copysign(max_omega, w), True
    return (vx, vy, w), clamped


class SwerveNode:
    def __init__(self, interface: str, hub: Optional[RowHub] = None, locks: Optional[LockStore] = None):
        self.interface = interface
        self.locks = locks or LockStore()
        self.lock_names = {"swerve"}        # + the instance name (server.py adds it)
        self.hub = hub or RowHub({"tick": len(FIELDS)})
        self.events = EventLog(on_add=self.hub.state_changed)
        self.slot = CommandSlot()
        self.lease = Lease(LEASE_HEARTBEAT, on_end=self._lease_ended)
        self.estop = EStop(on_trigger=self._estop_triggered)
        self.drive: Optional[SwerveDrive] = None
        self.drive_error: Optional[str] = None
        self.enabled = False
        self.soft = (HARD_MAX_SPEED, HARD_MAX_OMEGA)
        self._applied: Optional[tuple] = None   # (requested cmd, clamped twist) of this tick
        self._tick = 0
        self._lock = threading.Lock()

    # -- lifecycle (blocking: call from a worker thread) -------------------------------------------

    def start(self) -> None:
        try:
            self.drive = SwerveDrive(self.interface, pre_tick=self._pre_tick, on_tick=self._on_tick)
            self.drive_error = None
            self.events.add(f"drive started on {self.interface}")
        except Exception as e:
            self.drive, self.drive_error = None, f"{type(e).__name__}: {e}"
            self.events.add(f"drive failed to start: {self.drive_error}", "error")

    def close(self) -> None:
        if self.drive is not None:
            self.drive.close()
            self.drive = None

    def reset(self, token: Optional[str]) -> None:
        """Rebuild the drive (after a loop error or a failed start). Holder only; ends disabled."""
        self.lease.check(token)
        self.enabled = False
        self.slot.clear()
        if self.drive is not None:
            try:
                self.drive.close()
            except Exception as e:   # the bus may already be gone
                self.events.add(f"closing the old drive: {e}", "warning")
            self.drive = None
        self.start()

    @property
    def status(self) -> str:
        if self.drive is None:
            return "no drive" if self.drive_error else "starting"
        if self.drive.error is not None:
            return "error"
        return "running"

    # -- requests (any thread) ----------------------------------------------------------------

    def acquire(self, client: str):
        token, info = self.lease.acquire(client)
        self.events.add(f"lease {info.lease_id} acquired by {info.client}")
        return token, info

    def release(self, token: Optional[str]) -> None:
        self.lease.release(token)

    def renew(self, token: Optional[str]):
        return self.lease.renew(token)

    def chocks(self) -> list:
        return self.locks.matching("chock", self.lock_names)

    def enable(self, token: Optional[str]) -> None:
        self.lease.renew(token)
        ch = self.chocks()
        if ch:
            raise RequestError(423, f"chocked: {describe_locks(ch)}. An admin removes it on the robot: "
                                    "python -m robot_utils.admin.locks remove <id>")
        if self.estop.active:
            raise RequestError(409, "e-stop is latched; clear it first (POST /estop/clear)")
        if self.status != "running":
            raise RequestError(503, f"drive is not running ({self.status}: {self.drive_error or self._loop_error()})")
        if not self.enabled:
            self.slot.clear()   # never resume an old command
            self.enabled = True
            self.events.add("enabled")

    def disable(self, token: Optional[str]) -> None:
        self.lease.renew(token)
        self._disable("disabled")

    def command(self, token: Optional[str], twist, seq: int = 0, source: str = "slow") -> dict:
        """Write the CommandSlot (both the slow and a future fast path come through here)."""
        self.lease.renew(token)   # checks the token; any accepted command renews the lease
        if self.estop.active:
            raise RequestError(409, "e-stop is latched")
        if not self.enabled:
            raise RequestError(409, "not enabled (POST /enable)")
        if len(twist) != 3 or not all(math.isfinite(float(v)) for v in twist):
            raise ValueError("twist must be three finite numbers [vx, vy, omega]")
        cmd = self.slot.write(twist, seq, source)
        applied, clamped = clamp(cmd.twist, *self.soft)
        return {"seq": cmd.seq, "applied": list(applied), "clamped": clamped,
                "soft_limits": {"max_speed": self.soft[0], "max_omega": self.soft[1]}}

    def set_limits(self, token: Optional[str], max_speed: Optional[float], max_omega: Optional[float]):
        self.lease.renew(token)
        s, w = self.soft
        if max_speed is not None:
            if not 0 <= max_speed <= HARD_MAX_SPEED:
                raise ValueError(f"max_speed must be within 0..{HARD_MAX_SPEED} m/s")
            s = float(max_speed)
        if max_omega is not None:
            if not 0 <= max_omega <= HARD_MAX_OMEGA:
                raise ValueError(f"max_omega must be within 0..{HARD_MAX_OMEGA} rad/s")
            w = float(max_omega)
        self.soft = (s, w)
        self.events.add(f"soft limits {s} m/s, {w} rad/s")
        return self.limits()

    def limits(self) -> dict:
        return {"hard": {"max_speed": HARD_MAX_SPEED, "max_omega": HARD_MAX_OMEGA},
                "soft": {"max_speed": self.soft[0], "max_omega": self.soft[1]}}

    def trigger_estop(self, reason: str, source: str) -> EStopInfo:
        return self.estop.trigger(reason, source)

    def clear_estop(self, token: Optional[str]) -> None:
        self.lease.renew(token)
        if self.estop.active:
            self.estop.clear()
            self.events.add("e-stop cleared (still disabled: POST /enable)")

    # -- internals ----------------------------------------------------------------------------

    def _loop_error(self) -> Optional[str]:
        d = self.drive
        return None if d is None or d.error is None else f"{type(d.error).__name__}: {d.error}"

    def _disable(self, why: str) -> None:
        was = self.enabled
        self.enabled = False
        self.slot.clear()
        if self.drive is not None:
            self.drive.stop()
        if was:
            self.events.add(why)

    def _lease_ended(self, reason: str) -> None:
        self.soft = (HARD_MAX_SPEED, HARD_MAX_OMEGA)
        self._disable(f"lease {reason}: stopped and disabled")
        self.events.add(f"lease {reason}")

    def _estop_triggered(self, info: EStopInfo) -> None:
        self._disable(f"E-STOP ({info.source}: {info.reason})")

    def _pre_tick(self, drive: SwerveDrive) -> None:
        self._applied = None
        ch = self.chocks()                       # cached: a directory stat at most every 0.2 s
        if ch:
            if self.enabled:
                self._disable(f"CHOCKED ({describe_locks(ch)}): stopped and disabled")
            drive.stop()
            self.lease.expire_if_due()
            return
        self.lease.expire_if_due()
        if self.estop.active or not self.enabled:
            drive.stop()
            return
        cmd = self.slot.read()
        if cmd is None or time.monotonic() - cmd.t > COMMAND_TIMEOUT:
            drive.stop()
            return
        applied, _ = clamp(cmd.twist, *self.soft)
        drive.drive(*applied)
        self._applied = (cmd, applied)

    def _on_tick(self, drive: SwerveDrive, t_tick: float) -> None:
        self._tick += 1
        cmd = self.slot.read()
        info = self.lease.info
        nan = math.nan
        row = [t_tick, self._tick, float(drive.tick_enabled), float(self.enabled),
               float(self.estop.active), float(info.lease_id if info else 0),
               float(cmd.seq if cmd else 0), float(SOURCES.get(cmd.source, 0) if cmd else 0),
               (t_tick - cmd.t) if cmd else nan]
        row += list(cmd.twist) if cmd else [nan] * 3
        row += list(self._applied[1]) if self._applied else [nan] * 3
        row += [self.soft[0], self.soft[1], float(drive.error is None)]
        for i in range(len(MODULES)):
            s5 = drive.steers[i].get_periodic_status(PeriodicFrame.STATUS_5)
            s2 = drive.drives[i].get_periodic_status(PeriodicFrame.STATUS_2)
            s0s = drive.steers[i].get_periodic_status(PeriodicFrame.STATUS_0)
            s0d = drive.drives[i].get_periodic_status(PeriodicFrame.STATUS_0)
            sp_angle, sp_speed = drive.setpoints[i]
            row += [
                drive.wheel_angle(s5["DUTY_CYCLE_ENCODER_POSITION"], i) if s5.timestamp else nan,
                s5["DUTY_CYCLE_ENCODER_VELOCITY"] * 2 * math.pi / 60 if s5.timestamp else nan,
                s5.timestamp or nan,
                s2["PRIMARY_ENCODER_VELOCITY"] if s2.timestamp else nan,
                s2["PRIMARY_ENCODER_POSITION"] if s2.timestamp else nan,
                s2.timestamp or nan,
                sp_angle, sp_speed,
                s0s["APPLIED_OUTPUT"] if s0s.timestamp else nan,
                s0s["CURRENT"] if s0s.timestamp else nan,
                s0s.timestamp or nan,
                s0d["APPLIED_OUTPUT"] if s0d.timestamp else nan,
                s0d["CURRENT"] if s0d.timestamp else nan,
                s0d.timestamp or nan,
            ]
        self.hub.publish_rows(np.array([row]), "tick")

    # -- state for /state and /events -----------------------------------------------------------

    def state(self) -> dict:
        info, e = self.lease.info, self.estop.info
        cmd = self.slot.read()
        now = time.monotonic()
        last = self.hub.recent(1, "tick")
        # NaN (no value) -> null: JSON has no NaN
        latest = ({k: (v if math.isfinite(v) else None) for k, v in zip(FIELDS, last[0].tolist())}
                  if len(last) else None)
        recent = self.hub.recent(101, "tick")
        rate = (len(recent) - 1) / (recent[-1, 0] - recent[0, 0]) if len(recent) > 10 and recent[-1, 0] > recent[0, 0] else None
        return {
            "state": self.status,      # starting | running | error | no drive
            "error": self.drive_error or self._loop_error(),
            "interface": self.interface,
            "enabled": self.enabled,
            "chocked": bool(ch := self.chocks()),
            "locks": [l.public() for l in ch],
            "estop": {"latched": e is not None, "reason": e and e.reason, "source": e and e.source,
                      "t": e and e.t},
            "lease": {"held": info is not None, "holder": info and info.client,
                      "lease_id": info and info.lease_id,
                      "expires_in": None if info is None else round(info.expires - now, 3)},
            "limits": self.limits(),
            "command": None if cmd is None else {"twist": list(cmd.twist), "seq": cmd.seq,
                                                 "source": cmd.source, "age": round(now - cmd.t, 3)},
            "command_timeout": COMMAND_TIMEOUT,
            "lease_heartbeat": LEASE_HEARTBEAT,
            "loop_hz": None if rate is None else round(rate, 1),
            "clients": self.hub.subscribers,
            "events": self.events.list()[-20:],
            "latest": latest,
            "server_mono": now,
        }
