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
               While a drive-encoder rebase is in flight, a fresh command is held at zero.
               Then a pending rebase request sends SET_PRIMARY_ENCODER_POSITION 0 to the four
               drive SPARKs (batched with the tick's other frames).
``_on_tick``   tracks whether the base is stationary, confirms an in-flight rebase from fresh
               STATUS_2 frames (``enc_epoch`` increments only then), starts an automatic rebase
               when due, and builds the tick's telemetry row (FIELDS) and publishes it.

Encoder rebase (docs/swerve_node.md): the drive SPARKs' ``drive_pos`` is float32 on the wire, so
its resolution degrades with accumulated travel; setting the four drive encoders back to 0 now
and then (only while stationary) keeps it fine. The drives run velocity closed loop, so this
does not affect control; the steering (absolute encoder) is never touched.

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
    ("enc_epoch", "count"), ("enc_rebasing", "bool"), ("rebase_age", "s"),
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
    "enc_epoch": "drive-encoder rebases confirmed since the node started: drive_pos restarts near 0 when it changes",
    "enc_rebasing": "1 from sending a rebase until it is confirmed or fails: drive_pos may jump, don't integrate it",
    "rebase_age": "seconds since the last confirmed rebase (NaN if none yet)",
}

# Drive-encoder rebase (see the module docstring).
STILL_VEL = 0.01          # m/s: every |drive_vel| below this (and no non-zero command) = stationary
REBASE_STILL_S = 0.5      # s stationary before a manual rebase is allowed
REBASE_TOL = 1e-3         # m: |drive_pos| at most this after the rebase confirms it
REBASE_SETTLE = 0.015     # s: a confirming STATUS_2 must arrive this long after the command went out
REBASE_TIMEOUT = 0.5      # s from sending to confirmation, else the rebase failed
AUTO_INTERVAL_S = 600.0   # default: rebase automatically when the last one is older than this ...
AUTO_STILL_S = 1.0        # ... and the base has been stationary this long
AUTO_RETRY_S = 10.0       # s after a failed automatic rebase before the next attempt
DRIVE_MAX_AGE = 0.1       # s: an older STATUS_2 is not a reading


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
        # drive-encoder rebase: written on the control thread, read by requests, under _enc
        self._enc = threading.Condition()
        self.enc_epoch = 0
        self.last_rebase_t: Optional[float] = None
        self.last_rebase_error: Optional[str] = None
        self.auto_rebase = {"enabled": True, "interval_s": AUTO_INTERVAL_S, "still_s": AUTO_STILL_S}
        self._rebase: Optional[dict] = None      # the request in progress
        self._last_auto_try = -math.inf
        self._still_since: Optional[float] = None
        self._motion: Optional[str] = "no drive readings yet"   # why the base is not stationary
        self._wants_motion = False               # this tick's command is non-zero (even if held)
        self._max_abs_pos: Optional[float] = None

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
        self._gate(drive)
        self._send_rebase(drive)

    def _gate(self, drive: SwerveDrive) -> None:
        self._applied = None
        self._wants_motion = False
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
        self._wants_motion = any(v != 0.0 for v in applied)
        if self._rebase is not None:             # rebase in flight (tens of ms): hold still
            applied = (0.0, 0.0, 0.0)
        drive.drive(*applied)
        self._applied = (cmd, applied)

    # -- drive-encoder rebase ---------------------------------------------------------------------

    def rebase(self, source: str = "manual", timeout: float = REBASE_TIMEOUT + 0.5) -> dict:
        """Set the four drive encoders to 0 and wait for the readings to confirm it (blocking:
        call from a worker thread). Anyone may ask: it moves nothing. Raises RequestError 409
        when the base is not stationary (the reason says why), 503 when the drive is not running
        or the readings did not confirm in time."""
        if self.status != "running":
            raise RequestError(503, f"drive is not running ({self.status})")
        with self._enc:
            if self._rebase is None:
                why = self._not_still(REBASE_STILL_S, time.monotonic())
                if why:
                    raise RequestError(409, f"not stationary: {why}")
                self._rebase = {"source": source, "t_req": time.monotonic(), "t_sent": None,
                                "t_out": None, "result": None}
            req = self._rebase
            if not self._enc.wait_for(lambda: req["result"] is not None, timeout):
                raise RequestError(503, "rebase not confirmed in time (is the control loop running?)")
        ok, msg = req["result"]
        if not ok:
            raise RequestError(409 if "not stationary" in msg else 503, msg)
        return self.encoders()

    def set_auto_rebase(self, enabled: Optional[bool] = None, interval_s: Optional[float] = None,
                        still_s: Optional[float] = None) -> dict:
        if interval_s is not None and not interval_s >= 1.0:
            raise ValueError("interval_s must be at least 1 s")
        if still_s is not None and not REBASE_STILL_S <= still_s <= 60.0:
            raise ValueError(f"still_s must be within {REBASE_STILL_S}..60 s")
        with self._enc:
            for k, v in (("enabled", enabled), ("interval_s", interval_s), ("still_s", still_s)):
                if v is not None:
                    self.auto_rebase[k] = type(self.auto_rebase[k])(v)
            out = dict(self.auto_rebase)
        self.events.add(f"auto rebase: {'on' if out['enabled'] else 'off'}, every {out['interval_s']:g} s "
                        f"when stationary for {out['still_s']:g} s")
        return out

    def encoders(self) -> dict:
        now = time.monotonic()
        with self._enc:
            req = self._rebase
            return {"epoch": self.enc_epoch, "last_rebase_t": self.last_rebase_t,
                    "last_rebase_age": None if self.last_rebase_t is None else round(now - self.last_rebase_t, 3),
                    "rebasing": req is not None and req["t_sent"] is not None,
                    "last_error": self.last_rebase_error,
                    "auto": dict(self.auto_rebase),
                    "max_abs_drive_pos": self._max_abs_pos,
                    "stationary_s": None if self._still_since is None else round(max(0.0, now - self._still_since), 3),
                    "motion": self._motion}

    def _not_still(self, dwell: float, now: float) -> Optional[str]:
        """None if stationary for at least ``dwell`` s, else why not (call under _enc)."""
        if self._still_since is None:
            return self._motion or "moving"
        if now - self._still_since < dwell:
            return f"stationary for {now - self._still_since:.2f} s of the {dwell:g} s needed"
        return None

    def _finish_rebase(self, ok: bool, msg: str, now: float) -> None:
        """End the request in progress (under _enc)."""
        req, self._rebase = self._rebase, None
        req["result"] = (ok, msg)
        if ok:
            self.enc_epoch += 1
            self.last_rebase_t = now
            self.last_rebase_error = None
        else:
            self.last_rebase_error = msg
            if req["source"] == "auto":
                self._last_auto_try = now
        self._enc.notify_all()
        self.events.add(msg, "info" if ok else "warn")

    def _send_rebase(self, drive: SwerveDrive) -> None:
        """Control thread, end of pre-tick: send a pending rebase (batched with the tick's frames)."""
        with self._enc:
            req = self._rebase
            if req is None or req["t_sent"] is not None:
                return
            if self._still_since is None or self._wants_motion:
                self._finish_rebase(False, "rebase refused: not stationary "
                                           f"({self._motion or 'a non-zero command arrived'})", time.monotonic())
                return
            for d in drive.drives:
                d.get_encoder().set_position(0.0)
            req["t_sent"] = time.monotonic()

    def _encoder_tick(self, drive: SwerveDrive, now: float) -> None:
        """Control thread, on_tick: stationarity, rebase confirmation, automatic rebase."""
        vel, pos, fresh = [], [], True
        for d in drive.drives:
            s2 = d.get_periodic_status(PeriodicFrame.STATUS_2)
            if s2.timestamp is None or now - s2.timestamp > DRIVE_MAX_AGE:
                fresh = False
                continue
            vel.append(abs(s2["PRIMARY_ENCODER_VELOCITY"]))
            pos.append((s2.timestamp, s2["PRIMARY_ENCODER_POSITION"]))
        with self._enc:
            self._max_abs_pos = max(abs(p) for _, p in pos) if pos else None
            if not fresh:
                motion = "no fresh drive readings"
            elif self._wants_motion:
                motion = "a non-zero command is active"
            elif max(vel) >= STILL_VEL:
                motion = f"wheels moving (|drive_vel| up to {max(vel):.3f} m/s)"
            else:
                motion = None
            self._motion = motion
            if motion is not None:
                self._still_since = None
            elif self._still_since is None:
                self._still_since = now
            req = self._rebase
            if req is not None and req["t_sent"] is not None:
                if req["t_out"] is None:
                    req["t_out"] = now               # the tick's frames were flushed before on_tick
                elif (fresh and all(t > req["t_out"] + REBASE_SETTLE and abs(p) <= REBASE_TOL for t, p in pos)):
                    self._finish_rebase(True, f"drive encoders rebased to 0 ({req['source']}; epoch "
                                              f"{self.enc_epoch + 1}, {(now - req['t_out']) * 1e3:.0f} ms)", now)
                if self._rebase is req and now - req["t_out"] > REBASE_TIMEOUT:
                    self._finish_rebase(False, f"rebase failed: drive_pos not confirmed within {REBASE_TIMEOUT} s "
                                               f"(max |drive_pos| {self._max_abs_pos})", now)
            a = self.auto_rebase
            if (self._rebase is None and a["enabled"] and now - self._last_auto_try >= AUTO_RETRY_S
                    and (self.last_rebase_t is None or now - self.last_rebase_t >= a["interval_s"])
                    and self._not_still(a["still_s"], now) is None):
                self._rebase = {"source": "auto", "t_req": now, "t_sent": None, "t_out": None, "result": None}

    def _on_tick(self, drive: SwerveDrive, t_tick: float) -> None:
        self._tick += 1
        self._encoder_tick(drive, time.monotonic())
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
        with self._enc:
            rebasing = self._rebase is not None and self._rebase["t_sent"] is not None
            row += [float(self.enc_epoch), float(rebasing),
                    (t_tick - self.last_rebase_t) if self.last_rebase_t is not None else nan]
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
            "encoders": self.encoders(),
            "latest": latest,
            "server_mono": now,
        }
