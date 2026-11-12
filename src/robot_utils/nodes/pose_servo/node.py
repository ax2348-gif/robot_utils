"""Pose servo node core: estimator + swerve streams in, swerve twists out (no web code).

Threads:

- two ``Upstream`` readers (estimator.node): the estimator's pose stream and the swerve's
  telemetry stream, on this machine's clock (NTP-style offset; ~0 on the Pi);
- ``in``: consumes them, keeps the latest pose and the estimator's health (stale stream,
  ``reset_id`` change, pose jumps, missing wheel odometry or IMU heading) and the swerve's
  enable / e-stop / lease state;
- ``control``: every ``PERIOD`` (50 Hz) checks health, runs ``planner.Servo.update`` on the
  predicted pose and POSTs the twist to the swerve node (renews the lease when idle).

Enable = acquire the swerve lease + set its soft limits to the caps + swerve ``/enable``.
Disable = swerve ``/disable`` + release: immediate, no braking. Requests (server.py) only
touch state under ``self.lock``; swerve HTTP calls are serialised by ``self.io``.
"""
from __future__ import annotations

import logging
import math
import queue
import threading
import time
from dataclasses import asdict, dataclass, fields, replace
from typing import Optional

import numpy as np

from ..common.hub import RowHub
from ..common.lease import RequestError
from ..common.server import EventLog
from ..estimator.node import Upstream
from ..swerve.client import SwerveClient
from ..swerve.constants import MODULES
from ..swerve.node import HARD_MAX_OMEGA, HARD_MAX_SPEED
from .planner import (STATUS, STEER_MIN_SPEED, STEER_RATE_MAX, WHEEL_R, WHEEL_SPEED_MAX, Caps, Cmd, Goal, Motion,
                      Servo, check_step)

log = logging.getLogger("pose_servo")

PERIOD = 0.02            # s, control period (50 Hz)
EST_STALE_S = 0.20       # estimator stream older than this -> motion aborted
IMU_LOST_S = 0.20        # require_imu: no IMU-fused pose for this long -> aborted
JUMP_POS = 0.15          # m between consecutive poses -> "pose jump" -> aborted
JUMP_THETA = 0.30        # rad
MAX_PREDICT_S = 0.10     # pose extrapolation (estimator latency) is capped at this
CMD_FAIL_S = 0.20        # swerve commands failing for this long -> disabled
RENEW_S = 0.30           # lease renew interval while idle
ALIGN_TOL = math.radians(3)   # every wheel within this of its steer-only setpoint = aligned
SWERVE_GRACE_S = 0.25    # after enable, swerve telemetry this old may still show the old state

FIELDS = ["t", "status", "goal_id", "x", "y", "theta", "goal_x", "goal_y", "goal_theta",
          "err_pos", "err_theta", "vx", "vy", "omega", "ax", "ay", "alpha",
          "vdes_x", "vdes_y", "wdes", "scale_lin", "scale_rot", "ttg_lin", "ttg_rot",
          "lin_done", "rot_done", "kept", "boost", "wheel_max", "steer_rate", "est_age", "sent", "violations"]
UNITS = ["s", "enum", "id", "m", "m", "rad", "m", "m", "rad",
         "m", "rad", "m/s", "m/s", "rad/s", "m/s^2", "m/s^2", "rad/s^2",
         "m/s", "m/s", "rad/s", "", "", "s", "s",
         "bool", "bool", "", "m/s", "m/s", "rad/s", "s", "bool", "count"]
FIELD_DOC = {
    "t": "control tick time (this host's monotonic clock; the Pi's when run there)",
    "status": "0 disabled, 1 idle, 2 moving, 3 holding, 4 braking",
    "x": "pose used this tick: the estimator's latest, extrapolated to t with the commanded twist (NaN if none)",
    "goal_x": "goal in the estimator's world frame (NaN without a goal); goal_theta continuous",
    "err_pos": "distance to the goal (m); err_theta = goal_theta - theta (rad)",
    "vx": "commanded body twist sent this tick (vx, vy, omega; NaN if nothing sent)",
    "ax": "its rate: (twist - previous twist) / period",
    "vdes_x": "planner's desired body twist before jerk-limited tracking",
    "scale_lin": "synchronized mode time scale of each axis (1 = full limits)",
    "ttg_lin": "predicted time to the acceptance region per axis",
    "lin_done": "axis latched inside its acceptance region",
    "kept": "share of the planned step the limiter kept (1 = no intervention)",
    "boost": "static-friction wheel-speed floor in force (rises while the base does not move)",
    "wheel_max": "fastest wheel of the sent twist; steer_rate = fastest wheel steering of this step",
    "est_age": "age of the estimator pose used (s)",
    "sent": "1 if the twist was accepted by the swerve node",
    "violations": "steps that broke a constraint (expected 0 always)",
}


@dataclass(frozen=True)
class Config:
    """Per enable (reset to these defaults on disable)."""
    # safety caps: every twist sent stays within these
    max_speed: float = 0.3
    max_omega: float = 0.8
    max_accel: float = 0.5
    max_alpha: float = 1.5
    max_jerk: float = 2.0
    max_ang_jerk: float = 6.0
    max_distance: float = 3.0         # m, goals farther than this are rejected
    max_rotation: float = math.pi     # rad, goals turning more than this are rejected
    require_imu: bool = True          # abort when the estimator's heading is not IMU-fused
    # motion defaults (overridable per goal)
    mode: str = "synchronized"
    pos_tol: float = 0.02
    heading_tol: float = 0.035
    on_reach: str = "terminate"
    hold_band: float = 2.0
    min_wheel_speed: float = 0.08     # m/s static-friction boost limit (0 = off)
    stall_s: float = 3.0              # s without progress -> goal "stalled" (0 = never)

    @property
    def caps(self) -> Caps:
        return Caps(self.max_speed, self.max_omega, self.max_accel, self.max_alpha, self.max_jerk,
                    self.max_ang_jerk)


SAFETY_KEYS = ("max_speed", "max_omega", "max_accel", "max_alpha", "max_jerk", "max_ang_jerk",
               "max_distance", "max_rotation", "require_imu")
MOTION_KEYS = ("mode", "pos_tol", "heading_tol", "on_reach", "hold_band", "min_wheel_speed", "stall_s")
CAP_BOUNDS = {"max_speed": HARD_MAX_SPEED, "max_omega": HARD_MAX_OMEGA, "max_accel": 10.0,
              "max_alpha": 40.0, "max_jerk": 100.0, "max_ang_jerk": 400.0,
              "max_distance": 20.0, "max_rotation": 20 * math.pi}
GOAL_LIMIT_KEYS = {"speed": "max_speed", "omega": "max_omega", "accel": "max_accel",
                   "alpha": "max_alpha", "jerk": "max_jerk", "ang_jerk": "max_ang_jerk"}


def validate_motion(mode, pos_tol, heading_tol, on_reach, hold_band, min_wheel_speed=0.0, stall_s=0.0) -> None:
    if not (0 <= min_wheel_speed <= 0.3):
        raise ValueError("min_wheel_speed must be within [0, 0.3] m/s")
    if not (0 <= stall_s <= 60):
        raise ValueError("stall_s must be within [0, 60] s (0 = never)")
    if mode not in ("synchronized", "independent"):
        raise ValueError("mode must be 'synchronized' or 'independent'")
    if on_reach not in ("terminate", "hold"):
        raise ValueError("on_reach must be 'terminate' or 'hold'")
    if not (0 < pos_tol <= 1.0):
        raise ValueError("pos_tol must be within (0, 1] m")
    if not (0 < heading_tol <= 1.0):
        raise ValueError("heading_tol must be within (0, 1] rad")
    if not (1.0 <= hold_band <= 20.0):
        raise ValueError("hold_band must be within [1, 20]")


def make_config(base: Config, changes: dict) -> Config:
    unknown = set(changes) - {f.name for f in fields(Config)}
    if unknown:
        raise ValueError(f"unknown config keys: {sorted(unknown)}")
    c = replace(base, **changes)
    for k, hi in CAP_BOUNDS.items():
        v = getattr(c, k)
        if not (isinstance(v, (int, float)) and math.isfinite(v) and 0 < v <= hi):
            raise ValueError(f"{k} must be within (0, {hi:g}]")
    if c.max_speed + c.max_omega * WHEEL_R > WHEEL_SPEED_MAX:
        raise ValueError(f"max_speed + max_omega * {WHEEL_R:.3f} m must not exceed the wheel limit "
                         f"{WHEEL_SPEED_MAX} m/s")
    validate_motion(c.mode, c.pos_tol, c.heading_tol, c.on_reach, c.hold_band, c.min_wheel_speed, c.stall_s)
    return replace(c, **{k: float(getattr(c, k)) for k in CAP_BOUNDS}, require_imu=bool(c.require_imu))


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class EstimatorView:
    """The latest estimator pose and its health (updated by the input thread)."""

    def __init__(self):
        self.row: Optional[dict] = None     # latest row (fields by name), t on this host's clock
        self.rx: Optional[float] = None     # arrival time of the latest row
        self.reset_id: Optional[float] = None
        self.jumps = 0                      # pose jumps seen (count)
        self.last_jump: Optional[str] = None
        self.last_imu: Optional[float] = None


class PoseServoNode:
    def __init__(self, swerve: Optional[str] = "swerve", estimator: Optional[str] = "estimator"):
        self.swerve_target, self.estimator_target = swerve, estimator
        self.hub = RowHub({"servo": len(FIELDS)}, recent=500)
        self.events = EventLog(on_add=self.hub.state_changed)
        self.q: queue.Queue = queue.Queue(maxsize=10000)
        self.inputs: dict[str, Upstream] = {}
        if estimator:
            self.inputs["estimator"] = Upstream("estimator", estimator, "/stream", self.q, self.events)
        if swerve:
            self.inputs["swerve"] = Upstream("swerve", swerve, "/stream", self.q, self.events)
        self.lock = threading.RLock()       # node state (servo, config, enabled, views)
        self.io = threading.Lock()          # swerve HTTP calls
        self.stop_ev = threading.Event()
        self.est = EstimatorView()
        self.sw: Optional[dict] = None      # latest swerve telemetry (enabled, estop, lease_id, t, steer_err)
        self.align: Optional[tuple] = None  # steer-only unit twist held while idle (POST /align); None = off
        self.align_t: Optional[float] = None
        self.config = Config()
        self.servo = Servo(self.config.caps)
        self.enabled = False
        self.client: Optional[SwerveClient] = None
        self.lease_id: Optional[int] = None
        self.enabled_at = 0.0
        self.holder: Optional[str] = None
        self.goal_reset_id: Optional[float] = None
        self.goal_count = 0
        self._goal_jumps = 0
        self.last_goal: Optional[dict] = None
        self.disabled_reason: Optional[str] = None
        self._cmd_fail_since: Optional[float] = None
        self._last_renew = 0.0
        self._prev_sent = Cmd()
        self._tick_times: list = []
        self._threads = [threading.Thread(target=self._read, name="in", daemon=True),
                         threading.Thread(target=self._control, name="control", daemon=True)]

    # -- lifecycle ----------------------------------------------------------------------------

    def start(self):
        self.events.add(f"pose servo started (swerve {self.swerve_target}, estimator {self.estimator_target})")
        for u in self.inputs.values():
            u.start()
        for t in self._threads:
            t.start()

    def close(self):
        self.stop_ev.set()
        for u in self.inputs.values():
            u.stop.set()
        if self.enabled:
            self.disable("node shutting down")

    # -- input thread -------------------------------------------------------------------------

    def _read(self):
        while not self.stop_ev.is_set():
            try:
                key, rows, offset = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            now = time.monotonic()
            names = rows.dtype.names
            if key == "estimator":
                for r in rows:
                    self._est_row({n: float(r[n]) for n in names}, offset, now)
            elif key == "swerve" and len(rows):
                r = rows[-1]
                steer_err = None
                if "steer_vx" in names and math.isfinite(float(r["steer_vx"])):   # steer-only in force: how far off
                    errs = [abs(math.remainder(float(r[f"{m.name}_steer_angle"]) - float(r[f"{m.name}_steer_sp"]), 2 * math.pi))
                            for m in MODULES if f"{m.name}_steer_sp" in names]
                    steer_err = max(errs) if errs and all(math.isfinite(e) for e in errs) else None
                with self.lock:
                    self.sw = {"t": float(r["t_tick"]) - offset, "enabled": float(r["enabled"]),
                               "estop": float(r["estop"]), "lease_id": float(r["lease_id"]), "rx": now,
                               "steer_err": steer_err}

    def _est_row(self, row: dict, offset: float, now: float):
        row["t"] -= offset
        e = self.est
        with self.lock:
            p = e.row
            rid = row.get("reset_id", 0.0)
            if p is not None and rid == e.reset_id:
                dpos = math.hypot(row["x"] - p["x"], row["y"] - p["y"])
                dth = abs(row["theta"] - p["theta"])
                if dpos > JUMP_POS or dth > JUMP_THETA:
                    e.jumps += 1
                    e.last_jump = f"pose jumped {dpos:.3f} m / {math.degrees(dth):.1f}°"
            if rid != e.reset_id and e.reset_id is not None:
                self.events.add(f"estimator reset (reset_id {e.reset_id:g} -> {rid:g})", "warn")
            e.reset_id = rid
            if row.get("imu_ok", 0.0) == 1.0:
                e.last_imu = now
            e.row, e.rx = row, now

    def estimator_health(self, now: Optional[float] = None) -> Optional[str]:
        """None if the pose can be used, else why not."""
        now = time.monotonic() if now is None else now
        e = self.est
        if e.row is None:
            return "no estimator pose yet"
        if now - e.rx > EST_STALE_S:
            return f"estimator stream stale ({now - e.rx:.2f} s)"
        if e.row.get("odom_ok", 1.0) != 1.0:
            return "estimator has no wheel odometry"
        if self.config.require_imu and (e.last_imu is None or now - e.last_imu > IMU_LOST_S):
            return "estimator heading is not IMU-fused (require_imu)"
        return None

    def predicted_pose(self, now: float) -> Optional[tuple[float, float, float, float]]:
        """(x, y, theta, age): the latest pose moved forward by the commanded twist."""
        r = self.est.row
        if r is None:
            return None
        age = now - r["t"]
        lag = min(max(age, 0.0), MAX_PREDICT_S)
        c = self.servo.cmd
        th = r["theta"] + c.w * lag
        tm = r["theta"] + c.w * lag / 2
        x = r["x"] + (math.cos(tm) * c.vx - math.sin(tm) * c.vy) * lag
        y = r["y"] + (math.sin(tm) * c.vx + math.cos(tm) * c.vy) * lag
        return x, y, th, age

    # -- control thread -----------------------------------------------------------------------

    def _control(self):
        nxt = time.monotonic()
        while not self.stop_ev.is_set():
            nxt += PERIOD
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()      # overrun: restart the schedule, no burst
            try:
                self._tick()
            except Exception as e:          # never let the loop die; disable to be safe
                log.exception("control tick")
                self.events.add(f"control error: {type(e).__name__}: {e}", "error")
                if self.enabled:
                    self.disable(f"control error: {e}")

    def _tick(self):
        now = time.monotonic()
        nan = math.nan
        with self.lock:
            self._tick_times = (self._tick_times + [now])[-100:]
            if not self.enabled:
                self._publish(now, "disabled", None, None, None, 0.0)
                return
            # swerve state: stopped / lease lost / e-stopped elsewhere -> disable (no braking possible)
            sw = self.sw
            if sw is not None and sw["t"] > self.enabled_at + SWERVE_GRACE_S:
                why = None
                if sw["estop"]:
                    why = "swerve e-stop latched"
                elif sw["lease_id"] != self.lease_id:
                    why = "swerve lease lost"
                elif not sw["enabled"]:
                    why = "swerve disabled"
                if why:
                    self._disable_locked(why)
                    self._publish(now, "disabled", None, None, None, 0.0)
                    return
            servo = self.servo
            health = self.estimator_health(now)
            if servo.goal is not None:
                if health is None and self.est.reset_id != self.goal_reset_id:
                    health = "estimator reset (world frame changed)"
                if health is None and self.est.jumps != self._goal_jumps:
                    health = self.est.last_jump or "pose jump"
                if health is not None:
                    self._end_goal(health, "aborted")
            pp = self.predicted_pose(now) if health is None else None
            pose = pp[:3] if pp else None
            prev = servo.cmd
            cmd, diag = servo.update(pose if servo.goal is not None else None, PERIOD)
            goal = servo.goal
            align = self.align if (cmd is None and goal is None) else None
            self._sync_result()
        sent = 0.0
        if cmd is not None:
            sent = 1.0 if self._send(cmd) else 0.0
        elif align is not None:
            sent = 1.0 if self._send_steer(align) else 0.0
        else:
            self._renew_if_due(now)
        with self.lock:
            chk = check_step(prev, cmd, PERIOD) if cmd is not None else None
            self._publish(now, diag.status, cmd, diag, (goal, pp, chk), sent)

    def _send(self, cmd: Cmd) -> bool:
        try:
            with self.io:
                if self.client is None:
                    return False
                self.client.command(cmd.vx, cmd.vy, cmd.w)
            self._cmd_fail_since = None
            self._last_renew = time.monotonic()
            return True
        except Exception as e:
            now = time.monotonic()
            if self._cmd_fail_since is None:
                self._cmd_fail_since = now
                self.events.add(f"swerve command failed: {e}", "warn")
            if now - self._cmd_fail_since > CMD_FAIL_S:
                self.disable(f"swerve commands failing for {CMD_FAIL_S} s ({e})")
            return False

    def _send_steer(self, twist: tuple) -> bool:
        """Steer-only for the alignment (renews the lease like a twist)."""
        try:
            with self.io:
                if self.client is None:
                    return False
                self.client.steer(*twist)
            self._cmd_fail_since = None
            self._last_renew = time.monotonic()
            return True
        except Exception as e:
            now = time.monotonic()
            if self._cmd_fail_since is None:
                self._cmd_fail_since = now
                self.events.add(f"swerve steer command failed: {e}", "warn")
            if now - self._cmd_fail_since > CMD_FAIL_S:
                self.disable(f"swerve commands failing for {CMD_FAIL_S} s ({e})")
            return False

    # -- defensive alignment: wheels set for a unit twist, drives coasting, while idle ---------------

    def set_align(self, twist) -> dict:
        """Hold every wheel set for the unit twist ``twist`` [vx, vy, omega] (direction only) with the drive motors
        coasting, while no goal runs; until the next goal, stop, clear_align or disable."""
        with self.lock:
            if not self.enabled:
                raise RequestError(409, "not enabled (POST /enable)")
            if self.servo.goal is not None or self.servo.active:
                raise RequestError(409, f"the base is {self.servo.status}: align only when idle")
            t = [float(v) for v in twist]
            norm = math.hypot(*t) if len(t) == 3 and all(math.isfinite(v) for v in t) else 0.0
            if norm < 1e-9:
                raise ValueError("twist must be a direction: three finite numbers [vx, vy, omega], not all zero")
            new = tuple(v / norm for v in t)
            if new != self.align:
                self.align, self.align_t = new, time.monotonic()
                self.events.add("align: wheels set for twist (%.2f, %.2f, %.2f), drives coasting" % new)
            return self.align_state_locked()

    def clear_align(self, reason: str = "cleared") -> dict:
        with self.lock:
            self._clear_align_locked(reason)
            return self.align_state_locked()

    def _clear_align_locked(self, reason: str) -> None:
        if self.align is not None:
            self.events.add(f"align ended: {reason}")
        self.align, self.align_t = None, None

    def align_state_locked(self) -> dict:
        sw = self.sw or {}
        err = sw.get("steer_err") if self.align is not None else None
        return {"twist": list(self.align) if self.align is not None else None,
                "aligned": bool(self.align is not None and err is not None and err < ALIGN_TOL),
                "steer_err_deg": None if err is None else round(math.degrees(err), 2),
                "since_s": None if self.align_t is None else round(time.monotonic() - self.align_t, 2)}

    def _renew_if_due(self, now: float):
        if now - self._last_renew < RENEW_S:
            return
        self._last_renew = now
        try:
            with self.io:
                if self.client is not None:
                    self.client.renew()
        except Exception as e:
            self.disable(f"swerve lease renew failed: {e}")

    def _publish(self, now, status, cmd, diag, extra, sent):
        nan = math.nan
        goal, pp, chk = extra if extra else (None, None, None)
        if pp is None and self.est.row is not None:
            pp = self.predicted_pose(now)
        row = [now, float(STATUS[status]), float(diag.goal_id) if diag else 0.0]
        row += list(pp[:3]) if pp else [nan] * 3
        row += [goal.x, goal.y, goal.theta] if goal else [nan] * 3
        if diag:
            row += [diag.err_pos, diag.err_theta]
        else:
            row += [nan, nan]
        row += [cmd.vx, cmd.vy, cmd.w, cmd.ax, cmd.ay, cmd.al] if cmd else [nan] * 6
        if diag:
            row += [diag.vdes_x, diag.vdes_y, diag.wdes, diag.scale_lin, diag.scale_rot,
                    diag.ttg_lin, diag.ttg_rot, diag.lin_done, diag.rot_done, diag.kept, diag.boost]
        else:
            row += [nan] * 11
        row += [chk.wheel, chk.steer] if chk else [nan, nan]
        row += [pp[3] if pp else nan, sent, float(self.servo.violations)]
        self.hub.publish_rows(np.array([row], dtype=np.float64), "servo")

    def _end_goal(self, reason: str, result: str):
        if self.servo.goal is not None:
            self.events.add(f"goal {self.servo.goal.goal_id} {result}: {reason}", "warn" if result == "aborted" else "info")
        self.servo.stop(reason, result)

    def _sync_result(self, **extra):
        """Copy how the current goal ended (servo.result) into last_goal, once."""
        res, g = self.servo.result, self.last_goal
        if res is None or g is None or g.get("result") is not None or g.get("goal_id") != res["goal_id"]:
            return
        g.update(res, **extra)
        if res["result"] == "reached":
            self.events.add(f"goal {res['goal_id']} reached")
        elif res["result"] == "stalled":
            self.events.add(f"goal {res['goal_id']} stalled: {res['reason']}", "warn")

    # -- requests -----------------------------------------------------------------------------

    def enable(self, client: str = "", config: Optional[dict] = None) -> dict:
        with self.lock:
            if self.enabled:
                raise RequestError(409, f"already enabled (by {self.holder!r}); disable first")
            cfg = make_config(Config(), config or {})
        if not self.swerve_target:
            raise RequestError(503, "no swerve node configured")
        name = f"pose_servo ({client})" if client else "pose_servo"
        c = SwerveClient(self.swerve_target, timeout=0.25)
        try:
            out = c.acquire(name)
        except Exception as e:
            c.close()
            raise RequestError(getattr(e, "status", 503), f"swerve lease: {getattr(e, 'detail', e)}")
        try:
            c.set_limits(cfg.max_speed, cfg.max_omega)
            c.enable()
        except Exception as e:
            try:
                c.release()
            finally:
                c.close()
            raise RequestError(getattr(e, "status", 503), f"swerve enable: {getattr(e, 'detail', e)}")
        with self.lock:
            self.client, self.lease_id = c, out["lease_id"]
            self.config, self.holder = cfg, client or "anonymous"
            self.servo = Servo(cfg.caps)
            self.enabled, self.enabled_at, self.disabled_reason = True, time.monotonic(), None
            self._last_renew = time.monotonic()
            self._cmd_fail_since = None
            self.events.add(f"enabled by {self.holder} (swerve lease {self.lease_id})")
        return self.get_config()

    def disable(self, reason: str = "requested") -> dict:
        """Immediate: no braking. The swerve stops (its /disable), the lease is released."""
        with self.lock:
            was = self.enabled
            self._disable_locked(reason)
        return {"enabled": False, "was_enabled": was}

    def _disable_locked(self, reason: str):
        if not self.enabled:
            return
        if self.servo.goal is not None:
            self.events.add(f"goal {self.servo.goal.goal_id} aborted: disabled ({reason})", "warn")
        self.servo.reset()
        self._sync_result(reason=f"disabled: {reason}")
        self._clear_align_locked(f"disabled ({reason})")
        self.enabled = False
        self.disabled_reason = reason
        c, self.client = self.client, None
        self.config = Config()
        self.events.add(f"disabled: {reason}")
        if c is not None:
            def release():
                with self.io:
                    for call in (c.disable, c.release):
                        try:
                            call()
                        except Exception:
                            pass
                    c.close()
            threading.Thread(target=release, daemon=True).start()

    def get_config(self) -> dict:
        with self.lock:
            return {"enabled": self.enabled, "config": asdict(self.config), "hard": self.hard_limits()}

    @staticmethod
    def hard_limits() -> dict:
        return {"steer_rate": STEER_RATE_MAX, "wheel_speed": WHEEL_SPEED_MAX,
                "steer_min_speed": STEER_MIN_SPEED, "wheel_radius_from_centre": WHEEL_R,
                "swerve_max_speed": HARD_MAX_SPEED, "swerve_max_omega": HARD_MAX_OMEGA,
                "period": PERIOD}

    def set_config(self, changes: dict) -> dict:
        with self.lock:
            if not self.enabled:
                raise RequestError(409, "not enabled: the config is per enable (POST /enable with a config)")
            if self.servo.active:
                raise RequestError(409, f"the base is {self.servo.status}: change the config when idle")
            cfg = make_config(self.config, changes)
        if cfg.max_speed != self.config.max_speed or cfg.max_omega != self.config.max_omega:
            try:
                with self.io:
                    self.client.set_limits(cfg.max_speed, cfg.max_omega)
            except Exception as e:
                raise RequestError(503, f"swerve limits: {e}")
        with self.lock:
            self.config = cfg
            self.servo.caps = cfg.caps
            self.events.add("config: " + ", ".join(f"{k}={v}" for k, v in changes.items()))
        return self.get_config()

    def set_goal(self, frame: str, x: float, y: float, theta: float, overrides: dict) -> dict:
        with self.lock:
            if not self.enabled:
                raise RequestError(409, "not enabled (POST /enable)")
            cfg = self.config
            health = self.estimator_health()
            if health is not None:
                raise RequestError(503, f"cannot servo: {health}")
            # motion: config defaults, per-goal overrides (limits must be within the caps)
            kw = {k: getattr(cfg, k) for k in MOTION_KEYS}
            lim = {k: getattr(cfg, cap) for k, cap in GOAL_LIMIT_KEYS.items()}
            for k, v in overrides.items():
                if v is None:
                    continue
                if k in GOAL_LIMIT_KEYS:
                    cap = getattr(cfg, GOAL_LIMIT_KEYS[k])
                    if not (0 < v <= cap):
                        raise ValueError(f"{k} must be within (0, {cap:g}] (the {GOAL_LIMIT_KEYS[k]} cap)")
                    lim[k] = float(v)
                elif k in MOTION_KEYS:
                    kw[k] = v
                else:
                    raise ValueError(f"unknown goal option {k!r}")
            validate_motion(**kw)
            motion = Motion(**lim, **kw)
            now = time.monotonic()
            px, py, pth, _ = self.predicted_pose(now)
            if frame == "local":
                c, s = math.cos(pth), math.sin(pth)
                gx, gy, gth = px + c * x - s * y, py + s * x + c * y, pth + theta
            elif frame == "global":
                gx, gy, gth = x, y, pth + wrap(theta - pth)
            else:
                raise ValueError("frame must be 'local' or 'global'")
            dist, rot = math.hypot(gx - px, gy - py), abs(gth - pth)
            if dist > cfg.max_distance:
                raise ValueError(f"goal is {dist:.3f} m away; max_distance is {cfg.max_distance:g} m")
            if rot > cfg.max_rotation:
                raise ValueError(f"goal turns {rot:.3f} rad; max_rotation is {cfg.max_rotation:.3f} rad")
            self.goal_count += 1
            g = Goal(gx, gy, gth, motion, self.goal_count)
            moving = self.servo.active
            self._clear_align_locked(f"goal {g.goal_id}")
            self.servo.set_goal(g)
            self.goal_reset_id, self._goal_jumps = self.est.reset_id, self.est.jumps
            self.last_goal = {"goal_id": g.goal_id, "frame": frame, "request": {"x": x, "y": y, "theta": theta},
                              "x": gx, "y": gy, "theta": gth, "motion": asdict(motion),
                              "from": {"x": px, "y": py, "theta": pth}, "distance": dist, "rotation": gth - pth,
                              "result": None, "reason": None, "t": now}
            self.events.add(f"goal {g.goal_id} ({frame}): ({gx:.3f}, {gy:.3f}, {math.degrees(gth):.1f}°), "
                            f"{dist:.3f} m / {math.degrees(gth - pth):.1f}°" + (" (replacing the motion in progress)" if moving else ""))
            return dict(self.last_goal)

    def stop(self) -> dict:
        with self.lock:
            if not self.enabled:
                raise RequestError(409, "not enabled")
            self._end_goal("stop requested", "stopped")
            self._clear_align_locked("stop")
            self._sync_result()
            return {"status": self.servo.status}

    def estop(self, reason: str, source: str) -> dict:
        """Forwarded to the swerve (anyone; it latches and stops at once), then disable."""
        out = {"forwarded": False}
        if self.swerve_target:
            try:
                with SwerveClient(self.swerve_target, timeout=0.5) as c:
                    c.estop(f"pose_servo: {reason}" if reason else "pose_servo", source or "pose_servo")
                out["forwarded"] = True
            except Exception as e:
                out["error"] = str(e)
                self.events.add(f"e-stop forward failed: {e}", "error")
        self.disable(f"e-stop ({reason or 'no reason'})")
        return out

    # -- state --------------------------------------------------------------------------------

    def state(self) -> dict:
        now = time.monotonic()
        with self.lock:
            last = self.hub.recent(1, "servo")
            latest = ({k: (v if math.isfinite(v) else None) for k, v in zip(FIELDS, last[0].tolist())}
                      if len(last) else None)
            ts = self._tick_times
            rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 10 and ts[-1] > ts[0] else None
            e = self.est
            from ..common.locks import LockStore, lock_state
            if not hasattr(self, "_locks"):
                self._locks = LockStore()
            return {
                **lock_state(self._locks, chock={"swerve", "pose_servo"}),
                "status": "disabled" if not self.enabled else self.servo.status,
                "enabled": self.enabled,
                "holder": self.holder if self.enabled else None,
                "align": self.align_state_locked(),
                "disabled_reason": self.disabled_reason,
                "swerve_lease_id": self.lease_id if self.enabled else None,
                "goal": self.last_goal,
                "config": asdict(self.config),
                "estimator": {"health": self.estimator_health(now) or "ok",
                              "reset_id": e.reset_id, "pose_jumps": e.jumps,
                              "age": None if e.row is None else round(now - e.row["t"], 4)},
                "swerve": self.sw and {k: v for k, v in self.sw.items() if k != "rx"},
                "inputs": {k: u.status() for k, u in self.inputs.items()},
                "violations": self.servo.violations,
                "loop_hz": None if rate is None else round(rate, 1),
                "latest": latest,
                "events": self.events.list()[-30:],
            }
