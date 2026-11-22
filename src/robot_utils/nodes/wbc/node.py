"""The WBC node's core (docs/wbc.md): EE targets in, arm joint commands and base twists out. No web code.

Threads:

- the **control loop** (100 Hz; or ``step()`` by hand on an injected clock, the arm node's Rig pattern): drains
  the estimator / swerve inputs, the arm joint channel's state and the target inbox, checks every input and both
  buses, runs the controller, sends the arm command (joint channel) and the base twist (swerve sender thread),
  publishes one row;
- ``Upstream`` readers of the estimator's and the swerve's streams (into a queue the tick drains);
- the datagram ``Receiver`` (UDP + Unix), which only checks session / seq and queues;
- the swerve sender (links.SwerveBase).

Requests (server.py) only touch state under ``self.lock``; enable / disable talk to the peers outside the tick.

States: ``disabled`` | ``enabling`` | ``idle`` (enabled, no target: holding; always while disarmed) | ``tracking``
| ``holding`` (stale input, client lease gone, estimator unusable) | ``fault`` (latched until ``POST /disable``: an
infeasible solve, a cross-bus fault, a lost peer; the healthy half holds: the arm keeps its commanded joints, the base
gets zero).

**Armed latch** (inside enabled; false at enable): only an armed WBC follows targets. Disarmed = hold: the target
buffers and the per-datagram base override are cleared (never resumed), the base brakes to zero within its rate limit,
the arm stops at its commanded joints, the gripper keeps its last goal; targets in datagrams are ignored (counted, not
buffered). Disarmed by a datagram with ``armed: false`` (or without ``armed`` / ``arm_seq``), ``POST /disarm``, stale
input (``hold_timeout``), the client lease ending, a new session, enable, disable, an estimator reset or pose jump, a
planned motion pausing the arm, any fault, the e-stop and a control tick later than 3 dt. Armed only by a ``target``
or ``chunk`` datagram (never ``alive``) carrying ``armed: true`` and an ``arm_seq`` greater than the highest
``arm_seq`` seen when it was last disarmed (the floor; kept for the node's life, ``POST /session`` returns it), while
enabled, without a fault, e-stop or pause.
"""
from __future__ import annotations

import logging
import math
import queue
import secrets
import threading
import time
from dataclasses import asdict
from typing import Callable, Optional

import numpy as np
import pinocchio as pin

from ..common.estop import EStop, EStopInfo
from ..common.hub import RowHub
from ..common.lease import Lease, RequestError
from ..common.server import EventLog
from ..common import realtime
from .arm_channel import ARM_STATES, ArmChannel, ChannelError, joint_socket_path
from .controller import LIMITED, EEState, QPController, Snapshot
from .model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from .receiver import Inbox, Receiver
from .spec import MODES, Config, Hints, hints_public, make_config, merge, parse_base, parse_hints
from .targets import R_to_quat, TargetBuffer, quat_to_R

log = logging.getLogger("wbc")

STATES = ["disabled", "enabling", "idle", "tracking", "holding", "fault"]
SOLVE = ["none", "ok", "infeasible", "error"]
ARM_GRACE_S = 0.5        # after enable: the arm may still report its old state
SWERVE_GRACE_S = 0.25
CMD_FAIL_S = 0.25        # swerve commands failing this long -> fault
POSTURES = {"home": np.zeros(6), "ready": np.radians([0.0, -40.0, -60.0, 0.0, 30.0, 0.0])}


def stream_fields(ees) -> tuple[list, list, dict]:
    """The stream's columns (the contract's first, append-only after them), units and docs."""
    f = [("t_tick", "s"), ("seq", "count"), ("state", "enum"), ("base_mode", "enum"), ("solve_ms", "ms"),
         ("status", "enum"), ("base_x", "m"), ("base_y", "m"), ("base_yaw", "rad"), ("est_reset", "id"),
         ("base_vx", "m/s"), ("base_vy", "m/s"), ("base_w", "rad/s")]
    for e in ees:
        f += [(f"{e}_cmd_{k}", "m" if k in "xyz" else "") for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        f += [(f"{e}_meas_{k}", "m" if k in "xyz" else "") for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        f += [(f"{e}_twist_{k}", "m/s" if k[0] == "v" else "rad/s") for k in ("vx", "vy", "vz", "wx", "wy", "wz")]
        f += [(f"{e}_gripper", "")]
    f += [("arm_age", "s"), ("est_age", "s"), ("target_age", "s")]
    # appended (not in the frozen contract's list; append-only from here)
    f += [("limited", "bool"), ("limited_by", "enum"), ("input_age", "s"), ("hold", "bool"), ("deadband", "bool"),
          ("solves", "count"), ("wheel_max", "m/s")]
    for e in ees:
        f += [(f"{e}_tgt_{k}", "m" if k in "xyz" else "") for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        f += [(f"{e}_err_pos", "m"), (f"{e}_err_rot", "rad"), (f"{e}_ff_scale", ""), (f"{e}_target_id", "id"),
              (f"{e}_manip", ""), (f"{e}_reach", "")]
        f += [(f"{e}_q_cmd{i}", "rad") for i in range(1, 7)] + [(f"{e}_qd{i}", "rad/s") for i in range(1, 7)]
    f += [("armed", "bool"), ("tick_period", "ms"), ("tick_compute", "ms"), ("overruns", "count"),
          ("late_ticks", "count")]
    doc = {
        "t_tick": "control tick time, Pi monotonic s (the clock of target t)",
        "seq": "seq of the latest accepted datagram in force (0 = none)",
        "state": "0 disabled, 1 enabling, 2 idle (no target: holding), 3 tracking, 4 holding (stale input / no client "
                 "lease / estimator unusable), 5 fault",
        "base_mode": "0 free, 1 locked, 2 hint, 3 commanded (hints in force, per-command overrides applied)",
        "solve_ms": "controller time this tick (ms)",
        "status": "solve status: 0 none, 1 ok, 2 infeasible, 3 error",
        "base_x": "the base pose used this tick (world = the estimator's frame; its latest pose extrapolated to "
                  "t_tick with the commanded twist, at most max_predict); base_y, base_yaw likewise",
        "est_reset": "the estimator's reset_id of that pose (world targets are aborted when it changes)",
        "base_vx": "commanded base twist in the ROBOT BASE frame (body twist: forward, left, yaw rate), as the "
                   "swerve command and the velocity hint; base_vy, base_w likewise",
        "<ee>_cmd_*": "the commanded EE pose, world: FK of the commanded joints at the base pose used (exactly the "
                      "pose the WBC commands this tick; anchor relative targets to it)",
        "<ee>_meas_*": "the measured EE pose, world: FK of the arm's measured q (its latest joint-channel state) at "
                       "the base pose used",
        "<ee>_twist_*": "the commanded EE twist, world axes ([v; w] of the TCP from the commanded base and joint "
                        "velocities)",
        "<ee>_gripper": "gripper command 0 (closed) .. 1 (open) in force (NaN = none given)",
        "arm_age": "s since the arm state used (t_tick - its t_tick)",
        "est_age": "s since the estimator pose used was sampled",
        "target_age": "s since the target in force was stamped (reactive) / past the end of the chunk (0 inside)",
        "limited": "1 if a hard limit bound the motion this tick (see limited_by)",
        "limited_by": "0 none, 1 reach, 2 manipulability, 3 fence, 4 joint_limit, 5 base_limit",
        "input_age": "s since the last datagram (target, chunk or alive) of the session",
        "hold": "1 if the controller held this tick (stale input, no client, estimator unusable, no target)",
        "<ee>_tgt_*": "the target of this tick, world (after blending)",
        "<ee>_reach": "shoulder-to-target distance / the nominal reach (the limit is reach_fraction)",
        "<ee>_manip": "arm-only manipulability (Yoshikawa) at the commanded joints",
        "armed": "1 if armed (targets are followed); 0 disarmed: holding, targets ignored (/state disarm_reason)",
        "tick_period": "this tick's measured period (ms; nominal 1000 / rate_hz)",
        "tick_compute": "this tick's compute time up to the publish (ms)",
        "overruns": "ticks whose period exceeded 1.5 dt (since start)",
        "late_ticks": "ticks later than 3 dt after the previous one (each disarms; since start)",
    }
    return [n for n, _ in f], [u for _, u in f], doc


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class EstimatorView:
    def __init__(self):
        self.row: Optional[dict] = None
        self.rx: Optional[float] = None
        self.reset_id: Optional[float] = None
        self.jumps = 0
        self.last_jump: Optional[str] = None
        self.last_imu: Optional[float] = None


class WbcNode:
    def __init__(self, model: Optional[WbcModel] = None, *, model_source: str = "description",
                 arm: Optional[str] = "arm", swerve: Optional[str] = "swerve",
                 estimator: Optional[str] = "estimator", config: Optional[dict] = None,
                 clock: Callable[[], float] = time.monotonic, udp_host: Optional[str] = "127.0.0.1",
                 udp_port: int = 9410, unix_socket: Optional[str] = None, arm_socket: Optional[str] = None,
                 name: str = "wbc"):
        self.cfg: Config = make_config(config)
        self.clock = clock
        self.name = name
        self.arm_target, self.swerve_target, self.estimator_target = arm, swerve, estimator
        self.model_source = model_source
        self.model: Optional[WbcModel] = model
        self.model_info = "given" if model is not None else None
        self.model_error: Optional[str] = None
        self.ctl: Optional[QPController] = None
        self.ees: list = []
        self.lock = threading.RLock()
        self.events = EventLog()
        self.hub: Optional[RowHub] = None
        self.lease = Lease(self.cfg.lease_heartbeat, on_end=self._client_lease_ended, clock=clock)
        self.estop = EStop(on_trigger=self._estop_triggered, clock=clock)
        self.inq: queue.Queue = queue.Queue(maxsize=10000)
        self.inputs: dict = {}
        self.est = EstimatorView()
        self.sw: Optional[dict] = None
        self.inbox = Inbox()
        if unix_socket is None:                       # "" = none
            from ..common.serve import socket_dir
            unix_socket = str(socket_dir() / f"{name}.target.sock")
        self.receiver = Receiver(udp_host, udp_port, unix_socket, self._deliver)
        self.arm_socket = arm_socket or (str(joint_socket_path(arm)) if arm and "://" not in arm else None)
        self.session_token: Optional[str] = None
        self.hints = Hints()
        self.override: dict = {}
        self.override_t = -math.inf
        self.hint_t = -math.inf          # when the base hint targets in force were given (their feedforward fades)
        self.velocity_t = -math.inf
        # enabled state
        self.state = "disabled"
        self.enabled = False
        self.fault: Optional[str] = None
        self.fault_arm_ok = self.fault_base_ok = False
        self.disabled_reason: Optional[str] = None
        self.holder: Optional[str] = None
        self.arm_http = None
        self.arm_lease_id: Optional[int] = None
        self.channel: Optional[ArmChannel] = None
        self.base = None
        self.enabled_at = -math.inf
        self.arm_state = None
        self.q_cmd: dict = {}
        self.v_cmd: dict = {}
        self.base_twist = np.zeros(3)
        self.cmd_twist: dict = {}
        self.buffers: dict = {}
        self.seq_in_force = 0
        self.last_rx = -math.inf
        self.resyncs = 0
        self.resync = self.cfg.resync
        from ..arm.config import GAP_MAX
        self.gap_max = GAP_MAX
        self._paused = False
        # the armed latch (module docstring)
        self.armed = False
        self.arm_seq = 0                 # the arm_seq that armed it (0: none this session)
        self.arm_seq_seen = 0            # the highest arm_seq in this session's datagrams
        self.arm_floor = 0               # arm_seq_seen at the last disarm: arming needs a greater one
        self.disarm_reason: Optional[str] = "not armed yet"
        self.disarms = 0
        self.latch_counts = {"ignored_disarmed": 0, "arm_refused": 0, "limits_ignored": 0, "clamped": 0}
        # control tick timing (D): the measured period and compute time, overruns (> 1.5 dt), late ticks (> 3 dt)
        self.tick = {"period_ms": math.nan, "compute_ms": math.nan, "max_period_ms": 0.0, "max_compute_ms": 0.0,
                     "overruns": 0, "late": 0}
        self._tick_prev: Optional[float] = None
        self._t0 = time.perf_counter()
        self._late_log = -math.inf
        self.solve_fail = 0
        self.limited_counts: dict = {}
        self._tick_times: list = []
        self._last: dict = {}
        self.realtime = realtime.not_started(realtime.PRIO_CONTROL)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        if self.model is not None:
            self._setup_model()

    # -- model ---------------------------------------------------------------------------------------------

    def _setup_model(self) -> None:
        from ..swerve.constants import MODULES
        self.ctl = QPController(self.model, self.cfg, [(m.x, m.y) for m in MODULES])
        self.ees = list(self.model.ees)
        self.fields, self.units, self.field_doc = stream_fields(self.ees)
        self.col = {n: i for i, n in enumerate(self.fields)}
        self.hub = RowHub({"tick": len(self.fields)}, recent=500)
        self.events.on_add = self.hub.state_changed
        self.buffers = {e: TargetBuffer(e, self.cfg.ff_timeout, self.cfg.blend_tau) for e in self.ees}

    def load_model(self) -> None:
        urdf, how = fetch_urdf(self.model_source)
        self.model = WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))
        self.model_info = how
        self._setup_model()

    # -- lifecycle -----------------------------------------------------------------------------------------

    def start(self, loop: bool = True) -> None:
        """Load the model, start the inputs, the receiver and (``loop``) the control thread."""
        if self.model is None:
            self.load_model()
        from ..estimator.node import Upstream
        if self.estimator_target:
            self.inputs["estimator"] = Upstream("estimator", self.estimator_target, "/stream", self.inq, self.events)
        if self.swerve_target:
            self.inputs["swerve"] = Upstream("swerve", self.swerve_target, "/stream", self.inq, self.events)
        for u in self.inputs.values():
            u.start()
        self.receiver.start()
        self.events.add(f"wbc started (model: {self.model_info}; arm {self.arm_target}, swerve {self.swerve_target}, "
                        f"estimator {self.estimator_target}; targets udp {self.receiver.udp_port}, "
                        f"unix {self.receiver.unix_path})")
        if loop:
            self._thread = threading.Thread(target=self._run, name="wbc-control", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        for u in self.inputs.values():
            u.stop.set()
        if self._thread is not None:
            self._thread.join(2.0)
        if self.enabled:
            self.disable("node shutting down")
        self.receiver.close()

    def _run(self) -> None:
        self.realtime = realtime.make_realtime(realtime.PRIO_CONTROL, "wbc control loop")
        self.events.add(realtime.describe(self.realtime), "info" if self.realtime["error"] is None else "warn")
        dt = self.cfg.dt
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += dt
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()
            try:
                self.step()
            except Exception as e:  # noqa: BLE001 - never let the loop die
                log.exception("control tick")
                self.events.add(f"control error: {type(e).__name__}: {e}", "error")
                with self.lock:
                    if self.enabled:
                        self._fault_locked(f"control error: {e}", True, True)

    # -- inputs --------------------------------------------------------------------------------------------

    def feed_estimator(self, row: dict, now: Optional[float] = None) -> None:
        """One estimator row (fields by name, t on this host's clock)."""
        now = self.clock() if now is None else now
        e, cfg = self.est, self.cfg
        p = e.row
        rid = row.get("reset_id", 0.0)
        if p is not None and rid == e.reset_id:
            dpos = math.hypot(row["x"] - p["x"], row["y"] - p["y"])
            dth = abs(row["theta"] - p["theta"])
            if dpos > cfg.jump_pos or dth > cfg.jump_theta:
                e.jumps += 1
                e.last_jump = f"pose jumped {dpos:.3f} m / {math.degrees(dth):.1f}°"
        if rid != e.reset_id and e.reset_id is not None:
            self.events.add(f"estimator reset (reset_id {e.reset_id:g} -> {rid:g})", "warn")
        e.reset_id = rid
        if row.get("imu_ok", 0.0) == 1.0:
            e.last_imu = now
        e.row, e.rx = row, now

    def _drain_inputs(self, now: float) -> None:
        while True:
            try:
                key, rows, offset = self.inq.get_nowait()
            except queue.Empty:
                return
            names = rows.dtype.names
            if key == "estimator":
                for r in rows:
                    row = {n: float(r[n]) for n in names}
                    row["t"] -= offset
                    self.feed_estimator(row, now)
            elif key == "swerve" and len(rows):
                r = rows[-1]
                self.sw = {"t": float(r["t_tick"]) - offset, "enabled": float(r["enabled"]),
                           "estop": float(r["estop"]), "lease_id": float(r["lease_id"]), "rx": now}

    def estimator_health(self, now: float) -> Optional[str]:
        if not self.estimator_target:
            return None
        e, cfg = self.est, self.cfg
        if e.row is None:
            return "no estimator pose yet"
        if now - e.rx > cfg.est_stale:
            return f"estimator stream stale ({now - e.rx:.2f} s)"
        if not all(math.isfinite(e.row.get(k, math.nan)) for k in ("x", "y", "theta", "t")):
            return "estimator pose is not finite"
        if now - e.row["t"] > cfg.est_pose_stale:
            return f"estimator pose is {now - e.row['t']:.2f} s old"
        if e.row.get("odom_ok", 1.0) != 1.0:
            return "estimator has no wheel odometry"
        if cfg.require_imu and (e.last_imu is None or now - e.last_imu > 0.2):
            return "estimator heading is not IMU-fused (require_imu)"
        return None

    def base_pose(self, now: float) -> tuple[tuple, float]:
        """((x, y, yaw), age): the latest estimator pose moved forward by the commanded twist."""
        r = self.est.row
        if not self.estimator_target or r is None:
            return (0.0, 0.0, 0.0), math.nan
        age = now - r["t"]
        lag = min(max(age, 0.0), self.cfg.max_predict)
        vx, vy, w = self.base_twist
        th = r["theta"] + w * lag
        tm = r["theta"] + w * lag / 2
        return (r["x"] + (math.cos(tm) * vx - math.sin(tm) * vy) * lag,
                r["y"] + (math.sin(tm) * vx + math.cos(tm) * vy) * lag, th), age

    # -- datagrams -----------------------------------------------------------------------------------------

    def _deliver(self, msg: dict, source: str) -> None:
        """Receiver thread: a datagram of the session in force. Renews the WBC client lease."""
        tok = self.session_token
        if tok is not None:
            try:
                self.lease.renew(tok)
            except Exception:  # noqa: BLE001 - the lease lapsed: the tick holds
                pass
        self.inbox.put((msg, self.clock()))

    def open_session(self, token: Optional[str]) -> dict:
        self.lease.check(token)
        with self.lock:
            sid = secrets.randbits(31) or 1
            self.session_token = token
            self.receiver.open_session(sid)
            self.inbox.drain()                           # the previous session's datagrams never apply
            self._disarm_locked("new session", note=True)
            floor = self.arm_floor                       # kept for the node's life: arming needs a greater arm_seq
            self.events.add(f"session {sid} opened (arm_seq floor {floor})")
        return {"session": sid, "udp_port": self.receiver.udp_port, "unix_socket": self.receiver.unix_path,
                "arm_seq": floor}

    # -- the armed latch -------------------------------------------------------------------------------------

    def _disarm_locked(self, reason: str, note: bool = False) -> None:
        """Disarm (under ``self.lock``): clear the targets and the per-datagram base override (never resumed); the
        next ticks hold. One event per armed -> disarmed transition; ``note`` also records the reason when already
        disarmed (enable, a new session)."""
        self.arm_floor = max(self.arm_floor, self.arm_seq_seen, self.arm_seq)
        for b in self.buffers.values():
            b.clear()
        self.override = {}
        self.velocity_t = self.hint_t = -math.inf
        if self.armed:
            self.armed = False
            self.disarms += 1
            self.disarm_reason = reason
            self.events.add(f"DISARMED: {reason}", "warn")
            log.warning("disarmed: %s", reason)
        elif note:
            self.disarm_reason = reason

    def disarm(self, reason: str = "", source: str = "") -> dict:
        """``POST /disarm``: anyone (no lease, like the e-stop). Holds; arming again needs a new ``arm_seq``."""
        why = "POST /disarm" + (f" ({source})" if source else "") + (f": {reason}" if reason else "")
        with self.lock:
            was = self.armed
            self._disarm_locked(why)
            return {"armed": False, "was_armed": was, "disarm_reason": self.disarm_reason}

    def _live(self) -> bool:
        """Arming is possible: enabled, no fault, no e-stop, the arm not paused by a planned motion."""
        return self.enabled and self.fault is None and not self.estop.active and not self._paused

    def _apply_messages(self, now: float) -> None:
        live = self._live()
        for msg, t_rx in self.inbox.drain():
            if msg.get("session") != self.receiver.session:     # queued before a new session (or after its end)
                self.receiver.drop("unknown_session")
                continue
            why = None
            try:
                why = self._apply(msg, now, live)
            except (ValueError, TypeError, KeyError) as e:
                why = "invalid"
                self._note_invalid(f"{msg.get('type')} {msg.get('seq')}: {e}")
            if why:
                self.receiver.drop(why)
            else:
                self.last_rx = max(self.last_rx, t_rx)
                self.seq_in_force = int(msg["seq"])

    def _note_invalid(self, msg: str) -> None:
        t = self.clock()
        if t - self._last.get("invalid_ev", -math.inf) > 1.0:          # at most one event a second
            self._last["invalid_ev"] = t
            self.events.add(f"datagram refused: {msg}", "warn")

    def _apply(self, msg: dict, now: float, live: bool = True) -> Optional[str]:
        """Control thread: one datagram -> the armed latch, then buffers / overrides (only while armed). Returns a
        drop reason, or None (applied, or ignored while disarmed)."""
        kind = msg["type"]
        # the latch first, whatever else the datagram holds: armed / arm_seq are state, sent in every datagram
        flag = msg.get("armed")
        aseq = msg.get("arm_seq")
        aseq = aseq if isinstance(aseq, int) and not isinstance(aseq, bool) else None
        if aseq is not None:
            self.arm_seq_seen = max(self.arm_seq_seen, aseq)
        if self.armed and (flag is not True or aseq is None or aseq < self.arm_seq):
            self._disarm_locked("client: armed false" if flag is False else
                                "client: a datagram without armed / arm_seq" if flag is None or aseq is None else
                                f"client: arm_seq {aseq} older than {self.arm_seq}")
        er = msg.get("est_reset")
        if er is not None and self.est.reset_id is not None and float(er) != float(self.est.reset_id):
            return "est_reset"
        if kind == "alive":
            return None                              # never arms, never resumes
        arming = (not self.armed and flag is True and aseq is not None and aseq > self.arm_floor and live)
        if not self.armed and not arming:
            self.latch_counts["ignored_disarmed"] += 1   # disarmed: targets are ignored, not buffered
            if flag is True and aseq is not None and aseq <= self.arm_floor:
                self.latch_counts["arm_refused"] += 1    # an arm_seq from before the last disarm
            return None
        try:
            why = self._apply_targets(msg, kind, now)
        except BaseException:
            if arming:                               # an arming datagram that fails leaves nothing behind
                for b in self.buffers.values():
                    b.clear()
                self.override = {}
            raise
        if arming:
            if why is None:
                self.armed, self.arm_seq = True, aseq
                self.disarm_reason = None
                self.events.add(f"ARMED (arm_seq {aseq})", "warn")
                log.warning("armed (arm_seq %d)", aseq)
            else:
                for b in self.buffers.values():
                    b.clear()
                self.override = {}
        elif self.armed and aseq > self.arm_seq:     # a newer press while armed: stays armed, newer seq
            self.arm_seq = aseq
        return why

    def _base_moving(self) -> bool:
        return bool(np.any(np.asarray(self.base_twist) != 0.0))

    def _apply_targets(self, msg: dict, kind: str, now: float) -> Optional[str]:
        ees = msg.get("ees") or {}
        if not isinstance(ees, dict) or any(k not in self.buffers for k in ees):
            raise ValueError(f"ees must map end effectors {self.ees} to targets")
        for k, e in ees.items():
            if e.get("frame", "world") != "world":
                return "frame"                       # world-frame targets only (the owner's decision)
        cfg = self.cfg
        if "base" in msg and msg["base"] is not None:
            clamp: dict = {}
            ov = parse_base(msg["base"], cfg, self.ees, clamp=clamp)   # velocities beyond the caps: clamped
            self.latch_counts["clamped"] += clamp.get("n", 0)
            if self._base_moving():
                # caps never change while the base moves: the limits in force stay (counted if it tried)
                cur = self.effective_hints().limits
                if "limits" in ov and ov["limits"] != cur:
                    self.latch_counts["limits_ignored"] += 1
                ov["limits"] = cur
            self.override = ov
            if "target_pose" in ov or "target_heading" in ov:   # parse_base's keys (a "target" key never exists)
                self.hint_t = now
            if "velocity" in ov:
                self.velocity_t = now
        if kind == "target":
            t = msg.get("t")
            t = now if t is None else float(t)
            if not (math.isfinite(t) and abs(t - now) <= cfg.target_t_window):
                return "bad_t"
            for k, e in ees.items():
                p = np.asarray(e["pos"], float).reshape(3)
                R = quat_to_R(e["quat"])
                v = None if e.get("twist") is None else np.asarray(e["twist"], float).reshape(6)
                g = e.get("gripper")
                if not np.all(np.isfinite(p)) or (v is not None and not np.all(np.isfinite(v))):
                    raise ValueError("non-finite pose or twist")
                if g is not None and not (0.0 <= float(g) <= 1.0):
                    raise ValueError("gripper must be within 0..1")
                self.buffers[k].set_reactive(t, p, R, v, None if g is None else float(g), self.est.reset_id)
            return None
        if kind == "chunk":
            for k, e in ees.items():
                t = np.asarray(e["t"], float)
                n = len(t)
                if not (1 <= n <= cfg.chunk_max_len) or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
                    raise ValueError("t must be 1..chunk_max_len strictly increasing times")
                if abs(t[0] - now) > cfg.target_t_window or t[-1] - t[0] > cfg.chunk_max_horizon:
                    return "bad_t"
                if t[-1] < now:
                    return "stale_chunk"
                P = np.asarray(e["pos"], float).reshape(n, 3)
                Rs = np.array([quat_to_R(q) for q in np.asarray(e["quat"], float).reshape(n, 4)])
                V = None if e.get("twist") is None else np.asarray(e["twist"], float).reshape(n, 6)
                G = None if e.get("gripper") is None else np.asarray(e["gripper"], float).reshape(n)
                if not np.all(np.isfinite(P)) or (V is not None and not np.all(np.isfinite(V))):
                    raise ValueError("non-finite pose or twist")
                if G is not None and np.any((G < 0) | (G > 1)):
                    raise ValueError("gripper must be within 0..1")
                ap = aR = av = None
                if self.enabled and k in self.q_cmd:
                    T, V6 = self._cmd_pose(k, now)
                    ap, aR, av = T.translation, T.rotation, V6
                self.buffers[k].set_chunk(now, t, P, Rs, V, G, anchor_p=ap, anchor_R=aR, anchor_v=av,
                                          reset_id=self.est.reset_id)
            return None
        return "bad_type"

    # -- the tick -------------------------------------------------------------------------------------------

    def _cmd_pose(self, ee: str, now: float) -> tuple[pin.SE3, np.ndarray]:
        base, _ = self.base_pose(now)
        q = self.model.config(base, self.q_cmd)
        T = self.model.fk(q, ee)
        return T, self.cmd_twist.get(ee, np.zeros(6))

    def effective_hints(self) -> Hints:
        return merge(self.hints, self.override)

    def step(self) -> None:
        if self.model is None:
            return
        self._t0 = time.perf_counter()
        try:
            self._step()
        finally:
            c = (time.perf_counter() - self._t0) * 1e3
            t = self.tick
            t["compute_ms"] = c
            t["max_compute_ms"] = max(t["max_compute_ms"], c)

    def _tick_timing(self, now: float) -> tuple[float, Optional[float]]:
        """The measured period since the previous tick: counts overruns (> 1.5 dt) and late ticks (> 3 dt); returns
        (the integration step, the measured period clamped to [dt, 3 dt]; the late period or None)."""
        dt = self.cfg.dt
        prev, self._tick_prev = self._tick_prev, now
        if prev is None:
            return dt, None
        period = now - prev
        t = self.tick
        t["period_ms"] = period * 1e3
        t["max_period_ms"] = max(t["max_period_ms"], period * 1e3)
        if period > 1.5 * dt:
            t["overruns"] += 1
        late = None
        if period > 3 * dt:
            t["late"] += 1
            late = period
            if now - self._late_log > 1.0:              # the log at most once a second (the event: on disarm)
                self._late_log = now
                log.warning("control tick late: %.0f ms since the previous one (dt %.0f ms)", period * 1e3, dt * 1e3)
        return min(max(period, dt), 3 * dt), late

    def _step(self) -> None:
        now = self.clock()
        dt_int, late = self._tick_timing(now)
        self._drain_inputs(now)
        st = self.channel.poll(now) if self.channel is not None else None
        if st is not None:
            self.arm_state = st
        self.lease.expire_if_due()
        with self.lock:
            self._tick_times = (self._tick_times + [now])[-100:]
            if late is not None:
                self._disarm_locked(f"control tick overrun: {late * 1e3:.0f} ms")
            if not self.enabled:
                self._apply_messages(now)            # counted (and ignored: nothing to drive)
                for b in self.buffers.values():
                    b.clear()
                self._publish(now, None, hold=True)
                return
            self._check_peers(now)
            if self.fault is not None:
                self._apply_messages(now)
                self._send_fault_hold(now)
                self._publish(now, None, hold=True)
                return
            if self.estop.active:                    # e-stopped, until the disable: hold both halves, track nothing
                self._disarm_locked("e-stop")
                self._apply_messages(now)
                self._send_hold(arm=True, base=True)
                self._publish(now, None, hold=True)
                return
            # the estimator: resets and jumps abort the world-frame targets and disarm; unusable -> hold
            health = self.estimator_health(now)
            if self.est.reset_id != self._last.get("reset_id") or self.est.jumps != self._last.get("jumps"):
                if self._last.get("reset_id") is not None:
                    what = ("estimator reset (world frame changed)" if self.est.reset_id != self._last.get("reset_id")
                            else (self.est.last_jump or "pose jump"))
                    if any(b.kind for b in self.buffers.values()):
                        self.events.add("targets aborted: " + what, "warn")
                    self._disarm_locked(what)
                self._last["reset_id"], self._last["jumps"] = self.est.reset_id, self.est.jumps
            # a planned motion on the arm (HTTP) paused the session: disarm and hold, follow the measured joints,
            # send keep-alives; when it ends, restart from the measured joints (still disarmed)
            st = self.arm_state
            paused = st is not None and st.paused
            if paused != self._paused:
                self._paused = paused
                self.events.add("arm session paused by a planned motion: holding" if paused else
                                "arm session resumed: commands resynced to the measured joints", "warn")
                if paused:
                    self._disarm_locked("arm paused by a planned motion")
                if not paused and st is not None and np.all(np.isfinite(st.q)):
                    self.q_cmd["arm"], self.v_cmd["arm"] = st.q.copy(), np.zeros(len(st.q))
                    self.ctl.reset()
            self._apply_messages(now)
            if self.armed and now - self.last_rx > self.cfg.hold_timeout:
                self._disarm_locked(f"input stale ({now - self.last_rx:.2f} s without a datagram)")
            hints = self.effective_hints()
            # resync the integration to the measured joints past a threshold (well inside the arm's lead bound)
            if st is not None and "arm" in self.q_cmd and np.all(np.isfinite(st.q)) and not paused:
                if np.max(np.abs(st.q - self.q_cmd["arm"])) > self.resync:
                    self.resyncs += 1
                    self.events.add(f"arm command resynced to the measured joints (off by "
                                    f"{np.degrees(np.max(np.abs(st.q - self.q_cmd['arm']))):.1f}°)", "warn")
                    self.q_cmd["arm"] = st.q.copy()
                    self.v_cmd["arm"] = np.zeros(len(st.q))
            stale = now - self.last_rx > self.cfg.hold_timeout and any(b.kind for b in self.buffers.values())
            no_client = not self.lease.held
            why_hold = ("arm paused by a planned motion" if paused else ("estimator: " + health) if health else
                        ("no client lease" if no_client else ("input stale" if stale else
                                                              None if self.armed else
                                                              f"disarmed: {self.disarm_reason}")))
            base, est_age = self.base_pose(now)
            targets = {e: (None if why_hold or not self.armed else self.buffers[e].sample(now)) for e in self.ees}
            snap = Snapshot(now, base, self.base_twist.copy(),
                            {e: EEState(self.q_cmd[e].copy(), self.v_cmd[e].copy(), targets[e],
                                        hints.ee(e, self.cfg), self._posture(hints, e)) for e in self.ees},
                            hints, hold=why_hold is not None, base_available=self.base is not None,
                            velocity_age=now - self.velocity_t, hint_age=now - self.hint_t, dt=dt_int)
            out = self.ctl.step(snap)
            if out.status != "ok":
                self.solve_fail += 1
                self._fault_locked(f"controller: {out.status}: {out.reason}", True, True)
                self._send_fault_hold(now)
                self._publish(now, out, hold=True, targets=targets, base=base, est_age=est_age)
                return
            for e in self.ees:
                if self.armed:
                    self.q_cmd[e] = self.q_cmd[e] + out.dq[e]
                    self.v_cmd[e] = out.qd[e].copy()
                else:                                        # disarmed: the arm held at its commanded joints
                    self.v_cmd[e] = np.zeros_like(self.v_cmd[e])
            if paused and np.all(np.isfinite(st.q)):         # the planned motion moves the arm, not the WBC
                self.q_cmd["arm"], self.v_cmd["arm"] = st.q.copy(), np.zeros(len(st.q))
            self.base_twist = np.asarray(out.base_twist, float)
            self._cmd_twists(base)
            if out.limited_by != "none":
                self.limited_counts[out.limited_by] = self.limited_counts.get(out.limited_by, 0) + 1
            self.state = ("idle" if not self.armed else "holding" if why_hold else
                          "tracking" if any(t is not None for t in targets.values()) else "idle")
            self._hold_reason = why_hold
            gr = targets.get("arm")
            gap = math.nan
            if gr is not None and gr.gripper is not None:
                gap = gr.gripper * self.gap_max
            self._send(now, gap, keepalive=paused)
            self._publish(now, out, hold=why_hold is not None, targets=targets, base=base, est_age=est_age)

    def _posture(self, hints: Hints, ee: str) -> np.ndarray:
        p = hints.ee(ee, self.cfg).posture
        q = POSTURES[p] if isinstance(p, str) else np.asarray(p, float)
        lo, hi = self.model.arm_limits(ee)
        return np.clip(q, lo, hi)

    def _cmd_twists(self, base) -> None:
        m = self.model
        q = m.config(base, self.q_cmd)
        m.update(q)
        c, s = math.cos(base[2]), math.sin(base[2])
        v = np.zeros(m.nv)
        vx, vy, w = self.base_twist
        v[m.base_v] = [c * vx - s * vy, s * vx + c * vy, w]
        for e in self.ees:
            v[m.arm_v[e]] = self.v_cmd[e]
        for e in self.ees:
            self.cmd_twist[e] = m.jacobian(q, e) @ v

    def _send(self, now: float, gap: float, keepalive: bool = False) -> None:
        st = self.arm_state
        if self.channel is not None and "arm" in self.q_cmd:
            self.channel.send(self.q_cmd["arm"], self.v_cmd["arm"], gap,
                              t_basis=st.t_tick if st is not None else math.nan, keepalive=keepalive)
        if self.base is not None:
            self.base.send(self.base_twist)

    def _send_fault_hold(self, now: float) -> None:
        """In a fault: the healthy half holds (the arm its commanded joints, the base zero)."""
        self._send_hold(arm=self.fault_arm_ok, base=self.fault_base_ok)

    def _send_hold(self, arm: bool, base: bool) -> None:
        """Track nothing: the arm keeps its commanded joints (zero velocity), the base gets a zero twist."""
        self.base_twist = np.zeros(3)
        for e in self.v_cmd:
            self.v_cmd[e] = np.zeros_like(self.v_cmd[e])
        if arm and self.channel is not None and "arm" in self.q_cmd:
            st = self.arm_state
            self.channel.send(self.q_cmd["arm"], np.zeros(len(self.q_cmd["arm"])),
                              keepalive=st is not None and st.paused)
        if base and self.base is not None:
            self.base.send((0.0, 0.0, 0.0))

    def _check_peers(self, now: float) -> None:
        """Cross-bus checks: an e-stop on either bus e-stops the other; a fault, lease loss or silence on one stops
        the other half (fault)."""
        if self.fault is not None:
            return
        st = self.arm_state
        if self.channel is not None:
            if self.channel.closed_reason:                    # the arm's bye: lease end, e-stop, disable, replaced
                self._fault_locked(f"arm joint channel closed by the arm: {self.channel.closed_reason}", False, True)
                return
            if now - self.enabled_at > ARM_GRACE_S:
                if st is None or now - st.t_rx > self.cfg.arm_stale:
                    self._fault_locked("arm state stale (no joint-channel state)", False, True)
                    return
                if st.state_name in ("off", "fault"):
                    if st.state_name == "off":                   # the arm's e-stop puts it off: e-stop the base too
                        self._forward_estop("arm went off (e-stop?)", base=True, arm=False)
                    self._fault_locked(f"arm is {st.state_name}", False, True)
                    return
                if st.state_name not in ("floating", "servoing"):
                    self._fault_locked(f"arm is {st.state_name}", False, True)
                    return
        sw = self.sw
        if self.base is not None:
            if self.base.failing_for() > CMD_FAIL_S:
                self._fault_locked(f"swerve commands failing ({self.base.last_error})", True, False)
                return
            if sw is not None and sw["t"] > self.enabled_at + SWERVE_GRACE_S:
                if sw["estop"]:
                    self._forward_estop("swerve e-stop latched", base=False, arm=True)
                    self._fault_locked("swerve e-stop latched", False, False)
                elif sw["lease_id"] != self.base.lease_id:
                    self._fault_locked("swerve lease lost", True, False)
                elif not sw["enabled"]:
                    self._fault_locked("swerve disabled", True, False)

    def _fault_locked(self, reason: str, arm_ok: bool, base_ok: bool) -> None:
        if self.fault is not None:
            return
        self.fault = reason
        self.fault_arm_ok, self.fault_base_ok = arm_ok, base_ok
        self.state = "fault"
        self._disarm_locked(f"fault: {reason}")
        self.ctl.reset()
        self.events.add(f"FAULT: {reason} (" + ", ".join(
            [("arm holds" if arm_ok else "arm stopped"), ("base zero" if base_ok else "base stopped")]) +
            "); POST /disable to clear", "error")
        if not base_ok and self.base is not None:
            self.base.close(disable=True)
            self.base = None

    def _forward_estop(self, reason: str, *, base: bool, arm: bool) -> None:
        def go():
            if arm and self.arm_http is not None:
                try:
                    self.arm_http.estop(f"wbc: {reason}")
                except Exception as e:  # noqa: BLE001
                    self.events.add(f"e-stop forward to the arm failed: {e}", "error")
            if base and self.swerve_target:
                try:
                    from ..swerve.client import SwerveClient
                    with SwerveClient(self.swerve_target, timeout=0.5) as c:
                        c.estop(f"wbc: {reason}", "wbc")
                except Exception as e:  # noqa: BLE001
                    self.events.add(f"e-stop forward to the swerve failed: {e}", "error")
        threading.Thread(target=go, daemon=True).start()

    # -- publish -------------------------------------------------------------------------------------------

    def _publish(self, now: float, out, hold: bool, targets=None, base=None, est_age=math.nan) -> None:
        nan = math.nan
        if base is None:
            base, est_age = self.base_pose(now)
        h = self.effective_hints()
        st = self.arm_state
        row = [now, float(self.seq_in_force), float(STATES.index(self.state)), float(MODES.index(h.mode)),
               out.solve_ms if out else nan, float(SOLVE.index(out.status)) if out else 0.0,
               base[0], base[1], base[2], nan if self.est.reset_id is None else float(self.est.reset_id),
               *self.base_twist]
        m = self.model
        tail = []
        for e in self.ees:
            if e in self.q_cmd:
                T = m.fk(m.config(base, self.q_cmd), e)
                row += [*T.translation, *R_to_quat(T.rotation)]
            else:
                row += [nan] * 7
            if st is not None and np.all(np.isfinite(st.q)):
                qm = dict(self.q_cmd)
                qm[e] = st.q
                Tm = m.fk(m.config(base, qm if qm else {e: st.q}), e)
                row += [*Tm.translation, *R_to_quat(Tm.rotation)]
            else:
                row += [nan] * 7
            row += list(self.cmd_twist.get(e, [nan] * 6)) if self.enabled else [nan] * 6
            g = self.buffers[e].gripper if e in self.buffers else None
            row += [nan if g is None else g]
        tg_age = nan
        if targets:
            ages = [t.age for t in targets.values() if t is not None]
            tg_age = min(ages) if ages else nan
        row += [nan if st is None else now - st.t_tick, est_age, tg_age]
        lim = out.limited_by if out else "none"
        row += [float(lim != "none"), float(LIMITED.index(lim)), now - self.last_rx if self.last_rx > -math.inf else nan,
                float(hold), float(out.deadband) if out else 0.0, float(out.diag.get("solves", 0)) if out else 0.0,
                float(out.diag.get("wheel_max", nan)) if out else nan]
        for e in self.ees:
            t = targets.get(e) if targets else None
            row += [*t.p, *R_to_quat(t.R)] if t is not None else [nan] * 7
            d = out.diag if out else {}
            row += [d.get("err_pos", {}).get(e, nan), d.get("err_rot", {}).get(e, nan),
                    t.ff_scale if t is not None else nan, float(t.id) if t is not None else nan,
                    d.get("manip", {}).get(e, nan), d.get("reach", {}).get(e, nan)]
            row += list(self.q_cmd.get(e, [nan] * 6)) + list(self.v_cmd.get(e, [nan] * 6))
        tk = self.tick
        row += [float(self.armed), tk["period_ms"], (time.perf_counter() - self._t0) * 1e3, float(tk["overruns"]),
                float(tk["late"])]
        self._last["out"] = out
        self.hub.publish_rows(np.array([row], dtype=np.float64), "tick")

    # -- requests ------------------------------------------------------------------------------------------

    def acquire(self, client: str):
        token, info = self.lease.acquire(client)
        self.events.add(f"lease {info.lease_id} acquired by {info.client}")
        return token, info

    def renew(self, token):
        return self.lease.renew(token)

    def release(self, token) -> None:
        self.lease.release(token)

    def _client_lease_ended(self, reason: str) -> None:
        self.receiver.close_session()
        self.session_token = None
        with self.lock:
            self._disarm_locked(f"client lease {reason}")
        self.events.add(f"client lease {reason}" + (": holding (POST /disable releases the arm and base)"
                                                    if self.enabled else ""), "warn" if self.enabled else "info")

    def _estop_triggered(self, info: EStopInfo) -> None:
        self.events.add(f"E-STOP ({info.source}: {info.reason})", "error")

    def enable(self, token: Optional[str]) -> dict:
        """Take the arm and swerve leases together (rolling back on any failure), enable both, open the arm's
        joint channel and seed the commanded joints from its state. Blocking."""
        self.lease.check(token)
        if self.estop.active:
            raise RequestError(409, "e-stop is latched; clear it first (POST /estop/clear)")
        if self.model is None:
            raise RequestError(503, f"no model ({self.model_error or 'not loaded'})")
        with self.lock:
            if self.enabled or self.state == "enabling":
                raise RequestError(409, f"already {self.state}; disable first")
            self.state = "enabling"
            holder = self.lease.info.client if self.lease.info else "anonymous"
        name = f"wbc ({holder})"
        arm_http = channel = base = None
        try:
            if self.arm_target:
                arm_http = ArmHttpFactory(self.arm_target)
                try:
                    out = arm_http.acquire(name)
                except Exception as e:
                    raise RequestError(getattr(e, "status", 503), f"arm lease: {getattr(e, 'detail', e)}")
                arm_lease = out.get("lease_id")
            if self.swerve_target:
                from .links import SwerveBase
                base = SwerveBase(self.swerve_target)
                try:
                    base.open(name, self.cfg.max_speed, self.cfg.max_omega)
                except Exception as e:
                    base = None
                    from .links import LimitsError
                    status = 422 if isinstance(e, LimitsError) else getattr(e, "status", 503)
                    raise RequestError(status, f"swerve: {getattr(e, 'detail', e)}")
            q0 = None
            if arm_http is not None:
                try:
                    arm_http.enable()
                except Exception as e:
                    raise RequestError(getattr(e, "status", 503), f"arm enable: {getattr(e, 'detail', e)}")
                off = arm_http.tcp()
                if off is not None:
                    self.model.set_tcp("arm", off)
                channel = ArmChannel(self.arm_socket)
                try:
                    channel.open(arm_http.token)
                except ChannelError as e:
                    channel = None
                    raise RequestError(503, f"arm joint channel: {e}")
                t_end = time.monotonic() + 1.0
                st = None
                while time.monotonic() < t_end:
                    st = channel.poll(self.clock())
                    if st is not None and np.all(np.isfinite(st.q)):
                        break
                    time.sleep(0.005)
                if st is None or not np.all(np.isfinite(st.q)):
                    raise RequestError(503, "no state from the arm joint channel within 1 s")
                q0 = st.q.copy()
                self._apply_arm_limits(channel.limits)
        except BaseException:
            if channel is not None:
                channel.close()
            if base is not None:
                base.close(disable=True)
            if arm_http is not None:
                for call in (arm_http.disable, arm_http.release):
                    try:
                        call()
                    except Exception:  # noqa: BLE001
                        pass
                arm_http.close()
            with self.lock:
                self.state = "disabled"
            raise
        with self.lock:
            self.arm_http, self.channel, self.base = arm_http, channel, base
            self.arm_lease_id = arm_lease if arm_http is not None else None
            self.arm_state = st if arm_http is not None else None
            n = len(self.model.arm_q["arm"])
            self.q_cmd = {e: (q0.copy() if q0 is not None else self._posture(self.hints, e)) for e in self.ees}
            self.v_cmd = {e: np.zeros(n) for e in self.ees}
            self.base_twist = np.zeros(3)
            self.cmd_twist = {e: np.zeros(6) for e in self.ees}
            # disarmed at enable: datagrams queued before it never arm (their arm_seq counts as seen)
            for msg, _ in self.inbox.drain():
                a = msg.get("arm_seq")
                if isinstance(a, int) and not isinstance(a, bool) and msg.get("session") == self.receiver.session:
                    self.arm_seq_seen = max(self.arm_seq_seen, a)
            self._disarm_locked("enable", note=True)
            self.ctl.reset()
            self.enabled, self.fault, self.disabled_reason = True, None, None
            self._paused = False
            self.enabled_at = self.clock()
            self.holder = holder
            self.state = "idle"
            self.events.add(f"enabled by {holder} (arm lease {self.arm_lease_id}, swerve lease "
                            f"{base.lease_id if base else None})")
        return self.status()

    def _apply_arm_limits(self, lim: dict) -> None:
        """The arm's own limits from its welcome: joint range, joint speed and acceleration, lead bound, gap."""
        n = len(self.model.arm_q["arm"])
        try:
            if lim.get("lo") is not None and lim.get("hi") is not None:
                self.model.narrow_arm_limits("arm", lim["lo"][:n], lim["hi"][:n])
            from ..arm.config import DEFAULT_SOFT
            self.ctl.caps["arm"] = self.ctl.arm_caps(n, lim.get("joint_vel") or DEFAULT_SOFT.joint_vel,
                                                     lim.get("joint_acc") or DEFAULT_SOFT.joint_acc)
            if lim.get("max_lead"):
                self.resync = min(self.cfg.resync, 0.7 * float(lim["max_lead"]))
            if lim.get("gap_max"):
                self.gap_max = float(lim["gap_max"])
        except (TypeError, ValueError, IndexError) as e:
            self.events.add(f"arm limits in the welcome not usable ({e}); keeping the WBC's", "warn")

    def disable(self, reason: str = "requested", token: Optional[str] = None, check: bool = False) -> dict:
        if check:
            self.lease.check(token)
        with self.lock:
            was = self.enabled
            if not was and self.state != "fault":
                return {"enabled": False, "was_enabled": False}
            channel, base, arm_http = self.channel, self.base, self.arm_http
            # a last hold command, then close: the arm holds its command (stale = hold) until its lease ends
            if channel is not None and "arm" in self.q_cmd:
                channel.send(self.q_cmd["arm"], np.zeros(len(self.q_cmd["arm"])))
            self.channel = self.base = self.arm_http = None
            self._disarm_locked(f"disabled: {reason}")
            self.enabled, self.fault, self.state = False, None, "disabled"
            self.disabled_reason = reason
            self.base_twist = np.zeros(3)
            self.events.add(f"disabled: {reason}")
        if base is not None:
            base.close(disable=True)
        if channel is not None:
            channel.close()
        if arm_http is not None:
            def release():
                for call in (arm_http.disable, arm_http.release):
                    try:
                        call()
                    except Exception:  # noqa: BLE001
                        pass
                arm_http.close()
            threading.Thread(target=release, daemon=True).start()
        return {"enabled": False, "was_enabled": was}

    ESTOP_JOIN_S = 2.5          # the fan-out waits this long for the arm's and the swerve's answers

    def trigger_estop(self, reason: str, source: str) -> dict:
        """Anyone. First, under the lock, stop tracking: disarm, zero base twist, the arm held at its commanded
        joints (the ticks hold too until the disable). Then the e-stop goes to the arm and the swerve IN PARALLEL
        (both latch and coast: no braking by the robot), then disable."""
        info = self.estop.trigger(reason, source)
        with self.lock:
            self._disarm_locked(f"e-stop ({reason or 'no reason'})")
            if self.enabled:
                self._send_hold(arm=True, base=True)    # zero twist, the arm at its commanded joints
        fw: dict = {}
        msg = f"wbc: {reason}" if reason else "wbc"

        def to_arm():
            h = self.arm_http or ArmHttpFactory(self.arm_target)
            h.estop(msg, source or "wbc")

        def to_swerve():
            from ..swerve.client import SwerveClient
            with SwerveClient(self.swerve_target, timeout=0.5) as c:
                c.estop(msg, source or "wbc")

        def run(name, fn):
            try:
                fn()
                fw[name] = True
            except Exception as e:  # noqa: BLE001
                fw[name] = False
                self.events.add(f"e-stop forward to the {name} failed: {e}", "error")

        threads = [threading.Thread(target=run, args=(n, f), name=f"wbc-estop-{n}", daemon=True)
                   for n, f, t in (("arm", to_arm, self.arm_target), ("swerve", to_swerve, self.swerve_target)) if t]
        for t in threads:
            t.start()
        deadline = time.monotonic() + self.ESTOP_JOIN_S
        for t in threads:
            t.join(max(0.0, deadline - time.monotonic()))
        out = {"latched": True, "reason": info.reason, "source": info.source, "forwarded": {}}
        for n, t in (("arm", self.arm_target), ("swerve", self.swerve_target)):
            if t:
                out["forwarded"][n] = bool(fw.get(n, False))
                if n not in fw:
                    self.events.add(f"e-stop forward to the {n}: no answer within {self.ESTOP_JOIN_S} s", "error")
        self.disable(f"e-stop ({reason or 'no reason'})")
        return out

    def clear_estop(self, token: Optional[str]) -> None:
        self.lease.check(token)
        self.estop.clear()
        self.events.add("e-stop cleared (the arm's and the swerve's are cleared there)")

    def set_hints(self, token: Optional[str], doc: dict) -> dict:
        self.lease.check(token)
        h = parse_hints(doc, self.cfg, self.ees, POSTURES)
        with self.lock:
            self.hints, self.override = h, {}
            self.hint_t = self.clock()
            if h.velocity is not None:
                self.velocity_t = self.clock()
            self.events.add(f"hints: base {h.mode}")
        return self.get_hints()

    def get_hints(self) -> dict:
        with self.lock:
            return {"hints": hints_public(self.hints), "in_force": hints_public(self.effective_hints()),
                    "postures": {k: [round(float(x), 6) for x in v] for k, v in POSTURES.items()}}

    # -- state ---------------------------------------------------------------------------------------------

    def status(self) -> dict:
        with self.lock:
            return {"state": self.state, "enabled": self.enabled, "fault": self.fault, "armed": self.armed}

    def state_dict(self) -> dict:
        now = self.clock()
        with self.lock:
            last = self.hub.recent(1, "tick") if self.hub else []
            latest = ({k: (v if math.isfinite(v) else None) for k, v in zip(self.fields, last[0].tolist())}
                      if len(last) else None)
            ts = self._tick_times
            rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 10 and ts[-1] > ts[0] else None
            info, e = self.lease.info, self.estop.info
            out = self._last.get("out")
            st = self.arm_state
            lim = out.limited_by if out is not None else "none"
            return {
                "state": self.state,
                "enabled": self.enabled,
                "armed": self.armed,
                "disarm_reason": None if self.armed else self.disarm_reason,
                "latch": {"arm_seq": self.arm_seq, "arm_seq_seen": self.arm_seq_seen, "arm_floor": self.arm_floor,
                          "disarms": self.disarms, **self.latch_counts},
                "tick": {k: (round(v, 3) if isinstance(v, float) and math.isfinite(v) else
                             None if isinstance(v, float) else v) for k, v in self.tick.items()},
                "holder": self.holder if self.enabled else None,
                "fault": self.fault,
                "hold_reason": getattr(self, "_hold_reason", None) if self.enabled else None,
                "disabled_reason": self.disabled_reason,
                "limited_by": lim,
                "limited_counts": dict(self.limited_counts),
                "estop": {"latched": e is not None, "reason": e and e.reason, "source": e and e.source,
                          "t": e and e.t},
                "lease": {"held": info is not None, "holder": info and info.client,
                          "lease_id": info and info.lease_id,
                          "expires_in": None if info is None else round(info.expires - now, 3)},
                "session": {"session": self.receiver.session, "udp_port": self.receiver.udp_port,
                            "unix_socket": self.receiver.unix_path, "seq": self.receiver.seq,
                            "accepted": self.receiver.counts["accepted"],
                            "dropped": dict(self.receiver.counts["dropped"]),
                            "last_source": self.receiver.last_source,
                            "input_age": None if self.last_rx == -math.inf else round(now - self.last_rx, 4)},
                "model": {"source": self.model_info, "error": self.model_error,
                          "ees": {k: {"link": v.link, "tcp": list(v.tcp), "reach": round(self.model.reach[k], 4)}
                                  for k, v in self.model.ees.items()} if self.model else {}},
                "arm": {"target": self.arm_target, "lease_id": self.arm_lease_id if self.enabled else None,
                        "channel": None if self.channel is None else {
                            "session": self.channel.session, "stale_ms": self.channel.stale_ms,
                            **self.channel.counts, "last_error": self.channel.last_error},
                        "state": None if st is None else st.state_name,
                        "paused": bool(st is not None and st.paused),
                        "limits": None if self.channel is None else self.channel.limits,
                        "age": None if st is None else round(now - st.t_rx, 4),
                        "resyncs": self.resyncs},
                "swerve": {"target": self.swerve_target,
                           "lease_id": self.base.lease_id if self.base else None,
                           "sent": self.base.sent if self.base else None,
                           "errors": self.base.errors if self.base else None,
                           "telemetry": self.sw and {k: v for k, v in self.sw.items() if k != "rx"}},
                "estimator": {"health": self.estimator_health(now) or "ok", "reset_id": self.est.reset_id,
                              "pose_jumps": self.est.jumps,
                              "age": None if self.est.row is None else round(now - self.est.row["t"], 4)},
                "inputs": {k: u.status() for k, u in self.inputs.items()},
                "hints": hints_public(self.effective_hints()),
                "config": {k: (list(v) if isinstance(v, tuple) else v) for k, v in asdict(self.cfg).items()},
                "solve_failures": self.solve_fail,
                "loop_hz": None if rate is None else round(rate, 1),
                "realtime": dict(self.realtime),
                "latest": latest,
                "events": self.events.list()[-30:],
            }

    # -- runtime config ------------------------------------------------------------------------------

    STRUCTURAL = ("rate_hz", "lease_heartbeat", "joint_vel", "joint_acc", "reach")

    def get_config(self) -> dict:
        from dataclasses import asdict as _asdict
        from .spec import Config as _C
        js = lambda d: {k: (list(v) if isinstance(v, tuple) else v) for k, v in d.items()}   # noqa: E731
        hard = dict(self.base.hard) if self.base is not None and getattr(self.base, "hard", None) else {}
        return {"config": js(_asdict(self.cfg)), "defaults": js(_asdict(_C())), "hard": hard,
                "structural": list(self.STRUCTURAL)}

    BASE_CAPS = ("max_speed", "max_omega", "max_accel", "max_alpha", "wheel_speed")

    def _caps_check_locked(self, changes: dict) -> None:
        """The base caps change only while the base is still (disabled, or a zero commanded twist)."""
        changed = [k for k in self.BASE_CAPS if k in changes and changes[k] != getattr(self.cfg, k)]
        if changed and self.enabled and self._base_moving():
            tw = ", ".join(f"{v:.3f}" for v in self.base_twist)
            raise ValueError(f"{changed}: the base caps cannot change while the base moves (commanded twist {tw}); "
                             "stop it first (disarm, or base mode locked)")

    def set_config(self, changes: dict) -> dict:
        """Change config keys at runtime (any time; no lease: the operator tunes while a client drives). Validated
        as at start, the base caps against the swerve's hard limits and only while the base is still; structural
        keys only while disabled."""
        changes = dict(changes or {})
        with self.lock:
            if self.enabled and any(k in changes for k in self.STRUCTURAL):
                raise ValueError(f"{sorted(set(changes) & set(self.STRUCTURAL))}: only while disabled")
            self._caps_check_locked(changes)
            new = make_config(changes, base=self.cfg)
            hard = getattr(self.base, "hard", None) or {} if self.base is not None else {}
            hs, hw = hard.get("max_speed"), hard.get("max_omega")
            if (hs is not None and new.max_speed > hs) or (hw is not None and new.max_omega > hw):
                raise ValueError(f"max_speed / max_omega beyond the swerve's hard limits ({hs} m/s, {hw} rad/s)")
            old = self.cfg
            push = self.enabled and self.base is not None and (new.max_speed, new.max_omega) != (old.max_speed,
                                                                                                old.max_omega)
        if push:
            self.base.client.set_limits(new.max_speed, new.max_omega)
        moved = None
        with self.lock:
            try:
                self._caps_check_locked(changes)          # the base started moving meanwhile: undo the push
            except ValueError as e:
                moved = e
            base = self.base
            if moved is None:
                self.cfg = new
                self.ctl.cfg = new
                for b in self.buffers.values():
                    b.ff_timeout, b.blend_tau = new.ff_timeout, new.blend_tau
                self.events.add("config: " + ", ".join(f"{k}={v}" for k, v in changes.items()))
        if moved is not None:
            if push and base is not None and base.client is not None:
                try:
                    base.client.set_limits(old.max_speed, old.max_omega)
                except Exception:  # noqa: BLE001
                    pass
            raise moved
        return self.get_config()


def ArmHttpFactory(target: str):
    """The arm's HTTP client (a hook for tests)."""
    from .links import ArmHttp
    return ArmHttp(target)
