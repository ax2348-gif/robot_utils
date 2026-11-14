"""The arm node's core (docs/arm.md): one control thread owns the bus; requests only write small,
locked pieces of state and never touch it.

States (``state``):

  connecting     opening the bus (retries)
  uninitialized  connected, motors off (damping), gripper not checked yet
  initializing   the first enable's gripper check
  off            motors off (damping): after an e-stop, a fault or a stow. The arm sinks.
  disabled       arm motors on and floating (kp = kd = 0, gravity feedforward); gripper motor off
                 (damping); commands refused until enable (after disable, a lease that ended, a chock)
  floating       enabled; arm floating; the first closed-loop arm command (track, servo, chunk,
                 home, stow) engages servoing
  servoing       enabled; arm closed loop
  fault          motors off; ``reset`` reconnects

The gripper is separate from the arm: enabled, it is free (gainless) until a gripper command
makes it hold; float does not touch it; disable puts it into damping (its motor off); off puts
everything into damping. E-stop is a latched flag that forces ``off``.

Where the arm is comes only from the readings. Whenever it is not servoing (floating, disabled,
off) it may be moving (by hand, by inertia, sinking), so there is no commanded pose then
(``q_cmd`` is None) and anything that starts closed-loop control seeds from the latest reading:
the command starts at the motor-side positions with the measured velocity and is braked at the
acceleration limit; IK and plans start from the measured pose.

Control tick (100 Hz): read (missing or faulted reading → fault) → chocks, lease, cut / engage
requests → one MIT frame per motor (servo, float or released) → telemetry row.
"""
from __future__ import annotations

import base64
import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
import pinocchio as pin

from ..common.estop import EStop, EStopInfo
from ..common.hub import RowHub
from ..common.lease import Lease, RequestError
from ..common.locks import LockStore, describe as describe_locks
from ..common import realtime
from ..common.server import EventLog
from . import config as C
from .bus import ArmBus, BusError, ReadMiss
from .kin import ArmKinematics, N, pose_error, pose_public
from .motion import PlanError, Trajectory
from .planner import Planner
from .render import VIEW_DEFAULTS

log = logging.getLogger(__name__)

RATE_HZ = 100.0
DT = 1.0 / RATE_HZ
TWIST_TIMEOUT = 0.25          # s a track twist feedforward stays in force
LEASE_HEARTBEAT = 3.0         # s: a browser sharing its 6 connections across robot tabs can't renew
                              # reliably within 1 s, and a lapse disables (gripper into damping)
STALE_ON = 0.05               # s the latest complete reading may age (missed replies) with the motors on
STALE_OFF = 1.0               # s ... with the motors off
STALE_WARN = 0.03             # s without a complete reading (motors on) that is reported as a warning event,
STALE_WARN_EVERY = 5.0        # at most this often (the gaps in between are counted in the next one)
LATENCY_WINDOW = 500          # ticks (5 s) of reply latencies behind the stats in /state
LATENCY_FIRST = 300           # ticks after connecting whose latencies are reported as one event
LATENCY_MARGIN_WARN = 0.001   # s between the p99 reply latency and the tick period below which it warns:
LATENCY_WARN_EVERY = 60.0     # replies later than one tick count as missed (at most one warning a minute)
LATE_WARN = 0.08              # s between ticks reported as an event (the motors' own 100 ms watchdog
                              # protects them: they drop to damping and report it, which faults)
ENGAGE_LIMIT_TOL = math.radians(3.0)   # servoing may start with a joint this far outside its limits
ENC_MISMATCH_ENABLE = 0.05    # rad between output encoder and motor feedback that refuses enable
PLAN_DRIFT = 0.02             # rad the arm (measured) may move while a motion is planned
TRACK_IK_ITER = 30            # the Quest teleop's per-command IK iterations (warm-started)
TRACK_MAX_JUMP = math.radians(20.0)    # a track goal this far (any joint) from the measured pose is not taken (use servo)
STOW_NEAR = math.radians(3.0)  # every joint this close to home: stow turns the motors off at once
SLOW_TICK = 0.015             # s between ticks counted as an overrun
ENABLE_TIMEOUT = 8.0          # s a POST /enable waits for initialization
CONNECT_RETRY_S = 2.0
STATES = ["connecting", "uninitialized", "initializing", "off", "disabled", "floating", "servoing", "fault"]
SOURCES = ["none", "hold", "track", "servo", "pose", "home", "stow", "chunk"]
PHASES = ["none", "executing", "settling", "reached", "not_reached", "done", "aborted", "preempted", "jammed"]
JOINT_COLS = [("q", "rad"), ("q_mot", "rad"), ("qd", "rad/s"), ("tau", "Nm"), ("q_cmd", "rad"),
              ("qd_ff", "rad/s"), ("tau_g", "Nm"), ("e_mot", "rad"), ("tau_est", "Nm"), ("acc_lim", "bool"),
              ("t_mos", "C"), ("status", "enum")]
_BASE = [("t_tick", "s"), ("tick", "count"), ("state", "enum"), ("enabled", "bool"), ("motors_on", "bool"),
         ("servo", "bool"), ("g_servo", "bool"), ("estop", "bool"), ("lease_id", "id"), ("source", "enum"),
         ("seq", "count"), ("overruns", "count"), ("tick_dt", "s"), ("limited", "bool"), ("twist_age", "s"),
         ("read_age", "s"), ("read_misses", "count")]
_TCP = [("x", "m"), ("y", "m"), ("z", "m"), ("qx", ""), ("qy", ""), ("qz", ""), ("qw", ""),
        ("cmd_x", "m"), ("cmd_y", "m"), ("cmd_z", "m"),
        ("err_pos", "m"), ("err_rot", "rad"), ("goal_err_pos", "m"), ("goal_err_rot", "rad"),
        ("w", ""), ("w_lin", ""), ("sigma_min", "")]
_GRIP = [("g_gap", "m"), ("g_gap_cmd", "m"), ("g_gap_goal", "m"), ("g_qd", "rad/s"), ("g_tau", "Nm"),
         ("g_tau_est", "Nm"), ("g_force", "N"), ("g_force_demand", "N"), ("g_tau_ff", "Nm"), ("g_force_max", "N"), ("g_limiting", "bool"),
         ("g_status", "enum")]
_MOTION = [("motion_id", "id"), ("phase", "enum"), ("progress", "")]
_JAM = [("jam", "bool")]   # appended last: existing columns keep their positions
_ALL = (_BASE + [(f"j{i}_{n}", u) for i in range(1, N + 1) for n, u in JOINT_COLS] + _TCP + _GRIP + _MOTION + _JAM)
FIELDS = [n for n, _ in _ALL]
UNITS = [u for _, u in _ALL]
COL = {n: i for i, n in enumerate(FIELDS)}
JAC_FIELDS = ["t_tick"] + [f"J{r}{c}" for r in range(6) for c in range(1, N + 1)]
JAC_UNITS = ["s"] + [""] * 36
FIELD_DOC = {
    "state": "index into " + ", ".join(STATES),
    "servo / g_servo": "arm / gripper closed loop engaged",
    "source": "arm motion source in force: index into " + ", ".join(SOURCES) + " (none = not servoing)",
    "phase": "of the planned motion (servo / pose / home / stow / chunk): index into " + ", ".join(PHASES),
    "jam": "1 while a jam is in force: the arm static but still pushing (residual over its band; GET /jam)",
    "j<i>_q": "output encoder (xout): the arm's measured pose",
    "j<i>_q_mot / qd / tau": "motor-side feedback frame",
    "j<i>_q_cmd / qd_ff": "position command and the MIT velocity reference (track: the twist's joint velocity while fresh; planned motions: the trajectory's) (NaN when not servoing)",
    "j<i>_acc_lim": "1 if the acceleration limit held back an increase of this joint's commanded speed this tick",
    "j<i>_e_mot": "q_cmd - q_mot: the error the motor's MIT loop sees",
    "j<i>_tau_est": "kp*e_mot + kd*(qd_ff - qd) + tau_g: torque the MIT law applies",
    "j<i>_tau_g": "gravity feedforward (whenever the motors are on)",
    "j<i>_status": "Damiao status code (1 enabled, 0 disabled, >= 8 fault)",
    "x..qw": "TCP pose from the output encoders, arm frame (quaternion x, y, z, w)",
    "cmd_x..z": "TCP position of q_cmd",
    "err_pos / err_rot": "measured TCP vs commanded TCP (output side)",
    "goal_err_pos / rot": "commanded TCP vs the goal of the source",
    "w / w_lin / sigma_min": "manipulability sqrt(det(J J^T)) of the 6x6 TCP Jacobian, of its translational rows, and its smallest singular value, at the measured pose",
    "limited": "the joint speed limit held q_cmd back this tick (the step limit)",
    "read_age": "age of the reading this tick used: 0 normally; > 0 when replies were missed and the latest complete reading was kept (fault beyond 50 ms with the motors on)",
    "read_misses": "ticks with missed replies since the node started (per motor in /state)",
    "twist_age": "s since the track twist feedforward was written (NaN if none)",
    "g_gap / g_gap_cmd / g_gap_goal": "jaw gap measured / sent this tick (after the force limit) / goal (m)",
    "g_force": "squeeze force applied, estimated: the gripper's MIT law plus the force-limit feedforward, over the pinion (N)",
    "g_force_demand": "squeeze force the MIT law alone asks for, before the force limit (N); above g_force_max while limiting",
    "g_tau_ff": "force-limit feedforward (opposes closing; |g_tau_ff| <= g_tau_est)",
    "J<r><c>": "jacobian channel: TCP Jacobian at the measured pose, arm-frame axes (LOCAL_WORLD_ALIGNED), row r of [vx vy vz wx wy wz], column = joint",
}


@dataclass
class Track:
    q_goal: np.ndarray
    pose: pin.SE3
    twist: Optional[np.ndarray]
    twist_frame: str
    t: float
    seq: int


@dataclass
class Motion:
    traj: Trajectory
    phase: str = "executing"
    reason: Optional[str] = None
    t_end: Optional[float] = None          # when it left executing
    reached_since: Optional[float] = None
    bad_since: Optional[float] = None
    accepted: float = field(default_factory=time.monotonic)

    def public(self, now: float) -> dict:
        tr = self.traj
        prog = 1.0 if now >= tr.t[-1] else max(0.0, (now - tr.t[0]) / max(tr.duration, 1e-9))
        return {"id": tr.id, "kind": tr.kind, "name": tr.name, "phase": self.phase, "reason": self.reason,
                "progress": round(prog, 4), "duration": round(tr.duration, 3),
                "t_start": tr.t[0], "t_end_planned": tr.t[-1],
                "goal": None if tr.goal is None else pose_public(tr.goal), "info": tr.info}


@dataclass
class JamConfig:
    """Jam detection: a STATIC arm still pushing (the aftermath of a contact, not the instant of it). Judged only
    while servoing with no motion executing (a finished move counts; a move still settling too), every joint
    slower than ``static_qd`` for ``settle_s``. A joint's residual |tau - tau_g| above its band for ``hold_s``
    fires; it clears below ``release`` x band."""
    bands: list = field(default_factory=lambda: [1.0, 4.0, 4.0, 1.5, 0.8, 0.6])   # Nm per joint (draft, offline study)
    hold_s: float = 1.0
    release: float = 0.7
    static_qd: float = 0.05          # rad/s: every joint slower than this counts as static
    settle_s: float = 0.5            # s static before judging
    response: str = "hold"           # hold: command pulled to the reading (a move ends `jammed`) | report | off
    refire: bool = True              # fire again after clearing (e.g. the base keeps pushing the arm in)

    def public(self) -> dict:
        return {"bands": list(self.bands), "hold_s": self.hold_s, "release": self.release, "static_qd": self.static_qd,
                "settle_s": self.settle_s, "response": self.response, "refire": self.refire}


JAM_RESPONSES = ("hold", "report", "off")


@dataclass
class Gripper:
    goal: Optional[float] = None       # m gap: the MIT position command (None while free)
    sent: Optional[float] = None       # m gap actually sent (the force limit may pull it back)
    max_force: float = C.DEFAULT_MAX_FORCE


def clip_increase(v_prev: np.ndarray, v_des: np.ndarray, dv: np.ndarray) -> np.ndarray:
    """``v_des`` with only increases of speed limited to ``dv`` per tick: slowing down (toward
    zero) is never held back; a reversal passes zero freely and only its part past zero is
    limited (to ``dv``, an increase from rest)."""
    same = v_des * v_prev > 0.0
    cap = np.where(same, np.abs(v_prev) + dv, dv)
    return np.sign(v_des) * np.minimum(np.abs(v_des), cap)


def scale_to(v: np.ndarray, cap) -> np.ndarray:
    """``v`` scaled down as a whole so no component exceeds ``cap`` (scalar or per component)."""
    k = np.max(np.abs(v) / cap) if np.any(v) else 0.0
    return v / k if k > 1.0 else v


def finite(v):
    """Non-finite floats (NaN, inf) to None, recursively: JSON has neither."""
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {k: finite(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [finite(x) for x in v]
    return v


class ArmNode:
    def __init__(self, bus_factory: Callable[[], ArmBus], profile: str = C.DEFAULT_PROFILE,
                 settings: Optional[C.NodeSettings] = None, hub: Optional[RowHub] = None,
                 locks: Optional[LockStore] = None, clock: Callable[[], float] = time.monotonic,
                 description_node: str = "description"):
        self.bus_factory = bus_factory
        self.description_node = description_node     # where the arm frame's pose in the robot base frame comes from
        self.clock = clock
        self.settings = settings or C.NodeSettings()
        self.profile = C.load_profile(profile)
        self.locks = locks or LockStore()
        self.lock_names = {"arm"}
        self.hub = hub or RowHub({"tick": len(FIELDS), "jacobian": len(JAC_FIELDS)})
        self.events = EventLog(on_add=self.hub.state_changed)
        self.lease = Lease(LEASE_HEARTBEAT, on_end=self._lease_ended, clock=clock)
        self.estop = EStop(on_trigger=self._estop_triggered)
        self.bus: Optional[ArmBus] = None
        self.kin: Optional[ArmKinematics] = None
        self._base = "connecting"          # connecting | uninitialized | initializing | ready | fault
        self.error: Optional[str] = None
        self.initialized = False
        self.enabled = False
        self.motors_on = False
        self.servo = False                 # arm closed loop
        self.g_servo = False               # gripper closed loop
        self.g_motor_on = False            # gripper motor enabled (off = damping)
        self._g_cut = False                # gripper motor off requested (disable)
        self.soft = C.DEFAULT_SOFT.copy()
        self._lock = threading.RLock()
        self._enable_req: Optional[dict] = None
        self._cut: Optional[str] = None    # motors off requested (why)
        self._reconnect = False
        self.q_cmd: Optional[np.ndarray] = None   # only while servoing
        self.v_cmd = np.zeros(N)                  # q_cmd's own velocity (for the acceleration limit)
        self.qd_ff = np.zeros(N)                  # MIT velocity reference sent
        self.acc_lim = np.zeros(N, dtype=bool)
        self._last_send = 0.0
        self._plan_lock = threading.Lock()
        self.track: Optional[Track] = None
        self.motion: Optional[Motion] = None
        self.hold_q: Optional[np.ndarray] = None
        self.last_motion: Optional[Motion] = None
        self.gripper = Gripper()
        self._g_engage = False
        self.reading = None
        self.latched: list[str] = []      # motor fault codes latched while the motors are off (enable clears them)
        self._fresh_t: Optional[float] = None   # when the latest complete reading was taken
        self.read_age = 0.0
        self.read_misses = 0
        self.misses_by_motor: dict = {}
        self._lat: deque = deque(maxlen=LATENCY_WINDOW)   # s, complete reads only
        self._lat_n = 0
        self._lat_reported = False
        self._lat_warn_t = -math.inf
        self._stale_warned = False          # this gap has been counted
        self._stale_gaps = 0                # gaps over STALE_WARN not yet reported
        self._stale_warn_t = -math.inf
        self.late_ticks = 0
        self.realtime = realtime.not_started(realtime.PRIO_BUS)   # set by the loop's thread
        self._tick = 0
        self._last_t: Optional[float] = None
        self.overruns = 0
        self._limited = False
        self._goal_pose: Optional[pin.SE3] = None
        self._g = {"tau_est": math.nan, "force": math.nan, "demand": math.nan, "ff": 0.0, "limiting": False, "tau_g": None}
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._connect_at = 0.0
        self.jam = JamConfig()
        self.jam_active: Optional[dict] = None    # the jam in force: joint, residual, since, response
        self.jams: list[dict] = []                # recent jams (last 20), newest last
        self.jam_count = 0
        self._static_since: Optional[float] = None
        self._over_since = np.full(N, np.nan)     # per joint: when its residual went over its band (NaN = under)
        self._jam_fired_cmd: Optional[int] = None # with refire off: the command it fired for (fires once per command)
        self._cmd_count = 0                       # every new command / motion (refire off: one jam per command)

    # -- state ---------------------------------------------------------------------------------------

    @property
    def state(self) -> str:
        b = self._base
        if b != "ready":
            return b
        if not self.motors_on:
            return "off"
        if not self.enabled:
            return "disabled"
        return "servoing" if self.servo else "floating"

    status = state

    # -- lifecycle -------------------------------------------------------------------------------

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="arm-control", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._close_bus()

    def _close_bus(self) -> None:
        """Best effort, step by step: a failing disable must never keep the serial port open."""
        if self.bus is not None:
            for step in (self.bus.disable, self.bus.close):
                try:
                    step()
                except Exception as e:  # noqa: BLE001
                    log.warning("closing the bus (%s): %s", step.__name__, e)
        self.bus, self.motors_on, self.g_motor_on = None, False, False

    def _run(self) -> None:
        self.realtime = realtime.make_realtime(realtime.PRIO_BUS, "arm control loop")
        self.events.add(realtime.describe(self.realtime), "info" if self.realtime["error"] is None else "warn")
        next_t = time.monotonic()
        while not self._stop.is_set():
            try:
                self.step()
            except Exception as e:  # noqa: BLE001 - never let the loop die silently
                log.exception("control tick")
                self._fault(f"control loop: {type(e).__name__}: {e}")
            next_t += DT
            delay = next_t - time.monotonic()
            if delay < -0.05:
                next_t = time.monotonic()
            elif delay > 0:
                time.sleep(delay)

    def step(self) -> None:
        """One iteration of the control thread: connect if needed, else one tick."""
        if self._reconnect:
            self._reconnect = False
            self._close_bus()
            self._base, self.initialized = "connecting", False
            self._connect_at = 0.0
        if self._base == "connecting":
            if self.clock() >= self._connect_at:
                self._connect()
            return
        if self.bus is None:
            return
        self.tick()

    def _connect(self) -> None:
        try:
            self.bus = self.bus_factory()
            try:
                self.bus.connect()
            except Exception:
                try:
                    self.bus.close()       # a failed connect may have opened the port: release it
                except Exception:  # noqa: BLE001
                    pass
                raise
            self.kin = self.kin or ArmKinematics(self.bus.setpoint_limits)
            Planner.warm(self.kin)          # start the planner process now, not on the first plan
            self._base, self.error = "uninitialized", None
            self._lat.clear()
            self._lat_reported = False      # report the new connection's reply latency
            self._last_t = None             # the tick clock starts afresh (no bogus "late" tick)
            self._fresh_t = None
            self.events.add("connected (watchdog and joint readings checked); enable runs the gripper check")
        except Exception as e:  # noqa: BLE001 - stay in connecting and retry
            self.bus = None
            msg = f"connect failed: {type(e).__name__}: {e}"
            if msg != self.error:
                self.events.add(f"{msg} (retrying every {CONNECT_RETRY_S:g} s)", "error")
            self.error = msg
            self._connect_at = self.clock() + CONNECT_RETRY_S

    # -- control tick ------------------------------------------------------------------------------

    def tick(self) -> None:
        now = self.clock()
        dt_tick = math.nan if self._last_t is None else now - self._last_t
        self._last_t = now
        if dt_tick > SLOW_TICK:
            self.overruns += 1
        if dt_tick > LATE_WARN:
            self.late_ticks += 1
            self.events.add(f"control tick {dt_tick * 1e3:.0f} ms late", "warn")
        stale = False
        try:
            r = self.bus.read()
            self.reading, self._fresh_t, self.read_age = r, now, 0.0
            self._stale_warned = False
            self._note_latency(now)
        except ReadMiss as e:
            self.read_misses += 1
            for m in e.missing:
                self.misses_by_motor[m] = self.misses_by_motor.get(m, 0) + 1
            age = math.inf if self._fresh_t is None else now - self._fresh_t
            limit = STALE_ON if self.motors_on else STALE_OFF
            if self.motors_on and STALE_WARN <= age <= limit and not self._stale_warned:
                self._stale_warn(now, age, e.missing)
            if self.reading is None or age > limit:
                self._fault(f"{e} (no complete reading for {age * 1e3:.0f} ms)")
                return
            r, stale, self.read_age = self.reading, True, age     # the latest complete reading, briefly
        except BusError as e:
            self._fault(str(e))
            return
        if self._base == "fault":
            self._publish(now, dt_tick, r)
            return
        faults = [] if stale else r.faults(self.bus.names)
        if not stale:
            self.latched = [] if self.motors_on else faults
        if faults and self.motors_on:
            self._fault("motor fault: " + "; ".join(faults))
            return
        ch = self.chocks()
        if ch and self.enabled:
            self._disable(f"CHOCKED ({describe_locks(ch)}): disabled (arm floating)")
        self.lease.expire_if_due()
        if self._cut is not None:
            why, self._cut = self._cut, None
            self._motors_off(why)
        if self._g_cut:
            self._g_cut = False
            if self.motors_on and self.g_motor_on and not self.enabled:
                self.bus.gripper_motor(False)
                self.g_motor_on = False
        req = self._enable_req
        if req is not None and req.get("result") is None:
            self._do_enable(req)
            return
        if self.motors_on:
            self._control(now, r)
        self._publish(now, dt_tick, r)

    def _stale_warn(self, now: float, age: float, missing: list) -> None:
        """Control thread: a gap in the readings is nearing the fault (STALE_ON): say so, at most
        every STALE_WARN_EVERY s."""
        self._stale_warned = True
        self._stale_gaps += 1
        if now - self._stale_warn_t < STALE_WARN_EVERY:
            return
        more = self._stale_gaps - 1
        self._stale_warn_t, self._stale_gaps = now, 0
        self.events.add(f"no complete reading for {age * 1e3:.0f} ms (no reply from "
                        f"{', '.join(dict.fromkeys(missing))}); with the motors on the arm faults at "
                        f"{STALE_ON * 1e3:.0f} ms" + (f" ({more} more since the last report)" if more else ""),
                        "warn")

    def _note_latency(self, now: float) -> None:
        """Control thread: keep the bus's reply latency; report it once after connecting, then warn
        when the slowest replies come close to the tick period."""
        lat = getattr(self.bus, "reply_latency", None)
        if lat is None:
            return
        self._lat.append(lat)
        self._lat_n += 1
        if not self._lat_reported:
            if len(self._lat) < LATENCY_FIRST:
                return
            self._lat_reported = True
            st = self.latency_stats()
            if st is None:
                return
            low = st["margin_ms"] < LATENCY_MARGIN_WARN * 1e3
            self.events.add(f"reply latency over the first {st['n']} complete reads: p50 {st['p50_ms']:.1f} ms, "
                            f"p99 {st['p99_ms']:.1f} ms, max {st['max_ms']:.1f} ms: {st['margin_ms']:.1f} ms margin "
                            f"to the {DT * 1e3:.0f} ms tick" + (" (too little: later replies count as missed)" if low else ""),
                            "warn" if low else "info")
            return
        if self._lat_n % LATENCY_WINDOW or now - self._lat_warn_t < LATENCY_WARN_EVERY:
            return
        st = self.latency_stats()
        if st is not None and st["margin_ms"] < LATENCY_MARGIN_WARN * 1e3:
            self._lat_warn_t = now
            self.events.add(f"reply latency p99 {st['p99_ms']:.1f} ms (max {st['max_ms']:.1f} ms) over the last "
                            f"{st['n']} complete reads: {st['margin_ms']:.1f} ms margin to the {DT * 1e3:.0f} ms tick; "
                            "later replies count as missed, and with the motors on "
                            f"{STALE_ON * 1e3:.0f} ms without a complete reading faults", "warn")

    def latency_stats(self) -> Optional[dict]:
        """Reply latency over the last LATENCY_WINDOW complete reads (ms), or None with too few."""
        try:
            a = np.array(list(self._lat), dtype=float) * 1e3
        except RuntimeError:          # appended to meanwhile
            return None
        if len(a) < 50:
            return None
        p50, p99 = np.percentile(a, [50, 99])
        return {"n": int(len(a)), "p50_ms": round(float(p50), 2), "p99_ms": round(float(p99), 2),
                "max_ms": round(float(a.max()), 2), "margin_ms": round(DT * 1e3 - float(p99), 2)}

    def _motors_off(self, why: str) -> None:
        """Control thread: motors disabled (damping)."""
        if self.motors_on:
            try:
                self.bus.disable()
            except Exception:  # noqa: BLE001
                pass
            self.motors_on = self.g_motor_on = False
            self.events.add(f"motors off (damping): {why}")

    def _check_encoders(self, r) -> None:
        """At enable, at rest: each joint's output encoder (q, what kinematics, IK and plans use)
        and its motor's feedback position (q_mot, what the MIT loop servos on) must agree to within
        backlash, or every command would land off by the difference (a full turn for a 2π wrap)."""
        names = self.bus.names
        diff = np.abs(np.append(r.q - r.q_mot, r.g_q - r.g_q_mot))
        for i in np.flatnonzero(diff > ENC_MISMATCH_ENABLE):
            qo = r.q[i] if i < N else r.g_q
            qm = r.q_mot[i] if i < N else r.g_q_mot
            raise RequestError(409, f"{names[i]}: output encoder {math.degrees(qo):+.1f}° and motor feedback "
                                    f"{math.degrees(qm):+.1f}° disagree by {math.degrees(diff[i]):.1f}°: "
                                    "power-cycle the arm; not enabling")

    def _limit_problem(self, q) -> Optional[str]:
        """Why servoing must not start from ``q`` (a joint further outside its limits than
        ENGAGE_LIMIT_TOL: the command would have to snap it back), or None."""
        kin = self.kin
        out = np.maximum(kin.lo - q, q - kin.hi)
        for i in np.flatnonzero(out > ENGAGE_LIMIT_TOL):
            return (f"joint{i + 1} is at {math.degrees(q[i]):+.1f}°, {math.degrees(out[i]):.1f}° outside its limits "
                    f"{math.degrees(kin.lo[i]):+.1f}..{math.degrees(kin.hi[i]):+.1f}°: move it back by hand first "
                    "(the arm floats while enabled)")
        return None

    def _do_enable(self, req: dict) -> None:
        try:
            if self.estop.active or self.chocks() or not self.lease.held:
                raise RequestError(409, "e-stop, chock or lease changed while enabling")
            if not self.motors_on:
                if not self.initialized:
                    self._base = "initializing"
                    self.hub.state_changed()
                    g = self.bus.check_gripper()
                    self.initialized = True
                    self.events.add(f"gripper check passed (closed reads {math.degrees(g):+.1f}°)")
                if req.get("cancel"):
                    raise RequestError(409, f"cancelled: {req['cancel']}")
                self._check_encoders(self.bus.read())    # before any motor is switched on
                self.bus.enable()
                with self._lock:
                    self.servo = False
                    self.q_cmd = None
                    self._release_gripper()
                self.motors_on = self.g_motor_on = True
                self._base = "ready"
                now = self.clock()
                self._last_t = self._last_send = now
                self._control(now, self.bus.read())      # the first (floating) frame right away
            if not self.g_motor_on:            # from disabled: the gripper was in damping
                self.bus.gripper_motor(True)
                self.g_motor_on = True
            self.enabled = True
            self._base = "ready"
            self.events.add("enabled: the arm floats (gravity compensated) until a command engages it")
            req["result"] = (True, None, 200)
        except Exception as e:  # noqa: BLE001
            msg = f"{type(e).__name__}: {e}" if not isinstance(e, RequestError) else str(e)
            if not self.motors_on:
                try:
                    self.bus.disable()
                except Exception:  # noqa: BLE001
                    pass
            if self._base == "initializing":
                self._base = "ready" if self.initialized else "uninitialized"
            self.events.add(f"enable failed: {msg}", "error")
            req["result"] = (False, msg, e.status if isinstance(e, RequestError) else 503)
        finally:
            with self._lock:
                ev = req.get("event")
            if ev is not None:
                ev.set()

    # -- the command -----------------------------------------------------------------------------------

    def _engage(self, r) -> bool:
        """Start servoing from the reading (call under _lock): the command starts at the motor-side
        positions, moving with the measured velocity (clipped), and is braked from there."""
        why = self._limit_problem(r.q_mot)
        if why:
            self._drop_sources(f"not engaged: {why}")
            self.events.add(f"arm not engaged: {why}", "warn")
            return False
        # the command starts moving as the arm does: speeding up from there is acceleration
        # limited, slowing down is not
        self._command_from_reading(r, np.clip(r.qd, -self.soft.joint_vel, self.soft.joint_vel))
        self.servo = True
        self.events.add("arm engaged (servoing from the measured pose)")
        return True

    def _command_from_reading(self, r, v: np.ndarray) -> None:
        """The command becomes the arm's motor-side positions (the MIT loops run on those) and the
        driver is told so: it judges each setpoint against the last one it sent, and would
        otherwise refuse a pose outside the limits and resend an old command instead. The only way
        the command is ever replaced by where the arm is (call under _lock, control thread)."""
        self.q_cmd = r.q_mot.copy()
        self.v_cmd = np.asarray(v, float).copy()
        self.hold_q = None
        self.bus.seed(self.q_cmd)

    def _command_step(self, q_goal: np.ndarray) -> None:
        """Move q_cmd one tick toward the (literal) goal (call under _lock): at most the joint speed
        limit (the step limit); increases of speed limited by the acceleration limit (slowing down
        never is: see clip_increase); never further outside the joint limits."""
        kin, s = self.kin, self.soft
        # the limits, widened to include q_cmd: a joint engaged just outside them is never pushed
        # further out
        lo, hi = np.minimum(kin.lo, self.q_cmd), np.maximum(kin.hi, self.q_cmd)
        want = (np.clip(q_goal, lo, hi) - self.q_cmd) / DT
        v_des = np.clip(want, -s.joint_vel, s.joint_vel)
        self._limited = bool(np.any(np.abs(want) > s.joint_vel + 1e-12))
        v = clip_increase(self.v_cmd, v_des, s.joint_acc * DT)
        self.acc_lim = np.abs(v) < np.abs(v_des) - 1e-12
        # clipped: exact in arithmetic, but rounding can land a hair past a limit, and the driver
        # refuses any setpoint past one (home is exactly on joints 2 and 3's upper limits)
        self.q_cmd = np.clip(self.q_cmd + v * DT, lo, hi)
        self.v_cmd = v

    def _control(self, now: float, r) -> None:
        """One frame per motor (motors on). Arm: servoing (profile gains) or floating (kp = kd = 0);
        gravity feedforward either way. Gripper: holding its goal (with the force limit) or free."""
        kin, p = self.kin, self.profile
        src = None
        with self._lock:
            if self._has_source() and self.enabled and not self.servo and not self.estop.active:
                self._engage(r)
            if self.servo:
                src, q_goal, vel_ref, gap_goal = self._source(now)
                self._command_step(q_goal)
                self.qd_ff = scale_to(vel_ref, self.soft.joint_vel)
                q_cmd, qd_ff = self.q_cmd.copy(), self.qd_ff.copy()
            else:
                gap_goal = None
                q_cmd, qd_ff = r.q_mot.copy(), np.zeros(N)
                self.acc_lim = np.zeros(N, dtype=bool)
            g = self.gripper
            if gap_goal is not None and self.g_servo:
                g.goal = gap_goal
            if self._g_engage and self.enabled and self.g_motor_on:
                self.g_servo = True
            self._g_engage = False
            g_goal, max_force, g_on = g.goal, g.max_force, self.g_servo
        tau_g = kin.gravity(r.q) * p.tau_scale
        kp, kd = (p.kp, p.kd) if self.servo else (np.zeros(N), np.zeros(N))
        if g_on:
            # Gripper: MIT position at the goal, velocity reference 0 (the damping term never adds
            # grip force). Force limit from the torque its MIT law applies (motor side): away from
            # fully open an opposing feedforward, never larger than that torque (the net torque never
            # opens); within FF_OPEN_MARGIN of fully open no feedforward, the position command is
            # pulled back toward the jaws instead.
            g_q = C.gap_to_rad(g_goal)
            tau_est = p.g_kp * (g_q - r.g_q_mot) + p.g_kd * (0.0 - r.g_qd)
            force = C.force_of_torque(max(tau_est, 0.0))
            tau_lim = C.torque_of_force(max_force)
            g_ff, limiting = 0.0, False
            if force > max_force:
                limiting = True
                if C.rad_to_gap(r.g_q) > C.GAP_MAX - C.FF_OPEN_MARGIN:
                    g_q = min(g_q, r.g_q_mot + tau_lim / p.g_kp)          # closing is +rad
                else:
                    g_ff = -(tau_est - tau_lim)                          # opposes closing; |g_ff| < tau_est
            g_kp, g_kd = p.g_kp, p.g_kd
            g.sent = C.rad_to_gap(g_q)
            # what is applied: the MIT law with the position actually sent, plus the feedforward
            applied = C.force_of_torque(max(g_kp * (g_q - r.g_q_mot) + g_kd * (0.0 - r.g_qd) + g_ff, 0.0))
        else:
            g_q, tau_est, force, g_ff, limiting, g_kp, g_kd = r.g_q_mot, math.nan, math.nan, 0.0, False, 0.0, 0.0
            applied = math.nan
        self._g = {"tau_est": tau_est, "force": applied, "demand": force, "ff": g_ff, "limiting": limiting, "tau_g": tau_g}
        try:
            self.bus.send(q_cmd, qd_ff, kp, kd, tau_g, g_q, 0.0, g_kp, g_kd, g_ff, gripper=self.g_motor_on)
        except BusError as e:
            self._fault(str(e))
            return
        self._last_send = now
        if self.servo:
            self._supervise(now, r, src)
            self._jam_check(now, r, src, tau_g)
        else:
            self._jam_reset()

    def _has_source(self) -> bool:
        m = self.motion
        return self.track is not None or (m is not None and m.phase in ("executing", "settling"))

    def _source(self, now: float):
        """(source, goal, MIT velocity reference, gripper gap goal or None); servoing, under _lock.
        Track: the goal is the IK of the pose, literally; a fresh twist (TWIST_TIMEOUT) gives the
        velocity reference. Planned motions: the trajectory's position and velocity at ``now``."""
        m = self.motion
        if m is not None and m.phase in ("executing", "settling"):
            q, qd, gap, done = m.traj.sample(now)
            if done and m.phase == "executing":
                m.phase, m.t_end = "settling", now
                if m.traj.kind == "chunk":
                    self._end_motion("done")
            if gap is not None and not self.g_servo:
                gap = None                           # a chunk's gripper samples need it holding
            self._goal_pose = m.traj.goal
            return m.traj.kind, q, qd, gap
        t = self.track
        if t is not None:
            qd = np.zeros(N)
            if t.twist is not None and now - t.t <= TWIST_TIMEOUT:
                qd = self.kin.twist_to_qd(t.q_goal, t.twist, t.twist_frame)
            self._goal_pose = t.pose
            return "track", t.q_goal, qd, None
        self._goal_pose = None
        return "hold", (self.q_cmd if self.hold_q is None else self.hold_q), np.zeros(N), None

    def _supervise(self, now: float, r, src) -> None:
        """Planned motions: abort on a large tracking error; reached / not_reached when settling."""
        m = self.motion
        if m is None or m.phase not in ("executing", "settling"):
            return
        s = self.settings
        dp, dr = pose_error(self.kin.fk(r.q), self.kin.fk(self.q_cmd))
        if dp > s.abort_pos or dr > s.abort_rot:
            m.bad_since = m.bad_since or now
            if now - m.bad_since >= s.abort_hold_s:
                with self._lock:   # hold where the arm is, not where it was told to be
                    self._command_from_reading(r, np.zeros(N))
                self._end_motion("aborted", f"tracking error {dp * 1e3:.0f} mm / {math.degrees(dr):.0f}° "
                                            "(obstructed?): holding where it is")
                return
        else:
            m.bad_since = None
        if m.phase == "settling" and m.traj.goal is not None:
            ep, er = pose_error(self.kin.fk(r.q), m.traj.goal)
            if ep <= s.reach_pos and er <= s.reach_rot:
                m.reached_since = m.reached_since or now
                if now - m.reached_since >= s.reach_dwell_s:
                    self._end_motion("reached")
                    if m.traj.kind == "stow":
                        self._stowed("stowed: at home")
            else:
                m.reached_since = None
                if now - m.t_end > s.settle_timeout_s:
                    self._end_motion("not_reached", f"{ep * 1e3:.1f} mm / {math.degrees(er):.2f}° from the goal "
                                                    f"{s.settle_timeout_s:g} s after the planned end: holding")

    # -- jam detection: a static arm still pushing ----------------------------------------------------

    def _jam_reset(self) -> None:
        self._static_since = None
        self._over_since[:] = np.nan
        self.jam_active = None

    def _jam_check(self, now: float, r, src, tau_g) -> None:
        """Control thread, while servoing. See JamConfig. Planned motions and holds get the response; track (a
        teleop stream overwrites the command every tick) is report-only."""
        cfg = self.jam
        if cfg.response == "off" or tau_g is None:
            self._jam_reset()
            return
        m = self.motion
        moving = (m is not None and m.phase == "executing") or bool(np.any(np.abs(r.qd) > cfg.static_qd))
        if moving:
            self._static_since = None
            self._over_since[:] = np.nan
            return
        self._static_since = self._static_since or now
        if now - self._static_since < cfg.settle_s:
            return
        res = np.asarray(r.tau, float) - np.asarray(tau_g, float)
        bands = np.asarray(cfg.bands, float)
        if self.jam_active is not None:                       # hysteresis: clear only well under every band
            if np.all(np.abs(res) < cfg.release * bands):
                self.events.add(f"jam cleared (joint{self.jam_active['joint']})")
                self.jam_active = None
                self._over_since[:] = np.nan
            return
        over = np.abs(res) > bands
        self._over_since = np.where(over, np.where(np.isnan(self._over_since), now, self._over_since), np.nan)
        held = np.where(over, now - self._over_since, 0.0)
        if not np.any(held >= cfg.hold_s):
            return
        if not cfg.refire and self._jam_fired_cmd == self._cmd_count:
            return
        i = int(np.argmax(np.where(held >= cfg.hold_s, np.abs(res) / bands, 0.0)))
        track = src == "track"
        act = "report" if (cfg.response == "report" or track) else "hold"
        jam = {"t": now, "wall": time.time(), "joint": i + 1, "residual_nm": round(float(res[i]), 3),
               "band_nm": float(bands[i]), "held_s": round(float(held[i]), 2), "response": act, "source": src,
               "residuals_nm": [round(float(v), 3) for v in res]}
        self.jam_count += 1
        jam["count"] = self.jam_count
        self.jam_active, self._jam_fired_cmd = jam, self._cmd_count
        self.jams = (self.jams + [jam])[-20:]
        what = (f"jam: joint{i + 1} residual {res[i]:+.1f} Nm (band {bands[i]:g}) for {held[i]:.1f} s while static")
        if act == "hold":
            with self._lock:     # stop pushing: hold where the arm is, not where it was told to be
                self._command_from_reading(r, np.zeros(N))
            if self.motion is not None:
                self._end_motion("jammed", what + ": holding where it is")
            else:
                self.events.add(what + ": holding where it is", "warn")
        else:
            self.events.add(what + (" (track: report only)" if track else " (report only)"), "warn")

    def jam_state(self) -> dict:
        return finite({"config": self.jam.public(), "active": self.jam_active, "count": self.jam_count,
                       "recent": list(self.jams)})

    def set_jam(self, changes: dict) -> dict:
        """Change the jam detector (the server allows it for a human only)."""
        cfg = self.jam
        new = JamConfig(**{**cfg.public(), **{k: v for k, v in changes.items() if v is not None}})
        if len(new.bands) != N or not all(math.isfinite(b) and b > 0 for b in new.bands):
            raise ValueError(f"bands: {N} positive numbers (Nm, one per joint)")
        if new.response not in JAM_RESPONSES:
            raise ValueError(f"response: one of {', '.join(JAM_RESPONSES)}")
        if not (0 < new.release < 1) or new.hold_s <= 0 or new.settle_s < 0 or new.static_qd <= 0:
            raise ValueError("release in (0, 1); hold_s > 0; settle_s >= 0; static_qd > 0")
        self.jam = new
        self.events.add("jam detection: " + ", ".join(f"{k}={v}" for k, v in changes.items() if v is not None))
        return self.jam_state()

    def _end_motion(self, phase: str, reason: Optional[str] = None) -> None:
        with self._lock:
            m = self.motion
            if m is None:
                return
            m.phase, m.reason = phase, reason
            self.last_motion, self.motion = m, None
            if self.servo:
                self.hold_q = (m.traj.q[-1].copy() if phase in ("done", "reached", "not_reached")
                               else self.q_cmd.copy())
        self.events.add(f"{m.traj.kind} {m.traj.id}: {phase}" + (f" ({reason})" if reason else ""),
                        "warn" if phase in ("aborted", "not_reached") else "info")

    def _drop_sources(self, why: str) -> None:
        """No arm source any more (call under _lock)."""
        self.track = None
        if self.motion is not None:
            m = self.motion
            m.phase, m.reason = "aborted", why
            self.last_motion, self.motion = m, None

    def _publish(self, now: float, dt_tick: float, r) -> None:
        self._tick += 1
        kin = self.kin
        nan = math.nan
        info = self.lease.info
        with self._lock:
            servo = self.servo
            q_cmd = self.q_cmd.copy() if (servo and self.q_cmd is not None) else None
            v_cmd = self.qd_ff.copy()
            acc_lim = self.acc_lim.copy()
            m = self.motion or self.last_motion
            src = self._source_name()
            tr = self.track
            g = self.gripper
            g_on = self.g_servo
            seq = tr.seq if tr else 0
            goal = self._goal_pose if servo else None
        on = self.motors_on
        row = np.full(len(FIELDS), nan)
        row[:len(_BASE)] = [now, self._tick, STATES.index(self.state), float(self.enabled), float(on),
                            float(servo), float(g_on), float(self.estop.active), float(info.lease_id if info else 0),
                            SOURCES.index(src), seq, self.overruns, dt_tick, float(self._limited and servo),
                            (now - tr.t) if tr and tr.twist is not None else nan, self.read_age, self.read_misses]
        tau_g = self._g.get("tau_g") if on else None
        p = self.profile
        for i in range(N):
            b = len(_BASE) + i * len(JOINT_COLS)
            tg = tau_g[i] if tau_g is not None else nan
            if q_cmd is not None:
                qc, vc = q_cmd[i], v_cmd[i]
                e = qc - r.q_mot[i]
                te = p.kp[i] * e + p.kd[i] * (vc - r.qd[i]) + tg
                al = float(acc_lim[i])
            else:
                qc = vc = e = al = nan
                te = tg
            row[b:b + len(JOINT_COLS)] = [r.q[i], r.q_mot[i], r.qd[i], r.tau[i], qc, vc, tg, e, te, al,
                                          r.t_mos[i], r.status[i]]
        T = kin.fk(r.q)
        J = kin.jacobian(r.q, "arm")
        mn = kin.manipulability(J)
        quat = pin.Quaternion(T.rotation)
        b = COL["x"]
        row[b:b + 7] = [*T.translation, quat.x, quat.y, quat.z, quat.w]
        if q_cmd is not None:
            Tc = kin.fk(q_cmd)
            row[COL["cmd_x"]:COL["cmd_x"] + 3] = Tc.translation
            row[COL["err_pos"]], row[COL["err_rot"]] = pose_error(T, Tc)
            if goal is not None:
                row[COL["goal_err_pos"]], row[COL["goal_err_rot"]] = pose_error(Tc, goal)
        row[COL["w"]], row[COL["w_lin"]], row[COL["sigma_min"]] = mn["w"], mn["w_lin"], mn["sigma_min"]
        gd = self._g
        gl = on and g_on
        row[COL["g_gap"]:COL["g_gap"] + len(_GRIP)] = [
            C.rad_to_gap(r.g_q), g.sent if (gl and g.sent is not None) else nan,
            g.goal if (gl and g.goal is not None) else nan, r.g_qd, r.g_tau,
            gd["tau_est"] if gl else nan, gd["force"] if gl else nan, gd["demand"] if gl else nan,
            gd["ff"] if gl else nan, g.max_force,
            float(gd["limiting"] and gl), r.status[N]]
        if m is not None:
            mp = m.public(now)
            row[COL["motion_id"]], row[COL["phase"]], row[COL["progress"]] = m.traj.id, PHASES.index(m.phase), mp["progress"]
        else:
            row[COL["phase"]] = 0
        row[COL["jam"]] = float(self.jam_active is not None)
        self.hub.publish_rows(row[None], "tick")
        self.hub.publish_rows(np.concatenate([[now], J.reshape(-1)])[None], "jacobian")

    def _source_name(self) -> str:
        if not self.servo:
            return "none"
        m = self.motion
        if m is not None and m.phase in ("executing", "settling"):
            return m.traj.kind
        return "track" if self.track is not None else "hold"

    # -- safety ------------------------------------------------------------------------------------

    def chocks(self) -> list:
        return self.locks.matching("chock", self.lock_names)

    def _release_arm(self, why: str) -> None:
        """Arm out of servoing (floats if the motors are on). Call under _lock."""
        self._drop_sources(why)
        self.servo = False
        self.q_cmd = None
        self.hold_q = None
        self.v_cmd = np.zeros(N)

    def _release_gripper(self) -> None:
        self.g_servo = False
        self._g_engage = False
        self.gripper.goal = self.gripper.sent = None
        self.gripper.max_force = C.DEFAULT_MAX_FORCE

    def _disable(self, why: str) -> None:
        """Not enabled: the arm floats, the gripper goes into damping (its motor off, next tick),
        commands are refused."""
        was = self.enabled
        with self._lock:
            self.enabled = False
            self._release_arm(why)
            self._release_gripper()
            self._g_cut = True
            req = self._enable_req
            if req is not None and req.get("result") is None:
                req["cancel"] = why
        if was:
            self.events.add(why)

    def _off(self, why: str) -> None:
        """Motors off (damping), from any thread: disabled now, the bus switched off next tick."""
        self._disable(why)
        self._cut = why

    def _fault(self, msg: str) -> None:
        self._disable(f"FAULT: {msg}")
        if self.bus is not None and self.motors_on:
            try:
                self.bus.disable()
            except Exception:  # noqa: BLE001
                pass
        self.motors_on = False
        if self._base != "fault":
            self.events.add(f"fault: {msg}: motors off", "error")
        self._base, self.error = "fault", msg

    def _lease_ended(self, reason: str) -> None:
        self.soft = C.DEFAULT_SOFT.copy()
        self._disable(f"lease {reason}: disabled (the arm floats)")
        self.events.add(f"lease {reason}")

    def _estop_triggered(self, info: EStopInfo) -> None:
        self._off(f"E-STOP ({info.source}: {info.reason})")

    def _stowed(self, why: str) -> None:
        self._off(why)

    # -- requests (any thread) -----------------------------------------------------------------------

    def acquire(self, client: str):
        token, info = self.lease.acquire(client)
        self.events.add(f"lease {info.lease_id} acquired by {info.client}")
        return token, info

    def release(self, token) -> None:
        self.lease.release(token)

    def renew(self, token):
        return self.lease.renew(token)

    def _ready(self, token) -> None:
        self.lease.renew(token)
        if self.estop.active:
            raise RequestError(409, "e-stop is latched")
        if not self.enabled or not self.motors_on:
            raise RequestError(409, f"not enabled ({self.state}; POST /enable)")

    def _arm_start(self) -> np.ndarray:
        """Where the arm is, for a new arm command to start from: the latest reading (output
        encoders), always, servoing or not; never a past command. Refuses (409) when servoing could
        not start from there. Under _lock."""
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        why = self._limit_problem(r.q_mot)
        if why:
            raise RequestError(409, why)
        return r.q.copy()

    def enable(self, token, timeout: float = ENABLE_TIMEOUT) -> dict:
        """Blocking (call from a worker thread): motors on (the first time after connecting, the
        gripper check first); the arm floats until a command engages it."""
        self.lease.renew(token)
        ch = self.chocks()
        if ch:
            raise RequestError(423, f"chocked: {describe_locks(ch)}. An admin removes it on the robot: "
                                    "python -m robot_utils.admin.locks remove <id>")
        if self.estop.active:
            raise RequestError(409, "e-stop is latched; clear it first (POST /estop/clear)")
        if self._base in ("connecting", "fault") or self.bus is None:
            raise RequestError(503, f"arm not ready ({self.state}: {self.error}); POST /reset after a fault")
        if self.enabled and self.motors_on:
            return {"enabled": True, "state": self.state, "initialized": self.initialized}
        ev = threading.Event()
        with self._lock:
            req = self._enable_req
            if req is None or req.get("result") is not None:
                req = self._enable_req = {"event": ev, "result": None}
            else:
                ev = req["event"]
        if not ev.wait(timeout):
            raise RequestError(503, "enable did not finish in time (is the control loop running?)")
        ok, msg, status = req["result"]
        if not ok:
            raise RequestError(status, f"enable failed: {msg}")
        return {"enabled": True, "state": self.state, "initialized": self.initialized}

    def disable(self, token) -> dict:
        self.lease.renew(token)
        self._disable("disabled: the arm floats, the gripper is in damping")
        return {"enabled": False, "state": self.state}

    def float_arm(self, token) -> dict:
        """Enabled, arm out of servoing (floating); the gripper keeps what it is doing."""
        self._ready(token)
        with self._lock:
            was = self.servo
            self._release_arm("floated")
        if was:
            self.events.add("arm floating (gravity compensated, no servoing)")
        return {"state": self.state}

    def trigger_estop(self, reason: str, source: str) -> EStopInfo:
        return self.estop.trigger(reason, source)

    def clear_estop(self, token) -> None:
        self.lease.renew(token)
        if self.estop.active:
            self.estop.clear()
            self.events.add("e-stop cleared (motors stay off: POST /enable)")

    def reset(self, token) -> None:
        self.lease.renew(token)
        self._disable("reset: reconnecting")
        self.error = None
        self._reconnect = True

    # motion ---------------------------------------------------------------------------------------

    def _preempt(self, by: str) -> None:
        """Call under _lock."""
        if self.motion is not None:
            m = self.motion
            m.phase, m.reason = "preempted", f"by {by}"
            self.last_motion, self.motion = m, None
            self.events.add(f"{m.traj.kind} {m.traj.id}: preempted by {by}")

    def track_cmd(self, token, pose: pin.SE3, twist=None, twist_frame: str = "arm", seq: int = 0,
                  gripper: Optional[dict] = None) -> dict:
        self._ready(token)
        self._cmd_count += 1
        if twist is not None:
            twist = np.asarray(twist, dtype=float)
            if twist.shape != (6,) or not np.all(np.isfinite(twist)):
                raise ValueError("twist must be 6 finite numbers [vx, vy, vz, wx, wy, wz]")
        twist_frame = "arm" if twist_frame == "base" else twist_frame      # "base": the old name of the arm frame
        if twist_frame not in ("arm", "tcp"):
            raise ValueError("twist_frame must be arm or tcp")
        scaled = False
        if twist is not None:          # bounded by the soft servo speeds
            lin, ang = float(np.linalg.norm(twist[:3])), float(np.linalg.norm(twist[3:]))
            if lin > self.soft.lin_vel:
                twist[:3] *= self.soft.lin_vel / lin
                scaled = True
            if ang > self.soft.ang_vel:
                twist[3:] *= self.soft.ang_vel / ang
                scaled = True
        g_args = self._check_gripper(**gripper) if gripper is not None else None
        with self._lock:
            prev = self.track if self.servo else None
            seed = self._arm_start()                 # the measured pose: IK seed and jump reference
        if prev is not None and seq and prev.seq and seq < prev.seq:
            return {"ok": False, "seq": seq, "ik": None, "limited": self._limited,
                    "reason": f"stale: seq {seq} is older than {prev.seq} already applied"}
        r = self.kin.ik(pose, seed, max_iter=TRACK_IK_ITER)
        out = {"ok": r.ok, "seq": seq, "ik": r.public(), "limited": self._limited, "twist_scaled": scaled,
               "reason": r.reason}
        if r.ok:
            jump = float(np.max(np.abs(r.q - seed)))
            if jump > TRACK_MAX_JUMP:
                out.update(ok=False, reason=f"too far for track: a joint is {math.degrees(jump):.0f}° from where it is "
                                            f"(max {math.degrees(TRACK_MAX_JUMP):.0f}°); use servo for large moves")
        if not out["ok"]:
            return out
        with self._lock:
            if not self.enabled:
                raise RequestError(409, "disabled while solving")
            self._preempt("track")
            self.hold_q = None
            self.track = Track(r.q, pose, twist, twist_frame, self.clock(), int(seq))
            if g_args is not None:
                self._apply_gripper(*g_args)
        return out

    @staticmethod
    def _check_gripper(gap_mm: Optional[float] = None, max_force_n: Optional[float] = None):
        """Validated (gap m or None, max force N or None). Raises ValueError before anything changes."""
        f = None
        if max_force_n is not None:
            f = float(max_force_n)
            if not (math.isfinite(f) and 0.0 <= f <= C.HARD_MAX_FORCE):
                raise ValueError(f"max_force_n must be within 0..{C.HARD_MAX_FORCE:g} N")
        gap = None
        if gap_mm is not None:
            gap = float(gap_mm) / 1000.0
            if not (math.isfinite(gap) and 0.0 <= gap <= C.GAP_MAX + 1e-9):
                raise ValueError(f"gap_mm must be within 0..{C.GAP_MAX * 1000:.1f}")
        return gap, f

    def _apply_gripper(self, gap: Optional[float], max_force: Optional[float]) -> None:
        """Call under _lock. A gap makes the gripper hold it (from the next tick); the force limit
        is the one given, or the default when omitted."""
        self.gripper.max_force = C.DEFAULT_MAX_FORCE if max_force is None else max_force
        if gap is not None:
            self.gripper.goal = gap
            if not self.g_servo:
                self._g_engage = True

    def gripper_cmd(self, token, gap_mm: Optional[float] = None, max_force_n: Optional[float] = None) -> dict:
        self._ready(token)
        args = self._check_gripper(gap_mm, max_force_n)
        with self._lock:
            self._apply_gripper(*args)
            return self.gripper_state()

    def _start(self, traj: Trajectory, by: str, q0: np.ndarray) -> dict:
        """Put a planned motion in force. It was planned from ``q0`` (a reading): if the arm moved
        more than PLAN_DRIFT since (latest reading), a servo / pose / home / stow is not started rather
        than started from a stale pose (409, retryable); a chunk's lead-in is re-anchored on where the arm is now
        (latest reading) and its speed checked."""
        self._cmd_count += 1
        with self._lock:
            if not self.enabled:
                raise RequestError(409, "disabled while planning")
            now = self.clock()
            here = self._arm_start()
            if traj.kind == "chunk":
                k = int(np.searchsorted(traj.t[1:], now + max(traj.info.get("blend_s", 0.0), DT), side="left")) + 1
                if k >= len(traj.t):
                    raise RequestError(422, "not started (nothing moved): the chunk was over by the time it was planned",
                                       extra={"kind": "plan", "code": "chunk_stale", "retryable": True,
                                              "hint": "send it again with a later t0 (GET /time)"})
                keep = np.r_[0, np.arange(k, len(traj.t))]
                traj.t, traj.q, traj.qd = traj.t[keep], traj.q[keep], traj.qd[keep]
                if traj.gap is not None:
                    traj.gap = traj.gap[keep]
                traj.q[0], traj.t[0], traj.qd[0] = here, now, 0.0
                if len(traj.t) > 1:
                    v = np.abs(traj.q[1] - traj.q[0]) / max(traj.t[1] - traj.t[0], 1e-6)
                    if np.any(v > self.soft.joint_vel):
                        j = int(np.argmax(v / self.soft.joint_vel))
                        raise RequestError(422, f"not started (nothing moved): the chunk's first sample is too far from "
                                                f"the arm for its blend (joint{j + 1} at {v[j]:.2f} rad/s)",
                                           extra={"kind": "plan", "code": "chunk_blend", "retryable": True,
                                                  "hint": "increase blend_s or start the chunk nearer the arm"})
            else:
                moved = float(np.max(np.abs(here - q0)))
                if moved > PLAN_DRIFT:
                    raise RequestError(409, f"not started (nothing moved): the arm moved {math.degrees(moved):.1f}° "
                                            "while planning (a motion in force, or moved by hand while floating); "
                                            "send the same request again",
                                       extra={"kind": "plan", "code": "moved_while_planning", "retryable": True})
                traj.shift(now - traj.t[0])
            self._preempt(by)
            self.track = None
            self.hold_q = None
            self.motion = Motion(traj)
            out = self.motion.public(now)
        self.events.add(f"{traj.kind} {traj.id}: {traj.duration:.2f} s" + (f" to {traj.name}" if traj.name else ""))
        return out

    def _plan(self, job: str, *args, **kwargs):
        """Plans run in the planner process (planner.py), one at a time: in this process they
        would hold the interpreter long enough to starve the control loop."""
        if not self._plan_lock.acquire(blocking=False):
            raise RequestError(409, "already planning another motion; retry")
        try:
            return Planner.run(job, self.kin, *args, **kwargs)
        except PlanError as e:
            raise RequestError(422, f"not planned (nothing moved): {e}", extra=getattr(e, "extra", None))
        finally:
            self._plan_lock.release()

    def servo_cmd(self, token, pose: Optional[pin.SE3] = None, name: Optional[str] = None,
                  lin_vel=None, lin_acc=None, ang_vel=None, ang_acc=None, duration=None,
                  max_rot_deg: Optional[float] = None) -> dict:
        """Blocking (planning): call from a worker thread."""
        self._ready(token)
        h, s = self.settings.hard, self.soft
        lim = {}
        for k, v in (("lin_vel", lin_vel), ("lin_acc", lin_acc), ("ang_vel", ang_vel), ("ang_acc", ang_acc)):
            cap = getattr(s, k)
            if v is not None and not (math.isfinite(float(v)) and 0 < float(v) <= getattr(h, k)):
                raise ValueError(f"{k} must be within 0..{getattr(h, k):g}")
            lim[k] = min(float(v), cap) if v is not None else cap
        if name is not None and name not in self.settings.named_poses:
            raise RequestError(422, f"unknown named pose {name!r} (have: {', '.join(self.settings.named_poses)})")
        with self._lock:
            q0 = self._arm_start()
        if name is not None:
            traj = self._plan("joint", q0, [self.settings.named_poses[name]], s.home_vel, s.joint_vel,
                              kind="pose", name=name)
        else:
            rot_tol = None if max_rot_deg is None else math.radians(max_rot_deg)
            traj = self._plan("servo", q0, pose, joint_vel=s.joint_vel, duration=duration, rot_tol=rot_tol, **lim)
        return self._start(traj, "servo", q0)

    def _home_traj(self, kind: str):
        st = self.settings
        with self._lock:
            q0 = self._arm_start()
        way = [st.named_poses[n] for n in st.home_via] + [st.named_poses["home"]]
        return q0, self._plan("joint", q0, way, self.soft.home_vel, self.soft.joint_vel, kind=kind, name="home")

    def home(self, token) -> dict:
        self._ready(token)
        q0, traj = self._home_traj("home")
        return self._start(traj, "home", q0)

    def stow(self, token) -> dict:
        """Safe homing, then motors off (damping) at home. Already within STOW_NEAR of home on
        every joint (measured): motors off at once."""
        self._ready(token)
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        home = self.settings.named_poses["home"]
        if np.all(np.abs(r.q - home) <= STOW_NEAR):
            self._stowed("stowed: already at home")
            return {"stowed": True, "motion": None}
        q0, traj = self._home_traj("stow")
        return {"stowed": False, "motion": self._start(traj, "stow", q0)}

    def chunk(self, token, poses: list, dt: float, t0: Optional[float] = None, gaps_mm=None,
              blend_s: float = 0.1) -> dict:
        self._ready(token)
        if not 0 < len(poses) <= 1000:
            raise ValueError("a chunk has 1..1000 poses")
        if gaps_mm is not None and len(gaps_mm) != len(poses):
            raise ValueError("gaps_mm must have one value per pose")
        if gaps_mm is not None and not all(math.isfinite(g) and 0.0 <= g / 1000 <= C.GAP_MAX + 1e-9 for g in gaps_mm):
            raise ValueError(f"gaps_mm must be within 0..{C.GAP_MAX * 1000:.1f}")
        if not 0.0 <= blend_s <= 1.0:
            raise ValueError("blend_s must be within 0..1")
        if t0 is not None and not math.isfinite(t0):
            raise ValueError("t0 must be finite")
        with self._lock:
            q_now = self._arm_start()
            if gaps_mm is not None:
                self._apply_gripper(gaps_mm[0] / 1000.0, None)
        now = self.clock()
        traj = self._plan("chunk", q_now, poses, None if gaps_mm is None else [g / 1000 for g in gaps_mm],
                          t0=now if t0 is None else float(t0), dt=dt, now=now, blend_s=blend_s,
                          joint_vel=self.soft.joint_vel, q_now=q_now)
        return self._start(traj, "chunk", q_now)

    def cancel(self, token) -> dict:
        """End the motion or track: servoing holds where its command is (floating: nothing to do)."""
        self._ready(token)
        with self._lock:
            m = self.motion
            self.track = None
            if self.servo:
                self.hold_q = self.q_cmd.copy()
            if m is not None:
                m.phase, m.reason = "aborted", "cancelled: holding"
                self.last_motion, self.motion = m, None
        if m is not None:
            self.events.add(f"{m.traj.kind} {m.traj.id}: cancelled")
        return {"cancelled": m is not None}

    # settings --------------------------------------------------------------------------------------

    def set_profile(self, token, name: str) -> dict:
        self.lease.renew(token)
        if self.servo or self.g_servo:
            raise RequestError(409, "switch gain profiles only while nothing is servoing (float or disable first)")
        self.profile = C.load_profile(name)
        self.events.add(f"gain profile {name}")
        return self.profile.public()

    def set_tcp(self, token, offset) -> dict:
        self.lease.renew(token)
        if self.servo:
            raise RequestError(409, "change the TCP offset only while the arm is not servoing (float or disable first)")
        self.kin.set_tcp_offset(offset)
        self.events.add(f"TCP offset {list(offset)} m")
        return {"offset": self.kin.tcp_offset}

    def set_limits(self, token, **kw) -> dict:
        self.lease.renew(token)
        h = self.settings.hard
        s = self.soft.copy()
        for k, v in kw.items():
            if v is None:
                continue
            if k in ("joint_vel", "joint_acc"):
                v = np.asarray(v, float)
                cap = getattr(h, k)
                if v.shape != (N,) or not np.all(np.isfinite(v)) or np.any(v <= 0) or np.any(v > cap):
                    raise ValueError(f"{k} must be {N} finite values within 0..{cap.tolist()}")
            elif not (math.isfinite(float(v)) and 0 < float(v) <= getattr(h, k)):
                raise ValueError(f"{k} must be within 0..{getattr(h, k):g}")
            setattr(s, k, v if k in ("joint_vel", "joint_acc") else float(v))
        self.soft = s
        self.events.add("soft limits changed")
        return self.limits()

    def limits(self) -> dict:
        return {"hard": self.settings.hard.public(), "soft": self.soft.public()}

    # reads -----------------------------------------------------------------------------------------

    def _need_kin(self) -> ArmKinematics:
        if self.kin is None:
            raise RequestError(503, "the arm has not connected yet (no model)")
        return self.kin

    def ik_check(self, pose: pin.SE3, seed: str = "command", max_rot_deg: Optional[float] = None,
                 render: Optional[dict] = None) -> dict:
        """IK seed only (never taken as where the arm is): "command" = the command while servoing,
        else the measured pose; "measured" = the measured pose. With ``render`` (view options,
        render.VIEW_DEFAULTS) the answer also carries ``image``: a PNG data URL of the query."""
        if render is not None:
            out, png = self.ik_image(pose, seed, max_rot_deg, render)
            return {**out, "image": "data:image/png;base64," + base64.b64encode(png).decode("ascii"),
                    "view": {**VIEW_DEFAULTS, **{k: v for k, v in render.items() if v is not None}}}
        kin = self._need_kin()
        r = self.reading
        with self._lock:
            q_cmd = self.q_cmd.copy() if (self.servo and self.q_cmd is not None) else None
        if seed == "measured" or q_cmd is None:
            if r is None:
                raise RequestError(503, "no reading yet")
            q0 = r.q
        else:
            q0 = q_cmd
        rot_tol = None if max_rot_deg is None else math.radians(max_rot_deg)
        out = Planner.run("ik", kin, pose, np.asarray(q0, float).copy(), rot_tol=rot_tol)   # in the planner process
        return out.public()

    def ik_image(self, pose: pin.SE3, seed: str = "command", max_rot_deg: Optional[float] = None,
                 view: Optional[dict] = None) -> tuple[dict, bytes]:
        """The IK check and its picture (render.py), both in the planner process: (IK answer, PNG).
        Shows the measured arm (and gripper gap), so it needs a reading. No lease; nothing moves."""
        kin = self._need_kin()
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        with self._lock:
            q_cmd = self.q_cmd.copy() if (self.servo and self.q_cmd is not None) else None
        q0 = r.q if (seed == "measured" or q_cmd is None) else q_cmd
        gap = C.rad_to_gap(r.g_q) if math.isfinite(r.g_q) else 0.0
        rot_tol = None if max_rot_deg is None else math.radians(max_rot_deg)
        out, png = Planner.run("ik_render", kin, pose, np.asarray(q0, float).copy(), np.asarray(r.q, float).copy(),
                               min(max(gap, 0.0), C.GAP_MAX), dict(view or {}), rot_tol=rot_tol)
        return out.public(), png

    def kinematics(self, q, frame: str = "arm") -> dict:
        frame = "arm" if frame == "base" else frame      # "base": the old name of the arm frame
        kin = self._need_kin()
        q = np.asarray(q, float)
        if q.shape != (N,) or not np.all(np.isfinite(q)):
            raise ValueError(f"a configuration is {N} finite joint angles (rad)")
        T = kin.fk(q)
        J = kin.jacobian(q, frame)
        return {"q": q.tolist(), "within_limits": kin.within_limits(q), "tcp": pose_public(T),
                "frame": frame, "jacobian": J.round(9).tolist(), **kin.manipulability(J)}

    def kinematics_batch(self, qs: list, frame: str = "arm") -> list:
        """Lookups at many configurations, in the planner process (no lease; nothing moves)."""
        kin = self._need_kin()
        arr = np.asarray(qs, float)
        if arr.ndim != 2 or arr.shape[1] != N or not np.all(np.isfinite(arr)):
            raise ValueError(f"each configuration is {N} finite joint angles (rad)")
        return Planner.run("kinematics", kin, [list(map(float, q)) for q in arr], frame="arm" if frame == "base" else frame)

    def kinematics_now(self, frame: str = "arm", which: str = "measured") -> dict:
        """which "command": the command, only while servoing (409 otherwise)."""
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        if which == "command":
            with self._lock:
                if not (self.servo and self.q_cmd is not None):
                    raise RequestError(409, f"no command: the arm is not servoing ({self.state})")
                q = self.q_cmd.copy()
        else:
            q = r.q
        out = self.kinematics(q, frame)
        out["which"] = which
        return out

    # -- geometry as numbers (read only) -------------------------------------------------------------

    def arm_in_base(self) -> dict:
        """The arm frame's pose in the robot base frame (p_base = T p_arm), from the description node's
        ``arm`` frame (cached 10 s); its shipped default if that node is unreachable. Says which."""
        now = time.monotonic()
        c = getattr(self, "_aib", None)
        if c and now - c["t"] < 10.0:
            return c["v"]
        try:
            from ..common.client import NodeClient
            with NodeClient(self.description_node, timeout=1.5) as cl:
                d = cl.get("/transform", **{"from": "base", "to": "arm"})
            T, src = np.asarray(d["matrix"], float), f"description node ({self.description_node!r}): frame 'arm'"
        except Exception as e:
            import yaml
            from importlib import resources
            from ..description.frames import frame_T
            f = yaml.safe_load(resources.files("robot_utils.nodes.description").joinpath("default_description.yaml").read_text())
            T = frame_T(f["frames"]["arm"])
            src = f"shipped default (description node unreachable: {type(e).__name__})"
        v = {"T": T, "source": src}
        self._aib = {"t": now, "v": v}
        return v

    def geometry(self, q=None, pose: Optional[pin.SE3] = None, seed: str = "measured", max_rot_deg: Optional[float] = None,
                 gap_mm: Optional[float] = None) -> dict:
        """Gripper and finger geometry, and every box of the arm, as numbers in both the arm and the robot
        base frame: at the measured configuration, at ``q``, or at the IK solution of a TCP ``pose``."""
        from ..swerve.constants import FRAME
        from .body import GRASP_DEPTH, JAW_HEIGHT_ROOT, JAW_HEIGHT_TIP, JAW_LENGTH, JAW_THICKNESS, body
        kin = self._need_kin()
        r = self.reading
        ik = None
        if pose is not None:
            ik = self.ik_check(pose, seed, max_rot_deg)
            qv, which = np.asarray(ik["q"], float), "ik"
        elif q is not None:
            qv, which = np.asarray(q, float), "q"
            if qv.shape != (N,) or not np.all(np.isfinite(qv)):
                raise ValueError(f"a configuration is {N} finite joint angles (rad)")
        else:
            if r is None:
                raise RequestError(503, "no reading yet")
            qv, which = np.asarray(r.q, float), "measured"
        if gap_mm is not None:
            gap, gap_src = float(gap_mm) / 1000, "given"
        elif r is not None and math.isfinite(r.g_q):
            gap, gap_src = C.rad_to_gap(r.g_q), "measured"
        else:
            gap, gap_src = 0.0, "unknown (no reading): 0"
        F = [T.homogeneous for T in kin.frames(qv)]
        B = body(F, gap)
        aib = self.arm_in_base()
        Tb = aib["T"]
        rnd = lambda v: [round(float(x), 5) for x in v]
        pt = lambda p: {"arm": rnd(p), "base": rnd(Tb[:3, :3] @ p + Tb[:3, 3])}
        vec = lambda v: {"arm": rnd(v), "base": rnd(Tb[:3, :3] @ v)}
        pose2 = lambda T: {"arm": pose_public(pin.SE3(T)), "base": pose_public(pin.SE3(Tb @ T))}
        jaws = {k: {"tip": pt(j["tip"]), "inner_face_center": pt(j["inner_face_center"]),
                    "inner_face_root": pt(j["inner_face_root"]), "outer_face_tip": pt(j["outer_face_tip"]),
                    "inward": vec(j["inward"])} for k, j in B["jaws"].items()}
        boxes, allc, grip = [], [], []
        for name, link, corners, bp in B["boxes"]:
            cb = corners @ Tb[:3, :3].T + Tb[:3, 3]
            allc.append(cb)
            if link in ("end", "finger"):
                grip.append(cb)
            boxes.append({"name": name, "link": link, "center": pt(bp["center"]), "size_m": rnd(bp["size"]),
                          "axes_base": [rnd(Tb[:3, :3] @ bp["axes"][:, k]) for k in range(3)]})
        allc, grip = np.vstack(allc), np.vstack(grip)
        h = FRAME / 2
        ext = lambda P: {"min": rnd(P.min(0)), "max": rnd(P.max(0))}
        past = {"front": allc[:, 0].max() - h, "rear": -h - allc[:, 0].min(), "left": allc[:, 1].max() - h,
                "right": -h - allc[:, 1].min()}
        return {
            "which": which, "q": rnd(qv), "within_limits": kin.within_limits(qv), "ik": ik,
            "gap_mm": round(B["gap"] * 1000, 2), "gap_source": gap_src,
            "frames": {
                "arm_in_base": {"matrix": [rnd(row) for row in Tb], "xyz": rnd(Tb[:3, 3]), "source": aib["source"]},
                "note": "Every point is given in both frames: p_base = R p_arm + t (arm_in_base). The arm frame is "
                        "the arm's URDF base_link: the underside of its mount plate on joint 1's axis, axes parallel to "
                        "the robot base. Motion commands (/servo, /track, /chunk, /ik) take TCP poses in the arm frame.",
            },
            "tcp": {**pose2(B["tcp"]), "offset_in_end_link": kin.tcp_offset,
                    "note": "the point motion commands target, near the wrist (tcp_offset behind the fingertips)"},
            "fingertips": {"left": pt(B["jaws"]["left"]["tip"]), "right": pt(B["jaws"]["right"]["tip"]),
                           "center": pose2(B["end_link"]),
                           "note": "left/right: each jaw's tip on its inner (gripping) face, mid height; center: end_link, "
                                   "midway between them. end_link's x axis is the approach direction (out between the "
                                   "jaws), y the jaw travel (left jaw at +y), z the jaws' height"},
            "axes": {"approach": vec(B["approach"]), "jaw_travel": vec(B["jaw_axis"]), "up": vec(B["up"])},
            "jaws": jaws,
            "jaw_shape_m": {"grasp_depth": round(GRASP_DEPTH, 4), "length": round(JAW_LENGTH, 4),
                            "thickness": round(JAW_THICKNESS, 4), "height_root": JAW_HEIGHT_ROOT, "height_tip": JAW_HEIGHT_TIP,
                            "note": "approximate (from the meshes). Each jaw is a prong from its tip back along -approach "
                                    "to inner_face_root, where the jaw carriage begins: grasp_depth is how far an object "
                                    "can sit between the jaws, measured from the tips. The inner faces grip; each jaw is "
                                    "`thickness` thick outward from its inner face and tapers in height from root to tip"},
            "extent_base": {"arm": ext(allc), "gripper": ext(grip),
                            "past_footprint_m": {k: round(float(v), 4) for k, v in past.items()},
                            "note": f"axis-aligned bounds of every box in the robot base frame; past_footprint_m: how far "
                                    f"the arm reaches beyond each side of the {FRAME:.4f} m square frame (negative: inside)"},
            "boxes": boxes,
        }

    def links(self, q=None) -> list:
        """The joint frames, end_link and tcp at ``q`` (default measured), for drawing."""
        kin = self._need_kin()
        if q is None:
            if self.reading is None:
                raise RequestError(503, "no reading yet")
            q = self.reading.q
        return [pose_public(T) for T in kin.frames(q)]

    def gripper_state(self) -> dict:
        r, g = self.reading, self.gripper
        on = self.motors_on and self.g_servo
        mode = ("holding" if on else "free") if (self.motors_on and self.g_motor_on) else "damping"
        return {"mode": mode, "engaged": on, "gap_mm": None if r is None else round(C.rad_to_gap(r.g_q) * 1000, 2),
                "sent_mm": None if (g.sent is None or not on) else round(g.sent * 1000, 2),
                "goal_mm": None if g.goal is None else round(g.goal * 1000, 2),
                "max_force_n": g.max_force, "force_n": finite(round(self._g["force"], 2)) if on else None,
                "demand_n": finite(round(self._g["demand"], 2)) if on else None,
                "limiting": bool(self._g["limiting"] and on), "gap_max_mm": round(C.GAP_MAX * 1000, 2),
                "default_max_force_n": C.DEFAULT_MAX_FORCE}

    def joints(self) -> dict:
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        names = self.bus.names if self.bus else [f"joint{i}" for i in range(1, N + 1)]
        lo = hi = None
        if self.kin is not None:
            lo, hi = self.kin.lo.tolist(), self.kin.hi.tolist()
        with self._lock:
            q_cmd = self.q_cmd.tolist() if (self.servo and self.q_cmd is not None) else None
        return {"names": names[:N], "q": r.q.tolist(), "q_mot": r.q_mot.tolist(), "qd": r.qd.tolist(),
                "tau": r.tau.tolist(), "q_cmd": q_cmd,
                "status": r.status.tolist(), "t_mos": r.t_mos.tolist(), "t_rotor": r.t_rotor.tolist(),
                "limits": {"lo": lo, "hi": hi}, "t": r.t}

    def tracking(self) -> dict:
        last = self.hub.recent(1, "tick")
        if not len(last):
            raise RequestError(503, "no tick yet")
        row = last[0]
        g = lambda n: None if not math.isfinite(row[COL[n]]) else float(row[COL[n]])   # noqa: E731
        return {"servo": bool(row[COL["servo"]] == 1.0),
                "e_mot": [g(f"j{i}_e_mot") for i in range(1, N + 1)],
                "tau_est": [g(f"j{i}_tau_est") for i in range(1, N + 1)],
                "tau_g": [g(f"j{i}_tau_g") for i in range(1, N + 1)],
                "err_pos_m": g("err_pos"), "err_rot_rad": g("err_rot"),
                "goal_err_pos_m": g("goal_err_pos"), "goal_err_rot_rad": g("goal_err_rot"),
                "limited": bool(row[COL["limited"]] == 1.0), "t": float(row[0])}

    def state_dict(self) -> dict:
        info, e = self.lease.info, self.estop.info
        now = self.clock()
        last = self.hub.recent(1, "tick")
        latest = ({k: (v if math.isfinite(v) else None) for k, v in zip(FIELDS, last[0].tolist())}
                  if len(last) else None)
        recent = self.hub.recent(101, "tick")
        rate = ((len(recent) - 1) / (recent[-1, 0] - recent[0, 0])
                if len(recent) > 10 and recent[-1, 0] > recent[0, 0] else None)
        with self._lock:
            m = self.motion or self.last_motion
            tr = self.track if self.servo else None
            motion = m.public(now) if m else None
        ch = self.chocks()
        return finite({
            "state": self.state, "error": self.error, "sim": bool(self.bus and self.bus.sim),
            "initialized": self.initialized, "enabled": self.enabled, "motors_on": self.motors_on,
            "servo": self.servo, "gripper_engaged": self.g_servo,
            "latched_motor_faults": self.latched,
            "chocked": bool(ch), "locks": [l.public() for l in ch],
            "estop": {"latched": e is not None, "reason": e and e.reason, "source": e and e.source, "t": e and e.t},
            "lease": {"held": info is not None, "holder": info and info.client, "lease_id": info and info.lease_id,
                      "expires_in": None if info is None else round(info.expires - now, 3)},
            "source": self._source_name(),
            "track": None if tr is None else {"seq": tr.seq, "age": round(now - tr.t, 3), "goal": pose_public(tr.pose),
                                              "twist": None if tr.twist is None else tr.twist.tolist(),
                                              "twist_frame": tr.twist_frame},
            "motion": motion,
            "jam": self.jam_state(),
            "gripper": self.gripper_state(),
            "profile": self.profile.name, "profiles": list(C.PROFILES),
            "tcp_offset": None if self.kin is None else self.kin.tcp_offset,
            "named_poses": {k: np.degrees(v).round(3).tolist() for k, v in self.settings.named_poses.items()},
            "stow_near_deg": math.degrees(STOW_NEAR),
            "limits": self.limits(),
            "twist_timeout": TWIST_TIMEOUT, "lease_heartbeat": LEASE_HEARTBEAT,
            "loop_hz": None if rate is None else round(rate, 1), "overruns": self.overruns,
            "late_ticks": self.late_ticks, "read_misses": self.read_misses, "read_misses_by_motor": dict(self.misses_by_motor),
            "reply_latency": self.latency_stats(),
            "realtime": dict(self.realtime),
            "clients": self.hub.subscribers,
            "events": self.events.list()[-20:],
            "latest": latest,
            "server_mono": now,
        })
