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
from .bus import ArmBus, BusError, ReadLate, ReadMiss
from .config import N
from .gripper import Contact, Gripper
from .gripper import free as gripper_free
from .gripper import holding as gripper_holding
from .jam import JAM_RESPONSES, JamConfig, JamDetector  # noqa: F401  (JAM_RESPONSES re-exported)
from .jam import updated as jam_updated
from .kin import ArmKinematics, pose_error, pose_public
from .motion import Trajectory
from .planner import Planner
from .requests import ENGAGE_LIMIT_TOL, STOW_NEAR, TRACK_IK_ITER, TRACK_MAX_JUMP, ArmRequests  # noqa: F401
from .stream import JointStream, StreamConfig
from .telemetry import COL, FIELD_DOC, JAC_UNITS, PHASES, UNITS  # noqa: F401  (re-exported: server, models, tests)
from .telemetry import FIELDS, JAC_FIELDS, SOURCES, STATES, row_dict, tick_row

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
LATENCY_WARN_EVERY = 60.0     # replies later than one tick are waited for one more (at most one warning a minute)
LATE_WARN = 0.08              # s between ticks reported as an event (the motors' own 100 ms watchdog
                              # protects them: they drop to damping and report it, which faults)
ENC_MISMATCH_ENABLE = 0.05    # rad between output encoder and motor feedback that refuses enable
PLAN_DRIFT = 0.02             # rad the arm (measured) may move while a motion is planned
SLOW_TICK = 0.015             # s between ticks counted as an overrun
ENABLE_TIMEOUT = 8.0          # s a POST /enable waits for initialization
CONNECT_RETRY_S = 2.0



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


class ArmNode(JointStream, ArmRequests):
    dt = DT
    warm_planner = True             # the control process plans nothing (its server process does)

    def __init__(self, bus_factory: Callable[[], ArmBus], profile: str = C.DEFAULT_PROFILE,
                 settings: Optional[C.NodeSettings] = None, hub: Optional[RowHub] = None,
                 locks: Optional[LockStore] = None, clock: Callable[[], float] = time.monotonic,
                 description_node: str = "description", joint_socket=None):
        """``joint_socket``: path of the internal joint channel's socket (stream.py), started with the node; None:
        no joint channel."""
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
        self._seen_on = np.zeros(N + 1, bool)   # per motor: reported enabled (status 1) since it was switched on
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
        self._g_contact = Contact()               # the squeeze limit's contact detector (control thread)
        self._g_engage = False
        self.reading = None
        self.latched: list[str] = []      # motor fault codes latched while the motors are off (enable clears them)
        self._fresh_t: Optional[float] = None   # when the latest complete reading was taken
        self.read_age = 0.0
        self.read_misses = 0
        self.misses_by_motor: dict = {}
        self._lat: deque = deque(maxlen=LATENCY_WINDOW)   # s, complete reads only
        self._tick_time: deque = deque(maxlen=LATENCY_WINDOW)   # (wall s, thread CPU s) per tick
        self._lat_n = 0
        self._lat_reported = False
        self._lat_warn_t = -math.inf
        self._refresh_done: dict = {"fallbacks_by_motor": {}, "refreshes": 0}   # closed buses' counts
        self._fallback_reported = False     # this connection's first feedback-refresh fallback
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
        self.jam_detector = JamDetector()         # jam (its config), jam_active, jams, jam_count: properties below
        self._cmd_count = 0                       # every new command / motion (refire off: one jam per command)
        self._aib: Optional[dict] = None           # arm_in_base's cache (queries.arm_in_base)
        st = self.settings
        self._init_stream(joint_socket, StreamConfig(st.stream_stale_ms, st.stream_max_lead))

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
        self.start_joint_channel()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="arm-control", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self.stop_joint_channel()
        self._close_bus()

    def _close_bus(self) -> None:
        """Best effort, step by step: a failing disable must never keep the serial port open."""
        try:                                # bookkeeping only: it must never stand between close and disable
            fr = None if self.bus is None else self.bus.refresh
            if fr is not None:              # keep its counts: /state counts since the node started
                done = self._refresh_done
                for m, c in fr.fallbacks.items():
                    done["fallbacks_by_motor"][m] = done["fallbacks_by_motor"].get(m, 0) + c
                done["refreshes"] += fr.refreshes
        except Exception as e:  # noqa: BLE001
            log.warning("keeping the feedback-refresh counts: %s", e)
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
        self._steps += 1
        if self._reconnect:
            self._reconnect = False
            self._close_bus()
            self._base, self.initialized = "connecting", False
            self._connect_at = 0.0
        if self._base == "connecting" or self.bus is None:
            now = self.clock()
            self._stream_out(now, self.reading, math.inf if self._fresh_t is None else now - self._fresh_t)
            if self._base == "connecting" and now >= self._connect_at:
                self._connect()
            return
        w0, c0 = time.perf_counter(), time.thread_time()
        self.tick()
        self._tick_time.append((time.perf_counter() - w0, time.thread_time() - c0))

    def tick_time(self) -> Optional[dict]:
        """How long the last LATENCY_WINDOW ticks took (ms): wall, on the CPU (this thread's CPU time: Python
        running, i.e. holding the GIL, or native code) and off it (wall minus CPU: waiting for the GIL, for the bus,
        or preempted), or None with too few."""
        try:
            a = np.array(list(self._tick_time), dtype=float) * 1e3
        except RuntimeError:          # appended to meanwhile
            return None
        if len(a) < 50:
            return None
        w, c = a[:, 0], a[:, 1]
        off = np.maximum(w - c, 0.0)
        r = lambda v: round(float(v), 3)   # noqa: E731
        return {"n": int(len(a)), "wall_p50_ms": r(np.percentile(w, 50)), "wall_p99_ms": r(np.percentile(w, 99)),
                "wall_max_ms": r(w.max()), "cpu_p50_ms": r(np.percentile(c, 50)), "cpu_p99_ms": r(np.percentile(c, 99)),
                "off_cpu_p50_ms": r(np.percentile(off, 50)), "off_cpu_p99_ms": r(np.percentile(off, 99)),
                "off_cpu_max_ms": r(off.max())}

    def _connect(self) -> None:
        before = realtime.task_ids()        # the bus driver's own threads: those that appear by the end of connect
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
            self._raise_bus_threads(before)
            self.kin = self.kin or ArmKinematics(self.bus.setpoint_limits)
            if self.warm_planner:
                Planner.warm(self.kin)      # start the planner process now, not on the first plan
            self._base, self.error = "uninitialized", None
            self._lat.clear()
            self._lat_reported = False      # report the new connection's reply latency
            self._fallback_reported = False
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

    def _raise_bus_threads(self, before) -> None:
        """motorbridge (libmotor_abi) starts a native polling worker when the bus opens; created by this real-time
        thread it starts as a normal one (SCHED_RESET_ON_FORK), so under load it reads the replies late and may hold
        a lock this thread then waits for. Raise it just below this loop (every connect: a reconnect starts a new
        one). A simulated bus has none. Never fails the connect."""
        if self.bus.sim or before is None:
            self.realtime = {**self.realtime, "bus_threads": None}
            return
        rec = realtime.raise_native_threads(before, realtime.PRIO_BUS_IO, "bus reader")
        self.realtime = {**self.realtime, "bus_threads": rec["threads"]}
        self.events.add(realtime.describe_threads(rec, "bus reader"), rec["level"])

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
            self._note_fallbacks()
        except ReadMiss as e:
            if not isinstance(e, ReadLate):          # a late reply (still in flight, one tick) is not a miss
                self.read_misses += 1
                for m in e.missing:
                    self.misses_by_motor[m] = self.misses_by_motor.get(m, 0) + 1
            age = math.inf if self._fresh_t is None else now - self._fresh_t
            limit = self.settings.stale_on if self.motors_on else STALE_OFF
            if self.motors_on and STALE_WARN <= age <= limit and not self._stale_warned:
                self._stale_warn(now, age, e.missing)
            if self.reading is None or age > limit:
                self._fault(f"{e} (no complete reading for {age * 1e3:.0f} ms)")
                self._stream_out(now, self.reading, age)
                return
            r, stale, self.read_age = self.reading, True, age     # the latest complete reading, briefly
        except BusError as e:
            self._fault(str(e))
            self._stream_out(now, self.reading, math.inf if self._fresh_t is None else now - self._fresh_t)
            return
        self._stream_out(now, r, self.read_age)        # the joint channel's state: right after the read, every tick
        if self._base == "fault":
            self._publish(now, dt_tick, r)
            return
        faults = [] if stale else r.faults(self.bus.names)
        if not stale:
            self.latched = [] if self.motors_on else faults
        if faults and self.motors_on:
            self._fault("motor fault: " + "; ".join(faults))
            return
        off = [] if stale else self._dropped_off(r)
        if off:
            self._fault(f"motor disabled unexpectedly: {', '.join(off)} reports disabled while the motors are on")
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
        self._pre_control()
        if self.motors_on:
            self._control(now, r)
        self._publish(now, dt_tick, r)

    def _pre_control(self) -> None:
        """Right before the command (control process: the goals that arrived during the read)."""

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
                        f"{self.settings.stale_on * 1e3:.0f} ms" + (f" ({more} more since the last report)" if more else ""),
                        "warn")

    def _note_latency(self, now: float) -> None:
        """Control thread: keep the bus's reply latency; report it once after connecting, then warn
        when the slowest replies come close to the tick period."""
        lat = self.bus.reply_latency
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
                            f"to the {DT * 1e3:.0f} ms tick" + (" (too little: later replies are waited for one more tick, then count as missed)" if low else ""),
                            "warn" if low else "info")
            return
        if self._lat_n % LATENCY_WINDOW or now - self._lat_warn_t < LATENCY_WARN_EVERY:
            return
        st = self.latency_stats()
        if st is not None and st["margin_ms"] < LATENCY_MARGIN_WARN * 1e3:
            self._lat_warn_t = now
            self.events.add(f"reply latency p99 {st['p99_ms']:.1f} ms (max {st['max_ms']:.1f} ms) over the last "
                            f"{st['n']} complete reads: {st['margin_ms']:.1f} ms margin to the {DT * 1e3:.0f} ms tick; "
                            "later replies are waited for one more tick (missed after two), and with the motors on "
                            f"{self.settings.stale_on * 1e3:.0f} ms without a complete reading faults", "warn")

    def _note_fallbacks(self) -> None:
        """Control thread: the first time on a connection that a motor's feedback had to be refreshed
        because its MIT frames went unanswered, say so (counted in /state feedback_refresh)."""
        try:            # reporting only: it must never cost the tick (the e-stop and motors-off handling follow)
            self._report_fallbacks()
        except Exception as e:  # noqa: BLE001
            log.warning("feedback-refresh report: %s", e)

    def _report_fallbacks(self) -> None:
        fr = self.bus.refresh
        if self._fallback_reported or fr is None or not fr.fallbacks:
            return
        self._fallback_reported = True
        self.events.add(f"feedback refresh fallback: no feedback frame answered the last two MIT frames to "
                        f"{', '.join(fr.fallbacks)}; their feedback is refreshed instead (counted in /state "
                        "feedback_refresh; on firmware that answers MIT frames this never happens)", "warn")

    def feedback_refresh(self) -> dict:
        """Feedback refreshes since the node started (this connection's and closed ones')."""
        done = self._refresh_done
        by = dict(done["fallbacks_by_motor"])
        fr = None if self.bus is None else self.bus.refresh
        refreshes = done["refreshes"]
        if fr is not None:
            for m, c in list(fr.fallbacks.items()):
                by[m] = by.get(m, 0) + c
            refreshes += fr.refreshes
        return {"skip": None if fr is None else fr.skip, "fallbacks": sum(by.values()),
                "fallbacks_by_motor": by, "refreshes": refreshes}

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

    def _dropped_off(self, r) -> list[str]:
        """Motors that should be on but report disabled (status 0): a brown-out, an external disable, a simulated
        world that lost them. Only after the motor has reported enabled since it was switched on: the bus is
        pipelined, so the first readings after an enable can still carry the replies from before it."""
        expect = np.zeros(N + 1, bool)
        if self.motors_on:
            expect[:N] = True
            expect[N] = self.g_motor_on
        st = np.asarray(r.status)
        self._seen_on &= expect
        self._seen_on |= expect & (st == 1)
        return [n for n, e, seen, c in zip(self.bus.names, expect, self._seen_on, st) if e and seen and c == 0]

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
        if not np.all(np.isfinite(q_goal)):        # backstop: a NaN goal would make q_cmd NaN for good
            self.nonfinite_goals += 1
            if self.nonfinite_goals == 1 or self.nonfinite_goals % 1000 == 0:
                self.events.add(f"non-finite goal ({self.nonfinite_goals} so far): holding the command", "error")
            q_goal = self.q_cmd
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
        gravity feedforward either way. Gripper: holding its goal (the force limit in contact) or free."""
        kin, p = self.kin, self.profile
        src = None
        with self._lock:
            self._stream_applied = None              # set by _stream_target when the stream drives this tick
            if self._has_source() and self.enabled and not self.servo and not self.estop.active:
                self._engage(r)
            if self.servo:
                src, q_goal, vel_ref, gap_goal = self._source(now)
                self._command_step(q_goal)
                if not np.all(np.isfinite(vel_ref)):
                    vel_ref = np.zeros(N)
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
        if g_on:      # holding its goal; the force limit in contact (gripper.holding)
            gc = gripper_holding(g_goal, max_force, r.g_q, r.g_q_mot, r.g_qd, p.g_kp, p.g_kd, now,
                                 self._g_contact, self.settings.gripper)
            g.sent = C.rad_to_gap(gc.q)
        else:
            self._g_contact.reset()
            gc = gripper_free(r.g_q_mot)
        self._g = {"tau_est": gc.tau_est, "force": gc.applied, "demand": gc.demand, "ff": gc.ff,
                   "limiting": gc.limiting, "tau_g": tau_g}
        try:
            self.bus.send(q_cmd, qd_ff, kp, kd, tau_g, gc.q, 0.0, gc.kp, gc.kd, gc.ff, gripper=self.g_motor_on)
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
        return (self.track is not None or self.stream is not None
                or (m is not None and m.phase in ("executing", "settling")))

    def _arm_engaged(self) -> bool:
        """Servoing, or a command accepted that the control loop engages on its next tick. Call under _lock.
        Checks that mean "is the arm under command" use this, not ``servo`` alone: ``servo`` turns on only at
        the next tick, up to a loop period after the command was accepted."""
        return self.servo or (self._has_source() and self.enabled)

    def _source(self, now: float):
        """(source, goal, MIT velocity reference, gripper gap goal or None); servoing, under _lock.
        Track: the goal is the IK of the pose, literally; a fresh twist (TWIST_TIMEOUT) gives the
        velocity reference. Planned motions: the trajectory's position and velocity at ``now``.
        Stream: the joint goal, literally, with its rate-limited feedforward; stale, a ramp to a hold
        (stream.py). Track and stream never coexist (each replaces the other)."""
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
        if self.stream is not None:                  # the internal joint channel (stream.py)
            self._goal_pose = None
            q, qd = self._stream_target(now)
            return "stream", q, qd, None
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

    @property
    def jam(self) -> JamConfig:
        return self.jam_detector.cfg

    @jam.setter
    def jam(self, cfg: JamConfig) -> None:
        self.jam_detector.cfg = cfg

    @property
    def jam_active(self) -> Optional[dict]:
        """The jam in force: joint, residual, since, response."""
        return self.jam_detector.active

    @property
    def jams(self) -> list:
        """Recent jams (last 20), newest last."""
        return self.jam_detector.recent

    @property
    def jam_count(self) -> int:
        return self.jam_detector.count

    def _jam_reset(self) -> None:
        self.jam_detector.reset()

    def _jam_check(self, now: float, r, src, tau_g) -> None:
        """Control thread, while servoing (jam.JamDetector judges). Planned motions and holds get the response: the
        command is pulled to the reading (a motion ends `jammed`); track and stream (a client overwrites the
        command every tick) are report-only."""
        m = self.motion
        ev = self.jam_detector.check(now, r.qd, r.tau, tau_g, m is not None and m.phase == "executing", src,
                                     self._cmd_count)
        if ev is None:
            return
        if ev.cleared:
            self.events.add(f"jam cleared (joint{ev.joint})")
            return
        what = ev.message
        if ev.jam["response"] == "hold":
            with self._lock:     # stop pushing: hold where the arm is, not where it was told to be
                self._command_from_reading(r, np.zeros(N))
            if self.motion is not None:
                self._end_motion("jammed", what + ": holding where it is")
            else:
                self.events.add(what + ": holding where it is", "warn")
        else:
            self.events.add(what + (f" ({src}: report only)" if src in ("track", "stream") else " (report only)"), "warn")

    def jam_state(self) -> dict:
        return finite({"config": self.jam.public(), "active": self.jam_active, "count": self.jam_count,
                       "recent": list(self.jams)})

    def set_jam(self, changes: dict) -> dict:
        """Change the jam detector (the server allows it for a human only)."""
        self.jam = jam_updated(self.jam, changes)
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
        self.stream = None
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
            stream = self._stream_row(now)
        on = self.motors_on
        base = [now, self._tick, STATES.index(self.state), float(self.enabled), float(on),
                float(servo), float(g_on), float(self.estop.active), float(info.lease_id if info else 0),
                SOURCES.index(src), seq, self.overruns, dt_tick, float(self._limited and servo),
                (now - tr.t) if tr and tr.twist is not None else nan, self.read_age, self.read_misses]
        row, J = tick_row(now, base, r, kin, self.profile, q_cmd, v_cmd, acc_lim,
                          self._g.get("tau_g") if on else None, goal, g, self._g, on and g_on, m,
                          self.jam_active is not None, stream)
        self.hub.publish_rows(row[None], "tick")
        self.hub.publish_rows(np.concatenate([[now], J.reshape(-1)])[None], "jacobian")

    def _source_name(self) -> str:
        if not self.servo:
            return "none"
        m = self.motion
        if m is not None and m.phase in ("executing", "settling"):
            return m.traj.kind
        if self.track is not None:
            return "track"
        return "stream" if self.stream is not None else "hold"

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
        self._g_contact.reset()

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
        self.close_stream_session(f"lease {reason}")
        self.soft = C.DEFAULT_SOFT.copy()
        self._disable(f"lease {reason}: disabled (the arm floats)")
        self.events.add(f"lease {reason}")

    def _estop_triggered(self, info: EStopInfo) -> None:
        self._off(f"E-STOP ({info.source}: {info.reason})")
        self.close_stream_session("e-stop")

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
        self.close_stream_session("disabled")
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

    # -- ArmRequests' hooks (requests.py) ------------------------------------------------------------

    def _new_command(self) -> None:
        self._cmd_count += 1

    def _track_context(self):
        with self._lock:
            prev = self.track if self._arm_engaged() else None     # also a track not yet engaged
            return (None if prev is None else prev.seq), self._arm_start()

    def _track_apply(self, q, pose: pin.SE3, twist, twist_frame: str, seq: int, g_args) -> None:
        with self._lock:
            if not self.enabled:
                raise RequestError(409, "disabled while solving")
            self._preempt("track")
            self.stream = None                      # HTTP track replaces the joint stream (and vice versa)
            self.hold_q = None
            self.track = Track(q, pose, twist, twist_frame, self.clock(), int(seq))
            if g_args is not None:
                self._apply_gripper(*g_args)

    def _arm_start_now(self) -> np.ndarray:
        with self._lock:
            return self._arm_start()

    def _servo_cmd(self) -> Optional[np.ndarray]:
        """A copy of the command while servoing, else None (IK seeds)."""
        with self._lock:
            return self.q_cmd.copy() if (self.servo and self.q_cmd is not None) else None

    def _bus_names(self) -> Optional[list]:
        b = self.bus
        return None if b is None else list(b.names)

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
            self.stream = None
            if self.stream_session is not None:    # a planned motion pauses the joint stream until it ends
                self.stream_session.paused = True
            self.hold_q = None
            self.motion = Motion(traj)
            if traj.kind == "chunk" and traj.gap is not None:
                # a chunk's gripper samples act only while the gripper holds: engage it now that the chunk is in
                # force (not before planning: a refused chunk must not move it), keeping the force limit in force
                self._apply_gripper(float(traj.gap[0]), self.gripper.max_force)
            out = self.motion.public(now)
        self.events.add(f"{traj.kind} {traj.id}: {traj.duration:.2f} s" + (f" to {traj.name}" if traj.name else ""))
        return out

    def cancel(self, token) -> dict:
        """End the motion or track: servoing holds where its command is (floating: nothing to do)."""
        self._ready(token)
        with self._lock:
            m = self.motion
            self.track = None
            self.stream = None
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
        with self._lock:
            busy = self._arm_engaged() or self.g_servo or self._g_engage   # incl. commands engaging next tick
        if busy:
            raise RequestError(409, "switch gain profiles only while nothing is servoing (float or disable first)")
        self.profile = C.load_profile(name)
        self.events.add(f"gain profile {name}")
        return self.profile.public()

    def set_tcp(self, token, offset) -> dict:
        self.lease.renew(token)
        with self._lock:
            busy = self._arm_engaged()
        if busy:
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

    def core_state(self, now: float) -> dict:
        """The control-owned part of /state (``compose_state`` adds the lease, the e-stop, the chocks, the rows and
        the events). The two-process node's control process publishes this as its snapshot."""
        with self._lock:
            m = self.motion or self.last_motion
            tr = self.track if self._arm_engaged() else None      # incl. a track the next tick engages
            motion = m.public(now) if m else None
        return {
            "state": self.state, "error": self.error, "sim": bool(self.bus and self.bus.sim),
            "initialized": self.initialized, "enabled": self.enabled, "motors_on": self.motors_on,
            "servo": self.servo, "gripper_engaged": self.g_servo,
            "latched_motor_faults": list(self.latched),
            "source": self._source_name(),
            "track": None if tr is None else {"seq": tr.seq, "age": round(now - tr.t, 3), "goal": pose_public(tr.pose),
                                              "twist": None if tr.twist is None else tr.twist.tolist(),
                                              "twist_frame": tr.twist_frame},
            "motion": motion,
            "jam": self.jam_state(),
            "gripper": self.gripper_state(),
            "profile": self.profile.name,
            "tcp_offset": None if self.kin is None else self.kin.tcp_offset,
            "limits": self.limits(),
            "overruns": self.overruns, "late_ticks": self.late_ticks, "read_misses": self.read_misses,
            "read_misses_by_motor": dict(self.misses_by_motor),
            "reply_latency": self.latency_stats(),
            "feedback_refresh": self.feedback_refresh(),
            "realtime": dict(self.realtime),
            "joint_channel": self.stream_state(),
            "tick_time": self.tick_time(),
        }

    def state_dict(self) -> dict:
        now = self.clock()
        return compose_state(self.core_state(now), now, self.settings, self.lease.info, self.estop.info, self.chocks(),
                             self.hub, self.events, {"mode": "thread"})


def compose_state(core: dict, now: float, settings: C.NodeSettings, info, e, ch: list, hub: RowHub, events: EventLog,
                  process: Optional[dict] = None) -> dict:
    """GET /state: the control-owned part (``ArmNode.core_state``) with the lease ``info``, the e-stop ``e``, the
    chocks ``ch``, the latest rows and the events."""
    last = hub.recent(1, "tick")
    latest = row_dict(last[0]) if len(last) else None
    recent = hub.recent(101, "tick")
    rate = ((len(recent) - 1) / (recent[-1, 0] - recent[0, 0])
            if len(recent) > 10 and recent[-1, 0] > recent[0, 0] else None)
    out = {k: core[k] for k in ("state", "error", "sim", "initialized", "enabled", "motors_on", "servo",
                                "gripper_engaged", "latched_motor_faults")}
    out.update({
        "chocked": bool(ch), "locks": [l.public() for l in ch],
        "estop": {"latched": e is not None, "reason": e and e.reason, "source": e and e.source, "t": e and e.t},
        "lease": {"held": info is not None, "holder": info and info.client, "lease_id": info and info.lease_id,
                  "expires_in": None if info is None else round(info.expires - now, 3)},
        "source": core["source"], "track": core["track"], "motion": core["motion"], "jam": core["jam"],
        "gripper": core["gripper"],
        "profile": core["profile"], "profiles": list(C.PROFILES),
        "tcp_offset": core["tcp_offset"],
        "named_poses": {k: np.degrees(v).round(3).tolist() for k, v in settings.named_poses.items()},
        "stow_near_deg": math.degrees(STOW_NEAR),
        "limits": core["limits"],
        "twist_timeout": TWIST_TIMEOUT, "lease_heartbeat": LEASE_HEARTBEAT,
        "loop_hz": None if rate is None else round(rate, 1),
        **{k: core[k] for k in ("overruns", "late_ticks", "read_misses", "read_misses_by_motor", "reply_latency",
                                "feedback_refresh", "realtime")},
        "clients": hub.subscribers,
        "events": events.list()[-20:],
        "joint_channel": core["joint_channel"],
        "latest": latest,
        "server_mono": now,
    })
    if process is not None:
        out["process"] = {**process, "tick_time": core.get("tick_time")}
    return finite(out)
