"""The arm node's core (docs/arm.md): one control thread owns the bus; requests only write small,
locked pieces of state and never touch it.

States: connecting → uninitialized → (enable) initializing → enabled ⇄ disabled; fault from any
of them (``reset`` reconnects). The e-stop is a latched flag over all of them.

Control tick (100 Hz, ``tick``):
  1. read the bus (a missing reading or a motor fault code → fault, motors disabled);
  2. chocks, lease expiry, pending enable / disable;
  3. enabled: the motion source in force gives ``q_goal`` and ``qd_ff`` (hold / track / a planned
     trajectory); step limit ``q_cmd += clip(q_goal − q_cmd, ±v_max·dt)``; gravity from the
     measured pose × the profile's ``tau_scale``; the gripper's command with its force limit;
     one MIT frame per motor;
  4. telemetry: one row on the ``tick`` channel and the TCP Jacobian on ``jacobian``.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
import pinocchio as pin

from ..common.estop import EStop, EStopInfo
from ..common.hub import RowHub
from ..common.lease import Lease, RequestError
from ..common.locks import LockStore, describe as describe_locks
from ..common.server import EventLog
from . import config as C
from .bus import ArmBus, BusError
from .kin import ArmKinematics, IKOut, N, pose_error, pose_public
from .motion import PlanError, Trajectory, plan_chunk, plan_joint, plan_servo

log = logging.getLogger(__name__)

RATE_HZ = 100.0
DT = 1.0 / RATE_HZ
TWIST_TIMEOUT = 0.25          # s a track twist feedforward stays in force
LEASE_HEARTBEAT = 1.0         # s
LATE_TICK = 0.06              # s between ticks while enabled = fault (motor watchdog 100 ms)
LATE_SEND = 0.09              # s since the last command frame, checked just before sending
ENABLE_LIMIT_TOL = math.radians(3.0)   # a joint this far outside its limits is clamped at enable; further = refused
ENC_MISMATCH_ENABLE = 0.05    # rad between output encoder and motor that refuses enable
ENC_MISMATCH_RUN = 0.15       # rad that faults while running (backlash + load deflection stay below)
PLAN_DRIFT = 0.02             # rad the command may move while a servo / home is planned
TRACK_MAX_JUMP = math.radians(20.0)    # a track goal this far (any joint) from the previous one is refused
SLOW_TICK = 0.015             # s between ticks counted as an overrun
ENABLE_TIMEOUT = 8.0          # s a POST /enable waits for initialization
CONNECT_RETRY_S = 2.0
STATES = ["connecting", "uninitialized", "initializing", "enabled", "disabled", "fault"]
SOURCES = ["hold", "track", "servo", "pose", "home", "chunk"]
PHASES = ["none", "executing", "settling", "reached", "not_reached", "done", "aborted", "preempted"]
JOINT_COLS = [("q", "rad"), ("q_mot", "rad"), ("qd", "rad/s"), ("tau", "Nm"), ("q_cmd", "rad"),
              ("qd_ff", "rad/s"), ("tau_g", "Nm"), ("e_mot", "rad"), ("tau_est", "Nm"), ("t_mos", "C"),
              ("status", "enum")]
_BASE = [("t_tick", "s"), ("tick", "count"), ("state", "enum"), ("enabled", "bool"), ("motors_on", "bool"),
         ("estop", "bool"), ("lease_id", "id"), ("source", "enum"), ("seq", "count"), ("overruns", "count"),
         ("tick_dt", "s"), ("limited", "bool"), ("twist_age", "s")]
_TCP = [("x", "m"), ("y", "m"), ("z", "m"), ("qx", ""), ("qy", ""), ("qz", ""), ("qw", ""),
        ("cmd_x", "m"), ("cmd_y", "m"), ("cmd_z", "m"),
        ("err_pos", "m"), ("err_rot", "rad"), ("goal_err_pos", "m"), ("goal_err_rot", "rad"),
        ("w", ""), ("w_lin", ""), ("sigma_min", "")]
_GRIP = [("g_gap", "m"), ("g_gap_cmd", "m"), ("g_gap_goal", "m"), ("g_qd", "rad/s"), ("g_tau", "Nm"),
         ("g_tau_est", "Nm"), ("g_force", "N"), ("g_tau_ff", "Nm"), ("g_force_max", "N"), ("g_limiting", "bool"),
         ("g_status", "enum")]
_MOTION = [("motion_id", "id"), ("phase", "enum"), ("progress", "")]
_ALL = (_BASE + [(f"j{i}_{n}", u) for i in range(1, N + 1) for n, u in JOINT_COLS] + _TCP + _GRIP + _MOTION)
FIELDS = [n for n, _ in _ALL]
UNITS = [u for _, u in _ALL]
COL = {n: i for i, n in enumerate(FIELDS)}
JAC_FIELDS = ["t_tick"] + [f"J{r}{c}" for r in range(6) for c in range(1, N + 1)]
JAC_UNITS = ["s"] + [""] * 36
FIELD_DOC = {
    "state": "index into " + ", ".join(STATES),
    "source": "motion source in force: index into " + ", ".join(SOURCES),
    "phase": "of the planned motion (servo / pose / home / chunk): index into " + ", ".join(PHASES),
    "j<i>_q": "output encoder (xout): the arm's measured pose",
    "j<i>_q_mot / qd / tau": "motor-side feedback frame",
    "j<i>_e_mot": "q_cmd - q_mot: the error the motor's MIT loop sees",
    "j<i>_tau_est": "kp*e_mot + kd*(qd_ff - qd) + tau_g: torque the MIT law applies",
    "j<i>_status": "Damiao status code (1 enabled, 0 disabled, >= 8 fault)",
    "x..qw": "TCP pose from the output encoders, arm base frame (quaternion x, y, z, w)",
    "cmd_x..z": "TCP position of q_cmd",
    "err_pos / err_rot": "measured TCP vs commanded TCP (output side)",
    "goal_err_pos / rot": "commanded TCP vs the goal of the source (lag behind the goal: step limit, trajectory)",
    "w / w_lin / sigma_min": "manipulability sqrt(det(J J^T)) of the 6x6 TCP Jacobian, of its translational rows, and its smallest singular value, at the measured pose",
    "limited": "the step limit clipped q_cmd this tick",
    "twist_age": "s since the track twist feedforward was written (NaN if none)",
    "g_gap / g_gap_cmd / g_gap_goal": "jaw gap measured / commanded this tick / goal (m)",
    "g_force": "squeeze force estimated from the gripper's MIT law (N)",
    "g_tau_ff": "force-limit feedforward (opposes closing; |g_tau_ff| <= g_tau_est)",
    "J<r><c>": "jacobian channel: TCP Jacobian at the measured pose, base-frame axes (LOCAL_WORLD_ALIGNED), row r of [vx vy vz wx wy wz], column = joint",
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
class Gripper:
    goal: Optional[float] = None       # m gap
    cmd: Optional[float] = None        # m gap (rate limited)
    max_force: float = C.DEFAULT_MAX_FORCE
    v: float = 0.0                     # m/s of the command


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
                 locks: Optional[LockStore] = None, clock: Callable[[], float] = time.monotonic):
        self.bus_factory = bus_factory
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
        self.state = "connecting"
        self.error: Optional[str] = None
        self.initialized = False
        self.enabled = False
        self.motors_on = False
        self.soft = C.DEFAULT_SOFT.copy()
        self._lock = threading.RLock()
        self._enable_req: Optional[dict] = None
        self._reconnect = False
        self.q_cmd: Optional[np.ndarray] = None
        self.qd_ff = np.zeros(N)          # MIT velocity reference sent (= q_cmd's velocity)
        self.v_cmd = np.zeros(N)
        self._last_send = 0.0
        self._plan_lock = threading.Lock()
        self.track: Optional[Track] = None
        self.motion: Optional[Motion] = None
        self.hold_q: Optional[np.ndarray] = None   # what "hold" holds (None: q_cmd)
        self.last_motion: Optional[Motion] = None
        self.gripper = Gripper()
        self.reading = None
        self._tick = 0
        self._last_t: Optional[float] = None
        self.overruns = 0
        self._limited = False
        self._goal_pose: Optional[pin.SE3] = None
        self._g = {"tau_est": math.nan, "force": math.nan, "ff": 0.0, "limiting": False}
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._connect_at = 0.0

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
        if self.bus is not None:
            try:
                self.bus.disable()
                self.bus.close()
            except Exception as e:  # noqa: BLE001
                log.warning("closing the bus: %s", e)
        self.bus, self.motors_on = None, False

    def _run(self) -> None:
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
            self.state, self.initialized = "connecting", False
            self._connect_at = 0.0
        if self.state == "connecting":
            if self.clock() >= self._connect_at:
                self._connect()
            return
        if self.bus is None:
            return
        self.tick()

    def _connect(self) -> None:
        try:
            self.bus = self.bus_factory()
            self.bus.connect()
            self.kin = self.kin or ArmKinematics(self.bus.setpoint_limits)
            self.state, self.error = "uninitialized", None
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
        if self.motors_on and dt_tick > LATE_TICK:
            self._fault(f"control tick {dt_tick * 1e3:.0f} ms late (the motors' watchdog is 100 ms)")
            return
        if dt_tick > SLOW_TICK:
            self.overruns += 1
        try:
            r = self.bus.read()
        except BusError as e:
            self._fault(str(e))
            return
        self.reading = r
        faults = r.faults(self.bus.names)
        if faults and self.state != "fault":
            self._fault("motor fault: " + "; ".join(faults))
            return
        mism = np.abs(np.append(r.q - r.q_mot, r.g_q - r.g_q_mot))
        if self.motors_on and mism.max() > ENC_MISMATCH_RUN:
            i = int(np.argmax(mism))
            self._fault(f"{self.bus.names[i]}: output encoder and motor disagree by {math.degrees(mism[i]):.1f}°")
            return
        if self.state == "fault":
            self._publish(now, dt_tick, r)
            return
        ch = self.chocks()
        if ch and self.enabled:
            self._disable(f"CHOCKED ({describe_locks(ch)}): disabled")
        self.lease.expire_if_due()
        if not self.enabled and self.motors_on:
            self.bus.disable()
            self.motors_on = False
        req = self._enable_req
        if req is not None and req.get("result") is None:
            self._do_enable(req, r)
            return
        if self.enabled and self.motors_on and not self.estop.active:
            self._control(now, r)
        self._publish(now, dt_tick, r)

    def _enable_pose(self, r) -> np.ndarray:
        """The pose to hold at enable: the motor-side positions (the motors' MIT loops run on
        those, so the first frame asks for no correction), clamped into the setpoint limits.
        Refuses (409) when an output encoder disagrees with its motor (a wrap or a slipped zero
        would make the first frame snap) or when a joint is further outside its limits than
        ENABLE_LIMIT_TOL (clamping it would snap it back)."""
        kin = self.kin
        names = self.bus.names
        diff = np.abs(r.q - r.q_mot)
        for i in np.flatnonzero(diff > ENC_MISMATCH_ENABLE):
            raise RequestError(409, f"{names[i]}: output encoder {math.degrees(r.q[i]):+.1f}° and motor "
                                    f"{math.degrees(r.q_mot[i]):+.1f}° disagree by {math.degrees(diff[i]):.1f}° "
                                    "(encoder wrap or slipped zero): power-cycle the arm; not enabling")
        if abs(r.g_q - r.g_q_mot) > ENC_MISMATCH_ENABLE:
            raise RequestError(409, f"gripper: output encoder and motor disagree by "
                                    f"{math.degrees(abs(r.g_q - r.g_q_mot)):.1f}°: power-cycle the arm; not enabling")
        out = np.maximum(kin.lo - r.q_mot, r.q_mot - kin.hi)
        for i in np.flatnonzero(out > ENABLE_LIMIT_TOL):
            raise RequestError(409, f"{names[i]} is at {math.degrees(r.q_mot[i]):+.1f}°, "
                                    f"{math.degrees(out[i]):.1f}° outside its limits "
                                    f"{math.degrees(kin.lo[i]):+.1f}..{math.degrees(kin.hi[i]):+.1f}°: "
                                    "move it back by hand first (enabling would snap it back)")
        return np.clip(r.q_mot, kin.lo, kin.hi)

    def _do_enable(self, req: dict, r) -> None:
        try:
            if self.estop.active or self.chocks() or not self.lease.held:
                raise RequestError(409, "e-stop, chock or lease changed while enabling")
            if not self.initialized:
                self.state = "initializing"
                self.hub.state_changed()
                g = self.bus.check_gripper()
                self.initialized = True
                self.events.add(f"gripper check passed (closed reads {math.degrees(g):+.1f}°)")
            if req.get("cancel"):
                raise RequestError(409, f"cancelled: {req['cancel']}")
            self._enable_pose(self.bus.read())          # refuse before any motor is switched on
            self.bus.enable()
            r = self.bus.read()                          # fresh: the arm may have moved meanwhile
            q0 = self._enable_pose(r)
            with self._lock:
                self.q_cmd = q0
                self.v_cmd = np.zeros(N)
                self.hold_q = None
                self.qd_ff = np.zeros(N)
                self.track, self.motion = None, None
                self.gripper.goal = self.gripper.cmd = float(np.clip(C.rad_to_gap(r.g_q_mot), 0.0, C.GAP_CMD_MAX))
                self.gripper.v = 0.0
                self.gripper.max_force = C.DEFAULT_MAX_FORCE
            self.motors_on = self.enabled = True
            self.state = "enabled"
            now = self.clock()
            self._last_t = self._last_send = now
            self._control(now, r)                        # the first hold frame right away
            self.events.add("enabled: holding the measured pose")
            req["result"] = (True, None, 200)
        except Exception as e:  # noqa: BLE001
            msg = f"{type(e).__name__}: {e}" if not isinstance(e, RequestError) else str(e)
            self.enabled = False
            if self.state != "fault":
                self.state = "uninitialized" if not self.initialized else "disabled"
            try:
                self.bus.disable()
            except Exception:  # noqa: BLE001
                pass
            self.motors_on = False
            self.events.add(f"enable failed: {msg}", "error")
            req["result"] = (False, msg, e.status if isinstance(e, RequestError) else 503)
        finally:
            with self._lock:
                ev = req.get("event")
            if ev is not None:
                ev.set()

    def _command_step(self, q_goal: np.ndarray, v_goal: np.ndarray) -> None:
        """Move q_cmd one tick toward q_goal (call under _lock): joint speed and acceleration
        limited, braking so it stops at the goal, the goal's own velocity (trajectory or twist)
        fed forward; never outside the joint limits. Every source goes through here, so no source
        change, cancel or preemption can step the commanded velocity."""
        kin, s = self.kin, self.soft
        q_goal = np.clip(q_goal, kin.lo, kin.hi)
        e = q_goal - self.q_cmd
        corr = np.sign(e) * np.minimum(np.sqrt(2.0 * s.joint_acc * np.abs(e)), np.abs(e) / DT)
        want = v_goal + corr
        v_des = np.clip(want, -s.joint_vel, s.joint_vel)
        v = np.clip(v_des, self.v_cmd - s.joint_acc * DT, self.v_cmd + s.joint_acc * DT)
        self._limited = bool(np.any(np.abs(v - want) > 1e-9))
        q_new = self.q_cmd + v * DT
        q_c = np.clip(q_new, kin.lo, kin.hi)
        v[q_c != q_new] = 0.0
        self.q_cmd, self.v_cmd = q_c, v

    def _stopping_point(self) -> np.ndarray:
        """Where q_cmd comes to rest decelerating at the acceleration limit (call under _lock)."""
        a = self.soft.joint_acc
        return np.clip(self.q_cmd + self.v_cmd * np.abs(self.v_cmd) / (2.0 * a), self.kin.lo, self.kin.hi)

    def _gripper_step(self, gap_goal: Optional[float]) -> None:
        """Gap command toward its goal: rate and acceleration limited with braking, never past
        GAP_CMD_MAX (1 mm short of the printed stops). Call under _lock."""
        g, s = self.gripper, self.soft
        if gap_goal is not None:
            g.goal = gap_goal
        goal = min(max(g.goal, 0.0), C.GAP_CMD_MAX)
        e = goal - g.cmd
        vd = math.copysign(min(s.gripper_vel, math.sqrt(2.0 * s.gripper_acc * abs(e)), abs(e) / DT), e)
        g.v = min(max(vd, g.v - s.gripper_acc * DT), g.v + s.gripper_acc * DT)
        g.cmd = min(max(g.cmd + g.v * DT, 0.0), C.GAP_CMD_MAX)

    def _control(self, now: float, r) -> None:
        kin, p = self.kin, self.profile
        if now - self._last_send > LATE_SEND:
            self._fault(f"{(now - self._last_send) * 1e3:.0f} ms since the last command frame "
                        "(the motors' watchdog is 100 ms)")
            return
        with self._lock:
            src, q_goal, v_goal, gap_goal = self._source(now)
            self._command_step(q_goal, np.clip(v_goal, -self.soft.joint_vel, self.soft.joint_vel))
            q_cmd, v_cmd = self.q_cmd, self.v_cmd
            self.qd_ff = v_cmd
            self._gripper_step(gap_goal)
            g_cmd, max_force = self.gripper.cmd, self.gripper.max_force
        tau_g = kin.gravity(r.q) * p.tau_scale
        # Gripper: the torque its MIT law applies (motor side; no velocity reference, so kd only
        # damps), and the force limit. Away from fully open: an opposing feedforward, never larger
        # than that torque (so the net torque never opens). Within FF_OPEN_MARGIN of fully open:
        # no feedforward; the position command is pulled back toward the jaws instead.
        g_q = C.gap_to_rad(g_cmd)
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
        self._g = {"tau_est": tau_est, "force": force, "ff": g_ff, "limiting": limiting, "tau_g": tau_g}
        try:
            self.bus.send(q_cmd, v_cmd, p.kp, p.kd, tau_g, g_q, 0.0, p.g_kp, p.g_kd, g_ff)
        except BusError as e:
            self._fault(str(e))
            return
        self._last_send = now
        self._supervise(now, r, src)

    def _source(self, now: float):
        """(source, q_goal, goal velocity, gripper gap goal or None); call under _lock."""
        m = self.motion
        if m is not None and m.phase in ("executing", "settling"):
            q, qd, gap, done = m.traj.sample(now)
            if done and m.phase == "executing":
                m.phase, m.t_end = "settling", now
                if m.traj.kind == "chunk":
                    self._end_motion("done")
            self._goal_pose = m.traj.goal
            return m.traj.kind, q, qd, gap
        t = self.track
        if t is not None:
            # The pose was the goal when it was sent; a fresh twist says how it moves on from there,
            # so the goal runs ahead along it (at most TWIST_TIMEOUT worth, twist capped at the soft
            # servo speeds). When the twist times out the goal is the pose again.
            qd, q_goal = np.zeros(N), t.q_goal
            age = now - t.t
            if t.twist is not None and age <= TWIST_TIMEOUT:
                qd = np.clip(self.kin.twist_to_qd(t.q_goal, t.twist, t.twist_frame),
                             -self.soft.joint_vel, self.soft.joint_vel)
                q_goal = t.q_goal + qd * max(age, 0.0)
            self._goal_pose = t.pose
            return "track", q_goal, qd, None
        self._goal_pose = None
        return "hold", (self.q_cmd if self.hold_q is None else self.hold_q), np.zeros(N), None

    def _supervise(self, now: float, r, src: str) -> None:
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
                    self.q_cmd = np.clip(r.q_mot, self.kin.lo, self.kin.hi)
                    self.v_cmd = np.zeros(N)
                self._end_motion("aborted", f"tracking error {dp * 1e3:.0f} mm / {math.degrees(dr):.0f}° "
                                            "(obstructed?): holding")
                return
        else:
            m.bad_since = None
        if m.phase == "settling" and m.traj.goal is not None:
            ep, er = pose_error(self.kin.fk(r.q), m.traj.goal)
            if ep <= s.reach_pos and er <= s.reach_rot:
                m.reached_since = m.reached_since or now
                if now - m.reached_since >= s.reach_dwell_s:
                    self._end_motion("reached")
            else:
                m.reached_since = None
                if now - m.t_end > s.settle_timeout_s:
                    self._end_motion("not_reached", f"{ep * 1e3:.1f} mm / {math.degrees(er):.2f}° from the goal "
                                                    f"{s.settle_timeout_s:g} s after the planned end: holding")

    def _end_motion(self, phase: str, reason: Optional[str] = None) -> None:
        with self._lock:
            m = self.motion
            if m is None:
                return
            m.phase, m.reason = phase, reason
            self.last_motion, self.motion = m, None
            self.hold_q = m.traj.q[-1].copy() if phase in ("done", "reached", "not_reached") else self._stopping_point()
        self.events.add(f"{m.traj.kind} {m.traj.id}: {phase}" + (f" ({reason})" if reason else ""),
                        "warn" if phase in ("aborted", "not_reached") else "info")

    def _publish(self, now: float, dt_tick: float, r) -> None:
        self._tick += 1
        kin = self.kin
        nan = math.nan
        info = self.lease.info
        with self._lock:
            q_cmd = None if self.q_cmd is None else self.q_cmd.copy()
            qd_ff = self.qd_ff.copy()
            m = self.motion or self.last_motion
            src = self._source_name()
            tr = self.track
            g = self.gripper
            seq = tr.seq if tr else 0
            goal = self._goal_pose
        row = np.full(len(FIELDS), nan)
        row[:len(_BASE)] = [now, self._tick, STATES.index(self.state), float(self.enabled), float(self.motors_on),
                            float(self.estop.active), float(info.lease_id if info else 0), SOURCES.index(src),
                            seq, self.overruns, dt_tick, float(self._limited and self.motors_on),
                            (now - tr.t) if tr and tr.twist is not None else nan]
        on = self.motors_on
        tau_g = self._g.get("tau_g") if on else None
        p = self.profile
        for i in range(N):
            b = len(_BASE) + i * len(JOINT_COLS)
            qc = q_cmd[i] if q_cmd is not None and on else nan
            e = qc - r.q_mot[i]
            tg = tau_g[i] if tau_g is not None else nan
            row[b:b + len(JOINT_COLS)] = [r.q[i], r.q_mot[i], r.qd[i], r.tau[i], qc, qd_ff[i] if on else nan, tg,
                                          e, p.kp[i] * e + p.kd[i] * (qd_ff[i] - r.qd[i]) + tg,
                                          r.t_mos[i], r.status[i]]
        T = kin.fk(r.q)
        J = kin.jacobian(r.q, "base")
        mn = kin.manipulability(J)
        quat = pin.Quaternion(T.rotation)
        b = COL["x"]
        row[b:b + 7] = [*T.translation, quat.x, quat.y, quat.z, quat.w]
        if q_cmd is not None and on:
            Tc = kin.fk(q_cmd)
            row[COL["cmd_x"]:COL["cmd_x"] + 3] = Tc.translation
            row[COL["err_pos"]], row[COL["err_rot"]] = pose_error(T, Tc)
            if goal is not None:
                row[COL["goal_err_pos"]], row[COL["goal_err_rot"]] = pose_error(Tc, goal)
        row[COL["w"]], row[COL["w_lin"]], row[COL["sigma_min"]] = mn["w"], mn["w_lin"], mn["sigma_min"]
        gd = self._g
        row[COL["g_gap"]:COL["g_gap"] + len(_GRIP)] = [
            C.rad_to_gap(r.g_q), g.cmd if (on and g.cmd is not None) else nan,
            g.goal if (on and g.goal is not None) else nan, r.g_qd, r.g_tau,
            gd["tau_est"] if on else nan, gd["force"] if on else nan, gd["ff"] if on else nan, g.max_force,
            float(gd["limiting"] and on), r.status[N]]
        if m is not None:
            mp = m.public(now)
            row[COL["motion_id"]], row[COL["phase"]], row[COL["progress"]] = m.traj.id, PHASES.index(m.phase), mp["progress"]
        else:
            row[COL["phase"]] = 0
        self.hub.publish_rows(row[None], "tick")
        self.hub.publish_rows(np.concatenate([[now], J.reshape(-1)])[None], "jacobian")

    def _source_name(self) -> str:
        m = self.motion
        if m is not None and m.phase in ("executing", "settling"):
            return m.traj.kind
        return "track" if self.track is not None else "hold"

    # -- safety ------------------------------------------------------------------------------------

    def chocks(self) -> list:
        return self.locks.matching("chock", self.lock_names)

    def _disable(self, why: str) -> None:
        was = self.enabled
        with self._lock:
            self.enabled = False
            self.track = None
            if self.motion is not None:
                m = self.motion
                m.phase, m.reason = "aborted", why
                self.last_motion, self.motion = m, None
            req = self._enable_req
            if req is not None and req.get("result") is None:
                req["cancel"] = why
        if self.state == "enabled":
            self.state = "disabled"
        if was:
            self.events.add(why)

    def _fault(self, msg: str) -> None:
        self._disable(f"FAULT: {msg}: motors disabled")
        if self.bus is not None and self.motors_on:
            try:
                self.bus.disable()
            except Exception:  # noqa: BLE001
                pass
        self.motors_on = False
        if self.state != "fault":
            self.events.add(f"fault: {msg}", "error")
        self.state, self.error = "fault", msg

    def _lease_ended(self, reason: str) -> None:
        self.soft = C.DEFAULT_SOFT.copy()
        self._disable(f"lease {reason}: disabled")
        self.events.add(f"lease {reason}")

    def _estop_triggered(self, info: EStopInfo) -> None:
        self._disable(f"E-STOP ({info.source}: {info.reason}): motors disabled")

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
        if not self.enabled:
            raise RequestError(409, f"not enabled ({self.state}; POST /enable)")

    def enable(self, token, timeout: float = ENABLE_TIMEOUT) -> dict:
        """Blocking (call from a worker thread): runs the gripper check on the first enable."""
        self.lease.renew(token)
        ch = self.chocks()
        if ch:
            raise RequestError(423, f"chocked: {describe_locks(ch)}. An admin removes it on the robot: "
                                    "python -m robot_utils.admin.locks remove <id>")
        if self.estop.active:
            raise RequestError(409, "e-stop is latched; clear it first (POST /estop/clear)")
        if self.state in ("connecting", "fault") or self.bus is None:
            raise RequestError(503, f"arm not ready ({self.state}: {self.error}); POST /reset after a fault")
        if self.enabled:
            return {"enabled": True, "initialized": self.initialized}
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
        return {"enabled": True, "initialized": self.initialized}

    def disable(self, token) -> None:
        self.lease.renew(token)
        self._disable("disabled (motors off)")

    def trigger_estop(self, reason: str, source: str) -> EStopInfo:
        return self.estop.trigger(reason, source)

    def clear_estop(self, token) -> None:
        self.lease.renew(token)
        if self.estop.active:
            self.estop.clear()
            self.events.add("e-stop cleared (still disabled: POST /enable)")

    def reset(self, token) -> None:
        self.lease.renew(token)
        self._disable("reset: reconnecting")
        self.error = None
        self._reconnect = True
        if self.state != "connecting":
            self.state = "connecting"

    # motion ---------------------------------------------------------------------------------------

    def _preempt(self, by: str) -> None:
        """Call under _lock."""
        if self.motion is not None:
            m = self.motion
            m.phase, m.reason = "preempted", f"by {by}"
            self.last_motion, self.motion = m, None
            self.events.add(f"{m.traj.kind} {m.traj.id}: preempted by {by}")

    def track_cmd(self, token, pose: pin.SE3, twist=None, twist_frame: str = "base", seq: int = 0,
                  gripper: Optional[dict] = None) -> dict:
        self._ready(token)
        if twist is not None:
            twist = np.asarray(twist, dtype=float)
            if twist.shape != (6,) or not np.all(np.isfinite(twist)):
                raise ValueError("twist must be 6 finite numbers [vx, vy, vz, wx, wy, wz]")
        if twist_frame not in ("base", "tcp"):
            raise ValueError("twist_frame must be base or tcp")
        scaled = False
        if twist is not None:          # bounded by the soft servo speeds (it feeds the command forward)
            lin, ang = float(np.linalg.norm(twist[:3])), float(np.linalg.norm(twist[3:]))
            if lin > self.soft.lin_vel:
                twist[:3] *= self.soft.lin_vel / lin
                scaled = True
            if ang > self.soft.ang_vel:
                twist[3:] *= self.soft.ang_vel / ang
                scaled = True
        g_args = self._check_gripper(**gripper) if gripper is not None else None
        with self._lock:
            prev = self.track
            seed = prev.q_goal if prev is not None else self.q_cmd
        if prev is not None and seq and prev.seq and seq < prev.seq:
            return {"ok": False, "seq": seq, "ik": None, "limited": self._limited,
                    "reason": f"stale: seq {seq} is older than {prev.seq} already applied"}
        r = self.kin.ik(pose, seed, max_iter=50)
        out = {"ok": r.ok, "seq": seq, "ik": r.public(), "limited": self._limited, "twist_scaled": scaled,
               "reason": r.reason}
        if r.ok:
            jump = float(np.max(np.abs(r.q - seed)))
            if jump > TRACK_MAX_JUMP:
                out.update(ok=False, reason=f"too far for track: a joint would move {math.degrees(jump):.0f}° "
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
        """Call under _lock. An omitted force limit keeps the one in force (50 N after enable)."""
        if max_force is not None:
            self.gripper.max_force = max_force
        if gap is not None:
            self.gripper.goal = gap

    def gripper_cmd(self, token, gap_mm: Optional[float] = None, max_force_n: Optional[float] = None) -> dict:
        self._ready(token)
        args = self._check_gripper(gap_mm, max_force_n)
        with self._lock:
            self._apply_gripper(*args)
            return self.gripper_state()

    def _start(self, traj: Trajectory, by: str, q0: np.ndarray) -> dict:
        """Put a planned motion in force. It was planned from ``q0``; if the command moved since
        (a track or motion kept running while planning), a servo / pose / home is refused rather
        than started from a stale pose, and a chunk's lead-in is re-anchored on the command now and
        its speed checked."""
        with self._lock:
            if not self.enabled:
                raise RequestError(409, "disabled while planning")
            now = self.clock()
            if traj.kind == "chunk":
                # samples that came due while planning are dropped; the lead-in starts here, now
                k = int(np.searchsorted(traj.t[1:], now + max(traj.info.get("blend_s", 0.0), DT), side="left")) + 1
                if k >= len(traj.t):
                    raise RequestError(422, "refused, nothing moved: the chunk went stale while planning")
                keep = np.r_[0, np.arange(k, len(traj.t))]
                traj.t, traj.q, traj.qd = traj.t[keep], traj.q[keep], traj.qd[keep]
                if traj.gap is not None:
                    traj.gap = traj.gap[keep]
                traj.q[0], traj.t[0], traj.qd[0] = self.q_cmd.copy(), now, 0.0
                if len(traj.t) > 1:
                    v = np.abs(traj.q[1] - traj.q[0]) / max(traj.t[1] - traj.t[0], 1e-6)
                    if np.any(v > self.soft.joint_vel):
                        j = int(np.argmax(v / self.soft.joint_vel))
                        raise RequestError(422, f"refused, nothing moved: the chunk's first sample is too far from "
                                                f"the arm's command for its blend (joint{j + 1} at "
                                                f"{v[j]:.2f} rad/s); increase blend_s or start nearer")
            else:
                moved = float(np.max(np.abs(self.q_cmd - q0)))
                if moved > PLAN_DRIFT:
                    raise RequestError(409, f"refused, nothing moved: the arm's command moved "
                                            f"{math.degrees(moved):.1f}° while planning (a track or motion is in "
                                            "force); cancel it first, or retry")
                traj.shift(now - traj.t[0])
            self._preempt(by)
            self.track = None
            self.hold_q = None
            self.motion = Motion(traj)
            out = self.motion.public(now)
        self.events.add(f"{traj.kind} {traj.id}: {traj.duration:.2f} s" + (f" to {traj.name}" if traj.name else ""))
        return out

    def _plan(self, fn):
        """Plans run one at a time (each holds the interpreter for a while; several at once would
        starve the control loop)."""
        if not self._plan_lock.acquire(blocking=False):
            raise RequestError(409, "already planning another motion; retry")
        try:
            return fn()
        except PlanError as e:
            raise RequestError(422, f"refused, nothing moved: {e}")
        finally:
            self._plan_lock.release()

    def servo(self, token, pose: Optional[pin.SE3] = None, name: Optional[str] = None,
              lin_vel=None, lin_acc=None, ang_vel=None, ang_acc=None, duration=None) -> dict:
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
            q0 = self.q_cmd.copy()
        if name is not None:
            traj = self._plan(lambda: plan_joint(q0, [self.settings.named_poses[name]], s.home_vel, s.joint_vel,
                                                 self.kin, kind="pose", name=name))
        else:
            traj = self._plan(lambda: plan_servo(self.kin, q0, pose, joint_vel=s.joint_vel, duration=duration, **lim))
        return self._start(traj, "servo", q0)

    def home(self, token) -> dict:
        self._ready(token)
        st = self.settings
        with self._lock:
            q0 = self.q_cmd.copy()
        way = [st.named_poses[n] for n in st.home_via] + [st.named_poses["home"]]
        traj = self._plan(lambda: plan_joint(q0, way, self.soft.home_vel, self.soft.joint_vel, self.kin,
                                             kind="home", name="home"))
        return self._start(traj, "home", q0)

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
            q_now = self.q_cmd.copy()
        now = self.clock()
        traj = self._plan(lambda: plan_chunk(
            self.kin, q_now, poses, None if gaps_mm is None else [g / 1000 for g in gaps_mm],
            t0=now if t0 is None else float(t0), dt=dt, now=now, blend_s=blend_s,
            joint_vel=self.soft.joint_vel, q_now=q_now))
        return self._start(traj, "chunk", q_now)

    def cancel(self, token) -> dict:
        self._ready(token)
        with self._lock:
            m = self.motion
            self.track = None
            self.hold_q = self._stopping_point()
            if m is not None:
                m.phase, m.reason = "aborted", "cancelled: decelerating to a hold"
                self.last_motion, self.motion = m, None
        if m is not None:
            self.events.add(f"{m.traj.kind} {m.traj.id}: cancelled")
        return {"cancelled": m is not None}

    # settings --------------------------------------------------------------------------------------

    def set_profile(self, token, name: str) -> dict:
        self.lease.renew(token)
        if self.enabled:
            raise RequestError(409, "switch gain profiles only while disabled")
        self.profile = C.load_profile(name)
        self.events.add(f"gain profile {name}")
        return self.profile.public()

    def set_tcp(self, token, offset) -> dict:
        self.lease.renew(token)
        if self.enabled:
            raise RequestError(409, "change the TCP offset only while disabled")
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

    def ik_check(self, pose: pin.SE3, seed: str = "command") -> dict:
        kin = self._need_kin()
        r = self.reading
        if seed == "measured" or self.q_cmd is None:
            if r is None:
                raise RequestError(503, "no reading yet")
            q0 = r.q
        else:
            q0 = self.q_cmd
        out = kin.ik(pose, q0)
        return out.public()

    def kinematics(self, q, frame: str = "base") -> dict:
        kin = self._need_kin()
        q = np.asarray(q, float)
        if q.shape != (N,) or not np.all(np.isfinite(q)):
            raise ValueError(f"a configuration is {N} finite joint angles (rad)")
        T = kin.fk(q)
        J = kin.jacobian(q, frame)
        return {"q": q.tolist(), "within_limits": kin.within_limits(q), "tcp": pose_public(T),
                "frame": frame, "jacobian": J.round(9).tolist(), **kin.manipulability(J)}

    def kinematics_now(self, frame: str = "base", which: str = "measured") -> dict:
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        q = r.q if which == "measured" or self.q_cmd is None else self.q_cmd
        out = self.kinematics(q, frame)
        out["which"] = which
        return out

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
        return {"gap_mm": None if r is None else round(C.rad_to_gap(r.g_q) * 1000, 2),
                "cmd_mm": None if g.cmd is None else round(g.cmd * 1000, 2),
                "goal_mm": None if g.goal is None else round(g.goal * 1000, 2),
                "max_force_n": g.max_force, "force_n": finite(round(self._g["force"], 2)) if self.motors_on else None,
                "limiting": bool(self._g["limiting"] and self.motors_on), "gap_max_mm": round(C.GAP_MAX * 1000, 2),
                "default_max_force_n": C.DEFAULT_MAX_FORCE}

    def joints(self) -> dict:
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        names = self.bus.names if self.bus else [f"joint{i}" for i in range(1, N + 1)]
        lo = hi = None
        if self.kin is not None:
            lo, hi = self.kin.lo.tolist(), self.kin.hi.tolist()
        return {"names": names[:N], "q": r.q.tolist(), "q_mot": r.q_mot.tolist(), "qd": r.qd.tolist(),
                "tau": r.tau.tolist(), "q_cmd": None if self.q_cmd is None else self.q_cmd.tolist(),
                "status": r.status.tolist(), "t_mos": r.t_mos.tolist(), "t_rotor": r.t_rotor.tolist(),
                "limits": {"lo": lo, "hi": hi}, "t": r.t}

    def tracking(self) -> dict:
        last = self.hub.recent(1, "tick")
        if not len(last):
            raise RequestError(503, "no tick yet")
        row = last[0]
        g = lambda n: None if not math.isfinite(row[COL[n]]) else float(row[COL[n]])   # noqa: E731
        return {"e_mot": [g(f"j{i}_e_mot") for i in range(1, N + 1)],
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
            tr = self.track
            motion = m.public(now) if m else None
        ch = self.chocks()
        return finite({
            "state": self.state, "error": self.error, "sim": bool(self.bus and self.bus.sim),
            "initialized": self.initialized, "enabled": self.enabled, "motors_on": self.motors_on,
            "chocked": bool(ch), "locks": [l.public() for l in ch],
            "estop": {"latched": e is not None, "reason": e and e.reason, "source": e and e.source, "t": e and e.t},
            "lease": {"held": info is not None, "holder": info and info.client, "lease_id": info and info.lease_id,
                      "expires_in": None if info is None else round(info.expires - now, 3)},
            "source": self._source_name(),
            "track": None if tr is None else {"seq": tr.seq, "age": round(now - tr.t, 3), "goal": pose_public(tr.pose),
                                              "twist": None if tr.twist is None else tr.twist.tolist(),
                                              "twist_frame": tr.twist_frame},
            "motion": motion,
            "gripper": self.gripper_state(),
            "profile": self.profile.name, "profiles": list(C.PROFILES),
            "tcp_offset": None if self.kin is None else self.kin.tcp_offset,
            "named_poses": {k: np.degrees(v).round(3).tolist() for k, v in self.settings.named_poses.items()},
            "limits": self.limits(),
            "twist_timeout": TWIST_TIMEOUT, "lease_heartbeat": LEASE_HEARTBEAT,
            "loop_hz": None if rate is None else round(rate, 1), "overruns": self.overruns,
            "clients": self.hub.subscribers,
            "events": self.events.list()[-20:],
            "latest": latest,
            "server_mono": now,
        })

    @property
    def status(self) -> str:
        return self.state
