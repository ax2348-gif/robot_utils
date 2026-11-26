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

**Arms**: one end effector per arm of the robot (robots/: ``ee_specs``; the default robot's one EE ``arm``), each with
its own arm node (``arm_targets``: EE -> arm node instance; by the profile's convention the instance is named as the
EE), its own lease, joint channel, state, gains, limits and gap. The datagram, hint and stream formats are per EE
already. Every check, hold, resync, enable and disable covers every arm; the latch and the base are shared. The /state
``arm`` block (and ``arm_target``, ``channel``, ``arm_state``, ... here) is the primary arm (the first); with several
arms /state also has ``arms`` (the same block per EE).
"""
from __future__ import annotations

import logging
import math
import queue
import secrets
import threading
import time
from dataclasses import asdict, dataclass, field
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
from .model import WbcModel, ee_specs, fetch_urdf
from .receiver import Inbox, Receiver
from .swivel_geom import HintInput
from .spec import (MODES, Config, Hints, check_lead_torque, default_arm_gains, hints_public, make_config, merge,
                   parse_base, parse_hints)
from .targets import R_to_quat, TargetBuffer, quat_to_R

log = logging.getLogger("wbc")

STATES = ["disabled", "enabling", "idle", "tracking", "holding", "fault"]
SOLVE = ["none", "ok", "infeasible", "error"]
ARM_GRACE_S = 0.5        # after enable: the arm may still report its old state
SWERVE_GRACE_S = 0.25
CMD_FAIL_S = 0.25        # swerve commands failing this long -> fault
POSTURES = {"home": np.zeros(6), "ready": np.radians([0.0, -40.0, -60.0, 0.0, 30.0, 0.0])}
STRUCTURAL = ("rate_hz", "lease_heartbeat", "joint_vel", "joint_acc", "reach")   # POST /config: only while disabled
BASE_CAPS = ("max_speed", "max_omega", "max_accel", "max_alpha", "wheel_speed")


def stream_fields(ees, dofs=None) -> tuple[list, list, dict]:
    """``dofs``: EE -> its arm's joint count (default 6): the per-joint columns q_cmd / qd / lead_tau 1..n."""
    dofs = dofs or {}
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
        n = dofs.get(e, 6)
        f += [(f"{e}_q_cmd{i}", "rad") for i in range(1, n + 1)] + [(f"{e}_qd{i}", "rad/s") for i in range(1, n + 1)]
    f += [("armed", "bool"), ("tick_period", "ms"), ("tick_compute", "ms"), ("overruns", "count"),
          ("late_ticks", "count")]
    for e in ees:
        f += [(f"{e}_err_meas_pos", "m"), (f"{e}_err_meas_rot", "rad"), (f"{e}_lead_pos", "m"),
              (f"{e}_lead_rot", "rad")]
        f += [(f"{e}_lead_tau{i}", "Nm") for i in range(1, dofs.get(e, 6) + 1)]
    for e in ees:
        f += [(f"{e}_swivel", "rad"), (f"{e}_swivel_w", "")]
    for e in ees:
        f += [(f"{e}_limited_by", "enum")]
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
        "limited_by": "0 none, 1 reach, 2 manipulability, 3 fence, 4 joint_limit, 5 base_limit, 6 base_band, "
                      "7 lead (the lead bound: the command held within lead_torque / kp of the measured arm)",
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
        "<ee>_err_pos": "QP shortfall: |target - COMMANDED| EE position (m; the components would be world axes); "
                        "_err_rot the rotation angle of target vs commanded (rad). Not where the hand is",
        "<ee>_err_meas_pos": "actual error: |target - MEASURED| EE position (m, world axes): where the hand is vs "
                             "where it is wanted; _err_meas_rot the rotation angle (rad). NaN without a measured pose",
        "<ee>_lead_pos": "the servo's deflection: |COMMANDED - MEASURED| EE position (m, world axes): effort / "
                         "contact force (in free motion also the arm's lag); _lead_rot the rotation angle (rad). NaN "
                         "without a measured pose",
        "<ee>_swivel": "a redundant arm's elbow swivel about its shoulder-wrist line at the COMMANDED joints, "
                       "incremental (parallel-transported steps: no fixed zero, an arbitrary origin; the operator's "
                       "swivel hint anchors to it); NaN for a 6-joint arm",
        "<ee>_swivel_w": "the operator's swivel hint's weight in force (0..1: the ramped elbow x conf fades times the "
                         "robot elbow's flexion fade; 0 = none, the swivel model alone); NaN for a 6-joint arm",
        "<ee>_limited_by": "this arm's own limit binding this tick (its rows only; the LIMITED codes, as limited_by): "
                           "0 none, 1 reach, 2 manipulability, 4 joint_limit, 7 lead",
        "<ee>_lead_tau*": "per arm joint kp (q_cmd - q_meas): the servo's push from the lead (Nm; the lead bound "
                          "keeps it within lead_torque in contact, the measured arm extrapolated over its delay). NaN "
                          "without a measured pose",
    }
    return [n for n, _ in f], [u for _, u in f], doc


def parse_swivel_hint(h) -> tuple:
    """A target datagram's per-EE ``swivel`` (docs/wbc.md "The operator's elbow"): ``{"seq": int (the operator's
    engagement id), "dpsi": rad (their swivel change since it), "elbow": rad (their upper-arm-forearm angle, pi =
    straight), "conf": 0..1}`` -> (seq, dpsi, elbow, conf); ValueError otherwise (the datagram is dropped as invalid)."""
    if not isinstance(h, dict) or set(h) - {"seq", "dpsi", "elbow", "conf"}:
        raise ValueError("swivel must be {seq, dpsi, elbow, conf}")
    num = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)   # noqa: E731
    seq = h.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool):
        raise ValueError("swivel seq must be an integer")
    if not all(num(h.get(k)) for k in ("dpsi", "elbow", "conf")):
        raise ValueError("swivel dpsi, elbow and conf must be finite numbers")
    if not 0.0 <= h["elbow"] <= math.pi + 1e-6:
        raise ValueError("swivel elbow must be within 0..pi rad")
    if not 0.0 <= h["conf"] <= 1.0:
        raise ValueError("swivel conf must be within 0..1")
    return seq, float(h["dpsi"]), min(float(h["elbow"]), math.pi), float(h["conf"])


def posture_of(p, spec, n: int) -> np.ndarray:
    """A posture hint (a name or a joint vector) for an arm of ``n`` joints: "ready" is the arm's nominal posture (the
    robot profile's, ``EESpec.posture``) when it has one, "home" zeros; else POSTURES (the reBot's 6-joint ones)."""
    if not isinstance(p, str):
        return np.asarray(p, float)
    if p == "ready" and getattr(spec, "posture", None) is not None:
        return np.asarray(spec.posture, float)
    if p == "home":
        return np.zeros(n)
    q = POSTURES[p]
    if len(q) != n:
        raise ValueError(f"posture {p!r} has {len(q)} joints, the arm {n}")
    return q


def resolve_arms(ees, arm: Optional[str] = None, arms: Optional[dict] = None, arm_socket: Optional[str] = None,
                 arm_sockets: Optional[dict] = None) -> tuple[dict, dict]:
    """(EE -> arm node target, EE -> its joint channel socket or None for a URL). ``arms`` given: as is; else ``arm``
    (falsy: no arm) drives the one EE of a one-EE model, and with several EEs each EE's arm node is named as the EE
    (the robot profile's convention: arm_left, arm_right)."""
    ees = list(ees)
    if arms is not None:
        targets = {e: t for e, t in arms.items() if t}
        bad = set(targets) - set(ees)
        if bad:
            raise ValueError(f"arms for unknown end effectors {sorted(bad)} (have {ees})")
    elif not arm:
        targets = {}
    elif len(ees) == 1:
        targets = {ees[0]: arm}
    else:
        targets = {e: e for e in ees}
    socks = {}
    for e, t in targets.items():
        sk = (arm_sockets or {}).get(e) or (arm_socket if len(targets) == 1 else None)
        socks[e] = sk or (str(joint_socket_path(t)) if "://" not in t else None)
    return targets, socks


def arm_label(e: str, targets) -> str:
    """How messages name an arm: ``arm`` with one (as always), else its EE name."""
    return "arm" if len(targets) <= 1 else e


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


def robot_wbc_defaults(robot=None) -> dict:
    """The robot profile's WBC config defaults (its ``wbc:`` block, e.g. the OpenArm's arm-proximity weight); the
    node's ``config`` overrides them."""
    from ...robots import load
    try:
        return dict(load(robot).wbc)
    except Exception:  # noqa: BLE001 - an unknown robot fails where the model loads, with its own error
        return {}


class WbcNode:
    def __init__(self, model: Optional[WbcModel] = None, *, model_source: str = "description",
                 arm: Optional[str] = "arm", swerve: Optional[str] = "swerve",
                 estimator: Optional[str] = "estimator", config: Optional[dict] = None,
                 clock: Callable[[], float] = time.monotonic, udp_host: Optional[str] = "127.0.0.1",
                 udp_port: int = 9410, unix_socket: Optional[str] = None, arm_socket: Optional[str] = None,
                 name: str = "wbc", arms: Optional[dict] = None, arm_sockets: Optional[dict] = None,
                 robot: Optional[str] = None):
        """``arm`` / ``arm_socket``: the one arm (one-EE model); ``arms`` / ``arm_sockets``: EE -> arm node target /
        joint socket (``resolve_arms``); ``robot``: the robot profile the model's EEs come from (``load_model``)."""
        self.cfg: Config = make_config({**robot_wbc_defaults(robot), **(config or {})})
        kp0, ml0, _ = default_arm_gains()
        check_lead_torque(self.cfg.lead_torque, kp0, ml0)
        self.lead_binds = 0                 # ticks the lead bound bound (any joint; since start)
        self._posture_status: dict = {}       # EE -> the elbow's last limit-aware status (events on change)
        self.swivel_hints: dict = {}          # EE -> (seq, dpsi, elbow, conf, t received): the operator's swivel hint
        self.clock = clock
        self.name = name
        self.swerve_target, self.estimator_target = swerve, estimator
        self.robot = robot
        self._arms_in = (arm, arms, arm_socket, arm_sockets)
        self.arm_targets: dict = {}            # EE -> arm node target (instance name or URL)
        self.arm_sockets: dict = {}            # EE -> its joint channel socket
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
        self.arm_https: dict = {}              # EE -> the arm's HTTP client (its lease) while enabled
        self.arm_lease_ids: dict = {}
        self.channels: dict = {}               # EE -> ArmChannel while enabled
        self.base = None
        self.enabled_at = -math.inf
        self.arm_states: dict = {}             # EE -> the arm's latest joint-channel state
        self.q_cmd: dict = {}
        self.v_cmd: dict = {}
        self.base_twist = np.zeros(3)
        self.cmd_twist: dict = {}
        self.buffers: dict = {}
        self.seq_in_force = 0
        self.last_rx = -math.inf
        self.resyncs = 0
        self.resync = self.cfg.resync
        self.gap_maxes: dict = {}              # EE -> the arm's gap at gripper 1 (its welcome; default GAP_MAX)
        self._paused = False
        # the armed latch (module docstring)
        self.armed = False
        self.arm_seq = 0                 # the arm_seq that armed it (0: none this session)
        self.arm_seq_seen = 0            # the highest arm_seq in this session's datagrams
        self.arm_floor = 0               # arm_seq_seen at the last disarm: arming needs a greater one
        self.disarm_reason: Optional[str] = "not armed yet"
        self.disarms = 0
        self.latch_counts = {"ignored_disarmed": 0, "arm_refused": 0, "arm_far": 0, "limits_ignored": 0,
                             "clamped": 0}
        # control tick timing (D): the measured period and compute time, overruns (> 1.5 dt), late ticks (> 3 dt)
        self.tick = {"period_ms": math.nan, "compute_ms": math.nan, "max_period_ms": 0.0, "max_compute_ms": 0.0,
                     "overruns": 0, "late": 0}
        self._tick_prev: Optional[float] = None
        self._est_health: Optional[str] = None   # the estimator check of the newest enabled tick (None: usable)
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

    # -- arms (the primary arm under the one-arm names) -------------------------------------------------------

    @property
    def primary(self) -> str:
        """The primary EE: the first with an arm, else the first."""
        return next(iter(self.arm_targets), self.ees[0] if self.ees else "arm")

    @property
    def arm_target(self) -> Optional[str]:
        return self.arm_targets.get(self.primary)

    @property
    def arm_socket(self) -> Optional[str]:
        return self.arm_sockets.get(self.primary)

    @property
    def arm_http(self):
        return self.arm_https.get(self.primary)

    @property
    def channel(self) -> Optional[ArmChannel]:
        return self.channels.get(self.primary)

    @property
    def arm_state(self):
        return self.arm_states.get(self.primary)

    @property
    def arm_lease_id(self) -> Optional[int]:
        return self.arm_lease_ids.get(self.primary)

    @property
    def gap_max(self) -> float:
        from ..arm.config import GAP_MAX
        return self.gap_maxes.get(self.primary, GAP_MAX)

    def _label(self, e: str) -> str:
        return arm_label(e, self.arm_targets)

    # -- model ---------------------------------------------------------------------------------------------

    def _setup_model(self) -> None:
        from ..swerve.constants import MODULES
        self.ctl = QPController(self.model, self.cfg, [(m.x, m.y) for m in MODULES])
        self.ees = list(self.model.ees)
        self.arm_targets, self.arm_sockets = resolve_arms(self.ees, *self._arms_in)
        self.fields, self.units, self.field_doc = stream_fields(self.ees, self.dofs())
        self.col = {n: i for i, n in enumerate(self.fields)}
        self.hub = RowHub({"tick": len(self.fields)}, recent=500)
        self.events.on_add = self.hub.state_changed
        self.buffers = {e: TargetBuffer(e, self.cfg.ff_timeout, self.cfg.blend_tau) for e in self.ees}

    def load_model(self) -> None:
        urdf, how = fetch_urdf(self.model_source, self.robot)
        self.model = WbcModel(urdf, ee_specs(self.robot))
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
        arms = (self.arm_target if len(self.arm_targets) <= 1 else
                ", ".join(f"{e}={t}" for e, t in self.arm_targets.items()))
        self.events.add(f"wbc started (model: {self.model_info}; arm {arms}, swerve {self.swerve_target}, "
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
            sid, floor = self._open_session_locked(token)
        return {"session": sid, "udp_port": self.receiver.udp_port, "unix_socket": self.receiver.unix_path,
                "arm_seq": floor}

    def _open_session_locked(self, token: Optional[str]) -> tuple[int, int]:
        sid = secrets.randbits(31) or 1
        self.session_token = token
        self.receiver.open_session(sid)
        self.inbox.drain()                           # the previous session's datagrams never apply
        self.swivel_hints.clear()
        self._disarm_locked("new session", note=True)
        floor = self.arm_floor                       # kept for the node's life: arming needs a greater arm_seq
        self.events.add(f"session {sid} opened (arm_seq floor {floor})")
        return sid, floor

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
        """Arming is possible: enabled, no fault, no e-stop, the arm not paused by a planned motion, the estimator
        usable."""
        return (self.enabled and self.fault is None and not self.estop.active and not self._paused
                and self._est_health is None)

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
        if arming and why is None:
            far = self._arming_too_far(now)
            if far:                                  # not a drop; this press never arms (a new arm_seq is needed)
                self.arm_floor = max(self.arm_floor, aseq)
                self.latch_counts["arm_far"] += 1
                if now - self._last.get("arm_far_ev", -math.inf) > 1.0:   # at most one event a second
                    self._last["arm_far_ev"] = now
                    self.events.add(f"arm refused: {far}", "warn")
                for b in self.buffers.values():
                    b.clear()
                self.override = {}
                return None
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

    def _arming_too_far(self, now: float) -> Optional[str]:
        """The arming datagram's targets (as they apply now) against the commanded EE: the reason it is too far, or
        None. A chunk is blended in from the commanded pose, so it always passes."""
        cfg = self.cfg
        for e, b in self.buffers.items():
            if b.kind is None or e not in self.q_cmd:
                continue
            tg = b.sample(now)
            T, _ = self._cmd_pose(e, now)
            dp = float(np.linalg.norm(tg.p - T.translation))
            dr = float(np.linalg.norm(pin.log3(tg.R @ T.rotation.T)))
            if dp > cfg.arm_target_pos or dr > cfg.arm_target_rot:
                return (f"{e} target {dp * 1e3:.0f} mm / {math.degrees(dr):.1f}° from the commanded pose (more than "
                        f"{cfg.arm_target_pos * 1e3:.0f} mm / {math.degrees(cfg.arm_target_rot):.1f}°: a stale pose?)")
        return None

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
            hints = {k: parse_swivel_hint(e["swivel"]) for k, e in ees.items() if e.get("swivel") is not None}
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
            for k, h in hints.items():                    # absent: the last one ages (stale after the timeout)
                self.swivel_hints[k] = (*h, now)
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
        """The measured period since the previous tick: counts overruns (> 1.5 dt) and late ticks (> cfg.late_tick, which
        disarm); returns (the integration step, the measured period clamped to [dt, 3 dt]; the late period or None)."""
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
        if period > max(self.cfg.late_tick, 3 * dt):
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
        for e, ch in self.channels.items():
            s = ch.poll(now)
            if s is not None:
                self.arm_states[e] = s
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
            if self.estop.active:                    # e-stopped, until the disable: the arm holds, the base gets
                self._disarm_locked("e-stop")        # nothing (no braking command: it coasts on its own e-stop)
                if self.base is not None:
                    self.base.mute()
                self._apply_messages(now)
                self._send_hold(arm=True, base=False)
                self._publish(now, None, hold=True)
                return
            # the estimator: resets and jumps abort the world-frame targets and disarm; unusable (stale, non-finite,
            # too old) -> disarm too (like every automatic hold: nothing resumes on its own) and no arming meanwhile
            health = self.estimator_health(now)
            self._est_health = health
            if health:
                self._disarm_locked("estimator: " + health)
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
            # (one arm paused pauses the session: every arm holds, the paused ones follow their measured joints)
            sts = dict(self.arm_states)
            paused_by = {e: bool(s is not None and s.paused) for e, s in sts.items()}
            paused = any(paused_by.values())
            if paused != self._paused:
                self._paused = paused
                who = "arm" if len(self.arm_targets) <= 1 else ", ".join(e for e, v in paused_by.items() if v) or "arm"
                self.events.add(f"{who} session paused by a planned motion: holding" if paused else
                                "arm session resumed: commands resynced to the measured joints", "warn")
                if paused:
                    self._disarm_locked(f"{who} paused by a planned motion")
                if not paused:
                    synced = False
                    for e, s in sts.items():
                        if s is not None and np.all(np.isfinite(s.q)):
                            self.q_cmd[e], self.v_cmd[e] = s.q.copy(), np.zeros(len(s.q))
                            synced = True
                    if synced:
                        self.ctl.reset()
            self._apply_messages(now)
            if self.armed and now - self.last_rx > self.cfg.hold_timeout:
                self._disarm_locked(f"input stale ({now - self.last_rx:.2f} s without a datagram)")
            hints = self.effective_hints()
            # resync the integration to the measured joints past a threshold (well inside the arm's lead bound)
            for e, s in sts.items():
                if s is not None and e in self.q_cmd and np.all(np.isfinite(s.q)) and not paused:
                    if np.max(np.abs(s.q - self.q_cmd[e])) > self.resync:
                        self.resyncs += 1
                        self.events.add(f"{self._label(e)} command resynced to the measured joints (off by "
                                        f"{np.degrees(np.max(np.abs(s.q - self.q_cmd[e]))):.1f}°)", "warn")
                        self.q_cmd[e] = s.q.copy()
                        self.v_cmd[e] = np.zeros(len(s.q))
            stale = now - self.last_rx > self.cfg.hold_timeout and any(b.kind for b in self.buffers.values())
            no_client = not self.lease.held
            why_hold = ("arm paused by a planned motion" if paused else ("estimator: " + health) if health else
                        ("no client lease" if no_client else ("input stale" if stale else
                                                              None if self.armed else
                                                              f"disarmed: {self.disarm_reason}")))
            base, est_age = self.base_pose(now)
            targets = {e: (None if why_hold or not self.armed else self.buffers[e].sample(now)) for e in self.ees}
            meas = self._measured(now, paused)
            snap = Snapshot(now, base, self.base_twist.copy(),
                            {e: EEState(self.q_cmd[e].copy(), self.v_cmd[e].copy(), targets[e],
                                        hints.ee(e, self.cfg), self._posture(hints, e),
                                        *meas.get(e, (None, None, 0.0)), swivel_hint=self._swivel_hint(e, now))
                             for e in self.ees},
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
            if paused:                                       # the planned motion moves the arm, not the WBC
                for e, s in sts.items():
                    if s is not None and paused_by[e] and np.all(np.isfinite(s.q)):
                        self.q_cmd[e], self.v_cmd[e] = s.q.copy(), np.zeros(len(s.q))
            self.base_twist = np.asarray(out.base_twist, float)
            self._cmd_twists(base)
            if out.limited_by != "none":
                self.limited_counts[out.limited_by] = self.limited_counts.get(out.limited_by, 0) + 1
            if out.diag.get("lead_binding"):
                self.lead_binds += 1
            for e, sw in (out.diag.get("swivel") or {}).items():     # the elbow's limit-aware status: an event on change
                st_ = sw.get("status")
                prev = self._posture_status.get(e)
                if st_ in ("ok", "limited", "re-posing", "unreachable") and st_ != prev:
                    self._posture_status[e] = st_
                    if prev is not None or st_ != "ok":
                        msg = {"ok": "elbow back at its human-like angle",
                               "limited": "elbow held inside its room: the human-like angle is beyond the joint limits",
                               "re-posing": "elbow re-posing into its room within the joint limits (the hand stays)",
                               "unreachable": "this hand pose is unreachable from this arm configuration within the "
                                              "joint limits (another IK branch would need the hand to move)"}[st_]
                        self.events.add(f"{e}: {msg}", "warn" if st_ == "unreachable" else "info")
            self.state = ("idle" if not self.armed else "holding" if why_hold else
                          "tracking" if any(t is not None for t in targets.values()) else "idle")
            self._hold_reason = why_hold
            gaps = {}
            for e in self.channels:
                gr = targets.get(e)
                gaps[e] = gr.gripper * self.gap_maxes.get(e, self.gap_max) if (
                    gr is not None and gr.gripper is not None) else math.nan
            self._send(now, gaps, keepalive=paused_by)
            self._publish(now, out, hold=why_hold is not None, targets=targets, base=base, est_age=est_age)

    def _swivel_hint(self, e: str, now: float) -> Optional[HintInput]:
        """The operator's swivel hint for the controller (the latest, with its age) or None (none since the session
        opened; the controller fades a stale one out)."""
        h = self.swivel_hints.get(e)
        return None if h is None else HintInput(*h[:4], max(0.0, now - h[4]))

    def _measured(self, now: float, paused: bool) -> dict:
        """ee -> (q_meas, qd_meas, age) for the lead bound: each arm's latest state (age since it arrived, on this
        clock); none while paused (the commands follow the measured joints then) or without a finite reading."""
        if paused:
            return {}
        out = {}
        for e, st in self.arm_states.items():
            if st is None or not np.all(np.isfinite(st.q)):
                continue
            qd = st.qd if np.all(np.isfinite(st.qd)) else np.zeros(len(st.q))
            age = now - st.t_rx if math.isfinite(st.t_rx) else 0.0
            out[e] = (st.q.copy(), np.asarray(qd, float).copy(), min(max(age, 0.0), 0.1))
        return out

    def dofs(self) -> dict:
        """EE -> its arm's joint count (the model's)."""
        return {e: len(self.model.arm_q[e]) for e in self.ees} if self.model is not None else {}

    def _posture(self, hints: Hints, ee: str) -> np.ndarray:
        p = hints.ee(ee, self.cfg).posture
        q = posture_of(p, self.model.ees[ee], len(self.model.arm_q[ee]))
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

    def _send(self, now: float, gaps, keepalive=False) -> None:
        """Each arm its command (``gaps`` / ``keepalive``: per EE, or one value for every arm), the base its twist."""
        for e, ch in self.channels.items():
            if e not in self.q_cmd:
                continue
            st = self.arm_states.get(e)
            gap = gaps.get(e, math.nan) if isinstance(gaps, dict) else gaps
            ka = bool(keepalive.get(e, False)) if isinstance(keepalive, dict) else bool(keepalive)
            ch.send(self.q_cmd[e], self.v_cmd[e], gap, t_basis=st.t_tick if st is not None else math.nan, keepalive=ka)
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
        if arm:
            for e, ch in self.channels.items():
                if e in self.q_cmd:
                    st = self.arm_states.get(e)
                    ch.send(self.q_cmd[e], np.zeros(len(self.q_cmd[e])), keepalive=st is not None and st.paused)
        if base and self.base is not None:
            self.base.send((0.0, 0.0, 0.0))

    def _check_peers(self, now: float) -> None:
        """Cross-bus checks: an e-stop on either bus e-stops the other; a fault, lease loss or silence on one stops
        the other half (fault)."""
        if self.fault is not None:
            return
        for e, ch in self.channels.items():
            st, lab = self.arm_states.get(e), self._label(e)
            if ch.closed_reason:                              # the arm's bye: lease end, e-stop, disable, replaced
                self._fault_locked(f"{lab} joint channel closed by the arm: {ch.closed_reason}", False, True)
                return
            if now - self.enabled_at > ARM_GRACE_S:
                if st is None or now - st.t_rx > self.cfg.arm_stale:
                    self._fault_locked(f"{lab} state stale (no joint-channel state)", False, True)
                    return
                if st.state_name in ("off", "fault"):
                    if st.state_name == "off":                   # the arm's e-stop puts it off: e-stop the base too
                        # (and the other arms)
                        self._forward_estop(f"{lab} went off (e-stop?)", base=True, arm=len(self.channels) > 1,
                                            skip=e)
                    self._fault_locked(f"{lab} is {st.state_name}", False, True)
                    return
                if st.state_name not in ("floating", "servoing"):
                    self._fault_locked(f"{lab} is {st.state_name}", False, True)
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

    def _forward_estop(self, reason: str, *, base: bool, arm: bool, skip: Optional[str] = None) -> None:
        forward_estop(self.arm_https, self.swerve_target, reason, self.events, base=base, arm=arm, skip=skip)

    # -- publish -------------------------------------------------------------------------------------------

    def _publish(self, now: float, out, hold: bool, targets=None, base=None, est_age=math.nan) -> None:
        nan = math.nan
        if base is None:
            base, est_age = self.base_pose(now)
        h = self.effective_hints()
        row = [now, float(self.seq_in_force), float(STATES.index(self.state)), float(MODES.index(h.mode)),
               out.solve_ms if out else nan, float(SOLVE.index(out.status)) if out else 0.0,
               base[0], base[1], base[2], nan if self.est.reset_id is None else float(self.est.reset_id),
               *self.base_twist]
        m = self.model
        tail = []
        for e in self.ees:
            st = self.arm_states.get(e)
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
        ages = [now - s.t_tick for s in self.arm_states.values() if s is not None]
        row += [max(ages) if ages else nan, est_age, tg_age]          # arm_age: the oldest arm state
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
            nj = len(m.arm_q[e])
            row += list(self.q_cmd.get(e, [nan] * nj)) + list(self.v_cmd.get(e, [nan] * nj))
        tk = self.tick
        row += [float(self.armed), tk["period_ms"], (time.perf_counter() - self._t0) * 1e3, float(tk["overruns"]),
                float(tk["late"])]
        for e in self.ees:
            row += self._meas_columns(e, base, self.arm_states.get(e), targets.get(e) if targets else None)
        d = out.diag if out else {}
        for e in self.ees:
            hs = ((d.get("swivel") or {}).get(e) or {}).get("hint") or {}
            sc = (d.get("swivel_cont") or {}).get(e)
            row += [nan if sc is None else sc, nan if sc is None else hs.get("weight", 0.0)]
        le = d.get("limited_ees") or {}
        row += [float(LIMITED.index(le[e])) if e in le else 0.0 for e in self.ees]
        self._last["out"] = out
        self.hub.publish_rows(np.array([row], dtype=np.float64), "tick")

    def _meas_columns(self, e: str, base, st, tg) -> list:
        """[err_meas_pos, err_meas_rot, lead_pos, lead_rot, lead_tau1..6]: target vs measured, commanded vs
        measured (EE, world), kp (q_cmd - q_meas) per joint. NaN without a measured pose."""
        nan = math.nan
        if st is None or e not in self.q_cmd or not np.all(np.isfinite(st.q)):
            return [nan] * (4 + len(self.model.arm_q[e]))
        m = self.model
        qm = dict(self.q_cmd)
        qm[e] = st.q
        Tm = m.fk(m.config(base, qm), e)
        Tc = m.fk(m.config(base, self.q_cmd), e)
        out = [nan, nan] if tg is None else [float(np.linalg.norm(tg.p - Tm.translation)),
                                             float(np.linalg.norm(pin.log3(tg.R @ Tm.rotation.T)))]
        out += [float(np.linalg.norm(Tc.translation - Tm.translation)),
                float(np.linalg.norm(pin.log3(Tc.rotation @ Tm.rotation.T)))]
        kp = self.ctl.kp.get(e) if self.ctl is not None else None
        out += list(kp * (self.q_cmd[e] - st.q)) if kp is not None else [nan] * len(self.q_cmd[e])
        return out

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
        holder = self._enable_begin(token)
        peers = None
        channels: dict = {}
        sts: dict = {}
        try:
            peers = self._open_peers(holder)
            for e, http in peers.arm_http.items():         # each arm's joint channel, a first state, its limits
                lab = self._label(e)
                channel = ArmChannel(self.arm_sockets.get(e))
                try:
                    channel.open(http.token)
                except ChannelError as ex:
                    raise RequestError(503, f"{lab} joint channel: {ex}")
                channels[e] = channel
                st = None
                t_end = time.monotonic() + 1.0
                while time.monotonic() < t_end:
                    st = channel.poll(self.clock())
                    if st is not None and np.all(np.isfinite(st.q)):
                        break
                    time.sleep(0.005)
                if st is None or not np.all(np.isfinite(st.q)):
                    raise RequestError(503, f"no state from the {lab} joint channel within 1 s")
                sts[e] = st
                self._apply_arm_limits(channel.limits, e)
        except BaseException:
            for channel in channels.values():
                channel.close()
            if peers is not None:
                release_peers(peers.arm_http, peers.base, background=False)
            with self.lock:
                self.state = "disabled"
            raise
        with self.lock:
            self._enable_finish_locked(peers.arm_http, peers.arm_lease, channels, peers.base, sts, holder)
        return self.status()

    def _enable_begin(self, token: Optional[str]) -> str:
        """The enable's checks; the state becomes ``enabling``. Returns the holder."""
        self.lease.check(token)
        if self.estop.active:
            raise RequestError(409, "e-stop is latched; clear it first (POST /estop/clear)")
        if self.model is None:
            raise RequestError(503, f"no model ({self.model_error or 'not loaded'})")
        with self.lock:
            if self.enabled or self.state == "enabling":
                raise RequestError(409, f"already {self.state}; disable first")
            self.state = "enabling"
            return self.lease.info.client if self.lease.info else "anonymous"

    def _open_peers(self, holder: str) -> "Peers":
        """The arm's and the swerve's leases, limits, enables, the arm's TCP and gains (applied here)."""
        peers = open_peers(self.arm_targets, self.swerve_target, self.cfg, f"wbc ({holder})",
                           {e: len(self.model.arm_q[e]) for e in self.arm_targets}, self.events, self.ctl.kp_source)
        for e, tcp in peers.tcp.items():
            self.model.set_tcp(e, tcp)
        for e, kp in peers.kp.items():
            self.ctl.kp[e], self.ctl.kp_source = kp.copy(), peers.kp_source[e]
        return peers

    def _enable_finish_locked(self, arm_https: dict, arm_leases: dict, channels: dict, base, states: dict,
                              holder: str) -> None:
        """Under ``self.lock``: the peers are up (per EE: the arm's HTTP, lease id, joint channel, first state); seed
        the commands (each arm at its measured joints, an EE without an arm at its posture), disarmed, state idle."""
        self.arm_https, self.channels, self.base = dict(arm_https or {}), dict(channels or {}), base
        self.arm_lease_ids = {e: l for e, l in (arm_leases or {}).items() if e in self.channels}
        self.arm_states = {e: s for e, s in (states or {}).items() if e in self.channels}
        self.q_cmd = {e: (self.arm_states[e].q.copy() if self.arm_states.get(e) is not None
                          else self._posture(self.hints, e)) for e in self.ees}
        self.v_cmd = {e: np.zeros(len(self.model.arm_q[e])) for e in self.ees}
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
        leases = (f"arm lease {self.arm_lease_id}" if len(self.arm_targets) <= 1 else
                  "arm leases " + ", ".join(f"{e} {self.arm_lease_ids.get(e)}" for e in self.arm_targets))
        self.events.add(f"enabled by {holder} ({leases}, swerve lease {base.lease_id if base else None})")

    def _apply_arm_limits(self, lim: dict, e: Optional[str] = None) -> None:
        """An arm's own limits from its welcome (``e``: its EE; default the primary): joint range, joint speed and
        acceleration, lead bound, gap."""
        e = e or self.primary
        n = len(self.model.arm_q[e])
        try:
            if lim.get("lo") is not None and lim.get("hi") is not None:
                self.model.narrow_arm_limits(e, lim["lo"][:n], lim["hi"][:n])
            from ..arm.config import DEFAULT_SOFT
            self.ctl.caps[e] = self.ctl.arm_caps(n, lim.get("joint_vel") or DEFAULT_SOFT.joint_vel,
                                                 lim.get("joint_acc") or DEFAULT_SOFT.joint_acc)
            if lim.get("max_lead"):
                self.resync = min(self.resync, self.cfg.resync, 0.7 * float(lim["max_lead"]))
                self.ctl.max_lead[e] = float(lim["max_lead"])
            if lim.get("gap_max"):
                self.gap_maxes[e] = float(lim["gap_max"])
        except (TypeError, ValueError, IndexError) as e:
            self.events.add(f"arm limits in the welcome not usable ({e}); keeping the WBC's", "warn")

    def disable(self, reason: str = "requested", token: Optional[str] = None, check: bool = False) -> dict:
        if check:
            self.lease.check(token)
        with self.lock:
            r = self._disable_locked(reason)
        if r is None:
            return {"enabled": False, "was_enabled": False}
        was, channels, base, arm_https = r
        if base is not None:
            base.close(disable=True)
        for channel in channels.values():
            channel.close()
        release_peers(arm_https, None, background=True)
        return {"enabled": False, "was_enabled": was}

    def _disable_locked(self, reason: str):
        """Under ``self.lock``: the base muted, a last hold to the arm, the state disabled. Returns (was enabled,
        the channels, the base, the arms' HTTP; per EE) for the caller to close outside the lock, or None (nothing to
        do)."""
        was = self.enabled
        if not was and self.state != "fault":
            return None
        channels, base, arm_https = dict(self.channels), self.base, dict(self.arm_https)
        if base is not None:
            base.mute()               # disable coasts: no further twist (no zero twist), then the swerve disables
        # a last hold command, then close: each arm holds its command (stale = hold) until its lease ends
        for e, channel in channels.items():
            if e in self.q_cmd:
                channel.send(self.q_cmd[e], np.zeros(len(self.q_cmd[e])))
        self.channels, self.base, self.arm_https = {}, None, {}
        self._disarm_locked(f"disabled: {reason}")
        self.enabled, self.fault, self.state = False, None, "disabled"
        self.disabled_reason = reason
        self.base_twist = np.zeros(3)
        # the arm floats while disabled: no commanded / measured EE pose from the old command (the rows carry NaN)
        # that a client could capture as a riding pose; enable seeds them again from the measured joints
        self.q_cmd, self.v_cmd, self.cmd_twist = {}, {}, {}
        self.arm_states = {}
        self.events.add(f"disabled: {reason}")
        return was, channels, base, arm_https

    ESTOP_JOIN_S = 2.5          # the fan-out waits this long for the arm's and the swerve's answers

    def trigger_estop(self, reason: str, source: str) -> dict:
        """Anyone. First, under the lock, stop tracking: disarm, zero base twist, the arm held at its commanded
        joints (the ticks hold too until the disable). Then the e-stop goes to the arm and the swerve IN PARALLEL
        (both latch and coast: no braking by the robot), then disable."""
        info = self.estop.trigger(reason, source)
        with self.lock:
            self._estop_stop_locked(reason)
        fw = estop_fanout(self.arm_https, self.arm_targets, self.swerve_target, reason, source, self.events,
                          self.ESTOP_JOIN_S)
        out = {"latched": True, "reason": info.reason, "source": info.source, "forwarded": fw}
        self.disable(f"e-stop ({reason or 'no reason'})")
        return out

    def _estop_stop_locked(self, reason: str) -> None:
        """Under ``self.lock``: stop tracking at once (the e-stop's first step)."""
        self._disarm_locked(f"e-stop ({reason or 'no reason'})")
        if self.base is not None:
            self.base.mute()                         # the base gets NOTHING more (no zero twist: it coasts)
        if self.enabled:
            self._send_hold(arm=True, base=False)    # the arm at its commanded joints until its e-stop lands

    def clear_estop(self, token: Optional[str]) -> None:
        self.lease.check(token)
        self.estop.clear()
        self.events.add("e-stop cleared (the arm's and the swerve's are cleared there)")

    def set_hints(self, token: Optional[str], doc: dict) -> dict:
        self.lease.check(token)
        h = parse_hints(doc, self.cfg, self.ees, POSTURES, self.dofs())
        with self.lock:
            self._set_hints_locked(h)
        return self.get_hints()

    def _set_hints_locked(self, h: Hints) -> None:
        self.hints, self.override = h, {}
        self.hint_t = self.clock()
        if h.velocity is not None:
            self.velocity_t = self.clock()
        self.events.add(f"hints: base {h.mode}")

    def get_hints(self) -> dict:
        with self.lock:
            return hints_doc(self.hints, self.effective_hints())

    # -- state ---------------------------------------------------------------------------------------------

    def status(self) -> dict:
        with self.lock:
            return {"state": self.state, "enabled": self.enabled, "fault": self.fault, "armed": self.armed}

    def state_dict(self) -> dict:
        """The /state document. Only a snapshot of references and scalars is taken under ``self.lock`` (the lock the
        control tick takes); the document is built outside it, so a /state request or an SSE update never stalls
        the tick."""
        now = self.clock()
        with self.lock:
            sn = self._snapshot_locked(now)
        base = self.base
        return state_document(sn, now, self._server_parts(base))

    def _server_parts(self, base) -> dict:
        """What the /state document takes from the HTTP side (the server process in the process mode)."""
        last = self.hub.recent(1, "tick") if self.hub else []
        return {"estop": self.estop.info, "lease": self.lease.info, "arm_target": self.arm_target,
                "arm_targets": dict(self.arm_targets),
                "swerve_target": self.swerve_target, "model": model_doc(self.model, self.model_info, self.model_error),
                "base": None if base is None else {"lease_id": base.lease_id, "sent": base.sent, "errors": base.errors},
                "inputs": {k: u.status() for k, u in self.inputs.items()},
                "latest": ({k: (v if math.isfinite(v) else None) for k, v in zip(self.fields, last[0].tolist())}
                           if len(last) else None),
                "events": self.events.list()[-30:]}

    def _snapshot_locked(self, now: float) -> dict:
        """Under ``self.lock``: the control state as plain data (scalars, lists, dicts, the Config and Hints
        dataclasses): /state's input, and what the control process publishes to the server process."""
        out = self._last.get("out")
        rx = self.receiver
        ts = self._tick_times
        ctl = self.ctl
        arms = {e: self._arm_snapshot(e, out) for e in (self.ees or [self.primary])}
        a = arms.get(self.primary) or self._arm_snapshot(self.primary, out)
        return dict(
            t=now, state=self.state, enabled=self.enabled, armed=self.armed, disarm_reason=self.disarm_reason,
            latch={"arm_seq": self.arm_seq, "arm_seq_seen": self.arm_seq_seen, "arm_floor": self.arm_floor,
                   "disarms": self.disarms, **self.latch_counts},
            tick=dict(self.tick), holder=self.holder, fault=self.fault,
            hold_reason=getattr(self, "_hold_reason", None), disabled_reason=self.disabled_reason,
            limited_by=out.limited_by if out is not None else "none",
            lead_dev=a["lead_dev"], arms=arms if len(arms) > 1 else None,
            lead_binding=bool(out is not None and out.diag.get("lead_binding")),
            limited_counts=dict(self.limited_counts),
            arm=a["arm"], channel=a["channel"], limits=a["limits"], arm_lease_id=a["arm_lease_id"], sw=None if self.sw is None else dict(self.sw),
            hints=self.hints, in_force=self.effective_hints(), cfg=self.cfg, resyncs=self.resyncs,
            solve_fail=self.solve_fail,
            loop_hz=(len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 10 and ts[-1] > ts[0] else None,
            lead_binds=self.lead_binds, kp=a["kp"], max_lead=a["max_lead"],
            kp_source=None if ctl is None else ctl.kp_source, lead=a["lead"],
            reset_id=self.est.reset_id, jumps=self.est.jumps,
            est_age=None if self.est.row is None else round(now - self.est.row["t"], 4),
            est_health=self.estimator_health(now) or "ok",
            session={"session": rx.session, "udp_port": rx.udp_port, "unix_socket": rx.unix_path, "seq": rx.seq,
                     "accepted": rx.counts["accepted"], "dropped": dict(rx.counts["dropped"]),
                     "last_source": rx.last_source,
                     "input_age": None if self.last_rx == -math.inf else round(now - self.last_rx, 4)},
            realtime=dict(self.realtime), base_twist=[float(v) for v in self.base_twist],
        )

    def _arm_snapshot(self, e: str, out) -> dict:
        """One EE's arm as plain data (the /state ``arm`` block's input)."""
        ld = (out.diag.get("lead") or {}).get(e) if out is not None else None
        st, ch, ctl = self.arm_states.get(e), self.channels.get(e), self.ctl
        return dict(
            target=self.arm_targets.get(e),
            arm=None if st is None else {"state": st.state_name, "paused": bool(st.paused), "t_rx": float(st.t_rx)},
            channel=None if ch is None else {"session": ch.session, "stale_ms": ch.stale_ms, **ch.counts,
                                             "last_error": ch.last_error},
            limits=None if ch is None else dict(ch.limits), arm_lease_id=self.arm_lease_ids.get(e),
            lead_dev=None if ld is None else [float(x) for x in ld["dev"]],
            kp=None if ctl is None or ctl.kp.get(e) is None else [float(x) for x in ctl.kp[e]],
            max_lead=None if ctl is None else ctl.max_lead.get(e),
            lead=None if ctl is None or e not in ctl.kp else [float(x) for x in ctl.lead(e)],
            posture=self._posture_doc(e, out))

    @staticmethod
    def _posture_doc(e: str, out) -> Optional[dict]:
        """A redundant arm's elbow (docs/wbc.md "Null-space posture"): its swivel, the target, the limit-aware status
        (ok | limited: the human-like angle is outside the elbow's room within the joint limits, re-posing: the elbow
        is moving back into it, unreachable: no arm angle of this branch fits the target within the limits) and the
        room (rad); None for a 6-joint arm."""
        sw = ((out.diag.get("swivel") or {}).get(e) if out is not None else None)
        if not sw:
            return None
        r = lambda v: None if v is None else round(float(v), 4)    # noqa: E731
        h = sw.get("hint")
        return {"psi": r(sw.get("psi")), "psi_target": r(sw.get("psi_star")), "status": sw.get("status"),
                "room": None if sw.get("room") is None else [r(v) for v in sw["room"]],
                "swivel": r(sw.get("psi_cont")),
                "hint": None if not h else {"seq": h["seq"], "live": h["live"], "anchors": h["anchors"],
                                            **{k: r(h.get(k)) for k in ("weight", "anchor", "anchor_dpsi", "dpsi",
                                                                        "raw", "target", "age", "f_elbow", "f_conf",
                                                                        "blend", "f_robot")}}}

    # -- runtime config ------------------------------------------------------------------------------

    STRUCTURAL = STRUCTURAL

    def get_config(self) -> dict:
        return config_doc(self.cfg, self.base)

    BASE_CAPS = BASE_CAPS

    def _caps_check_locked(self, changes: dict) -> None:
        """The base caps change only while the base is still (disabled, or a zero commanded twist)."""
        caps_check(self.cfg, changes, self.enabled, self.base_twist)

    def set_config(self, changes: dict) -> dict:
        """Change config keys at runtime (any time; no lease: the operator tunes while a client drives). Validated
        as at start, the base caps against the swerve's hard limits and only while the base is still; structural
        keys only while disabled."""
        changes = dict(changes or {})
        with self.lock:
            hard = getattr(self.base, "hard", None) or {} if self.base is not None else {}
            new = validate_config(self.cfg, changes, self.enabled, self.base_twist,
                                  None if self.ctl is None else self.ctl.kp.get(self.primary),
                                  None if self.ctl is None else self.ctl.max_lead.get(self.primary), hard)
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
                self._apply_config_locked(changes, new)
        if moved is not None:
            if push and base is not None and base.client is not None:
                try:
                    base.client.set_limits(old.max_speed, old.max_omega)
                except Exception:  # noqa: BLE001
                    pass
            raise moved
        return self.get_config()

    def _apply_config_locked(self, changes: dict, new: Config) -> None:
        self.cfg = new
        self.ctl.cfg = new
        for b in self.buffers.values():
            b.ff_timeout, b.blend_tau = new.ff_timeout, new.blend_tau
        self.events.add("config: " + ", ".join(f"{k}={v}" for k, v in changes.items()))



def caps_check(cfg: Config, changes: dict, enabled: bool, base_twist) -> None:
    """The base caps change only while the base is still (disabled, or a zero commanded twist): ValueError."""
    changed = [k for k in BASE_CAPS if k in changes and changes[k] != getattr(cfg, k)]
    if changed and enabled and bool(np.any(np.asarray(base_twist) != 0.0)):
        tw = ", ".join(f"{v:.3f}" for v in base_twist)
        raise ValueError(f"{changed}: the base caps cannot change while the base moves (commanded twist {tw}); "
                         "stop it first (disarm, or base mode locked)")


def validate_config(cfg: Config, changes: dict, enabled: bool, base_twist, kp, max_lead, hard: dict) -> Config:
    """``POST /config``'s checks; the new Config (ValueError with the reason otherwise)."""
    if enabled and any(k in changes for k in STRUCTURAL):
        raise ValueError(f"{sorted(set(changes) & set(STRUCTURAL))}: only while disabled")
    caps_check(cfg, changes, enabled, base_twist)
    new = make_config(changes, base=cfg)
    if "lead_torque" in changes and kp is not None:
        check_lead_torque(new.lead_torque, np.asarray(kp, float), max_lead)
    hs, hw = (hard or {}).get("max_speed"), (hard or {}).get("max_omega")
    if (hs is not None and new.max_speed > hs) or (hw is not None and new.max_omega > hw):
        raise ValueError(f"max_speed / max_omega beyond the swerve's hard limits ({hs} m/s, {hw} rad/s)")
    return new


def config_doc(cfg: Config, base) -> dict:
    """``GET /config``."""
    from .spec import Config as _C
    js = lambda d: {k: (list(v) if isinstance(v, tuple) else v) for k, v in d.items()}   # noqa: E731
    hard = dict(base.hard) if base is not None and getattr(base, "hard", None) else {}
    return {"config": js(asdict(cfg)), "defaults": js(asdict(_C())), "hard": hard, "structural": list(STRUCTURAL)}


def hints_doc(hints: Hints, in_force: Hints) -> dict:
    """``GET /hints``."""
    return {"hints": hints_public(hints), "in_force": hints_public(in_force),
            "postures": {k: [round(float(x), 6) for x in v] for k, v in POSTURES.items()}}


def model_doc(model, info, error) -> dict:
    return {"source": info, "error": error,
            "ees": {k: {"link": v.link, "tcp": list(v.tcp), "reach": round(model.reach[k], 4)}
                    for k, v in model.ees.items()} if model else {}}


def state_document(sn: dict, now: float, srv: dict) -> dict:
    """The /state document from a control snapshot (``WbcNode._snapshot_locked``) and the HTTP side's parts
    (``WbcNode._server_parts``)."""
    info, e = srv["lease"], srv["estop"]
    base, sw = srv["base"], sn["sw"]
    doc = {
        "state": sn["state"],
        "enabled": sn["enabled"],
        "armed": sn["armed"],
        "disarm_reason": None if sn["armed"] else sn["disarm_reason"],
        "latch": sn["latch"],
        "tick": {k: (round(v, 3) if isinstance(v, float) and math.isfinite(v) else
                     None if isinstance(v, float) else v) for k, v in sn["tick"].items()},
        "holder": sn["holder"] if sn["enabled"] else None,
        "fault": sn["fault"],
        "hold_reason": sn["hold_reason"] if sn["enabled"] else None,
        "disabled_reason": sn["disabled_reason"],
        "limited_by": sn["limited_by"],
        "limited_counts": sn["limited_counts"],
        "estop": {"latched": e is not None, "reason": e and e.reason, "source": e and e.source,
                  "t": e and e.t},
        "lease": {"held": info is not None, "holder": info and info.client,
                  "lease_id": info and info.lease_id,
                  "expires_in": None if info is None else round(info.expires - now, 3)},
        "session": sn["session"],
        "model": srv["model"],
        "arm": arm_block(sn, now, srv["arm_target"]),
        "swerve": {"target": srv["swerve_target"],
                   "lease_id": base["lease_id"] if base else None,
                   "sent": base["sent"] if base else None,
                   "errors": base["errors"] if base else None,
                   "telemetry": sw and {k: v for k, v in sw.items() if k != "rx"}},
        "estimator": {"health": sn["est_health"], "reset_id": sn["reset_id"],
                      "pose_jumps": sn["jumps"], "age": sn["est_age"]},
        "inputs": srv["inputs"],
        "hints": hints_public(sn["in_force"]),
        "config": {k: (list(v) if isinstance(v, tuple) else v) for k, v in asdict(sn["cfg"]).items()},
        "solve_failures": sn["solve_fail"],
        "loop_hz": None if sn["loop_hz"] is None else round(sn["loop_hz"], 1),
        "realtime": sn["realtime"],
        "latest": srv["latest"],
        "events": srv["events"],
    }
    if sn.get("arms"):                     # several arms: the same block per EE (the primary's is "arm" above)
        targets = srv.get("arm_targets") or {}
        doc["arms"] = {ee: arm_block({**sn, **a}, now, targets.get(ee, a.get("target"))) for ee, a in sn["arms"].items()}
    return doc


def arm_block(sn: dict, now: float, target) -> dict:
    """The /state ``arm`` block of one arm (its snapshot keys merged over the control snapshot)."""
    r6 = lambda v: None if v is None else [round(float(x), 6) for x in v]   # noqa: E731
    st, dev = sn["arm"], sn["lead_dev"]
    return {"target": target, "lease_id": sn["arm_lease_id"] if sn["enabled"] else None,
            "channel": sn["channel"],
            "state": None if st is None else st["state"],
            "paused": bool(st is not None and st["paused"]),
            "limits": sn["limits"],
            "age": None if st is None else round(now - st["t_rx"], 4),
            "resyncs": sn["resyncs"],
            # the lead bound: per joint the bound (rad) = lead_torque / kp (at most max_lead), the command's
            # deviation beyond the measured window this tick (rad) and its torque, how often it bound
            "lead": {"lead_torque": list(sn["cfg"].lead_torque), "kp": r6(sn["kp"]),
                     "kp_source": sn["kp_source"], "max_lead": sn["max_lead"], "lead_delay": sn["cfg"].lead_delay,
                     "bound": r6(sn["lead"]), "dev": r6(dev),
                     "torque": (r6(np.asarray(sn["kp"]) * np.asarray(dev))
                                if dev is not None and sn["kp"] is not None else None),
                     "binding": sn["lead_binding"],
                     "binds": sn["lead_binds"]},
            "posture": sn.get("posture")}


@dataclass
class Peers:
    """The enable's HTTP side, per EE: each arm's control plane (its lease), its TCP and gains; the swerve base."""
    arm_http: dict = field(default_factory=dict)
    arm_lease: dict = field(default_factory=dict)
    base: object = None
    tcp: dict = field(default_factory=dict)
    kp: dict = field(default_factory=dict)
    kp_source: dict = field(default_factory=dict)


def _arm_https(x) -> list:
    """The arms' HTTP clients from a dict, a list, one client or None."""
    if x is None:
        return []
    if isinstance(x, dict):
        return [v for v in x.values() if v is not None]
    if isinstance(x, (list, tuple)):
        return [v for v in x if v is not None]
    return [x]


def open_peers(arm_targets, swerve_target, cfg: Config, name: str, n_arm, events, kp_source: str) -> Peers:
    """Take every arm's lease and the swerve's together, set the swerve's soft limits, enable all, read each arm's
    TCP and gains (the lead bound's kp). ``arm_targets``: EE -> arm node target (a string: the one arm ``arm``);
    ``n_arm``: EE -> its joint count (or one count). Rolls everything back on a failure (RequestError)."""
    if isinstance(arm_targets, str) or arm_targets is None:
        arm_targets = {"arm": arm_targets} if arm_targets else {}
    n_of = (lambda e: n_arm[e]) if isinstance(n_arm, dict) else (lambda e: n_arm)   # noqa: E731
    p = Peers()
    try:
        for e, target in arm_targets.items():
            lab = arm_label(e, arm_targets)
            http = ArmHttpFactory(target)
            try:
                out = http.acquire(name)
            except Exception as ex:
                raise RequestError(getattr(ex, "status", 503), f"{lab} lease: {getattr(ex, 'detail', ex)}")
            p.arm_http[e] = http
            p.arm_lease[e] = out.get("lease_id")
        if swerve_target:
            from .links import SwerveBase
            base = SwerveBase(swerve_target)
            try:
                base.open(name, cfg.max_speed, cfg.max_omega)
            except Exception as e:
                from .links import LimitsError
                status = 422 if isinstance(e, LimitsError) else getattr(e, "status", 503)
                raise RequestError(status, f"swerve: {getattr(e, 'detail', e)}")
            p.base = base
        for e, http in p.arm_http.items():
            lab, n = arm_label(e, arm_targets), n_of(e)
            try:
                http.enable()
            except Exception as ex:
                raise RequestError(getattr(ex, "status", 503), f"{lab} enable: {getattr(ex, 'detail', ex)}")
            tcp = http.tcp()
            if tcp is not None:
                p.tcp[e] = tcp
            # the servo's kp for the lead bound: the arm's gain profile in force (its GET /gains); without it the
            # arm's default profile file stays (an event)
            try:
                g = http.get("/gains")
                kp = np.asarray(g["kp"], float)
                if kp.shape[0] < n or not np.all(np.isfinite(kp[:n])) or np.any(kp[:n] <= 0):
                    raise ValueError(f"kp {g['kp']}")
                p.kp[e], p.kp_source[e] = kp[:n].copy(), f"{lab} GET /gains (profile {g.get('name')})"
            except Exception as ex:  # noqa: BLE001 - optional: keep the default profile's
                events.add(f"{lab} gains not readable ({type(ex).__name__}: {ex}); the lead bound uses {kp_source}",
                           "warn")
    except BaseException:
        release_peers(p.arm_http, p.base, background=False)
        raise
    return p


def release_peers(arm_http, base, background: bool = True) -> None:
    """Disable and release the swerve (its own background thread) and the arms (``arm_http``: per EE, a list or
    one; ``background``: in a thread)."""
    if base is not None:
        base.close(disable=True)
    https = _arm_https(arm_http)
    if not https:
        return

    def release():
        for h in https:
            for call in (h.disable, h.release):
                try:
                    call()
                except Exception:  # noqa: BLE001
                    pass
            h.close()
    if background:
        threading.Thread(target=release, daemon=True).start()
    else:
        release()


def estop_fanout(arm_http, arm_target, swerve_target, reason: str, source: str, events, join_s: float) -> dict:
    """The e-stop to every arm and the swerve IN PARALLEL (a slow one never delays the others); waits at most
    ``join_s``. Returns ``forwarded``: per peer whether it answered (``arm`` with one arm, else each arm's EE).
    ``arm_http`` / ``arm_target``: per EE (or the one arm's)."""
    fw: dict = {}
    msg = f"wbc: {reason}" if reason else "wbc"
    targets = arm_target if isinstance(arm_target, dict) else ({"arm": arm_target} if arm_target else {})
    https = arm_http if isinstance(arm_http, dict) else ({next(iter(targets), "arm"): arm_http} if arm_http else {})

    def to_arm(e, target):
        h = https.get(e) or ArmHttpFactory(target)
        h.estop(msg, source or "wbc")

    def to_swerve():
        from ..swerve.client import SwerveClient
        with SwerveClient(swerve_target, timeout=0.5) as c:
            c.estop(msg, source or "wbc")

    def run(name, fn):
        try:
            fn()
            fw[name] = True
        except Exception as e:  # noqa: BLE001
            fw[name] = False
            events.add(f"e-stop forward to the {name} failed: {e}", "error")

    peers = [(arm_label(e, targets), (lambda e=e, t=t: to_arm(e, t)), t) for e, t in targets.items()]
    peers.append(("swerve", to_swerve, swerve_target))
    threads = [threading.Thread(target=run, args=(n, f), name=f"wbc-estop-{n}", daemon=True) for n, f, t in peers if t]
    for t in threads:
        t.start()
    deadline = time.monotonic() + join_s
    for t in threads:
        t.join(max(0.0, deadline - time.monotonic()))
    out = {}
    for n, _, t in peers:
        if t:
            out[n] = bool(fw.get(n, False))
            if n not in fw:
                events.add(f"e-stop forward to the {n}: no answer within {join_s} s", "error")
    return out


def forward_estop(arm_http, swerve_target, reason: str, events, *, base: bool, arm: bool,
                  skip: Optional[str] = None) -> None:
    """A cross-bus e-stop (one bus's e-stop e-stops the others), in the background. ``arm_http``: per EE (or one);
    ``skip``: the EE whose own e-stop this is."""
    https = ({e: h for e, h in arm_http.items() if e != skip and h is not None} if isinstance(arm_http, dict) else
             ({"arm": arm_http} if arm_http is not None else {}))

    def go():
        for e, h in (https.items() if arm else ()):
            try:
                h.estop(f"wbc: {reason}")
            except Exception as ex:  # noqa: BLE001
                events.add(f"e-stop forward to the {arm_label(e, https)} failed: {ex}", "error")
        if base and swerve_target:
            try:
                from ..swerve.client import SwerveClient
                with SwerveClient(swerve_target, timeout=0.5) as c:
                    c.estop(f"wbc: {reason}", "wbc")
            except Exception as e:  # noqa: BLE001
                events.add(f"e-stop forward to the swerve failed: {e}", "error")
    threading.Thread(target=go, daemon=True).start()


def ArmHttpFactory(target: str):
    """The arm's HTTP client (a hook for tests)."""
    from .links import ArmHttp
    return ArmHttp(target)
