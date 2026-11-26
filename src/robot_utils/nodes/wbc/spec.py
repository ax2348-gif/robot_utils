"""The WBC's cost and constraint spec: the node's caps and gains (``Config``) and the hint structure
(docs/plans/wbc_v1.md "Hints"), shared by every controller (the QP now, an MPC later).

Hints are session defaults (``POST /hints``), overridable per command (a ``base`` field on a target or chunk
datagram); every numeric hint is bounded by the caps. ``parse_hints`` validates a JSON hint document into
``Hints``; ``merge`` applies an override (the same keys, any subset) over the hints in force.
"""
from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass, field, fields, replace
from typing import Optional

import numpy as np

MODES = ("free", "locked", "hint", "commanded")

# The base speed and yaw-rate ceilings are the swerve's (its GET /limits "hard", checked at enable). The wheel speed has
# no swerve endpoint: pose_servo's planner keeps it (the steering model's bound), so it is read from there.
from ..pose_servo.planner import WHEEL_SPEED_MAX as WHEEL_SPEED_HARD   # noqa: E402  m/s, each wheel


@dataclass(frozen=True)
class Config:
    """Caps, gains and timeouts (``--config`` YAML keys; the defaults below)."""
    rate_hz: float = 100.0
    # base caps (the swerve's soft limits are set to max_speed / max_omega at enable)
    max_speed: float = 0.8          # m/s, |(vx, vy)| body (inscribed octagon); <= the swerve's hard limit (at enable)
    max_omega: float = 0.9          # rad/s; <= the swerve's hard limit (at enable)
    max_accel: float = 2.0          # m/s², |twist rate| linear (octagon)
    max_alpha: float = 1.5          # rad/s²
    wheel_speed: float = 2.0        # m/s, every wheel (inscribed octagon, 8 rows per wheel)
    # arm caps (within the arm node's soft limits)
    # None = the arm's own (its joint-channel welcome `limits`; without one, the arm config's soft defaults
    # 2,2,2,6,6,6 rad/s and 10,10,10,30,30,30 rad/s²); a tuple narrows them further (never widens)
    joint_vel: Optional[tuple] = None   # rad/s
    joint_acc: Optional[tuple] = None   # rad/s²
    limit_gain: float = 0.5         # joint position velocity damper: a step covers at most this share of the
                                    # distance to a limit (pink ConfigurationLimit)
    # yielding at the arm's limits (constraints, not costs: the base hint stops at the boundary)
    reach_fraction: float = 0.88    # shoulder (joint 1 axis point) to EE target at most this share of the nominal
    reach: float = 0.0              # m, the nominal shoulder-to-TCP reach (0 = from the model, ~0.68 m)
    reach_influence: float = 0.80   # the reach damper acts beyond this share of the nominal reach
    xi_reach: float = 0.3           # m/s, the reach damper's gain (approach speed at the influence distance)
    # arm-only translational manipulability w = sqrt(det(Jp Jp^T)) (m^3/rad^3; start pose ~0.011; 10 % of the joint
    # range below ~0.003): w_min > 0 turns on a floor constraint (0 = off, the default: reach and joint limits bound
    # the workspace); below w_influence the arm gets the adaptive damping (lm_singular)
    w_min: float = 0.0
    w_influence: float = 0.003
    xi_manip: float = 2.0           # 1/s: dw/dt >= -xi_manip w_influence (w - w_min) / (w_influence - w_min)
    lm_singular: float = 0.02       # adaptive damping on the arm, x (1 - w / w_influence)^2 below w_influence
    resync: float = 0.25            # rad: |q_meas - q_cmd| beyond this (any joint) resyncs the integration (at most
                                    # 0.7 of the arm's lead bound; the world arm trails ~5 ticks: 0.1 rad at 2 rad/s);
                                    # with the lead bound below, a last-resort safety net only
    # the lead bound (docs/wbc.md "Lead bound"): each arm joint's next command stays within lead_i = lead_torque_i /
    # kp_i of the measured joints extrapolated over lead_delay (+ the arm state's age), so the servo's push
    # kp (q_cmd - q) in contact stays within lead_torque. kp: the arm's gain profile (its GET /gains at enable);
    # each value at most kp x the arm's max_lead (the arm drops goals beyond that)
    lead_torque: tuple = (6.0, 6.0, 6.0, 1.5, 1.5, 1.5)   # Nm per joint
    lead_delay: float = 0.08        # s: the measured arm's lag behind the command (read pipeline + servo; the world
                                    # sim trails ~5 ticks) + the one tick the new command leads by + 2 ticks of margin
                                    # (too long only widens the window on the moving side: safe); 0 = no extrapolation
    # tracking
    k_pos: float = 4.0              # 1/s, P on the EE position error
    k_rot: float = 4.0              # 1/s, P on the EE orientation error
    max_err_pos: float = 0.25       # m: larger position errors are clipped (velocity limits do the rest)
    max_err_rot: float = 1.0        # rad
    w_position: float = 1.0         # default EE task weights (hints override)
    w_orientation: float = 0.3
    lm_damping: float = 1e-3        # Levenberg-Marquardt damping on the EE task
    damping: float = 1e-6           # Tikhonov regularisation on everything
    w_arm: float = 2e-3             # arm joint regularisation (per rad²)
    # base references (one QP; docs/plans/wbc_v1.md "Formulation"): the base twist is pulled toward
    # v_ref + k (pose_ref - pose) with weight w_base_ref (x hints.arm_vs_base), toward zero with w_base_free when no
    # reference is given; the arm/base split of EE motion follows from w_base_* against the EE task weights
    w_base_ref: float = 1.0
    w_base_free: float = 1e-3
    # allowed deviation from the references (hard bands with an L1 slack penalised by w_slack, so always feasible):
    # pose band: |pose - pose_ref| <= band_pos / band_yaw, approached with gain k_band, never forced back faster;
    # twist band: the base twist stays in conv{0, ball(v_ref, band_twist)} (may lag the reference down to zero, may
    # not exceed it by more than band_twist): fast EE motion goes to the arm
    band_pos: float = 0.15          # m
    band_yaw: float = 0.349         # rad (20 deg)
    band_twist: float = 0.1         # m/s
    band_omega: float = 0.3         # rad/s
    k_band: float = 2.0             # 1/s
    w_slack: float = 1e3            # per unit of band violation (velocity units)
    # flat (L1) cost of moving the base at all, per m/s (octagon norm) and per rad/s: the principled deadband. The
    # base stays exactly still unless the pull to move (2 w_base_ref |v_des|, or the EE task) exceeds it, so tiny,
    # sign-flipping twists (swerve modules re-steering: chatter) are not worth commanding; while moving it biases the
    # speed by w_base_flat / (2 w_base_ref), which the pose correction absorbs. 0 = off
    w_base_flat: float = 0.0        # (L1 per m/s; at 0.10 a constant +5 cm walking lag: off, superseded by the charge)
    w_base_flat_yaw: float = 0.0
    # the fixed charge of starting the base: at rest (below rest_lin / rest_ang) two QPs are solved, the base held
    # (braking to zero) and free; free wins only if its cost plus base_start_charge is lower. Exact (two discrete
    # outcomes), no bias while moving; tiny motions (sway, small EE pushes: swerve chatter) are never worth it.
    # Objective units (1/2 |residual|^2, residuals in weighted m/s). 0 = off
    base_start_charge: float = 0.01
    rest_lin: float = 0.02          # m/s
    rest_ang: float = 0.05          # rad/s
    base_yaw_radius: float = 0.3    # m: yaw is weighted like a point this far from the centre
    posture_weight: float = 1e-4    # default (hints override); 2e-3 biased the EE by ~3 mm
    # a redundant arm (more joints than the EE task's 6 rows, e.g. a 7-DOF OpenArm): the posture is a cost on the
    # task's NULL SPACE only (controller.py "Null-space posture"): it cannot move the EE, so it may be strong; a hint's
    # posture_weight scales it relative to posture_weight (the default hint = this weight)
    w_null_posture: float = 0.5
    null_posture_speed: float = 0.3   # rad/s per joint: the null-space posture's velocity cap (a slow preference)
    null_limit_margin: float = 0.1    # rad: a joint this close to a limit is left out of the null-space terms
    # what the null space of a redundant arm does (wbc/humanlike.py, docs/wbc.md "Null-space posture"):
    # "humanlike" (the default): the swivel model + effort + centering (+ the arm-proximity term); "nominal": attract
    # the fixed nominal posture (the hint's posture, w_null_posture), kept for comparison
    null_mode: str = "humanlike"
    w_swivel: float = 1.0           # the swivel attractor's weight (priority: proximity > swivel > effort > centering)
    k_swivel: float = 1.0           # 1/s: the swivel's convergence rate
    swivel_dir: str = "head"        # the manipulability direction: head | torso | motion; down = no direction, the
                                    # elbow hangs below the S-W line (psi* 0: the relaxed arm; humanlike.py)
    swivel_range: tuple = (-20.0, 100.0)   # deg: the swivel target's range (0 elbow down, + out)
    swivel_head: tuple = (0.12, 0.0, 1.0)  # m, base_link: the head point (swivel_dir head / motion)
    swivel_torso: tuple = (0.12, 0.0, 0.45)  # m, base_link: the torso centre (swivel_dir torso)
    swivel_beta: float = 10.0       # the soft argmax's sharpness (higher: closer to the hard argmax)
    swivel_tau: float = 0.3         # s: the swivel target's low-pass
    w_effort: float = 0.3           # gravity effort minimisation's weight
    k_effort: float = 1.0           # rad²/s: its gain on the self-motion gradient of log Σ (τ_g / τ_max)²
    w_center: float = 0.1           # joint-limit centering (shoulder, wrist)'s weight
    k_center: float = 0.1           # rad/s at a limit
    # the soft arm-proximity term (upper arm / forearm capsules only; 0 = off; off with one arm): strictly short
    # range: exactly zero beyond proximity_clearance (m of clearance between the capsules), ramping in as s²
    w_arm_proximity: float = 0.0
    proximity_clearance: float = 0.05
    proximity_speed: float = 0.2    # m/s: the wanted separation speed at contact (s = 1)
    # limit-aware swivel (wbc/swivel_feasible.py): the swivel target clipped into the elbow's room within the joint
    # limits (minus null_limit_margin) at the target pose, re-planned every swivel_plan_period s (the arms in turn)
    swivel_limit_aware: bool = False  # opt-in: measured no coverage gain, up to 18 ms p99 per tick (docs/wbc.md)
    swivel_plan_period: float = 0.1  # s
    # joint-limit repulsion in the null space: within limit_band of a limit, away at limit_speed s (rad/s), weight
    # w_limit s^2 (s = (band - distance) / band); exactly zero outside the band (0 = off)
    w_limit: float = 0.0            # opt-in: 10 measured no gain (docs/wbc.md "Near the joint limits")
    limit_band: float = 0.15        # rad
    limit_speed: float = 0.5        # rad/s
    # the operator's swivel hint (a per-EE ``swivel`` {seq, dpsi, elbow, conf} in the target datagrams: the operator's
    # swivel change since their engagement; humanlike.py, docs/wbc.md "The operator's elbow"): anchored at the robot's
    # commanded swivel on each new engagement, it replaces the model's swivel target by its weight, null space only
    swivel_hint_gain: float = 1.0   # robot swivel per operator swivel (target = anchor + gain dpsi)
    w_swivel_hint: float = 3.0      # its weight (below the proximity term's: that one still acts)
    # the speeds (2026-11-26, the owner: "barely does anything"; it followed ~half of the operator's swivel in 1-2 s
    # grips at 3 / 1.5 / 0.1 / 1.0 / 0.3): the hand stays primary (null space), so a fast elbow costs no EE error
    k_swivel_hint: float = 5.0      # 1/s: the elbow's convergence rate to the target
    swivel_hint_tau: float = 0.05   # s: the target's low-pass
    swivel_hint_rate: float = 2.0   # rad/s: the target's rate cap (and the swivel speed the term asks for at most)
    swivel_hint_speed: float = 1.2  # rad/s: the null-space velocity cap while it is in force (null_posture_speed
                                    # otherwise; blended by its weight)
    swivel_hint_blend: float = 0.15  # s: its weight ramps over this (0 -> 1): in, out, and on every fade change
    swivel_hint_timeout: float = 0.3  # s: no swivel field for this long: stale (released, or the body lost)
    swivel_hint_hold: bool = True   # stale: keep the target and the weight (the elbow stays where the operator left it,
                                    # as the hand does when released; the next engagement re-anchors there) instead of
                                    # fading back to the model's swivel, which undid every grip's swivel between clutches
                                    # (the owner, 2026-11-26). A controller reset (disable) clears it
    swivel_hint_flex: tuple = (10.0, 30.0)   # deg of elbow flexion (pi - the upper-arm-forearm angle), the
                                    # operator's and the robot's alike: the weight is 0 below the first, 1 above the
                                    # second (smoothstep): it vanishes toward either arm's full extension
    swivel_hint_conf: tuple = (0.3, 0.6)     # the operator's conf: 0 below the first, 1 above the second (smoothstep)
    k_posture: float = 1.0          # 1/s
    tie_speed: float = 0.05         # m/s: posture and manipulability act fully at this desired EE speed, scaled
                                    # down below it (0 when the task is satisfied: no self-motion)
    w_manip: float = 0.0            # pull toward higher manipulability (objective; 0 = off)
    k_heading: float = 2.0          # 1/s, base heading hint
    k_base_pose: float = 2.0        # 1/s, base pose hint / commanded pose
    # inputs
    ff_timeout: float = 0.1         # s: past this the twist feedforward fades (to 0 at 2 x) and the pose holds
    hold_timeout: float = 0.25      # s without a datagram from the client (≈ 2.5 x its period): base stops, arm holds
    target_t_window: float = 1.0    # s: a target / chunk start must lie within now ± this (Pi clock)
    chunk_max_len: int = 500
    chunk_max_horizon: float = 10.0  # s
    blend_tau: float = 0.15         # s, the chunk cross-fade's time constant
    max_ttl: float = 1.0            # s, commanded base velocity ttl cap
    # an ARMING datagram's target must lie this close to the commanded EE (else refused: the press does not arm), so a
    # client holding a stale riding pose (the arm moved meanwhile, e.g. floating while disabled) can never jump it
    arm_target_pos: float = 0.02    # m
    arm_target_rot: float = 0.0873  # rad (5 deg)
    # estimator
    require_imu: bool = False
    est_stale: float = 0.35         # s without an estimator row -> hold (0.2 tripped on WBC process stalls on the Pi, 2026-11-21)
    est_pose_stale: float = 0.3     # s: a pose older than this -> hold
    max_predict: float = 0.1        # s, pose extrapolation cap
    jump_pos: float = 0.15          # m between consecutive poses: a jump (targets aborted)
    jump_theta: float = 0.30        # rad
    lease_heartbeat: float = 1.0    # s, the WBC's own client lease
    late_tick: float = 0.1          # s: a control tick later than this disarms (stopgap: 3 dt = 30 ms tripped on Python GIL
                                    # stalls on the Pi; the arm's own 30 ms watchdog and the swerve's 0.25 s remain the motor-level stops)
    arm_stale: float = 0.2          # s without an arm joint-channel state while enabled -> fault (Pi arm-loop gaps of
                                    # 105-121 ms were measured under camera load: docs/realtime.md)
    # the control process (docs/wbc.md "Processes"): each watches the other's heartbeat in shared memory
    ctl_heartbeat: float = 0.2      # s without the control process's heartbeat -> the server faults (disable, leases
                                    # released); the operator restarts the node
    srv_heartbeat: float = 0.5      # s without the server process's heartbeat -> the control process disarms and holds

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz


KP_FALLBACK = (72.0, 72.0, 96.0, 18.0, 18.0, 18.0)   # the soft profile's (rebotarm_dm_soft.yaml) if it cannot load


def default_arm_gains() -> tuple[np.ndarray, float, str]:
    """(kp per joint, the arm's max_lead, where kp came from) before the arm's own are known: the arm's default
    gain profile file (rebot_ctl), else KP_FALLBACK."""
    from ..arm.stream import MAX_LEAD
    try:
        from ..arm.config import DEFAULT_PROFILE, load_profile
        return np.asarray(load_profile(DEFAULT_PROFILE).kp, float), MAX_LEAD, f"profile {DEFAULT_PROFILE} (file)"
    except Exception:  # noqa: BLE001 - rebot_ctl missing: the copy
        return np.asarray(KP_FALLBACK, float), MAX_LEAD, "fallback (soft profile copy)"


def fit(v, n: int) -> np.ndarray:
    """A per-joint vector for an arm of ``n`` joints: ``v``'s first n, its last value repeated past its end (a 6-joint
    default, e.g. lead_torque, on a 7-joint arm: the wrist value for the extra wrist joint)."""
    v = np.asarray(v, float).reshape(-1)
    return v[:n] if len(v) >= n else np.concatenate([v, np.full(n - len(v), v[-1])])


def check_lead_torque(lead_torque, kp, max_lead: float) -> None:
    """The hard ceiling: lead_torque_i <= kp_i x max_lead (beyond it the arm drops the goals)."""
    ceil = np.asarray(kp, float)[:6] * float(max_lead)
    lt = np.asarray(lead_torque, float)[:6]
    if np.any(lt > ceil + 1e-9):
        raise ValueError("lead_torque beyond kp x the arm's max_lead (" +
                         ", ".join(f"{c:.2f}" for c in ceil) + " Nm)")


def make_config(changes: Optional[dict] = None, base: Optional[Config] = None) -> Config:
    changes = dict(changes or {})
    names = {f.name for f in fields(Config)}
    unknown = set(changes) - names
    if unknown:
        raise ValueError(f"unknown config keys: {sorted(unknown)}")
    for k in ("joint_vel", "joint_acc"):
        if changes.get(k) is not None:
            changes[k] = tuple(float(v) for v in changes[k])
            if not all(math.isfinite(v) and v > 0 for v in changes[k]):
                raise ValueError(f"{k} must be positive finite numbers (or null: the arm's)")
    if "lead_torque" in changes:
        v = changes["lead_torque"]
        if not isinstance(v, (list, tuple)) or len(v) != 6 or any(
                isinstance(x, bool) or not isinstance(x, (int, float)) for x in v):
            raise ValueError("lead_torque must be 6 numbers (Nm per arm joint)")
        changes["lead_torque"] = tuple(float(x) for x in v)
        if not all(math.isfinite(x) and x > 0 for x in changes["lead_torque"]):
            raise ValueError("lead_torque must be positive finite numbers (Nm)")
    for k, n in (("swivel_range", 2), ("swivel_head", 3), ("swivel_torso", 3), ("swivel_hint_flex", 2),
                 ("swivel_hint_conf", 2)):
        if changes.get(k) is not None:
            v = changes[k]
            if not isinstance(v, (list, tuple)) or len(v) != n or any(
                    isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in v):
                raise ValueError(f"{k} must be {n} finite numbers")
            changes[k] = tuple(float(x) for x in v)
    c = replace(base or Config(), **changes)
    if c.null_mode not in ("humanlike", "nominal"):
        raise ValueError("null_mode must be humanlike or nominal")
    if c.swivel_dir not in ("head", "torso", "motion", "down"):
        raise ValueError("swivel_dir must be head, torso, motion or down")
    if not c.swivel_range[0] < c.swivel_range[1]:
        raise ValueError("swivel_range must be [low, high] degrees, low < high")
    if not 0 <= c.swivel_hint_flex[0] < c.swivel_hint_flex[1] <= 180:
        raise ValueError("swivel_hint_flex must be [low, high] degrees with 0 <= low < high <= 180")
    if not 0 <= c.swivel_hint_conf[0] < c.swivel_hint_conf[1] <= 1:
        raise ValueError("swivel_hint_conf must be [low, high] with 0 <= low < high <= 1")
    nonneg = ("w_min", "w_null_posture", "w_swivel", "w_effort", "w_center", "w_arm_proximity", "w_limit", "w_swivel_hint", "lead_delay", "w_manip", "w_base_free", "lm_singular", "posture_weight", "w_base_flat",
              "w_base_flat_yaw", "base_start_charge")   # 0 = off
    pos = [f.name for f in fields(Config) if f.type in ("float", float) and f.name not in ("damping", "reach")]
    for k in pos:
        v = getattr(c, k)
        ok = isinstance(v, (int, float)) and math.isfinite(v) and (v >= 0 if k in nonneg else v > 0)
        if not ok:
            raise ValueError(f"{k} must be a {'non-negative' if k in nonneg else 'positive'} finite number")
    if not (math.isfinite(c.reach) and c.reach >= 0):
        raise ValueError("reach must be >= 0 (0 = from the model)")
    if not (0 < c.reach_influence < c.reach_fraction <= 1.0) or not (0 <= c.w_min < c.w_influence):
        raise ValueError("need 0 < reach_influence < reach_fraction <= 1 and 0 <= w_min < w_influence")
    if c.wheel_speed > WHEEL_SPEED_HARD:
        raise ValueError(f"wheel_speed beyond the wheels' limit {WHEEL_SPEED_HARD} m/s")
    if c.hold_timeout <= c.ff_timeout:
        raise ValueError("hold_timeout must exceed ff_timeout")
    return c


# -- hints -------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class PoseHint:
    x: float
    y: float
    weight: float = 1.0
    tol: float = 0.05
    vx: float = 0.0                   # m/s world: the target's velocity (feedforward; fades like the EE twist)
    vy: float = 0.0


@dataclass(frozen=True)
class HeadingHint:
    yaw: Optional[float] = None       # world rad
    face: Optional[str] = None        # an EE name: turn so its target lies along the arm's mount x axis
    point: Optional[tuple] = None     # (x, y) world: face that point
    weight: float = 0.5
    tol: float = 0.05
    omega: float = 0.0                # rad/s: the target heading's rate (feedforward)


@dataclass(frozen=True)
class VelocityHint:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    frame: str = "base"
    ttl: float = 0.3


@dataclass(frozen=True)
class BasePoseCmd:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class BaseLimits:
    max_speed: Optional[float] = None
    max_omega: Optional[float] = None
    max_accel: Optional[float] = None


@dataclass(frozen=True)
class EEHint:
    position: float = 1.0
    orientation: float = 0.3
    orientation_mask: tuple = (1.0, 1.0, 1.0)
    posture: object = "ready"         # a named posture or a joint vector (rad)
    posture_weight: float = 1e-4


@dataclass(frozen=True)
class Hints:
    mode: str = "free"
    target_pose: Optional[PoseHint] = None
    target_heading: Optional[HeadingHint] = None
    velocity: Optional[VelocityHint] = None
    pose: Optional[BasePoseCmd] = None
    limits: BaseLimits = BaseLimits()
    arm_vs_base: float = 1.0
    ees: dict = field(default_factory=dict)        # name -> EEHint

    def ee(self, name: str, cfg: Config) -> EEHint:
        return self.ees.get(name) or EEHint(cfg.w_position, cfg.w_orientation, (1.0, 1.0, 1.0), "ready",
                                            cfg.posture_weight)


def _num(d: dict, k: str, lo: float, hi: float, default=None, path: str = "") -> Optional[float]:
    v = d.get(k, default)
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f"{path}{k} must be a finite number")
    if not lo <= v <= hi:
        raise ValueError(f"{path}{k} must be within [{lo:g}, {hi:g}]")
    return float(v)


def _only(d, keys, path):
    if not isinstance(d, dict):
        raise ValueError(f"{path or 'hints'} must be an object")
    bad = set(d) - set(keys)
    if bad:
        raise ValueError(f"{path}: unknown keys {sorted(bad)}")


def _vel(d: dict, keys: tuple, cap: float, path: str, clamp: Optional[dict]) -> list:
    """Hint velocity components (default 0). ``clamp`` None (``POST /hints``): each component within ±``cap``, else
    refused (422). A dict (datagrams): beyond the cap (the norm of the components) they are scaled down to it,
    direction kept, and counted in ``clamp["n"]``: clamped, not dropped."""
    if clamp is None:
        return [_num(d, k, -cap, cap, 0.0, path) for k in keys]
    v = [_num(d, k, -math.inf, math.inf, 0.0, path) for k in keys]
    n = math.sqrt(sum(x * x for x in v))
    if n > cap:
        clamp["n"] = clamp.get("n", 0) + 1
        v = [x * cap / n for x in v]
    return v


def parse_base(b: dict, cfg: Config, ee_names, to_world=None, cur: Optional[Hints] = None,
               clamp: Optional[dict] = None) -> dict:
    """The ``base`` part of a hint document (or a per-command override) -> Hints field values.
    ``to_world(x, y, yaw) -> (x, y, yaw)`` converts base-frame poses (None: base frames refused). ``clamp`` (a dict,
    the datagram path): velocities beyond ``max_speed`` / ``max_omega`` are clamped to them and counted, not refused."""
    _only(b, ("mode", "target", "velocity", "pose", "limits", "arm_vs_base"), "base")
    out: dict = {}
    if "mode" in b:
        if b["mode"] not in MODES:
            raise ValueError(f"base.mode must be one of {', '.join(MODES)}")
        out["mode"] = b["mode"]

    def world(frame, x, y, yaw, path):
        if frame == "world":
            return x, y, yaw
        if frame == "base":
            if to_world is None:
                raise ValueError(f"{path}.frame base needs the estimator pose")
            return to_world(x, y, yaw)
        raise ValueError(f"{path}.frame must be world or base")

    if "target" in b:
        t = b["target"] or {}
        _only(t, ("pose", "heading"), "base.target")
        out["target_pose"] = out["target_heading"] = None
        if t.get("pose") is not None:
            p = t["pose"]
            _only(p, ("x", "y", "frame", "weight", "tol", "vx", "vy"), "base.target.pose")
            x, y, _ = world(p.get("frame", "world"), _num(p, "x", -1e3, 1e3, path="base.target.pose."),
                            _num(p, "y", -1e3, 1e3, path="base.target.pose."), 0.0, "base.target.pose")
            if x is None or y is None:
                raise ValueError("base.target.pose needs x and y")
            if p.get("frame", "world") != "world" and (p.get("vx") or p.get("vy")):
                raise ValueError("base.target.pose: a velocity needs frame world")
            pvx, pvy = _vel(p, ("vx", "vy"), cfg.max_speed, "base.target.pose.", clamp)
            out["target_pose"] = PoseHint(x, y, _num(p, "weight", 0, 100, 1.0, "base.target.pose."),
                                          _num(p, "tol", 0, 10, 0.05, "base.target.pose."), pvx, pvy)
        if t.get("heading") is not None:
            h = t["heading"]
            _only(h, ("yaw", "face", "frame", "weight", "tol", "omega"), "base.target.heading")
            w = _num(h, "weight", 0, 100, 0.5, "base.target.heading.")
            tol = _num(h, "tol", 0, math.pi, 0.05, "base.target.heading.")
            om = _vel(h, ("omega",), cfg.max_omega, "base.target.heading.", clamp)[0]
            face = h.get("face")
            if face is not None:
                if isinstance(face, str):
                    if face not in ee_names:
                        raise ValueError(f"base.target.heading.face: no end effector {face!r} "
                                         f"(have {', '.join(ee_names)})")
                    out["target_heading"] = HeadingHint(face=face, weight=w, tol=tol, omega=om)
                elif isinstance(face, (list, tuple)) and len(face) == 2:
                    fx, fy = (float(v) for v in face)
                    fx, fy, _ = world(h.get("frame", "world"), fx, fy, 0.0, "base.target.heading")
                    out["target_heading"] = HeadingHint(point=(fx, fy), weight=w, tol=tol, omega=om)
                else:
                    raise ValueError("base.target.heading.face must be an end effector name or a point [x, y]")
            else:
                yaw = _num(h, "yaw", -1e3, 1e3, path="base.target.heading.")
                if yaw is None:
                    raise ValueError("base.target.heading needs yaw or face")
                _, _, yaw = world(h.get("frame", "world"), 0.0, 0.0, yaw, "base.target.heading")
                out["target_heading"] = HeadingHint(yaw=yaw, weight=w, tol=tol, omega=om)
    if "velocity" in b:
        v = b["velocity"]
        if v is None:
            out["velocity"] = None
        else:
            _only(v, ("vx", "vy", "omega", "frame", "ttl"), "base.velocity")
            fr = v.get("frame", "base")
            if fr not in ("base", "world"):
                raise ValueError("base.velocity.frame must be base or world")
            vvx, vvy = _vel(v, ("vx", "vy"), cfg.max_speed, "base.velocity.", clamp)
            vom = _vel(v, ("omega",), cfg.max_omega, "base.velocity.", clamp)[0]
            out["velocity"] = VelocityHint(vvx, vvy, vom, fr, _num(v, "ttl", 0.0, cfg.max_ttl, 0.3, "base.velocity."))
    if "pose" in b:
        p = b["pose"]
        if p is None:
            out["pose"] = None
        else:
            _only(p, ("x", "y", "yaw", "frame"), "base.pose")
            x, y, yaw = (_num(p, k, -1e3, 1e3, 0.0 if k == "yaw" else None, "base.pose.") for k in ("x", "y", "yaw"))
            if x is None or y is None:
                raise ValueError("base.pose needs x and y")
            out["pose"] = BasePoseCmd(*world(p.get("frame", "world"), x, y, yaw, "base.pose"))
    if "limits" in b:
        lim = b["limits"] or {}
        _only(lim, ("max_speed", "max_omega", "max_accel"), "base.limits")
        out["limits"] = BaseLimits(_num(lim, "max_speed", 0, cfg.max_speed, path="base.limits."),
                                   _num(lim, "max_omega", 0, cfg.max_omega, path="base.limits."),
                                   _num(lim, "max_accel", 0, cfg.max_accel, path="base.limits."))
    if "arm_vs_base" in b:
        out["arm_vs_base"] = _num(b, "arm_vs_base", 0.01, 100.0, path="base.")
    return out


def parse_ee(name: str, e: dict, cfg: Config, postures: dict, n_joints: int) -> EEHint:
    _only(e, ("weights", "orientation_mask", "posture", "posture_weight"), f"ees.{name}")
    w = e.get("weights") or {}
    _only(w, ("position", "orientation"), f"ees.{name}.weights")
    mask = e.get("orientation_mask", [1, 1, 1])
    if not (isinstance(mask, (list, tuple)) and len(mask) == 3 and all(isinstance(m, (int, float)) and 0 <= m <= 1
                                                                       for m in mask)):
        raise ValueError(f"ees.{name}.orientation_mask must be three numbers in [0, 1]")
    posture = e.get("posture", "ready")
    if isinstance(posture, str):
        if posture not in postures:
            raise ValueError(f"ees.{name}.posture: unknown named posture {posture!r} (have {', '.join(postures)})")
    else:
        v = np.asarray(posture, float)
        if v.shape != (n_joints,) or not np.all(np.isfinite(v)):
            raise ValueError(f"ees.{name}.posture must be a name or {n_joints} joint angles (rad)")
        posture = tuple(float(x) for x in v)
    return EEHint(_num(w, "position", 0, 100, cfg.w_position, f"ees.{name}.weights."),
                  _num(w, "orientation", 0, 100, cfg.w_orientation, f"ees.{name}.weights."),
                  tuple(float(m) for m in mask), posture,
                  _num(e, "posture_weight", 0, 1.0, cfg.posture_weight, f"ees.{name}."))


def parse_hints(doc: dict, cfg: Config, ee_names, postures: dict, n_joints=6, to_world=None,
                cur: Optional[Hints] = None) -> Hints:
    """A full hint document (``base``, ``ees``): absent keys take the defaults (not the hints in force)."""
    doc = doc or {}
    _only(doc, ("base", "ees"), "")
    h = Hints(**parse_base(doc.get("base") or {}, cfg, ee_names, to_world))
    ees = {}
    for name, e in (doc.get("ees") or {}).items():
        if name not in ee_names:
            raise ValueError(f"ees: no end effector {name!r} (have {', '.join(ee_names)})")
        ees[name] = parse_ee(name, e or {}, cfg, postures,
                             n_joints.get(name, 6) if isinstance(n_joints, dict) else n_joints)
    return replace(h, ees=ees)


def merge(h: Hints, override: dict) -> Hints:
    """``override``: parsed base fields (``parse_base``) applied over ``h``."""
    return replace(h, **override) if override else h


def hints_public(h: Hints) -> dict:
    """The hint structure back as JSON (world frame)."""
    base: dict = {"mode": h.mode, "arm_vs_base": h.arm_vs_base}
    tgt: dict = {}
    if h.target_pose:
        tgt["pose"] = {**asdict(h.target_pose), "frame": "world"}
    if h.target_heading:
        hh = h.target_heading
        d = {"weight": hh.weight, "tol": hh.tol, "omega": hh.omega}
        if hh.face:
            d["face"] = hh.face
        elif hh.point:
            d.update(face=list(hh.point), frame="world")
        else:
            d.update(yaw=hh.yaw, frame="world")
        tgt["heading"] = d
    base["target"] = tgt or None
    base["velocity"] = asdict(h.velocity) if h.velocity else None
    base["pose"] = {**asdict(h.pose), "frame": "world"} if h.pose else None
    base["limits"] = {k: v for k, v in asdict(h.limits).items() if v is not None}
    ees = {k: {"weights": {"position": e.position, "orientation": e.orientation},
               "orientation_mask": list(e.orientation_mask),
               "posture": e.posture if isinstance(e.posture, str) else list(e.posture),
               "posture_weight": e.posture_weight} for k, e in h.ees.items()}
    return {"base": base, "ees": ees}


def effective_limits(h: Hints, cfg: Config) -> tuple[float, float, float]:
    lim = h.limits
    return (min(cfg.max_speed, lim.max_speed if lim.max_speed is not None else math.inf),
            min(cfg.max_omega, lim.max_omega if lim.max_omega is not None else math.inf),
            min(cfg.max_accel, lim.max_accel if lim.max_accel is not None else math.inf))


def copy_hints(h: Hints) -> Hints:
    return copy.deepcopy(h)
