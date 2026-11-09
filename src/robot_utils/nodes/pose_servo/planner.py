"""Pose servo controller: receding-horizon kinematic planner + constraint limiter. Pure, no I/O.

The node (node.py) calls ``Servo.update(pose, dt)`` every control period with the latest pose
estimate and sends the returned body twist to the swerve node.

**Commanded state.** The servo integrates its own commanded body twist ``u = (vx, vy, w)`` and
its rate ``a = (ax, ay, al)`` (the twist's finite difference per period). The constraints apply
to this sequence of twists, the thing actually sent, so they are checked exactly:

- speed ``|(vx, vy)| <= max_speed``, ``|w| <= max_omega``
- acceleration ``|(ax, ay)| <= max_accel``, ``|al| <= max_alpha``  (= (u_k - u_{k-1}) / dt)
- jerk ``|(jx, jy)| <= max_jerk``, ``|jr| <= max_ang_jerk``        (= (a_k - a_{k-1}) / dt)
- each wheel's speed ``<= WHEEL_SPEED_MAX`` (3 m/s)
- each wheel's steering rate ``<= STEER_RATE_MAX`` (4 pi rad/s): the change of the wheel
  velocity's direction (mod pi: a swerve module may flip) between consecutive twists, for
  wheels moving at least ``STEER_MIN_SPEED`` in both (slower, the direction is undefined).

A new goal starts from the current commanded state (the base may be moving), never from rest.

**Planning, every period (receding horizon).** From the current pose error:

- translation: a desired world velocity toward the goal, of the largest speed from which a
  jerk-limited stop (at ``PLAN_MARGIN`` of the motion's accel / jerk) ends at the goal, capped
  by the motion speed and, near the goal, by ``d / TAU_POS`` (a linear final approach);
- heading: the same, 1-D;
- synchronized mode: the time to go of each axis (closed-form, jerk-limited, from its current
  velocity) is predicted and the faster axis's speed / accel / jerk are scaled by s, s^2, s^3
  (a time scaling by 1/s) so that both end together. Independent: no scaling.

The desired body twist is tracked by a jerk-limited velocity law: the acceleration that reaches
the desired velocity after unwinding the current acceleration (``a|a| / 2J``), capped by the
motion's accel, ``sqrt(2 J |e|)`` and ``|e| / TAU_VEL``; the jerk takes it there in one period
if allowed.

**Static friction.** The real base does not move for a few cm/s at the wheels, so the linear
final approach could stall just short of the goal. When the error has not improved over a
``BOOST_WINDOW`` although motion is commanded, a wheel-speed floor (``boost``) rises by
``BOOST_STEP`` per window up to ``min_wheel_speed``; the desired twist is scaled up (axis ratio
kept, so synchronization holds) until its fastest wheel reaches it, with no axis faster than
what stops within the hold band past its goal. It holds while the base moves and resets when
the goal is settled or replaced; a base that follows its commands never gets one. A goal whose error stops improving for ``stall_s`` ends as ``stalled`` (braking).

**Acceptance region.** Each axis latches *done* inside its tolerance (it then only brakes and
holds still) and is released only beyond ``hold_band`` x the tolerance: no hunting at the goal.
With both done the goal is reached: ``terminate`` brakes to zero and ends; ``hold`` keeps
servoing (an axis pushed out of its band moves back).

**Limiter (the guarantee).** The planner's jerk is projected onto the jerk ball and the
acceleration ball (projection onto a convex set never increases the jerk), then checked
against the speed limits (with the speed the current acceleration will still add while it is
unwound: a safe set that can always be kept), the wheel speeds and the steering rates. If the
step fails, the largest fraction of it toward the "unwind the acceleration" jerk that passes is
taken (bisection). Every twist is finally verified; a violation (not expected) is counted in
``violations`` and reported.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, replace
from typing import Optional

from ..swerve.constants import MODULES

# Hard constraints (fixed in code).
STEER_RATE_MAX = 4 * math.pi       # rad/s, each wheel's steering (direction of its velocity)
WHEEL_SPEED_MAX = 3.0              # m/s, each wheel
STEER_MIN_SPEED = 0.02             # m/s: slower wheels have no defined direction (not rate-limited)
WHEEL_R = max(math.hypot(m.x, m.y) for m in MODULES)   # m, centre to the farthest wheel

# Planner tuning.
PLAN_MARGIN = 0.8      # plan stops with this share of the motion's accel and jerk (the rest = tracking headroom)
TAU_VEL = 0.10         # s, velocity tracking time constant near zero error
TAU_POS = 0.40         # s, final-approach time constant (desired speed <= error / TAU_POS)
SYNC_MIN_SCALE = 0.02  # lowest time scale of the faster axis in synchronized mode
LOOKAHEAD_DT = 0.02    # s, added to the steering look-ahead (one control period)
STEER_SHAPE = 0.5      # the planner turns wheels at most this share of STEER_RATE_MAX


# --------------------------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Caps:
    """Hard per-enable limits on every commanded twist (the safety config)."""
    max_speed: float = 0.3       # m/s, |(vx, vy)|
    max_omega: float = 0.8       # rad/s
    max_accel: float = 0.5       # m/s^2, |(ax, ay)|
    max_alpha: float = 1.5       # rad/s^2
    max_jerk: float = 2.0        # m/s^3
    max_ang_jerk: float = 6.0    # rad/s^3


@dataclass(frozen=True)
class Motion:
    """How one goal is approached (each value within the caps)."""
    speed: float
    omega: float
    accel: float
    alpha: float
    jerk: float
    ang_jerk: float
    mode: str = "synchronized"      # or "independent"
    pos_tol: float = 0.02           # m
    heading_tol: float = 0.035      # rad
    on_reach: str = "terminate"     # or "hold"
    hold_band: float = 2.0          # an axis done inside tol is released beyond hold_band * tol
    min_wheel_speed: float = 0.08   # m/s: up to this wheel speed is added while the base is not moving (static friction)
    stall_s: float = 3.0            # s without progress -> the goal ends as "stalled" (0 = never)

    @classmethod
    def from_caps(cls, caps: Caps, **kw) -> "Motion":
        return cls(caps.max_speed, caps.max_omega, caps.max_accel, caps.max_alpha, caps.max_jerk,
                   caps.max_ang_jerk, **kw)


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    theta: float          # continuous (not wrapped): the heading to reach, turns included
    motion: Motion
    goal_id: int = 0


@dataclass
class Cmd:
    """Commanded body twist and its rate (as sent)."""
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    al: float = 0.0

    def is_zero(self) -> bool:
        return not any((self.vx, self.vy, self.w, self.ax, self.ay, self.al))


# --------------------------------------------------------------------------------------------
# 1-D jerk-limited profile formulas (rest <-> v with accel A, jerk J; S-curve)
# --------------------------------------------------------------------------------------------

def stop_distance(v: float, A: float, J: float) -> float:
    """Distance to stop from speed v (accel 0) with accel <= A, jerk <= J (= to reach v from rest)."""
    v = abs(v)
    if v * J >= A * A:
        return 0.5 * v * (v / A + A / J)
    return v * math.sqrt(v / J)


def speed_for_distance(d: float, A: float, J: float) -> float:
    """Largest speed from which a jerk-limited stop takes at most d (inverse of stop_distance)."""
    if d <= 0:
        return 0.0
    if d >= A ** 3 / J ** 2:
        k = A * A / J
        return 0.5 * (-k + math.sqrt(k * k + 8 * A * d))
    return (d * d * J) ** (1 / 3)


def accel_time(v: float, A: float, J: float) -> float:
    """Time from rest to speed v (accel back to 0)."""
    v = abs(v)
    if v * J >= A * A:
        return v / A + A / J
    return 2 * math.sqrt(v / J)


def rest_to_rest_time(d: float, V: float, A: float, J: float) -> float:
    if d <= 0:
        return 0.0
    if d >= 2 * stop_distance(V, A, J):
        return 2 * accel_time(V, A, J) + (d - 2 * stop_distance(V, A, J)) / V
    return 2 * accel_time(speed_for_distance(d / 2, A, J), A, J)


def time_to_go(d: float, v0: float, V: float, A: float, J: float) -> float:
    """Approximate time to cover d >= 0 and stop, starting at v0 toward the target (negative =
    away), from a virtual rest-to-rest profile."""
    V = max(V, 1e-9)
    if v0 >= 0:
        v0 = min(v0, V)
        return max(rest_to_rest_time(d + stop_distance(v0, A, J), V, A, J) - accel_time(v0, A, J), 0.0)
    u = min(-v0, V)
    return accel_time(u, A, J) + rest_to_rest_time(d + stop_distance(u, A, J), V, A, J)


def approach_time(d: float, v0: float, V: float, A: float, J: float, tol: float) -> float:
    """Predicted time until the error is within ``tol``: the S-curve time to go plus the final
    linear approach (desired speed = d / TAU_POS below the distance d* where that law takes over
    from the stopping-speed law: an exponential tail TAU_POS ln(d* / tol))."""
    if d <= tol:
        return 0.0
    d_star = max(min(V * TAU_POS, J * TAU_POS ** 3), tol)     # where d / TAU_POS meets the profile
    tail = TAU_POS * math.log(min(d, d_star) / tol)
    return time_to_go(max(d - d_star, 0.0), v0, V, A, J) + tail


# --------------------------------------------------------------------------------------------
# Small vector helpers (plain floats: faster than numpy for 2-vectors)
# --------------------------------------------------------------------------------------------

def _clip_norm(x: float, y: float, m: float) -> tuple[float, float]:
    n = math.hypot(x, y)
    if n > m:
        k = m / n if n > 0 else 0.0
        return x * k, y * k
    return x, y


def _track(e_x: float, e_y: float, a_x: float, a_y: float, A: float, J: float) -> tuple[float, float]:
    """Jerk-limited velocity tracking (2-D; pass e_y = a_y = 0 for 1-D): desired acceleration
    toward a velocity error e, given the current acceleration a."""
    na = math.hypot(a_x, a_y)
    rx, ry = e_x - a_x * na / (2 * J), e_y - a_y * na / (2 * J)   # error left after unwinding a
    n = math.hypot(rx, ry)
    if n < 1e-12:
        return 0.0, 0.0
    mag = min(A, math.sqrt(2 * J * n), n / TAU_VEL)
    return rx / n * mag, ry / n * mag


def _track1(e: float, a: float, A: float, J: float) -> float:
    return _track(e, 0.0, a, 0.0, A, J)[0]


def wheel_vectors(vx: float, vy: float, w: float) -> list[tuple[float, float]]:
    return [(vx - w * m.y, vy + w * m.x) for m in MODULES]


def steer_rate(prev: tuple, new: tuple, dt: float) -> float:
    """Largest steering rate (rad/s) of the wheels between two body twists, counting only wheels
    at >= STEER_MIN_SPEED in both; direction compared mod pi (modules may flip)."""
    worst = 0.0
    for (px, py), (nx, ny) in zip(wheel_vectors(*prev), wheel_vectors(*new)):
        if math.hypot(px, py) < STEER_MIN_SPEED or math.hypot(nx, ny) < STEER_MIN_SPEED:
            continue
        d = math.atan2(px * ny - py * nx, px * nx + py * ny)        # signed angle prev -> new
        d = (d + math.pi / 2) % math.pi - math.pi / 2                # mod pi
        worst = max(worst, abs(d) / dt)
    return worst


def shape_for_steering(c: "Cmd", ax: float, ay: float, al: float, J: float, Jr: float,
                       share: float = STEER_SHAPE) -> tuple[float, float, float]:
    """Scale down the part of a desired twist acceleration that turns the wheels.

    Wheel velocities are linear in the twist, so the part of the acceleration parallel to the
    current twist (in (vx, vy, w * WHEEL_R) space) only changes wheel speeds; the perpendicular
    part turns them. It is scaled so that no wheel moving at >= STEER_MIN_SPEED would turn faster
    than ``share`` x STEER_RATE_MAX: the planner stays clear of the limit, leaving the limiter the
    margin it needs (the jerk limit cannot remove a sideways acceleration at once).

    Look-ahead: a wheel's speed is taken as the lowest it reaches (never below
    STEER_MIN_SPEED) while the current acceleration is unwound at the jerk limits, so the
    acceleration is reduced before a wheel slows down (e.g. as the centre of rotation sweeps
    past it), not when it already has."""
    R = WHEEL_R
    u = (c.vx, c.vy, c.w * R)
    uu = u[0] ** 2 + u[1] ** 2 + u[2] ** 2
    if uu < 1e-18:
        return ax, ay, al
    a = (ax, ay, al * R)
    k = (a[0] * u[0] + a[1] * u[1] + a[2] * u[2]) / uu
    par = (k * u[0], k * u[1], k * u[2])
    px, py, pr = a[0] - par[0], a[1] - par[1], (a[2] - par[2]) / R      # perpendicular part
    h = max(math.hypot(c.ax, c.ay) / J, abs(c.al) / Jr) + 2 * LOOKAHEAD_DT
    taus = [h * i / 4 for i in range(1, 5)]
    future = [wheel_vectors(c.vx + c.ax * t, c.vy + c.ay * t, c.w + c.al * t) for t in taus]
    worst = 0.0
    for i, (m, (wx, wy)) in enumerate(zip(MODULES, wheel_vectors(c.vx, c.vy, c.w))):
        sp = math.hypot(wx, wy)
        if sp < STEER_MIN_SPEED:
            continue
        s_eff = max(STEER_MIN_SPEED, min([sp] + [math.hypot(*f[i]) for f in future]))
        dx, dy = px - pr * m.y, py + pr * m.x
        worst = max(worst, abs(wx * dy - wy * dx) / (sp * s_eff))
    allowed = share * STEER_RATE_MAX
    f = 1.0 if worst <= allowed else allowed / worst
    return par[0] + f * px, par[1] + f * py, (par[2] / R) + f * pr


def max_wheel_speed(vx: float, vy: float, w: float) -> float:
    return max(math.hypot(x, y) for x, y in wheel_vectors(vx, vy, w))


# --------------------------------------------------------------------------------------------
# Limiter
# --------------------------------------------------------------------------------------------

@dataclass
class StepCheck:
    speed: float
    omega: float
    accel: float
    alpha: float
    jerk: float
    ang_jerk: float
    wheel: float
    steer: float


def check_step(prev: Cmd, new: Cmd, dt: float) -> StepCheck:
    """Measured quantities of one step (what the tests and the node verify)."""
    return StepCheck(
        speed=math.hypot(new.vx, new.vy), omega=abs(new.w),
        accel=math.hypot(new.vx - prev.vx, new.vy - prev.vy) / dt, alpha=abs(new.w - prev.w) / dt,
        jerk=math.hypot(new.ax - prev.ax, new.ay - prev.ay) / dt, ang_jerk=abs(new.al - prev.al) / dt,
        wheel=max_wheel_speed(new.vx, new.vy, new.w),
        steer=steer_rate((prev.vx, prev.vy, prev.w), (new.vx, new.vy, new.w), dt))


def within(c: StepCheck, caps: Caps, tol: float = 1e-6) -> list[str]:
    """Names of the violated constraints (empty = all held)."""
    lim = {"speed": caps.max_speed, "omega": caps.max_omega, "accel": caps.max_accel,
           "alpha": caps.max_alpha, "jerk": caps.max_jerk, "ang_jerk": caps.max_ang_jerk,
           "wheel": WHEEL_SPEED_MAX, "steer": STEER_RATE_MAX}
    return [k for k, m in lim.items() if getattr(c, k) > m * (1 + tol) + 1e-9]


def _integrate(c: Cmd, jx: float, jy: float, jr: float, dt: float) -> Cmd:
    ax, ay, al = c.ax + jx * dt, c.ay + jy * dt, c.al + jr * dt
    return Cmd(c.vx + ax * dt, c.vy + ay * dt, c.w + al * dt, ax, ay, al)


def _unwind_accel(c: Cmd, caps: Caps, dt: float) -> tuple[float, float, float]:
    """The fallback: the next acceleration with the current one ramped toward 0 as fast as the
    jerk caps allow (linear and angular separately; exactly 0 once within one step). With zero
    acceleration the twist stays constant."""
    na = math.hypot(c.ax, c.ay)
    f = max(0.0, 1.0 - caps.max_jerk * dt / na) if na > 0 else 0.0
    al = math.copysign(max(0.0, abs(c.al) - caps.max_ang_jerk * dt), c.al)
    return c.ax * f, c.ay * f, al


def _with_accel(c: Cmd, ax: float, ay: float, al: float, dt: float) -> Cmd:
    return Cmd(c.vx + ax * dt, c.vy + ay * dt, c.w + al * dt, ax, ay, al)


def _step_ok(prev: Cmd, n: Cmd, caps: Caps, dt: float) -> bool:
    """Velocity-level constraints of one step (accel / jerk hold by construction)."""
    eps = 1 + 1e-9
    if math.hypot(n.vx, n.vy) > caps.max_speed * eps or abs(n.w) > caps.max_omega * eps:
        return False
    if max_wheel_speed(n.vx, n.vy, n.w) > WHEEL_SPEED_MAX:
        return False
    return steer_rate((prev.vx, prev.vy, prev.w), (n.vx, n.vy, n.w), dt) <= STEER_RATE_MAX * eps


MAX_ROLLOUT = 2000    # steps; unwinding takes max_accel / (max_jerk dt) at most


def _safe(prev: Cmd, n: Cmd, caps: Caps, dt: float) -> bool:
    """The step prev -> n is allowed AND n is recoverable: unwinding its acceleration with the
    fallback jerk (what limit_step does when nothing else passes) keeps every constraint until
    the acceleration is 0 (from then on the twist is constant: no steering, same speeds).
    So an accepted state always has an allowed next step (the rollout's first one): the
    commanded sequence can never be cornered into a violation."""
    if not _step_ok(prev, n, caps, dt):
        return False
    c = n
    for _ in range(MAX_ROLLOUT):
        if c.ax == 0.0 and c.ay == 0.0 and c.al == 0.0:
            return True
        nxt = _with_accel(c, *_unwind_accel(c, caps, dt), dt)
        if not _step_ok(c, nxt, caps, dt):
            return False
        c = nxt
    return False


def limit_step(c: Cmd, jx: float, jy: float, jr: float, caps: Caps, dt: float) -> tuple[Cmd, float]:
    """The next commanded state from a desired jerk, within every constraint.
    Returns (state, fraction of the desired step kept: 1 = unmodified by the safety checks).

    Jerk and acceleration are clipped to their balls (projecting the acceleration onto its ball
    never increases the jerk: projection onto a convex set is non-expansive and the current
    acceleration is inside). If the result is not ``_safe``, the largest share of the step
    between the fallback jerk (share 0) and the desired one that is safe is taken; share 0 is
    safe whenever the current state is, so the constraints always hold."""
    jx, jy = _clip_norm(jx, jy, caps.max_jerk)
    jr = max(-caps.max_ang_jerk, min(caps.max_ang_jerk, jr))
    ax, ay = _clip_norm(c.ax + jx * dt, c.ay + jy * dt, caps.max_accel)
    al = max(-caps.max_alpha, min(caps.max_alpha, c.al + jr * dt))
    n = _with_accel(c, ax, ay, al, dt)
    if _safe(c, n, caps, dt):
        return n, 1.0
    fx, fy, fr = _unwind_accel(c, caps, dt)        # share 0: the fallback (exact)
    lo, hi = 0.0, 1.0
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        m = _with_accel(c, fx + mid * (ax - fx), fy + mid * (ay - fy), fr + mid * (al - fr), dt)
        if _safe(c, m, caps, dt):
            lo = mid
        else:
            hi = mid
    return _with_accel(c, fx + lo * (ax - fx), fy + lo * (ay - fy), fr + lo * (al - fr), dt), lo


# --------------------------------------------------------------------------------------------
# The servo
# --------------------------------------------------------------------------------------------

STATUS = {"disabled": 0, "idle": 1, "moving": 2, "holding": 3, "braking": 4}
STALL_POS = 0.005      # m: less improvement than this of the best distance is not progress
STALL_THETA = 0.01     # rad
BOOST_WINDOW = 0.25    # s: progress check of the static-friction boost
BOOST_STEP = 0.02      # m/s of wheel speed added per window without progress
BOOST_POS = 0.002      # m / rad of error improvement per window that counts as moving
BOOST_THETA = 0.004


@dataclass
class Diag:
    """Per-update diagnostics (one stream row with the twist)."""
    status: str = "idle"
    goal_id: int = 0
    err_pos: float = math.nan       # m, distance to the goal
    err_theta: float = math.nan     # rad, goal - heading
    vdes_x: float = math.nan        # desired body velocity (before tracking)
    vdes_y: float = math.nan
    wdes: float = math.nan
    scale_lin: float = math.nan     # synchronized-mode time scale of each axis (1 = full)
    scale_rot: float = math.nan
    ttg_lin: float = math.nan       # predicted time to go, s
    ttg_rot: float = math.nan
    lin_done: float = 0.0
    rot_done: float = 0.0
    kept: float = 1.0               # share of the planned step the limiter kept
    boost: float = 0.0              # m/s static-friction wheel-speed floor in force
    violations: int = 0             # steps that broke a constraint (expected 0)


class Servo:
    """Goal + commanded state; ``update`` once per period. Not thread safe (one control thread)."""

    def __init__(self, caps: Caps = Caps()):
        self.caps = caps
        self.cmd = Cmd()
        self.goal: Optional[Goal] = None
        self.braking: Optional[str] = None      # reason, while braking to a stop without a goal
        self.lin_done = self.rot_done = False
        self.result: Optional[dict] = None      # how the last goal ended
        self.violations = 0
        self.last_check: Optional[StepCheck] = None
        self._t = 0.0
        self._progress = (math.inf, math.inf, 0.0)     # best distance, best |heading error|, when
        self.boost = 0.0                               # m/s wheel-speed floor (static friction), adaptive
        self._boost_ref = (math.inf, math.inf, 0.0)    # errors at the start of the boost window

    # -- commands ---------------------------------------------------------------------------

    def set_goal(self, goal: Goal) -> None:
        """Replace the goal; the motion continues from the current commanded state."""
        self.goal, self.braking = goal, None
        self.lin_done = self.rot_done = False
        self.result = None
        self._progress = (math.inf, math.inf, self._t)
        self.boost, self._boost_ref = 0.0, (math.inf, math.inf, self._t)

    def stop(self, reason: str, result: str = "stopped") -> None:
        """Brake to zero within the caps (no pose needed) and drop the goal."""
        if self.goal is not None:
            self.result = {"goal_id": self.goal.goal_id, "result": result, "reason": reason}
        self.goal = None
        self.braking = reason if not self.cmd.is_zero() else None

    def reset(self) -> None:
        """Forget everything (the swerve was stopped outside the servo: disable, e-stop)."""
        self.cmd = Cmd()
        if self.goal is not None:
            self.result = {"goal_id": self.goal.goal_id, "result": "aborted", "reason": "disabled"}
        self.goal, self.braking = None, None

    @property
    def status(self) -> str:
        if self.goal is not None:
            return "holding" if (self.lin_done and self.rot_done) else "moving"
        return "braking" if self.braking is not None else "idle"

    @property
    def active(self) -> bool:
        """True while twists must be sent (moving, holding or braking)."""
        return self.goal is not None or self.braking is not None

    # -- the control step -------------------------------------------------------------------

    def update(self, pose: Optional[tuple[float, float, float]], dt: float) -> tuple[Optional[Cmd], Diag]:
        """One period. ``pose`` = (x, y, theta) now (None is allowed only without a goal).
        Returns the twist to send (None = nothing to send: idle) and diagnostics."""
        d = Diag(goal_id=self.goal.goal_id if self.goal else 0)
        self._t += dt
        if not self.active:
            d.status = "idle"
            return None, d
        caps, c = self.caps, self.cmd
        g = self.goal
        if g is None:      # braking: desired twist 0, with the caps' accel / jerk
            m = Motion.from_caps(caps)
            vdx = vdy = wd = 0.0
        else:
            m = g.motion
            assert pose is not None
            vdx, vdy, wd = self._plan(g, pose, d)
            if self._stalled(g, d):
                self.stop(f"no progress for {g.motion.stall_s:g} s (error {d.err_pos:.3f} m, "
                          f"{d.err_theta:+.3f} rad): the base is not following", "stalled")
                g, m = None, Motion.from_caps(caps)
                vdx = vdy = wd = 0.0
                if self.braking is None:          # already at rest
                    d.status = "idle"
                    return None, d
        d.vdes_x, d.vdes_y, d.wdes = vdx, vdy, wd
        # track the desired body twist (motion accel / jerk); limiter enforces the caps
        tax, tay = _track(vdx - c.vx, vdy - c.vy, c.ax, c.ay, m.accel, m.jerk)
        tal = _track1(wd - c.w, c.al, m.alpha, m.ang_jerk)
        tax, tay, tal = shape_for_steering(c, tax, tay, tal, m.jerk, m.ang_jerk)
        jx, jy = (tax - c.ax) / dt, (tay - c.ay) / dt
        jr = (tal - c.al) / dt
        jx, jy = _clip_norm(jx, jy, m.jerk)
        jr = max(-m.ang_jerk, min(m.ang_jerk, jr))
        n, kept = limit_step(c, jx, jy, jr, caps, dt)
        d.kept = kept
        # finishing a stop: land exactly on zero when that step is allowed
        settled = g is not None and self.lin_done and self.rot_done     # braking to rest at the goal
        finishing = g is None or (settled and g.motion.on_reach == "terminate")
        if (finishing or settled) and abs(n.vx) + abs(n.vy) + abs(n.w) < 2e-3:
            z = Cmd()
            if not within(check_step(c, z, dt), caps):
                n = z
        chk = check_step(c, n, dt)
        bad = within(chk, caps)
        if bad:
            self.violations += 1
        d.violations = self.violations
        self.last_check = chk
        self.cmd = n
        if n.is_zero() and finishing:
            if g is not None:
                self.result = {"goal_id": g.goal_id, "result": "reached", "reason": ""}
                self.goal = None
            self.braking = None
        d.status = self.status if self.active else "idle"
        if finishing and g is not None and self.goal is not None:
            d.status = "braking"
        d.lin_done, d.rot_done = float(self.lin_done), float(self.rot_done)
        return n, d

    def _update_boost(self, g: Goal, d: Diag, commanding: bool) -> None:
        """Adaptive static-friction floor: rises while the error does not improve although motion is
        commanded, holds while moving, resets once both axes are done. A base that follows its
        commands never gets one (no effect in simulation or on a smooth floor)."""
        if self.lin_done and self.rot_done:
            self.boost, self._boost_ref = 0.0, (d.err_pos, abs(d.err_theta), self._t)
            return
        p0, t0, since = self._boost_ref
        if self._t - since < BOOST_WINDOW:
            return
        moved = d.err_pos < p0 - BOOST_POS or abs(d.err_theta) < t0 - BOOST_THETA
        if not moved and commanding:
            self.boost = min(g.motion.min_wheel_speed, self.boost + BOOST_STEP)
        self._boost_ref = (d.err_pos, abs(d.err_theta), self._t)

    def _stalled(self, g: Goal, d: Diag) -> bool:
        """True when a moving goal's error has not improved for stall_s (the base is stuck,
        e.g. commanded speeds the drive does not respond to). Holding / settled resets it."""
        if g.motion.stall_s <= 0 or (self.lin_done and self.rot_done):
            self._progress = (d.err_pos, abs(d.err_theta), self._t)
            return False
        bp, bt, since = self._progress
        if d.err_pos < bp - STALL_POS or abs(d.err_theta) < bt - STALL_THETA:
            self._progress = (min(bp, d.err_pos), min(bt, abs(d.err_theta)), self._t)
            return False
        return self._t - since > g.motion.stall_s

    def _plan(self, g: Goal, pose, d: Diag) -> tuple[float, float, float]:
        x, y, th = pose
        m = g.motion
        ex, ey = g.x - x, g.y - y
        dist = math.hypot(ex, ey)
        eth = g.theta - th
        d.err_pos, d.err_theta = dist, eth
        # acceptance region with hysteresis
        if dist <= m.pos_tol:
            self.lin_done = True
        elif dist > m.hold_band * m.pos_tol:
            self.lin_done = False
        if abs(eth) <= m.heading_tol:
            self.rot_done = True
        elif abs(eth) > m.hold_band * m.heading_tol:
            self.rot_done = False
        c = self.cmd
        # current commanded velocity in the world frame
        ct, st = math.cos(th), math.sin(th)
        vwx, vwy = ct * c.vx - st * c.vy, st * c.vx + ct * c.vy
        ux, uy = (ex / dist, ey / dist) if dist > 1e-9 else (0.0, 0.0)
        v_along = vwx * ux + vwy * uy
        w_along = c.w * (1.0 if eth >= 0 else -1.0)
        A_l, J_l = PLAN_MARGIN * m.accel, PLAN_MARGIN * m.jerk
        A_r, J_r = PLAN_MARGIN * m.alpha, PLAN_MARGIN * m.ang_jerk
        s_lin = s_rot = 1.0
        lin_active, rot_active = not self.lin_done, not self.rot_done
        t_lin = approach_time(dist, v_along, m.speed, A_l, J_l, m.pos_tol) if lin_active else 0.0
        t_rot = approach_time(abs(eth), w_along, m.omega, A_r, J_r, m.heading_tol) if rot_active else 0.0
        d.ttg_lin, d.ttg_rot = t_lin, t_rot
        if m.mode == "synchronized" and lin_active and rot_active and t_lin > 0 and t_rot > 0:
            if t_lin < t_rot:
                s_lin = max(SYNC_MIN_SCALE, t_lin / t_rot)
            else:
                s_rot = max(SYNC_MIN_SCALE, t_rot / t_lin)
        d.scale_lin, d.scale_rot = s_lin, s_rot
        # desired speeds (stop at the goal with the scaled limits; linear final approach)
        sp = 0.0
        if lin_active:
            sp = min(m.speed * s_lin, speed_for_distance(dist, A_l * s_lin ** 2, J_l * s_lin ** 3),
                     dist * s_lin / TAU_POS)
            vdw = (ux * sp, uy * sp)
        else:
            vdw = (0.0, 0.0)
        if rot_active:
            w = min(m.omega * s_rot, speed_for_distance(abs(eth), A_r * s_rot ** 2, J_r * s_rot ** 3),
                    abs(eth) * s_rot / TAU_POS)
            wd = math.copysign(w, eth)
        else:
            wd = 0.0
        # world -> body at the heading half a step ahead is not worth it; use the current one
        vdx, vdy = ct * vdw[0] + st * vdw[1], -st * vdw[0] + ct * vdw[1]
        # static friction: while the base does not move, scale the whole desired twist (axis ratio kept)
        # so its fastest wheel turns at least `boost`; no axis faster than what stops within the hold
        # band past its goal (an overshoot never releases a done axis: no hunting)
        fastest = max_wheel_speed(vdx, vdy, wd)
        self._update_boost(g, d, fastest > 0)
        d.boost = self.boost
        if 0 < fastest < self.boost:
            k = self.boost / fastest
            if sp > 0:
                k = min(k, min(m.speed, speed_for_distance(dist + m.hold_band * m.pos_tol, A_l, J_l)) / sp)
            if wd != 0:
                k = min(k, min(m.omega, speed_for_distance(abs(eth) + m.hold_band * m.heading_tol, A_r, J_r)) / abs(wd))
            if k > 1:
                vdx, vdy, wd = vdx * k, vdy * k, wd * k
        return vdx, vdy, wd


def motion_dict(m: Motion) -> dict:
    return asdict(m)


def caps_dict(c: Caps) -> dict:
    return asdict(c)


__all__ = ["Caps", "Motion", "Goal", "Cmd", "Servo", "Diag", "STATUS", "limit_step", "check_step", "within",
           "STEER_RATE_MAX", "WHEEL_SPEED_MAX", "STEER_MIN_SPEED", "WHEEL_R", "replace", "field"]
