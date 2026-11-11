"""Planned motions for the arm node: servo (a Cartesian move to a pose), joint-space moves to named
poses (home), and action chunks. Pure computation; the node executes the result.

Servo is rebot_ctl's end-pose controller pipeline (min-jerk SE(3) geodesic + CLIK tracker), as
tested on the arm; home and named poses are min-jerk joint-space moves like its safe_home; chunks
are IK per sample (warm-started along the chunk). Every plan is complete before anything moves and
is not planned (``PlanError``, with the reason, sample index and a retry) if it would break a joint's speed
limit, or (chunks) an IK sample fails or jumps branch.

A ``Trajectory`` is time-indexed joint samples (server monotonic time) with velocity feedforward
from finite differences; the node samples it every control tick (linear interpolation) and still
applies the step limit, so a plan can never move faster than the limit even if it were wrong.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import pinocchio as pin

from reBotArm_control_py.trajectory import IKParams as ClikIKParams
from reBotArm_control_py.trajectory import TrajPlanParams, TrajProfile, plan_cartesian_geodesic_trajectory, track_trajectory

from .kin import ArmKinematics, N, pose_error, pose_public

PLAN_DT = 0.02           # s between planned samples
BRANCH_JUMP = 0.5        # rad between consecutive samples = an IK branch change
MAX_SAMPLES = 3000       # 60 s at PLAN_DT
_ids = itertools.count(1)


class PlanError(ValueError):
    """A motion that could not be planned (nothing moves). Not a permission boundary: a property of
    this request (geometry, limits, timing). ``code`` is stable for branching on; ``retry`` (when the
    planner can compute one) is a patch to the same request body that plans; ``nearest`` (for poses)
    is the closest reachable one; ``hint`` says what else to try. Pickles (planner process)."""

    def __init__(self, msg: str, index: Optional[int] = None, *, code: str = "not_planned",
                 retry: Optional[dict] = None, nearest: Optional[dict] = None, hint: Optional[str] = None):
        super().__init__(msg if index is None else f"{msg} (sample {index})")
        self.index = index
        self.extra = {k: v for k, v in (("kind", "plan"), ("code", code), ("retryable", True), ("sample", index),
                                        ("retry", retry), ("nearest", nearest), ("hint", hint)) if v is not None}

    def __reduce__(self):   # keep the keyword fields across the process boundary
        return (_plan_error, (self.args[0], self.extra))


def _plan_error(msg: str, extra: dict) -> "PlanError":
    e = PlanError.__new__(PlanError)
    ValueError.__init__(e, msg)
    e.index, e.extra = extra.get("sample"), dict(extra)
    return e


@dataclass
class Trajectory:
    kind: str                       # servo | pose | home | chunk
    t: np.ndarray                   # (n,) server monotonic s
    q: np.ndarray                   # (n, 6)
    qd: np.ndarray                  # (n, 6)
    gap: Optional[np.ndarray] = None  # (n,) m, chunks only
    goal: Optional[pin.SE3] = None   # final TCP pose
    name: Optional[str] = None       # named pose
    id: int = field(default_factory=lambda: next(_ids))
    info: dict = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    def shift(self, dt: float) -> None:
        self.t = self.t + dt

    def sample(self, now: float):
        """(q, qd, gap or None, done) at ``now``: before the start, the first sample; after the
        end, the last sample with zero velocity."""
        t = self.t
        if now >= t[-1]:
            return self.q[-1], np.zeros(N), (None if self.gap is None else float(self.gap[-1])), True
        if now <= t[0]:
            return self.q[0], np.zeros(N), (None if self.gap is None else float(self.gap[0])), False
        i = int(np.searchsorted(t, now, side="right")) - 1
        a = (now - t[i]) / (t[i + 1] - t[i])
        q = self.q[i] * (1 - a) + self.q[i + 1] * a
        qd = self.qd[i] * (1 - a) + self.qd[i + 1] * a
        gap = None if self.gap is None else float(self.gap[i] * (1 - a) + self.gap[i + 1] * a)
        return q, qd, gap, False


def min_jerk(s: np.ndarray) -> np.ndarray:
    """10s³ − 15s⁴ + 6s⁵: the profile of rebot_ctl's safe_home (zero velocity and acceleration at
    both ends; peak speed 1.875× the mean)."""
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def _qd(t: np.ndarray, q: np.ndarray) -> np.ndarray:
    qd = np.zeros_like(q)
    if len(t) > 2:
        qd[1:-1] = (q[2:] - q[:-2]) / (t[2:, None] - t[:-2, None])
    return qd                                   # zero at both ends


def _check_speeds(t, q, vmax, start_index=0) -> tuple[float, int]:
    """Largest ratio of joint speed to its limit between consecutive samples, and where."""
    if len(t) < 2:
        return 0.0, 0
    v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
    r = (v / vmax).max(axis=1)
    i = int(np.argmax(r))
    return float(r[i]), i + start_index


def _branch_check(q: np.ndarray) -> None:
    if len(q) > 1:
        jump = np.abs(np.diff(q, axis=0)).max(axis=1)
        i = int(np.argmax(jump))
        if jump[i] > BRANCH_JUMP:
            raise PlanError(f"the straight path crosses a singularity or a joint limit (the joints jump "
                            f"{math.degrees(jump[i]):.0f}° between two samples)", i + 1, code="singular_path",
                            hint="go through an intermediate pose on either side, or use a joint-space move "
                                 "(a named pose with POST /servo {name}, or POST /home)")


def _ik_path(kin: ArmKinematics, poses: list, seed: np.ndarray) -> np.ndarray:
    out = np.zeros((len(poses), N))
    q = np.asarray(seed, float)
    for i, T in enumerate(poses):
        r = kin.ik(T, q)
        if not r.ok:
            raise PlanError(f"no IK solution for this sample: {r.reason}", i, code="sample_unreachable",
                            nearest=r.public())
        q = out[i] = r.q
    return out


# -- planners --------------------------------------------------------------------------------

# rebot_ctl's end-pose controller (RebotArmEndPose.move_to_traj), as tested on the arm
SERVO_DT = 0.01
CLIK = ClikIKParams(max_iter=200, tolerance=1e-4, damping=1e-6, step_size=0.8)
NULL_GAIN = 0.1
MJ_PEAK_VEL = 1.875          # min-jerk: peak speed = 1.875·D/T
MJ_PEAK_ACC = 5.7735         # min-jerk: peak acceleration = 5.7735·D/T²


def plan_servo(kin: ArmKinematics, q_start, goal: pin.SE3, *, lin_vel: float, lin_acc: float,
               ang_vel: float, ang_acc: float, joint_vel: np.ndarray,
               duration: Optional[float] = None, rot_tol: Optional[float] = None) -> Trajectory:
    """rebot_ctl's end-pose controller pipeline (RebotArmEndPose.move_to_traj): IK for the goal from
    ``q_start``; a min-jerk SE(3) geodesic from FK(q_start) to FK(IK solution); CLIK
    (``track_trajectory``, null-space joint-limit avoidance) into joint samples; finite-difference
    velocities, zero at the end. The node adds only: the duration from the TCP speed and
    acceleration limits (min-jerk peaks, linear and angular parts of the geodesic), or the
    requested one if it keeps within them; and no plan (PlanError) if a joint would exceed its speed limit
    (the step limit would otherwise distort the path). Times start at 0."""
    q_start = np.asarray(q_start, float)
    end = kin.ik(goal, q_start, rot_tol=rot_tol)
    if not end.ok:
        n = end.nearest
        raise PlanError(f"{end.reason}. nearest: pos {n['pos']} rpy {n['rpy']}", code="unreachable",
                        retry={"pose": {"pos": n["pos"], "quat": n["quat"]}}, nearest=end.public())
    T_start, T_end = kin.fk(q_start), kin.fk(end.q)
    requested = None
    if not end.exact:   # accepted within the IK tolerance: the move goes to (and is judged against) T_end
        requested = {"pose": pose_public(goal), "pos_err_mm": round(end.pos_err_mm, 2),
                     "rot_err_deg": round(end.rot_err_deg, 2)}
        goal = T_end
    xi = pin.log6(T_start.inverse() * T_end)
    d, th = float(np.linalg.norm(xi.linear)), float(np.linalg.norm(xi.angular))
    if d < 1e-5 and th < 1e-4:
        t = np.array([0.0, SERVO_DT])
        q = np.array([q_start, q_start])
        info = {"distance_m": d, "angle_rad": th}
        if requested:
            info["requested"] = requested
        return Trajectory("servo", t, q, np.zeros_like(q), goal=goal, info=info)
    T_min = max(MJ_PEAK_VEL * d / lin_vel, math.sqrt(MJ_PEAK_ACC * d / lin_acc),
                MJ_PEAK_VEL * th / ang_vel, math.sqrt(MJ_PEAK_ACC * th / ang_acc))
    if duration is not None:
        if duration < T_min - 1e-6:
            raise PlanError(f"duration {duration:g} s is shorter than the speed / acceleration limits allow "
                            f"for this move: {T_min:.2f} s at least", code="duration_too_short",
                            retry={"duration": round(math.ceil(T_min * 100) / 100, 2)})
        T = float(duration)
    else:
        T = T_min
    if T / SERVO_DT > MAX_SAMPLES:
        raise PlanError(f"the move would take {T:.1f} s, longer than one planned move can be "
                        f"({MAX_SAMPLES * SERVO_DT:.0f} s)", code="move_too_long",
                        hint="split it into shorter moves, or give a higher speed")
    cart = plan_cartesian_geodesic_trajectory(T_start, T_end, T, TrajPlanParams(dt=SERVO_DT, profile=TrajProfile.MIN_JERK))
    full = np.zeros(kin.model.nq)
    full[:N] = q_start
    pts = track_trajectory(kin.model, kin.tcp, cart.trajectory, full, CLIK, null_gain=NULL_GAIN, controlled_joints=N)
    if not pts:
        raise PlanError("the path tracker returned no samples", code="planner_empty",
                        hint="retry; if it repeats, use a goal slightly nearer the arm")
    t = np.array([p.time for p in pts])
    q = np.array([p.q[:N] for p in pts])
    # finite-difference velocity feedforward, zero at the end (as RebotArmEndPose, but over the
    # samples' actual spacing: it divided T by the number of points, not of intervals)
    qd = np.zeros_like(q)
    qd[:-1] = (q[1:] - q[:-1]) / np.diff(t)[:, None]
    ratio, idx = _check_speeds(t, q, joint_vel)
    if ratio > 1.0:
        j = int(np.argmax(np.abs(q[idx + 1] - q[idx]) / joint_vel)) + 1
        need = math.ceil(float(t[-1]) * ratio * 1.05 * 10) / 10
        retry = {"duration": need} if need <= 60 and need / SERVO_DT <= MAX_SAMPLES else None
        msg = (f"at this speed joint{j} would need {ratio:.2f}× its speed limit (near a singularity the joints "
               "move fast for small TCP motions); ")
        msg += (f"the same move with duration {need:g} s plans" if retry else
                "no single move is slow enough (it would take over 60 s): the path runs through a singularity")
        raise PlanError(msg, idx, code="joint_speed", retry=retry,
                        hint=None if retry else "route around the singularity: go through an intermediate pose to "
                                                "one side, or use a joint-space move (a named pose or POST /home)")
    info = {"distance_m": round(d, 5), "angle_rad": round(th, 5),
            "clik_unconverged": int(sum(not p.ik_success for p in pts))}
    if requested:
        info["requested"] = requested   # the goal was not exactly reachable: this is what was asked for
    return Trajectory("servo", t, q, qd, goal=goal, info=info)


def plan_joint(q_start, waypoints: list, vel: float, joint_vel: np.ndarray, kin: ArmKinematics,
               kind: str = "pose", name: Optional[str] = None) -> Trajectory:
    """Min-jerk joint-space segments through ``waypoints`` (each within limits), the fastest joint
    of each segment at no more than ``vel`` (and every joint within ``joint_vel``)."""
    pts = [np.asarray(q_start, float)] + [np.asarray(w, float) for w in waypoints]
    for i, w in enumerate(pts[1:], 1):
        if not kin.within_limits(w):
            raise PlanError(f"waypoint {i} is outside the joint limits", code="waypoint_outside_limits")
    ts, qs, t0 = [], [], 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        dq = b - a
        peak = 1.875                                    # min-jerk peak speed / mean speed
        T = max(float(np.max(np.abs(dq) * peak / np.minimum(vel, joint_vel))), PLAN_DT)
        n = max(2, int(math.ceil(T / PLAN_DT)) + 1)
        tt = np.linspace(0.0, T, n)
        seg = a + np.outer(min_jerk(tt / T), dq)
        if ts:
            tt, seg = tt[1:], seg[1:]
        ts.append(t0 + tt)
        qs.append(seg)
        t0 += T
    t, q = np.concatenate(ts), np.concatenate(qs)
    if len(t) > MAX_SAMPLES:
        raise PlanError(f"the move would take {t[-1]:.1f} s, longer than one planned move can be",
                        code="move_too_long", hint="go through fewer or nearer waypoints")
    return Trajectory(kind, t, q, _qd(t, q), goal=kin.fk(q[-1]), name=name)


def plan_chunk(kin: ArmKinematics, q_seed, poses: list, gaps: Optional[list], *, t0: float, dt: float,
               now: float, blend_s: float, joint_vel: np.ndarray, q_now) -> Trajectory:
    """An action chunk: ``poses[i]`` is due at ``t0 + i*dt`` (server monotonic). Samples already
    past are skipped; the move starts from ``q_now`` at ``now`` and joins the chunk at its first
    sample at or after ``now + blend_s``."""
    if dt <= 0 or not math.isfinite(dt):
        raise PlanError("dt must be positive", code="invalid_input")
    times = t0 + dt * np.arange(len(poses))
    keep = np.flatnonzero(times >= now + blend_s)
    if len(keep) == 0:
        raise PlanError(f"the chunk is already over: its last sample was due {now - times[-1]:.3f} s ago (or "
                        "within the blend)", code="chunk_stale", hint="send it again with a later t0 (GET /time)")
    first = int(keep[0])
    q = _ik_path(kin, [poses[i] for i in keep], q_seed)
    _branch_check(np.vstack([np.asarray(q_now, float)[None], q]))
    lead = np.abs(q[0] - np.asarray(q_now, float)) / max(times[first] - now, 1e-6)
    if np.any(lead > joint_vel):
        need = math.ceil(float((np.abs(q[0] - np.asarray(q_now, float)) / joint_vel).max()) * 1.1 * 100) / 100
        raise PlanError(f"the first sample is too far from the arm to reach within the blend "
                        f"({(lead / joint_vel).max():.2f}× the joint speed limit)", first, code="chunk_blend",
                        retry={"blend_s": need} if need <= 1.0 else None,
                        hint=None if need <= 1.0 else "start the chunk nearer the arm (servo there first)")
    ratio, idx = _check_speeds(times[keep], q, joint_vel, first)
    if ratio > 1.0:
        ndt = math.ceil(dt * ratio * 1.05 * 1000) / 1000
        raise PlanError(f"the chunk moves the joints {ratio:.2f}× faster than their speed limits", idx,
                        code="joint_speed", retry={"dt": ndt} if ndt <= 1.0 else None,
                        hint="spread the poses further apart in time (larger dt)")
    t = np.concatenate([[now], times[keep]])
    qq = np.vstack([np.asarray(q_now, float)[None], q])
    gap = None
    if gaps is not None:
        g = np.asarray(gaps, float)
        gap = np.concatenate([[g[first]], g[keep]])
    qd = _qd(t, qq)
    qd[0] = 0.0
    return Trajectory("chunk", t, qq, qd, gap=gap, goal=poses[-1],
                      info={"skipped": first, "n": len(poses), "t0": t0, "dt": dt, "blend_s": blend_s})
