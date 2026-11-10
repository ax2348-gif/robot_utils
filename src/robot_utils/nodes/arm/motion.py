"""Planned motions for the arm node: servo (a Cartesian move to a pose), joint-space moves to named
poses (home), and action chunks. Pure computation; the node executes the result.

Every plan is complete and checked before anything moves: IK at every sample (warm-started along
the path), joint limits, joint speeds against the step limit, and no IK branch jumps. A plan that
fails anywhere raises ``PlanError`` with the reason and the sample index.

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

from .kin import ArmKinematics, N, pose_error

PLAN_DT = 0.02           # s between planned samples
BRANCH_JUMP = 0.5        # rad between consecutive samples = an IK branch change
MAX_SAMPLES = 3000       # 60 s at PLAN_DT
_ids = itertools.count(1)


class PlanError(ValueError):
    def __init__(self, msg: str, index: Optional[int] = None):
        super().__init__(msg if index is None else f"{msg} (sample {index})")
        self.index = index


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


# -- time scaling ------------------------------------------------------------------------------

def trapezoid(v: float, a: float) -> float:
    """Duration of a 0→1 move with peak speed v and acceleration a (normalized units)."""
    if v * v / a >= 1.0:            # never reaches v: triangle
        return 2.0 * math.sqrt(1.0 / a)
    return 1.0 / v + v / a


def trapezoid_s(t: np.ndarray, T: float, v: float, a: float) -> np.ndarray:
    """Normalized progress s(t) in [0, 1] of a symmetric trapezoid (or triangle) of duration T."""
    if v * v / a >= 1.0:
        v = math.sqrt(a)             # triangle peak
    ta = v / a
    s = np.where(t < ta, 0.5 * a * t ** 2,
                 np.where(t < T - ta, 0.5 * a * ta ** 2 + v * (t - ta),
                          1.0 - 0.5 * a * np.maximum(T - t, 0.0) ** 2))
    return np.clip(s, 0.0, 1.0)


def min_jerk(s: np.ndarray) -> np.ndarray:
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
            raise PlanError(f"IK changes branch mid-path ({math.degrees(jump[i]):.0f}° in one step): "
                            "the straight-line path passes a singularity or a joint limit", i + 1)


def _ik_path(kin: ArmKinematics, poses: list, seed: np.ndarray) -> np.ndarray:
    out = np.zeros((len(poses), N))
    q = np.asarray(seed, float)
    for i, T in enumerate(poses):
        r = kin.ik(T, q)
        if not r.ok:
            raise PlanError(f"IK failed: {r.reason}", i)
        q = out[i] = r.q
    return out


# -- planners --------------------------------------------------------------------------------

def plan_servo(kin: ArmKinematics, q_start, goal: pin.SE3, *, lin_vel: float, lin_acc: float,
               ang_vel: float, ang_acc: float, joint_vel: np.ndarray,
               duration: Optional[float] = None) -> Trajectory:
    """A straight TCP line (position linear, orientation slerp) from FK(q_start) to ``goal``, timed
    by a trapezoid within the linear and angular limits (or exactly ``duration``, refused if that
    breaks them), then slowed further if a joint would exceed its speed limit. Times start at 0."""
    q_start = np.asarray(q_start, float)
    end = kin.ik(goal, q_start)
    if not end.ok:
        raise PlanError(f"goal unreachable: {end.reason}")
    start = kin.fk(q_start)
    d, th = pose_error(start, goal)
    dlog = pin.log3(start.rotation.T @ goal.rotation)
    if d < 1e-5 and th < 1e-4:
        t = np.array([0.0, PLAN_DT])
        q = np.array([q_start, q_start])
        return Trajectory("servo", t, q, np.zeros_like(q), goal=goal, info={"distance_m": d, "angle_rad": th})
    # normalized limits: s'' and s' bounded by every component's limit
    v_s = min(lin_vel / d if d > 1e-9 else math.inf, ang_vel / th if th > 1e-9 else math.inf)
    a_s = min(lin_acc / d if d > 1e-9 else math.inf, ang_acc / th if th > 1e-9 else math.inf)
    T_min = trapezoid(v_s, a_s)
    if duration is not None:
        if duration < T_min - 1e-6:
            raise PlanError(f"duration {duration:g} s is too short for the speed limits (at least {T_min:.2f} s)")
        T = float(duration)
        v_s, a_s = 1.0 / (0.75 * T), 1.0 / (0.75 * T) / (0.25 * T)   # 25 % accel, 50 % cruise, 25 % decel
    else:
        T = T_min
    for attempt in range(5):
        n = max(2, int(math.ceil(T / PLAN_DT)) + 1)
        if n > MAX_SAMPLES:
            raise PlanError(f"move too long ({T:.1f} s)")
        t = np.linspace(0.0, T, n)
        s = trapezoid_s(t, T, v_s, a_s)
        poses = [pin.SE3(start.rotation @ pin.exp3(si * dlog), start.translation + si * (goal.translation - start.translation))
                 for si in s]
        q = _ik_path(kin, poses, q_start)
        _branch_check(q)
        ratio, idx = _check_speeds(t, q, joint_vel)
        if ratio <= 1.0:
            return Trajectory("servo", t, q, _qd(t, q), goal=goal,
                              info={"distance_m": round(d, 5), "angle_rad": round(th, 5), "slowed": attempt > 0})
        if duration is not None:
            j = int(np.argmax(np.abs(q[idx + 1] - q[idx]) / joint_vel)) + 1
            raise PlanError(f"joint{j} would exceed its speed limit ({ratio:.2f}× at this duration)", idx)
        # slow down: stretch time (the trapezoid keeps its shape)
        T *= ratio * 1.05
        v_s, a_s = v_s / (ratio * 1.05), a_s / (ratio * 1.05) ** 2
    raise PlanError("could not keep the joints within their speed limits")


def plan_joint(q_start, waypoints: list, vel: float, joint_vel: np.ndarray, kin: ArmKinematics,
               kind: str = "pose", name: Optional[str] = None) -> Trajectory:
    """Min-jerk joint-space segments through ``waypoints`` (each within limits), the fastest joint
    of each segment at no more than ``vel`` (and every joint within ``joint_vel``)."""
    pts = [np.asarray(q_start, float)] + [np.asarray(w, float) for w in waypoints]
    for i, w in enumerate(pts[1:], 1):
        if not kin.within_limits(w):
            raise PlanError(f"waypoint {i} is outside the joint limits")
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
        raise PlanError(f"move too long ({t[-1]:.1f} s)")
    return Trajectory(kind, t, q, _qd(t, q), goal=kin.fk(q[-1]), name=name)


def plan_chunk(kin: ArmKinematics, q_seed, poses: list, gaps: Optional[list], *, t0: float, dt: float,
               now: float, blend_s: float, joint_vel: np.ndarray, q_now) -> Trajectory:
    """An action chunk: ``poses[i]`` is due at ``t0 + i*dt`` (server monotonic). Samples already
    past are skipped; the move starts from ``q_now`` at ``now`` and joins the chunk at its first
    sample at or after ``now + blend_s``."""
    if dt <= 0 or not math.isfinite(dt):
        raise PlanError("dt must be positive")
    times = t0 + dt * np.arange(len(poses))
    keep = np.flatnonzero(times >= now + blend_s)
    if len(keep) == 0:
        raise PlanError(f"chunk is stale: its last sample was due {now - times[-1]:.3f} s ago (or within the blend)")
    first = int(keep[0])
    q = _ik_path(kin, [poses[i] for i in keep], q_seed)
    _branch_check(q)
    ratio, idx = _check_speeds(times[keep], q, joint_vel, first)
    if ratio > 1.0:
        raise PlanError(f"chunk is faster than the joint speed limits ({ratio:.2f}×)", idx)
    t = np.concatenate([[now], times[keep]])
    qq = np.vstack([np.asarray(q_now, float)[None], q])
    gap = None
    if gaps is not None:
        g = np.asarray(gaps, float)
        gap = np.concatenate([[g[first]], g[keep]])
    qd = _qd(t, qq)
    qd[0] = 0.0
    return Trajectory("chunk", t, qq, qd, gap=gap, goal=poses[-1],
                      info={"skipped": first, "n": len(poses), "t0": t0, "dt": dt})
