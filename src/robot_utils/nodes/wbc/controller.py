"""The controller interface (the swap point: QP now, MPC later) and the QP controller (pinocchio + daqp).

Each tick the node hands the controller a ``Snapshot`` (the base pose used, the previous command, per end
effector the commanded joints, the target of this tick and its hints) and gets an ``Output`` (the base twist and
the arm joint steps of this tick, the solve status, what limited the motion, diagnostics; optionally a planned
trajectory). Model, target buffers, blending, hints, safety and I/O stay in the node.

**QP (v1).** Decision vector: the velocity ``x`` of ``[x, y, yaw (world), arm joints]`` (the step is ``x dt``).

Costs: per EE pose tracking ``|W (J x - (v_ff + K e))|²`` (separate position / orientation weights, the
orientation rows in the TCP's own axes so a mask can drop one, Levenberg–Marquardt damping ``lm |W e|²`` plus
adaptive damping on the arm growing as its manipulability falls); base regularisation scheduled by the EE's
distance to its target (geometric from ``w_base_near`` to ``w_base_far``, times ``arm_vs_base``); arm
regularisation; the arm posture; the arm-only manipulability gradient (of ``log w``); base heading and base pose
hints (mode ``hint``, no cost inside ``tol``).

Constraints (hard; the costs above, base hints included, yield to them):

- arm joints: a box per joint from the joint speed cap, the acceleration limit around the previous velocity, a
  position damper (a step covers at most ``limit_gain`` of the distance to a limit) and the braking distance
  ``sqrt(2 a d)``; when braking cannot be met the acceleration limit wins (always feasible). pink's
  ``ConfigurationLimit`` + ``AccelerationLimit`` were tried first and went infeasible approaching a limit
  (their braking term is not consistent with the discrete step), hence this box.
- **reach**: the distance from the arm's shoulder (the first joint's axis point) to each EE *target* stays within
  ``reach_fraction`` of the nominal reach: a velocity damper ``d' <= xi (d_max - d) / (d_max - d_i)`` beyond
  ``d_i``, linear in the base velocity (only the base moves the shoulder), so the base may move along the
  boundary but never across it.
- **manipulability floor**: the arm-only manipulability ``w``: ``grad w . q' >= -xi w_i (w - w_min) /
  (w_i - w_min)`` below ``w_i`` (bounded by what the joint box allows, so always feasible).
- base speed (inscribed octagon), yaw rate, base twist rate (octagon + yaw), every wheel's speed (inscribed
  octagon, 8 rows each: the wheel velocity is linear in the twist). Modes ``locked`` / ``commanded`` and holds
  fix the base velocity (an equality, computed within the rate and wheel limits).

Deadband with hysteresis: a first solve without the twist-rate rows gives the base's *desired* twist; a stopped
base starts only above ``deadband_*_on``, a moving one brakes below ``deadband_*_off`` (then the base velocity is
fixed to the braking step and the arm takes the motion). If the desired solution breaks a rate row, a second
solve with the rate rows gives the command. An infeasible or failed solve returns ``status`` infeasible / error
with zero velocity: the node holds and raises a fault.

``Output.limited_by``: which hard constraint bound the solution (reach, manipulability, joint_limit,
base_limit; none), for operator feedback.
"""
from __future__ import annotations

import abc
import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import pinocchio as pin
import qpsolvers

from .model import WbcModel
from .spec import Config, EEHint, Hints, effective_limits
from .targets import Target

C8 = math.cos(math.pi / 8)
OCT = np.array([[math.cos(k * math.pi / 4), math.sin(k * math.pi / 4)] for k in range(8)])   # 8 unit normals
LIMITED = ["none", "reach", "manipulability", "fence", "joint_limit", "base_limit"]
BIND_TOL = 1e-6          # velocity units: a row this close to its bound is binding
HINT_BAND = 1e-5         # weighted task velocity units: how far stage 2 (the hints) may move the EE tasks


def oct_norm(u) -> float:
    """The norm whose unit ball is the inscribed octagon (>= the Euclidean norm, at most 1/cos(pi/8) of it)."""
    return float(np.max(OCT @ np.asarray(u, float)[:2]) / C8)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def shrink(e, tol: float):
    """e moved toward 0 by tol (0 inside it): a dead zone without a step."""
    if np.ndim(e) == 0:
        return math.copysign(max(0.0, abs(e) - tol), e)
    n = float(np.linalg.norm(e))
    return e * (max(0.0, n - tol) / n) if n > 0 else e


def clip_norm(e: np.ndarray, m: float) -> np.ndarray:
    n = float(np.linalg.norm(e))
    return e * (m / n) if n > m else e


@dataclass
class EEState:
    q_cmd: np.ndarray                 # the arm's commanded joints (integration state)
    v_prev: np.ndarray                # the previous tick's commanded joint velocity
    target: Optional[Target]
    hint: EEHint
    posture: np.ndarray               # the posture the null space prefers (rad)


@dataclass
class Snapshot:
    t: float
    base: tuple                       # (x, y, yaw) world, the pose used this tick
    base_prev: np.ndarray             # previous commanded body twist (vx, vy, w)
    ees: dict                         # name -> EEState
    hints: Hints
    hold: bool = False                # stale input / no client: base brakes, arm stops
    base_available: bool = True       # False: no base (treated as locked)
    velocity_age: float = math.inf    # s since the commanded base velocity was given (mode commanded)


@dataclass
class Output:
    status: str                       # ok | infeasible | error
    base_twist: np.ndarray            # body (vx, vy, w)
    dq: dict                          # name -> arm joint step this tick (rad)
    qd: dict                          # name -> arm joint velocity (rad/s)
    solve_ms: float = 0.0
    reason: Optional[str] = None
    deadband: bool = False
    base_fixed: Optional[str] = None  # why the base velocity was fixed (locked, commanded, hold, deadband, ...)
    limited_by: str = "none"          # LIMITED
    diag: dict = field(default_factory=dict)
    plan: Optional[dict] = None       # a planned trajectory (an MPC's); None for the QP


class Controller(abc.ABC):
    """The swap point. ``step`` must not block (it runs in the 100 Hz loop) and must not keep references to
    the snapshot's arrays."""

    name = "controller"

    @abc.abstractmethod
    def step(self, snap: Snapshot) -> Output:
        ...

    def reset(self) -> None:
        """Forget internal state (deadband, warm starts): called on enable and after a fault."""


class QPController(Controller):
    name = "qp"

    def __init__(self, model: WbcModel, cfg: Config, wheels, solver: str = "daqp"):
        self.m, self.cfg = model, cfg
        self.wheels = np.asarray(wheels, float).reshape(-1, 2)
        self.solver = solver
        self.bv = model.base_v
        self.reach = {k: (cfg.reach or r) for k, r in model.reach.items()}
        self.active = False                                   # base deadband state
        self.solves = 0

    def reset(self) -> None:
        self.active = False

    # -- base helpers ----------------------------------------------------------------------------------

    def _wheel_max(self, tw) -> float:
        vx, vy, w = tw
        return max(math.hypot(vx - w * y, vy + w * x) for x, y in self.wheels)

    def _limit_twist(self, tw, speed, omega) -> np.ndarray:
        """Scaled (direction kept) into the speed octagon, the yaw rate and the wheel octagons."""
        tw = np.array(tw, float)
        n = oct_norm(tw[:2])
        if n > speed:
            tw[:2] *= speed / n
        if abs(tw[2]) > omega:
            tw[2] = math.copysign(omega, tw[2])
        wm = max(oct_norm((tw[0] - tw[2] * y, tw[1] + tw[2] * x)) for x, y in self.wheels)
        if wm > self.cfg.wheel_speed:
            tw *= self.cfg.wheel_speed / wm
        return tw

    def _step_toward(self, prev, goal, accel, dt) -> np.ndarray:
        """From prev toward goal within the twist-rate limits."""
        d = np.asarray(goal, float) - np.asarray(prev, float)
        n = oct_norm(d[:2])
        if n > accel * dt:
            d[:2] *= accel * dt / n
        d[2] = float(np.clip(d[2], -self.cfg.max_alpha * dt, self.cfg.max_alpha * dt))
        return np.asarray(prev, float) + d

    def _fixed_twist(self, snap: Snapshot, dt: float) -> tuple[Optional[np.ndarray], Optional[str]]:
        """The base twist when the mode (or a hold) fixes it, else (None, None)."""
        h = snap.hints
        speed, omega, accel = effective_limits(h, self.cfg)
        prev = snap.base_prev
        if snap.hold:
            return self._step_toward(prev, np.zeros(3), accel, dt), "hold"
        if not snap.base_available or h.mode == "locked":
            return self._step_toward(prev, np.zeros(3), accel, dt), "locked" if snap.base_available else "no base"
        if h.mode == "commanded":
            goal = np.zeros(3)
            why = "commanded (nothing given)"
            x, y, yaw = snap.base
            if h.velocity is not None and snap.velocity_age <= h.velocity.ttl:
                v = h.velocity
                goal = np.array([v.vx, v.vy, v.omega])
                if v.frame == "world":
                    c, s = math.cos(yaw), math.sin(yaw)
                    goal[:2] = [c * v.vx + s * v.vy, -s * v.vx + c * v.vy]
                why = "commanded velocity"
            elif h.velocity is not None:
                why = "commanded velocity (ttl expired)"
            elif h.pose is not None:
                p = h.pose
                ex, ey = p.x - x, p.y - y
                c, s = math.cos(yaw), math.sin(yaw)
                k = self.cfg.k_base_pose
                goal = np.array([k * (c * ex + s * ey), k * (-s * ex + c * ey), k * wrap(p.yaw - yaw)])
                why = "commanded pose"
            goal = self._limit_twist(goal, speed, omega)
            return self._limit_twist(self._step_toward(prev, goal, accel, dt), speed, omega), why
        return None, None

    def _joint_box(self, ee: str, q: np.ndarray, v_prev: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(L, U, position-bound mask) velocity bounds of an arm's joints this tick (always L <= U)."""
        cfg, dt = self.cfg, self.cfg.dt
        n = len(q)
        lo, hi = self.m.arm_limits(ee)
        vm = np.asarray(cfg.joint_vel[:n], float)
        a = np.asarray(cfg.joint_acc[:n], float)
        dh, dl = hi - q, q - lo
        # within the limits: the damper and the braking distance; beyond one: back toward it (same law)
        U_pos = np.sign(dh) * np.minimum(cfg.limit_gain * np.abs(dh) / dt, np.sqrt(2 * a * np.abs(dh)))
        L_pos = -np.sign(dl) * np.minimum(cfg.limit_gain * np.abs(dl) / dt, np.sqrt(2 * a * np.abs(dl)))
        U = np.minimum(np.minimum(vm, v_prev + a * dt), U_pos)
        L = np.maximum(np.maximum(-vm, v_prev - a * dt), L_pos)
        # infeasible (moving too fast toward a limit, or beyond a cap): the acceleration limit wins
        U = np.maximum(U, np.minimum(v_prev - a * dt, vm))
        L = np.minimum(L, np.maximum(v_prev + a * dt, -vm))
        U = np.maximum(U, L)
        pos = np.concatenate([U_pos <= np.minimum(vm, v_prev + a * dt), L_pos >= np.maximum(-vm, v_prev - a * dt)])
        # without the acceleration limit (the deadband's "desired" solve)
        Ur, Lr = np.minimum(vm, U_pos), np.maximum(-vm, L_pos)
        return L, U, pos, Lr, np.maximum(Ur, Lr)

    # -- the QP ----------------------------------------------------------------------------------------

    def step(self, snap: Snapshot) -> Output:
        t0 = time.perf_counter()
        try:
            out = self._step(snap)
        except Exception as e:  # noqa: BLE001 - a failed solve is a fault, never an exception in the loop
            out = self._zero(snap, "error", f"{type(e).__name__}: {e}")
        out.solve_ms = (time.perf_counter() - t0) * 1e3
        return out

    def _zero(self, snap: Snapshot, status: str, reason: str) -> Output:
        self.active = False
        z = {k: np.zeros(len(e.q_cmd)) for k, e in snap.ees.items()}
        return Output(status, np.zeros(3), z, {k: v.copy() for k, v in z.items()}, reason=reason)

    def _step(self, snap: Snapshot) -> Output:
        cfg, m = self.cfg, self.m
        dt = cfg.dt
        nv = m.nv
        x, y, yaw = snap.base
        q = m.config(snap.base, {k: e.q_cmd for k, e in snap.ees.items()})
        if not np.all(np.isfinite(q)):
            raise ValueError("non-finite configuration")
        hold = snap.hold or all(e.target is None for e in snap.ees.values())
        h = snap.hints
        speed, omega, accel = effective_limits(h, cfg)
        prev = np.asarray(snap.base_prev, float)
        cy, sy = math.cos(yaw), math.sin(yaw)
        bx, by, bw = self.bv
        x_prev = np.zeros(nv)                                      # previous velocity (world base, arm joints)
        x_prev[[bx, by, bw]] = [cy * prev[0] - sy * prev[1], sy * prev[0] + cy * prev[1], prev[2]]
        H = cfg.damping * np.eye(nv)
        c = np.zeros(nv)
        diag: dict = {"err_pos": {}, "err_rot": {}, "manip": {}, "reach": {}}
        groups: list = []          # (kind, G, h): G x <= h, x the velocity
        dmax = 0.0
        manip_rows, reach_rows, task_rows = [], [], []
        for name, e in snap.ees.items():
            iv = m.arm_v[name]
            x_prev[iv] = e.v_prev
            L, U, posmask, Lr, Ur = self._joint_box(name, e.q_cmd, e.v_prev)
            S = np.zeros((len(iv), nv))
            S[np.arange(len(iv)), iv] = 1.0
            groups.append(("joint_limit_box", np.vstack([S, -S]), np.concatenate([U, -L]), posmask,
                           np.concatenate([Ur, -Lr])))
            mu, g = m.manipulability_gradient(q, name)
            diag["manip"][name] = mu
            m.update(q)
            J = m.jacobian(q, name)
            if hold or e.target is None:
                H[np.ix_(iv, iv)] += np.eye(len(iv))              # stop the arm (as fast as its limits allow)
                continue
            tg = e.target
            if not (np.all(np.isfinite(tg.p)) and np.all(np.isfinite(tg.R)) and np.all(np.isfinite(tg.v))):
                raise ValueError(f"{name}: non-finite target")
            T = m.data.oMf[m.tcp[name]]
            p_cur, R_cur = T.translation, T.rotation
            ep = tg.p - p_cur
            er = pin.log3(tg.R @ R_cur.T)
            diag["err_pos"][name], diag["err_rot"][name] = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
            dmax = max(dmax, float(np.linalg.norm(ep)))
            ep, er = clip_norm(ep, cfg.max_err_pos), clip_norm(er, cfg.max_err_rot)
            vd = tg.v + np.concatenate([cfg.k_pos * ep, cfg.k_rot * er])
            Rt = R_cur.T                                           # rotation rows in the TCP's own axes
            Jw = np.vstack([J[:3], Rt @ J[3:]])
            vdw = np.concatenate([vd[:3], Rt @ vd[3:]])
            hint = e.hint
            w = np.sqrt(np.array([hint.position] * 3 + [hint.orientation * mk for mk in hint.orientation_mask]))
            A = w[:, None] * Jw
            b = w * vdw
            err_w = w * np.concatenate([ep, Rt @ er])
            task_rows.append(A)
            H += A.T @ A + cfg.lm_damping * float(err_w @ err_w) * np.eye(nv)
            c -= A.T @ b
            # arm regularisation (+ adaptive damping near singularities), posture, manipulability gradient
            lam = cfg.lm_singular * (1.0 - mu / cfg.w_influence) ** 2 if mu < cfg.w_influence else 0.0
            diag.setdefault("lm_adaptive", {})[name] = lam
            # the posture and manipulability gradient only break ties while the task moves: scaled by the desired
            # EE speed, so a satisfied task (or a hold) never makes the arm move on its own (no self-motion)
            tie = min(1.0, (float(np.linalg.norm(vd[:3])) + 0.1 * float(np.linalg.norm(vd[3:]))) / cfg.tie_speed)
            diag.setdefault("tie", {})[name] = tie
            H[np.ix_(iv, iv)] += (cfg.w_arm + hint.posture_weight + lam) * np.eye(len(iv))
            c[iv] -= tie * hint.posture_weight * cfg.k_posture * (e.posture - e.q_cmd)
            if cfg.w_manip > 0 and mu > 1e-12:
                c[iv] -= tie * cfg.w_manip * (g / mu)
            # manipulability floor: grad w . q' >= -eta (w - w_min) / (w_i - w_min)
            if mu < cfg.w_influence:
                rhs = cfg.xi_manip * cfg.w_influence * (mu - cfg.w_min) / (cfg.w_influence - cfg.w_min)
                # a limit only stops motion, never starts it: standing still is always allowed (rhs >= 0), and
                # so is what the joint box forces (braking)
                amax = float(np.sum(np.maximum(g * L, g * U)))        # the most the joint box allows
                rhs = max(rhs, 0.0, -1.1 * amax)
                G = np.zeros((1, nv))
                G[0, iv] = -g
                manip_rows.append((G, np.array([rhs])))
            # reach: shoulder to target within reach_fraction of the nominal reach (rows on the base only)
            s, Js = m.shoulder_point(q, name)
            d_vec = tg.p - s
            d = float(np.linalg.norm(d_vec))
            r = self.reach[name]
            d_max, d_i = cfg.reach_fraction * r, cfg.reach_influence * r
            diag["reach"][name] = d / r if r > 0 else math.nan
            if d > 0.5 * r and d > 1e-6:
                u = d_vec / d
                G = -(u @ Js).reshape(1, nv)                          # the shoulder's part of d'
                gh = math.hypot(u[0], u[1])
                # the damper beyond d_i, and everywhere the base's braking distance to d_max
                hr = math.sqrt(2 * 0.8 * accel * max(gh, 0.1) * max(d_max - d, 0.0))
                if d > d_i:
                    hr = min(hr, cfg.xi_reach * (d_max - d) / (d_max - d_i))
                hr -= float(u @ tg.v[:3])
                f_prev = float((G @ x_prev)[0])
                floor = max(f_prev - 0.8 * gh * accel * dt, 0.0)     # stopping (within the rate limit) is allowed
                reach_rows.append((G, np.array([max(hr, floor)])))
        # base cost, scheduled by the distance to the target
        if dmax <= cfg.near_dist:
            wb = cfg.w_base_near
        elif dmax >= cfg.far_dist:
            wb = cfg.w_base_far
        else:
            a = (dmax - cfg.near_dist) / (cfg.far_dist - cfg.near_dist)
            wb = cfg.w_base_near * (cfg.w_base_far / cfg.w_base_near) ** a
        wb *= h.arm_vs_base
        diag["w_base"] = wb
        H[bx, bx] += wb
        H[by, by] += wb
        H[bw, bw] += wb * cfg.base_yaw_radius ** 2
        # base hints: a second, lower-priority stage (see solve_lex): they use only the EE tasks' freedom
        Hh, ch = np.zeros((nv, nv)), np.zeros(nv)
        if h.mode == "hint" and not hold:
            H0, c0 = H, c
            H, c = Hh, ch
            if h.target_pose is not None:
                tp = h.target_pose
                exy = shrink(np.array([tp.x - x, tp.y - y]), tp.tol)
                H[bx, bx] += tp.weight
                H[by, by] += tp.weight
                c[[bx, by]] -= tp.weight * cfg.k_base_pose * exy
            if h.target_heading is not None:
                hh = h.target_heading
                yt = hh.yaw
                if hh.face is not None:
                    tgt = snap.ees[hh.face].target if hh.face in snap.ees else None
                    yt = None if tgt is None else math.atan2(tgt.p[1] - y, tgt.p[0] - x) - m.mount_yaw.get(hh.face, 0.0)
                elif hh.point is not None:
                    yt = math.atan2(hh.point[1] - y, hh.point[0] - x) - next(iter(m.mount_yaw.values()), 0.0)
                if yt is not None:
                    eh = shrink(wrap(yt - yaw), hh.tol)
                    H[bw, bw] += hh.weight
                    c[bw] -= hh.weight * cfg.k_heading * eh
            if h.target_heading is None and h.target_pose is not None:
                H[bw, bw] += cfg.w_hint_yaw          # a position hint moves the base, it does not turn it
            H, c = H0, c0
        has_hint = bool(np.any(Hh) or np.any(ch))
        A_task = np.vstack(task_rows) if task_rows else None
        # base rows (velocity units)
        Bm = np.zeros((2, nv))                     # body linear velocity = Bm x
        Bm[0, bx], Bm[0, by], Bm[1, bx], Bm[1, by] = cy, sy, -sy, cy
        Wy = np.zeros(nv)
        Wy[bw] = 1.0
        base_G = [OCT @ Bm] + [OCT @ (Bm + np.outer([-wy, wx], Wy)) for wx, wy in self.wheels] + [np.vstack([Wy, -Wy])]
        base_h = [np.full(8, speed * C8)] + [np.full(8, cfg.wheel_speed * C8)] * len(self.wheels) + \
                 [np.array([omega, omega])]
        rate_G = np.vstack([OCT @ Bm, Wy, -Wy])
        rate_h = np.concatenate([accel * dt * C8 + OCT @ prev[:2],
                                 [cfg.max_alpha * dt + prev[2], cfg.max_alpha * dt - prev[2]]])

        def solve_one(base_fixed: Optional[np.ndarray], with_rate: bool, band=None, relax=False):
            G = [g_[1] for g_ in groups] + [r_[0] for r_ in manip_rows]
            hh = [g_[4 if relax else 2] for g_ in groups] + [r_[1] for r_ in manip_rows]
            Hq, cq = H, c
            if band is not None:                       # stage 2: the hints, the EE tasks' velocities kept
                Hq, cq = H + Hh, c + ch
                G += [A_task, -A_task]
                hh += [band + HINT_BAND, -band + HINT_BAND]
            A = b = None
            if base_fixed is None:
                G += base_G + [r_[0] for r_ in reach_rows]
                hh += base_h + [r_[1] for r_ in reach_rows]
                if with_rate:
                    G.append(rate_G)
                    hh.append(rate_h)
            else:
                A = np.zeros((3, nv))
                A[0, bx] = A[1, by] = A[2, bw] = 1.0
                vb = base_fixed
                b = np.array([cy * vb[0] - sy * vb[1], sy * vb[0] + cy * vb[1], vb[2]])
            prob = qpsolvers.Problem(Hq, cq, np.vstack(G), np.concatenate(hh), A, b)
            sol = qpsolvers.solve_problem(prob, solver=self.solver)
            self.solves += 1
            if not sol.found or sol.x is None or not np.all(np.isfinite(sol.x)):
                return None
            return np.asarray(sol.x)

        def solve(base_fixed: Optional[np.ndarray], with_rate: bool, relax: bool = False):
            """Lexicographic: the EE tasks (with every constraint) first; then, if base hints are given and the
            base is free, the hints within what keeps the EE tasks' velocities (so a hint never costs tracking:
            it yields at the arm's limits). Stage 2 failing keeps stage 1."""
            x1 = solve_one(base_fixed, with_rate, relax=relax)
            if x1 is None or not has_hint or base_fixed is not None:
                return x1
            if A_task is None:
                return x1
            x2 = solve_one(None, with_rate, band=A_task @ x1, relax=relax)
            return x1 if x2 is None else x2

        def twist_of(xs):
            return np.array([cy * xs[bx] + sy * xs[by], -sy * xs[bx] + cy * xs[by], xs[bw]])

        def limited_of(xs, base_free: bool) -> str:
            """What bound the solution (the first binding kind, by priority)."""
            if base_free and any(float((G @ xs)[0]) >= float(hr[0]) - BIND_TOL for G, hr in reach_rows):
                return "reach"
            if any(float((G @ xs)[0]) >= float(hr[0]) - BIND_TOL for G, hr in manip_rows):
                return "manipulability"
            if any(np.any((G @ xs >= hr - BIND_TOL) & pm) for _, G, hr, pm, _r in groups):
                return "joint_limit"
            if base_free and any(np.any(G @ xs >= hr - BIND_TOL) for G, hr in zip(base_G, base_h)):
                return "base_limit"
            return "none"

        fixed, why = self._fixed_twist(snap, dt)
        deadband = False
        limited_free = "none"
        n_solves = 1
        if fixed is not None:
            self.active = bool(np.any(np.abs(fixed) > 1e-9))
            xs = solve(fixed, False)
        else:
            # the base's desired twist, without the acceleration limits (from rest the arm could not yet
            # compensate the base this tick), decides the deadband
            xd = solve(None, False, relax=True)
            if xd is None:
                xd = solve(None, False)
            n_solves += 1
            xs = xd
            if xd is not None:
                tw = twist_of(xd)
                lin, ang = math.hypot(tw[0], tw[1]), abs(tw[2])
                if self.active:
                    on = lin >= cfg.deadband_lin_off or ang >= cfg.deadband_w_off
                else:
                    on = lin >= cfg.deadband_lin_on or ang >= cfg.deadband_w_on
                if not on:
                    limited_free = limited_of(xd, True)     # e.g. the base stopped at the reach boundary
                    self.active = False
                    fixed, why, deadband = self._step_toward(prev, np.zeros(3), accel, dt), "deadband", True
                    xs = solve(fixed, False)
                else:
                    self.active = True
                    xs = solve(None, False)
                    if xs is not None and np.any(rate_G @ xs > rate_h + 1e-9):
                        n_solves += 1
                        xs2 = solve(None, True)
                        if xs2 is None:                # e.g. the limits were lowered while faster: brake into them
                            fixed = self._limit_twist(self._step_toward(prev, self._limit_twist(prev, speed, omega),
                                                                        accel, dt), speed, omega)
                            why = "rate fallback"
                            xs2 = solve(fixed, False)
                            n_solves += 1
                        xs = xs2
        if xs is None:
            return self._zero(snap, "infeasible", f"QP infeasible (base {why or 'free'})")
        tw = fixed if fixed is not None else twist_of(xs)
        limited = limited_of(xs, fixed is None)
        if limited == "none" or (limited == "base_limit" and limited_free != "none"):
            limited = limited_free if limited_free != "none" else limited
        dqs = {k: xs[m.arm_v[k]] * dt for k in snap.ees}
        diag.update(solves=n_solves, wheel_max=self._wheel_max(tw), hold=hold)
        return Output("ok", np.asarray(tw, float), dqs, {k: xs[m.arm_v[k]].copy() for k in snap.ees},
                      deadband=deadband, base_fixed=why, limited_by=limited, diag=diag)
