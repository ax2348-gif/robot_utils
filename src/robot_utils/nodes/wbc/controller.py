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

- **lead bound** (per arm joint): the next command within ``lead`` (= ``lead_torque / kp``, at most the arm's
  ``max_lead``) of ``q_ref``, the measured joints extrapolated by their measured velocity over the measurement's
  age + ``lead_delay``: ``|q_cmd + q' ds - q_ref| <= lead`` (the window is the union of the ones around the
  measured and the extrapolated joints: the extrapolation only widens it), so in contact the servo's push
  ``kp (q_cmd - q)`` stays within ``lead_torque``. Outside the bound (a sudden blockage) the command returns toward it, never faster than the
  joint box allows (the joint box wins: always feasible).

``Output.limited_by``: which hard constraint bound the solution (reach, manipulability, joint_limit, lead,
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
LIMITED = ["none", "reach", "manipulability", "fence", "joint_limit", "base_limit", "base_band", "lead"]  # append-only
BIND_TOL = 1e-6          # velocity units: a row this close to its bound is binding


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


def hull_halfplanes(points) -> list:
    """The convex hull of 2D points as half-planes [(n, h)]: n . v <= h (n outward unit normals)."""
    pts = sorted({(round(float(p[0]), 12), round(float(p[1]), 12)) for p in points})
    if len(pts) < 3:
        return []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]                 # counter-clockwise
    out = []
    for i, a in enumerate(hull):
        b = hull[(i + 1) % len(hull)]
        d = np.array([b[0] - a[0], b[1] - a[1]])
        n = np.array([d[1], -d[0]]) / np.linalg.norm(d)
        out.append((n, float(n @ np.array(a))))
    return out


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
    q_meas: Optional[np.ndarray] = None   # the arm's measured joints (None: no lead bound this tick)
    qd_meas: Optional[np.ndarray] = None  # its measured joint velocity (extrapolates q_meas; None = 0)
    meas_age: float = 0.0             # s since that measurement (the arm state's t_tick to this tick)


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
    hint_age: float = 0.0             # s since the base hint in force was given (its velocity feedforward fades)
    dt: Optional[float] = None        # s, this tick's measured step (the node clamps it to [dt, 3 dt]): the arm
                                      # joint step is qd * dt and the joint position damper uses it; None = cfg.dt


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
        # per arm (joint_vel, joint_acc): the arm's own (the arm config's soft defaults until the node applies the
        # joint-channel welcome), narrowed by the config's when it gives them
        from ..arm.config import DEFAULT_SOFT
        self.caps = {k: self.arm_caps(len(v), DEFAULT_SOFT.joint_vel, DEFAULT_SOFT.joint_acc)
                     for k, v in model.arm_v.items()}
        # per arm the servo's kp (Nm/rad) and the arm's max_lead (rad): the arm's default profile until the node
        # applies the arm's own (GET /gains, the welcome)
        from .spec import default_arm_gains
        kp, ml, self.kp_source = default_arm_gains()
        self.kp = {k: kp[:len(v)].copy() for k, v in model.arm_v.items()}
        self.max_lead = {k: ml for k in model.arm_v}
        self.active = False                                   # base deadband state
        self.solves = 0

    def reset(self) -> None:
        self.active = False

    def arm_caps(self, n: int, arm_vel, arm_acc) -> tuple:
        """(joint_vel, joint_acc) from the arm's limits and the config's optional narrower ones."""
        vm, a = np.asarray(arm_vel, float)[:n], np.asarray(arm_acc, float)[:n]
        if self.cfg.joint_vel is not None:
            vm = np.minimum(vm, np.asarray(self.cfg.joint_vel, float)[:n])
        if self.cfg.joint_acc is not None:
            a = np.minimum(a, np.asarray(self.cfg.joint_acc, float)[:n])
        return vm, a

    def lead(self, ee: str) -> np.ndarray:
        """The lead bound per joint (rad): lead_torque / kp, at most the arm's max_lead."""
        kp = self.kp[ee]
        return np.minimum(np.asarray(self.cfg.lead_torque, float)[:len(kp)] / kp, self.max_lead[ee])

    def lead_ref(self, ee: str, e: "EEState"):
        """(q_ref, its velocity): the measured joints extrapolated by their measured velocity (capped at the joint
        speed) over the measurement's age + lead_delay. None without a usable measurement."""
        if e.q_meas is None or not np.all(np.isfinite(e.q_meas)):
            return None
        vm = self.caps[ee][0]
        qd = np.zeros(len(e.q_meas)) if e.qd_meas is None else np.nan_to_num(np.asarray(e.qd_meas, float))
        qd = np.clip(qd, -vm, vm)
        h = max(0.0, float(e.meas_age) if math.isfinite(e.meas_age) else 0.0) + self.cfg.lead_delay
        return np.asarray(e.q_meas, float) + qd * h, qd

    def _lead_bounds(self, ee: str, e: "EEState", L: np.ndarray, U: np.ndarray, ds: float):
        """(Ll, Ul, diag) velocity bounds of the lead bound, made feasible with the joint box [L, U] (the box
        wins); None without a measurement."""
        r = self.lead_ref(ee, e)
        if r is None:
            return None
        q_ref, _ = r
        lead = self.lead(ee)
        q = np.asarray(e.q_cmd, float)
        # the window: lead around the measured joints AND around their extrapolation (the union), so the
        # extrapolation only ever widens it on the side the arm moves to (a too long lead_delay never drags the
        # command along; a stopping command is always allowed); in contact (qd 0) both are the measured joints
        qm = np.asarray(e.q_meas, float)
        d_up = np.maximum(q_ref, qm) + lead - q       # room to the upper edge (< 0: outside, above)
        d_lo = q - (np.minimum(q_ref, qm) - lead)     # room to the lower edge
        # inside: up to the edge within this step; outside (a sudden blockage): back toward it, no faster than the
        # joint box allows (the box wins, so always feasible: the joint acceleration limit). No braking-distance
        # term: relative to a delayed measurement it cannot tell "about to follow" from "blocked" and throttled
        # free acceleration; approaching a still edge at speed v overshoots it by at most v^2 / 2a (the box).
        # The return from outside is no faster than it can stop at the edge (the discrete braking speed
        # v^2 / 2a + v ds / 2 <= d), else the acceleration-limited return overshoots inward and rings.
        # d_up + d_lo >= 2 lead > 0: the two edges never cross
        a = self.caps[ee][1]
        hb = 0.5 * a * ds

        def toward(d):          # d < 0: the (negative) speed limit of the way back
            return -np.minimum(-d / ds, -hb + np.sqrt(hb * hb + 2 * a * np.maximum(-d, 0.0)))
        Ul = np.maximum(np.where(d_up >= 0, d_up / ds, toward(d_up)), L)
        Ll = np.minimum(np.where(d_lo >= 0, -d_lo / ds, -toward(d_lo)), U)
        return Ll, Ul, {"lead": lead, "dev": q - np.clip(q, np.minimum(q_ref, qm), np.maximum(q_ref, qm))}

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
        """The base twist when safety fixes it (a hold, mode ``locked``, no base): braking to zero within the rate
        limit. Everything else is the QP's (the references and their bands)."""
        h = snap.hints
        _, _, accel = effective_limits(h, self.cfg)
        prev = snap.base_prev
        if snap.hold:
            return self._step_toward(prev, np.zeros(3), accel, dt), "hold"
        if not snap.base_available or h.mode == "locked":
            return self._step_toward(prev, np.zeros(3), accel, dt), "locked" if snap.base_available else "no base"
        return None, None

    def _base_refs(self, snap: Snapshot):
        """The base references in world axes: (pose_ref (x, y) | None, yaw_ref | None, twist_ref (vx, vy, w) | None,
        pose tolerance, yaw tolerance). From mode ``hint`` (target pose/heading with their velocities, the velocity
        fading like the EE twist), ``commanded`` (a velocity while fresh, or a pose); ``free``: none."""
        h, cfg, m = snap.hints, self.cfg, self.m
        x, y, yaw = snap.base
        p_ref = yaw_ref = v_ref = None
        tol_p = tol_y = 0.0
        if h.mode == "hint":
            a = snap.hint_age
            kff = 1.0 if a <= cfg.ff_timeout else max(0.0, 1.0 - (a - cfg.ff_timeout) / cfg.ff_timeout)
            v_ref = np.zeros(3)
            if h.target_pose is not None:
                tp = h.target_pose
                p_ref, tol_p = np.array([tp.x, tp.y]), tp.tol
                v_ref[:2] = kff * np.array([tp.vx, tp.vy])
            if h.target_heading is not None:
                hh = h.target_heading
                yt = hh.yaw
                if hh.face is not None:
                    tgt = snap.ees[hh.face].target if hh.face in snap.ees else None
                    yt = None if tgt is None else math.atan2(tgt.p[1] - y, tgt.p[0] - x) - m.mount_yaw.get(hh.face, 0.0)
                elif hh.point is not None:
                    yt = math.atan2(hh.point[1] - y, hh.point[0] - x) - next(iter(m.mount_yaw.values()), 0.0)
                yaw_ref, tol_y = yt, hh.tol
                v_ref[2] = kff * hh.omega
        elif h.mode == "commanded":
            if h.velocity is not None:
                v_ref = np.zeros(3)
                if snap.velocity_age <= h.velocity.ttl:
                    v = h.velocity
                    v_ref = np.array([v.vx, v.vy, v.omega], float)
                    if v.frame != "world":
                        c, s_ = math.cos(yaw), math.sin(yaw)
                        v_ref[:2] = [c * v.vx - s_ * v.vy, s_ * v.vx + c * v.vy]
            elif h.pose is not None:
                p_ref, yaw_ref, v_ref = np.array([h.pose.x, h.pose.y]), h.pose.yaw, np.zeros(3)
        return p_ref, yaw_ref, v_ref, tol_p, tol_y

    def _joint_box(self, ee: str, q: np.ndarray, v_prev: np.ndarray,
                   dt_step: Optional[float] = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(L, U, position-bound mask) velocity bounds of an arm's joints this tick (always L <= U). ``dt_step``: the
        step the velocity is integrated over (the measured one on a late tick), so a step never crosses a limit."""
        cfg, dt = self.cfg, self.cfg.dt
        ds = dt_step or dt
        n = len(q)
        lo, hi = self.m.arm_limits(ee)
        vm, a = self.caps[ee]
        dh, dl = hi - q, q - lo
        # within the limits: the damper and the braking distance; beyond one: back toward it (same law)
        U_pos = np.sign(dh) * np.minimum(cfg.limit_gain * np.abs(dh) / ds, np.sqrt(2 * a * np.abs(dh)))
        L_pos = -np.sign(dl) * np.minimum(cfg.limit_gain * np.abs(dl) / ds, np.sqrt(2 * a * np.abs(dl)))
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
        manip_rows, reach_rows, task_rows, lead_rows = [], [], [], []
        for name, e in snap.ees.items():
            iv = m.arm_v[name]
            x_prev[iv] = e.v_prev
            L, U, posmask, Lr, Ur = self._joint_box(name, e.q_cmd, e.v_prev, snap.dt)
            S = np.zeros((len(iv), nv))
            S[np.arange(len(iv)), iv] = 1.0
            groups.append(("joint_limit_box", np.vstack([S, -S]), np.concatenate([U, -L]), posmask,
                           np.concatenate([Ur, -Lr])))
            lb = self._lead_bounds(name, e, L, U, snap.dt or dt)
            if lb is not None:
                Ll, Ul, ld = lb
                lead_rows.append((np.vstack([S, -S]), np.concatenate([Ul, -Ll])))
                diag.setdefault("lead", {})[name] = {k: [float(v) for v in vv] for k, vv in ld.items()}
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
            if cfg.w_min > 0 and mu < cfg.w_influence:
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
                # how fast the base can slow d' down: by translating (max_accel; the braking distance counts on
                # this only: the yaw rate may be saturated) and, within one tick, also by turning (max_alpha: the
                # shoulder sits off the base's centre); a margin for the octagons
                a_lin = 0.7 * math.hypot(G[0, bx], G[0, by]) * accel
                a_eff = a_lin + 0.7 * abs(G[0, bw]) * cfg.max_alpha
                # the damper beyond d_i, and everywhere the base's braking distance to d_max
                hr = math.sqrt(2 * max(a_lin, 1e-3) * max(d_max - d, 0.0))
                if d > d_i:
                    hr = min(hr, cfg.xi_reach * (d_max - d) / (d_max - d_i))
                hr -= float(u @ tg.v[:3])
                f_prev = float((G @ x_prev)[0])
                floor = max(f_prev - a_eff * dt, 0.0)              # stopping (within the rate limits) is allowed
                reach_rows.append((G, np.array([max(hr, floor)])))
        # -- the base: references, bands, caps (one QP; docs/plans/wbc_v1.md "Formulation") ----------------
        fixed, why = self._fixed_twist(snap, dt)
        p_ref, yaw_ref, v_ref, tol_p, tol_y = self._base_refs(snap)
        has_ref = p_ref is not None or yaw_ref is not None or v_ref is not None
        v_des = np.zeros(3)
        if v_ref is not None:
            v_des += v_ref
        if p_ref is not None:
            v_des[:2] += cfg.k_base_pose * shrink(p_ref - np.array([x, y]), tol_p)
        if yaw_ref is not None:
            v_des[2] += cfg.k_heading * shrink(wrap(yaw_ref - yaw), tol_y)
        wb = (cfg.w_base_ref if has_ref else cfg.w_base_free) * h.arm_vs_base
        diag.update(w_base=wb, v_des=[float(v) for v in v_des])
        for i, (k, r2) in enumerate(((bx, 1.0), (by, 1.0), (bw, cfg.base_yaw_radius ** 2))):
            H[k, k] += wb * r2
            c[k] -= wb * r2 * v_des[i]
        # auxiliary variables (each >= 0, a linear cost, a tiny quadratic so H stays positive definite):
        # [s_pose, s_twist] the band slacks (w_slack); [t_lin, t_yaw] >= the base speed (octagon norm) and |yaw rate|,
        # costed at w_base_flat / w_base_flat_yaw: the flat cost of moving the base at all
        ns = 4
        N = nv + ns
        Hx = np.zeros((N, N))
        Hx[:nv, :nv] = H
        Hx[nv:, nv:] = 1e-6 * np.eye(ns)
        cx = np.concatenate([c, [cfg.w_slack, cfg.w_slack, cfg.w_base_flat, cfg.w_base_flat_yaw]])

        def pad(G_, slack: Optional[int] = None):
            G_ = np.atleast_2d(G_)
            out = np.zeros((G_.shape[0], N))
            out[:, :nv] = G_
            if slack is not None:
                out[:, nv + slack] = -1.0
            return out

        Bm = np.zeros((2, nv))                     # body linear velocity = Bm x
        Bm[0, bx], Bm[0, by], Bm[1, bx], Bm[1, by] = cy, sy, -sy, cy
        Wxy = np.zeros((2, nv))                    # world linear velocity
        Wxy[0, bx], Wxy[1, by] = 1.0, 1.0
        Wy = np.zeros(nv)
        Wy[bw] = 1.0
        rows: list = []                            # (kind, G (N cols), h)
        for _, G_, h_, pm, _r in groups:           # the arm joint boxes (hard); position bounds report joint_limit
            rows.append(("joint_limit", pad(G_[pm]), h_[pm]))
            rows.append(("joint_cap", pad(G_[~pm]), h_[~pm]))
        for G_, h_ in manip_rows:
            rows.append(("manipulability", pad(G_), h_))
        for G_, h_ in lead_rows:                   # the lead bound (hard; feasible with the joint box)
            rows.append(("lead", pad(G_), h_))
        rows.append(("slack", np.hstack([np.zeros((ns, nv)), -np.eye(ns)]), np.zeros(ns)))
        # t_lin >= octagon norm of the base's linear velocity, t_yaw >= |yaw rate| (world axes: the norm is rotation
        # invariant up to the octagon)
        Wxy0 = np.zeros((2, nv))
        Wxy0[0, bx], Wxy0[1, by] = 1.0, 1.0
        rows.append(("flat", pad(OCT @ Wxy0, 2) * np.concatenate([np.ones(nv), [1, 1, C8, 1]]), np.zeros(8)))
        Wy0 = np.zeros(nv)
        Wy0[bw] = 1.0
        rows.append(("flat", pad(np.vstack([Wy0, -Wy0]), 3), np.zeros(2)))
        if fixed is None:
            # caps (hard), relaxed to the previous twist so braking into a lowered cap stays feasible
            def cap_rows(G_, bound):
                return np.maximum(bound, G_ @ x_prev - 1e-9)
            G_sp = OCT @ Bm
            rows.append(("base_limit", pad(G_sp), cap_rows(G_sp, speed * C8)))
            G_w = np.vstack([Wy, -Wy])
            rows.append(("base_limit", pad(G_w), cap_rows(G_w, omega)))
            for wx, wy in self.wheels:
                G_wh = OCT @ (Bm + np.outer([-wy, wx], Wy))
                rows.append(("base_limit", pad(G_wh), cap_rows(G_wh, cfg.wheel_speed * C8)))
            # twist rate (hard)
            rows.append(("rate", pad(np.vstack([OCT @ Bm, Wy, -Wy])),
                         np.concatenate([accel * dt * C8 + OCT @ prev[:2],
                                         [cfg.max_alpha * dt + prev[2], cfg.max_alpha * dt - prev[2]]])))
            for G_, h_ in reach_rows:
                rows.append(("reach", pad(G_), h_))
            # pose band: the deviation from the pose reference grows at most at k_band (band - deviation), never
            # forced back faster than that (outside the band: it may only shrink); slack 0
            vr = v_ref if v_ref is not None else np.zeros(3)
            if p_ref is not None:
                e = np.array([x, y]) - p_ref
                rows.append(("base_band", pad(OCT @ Wxy, 0),
                             OCT @ vr[:2] + np.maximum(cfg.k_band * (cfg.band_pos - OCT @ e), 0.0)))
            if yaw_ref is not None:
                ey = wrap(yaw - yaw_ref)
                rows.append(("base_band", pad(np.vstack([Wy, -Wy]), 0),
                             np.array([vr[2] + max(cfg.k_band * (cfg.band_yaw - ey), 0.0),
                                       -vr[2] + max(cfg.k_band * (cfg.band_yaw + ey), 0.0)])))
            # twist band around the base's OWN reference twist v_des (= v_ref + the pose-reference correction):
            # conv{0, octagon(v_des, band_twist)}: the base may lag it down to zero, but the EE task may push it at
            # most band_twist beyond its reference (it never stops the base from tracking it); slack 1
            if has_ref:
                verts = [np.zeros(2)] + [v_des[:2] + cfg.band_twist / C8 * np.array([math.cos((k + 0.5) * math.pi / 4),
                                                                                     math.sin((k + 0.5) * math.pi / 4)])
                                         for k in range(8)]
                for n_, h0 in hull_halfplanes(verts):
                    rows.append(("base_band", pad((n_ @ Wxy).reshape(1, nv), 1), np.array([h0])))
                lo, hi = min(0.0, v_des[2] - cfg.band_omega), max(0.0, v_des[2] + cfg.band_omega)
                rows.append(("base_band", pad(np.vstack([Wy, -Wy]), 1), np.array([hi, -lo])))

        def solve_one(base_fixed: Optional[np.ndarray], rate: bool = True):
            use = [r_ for r_ in rows if rate or r_[0] != "rate"]
            G = np.vstack([r_[1] for r_ in use])
            hh = np.concatenate([np.atleast_1d(r_[2]) for r_ in use])
            A = b = None
            if base_fixed is not None:
                A = np.zeros((3, N))
                A[0, bx] = A[1, by] = A[2, bw] = 1.0
                vb = base_fixed
                b = np.array([cy * vb[0] - sy * vb[1], sy * vb[0] + cy * vb[1], vb[2]])
            sol = qpsolvers.solve_problem(qpsolvers.Problem(Hx, cx, G, hh, A, b), solver=self.solver)
            self.solves += 1
            if not sol.found or sol.x is None or not np.all(np.isfinite(sol.x)):
                return None
            return np.asarray(sol.x)

        def twist_of(xs):
            return np.array([cy * xs[bx] + sy * xs[by], -sy * xs[bx] + cy * xs[by], xs[bw]])

        n_solves = 1
        J = lambda x_: float(0.5 * x_ @ Hx @ x_ + cx @ x_)      # noqa: E731  the QP's objective
        at_rest = (fixed is None and cfg.base_start_charge > 0 and math.hypot(prev[0], prev[1]) < cfg.rest_lin
                   and abs(prev[2]) < cfg.rest_ang)
        if at_rest:
            # the fixed charge of starting the base: held (braking to zero) vs free, the charge added to free
            # decided on the base's DESIRED motion (free without the twist-rate rows: from rest one tick's
            # rate-limited step is too small to ever beat a charge) against holding; then the command
            hold_tw = self._step_toward(prev, np.zeros(3), accel, dt)
            x_want, x_hold = solve_one(None, rate=False), solve_one(hold_tw, rate=False)
            n_solves += 1
            j_free = J(x_want) if x_want is not None else math.inf
            j_hold = J(x_hold) if x_hold is not None else math.inf
            diag.update(start_charge={"free": j_free, "hold": j_hold, "charge": cfg.base_start_charge})
            if x_hold is not None and j_free + cfg.base_start_charge >= j_hold:
                xs, fixed, why = solve_one(hold_tw), hold_tw, "at rest (start charge)"
            else:
                xs = solve_one(None)
            n_solves += 1
        else:
            xs = solve_one(fixed)
        if xs is None and fixed is None:          # e.g. caps lowered while faster: brake within the rate limit
            fixed = self._limit_twist(self._step_toward(prev, self._limit_twist(prev, speed, omega), accel, dt),
                                      speed, omega)
            why = "rate fallback"
            xs = solve_one(fixed)
            n_solves += 1
        if xs is None:
            return self._zero(snap, "infeasible", f"QP infeasible (base {why or 'free'})")
        slack = xs[nv:]
        u = xs[:nv]
        # what bound the solution (by priority): a band that needed slack or binds, then the hard rows
        limited = "none"
        for kind in ("reach", "manipulability", "joint_limit", "lead", "base_band", "base_limit", "rate"):
            for k_, G_, h_ in rows:
                if k_ == kind and np.any(G_ @ xs >= np.atleast_1d(h_) - BIND_TOL):
                    limited = kind
                    break
            if limited != "none":
                break
        if limited == "rate":
            limited = "base_limit"                 # the base's twist-rate rows (LIMITED is the stream's enum)
        diag["lead_binding"] = any(k_ == "lead" and np.any(G_ @ xs >= np.atleast_1d(h_) - BIND_TOL)
                                   for k_, G_, h_ in rows)
        tw = fixed if fixed is not None else twist_of(u)
        self.active = bool(np.any(np.abs(tw) > 1e-9))
        dqs = {k: u[m.arm_v[k]] * (snap.dt or dt) for k in snap.ees}   # integrated over the measured step
        diag.update(solves=n_solves, wheel_max=self._wheel_max(tw), hold=hold,
                    slack=[float(v) for v in slack[:2]], flat=[float(v) for v in slack[2:]])
        return Output("ok", np.asarray(tw, float), dqs, {k: u[m.arm_v[k]].copy() for k in snap.ees},
                      deadband=False, base_fixed=why, limited_by=limited, diag=diag)
