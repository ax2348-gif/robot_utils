"""The human-like null-space posture of a redundant arm (a 7-DOF arm: one free angle, the elbow's swivel about the
shoulder-wrist line), and the soft arm-proximity term (docs/wbc.md "Null-space posture").

Each term is a joint-velocity attractor ``(w, v)`` on the arm's joints; the controller costs them on the EE task's
null space only (``½ Σ w_i |P (q' - v_i)|²``, P the free joints' null-space projector), so none of them moves the
EE. The weights set the priority (EE task >> proximity > swivel > effort > centering):

- **proximity** (``w_arm_proximity``; 0 = off, and off with one arm): the upper-arm / forearm capsules of two arms
  (the arm type's ``geometry.capsules``; never the wrists or grippers: the hands come together freely). Strictly
  short range: with ``s = (c0 - d) / c0`` for a clearance ``d`` below ``proximity_clearance`` (c0), the weight is
  ``w s²`` and the wanted separation speed ``proximity_speed s``; beyond c0 the term is exactly zero (value and
  gradient), so in the natural posture it is dormant and the posture is the same with it on or off. Per pair and
  arm: ``v = g ḋ / |g|²``, g = ∂d/∂q (the closest point's Jacobian along the pair's normal; the other arm held).
- **swivel** (``w_swivel``): a target swivel ψ* from the hand pose, manipulability based (Kim & Rosen): for the
  shoulder S and wrist W fixed, the elbow lies on a circle; on a 4-joint arm model (a spherical shoulder, the elbow)
  the wrist's velocity transmission along a direction u is ``|r × u|² + (u · n × (W - E))²``, n the elbow's axis;
  ψ* is its soft argmax over ``swivel_range`` (continuous in the hand pose: no flips), low-passed (``swivel_tau``).
  ``swivel_dir``: ``head`` (u from the wrist to a head point: the feeding / looking-at-hand direction of the human
  studies), ``torso`` (to the torso centre: pulling in), ``motion`` (along the hand's wanted velocity, the head
  direction while it is slow). ψ is measured numerically (the elbow off the S-W line against the world's down
  projected on its normal plane; + is elbow out, away from the body), so no spherical-joint assumption is needed;
  ``v = k (ψ* - ψ) ∇ψ / |∇ψ|²``.
- **the operator's swivel hint** (a per-EE ``swivel`` {seq, dpsi, elbow, conf} in the target datagrams: the
  operator's swivel change since their engagement): on a new ``seq`` (or its return after going stale) it anchors at
  the robot's commanded continuous swivel ψ_r (``swivel_geom.py``: parallel-transported increments, no fixed zero,
  nothing singular with the S-W line vertical); the target ``ψ_r0 + swivel_hint_gain (dpsi - dpsi0)`` is low-passed
  and rate-capped; its weight (the operator's elbow flexion, conf, ramped over ``swivel_hint_blend``; times the robot
  elbow's flexion fade) replaces ψ*'s attractor by its share: ``w_swivel_hint``, rate ``k_swivel_hint (target -
  ψ_r)`` capped at ``swivel_hint_rate``, along ∇ψ_r. Stale (released, or the body lost): held by default
  (``swivel_hint_hold``: the target and weight stay, the elbow where the operator left it; the next engagement
  re-anchors there), else the weight ramps out to the model. docs/wbc.md "The operator's elbow".
- **effort** (``w_effort``): ``-k ∇ Σ (τ_g,i / τ_max,i)²``: the gravity torque the arm holds, per joint relative to
  its torque limit (pinocchio's gravity derivatives).
- **centering** (``w_center``): ``k (mid - q) / half-range`` on the shoulder and wrist joints (the arm type's
  ``geometry.centering``), a RULA-like comfort preference.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pinocchio as pin

from . import swivel_geom as SG

DOWN = np.array([0.0, 0.0, -1.0])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def seg_closest(p0, p1, q0, q1) -> tuple[float, float, float]:
    """(distance, s, t): the closest points p0 + s (p1 - p0), q0 + t (q1 - q0) of two segments."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    c, b = float(d1 @ r), float(d1 @ d2)
    den = a * e - b * b
    s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
    t = (b * s + f) / e if e > 1e-12 else 0.0
    if t < 0.0:
        t, s = 0.0, min(max(-c / a, 0.0), 1.0) if a > 1e-12 else 0.0
    elif t > 1.0:
        t, s = 1.0, min(max((b - c) / a, 0.0), 1.0) if a > 1e-12 else 0.0
    return float(np.linalg.norm((p0 + s * d1) - (q0 + t * d2))), s, t


def swivel_frame(S, W):
    """(axis, r0, r1): the S-W axis, the world's down projected normal to it (the swivel's zero) and axis x r0."""
    a = W - S
    a = a / max(np.linalg.norm(a), 1e-9)
    r0 = DOWN - (DOWN @ a) * a
    if np.linalg.norm(r0) < 1e-3:                                 # the arm along the vertical: x as the reference
        r0 = np.array([1.0, 0.0, 0.0]) - a[0] * a
    r0 = r0 / np.linalg.norm(r0)
    return a, r0, np.cross(a, r0)


def swivel_angle(S, E, W, sign: float) -> float:
    """The elbow's swivel (rad): 0 elbow straight down the S-W line's normal plane, + out (``sign``: +1 for an arm
    on the +y side, -1 on the -y side)."""
    a, r0, r1 = swivel_frame(S, W)
    p = E - S
    return math.atan2(sign * float(p @ r1), float(p @ r0))


def swivel_angles(S, E, W, sign: float) -> np.ndarray:
    """``swivel_angle`` over rows of points (n, 3)."""
    a = W - S
    a = a / np.maximum(np.linalg.norm(a, axis=1, keepdims=True), 1e-9)
    r0 = DOWN - (a @ DOWN)[:, None] * a
    flat = np.linalg.norm(r0, axis=1) < 1e-3
    if flat.any():
        r0[flat] = np.array([1.0, 0.0, 0.0]) - a[flat, :1] * a[flat]
    r0 = r0 / np.linalg.norm(r0, axis=1, keepdims=True)
    r1 = np.cross(a, r0)
    p = E - S
    return np.arctan2(sign * np.sum(p * r1, axis=1), np.sum(p * r0, axis=1))


def swivel_target(S, E, W, u, sign: float, lo: float, hi: float, beta: float, n: int = 61) -> float:
    """ψ* (rad): the swivel in [lo, hi] maximising the wrist's velocity transmission along u on the 4-joint
    model (the soft argmax, continuous in its inputs)."""
    a, r0, r1 = swivel_frame(S, W)
    L1, L2, d = np.linalg.norm(E - S), np.linalg.norm(W - E), np.linalg.norm(W - S)
    x = (L1 * L1 - L2 * L2 + d * d) / (2 * d)
    r = math.sqrt(max(L1 * L1 - x * x, 1e-8))
    psi = np.linspace(lo, hi, n)
    Es = S + x * a + r * (np.cos(psi)[:, None] * r0 + (sign * np.sin(psi))[:, None] * r1)
    nx = np.cross(Es - S, W - Es)
    nx /= np.maximum(np.linalg.norm(nx, axis=1, keepdims=True), 1e-9)
    c = np.cross(nx, W - Es)
    mval = (c @ u) ** 2 + float(np.linalg.norm(np.cross(W - S, u)) ** 2)
    w = np.exp(beta * (mval / max(float(mval.max()), 1e-12) - 1.0))
    return float(w @ psi / w.sum())


def along(N, g, rate: float, eps: float = 1e-3) -> np.ndarray:
    """The joint velocity that changes a scalar of gradient g at ``rate`` per second moving only along the columns
    of N (the EE task's null space: the self-motion; damped where the null space barely changes the scalar), or
    along g itself without N."""
    if N is None or N.shape[1] == 0:
        return g * rate / max(float(g @ g), eps)
    a = N.T @ g
    return N @ a * (rate / (float(a @ a) + eps))


@dataclass
class ArmGeom:
    ee: str
    S: int              # joint ids (the model's) of the shoulder, elbow, wrist points
    E: int
    W: int
    sign: float
    center: np.ndarray  # indices into the arm's joints
    capsules: list      # (name, joint id from, joint id to, radius, trim)
    tau_max: np.ndarray


@dataclass
class HintState:
    """One arm's operator swivel hint (``Humanlike._hint_update``); swivels on the arm's continuous scale."""
    target: float                     # the smoothed, rate-capped target in force
    seq: int = -1                     # the engagement anchored
    psi0: float = math.nan            # the anchor: the robot's commanded swivel then
    dpsi0: float = 0.0                # the operator's dpsi then (0 at a fresh engagement)
    dpsi: float = 0.0                 # the latest dpsi
    raw: float = math.nan             # psi0 + gain (dpsi - dpsi0), before smoothing
    age: Optional[float] = None       # s since the latest input arrived
    live: bool = False                # fresh and anchored (False: stale, fading out, re-anchors on return)
    alpha: float = 0.0                # the operator-side weight, ramped (elbow fade x conf fade)
    f_elbow: float = 0.0
    f_conf: float = 0.0
    anchors: int = 0                  # anchorings so far (new seqs and returns)


class Humanlike:
    def __init__(self, m, cfg):
        from ...robots import arm_types
        self.m, self.cfg = m, cfg
        self.arms: dict[str, ArmGeom] = {}
        self.psi_star: dict[str, float] = {}
        self.prox_active: dict[str, float] = {}
        self.psi_acc: dict[str, float] = {}       # the continuous swivel of the commanded arm (swivel_geom.py), base frame
        self._base_j = m.model.getJointId("base_yaw")   # the base frame: the planar root's last joint (model.py)
        self._acc: dict = {}
        self.hint: dict[str, HintState] = {}      # the operator's swivel hint per arm (while given or fading)
        for name, e in m.ees.items():
            t = arm_types.load(e.arm_type) if e.arm_type else None
            g = (t.geometry if t is not None else None) or None
            if g is None or len(e.joints) <= 6:
                continue
            jid = [m.model.getJointId(j) for j in e.joints]
            caps = [(c["name"], jid[c["from"]], jid[c["to"]], float(c["radius"]), float(c.get("trim", 0.0)))
                    for c in g.get("capsules", [])]
            sign = 1.0 if m.mount_xy[name][1] >= 0 else -1.0
            self.arms[name] = ArmGeom(name, jid[g["shoulder"]], jid[g["elbow"]], jid[g["wrist"]], sign,
                                      np.array(g.get("centering", []), int), caps, np.asarray(t.torque_max, float))
            self._acc[name] = SG.Accumulator(sign)

    def reset(self) -> None:
        """Forget the low-passed targets, the continuous swivels and the hints (a controller reset)."""
        self.psi_star.clear()
        self.psi_acc.clear()
        self.hint.clear()
        for a in self._acc.values():
            a.reset()

    # -- per tick --------------------------------------------------------------------------------------

    def prepare(self, q: np.ndarray) -> None:
        """Kinematics, joint Jacobians and the gravity derivatives at q (once per tick, before terms())."""
        m = self.m
        self._planned_this_tick = False                   # at most one swivel trace per tick (the arms in turn)
        self._qa = {k: np.array(q[m.arm_q[k]]) for k in self.arms}
        pin.forwardKinematics(m.model, m.data, q)
        pin.computeJointJacobians(m.model, m.data, q)
        self._g = pin.computeGeneralizedGravity(m.model, m.data, q).copy()
        self._dg = pin.computeGeneralizedGravityDerivatives(m.model, m.data, q).copy()
        self._pos, self._J = {}, {}
        for a in self.arms.values():              # cached now: the controller reuses m.data afterwards
            for j in {a.S, a.E, a.W} | {c[1] for c in a.capsules} | {c[2] for c in a.capsules}:
                self._pos[j] = np.array(m.data.oMi[j].translation)
                self._J[j] = np.array(pin.getJointJacobian(m.model, m.data, j, pin.LOCAL_WORLD_ALIGNED)[:3])
            # the continuous swivel in the base frame: in the world a base turn moves the S-W line about the vertical,
            # and parallel transport along that path is not the rigid rotation (it differs by ~dyaw cos(the line's
            # angle to the vertical)), so the swivel drifted with no arm motion and the held hint moved the elbow to
            # "correct" it (the owner, 2026-11-26: the elbows drifted when driving by stick and stayed there). The
            # steps are invariant under a rotation of both samples, so the hint's gradient (world) is unchanged
            Tb = m.data.oMi[self._base_j]
            self.psi_acc[a.ee] = self._acc[a.ee].update(*(Tb.actInv(self._pos[j]) for j in (a.S, a.E, a.W)))

    def _p(self, j: int):
        return self._pos[j], self._J[j]

    def swivel(self, name: str) -> float:
        a = self.arms[name]
        return swivel_angle(self._p(a.S)[0], self._p(a.E)[0], self._p(a.W)[0], a.sign)

    def gravity_effort(self, name: str) -> float:
        """Σ (τ_g / τ_max)² of the arm (dimensionless), at the prepared q."""
        a = self.arms[name]
        return float(np.sum((self._g[self.m.arm_v[name]] / a.tau_max) ** 2))

    def capsules(self, name: str) -> list:
        """[(name, p0, p1, J0, J1, radius)] of the arm's capsules at the prepared q (world)."""
        out = []
        for cn, j0, j1, rad, trim in self.arms[name].capsules:
            p0, J0 = self._p(j0)
            p1, J1 = self._p(j1)
            L = float(np.linalg.norm(p1 - p0))
            f = max(0.0, 1.0 - trim / L) if L > 1e-9 else 1.0      # trimmed short of the far end
            out.append((cn, p0, p0 + f * (p1 - p0), J0, J0 + f * (J1 - J0), rad))
        return out

    def arm_clearance(self) -> Optional[float]:
        """The smallest capsule-to-capsule clearance between two arms (m; None with fewer than two arms)."""
        names = list(self.arms)
        best = None
        for i in range(len(names)):
            for k in range(i + 1, len(names)):
                for _, a0, a1, _, _, ra in self.capsules(names[i]):
                    for _, b0, b1, _, _, rb in self.capsules(names[k]):
                        d = seg_closest(a0, a1, b0, b1)[0] - ra - rb
                        best = d if best is None else min(best, d)
        return best

    def proximity(self, N: dict) -> tuple[dict, Optional[float]]:
        """({arm: [(w, v)]}, the smallest clearance): the arm-proximity attractors (none beyond the clearance)."""
        cfg, m = self.cfg, self.m
        out: dict = {k: [] for k in self.arms}
        if cfg.w_arm_proximity <= 0 or len(self.arms) < 2:
            return out, None
        c0 = cfg.proximity_clearance
        names = list(self.arms)
        best = None
        for i in range(len(names)):
            for k in range(i + 1, len(names)):
                A, B = names[i], names[k]
                for _, a0, a1, Ja0, Ja1, ra in self.capsules(A):
                    for _, b0, b1, Jb0, Jb1, rb in self.capsules(B):
                        dist, s, t = seg_closest(a0, a1, b0, b1)
                        d = dist - ra - rb
                        best = d if best is None else min(best, d)
                        if d >= c0 or dist < 1e-9:
                            continue
                        pa, pb = a0 + s * (a1 - a0), b0 + t * (b1 - b0)
                        nrm = (pa - pb) / dist
                        sv = min((c0 - d) / c0, 1.0)
                        w, dd = cfg.w_arm_proximity * sv * sv, cfg.proximity_speed * sv
                        for arm, J, sg in ((A, Ja0 + s * (Ja1 - Ja0), 1.0), (B, Jb0 + t * (Jb1 - Jb0), -1.0)):
                            g = sg * (nrm @ J)[m.arm_v[arm]]            # d' of this arm's joints (the other held)
                            out[arm].append((w, along(N.get(arm), g, dd)))
        return out, best

    def _planner(self, name):
        from .swivel_feasible import SwivelPlanner
        pl = self.__dict__.setdefault("_planners", {})
        if name not in pl:
            a, mdl = self.arms[name], self.m.model
            pl[name] = SwivelPlanner(self.m, name, mdl.names[a.S], mdl.names[a.E], mdl.names[a.W], a.sign)
        return pl[name]

    def _feasible_target(self, name, base, tg, psi_pref, t):
        """The limit-aware swivel target (swivel_feasible.py) for the target pose (the lookahead: while tracking it is
        the hand's own pose): the human-like preference clipped into the elbow's room within the joint limits. The room
        is re-planned every ``swivel_plan_period`` s (the arms in turn), the preference clipped into it every tick;
        (psi target or None, status, room)."""
        cfg, m = self.cfg, self.m
        cache = self.__dict__.setdefault("_feas", {})
        c = cache.get(name)
        due = c is None or t - c["t"] >= cfg.swivel_plan_period or t < c["t"]
        if due and not self.__dict__.get("_planned_this_tick"):
            from .swivel_feasible import choose
            x, y, yaw = base
            Tb = pin.SE3(pin.rpy.rpyToMatrix(0.0, 0.0, yaw), np.array([x, y, 0.0])).inverse()
            Tg = Tb * pin.SE3(np.asarray(tg.R, float), np.asarray(tg.p, float))
            lo, hi = m.arm_limits(name)
            tr = self._planner(name).trace(self._qa[name], Tg, lo, hi, cfg.null_limit_margin)
            room, status = choose(tr, psi_pref)
            c = cache[name] = {"t": t, "room": room, "status": status}
            self._planned_this_tick = True
        if c is None:
            return None, "pending", None                  # not planned yet (the other arm's turn)
        if c["room"] is None:
            return None, "unreachable", None
        a, b = c["room"]
        mid = 0.5 * (a + b)
        pref = mid + wrap(psi_pref - mid)                  # the preference, unwrapped near the room
        tgt = min(max(pref, a), b)
        status = c["status"] if c["status"] == "re-posing" else ("ok" if tgt == pref else "limited")
        return tgt, status, (a, b)

    def _hint_update(self, name: str, hint: Optional[SG.HintInput], dt: float) -> Optional[HintState]:
        """The operator's swivel hint, one tick: anchoring (a new seq, or the first fresh input after none or stale:
        ψ_r0 = the robot's commanded swivel, dpsi0 = the input's dpsi), the target (``ψ_r0 + gain (dpsi - dpsi0)``,
        low-passed over swivel_hint_tau, rate-capped at swivel_hint_rate), the operator-side weight (the elbow's
        flexion fade times the conf fade, ramped at 1 / swivel_hint_blend per s; toward 0 while stale, the target
        held). None once it has faded out with no input."""
        cfg = self.cfg
        st = self.hint.get(name)
        fresh = hint is not None and hint.age <= cfg.swivel_hint_timeout
        if not fresh and st is None:
            return None
        psi_r = self.psi_acc[name]
        if st is None:
            st = self.hint[name] = HintState(target=psi_r)
        if fresh:
            if not st.live or hint.seq != st.seq:
                st.seq, st.psi0, st.dpsi0, st.live = int(hint.seq), psi_r, float(hint.dpsi), True
                st.anchors += 1
                if st.alpha <= 0.0:
                    st.target = psi_r                       # nothing in force: start where the robot is
            st.dpsi, st.age = float(hint.dpsi), float(hint.age)
            st.raw = st.psi0 + cfg.swivel_hint_gain * (st.dpsi - st.dpsi0)
            lp = st.target + min(1.0, dt / cfg.swivel_hint_tau) * (st.raw - st.target)
            cap = cfg.swivel_hint_rate * dt
            st.target += min(max(lp - st.target, -cap), cap)
            st.f_elbow = SG.fade(math.pi - float(hint.elbow), math.radians(cfg.swivel_hint_flex[0]),
                                 math.radians(cfg.swivel_hint_flex[1]))
            st.f_conf = SG.fade(float(hint.conf), cfg.swivel_hint_conf[0], cfg.swivel_hint_conf[1])
            want = st.f_elbow * st.f_conf
        else:
            st.live = False
            st.age = None if hint is None else float(hint.age)
            # held (swivel_hint_hold): the target and the weight stay, the elbow where the operator left it; else the
            # weight ramps out, back to the model's swivel
            want = st.alpha if cfg.swivel_hint_hold else 0.0
        step = dt / cfg.swivel_hint_blend
        st.alpha += min(max(want - st.alpha, -step), step)
        if st.alpha <= 1e-9 and not fresh:
            del self.hint[name]                              # faded out: the next input re-anchors from scratch
            return None
        return st

    def hint_grad(self, name: str) -> np.ndarray:
        """d(continuous swivel)/dq of the arm's joints at the prepared q (central differences of the parallel-
        transported steps: defined for any S-W direction)."""
        a = self.arms[name]
        iv = self.m.arm_v[name]
        (S, JS), (E, JE), (W, JW) = self._p(a.S), self._p(a.E), self._p(a.W)
        h = 1e-5
        dS, dE, dW = (h * J[:, iv].T for J in (JS, JE, JW))
        pp = SG.steps(S, E, W, S + dS, E + dE, W + dW, a.sign)
        pm = SG.steps(S, E, W, S - dS, E - dE, W - dW, a.sign)
        return (pp - pm) / (2 * h)

    def terms(self, name: str, base, v_hand: np.ndarray, dt: float, diag: dict, N=None, tg=None,
              t: float = 0.0, hint: Optional[SG.HintInput] = None) -> list:
        """[(w, v)]: the swivel, effort, centering and joint-limit attractors of the arm ``name`` (arm joints, rad/s).
        ``tg``: the arm's target this tick (the limit-aware swivel's lookahead pose). ``hint``: the operator's swivel
        hint (``swivel_geom.HintInput``) or None."""
        cfg, m = self.cfg, self.m
        a = self.arms[name]
        iv = m.arm_v[name]
        out = []
        (S, JS), (E, JE), (W, JW) = self._p(a.S), self._p(a.E), self._p(a.W)
        psi = swivel_angle(S, E, W, a.sign)
        hs = self._hint_update(name, hint, dt) if (hint is not None or name in self.hint) else None
        alpha = hs.alpha if hs is not None else 0.0
        if cfg.w_swivel > 0 or alpha > 0:
            x, y, yaw = base
            cy, sy = math.cos(yaw), math.sin(yaw)

            def world(p):
                return np.array([x + cy * p[0] - sy * p[1], y + sy * p[0] + cy * p[1], p[2]])
            point = world(cfg.swivel_torso if cfg.swivel_dir == "torso" else cfg.swivel_head)
            u = point - W
            u = u / max(np.linalg.norm(u), 1e-9)
            if cfg.swivel_dir == "motion":
                sp = float(np.linalg.norm(v_hand))
                al = min(1.0, sp / 0.05)
                if sp > 1e-6:
                    u = (1 - al) * u + al * v_hand / sp
                    u = u / max(np.linalg.norm(u), 1e-9)
            lo, hi = math.radians(cfg.swivel_range[0]), math.radians(cfg.swivel_range[1])
            # down: the relaxed arm, the elbow hanging below the S-W line (psi 0 is the world's down projected onto
            # its normal plane), within the range; else the transmission-maximising swivel toward u
            tgt = min(max(0.0, lo), hi) if cfg.swivel_dir == "down" else swivel_target(S, E, W, u, a.sign, lo, hi,
                                                                                       cfg.swivel_beta)
            status, room = "ok", None
            if cfg.swivel_limit_aware and tg is not None:
                psi_t, status, room = self._feasible_target(name, base, tg, tgt, t)
                if status != "pending":
                    # no feasible arm angle at the target pose (this branch): hold the elbow where it is
                    tgt = psi if psi_t is None else psi + wrap(psi_t - psi)
            prev = self.psi_star.get(name)
            ps = tgt if prev is None else prev + min(1.0, dt / cfg.swivel_tau) * wrap(tgt - prev)
            self.psi_star[name] = ps
            # dψ/dq: central differences along each joint's motion of S, E, W (their Jacobians), in one batch
            h = 1e-5
            dS, dE, dW = (h * J[:, iv].T for J in (JS, JE, JW))             # (n, 3) each
            pp = swivel_angles(S + dS, E + dE, W + dW, a.sign)
            pm = swivel_angles(S - dS, E - dE, W - dW, a.sign)
            grad = np.angle(np.exp(1j * (pp - pm))) / (2 * h)
            v_model = along(N, grad, cfg.k_swivel * wrap(ps - psi))
            sd = {"psi": psi, "psi_star": ps, "status": status, "room": room, "psi_cont": self.psi_acc.get(name)}
            fr = SG.fade(SG.flexion(S, E, W), math.radians(cfg.swivel_hint_flex[0]),
                         math.radians(cfg.swivel_hint_flex[1])) if hs is not None else 0.0
            ae = alpha * fr                                  # the hint's weight in force
            if ae > 0:
                # the operator's hint replaces the model's ψ* by its weight: two attractors, (1 - ae) w_swivel and
                # ae w_swivel_hint (their weighted mean is the blend); the hint's along the continuous swivel
                rate = min(max(cfg.k_swivel_hint * (hs.target - self.psi_acc[name]), -cfg.swivel_hint_rate),
                           cfg.swivel_hint_rate)
                if cfg.w_swivel > 0 and ae < 1:
                    out.append(((1 - ae) * cfg.w_swivel, v_model))
                out.append((ae * cfg.w_swivel_hint, along(N, self.hint_grad(name), rate)))
            elif cfg.w_swivel > 0:
                out.append((cfg.w_swivel, v_model))
            if hs is not None:
                sd["hint"] = {"seq": hs.seq, "live": hs.live, "anchor": hs.psi0, "anchor_dpsi": hs.dpsi0,
                              "dpsi": hs.dpsi, "raw": hs.raw, "target": hs.target, "age": hs.age,
                              "f_elbow": hs.f_elbow, "f_conf": hs.f_conf, "blend": alpha, "f_robot": fr,
                              "weight": ae, "anchors": hs.anchors}
            diag.setdefault("swivel", {})[name] = sd
        if cfg.w_effort > 0:
            tg = self._g[iv] / a.tau_max
            eff = float(tg @ tg)
            grad = 2.0 * (self._dg[np.ix_(iv, iv)].T @ (tg / a.tau_max)) / max(eff, 1e-4)    # of log(effort)
            if N is not None:
                grad = N @ (N.T @ grad)                    # its self-motion part
            out.append((cfg.w_effort, -cfg.k_effort * grad))
        diag.setdefault("effort", {})[name] = float(np.sum((self._g[iv] / a.tau_max) ** 2))
        if cfg.w_center > 0 and len(a.center):
            lo_, hi_ = m.arm_limits(name)
            v = np.zeros(len(iv))
            qa = self._qa[name]
            mid, half = 0.5 * (lo_ + hi_), np.maximum(0.5 * (hi_ - lo_), 1e-3)
            v[a.center] = cfg.k_center * (mid - qa)[a.center] / half[a.center]
            out.append((cfg.w_center, v))
        if cfg.w_limit > 0:
            # joint-limit repulsion: within limit_band of a limit, the joint is moved away along the self-motion at
            # limit_speed s with weight w_limit s^2 (s = (band - distance) / band): exactly zero outside the band
            lo_, hi_ = m.arm_limits(name)
            qa = self._qa[name]
            for i in range(len(iv)):
                for dist, away in ((qa[i] - lo_[i], 1.0), (hi_[i] - qa[i], -1.0)):
                    if dist < cfg.limit_band:
                        s = min(1.0, (cfg.limit_band - dist) / cfg.limit_band)
                        g = np.zeros(len(iv))
                        g[i] = away
                        out.append((cfg.w_limit * s * s, along(N, g, cfg.limit_speed * s)))
        return out
