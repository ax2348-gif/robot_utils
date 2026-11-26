"""The feasible swivel interval of a redundant arm for a hand pose (wbc/humanlike.py, docs/wbc.md "Null-space
posture": the limit-aware swivel target).

For a 7-DOF arm and a hand (TCP) pose the IK solutions of one branch form a one-dimensional self-motion manifold,
parametrised here by the elbow's swivel angle psi (humanlike.swivel_angle). Computed numerically (no near-SRS
closed form, so any 7-joint arm type works): the current joints are projected onto the pose (damped least squares,
unconstrained), then the manifold is traced in both directions in ``step`` swivel steps (a step along the TCP
Jacobian's null vector, scaled by d psi / d q along it, then two DLS corrections back onto the pose) for up to
``span`` each way. Each sample is feasible when every joint is within the WBC's joint limits minus ``margin``. The
arm can move along the manifold with the hand still only inside the feasible run that contains its own swivel: that
run is the elbow's room; other runs need the hand to leave the pose (another branch), which the WBC does not do on its
own: it reports the pose unreachable from this configuration.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pinocchio as pin

from .humanlike import swivel_angle, wrap


@dataclass
class Trace:
    psi: np.ndarray            # unwrapped, ascending, the start's continuation
    feasible: np.ndarray       # bool per sample
    start: int                 # the sample of the projected current joints
    err: float                 # the projection's position error (m): large = the pose is out of this branch's reach


class SwivelPlanner:
    def __init__(self, m, ee: str, S: str, E: str, W: str, sign: float):
        """``m``: the WbcModel; ``S``, ``E``, ``W``: the model's joint names whose origins are the shoulder, elbow and
        wrist points (the arm type's geometry); ``sign``: the swivel's outward sign (humanlike.ArmGeom)."""
        model = m.model
        keep = set(m.ees[ee].joints)
        lock = [model.getJointId(model.names[i]) for i in range(1, model.njoints) if model.names[i] not in keep]
        self.model = pin.buildReducedModel(model, lock, pin.neutral(model))       # the base at its origin
        self.data = self.model.createData()
        self.tcp = self.model.getFrameId(f"{ee}_tcp")
        self.jS, self.jE, self.jW = (self.model.getJointId(n) for n in (S, E, W))
        self.sign = sign
        self.iq = [self.model.joints[self.model.getJointId(j)].idx_q for j in m.ees[ee].joints]

    def _fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        d = self.data
        return d.oMf[self.tcp], swivel_angle(d.oMi[self.jS].translation, d.oMi[self.jE].translation,
                                             d.oMi[self.jW].translation, self.sign)

    def _correct(self, q, T_goal, iters):
        for _ in range(iters):
            T, _ = self._fk(q)
            e = np.concatenate([T_goal.translation - T.translation, pin.log3(T_goal.rotation @ T.rotation.T)])
            pin.computeJointJacobians(self.model, self.data, q)
            J = pin.getFrameJacobian(self.model, self.data, self.tcp, pin.LOCAL_WORLD_ALIGNED)
            q = q + J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
        T, psi = self._fk(q)
        return q, float(np.linalg.norm(T_goal.translation - T.translation)), psi

    def trace(self, q_arm: np.ndarray, T_goal: pin.SE3, lo: np.ndarray, hi: np.ndarray, margin: float,
              step: float = math.radians(10), span: float = math.radians(180)) -> Optional[Trace]:
        """``T_goal``: the TCP pose in the base frame. None when the joints do not project onto the pose."""
        q0 = pin.neutral(self.model)
        q0[self.iq] = q_arm
        q0, err, psi0 = self._correct(q0, T_goal, 8)
        if err > 0.01:
            return Trace(np.array([psi0]), np.array([False]), 0, err)

        def ok(q):
            a = q[self.iq]
            return bool(np.all(a >= lo + margin) and np.all(a <= hi - margin))
        samples = [(0.0, ok(q0))]
        for direction in (1.0, -1.0):
            q, psi, acc = q0.copy(), psi0, 0.0
            while acc < span - 1e-9:
                pin.computeJointJacobians(self.model, self.data, q)
                J = pin.getFrameJacobian(self.model, self.data, self.tcp, pin.LOCAL_WORLD_ALIGNED)
                n = np.linalg.svd(J)[2][-1]
                h = 1e-4
                _, p1 = self._fk(q + h * n)
                dpsi = wrap(p1 - psi) / h
                if abs(dpsi) < 1e-3:
                    break                                       # the swivel does not move along the null space
                dq = n * (direction * step / dpsi)
                if np.max(np.abs(dq)) > 0.5:
                    dq *= 0.5 / np.max(np.abs(dq))
                q, err, p_new = self._correct(q + dq, T_goal, 2)
                if err > 0.003:
                    break
                acc += abs(wrap(p_new - psi))
                psi = p_new
                samples.append((direction * acc, ok(q)))
        samples.sort(key=lambda s: s[0])
        psis = np.array([psi0 + s[0] for s in samples])
        feas = np.array([s[1] for s in samples])
        return Trace(psis, feas, int(np.argmin(np.abs(psis - psi0))), 0.0)


def choose(tr: Optional[Trace], psi_pref: float, edge: float = math.radians(5)) -> tuple:
    """((low, high) the elbow's room: the feasible run the arm is in, ``edge`` inside its ends, or the nearest feasible
    run when the arm's own sample is infeasible; status ok | re-posing) or (None, unreachable) when no sample is
    feasible (or the pose is out of reach). ``psi_pref`` is not used to pick the run (the arm stays in its own)."""
    if tr is None or not tr.feasible.any():
        return None, "unreachable"
    i = tr.start
    status = "ok"
    if not tr.feasible[i]:
        cand = np.where(tr.feasible)[0]
        i = int(cand[np.argmin(np.abs(cand - tr.start))])
        status = "re-posing"
    a = b = i
    while a > 0 and tr.feasible[a - 1]:
        a -= 1
    while b < len(tr.psi) - 1 and tr.feasible[b + 1]:
        b += 1
    lo_, hi_ = float(tr.psi[a]), float(tr.psi[b])
    if hi_ - lo_ > 2 * edge:
        lo_, hi_ = lo_ + edge, hi_ - edge
    else:
        lo_ = hi_ = 0.5 * (lo_ + hi_)
    return (lo_, hi_), status
