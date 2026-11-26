"""The elbow's swivel without a fixed zero (numpy only; the operator's ``dpsi`` in the swivel hint is meant to be the
same measure taken on their arm).

The swivel is the elbow's rotation about the shoulder-wrist line. A fixed zero (the world's down projected normal to
that line, as humanlike.py's ``swivel_angle``) is singular when the line is vertical (reaching straight down). Here
the swivel is only ever measured as an **increment**: between two samples, the elbow's offset from the S-W line at
the first sample is parallel-transported to the second sample's line (the smallest rotation taking one line onto the
other) and the signed angle from it to the second offset, about the second line, is the step. Accumulated, the steps
give a continuous swivel with an arbitrary origin, which only differences of are used (the relative hint: docs/wbc.md
"The operator's elbow: the swivel hint").

``sign``: +1 for an arm on the +y (left) side, -1 on the -y (right) side, as ``humanlike.swivel_angle``: a positive
step is the elbow moving out, away from the body, for either arm (and for the operator's arms, side by side).

``flexion``: pi minus the upper-arm-forearm angle (0 at full extension, where the swivel is undefined); ``fade`` maps
it (or a confidence) to a weight that goes to zero smoothly below a threshold.
"""
from __future__ import annotations

import math
from typing import NamedTuple, Optional

import numpy as np

EPS_BEND = 1e-3            # a bend below this: no step is measured (the elbow on the S-W line)


def _unit(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def offsets(S, E, W):
    """(a, n): the unit S-W axis and the elbow's offset from the S-W line (perpendicular to a), rows or single."""
    S, E, W = (np.asarray(x, float) for x in (S, E, W))
    a = _unit(W - S)
    p = E - S
    n = p - np.sum(p * a, axis=-1, keepdims=True) * a
    return a, n


def flexion(S, E, W) -> float:
    """The elbow's flexion (rad): pi - the angle at E between the upper arm (E->S) and the forearm (E->W)."""
    u = np.asarray(S, float) - np.asarray(E, float)
    f = np.asarray(W, float) - np.asarray(E, float)
    c = float(u @ f) / max(float(np.linalg.norm(u) * np.linalg.norm(f)), 1e-12)
    return math.pi - math.acos(min(max(c, -1.0), 1.0))


def fade(b: float, lo: float, hi: float) -> float:
    """Smoothstep: 0 below lo, 1 above hi (C1: zero slope at both ends)."""
    if hi <= lo:
        return 1.0 if b >= hi else 0.0
    u = min(max((b - lo) / (hi - lo), 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def transport(v, a0, a1):
    """v (normal to a0) carried to a1's normal plane by the smallest rotation taking a0 onto a1 (rows or single)."""
    c = np.sum(a0 * a1, axis=-1, keepdims=True)
    k = np.cross(a0, a1)
    # Rodrigues with the unnormalised axis k = a0 x a1 (|k| = sin): v cos + (k x v) + k (k.v) / (1 + cos)
    return v * c + np.cross(k, v) + k * np.sum(k * v, axis=-1, keepdims=True) / np.maximum(1.0 + c, 1e-9)


def steps(S0, E0, W0, S1, E1, W1, sign: float = 1.0) -> np.ndarray:
    """The swivel steps (rad) from sample 0 to sample(s) 1 (rows broadcast against the single sample 0); 0 where
    either elbow is (nearly) on its S-W line."""
    a0, n0 = offsets(S0, E0, W0)
    a1, n1 = offsets(S1, E1, W1)
    t = transport(n0, a0, a1)
    s = np.sum(a1 * np.cross(t, n1), axis=-1)
    c = np.sum(t * n1, axis=-1)
    L0 = np.linalg.norm(np.asarray(E0, float) - np.asarray(S0, float))
    L1 = np.linalg.norm(np.asarray(E1, float) - np.asarray(S1, float), axis=-1)
    ok = (np.linalg.norm(n0, axis=-1) > EPS_BEND * np.maximum(L0, 1e-9)) & \
         (np.linalg.norm(n1, axis=-1) > EPS_BEND * np.maximum(L1, 1e-9))
    return np.where(ok, sign * np.arctan2(s, c), 0.0)


def step(S0, E0, W0, S1, E1, W1, sign: float = 1.0) -> float:
    return float(steps(S0, E0, W0, S1, E1, W1, sign))


class Accumulator:
    """A continuous swivel: the sum of the steps between successive samples (an arbitrary origin; 0 at the first)."""

    def __init__(self, sign: float = 1.0):
        self.sign = sign
        self.psi = 0.0
        self.last: Optional[tuple] = None

    def reset(self) -> None:
        self.psi, self.last = 0.0, None

    def update(self, S, E, W) -> float:
        cur = tuple(np.array(x, float) for x in (S, E, W))
        if self.last is not None:
            d = step(*self.last, *cur, self.sign)
            if math.isfinite(d):
                self.psi += d
        self.last = cur
        return self.psi


class HintInput(NamedTuple):
    """The operator's swivel hint as the controller gets it (a target datagram's per-EE ``swivel``, docs/wbc.md "The
    operator's elbow"): the engagement id, the operator's swivel change since it (rad, mirror-adjusted), their elbow
    angle (rad, pi = straight), the confidence (0..1), and the s since it arrived."""
    seq: int
    dpsi: float
    elbow: float
    conf: float
    age: float
