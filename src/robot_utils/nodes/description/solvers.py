"""Calibration maths (pure numpy, no I/O).

Frames: ``base`` is on the floor at the centre of the wheels, x forward, y left, z up. The camera
frame is the camera node's ``depth_optical`` (x right, y down, z out of the lens). A calibration
estimates the pose of ``depth_optical`` in ``base``: ``p_base = R p_cam + t``.

- ``fit_ground_plane``: robust plane through floor points (camera frame) -> floor normal (pointing
  from the floor towards the camera) and the camera's height above the floor.
- ``fit_motion_line``: time-free line fit of accelerometer samples taken while the base translates
  back and forth along one direction: the line's direction is that direction in the IMU frame, its
  centroid the static specific force (gravity reaction + bias). The sign is fixed afterwards by the
  correlation with a +/-1 label (the measured base acceleration's sign); the label is never part of
  the fit.
- ``solve_rotation``: Wahba over the available vector pairs (floor normal <-> base z, motion
  directions <-> their base vectors, plus a weak prior) -> R.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .frames import wahba


# -- ground plane -------------------------------------------------------------------------------

def unproject(depth_m: np.ndarray, K: dict, mask: Optional[np.ndarray] = None, stride: int = 1) -> np.ndarray:
    """(N, 3) camera-frame points (m) from a depth image (m; 0/NaN = invalid) and pinhole intrinsics
    scaled to the image size."""
    h, w = depth_m.shape
    sx, sy = w / K["width"], h / K["height"]
    fx, fy, cx, cy = K["fx"] * sx, K["fy"] * sy, K["cx"] * sx, K["cy"] * sy
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth_m[::stride, ::stride]
    keep = np.isfinite(z) & (z > 0)
    if mask is not None:
        keep &= mask[::stride, ::stride]
    u, v, z = u[keep].astype(float), v[keep].astype(float), z[keep]
    return np.column_stack([(u - cx) / fx * z, (v - cy) / fy * z, z])


def polygon_mask(shape: tuple, polygons: list) -> np.ndarray:
    """Boolean mask of the union of polygons given as [[x, y], ...] in normalised image coords (0..1)."""
    h, w = shape
    yy, xx = np.mgrid[0:h, 0:w]
    px, py = (xx + 0.5) / w, (yy + 0.5) / h
    mask = np.zeros(shape, dtype=bool)
    for poly in polygons:
        P = np.asarray(poly, dtype=float)
        if len(P) < 3:
            continue
        inside = np.zeros(shape, dtype=bool)
        j = len(P) - 1
        for i in range(len(P)):           # even-odd ray casting, vectorised over pixels
            xi, yi, xj, yj = P[i, 0], P[i, 1], P[j, 0], P[j, 1]
            cross = ((yi > py) != (yj > py)) & (px < (xj - xi) * (py - yi) / ((yj - yi) or 1e-12) + xi)
            inside ^= cross
            j = i
        mask |= inside
    return mask


def _rows_dot(P: np.ndarray, n: np.ndarray) -> np.ndarray:
    """P @ n for (N, 3) rows (elementwise: Accelerate's matmul raises spurious FP warnings on macOS)."""
    return (P * n).sum(1)


def _plane_svd(P: np.ndarray) -> tuple[np.ndarray, float]:
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    n = vt[2]
    return n, -float(n @ c)


def fit_ground_plane(P: np.ndarray, *, gravity_up: Optional[np.ndarray] = None, max_tilt_deg: float = 30.0,
                     iters: int = 300, tol: float = 0.01, seed: int = 0) -> dict:
    """RANSAC + least-squares refit. ``gravity_up`` (camera frame, pointing up) restricts candidate
    planes to within ``max_tilt_deg`` of horizontal and below the camera (automatic mode).
    Returns normal (towards the camera), height (m), rms (m), inliers, n_points."""
    P = np.asarray(P, dtype=float)
    if len(P) < 50:
        raise ValueError(f"only {len(P)} valid points; need at least 50")
    rng = np.random.default_rng(seed)
    up = None if gravity_up is None else np.asarray(gravity_up, float) / np.linalg.norm(gravity_up)
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        d = -float(n @ a)
        if d < 0:
            n, d = -n, -d                   # normal points to the camera's side (origin)
        if up is not None and (float(n @ up) < math.cos(math.radians(max_tilt_deg))):
            continue
        inl = np.abs(_rows_dot(P, n) + d) < tol
        k = int(inl.sum())
        if k > best_n:
            best, best_n = inl, k
    if best is None or best_n < 30:
        raise ValueError("no plane found" + (" near horizontal below the camera" if up is not None else ""))
    n, d = _plane_svd(P[best])
    for _ in range(2):                      # refit on the refit's inliers
        inl = np.abs(_rows_dot(P, n) + d) < tol
        n, d = _plane_svd(P[inl])
    if d < 0:
        n, d = -n, -d
    res = _rows_dot(P[inl], n) + d
    out = {"normal": n.tolist(), "height": d, "rms": float(np.sqrt(np.mean(res ** 2))),
           "inliers": int(inl.sum()), "n_points": int(len(P))}
    if up is not None:
        out["angle_to_gravity_deg"] = math.degrees(math.acos(max(-1.0, min(1.0, float(n @ up)))))
    return out


# -- motion line (accelerometer) ------------------------------------------------------------------

def lowpass(x: np.ndarray, t: np.ndarray, fc: float) -> np.ndarray:
    """Zero-phase Gaussian low-pass of (N, ...) samples at times ``t`` (s; ~uniform), -3 dB near ``fc`` Hz.

    sigma = 1 / (2 pi fc): gain exp(-(f/fc)^2 / 2), so 0.88 at fc/2 and 0.61 at fc. Edges are
    normalised by the kernel weight inside the data (no zero-padding bias)."""
    x = np.asarray(x, dtype=float)
    if len(x) < 3 or fc <= 0:
        return x.copy()
    dt = float(np.median(np.diff(t)))
    s = 1.0 / (2 * np.pi * fc) / dt                       # sigma in samples
    h = max(1, int(np.ceil(3 * s)))
    w = np.exp(-0.5 * (np.arange(-h, h + 1) / s) ** 2)
    norm = np.convolve(np.ones(len(x)), w, mode="same")
    flat = x.reshape(len(x), -1)
    out = np.column_stack([np.convolve(flat[:, j], w, mode="same") / norm for j in range(flat.shape[1])])
    return out.reshape(x.shape)


def fit_motion_line(accel: np.ndarray, label: Optional[np.ndarray] = None) -> dict:
    """Principal axis of (N, 3) accelerometer samples (m/s^2, IMU frame).

    ``label`` (N,) = +/-1 sign of the base's acceleration along the commanded direction, sampled
    at the same instants (coarse sync is enough); it only decides the sign of the axis.
    Returns direction (unit), centroid, amplitude (rms along the line), linearity
    (sqrt(lambda2/lambda1): 0 = a perfect line), sign_corr (normalised correlation with the label)."""
    A = np.asarray(accel, dtype=float)
    c = A.mean(0)
    X = A - c
    w, V = np.linalg.eigh(np.einsum("ni,nj->ij", X, X) / len(X))
    d = V[:, 2]
    s = (X * d).sum(1)   # (einsum / elementwise: Accelerate's matmul raises spurious FP warnings on macOS)
    out = {"centroid": c.tolist(), "amplitude": float(np.sqrt(w[2])),
           "linearity": float(np.sqrt(max(w[1], 0.0) / w[2])) if w[2] > 0 else float("inf")}
    corr = None
    if label is not None:
        L = np.asarray(label, dtype=float)
        ok = np.isfinite(L) & (L != 0)
        if ok.sum() > 10:
            corr = float(np.mean(s[ok] * L[ok]) / (np.std(s[ok]) or 1.0))
            if corr < 0:
                d, corr = -d, -corr
    out["direction"] = d.tolist()
    out["sign_corr"] = corr
    return out


# -- rotation from vector pairs -------------------------------------------------------------------

def solve_rotation(pairs: list[tuple]) -> tuple[np.ndarray, dict]:
    """Rotation base <- camera from (camera-frame vector, base-frame vector, weight) pairs.
    Returns R and per-pair residual angles (deg)."""
    body = [np.asarray(b, float) / np.linalg.norm(b) for b, _, _ in pairs]
    ref = [np.asarray(r, float) / np.linalg.norm(r) for _, r, _ in pairs]
    R = wahba(body, ref, [w for _, _, w in pairs])
    res = [math.degrees(math.acos(max(-1.0, min(1.0, float(r @ (R @ b)))))) for b, r in zip(body, ref)]
    return R, {"residual_deg": [round(x, 4) for x in res]}


def angle_deg(a, b) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))))))
