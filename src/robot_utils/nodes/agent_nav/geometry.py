"""agent_nav geometry: selections, unprojection, region / plane queries, overlays, BEV, path check.

Pure (numpy + PIL), no I/O. Everything here is deterministic math the agent should not redo by
hand; every decision stays with the agent.

Frames:
- image pixels (u, v): full-resolution colour image (what the agent sees), origin top left;
- colour optical: x right, y down, z forward (m);
- base: floor level, wheel centre, x forward, y left, z up, at the snapshot's capture time;
- world: the estimator's frame; ``T_wb`` = the base pose at capture.
"""
from __future__ import annotations

import io
import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from ..swerve.constants import FRAME


def _quiet(fn):
    """NaN depth (no return) flows through the products; masks handle it, so mute the warnings."""
    import functools

    @functools.wraps(fn)
    def w(*a, **k):
        with np.errstate(all="ignore"):
            return fn(*a, **k)
    return w

HALF = FRAME / 2                       # m, half the base's square frame
FOOT_MARGIN = 0.05                     # m added around the footprint for path checks
OBST_Z = (0.04, 1.6)                   # m: points in this height band are obstacles (below: floor)
MAX_RANGE = 8.0                        # m: farther depth is ignored
SEL_COLOR = (255, 0, 200)              # magenta: selections
OK_COLOR = (0, 200, 90)
WARN_COLOR = (255, 60, 40)


# ----------------------------------------------------------------------------------------------
# Snapshot data
# ----------------------------------------------------------------------------------------------

@dataclass
class Camera:
    """Intrinsics of the registered depth grid + its scale to the full colour image."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist: dict                 # k1..k6, p1, p2 (OpenCV rational / Brown-Conrady)
    scale: float               # grid pixels per colour pixel (e.g. 0.5)
    color_w: int
    color_h: int

    @classmethod
    def from_color(cls, color: dict, grid_w: int, grid_h: int) -> "Camera":
        """Grid intrinsics for any registration scale: the colour intrinsics scaled about pixel
        centres (what 3d_perception uses: c' = (c + 0.5) s - 0.5)."""
        s = grid_w / color["width"]
        return cls(color["fx"] * s, color["fy"] * s, (color["cx"] + 0.5) * s - 0.5, (color["cy"] + 0.5) * s - 0.5,
                   int(grid_w), int(grid_h), color.get("distortion") or {}, s, int(color["width"]), int(color["height"]))

    @classmethod
    def from_calibration(cls, cal: dict) -> "Camera":
        g, c = cal["depth_grid"], cal["color"]
        return cls(g["fx"], g["fy"], g["cx"], g["cy"], int(g["width"]), int(g["height"]),
                   g.get("distortion") or {}, g["width"] / c["width"], int(c["width"]), int(c["height"]))


_rays_cache: dict = {}


def rays(cam: Camera) -> np.ndarray:
    """(H, W, 3) unit-z rays (x_n, y_n, 1) of every grid pixel, lens distortion removed (iterative)."""
    key = (cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height, tuple(sorted(cam.dist.items())))
    r = _rays_cache.get(key)
    if r is not None:
        return r
    d = {k: float(cam.dist.get(k, 0.0)) for k in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")}
    u, v = np.meshgrid(np.arange(cam.width, dtype=np.float64), np.arange(cam.height, dtype=np.float64))
    xd, yd = (u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy
    x, y = xd.copy(), yd.copy()
    for _ in range(8):
        r2 = x * x + y * y
        rad = (1 + d["k1"] * r2 + d["k2"] * r2 ** 2 + d["k3"] * r2 ** 3) / \
              (1 + d["k4"] * r2 + d["k5"] * r2 ** 2 + d["k6"] * r2 ** 3)
        dx = 2 * d["p1"] * x * y + d["p2"] * (r2 + 2 * x * x)
        dy = d["p1"] * (r2 + 2 * y * y) + 2 * d["p2"] * x * y
        x, y = (xd - dx) / rad, (yd - dy) / rad
    r = np.stack([x, y, np.ones_like(x)], -1)
    _rays_cache.clear()
    _rays_cache[key] = r
    return r


@_quiet
def project(cam: Camera, p_c: np.ndarray) -> np.ndarray:
    """Colour-optical points (N, 3) -> full-resolution colour pixels (N, 2) (with distortion)."""
    d = {k: float(cam.dist.get(k, 0.0)) for k in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")}
    z = np.where(p_c[:, 2] > 1e-6, p_c[:, 2], np.nan)
    x, y = p_c[:, 0] / z, p_c[:, 1] / z
    r2 = x * x + y * y
    rad = (1 + d["k1"] * r2 + d["k2"] * r2 ** 2 + d["k3"] * r2 ** 3) / (1 + d["k4"] * r2 + d["k5"] * r2 ** 2 + d["k6"] * r2 ** 3)
    xd = x * rad + 2 * d["p1"] * x * y + d["p2"] * (r2 + 2 * x * x)
    yd = y * rad + d["p1"] * (r2 + 2 * y * y) + 2 * d["p2"] * x * y
    ug, vg = cam.fx * xd + cam.cx, cam.fy * yd + cam.cy
    return np.stack([ug / cam.scale, vg / cam.scale], -1)


def pose_matrix(x: float, y: float, theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.0]])


class Cloud:
    """A snapshot's registered depth as points in colour, base and world frames (lazy)."""

    def __init__(self, depth_mm: np.ndarray, cam: Camera, T_bc: Optional[np.ndarray],
                 pose: Optional[tuple[float, float, float]]):
        self.cam, self.T_bc, self.pose = cam, T_bc, pose
        z = depth_mm.astype(np.float64) / 1000.0
        z[(z <= 0) | (z > MAX_RANGE)] = np.nan
        self.z = z                                          # (H, W) m, NaN = no depth
        self.p_c = rays(cam) * z[..., None]                 # (H, W, 3) colour optical
        self._p_b = None

    @property
    def calibrated(self) -> bool:
        return self.T_bc is not None

    @property
    @_quiet
    def p_b(self) -> Optional[np.ndarray]:
        """(H, W, 3) base frame (None without a calibrated camera pose)."""
        if self.T_bc is None:
            return None
        if self._p_b is None:
            R, t = self.T_bc[:3, :3], self.T_bc[:3, 3]
            self._p_b = self.p_c @ R.T + t
        return self._p_b

    @_quiet
    def to_world(self, p_b: np.ndarray) -> Optional[np.ndarray]:
        if self.pose is None:
            return None
        T = pose_matrix(*self.pose)
        return p_b @ T[:3, :3].T + T[:3, 3]

    def base_to_world_xy(self, x: float, y: float) -> Optional[tuple[float, float]]:
        if self.pose is None:
            return None
        px, py, th = self.pose
        c, s = math.cos(th), math.sin(th)
        return px + c * x - s * y, py + s * x + c * y


# ----------------------------------------------------------------------------------------------
# Selections (full-resolution colour pixels)
# ----------------------------------------------------------------------------------------------

def _shape_mask(shape: dict, W: int, H: int, scale: float) -> np.ndarray:
    """Boolean (H, W) grid mask of one selection shape given in colour pixels."""
    img = Image.new("1", (W, H), 0)
    dr = ImageDraw.Draw(img)
    s = scale
    if "point" in shape:
        u, v = shape["point"]
        r = max(1.5, 3 * s)
        dr.ellipse([u * s - r, v * s - r, u * s + r, v * s + r], fill=1)
    elif "disk" in shape:
        u, v, r = shape["disk"]
        dr.ellipse([(u - r) * s, (v - r) * s, (u + r) * s, (v + r) * s], fill=1)
    elif "box" in shape:
        u0, v0, u1, v1 = shape["box"]
        dr.rectangle([min(u0, u1) * s, min(v0, v1) * s, max(u0, u1) * s, max(v0, v1) * s], fill=1)
    elif "polygon" in shape:
        pts = [(float(u) * s, float(v) * s) for u, v in shape["polygon"]]
        if len(pts) < 3:
            raise ValueError("a polygon needs at least 3 points")
        dr.polygon(pts, fill=1)
    else:
        raise ValueError("a selection is one of {point: [u, v]}, {disk: [u, v, r]}, {box: [u0, v0, u1, v1]}, "
                         "{polygon: [[u, v], ...]}")
    return np.array(img, dtype=bool)


def selection_mask(select: dict, exclude: Sequence[dict], cam: Camera) -> np.ndarray:
    m = _shape_mask(select, cam.width, cam.height, cam.scale)
    for e in exclude or ():
        m &= ~_shape_mask(e, cam.width, cam.height, cam.scale)
    return m


def check_shape(shape: dict, cam: Camera) -> None:
    """Raise ValueError for malformed shapes or shapes entirely outside the image."""
    keys = [k for k in ("point", "disk", "box", "polygon") if k in shape]
    if len(keys) != 1:
        raise ValueError("a selection has exactly one of point, disk, box, polygon")
    k = keys[0]
    n = {"point": 2, "disk": 3, "box": 4}.get(k)
    vals = shape[k]
    if n is not None and (len(vals) != n or not all(isinstance(x, (int, float)) for x in vals)):
        raise ValueError(f"{k} takes {n} numbers")
    if k == "polygon" and (len(vals) < 3 or any(len(p) != 2 for p in vals)):
        raise ValueError("polygon takes at least 3 [u, v] points")


# ----------------------------------------------------------------------------------------------
# Queries
# ----------------------------------------------------------------------------------------------

def _pct(a: np.ndarray, q=(5, 50, 95)) -> list:
    return [round(float(x), 3) for x in np.percentile(a, q)] if len(a) else None


@_quiet
def region_stats(cloud: Cloud, mask: np.ndarray) -> dict:
    n_sel = int(mask.sum())
    valid = mask & np.isfinite(cloud.z)
    out = {"pixels": n_sel, "with_depth": int(valid.sum()),
           "depth_fraction": round(float(valid.sum()) / n_sel, 3) if n_sel else 0.0}
    if not valid.any():
        out["warning"] = "no depth in the selection"
        return out
    pc = cloud.p_c[valid]
    rng = np.linalg.norm(pc, axis=1)
    out["range_m"] = _pct(rng)
    spread = float(np.percentile(rng, 95) - np.percentile(rng, 5))
    out["range_spread_m"] = round(spread, 3)
    if spread > 0.4:
        out.setdefault("warnings", []).append(
            f"MIXED: the selection's distances span {spread:.2f} m (5-95%): it likely includes floor or background "
            "pixels. Use a polygon or a smaller selection inside the object, or exclude the rest")
    out["touches_image_edge"] = bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())
    pb = cloud.p_b
    if pb is not None:
        b = pb[valid]
        out["base"] = {"x": _pct(b[:, 0]), "y": _pct(b[:, 1]), "z": _pct(b[:, 2]),
                       "centroid": [round(float(v), 3) for v in np.median(b, 0)],
                       "nearest_xy": [round(float(v), 3) for v in b[np.argmin(np.hypot(b[:, 0], b[:, 1]))]]}
        w = cloud.to_world(b)
        if w is not None:
            out["world"] = {"x": _pct(w[:, 0]), "y": _pct(w[:, 1]), "z": _pct(w[:, 2]),
                            "centroid": [round(float(v), 3) for v in np.median(w, 0)]}
    else:
        out["warning"] = "camera pose in base not calibrated: colour-frame results only"
        out["color_optical_centroid"] = [round(float(v), 3) for v in np.median(pc, 0)]
    return out


@_quiet
def fit_plane(cloud: Cloud, mask: np.ndarray, iters: int = 200, thresh: float = 0.02,
              rng: Optional[np.random.Generator] = None) -> dict:
    """Robust plane (RANSAC + least squares) through the selected points, in the base frame."""
    pb = cloud.p_b
    if pb is None:
        raise ValueError("plane fit needs a calibrated camera pose in base")
    valid = mask & np.isfinite(cloud.z)
    P = pb[valid]
    if len(P) < 30:
        raise ValueError(f"only {len(P)} points with depth in the selection (need >= 30)")
    rng = rng or np.random.default_rng(0)
    S = P[rng.choice(len(P), min(len(P), 6000), replace=False)]
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = S[rng.choice(len(S), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        k = int((np.abs((S - a) @ n) < thresh).sum())
        if k > best_n:
            best, best_n = (a, n), k
    a, n = best
    inl = np.abs((P - a) @ n) < thresh
    Q = P[inl]
    c = Q.mean(0)
    _, sv, vt = np.linalg.svd(Q - c, full_matrices=False)
    n = vt[2]
    cam_pos = cloud.T_bc[:3, 3]
    if n @ (cam_pos - c) < 0:
        n = -n                                     # normal toward the camera
    vertical = abs(n[2]) < 0.5
    up = np.array([0, 0, 1.0])
    if vertical:
        e1 = np.cross(up, n)
        e1 /= np.linalg.norm(e1)                   # horizontal, in the plane
        e2 = np.cross(n, e1)                       # up-ish, in the plane
    else:
        e1, e2 = vt[0], vt[1]
    s1, s2 = (Q - c) @ e1, (Q - c) @ e2
    lo1, hi1 = np.percentile(s1, [1, 99])
    lo2, hi2 = np.percentile(s2, [1, 99])
    centre = c + e1 * (lo1 + hi1) / 2 + e2 * (lo2 + hi2) / 2          # middle of the extent
    inl_mask = np.zeros_like(valid)
    inl_mask[valid] = inl
    touches = bool(inl_mask[:2].any() or inl_mask[-2:].any() or inl_mask[:, :2].any() or inl_mask[:, -2:].any())
    nh = n[:2] / (np.linalg.norm(n[:2]) or 1.0)
    out = {
        "points": int(len(P)), "inliers": int(inl.sum()), "inlier_fraction": round(float(inl.mean()), 3),
        "rms_m": round(float(sv[2] / math.sqrt(len(Q))), 4),
        "orientation": "vertical" if vertical else ("horizontal" if abs(n[2]) > 0.9 else "inclined"),
        "tilt_from_vertical_deg": round(float(math.degrees(math.asin(min(1.0, abs(n[2]))))), 1),
        "base": {"centre": [round(float(v), 3) for v in centre], "normal": [round(float(v), 3) for v in n],
                 "facing_heading": round(float(math.atan2(nh[1], nh[0])), 3)},
        "width_m": round(float(hi1 - lo1), 3), "height_m": round(float(hi2 - lo2), 3),
        "touches_image_edge": touches,
        "corners_base": [[round(float(v), 3) for v in centre + e1 * a1 + e2 * a2]
                         for a1, a2 in (((lo1 - hi1) / 2, (lo2 - hi2) / 2), ((hi1 - lo1) / 2, (lo2 - hi2) / 2),
                                        ((hi1 - lo1) / 2, (hi2 - lo2) / 2), ((lo1 - hi1) / 2, (hi2 - lo2) / 2))],
    }
    warns = []
    if touches:
        warns.append("the fitted surface reaches the image edge: it may extend beyond the view, so its centre "
                     "and width may be biased")
    if inl.mean() < 0.8 or out["rms_m"] > 0.015:
        warns.append(f"MIXED: only {100 * inl.mean():.0f}% of the selected points lie on the plane (rms "
                     f"{out['rms_m'] * 100:.1f} cm): the selection likely includes other surfaces. Trace a polygon "
                     "inside the surface, or exclude the rest")
    if warns:
        out["warning"] = "; ".join(warns)
    if cloud.pose is not None:
        w = cloud.to_world(centre[None])[0]
        th = cloud.pose[2]
        nw = np.array([math.cos(th) * n[0] - math.sin(th) * n[1], math.sin(th) * n[0] + math.cos(th) * n[1], n[2]])
        out["world"] = {"centre": [round(float(v), 3) for v in w], "normal": [round(float(v), 3) for v in nw],
                        "facing_heading": round(float(math.atan2(nw[1], nw[0])), 3)}
    out["_inlier_mask"] = inl_mask
    return out


@_quiet
def point_samples(cloud: Cloud, points_px: Sequence[Sequence[float]], radius: float = 4.0) -> list:
    """Base-frame point (median of a small disk) for each colour pixel; None where there is no depth."""
    out = []
    for u, v in points_px:
        m = _shape_mask({"disk": [float(u), float(v), radius]}, cloud.cam.width, cloud.cam.height, cloud.cam.scale)
        m &= np.isfinite(cloud.z)
        out.append(None if not m.any() else np.median(cloud.p_b[m], 0))
    return out


@_quiet
def fit_line(cloud: Cloud, mask: Optional[np.ndarray] = None, points_px: Optional[Sequence] = None,
             thresh: float = 0.02, rng: Optional[np.random.Generator] = None) -> dict:
    """Robust 3D line (base frame) through a selection's points or through pixel points (an edge,
    a row of legs, a cable on the floor). Direction is oriented from the first point / left to right."""
    if cloud.p_b is None:
        raise ValueError("line fit needs a calibrated camera pose in base")
    per_point = None
    if points_px is not None:
        if len(points_px) < 2:
            raise ValueError("a line needs at least 2 points")
        pts = point_samples(cloud, points_px)
        per_point = [None if p is None else [round(float(v), 3) for v in p] for p in pts]
        P = np.array([p for p in pts if p is not None])
        if len(P) < 2:
            raise ValueError("fewer than 2 of the points have depth")
    else:
        valid = mask & np.isfinite(cloud.z)
        P = cloud.p_b[valid]
        if len(P) < 10:
            raise ValueError(f"only {len(P)} points with depth in the selection (need >= 10)")
    rng = rng or np.random.default_rng(0)
    inl = np.ones(len(P), bool)
    if len(P) >= 8:
        S = P[rng.choice(len(P), min(len(P), 4000), replace=False)]
        best, best_n = None, -1
        for _ in range(150):
            a, b = S[rng.choice(len(S), 2, replace=False)]
            d = b - a
            nd = np.linalg.norm(d)
            if nd < 1e-6:
                continue
            d /= nd
            r = np.linalg.norm(np.cross(S - a, d), axis=1)
            k = int((r < thresh).sum())
            if k > best_n:
                best, best_n = (a, d), k
        a, d = best
        inl = np.linalg.norm(np.cross(P - a, d), axis=1) < thresh
    Q = P[inl]
    c = Q.mean(0)
    _, sv, vt = np.linalg.svd(Q - c, full_matrices=False)
    d = vt[0]
    if points_px is not None:
        first = P[0]
        if (P[-1] - first) @ d < 0:
            d = -d
    elif d[1] > 0:                                       # selection: point to the robot's right (-y) ... left to right in the image
        d = -d
    t = (Q - c) @ d
    lo, hi = (np.percentile(t, [1, 99]) if len(Q) > 10 else (t.min(), t.max()))
    p0, p1 = c + d * lo, c + d * hi
    resid = np.linalg.norm(np.cross(Q - c, d), axis=1)
    rms = float(np.sqrt((resid ** 2).mean()))
    dh = d[:2] / (np.linalg.norm(d[:2]) or 1.0)
    out = {"points": int(len(P)), "inliers": int(inl.sum()), "inlier_fraction": round(float(inl.mean()), 3),
           "rms_m": round(rms, 4), "length_m": round(float(hi - lo), 3),
           "tilt_from_horizontal_deg": round(float(math.degrees(math.asin(min(1.0, abs(d[2]))))), 1),
           "base": {"centre": [round(float(v), 3) for v in (p0 + p1) / 2], "direction": [round(float(v), 3) for v in d],
                    "heading": round(float(math.atan2(dh[1], dh[0])), 4),
                    "ends": [[round(float(v), 3) for v in p0], [round(float(v), 3) for v in p1]]}}
    if per_point is not None:
        out["per_point_base"] = per_point
    if cloud.pose is not None:
        th = cloud.pose[2]
        out["world"] = {"centre": [round(float(v), 3) for v in cloud.to_world(((p0 + p1) / 2)[None])[0]],
                        "heading": round(float(math.atan2(dh[1], dh[0]) + th), 4),
                        "ends": [[round(float(v), 3) for v in cloud.to_world(e[None])[0]] for e in (p0, p1)]}
    warns = []
    if len(P) >= 8 and (inl.mean() < 0.8 or rms > 0.02):
        warns.append(f"MIXED: only {100 * inl.mean():.0f}% of the points lie on one line (rms {rms * 100:.1f} cm): "
                     "the selection likely spans several things. Use points / disks on the edge itself")
    if points_px is not None and rms > 0.03:
        warns.append(f"the points are not collinear (rms {rms * 100:.1f} cm): check that each lies on the edge")
    if warns:
        out["warning"] = "; ".join(warns)
    out["_ends_base"] = (p0, p1)
    return out


# ----------------------------------------------------------------------------------------------
# Path check (swept footprint against the snapshot's obstacle points)
# ----------------------------------------------------------------------------------------------

def _footprints(start, goal, step=0.05):
    """Sampled poses along a straight move with the heading interpolated (world)."""
    (x0, y0, t0), (x1, y1, t1) = start, goal
    n = max(2, int(max(math.hypot(x1 - x0, y1 - y0) / step, abs(t1 - t0) / 0.1)) + 1)
    s = np.linspace(0, 1, n)
    return np.stack([x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, t0 + (t1 - t0) * s], -1)


@_quiet
def _seen_mask(cloud: Cloud, S: np.ndarray) -> np.ndarray:
    """Which world floor samples (N, 2) this snapshot saw (the floor there, or something in front of it)."""
    T = pose_matrix(*cloud.pose)
    sb = (S - T[:2, 3]) @ T[:2, :2]                                   # world -> base (xy)
    Sb = np.c_[sb, np.zeros(len(sb))]
    Rc, tc = cloud.T_bc[:3, :3], cloud.T_bc[:3, 3]
    Sc = (Sb - tc) @ Rc                                                # base -> colour optical
    uv = project(cloud.cam, Sc)
    gw, gh = cloud.cam.width, cloud.cam.height
    ug, vg = uv[:, 0] * cloud.cam.scale, uv[:, 1] * cloud.cam.scale
    inview = (Sc[:, 2] > 0.1) & (ug >= 0) & (ug < gw) & (vg >= 0) & (vg < gh)
    iu, iv = np.clip(np.nan_to_num(ug).astype(int), 0, gw - 1), np.clip(np.nan_to_num(vg).astype(int), 0, gh - 1)
    zmeas = cloud.z[iv, iu]
    return inview & np.isfinite(zmeas) & (zmeas >= Sc[:, 2] - 0.15)


@_quiet
def path_check(cloud: Cloud, start: tuple, goal: tuple, margin: float = FOOT_MARGIN,
               others: Sequence[Cloud] = ()) -> dict:
    """Loud, advisory check of a straight move start -> goal (world poses). Never refuses anything.

    Obstacle points (0.04-1.6 m above the floor) come from ``cloud`` and the ``others`` (recent
    snapshots in the same world frame); the swept area counts as seen where ANY of them saw the
    floor (or something in front of it). A turn in place is reported as such: the ring it sweeps
    around the base is below the camera and never visible."""
    clouds = [c for c in [cloud, *others] if c.p_b is not None and c.pose is not None]
    if not clouds:
        return {"checked": False, "warnings": ["cannot check the path: no calibrated camera pose or no pose"]}
    Ws = []
    for c in clouds:
        B = c.p_b[np.isfinite(c.z)]
        ob = B[(B[:, 2] > OBST_Z[0]) & (B[:, 2] < OBST_Z[1])]
        if len(ob):
            Ws.append(c.to_world(ob)[:, :2])
    W = np.concatenate(Ws) if Ws else np.zeros((0, 2))
    poses = _footprints(start, goal)
    h = HALF + margin
    inside = np.zeros(len(W), bool)
    clear = np.full(len(W), np.inf)
    for x, y, th in poses:
        c, s = math.cos(th), math.sin(th)
        dx, dy = W[:, 0] - x, W[:, 1] - y
        lx, ly = c * dx + s * dy, -s * dx + c * dy                     # in that footprint's frame
        ox, oy = np.maximum(np.abs(lx) - HALF, 0), np.maximum(np.abs(ly) - HALF, 0)
        clear = np.minimum(clear, np.hypot(ox, oy))                    # distance to the bare footprint
        inside |= (np.abs(lx) <= h) & (np.abs(ly) <= h)
    length = float(math.hypot(goal[0] - start[0], goal[1] - start[1]))
    turn = float(goal[2] - start[2])
    st = np.array(start[:2])
    samples = []
    for x, y, th in poses[:: max(1, len(poses) // 40)]:
        c, s = math.cos(th), math.sin(th)
        for a in np.linspace(-HALF, HALF, 5):
            for b in np.linspace(-HALF, HALF, 5):
                samples.append((x + c * a - s * b, y + s * a + c * b))
    S = np.array(samples)
    own = np.hypot(S[:, 0] - st[0], S[:, 1] - st[1]) < HALF * 1.2      # where the robot stands now
    seen = np.zeros(len(S), bool)
    for c in clouds:
        seen |= _seen_mask(c, S)
    in_place = length < 0.03
    cov = float(seen[~own].mean()) if (~own).any() else 1.0
    n_in = int(inside.sum())
    out = {"checked": True, "length_m": round(length, 3), "turn_rad": round(turn, 3),
           "footprint_m": round(2 * HALF, 3), "margin_m": margin,
           "obstacle_points_in_path": n_in,
           "min_clearance_m": None if not len(W) else round(float(clear.min()), 3),
           "verified_fraction": None if in_place else round(cov, 3),
           "snapshots_used": len(clouds), "warnings": []}
    if n_in:
        idx = np.where(inside)[0]
        near = idx[np.argsort(clear[idx])[:5]]
        out["nearest_in_path_world"] = [[round(float(W[i, 0]), 3), round(float(W[i, 1]), 3)] for i in near]
        out["warnings"].append(f"OBSTACLE: {n_in} points above the floor inside the swept footprint "
                               f"(+{margin:.2f} m margin); nearest at {out['nearest_in_path_world'][0]} (world)")
    elif len(W) and clear.min() < 0.15:
        out["warnings"].append(f"tight: nearest obstacle point {clear.min():.2f} m from the footprint")
    if in_place:
        r = math.hypot(HALF, HALF)
        out["note"] = (f"turn in place: the corners sweep a {r:.2f} m radius around the base centre, below the "
                       "camera's view; only objects seen there earlier would be reported")
    elif cov < 0.9:
        unseen = S[~seen & ~own]
        d = np.hypot(unseen[:, 0] - st[0], unseen[:, 1] - st[1]) if len(unseen) else np.zeros(0)
        where = (f"within {d.max():.2f} m of the start" if len(d) and d.max() < 1.0 else "along the path")
        out["warnings"].append(f"UNVERIFIED: {100 * (1 - cov):.0f}% of the swept area ({where}) was not seen in "
                               f"the last {len(clouds)} snapshot(s): blind zone, outside the view, or occluded")
    out["_poses"], out["_inside_world"], out["_unseen_world"] = poses, W[inside], S[~seen & ~own]
    return out


# ----------------------------------------------------------------------------------------------
# Rendering
# ----------------------------------------------------------------------------------------------

def _font(size=14):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:                     # old Pillow
        return ImageFont.load_default()


def draw_shapes(img: Image.Image, shapes: Sequence[tuple[dict, tuple]], width: int = 4) -> None:
    dr = ImageDraw.Draw(img)
    for shape, color in shapes:
        if "point" in shape:
            u, v = shape["point"]
            dr.ellipse([u - 9, v - 9, u + 9, v + 9], outline=color, width=width)
            dr.line([u - 16, v, u + 16, v], fill=color, width=2)
            dr.line([u, v - 16, u, v + 16], fill=color, width=2)
        elif "disk" in shape:
            u, v, r = shape["disk"]
            dr.ellipse([u - r, v - r, u + r, v + r], outline=color, width=width)
        elif "box" in shape:
            u0, v0, u1, v1 = shape["box"]
            dr.rectangle([min(u0, u1), min(v0, v1), max(u0, u1), max(v0, v1)], outline=color, width=width)
        elif "polygon" in shape:
            pts = [tuple(map(float, p)) for p in shape["polygon"]]
            dr.line(pts + [pts[0]], fill=color, width=width, joint="curve")


def draw_grid(img: Image.Image, step: int = 100) -> None:
    """Labelled pixel grid so the agent can name coordinates."""
    dr = ImageDraw.Draw(img, "RGBA")
    f = _font(13)
    W, H = img.size
    for u in range(0, W, step):
        dr.line([u, 0, u, H], fill=(255, 255, 0, 110), width=1)
        dr.text((u + 2, 2), str(u), fill=(255, 255, 0, 255), font=f, stroke_width=2, stroke_fill=(0, 0, 0, 255))
    for v in range(0, H, step):
        dr.line([0, v, W, v], fill=(255, 255, 0, 110), width=1)
        dr.text((2, v + 2), str(v), fill=(255, 255, 0, 255), font=f, stroke_width=2, stroke_fill=(0, 0, 0, 255))


@_quiet
def overlay_image(jpeg: bytes, *, grid: bool = False, shapes=(), plane: Optional[dict] = None,
                  cam: Optional[Camera] = None, T_bc: Optional[np.ndarray] = None, label: str = "",
                  line: Optional[dict] = None) -> bytes:
    img = Image.open(io.BytesIO(jpeg)).convert("RGB")
    if grid:
        draw_grid(img)
    draw_shapes(img, shapes)
    if plane is not None and cam is not None and T_bc is not None:
        C = np.array(plane["corners_base"])
        Rc, tc = T_bc[:3, :3], T_bc[:3, 3]
        uv = project(cam, (C - tc) @ Rc)
        if np.isfinite(uv).all():
            pts = [tuple(p) for p in uv]
            ImageDraw.Draw(img).line(pts + [pts[0]], fill=OK_COLOR, width=4)
            cu, cv = project(cam, (np.array([plane["base"]["centre"]]) - tc) @ Rc)[0]
            if np.isfinite(cu):
                ImageDraw.Draw(img).ellipse([cu - 8, cv - 8, cu + 8, cv + 8], outline=OK_COLOR, width=4)
    if line is not None and cam is not None and T_bc is not None:
        Rc, tc = T_bc[:3, :3], T_bc[:3, 3]
        e = np.array(line["base"]["ends"])
        uv = project(cam, (e - tc) @ Rc)
        if np.isfinite(uv).all():
            dr = ImageDraw.Draw(img)
            dr.line([tuple(uv[0]), tuple(uv[1])], fill=OK_COLOR, width=5)
            for q in uv:
                dr.ellipse([q[0] - 7, q[1] - 7, q[0] + 7, q[1] + 7], outline=OK_COLOR, width=3)
    if label:
        ImageDraw.Draw(img).text((10, img.size[1] - 26), label, fill=(255, 255, 255), font=_font(16),
                                 stroke_width=3, stroke_fill=(0, 0, 0))
    b = io.BytesIO()
    img.save(b, "JPEG", quality=85)
    return b.getvalue()


def thumbnail(jpeg: bytes, width: int = 320) -> bytes:
    img = Image.open(io.BytesIO(jpeg)).convert("RGB")
    img.thumbnail((width, width))
    b = io.BytesIO()
    img.save(b, "JPEG", quality=70)
    return b.getvalue()


@_quiet
def bev_image(cloud: Cloud, *, size: int = 640, forward: float = 5.0, back: float = 1.0, side: float = 3.0,
              selection: Optional[np.ndarray] = None, plane: Optional[dict] = None,
              path: Optional[dict] = None, goal_base: Optional[tuple] = None, label: str = "",
              line: Optional[dict] = None) -> bytes:
    """Top-down view in the snapshot's base frame (robot at the bottom, x up, y left), grid 0.5 m.
    Floor points light, obstacles coloured by height; the selection's footprint is outlined with a
    thick magenta border; a path check draws the swept footprint (green; red where blocked; hatched
    grey where unverified)."""
    H = size
    span_x = forward + back
    W = int(round(H * (2 * side) / span_x))
    k = H / span_x                                           # px per m

    def to_px(x, y):
        return (W / 2 - y * k, H - (x + back) * k)

    img = Image.new("RGB", (W, H), (24, 26, 30))
    dr = ImageDraw.Draw(img, "RGBA")
    f = _font(12)
    for g in np.arange(-math.floor(back / 0.5) * 0.5, forward + 1e-9, 0.5):
        a, b = to_px(g, side), to_px(g, -side)
        dr.line([a, b], fill=(255, 255, 255, 40 if g % 1 else 70), width=1)
        if g % 1 == 0:
            dr.text((2, a[1] - 14), f"{g:.0f} m", fill=(200, 200, 200), font=f)
    for g in np.arange(-math.floor(side / 0.5) * 0.5, side + 1e-9, 0.5):
        a, b = to_px(-back, g), to_px(forward, g)
        dr.line([a, b], fill=(255, 255, 255, 40 if g % 1 else 70), width=1)
    pb = cloud.p_b
    if pb is not None:
        fin = np.isfinite(cloud.z)
        P = pb[fin][::2]
        px = np.array([to_px(x, y) for x, y in P[:, :2]]) if len(P) else np.zeros((0, 2))
        z = P[:, 2]
        floor = z <= OBST_Z[0]
        arr = np.array(img)
        ok = (px[:, 0] >= 0) & (px[:, 0] < W) & (px[:, 1] >= 0) & (px[:, 1] < H)
        pi = px[ok].astype(int)
        fl, zz = floor[ok], z[ok]
        arr[pi[fl, 1], pi[fl, 0]] = (90, 95, 105)
        ob = ~fl & (zz < OBST_Z[1])
        t = np.clip(zz[ob] / 1.2, 0, 1)
        cols = np.stack([255 * np.ones_like(t), 210 * (1 - t) + 40, 60 * np.ones_like(t)], -1).astype(np.uint8)
        arr[pi[ob, 1], pi[ob, 0]] = cols
        img = Image.fromarray(arr)
        dr = ImageDraw.Draw(img, "RGBA")
        # camera field of view on the floor (the corners of the image projected to z = 0)
        Rc, tc = cloud.T_bc[:3, :3], cloud.T_bc[:3, 3]
        cam = cloud.cam
        corner_rays = []
        for u, v in ((0, cam.height - 1), (cam.width - 1, cam.height - 1)):
            ray = Rc @ rays(cam)[v, u]
            if ray[2] < -1e-6:
                corner_rays.append(tc - ray * tc[2] / ray[2])
        if len(corner_rays) == 2:
            dr.line([to_px(*corner_rays[0][:2]), to_px(*corner_rays[1][:2])], fill=(120, 170, 255, 160), width=2)
            dr.text(to_px(*((corner_rays[0][:2] + corner_rays[1][:2]) / 2)), "nearest visible floor",
                    fill=(150, 190, 255), font=f, anchor="mt")
    if path is not None and path.get("checked") and cloud.pose is not None:
        T = pose_matrix(*cloud.pose)

        def w2b(xy):
            v = (np.asarray(xy, float) - T[:2, 3]) @ T[:2, :2]
            return v

        blocked = path["obstacle_points_in_path"] > 0
        col = (255, 60, 40, 70) if blocked else (0, 200, 90, 55)
        for x, y, th in path["_poses"][:: max(1, len(path["_poses"]) // 30)]:
            c, s = math.cos(th), math.sin(th)
            corners = [(x + c * a - s * b, y + s * a + c * b) for a, b in ((HALF, HALF), (HALF, -HALF), (-HALF, -HALF), (-HALF, HALF))]
            dr.polygon([to_px(*w2b(p)) for p in corners], fill=col)
        for p in path["_unseen_world"]:
            q = to_px(*w2b(p))
            dr.rectangle([q[0] - 2, q[1] - 2, q[0] + 2, q[1] + 2], fill=(160, 160, 160, 160))
        for p in path["_inside_world"][:2000]:
            q = to_px(*w2b(p))
            dr.ellipse([q[0] - 3, q[1] - 3, q[0] + 3, q[1] + 3], fill=(255, 40, 40, 255))
    if selection is not None and pb is not None:
        sel = selection & np.isfinite(cloud.z)
        S = pb[sel][:, :2]
        if len(S) >= 3:
            px = np.array([to_px(x, y) for x, y in S])
            hull = _convex_hull(px)
            dr.polygon([tuple(p) for p in hull], fill=SEL_COLOR + (60,))
            dr.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=SEL_COLOR + (255,), width=5)
        elif len(S):
            q = to_px(*S[0])
            dr.ellipse([q[0] - 8, q[1] - 8, q[0] + 8, q[1] + 8], outline=SEL_COLOR, width=4)
    if plane is not None:
        c = plane["base"]["centre"]
        n = plane["base"]["normal"]
        cs = plane["corners_base"]
        a, b = to_px(*cs[0][:2]), to_px(*cs[1][:2])
        dr.line([a, b], fill=OK_COLOR, width=5)
        p0 = to_px(c[0], c[1])
        p1 = to_px(c[0] + 0.5 * n[0], c[1] + 0.5 * n[1])
        dr.line([p0, p1], fill=OK_COLOR, width=3)
        dr.ellipse([p0[0] - 5, p0[1] - 5, p0[0] + 5, p0[1] + 5], fill=OK_COLOR)
    if line is not None:
        e = line["base"]["ends"]
        a, b = to_px(e[0][0], e[0][1]), to_px(e[1][0], e[1][1])
        dr.line([a, b], fill=OK_COLOR, width=5)
        dr.ellipse([a[0] - 5, a[1] - 5, a[0] + 5, a[1] + 5], fill=OK_COLOR)
        for p in line.get("per_point_base") or []:
            if p is not None:
                q = to_px(p[0], p[1])
                dr.ellipse([q[0] - 6, q[1] - 6, q[0] + 6, q[1] + 6], outline=SEL_COLOR, width=3)
    # the robot (footprint, heading) at the snapshot, and an optional goal
    _robot(dr, to_px, (0.0, 0.0, 0.0), (90, 160, 255))
    if goal_base is not None:
        _robot(dr, to_px, goal_base, (0, 220, 120))
    if label:
        dr.text((8, 8), label, fill=(255, 255, 255), font=_font(15), stroke_width=2, stroke_fill=(0, 0, 0))
    b = io.BytesIO()
    img.save(b, "PNG")
    return b.getvalue()


def _robot(dr, to_px, pose, color):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    corners = [(x + c * a - s * b, y + s * a + c * b) for a, b in ((HALF, HALF), (HALF, -HALF), (-HALF, -HALF), (-HALF, HALF))]
    pts = [to_px(*p) for p in corners]
    dr.line(pts + [pts[0]], fill=color, width=3)
    dr.line([to_px(x, y), to_px(x + c * HALF * 1.6, y + s * HALF * 1.6)], fill=color, width=3)


def _convex_hull(pts: np.ndarray) -> np.ndarray:
    p = np.unique(np.round(pts, 1), axis=0)
    if len(p) < 3:
        return p
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def half(points):
        h = []
        for q in points:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(q)
        return h

    lower, upper = half(p), half(p[::-1])
    return np.array(lower[:-1] + upper[:-1])


# ----------------------------------------------------------------------------------------------
# Robot description (for the agent's situational awareness)
# ----------------------------------------------------------------------------------------------

def _floor_hit(T_bc: np.ndarray, ray_c: np.ndarray) -> Optional[np.ndarray]:
    """Where a colour-optical ray from the camera meets the floor (base frame), or None."""
    R, t = T_bc[:3, :3], T_bc[:3, 3]
    d = R @ ray_c
    if d[2] >= -1e-6:
        return None
    return t - d * t[2] / d[2]


@_quiet
def camera_facts(cam: Camera, T_bc: np.ndarray) -> dict:
    """Derived camera facts in the base frame: position, viewing direction, field of view, the floor it sees."""
    R, t = T_bc[:3, :3], T_bc[:3, 3]
    fwd = R @ np.array([0, 0, 1.0])
    right = R @ np.array([1.0, 0, 0])
    r = rays(cam)
    H, W = cam.height, cam.width
    hfov = math.degrees(math.atan2(r[H // 2, -1, 0], 1) - math.atan2(r[H // 2, 0, 0], 1))
    vfov = math.degrees(math.atan2(r[-1, W // 2, 1], 1) - math.atan2(r[0, W // 2, 1], 1))
    bottom = [_floor_hit(T_bc, r[-1, u]) for u in (0, W // 2, W - 1)]
    centre = _floor_hit(T_bc, r[H // 2, W // 2])
    top = _floor_hit(T_bc, r[0, W // 2])
    out = {
        "position_base_m": [round(float(v), 3) for v in t],
        "height_m": round(float(t[2]), 3),
        "optical_axis_base": [round(float(v), 3) for v in fwd],
        "pitch_down_deg": round(float(math.degrees(math.asin(-fwd[2]))), 1),
        "yaw_deg": round(float(math.degrees(math.atan2(fwd[1], fwd[0]))), 1),
        "roll_deg": round(float(math.degrees(math.asin(max(-1, min(1, right[2]))))), 1),
        "hfov_deg": round(hfov, 1), "vfov_deg": round(vfov, 1),
        "image_size": [cam.color_w, cam.color_h],
        "nearest_visible_floor_m": None if bottom[1] is None else round(float(bottom[1][0]), 3),
        "nearest_visible_floor_width_m": None if bottom[0] is None or bottom[2] is None else round(float(abs(bottom[0][1] - bottom[2][1])), 3),
        "image_centre_hits_floor_at_m": None if centre is None else round(float(centre[0]), 3),
        "top_of_image": "above the horizon (sees walls and tall objects)" if top is None else f"floor at {top[0]:.2f} m",
    }
    out["blind_zone"] = (f"the floor within ~{out['nearest_visible_floor_m']:.2f} m of the base centre "
                         f"(~{out['nearest_visible_floor_m'] - HALF:.2f} m beyond the front edge) is never visible, nor "
                         "anything beside or behind the robot" if out["nearest_visible_floor_m"] else "unknown")
    out["_bottom"], out["_top"] = bottom, top
    return out


def robot_diagram(cam: Camera, T_bc: np.ndarray, wheels: Sequence[dict], facts: dict) -> bytes:
    """Side view (x-z) and top view (x-y) of the base, the camera and what it sees, to scale."""
    Wd, Hd = 1100, 520
    img = Image.new("RGB", (Wd, Hd), (24, 26, 30))
    dr = ImageDraw.Draw(img, "RGBA")
    f, fb = _font(13), _font(16)
    R, t = T_bc[:3, :3], T_bc[:3, 3]
    r = rays(cam)
    H, W = cam.height, cam.width
    # ---- side view: x forward (right), z up; 0 .. 4 m, -0.1 .. 1.6 m
    ox, oy, k = 40, 330, 105.0

    def S(x, z):
        return (ox + (x + 0.35) * k, oy - z * k)

    dr.text((ox, 12), "side view, from the robot's left: x forward = right, z up", fill=(230, 230, 230), font=fb)
    for x in np.arange(0, 4.01, 0.5):
        dr.line([S(x, -0.05), S(x, 1.55)], fill=(255, 255, 255, 30 if x % 1 else 60))
        dr.text(S(x, -0.08), f"{x:.1f} m" if x % 1 == 0 else "", fill=(170, 170, 170), font=f, anchor="mt")
    for z in (0.5, 1.0, 1.5):
        dr.line([S(-0.35, z), S(4.0, z)], fill=(255, 255, 255, 30))
        dr.text(S(-0.35, z), f"{z:.1f}", fill=(150, 150, 150), font=f, anchor="rm")
    dr.line([S(-0.35, 0), S(4.0, 0)], fill=(200, 200, 200), width=2)                       # floor
    dr.rectangle([S(-HALF, 0.17), S(HALF, 0.03)], outline=(90, 160, 255), width=3)            # base
    for wx in (-0.1656, 0.1656):
        c = S(wx, 0.038)
        dr.ellipse([c[0] - 4, c[1] - 4, c[0] + 4, c[1] + 4], fill=(90, 160, 255))
    dr.line([S(t[0], 0.17), S(t[0], t[2])], fill=(90, 160, 255), width=3)                     # mast
    cam_px = S(t[0], t[2])
    # vertical fov in the camera's central vertical plane
    for v, col in ((0, (120, 170, 255, 220)), (H // 2, (255, 255, 255, 120)), (H - 1, (120, 170, 255, 220))):
        d = R @ r[v, W // 2]
        hit = _floor_hit(T_bc, r[v, W // 2])
        end = hit if hit is not None and hit[0] < 4.0 else t + d / np.linalg.norm(d[[0, 2]]) * 4.0
        dr.line([cam_px, S(end[0], end[2])], fill=col, width=2)
    b = facts["_bottom"][1]
    if b is not None:
        dr.line([S(0, 0), S(b[0], 0)], fill=(255, 60, 40, 220), width=6)                        # blind floor
        dr.text(S(b[0] / 2, 0.06), f"blind: {b[0]:.2f} m", fill=(255, 120, 100), font=f, anchor="mb")
    dr.ellipse([cam_px[0] - 6, cam_px[1] - 6, cam_px[0] + 6, cam_px[1] + 6], fill=(255, 210, 60))
    dr.text((cam_px[0] + 10, cam_px[1] - 18), f"camera {t[2]:.2f} m up, pitched {facts['pitch_down_deg']:.0f}° down, "
            f"vertical fov {facts['vfov_deg']:.0f}°", fill=(255, 220, 120), font=f)
    # ---- top view: x up, y left; robot near the bottom
    tx0, ty0, kt = 820, 470, 62.0

    def T(x, y):
        return (tx0 - y * kt, ty0 - x * kt)

    dr.text((560, 12), "top view: x forward = up, y left = left", fill=(230, 230, 230), font=fb)
    for x in np.arange(-0.5, 7.01, 0.5):
        dr.line([T(x, 3.6), T(x, -3.6)], fill=(255, 255, 255, 30 if x % 1 else 60))
        if x % 1 == 0 and x >= 0:
            dr.text(T(x, 3.6), f"{x:.0f} m", fill=(150, 150, 150), font=f, anchor="lm")
    for y in np.arange(-3.5, 3.51, 0.5):
        dr.line([T(-0.5, y), T(7.0, y)], fill=(255, 255, 255, 30 if y % 1 else 60))
    # visible floor polygon: bottom edge hits + far edge at the image's horizon (clipped at 6 m)
    far = []
    for u in (0, W - 1):
        d = R @ r[0, u]
        dh = d[:2] / (np.linalg.norm(d[:2]) or 1)
        far.append(t[:2] + dh * 6.0)
    bl, _, br = facts["_bottom"]
    if bl is not None and br is not None:
        poly = [T(*bl[:2]), T(*br[:2]), T(*far[1]), T(*far[0])]
        dr.polygon(poly, fill=(120, 170, 255, 45), outline=(120, 170, 255, 200))
        dr.polygon([T(*t[:2]), T(*bl[:2]), T(*br[:2])], fill=(255, 60, 40, 70))
        dr.text(T(bl[0] + 0.15, 0), "visible floor (to the horizon)", fill=(150, 190, 255), font=f, anchor="ms")
        dr.text(T(bl[0] * 0.45, 0), "blind", fill=(255, 140, 120), font=f, anchor="mm")
    corners = [T(a, b) for a, b in ((HALF, HALF), (HALF, -HALF), (-HALF, -HALF), (-HALF, HALF))]
    dr.line(corners + [corners[0]], fill=(90, 160, 255), width=3)
    dr.line([T(0, 0), T(HALF * 1.7, 0)], fill=(90, 160, 255), width=3)
    for w in wheels:
        c = T(w["x"], w["y"])
        dr.rectangle([c[0] - 4, c[1] - 7, c[0] + 4, c[1] + 7], fill=(90, 160, 255))
        dr.text((c[0] + 7, c[1]), w["name"], fill=(150, 190, 255), font=f, anchor="lm")
    cp = T(*t[:2])
    dr.ellipse([cp[0] - 5, cp[1] - 5, cp[0] + 5, cp[1] + 5], fill=(255, 210, 60))
    dr.text((560, 36), f"footprint {2 * HALF:.2f} m square · horizontal fov {facts['hfov_deg']:.0f}° · camera yaw "
            f"{facts['yaw_deg']:+.1f}° · nearest visible floor {facts['nearest_visible_floor_m']} m",
            fill=(220, 220, 220), font=f)
    b = io.BytesIO()
    img.save(b, "PNG")
    return b.getvalue()
