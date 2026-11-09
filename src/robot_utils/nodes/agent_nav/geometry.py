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
    if touches:
        out["warning"] = ("the fitted surface reaches the image edge: it may extend beyond the view, so its "
                          "centre and width may be biased")
    if cloud.pose is not None:
        w = cloud.to_world(centre[None])[0]
        th = cloud.pose[2]
        nw = np.array([math.cos(th) * n[0] - math.sin(th) * n[1], math.sin(th) * n[0] + math.cos(th) * n[1], n[2]])
        out["world"] = {"centre": [round(float(v), 3) for v in w], "normal": [round(float(v), 3) for v in nw],
                        "facing_heading": round(float(math.atan2(nw[1], nw[0])), 3)}
    out["_inlier_mask"] = inl_mask
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
def path_check(cloud: Cloud, start: tuple, goal: tuple, margin: float = FOOT_MARGIN) -> dict:
    """Loud, advisory check of a straight move start -> goal (world poses) against the obstacle
    points of ``cloud`` and the area the snapshot actually saw. Never refuses anything."""
    pb = cloud.p_b
    if pb is None or cloud.pose is None:
        return {"checked": False, "warnings": ["cannot check the path: no calibrated camera pose or no pose"]}
    fin = np.isfinite(cloud.z)
    B = pb[fin]
    obst_b = B[(B[:, 2] > OBST_Z[0]) & (B[:, 2] < OBST_Z[1])]
    W = cloud.to_world(obst_b)[:, :2] if len(obst_b) else np.zeros((0, 2))
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
    # coverage: floor samples over the swept area, seen by the snapshot?
    samples = []
    for x, y, th in poses[:: max(1, len(poses) // 40)]:
        c, s = math.cos(th), math.sin(th)
        for a in np.linspace(-HALF, HALF, 5):
            for b in np.linspace(-HALF, HALF, 5):
                samples.append((x + c * a - s * b, y + s * a + c * b))
    S = np.array(samples)
    T = pose_matrix(*cloud.pose)
    sb = (S - T[:2, 3]) @ T[:2, :2]                                   # world -> base (xy)
    Sb = np.c_[sb, np.zeros(len(sb))]
    Rc, tc = cloud.T_bc[:3, :3], cloud.T_bc[:3, 3]
    Sc = (Sb - tc) @ Rc                                                # base -> colour optical
    uv = project(cloud.cam, Sc)
    gw, gh = cloud.cam.width, cloud.cam.height
    ug, vg = uv[:, 0] * cloud.cam.scale, uv[:, 1] * cloud.cam.scale
    inview = (Sc[:, 2] > 0.1) & (ug >= 0) & (ug < gw) & (vg >= 0) & (vg < gh)
    seen = np.zeros(len(S), bool)
    iu, iv = np.clip(ug.astype(int), 0, gw - 1), np.clip(vg.astype(int), 0, gh - 1)
    zmeas = cloud.z[iv, iu]
    # the floor there was seen (depth reaches it) or something nearer stands in front (an obstacle, reported above)
    seen = inview & np.isfinite(zmeas) & (zmeas >= Sc[:, 2] - 0.15)
    # the robot's own footprint at the start is never visible, and is where it stands: don't count it
    st = np.array(start[:2])
    own = np.hypot(S[:, 0] - st[0], S[:, 1] - st[1]) < HALF * 1.2
    cov = float(seen[~own].mean()) if (~own).any() else 1.0
    n_in = int(inside.sum())
    out = {"checked": True, "length_m": round(float(math.hypot(goal[0] - start[0], goal[1] - start[1])), 3),
           "turn_rad": round(float(goal[2] - start[2]), 3),
           "footprint_m": round(2 * HALF, 3), "margin_m": margin,
           "obstacle_points_in_path": n_in,
           "min_clearance_m": None if not len(W) else round(float(clear.min()), 3),
           "verified_fraction": round(cov, 3), "warnings": []}
    if n_in:
        idx = np.where(inside)[0]
        near = idx[np.argsort(clear[idx])[:5]]
        out["nearest_in_path_world"] = [[round(float(W[i, 0]), 3), round(float(W[i, 1]), 3)] for i in near]
        out["warnings"].append(f"OBSTACLE: {n_in} points above the floor inside the swept footprint "
                               f"(+{margin:.2f} m margin); nearest at {out['nearest_in_path_world'][0]} (world)")
    elif len(W) and clear.min() < 0.15:
        out["warnings"].append(f"tight: nearest obstacle point {clear.min():.2f} m from the footprint")
    if cov < 0.95:
        out["warnings"].append(f"UNVERIFIED: {100 * (1 - cov):.0f}% of the swept area was not visible in this "
                               "snapshot (blind zone in front of the robot, outside the view, or occluded)")
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
                  cam: Optional[Camera] = None, T_bc: Optional[np.ndarray] = None, label: str = "") -> bytes:
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
              path: Optional[dict] = None, goal_base: Optional[tuple] = None, label: str = "") -> bytes:
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
