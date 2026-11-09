"""Depth -> colour registration with a motion transform: pure numpy, no I/O (unit-testable).

A depth pixel (u_d, v_d) with depth z_d unprojects to ``p_d = z_d · ray_d(u_d, v_d)`` in the depth
camera frame at the depth frame's time t_d. The node supplies ``M`` = the pose change of the depth
camera between t_d and the colour frame's time t_c (``p_d(t_c) = M · p_d(t_d)``, see motion.py), so

    p_c = E · M · p_d          E = depth -> colour extrinsic (factory)

is the point in the **colour camera frame at t_c**. It is projected with the colour intrinsics
(including lens distortion, OpenCV's rational/Brown-Conrady model) into an output grid that is the
colour image scaled by ``scale`` (0.5 = half the colour resolution), and z-buffered: where several
depth pixels land on one output pixel the nearest wins. The result is uint16 millimetres of
**z in the colour camera frame** (0 = no depth). The 3D point of an output or colour pixel is then
``z · ray_c(u, v)`` with the colour camera's undistorted ray (``Intrinsics.ray``).

Cost: everything is vectorised over the (strided) depth image with in-place float32 arithmetic.
When the depth camera has no distortion (this camera) its rays are separable (x depends only on
the column, y only on the row), so the rotation of the rays costs one broadcast add per
coordinate instead of a 3x3 product per pixel.

Conventions: pixel centres at integer coordinates (OpenCV / Orbbec); optical frames x right,
y down, z out of the lens; metres for transforms, millimetres for depth images.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional

import numpy as np

DIST_KEYS = ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole + OpenCV rational distortion (k1..k6, p1, p2); Brown-Conrady when k4..k6 = 0."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist: tuple = field(default=(0.0,) * 8)    # k1 k2 k3 k4 k5 k6 p1 p2

    @classmethod
    def from_json(cls, d: dict) -> "Intrinsics":
        dd = d.get("distortion") or {}
        return cls(float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"]), int(d["width"]),
                   int(d["height"]), tuple(float(dd.get(k, 0.0) or 0.0) for k in DIST_KEYS))

    def to_json(self) -> dict:
        return {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy, "width": self.width,
                "height": self.height, "distortion": dict(zip(DIST_KEYS, self.dist))}

    @property
    def distorted(self) -> bool:
        return any(abs(k) > 0 for k in self.dist)

    def scaled(self, s: float) -> "Intrinsics":
        """The same camera sampled on a grid ``s`` times the size (pixel centres kept consistent:
        u' + 0.5 = s (u + 0.5)). Distortion is in normalised coordinates, so it is unchanged."""
        if s == 1.0:
            return self
        w, h = max(1, int(round(self.width * s))), max(1, int(round(self.height * s)))
        sx, sy = w / self.width, h / self.height
        return replace(self, fx=self.fx * sx, fy=self.fy * sy, cx=(self.cx + 0.5) * sx - 0.5,
                       cy=(self.cy + 0.5) * sy - 0.5, width=w, height=h)

    # -- normalised <-> distorted --------------------------------------------------------------

    def distort(self, x, y):
        """Undistorted normalised coords -> distorted normalised coords (vectorised)."""
        k1, k2, k3, k4, k5, k6, p1, p2 = self.dist
        r2 = x * x + y * y
        radial = (1 + r2 * (k1 + r2 * (k2 + r2 * k3)))
        if k4 or k5 or k6:
            radial = radial / (1 + r2 * (k4 + r2 * (k5 + r2 * k6)))
        xy = x * y
        return (x * radial + 2 * p1 * xy + p2 * (r2 + 2 * x * x),
                y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * xy)

    def undistort(self, xd, yd, iterations: int = 8):
        """Distorted normalised coords -> undistorted (fixed-point iteration, as OpenCV)."""
        xd, yd = np.asarray(xd, np.float64), np.asarray(yd, np.float64)
        if not self.distorted:
            return xd, yd
        k1, k2, k3, k4, k5, k6, p1, p2 = self.dist
        x, y = xd.copy(), yd.copy()
        for _ in range(iterations):
            r2 = x * x + y * y
            icdist = (1 + r2 * (k4 + r2 * (k5 + r2 * k6))) / (1 + r2 * (k1 + r2 * (k2 + r2 * k3)))
            dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            x = (xd - dx) * icdist
            y = (yd - dy) * icdist
        return x, y

    def ray(self, u, v):
        """Pixel -> undistorted ray (x/z, y/z) of the point seen there."""
        return self.undistort((np.asarray(u, np.float64) - self.cx) / self.fx,
                              (np.asarray(v, np.float64) - self.cy) / self.fy)

    def project(self, p):
        """(..., 3) points in this camera's frame -> (u, v) pixel coords (with distortion)."""
        p = np.asarray(p, np.float64)
        x, y = self.distort(p[..., 0] / p[..., 2], p[..., 1] / p[..., 2])
        return self.fx * x + self.cx, self.fy * y + self.cy


def rigid(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def inv(T) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return rigid(R.T, -R.T @ t)


class Registrar:
    """Reprojects depth images into the (scaled) colour grid. Build once per calibration.

    ``depth_to_color``: 4x4, p_colour = E · p_depth (metres). ``scale``: output grid = colour
    resolution × scale. ``stride``: use every stride-th depth pixel in both directions.
    ``oversample``: resolution of the distortion lookup table (cells per output pixel).
    ``band_pixels``: depth pixels processed per step (sized for the CPU cache).
    ``splat``: 1 = each depth sample writes one output pixel; 2 = a 2x2 block (fills the gaps when
    the output grid is denser than the depth samples, e.g. full colour resolution)."""

    def __init__(self, depth: Intrinsics, color: Intrinsics, depth_to_color, *, scale: float = 0.5,
                 stride: int = 1, splat: int = 1, min_mm: float = 100.0, max_mm: float = 20000.0,
                 oversample: int = 2, band_pixels: int = 32768):
        self.depth, self.color = depth, color
        self.out = color.scaled(scale)
        self.E = np.asarray(depth_to_color, np.float64)
        self.scale, self.stride, self.splat = float(scale), int(max(1, stride)), int(max(1, splat))
        self.min_mm, self.max_mm = float(min_mm), float(max_mm)
        s = self.stride
        us = np.arange(0, depth.width, s, dtype=np.float64)
        vs = np.arange(0, depth.height, s, dtype=np.float64)
        self.shape = (len(vs), len(us))
        if depth.distorted:   # full ray maps (H, W)
            xr, yr = depth.ray(us[None, :], vs[:, None])
            self._rx, self._ry = (np.ascontiguousarray(a, np.float32) for a in np.broadcast_arrays(xr, yr))
            self._separable = False
        else:                 # x depends on the column only, y on the row only
            self._rx = ((us - depth.cx) / depth.fx).astype(np.float32)[None, :]
            self._ry = ((vs - depth.cy) / depth.fy).astype(np.float32)[:, None]
            self._separable = True
        self.band = max(1, int(band_pixels // self.shape[1]))
        self._buf = [np.empty((min(self.band, self.shape[0]), self.shape[1]), np.float32) for _ in range(4)]
        self._build_lut(oversample)

    def _build_lut(self, k: int) -> None:
        """Lookup table: ideal (pinhole, undistorted) position on the output grid, in cells of
        1/k pixel -> flat index of the output pixel the distorted lens puts it on (or the trash
        slot ``out.width * out.height`` when that falls outside). One table per splat offset.
        Folds distortion, rounding and bounds checks into one gather per point; the rounding of
        the ideal position adds at most 0.5/k pixel to the usual 0.5."""
        o = self.out
        self.trash = o.width * o.height
        # ideal-coordinate extent that can land in the image: undistort the image border
        b = np.concatenate([np.stack([np.linspace(-0.5, o.width - 0.5, 200), np.full(200, y)], 1)
                            for y in (-0.5, o.height - 0.5)] +
                           [np.stack([np.full(200, x), np.linspace(-0.5, o.height - 0.5, 200)], 1)
                            for x in (-0.5, o.width - 0.5)])
        xn, yn = o.ray(b[:, 0], b[:, 1])
        ui, vi = o.fx * xn + o.cx, o.fy * yn + o.cy
        self.u0 = math.floor(min(ui.min(), -0.5)) - 2.0
        self.v0 = math.floor(min(vi.min(), -0.5)) - 2.0
        u1 = math.ceil(max(ui.max(), o.width - 0.5)) + 2.0
        v1 = math.ceil(max(vi.max(), o.height - 0.5)) + 2.0
        self.k = k
        self.lw, self.lh = int((u1 - self.u0) * k), int((v1 - self.v0) * k)
        cu = self.u0 + (np.arange(self.lw) + 0.5) / k          # cell centres, ideal pixel coords
        cv = self.v0 + (np.arange(self.lh) + 0.5) / k
        xd, yd = o.distort(((cu - o.cx) / o.fx)[None, :], ((cv - o.cy) / o.fy)[:, None])
        pu = np.floor(o.fx * xd + o.cx + 0.5).astype(np.int64)
        pv = np.floor(o.fy * yd + o.cy + 0.5).astype(np.int64)
        offsets = ((0, 0), (1, 0), (0, 1), (1, 1)) if self.splat > 1 else ((0, 0),)
        self.luts = []
        for du, dv in offsets:
            u, v = pu + du, pv + dv
            ok = (u >= 0) & (u < o.width) & (v >= 0) & (v < o.height)
            lut = np.where(ok, v * o.width + u, self.trash).astype(np.int32)
            lut[0, :] = lut[-1, :] = lut[:, 0] = lut[:, -1] = self.trash    # clipped points land here
            self.luts.append(lut.ravel())

    def _coord(self, R_row, t, z, out, rx, ry):
        """out = z * (R_row · ray) + t, in place (float32)."""
        a, b, c = (np.float32(v) for v in R_row)
        if self._separable:
            np.add(a * rx + np.float32(c), b * ry, out=out)   # (1,W) + (h,1) -> (h,W)
        else:
            np.multiply(rx, a, out=out)
            out += b * ry
            out += np.float32(c)
        out *= z
        out += np.float32(t)
        return out

    def register(self, depth_raw: np.ndarray, depth_scale_mm: float = 1.0,
                 motion: Optional[np.ndarray] = None) -> np.ndarray:
        """depth_raw: (H, W) uint16 depth units (× depth_scale_mm = mm). motion: 4x4 depth(t_d) ->
        depth(t_c), None = identity. Returns (out.height, out.width) uint16 mm, z in the colour frame.

        Works in bands of rows so the float32 temporaries stay in the CPU cache (on the Pi 5 a
        whole-image pass was several times slower per pixel than a cache-sized one)."""
        s = self.stride
        d = depth_raw[::s, ::s] if s > 1 else depth_raw
        if d.shape != self.shape:
            raise ValueError(f"depth image {depth_raw.shape} does not match the calibration "
                             f"({self.depth.height}, {self.depth.width})")
        A = self.E if motion is None else self.E @ np.asarray(motion, np.float64)
        R, t = A[:3, :3], A[:3, 3]
        o, k = self.out, self.k
        out = np.full(self.trash + 1, 0xFFFF, np.uint16)
        zmin, zmax = np.float32(self.min_mm / 1000.0), np.float32(self.max_mm / 1000.0)
        H = self.shape[0]
        for r0 in range(0, H, self.band):
            r1 = min(H, r0 + self.band)
            n = r1 - r0
            z, X, Y, Z = (b[:n] for b in self._buf)
            db = d[r0:r1]
            rx = self._rx if self._separable else self._rx[r0:r1]
            ry = self._ry[r0:r1]
            np.multiply(db, np.float32(depth_scale_mm / 1000.0), out=z, casting="unsafe")
            np.copyto(z, np.float32(np.nan), where=db == 0)   # no depth: NaN all the way -> trash slot
            self._coord(R[0], t[0], z, X, rx, ry)
            self._coord(R[1], t[1], z, Y, rx, ry)
            self._coord(R[2], t[2], z, Z, rx, ry)
            with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
                # ideal position in LUT cells: k (f x/z + c - c0)
                X /= Z
                X *= np.float32(o.fx * k)
                X += np.float32((o.cx - self.u0) * k)
                Y /= Z
                Y *= np.float32(o.fy * k)
                Y += np.float32((o.cy - self.v0) * k)
                ui = X.astype(np.int32).ravel()    # NaN / inf / huge -> some int, clipped onto the trash border
                vi = Y.astype(np.int32).ravel()
            np.clip(ui, 0, self.lw - 1, out=ui)
            np.clip(vi, 0, self.lh - 1, out=vi)
            vi *= self.lw
            vi += ui
            # z in mm; behind / too close / too far / NaN -> 65535 (never wins, cleared below)
            zq = Z.ravel()
            bad = ~((zq >= zmin) & (zq <= zmax))
            zq *= np.float32(1000.0)
            zq += np.float32(0.5)
            np.copyto(zq, np.float32(65535.0), where=bad)
            zq16 = zq.astype(np.uint16)
            for lut in self.luts:
                np.minimum.at(out, lut[vi], zq16)          # z-buffer: nearest wins
        out = out[:-1]
        out[out == 0xFFFF] = 0
        return out.reshape(o.height, o.width)


def query_point(depth_out: np.ndarray, out: Intrinsics, color: Intrinsics, u: float, v: float,
                radius: int = 2) -> dict:
    """Depth and 3D point (colour camera frame, m) at colour pixel (u, v) of a registered image.

    ``depth_out`` is on the ``out`` grid (a scaled copy of ``color``). The depth is the output
    pixel's own value when valid; otherwise the median of the valid pixels within ``radius``
    output pixels (``source`` says which; null if none). The point is that depth along the colour
    camera's (undistorted) ray through (u, v), so it is exact for the clicked pixel."""
    sx, sy = out.width / color.width, out.height / color.height
    uo, vo = (u + 0.5) * sx - 0.5, (v + 0.5) * sy - 0.5
    iu, iv = int(round(uo)), int(round(vo))
    res = {"u": u, "v": v, "u_out": iu, "v_out": iv, "depth_mm": None, "source": None,
           "n_valid": 0, "point": None, "ray": None}
    xr, yr = color.ray(u, v)
    res["ray"] = [float(xr), float(yr), 1.0]
    if not (0 <= iu < out.width and 0 <= iv < out.height):
        res["source"] = "outside"
        return res
    win = depth_out[max(0, iv - radius): iv + radius + 1, max(0, iu - radius): iu + radius + 1]
    valid = win[win > 0]
    res["n_valid"] = int(valid.size)
    z = int(depth_out[iv, iu])
    if z > 0:
        res["source"] = "pixel"
    elif valid.size:
        z = int(np.median(valid))
        res["source"] = "window_median"
    else:
        return res
    zm = z / 1000.0
    res["depth_mm"] = z
    res["point"] = [float(xr) * zm, float(yr) * zm, zm]
    if valid.size:
        res["window_mm"] = {"min": int(valid.min()), "median": int(np.median(valid)), "max": int(valid.max())}
    return res
