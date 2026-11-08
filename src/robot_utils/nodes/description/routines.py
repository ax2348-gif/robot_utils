"""Calibration routines: gather data from the camera (and drive the base), then solve.

They talk to the other nodes through two small I/O classes so tests can substitute simulated ones:
``HttpCameraIO`` (camera node) and ``HttpSwerveIO`` (swerve node). All times are this machine's
``time.monotonic()``; upstream timestamps are converted with an NTP-style offset per node.

Ground (``run_ground``): N raw depth frames -> per-pixel median -> points inside the polygons (or
all, with gravity as a prior) -> robust plane -> floor normal + camera height. Never moves.

Yaw (``run_yaw``): MOVES THE ROBOT. For each base-frame unit vector: pre-align the wheels (a slow
nudge along the vector, then stop), then translate back and forth along it, v(t) = A w sin(w t)
for N cycles (position A (1 - cos w t): from 0 to 2A and back), omega always 0. Meanwhile the raw
IMU (1 kHz) and the swerve ticks are recorded. Fit: a time-free line through the IMU accelerometer
samples (``solvers.fit_motion_line``); the sign comes from correlating with the sign of the
measured base acceleration. Safety: limits on amplitude / speed / travel, explicit confirmation,
abort (stop + disable + release) on yaw rate or heading drift (the robot may be tethered), lease
loss, e-stop on either node, stalled streams, or an abort request.
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from ..common.client import NodeClient
from ..common.protocol import RowsDecoder
from ..swerve.constants import MODULES
from . import __version__
from .solvers import angle_deg, fit_ground_plane, fit_motion_line, polygon_mask, unproject

log = logging.getLogger("description.routines")


class Aborted(RuntimeError):
    """A routine stopped early. ``raw``: what it recorded until then (for diagnosis), or None."""
    raw: Optional[dict] = None


# -- I/O ----------------------------------------------------------------------------------------

class HttpCameraIO:
    def __init__(self, url: str):
        self.client = NodeClient(url, timeout=10.0)
        self.offset = self.client.sync().offset    # camera clock - local clock

    def description(self) -> dict:
        return self.client.get("/description")

    def depth_frames(self, n: int, timeout: float = 10.0) -> list:
        """n distinct raw depth frames (depth in m as float32 arrays, plus header)."""
        from ..orbbec.protocol import RecordReader, decode_payload
        out, seen, deadline = [], set(), time.monotonic() + timeout
        while len(out) < n and time.monotonic() < deadline:
            r = self.client.http.get("/frame/depth", params={"mode": "raw"})
            if r.status_code >= 400:
                raise RuntimeError(f"camera depth frame: HTTP {r.status_code} {r.text[:200]}")
            (h, p), = RecordReader().feed(r.content)
            if h["seq"] in seen:
                time.sleep(0.03)
                continue
            seen.add(h["seq"])
            img = decode_payload(h, p).astype(np.float32) * float(h.get("depth_scale", 1.0)) / 1000.0
            out.append((img, h))
        if len(out) < n:
            raise RuntimeError(f"got {len(out)} of {n} depth frames in {timeout:.0f} s (is the depth stream on?)")
        return out

    def gravity_up(self) -> Optional[np.ndarray]:
        """Mean accelerometer vector (points up when still), IMU frame, from the recent raw samples."""
        r = self.client.get("/imu/latest", n=500, raw="true")
        a = np.array([[s[r["fields"].index(k)] for k in ("ax", "ay", "az")] for s in r["samples"]], float)
        a = a[np.isfinite(a).all(1)]
        return a.mean(0) if len(a) else None

    def imu_rows(self, stop: threading.Event):
        """Yield raw IMU record arrays (fields t_sensor, ax.., gx..) until stop; times converted to local."""
        dec = RowsDecoder()
        for h, p, _ in self.client.records("/imu/stream", raw="true", batch_ms=10):
            if stop.is_set():
                return
            rows = dec.decode(h, p)
            if rows is not None and len(rows):
                yield rows, self.offset


class HttpSwerveIO:
    def __init__(self, url: str):
        from ..swerve.client import SwerveClient
        self.client = SwerveClient(url, timeout=3.0)
        self.offset = self.client.sync().offset

    def state(self) -> dict:
        return self.client.state()

    def control(self, who: str):
        return self.client.control(who)

    def command(self, vx: float, vy: float) -> None:
        self.client.command(vx, vy, 0.0)        # never any rotation (the robot may be tethered)

    def estop(self, reason: str) -> None:
        self.client.estop(reason, "description node")

    def tick_rows(self, stop: threading.Event):
        dec = RowsDecoder()
        for h, p, _ in self.client.records("/stream"):
            if stop.is_set():
                return
            rows = dec.decode(h, p)
            if rows is not None and len(rows):
                yield rows, self.offset


def depth_to_imu_rotation(cam_desc: dict) -> np.ndarray:
    """R such that v_depth = R v_imu (rotation of the imu frame in depth_optical)."""
    f = (cam_desc.get("frames") or {}).get("imu")
    return np.eye(3) if not f else np.asarray(f["rotation"], float)


# -- ground ----------------------------------------------------------------------------------------

@dataclass
class GroundParams:
    polygons: Optional[list] = None      # [[[x, y], ...], ...] normalised image coords; None = automatic
    frames: int = 10
    max_tilt_deg: float = 30.0           # automatic mode: plane within this of horizontal (gravity)
    tol: float = 0.01                    # m, RANSAC inlier distance


def run_ground(cam, p: GroundParams, progress: Callable[[str], None] = lambda m: None) -> tuple[dict, dict]:
    desc = cam.description()
    if desc.get("aligned_depth"):
        K = desc["intrinsics"]["color"]
    else:
        K = desc["intrinsics"]["depth"]
    progress(f"reading {p.frames} depth frames")
    frames = cam.depth_frames(p.frames)
    stack = np.stack([f for f, _ in frames])
    stack[stack <= 0] = np.nan
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)      # all-invalid pixels -> NaN -> 0 below
        depth = np.nanmedian(stack, axis=0)
    depth = np.nan_to_num(depth, nan=0.0)
    mask = polygon_mask(depth.shape, p.polygons) if p.polygons else None
    P = unproject(depth, K, mask)
    g_imu = cam.gravity_up()
    up = None if g_imu is None else depth_to_imu_rotation(desc) @ g_imu
    progress(f"fitting a plane to {len(P)} points")
    fit = fit_ground_plane(P, gravity_up=up if not p.polygons else None, max_tilt_deg=p.max_tilt_deg, tol=p.tol)
    if up is not None and "angle_to_gravity_deg" not in fit:
        fit["angle_to_gravity_deg"] = angle_deg(fit["normal"], up)
    meta = {"params": p.__dict__, "result": fit, "gravity_up_depth": None if up is None else up.tolist(),
            "camera": desc, "frame_seqs": [h["seq"] for _, h in frames], "version": __version__,
            "summary": f"height {fit['height']:.4f} m, rms {fit['rms'] * 1000:.1f} mm, {fit['inliers']} inliers"
                       + (f", {fit['angle_to_gravity_deg']:.2f}° from gravity" if "angle_to_gravity_deg" in fit else "")}
    raw = {"depth_mm": np.round(depth * 1000).astype(np.uint16)}
    if mask is not None:
        raw["mask"] = mask
    return meta, raw


# -- yaw (moves the robot) ----------------------------------------------------------------------------

LIMITS = {"amplitude_max": 0.15, "frequency_min": 0.3, "frequency_max": 2.0, "peak_speed_max": 0.8,
          "cycles_min": 2, "cycles_max": 20, "vectors_max": 8, "travel_max": 12.0,
          # yaw rate: mean over yaw_rate_window_s (single gyro samples spike past 10 °/s from motor
          # vibration and the reversals while the base doesn't turn); heading: the integrated rotation.
          "yaw_rate_abort_dps": 15.0, "yaw_rate_window_s": 0.1, "heading_abort_deg": 5.0,
          "stream_stall_s": 0.5}


@dataclass
class YawParams:
    vectors: list = field(default_factory=lambda: [[1.0, 0.0], [0.0, 1.0]])   # base-frame directions
    amplitude: float = 0.08            # m (position swings 0 .. 2A)
    frequency: float = 1.0             # Hz
    cycles: int = 5
    align_speed: float = 0.03          # m/s nudge along the vector before each run
    align_s: float = 0.4
    settle_s: float = 0.8
    confirm: bool = False


def validate_yaw(p: YawParams) -> list[list[float]]:
    L = LIMITS
    if not p.confirm:
        raise ValueError("this routine drives the robot: pass confirm=true (clear space, e-stop at hand)")
    if not (0 < p.amplitude <= L["amplitude_max"]):
        raise ValueError(f"amplitude must be in (0, {L['amplitude_max']}] m")
    if not (L["frequency_min"] <= p.frequency <= L["frequency_max"]):
        raise ValueError(f"frequency must be in [{L['frequency_min']}, {L['frequency_max']}] Hz")
    peak = p.amplitude * 2 * math.pi * p.frequency
    if peak > L["peak_speed_max"]:
        raise ValueError(f"peak speed A·2πf = {peak:.2f} m/s exceeds {L['peak_speed_max']} m/s")
    if not (L["cycles_min"] <= p.cycles <= L["cycles_max"]):
        raise ValueError(f"cycles must be in [{L['cycles_min']}, {L['cycles_max']}]")
    if not (1 <= len(p.vectors) <= L["vectors_max"]):
        raise ValueError(f"1 to {L['vectors_max']} vectors")
    if not (0 < p.align_speed <= 0.1 and 0 < p.align_s <= 1.0):
        raise ValueError("align_speed must be in (0, 0.1] m/s and align_s in (0, 1] s")
    vecs = []
    for v in p.vectors:
        v = np.asarray(v, float)
        if v.shape != (2,) or not np.all(np.isfinite(v)) or np.linalg.norm(v) < 1e-6:
            raise ValueError(f"bad vector {v.tolist()}: give [vx, vy] in the base frame")
        vecs.append((v / np.linalg.norm(v)).tolist())
    travel = len(vecs) * (4 * p.amplitude * p.cycles + p.align_speed * p.align_s)
    if travel > L["travel_max"]:
        raise ValueError(f"total travel {travel:.1f} m exceeds {L['travel_max']} m")
    return vecs


def base_velocity(rows) -> np.ndarray:
    """(N, 2) base velocity (vx, vy) per swerve tick from the measured wheel speeds and angles."""
    n = len(np.asarray(rows[f"{MODULES[0].name}_drive_vel"]))   # works for record arrays and dicts of columns
    vx = np.zeros(n)
    vy = np.zeros(n)
    for m in MODULES:
        a = np.asarray(rows[f"{m.name}_steer_angle"], float)
        v = np.asarray(rows[f"{m.name}_drive_vel"], float)
        vx += v * np.cos(a)
        vy += v * np.sin(a)
    return np.column_stack([vx, vy]) / len(MODULES)


class _Recorder(threading.Thread):
    """Collects every row of one stream into a list (times already converted to local)."""

    def __init__(self, gen_fn, tcol: str, stop: threading.Event, name: str):
        super().__init__(name=name, daemon=True)
        self.gen_fn, self.tcol, self.stop, self.rows, self.error = gen_fn, tcol, stop, [], None
        self.last = None

    def run(self):
        try:
            for rows, offset in self.gen_fn(self.stop):
                cols = {n: np.asarray(rows[n], float) for n in rows.dtype.names}
                cols[self.tcol] = cols[self.tcol] - offset
                self.rows.append(cols)
                self.last = time.monotonic()
        except Exception as e:      # reported by the routine (stall check)
            if not self.stop.is_set():
                self.error = f"{type(e).__name__}: {e}"

    def table(self) -> dict:
        if not self.rows:
            return {}
        return {k: np.concatenate([r[k] for r in self.rows]) for k in self.rows[0]}


def run_yaw(cam, swerve, p: YawParams, *, should_abort: Callable[[], Optional[str]] = lambda: None,
            progress: Callable[[str], None] = lambda m: None, clock=time.monotonic,
            sleep=time.sleep) -> tuple[dict, dict]:
    vecs = validate_yaw(p)
    st = swerve.state()
    if st.get("state") != "running":
        raise RuntimeError(f"swerve node is {st.get('state')!r}, not running")
    if (st.get("estop") or {}).get("latched"):
        raise RuntimeError("the swerve e-stop is latched")
    if (st.get("lease") or {}).get("held"):
        raise RuntimeError(f"the swerve lease is held by {st['lease'].get('holder')!r}")
    desc = cam.description()
    R_di = depth_to_imu_rotation(desc)
    stop = threading.Event()
    imu = _Recorder(cam.imu_rows, "t_sensor", stop, "rec-imu")
    tick = _Recorder(swerve.tick_rows, "t_tick", stop, "rec-swerve")
    imu.start()
    tick.start()
    L = LIMITS
    w = 2 * math.pi * p.frequency
    windows = []          # (vector, t_start, t_end) of each sinusoid, local time
    mon = {"up": None, "bias": None, "heading": 0.0, "idx": 0, "t_last": None,   # safety monitor state
           "tail_t": np.zeros(0), "tail_r": np.zeros(0), "peak": 0.0, "marks": []}
    W = L["yaw_rate_window_s"]

    def yaw_rate(r) -> np.ndarray:
        return ((np.column_stack([r["gx"], r["gy"], r["gz"]]) - mon["bias"]) * mon["up"]).sum(1)

    def windowed_mean(t, rate) -> np.ndarray:
        """Mean yaw rate over the last W seconds at each new sample (continues from the previous rows)."""
        n0 = len(mon["tail_t"])
        tt, rr = np.concatenate([mon["tail_t"], t]), np.concatenate([mon["tail_r"], rate])
        c = np.concatenate([[0.0], np.cumsum(rr)])
        i = np.arange(len(tt))
        lo = np.searchsorted(tt, tt - W, side="left")
        keep = tt > tt[-1] - W
        mon["tail_t"], mon["tail_r"] = tt[keep], rr[keep]
        return ((c[i + 1] - c[lo]) / (i + 1 - lo))[n0:]

    def monitor() -> Optional[str]:
        """Integrate the bias-corrected yaw rate (about gravity) over the new IMU rows; check its W-mean."""
        rows = imu.rows
        while mon["idx"] < len(rows):
            r = rows[mon["idx"]]
            mon["idx"] += 1
            t = r["t_sensor"]
            if not len(t):
                continue
            rate = yaw_rate(r)
            prev = mon["t_last"] if mon["t_last"] is not None else t[0]
            dt = np.diff(np.concatenate([[prev], t]))
            mon["heading"] += float(np.degrees(np.sum(rate * np.clip(dt, 0, 0.05))))
            mon["t_last"] = t[-1]
            peak = float(np.degrees(np.max(np.abs(windowed_mean(t, rate)))))
            mon["peak"] = max(mon["peak"], peak)
            if peak > L["yaw_rate_abort_dps"]:
                return (f"yaw rate {peak:.1f}°/s (mean over {W * 1000:.0f} ms) > {L['yaw_rate_abort_dps']}°/s, "
                        f"heading {mon['heading']:+.1f}°")
        if abs(mon["heading"]) > L["heading_abort_deg"]:
            return f"heading drifted {abs(mon['heading']):.1f}° > {L['heading_abort_deg']}°"
        return None

    def mark(label: str):
        mon["marks"].append((clock(), label))
        progress(label)

    def recording() -> dict:
        """Everything recorded so far (an aborted run's raw data)."""
        raw = {f"imu_{k}": v for k, v in imu.table().items()}
        raw.update({f"swerve_{k}": v for k, v in tick.table().items()})
        raw["marks_t"] = np.array([m[0] for m in mon["marks"]], float)
        raw["marks"] = np.array([m[1] for m in mon["marks"]], str)
        for k in ("up", "bias"):
            if mon[k] is not None:
                raw[f"monitor_{k}_imu"] = np.asarray(mon[k], float)
        raw["monitor_limits"] = np.array(json.dumps(L))
        return raw

    class _Safety(Aborted):
        pass

    def check():
        r = should_abort()
        if r is not None:
            raise Aborted(r)                    # requested: stop, disable, release (no e-stop)
        now = clock()
        for rec, nm in ((imu, "camera IMU"), (tick, "swerve")):
            if rec.error:
                raise _Safety(f"{nm} stream failed: {rec.error}")
            if rec.last is None or now - rec.last > L["stream_stall_s"]:
                raise _Safety(f"{nm} stream stalled")
        r = monitor()
        if r is not None:
            raise _Safety(r)

    try:
        progress("waiting for the streams")
        t0 = clock()
        while (imu.last is None or tick.last is None) and clock() - t0 < 5.0:
            sleep(0.05)
        if imu.last is None or tick.last is None:
            raise RuntimeError("no data from " + ("the camera IMU" if imu.last is None else "the swerve node"))
        sleep(0.5)                                   # still window: gravity and gyro-bias reference
        tab = imu.table()
        g = np.column_stack([tab["ax"], tab["ay"], tab["az"]]).mean(0)
        mon["up"] = g / np.linalg.norm(g)
        mon["bias"] = np.column_stack([tab["gx"], tab["gy"], tab["gz"]]).mean(0)
        still = tab["t_sensor"] > tab["t_sensor"][-1] - W          # the rate window starts on still data
        mon["tail_t"] = tab["t_sensor"][still]
        mon["tail_r"] = yaw_rate({k: tab[k][still] for k in ("gx", "gy", "gz")})
        if np.degrees(np.linalg.norm(mon["bias"])) > 2.0:
            raise RuntimeError(f"the robot is not still before the run: gyro reads "
                               f"{np.degrees(np.linalg.norm(mon['bias'])):.1f}°/s")
        mon["idx"] = len(imu.rows)
        with swerve.control("description node: yaw calibration"):
            for i, v in enumerate(vecs):
                mark(f"vector {i + 1}/{len(vecs)} {v}: aligning wheels")
                end = clock() + p.align_s
                while clock() < end:
                    check()
                    swerve.command(v[0] * p.align_speed, v[1] * p.align_speed)
                    sleep(0.02)
                end = clock() + p.settle_s
                while clock() < end:
                    check()
                    swerve.command(0.0, 0.0)
                    sleep(0.02)
                mark(f"vector {i + 1}/{len(vecs)} {v}: {p.cycles} cycles at {p.frequency} Hz, ±{p.amplitude} m")
                ts = clock()
                te = ts + p.cycles / p.frequency
                while (now := clock()) < te:
                    check()
                    s = p.amplitude * w * math.sin(w * (now - ts))
                    swerve.command(v[0] * s, v[1] * s)
                    sleep(0.02)
                end = clock() + p.settle_s
                while clock() < end:
                    check()
                    swerve.command(0.0, 0.0)
                    sleep(0.02)
                windows.append((v, ts, te))
                mark(f"vector {i + 1}/{len(vecs)} {v}: done (heading {mon['heading']:+.2f}°, "
                     f"peak yaw rate {mon['peak']:.1f}°/s)")
        progress("solving")
    except Aborted as e:
        if isinstance(e, _Safety):
            try:
                swerve.estop(f"description node safety abort: {e}")
            except Exception:
                pass
            e2 = Aborted(f"safety abort (swerve e-stop latched): {e}")
        else:
            e2 = Aborted(str(e))
        stop.set()
        mark(f"aborted: {e}")
        e2.raw = recording()
        raise e2 from None
    finally:
        stop.set()
    up_imu = mon["up"]
    I, T = imu.table(), tick.table()
    return solve_yaw(I, T, windows, vecs, p, desc, R_di, up_imu)


def solve_yaw(I: dict, T: dict, windows, vecs, p: YawParams, desc: dict, R_di: np.ndarray,
              up_imu: Optional[np.ndarray]) -> tuple[dict, dict]:
    """Fit each vector's window; returns (meta, raw)."""
    half = 0.5 / p.frequency
    tick_t = T["t_tick"]
    vel = base_velocity(T)
    per = []
    dot = lambda M, v: (M * np.asarray(v)).sum(-1)    # noqa: E731  (matmul warns spuriously on macOS)
    for v, ts, te in windows:
        sel = (I["t_sensor"] >= ts + half) & (I["t_sensor"] <= te - half)
        A = np.column_stack([I["ax"][sel], I["ay"][sel], I["az"][sel]])
        G = np.column_stack([I["gx"][sel], I["gy"][sel], I["gz"][sel]])
        # label: sign of the measured base acceleration along v (smoothed over 5 ticks)
        va = dot(vel, v)
        k = np.ones(5) / 5
        va_s = np.convolve(va, k, mode="same")
        acc = np.gradient(va_s, tick_t) if len(tick_t) > 2 else np.zeros_like(va_s)
        lab = np.sign(np.interp(I["t_sensor"][sel], tick_t, acc))
        fit = fit_motion_line(A, lab)
        d_depth = dot(R_di, fit["direction"]).tolist()
        yaw_rate = np.degrees((G * up_imu).sum(1)) if up_imu is not None and len(G) else np.array([0.0])
        per.append({"vector": v, "t_start": ts, "t_end": te, "n": int(sel.sum()), **fit,
                    "direction_depth": d_depth,
                    "yaw_rate_max_dps": float(np.max(np.abs(yaw_rate))),
                    "gyro_rms_dps": float(np.degrees(np.sqrt(np.mean(G ** 2)))) if len(G) else 0.0})
    checks = {}
    for i in range(len(per)):
        for j in range(i + 1, len(per)):
            exp = angle_deg(per[i]["vector"] + [0], per[j]["vector"] + [0])
            got = angle_deg(per[i]["direction_depth"], per[j]["direction_depth"])
            checks[f"angle_{i}_{j}"] = {"expected_deg": exp, "measured_deg": got, "error_deg": got - exp}
    normal = None
    if len(per) >= 2:   # floor normal from the two most orthogonal lines
        best = max(((i, j) for i in range(len(per)) for j in range(i + 1, len(per))),
                   key=lambda ij: abs(math.sin(math.radians(angle_deg(per[ij[0]]["vector"] + [0], per[ij[1]]["vector"] + [0])))))
        i, j = best
        zb = np.cross(per[i]["vector"] + [0], per[j]["vector"] + [0])
        nc = np.cross(per[i]["direction_depth"], per[j]["direction_depth"])
        normal = (nc * np.sign(zb[2]) / np.linalg.norm(nc)).tolist()
    worst_lin = max(x["linearity"] for x in per)
    worst_sign = min((x["sign_corr"] or 0) for x in per)
    meta = {"params": {**p.__dict__, "vectors": vecs}, "vectors": per, "checks": checks,
            "floor_normal_depth": normal, "gravity_up_imu": None if up_imu is None else up_imu.tolist(),
            "camera": desc, "limits": LIMITS, "version": __version__,
            "summary": f"{len(per)} vectors, worst linearity {worst_lin:.3f}, worst sign corr {worst_sign:.2f}"
                       + "".join(f", {k} err {c['error_deg']:+.2f}°" for k, c in checks.items())}
    raw = {k: v for k, v in I.items()}
    raw = {f"imu_{k}": v for k, v in raw.items()}
    raw.update({f"swerve_{k}": T[k] for k in ("t_tick", "cmd_vx", "cmd_vy", "cmd_omega") if k in T})
    raw["swerve_base_vel"] = vel
    raw["windows"] = np.array([[ts, te] for _, ts, te in windows])
    return meta, raw
