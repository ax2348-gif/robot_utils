"""Offscreen camera rendering for the sim world (docs/sim.md, "Cameras").

One thread renders every camera stream that a backend asked for (``start``; it lapses 2 s after the last
``frames`` request), each at its configured rate in **sim time**: a paused world delivers no new frames.
Each frame is rendered from a copy of the world's state taken under the world lock (the physics never
waits on the GPU): colour as JPEG (what the camera sends as MJPG), depth as uint16 millimetres (Y16,
0 = no depth outside ``depth_range``). ``t_sensor`` is the host monotonic time of that state;
``t_capture`` = ``t_sensor`` + the configured camera-to-host delay (the backend hands the frame over no
earlier). MuJoCo's default GL works headless on macOS (CGL) and Linux with EGL/OSMesa via MUJOCO_GL.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Optional

import mujoco
import numpy as np

log = logging.getLogger("sim.render")

ACTIVE_S = 2.0           # s a camera keeps rendering after its last start / frames request
STREAMS = ("color", "depth")


class Camera:
    def __init__(self, name: str, cfg: dict):
        self.name, self.cfg = name, cfg
        self.streams: set = set()
        self.active_until = 0.0
        self.seq = {s: 0 for s in STREAMS}
        self.latest: dict = {}             # stream -> (meta dict, bytes)
        self.next_due = {s: 0.0 for s in STREAMS}
        self.cond = threading.Condition()
        self.rendered = {s: 0 for s in STREAMS}
        self.render_s = {s: 0.0 for s in STREAMS}     # wall s spent rendering + encoding (sum)

    def profile(self, stream: str) -> dict:
        return self.cfg[stream]

    def intrinsics(self, stream: str) -> dict:
        c = self.cfg[stream]
        w, h = int(c["width"]), int(c["height"])
        f = (h / 2) / math.tan(math.radians(float(c["fovy_deg"])) / 2)     # MuJoCo: fovy, square pixels
        return {"fx": f, "fy": f, "cx": w / 2, "cy": h / 2, "width": w, "height": h}


class Cameras:
    def __init__(self, world, cfg: dict):
        self.world = world
        self.cams = {name: Camera(name, c) for name, c in (cfg or {}).items()}
        self._renderers: dict = {}
        self._gen = -1
        self._rd: Optional[mujoco.MjData] = None
        self._m = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[str] = None
        self._jobs: list = []               # one-off renders (the page's snapshots), run on the render thread

    # -- requests (any thread) ---------------------------------------------------------------------

    def cam(self, name: str) -> Camera:
        if name not in self.cams:
            raise KeyError(f"no camera {name!r}")
        return self.cams[name]

    def info(self, name: str) -> dict:
        c = self.cam(name)
        with self.world.lock:
            d, m = self.world.d, self.world.m
            bd, bc = m.body(c.cfg["depth"]["link"]).id, m.body(c.cfg["color"]["link"]).id
            Rd, td = d.xmat[bd].reshape(3, 3).copy(), d.xpos[bd].copy()
            Rc, tc = d.xmat[bc].reshape(3, 3).copy(), d.xpos[bc].copy()
        # depth_to_color maps a depth-frame point into the colour frame (as the Orbbec SDK): p_c = R p_d + t
        R = Rc.T @ Rd
        t = Rc.T @ (td - tc)
        return {"name": name, "serial": c.cfg.get("serial", f"SIM-{name}"),
                "color": {k: c.cfg["color"][k] for k in ("width", "height", "fps", "fovy_deg", "delay")},
                "depth": {k: c.cfg["depth"][k] for k in ("width", "height", "fps", "fovy_deg", "delay")},
                "intrinsics": {s: c.intrinsics(s) for s in STREAMS},
                "depth_to_color": {"rotation": np.round(R, 9).tolist(), "translation_mm": np.round(t * 1000, 6).tolist()},
                "depth_range": list(c.cfg.get("depth_range", [0.1, 10.0])),
                "imu": ({"rate_hz": round(1.0 / self.world.m.opt.timestep), "delay": float(c.cfg["imu"].get("delay", 0.0))}
                        if name in self.world.imu_adr else None)}

    def start(self, name: str, streams) -> None:
        c = self.cam(name)
        bad = set(streams) - set(STREAMS)
        if bad:
            raise ValueError(f"unknown streams {sorted(bad)}")
        with c.cond:
            c.streams = set(streams)
            c.active_until = time.monotonic() + ACTIVE_S
            c.latest = {s: v for s, v in c.latest.items() if s in c.streams}
        self._ensure_thread()

    def stop(self, name: str) -> None:
        c = self.cam(name)
        with c.cond:
            c.streams = set()
            c.latest = {}

    def frames(self, name: str, after: dict, timeout: float) -> tuple[list, bytes]:
        """Frames newer than ``after`` ({stream: seq}), waiting up to ``timeout`` s for one."""
        c = self.cam(name)
        deadline = time.monotonic() + max(0.0, min(float(timeout), 5.0))
        with c.cond:
            c.active_until = time.monotonic() + ACTIVE_S
            while True:
                new = [(s, v) for s, v in c.latest.items() if v[0]["seq"] > int(after.get(s, 0))]
                left = deadline - time.monotonic()
                if new or left <= 0:
                    break
                c.cond.wait(left)
        metas, blobs = [], []
        for _, (meta, data) in sorted(new):
            metas.append({**meta, "len": len(data)})
            blobs.append(data)
        return metas, b"".join(blobs)

    def snapshot_jpeg(self, name: str, stream: str = "color") -> bytes:
        """One frame rendered now (for the page; works paused). Depth comes colourised."""
        c = self.cam(name)
        out: dict = {}
        done = threading.Event()

        def job():
            try:
                meta, data = self._render(c, stream)
                if stream == "color":
                    out["v"] = data
                else:
                    from ..orbbec.encode import colorize_depth, encode_jpeg
                    raw = np.frombuffer(data, "<u2").reshape(meta["height"], meta["width"])
                    from ..orbbec.models import DisplaySettings
                    img = colorize_depth(raw, 1.0, DisplaySettings())[0]
                    out["v"] = encode_jpeg(img, 80)
            except Exception as e:  # noqa: BLE001
                out["e"] = e
            finally:
                done.set()
        self._jobs.append(job)
        self._ensure_thread()
        if not done.wait(5.0):
            raise TimeoutError("render timed out")
        if "e" in out:
            raise out["e"]
        return out["v"]

    # -- render thread ------------------------------------------------------------------------------

    def _ensure_thread(self) -> None:
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="sim-render", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)

    def _close_renderers(self) -> None:
        """On the render thread only: a GL context is current there (closing one elsewhere deadlocks in CGL)."""
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        self._renderers = {}

    def _loop(self) -> None:
        try:
            self._serve()
        finally:
            self._close_renderers()

    def _serve(self) -> None:
        while not self._stop.is_set():
            did = False
            while self._jobs:
                self._jobs.pop(0)()
                did = True
            now = time.monotonic()
            t = self.world.t
            for c in self.cams.values():
                if now > c.active_until or not c.streams:
                    continue
                for s in sorted(c.streams):
                    period = 1.0 / float(c.cfg[s]["fps"])
                    if c.next_due[s] > t + period:     # sim time went back (a reset to 0): restart the schedule, or the
                        c.next_due[s] = t              # stream waits as long as the world had run (seen 2026-11-24)
                    if t + 1e-9 < c.next_due[s]:
                        continue
                    nxt = c.next_due[s] + period
                    c.next_due[s] = nxt if 0 < c.next_due[s] and nxt > t else t + period   # behind: skip, never burst
                    try:
                        t0 = time.monotonic()
                        meta, data = self._render(c, s)
                        c.render_s[s] += time.monotonic() - t0
                        c.rendered[s] += 1
                        self.error = None
                    except Exception as e:  # noqa: BLE001 - reported in /state, retried next frame
                        if self.error is None:
                            log.exception("rendering %s %s", c.name, s)
                        self.error = f"{type(e).__name__}: {e}"
                        continue
                    with c.cond:
                        if s in c.streams:
                            c.seq[s] += 1
                            meta["seq"] = c.seq[s]
                            c.latest[s] = (meta, data)
                            c.cond.notify_all()
                    did = True
            if not did:
                time.sleep(0.002)

    def _state(self) -> tuple:
        """Copy the world's state for rendering (under its lock); rebuild after a recompile."""
        w = self.world
        with w.lock:
            if w.generation != self._gen or self._m is not w.m:
                self._close_renderers()
                self._m, self._gen = w.m, w.generation
                self._rd = mujoco.MjData(w.m)
            rd = self._rd
            rd.qpos[:] = w.d.qpos
            rd.time = w.d.time
            t_host = time.monotonic()
        mujoco.mj_kinematics(self._m, rd)
        mujoco.mj_camlight(self._m, rd)
        return rd, t_host

    def _renderer(self, w: int, h: int, depth: bool) -> mujoco.Renderer:
        key = (w, h, depth)
        r = self._renderers.get(key)
        if r is None:
            r = mujoco.Renderer(self._m, h, w)
            if depth:
                r.enable_depth_rendering()
            self._renderers[key] = r
        return r

    def _render(self, c: Camera, stream: str) -> tuple[dict, bytes]:
        rd, t_host = self._state()
        p = c.cfg[stream]
        w, h = int(p["width"]), int(p["height"])
        r = self._renderer(w, h, stream == "depth")
        r.update_scene(rd, camera=f"{c.name}_{stream}")
        rc = self.world.cfg.get("render") or {}
        r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = bool(rc.get("shadows", True))
        r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = bool(rc.get("reflections", True))
        img = r.render()
        if stream == "color":
            from ..orbbec.encode import encode_jpeg
            data, fmt = encode_jpeg(np.ascontiguousarray(img), int(c.cfg.get("jpeg_quality", 80))), "MJPG"
        else:
            lo, hi = (float(v) for v in c.cfg.get("depth_range", [0.1, 10.0]))
            mm = np.where((img >= lo) & (img <= hi), np.round(img * 1000.0), 0.0)
            data, fmt = mm.astype("<u2").tobytes(), "Y16"
        delay = float(p.get("delay", 0.0))
        return {"stream": stream, "format": fmt, "width": w, "height": h, "t_sensor": t_host,
                "t_capture": t_host + delay, "t_sim": float(rd.time), "depth_scale": 1.0, "seq": 0}, data
