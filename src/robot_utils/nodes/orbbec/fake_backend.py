"""Synthetic camera for development and tests without hardware (``--fake``).

Same interface as SdkBackend. Color is a moving gradient encoded to JPEG (like the camera's
MJPG), depth a moving uint16 ramp, IR a gray pattern, IMU a slow sine plus gravity. The
``1920x1080@30 MJPG`` color profile is listed but refused at start, to exercise the
restore-on-reject path; ``640x480@30 RGB`` starts but delivers no color frames (like the real camera on USB 2).
With ``dual=True`` it also has the dual_color mode (like a Gemini 305): ``color`` = left, ``color_right`` = right,
the right side uncompressed YUYV only, no depth or IR.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from .encode import encode_jpeg
from .hub import Frame
from .models import Catalog, DeviceInfo, ImuCatalog, ProfileOption, Settings
from .owner import NoDevice
from .protocol import IMU_FIELDS
from .sdk_backend import rate_hz

COLOR = [("1920x1080@30 MJPG", 1920, 1080, 30, False), ("1280x720@10 MJPG", 1280, 720, 10, True),
         ("640x480@30 MJPG", 640, 480, 30, False), ("640x480@30 RGB", 640, 480, 30, False)]
STEREO = [("848x480@10", 848, 480, 10, True), ("640x400@30", 640, 400, 30, False),
          ("480x270@60", 480, 270, 60, False)]
COLOR_RIGHT = [("1280x720@10 YUYV", 1280, 720, 10, True), ("640x480@30 YUYV", 640, 480, 30, False)]
REJECT = {"1920x1080@30 MJPG"}     # refused at start
SILENT = {"640x480@30 RGB"}        # starts but never delivers (like uncompressed color on USB 2)


class FakeBackend:
    def __init__(self, present: bool = True, dual: bool = False):
        self.present = present
        self.modes = ["depth", "dual_color"] if dual else ["depth"]
        self.mode = "depth"
        self.mode_switches = 0
        self.is_open = False
        self.running = False
        self._s: Optional[Settings] = None
        self._next: dict = {}
        self._props = {"OB_PROP_COLOR_AUTO_EXPOSURE_BOOL": True, "OB_PROP_COLOR_EXPOSURE_INT": 156,
                       "OB_PROP_LASER_CONTROL_INT": 1}
        self._imu_thread = None
        self._imu_stop = threading.Event()
        self._jpeg_cache = {}

    def open(self, mode: str = "depth"):
        if not self.present:
            raise NoDevice()
        self.is_open = True
        self.mode = mode if mode in self.modes else "depth"
        return DeviceInfo(name="Fake Gemini", serial="FAKE0001", firmware="0.0", connection="none",
                          global_timestamp=True), self._catalog()

    def _catalog(self) -> Catalog:
        opt = lambda rows: [ProfileOption(key=k, width=w, height=h, fps=f, format=k.split()[1], default=d)  # noqa: E731
                            for k, w, h, f, d in rows]
        dual = self.mode == "dual_color"
        return Catalog(
            modes=list(self.modes), mode=self.mode,
            available=["color", "color_right"] if dual else ["color", "depth", "ir_left", "ir_right"],
            color=opt(COLOR), color_right=opt(COLOR_RIGHT) if dual else [],
            stereo=[] if dual else [ProfileOption(key=k, width=w, height=h, fps=f, default=d)
                                    for k, w, h, f, d in STEREO],
            imu=ImuCatalog(available=True, sample_rates=["50Hz", "100Hz", "200Hz", "500Hz", "1kHz"],
                           default_rate="1kHz", accel_ranges=["4g"], gyro_ranges=["1000dps"]))

    def set_mode(self, mode: str) -> Catalog:
        if mode not in self.modes:
            raise ValueError(f"this device has no {mode!r} mode")
        self.stop()
        self.mode = mode
        self.mode_switches += 1
        return self._catalog()

    def start(self, s: Settings):
        self.stop()
        self._s = s
        if s.streams.get("color") and s.color_profile in REJECT:
            raise RuntimeError("fake device refuses this profile")
        if s.streams.get("color_right") and not any(r[1:4] == self._fps("color") for r in COLOR_RIGHT) \
                and self.mode == "dual_color":
            raise RuntimeError("the right colour sensor has no profile at this size and rate")
        names = [n for n, on in s.streams.items() if on
                 and not (n == "color" and s.color_profile in SILENT)]
        now = time.monotonic()
        self._next = {n: now + 0.05 for n in names}
        self.running = any(s.streams.values())

    def calibration(self, s):
        """Synthetic pinhole intrinsics (~90 deg horizontal field of view), color 25 mm right of depth."""
        out = {}
        for name, key, rows in (("depth", s.stereo_mode, STEREO), ("color", s.color_profile, COLOR)):
            row = next((r for r in rows if r[0] == key), None)
            if row is None:
                continue
            w, h = row[1], row[2]
            f = w / 2.0
            out[name] = {"fx": f, "fy": f, "cx": w / 2.0, "cy": h / 2.0, "width": w, "height": h}
        out["depth_to_color"] = {"rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],   # floats, as the SDK
                                 "translation_mm": [-25.0, 0.0, 0.0]}
        if self.mode == "dual_color":
            out.pop("depth", None)
            out["color_right"] = dict(out["color"])
            out["color_to_color_right"] = {"rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                                           "translation_mm": [-18.0, 0.0, 0.0]}
        return out

    def _fps(self, n):
        key = self._s.color_profile if n in ("color", "color_right") else self._s.stereo_mode
        rows = COLOR if n in ("color", "color_right") else STEREO
        return next(r for r in rows if r[0] == key)[1:4]

    def stop(self):
        self.running = False

    def wait(self, timeout_ms):
        if not self._next:
            time.sleep(timeout_ms / 1000)
            return None
        n_due = min(self._next.values())
        delay = n_due - time.monotonic()
        if delay > timeout_ms / 1000:
            time.sleep(timeout_ms / 1000)
            return None
        if delay > 0:
            time.sleep(delay)
        now = time.monotonic()
        out = []
        for n, due in list(self._next.items()):
            if due > now + 0.001:
                continue
            w, h, fps = self._fps(n)
            self._next[n] = due + 1.0 / fps
            out.append(self._make(n, w, h, due))
        return out

    def _make(self, n, w, h, t):
        phase = int(t * 30) % 256
        if n == "color":
            key = (w, h, phase // 8)
            data = self._jpeg_cache.get(key)
            if data is None:
                x = np.linspace(0, 255, w, dtype=np.float32)[None, :]
                y = np.linspace(0, 255, h, dtype=np.float32)[:, None]
                img = np.stack([np.broadcast_to((x + phase) % 256, (h, w)), np.broadcast_to(y, (h, w)),
                                np.full((h, w), 128, np.float32)], -1).astype(np.uint8)
                data = self._jpeg_cache[key] = encode_jpeg(img, 80)
                if len(self._jpeg_cache) > 64:
                    self._jpeg_cache.clear()
            fr = Frame(n, "MJPG", w, h, data, 0.0, None, 0, 0)
        elif n == "color_right":
            fr = Frame(n, "YUYV", w, h, np.full(w * h * 2, 128, np.uint8), 0.0, None, 0, 0)
        elif n == "depth":
            x = np.arange(w, dtype=np.uint16)[None, :]
            d = (500 + (x * 4 + phase * 8) % 3000).astype(np.uint16)
            d = np.broadcast_to(d, (h, w)).copy()
            d[: h // 10] = 0
            fr = Frame(n, "Y16", w, h, d, 0.0, None, 0, 0, depth_scale=1.0)
        else:
            fr = Frame(n, "Y8", w, h, np.full(w * h, phase, np.uint8), 0.0, None, 0, 0)
        fr.t_sensor = t - 0.02
        fr.t_capture = t
        fr.t_device_us = int(t * 1e6)
        fr.index = int(t * 1000)
        fr.t_owner = time.monotonic()
        return fr

    def close(self):
        self.imu_stop()
        self.stop()
        self.is_open = False

    def properties(self, all_props=False):
        return [{"group": "fake", "id": k, "label": k[8:].lower(), "type": "bool" if k.endswith("BOOL") else "int",
                 "value": v, **({} if k.endswith("BOOL") else {"min": 0, "max": 1000, "step": 1, "default": 0})}
                for k, v in self._props.items()]

    def set_property(self, pid, value):
        if pid not in self._props:
            raise ValueError(f"unknown property {pid!r}")
        self._props[pid] = bool(value) if pid.endswith("BOOL") else int(value)

    @property
    def imu_running(self):
        return self._imu_thread is not None and self._imu_thread.is_alive()

    def imu_start(self, s, sink, on_error, batch_ms):
        self.imu_stop()
        period = 1.0 / rate_hz(s.sample_rate)
        self._imu_stop = stop = threading.Event()

        def loop():
            t, rows, t_first = time.monotonic(), [], 0.0
            while not stop.is_set():
                t += period
                d = t - time.monotonic()
                if d > 0:
                    time.sleep(d)
                rows.append((0, t, t - 0.003, t * 1e6, 0.1 * np.sin(t), -9.81, 0.0, 0.01 * np.cos(t), 0, 0, 30.0))
                if len(rows) == 1:
                    t_first = t
                if (t - t_first) * 1000 + period * 1000 >= batch_ms():
                    sink(np.array(rows, np.float64).reshape(-1, len(IMU_FIELDS)), 0)
                    rows = []

        self._imu_thread = threading.Thread(target=loop, name="fake-imu", daemon=True)
        self._imu_thread.start()

    def imu_stop(self):
        if self._imu_thread is not None:
            self._imu_stop.set()
            self._imu_thread.join(timeout=2)
            self._imu_thread = None
