#!/usr/bin/env python3
"""Live browser viewer for an Orbbec depth camera (written for the Gemini 336L).

Run it on the machine the camera is plugged into; open the printed URL from any machine on
the network. Stdlib http.server + numpy + pillow (cv2 is used when importable, for faster
JPEG encoding and more colormaps); the page is inline, no CDN.

    python examples/orbbec_web_viewer.py [--host 0.0.0.0] [--port 8080] [--streams color,depth]

Features:
    streams     color, depth, left IR, right IR, tiled; profiles come from the device's own
                list (the depth and IR streams must share one resolution/fps, so they have a
                single "stereo mode" setting)
    depth       colormap, invert, range (auto percentiles or manual min/max mm), invalid-pixel
                color, hover to read mm, click to pin a point, optional depth-to-color
                alignment (SDK software AlignFilter; depth then has the color resolution)
    device      exposure / gain / white balance / laser controls, as far as the device
                reports them supported (shown with the device's ranges)
    misc        per-stream fps and frame age, client display fps, JPEG snapshots and a
                16-bit PNG of the raw depth, an MJPEG URL per stream (/stream/<name>.mjpg)

Settings are shared by every viewer (frames are encoded once per frame and fanned out).
Changing a stream or profile restarts the pipeline; a profile the device rejects is
reported and the previous one restored. If the camera stops delivering (unplugged, USB
error), the server reports it in the page and keeps trying to reopen it.

Color MJPG frames are passed through unchanged; depth/IR frames are encoded on demand,
only while someone is watching. Ctrl-C to quit.
"""
import argparse
import io
import json
import logging
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, urlparse

import numpy as np
from PIL import Image
from pyorbbecsdk import (AlignFilter, Config, Context, OBFrameType, OBPermissionType, OBPropertyID,
                         OBSensorType, OBStreamType, Pipeline)

try:
    import cv2
except ImportError:  # optional: PIL encodes, fewer colormaps, no YUYV color profiles
    cv2 = None

log = logging.getLogger("orbbec_web_viewer")

# name -> (label, sensor, frame type)
STREAMS = {
    "color": ("Color", OBSensorType.COLOR_SENSOR, OBFrameType.COLOR_FRAME),
    "depth": ("Depth", OBSensorType.DEPTH_SENSOR, OBFrameType.DEPTH_FRAME),
    "ir_left": ("Left IR", OBSensorType.LEFT_IR_SENSOR, OBFrameType.LEFT_IR_FRAME),
    "ir_right": ("Right IR", OBSensorType.RIGHT_IR_SENSOR, OBFrameType.RIGHT_IR_FRAME),
}
STEREO = ("depth", "ir_left", "ir_right")    # share one resolution/fps on the device
COLOR_FORMATS = ["MJPG", "RGB", "BGR", "RGBA", "BGRA", "Y8"] + (["YUYV", "UYVY"] if cv2 else [])
STEREO_FORMATS = {"depth": ["Y16"], "ir_left": ["Y8", "Y16"], "ir_right": ["Y8", "Y16"]}

# Device controls shown when the device reports them read/write: (group, property id, label)
CONTROLS = [
    ("color", "OB_PROP_COLOR_AUTO_EXPOSURE_BOOL", "Auto exposure"),
    ("color", "OB_PROP_COLOR_EXPOSURE_INT", "Exposure"),
    ("color", "OB_PROP_COLOR_GAIN_INT", "Gain"),
    ("color", "OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL", "Auto white balance"),
    ("color", "OB_PROP_COLOR_WHITE_BALANCE_INT", "White balance"),
    ("color", "OB_PROP_COLOR_BRIGHTNESS_INT", "Brightness"),
    ("depth / IR", "OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL", "Auto exposure"),
    ("depth / IR", "OB_PROP_DEPTH_EXPOSURE_INT", "Exposure"),
    ("depth / IR", "OB_PROP_DEPTH_GAIN_INT", "Gain"),
    ("depth / IR", "OB_PROP_LASER_CONTROL_INT", "Laser control"),
    ("depth / IR", "OB_PROP_LASER_POWER_LEVEL_CONTROL_INT", "Laser power level"),
]

DISPLAY_DEFAULTS = {"colormap": "turbo", "invert": False, "range_mode": "auto", "pct_lo": 2.0,
                    "pct_hi": 98.0, "min_mm": 200.0, "max_mm": 3000.0, "invalid_color": "#000000",
                    "jpeg_quality": 80}
PIPELINE_KEYS = ("streams", "color_profile", "stereo_mode", "align")
STALL_S = 4.0         # no frames for this long while streaming -> treat the camera as lost
STARTUP_S = 8.0       # allowance for the first frame after a (re)start
RETRY_S = 2.0         # between attempts to (re)open the camera


# -- colormaps / encoding ------------------------------------------------------------------

def _jet(x):
    return np.stack([np.clip(1.5 - abs(4 * x - k), 0, 1) for k in (3, 2, 1)], -1)


def _turbo(x):   # polynomial approximation (Mikhailov, Google), used when cv2 is missing
    r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))))
    g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))))
    b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))))
    return np.clip(np.stack([r, g, b], -1), 0, 1)


def _build_colormaps():
    x = np.linspace(0, 1, 256)
    maps = {"turbo": _turbo(x), "jet": _jet(x), "gray": np.stack([x, x, x], -1)}
    maps = {k: (v * 255 + 0.5).astype(np.uint8) for k, v in maps.items()}
    if cv2 is not None:
        ramp = np.arange(256, dtype=np.uint8).reshape(256, 1)
        for name in ("turbo", "jet", "inferno", "viridis", "magma", "plasma", "bone", "hot", "rainbow"):
            cm = getattr(cv2, "COLORMAP_" + name.upper(), None)
            if cm is not None:
                maps[name] = cv2.applyColorMap(ramp, cm)[:, 0, ::-1].copy()   # BGR -> RGB
    return maps


COLORMAPS = _build_colormaps()


def encode_jpeg(img, quality):
    """img: HxW gray or HxWx3 RGB uint8."""
    if cv2 is not None:
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return buf.tobytes()
    out = io.BytesIO()
    Image.fromarray(img).save(out, "JPEG", quality=int(quality))
    return out.getvalue()


def stretch16(a):
    """uint16 image -> uint8 using its 1..99th percentile (sampled)."""
    s = a[::4, ::4]
    lo, hi = np.percentile(s, [1, 99]) if s.size else (0, 1)
    hi = max(hi, lo + 1)
    return np.clip((a.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)


def colorize_depth(mm, valid, ds):
    """mm: HxW float32 millimetres; returns (RGB image, lo, hi)."""
    if ds["range_mode"] == "manual":
        lo, hi = float(ds["min_mm"]), float(ds["max_mm"])
    else:
        sample = mm[::4, ::4][valid[::4, ::4]]
        lo, hi = (np.percentile(sample, [ds["pct_lo"], ds["pct_hi"]]) if sample.size > 50 else (0.0, 1.0))
        lo, hi = float(lo), float(hi)
    hi = max(hi, lo + 1.0)
    idx = np.clip((mm - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)
    if ds["invert"]:
        idx = 255 - idx
    rgb = COLORMAPS.get(ds["colormap"], COLORMAPS["gray"])[idx]
    c = ds["invalid_color"].lstrip("#")
    rgb[~valid] = (int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16))
    return rgb, lo, hi


def to_rgb(raw):
    """Decode a non-MJPG color/IR frame to something encode_jpeg takes."""
    a, w, h, fmt = raw.data, raw.w, raw.h, raw.fmt
    if fmt == "Y8":
        return a.reshape(h, w)
    if fmt == "Y16":
        return stretch16(a.view(np.uint16).reshape(h, w))
    if fmt in ("RGB", "BGR"):
        img = a.reshape(h, w, 3)
        return img if fmt == "RGB" else np.ascontiguousarray(img[..., ::-1])
    if fmt in ("RGBA", "BGRA"):
        img = a.reshape(h, w, 4)
        return np.ascontiguousarray(img[..., :3] if fmt == "RGBA" else img[..., 2::-1])
    if fmt in ("YUYV", "UYVY") and cv2 is not None:
        code = cv2.COLOR_YUV2RGB_YUYV if fmt == "YUYV" else cv2.COLOR_YUV2RGB_UYVY
        return cv2.cvtColor(a.reshape(h, w, 2), code)
    raise ValueError(f"unsupported format {fmt}")


BYTES_PER_PIXEL = {"Y8": 1, "Y16": 2, "RGB": 3, "BGR": 3, "RGBA": 4, "BGRA": 4, "YUYV": 2, "UYVY": 2}


# -- camera ----------------------------------------------------------------------------------

@dataclass
class Raw:
    seq: int
    name: str
    fmt: str
    w: int
    h: int
    data: object          # np.uint8 array (or JPEG bytes for MJPG); depth: HxW uint16
    t_frame: float        # host time the SDK received the frame (s)
    scale: float = 1.0    # depth: mm per unit
    aligned: bool = False


@dataclass
class StreamStats:
    times: deque = field(default_factory=lambda: deque(maxlen=30))
    w: int = 0
    h: int = 0
    fmt: str = ""

    def fps(self):
        if len(self.times) < 2 or time.monotonic() - self.times[-1] > 2.0:
            return 0.0
        return (len(self.times) - 1) / max(self.times[-1] - self.times[0], 1e-6)


class Camera:
    """Owns the SDK objects; a single thread opens/(re)starts the pipeline and ingests frames."""

    def __init__(self, streams):
        self.settings = {"streams": {n: n in streams for n in STREAMS}, "color_profile": None,
                         "stereo_mode": None, "align": False, **DISPLAY_DEFAULTS}
        self.settings_version = 0
        self.active = None            # settings the pipeline is running with
        self.lock = threading.RLock()     # settings / status
        self.dev_lock = threading.RLock()  # SDK device handle (open/close/start vs. property access)
        self.cond = threading.Condition()  # new frames
        self.restart = threading.Event()
        self.stopping = threading.Event()
        self.ctx = self.dev = self.pipe = self.align = None
        self.running = False
        self.state, self.error = "starting", None
        self.events = deque(maxlen=12)
        self.info, self.catalog, self.catalog_id = {}, {"color": [], "stereo": [], "available": []}, 0
        self._profiles = {}
        self.raw, self.enc = {}, {}
        self.enc_locks = {n: threading.Lock() for n in STREAMS}
        self.stats = {n: StreamStats() for n in STREAMS}
        self.seq = 0
        self.last_frame = 0.0
        self.thread = threading.Thread(target=self._run, name="camera", daemon=True)

    # -- status ------------------------------------------------------------------------------

    def _event(self, msg, error=False):
        (log.warning if error else log.info)(msg)
        with self.lock:
            self.events.append({"t": time.time(), "msg": msg, "error": error})
            if error:
                self.error = msg

    def _set_state(self, state, clear_error=False):
        with self.lock:
            self.state = state
            if clear_error:
                self.error = None

    # -- lifecycle ---------------------------------------------------------------------------

    def start(self):
        self.thread.start()

    def close(self):
        self.stopping.set()
        self.restart.set()
        self.thread.join(timeout=5)
        with self.cond:
            self.cond.notify_all()

    def _run(self):
        while not self.stopping.is_set():
            try:
                if self.pipe is None:
                    self._open()
                self._serve()
            except _Quiet:
                pass
            except Exception as e:  # camera lost / SDK error: report, drop everything, retry
                if not self.stopping.is_set():
                    self._event(f"camera error: {e}", error=True)
                    self._set_state("reconnecting")
            self._teardown()
            self.stopping.wait(RETRY_S)
        self._teardown()

    def _open(self):
        with self.dev_lock:
            if self.ctx is None:
                self.ctx = Context()    # keep it referenced: devices hold a pointer into it
            devices = self.ctx.query_devices()
            if devices.get_count() == 0:
                with self.lock:
                    if self.state != "no device":
                        self._event("no camera found; waiting for one to be plugged in", error=True)
                    self.state = "no device"
                raise _Quiet()
            self.dev = devices.get_device_by_index(0)
            self.pipe = Pipeline(self.dev)
            info = self.dev.get_device_info()
            self.info = {"name": info.get_name(), "serial": info.get_serial_number(),
                         "firmware": info.get_firmware_version(),
                         "connection": _try(info.get_connection_type, ""), "cv2": cv2 is not None}
            self._build_catalog()
        self._event(f"opened {self.info['name']} ({self.info['serial']}, fw {self.info['firmware']}, "
                    f"{self.info['connection']})")
        with self.lock:
            want = self._fix_settings()
        self._start(want, first=True)

    def _build_catalog(self):
        profiles, color, stereo_sets, available = {}, {}, [], []
        for name, (_, sensor, _) in STREAMS.items():
            try:
                plist = self.pipe.get_stream_profile_list(sensor)
            except Exception:
                continue    # the device has no such sensor
            available.append(name)
            default = plist.get_default_video_stream_profile()
            dkey = (default.get_width(), default.get_height(), default.get_fps(), default.get_format().name)
            modes = {}
            for i in range(plist.get_count()):
                p = plist.get_stream_profile_by_index(i).as_video_stream_profile()
                k = (p.get_width(), p.get_height(), p.get_fps(), p.get_format().name)
                profiles[(name, k)] = p
                if name == "color" and k[3] in COLOR_FORMATS:
                    color[f"{k[0]}x{k[1]}@{k[2]} {k[3]}"] = (k, k == dkey)
                elif name in STEREO and k[3] in STEREO_FORMATS[name]:
                    modes[f"{k[0]}x{k[1]}@{k[2]}"] = (k[:3], k == dkey)
            if name in STEREO:
                stereo_sets.append(modes)

        def order(item):
            (w, h, fps, *fmt), _ = item[1]
            return (-w * h, -fps, COLOR_FORMATS.index(fmt[0]) if fmt else 0)

        stereo = {}
        if stereo_sets:
            common = set.intersection(*(set(m) for m in stereo_sets))
            stereo = {k: (stereo_sets[0][k][0], any(m[k][1] for m in stereo_sets)) for k in common}
        self._profiles = profiles
        self.catalog = {
            "available": available,
            "color": [{"key": k, "default": d} for k, (_, d) in sorted(color.items(), key=order)],
            "stereo": [{"key": k, "default": d} for k, (_, d) in sorted(stereo.items(), key=order)],
            "_color": {k: v[0] for k, v in color.items()},
            "_stereo": {k: v[0] for k, v in stereo.items()},
        }
        self.catalog_id += 1

    def _fix_settings(self):
        """Make the settings valid for the opened device (defaults for unknown profiles)."""
        s, c = self.settings, self.catalog
        for n in STREAMS:
            s["streams"][n] = bool(s["streams"].get(n)) and n in c["available"]
        if s["color_profile"] not in c["_color"]:
            s["color_profile"] = next((p["key"] for p in c["color"] if p["default"]),
                                      c["color"][0]["key"] if c["color"] else None)
        if s["stereo_mode"] not in c["_stereo"]:
            s["stereo_mode"] = next((p["key"] for p in c["stereo"] if p["default"]),
                                    c["stereo"][0]["key"] if c["stereo"] else None)
        self.settings_version += 1
        return _copy(s)

    def _profile(self, name, s):
        if name == "color":
            return self._profiles[("color", self.catalog["_color"][s["color_profile"]])]
        w, h, fps = self.catalog["_stereo"][s["stereo_mode"]]
        for fmt in STEREO_FORMATS[name]:
            p = self._profiles.get((name, (w, h, fps, fmt)))
            if p is not None:
                return p
        raise ValueError(f"{STREAMS[name][0]} has no {w}x{h}@{fps} profile")

    def _start(self, s, first=False):
        """(Re)start the pipeline with settings s; on rejection restore the previous settings."""
        try:
            self._start_pipeline(s)
            with self.lock:
                self.active = s
            self._set_state("streaming" if self.running else "idle", clear_error=True)
            self._event("streaming " + _describe(s) if self.running else "all streams off")
        except Exception as e:
            prev = self.active
            if first or prev is None or self.stopping.is_set():
                raise
            self._event(f"could not start {_describe(s)}: {e}; restoring the previous settings", error=True)
            with self.lock:
                for k in PIPELINE_KEYS:
                    self.settings[k] = _copy(prev[k])
                self.settings_version += 1
            self._start_pipeline(prev)
            self._set_state("streaming" if self.running else "idle")

    def _start_pipeline(self, s):
        with self.dev_lock:
            self._stop_pipeline()
            names = [n for n in STREAMS if s["streams"].get(n)]
            with self.cond:   # drop frames of streams that are now off or change shape
                self.raw.clear()
                self.enc.clear()
            if not names:
                return
            cfg = Config()
            for n in names:
                cfg.enable_stream(self._profile(n, s))
            self._set_state("starting")
            self.pipe.start(cfg)
            self.running = True
            self.align = (AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
                          if s["align"] and s["streams"]["depth"] and s["streams"]["color"] else None)
            self.last_frame = time.monotonic() + STARTUP_S - STALL_S

    def _stop_pipeline(self):
        if self.running:
            self.running = False
            try:
                self.pipe.stop()
            except Exception as e:
                log.debug("pipeline stop: %s", e)

    def _teardown(self):
        with self.dev_lock:
            if self.pipe is not None:
                self._stop_pipeline()
            self.pipe = self.dev = self.align = None
            with self.cond:
                self.raw.clear()
                self.enc.clear()
                self.cond.notify_all()

    def _serve(self):
        while not self.stopping.is_set():
            if self.restart.is_set():
                self.restart.clear()
                with self.lock:
                    want = _copy(self.settings)
                self._start(want)
            if not self.running:
                self.restart.wait(0.5)
                continue
            fs = self.pipe.wait_for_frames(200)
            if fs is None:
                if time.monotonic() - self.last_frame > STALL_S:
                    raise RuntimeError(f"no frames for {STALL_S:.0f} s (unplugged or USB error?)")
                continue
            self.last_frame = time.monotonic()
            self._ingest(fs)

    # -- frames ------------------------------------------------------------------------------

    def _ingest(self, fs):
        s = self.active
        out = []
        for name, (_, _, ftype) in STREAMS.items():
            if not s["streams"].get(name):
                continue
            f = fs.get_frame(ftype)
            if f is None:
                continue
            aligned = False
            if name == "depth" and self.align is not None:
                if fs.get_color_frame() is None:
                    continue
                res = self.align.process(fs)     # None until it has a depth+color pair
                f = res.as_frame_set().get_depth_frame() if res is not None else None
                if f is None:
                    continue
                aligned = True
            raw = self._to_raw(name, f, aligned)
            if raw is not None:
                out.append(raw)
        if not out:
            return
        now = time.monotonic()
        with self.cond:
            for raw in out:
                self.seq += 1
                raw.seq = self.seq
                self.raw[raw.name] = raw
                st = self.stats[raw.name]
                st.times.append(now)
                st.w, st.h, st.fmt = raw.w, raw.h, raw.fmt
            self.cond.notify_all()

    def _to_raw(self, name, f, aligned):
        vf = f.as_video_frame()
        w, h, fmt = vf.get_width(), vf.get_height(), vf.get_format().name
        data = np.frombuffer(vf.get_data(), np.uint8)
        t_frame = vf.get_system_timestamp_us() / 1e6
        if fmt == "MJPG":
            if data.size < 4 or data[0] != 0xFF or data[1] != 0xD8:
                return None
            return Raw(0, name, fmt, w, h, data.tobytes(), t_frame)
        bpp = BYTES_PER_PIXEL.get(fmt)
        if bpp is None or data.size != w * h * bpp:
            log.debug("%s: dropped %s frame of %d bytes (%dx%d)", name, fmt, data.size, w, h)
            return None
        if name == "depth":
            scale = float(f.as_depth_frame().get_depth_scale())
            return Raw(0, name, fmt, w, h, data.view(np.uint16).reshape(h, w).copy(), t_frame, scale, aligned)
        return Raw(0, name, fmt, w, h, data.copy(), t_frame)

    def wait_newer(self, last, timeout):
        """Block until some stream has a frame newer than last[name]; returns the raw frames."""
        with self.cond:
            self.cond.wait_for(lambda: self.stopping.is_set() or any(
                r.seq > last.get(n, 0) for n, r in self.raw.items()), timeout)
            return {n: r for n, r in self.raw.items() if r.seq > last.get(n, 0)}

    def encoded(self, name):
        """(raw, jpeg bytes, meta) for the latest frame of a stream; encoded once per frame."""
        with self.cond:
            raw = self.raw.get(name)
        if raw is None:
            return None
        with self.enc_locks[name]:
            hit = self.enc.get(name)
            if hit is not None and hit[0].seq == raw.seq:
                return hit
            with self.lock:
                ds = {k: self.settings[k] for k in DISPLAY_DEFAULTS}
            meta = {"type": "frame", "stream": name, "seq": raw.seq, "w": raw.w, "h": raw.h, "fmt": raw.fmt,
                    "age_ms": round((time.time() - raw.t_frame) * 1000, 1), "aligned": raw.aligned}
            if raw.fmt == "MJPG":
                jpeg = raw.data
            elif name == "depth":
                valid = raw.data > 0
                mm = raw.data.astype(np.float32)
                if raw.scale != 1.0:
                    mm *= raw.scale
                rgb, lo, hi = colorize_depth(mm, valid, ds)
                meta.update(lo=round(lo), hi=round(hi), valid=round(float(valid.mean()), 3))
                jpeg = encode_jpeg(rgb, ds["jpeg_quality"])
            else:
                jpeg = encode_jpeg(to_rgb(raw), ds["jpeg_quality"])
            hit = (raw, jpeg, meta)
            self.enc[name] = hit
            return hit

    def depth_at(self, x, y):
        with self.cond:
            raw = self.raw.get("depth")
        if raw is None:
            return {"mm": None}
        u = min(max(int(x * raw.w), 0), raw.w - 1)
        v = min(max(int(y * raw.h), 0), raw.h - 1)
        win = raw.data[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3]
        win = win[win > 0]
        return {"u": u, "v": v, "mm": round(float(raw.data[v, u]) * raw.scale, 1) or None,
                "median5": round(float(np.median(win)) * raw.scale, 1) if win.size else None}

    # -- settings / properties ---------------------------------------------------------------

    def update(self, patch):
        if not isinstance(patch, dict):
            raise ValueError("expected a JSON object")
        with self.lock:
            new = _copy(self.settings)
            for k, v in patch.items():
                if k == "streams":
                    if not isinstance(v, dict):
                        raise ValueError("streams must be an object")
                    for n, on in v.items():
                        if n not in STREAMS:
                            raise ValueError(f"unknown stream {n}")
                        if on and n not in self.catalog["available"]:
                            raise ValueError(f"{STREAMS[n][0]} is not available")
                        new["streams"][n] = bool(on)
                elif k == "color_profile":
                    if v not in self.catalog["_color"]:
                        raise ValueError(f"unknown color profile {v!r} (camera not connected?)")
                    new[k] = v
                elif k == "stereo_mode":
                    if v not in self.catalog["_stereo"]:
                        raise ValueError(f"unknown depth/IR mode {v!r} (camera not connected?)")
                    new[k] = v
                elif k in ("align", "invert"):
                    new[k] = bool(v)
                elif k == "colormap":
                    if v not in COLORMAPS:
                        raise ValueError(f"unknown colormap {v!r}")
                    new[k] = v
                elif k == "range_mode":
                    if v not in ("auto", "manual"):
                        raise ValueError("range_mode must be auto or manual")
                    new[k] = v
                elif k in ("pct_lo", "pct_hi", "min_mm", "max_mm"):
                    new[k] = float(v)
                elif k == "jpeg_quality":
                    new[k] = int(min(max(int(v), 30), 95))
                elif k == "invalid_color":
                    if not (isinstance(v, str) and len(v) == 7 and v[0] == "#"):
                        raise ValueError("invalid_color must be #rrggbb")
                    int(v[1:], 16)
                    new[k] = v.lower()
                else:
                    raise ValueError(f"unknown setting {k!r}")
            if not 0 <= new["pct_lo"] < new["pct_hi"] <= 100:
                raise ValueError("need 0 <= low percentile < high percentile <= 100")
            if not 0 <= new["min_mm"] < new["max_mm"]:
                raise ValueError("need 0 <= min mm < max mm")
            restart = any(new[k] != self.settings[k] for k in PIPELINE_KEYS)
            self.settings = new
            self.settings_version += 1
            if restart:
                self.state = "restarting"
        if restart:
            self.restart.set()

    def properties(self):
        out = []
        with self.dev_lock:
            dev = self.dev
            if dev is None:
                return out
            for group, pid, label in CONTROLS:
                prop = getattr(OBPropertyID, pid, None)
                try:
                    if prop is None or not dev.is_property_supported(prop, OBPermissionType.PERMISSION_READ_WRITE):
                        continue
                    if pid.endswith("_BOOL"):
                        out.append({"group": group, "id": pid, "label": label, "type": "bool",
                                    "value": bool(dev.get_bool_property(prop))})
                    else:
                        r = dev.get_int_property_range(prop)
                        out.append({"group": group, "id": pid, "label": label, "type": "int",
                                    "value": int(dev.get_int_property(prop)), "min": r.min, "max": r.max,
                                    "step": r.step, "default": r.default_value})
                except Exception as e:
                    out.append({"group": group, "id": pid, "label": label, "type": "error", "error": str(e)})
        return out

    def set_property(self, pid, value):
        if pid not in {c[1] for c in CONTROLS}:
            raise ValueError(f"unknown property {pid!r}")
        with self.dev_lock:
            if self.dev is None:
                raise ValueError("camera not connected")
            prop = getattr(OBPropertyID, pid)
            if pid.endswith("_BOOL"):
                self.dev.set_bool_property(prop, bool(value))
            else:
                self.dev.set_int_property(prop, int(value))
        self._event(f"set {pid} = {value}")

    def status(self):
        now = time.time()
        with self.lock:
            st = {"state": self.state, "error": self.error, "events": list(self.events), "device": self.info,
                  "settings": self.settings, "settings_version": self.settings_version,
                  "catalog_id": self.catalog_id, "server_time": now}
        with self.cond:
            st["streams"] = {n: {"label": STREAMS[n][0], "available": n in self.catalog["available"],
                                 "fps": round(s.fps(), 1), "w": s.w, "h": s.h, "fmt": s.fmt,
                                 "age_ms": (round((now - self.raw[n].t_frame) * 1000, 1)
                                            if n in self.raw else None)}
                             for n, s in self.stats.items()}
        return st

    def public_catalog(self):
        stops = {k: ["#%02x%02x%02x" % tuple(v[i]) for i in range(0, 256, 32)] + ["#%02x%02x%02x" % tuple(v[255])]
                 for k, v in COLORMAPS.items()}
        return {"color": self.catalog["color"], "stereo": self.catalog["stereo"],
                "available": self.catalog["available"], "colormaps": stops, "catalog_id": self.catalog_id}


class _Quiet(Exception):
    """Already reported (e.g. no device yet)."""


def _try(fn, default):
    try:
        return fn()
    except Exception:
        return default


def _copy(x):
    return json.loads(json.dumps(x))


def _describe(s):
    parts = []
    if s["streams"].get("color"):
        parts.append(f"color {s['color_profile']}")
    stereo = [STREAMS[n][0].lower() for n in STEREO if s["streams"].get(n)]
    if stereo:
        parts.append(f"{'+'.join(stereo)} {s['stereo_mode']}")
    if s["align"] and s["streams"].get("depth") and s["streams"].get("color"):
        parts.append("depth aligned to color")
    return ", ".join(parts)


# -- HTTP ------------------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    server_version = "OrbbecWebViewer/1"
    cam: Camera = None   # set in main()

    def log_message(self, fmt, *args):
        log.debug("%s %s", self.address_string(), fmt % args)

    def _send(self, code, body, ctype, headers=()):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[-1] for k, v in parse_qs(url.query).items()}
        path = url.path
        try:
            if path == "/":
                self._send(200, PAGE.encode(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(self.cam.status())
            elif path == "/api/catalog":
                self._json(self.cam.public_catalog())
            elif path == "/api/props":
                self._json(self.cam.properties())
            elif path == "/api/depth_at":
                self._json(self.cam.depth_at(float(q.get("x", 0.5)), float(q.get("y", 0.5))))
            elif path == "/api/live":
                self._live(q.get("s", ",".join(STREAMS)).split(","))
            elif path.startswith("/stream/") and path.endswith(".mjpg"):
                self._mjpeg(path[len("/stream/"):-len(".mjpg")])
            elif path.startswith("/snapshot/"):
                self._snapshot(path[len("/snapshot/"):])
            elif path == "/favicon.ico":
                self._send(204, b"", "image/x-icon")
            else:
                self._json({"error": "not found"}, 404)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            pass
        except ValueError as e:
            self._json({"error": str(e)}, 400)

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n > 65536:
                raise ValueError("body too large")
            body = json.loads(self.rfile.read(n) or b"{}")
            if self.path == "/api/settings":
                self.cam.update(body)
                self._json({"ok": True})
            elif self.path == "/api/props":
                self.cam.set_property(body.get("id"), body.get("value"))
                self._json(self.cam.properties())
            else:
                self._json({"error": "not found"}, 404)
        except (ValueError, TypeError) as e:
            self._json({"error": str(e)}, 400)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:   # SDK refused (e.g. exposure while auto exposure is on)
            self._json({"error": f"{type(e).__name__}: {e}"}, 500)

    def _frames(self, names):
        """Yield (raw, jpeg, meta) for new frames of the given streams, or None every ~2 s idle."""
        names = [n for n in names if n in STREAMS]
        last, idle = {}, time.monotonic()
        self.connection.settimeout(15)   # a stalled client must not hold the thread forever
        while not self.cam.stopping.is_set():
            new = self.cam.wait_newer({n: last.get(n, 0) for n in names}, timeout=1.0)
            sent = False
            for n in names:
                if n in new:
                    hit = self.cam.encoded(n)
                    if hit is not None and hit[0].seq > last.get(n, 0):
                        last[n] = hit[0].seq
                        sent = True
                        yield hit
            if sent:
                idle = time.monotonic()
            elif time.monotonic() - idle > 2.0:
                idle = time.monotonic()
                yield None

    def _live(self, names):
        """All streams over one connection: [u32 header length][JSON header][JPEG] ..."""
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        for hit in self._frames(names):
            if hit is None:
                hdr, body = json.dumps({"type": "ping", "len": 0}).encode(), b""
            else:
                _, body, meta = hit
                hdr = json.dumps({**meta, "len": len(body)}).encode()
            self.wfile.write(struct.pack(">I", len(hdr)) + hdr + body)

    def _mjpeg(self, name):
        if name not in STREAMS:
            raise ValueError(f"unknown stream {name!r}")
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        for hit in self._frames([name]):
            if hit is not None:
                body = hit[1]
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(body)
                                 + body + b"\r\n")

    def _snapshot(self, fname):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        if fname == "depth.png":    # raw 16-bit depth (units: depth scale, normally mm)
            with self.cam.cond:
                raw = self.cam.raw.get("depth")
            if raw is None:
                raise ValueError("no depth frame")
            out = io.BytesIO()
            Image.fromarray(raw.data).save(out, "PNG")
            self._send(200, out.getvalue(), "image/png",
                       [("Content-Disposition", f'attachment; filename="depth_raw_{stamp}.png"'),
                        ("X-Depth-Scale-mm", str(raw.scale))])
            return
        name = fname.rsplit(".", 1)[0]
        if name not in STREAMS or not fname.endswith(".jpg"):
            raise ValueError(f"unknown snapshot {fname!r}")
        hit = self.cam.encoded(name)
        if hit is None:
            raise ValueError(f"no {name} frame")
        self._send(200, hit[1], "image/jpeg",
                   [("Content-Disposition", f'attachment; filename="{name}_{stamp}.jpg"')])


def lan_addresses():
    addrs = []
    try:
        addrs = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout.split()
    except Exception:
        pass
    if not addrs:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("192.0.2.1", 9))   # no packet is sent; picks the outgoing interface
                addrs = [s.getsockname()[0]]
        except OSError:
            pass
    return [a for a in addrs if ":" not in a] + [socket.gethostname()]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0", help="address to bind (default: all interfaces)")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--streams", default="color,depth",
                        help="streams on at start, comma separated: " + ",".join(STREAMS))
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    streams = [s for s in args.streams.split(",") if s]
    unknown = set(streams) - set(STREAMS)
    if unknown:
        parser.error(f"unknown stream(s) {', '.join(sorted(unknown))}")

    cam = Camera(streams)
    Handler.cam = cam
    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    cam.start()
    hosts = lan_addresses() if args.host in ("0.0.0.0", "") else [args.host]
    print("Orbbec web viewer; open one of:", flush=True)
    for h in hosts:
        print(f"    http://{h}:{args.port}/", flush=True)
    print("Ctrl-C to quit.", flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        cam.close()
        server.server_close()


# -- page ------------------------------------------------------------------------------------

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Orbbec viewer</title>
<style>
:root{--bg:#111418;--panel:#1a1f26;--line:#2b323c;--fg:#dde3ea;--dim:#8a96a3;--acc:#4ea1ff;--err:#ff6b6b;--ok:#4cd28a;--warn:#f5c04e}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.4 system-ui,sans-serif}
header{display:flex;gap:12px;align-items:center;padding:8px 14px;border-bottom:1px solid var(--line);flex-wrap:wrap}
header h1{font-size:15px;margin:0}#dev{color:var(--dim)}
.pill{padding:2px 8px;border-radius:10px;background:var(--line);font-weight:600}
.pill.streaming{background:#1d4d33;color:var(--ok)}.pill.err{background:#4d1d1d;color:var(--err)}.pill.busy{background:#4d401d;color:var(--warn)}
#banner{display:none;background:#4d1d1d;color:#ffd0d0;padding:6px 14px;border-bottom:1px solid #6d2d2d}
#main{display:flex;align-items:flex-start}
aside{width:290px;flex:none;padding:10px;border-right:1px solid var(--line);min-height:calc(100vh - 42px)}
section.box{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:8px 10px;margin-bottom:10px}
section.box h2{font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:var(--dim);margin:0 0 6px}
label.row,div.row{display:flex;align-items:center;gap:6px;margin:4px 0}
label.row span.k,div.row span.k{flex:1;color:var(--dim)}
select,input[type=number]{background:#0d1014;color:var(--fg);border:1px solid var(--line);border-radius:4px;padding:2px 4px;font:inherit}
input[type=number]{width:74px}select{max-width:170px}input[type=range]{width:100%}
input[type=color]{width:40px;height:22px;border:none;background:none;padding:0}
.bar{height:10px;border-radius:3px;margin:4px 0}
.small{font-size:11px;color:var(--dim)}
#tiles{flex:1;display:grid;grid-template-columns:repeat(auto-fit,minmax(380px,1fr));gap:10px;padding:10px;align-content:start}
.tile{background:var(--panel);border:1px solid var(--line);border-radius:6px;overflow:hidden}
.tile .hd{display:flex;gap:8px;align-items:center;padding:4px 8px;border-bottom:1px solid var(--line);flex-wrap:wrap}
.tile .hd b{font-size:13px}.tile .hd .st{color:var(--dim);flex:1;font-variant-numeric:tabular-nums}
.tile .hd a{color:var(--acc);text-decoration:none;font-size:12px}
.wrap{position:relative;background:#000;line-height:0}
.wrap img{width:100%;height:auto;display:block;min-height:120px;cursor:crosshair}
.ro{position:absolute;left:6px;top:6px;background:#000a;padding:2px 6px;border-radius:4px;line-height:1.4;font-variant-numeric:tabular-nums;pointer-events:none}
.ro:empty{display:none}
.nof{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--dim);line-height:1.4}
.pin{position:absolute;width:14px;height:14px;margin:-7px 0 0 -7px;border:2px solid #fff;border-radius:50%;box-shadow:0 0 0 1px #000;pointer-events:none}
.cb{display:flex;align-items:center;gap:6px;padding:4px 8px}.cb .bar{flex:1;margin:0}
#log div{margin:2px 0}#log .e{color:var(--err)}
@media (max-width:760px){#main{flex-direction:column}aside{width:100%;min-height:0;border-right:none}#tiles{width:100%;grid-template-columns:1fr}}
</style></head><body>
<header><h1>Orbbec viewer</h1><span id="dev">connecting...</span><span id="pill" class="pill">...</span><span id="msg" class="small"></span></header>
<div id="banner"></div>
<div id="main">
<aside>
<section class="box"><h2>Streams</h2><div id="streams"></div>
<div class="small">Settings are shared by everyone viewing this page.</div></section>
<section class="box"><h2>Profiles</h2>
<label class="row"><span class="k">Color</span><select id="color_profile"></select></label>
<label class="row"><span class="k">Depth / IR</span><select id="stereo_mode"></select></label>
<label class="row"><span class="k">Align depth to color</span><input type="checkbox" id="align"></label>
<div class="small">Depth and IR always share one resolution/fps. Aligned depth has the color resolution.</div></section>
<section class="box"><h2>Depth display</h2>
<label class="row"><span class="k">Colormap</span><select id="colormap"></select></label>
<div class="bar" id="cmapbar"></div>
<label class="row"><span class="k">Invert (near = end)</span><input type="checkbox" id="invert"></label>
<label class="row"><span class="k">Range</span><select id="range_mode"><option value="auto">auto (percentiles)</option><option value="manual">manual (mm)</option></select></label>
<div class="row" id="pctrow"><span class="k">Percentiles</span><input type="number" id="pct_lo" min="0" max="100" step="0.5"><input type="number" id="pct_hi" min="0" max="100" step="0.5"></div>
<div class="row" id="mmrow"><span class="k">Min / max mm</span><input type="number" id="min_mm" min="0" step="50"><input type="number" id="max_mm" min="0" step="50"></div>
<label class="row"><span class="k">Invalid pixels</span><input type="color" id="invalid_color"></label>
<label class="row"><span class="k">JPEG quality</span><input type="number" id="jpeg_quality" min="30" max="95" step="5"></label>
<div class="small">Hover depth to read mm; click to pin a point (click again to clear).</div></section>
<section class="box"><h2>Device controls <a href="#" id="propsref" class="small" style="float:right">refresh</a></h2><div id="props" class="small">-</div></section>
<section class="box"><h2>Log</h2><div id="log" class="small"></div></section>
</aside>
<div id="tiles"></div>
</div>
<script>
"use strict";
const $=s=>document.querySelector(s), ORDER=["color","depth","ir_left","ir_right"];
let S=null, CAT=null, catId=null, setVer=-1, tiles={};
const sleep=ms=>new Promise(r=>setTimeout(r,ms)), dec=new TextDecoder();
function el(tag,attrs,...kids){const e=document.createElement(tag);for(const[k,v]of Object.entries(attrs||{})){if(k==="class")e.className=v;else if(k.startsWith("on"))e[k]=v;else e.setAttribute(k,v)}for(const c of kids)e.append(c);return e}
async function api(path,body){
  const r=await fetch(path,body===undefined?{cache:"no-store"}:{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
  const j=await r.json().catch(()=>({error:r.statusText}));
  if(!r.ok)throw new Error(j.error||r.statusText);return j}
let msgT=0;function flash(t,bad){const m=$("#msg");m.textContent=t;m.style.color=bad?"var(--err)":"var(--dim)";clearTimeout(msgT);if(t)msgT=setTimeout(()=>m.textContent="",bad?8000:2500)}
async function post(patch){try{await api("/api/settings",patch);flash("applied")}catch(e){flash(e.message,true)}poll()}

function buildControls(){
  const st=$("#streams");st.innerHTML="";
  for(const n of ORDER){
    const ok=CAT.available.includes(n);
    const cb=el("input",{type:"checkbox",id:"s_"+n});cb.disabled=!ok;
    cb.onchange=()=>post({streams:{[n]:cb.checked}});
    st.append(el("label",{class:"row"},el("span",{class:"k"},S.streams[n].label+(ok?"":" (n/a)")),el("span",{class:"small",id:"fps_"+n}),cb))}
  const fill=(id,list)=>{const s=$("#"+id);s.innerHTML="";for(const p of list)s.append(el("option",{value:p.key},p.key+(p.default?" (default)":"")))};
  fill("color_profile",CAT.color);fill("stereo_mode",CAT.stereo);
  const cm=$("#colormap");cm.innerHTML="";for(const k of Object.keys(CAT.colormaps))cm.append(el("option",{value:k},k));
  for(const id of ["color_profile","stereo_mode","colormap","range_mode","invalid_color"])$("#"+id).onchange=e=>post({[id]:e.target.value});
  for(const id of ["align","invert"])$("#"+id).onchange=e=>post({[id]:e.target.checked});
  for(const id of ["pct_lo","pct_hi","min_mm","max_mm","jpeg_quality"])$("#"+id).onchange=e=>post({[id]:Number(e.target.value)});
}
function grad(name){const c=CAT.colormaps[name]||[];return "linear-gradient(to right,"+c.join(",")+")"}
function syncControls(){
  const s=S.settings;
  for(const n of ORDER){const cb=$("#s_"+n);if(cb)cb.checked=!!s.streams[n]}
  for(const id of ["color_profile","stereo_mode","colormap","range_mode","invalid_color","pct_lo","pct_hi","min_mm","max_mm","jpeg_quality"]){
    const e=$("#"+id);if(e&&document.activeElement!==e&&s[id]!=null)e.value=s[id]}
  $("#align").checked=s.align;$("#invert").checked=s.invert;
  $("#align").disabled=!(s.streams.color&&s.streams.depth);
  $("#pctrow").style.display=s.range_mode==="auto"?"":"none";$("#mmrow").style.display=s.range_mode==="manual"?"":"none";
  $("#cmapbar").style.background=grad(s.colormap);$("#cmapbar").style.transform=s.invert?"scaleX(-1)":"";
  for(const t of Object.values(tiles))if(t.bar){t.bar.style.background=grad(s.colormap);t.bar.style.transform=s.invert?"scaleX(-1)":""}
}
function renderStatus(){
  const d=S.device||{};$("#dev").textContent=d.name?`${d.name} - SN ${d.serial} - fw ${d.firmware} - ${d.connection}`:"no camera";
  const p=$("#pill");p.textContent=S.state;p.className="pill "+(S.state==="streaming"?"streaming":S.state==="idle"?"":["starting","restarting"].includes(S.state)?"busy":"err");
  const b=$("#banner");b.style.display=S.error?"block":"none";b.textContent=S.error||"";
  $("#log").innerHTML="";for(const e of S.events.slice().reverse()){$("#log").append(el("div",{class:e.error?"e":""},new Date(e.t*1000).toLocaleTimeString()+" "+e.msg))}
  for(const n of ORDER){const f=$("#fps_"+n),x=S.streams[n];if(f)f.textContent=x.fps?x.fps.toFixed(1)+" fps":""}
}
function makeTile(n){
  const img=el("img",{alt:n}),ro=el("div",{class:"ro"}),nof=el("div",{class:"nof"},"waiting for frames..."),pin=el("div",{class:"pin"});pin.style.display="none";
  const st=el("span",{class:"st"}),links=el("span",{});
  links.append(el("a",{href:"/snapshot/"+n+".jpg",download:"",title:"save the current frame as JPEG"},"jpg"));
  if(n==="depth"){links.append(" ",el("a",{href:"/snapshot/depth.png",download:"",title:"raw 16-bit depth (mm) as PNG"},"raw png"))}
  links.append(" ",el("a",{href:"/stream/"+n+".mjpg",target:"_blank",title:"MJPEG stream of this sensor alone"},"mjpg"));
  const wrap=el("div",{class:"wrap"},img,nof,pin,ro);
  const t={n,img,ro,nof,pin,st,url:null,frames:[],meta:null,busy:false,pinxy:null};
  const box=el("div",{class:"tile"},el("div",{class:"hd"},el("b",{},S.streams[n].label),st,links),wrap);
  if(n==="depth"){t.lo=el("span",{class:"small"});t.hi=el("span",{class:"small"});t.bar=el("div",{class:"bar"});box.append(el("div",{class:"cb"},t.lo,t.bar,t.hi))}
  const xy=e=>[e.offsetX/img.clientWidth,e.offsetY/img.clientHeight];
  img.onmousemove=e=>{t.hover=xy(e);hover(t)};
  img.onmouseleave=()=>{t.hover=null;t.ro.textContent=pinText(t)};
  img.onclick=e=>{if(n!=="depth")return;t.pinxy=t.pinxy?null:xy(e);t.pinval=null;placePin(t)};
  t.box=box;return t}
function placePin(t){t.pin.style.display=t.pinxy?"":"none";if(t.pinxy){t.pin.style.left=(t.pinxy[0]*100)+"%";t.pin.style.top=(t.pinxy[1]*100)+"%"}}
const fmtmm=r=>r&&r.mm!=null?`${r.mm} mm (5x5 median ${r.median5??"-"})`:"no depth";
function pinText(t){return t.pinxy&&t.pinval?"pin: "+fmtmm(t.pinval):""}
async function hover(t){
  if(!t.hover||!t.meta)return;
  const m=t.meta,u=Math.floor(t.hover[0]*m.w),v=Math.floor(t.hover[1]*m.h);
  if(t.n!=="depth"){t.ro.textContent=`(${u}, ${v})`;return}
  if(t.busy)return;t.busy=true;
  try{const r=await api(`/api/depth_at?x=${t.hover[0]}&y=${t.hover[1]}`);if(t.hover)t.ro.textContent=`(${r.u}, ${r.v}) ${fmtmm(r)}`+(t.pinxy?"\n":"")}catch(e){}
  t.busy=false}
setInterval(async()=>{const t=tiles.depth;if(!t||!t.pinxy)return;
  try{t.pinval=await api(`/api/depth_at?x=${t.pinxy[0]}&y=${t.pinxy[1]}`);if(!t.hover)t.ro.textContent=pinText(t)}catch(e){}},500);
function renderTiles(){
  const want=ORDER.filter(n=>S.settings.streams[n]&&S.streams[n].available);
  for(const n of Object.keys(tiles))if(!want.includes(n)){tiles[n].box.remove();if(tiles[n].url)URL.revokeObjectURL(tiles[n].url);delete tiles[n]}
  for(const n of want){if(!tiles[n]){tiles[n]=makeTile(n);syncControls()}}
  const box=$("#tiles");for(const n of want)box.append(tiles[n].box);  // keeps order
  for(const n of want){const t=tiles[n],x=S.streams[n],now=performance.now();
    t.frames=t.frames.filter(f=>now-f<2000);const dfps=t.frames.length>1?(t.frames.length-1)*1000/(t.frames[t.frames.length-1]-t.frames[0]):0;
    t.st.textContent=x.w?`${x.w}x${x.h} ${x.fmt}${t.meta&&t.meta.aligned?" (aligned)":""} | cam ${x.fps.toFixed(1)} fps | shown ${dfps.toFixed(1)} fps | age ${t.meta?t.meta.age_ms.toFixed(0):"-"} ms`:"no frames";
    if(!x.fps&&S.state!=="streaming"){t.nof.style.display="";t.nof.textContent=S.state}}
}
function onFrame(h,body){
  const t=tiles[h.stream];if(!t)return;
  t.meta=h;t.frames.push(performance.now());t.nof.style.display="none";
  const url=URL.createObjectURL(new Blob([body],{type:"image/jpeg"}));
  const old=t.url;t.url=url;t.img.onload=()=>{if(old)URL.revokeObjectURL(old)};t.img.src=url;
  if(t.lo&&h.lo!=null){t.lo.textContent=h.lo+" mm";t.hi.textContent=h.hi+" mm  (valid "+Math.round(h.valid*100)+"%)"}
}
async function live(){
  for(;;){
    try{
      const r=await fetch("/api/live",{cache:"no-store"});const rd=r.body.getReader();let buf=new Uint8Array(0);
      for(;;){
        const{value,done}=await rd.read();if(done)break;
        const nb=new Uint8Array(buf.length+value.length);nb.set(buf);nb.set(value,buf.length);buf=nb;
        for(;;){
          if(buf.length<4)break;
          const hl=new DataView(buf.buffer,buf.byteOffset,4).getUint32(0);if(buf.length<4+hl)break;
          const h=JSON.parse(dec.decode(buf.subarray(4,4+hl)));if(buf.length<4+hl+h.len)break;
          const body=buf.slice(4+hl,4+hl+h.len);buf=buf.slice(4+hl+h.len);
          if(h.type==="frame")onFrame(h,body)}}
    }catch(e){}
    await sleep(1000)}
}
async function loadProps(){
  let props;try{props=await api("/api/props")}catch(e){$("#props").textContent=e.message;return}
  renderProps(props)}
function renderProps(props){
  const box=$("#props");box.innerHTML="";if(!props.length){box.textContent="none reported";return}
  let group=null;
  for(const p of props){
    if(p.group!==group){group=p.group;box.append(el("div",{style:"margin-top:6px;font-weight:600;color:var(--fg)"},group))}
    const set=async v=>{try{renderProps(await api("/api/props",{id:p.id,value:v}));flash(p.label+" set")}catch(e){flash(p.label+": "+e.message,true);loadProps()}};
    if(p.type==="bool"){const cb=el("input",{type:"checkbox"});cb.checked=p.value;cb.onchange=()=>set(cb.checked);box.append(el("label",{class:"row"},el("span",{class:"k"},p.label),cb))}
    else if(p.type==="int"){
      const num=el("input",{type:"number",min:p.min,max:p.max,step:p.step||1});num.value=p.value;
      const rng=el("input",{type:"range",min:p.min,max:p.max,step:p.step||1});rng.value=p.value;
      rng.oninput=()=>num.value=rng.value;rng.onchange=()=>set(Number(rng.value));num.onchange=()=>set(Number(num.value));
      box.append(el("div",{class:"row"},el("span",{class:"k",title:p.id},`${p.label} [${p.min}..${p.max}]`),num),rng)}
    else box.append(el("div",{class:"row"},el("span",{class:"k"},p.label),el("span",{class:"small"},"error: "+p.error)))}
}
$("#propsref").onclick=e=>{e.preventDefault();loadProps()};
async function poll(){
  try{S=await api("/api/state")}catch(e){const p=$("#pill");p.textContent="server unreachable";p.className="pill err";return}
  if(S.catalog_id!==catId){try{CAT=await api("/api/catalog");catId=S.catalog_id;buildControls();loadProps();setVer=-1}catch(e){return}}
  if(S.settings_version!==setVer){setVer=S.settings_version;syncControls()}
  renderStatus();renderTiles()}
(async()=>{await poll();setInterval(poll,1000);live()})();
</script></body></html>
"""

if __name__ == "__main__":
    main()
