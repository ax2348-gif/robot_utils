"""Orbbec SDK backend (pyorbbecsdk2). Only the owner thread and the IMU thread it starts use it.

Verified on a Gemini 336L, pyorbbecsdk2 2.1.3 (libOrbbecSDK 2.10.6), Pi 5 USB 2.1:
- Keep Context() referenced while the device is used (chaining off a temporary crashes).
- Depth / left IR / right IR must share resolution+fps.
- AlignFilter.process() returns None for the first framesets.
- A second Pipeline on a streaming device fails for *video*, but works for the IMU: IMU in the
  video pipeline gets batched into the video framesets (half the samples, ~125 ms late), so the
  IMU runs on its own Pipeline in its own thread.
- Frame get_system_timestamp_us(): host wall clock (epoch) at SDK receipt. With
  device.enable_global_timestamp(True), get_global_timestamp_us() is the device timestamp mapped
  to the host wall clock (SDK-internal clock sync), i.e. close to the actual capture instant.
- Frame aggregation (measured, color 1280x720@10 + depth 848x480@10, median wait between SDK
  receipt and wait_for_frames returning):
      default / frame sync on        color 69 ms (waits for its depth), depth 8 ms
      sync off + ANY_SITUATION / FULL_FRAME_REQUIRE / COLOR_FRAME_REQUIRE
                                     color 0.3 ms, depth 140 ms (held for, and paired with, the
                                     *next* color frame: wrong pairing and a frame late)
      sync off + DISABLE             color 0.3 ms, depth 8 ms, one frame per frameset
  so without frame sync the node uses DISABLE.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

import numpy as np

from .hub import Frame
from .models import (STEREO, Catalog, DeviceInfo, ImuCatalog, ImuSettings, ProfileOption, Settings)
from .owner import NoDevice
from .protocol import IMU_FIELDS
from .timesync import epoch_minus_mono

log = logging.getLogger("orbbec_node.sdk")

COLOR_FORMATS = ["MJPG", "RGB", "BGR", "RGBA", "BGRA", "Y8", "YUYV", "UYVY"]
STEREO_FORMATS = {"depth": ["Y16"], "ir_left": ["Y8", "Y16"], "ir_right": ["Y8", "Y16"]}
BYTES_PER_PIXEL = {"Y8": 1, "Y16": 2, "RGB": 3, "BGR": 3, "RGBA": 4, "BGRA": 4, "YUYV": 2, "UYVY": 2}

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


def rate_name(enum_value) -> str:
    """OBGyroSampleRate.SAMPLE_RATE_200_HZ -> '200Hz', SAMPLE_RATE_1_KHZ -> '1kHz'."""
    s = enum_value.name.replace("SAMPLE_RATE_", "")
    num, _, unit = s.rpartition("_")
    return num.replace("_", ".") + ("kHz" if unit == "KHZ" else "Hz")


def rate_hz(name: str) -> float:
    """'200Hz' -> 200.0, '1kHz' -> 1000.0."""
    return float(name[:-3]) * 1000 if name.endswith("kHz") else float(name[:-2])


def range_name(enum_value) -> str:
    """ACCEL_FS_4g -> '4g', FS_1000dps -> '1000dps'."""
    return enum_value.name.replace("ACCEL_FS_", "").replace("FS_", "")


def _try(fn, default):
    try:
        return fn()
    except Exception:
        return default


class SdkBackend:
    def __init__(self):
        import pyorbbecsdk as ob   # imported here so the package works without the SDK (tests, --fake)
        self.ob = ob
        self.ctx = self.dev = self.pipe = self.align = None
        self.running = False
        self.global_ts = False
        self._profiles = {}
        self._color, self._stereo = {}, {}
        self._imu_profiles = {}
        self._active: Optional[Settings] = None
        self._imu_thread: Optional[threading.Thread] = None
        self._imu_stop = threading.Event()
        self.streams = {"color": (ob.OBSensorType.COLOR_SENSOR, ob.OBFrameType.COLOR_FRAME),
                        "depth": (ob.OBSensorType.DEPTH_SENSOR, ob.OBFrameType.DEPTH_FRAME),
                        "ir_left": (ob.OBSensorType.LEFT_IR_SENSOR, ob.OBFrameType.LEFT_IR_FRAME),
                        "ir_right": (ob.OBSensorType.RIGHT_IR_SENSOR, ob.OBFrameType.RIGHT_IR_FRAME)}

    @property
    def is_open(self):
        return self.pipe is not None

    # -- open / catalog ----------------------------------------------------------------------

    def open(self):
        ob = self.ob
        if self.ctx is None:
            self.ctx = ob.Context()     # keep it referenced: devices hold a pointer into it
        devices = self.ctx.query_devices()
        if devices.get_count() == 0:
            raise NoDevice()
        self.dev = devices.get_device_by_index(0)
        self.global_ts = False
        try:
            if self.dev.is_global_timestamp_supported():
                self.dev.enable_global_timestamp(True)
                self.global_ts = True
        except Exception as e:
            log.info("global timestamp not available: %s", e)
        self.pipe = ob.Pipeline(self.dev)
        i = self.dev.get_device_info()
        info = DeviceInfo(name=i.get_name(), serial=i.get_serial_number(), firmware=i.get_firmware_version(),
                          connection=_try(i.get_connection_type, ""), global_timestamp=self.global_ts)
        return info, self._catalog()

    def _catalog(self) -> Catalog:
        profiles, color, stereo_sets, available = {}, {}, [], []
        for name, (sensor, _) in self.streams.items():
            try:
                plist = self.pipe.get_stream_profile_list(sensor)
            except Exception:
                continue    # the device has no such sensor
            available.append(name)
            d = plist.get_default_video_stream_profile()
            dkey = (d.get_width(), d.get_height(), d.get_fps(), d.get_format().name)
            modes = {}
            for i in range(plist.get_count()):
                p = plist.get_stream_profile_by_index(i).as_video_stream_profile()
                k = (p.get_width(), p.get_height(), p.get_fps(), p.get_format().name)
                profiles[(name, k)] = p
                if name == "color" and k[3] in COLOR_FORMATS:
                    color[f"{k[0]}x{k[1]}@{k[2]} {k[3]}"] = (k, k == dkey)
                elif name in STEREO and k[3] in STEREO_FORMATS[name]:
                    key = f"{k[0]}x{k[1]}@{k[2]}"
                    modes[key] = (k[:3], modes.get(key, (None, False))[1] or k == dkey)
            if name in STEREO:
                stereo_sets.append(modes)
        stereo = {}
        if stereo_sets:
            common = set.intersection(*(set(m) for m in stereo_sets))
            stereo = {k: (stereo_sets[0][k][0], any(m[k][1] for m in stereo_sets)) for k in common}

        def order(item):
            (w, h, fps, *fmt), _ = item[1]
            return (-w * h, -fps, COLOR_FORMATS.index(fmt[0]) if fmt else 0)

        self._profiles = profiles
        self._color = {k: v[0] for k, v in color.items()}
        self._stereo = {k: v[0] for k, v in stereo.items()}
        cat = Catalog(
            available=available,
            color=[ProfileOption(key=k, width=v[0][0], height=v[0][1], fps=v[0][2], format=v[0][3], default=v[1])
                   for k, v in sorted(color.items(), key=order)],
            stereo=[ProfileOption(key=k, width=v[0][0], height=v[0][1], fps=v[0][2], default=v[1])
                    for k, v in sorted(stereo.items(), key=order)],
            imu=self._imu_catalog())
        return cat

    def _imu_catalog(self) -> ImuCatalog:
        ob = self.ob
        found = {}
        for kind, sensor in (("accel", ob.OBSensorType.ACCEL_SENSOR), ("gyro", ob.OBSensorType.GYRO_SENSOR)):
            try:
                plist = self.pipe.get_stream_profile_list(sensor)
            except Exception:
                return ImuCatalog()
            items = {}
            for i in range(plist.get_count()):
                p = plist.get_stream_profile_by_index(i)
                q = p.as_accel_stream_profile() if kind == "accel" else p.as_gyro_stream_profile()
                items[(rate_name(q.get_sample_rate()), range_name(q.get_full_scale_range()))] = p
            found[kind] = items
        self._imu_profiles = found
        rates_a = {r for r, _ in found["accel"]}
        rates = [r for r, _ in found["gyro"] if r in rates_a]
        rates = sorted(dict.fromkeys(rates), key=rate_hz)
        return ImuCatalog(available=bool(rates), sample_rates=rates,
                          default_rate=next((r for r in ("100Hz", "200Hz") if r in rates), rates[0] if rates else None),
                          accel_ranges=sorted({g for _, g in found["accel"]}),
                          gyro_ranges=sorted({g for _, g in found["gyro"]}))

    def _profile(self, name, s: Settings):
        if name == "color":
            return self._profiles[("color", self._color[s.color_profile])]
        w, h, fps = self._stereo[s.stereo_mode]
        for fmt in STEREO_FORMATS[name]:
            p = self._profiles.get((name, (w, h, fps, fmt)))
            if p is not None:
                return p
        raise ValueError(f"{name} has no {w}x{h}@{fps} profile")

    # -- video -------------------------------------------------------------------------------

    def start(self, s: Settings):
        ob = self.ob
        self.stop()
        names = [n for n in self.streams if s.streams.get(n)]
        self._active = s
        if not names:
            return
        sync = s.frame_sync or (s.align and s.streams.get("depth") and s.streams.get("color"))
        cfg = ob.Config()
        for n in names:
            cfg.enable_stream(self._profile(n, s))
        try:
            cfg.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE if sync
                                                else ob.OBFrameAggregateOutputMode.DISABLE)
        except Exception as e:
            log.debug("aggregate mode: %s", e)
        try:
            (self.pipe.enable_frame_sync if sync else self.pipe.disable_frame_sync)()
        except Exception as e:
            log.debug("frame sync: %s", e)
        self.pipe.start(cfg)
        self.running = True
        self.align = (ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
                      if s.align and s.streams.get("depth") and s.streams.get("color") else None)

    def stop(self):
        if self.running:
            self.running = False
            try:
                self.pipe.stop()
            except Exception as e:
                log.debug("pipeline stop: %s", e)

    def wait(self, timeout_ms: int) -> Optional[list[Frame]]:
        fs = self.pipe.wait_for_frames(timeout_ms)
        if fs is None:
            return None
        t_owner = time.monotonic()
        off = epoch_minus_mono()
        s = self._active
        out = []
        for name, (_, ftype) in self.streams.items():
            if not s.streams.get(name):
                continue
            f = fs.get_frame(ftype)
            if f is None:
                continue
            t_sys, t_glob, t_dev, idx = (f.get_system_timestamp_us(), f.get_global_timestamp_us() if self.global_ts
                                         else 0, f.get_timestamp_us(), f.get_index())
            aligned = False
            if name == "depth" and self.align is not None:
                if fs.get_color_frame() is None:
                    continue
                res = self.align.process(fs)     # None until it has a depth+color pair
                f = res.as_frame_set().get_depth_frame() if res is not None else None
                if f is None:
                    continue
                aligned = True
            fr = self._to_frame(name, f, aligned)
            if fr is None:
                continue
            fr.t_capture = t_sys / 1e6 - off
            fr.t_sensor = (t_glob / 1e6 - off) if t_glob else None
            fr.t_device_us, fr.index, fr.t_owner = int(t_dev), int(idx), t_owner
            out.append(fr)
        return out

    def _to_frame(self, name, f, aligned) -> Optional[Frame]:
        vf = f.as_video_frame()
        w, h, fmt = vf.get_width(), vf.get_height(), vf.get_format().name
        data = np.frombuffer(vf.get_data(), np.uint8)
        if fmt == "MJPG":
            if data.size < 4 or data[0] != 0xFF or data[1] != 0xD8:
                return None
            return Frame(name, fmt, w, h, data.tobytes(), 0.0, None, 0, 0)
        bpp = BYTES_PER_PIXEL.get(fmt)
        if bpp is None or data.size != w * h * bpp:
            log.debug("%s: dropped %s frame of %d bytes (%dx%d)", name, fmt, data.size, w, h)
            return None
        if name == "depth":
            scale = float(f.as_depth_frame().get_depth_scale())
            return Frame(name, fmt, w, h, data.view(np.uint16).reshape(h, w).copy(), 0.0, None, 0, 0,
                         depth_scale=scale, aligned=aligned)
        return Frame(name, fmt, w, h, data.copy(), 0.0, None, 0, 0)

    def close(self):
        self.imu_stop()
        self.stop()
        self.pipe = self.dev = self.align = None
        self._active = None

    # -- properties --------------------------------------------------------------------------

    def properties(self, all_props: bool = False) -> list[dict]:
        ob, dev = self.ob, self.dev
        out = []
        items = list(CONTROLS)
        if all_props:
            known = {c[1] for c in CONTROLS}
            for i in range(_try(dev.get_support_property_count, 0)):
                item = _try(lambda: dev.get_supported_property(i), None)
                name = getattr(getattr(item, "id", None), "name", None) or getattr(item, "name", None)
                if name and name not in known and name.startswith("OB_PROP_"):
                    items.append(("other", name, name[8:].lower()))
        for group, pid, label in items:
            prop = getattr(ob.OBPropertyID, pid, None)
            try:
                if prop is None or not dev.is_property_supported(prop, ob.OBPermissionType.PERMISSION_READ_WRITE):
                    continue
                if pid.endswith("_BOOL"):
                    out.append({"group": group, "id": pid, "label": label, "type": "bool",
                                "value": bool(dev.get_bool_property(prop))})
                elif pid.endswith("_FLOAT"):
                    r = dev.get_float_property_range(prop)
                    out.append({"group": group, "id": pid, "label": label, "type": "float",
                                "value": float(dev.get_float_property(prop)), "min": r.min, "max": r.max,
                                "step": r.step, "default": r.default_value})
                elif pid.endswith("_INT"):
                    r = dev.get_int_property_range(prop)
                    out.append({"group": group, "id": pid, "label": label, "type": "int",
                                "value": int(dev.get_int_property(prop)), "min": r.min, "max": r.max,
                                "step": r.step, "default": r.default_value})
            except Exception as e:
                out.append({"group": group, "id": pid, "label": label, "type": "error", "error": str(e)})
        return out

    def set_property(self, pid: str, value):
        ob = self.ob
        prop = getattr(ob.OBPropertyID, pid, None) if pid.startswith("OB_PROP_") else None
        if prop is None:
            raise ValueError(f"unknown property {pid!r}")
        if not self.dev.is_property_supported(prop, ob.OBPermissionType.PERMISSION_READ_WRITE):
            raise ValueError(f"{pid} is not writable on this device")
        if pid.endswith("_BOOL"):
            self.dev.set_bool_property(prop, bool(value))
        elif pid.endswith("_FLOAT"):
            self.dev.set_float_property(prop, float(value))
        else:
            if isinstance(value, float) and not value.is_integer():
                raise ValueError(f"{pid} takes an integer")
            r = self.dev.get_int_property_range(prop)
            if not r.min <= int(value) <= r.max:
                raise ValueError(f"{pid} must be in [{r.min}, {r.max}]")
            self.dev.set_int_property(prop, int(value))

    # -- IMU (own pipeline, own thread) --------------------------------------------------------

    @property
    def imu_running(self):
        return self._imu_thread is not None and self._imu_thread.is_alive()

    def imu_start(self, s: ImuSettings, sink: Callable, on_error: Callable, batch_ms: Callable[[], float]):
        self.imu_stop()
        ob = self.ob
        pa = self._imu_profiles["accel"].get((s.sample_rate, s.accel_range))
        pg = self._imu_profiles["gyro"].get((s.sample_rate, s.gyro_range))
        if pa is None or pg is None:
            raise ValueError(f"no IMU profile {s.sample_rate} {s.accel_range}/{s.gyro_range}")
        pipe = ob.Pipeline(self.dev)
        cfg = ob.Config()
        cfg.enable_stream(pa)
        cfg.enable_stream(pg)
        try:
            cfg.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.ANY_SITUATION)
        except Exception:
            pass
        pipe.start(cfg)
        self._imu_stop = stop = threading.Event()
        self._imu_thread = threading.Thread(target=self._imu_loop, args=(pipe, stop, sink, on_error, batch_ms,
                                                                        1000.0 / rate_hz(s.sample_rate)),
                                            name="orbbec-imu", daemon=True)
        self._imu_thread.start()

    def imu_stop(self):
        t = self._imu_thread
        if t is not None:
            self._imu_stop.set()
            t.join(timeout=3)
            self._imu_thread = None

    def _imu_loop(self, pipe, stop, sink, on_error, batch_ms, period_ms):
        ob = self.ob
        A, G = ob.OBFrameType.ACCEL_FRAME, ob.OBFrameType.GYRO_FRAME
        nan = float("nan")
        rows, t_first, last_idx, gaps = [], 0.0, None, 0
        idle_since = time.monotonic()
        try:
            while not stop.is_set():
                fs = pipe.wait_for_frames(50)
                now = time.monotonic()
                if fs is not None:
                    a, g = fs.get_frame(A), fs.get_frame(G)
                    ref = a if a is not None else g
                    if ref is not None:
                        idle_since = now
                        off = epoch_minus_mono()
                        t_sys = ref.get_system_timestamp_us()
                        t_glob = ref.get_global_timestamp_us() if self.global_ts else 0
                        idx = ref.get_index()
                        if last_idx is not None and idx > last_idx + 1:
                            gaps += idx - last_idx - 1
                        last_idx = idx
                        av = a.as_accel_frame() if a is not None else None
                        gv = g.as_gyro_frame() if g is not None else None
                        rows.append((0.0, t_sys / 1e6 - off, (t_glob / 1e6 - off) if t_glob else nan,
                                     float(ref.get_timestamp_us()),
                                     av.get_x() if av else nan, av.get_y() if av else nan, av.get_z() if av else nan,
                                     gv.get_x() if gv else nan, gv.get_y() if gv else nan, gv.get_z() if gv else nan,
                                     (av or gv).get_temperature()))
                        if len(rows) == 1:
                            t_first = now
                elif now - idle_since > 4.0:
                    raise RuntimeError("no IMU samples for 4 s")
                # flush when waiting for the next sample would overshoot the batch window
                if rows and (now - t_first) * 1000 + period_ms >= batch_ms():
                    sink(np.array(rows, dtype=np.float64).reshape(-1, len(IMU_FIELDS)), gaps)
                    rows, gaps = [], 0
        except Exception as e:
            if not stop.is_set():
                on_error(str(e))
        finally:
            try:
                pipe.stop()
            except Exception:
                pass
