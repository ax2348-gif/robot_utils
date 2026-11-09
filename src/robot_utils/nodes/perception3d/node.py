"""3d_perception node core: reads the camera, IMU, pose and description, registers, publishes.

Threads (no web code here; server.py serves it):

- one **reader per upstream stream** (``StreamReader``, modelled on the estimator's ``Upstream``
  but for any record type): camera frames (``/stream?streams=color,depth&mode=raw``), the camera's
  IMU filter records (``/imu/stream``) and the estimator's poses (``/stream``). Each syncs its
  clock to the upstream (``GET /time``, every 30 s; ~0 on the same Pi), converts timestamps to
  this machine's monotonic clock, puts items on one queue, reconnects every second after errors.
- a **config poller**: the camera's ``/calibration`` (intrinsics, depth->colour and IMU->depth
  extrinsics; re-read every 5 s and when the frame sizes change) and the description's
  ``/description`` (the depth camera's pose in the base and which calibrations it came from).
- the **worker**: the only code touching the motion tracks and the ``Registrar``. It keeps the
  newest depth frame; every colour frame waits (at most ``wait_s``) until the IMU / pose tracks
  cover its capture time, is paired with the newest depth frame, motion-compensated, registered
  and published (latest-frame-wins ``FrameSlot``). A colour frame replaced before it was processed
  is counted as skipped.

When nobody has asked for frames for ``idle_s`` (no stream client, no /frame or /point), the
worker stops registering (state ``idle``) and the next request wakes it.
"""
from __future__ import annotations

import asyncio
import logging
import queue
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..common.client import NodeClient
from ..common.protocol import RowsDecoder
from ..common.server import EventLog
from .motion import Compensator
from .register import Intrinsics, Registrar, inv, query_point, rigid

log = logging.getLogger("perception3d")

SYNC_EVERY_S = 30.0
RECONNECT_S = 1.0
CONFIG_EVERY_S = 5.0
LIVE_S = 0.5                    # an input is live if it delivered within this
IMU_FLAG_INITIALIZING = 4


# -- upstream readers -----------------------------------------------------------------------------

class Skip(Exception):
    """A colour frame not published (start-up transient), silently."""


class StreamReader(threading.Thread):
    """Reads one upstream binary stream; ``handle(header, payload)`` returns an item (or None)
    for the worker queue. Keeps its own status and clock offset (upstream - local)."""

    def __init__(self, key: str, target: str, path: str, out: queue.Queue, events: EventLog,
                 params: Optional[dict] = None):
        super().__init__(name=f"in-{key}", daemon=True)
        self.key, self.target, self.path, self.out, self.events = key, target, path, out, events
        self.params = params or {}
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.offset = 0.0
        self.uncertainty: Optional[float] = None
        self.connected = False
        self.error: Optional[str] = None
        self.count = 0
        self.last_rx: Optional[float] = None
        self._times: deque = deque(maxlen=200)
        self._client: Optional[NodeClient] = None
        self.paused_fn = None           # callable: True = stay disconnected (checked per record)
        self.paused = False
        self.connected_at = 0.0
        self.wake = threading.Event()   # set to re-check paused_fn at once

    def handle(self, h: dict, p: bytes):
        raise NotImplementedError

    def live(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return self.connected and self.last_rx is not None and now - self.last_rx < LIVE_S

    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            ts = list(self._times)
            rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 5 and ts[-1] > ts[0] else None
            return {"target": self.target, "path": self.path, "connected": self.connected, "error": self.error,
                    "paused": self.paused,
                    "rate_hz": None if rate is None or now - ts[-1] > 1.0 else round(rate, 1),
                    "last_age": None if self.last_rx is None else round(now - self.last_rx, 3),
                    "count": self.count, "clock_offset": round(self.offset, 6),
                    "clock_uncertainty": None if self.uncertainty is None else round(self.uncertainty, 6)}

    def _sync(self):
        est = self._client.sync()
        with self.lock:
            self.offset, self.uncertainty = est.offset, est.uncertainty

    def _received(self, n: int = 1):
        now = time.monotonic()
        with self.lock:
            self.count += n
            self.last_rx = now
            self._times.append(now)

    def run(self):
        reported = None
        while not self.stop.is_set():
            if self.paused_fn is not None and self.paused_fn():      # idle: stay disconnected
                with self.lock:
                    self.paused = True
                self.wake.wait(0.5)
                self.wake.clear()
                continue
            with self.lock:
                self.paused = False
            try:
                self._client = NodeClient.for_instance(self.target)
                self._sync()
                next_sync = time.monotonic() + SYNC_EVERY_S
                with self.lock:
                    self.connected, self.error = True, None
                    self.connected_at = time.monotonic()
                self.events.add(f"{self.key}: connected to {self.target}{self.path} "
                                f"(clock offset {self.offset * 1e3:+.2f} ± {self.uncertainty * 1e3:.2f} ms)")
                reported = None
                for h, p, t_rx in self._client.records(self.path, **self.params):
                    if self.stop.is_set():
                        return
                    item = self.handle(h, p)
                    if item is not None:
                        self.out.put((self.key, item, self.offset, t_rx))
                    if time.monotonic() > next_sync:
                        next_sync = time.monotonic() + SYNC_EVERY_S
                        self._sync()
                    if self.paused_fn is not None and self.paused_fn():
                        break
                else:
                    raise ConnectionError("stream ended")
                self._client.close()
                with self.lock:
                    self.connected = False
                self.events.add(f"{self.key}: disconnected while idle (no consumers)")
            except Exception as e:   # unreachable, stream ended, bad data: report, retry
                msg = f"{type(e).__name__}: {e}"
                with self.lock:
                    self.connected, self.error = False, msg
                if msg != reported:
                    self.events.add(f"{self.key}: {msg} (retrying every {RECONNECT_S:.0f} s)", "warn")
                    reported = msg
                if self._client is not None:
                    self._client.close()
                self.stop.wait(RECONNECT_S)


class FrameReader(StreamReader):
    def handle(self, h, p):
        if h.get("type") != "frame" or h.get("stream") not in ("color", "depth"):
            return None
        self._received()
        return h, p


class RowsReader(StreamReader):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.dec = RowsDecoder()

    def handle(self, h, p):
        rows = self.dec.decode(h, p)
        if rows is None or not len(rows):
            return None
        self._received(len(rows))
        return rows


# -- latest fused frame -> event loop -------------------------------------------------------------

@dataclass
class Fused:
    header: dict
    color: bytes                    # the camera's colour payload, untouched (MJPG)
    depth: np.ndarray               # (h, w) uint16 mm, z in the colour camera frame at t_color
    out: Intrinsics                 # the grid of ``depth``
    color_k: Intrinsics             # full colour intrinsics
    T_cd_base: Optional[np.ndarray] = None   # colour camera in base (for point_base), or None
    t_published: float = 0.0


class FrameSlot:
    """Latest-frame-wins handoff from the worker to async consumers (no polling)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest: Optional[Fused] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._fut: Optional[asyncio.Future] = None

    def attach(self, loop):
        self.loop = loop

    def publish(self, f: Fused):
        with self.lock:
            self.latest = f
        loop = self.loop
        if loop is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(self._wake)
            except RuntimeError:
                pass

    def get(self) -> Optional[Fused]:
        with self.lock:
            return self.latest

    def changed(self) -> asyncio.Future:
        """Future resolved at the next publish. Take it *before* reading ``get()``."""
        if self._fut is None or self._fut.done():
            self._fut = asyncio.get_running_loop().create_future()
        return self._fut

    def _wake(self):
        f, self._fut = self._fut, None
        if f is not None and not f.done():
            f.set_result(None)


# -- the node -------------------------------------------------------------------------------------

@dataclass
class Params:
    scale: float = 0.5               # output grid = colour resolution x scale
    stride: int = 1                  # depth pixel subsampling
    splat: int = 1                   # 1 or 2 (2x2 blocks: for outputs denser than the depth)
    compensation: str = "auto"       # auto (best available) | rotation (IMU only) | none
    wait_s: float = 0.06             # max wait for IMU/pose coverage of a colour frame
    max_depth_age_s: float = 0.5     # older depth is not used (frame published without depth)
    idle_s: float = 30.0             # stop registering after this long without consumers (0 = never)
    allow_nominal: bool = False      # use the description's nominal (uncalibrated) camera pose

    def to_json(self):
        return dict(self.__dict__)


class PerceptionNode:
    def __init__(self, camera: Optional[str] = "camera", estimator: Optional[str] = "estimator",
                 description: Optional[str] = "description", params: Optional[Params] = None):
        self.params = params or Params()
        self.camera_target, self.estimator_target, self.description_target = camera, estimator, description
        self.events = EventLog(on_add=self._state_changed)
        self.slot = FrameSlot()
        self.q: queue.Queue = queue.Queue(maxsize=5000)
        self.comp = Compensator()
        self.inputs: dict[str, StreamReader] = {}
        if camera:
            self.inputs["frames"] = FrameReader("frames", camera, "/stream", self.q, self.events,
                                                {"streams": "color,depth", "mode": "raw"})
            self.inputs["imu"] = RowsReader("imu", camera, "/imu/stream", self.q, self.events)
        if estimator:
            self.inputs["pose"] = RowsReader("pose", estimator, "/stream", self.q, self.events)
        for r in self.inputs.values():    # no consumers: disconnect (raw depth is 8 MB/s; 100 Hz rows ~10 % CPU)
            r.paused_fn = self.idle
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._state_fut: Optional[asyncio.Future] = None
        # config (poller -> worker), under lock
        self.cam_cal: Optional[dict] = None
        self.cam_cal_error: Optional[str] = None
        self.desc: Optional[dict] = None              # {T_bd, from, calibrated, serial}
        self.desc_error: Optional[str] = None
        self._cal_refresh = threading.Event()
        # worker state
        self._depth = None                            # (header, array, t_local)
        self._pending = None                          # (header, payload, t_c local, arrival)
        self._registrar: Optional[Registrar] = None
        self._reg_key = None
        self._pose_reset_id = None
        self.seq = 0
        self.skipped = 0
        self.error: Optional[str] = None
        self.clients = 0
        self._last_demand = time.monotonic()
        self._stats: deque = deque(maxlen=100)       # per published frame
        self._color_times: deque = deque(maxlen=50)
        self._threads = [threading.Thread(target=self._run, name="worker", daemon=True),
                         threading.Thread(target=self._poll_config, name="config", daemon=True)]

    # -- lifecycle --------------------------------------------------------------------------------

    def start(self):
        self.events.add(f"3d_perception started: camera {self.camera_target}, estimator {self.estimator_target}, "
                        f"description {self.description_target}")
        for r in self.inputs.values():
            r.start()
        for t in self._threads:
            t.start()

    def close(self):
        self.stop.set()
        for r in self.inputs.values():
            r.stop.set()

    def attach(self, loop):
        self.loop = loop
        self.slot.attach(loop)

    # -- event-loop helpers -----------------------------------------------------------------------

    def _state_changed(self):
        loop = self.loop
        if loop is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(self._wake_state)
            except RuntimeError:
                pass

    def _wake_state(self):
        f, self._state_fut = self._state_fut, None
        if f is not None and not f.done():
            f.set_result(None)

    def state_future(self) -> asyncio.Future:
        if self._state_fut is None or self._state_fut.done():
            self._state_fut = asyncio.get_running_loop().create_future()
        return self._state_fut

    def demand(self):
        """A consumer wants frames: keep (or start) registering."""
        self._last_demand = time.monotonic()
        for r in self.inputs.values():
            r.wake.set()

    def idle(self, now: Optional[float] = None) -> bool:
        if self.params.idle_s <= 0 or self.clients > 0:
            return False
        return (time.monotonic() if now is None else now) - self._last_demand > self.params.idle_s

    def set_params(self, **kw) -> Params:
        with self.lock:
            for k, v in kw.items():
                if v is not None:
                    setattr(self.params, k, v)
            self._reg_key = None          # rebuild the registrar on the next frame
        self.events.add("settings: " + ", ".join(f"{k}={v}" for k, v in kw.items() if v is not None))
        return self.params

    # -- config poller ----------------------------------------------------------------------------

    def _poll_config(self):
        cam = desc = None
        while not self.stop.is_set():
            try:
                if self.camera_target:
                    cam = cam or NodeClient.for_instance(self.camera_target)
                    cal = cam.get("/calibration")
                    with self.lock:
                        changed = cal != self.cam_cal
                        self.cam_cal, self.cam_cal_error = cal, None
                    if changed:
                        self.events.add(f"camera calibration: depth {self._size(cal, 'depth')}, colour "
                                        f"{self._size(cal, 'color')}, settings_version {cal.get('settings_version')}")
            except Exception as e:
                with self.lock:
                    self.cam_cal_error = f"{type(e).__name__}: {e}"
                if cam is not None:
                    cam.close()
                cam = None
            try:
                if self.description_target:
                    desc = desc or NodeClient.for_instance(self.description_target)
                    self._apply_description(desc.get("/description"))
            except Exception as e:
                with self.lock:
                    self.desc, self.desc_error = None, f"{type(e).__name__}: {e}"
                if desc is not None:
                    desc.close()
                desc = None
            self._cal_refresh.wait(CONFIG_EVERY_S)
            self._cal_refresh.clear()
        for c in (cam, desc):
            if c is not None:
                c.close()

    @staticmethod
    def _size(cal, k):
        c = (cal or {}).get(k)
        return f"{c['width']}x{c['height']}" if c else "none"

    def _apply_description(self, d: dict):
        cams = d.get("cameras") or {}
        entry = next((c for c in cams.values() if c.get("node") == self.camera_target), None)
        if entry is None and len(cams) == 1:
            entry = next(iter(cams.values()))
        if entry is None or not entry.get("pose_in_base"):
            with self.lock:
                self.desc, self.desc_error = None, f"no camera for node {self.camera_target!r} in the description"
            return
        src = entry.get("from") or {}
        calibrated = bool(src.get("ground") or src.get("yaw"))
        new = {"T_bd": np.asarray(entry["pose_in_base"]["matrix"], float), "from": src,
               "calibrated": calibrated, "serial": entry.get("serial")}
        with self.lock:
            old = self.desc
            self.desc, self.desc_error = new, None
        if old is None or old["from"] != src or not np.allclose(old["T_bd"], new["T_bd"]):
            what = ", ".join(f"{k} {v}" for k, v in src.items() if k in ("ground", "yaw") and v) or "nominal only"
            self.events.add(f"description: camera pose in base from {what}"
                            + ("" if calibrated or self.params.allow_nominal else
                               " (not calibrated: translation compensation off)"))

    # -- worker -----------------------------------------------------------------------------------

    def _run(self):
        while not self.stop.is_set():
            try:
                item = self.q.get(timeout=0.01)
            except queue.Empty:
                item = None
            while item is not None:
                try:
                    self._ingest(*item)
                except Exception as e:   # bad data must not kill the worker
                    log.exception("ingest")
                    self.error = f"{type(e).__name__}: {e}"
                try:
                    item = self.q.get_nowait()
                except queue.Empty:
                    item = None
            if self._pending is not None and self._ready(self._pending):
                pending, self._pending = self._pending, None
                if self.idle():
                    continue
                try:
                    self._process(*pending)
                    if self.error:
                        self.error = None
                        self._state_changed()
                except Skip:
                    pass
                except Exception as e:
                    if not isinstance(e, RuntimeError):    # RuntimeError: an expected "not ready" condition
                        log.exception("process")
                    msg = str(e) if isinstance(e, RuntimeError) else f"{type(e).__name__}: {e}"
                    if msg != self.error:
                        self.events.add(f"frame failed: {msg}", "error")
                    self.error = msg

    def _ingest(self, key, item, offset, t_rx):
        if key == "imu":
            names = item.dtype.names
            init = item["flags"].astype(np.int64) & IMU_FLAG_INITIALIZING if "flags" in names else np.zeros(len(item))
            for r, bad in zip(item, init):
                if not bad:
                    self.comp.imu.add(float(r["t_end"]) - offset, (float(r["qw"]), float(r["qx"]),
                                                                    float(r["qy"]), float(r["qz"])))
        elif key == "pose":
            names = item.dtype.names
            for r in item:
                if "reset_id" in names:
                    rid = float(r["reset_id"])
                    if self._pose_reset_id is not None and rid != self._pose_reset_id:
                        self.comp.pose.clear()          # the pose jumped: don't interpolate across it
                    self._pose_reset_id = rid
                self.comp.pose.add(float(r["t"]) - offset, float(r["x"]), float(r["y"]), float(r["theta"]))
        elif key == "frames":
            h, p = item
            t = h.get("t_sensor") or h.get("t_capture")
            t_local = None if t is None else float(t) - offset
            if h["stream"] == "depth":
                if h.get("compression"):
                    import zlib
                    p = zlib.decompress(p)
                arr = np.frombuffer(p, h.get("dtype") or "<u2").reshape(h["height"], h["width"])
                self._depth = (h, arr, t_local)
            else:
                now = time.monotonic()
                with self.lock:
                    self._color_times.append(now)
                if self._pending is not None:
                    self.skipped += 1
                self._pending = (h, p, t_local, now)

    def _sources(self):
        """(use_imu, use_pose) given the inputs and the compensation setting."""
        mode = self.params.compensation
        imu = mode in ("auto", "rotation") and "imu" in self.inputs and self.inputs["imu"].connected
        with self.lock:
            d = self.desc
        pose_ok = (d is not None and (d["calibrated"] or self.params.allow_nominal))
        pose = mode == "auto" and "pose" in self.inputs and self.inputs["pose"].connected and pose_ok
        return imu, pose

    def _ready(self, pending) -> bool:
        h, p, t_c, arrival = pending
        if t_c is None or time.monotonic() - arrival >= self.params.wait_s:
            return True
        imu, pose = self._sources()
        if imu and self.inputs["imu"].live() and not self.comp.imu.covers(t_c):
            return False
        if pose and self.inputs["pose"].live() and not self.comp.pose.covers(t_c):
            return False
        return True

    def _calibration(self, hc, hd):
        """(Registrar, extras) for the current calibration, frame sizes and params (cached)."""
        with self.lock:
            cal, params = self.cam_cal, self.params
        if cal is None or "color" not in cal or "depth" not in cal or "depth_to_color" not in cal:
            raise RuntimeError("no camera calibration yet (GET /calibration of the camera node)")
        if cal.get("aligned_depth"):
            raise RuntimeError("the camera aligns depth to colour itself (align: true); turn it off")
        sizes = ((hc["width"], hc["height"]), (hd["width"], hd["height"]))
        cs = ((cal["color"]["width"], cal["color"]["height"]), (cal["depth"]["width"], cal["depth"]["height"]))
        if sizes != cs:
            self._cal_refresh.set()
            raise RuntimeError(f"frames {sizes} do not match the calibration {cs} (profile changed?); re-reading it")
        key = (cal.get("settings_version"), sizes, params.scale, params.stride, params.splat,
               str(cal["depth_to_color"]), str(cal["color"]), str(cal["depth"]))
        if key != self._reg_key:
            e = cal["depth_to_color"]
            E = rigid(e["rotation"], np.asarray(e["translation_mm"], float) / 1000.0)
            self._registrar = Registrar(Intrinsics.from_json(cal["depth"]), Intrinsics.from_json(cal["color"]), E,
                                        scale=params.scale, stride=params.stride, splat=params.splat)
            a2d = ((cal.get("imu") or {}).get("accel_to_depth") or {}).get("rotation")
            self.comp.set_imu_to_depth(a2d)
            self._reg_key = key
            o = self._registrar.out
            self.events.add(f"registrar: output {o.width}x{o.height} (scale {params.scale}), depth stride "
                            f"{params.stride}, splat {params.splat}")
        return self._registrar

    def _process(self, hc, color, t_c, arrival):
        t_start = time.monotonic()
        t0, c0 = time.perf_counter(), time.thread_time()
        if self._depth is None:
            if time.monotonic() - self.inputs["frames"].connected_at < 1.0:
                raise Skip()
            raise RuntimeError("no depth frame yet (is the camera's depth stream on?)")
        hd, depth, t_d = self._depth
        reg = self._calibration(hc, hd)
        use_imu, use_pose = self._sources()
        with self.lock:
            d = self.desc
        self.comp.use_imu, self.comp.use_pose = use_imu, use_pose
        self.comp.set_base_to_depth(d["T_bd"] if d is not None and (d["calibrated"] or self.params.allow_nominal) else None)
        dt = None if (t_c is None or t_d is None) else t_c - t_d
        have_depth = dt is not None and abs(dt) <= self.params.max_depth_age_s
        if not have_depth and time.monotonic() - self.inputs["frames"].connected_at < 1.0:
            raise Skip()        # just (re)connected: the first depth frame is on its way
        if have_depth:
            # just woken up: wait until the motion tracks reach back to the depth frame's time
            for key, track, used in (("imu", self.comp.imu, use_imu), ("pose", self.comp.pose, use_pose)):
                r = self.inputs.get(key)
                if (used and r is not None and time.monotonic() - r.connected_at < 1.0
                        and (track.start is None or track.start > t_d)):
                    raise Skip()
            M, mode = self.comp.relative(t_d, t_c)
            img = reg.register(depth, float(hd.get("depth_scale") or 1.0), M)
        else:
            M, mode = np.eye(4), "no_depth"
            img = np.zeros((reg.out.height, reg.out.width), np.uint16)
        t_done = time.monotonic()
        compute_ms = (time.perf_counter() - t0) * 1e3
        cpu_ms = (time.thread_time() - c0) * 1e3
        self.seq += 1
        valid = float(np.count_nonzero(img)) / img.size
        o = reg.out
        cbytes = bytes(color)
        dbytes = img.astype("<u2", copy=False).tobytes()
        T_cd_base = None
        if d is not None and (d["calibrated"] or self.params.allow_nominal):
            T_cd_base = d["T_bd"] @ inv(reg.E)        # colour camera in base
        header = {
            "type": "fused", "seq": self.seq,
            "t_color": t_c, "t_depth": t_d if have_depth else None,
            "dt_color_depth": None if dt is None else round(dt, 6),
            "mode": mode, "motion": np.round(M, 9).tolist(), **self.comp.summary(M),
            "color_seq": hc.get("seq"), "depth_seq": hd.get("seq"),
            "color": {"format": hc.get("format"), "width": hc["width"], "height": hc["height"],
                      "offset": 0, "len": len(cbytes)},
            "depth": {"width": o.width, "height": o.height, "dtype": "<u2", "units": "mm",
                      "frame": "color_optical", "offset": len(cbytes), "len": len(dbytes), "valid": round(valid, 4)},
            "scale": reg.scale, "stride": reg.stride,
            "t_color_capture": None if hc.get("t_capture") is None else float(hc["t_capture"]) - self.inputs["frames"].offset,
            "t_received": arrival, "t_published": t_done,
            "wait_ms": round((t_start - arrival) * 1e3, 3),
            "compute_ms": round(compute_ms, 3), "cpu_ms": round(cpu_ms, 3),
            "latency_ms": None if t_c is None else round((t_done - t_c) * 1e3, 3),
        }
        f = Fused(header, cbytes, img, o, reg.color, T_cd_base, t_done)
        with self.lock:
            self._stats.append({"mode": mode, "compute_ms": compute_ms, "cpu_ms": cpu_ms,
                                "latency_ms": header["latency_ms"], "dt_ms": None if dt is None else dt * 1e3,
                                "wait_ms": header["wait_ms"], "t": t_done, "valid": valid})
        self.slot.publish(f)

    # -- queries ----------------------------------------------------------------------------------

    @staticmethod
    def point(f: Fused, u: float, v: float, radius: int = 2) -> dict:
        res = query_point(f.depth, f.out, f.color_k, u, v, radius)
        res.update({"seq": f.header["seq"], "t_color": f.header["t_color"], "mode": f.header["mode"],
                    "frame": "color_optical", "age_s": round(time.monotonic() - f.t_published, 4)})
        res["point_base"] = None
        if res["point"] is not None and f.T_cd_base is not None:
            p = f.T_cd_base @ np.array([*res["point"], 1.0])
            res["point_base"] = [float(x) for x in p[:3]]
        return res

    def calibration(self) -> dict:
        reg = self._registrar
        with self.lock:
            d = self.desc
        if reg is None:
            return {"ready": False}
        out = {"ready": True, "color": reg.color.to_json(), "depth_grid": reg.out.to_json(), "scale": reg.scale,
               "stride": reg.stride, "splat": reg.splat, "depth": reg.depth.to_json(),
               "depth_to_color": np.round(reg.E, 9).tolist(), "imu_to_depth": np.round(self.comp.R_di, 9).tolist(),
               "frames": "colour optical: x right, y down, z out of the lens; m",
               "base_to_color": None, "base_calibration": None}
        if d is not None:
            out["base_to_color"] = np.round(d["T_bd"] @ inv(reg.E), 9).tolist()
            out["base_calibration"] = {"from": d["from"], "calibrated": d["calibrated"]}
        return out

    # -- state ------------------------------------------------------------------------------------

    def state(self) -> dict:
        now = time.monotonic()
        with self.lock:
            stats = list(self._stats)
            ct = list(self._color_times)
            cam_err, desc_err, d = self.cam_cal_error, self.desc_error, self.desc
        recent = [s for s in stats if now - s["t"] < 5.0]
        idle = self.idle(now)
        live = bool(recent) and now - recent[-1]["t"] < 1.0

        def pct(key, q):
            v = [s[key] for s in recent if s[key] is not None]
            return None if not v else round(float(np.percentile(v, q)), 2)
        rate = (len(recent) - 1) / (recent[-1]["t"] - recent[0]["t"]) if len(recent) > 2 and recent[-1]["t"] > recent[0]["t"] else None
        crate = (len(ct) - 1) / (ct[-1] - ct[0]) if len(ct) > 2 and ct[-1] > ct[0] and now - ct[-1] < 1 else None
        inputs = {k: r.status() for k, r in self.inputs.items()}
        errors = [f"{k}: {v['error']}" for k, v in inputs.items() if v["error"]]
        if idle:
            st = "idle"
        elif live:
            st = "running"
        else:
            st = "waiting"
        err = self.error if self.error else ("; ".join(errors) if errors and not live else None)
        f = self.slot.get()
        return {
            "state": st, "error": err,
            "mode": f.header["mode"] if f is not None and live else None,
            "modes": dict(Counter(s["mode"] for s in recent)),
            "rate_hz": None if rate is None else round(rate, 2),
            "color_rate_hz": None if crate is None else round(crate, 2),
            "seq": self.seq, "skipped": self.skipped, "clients": self.clients,
            "timing": {"compute_ms": {"median": pct("compute_ms", 50), "p95": pct("compute_ms", 95)},
                       "cpu_ms": {"median": pct("cpu_ms", 50), "p95": pct("cpu_ms", 95)},
                       "latency_ms": {"median": pct("latency_ms", 50), "p95": pct("latency_ms", 95)},
                       "wait_ms": {"median": pct("wait_ms", 50), "p95": pct("wait_ms", 95)},
                       "dt_color_depth_ms": {"median": pct("dt_ms", 50), "p95": pct("dt_ms", 95)},
                       "valid": pct("valid", 50)},
            "latest": None if f is None else {k: f.header[k] for k in ("seq", "t_color", "t_depth", "dt_color_depth",
                                                                     "mode", "rotation_deg", "translation_mm",
                                                                     "compute_ms", "latency_ms")},
            "params": self.params.to_json(),
            "sources": {"imu": {"track_end": self.comp.imu.end, "samples": len(self.comp.imu.t)},
                        "pose": {"track_end": self.comp.pose.end, "samples": len(self.comp.pose.t)},
                        "camera_calibration": {"ok": self.cam_cal is not None and cam_err is None, "error": cam_err},
                        "description": {"ok": d is not None, "error": desc_err,
                                        "calibrated": None if d is None else d["calibrated"],
                                        "from": None if d is None else d["from"]}},
            "inputs": inputs,
            "events": self.events.list(),
        }
