"""Python client for the Orbbec node: settings, clock sync, frame/IMU streams, recording.

    from robot_utils.nodes.orbbec.client import OrbbecClient

    cam = OrbbecClient("http://100.80.153.120:8000")
    est = cam.sync()                                 # NTP-style; est.uncertainty in s
    cam.update_settings({"streams": {"color": True, "depth": True}})
    for fr in cam.frames("color,depth", mode="raw"):
        img = fr.image                               # color: HxWx3 RGB; depth: HxW uint16
        print(fr.stream, fr.seq, fr.image.shape, "latency %.1f ms" % (fr.latency * 1e3))

Needs httpx and numpy (+ OpenCV or Pillow to decode JPEG). Every server timestamp is on the
server's time.monotonic(); ``t_recv`` is this machine's time.monotonic() when the record's last
byte arrived, ``t_recv_server`` the same instant mapped into the server clock.
"""
from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Iterator, Optional

import httpx
import numpy as np

from .protocol import RecordReader, decode_payload, iter_records, pack_record
from .timesync import ClockSync, SyncEstimate, collect_samples, estimate_offset


class OrbbecError(RuntimeError):
    def __init__(self, status: int, detail: str):
        super().__init__(f"HTTP {status}: {detail}")
        self.status, self.detail = status, detail


@dataclass
class Record:
    """One received record (frame, imu or ping)."""
    header: dict
    payload: bytes
    t_recv: float                      # client monotonic s (last byte received)
    t_recv_server: Optional[float]     # t_recv on the server clock (None before sync)
    decode_jpeg: bool = True
    _data: object = field(default=None, repr=False)

    @property
    def type(self) -> str:
        return self.header["type"]

    @property
    def stream(self) -> Optional[str]:
        return self.header.get("stream")

    @property
    def seq(self) -> Optional[int]:
        return self.header.get("seq")

    @property
    def data(self):
        """Decoded payload: image array (frames), structured sample array (imu), None (ping)."""
        if self._data is None and self.type != "ping":
            self._data = decode_payload(self.header, self.payload, self.decode_jpeg)
        return self._data

    image = data   # alias for frames
    samples = data  # alias for IMU batches

    @property
    def depth_mm(self) -> np.ndarray:
        """Raw depth frame in millimetres (float32); 0 = no depth."""
        if self.stream != "depth" or self.header.get("encoding") != "raw":
            raise ValueError("depth_mm needs a raw depth frame (mode=raw)")
        return self.data.astype(np.float32) * float(self.header.get("depth_scale", 1.0))

    @property
    def latency(self) -> Optional[float]:
        """Server capture (SDK receipt) -> fully received here, in s (needs sync)."""
        if self.t_recv_server is None or "t_capture" not in self.header:
            return None
        return self.t_recv_server - self.header["t_capture"]


class OrbbecClient:
    def __init__(self, base_url: str = "http://localhost:8000", timeout: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.http = httpx.Client(base_url=self.base_url, timeout=timeout)
        self.clock = ClockSync()
        self._clock_lock = threading.Lock()
        self._resync: Optional[threading.Thread] = None
        self._resync_stop = threading.Event()

    def close(self):
        self.stop_auto_sync()
        self.http.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- plain requests ----------------------------------------------------------------------

    def _check(self, r: httpx.Response):
        if r.status_code >= 400:
            try:
                detail = r.json().get("detail")
            except Exception:
                detail = r.text
            raise OrbbecError(r.status_code, detail if isinstance(detail, str) else json.dumps(detail))
        return r

    def get(self, path, **params):
        return self._check(self.http.get(path, params=params or None)).json()

    def post(self, path, body):
        return self._check(self.http.post(path, json=body)).json()

    def state(self) -> dict:
        return self.get("/state")

    def catalog(self) -> dict:
        return self.get("/catalog")

    def settings(self) -> dict:
        return self.get("/settings")

    def update_settings(self, patch: dict) -> dict:
        """POST /settings. Returns the SettingsResult; raises OrbbecError (400 invalid,
        409 refused by the device - previous settings restored, 503 no camera)."""
        return self.post("/settings", patch)

    def properties(self, all: bool = False) -> list:
        return self.get("/properties", all=str(all).lower())

    def set_property(self, pid: str, value) -> list:
        return self.post("/properties", {"id": pid, "value": value})

    # -- clock sync --------------------------------------------------------------------------

    def sync(self, n: int = 16, keep: int = 8, http: Optional[httpx.Client] = None) -> SyncEstimate:
        """NTP-style burst against GET /time; updates self.clock and returns the estimate."""
        http = http or self.http
        http.get("/time")   # warm up the connection (TCP/TLS setup is not a sample)
        est = estimate_offset(collect_samples(lambda: self._check(http.get("/time")).json()["mono"], n), keep)
        with self._clock_lock:
            self.clock.add(est)
        return est

    def to_server(self, t_client: float) -> float:
        with self._clock_lock:
            return self.clock.to_server(t_client)

    def start_auto_sync(self, interval: float = 30.0, n: int = 16):
        """Re-sync every ``interval`` s in a background thread (own connection); fits drift."""
        self.stop_auto_sync()
        self._resync_stop = stop = threading.Event()

        def loop():
            with httpx.Client(base_url=self.base_url, timeout=self.timeout) as http:
                while not stop.wait(interval):
                    try:
                        self.sync(n, http=http)
                    except Exception:
                        pass
        self._resync = threading.Thread(target=loop, name="orbbec-resync", daemon=True)
        self._resync.start()

    def stop_auto_sync(self):
        if self._resync is not None:
            self._resync_stop.set()
            self._resync.join(timeout=2)
            self._resync = None

    # -- streams -----------------------------------------------------------------------------

    def records(self, path: str, params: dict, decode_jpeg: bool = True, include_pings: bool = False,
                read_timeout: float = 10.0) -> Iterator[Record]:
        """Low-level: iterate the records of a streaming endpoint."""
        timeout = httpx.Timeout(self.timeout, read=read_timeout)
        with self.http.stream("GET", path, params=params, timeout=timeout) as r:
            if r.status_code >= 400:
                r.read()
                self._check(r)
            reader = RecordReader()
            for chunk in r.iter_raw():
                done = reader.feed(chunk)
                if not done:
                    continue
                t = time.monotonic()
                with self._clock_lock:
                    ts = self.clock.to_server(t) if self.clock.latest else None
                for h, p in done:
                    if h["type"] == "ping" and not include_pings:
                        continue
                    yield Record(h, p, t, ts, decode_jpeg)

    def frames(self, streams: str = "color,depth", mode: str = "jpeg", compress: Optional[str] = None,
               decode_jpeg: bool = True, **kw) -> Iterator[Record]:
        """Live frames (latest-wins). ``streams`` like ``"color,depth"`` or ``"color,depth:raw"``.
        With decode_jpeg=False, color (and jpeg-mode depth/IR) stay as JPEG bytes in ``.data``."""
        params = {"streams": streams, "mode": mode}
        if compress:
            params["compress"] = compress
        return self.records("/stream", params, decode_jpeg, **kw)

    def imu(self, raw: bool = False, batch_ms: float = 0.0, max_backlog: int = 20000, **kw) -> Iterator[Record]:
        """IMU records; ``rec.samples`` is a structured array named by ``rec.header["fields"]``.

        Default: the node's attitude-filter output (~100 Hz; imu_filter.STATE_FIELDS: orientation,
        delta angle/velocity, means, gyro bias, flags). ``raw=True``: every 1 kHz sample
        (protocol.IMU_FIELDS); use ``batch_ms`` (e.g. 10) to get them in fewer, larger records."""
        return self.records("/imu/stream", {"raw": raw, "batch_ms": batch_ms, "max_backlog": max_backlog}, **kw)

    def imu_calibrate(self, seconds: Optional[float] = None) -> dict:
        """Calibrate gyro bias + gravity from the next ``seconds`` of still data (keep it still)."""
        r = self.http.post("/imu/calibrate", json={"seconds": seconds}, timeout=(seconds or 1.0) + 15)
        return self._check(r).json()

    def imu_reset(self) -> dict:
        """Restart the orientation from gravity (yaw 0), keeping the calibration."""
        return self.post("/imu/reset", {})

    def frame(self, stream: str, mode: str = "raw") -> Record:
        """The latest frame of one stream (GET /frame/{stream})."""
        r = self._check(self.http.get(f"/frame/{stream}", params={"mode": mode}))
        t = time.monotonic()
        (h, p), = RecordReader().feed(r.content)
        with self._clock_lock:
            ts = self.clock.to_server(t) if self.clock.latest else None
        return Record(h, p, t, ts)


# -- client-side recording -------------------------------------------------------------------

def augment(rec: Record, sync: Optional[SyncEstimate]) -> bytes:
    """Re-pack a received record with the client receive times added to its header."""
    h = dict(rec.header)
    h["t_recv_client"] = rec.t_recv
    h["t_recv"] = rec.t_recv_server
    if sync is not None:
        h["sync_uncertainty"] = sync.uncertainty
    return pack_record(h, rec.payload)


def record(base_url: str, out_dir: str, streams: str = "color,depth", mode: str = "raw",
           duration: Optional[float] = None, imu: bool = True, resync_s: float = 30.0,
           progress: bool = True) -> dict:
    """Record streams to ``out_dir`` on *this* machine (the Pi has no disk to spare).

    Writes ``video.rec``, ``imu.rec`` (filter output, ~100 Hz) and ``imu_raw.rec`` (every 1 kHz
    sample) — the wire records, headers augmented with t_recv — and
    ``meta.json`` (settings, catalog, device, clock-sync history). Read back with
    ``read_recording``. Runs until ``duration`` s or Ctrl-C. Returns counts.
    """
    os.makedirs(out_dir, exist_ok=True)
    cam = OrbbecClient(base_url)
    est = cam.sync()
    cam.start_auto_sync(resync_s)
    stop = threading.Event()
    counts = {"frames": 0, "imu_records": 0, "imu_raw_samples": 0, "imu_dropped": 0, "frames_dropped": 0}
    meta = {"base_url": base_url, "streams": streams, "mode": mode, "started_wall": time.time(),
            "state": cam.state(), "catalog": cam.catalog(), "sync": [est.as_dict()],
            "clock": "all t_* in server time.monotonic() seconds; t_recv_client = this machine's monotonic"}

    def imu_loop(raw: bool):
        name, key = ("imu_raw.rec", "imu_raw_samples") if raw else ("imu.rec", "imu_records")
        with open(os.path.join(out_dir, name), "wb") as f:
            for rec in cam.imu(raw=raw, batch_ms=10.0 if raw else 0.0):
                f.write(augment(rec, cam.clock.latest))
                counts[key] += rec.header["n"]
                counts["imu_dropped"] += rec.header.get("dropped", 0)
                if stop.is_set():
                    break

    threads = []
    if imu:
        for raw in (False, True):
            threads.append(threading.Thread(target=imu_loop, args=(raw,), daemon=True))
            threads[-1].start()
    t_end = None if duration is None else time.monotonic() + duration
    try:
        with open(os.path.join(out_dir, "video.rec"), "wb") as f:
            for rec in cam.frames(streams, mode=mode, decode_jpeg=False):
                f.write(augment(rec, cam.clock.latest))
                counts["frames"] += 1
                counts["frames_dropped"] += rec.header.get("dropped", 0)
                if progress and counts["frames"] % 50 == 0:
                    print(f"\r{counts}", end="", flush=True)
                if t_end is not None and time.monotonic() > t_end:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for th in threads:
            th.join(timeout=3)
        meta["sync"] = [e.as_dict() for e in cam.clock.history]
        meta["drift"] = cam.clock.drift
        meta["counts"] = counts
        with open(os.path.join(out_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=1, default=str)
        cam.close()
    if progress:
        print(f"\r{counts}")
    return counts


def read_recording(path: str, decode_jpeg: bool = True) -> Iterator[tuple[dict, object]]:
    """Iterate (header, decoded data) of a ``.rec`` file written by ``record``."""
    with open(path, "rb") as f:
        for h, p in iter_records(f):
            yield h, decode_payload(h, p, decode_jpeg)
