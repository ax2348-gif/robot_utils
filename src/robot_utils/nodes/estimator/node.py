"""Estimator node core: reads the swerve and camera-IMU streams, fuses them (fusion.py), publishes poses.

Threads (no web code here; server.py serves it):

- one **input thread per upstream** (``Upstream``): reads the upstream's binary stream with
  ``common.client.NodeClient``, converts its timestamps to this machine's monotonic clock with
  an NTP-style offset (re-synced every 30 s; ~0 when the upstream runs on the same Pi), and puts
  parsed samples on the fusion queue. It reconnects after errors.
- the **fusion thread**: the only code that touches ``PoseFusion``. The swerve odometry is the
  master: each tick waits until the IMU heading track covers its sample time (the IMU arrives a
  few ms later) or ``IMU_WAIT_S`` passes (then the wheel heading is used for that tick), and is
  fused at its own timestamp. With no swerve data for ``ODOM_STALE_S``, IMU records become the
  master (heading only).
"""
from __future__ import annotations

import logging
import math
import queue
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from ..common.client import NodeClient
from ..common.hub import RowHub
from ..common.protocol import RowsDecoder
from ..common.server import EventLog
from .fusion import FIELDS, MASTER_IMU, PoseFusion, odometry_sample

log = logging.getLogger("estimator")

IMU_WAIT_S = 0.10        # max wait for IMU coverage of an odometry tick before using the wheel heading
ODOM_STALE_S = 0.5       # no odometry for this long -> heading-only mode on IMU records
SYNC_EVERY_S = 30.0
RECONNECT_S = 1.0


class Upstream(threading.Thread):
    """Reads one upstream stream into the fusion queue; keeps its own status."""

    def __init__(self, name: str, url: str, path: str, out: queue.Queue, events: EventLog,
                 params: Optional[dict] = None):
        super().__init__(name=f"in-{name}", daemon=True)
        self.key, self.url, self.path, self.out, self.events = name, url.rstrip("/"), path, out, events
        self.params = params or {}
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.offset = 0.0                 # upstream clock - local clock (s)
        self.uncertainty: Optional[float] = None
        self.connected = False
        self.error: Optional[str] = None
        self.count = 0
        self.last_rx: Optional[float] = None
        self._times: deque = deque(maxlen=200)
        self._client: Optional[NodeClient] = None

    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            ts = list(self._times)
            rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 10 and ts[-1] > ts[0] else None
            return {"url": self.url + self.path, "connected": self.connected, "error": self.error,
                    "rate_hz": None if rate is None or now - ts[-1] > 1.0 else round(rate, 1),
                    "last_age": None if self.last_rx is None else round(now - self.last_rx, 3),
                    "samples": self.count, "clock_offset": round(self.offset, 6),
                    "clock_uncertainty": None if self.uncertainty is None else round(self.uncertainty, 6)}

    def _sync(self):
        est = self._client.sync()
        with self.lock:
            self.offset, self.uncertainty = est.offset, est.uncertainty

    def run(self):
        reported = None
        while not self.stop.is_set():
            try:
                self._client = NodeClient.for_instance(self.url)   # name -> socket (or URL)
                self._sync()
                next_sync = time.monotonic() + SYNC_EVERY_S
                dec = RowsDecoder()
                with self.lock:
                    self.connected, self.error = True, None
                self.events.add(f"{self.key}: connected to {self.url}{self.path} "
                                f"(clock offset {self.offset * 1e3:+.2f} ± {self.uncertainty * 1e3:.2f} ms)")
                reported = None
                for h, p, _ in self._client.records(self.path, **self.params):
                    if self.stop.is_set():
                        return
                    rows = dec.decode(h, p)
                    if rows is not None and len(rows):
                        now = time.monotonic()
                        with self.lock:
                            self.count += len(rows)
                            self.last_rx = now
                            self._times.extend([now] * len(rows))
                        self.out.put((self.key, rows, self.offset))
                    if time.monotonic() > next_sync:
                        next_sync = time.monotonic() + SYNC_EVERY_S
                        self._sync()       # brief pause of reading; the socket buffers meanwhile
                raise ConnectionError("stream ended")
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


class EstimatorNode:
    def __init__(self, swerve_url: Optional[str] = "swerve", camera_url: Optional[str] = "camera"):
        """Upstreams: instance names on this machine (unix sockets) or URLs; None = off."""
        self.hub = RowHub({"pose": len(FIELDS)}, recent=1000)
        self.events = EventLog(on_add=self.hub.state_changed)
        self.q: queue.Queue = queue.Queue(maxsize=10000)
        self.fusion = PoseFusion()
        self.inputs: dict[str, Upstream] = {}
        if swerve_url:
            self.inputs["swerve"] = Upstream("swerve", swerve_url, "/stream", self.q, self.events)
        if camera_url:
            self.inputs["imu"] = Upstream("imu", camera_url, "/imu/stream", self.q, self.events)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self._reset: Optional[tuple] = None
        self._pending: deque = deque()          # (OdomSample, arrival) waiting for IMU coverage
        self._last_odom_rx = -1e9
        self._out_times: deque = deque(maxlen=200)
        self._imu_used: deque = deque(maxlen=500)    # imu_ok of the last 500 ticks (5 s)
        self._thread = threading.Thread(target=self._run, name="fusion", daemon=True)

    # -- lifecycle ----------------------------------------------------------------------------

    def start(self):
        self.events.add("estimator started: master = swerve odometry, heading = camera IMU (interpolated)")
        for u in self.inputs.values():
            u.start()
        self._thread.start()

    def close(self):
        self.stop.set()
        for u in self.inputs.values():
            u.stop.set()

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Set the pose (applied by the fusion thread before the next sample)."""
        with self.lock:
            self._reset = (x, y, theta)
        self.events.add(f"pose reset to x={x:.3f} m, y={y:.3f} m, theta={math.degrees(theta):.1f}°")

    # -- fusion thread ------------------------------------------------------------------------

    def _run(self):
        while not self.stop.is_set():
            try:
                item = self.q.get(timeout=0.02)
            except queue.Empty:
                item = None
            with self.lock:
                r, self._reset = self._reset, None
            if r is not None:
                self.fusion.reset(*r)
                self._pending.clear()
            out = []
            while item is not None:
                key, rows, offset = item
                if key == "imu":
                    out += self._imu(rows, offset)
                else:
                    self._odom(rows, offset)
                try:
                    item = self.q.get_nowait()
                except queue.Empty:
                    item = None
            out += self._drain()
            if out:
                now = time.monotonic()
                with self.lock:
                    self._out_times.extend([now] * len(out))
                self.hub.publish_rows(np.array(out, dtype=np.float64), "pose")

    def _imu(self, rows, offset) -> list:
        out = []
        imu_master = "swerve" not in self.inputs or time.monotonic() - self._last_odom_rx > ODOM_STALE_S
        for r in rows:
            t1 = float(r["t_end"]) - offset
            d = self.fusion.heading.add(float(r["t_start"]) - offset, t1,
                                        (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])),
                                        (float(r["dax"]), float(r["day"]), float(r["daz"])), float(r["flags"]))
            if d is not None and imu_master:
                row = self.fusion.imu_only(t1)
                if row is not None:
                    out.append(row)
        return out

    def _odom(self, rows, offset):
        now = time.monotonic()
        self._last_odom_rx = now
        for r in rows:
            s = odometry_sample(r)
            s.t -= offset
            self._pending.append((s, now))

    def _drain(self) -> list:
        out, now = [], time.monotonic()
        imu_live = "imu" in self.inputs and self.inputs["imu"].connected
        while self._pending:
            s, arrived = self._pending[0]
            end = self.fusion.heading.end
            covered = end is not None and end >= s.t
            if not covered and imu_live and now - arrived < IMU_WAIT_S:
                break                                   # wait a little for the IMU to catch up
            self._pending.popleft()
            row = self.fusion.odom(s, end)
            if row is not None:
                out.append(row)
                with self.lock:
                    self._imu_used.append(row[FIELDS.index("imu_ok")] == 1.0)
        return out

    # -- state --------------------------------------------------------------------------------

    def state(self) -> dict:
        now = time.monotonic()
        last = self.hub.recent(1, "pose")
        pose = ({k: (v if math.isfinite(v) else None) for k, v in zip(FIELDS, last[0].tolist())}
                if len(last) else None)
        with self.lock:
            ts = list(self._out_times)
            used = list(self._imu_used)
        live = bool(ts) and now - ts[-1] < 1.0
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if live and len(ts) > 10 and ts[-1] > ts[0] else None
        inputs = {k: u.status() for k, u in self.inputs.items()}
        sw = inputs.get("swerve", {}).get("rate_hz")
        im = inputs.get("imu", {}).get("rate_hz")
        if not live:
            mode = "waiting for inputs"
        elif pose and pose["master"] == MASTER_IMU:
            mode = "heading only (IMU; no swerve data)"
        elif im and sw:
            mode = "fused (wheel odometry + IMU heading)"
        else:
            mode = "odometry only (no IMU)"
        errors = [f"{k}: {v['error']}" for k, v in inputs.items() if v["error"]]
        return {
            "state": "running" if live else "waiting",
            "error": "; ".join(errors) if errors and not live else None,
            "mode": mode,
            "pose": pose,
            "rate_hz": None if rate is None else round(rate, 1),
            "imu_coverage": None if not used else round(sum(used) / len(used), 4),   # last 5 s of ticks
            "imu_still": self.fusion.heading.still,
            "inputs": inputs,
            "events": self.events.list(),
        }
