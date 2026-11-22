"""One recorded stream: its URL, its files (rotated by size), its gap sidecar.

Stream specs (``--streams``), relative to ``<robot>/nodes/``:

- ``<node>/<path>/stream`` (e.g. ``arm/stream``, ``quest_teleop/oplog/stream``): that endpoint as is.
- ``<camera node>/<camera stream>`` (e.g. ``camera/color``, ``wrist_camera/color``): the Orbbec node's
  ``/stream?streams=<camera stream>&mode=jpeg`` (colour = the camera's MJPG bytes as sent).
- anything with a ``?`` is used verbatim (``camera/stream?streams=depth&mode=raw``).

File names: the spec with a trailing ``/stream`` dropped and ``/`` -> ``.`` (``arm``,
``camera.color``, ``quest_teleop.oplog``), then ``.NNN.bin`` per rotation and ``.gaps.jsonl``.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import httpx

from .records import RawSplitter, header_time, rows_times

log = logging.getLogger("collect")

DEFAULT_STREAMS = ("camera/color", "wrist_camera/color", "arm/stream", "swerve/stream", "estimator/stream",
                   "wbc/stream", "quest_teleop/oplog/stream")
OPLOG_SPEC = "quest_teleop/oplog/stream"


@dataclass
class StreamSpec:
    spec: str
    name: str
    path: str                     # under the robot's /nodes/
    params: dict = field(default_factory=dict)

    @classmethod
    def parse(cls, spec: str) -> "StreamSpec":
        spec = spec.strip().strip("/")
        if not spec or "/" not in spec:
            raise ValueError(f"stream spec {spec!r}: expected <node>/<...>")
        if "?" in spec:
            path, _, query = spec.partition("?")
            params = dict(kv.split("=", 1) for kv in query.split("&") if "=" in kv)
            name = re.sub(r"[^A-Za-z0-9_.-]+", ".", spec).strip(".")
            return cls(spec, name, path, params)
        if spec.endswith("/stream"):
            return cls(spec, spec[:-len("/stream")].replace("/", "."), spec)
        node, _, cam = spec.partition("/")
        if "/" in cam:
            raise ValueError(f"stream spec {spec!r}: a camera spec is <node>/<stream>")
        return cls(spec, f"{node}.{cam}", f"{node}/stream", {"streams": cam, "mode": "jpeg"})

    def url(self, robot: str) -> str:
        return f"{robot.rstrip('/')}/nodes/{self.path}"

    def to_json(self, robot: str) -> dict:
        return {"spec": self.spec, "name": self.name, "url": self.url(robot), "params": self.params}


def rotated_files(session: Path, name: str) -> list[Path]:
    """A stream's recording files in order."""
    return sorted(session.glob(f"{glob_escape(name)}.[0-9][0-9][0-9].bin"))


def glob_escape(s: str) -> str:
    return re.sub(r"([*?\[])", r"[\1]", s)


class StreamStats:
    def __init__(self):
        self.records = 0
        self.rows = 0
        self.bytes = 0
        self.dropped = 0
        self.gaps = 0
        self.connected = False
        self.last_pi_t: Optional[float] = None
        self._win: list[tuple[float, int]] = []     # (our monotonic, items) for the rate

    def count(self, items: int):
        now = time.monotonic()
        self._win.append((now, items))
        while self._win and now - self._win[0][0] > 5.0:
            self._win.pop(0)

    def rate(self) -> float:
        if len(self._win) < 2:
            return 0.0
        span = self._win[-1][0] - self._win[0][0]
        return sum(n for _, n in self._win[1:]) / span if span > 0 else 0.0


class StreamRecorder:
    """Records one stream continuously: raw record bytes to ``<name>.NNN.bin`` (rotated at record
    boundaries once a file passes ``rotate_bytes``; a new file starts with the stream's schema
    records so it decodes on its own), reconnecting on any error with a line per gap in
    ``<name>.gaps.jsonl``."""

    def __init__(self, spec: StreamSpec, robot: str, session: Path, *, rotate_bytes: int = 512 << 20,
                 read_timeout: float = 10.0, on_record: Optional[Callable[[dict], None]] = None,
                 on_connect: Optional[Callable[[], None]] = None, retry_max: float = 5.0):
        self.spec = spec
        self.url = spec.url(robot)
        self.session = session
        self.rotate_bytes = rotate_bytes
        self.read_timeout = read_timeout
        self.on_record = on_record
        self.on_connect = on_connect
        self.retry_max = retry_max
        self.stats = StreamStats()
        self._file = None
        self._file_index = -1
        self._file_bytes = 0
        self._schemas: dict = {}          # schema id -> raw bytes (re-written at each rotation)
        self._fields: dict = {}           # schema id -> fields
        self._gap: Optional[dict] = None  # open gap (between an error and the first timed record after)
        self._last_flush = time.monotonic()
        self.files: list[str] = []

    # -- files ---------------------------------------------------------------------------------

    def _open_next(self):
        if self._file is not None:
            self._file.close()
        self._file_index += 1
        path = self.session / f"{self.spec.name}.{self._file_index:03d}.bin"
        self._file = open(path, "wb")
        self.files.append(path.name)
        self._file_bytes = 0
        for raw in self._schemas.values():
            self._write(raw)

    def _write(self, raw: bytes):
        self._file.write(raw)
        self._file_bytes += len(raw)
        self.stats.bytes += len(raw)

    def _write_record(self, raw: bytes):
        if self._file is None or (self._file_bytes and self._file_bytes + len(raw) > self.rotate_bytes):
            self._open_next()
        self._write(raw)
        now = time.monotonic()
        if now - self._last_flush > 1.0:
            self._file.flush()
            self._last_flush = now

    def close(self):
        if self._gap is not None:
            self._write_gap(None)
        if self._file is not None:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._file.close()
            self._file = None

    # -- gaps ----------------------------------------------------------------------------------

    def _gap_start(self, error: str, partial: int):
        self.stats.connected = False
        if self._gap is None:
            self._gap = {"wall": time.time(), "mono": time.monotonic(), "error": error,
                         "pi_t_last": self.stats.last_pi_t, "partial_bytes": partial, "attempts": 0,
                         "errors": []}
            self.stats.gaps += 1
            log.warning("%s: %s; reconnecting", self.spec.name, error)
        else:
            if len(self._gap["errors"]) < 20:
                self._gap["errors"].append(error)

    def _write_gap(self, pi_t_first: Optional[float]):
        g = self._gap
        self._gap = None
        line = {"wall": g["wall"], "mono": g["mono"], "error": g["error"], "pi_t_last": g["pi_t_last"],
                "partial_bytes": g["partial_bytes"],
                "reconnect": None if pi_t_first is None and "reconnect_wall" not in g else {
                    "wall": g.get("reconnect_wall"), "mono": g.get("reconnect_mono"),
                    "attempts": g["attempts"], "pi_t_first": pi_t_first},
                "retry_errors": g["errors"]}
        with open(self.session / f"{self.spec.name}.gaps.jsonl", "a") as f:
            f.write(json.dumps(line) + "\n")
            f.flush()
            os.fsync(f.fileno())

    # -- records -------------------------------------------------------------------------------

    def _handle(self, raw: bytes, h: dict):
        kind = h.get("type")
        payload = raw[len(raw) - int(h.get("len", 0)):]
        t = None
        items = 1
        if kind == "schema":
            self._schemas[h.get("schema")] = raw
            self._fields[h.get("schema")] = h.get("fields")
        else:
            fields = h.get("fields") or (self._fields.get(h.get("schema")) if kind == "rows" else None)
            if fields:
                ts = rows_times(payload, fields, h.get("dtype", "<f8"))
                items = len(ts) if ts is not None else int(h.get("n", 1))
                if ts is not None and len(ts):
                    t = float(ts[-1])
                self.stats.rows += items
            else:
                t = header_time(h)
            self.stats.dropped += int(h.get("dropped") or 0)
        self._write_record(raw)
        self.stats.records += 1
        if kind not in ("schema", "ping"):
            self.stats.count(items)
        if t is not None:
            self.stats.last_pi_t = t
            if self._gap is not None and "reconnect_wall" in self._gap:
                self._write_gap(t)
        if self.on_record is not None:
            self.on_record(h)

    # -- loop ----------------------------------------------------------------------------------

    async def run(self, client: httpx.AsyncClient):
        """Record until cancelled."""
        delay = 0.2
        timeout = httpx.Timeout(5.0, read=self.read_timeout)
        try:
            while True:
                splitter = RawSplitter()
                try:
                    async with client.stream("GET", self.url, params=self.spec.params, timeout=timeout) as r:
                        if r.status_code >= 400:
                            await r.aread()
                            raise RuntimeError(f"HTTP {r.status_code}: {r.text[:200]}")
                        self.stats.connected = True
                        if self._gap is not None:
                            self._gap["reconnect_wall"] = time.time()
                            self._gap["reconnect_mono"] = time.monotonic()
                        if self.on_connect is not None:
                            self.on_connect()
                        delay = 0.2
                        async for chunk in r.aiter_raw():
                            for raw, h in splitter.feed(chunk):
                                self._handle(raw, h)
                    raise RuntimeError("stream ended")
                except asyncio.CancelledError:
                    raise
                except Exception as e:     # noqa: BLE001 (any failure: gap + reconnect)
                    err = f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
                    self._gap_start(err, splitter.pending)
                    self._gap["attempts"] += 1
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, self.retry_max)
        finally:
            self.stats.connected = False
