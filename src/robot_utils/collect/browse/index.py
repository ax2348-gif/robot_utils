"""The time index of a recording: where every record is and when it happened, without loading the data.

One ``StreamIndex`` per recorded stream (a session's rotated ``<name>.NNN.bin`` files or an episode's
``<name>.bin``). Building it reads each file once:

- every record's header (the JSON) and its position: ``hoff`` / ``hlen`` (the header JSON), ``poff`` /
  ``plen`` (the payload), ``file`` (which file), ``kind`` (record type code, ``KINDS``);
- its Pi time span ``t0`` / ``t1`` (``collect.records``: per row for rows records, the type's field otherwise)
  and its row count ``n`` / schema (``schema``: an index into ``schemas``; -1 when it has none);
- frames: ``width`` / ``height``, and whether the payload is a JPEG (``jpeg``).

Pings and schema records are not indexed (schemas are kept in ``schemas``). The arrays are written once to a
cache directory (``.npy`` per array, opened memory-mapped) keyed by the files' paths, sizes and mtimes, so a
reopened recording indexes instantly and a large one never sits in memory.

Reading rows (``rows``) gathers only the requested columns of the records that overlap a time window, straight
from the memory-mapped files (one vectorised gather per schema and file: no per-record Python on the hot path).
"""
from __future__ import annotations

import hashlib
import json
import mmap
import os
import struct
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from ..records import HEADER_TIME_FIELDS, ROW_TIME_FIELDS, event_line

INDEX_VERSION = 4
KINDS = {"rows": 1, "frame": 2, "headset": 3, "sent": 4, "event": 5, "imu": 6, "scene3d": 7}
KIND_NAMES = {v: k for k, v in KINDS.items()}
_LEN = struct.Struct(">I")
_ARRAYS = ("file", "kind", "hoff", "hlen", "poff", "plen", "t0", "t1", "n", "schema", "width", "height", "jpeg")
_DTYPES = {"file": np.uint16, "kind": np.uint8, "hoff": np.uint64, "hlen": np.uint32, "poff": np.uint64,
           "plen": np.uint32, "t0": np.float64, "t1": np.float64, "n": np.uint32, "schema": np.int32,
           "width": np.uint16, "height": np.uint16, "jpeg": np.bool_}


def cache_root() -> Path:
    return Path(os.environ.get("ROBOT_UTILS_BROWSE_CACHE", Path.home() / ".cache" / "robot_utils" / "browse"))


def files_key(files: Sequence[Path], extra: str = "") -> str:
    h = hashlib.sha1(f"v{INDEX_VERSION}{extra}".encode())
    for f in files:
        st = os.stat(f)
        h.update(f"{Path(f).resolve()}|{st.st_size}|{st.st_mtime_ns};".encode())
    return h.hexdigest()[:20]


@dataclass
class Schema:
    fields: list
    units: list
    dtype: str = "<f8"
    time_cols: list = field(default_factory=list)    # indices of the time columns, in preference order

    @classmethod
    def from_header(cls, h: dict) -> "Schema":
        fields = list(h.get("fields") or [])
        units = list(h.get("units") or [""] * len(fields))
        return cls(fields, units, h.get("dtype", "<f8"), [fields.index(k) for k in ROW_TIME_FIELDS if k in fields])

    def key(self) -> tuple:
        return (tuple(self.fields), self.dtype)


class _Builder:
    def __init__(self):
        self.cols = {k: [] for k in _ARRAYS}
        self.schemas: list[Schema] = []
        self._schema_ix: dict[tuple, int] = {}
        self.types: dict[str, int] = {}
        self.dropped = 0

    def schema(self, h: dict) -> int:
        s = Schema.from_header(h)
        k = s.key()
        if k not in self._schema_ix:
            self._schema_ix[k] = len(self.schemas)
            self.schemas.append(s)
        return self._schema_ix[k]

    def add(self, **kw):
        for k in _ARRAYS:
            self.cols[k].append(kw.get(k, 0))

    def arrays(self) -> dict:
        return {k: np.asarray(v, dtype=_DTYPES[k]) for k, v in self.cols.items()}


def _first_time(d: dict, names) -> float:
    for k in names:
        v = d.get(k)
        if isinstance(v, (int, float)) and not isinstance(v, bool) and np.isfinite(v):
            return float(v)
    return np.nan


def scan_file(path: Path, file_ix: int, b: _Builder) -> None:
    """Index one recording file (a truncated last record is ignored, as ``records.iter_raw`` does)."""
    size = os.path.getsize(path)
    if size == 0:
        return
    with open(path, "rb") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    try:
        current: Optional[int] = None     # the schema in force for header-less rows records
        by_id: dict = {}
        pos = 0
        while pos + 4 <= size:
            n = _LEN.unpack_from(mm, pos)[0]
            if pos + 4 + n > size:
                break
            h = json.loads(mm[pos + 4:pos + 4 + n])
            plen = int(h.get("len", 0))
            poff = pos + 4 + n
            if poff + plen > size:
                break
            kind = h.get("type")
            b.types[kind] = b.types.get(kind, 0) + 1
            if kind == "schema":
                current = b.schema(h)
                by_id[h.get("schema")] = current
            elif kind == "ping":
                pass
            elif h.get("fields") is not None or kind == "rows":
                si = b.schema(h) if h.get("fields") is not None else by_id.get(h.get("schema"), current)
                if si is not None:
                    s = b.schemas[si]
                    ncol = len(s.fields)
                    if ncol and s.time_cols and plen:
                        a = np.frombuffer(mm, s.dtype, count=plen // np.dtype(s.dtype).itemsize,
                                          offset=poff).reshape(-1, ncol)
                        t = a[:, s.time_cols[0]].astype(float)
                        for c in s.time_cols[1:]:
                            bad = np.isnan(t)
                            if not bad.any():
                                break
                            t[bad] = a[bad, c]
                        nrow = a.shape[0]
                        del a                                   # release the mmap export before close
                        ok = t[np.isfinite(t)]
                        if ok.size:
                            b.add(file=file_ix, kind=KINDS.get(kind, KINDS["rows"]), hoff=pos + 4, hlen=n, poff=poff,
                                  plen=plen, t0=float(ok.min()), t1=float(ok.max()), n=nrow, schema=si)
                b.dropped += int(h.get("dropped") or 0)
            else:
                if kind == "event":
                    t = _first_time(event_line(h), ("t",))
                else:
                    t = _first_time(h, HEADER_TIME_FIELDS.get(kind, ()))
                if np.isfinite(t):
                    jpeg = kind == "frame" and plen > 3 and mm[poff:poff + 2] == b"\xff\xd8"
                    b.add(file=file_ix, kind=KINDS.get(kind, 0), hoff=pos + 4, hlen=n, poff=poff, plen=plen,
                          t0=t, t1=t, n=1, schema=-1, width=int(h.get("width") or 0), height=int(h.get("height") or 0),
                          jpeg=jpeg)
                b.dropped += int(h.get("dropped") or 0)
            pos = poff + plen
    finally:
        try:
            mm.close()
        except BufferError:                                   # a view still alive: the GC closes it
            pass


class StreamIndex:
    """The index of one stream (see the module docstring). ``kind``: ``rows`` | ``frames`` | ``oplog`` | ``scene3d``
    (rigid-scene pose frames, dated by ``t_rx``) | ``other``, from what the stream holds."""

    def __init__(self, name: str, files: Sequence[Path], arrays: dict, meta: dict):
        self.name = name
        self.files = [Path(f) for f in files]
        self.a = arrays
        self.schemas = [Schema(**s) for s in meta["schemas"]]
        self.types: dict = meta.get("types", {})
        self.dropped: int = meta.get("dropped", 0)
        self.kind = meta.get("stream_kind", "other")
        self._mm: dict[int, np.ndarray] = {}
        self._lock = threading.Lock()
        t0, t1 = self.a["t0"], self.a["t1"]
        self.n_records = len(t0)
        # monotone envelopes for window lookups (records are in time order up to small reorderings)
        self._t1max = np.maximum.accumulate(t1) if len(t1) else t1
        self._t0max = np.maximum.accumulate(t0) if len(t0) else t0

    # -- building / caching ---------------------------------------------------------------------

    @classmethod
    def open(cls, name: str, files: Sequence[Path], cache: Optional[Path] = None) -> "StreamIndex":
        files = [Path(f) for f in files]
        root = cache_root() if cache is None else Path(cache)
        d = root / f"{name}-{files_key(files)}"
        meta_p = d / "meta.json"
        if meta_p.exists():
            try:
                meta = json.loads(meta_p.read_text())
                arrays = {k: np.load(d / f"{k}.npy", mmap_mode="r") for k in _ARRAYS}
                return cls(name, files, arrays, meta)
            except (OSError, ValueError, KeyError):
                pass
        b = _Builder()
        for i, f in enumerate(files):
            scan_file(f, i, b)
        arrays = b.arrays()
        if len(arrays["t0"]):                                   # time order (stable): a reconnect never goes back
            order = np.argsort(arrays["t0"], kind="stable")
            if np.any(order != np.arange(len(order))):
                arrays = {k: v[order] for k, v in arrays.items()}
        meta = {"schemas": [s.__dict__ for s in b.schemas], "types": b.types, "dropped": b.dropped,
                "stream_kind": cls._kind_of(b.types), "files": [str(f) for f in files]}
        try:
            d.mkdir(parents=True, exist_ok=True)
            for k, v in arrays.items():
                np.save(d / f"{k}.npy", v)
            tmp = d / "meta.json.tmp"
            tmp.write_text(json.dumps(meta))
            os.replace(tmp, meta_p)
            arrays = {k: np.load(d / f"{k}.npy", mmap_mode="r") for k in _ARRAYS}
        except OSError:
            pass                                                 # read-only cache: keep it in memory
        return cls(name, files, arrays, meta)

    @staticmethod
    def _kind_of(types: dict) -> str:
        if types.get("headset") or types.get("sent") or types.get("event"):
            return "oplog"
        if types.get("frame"):
            return "frames"
        if types.get("scene3d"):
            return "scene3d"
        if types.get("rows") or types.get("imu"):
            return "rows"
        return "other"

    # -- raw access ------------------------------------------------------------------------------

    def _map(self, file_ix: int) -> np.ndarray:
        m = self._mm.get(file_ix)
        if m is None:
            with self._lock:
                m = self._mm.get(file_ix)
                if m is None:
                    p = self.files[file_ix]
                    m = np.memmap(p, np.uint8, "r") if os.path.getsize(p) else np.zeros(0, np.uint8)
                    self._mm[file_ix] = m
        return m

    def header(self, i: int) -> dict:
        m = self._map(int(self.a["file"][i]))
        o = int(self.a["hoff"][i])
        return json.loads(m[o:o + int(self.a["hlen"][i])].tobytes())

    def payload(self, i: int) -> bytes:
        m = self._map(int(self.a["file"][i]))
        o = int(self.a["poff"][i])
        return m[o:o + int(self.a["plen"][i])].tobytes()

    # -- time lookups ----------------------------------------------------------------------------

    @property
    def t_range(self) -> tuple[float, float]:
        if not self.n_records:
            return (np.nan, np.nan)
        return float(np.min(self.a["t0"])), float(np.max(self.a["t1"]))

    def window(self, a: float, b: float, kind: Optional[str] = None) -> np.ndarray:
        """Indices of the records whose time span overlaps ``[a, b]`` (optionally of one record type)."""
        if not self.n_records:
            return np.zeros(0, np.int64)
        lo = int(np.searchsorted(self._t1max, a, "left"))
        hi = int(np.searchsorted(self._t0max, b, "right"))
        ix = np.arange(lo, max(lo, hi))
        if len(ix):
            ix = ix[(self.a["t1"][ix] >= a) & (self.a["t0"][ix] <= b)]
        if kind is not None and len(ix):
            ix = ix[self.a["kind"][ix] == KINDS[kind]]
        return ix

    def at(self, t: float, kind: Optional[str] = None) -> int:
        """The newest record at or before ``t`` (-1 if none); for frames: the frame shown at ``t``."""
        t0 = self.a["t0"]
        i = int(np.searchsorted(t0, t, "right")) - 1
        if kind is not None:
            k = KINDS[kind]
            while i >= 0 and self.a["kind"][i] != k:
                i -= 1
        return i

    # -- rows ------------------------------------------------------------------------------------

    def fields(self) -> list[tuple[str, str]]:
        """Every column any schema of this stream has, in first-seen order, with its unit."""
        out, seen = [], set()
        for s in self.schemas:
            for f, u in zip(s.fields, s.units):
                if f not in seen:
                    seen.add(f)
                    out.append((f, u))
        return out

    def rows(self, a: float, b: float, fields: Sequence[str], stride: int = 1) -> tuple[np.ndarray, np.ndarray]:
        """``(t, values)`` for the rows with Pi time in ``[a, b]``: ``t`` (n,) float64 sorted, ``values`` (n, k)
        float64 with NaN where a row's schema lacks a column. ``stride`` > 1 keeps every stride-th row."""
        ix = self.window(a, b)
        if len(ix):
            ix = ix[self.a["schema"][ix] >= 0]
        ts, vs = [], []
        if len(ix):
            schema = self.a["schema"][ix]
            files = self.a["file"][ix]
            for si in np.unique(schema):
                s = self.schemas[int(si)]
                if not s.time_cols:
                    continue
                cols = [s.fields.index(f) if f in s.fields else -1 for f in fields]
                for fi in np.unique(files[schema == si]):
                    sel = ix[(schema == si) & (files == fi)]
                    t, v = self._gather(int(fi), sel, s, cols)
                    m = (t >= a) & (t <= b)
                    ts.append(t[m])
                    vs.append(v[m])
        if not ts:
            return np.zeros(0), np.zeros((0, len(fields)))
        t = np.concatenate(ts)
        v = np.concatenate(vs)
        if len(ts) > 1 or np.any(np.diff(t) < 0):
            o = np.argsort(t, kind="stable")
            t, v = t[o], v[o]
        if stride > 1:
            t, v = t[::stride], v[::stride]
        return t, v

    def _gather(self, file_ix: int, recs: np.ndarray, s: Schema, cols: list) -> tuple[np.ndarray, np.ndarray]:
        m = self._map(file_ix)
        isz = np.dtype(s.dtype).itemsize
        ncol = len(s.fields)
        n = self.a["n"][recs].astype(np.int64)
        total = int(n.sum())
        starts = np.repeat(self.a["poff"][recs].astype(np.int64), n)
        within = np.arange(total) - np.repeat(np.cumsum(n) - n, n)
        rowoff = starts + within * (ncol * isz)
        byte = np.arange(isz)

        def col(c: int) -> np.ndarray:
            return m[(rowoff + c * isz)[:, None] + byte].reshape(-1).view(s.dtype).astype(np.float64)

        t = col(s.time_cols[0])
        for c in s.time_cols[1:]:
            bad = np.isnan(t)
            if not bad.any():
                break
            t[bad] = col(c)[bad]
        v = np.full((total, len(cols)), np.nan)
        for k, c in enumerate(cols):
            if c >= 0:
                v[:, k] = col(c)
        return t, v

    def envelope(self, a: float, b: float, field_name: str, buckets: int) -> dict:
        """Min / max / mean of one column per time bucket over ``[a, b]`` (an overview strip)."""
        t, v = self.rows(a, b, [field_name])
        edges = np.linspace(a, b, buckets + 1)
        k = np.clip(np.searchsorted(edges, t, "right") - 1, 0, buckets - 1)
        v = v[:, 0]
        ok = np.isfinite(v)
        lo = np.full(buckets, np.nan)
        hi = np.full(buckets, np.nan)
        if ok.any():
            np.fmin.at(lo, k[ok], v[ok])
            np.fmax.at(hi, k[ok], v[ok])
        return {"t0": a, "t1": b, "min": lo, "max": hi}
