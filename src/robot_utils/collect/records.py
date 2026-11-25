"""Raw record splitting and the Pi time of a record.

``RawSplitter`` cuts a byte stream into whole records and hands back each record's exact bytes
(what the node sent, never re-serialised) with its parsed header, so a recording can be rotated
and cut at record boundaries and still be byte-for-byte the node's stream.

``record_times`` says when a record happened on the Pi clock (``time.monotonic()`` of the Pi),
the time base of the whole collection. The field per record type:

==============  ======================================================================
record          time field
==============  ======================================================================
``rows``        per row: ``t_tick`` (arm, swerve, wbc), else ``t`` (estimator), else
                ``t_sensor`` / ``t_end`` / ``t_capture`` (camera IMU); NaN falls back to
                the next present column
``imu``         the same, from the fields carried inline
``frame``       ``t_sensor`` (≈ the capture instant, camera clock mapped by the SDK),
                else ``t_capture`` (SDK receipt on the Pi) when ``t_sensor`` is null
``headset``     ``t_rx`` (the Pi's arrival stamp of the Quest frame)
``sent``        ``t_send`` (when the datagram went to the WBC)
``event``       ``t`` (the event line's Pi monotonic)
``ping``        ``t_send`` (keep-alive; never cut into an episode)
``scene3d``     ``t_rx`` (this recorder's receipt of a rigid-scene pose frame; see
                ``Rsf1Splitter``)
==============  ======================================================================
"""
from __future__ import annotations

import json
import struct
import time
from typing import Callable, Iterator, Optional

import numpy as np

from robot_utils.nodes.common.protocol import MAX_HEADER, ROW_DTYPE, pack_record
from robot_utils.scene3d.format import _HEADER, FRAME_MAGIC, frame_bytes

_LEN = struct.Struct(">I")

ROW_TIME_FIELDS = ("t_tick", "t", "t_sensor", "t_end", "t_capture")
HEADER_TIME_FIELDS = {"headset": ("t_rx",), "sent": ("t_send",), "event": ("t",), "ping": ("t_send",),
                      "frame": ("t_sensor", "t_capture"), "scene3d": ("t_rx",)}


class RawSplitter:
    """``feed(chunk)`` -> ``[(raw_bytes, header), ...]`` for the records completed by the chunk."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list[tuple[bytes, dict]]:
        self._buf += chunk
        out = []
        pos = 0
        buf = self._buf
        while len(buf) - pos >= 4:
            n = _LEN.unpack_from(buf, pos)[0]
            if n > MAX_HEADER:
                raise ValueError(f"header length {n} too large (stream out of sync?)")
            if len(buf) - pos < 4 + n:
                break
            header = json.loads(bytes(buf[pos + 4:pos + 4 + n]))
            end = pos + 4 + n + int(header.get("len", 0))
            if len(buf) < end:
                break
            out.append((bytes(buf[pos:end]), header))
            pos = end
        if pos:
            del self._buf[:pos]
        return out

    @property
    def pending(self) -> int:
        """Bytes of an incomplete record held back (discarded if the connection ends here)."""
        return len(self._buf)


class Rsf1Splitter:
    """The sim's rigid-scene pose stream (``<sim>/scene3d/stream?format=bin``: RSF1 frames back to back, no record
    framing; docs/scene3d.md) -> records, so it is recorded, rotated and cut like every other stream: each frame
    becomes one ``scene3d`` record whose payload is the frame's exact bytes and whose header says when it arrived
    (``t_rx``, this machine's ``time.monotonic()``) and what it is (``seq``, ``t_sim`` (the frame's own ``t``: sim
    time, which stops on a pause and restarts on a reset, so not a time base), ``reset_seq``, ``scene_id``, ``n``,
    ``m``).

    ``t_rx`` is the recorder's clock: it is the robot's ("Pi") time only when the recorder runs on the sim's machine
    (the quest_teleop node's own recorder always does; ``collect run`` against a sim on another machine does not)."""

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self._buf = bytearray()
        self.clock = clock

    def feed(self, chunk: bytes) -> list[tuple[bytes, dict]]:
        self._buf += chunk
        out = []
        pos = 0
        buf = self._buf
        now = self.clock()
        while len(buf) - pos >= _HEADER.size:
            magic, seq, t, reset_seq, n, m, sid = _HEADER.unpack_from(buf, pos)
            if magic != FRAME_MAGIC:
                raise ValueError(f"bad RSF1 frame magic {bytes(magic)!r} (stream out of sync?)")
            end = pos + frame_bytes(n, m)
            if len(buf) < end:
                break
            h = {"type": "scene3d", "t_rx": now, "seq": seq, "t_sim": t, "reset_seq": reset_seq,
                 "scene_id": sid.hex(), "n": n, "m": m}
            payload = bytes(buf[pos:end])
            out.append((pack_record(h, payload), {**h, "len": len(payload)}))
            pos = end
        if pos:
            del self._buf[:pos]
        return out

    @property
    def pending(self) -> int:
        return len(self._buf)


def iter_raw(paths, chunk_size: int = 1 << 20) -> Iterator[tuple[bytes, dict, bytes]]:
    """(raw, header, payload) for every record of one or more recording files, in order.
    A truncated record at the end of a file (a crash mid-write) is ignored."""
    for path in paths:
        sp = RawSplitter()
        with open(path, "rb") as f:
            while True:
                chunk = f.read(chunk_size)
                if not chunk:
                    break
                for raw, h in sp.feed(chunk):
                    yield raw, h, raw[len(raw) - int(h.get("len", 0)):]


def _first_time(values: dict, names) -> Optional[float]:
    for k in names:
        v = values.get(k)
        if v is not None and not (isinstance(v, float) and np.isnan(v)):
            return float(v)
    return None


def row_time_columns(fields) -> list[int]:
    """Indices of the time columns of a rows schema, in preference order."""
    return [fields.index(k) for k in ROW_TIME_FIELDS if k in fields]


def rows_times(payload: bytes, fields, dtype: str = ROW_DTYPE) -> Optional[np.ndarray]:
    """Per-row Pi time of a rows payload (NaN where no time column has a value), or None if the
    schema has no time column."""
    cols = row_time_columns(list(fields))
    if not cols:
        return None
    a = np.frombuffer(payload, dtype).reshape(-1, len(fields))
    t = a[:, cols[0]].astype(float)
    for c in cols[1:]:
        bad = np.isnan(t)
        if not bad.any():
            break
        t[bad] = a[bad, c]
    return t


def event_line(header: dict) -> dict:
    """The event line of an ``event`` record: its header fields (the contract's reading), or a nested
    ``event`` object if the node sends it that way."""
    nested = header.get("event")
    if isinstance(nested, dict):
        return nested
    return {k: v for k, v in header.items() if k not in ("type", "len")}


def header_time(header: dict) -> Optional[float]:
    """The Pi time of a non-rows record (see the module table), or None."""
    if header.get("type") == "event":
        return _first_time(event_line(header), ("t",))
    names = HEADER_TIME_FIELDS.get(header.get("type"))
    return _first_time(header, names) if names else None


def time_field_name(header: dict, fields=None) -> Optional[str]:
    """The name of the field that dates this kind of record (for the docs / episode.json)."""
    if fields is not None:
        cols = [k for k in ROW_TIME_FIELDS if k in fields]
        return cols[0] if cols else None
    names = HEADER_TIME_FIELDS.get(header.get("type"))
    return names[0] if names else None


def slice_rows_record(header: dict, payload: bytes, fields, mask: np.ndarray) -> bytes:
    """A rows / imu record with only the rows in ``mask`` (header fields kept, ``n`` updated)."""
    dtype = header.get("dtype", ROW_DTYPE)
    a = np.frombuffer(payload, dtype).reshape(-1, len(fields))[mask]
    h = {k: v for k, v in header.items() if k != "len"}
    if "n" in h:
        h["n"] = int(a.shape[0])
    return pack_record(h, np.ascontiguousarray(a).tobytes())
