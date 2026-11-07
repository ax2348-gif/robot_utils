"""Binary record framing shared by every node's streaming endpoints (and client recordings).

A stream is a sequence of records; each record is::

    [u32 big-endian header length N][N bytes of UTF-8 JSON header][header["len"] bytes of payload]

Every header has ``type`` and ``len``. Types used across nodes:

``ping``    keep-alive (no payload), sent when nothing else was sent for about 1 s.
``schema``  describes the columns of the ``rows`` records that follow on the same stream:
            ``{"type": "schema", "schema": <id>, "fields": [...], "units": [...], "dtype": "<f8"}``.
            Sent once at the start of a stream (and again if the columns ever change).
``rows``    every-sample data: a little-endian float64 array of shape ``(n, len(fields))``;
            the header has ``n`` and ``schema`` (the id of the schema record that names the
            columns). A header may instead carry ``fields`` itself (the Orbbec IMU records do),
            which is simpler for low-rate streams with few columns. Both may carry an optional
            ``kind`` naming what the rows are (swerve "tick"; camera IMU "raw" / "state").

Node-specific types (e.g. the camera's ``frame`` and ``imu``) use the same envelope. A client
recording is simply the bytes of the stream written to a file; ``iter_records`` reads it back.

Pure Python + numpy.
"""
from __future__ import annotations

import json
import struct
from typing import Iterable, Iterator, Optional, Sequence

import numpy as np

_LEN = struct.Struct(">I")
MAX_HEADER = 1 << 20
ROW_DTYPE = "<f8"


def pack_record(header: dict, payload=b"") -> bytes:
    """Serialise one record. ``header["len"]`` is set from the payload."""
    payload = memoryview(payload).cast("B") if not isinstance(payload, (bytes, bytearray)) else payload
    h = dict(header)
    h["len"] = len(payload)
    hb = json.dumps(h, separators=(",", ":")).encode()
    return b"".join((_LEN.pack(len(hb)), hb, payload))


def pack_parts(header: dict, payload=b"") -> tuple[bytes, object]:
    """Like pack_record but returns (prefix, payload) to avoid copying a large payload."""
    n = memoryview(payload).nbytes
    h = dict(header)
    h["len"] = n
    hb = json.dumps(h, separators=(",", ":")).encode()
    return _LEN.pack(len(hb)) + hb, payload


class RecordReader:
    """Incremental parser: ``feed(chunk)`` returns the records completed by that chunk.

    Works on any byte source (HTTP chunks of arbitrary size, a file read in blocks).
    """

    def __init__(self):
        self._buf = bytearray()
        self._hdr: Optional[dict] = None

    def feed(self, chunk: bytes) -> list[tuple[dict, bytes]]:
        self._buf += chunk
        out = []
        while True:
            if self._hdr is None:
                if len(self._buf) < 4:
                    break
                n = _LEN.unpack_from(self._buf, 0)[0]
                if n > MAX_HEADER:
                    raise ValueError(f"header length {n} too large (stream out of sync?)")
                if len(self._buf) < 4 + n:
                    break
                self._hdr = json.loads(bytes(self._buf[4:4 + n]))
                del self._buf[:4 + n]
            need = int(self._hdr.get("len", 0))
            if len(self._buf) < need:
                break
            payload = bytes(self._buf[:need])
            del self._buf[:need]
            out.append((self._hdr, payload))
            self._hdr = None
        return out

    @property
    def pending(self) -> int:
        return len(self._buf)


def iter_records(fileobj, chunk_size: int = 1 << 16) -> Iterator[tuple[dict, bytes]]:
    """Iterate the records of a binary file (e.g. a client-side recording)."""
    r = RecordReader()
    while True:
        chunk = fileobj.read(chunk_size)
        if not chunk:
            break
        yield from r.feed(chunk)
    if r.pending:
        raise ValueError(f"truncated record at end of file ({r.pending} bytes left)")


# -- every-sample rows -------------------------------------------------------------------------

def schema_record(schema_id: int, fields: Sequence[str], units: Sequence[str], **extra) -> bytes:
    """The ``schema`` record naming the columns of later ``rows`` records."""
    if len(fields) != len(units):
        raise ValueError("fields and units must have the same length")
    return pack_record({"type": "schema", "schema": schema_id, "fields": list(fields),
                        "units": list(units), "dtype": ROW_DTYPE, **extra})


def rows_record(rows: np.ndarray, schema_id: Optional[int] = None, **extra) -> bytes:
    """A ``rows`` record for an ``(n, columns)`` array (converted to little-endian float64)."""
    a = np.ascontiguousarray(np.atleast_2d(rows), ROW_DTYPE)
    h = {"type": "rows", "n": int(a.shape[0]), **extra}
    if schema_id is not None:
        h["schema"] = schema_id
    return pack_record(h, a.tobytes())


class RowsDecoder:
    """Turns the records of a stream into structured numpy arrays.

    ``decode(header, payload)`` returns a record array with one named field per column for
    ``rows`` records (and for any record whose header carries ``fields``), and None for
    ``schema`` / ``ping`` and other types (``schema`` records are remembered).
    """

    def __init__(self):
        self.schemas: dict[int, dict] = {}

    def decode(self, header: dict, payload: bytes):
        kind = header.get("type")
        if kind == "schema":
            self.schemas[header["schema"]] = header
            return None
        fields = header.get("fields")
        if fields is None and kind == "rows":
            schema = self.schemas.get(header.get("schema"))
            if schema is None:
                raise ValueError(f"rows record refers to unknown schema {header.get('schema')!r}")
            fields = schema["fields"]
        if fields is None:
            return None
        a = np.frombuffer(payload, header.get("dtype", ROW_DTYPE)).reshape(-1, len(fields))
        return np.rec.fromarrays(a.T, names=list(fields))

    def units(self, schema_id: int) -> dict[str, str]:
        s = self.schemas[schema_id]
        return dict(zip(s["fields"], s["units"]))


def iter_rows(records: Iterable[tuple[dict, bytes]]) -> Iterator[tuple[dict, np.recarray]]:
    """(header, rows) for every rows-carrying record of a record iterator (file or stream)."""
    dec = RowsDecoder()
    for header, payload in records:
        rows = dec.decode(header, payload)
        if rows is not None:
            yield header, rows
