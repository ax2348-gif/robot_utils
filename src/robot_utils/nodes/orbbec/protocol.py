"""Binary record framing used by the streaming endpoints (and by client-side recordings).

A stream is a sequence of records; each record is::

    [u32 big-endian header length N][N bytes of UTF-8 JSON header][header["len"] bytes of payload]

Header ``type`` is one of:

``frame``  one camera frame (video). Payload depends on ``encoding``:
           ``jpeg`` (monitoring JPEG, or the camera's own MJPEG bytes for color),
           ``raw`` (pixel bytes, row major, ``dtype`` gives numpy dtype incl. byte order,
           e.g. ``"<u2"`` for depth Y16), optionally ``compression: "zlib"`` on top.
``imu``    a batch of IMU samples. Payload: little-endian float64 array of shape
           ``(n, len(fields))``; ``fields`` names the columns (see IMU_FIELDS).
``ping``   keep-alive (no payload), sent when nothing else was sent for ~1 s.

Pure Python + numpy; the decoders for JPEG use OpenCV if installed, else Pillow.
"""
from __future__ import annotations

import io
import json
import struct
import zlib
from typing import Iterator, Optional

import numpy as np

_LEN = struct.Struct(">I")
MAX_HEADER = 1 << 20

# Columns of an IMU record payload (float64 each).
IMU_FIELDS = (
    "seq",          # per-sample counter on the server (gaps = samples the SDK dropped)
    "t_capture",    # server monotonic s: SDK host receipt time (system timestamp)
    "t_sensor",     # server monotonic s: device timestamp mapped to host by the SDK (global ts); NaN if n/a
    "t_device_us",  # raw device timestamp (us, camera clock)
    "ax", "ay", "az",   # accelerometer, m/s^2 (camera IMU frame)
    "gx", "gy", "gz",   # gyroscope, rad/s
    "temperature",      # IMU temperature, deg C
)

# numpy dtype of raw payloads by pixel format
RAW_DTYPES = {"Y16": "<u2", "Y8": "|u1", "RGB": "|u1", "BGR": "|u1", "RGBA": "|u1", "BGRA": "|u1",
              "YUYV": "|u1", "UYVY": "|u1"}
CHANNELS = {"Y16": 1, "Y8": 1, "RGB": 3, "BGR": 3, "RGBA": 4, "BGRA": 4, "YUYV": 2, "UYVY": 2}


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
        self._need_hdr: Optional[int] = None
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


# -- payload decoding --------------------------------------------------------------------------

def decode_jpeg(data: bytes) -> np.ndarray:
    """JPEG -> HxWx3 RGB uint8 (or HxW for grayscale JPEGs)."""
    try:
        import cv2
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise ValueError("JPEG decode failed")
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if img.ndim == 3 else img
    except ImportError:
        from PIL import Image
        return np.asarray(Image.open(io.BytesIO(data)))


def decode_payload(header: dict, payload: bytes, decode_jpeg_images: bool = True):
    """Turn a record payload into a numpy array.

    frame/raw  -> HxW (Y16 depth: uint16 in units of ``depth_scale`` mm; Y8/Y16 IR) or HxWxC
    frame/jpeg -> decoded RGB array if decode_jpeg_images, else the JPEG bytes unchanged
    imu        -> structured array with fields IMU_FIELDS (float64)
    ping       -> None
    """
    kind = header.get("type")
    if kind == "ping":
        return None
    if header.get("compression") == "zlib":
        payload = zlib.decompress(payload)
    if kind == "imu":
        fields = header.get("fields", IMU_FIELDS)
        a = np.frombuffer(payload, "<f8").reshape(-1, len(fields))
        return np.rec.fromarrays(a.T, names=list(fields))
    if kind != "frame":
        raise ValueError(f"unknown record type {kind!r}")
    if header["encoding"] == "jpeg":
        return decode_jpeg(payload) if decode_jpeg_images else payload
    if header["encoding"] != "raw":
        raise ValueError(f"unknown encoding {header['encoding']!r}")
    fmt, w, h = header["format"], header["width"], header["height"]
    dtype = np.dtype(header.get("dtype") or RAW_DTYPES[fmt])
    a = np.frombuffer(payload, dtype)
    c = CHANNELS.get(fmt, 1)
    if a.size != w * h * c:
        raise ValueError(f"{fmt} payload has {a.size} elements, expected {w}x{h}x{c}")
    return a.reshape(h, w) if c == 1 else a.reshape(h, w, c)


def imu_payload(samples: np.ndarray) -> bytes:
    """(n, len(IMU_FIELDS)) float64 -> payload bytes."""
    return np.ascontiguousarray(samples, "<f8").tobytes()
