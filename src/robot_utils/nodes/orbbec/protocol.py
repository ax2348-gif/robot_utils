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
import zlib

import numpy as np

# The record envelope is shared by every node (robot_utils.nodes.common.protocol);
# re-exported here so existing imports keep working.
from ..common.protocol import (MAX_HEADER, RecordReader, iter_records,  # noqa: F401
                               pack_parts, pack_record)

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
# (unit, meaning) per column, served by GET /imu/fields?raw=true (the comments above, as data)
IMU_FIELD_INFO = {
    "seq": ("count", "per-sample counter on the server (gaps = samples the SDK dropped)"),
    "t_capture": ("s", "server monotonic time the SDK received the sample on the host (system timestamp)"),
    "t_sensor": ("s", "server monotonic time of the sample: device timestamp mapped to the host clock by the SDK "
                      "(global timestamp); NaN if not available. Use this one for timing"),
    "t_device_us": ("us", "raw device timestamp, camera clock (not synchronised to the host)"),
    **{f"a{k}": ("m/s^2", f"accelerometer {k}, camera IMU frame (specific force: reads +g up when still)")
       for k in "xyz"},
    **{f"g{k}": ("rad/s", f"gyroscope {k}, camera IMU frame (raw: not bias-corrected)") for k in "xyz"},
    "temperature": ("degC", "IMU temperature"),
}
assert tuple(IMU_FIELD_INFO) == IMU_FIELDS

# numpy dtype of raw payloads by pixel format
RAW_DTYPES = {"Y16": "<u2", "Y8": "|u1", "RGB": "|u1", "BGR": "|u1", "RGBA": "|u1", "BGRA": "|u1",
              "YUYV": "|u1", "UYVY": "|u1"}
CHANNELS = {"Y16": 1, "Y8": 1, "RGB": 3, "BGR": 3, "RGBA": 4, "BGRA": 4, "YUYV": 2, "UYVY": 2}


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
