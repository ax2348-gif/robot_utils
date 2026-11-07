"""Frame encoders (run in the encode thread pool, never on the event loop or the SDK thread).

Monitoring: depth -> colormapped JPEG, IR -> gray JPEG, color MJPG -> passed through.
Raw: depth Y16 / IR Y8|Y16 bytes as captured (optionally zlib), color -> the camera's MJPG bytes.
"""
from __future__ import annotations

import io
import zlib

import numpy as np

try:
    import cv2
except ImportError:  # Pillow fallback: fewer colormaps, slower JPEG
    cv2 = None

from .protocol import RAW_DTYPES


def _jet(x):
    return np.stack([np.clip(1.5 - abs(4 * x - k), 0, 1) for k in (3, 2, 1)], -1)


def _turbo(x):   # polynomial approximation (Mikhailov, Google), used when cv2 is missing
    r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))))
    g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))))
    b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))))
    return np.clip(np.stack([r, g, b], -1), 0, 1)


def _build_colormaps():
    x = np.linspace(0, 1, 256)
    maps = {"turbo": _turbo(x), "jet": _jet(x), "gray": np.stack([x, x, x], -1)}
    maps = {k: (v * 255 + 0.5).astype(np.uint8) for k, v in maps.items()}
    if cv2 is not None:
        ramp = np.arange(256, dtype=np.uint8).reshape(256, 1)
        for name in ("turbo", "jet", "inferno", "viridis", "magma", "plasma", "bone", "hot", "rainbow"):
            cm = getattr(cv2, "COLORMAP_" + name.upper(), None)
            if cm is not None:
                maps[name] = cv2.applyColorMap(ramp, cm)[:, 0, ::-1].copy()   # BGR -> RGB
    return maps


COLORMAPS = _build_colormaps()


def colormap_stops():
    return {k: ["#%02x%02x%02x" % tuple(v[i]) for i in range(0, 256, 32)] + ["#%02x%02x%02x" % tuple(v[255])]
            for k, v in COLORMAPS.items()}


def encode_jpeg(img: np.ndarray, quality: int) -> bytes:
    """img: HxW gray or HxWx3 RGB uint8."""
    if cv2 is not None:
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return buf.tobytes()
    from PIL import Image
    out = io.BytesIO()
    Image.fromarray(img).save(out, "JPEG", quality=int(quality))
    return out.getvalue()


def stretch16(a: np.ndarray) -> np.ndarray:
    """uint16 image -> uint8 using its 1..99th percentile (sampled)."""
    s = a[::4, ::4]
    lo, hi = np.percentile(s, [1, 99]) if s.size else (0, 1)
    hi = max(hi, lo + 1)
    return np.clip((a.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)


def colorize_depth(raw: np.ndarray, scale: float, ds) -> tuple[np.ndarray, float, float, float]:
    """raw: HxW uint16 depth units; ds: DisplaySettings. Returns (RGB, lo_mm, hi_mm, valid fraction)."""
    valid = raw > 0
    mm = raw.astype(np.float32)
    if scale != 1.0:
        mm *= scale
    if ds.range_mode == "manual":
        lo, hi = float(ds.min_mm), float(ds.max_mm)
    else:
        sample = mm[::4, ::4][valid[::4, ::4]]
        lo, hi = (np.percentile(sample, [ds.pct_lo, ds.pct_hi]) if sample.size > 50 else (0.0, 1.0))
        lo, hi = float(lo), float(hi)
    hi = max(hi, lo + 1.0)
    idx = np.clip((mm - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)
    if ds.invert:
        idx = 255 - idx
    rgb = COLORMAPS.get(ds.colormap, COLORMAPS["gray"])[idx]
    c = ds.invalid_color.lstrip("#")
    rgb[~valid] = (int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16))
    return rgb, lo, hi, float(valid.mean())


def to_display(fmt: str, data: np.ndarray, w: int, h: int) -> np.ndarray:
    """Non-MJPG color/IR frame -> gray or RGB uint8 for JPEG."""
    if fmt == "Y8":
        return data.reshape(h, w)
    if fmt == "Y16":
        return stretch16(data.view(np.uint16).reshape(h, w))
    if fmt in ("RGB", "BGR"):
        img = data.reshape(h, w, 3)
        return img if fmt == "RGB" else np.ascontiguousarray(img[..., ::-1])
    if fmt in ("RGBA", "BGRA"):
        img = data.reshape(h, w, 4)
        return np.ascontiguousarray(img[..., :3] if fmt == "RGBA" else img[..., 2::-1])
    if fmt in ("YUYV", "UYVY") and cv2 is not None:
        code = cv2.COLOR_YUV2RGB_YUYV if fmt == "YUYV" else cv2.COLOR_YUV2RGB_UYVY
        return cv2.cvtColor(data.reshape(h, w, 2), code)
    raise ValueError(f"unsupported format {fmt}")


def encode_frame(frame, mode: str, display, compress: bool = False) -> tuple[object, dict]:
    """Encode a published Frame for delivery. Returns (payload bytes-like, extra header fields).

    mode "jpeg": monitoring JPEG (color MJPG passed through).
    mode "raw":  lossless: Y16/Y8 bytes (little endian), color MJPG bytes as captured.
    """
    if frame.fmt == "MJPG":     # the camera's own JPEG, both modes
        return frame.data, {"encoding": "jpeg", "format": "MJPG"}
    if mode == "raw":
        payload = memoryview(frame.data).cast("B")
        extra = {"encoding": "raw", "format": frame.fmt, "dtype": RAW_DTYPES.get(frame.fmt, "|u1")}
        if compress:
            payload = zlib.compress(payload, 1)
            extra["compression"] = "zlib"
        return payload, extra
    if frame.stream == "depth":
        rgb, lo, hi, valid = colorize_depth(frame.data, frame.depth_scale, display)
        return encode_jpeg(rgb, display.jpeg_quality), {
            "encoding": "jpeg", "format": "JPEG", "lo_mm": round(lo, 1), "hi_mm": round(hi, 1),
            "valid": round(valid, 3)}
    img = to_display(frame.fmt, frame.data, frame.w, frame.h)
    return encode_jpeg(img, display.jpeg_quality), {"encoding": "jpeg", "format": "JPEG"}
