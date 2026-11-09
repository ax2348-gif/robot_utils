"""Procedural frames for a covered camera (common/locks.py ``cover``): no image is captured.

Each enabled stream gets frames of its configured size and rate, flagged ``covered``:
- colour: a slowly drifting soft pattern with a banner ("camera covered", reason, who, since when),
  re-rendered at 2 Hz and re-sent at the stream's rate;
- depth: a synthetic scene (a floor sloping away under a far wall) in millimetres, slowly breathing;
- IR: a gray pattern.
So every endpoint, page and consumer (3d_perception, agent_nav) keeps working on frames that are
clearly marked as not real.
"""
from __future__ import annotations

import math
import re
import time
from typing import Optional

import numpy as np

from .encode import encode_jpeg
from .hub import Frame

RENDER_HZ = 2.0


def profile_size(key: Optional[str], default=(640, 480, 10)) -> tuple[int, int, float]:
    m = re.match(r"(\d+)x(\d+)@(\d+)", key or "")
    return (int(m.group(1)), int(m.group(2)), float(m.group(3))) if m else default


class CoverSource:
    def __init__(self):
        self._next: dict[str, float] = {}
        self._jpeg: Optional[bytes] = None
        self._jpeg_key = None
        self._rendered = 0.0
        self._index = 0

    def reset(self):
        self._next.clear()
        self._jpeg = None

    def frames(self, settings, lock_text: str, now: Optional[float] = None) -> list[Frame]:
        """Frames due now for the enabled streams (empty if none is due)."""
        now = time.monotonic() if now is None else now
        out = []
        streams = [n for n, on in (settings.streams or {}).items() if on]
        for n in streams:
            if n == "color":
                w, h, fps = profile_size(settings.color_profile, (1280, 720, 10))
            else:
                w, h, fps = profile_size(settings.stereo_mode, (848, 480, 10))
            due = self._next.get(n)
            if due is None:
                self._next[n] = now
                due = now
            if now < due:
                continue
            self._next[n] = max(due + 1.0 / max(fps, 1.0), now - 0.5)
            self._index += 1
            out.append(self._frame(n, w, h, lock_text, now))
        return out

    def next_due(self, now: Optional[float] = None) -> float:
        now = time.monotonic() if now is None else now
        return min(self._next.values()) - now if self._next else 0.05

    def _frame(self, stream: str, w: int, h: int, text: str, now: float) -> Frame:
        common = dict(t_capture=now, t_sensor=now, t_device_us=int(now * 1e6), index=self._index,
                      t_owner=now, covered=True)
        if stream == "color":
            return Frame(stream="color", fmt="MJPG", w=w, h=h, data=self._color(w, h, text, now), **common)
        if stream == "depth":
            return Frame(stream="depth", fmt="Y16", w=w, h=h, data=self._depth(w, h, now), depth_scale=1.0, **common)
        return Frame(stream=stream, fmt="Y8", w=w, h=h, data=self._ir(w, h, now).ravel(), **common)

    def _color(self, w: int, h: int, text: str, now: float) -> bytes:
        key = (w, h, text)
        if self._jpeg is not None and key == self._jpeg_key and now - self._rendered < 1.0 / RENDER_HZ:
            return self._jpeg
        sw, sh = max(32, w // 8), max(18, h // 8)
        y, x = np.mgrid[0:sh, 0:sw].astype(np.float32)
        t = now * 0.15
        r = 0.5 + 0.5 * np.sin(x / sw * 4.0 + t) * np.cos(y / sh * 3.0 - t * 0.7)
        g = 0.5 + 0.5 * np.sin((x + y) / (sw + sh) * 5.0 - t * 1.3)
        b = 0.5 + 0.5 * np.cos(np.hypot(x - sw / 2, y - sh / 2) / sw * 6.0 + t)
        small = (np.stack([r * 0.35 + 0.1, g * 0.3 + 0.12, b * 0.45 + 0.2], -1) * 255).astype(np.uint8)
        img = np.repeat(np.repeat(small, 8, 0), 8, 1)[:h, :w]
        if img.shape[0] < h or img.shape[1] < w:
            img = np.pad(img, ((0, h - img.shape[0]), (0, w - img.shape[1]), (0, 0)), mode="edge")
        img = self._banner(img, text)
        self._jpeg = encode_jpeg(img, 70)
        self._jpeg_key, self._rendered = key, now
        return self._jpeg

    @staticmethod
    def _banner(img: np.ndarray, text: str) -> np.ndarray:
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError:
            return img
        im = Image.fromarray(img)
        dr = ImageDraw.Draw(im, "RGBA")
        W, H = im.size
        try:
            big = ImageFont.load_default(size=max(18, H // 14))
            small = ImageFont.load_default(size=max(12, H // 32))
        except TypeError:
            big = small = ImageFont.load_default()
        dr.rectangle([0, H * 0.36, W, H * 0.64], fill=(0, 0, 0, 150))
        dr.text((W / 2, H * 0.44), "CAMERA COVERED", fill=(255, 255, 255), font=big, anchor="mm")
        dr.text((W / 2, H * 0.56), text[:120], fill=(220, 220, 220), font=small, anchor="mm")
        return np.asarray(im)

    @staticmethod
    def _depth(w: int, h: int, now: float) -> np.ndarray:
        v = np.linspace(0, 1, h, dtype=np.float32)[:, None]
        u = np.linspace(-1, 1, w, dtype=np.float32)[None, :]
        floor = 800.0 / np.maximum(v - 0.35, 0.02)               # a floor receding toward the horizon
        wall = 3500.0 + 300.0 * math.sin(now * 0.3) + 200.0 * u   # a far wall, slowly breathing
        d = np.minimum(np.where(v > 0.38, floor, wall), wall)
        return np.clip(d, 0, 65535).astype(np.uint16)

    @staticmethod
    def _ir(w: int, h: int, now: float) -> np.ndarray:
        y, x = np.mgrid[0:h, 0:w]
        return (96 + 48 * np.sin((x + y) / 37.0 + now * 0.5)).astype(np.uint8)
