"""What the operator saw, as frames or an mp4: the rigid scene from the operator's eye through the AR placement,
with both controllers where they were (no HUD, no status panel, no passthrough: a plain background).

Per output frame at time ``t`` (the robot's clock, ``fps`` apart): the newest headset record at or before ``t``
(its head pose, the newest tracked one; its controllers; its placement), the newest scene pose frame at or before
``t`` (its receipt by the recorder; the headset drew the newest frame it had, a Wi-Fi hop later), and that frame's
scene. The placement is the logged one (``robot_ar`` in the records) when the log has it, else reconstructed
(``ar.Reconstruction``; the summary says which).
"""
from __future__ import annotations

import bisect
import glob
import json
import logging
import math
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterator, Optional

import numpy as np

from . import ar
from .log import Recording
from .render import DEFAULT_BG, SceneModel, ViewRenderer

log = logging.getLogger("replay")


@dataclass
class ViewOptions:
    fps: float = 30.0
    width: int = 960
    eye: str = "center"                  # center | left | right
    controllers: str = "overlay"         # overlay | depth | off
    role: Optional[str] = None           # auto | visual | collision (default: the view setting saved with the scene)
    bg: tuple = DEFAULT_BG
    start: float = 0.0                   # s after the first renderable instant (or the episode's start)
    duration: Optional[float] = None     # s (default: to the end)
    max_frames: Optional[int] = None
    ar_offset: tuple = (0.0, 0.0, 0.0)   # reconstruction only (old logs): the headset's --ar-offset / yaw / scale
    ar_yaw_deg: float = 0.0
    ar_scale: float = 1.0


@dataclass
class FrameInfo:
    index: int
    t: float
    head_t: Optional[float]
    scene_id: Optional[str]
    scene_seq: Optional[int]
    placement: Optional[str]             # logged | reconstructed | None (not placed: background only)
    eye_local: Optional[tuple]           # (p, R) in LOCAL
    camera_sim: Optional[tuple]          # (p, R) in the sim world
    hands: dict = field(default_factory=dict)


def placements(rec: Recording, opts: ViewOptions) -> tuple[list, str]:
    """The placement in force at each headset sample, and where they come from (logged | reconstructed)."""
    if rec.has_logged_placement:
        return [ar.logged_placement({"robot_ar": h.robot_ar}) if h.robot_ar is not None else None
                for h in rec.headset], "logged"
    rc = ar.Reconstruction(opts.ar_offset, math.radians(opts.ar_yaw_deg), opts.ar_scale)
    out = []
    for h in rec.headset:
        f = rec.scene_frame_at(h.t)
        frame = {"session_id": h.session, "head": None if h.head is None else {
            "position_tracked": True, "orientation_tracked": True, "pose": ar.matrix16(*h.head)}}
        out.append(rc.update(frame, None if f is None else f.reset_seq))
    return out, "reconstructed"


def views(rec: Recording) -> dict:
    for h in rec.headset:
        if h.robot_ar is not None:
            v = ar.logged_views({"robot_ar": h.robot_ar})
            if v is not None:
                return v
    return ar.default_views()


def image_size(eye: ar.Eye, width: int) -> tuple[int, int]:
    """Even width and height with the eye's aspect (yuv420p needs even sizes)."""
    tl, tr, tu, td = eye.tangents()
    w = int(width) // 2 * 2
    h = int(round(w * (tu - td) / (tr - tl) / 2)) * 2
    return w, max(h, 2)


class OperatorView:
    """The frames of one recording: ``summary`` (placement source, eye, size, time span) is known up front;
    ``frames()`` yields (image, info) per output frame."""

    def __init__(self, rec: Recording, opts: ViewOptions = ViewOptions()):
        if not rec.headset:
            raise ValueError(f"{rec.path}: no headset frames in the operator stream")
        self.rec, self.opts = rec, opts
        self.placements, self.source = placements(rec, opts)
        self.eye = ar.make_eye(views(rec), opts.eye)
        self.width, self.height = image_size(self.eye, opts.width)
        self.last_head: list = []
        cur = None
        for h in rec.headset:                    # the newest tracked head at or before each sample
            cur = h.head if h.head is not None else cur
            self.last_head.append(cur)
        ready = [i for i, h in enumerate(rec.headset) if self.last_head[i] is not None and self.placements[i] is not None]
        if not ready:
            raise ValueError(f"{rec.path}: no headset sample with a tracked head and a placement")
        t_first = rec.headset[ready[0]].t
        if rec.scene_t:
            t_first = max(t_first, rec.scene_t[0])
        t0 = max(t_first, rec.window[0]) if rec.window else t_first
        self.t0 = t0 + opts.start
        t_end = rec.window[1] if rec.window else rec.headset[-1].t
        if opts.duration is not None:
            t_end = min(t_end, self.t0 + opts.duration)
        n = max(0, int(math.floor((t_end - self.t0) * opts.fps + 1e-6)) + 1)
        self.n = min(n, opts.max_frames) if opts.max_frames is not None else n
        self._hts = [h.t for h in rec.headset]

    @property
    def summary(self) -> dict:
        placements_n = max((p.placements for p in self.placements if p is not None), default=0)
        return {"placement": self.source, "placements": placements_n, "eye": self.opts.eye,
                "fov_source": self.eye.source, "fov_rad": [round(a, 4) for a in self.eye.fov], "width": self.width,
                "height": self.height, "t0": self.t0, "frames_planned": self.n}

    def frames(self) -> Iterator[tuple[np.ndarray, FrameInfo]]:
        rec, opts, eye = self.rec, self.opts, self.eye
        renderer = ViewRenderer(self.width, self.height, opts.controllers)
        models: dict = {}
        blank = np.empty((self.height, self.width, 3), np.uint8)
        blank[:] = (np.asarray(opts.bg) * 255).round().astype(np.uint8)
        for k in range(self.n):
            t = self.t0 + k / opts.fps
            i = bisect.bisect_right(self._hts, t) - 1
            h = rec.headset[i] if i >= 0 else None
            head = self.last_head[i] if i >= 0 else None
            pl = self.placements[i] if i >= 0 else None
            frame = rec.scene_frame_at(t)
            info = FrameInfo(k, t, h.t if h else None, frame.scene_id if frame else None,
                             frame.seq if frame else None, pl.source if pl else None, None, None,
                             {s: (h.hands.get(s) is not None) for s in ("left", "right")} if h else {})
            scene = rec.scene(frame.scene_id) if frame is not None else None
            if head is None or pl is None or scene is None:
                yield blank.copy(), info
                continue
            role = opts.role or rec.view(frame.scene_id).get("role") or "auto"
            key = (frame.scene_id, round(pl.scale, 6), role)
            model = models.get(key)
            if model is None:
                models.clear()
                model = models[key] = SceneModel(scene[0], scene[1], role=role, scale=pl.scale, bg=opts.bg,
                                                 width=self.width, height=self.height)
                if model.skipped:
                    log.warning("scene %s: %d geoms not drawable", frame.scene_id, model.skipped)
            el = ar.eye_in_local(head, eye)
            info.eye_local, info.camera_sim = el, pl.camera(*el)
            yield renderer.render(model, frame, pl, el, eye, h.hands), info


def find_ffmpeg(explicit: Optional[str] = None) -> str:
    """An ffmpeg executable: ``explicit``, ``$IMAGEIO_FFMPEG_EXE``, ``imageio_ffmpeg``'s binary, ``ffmpeg`` on the
    PATH, or the binary of an ``imageio_ffmpeg`` in the conda base env next to this env (never installed here)."""
    cands = [explicit, os.environ.get("IMAGEIO_FFMPEG_EXE")]
    try:
        import imageio_ffmpeg     # noqa: PLC0415
        cands.append(imageio_ffmpeg.get_ffmpeg_exe())
    except Exception:     # noqa: BLE001
        pass
    cands.append(shutil.which("ffmpeg"))
    prefix = Path(sys.prefix)
    for base in (prefix, prefix.parent.parent):          # this env, then <conda>/ from <conda>/envs/<name>
        cands += sorted(glob.glob(str(base / "lib" / "python3*" / "site-packages" / "imageio_ffmpeg" / "binaries"
                                      / "ffmpeg-*")))
    for c in cands:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    raise RuntimeError("no ffmpeg found: pass --ffmpeg PATH or set IMAGEIO_FFMPEG_EXE")


def render_video(rec: Recording, out: Path, opts: ViewOptions = ViewOptions(), ffmpeg: Optional[str] = None,
                 crf: int = 20, progress: Optional[Callable[[int, int], None]] = None) -> dict:
    """Write the mp4 (H.264, yuv420p) and ``<out>.json`` (the summary and each frame's camera); returns the summary."""
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    exe = find_ffmpeg(ffmpeg)
    proc = None
    frames = []
    t_wall = time.monotonic()
    n = 0
    try:
        view = OperatorView(rec, opts)
        for img, info in view.frames():
            if proc is None:
                h, w = img.shape[:2]
                proc = subprocess.Popen([exe, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                         "-s", f"{w}x{h}", "-r", f"{opts.fps:g}", "-i", "-", "-an", "-c:v", "libx264",
                                         "-pix_fmt", "yuv420p", "-crf", str(crf), "-movflags", "+faststart",
                                         str(out)], stdin=subprocess.PIPE)
            proc.stdin.write(np.ascontiguousarray(img).tobytes())
            n += 1
            frames.append({"i": info.index, "t": round(info.t, 4), "head_t": info.head_t, "scene_id": info.scene_id,
                           "scene_seq": info.scene_seq, "placement": info.placement, "hands": info.hands,
                           "eye_local": None if info.eye_local is None else ar.matrix16(*info.eye_local),
                           "camera_sim": None if info.camera_sim is None else ar.matrix16(*info.camera_sim)})
            if progress is not None:
                progress(n, info.index)
    finally:
        if proc is not None:
            proc.stdin.close()
            rc = proc.wait()
            if rc != 0:
                raise RuntimeError(f"ffmpeg exited with {rc}")
    wall = time.monotonic() - t_wall
    summary = {**view.summary, "source": str(rec.path), "label": rec.label, "out": str(out),
               "frames": n, "fps": opts.fps, "duration_s": round(n / opts.fps, 3), "render_s": round(wall, 2),
               "render_fps": round(n / wall, 1) if wall > 0 else None,
               "blank_frames": sum(f["camera_sim"] is None for f in frames),
               "scenes": sorted({f["scene_id"] for f in frames if f["scene_id"]}), "ffmpeg": exe}
    Path(str(out) + ".json").write_text(json.dumps({"summary": summary, "frames": frames}))
    return summary
