"""Replay video export: frames in (JPEGs), an H.264 / HEVC MP4 out, through an ``ffmpeg`` binary.

Two sources of frames, one encoder path:

- the page renders the replay (cameras, 3D views, plots, its layout) frame by frame at the export's own clock
  and posts each frame as a JPEG (``ExportJob.add_frame``): what you see is what you get, at any replay rate;
- ``camera_frames`` takes one recorded camera's JPEGs as they are (no re-render; the fast path, also the CLI's).

Frames are spooled to a temporary MJPEG file (concatenated JPEGs), then encoded once (quality mode, CRF) or
twice (size mode: two-pass at ``bitrate = 8 * target_bytes / duration``, less a container allowance), so a
target size is met within a few percent. ``estimate`` predicts the size before anything is rendered (a
bits-per-pixel model per codec and quality); ``probe`` measures it by encoding a short sample of the real frames
with the same settings (the page renders a few seconds from across the range).

ffmpeg: ``--ffmpeg``, ``$FFMPEG``, ``ffmpeg`` on PATH, or the binary the ``imageio-ffmpeg`` package bundles (in
the running environment or a conda environment on this machine); none found: export is off, the rest works.
"""
from __future__ import annotations

import glob
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Optional

CONTAINER_OVERHEAD = 0.015        # MP4 boxes + faststart: about 1.5 % of a video-only file
MIN_BPP = 0.01                    # bits per pixel per frame below which a replay turns to blocks

PRESETS = {
    "draft": {"label": "Draft: fastest encode, larger file", "codec": "h264", "speed": "ultrafast", "crf": 28,
              "hw": True},
    "balanced": {"label": "Balanced", "codec": "h264", "speed": "veryfast", "crf": 23},
    "quality": {"label": "Quality: slow encode, sharp", "codec": "h264", "speed": "slow", "crf": 18},
    "small": {"label": "Small: HEVC (Apple / modern players)", "codec": "hevc", "speed": "medium", "crf": 28},
}
# bits per pixel per frame at CRF 23 for replay pages (mostly flat panels, lines, two camera tiles); halves
# every +6 CRF. Measured on the sim sample (docs/browse.md "Export"): a starting guess, ``probe`` refines it.
BPP_AT_23 = {"h264": 0.055, "hevc": 0.035}
SPEED_FACTOR = {"ultrafast": 1.6, "superfast": 1.4, "veryfast": 1.15, "faster": 1.05, "fast": 1.0, "medium": 1.0,
                "slow": 0.93, "slower": 0.9, "veryslow": 0.88}


def find_ffmpeg(explicit: Optional[str] = None) -> Optional[str]:
    cands = [explicit, os.environ.get("FFMPEG"), shutil.which("ffmpeg")]
    try:
        import imageio_ffmpeg  # type: ignore
        cands.append(imageio_ffmpeg.get_ffmpeg_exe())
    except Exception:  # noqa: BLE001
        pass
    home = Path.home()
    for root in (home / "miniconda3", home / "anaconda3", home / "miniforge3", Path("/opt/conda")):
        cands += sorted(glob.glob(str(root / "lib/python3*/site-packages/imageio_ffmpeg/binaries/ffmpeg-*")))
        cands += sorted(glob.glob(str(root / "envs/*/lib/python3*/site-packages/imageio_ffmpeg/binaries/ffmpeg-*")))
    cands += ["/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg"]
    for c in cands:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def ffmpeg_encoders(ffmpeg: str) -> set[str]:
    try:
        out = subprocess.run([ffmpeg, "-hide_banner", "-encoders"], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return set()
    return {m.group(1) for m in re.finditer(r"^\s*V\S*\s+(\S+)", out, re.M)}


@dataclass
class Settings:
    """One export's encode settings (the page's form; ``preset`` fills what is not given)."""
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    preset: str = "balanced"
    codec: Optional[str] = None          # h264 | hevc
    speed: Optional[str] = None          # x264 / x265 preset
    crf: Optional[float] = None
    target_mb: Optional[float] = None    # size mode when set: MB (1e6 bytes)
    two_pass: bool = True                # size mode: two passes (exact size) or one (faster, looser)
    hw: Optional[bool] = None            # VideoToolbox when available (fast, no CRF: quality mapped to -q:v)

    def resolved(self) -> "Settings":
        p = PRESETS.get(self.preset, PRESETS["balanced"])
        s = Settings(**asdict(self))
        s.codec = s.codec or p["codec"]
        s.speed = s.speed or p["speed"]
        s.crf = p["crf"] if s.crf is None else s.crf
        s.hw = p.get("hw", False) if s.hw is None else s.hw
        s.width, s.height = int(s.width) // 2 * 2, int(s.height) // 2 * 2
        return s


def estimate(s: Settings, duration_s: float, bpp: Optional[float] = None) -> dict:
    """Predicted size of a ``duration_s`` video (output time) with ``s``; ``bpp`` measured overrides the model."""
    s = s.resolved()
    px = s.width * s.height * s.fps
    if s.target_mb:
        size = s.target_mb * 1e6
        rate = 8 * size * (1 - CONTAINER_OVERHEAD) / max(duration_s, 1e-6)
        out = {"bytes": size, "bitrate": rate, "bpp": rate / px, "how": "target"}
    else:
        b = bpp if bpp is not None else (BPP_AT_23[s.codec] * 2 ** (-(s.crf - 23) / 6.0)
                                         * SPEED_FACTOR.get(s.speed, 1.0) * (1.25 if s.hw else 1.0))
        rate = b * px
        out = {"bytes": rate * duration_s / 8 * (1 + CONTAINER_OVERHEAD), "bitrate": rate, "bpp": b,
               "how": "measured" if bpp is not None else "model"}
    out["duration"] = duration_s
    out["warning"] = ("below %.3f bits per pixel: expect blocks; lower the resolution or the fps" % MIN_BPP
                      if out["bpp"] < MIN_BPP else None)
    return out


def video_args(s: Settings, encoders: set[str], bitrate: Optional[float] = None, pass_no: Optional[int] = None,
               passlog: Optional[str] = None) -> list[str]:
    s = s.resolved()
    hw = f"{s.codec}_videotoolbox"
    if s.hw and hw in encoders and pass_no is None:
        a = ["-c:v", hw]
        a += ["-b:v", str(int(bitrate))] if bitrate else ["-q:v", str(int(max(1, min(100, 100 - 2.2 * s.crf))))]
        if s.codec == "hevc":
            a += ["-tag:v", "hvc1"]
        return a
    if s.codec == "hevc":
        lib = "libx265"
        a = ["-c:v", lib, "-preset", s.speed, "-tag:v", "hvc1"]
        params = ["log-level=error"]
        if bitrate:
            a += ["-b:v", str(int(bitrate))]
            if pass_no:
                params += [f"pass={pass_no}", f"stats={passlog}"]
        else:
            a += ["-crf", f"{s.crf:g}"]
        return a + ["-x265-params", ":".join(params)]
    a = ["-c:v", "libx264", "-preset", s.speed]
    if bitrate:
        a += ["-b:v", str(int(bitrate))]
        if pass_no:
            a += ["-pass", str(pass_no), "-passlogfile", passlog]
        else:
            a += ["-maxrate", str(int(bitrate * 1.5)), "-bufsize", str(int(bitrate * 2))]
    else:
        a += ["-crf", f"{s.crf:g}"]
    return a


def encode(ffmpeg: str, spool: Path, n_frames: int, s: Settings, out: Path, encoders: set[str],
           progress: Optional[Callable[[float, str], None]] = None, title: Optional[str] = None) -> dict:
    """Encode the spooled JPEGs (``n_frames`` at ``s.fps``) to ``out``; returns ``{bytes, seconds, passes, ...}``."""
    s = s.resolved()
    duration = n_frames / s.fps
    inp = ["-hide_banner", "-loglevel", "error", "-y", "-f", "image2pipe", "-c:v", "mjpeg", "-framerate",
           f"{s.fps:g}", "-i", str(spool)]
    vf = ["-vf", f"scale={s.width}:{s.height}:flags=bicubic,format=yuv420p", "-r", f"{s.fps:g}"]
    meta = ["-metadata", f"title={title}"] if title else []
    t0 = time.monotonic()
    bitrate = estimate(s, duration)["bitrate"] if s.target_mb else None
    passes = []
    with tempfile.TemporaryDirectory(prefix="ru-export-") as td:
        passlog = os.path.join(td, "pass")
        if bitrate and s.two_pass:
            passes = [(1, ["-an", "-f", "mp4", os.devnull]), (2, ["-movflags", "+faststart", str(out)])]
        else:
            passes = [(None, ["-movflags", "+faststart", str(out)])]
        for k, (pn, tail) in enumerate(passes):
            cmd = [ffmpeg, *inp, *vf, *video_args(s, encoders, bitrate, pn, passlog), *meta,
                   "-progress", "pipe:1", "-nostats", *tail]
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, cwd=td)
            for line in p.stdout:
                if line.startswith("frame=") and progress:
                    try:
                        f = int(line.split("=", 1)[1])
                    except ValueError:
                        continue
                    progress((k + min(1.0, f / max(1, n_frames))) / len(passes),
                             f"pass {k + 1}/{len(passes)}" if len(passes) > 1 else "encoding")
            err = p.stderr.read()
            if p.wait() != 0:
                raise RuntimeError(f"ffmpeg failed ({p.returncode}): {err.strip()[-800:]}")
    size = out.stat().st_size
    res = {"bytes": size, "seconds": time.monotonic() - t0, "passes": len(passes), "duration": duration,
           "bitrate": 8 * size / max(duration, 1e-6), "bpp": 8 * size / max(duration, 1e-6) / (s.width * s.height * s.fps)}
    if s.target_mb:
        res["target_bytes"] = s.target_mb * 1e6
        res["target_error"] = size / (s.target_mb * 1e6) - 1
    return res


@dataclass
class ExportJob:
    id: str
    settings: Settings
    out: Path
    n_expected: int
    probe: bool = False
    title: Optional[str] = None
    state: str = "receiving"           # receiving | encoding | done | error | cancelled
    frames: int = 0
    progress: float = 0.0
    phase: str = ""
    result: Optional[dict] = None
    error: Optional[str] = None
    created: float = field(default_factory=time.time)
    spool: Path = field(default=None)
    _f: object = field(default=None, repr=False)

    def public(self) -> dict:
        return {"id": self.id, "state": self.state, "frames": self.frames, "expected": self.n_expected,
                "progress": round(self.progress, 4), "phase": self.phase, "result": self.result, "error": self.error,
                "out": str(self.out) if not self.probe else None, "probe": self.probe,
                "settings": asdict(self.settings.resolved())}


class Exporter:
    """The export jobs of one browser server."""

    def __init__(self, ffmpeg: Optional[str], out_dir: Path):
        self.ffmpeg = ffmpeg
        self.encoders = ffmpeg_encoders(ffmpeg) if ffmpeg else set()
        self.out_dir = Path(out_dir)
        self.jobs: dict[str, ExportJob] = {}
        self._lock = threading.Lock()
        self._tmp = Path(tempfile.mkdtemp(prefix="ru-browse-spool-"))

    def info(self) -> dict:
        return {"ffmpeg": self.ffmpeg, "available": bool(self.ffmpeg),
                "codecs": [c for c, e in (("h264", "libx264"), ("hevc", "libx265")) if e in self.encoders],
                "hw": sorted(e for e in self.encoders if e.endswith("_videotoolbox")),
                "presets": PRESETS, "out_dir": str(self.out_dir)}

    def create(self, s: Settings, n_expected: int, name: str, probe: bool = False, title: Optional[str] = None) -> ExportJob:
        if not self.ffmpeg:
            raise RuntimeError("no ffmpeg binary found (see docs/browse.md 'Export')")
        jid = uuid.uuid4().hex[:10]
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "replay"
        out = (self._tmp / f"probe-{jid}.mp4") if probe else (self.out_dir / f"{safe}.mp4")
        if not probe:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            k = 2
            while out.exists():
                out = self.out_dir / f"{safe}-{k}.mp4"
                k += 1
        job = ExportJob(jid, s, out, n_expected, probe, title)
        job.spool = self._tmp / f"{jid}.mjpg"
        job._f = open(job.spool, "wb")
        with self._lock:
            self.jobs[jid] = job
        return job

    def get(self, jid: str) -> ExportJob:
        return self.jobs[jid]

    def add_frame(self, jid: str, jpeg: bytes, repeat: int = 1) -> int:
        job = self.jobs[jid]
        if job.state != "receiving":
            raise RuntimeError(f"export {jid} is {job.state}")
        if not (len(jpeg) > 3 and jpeg[:2] == b"\xff\xd8"):
            raise ValueError("not a JPEG")
        for _ in range(max(1, repeat)):
            job._f.write(jpeg)
        job.frames += max(1, repeat)
        job.progress = 0.5 * job.frames / max(1, job.n_expected)
        return job.frames

    def finish(self, jid: str, wait: bool = False) -> ExportJob:
        job = self.jobs[jid]
        job._f.close()
        job.state = "encoding"

        def run():
            def prog(f, phase):
                job.progress = 0.5 + 0.5 * f
                job.phase = phase
            try:
                job.result = encode(self.ffmpeg, job.spool, job.frames, job.settings, job.out, self.encoders,
                                    prog, job.title)
                job.state, job.progress = "done", 1.0
            except Exception as e:  # noqa: BLE001
                job.state, job.error = "error", str(e)
            finally:
                try:
                    job.spool.unlink()
                except OSError:
                    pass
                if job.probe:
                    try:
                        job.out.unlink()
                    except OSError:
                        pass

        if wait:
            run()
        else:
            threading.Thread(target=run, name=f"export-{jid}", daemon=True).start()
        return job

    def cancel(self, jid: str) -> None:
        job = self.jobs.get(jid)
        if job and job.state == "receiving":
            job._f.close()
            job.state = "cancelled"
            try:
                job.spool.unlink()
            except OSError:
                pass


def camera_frames(idx, a: float, b: float, fps: float, rate: float = 1.0) -> Iterable[tuple[int, int]]:
    """``(frame index, repeat)`` covering ``[a, b]`` of a frames stream at output ``fps`` and replay ``rate``
    (video time advances ``rate / fps`` per output frame; each output frame shows the newest camera frame at or
    before its time; runs of the same frame are merged)."""
    import numpy as np
    n = int(np.floor((b - a) * fps / rate)) + 1
    times = a + np.arange(n) * rate / fps
    k = np.searchsorted(np.asarray(idx.a["t0"]), times, "right") - 1
    frames = np.asarray(idx.a["kind"]) == 2
    # map to frame records only (the index of a frames stream holds frames; guard anyway)
    k = np.clip(k, 0, max(0, idx.n_records - 1))
    last, run = None, 0
    for i in k.tolist():
        while i > 0 and not frames[i]:
            i -= 1
        if i == last:
            run += 1
            continue
        if last is not None:
            yield last, run
        last, run = i, 1
    if last is not None:
        yield last, run
