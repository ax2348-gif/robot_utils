"""``python -m robot_utils.collect browse PATH`` (docs/browse.md): serve the episode browser, or export one camera
headlessly (``--export-camera``)."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def add_parser(sub) -> argparse.ArgumentParser:
    b = sub.add_parser("browse", help="episode browser: replay, plots, 3D, video export (a local web page)")
    b.add_argument("path", help="a session, a cut episode, or a directory of sessions (collect run --out)")
    b.add_argument("--host", default="127.0.0.1", help="listen address (0.0.0.0: reachable from the LAN, e.g. on erid)")
    b.add_argument("--port", type=int, default=8770)
    b.add_argument("--description", help="robot description: a JSON file, a description node URL or a description "
                                         "data dir (default: <session>/description.json, else the package default)")
    b.add_argument("--ffmpeg", help="ffmpeg binary (default: $FFMPEG, PATH, or imageio-ffmpeg's)")
    b.add_argument("--export-dir", help="where videos go (default: ~/robot_utils_exports/<path name>; never inside the data)")
    b.add_argument("--cache", help="index cache directory (default ~/.cache/robot_utils/browse)")
    b.add_argument("--open", action="store_true", help="open the page in the default browser")
    x = b.add_argument_group("headless export of one camera (no server)")
    x.add_argument("--export-camera", metavar="STREAM", help="e.g. camera.color: encode its frames and exit")
    x.add_argument("--episode", help="the episode id to export (default: the whole recording)")
    x.add_argument("--out", help="output .mp4 (default: <export-dir>/<episode>-<stream>.mp4)")
    x.add_argument("--rate", type=float, default=1.0, help="replay rate (2 = twice as fast)")
    x.add_argument("--fps", type=float, default=30.0)
    x.add_argument("--size", default="1280x720", help="WxH")
    x.add_argument("--preset", default="balanced", help="draft | balanced | quality | small")
    x.add_argument("--crf", type=float)
    x.add_argument("--target-mb", type=float, help="encode to this size (two-pass)")
    return b


def run(a) -> int:
    from .dataset import Library
    from .export import Exporter, Settings, camera_frames, encode, estimate, find_ffmpeg
    path = Path(a.path).expanduser().resolve()
    if not path.exists():
        print(f"browse: {path}: no such directory", file=sys.stderr)
        return 1
    cache = Path(a.cache).expanduser() if a.cache else None
    lib = Library(path, cache)
    out_dir = Path(a.export_dir).expanduser() if a.export_dir else Path.home() / "robot_utils_exports" / path.name
    ffmpeg = find_ffmpeg(a.ffmpeg)
    if a.export_camera:
        if not ffmpeg:
            print("browse: no ffmpeg binary (see docs/browse.md 'Export')", file=sys.stderr)
            return 1
        items = lib.items()
        key = items[0]["key"] if items else "."
        rec = lib.open(key)
        s = rec.streams.get(a.export_camera)
        if s is None or getattr(s, "kind", "") != "frames":
            print(f"browse: no camera stream {a.export_camera!r} (have: {', '.join(rec.streams)})", file=sys.stderr)
            return 1
        t0, t1 = rec.t_range
        name = a.export_camera
        if a.episode:
            ep = next((e for e in rec.episodes() if e.get("id") == a.episode), None)
            if ep is None:
                print(f"browse: no episode {a.episode!r}", file=sys.stderr)
                return 1
            t0, t1 = ep["t_start"], ep["t_stop"] or t1
            name = f"{a.episode}-{a.export_camera}"
        w, h = (int(v) for v in a.size.lower().split("x"))
        st = Settings(width=w, height=h, fps=a.fps, preset=a.preset, crf=a.crf, target_mb=a.target_mb)
        ex = Exporter(ffmpeg, out_dir)
        plan = list(camera_frames(s, t0, t1, a.fps, a.rate))
        n = sum(k for _, k in plan)
        e = estimate(st, n / a.fps)
        print(f"browse: {name}: {n} frames, {n / a.fps:.1f} s of video; estimate {e['bytes'] / 1e6:.1f} MB "
              f"({e['how']})", file=sys.stderr)
        job = ex.create(st, n, name)
        if a.out:
            job.out = Path(a.out).expanduser()
            job.out.parent.mkdir(parents=True, exist_ok=True)
        for i, k in plan:
            ex.add_frame(job.id, s.payload(i), k)
        ex.finish(job.id, wait=True)
        if job.state != "done":
            print(f"browse: export failed: {job.error}", file=sys.stderr)
            return 1
        r = job.result
        print(f"{job.out}  {r['bytes'] / 1e6:.2f} MB, {r['passes']} pass(es), encode {r['seconds']:.1f} s"
              + (f", {100 * r['target_error']:+.1f} % of the target" if "target_error" in r else ""))
        return 0
    import uvicorn
    from .server import build_app
    app = build_app(lib, Exporter(ffmpeg, out_dir), a.description)
    url = f"http://{'127.0.0.1' if a.host in ('0.0.0.0', '::') else a.host}:{a.port}/"
    print(f"browse: {path} at {url} (ffmpeg: {ffmpeg or 'none: export off'})", file=sys.stderr)
    if a.open:
        import threading
        import webbrowser
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    uvicorn.run(app, host=a.host, port=a.port, log_level="warning")
    return 0
