"""python -m robot_utils.replay SESSION_OR_EPISODE [-o OUT.mp4] [--episode ID] [--eye center|left|right] ...

The operator's view of a recorded sim teleop session or episode, as an mp4 (docs/collect.md, "Operator view
replay"): the rigid scene from the logged head pose through the logged AR placement (reconstructed for older logs),
the controllers where they were, a plain background; no HUD, no passthrough. Writes OUT.mp4 and OUT.mp4.json (the
summary and each frame's camera).
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

from .log import load
from .operator_view import ViewOptions, render_video


def _floats(s: str, n: int) -> tuple:
    v = tuple(float(x) for x in s.split(","))
    if len(v) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated numbers")
    return v


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m robot_utils.replay", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("path", help="a session directory (session.json) or a cut episode directory (episode.json)")
    p.add_argument("-o", "--out", help="the mp4 (default: <path>/operator_view[_<episode>].mp4)")
    p.add_argument("--episode", help="one episode of a session (its window from episodes.jsonl)")
    p.add_argument("--eye", choices=("center", "left", "right"), default="center",
                   help="center: between the eyes with the binocular field (default); left / right: that eye as it "
                        "was (its pose and its asymmetric field of view)")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--width", type=int, default=960, help="pixels; the height follows the field of view")
    p.add_argument("--controllers", choices=("overlay", "depth", "off"), default="overlay",
                   help="overlay: on top of the scene (default); depth: hidden behind what is in front")
    p.add_argument("--role", choices=("auto", "visual", "collision"), default=None,
                   help="which geoms (default: the headset's setting saved with the scene, else auto)")
    p.add_argument("--bg", type=lambda s: _floats(s, 3), default=(0.12, 0.12, 0.13), help="background r,g,b 0..1")
    p.add_argument("--start", type=float, default=0.0, help="s to skip at the start")
    p.add_argument("--duration", type=float, default=None, help="s to render (default: all)")
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--ar-offset", type=lambda s: _floats(s, 3), default=(0.0, 0.0, 0.0),
                   help="old logs only (reconstructed placement): the headset's --ar-offset x,y,z")
    p.add_argument("--ar-yaw", type=float, default=0.0, help="old logs only: the headset's --ar-yaw, deg")
    p.add_argument("--ar-scale", type=float, default=1.0, help="old logs only: the headset's --ar-scale")
    p.add_argument("--ffmpeg", help="the ffmpeg executable (default: imageio_ffmpeg's, or ffmpeg on the PATH)")
    p.add_argument("--crf", type=int, default=20, help="H.264 quality (lower is better)")
    p.add_argument("-v", "--verbose", action="store_true")
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    try:
        rec = load(a.path, a.episode)
    except (OSError, ValueError) as e:
        print(f"replay: {e}", file=sys.stderr)
        return 1
    out = Path(a.out) if a.out else Path(a.path) / (f"operator_view_{a.episode}.mp4" if a.episode
                                                     else "operator_view.mp4")
    opts = ViewOptions(fps=a.fps, width=a.width, eye=a.eye, controllers=a.controllers, role=a.role, bg=a.bg,
                       start=a.start, duration=a.duration, max_frames=a.max_frames, ar_offset=a.ar_offset,
                       ar_yaw_deg=a.ar_yaw, ar_scale=a.ar_scale)

    def progress(n, _):
        if n % 100 == 0:
            print(f"replay: {n} frames", file=sys.stderr, flush=True)

    try:
        summary = render_video(rec, out, opts, ffmpeg=a.ffmpeg, crf=a.crf, progress=progress)
    except (RuntimeError, ValueError) as e:
        print(f"replay: {e}", file=sys.stderr)
        return 1
    if summary["placement"] == "reconstructed":
        print("replay: this log has no logged AR placement (an older headset build): the placement is RECONSTRUCTED "
              "from the first tracked head and each reset_seq, with the floor estimated 1.6 m below the head",
              file=sys.stderr)
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
