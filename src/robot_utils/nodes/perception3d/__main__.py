"""python -m robot_utils.nodes.perception3d [--name 3d_perception] [--camera NAME|URL] [--estimator NAME|URL]
                                        [--description NAME|URL] [--scale 0.5] [--stride 1] [--port N]

Runs the 3d_perception node: the camera's newest depth frame registered into every colour frame at the
colour frame's capture time, motion-compensated with the camera IMU (rotation) and the estimator's
base pose + the description's camera pose (translation) (docs/perception3d_node.md). Peers are
instance names on this machine (their unix sockets) or URLs; ``none`` disables one (the estimator and
description are optional: rotation-only compensation without them).
Serves on <runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/; --port is for debugging.
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--camera", default="camera", help="camera node: instance name or URL")
    p.add_argument("--estimator", default="estimator", help="estimator node (base pose): name or URL ('none' to disable)")
    p.add_argument("--description", default="description",
                   help="description node (camera pose in base): name or URL ('none' to disable)")
    p.add_argument("--scale", type=float, default=0.5, help="output depth grid = colour resolution x scale (default 0.5)")
    p.add_argument("--stride", type=int, default=1, help="use every n-th depth pixel (default 1)")
    p.add_argument("--splat", type=int, choices=(1, 2), default=1, help="2: write 2x2 blocks (for scale near 1)")
    p.add_argument("--idle-s", type=float, default=30.0,
                   help="stop registering after this long without consumers (0 = always run; default 30)")
    p.add_argument("--allow-nominal", action="store_true",
                   help="use the description's nominal (uncalibrated) camera pose for translation compensation")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "3d_perception")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .node import Params, PerceptionNode
    from .server import create_app
    off = lambda u: None if u.lower() == "none" else u   # noqa: E731
    node = PerceptionNode(a.camera, off(a.estimator), off(a.description),
                          Params(scale=a.scale, stride=a.stride, splat=a.splat, idle_s=a.idle_s,
                                 allow_nominal=a.allow_nominal))
    serve(create_app(node), a, banner=f"3d_perception node {a.name!r} (camera {a.camera}, estimator {a.estimator}, "
                                      f"description {a.description})")


if __name__ == "__main__":
    main()
