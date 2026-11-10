"""python -m robot_utils.nodes.estimator [--name estimator] [--swerve NAME|URL] [--camera NAME|URL] [--history-s 14400] [--port 8110]

Runs the estimator node: planar pose from the swerve node's wheel odometry and the camera node's
IMU heading (docs/estimator_node.md). Upstreams are instance names on this machine (their unix
sockets; default "swerve" and "camera") or URLs (e.g. http://botcore/nodes/swerve); either may be
missing or down (it then runs on the other alone and reports the mode). ``none`` disables one.
Serves on <runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/; --port is for debugging.
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--swerve", default="swerve", help="swerve node: instance name or URL ('none' to disable)")
    p.add_argument("--camera", default="camera", help="camera node: instance name or URL ('none' to disable)")
    p.add_argument("--history-s", type=float, default=4 * 3600.0,
                   help="pose history window in s (default 4 h, ~86 MB; cleared by POST /reset)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "estimator")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)    # clock sync = 17 requests every 30 s
    from .node import EstimatorNode
    from .server import create_app
    off = lambda u: None if u.lower() == "none" else u   # noqa: E731
    node = EstimatorNode(off(a.swerve), off(a.camera), history_s=a.history_s)
    app = create_app(node)
    serve(app, a, banner=f"estimator node {a.name!r} (swerve {a.swerve}, camera {a.camera})")


if __name__ == "__main__":
    main()
