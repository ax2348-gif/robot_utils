"""python -m robot_utils.nodes.pose_servo [--name pose_servo] [--swerve NAME|URL] [--estimator NAME|URL]

Runs the pose servo node (docs/pose_servo_node.md): goals in, jerk-limited swerve twists out, using
the estimator node's pose. Peers are instance names on this machine (their unix sockets; default
"swerve" and "estimator") or URLs. Serves on <runtime dir>/robot/<name>.sock =
http://<robot>/nodes/<name>/; --port is for debugging.
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--swerve", default="swerve", help="swerve node: instance name or URL")
    p.add_argument("--estimator", default="estimator", help="estimator node: instance name or URL")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "pose_servo")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)    # 50 commands/s while moving
    from .node import PoseServoNode
    from .server import create_app
    node = PoseServoNode(a.swerve, a.estimator)
    serve(create_app(node), a, banner=f"pose servo node {a.name!r} (swerve {a.swerve}, estimator {a.estimator})")


if __name__ == "__main__":
    main()
