"""python -m robot_utils.nodes.agent_nav [--name agent_nav] [--pose-servo NAME|URL] [--perception NAME|URL]
                                       [--estimator NAME|URL] [--profile cautious|normal] [--data-dir DIR]

The agent <-> robot boundary (docs/agent_nav.md; agents read GET /guide.md). Read-only perception and
planning; motion only by explicit goals through pose_servo; a stop-only reflex; a dialogue transcript.
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pose-servo", default="pose_servo")
    p.add_argument("--perception", default="3d_perception")
    p.add_argument("--estimator", default="estimator")
    p.add_argument("--description", default="description", help="robot description node ('none' to skip)")
    p.add_argument("--arm", default="arm", help="arm node, for the robot drawn by /plan/check ('none' to skip)")
    p.add_argument("--profile", default="cautious", choices=["cautious", "normal"])
    p.add_argument("--no-require-imu", action="store_true", help="allow goals without IMU heading (simulation)")
    p.add_argument("--data-dir", default=None, help="transcripts (default ~/.local/share/robot_utils/agent_nav)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "agent_nav")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .node import DATA_DIR, AgentNav
    from .server import create_app
    node = AgentNav(a.pose_servo, a.perception, a.estimator, a.data_dir or DATA_DIR, a.profile,
                    require_imu=not a.no_require_imu,
                    description=None if a.description.lower() == "none" else a.description,
                    arm=None if a.arm.lower() == "none" else a.arm)
    serve(create_app(node), a, banner=f"agent_nav {a.name!r} (profile {a.profile})")


if __name__ == "__main__":
    main()
