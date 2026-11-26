"""python -m robot_utils.nodes.agent_interface [--name agent] [--pose-servo NAME|URL] [--perception NAME|URL]
                                       [--estimator NAME|URL] [--profile cautious|normal] [--data-dir DIR]

The agent <-> robot boundary (docs/agent_interface.md; agents read GET /guide.md). Read-only perception and
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
    p.add_argument("--arm", default=None, help="arm node(s), comma-separated: the first is /arm/..., each is "
                                               "/arms/{name}/... (default: the robot's arms; 'none' to skip)")
    p.add_argument("--robot", default=None, help="robot profile (robots/; default $ROBOT_UTILS_ROBOT, else rebot)")
    p.add_argument("--cameras", default="camera,wrist_camera",
                   help="cameras POST /view may read: names of camera node instances, or name=URL, comma separated")
    p.add_argument("--profile", default="cautious", choices=["cautious", "normal"])
    p.add_argument("--no-require-imu", action="store_true", help="allow goals without IMU heading (simulation)")
    p.add_argument("--data-dir", default=None, help="transcripts (default ~/.local/share/robot_utils/agent_interface)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "agent_interface")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .node import DATA_DIR, AgentNav
    from .server import create_app
    node = AgentNav(a.pose_servo, a.perception, a.estimator, a.data_dir or DATA_DIR, a.profile,
                    require_imu=not a.no_require_imu,
                    description=None if a.description.lower() == "none" else a.description,
                    arms=arm_nodes(a.arm, a.robot),
                    cameras=dict((c.split("=", 1) if "=" in c else (c, c)) for c in a.cameras.split(",") if c))
    serve(create_app(node), a, banner=f"agent {a.name!r} (profile {a.profile})")


def arm_nodes(arm, robot) -> list:
    """The arm nodes: ``--arm`` (comma-separated; 'none': none), else the robot profile's arms."""
    if arm is not None:
        return [] if arm.lower() == "none" else [x for x in arm.split(",") if x]
    from ...robots import load
    return [x.name for x in load(robot).arms]


if __name__ == "__main__":
    main()
