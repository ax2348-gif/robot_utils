"""python -m robot_utils.nodes.description [--name description] [--swerve NAME|URL] [--data-dir DIR] [--port N]

Runs the robot description node: the robot's frames and their calibration (docs/description_node.md).
Cameras come from the description (default_description.yaml; override in the data directory):
each names its camera node instance. The yaw calibration drives the base through the swerve node
(--swerve: instance name or URL). Serves on <runtime dir>/robot/<name>.sock =
http://<robot>/nodes/<name>/; --port is for debugging.
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve
from .store import DEFAULT_DATA_DIR


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--swerve", default="swerve", help="swerve node: instance name or URL ('none': no yaw calibration)")
    p.add_argument("--data-dir", default=DEFAULT_DATA_DIR, help=f"live state and calibrations (default {DEFAULT_DATA_DIR})")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "description")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .node import DescriptionNode
    from .server import create_app
    node = DescriptionNode(a.data_dir, None if a.swerve.lower() == "none" else a.swerve)
    serve(create_app(node), a, banner=f"description node {a.name!r} (data {a.data_dir}, swerve {a.swerve})")


if __name__ == "__main__":
    main()
