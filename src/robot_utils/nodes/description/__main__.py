"""python -m robot_utils.nodes.description [--swerve URL] [--data-dir DIR] [--host 0.0.0.0] [--port 8104]

Runs the robot description node: the robot's frames and their calibration (docs/description_node.md).
Cameras and their URLs come from the description (default_description.yaml; override in the data
directory). The yaw calibration drives the base through the swerve node at --swerve.
"""
import argparse
import logging

from ..swerve.__main__ import lan_addresses
from .store import DEFAULT_DATA_DIR

DEFAULT_PORT = 8104


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--swerve", default="http://127.0.0.1:8102", help="swerve node URL ('none': no yaw calibration)")
    p.add_argument("--data-dir", default=DEFAULT_DATA_DIR, help=f"live state and calibrations (default {DEFAULT_DATA_DIR})")
    p.add_argument("--host", default="0.0.0.0", help="address to bind (default: all interfaces)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("-v", "--verbose", action="store_true")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    import uvicorn
    from .node import DescriptionNode
    from .server import create_app
    node = DescriptionNode(a.data_dir, None if a.swerve.lower() == "none" else a.swerve)
    app = create_app(node)
    print(f"description node (data {a.data_dir}, swerve {a.swerve}); open:")
    for addr in lan_addresses():
        print(f"  http://{addr}:{a.port}/      (API docs: http://{addr}:{a.port}/docs)")
    uvicorn.run(app, host=a.host, port=a.port, workers=1, log_level="warning")


if __name__ == "__main__":
    main()
