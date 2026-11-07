"""python -m robot_utils.nodes.estimator [--swerve URL] [--camera URL] [--host 0.0.0.0] [--port 8110]

Runs the estimator node: planar pose from the swerve node's wheel odometry and the camera node's
IMU heading (docs/estimator_node.md). Both upstreams default to this machine; either may be
missing or down (it then runs on the other alone and reports the mode). ``none`` disables one.
"""
import argparse
import logging

from ..swerve.__main__ import lan_addresses

DEFAULT_PORT = 8110


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--swerve", default="http://127.0.0.1:8102", help="swerve node URL ('none' to disable)")
    p.add_argument("--camera", default="http://127.0.0.1:8101", help="camera node URL ('none' to disable)")
    p.add_argument("--host", default="0.0.0.0", help="address to bind (default: all interfaces)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("-v", "--verbose", action="store_true")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)    # clock sync = 17 requests every 30 s
    import uvicorn
    from .node import EstimatorNode
    from .server import create_app
    off = lambda u: None if u.lower() == "none" else u   # noqa: E731
    node = EstimatorNode(off(a.swerve), off(a.camera))
    app = create_app(node)
    print(f"estimator node (swerve {a.swerve}, camera {a.camera}); open:")
    for addr in lan_addresses():
        print(f"  http://{addr}:{a.port}/      (API docs: http://{addr}:{a.port}/docs)")
    uvicorn.run(app, host=a.host, port=a.port, workers=1, log_level="warning")


if __name__ == "__main__":
    main()
