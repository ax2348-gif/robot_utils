"""python -m robot_utils.frontend [--config PATH] [--port 80]

Serves the landing page on the unix socket <runtime dir>/robot/frontend.sock, which the robot's
port-80 proxy (nginx, robot-proxy.service) maps to http://<robot>/. ``--port`` additionally
listens on TCP: for running without nginx (the frontend's own /nodes/ proxy then reaches the
nodes) or for debugging. install.sh runs it as robot-frontend.service (docs/frontend.md).
"""

from __future__ import annotations

import argparse
import sys

from ..nodes.common.serve import add_serve_args, serve
from .app import create_app
from .config import ConfigError, load_config


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.frontend", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="node registry YAML (default: the packaged nodes.yaml)")
    add_serve_args(ap, "frontend")
    args = ap.parse_args(argv)

    try:
        nodes = load_config(args.config)
    except ConfigError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    enabled = [n.name for n in nodes if n.enabled]
    serve(create_app(nodes), args,
          banner=f"robot frontend -> nodes: {', '.join(enabled) or 'none configured'}", public_path="/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
