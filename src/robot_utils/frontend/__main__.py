"""python -m robot_utils.frontend [--host 0.0.0.0] [--port 80] [--config PATH]"""

from __future__ import annotations

import argparse
import errno
import socket
import sys

import uvicorn

from .app import create_app
from .config import ConfigError, load_config


def bind(host: str, port: int) -> socket.socket:
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    sock = socket.socket(family, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((host, port))
    except OSError:
        sock.close()
        raise
    sock.listen(128)
    sock.set_inheritable(True)
    return sock


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.frontend", description=__doc__)
    ap.add_argument("--host", default="0.0.0.0", help="address to listen on (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=80, help="port to listen on (default 80)")
    ap.add_argument("--config", help="node registry YAML (default: the packaged nodes.yaml)")
    args = ap.parse_args(argv)

    try:
        nodes = load_config(args.config)
    except ConfigError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    try:
        sock = bind(args.host, args.port)
    except PermissionError:
        print(
            f"error: not allowed to listen on port {args.port} as a normal user.\n"
            "  Allow ports >= 80 once (persists across reboots):\n"
            "    sudo tools/enable_low_ports.sh\n"
            "  or use a high port: python -m robot_utils.frontend --port 8080",
            file=sys.stderr,
        )
        return 1
    except OSError as e:
        if e.errno == errno.EADDRINUSE:
            print(f"error: port {args.port} is already in use (another frontend running?)", file=sys.stderr)
        else:
            print(f"error: cannot listen on {args.host}:{args.port}: {e}", file=sys.stderr)
        return 1

    enabled = [n for n in nodes if n.enabled]
    print(
        f"robot frontend on http://{args.host}:{args.port}/ -> "
        + (", ".join(f"{n.name}:{n.port}" for n in enabled) or "no nodes configured"),
        flush=True,
    )
    server = uvicorn.Server(uvicorn.Config(create_app(nodes), log_level="warning"))
    server.run(sockets=[sock])
    return 0


if __name__ == "__main__":
    sys.exit(main())
