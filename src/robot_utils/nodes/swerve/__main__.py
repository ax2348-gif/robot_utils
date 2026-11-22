"""python -m robot_utils.nodes.swerve [--name swerve] [--interface slcan:/dev/...] [--sim [world[=NAME|DIR]]] [--port 8102]

Runs the swerve node: the 100 Hz control loop plus its HTTP API (docs/swerve_node.md), on the
unix socket <runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/ through the robot's
port-80 proxy; --port adds a TCP port for debugging (docs/nodes.md).
Configure the SPARKs once with ``python -m robot_utils.nodes.swerve.setup_sparks`` first.
``--sim`` runs against 8 simulated SPARKs on a virtual bus (no hardware), for trying the API; ``--sim world``
couples those SPARKs to the MuJoCo sim world (the sim node ``sim``; ``world=NAME`` another instance,
``world=DIR`` its socket directory; docs/sim.md).
"""
import argparse
import glob
import logging
import os
import socket
import subprocess

from ..common.serve import add_serve_args, serve

DEBUG_PORT = 8102   # suggested --port (debugging only)


def default_interface() -> str:
    env = os.environ.get("CAN")
    if env:
        return env
    canable = sorted(glob.glob("/dev/serial/by-id/usb-Openlight_Labs_CANable2*"))
    return f"slcan:{canable[0]}" if canable else "slcan:/dev/ttyACM0"


def lan_addresses() -> list[str]:
    """This machine's IPv4 addresses and hostname (kept for nodes that still print TCP URLs,
    e.g. the description node until it moves to the serve helper)."""
    try:
        addrs = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout.split()
    except (OSError, subprocess.SubprocessError):
        addrs = []
    return [a for a in addrs if ":" not in a] + [socket.gethostname()]


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--interface", default=None, help="sparkcan interface (default: $CAN, else the CANable "
                                                     "under /dev/serial/by-id, else slcan:/dev/ttyACM0)")
    p.add_argument("--sim", nargs="?", const="toy", default=None, metavar="world[=NAME|DIR]",
                   help="simulated SPARKs on a virtual bus (no hardware): alone, or moving the sim world's base "
                        "(--sim world; docs/sim.md)")
    p.add_argument("--status-ms", type=int, default=None, help="SPARK status frame period at start, ms (default 10; "
                                                               "changed live with POST /status_period)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "swerve")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")

    from .node import SwerveNode
    from .server import create_app

    sims = world = None
    if args.sim == "toy":
        from .sim import start_sim
        sims, interface = start_sim()
    elif args.sim:
        from ..sim.backends import wait_for_world, world_socket
        from ..sim.backends.swerve import start_world_sim
        path = world_socket(args.sim, "swerve")
        if not wait_for_world(path):
            p.error(f"--sim {args.sim}: no sim world at {path} (start python -m robot_utils.nodes.sim first)")
        sims, interface, world = start_world_sim(socket_path=path)
    else:
        interface = args.interface or default_interface()
    node = SwerveNode(interface) if args.status_ms is None else SwerveNode(interface, status_period_ms=args.status_ms)
    app = create_app(node)
    try:
        serve(app, args, banner=f"swerve node {args.name!r} on {interface}")
    finally:
        if world is not None:
            from ..sim.backends.swerve import stop_world_sim
            stop_world_sim(sims, interface, world)
        elif sims is not None:
            from .sim import stop_sim
            stop_sim(sims, interface)


if __name__ == "__main__":
    main()
