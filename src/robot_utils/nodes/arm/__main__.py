"""python -m robot_utils.nodes.arm [--name arm] [--profile soft|safe] [--channel PORT] [--sim] [--port 8106]

Runs the arm node (docs/arm.md): the 100 Hz control loop and its HTTP API, on the unix socket
<runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/ through the robot's port-80 proxy.
The serial bridge comes from the profile's hardware yaml (rebot_ctl config; ``serial_by_id``);
``--channel`` overrides it. ``--sim`` runs a simulated arm (no hardware).
Nothing moves until a lease holder enables it; the first enable closes the gripper to check it.
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve

DEBUG_PORT = 8106


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--profile", default="soft", choices=["soft", "safe"], help="gain profile at start (default soft)")
    p.add_argument("--channel", default=None, help="serial bridge (default: the hardware yaml's serial_by_id)")
    p.add_argument("--sim", action="store_true", help="simulated arm (no hardware)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "arm")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")

    from . import config as C
    from .node import ArmNode
    from .server import create_app

    if args.sim:
        from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg

        from .bus import SimBus
        limits = joint_limits(load_cfg(C.PROFILES[args.profile]))
        factory = lambda: SimBus(limits=limits)   # noqa: E731
    else:
        from .bus import HardwareBus
        factory = lambda: HardwareBus(C.PROFILES[args.profile], channel=args.channel)   # noqa: E731
    node = ArmNode(factory, profile=args.profile, settings=C.load_settings())
    app = create_app(node, sim=args.sim)
    serve(app, args, banner=f"arm node {args.name!r} ({'sim' if args.sim else 'hardware'}, profile {args.profile})")


if __name__ == "__main__":
    main()
