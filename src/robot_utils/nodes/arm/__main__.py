"""python -m robot_utils.nodes.arm [--name arm] [--profile soft|safe] [--channel PORT] [--sim] [--port 8106]

Runs the arm node (docs/arm.md): the 100 Hz control loop and its HTTP API, on the unix socket
<runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/ through the robot's port-80 proxy.
The serial bridge comes from the profile's hardware yaml (rebot_ctl config; ``serial_by_id``);
``--channel`` overrides it. ``--sim`` runs a simulated arm (no hardware).
Nothing moves until a lease holder enables it; the first enable closes the gripper to check it.
"""
import argparse
import logging
import sys

from ..common.serve import add_serve_args, serve

DEBUG_PORT = 8106


def parse_cpus(text):
    """'3' or '2,3' or '1-3' -> {CPU numbers}; None or '' -> None."""
    if not text:
        return None
    out = set()
    for part in text.split(","):
        lo, _, hi = part.strip().partition("-")
        out.update(range(int(lo), int(hi or lo) + 1))
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--profile", default="soft", choices=["soft", "safe"], help="gain profile at start (default soft)")
    p.add_argument("--channel", default=None, help="serial bridge (default: the hardware yaml's serial_by_id)")
    p.add_argument("--sim", action="store_true", help="simulated arm (no hardware)")
    p.add_argument("--switch-interval", type=float, default=0.001, metavar="S",
                   help="interpreter switch interval, s (default 0.001; Python's own is 0.005): how long the control "
                        "thread waits for another Python thread holding the interpreter lock before forcing a switch")
    p.add_argument("--rt-cpus", default=None, metavar="LIST",
                   help="pin the control thread (and the bus poller it starts) to these CPUs, e.g. 3 or 2,3 "
                        "(default: not pinned)")
    p.add_argument("--tick-phases", action="store_true",
                   help="record where each control tick's time goes (GET /debug/tick_phases)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "arm")
    args = p.parse_args()
    # Request threads (IK, planning) and the 100 Hz control thread share one interpreter: switch
    # more often (default 5 ms) so the control tick waits less behind them.
    sys.setswitchinterval(args.switch_interval)
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
    node = ArmNode(factory, profile=args.profile, settings=C.load_settings(), tick_phases=args.tick_phases,
                   rt_cpus=parse_cpus(args.rt_cpus))
    app = create_app(node, sim=args.sim)
    serve(app, args, banner=f"arm node {args.name!r} ({'sim' if args.sim else 'hardware'}, profile {args.profile})")


if __name__ == "__main__":
    main()
