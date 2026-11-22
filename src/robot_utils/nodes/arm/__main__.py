"""python -m robot_utils.nodes.arm [--name arm] [--profile soft|safe] [--channel PORT] [--sim [world[=NAME|DIR]]]
[--control process|thread] [--port 8106]

Runs the arm node (docs/arm.md): the 100 Hz control loop and its HTTP API, on the unix socket
<runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/ through the robot's port-80 proxy.
The serial bridge comes from the profile's hardware yaml (rebot_ctl config; ``serial_by_id``);
``--channel`` overrides it. ``--sim`` runs a simulated arm (no hardware); ``--sim world`` runs it on the
MuJoCo sim world (the sim node ``sim``; ``world=NAME`` another instance, ``world=DIR`` its socket directory;
docs/sim.md).
Nothing moves until a lease holder enables it; the first enable closes the gripper to check it.

The control loop and the bus run in their own process (``--control process``, the default: docs/arm.md "Processes"),
free of this process's GIL (HTTP, SSE, the stream clients, IK); ``--control thread`` runs them as a thread of this
process (the old structure, for comparison and debugging).
"""
import argparse
import functools
import logging
import sys

from ..common.serve import add_serve_args, serve, socket_path

DEBUG_PORT = 8106


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--profile", default="soft", choices=["soft", "safe"], help="gain profile at start (default soft)")
    p.add_argument("--channel", default=None, help="serial bridge (default: the hardware yaml's serial_by_id)")
    p.add_argument("--sim", nargs="?", const="toy", default=None, metavar="world[=NAME|DIR]",
                   help="simulated arm (no hardware): alone, or on the sim world (--sim world; docs/sim.md)")
    p.add_argument("--control", choices=("process", "thread"), default="process",
                   help="where the control loop and the bus run: their own process (default) or a thread of the "
                        "server process")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "arm")
    args = p.parse_args()
    if args.control == "thread":
        # Request threads (IK, planning) and the 100 Hz control thread share one interpreter: switch
        # more often (default 5 ms) so the control tick waits less behind them.
        sys.setswitchinterval(0.001)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")

    from . import config as C
    from .server import create_app

    if args.sim:
        from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
        limits = joint_limits(load_cfg(C.PROFILES[args.profile]))
        if args.sim == "toy":
            from .bus import SimBus
            factory = functools.partial(SimBus, limits=limits)    # picklable: built in the control process
        else:
            from ..sim.backends import world_socket
            from ..sim.backends.arm import WorldArmBus
            path = world_socket(args.sim, "arm")       # ValueError: a bad --sim value
            factory = functools.partial(WorldArmBus, limits=limits, socket_path=path)
    else:
        from .bus import HardwareBus
        factory = functools.partial(HardwareBus, C.PROFILES[args.profile], channel=args.channel)
    kw = dict(profile=args.profile, settings=C.load_settings(),
              joint_socket=socket_path(args.name + ".joint"))   # internal joint channel (docs/arm.md)
    if args.control == "process":
        from .proc_node import ProcNode
        node = ProcNode(factory, **kw, log_level=logging.DEBUG if args.verbose else logging.INFO)
    else:
        from .node import ArmNode
        node = ArmNode(factory, **kw)
    app = create_app(node, sim=bool(args.sim))
    kind = {None: "hardware", "toy": "sim"}.get(args.sim, f"sim {args.sim}")
    serve(app, args, banner=f"arm node {args.name!r} ({kind}, profile {args.profile}; control loop: {args.control})")


if __name__ == "__main__":
    main()
