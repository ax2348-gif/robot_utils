"""python -m robot_utils.nodes.arm [--name arm] [--robot NAME] [--profile soft|safe] [--channel PORT]
[--sim [world[=NAME|DIR]]] [--control process|thread] [--port 8106]

Runs the arm node (docs/arm.md): the 100 Hz control loop and its HTTP API, on the unix socket
<runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/ through the robot's port-80 proxy.
The serial bridge comes from the profile's hardware yaml (rebot_ctl config; ``serial_by_id``);
``--channel`` overrides it. ``--sim`` runs a simulated arm (no hardware); ``--sim world`` runs it on the
MuJoCo sim world (the sim node ``sim``; ``world=NAME`` another instance, ``world=DIR`` its socket directory;
docs/sim.md).
``--robot`` (robots/; default $ROBOT_UTILS_ROBOT, else rebot) says which arm of the robot this instance runs: the
arm named as the instance (``--name arm_left``; a one-arm robot's arm serves any name). It gives the arm's
kinematics (a mirrored arm's mirrored URDF), its description frame and, with ``--sim world``, its backend socket
(``<arm>.sock``). A sim-only robot or a mirrored arm refuses hardware.
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
    p.add_argument("--timing", choices=("auto", "strict", "lenient"), default="auto",
                   help="stall limits: strict = the robot's (a 50 ms reading gap or a 0.25 s silent control process "
                        "faults); lenient = 0.5 s / 2 s, for a simulated arm only; auto (default) = lenient for a "
                        "simulated arm on macOS (no real-time scheduling: it stalled that long under load)")
    p.add_argument("--sim", nargs="?", const="toy", default=None, metavar="world[=NAME|DIR]",
                   help="simulated arm (no hardware): alone, or on the sim world (--sim world; docs/sim.md)")
    p.add_argument("--control", choices=("process", "thread"), default="process",
                   help="where the control loop and the bus run: their own process (default) or a thread of the "
                        "server process")
    p.add_argument("--robot", default=None, help="robot profile (robots/; default $ROBOT_UTILS_ROBOT, else rebot)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "arm")
    args = p.parse_args()
    if args.control == "thread":
        # Request threads (IK, planning) and the 100 Hz control thread share one interpreter: switch
        # more often (default 5 ms) so the control tick waits less behind them.
        sys.setswitchinterval(0.001)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")

    from ...robots import load as load_robot
    robot = load_robot(args.robot)
    arm = robot.arm(args.name)
    arm_type = arm.arm_type()                      # None: the reBot (rebot_ctl's config)
    if arm_type is not None:
        # another arm type (robots/arm_types.py): in the environment before arm/config.py is imported, so N, the gains
        # and the limits are its, here and in the control process (spawned: it inherits the environment)
        import json
        import os

        from ...robots.arm_types import ENV
        os.environ[ENV] = json.dumps(arm_type.public())
    from . import config as C
    from .server import create_app
    if arm_type is not None and not args.sim:
        p.error(f"arm {arm.name!r}: arm type {arm.type!r} has no hardware driver here (sim only); use --sim")
    if not args.sim and (robot.sim_only or arm.mirror):
        p.error(f"robot {robot.name!r}, arm {arm.name!r}: sim only (a mirrored arm needs a per-joint sign map in its "
                "driver: not built); use --sim")

    if args.sim:
        if arm_type is None:
            from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
            limits = joint_limits(load_cfg(C.PROFILES[args.profile]))
        else:                                      # the arm's URDF limits (as its driver would enforce them)
            from ...robots.arm_types import joint_limits as urdf_limits
            limits = urdf_limits(arm_type.urdf, arm.joints)
        if args.sim == "toy":
            from .bus import SimBus
            factory = functools.partial(SimBus, limits=limits)    # picklable: built in the control process
        else:
            from ..sim.backends import world_socket
            from ..sim.backends.arm import WorldArmBus
            path = world_socket(args.sim, arm.name)    # ValueError: a bad --sim value
            factory = functools.partial(WorldArmBus, limits=limits, socket_path=path)
    else:
        from .bus import HardwareBus
        factory = functools.partial(HardwareBus, C.PROFILES[args.profile], channel=args.channel)
    try:
        settings = C.apply_timing(C.load_settings(), args.timing, bool(args.sim), sys.platform)
    except ValueError as e:
        p.error(str(e))
    kw = dict(profile=args.profile, settings=settings,
              joint_socket=socket_path(args.name + ".joint"),   # internal joint channel (docs/arm.md)
              urdf=arm.urdf_path(), arm_frame=arm.frame, robot=robot.name)
    if args.control == "process":
        from .proc_node import ProcNode
        node = ProcNode(factory, **kw, log_level=logging.DEBUG if args.verbose else logging.INFO)
    else:
        from .node import ArmNode
        node = ArmNode(factory, **kw)
    node.events.add(f"timing {settings.timing}: a {settings.stale_on * 1e3:.0f} ms reading gap or a "
                    f"{settings.ctl_heartbeat:g} s silent control process faults")
    node.mirror = arm.mirror                       # GET /links says so (the pages mirror the arm's boxes)
    app = create_app(node, sim=bool(args.sim))
    kind = {None: "hardware", "toy": "sim"}.get(args.sim, f"sim {args.sim}")
    serve(app, args, banner=f"arm node {args.name!r} (robot {robot.name}, arm {arm.name}{' (mirrored)' if arm.mirror else ''}; "
                            f"{kind}, profile {args.profile}; control loop: {args.control})")


if __name__ == "__main__":
    main()
