"""python -m robot_utils.nodes.wbc [--name wbc] [--model description|local|FILE.urdf] [--arm NAME] [--swerve NAME|URL]
[--estimator NAME|URL] [--udp-host ADDR] [--udp-port N] [--config FILE.yaml] [--control process|thread]
[--switch-interval S]

Runs the whole-body control node (docs/wbc.md): end-effector targets in the world frame (datagrams), the arm
(through its local joint channel) and the base (the swerve's /command) out. Serves on
<runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/; target datagrams on UDP --udp-host:--udp-port
(the robot's LAN address on the robot; never 0.0.0.0) and <runtime dir>/robot/<name>.target.sock.

The 100 Hz control loop runs in its own process (``--control process``, the default: docs/wbc.md "Processes"), free
of this process's GIL (HTTP, SSE, the stream readers); ``--control thread`` runs it as a thread of this process (the
old structure, for comparison and debugging).
"""
import argparse
import logging
import os
import sys

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default="description",
                   help="robot model: a description node (instance name or URL), 'local', or a .urdf (base=planar)")
    p.add_argument("--arm", default="arm", help="arm node instance (its HTTP and <name>.joint.sock); 'none' = off")
    p.add_argument("--swerve", default="swerve", help="swerve node: instance name or URL; 'none' = no base")
    p.add_argument("--estimator", default="estimator", help="estimator node: instance name or URL; 'none' = off")
    p.add_argument("--udp-host", default="127.0.0.1", help="address for target datagrams ('none' = UDP off)")
    p.add_argument("--udp-port", type=int, default=9410)
    p.add_argument("--config", help="YAML of Config keys (caps, gains, timeouts) over the defaults")
    p.add_argument("--control", choices=("process", "thread"), default="process",
                   help="where the control loop runs: its own process (default) or a thread of the server process")
    p.add_argument("--switch-interval", type=float,
                   default=float(os.environ.get("WBC_SWITCH_INTERVAL", "0.005")),
                   help="Python's GIL switch interval (s; sys.setswitchinterval) for this process: the real-time "
                        "control thread waits at most about this long for the GIL held by a normal-priority thread "
                        "(HTTP, SSE, stream readers; with --control thread). Default 0.005 = Python's own (env WBC_SWITCH_INTERVAL); 0.0005 made the robot's tick overruns much worse on 2026-11-21 (more GIL handoffs, more priority inversion)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "wbc")
    a = p.parse_args()
    if not 0 < a.switch_interval <= 0.1:
        p.error("--switch-interval must be within (0, 0.1] s")
    sys.setswitchinterval(a.switch_interval)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)    # 100 commands/s
    cfg = None
    if a.config:
        import yaml
        with open(a.config) as f:
            cfg = yaml.safe_load(f) or {}
    off = lambda v: None if v in (None, "", "none") else v   # noqa: E731
    from .server import create_app
    kw = dict(model_source=a.model, arm=off(a.arm), swerve=off(a.swerve), estimator=off(a.estimator), config=cfg,
              udp_host=off(a.udp_host), udp_port=a.udp_port, name=a.name)
    if a.control == "process":
        from .proc_node import ProcNode
        node = ProcNode(**kw, log_level=logging.DEBUG if a.verbose else logging.INFO)
    else:
        from .node import WbcNode
        node = WbcNode(**kw)
    serve(create_app(node), a, banner=f"wbc node {a.name!r} (arm {a.arm}, swerve {a.swerve}, estimator {a.estimator}, "
                                      f"targets udp {a.udp_host}:{a.udp_port}; control loop: {a.control})")


if __name__ == "__main__":
    main()
