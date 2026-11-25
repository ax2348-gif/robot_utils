"""python -m robot_utils.nodes.quest_teleop [--name quest_teleop] [--wbc NAME|URL] [--ssh-host quest] [--quest ADDR]
                                         [--ar :80|:PORT|URL|off] [--hand right] ...

Runs the Quest whole-body teleop client (tools/quest_teleop) as a node on this machine, against the local WBC node
(target datagrams over its unix socket), with a page to start / stop it, watch it, and bring the headset's telemetry
up or down over ssh (docs/quest_teleop_node.md). Same on the robot and in the sim stack. Writes only its event log
(--data-dir; events, never frames); in a sim stack (--sim) it also records every teleop run with the collect script
(--record DIR, --no-record; docs/collect.md "Sim teleop records itself").
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--wbc", default="wbc", help="WBC node: instance name (its unix sockets) or URL")
    p.add_argument("--arm", default="arm", help="arm node (instance or URL): the e-stop goes to it directly too; "
                                                "'none' = off")
    p.add_argument("--swerve", default="swerve", help="swerve node (instance or URL): the e-stop goes to it directly "
                                                      "too; 'none' = off")
    p.add_argument("--max-latency-ms", type=float, default=150.0,
                   help="headset frames later than this over the session's best count as stale (disarm)")
    p.add_argument("--min-rate", type=float, default=30.0, help="headset rate (over 0.5 s) below this = stale")
    p.add_argument("--ssh-host", default="quest", help="ssh host of the headset for quest up / down (this machine's "
                                                       "~/.ssh/config; default %(default)s)")
    p.add_argument("--quest", default=None, help="headset stream address (default: the ssh host's HostName, "
                                                 "from `ssh -G`)")
    p.add_argument("--quest-port", type=int, default=9391)
    p.add_argument("--ar", default=":80",
                   help="robot AR view the headset opens on quest up: ':PORT' = this machine's address toward the "
                        "headset at that port (the robot's frontend: :80; the sim stack: :8080), 'host[:port]' or a "
                        "URL, or 'off' (default %(default)s)")
    p.add_argument("--scheme", choices=("body-frozen", "standard"), default="body-frozen")
    p.add_argument("--hand", choices=("left", "right"), default="right", help="the hand that drives the arm")
    p.add_argument("--ee", default="arm", help="end-effector name in the WBC")
    p.add_argument("--no-head-walk", action="store_true",
                   help="engaged: the base hint follows only the stick offset, not the head's travel and yaw")
    p.add_argument("--mode", choices=("third", "first"), default="third",
                   help="third person (default: the sticks move the robot, directions stay room-aligned) or first "
                        "person (the sticks shift the anchor); switchable on the page, from the next grip")
    p.add_argument("--no-headless", action="store_true",
                   help="third person: the left stick moves the robot along its own forward instead of the "
                        "operator's facing (headless, the default; switchable on the page)")
    p.add_argument("--no-gripper", action="store_true")
    p.add_argument("--max-speed", type=float, default=0.8,
                   help="m/s: the left stick's full deflection and the base hint's clamp (the WBC's own caps still "
                        "apply; switchable on the page; default %(default)s)")
    p.add_argument("--max-omega", type=float, default=0.9,
                   help="rad/s: the right stick's full deflection and the base hint's clamp (default %(default)s)")
    p.add_argument("--stale-ms", type=float, default=200.0, help="headset freshness timeout")
    p.add_argument("--ctl-timeout", type=float, default=240.0, help="s: overall limit of one quest up / down")
    p.add_argument("--wait", type=float, default=30.0, help="s: quest up waits this long for the stream")
    p.add_argument("--data-dir", default=None,
                   help="the event log's directory (default ~/.local/share/robot_utils/quest_teleop)")
    p.add_argument("--log-cap-mb", type=float, default=200.0, help="the event log's size cap: oldest files pruned")
    p.add_argument("--sim", default="none", help="the sim node (instance or URL) when this stack is a sim: A resets "
                                                 "it before an episode; 'none' (the robot) = no reset")
    p.add_argument("--reset-timeout", type=float, default=5.0,
                   help="s: a sim reset waits this long for the estimator's reset in the WBC stream")
    p.add_argument("--auto-pause", action="store_true",
                   help="sim only: sim time stops while the operator is neither clutched nor driving the base, and "
                        "runs at the next grip or stick (switchable on the page)")
    p.add_argument("--no-ghost", action="store_true",
                   help="sim only: no gripper ghost at the end effector's goal (on by default; switchable on the page)")
    p.add_argument("--record", default="~/data/sim", metavar="DIR",
                   help="sim only: every teleop run is recorded (python -m robot_utils.collect run, as a child process) "
                        "to DIR/<YYYYMMDD-HHMMSS>, from Start to Stop, with the sim's rigid scene (default %(default)s)")
    p.add_argument("--no-record", action="store_true", help="sim only: do not record teleop runs")
    p.add_argument("--record-streams", default=None,
                   help="sim only: the recorder's stream specs (default: collect's defaults; the rigid scene is added)")
    p.add_argument("--record-robot", default="local",
                   help="sim only: where the recorder reaches the nodes: 'local' (their unix sockets, default) or the "
                        "frontend's URL")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "quest_teleop")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .client import Config, set_top_speed
    from .recording import RecordConfig
    from .eventlog import DATA_DIR
    from .node import QuestTeleopNode
    from .sim_aids import AidsConfig
    from .server import create_app
    cfg = Config(hand=a.hand, ee=a.ee, head_walk=not a.no_head_walk, mode=a.mode, headless=not a.no_headless, gripper=not a.no_gripper)
    try:
        set_top_speed(cfg, a.max_speed, a.max_omega)
    except ValueError as e:
        p.error(str(e))
    off = lambda v: None if v in ("none", "") else v   # noqa: E731
    node = QuestTeleopNode(a.wbc, arm=off(a.arm), swerve=off(a.swerve), quest_host=a.quest, quest_port=a.quest_port,
                           ssh_host=a.ssh_host, cfg=cfg, stale_s=a.stale_ms / 1e3, max_latency=a.max_latency_ms / 1e3,
                           min_rate=a.min_rate, ar=a.ar, scheme=a.scheme, ctl_timeout=a.ctl_timeout,
                           wait=a.wait, name=a.name, data_dir=a.data_dir or DATA_DIR,
                           log_cap_bytes=int(a.log_cap_mb * 1024 * 1024), sim=off(a.sim),
                           reset_timeout=a.reset_timeout,
                           aids=AidsConfig(ghost=not a.no_ghost, auto_pause=a.auto_pause),
                           record=None if a.no_record else RecordConfig(out=a.record, robot=a.record_robot,
                                                                        streams=a.record_streams))
    serve(create_app(node), a, banner=f"quest teleop node {a.name!r} (WBC {a.wbc}, headset {node.quest_host}, "
                                      f"ssh {a.ssh_host}, AR {a.ar})")


if __name__ == "__main__":
    main()
