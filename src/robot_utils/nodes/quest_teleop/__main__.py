"""python -m robot_utils.nodes.quest_teleop [--name quest_teleop] [--wbc NAME|URL] [--ssh-host quest] [--quest ADDR]
                                         [--ar :80|:PORT|URL|off] [--hand right] ...

Runs the Quest whole-body teleop client (tools/quest_teleop) as a node on this machine, against the local WBC node
(target datagrams over its unix socket), with a page to start / stop it, watch it, and bring the headset's telemetry
up or down over ssh (docs/quest_teleop_node.md). Same on the robot and in the sim stack. Writes nothing to disk.
"""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--wbc", default="wbc", help="WBC node: instance name (its unix sockets) or URL")
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
    p.add_argument("--no-gripper", action="store_true")
    p.add_argument("--stale-ms", type=float, default=200.0, help="headset freshness timeout")
    p.add_argument("--ctl-timeout", type=float, default=240.0, help="s: overall limit of one quest up / down")
    p.add_argument("--wait", type=float, default=30.0, help="s: quest up waits this long for the stream")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "quest_teleop")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .client import Config
    from .node import QuestTeleopNode
    from .server import create_app
    cfg = Config(hand=a.hand, ee=a.ee, head_walk=not a.no_head_walk, gripper=not a.no_gripper)
    node = QuestTeleopNode(a.wbc, quest_host=a.quest, quest_port=a.quest_port, ssh_host=a.ssh_host, cfg=cfg,
                           stale_s=a.stale_ms / 1e3, ar=a.ar, scheme=a.scheme, ctl_timeout=a.ctl_timeout,
                           wait=a.wait, name=a.name)
    serve(create_app(node), a, banner=f"quest teleop node {a.name!r} (WBC {a.wbc}, headset {node.quest_host}, "
                                      f"ssh {a.ssh_host}, AR {a.ar})")


if __name__ == "__main__":
    main()
