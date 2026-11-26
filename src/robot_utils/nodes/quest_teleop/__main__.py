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
    p.add_argument("--robot", default=None,
                   help="robot profile (robots/; default $ROBOT_UTILS_ROBOT, else rebot): its arms are the e-stop's "
                        "arm targets and, with two, each hand drives one (left -> the left arm, right -> the right)")
    p.add_argument("--arm", default=None, help="arm node(s) (instance or URL, comma-separated): the e-stop goes to them "
                                               "directly too (default: the robot's arms); 'none' = off")
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
    p.add_argument("--body", choices=("off", "core", "all"), default="off",
                   help="quest up: Meta body tracking in each headset frame (quest_env BODY_TRACKING.md; runtime "
                        "1.7.0); recorded and shown on /state; the chest / hips hint sources and --elbow-hint use it "
                        "(default %(default)s)")
    p.add_argument("--hand", choices=("left", "right"), default=None,
                   help="one hand drives one arm (default right; with a two-arm robot this selects one-hand teleop)")
    p.add_argument("--ee", default=None, help="end-effector name in the WBC for one hand (default: the robot's one arm; "
                                              "on a two-arm robot its right arm)")
    p.add_argument("--hands", default=None, metavar="left=EE,right=EE",
                   help="two hands, two arms: each hand's grip clutches its EE (default on a two-arm robot: left -> "
                        "its left arm, right -> its right arm)")
    p.add_argument("--mirror", choices=("off", "auto", "straight", "mirror"), default="auto",
                   help="two hands: which arm each hand drives, decided when the first hand engages from all released "
                        "and locked until both release: off / straight = left hand -> left arm; mirror = "
                        "swapped; auto (default) = mirrored when the robot faces the operator (headings > 90 deg apart); "
                        "switchable on the page while no hand is engaged")
    p.add_argument("--no-head-walk", action="store_true",
                   help="engaged: the base hint follows only the stick offset, not the head's travel and yaw")
    p.add_argument("--mode", choices=("third", "first"), default="third",
                   help="third person (default: the sticks move the robot, directions stay room-aligned) or first "
                        "person (the sticks shift the anchor); switchable on the page, from the next grip")
    p.add_argument("--no-headless", action="store_true",
                   help="third person: the left stick moves the robot along its own forward instead of the "
                        "operator's facing (headless, the default; switchable on the page)")
    p.add_argument("--no-gripper", action="store_true")
    p.add_argument("--tether", action="store_true",
                   help="engaged: hold the stick offset within 5 cm / 0.1 rad of the WBC's commanded base (the stick "
                        "tether; off by default: the stick ramp instead; switchable on the page, from the next grip)")
    p.add_argument("--hint-source", choices=("head", "chest", "hips"), default="head",
                   help="what moves the base hint while engaged: the headset (default), or with body tracking "
                        "(--body) the chest or the pelvis, smoothed, falling back to the headset (switchable on the "
                        "page; from the next grip; tools/quest_teleop/README.md)")
    p.add_argument("--elbow-hint", action="store_true",
                   help="send each engaged arm's elbow-swivel input to the WBC (the operator's swivel change since "
                        "the grip, from body tracking: needs --body; switchable on the page; on applies from the "
                        "next grip; tools/quest_teleop/swivel.py)")
    p.add_argument("--hint-tau", type=float, default=0.15, help="s: body sources' low-pass time constant")
    p.add_argument("--hint-deadband", type=float, nargs=2, default=(0.02, 0.03), metavar=("M", "RAD"),
                   help="body sources' deadband: metres in the plane, radians of yaw")
    p.add_argument("--hint-min-confidence", type=float, default=0.5,
                   help="body confidence below this: the base hint falls back to the headset")
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
    p.add_argument("--autostart", action="store_true",
                   help="sim only: start teleop by itself once the WBC answers (the sim stack passes it), as if Start "
                        "were pressed: the sim resets, recording starts")
    p.add_argument("--estimator", default="estimator",
                   help="the estimator instance a sim reset resets with it (default %(default)s)")
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
    p.add_argument("--resume-grace", type=float, default=2.0, metavar="S",
                   help="tiered recovery: a transient dropout (stale / late input, a gap, an unfocused frame, the WBC's "
                        "own short dropout) holds, and releasing and re-gripping resumes without Y, if the cause "
                        "cleared within S seconds (default %(default)s); longer: a disarm (Y)")
    p.add_argument("--resume-hold-max", type=float, default=60.0, metavar="S",
                   help="a hold not resumed within S seconds escalates to a disarm (default %(default)s)")
    p.add_argument("--no-resume", action="store_true", help="every disarm needs Y (the behaviour before the tiers)")
    p.add_argument("--resume-off", default="", metavar="CLASS,...",
                   help="transient classes that disarm instead of holding: input_stale, input_late, input_gap, "
                        "input_invalid, left_inactive, wbc_stale, wbc_disarm, no_command")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "quest_teleop")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .client import Config, ResumeConfig, set_top_speed
    from .recording import RecordConfig
    from .eventlog import DATA_DIR
    from .node import QuestTeleopNode
    from .sim_aids import AidsConfig
    from .server import create_app
    if a.autostart and a.sim in ("none", ""):
        p.error("--autostart is for a sim stack only (--sim)")
    from ...robots import load as load_robot
    robot = load_robot(a.robot)
    arms = [x.name for x in robot.arms]
    hand, ee, hands = hand_mapping(robot.arms, a.hand, a.ee, a.hands)
    if a.arm is None:
        a.arm = ",".join(arms) or "none"
    cfg = Config(hand=hand, ee=ee, hands=hands, head_walk=not a.no_head_walk, mode=a.mode, headless=not a.no_headless,
                 gripper=not a.no_gripper, mirror=a.mirror if hands else "off",
                 hint_source=a.hint_source, hint_tau=a.hint_tau, hint_deadband_m=a.hint_deadband[0],
                 hint_deadband_rad=a.hint_deadband[1], hint_min_confidence=a.hint_min_confidence,
                 elbow_hint=a.elbow_hint, tether=a.tether)
    if a.mirror != "off" and not hands:
        logging.warning("--mirror %s ignored: one-hand teleop", a.mirror)
    if a.elbow_hint and a.body == "off":
        logging.warning("--elbow-hint without --body: no swivel is sent unless the headset already sends body tracking")
    try:
        set_top_speed(cfg, a.max_speed, a.max_omega)
    except ValueError as e:
        p.error(str(e))
    resume = ResumeConfig(enabled=not a.no_resume, grace_s=a.resume_grace, hold_max_s=a.resume_hold_max)
    for c in filter(None, (x.strip() for x in a.resume_off.split(","))):
        if c not in resume.classes:
            p.error(f"--resume-off: unknown class {c!r} (known: {', '.join(resume.classes)})")
        resume.classes[c] = False
    off = lambda v: None if v in ("none", "") else v   # noqa: E731
    node = QuestTeleopNode(a.wbc, arm=off(a.arm), swerve=off(a.swerve), quest_host=a.quest, quest_port=a.quest_port,
                           ssh_host=a.ssh_host, cfg=cfg, stale_s=a.stale_ms / 1e3, max_latency=a.max_latency_ms / 1e3,
                           min_rate=a.min_rate, ar=a.ar, scheme=a.scheme, body=a.body, ctl_timeout=a.ctl_timeout,
                           wait=a.wait, name=a.name, data_dir=a.data_dir or DATA_DIR,
                           log_cap_bytes=int(a.log_cap_mb * 1024 * 1024), sim=off(a.sim),
                           reset_timeout=a.reset_timeout,
                           aids=AidsConfig(ghost=not a.no_ghost, auto_pause=a.auto_pause),
                           autostart=a.autostart, estimator=off(a.estimator),
                           record=None if a.no_record else RecordConfig(out=a.record, robot=a.record_robot,
                                                                        streams=a.record_streams,
                                                                        robot_profile=robot.name),
                           resume=resume)
    serve(create_app(node), a, banner=f"quest teleop node {a.name!r} (WBC {a.wbc}, headset {node.quest_host}, "
                                      f"ssh {a.ssh_host}, AR {a.ar})")


def hand_mapping(arms: list, hand=None, ee=None, hands_spec=None) -> tuple:
    """(hand, ee, hands) from the robot's arms (robots.ArmSpec: name and ``side``; or names) and the options. One arm:
    ``hand`` (right) drives it. Several: two-hand teleop (``hands_spec`` ``left=EE,right=EE``, default each hand drives
    the arm of its ``side`` in the profile; arms without a side: the first left, the last right) unless ``hand`` or
    ``ee`` asks for one hand, which then drives ``ee`` (default the right arm)."""
    sides = {getattr(a, "side", None): getattr(a, "name", a) for a in arms}
    arms = [getattr(a, "name", a) for a in arms]
    if hands_spec:
        hands = dict(kv.split("=", 1) for kv in hands_spec.split(","))
        h = hand or ("right" if "right" in hands else next(iter(hands)))
        return h, hands[h], hands
    if len(arms) <= 1 or hand is not None or ee is not None:
        right = sides.get("right") or (arms[-1] if arms else "arm")
        return hand or "right", ee or (arms[0] if len(arms) == 1 else right), None
    left, right = sides.get("left") or arms[0], sides.get("right") or arms[-1]
    return "right", right, {"left": left, "right": right}


if __name__ == "__main__":
    main()
