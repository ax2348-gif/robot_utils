#!/usr/bin/env python3
"""Quest whole-body teleop through the WBC node (runs on the Mac or erid, never the Pi).

    python tools/quest_teleop/teleop.py --quest 100.72.77.83 --wbc http://botcore-desktop.local/nodes/wbc
    python tools/quest_teleop/teleop.py --quest 127.0.0.1 --wbc http://127.0.0.1:8150   # fake headset + sim

Reads the headset's ``teleop.quest_state.v1`` stream (TCP 9391), applies the remote-consumer rules and the
body-frozen scheme (README.md), and drives the WBC: lease, hints, enable, a datagram session, one target datagram
per batch of new headset frames. On stale or invalid input it stops sending (the WBC holds, then stops the base).
Ctrl-C: stop sending, disable, release the lease.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

if __package__ in (None, ""):
    _here = Path(__file__).resolve()
    sys.path.insert(0, str(_here.parent.parent))
    try:
        import robot_utils  # noqa: F401
    except ImportError:
        sys.path.insert(0, str(_here.parents[2] / "src"))
    from quest_teleop.body_frozen import Config, RobotState, TeleopCore   # noqa: E402
    from quest_teleop.quest_stream import DEFAULT_PORT, STALE_S, QuestGate, QuestReader   # noqa: E402
    from quest_teleop.wbc_client import WbcClient, WbcStream   # noqa: E402
else:
    from .body_frozen import Config, RobotState, TeleopCore
    from .quest_stream import DEFAULT_PORT, STALE_S, QuestGate, QuestReader
    from .wbc_client import WbcClient, WbcStream


def default_hints(cfg: Config) -> dict:
    """Session defaults set over HTTP at start; each datagram overrides ``base`` with the live state."""
    return {"base": {"mode": "locked"},
            "ees": {cfg.ee: {"weights": {"position": 1.0, "orientation": 0.3}}}}


class TeleopClient:
    """The loop: headset frames in, one datagram out per batch of new frames."""

    def __init__(self, cfg: Config, reader: QuestReader, stream: WbcStream, wbc: WbcClient, *,
                 stale_s: float = STALE_S, clock: Callable[[], float] = time.monotonic):
        self.cfg = cfg
        self.reader, self.stream, self.wbc = reader, stream, wbc
        self.gate = QuestGate(stale_s)
        self.core = TeleopCore(cfg)
        self.clock = clock
        self.sending = False
        self.sent_log: Optional[list] = None   # tests: every datagram sent

    def robot(self) -> Optional[RobotState]:
        row, t = self.stream.latest()
        return None if row is None else RobotState.from_row(row, t, self.cfg.ee)

    def step(self, wait: float = 0.05) -> Optional[dict]:
        first = self.reader.get(timeout=wait)
        frames = ([first] + self.reader.drain()) if first is not None else []
        robot = self.robot()
        for f in frames:
            self.core.on_frame(f.data, self.gate.check(f), robot, self.clock())
        now = self.clock()
        cmd = self.core.command(now, self.gate.fresh(now), robot)
        msg = None
        if cmd is None:
            self.sending = False
        elif frames:          # at the stream rate
            msg = self.wbc.send_alive() if cmd == "alive" else self.wbc.send_target(cmd)
            self.sending = True
            if self.sent_log is not None:
                self.sent_log.append(msg)
        return msg

    def run(self, stop: threading.Event, status: Optional[Callable[["TeleopClient"], None]] = None,
            status_period: float = 0.2) -> None:
        next_status = 0.0
        while not stop.is_set():
            self.step()
            if status is not None and self.clock() >= next_status:
                next_status = self.clock() + status_period
                status(self)


def open_wbc(wbc: WbcClient, cfg: Config) -> dict:
    """Lease (renewed in the background), session-default hints, enable, a datagram session."""
    wbc.acquire()
    wbc.set_hints(default_hints(cfg))
    wbc.enable()
    return wbc.open_session()


def close_wbc(wbc: WbcClient, log: Callable[[str], None] = print) -> None:
    """Disable and release, each attempted even if the other fails."""
    for f in (wbc.disable, wbc.release):
        try:
            f()
        except Exception as e:  # noqa: BLE001
            log(f"  {f.__name__}: {e}")


def status_line(c: TeleopClient) -> str:
    """Engagement state, sending, input ages, the stick offset (world x, y, yaw) and the base hint sent."""
    s = c.core.status
    _, t = c.stream.latest()
    age = f"{(c.clock() - t) * 1e3:4.0f}ms" if t is not None else "  none"
    quest = "ok" if c.gate.fresh(c.clock()) else "--"
    o, h = c.core.offset, c.core.hint
    hint = f"({h['pose']['x']:+.2f},{h['pose']['y']:+.2f},{h['heading']['yaw']:+.2f})" if h else "none"
    return (f"[{s.state:13}] {'SENDING' if c.sending else 'silent '} quest={quest} wbc_row={age}"
            f" eng={s.engagements} sent={c.wbc.sent.get('target', 0)}"
            f" offset=({o[0]:+.2f},{o[1]:+.2f},{o[2]:+.2f}) hint={hint} {s.reason[:32]:32}")


def print_status(c: TeleopClient) -> None:
    print("\r" + status_line(c), end="", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quest", default="100.72.77.83", help="headset address (default %(default)s)")
    ap.add_argument("--quest-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--wbc", default="http://botcore-desktop.local/nodes/wbc",
                    help="WBC node base URL: robot LAN (default), the erid proxy, or the local sim stack")
    ap.add_argument("--udp-host", default=None, help="datagram destination host (default: the --wbc host)")
    ap.add_argument("--hand", choices=("left", "right"), default="right", help="the hand that drives the arm")
    ap.add_argument("--ee", default="arm", help="end-effector name in the WBC (default %(default)s)")
    ap.add_argument("--no-head-walk", action="store_true",
                    help="engaged: the base hint follows only the stick offset, not the head's travel and yaw")
    ap.add_argument("--stale-ms", type=float, default=STALE_S * 1e3, help="headset freshness timeout")
    ap.add_argument("--no-gripper", action="store_true")
    args = ap.parse_args(argv)

    cfg = Config(hand=args.hand, ee=args.ee, head_walk=not args.no_head_walk, gripper=not args.no_gripper)
    wbc = WbcClient(args.wbc, udp_host=args.udp_host)
    stream = WbcStream(args.wbc).start()
    reader = QuestReader(args.quest, args.quest_port).start()
    client = TeleopClient(cfg, reader, stream, wbc, stale_s=args.stale_ms / 1e3)
    stop = threading.Event()
    try:
        info = open_wbc(wbc, cfg)
        print(f"WBC session {info['session']} -> udp {wbc.udp_host}:{wbc.udp_port}; headset {args.quest}:"
              f"{args.quest_port}, {args.hand} grip clutches the arm. Ctrl-C to stop.", flush=True)
        client.run(stop, print_status)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print("\nstopping: disable, release", flush=True)
        close_wbc(wbc)
        reader.close()
        stream.close()
        wbc.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
