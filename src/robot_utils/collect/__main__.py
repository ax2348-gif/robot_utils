"""CLI: ``python -m robot_utils.collect run|cut`` (docs/collect.md)."""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from .cut import cut_session
from .session import Session
from .streams import DEFAULT_STREAMS, OPLOG_SPEC


def _size(s: str) -> int:
    s = s.strip().upper()
    for suffix, mult in (("GB", 1 << 30), ("G", 1 << 30), ("MB", 1 << 20), ("M", 1 << 20), ("KB", 1 << 10),
                         ("K", 1 << 10)):
        if s.endswith(suffix):
            return int(float(s[:-len(suffix)]) * mult)
    return int(s)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m robot_utils.collect", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="record a session (Ctrl-C stops)")
    r.add_argument("--robot", required=True, help="the robot's frontend, e.g. http://botcore")
    r.add_argument("--out", required=True, help="parent directory; the session is <out>/<YYYYMMDD-HHMMSS>")
    r.add_argument("--task", required=True, help="task text of the next episodes (POST quest_teleop/task)")
    r.add_argument("--streams", default=",".join(DEFAULT_STREAMS), help="comma-separated stream specs")
    r.add_argument("--teleop", default="quest_teleop",
                   help="the quest_teleop node instance (its /task, /start, /stop, /eventlog; the default streams' "
                        "operator stream becomes <teleop>/oplog/stream)")
    r.add_argument("--batch-ms", type=float, default=50.0,
                   help="the row streams (<node>/stream: arm, swerve, estimator, wbc) in batches this long: the same "
                        "rows and timestamps, far fewer records and packets on the robot (0 = every row on its own, "
                        "e.g. for a policy run that consumes them live)")
    r.add_argument("--start-teleop", action="store_true", help="POST quest_teleop/start after the task")
    r.add_argument("--stop-teleop", action="store_true", help="POST quest_teleop/stop on Ctrl-C")
    r.add_argument("--rotate", type=_size, default=512 << 20, help="rotate each stream's file at this size")
    r.add_argument("--read-timeout", type=float, default=10.0, help="s without bytes before reconnecting")
    r.add_argument("--status-every", type=float, default=5.0, help="status line period, s")
    r.add_argument("--settle", type=float, default=1.0, help="s to keep recording after Ctrl-C (last events)")
    r.add_argument("--name", help="session directory name (default: the local time)")
    r.add_argument("-v", "--verbose", action="store_true")
    c = sub.add_parser("cut", help="cut a session into episodes/<id>/")
    c.add_argument("session")
    c.add_argument("--out", help="default: <session>/episodes")
    c.add_argument("--pre-roll", type=float, default=0.0, help="s before t_start")
    c.add_argument("--post-roll", type=float, default=0.0, help="s after t_stop")
    c.add_argument("--episodes", help="comma-separated ids (default: all)")
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if getattr(a, "verbose", False) else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    if a.cmd == "run":
        streams = [x for x in a.streams.split(",") if x.strip()]
        if a.teleop != "quest_teleop":
            streams = [f"{a.teleop}/oplog/stream" if x.strip() == OPLOG_SPEC else x for x in streams]
        s = Session(a.robot, a.out, a.task, streams, teleop=a.teleop, batch_ms=a.batch_ms,
                    start_teleop=a.start_teleop, stop_teleop=a.stop_teleop, rotate_bytes=a.rotate,
                    read_timeout=a.read_timeout, status_every=a.status_every, settle=a.settle,
                    session_name=a.name)
        try:
            asyncio.run(s.run())
        except RuntimeError as e:
            print(f"collect: {e}", file=sys.stderr)
            return 1
        return 0
    eps = cut_session(a.session, a.out, a.pre_roll, a.post_roll,
                      [x for x in a.episodes.split(",")] if a.episodes else None)
    for e in eps:
        counts = ", ".join(f"{k} {v['records']}" for k, v in e["streams"].items())
        ngaps = sum(len(g) for g in e["gaps"].values())
        print(f"{e['id']} {e['outcome']}{' (' + e['reason'] + ')' if e['reason'] else ''}: "
              f"{e['t_stop'] - e['t_start']:.1f} s; {counts}" + (f"; {ngaps} gaps" if ngaps else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
