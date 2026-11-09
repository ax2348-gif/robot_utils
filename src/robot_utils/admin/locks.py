"""Lockout / tagout on the robot (common/locks.py): chock actuators, cover cameras. Run on the robot.

    python -m robot_utils.admin.locks status
    python -m robot_utils.admin.locks chock swerve --reason "battery swap"     # or: arm, <instance>, all
    python -m robot_utils.admin.locks cover camera --reason "privacy: meeting"  # or: <instance>, all
    python -m robot_utils.admin.locks remove <id>          # remove one lock (anyone with SSH is an admin)
    python -m robot_utils.admin.locks remove --all [--kind chock|cover]

A chocked node stops at once and refuses to enable until every chock on it is removed; a covered
camera captures nothing and serves procedural frames. Anyone can also place locks from the landing
page (rate-limited); only SSH removes them.
"""
from __future__ import annotations

import argparse
import getpass
import os
import socket
import sys
import time

from ..nodes.common.lease import RequestError
from ..nodes.common.locks import LockStore


def who_am_i() -> str:
    user = os.environ.get("SUDO_USER") or getpass.getuser()
    client = (os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_CLIENT") or "").split()
    if not client:
        return f"{user}@{socket.gethostname()} (console)"
    ip = client[0]
    try:
        from ..frontend.observe import Identities
        label = Identities().get(ip)["label"]
    except Exception:
        label = ip
    return f"{user} via ssh from {label}"


def fmt(l) -> str:
    age = time.time() - l.placed
    ago = f"{age / 60:.0f} min" if age < 7200 else f"{age / 3600:.1f} h"
    return f"  {l.id:32s} {l.kind:5s} {l.target:14s} by {l.by}  ({ago} ago, {l.source})" + \
        (f"\n  {'':32s} reason: {l.reason}" if l.reason else "")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.admin.locks", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    for k in ("chock", "cover"):
        p = sub.add_parser(k)
        p.add_argument("target", help="a node kind (swerve, arm, camera), an instance name, or 'all'")
        p.add_argument("--reason", default="", help="why (shown to everyone: the tag)")
        p.add_argument("--as", dest="by", default=None, help="your name (default: login and SSH client)")
    r = sub.add_parser("remove")
    r.add_argument("id", nargs="?")
    r.add_argument("--all", action="store_true")
    r.add_argument("--kind", choices=["chock", "cover"])
    a = ap.parse_args(argv)
    store = LockStore()
    try:
        if a.cmd == "status":
            locks = store.active(force=True)
            print(f"{len(locks)} lock(s) in {store.path}" + (":" if locks else ""))
            for l in locks:
                print(fmt(l))
        elif a.cmd in ("chock", "cover"):
            lock, _ = store.place(a.cmd, a.target, a.by or who_am_i(), a.reason, source="ssh")
            print(f"placed:\n{fmt(lock)}")
        else:
            if a.all:
                gone = store.remove_all(a.kind)
                print(f"removed {len(gone)} lock(s)")
                for l in gone:
                    print(fmt(l))
            elif a.id:
                print(f"removed:\n{fmt(store.remove(a.id))}")
            else:
                ap.error("remove needs a lock id or --all")
            left = store.active(force=True)
            print(f"{len(left)} lock(s) left")
        return 0
    except RequestError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
