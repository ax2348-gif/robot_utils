"""Robot-wide page theme (frontend/themes.py). Run on the robot; viewers can still pick their own.

    python -m robot_utils.admin.theme list
    python -m robot_utils.admin.theme default <name>     # robot-wide default for everyone who hasn't chosen
    python -m robot_utils.admin.theme default --clear    # back to the built-in look

Themes come from the repo (src/robot_utils/frontend/themes/) and ~/.config/robot_utils/themes/
(admin-added; same name overrides). The landing page picks up a change on the next page load.
"""
from __future__ import annotations

import argparse
import sys

from ..frontend.themes import Themes


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.admin.theme", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    d = sub.add_parser("default")
    d.add_argument("name", nargs="?")
    d.add_argument("--clear", action="store_true")
    a = ap.parse_args(argv)
    t = Themes()
    if a.cmd == "list":
        cur = t.default()
        print(f"robot-wide default: {cur or 'built-in look'}")
        print(f"  {'builtin':16s} the original look (always available)")
        for th in t.list():
            mark = "*" if th["name"] == cur else " "
            print(f"{mark} {th['name']:16s} {th['title']}  [{th['source']}]" + (f"\n  {'':16s} {th['description']}" if th["description"] else ""))
        return 0
    if a.clear or a.name in (None, "builtin"):
        if not a.clear and a.name is None:
            ap.error("give a theme name, or --clear")
        t.set_default(None)
        print("robot-wide default: built-in look")
        return 0
    try:
        t.set_default(a.name)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(f"robot-wide default: {a.name} (viewers who chose their own keep it)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
