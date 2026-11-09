"""Chat commands for humans on the page: the same calls an agent makes, typed as one line.

    look [scale] [grid]            region <shape> [exclude <shape>...] [on <snapshot>]
    plane <shape> [exclude ...]    approach <query> [standoff]
    check <x> <y> [theta] [frame]  goal <x> <y> [theta] [frame] [key=value ...]
    robot                          stop | enable | disable | release | estop [reason]
    note <text>                    place <name> [x y theta [frame]]
    help

Shapes: ``point u v`` | ``disk u v r`` | ``box u0 v0 u1 v1`` | ``poly u,v u,v u,v ...`` (colour
image pixels). Angles in degrees when written with a trailing ``d`` (``90d``), else radians.
Frames: local (default) | global | snapshot:<id> | place:<name>.
"""
from __future__ import annotations

import math

HELP = __doc__.split("\n\n", 1)[1].strip()


def _num(s: str) -> float:
    if s.endswith("d"):
        return math.radians(float(s[:-1]))
    return float(s)


def _shape(tok: list, i: int) -> tuple[dict, int]:
    k = tok[i]
    if k == "point":
        return {"point": [float(tok[i + 1]), float(tok[i + 2])]}, i + 3
    if k == "disk":
        return {"disk": [float(tok[i + 1]), float(tok[i + 2]), float(tok[i + 3])]}, i + 4
    if k == "box":
        return {"box": [float(x) for x in tok[i + 1:i + 5]]}, i + 5
    if k in ("poly", "polygon"):
        pts, j = [], i + 1
        while j < len(tok) and "," in tok[j]:
            u, v = tok[j].split(",")
            pts.append([float(u), float(v)])
            j += 1
        return {"polygon": pts}, j
    raise ValueError(f"unknown shape {k!r} (point | disk | box | poly)")


def parse(text: str) -> tuple[str, dict]:
    """One command line -> (node method, kwargs). Raises ValueError with a hint."""
    tok = text.strip().split()
    if not tok:
        raise ValueError("empty command; try 'help'")
    cmd, a = tok[0].lower(), tok[1:]
    if cmd == "help":
        return "help", {}
    if cmd == "robot":
        return "describe_robot", {}
    if cmd == "look":
        kw = {"grid": "grid" in a, "scale": None}
        for t in a:
            if t not in ("grid",):
                kw["scale"] = float(t)
        return "look", kw
    if cmd in ("region", "plane"):
        sel, i = _shape(a, 0)
        exclude, snap = [], None
        while i < len(a):
            if a[i] == "exclude":
                e, i = _shape(a, i + 1)
                exclude.append(e)
            elif a[i] == "on":
                snap, i = a[i + 1], i + 2
            else:
                raise ValueError(f"unexpected {a[i]!r}")
        return "query", {"kind": cmd, "snapshot": snap, "select": sel, "exclude": exclude}
    if cmd == "approach":
        return "plan_approach", {"query": a[0], "standoff": float(a[1]) if len(a) > 1 else 1.0, "face": True}
    if cmd in ("check", "goal"):
        nums, frame, opts = [], "local", {}
        for t in a:
            if "=" in t:
                k, v = t.split("=", 1)
                try:
                    opts[k] = float(v)
                except ValueError:
                    opts[k] = v
            elif t in ("local", "global") or t.startswith(("snapshot:", "place:")):
                frame = t
            else:
                nums.append(_num(t))
        while len(nums) < 3:
            nums.append(0.0)
        x, y, th = nums[:3]
        if cmd == "check":
            return "plan_check", {"frame": frame, "x": x, "y": y, "theta": th}
        return "goal", {"frame": frame, "x": x, "y": y, "theta": th, "options": opts}
    if cmd in ("stop", "enable", "disable", "release"):
        return cmd, {}
    if cmd == "estop":
        return "estop", {"reason": " ".join(a)}
    if cmd == "note":
        return "note", {"text": " ".join(a)}
    if cmd == "place":
        if len(a) == 1:
            return "place", {"name": a[0], "frame": "local", "x": 0.0, "y": 0.0, "theta": 0.0}
        nums = [_num(t) for t in a[1:4]]
        frame = a[4] if len(a) > 4 else "local"
        return "place", {"name": a[0], "frame": frame, "x": nums[0], "y": nums[1],
                         "theta": nums[2] if len(nums) > 2 else None}
    raise ValueError(f"unknown command {cmd!r}; try 'help'")
