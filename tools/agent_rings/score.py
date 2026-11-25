"""The rings task's score, from the simulator's ground truth: privileged, run by the harness outside the agent's view
(tools/agent_rings/README.md).

The task: every red object on the **outer** ring, every blue one on the **inner** ring. An object counts as on a ring
when it rests on its top surface within its radii; anything else (on the floor, in the hollow, in the gripper, off
the rings) is "off". Score = objects on their right ring; at the start about half are (each ring starts half red,
half blue).

    python tools/agent_rings/score.py --sim unix:/tmp/ar/robot/sim.sock --every 10 --out SCORES.jsonl

One JSON line per sample: wall time, sim time, reset_seq, counts (right / wrong / off per colour) and the score.
"""
from __future__ import annotations

import argparse
import json
import math
import time

import httpx

# ring geometry (scene.py rings(): measured from the scene, re-read from it at start)
SURFACE_TOL = 0.03          # m: an object's centre within this of the ring top + its half height counts as resting


def ring_geometry(scene: dict) -> dict:
    rings = next(o for o in scene["objects"] if o["name"] == "rings")
    cx, cy = rings["xyz"][0], rings["xyz"][1]
    radii = {}
    for ln in rings["links"]:
        nm = ln["name"]
        if not (nm.startswith("outer") or nm.startswith("inner")):
            continue
        for g in ln["geoms"]:
            r = math.hypot(*g["pos"][:2])
            half = g["size"][0] / 2 if g["shape"] == "box" else 0.0
            key = "outer" if nm.startswith("outer") else "inner"
            lo, hi, top = radii.get(key, (math.inf, -math.inf, -math.inf))
            radii[key] = (min(lo, r - half), max(hi, r + half),
                          max(top, g["pos"][2] + (g["size"][2] / 2 if g["shape"] == "box" else 0)))
    return {"cx": cx, "cy": cy, "radii": radii}


def colour(o: dict) -> str:
    r, g, b = o["rgba"][:3]
    return "red" if r > b else "blue"


def classify(o: dict, geo: dict) -> str:
    x, y, z = o["xyz"]
    r = math.hypot(x - geo["cx"], y - geo["cy"])
    for ring, (lo, hi, top) in geo["radii"].items():
        if lo <= r <= hi and abs(z - top) < 0.06 + SURFACE_TOL:
            return ring
    return "off"


def score(scene: dict, geo: dict) -> dict:
    counts = {c: {"right": 0, "wrong": 0, "off": 0} for c in ("red", "blue")}
    for o in scene["objects"]:
        if o["name"] == "rings" or o.get("shape") == "articulated":
            continue
        c, where = colour(o), classify(o, geo)
        want = "outer" if c == "red" else "inner"
        counts[c]["off" if where == "off" else ("right" if where == want else "wrong")] += 1
    total = sum(sum(v.values()) for v in counts.values())
    right = counts["red"]["right"] + counts["blue"]["right"]
    return {"counts": counts, "right": right, "total": total}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", required=True, help="unix:/path/sim.sock or http://host:port/nodes/sim")
    p.add_argument("--every", type=float, default=10.0)
    p.add_argument("--out", required=True)
    a = p.parse_args()
    if a.sim.startswith("unix:"):
        client = httpx.Client(transport=httpx.HTTPTransport(uds=a.sim[5:]), base_url="http://sim", timeout=10)
    else:
        client = httpx.Client(base_url=a.sim.rstrip("/"), timeout=10)
    geo = None
    with open(a.out, "a", buffering=1) as out:
        while True:
            try:
                scene = client.get("/scene").json()
                st = client.get("/state").json()
                if geo is None:
                    geo = ring_geometry(scene)
                s = score(scene, geo)
                rec = {"t": round(time.time(), 1), "t_sim": st.get("t"), "reset_seq": st.get("reset_seq"), **s}
                out.write(json.dumps(rec, separators=(",", ":")) + "\n")
                print(f"{time.strftime('%H:%M:%S')}  score {s['right']}/{s['total']}  "
                      f"red {s['counts']['red']}  blue {s['counts']['blue']}", flush=True)
            except Exception as e:  # noqa: BLE001 (the sim restarting: keep going)
                print(f"{time.strftime('%H:%M:%S')}  score: {type(e).__name__}: {e}", flush=True)
            time.sleep(a.every)


if __name__ == "__main__":
    main()
