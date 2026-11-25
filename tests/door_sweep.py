"""A kinematic check of the revolving door's passage (tests/test_sim_revolving_door.py, docs/sim.md "The revolving door
scene"): how far the rotor must turn between a body leaving the near side and reaching the far side.

The body is a disc of the base's inscribed radius (``FRAME`` / 2, less one grid cell): every pose of the robot covers
it, so where the disc cannot go the robot cannot either. On a grid over the door (its own frame: facade along y, near
side x < 0), for every rotor angle q (1 deg steps) the free cells are those at least that radius from every wall,
wing and the post; their connected components are what the disc can reach without the rotor turning. A body that
leaves the near side at q0 is in the components that touch the near side at q0; as the rotor turns it stays in the
components (of the next angle) that overlap where it could be (a wing sweeping over a cell pushes the body along, it
never jumps a wing: the wing's inflated footprint is wider than a step). The first q at which that set touches the
far side gives the turn; the least over every q0 (one compartment's span, by the rotor's symmetry) is the door's
minimum."""
from __future__ import annotations

import math

import numpy as np

CELL = 0.025                         # m, the grid


def _parts(link: dict) -> list:
    """(kind, centre xy, half extents xy | radius, yaw) of a link's geoms, seen from above."""
    out = []
    for g in link["geoms"]:
        yaw = (g.get("rpy") or [0, 0, 0])[2]
        if g["shape"] == "box":
            out.append(("box", np.array(g["pos"][:2]), np.array(g["size"][:2]) / 2, yaw))
        else:
            out.append(("disc", np.array(g["pos"][:2]), g["radius"], 0.0))
    return out


def _dist(parts: list, pts: np.ndarray, rot: float = 0.0) -> np.ndarray:
    """Distance from each point to the nearest part, the parts turned ``rot`` about the origin."""
    c, s = math.cos(rot), math.sin(rot)
    d = np.full(len(pts), np.inf)
    for kind, ctr, ext, yaw in parts:
        ctr = np.array([c * ctr[0] - s * ctr[1], s * ctr[0] + c * ctr[1]])
        p = pts - ctr
        if kind == "disc":
            d = np.minimum(d, np.maximum(np.hypot(p[:, 0], p[:, 1]) - ext, 0))
            continue
        a = yaw + rot
        lx = np.abs(math.cos(a) * p[:, 0] + math.sin(a) * p[:, 1]) - ext[0]
        ly = np.abs(-math.sin(a) * p[:, 0] + math.cos(a) * p[:, 1]) - ext[1]
        d = np.minimum(d, np.hypot(np.maximum(lx, 0), np.maximum(ly, 0)))
    return d


def sweep(door: dict, radius: float, margin: float = 1.0, step_deg: int = 1, reverse: bool = False) -> dict:
    """The least rotor turn (deg) for a disc of ``radius`` to get from the near side (x < 0, outside the drum) to the
    far side, the rotor turning counterclockwise (``reverse``: clockwise, as if there were no ratchet). ``door``:
    ``scene.revolving_door()``'s fields; the grid spans the drum plus ``margin`` each way (inside the facade's span).
    ``turns``: per start angle q0 (deg, the rotor's), the turn, None where nothing can get in at q0, inf where what
    got in can never get out; ``min``: the least of them."""
    from scipy import ndimage
    from robot_utils.nodes.sim import scene as S
    links = {ln["name"]: ln for ln in door["links"]}
    wing = links["rotor"]["geoms"][1]
    r_in = math.hypot(*wing["pos"][:2]) + wing["size"][0] / 2 + S.WING_GAP      # the drum's inner radius
    wings = len(links["rotor"]["geoms"]) - 1
    half = r_in + S.DOOR_WALL_T + margin
    assert S.DOOR_WALL_T + S.FACADE_L > margin, "the grid must end inside the facade's span"
    xs = np.arange(-half, half + 1e-9, CELL)
    X, Y = np.meshgrid(xs, xs, indexing="ij")
    pts = np.stack([X.ravel(), Y.ravel()], 1)
    rad = radius - CELL
    static = _dist(_parts(links["drum"]), pts) >= rad
    rotor = _parts(links["rotor"])
    outside = np.hypot(X, Y).ravel() > r_in + S.DOOR_WALL_T + 0.05
    near = (outside & (pts[:, 0] < 0)).reshape(X.shape)
    far = (outside & (pts[:, 0] > 0)).reshape(X.shape)
    inner = np.hypot(X, Y) < r_in - radius        # past the doorway, in a compartment
    period = 360 // wings
    labels = []
    for k in range(360):
        free = (static & (_dist(rotor, pts, math.radians(k)) >= rad)).reshape(X.shape)
        lab, _ = ndimage.label(free & ~near)
        labels.append((lab, free))
    turns = []
    sgn = -1 if reverse else 1
    for q0 in range(0, period, step_deg):
        lab, free = labels[q0 % 360]
        seed, _ = ndimage.label(free)
        ids = np.unique(seed[near & free])
        reach = np.isin(seed, ids[ids > 0])
        if not (reach & inner).any():
            turns.append(None)                  # no compartment open to the near side at q0: nothing gets in
            continue
        turn = math.inf                         # in, but never out (stuck)
        for k in range(0, 361):
            lab, free = labels[(q0 + sgn * k) % 360]
            if k:
                ids = np.unique(lab[reach])
                reach = np.isin(lab, ids[ids > 0])
            if (reach & far).any():
                turn = k
                break
        turns.append(turn)
    known = [t for t in turns if t is not None]
    return {"min": min(known) if known else None, "turns": turns, "cells": X.size}
