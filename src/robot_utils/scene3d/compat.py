"""A rigid scene for the Quest AR view as it is today (quest_env ROBOT_AR.md), no headset update needed.

The view draws a description (links of primitive visuals, placed by forward kinematics from the line's ``base``)
and robot_state.v1 lines whose ``objects`` are rigid bodies (a world pose and primitive visuals). A rigid scene maps
onto that as:

- **model**: just the root link ``world`` with nothing to draw (its ``model_version`` is the ``scene_id``: the view
  fetches the model again when a line's changes). Static geometry cannot live in the model: the view puts the
  model's root at the line's ``base``, which is the robot's base here (markers in the base frame need it);
- **state lines**: every body as an object, the static ones at their fixed pose (their visuals are a few hundred
  bytes per line: fine for a prototype, the reason for the native format).

The view draws primitives only (box, sphere, cylinder, capsule): an ellipsoid becomes the box around it, a mesh the
box around its vertices (``bounds``, in the mesh frame), a plane is left out (the view marks the floor itself).
``role`` picks the geoms: ``collision`` (default: what the physics touches) or ``visual``; a body without any of
that role falls back to its others.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .format import Frame
from .pose import compose, qmul, qrot  # noqa: F401 (re-exported: compat.compose)


def _pick(geoms: list, role: str) -> list:
    want = {"collision": ("collision", "both"), "visual": ("visual", "both")}[role]
    mine = [g for g in geoms if g["role"] in want]
    return mine or geoms


def visual(g: dict, index: dict, pose=None) -> Optional[dict]:
    """One geom as the view's visual (``pose``: put it in that frame instead of its body's); None if it has none."""
    local = np.array(g["pos"] + g["quat"], float)
    t = g["type"]
    if t == "plane":
        return None
    if t == "mesh":
        lo, hi = (np.array(b, float) for b in index["meshes"][g["mesh"]]["bounds"])
        local = compose(local, np.concatenate([(lo + hi) / 2, [0, 0, 0, 1]]))
        out = {"type": "box", "half_size": [round(float(x), 6) for x in np.maximum((hi - lo) / 2, 1e-4)]}
    elif t == "box":
        out = {"type": "box", "half_size": g["half_extents"]}
    elif t == "ellipsoid":
        out = {"type": "box", "half_size": g["radii"]}
    elif t == "sphere":
        out = {"type": "sphere", "radius": g["radius"]}
    else:
        out = {"type": t, "radius": g["radius"], "length": round(2 * g["half_length"], 6)}
    if pose is not None:
        local = compose(pose, local)
    out["origin"] = {"xyz": [round(float(x), 6) for x in local[:3]], "quat": [round(float(x), 7) for x in local[3:]]}
    out["rgba"] = g["rgba"]
    return out


def model(index: dict) -> dict:
    """The view's description: the root link ``world`` alone (every body comes in the state lines)."""
    return {"model_version": index["scene_id"], "root": "world", "links": {"world": {"visuals": []}}, "joints": []}


class StateLines:
    """robot_state.v1-shaped lines from frames: every body as an object (visuals computed once per scene)."""

    def __init__(self, index: dict, role: str = "collision"):
        self.scene_id = index["scene_id"]
        self.names, self.geoms, self.static = [], [], {}
        for i in index["dynamic"]:
            b = index["bodies"][i]
            self.names.append(b["name"])
            self.geoms.append([v for v in (visual(g, index) for g in _pick(b["geoms"], role)) if v is not None])
        for b in index["bodies"]:
            if not b["dynamic"]:
                geoms = [v for v in (visual(g, index) for g in _pick(b["geoms"], role)) if v is not None]
                if geoms:
                    self.static[b["name"]] = {"pose": b["pose"], "geom": geoms}

    def line(self, f: Frame, base=None, overlays: Optional[dict] = None) -> dict:
        """The frame as a line; a colour override recolours the body's visuals (alpha 0: left out)."""
        objects = dict(self.static)
        for slot, (name, geoms, p) in enumerate(zip(self.names, self.geoms, f.poses)):
            if not geoms:
                continue
            c = f.colors.get(slot)
            if c is not None:
                if c[3] <= 0:
                    continue
                geoms = [{**g, "rgba": [round(float(x), 3) for x in c]} for g in geoms]
            objects[name] = {"pose": [round(float(x), 6) for x in p], "geom": geoms}
        out = {"schema": "robot_state.v1", "source": "rigid_scene", "seq": f.seq, "t": round(f.t, 4),
               "reset_seq": f.reset_seq, "model_version": self.scene_id, "frame": "world", "objects": objects}
        if base is not None:
            out["base"] = [round(float(x), 6) for x in base]
        if overlays:
            out["overlays"] = overlays
        return out
