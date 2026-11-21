"""The robot's kinematic tree as plain JSON (``GET /model/tree``, schema ``robot_tree.v1``): what the assembled URDF
encodes, for clients that compute their own kinematics without a URDF parser (the Quest's AR view, web pages).

Read straight from the URDF text that ``GET /model/urdf`` serves with the same options, so the two never disagree:

- ``links``: ``{name: {"visuals": [{"type": box|cylinder|sphere|mesh, size params, "pos", "quat", "rgba", "mesh"}]}}``
  (box ``size`` = full extents; cylinder ``radius`` + ``length`` along its z; sphere ``radius``; mesh ``url`` +
  ``scale``), each visual's pose in its link;
- ``joints``: ``[{name, type, parent, child, pos, quat, axis, lower, upper, velocity, effort, mimic}]`` in tree
  order (parents first); ``pos`` + ``quat`` = the child frame in the parent at q = 0;
- ``root``; ``model_version``.

Conventions: metres, radians; quaternions ``[x, y, z, w]``; a joint's child pose in its parent is
``T(pos, quat) * motion(axis, q)`` with motion a rotation about ``axis`` by q (revolute, continuous) or a
translation along it by q (prismatic); a mimic joint's q = multiplier * q(joint) + offset.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Optional

import numpy as np

SCHEMA = "robot_tree.v1"
MOVABLE = ("revolute", "continuous", "prismatic")


def _floats(s: Optional[str], n: int, default: float = 0.0) -> list:
    v = [float(x) for x in (s or "").split()]
    return v if len(v) == n else [default] * n


def rpy_quat(r: float, p: float, y: float) -> list:
    """URDF rpy (fixed X, Y, Z) -> quaternion [x, y, z, w]."""
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def _origin(el) -> tuple[list, list]:
    o = el.find("origin") if el is not None else None
    xyz = _floats(o.get("xyz") if o is not None else None, 3)
    rpy = _floats(o.get("rpy") if o is not None else None, 3)
    return [round(v, 9) for v in xyz], [round(v, 9) for v in rpy_quat(*rpy)]


def _rgba(vis, materials: dict) -> list:
    m = vis.find("material")
    if m is None:
        return [0.7, 0.7, 0.7, 1.0]
    c = m.find("color")
    if c is not None and c.get("rgba"):
        return _floats(c.get("rgba"), 4, 1.0)
    return materials.get(m.get("name"), [0.7, 0.7, 0.7, 1.0])


def tree(urdf: str, model_version: Optional[str] = None, options: Optional[dict] = None) -> dict:
    robot = ET.fromstring(urdf)
    materials = {}
    for m in robot.findall("material"):
        c = m.find("color")
        if c is not None and c.get("rgba"):
            materials[m.get("name")] = _floats(c.get("rgba"), 4, 1.0)
    links = {}
    for lk in robot.findall("link"):
        visuals = []
        for vis in lk.findall("visual"):
            geo = vis.find("geometry")
            if geo is None or not len(geo):
                continue
            g = geo[0]
            pos, quat = _origin(vis)
            out = {"type": g.tag, **({"name": vis.get("name")} if vis.get("name") else {}), "pos": pos, "quat": quat,
                   "rgba": _rgba(vis, materials)}
            if g.tag == "box":
                out["size"] = _floats(g.get("size"), 3)
            elif g.tag == "cylinder":
                out["radius"], out["length"] = float(g.get("radius")), float(g.get("length"))
            elif g.tag == "sphere":
                out["radius"] = float(g.get("radius"))
            elif g.tag == "mesh":
                out["url"] = g.get("filename")
                out["scale"] = _floats(g.get("scale"), 3, 1.0) if g.get("scale") else [1.0, 1.0, 1.0]
            else:
                continue
            visuals.append(out)
        links[lk.get("name")] = {"visuals": visuals}
    raw = []
    for j in robot.findall("joint"):
        pos, quat = _origin(j)
        ax = j.find("axis")
        lim = j.find("limit")
        mim = j.find("mimic")
        out = {"name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
               "child": j.find("child").get("link"), "pos": pos, "quat": quat,
               "axis": _floats(ax.get("xyz") if ax is not None else None, 3) if ax is not None else [1.0, 0.0, 0.0]}
        if lim is not None:
            for k in ("lower", "upper", "velocity", "effort"):
                if lim.get(k) is not None:
                    out[k] = float(lim.get(k))
        if mim is not None:
            out["mimic"] = {"joint": mim.get("joint"), "multiplier": float(mim.get("multiplier", 1.0)),
                            "offset": float(mim.get("offset", 0.0))}
        raw.append(out)
    children = {j["child"] for j in raw}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise ValueError(f"the URDF has {len(roots)} root links: {roots}")
    order, frontier = [], [roots[0]]                  # parents first
    by_parent: dict = {}
    for j in raw:
        by_parent.setdefault(j["parent"], []).append(j)
    while frontier:
        p = frontier.pop(0)
        for j in by_parent.get(p, []):
            order.append(j)
            frontier.append(j["child"])
    return {"schema": SCHEMA, "model_version": model_version, "root": roots[0], "options": options or {},
            "links": links, "joints": order, "movable": [j["name"] for j in order if j["type"] in MOVABLE],
            "units": {"length": "m", "angle": "rad", "quat": "x, y, z, w"}}


# -- forward kinematics from the tree (the reference a client implements; tests compare it with pinocchio) --------

def _quat_R(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _axis_R(axis, a: float) -> np.ndarray:
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def fk(t: dict, q: dict, root_T: Optional[np.ndarray] = None) -> dict:
    """{link: 4x4 pose in the root's parent frame} for joint positions ``q`` ({name: value}; missing = 0)."""
    out = {t["root"]: np.eye(4) if root_T is None else np.asarray(root_T, float)}
    byname = {j["name"]: j for j in t["joints"]}
    for j in t["joints"]:
        v = q.get(j["name"], 0.0)
        if "mimic" in j and j["mimic"]["joint"] in byname:
            v = j["mimic"]["multiplier"] * q.get(j["mimic"]["joint"], 0.0) + j["mimic"]["offset"]
        T = np.eye(4)
        T[:3, :3] = _quat_R(j["quat"])
        T[:3, 3] = j["pos"]
        M = np.eye(4)
        if j["type"] in ("revolute", "continuous"):
            M[:3, :3] = _axis_R(j["axis"], v)
        elif j["type"] == "prismatic":
            M[:3, 3] = np.asarray(j["axis"], float) / np.linalg.norm(j["axis"]) * v
        out[j["child"]] = out[j["parent"]] @ T @ M
    return out
