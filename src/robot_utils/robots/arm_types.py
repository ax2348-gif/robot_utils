"""Arm types: what the arm node, the sim and the WBC need of an arm that is not the reBot (docs/plans/
bimanual_and_robot_swap.md, "the arm type seam").

``rebot`` (the default) keeps every existing path: rebot_ctl's hardware config, profiles, limits, motor models.
Another type is a YAML next to its URDFs (``robots/<package>/arm_type.yaml``, e.g. ``openarm_v2``): its DOF, MIT gains,
gravity scale, torque limits, the arm node's speed / acceleration caps; the gripper is the reBot's (its constants and
gains). The joint limits come from the arm's URDF.

The arm node runs one arm per process: its ``__main__`` puts the arm's type in the environment (``ENV``, JSON) before
importing the rest, so arm/config.py (``N``, the profile, the limits) and its spawned control process see it. The sim
and the WBC serve several arms in one process: they ask ``load(name)`` per arm.
"""
from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from functools import lru_cache
from importlib import resources
from typing import Optional

import yaml

REBOT = "rebot"
ENV = "ROBOT_ARM_TYPE"
TYPES = {"openarm_v2": "openarm/arm_type.yaml"}


@dataclass(frozen=True)
class ArmType:
    name: str
    dof: int
    kp: tuple
    kd: tuple
    tau_scale: tuple
    torque_max: tuple
    gripper_torque_max: float
    joint_vel: tuple
    joint_acc: tuple
    g_kp: float = 1.0           # the gripper's MIT gains (the reBot soft profile's, filled by load)
    g_kd: float = 0.05
    urdf: Optional[str] = None  # the arm's URDF file (set per arm: left / right)
    # a redundant arm's human-like geometry (wbc/humanlike.py): its shoulder / elbow / wrist points (indices into
    # the arm's joints: the joint origins), the joints a comfort term centres, the link capsules of the arm-proximity
    # term (upper arm and forearm only)
    geometry: Optional[dict] = None
    jam_bands: tuple = ()       # the arm node's jam detection bands (N m per joint)
    view_centre: tuple = (0.12, 0.0, 0.2)   # m, arm frame: where the arm page's view centres (its workspace)
    gripper: str = "rebot"      # robots/grippers.py
    gripper_spec: Optional[dict] = None     # its GripperType.public() (the arm node reads it from the environment)

    def public(self) -> dict:
        return asdict(self)

    def gripper_type(self):
        from .grippers import get
        return get(self.gripper)


@lru_cache(maxsize=None)
def _raw(name: str) -> dict:
    if name not in TYPES:
        raise ValueError(f"unknown arm type {name!r} (have {REBOT}, {', '.join(TYPES)})")
    return yaml.safe_load(resources.files(__package__).joinpath(TYPES[name]).read_text())


def load(name: str, urdf: Optional[str] = None) -> Optional[ArmType]:
    """The arm type ``name`` (None for the reBot: its own rebot_ctl paths)."""
    if name in (None, REBOT):
        return None
    r = _raw(name)
    # the gripper type (robots/grippers.py): its motor's torque limit and MIT gains (never by importing arm/config.py
    # here: the arm node loads its type before that import, which fixes N for the process and its control process)
    from .grippers import get as gripper
    g = gripper(r.get("gripper"))
    t = tuple
    return ArmType(name=r["name"], dof=int(r["dof"]), kp=t(r["kp"]), kd=t(r["kd"]), tau_scale=t(r["tau_scale"]),
                   torque_max=t(r["torque_max"]), gripper_torque_max=g.motor_torque_max,
                   joint_vel=t(r["joint_vel"]), joint_acc=t(r["joint_acc"]), g_kp=g.g_kp, g_kd=g.g_kd, urdf=urdf,
                   geometry=r.get("geometry"), jam_bands=t(r["jam_bands"]),
                   view_centre=t(r.get("view_centre", (0.12, 0.0, 0.2))), gripper=g.name, gripper_spec=g.public())


def current() -> Optional[dict]:
    """This process's arm type (the arm node's, from ``ENV``), or None (the reBot)."""
    s = os.environ.get(ENV)
    return json.loads(s) if s else None


def joint_limits(urdf: str, joints) -> dict:
    """{joint: (lower, upper)} from a URDF (the arm's joints, unprefixed)."""
    import xml.etree.ElementTree as ET
    r = ET.parse(urdf).getroot()
    out = {}
    for n in joints:
        lim = r.find(f"joint[@name='{n}']/limit")
        out[n] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


@lru_cache(maxsize=None)
def arm_boxes(urdf: str, gripper: Optional[str] = "rebot") -> tuple[list, list]:
    """(boxes, names) of an arm type's arm drawn as boxes, in arm/geometry.py's ARM_BOXES format ``[frame, cx, cy,
    cz, sx, sy, sz]`` (mm, axis-aligned in the frame): ``base`` and ``link1``..``linkN`` each the bounding box of
    that link's collision meshes and boxes in its frame (the URDF's collision origins applied; the OpenArm's link7:
    its flange adapter box; a link without geometry has none), then the gripper's ``end`` and ``finger`` boxes (robots/grippers.py: the
    gripper the arm carries). The frames are the arm node's GET links frames: link i's is joint i's (a URDF joint's frame is its
    child link's), ``end`` = end_link."""
    import math
    import xml.etree.ElementTree as ET
    from pathlib import Path

    import numpy as np

    from .grippers import get as gripper_type
    from ..nodes.description.model import _mesh_points
    root = ET.parse(urdf).getroot()
    boxes, names = [], []
    for lk in root.findall("link"):
        n = lk.get("name")
        frame = "base" if n == "base_link" else n
        if frame != "base" and not (n.startswith("link") and n[4:].isdigit()):
            continue
        pts = []
        for col in lk.findall("collision"):
            mesh, box = col.find("geometry/mesh"), col.find("geometry/box")
            if mesh is None and box is None:
                continue
            o = col.find("origin")
            xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
            r, p, y = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
            Rm = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                           [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])
            if mesh is not None:
                scale = [float(v) for v in (mesh.get("scale") or "1 1 1").split()]
                P = _mesh_points((Path(urdf).parent / mesh.get("filename")).resolve(), scale)
            else:                                                   # a box: its corners
                h = np.array([float(v) for v in box.get("size").split()]) / 2
                P = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * h
            with np.errstate(all="ignore"):     # spurious FP flags from Accelerate's matmul (macOS)
                pts.append(P @ Rm.T + np.asarray(xyz))
        if not pts:
            continue
        P = np.vstack(pts) * 1000.0
        lo, hi = P.min(axis=0), P.max(axis=0)
        boxes.append([frame, *[round(float(v), 1) for v in (lo + hi) / 2], *[round(float(v), 1) for v in hi - lo]])
        names.append("mount" if frame == "base" else frame)
    g = gripper_type(gripper)
    for b, n in zip(g.boxes, g.box_names):
        boxes.append(list(b))
        names.append(n)
    return boxes, names
