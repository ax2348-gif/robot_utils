"""Mirror a URDF across its root frame's x-z plane (y -> -y): the left-hand copy of a right-hand arm.

Reflecting a kinematic chain is not a rigid motion, so every frame is replaced by its conjugate
``M F M`` (``M = diag(1, -1, 1)``): frames stay right-handed, and the mirrored model at joint values ``q``
is exactly the mirror image of the original at the same ``q`` ("mirrored convention"). Per element:

- an ``origin`` (joint, visual, collision, inertial): xyz -> (x, -y, z); rpy -> (-roll, pitch, -yaw)
  (``M Rz(a) Ry(b) Rx(c) M = Rz(-a) Ry(b) Rx(-c)``);
- a revolute / continuous joint's axis a -> -M a = (-ax, ay, -az) (a rotation axis is a pseudovector:
  ``M Rot(a, q) M = Rot(-M a, q)``); a prismatic joint's axis a -> M a = (ax, -ay, az);
- joint limits unchanged (the same q is the mirrored pose, so the same range is the mirrored range);
- an inertia tensor I -> M I M (ixy and iyz change sign);
- a mesh: its scale's y component changes sign (MuJoCo and pinocchio accept a negative scale); boxes,
  cylinders, spheres and capsules are symmetric in their own frames and need nothing.

Gravity torques are unchanged at the same q (the potential energy is mirror-invariant), so gravity
compensation tuned on the original arm carries over. Hardware is another matter: a physical arm's motors
turn one way, so a real mirrored arm needs a per-joint sign map in its driver (the joints whose axis
flipped) -- the sim needs none. Pure: XML in, XML out.
"""
from __future__ import annotations

import copy
import xml.etree.ElementTree as ET

REVOLUTE = ("revolute", "continuous")


def _floats(s, n=3, default=0.0) -> list:
    v = [float(x) for x in (s or "").split()]
    return v if len(v) == n else [default] * n


def _fmt(v) -> str:
    return " ".join(repr(float(x)) if x != 0 else "0" for x in v)


def _origin(el: ET.Element) -> None:
    o = el.find("origin")
    if o is None:
        return
    x, y, z = _floats(o.get("xyz"))
    r, p, w = _floats(o.get("rpy"))
    o.set("xyz", _fmt([x, -y, z]))
    o.set("rpy", _fmt([-r, p, -w]))


def mirror_urdf(robot: ET.Element) -> ET.Element:
    """A mirrored copy of a parsed URDF ``<robot>`` (names unchanged)."""
    out = copy.deepcopy(robot)
    for j in out.findall("joint"):
        _origin(j)
        ax = j.find("axis")
        if ax is not None and j.get("type") != "fixed":
            a = _floats(ax.get("xyz"), default=0.0)
            ax.set("xyz", _fmt([-a[0], a[1], -a[2]] if j.get("type") in REVOLUTE else [a[0], -a[1], a[2]]))
    for lk in out.findall("link"):
        for tag in ("visual", "collision", "inertial"):
            for e in lk.findall(tag):
                _origin(e)
                for mesh in e.iter("mesh"):
                    sx, sy, sz = _floats(mesh.get("scale"), default=1.0)
                    mesh.set("scale", _fmt([sx, -sy, sz]))
        it = lk.find("inertial/inertia")
        if it is not None:
            for k in ("ixy", "iyz"):
                if it.get(k) is not None:
                    it.set(k, repr(-float(it.get(k))) if float(it.get(k)) != 0 else "0")
    return out


def mirror_urdf_text(text: str) -> str:
    return ET.tostring(mirror_urdf(ET.fromstring(text)), encoding="unicode")
