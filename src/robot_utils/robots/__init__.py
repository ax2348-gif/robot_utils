"""Robot profiles: which robot the stack and the sim run (docs/plans/bimanual_and_robot_swap.md).

A profile is ``robots/<name>.yaml``:

- ``description``: the description YAML the description node starts from (before its data dir's live
  overrides): ``file: <package>:<path>``, or ``extends: <profile>`` plus an ``overlay`` deep-merged over that
  profile's description (a ``null`` value deletes the key).
- ``sim``: merged over the sim world's default_sim.yaml (cameras, per-arm start poses, ...).
- ``stack``: the sim stack's node set: ``without`` (registry instances this robot cannot run yet).
- ``sim_only``: true for a robot that exists only in simulation (nodes refuse hardware).

The arms are part of the description (its ``arms:`` map, ``arms_of``), so they travel with the model: the
description node publishes them in GET /model (``arms``), the sim world reads them from the bundle. A description
without ``arms:`` has the one arm ``arm`` (component ``arm``) when it has such a component: the robot as it was.

One name per arm, everywhere: the arm's ``name`` is its arm node instance, its description component (links and
joints prefixed ``<name>_``), its description frame, its sim backend socket (``<name>.sock``) and its WBC end
effector. Selection: ``--robot NAME`` (description, sim, arm nodes, the sim stack); default ``$ROBOT_UTILS_ROBOT``,
else ``rebot``.
"""
from __future__ import annotations

import hashlib
import os
import tempfile
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path
from typing import Optional

import yaml

DEFAULT = "rebot"
ENV = "ROBOT_UTILS_ROBOT"

# the reBot arm's defaults (an arm entry may override each): its URDF's arm joints, end link, gripper joint, TCP
REBOT_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
REBOT_EE_LINK = "end_link"
REBOT_GRIPPER = "finger_left"
REBOT_TCP = (-0.15, 0.0, 0.0)


class ProfileError(ValueError):
    """An unknown robot or an invalid profile."""


def default_name() -> str:
    return os.environ.get(ENV) or DEFAULT


def available() -> list[str]:
    return sorted(p.name[:-5] for p in resources.files(__package__).iterdir() if p.name.endswith(".yaml"))


def merge(a: dict, b: dict) -> dict:
    """``b`` deep-merged over ``a`` (common/merge.py: the one rule of every YAML layer, a null deletes the key)."""
    from ..nodes.common.merge import deep_merge
    return deep_merge(a, b)


def _raw(name: str) -> dict:
    if name not in available():
        raise ProfileError(f"unknown robot {name!r} (have {', '.join(available())})")
    return yaml.safe_load(resources.files(__package__).joinpath(f"{name}.yaml").read_text()) or {}


def _description(name: str, seen: tuple = ()) -> dict:
    if name in seen:
        raise ProfileError(f"robot profiles extend each other in a loop: {' -> '.join(seen + (name,))}")
    d = _raw(name).get("description") or {}
    if "file" in d:
        from ..nodes.description.model import resolve_file
        return yaml.safe_load(resolve_file(d["file"]).read_text()) or {}
    if "extends" in d:
        return merge(_description(d["extends"], seen + (name,)), d.get("overlay") or {})
    raise ProfileError(f"robot {name!r}: description needs `file` or `extends`")


@dataclass(frozen=True)
class ArmSpec:
    """One arm of the robot, resolved against its description component."""
    name: str
    component: str
    prefix: str                       # the model's link / joint prefix (the component's)
    file: str                         # the arm's URDF (``<package>:<path>``)
    mirror: bool                      # the component is the mirror image of its URDF (robots/mirror.py)
    frame: str                        # the description frame the arm's base hangs from (its arm frame)
    joints: tuple = REBOT_JOINTS      # arm joints, URDF names (unprefixed), in order
    ee_link: str = REBOT_EE_LINK
    gripper: Optional[str] = REBOT_GRIPPER     # the gripper's driving joint (URDF name); None: no gripper
    tcp: tuple = REBOT_TCP
    base_link: str = "base_link"      # the arm URDF's root link (its mount)
    side: Optional[str] = None        # left | right: which side of the robot (the teleop's hand mapping); None: one arm
    type: str = "rebot"                # the arm type (robots/arm_types.py): rebot, or e.g. openarm_v2
    posture: Optional[tuple] = None   # the nominal posture (rad, the WBC's null-space posture; None: the WBC's own)

    @property
    def dof(self) -> int:
        return len(self.joints)

    def arm_type(self):
        """Its ``robots.arm_types.ArmType`` (None for the reBot), with its URDF."""
        from .arm_types import load
        return load(self.type, self.urdf_path())

    @property
    def model_joints(self) -> list[str]:
        return [self.prefix + j for j in self.joints]

    @property
    def model_ee_link(self) -> str:
        return self.prefix + self.ee_link

    @property
    def model_base_link(self) -> str:
        return self.prefix + self.base_link

    @property
    def model_gripper(self) -> Optional[str]:
        return None if self.gripper is None else self.prefix + self.gripper

    def urdf_text(self) -> str:
        """The arm's own URDF (unprefixed), mirrored when the arm is."""
        from ..nodes.description.model import resolve_file
        text = resolve_file(self.file).read_text(encoding="utf-8-sig")
        if self.mirror:
            from .mirror import mirror_urdf_text
            text = mirror_urdf_text(text)
        return text

    def urdf_path(self) -> Optional[str]:
        """A URDF file for the arm node's own kinematics: None = the arm driver's (rebot_ctl's hardware config);
        a mirrored arm gets its mirrored URDF written to a cache file (named by its content)."""
        if not self.mirror:
            if self.type == "rebot":
                return None                       # rebot_ctl's own (its hardware config's URDF)
            from ..nodes.description.model import resolve_file
            return str(resolve_file(self.file))
        text = self.urdf_text()
        d = Path(tempfile.gettempdir()) / "robot_utils-urdf"
        d.mkdir(parents=True, exist_ok=True)
        p = d / f"{self.name}-{hashlib.sha256(text.encode()).hexdigest()[:12]}.urdf"
        if not p.exists():
            tmp = p.with_suffix(f".tmp{os.getpid()}")
            tmp.write_text(text)
            os.replace(tmp, p)
        return str(p)

    def public(self) -> dict:
        return {"name": self.name, "component": self.component, "prefix": self.prefix, "file": self.file,
                "mirror": self.mirror, "frame": self.frame, "joints": self.model_joints,
                "ee_link": self.model_ee_link, "base_link": self.model_base_link, "gripper": self.model_gripper,
                "tcp": list(self.tcp), "side": self.side, "type": self.type,
                "posture": None if self.posture is None else list(self.posture)}


def arms_of(description: dict) -> list[ArmSpec]:
    """The arms of a description (its ``arms:`` map, in order; see the module doc for the default)."""
    comps = description.get("components") or {}
    arms = description.get("arms")
    if arms is None:
        arms = {"arm": {"component": "arm"}} if (comps.get("arm") or {}).get("source") == "urdf" else {}
    out = []
    for name, a in arms.items():
        a = a or {}
        cname = a.get("component", name)
        c = comps.get(cname)
        if c is None or c.get("source") != "urdf":
            raise ProfileError(f"arm {name!r}: no URDF component {cname!r} in the description")
        mount = c.get("mount") or {}
        frame = a.get("frame") or mount.get("frame") or c.get("frame") or name
        out.append(ArmSpec(name=name, component=cname, prefix=c.get("prefix", f"{cname}_"), file=c["file"],
                           mirror=bool(c.get("mirror", False)), frame=frame,
                           joints=tuple(a.get("joints") or REBOT_JOINTS), ee_link=a.get("ee_link", REBOT_EE_LINK),
                           gripper=a.get("gripper", REBOT_GRIPPER), tcp=tuple(a.get("tcp") or REBOT_TCP),
                           base_link=a.get("base_link", "base_link"), side=_side(name, a.get("side")),
                           type=str(a.get("type", "rebot")),
                           posture=None if a.get("posture") is None else tuple(float(v) for v in a["posture"])))
    return out


def _side(name: str, side) -> Optional[str]:
    if side not in (None, "left", "right"):
        raise ProfileError(f"arm {name!r}: side must be left or right, not {side!r}")
    return side


@dataclass
class Profile:
    name: str
    doc: str
    description: dict
    sim: dict = field(default_factory=dict)
    stack: dict = field(default_factory=dict)
    sim_only: bool = False
    wbc: dict = field(default_factory=dict)      # the WBC's config defaults for this robot (wbc/spec.py Config keys)

    @property
    def arms(self) -> list[ArmSpec]:
        return arms_of(self.description)

    def arm(self, name: str) -> ArmSpec:
        """The arm an arm node instance ``name`` runs. A one-arm robot's arm serves any instance name (an arm
        node run under another name, as before profiles); with several, the name picks one."""
        arms = self.arms
        for a in arms:
            if a.name == name:
                return a
        if len(arms) == 1:
            return arms[0]
        raise ProfileError(f"robot {self.name!r} has no arm {name!r} (arms: {', '.join(a.name for a in arms) or 'none'})")

    def public(self) -> dict:
        return {"name": self.name, "doc": self.doc, "sim_only": self.sim_only,
                "arms": [a.public() for a in self.arms]}


def load(name: Optional[str] = None) -> Profile:
    name = name or default_name()
    raw = _raw(name)
    return Profile(name=name, doc=str(raw.get("doc") or "").strip(), description=_description(name),
                   sim=raw.get("sim") or {}, stack=raw.get("stack") or {}, sim_only=bool(raw.get("sim_only")),
                   wbc=raw.get("wbc") or {})
