"""Arm node settings: gain profiles (rebot_ctl's hardware yaml files), gripper geometry, speed
limits and named poses.

Gain profiles are rebot_ctl hardware files: ``soft`` (production) = rebotarm_dm_soft.yaml,
``safe`` (debug, low stiffness) = rebotarm_dm_safe.yaml. A profile supplies the MIT gains and the
gravity ``tau_scale``; the bus, joints and limits are the same in both.

Named poses and limits: packaged defaults below, overridden by ``~/.config/robot_utils/arm.yaml``
(``$XDG_CONFIG_HOME``) if present, an admin file::

    named_poses:          # joint degrees, joint1..joint6
      home: [0, 0, 0, 0, 0, 0]
      ready: [0, -40, -60, 0, 30, 0]
    home_via: [ready]     # optional: poses `home` passes through, in order
    joint_vel_max: [2, 2, 2, 6, 6, 6]   # rad/s, hard caps of the step limit
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

N = 6
PROFILES = {"soft": "rebotarm_dm_soft.yaml", "safe": "rebotarm_dm_safe.yaml"}
DEFAULT_PROFILE = "soft"

# Gripper: module 1, 16 teeth pinion (pitch radius 8 mm) driving both racks.
PITCH_RADIUS = 0.008                 # m
GAP_PER_RAD = 2 * PITCH_RADIUS       # m of jaw gap per rad
GRIPPER_OPEN_RAD = -3.1              # fully open (calibrated on the robot; 0 = closed)
GAP_MAX = -GRIPPER_OPEN_RAD * GAP_PER_RAD   # 49.6 mm
FF_OPEN_MARGIN = 0.005               # m: the force-limit feedforward is off this close to fully open
DEFAULT_MAX_FORCE = 50.0             # N squeeze
HARD_MAX_FORCE = 120.0               # N


def gap_to_rad(gap_m: float) -> float:
    return -float(gap_m) / GAP_PER_RAD


def rad_to_gap(q: float) -> float:
    return -float(q) * GAP_PER_RAD


def force_of_torque(tau: float) -> float:
    """Squeeze force (N) for a closing torque (Nm): one pinion drives both jaws."""
    return float(tau) / (2 * PITCH_RADIUS)


def torque_of_force(f: float) -> float:
    return float(f) * 2 * PITCH_RADIUS


@dataclass
class Profile:
    name: str
    file: str
    kp: np.ndarray
    kd: np.ndarray
    g_kp: float
    g_kd: float
    tau_scale: np.ndarray

    def public(self) -> dict:
        return {"name": self.name, "file": self.file, "kp": self.kp.tolist(), "kd": self.kd.tolist(),
                "gripper": {"kp": self.g_kp, "kd": self.g_kd}, "tau_scale": self.tau_scale.tolist()}


def load_profile(name: str) -> Profile:
    if name not in PROFILES:
        raise ValueError(f"unknown gain profile {name!r} (one of {', '.join(PROFILES)})")
    from reBotArm_control_py.actuator.rebotarm import _resolve_hw_cfg_path, load_cfg
    file = PROFILES[name]
    cfg = load_cfg(file)
    joints = {j.name: j for j in cfg["joints"]}
    arm = [joints[f"joint{i}"] for i in range(1, N + 1)]
    raw = yaml.safe_load(_resolve_hw_cfg_path(file).read_text()) or {}
    scale = (raw.get("gravity_compensation") or {}).get("tau_scale") or [1.0] * N
    return Profile(name, file, np.array([j.kp for j in arm], float), np.array([j.kd for j in arm], float),
                   float(joints["gripper"].kp), float(joints["gripper"].kd), np.array(scale[:N], float))


@dataclass
class Limits:
    """Speed limits. ``hard`` are fixed (config); the lease holder may lower ``soft`` within them."""
    joint_vel: np.ndarray = field(default_factory=lambda: np.array([2.0, 2.0, 2.0, 6.0, 6.0, 6.0]))  # rad/s
    lin_vel: float = 0.15        # m/s   servo default
    lin_acc: float = 0.5         # m/s²
    ang_vel: float = 1.0         # rad/s
    ang_acc: float = 3.0         # rad/s²
    gripper_vel: float = 0.1     # m/s of gap
    home_vel: float = 0.5        # rad/s, joint-space moves (home, named poses)

    def public(self) -> dict:
        return {"joint_vel": [round(float(v), 4) for v in self.joint_vel], "lin_vel": self.lin_vel,
                "lin_acc": self.lin_acc, "ang_vel": self.ang_vel, "ang_acc": self.ang_acc,
                "gripper_vel": self.gripper_vel, "home_vel": self.home_vel}

    def copy(self) -> "Limits":
        return Limits(self.joint_vel.copy(), self.lin_vel, self.lin_acc, self.ang_vel, self.ang_acc,
                      self.gripper_vel, self.home_vel)


HARD = Limits(joint_vel=np.array([2.0, 2.0, 2.0, 6.0, 6.0, 6.0]), lin_vel=0.5, lin_acc=2.0, ang_vel=3.0,
              ang_acc=10.0, gripper_vel=0.15, home_vel=1.0)
DEFAULT_SOFT = Limits(joint_vel=HARD.joint_vel.copy())


@dataclass
class NodeSettings:
    named_poses: dict = field(default_factory=lambda: {"home": np.zeros(N)})   # rad
    home_via: list = field(default_factory=list)
    hard: Limits = field(default_factory=HARD.copy)
    # servo / home abort: output-side TCP error from the command beyond these for ABORT_HOLD_S
    abort_pos: float = 0.03            # m
    abort_rot: float = math.radians(15)
    abort_hold_s: float = 0.1
    reach_pos: float = 0.002           # m, servo "reached"
    reach_rot: float = math.radians(1.0)
    reach_dwell_s: float = 0.2
    settle_timeout_s: float = 2.0


def config_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))
    return Path(base) / "robot_utils" / "arm.yaml"


def load_settings(path: Optional[Path] = None) -> NodeSettings:
    s = NodeSettings()
    p = path or config_path()
    if not p.is_file():
        return s
    raw = yaml.safe_load(p.read_text()) or {}
    poses = raw.get("named_poses") or {}
    for name, deg in poses.items():
        v = np.radians(np.asarray(deg, dtype=float))
        if v.shape != (N,) or not np.all(np.isfinite(v)):
            raise ValueError(f"{p}: named pose {name!r} must be {N} joint angles in degrees")
        s.named_poses[str(name)] = v
    s.home_via = [str(n) for n in raw.get("home_via") or []]
    for n in s.home_via:
        if n not in s.named_poses:
            raise ValueError(f"{p}: home_via names an unknown pose {n!r}")
    if "joint_vel_max" in raw:
        v = np.asarray(raw["joint_vel_max"], dtype=float)
        if v.shape != (N,) or np.any(v <= 0) or np.any(v > 20):
            raise ValueError(f"{p}: joint_vel_max must be {N} speeds in 0..20 rad/s")
        s.hard.joint_vel = v
    return s
