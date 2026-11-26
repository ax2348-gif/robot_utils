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
    gripper:              # the contact detector of the squeeze limit (GripperContact; defaults shown)
      stall_vel: 0.5      # rad/s motor
      stall_ms: 30
      contact_err_mm: 0.5
      release_mm: 1.0
      approach_mm: 3.0
      forget_mm: 10.0
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

# The arm type of this process (robots/arm_types.py: the arm node's __main__ sets it in the environment before
# importing the rest; None = the reBot, rebot_ctl's config): another type gives N, the gains and the limits below.
from ...robots.arm_types import current as _current_type
ARM_TYPE = _current_type()

N = 6 if ARM_TYPE is None else int(ARM_TYPE["dof"])   # arm joints (the gripper is separate); defined here only:
                                                        # bus, kin, the node, ... import it
PROFILES = {"soft": "rebotarm_dm_soft.yaml", "safe": "rebotarm_dm_safe.yaml"}
DEFAULT_PROFILE = "soft"

# Gripper: module 1, 16 teeth pinion (pitch radius 8 mm) driving both racks.
PITCH_RADIUS = 0.008                 # m
GRIPPER_OPEN_RAD = -3.1              # fully open (calibrated on the robot; 0 = closed)
DEFAULT_MAX_FORCE = 50.0             # N squeeze
HARD_MAX_FORCE = 120.0               # N
if ARM_TYPE is not None and ARM_TYPE.get("gripper_spec"):     # another arm type's gripper (robots/grippers.py)
    _g = ARM_TYPE["gripper_spec"]
    PITCH_RADIUS, GRIPPER_OPEN_RAD = float(_g["pitch_radius"]), float(_g["open_rad"])
    DEFAULT_MAX_FORCE, HARD_MAX_FORCE = float(_g["default_max_force"]), float(_g["hard_max_force"])
GAP_PER_RAD = 2 * PITCH_RADIUS       # m of jaw gap per rad
GAP_MAX = -GRIPPER_OPEN_RAD * GAP_PER_RAD   # 49.6 mm (the reBot's)
FF_OPEN_MARGIN = 0.005               # m: the force-limit feedforward is off this close to fully open
                                     # (the position command is pulled back instead)

# Damiao motor torque at the output shaft (N m): the peak (datasheet) and TMAX, the MIT frame's torque mapping range
# (motorbridge's model table, motor_vendors/damiao/src/motor.rs: what a frame's tau_ff and the feedback can encode).
# A motor delivers at most min(peak, TMAX). Which model each joint is comes from the gain profile's hardware file.
MOTOR_TORQUE = {"4340P": {"peak": 27.0, "tmax": 28.0}, "4340": {"peak": 27.0, "tmax": 28.0},
                "4310": {"peak": 7.0, "tmax": 10.0}, "4310P": {"peak": 7.0, "tmax": 10.0}}


def motor_torque_limits(profile: str = DEFAULT_PROFILE) -> tuple:
    """(arm joints' torque limits (N,), the gripper motor's) in N m at the output: min(datasheet peak, TMAX) of each
    joint's motor model as the profile's hardware file names it (rebotarm_dm_*.yaml ``model``); another arm type:
    its own (robots/arm_types.py)."""
    if ARM_TYPE is not None:
        return np.array(ARM_TYPE["torque_max"], float), float(ARM_TYPE["gripper_torque_max"])
    from reBotArm_control_py.actuator.rebotarm import load_cfg
    joints = {j.name: j for j in load_cfg(PROFILES[profile])["joints"]}

    def lim(name):
        mt = MOTOR_TORQUE[str(joints[name].model)]
        return min(mt["peak"], mt["tmax"])
    return np.array([lim(f"joint{i}") for i in range(1, N + 1)]), lim("gripper")


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
    if ARM_TYPE is not None:                      # another arm type: its gains (one profile for every name)
        t = ARM_TYPE
        return Profile(name, f"arm type {t['name']}", np.array(t["kp"], float), np.array(t["kd"], float),
                       float(t["g_kp"]), float(t["g_kd"]), np.array(t["tau_scale"], float))
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


# the hard joint caps: the reBot's, or the arm type's
_HARD_VEL = np.array([2.0, 2.0, 2.0, 6.0, 6.0, 6.0] if ARM_TYPE is None else ARM_TYPE["joint_vel"], float)
_HARD_ACC = np.array([20.0, 20.0, 20.0, 60.0, 60.0, 60.0] if ARM_TYPE is None else ARM_TYPE["joint_acc"], float)


@dataclass
class Limits:
    """Speed limits. ``hard`` are fixed (config); the lease holder may lower ``soft`` within them."""
    joint_vel: np.ndarray = field(default_factory=lambda: _HARD_VEL.copy())          # rad/s
    joint_acc: np.ndarray = field(default_factory=lambda: _HARD_ACC.copy() / 2)      # rad/s²
    lin_vel: float = 0.15        # m/s   servo default
    lin_acc: float = 0.5         # m/s²
    ang_vel: float = 1.0         # rad/s
    ang_acc: float = 3.0         # rad/s²
    home_vel: float = 0.5        # rad/s, joint-space moves (home, named poses)

    def public(self) -> dict:
        return {"joint_vel": [round(float(v), 4) for v in self.joint_vel],
                "joint_acc": [round(float(v), 4) for v in self.joint_acc], "lin_vel": self.lin_vel,
                "lin_acc": self.lin_acc, "ang_vel": self.ang_vel, "ang_acc": self.ang_acc,
                "home_vel": self.home_vel}

    def copy(self) -> "Limits":
        return Limits(self.joint_vel.copy(), self.joint_acc.copy(), self.lin_vel, self.lin_acc, self.ang_vel,
                      self.ang_acc, self.home_vel)


HARD = Limits(joint_vel=_HARD_VEL.copy(), joint_acc=_HARD_ACC.copy(),
              lin_vel=0.5, lin_acc=2.0, ang_vel=3.0, ang_acc=10.0, home_vel=1.0)
DEFAULT_SOFT = Limits(joint_vel=HARD.joint_vel.copy())


@dataclass
class GripperContact:
    """The gripper's contact detector (gripper.py): the squeeze limit applies in contact only. Gap distances in m
    (16 mm of gap per rad of the motor). Defaults from the soft profile's gripper gains (kp 1.0 Nm/rad,
    kd 0.05 Nm s/rad; F = tau / 16 mm, so 1 mm of gap error is 0.0625 Nm = 3.9 N of squeeze):

    - ``stall_vel`` 0.5 rad/s (8 mm/s of gap): "the jaws are not moving". A free jaw with the goal past the limit's
      own error is driven by more than ``max_force`` worth of torque and reaches this in a few ms; its damping term
      kd x stall_vel is 0.025 Nm (1.6 N), negligible against the limit. A blocked jaw reads ~0.
    - ``stall_s`` 30 ms (3 ticks): the stall must last this long, so a command starting from rest (the reading can
      lag the command by up to 2 ticks) is not taken for a contact. It bounds the impact transient's duration.
    - ``contact_err`` 0.5 mm: the goal must be past the jaws by more than this as well (2 N at kp 1.0): a free
      jaw resting at its goal against friction is not a contact.
    - ``release`` 1 mm: hysteresis. Contact ends when the goal comes back within (limit error - 1 mm) of the jaws,
      i.e. the law alone would squeeze 3.9 N less than the limit; also how far the jaws may close past a remembered
      contact before it is forgotten (the object is gone).
    - ``approach`` 3 mm: closing back to within this of a remembered contact position is capped too (a re-grasp).
    - ``forget`` 10 mm: opening this far past the remembered contact forgets it.
    """
    stall_vel: float = 0.5        # rad/s, motor side
    stall_s: float = 0.03
    contact_err: float = 0.0005   # m of gap
    release: float = 0.001
    approach: float = 0.003
    forget: float = 0.010


@dataclass
class NodeSettings:
    named_poses: dict = field(default_factory=lambda: {"home": np.zeros(N)})   # rad
    home_via: list = field(default_factory=list)
    hard: Limits = field(default_factory=HARD.copy)
    # servo / home abort: output-side TCP error from the command beyond these for ABORT_HOLD_S
    abort_pos: float = 0.10            # m (soft gains track loosely; 30 mm tripped named-pose moves)
    abort_rot: float = math.radians(15)
    abort_hold_s: float = 0.1
    reach_pos: float = 0.002           # m, servo "reached"
    reach_rot: float = math.radians(1.0)
    reach_dwell_s: float = 0.2
    settle_timeout_s: float = 2.0
    # the internal joint channel (stream.py): ramp to a hold after this long without a fresh goal; a goal may lead
    # the measured pose by at most stream_max_lead (any joint)
    stream_stale_ms: float = 30.0
    stream_max_lead: float = math.radians(20.0)
    gripper: GripperContact = field(default_factory=GripperContact)
    # stall limits (docs/arm.md "Timing"): with the motors on, a reading this old faults (node.STALE_ON); the control
    # process silent this long faults the node (proc_node). The robot's values; --timing lenient (the sim world on
    # macOS, where scheduling stalls of 0.1-0.5 s happen under load) raises them, never on hardware.
    stale_on: float = 0.05
    ctl_heartbeat: float = 0.25
    timing: str = "strict"


LENIENT_TIMING = {"stale_on": 0.5, "ctl_heartbeat": 2.0}


def apply_timing(settings: "NodeSettings", mode: str, sim: bool, platform: str) -> "NodeSettings":
    """``--timing``: strict keeps the robot's stall limits; lenient raises them (a simulated arm only: ValueError on
    hardware); auto = lenient for a simulated arm on macOS. Changes ``settings`` in place and returns it."""
    if mode not in ("auto", "strict", "lenient"):
        raise ValueError(f"timing is auto, strict or lenient, not {mode!r}")
    lenient = mode == "lenient" or (mode == "auto" and sim and platform == "darwin")
    if lenient and not sim:
        raise ValueError("lenient timing is for a simulated arm only: the robot keeps its stall limits")
    if lenient:
        settings.stale_on, settings.ctl_heartbeat = LENIENT_TIMING["stale_on"], LENIENT_TIMING["ctl_heartbeat"]
    settings.timing = ("lenient (sim on macOS)" if mode == "auto" else "lenient") if lenient else "strict"
    return settings


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
    jc = raw.get("joint_channel") or {}
    if "stale_ms" in jc:
        v = float(jc["stale_ms"])
        if not 10.0 <= v <= 500.0:
            raise ValueError(f"{p}: joint_channel.stale_ms must be within 10..500")
        s.stream_stale_ms = v
    if "max_lead_deg" in jc:
        v = float(jc["max_lead_deg"])
        if not 0.5 <= v <= 45.0:
            raise ValueError(f"{p}: joint_channel.max_lead_deg must be within 0.5..45")
        s.stream_max_lead = math.radians(v)
    gr = raw.get("gripper") or {}
    for key, attr, lo, hi, scale in (("stall_vel", "stall_vel", 0.05, 5.0, 1.0),
                                     ("stall_ms", "stall_s", 10.0, 200.0, 1e-3),
                                     ("contact_err_mm", "contact_err", 0.0, 5.0, 1e-3),
                                     ("release_mm", "release", 0.0, 10.0, 1e-3),
                                     ("approach_mm", "approach", 0.0, 20.0, 1e-3),
                                     ("forget_mm", "forget", 0.0, GAP_MAX * 1000, 1e-3)):
        if key in gr:
            v = float(gr[key])
            if not lo <= v <= hi:
                raise ValueError(f"{p}: gripper.{key} must be within {lo:g}..{hi:g}")
            setattr(s.gripper, attr, v * scale)
    return s
