"""The sim world's actuator limits, derived from the robot codebase's own configuration (docs/sim.md, "Limits come
from the robot config"). The sim carries no limits of its own: change the robot's config and the sim follows.

Swerve (``swerve/constants.py``, as ``setup_sparks.py`` configures the SPARKs):

- drive speed: the closed loop's ``output_range`` (``DRIVE_MAX_OUTPUT`` duty) x the free speed (``DRIVE_FREE_SPEED``):
  the wheel speed a duty cap allows with no load, about 1.86 m/s;
- drive acceleration: the closed-loop ramp (``DRIVE_RAMP`` s from 0 to full output) x the free speed;
- steering rate: ``STEER_MAX_OUTPUT`` duty x the module's free rate (NEO 550 free speed / ``STEER_REDUCTION``),
  about 7.4 rad/s (1.2 module turns/s);
- torque at the wheel / module: the smaller of the duty cap at stall (duty x stall torque) and the smart current
  limit (``*_CURRENT_LIMIT`` x the motor's torque per amp), through the gearing.

The simulated SPARK is a speed clamp plus a torque cap (the two ends of the duty cap's torque-speed line), not the
line itself: in between it is a little stronger than the motor.

Arm (``arm/config.py``, as the node loads it: ``load_settings()``, i.e. ``~/.config/robot_utils/arm.yaml``'s
``joint_vel_max`` when set): the joint speed caps per joint (``hard.joint_vel``); the gripper's squeeze force cap
``HARD_MAX_FORCE``.
"""
from __future__ import annotations

import numpy as np

# keys a sim config may no longer set (they come from here)
DERIVED_KEYS = {("base", "steer_rate_max"), ("base", "wheel_speed_max"), ("base", "wheel_accel_max"),
                ("base", "steer", "torque_max"), ("base", "wheel", "torque_max"),
                ("components", "arm", "joint_speed_max"), ("components", "arm", "finger_force_max")}


def swerve_limits() -> dict:
    from ..swerve import constants as K
    drive_kt = K.VORTEX_STALL_TORQUE / K.VORTEX_STALL_CURRENT
    steer_kt = K.NEO550_STALL_TORQUE / K.NEO550_STALL_CURRENT
    return {
        "wheel_speed_max": K.DRIVE_MAX_OUTPUT * K.DRIVE_FREE_SPEED,                                # m/s
        "wheel_accel_max": K.DRIVE_FREE_SPEED / K.DRIVE_RAMP,                                     # m/s^2
        "wheel_torque_max": min(K.DRIVE_MAX_OUTPUT * K.VORTEX_STALL_TORQUE,
                                drive_kt * K.DRIVE_CURRENT_LIMIT) * K.DRIVE_REDUCTION,             # N m at the wheel
        "steer_rate_max": K.STEER_MAX_OUTPUT * K.STEER_FREE_RATE,                                 # rad/s
        "steer_torque_max": min(K.STEER_MAX_OUTPUT * K.NEO550_STALL_TORQUE,
                                steer_kt * K.STEER_CURRENT_LIMIT) * K.STEER_REDUCTION,            # N m at the module
    }


def arm_limits(settings=None) -> dict:
    from ..arm import config as C
    s = settings or C.load_settings()
    return {"joint_speed_max": np.asarray(s.hard.joint_vel, float).copy(),     # rad/s per joint
            "finger_force_max": float(C.HARD_MAX_FORCE)}                       # N (the jaws' squeeze)


def robot_limits() -> dict:
    return {**swerve_limits(), **arm_limits()}


def refuse_overrides(cfg: dict) -> list:
    """The derived keys a config sets (it must not: the world raises)."""
    out = []
    for path in DERIVED_KEYS:
        d = cfg
        for k in path:
            d = d.get(k) if isinstance(d, dict) else None
            if d is None:
                break
        if d is not None:
            out.append(".".join(path))
    return sorted(out)


def describe(lim: dict) -> dict:
    """JSON-able, rounded (for /state and the docs)."""
    return {k: (np.round(v, 4).tolist() if isinstance(v, np.ndarray) else round(float(v), 4)) for k, v in lim.items()}
