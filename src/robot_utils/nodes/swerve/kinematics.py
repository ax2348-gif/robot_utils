"""Swerve kinematics: robot twist <-> per-wheel velocities and module commands.

Pure functions (no I/O) over the module geometry in ``constants.MODULES``.
Ported unchanged from rev_ctl ``examples/swerve_drive.py`` (commit cfcfc32).
"""
from __future__ import annotations

import math

from .constants import MAX_WHEEL_SPEED, MODULES

MIN_SPEED = 1e-3     # m/s; slower than this, a wheel keeps its angle instead of turning


def wrap_angle(a: float) -> float:
    """Angle folded into [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def wheel_velocities(vx: float, vy: float, omega: float) -> list:
    """Desired (x, y) velocity in m/s of each wheel for a robot twist.

    Each wheel moves with the robot: (vx - omega * y, vy + omega * x). If any wheel would
    exceed MAX_WHEEL_SPEED, all are scaled down together, keeping the motion's shape.
    """
    velocities = [(vx - omega * m.y, vy + omega * m.x) for m in MODULES]
    fastest = max(math.hypot(wx, wy) for wx, wy in velocities)
    scale = min(1.0, MAX_WHEEL_SPEED / fastest) if fastest > 0 else 1.0
    return [(wx * scale, wy * scale) for wx, wy in velocities]


def module_command(wx: float, wy: float, heading: float) -> tuple:
    """(steering angle rad, drive speed m/s) for a wheel's desired velocity and current heading.

    The wheel's axis is a line, so the angle is whichever direction along the velocity is
    nearer the current heading (at most 90 degrees away), with the speed negated for the
    opposite one. The speed is scaled by cos(steering error), so a wheel that has not turned
    yet does not push sideways. A wheel that should (almost) stand still keeps its heading.
    """
    speed = math.hypot(wx, wy)
    if speed < MIN_SPEED:
        return heading, 0.0
    angle = math.atan2(wy, wx)
    if abs(wrap_angle(angle - heading)) > math.pi / 2:
        angle, speed = wrap_angle(angle + math.pi), -speed
    return angle, speed * math.cos(angle - heading)


def robot_twist(wheel_velocities: list) -> tuple:
    """The robot twist (vx, vy, omega) that best fits measured (x, y) wheel velocities.

    Least squares of wx = vx - omega * y, wy = vy + omega * x over the modules; with the
    modules centred on the robot this is the mean wheel velocity, and for omega the
    tangential components weighted by distance.
    """
    n = len(MODULES)
    vx = sum(wx for wx, _ in wheel_velocities) / n
    vy = sum(wy for _, wy in wheel_velocities) / n
    omega = (sum(m.x * wy - m.y * wx for m, (wx, wy) in zip(MODULES, wheel_velocities))
             / sum(m.x ** 2 + m.y ** 2 for m in MODULES))
    return vx, vy, omega
