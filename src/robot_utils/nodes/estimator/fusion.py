"""Planar pose fusion: swerve wheel odometry (translation) + camera-IMU heading. Pure, no I/O.

Inputs, already on the estimator's clock (Pi ``time.monotonic()`` seconds):

- **Odometry samples** (the *master*): one per swerve control tick, built by
  ``odometry_sample`` from a telemetry row. Translation comes from the change of each wheel's
  travel (``drive_pos``) along its steering angle between ticks, fitted to a body motion
  (dx, dy, dtheta) by least squares (``swerve.kinematics.robot_twist`` on displacements).
- **IMU records** from the camera node's attitude filter (~100 Hz): ``q`` (IMU -> world, world
  z up) and the window's delta angle ``da`` (IMU frame, bias corrected, every 1 kHz sample
  composed). The heading increment of a record is the world-vertical component of that
  rotation, ``(R(q) da)_z``. That is the base's heading change on a level floor **whatever the
  camera's mounting**, so no camera-to-base calibration is needed. ``HeadingTrack`` keeps the
  cumulative heading as a function of time.

Fusion runs at the master's rate and timestamps: for each odometry sample at ``t_k`` the IMU
heading is interpolated at ``t_{k-1}`` and ``t_k`` (linearly between record end times) and its
difference is the heading increment; the wheel-derived increment is the fallback when the IMU
does not cover the interval (``imu_ok`` = 0). Position integrates the body displacement rotated
by the mid-interval heading. A parallel odometry-only pose (heading from the wheels) is kept for
comparison; ``omega_wheel - omega_imu`` shows wheel slip / scrub.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Optional

from ..swerve.constants import MODULES
from ..swerve.kinematics import robot_twist

MODULE_NAMES = [m.name for m in MODULES]
IMU_FLAG_INITIALIZING = 4   # orbbec imu_filter.FLAG_INITIALIZING
IMU_FLAG_STILL = 2

FIELDS = ["t", "x", "y", "theta", "vx", "vy", "omega",
          "vx_world", "vy_world", "speed", "distance",
          "x_odom", "y_odom", "theta_odom", "omega_wheel", "omega_imu",
          "imu_ok", "odom_ok", "imu_lead", "dt", "master", "reset_id"]
UNITS = ["s", "m", "m", "rad", "m/s", "m/s", "rad/s",
         "m/s", "m/s", "m/s", "m",
         "m", "m", "rad", "rad/s", "rad/s",
         "bool", "bool", "s", "s", "enum", "count"]
FIELD_DOC = {
    "t": "sample time (master: swerve drive-reading time; IMU record end in heading-only mode), Pi monotonic s",
    "x": "position in the world frame (m); world = robot pose at the last reset, x forward, y left",
    "y": "position (m)",
    "theta": "heading (rad, CCW +), continuous (not wrapped): IMU heading where available",
    "vx": "body-frame velocity forward (m/s), from the wheels",
    "vy": "body-frame velocity left (m/s), from the wheels",
    "omega": "fused yaw rate (rad/s) = dtheta / dt",
    "vx_world": "world-frame velocity x (m/s)", "vy_world": "world-frame velocity y (m/s)",
    "speed": "|v| (m/s)", "distance": "path length since reset (m)",
    "x_odom": "odometry-only position (heading from the wheels), for comparison",
    "y_odom": "odometry-only position", "theta_odom": "odometry-only heading (rad)",
    "omega_wheel": "yaw rate from the wheels (rad/s)",
    "omega_imu": "yaw rate from the IMU over the same interval (rad/s; NaN if not covered)",
    "imu_ok": "1 if the IMU heading covered this interval (else the wheel heading was used)",
    "odom_ok": "1 if all four wheel readings were valid",
    "imu_lead": "IMU coverage end minus t when the sample was fused (s): how far the IMU was ahead",
    "dt": "interval since the previous sample (s)",
    "master": "1 = swerve odometry tick, 2 = IMU record (heading-only mode, no swerve data)",
    "reset_id": "number of pose resets since the node started: a change means the world frame moved",
}
MASTER_ODOM, MASTER_IMU = 1.0, 2.0
MAX_DT = 0.25    # s: a longer gap between master samples is not integrated across (flagged)


def quat_rotate(q, v):
    """R(q) v for a unit quaternion (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 q_vec x v; v' = v + w t + q_vec x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


@dataclass
class OdomSample:
    t: float                       # drive-reading time (median of the modules' t_drive)
    pos: tuple                     # per module wheel travel (m)
    angle: tuple                   # per module steering angle (rad)
    vel: tuple                     # per module wheel speed (m/s)
    ok: bool


def odometry_sample(row: dict) -> OdomSample:
    """Build a sample from a swerve telemetry row (dict or record with the swerve FIELDS)."""
    pos, ang, vel, ts = [], [], [], []
    for n in MODULE_NAMES:
        pos.append(float(row[f"{n}_drive_pos"]))
        ang.append(float(row[f"{n}_steer_angle"]))
        vel.append(float(row[f"{n}_drive_vel"]))
        ts.append(float(row[f"{n}_t_drive"]))
    ok = all(math.isfinite(v) for v in pos + ang + vel)
    good_t = sorted(t for t in ts if math.isfinite(t))
    t = good_t[len(good_t) // 2] if good_t else float(row["t_tick"])
    return OdomSample(t, tuple(pos), tuple(ang), tuple(vel), ok)


class HeadingTrack:
    """Cumulative world-yaw of the IMU as a piecewise-linear function of time."""

    def __init__(self, keep_s: float = 10.0):
        self.t: list[float] = []
        self.psi: list[float] = []
        self.keep_s = keep_s
        self.still = False
        self.last_q = None

    def add(self, t_start: float, t_end: float, q, da, flags: float = 0.0) -> Optional[float]:
        """Add one IMU filter record; returns its heading increment (rad) or None if unusable."""
        if int(flags) & IMU_FLAG_INITIALIZING or not (t_end > t_start):
            return None
        self.still = bool(int(flags) & IMU_FLAG_STILL)
        self.last_q = q
        dpsi = quat_rotate(q, da)[2]
        if not self.t or t_start > self.t[-1] + 0.05:       # first record, or after a gap
            self.t.append(t_start)
            self.psi.append(self.psi[-1] if self.psi else 0.0)
        elif t_end <= self.t[-1]:
            return None                                       # out of order / duplicate
        self.t.append(t_end)
        self.psi.append(self.psi[-1] + dpsi)
        cut = bisect.bisect_left(self.t, self.t[-1] - self.keep_s)
        if cut > 0:
            del self.t[:cut], self.psi[:cut]
        return dpsi

    @property
    def end(self) -> Optional[float]:
        return self.t[-1] if self.t else None

    def at(self, t: float) -> Optional[float]:
        """Interpolated cumulative heading at t, or None outside the covered span."""
        if not self.t or t < self.t[0] or t > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t:
            return self.psi[i]
        t0, t1 = self.t[i - 1], self.t[i]
        return self.psi[i - 1] + (self.psi[i] - self.psi[i - 1]) * (t - t0) / (t1 - t0)

    def covers(self, t0: float, t1: float) -> bool:
        """Whether both ends are covered by one continuous (gap-free) stretch."""
        a, b = self.at(t0), self.at(t1)
        if a is None or b is None:
            return False
        i0, i1 = bisect.bisect_left(self.t, t0), bisect.bisect_left(self.t, t1)
        start = i0 if self.t[i0] == t0 else i0 - 1     # segment k spans t[k]..t[k+1]
        return all(self.t[k + 1] - self.t[k] <= 0.05 for k in range(max(start, 0), min(i1, len(self.t) - 1)))


class PoseFusion:
    """Integrates master samples into the fused and the odometry-only pose."""

    def __init__(self):
        self.heading = HeadingTrack()
        self.reset_id = -1
        self.reset()

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.reset_id += 1
        self.x, self.y, self.theta = x, y, theta
        self.xo, self.yo, self.thetao = x, y, theta
        self.distance = 0.0
        self.prev: Optional[OdomSample] = None
        self.prev_imu_t: Optional[float] = None
        self.last_row: Optional[list] = None

    # -- master: odometry ---------------------------------------------------------------------

    def odom(self, s: OdomSample, imu_t_end: Optional[float] = None) -> Optional[list]:
        """Fuse one odometry sample (call in time order). Returns the output row (FIELDS) or None
        for the first sample after a reset / gap (it only sets the reference)."""
        p, self.prev = self.prev, s
        if p is None or not s.ok or not p.ok:
            return None
        dt = s.t - p.t
        if not (0 < dt <= MAX_DT):
            return None
        disp = []
        for i in range(len(MODULE_NAMES)):
            d = s.pos[i] - p.pos[i]
            a = math.atan2(math.sin(s.angle[i]) + math.sin(p.angle[i]), math.cos(s.angle[i]) + math.cos(p.angle[i]))
            disp.append((d * math.cos(a), d * math.sin(a)))
        dx, dy, dth_wheel = robot_twist(disp)
        vx, vy, _ = robot_twist([(v * math.cos(a), v * math.sin(a)) for v, a in zip(s.vel, s.angle)])
        dth_imu = None
        if self.heading.covers(p.t, s.t):
            dth_imu = self.heading.at(s.t) - self.heading.at(p.t)
        imu_ok = dth_imu is not None
        dth = dth_imu if imu_ok else dth_wheel
        # fused pose: heading from the IMU, translation from the wheels at the mid-interval heading
        th_mid = self.theta + 0.5 * dth
        c, sn = math.cos(th_mid), math.sin(th_mid)
        self.x += c * dx - sn * dy
        self.y += sn * dx + c * dy
        self.theta += dth
        # odometry-only pose
        tho_mid = self.thetao + 0.5 * dth_wheel
        co, so = math.cos(tho_mid), math.sin(tho_mid)
        self.xo += co * dx - so * dy
        self.yo += so * dx + co * dy
        self.thetao += dth_wheel
        self.distance += math.hypot(dx, dy)
        ct, st = math.cos(self.theta), math.sin(self.theta)
        lead = (imu_t_end - s.t) if imu_t_end is not None else float("nan")
        row = [s.t, self.x, self.y, self.theta, vx, vy, dth / dt,
               ct * vx - st * vy, st * vx + ct * vy, math.hypot(vx, vy), self.distance,
               self.xo, self.yo, self.thetao, dth_wheel / dt,
               (dth_imu / dt) if imu_ok else float("nan"),
               float(imu_ok), 1.0, lead, dt, MASTER_ODOM, float(self.reset_id)]
        self.last_row = row
        return row

    def odom_gap(self):
        """Forget the odometry reference (stream restarted): the next sample starts afresh."""
        self.prev = None

    # -- master: IMU (no swerve data) ---------------------------------------------------------

    def imu_only(self, t: float) -> Optional[list]:
        """Heading-only update at an IMU record end (used while no odometry arrives)."""
        p, self.prev_imu_t = self.prev_imu_t, t
        if p is None or not self.heading.covers(p, t):
            return None
        dt = t - p
        if not (0 < dt <= MAX_DT):
            return None
        dth = self.heading.at(t) - self.heading.at(p)
        self.theta += dth
        nan = float("nan")
        row = [t, self.x, self.y, self.theta, 0.0, 0.0, dth / dt, 0.0, 0.0, 0.0, self.distance,
               self.xo, self.yo, self.thetao, nan, dth / dt, 1.0, 0.0, 0.0, dt, MASTER_IMU, float(self.reset_id)]
        self.last_row = row
        return row
