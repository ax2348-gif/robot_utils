"""IMU attitude filter: 1 kHz accel + gyro in, 100 Hz orientation + preintegrated records out.

Runs in the IMU thread, on every sample, in plain Python floats (about 2-3 µs per sample on the
Pi 5; numpy is far slower for 3/4-element math). No SDK calls; easy to test offline.

Model and conventions
- ``q`` = (w, x, y, z): rotation from the IMU frame to a gravity-aligned world frame (world +z
  up, yaw 0 at start or at the last reset). There is no magnetometer: roll/pitch are anchored to
  gravity, yaw only integrates the (bias-corrected) gyro and drifts slowly.
- Gyro integration uses the exact exponential map per sample, with dt from the device clock
  (``t_sensor``), never from host receipt times (samples arrive in ~10 ms bursts).
- Mahony correction: when ``| |a| - g_ref | < accel_gate`` the error between the measured and
  the predicted gravity direction is fed back (gain ``kp``) and integrated into the gyro bias
  (gain ``ki``). Linear acceleration beyond the gate disables the correction for that sample.
- Zero-rate update (ZARU): when the IMU is still (low gyro and accel spread over ``still_s``),
  the gyro bias of all three axes, including yaw which gravity cannot observe, relaxes towards
  the measured rate with time constant ``zaru_tau_s``.
- Calibration (startup, or on request): ``calib_s`` seconds of still data give the gyro bias,
  ``g_ref`` (the accelerometer's magnitude at rest; this unit reads ~3.6 % above 9.81) and the
  initial roll/pitch. If the IMU moves during it the calibration is refused and the previous
  (or stored) values are kept.

Output records (one per window of ``1/output_rate_hz`` by sensor time), columns ``STATE_FIELDS``:
orientation at the window end, the window's delta angle (rotation vector composing every
bias-corrected 1 kHz sample, IMU frame, rad) and delta velocity (sum of a·dt, raw accel incl.
gravity, IMU frame, m/s), mean accel/gyro, current gyro bias, status flags.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# raw sample columns (protocol.IMU_FIELDS)
_T_CAPTURE, _T_SENSOR, _T_DEVICE = 1, 2, 3
_AX, _GX, _TEMP = 4, 7, 10

STATE_FIELDS = (
    "seq",                  # output record counter
    "t_start", "t_end",     # server monotonic s (sensor time) spanned by the window
    "t_capture",            # server monotonic s: SDK host receipt of the window's last sample
    "n",                    # 1 kHz samples in the window
    "qw", "qx", "qy", "qz",  # orientation IMU -> world (z up) at t_end
    "dax", "day", "daz",    # delta angle over the window, rad (IMU frame, bias-corrected)
    "dvx", "dvy", "dvz",    # delta velocity over the window, m/s (IMU frame, raw accel incl. gravity)
    "ax", "ay", "az",       # mean accel, m/s^2
    "gx", "gy", "gz",       # mean bias-corrected gyro, rad/s
    "bgx", "bgy", "bgz",    # current gyro bias estimate, rad/s
    "flags",                # bit field, see FLAG_*
    "accel_used",           # fraction of samples whose accel corrected the attitude
    "temperature",          # degC
)
FLAG_CALIBRATED = 1     # bias/g_ref from a successful still calibration (this run or stored)
FLAG_STILL = 2          # the IMU was still at the end of the window
FLAG_INITIALIZING = 4   # startup calibration not finished yet (orientation not meaningful)
FLAG_GAP = 8            # a sample-time gap (> 5 nominal periods) fell inside the window


@dataclass
class FilterConfig:
    output_rate_hz: float = 100.0
    kp: float = 0.5               # 1/s; tilt converges with ~1/kp s time constant
    ki: float = 0.02              # 1/s^2; bias tracking through the gravity error (roll/pitch axes)
    accel_gate: float = 0.5       # m/s^2 around g_ref
    zaru: bool = True
    zaru_tau_s: float = 5.0
    still_s: float = 0.25
    still_gyro_std: float = 0.004  # rad/s per sample (this unit's noise is ~0.001)
    still_gyro_rate: float = 0.01  # rad/s, |mean rate - bias| (0.57 deg/s)
    still_accel_std: float = 0.06  # m/s^2 per sample (noise ~0.017)
    calib_s: float = 1.0
    max_bias: float = 0.05         # rad/s: a calibration whose mean rate is larger is a rotation, not bias
    nominal_hz: float = 1000.0


@dataclass
class Calibration:
    gyro_bias: tuple = (0.0, 0.0, 0.0)
    g_ref: float = 9.80665
    gyro_std: tuple = (0.0, 0.0, 0.0)
    accel_std: float = 0.0
    temperature: float = float("nan")
    t: float = 0.0                 # server monotonic time it was taken (0 = stored/default)
    source: str = "default"        # default | stored | startup | request

    def to_json(self) -> dict:
        return {"gyro_bias": list(self.gyro_bias), "g_ref": self.g_ref, "gyro_std": list(self.gyro_std),
                "accel_std": self.accel_std, "temperature": self.temperature, "source": self.source}

    @classmethod
    def from_json(cls, d: dict, source: str = "stored") -> "Calibration":
        return cls(gyro_bias=tuple(float(x) for x in d["gyro_bias"]), g_ref=float(d["g_ref"]),
                   gyro_std=tuple(float(x) for x in d.get("gyro_std", (0, 0, 0))),
                   accel_std=float(d.get("accel_std", 0.0)), temperature=float(d.get("temperature", "nan")),
                   source=source)


# -- small quaternion helpers (tuples of floats) ----------------------------------------------

def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def qexp(rx, ry, rz):
    """Rotation vector (rad) -> unit quaternion."""
    a = math.sqrt(rx * rx + ry * ry + rz * rz)
    if a < 1e-9:
        return (1.0, 0.5 * rx, 0.5 * ry, 0.5 * rz)
    s = math.sin(0.5 * a) / a
    return (math.cos(0.5 * a), rx * s, ry * s, rz * s)


def qlog(q):
    """Unit quaternion -> rotation vector (rad), shortest rotation."""
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-9:
        return (2 * x, 2 * y, 2 * z)
    a = 2 * math.atan2(s, w)
    return (x / s * a, y / s * a, z / s * a)


def qnorm(q):
    n = 1.0 / math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] * n, q[1] * n, q[2] * n, q[3] * n)


def q_from_gravity(ax, ay, az):
    """Orientation (IMU -> world, yaw 0) that maps the measured specific force to world +z."""
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-6:
        return (1.0, 0.0, 0.0, 0.0)
    ux, uy, uz = ax / n, ay / n, az / n
    # rotate u onto (0, 0, 1): axis u x z, angle acos(uz)
    rx, ry = uy, -ux
    s = math.sqrt(rx * rx + ry * ry)
    if s < 1e-9:
        return (1.0, 0.0, 0.0, 0.0) if uz > 0 else (0.0, 1.0, 0.0, 0.0)
    a = math.atan2(s, uz)
    return qexp(rx / s * a, ry / s * a, 0.0)


def euler_zyx(q):
    """(roll, pitch, yaw) in degrees, ZYX convention, for display."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class _Still:
    """Running window statistics for still detection (fixed number of samples)."""

    def __init__(self, n: int):
        self.n = max(2, n)
        self.buf = []
        self.i = 0
        self.s = [0.0] * 7       # sums of gx, gy, gz, |a|, gx2, gy2, gz2 ...
        self.s2 = [0.0] * 4      # sums of squares of gx, gy, gz, |a|

    def add(self, gx, gy, gz, an):
        v = (gx, gy, gz, an)
        if len(self.buf) < self.n:
            self.buf.append(v)
        else:
            old = self.buf[self.i]
            self.buf[self.i] = v
            self.i = (self.i + 1) % self.n
            for k in range(4):
                self.s[k] -= old[k]
                self.s2[k] -= old[k] * old[k]
        for k in range(4):
            self.s[k] += v[k]
            self.s2[k] += v[k] * v[k]

    def full(self):
        return len(self.buf) >= self.n

    def mean(self, k):
        return self.s[k] / len(self.buf)

    def std(self, k):
        m = len(self.buf)
        return math.sqrt(max(0.0, self.s2[k] / m - (self.s[k] / m) ** 2))


class AttitudeFilter:
    def __init__(self, cfg: Optional[FilterConfig] = None, calibration: Optional[Calibration] = None):
        self.cfg = cfg or FilterConfig()
        self.cal = calibration or Calibration()
        self.calibrated = calibration is not None and calibration.source != "default"
        self.on_calibration = None        # callback(Calibration) after a successful calibration
        self.on_calibration_failed = None  # callback(str)
        self.reset(recalibrate=True)

    # -- control (call from the IMU thread, or between feeds) --------------------------------

    def configure(self, cfg: FilterConfig):
        rate_changed = cfg.output_rate_hz != self.cfg.output_rate_hz or cfg.still_s != self.cfg.still_s
        self.cfg = cfg
        if rate_changed:
            self._still = _Still(int(cfg.still_s * cfg.nominal_hz))

    def reset(self, recalibrate: bool = False):
        """Restart the orientation from gravity (yaw 0). With recalibrate, first gather
        ``calib_s`` of still data (startup); otherwise use the current bias at once."""
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.bias = list(self.cal.gyro_bias)
        self._still = _Still(int(self.cfg.still_s * self.cfg.nominal_hz))
        self._t_prev = None
        self._win = None
        self.seq = 0
        self._calib = [] if recalibrate else None
        self._calib_source = "startup"
        self._need_attitude = True
        self.still = False

    def request_calibration(self, seconds: Optional[float] = None, source: str = "request"):
        """Collect still data for a new calibration; orientation keeps running meanwhile."""
        self._calib = []
        self._calib_len = seconds if seconds is not None else self.cfg.calib_s
        self._calib_source = source

    @property
    def initializing(self) -> bool:
        return self._calib is not None and self._calib_source == "startup"

    # -- processing ---------------------------------------------------------------------------

    def feed(self, rows: np.ndarray) -> list[tuple]:
        """Process raw sample rows (protocol.IMU_FIELDS columns). Returns output rows."""
        out = []
        for r in rows.tolist():
            res = self._sample(r)
            if res is not None:
                out.append(res)
        return out

    def _sample(self, r):
        cfg = self.cfg
        t = r[_T_SENSOR]
        if t != t:           # no global timestamp: fall back to the device clock (us)
            t = r[_T_DEVICE] * 1e-6
        ax, ay, az = r[_AX], r[_AX + 1], r[_AX + 2]
        gx, gy, gz = r[_GX], r[_GX + 1], r[_GX + 2]
        an = math.sqrt(ax * ax + ay * ay + az * az)
        nominal = 1.0 / cfg.nominal_hz
        gap = False
        if self._t_prev is None:
            dt = nominal
        else:
            dt = t - self._t_prev
            if dt <= 0 or dt > 5 * nominal:   # out of order or a gap: don't integrate across it
                gap = dt > 5 * nominal
                dt = nominal if dt <= 0 else min(dt, 5 * nominal)
        self._t_prev = t

        # still detection on raw gyro/accel
        self._still.add(gx, gy, gz, an)
        b = self.bias
        if self._still.full():
            self.still = (max(self._still.std(0), self._still.std(1), self._still.std(2)) < cfg.still_gyro_std
                          and self._still.std(3) < cfg.still_accel_std
                          and math.sqrt((self._still.mean(0) - b[0]) ** 2 + (self._still.mean(1) - b[1]) ** 2
                                        + (self._still.mean(2) - b[2]) ** 2) < cfg.still_gyro_rate)
        else:
            self.still = False

        if self._calib is not None:
            self._calibrate_step(r, ax, ay, az, an, gx, gy, gz)

        if self._need_attitude and not self.initializing:
            self.q = q_from_gravity(ax, ay, az)
            self._need_attitude = False

        # ZARU: when still, relax all three biases towards the measured rate
        if cfg.zaru and self.still and not self.initializing:
            k = min(1.0, dt / cfg.zaru_tau_s)
            b[0] += (self._still.mean(0) - b[0]) * k
            b[1] += (self._still.mean(1) - b[1]) * k
            b[2] += (self._still.mean(2) - b[2]) * k

        wx, wy, wz = gx - b[0], gy - b[1], gz - b[2]     # bias-corrected rate (for delta angle)
        cx, cy, cz = wx, wy, wz                          # + attitude correction (for q only)
        used = False
        if an > 1e-6 and abs(an - self.cal.g_ref) < cfg.accel_gate and not self.initializing:
            w, x, y, z = self.q
            # predicted gravity direction in the IMU frame: R^T e_z
            vx, vy, vz = 2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z
            ux, uy, uz = ax / an, ay / an, az / an
            ex, ey, ez = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            cx += cfg.kp * ex
            cy += cfg.kp * ey
            cz += cfg.kp * ez
            b[0] -= cfg.ki * ex * dt
            b[1] -= cfg.ki * ey * dt
            b[2] -= cfg.ki * ez * dt
            used = True
        self.q = qnorm(qmul(self.q, qexp(cx * dt, cy * dt, cz * dt)))
        return self._window(r, t, dt, wx, wy, wz, ax, ay, az, used, gap)

    def _window(self, r, t, dt, wx, wy, wz, ax, ay, az, used, gap):
        win = self._win
        if win is None:
            win = self._win = {"t0": t - dt, "dq": (1.0, 0.0, 0.0, 0.0), "dv": [0.0, 0.0, 0.0],
                               "sa": [0.0, 0.0, 0.0], "sg": [0.0, 0.0, 0.0], "n": 0, "used": 0, "gap": False}
        win["dq"] = qmul(win["dq"], qexp(wx * dt, wy * dt, wz * dt))
        dv, sa, sg = win["dv"], win["sa"], win["sg"]
        dv[0] += ax * dt; dv[1] += ay * dt; dv[2] += az * dt
        sa[0] += ax; sa[1] += ay; sa[2] += az
        sg[0] += wx; sg[1] += wy; sg[2] += wz
        win["n"] += 1
        win["used"] += used
        win["gap"] |= gap
        if t - win["t0"] < 1.0 / self.cfg.output_rate_hz - 0.25 / self.cfg.nominal_hz:
            return None
        self._win = None
        n = win["n"]
        flags = ((FLAG_CALIBRATED if self.calibrated else 0) | (FLAG_STILL if self.still else 0)
                 | (FLAG_INITIALIZING if self.initializing else 0) | (FLAG_GAP if win["gap"] else 0))
        self.seq += 1
        da = qlog(win["dq"])
        return (float(self.seq), win["t0"], t, r[_T_CAPTURE], float(n), *self.q, *da, *dv,
                sa[0] / n, sa[1] / n, sa[2] / n, sg[0] / n, sg[1] / n, sg[2] / n, *self.bias,
                float(flags), win["used"] / n, r[_TEMP])

    def _calibrate_step(self, r, ax, ay, az, an, gx, gy, gz):
        c = self._calib
        c.append((gx, gy, gz, ax, ay, az, an, r[_TEMP]))
        seconds = self._calib_len if self._calib_source != "startup" else self.cfg.calib_s
        if len(c) < seconds * self.cfg.nominal_hz:
            return
        A = np.array(c)
        self._calib = None
        g_std = A[:, 0:3].std(0)
        a_std = float(A[:, 6].std())
        g_mean = A[:, 0:3].mean(0)
        if (g_std.max() >= self.cfg.still_gyro_std or a_std >= self.cfg.still_accel_std
                or np.abs(g_mean).max() >= self.cfg.max_bias):
            msg = (f"IMU moved during calibration (gyro mean {np.round(g_mean, 4).tolist()} std "
                   f"{np.round(g_std, 4).tolist()} rad/s, |a| std {a_std:.3f} m/s^2); kept the previous calibration")
            if self.on_calibration_failed:
                self.on_calibration_failed(msg)
            if self._calib_source == "startup":
                self._need_attitude = True   # start anyway, from gravity, with the old bias
            return
        self.cal = Calibration(gyro_bias=tuple(float(x) for x in g_mean), g_ref=float(A[:, 6].mean()),
                               gyro_std=tuple(float(x) for x in g_std), accel_std=a_std,
                               temperature=float(np.nanmean(A[:, 7])), t=r[_T_CAPTURE], source=self._calib_source)
        self.bias = list(self.cal.gyro_bias)
        self.calibrated = True
        if self._calib_source == "startup":
            m = A[:, 3:6].mean(0)
            self.q = q_from_gravity(*m)
            self._need_attitude = False
        if self.on_calibration:
            self.on_calibration(self.cal)

    def status(self) -> dict:
        roll, pitch, yaw = euler_zyx(self.q)
        return {"initializing": self.initializing, "calibrated": self.calibrated, "still": self.still,
                "calibrating": self._calib is not None,
                "q": [round(v, 6) for v in self.q],
                "rpy_deg": [round(roll, 2), round(pitch, 2), round(yaw, 2)],
                "gyro_bias": [round(v, 7) for v in self.bias], "calibration": self.cal.to_json()}
