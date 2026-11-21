#!/usr/bin/env python3
"""Head camera IMU sample rate vs CPU (run on the Pi, robot venv): for each rate, the camera and estimator
processes' CPU over SEC s, the camera's IMU stream rate as the estimator sees it, and the estimator's
IMU coverage / mode. Restores the original rate. A settings change only; nothing moves.
Usage: pi_imu_rate.py [SEC]"""
import json, os, sys, time

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient  # noqa: E402

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TCK = os.sysconf("SC_CLK_TCK")
cam, est = NodeClient("camera", timeout=30), NodeClient("estimator", timeout=10)


def pid(name):
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                cmd = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if b"--name" in cmd and cmd[cmd.index(b"--name") + 1] == name.encode():
                return int(p)


def cpu(p):
    f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()
    return (int(f[11]) + int(f[12])) / TCK


def measure(label):
    time.sleep(4)
    pc, pe = pid("camera"), pid("estimator")
    c0, e0, t0 = cpu(pc), cpu(pe), time.monotonic()
    time.sleep(SEC)
    dt = time.monotonic() - t0
    c, e = 100 * (cpu(pc) - c0) / dt, 100 * (cpu(pe) - e0) / dt
    s = est.get("/state")
    imu_in = s["inputs"]["imu"]
    print(f"{label:10s} camera {c:5.1f}%  estimator {e:5.1f}%  | IMU stream at estimator {imu_in['rate_hz']} Hz, "
          f"connected {imu_in['connected']}, error {imu_in['error']} | estimator mode {s.get('mode')}, "
          f"imu_coverage {json.dumps(s.get('imu_coverage'))[:80]}", flush=True)


orig = cam.get("/settings")["imu"]["sample_rate"]
print("original IMU sample rate:", orig, flush=True)
try:
    for r in ("1kHz", "500Hz", "200Hz", "100Hz"):
        cam.post("/settings", {"imu": {"sample_rate": r}})
        measure(r)
finally:
    cam.post("/settings", {"imu": {"sample_rate": orig}})
    print("restored:", cam.get("/settings")["imu"]["sample_rate"], flush=True)
