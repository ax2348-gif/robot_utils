#!/usr/bin/env python3
"""IMU rate vs kernel work (run on the Pi): interrupts/s per source, the camera process's context switches/s and
user vs system CPU, at 1 kHz and 100 Hz; restores the original rate. Usage: pi_imu_irq.py [SEC]"""
import os, sys, time
sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient  # noqa: E402

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
TCK = os.sysconf("SC_CLK_TCK")
cam = NodeClient("camera", timeout=30)


def pid():
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                cmd = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if b"--name" in cmd and cmd[cmd.index(b"--name") + 1] == b"camera":
                return int(p)


def irqs():
    lines = open("/proc/interrupts").read().splitlines()
    n = len(lines[0].split())
    out = {}
    for l in lines[1:]:
        p = l.split()
        out[" ".join(p[1 + n:])[-40:] or p[0]] = sum(int(x) for x in p[1:1 + n] if x.isdigit())
    return out


def proc(p):
    f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()
    v = n = 0
    for t in os.listdir(f"/proc/{p}/task"):
        try:
            for line in open(f"/proc/{p}/task/{t}/status"):
                if line.startswith("voluntary_ctxt"):
                    v += int(line.split()[1])
                elif line.startswith("nonvoluntary_ctxt"):
                    n += int(line.split()[1])
        except OSError:
            pass
    return int(f[11]) / TCK, int(f[12]) / TCK, v, n


def measure(label):
    time.sleep(4)
    p = pid()
    i0, a, t0 = irqs(), proc(p), time.monotonic()
    time.sleep(SEC)
    i1, b, dt = irqs(), proc(p), time.monotonic() - t0
    top = sorted(((round((i1[k] - i0.get(k, 0)) / dt), k) for k in i1), reverse=True)[:5]
    print(f"{label:6s} camera user {100*(b[0]-a[0])/dt:5.1f}%  sys {100*(b[1]-a[1])/dt:5.1f}%  "
          f"vol ctx/s {round((b[2]-a[2])/dt)}  invol ctx/s {round((b[3]-a[3])/dt)}  | IRQs/s: "
          + ", ".join(f"{k.strip()} {v}" for v, k in top), flush=True)


orig = cam.get("/settings")["imu"]["sample_rate"]
try:
    for r in ("1kHz", "100Hz"):
        cam.post("/settings", {"imu": {"sample_rate": r}})
        measure(r)
finally:
    cam.post("/settings", {"imu": {"sample_rate": orig}})
    print("restored:", cam.get("/settings")["imu"]["sample_rate"], flush=True)
