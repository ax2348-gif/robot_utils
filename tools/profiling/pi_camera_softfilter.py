#!/usr/bin/env python3
"""Head camera CPU with the SDK's software depth filter (OB_PROP_DEPTH_SOFT_FILTER_BOOL) on vs off, then
restore its original value (run on the Pi). A device property change only; nothing moves.
Usage: pi_camera_softfilter.py [SEC]"""
import http.client, json, os, socket, sys, time

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
SOCK = f"/run/user/{os.getuid()}/robot/camera.sock"
PROP = "OB_PROP_DEPTH_SOFT_FILTER_BOOL"
TCK = os.sysconf("SC_CLK_TCK")


class U(http.client.HTTPConnection):
    def __init__(self):
        super().__init__("x", timeout=30)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX); self.sock.settimeout(30); self.sock.connect(SOCK)


def call(method, path, body=None):
    c = U(); c.request(method, path, json.dumps(body) if body is not None else None, {"Content-Type": "application/json"})
    r = c.getresponse(); d = r.read(); c.close()
    if r.status >= 400:
        raise RuntimeError(f"{method} {path}: {r.status} {d[:300]}")
    return json.loads(d) if d else None


def value():
    return next(p["value"] for p in call("GET", "/properties?all=true") if p["id"] == PROP)


def pid():
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                cmd = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if b"robot_utils.nodes.orbbec" in cmd and b"camera" in cmd and b"wrist_camera" not in cmd:
                return int(p)


def threads(p):
    out = {}
    for t in os.listdir(f"/proc/{p}/task"):
        try:
            f = open(f"/proc/{p}/task/{t}/stat").read().rsplit(")", 1)[1].split()
            out[int(t)] = int(f[11]) + int(f[12])
        except OSError:
            pass
    return out


def measure(label):
    p = pid(); time.sleep(3)
    a = threads(p); t0 = time.monotonic(); time.sleep(SEC)
    b = threads(p); dt = time.monotonic() - t0
    per = sorted(((100 * (b[t] - a.get(t, b[t])) / TCK / dt, t) for t in b), reverse=True)
    print(f"{label:28s} process {sum(x for x, _ in per):5.1f}%   top: " + ", ".join(f"{t}:{x:.1f}%" for x, t in per[:3]),
          flush=True)


v0 = value()
print(f"{PROP} originally {v0}", flush=True)
try:
    measure(f"soft filter {v0}")
    call("POST", "/properties", {"id": PROP, "value": not v0})
    print(f"set to {value()}", flush=True)
    measure(f"soft filter {not v0}")
finally:
    call("POST", "/properties", {"id": PROP, "value": v0})
    print(f"restored: {value()}", flush=True)
    measure(f"soft filter {v0} (restored)")
