#!/usr/bin/env python3
"""Head camera CPU per stream (run on the Pi): measure the camera node's process CPU (and its busiest native
threads) for SEC seconds under each stream configuration, then restore the original settings exactly.
Settings changes only (POST /settings): nothing moves. Usage: pi_camera_toggle.py [SEC] [NODE]"""
import http.client, json, os, socket, sys, time

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
NODE = sys.argv[2] if len(sys.argv) > 2 else "camera"
SOCK = f"/run/user/{os.getuid()}/robot/{NODE}.sock"
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


def pid():
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                cmd = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if b"robot_utils.nodes.orbbec" in cmd and NODE.encode() in cmd:
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
    p = pid()
    time.sleep(3)                                   # let the pipeline settle after a restart
    a = threads(p); t0 = time.monotonic()
    time.sleep(SEC)
    b = threads(p); dt = time.monotonic() - t0
    per = sorted(((100 * (b[t] - a.get(t, b[t])) / TCK / dt, t) for t in b), reverse=True)
    tot = sum(x for x, _ in per)
    print(f"{label:34s} process {tot:5.1f}%   top threads: " + ", ".join(f"{t}:{x:.1f}%" for x, t in per[:4]), flush=True)


orig = call("GET", "/settings")
streams0 = orig["streams"]
imu0 = orig["imu"]["enabled"]
print("original:", json.dumps({"streams": streams0, "color": orig.get("color_profile"), "stereo": orig.get("stereo_mode"),
                               "imu": imu0}), flush=True)
try:
    measure("as configured")
    call("POST", "/settings", {"imu": {"enabled": False}}); measure("IMU off")
    call("POST", "/settings", {"imu": {"enabled": imu0}, "streams": {"depth": False}}); measure("depth off (colour + IMU)")
    call("POST", "/settings", {"streams": {"depth": streams0.get("depth", True), "color": False}}); measure("colour off (depth + IMU)")
    call("POST", "/settings", {"streams": {"color": False, "depth": False}}); measure("colour + depth off (IMU only)")
    call("POST", "/settings", {"streams": {"color": False, "depth": False}, "imu": {"enabled": False}}); measure("everything off")
finally:
    call("POST", "/settings", {"streams": streams0, "imu": {"enabled": imu0}})
    now = call("GET", "/settings")
    ok = now["streams"] == streams0 and now["imu"]["enabled"] == imu0
    print("restored:", "OK" if ok else f"MISMATCH {now['streams']} imu {now['imu']['enabled']}", flush=True)
    measure("as configured (after restore)")
