#!/usr/bin/env python3
"""Stream fan-out cost (run on the Pi, robot venv): for K = 0, 1, 2, 4 extra local subscribers to a node's
/stream (each its own process, reading and discarding bytes), the producing node's user / system CPU and
context switches, and the delivery lag one measuring subscriber sees (receipt - row t, same clock).
Read-only. Usage: pi_sub_scaling.py NODE [SEC] [PARAMS]   e.g. pi_sub_scaling.py swerve 20 max_hz=100"""
import http.client, os, socket, subprocess, sys, threading, time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.protocol import RecordReader, RowsDecoder  # noqa: E402

NODE = sys.argv[1]
SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
PARAMS = sys.argv[3] if len(sys.argv) > 3 else ""
SOCK = f"/run/user/{os.getuid()}/robot/{NODE}.sock"
TCK = os.sysconf("SC_CLK_TCK")
DRAIN = f"""
import socket
s = socket.socket(socket.AF_UNIX); s.connect({SOCK!r})
s.sendall(b"GET /stream?{PARAMS} HTTP/1.1\\r\\nHost: x\\r\\n\\r\\n")
while s.recv(65536):
    pass
"""


class U(http.client.HTTPConnection):
    def __init__(self):
        super().__init__("x", timeout=10)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX); self.sock.settimeout(10); self.sock.connect(SOCK)


def pid():
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                cmd = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if b"--name" in cmd and cmd[cmd.index(b"--name") + 1] == NODE.encode():
                return int(p)


def proc(p):
    f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()
    v = 0
    for t in os.listdir(f"/proc/{p}/task"):
        try:
            for line in open(f"/proc/{p}/task/{t}/status"):
                if line.startswith("voluntary_ctxt"):
                    v += int(line.split()[1])
        except OSError:
            pass
    return int(f[11]) / TCK, int(f[12]) / TCK, v


def measure_lag(stop, out):
    c = U(); c.request("GET", "/stream?" + PARAMS); r = c.getresponse()
    rr, dec, lags = RecordReader(), RowsDecoder(), []
    tf = None
    while not stop.is_set():
        chunk = r.read1(65536)
        now = time.monotonic()
        for h, payload in rr.feed(chunk):
            rows = dec.decode(h, payload)
            if rows is not None:
                tf = tf or ("t_tick" if "t_tick" in rows.dtype.names else "t")
                lags.extend(now - rows[tf])
    out.extend(lags)


def run(k):
    subs = [subprocess.Popen([sys.executable, "-c", DRAIN]) for _ in range(k)]
    time.sleep(2)
    p = pid()
    stop, lags = threading.Event(), []
    th = threading.Thread(target=measure_lag, args=(stop, lags), daemon=True)
    th.start()
    time.sleep(1)
    a, t0 = proc(p), time.monotonic()
    time.sleep(SEC)
    b, dt = proc(p), time.monotonic() - t0
    stop.set()
    th.join(timeout=5)
    for s in subs:
        s.terminate()
    for s in subs:
        s.wait()
    lag = np.array(lags) * 1e3
    print(f"{NODE} +{k} subscribers: user {100*(b[0]-a[0])/dt:5.1f}%  sys {100*(b[1]-a[1])/dt:5.1f}%  "
          f"vol ctx/s {round((b[2]-a[2])/dt):5d}  | lag ms p50 {np.percentile(lag,50):5.1f}  p99 {np.percentile(lag,99):5.1f}"
          f"  max {lag.max():6.1f}  (n={len(lag)})", flush=True)
    time.sleep(2)


for k in (0, 1, 2, 4):
    run(k)
