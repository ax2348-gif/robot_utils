#!/usr/bin/env python3
"""Enable the arm and swerve control loops for a while WITHOUT commanding any motion (run on the Pi).

  ~/robot_utils/.venv/bin/python pi_control_hold.py 70

Takes both leases, POST /enable on each (the arm floats: motors on, gravity compensated, kp = kd = 0; the swerve
accepts commands but none are sent), renews the leases every second, then disables both and switches the arm motors back off (arm e-stop + clear: damping, no
motion). A watchdog disables at
once if any arm joint drifts more than DRIFT_RAD from where it started or moves faster than QD_MAX, or if
either node reports a fault. Never sends /command, /track, /servo or anything that moves.
"""
import json
import os
import socket
import sys
import threading
import time
import http.client

RUN = f"/run/user/{os.getuid()}/robot"
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
DRIFT_RAD, QD_MAX = 0.05, 0.2


class UConn(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__("localhost", timeout=15)
        self.path_ = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path_)


def call(node, method, path, body=None, token=None):
    c = UConn(f"{RUN}/{node}.sock")
    h = {"Content-Type": "application/json"}
    if token:
        h["X-Lease-Token"] = token
    c.request(method, path, json.dumps(body) if body is not None else None, h)
    r = c.getresponse()
    data = r.read()
    c.close()
    out = json.loads(data) if data else {}
    if r.status >= 400:
        raise RuntimeError(f"{node} {method} {path}: {r.status} {out}")
    return out


def renewer(toks, stop):
    while not stop.is_set():
        for n, t in toks.items():
            try:
                call(n, "POST", "/lease/renew", token=t)
            except Exception as e:  # noqa: BLE001
                print(n, "renew failed:", e, flush=True)
        stop.wait(0.3)


def main():
    toks, stop = {}, threading.Event()
    th = threading.Thread(target=renewer, args=(toks, stop), daemon=True)
    drift = 0.0
    try:
        for n in ("arm", "swerve"):
            for attempt in range(20):            # a lease left by a previous run lapses within its 1 s heartbeat
                try:
                    toks[n] = call(n, "POST", "/lease", {"client": "profiling (no motion)"})["token"]
                    break
                except RuntimeError as e:
                    if "409" not in str(e) or attempt == 19:
                        raise
                    time.sleep(0.5)
        th.start()                               # leases renewed every 0.3 s from here (heartbeat 1 s)
        q0 = call("arm", "GET", "/joints")["q"]
        for n in ("swerve", "arm"):
            print(n, "enable:", call(n, "POST", "/enable", token=toks[n]), flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < DUR:
            j = call("arm", "GET", "/joints")
            drift = max(abs(a - b) for a, b in zip(j["q"], q0))
            qd = max(abs(v) for v in j["qd"])
            st = call("arm", "GET", "/state")
            sw = call("swerve", "GET", "/state")
            if drift > DRIFT_RAD or qd > QD_MAX or st.get("state") == "fault" or not sw.get("enabled"):
                print(f"WATCHDOG: drift {drift:.3f} rad, qd {qd:.3f} rad/s, arm {st.get('state')}, "
                      f"swerve enabled {sw.get('enabled')}: stopping", flush=True)
                break
            time.sleep(1.0)
        print(f"held {time.monotonic() - t0:.0f} s; max drift at end {drift:.4f} rad", flush=True)
    finally:
        for n in ("arm", "swerve"):
            if n in toks:
                try:
                    print(n, "disable:", call(n, "POST", "/disable", token=toks[n]), flush=True)
                except Exception as e:  # noqa: BLE001
                    print(n, "disable FAILED:", e, flush=True)
        if "arm" in toks:                        # back to motors off (damping) as found: e-stop, then clear it
            try:
                call("arm", "POST", "/estop", {"reason": "profiling done: motors off as before", "source": "profiling"})
                time.sleep(0.3)
                call("arm", "POST", "/estop/clear", token=toks["arm"])
                print("arm:", call("arm", "GET", "/state").get("state"), flush=True)
            except Exception as e:  # noqa: BLE001
                print("arm motors-off FAILED:", e, flush=True)
        stop.set()


if __name__ == "__main__":
    main()
