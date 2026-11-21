#!/usr/bin/env python3
"""Simulated remote viewers (run on the Mac): open the same requests the robot's node pages make, through the
robot's nginx proxy over the robot LAN like a browser, and read them at full rate (discarding the data).

  python3 sim_viewers.py SECONDS [CAMERA_VIEWERS]

Per camera page (head and wrist, CAMERA_VIEWERS each, default 2): the live JPEG stream (all streams, as the page
asks), the IMU stream (head), and the SSE events. One 3d_perception page (its stream, events, calibration every
5 s), one arm page (stream?max_hz=30, events, links at 10 Hz, kinematics at 5 Hz) and one agent_interface page
(events). Prints bytes received per request at the end. Read-only.
"""
import os, sys, threading, time, urllib.request

HOST = os.environ.get("ROBOT_NODES", "http://botcore-desktop.local/nodes")   # robot LAN (mDNS), not Tailscale
SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 120
CAMV = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 2
# or a page list instead of the count, e.g. camera,camera,wrist,perception,arm,agent
PAGES = sys.argv[2].split(",") if len(sys.argv) > 2 and not sys.argv[2].isdigit() else None
stop = threading.Event()
got: dict = {}
errors: dict = {}


def stream(path):
    while not stop.is_set():
        try:
            with urllib.request.urlopen(f"{HOST}/{path}", timeout=10) as r:
                while not stop.is_set():
                    b = r.read1(65536) if hasattr(r, "read1") else r.read(65536)
                    if not b:
                        break
                    got[path] = got.get(path, 0) + len(b)
        except Exception as e:  # noqa: BLE001
            errors[path] = f"{type(e).__name__}: {e}"[:120]
            stop.wait(1.0)


def poll(path, hz):
    while not stop.is_set():
        t = time.monotonic()
        try:
            with urllib.request.urlopen(f"{HOST}/{path}", timeout=5) as r:
                got[path] = got.get(path, 0) + len(r.read())
        except Exception as e:  # noqa: BLE001
            errors[path] = f"{type(e).__name__}: {e}"[:120]
        stop.wait(max(0.0, 1.0 / hz - (time.monotonic() - t)))


PAGE_JOBS = {
    "camera": [(stream, ("camera/stream?streams=color,depth,ir_left,ir_right&mode=jpeg",)),
               (stream, ("camera/imu/stream",)), (stream, ("camera/events",))],
    "wrist": [(stream, ("wrist_camera/stream?streams=color,depth,ir_left,ir_right&mode=jpeg",)),
              (stream, ("wrist_camera/events",))],
    "perception": [(stream, ("3d_perception/stream",)), (stream, ("3d_perception/events",)),
                   (poll, ("3d_perception/calibration", 0.2))],
    "arm": [(stream, ("arm/stream?max_hz=30",)), (stream, ("arm/events",)),
            (poll, ("arm/links", 10)), (poll, ("arm/kinematics", 5))],
    "agent": [(stream, ("agent_interface/events",))],
}


def main():
    jobs = []
    if PAGES is not None:
        for pg in PAGES:
            jobs += PAGE_JOBS[pg]
        return run(jobs)
    for _ in range(CAMV):
        jobs += [(stream, ("camera/stream?streams=color,depth,ir_left,ir_right&mode=jpeg",)),
                 (stream, ("camera/imu/stream",)), (stream, ("camera/events",)),
                 (stream, ("wrist_camera/stream?streams=color,depth,ir_left,ir_right&mode=jpeg",)),
                 (stream, ("wrist_camera/events",))]
    jobs += [(stream, ("3d_perception/stream",)), (stream, ("3d_perception/events",)),
             (poll, ("3d_perception/calibration", 0.2)),
             (stream, ("arm/stream?max_hz=30",)), (stream, ("arm/events",)),
             (poll, ("arm/links", 10)), (poll, ("arm/kinematics", 5)),
             (stream, ("agent_interface/events",))]
    return run(jobs)


def run(jobs):
    ths = [threading.Thread(target=f, args=a, daemon=True) for f, a in jobs]
    for t in ths:
        t.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < SEC:
        time.sleep(1)
    stop.set()
    dt = time.monotonic() - t0
    print(f"{len(jobs)} requests for {dt:.0f} s; received per request (kB/s):", flush=True)
    for k in sorted(got):
        print(f"  {got[k] / 1024 / dt:9.1f}  {k}", flush=True)
    for k, v in errors.items():
        print(f"  ERROR {k}: {v}", flush=True)


if __name__ == "__main__":
    main()
