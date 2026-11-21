#!/usr/bin/env python3
"""Realistic control-loop check (run on the Pi, robot venv; supervised):

  ~/robot_utils/.venv/bin/python pi_realistic_servo.py SECONDS

- arm: lease, enable, then stream its CURRENT TCP pose to /track at 20 Hz (servoing engaged, no motion); a
  watchdog stops if any joint drifts > DRIFT_RAD or the arm faults; afterwards disable (motors stay on, as found);
- base: pose_servo enable, hold the current pose (on_reach hold); at 1/3 of the window turn +0.1 rad, at 2/3 back;
  afterwards stop + disable;
- meanwhile pi_load_sample.py runs for the window (CPU, streams, arm/swerve loop timing, /health latency) and the
  arm's and pose_servo's own loop stats are read before/after.
"""
import json, os, subprocess, sys, threading, time

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient  # noqa: E402

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
DRIFT_RAD = 0.05
arm = NodeClient("arm", timeout=15)
ps = NodeClient("pose_servo", timeout=10)
stop = threading.Event()
log = []


def note(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    log.append(line)
    print(line, flush=True)


def arm_loop(token, pose, q0):
    arm.headers["X-Lease-Token"] = token
    n = errs = 0
    while not stop.is_set():
        t = time.monotonic()
        try:
            r = arm.post("/track", {"pose": pose, "seq": n})
            n += 1
            if r and r.get("ok") is False:
                errs += 1
        except Exception as e:  # noqa: BLE001
            errs += 1
            note(f"arm track error: {e}")
            if errs > 20:
                stop.set()
        if n % 20 == 0:
            j = arm.get("/joints")
            drift = max(abs(a - b) for a, b in zip(j["q"], q0))
            if drift > DRIFT_RAD:
                note(f"WATCHDOG: arm drift {drift:.3f} rad: stopping")
                stop.set()
        stop.wait(max(0.0, 0.05 - (time.monotonic() - t)))
    note(f"arm: {n} track commands, {errs} errors")


def main():
    st0 = {"arm": arm.get("/state"), "ps": ps.get("/state")}
    note(f"arm {st0['arm']['state']} loop {st0['arm'].get('loop_hz')} Hz overruns {st0['arm'].get('overruns')}; "
         f"pose_servo {st0['ps']['status']} loop {st0['ps'].get('loop_hz')} Hz")
    L = st0["arm"]["latest"]
    pose = {"pos": [L["x"], L["y"], L["z"]], "quat": [L["qx"], L["qy"], L["qz"], L["qw"]]}
    q0 = arm.get("/joints")["q"]
    token = arm.post("/lease", {"client": "realistic loop test (stationary)"})["token"]
    arm.headers["X-Lease-Token"] = token
    th = None
    sampler = None
    try:
        note(f"arm enable: {arm.post('/enable', {})}")
        th = threading.Thread(target=arm_loop, args=(token, pose, q0), daemon=True)
        th.start()
        note(f"pose_servo enable: {ps.post('/enable', {'client': 'realistic loop test'}).get('enabled')}")
        g = ps.post("/goal", {"frame": "local", "x": 0, "y": 0, "theta": 0, "on_reach": "hold"})
        note(f"base hold goal {g.get('goal_id')}")
        sampler = subprocess.Popen([sys.executable, os.path.expanduser("~/profiling/pi_load_sample.py"), str(int(SEC)),
                                    "realistic"], stdout=open(os.path.expanduser("~/profiling/realistic.json"), "w"))
        t0 = time.monotonic()
        turns = [(SEC / 3, 0.1), (2 * SEC / 3, -0.1)]
        while time.monotonic() - t0 < SEC and not stop.is_set():
            if turns and time.monotonic() - t0 >= turns[0][0]:
                _, th_ = turns.pop(0)
                g = ps.post("/goal", {"frame": "local", "x": 0, "y": 0, "theta": th_, "omega": 0.2, "on_reach": "hold"})
                note(f"base turn {th_:+.2f} rad (goal {g.get('goal_id')})")
            s = ps.get("/state")
            if s.get("status") in ("disabled",) or (s.get("estimator") or {}).get("health") not in ("ok", None):
                note(f"pose_servo: {s.get('status')} / estimator {s.get('estimator')}")
            time.sleep(1.0)
    finally:
        stop.set()
        if th:
            th.join(timeout=3)
        for call in (lambda: ps.post("/stop", {}), lambda: ps.post("/disable", {}), lambda: arm.post("/disable", {})):
            try:
                call()
            except Exception as e:  # noqa: BLE001
                note(f"cleanup: {e}")
        if sampler:
            sampler.wait(timeout=SEC + 60)
        st1 = {"arm": arm.get("/state"), "ps": ps.get("/state")}
        note(f"after: arm {st1['arm']['state']} overruns {st1['arm'].get('overruns')} "
             f"(+{(st1['arm'].get('overruns') or 0) - (st0['arm'].get('overruns') or 0)}); "
             f"latency {json.dumps(st1['arm'].get('latency') or st1['arm'].get('reply_latency'))[:200]}; "
             f"pose_servo {st1['ps']['status']} goal {json.dumps((st1['ps'].get('goal') or {}).get('result'))}")
        json.dump({"log": log, "before": st0, "after": st1}, open(os.path.expanduser("~/profiling/realistic_states.json"), "w"),
                  default=str)


if __name__ == "__main__":
    main()
