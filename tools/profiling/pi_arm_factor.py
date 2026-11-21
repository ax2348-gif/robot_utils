#!/usr/bin/env python3
"""One factor-test window (run on the Pi, robot venv; supervised): pi_arm_factor.py LABEL SECONDS ARM BASE
  ARM:  off | enabled (floating, no commands) | track (stream the current TCP pose at 20 Hz: servoing, no motion)
  BASE: off | hold (pose_servo enabled, holding the current pose)
Reports the arm's and swerve's real tick rates (tick counter deltas), arm overruns / read misses / tick_dt from its
stream state, pose_servo's loop, CPU. Leaves the arm disabled (motors on) and pose_servo disabled afterwards."""
import json, os, sys, threading, time

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient  # noqa: E402

LABEL, SEC, ARM, BASE = sys.argv[1], float(sys.argv[2]), sys.argv[3], sys.argv[4]
arm, ps, sw = NodeClient("arm", timeout=15), NodeClient("pose_servo", timeout=10), NodeClient("swerve", timeout=5)
TCK = os.sysconf("SC_CLK_TCK")
stop = threading.Event()


def cpu_busy():
    v = list(map(int, open("/proc/stat").readline().split()[1:8]))
    return sum(v) - v[3] - v[4], sum(v)


def tracker(pose, q0):
    n = 0
    while not stop.is_set():
        t = time.monotonic()
        arm.post("/track", {"pose": pose, "seq": n})
        n += 1
        if n % 20 == 0:
            q = arm.get("/joints")["q"]
            if max(abs(a - b) for a, b in zip(q, q0)) > 0.05:
                print("WATCHDOG: arm drift > 0.05 rad", flush=True)
                stop.set()
        stop.wait(max(0.0, 0.05 - (time.monotonic() - t)))


def node_pids():
    out = {}
    for pid in os.listdir("/proc"):
        if pid.isdigit():
            try:
                cmd = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if b"--name" in cmd and any(b"robot_utils" in c for c in cmd):
                out[cmd[cmd.index(b"--name") + 1].decode()] = int(pid)
    return out


def node_cpu(pids):
    out = {}
    for n, p in pids.items():
        try:
            f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()
            out[n] = (int(f[11]) + int(f[12])) / os.sysconf("SC_CLK_TCK")
        except OSError:
            pass
    return out


def snap():
    a, s = arm.get("/state"), sw.get("/state")
    fr = a.get("feedback_refresh") or {}
    return {"t": time.monotonic(), "arm_tick": a["latest"]["tick"], "arm_over": a.get("overruns"),
            "arm_miss": a["latest"].get("read_misses"), "sw_tick": s["latest"]["tick"], "cpu": cpu_busy(),
            "refreshes": fr.get("refreshes"), "fallbacks": fr.get("fallbacks"), "skip": fr.get("skip"),
            "late": a.get("late_ticks"), "latency": a.get("reply_latency")}


th, tok = None, None
try:
    st = arm.get("/state")
    L = st["latest"]
    pose = {"pos": [L["x"], L["y"], L["z"]], "quat": [L["qx"], L["qy"], L["qz"], L["qw"]]}
    if ARM != "off":
        tok = arm.post("/lease", {"client": "arm factor test (stationary)"})["token"]
        arm.headers["X-Lease-Token"] = tok
        arm.post("/enable", {})
        if ARM == "track":
            th = threading.Thread(target=tracker, args=(pose, arm.get("/joints")["q"]), daemon=True)
            th.start()
        elif ARM == "enabled":
            def renew():
                while not stop.is_set():
                    arm.post("/lease/renew", {})
                    stop.wait(0.3)
            threading.Thread(target=renew, daemon=True).start()
    if BASE == "hold":
        ps.post("/enable", {"client": "arm factor test"})
        ps.post("/goal", {"frame": "local", "x": 0, "y": 0, "theta": 0, "on_reach": "hold"})
    time.sleep(3)
    pids = node_pids()
    c0 = node_cpu(pids)
    a = snap()
    tick_dts, samples = [], 0
    est_ages = []
    t_end = time.monotonic() + SEC
    while time.monotonic() < t_end and not stop.is_set():
        tick_dts.append(arm.get("/state")["latest"]["tick_dt"] * 1e3)
        try:
            ea = (ps.get("/state").get("estimator") or {}).get("age")
            if ea is not None:
                est_ages.append(float(ea))
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.5)
    b = snap()
    dt = b["t"] - a["t"]
    c1 = node_cpu(pids)
    ncpu = {n: round(100 * (c1[n] - c0[n]) / dt, 1) for n in c1 if n in c0}
    est_ages.sort()
    busy = 100 * (b["cpu"][0] - a["cpu"][0]) / (b["cpu"][1] - a["cpu"][1])
    psl = ps.get("/state").get("loop_hz")
    tick_dts.sort()
    print(json.dumps({"label": LABEL, "arm": ARM, "base": BASE, "seconds": round(dt, 1),
                      "arm_hz": round((b["arm_tick"] - a["arm_tick"]) / dt, 1),
                      "arm_overruns_s": round(((b["arm_over"] or 0) - (a["arm_over"] or 0)) / dt, 1),
                      "arm_misses": (b["arm_miss"] or 0) - (a["arm_miss"] or 0),
                      "arm_tick_dt_ms_p50_p90_max": [round(tick_dts[len(tick_dts) // 2], 1),
                                                     round(tick_dts[int(len(tick_dts) * 0.9)], 1), round(tick_dts[-1], 1)],
                      "swerve_hz": round((b["sw_tick"] - a["sw_tick"]) / dt, 1), "pose_servo_loop_hz": psl,
                      "refresh_skip": b["skip"],
                      "refreshes_s": None if b["refreshes"] is None else round((b["refreshes"] - a["refreshes"]) / dt, 1),
                      "fallbacks": None if b["fallbacks"] is None else b["fallbacks"] - a["fallbacks"],
                      "reply_latency_ms": b["latency"],
                      "node_cpu_pct": dict(sorted(ncpu.items(), key=lambda kv: -kv[1])),
                      "estimator_age_ms": None if not est_ages else
                      [round(1e3 * est_ages[len(est_ages) // 2]), round(1e3 * est_ages[int(len(est_ages) * 0.95)]),
                       round(1e3 * est_ages[-1])],
                      "cpu_busy_pct": round(busy, 1)}), flush=True)
finally:
    stop.set()
    if th:
        th.join(timeout=3)
    if BASE == "hold":
        for p in ("/stop", "/disable"):
            try:
                ps.post(p, {})
            except Exception as e:  # noqa: BLE001
                print("cleanup", p, e, flush=True)
    if tok:
        try:
            arm.post("/disable", {})
        except Exception as e:  # noqa: BLE001
            print("cleanup arm", e, flush=True)
