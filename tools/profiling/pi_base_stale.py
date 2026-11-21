#!/usr/bin/env python3
"""Base staleness check (run on the Pi, robot venv; supervised): pi_base_stale.py N_GOALS [OMEGA]

Enables pose_servo and runs N small in-place turns, alternating +0.1 / -0.1 rad (local frame) at OMEGA rad/s, each
to its result. Meanwhile polls pose_servo /state at about 20 Hz and records the estimator stream's age (pose_servo
aborts a goal when it exceeds 0.2 s), the health strings it reports, and the swerve stream's age. Disables
pose_servo afterwards. Prints one JSON line: goal results, aborts by reason, age distribution, time over thresholds.
"""
import json, os, sys, threading, time

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 20
OMEGA = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
ps = NodeClient("pose_servo", timeout=10)
poll = NodeClient("pose_servo", timeout=5)
stop = threading.Event()
ages, sw_ages, health = [], [], {}


def watcher():
    while not stop.is_set():
        try:
            s = poll.get("/state")
            e = s.get("estimator") or {}
            if e.get("age") is not None:
                ages.append(float(e["age"]))
            h = e.get("health")
            health[h] = health.get(h, 0) + 1
            sw = (s.get("swerve") or {}).get("age")
            if sw is not None:
                sw_ages.append(float(sw))
        except Exception as ex:  # noqa: BLE001
            health[f"poll error: {type(ex).__name__}"] = health.get(f"poll error: {type(ex).__name__}", 0) + 1
        stop.wait(0.05)


def pct(a, q):
    if not a:
        return None
    a = sorted(a)
    return round(a[min(len(a) - 1, int(q / 100 * len(a)))], 3)


def main():
    results = []
    ps.post("/enable", {"client": "base staleness check (small turns)"})
    th = threading.Thread(target=watcher, daemon=True)
    th.start()
    try:
        for k in range(N):
            theta = 0.1 if k % 2 == 0 else -0.1
            g = ps.post("/goal", {"frame": "local", "x": 0, "y": 0, "theta": theta, "omega": OMEGA})
            gid, t0 = g.get("goal_id"), time.monotonic()
            res = None
            while time.monotonic() - t0 < 15:
                st = ps.get("/state")
                goal = st.get("goal") or {}
                if goal.get("goal_id") == gid and goal.get("result"):
                    res = (goal["result"], goal.get("reason") or "")
                    break
                time.sleep(0.1)
            if res is None:
                ps.post("/stop", {})
                res = ("timeout", "")
            results.append({"goal": gid, "theta": theta, "result": res[0], "reason": res[1],
                            "s": round(time.monotonic() - t0, 2)})
            print(f"goal {gid} {theta:+.2f}: {res[0]} {res[1]}", flush=True)
            if res[0] in ("aborted", "timeout") and "stale" not in res[1] and "IMU" not in res[1]:
                print("non-staleness abort: stopping the run", flush=True)
                break
            time.sleep(0.5)
    finally:
        stop.set()
        th.join(timeout=2)
        for p in ("/stop", "/disable"):
            try:
                ps.post(p, {})
            except Exception as ex:  # noqa: BLE001
                print("cleanup", p, ex, flush=True)
    reasons = {}
    for r in results:
        if r["result"] != "reached":
            reasons[r["reason"] or r["result"]] = reasons.get(r["reason"] or r["result"], 0) + 1
    print(json.dumps({"goals": len(results), "reached": sum(r["result"] == "reached" for r in results),
                      "not_reached_by_reason": reasons,
                      "estimator_age_s": {"n": len(ages), "p50": pct(ages, 50), "p99": pct(ages, 99),
                                          "max": round(max(ages), 3) if ages else None,
                                          "n_over_0.1": sum(a > 0.1 for a in ages),
                                          "n_over_0.2": sum(a > 0.2 for a in ages)},
                      "swerve_age_s": {"p99": pct(sw_ages, 99), "max": round(max(sw_ages), 3) if sw_ages else None},
                      "health_samples": health}), flush=True)


if __name__ == "__main__":
    main()
