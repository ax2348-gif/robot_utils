#!/usr/bin/env python3
"""Supervised in-place turn through pose_servo (run on the Pi): pi_turn.py THETA_RAD [OMEGA] | pi_turn.py off
Enables pose_servo if needed (takes the swerve lease), sends a local goal (0, 0, THETA) at OMEGA rad/s, waits for
the result, prints it. 'off' disables pose_servo (releases the swerve). Ctrl-C / any error sends /stop."""
import json, os, sys, time
sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient

ps = NodeClient("pose_servo", timeout=10)
if sys.argv[1] == "off":
    print(ps.post("/disable", {})); sys.exit()
theta = float(sys.argv[1]); omega = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
st = ps.get("/state")
print("estimator health:", st.get("estimator", {}).get("health", st.get("health")))
if not st.get("enabled"):
    print("enable:", {k: v for k, v in ps.post("/enable", {"client": "supervised filter-test turn"}).items() if k in ("enabled",)})
try:
    g = ps.post("/goal", {"frame": "local", "x": 0.0, "y": 0.0, "theta": theta, "omega": omega})
    print("goal:", json.dumps(g)[:200])
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30:
        time.sleep(0.25)
        s = ps.get("/state")
        goal = s.get("goal") or {}
        if goal.get("result"):
            print("result:", goal.get("result"), goal.get("reason", ""), "pose:", s.get("pose"))
            break
    else:
        print("timeout: stopping", ps.post("/stop", {}))
except BaseException as e:
    print("error, stopping:", e, ps.post("/stop", {}))
    raise
