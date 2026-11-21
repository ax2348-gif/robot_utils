#!/bin/bash
# Run the Damiao send-timing bench across pacing gap x CPU load x frames/tick (on the Pi). Takes the arm's motors off
# (e-stop + clear: damping) and stops the arm node first (the bench needs the adapter), restarts the node after.
set -u
cd ~/profiling
R=/run/user/$(id -u)/robot
PY=~/robot_utils/.venv/bin/python
OUT=dm_bench.jsonl; : > $OUT
$PY - <<'PYEOF'
import os, sys, time
sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient
a = NodeClient("arm", timeout=15)
st = a.get("/state"); print("arm before:", st["state"], "motors_on", st["motors_on"], flush=True)
if st["motors_on"]:
    tok = a.post("/lease", {"client": "dm bench: motors off"})["token"]; a.headers["X-Lease-Token"] = tok
    a.post("/estop", {"reason": "dm bench: motors off (damping) before stopping the node", "source": "profiling"})
    time.sleep(0.5); a.post("/estop/clear", {})
print("arm now:", a.get("/state")["state"], flush=True)
PYEOF
curl -s -m 30 -X POST http://localhost/api/nodes/arm/stop >/dev/null; sleep 3
echo "arm node: $(systemctl --user is-active robot-node-arm)"
stress_on() { for i in 1 2; do $PY -c "while True: pass" & echo $! >> stress.pids; done; }
stress_off() { [ -f stress.pids ] && kill $(cat stress.pids) 2>/dev/null; rm -f stress.pids; }
for gap in auto 0 500; do
  for load in idle stress; do
    for mode in both xout; do
      [ $load = stress ] && stress_on
      if [ $gap = auto ]; then env -u MOTORBRIDGE_TX_GAP_US $PY pi_dm_bench.py ${gap}_${load}_${mode} 20 $mode >> $OUT 2>bench_err.txt
      else MOTORBRIDGE_TX_GAP_US=$gap $PY pi_dm_bench.py ${gap}_${load}_${mode} 20 $mode >> $OUT 2>bench_err.txt; fi
      stress_off; tail -1 $OUT | cut -c1-160; sleep 2
    done
  done
done
curl -s -m 40 -X POST http://localhost/api/nodes/arm/start >/dev/null; sleep 8
echo "arm node restarted: $(curl -s -m 3 --unix-socket $R/arm.sock http://x/health)"
echo ALL DONE
