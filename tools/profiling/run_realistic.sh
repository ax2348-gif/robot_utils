#!/bin/bash
# Realistic experiment, no viewers (from the Mac): waits until the UPS pack has recovered (lowest cell >= GATE_V while
# charging), brings all nodes up, then runs the recorder + the arm servoing in place + base small-turn goals.
# Usage: run_realistic.sh OUT [GATE_V]
set -u
OUT=${1:?}; GATE=${2:-3.60}; mkdir -p "$OUT"
REPO=$(cd "$(dirname "$0")/../.." && pwd)
pw() { ssh -o ConnectTimeout=10 botcore 'curl -s --unix-socket /run/user/$(id -u)/robot/power.sock http://x/state' |
       python3 -c "import json,sys;d=json.load(sys.stdin);u=d['sources'][0];print(min(u['cells_v']),u['power_w'],d['state'])"; }
while true; do
  read mn w st <<<"$(pw)"; echo "$(date +%T) lowest cell $mn V, pack $w W ($st)"
  python3 -c "import sys; sys.exit(0 if float('$mn') >= $GATE else 1)" && break
  sleep 60
done
ssh botcore 'for n in camera wrist_camera 3d_perception description agent_interface; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/start >/dev/null; sleep 2; done; sleep 8;
  R=/run/user/$(id -u)/robot; for n in camera wrist_camera swerve arm estimator pose_servo description 3d_perception agent_interface; do
  printf "%s=%s " $n "$(curl -s -m 3 --unix-socket $R/$n.sock http://x/health | grep -o "\"ok\":true" | head -1)"; done; echo' | tee "$OUT/nodes.txt"
python3 "$REPO/tools/agent_mm/record.py" "$OUT/rec" > "$OUT/rec.log" 2>&1 & REC=$!
sleep 10
ssh botcore 'cd ~/profiling; ~/robot_utils/.venv/bin/python pi_arm_factor.py realistic_arm 240 track off > real_arm.txt 2>&1 &
  sleep 15; ~/robot_utils/.venv/bin/python pi_base_stale.py 16 0.2 > real_base.txt 2>&1; wait; cat real_arm.txt real_base.txt' > "$OUT/pi.txt" 2>&1
kill -TERM $REC 2>/dev/null; sleep 3; pkill -TERM -f agent_mm/record.py 2>/dev/null
echo "after: $(pw)"
echo ALL DONE
