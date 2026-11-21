#!/bin/bash
# Whole-system robustness run (from the Mac): simulated remote viewers + recorder + arm servoing in place + base
# small-turn goals + a system sample, all at once. Usage: run_system_test.sh OUT
set -u
OUT=${1:?}; mkdir -p "$OUT"
REPO=$(cd "$(dirname "$0")/../.." && pwd)
python3 "$REPO/tools/profiling/sim_viewers.py" 330 2 > "$OUT/viewers.txt" 2>&1 & VW=$!
python3 "$REPO/tools/agent_mm/record.py" "$OUT/rec" > "$OUT/rec.log" 2>&1 & REC=$!
sleep 10
ssh botcore 'cd ~/profiling; ~/robot_utils/.venv/bin/python pi_arm_factor.py system_arm 240 track off > sys_arm.txt 2>&1 &
  sleep 15; ~/robot_utils/.venv/bin/python pi_base_stale.py 16 0.2 > sys_base.txt 2>&1 &
  sleep 5; ~/robot_utils/.venv/bin/python pi_load_sample.py 180 system > sys_load.json 2>&1; wait; sleep 1;
  cat sys_arm.txt sys_base.txt' > "$OUT/pi.txt" 2>&1
scp -q botcore:profiling/sys_load.json "$OUT/"
kill -TERM $REC 2>/dev/null; sleep 3; pkill -TERM -f agent_mm/record.py 2>/dev/null
wait $VW
echo ALL DONE
