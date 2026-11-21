#!/bin/bash
# Arm/base breaking point vs page viewers (from the Mac). Fixed base load: all nodes, the recorder, the arm servoing
# in place, the base holding. Steps add simulated viewers one page at a time. Usage: run_breaking_point.sh OUT [SEC]
set -u
OUT=${1:?}; SEC=${2:-45}; mkdir -p "$OUT"
REPO=$(cd "$(dirname "$0")/../.." && pwd)
python3 "$REPO/tools/agent_mm/record.py" "$OUT/rec" > "$OUT/rec.log" 2>&1 & REC=$!
sleep 8
step() {   # label, pages (or "none")
  local label=$1 pages=$2 VW=""
  if [ "$pages" != none ]; then python3 "$REPO/tools/profiling/sim_viewers.py" $((SEC + 15)) "$pages" > "$OUT/viewers_$label.txt" 2>&1 & VW=$!; sleep 5; fi
  ssh botcore "cd ~/profiling && ~/robot_utils/.venv/bin/python pi_arm_factor.py $label $SEC track hold" | grep "^{" | tee -a "$OUT/steps.jsonl"
  [ -n "$VW" ] && { kill $VW 2>/dev/null; wait $VW 2>/dev/null; }
  sleep 3
}
step s0_none none
step s1_cam camera
step s2_cam2 camera,camera
step s3_cam2_wrist camera,camera,wrist
step s4_plus_perception camera,camera,wrist,perception
step s5_plus_armpage camera,camera,wrist,perception,arm
step s6_armpage_only arm
kill -TERM $REC 2>/dev/null; sleep 3; pkill -TERM -f agent_mm/record.py 2>/dev/null
echo ALL DONE
