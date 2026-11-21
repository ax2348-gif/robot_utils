#!/bin/bash
# Arm loop rate vs each factor: arm servoing (track), base hold, the recorder. From the Mac. Usage: run_arm_factors.sh OUT [SEC]
set -u
OUT=${1:?}; SEC=${2:-45}
REPO=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$OUT"
run() { ssh botcore "cd ~/profiling && ~/robot_utils/.venv/bin/python pi_arm_factor.py $1 $SEC $2 $3" | tee -a "$OUT/factors.jsonl"; sleep 3; }
rec_on() { python3 "$REPO/tools/agent_mm/record.py" "$OUT/rec_$1" > "$OUT/rec_$1.log" 2>&1 & echo $! > "$OUT/rec.pid"; sleep 8; }
rec_off() { kill -TERM "$(cat "$OUT/rec.pid")" 2>/dev/null; sleep 3; pkill -TERM -f agent_mm/record.py 2>/dev/null; sleep 1; }
run idle off off
run enabled enabled off
run track track off
run track_base track hold
rec_on a; run track_rec track off; rec_off
rec_on b; run track_base_rec track hold; rec_off
echo ALL DONE
