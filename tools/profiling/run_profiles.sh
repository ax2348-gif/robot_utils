#!/bin/bash
# From the Mac: per-node stack profiles idle, then under the recorder + two MJPEG live views. Results on the Pi
# in ~/profiling/<tag>_idle and <tag>_load, copied to $OUT.
set -u
OUT=${1:?}; TAG=${2:-p}; SEC=${3:-30}
REPO=$(cd "$(dirname "$0")/../.." && pwd)
HOST=${ROBOT_NODES:-http://botcore-desktop.local/nodes}
mkdir -p "$OUT"
ssh botcore "bash ~/profiling/profile_nodes.sh ~/profiling/${TAG}_idle $SEC 100"
python3 "$REPO/tools/agent_mm/record.py" "$OUT/rec" > "$OUT/rec.log" 2>&1 & REC=$!
curl -s -o /dev/null "$HOST/camera/mjpeg/color" & M1=$!
curl -s -o /dev/null "$HOST/wrist_camera/mjpeg/color" & M2=$!
sleep 8
ssh botcore "bash ~/profiling/profile_nodes.sh ~/profiling/${TAG}_load $SEC 100"
kill -TERM $REC $M1 $M2 2>/dev/null; sleep 3
pgrep -f "agent_mm/record.py" >/dev/null && { echo "recorder still running; stopping"; pkill -TERM -f "agent_mm/record.py"; }
scp -q -r "botcore:profiling/${TAG}_idle" "botcore:profiling/${TAG}_load" "$OUT/"
echo ALL DONE
