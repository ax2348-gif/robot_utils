#!/bin/bash
# Run the Pi load profile under each condition (from the Mac). Results: $OUT/<label>.json
# Conditions: baseline; recorder (tools/agent_mm/record.py, both cameras); recorder + two MJPEG live views;
# control loops enabled (no motion); everything together.
set -u
OUT=${1:?out dir}
SEC=${2:-60}
REPO=$(cd "$(dirname "$0")/../.." && pwd)
HOST=${ROBOT_NODES:-http://botcore-desktop.local/nodes}
mkdir -p "$OUT"
sample() { ssh botcore "cd ~/profiling && ~/robot_utils/.venv/bin/python pi_load_sample.py $SEC $1" > "$OUT/$1.json"; echo "done $1"; }
rec_start() { python3 "$REPO/tools/agent_mm/record.py" "$OUT/rec_$1" > "$OUT/rec_$1.log" 2>&1 & echo $! > "$OUT/rec.pid"; sleep 6; }
rec_stop() { kill -TERM "$(cat "$OUT/rec.pid")" 2>/dev/null; sleep 3; if pgrep -f "agent_mm/record.py" >/dev/null; then echo "recorder still running" >&2; pkill -TERM -f "agent_mm/record.py"; sleep 2; fi; }
mjpeg_start() { curl -s -o /dev/null "$HOST/camera/mjpeg/color" & echo $! > "$OUT/mj1.pid"; curl -s -o /dev/null "$HOST/wrist_camera/mjpeg/color" & echo $! > "$OUT/mj2.pid"; sleep 4; }
mjpeg_stop() { kill "$(cat "$OUT/mj1.pid")" "$(cat "$OUT/mj2.pid")" 2>/dev/null; sleep 2; }
hold_start() { ssh botcore "cd ~/profiling && ~/robot_utils/.venv/bin/python pi_control_hold.py $((SEC + 30))" > "$OUT/hold_$1.log" 2>&1 & echo $! > "$OUT/hold.pid"; sleep 8; }
hold_wait() { wait "$(cat "$OUT/hold.pid")" 2>/dev/null; sleep 4; }

sample baseline
rec_start recorder; sample recorder; rec_stop
mjpeg_start; sample mjpeg; mjpeg_stop
rec_start recmjpeg; mjpeg_start; sample recorder_mjpeg; mjpeg_stop; rec_stop
hold_start control; sample control_enabled; hold_wait
hold_start all; rec_start all; mjpeg_start; sample all; mjpeg_stop; rec_stop; hold_wait
sample baseline_after
echo ALL DONE
