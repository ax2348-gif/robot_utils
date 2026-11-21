#!/bin/bash
# Core test (from the Mac): waits until the UPS pack has recovered (lowest cell >= GATE_V), brings the core nodes up
# (not 3d_perception / agent_interface: client side), re-applies the camera pin, then per depth mode (zlib, raw) runs
# one full-rate LAN consumer of every stream here while the Pi runs the arm servoing in place + the base holding.
# Usage: run_core.sh OUT [GATE_V]
set -u
OUT=${1:?}; GATE=${2:-3.60}; mkdir -p "$OUT"
HERE=$(cd "$(dirname "$0")" && pwd)
pw() { ssh -o ConnectTimeout=10 botcore 'curl -s --unix-socket /run/user/$(id -u)/robot/power.sock http://x/state' |
       python3 -c "import json,sys;d=json.load(sys.stdin);u=d['sources'][0];print(min(u['cells_v']),u['power_w'],d['state'])"; }
while true; do
  read mn w st <<<"$(pw)"; echo "$(date +%T) lowest cell $mn V, pack $w W ($st)"
  python3 -c "import sys; sys.exit(0 if float('$mn') >= $GATE else 1)" && break
  sleep 60
done
ssh botcore 'for n in swerve estimator pose_servo arm camera wrist_camera description; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/start >/dev/null; sleep 2; done; sleep 8;
  cd ~/profiling && ~/robot_utils/.venv/bin/python pi_camera_pin.py apply | tail -2
  R=/run/user/$(id -u)/robot; for n in camera wrist_camera swerve arm estimator pose_servo description; do
  printf "%s=%s " $n "$(curl -s -m 3 --unix-socket $R/$n.sock http://x/health | grep -o "\"ok\":true" | head -1)"; done; echo' | tee "$OUT/nodes.txt"
for mode in zlib raw; do
  echo "$(date +%T) core_$mode: $(pw)" | tee -a "$OUT/power.txt"
  python3 "$HERE/full_rate_consumer.py" 75 $mode "$OUT/consumer_$mode.json" > "$OUT/consumer_$mode.log" 2>&1 & C=$!
  sleep 8
  ssh botcore "cd ~/profiling; ~/robot_utils/.venv/bin/python pi_arm_factor.py core_$mode 60 track hold" > "$OUT/pi_$mode.txt" 2>&1
  wait $C
  echo "$(date +%T) after core_$mode: $(pw)" | tee -a "$OUT/power.txt"
  sleep 20
done
echo ALL DONE
