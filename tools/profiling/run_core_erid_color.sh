#!/bin/bash
# Colour-only core test (what most policies use), erid as the consumer (from the Mac). Waits for the UPS (lowest cell
# >= GATE_V), brings the core nodes up, applies the camera pin, turns depth off on both cameras, then per variant
# runs erid's consumer (75 s) while the Pi servos the arm in place and holds the base (60 s):
#   C1   head colour 1280x720@30 MJPG, wrist colour 640x480@30 MJPG, head IMU
#   C2   head colour 1280x800@30 MJPG (the head's largest), wrist as C1
# Afterwards: the pin re-applied (depth back on), every node but power stopped (the arm is at home).
# Usage: run_core_erid_color.sh OUT [GATE_V]
set -u
OUT=${1:?}; GATE=${2:-3.60}; mkdir -p "$OUT"
ER="cd ~/robot_core_test && ROBOT_NODES=http://192.168.8.209/nodes"
pw() { ssh -o ConnectTimeout=10 botcore 'curl -s --unix-socket /run/user/$(id -u)/robot/power.sock http://x/state' |
       python3 -c "import json,sys;d=json.load(sys.stdin);u=d['sources'][0];print(min(u['cells_v']),u['power_w'],d['state'])"; }
cam() {   # NODE PATCH
  ssh botcore "curl -s -m 60 --unix-socket /run/user/\$(id -u)/robot/$1.sock -X POST http://x/settings -H 'Content-Type: application/json' -d '$2'" |
    python3 -c "import json,sys;d=json.load(sys.stdin);s=d.get('settings') or {};print('  $1:', d.get('detail') or (s.get('color_profile'), [k for k,v in (s.get('streams') or {}).items() if v]))"
}
while true; do
  read mn w st <<<"$(pw)"; echo "$(date +%T) lowest cell $mn V, pack $w W ($st)"
  python3 -c "import sys; sys.exit(0 if float('$mn') >= $GATE else 1)" && break
  sleep 60
done
ssh botcore 'for n in swerve estimator pose_servo arm camera wrist_camera description; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/start >/dev/null; sleep 2; done; sleep 8;
  cd ~/profiling && ~/robot_utils/.venv/bin/python pi_camera_pin.py apply | tail -2
  R=/run/user/$(id -u)/robot; for n in camera wrist_camera swerve arm estimator pose_servo description; do
  printf "%s=%s " $n "$(curl -s -m 3 --unix-socket $R/$n.sock http://x/health | grep -o "\"ok\":true" | head -1)"; done; echo' | tee "$OUT/nodes.txt"
cam camera '{"streams":{"depth":false}}'; cam wrist_camera '{"streams":{"depth":false}}'

run() {   # NAME
  read mn w st <<<"$(pw)"
  if ! python3 -c "import sys; sys.exit(0 if float('$mn') >= 3.50 else 1)"; then
    echo "$(date +%T) $1 skipped: lowest cell $mn V < 3.50" | tee -a "$OUT/power.txt"; return; fi
  echo "$(date +%T) $1: $mn $w $st" | tee -a "$OUT/power.txt"
  ssh erid "$ER HEAD_STREAMS=color WRIST_STREAMS=color ~/miniconda3/bin/python tools/profiling/full_rate_consumer.py 75 raw /tmp/consumer_$1.json" \
    > "$OUT/consumer_$1.log" 2>&1 & C=$!
  sleep 8
  ssh botcore "cd ~/profiling; ~/robot_utils/.venv/bin/python pi_arm_factor.py erid_$1 60 track hold" > "$OUT/pi_$1.txt" 2>&1
  wait $C
  scp -q "erid:/tmp/consumer_$1.json" "$OUT/consumer_$1.json"
  echo "$(date +%T) after $1: $(pw)" | tee -a "$OUT/power.txt"
  sleep 15
}

echo "== C1: head 1280x720"; cam camera '{"color_profile":"1280x720@30 MJPG"}'; sleep 3
run C1
echo "== C2: head 1280x800"; cam camera '{"color_profile":"1280x800@30 MJPG"}'; sleep 3
run C2
echo "== restore"; ssh botcore 'cd ~/profiling && ~/robot_utils/.venv/bin/python pi_camera_pin.py apply | tail -2'
ssh botcore 'for n in camera wrist_camera arm pose_servo estimator swerve description; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/stop -o /dev/null -w "$n %{http_code}  "; done; echo'
echo "after all: $(pw)"
echo ALL DONE
