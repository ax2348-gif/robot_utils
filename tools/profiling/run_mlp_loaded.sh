#!/bin/bash
# A neural "system 0" on the Pi beside the colour-only core load (from the Mac). Battery-gated. Brings the core
# nodes up with the camera pin, colour only (head 1280x800@30, wrist 640x480@30); erid consumes every stream at full
# rate; the Pi servos the arm in place + holds the base (70 s window) while pi_mlp_bench paces the 0.2M, 1M, 2M and
# 13M MLPs at 100 Hz (14 s each, numpy, one thread). Compare the arm's rate/overruns with the C2 baseline
# (100 Hz, 0 overruns, 33% CPU). Afterwards the pin is re-applied and every node but power stopped.
# Usage: run_mlp_loaded.sh OUT [GATE_V]
set -u
OUT=${1:?}; GATE=${2:-3.60}; mkdir -p "$OUT"
ER="cd ~/robot_core_test && ROBOT_NODES=http://192.168.8.209/nodes"
pw() { ssh -o ConnectTimeout=10 botcore 'curl -s --unix-socket /run/user/$(id -u)/robot/power.sock http://x/state' |
       python3 -c "import json,sys;d=json.load(sys.stdin);u=d['sources'][0];print(min(u['cells_v']),u['power_w'],d['state'])"; }
cam() {
  ssh botcore "curl -s -m 60 --unix-socket /run/user/\$(id -u)/robot/$1.sock -X POST http://x/settings -H 'Content-Type: application/json' -d '$2'" |
    python3 -c "import json,sys;d=json.load(sys.stdin);s=d.get('settings') or {};print('  $1:', d.get('detail') or (s.get('color_profile'), [k for k,v in (s.get('streams') or {}).items() if v]))"
}
while true; do
  read mn w st <<<"$(pw)"; echo "$(date +%T) lowest cell $mn V, pack $w W ($st)"
  python3 -c "import sys; sys.exit(0 if float('$mn') >= $GATE else 1)" && break
  sleep 60
done
ssh botcore 'for n in swerve estimator pose_servo arm camera wrist_camera description; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/start >/dev/null; sleep 2; done; sleep 8;
  cd ~/profiling && ~/robot_utils/.venv/bin/python pi_camera_pin.py apply | tail -2' | tee "$OUT/nodes.txt"
cam camera '{"streams":{"depth":false}}'; cam wrist_camera '{"streams":{"depth":false}}'
cam camera '{"color_profile":"1280x800@30 MJPG"}'; sleep 3
echo "$(date +%T) start: $(pw)" | tee -a "$OUT/power.txt"
ssh erid "$ER HEAD_STREAMS=color WRIST_STREAMS=color ~/miniconda3/bin/python tools/profiling/full_rate_consumer.py 85 raw /tmp/consumer_mlp.json" \
  > "$OUT/consumer.log" 2>&1 & C=$!
sleep 6
ssh botcore "cd ~/profiling; ~/robot_utils/.venv/bin/python pi_arm_factor.py mlp_loaded 70 track hold" > "$OUT/pi.txt" 2>&1 & A=$!
sleep 4
ssh botcore "cd ~/profiling; ~/robot_utils/.venv/bin/python pi_mlp_bench.py paced 14" > "$OUT/mlp.txt" 2>&1
wait $A; wait $C
scp -q erid:/tmp/consumer_mlp.json "$OUT/consumer.json"
echo "$(date +%T) after: $(pw)" | tee -a "$OUT/power.txt"
echo "== restore"; ssh botcore 'cd ~/profiling && ~/robot_utils/.venv/bin/python pi_camera_pin.py apply | tail -2'
ssh botcore 'for n in camera wrist_camera arm pose_servo estimator swerve description; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/stop -o /dev/null -w "$n %{http_code}  "; done; echo'
echo "after all: $(pw)"
echo ALL DONE
