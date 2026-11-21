#!/bin/bash
# "System 0 on erid": control-stream latency and a 100 Hz closed command loop from erid over the robot Wi-Fi, while
# the colour-only vision load (C2: head 1280x800@30, wrist 640x480@30, head IMU) streams to erid too and the base
# holds its pose. From the Mac. Battery-gated; afterwards the pin is re-applied and every node but power stopped.
#   erid: full_rate_consumer (vision load, 70 s) + erid_loop_probe (15 s passive, then 30 s active stationary track)
#   Pi:   pi_arm_factor ARM=off BASE=hold (60 s): arm / swerve rates, pose_servo, CPU (the arm is erid's)
# Usage: run_erid_loop.sh OUT [GATE_V]
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
ssh erid "$ER HEAD_STREAMS=color WRIST_STREAMS=color ~/miniconda3/bin/python tools/profiling/full_rate_consumer.py 70 raw /tmp/consumer_loop.json" \
  > "$OUT/consumer.log" 2>&1 & C=$!
ssh erid "cd ~/robot_core_test && ~/miniconda3/bin/python tools/profiling/erid_loop_probe.py 15 30 /tmp/loop_probe.json" \
  > "$OUT/probe.log" 2>&1 & P=$!
sleep 3
ssh botcore "cd ~/profiling; ~/robot_utils/.venv/bin/python pi_arm_factor.py erid_loop 60 off hold" > "$OUT/pi.txt" 2>&1
wait $P; wait $C
scp -q erid:/tmp/consumer_loop.json "$OUT/consumer.json"; scp -q erid:/tmp/loop_probe.json "$OUT/probe.json"
echo "$(date +%T) after: $(pw)" | tee -a "$OUT/power.txt"
echo "== restore"; ssh botcore 'cd ~/profiling && ~/robot_utils/.venv/bin/python pi_camera_pin.py apply | tail -2'
ssh botcore 'for n in camera wrist_camera arm pose_servo estimator swerve description; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/stop -o /dev/null -w "$n %{http_code}  "; done; echo'
echo "after all: $(pw)"
echo ALL DONE
