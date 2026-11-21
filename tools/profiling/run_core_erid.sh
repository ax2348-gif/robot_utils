#!/bin/bash
# Core test with erid as the one full-rate consumer (from the Mac). Waits for the UPS (lowest cell >= GATE_V), brings
# the core nodes up, applies the camera pin, then per variant sets the wrist and runs erid's consumer (75 s, over the
# robot LAN) while the Pi runs the arm servoing in place + the base holding (60 s window):
#   A        pinned head (zlib depth), wrist colour only (no wrist depth)
#   B_raw    pinned head, wrist dual colour 640x480@30 (left MJPG + right YUYV, raw)
#   B_zlib   the same, the wrist's raw frames zlib-compressed on the Pi
# Afterwards: the wrist back in depth mode, every node but power stopped (the arm is at home).
# Usage: run_core_erid.sh OUT [GATE_V]
set -u
OUT=${1:?}; GATE=${2:-3.60}; mkdir -p "$OUT"
ER="cd ~/robot_core_test && ROBOT_NODES=http://192.168.8.209/nodes"
pw() { ssh -o ConnectTimeout=10 botcore 'curl -s --unix-socket /run/user/$(id -u)/robot/power.sock http://x/state' |
       python3 -c "import json,sys;d=json.load(sys.stdin);u=d['sources'][0];print(min(u['cells_v']),u['power_w'],d['state'])"; }
wrist() {   # POST a settings patch to the wrist camera, print the result
  ssh botcore "curl -s -m 60 --unix-socket /run/user/\$(id -u)/robot/wrist_camera.sock -X POST http://x/settings -H 'Content-Type: application/json' -d '$1'" |
    python3 -c "import json,sys;d=json.load(sys.stdin);s=d.get('settings') or {};print('  wrist:', d.get('detail') or (s.get('camera_mode'), s.get('color_profile'), [k for k,v in (s.get('streams') or {}).items() if v]))"
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

run() {   # NAME WRIST_STREAMS WRIST_COMPRESS
  read mn w st <<<"$(pw)"
  if ! python3 -c "import sys; sys.exit(0 if float('$mn') >= 3.50 else 1)"; then
    echo "$(date +%T) $1 skipped: lowest cell $mn V < 3.50" | tee -a "$OUT/power.txt"; return; fi
  echo "$(date +%T) $1: $mn $w $st" | tee -a "$OUT/power.txt"
  ssh erid "$ER WRIST_STREAMS=$2 WRIST_COMPRESS=$3 ~/miniconda3/bin/python tools/profiling/full_rate_consumer.py 75 zlib /tmp/consumer_$1.json" \
    > "$OUT/consumer_$1.log" 2>&1 & C=$!
  sleep 8
  ssh botcore "cd ~/profiling; ~/robot_utils/.venv/bin/python pi_arm_factor.py erid_$1 60 track hold" > "$OUT/pi_$1.txt" 2>&1
  wait $C
  scp -q "erid:/tmp/consumer_$1.json" "$OUT/consumer_$1.json"
  echo "$(date +%T) after $1: $(pw)" | tee -a "$OUT/power.txt"
  sleep 15
}

echo "== A: wrist colour only"; wrist '{"streams":{"depth":false}}'; sleep 3
run A color raw
echo "== B: wrist dual colour"; wrist '{"camera_mode":"dual_color"}'; wrist '{"color_profile":"640x480@30 MJPG"}'; sleep 3
run B_raw color,color_right raw
run B_zlib color,color_right zlib
echo "== restore"; wrist '{"camera_mode":"depth"}'
ssh botcore 'for n in camera wrist_camera arm pose_servo estimator swerve description; do curl -s -m 40 -X POST http://localhost/api/nodes/$n/stop -o /dev/null -w "$n %{http_code}  "; done; echo'
echo "after all: $(pw)"
echo ALL DONE
