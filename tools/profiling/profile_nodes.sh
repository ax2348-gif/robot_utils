#!/bin/bash
# On the Pi: GET /debug/profile from each node in turn. Usage: profile_nodes.sh OUTDIR SECONDS HZ
OUT=${1:?}; SEC=${2:-30}; HZ=${3:-100}
R=/run/user/$(id -u)/robot
mkdir -p "$OUT"
for n in agent_interface camera wrist_camera swerve estimator pose_servo 3d_perception description; do
  curl -s -m $((SEC + 30)) --unix-socket "$R/$n.sock" "http://x/debug/profile?seconds=$SEC&hz=$HZ&folded=true" > "$OUT/$n.json"
  echo "done $n $(wc -c < "$OUT/$n.json")"
done
