#!/bin/bash
# While profiling on the undersized UPS input: every 20 s read the power node; if the lowest cell drops below
# LIMIT, stop the camera nodes (the biggest load; no motion) and exit. Usage: power_guard.sh [LIMIT_V]
LIMIT=${1:-3.40}
while true; do
  v=$(ssh -o ConnectTimeout=5 -o BatchMode=yes botcore 'curl -s -m 3 --unix-socket /run/user/$(id -u)/robot/power.sock http://x/state' 2>/dev/null \
      | python3 -c "import json,sys; u=json.load(sys.stdin)['sources'][0]; print(min(u['cells_v']), u['power_w'])" 2>/dev/null)
  if [ -n "$v" ]; then
    set -- $v
    echo "$(date +%T) min cell $1 V, pack $2 W"
    if python3 -c "import sys; sys.exit(0 if float('$1') < $LIMIT else 1)"; then
      echo "$(date +%T) GUARD: min cell $1 V < $LIMIT V: stopping camera nodes"
      ssh botcore 'for n in camera wrist_camera; do curl -s -m 20 -X POST http://localhost/api/nodes/$n/stop >/dev/null; done'
      exit 0
    fi
  else
    echo "$(date +%T) power node unreachable"
  fi
  sleep 20
done
