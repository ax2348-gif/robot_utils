#!/usr/bin/env bash
# Quest teleop: arm (rebot_ctl) and swerve base (rev_ctl) as two processes.
#
#     ./quest_teleop.sh
#
# Ctrl-C sends one SIGINT to each, so both run their cleanup (arm disconnect,
# swerve close). If either exits on its own, the other is stopped the same way.
# Each process keeps its own Python interpreter, so the arm loop cannot delay
# the swerve heartbeat.

cd "$(dirname "$0")" || exit 1
PY=.venv/bin/python
CAN=${CAN:-slcan:/dev/ttyACM0}

# Job control gives each child its own process group with normal SIGINT
# handling. Without it bash starts background jobs with SIGINT ignored, and
# Python then never raises KeyboardInterrupt.
set -m

"$PY" rebot_ctl/tools/quest_teleop.py & arm=$!
"$PY" rev_ctl/examples/quest_teleop.py "$CAN" & base=$!

stopping=
stop() {
    # Once only: a second SIGINT would interrupt the children's cleanup.
    [ -n "$stopping" ] && return
    stopping=1
    echo; echo "Stopping arm and base..."
    kill -INT "$arm" "$base" 2>/dev/null
}
trap stop INT TERM

declare -A status
wait -n -p done "$arm" "$base"   # returns when either exits, or on Ctrl-C
s=$?
[ -n "$done" ] && status[$done]=$s
stop
for pid in "$arm" "$base"; do
    [ -n "${status[$pid]+set}" ] && continue
    while :; do
        wait "$pid"; s=$?   # returns early if a signal arrives; keep waiting
        kill -0 "$pid" 2>/dev/null || break
    done
    status[$pid]=$s
done
echo "arm exited ${status[$arm]}, base exited ${status[$base]}"
[ "${status[$arm]}" -eq 0 ] && exit "${status[$base]}" || exit "${status[$arm]}"
