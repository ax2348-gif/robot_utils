#!/usr/bin/env bash
# Stop a rings run (tools/agent_rings/README.md): the agent first (Ctrl-C x2), then the recorder (lets it settle),
# scorer, proxy and the stack, then the tmux session. The run directory stays.
#   tools/agent_rings/stop.sh rings-<ts>
set -uo pipefail
S="${1:?usage: stop.sh rings-<ts>}"
tmux has-session -t "$S" 2>/dev/null || { echo "no tmux session $S"; exit 1; }
w(){ tmux list-windows -t "$S" -F '#W' | grep -qx "$1"; }
if w agent; then tmux send-keys -t "$S:agent" C-c; sleep 1; tmux send-keys -t "$S:agent" C-c; sleep 2; fi
for x in record score proxy; do w $x && tmux send-keys -t "$S:$x" C-c; done
sleep 5
w stack && tmux send-keys -t "$S:stack" C-c
for i in $(seq 1 30); do tmux capture-pane -p -t "$S:stack" 2>/dev/null | grep -q "stack\] stopped" && break; sleep 1; done
tmux kill-session -t "$S"
echo "stopped $S; run directory ~/agent_runs/$S"
