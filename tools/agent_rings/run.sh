#!/usr/bin/env bash
# Launch a rings coding-agent run in tmux (tools/agent_rings/README.md).
#
#   tools/agent_rings/run.sh                 # a new run under ~/agent_runs/rings-<YYYYmmdd-HHMMSS>
#   tools/agent_rings/run.sh --no-agent      # everything but the agent (to check the harness)
#   tools/agent_rings/run.sh --inherit ~/agent_runs/rings-<old>/workspace
#                                            # a fresh agent given an earlier agent's code (its *.py / *.sh, in
#                                            # workspace/inherited/, plus its venv's package list), nothing else:
#                                            # no notes, logs, images, measured data or transcript
#
# tmux session rings-<ts>, windows: stack (the agent's sim stack, socket-only), proxy (the agent's only door,
# unix socket $SOCK_DIR/robot.sock), score (privileged scorer), record (state + rigid scene for replay), agent (interactive Claude Code
# with Remote Control, sandboxed to the workspace). Stop with tools/agent_rings/stop.sh rings-<ts>.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
PY="${ROBOT_PY:-$HOME/miniconda3/envs/robot_utils_test/bin/python}"      # runs the stack (not the agent's)
BASE_PY="${AGENT_BASE_PY:-$HOME/miniconda3/bin/python3}"                 # the agent's venv is made from this
CLAUDE="${CLAUDE_BIN:-$(ls -d "$HOME/Library/Application Support/Claude/claude-code/"*/*/claude.app/Contents/MacOS/claude 2>/dev/null | sort -V | tail -1)}"
RT="${AGENT_RT:-/tmp/ar}"                # the agent stack's runtime dir (short: unix socket paths); agent can't read it
SOCK_DIR="${AGENT_SOCK_DIR:-/tmp/ara}"      # the proxy's socket: the one thing the agent's sandbox may connect to
SOCK="$SOCK_DIR/robot.sock"
WBC_SOCK="$SOCK_DIR/wbc.target.sock"       # dgram_relay.py: the WBC's target datagrams
SIM_CONFIG="${AGENT_SIM_CONFIG:-$REPO/deploy/sim/sim_rings_dynamic.yaml}"
NODES=description,sim,arm,swerve,camera,wrist_camera,power,estimator,pose_servo,wbc,3d_perception,agent_interface
AGENT=1
INHERIT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-agent) AGENT=0 ;;
    --inherit) INHERIT="$(cd "${2:?--inherit needs a workspace}" && pwd)"; shift ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done

TS="$(date +%Y%m%d-%H%M%S)"
NAME="rings-$TS"
RUN="$HOME/agent_runs/$NAME"
WS="$RUN/workspace"
H="$HOME/agent_harness/$NAME"            # out of every agent's reach (denied as a whole)

[[ -x "$CLAUDE" || $AGENT == 0 ]] || { echo "no claude CLI found (set CLAUDE_BIN)"; exit 1; }
if [[ -S "$RT/robot/frontend.sock" ]] && curl -s --max-time 2 --unix-socket "$RT/robot/frontend.sock" http://x/api/health >/dev/null; then
  echo "a stack already answers in $RT (another run?); stop it first"; exit 1
fi
if [[ -S "$SOCK" ]] && curl -s --max-time 2 --unix-socket "$SOCK" http://robot/api/health >/dev/null; then
  echo "a proxy already answers on $SOCK (another run?)"; exit 1
fi
mkdir -p "$SOCK_DIR"

mkdir -p "$WS/docs" "$H"
cp "$HERE/workspace/CLAUDE.md" "$WS/CLAUDE.md"
for d in agent_interface arm swerve_node pose_servo_node estimator_node orbbec_node perception3d_node \
         description_node nodes locks wbc; do
  cp "$REPO/docs/$d.md" "$WS/docs/"
done
printf '# Notes\n\n' > "$WS/NOTES.md"
if [[ -n "$INHERIT" ]]; then      # code only: the earlier agent's notes, logs, images and data stay behind
  mkdir -p "$WS/inherited"
  (cd "$INHERIT" && find . \( -name .venv -o -name __pycache__ -o -name docs -o -name inherited -o -name .claude \) -prune \
     -o -type f \( -name '*.py' -o -name '*.sh' \) -print) | while read -r f; do
    mkdir -p "$WS/inherited/$(dirname "$f")"; cp "$INHERIT/$f" "$WS/inherited/$f"
  done
  [[ -x "$INHERIT/.venv/bin/pip" ]] && "$INHERIT/.venv/bin/pip" freeze 2>/dev/null > "$WS/inherited/requirements.txt"
  cat "$HERE/workspace/INHERITED.md" >> "$WS/CLAUDE.md"
fi
"$BASE_PY" -m venv "$WS/.venv"
# pip's own CA bundle (the sandbox blocks the macOS keychain lookup) and a cache in the workspace
printf '[global]\nuse-deprecated = legacy-certs\ncache-dir = %s\n' "$WS/.cache/pip" > "$WS/.venv/pip.conf"
"$WS/.venv/bin/pip" install -q --upgrade pip >/dev/null 2>&1 || true

# --- the agent's settings (settings.py): dontAsk, the OS sandbox around Bash, deny rules; a harness-side file -----
DENY_PATHS=("$HOME/Documents" "$HOME/data" "$HOME/.local/share/robot_utils" "$HOME/.claude" "$HOME/.claude.json"
            "$HOME/agent_harness"
            "$RT" "/private$RT" "$HOME/robot_utils_exports" "$HOME/quest-backups" "$HOME/.ssh" "$HOME/.aws"
            "$HOME/Library/Application Support/Claude" "$HOME/miniconda3/envs"
            "$("$BASE_PY" -c 'import tempfile;print(tempfile.gettempdir())')/robot-$(id -u)")
for d in /tmp/claude-$(id -u)/*/; do DENY_PATHS+=("${d%/}" "/private${d%/}"); done    # other sessions' scratch, stacks
for r in "$HOME"/agent_runs/*/; do [[ "${r%/}" != "$RUN" ]] && DENY_PATHS+=("${r%/}"); done   # earlier runs
DENY_ARGS=(); for d in "${DENY_PATHS[@]}"; do DENY_ARGS+=(--deny "$d"); done
"$BASE_PY" "$HERE/settings.py" "$H/agent-settings.json" --workspace "$WS" --robot-socket "$SOCK" --robot-socket "/private$SOCK" \
  --robot-socket "$WBC_SOCK" --robot-socket "/private$WBC_SOCK" "${DENY_ARGS[@]}"
SID="$(uuidgen | tr A-Z a-z)"
cat > "$H/run.json" <<EOF
{"name": "$NAME", "workspace": "$WS", "session_id": "$SID", "socket": "$SOCK", "runtime_dir": "$RT",
 "sim_config": "$SIM_CONFIG", "repo": "$REPO", "commit": "$(git -C "$REPO" rev-parse HEAD)", "inherited_from": "$INHERIT",
 "transcript": "$HOME/.claude/projects/$(echo "$WS" | sed 's#[^A-Za-z0-9]#-#g')/$SID.jsonl", "started": "$TS"}
EOF

# --- tmux ---------------------------------------------------------------------------------------------------------
ENVS="cd '$REPO' && export PYTHONPATH='$REPO/src' ROBOT_SIM_CONFIG='$SIM_CONFIG' XDG_RUNTIME_DIR='$RT'"
tmux new-session -d -s "$NAME" -n stack -c "$REPO"
tmux send-keys -t "$NAME:stack" "$ENVS && '$PY' -m robot_utils.nodes.sim.stack --port 0 --runtime-dir '$RT' \
--config '$HERE/stack_nodes.yaml' --nodes $NODES --no-pull 2>&1 | tee -i -a '$H/stack.log'" Enter

echo -n "waiting for the stack"
for i in $(seq 1 120); do
  if curl -s --max-time 2 --unix-socket "$RT/robot/sim.sock" http://x/state >/dev/null 2>&1 \
     && curl -s --max-time 2 --unix-socket "$RT/robot/agent_interface.sock" http://x/ >/dev/null 2>&1; then break; fi
  echo -n .; sleep 2
done; echo

tmux new-window -t "$NAME" -n proxy -c "$REPO"
tmux send-keys -t "$NAME:proxy" "$ENVS && '$PY' '$HERE/proxy.py' --upstream 'unix:$RT/robot/frontend.sock' \
--uds '$SOCK' --wbc-socket '$WBC_SOCK' --log '$H/requests.jsonl'" Enter
tmux new-window -t "$NAME" -n relay -c "$REPO"
tmux send-keys -t "$NAME:relay" "$ENVS && '$PY' '$HERE/dgram_relay.py' --listen '$WBC_SOCK' --to 'unix:$RT/robot/wbc.target.sock' \
--log '$H/wbc_datagrams.jsonl'" Enter
tmux new-window -t "$NAME" -n score -c "$REPO"
tmux send-keys -t "$NAME:score" "$ENVS && '$PY' '$HERE/score.py' --sim 'unix:$RT/robot/sim.sock' --every 10 \
--out '$H/scores.jsonl'" Enter
tmux new-window -t "$NAME" -n record -c "$REPO"
tmux send-keys -t "$NAME:record" "$ENVS && '$PY' -m robot_utils.collect run --robot local --out '$H/recording' \
--task 'rings (coding agent $NAME)' --no-set-task --streams arm/stream,swerve/stream,estimator/stream,wbc/stream --scene3d sim \
--rotate 500M" Enter

if [[ $AGENT == 1 ]]; then
  tmux new-window -t "$NAME" -n agent -c "$WS"
  # a clean environment: no PYTHONPATH / runtime dir / repo paths leak into the agent's shell
  tmux send-keys -t "$NAME:agent" "cd '$WS' && env -u PYTHONPATH -u XDG_RUNTIME_DIR -u ROBOT_SIM_CONFIG \
-u CONDA_PREFIX -u CONDA_DEFAULT_ENV PATH='$WS/.venv/bin:/usr/bin:/bin:/usr/sbin:/sbin:/opt/homebrew/bin' \
ROBOT_SOCKET='$SOCK' WBC_TARGET_SOCKET='$WBC_SOCK' '$CLAUDE' --permission-mode dontAsk --setting-sources '' \
--settings '$H/agent-settings.json' --session-id $SID -n $NAME --remote-control $NAME \
'Read CLAUDE.md and carry out the task it describes. Work autonomously for as long as it takes; nobody will answer questions.'" Enter
fi

echo "run:        $RUN"
echo "tmux:       tmux attach -t $NAME"
echo "robot:      curl --unix-socket $SOCK http://robot/"
echo "transcript: $(sed -n 's/.*"transcript": "\([^"]*\)".*/\1/p' "$H/run.json")"
