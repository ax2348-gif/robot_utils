#!/usr/bin/env bash
# Probe what a sandboxed agent can reach, from inside its Bash sandbox (tools/agent_rings/README.md). Run by the agent
# CLI with the run's settings; every line should read ALLOWED / BLOCKED as the comments say.
#   in a run's workspace: claude -p --permission-mode dontAsk --setting-sources '' --settings harness/agent-settings.json \
#       "Run: bash sandbox_check.sh <harness dir> <runtime dir>  and print its output verbatim"
H="$1"; RT="$2"
t(){ local want="$1" what="$2"; shift 2; if out=$("$@" 2>&1); then r=ALLOWED; else r=BLOCKED; fi
     printf '%-8s (want %-7s) %-34s %s\n' "$r" "$want" "$what" "$(echo "$out" | tr '\n' ' ' | cut -c1-70)"; }
t ALLOWED "robot proxy socket"       curl -sf -m 5 -o /dev/null --unix-socket /tmp/ara/robot.sock http://robot/api/health
t ALLOWED "py robot proxy socket"    .venv/bin/python -c "import socket;s=socket.socket(socket.AF_UNIX);s.connect('/tmp/ara/robot.sock')"
t BLOCKED "[::1]:8091 tcp"           curl -sf -m 5 -o /dev/null 'http://[::1]:8091/api/health'
t BLOCKED "sim via proxy"           curl -sf -m 5 -o /dev/null --unix-socket /tmp/ara/robot.sock http://robot/nodes/sim/state
t BLOCKED "127.0.0.1:8080 (stack)"  curl -sf -m 5 -o /dev/null 'http://127.0.0.1:8080/api/health'
t BLOCKED "127.0.0.1:8095"          curl -sf -m 5 -o /dev/null 'http://127.0.0.1:8095/api/health'
t BLOCKED "localhost:8080"          curl -sf -m 5 -o /dev/null 'http://localhost:8080/api/health'
t BLOCKED "noproxy 127.0.0.1:8080"  curl -sf -m 5 --noproxy '*' -o /dev/null 'http://127.0.0.1:8080/api/health'
t BLOCKED "raw socket 127.0.0.1:8080" .venv/bin/python -c "import socket;socket.create_connection(('127.0.0.1',8080),3)"
t BLOCKED "unix socket sim"         curl -sf -m 5 --unix-socket "$RT/robot/sim.sock" http://x/state
t BLOCKED "py unix socket sim"      .venv/bin/python -c "import socket;s=socket.socket(socket.AF_UNIX);s.connect('$RT/robot/sim.sock')"
t BLOCKED "example.com"             curl -sf -m 8 -o /dev/null https://example.com/
t ALLOWED "pypi"                    curl -sf -m 8 -o /dev/null https://pypi.org/simple/six/
t BLOCKED "cat harness"             cat "$H/run.json"
t BLOCKED "py open harness"         .venv/bin/python -c "open('$H/run.json').read()"
t BLOCKED "ls repo"                 ls "$HOME/Documents/code"
t BLOCKED "cat ~/.claude.json"      head -c 10 "$HOME/.claude.json"
t BLOCKED "ls ~/.claude"            ls "$HOME/.claude"
t BLOCKED "write outside ws"        touch "$HOME/agent_runs/x-sandbox-probe"
t ALLOWED "write in ws"             touch ./x-sandbox-probe
t ALLOWED "pip install"             .venv/bin/pip install -q six
rm -f ./x-sandbox-probe
