# Rings coding agent

A harness for letting a Claude Code agent do the sim's **rings** sorting task (red objects onto the outer ring, blue
onto the inner one) through the robot's own API only: no simulator ground truth, no source code, no neural
perception, this Mac only. Rules are stated in the agent's brief ([workspace/CLAUDE.md](workspace/CLAUDE.md)) and
audited afterwards from the transcript and the request log. The setup also enforces what it cheaply can.

    tools/agent_rings/run.sh              # launch a run in tmux (prints the session name)
    tools/agent_rings/run.sh --no-agent   # everything but the agent
    tools/agent_rings/stop.sh rings-<ts>  # stop it (the run directory stays)
    tools/agent_rings/run.sh --inherit ~/agent_runs/rings-<old>/workspace
                                          # a fresh agent with an earlier agent's code only

**Inheriting:** `--inherit` gives the new agent an earlier run's code and nothing else: its `*.py` / `*.sh` files go to
`workspace/inherited/` and its venv's `pip freeze` to `inherited/requirements.txt`, and the brief gets a short section
([workspace/INHERITED.md](workspace/INHERITED.md)). Its notes, logs, images, measured data and transcript stay behind.
Its code comments do carry what it learned, so the new run isn't context-free in a strict sense.

## What runs (tmux session `rings-<ts>`)

| window | what |
|---|---|
| `stack` | the agent's own sim stack ([stack_nodes.yaml](stack_nodes.yaml): the WBC with UDP off, no headset teleop), `sim_rings_dynamic.yaml`, **socket-only** (`--port 0`) under runtime dir `/tmp/ar`. It doesn't touch any other stack. |
| `proxy` | [proxy.py](proxy.py) on the unix socket `/tmp/ara/robot.sock`: the agent's only door. It forwards the robot nodes and refuses `/nodes/sim` and the landing page's sim / observe / restart APIs (403). It logs every request to `~/agent_harness/<run>/requests.jsonl`. |
| `relay` | [dgram_relay.py](dgram_relay.py) on `/tmp/ara/wbc.target.sock`: the agent's WBC target datagrams, forwarded to the WBC's own socket. Each second is summarised in `~/agent_harness/<run>/wbc_datagrams.jsonl`. The proxy rewrites the WBC's `POST /session` reply so it names this socket. |
| `score` | [score.py](score.py) reads the sim's ground truth over its socket every 10 s and writes `~/agent_harness/<run>/scores.jsonl`. The baseline is 16/32. |
| `record` | `collect run`: the arm, swerve, estimator and WBC streams plus the rigid scene, into `~/agent_harness/<run>/recording/`, about 15 MB/min. `collect browse` and the replay tool show it. |
| `agent` | Claude Code, interactive, with **Remote Control** (`--remote-control rings-<ts>`), so the run can be followed from claude.ai or the phone. |

## Run directories: `~/agent_runs/rings-<ts>/workspace`, `~/agent_harness/rings-<ts>/`

- **`workspace/`** is the agent's working directory, holding:
  - `CLAUDE.md`, the brief;
  - `docs/`, copies of the robot nodes' docs (not sim, scene3d or collect);
  - `.venv`, a plain venv the agent may pip-install into;
  - `NOTES.md`.
- **`~/agent_harness/rings-<ts>/`** is out of every agent's reach (all of `~/agent_harness` is denied, so a later run's scores are hidden from a live agent too). It holds:
  - `run.json`: the session id, transcript path and commit;
  - `agent-settings.json`;
  - `requests.jsonl`, `scores.jsonl`, `stack.log` and `recording/`.

## Containment

The agent CLI runs with `--permission-mode dontAsk --setting-sources '' --settings ~/agent_harness/<run>/agent-settings.json`, written by [settings.py](settings.py). Because the settings come in as a flag, they need no workspace trust, and the agent can't edit them.

- **Tools:** Read, Edit, Write, Glob, Grep and Bash are allowed. Anything else is refused without a prompt: WebFetch, WebSearch, ssh, scp, rsync, sudo, `git push`, and edits to `.claude/`.
- **Bash runs in the OS sandbox** (seatbelt; `allowUnsandboxedCommands: false`):
  - **Writes:** only in the workspace.
  - **Reads:** denied for the repo (`~/Documents`), `~/data`, `~/.claude*`, the harness dir, the stack's runtime dir `/tmp/ar` (its unix sockets included), other stacks' socket dirs, other Claude sessions' `/tmp/claude-<uid>/*`, conda envs, exports, ssh and aws.
  - **Network:** PyPI through the sandbox's proxy, plus two unix sockets, the robot proxy's and the WBC relay's (`allowUnixSockets`). No loopback TCP at all. The allowlist is per host, not per port, and `allowLocalBinding` opens all of loopback, so a TCP robot port would expose every other stack on the Mac too (the operator's `*:8080`, others on `127.0.0.1`). Checked with sandbox_check.sh.
  - **pip:** the venv's `pip.conf` uses pip's own CA bundle (the sandbox blocks the keychain lookup, so the default fails) and a cache inside the workspace.
- **The Read, Edit, Grep and Glob tools** get matching deny rules.
- **Not enforced, only audited:** the no-neural-perception rule and "no reset as a shortcut". The agent can still stop and start robot nodes through the landing page. That's its recovery path, and the sim node is excluded.

[sandbox_check.sh](sandbox_check.sh) probes all of the above from inside the sandbox. Run it in a run's workspace before trusting a new setup or CLI version.

## Audit afterwards

- **The transcript:** `~/agent_harness/<run>/run.json` → `transcript`, kept by Claude Code for 30 days. Copy it out.
- **`~/agent_harness/<run>/requests.jsonl`:** filter it for `"blocked": true`.
- **The score over time:** `~/agent_harness/<run>/scores.jsonl`.
