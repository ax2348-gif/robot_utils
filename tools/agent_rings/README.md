# Rings coding agent

A harness for letting a Claude Code agent do the sim's **rings** sorting task (red objects onto the outer ring, blue
onto the inner one) through the robot's own API only: no simulator ground truth, no source code, no neural
perception, this Mac only. Rules are stated in the agent's brief ([workspace/CLAUDE.md](workspace/CLAUDE.md)) and
audited afterwards from the transcript and the request log. The setup also enforces what it cheaply can.

    tools/agent_rings/run.sh              # launch a run in tmux (prints the session name)
    tools/agent_rings/run.sh --no-agent   # everything but the agent
    tools/agent_rings/stop.sh rings-<ts>  # stop it (the run directory stays)

## What runs (tmux session `rings-<ts>`)

| window | what |
|---|---|
| `stack` | the agent's own sim stack ([stack_nodes.yaml](stack_nodes.yaml): no WBC, no headset teleop), `sim_rings_dynamic.yaml`, **socket-only** (`--port 0`) under runtime dir `/tmp/ar`. It doesn't touch any other stack. |
| `proxy` | [proxy.py](proxy.py) on `[::1]:8091`: the agent's only door. It forwards the robot nodes and refuses `/nodes/sim` and the landing page's sim / observe / restart APIs (403). It logs every request to `harness/requests.jsonl`. |
| `score` | [score.py](score.py) reads the sim's ground truth over its socket every 10 s and writes `harness/scores.jsonl`. The baseline is 16/32. |
| `record` | `collect run`: the arm, swerve and estimator streams plus the rigid scene, into `harness/recording/`, about 15 MB/min. `collect browse` and the replay tool show it. |
| `agent` | Claude Code, interactive, with **Remote Control** (`--remote-control rings-<ts>`), so the run can be followed from claude.ai or the phone. |

## Run directory `~/agent_runs/rings-<ts>/`

- **`workspace/`** is the agent's working directory, holding:
  - `CLAUDE.md`, the brief;
  - `docs/`, copies of the robot nodes' docs (not sim, scene3d or collect);
  - `.venv`, a plain venv the agent may pip-install into;
  - `NOTES.md`.
- **`harness/`** is out of the agent's reach. It holds:
  - `run.json`: the session id, transcript path and commit;
  - `agent-settings.json`;
  - `requests.jsonl`, `scores.jsonl`, `stack.log` and `recording/`.

## Containment

The agent CLI runs with `--permission-mode dontAsk --setting-sources '' --settings harness/agent-settings.json`, written by [settings.py](settings.py). Because the settings come in as a flag, they need no workspace trust, and the agent can't edit them.

- **Tools:** Read, Edit, Write, Glob, Grep and Bash are allowed. Anything else is refused without a prompt: WebFetch, WebSearch, ssh, scp, rsync, sudo, `git push`, and edits to `.claude/`.
- **Bash runs in the OS sandbox** (seatbelt; `allowUnsandboxedCommands: false`):
  - **Writes:** only in the workspace.
  - **Reads:** denied for the repo (`~/Documents`), `~/data`, `~/.claude*`, the harness dir, the stack's runtime dir `/tmp/ar` (its unix sockets included), other stacks' socket dirs, other Claude sessions' `/tmp/claude-<uid>/*`, conda envs, exports, ssh and aws.
  - **Network:** only through the sandbox proxy, to `::1` and PyPI. The sandbox's network allowlist is per host, not per port. So the proxy listens on `[::1]`, while the other stacks (the operator's on `*:8080`, others on `127.0.0.1`) stay unreachable.
- **The Read, Edit, Grep and Glob tools** get matching deny rules.
- **Not enforced, only audited:** the no-neural-perception rule and "no reset as a shortcut". The agent can still stop and start robot nodes through the landing page. That's its recovery path, and the sim node is excluded.

[sandbox_check.sh](sandbox_check.sh) probes all of the above from inside the sandbox. Run it in a run's workspace before trusting a new setup or CLI version.

## Audit afterwards

- **The transcript:** `harness/run.json` → `transcript`, kept by Claude Code for 30 days. Copy it out.
- **`harness/requests.jsonl`:** filter it for `"blocked": true`.
- **The score over time:** `harness/scores.jsonl`.
