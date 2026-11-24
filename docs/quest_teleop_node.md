# Quest teleop node

`python -m robot_utils.nodes.quest_teleop` runs the Quest whole-body teleop client (`tools/quest_teleop`, its
README: the body-frozen scheme, the controls, the remote-consumer rules) as a node on the machine that runs the WBC:
the robot, or the sim stack on a Mac. Its page (`/nodes/quest_teleop/`) replaces the CLI: start / stop teleop,
bring the headset's telemetry up or down, and watch the link, the hands, the WBC and a top-down view.

```bash
python -m robot_utils.nodes.quest_teleop                         # WBC instance `wbc`, headset `ssh quest`, AR :80
python -m robot_utils.nodes.quest_teleop --ar :8080              # the sim stack (stack_nodes.yaml passes this)
python -m robot_utils.nodes.quest_teleop --quest 192.168.8.60 --ssh-host quest-lan --ar off
```

Options: `--wbc` (instance name or URL; default `wbc`), `--arm`, `--swerve` (instance names or URLs, defaults
`arm`, `swerve`, `none` = off: the e-stop goes to them directly), `--max-latency-ms` (150), `--min-rate` (30 Hz),
`--ssh-host` (default `quest`: this machine's
`~/.ssh/config`), `--quest` / `--quest-port` (the headset's stream; default: the ssh host's `HostName` from
`ssh -G`, port 9391), `--ar` (below), `--scheme` (default `body-frozen`), `--hand`, `--ee`, `--no-head-walk`,
`--no-gripper`, `--stale-ms` (as `teleop.py`), `--ctl-timeout` (overall limit of one quest up / down, default
240 s), `--wait` (quest up waits this long for the stream, default 30 s), `--data-dir` (the event log, default
`~/.local/share/robot_utils/quest_teleop/`), `--log-cap-mb` (its size cap, 200), `--sim` (the sim node, instance or
URL, when the stack is a sim: A resets it; default `none`), `--reset-timeout` (5 s), `--max-speed` /
`--max-omega` (the top speed, below; 0.8 m/s, 1.0 rad/s), `--name`, `--port` (debug).

**Top speed and the WBC caps.** `--max-speed` / `--max-omega` (or `POST /top_speed {"max_speed", "max_omega"}`,
the page's **top speed** row) set the sticks' full deflection (the native 0.45 m/s / 0.8 rad/s rescaled; the left
stick clamped to a circle) and the base hint's velocity clamp. Start asserts them as the WBC's `max_speed` /
`max_omega` (`POST /config`, after the lease and before the enable, while the base is still) and remembers the
values they replaced; Stop (any stop: requested, e-stop, auto-stop, a refused start) restores those after the
disable and before the release, so the next lease holder gets the WBC as it was. A cap someone else changed
meanwhile (`POST /config` needs no lease) is left as theirs. Changed while running, the WBC takes the new caps only
while the base is still (disarm with X first: otherwise 409 and nothing changes); stopped, they apply at the next
Start. `/state`'s `teleop.wbc_caps` shows what is asserted and what it replaced. Not covered: a node killed
without a stop leaves the caps asserted (the lease lapses, the caps stay); `wheel_speed`, `max_accel` stay the
WBC's own (a `wheel_speed` below the top speed caps the command first).

Registered in `frontend/nodes.yaml` (the robot, `--ar :80`) and `nodes/sim/stack_nodes.yaml` (the sim stack,
`--ar :8080 --sim sim`), instance `quest_teleop`; in the sim stack it is not in the launcher's default list: Start it
from its tile. The landing page's **Tools** section lists **Quest teleop** (`frontend/tools/quest_teleop.html`, a link to
this node's page). Starting the node moves nothing.

## The page

- **Teleop: Start / Stop.** Start = `teleop.open_wbc`: the WBC lease (holder `quest_teleop node`), the session
  hints (base `locked`), `POST /enable` (the WBC takes the arm and swerve leases), `POST /session`. **Start is not
  live**: nothing moves until **Y** (left controller) arms; X disarms and disables (the session stays open; Y
  re-enables). The rules are the client's (tools/quest_teleop/README.md, "Arming: Y and X"). Armed, one target
  datagram per batch of headset frames; disarmed, keep-alives with `armed: false`; to the WBC's **unix socket**
  (`wbc.target.sock`; UDP only when no unix socket exists on this machine). A refusal shows the WBC's reason and
  gives the lease back. Stop = stop sending, `POST /disable`, release. The armed line: ARMED (press n) or disarmed
  with the reason. The client state: released, engaged, needs release, stale, refused (an engagement refused, e.g.
  no fresh WBC state), with the last reason. `POST /start` stays unauthenticated for now (postponed by the owner).
- **Headset: Quest up / Quest down** run `quest_ctl` up / down on the headset over this machine's `ssh` (one at a
  time, under `--ctl-timeout`; the output appears under the buttons). Up: ssh reachability, update if behind,
  `quest run --no-viewer --scheme=body-frozen [--ar=URL]`, wait for the stream. Down: `quest stop`. **Quest up is
  never run automatically** (starting XR on a headset nobody wears blanks it): only the button. Link health: stream
  connection, frames/s, age, the latency (over the session's best) and the 0.5 s rate (stale and why), focus, the
  **headset battery** (`87%, plugged, 35 C` from the telemetry's `device` block; "not reported" on older headset
  builds), XR session state, session id and scheme, a 30 s rate plot.
- **E-STOP** (the shared button, `POST /estop`): posted to the **arm's, the swerve's and the WBC's** `/estop`
  directly and **in parallel** (one thread each: a slow or dead one never delays the others; each latches and
  coasts), then teleop stops. `latched` in the reply = any latched; `targets` per node. Clear the latches on the
  WBC, arm and swerve pages (this node has no latch of its own; it shows the WBC's).

### Stop paths

| Trigger | Path | Arm | Base | Gripper |
| --- | --- | --- | --- | --- |
| X (left controller) | `armed: false`, WBC `POST /disable` (session open) | floats (gravity compensation) | coasts | damping |
| Stale / late / slow / unfocused input, a discontinuity, the left controller off | `armed: false` (WBC disarm) | held at its commanded joints | brakes to zero within its rate limit, held | keeps its goal |
| A WBC-side disarm (late tick, estimator reset, arm pause, fault, `POST /disarm`) | the WBC's latch | held | brakes to zero (a lost base coasts) | keeps its goal |
| Stop (page) | WBC `POST /disable`, lease released | floats | coasts | damping |
| E-STOP (page) | arm, swerve, WBC `/estop` in parallel; teleop stops | latched, off | latched, coasts (no braking command) | off |
- **Episode:** the open episode (id, time), or resetting / none; the task (`POST /task`), the last episode's
  outcome (kept / aborted: reason), kept / aborted counts, the last event (seq, kind, episode, button or reason), the
  event log (on disk or memory only, seq, lost lines and write errors if any) and why the last A was refused.
- **Hands:** per hand, the clutch (ring outline; thick when engaged) and the trigger (gripper) position.
- **WBC:** limited or not and by what (reach, manipulability, joint limit, ...), state, base mode, lease holder,
  stream age, dropped datagrams.
- **Top-down** (world frame, x up, centred on the base): the base footprint and heading, the base hint (dashed:
  pose + heading), the commanded EE (circle) and the WBC's EE target (cross), and while engaged the operator's head
  mapped through the frozen frames (the base pose at engagement + the head's travel and turn).

Teleop runs in the node, not in the page: closing the page does not stop it. The headset is the dead-man: stale,
late, slow, unfocused or invalid input disarms (README, "Arming: Y and X"). The node also stops teleop itself when
the WBC faults, e-stops, is disabled by anything but our own X, its lease is no longer ours, or its stream is gone
for 2 s.

## Episodes

The controller buttons of docs/plans/data_collection_v1.md ("Decided"), on the right controller. The node only says
what happens, as events (below); **recording is the client-side script's job** (it reads the operator stream and
the event log; the Pi records nothing else).

- **A** (`hands.right.buttons.primary`) = start an episode. A deliberate press, as Y's: A seen released in a valid
  frame first, then held for 3 consecutive fresh valid frames; one action per press. A frame that is not valid
  (unfocused, a discontinuity, the right controller inactive) or not fresh spoils it: a press made or held during
  focus loss never counts. **Armed only**: A while disarmed or disabled (or during a sim reset) is refused: a
  `button` event with `action: "refused"` and the `reason`. A while an episode is open: `episode_stop` (kept) for it,
  then the next `episode_start`, in that order.
- **B tap** (`hands.right.buttons.secondary`, at least 3 fresh valid frames, released < 2 s after it went down) =
  stop the episode and keep it. B held 2 s: the headset stops its telemetry; the open episode is aborted (`b_hold`).
- **Aborts** (`episode_abort`, `reason` a code, `why` the text): `x` (X), `disarm` (any disarm: the WBC's latch, stale
  input, a discontinuity, ...), `disabled` (the WBC disabled by anything), `estop` (ours or seen latched on the WBC),
  `fault` (the WBC's stream or `/state`), `stale` (staleness / latency / rate), `telemetry_loss` (the headset stream
  closed), `b_hold`, `teleop_stop` (Stop, the watchdog, an error, the node shutting down), `reset` (the sim reset
  failed or timed out). Aborted episodes are kept by the script (training filters on the reason).
- **The sim** (`--sim sim` in the sim stack): A first resets the world, one press for the operator: disarm (sent at
  once; Y presses are refused until the sequence ends) -> WBC `POST /disable` (arm and swerve disabled, the
  joint-channel session closed) -> `POST /scene/reset` on the sim (`sim_reset` with its `reset_seq`) -> wait for the
  estimator's reset in the WBC stream (`est_reset` changed, within `--reset-timeout`) -> WBC `POST /enable` (re-seeded
  from the measured joints) -> `episode_start`. The WBC ends **enabled and disarmed**: the operator arms with Y as
  always. **Nothing in the node ever arms**; the robot is never enabled across the jump. A timeout or failure aborts
  the pending episode (`reset`, its id in `episode`) and leaves the WBC disabled (Y enables and arms, as after X); X,
  an e-stop, a fault, stale input, a B tap or stopping teleop during the sequence cancel it the same way (an enable
  already made is undone). On the robot (no `--sim`) A starts the episode at once, armed.
- **Headset (not yet changed):** in body-frozen mode the headset still realigns on A (a new operator-frame revision:
  a discontinuity, which disarms and spoils the press). The plan's *headset* change (A no longer realigns in
  body-frozen mode) is needed before A starts episodes on the real headset.

### Event log

Every event is one line (the contract): `{"seq", "t" (this machine's monotonic s), "wall" (unix s), "kind",
"episode" (the open episode's id or null), "detail"}`. Kinds and details: `button` (`button` A/B/X/Y, `pressed`;
raw edges in valid frames, plus a refused A: `action`, `reason`), `armed` (`arm_seq`), `disarmed` (`reason`),
`enabled` (`by`), `disabled` (`reason`), `episode_start` (`task`), `episode_stop`, `episode_abort` (`reason`, `why`),
`estop` (`reason`, `source`), `fault` (`reason`), `hold` (`reason`: the WBC's hold reason when it changes),
`stale` (`reason`: staleness / latency / rate; `why`), `teleop_start` (`session`, `transport`), `teleop_stop`
(`reason`), `sim_reset` (`reset_seq`). Episode ids `YYYYMMDD-NNNN` (a counter per local day).

One append-only file per local day, `<data dir>/events-YYYYMMDD.jsonl`; each line is flushed and fsynced as it is
written. `seq` is gap-free for the node's life and continues across restarts (the newest file's last line is read
at start; a line cut by a crash is skipped and the next starts on its own line); so does the day's episode counter.
Over the cap (`--log-cap-mb`) the oldest files are deleted (never the one being written). Writing is off the teleop
loop: `emit` numbers the line, hands it to the operator stream and queues it for a writer thread; a full queue (10000
lines) or a failed write (3 attempts) is counted in `/state.eventlog` (`dropped`, `write_errors`, `last_error`) and
logged, never silent; the node writes the queue out before it exits.

### Operator stream

`GET /oplog/stream` (`?max_records=5000`): node-protocol records (docs/nodes.md, "Streams"), JSON header, no
payload, in order of occurrence: `headset` (`t_rx`: arrival, this machine's monotonic clock; `frame`: the raw
`teleop.quest_state.v1` record; every frame, teleop running or not), `sent` (`t_send`, `datagram`: exactly what went
to the WBC), `event` (the event line, its fields flat in the header). A `ping` first and after 1 s idle. A subscriber
more than `max_records` behind loses the oldest records and the next record carries `dropped` (as the rows streams);
publishing never blocks the teleop loop. Lost events are in the log: `GET /eventlog?since_seq=N`.

## The AR view (`--ar`)

Quest up passes `--ar=<URL>` so the headset's robot AR view opens the frontend that serves this node's robot.
`:PORT` (the default form) means this machine's address **on the route to the headset** at that port
(`quest_ctl.resolve_ar`: a UDP connect toward the headset picks the source address; nothing is sent): the Tailscale
address when the headset is reached over Tailscale, the LAN address on a shared LAN. Defaults per environment come
from the registry: `:80` on the robot (its nginx frontend), `:8080` in the sim stack (its landing page; start the
stack with `--host 0.0.0.0`, or the headset cannot reach it). `host[:port]` or a full URL is used as given; `off`
passes none. With no route to the headset the AR view is left off (said in the output).

## API

| | |
|---|---|
| `GET /state` | `state` (client state), `error`, `teleop` (with `armed`, `arm_seq`, `disarm_reason`, `disabled_by_x`, `input`: fresh, why, latency, rate), `headset` (with `battery`), `battery_pct`, `wbc` (echo of its `/state`, 1 Hz, with `armed`, `disarm_reason`, and stream age), `lease` (the WBC's), `estop` (the WBC's), `ctl` (the headset command: busy, rc, output lines), `config`, `latest` (row), `events` |
| `GET /events` | SSE of `/state` |
| `GET /fields`, `GET /stream` | rows every 50 ms: client state, sending, headset link (connected, rate, age, focus, XR state), per hand valid / engaged / gripper, engagements, targets sent, the WBC's state / base mode / limited / limited_by / row age, base pose, EE commanded / target / sent, base hint, head, stick offset; appended: `armed`, `wbc_armed`, `quest_latency`, `quest_rate_short`, `battery_pct` |
| `POST /start`, `POST /stop` | teleop (409 with the WBC's reason); Start is not live (Y arms) |
| `POST /estop` | the arm, the swerve and the WBC in parallel; stops teleop |
| `POST /quest/up`, `POST /quest/down` | quest_ctl on the headset (409 while one runs) |
| `POST /task {"text"}` | the task of the next episodes (in `/state.task` and each `episode_start`) |
| `GET /oplog/stream` | the operator stream (above) |
| `GET /eventlog?since_seq=N&limit=M` | JSON lines (`application/x-ndjson`) after N from the log files, at most M (default 1000): repeat from the last seq until fewer come back |
| `GET /eventlog/files` | `[{name, bytes, first_seq, last_seq}]` |

`/state` also has `episode` (the open one: `{id, task, t_start, start_seq}`, else null; the contract), `episodes`
(state idle / resetting / recording, pending id, task, last outcome, counts, the last refusal), `task`,
`last_event`, `eventlog` (dir, seq, written_seq, queued, dropped, write_errors, last_error, bytes, cap_bytes,
pruned) and `oplog` (subscribers, records per type).

## Storage

Only the event log (above; events, never frames) under `--data-dir`, a deliberate small exception like
agent_interface's transcripts: no recording (the client-side script records the streams, docs/nodes.md), no state
files; log lines go to stderr (the journal under systemd). Constructed without a `data_dir` (tests, library use) the
event log stays in memory. Tested: start, teleop,
stop, e-stop and both headset commands leave the node's working directory and `$HOME` empty.

## Setting up a machine

- **ssh to the headset:** `Host quest` in the user's `~/.ssh/config` (HostName, `Port 8022`, the key) with the key
  accepted by the headset's quest-dev-runtime SSH server; `ssh -T -o BatchMode=yes quest echo ok` must work
  non-interactively. On the Mac this already works (Tailscale, 100.72.77.83).
- **A route to the headset** for the stream (TCP 9391) and back for the AR view (the frontend's port). On the robot
  without Tailscale, both on the robot's Wi-Fi: give the headset a fixed (DHCP-reserved) address there and set
  `HostName` to it (or pass `--quest`).

## Tests

`tests/test_quest_teleop_node.py`: start / stop against a fake WBC (HTTP, rows, its unix datagram socket) and the
fake headset; a refused start; the e-stop to the WBC; quest up / down through a scripted ssh runner (commands, the AR
URL, one at a time, the overall deadline); no files written; the registries and the tool entry; and the node with
the real WBC node (arm only, the fake arm) over its unix socket, e-stop reaching the arm.
`tests/test_teleop_events.py`: the event log (fsync per line, the line's shape, seq and the episode counter across
restarts and a cut line, pruning, `since_seq` across files with `limit`, memory mode, a full queue counted, write
errors, close writes the queue, the HTTP endpoints). `tests/test_teleop_oplog.py`: order across record types, drops
counted on the next record, publishing never blocks, the stream over HTTP, and end to end (fake headset, fake WBC)
every frame and every datagram the WBC received, in order with the events. `tests/test_teleop_episodes.py`: A / B
press rules, refusals, stop-then-start, every abort reason, the sim reset sequence (disable -> reset -> est_reset ->
enable, still disarmed; Y refused meanwhile; timeout; X cancels), no arming but a Y press.
`tests/test_quest_teleop_node_estop.py`: the parallel e-stop (a slow arm does not delay the swerve or the WBC; a dead
one does not stop the others), Start not live, X disabling without the watchdog stopping teleop, Y arming through
the node, armed / latency / battery in `/state` and the stream. The page is in
`tests/test_pages_contract.py` and the layout contract of `tests/test_visual_safety.py`.

## Code

`client.py` is the adapter: it imports the client from `tools/quest_teleop` (as the top-level package
`quest_teleop`) and sends to the unix socket (and reports each datagram to the operator stream). The client core
gained observer hooks for the node (`TeleopClient.on_frame` / `on_step` / `on_event`), the latch an `inhibit` (no Y
press arms while set: the sim reset) and `episodes.py` (the A / B press rules, pure). In the node: `eventlog.py`
(the event log), `oplog.py` (the operator stream), `episodes.py` (the episode states and the sim reset). Moving the client core into the package is planned once the velocity
feedforward work on `body_frozen.py` / `wbc_client.py` has landed.
