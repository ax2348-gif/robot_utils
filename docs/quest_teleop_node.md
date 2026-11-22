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
240 s), `--wait` (quest up waits this long for the stream, default 30 s), `--name`, `--port` (debug).

Registered in `frontend/nodes.yaml` (the robot, `--ar :80`) and `nodes/sim/stack_nodes.yaml` (the sim stack,
`--ar :8080`), instance `quest_teleop`; in the sim stack it is not in the launcher's default list: Start it from its
tile. The landing page's **Tools** section lists **Quest teleop** (`frontend/tools/quest_teleop.html`, a link to
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

## Storage

Nothing is written to disk on any machine: no recording option (clients record streams, docs/nodes.md; a recorder
will be its own tool), no state files; log lines go to stderr (the journal under systemd). Tested: start, teleop,
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
`tests/test_quest_teleop_node_estop.py`: the parallel e-stop (a slow arm does not delay the swerve or the WBC; a dead
one does not stop the others), Start not live, X disabling without the watchdog stopping teleop, Y arming through
the node, armed / latency / battery in `/state` and the stream. The page is in
`tests/test_pages_contract.py` and the layout contract of `tests/test_visual_safety.py`.

## Code

`client.py` is the adapter: it imports the client from `tools/quest_teleop` unchanged (as the top-level package
`quest_teleop`) and sends to the unix socket. Moving the client core into the package is planned once the velocity
feedforward work on `body_frozen.py` / `wbc_client.py` has landed.
