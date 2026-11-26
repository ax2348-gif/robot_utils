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
`--no-gripper`, `--tether` (below), `--stale-ms` (as `teleop.py`), `--ctl-timeout` (overall limit of one quest
up / down, default 240 s), `--wait` (quest up waits this long for the stream, default 30 s), `--data-dir` (the event log, default
`~/.local/share/robot_utils/quest_teleop/`), `--log-cap-mb` (its size cap, 200), `--sim` (the sim node, instance or
URL, when the stack is a sim: A resets it; default `none`), `--reset-timeout` (5 s), `--max-speed` /
`--max-omega` (the top speed, below; 0.8 m/s, 0.9 rad/s), `--resume-grace` (2 s), `--resume-hold-max` (60 s),
`--resume-off CLASS,...`, `--no-resume` (the tiered recovery, below), `--name`, `--port` (debug).

**Tiered recovery** (tools/quest_teleop/README.md, "Hold and re-grip"). A transient dropout (stale, late or slow
input, a gap, an unfocused frame, the left controller briefly off, the WBC rows stale, the WBC's own short dropout:
a tick overrun, its `hold_timeout`, an unusable estimator pose) **holds**: the robot holds as on a disarm, the
armed line shows `HOLD — <cause>: release and re-grip to resume` (or `waiting for it to clear (t / 2.0 s)`), and once
the cause has cleared **a release and re-grip resumes with no Y** (a new `arm_seq`; the WBC's latch and its
`arm_target_pos/rot` check are unchanged). A reconnect of the same headset session (the reader reopens a connection
silent for 1 s: a Wi-Fi blackout) is a gap, and the episode goes on through it. Longer than `--resume-grace`, or
serious (a new headset session, an estimator reset or pose jump, the WBC disabled or in a fault, any other WBC disarm reason): `DISARMED — press Y`.
X, the e-stop, Stop and a sim reset are deliberate: never resumed. The page's **re-grip resume** row: On / Off, the
grace window, and one toggle per transient class (`input_stale`, `input_late`, `input_gap`, `input_invalid`,
`left_inactive`, `wbc_stale`, `wbc_disarm`, `no_command`; off: that cause disarms as before); `POST /resume`
(`enabled`, `grace_s`, `hold_max_s`, `classes`), applied at once. The operator stream's `tier` column: 0 disarmed, 1
hold, 2 armed.

**Stick tether** (off by default since 2026-11-26, the stick ramp instead; `--tether`, the page's "Stick tether" toggle, `POST /tether/{on|off}`; from the next
grip). Engaged, the stick offset leads the WBC's commanded base by at most 5 cm / 0.1 rad (zero while the base brakes
from the stick, frozen without a fresh WBC row), and the arm keeps the commanded base's heading for the stick's yaw,
so the arm does not reach ahead while the base accelerates and does not swing back when it brakes. The base hint's
velocity is still the raw stick. Walking's lead is unchanged. `/state`: `config.tether`; `teleop.tether` (engaged,
on): `xy`, `yaw` (`idle` / `active` / `braking`), `lead_m`, `lead_rad` (the stick's share of the lead).
tools/quest_teleop/body_frozen.py, "The stick tether".

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
| Stale / late / slow / unfocused input, a discontinuity, the left controller off | `armed: false` (WBC disarm); transient: hold, a re-grip resumes | held at its commanded joints | brakes to zero within its rate limit, held | keeps its goal |
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
  **A hold keeps the episode** (the re-grip resume tiers): a transient dropout the latch holds through (`held`) does
  not abort, nor does its `stale` event; the episode goes on through the hold and the resume. A hold that escalates
  (`escalated`: past the grace window, or serious) aborts it as `disarm`. With `--no-resume` (or the class off),
  every dropout aborts as before (`stale` / `disarm`). Training can cut the held stretches from the `held` /
  `resumed` events.
- **The sim** (`--sim sim` in the sim stack): A first resets the world, one press for the operator: disarm (sent at
  once; Y presses are refused until the sequence ends) -> WBC `POST /disable` (arm and swerve disabled, the
  joint-channel session closed) -> `POST /scene/reset` on the sim (`sim_reset` with its `reset_seq`) -> wait for the
  estimator's reset in the WBC stream (`est_reset` changed, within `--reset-timeout`) -> WBC `POST /enable` (re-seeded
  from the measured joints) -> `episode_start`. The WBC ends **enabled and disarmed**: the operator arms with Y as
  always. **Nothing in the node ever arms**; the robot is never enabled across the jump. A timeout or failure aborts
  the pending episode (`reset`, its id in `episode`) and leaves the WBC disabled (Y enables and arms, as after X); X,
  an e-stop, a fault, stale input, a B tap or stopping teleop during the sequence cancel it the same way (an enable
  already made is undone). On the robot (no `--sim`) A starts the episode at once, armed.
- **Sim aids** (`--sim`; `nodes/quest_teleop/sim_aids.py`, the page's sim row, `POST /sim/ghost/{on|off}`,
  `POST /sim/auto_pause/{on|off}`, `/state` `sim_aids`):
  - **Goal ghost** (on; `--no-ghost`): the gripper drawn at the WBC's target, coloured by target − measured
    (green, yellow at half, red from 5 cm or 20°), see-through. Posted to the sim relative to the measured end
    link (tcp · meas⁻¹ · tgt · tcp⁻¹), so estimator drift cancels; hidden while there is no target.
  - **Auto pause** (off; `--auto-pause`): sim time stops while the operator is neither clutched nor touching a
    stick for 0.25 s, and runs again at the next grip or stick. Only while teleop runs, never during an episode's
    sim reset (it waits on the estimator); the node only resumes what it paused. Events `sim_pause` / `sim_resume`
    mark the frozen spans for recordings. Measured: 4 s paused leaves the WBC enabled, the estimator healthy and the
    arm servoing.
- **Headset (not yet changed):** in body-frozen mode the headset still realigns on A (a new operator-frame revision:
  a discontinuity, which disarms and spoils the press). The plan's *headset* change (A no longer realigns in
  body-frozen mode) is needed before A starts episodes on the real headset.

### Event log

Every event is one line (the contract): `{"seq", "t" (this machine's monotonic s), "wall" (unix s), "kind",
"episode" (the open episode's id or null), "detail"}`. Kinds and details: `button` (`button` A/B/X/Y, `pressed`;
raw edges in valid frames, plus a refused A: `action`, `reason`), `armed` (`arm_seq`), `disarmed` (`reason`),
`enabled` (`by`), `disabled` (`reason`), `episode_start` (`task`), `episode_stop`, `episode_abort` (`reason`, `why`),
`estop` (`reason`, `source`), `fault` (`reason`), `hold` (`reason`: the WBC's hold reason when it changes),
`stale` (`reason`: staleness / latency / rate; `why`), the tiered recovery's `held` (`reason`, `class`, `cue`: a
transient put the client in the hold; not the WBC's `hold` above), `resumed` (`arm_seq`, `hand`, `after_s`, `cause`,
`class`: a re-grip resumed it), `resume_refused` (`arm_seq`, `cause`, `refusals`: the WBC refused the resume; still
holding), `escalated` (`reason`, `cause`, `class`, `cue`: the hold became a disarm, Y re-arms), `y_refused` (`reason`:
a Y press that did not arm, e.g. input not fresh, a grip pressed, the WBC's state stale; also felt), `teleop_start` (`session`, `transport`), `teleop_stop`
(`reason`), `sim_reset` (`reset_seq`), `sim_pause` / `sim_resume` (auto pause: `reason`). Episode ids `YYYYMMDD-NNNN` (a counter per local day).

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

## Recording in the sim (`--record`)

In a sim stack (`--sim`), Start also starts the collect script and Stop (any stop: the button, an auto-stop, a fault,
the node shutting down) ends it: every teleop run becomes a session under `--record DIR` (default `~/data/sim`),
with the sim's rigid scene; `--no-record` turns it off. Details, the design and the files: docs/collect.md, "Sim
teleop records itself". `/state` `recording`: `enabled`, `active`, `dir`, `pid`, `exit_code`, `error`, the
streams (connected, rate, records, gaps), episodes and scenes saved; the page's Episode card shows it.

## Two hands, two arms (`--robot`, `--hands`)

On a two-arm robot (`--robot rebot_bimanual`) each hand drives its own arm by default: the left grip clutches
`arm_left`, the right `arm_right`; each trigger is its own arm's gripper; a released arm rides with the base; a hand
that is not tracked only rides. The base: no hand engaged, the sticks; one engaged, that hand's engagement (as with
one hand); both engaged, the hand that engaged first keeps the base until it releases, then the other's engagement
takes over (it tracked the same head and stick motion, each tethered to the same commanded base, so it continues
from where the base is). `--hands
left=EE,right=EE` maps them otherwise; `--hand` / `--ee` select one-hand teleop (default `arm_right`). The e-stop
goes to every arm; the page labels each hand's arm (and which drives the base); `/state` `teleop.hands` per hand. Dry
run: `python tools/quest_teleop/fake_headset.py --port 9396 --scenario two_hands` and the sim stack's
`--quest 127.0.0.1:9396`. tools/quest_teleop/body_frozen.py (`MultiCore`).

### Mirror pairing (`--mirror off|auto|straight|mirror`)

Which arm each hand drives, decided when the first hand engages from all released and locked until both release:
`off` and `straight` = the mapping above; `mirror` = swapped (the operator's left hand drives the robot's right
arm); `auto` (the node's default) = mirrored when the operator's and the robot's headings are more than 90° apart (the
robot faces the operator). Only the pairing changes: the EE motions, the clutch and the riding are as above, and the trigger
follows the paired arm. The page's "pairing" row switches it while no hand is engaged (`POST /mirror/{mode}`, 409
while one is); `/state` `config.mirror` and `teleop.pairing` (`mode`, `pairing`, `hands`, `locked`, per arm
`mirror`, the last `decision`); a `pairing` event per decision. Dry run: `--scenario mirror`.
body_frozen.py "Mirror pairing".

## The AR view (`--ar`)

Quest up passes `--ar=<URL>` so the headset's robot AR view opens the frontend that serves this node's robot.
`:PORT` (the default form) means this machine's address **on the route to the headset** at that port
(`quest_ctl.resolve_ar`: a UDP connect toward the headset picks the source address; nothing is sent): the Tailscale
address when the headset is reached over Tailscale, the LAN address on a shared LAN. Defaults per environment come
from the registry: `:80` on the robot (its nginx frontend), `:8080` in the sim stack (its landing page; start the
stack with `--host 0.0.0.0`, or the headset cannot reach it). `host[:port]` or a full URL is used as given; `off`
passes none. With no route to the headset the AR view is left off (said in the output).

## Body tracking (`--body`)

The headset can forward Meta body tracking: shoulders, elbows, spine and head (and legs, with the full-body joint
set). It sends every tracked joint's LOCAL pose and flags, the joint set, confidence and fidelity, as a `body`
block in each frame. The block is quest_env BODY_TRACKING.md, schema `teleop.quest_body.v1`.

`--body core|all` (default `off`) makes quest up run the headset's telemetry with `--body=core|all`. It needs
quest_env with body tracking and runtime 1.7.0 with the permission granted (docs/quest_setup.md).

The reader parses and validates the block (`tools/quest_teleop/body.py`). Only the base hint's source (`chest`,
`hips`) and the elbow-swivel hint (`--elbow-hint`) use it, both off by default: otherwise the commands are identical
with or without it (`test_quest_body.py`, `test_quest_swivel.py`). A malformed block is counted and dropped; its
frame is used as before.

- `/state` `headset.body`:
  - `present`: a valid block in the last second;
  - `age`, `frames`;
  - `available`, or `reason` when the headset has no tracker;
  - `active`, `joint_set`, `sent`, `valid`, `tracked`, `confidence`, `fidelity`, `fidelity_requested`,
    `skeleton_changed_count`;
  - `bad` and `last_error`: malformed blocks.
- The page's headset link shows a "body" line.
- The operator stream records the block with the rest of the frame, so recordings and replays have it.
- `config.body` is the setting.

Use `body.parse_body(frame)` to read it. `Body.joint("left_elbow")` returns `(p, q_xyzw, flags)` in LOCAL. The
elbow is OpenXR's "arm lower" joint (ids 11 and 16).

### The base hint's source (`--hint-source head|chest|hips`)

While engaged, the base hint follows the headset by default (`head`). With body tracking it can follow the chest or
the pelvis (`hips`) instead, smoothed, and falling back to the headset when the body is not usable. The rules are
in tools/quest_teleop/README.md, "The base hint's source". It applies to the robot and the sim alike, from the next
grip.

- **Settings.** `--hint-tau`, `--hint-deadband M RAD` and `--hint-min-confidence`.
- **Page.** "base follows Head / Chest / Hips", with the source in force and the body's confidence.
- **API.** `POST /hint_source/{head|chest|hips}`.
- **`/state`.** `config.hint_source`, and `teleop.hint`:
  - `selected`;
  - `engaged_source` (this engagement's);
  - `in_force` (`head` while falling back, null when lost);
  - `confidence`;
  - `why_not` (why the selected body source is unusable);
  - `switches`.
- **Operator stream.** While teleop runs, a `hint` record per valid headset frame holds all three candidates (head,
  chest, hips), raw and smoothed, whichever source is selected.
- **Analysis.** `tools/quest_teleop/hint_analysis.py <recording>` reports jitter at rest, filter lag and head-vs-body
  yaw divergence.

### The elbow-swivel hint (`--elbow-hint`)

Teleop's half of a relative elbow-swivel hint: **input and UI only**. Per engaged hand, the target datagram's
paired EE carries a `swivel` object; the WBC fades, smooths and anchors it to the robot's arm (none of that here).
tools/quest_teleop/swivel.py has the rules.

```json
"ees": {"arm_right": {"frame": "world", "pos": [...], "quat": [...], "swivel": {"seq": 812345671, "dpsi": 0.31, "elbow": 2.05, "conf": 0.9}}}
```

| Field | Meaning |
|---|---|
| `seq` | new on every engagement of the hand paired to this EE: strictly increasing per client, seeded from the clock in ms (not the headset's per-hand `engagement_id`, which restarts with each XR session and repeats across hands) |
| `dpsi` | rad: the operator's swivel change since that engagement, **positive = the elbow moving out** (away from the body) for either arm: the right-handed angle about the shoulder → wrist axis × +1 (left arm) / −1 (right arm), the WBC's convention (`wbc/swivel_geom.py`). So out maps to out in either pairing: mirrored, the operator's left elbow out drives the robot's right elbow out (its mirror image), no extra sign |
| `elbow` | rad: the operator's upper-arm–forearm angle, π = straight |
| `conf` | 0..1: the body's confidence × 0.5 per joint (of the three) whose position is valid but not tracked |

- **The joints.** The shoulder is `<side>_arm_upper` (the shoulder centre; `shoulder` and `scapula` are on the
  trunk), the elbow `<side>_arm_lower`, the wrist the body's own `<side>_hand_wrist` (not the controller shifted to
  the wrist: one solver and one sample time, so a body lag cannot read as a swivel).
- **The swivel change.** The elbow's rotation about the shoulder → wrist axis (right-handed), summed frame by frame
  by parallel transport since the engagement. There is no fixed reference direction, so an arm hanging or raised
  vertically swivels like any other. While the arm is nearly straight (the elbow within 2 cm of the axis) or the
  joints are not valid, `dpsi` holds and the next defined frame continues from the last one.
- **Omitted** when the frame has no body block, an inactive one, or the arm's three joints are not position-valid;
  and on released arms.
- **Toggle.** `--elbow-hint` (default off), the page's "elbow hint On / Off", `POST /elbow_hint/{on|off}`. On
  applies from each hand's next grip (the WBC anchors at a new `seq`), off at once. Off, the datagrams are exactly
  those without it. It needs `--body` (warned otherwise).
- **`/state`.** `config.elbow_hint`, and `teleop.swivel` per EE (the pairing in force, while teleop runs): `hand`,
  `enabled`, `engaged`, `pending` (on, but this engagement started before), `sending`, `seq`,
  `valid` / `reason` (the input), `dpsi`, `elbow`, `conf`, `tracked` (of the three joints), `straight`, and the
  WBC's `robot_swivel` / `robot_swivel_w` when its stream has `<ee>_swivel` / `<ee>_swivel_w` (else null).
- **Page.** A line per arm under the toggle: the input (elbow angle and confidence, or why it is not valid), Δψ while
  engaged, sent / not sent, and the robot's swivel and the hint in force when the WBC publishes them.
- **Dry run.** `python tools/quest_teleop/fake_headset.py --port 9396 --body core --scenario elbow`: the right
  elbow swivels +0.6 rad, back to +0.2, a new grip, then -0.4 rad (`HeadsetState.elbow_swivel`).

### On-device check (not yet done)

The headset is worn on the neck, so Meta's body model sees a "head" about 25 cm low. Before the elbows are used,
check their quality:

1. Install runtime 1.7.0 and grant the permission. Start telemetry with `--body=all` (or `quest_up --body all`).
2. Record with `nc <quest>:9391 > body.jsonl` (or `collect`) while you do these motions, about 10 s each:
   1. arms relaxed at the sides;
   2. reaching forward at chest height;
   3. elbows out (chicken wings);
   4. elbows tucked in with the forearms forward;
   5. reaching overhead;
   6. **the pull-up pose**: hands above the head, elbows bent and pulled down to the sides;
   7. crossing in front of the body;
   8. walking with the controllers held.
3. For each motion, compute from the records:
   - `active` and `confidence`;
   - the fidelity;
   - the flags of joints 8–17 (shoulders, upper arms, elbows, wrist twists);
   - the elbow's distance to the controller (it should be about the forearm length, steady);
   - its position relative to the shoulder (the elbow should go where it visibly is).

   Run the same check once with the headset on the head as a reference.
4. Run `tools/quest_teleop/hint_analysis.py` on a recording of standing, reaching with a torso twist and walking.
   It compares how the head, chest and hips sources jitter and diverge before you choose one.
5. Look for the failure modes:
   - the elbows frozen or mirrored;
   - the flags dropping to 3 (inferred, not tracked): with controllers in hand this is normal for the arms (Meta infers
     them), so the hint's `conf` is the body confidence alone, not reduced per inferred joint;
   - confidence collapsing in the pull-up or overhead poses;
   - fidelity falling to low.

## Haptics (controller vibration)

`haptics.py`. The node turns its events and what the WBC and the arms report into **cues**, and serves the headset
the state to play, schema `teleop.quest_haptics.v1` (quest_env HAPTICS.md): per hand a list of **pulses**
(`id`, `group`, `amplitude` 0..1, `duration_s`, `frequency_hz` (0: the runtime's), `delay_s`, `age_s`) and a
continuous **level** (0..1, valid `ttl_s` = 0.5 s after the line arrives). Pulse ids increase for the node's life
(`epoch` names it): the headset plays each once. Pulses of the last 1 s are repeated in every line, so a reconnect
loses none; the headset skips one older than 0.5 s.

**Channel.** The headset holds one `GET /haptics?stream=1` open (chunked NDJSON through the frontend, like the AR
state stream; quest_env derives the URL from `--ar`: `<ar>/nodes/quest_teleop/haptics?stream=1`). The node writes a
line the moment a cue is made or a level changes, else a keepalive every 0.2 s while anything is on (the level's
TTL) and every 1 s idle (the headset reconnects after 2 s without a byte). So event -> buzz is the one-way network
time plus at most one XR frame (14 ms at 72 Hz): under the 50 ms target on a LAN or a direct Tailscale path (not
yet measured on the headset: `ack_ms` shows it, one frame period more than event -> buzz); a plain poll
would add half its period plus a TCP connect per request (the headset's HTTP client is one request per connection),
i.e. 50 Hz of connects for the same latency. `GET /haptics` (no `stream`) is one snapshot (debugging). The headset
acknowledges in its frames (`haptics`: `schema`, `epoch`, `last_id`, `connected`): `GET /haptics/info` `ack_ms` is
the round trip cue -> frame acknowledging it. Without `--ar` (or an older quest_env) there is no stream: `readers` 0.

**Cues** (`CUES`; the event kinds map through `EVENT_CUES`, a table: a new kind is one line):

| cue | from | pattern | default |
|---|---|---|---|
| armed | `armed` | one short (60 ms), both hands | on, 0.6 |
| disarmed | `disarmed`, `escalated` (a hold escalated to a disarm) | two short (60 ms, 80 ms apart) | on, 0.6 |
| held | `hold` (the WBC's; not its echo of a disarm), `held`, `resume_refused` | two short | on, 0.6 |
| resumed | `resumed` | one short | on, 0.6 |
| disabled | `disabled`, `estop` | one long (400 ms) | on, 0.8 |
| refused | `y_refused` (a Y press that did not arm; why: the event and the page) | three quick taps (30 ms) | on, 0.7 |
| contact | an arm's gripper closing on something: its force limit binding (`g_limiting`) or the jaws stalled short of the goal for 50 ms | one tap (40 ms) on the hand whose trigger drives that gripper | on, 0.5 |
| limit | the WBC's per-arm `<ee>_limited_by` (reach, manipulability, joint_limit; `limit_kinds`) while armed and that hand engaged; an older WBC: its global `limited_by` on every engaged hand | continuous, held 150 ms past the last limited row | off, 0.15 |
| grip | in contact | continuous, amplitude x max(0.3, `g_force / g_force_max`) | off, 0.3 |

Hands follow the pairing in force (mirror: the left hand drives the right arm, and feels its limit and its
gripper). A pulse cue does not repeat one of the same or higher priority (disabled > disarmed / held > armed /
resumed) on that hand within 200 ms: X (disarm + disable) is one long. `held`, `resumed`, `resume_refused` and
`escalated` are feat/teleop-resume's kinds; this node maps them already. The contact and grip cues read each arm
node's stream (30 Hz, only while one of them is on).

**Configuration.** The page's Haptics card: the master switch, each cue on / off and its amplitude, a Test button
per cue (plays it even when off; a level cue as a 1 s pulse). `POST /haptics/config {"enabled", "cues": {name:
{"enabled", "amplitude"}}, "limit_kinds"}` (422 on an unknown cue or kind); `POST /haptics/test {"cue", "hand":
"left|right|both"}`. Not persisted: a restart is back to the defaults. `/state` `haptics` = `GET /haptics/info`
(config, levels and their cue, the headset's block, `readers`, `ack_ms`, the arm streams, contact per arm, counts).

## API

| | |
|---|---|
| `GET /state` | `state` (client state), `error`, `teleop` (with `armed`, `arm_seq`, `disarm_reason`, `disabled_by_x`, `tier`: `tier` armed / hold / disarmed, `why`, `hold` (`cause`, `class`, `since_s`, `cause_present_s`, `resumable`, `released`, `refusals`, `last_refused`), `resumes`, `config`; `input`: fresh, why, latency, rate), `headset` (with `battery`, `body`), `battery_pct`, `wbc` (echo of its `/state`, 1 Hz, with `armed`, `disarm_reason`, and stream age), `lease` (the WBC's), `estop` (the WBC's), `ctl` (the headset command: busy, rc, output lines), `config`, `latest` (row), `events` |
| `GET /events` | SSE of `/state` |
| `GET /fields`, `GET /stream` | rows every 50 ms: client state, sending, headset link (connected, rate, age, focus, XR state), per hand valid / engaged / gripper, engagements, targets sent, the WBC's state / base mode / limited / limited_by / row age, base pose, EE commanded / target / sent, base hint, head, stick offset; appended: `armed`, `wbc_armed`, `quest_latency`, `quest_rate_short`, `battery_pct` |
| `POST /start`, `POST /stop` | teleop (409 with the WBC's reason); Start is not live (Y arms) |
| `POST /resume {"enabled", "grace_s", "hold_max_s", "classes"}` | the tiered recovery (each field optional; 400 on an unknown class or a value out of range) |
| `POST /estop` | the arm, the swerve and the WBC in parallel; stops teleop |
| `POST /quest/up`, `POST /quest/down` | quest_ctl on the headset (409 while one runs) |
| `POST /mirror/{off\|auto\|straight\|mirror}` | two hands: the pairing mode (409 while a hand is engaged) |
| `POST /hint_source/{head\|chest\|hips}` | the base hint's source, from the next grip |
| `POST /elbow_hint/{on\|off}` | the elbow-swivel hint: on from each hand's next grip, off at once |
| `POST /tether/{on\|off}` | the stick tether, from the next grip |
| `POST /task {"text"}` | the task of the next episodes (in `/state.task` and each `episode_start`) |
| `GET /oplog/stream` | the operator stream (above) |
| `GET /eventlog?since_seq=N&limit=M` | JSON lines (`application/x-ndjson`) after N from the log files, at most M (default 1000): repeat from the last seq until fewer come back |
| `GET /eventlog/files` | `[{name, bytes, first_seq, last_seq}]` |
| `GET /haptics[?stream=1]` | the haptics state (`teleop.quest_haptics.v1`); `stream=1`: NDJSON lines, the headset's (above: "Haptics") |
| `GET /haptics/info`, `POST /haptics/config`, `POST /haptics/test` | the cues' config, levels, ack and counters; change them; play one |

`/state` also has `episode` (the open one: `{id, task, t_start, start_seq}`, else null; the contract), `episodes`
(state idle / resetting / recording, pending id, task, last outcome, counts, the last refusal), `task`,
`last_event`, `eventlog` (dir, seq, written_seq, queued, dropped, write_errors, last_error, bytes, cap_bytes,
pruned) and `oplog` (subscribers, records per type).

## Storage

Only the event log (above; events, never frames) under `--data-dir`, a deliberate small exception like
agent_interface's transcripts: no recording of its own (the client-side script records the streams, docs/nodes.md;
in a sim stack the node runs that script as a child process, `--record`), no state files; log lines go to stderr (the journal under systemd). Constructed without a `data_dir` (tests, library use) the
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
`tests/test_quest_haptics.py`: the cue table and its defaults, pulse ids and groups, each event kind's cue, the
dedupe (X = one long), the toggles and amplitudes, the limit level on the hand driving the limited arm (straight and
mirrored, engaged, armed, stale, the hold, the global fallback), contact and grip from the arm rows, lines published
on change only, the stream (pushed at once, keepalives, readers), the ack's round trip, the fake headset's player
(each pulse once, the pairs' timing, too-old pulses, a level's TTL), and end to end the node's stream played by the
fake headset (`--haptics URL`) with the ack back in its frames. `tests/test_quest_teleop_node_estop.py`: the parallel e-stop (a slow arm does not delay the swerve or the WBC; a dead
one does not stop the others), Start not live, X disabling without the watchdog stopping teleop, Y arming through
the node, armed / latency / battery in `/state` and the stream. `tests/test_quest_swivel.py`: the elbow-swivel hint
over the fake headset's scripted elbow (sign and magnitude, straight and mirrored, the vertical arm, a new seq per
engagement, omitted without a valid body, held while straight, toggle off = the old datagrams, the node's toggle
and state, end to end). `tests/test_quest_resume.py`: the tiered recovery (latch level on the fake rig, plus the
node's `held` / `resumed` / `escalated` events, `/state` `teleop.tier`, `set_resume`). The page is in
`tests/test_pages_contract.py` and the layout contract of `tests/test_visual_safety.py`.

## Code

`client.py` is the adapter: it imports the client from `tools/quest_teleop` (as the top-level package
`quest_teleop`) and sends to the unix socket (and reports each datagram to the operator stream). The client core
gained observer hooks for the node (`TeleopClient.on_frame` / `on_step` / `on_event`), the latch an `inhibit` (no Y
press arms while set: the sim reset) and `episodes.py` (the A / B press rules, pure). In the node: `eventlog.py`
(the event log), `oplog.py` (the operator stream), `episodes.py` (the episode states and the sim reset). Moving the client core into the package is planned once the velocity
feedforward work on `body_frozen.py` / `wbc_client.py` has landed.
