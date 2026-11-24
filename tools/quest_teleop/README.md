# Quest whole-body teleop client

Drives the WBC node (`nodes/wbc`, design and frozen contract in `docs/plans/wbc_v1.md`, "Interfaces for the first
build", items 2–5) from the Quest headset's `teleop.quest_state.v1` stream. The CLI runs on the Mac or erid; on the machine that runs the WBC (the robot, or the sim stack) the same client runs as the `quest_teleop` node, with a page (docs/quest_teleop_node.md).

```bash
tools/quest_teleop/quest_up            # headset side: wake check, update if behind, telemetry headless, stream up
# the robot, on its LAN (the default URL); the headset at its usual address
python tools/quest_teleop/teleop.py --quest 100.72.77.83 --wbc http://botcore-desktop.local/nodes/wbc
# through a proxy that does not carry UDP: name the robot's address for the datagrams
python tools/quest_teleop/teleop.py --wbc <erid proxy URL>/nodes/wbc --udp-host botcore-desktop.local
# a dry run without the headset: the fake headset, against the local sim stack's WBC
python tools/quest_teleop/fake_headset.py --port 9391 --loop &
python tools/quest_teleop/teleop.py --quest 127.0.0.1 --wbc http://localhost/nodes/wbc
tools/quest_teleop/quest_down          # stop the headset's telemetry
```

`teleop.py` options: `--hand left|right` (the hand that drives the arm, default right), `--ee` (default `arm`),
`--no-head-walk` (engaged: the base hint follows only the stick offset), `--stale-ms` (default 200),
`--max-latency-ms` (default 150), `--min-rate` (default 30 Hz), `--no-gripper`, `--udp-host`, `--max-speed` /
`--max-omega` (the top speed: the sticks' full deflection and the hint clamp, asserted as the WBC's caps for the
lease and restored at the end; docs/quest_teleop_node.md). Ctrl-C stops sending, disables, restores the WBC caps
and releases the lease. The status line shows ARMED / disarmed, the engagement state, whether
datagrams are going out, the headset latency and rate, the age of the newest WBC row, the stick offset
`(x, y, yaw)` and the base hint sent `(x, y, heading)`, and the last reason (a disarm, or an engagement ended or
refused).

## Arming: Y and X (the left controller)

**Start is not live.** `teleop.py` (and the node's Start) only leases, enables and opens the session: nothing
moves until **Y** arms. Until then, and whenever disarmed, the client sends keep-alives with `armed: false`; the
WBC holds (docs/wbc.md "The armed latch").

- **Y arms** (`hands.left.buttons.secondary`), and enables first when the WBC is disabled (`POST /enable`, then arms
  if Y is still held), only on a deliberate press: Y seen released in a valid frame since the last disarm, disable
  or discontinuity, then held for **3 consecutive fresh valid frames** with X up, both grips < 0.4, both sticks
  inside the native dead zone (0.18), and a fresh WBC row (< 0.3 s) showing it enabled (or our enable just
  succeeded) without a fault. Anything failing during the press spoils it: release Y and press again. **One arm per
  press**: `arm_seq` += 1 (numbered above the WBC's floor from `POST /session`); the arming datagram is a target.
- **X disables** (`hands.left.buttons.primary`), LEVEL-triggered: any accepted frame with X down (focused or not)
  sends `armed: false` at once and `POST /disable` (again every 1 s while X is held and the WBC still shows
  enabled). The arm then floats with gravity compensation, the gripper goes into damping, the base coasts (the
  owner's decision). The session stays open, disabled; Y re-enables and arms.
- **Disarm** (`armed: false` at once, and in every datagram after): X; any discontinuity (reconnect, session or
  revision change, gap); an invalid frame (unfocused, ...); stale input (no valid frame for 200 ms); **late or slow
  input** (below); the **left controller inactive** (X / Y gone); nothing to send (the arm hand invalid); the WBC's
  rows stale (the newest older than 0.3 s, `LatchConfig.wbc_stale_s`); the WBC's own row (once it processed our
  arming datagram) showing it disarmed (an overrun, an estimator dropout, a fault, ...) or not enabled. The WBC also
  refuses an arming target more than 2 cm / 5° from its commanded pose (a stale riding pose). If
  the link is gone the WBC's `hold_timeout` (0.25 s) disarms anyway.
- **Nothing re-arms on its own**: recovery from stale, late, unfocused, a reconnect or a new session, a lease renew,
  an estimator reset: all need a new Y press.
- **After arming** the grip needs its release first (as always: an engagement counts only from a release) and the
  sticks give no base velocity until both are seen neutral. **The gripper** (the owner's rebot_ctl behaviour): no
  gripper command before arming (after Start, after any disarm or disable: the gripper stays where it is); from the
  arming frame on, `1 - trigger` in every target (a released trigger opens it at once). A disarm drops the engagement
  and the riding pose: the next arm captures the current command (no jump back to a stale pose).

**Headset latency detection** (`QuestGate`): per frame, latency = arrival (our clock) − `sample_time_ns` (the
headset's), over the running minimum of that difference in the session (the clock offset cancels). A frame more
than 150 ms over that baseline, or fewer than 30 frames/s over the last 0.5 s, makes the input **stale**
(disarm). Nothing is fresh before a full 0.5 s rate window has been judged after each (re)connect (no lenient
start-up: a 10 Hz link can never arm); after a bad frame, input counts as fresh again only after 0.5 s of good frames,
and that never re-arms. Known limits (a later item: a windowed minimum or a real clock-offset estimate): the baseline
(the running minimum of arrival − sample time since the connect) cannot see a latency that is constant from the
connect, and it drifts with the two clocks' relative drift; for the first tests the headset runs on the direct LAN.

### Stop paths

| Operator / event | Client | WBC | Arm | Base | Gripper |
| --- | --- | --- | --- | --- | --- |
| X | `armed: false`, `POST /disable` | disarmed, disabled | floats (gravity compensation) | coasts | damping |
| Stale / late / slow / unfocused input, a discontinuity, left controller off | `armed: false` (or silence) | disarmed (or `hold_timeout`) | held at its commanded joints | brakes to zero within its rate limit, held | keeps its goal |
| The WBC disarms itself (overrun, estimator reset, arm pause, fault, `POST /disarm`) | disarms when its row shows it | disarmed (fault: latched) | held | brakes to zero (a lost base: coasts) | keeps its goal |
| E-STOP (node page) | teleop stops | e-stopped, disabled | its own e-stop: latched, off | its own e-stop: latched, coasts (no braking command) | off |

**The rule:** the robot never moves when the clutch is released and re-engaged, however the operator moved in
between. Every anchor is the WBC's *commanded* state, and every head or stick contribution restarts at zero on each
engagement (tested directly: `test_release_and_reengage_never_moves_the_robot`).

## Controls

All of this only while armed (Y, above).

| Input | Released (no engagement) | Engaged |
| --- | --- | --- |
| Y (left) | arm (enable first if disabled) | — (already armed) |
| X (left) | disarm and disable | the same |
| Grip (`--hand`) | engage: the arm follows the controller (body-frozen) | release: the arm rides with the base |
| Trigger (`--hand`) | gripper 0 open – 1 closed | the same (independent of the clutch) |
| Left stick | base forward/left, directly (`commanded`) | adds to the base hint's offset (robot frame) |
| Right stick X | base yaw, directly | adds to the hint's heading offset |
| Head walk / turn | nothing | moves the base hint (the robot mirrors the operator) |
| A (both grips released) | headset realignment: a new revision; the next engagement needs a release first | |
| A (right), the teleop node | start an episode (armed only; in the sim stack: the sim reset first, leaving it disarmed) | the same |
| B tap (right, < 2 s), the teleop node | stop the episode (kept); held 2 s: the headset stops telemetry, the episode is aborted | the same |

Stick scaling: the stream's `control_intent.base_velocity` (native: 18 % dead zone, 0.45 m/s and 0.8 rad/s full
scale, per axis) is rescaled here to `Config.stick_max_speed` / `stick_max_omega` (0.8 m/s, 1.0 rad/s), the left
stick's translation clamped to a circle. The native constants are duplicated in `body_frozen.py`
(`NATIVE_STICK_*`) until a joint refactor with quest_env moves the scaling to one place. The WBC still clamps to
its own `max_speed` / `max_omega`. Stick clicks are not mapped (left click was the Quest app's automove; the WBC replaces it).

## The body-frozen scheme (quest_env DEMO.md, "Experimental walking scheme")

On each engagement of the arm's hand, seen from its start:

- **Operator frame** from the raw head pose of that frame: origin = head position, X = the gravity-aligned head yaw
  (the head's LOCAL −Z mapped to operator FLU by `C = [[0,0,-1],[-1,0,0],[0,1,0]]`, projected on the floor), Z up.
  Refused when looking almost straight up or down (horizontal part < 0.15, as native). The controller's anchor
  `(p0, R0)` and every later pose are expressed in it; `dp = p − p0`, `dR = R R0ᵀ` (no 0.45 m cap).
- **Robot frame** from the newest WBC stream row (younger than 0.3 s, else the engagement is refused): the upright
  platform frame `F` (`base_x`, `base_y`, `base_yaw`, world) and the **commanded** EE (`arm_cmd_*`, world) in it.
  DEMO.md says measured because a remote consumer normally lacks the command; we have it, and the measured pose
  would move the arm by its sag/backlash error at engagement. (`arm_meas_*` only while the command is not finite.)
- **Target** (world) `= F · (e0 + dp, dR · E0)`: at engagement exactly the current command. Platform motion does not
  move it; operator travel does; equal operator and platform travel leaves it fixed relative to the platform.
  `F`'s origin cancels (`target.p = ee.p + R_F dp`), so the platform frame and DEMO's shoulder frame give the same
  target. That is the formula with head walking off (`--no-head-walk`).
- **Two links with head walking on** (the default). The headset is worn on the neck, so it is a torso proxy. The
  robot chain mirrors the operator's: world → base reference (link 1: the base at engagement + the head's motion +
  the stick offset, i.e. the base hint) → EE (link 2: `F⁻¹ · command` at engagement + the hand's delta **in the
  current head frame**: horizontal position + gravity-aligned yaw). Target = link 1 ∘ link 2.
  - **The invariant:** while the hand keeps its pose relative to the head, the EE target keeps its pose relative to
    the base reference, for any walking or turning.
  - **Only what the base can follow is absorbed.** The head frame keeps the head's height at engagement, so
    crouching (or rising) moves the hand, and so the arm, down (or up): a way to lower the arm.
  - The frozen-frame formula broke it on turns. The hand sweeps an arc about the head while the base turns about its
    own origin, so the EE drifted sideways by about `|Δyaw| · |ee_in_base − hand_in_head|` in the plane: about 0.15 m
    for a 30° turn with a 0.28 m lever.
  - Clutch invariance is unchanged: both links' deltas start at zero on engagement.
  - A head lost mid-engagement freezes both links: the hand is read in the last tracked head frame. When the head
    returns, both re-anchor where they are, so the robot does not move.
- **Sticks: third vs first person** (`--mode`, the node page's toggle; read at each grip). The grip registers the
  headset to the robot (you face where the robot faces). With `A` = that registration, `O(t)` your body pose (neck:
  x, y, yaw), `δ` the hand delta in your body frame and `E0` the EE in the base at grip:
  - **third person** (default; shared room): base `= T(s) · A · O(t) · R(σ)`, EE `= T(s) · A · O(t) · [δ ⊙ R(σ)E0]`.
    The stick translation `s` moves the robot in the world, and the stick yaw `σ` turns only the robot (and swings
    the arm with it). Walking and reaching stay room-aligned.
  - **first person** (egocentric): one anchor `A(t)`; a stick increment `Δ` (robot frame) updates it as
    `A ← (Base·Δ·Base⁻¹)·A`, i.e. `Base → Base·Δ`, a turn about the robot. Your forward stays the robot's forward
    for walking and reaching.
  - Without a stick yaw the two modes command exactly the same; they differ only after one.
  - **Registration (third person)** is persistent: made at the **first enable** (the Y press that arms, after Start:
    face where the robot faces), or by a **right stick click** (released: at once; engaged: in place, nothing moves).
    Later clutches keep it: the clutch puts the robot's offset from its registered pose into the stick slots
    (`s`, `σ`), so it moves nothing and the room directions stay. It is dropped on an estimator reset, a new XR session or an operator
    frame realign (not on gaps or reconnects), and the next enable or click registers again. A system recenter
    of the headset is not visible in the stream: click to re-register. A clutch without
    any registration (nothing armed: tools, tests) registers. First person ignores it: every clutch zeroes.
  - **Headless left stick** (third person, **on by default**; `--no-headless` or the page's toggle turns it off,
    applies at once): with a
    registration, the left stick moves the robot along **your facing** (`A`'s yaw + your yaw), not the robot's
    forward. Engaged it integrates into `s` as before; released it is the same world direction expressed in the base
    frame. The right stick's yaw is unchanged.

Released or not yet engaged, the arm rides with the platform. The WBC takes world targets only, so the client
computes the riding target: at release it stores the **commanded EE at release** relative to the base (`arm_cmd_*`
and the base pose of the same stream row; at start, the current command); every frame it sends the world target =
the current base pose ∘ that relative pose, with `twist` = the base's rigid-body velocity at the EE (from the
stream's commanded base twist `base_vx, base_vy` read as base-frame components and `base_w`), which cancels the
~20 ms lag.

**Velocity feedforward while engaged.** Without it the WBC tracks on the error alone and trails a walking operator
by `v / k` (measured in the QP at 0.5 m/s: the EE 142 mm behind, the base 250 mm). So the engaged EE target carries
`twist` = the target's own velocity (the hand's motion through the frozen frames, linear and angular), and the base
hint carries `vx, vy` (pose) and `omega` (heading) = the hint target's velocity (the head's motion mapped as the
hint's position is, plus the stick offset's rate, exact). Both are least-squares slopes over the frames of the last
`ff_window` = 60 ms on `sample_time_ns`: exact for a steady motion, about 30 ms (half the window) behind a change of
speed. Clamped: the EE 1 m/s and 3 rad/s, the hint to the stick maxima (`hint_max_speed` / `hint_max_omega`, 0.8 m/s, 1.0 rad/s; the WBC clamps further to its own caps). Zero
while the head is lost (its history restarts). **Every engagement starts them at zero**: only this engagement's
frames count, and they ramp in as the window fills, so re-engaging never moves the robot. With it: the EE about 1 mm
behind, the base about 18 mm (the same QP test). `Config(feedforward=False)` turns it off.

## The base

- **Released:** no hint. Sticks deflected: `{mode: commanded, velocity: {vx, vy, omega, frame: base, ttl: 0.2}}`;
  idle: `{mode: locked}` (the base decelerates to a stop and holds).
- **Engaged:** `{mode: hint, target: {pose: {x, y, vx, vy, frame: world, weight 0.5, tol 0.10}, heading: {yaw,
  omega, frame: world, weight 0.5, tol 0.05}}}` (`vx, vy, omega`: the feedforward above) = the base pose at engagement + the head's motion since engagement (displacement in
  the engagement's operator frame mapped through the platform yaw at engagement; gravity-aligned yaw change) + the
  stick **offset**. The offset starts at zero on every engagement and integrates the sticks per frame: x/y along the
  robot base frame at integration time (stick forward = robot forward), yaw from the right stick. Stand still and
  drive with the sticks, walk and let the robot mirror, or both. Whatever the sticks achieved is in the base pose the
  next engagement captures, so nothing carries over.

## The remote-consumer rules (DEMO.md)

Per frame (`quest_stream.QuestGate`): schema and `control_intent` present; a new TCP connection, provider or intent
`session_id`, or `operator_frame_revision` is a **discontinuity**; `frame_seq` must increase within a connection
and session (older or repeated frames are dropped; native restarts it per session); an arrival gap > 200 ms is a
discontinuity; `input_focused` and `action_sync_result == 0`, else the frame is invalid. Per control step: no valid
frame for 200 ms on our clock = **stale**.

- Stale (also late or slow, "Arming"), unfocused, or the arm hand's intent `valid=false`: **disarm** (one
  `armed: false`), then stop sending. The WBC holds; nothing resumes without a new Y press.
- Any discontinuity, invalid frame, staleness, an engagement id change we did not see released, an estimator reset
  (`est_reset` changes in the WBC stream) or a refused capture ends the engagement and **requires a release**: an
  engagement counts only if this client saw the hand released (valid, not engaged) since. So attaching or
  reconnecting mid-engagement waits for release and a new engagement; an existing delta is never applied to a new
  anchor. The anchor is captured on receipt (network delay included), as DEMO.md notes.
- Engagement itself (grip hysteresis 0.6/0.4, native release rules) comes from the intent's `engaged` /
  `engagement_id`; the client computes `dp`/`dR` from the raw poses, so the headset's scheme setting does not
  change what is sent.

## What is sent

HTTP at start: `POST /lease {client: "quest_teleop"}` (token in `X-Lease-Token`, renewed every heartbeat/3),
`POST /hints` (session defaults: base `locked`, `ees.arm.weights`), `POST /enable`, `POST /session {}`. At exit
`POST /disable`, `DELETE /lease`.

One datagram per batch of new headset frames (the stream rate), UDP to `--udp-host` (default: the `--wbc` host) at
the session's `udp_port`, `seq` strictly increasing per session, `t: null`, `est_reset` = the stream epoch the
target was computed against (an engagement's own for world targets):

| State | `ees.arm` | `base` |
| --- | --- | --- |
| disarmed (input fresh) | — | `{"type": "alive", "armed": false}` instead |
| armed, released / waiting for a release | `frame: world`, base pose ∘ the command at release, + `twist` | stick velocity (`commanded`) or `locked` |
| armed, engaged | `frame: world`, the body-frozen target | `hint`: pose + heading |
| armed, input fine, no WBC row yet | — | `{"type": "alive"}` instead |
| stale / unfocused / hand invalid | one `armed: false`, then nothing | |

Every datagram carries `armed` and `arm_seq`. All targets are `frame: "world"`. `gripper` is in every target while
the hand is valid and the client armed (from the arming datagram on). The full `base` state goes in every datagram.

## Headset control: `quest_up`, `quest_down`, `quest_status`

`quest_ctl.py` (stdlib only; the three shims call it) runs quest_env's own script on the headset over SSH (`Host
quest` in `~/.ssh/config`, the quest-dev-runtime SSH server): `sh "$QUEST_ENV_ROOT/quest" …`, the root read from
`~/.config/quest-env/root` as `profile.sh` does (`q` itself exists only in interactive shells). It automates only the
headset side.

- `quest_up [--scheme body-frozen|standard] [--demo] [--ar [URL|:PORT]] [--restart] [--wait 30]`: SSH reachability (a clear message if
  the headset is asleep or offline); `git fetch` and compare with the upstream, `quest pull` only when behind and
  never over a dirty tree (reported, not forced); `quest run --no-viewer --scheme=…` (`-s` with `--demo`), skipped
  when the stream already answers in the wanted mode and nothing was pulled; waits for port 9391; prints session,
  rate, focus, scheme and battery. The scheme works without `-s`: the script writes `off body-frozen` to
  `~/.config/quest-env/demo-mode` and native reads the second word independently of the demo flag.
  `--ar` adds the headset's robot AR view (quest_env ROBOT_AR.md). Bare `--ar` is this machine's sim stack (started
  with `--host 0.0.0.0`, port 8080; `--ar :PORT` for another) at this machine's address on the route to the headset
  (its Tailscale address when the headset is reached over Tailscale); `--ar URL` or `--ar host[:port]` otherwise. A changed or dropped
  URL restarts telemetry (it is read once at native start).
- `quest_down`: `quest stop`. `quest_status`: `quest status`, the mode file, the XR session state (the telemetry
  process's last `OpenXR session state` line in logcat), stream liveness, battery.
- Live check (2026-11-21, headset awake but not worn): SSH, the root, fetch/behind/dirty, `quest run`, `quest stop`
  and `quest status` all work non-interactively (`ssh -T`). The telemetry process starts and logs "controller server
  listening on 0.0.0.0:9391", but its XR session stays IDLE and the port does not answer (connects time out, even
  on the headset itself) until the headset is worn; `quest_up` then fails with "XR session: IDLE (headset not
  worn?)". The battery is not readable from the runtime's sandbox (sysfs permission denied, no `battery` service):
  it prints "unknown".
- `--ssh-host`, `--stream-host` (default: `ssh -G`'s HostName for the ssh host), `--port`.

Nothing destructive is ever sent (read-only queries, `fetch`, `pull --ff-only` through the script, `run`, `stop`).

## Fake headset

`fake_headset.py` serves native-shaped frames (raw head and controller poses, buttons, tracking flags and a
`control_intent` from `NativeIntent`, a Python port of the native `teleop::Intent`: hysteresis, release rules,
body-frozen deltas, the stick mapping). Scenarios are keyframes in operator terms (`demo`: engage, reach, walk,
turn the head, stick, release, focus loss, session change; `idle`). `play(..., pause=, on_tick=)` simulates
network outages and dropped connections in tests.

## Tests

```bash
PYTHONPATH=$PWD/src python -m pytest tests/test_quest_teleop.py tests/test_quest_teleop_e2e.py tests/test_quest_ctl.py
```

`test_quest_teleop.py` (pure): the C mapping, gravity-aligned yaw, zero jump at engagement, operator travel alone,
equal operator/platform travel, platform motion, rotations, agreement with the native body-frozen delta; the
consumer rules (stale, unfocused, session, revision, out of order, reconnect, start mid-engagement, robot state,
estimator reset); the commanded anchor; released targets riding with a stick-driven base (pose and twist); released sticks, the engaged hint (head, sticks, both), the
offset restart, the release/re-engage rule across operator motions, `est_reset`, gripper.
`test_quest_teleop_e2e.py`: the client against the fake headset and a fake WBC peer (`tests/quest_teleop_fakes.py`:
HTTP, a rows stream, UDP) through engagement, stick, release, a network outage and a reconnect (about 5 s).
`test_quest_ctl.py`: `quest_up/down/status` against a fake SSH runner and the fake headset stream.
`test_quest_buttons.py` (Y / X, every client disarm trigger, no recovery re-arms, the gates after arming) and
`test_quest_latency.py` (late, slow, recovery), pure on `tests/quest_latch_fakes.py`; `test_quest_stick_offset.py`
(the target and the hint share this frame's stick offset).
