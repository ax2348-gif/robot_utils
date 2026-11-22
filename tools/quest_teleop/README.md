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
`--no-gripper`, `--udp-host`. Ctrl-C stops sending, disables and releases the lease. The status line shows the
engagement state, whether datagrams are going out, the age of the newest WBC row, the stick offset `(x, y, yaw)`
and the base hint sent `(x, y, heading)`, and the last reason an engagement ended or was refused.

**The rule:** the robot never moves when the clutch is released and re-engaged, however the operator moved in
between. Every anchor is the WBC's *commanded* state, and every head or stick contribution restarts at zero on each
engagement (tested directly: `test_release_and_reengage_never_moves_the_robot`).

## Controls

| Input | Released (no engagement) | Engaged |
| --- | --- | --- |
| Grip (`--hand`) | engage: the arm follows the controller (body-frozen) | release: the arm rides with the base |
| Trigger (`--hand`) | gripper 0 open – 1 closed | the same (independent of the clutch) |
| Left stick | base forward/left, directly (`commanded`) | adds to the base hint's offset (robot frame) |
| Right stick X | base yaw, directly | adds to the hint's heading offset |
| Head walk / turn | nothing | moves the base hint (the robot mirrors the operator) |
| A (both grips released) | headset realignment: a new revision; the next engagement needs a release first | |

Stick scaling is the stream's own (`control_intent.base_velocity`: 18 % dead zone, 0.45 m/s and 0.8 rad/s full
scale). Stick clicks are not mapped (left click was the Quest app's automove; the WBC replaces it).

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
  target.

Released or not yet engaged, the arm rides with the platform. The WBC takes world targets only, so the client
computes the riding target: at release it stores the **commanded EE at release** relative to the base (`arm_cmd_*`
and the base pose of the same stream row; at start, the current command); every frame it sends the world target =
the current base pose ∘ that relative pose, with `twist` = the base's rigid-body velocity at the EE (from the
stream's commanded base twist `base_vx, base_vy` read as base-frame components and `base_w`), which cancels the
~20 ms lag.

## The base

- **Released:** no hint. Sticks deflected: `{mode: commanded, velocity: {vx, vy, omega, frame: base, ttl: 0.2}}`;
  idle: `{mode: locked}` (the base decelerates to a stop and holds).
- **Engaged:** `{mode: hint, target: {pose: {x, y, frame: world, weight 0.5, tol 0.10}, heading: {yaw, frame:
  world, weight 0.5, tol 0.05}}}` = the base pose at engagement + the head's motion since engagement (displacement in
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

- Stale, unfocused, or the arm hand's intent `valid=false`: **stop sending**. The WBC holds the last world pose;
  the stick velocity's ttl and the WBC's staleness stop the base.
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
| released / waiting for a release | `frame: world`, base pose ∘ the command at release, + `twist` | stick velocity (`commanded`) or `locked` |
| engaged | `frame: world`, the body-frozen target | `hint`: pose + heading |
| input fine, no WBC row yet | — | `{"type": "alive"}` instead |
| stale / unfocused / hand invalid | nothing sent | |

All targets are `frame: "world"`. `gripper` is in every target while the hand is valid. The full `base` state goes
in every datagram.

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
