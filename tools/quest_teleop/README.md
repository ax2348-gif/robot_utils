# Quest whole-body teleop client

Drives the WBC node (`nodes/wbc`, design and frozen contract in `docs/plans/wbc_v1.md`, "Interfaces for the first
build", items 2–5) from the Quest headset's `teleop.quest_state.v1` stream. Runs on the Mac or erid, **never the Pi**.

```bash
# the robot, on its LAN (the default URL); the headset at its usual address
python tools/quest_teleop/teleop.py --quest 100.72.77.83 --wbc http://botcore-desktop.local/nodes/wbc
# through a proxy that does not carry UDP: name the robot's address for the datagrams
python tools/quest_teleop/teleop.py --wbc <erid proxy URL>/nodes/wbc --udp-host botcore-desktop.local
# a dry run without the headset: the fake headset, against the local sim stack's WBC
python tools/quest_teleop/fake_headset.py --port 9391 --loop &
python tools/quest_teleop/teleop.py --quest 127.0.0.1 --wbc http://localhost/nodes/wbc
```

Options: `--hand left|right` (the hand that drives the arm, default right), `--ee` (default `arm`),
`--no-heading-hint`, `--pose-hint` (off by default), `--max-speed` / `--max-omega` (stick full scale, default
0.3 m/s and 0.8 rad/s), `--stale-ms` (default 200), `--no-gripper`, `--udp-host`. Ctrl-C stops sending, disables
and releases the lease. The status line shows the engagement state, whether datagrams are going out, the age of the
newest WBC row and the last reason an engagement ended or was refused.

On the headset run the app as usual (`qr`, or `qr -s --scheme=body-frozen` to see the scheme in the headset's own
demo too). The client computes the body-frozen scheme itself from the raw head and controller poses, so the
headset's scheme setting does not change what is sent.

## Controls

| Input | Effect |
| --- | --- |
| Grip (`--hand`) | Clutch: engage = the arm follows the controller (body-frozen); release = the arm holds |
| Trigger (`--hand`) | Gripper, 0 open – 1 closed, independent of the clutch |
| Left stick | Base forward/left; right stick X: base yaw (the intent's `base_velocity`, 18 % dead zone) |
| Head yaw (while engaged) | Base `heading` hint |
| Head displacement (while engaged, `--pose-hint`) | Soft base `pose` hint |
| A (both grips released) | Headset realignment: a new operator-frame revision; the next engagement needs a release first |

## The body-frozen scheme (quest_env DEMO.md, "Experimental walking scheme")

On each engagement of the arm's hand, seen from its start:

- **Operator frame** from the raw head pose of that frame: origin = head position, X = the gravity-aligned head yaw
  (the head's LOCAL −Z mapped to operator FLU by `C = [[0,0,-1],[-1,0,0],[0,1,0]]`, projected on the floor), Z up.
  Refused when looking almost straight up or down (horizontal part < 0.15, as native). The controller's anchor
  `(p0, R0)` and every later pose are expressed in it; `dp = p − p0`, `dR = R R0ᵀ` (no 0.45 m cap).
- **Robot frame** from the newest WBC stream row (younger than 0.3 s, else the engagement is refused): the upright
  platform frame `F` (`base_x`, `base_y`, `base_yaw`, world) and the **measured** EE (`arm_meas_*`, world) in it.
- **Target** (world) `= F · (e0 + dp, dR · E0)`. At engagement it is exactly the measured EE (no jump). Platform
  motion does not move it; operator travel does; equal operator and platform travel leaves it fixed relative to the
  platform. `F`'s origin cancels (`target.p = ee.p + R_F dp`), so the platform frame and the shoulder frame of the
  DEMO give the same target (only the yaw matters).

Released or not yet engaged, the arm holds **in the base frame**: the last target (or the measured EE at start)
expressed in the platform frame at release, sent with `frame: "base"`, so a platform the stick moves carries the arm
(DEMO.md: release holds the arm joints).

## The remote-consumer rules (DEMO.md)

Per frame (`quest_stream.QuestGate`): schema and `control_intent` present; a new TCP connection, provider or intent
`session_id`, or `operator_frame_revision` is a **discontinuity**; `frame_seq` must increase within a connection
and session (older or repeated frames are dropped; native restarts it per session); an arrival gap > 200 ms is a
discontinuity; `input_focused` and `action_sync_result == 0`, else the frame is invalid. Per control step: no valid
frame for 200 ms on our clock = **stale**.

- Stale, unfocused, or the arm hand's intent `valid=false`: **stop sending** (no targets, no keep-alives). The WBC
  holds, then stops the base (its `hold_timeout`).
- Any discontinuity, invalid frame, staleness, an engagement id change we did not see released, an estimator reset
  (`est_reset` changes in the WBC stream) or a refused capture ends the engagement and **requires a release**: an
  engagement counts only if this client saw the hand released (valid, not engaged) since. So attaching or
  reconnecting mid-engagement waits for release and a new engagement; an existing delta is never applied to a new
  anchor. The anchor is captured on receipt (network delay included), as DEMO.md notes.
- Engagement itself (grip hysteresis 0.6/0.4, native release rules) comes from the intent's `engaged` /
  `engagement_id`; the client computes `dp`/`dR` from the raw poses.

## What is sent

HTTP at start: `POST /lease {client: "quest_teleop"}` (token in `X-Lease-Token`, renewed every heartbeat/3),
`POST /hints` (session defaults: base `locked`, `ees.arm.weights`), `POST /enable`, `POST /session`. At exit
`POST /disable`, `DELETE /lease`.

One datagram per batch of new headset frames (the stream rate), UDP to `--udp-host` (default: the `--wbc` host) at
the session's `udp_port`, `seq` strictly increasing per session, `t: null`:

| State | `ees.arm` | `base` |
| --- | --- | --- |
| released / waiting for a release | `frame: base`, the hold pose | `{mode: locked}` |
| engaged | `frame: world`, the body-frozen target | `{mode: hint, target: {heading[, pose]}}` (`free` without hints) |
| stick deflected (any state) | as above | `{mode: commanded, velocity: {vx, vy, omega, frame: base, ttl: 0.3}}` |
| input fine, no WBC row yet | — | `{"type": "alive"}` instead |
| stale / unfocused / hand invalid | nothing sent | |

`gripper` is in every target while the hand is valid. The `base` override goes in every datagram (the full state,
not a delta). Heading: `yaw = base_yaw0 + wrap(head_yaw − head_yaw0)` (world; weight 0.5, tol 0.05), referenced at
the engagement and re-referenced when the stick is released (so the hint does not undo what the stick did). Pose
(`--pose-hint`): the head's displacement in the engagement's operator frame, applied to the platform position at
engagement (weight 0.5, tol 0.10).

## Fake headset

`fake_headset.py` serves native-shaped frames (raw head and controller poses, buttons, tracking flags and a
`control_intent` from `NativeIntent`, a Python port of the native `teleop::Intent`: hysteresis, release rules,
body-frozen deltas, the stick mapping). Scenarios are keyframes in operator terms (`demo`: engage, reach, walk,
turn the head, stick, release, focus loss, session change; `idle`). `play(..., pause=, on_tick=)` simulates
network outages and dropped connections in tests.

## Tests

```bash
PYTHONPATH=$PWD/src python -m pytest tests/test_quest_teleop.py tests/test_quest_teleop_e2e.py
```

`test_quest_teleop.py` (pure): the C mapping, gravity-aligned yaw, zero jump at engagement, operator travel alone,
equal operator/platform travel, platform motion, rotations, agreement with the native body-frozen delta; the
consumer rules (stale, unfocused, session, revision, out of order, reconnect, start mid-engagement, robot state,
estimator reset); stick, heading and pose hints, gripper. `test_quest_teleop_e2e.py`: the client against the fake
headset and a fake WBC peer (`tests/quest_teleop_fakes.py`: HTTP, a rows stream, UDP), checking the control-plane
calls and the datagrams through engagement, stick, release, a network outage and a reconnect (about 5 s).
