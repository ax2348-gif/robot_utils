# Agent MoMa node, v1: plan

Status (2026-11-11): steps 1–5 built on `claude/agent-node` (tests green in simulation, with a fake arm node for agent_interface); step 6, the supervised robot test, is next.

## Goal

One agent-facing node for mobile manipulation: navigation as agent_nav does it today, plus the arm
through the same node. Agents get one session, one transcript, one place for events. Humans keep
authority over every guardrail.

agent_nav is copied, not renamed. The copy is developed; agent_nav stays frozen as it is (so old
briefs, links and episode docs keep working) and is stopped by default.

## Decisions already made

- **Copy, don't rename.** `agent_nav` frozen: marked in its `/meta` and docs, not maintained, not
  started by default (startable from the landing page).
- **Low speed is solved by the drive gains** (kS 0.10 V, I 0.0002, `9b2dd7d`): no turn-rate warnings
  or stall-message work.
- **Events piggyback on agent-facing replies only** (`since_last_call`). The arm node, swerve, and
  pose_servo replies stay as they are: their pages and scripts aren't agents.
- **Neither arm jams nor base anomalies are emergencies.** They are reported (and a jam pulls the arm
  back); stopping everything stays the human's e-stop.
- **Jam detector:** highly configurable, refires by default, response during a planned motion = the
  setpoint pulled back to the current reading, report-only during `track`. The human can disable it;
  agents can't.
- **Defensive wheel alignment:** on by default, chosen per call by the agent, with a human "force"
  toggle that overrides. Mostly vertical arm moves are ignored (horizontal deadzone), small direction
  changes too (hysteresis), and the axis has no sign.
- **The same human-force / agent-option pattern for the stop reflex**: when the human doesn't force
  it, agents may still turn it on for a goal, purely as a utility.
- **Work happens in a worktree** (`claude/agent-node`).

## Parts

### 1. agent node (copy of agent_nav)

- **Copy and rename:** package `robot_utils.nodes.agent_interface` (it is the agents' interface, not an agent),
  service unit `robot-node-agent_interface`, proxy path `/nodes/agent_interface/`, data directory
  `~/.local/share/robot_utils/agent_interface`, own tests, title **"Agent MoMa"**
  (MoMa = mobile manipulation; fits a landing-page tile on one row). The guide becomes the MoMa guide.
- **One enable for the whole robot.** `POST /enable` enables the base (pose_servo lease, caps) and the
  arm (the node takes the arm lease and enables it: the arm floats under gravity compensation until a
  servo engages it); `{"arm": false}` / `{"base": false}` for a session that needs only one. `/disable`, the
  human's hold and the e-stop latch cover both. The arm's first enable after power-up runs the gripper
  check (it closes the gripper): said in the guide.
- **Arm interface, forwarded to the arm node.** The node holds the arm lease for the session and
  renews it itself (agents no longer leave background lease loops running, as Codex did). v1 forwards:
  - motion: `POST /arm/enable`, `/arm/disable`, `/arm/servo` (pose or named pose), `/arm/home`,
    `/arm/gripper`, `/arm/cancel`;
  - read-only: `POST /arm/ik`, `GET`/`POST /arm/geometry`, `GET /arm/state`.

  Every call is a transcript turn with the agent's comment. Replies are the arm node's, unchanged,
  plus `since_last_call`. Not forwarded in v1: `track`, `chunk`, profiles, TCP offset, limits (the
  agent works through servo; discouraged per the operator).
- **Defensive wheel alignment** on `POST /arm/servo` (option `align_wheels`, default true):
  1. From the measured TCP and the target, the horizontal direction of travel in base axes.
  2. If the horizontal travel is under the deadzone (default 0.02 m) or within the hysteresis angle
     (default 15°) of the current alignment (axis, no sign): no change.
  3. Otherwise ask pose_servo to align the wheels for the unit twist of that horizontal direction (a
     translation; the swerve takes any unit twist) with the drives coasting, wait until aligned (timeout ~1 s),
     then forward the servo.
  4. If the base was left out of the enable (`"base": false`), alignment is skipped and the reply says so.

  The human toggle "force defensive alignment" (human-only, as `/reflex` today) makes it apply even when
  an agent passes `align_wheels: false`. The reply says what was done.
- **Reflex: force / opt-in.** The human-only setting becomes `force` (default on: every goal has the
  reflex, agents can't turn it off). With `force` off, a goal's `reflex: true` turns it on for that goal;
  `reflex_margin_m` works as now.
- **`since_last_call`** on every agent-facing reply: events since that caller's previous call, oldest
  first, each with time, source and text:
  - arm jams (read from the arm node's `/state` events);
  - base movement anomalies (e.g. the arm pushing the coasting base): the pose departing from where an
    idle stretch began by more than 0.01 m or 1° (heading threshold to be checked against the
    estimator's stationary drift in the recordings). Checked only while: no goal runs and pose_servo is not
    moving, braking or holding; ≥ 1 s after the last goal end or stop (braking settles); and the swerve lease
    is pose_servo's or nobody's (a teleop page driving the base is not an anomaly). Each idle stretch has its
    own reference pose, so drift can't build up across goals;
  - its own: reflex stops, stalls, goals ending.

  The same events are robot turns in the transcript.
- **Guide:** the arm through this node; `since_last_call` (read it before deciding the next step);
  reflex and alignment options; jams and what the agent should do after one (back off along the
  opposite direction; don't retry into it).

### 2. arm node

- **Jam detector** (v1: joint space, from the offline study):
  - **only on a static arm** (it detects the aftermath, not the instant of contact; dynamic cases are out of
    scope): no planned motion executing (a finished move counts, `not_reached` settles included), measured
    joint speeds near zero, and a short settle time after the last motion. All four labelled jams (J1–J4)
    were static holds after a move ended;
  - signal: residual τ − τ_g per joint, held ≥ `hold_s` (1 s), hysteresis (release at 0.7×);
  - bands per joint (study's draft: 1.0, 4.0, 4.0, 1.5, 0.8, 0.6 Nm), to be confirmed when the study is
    finished. Fixed bands, no payload re-baselining: agents aren't expected to grasp anything heavy enough
    to read as a jam (said in the guide);
  - response, configurable: `hold` (default: setpoint pulled to the current reading during
    servo / chunk / home; the motion ends `jammed` with joint, residual and duration), `report`, `off`;
    report-only during `track`;
  - refires by default (`refire: true`);
  - configuration: `GET`/`POST /jam` (bands, hold time, hysteresis, response, refire). Changing it is
    human-only (`X-Operator: human`); agents can read it. Also on the arm page.
  - events in `/state` (time, joint, residual, duration, count), and a `jam` stream column.
- The TCP-frame wrench version (horizontal / upward / vertical-axis torque, payload-bounded downward)
  comes in a later version, once the study compares it.

### 3. pose_servo and swerve

- **swerve: steer-only mode.** `POST /command {"steer": [vx, vy, omega]}`: a unit twist (only its direction
  matters); each wheel is set the way it would roll for it, with the drive motors left out of the heartbeat
  (disabled, so they coast in coast idle mode). Same lease rules and 0.25 s hold as a twist.
- **pose_servo: `POST /align {twist}`** (holder only, only when no goal is running): steer-only for that unit
  twist until the next goal or `stop`; reports when aligned. A goal ends the alignment. Off when pose_servo
  isn't enabled.

## Open decisions

Settled: title "Agent MoMa"; one enable for base and arm (alignment always available while enabled);
anomaly threshold 1 cm, guarded so it can't fire while the base is driven.

None left: payload = fixed bands (no heavy grasps); jams only on a static arm; the API audit landed first
(`5652f75`).

## Order of work (proposed)

1. Land the agent_nav API audit on main (already tested); review the arm and description parts.
2. Copy agent_nav → agent (rename only, tests green); freeze agent_nav.
3. swerve steer-only + pose_servo `/align` (teleop-testable: wheels align, drives coast by hand).
4. arm jam detector (offline-validated thresholds, configurable, human-only config).
5. agent node: arm forwarding + lease keeper, defensive alignment, reflex force / opt-in,
   `since_last_call` with arm and base events, guide.
6. Supervised test on the robot: alignment and coasting by hand, a deliberate slow arm press against a
   soft obstacle for the jam response, then a short agent session.
