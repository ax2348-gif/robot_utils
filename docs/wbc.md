# Whole-body control (WBC) node

`python -m robot_utils.nodes.wbc` moves the arm and the base together so the end effector (EE) follows targets
in the world (estimator) frame. Design: [plans/wbc_v1.md](plans/wbc_v1.md). One end effector for now, `arm`
(names come from the model; a second arm adds a key).

```bash
python -m robot_utils.nodes.wbc                                   # model from the local description node
python -m robot_utils.nodes.wbc --model local --udp-host 192.168.8.50 --config my_wbc.yaml
python -m robot_utils.nodes.wbc --swerve none --estimator none   # arm only (base locked, world = base frame)
```

Options: `--model` (description instance / URL, `local`, or a `.urdf` with `base=planar`), `--arm`, `--swerve`,
`--estimator` (instance names or URLs; `none` = off), `--udp-host` / `--udp-port` (default `127.0.0.1:9410`; on
the robot set the LAN address, never `0.0.0.0`), `--config` (YAML of `spec.Config` keys), `--switch-interval`
(Python's GIL switch interval for the process, default 0.0005 s, env `WBC_SWITCH_INTERVAL`; Python's own is
0.005 s: the SCHED_FIFO control thread shares the GIL with normal-priority HTTP / SSE / stream-reader threads, and a
descheduled thread holding the GIL blocks it — measured on the robot as `control tick overrun: 35 ms` and
`estimator stream stale` disarms), `--name`, `--port`.
Registered in `frontend/nodes.yaml` as `wbc`; page at the node root (state, base mode, EE commanded / measured /
target, tracking error, base twist, solve time, events).

### Defaults (for the first tests, the owner's)

Base: `max_speed` 0.6 m/s (body), `max_omega` 0.8 rad/s, `max_accel` 1.0 m/s², `max_alpha` 1.5 rad/s², every
wheel `wheel_speed` 1.0 m/s. Arm: `joint_vel` / `joint_acc` default to **the arm's own** (null in the config): the
arm's joint-channel welcome `limits` (2, 2, 2, 6, 6, 6 rad/s; 10, 10, 10, 30, 30, 30 rad/s²; until a welcome, the
arm config's soft defaults, the same numbers); a tuple in the config narrows them, never widens. Ceilings: the base
speed and yaw rate are checked at `POST /enable` against the swerve's own `GET /limits` "hard" (beyond them: 422,
both leases given back); the wheel speed has no swerve endpoint and is bounded by pose_servo's
`planner.WHEEL_SPEED_MAX` (3 m/s, the steering model's limit) when the config is made. `resync` is 0.25 rad, at
most 0.7 of the arm's lead bound (20°): a last-resort net behind the **lead bound** (below): `lead_torque`
6, 6, 6, 1.5, 1.5, 1.5 Nm, `lead_delay` 0.08 s.

### Lead bound (contact force)

The arm node servos each joint with the soft MIT law `tau = kp (q_cmd − q) + kd (qd_ff − qd) + gravity`
(kp 72, 72, 96, 18, 18, 18 Nm/rad, `rebotarm_dm_soft.yaml`). The WBC integrates `q_cmd` from its previous command,
so in sustained contact the command kept following the operator while the real arm was blocked: the push grew to
kp × `resync` (17.6 / 17.6 / 23.4 / 4.4 Nm) before a resync, then again (a sawtooth). The lead bound is a
**torque budget** per joint: `lead_i = lead_torque_i / kp_i` (at most the arm's `max_lead`), and every tick, hard
in the QP, each arm joint's next command stays within `lead_i` of `q_ref`:

`|q_cmd + q' ds − q_ref| ≤ lead`, `q_ref` = the measured joints extrapolated by their measured velocity (capped at
the joint speed) over the state's age + `lead_delay`. The window is the **union** of the windows around the
measured and around the extrapolated joints, so the extrapolation only ever widens it on the side the arm moves to:
a `lead_delay` too long never drags the command along (a stopping command is always allowed); too short throttles
fast motion (the arm then trails beyond the lead: `limited_by` lead in free motion). In contact the measured
velocity is 0 and the window is exactly `lead` around the blocked joints: the servo's push `kp (q_cmd − q)` stays
within `lead_torque` and the command slides along with the target only inside it (no resync, no sawtooth).

- Outside the window (a sudden blockage while moving: the command led by the lag): the command returns toward it no
  faster than it can stop at the edge (the discrete braking speed), within the joint box (the joint box wins:
  always feasible). The transient is not bounded by the budget: the command's own acceleration limit lets it run
  on by up to `v² / 2a` (0.2 rad at 2 rad/s on joints 1–3) before it comes back; at the speeds of contact work it
  is small (tests: +0.06 Nm over the budget approaching a still edge).
- `kp` comes from the arm's `GET /gains` (the profile in force) at enable; until then, and if it cannot be read
  (an event), from the arm's default profile file (`soft`), else a copy of its kp. `max_lead` from the welcome.
- `POST /config` `lead_torque` (6 positive numbers, Nm) and `lead_delay` (s, ≥ 0) apply at once; `lead_torque`
  beyond kp × `max_lead` per joint is refused (422: the arm drops goals beyond `max_lead`; with the soft profile
  25.1, 25.1, 33.5, 6.3, 6.3, 6.3 Nm). A budget above kp × `resync` (0.244 rad) lets the resync fire before the
  bound in contact.
- `limited_by` `lead` (7) when it binds (and nothing of higher priority: reach, manipulability, joint limits);
  `/state.arm.lead`: `lead_torque`, `kp`, `kp_source`, `max_lead`, `lead_delay`, `bound` (rad), `dev` (the command's
  distance beyond the measured window this tick, rad), `torque` (kp × dev), `binding`, `binds` (ticks it bound,
  any joint, even when `limited_by` names a higher-priority limit).

**Defaults.** 6 Nm on joints 1–3 (a third of the old sawtooth peak; above the gravity-compensation and payload
errors a few Nm deep), 1.5 Nm on the wrist (joints 4–6, a third of its 4.4 Nm peak). Leads: 0.0833, 0.0833, 0.0625,
0.0833, 0.0833, 0.0833 rad (4.8°, 4.8°, 3.6°, 4.8° ...). The hand force they allow, from the hand stiffness
`(Jp K⁻¹ Jp^T)⁻¹` at the start pose (q 0, −0.6, −0.9, 0, 0.5, 0; TCP at 0.24, −0.16, 0.66 m): the force along an
axis at which the first joint reaches its budget, and the hand's deflection then:

| Axis (base) | Max hand force | Joint at its budget | Hand stiffness | Deflection |
| --- | --- | --- | --- | --- |
| x (forward) | 19.9 N | 2 | 640 N/m | 31 mm |
| y (left) | 49.4 N | 1 | 3950 N/m | 13 mm |
| z (up / down) | 14.5 N | 4 | 560 N/m | 26 mm |

(The ready posture: 17.8, 46.5, 14.8 N.) A static load beyond the budget (a heavy payload, a gravity-compensation
error) makes the arm sag rather than push harder: the budget bounds holding torque too.

`lead_delay` 0.08 s: the world sim's measured arm trails the command by ~5 ticks (servo + pipelined read, docs/arm.md)
+ the one tick the new command leads by + 2 ticks of margin. In the controller tests (a pure-delay arm, the joints
at their speed caps with 30 rad/s² reversals on the wrist) free motion is untouched up to a 6-tick lag; at 7 ticks
it throttles reversals (½ a h² approaches the wrist's 0.083 rad lead). **The real arm's delay was not measured**:
if fast teleop shows `limited_by` lead (and `binds` growing) without contact, raise `lead_delay` (safe: the window
only widens on the moving side) or the wrist budget.

### Three error numbers

| Stream column | Difference | Meaning |
| --- | --- | --- |
| `<ee>_err_pos` / `_err_rot` | target − **commanded** | the QP's shortfall: how far the command is from the target (limits, the lead bound, the gains). Not where the hand is |
| `<ee>_err_meas_pos` / `_err_meas_rot` | target − **measured** | the actual error: where the hand is vs where it is wanted |
| `<ee>_lead_pos` / `_lead_rot` | commanded − **measured** | the servo's deflection: effort / contact force (in free motion also the arm's lag) |

Position: the norm (m) of a world-axes difference; rotation: the angle (rad). NaN without a measured pose
(disabled). Per joint `<ee>_lead_tau1..6` = kp (q_cmd − q_meas) (Nm). The page plots the three for position and
orientation and shows each joint's lead torque against its budget.

## Using it

1. `POST /lease {"client": "me"}` → `{token, heartbeat, lease_id, client}`; the token goes in `X-Lease-Token`;
   `POST /lease/renew`, `DELETE /lease` (common/lease, as the arm and the swerve). Every accepted datagram also
   renews it.
2. `POST /enable`: takes the **arm and swerve leases together** (if one fails, the other is given back), sets the
   swerve's soft limits to `max_speed` / `max_omega`, enables both, opens the arm's **internal joint channel**
   (`<runtime>/robot/arm.joint.sock`, hello with the arm lease token) and seeds the commanded joints from the
   arm's measured ones. Refuses with the reason (409 / 503) otherwise. The WBC and pose_servo exclude each other
   through the swerve lease.
3. `POST /session {}` → `{session, udp_port, unix_socket, arm_seq}`. Send UDP **directly to the node's host** (not
   through the proxy) or to the Unix socket on the robot. `arm_seq` is the armed latch's floor (below): number
   arming presses above it.
4. Datagrams (JSON v0; `seq` strictly increasing; older / equal and unknown sessions are dropped and counted). Every
   datagram carries **`armed`** (bool) and **`arm_seq`** (int): the client's latch state, not events (a datagram
   without them counts as `armed: false`). **Nothing moves until the WBC is armed** (below):
   - reactive: `{"type": "target", "session", "seq", "t": null | <Pi monotonic s>, "est_reset": <optional>,
     "ees": {"arm": {"frame": "world", "pos": [x, y, z], "quat": [x, y, z, w], "twist": [vx, vy, vz, wx, wy,
     wz] (optional, world axes), "gripper": 0..1 (optional; 0 closed, 1 open)}}, "base": {<hint overrides>}
     (optional)}`; `t: null` = now; the newest wins; the twist is the feedforward while fresh.
   - chunk: `{"type": "chunk", "session", "seq", "est_reset"?, "ees": {"arm": {"frame": "world", "t": [...],
     "pos": [[...]], "quat": [[...]], "twist": [[...]]?, "gripper": [...]?}}}`.
   - keep-alive: `{"type": "alive", "session", "seq"}`.
   **World frame only** (the owner's decision, 2026-11-21): a `frame` other than `world` is dropped (`frame`).
   Clients do any base-relative math from the stream (`base_x/y/yaw` and the body twist `base_vx/vy/w`); a
   released teleop arm is sent as world targets that follow the base, with the base's velocity as feedforward.
   `est_reset`: the estimator reset id the client computed its targets against (the stream's `est_reset`); a
   different one is dropped (`est_reset`).
5. `POST /hints` (the hint structure below), `POST /disarm` (anyone, no lease: hold; arming again needs a new
   press), `POST /disable` (no further twist: the swerve is disabled and released and coasts; a last hold command to
   the arm, then its channel closed and its lease released: the arm node then floats it), `POST /estop` (anyone;
   below; the WBC latches until `POST /estop/clear`). `GET /fence` answers 501 (not built).

### The armed latch

`armed` lives inside enabled and is **false at enable**. Disarmed = **hold**: the target buffers and the datagram's
base override are cleared (never resumed), the base brakes to zero within its rate limit (the hold path), the arm is
held at its commanded joints (zero velocity, at once), the gripper keeps its last goal (no gripper value is sent).
Target and chunk datagrams are ignored while disarmed (counted in `/state.latch.ignored_disarmed`, never buffered);
the state is `idle` with `hold_reason` `disarmed: <reason>`.

- **Arms** only from a `target` or `chunk` datagram with `armed: true` and an `arm_seq` **greater than the floor**
  (the highest `arm_seq` seen when it was last disarmed; kept for the node's life, returned by `POST /session`),
  while enabled without a fault, an e-stop, an arm pause or an unusable estimator, and only if the datagram itself
  is valid (its target is applied) and its target lies within `arm_target_pos` (0.02 m) and `arm_target_rot` (5°) of
  the commanded EE: farther, the press is refused for good (counted `arm_far`, an event at most once a second; a new
  `arm_seq` is needed), so a client's stale riding pose can never jump the arm. While disabled the rows carry NaN for
  the commanded and measured EE (and `q_cmd`): the arm floats, there is no pose to capture; enable seeds them again
  from the measured joints. A client still sending `armed: true` with an old `arm_seq` can never re-arm after any disarm (counted
  `arm_refused`). An `alive` never arms or resumes anything. Datagrams queued before an enable or a new session never
  arm.
- **Disarms** (one `DISARMED: <reason>` event each; `/state.disarm_reason`): a datagram with `armed: false` (or
  without `armed` / `arm_seq`, or an older `arm_seq`); `POST /disarm`; stale input (no datagram for `hold_timeout`);
  the client lease ending; a new session; enable; disable; an estimator reset, pose jump or unusable pose; a
  planned motion pausing
  the arm; any fault; the e-stop; a control tick later than 3 dt (below). Headset latency / staleness is the
  client's: it sends `armed: false` or stops (then `hold_timeout`).
- `/state`: `armed`, `disarm_reason`, `latch` (`arm_seq`, `arm_seq_seen`, `arm_floor`, `disarms`,
  `ignored_disarmed`, `arm_refused`, `limits_ignored`, `clamped`); the stream's `armed` column.

### Control tick timing

Each tick's period (the clock) and compute time are measured: `overruns` (period > 1.5 dt) and `late_ticks`
(> 3 dt) are counted (`/state.tick`, the stream's `tick_period`, `tick_compute`, `overruns`, `late_ticks`). A tick
later than 3 dt after the previous one **disarms** (`control tick overrun: N ms`, logged). The arm command is
integrated over the **measured** period clamped to `[dt, 3 dt]` (not the nominal dt), and the joint position damper
uses the same step, so a long step never crosses a joint limit.

### Stop paths: what the motors do

| Trigger | Latch | Arm | Base | Gripper |
| --- | --- | --- | --- | --- |
| Disarm (any trigger above; Quest X's first effect) | disarmed | held at its commanded joints (zero velocity) | brakes to zero within `max_accel` / `max_alpha`, then held at zero | keeps its goal |
| `POST /disable` (Quest X) | disarmed | a last hold command, then released: the arm node floats it (gravity compensation) | no further twist; the swerve is disabled and released: coasts | released with the arm (damping) |
| `POST /estop` (WBC) | disarmed | held at its commanded joints until its own e-stop lands (latched, off) | no further command at all (no zero twist); its own e-stop: latched, coasts | with the arm |
| Fault | disarmed | the healthy half holds its commanded joints | the healthy half gets zero; a lost base is disabled (coasts) | keeps its goal |

The e-stop, in order: under the lock, stop tracking (disarm, the base twist zeroed in the WBC's own state and the
base sender muted: nothing more is sent to the swerve; the arm's hold command); then the e-stop to the arm and the
swerve **in parallel** (a slow one never delays the other), then disable. Neither e-stop nor disable brakes the base
of the robot's own volition.

Dropped datagrams are counted in `/state.session.dropped` by reason: `not_json`, `bad_type`, `bad_header`,
`unknown_session`, `old_seq`, `est_reset`, `frame`, `bad_t` (|t - now| beyond `target_t_window`, a chunk longer
than `chunk_max_horizon`), `stale_chunk` (ends in the past), `invalid` (shape, non-finite, gripper out of 0..1,
unknown EE; one event a second at most).

### Targets, chunks, staleness

- A chunk replaces the buffer; samples already in the past are dropped (the trajectory is evaluated from now);
  it is blended in with a critically damped offset anchored at the **commanded** EE pose and velocity at that
  tick (`blend_tau` 0.15 s): continuous in pose and velocity at every seam (tested). Without twists the chunk's
  velocity is its finite differences.
- Staleness, two stages: past `ff_timeout` (0.1 s) after the sample the feedforward fades (zero at 2×) and the
  pose holds; no datagram for `hold_timeout` (0.25 s) → **disarmed** (the targets cleared: nothing resumes; a new
  press arms). A lapsed client lease disarms the same way (the downstream leases stay until `POST /disable`).
- Estimator: a reset (`reset_id` changes) or a pose jump aborts the target buffers and disarms; so does an unusable
  pose (stale stream, non-finite, too old: `disarm_reason` `estimator: ...`), and no datagram arms until it is usable
  again; its return never resumes anything (a new press). The base pose used is the estimator's latest, extrapolated by the commanded twist (at
  most `max_predict`).

### Hints

Session defaults (`POST /hints`, replaced as a whole), overridable per datagram (`base`: any subset of the base
keys; it stays in force until the next datagram that carries `base`, the next `POST /hints`, or a disarm). Numbers
are bounded by the caps (`POST /hints`: 422 beyond them). In a datagram, velocities beyond `max_speed` /
`max_omega` (`velocity.vx/vy/omega`, `target.pose.vx/vy`, `target.heading.omega`) are **clamped** to the caps
(direction kept; counted `latch.clamped`), not dropped. A datagram's `base.limits` is **ignored while the base
moves** (the limits in force stay; counted `limits_ignored`).

```yaml
base:
  mode: free            # free | locked | hint | commanded
  target:               # mode hint: soft targets for the base, each optional (no cost inside tol)
    pose: {x: 1.2, y: 0.4, vx: 0.0, vy: 0.0, frame: world, weight: 1.0, tol: 0.05}
    heading: {yaw: 1.57, omega: 0.0, frame: world, weight: 0.5, tol: 0.05}   # or {face: arm} / {face: [x, y]}
  velocity: {vx: 0.1, vy: 0.0, omega: 0.0, frame: base, ttl: 0.3}   # mode commanded
  pose: {x: 1.2, y: 0.4, yaw: 0.0, frame: world}                    # mode commanded, without velocity
  limits: {max_speed: 0.3, max_omega: 0.8, max_accel: 0.5}          # tighter than the caps only
  arm_vs_base: 1.0      # > 1 prefers the arm, < 1 the base (scales the base cost)
ees:
  arm:
    weights: {position: 1.0, orientation: 0.3}
    orientation_mask: [1, 1, 1]   # in the TCP's own axes; 0 drops that rotation (this TCP's tool axis is x)
    posture: ready                # home | ready | 6 joint angles (rad)
    posture_weight: 0.0001
```

**Velocity feedforward on the hint targets** (`vx`, `vy` on the pose, world m/s; `omega` on the heading, rad/s;
optional, default 0, bounded by `max_speed` / `max_omega`): the hint stage tracks `v_ff + k_base_pose · error`
(`omega + k_heading · error` for the heading) instead of the error alone, within the caps, so a moving hint target
(a walking operator) is followed without the `v / k` lag; with no velocity it is the position-only behaviour. The
feedforward fades like the EE twist: full for `ff_timeout` after the hint was given (a datagram's `base.target`, or
`POST /hints`), zero at twice that (diagnostic `hint_ff_scale`). Measured in the QP (`test_wbc_controller.py`), walking
at 0.5 m/s with the EE target and a base pose hint moving together: the EE 142 mm behind without its twist, 0.9 mm
with; the base 250 mm behind its hint without the velocity, 18 mm with.

`locked` brakes the base to zero within its rate limit and holds it there; `commanded` follows the velocity (zero
after `ttl`) or drives to the pose (P, capped), the arm compensating. **Base hints yield**: they are solved in a
second, lower-priority stage that may only use the freedom the EE tasks leave (their velocities fixed to the first
stage's), so a hint never costs tracking; where a limit binds (reach, manipulability, joint limits) the base stops
at the boundary.

## The controller

`controller.Controller` is the swap point (QP now, MPC later): `step(Snapshot) -> Output` (base twist, arm joint
steps, status, `limited_by`, diagnostics, an optional plan). The QP (`QPController`, pinocchio + qpsolvers/daqp)
solves for the velocity of `[x, y, yaw (world), 6 arm joints]` every tick:

- **Costs:** EE tracking `|W (J q' - (v_ff + K e))|²` (`k_pos`, `k_rot` 4/s, errors clipped at `max_err_pos` /
  `max_err_rot`; orientation rows in the TCP's axes with the mask; Levenberg–Marquardt `lm |W e|²` plus adaptive
  damping on the arm growing as its manipulability falls below `w_influence`); base regularisation scheduled by
  the EE's distance to its target (from `w_base_near` within `near_dist` to `w_base_far` beyond `far_dist`,
  geometric, × `arm_vs_base`; yaw weighted at `base_yaw_radius`); arm regularisation; posture and the arm-only
  manipulability gradient (of log w) as **tie-breakers**, scaled by the desired EE speed (`tie_speed`), so a
  satisfied target produces no self-motion (measured: no drift over 10 s); base hints (second stage, above).
- **Hard constraints:** per arm joint a velocity box from the speed cap, the acceleration limit around the
  previous velocity, a position damper (`limit_gain` of the distance per step) and the braking distance
  `sqrt(2 a d)` (beyond a limit: back toward it), always feasible (the acceleration limit wins); **reach**: the
  shoulder (joint 1's axis point) to each target within `reach_fraction` (0.88) of the nominal reach (0.68 m,
  sampled from the model), as a damper beyond `reach_influence` and a braking-distance bound, on the base only —
  it only ever stops motion, never starts it; **manipulability floor**: `∇w·q' ≥ -xi w_i (w - w_min)/(w_i -
  w_min)` below `w_influence` (0.002; `w_min` 0.0008; the home pose has ~0.005), never below what the joint box
  allows; base speed (inscribed octagon), yaw rate, twist rate (octagon + yaw), every wheel's speed (inscribed
  octagon, 8 rows each; the swerve modules from `swerve/constants.py`); the **lead bound** per arm joint (above).
- **Deadband with hysteresis** on the base: a solve without the acceleration limits gives the desired twist; a
  stopped base starts above `deadband_*_on` (0.02 m/s, 0.06 rad/s), a moving one brakes below `*_off`; otherwise
  the base velocity is fixed to the braking step and the arm takes the motion. A second solve with the
  twist-rate rows runs only when the first breaks them. 1–4 QPs per tick.
- **Failure:** an infeasible or failed solve outputs zero velocity → the node faults and holds.

Arm output: integrated from the previous **commanded** joints; resynced to the measured ones only past `resync`
(0.25 rad, an event; with the lead bound a last-resort net) and held within the lead bound of them; q' is sent as
the velocity feedforward. Gripper: `gripper × 49.6 mm` as the gap goal.

Known limits of v1 (measured in the tests): a velocity-level QP can end in a local minimum with the arm at a joint
limit (a target behind and below the base): the error stays, reported (`limited_by` joint_limit), no yank. With
the arm limited and the base stopped, up to `deadband_lin_on / k_pos` ≈ 5 mm of error can remain (that
correction would need the base). Sliding along the reach boundary under a base hint gave a 15 mm transient.

## Safety and faults

- **Cross-bus:** an e-stop on either bus e-stops the other (the swerve's latched e-stop seen in its stream; the
  arm turning `off`); a fault, a lease loss, a disabled peer, failing swerve commands (> 0.25 s), a closed joint
  channel or no arm state for `arm_stale` (0.2 s: Pi arm-loop gaps of 105–121 ms were measured under camera load,
  docs/realtime.md) → **fault** (disarms): latched until `POST /disable`; the healthy half
  holds (the arm keeps its commanded joints, the base gets zero) and a lost base is disabled and released.
- The arm node renews its lease on every joint-channel command; the WBC renews the swerve lease with every twist
  (100 Hz).
- The WBC joins the frontend's e-stop fan-out (`actuated: true` in `/meta`).

## Stream (`/stream`, `/fields`)

The arm's convention: timestamped rows on the Pi clock, append-only. The contract's columns first: `t_tick`,
`seq`, `state` (0 disabled, 1 enabling, 2 idle, 3 tracking, 4 holding, 5 fault), `base_mode` (0 free, 1 locked,
2 hint, 3 commanded), `solve_ms`, `status` (0 none, 1 ok, 2 infeasible, 3 error), `base_x`, `base_y`, `base_yaw`
(world, the pose used), `est_reset`, `base_vx`, `base_vy`, `base_w` (the commanded twist in the **robot base
frame**: forward, left, yaw rate — as the swerve command and the velocity hint), per EE `<ee>_cmd_x..qw` (the pose
commanded this tick, world: anchor relative targets to it), `<ee>_meas_x..qw` (the arm's measured q at the base
pose used), `<ee>_twist_vx..wz` (commanded, world axes), `<ee>_gripper`; `arm_age`, `est_age`, `target_age`.
Appended: `limited`, `limited_by` (0 none, 1 reach, 2 manipulability, 3 fence, 4 joint_limit, 5 base_limit,
6 base_band, 7 lead), `input_age`, `hold`, `deadband`, `solves`, `wheel_max`, per EE the target (`_tgt_*`),
`_err_pos`, `_err_rot`, `_ff_scale`, `_target_id`, `_manip`, `_reach`, `_q_cmd1..6`, `_qd1..6`; then `armed`,
`tick_period` (ms), `tick_compute` (ms), `overruns`, `late_ticks`; then per EE `_err_meas_pos`, `_err_meas_rot`,
`_lead_pos`, `_lead_rot`, `_lead_tau1..6` (above: "Three error numbers"). `/state` shows `armed`,
`disarm_reason`, `latch`, `tick`, `limited_by`, `hold_reason`, `fault`, the session counters, the inputs, the lead
bound (`arm.lead`) and the latest row.

`/state` and the page's `/events` never stall the control tick: `state_dict` copies a snapshot (references and
scalars) under the control lock and builds the document outside it; `/events` (SSE) sends on a change at most every
0.5 s (2 Hz; changes in between coalesced) and at least every second.

`POST /config` (runtime tuning, no lease): the base caps `max_speed`, `max_omega`, `max_accel`, `max_alpha`,
`wheel_speed` are refused (422, the reason) unless the base is still (disabled, or a zero commanded twist: disarm
or lock it first); a value equal to the one in force is not a change. Structural keys only while disabled.

## Performance (MacBook M5, 2026-11-21, measured)

Per control tick (all QPs, the model, the constraints): free base reaching 0.6 m: p50 0.18 ms, p99 0.26 ms
(2.2 QPs per tick); base locked: p50 0.15 ms (1 QP); with a base hint (two stages): p50 0.21 ms, p99 0.28 ms.
The manipulability gradient (forward differences, 7 Jacobians) is 0.03 ms. Estimate for the Pi 5 (4–6× slower
in pure-Python/numpy code here): ~1 ms p50, ~1.5 ms p99 of the 10 ms tick.

## Internal interfaces and tests

- `arm_channel.py`: the WBC's session on the arm's internal joint channel, on `nodes/common/fastpath.py` (packets,
  control messages, `bind_dgram`). It uses the arm's welcome: `lo` / `hi` narrow the model's joint range,
  `joint_vel` / `joint_acc` are the controller's arm caps (the config's narrow them), `max_lead` bounds the resync (at most 0.7 of
  it: the arm drops goals beyond it), `gap_max` scales the gripper. The arm's `bye` (replaced, lease end, e-stop,
  disable, shutdown) is losing the arm: a fault. A planned motion on the arm (HTTP home / stow / servo / pose /
  chunk) pauses the session (the PAUSED flag): the WBC holds (reason `arm paused by a planned motion`), follows the
  measured joints and sends keep-alives; when it ends, the commands restart from the measured joints. RUS1's `seq`
  is the goal the previous arm tick applied; in the world sim the measured arm trails the command by about 5 ticks
  (well inside the resync threshold).
- Tests: `tests/test_wbc_controller.py` (pinocchio only), `tests/test_wbc_node.py` (lockstep against a fake arm:
  HTTP + a joint-channel peer on fastpath, `tests/wbc_fakes.py`, including the pause and the arm's bye; and real
  time with the swerve node on its virtual SPARKs and the estimator), `tests/test_wbc_sim.py` (the MuJoCo world
  with the real nodes: the arm node on `WorldArmBus` with its joint channel, the swerve on world-coupled SPARKs,
  the estimator, the WBC; the arm drops none of the WBC's goals). `tests/test_wbc_armed.py` (the armed latch: every
  disarm trigger, no recovery re-arms, the arm_seq floor, braking / holding / gripper while disarmed, the caps while
  moving, clamped hint velocities, the e-stop's order and parallel fan-out, no command reaching the base on e-stop
  or disable), `tests/test_wbc_overrun.py` (tick timing: late-tick disarm, overrun counts, the measured-period
  integration, a stalled controller in real time). `tests/test_wbc_lead.py` (the lead bound on a measured arm that
  lags or is blocked: contact within the budget, free motion at the caps untouched, release, sudden blockage; the
  node: kp from the arm's /gains, runtime config, `/state.arm.lead`, the three error numbers; `/state` holding the
  control lock only briefly, `/events` at most 2 Hz).

## Not done yet

The base fence; the MPC; a measured `ff_timeout` / `hold_timeout` / deadband from teleop sessions; `--udp-host` on
the robot (the LAN address in `nodes.yaml`).
