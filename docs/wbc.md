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
the robot set the LAN address, never `0.0.0.0`), `--config` (YAML of `spec.Config` keys), `--name`, `--port`.
Registered in `frontend/nodes.yaml` as `wbc`; page at the node root (state, base mode, EE commanded / measured /
target, tracking error, base twist, solve time, events).

## Using it

1. `POST /lease {"client": "me"}` → `{token, heartbeat, lease_id, client}`; the token goes in `X-Lease-Token`;
   `POST /lease/renew`, `DELETE /lease` (common/lease, as the arm and the swerve). Every accepted datagram also
   renews it.
2. `POST /enable`: takes the **arm and swerve leases together** (if one fails, the other is given back), sets the
   swerve's soft limits to `max_speed` / `max_omega`, enables both, opens the arm's **internal joint channel**
   (`<runtime>/robot/arm.joint.sock`, hello with the arm lease token) and seeds the commanded joints from the
   arm's measured ones. Refuses with the reason (409 / 503) otherwise. The WBC and pose_servo exclude each other
   through the swerve lease.
3. `POST /session {}` → `{session, udp_port, unix_socket}`. Send UDP **directly to the node's host** (not
   through the proxy) or to the Unix socket on the robot.
4. Datagrams (JSON v0; `seq` strictly increasing; older / equal and unknown sessions are dropped and counted):
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
5. `POST /hints` (the hint structure below), `POST /disable` (base stopped and the swerve released; a last hold
   command to the arm, then its channel closed and its lease released: the arm node then floats it), `POST /estop`
   (anyone; forwarded to the arm and the swerve, both latch; the WBC latches too until `POST /estop/clear`).
   `GET /fence` answers 501 (not built).

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
  pose holds; no datagram for `hold_timeout` (0.25 s) while a target exists → **holding**: the base brakes to zero
  within `max_accel`, the arm decelerates within its joint limits and stops. Any datagram resumes the target in
  force. A lapsed client lease holds the same way (the downstream leases stay until `POST /disable`).
- Estimator: a reset (`reset_id` changes) or a pose jump aborts the target buffers (idle = hold); a stale or
  non-finite pose holds. The base pose used is the estimator's latest, extrapolated by the commanded twist (at
  most `max_predict`).

### Hints

Session defaults (`POST /hints`, replaced as a whole), overridable per datagram (`base`: any subset of the base
keys; it stays in force until the next datagram that carries `base`, or the next `POST /hints`). Numbers are
bounded by the caps (422 beyond them).

```yaml
base:
  mode: free            # free | locked | hint | commanded
  target:               # mode hint: soft targets for the base, each optional (no cost inside tol)
    pose: {x: 1.2, y: 0.4, frame: world, weight: 1.0, tol: 0.05}
    heading: {yaw: 1.57, frame: world, weight: 0.5, tol: 0.05}   # or {face: arm} / {face: [x, y]}
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
  octagon, 8 rows each; the swerve modules from `swerve/constants.py`).
- **Deadband with hysteresis** on the base: a solve without the acceleration limits gives the desired twist; a
  stopped base starts above `deadband_*_on` (0.02 m/s, 0.06 rad/s), a moving one brakes below `*_off`; otherwise
  the base velocity is fixed to the braking step and the arm takes the motion. A second solve with the
  twist-rate rows runs only when the first breaks them. 1–4 QPs per tick.
- **Failure:** an infeasible or failed solve outputs zero velocity → the node faults and holds.

Arm output: integrated from the previous **commanded** joints; resynced to the measured ones only past `resync`
(0.12 rad, an event); q' is sent as the velocity feedforward. Gripper: `gripper × 49.6 mm` as the gap goal.

Known limits of v1 (measured in the tests): a velocity-level QP can end in a local minimum with the arm at a joint
limit (a target behind and below the base): the error stays, reported (`limited_by` joint_limit), no yank. With
the arm limited and the base stopped, up to `deadband_lin_on / k_pos` ≈ 5 mm of error can remain (that
correction would need the base). Sliding along the reach boundary under a base hint gave a 15 mm transient.

## Safety and faults

- **Cross-bus:** an e-stop on either bus e-stops the other (the swerve's latched e-stop seen in its stream; the
  arm turning `off`); a fault, a lease loss, a disabled peer, failing swerve commands (> 0.25 s), a closed joint
  channel or no arm state for `arm_stale` (0.1 s) → **fault**: latched until `POST /disable`; the healthy half
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
Appended: `limited`, `limited_by` (0 none, 1 reach, 2 manipulability, 3 fence, 4 joint_limit, 5 base_limit),
`input_age`, `hold`, `deadband`, `solves`, `wheel_max`, per EE the target (`_tgt_*`), `_err_pos`, `_err_rot`,
`_ff_scale`, `_target_id`, `_manip`, `_reach`, `_q_cmd1..6`, `_qd1..6`. `/state` shows `limited_by`,
`hold_reason`, `fault`, the session counters, the inputs and the latest row.

## Performance (MacBook M5, 2026-11-21, measured)

Per control tick (all QPs, the model, the constraints): free base reaching 0.6 m: p50 0.18 ms, p99 0.26 ms
(2.2 QPs per tick); base locked: p50 0.15 ms (1 QP); with a base hint (two stages): p50 0.21 ms, p99 0.28 ms.
The manipulability gradient (forward differences, 7 Jacobians) is 0.03 ms. Estimate for the Pi 5 (4–6× slower
in pure-Python/numpy code here): ~1 ms p50, ~1.5 ms p99 of the 10 ms tick.

## Internal interfaces and tests

- `arm_channel.py`: the WBC's session on the arm's internal joint channel, on `nodes/common/fastpath.py` (packets,
  control messages, `bind_dgram`). It uses the arm's welcome: `lo` / `hi` narrow the model's joint range,
  `joint_vel` / `joint_acc` narrow the controller's caps, `max_lead` bounds the resync threshold (at most half of
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
  the estimator, the WBC; the arm drops none of the WBC's goals).

## Not done yet

The base fence; the MPC; a measured `ff_timeout` / `hold_timeout` / deadband from teleop sessions; `--udp-host` on
the robot (the LAN address in `nodes.yaml`).
