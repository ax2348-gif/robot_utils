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
the robot set the LAN address, never `0.0.0.0`), `--config` (YAML of `spec.Config` keys), `--control`
(`process`, the default: the control loop in its own process, below; `thread`: the old single-process structure, for
comparison and debugging), `--switch-interval` (Python's GIL switch interval of the server process, default 0.005 s =
Python's own, env `WBC_SWITCH_INTERVAL`; with `--control thread` it is the control thread's too: 0.0005 s made the
robot's tick overruns much worse), `--name`, `--port`.
Registered in `frontend/nodes.yaml` as `wbc`; page at the node root (state, base mode, EE commanded / measured /
target, tracking error, base twist, solve time, events).

### Processes

The 100 Hz control loop runs **in its own process**, alone with its GIL. Measured on the Pi (2026-11-21/22) with the
loop as a SCHED_FIFO thread of the server process: that process ran many normal-priority threads (uvicorn, the
page's SSE, the estimator and swerve stream readers at 100 Hz, the joint-channel and datagram receivers, the swerve
sender) and the control tick regularly waited 30–250 ms for the GIL held by a descheduled normal thread (priority
inversion): `control tick overrun` disarms every few seconds, `estimator stream stale`, the client's `WBC state
stale`. Neither a shorter switch interval nor higher thresholds fixed it: the GIL is per process.

- **Control process** (`control_proc.py`, spawned by the node at start; SCHED_FIFO 20 on its one thread, on macOS
  the QoS / App Nap path): ONE thread. Each tick it polls its non-blocking sockets (the arm's joint channel: it owns
  the arm session; the target datagrams, UDP and Unix: it parses and applies them, so the armed latch is exactly as
  below), reads the estimator and swerve rows from shared memory, runs the tick (the latch, staleness, holds, the QP,
  integration, the outputs: the same `WbcNode` code), sends the arm command, and writes the base twist, the stream
  row and (every 2nd tick and after every command) a `/state` snapshot to shared memory. No HTTP, no JSON documents,
  no other threads; `gc.freeze()` after the setup.
- **Server process** (the FastAPI node, `proc_node.py`): HTTP and the page, SSE, `/stream` (the rows from the
  shared ring), the estimator and swerve stream readers (their rows into rings for the control process), the swerve
  sender (posting the twist slot; muted on e-stop / disable as before), the client lease (renewed for every
  datagram the control process counts), the arm and swerve leases, enable / disable, `/config`, `/hints`,
  `/session`, `/disarm`, `/estop` with the parallel fan-out.
- **IPC** (`ipc.py`): one shared-memory file (mmap, a private 0700 directory on tmpfs: `$XDG_RUNTIME_DIR` on the
  robot) with heartbeats, seqlocked latest-value slots (the twist, the server's lease / sender status, the snapshot;
  CRC-checked) and single-producer rings (stream rows, estimator rows, swerve rows; drop-oldest, counted); commands on
  two Unix datagram sockets (pickled; non-blocking on the control side). After every tick the control process sends
  a `tick` message: the server's one IPC thread then posts the twist and publishes the row (event-driven, no
  polling: an earlier 500 Hz polling pump made the server process's other threads late). The control loop waits for its tick on the
  command socket, so a command is applied at once between ticks: the e-stop first (it disarms and holds the arm on
  receipt). Requests that need an answer (enable, disable, session, hints, config, disarm, e-stop) wait for the
  control process's.
- **Failures.** The control process watches the server's heartbeat: missing for `srv_heartbeat` (0.5 s) → disarm and
  hold (arm at its commanded joints, base braking to zero), the arm channel kept alive (the arm's own watchdog
  semantics), no arming until it is back (then still disarmed: a new press). The server watches the control
  process's: missing for `ctl_heartbeat` (0.2 s), or the process gone → **fault** (an event; the base muted; the
  swerve and the arm disabled and released; `/state` `state: fault`, `fault: <reason>`; every request needing the
  control process answers 503). **Nothing restarts automatically: the operator restarts the node.** The server's
  process gone → the control process ends (the joint channel's bye). Node shutdown stops it cleanly.
- `/state.process` (added): `mode`, `pid`, `alive`, `fault`, `heartbeat_age`, `snapshot_age`, `rows_dropped`,
  `commands_dropped`, `pump_max_ms`. `/state` is the snapshot (≤ 20 ms old); everything else is unchanged: the
  routes, the stream fields, the datagram formats, the latch, the stop paths.
- Measured (`tests/test_wbc_proc.py`, MacBook, no SCHED_FIFO; the server hammered by 6 `/state` pollers, 3 SSE
  and 3 `/stream` clients and three simulated 100 Hz readers while tracking a moving target): tick period p99
  11.9–13.3 ms, max 12.0–13.7 ms, 0 overruns with the process; 13.1–17.4 ms p99, 13.3–20.1 ms max, 1–40 overruns
  as a thread (two runs). On the Mac neither has SCHED_FIFO; the robot's inversion (a FIFO thread waiting for a
  descheduled normal one) cannot happen across processes.

### Defaults (for the first tests, the owner's)

Base: `max_speed` 0.8 m/s (body), `max_omega` 0.9 rad/s, `max_accel` 2.0 m/s², `max_alpha` 1.5 rad/s², every
wheel `wheel_speed` 2.0 m/s (the owner's tuned values, 2026-11-24; first tests: 0.6, 0.8, 1.0, 1.0). Arm: `joint_vel` / `joint_acc` default to **the arm's own** (null in the config): the
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
     wz] (optional, world axes), "gripper": 0..1 (optional; 0 closed, 1 open), "swivel": {"seq", "dpsi",
     "elbow", "conf"} (optional; a 7-DOF arm's elbow: "The operator's elbow" below)}}, "base": {<hint overrides>}
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
  the arm; any fault; the e-stop; a control tick later than `late_tick` (below); the server process's heartbeat
  lost ("Processes"). Headset latency / staleness is the
  client's: it sends `armed: false` or stops (then `hold_timeout`).
- `/state`: `armed`, `disarm_reason`, `latch` (`arm_seq`, `arm_seq_seen`, `arm_floor`, `disarms`,
  `ignored_disarmed`, `arm_refused`, `limits_ignored`, `clamped`); the stream's `armed` column.

### Control tick timing

Each tick's period (the clock) and compute time are measured: `overruns` (period > 1.5 dt) and `late_ticks`
(> `late_tick`, 0.1 s) are counted (`/state.tick`, the stream's `tick_period`, `tick_compute`, `overruns`,
`late_ticks`). A tick later than `late_tick` after the previous one **disarms** (`control tick overrun: N ms`,
logged); one between 3 dt and `late_tick` is an overrun only (`late_tick` was 3 dt before the GIL stalls; with the
control process it can come down again once the robot shows no overruns). The arm command is
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
optional, default 0, bounded by `max_speed` / `max_omega`): the base reference becomes `v_ff + k_base_pose · error`
(`omega + k_heading · error` for the heading) instead of the error alone, within the caps, so a moving hint target
(a walking operator) is followed without the `v / k` lag; with no velocity it is the position-only behaviour. The
feedforward fades like the EE twist: full for `ff_timeout` after the hint was given (a datagram's `base.target`, or
`POST /hints`), zero at twice that. Measured in the QP (`test_wbc_controller.py`), walking
at 0.5 m/s with the EE target and a base pose hint moving together: the EE 142 mm behind without its twist, 0.9 mm
with; the base 250 mm behind its hint without the velocity, 18 mm with.

`locked` (and a hold, or no base) brakes the base to zero within its rate limit and holds it there: the only modes
that fix the base velocity. `hint` and `commanded` give the base a **reference** in the same QP as the EE tasks
("The controller" below): `commanded` a velocity (zero after `ttl`) or a pose (P, capped), `hint` the pose / heading
targets with their feedforward; `free` none (the base is regularised toward zero, weakly). A reference is a cost
weighed against EE tracking, not a priority level: there is **one QP, no second stage**. What keeps the base with
its reference is the weighting (`w_base_ref` × `arm_vs_base`) and the **bands** (hard, with a penalised slack): the
base stays within `band_pos` (0.15 m) / `band_yaw` (20°) of the reference pose, and its twist may lag the reference
down to zero but exceed it by at most `band_twist` (0.1 m/s) / `band_omega` (0.3 rad/s), so fast EE motion goes to
the arm. Within those bands the base moves to help the EE tasks; where a limit binds (reach, manipulability, joint
limits) the base stops at the boundary.

### Several arms (`--robot`)

The model's end effectors are the robot profile's arms (`src/robot_utils/robots/`; `ee_specs`): the default robot's
one EE `arm`, the bimanual robot's `arm_left` and `arm_right`. Each EE has its own arm node (named as the EE; a
one-EE model keeps `--arm`): its own lease, joint channel, state, gains, joint limits and gap, all taken and
released together at enable / disable. Every check (stale state, the arm's bye, off / fault), hold, resync, pause
and e-stop covers every arm; the armed latch, the session and the base are shared, so one arm's fault holds both
and one arm's e-stop goes to the base and the other arms. Datagrams carry each EE under `ees` as before; the stream
has each EE's `<ee>_*` columns; `arm_age` is the oldest arm state's. `/state` keeps `arm` (the first arm) and adds
`arms` (the same block per EE). The QP has no arm-arm collision constraint (decided); a redundant arm pair may have the soft, short-range
arm-proximity term ("Null-space posture" below). The robot profile's `wbc:` block gives the config's defaults for
that robot (the node's `--config` overrides them). docs/plans/bimanual_and_robot_swap.md.

## The controller

`controller.Controller` is the swap point (QP now, MPC later): `step(Snapshot) -> Output` (base twist, arm joint
steps, status, `limited_by`, diagnostics, an optional plan). The QP (`QPController`, pinocchio + qpsolvers/daqp)
solves **one QP** for the velocity of `[x, y, yaw (world), the arms' joints]` every tick (6 per reBot arm, 7 per OpenArm;
docs/plans/wbc_v1.md, "Formulation"):

- **Costs:** EE tracking `|W (J q' - (v_ff + K e))|²` (`k_pos`, `k_rot` 4/s, errors clipped at `max_err_pos` /
  `max_err_rot`; orientation rows in the TCP's axes with the mask; Levenberg–Marquardt `lm |W e|²` plus adaptive
  damping on the arm growing as its manipulability falls below `w_influence`); the **base reference**: the base
  twist pulled toward `v_des = v_ref + k (pose_ref - pose)` (tolerance `tol` inside which the error counts as zero)
  with weight `w_base_ref` (1.0) when the mode gives a reference (`hint`, `commanded`), toward zero with
  `w_base_free` (0.001) when it does not (`free`), × `arm_vs_base`, yaw weighted at `base_yaw_radius`: the arm / base
  split of EE motion follows from these weights against the EE task's; arm regularisation; posture and the arm-only
  manipulability gradient (of log w) as **tie-breakers**, scaled by the desired EE speed (`tie_speed`), so a
  satisfied target produces no self-motion (measured: no drift over 10 s). A flat (L1) cost of moving the base at
  all (`w_base_flat`, `w_base_flat_yaw`) exists and is off (0): it biased the walking speed (a constant 5 cm lag at
  0.10); the start charge below replaced it.
- **Hard constraints:** per arm joint a velocity box from the speed cap, the acceleration limit around the
  previous velocity, a position damper (`limit_gain` of the distance per step) and the braking distance
  `sqrt(2 a d)` (beyond a limit: back toward it), always feasible (the acceleration limit wins); **reach**: the
  shoulder (joint 1's axis point) to each target within `reach_fraction` (0.88) of the nominal reach (0.68 m,
  sampled from the model), as a damper beyond `reach_influence` and a braking-distance bound, on the base only —
  it only ever stops motion, never starts it; **manipulability floor**: `∇w·q' ≥ -xi w_i (w - w_min)/(w_i -
  w_min)` below `w_influence` (0.002; `w_min` 0.0008; the home pose has ~0.005), never below what the joint box
  allows; base speed (inscribed octagon), yaw rate, twist rate (octagon + yaw), every wheel's speed (inscribed
  octagon, 8 rows each; the swerve modules from `swerve/constants.py`); the **lead bound** per arm joint (above);
  the **base bands** around the reference (pose: the deviation grows at most at `k_band` × (band − deviation), never
  forced back faster; twist: conv{0, octagon(v_des, band_twist)}), each with an L1 slack penalised by `w_slack` (1e3)
  so the QP stays feasible.
- **Start charge** (no deadband): with the base at rest (below `rest_lin` 0.02 m/s / `rest_ang` 0.05 rad/s), two
  QPs without the twist-rate rows compare the base **held** (braking to zero) and **free**; the base starts only if
  the free solution's cost plus `base_start_charge` (0.01, objective units) is lower, so tiny motions (sway, small EE
  pushes: swerve chatter) are never worth starting; then the command is solved (held, or free with the rate rows).
  Exact (two discrete outcomes), no bias while moving. The controller's `Output` records the decision
  (`base_fixed` "at rest (start charge)", `diag.start_charge` with both costs); neither reaches the stream or
  `/state` yet. 1 QP per tick while the base moves or is fixed, 3 at rest, +1 if the
  base must brake into lowered caps (`rate fallback`).
- **Failure:** an infeasible or failed solve outputs zero velocity → the node faults and holds.

Arm output: integrated from the previous **commanded** joints; resynced to the measured ones only past `resync`
(0.25 rad, an event; with the lead bound a last-resort net) and held within the lead bound of them; q' is sent as
the velocity feedforward. Gripper: `gripper × 49.6 mm` as the gap goal.

### Null-space posture (a redundant arm: 7 DOF)

An arm with more joints than the EE task's 6 rows (the OpenArm, `openarm_bimanual`; the DOF comes from the model)
has one free angle per hand pose: the elbow's **swivel** about the shoulder-wrist line. Its posture terms live in the
EE task's **null space**: the cost `½ Σ wᵢ |P (q' - vᵢ)|²`, P the null-space projector of the EE task's weighted rows
over that arm's joints (damped pseudo-inverse), each term a joint-velocity attractor `vᵢ` with a weight `wᵢ`. P is a
symmetric projector, so this is `H += (Σ w) P`, `c -= P Σ wᵢ vᵢ`, in the one QP: the joint box, the lead bound, the
reach and the base coupling stay hard rows. Why not a lexicographic (second) QP: it would double the solves per tick
and repeat every hard row; the projection gives the same priority for the unconstrained part. **Saturation**: a joint
within `null_limit_margin` (0.1 rad) of a limit that the projected motion would push further in is taken out of the
projector and the projection redone (otherwise the joint box refuses that motion and the rest of it moves the EE: a
10 mm steady error with OpenArm joint 6 at its limit, before). The summed attractor is capped at
`null_posture_speed` (0.3 rad/s). A 6-joint arm keeps the weighted posture tie-breaker above (no null space).

`null_mode` picks the terms (`wbc/humanlike.py`):

- **`humanlike`** (the default), modelled on human arms: one natural elbow placement per hand pose, settled into
  unless something forces another. Priorities, by weight: EE task ≫ arm proximity > swivel > effort > centering.
  - **Swivel model** (`w_swivel` 1, `k_swivel` 1/s): a target swivel ψ* from the hand pose, manipulability based
    (Kim & Rosen's human-arm redundancy model): with the shoulder S and the wrist W fixed, the elbow lies on a
    circle; on a 4-joint arm model (a spherical shoulder and the elbow) the wrist's velocity transmission along a
    direction u is `|(W-S) × u|² + (u · n × (W-E))²` (n the elbow's axis); ψ* is its soft argmax over
    `swivel_range` (−20..100°; 0 = elbow straight down, + out; `swivel_beta` 10), continuous in the hand pose (no
    flips), low-passed (`swivel_tau` 0.3 s). ψ is measured numerically (the elbow off the S-W line against the
    world's down; OpenArm's joint origins 2 / 4 / 6 as S / E / W, the arm type's `geometry`), so no spherical-joint
    assumption. The attractor moves along the arm's self-motion only (`v = N (Nᵀ∇ψ) k (ψ*-ψ) / |Nᵀ∇ψ|²`, N the
    TCP Jacobian's null space). `swivel_dir`: **`head`** (default; u from the wrist to a head point `swivel_head`,
    base_link, the OpenArm profile's (0.12, 0, 1.229): its human-scale eye height; the "bring the hand toward the
    face / look at the hand" direction of the human studies; gives a moderate elbow-out swivel, ψ* ≈ 50° at the
    nominal hand pose, vs 61° for the fixed nominal posture), **`torso`** (u to the torso centre `swivel_torso`,
    the profile's (0.12, 0, 0.95): elbows further out, ≈ 83°, for pulling toward the body), **`motion`** (u along the hand's wanted
    velocity, blended in above 5 cm/s, the head direction while slow: the elbow turns to move the hand easily where
    it is going).
  - **Effort** (`w_effort` 0.3, `k_effort` 1): down the self-motion gradient of `log Σ (τ_g,i / τ_max,i)²`, the
    gravity torque the arm holds relative to each joint's torque limit (pinocchio's gravity derivatives): the "lazy"
    elbow-down-and-slightly-out posture (alone: ψ ≈ 19-21° at the nominal hand pose, Σ(τ/τmax)² 0.034 / 0.038 vs
    the nominal posture's 0.042 / 0.047; with the defaults the elbow settles at ψ ≈ 48°, 2° below ψ*).
  - **Joint centering** (`w_center` 0.1, `k_center` 0.1 rad/s): toward mid-range on the shoulder and wrist joints
    (the arm type's `geometry.centering`; not the elbow), a light RULA-like comfort term.
  - **Arm proximity** (`w_arm_proximity`, 0 = off; per robot profile: the OpenArm's `wbc:` block sets 30; off with
    one arm): a soft term, **never a hard collision constraint**, between two arms' **upper-arm and forearm
    capsules only** (the arm type's `geometry.capsules`: shoulder→elbow r 0.05 m, elbow→wrist r 0.045 m ending
    4 cm short of the wrist centre). The wrists and grippers are left out so the hands come together freely
    (handovers, two-hand grasps). **Strictly short range**: with clearance d below `proximity_clearance` c0 (0.05 m)
    and `s = (c0 - d)/c0`, the weight is `w s²` and the wanted separation speed `proximity_speed s` (0.2 m/s at
    contact), i.e. a push growing as s³ (C² at c0), exactly zero (value and gradient) at and beyond c0, so in the
    natural posture it is dormant and the posture is identical with it on or off (tested tick for tick). Per pair it
    moves each arm along its self-motion (the closest point's Jacobian along the pair's normal, the other arm held).
    At weight 30 over the swivel's 1 it wins near contact and lets go as the arms part; the elbow then drifts back.
- **`nominal`**: attract the fixed nominal posture (the profile's `posture`, the hint's `posture`) at
  `w_null_posture`; kept for comparison.

Tests (`tests/test_openarm.py`): the EE error history with every null-space term on matches the one with none,
within 2 mm on 0.07 m reaches (the transient; the reach itself to < 1 mm), and while the hands sweep fast (a ~2.4
cm lag behind reactive targets) within 5 mm (measured 3.2-3.7 mm: the null-space motion shares the joints' speed /
acceleration caps); the elbow settles on ψ* within 2° (swivel
alone) / 6° (with effort and centering) for three hand poses; ψ* and ψ change by < 0.5° per 10 ms tick as the hands
sweep (and the model is continuous along a dense line of hand poses); an elbow pushed ±0.4 (scaled) along the
self-motion returns within 1.5°, the hand within 1 mm; effort alone holds less gravity torque than the nominal
posture at four hand poses (mean ratio < 0.95); with targets drawing the left forearm onto the right elbow the
proximity term keeps the smallest upper-arm / forearm clearance larger (17.2 → 19.7 mm during the reach, where the
elbow's swivel speed is capped; 27.1 → 42.4 mm settled) with the hands still on target; far apart it changes nothing (identical joints tick for tick).

### Near the joint limits (a redundant arm): measured, and opt-in

On the OpenArm the velocity-level QP's main workspace shortfall is joint-limit local minima: the hand driven straight
at a target runs a joint into its limit and the arm stops there, though the target may be reachable in another
configuration. Measured (the robot profile's Anvil wrist and gripper, perfect tracking, base locked; scratch study,
2026-11-25, re-scored against the limits the WBC uses; docs/plans/bimanual_and_robot_swap.md "OpenArm workspace"):
of 68 pull-up-top hand poses (hands at and above shoulder height, 0.05-0.25 m in front of it, pull-up grips)
reachable within the URDF limits and collision-free, the QP reaches 51 from the nominal posture; 9 of the 17 misses
need a joint inside the WBC's 2 deg limit margin (60 reached with a 1 deg margin), 8 are true failures (6 on another
IK branch, 2 stopped where a local IK from there succeeds). Whole workspace (200 collision-free targets per arm, one
per 5 cm cell): 64 % (65 % of those reachable within the WBC's limits), every miss at a joint limit, 82 % of the true
failures on another IK branch. (These numbers predate Anvil's URDF, whose J6 range is the reverse of the one measured:
of the four pull-up-top poses in tests/test_openarm.py one is still reached from the re-solved nominal posture, three
stall; a re-run of the study is due.) Two null-space aids exist, **off by default** (they changed none of these numbers,
and the swivel planner costs up to 18 ms in a tick's 99th percentile):

- `swivel_limit_aware` (`wbc/swivel_feasible.py`): the self-motion manifold of the target pose traced numerically
  (10 deg swivel steps along the TCP Jacobian's null vector, two DLS corrections each; no near-SRS closed form) every
  `swivel_plan_period`; the human-like swivel target is clipped into the feasible run the arm is in (all joints within
  the WBC's limits minus `null_limit_margin`), so the elbow never steers into a dead end of its own branch. Status per
  arm in /state (`arms.<ee>.posture`: ok, limited, re-posing, unreachable) and an event on each change.
- `w_limit` (with `limit_band`, `limit_speed`): a joint-limit repulsion in the null space, zero outside the band.

Neither moves the hand: both act in the EE task's null space (tested: the hand within 1 mm while the elbow re-poses).
The misses they cannot fix need the hand to leave its path (another branch) or a joint inside the margin; the WBC
reports them (`limited_by: joint_limit`, the posture status `unreachable`) and does not move the hand on its own. A
predictive (MPC) WBC is the candidate for the remaining cases: docs/plans/bimanual_and_robot_swap.md. The branch is
the elbow's choice, and the operator can make it: below.

### The operator's elbow: the swivel hint

A teleop client can steer a redundant arm's elbow with a per-EE **`swivel`** object in its `target` datagrams. The
split: the client sends the operator's raw measurements, the WBC does all the computation (anchoring, smoothing,
fades). Absent (a client that never sends it, or a 6-joint arm): no hint, behaviour unchanged.

```json
"ees": {"arm_left": {"pos": [...], "quat": [...], "swivel": {"seq": 3, "dpsi": 0.42, "elbow": 2.1, "conf": 0.9}}}
```

- `seq` (int): the operator's engagement id; a new value is a new engagement (re-anchor).
- `dpsi` (rad): the operator's swivel change since that engagement, accumulated incrementally about their
  shoulder-wrist axis (the same measure as below), already sign-adjusted for mirror pairing (+ = elbow out).
- `elbow` (rad, 0..π): the operator's upper-arm-forearm angle, π = straight.
- `conf` (0..1): the client's confidence in the body tracking.

Validated with the rest of the datagram: a malformed `swivel` (a missing or extra key, a non-integer `seq`, a
non-finite number, `elbow` outside 0..π, `conf` outside 0..1) drops the datagram as `invalid`, counted in
`/state.session.dropped`. Reactive targets only (a chunk's `swivel` is ignored). A datagram without the field
leaves the last hint to age.

- **The swivel without a fixed zero** (`wbc/swivel_geom.py`): the elbow's rotation about the shoulder-wrist line,
  measured only as increments: between two ticks the elbow's offset from the S-W line is parallel-transported to the
  new line (the smallest rotation taking one onto the other) and the signed angle to the new offset, about the new
  line, is the step (+ elbow out, for either arm; zero while the elbow is on the line). The WBC sums the steps of the
  **commanded** arm every tick into the robot's swivel ψ_r, with an arbitrary origin (only differences are used). A
  fixed zero (the world's down projected, as the swivel model's ψ) is singular when the S-W line is vertical (reaching
  straight down); the increments are not.
- **Anchoring**: on a new `seq`, or the first fresh input after none or a stale one, `ψ_r0` = ψ_r and `dpsi0` = that
  datagram's `dpsi` (0 at a fresh engagement; its value at a return after a dropout, so what the operator did
  meanwhile is not replayed). The raw target is `ψ_r0 + swivel_hint_gain (dpsi - dpsi0)` (gain 1).
- **Smoothing**: the target in force is the raw one low-passed (`swivel_hint_tau` 0.05 s) and rate-capped
  (`swivel_hint_rate` 2 rad/s); at the first anchoring it starts at ψ_r (engaging moves nothing); a new `seq` while
  a hint is in force keeps it continuous (it eases to the new anchor).
- **The weight** = the ramped operator-side fade × the robot's fade. Operator side: the elbow fade (`π - elbow`, the
  operator's flexion, a smoothstep over `swivel_hint_flex` 10..30°) × the confidence fade (a smoothstep over
  `swivel_hint_conf` 0.3..0.6: none below 0.3), ramped toward that at 1 / `swivel_hint_blend` (0.15 s) per s. Robot
  side: the same flexion smoothstep on the robot's own elbow (π - the angle at the elbow between S and W; the
  OpenArm's equals joint 4). So it fades smoothly to zero toward either arm's full extension, where the swivel is
  undefined.
- **Stale or invalid** (released: teleop sends the field only while a hand is engaged; or the body lost): no valid
  `swivel` for `swivel_hint_timeout` (0.3 s). **Held** by default (`swivel_hint_hold`): the target and the weight
  stay, so the elbow stays where the operator left it, as the hand does when released, and the next engagement
  re-anchors there: swivels accumulate across clutches. Before (2026-11-26) the weight ramped to 0 and the model's ψ*
  took over, so every release drifted the elbow back and the relative mapping could not build up (the owner: "the
  relative formulation becomes impossible to use"). With the hold off, that fade-out (no step). A controller reset
  (disable) clears it. The next valid input re-anchors as a new `seq` does.
- **The term, null space only**: the hint replaces the swivel model's attractor by its weight `a`: two attractors,
  the model's ψ* at `(1 - a) w_swivel` and the hint's at `a w_swivel_hint` (3; below the proximity term's 30, which
  still acts), the hint's rate `k_swivel_hint (target - ψ_r)` (5/s) capped at `swivel_hint_rate`, along the gradient
  of ψ_r (central differences of the steps: defined for any S-W direction) in the self-motion. The null-space speed
  cap rises from `null_posture_speed` toward `swivel_hint_speed` (1.2 rad/s) by `a`. The speeds (2026-11-26; before 3/s, 1.5 rad/s, 0.1 s,
  1 rad/s, 0.3 s, which followed about half of the operator's swivel in 1–2 s grips): a 0.4 rad step reaches 90% in
  0.9 s (1.2 s before); the price is up to 1.4 mm more hand error during a fast reach (0.75 mm before), 0.2 mm at
  rest; at 6/s, 2 rad/s, 1.5 rad/s: 0.75 s, 2 mm, and 4.4 mm with the S–W line vertical. The hand stays primary, the joint
  box and every hard row stay; a hint the arm cannot follow is followed as far as the null space and the limits
  allow, never at the EE's cost. Only with `null_mode` humanlike.
- **Out**: the stream's `<ee>_swivel` (ψ_r) and `<ee>_swivel_w` (the weight in force `a`; 0 none); `/state`
  `arms.<ee>.posture.swivel` (ψ_r) and `.hint` (null when none): `seq`, `live` (fresh and anchored), `anchors`
  (anchorings so far), `weight` (in force), `anchor` (ψ_r0), `anchor_dpsi` (dpsi0), `dpsi`, `raw`, `target` (in
  force), `age`, `f_elbow`, `f_conf`, `blend` (the ramped operator side), `f_robot`.

Measured (tests/test_swivel_hint.py; perfect tracking, base locked): the operator turning ±0.5 rad moves the elbow
±0.5 from where it was at the engagement (gain 2: twice), to within 0.02 rad in 3 s, the hand within 0.5 mm, the
other arm untouched; with the operator sweeping ±0.4 rad during reaches the EE error history is the plain run's
within 1 mm; engaging moves nothing (< 0.005 rad/s change per tick); the fades at a dropout, its return and the end
are ramps (≤ 0.04 rad/s per tick, under 1/5 of the same cycle with the weight switched in one tick); a new `seq`
with the elbow still converging slows it over ~0.1 s; the robot's elbow at 3°: weight < 0.02 and the run is the
plain one; the operator's elbow straight or `conf` 0.1: weight 0; half-way along either fade: 0.5; the S-W line
within 3° of vertical: the hint moves the elbow as anywhere else; no hint, `conf` 0 or the operator's arm straight:
identical joints tick for tick. The elbow converges with a time constant of ~0.35 s (`k_swivel_hint`, diluted by the
effort and centering terms).

The pull-up top (the workspace study's 8 true failures: docs/plans/bimanual_and_robot_swap.md "OpenArm workspace";
the hand brought to the target along a 4 s path from the nominal posture, base locked; scratch `bm/hint_dpsi.py`):
the operator's `dpsi` scripted as d × (the path's progress, or over the first second), d ±0.5..2 rad, elbow and
conf full (mean weight 0.98: the robot's elbow stays bent). Final miss, mm (plain → best d):

| target | arm | class | plain | best scripted dpsi |
|---|---|---|---|---|
| 11 (overhand, 5 cm ahead, 10 cm above) | left | local | 126.5 | **0** (+0.5 early; also +1..+2 either shape: 7 of 16) |
| 26 | left | branch | 28.4 | 28.4 (none helps) |
| 33 (chin-up) | left | local | 6.6 | 6.6 (none helps) |
| 55 | right | branch | 319.7 | 277.2 (+0.5 early) |
| 56 | right | branch | 384.6 | 189.4 (-2 ramp) |
| 57 | right | branch | 345.9 | 190.3 (+0.5 ramp) |
| 60 | right | branch | 201.2 | 120.5 (-1 ramp) |
| 63 | right | branch | 183.5 | 119.7 (-1 ramp) |

One of the 8 reached (the elbow raised, as a person grips overhead; tested), 5 roughly halved, 2 unchanged; the
right arm's branch cases need the hand to leave its path, which the null space cannot do (the MPC's cases). In none
of these is the hand on its path anyway (the plain runs' path error 43-430 mm: the arm meets its limits on the way);
the hint changes where it meets them (max path error 228 vs 142 mm on 11, lower on 56, 57, 60, 63).

Known limits of v1 (measured in the tests): a velocity-level QP can end in a local minimum with the arm at a joint
limit (a target behind and below the base): the error stays, reported (`limited_by` joint_limit), no yank. With
the arm limited and the base at rest, a small error can remain when moving the base would not be worth the start
charge (that correction would need the base; not re-measured since the start charge replaced the deadband). Sliding along the reach boundary under a base hint gave a 15 mm transient.

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
6 base_band, 7 lead), `input_age`, `hold`, `deadband` (always 0 since the start charge replaced the deadband; the
start charge's decision is not streamed), `solves`, `wheel_max`, per EE the target (`_tgt_*`),
`_err_pos`, `_err_rot`, `_ff_scale`, `_target_id`, `_manip`, `_reach`, `_q_cmd1..6`, `_qd1..6`; then `armed`,
`tick_period` (ms), `tick_compute` (ms), `overruns`, `late_ticks`; then per EE `_err_meas_pos`, `_err_meas_rot`,
`_lead_pos`, `_lead_rot`, `_lead_tau1..6` (above: "Three error numbers"); then per EE `_swivel`, `_swivel_w`
(above: "The operator's elbow"; NaN for a 6-joint arm); then per EE `_limited_by`: that arm's own binding limit this
tick (its rows only, the `limited_by` codes: 0 none, 1 reach, 2 manipulability, 4 joint_limit, 7 lead; by the same
priority; `diag["limited_ees"]`), so the teleop's haptics buzz the hand driving that arm. `/state` shows `armed`,
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
(2.2 QPs per tick); base locked: p50 0.15 ms (1 QP); with a base hint: p50 0.21 ms, p99 0.28 ms. (Measured before
the one-QP formulation with bands and the start charge; not re-measured since.)
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
- Tests: `tests/test_wbc_proc.py` (the control process: spawn, enable / arm / tracking with the rows through
  the ring, config / hints / session / disarm / e-stop through the command channel, the lease renewed by datagrams
  and its lapse, heartbeat loss both ways, a stalled or killed control process, a clean shutdown, the IPC slot and
  ring, and the load test against the thread structure; the other node tests step `WbcNode` in-process, the same
  tick code), `tests/test_wbc_controller.py` (pinocchio only), `tests/test_wbc_node.py` (lockstep against a fake arm:
  HTTP + a joint-channel peer on fastpath, `tests/wbc_fakes.py`, including the pause and the arm's bye; and real
  time with the swerve node on its virtual SPARKs and the estimator), `tests/test_wbc_sim.py` (the MuJoCo world
  with the real nodes, the WBC in its control process: the arm node on `WorldArmBus` with its joint channel, the swerve on world-coupled SPARKs,
  the estimator, the WBC; the arm drops none of the WBC's goals). `tests/test_wbc_armed.py` (the armed latch: every
  disarm trigger, no recovery re-arms, the arm_seq floor, braking / holding / gripper while disarmed, the caps while
  moving, clamped hint velocities, the e-stop's order and parallel fan-out, no command reaching the base on e-stop
  or disable), `tests/test_wbc_overrun.py` (tick timing: late-tick disarm, overrun counts, the measured-period
  integration, a stalled controller in real time). `tests/test_wbc_lead.py` (the lead bound on a measured arm that
  lags or is blocked: contact within the budget, free motion at the caps untouched, release, sudden blockage; the
  node: kp from the arm's /gains, runtime config, `/state.arm.lead`, the three error numbers; `/state` holding the
  control lock only briefly, `/events` at most 2 Hz). `tests/test_swivel_hint.py` (the operator's swivel hint: the
  incremental swivel, the null-space term, anchoring and fades, the pull-up-top case, the datagram field, the stream
  and `/state`).

## Not done yet

The base fence; the MPC; a measured `ff_timeout` / `hold_timeout` / `base_start_charge` from teleop sessions; `--udp-host` on
the robot (the LAN address in `nodes.yaml`).

## Known bugs

- **A released arm near a singularity blocks the base** (found 2026-11-26, sim, OpenArm; deferred). With the hands
  released, a stick translation barely moves the base, which turns at `band_omega` (0.30 rad/s) instead until the
  direction frees up. Recording 20261126-025455, 03:08:49–03:09:29: stick (+0.4, −0.7) m/s gave base (+0.03, −0.04,
  +0.30); the left arm's elbow was nearly straight (J4 4°), and the swerve delivered every command. Cause: released
  arms ride on world-frame EE targets (the stream's base pose ∘ the relative pose at release, plus a twist
  feedforward), so each base speed change is a tick of EE error that the arm must absorb. The EE task outweighs the
  base command in the one QP, and a singular arm cannot absorb motion along its axis. The apparent dependence on world
  orientation is the travel direction relative to the straight arm. Fix to build: base-relative riding in the WBC (a
  released arm's EE target in the base frame, so base motion makes no EE error). Workaround: grip, bend that elbow,
  release.
