# Arm node — design draft

Status: implemented (sim-tested; not yet run on the hardware). The hardware backend for the reBot Arm
B601-DM (6 Damiao joints + gripper on the dm-serial bridge), in the shape of the swerve node:
one process owns the bus, requests never touch it, a lease gates motion, e-stop is for anyone.
Teleop and any other "what should the arm do" logic stay in clients.

**Frames:** every pose, twist and reading is in the arm's own base frame (its URDF `base_link`).
The node knows nothing of the robot body or the world; whole-body coordination (base + arm) is
out of scope here and belongs to clients.

Built on `rebot_ctl` (submodule, pinned): `RebotArm` for the bus, pinocchio model, `solve_ik`,
`compute_generalized_gravity`, the geodesic planner and CLIK tracker.

## Scope

In: gravity-compensated Cartesian control of the TCP in three layers (track, servo, chunk),
the gripper in millimetres with a force limit, IK checks, gain profiles, state streaming.

Out, by design: joint-space commands of any kind (no `qpos` writes; joint positions exist only
as readings, and as named poses an admin defines in config), POS_VEL mode, teleop logic, zeroing and
calibration (stay as `rebot_ctl` CLI tools), payload identification.

## States

```
connecting ─► uninitialized ─(enable)─► initializing ─► enabled ─(disable)─► disabled
                    ▲                        │ check fails                     │
                    └──────── fault ◄────────┘◄──── read loss / watchdog ◄─────┘
estop: latched flag over any state; motors disabled until the holder clears it
```

- **connecting**: open the bridge, verify every motor's CAN watchdog (100 ms), check joint
  readings against limits (2π wrap check). Motors stay disabled.
- **uninitialized**: connected, gripper not checked. Reads work; no torque.
- **initializing** (first `enable` after connect): clear faults, close the gripper and check it
  reads near zero (up to 2 tries, about 0.5 s each). Later enables skip it while the connection
  lasts.
- **enabled**: the arm holds the pose it was enabled at, gravity compensated; commands move it.
  Enable holds the *motor-side* positions (the motors' MIT loops run on those, so the first frame
  asks for no correction), clamped into the joint limits, and the driver's remembered setpoints
  are forgotten. It is refused when an output encoder disagrees with its motor by more than
  0.05 rad (a 2π wrap or slipped zero), or a joint is more than 3° outside its limits (clamping
  would snap it back: move it by hand). While running, a disagreement over 0.15 rad faults.
- **disabled**: motors off, no torque. The arm falls under gravity. `disable` and e-stop both
  land here.
- **fault**: a motor read failed, a motor reported an error, or the control loop overran.
  Motors disabled; `reset` (holder) reconnects and ends in `uninitialized`.

Enable needs the lease and refuses while the arm's own lock is held. The arm joins E-STOP ALL.
A lease that ends or lapses disables the arm (as on swerve), so a crashed client leaves it
limp; `home` (below) brings it back without anyone repositioning it by hand.

## Control loop (100 Hz, one thread, sole owner of the bus)

Each tick:

1. Read output encoders (xout) for all joints, plus the feedback frames (motor-side position,
   velocity, torque). A missing reading is a **fault**, never a 0.
2. Pick the motion source in force (hold / track / servo / chunk) and get its joint goal
   `q_goal` and joint velocity feedforward `qd_ff` for this tick.
3. Command step (every source goes through it): `q_cmd` moves toward `q_goal` with per-joint
   speed (`v_max`) and acceleration (`a_max`) limits, braking so it stops at the goal, the goal's
   own velocity (trajectory, twist) fed forward; never outside the joint limits. The MIT velocity
   reference is `q_cmd`'s own velocity, so no source change, cancel or preemption steps the
   commanded velocity, and a velocity reference can't drag the arm off its position command.
4. Gravity: `tau = g(q_out) · tau_scale` (profile; 0.84 on joints 4 and 5).
5. `send_mit(q_cmd, qd_ff, kp, kd, tau)` for the arm; the gripper's own command (below).
6. Publish one state row.

Tick overruns are counted and published; a tick later than 60 ms (watchdog is 100 ms) is a fault.
IK for track runs in the request; servo and chunk planning run in a worker thread, never in the
loop. Being its own process keeps other nodes out of the way; inside it, the loop shares one
interpreter with the HTTP server and those workers, so each can delay a tick by about one
interpreter switch (5 ms). The overrun counter shows whether that ever matters; swerve runs the
same way.

## Motion layers

Exactly one source owns `q_goal`. A new command of any layer replaces the one in force (the old
one ends as `preempted`, logged in events). With none, the arm **holds** `q_cmd`.

### Track — low-level, streamed (`POST /track`)

The rapid, stateless primitive. Each call: a TCP pose (arm base frame) and optionally a TCP twist
and a gripper command. There is no "done": clients close their own loop on the streamed errors.

- IK is solved in the request, warm-started from the previous track goal (else `q_cmd`; ≤ 50
  iterations). On failure the call answers 200 with `ok: false` and the residuals and the previous goal stays in force: never a partial
  solution.
- The pose never times out: the last accepted pose is held indefinitely.
- The pose was the goal when sent; a fresh twist moves the goal on from it (at most 0.25 s
  worth), and is capped at the soft servo speeds. A goal more than 20° (any joint) from the
  previous one is refused: track is for small steps, servo for large moves.
- The twist is the TCP's velocity, by default in the arm base frame's axes (`twist_frame: "base"`, the
  frame poses are given in), or in the TCP's own axes (`"tcp"`). It maps to
  `qd_ff = J⁺(q_cmd)·twist`. It is used for 0.25 s after the call,
  then zero, so a client that stops calling ends in a gravity-compensated hold.
- Response: `{ok, seq, ik: {ok, pos_err_mm, rot_err_deg}, limited}`. `limited` means the step
  limit is clipping this tick, so `q_cmd` lags `q_goal`.

### Servo — go to a pose (`POST /servo`)

The high-level move, a refined end-pose controller. "Reached" only makes sense here.

1. **Plan** (worker): IK for the goal from `q_cmd`; an SE(3) geodesic from the current
   commanded TCP to the goal, timed by a velocity/acceleration profile (per request: linear and
   angular `max_vel` and `max_acc`, defaulting to the node's limits and never above them; or a
   requested duration, refused if it breaks them); CLIK-track it into a joint trajectory.
2. **Validate the whole path before moving**: CLIK residual at every sample, joint limits,
   joint speeds within `v_max`, first sample continuous with `q_cmd`. Any failure means the
   request is refused with the reason and the sample index; nothing moves.
3. **Execute** in the loop, indexed by monotonic time (not a sleeping thread), with `qd_ff` from
   the trajectory's derivative.
4. **Settle**: done when the output-side TCP error is under the tolerance (default 2 mm, 1°) for
   0.2 s; `not_reached` if that hasn't happened by the trajectory's end + a settle timeout.
5. **Abort** on: output-side
   tracking error over a threshold (an obstruction or collision), cancel, preemption, disable,
   e-stop, lease end. An obstruction abort holds where the arm actually is (releasing the force);
   cancel and preemption decelerate at the acceleration limit to a stop.
6. Plans run one at a time. A servo / home planned while the command kept moving (a track or
   motion in force) is refused rather than started from a stale pose; a chunk's lead-in is
   re-anchored on the command when it is accepted and speed-checked.

Observable: `servo {id, phase: planning|executing|settling|reached|not_reached|aborted|refused,
progress, t, duration, reason}` in every state row.

### Chunk — action chunks from a policy (`POST /chunk`)

Time-indexed TCP poses (+ gripper per step) at a fixed `dt`, the output of a chunking policy.

- `t0`: when sample 0 is meant to happen, on the node's clock (common/timesync gives clients the
  offset). Samples already in the past are skipped, so observation-to-action latency doesn't
  replay stale actions.
- Validated as a whole before it is accepted: IK for every sample warm-started along the
  sequence, joint limits, joint speeds within `v_max`. A chunk that fails anywhere is refused.
- A new chunk replaces the one in force from its first future sample, blended over `blend_s`
  (default 0.1 s) to avoid a step.
- After the last sample: hold (no extrapolation).
- Observable: `chunk {id, index, n, lag_ms, phase, reason}`.

### Home and named poses (`POST /home`, `POST /servo {pose_name}`)

Named poses are joint configurations defined in config (`home`, `stow`, ...; the packaged
default is only `home` = all zeros, as rebot_ctl's `safe_home`; an admin adds more in
`~/.config/robot_utils/arm.yaml`, see nodes/arm/config.py): the one place joint
positions are inputs, and only an admin writes them. Servo accepts a name in place of a pose
(planned in joint space, same validation, observability and aborts).

`home` is the recovery routine, for an arm that fell when it was disabled or that a lapsed lease
left wherever it was:

1. Enable holds the measured pose (wherever the arm lies, even on its own stops).
2. Joint-space min-jerk to the `home` pose at a low speed (default 0.5 rad/s, below `v_max`),
   optionally through intermediate named poses listed in config (for example: lift the shoulder
   before swinging joint 1), so it doesn't sweep through the base.
3. Aborts and holds on a tracking error over the threshold (something in the way); reports
   `reached` like servo.

### Gripper (part of any command, or `POST /gripper`)

- Command: `gap_mm` (jaw gap; 0 = closed) and optional `max_force_n` (50 N after enable; an
  omitted limit keeps the one in force). The gap command is rate and acceleration limited and
  stops 1 mm short of fully open (the printed stops); no velocity reference is sent, so kd only
  damps.
- Geometry: module 1, 16 teeth, pitch radius 8 mm; one pinion drives both racks, so
  `gap_mm = 16 · |θ|` (θ in rad, closed 0, opening negative) and squeeze force
  `F = τ / (2 · 0.008 m)`.
- Force limit, every tick: `τ_est = kp·(θ_cmd − θ) + kd·(θ̇_cmd − θ̇)` from motor-side
  feedback; if `F(τ_est)` exceeds `max_force_n` in the closing direction, a feedforward
  `τ_ff` opposes the excess. `|τ_ff| ≤ |τ_est|` and it only ever opposes closing, so it can never
  drive the jaws open. Within 5 mm of fully open there is no feedforward (protecting the printed
  stops); there the position command is pulled back toward the jaws instead, so wide objects are
  force-limited too.
  `max_force_n ≥ 0`.
- Its own step limit (mm/s).

## Kinematics: Jacobian and manipulability

Available three ways:

- **Streamed** every tick (at the measured pose): manipulability `w = sqrt(det(J·Jᵀ))` of the
  full 6×6 TCP Jacobian, the translational `w_lin` (from its top 3 rows), and the smallest
  singular value `σ_min` (distance to a singularity), as stream columns. The full Jacobian is
  36 numbers, so it streams on its own channel (`/stream?channel=jacobian`) for clients that want it.
- **Current pose** (`GET /kinematics`): TCP pose, J (`frame: base | tcp`, as for the twist),
  w, w_lin, σ_min, the singular values and the principal axes (for the ellipsoid).
- **Arbitrary lookups** (`POST /kinematics`): a batch of joint configurations in, the same
  quantities out for each (plus within-limits), for client-side planning. Joint positions here are
  inputs to a computation only: nothing moves, and no lease is needed. Capped batch size.

## Other APIs

- `POST /ik`: IK check only. Pose + seed (`command` | `measured`) → `{ok, q, pos_err_mm,
  rot_err_deg, within_limits}`. Never moves anything.
- `GET/POST /tcp`: the TCP offset in `end_link` (default −0.15 m along x, as in teleop). Only
  while disabled.
- `GET/POST /profile`: gain profiles from yaml (`soft` production, `safe` debug). Switching only
  while disabled.
- `GET/POST /limits`: joint speed caps `v_max` (defaults below the teleop's 6 / 18 rad/s), TCP
  speed caps for servo, gripper speed. Settable within the hard limits; reset when the lease ends.
- Lease, enable, disable, estop, estop/clear, reset: as swerve.

## State: one row per tick, batched

`/stream?max_hz=` (rows, thinned like swerve), `/fields` (column names and units), `/state`
(latest row + status), `/events` (SSE). One row holds everything; per-part reads (`/joints`,
`/ee`, `/tracking`, `/gains`, `/gripper`) are conveniences over the same snapshot.

| group | columns |
|---|---|
| status | t, state, estop, lease_id, source (hold/track/servo/chunk), seq, loop overruns |
| joints | q (output), q_motor, qd, tau_meas, q_cmd, qd_ff, tau_g |
| tracking | motor-side e, ė, τ_est = kp·e + kd·ė + τ_g; output-side TCP error (mm, deg) vs command and vs goal; `limited` |
| TCP | pose from the output encoders (xyz + quaternion) |
| gripper | gap_mm, gap_cmd_mm, τ_est, force_n, τ_ff, force limit active |
| layers | servo {...}, chunk {...} |
| gains | profile name (full gains on `/gains`, they change only on a switch) |

## Simulation

`--sim`: the same node with a simulated bus (each joint follows its MIT command as a stiff
second-order system, the gripper stops at 0 mm or an "object" gap). Needed by the preview harness
and the tests; nothing in the tests touches hardware.

## Config changes

- `rebotarm_dm_safe.yaml`: gripper kp 8.0 → 0.5.
- Both profiles: `tau_scale [1, 1, 1, 0.84, 0.84, 1]`.
- Joint limits independent of the profile: joint 1's lower limit −36° in both (safe has −39°
  today).
- `named_poses:` (joint degrees) with `home`, and `home_via:` (optional list of names).

## Companion page

Served by the node at `/nodes/arm/`, built on the shared toolkit and `RU.viz` (wireframe, like
the other pages).

- **Visualisation, always on**: the arm from the streamed joint positions (output encoders), with
  the gripper drawn at its measured gap; the commanded pose and the servo goal as ghost frames.
- **Jog**: four controls, hold-to-move, each a track client (the page streams `track` with a
  twist while held; release stops sending and the twist times out into a hold):
  - xy stick (TCP x/y in the arm base frame), vertical slider for z;
  - pitch/yaw stick and horizontal roll slider (rotation about the TCP's axes).
- **Servo**: target x, y, z, roll, pitch, yaw prefilled from the measured TCP pose (so Go with no
  edits holds still), linear/angular max velocity and acceleration, Go / Cancel, phase and
  progress; named poses as buttons.
- **Gripper**: gap (mm) and max force (N).
- **Manipulability**: the translational velocity ellipsoid drawn at the TCP (axes from the
  Jacobian's singular vectors, scaled by its singular values), plus w and σ_min readouts that
  warn near a singularity.
- **Status and recovery**: lease, enable/disable, e-stop and clear, home, profile, TCP offset,
  speed limits, tracking errors.
