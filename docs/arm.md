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

| state | arm | gripper | how it gets there |
|---|---|---|---|
| connecting | — | — | start, `reset` |
| uninitialized | motors off (damping) | motors off | connected; gripper not checked yet |
| initializing | off | closing check | the first `enable` after connecting |
| **off** | motors off (damping): sinks | damping | e-stop, a fault, `stow` |
| **disabled** | floating (kp = kd = 0, gravity feedforward) | damping (motor off) | `disable`, lease released or lapsed, chock |
| **floating** | floating | free (gainless) until a gripper command makes it hold | `enable`; `float` from servoing |
| **servoing** | closed loop | as commanded | the first closed-loop arm command (track, servo, chunk, home, stow) |
| fault | off | off | read failure, motor fault code, late tick, encoder mismatch, refused setpoint; `reset` reconnects |

- **Enable** (lease; refused while the arm's chock is held or the e-stop is latched): motors on,
  gravity compensation on, the arm floating. The first enable after connecting closes the gripper
  and checks it reads near zero (up to 2 tries). Refused (409) when an output encoder disagrees
  with its motor by more than 0.05 rad (a 2π wrap or slipped zero); while the motors are on, a
  disagreement over 0.15 rad faults.
- **Float** (`POST /float`): an enabled state for the arm only; servoing stops, the gripper keeps
  doing what it does. **Disable** is different: commands are refused until enable, and the gripper
  goes into damping.
- **E-stop** cuts everything to damping and latches; anyone may press it, the holder clears it.
  The arm joins E-STOP ALL.
- **Stow** (`POST /stow`): safe homing, then off at home; at once if every joint is already
  within 3° of home. The normal way to turn gravity compensation off.

**Where the arm is** comes only from the readings. Whenever it is not servoing it may be moving
(by hand, by inertia, sinking), so there is no commanded pose then. Engaging starts the command
at the motor-side positions (the MIT loops run on those) with the measured velocity; IK seeds and
plans start from the measured pose; a plan is refused if the arm moved while it was computed.
Engaging is refused while a joint is more than 3° outside its limits (move it by hand: it
floats); within that the joint is held where it is and only brought in by a motion.

## Control loop (100 Hz, one thread, sole owner of the bus)

Each tick:

1. Read output encoders (xout) for all joints, plus the feedback frames (motor-side position,
   velocity, torque). A missing reading is a **fault**, never a 0.
2. Servoing: the source in force (hold / track / planned motion) gives a goal and an MIT velocity
   reference. The goal is literal: `q_cmd` moves toward it at most the joint speed limit per tick
   (the step limit) and never past a joint limit. **Acceleration limit**: an *increase* of a joint's
   commanded speed is limited to `joint_acc`·10 ms per tick; slowing down never is, and a reversal
   passes zero freely, only its part past zero is limited. Which joints were held back is streamed
   per joint (`j<i>_acc_lim`). `joint_acc` is configurable (`POST /limits`, within the hard caps).
3. Floating: kp = kd = 0, no position or velocity reference.
4. Gravity, whenever the motors are on: `tau = g(q_out) · tau_scale` (profile; 0.84 on joints 4, 5).
5. One MIT frame per motor; the gripper's own command (below) unless it is in damping.
6. Publish one state row.

A tick later than 60 ms, or 90 ms since the last frame (the motors' watchdog is 100 ms), is a
fault. Planning, IK checks and kinematics lookups run in a separate planner process (one job at a
time): in the node's own interpreter they starve the control loop.

## Motion layers

Exactly one source owns `q_goal`. A new command of any layer replaces the one in force (the old
one ends as `preempted`, logged in events). With none, the arm **holds** `q_cmd`.

### Track — low-level, streamed (`POST /track`)

The rapid, stateless primitive. Each call: a TCP pose (arm base frame) and optionally a TCP twist
and a gripper command. There is no "done": clients close their own loop on the streamed errors.

- IK is solved in the request, warm-started from the measured pose (≤ 50 iterations). On failure the call answers 200 with `ok: false` and the residuals and the previous goal stays in force: never a partial
  solution.
- The pose never times out: the last accepted pose is held indefinitely.
- The pose is the goal, literally. A goal more than 20° (any joint) from where the arm is (measured) is
  refused (track is for small steps, servo for large moves); an older `seq` than one already
  applied is ignored.
- The twist is the TCP's velocity, by default in the arm base frame's axes (`twist_frame: "base"`, the
  frame poses are given in), or in the TCP's own axes (`"tcp"`). It maps to
  `qd_ff = J⁺(q_goal)·twist` (damped pseudo-inverse), sent as the MIT velocity reference for
  0.25 s after the call, then zero, so a client that stops calling ends in a hold. The twist is
  capped at the soft servo speeds and `qd_ff` at the joint speed limits.
- Response: `{ok, seq, ik: {ok, pos_err_mm, rot_err_deg}, limited}`. `limited` means the step
  limit is clipping this tick, so `q_cmd` lags `q_goal`.

### Servo — go to a pose (`POST /servo`)

The high-level move, a refined end-pose controller. "Reached" only makes sense here.

1. **Plan** (planner process): IK for the goal from the measured pose; an SE(3) path from the
   measured TCP to the goal, timed by a velocity/acceleration profile (per request: linear and
   angular `max_vel` and `max_acc`, defaulting to the node's limits and never above them; or a
   requested duration, refused if it breaks them); CLIK-track it into a joint trajectory.
2. **Validate the whole path before moving**: CLIK residual at every sample, joint limits,
   joint speeds within `v_max`, first sample at the measured pose. Any failure means the
   request is refused with the reason and the sample index; nothing moves.
3. **Execute** in the loop, indexed by monotonic time (not a sleeping thread), with `qd_ff` from
   the trajectory's derivative.
4. **Settle**: done when the output-side TCP error is under the tolerance (default 2 mm, 1°) for
   0.2 s; `not_reached` if that hasn't happened by the trajectory's end + a settle timeout.
5. **Abort** on: output-side
   tracking error over a threshold (an obstruction or collision), cancel, preemption, disable,
   e-stop, lease end. An obstruction abort holds where the arm actually is (releasing the force);
   cancel holds where the command is.
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

`home` is the recovery routine, for an arm that sank after an e-stop or a fault:

1. Enable (the arm floats where it lies, even on its own stops).
2. Joint-space min-jerk to the `home` pose at a low speed (default 0.5 rad/s, below `v_max`),
   optionally through intermediate named poses listed in config (for example: lift the shoulder
   before swinging joint 1), so it doesn't sweep through the base.
3. Aborts and holds on a tracking error over the threshold (something in the way); reports
   `reached` like servo.

### Gripper (part of any command, or `POST /gripper`)

- Command: `gap_mm` (jaw gap; 0 = closed, range up to fully open) and optional `max_force_n`
  (default 50 N when omitted). The gap goes straight to the MIT position command with velocity
  reference 0, so the damping term never adds grip force. No ramp.
- Free (gainless) after enable until a gripper command; damping when disabled or off.
- Geometry: module 1, 16 teeth, pitch radius 8 mm; one pinion drives both racks, so
  `gap_mm = 16 · |θ|` (θ in rad, closed 0, opening negative) and squeeze force
  `F = τ / (2 · 0.008 m)`.
- Force limit, every tick: `τ_est = kp·(θ_cmd − θ) − kd·θ̇` from motor-side feedback; if
  `F(τ_est)` exceeds `max_force_n` in the closing direction, a feedforward `τ_ff` opposes the
  excess. `|τ_ff| ≤ |τ_est|` and it only ever opposes closing, so it can never drive the jaws
  open. Within 5 mm of fully open there is no feedforward (protecting the printed stops); there
  the position command is pulled back toward the jaws instead, so wide objects are force-limited
  too. `max_force_n ≥ 0`.

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
  while the arm is not servoing.
- `GET/POST /profile`: gain profiles from yaml (`soft` production, `safe` debug). Switching only
  while nothing is servoing.
- `GET/POST /limits`: joint speed caps (the step limit; defaults 2 / 6 rad/s, below the teleop's
  6 / 18), joint acceleration caps (increases only; 10 / 30 rad/s²), TCP speed and acceleration
  for servo, joint speed for home. Settable within the hard limits; reset when the lease ends.
- Lease, enable, disable, float, stow, estop, estop/clear, reset (see States).

## State: one row per tick, batched

`/stream?max_hz=` (rows, thinned like swerve), `/fields` (column names and units), `/state`
(latest row + status), `/events` (SSE). One row holds everything; per-part reads (`/joints`,
`/ee`, `/tracking`, `/gains`, `/gripper`) are conveniences over the same snapshot.

| group | columns |
|---|---|
| status | t, state, enabled, motors_on, servo, g_servo, estop, lease_id, source, seq, overruns, `limited` (speed) |
| joints | q (output), q_motor, qd, tau_meas, q_cmd, qd_ff (MIT velocity reference), tau_g, `acc_lim` (per joint), temperature, status |
| tracking | motor-side e, ė, τ_est = kp·e + kd·ė + τ_g; output-side TCP error (mm, deg) vs command and vs goal; `limited` |
| TCP | pose from the output encoders (xyz + quaternion) |
| gripper | gap, gap sent (after the force limit), goal, τ_est, force, τ_ff, force limit active, status |
| layers | servo {...}, chunk {...} |
| gains | profile name (full gains on `/gains`, they change only on a switch) |

## Simulation

`--sim`: the same node with a simulated bus (each joint follows its MIT command as a
second-order system with a little friction; optionally gravity, so floating, sinking in damping
and a wrong gravity feedforward show; the driver's refusal of setpoints outside the limits; the
gripper stops at 0 mm or an "object" gap). Needed by the preview harness
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
  twist while held; release stops sending and the twist times out; the arm holds):
  - xy stick (TCP x/y in the arm base frame), vertical slider for z;
  - pitch/yaw stick and horizontal roll slider (rotation about the TCP's axes).
- **Servo**: target x, y, z, roll, pitch, yaw prefilled from the measured TCP pose (so Go with no
  edits holds still), linear/angular max velocity and acceleration, Go / Cancel, phase and
  progress; named poses as buttons.
- **Gripper**: gap (mm) and max force (N).
- **Manipulability**: the translational velocity ellipsoid drawn at the TCP (axes from the
  Jacobian's singular vectors, scaled by its singular values), plus w and σ_min readouts that
  warn near a singularity.
- **Status and recovery**: lease, enable / float / disable / stow, e-stop and clear, home,
  profile, TCP offset, limits, tracking errors.
