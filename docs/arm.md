# Arm node — design draft

Status: implemented (sim-tested; not yet run on the hardware). The hardware backend for the reBot Arm
B601-DM (6 Damiao joints + gripper on the dm-serial bridge), in the shape of the swerve node:
one process owns the bus, requests never touch it, a lease gates motion, e-stop is for anyone.
Teleop and any other "what should the arm do" logic stay in clients.

**Frames:** every pose, twist and reading is in the **arm frame** (`arm`: the arm's URDF `base_link`, the
underside of its mount plate on joint 1's axis, axes parallel to the robot base). Its pose in the robot base frame
comes from the description node's `arm` frame; `GET`/`POST /geometry` report it (`frames.arm_in_base`) and give
their points in both frames. Commands stay in the arm frame; whole-body coordination (base + arm) belongs to
clients. (`base` was the old name of the arm frame in `twist_frame` and the Jacobian `frame`; it is still
accepted and means `arm`.)

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
| fault | off | off | read failure, motor fault code, late tick, refused setpoint; `reset` reconnects |

- **Connect** only opens the bus and checks (watchdog settings, joint readings for 2π wraps), then
  disables all motors; no mode change.
- **Enable** (lease; refused while the arm's chock is held or the e-stop is latched): faults
  cleared, MIT mode set and confirmed, motors on, gravity compensation on, the arm floating. The first enable after connecting closes the gripper
  and checks it reads near zero (up to 2 tries). Before the motors go on, each joint's output
  encoder (used for kinematics, IK and plans) and its motor's feedback position (what the MIT loop
  servos on) must agree within 0.05 rad, or enable is refused (409): otherwise every command would
  land off by the difference.
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

**Robustness.** Replies are awaited at most 5 ms per tick. A missed reply (an ordinary event on a
serial bus) keeps the latest complete reading for that tick; the node faults only when that
reading is older than 50 ms with the motors on (1 s with them off). Misses are counted, per motor,
in `/state` and the stream (`read_age`: 0 when this tick's read completed, else the kept reading's age; `read_misses`). A gap of 30 ms or more with the motors on is
a warning event (at most one every 5 s, counting the gaps in between), so a fault has warnings
before it. The bus also times each tick's replies (requests to the last reply): `/state`
`reply_latency` has p50, p99 and max over the last 500 complete reads and the margin to the 10 ms
tick (a reply later than one tick is waited for one more; only one missing after two ticks counts as
missed, and the waiting tick reuses the latest complete reading). The first 300 complete reads after
connecting are reported as one event (motors off then, so lighter than in use), and a margin under
1 ms is a warning event, at most one a minute. Each tick's requests are two per motor while the
motors are on: the output encoder (xout, read every tick: the planetary gears' backlash makes it the
true joint position) and the MIT frame, which a Damiao motor answers with its feedback frame. A
separate feedback request is sent only to a motor that got no MIT frame the tick before (motors
off, the gripper in damping, the first read after connecting or enabling), or as a fallback when
neither of its last two MIT frames was answered (about two ticks without feedback): `/state`
`feedback_refresh` counts both (`fallbacks` stays 0 on firmware that answers MIT frames; the first
on a connection is a warning event). A motorbridge without the feedback's age (`get_state_with_age`)
requests every motor's feedback every tick, as before (logged once). On 2026-11-13 a full-resolution look in
agent_interface (1280x720 colour) stretched the replies until gaps passed 50 ms and the arm faulted;
see docs/realtime.md. A late tick is reported (an event, the
`late_ticks` count), not a fault: the motors' own 100 ms watchdog protects them, and if it trips
they report "communication lost", which faults. Setup steps that block (mode switches, the gripper
check) are only ever run with every motor off; the bus refuses them otherwise. The lease
heartbeat is 3 s (a browser shares 6 connections across all its tabs on the robot). Planning, IK checks and kinematics lookups run in a separate planner process (one job at a
time): in the node's own interpreter they starve the control loop.

### Processes

The control loop and the bus run **in their own process** (`--control process`, the default; `--control thread`
keeps the old structure: the loop a SCHED_FIFO thread of the server process). Measured on the Pi (2026-11-22, the
full stack plus a recorder, ~93% CPU busy) with the loop as a thread: it shared the process's GIL with uvicorn, SSE,
the stream clients and the request threads, and while a normal-priority thread holding the GIL was descheduled the
real-time tick waited for it (priority inversion): `loop_hz` 64–87, 1,706 overruns in ~4 min, reply latency p50
9.5 ms / p99 20.5 ms (5–7 ms quiet), feedback-refresh fallbacks, a 793 ms late tick during an enable. The WBC had the
same structure and symptoms; its own process fixed them (docs/wbc.md "Processes").

- **Control process** (`arm/control_proc.py`, spawned by the node at start; SCHED_FIFO 40 on its one thread):
  `ControlCore`, the very same `ArmNode` with its surroundings swapped. It owns the bus (`RebotArm`, the serial port;
  or `SimBus` / `WorldArmBus`), connects and checks it, runs the tick (read, chocks, cut / engage, servo / track /
  stream / hold, the gripper loop, jam detection, faults), the enable (the gripper check, the MIT mode switches) and
  the **joint channel** (`<name>.joint.sock`: no receiver thread; the loop waits for its tick on the command socket
  and the joint socket together, handles each datagram as it arrives, and polls once more right before the command
  for goals that arrived during the bus read). No HTTP, no planning, no other Python threads; `gc.freeze()` after
  the setup.
- **Server process** (`arm/proc_node.py`, `ProcNode`: the same request API as `ArmNode`, server.py serves either):
  HTTP and the page, SSE, `/stream` (the tick and Jacobian rows from two rings), the **lease** (its holder and a
  digest of its token published to the control process for the joint channel's hello; renewed for every
  joint-channel datagram the control process counts; its end sent as `lease_ended`), the e-stop latch, the chocks'
  check at enable, the **track IK and the planning** (`arm/requests.py` `ArmRequests`, the same code as in-process,
  on the latest snapshot; the planner process is this process's), the lookups (kinematics, geometry, IK checks:
  this side's model, kept at the control process's TCP offset). Everything that changes the control state is a
  command answered by the control process: enable, disable, float, the solved track, the planned motion (re-checked
  there: enabled, the drift since planning), gripper, cancel, reset, stow, profile, TCP, limits, jam, the e-stop.
- **IPC** (`common/ipc.py`, shared with the WBC): one shared-memory file (a private 0700 directory on tmpfs,
  `$XDG_RUNTIME_DIR` on the robot; every page allocated at creation, so the loop never page-faults on it): heartbeats,
  a seqlocked, CRC-checked snapshot slot (`ArmNode.core_state` plus the reading, the command, the soft limits, the
  setpoint limits; every 2nd tick and after every command, with the events it explains sent after it) and the lease
  slot; drop-oldest rings for the rows. Commands on two Unix datagram sockets (pickled), the e-stop first; a command
  over 32 KB (a long planned motion: Linux caps a datagram at 208 KiB) goes through a bulk slot, one at a time. The server
  is **event-driven**: the control process sends a `t` message after every tick and the server's one IPC thread
  publishes the rows, renews the lease and checks the heartbeat then (no polling of shared memory faster than the
  loop: the WBC's 500 Hz pump cost CPU and GIL hand-offs).
- **The e-stop** goes out at once (the HTTP handler never waits; the confirmation is awaited in the background) and
  the control process switches the motors off **on receipt**, between ticks, not at the next tick. A stow's motors-off
  likewise.
- **Blocking steps.** Connecting (the bus's checks), enabling (the gripper check, the mode switches) and closing the
  bus block the control loop as before (no rows, no joint-channel state packets meanwhile: ~1 s for the first
  enable). The control process announces each (`/state.process.busy`) with a deadline (connect 30 s, enable 10 s);
  its heartbeat is excused until then. The server stays responsive throughout.
- **Failures.** The control process watches the server's heartbeat: missing for 0.5 s → it **holds** (servoing: at
  its command; the planned motion, track and stream dropped, an event; the gripper keeps its goal) and accepts no new
  motion (joint-channel goals dropped as `server`) until the heartbeat is back (then accepted again). The server's
  process gone → the joint channel's bye, the motors off (damping), the process ends. The server watches the control
  process's heartbeat: missing for 0.25 s outside an announced blocking step, or the process gone → **fault**
  (latched; an event; `/state` `state: fault`, `error` the reason, the motors reported off; every request needing
  the control process answers 503 "restart the node"; the e-stop still latches). The motors go to damping by their
  own 100 ms CAN watchdog; a control process that was only stalled is told to fault (motors off) and, still silent
  1 s later, killed (the serial port released). **Nothing restarts automatically: the operator restarts the node.**
  Node shutdown stops it cleanly (bye, motors off, port released).
- `/state.process` (added; also in thread mode, `mode: thread`): `mode`, `pid`, `alive`, `fault`, `heartbeat_age`,
  `snapshot_age` (≤ 20 ms), `busy`, `rows_dropped`, `commands_dropped`, `service_max_ms`, and `tick_time`: per tick
  over the last 500, wall time, the control thread's CPU time and the rest ("off CPU": waiting for the GIL, for the
  bus, or preempted), p50 / p99 / max. Everything else is unchanged for clients: the routes, the stream fields, the
  states, the joint-channel protocol; `/state` is built from the snapshot (≤ 20 ms old; a track's `age` is brought
  to now).
- **Measured** (`tests/test_arm_proc.py`, MacBook, no SCHED_FIFO; the server hammered by 6 `/state` and 2
  `/kinematics` pollers, 3 SSE and 3 `/stream` clients and three threads of Python work every 10 ms; a joint-channel
  client in its own process at ~97 Hz): process vs thread, tick period p99 11.7 vs 12.4–12.9 ms, max 11.8–12.1 vs
  13.7–18.2 ms; off-CPU per tick p99 0.05 vs 0.9 ms, max 0.1 vs 1.4–12.9 ms (the thread's GIL waits); joint channel
  goal → state packet p50 15.0 vs 15.4 ms, p99 20.9 vs 21.2 ms; the server answered ~60% more `/state` requests. With
  every core also busy (normal-priority burners: the robot's situation, but here nothing is real time, so both are
  preempted): tick period p99 12.1 vs 14.7 ms, max 22 vs 38 ms, overruns 4 vs 10, joint channel p99 22 vs 28 ms.
  On the robot, compare `/state` `loop_hz`, `overruns`, `process.tick_time` and `reply_latency` between the modes.
- **The bus's own reader thread.** motorbridge (the native `libmotor_abi`) starts a polling worker thread of its own
  when the bus opens (`CoreController::start_polling_if_needed`; on the robot the one normal thread that appears
  ~30 ms after the control thread starts, ~4% CPU, mostly system time). It holds no GIL, so the split does not
  touch it, but created by the real-time control thread it starts as a normal thread (SCHED_RESET_ON_FORK): under
  load it is scheduled late (replies are then stamped when the control thread's own `poll_feedback_once` drains
  them, so the reply latency reads about a tick), and the control thread may wait for a lock it holds (an inference
  from thread listings, not verified). So at every connect (a reconnect starts a new one) the node lists its thread
  ids (`/proc/self/task`) before the bus is made and after it connected, takes the new ones that are not Python
  threads, and gives the one it finds SCHED_FIFO 39, just below the loop's 40 (`common/realtime.py`
  `raise_native_threads`). An event says what it found and set (tid, name, policy); `/state.realtime.bus_threads`
  shows it. None or several new threads, or the change refused (RLIMIT_RTPRIO): a warning event and nothing changed;
  the connect goes on regardless. Nothing on macOS (no `/proc`) or with a simulated bus; `ROBOT_UTILS_RT=0` turns it
  off with the loop's own real time. Both modes (process: in the control process; thread: in the node).

## Motion layers

Exactly one source owns `q_goal`. A new command of any layer replaces the one in force (the old
one ends as `preempted`, logged in events). With none, the arm **holds** `q_cmd`.

### Track — low-level, streamed (`POST /track`)

The rapid, stateless primitive. Each call: a TCP pose (arm base frame) and optionally a TCP twist
and a gripper command. There is no "done": clients close their own loop on the streamed errors.

- IK is solved in the request, warm-started from the measured pose (≤ 30 iterations). On failure the call answers 200 with `ok: false` and the residuals and the previous goal stays in force: never a partial
  solution.
- The pose never times out: the last accepted pose is held indefinitely.
- The pose is the goal, literally. A goal more than 20° (any joint) from where the arm is (measured) is
  refused (track is for small steps, servo for large moves); an older `seq` than one already
  applied is ignored.
- The twist is the TCP's velocity, by default in the arm frame's axes (`twist_frame: "arm"`, the
  frame poses are given in), or in the TCP's own axes (`"tcp"`). It maps to
  `qd_ff = J⁺(q_goal)·twist` (damped pseudo-inverse), sent as the MIT velocity reference for
  0.25 s after the call, then zero, so a client that stops calling ends in a hold. The twist is
  capped at the soft servo speeds and `qd_ff` at the joint speed limits.
- Response (`TrackOut` in `/docs`): `{ok, seq, ik, limited, twist_scaled, reason}`: `ik` is the full IK
  result (as `POST /ik`; null for a stale `seq`, which also omits `twist_scaled`), `reason` why not ok.
  `limited` means the step limit is clipping this tick, so `q_cmd` lags `q_goal`.

### Servo — go to a pose (`POST /servo`)

The high-level move: rebot_ctl's end-pose controller (`RebotArmEndPose.move_to_traj`), as tested
on the arm. "Reached" only makes sense here.

1. **Plan** (planner process), as the end-pose controller: IK for the goal from the measured pose
   (200 iterations, tolerance 1e-4, rotation weight 0.02); a min-jerk SE(3) geodesic from the
   measured TCP to the IK solution's pose; `track_trajectory` (CLIK with null-space joint-limit
   avoidance, gain 0.1) into joint samples every 10 ms; finite-difference velocities, zero at the
   end. The node adds the duration: from the TCP speed and acceleration limits (per request, never
   above the node's: min-jerk peaks 1.875·d/T and 5.77·d/T², linear and angular parts of the
   geodesic), or a requested duration, refused if it breaks them.
2. **Refused before moving** if the goal is unreachable or a joint would exceed its speed limit;
   nothing moves.
3. **Execute** in the loop, indexed by monotonic time (not a sleeping thread), with the
   trajectory's velocity as the MIT velocity reference.
4. **Settle**: done when the output-side TCP error is under the tolerance (default 2 mm, 1°) for
   0.2 s; `not_reached` if that hasn't happened by the trajectory's end + a settle timeout.
5. **Abort** on: output-side
   tracking error over a threshold (an obstruction or collision), cancel, preemption, disable,
   e-stop, lease end. An obstruction abort holds where the arm actually is (releasing the force);
   cancel holds where the command is.
6. Plans run one at a time. A servo / home planned while the command kept moving (a track or
   motion in force) is refused rather than started from a stale pose; a chunk's lead-in is
   re-anchored on the command when it is accepted and speed-checked.

Observable: the reply and `/state` `motion` (`MotionOut` in `/docs`): `{id, kind, name, phase:
executing|settling|reached|not_reached|done|aborted|preempted, reason, progress, duration, t_start,
t_end_planned, goal, info}`; the stream carries `motion_id`, `phase` and `progress` every tick.

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
- Observable: as servo (`motion`, kind `chunk`; it ends `done` after the last sample), with `info: {skipped,
  n, t0, dt, blend_s}`.

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
- Force limit: it bounds the **squeeze**, in contact only (`nodes/arm/gripper.py`). Free motion is the plain
  MIT law toward the goal with the profile's gains, uncapped: as fast as the raw interface.
  - Contact detection, every tick, from the motor-side feedback (position θ, velocity θ̇): the jaws are
    *stalled* (`|θ̇| < stall_vel`) while the goal is past them by more than the limit's own error
    (`kp·(θ_goal − θ) > τ_lim`, `τ_lim = max_force_n · 16 mm`) and by more than `contact_err`, for `stall_s`.
    The motor's torque reading `g_tau` is not used: under the MIT law the motor delivers what it is
    commanded whether the jaws move or not, so the reading carries no contact information.
  - In contact the law's position term is capped at `τ_lim` (the damping term `−kd·θ̇` is left as is: it
    brakes, and cancelling it would come back as a spike when the jaws stop): away from fully open a
    feedforward `τ_ff = −(kp·(θ_goal − θ) − τ_lim)` opposes the excess, `|τ_ff|` below the position term,
    so it can never drive the jaws open. Within 5 mm of fully open there is no feedforward (protecting the
    printed stops); there the position command is pulled back toward the jaws instead, so wide objects are
    limited too. At rest the squeeze is exactly `max_force_n`; a jaw moving adds or loses `kd·θ̇`.
  - Leaving contact (hysteresis): only when the command stops squeezing past the limit, i.e. the goal comes
    back to within `τ_lim/kp − release` of the jaws. A soft object pushing back or creeping does not end it;
    an object slipping out lets the jaws close at `τ_lim` until near the goal, then free.
  - Re-grasp: where the jaws stalled is remembered. Closing back to within `approach` of it is capped too
    (a loosen-and-squeeze never hits the object with more than the limit); it is forgotten when the jaws
    close `release` past it (the object is gone) or open `forget` beyond it.
  - Opening is never limited: the cap only ever removes closing torque.
  - Settings (`~/.config/robot_utils/arm.yaml` `gripper:`; `config.GripperContact`), defaults from the soft
    profile (gripper kp 1.0 Nm/rad, kd 0.05; 1 mm of gap error = 0.0625 Nm = 3.9 N of squeeze):

    | key | default | why |
    |---|---|---|
    | `stall_vel` | 0.5 rad/s (8 mm/s of gap) | a free jaw driven by more than the limit passes it within a few ms; its damping there is 1.6 N |
    | `stall_ms` | 30 | 3 ticks: a command starting from rest (feedback up to 2 ticks late) is not a contact |
    | `contact_err_mm` | 0.5 | 2 N: a free jaw resting at its goal against friction is not a contact |
    | `release_mm` | 1.0 | the hysteresis: 3.9 N of demand below the limit |
    | `approach_mm` | 3.0 | the re-grasp zone |
    | `forget_mm` | 10.0 | opened this far past the contact: forgotten |

  - The impact transient, guaranteed: on first contact with an unknown object the law is the free one until
    detection, so the squeeze is at most the raw MIT law's `kp·(θ_goal − θ_contact)` (what the old raw teleop
    held permanently; e.g. 78 N for a close to 0 on a 20 mm object, 194 N at most for a full stroke), for at
    most `stall_s` + 2 ticks + the feedback delay after the jaws stop (the test harness: 40, 50, 60 ms at
    0, 1, 2 ticks of feedback delay), then
    `max_force_n`. A re-grasp within `approach` of the last contact: at most `max_force_n` throughout. Not
    bounded by any position law: the jaws' momentum at impact (same as under the raw law; only a slower
    approach would reduce it).
  - Below the gripper's breakaway friction (`max_force_n` of a few N) a jaw stopped short by friction is
    indistinguishable from one on an object: it stays capped (as before the redesign, which capped always).
- Reported (`/state` gripper `force_n`, stream `g_force`): the squeeze force applied, the MIT law
  with the position sent plus the feedforward, so at most `max_force_n` (at rest) while limiting. The demand
  before the limit is `demand_n` / `g_force_demand` (the law alone, damping included). `limiting` /
  `g_limiting`: the cap binds this tick (in contact, or closing back onto a remembered contact); free motion
  never limits, however large the demand. Both forces are model estimates (stiffness × error over an ideal
  pinion); the motor's own torque reading is `g_tau`.

## Kinematics: Jacobian and manipulability

Available three ways:

- **Streamed** every tick (at the measured pose): manipulability `w = sqrt(det(J·Jᵀ))` of the
  full 6×6 TCP Jacobian, the translational `w_lin` (from its top 3 rows), and the smallest
  singular value `σ_min` (distance to a singularity), as stream columns. The full Jacobian is
  36 numbers, so it streams on its own channel (`/stream?channel=jacobian`) for clients that want it.
- **Current pose** (`GET /kinematics`): TCP pose, J (`frame: arm | tcp`, as for the twist),
  w, w_lin, σ_min, the singular values and the principal axes (for the ellipsoid).
- **Arbitrary lookups** (`POST /kinematics`): a batch of joint configurations in, the same
  quantities out for each (plus within-limits), for client-side planning. Joint positions here are
  inputs to a computation only: nothing moves, and no lease is needed. Capped batch size.

## Geometry as numbers: fingertips, jaws, extent (`GET /geometry`, `POST /geometry`)

Read only (nothing moves, no lease). At the measured configuration (`GET`), or (`POST`) at joint angles `q` or at
the IK solution of a TCP `pose` (`ik` carries the IK result), with the jaws at the measured gap or `gap_mm`.
Every point is given in both the arm frame and the robot base frame, with the transform (`frames.arm_in_base`,
from the description node; the shipped default if it is unreachable, and `source` says which).

- `fingertips`: `left` and `right` (each jaw's tip on its inner face, mid height) and `center` (end_link: midway
  between them, with its orientation). end_link's x is the approach direction, y the jaw travel (left jaw at +y).
- `jaws`: per jaw its tip, inner (gripping) face centre and root, outer face at the tip, and `inward`.
  `jaw_shape_m`: `grasp_depth` (how far an object can sit between the jaws before the jaw carriage, ~59 mm),
  length, thickness, height at root and tip. Approximate, from the meshes.
- `tcp`: the point motion commands target, `offset_in_end_link` from the fingertip centre (default 0.15 m behind
  it, near the wrist). The offset stays: a TCP near the wrist keeps orientation corrections from swinging the
  TCP. To put the fingertips at a place, command the TCP pose that puts them there (`tcp` and `fingertips.center`
  differ by the offset along the approach axis).
- `axes` (approach, jaw travel, up), `extent_base` (bounds of the whole arm and of the gripper in the base frame,
  and how far the arm reaches past each side of the frame), `boxes` (every part as an oriented box: centre, axes,
  size).

## Other APIs

- `POST /ik`: IK check only. Pose + seed (`command` | `measured`) → `{ok, exact, q, pos_err_mm,
  rot_err_deg, within_limits, joints_at_limit, iterations, reason, nearest}`. Never moves anything.
- **Agent picture of an IK query**: `POST /ik` with `render` (an object; `{}` takes the defaults)
  also returns `image`, a PNG data URL, and `view`, the view used. `POST /ik/render` takes the same
  body and answers the PNG itself (`image/png`; the IK result in `X-IK-Ok`, `X-IK-Exact`,
  `X-IK-Pos-Err-Mm`, `X-IK-Rot-Err-Deg`). For agents (vision models) deciding how to move: a plain
  white-background wireframe with the arm at its measured pose (dark grey, the page's boxes:
  `geometry.ARM_BOXES`) and gripper gap, the IK solution as a dashed ghost arm (blue; orange when
  IK fails: then it is the nearest reachable configuration), the target TCP frame (thick x/y/z =
  red/green/blue) and the `nearest` TCP frame (dashed, joined to the target) when they differ,
  joints at a limit (red rings), the measured TCP, the base frame's labelled axes, reach rings on
  the floor (0.2 m apart), drop lines to the floor, a scale bar and annotations (ok / exact /
  failed and why, the errors, joints at a limit, target and nearest positions, the view).
  - View, orthographic, all optional: `azimuth_deg` (default 45) is the direction the camera looks
    from, about +z: 0 = from +x toward the base (+y to the right), 90 = from +y (+x to the
    left), 180 = from −x, −90 = from −y; `elevation_deg` (default 25): 0 level, 90 from straight
    above (at azimuth 0, +y right and −x up), negative from below; `zoom` (default 1, 0.25–8):
    1 fits both arms, the target and the nearest pose, more magnifies about their centre;
    `width` × `height` (default 640 × 480; 160–1600 × 120–1200 px).
  - Depth is ambiguous in one orthographic view: ask for two (e.g. azimuth 0 and 90 at
    elevation 0) or one from above.
  - Drawn with Pillow in the planner process, like IK (about 50 ms at 640 × 480 on a Mac), so it
    never delays the control loop. No lease; nothing moves. Needs a joint reading (503 before).
- `GET/POST /tcp`: the TCP offset in `end_link` (default −0.15 m along x, as in teleop). Only
  while the arm is not servoing.
- `GET /gains`, `POST /profile`: gain profiles from yaml (`soft` production, `safe` debug). Switching only
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
| status | t (tick start), state, enabled, motors_on, servo, g_servo, estop, lease_id, source, seq, overruns, `limited` (speed) |
| joints | q (output), q_motor, qd, tau_meas, q_cmd, qd_ff (MIT velocity reference), tau_g, `acc_lim` (per joint), temperature, status |
| tracking | motor-side e, ė, τ_est = kp·e + kd·ė + τ_g; output-side TCP error (mm, deg) vs command and vs goal; `limited` |
| TCP | pose from the output encoders (xyz + quaternion) |
| gripper | gap, gap sent (after the force limit), goal, τ_est, force, τ_ff, force limit active, status |
| layers | servo {...}, chunk {...} |
| gains | profile name (full gains on `/gains`, they change only on a switch) |
| measurement times | `t_meas`, `t_meas_mot` (appended; below) |
| joint channel | `stream_seq`, `stream_age`, `stream_lat` (appended last; "Internal joint channel") |

**When the data was measured.** `t_tick` is the tick's start, taken before its read, and the bus is
pipelined: a tick collects the replies to requests sent at the end of the previous tick's read. So a
row's output encoders (`q`, `g_gap`) answer requests sent about one tick before `t_tick` (two on a tick
whose replies were late and that kept the previous reading), and its motor-side feedback (`q_mot`,
`qd`, `tau`) answers the previous tick's MIT frames: newer than `q` in the same row. `t_meas` is when
the requests behind `q` were sent (a lower bound of when `q` was measured); `t_meas_mot` is when the
oldest arm motor's feedback frame arrived (every joint's feedback is at least that new; NaN with a
motorbridge without the feedback's age). Both are on `t_tick`'s clock. Latency compensation should use
these, not `t_tick`. `read_age` only says whether this tick's read completed (0) or an earlier reading
was kept (its age), not how old the data is. The simulated bus samples at the read, so both equal the
sample time there (no pipeline delay).

## Internal joint channel (robot-internal; not the public API)

> **Internal interface.** For on-robot controllers only (the whole-body controller now; an on-Pi policy or a radio
> relay later). It is not part of the public API and must not appear in an agent guide or a public page: outside
> clients drive the arm through the pose-level API or the WBC, never through joints. Design:
> [plans/fast_path_v1.md](plans/fast_path_v1.md) ("Local only"); contract: [plans/wbc_v1.md](plans/wbc_v1.md)
> ("Interfaces for the first build", 1).

**Transport.** One Unix datagram socket, `<runtime dir>/robot/<name>.joint.sock` (`arm.joint.sock`), owner-only
(0600, in the 0700 runtime directory): only processes of the robot user on the Pi reach it. No UDP, no HTTP route,
nothing through nginx or OpenAPI. Code: `nodes/common/fastpath.py` (packets, control messages, a small `Client`),
`nodes/arm/joint_channel.py` (socket and receiver thread), `nodes/arm/stream.py` (session and the `stream` source).
The client binds its own socket (the node sends there) and sends from it.

**Session.** `{"type": "hello", "token": <arm lease token>, "client": "wbc"}` → `{"type": "welcome", "session": u32,
"stale_ms": 30, "limits": {lo, hi, joint_vel, joint_acc, max_lead, limit_tol, gap_max}, "rate_hz": 100}` or
`{"type": "refused", "reason"}` (no or wrong token, e-stop latched, not connected yet). One session; a new hello
replaces it (the old client gets a bye). `{"type": "bye"}` from the client closes it. The node closes it with
`{"type": "bye", "reason"}` on lease end, e-stop, `POST /disable` and shutdown. A fault, a chock or a stow leave it
open: the client keeps getting state packets (state `fault` / `disabled` / `off`), its goals are dropped until the arm
is enabled again. Every datagram of the session (from its address, with a newer `seq`) renews the lease
(`Lease.renew_holder`), so the client need not renew over HTTP.

**Commands** (`RUC1`, 88 bytes, `<4sBBHIIdd6f6fff`): magic, version 1, mode (0 keep-alive, 1 joint goal), flags (0),
session, seq, t_send, t_basis (the `t_tick` of the state packet the goal was computed from, or NaN), q (6, rad, output
side), qd (6, rad/s feedforward), gap (m; NaN = unchanged), reserved. `seq` strictly increases per session over
keep-alives and goals; older or equal is dropped.

- **Checked in the receiver** (dropped and counted by reason, never an event per datagram): finite q / qd / gap;
  q within the joint limits + 0.5°; gap within 0..49.6 mm; to engage, the arm within its limits (the engage
  check); every joint within `max_lead` (20°) of the measured pose. **Gated at write time**: enabled, motors on, no
  e-stop; a goal arriving while disabled never engages later.
- **A goal is (q, qd) from the tick that first uses it**: until the next goal it moves on at qd (with qd = 0 it is q,
  literally). Every goal goes through the command step (joint speed and acceleration limits, joint limits) as track
  and planned motions do; a non-finite goal reaching it is held instead (`nonfinite_goals`, any source). The
  feedforward changes by at most `joint_acc` per second and is capped at `joint_vel`.
- **Stale** (no fresh goal for `stale_ms`; keep-alives do not refresh a goal): the command ramps to rest at the soft
  joint acceleration from its current velocity and holds there (no instant brake); the feedforward decays at the same
  rate; the STALE flag is set. The next fresh goal resumes.
- **Preemption.** A goal replaces track, HTTP track replaces the stream (newest wins). **A planned motion (servo,
  pose, home, stow, chunk) is never preempted by the stream:** starting one pauses the session (`paused`, the PAUSED
  flag); goals are dropped while it executes or settles; the first goal after it ended resumes (so a client must
  recompute its goal from the state packets: a stale goal far from the arm is refused by the lead bound).
  `POST /cancel` and `/float` drop the stream's goal like track's; e-stop and disable close the session.
- Jam detection is report-only for the stream, as for track.

**State** (`RUS1`, 136 bytes, `<4sBBHIIdIBB2x6f6f6f6fff`), sent every control-loop iteration while the session is
open, right after the bus read (also on fault and connecting iterations, with the latest reading): magic, version,
reserved (0), flags (1 servoing, 2 stale, 4 jam, 8 limited, 16 paused), session, tick (loop iterations since the node
started), t_tick, seq of the goal the previous control tick applied (0: none), source (index into the row's `source` list; `stream` = 8),
state (index), q (output encoders), qd, tau (motor feedback), q_cmd (NaN when not servoing), gap (m), read_age.
Non-blocking: a full or vanished client socket counts a `send_errors`, never delays the tick. As in the row,
`t_tick` is before a pipelined read: `q` answers requests sent about one tick earlier (see "When the data was
measured"; `t_meas` / `t_meas_mot` are in the row, not in the packet).

**Observability.** Row columns `stream_seq`, `stream_age` (s since the goal the tick applied arrived) and `stream_lat`
(`t_tick` − the goal's `t_basis`: the client's loop latency; about one tick for a client that answers each state
packet), appended last. `/state` `joint_channel`: socket, config (`stale_ms`, `max_lead_deg`, `limit_tol_deg`), the
session (id, client, lease, address, seq, paused, stale, counts: goals, keep-alives, stale, sent, send errors;
dropped by reason), the goal in force, datagrams without a session by reason, `nonfinite_goals`. Config:
`~/.config/robot_utils/arm.yaml` `joint_channel: {stale_ms: 30, max_lead_deg: 20}`.

**Measured (sim, in-process, `tests/test_sim_integration.py`):** a client paced by the state packets at 100 Hz on
the world's pipelined arm: `stream_lat` p50 ≈ 9 ms, p99 ≈ 12 ms (one tick); state packet delivery to the client
p50 ≈ 0.7 ms; the command follows the goal with no lag of its own; the measured pose trails the command by ~5 ticks
(the world arm's servo plus the pipelined read).

## Simulation

`--sim`: the same node with a simulated bus (each joint follows its MIT command as a
second-order system with a little friction; optionally gravity, so floating, sinking in damping
and a wrong gravity feedforward show; the driver's refusal of setpoints outside the limits; the
gripper stops at 0 mm or an "object" gap). Needed by the preview harness
and the tests; nothing in the tests touches hardware.

### Timing (`--timing`)

Two stall limits fault the node: with the motors on, a reading older than `stale_on` (50 ms), and the control
process silent for `ctl_heartbeat` (0.25 s; latched until the node restarts). These are the robot's. `--timing
lenient` raises them to 0.5 s / 2 s, for a simulated arm only (refused on hardware); `auto` (the default) is lenient
for a simulated arm on macOS, which has no real-time scheduling: under load its control process stalled past 0.25 s
at start and was reported dead. The sim world's own motor watchdog is lenient there too (docs/sim.md). The node's
events say which is in force.

Several arms (`--robot`, robot profiles in `src/robot_utils/robots/`): one arm node instance per arm, named as the
arm (`--name arm_left --robot rebot_bimanual --sim world`). The profile gives the instance its arm: the backend
socket `<arm>.sock`, its description frame (`arm_in_base`), and for a mirrored arm its mirrored URDF for the
kinematics (IK, track, servo, geometry; the planner process too). A one-arm robot's arm serves any instance name.
A mirrored arm or a `sim_only` robot refuses hardware (a real mirrored arm needs a per-joint sign map in the
driver). docs/plans/bimanual_and_robot_swap.md.

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
