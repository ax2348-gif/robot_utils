# Whole-body control (WBC), v1: draft

Status (2026-11-21): v1 built (docs/wbc.md): the QP controller, target buffers, the node with leases, the datagram
session, the stream and cross-bus faults; tested against a fake arm joint-channel peer, the swerve sim and the
MuJoCo world. Decided while building: world-frame targets only (no `base` frame), `est_reset` in datagrams, reach and
manipulability limits as constraints, base hints in a lower-priority stage. Not yet: the fence, the MPC, the real
arm side of the joint channel (parallel branch).

## Goal

One node that moves the arm and the base together to follow end-effector (EE) targets, for teleop, policies and
agents: the operator or policy says where the gripper should be, the WBC decides how much the base and how much the
arm move. Runs on the Pi at 100 Hz. Open to bimanual and 7-DoF arms (nothing is hard-coded to one 6-DoF arm).

## Decisions already made

- **On the Pi, 100 Hz**, CPU (a small QP: daqp solves 9 variables in ~7 µs; a full pink/mink-style step was 0.04 ms
  on the Mac, estimated ~0.1–0.3 ms on the Pi).
- **pink (Pinocchio) for kinematics and the QP, in the sim and on the robot alike**: one kinematic model from the
  description node's `/model/urdf?base=planar`, the same code path in both; MuJoCo stays physics-only and off the
  robot. Solver: daqp through qpsolvers.
- **QP first, MPC swappable** behind one controller interface (below).
- **EE targets are canonical in the world (odometry) frame**; pose plus optional velocity feedforward (with a staleness
  timeout); reactive streams and timestamped chunks are one mechanism (a target trajectory).
- **Relative vs absolute is the client's business**; the WBC stream publishes the commanded EE pose so deltas can be
  resolved against it.
- **Arbitration by leases**: the WBC holds the arm and swerve leases (so it and pose_servo cannot run together);
  agent_interface moves onto the WBC later instead of driving the arm and pose_servo separately.
- **The stream follows the arm's convention** (timestamped rows, append-only schema) with the additions below.

## Research it builds on (2026-11-21)

- **Astribot Suite** (arXiv 2507.17141): EE-space whole-body commands beat joint space (whole-body sorting 18/20 vs
  5/20); the WBC sits in the teleop loop (100 Hz, ~20 ms) so demos and deployment share it; chunk blending: drop the
  stale prefix, cross-fade with a decaying weight under velocity limits; delta actions are much smoother at chunk
  seams. Its controller itself is unpublished.
- **Haviland et al.** (RA-L 2022) and **M⁴Diffuser**: one weighted velocity-level QP over base (planar x/y/yaw) and arm;
  base cost scheduled by distance to the goal (cheap far, expensive near); arm-only manipulability (including the
  base: 161 vs 39 failures); joint-limit velocity dampers; a base-heading term.
- **HoMeR / TidyBot++**: the closest robot (holonomic base + arm); mink QP (EE task, posture 2e-3, base damping),
  the same WBC in teleop; base twist shaped by an online trajectory generator; base stops after 2.5 policy periods
  without a command.
- **UMI on Legs**: world-frame EE trajectories are essential (15.5 vs 2.1 cm error without); the controller sees a
  preview window (−60..+60 ms and 1 s ahead) — evidence for keeping the horizon in the interface.
- **Mobile ALOHA**: the base lags its commands; shift base actions earlier by the lag.

## Architecture

```
targets (fast path / HTTP) ─► target buffers (per EE, world frame, timestamped, blended)
base hints, base fence     ─►─┐
state (arm stream with t_meas, estimator, swerve) ─► state snapshot
                               └─► Controller (QP now, MPC later) ─► arm q/qd (arm fast path), base twist (swerve)
                                         ▲
                     cost & constraint spec (declarative, shared by both controllers)
```

**The controller interface** (the swap point): each tick it gets the state snapshot, the target buffers with their
full horizon, the hints, the active constraints (limits, base fence) and the spec; it returns this tick's command and
optionally its planned trajectory (published in the stream either way). Everything else — model, buffers, blending,
hints, fence processing, safety, I/O — is shared.

- **QP (v1):** decision vector = base twist (x, y, yaw as planar joints) + arm joint velocities. Tasks: EE pose
  tracking (feedforward + P on the pose error, separate position/orientation weights, Levenberg–Marquardt damping),
  base-vs-arm weighting scheduled by EE distance to its target, arm posture regulariser + arm-only manipulability
  gradient, base heading. Constraints: joint position (velocity dampers), joint velocity and acceleration (protects the
  backlash), base twist and twist-rate limits, per-wheel speed limits (wheel velocity is linear in the twist: an
  inscribed octagon, 4 rows per wheel), base fence rows. The horizon enters only through the velocity feedforward.
- **MPC (later):** kinematic, linearised along the horizon (~0.5–1 s), the same spec as costs over the horizon,
  steering reorientation as a lag, the fence along the horizon. OSQP or acados (C code generation; check aarch64).
  The switch is decided in the sim on the same scenarios and metrics.

**Swerve-specific:** steering rate is not linear in the twist (it blows up near zero wheel speed), so it stays in the
swerve node; the WBC bounds the twist rate and uses a deadband with hysteresis on tiny base commands (no steering
chatter).

**Base lag:** the measured base pose is fed back every tick (the arm absorbs base tracking error at the EE: why
targets are in the world frame), and the base is rate-limited harder than the arm, so the arm does not over-correct
and then fight the base (the classic base–arm oscillation). Tested in the sim with the base's modelled lag.

**Arm output:** integrate from the previous *commanded* q (resync to the measured q only past a threshold; integrating
from the measured q every tick makes the backlash chatter); q̇ is the MIT velocity feedforward.

## Interface

### Targets (per EE)

A timestamped trajectory in the world frame: samples `(t, pose[, twist][, gripper])`, `t` on the Pi's clock.

- **Reactive:** one sample, "now"; newest wins; the twist feedforward is used while fresh.
- **Chunk:** a sequence; replaces the buffer from its first timestamp on; the stale prefix is dropped; blended into the
  old trajectory with a decaying weight; every replacement is anchored at the *commanded* state (pose and velocity),
  never the measured one; `t0` bounded against the Pi clock (a wrong clock is a 422, not a hold).
- **Frames accepted:** `world` (canonical), `base` (converted to world at the command's timestamp), `ee` deltas are
  resolved client-side. World targets carry the estimator's reset id; across an estimator reset they are aborted.
- **Staleness, two stages:** past `ff_timeout` the feedforward fades to zero and the pose holds; past `hold_timeout`
  (≈ 2.5 × the expected command period) the base stops and the arm holds.

### Hints (the structure)

Hints say *how* to reach the targets. They are session defaults set over HTTP, overridable per command (a field on a
target command or chunk); they persist until replaced; every numeric hint is bounded by the node's caps.

```yaml
base:
  mode: free            # free | locked | hint | commanded
  # mode hint: a soft target for the base, each part optional, each with its own weight and tolerance
  target:
    pose: {x: 1.2, y: 0.4, frame: world, weight: 1.0, tol: 0.05}   # "stay near here"
    heading: {yaw: 1.57, frame: world, weight: 0.5, tol: 0.05}     # or {face: ee_right}: face an EE / a point
  # mode commanded: the base follows this; the arm compensates to hold the EE targets
  velocity: {vx: 0.1, vy: 0.0, omega: 0.0, frame: base, ttl: 0.3}  # teleop stick; zero after ttl
  pose: {x: 1.2, y: 0.4, yaw: 0.0, frame: world}                   # or a base pose to drive to
  limits: {max_speed: 0.3, max_omega: 0.8, max_accel: 0.5}         # tighter than the caps, never looser
  arm_vs_base: 1.0      # > 1: prefer moving the arm; < 1: prefer moving the base (scales the base cost)
ees:
  ee_right:
    weights: {position: 1.0, orientation: 0.3}     # orientation weight 0: position-only task
    orientation_mask: [1, 1, 0]                    # ignore yaw about the tool axis (5-DoF tasks)
    posture: home       # a named arm posture (or a q vector) the null space prefers, with its weight
    posture_weight: 0.002
```

- **`free`** (default): the base moves as the cost decides; **`locked`**: base twist = 0 as a hard constraint;
  **`hint`**: free plus the soft target above (no cost inside `tol`); **`commanded`**: the base follows `velocity`
  (with its `ttl`) or `pose`, and the arm compensates.
- Bimanual: `ees` has one entry per EE; the base hint is shared. 7-DoF: the posture hint is where null-space
  preferences go first; dedicated null-space tasks are a later addition behind the same spec.

### Base fence (constraints from perception or a policy)

- **Input:** a 2D cost/occupancy grid in the world frame (resolution, origin, values; inflation by the producer or the
  WBC) and/or polygons (keep-out areas, a keep-in workspace), each with a timestamp and a lifetime. Set over HTTP;
  a stream option for live perception. Producers: perception3d, the agent's depth grid (the reflex computes one
  today), a policy.
- **Use:** each tick, a distance field around the base footprint gives the nearest obstacle points; each adds one
  velocity-damper row (distance may shrink no faster than a rate that reaches zero at the safety margin): linear in the
  base twist. Hard keep-out for occupied cells and polygons, a soft proximity cost in the inflation band. The MPC
  applies the same rows along its horizon (it slows early instead of at the edge).
- **Expiry:** configurable; default: slow, then lock the base.
- **Footprint:** v1 the base outline; the arm's projected overhang and 3D arm–environment avoidance later, same
  mechanism.

### Stream

The arm's convention (timestamped rows on the Pi clock, append-only, `/fields`), plus: per EE (columns prefixed by the
EE name, generated from the model) the commanded and measured EE pose (position + quaternion) and twist in the world
frame; the base pose and estimator reset id used; the target/chunk id and `seq` in force; input ages (using the arm's
`t_meas`); the base mode and active fence rows; the controller's solve time and status.

## Safety and arbitration

- **Leases:** the WBC acquires the arm and swerve leases together (if the second fails, it releases the first) and
  exposes its own lease (`common/lease`) to its client; when that client lapses or goes silent, the WBC holds/stops the
  whole body although its own downstream leases are still valid.
- **Cross-bus faults:** an e-stop or fault on either bus stops the other half; the WBC joins every e-stop fan-out.
- **What pose_servo does today that the WBC must match** (non-perceptive, goal-level): abort on a stale, non-finite or
  too-old estimator pose, on pose jumps and estimator resets; stall detection; swerve lease and enable checks; speed /
  acceleration / jerk caps and a maximum distance and rotation per goal. The perceptive stop reflex lives in
  agent_interface (it stops the pose_servo goal when its depth grid shows an obstacle ahead); the base fence is its
  successor once agent_interface drives the WBC.
- **Failure:** an infeasible or failed solve outputs zero velocity, holds position, and raises a fault; arm-only
  (`locked`) is the safe fallback mode.

## Open decisions

- The second research pass on streaming interfaces (UR `servoL` lookahead/gain, Franka's Cartesian streaming, MoveIt
  Servo timeouts and slow-downs, DROID / Open-X / LeRobot / openpi action conventions) before freezing the target
  message.
- Timeouts (`ff_timeout`, `hold_timeout`) and the deadband values: from the sim.
- The fence's default expiry policy and inflation.
- Gripper: in the EE target (default), or its own channel?
- Where the WBC's own command input arrives: its own fast-path session (datagrams, like the arm's) for streams and
  chunks, HTTP for hints, fence and config.

## Order of work (proposed)

1. The spec, model loading from `/model`, target buffers and blending, the QP controller; unit tests with
   pinocchio only.
2. The node: leases, stream, HTTP hints/fence/config, fast-path input; in-process tests against the sim backends.
3. Sim scenarios: reach while driving, chunk replacement, base lag, fence, stale inputs, faults; metrics shared with
   a later MPC.
4. Teleop through the WBC (Quest), then a policy; supervised robot test.

## Interfaces for the first build (frozen 2026-11-21: whole-body Quest teleop validated in the sim)

Four pieces are built in parallel against these contracts; change them only together.

**1. Arm joint channel (internal, see `fast_path_v1.md` "Local only").** A Unix datagram socket
`<runtime>/robot/arm.joint.sock` (owner-only). Control datagrams are small JSON: `{"type": "hello", "token": <arm lease
token>, "client": "wbc"}` → reply `{"type": "welcome", "session": u32, "stale_ms": 30, "limits": {...}}` or
`{"type": "refused", "reason": ...}`; `{"type": "bye"}`. Commands are the 88-byte `RUC1` struct; state is the 136-byte
`RUS1` struct, sent every tick to the client's bound socket address while the session is open (including fault). The
arm node renews its lease on every valid command (`Lease.renew_holder()`), so the WBC need not renew over HTTP.

**2. WBC node (`nodes/wbc`, a normal node: unix socket behind nginx at `/nodes/wbc/`, page, `/meta`, `/state`,
`/events`, `/stream`).** HTTP: `POST /lease` (+ renew/release, `common/lease`), `POST /enable` (acquires the arm and
swerve leases together, opens the arm joint channel; refuses with the reason if any part fails), `POST /disable`,
`POST /estop` (+ fan-out), `GET/POST /hints` (the hint structure above), `GET/POST /fence` (later), `POST /session`
(→ `{"session": u32, "udp_port": int, "unix_socket": path}`). One end effector for now: `"arm"` (link and TCP from the
model; names come from the model so a second arm adds a key).

**3. WBC target datagrams (the external fast path).** UDP on the robot LAN address (the Mac's loopback in the sim) and
a Unix datagram socket; JSON v0 (binary later if a measurement asks):
- reactive: `{"type": "target", "session": u32, "seq": u32, "t": null | <Pi monotonic s>, "ees": {"arm": {"frame":
  "world" | "base", "pos": [x, y, z], "quat": [x, y, z, w], "twist": [vx, vy, vz, wx, wy, wz] (optional), "gripper":
  0..1 (optional)}}, "base": {<hint overrides, same keys as /hints base>} (optional)}`; `t: null` = now.
- chunk: `{"type": "chunk", "session", "seq", "ees": {"arm": {"frame", "t": [...], "pos": [[...]], "quat": [[...]],
  "twist": [[...]] (optional), "gripper": [...] (optional)}}}`.
- keep-alive: `{"type": "alive", "session", "seq"}`. Older `seq` dropped; unknown session dropped (counted).

**4. WBC stream (rows, the arm's convention, append-only).** `t_tick`, `seq` in force, `state`, `base_mode`, `solve_ms`,
`status`; the base pose used (`base_x`, `base_y`, `base_yaw`, world) and `est_reset`; the base twist commanded
(`base_vx`, `base_vy`, `base_w`); per end effector `<ee>_` + `cmd_x..cmd_qw` (commanded pose, world), `meas_x..meas_qw`
(measured, world, from the arm's `q` at `t_meas` and the estimator pose), `twist_vx..twist_wz` (commanded), `gripper`;
input ages `arm_age`, `est_age`, `target_age`.

**5. Quest client (`tools/quest_teleop/`, runs on the Mac or erid).** Reads the headset's `teleop.quest_state.v1`
stream directly (port 9391), implements the body-frozen scheme (per engagement: the operator frame from head position
and gravity-aligned yaw, the measured EE anchor from the WBC stream's `arm_meas_*`, targets in the world frame), maps
the head's yaw to the base `heading` hint (optionally its displacement to a soft base `pose` hint), opens a WBC session
and sends target datagrams at the stream rate; on stale or invalid input it stops sending (the WBC holds, then stops
the base). The triggers drive the gripper.
