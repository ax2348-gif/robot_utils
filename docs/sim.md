# Sim world node

`python -m robot_utils.nodes.sim` runs a MuJoCo world of the robot (design: [plans/sim_v1.md](plans/sim_v1.md)).
The robot model comes from the description node ([description_node.md](description_node.md), "Robot model");
the arm, swerve and camera nodes attach to the world through their **hardware backends**, so their own code
(leases, e-stops, limits, the swerve node's CAN path, the camera owner) runs unchanged. Development machines,
erid or CI only, never the robot. Needs the `sim` extra (`mujoco`).

**The whole stack in one command** (landing page + every node in sim mode): see "The sim stack" below.

```bash
python -m robot_utils.nodes.sim.stack            # then open http://localhost:8080/ (Ctrl-C stops everything)
```

```bash
python -m robot_utils.nodes.sim                       # model from the local description node (instance "description")
python -m robot_utils.nodes.sim --model local         # assemble the shipped description here (nominal camera poses)
python -m robot_utils.nodes.sim --model http://botcore/nodes/description --port 8130
python -m robot_utils.nodes.sim --paused --config my_sim.yaml --variant simple
```

Options: `--model` (description node instance or URL, a `bundle.zip`, or `local`), `--config` (YAML merged over
`nodes/sim/default_sim.yaml`; default `$ROBOT_SIM_CONFIG`), `--variant full|simple`, `--paused`, `--rtf`,
`--scene-dir`, `--name`, `--port`.

**The page** (node root, `/nodes/sim/`): a live 3D view of the world (three.js, pinned `three@0.170.0` from
cdn.jsdelivr.net: the browser needs the internet; without it the rest of the page works): the robot from the
model's visual geoms (base, modules steering and wheels rolling, arm, gripper fingers), scene objects, the cameras'
frustums, the base's trail, and the **markers** other nodes post (below), at 30 Hz. Drag to orbit, wheel to zoom,
double-click / *3D* / *Top* for the preset views, *Follow* keeps the robot centred, *Lines* draws edges only.
Beside it: time (run / pause / rtf / step / reset, the sim clock), the robot (base pose and twist, arm, modules,
backends connected); below: the scene (add / move / remove objects, save / load), the **privileged physics** panel
(push a body, teleport the base; marked SIM-ONLY), markers, camera previews (rendered on request: each refresh
costs the physics thread a little, so periodic refresh is off by default), events.

## What the world is

The description node's `GET /model/bundle.zip?base=swerve` (URDF + meshes), plus from the sim config: a free
joint on `base_link`, the floor (friction), actuators, cameras, a light, scene objects. 1 kHz, `implicitfast`.
The robot never collides with itself; it collides with the floor and objects.

| component | fidelity (v1) | model |
|---|---|---|
| base | hybrid (decided) | per module a steer position servo and a wheel velocity servo, each torque-capped at what the SPARK's duty and current limits allow (9.0 N m steer, 3.5 N m wheel). Their references follow the SPARK setpoints within the robot's own limits: steer rate 7.4 rad/s, wheel speed 1.87 m/s, wheel acceleration 10.7 m/s² (see "Limits come from the robot config"). The chassis is a free body moved by wheel–floor friction; soft wheel contact (`solref 0.02 s`) is the modules' compliance (all four wheels stay loaded, tested). A disabled drive coasts (no torque), a disabled steer holds its angle; no setpoints for 0.1 s disables every module (a lost heartbeat). |
| arm | kinematic | the reference follows each MIT position setpoint within the arm node's hard joint speed caps (2, 2, 2, 6, 6, 6 rad/s); a joint sent kp = 0 holds where it is. A stiff servo plus a `qfrc_bias` feedforward applied as joint torque keeps the joints on it, so the chassis carries the arm's weight and reaction. No MIT frame for 0.1 s while enabled: status 0xD (the motors' CAN watchdog); a lost backend connection likewise. |
| gripper | kinematic | the motor angle follows its goal (kp > 0), or closes / opens under a torque command, at `gripper_speed`; the fingers (gap = −q · 16 mm/rad) are force-capped, so they stop on an object. |
| cameras | rendered | head and wrist: colour (JPEG, as the camera's MJPG) and depth (Y16 mm, 0 outside `depth_range`) at the configured size / rate / field of view, rendered from the links `<camera>_color_optical` / `_depth_optical`. `t_sensor` = the host time of the rendered state, `t_capture` = `t_sensor` + the measured delay (head colour 30 ms, depth 17 ms, wrist 47 ms). No noise or exposure yet. |
| IMU | MuJoCo sensors | the head camera's (`cameras.camera.imu`): accelerometer (specific force: +9.81 m/s² up at rest) and gyro at the link's origin, in its frame (the depth optical frame), sampled every physics step (1 kHz); `t_sensor` = the host time the sample stands for (real-time mode: the wall-clock anchor), `t_capture` = + `delay` (2 ms). No bias, noise or temperature model. The camera node's attitude filter and the estimator's fused heading run on it unchanged. The wrist camera has none (as the Gemini 305). |

`dynamic` fidelity (MIT motors, backlash, friction, gripper contact law) is the next step: the config already
has `components.arm.fidelity`; anything but `kinematic` is refused for now.

## The default scene

The world starts with a tabletop scene (`scene.tabletop: true` in `default_sim.yaml`; `nodes/sim/scene.py`), the base
at the origin facing +x. Derived from the model, not guessed:

- **table** (fixed, 0.6 × 1.6 m, 3 cm top, four legs): top at the arm's mount height (the base's top surface
  `TOP_Z` = 0.222 m, where the arm's plate sits) + 0.08 m = **0.302 m**, in the band where the arm reaches farthest
  pointing straight down (sampled from the model's joint ranges: top-down grasps reach x ≈ 0.55–0.6 m for table
  heights 0.15–0.40 m); its front edge **4 cm in front of the base** (`FRAME`/2 + 0.04 = 0.256 m), so the first
  ~0.3 m of the top is in reach from the start pose. It is long in y: driving sideways along it brings the rest in
  reach (the robot cannot pass under it: the arm and camera are above its top).
- **on it**: `cube` (4 cm, red, x 0.36 y −0.15), `can` (cylinder ⌀ 40 mm × 8 cm, blue, x 0.38 y 0), `block`
  (3 × 8 × 5 cm, green, turned 0.4 rad, x 0.40 y −0.30), placed so a top-down grasp and a 10 cm lift are in reach
  (docs: "Grasping"): each narrower than 0.8 × the gripper's opening (`GAP_MAX`
  49.6 mm) and reachable pointing down from the start pose (tested with the model's kinematics); `cube_far` (4 cm,
  yellow, y 0.55): out of reach until the base moves ~0.5 m left.
- **in the head camera's view**: `cube_view` (4.5 cm, white, x 0.74 y 0, near the table's far edge). On the robot the
  head camera sits ~1.0 m up pitched ~19° down (its calibrated mount), so it sees the table top only beyond
  x ≈ 0.68 m: the near objects are seen by the wrist camera (the gripper looking down), as on the robot.
- **on the floor**: `crate` (0.3 m box, 3 kg, at (1.2, −1.4)) and `bin` (cylinder ⌀ 0.3 × 0.4 m, 2 kg, at (−1.0, 1.3)),
  for walking tasks.

They are in `/scene` (fixed ones with `fixed: true`), the page's 3D view, both camera renders, and `robot_state.v1`
`objects` (`fixed` too). `scene: {tabletop: false}` in a sim config starts empty (the tests' default world does).

**Articulated objects and the cabinet**: an object can be `shape: articulated`, a tree of primitive links joined by
slide / hinge joints; `scene: {cabinet: true}` (or `ROBOT_SIM_CONFIG=deploy/sim/sim_cabinet.yaml`) adds a cabinet
with two drawers and a door beside the table, a cube in its upper drawer. docs/scene3d.md, which also covers
`/scene3d`: the world as rigid bodies (geometry once, poses streamed) for the headset.

## Grasping

- **Jaw collision**: in every model variant the finger links collide as the **jaw boxes** (`arm/geometry.py`
  `ARM_BOXES`: the carriage and the prong, bounded from the visual meshes; the same boxes the arm pages draw), not
  their CAD meshes: MuJoCo collides a mesh as its convex hull, and the finger mesh's hull filled the space between
  the pads, so the jaws closed through objects. Visual meshes stay. The right finger follows the left through the
  URDF's mimic (a joint equality), stiffened to `contact.fingers.mimic_solref`.
- **Contact** (`default_sim.yaml` `contact:`, sim-only physics): elliptic friction cones, `noslip_iterations: 5`;
  fingers `condim 4` (torsional friction), friction 1.5, stiff `solref [0.004, 1]`; objects `condim 4`, friction 0.8
  (MuJoCo takes the larger friction of the two geoms). Without them (pyramidal, condim 3, default solref) the jaws
  sank ~4 mm into the objects and they slipped 4–10 mm in 2 s.
- **A top-down grasp that works** (`tests/test_sim_grasp.py`, measured: cube / can / block lifted 97 / 97 / 96 mm of
  100, the gripper itself 99 mm, slip 0.0 mm over 2 s, in the full, boxes and simple variants): fingertips (the
  end_link origin) 6 mm above the table, or higher when the object is taller than the rail's face behind the tips
  (73 mm) so the rail stays above its top; the jaws closing along a box's narrow side; a straight vertical approach
  (IK in 1 cm steps: a joint-space move between two poses swings the open jaws sideways through the object); the
  base held (zero twist: an idle base coasts, and the arm's reaction moved it 12 mm, enough to land the 4.8 mm
  per-side clearance of the open jaws on the cube). The top-down reach ends about 10 cm above the table near the
  front: the lift lets the wrist tilt.
- **Grip force**: the sim's finger servo pushes 2000 N/m × the jaw's position error, capped at `HARD_MAX_FORCE`
  (120 N): on a 40 mm cube closed to 0, ~39 N. The arm node's own force limit (`max_force_n`) acts on the command it
  sends, as on the robot. Commanded closed through the arm node (`POST /gripper {gap_mm: 0}`), the world's jaws close
  (tested); the world's `g_ref` runs from 0 (closed) to `GRIPPER_OPEN_RAD` (open).

## Limits come from the robot config

The world has no actuator limits of its own: `nodes/sim/limits.py` computes them from the robot codebase at start
(shown in `/state` `limits`), and a sim config that sets one is refused. Controllers (pose_servo, the WBC) keep
their own, tighter settings on top, as on the robot.

| limit | value now | from |
|---|---|---|
| wheel speed | 1.87 m/s | `swerve/constants.py` `DRIVE_MAX_OUTPUT` (0.35, the drive SPARK's closed-loop `output_range`, `setup_sparks.py`) × `DRIVE_FREE_SPEED` |
| wheel acceleration | 10.7 m/s² | `DRIVE_FREE_SPEED / DRIVE_RAMP` (the drive SPARK's closed-loop ramp: 0.5 s from 0 to full output) |
| wheel torque | 3.5 N m | min(`DRIVE_MAX_OUTPUT` × Vortex stall torque, `DRIVE_CURRENT_LIMIT` (40 A) × its torque per amp) × `DRIVE_REDUCTION` (the current limit governs) |
| steer rate | 7.4 rad/s (1.18 turns/s) | `STEER_MAX_OUTPUT` (0.3, the steer SPARK's `output_range`) × `STEER_FREE_RATE` (NEO 550 free speed / `STEER_REDUCTION` 46.42) |
| steer torque | 9.0 N m | min(`STEER_MAX_OUTPUT` × NEO 550 stall torque, `STEER_CURRENT_LIMIT` (20 A) × its torque per amp) × `STEER_REDUCTION` |
| arm joint speeds | 2, 2, 2, 6, 6, 6 rad/s | `arm/config.py` `HARD.joint_vel`, as the arm node loads it (`load_settings()`: `~/.config/robot_utils/arm.yaml` `joint_vel_max` overrides) |
| gripper force | 120 N per finger | `arm/config.py` `HARD_MAX_FORCE` |

The motor figures (`VORTEX_STALL_TORQUE` / `_CURRENT`, `NEO550_FREE_RPM`, `NEO550_STALL_TORQUE` / `_CURRENT`,
`STEER_REDUCTION`) are REV datasheet values, added to `swerve/constants.py` next to the existing `VORTEX_FREE_RPM`.
The simulated SPARK is a speed clamp plus a torque cap (the two ends of the duty cap's torque–speed line, slightly
stronger than the motor in between). Not from the robot config, so listed as estimates: `gripper_speed` (6 rad/s of
the gripper motor: the robot sets no speed, the node sends the goal through MIT PD; ~0.5 s full stroke), the arm's
`watchdog` (0.1 s: the Damiao motors' CAN timeout) and `reply_latency` (7 ms, measured). Sim-only keys:
servo stiffness / damping and armatures, contact and friction.

Before this (2026-11-21) the sim used the pose_servo planner's caps (4π rad/s steering, 3 m/s, 0.85 m/s²) and
stall-torque estimates: it steered faster and pushed harder than the robot can.

## Time

`POST /run {rtf}` locks the sim clock to the wall clock × rtf (a physics thread; if it falls behind by 0.1 s it
drops the backlog and counts it in `behind`); `POST /pause`; `POST /step {n | seconds}` advances by hand (meant for
paused). `/state` has `t` (sim s), `rtf_measured`, `step_rate`. Cameras render on sim time: a paused world
delivers no new frames (the camera node then reports a stall after 4 s). Tests run the world in-process:
`World.step(n)` for deterministic stepping, `World.run()` for real time.

## Scene and privileged controls

`GET /scene`; `POST /scene/objects {name, shape: box|cylinder, size | radius+length, mass, fixed, xyz, rpy|yaw}`
(`fixed: true` welds it to the world: furniture, no mass, pushes do nothing; a teleport still moves it);
`PUT /scene/objects/{name} {xyz, rpy|yaw}` (teleport); `DELETE /scene/objects/{name}`; `POST /scene/reset` (time
0, robot at its start pose, objects where added, commands dropped; `reset_seq` + 1, in the reply, `/state` and every
`robot_state.v1` record: the headset re-anchors its AR scene when it changes; `{"start": true}` reloads the start scene
instead: the default tabletop and the config's objects); `POST /scene/save {name}` (current poses to
`<scene dir>/<name>.yaml`) and `POST /scene/load {name | objects}`. Adding or removing an object recompiles the
model with the state kept.

**Sim-only, never on a hardware node:** `POST /physics/wrench {body: base|<object>|<link>, force, torque,
duration}` (world frame, at the body's centre of mass) and `POST /physics/teleport {xyz, yaw|rpy}` (the base,
velocity zeroed).

No lease and no e-stop (decided): the sim substitutes for the hardware. Stop the robot through its nodes.

## Protocol (backend sockets)

One Unix socket per node backend under `<runtime dir>/robot/sim/<sim name>/`: `arm.sock`, `swerve.sock`,
`camera-<camera>.sock` (`protocol.py`). A message is `u32 big-endian N | N bytes of JSON header | header.len
bytes of payload`. The client sends `{"op": ..., ...args}`, the world answers `{"ok": true, ...}` or
`{"ok": false, "error": "..."}`, in order, one reply per request. Only images use the payload.

| socket | ops |
|---|---|
| `arm.sock` | `hello` → joints, `reply_latency`; `enable`, `disable`, `gripper_motor {on}`, `check_gripper` → `g_q`; `send {q, qd, kp, kd, tau (6 each), g_q, g_qd, g_kp, g_kd, g_tau, gripper}`; `read` → `t, q, qd, tau, status (7), g_q, g_qd, g_tau` (the state now) |
| `swerve.sock` | `hello` → modules, wheel radii; `modules {steer: {FL: [enabled, angle rad]}, drive: {FL: [enabled, m/s]}}` → `t`, per module `steer` (rad), `steer_rate`, `wheel` (rad), `speed` (m/s) |
| `camera-<c>.sock` | `info` → serial, profiles, intrinsics, `depth_to_color`, `imu` (null or `{rate_hz, delay}`); `start {streams}`; `stop`; `frames {after: {stream: seq}, wait: s}` → `frames: [{stream, format, width, height, seq, t_sensor, t_capture, t_sim, depth_scale, len}]`, payloads concatenated in that order; `imu {after: seq, wait: s}` → `samples: [[seq, t_host, t_sim, ax, ay, az, gx, gy, gz], ...]` |

## Backends (`nodes/sim/backends/`)

- `arm.WorldArmBus(sim, limits)`: an `ArmBus` (`ArmNode(lambda: WorldArmBus("sim", limits=...))`). Pipelined like
  `HardwareBus`: each `read` returns the world's state as of the previous read's request and sends the next;
  the first read waits for its own. The driver's setpoint check as rebot_ctl. **Measurement times** as the hardware's, in `Reading.t_meas` / `t_meas_mot` (the arm row's
  last columns): `t_meas` = when the requests that gave `q` were sent (the previous read, so `t_tick - t_meas`
  is about one tick, tested), `t_meas_mot` = when the world answered the last MIT frames (the motor-side
  feedback), else the read reply.
- `swerve.start_world_sim(sim)` → `(sparks, interface, coupler)`: eight sparkcan `SimulatedSpark`s configured as
  `setup_sparks` would, on a virtual CAN bus, whose motion is the world's (`SwerveNode(interface)`). The steering
  SPARK turns its POSITION setpoint (absolute encoder, 0..1) into a module angle and reports the world's angle as
  that encoder reading (status 5); the drive SPARK sends its VELOCITY setpoint (m/s) and reports wheel speed and
  travel (status 2). One coupler thread exchanges all eight with the world every 2 ms. `stop_world_sim(...)`.
- `camera.WorldCameraBackend(sim, camera)`: the orbbec backend interface (`Owner(WorldCameraBackend("sim",
  "camera"), hub, ...)`): one colour profile (MJPG) and one depth mode, no IR; the IMU (1 kHz, `1kHz` / `4g` /
  `1000dps` in the catalog, extrinsics identity: IMU frame = depth frame) when the world's camera has one, polled
  on its own connection; frames handed over no earlier than `t_capture`.

**On the command line** (`--sim world[=NAME|DIR]`: `world` = the sim instance `sim`; `world=NAME` another instance;
`world=DIR` a directory with the backend sockets):

```bash
python -m robot_utils.nodes.arm --sim world                     # WorldArmBus (--sim alone: the old toy SimBus)
python -m robot_utils.nodes.swerve --sim world                  # world-coupled SPARKs (--sim alone: the toy SPARKs);
                                                                # waits up to 60 s for the world
python -m robot_utils.nodes.orbbec --sim world                  # instance camera -> the world's camera "camera"
python -m robot_utils.nodes.orbbec --name wrist_camera --sim world --no-imu
python -m robot_utils.nodes.power --fake                        # the simulated UPS (never powers off)
python -m robot_utils.nodes.estimator                           # unchanged: swerve odometry + the sim camera's IMU
```

The arm and camera nodes retry until the world answers (start order doesn't matter); after a sim node restart
every backend reconnects (tested: arm, swerve, both cameras). The integration tests
(`tests/test_sim_integration.py`) also construct the real `ArmNode`, `SwerveNode` and orbbec `Owner` in-process.

## The sim stack (one command, on the Mac)

```bash
python -m robot_utils.nodes.sim.stack                   # landing page http://localhost:8080/, sim at /nodes/sim/
python -m robot_utils.nodes.sim.stack --port 8090 --host 0.0.0.0   # another port, reachable from the LAN / a headset
python -m robot_utils.nodes.sim.stack --nodes description,sim,arm,swerve,estimator   # fewer nodes
python -m robot_utils.nodes.sim.stack --runtime-dir /tmp/rt2       # a second stack (own sockets; keep it short)
```

It runs the landing page (`robot_utils.frontend --config nodes/sim/stack_nodes.yaml --port 8080`, its own `/nodes/`
proxy: no nginx needed) and asks it to start, in order and each once its socket answers: `description`
(data dir `~/.local/share/robot_utils/sim/description`, so a sim never touches real calibrations), `sim`, `arm`,
`swerve`, `camera`, `wrist_camera` (`--sim world`), `power` (`--fake`), `estimator`, `pose_servo`, `wbc` (target
datagrams on UDP 127.0.0.1:9410, for a teleop client on the same machine). The registry
also lists `3d_perception` (`--allow-nominal`) and `agent_interface` (`--no-require-imu`) with Start buttons. The
landing page then works as on the robot: tiles, pages, e-stop all, Start / Stop / logs per node (child processes
of the landing page on the Mac; systemd user units where a user session exists, e.g. erid). Same instance names
as on the robot, so clients and node-to-node links work unchanged; the sockets are the usual
`<runtime dir>/robot/<name>.sock` (macOS: `$TMPDIR/robot-<uid>/robot/`). Ctrl-C (SIGTERM) stops every node newest
first, then the landing page (tested: no socket left behind). If the description or sim node does not come up the
launcher prints its last log lines and stops. The landing page's proxy streams fine, but browsers allow 6
connections per origin (the sim page uses 2: `events`, `viz/stream`).

## The robot's calibrations in the sim

The sim's camera mounts come from the description node it loads the model from, with `mounts=calibrated`. In the
stack that node runs on its own data dir (`~/.local/share/robot_utils/sim/description`), which holds a **snapshot of
the robot's description state**: `description.yaml` (the measured nominal camera poses and each camera's
`last_serial`), `active.json` and the **active** calibration directories only (`calibrations/<type>/<serial>/<id>/`),
copied read-only from the robot's store (`botcore:~/.local/share/robot_utils/description`: one `cat active.json`,
one `tar c` of those files; nothing is written on the robot) plus `snapshot.json` (host, when, the active ids).

- **At start** the stack refreshes it when the robot answers (ssh, about 6 s timeout; `--robot-host HOST`, default
  `botcore`; `--no-pull` to skip), else keeps the previous snapshot, and prints which one it uses:
  `[sim stack] robot description snapshot from botcore pulled 2026-11-21 19:21:42: CPC64630004G: ground
  20261116-211209, yaw 20261111-183418; camera serials camera=CPC64630004G`, or plainly
  `... snapshot: NONE ...: camera mounts are NOMINAL`. After the sim node is up it prints each sim camera's serial
  and whether its mount is calibrated (also on the sim page, "cameras", and in `/state` `model.camera_serials` /
  `camera_mounts`).
- **By hand**, without the stack: `python -m robot_utils.nodes.sim.snapshot` (`--host`, `--dir`; `--show` only
  prints what is there). A pull replaces the whole directory (the sim description node's own writes go with it).
- **Serials**: the description node looks calibrations up by the camera's reported serial, so the sim node gives
  each sim camera the robot's serial for that camera name (the description's `last_serial`, read from the
  description node's `GET /description` when it loads the model), falling back to the sim config's
  (`SIMHEAD0001`, `SIMWRIST0001`: no calibration matches those, so nominal mounts). The camera nodes then report
  that serial and the description node applies the active calibrations as on the robot; it has no sim code path.
- The registry's description data dir is never the robot's own (`~/.local/share/robot_utils/description`): the
  stack refuses to snapshot into it.

## Docker

`deploy/sim/`: `Dockerfile` (python 3.12-slim, nginx-light, OSMesa / EGL, the nodes' dependencies, `rev_ctl` and
`rebot_ctl` from their checkouts) and `docker-compose.yml`: service `stack` (the launcher above: landing page + nodes
as its children, so Start / Stop work) and service `nginx` (the robot's port-80 config from `frontend/proxy.py`,
listening on 8080 in the container), sharing the socket directory as the volume `sockets`; descriptions and saved
scenes in the volume `data`. Everything runs as uid 1000.

```bash
docker compose -f deploy/sim/docker-compose.yml up --build -d     # http://localhost:8080/
SIM_PORT=9000 SIM_BIND=0.0.0.0 docker compose -f deploy/sim/docker-compose.yml up -d   # another port / the LAN
docker compose -f deploy/sim/docker-compose.yml logs -f stack     # the launcher and the landing page
docker compose -f deploy/sim/docker-compose.yml down              # stops every node
```

- **Submodules**: the build takes `rev_ctl` and `rebot_ctl` as named build contexts, default the repo's own
  checkouts (`git submodule update --init rev_ctl rebot_ctl`); from a worktree without them:
  `REV_CTL_DIR=~/robot_utils/rev_ctl REBOT_CTL_DIR=~/robot_utils/rebot_ctl docker compose ... build`.
- **The robot's description snapshot**: the host's `~/.local/share/robot_utils/sim/description` (or
  `ROBOT_SNAPSHOT_DIR`) is mounted as the container's sim description data dir; the container does not pull
  (`ROBOT_SNAPSHOT_NO_PULL=1`: no ssh keys in it). Refresh on the host with `python -m robot_utils.nodes.sim.snapshot`,
  then restart the stack (`docker compose ... restart stack`): a pull replaces the directory, which a running
  container does not see. Without a snapshot the mounts are nominal (the launcher's log says so).
- **Rendering**: software OpenGL (OSMesa) by default, with `deploy/sim/sim_software_gl.yaml` as `ROBOT_SIM_CONFIG`
  (shadows and reflections off: on a MacBook's Docker, full model, 640×480 colour frame 540 → 170 ms; physics keeps
  rtf 1.0; the four streams run at ~3–10 fps). Faster: `model: {variant: simple}` in that file (40 ms).
- **NVIDIA GPU (erid)**: `-f deploy/sim/docker-compose.gpu.yml` (EGL on the GPU, shadows back on); needs the NVIDIA
  driver and the NVIDIA Container Toolkit. Not tested yet.
- **Tailscale**: `TS_AUTHKEY=tskey-... docker compose -f deploy/sim/docker-compose.yml --profile tailscale up -d`
  adds a `tailscale` container (userspace networking) serving `https://robot-sim.<tailnet>.ts.net/` → `nginx:8080`
  (`deploy/sim/tailscale-serve.json`; needs HTTPS enabled on the tailnet; `TS_HOSTNAME` renames it). Off by
  default; not tested yet.
- Mesh URLs that the description node builds (`mesh_uri=http`) take the request's Host header, which nginx passes
  without the port: through a non-80 published port fetch them with the port added.

## 3D view and markers

The page's data (`tags: view`): `GET /viz/model?cell=0.002` (bodies, drawable geoms with local pose and colour, the
mesh index; robot links' visual geoms, objects), `GET /viz/meshes.bin` (float32 vertices, uint16/uint32 triangles;
meshes simplified by vertex clustering at `cell` m: ~216 k → ~38 k vertices, ~1 MB), `GET /viz/stream?hz=30` (SSE:
every body's world pose by index, the clock, the base, markers when they change), `GET /viz/frame`. These are the
page's internals; other clients use `robot_state.v1` below.

**Markers** (`tags: markers`): anything a client wants drawn in the view, e.g. the WBC's end-effector target and
commanded pose. `POST /markers {source, replace, markers: [{name, kind, frame, pos, quat, points, color, size,
label, ttl}]}`: `kind` frame (axes, x red / y green / z blue, a dot in `color`) | point | line (`points`) | text;
`frame` = `world`, `base` (base_link) or any model body / object name (e.g. `arm_end_link`); `quat` = [x, y, z, w];
`ttl` s removes it unless re-posted (post live targets at ~10 Hz with ttl 0.5). Posting a name again replaces it;
`replace: true` drops the source's other markers first; one bad marker refuses the whole post. `GET /markers`,
`DELETE /markers[?source=]`, `DELETE /markers/{name}`.

```bash
curl -X POST localhost:8080/nodes/sim/markers -H 'content-type: application/json' -d '{"source": "wbc", "markers":
  [{"name": "wbc/arm/target", "frame": "world", "pos": [0.5, 0.1, 0.6], "quat": [0, 0.7071, 0, 0.7071],
    "color": "#e0a030", "label": "target", "ttl": 0.5}]}'
```

## Robot state stream (`robot_state.v1`) and the kinematic tree (`robot_tree.v1`)

For clients that draw or reason about the robot themselves (the Quest's AR view): the **description** comes from
the description node, the **state** from a stream, and the client does the kinematics. The real robot will
publish the same `robot_state.v1` later (likely the description node aggregating the arm, swerve and estimator
streams: `t` then on the Pi's monotonic clock, `clock: "monotonic"`, `source: "robot"`), so a client written
against the sim works on the robot unchanged.

**Tree**: `GET /nodes/description/model/tree?variant=simple&base=swerve&mesh_uri=http` (these are its defaults):
`{schema: "robot_tree.v1", model_version, root: "base_link", links: {name: {visuals: [{type: box | cylinder |
sphere | mesh, pos, quat, rgba, size | radius (+ length) | url + scale}]}}, joints: [{name, type, parent, child, pos,
quat, axis, lower, upper, velocity, effort, mimic}], movable: [...]}`, joints parents first. A child's pose in its
parent = T(pos, quat) · motion(axis, q): a rotation about `axis` by q (revolute, continuous) or a translation along
it (prismatic); a mimic joint's q = multiplier · q(joint) + offset. `description/tree.py` `fk()` is the reference
(tested equal to pinocchio's FK of the same URDF). Metres, radians, quaternions [x, y, z, w].

`variant=boxes` draws the robot as the description node's page does, boxes only, with the same links, joints and
`movable` as `simple` (and the same `model_version`: the options are in `options`): each arm link's visuals are its
`ARM_BOXES` boxes (`nodes/arm/geometry.py`), axis-aligned in the link frame (`quat` identity, `size` full extents),
named `<link>_<part>` (`arm_link3_forearm`, `arm_end_link_gripper_rail`, ...); the jaw boxes ride on
`arm_finger_left_link` / `arm_finger_right_link` (mirrored in y), so `arm_finger_left` = gap / 2 opens them as the
page does; `arm_link6` and each `_wheel` link have no visuals; each module's steer link carries its wheel as a box
(it steers, it does not spin); the chassis and cameras are boxes as in `simple`. For example the forearm:

```json
"arm_link3": {"visuals": [
  {"type": "box", "name": "arm_link3_elbow_hub", "pos": [0.0065, 0.0026, -0.0311], "quat": [0, 0, 0, 1], "rgba": [0.7, 0.72, 0.74, 1], "size": [0.0578, 0.0399, 0.0802]},
  {"type": "box", "name": "arm_link3_forearm", "pos": [0.1222, -0.052, -0.0311], "quat": [0, 0, 0, 1], "rgba": [0.7, 0.72, 0.74, 1], "size": [0.2978, 0.0609, 0.0802]}]}
```

**State**: `GET /nodes/sim/robot_state` (one record) and `GET /nodes/sim/robot_state/stream?format=jsonl&hz=30`:
one JSON object per line (`application/x-ndjson`, never ends):

```json
{"schema":"robot_state.v1","source":"sim","seq":812,"t":27.0333,"clock":"sim","reset_seq":2,"t_host":986921.5531,
 "model_version":"59b6e83a1fe52a4b","frame":"world","base":[0.8525,0.7738,-0.00015,0.0001,0.0001,0.6812,0.7321],
 "joints":{"base_FL_steer":0.2341,"base_FL_wheel_joint":31.07,"...":0,"arm_joint1":0.0,"arm_joint2":-0.3,
           "arm_finger_left":0.012,"arm_finger_right":0.012},
 "objects":{"crate":{"pose":[1.2,0.3,0.125,0,0,0,1],"geom":{"type":"box","size":[0.25,0.25,0.25],"rgba":[0.85,0.55,0.2,1]}}},
 "overlays":{"markers":[{"name":"wbc/arm/target","kind":"frame","frame":"world","pos":[0.5,0.1,0.6],"quat":[0,0.7071,0,0.7071],"source":"wbc"}],
             "sim":{"running":true,"rtf":1.0},"arm":{"enabled":true,"fault":0}}}
```

`base` = base_link's world pose [x, y, z, qx, qy, qz, qw] (z up, m); `joints` = every movable joint of the tree by
name (arm joints, both fingers, each module's `_steer` and `_wheel_joint`; continuous joints unwrapped);
`objects` = scene objects (box `size` = full extents, cylinder `radius` + `length` along z); `overlays.markers` =
the markers input passed through (so whatever the WBC posts, targets, commanded poses, its limit flag and base
hint as labels or frames, reaches the client). New keys may be added; never renamed.

Reading it from a raw TCP client: send `GET /nodes/sim/robot_state/stream?format=jsonl&hz=30 HTTP/1.0`; through
nginx (robot, Docker) the body after the headers is plain lines. Uvicorn (the node directly, or the Mac's landing
page proxy) always uses chunked encoding: each record is its own chunk, so read lines and skip those not starting
with `{` (the chunk sizes), as `tests/test_sim_stack.py` does.


## Config (`default_sim.yaml`)

`physics` (timestep, integrator, gravity), `model.variant`, `floor`, `base` (servo gains, armature, wheel contact,
start pose), `components.arm` (fidelity, servo, gripper speed, reply latency, watchdog, start pose), `cameras.<name>` (`serial`, per stream `link`, `width`, `height`, `fps`, `fovy_deg`,
`delay`; `depth_range`, `jpeg_quality`, `imu: {link, delay}`), `render` (`shadows`, `reflections`), `scene.objects`,
`time` (`run`, `rtf`).

## Performance (MacBook, 2026-11-21, measured)

| | full model (meshes) | simple model (capsules) |
|---|---|---|
| physics alone | ~66 k steps/s (66 × real time) | ~68 k steps/s |
| render + encode, one stream (640×480 colour / 640×400 depth) | head 42 / 77 fps, wrist 54 / 101 fps | 111 / 139, 100 / 153 fps |
| real time, all four streams at 30 fps | rtf 1.00, each stream ~22 fps (one render thread is the limit) | rtf 1.00, all 30 fps |

macOS's GL lacks `ARB_clip_control`, so MuJoCo warns that depth precision is limited (fine at these ranges);
`MUJOCO_LOG.TXT` gets that warning in the working directory (git-ignored).

## Not done yet

- Dynamic fidelity (MIT PD motors, torque limits, backlash, friction, gripper force law), sensor noise (camera,
  IMU bias / noise), fault injection (dropped motor replies, camera stalls), friction / mass changes at run time.
- The Docker stack's GPU override and Tailscale sidecar are written but untested (no NVIDIA host / auth key here).
- Rendering parallelism (all four streams at 30 fps with the full meshes); the 3D view's own mesh rendering is
  in the browser, independent of it.
- Measured masses (base, cameras) and the wrist camera's mount pose (estimates in the description YAML); a mast.
