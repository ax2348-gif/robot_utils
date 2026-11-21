# Sim world node

`python -m robot_utils.nodes.sim` runs a MuJoCo world of the robot (design: [plans/sim_v1.md](plans/sim_v1.md)).
The robot model comes from the description node ([description_node.md](description_node.md), "Robot model");
the arm, swerve and camera nodes attach to the world through their **hardware backends**, so their own code
(leases, e-stops, limits, the swerve node's CAN path, the camera owner) runs unchanged. Development machines,
erid or CI only, never the robot. Needs the `sim` extra (`mujoco`).

```bash
python -m robot_utils.nodes.sim                       # model from the local description node (instance "description")
python -m robot_utils.nodes.sim --model local         # assemble the shipped description here (nominal camera poses)
python -m robot_utils.nodes.sim --model http://botcore/nodes/description --port 8130
python -m robot_utils.nodes.sim --paused --config my_sim.yaml --variant simple
```

Options: `--model` (description node instance or URL, a `bundle.zip`, or `local`), `--config` (YAML merged over
`nodes/sim/default_sim.yaml`), `--variant full|simple`, `--paused`, `--rtf`, `--scene-dir`, `--name`, `--port`.
Page at the node root: time controls, base / modules / arm, objects, the sim-only push and teleport, camera
previews, events.

## What the world is

The description node's `GET /model/bundle.zip?base=swerve` (URDF + meshes), plus from the sim config: a free
joint on `base_link`, the floor (friction), actuators, cameras, a light, scene objects. 1 kHz, `implicitfast`.
The robot never collides with itself; it collides with the floor and objects.

| component | fidelity (v1) | model |
|---|---|---|
| base | hybrid (decided) | per module a steer position servo and a wheel velocity servo, each torque-capped near the motor's stall torque (45 N m steer, 18.3 N m wheel). Their references follow the SPARK setpoints through the planner's bounds: steer rate 4π rad/s, wheel speed 3 m/s, wheel acceleration 0.85 m/s² (`max_accel + max_alpha · WHEEL_R` of the planner caps). The chassis is a free body moved by wheel–floor friction; soft wheel contact (`solref 0.02 s`) is the modules' compliance (all four wheels stay loaded, tested). A disabled drive coasts (no torque), a disabled steer holds its angle; no setpoints for 0.1 s disables every module (a lost heartbeat). |
| arm | kinematic | the reference follows each MIT position setpoint within `joint_speed_max` (6 rad/s); a joint sent kp = 0 holds where it is. A stiff servo plus a `qfrc_bias` feedforward applied as joint torque keeps the joints on it, so the chassis carries the arm's weight and reaction. No MIT frame for 0.1 s while enabled: status 0xD (the motors' CAN watchdog); a lost backend connection likewise. |
| gripper | kinematic | the motor angle follows its goal (kp > 0), or closes / opens under a torque command, at `gripper_speed`; the fingers (gap = −q · 16 mm/rad) are force-capped, so they stop on an object. |
| cameras | rendered | head and wrist: colour (JPEG, as the camera's MJPG) and depth (Y16 mm, 0 outside `depth_range`) at the configured size / rate / field of view, rendered from the links `<camera>_color_optical` / `_depth_optical`. `t_sensor` = the host time of the rendered state, `t_capture` = `t_sensor` + the measured delay (head colour 30 ms, depth 17 ms, wrist 47 ms). No IMU, noise or exposure yet. |

`dynamic` fidelity (MIT motors, backlash, friction, gripper contact law) is the next step: the config already
has `components.arm.fidelity`; anything but `kinematic` is refused for now.

## Time

`POST /run {rtf}` locks the sim clock to the wall clock × rtf (a physics thread; if it falls behind by 0.1 s it
drops the backlog and counts it in `behind`); `POST /pause`; `POST /step {n | seconds}` advances by hand (meant for
paused). `/state` has `t` (sim s), `rtf_measured`, `step_rate`. Cameras render on sim time: a paused world
delivers no new frames (the camera node then reports a stall after 4 s). Tests run the world in-process:
`World.step(n)` for deterministic stepping, `World.run()` for real time.

## Scene and privileged controls

`GET /scene`; `POST /scene/objects {name, shape: box|cylinder, size | radius+length, mass, xyz, rpy|yaw}`;
`PUT /scene/objects/{name} {xyz, rpy|yaw}` (teleport); `DELETE /scene/objects/{name}`; `POST /scene/reset` (time
0, robot at its start pose, objects where added, commands dropped); `POST /scene/save {name}` (current poses to
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
| `camera-<c>.sock` | `info` → serial, profiles, intrinsics, `depth_to_color`; `start {streams}`; `stop`; `frames {after: {stream: seq}, wait: s}` → `frames: [{stream, format, width, height, seq, t_sensor, t_capture, t_sim, depth_scale, len}]`, payloads concatenated in that order |

## Backends (`nodes/sim/backends/`)

- `arm.WorldArmBus(sim, limits)`: an `ArmBus` (`ArmNode(lambda: WorldArmBus("sim", limits=...))`). Pipelined like
  `HardwareBus`: each `read` returns the world's state as of the previous read's request and sends the next;
  the first read waits for its own. The driver's setpoint check as rebot_ctl. **Measurement times** for the
  `Reading.t_meas` / `t_meas_mot` fields the arm refactor adds: every reading carries them as attributes and
  `measurement_times()` returns them; `t_meas` = when the requests that gave `q` were sent (the previous read),
  `t_meas_mot` = when the world answered the last MIT frames (the motor-side feedback), else the read reply.
  Wiring them into the row after the merge is a one-liner in the node.
- `swerve.start_world_sim(sim)` → `(sparks, interface, coupler)`: eight sparkcan `SimulatedSpark`s configured as
  `setup_sparks` would, on a virtual CAN bus, whose motion is the world's (`SwerveNode(interface)`). The steering
  SPARK turns its POSITION setpoint (absolute encoder, 0..1) into a module angle and reports the world's angle as
  that encoder reading (status 5); the drive SPARK sends its VELOCITY setpoint (m/s) and reports wheel speed and
  travel (status 2). One coupler thread exchanges all eight with the world every 2 ms. `stop_world_sim(...)`.
- `camera.WorldCameraBackend(sim, camera)`: the orbbec backend interface (`Owner(WorldCameraBackend("sim",
  "camera"), hub, ...)`): one colour profile (MJPG) and one depth mode, no IR, no IMU; frames handed over no
  earlier than `t_capture`.

Not wired into the nodes' command lines yet (`--sim world`): that touches the node packages. Until then the
integration tests (`tests/test_sim_integration.py`) construct the real `ArmNode`, `SwerveNode` and orbbec `Owner`
in-process with these backends against a world running in real time.

## Config (`default_sim.yaml`)

`physics` (timestep, integrator, gravity), `model.variant`, `floor`, `base` (bounds, servo gains, torque caps,
armature, wheel contact, start pose), `components.arm` (fidelity, speeds, servo, finger force, reply latency,
watchdog, start pose), `cameras.<name>` (`serial`, per stream `link`, `width`, `height`, `fps`, `fovy_deg`,
`delay`; `depth_range`, `jpeg_quality`), `scene.objects`, `time` (`run`, `rtf`).

## Performance (MacBook, 2026-11-21, measured)

| | full model (meshes) | simple model (capsules) |
|---|---|---|
| physics alone | ~66 k steps/s (66 × real time) | ~68 k steps/s |
| render + encode, one stream (640×480 colour / 640×400 depth) | head 42 / 77 fps, wrist 54 / 101 fps | 111 / 139, 100 / 153 fps |
| real time, all four streams at 30 fps | rtf 1.00, each stream ~22 fps (one render thread is the limit) | rtf 1.00, all 30 fps |

macOS's GL lacks `ARB_clip_control`, so MuJoCo warns that depth precision is limited (fine at these ranges);
`MUJOCO_LOG.TXT` gets that warning in the working directory (git-ignored).

## Not done yet

- Dynamic fidelity (MIT PD motors, torque limits, backlash, friction, gripper force law), sensor noise, the IMU
  (and the head camera's IMU stream), fault injection (dropped motor replies, camera stalls), friction / mass
  changes at run time.
- `--sim world` flags in the arm, swerve and orbbec nodes (other branch's packages), the power node's fake, a
  `docker compose` stack.
- A 3D view on the page; rendering parallelism (all four streams at 30 fps with the full meshes).
- Measured masses (base, cameras) and the wrist camera's mount pose (estimates in the description YAML); a mast.
