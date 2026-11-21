# Robot model and simulation, v1: draft

Status (2026-11-21): design settled with the owner (see "Decided" below). Built (v1, kinematic fidelity): Part A, the
model in the description node (`GET /model*`, docs/description_node.md "Robot model"), and Part B, the sim node with
the hybrid base, the kinematic arm, rendered cameras and the three backends (docs/sim.md), tested with the real nodes
in-process. Not yet: dynamic fidelity, the nodes' `--sim world` flags, docker compose (docs/sim.md "Not done yet").
Step 4 of the pre-fast-path plan; the fast path (`fast_path_v1.md`) is built and tested against it first.

## Goal

1. **One robot model, served by the description node:** a URDF of the whole robot (base, mast, arm, gripper,
   cameras, their calibrated mounts) assembled from components, each either a full model (the arm's URDF with
   meshes and inertias) or generated procedurally from dimensions. Clients (simulators, visualisers, planners,
   policies, erid) get it through the API instead of each keeping its own geometry.
2. **A MuJoCo sim world** that the existing nodes attach to through their hardware backends, so every node, page,
   lease, safety rule and client runs unchanged against the simulation. Two fidelities, chosen per component:
   **kinematic** (ideal tracking with the real latencies) and **dynamic** (MIT PD motors, gravity, torque limits,
   backlash, friction).

Out of scope for v1: Isaac (large-scale training later), photoreal rendering, deformables, a sim on the robot itself.

## Decisions already made

- **MuJoCo** for robot_utils development, tests and integration; Isaac later for training at scale.
- **The abstraction layer is the node's hardware backend** (`ArmBus`, the swerve drive, the camera backend, the UPS
  reader), not something between the nodes and the web interface: the sim must exercise the nodes' own code.
- **Both fidelities available, configurable per component** (e.g. dynamic arm on a kinematic base).
- **The description node owns the robot model**, including procedurally generated parts and mix-and-match with
  components that have full URDFs.

## Decided (2026-11-21)

1. **Dimensions:** the description served today is good enough (no new measurements needed for v1).
2. **The arm's `simple` variant: capsules** (fitted per link), not the `ARM_BOXES`.
3. **The base is a kinematic–dynamic hybrid.** The swerve modules are kinematic, with the planner's own model and
   bounds (steer rate, wheel speed and acceleration caps, as pose_servo's planner and limiter use): no motor, gear,
   steering or wheel inertia models. But there are **four steerable, rotating wheels that contact the ground**, and
   that is where kinematics ends and dynamics begins: the chassis (and the arm on it) is a free body moved by
   wheel–floor friction, so slip, pushing, payload and tipping emerge. Two details: each wheel servo's torque is capped
   (about the motors' stall torque: a blocked robot stalls instead of shoving with unbounded force), and each module
   gets a little compliance (a stiff spring or soft contact) so four rigid wheels stay loaded instead of one
   chattering.
4. **IPC: Unix sockets**, one per node backend. Today every simulator runs inside its node's process (`SimBus` as an
   object; the swerve's eight `SimulatedSpark` threads on an in-process virtual CAN bus; the camera and power fakes):
   no IPC exists, which is why there is no shared world. The world is its own process. The swerve attaches at the
   SPARK level: the `SimulatedSpark`s keep the CAN protocol (heartbeats, parameters, status frames) but take their
   motion from the world and send it their setpoints, so the swerve node's whole real code path stays under test.
   Shared memory for images only if a measurement asks for it.
5. **Rendering resolution and rate are configuration**, per camera.
6. **The sim node has no lease or e-stop**: it substitutes for the hardware, it doesn't command it. It exposes
   **time** (run, pause, step, real-time factor, sim clock), the **scene** (load, reset, add/move/remove objects,
   save) and, optionally, **privileged physics controls** (apply a force or wrench to a body, teleport the base or an
   object, change friction/mass, inject faults such as a dropped motor reply or a camera stall), clearly marked as
   sim-only and never exposed by a hardware node.

## What exists today

| piece | where | state |
|---|---|---|
| arm URDF, meshes, inertias, MuJoCo collision meshes | `rebot_ctl/urdf/DM/urdf/ReBot_Arm_DM.urdf` (+ `_mount_right`) | full: 10 links, 8 with mass, joint limits |
| arm simple geometry (boxes) | `nodes/arm/geometry.py` `ARM_BOXES` | used for collision checks and the agent's text geometry |
| base dimensions, swerve modules, wheel, drive gains | `nodes/swerve/constants.py` | robot geometry living in the swerve node (the review flagged it) |
| frames `base`, `arm`; camera mounts (nominal + calibrated) | description node, `default_description.yaml` + its calibration store | no mast, no masses, no URDF |
| per-node simulators | arm `SimBus`, swerve `--sim` (virtual SPARKs), orbbec `FakeBackend`, power `--fake` | independent toys: no shared world, no physics, `SimBus` has no read delay |

## Part A: the robot model (description node)

**Components.** The description YAML grows a `components:` list; each component has a parent frame, a mount pose
and a source:

- `urdf`: a file (+ meshes) from a package, e.g. the arm: `{source: urdf, file: rebot_ctl:urdf/DM/urdf/ReBot_Arm_DM.urdf,
  prefix: arm_}`. Optionally a `simple:` alternative (e.g. the arm's `ARM_BOXES` as capsules/boxes with the full
  model's masses lumped per link).
- `procedural`: generated from dimensions: `box`, `cylinder`, `swerve_base` (frame box + 4 modules with steer and
  wheel joints, from the dimensions now in `swerve/constants.py`), `mast`, `camera` (a box for the body, with the
  Orbbec frame names `<cam>_depth_optical`, `<cam>_color_optical` as child frames). Mass from a given value or
  density × volume; inertia from the primitive.
- Camera mounts take the **active calibration** when there is one, else the nominal pose.

**Assembly.** One URDF, root `base_link` (the existing `base` frame: floor level, centre of the wheels), fixed joints
to each component's root, component links prefixed to stay unique. Options:

- `variant=full|simple` globally, overridable per component (`arm=full,base=simple`);
- `base=fixed|planar|swerve`: fixed to the world (visualisers), a planar x/y/yaw joint triple (simulators that
  drive the base by twist), or the full swerve modules (dynamic base);
- `mounts=nominal|calibrated`.

**API** (new, read-only, no lease):

- `GET /model`: JSON: components, the frame tree, units, `model_version` (a hash of the YAML + active calibrations
  + component file hashes; changes when a calibration is activated).
- `GET /model/urdf?variant=&base=&mounts=&mesh_uri=package|http`: the assembled URDF text. `mesh_uri=http` rewrites
  mesh paths to the node's own mesh route so remote clients (erid) need nothing installed.
- `GET /model/meshes/{component}/{path}`: mesh files.
- `GET /model/bundle.zip?...`: URDF + meshes, for offline use and caching by `model_version`.

**Single source of truth.** Dimensions move into the description YAML (the procedural components read them); the
swerve node keeps its kinematic constants, and a test asserts they match the model (no silent drift). The
existing `/description` and `/transform` endpoints keep working, now computed from the same component tree.

## Part B: the MuJoCo sim world

**A sim node** (`robot_utils.nodes.sim`, its own page): loads the model (`/model/urdf?base=planar|swerve`, or a local
file), adds what URDF lacks from a sim config (actuators, cameras, sensors, contact parameters, a scene), steps
the physics (1 kHz), renders the cameras, and serves the node backends over local IPC. Its page: a 3D view, time
controls (run, pause, real-time factor), fidelity per component, reset, scene objects.

**Fidelity per component:**

| component | kinematic | dynamic |
|---|---|---|
| arm | joints follow `q_cmd` ideally (within the motor speed limits), with the real pipeline: a reading answers the previous tick's request | MIT PD per motor (`kp, kd, q_cmd, qd_ff, tau_ff` exactly as the bus sends), torque limits, gravity, joint friction; **output vs motor side**: motor and output joints coupled by a stiff spring with a backlash deadband, so the output encoder and motor feedback differ as on the robot |
| gripper | gap follows the goal | the force-limited law against contacts |
| base | hybrid (decided above): steer angles and wheel speeds follow the SPARK setpoints through the planner's rate and acceleration bounds, as torque-capped stiff servos; four wheels contact the floor; the chassis is a free body | same (the base has one model) |
| cameras | rendered RGB (encoded MJPG, as the camera sends it) and depth (Y16) at the configured profiles, with `t_sensor` / `t_capture` and the measured camera-to-host delays (head colour ~30 ms, depth ~17 ms, wrist ~47 ms) | same; plus noise, holes and exposure later |
| IMU | from the base body's accelerations and rates, 1 kHz | same, with bias and noise |

**Node backends** (one per node, selected by `--sim world[=URL]`; the existing toy simulators stay for unit tests):

- arm: `WorldArmBus(ArmBus)`: the same MIT frames, reply latencies and pipelined reads as `HardwareBus`; this is
  also the review's "pipelined fake bus" (step 7).
- swerve: the existing `SimulatedSpark`s on the virtual CAN bus, with their motion taken from the world (setpoints
  out, wheel states back), so the swerve node runs its full CAN path.
- cameras: `WorldCameraBackend` implementing the orbbec backend interface (catalog, start, wait → frames).
- power: the existing fake.

**Time.** Real-time mode (the world locked to the wall clock) for the web UI, the fast path and human testing;
stepped mode (the world advanced by the test, node clocks injected as the arm `Rig` does) for deterministic tests.

**Where it runs.** The Mac, erid or CI: nginx + every node in `--sim world` mode + the sim node, later as one
`docker compose up`. Never on the robot.

## Validation

- Unit: model assembly (frame tree equals today's `/description`; masses, limits and names), URDF loads in MuJoCo
  and pinocchio, procedural inertias.
- Behaviour: the arm and swerve node test suites run against the world backends in both fidelities.
- Sim vs real: replay a recorded episode's commands in the sim and compare (step responses, tracking error,
  latencies); fit backlash width and friction from real data.

## Open decisions

- Capsule fitting for the arm: from the meshes automatically (bounding capsule per link) or hand-tuned once.
- Which physics controls ship in v1 (proposal: time, scene, apply wrench, teleport; fault injection after).
- Floor and contact parameters (friction, compliance): defaults, then fitted from real pushes and slips.

## Order of work (proposed)

1. Description node: components, procedural primitives and `swerve_base`, URDF assembly, `/model*` endpoints;
   dimensions moved into the YAML with the swerve consistency test.
2. Sim node, kinematic fidelity: load the model, step, render the cameras.
3. Backends: `WorldArmBus`, the world drive, `WorldCameraBackend`; node suites passing against them.
4. Dynamic fidelity for the arm: MIT motors, backlash, gripper contact (the base is hybrid from step 2 on).
5. `docker compose` dev stack; then the fast path, built against it.
