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
| arm | **dynamic** (opt-in) | each Damiao motor applies its latest MIT frame as joint torque every physics step (1 kHz): `tau = kp (q − q_m) + kd (qd − qd_m) + tau_ff`, clamped at the motor's torque limit (27 N m joints 1–3, 7 N m joints 4–6: robot config). **No gravity feedforward of the sim's own**: the node's `tau_ff` is the only one, so its model error shows as droop. kp = 0 floats (kd and `tau_ff` only). Off, disabled or after the watchdog: zero torque (the node's "damping"): the arm sinks against the joint friction onto its stops. Watchdog, status codes and the enable state across a reset as kinematic. Below: "Dynamic arm". |
| gripper | **dynamic** (with the arm) | the gripper motor's MIT law drives the left finger (x = −q · 8 mm; the right one follows through the mimic), clamped at the 4310's 7 N m (875 N on the finger); off: zero torque. |
| cameras | rendered | head and wrist: colour (JPEG, as the camera's MJPG) and depth (Y16 mm, 0 outside `depth_range`) at the configured size / rate / field of view, rendered from the links `<camera>_color_optical` / `_depth_optical`. `t_sensor` = the host time of the rendered state, `t_capture` = `t_sensor` + the measured delay (head colour 30 ms, depth 17 ms, wrist 47 ms). No noise or exposure yet. |
| IMU | MuJoCo sensors | the head camera's (`cameras.camera.imu`): accelerometer (specific force: +9.81 m/s² up at rest) and gyro at the link's origin, in its frame (the depth optical frame), sampled every physics step (1 kHz); `t_sensor` = the host time the sample stands for (real-time mode: the wall-clock anchor), `t_capture` = + `delay` (2 ms). No bias, noise or temperature model. The camera node's attitude filter and the estimator's fused heading run on it unchanged. The wrist camera has none (as the Gemini 305). |

`components.arm.fidelity` is `kinematic` (the default, unchanged) or `dynamic`
(`deploy/sim/sim_dynamic_arm.yaml`; `ROBOT_SIM_CONFIG=deploy/sim/sim_dynamic_arm.yaml python -m
robot_utils.nodes.sim.stack`); anything else is refused. `hello` on `arm.sock` reports it.

### Dynamic arm

What becomes physical: the commanded-vs-measured deflection (`q_cmd − q`: effort / kp), contact forces at handles
and objects (a push gives deflection ≈ contact torque / kp), the WBC's lead bound (its `lead_tau` is now a real
torque), the residual of the node's gravity model (none, by construction: below).

- **The law.** One affine MuJoCo actuator per joint, `force = ctrl + b1 q + b2 qd` with, per step, `b1 = −kp`,
  `b2 = −kd`, `ctrl = kp q_des + kd qd_des + tau_ff`: exactly the MIT law, with kd treated implicitly by
  `implicitfast` (the wrist's kd 2 on its ~1e-3 kg m² is unstable explicitly at 1 kHz). `forcerange` is the torque
  clamp. The reading's `tau` is the motor's (clamped) torque; `g_tau` the gripper motor's, MIT sign (closing > 0).
- **Transport.** A frame reaches the motor `command_delay` (2 ms, ESTIMATE: serial bridge + CAN) after the world
  receives it (a per-frame queue); the watchdog counts from receipt. The rest of the end-to-end lag (the node's 10 ms
  tick, its pipelined read) comes from the stack itself, so the 19–29 ms end-to-end figure is not put in here.
- **Reset.** The settle (0.3 s) holds the arm at the start pose with the kinematic servo (a teleport); then the
  motors' enable state carries over as before, with no frame until the node sends one (zero torque meanwhile).
- **URDF effort limits** (MuJoCo's per-joint `actuatorfrcrange`) are lifted in dynamic mode: the actuator carries the
  motor's limit. They equal it on the arm (27 / 7 N m); the finger's URDF effort (8 N) is no motor limit.
- **Gravity fitted to the calibrated real arm** (`gravity_fit`, `World._fit_arm_gravity`, at start). The node's
  feedforward is `g_pinocchio(q) × tau_scale` (0.84 on joints 4, 5: measured on the robot, where float holds). The
  URDF's masses (plus the wrist camera and finger links the node's model lacks) gave up to 1.3 N m more: float
  drifted ~40–90° in 10 s and the servo drooped 2° on the wrist. Now the masses and centres of mass of links 2–6 and
  the gripper are least-squares fitted to the node's model × `tau_scale` over 200 random poses in the joint ranges
  (a small ridge keeps them near the URDF's; masses 3.73 kg against 3.93), so stiffness stays the commanded kp.
  `tau_scale` scales joints 4, 5 but not 3, 6, which no mass distribution reproduces exactly; the residual is
  shared out by each joint's dry friction (rows weighted 1 / `coulomb`), i.e. what stiction must hold. Residual on
  200 other poses, max (rms) N m: joints 2–6 0.36 (0.17), 0.34 (0.17), 0.006 (0.004), 0.001, 0.0002, against
  1.33, 0.83, 0.68, 0.33, 0.035 before; `/state` has it (`gravity_fit`). The alternative, a motor torque gain of
  1 / `tau_scale`, would match exactly but scale kp and kd by 1.19 on the wrist (deflection no longer the commanded
  kp's): not taken.

| parameter | value | source |
|---|---|---|
| torque limit | 27, 27, 27, 7, 7, 7 N m; gripper 7 N m | robot config: each joint's motor model in the gain profile's hardware file (`rebotarm_dm_soft.yaml`: 4340P, 4310) → `arm/config.py` `MOTOR_TORQUE`: min(datasheet peak, TMAX of the MIT frame, motorbridge's model table 28 / 10) (`motor_torque_limits()`, `limits.py`); agrees with the URDF's effort limits |
| kp, kd, tau_ff | whatever the node sends (soft: kp 72, 72, 96, 18, 18, 18, kd 8, 8, 8, 2, 2, 2; gripper 1.0 / 0.05) | the node (its profile, its gravity model × `tau_scale`) |
| `armature` | 0.03, 0.03, 0.03, 0.001, 0.001, 0.001 kg m²; gripper 0.001 | ESTIMATE (rotor inertia × ratio², 4340P 40:1, 4310 10:1) |
| `viscous` | 0.1 ×3, 0.02 ×3 N m s/rad; gripper 0.1 | ESTIMATE (gripper: about the ~0.5 s stroke seen on the robot) |
| `coulomb` | 0.4 ×3, 0.05 ×3 N m; gripper 0.05 | ESTIMATE (MuJoCo `frictionloss`; 0.4 on the 4340Ps: above the gravity fit's 0.36 N m residual, a stiction a 40:1 planetary plausibly has. Higher on the wrist (0.1) put the hand ~5 mm off a target: the deadband is coulomb / kp) |
| `command_delay` | 2 ms | ESTIMATE |
| watchdog, reply latency | 0.1 s, 7 ms | as kinematic |

**Measured (MacBook, 2026-11-24).** With the node's feedforward (pinocchio × `tau_scale`), six poses
(`tests/test_sim_dynamic_arm.py`): **float** (kp = kd = 0) drifts at most 0.24° in 10 s (joint 4; the others
≤ 0.06°), was up to 92°; **servoing** (soft profile) the droop `q_cmd − q` is ≤ 0.06° on every joint, was 2.1° on the
wrist and 0.5° on the elbow before the gravity fit. Without `tau_ff` the wrist sags by g / kp (~0.1 rad). Pushing the hand into a fixed box with joint 1: 0.126 rad of
deflection, 9.0 N m motor torque, 8.8 N m carried by the contact (the rest is joint friction). In the stack (arm,
WBC enabled and holding, gripper check passed, no fault): `/state.arm.lead` `torque` at rest −0.58 N m (joint 3),
−0.36 N m (joint 4) of the 6 / 1.5 N m budgets, `binding` false. Physics cost: ~25 k steps/s alone.

**What the node and WBC show against a compliant arm** (not bugs of the sim):
- an arm that was never enabled (the stack's start: uninitialized, motors off) sinks onto its joint stops before the
  first enable, as the real one would;
- the WBC seeds its commanded joints from the measured ones at enable, so every enable steps the hold down by the
  current droop (~0.02 rad on the wrist each time before the gravity fit; now below 0.001 rad). The lead numbers
  above were measured before the fit.

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

### The rings scene (dynamic sorting)

`scene: {tabletop: false, rings: true}` (or `ROBOT_SIM_CONFIG=deploy/sim/sim_rings.yaml`): two concentric horizontal
rings turning in opposite directions about a vertical axis, red and blue objects scattered on both (`scene.py`
`rings_scene`), for sorting moving objects. Derived like the table:

- **size**: outer ring ⌀ **2.0 m**, **20 cm** wide (r 0.80–1.00); a 1 cm gap; inner ring **30 cm** wide (r 0.49–0.79);
  the centre is **hollow** (r < 0.49): an object knocked off the inner edge falls to the floor.
- **height**: the surface is at the table's **0.302 m** (the mount + 0.08 m): above the chassis top (0.222 m), in the
  arm's best top-down band.
- **placement**: the outer edge **4 cm ahead of the base's front** (x 0.256 m) at the start pose, the centre on the
  arm mount's line (y −0.146 m, the mount plate is flush with the right face), so the arm reaches radially over the
  outer ring onto the inner one. Along that line the outer ring spans x 0.256–0.456 and the inner ring x 0.466–0.766.
- **reach** (IK on the model's joint ranges, fingertip 5 mm above the surface at the mount's y, no arm geom touching
  the ring surface): straight down to **x 0.60 m** (the first **13 cm** of the inner ring), tilted 15° outward to
  **x 0.68 m** (21 cm). The same at other surface heights: 0.26 m → 0.62 / 0.68, 0.34 m → 0.58 / 0.66, 0.38 m →
  0.52 / 0.62 (down / 15°), so 0.302 m keeps the table's height. The inner ring's far half is out of reach by design:
  the rings bring everything past.
- **rotation**: `scene.rings_setup`: `outer_rim_speed` **+0.04 m/s** (counterclockwise seen from above, 0.04 rad/s, a
  turn in 2.6 min), `inner_rim_speed` **−0.04 m/s** at its outer edge (0.051 rad/s). Each ring is driven by velocity
  servos (`drive`, below) capped at 20 N m in all with a gain of 1000 N m s/rad (a 10 N m brake costs 0.01 rad/s); it
  stalls rather than shove the robot. Objects ride along by friction.
- **objects**: `per_ring` (default **8**) on each ring, half red, half blue, at `seed`-ed random angles (≥ 10 cm
  apart), radii (≥ 3.5 cm from the edges) and headings, named `red_outer3`, `blue_inner5`, …: 35 mm cubes, cylinders
  ⌀ 35 × 60 mm and 25 × 35 × 50 mm blocks (each horizontal width < 0.8 × `GAP_MAX` = 39.7 mm), 0.05–0.08 kg, the
  objects' friction (as the tabletop's). The same seed gives the same scene.
- **geometry**: each ring is **16 sector bodies**, each a few boxes with coplanar tops (edges within 3 mm of the
  circle, the boxes overlapping inward so there are no gaps) and 0.5 mm radial slits between sectors. Each sector
  has its own drive and is held at the first sector's angle (`mimic`), so the ring turns as one. One body per ring
  would couple every contact on it in the solver (with `noslip_iterations` that cost ~0.9 ms per step at 16 objects);
  sectors keep the coupling local. Each ring rides 5 mm above a thin cylindrical support wall (the outer one under
  the outer ring's inner part, 17 cm in from its edge, clear of the base's approach; the inner one 10 cm outside the
  hollow).
- **cost**: ~0.6 ms per physics step (the tabletop: ~0.12 ms); measured live at `rtf_measured` 1.0, `step_rate`
  ~1000 /s with occasional `behind` re-anchors on a loaded MacBook (2026-11-24). Fewer objects (`per_ring`) is the
  knob if the physics falls behind.

**Joint drives and mimics** (any articulated object): a link's `joint` may carry `drive: {velocity, max_torque,
gain}` (a MuJoCo velocity actuator: rad/s or m/s, N m or N, default gain 1000) and `mimic: <earlier link>` (a joint
equality holding it at that link's joint position). Both are validated like the other link fields, accepted by
`POST /scene/objects`, kept by save / load, and a reset restarts the drives at their velocity with the joints at 0.

**Start speeds and ratchets** (any articulated object): a link's `joint` may also carry `qvel0` (rad/s or m/s: the
joint's speed when the object is added or loaded and after every reset, which otherwise zeroes all velocities; set
after the reset's 0.3 s settle, so at time 0 the joint is at 0 moving at `qvel0`) and `ratchet: {direction: 1 | -1,
max_torque, gain}` (a one-way clutch: it moves freely along `direction` and is held against the other way with up to
`max_torque`; a MuJoCo velocity actuator at 0 whose force range is one-sided, default gain 1e5 N m s/rad, so 100 N m
against it creeps 1 mrad/s; it adds no drag going the free way, measured). Not a drive and a ratchet on one joint.
Both are validated like the other link fields (`qvel0` finite, a number not a bool), accepted by `POST /scene/objects`
and kept by save / load. A `qvel0` on a `mimic` leader does not start its followers: give them the same `qvel0`.

### The revolving door scene

`scene: {tabletop: false, revolving_door: true}` (or `ROBOT_SIM_CONFIG=deploy/sim/sim_revolving_door_dynamic.yaml`,
with the dynamic arm and `view_anchor` [0.6, 0, 0]: the robot ~1 m in front of the operator, the door beyond it): a
passive revolving door straight ahead of the robot, spinning when the scene starts and after every reset, slowing to
a stop on its bearing's friction (`scene.py` `revolving_door`).

- **The layout**: a glass drum, inner radius **1.50 m** (3.06 m outside, 2.2 m tall), set in a **facade** (the
  dividing wall: 10 cm thick, 2.2 m tall, running 2.5 m out from the drum on each side) across the robot's way, so
  the robot's side (near, −x) and the far side (+x) meet only through the drum. Its centre is on the robot's x axis
  at x **2.35 m**: the drum **0.60 m** ahead of the base's front at the start pose, the robot facing its centre.
- **The rotor**: **5 radial wings** (72° compartments; 8 left barely any room) on a ⌀ 12 cm post, 2.1 m tall, 1 cm off the floor and 2 cm
  short of the drum. Each wing is 6 mm tempered glass in a light aluminium frame, **53 kg** (15 kg/m² × 1.42 × 2.1 m
  plus 1.2 kg/m of frame), the rotor ~**202 kg m²** about its axis. It is passive:
  no motor, only a ratchet (below).
- **The path through ("nearly a full rotation")**: the two doorways (0.70 m chords) sit **side by side where the
  facade meets the drum** (the drum's +y side), separated by a **1.05 m** closed stretch of drum wall (below ~0.95 m one 72° compartment spans both doorways
  and the robot passes straight through: 0°): the **entrance** on the near side (front-left of the robot), the
  **exit** on the far side. A **ratchet** lets the rotor turn only counterclockwise (seen from above), so a compartment taken in
  at the entrance swings away from the exit, past −x, −y and +x, and lines up with the exit only after most of a
  turn. Proven kinematically (`tests/door_sweep.py`): a disc of the base's inscribed radius (0.216 m: every robot
  pose covers it, so where it cannot go the robot cannot), on a 2.5 cm grid, can get from the near side to the far
  side only after the rotor turns **232–292°** (whatever the rotor's angle when it goes in; 3° steps). Two things
  make that work: the closed wall between the doorways is wider than a compartment's reach across them, and the
  ratchet. Without it the same disc gets out with as little as **4°** of backward turn:
  the robot could push the wings back the short way. A physics check agrees: a 0.43 m, 20 kg box shoved at the exit
  from just inside the entrance for 3 s stays in the drum, the rotor does not turn back.
- **The fit** (the robot's footprint from the model at its start pose, the arm at `start_q`: 0.576 × 0.454 m, x
  −0.216…+0.360, y −0.238…+0.216 about the base's centre; the base alone is the 0.432 m `FRAME` square): the
  72° compartments leave room to turn in place with the arm out; a doorway is **12.3 cm** wider than the robot on
  each side. The robot can push the wings with the base or the arm; with
  18.6 kg on the floor and 0.75 N m of bearing friction, 50 N at 1 m starts the rotor at ~0.24 rad/s².
- **The spin**: `qvel0` **0.35 rad/s** counterclockwise (3.3 rpm, an automatic revolving door's pace: the wing tips
  at 0.52 m/s), slowing on **0.75 N m** of dry friction (`frictionloss`) and **0.75 N m s/rad** of damping (5/8 of the 8-wing
  door's, as its inertia): dω/dt = −(0.75 + 0.75 ω) / 202, so ω(t) = (0.35 + 1) e^(−t/270) − 1 until it stops at
  t = 270 ln 1.35 ≈ **81 s**. Measured (stepped): 0.350 rad/s at 0 s, 0.301 at 10 s, 0.254 at 20 s, 0.208 at 30 s,
  0.164 at 40 s, 0.122 at 50 s, **0.081 at 60 s**, 0.042 at 70 s, 0.004 at 80 s, **stopped at 81 s** (and stays
  stopped) after ~2.1 turns (771°). A reset (`POST /scene/reset`) puts the rotor back at 0 spinning at 0.35 rad/s
  again; so does a reload of the start scene.
- **The rigid-scene export** (docs/scene3d.md): the rotor is one dynamic body (`revolving_door/rotor`, hinge joint),
  its wings see-through (alpha 0.45; the drum 0.25, the facade 0.35); the drum and facade are static.
- **Cost**: the door alone ~0.04 ms per physics step (stepped, the robot idle). Live (the preset, `--nodes
  description,sim`, a loaded MacBook, 2026-11-24): `rtf_measured` 1.0, `step_rate` 999–1001 /s, `behind` 0 over the
  first minute; one re-anchor (`behind` 1) at a `POST /scene/reset`, none after; the rotor's angle tracked the stepped
  decay (8.76 rad at 32 s) and a reset set it back to 0 at 0.35 rad/s.

## Grasping

- **Jaw collision**: in every model variant the finger links collide as the **jaw boxes** (`arm/geometry.py`
  `ARM_BOXES`: the carriage and the prong, bounded from the visual meshes; the same boxes the arm pages draw), not
  their CAD meshes: MuJoCo collides a mesh as its convex hull, and the finger mesh's hull filled the space between
  the pads, so the jaws closed through objects. Visual meshes stay. The right finger follows the left through the
  URDF's mimic (a joint equality), stiffened to `contact.fingers.mimic_solref`.
- **Contact** (`default_sim.yaml` `contact:`, sim-only physics): elliptic friction cones, `noslip_iterations: 5`;
  fingers `priority 1` (their parameters govern jaw contacts), `condim 4` (torsional friction), friction 1.5,
  `solref [0.002, 1]` (rigid jaws on rigid objects: the stiffest MuJoCo integrates stably, 2 × the timestep); objects
  `condim 4`, friction 0.8. Without them (pyramidal, condim 3, default solref) the jaws sank ~4 mm into the objects
  and they slipped 4–10 mm in 2 s.
- **Holding while the arm moves** (`tests/test_sim_grasp_hold.py`: the node's own commands tick by tick, soft gains
  and gravity feedforward, the gripper through its force-limited law at the default 50 N; top-down grasp, lift 10 cm,
  carry 10 cm at 0.3 m/s, shake ±2 cm at 2 Hz, wrist pitch ±0.5 rad and roll ±0.8 rad at 1 Hz). Two causes of the
  slip seen in teleop (2026-11-24), measured: (1) **the jaws' contact parameters were not in force**: without a
  priority MuJoCo averages solref / solimp with the object's (0.02 s), so a 50 N squeeze sank 1.3 mm into each jaw,
  and the object crept out of the grip ~0.5 mm per wrist reversal (insensitive to friction, cone, condim, noslip
  iterations or impratio; proportional to the penetration). (2) **the kinematic finger was capped at 8 N** by the
  URDF's finger effort limit (MuJoCo's `actuatorfrcrange`), 4 N per jaw: objects turned and dropped. Slip / turn,
  max over the run, cube and can: dynamic 5.1 mm / 1.5° → **0.46 mm / 0.3°**; kinematic dropped (215 mm / 174°) and
  19 mm / 64° → 4.6 mm / 12.5° and 0.7 mm / 9.7°. The kinematic gripper still squeezes with the sim servo
  (2000 N/m × error, ~20 N per jaw on a 40 mm object) rather than the node's force limit, and a fast wrist roll turns
  objects in its jaws by ~10°: use the dynamic arm for grasping work.
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
| arm motor torque (dynamic) | 27, 27, 27, 7, 7, 7 N m; gripper 7 N m | `arm/config.py` `motor_torque_limits()`: the profile's motor models (4340P, 4310), min(datasheet peak, MIT TMAX) |

The motor figures (`VORTEX_STALL_TORQUE` / `_CURRENT`, `NEO550_FREE_RPM`, `NEO550_STALL_TORQUE` / `_CURRENT`,
`STEER_REDUCTION`) are REV datasheet values, added to `swerve/constants.py` next to the existing `VORTEX_FREE_RPM`.
The simulated SPARK is a speed clamp plus a torque cap (the two ends of the duty cap's torque–speed line, slightly
stronger than the motor in between). Not from the robot config, so listed as estimates: `gripper_speed` (6 rad/s of
the gripper motor: the robot sets no speed, the node sends the goal through MIT PD; ~0.5 s full stroke), the arm's
`watchdog` (0.1 s: the Damiao motors' CAN timeout) and `reply_latency` (7 ms, measured), the dynamic arm's
`components.arm.dynamic` (armature, friction, command delay: "Dynamic arm"). The Damiao peak torques (27 / 7 N m) are
datasheet values added to `arm/config.py`, as the REV ones. Sim-only keys: servo stiffness / damping and armatures,
contact and friction.

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
start pose), `components.arm` (fidelity, servo, gripper speed, reply latency, watchdog, start pose; `dynamic`: armature,
viscous, coulomb, gripper, command_delay), `cameras.<name>` (`serial`, per stream `link`, `width`, `height`, `fps`, `fovy_deg`,
`delay`; `depth_range`, `jpeg_quality`, `imu: {link, delay}`), `render` (`shadows`, `reflections`), `scene`
(`tabletop`, `cabinet`, `rings`, `revolving_door`, `view_anchor`, `objects`), `time` (`run`, `rtf`).

## Performance (MacBook, 2026-11-21, measured)

| | full model (meshes) | simple model (capsules) |
|---|---|---|
| physics alone | ~66 k steps/s (66 × real time) | ~68 k steps/s |
| render + encode, one stream (640×480 colour / 640×400 depth) | head 42 / 77 fps, wrist 54 / 101 fps | 111 / 139, 100 / 153 fps |
| real time, all four streams at 30 fps | rtf 1.00, each stream ~22 fps (one render thread is the limit) | rtf 1.00, all 30 fps |

macOS's GL lacks `ARB_clip_control`, so MuJoCo warns that depth precision is limited (fine at these ranges);
`MUJOCO_LOG.TXT` gets that warning in the working directory (git-ignored).

## Not done yet

- The dynamic arm's estimates measured on the robot (joint friction, rotor inertia, transport delay, the passive
  arm's sink rate); backlash; the motors' torque–speed line (back-EMF: only the peak torque is a limit); the
  motor's own fault codes (overcurrent, overtemperature); sensor noise (camera,
  IMU bias / noise), fault injection (dropped motor replies, camera stalls), friction / mass changes at run time.
- The Docker stack's GPU override and Tailscale sidecar are written but untested (no NVIDIA host / auth key here).
- Rendering parallelism (all four streams at 30 fps with the full meshes); the 3D view's own mesh rendering is
  in the browser, independent of it.
- Measured masses (base, cameras) and the wrist camera's mount pose (estimates in the description YAML); a mast.
