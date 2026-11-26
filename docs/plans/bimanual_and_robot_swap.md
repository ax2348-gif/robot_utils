# Bimanual robot and robot swap (plan, 2026-11-24)

Goal: the stack and the sim run a robot chosen by configuration: today's single-arm reBot, a bimanual robot
(first a mirrored second reBot arm in the sim, then the OpenArm torso), and later simulators and benchmarks that
are not our robot. Robot-specific knowledge sits behind one interface where that is cheap.

Status (`feat/bimanual`): steps 1 to 4 below are built: the robot profile, the mirrored bimanual variant through the
description, the sim world and the arm nodes, the WBC node with N arms, two-hand Quest teleop, the pages,
agent_interface, collect and the sim stack. `--robot rebot_bimanual` runs the whole sim stack on two arms (smoke-run
end to end with the fake headset). Step 5 (a real bimanual robot) and the OpenArm torso remain.

## What is robot-specific today

Verified by reading the code (file:line as of `fix/sim-reset-arm-enable`); "assumed" marks what I did not check
end to end.

**Description node** (`nodes/description/`)
- `default_description.yaml`: the robot. Frame `arm` (front-right mount), components `base` (swerve_base),
  `arm` (rebot_ctl `ReBot_Arm_DM_mount_right.urdf`, prefix `arm_`), `camera`, `wrist_camera` (parent
  `arm_end_link`). `store.py` loaded only this file (plus the data dir's overrides).
- `model.py`: generic over components (any number of URDF components, each prefixed). reBot-specific: the
  `boxes` variant (`ARM_BOXES`, falls back to capsules for other URDFs), `package_root` knows `rebot_ctl` only.
  `model_version` hashes the description, calibrations and files: a robot swap changes it, as it should.
- Visual / collision element names of a URDF were not prefixed: using one URDF twice gave duplicate geom names in
  MuJoCo (fixed).
- `page.py` / `page_viz.py`: draw one arm (`frames_in_base.arm`, `../arm/links`).

**Sim** (`nodes/sim/`)
- `world.py`: `ARM_JOINTS = arm_joint1..6`, `has_arm`, one `Arm`, actuators `arm_joint*` / `arm_finger`, `_jaws`
  on `arm_finger_*`, `arm_op`, `_arm_reading`, `arm_lost`, snapshot `arm`. The arm model itself (`Arm`,
  `_arm_control`: reBot gripper constants, 6 joints, a kinematic servo) is reBot-specific.
- `node.py`: one backend socket `arm.sock`; robot_state.v1 overlay `arm`. `default_sim.yaml`: `components.arm`
  (start_q, servo, gripper speed, latency, watchdog), cameras by link name (`wrist_camera_*`).
- `limits.py`: arm joint speed / finger force from `arm/config.py` (the reBot's), swerve limits from
  `swerve/constants.py`. `stack_nodes.yaml` / `stack.py`: one `arm` instance.
- robot_state.v1 (`viz.py`): joints by model name: already generic.

**Arm node** (`nodes/arm/`): one reBot arm per process (the right design: one node instance per arm, one bus).
- Kinematics from rebot_ctl's hardware config URDF (`kin.py` `load_robot_model()`), the planner process
  rebuilds it by (limits, tcp) only. Its arm frame is the description frame `arm` (`queries.arm_in_base`,
  fallback the shipped yaml). `--sim world` always used `arm.sock`. `lock_names = {"arm"}` (both instances
  share the admin lock: conservative, kept). Gripper geometry, gains, `N = 6` are reBot facts.

**WBC** (`nodes/wbc/`): the math is per end effector, the node is one arm.
- Per-EE already: `model.py` `EESpec` / `WbcModel(urdf, ees)` (dicts keyed by EE), `controller.py`
  (`Snapshot.ees`, `Output.dq/qd` dicts, one QP over all EEs + the base, per-EE caps / kp / lead), `spec.py`
  (`Hints.ees`, `parse_hints`), datagrams `{"ees": {name: ...}}`, stream columns `<ee>_*`, `TargetBuffer` per EE.
- One arm: `WbcModel(urdf, (EESpec(...),))` with the default `"arm"` (`node.py:264`, `proc_node.py:136`);
  `--arm` = one arm node; single `arm_http`, `channel`, `arm_state`, `arm_lease_id`, `gap_max`, `Peers`,
  e-stop fan-out; the literal key `"arm"` in ~110 places of `node.py` and ~30 in `proc_node.py` /
  `control_proc.py` (commands, resync, gripper, lead bound, enable, limits, `/state`); `POSTURES` and stream
  columns `q_cmd1..6`; one `arm_age`.

**Quest teleop** (`nodes/quest_teleop/`, `tools/quest_teleop/`): `Config(hand="right", ee="arm")`: one
`Engagement` clutched by that hand's grip, gripper = that hand's trigger, output `{"ees": {cfg.ee: ...}}`,
`RobotState.from_row(row, ee="arm")`; the base reference (head walk, stick offset) is anchored to that one
engagement; e-stop to one `--arm`; oplog one `ee_*`. Hands telemetry already has both hands; buttons (Y/X arm,
A/B episodes) are global.

**Frontend / pages / recording**: instances are `{name, kind, start}`; two `kind: arm` instances work (sockets,
units and proxy paths come from the name). `page.html` `byKind("arm")` shows the first arm only; `viz_scene.py`
draws one arm. `collect` stream specs come from node names (`arm_left/stream` -> `arm_left.bin`): only the
default list says `arm/stream`. `agent_interface`: one `--arm`, `/arm/*` routes, geometry key `arm`.
`pose_servo`, `perception3d`, `estimator`: no arm assumptions found.

## The robot profile

A **robot profile** (`src/robot_utils/robots/<name>.yaml`, built) is what changes when the robot changes:

| key | what |
|---|---|
| `description` | the description YAML the description node starts from: `file: <pkg>:<path>`, or `extends: <profile>` + `overlay` (deep merge, `null` deletes). It holds the frames, components (URDF / procedural), cameras, **and the arms**. |
| `description.arms` | `{name: {component, joints, ee_link, gripper, tcp, base_link, side, type, posture}}`; reBot defaults. Absent: the one arm `arm` if there is an `arm` URDF component (the robot as it was). `side`: left / right (the hand that drives it); `type`: the arm type (`robots/arm_types.py`: `rebot`, `openarm_v2`); `posture`: the nominal posture (rad: the start pose, the WBC's "ready" and `null_mode: nominal`). |
| `sim` | merged over `default_sim.yaml` (per-arm `components.arms.<name>`, cameras, ...). |
| `stack` | `without`: registry instances that cannot run this robot yet. |
| `sim_only` | nodes refuse hardware. |
| `wbc` | the WBC's config defaults for this robot (`wbc/spec.py` Config keys; the node's `--config` overrides them), e.g. the OpenArm's arm-proximity weight. |

Decisions in the prototype (flagged for the user below):
- **One name per arm, everywhere**: the arm's name is its arm node instance, its description component (links and
  joints `<name>_*`), its description frame, its sim backend socket `<name>.sock` and its WBC end effector. No
  mapping tables.
- **The arms live in the description**, so they travel with the model: `GET /model` (and the bundle's model.json)
  has `arms` (`ModelArm`: name, prefix, joints, ee_link, base_link, gripper, mirror, frame, tcp). The sim world
  reads them from the bundle; the WBC would too. Consumers never need the profile file, only the description node.
- **Selection**: `--robot NAME` on the description, sim and arm nodes and on the sim stack; default
  `$ROBOT_UTILS_ROBOT`, else `rebot`. The stack writes a registry for the robot (`robot_registry`): description and
  sim get `--robot`, the `arm` entry becomes one entry per arm, `stack.without` instances are left out, and the
  start list's `arm` becomes the arm names. On the robot, `frontend/nodes.yaml` would list the instances the same
  way (not done: no bimanual hardware yet).
- **Per-arm node instances** take `--robot` and pick their arm by instance name (`Profile.arm(name)`; a one-arm
  robot's arm serves any name, so `--name arm2` still works on the default robot).

## The mirrored bimanual variant (built)

`robots/rebot_bimanual.yaml` (sim only): `arm_right` = today's arm (same URDF, same mount, renamed), `arm_left` =
its mirror image across the sagittal (x-z) plane at the front-left corner; the wrist camera stays on `arm_right`.

Mirroring (`robots/mirror.py`, description component key `mirror: true`): every frame is conjugated by
`M = diag(1, -1, 1)` (origins: y and roll / yaw negated; revolute axes `-M a`, prismatic `M a`; inertia `M I M`;
mesh scale y negated, which MuJoCo 4 and pinocchio accept). Consequences, all tested:
- the same joint values give the mirror-image pose ("mirrored convention"); joint limits, `start_q`, named poses
  and the arm config's `q_min_deg` carry over unchanged; gravity torques are identical at the same q;
- the left arm node loads the mirrored URDF for its own kinematics (`ArmSpec.urdf_path()`, a cache file in the
  temp dir; `ArmKinematics(urdf=...)`, also in the planner process), so its Cartesian API (IK, track, servo,
  geometry) is correct in its own arm frame;
- real hardware would need a per-joint sign map in the driver (the motors turn one way): the arm node refuses
  hardware for a mirrored arm or a `sim_only` robot.

Sim world: one `Arm` per arm (`arms` dict, `ArmIndex` per arm: qpos / qvel / actuators), per-arm config
(`components.arm` + `components.arms.<name>`), per-arm jaw collision boxes (mirrored for a mirrored arm), one
backend socket per arm (`<arm>.sock`; the one-arm robot keeps `arm.sock`), `/state` `arms` (and `arm` = the primary
arm: `arm`, else the first), robot_state.v1 overlay `arms`. `WorldArmBus(arm=...)`.

Default robot unchanged: same `model_version`, same joints and sockets; the existing sim, description and arm tests
pass. One visible change: URDF visual / collision names are now prefixed (`arm_finger_left_body`); nothing reads
them.

## Order to get there

1. **Profile + description + sim + arm nodes + stack** (built).
2. **WBC node, N arms** (built). The model's EEs come from the profile (`ee_specs(robot)`, `EESpec.from_arm`);
   `resolve_arms` maps each EE to its arm node (a one-EE model: `--arm` as before; several: the instance named as
   the EE). The arm's HTTP client, lease, joint channel, state, gains, limits and gap are dicts by EE (the primary,
   the first, under the old one-arm names); every check, hold, resync, pause, enable, disable and e-stop covers
   every arm; the armed latch and the base are shared, so one arm's fault holds both. The control process owns every
   arm's joint channel. The datagram, hint and stream formats were already per EE; `arm_age` is the oldest arm's;
   /state keeps `arm` (the primary) and adds `arms` with several. No arm-arm collision term (decided: the WBC
   does not need one). The single-arm robot is unchanged (same fields, same /state, the WBC tests pass).
3. **Teleop, two hands** (built). `Config.hands` (`left -> arm_left`, `right -> arm_right`, the default on a
   two-arm robot) runs one `TeleopCore` per hand (`MultiCore`): each grip clutches its own arm, each trigger is its
   own gripper, a released arm rides with the base, a hand that is not valid only rides. The base: no hand engaged,
   the sticks; one hand engaged, that hand's engagement (today's semantics); both engaged, the **first-engaged
   hand** keeps the base until it releases, then the other hand's engagement takes over (its base reference tracked
   the same head and stick motion since its own engagement, so it continues where the base is, up to the base's
   tracking error at that engagement). The page's mode / headless / top speed apply to both hands. `--hand` /
   `--ee` select one-hand teleop on a two-arm robot (default `arm_right`); the e-stop goes to every arm. The oplog
   keeps one `ee_*` set (the right arm's).
4. **Pages, agent_interface, collect** (built). Arm page, landing page and description page draw every arm on its
   description frame (`GET /links` says `mirror` for a mirrored arm); the WBC page shows each EE; the teleop page
   labels each hand's arm. agent_interface: `/arm/*` stays the first arm, `/arms/{name}/*` drives each with its own
   lease, enable / disable / e-stop cover all, `GET /arms`. collect's default streams record each arm
   (`--robot-profile`).
5. **The real bimanual robot**: its profile, `frontend/nodes.yaml` instances, per-arm hardware config (serial
   bridge / CAN channel per arm node: the `--channel` already exists).

## How the OpenArm torso slots in (built, sim only: `openarm_bimanual`)

**Collaborator releases (collab-v4 on):** Enactic's OpenArm v2.0 arm (Apache-2.0) with Anvil's joint ranges (J1
Enactic's window, J6 left −45..+70°, right −70..+45°) and Anvil's gripper (robots/openarm/README.md). Anvil's arm
model is not distributed; the update below describes the maintainer's model.

**Update 2026-11-25: Anvil's URDF** (branch `feat/anvil-urdf`; robots/openarm/README.md). The arms and gripper now come
from Anvil's own OpenArm v2 bimanual URDF (the owner's copy, no licence included: not for collaborator releases):
Anvil's meshes (multi-material OBJs), inertials, limits and gripper, in our conventions, J1 still Enactic's window.
What that changes below: J6 is Anvil's **left −45..+70°, right −70..+45°** (the 70° side tilts the fingertips toward
the wrist camera; the "ulnar" inference below had the sign reversed); the gripper is Anvil's URDF's (0-100 mm, the
hand 0.7 kg, the TCP 22.5 mm behind the fingertips, boxes measured from its meshes), not estimates; the collisions are
Anvil's convex decompositions (the gripper as boxes); the nominal posture was re-solved for the J6 range. The text
below describes the first build.

Branch `feat/openarm-7dof`. The robot profile `robots/openarm_bimanual.yaml` ("OpenArm 2.0, Anvil wrist (J6
-45..+70°) and gripper, Enactic J1 window"): the swerve base, a procedural torso (column + shoulder block) and two
**OpenArm v2.0** 7-DOF arms with the Anvil OpenArm 2.0's wrist range and **Anvil's Customizable Gripper** on the
wrist flange instead of OpenArm's pinch gripper.

- **Limits**: Enactic's standard 2.0 (upstream's URDF) but for J6, the Anvil OpenArm 2.0's −45..+70° (docs.anvil.bot,
  the "Anvil OpenARM 2.0" column), the extra 25° on the ulnar side (right −45..+70, left −70..+45: an inference from
  the zero pose, robots/openarm/README.md). J1 keeps Enactic's window (right −80..+200°, left −200..+80°), not Anvil's
  ±135°: the window's position is a mounting choice, not a joint property (280° vs 270° of span, centred 60° apart;
  re-clocking the J1 housing or another mounting plate shifts it), and Enactic's keeps the overhead forward reach,
  which matters more for human teleop than Anvil's 55° more backward. Upstream's geometry is kept (Anvil's wrist
  bracket: no CAD here).
- **Gripper** (`robots/grippers.py` `anvil`, a gripper type through every layer: the URDF, the arm node's gripper
  law and limits (gap, squeeze, gains from the type), the sim's fingers, jaw contact boxes and force cap, the block /
  page / description / headset views, the WBC TCP): Anvil's stated 0-110 mm, linear direct drive, DM-J4310-2EC,
  0 to −2.858 rad, 150 N rated / 350 N max (both fingers: the arm node's squeeze is 75 N default, 175 N max), 0.658 kg,
  ~170 mm flange to fingertips (Anvil's reach 650 − 480 mm). Boxes and inertias derived from those numbers (no CAD
  fetched): ESTIMATES. TCP 35 mm behind the fingertips (mid-finger). Grasp-hold in the dynamic sim: 30, 60, 80 and
  100 mm boxes lifted 10 cm, shaken ±2 cm at 2 Hz, slip under 0.5 mm (tests/test_openarm.py). The reBot keeps its
  gripper (`grippers.get("rebot")`, the same constants).

- **URDFs**: vendored from enactic/openarm_description (commit `c710e19`, Apache-2.0) in `robots/openarm/` with a
  README (source, licence, what was changed) and the generator `build_urdf.py` (upstream's xacro-expanded example
  URDF, no xacro needed). Upstream's own left and right models (no mirroring), their inertias and joint limits
  (efforts 40/40/27/27/7/7/7 N m), collision STLs as visuals (the visual meshes are COLLADA).
- **Mounting, human scale** (owner's rule): the shoulders at the shoulder height of a human whose arms are
  OpenArm's size. OpenArm's upper arm (joint2's origin, the shoulder centre, to joint4's, the elbow) is 0.220 m, its
  forearm (to joint6's origin, the wrist centre) 0.216 m (the URDF). Winter's anthropometric segment lengths (D. A.
  Winter, *Biomechanics and Motor Control of Human Movement*, 4th ed. 2009, after Drillis & Contini 1966): upper arm
  0.186 H, forearm 0.146 H, shoulder height 0.818 H. Shoulder to wrist 0.436 m = 0.332 H gives **H = 1.313 m** and
  **shoulder height 1.074 m** (the owner checked it independently; Enactic's "160-165 cm" claim is not tied to a
  dimension; Enactic's own v2 example URDF mounts the arm base_links 0.697 m above `openarm_body_link0`, ±0.03 m) (each arm's `base_link`: the shoulder centre lies in its plane), x 0.12, y ±0.03
  (OpenArm's body places its arms 0.03 m off its centre line). The torso (column + shoulder block, procedural) spans
  the base's top (0.222 m) to 1.164 m; the head camera at the human's eye height 0.936 H = 1.229 m, pitched 40° down.
  The work surface (table top, rings) follows the robot: the sim config's new `scene.work_top` (the reBot's mount +
  0.08 = 0.302 m by default), the profile's **0.70 m** (a standing light-work surface 10-15 cm below this human's
  elbow, 0.632 H = 0.83 m). At the old 0.302 m these arms cannot reach the surface ahead of the base at all (the
  fingertips go no lower than ~0.44 m). Reach at 0.70 m (fingertips 0-5 cm above it, the tool within 45° of down;
  sampled within the joint limits): out to x 0.63 m ahead of the base centre, 95 % within 0.44 m; on the rings (60k
  samples per arm): right 119 / 17 hits on the outer / inner ring, left 92 / 22. The rings centre on the robot
  profile's right arm mount (`World.ring_mount_y`, from the model's `frames_in_base`): the reBot's −0.146 m as
  before, the OpenArm's −0.03 m. The nominal posture (start pose): fingertips 0.34 m ahead of and 0.30 m below the
  shoulder (7 cm over the surface), the tool 50° down, the elbows out.
- **Block views** (the description page's 3D / top / side / front wireframes, the arm page's view, the description
  node's `variant=boxes` model, the agent render): an arm type's own boxes (`arm_types.arm_boxes`: each link's
  collision-mesh bounds, then its gripper type's boxes on end_link), served by its arm node's `GET links` (`boxes`)
  and used per arm; any joint count (end_link = the second-to-last frame); the torso drawn under the arms (GET
  description's `robot.bodies`). Before, OpenArm was drawn with the reBot's arm boxes on its frames, the gripper on
  joint 7, and the arms floated over the base.
- **Arm type seam** (`robots/arm_types.py`, `robots/openarm/arm_type.yaml`): DOF, MIT gains (OpenArm's), torque
  limits (the Damiao motors: DM8009 joints 1-2, DM4340 3-4, DM4310 5-7), the arm node's speed / acceleration caps,
  jam bands, the gripper type (Anvil's), the WBC's human-like geometry. `rebot` keeps every existing path. The arm
  node puts the type in its environment before importing `arm/config.py` (`N`, gains, limits), so its spawned
  control process agrees; the sim and the WBC load it per arm. The joint channel's packets carry n joints (the size
  gives n; the reBot's 88 / 136-byte packets unchanged).
- **Sim**: both arms dynamic (MIT law per joint, clamped at the type's torque limits; gravity fit per arm to its
  URDF; watchdog per arm): `deploy/sim/sim_openarm_dynamic.yaml`; rings: `deploy/sim/sim_rings_dynamic.yaml` with
  `--robot openarm_bimanual`. Friction / armature per joint: estimates (the profile's `sim:`).
- **WBC**: the DOF comes from the model; a redundant arm gets its null-space terms (docs/wbc.md "Null-space
  posture"): by default the human-like posture (a manipulability swivel model, effort, joint centering) and, for this
  robot, the strictly short-range arm-proximity term (the profile's `wbc:` block); the fixed nominal posture is kept
  as `null_mode: nominal` for comparison.
- **Teleop**: unchanged (hands by `side:`); the fake headset has a `hands_close` scenario (hands meet, the left
  reaches across, back).

Not done: OpenArm hardware (no CAN driver here: the arm node refuses it), the arm page's drawing (reBot boxes and 6
joints), the OpenArm torso's own mesh, measured friction / armature, the arm-proximity term for the 6-DOF reBot (no
null space: it could only act by trading EE accuracy, or with a masked orientation; not built).

## OpenArm workspace: reference vs the WBC (study, 2026-11-25)

Scripts: the session's scratch `bm/ws_*.py` (not in the repo). Reference: 100k random configurations per arm within
the URDF limits, the other arm at its nominal posture, kept when nothing intersects (the collision meshes as convex
hulls, as MuJoCo collides them; the arm's own non-adjacent links, the other arm, the torso, the base; pairs
intersecting at the nominal posture and in > 95 % of samples disabled as model artefacts: the two arms' overlapping
base_links, the left link1 in the shoulder block, the wrist camera on its hand). 96 % of configurations are
collision-free; 6800 cells of 5 cm per arm (850 l), 5.5 of 26 tool directions per cell on average. Anvil vs standard
J6: positions ±3 % (sampling noise), tool directions per cell +16 % with the Anvil wrist. (Measured on Enactic's
geometry with the inferred J6 sign; Anvil's URDF has it reversed, so these numbers need a re-run. The "left link1 in
the shoulder block" artefact was the old left arm's unmirrored base_link / link1 meshes, fixed with Anvil's model.)

**Two references.** The full-limit one above, and the **WBC-consistent** one: a target counts only if it is
reachable within the limits the WBC uses (the URDF's narrowed by `LIMIT_MARGIN` 2°; the arm node's welcome applies
the URDF's own for this arm type): its sampled configuration inside them, or a multi-start IK solution (40 seeds)
inside them that intersects nothing, or the WBC reached it. Misses split into those needing a joint inside the
margin and **true failures** (reachable within the WBC's limits, missed).

| Stratified targets (one per 5 cm cell, 200 per arm, from the nominal posture; < 5 mm, < 3°) | vs full limits | vs WBC-consistent | need the margin | true failures |
|---|---|---|---|---|
| base locked, QP alone | 64.3 % | 65.4 % (of 393) | 7 | 136 |
| base locked, + limit-aware swivel + limit repulsion (null space, opt-in) | 64.8 % | 65.9 % | 7 | 134 |
| base locked, the reconfiguration (whole-arm move, removed) | 75.3 % | 76.6 % | 7 | 92 |
| base locked, QP alone, margin 1° | 65.8 % | 65.8 % (of 400) | 0 | 137 |
| base locked, QP alone, margin 0.5° | 66.3 % | 66.3 % | 0 | 135 |
| base hint as teleop (operator still), QP alone (100 per arm) | 70.0 % | 70.7 % | 2 | 58 |
| the same, `base_start_charge` 0 | 74.5 % | 74.9 % | 1 | 50 |
| hint, + the null-space aids | 71.0 % | 71.7 % | 2 | 56 |
| hint, + the null-space aids, start charge 0 | 75.5 % | 75.9 % | 1 | 48 |

| Pull-up-top probe (162 hand poses at and above shoulder height, 68 reachable and collision-free) | vs full limits | vs WBC-consistent | need the margin | true failures |
|---|---|---|---|---|
| base locked, QP alone (margin 2°) | 51 (75 %) | 51 of 59 (86 %) | 9 | 8 (6 branch, 2 local) |
| base locked, margin 1° | 60 (88 %) | 60 of 67 (90 %) | 1 | 7 (6 branch, 1 local) |
| base locked, margin 0.5° | 60 (88 %) | 60 of 67 (90 %) | 1 | 7 |
| base hinted, QP alone / start charge 0 | 59 / 60 (87 / 88 %) | ~89-90 % | 2 / 1 | 7 / 7 |

The whole-space numbers barely move: the margin explains 7 of 143 locked misses; **the rest are true failures**. In
the pull-up-top region it is the other way round: 9 of 17 locked misses need a joint inside 2° (at 1° the probe reaches
60 of 68, as many as the hinted base). The teleop walks were already scored within the WBC's limits (unchanged).

**True failures, classified** (locked; each re-run from four swivelled starts and with local IK from where it
stopped): `branch` (the target's solution is on another IK branch than the one the arm reached: the hand would have
to leave its path) 112 of 136 (82 %); `local` (a local IK from the stop reaches it: the order the limits were met in)
15; `start` (reached from a swivelled start: an early elbow choice, what a preview or an MPC would make) 9. The
reconfiguration's 92: 81 branch, 9 local, 2 start (it fixes branch cases by moving the hand off its path, which the
owner ruled out). Base hinted: every true failure stops at a joint limit; 33 of 58 with the base at its 0.15 m band
(the base used all it may), the rest with the base inside the band and held at rest by the start charge (58 of 58
held at the end); with the start charge 0 the base is free and 50 remain (33 at the band). Pull-up-top: 6 branch, 1-2
local, none band-limited. None involve the proximity term. The missed region: behind the shoulder plane and across
the body (x < 0, the far side of the centre line).

**The margin** (2° on every joint; never changed here). In the dynamic sim (MIT law, the arm type's gains, gravity
feedforward), a joint commanded at the WBC's speed cap and braked at its acceleration cap to a stop at the narrowed
limit overshoots that stop by 0.7-1.9° (j1 2.0°: it reaches the hard stop; j2 1.9, j3 1.0, j4 1.3, j5 1.3, j6 0.7,
j7 1.4); the braking distances are 11.5° (j1-4) and 34° (j5-7). So 2° is what keeps a full-speed approach inside the
URDF limits; 1° would let every joint but j6 pass the URDF limit, by up to 1°, at full speed. What it costs: 1.5 points of
whole-space coverage, 13 points (9 poses) at the pull-up top. Option (not done): a speed-dependent margin, 0.5° at
rest growing to 2° at the cap (the overshoot scales with the approach speed), which keeps the braking guarantee and
gives the slow final approach of a grasp the extra 1.5°.

Teleop-like walks (each next pose within 0.25 m / 60°, 0.25 m/s, 3 s still; 72 poses), base hinted: operator still
5.7 mm mean EE error, 6 poses not settled, 14.6 s at a limit with the hand > 2 cm off (5.3 s of it with the base held
by the start charge); start charge 0: 4.2 mm, 2, 12.7 s, but twist sign flips 24 -> 73 per minute (chatter). Operator
walking (0.4 m/s, turning 0.3 rad/s, hand targets carried along with their feedforward): 7.9 mm mean EE error, the base
54 mm mean / 142 mm max from the hint (inside its 0.15 m band; slack <= 0.22), no start-charge holds. Solve time 1.4-2.8
ms mean (the limit-aware swivel: 2.2-4.1 ms, p99 8-17 ms).

**What remains and why**: 82 % of the true failures are IK-branch failures: from the nominal posture the velocity-level
QP follows the branch it starts on, and the target's solutions within the limits are on another; leaving the branch
needs the hand to leave its path, which the WBC must not do on its own. The null-space aids cannot change that (they
act within the branch). A preview of the swivel choice fixes the `start` class only (7 %).

**Recommendation.** (1) The operator's elbow (the swivel hint, docs/wbc.md "The operator's elbow"): the branch is the
elbow's choice, and the operator sees which one the task needs; a relative swivel hint lets them steer it, within the
null space (the hand stays). (2) For the pull-up top, the margin, not the controller: a speed-dependent margin
(above) is worth more there (9 poses) than any controller change. (3) A short-horizon MPC only for what remains: it
helps the `start` and `local` classes and the teleop stalls (it sees a limit coming along a moving target), not the
branch class of jump targets. Sizing: a 0.5-1.0 s horizon (the arm's 2 rad/s caps need ~1 s to swing a joint across),
20-25 steps of 40 ms, the decision [base (3), 2 x 7 joints] velocities: ~430 variables, the same costs (EE task,
posture, proximity) and hard rows (joint box, lead bound, reach, base caps) per step, linearised about the previous
solution (real-time iteration, one SQP step per tick). Solve time at 100 Hz: OSQP on the condensed QP ~2-5 ms on this
Mac; acados (HPIPM, a casadi model from pinocchio's casadi bindings) ~1 ms. Path: a sampled preview first (the current
QP's command rolled out 1 s along the target's motion with the swivel alternatives; a week), then the linear MPC behind
the existing `Controller` interface (2-3 weeks), the whole-body numbers above as its acceptance tests.

## Other simulators and benchmarks

The profile + description node are the seam: a benchmark robot is a profile whose description names its URDF;
the arm / base backends are the per-node `--sim world` sockets (protocol.py), so a different simulator can serve
the same sockets (arm protocol: hello, enable, disable, gripper_motor, check_gripper, send, read). Cheap next step
(not done): document the arm and swerve socket protocols as the integration contract, and let `ArmSpec` carry a
`backend` (world socket name) so a non-MuJoCo simulator can stand in per arm.

## Decisions for the user

1. **Naming**: `arm_left` / `arm_right` (prefix-sorted with `arm`) vs `left_arm` / `right_arm`. The single-arm robot
   keeps `arm` (unchanged), so `arm_right` is a rename of today's arm on the bimanual robot.
2. **Selecting a robot**: `--robot` + `$ROBOT_UTILS_ROBOT` (built). Alternative: the description node is the only
   place that knows, everything else asks it (`GET /model` `arms`): the sim already does; the arm node uses its
   own `--robot` because it needs its URDF before the description node is up (could fetch it instead).
Decided (2026-11-24): naming `arm_left` / `arm_right`; selection by `--robot` / `$ROBOT_UTILS_ROBOT`; left hand ->
`arm_left`, right -> `arm_right` (third person), one-hand teleop on the bimanual robot drives `arm_right`; the
mirrored convention as built; the shared `arm` admin lock for now; no arm-arm collision term in the WBC.

Still open:
1. **The base with both hands engaged**: first-engaged hand (built). Alternatives: the head alone (no hand anchors
   link 1), or the midpoint of both engagements' references.
2. **Per-arm admin locks** when one arm must be locked while the other works.
3. **The real OpenArm convention**: the vendor's left / right URDFs probably have their own joint conventions; the
   profile takes them as they are (`mirror: false`).
4. **One name per arm, fully**: the one-arm aliases (`World.primary_arm` / `world.arm` / `arm_q` / `finger_*`, the
   WBC's `arm_target` / `channel` / `arm_state`, /state `arm` beside `arms`) keep today's clients and tests working;
   drop them once every client reads per arm (not done in the integration with sim-integration: it would have
   widened the merge).

Rules settled in the integration (2026-11-24): every YAML layer merges with one helper (nodes/common/merge.py:
a dict merges, anything else replaces, a `null` deletes the key); the profile's `arms:` entries say their `side`
(left / right), which the two-hand teleop maps hands by; the sim's lenient arm watchdog is at least the arm node's
lenient `ctl_heartbeat` + 0.5 s (2.5 s), so a stall the node forgives never drops the motors.
