# Robot description node

`python -m robot_utils.nodes.description` keeps the robot's **frames** (base, cameras and their
internal frames) and **calibrates** where the camera sits on the base. It runs the calibration
routines itself (so a human on the page and an agent on the API get the same procedure and the same
stored records) and saves the results on the robot. Shared conventions: [nodes.md](nodes.md).

- Page: `http://<pi>/nodes/description/`: 3D (orbit) + top / side / front views of the robot and each
  camera from the active calibration (camera body, depth frustum from the intrinsics, optical / colour /
  IMU frames, where the image and the optical axis meet the floor, the nominal pose as a ghost), camera pose, ground-plane
  polygon editor, yaw calibration form with ABORT, calibration records, e-stop
- Interactive API docs: `http://<pi>/nodes/description/docs`

```bash
cd ~/robot_utils && .venv/bin/python -m robot_utils.nodes.description      # socket <runtime dir>/robot/description.sock
```

Options: `--swerve NAME|URL` (default the instance `swerve`; `none` disables the yaw routine),
`--data-dir` (default `~/.local/share/robot_utils/description`), `--name` (instance), `--port` (debug: TCP).

## Frames and conventions

- `base`: on the floor at the centre of the wheels; x forward, y left, z up.
- `arm`: the arm's URDF `base_link`, the underside of its 200 x 140 mm mount plate on joint 1's axis, axes as
  `base`. The plate is clamped flush with the frame's front and right faces on the 8.75 in top surface:
  (0.1159, −0.1459, 0.22225). The arm node reports its poses in this frame and reads this transform for
  `GET /geometry`.
- `robot` in `GET /description` (from swerve.constants): `frame_size` (17 in square), `frame_z` (the 2x1
  frame tubes, 2.75–4.75 in), `top_rail_z` (the 1x1 top rails, 7.75–8.75 in), `top_z`, `modules` (wheels).
- Camera frames come from the camera node's `GET /description`: `depth_optical` (the frame depth
  pixels unproject into), `color_optical`, `imu`; optical convention x right, y down, z out of the
  lens. Here they are named `<camera>/depth_optical` etc.
- A frame's pose in its parent: `p_parent = R p_child + t`; `rpy` = fixed-axis X, Y, Z (as URDF,
  `R = Rz(yaw) Ry(pitch) Rx(roll)`), radians; `xyz` metres.
- The description is a **flat map of frames, each naming its parent** (a tree). `GET /transform?from=a&to=b`
  returns the pose of `b` in `a` (`xyz`, `rpy`, `rpy_deg`, `rotation`, 4x4 `matrix`).

**What is calibrated:** the pose of `<camera>/depth_optical` in `base`, composed from

| part | from |
|---|---|
| x, y | the **nominal** pose: measure it (tape) and set it, `POST /cameras/<name>/nominal {xyz, rpy}` |
| roll, pitch | the active **ground** calibration (floor normal ↔ base z) |
| height (z) | the active **ground** calibration |
| yaw | the active **yaw** calibration (motion directions ↔ base x, y) |
| anything not calibrated yet | the nominal pose |

The rotation is a weighted Wahba fit over those vector pairs (floor normal weight 10, each motion
direction 1, the nominal axes 0.001 as a weak prior, so a ground-only calibration keeps the nominal
heading). `GET /description` shows, per camera, `pose_in_base`, which calibrations it came `from`,
and the per-pair `residual_deg` (large residuals = the calibrations disagree).

## Calibrations

### Ground plane (never moves)

`POST /calibrate/ground {"camera", "polygons", "frames": 10}`: takes the per-pixel median of N raw
depth frames, unprojects the pixels inside the polygons (normalised image coords, `[[x, y], ...]`,
origin top left; draw them on the page over `GET /cameras/<name>/depth.jpg`), and fits a plane
robustly (RANSAC, 1 cm inliers, then least squares). Without polygons it takes the largest plane
within 30° of horizontal (using the IMU's gravity) below the camera. Result: floor normal (camera
frame, pointing up), camera **height**, rms, inliers, and the angle between the floor normal and
gravity (a level-floor / IMU-to-depth check: the plane gives roll and pitch relative to the **base**,
the IMU relative to **gravity**; they agree only on a level floor).

### Camera yaw (DRIVES THE BASE)

`POST /calibrate/yaw {"camera", "vectors": [[1,0],[0,1]], "amplitude": 0.08, "frequency": 1.0, "cycles": 5, "confirm": true}`

For each base-frame vector: nudge the wheels along it (0.03 m/s for 0.4 s) and stop (so they are
aligned and never re-steer: reversing only flips the wheel speed), then translate back and forth,
`v(t) = A·2πf·sin(2πf t)` for N cycles (the base swings from 0 to 2A and back), **ω always 0**. Meanwhile
it records the camera's raw IMU (1 kHz) and every swerve tick.

Solve (time-free): the accelerometer samples of a run lie on a line in acceleration space. Its
**direction** is the motion direction in the camera frame; its **centroid** is the static specific
force (gravity reaction + bias). The first and last half-cycle are left out. The **sign** comes from
correlating the samples' projection on the line with the sign of the measured base acceleration
(from the wheel speeds; this is the only use of timing, and any lag under a quarter period still
gives the right sign). Two non-parallel runs also give the floor normal (cross product), compared
with the ground calibration's.

**Safety** (the robot may be on the floor and tethered):
- `confirm: true` is required; limits: amplitude ≤ 0.15 m, frequency 0.3–2 Hz, peak speed
  A·2πf ≤ 0.8 m/s, 2–20 cycles, ≤ 8 vectors, total travel ≤ 12 m.
- Refuses to start if the swerve lease is held, its e-stop is latched, or the robot is not still
  (gyro > 2°/s before the run). It takes the swerve lease for the run and always disables and
  releases it at the end.
- **Aborts** on: yaw rate > 15°/s averaged over 0.1 s, or heading drift > 5° (bias-corrected gyro
  about gravity), a lost or stalled camera/swerve stream (> 0.5 s), `POST /jobs/abort`, or `POST /estop`.
  Safety aborts also **latch the swerve e-stop**; a requested abort only stops, disables and releases.
  The rate is averaged because single gyro samples spike past 10°/s from vibration and the
  reversals while the base doesn't turn; the heading limit bounds any real rotation.
- The solver low-passes the accelerometer (zero-phase Gaussian, `accel_lowpass_hz`, default 2 × the
  frequency) before the line fit: on the robot ~75% of the accelerometer's power during the motion
  is vibration above 3.5 Hz (rms ~3.5 m/s², more than the 1.9 m/s² motion signal), which made the
  unfiltered fit useless (linearity 0.8, the two directions 116° apart; filtered: 0.01-0.04, 90.5°).
  Each vector reports `linearity_unfiltered` and `vibration_rms`.
- `POST /calibrations/yaw/{serial}/{id}/resolve` (page: Re-solve) solves a stored run again from its
  raw data with the current solver (optionally another `accel_lowpass_hz`): a new record, nothing moves.
- An aborted run keeps what it recorded (IMU and swerve rows, phase marks): `GET /jobs/{id}/raw.npz`
  (the page links it), also in `<data dir>/aborted/` (newest 5).

### Jobs and records

- One job at a time: the `POST /calibrate/*` call returns the job; `GET /jobs/{id}` (and `/state`,
  `/events`) shows its progress and result; `POST /jobs/abort` aborts it.
- Every run is saved under `<data-dir>/calibrations/<type>/<camera serial>/<id>/` (`meta.json`:
  inputs, results, quality, the camera description at the time; `raw.npz`: the median depth image
  or the IMU and swerve data). Download: `GET /calibrations/<type>/<serial>/<id>` and `.../raw.npz`.
- **Activation**: a run becomes the active one for its type and camera only if it passes the quality
  gate (ground: rms ≤ 20 mm, ≥ 2000 inliers; yaw: line linearity ≤ 0.3, sign correlation ≥ 0.5,
  angle between runs within 3° of the commanded angle); otherwise it is saved with the reasons.
  `POST /calibrations/<type>/activate {"serial", "id"}` chooses any record (or `"id": null` for none).
- **Retention**: the newest 5 per type and camera, plus the active one (never deleted).

### E-stop

`POST /estop` (anyone; the landing page's E-STOP ALL includes this node, its `/meta` says it is
actuated) aborts a running job and forwards an e-stop to the swerve node. It latches; new yaw runs
are refused until `POST /estop/clear` (this node's; the swerve node's e-stop is cleared there).

## Robot model

One URDF of the whole robot, assembled from the `components:` of the description YAML
(`default_description.yaml`; `model.py`). Read-only, no lease. Clients (the sim, visualisers, planners,
erid) take the geometry from here instead of keeping their own.

| component | source | what |
|---|---|---|
| `base` | procedural `swerve_base` | root `base_link` (= the `base` frame): the chassis (frame tubes and top rails as two boxes) and four modules, each a steer joint `base_<M>_steer` (vertical, at the module) and a wheel joint `base_<M>_wheel_joint` (axle, y when pointing forward). Dimensions in the YAML; a test asserts they equal `swerve/constants.py` |
| `arm` | `urdf` | `rebot_ctl:urdf/DM/urdf/ReBot_Arm_DM_mount_right.urdf` (the file the arm node's kinematics load; a test checks), prefixed `arm_`, mounted at the `arm` frame |
| `camera` | procedural `camera` | the head camera: a box body, `camera_depth_optical` at its composed pose (`{camera: camera}`), `camera_color_optical`, `camera_imu` from its camera node's factory frames (a default when unreachable) |
| `wrist_camera` | procedural `camera` | on `arm_end_link`, at an **estimated** pose (not measured yet) |

A component hangs from a `parent` link at a `mount`: explicit `xyz`/`rpy`, `{frame: <description frame>}` or
`{camera: <camera>}` (nominal refined by the active calibrations, or nominal with `mounts=nominal`). Procedural
types: `box` (`size`), `cylinder` / `capsule` (`radius`, `length`, along z), `sphere`, `swerve_base`, `camera`;
mass from `mass` or `density` x volume, inertia from the primitive (checked against MuJoCo's). Masses of the base
and cameras are estimates (marked in the YAML).

Options (query parameters of every `/model*` route):

- `variant=full|simple`, per component overridable (`simple,arm=full`). `simple`: each URDF link's geometry is one
  capsule bounding its collision meshes (principal axis, then the shortest segment that keeps every vertex inside),
  written as a cylinder plus two spheres (URDF has no capsule); masses and inertias stay the full model's.
- `base=fixed|planar|swerve`: modules rigid; a root `odom` with `base_x`, `base_y` (prismatic) and `base_yaw`
  joints; or the steer and wheel joints movable (a simulator adds the floating joint: URDF has none).
- `mounts=nominal|calibrated` (default calibrated).
- `mesh_uri=package|http|relative|file`: `package://<component>/<path>`, this node's
  `/model/meshes/<component>/<path>` (remote clients need nothing installed), `meshes/<component>/<path>`, or
  absolute paths on the robot.

Routes: `GET /model` (components, the frame tree with each link's parent joint and pose, `frames_in_base` at
zero joint positions, the movable joints with their limits, `description_frames`: description frame -> link, and
`model_version`), `GET /model/urdf` (header `X-Model-Version`), `GET /model/meshes/{component}/{path}`,
`GET /model/bundle.zip` (robot.urdf with relative mesh paths, the meshes it uses, model.json).
`model_version` hashes the YAML, the active calibrations, the camera nodes' factory frames and every component
file; it changes when a calibration is activated, not with the options. `/description` and `/transform` agree
with the model at zero joint positions (tested for every frame both have).

## API replies

Every JSON reply has a typed, described schema in `/docs` (`/openapi.json`). The ones that are not obvious:

- `GET /jobs`, `GET /jobs/{id}`, `POST /calibrate/*`, `/state` `job`: a `GroundJob` or `YawJob` by `kind`
  (`state` running | done | failed | aborted; `result` set once done; `raw` the recording of an aborted run).
  `POST /jobs/abort` returns the job, or `{"detail": "no job running"}`.
- `GET /calibrations?type=&serial=`: summaries, newest first, with `active` and `has_raw`.
- `GET /calibrations/{type}/{serial}/{id}`: the stored `meta.json` as is (a `GroundRecord` or `YawRecord` by
  `type`), including `camera` (the camera node's `GET /description` at the time) and, for yaw, `limits`. Keys
  an older record lacks are absent; unknown keys are passed through.
- `POST /calibrations/{type}/activate` and `/state` `active`: camera serial -> {type -> calibration id}.
- Yaw `checks`: `angle_<i>_<j>` -> {expected_deg, measured_deg, error_deg} per pair of vectors, and
  `floor_normal_vs_ground_deg` (a number) when a ground calibration is active.

## Camera node addition

`GET /description` on the camera node (`robot_utils.nodes.orbbec`): `root` `depth_optical`, the device
serial, conventions, `frames` (color_optical and imu with their factory pose in depth_optical:
`rotation`, `translation` in m) and the active profiles' `intrinsics`.

## Not done yet

- The rotation-based calibration of the camera's x, y on the base (deferred: measure them).
