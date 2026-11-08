# Robot description node

`python -m robot_utils.nodes.description` keeps the robot's **frames** (base, cameras and their
internal frames) and **calibrates** where the camera sits on the base. It runs the calibration
routines itself (so a human on the page and an agent on the API get the same procedure and the same
stored records) and saves the results on the robot. Shared conventions: [nodes.md](nodes.md).

- Page: `http://<pi>/nodes/description/`: robot drawing from the active calibration, camera pose, ground-plane
  polygon editor, yaw calibration form with ABORT, calibration records, e-stop
- Interactive API docs: `http://<pi>/nodes/description/docs`

```bash
cd ~/robot_utils && .venv/bin/python -m robot_utils.nodes.description      # socket <runtime dir>/robot/description.sock
```

Options: `--swerve NAME|URL` (default the instance `swerve`; `none` disables the yaw routine),
`--data-dir` (default `~/.local/share/robot_utils/description`), `--name` (instance), `--port` (debug: TCP).

## Frames and conventions

- `base`: on the floor at the centre of the wheels; x forward, y left, z up.
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
- **Aborts** on: yaw rate > 10°/s or heading drift > 5° (bias-corrected gyro about gravity), a lost or
  stalled camera/swerve stream (> 0.5 s), `POST /jobs/abort`, or `POST /estop`. Safety aborts also
  **latch the swerve e-stop**; a requested abort only stops, disables and releases.

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

## Camera node addition

`GET /description` on the camera node (`robot_utils.nodes.orbbec`): `root` `depth_optical`, the device
serial, conventions, `frames` (color_optical and imu with their factory pose in depth_optical:
`rotation`, `translation` in m) and the active profiles' `intrinsics`.

## Not done yet

- The rotation-based calibration of the camera's x, y on the base (deferred: measure them).
- Node instance names / Unix sockets (the routing work): the node will take `--name` / `--uds` from
  the shared helper once it exists.
- Not yet run on the robot: everything above is tested with a fake camera and simulated swerve only.
