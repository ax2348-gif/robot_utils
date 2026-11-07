# Estimator node

`python -m robot_utils.nodes.estimator` estimates the base's planar pose (x, y, heading) at
100 Hz by fusing the **swerve node's wheel odometry** (translation) with the **camera node's IMU
attitude filter** (heading). It moves nothing and needs no lease. Shared conventions (time,
framing, `/meta`, the browser toolkit): [nodes.md](nodes.md).

- Page: `http://<pi>:8110/`: trajectory map, pose, inputs, heading / yaw-rate / speed plots
- Interactive API docs: `http://<pi>:8110/docs`
- `<pi>` = `botcore-desktop`, `192.168.8.209` (LAN) or `100.80.153.120` (Tailscale)

## Quick start

On the robot, with the swerve (8102) and camera (8101) nodes running:

```bash
cd ~/robot_utils && .venv/bin/python -m robot_utils.nodes.estimator      # port 8110
```

Options: `--swerve URL` and `--camera URL` (defaults `http://127.0.0.1:8102` / `:8101`; `none`
disables one), `--host`, `--port`. Either upstream may be down or start later: the node
reconnects every second and reports what it is running on (`mode`).

Without hardware: run the camera node with `--fake` and the swerve node with `--sim`, point the
estimator at them, and drive the simulated base from the swerve page.

```python
from robot_utils.nodes.common.client import NodeClient
with NodeClient("http://botcore-desktop:8110") as est:
    est.post("/reset", {})                      # pose = origin (x forward, y left)
    print(est.get("/state")["pose"])            # latest pose as JSON
    for header, rows in est.rows("/stream"):    # every pose, 100 Hz
        print(rows["t"][-1], rows["x"][-1], rows["y"][-1], rows["theta"][-1])
        break
```

## How it works

**Odometry is the master.** Every swerve control tick (100 Hz) gives each wheel's steering angle
and travel (`drive_pos`). The change of travel along the (mid-interval) steering angle is that
wheel's displacement; the four are fitted to a body motion (dx, dy, dθ) by least squares
(`swerve.kinematics.robot_twist`). A fused pose is produced **at each tick's own sample time**
(the median arrival time of the four drive readings, the Pi's monotonic clock).

**Heading comes from the IMU, interpolated to the master's times.** The camera node's filter
records (~100 Hz) give the IMU's orientation `q` (IMU → gravity-aligned world) and the window's
bias-corrected delta angle `da`. The heading increment of a record is the world-vertical
component of that rotation, `(R(q)·da)_z`; their running sum is a heading-vs-time track (record
end times). For a tick at `t_k`, the heading increment is `ψ(t_k) − ψ(t_{k−1})` with ψ linearly
interpolated: the two streams don't need to be aligned or at the same rate. On a level floor
this is the base's heading change **whatever way the camera is mounted** (upright, facing
forward, tilted): no camera-to-base calibration.

**Waiting and fallback.** A tick waits until the IMU track covers its time (the IMU arrives a
few ms after the swerve tick; `imu_lead` reports the margin) for at most **100 ms**; then the
wheel-derived dθ is used for that tick and `imu_ok` = 0. Gaps in the IMU track (> 50 ms) are not
bridged. If the swerve stream stops for 0.5 s, IMU records become the master and only the
heading is updated (`master` = 2). If the camera is missing, it runs on odometry alone.

**Position** integrates the body displacement rotated by the mid-interval fused heading. An
**odometry-only** pose (heading from the wheels) runs alongside for comparison:
`omega_wheel − omega_imu` is wheel slip / scrub, and the two trails diverge when it builds up.

**Clocks.** Each upstream is synced NTP-style (`GET /time`, every 30 s); its timestamps are
mapped to this machine's clock (offset ≈ 0 when everything runs on the Pi). Output `t` is on
the estimator host's `time.monotonic()`, i.e. the Pi's clock when run there.

**Limits (start simple).** No absolute reference: x, y and heading drift without bound (heading
slowly, from the IMU's residual yaw bias; position with wheel slip). No covariance, no SLAM or
visual odometry. The world frame is the pose at the last `POST /reset`. Assumes a level floor.

## Endpoints

| request | what |
|---|---|
| `GET /state` | `state` (running / waiting), `error`, `mode`, `pose` (latest row as JSON), `rate_hz`, `imu_coverage` (share of the last 500 ticks fused with the IMU), `imu_still`, `inputs` (per upstream: url, connected, rate, last age, clock offset ± uncertainty, error), `events` |
| `GET /pose` | the latest pose row only |
| `GET /stream[?batch_ms=0&max_backlog_s=20]` | every pose: `schema` record (kind `"pose"`), then `rows` (float64) |
| `GET /fields` | columns, units and descriptions |
| `POST /reset {"x", "y", "theta"}` | set the pose (all optional, default 0); the distance restarts at 0 |
| `GET /events` | Server-Sent Events of `/state` |
| `GET /meta`, `/health`, `/time`, `/docs`, `/` | as every node (`/meta` adds `footprint` = half length/width of the wheel base, m) |

### Pose rows (`/stream`, `/state.pose`)

| column | unit | meaning |
|---|---|---|
| `t` | s | sample time (Pi monotonic): the swerve drive readings' time (IMU record end in heading-only mode) |
| `x`, `y` | m | position in the world frame (pose at the last reset; x forward, y left) |
| `theta` | rad | heading, CCW +, **continuous** (not wrapped) |
| `vx`, `vy` | m/s | body-frame velocity from the wheels |
| `omega` | rad/s | fused yaw rate (dθ/dt) |
| `vx_world`, `vy_world`, `speed` | m/s | world-frame velocity, its norm |
| `distance` | m | path length since reset |
| `x_odom`, `y_odom`, `theta_odom` | m, rad | odometry-only pose (heading from the wheels) |
| `omega_wheel`, `omega_imu` | rad/s | yaw rate from the wheels / from the IMU over the same interval (NaN if not covered) |
| `imu_ok`, `odom_ok` | bool | IMU heading used for this sample; wheel readings valid |
| `imu_lead` | s | IMU coverage end minus `t` when fused (how far ahead the IMU was) |
| `dt` | s | interval since the previous sample |
| `master` | enum | 1 swerve tick, 2 IMU record (heading-only mode) |

### curl / JavaScript

```bash
N=http://botcore-desktop:8110
curl -s $N/pose
curl -s -X POST $N/reset -H 'content-type: application/json' -d '{"x": 0, "y": 0, "theta": 0}'
curl -s -N $N/stream | head -c 2000 | xxd | head        # binary records
```

```js
// on another page (CORS is open); with common/ui.js: RU.rows(N + "stream", d => ...)
const N = "http://botcore-desktop:8110/";
setInterval(async () => { const p = await (await fetch(N + "pose")).json(); console.log(p.x, p.y, p.theta); }, 500);
```

## The page

- **Trajectory map**, accumulated **in the page** from `/stream` (the node keeps no history;
  reloading starts a new trail): fused trail (blue), odometry-only trail (grey, dashed),
  the robot's footprint and heading, world axes at the origin (x red, y green), 1 m grid.
  Controls: follow robot, heading-up view, show/hide the odometry trail, Recentre, Fit trail,
  Origin, zoom +/−, Clear trail (keeps the pose), Reset pose (`POST /reset` and clear). Drag pans
  (and turns follow off), the wheel zooms about the cursor, double-click recentres.
- **Pose** readout, **inputs** with their rates and clock sync, plots of heading (fused vs
  odometry), yaw rate (IMU vs wheels: the gap is slip) and speed over 20 s, and the event log.

## Measured

Locally (Mac, fake camera + simulated swerve, separate processes): 100.4 Hz output, IMU used for
100 % of ticks, upstream clock offsets ~0.02 ms (same machine). Driving the simulated base
0.6 m forward then 0.6 m left gave (0.589, 0.578) m (the shortfall is the simulated drive's
ramp-up). With a fake IMU that does not rotate, a 1 rad commanded turn left the fused heading at
0° and the odometry-only heading at 60.7°, as intended (the IMU wins).
