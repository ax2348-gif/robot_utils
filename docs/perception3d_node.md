# 3D perception node (`3d_perception`)

`python -m robot_utils.nodes.perception3d` produces **RGB-D frames in the colour camera's frame**:
for every colour frame, the camera's newest depth frame is reprojected into that colour image's
pixel grid **at the colour frame's capture time**, compensating the camera's motion between the
two captures. Per-pixel depth in the colour image, and the 3D point behind any colour pixel, are
one request away (`GET /point?u=&v=`): the basis for a click-a-point-and-servo-there tool. It moves
nothing and needs no lease. Shared conventions (time, framing, `/meta`, sockets, the browser
toolkit): [nodes.md](nodes.md).

- Page: `http://<robot>/nodes/3d_perception/`: colour image with the registered depth over it
  (RGB / blend / depth, opacity, range), hover readout of depth and 3D point, click = `GET /point`,
  compensation and grid settings, timing, inputs, events
- Interactive API docs: `http://<robot>/nodes/3d_perception/docs`
- Package `robot_utils.nodes.perception3d` (a module name cannot start with a digit); instance name
  and kind `3d_perception` (accepted by `serve.check_name`)

## Quick start

On the robot, with the camera node streaming colour and depth (its defaults; keep its `align` and
`frame_sync` **off**), and optionally the estimator and description nodes:

```bash
cd ~/robot_utils && .venv/bin/python -m robot_utils.nodes.perception3d    # instance '3d_perception'
```

or Start it from the landing page (`nodes.yaml` has the entry). Options: `--camera`, `--estimator`,
`--description` (instance names or URLs; `none` disables the latter two), `--scale 0.5` (output depth
grid = colour resolution × scale), `--stride 1` (depth subsampling), `--splat 1|2`, `--idle-s 30`,
`--allow-nominal`, and the usual `--name`, `--port`.

```python
from robot_utils.nodes.common.client import NodeClient
with NodeClient("http://botcore/nodes/3d_perception") as p:       # on the robot: NodeClient("3d_perception")
    q = p.get("/point", u=640, v=360)            # colour pixel (1280x720 colour)
    print(q["depth_mm"], q["point"], q["point_base"], q["mode"])   # m, colour camera frame / base frame
```

## Why (and what the camera SDK would do instead)

The Orbbec SDK can pair and align itself (`frame_sync` / `align` on the camera node), but that holds
every colour frame ~70 ms until its depth frame arrives and costs ~8.5 ms of the camera node's CPU
per frame ([orbbec_node.md](orbbec_node.md), Results). Instead the camera streams both unsynchronised
and this node pairs each colour frame with the **newest depth frame**. On this camera the colour and
depth captures are simultaneous but depth reaches the Pi about one frame period later (~90 ms at
10 fps), so the newest depth is always the **previous** capture: `t_color − t_depth` = 99.8 ms
measured (60–150 ms in general). A robot turning at 1 rad/s moves the image by fx·ω·dt ≈ 30 px
(1280x720) in 50 ms, so that time difference must be compensated.

## How it works

```
camera /stream?streams=color,depth&mode=raw ──► newest depth (t_d)   colour (t_c) ──┐
camera /imu/stream (100 Hz, q: IMU→world)   ──► QuatTrack (slerp)                     │ wait ≤ 60 ms for the
estimator /stream (100 Hz, x y θ)           ──► PoseTrack (linear)                    │ tracks to cover t_c
description /description (camera in base)  ──► T_bd                                  ▼
camera /calibration (K_d, K_c, E, R_di)     ──► Registrar ── p_c = E · M · p_d ──► z-buffer ──► latest frame
```

**Motion.** `M` is the pose change of the depth camera from t_d to t_c (`p(t_c) = M · p(t_d)` for a
static point), `motion.py`:

1. **Rotation from the camera's IMU** (the dominant term). The camera node's attitude filter gives the
   IMU orientation `q(t)` (IMU → gravity-aligned world) at ~100 Hz on the Pi clock, sensor time;
   slerp at t_d and t_c. With `R_di` = IMU → depth rotation (the camera's `/calibration`
   `imu.accel_to_depth`, identity on this unit): `R_rel = R_di · R_wi(t_c)ᵀ · R_wi(t_d) · R_di ᵀ`.
   Needs no base calibration.
2. **Translation from the base pose.** The estimator's planar pose (x, y, θ; continuous θ;
   linearly interpolated) and the depth camera's pose in the base `T_bd` (the description node's
   `cameras.<camera>.pose_in_base`, composed from its ground/yaw calibrations):
   `M_base = T_bdˉ¹ · T_wb(t_c)ˉ¹ · T_wb(t_d) · T_bd`. Its translation includes the lever arm of the
   camera about the base's rotation centre.

| `mode` (per frame) | rotation | translation | when |
|---|---|---|---|
| `full` | IMU | `M_base` | IMU, estimator and a **calibrated** camera pose (ground or yaw calibration active) |
| `base` | `M_base` (planar) | `M_base` | no IMU, estimator + calibrated pose |
| `rotation` | IMU | none | IMU only (no estimator, or the description has only the nominal pose, or the pose stream stalled) |
| `none` | identity | none | nothing usable (or `compensation: none`) |
| `no_depth` | | | no depth frame within `max_depth_age_s` (0.5 s): colour published with an empty depth |

`--allow-nominal` (or `POST /settings {"allow_nominal": true}`) uses an uncalibrated (tape-measured)
camera pose for the translation. A track is interpolated only between samples ≤ 100 ms apart and
extrapolated at most 50 ms (IMU) / 100 ms (pose) beyond its newest sample; otherwise that source is
dropped for the frame. A pose `reset_id` change (estimator reset) clears the pose track, so a pose
jump is never interpolated across.

**Waiting.** A colour frame is processed as soon as the live IMU and pose tracks cover `t_c`, at most
`wait_s` (60 ms) after it arrived. Measured: colour arrives ~29 ms after `t_c` (camera), IMU ~5–11 ms
and the estimator's pose ~23 ms after their times, so the median wait is 0.04 ms.

**Registration** (`register.py`, pure numpy). Each valid depth pixel unprojects along its ray
(`z · ray_d`), is moved by `E · M` (E = factory depth → colour extrinsic), projected with the colour
intrinsics **including lens distortion** (OpenCV rational / Brown-Conrady k1..k6, p1, p2; this colour
lens has k1 = −0.032, ~20 px at the corners at full resolution) onto the output grid, and z-buffered:
`np.minimum.at`, nearest depth wins. Output: uint16 mm of **z in the colour camera frame at t_c**,
0 = no depth. To make that cheap on the Pi:

- the depth camera has no distortion, so its rays are separable (x per column, y per row) and the
  rotation of the rays is a broadcast add per coordinate instead of a 3x3 product per pixel;
- the colour lens distortion, rounding and bounds checks are folded into one precomputed **lookup
  table** (ideal pinhole position in ½-pixel cells → output pixel index, or a trash slot), one gather
  per point (adds ≤ 0.25 px of rounding);
- invalid pixels are not compressed out (that costs more than it saves): they carry NaN to the trash slot;
- the image is processed in **cache-sized row bands** (32k pixels): whole-image float32 passes were
  ~3× slower per pixel on the Pi 5 (the arrays exceed its caches).

The output grid is the colour image scaled by `scale` (default 0.5 = 640x360): at 0.5 the depth camera
samples each output pixel ~1.8 times per axis, so there are no gaps. At `scale` 1 (1280x720) the
output is denser than the depth (fx 609 vs 415): use `splat 2` (each depth sample writes 2x2 pixels).

**3D point of a pixel** (`/point`, the page's hover): the depth at that output pixel (or the median of
the valid ones within `radius` if it has none), along the colour camera's undistorted ray through the
clicked pixel: `p = z · (x_n, y_n, 1)`, colour camera frame at `t_color`. With a calibrated camera pose
also `point_base = T_bc · p` (base frame at t_color; `T_bc` = `/calibration` `base_to_color`).

**Idle.** With no `/stream` client and no `/frame`, `/point`, `/color.jpg`, `/depth.png`,
`/calibration` request for `idle_s` (30 s; 0 = never idle) the node stops registering and
**disconnects all its input streams** (raw depth is ~8 MB/s; reading the 100 Hz IMU and pose rows costs
~10 % of a core). The next request reconnects; `/point` and `/frame` wait for a fresh frame (330–400 ms
measured), skipping frames until the motion tracks reach back to the depth frame's time.

## Endpoints

| request | what |
|---|---|
| `GET /stream[?color=true&depth=true]` | every fused frame, latest-wins (a slow client skips frames: `dropped`), binary records (below) |
| `GET /frame[?color&depth&max_age=0.5&timeout=3]` | the latest fused frame as one record (waits for one newer than `max_age`) |
| `GET /point?u=&v=[&norm=false&radius=2&max_age=0.5]` | depth and 3D point at colour pixel (u, v) of the latest frame (JSON, below) |
| `GET /color.jpg`, `GET /depth.png` | the latest frame's colour JPEG / registered depth as a 16-bit PNG (mm) |
| `GET /calibration` | `color` (full-resolution colour intrinsics + distortion: `/point` pixels), `depth_grid` (intrinsics of the registered depth = colour × scale), `depth`, `depth_to_color`, `imu_to_depth`, `base_to_color` (4x4 or null), `base_calibration`; `ready` false before the first frame |
| `GET /settings`, `POST /settings` | `scale`, `stride`, `splat`, `compensation` (auto / rotation / none), `wait_s`, `max_depth_age_s`, `idle_s`, `allow_nominal` (partial update; applies to the next frame, for every client) |
| `GET /state`, `GET /events` | `state` (running / waiting / idle), `error`, `mode`, `modes` (last 5 s), rates, `timing` (compute, CPU, t_color → published, wait, colour − depth time: median / p95 over 5 s), `latest`, `sources` (tracks, calibrations), `inputs` (per stream: connected, paused, rate, clock offset, error), `events` |
| `GET /meta`, `/health`, `/time`, `/docs`, `/` | as every node (`/meta` `kind` = `3d_perception`, `api` names the endpoints above) |

### Fused frame record (`/stream`, `/frame`)

`[u32 BE header length][JSON header][payload]` (docs/nodes.md framing), header `type: "fused"`.
Payload = the colour bytes then the depth bytes; `color.offset/len` and `depth.offset/len` locate
them (`offset` null and `len` 0 when left out with `color=false` / `depth=false`).

| header field | meaning |
|---|---|
| `seq` | fused frame counter |
| `t_color`, `t_depth` | capture times (camera `t_sensor`), s, Pi monotonic; `t_depth` null in `no_depth` |
| `dt_color_depth` | `t_color − t_depth`, s |
| `mode` | `full` / `base` / `rotation` / `none` / `no_depth` (table above) |
| `motion` | the applied 4x4 M (depth frame at t_depth → depth frame at t_color); `rotation_deg`, `translation_mm` its size |
| `color` | `format` (MJPG: the camera's JPEG bytes, untouched), `width`, `height` (1280x720), `offset`, `len` |
| `depth` | `width`, `height` (the `depth_grid`), `dtype` `<u2`, `units` mm, `frame` color_optical, `offset`, `len`, `valid` (fraction of pixels with depth) |
| `scale`, `stride` | registration parameters |
| `color_seq`, `depth_seq` | the camera node's frame sequence numbers |
| `t_color_capture`, `t_received`, `t_published`, `t_send` | camera SDK receipt, received here, published, sent (s) |
| `wait_ms`, `compute_ms`, `cpu_ms`, `latency_ms` | wait for the motion tracks, registration wall / CPU time, `t_published − t_color` |
| `dropped` | frames skipped for this client since its previous record |

```python
import numpy as np
from robot_utils.nodes.common.client import NodeClient
p = NodeClient("3d_perception")
for h, payload, _ in p.records("/stream"):
    if h["type"] != "fused":
        continue
    jpeg = payload[h["color"]["offset"]: h["color"]["offset"] + h["color"]["len"]]
    d = h["depth"]
    depth_mm = np.frombuffer(payload, "<u2", d["width"] * d["height"], d["offset"]).reshape(d["height"], d["width"])
```

### `GET /point`

`u`, `v`: colour pixel (full colour resolution, pixel centres at integers; `norm=true`: fractions
0..1 of width/height). Answer:

```json
{"u": 312.6, "v": 515.7, "depth_mm": 843, "source": "pixel", "n_valid": 25,
 "point": [-0.457, 0.225, 0.843], "point_base": [0.718, 0.383, 0.592], "frame": "color_optical",
 "ray": [-0.542, 0.267, 1.0], "window_mm": {"min": 838, "median": 843, "max": 851},
 "u_out": 156, "v_out": 258, "seq": 1219, "t_color": 186120.71, "mode": "full", "age_s": 0.032}
```

`source`: `pixel` (that pixel's depth), `window_median` (it had none: median of the valid pixels
within `radius` depth-grid pixels), `outside`, or null (no depth near it: `depth_mm`, `point` null).
`point` is in the colour camera frame at `t_color` (x right, y down, z forward, m); `point_base` in the
base frame at that time (null without a calibrated camera pose). A click-to-servo tool should use
`point_base` together with the estimator's pose at `t_color` (the robot may have moved since).

## The page

Colour image with the registered depth (turbo colormap, auto range = 2nd–98th percentile, or fixed)
blended over it; *depth* view = depth only. Hovering shows the pixel, its depth and its 3D point
(computed in the browser from the frame and `/calibration`, no requests); clicking calls `GET /point`
and shows the answer (what a click tool gets). Settings (compensation, grid scale, stride) are the
node's, shared by every client: switching compensation to `none` while the robot turns shows the
misregistration directly. Side panels: the latest frame's header, timing (5 s), inputs, events.

## Measured (2026-11-08, Pi 5, robot on the floor, stationary)

Camera: colour 1280x720@10 MJPG, depth 848x480@10 raw, no SDK sync/align. Defaults (640x360 grid,
stride 1). All frames in `full` mode with the estimator running and the camera ground+yaw calibrated
(they were in `rotation` while the estimator was not publishing).

| | value |
|---|---|
| registration (live), wall / CPU per frame | **15.0 / 14.2 ms** median, 16.4 / 15.4 p95 (17.6 / 15.7 with other nodes busier) |
| node process CPU, streaming to a client | 25–26 % of one core (≈ 15 % registration and publishing, ≈ 10 % reading the input streams) |
| node process CPU, idle (inputs disconnected) | < 1 % |
| camera node CPU for the extra raw depth client | +3 % (31.9 → 34.9 %) |
| `t_color` → published | **46–50 ms** median (≈ 29 ms camera exposure → Pi, ~1.5 ms transfer, 15 ms registration), p95 48–61 ms |
| `t_color` → received, client on the Pi | 49–53 ms median, p95 51–65 ms |
| `t_color` → received, Mac via nginx (Wi-Fi) | 59 ms median |
| `t_color − t_depth` | 99.8 ms (always the previous depth capture at 10 fps) |
| output | 10.05 fps (= camera), 0 dropped, 5.7–6.1 MB/s per stream client, 84–87 % valid pixels |
| wait for IMU / pose coverage | 0.04 ms median (pose arrives ~23 ms after its time, IMU ~5–11 ms, colour ~29 ms) |
| `/point` round trip | 2.4–3 ms on the Pi, 8.6 ms from the Mac; from idle 330–400 ms (inputs reconnect) |

Registration alone on the Pi 5 (synthetic 848x480 depth, 90 % valid, the robot camera's calibration):

| output grid | depth stride | splat | wall | valid output |
|---|---|---|---|---|
| 640x360 (default) | 1 | 1 | 14.3 ms | 86 % |
| 1280x720 | 1 | 2 | 40.1 ms | 86 % |
| 1280x720 | 1 | 1 | 21.9 ms | 39 % (gaps: output denser than depth) |
| 640x360 | 2 | 1 | 6.0 ms | 39 % |
| 320x180 | 2 | 1 | 3.9 ms | 86 % |

The first straightforward version (whole-image passes, float32 distortion per point, boolean
compression) took 63 ms for the default case; the lookup table brought it to 34 ms and the row bands
to 14 ms. The motion computation (two slerps, two pose interpolations, 4x4 products) is ~0.16 ms.

Not measured on the robot: the compensation under real motion (the robot is on the floor and was not
driven); it is verified on synthetic scenes (tests below).

## Tests

`tests/test_perception3d_core.py` (no I/O): a synthetic scene (a wall 3 m away, a box face 1.5 m away)
rendered as depth and ray-cast independently as ground truth in the colour camera at t_c, with the
robot camera's real intrinsics, distortion and extrinsic. Static registration and registration with
the true motion (1 rad/s yaw over 50 ms; rotation + 8 cm translation) match the ground truth
(≤ 0.07 % of pixels off by > 3 cm, median error 0.25 mm); without compensation 76–80 % are (median
error 70–79 mm: the turn also changes every point's z), and the box edge lands 27–36 px off at full
resolution (≤ 1 px compensated). Also: stride / scale / splat, distortion round trip,
the point query, slerp, track interpolation / extrapolation / gaps, and the modes and fallbacks with
a simulated robot on an arc (IMU orientation from a non-identity IMU mount, tilted camera with a lever
arm): `full` and `base` reproduce the true M, `rotation` its rotation.
`tests/test_perception3d_node.py`: the fake camera node and real servers: rotation mode, record
format, `/point`, `/frame`, images, settings, `/meta`, page; full mode with a stub estimator (whose
rows end with `reset_id`) and description; the fallback for an uncalibrated description; idle and
wake-up.

## Limits

- **Moving objects** are not compensated: only the camera's own motion is. Anything that moved
  between t_d and t_c (≈ 100 ms) is drawn where it was at t_d.
- **Disocclusion holes**: surfaces the depth camera did not see at t_d (behind edges, uncovered by the
  baseline or the motion) have no depth at t_c; they stay 0. Nearest-wins resolves occlusion
  (foreground over background) but not holes; `/point` falls back to the median of a small window.
- **Rolling shutter**: if the colour sensor is rolling-shutter, rows are exposed at different times;
  the node uses one `t_color` for the whole image. Not characterised on this camera.
- **Rotation-only mode** rotates about the depth camera's origin: the translation of the camera
  caused by the base turning about its centre (lever arm) and driving is not compensated. The
  translation needs the estimator and a calibrated camera pose (`full`).
- The estimator's pose is planar (x, y, θ on a level floor); pitch/roll wobble of the base is only
  captured by the IMU rotation.
- Timing relies on the camera's `t_sensor` for both streams and the IMU (one camera clock mapped by
  the SDK) and the estimator's swerve tick times; all on the Pi clock.
- One colour profile at a time: when the camera profiles change, the node re-reads `/calibration`
  (frames that don't match it are rejected until then).
- No accumulation across frames (each frame is registered on its own), no hole filling, no depth
  filtering beyond the camera's.
