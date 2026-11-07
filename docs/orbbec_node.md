# Orbbec camera node

`robot_utils.nodes.orbbec` serves the robot's Orbbec Gemini 336L (color, depth, left/right IR,
IMU) over HTTP from the Raspberry Pi the camera is plugged into. It is the first node of the
HTTP-based robot framework: one process per hardware bus, FastAPI + uvicorn, binary streaming for
data, JSON for control, every timestamp on the Pi's `time.monotonic()` clock.

- Server: `src/robot_utils/nodes/orbbec/` (`python -m robot_utils.nodes.orbbec`)
- Client library: `src/robot_utils/nodes/orbbec/client.py`
- Examples: `examples/orbbec_node_record.py` (record on the client), `examples/orbbec_node_profile.py`
  (latency profile)
- Interactive API docs (OpenAPI): `http://<pi>:8101/docs`; quick-look page: `http://<pi>:8101/`
- Tests (no hardware): `PYTHONPATH=src python -m pytest tests/test_orbbec_node.py`

The older stdlib viewer (`examples/orbbec_web_viewer.py`, `docs/orbbec_viewer.md`) does the same
viewing job; this node adds a documented API, raw/lossless delivery for recording, the IMU, and
clock sync. Only one of them can hold the camera at a time.

## Quick start

On the Pi (needs `fastapi` and `uvicorn` in the venv, see [Dependencies](#dependencies)):

```bash
cd ~/robot_utils && source .venv/bin/activate
python -m robot_utils.nodes.orbbec --host 0.0.0.0 --port 8101           # Ctrl-C to stop
python -m robot_utils.nodes.orbbec --streams color,depth,ir_left --no-imu -v
python -m robot_utils.nodes.orbbec --fake --port 8765                   # synthetic camera, any machine
```

It prints a URL per interface (LAN `http://192.168.8.209:8101/`, Tailscale
`http://100.80.153.120:8101/`). Open it in a browser for live tiles, settings, device controls,
IMU readout and the true capture→display latency (the page syncs its clock to the Pi).

In the background with a log (how it was left running):

```bash
PYTHONPATH=~/robot_utils/src nohup ~/robot_utils/.venv/bin/python -m robot_utils.nodes.orbbec \
    --host 0.0.0.0 --port 8101 > /tmp/orbbec_node.log 2>&1 &
pkill -f "robot_utils.nodes.orbbec"          # stop
```

From any machine (Python):

```python
from robot_utils.nodes.orbbec.client import OrbbecClient

cam = OrbbecClient("http://100.80.153.120:8101")
print(cam.sync())                      # offset to the Pi clock, ± uncertainty
cam.update_settings({"streams": {"color": True, "depth": True}, "stereo_mode": "848x480@10"})
for fr in cam.frames("color,depth", mode="raw"):
    if fr.stream == "depth":
        mm = fr.depth_mm               # HxW float32 millimetres (0 = no depth)
    else:
        rgb = fr.image                 # HxWx3 uint8 (decoded from the camera's MJPEG)
    print(fr.stream, fr.seq, f"{fr.latency * 1e3:.1f} ms after capture")
```

## Architecture

```
            Pi: one process, one uvicorn worker, one asyncio event loop
 ┌──────────────────────────────────────────────────────────────────────────────┐
 │ owner thread "orbbec-owner"   (the only code that calls the SDK for video)    │
 │   open device / reconnect loop, Pipeline.start/stop, wait_for_frames(50 ms),  │
 │   D2C align, stamp + copy frames, execute commands (settings, properties)     │
 │        │ Hub.publish_frames()            ▲ queue.Queue of (cmd, cf.Future)    │
 │        ▼ (store latest per stream,       │                                    │
 │           loop.call_soon_threadsafe)     │                                    │
 │ IMU thread "orbbec-imu"  (own SDK Pipeline, 1 kHz accel+gyro, attitude filter)│
 │        │ Hub.publish_imu()  -> per-client bounded queues                       │
 │        ▼                                                                      │
 │ event loop (FastAPI)       never calls the SDK, never blocks                  │
 │   /stream generators await Hub "changed" futures -> latest frame per stream   │
 │   EncodeCache: one encode per frame+mode, run in a 3-thread pool, shared      │
 │   POST /settings -> owner queue -> await asyncio.wrap_future(...)             │
 └──────────────────────────────────────────────────────────────────────────────┘
```

- **One hardware owner.** The owner thread opens the device, starts/stops the video pipeline,
  waits for framesets and runs every command that touches the device. The API hands it commands
  through a `queue.Queue` with a `concurrent.futures.Future` each; the async handler awaits
  `asyncio.wrap_future(fut)`. The IMU runs on a second SDK pipeline in its own thread, started and
  stopped by the owner (an IMU inside the video pipeline gets bundled into video framesets: half
  the samples arrive, ~125 ms late).
- **Thread → asyncio handoff without polling.** `Hub.publish_frames()` stores the latest frame per
  stream under a short lock and schedules a wake-up with `loop.call_soon_threadsafe`. Stream
  generators take the current "changed" future *before* reading the latest frames (no lost
  wake-ups), send what is new, then await the future with a 1 s timeout (keep-alive ping).
- **Latest frame wins.** Each client remembers the last sequence number it sent per stream. A slow
  client skips frames (its `dropped` counter says how many) instead of building a backlog; the
  server's memory stays flat. Uvicorn's write flow control blocks a generator while the client's
  socket is full, which is exactly when it should skip.
- **Encode once.** JPEG / zlib encodes run in a small thread pool, once per frame and mode, and the
  result is shared by every client (the future is shielded, so one client disconnecting never
  cancels another's encode). Color MJPG is passed through untouched; raw depth/IR is sent
  zero-copy.
- **One worker, keep-alive.** uvicorn runs exactly one worker (a second process could not open the
  camera). HTTP/1.1 keep-alive (60 s) keeps `/time` exchanges on one warm connection.
- **CORS** is open (`*`) so a future dashboard on another origin can talk to several nodes.
- **Global settings.** One pipeline, so stream selection, profiles, alignment and display options
  are shared by all clients. Changes go through `POST /settings`; everyone sees them through
  `GET /state` or the `GET /events` SSE stream.
- **Robust.** A profile the device refuses, or one that starts but stays silent for 8 s, is
  reported (409) and the previous configuration restored. No frames at all for 4 s (unplug, USB
  error) or an SDK exception: the owner closes everything and reopens the device every 2 s,
  reporting `reconnecting` / `no device` in `/state`. A single stream that goes silent while
  others still run is reported as an error event.

## Endpoints

All responses are JSON unless noted. `<pi>` = `100.80.153.120:8101` (Tailscale) or
`192.168.8.209:8101` (LAN).

| Method & path | What |
|---|---|
| `GET /` | quick-look page |
| `GET /docs`, `GET /openapi.json` | interactive API docs / schema |
| `GET /time` | `{mono, wall, epoch_minus_mono}` for clock sync |
| `GET /health` | `{ok, state}` |
| `GET /state` | node state, error, event log, device, settings, per-stream fps/size/seq/age, IMU status, open streams, CPU |
| `GET /events` | Server-Sent Events: `event: state` with the `/state` JSON on every change and every 1 s |
| `GET /catalog` | color profiles, depth/IR modes, IMU rates/ranges, colormaps the device offers |
| `GET /settings` | current settings |
| `POST /settings` | partial update (see below) → `{settings, settings_version, restarted, t_applied}` |
| `GET /properties[?all=true]` | device properties the device reports read/write (exposure, gain, white balance, laser…) |
| `POST /properties` | `{id, value}` → updated list |
| `GET /stream?streams=…&mode=jpeg\|raw&compress=none\|zlib` | live frames, binary records (below) |
| `GET /frame/{stream}?mode=raw\|jpeg` | the latest frame as one binary record |
| `GET /snapshot/{stream}?format=jpg\|png` | the latest frame as an image (`png`: lossless 16-bit depth/IR); header JSON in `X-Frame-Header` |
| `GET /mjpeg/{stream}` | plain `multipart/x-mixed-replace` MJPEG (for `<img>`, VLC, OpenCV) |
| `GET /depth_at?x=&y=` | depth in mm at a point (fractions of width/height) of the latest depth frame |
| `GET /calibration` | factory intrinsics of the active depth/colour profiles, depth→colour extrinsic, IMU→depth extrinsic |
| `GET /imu/stream[?raw=false&batch_ms=0&max_backlog=20000]` | IMU attitude-filter records (~100 Hz), or every raw 1 kHz sample with `raw=true` |
| `GET /imu/latest?n=100[&raw=true]` | recent IMU filter records (or raw samples) as JSON |
| `POST /imu/calibrate` | calibrate gyro bias + gravity from the next `seconds` of still data |
| `POST /imu/reset` | restart the orientation from gravity (yaw 0) |

Errors are `{"detail": "..."}`: 400 invalid value / unknown profile, 409 the device refused the
configuration (previous one restored), 422 malformed body (unknown key, wrong type), 503 no camera
or no frame, 504 the camera thread did not answer within 20 s.

### Settings

`POST /settings` takes any subset of:

```json
{
  "streams": {"color": true, "depth": true, "ir_left": false, "ir_right": false},
  "color_profile": "1280x720@10 MJPG",
  "stereo_mode": "848x480@10",
  "align": false,
  "frame_sync": false,
  "display": {"colormap": "turbo", "invert": false, "range_mode": "auto", "pct_lo": 2, "pct_hi": 98,
              "min_mm": 200, "max_mm": 3000, "invalid_color": "#000000", "jpeg_quality": 80},
  "imu": {"enabled": true, "sample_rate": "1kHz", "accel_range": "4g", "gyro_range": "1000dps",
          "filter": {"output_rate_hz": 100, "kp": 0.5, "ki": 0.02, "accel_gate": 0.5, "zaru": true,
                     "zaru_tau_s": 5, "calib_s": 1}}
}
```

- `color_profile` / `stereo_mode` must be keys from `GET /catalog`. Depth, left IR and right IR
  always share one `stereo_mode` (the device requires it).
- `align`: SDK software depth-to-color alignment; aligned depth has the color resolution and
  implies frame sync.
- `frame_sync`: the SDK pairs color and depth of the same instant. Off (default), each frame leaves
  the SDK as soon as it arrives, which saves tens of ms of latency (see results).
- `display.*` only affects the monitoring (`mode=jpeg`) encoding of depth/IR and applies to the next
  frame. `imu.filter.*` also applies immediately. Everything else restarts the video pipeline
  (or the IMU), and the call returns once every enabled stream has delivered a frame.

curl:

```bash
curl -s http://<pi>/catalog | python3 -m json.tool | less
curl -s -X POST http://<pi>/settings -H 'Content-Type: application/json' \
     -d '{"streams":{"color":true,"depth":true},"color_profile":"640x480@30 MJPG","stereo_mode":"640x400@30"}'
curl -s -X POST http://<pi>/settings -H 'Content-Type: application/json' -d '{"display":{"colormap":"jet"}}'
curl -s http://<pi>/properties
curl -s -X POST http://<pi>/properties -H 'Content-Type: application/json' \
     -d '{"id":"OB_PROP_COLOR_AUTO_EXPOSURE_BOOL","value":false}'
curl -s -o depth.png "http://<pi>/snapshot/depth?format=png"     # 16-bit, units = X-Depth-Scale mm
curl -s -N http://<pi>/events                                       # SSE state stream
```

Python:

```python
cam = OrbbecClient("http://<pi>")
cam.update_settings({"stereo_mode": "480x270@60", "streams": {"color": False, "depth": True}})
cam.update_settings({"imu": {"sample_rate": "1kHz"}})
cam.set_property("OB_PROP_LASER_CONTROL_INT", 1)
```

JavaScript (browser or Node 18+; CORS is open):

```js
const base = "http://100.80.153.120:8101";
await fetch(base + "/settings", {method: "POST", headers: {"Content-Type": "application/json"},
  body: JSON.stringify({streams: {ir_left: true}})}).then(r => r.json());
const es = new EventSource(base + "/events");
es.addEventListener("state", e => console.log(JSON.parse(e.data).streams.depth.fps));
```

## Binary stream format

`GET /stream`, `GET /frame/{stream}` and `GET /imu/stream` send records:

```
[u32 big-endian N][N bytes UTF-8 JSON header][header.len bytes payload] ...
```

`streams=color,depth` selects streams; `mode` sets the encoding for all of them and a suffix
overrides it per stream: `streams=color,depth:raw` = color as MJPEG, depth lossless.

| `mode` | color | depth | IR |
|---|---|---|---|
| `jpeg` (monitoring) | camera MJPEG bytes, untouched | colormapped JPEG (`display` settings) | 8-bit JPEG (Y16 stretched) |
| `raw` (lossless) | camera MJPEG bytes, untouched (RGB/BGR/Y8 profiles: raw pixels) | uint16 little endian, `× depth_scale` = mm | Y8 or Y16 as captured |

`compress=zlib` adds level-1 zlib to raw payloads (`compression: "zlib"` in the header; depth gets
~2.5× smaller, costs Pi CPU).

Frame header (`type: "frame"`):

| field | meaning |
|---|---|
| `stream` | `color`, `depth`, `ir_left`, `ir_right` |
| `seq` | per-stream sequence number on the server (never resets while the node runs) |
| `frameset` | frames from one SDK frameset share it |
| `index` | the SDK's frame index |
| `dropped` | frames of this stream skipped for *this* client since its previous record |
| `width`, `height` | of the payload image |
| `format` | payload format: `MJPG`, `JPEG`, `Y16`, `Y8`, `RGB`, … |
| `source_format` | camera pixel format |
| `encoding` | `jpeg` or `raw` |
| `dtype` | numpy dtype of raw payloads (`<u2` for Y16, `\|u1` for Y8/RGB) |
| `compression` | `zlib` or absent |
| `depth_scale` | depth only: mm per depth unit (1.0 on this camera's default modes) |
| `aligned` | depth was aligned to color (has the color resolution) |
| `lo_mm`, `hi_mm`, `valid` | depth monitoring only: colormap range used, fraction of valid pixels |
| `t_sensor` | device timestamp mapped to the host clock by the SDK ≈ capture instant (null if unsupported) |
| `t_capture` | SDK received the frame on the Pi (its system timestamp) |
| `t_device_us` | raw device timestamp, camera clock (µs) |
| `t_owner`, `t_publish`, `t_encoded`, `t_send` | owner got the frameset, frame published, payload ready, record handed to the socket |
| `len` | payload bytes |

All `t_*` are **seconds on the Pi's `time.monotonic()`**. A `ping` record (`{"type":"ping","t_send":…,"len":0}`)
is sent after 1 s without data.

Parsing in Python is `RecordReader().feed(chunk)` → `[(header, payload), …]` and
`decode_payload(header, payload)` → numpy array (`protocol.py`); the client does both. In
JavaScript:

```js
async function* records(url) {
  const rd = (await fetch(url)).body.getReader(); let buf = new Uint8Array(0);
  for (;;) {
    const {value, done} = await rd.read(); if (done) return;
    const nb = new Uint8Array(buf.length + value.length); nb.set(buf); nb.set(value, buf.length); buf = nb;
    for (;;) {
      if (buf.length < 4) break;
      const n = new DataView(buf.buffer, buf.byteOffset).getUint32(0);      // big endian
      if (buf.length < 4 + n) break;
      const h = JSON.parse(new TextDecoder().decode(buf.subarray(4, 4 + n)));
      if (buf.length < 4 + n + h.len) break;
      yield [h, buf.slice(4 + n, 4 + n + h.len)]; buf = buf.slice(4 + n + h.len);
    }
  }
}
for await (const [h, body] of records(base + "/stream?streams=depth&mode=raw")) {
  if (h.type !== "frame") continue;
  const depth = new Uint16Array(body.buffer, body.byteOffset, h.width * h.height);  // little endian
}
```

IMU record (`type: "imu"`, `kind`: `state` = filter output, see [IMU](#imu); `raw` = samples as
below): `n` rows, `fields`, `units`, `dtype: "<f8"`, `seq_first`,
`seq_last`, `dropped` (samples dropped because this client fell more than `max_backlog` behind),
`t_send`. Raw payload: little-endian float64 array `(n, 11)` with columns

| column | unit |
|---|---|
| `seq` | server sample counter (gaps = samples missing) |
| `t_capture` | s, Pi monotonic: SDK received the sample |
| `t_sensor` | s, Pi monotonic: device timestamp mapped by the SDK |
| `t_device_us` | µs, camera clock (same counter as the video `t_device_us`) |
| `ax`, `ay`, `az` | m/s² (camera IMU axes; magnitude ≈ 9.8 at rest) |
| `gx`, `gy`, `gz` | rad/s |
| `temperature` | °C |

## Timestamps and clock sync

**Server clock.** Every timestamp the node sends is `time.monotonic()` of the Pi
(`CLOCK_MONOTONIC`: system-wide, so future nodes on the Pi share it; it never steps). The SDK
stamps frames with the host *wall* clock (`get_system_timestamp_us`, Unix epoch, at SDK receipt);
the owner converts each one with `t_mono = t_sys − (time.time() − time.monotonic())`, measuring that
offset fresh for every frameset because NTP can step the wall clock.

**Two capture times per frame.**

- `t_capture`: the SDK received the frame (after exposure, on-camera processing and USB transfer).
- `t_sensor`: the camera's own timestamp (`get_timestamp_us`, camera clock) mapped to host time
  by the SDK's device clock sync (`device.enable_global_timestamp(True)`,
  `get_global_timestamp_us`). This is the best estimate of when the image was taken.
  `t_capture − t_sensor` is the exposure-to-host delay (see the results).

The IMU samples carry the same pair. Video and IMU device timestamps come from the same camera
counter, and both are mapped by the same SDK clock model, so **use `t_sensor` to align IMU with
video** (`t_capture` adds a stream-dependent delay: 2–5 ms for IMU, ~29 ms for color, about one frame
period for depth/IR: 90 ms at 10 fps, 32 ms at 30 fps).

**Client ↔ server sync (NTP-style).** The client reads its monotonic clock `t0`, calls
`GET /time` (the server reads `ts = time.monotonic()` in the handler), reads `t1`:

```
offset      = ts − (t0 + t1) / 2         server − client
uncertainty = (t1 − t0) / 2              hard bound: the true offset lies in [ts − t1, ts − t0]
t_server    = t_client + offset
```

`OrbbecClient.sync()` takes 16 samples over a warm keep-alive connection, keeps the 8 with the
smallest round trip and intersects their bound intervals (at least as tight as the best single
sample; falls back to the min-RTT sample if they do not overlap). `start_auto_sync(30)` re-syncs in
the background; once the history spans 20 s it fits a linear drift (`ClockSync.drift`, s/s) so
`to_server()` stays good between bursts. Every received record gets `t_recv` (client monotonic) and
`t_recv_server` (mapped), so `latency = t_recv_server − t_capture` is end-to-end
capture→receive on one clock. The browser page does the same with `performance.now()` and shows
capture→display.

Achieved sync uncertainty: see [Results](#results).

## IMU

The 336L's IMU offers 50 Hz … 1 kHz (accelerometer ±4 g, gyroscope ±1000 °/s on this unit). The
node samples it at **1 kHz** (default) on its own SDK pipeline in the `orbbec-imu` thread,
independent of video restarts, and runs an **attitude filter** on every sample. Clients get the
filter output at **100 Hz** by default; every raw sample is still available.

### What `/imu/stream` sends

| request | records | columns |
|---|---|---|
| `GET /imu/stream` (default) | filter output, one record per 10 ms window (`filter.output_rate_hz`) | `imu_filter.STATE_FIELDS` |
| `GET /imu/stream?raw=true` | every 1 kHz sample | `protocol.IMU_FIELDS` |
| `&batch_ms=10` (either) | per client: after the first pending row, wait this long and send what accumulated as one record | |

Filter record columns (`header.fields`; units in `header.units`):

| columns | meaning |
|---|---|
| `seq`, `n` | record counter; raw samples in the window |
| `t_start`, `t_end` | window span, server monotonic s, **sensor time** (use these to align with video `t_sensor`) |
| `t_capture` | SDK host receipt of the window's last sample |
| `qw qx qy qz` | orientation at `t_end`: rotation IMU frame → world (world z up, yaw 0 at start/reset) |
| `dax day daz` | delta angle over the window, rad, IMU frame: every 1 kHz bias-corrected rotation composed exactly |
| `dvx dvy dvz` | delta velocity over the window, m/s, IMU frame (raw accel incl. gravity) |
| `ax ay az`, `gx gy gz` | mean accel (m/s²) and mean bias-corrected gyro (rad/s) |
| `bgx bgy bgz` | current gyro bias estimate, rad/s |
| `flags` | bits: 1 calibrated, 2 still, 4 initializing (orientation not meaningful yet), 8 sample gap |
| `accel_used`, `temperature` | fraction of samples that corrected the attitude; °C |

The delta angle/velocity pair is the standard pre-integrated IMU format: downstream fusion (the
estimator node) gets everything the 1 kHz stream carries about motion, at 100 Hz. Averaging or
decimating the gyro instead loses up to ~1° at 5–10 Hz motion in simulation.

**Frames.** The IMU frame equals the depth camera frame (`/catalog` → `imu.extrinsics`:
`accel_to_depth` is the identity rotation, ~16 mm offset; `accel_to_color` is within 0.2°): x right,
y down, z out of the lens. World is z up; the camera pointing straight up reads roll ≈ pitch ≈ 0.
The factory IMU intrinsics (`imu.intrinsics`) are placeholders on this unit (zero bias, identity
scale), so the node does not apply them; it calibrates bias and gravity itself.

### Filter

Mahony complementary filter in plain Python (numpy is ~13× slower for 4-element math):

- **Gyro integration** with the exact exponential map per sample, dt from sensor timestamps (host
  receipt is bursty: samples arrive in ~10 ms batches).
- **Gravity correction** (`filter.kp`, default 0.5 /s): the error between measured and predicted
  gravity direction is fed back, only while `| |a| − g_ref | < filter.accel_gate` (0.5 m/s²), so
  linear acceleration does not tilt the estimate. The same error trains the roll/pitch-axis gyro
  bias (`filter.ki`).
- **Zero-rate update** (`filter.zaru`, time constant `zaru_tau_s` 5 s): while the IMU is still (low
  gyro/accel spread over 0.25 s), all three biases relax towards the measured rate. This is the
  only thing that tracks the **yaw** bias, which gravity cannot observe.
- **Yaw drifts** without a magnetometer; with a calibrated bias the drift is tiny when still (below)
  but grows with motion and temperature. Heading must come from elsewhere (odometry, vision) in
  the estimator node.

All `imu.filter.*` settings apply at once, without restarting anything:

```python
cam.update_settings({"imu": {"filter": {"kp": 0.3, "accel_gate": 0.3}}})
```

### Calibration

- **At startup** (and after an IMU restart) the node skips the first 0.3 s (noisy on this unit),
  then takes `filter.calib_s` (1 s) of data: gyro bias = mean rate, `g_ref` = mean |a| (this unit
  reads **10.17 m/s²**, 3.6 % above 9.81, so gravity is calibrated, not assumed), initial roll/pitch
  from the mean accel. Records are flagged `initializing` until then.
- If the IMU moves during it (gyro or |a| spread too large, or a mean rate > 0.05 rad/s, which is
  a rotation rather than a bias), the calibration is refused: the filter runs with the previous
  bias (the stored one if any) and **retries automatically whenever the IMU is still** again.
- **On request:** `POST /imu/calibrate` (body `{"seconds": 2}` optional) — keep the camera still;
  returns the new calibration or 400 if it moved. `POST /imu/reset` restarts the orientation from
  gravity (yaw 0) keeping the calibration. Both are also buttons on the page.
- Successful calibrations are saved per camera serial in `~/.config/robot_utils/orbbec_imu.json`
  on the Pi (a few hundred bytes; `--imu-calibration-file` to change, `none` to disable) and used
  as the starting bias next time.
- `GET /state` → `imu.filter`: orientation (quaternion and roll/pitch/yaw), bias, calibration
  (bias, `g_ref`, noise, temperature, source), `calibrated`/`still`/`initializing` flags.

Measured on the robot (stationary, camera facing up; this unit at 34.6 °C):

| quantity | value |
|---|---|
| raw rate (device clock) | 1013.4 Hz, no gaps; host receipt in bursts of 0–13 ms |
| gyro bias | (−0.00096, −0.0017, +0.0059) rad/s; z = 0.34 °/s = **20 °/min of yaw drift uncorrected** |
| gyro noise | ~0.001 rad/s per sample (std); LSB 0.000533 rad/s = 1000 dps / 32768, confirming rad/s |
| bias stability | 1 s means vary by ~0.002 °/s |
| accel | |a| = 10.168 m/s² at rest, noise 0.016 m/s² |
| **filter, 120 s still** | yaw drift **0.014°** (raw gyro: ~41°), roll/pitch noise 0.0025° std, 101.3 records/s |
| record end → client (Mac, Wi-Fi LAN) | median 16 ms, p95 63 ms |
| CPU | IMU at 1 kHz + filter ≈ 14 % of one core (mostly the SDK sample loop; the filter math ~2 %) |
| filter cost per sample | ~2–3 µs on the Pi 5 (math), ~5.5 µs on an M-series Mac incl. windowing |

Simulated motion (1 kHz samples, filter at 1 kHz, tilt error): 0.04–0.12° RMS for 0.5–10 Hz
oscillations up to ±120 °/s.

```python
for rec in cam.imu():                          # filter output, ~100 Hz
    s = rec.samples                            # numpy record array, STATE_FIELDS
    print(s.qw[-1], s.qx[-1], s.qy[-1], s.qz[-1], int(s.flags[-1]))
for rec in cam.imu(raw=True, batch_ms=10):     # every 1 kHz sample, ~10 per record
    s = rec.samples                            # IMU_FIELDS
cam.imu_calibrate(2.0)                         # keep still
```

## Page visualisations

The page at `/` has a **Visualisation** box (left column) with two tiles, both built from the
public API in the browser (no CDN, no server-side rendering):

- **Orientation** (on by default): reads the 100 Hz filter records from `imu/stream`. A 3D view
  (drag to orbit) shows the camera body and its lens axis in the gravity-aligned world (z up),
  with world axes and a floor grid; the readout gives roll/pitch/yaw, calibration and still state,
  accel use and the gyro bias. Strip charts show the last 10 s of roll/pitch, yaw, |ω| and |a|, with
  still periods shaded green.
- **Point cloud** (off by default; it adds the raw depth stream, ~8 MB/s at 848x480@10): WebGL2
  unprojects every depth pixel on the GPU (depth as an R16UI texture, pinhole intrinsics from
  `/calibration`, pixel stride 1/2/4). Colour: each point is projected into the **colour camera**
  with the factory depth→colour extrinsic and sampled from the colour tile's latest JPEG, so no
  server-side alignment (and none of its ~70 ms) is needed; or by height (auto-ranged, 2nd–98th
  percentile) or distance. "Level with IMU" applies the filter orientation (and the IMU→depth
  extrinsic), so the cloud is gravity-aligned: floors are horizontal however the camera is held.
  Drag orbits around the cloud's median depth, wheel zooms, double-click resets.

Measured with the robot's camera facing a ~4 m ceiling (Mac on Wi-Fi LAN): 101,760 points at
stride 2, 10 fps (the camera rate), capture → drawn ~45–135 ms; the orientation tile's sensor →
display ~20–35 ms. Limits of the prototype: colour and depth frames are not time-synchronised
(colour may lag/lead by up to one frame), colour lens distortion is not applied (small on this
unit: k1 = −0.03), and each frame is drawn on its own (no accumulation across frames — that needs
a pose source and belongs in the estimator node).

## Recording on the client

The Pi does not record (small, slow disk). Recording is done by a client, which gets everything it
needs over the API: lossless depth (`mode=raw`, uint16 + `depth_scale`), IR as captured, color as
the camera's own MJPEG bytes, every IMU sample, and per-frame timestamps on one clock.

```bash
PYTHONPATH=src python examples/orbbec_node_record.py --url http://100.80.153.120:8101 \
    --out rec_001 --streams color,depth --mode raw --duration 30
PYTHONPATH=src python examples/orbbec_node_record.py --read rec_001 --export   # PNG/JPEG + CSV
```

In code:

```python
from robot_utils.nodes.orbbec.client import record, read_recording

record("http://100.80.153.120:8101", "rec_001", streams="color,depth", mode="raw", duration=30)
for h, img in read_recording("rec_001/video.rec", decode_jpeg=False):
    print(h["stream"], h["seq"], h["t_sensor"], h["t_capture"], h["t_recv"])   # img: uint16 depth / MJPEG bytes
imu = [s for _, s in read_recording("rec_001/imu.rec")]          # filter output, ~100 Hz
imu_raw = [s for _, s in read_recording("rec_001/imu_raw.rec")]  # every 1 kHz sample
```

The `.rec` files are the wire records with each header extended by `t_recv` (receive time on the
server clock), `t_recv_client` and `sync_uncertainty`; `meta.json` holds the settings, catalog,
device and clock-sync history. Over a slow link raw depth may not keep up with the camera; the
`dropped` field says how many frames were skipped (see the bandwidth column in the results).

## Results

Measured 2026-11-06 with `examples/orbbec_node_profile.py` (12 s per case after 1.5 s warm-up, IMU
off during video cases), node commit on `feat/orbbec-node`, Pi 5 (4 cores), camera on USB 2.1.
Raw JSON: `docs/orbbec_node_profile/*.json`. Three client locations:

- **Pi localhost**: the profiler on the Pi itself (`http://127.0.0.1:8101`).
- **Mac → Pi LAN**: `http://192.168.8.209:8101`; the Mac is on **Wi-Fi** (en0), the Pi on Ethernet.
- **Mac → Pi Tailscale**: `http://100.80.153.120:8101`; `tailscale ping` shows a *direct* path via
  the same LAN (192.168.8.209:41641), i.e. the same Wi-Fi link plus WireGuard.

### Clock sync

| client | RTT min / median | sync uncertainty (bound) | offset spread over 5 bursts |
|---|---|---|---|
| Pi localhost | 0.91 / 1.05 ms | ±0.45–0.48 ms | 0.02 ms |
| Mac → Pi LAN (Wi-Fi) | 3.7 / 4.7 ms | ±1.7–2.1 ms | 0.36 ms |
| Mac → Pi Tailscale | 5.3 / 5.8 ms | ±2.5–2.6 ms | 0.40 ms |

On localhost the true offset is known to be 0 (same `CLOCK_MONOTONIC`): the estimate came out at
+19…+27 µs, far inside its ±0.46 ms bound (the RTT is mostly httpx/uvicorn processing, nearly
symmetric). The bound is what to quote; the actual error is typically much smaller. Every Mac
latency below can be off by up to the sync uncertainty (all frames of a case share one offset).

### Capture → client receive latency

`t_recv_server − t_capture` per frame (SDK receipt on the Pi → last byte received by the client
process, client clock mapped to the Pi clock), median / p95 / p99 in ms. `cam fps` is what the
camera delivered; every client received every frame (**0 dropped** in every case and location, so
the received fps equals the camera fps throughout).

| case | stream | cam fps | KB/frame | Pi localhost med / p95 / p99 | Mac→Pi LAN (Wi-Fi) | Mac→Pi Tailscale | dropped (all runs) | Pi CPU % |
|---|---|---|---|---|---|---|---|---|
| color 720p10 | color | 9.99 | 126 | 1.4 / 1.6 / 1.8 | 7 / 48.3 / 67.7 | 8.2 / 47.6 / 71.6 | 0 | 2.5 |
| depth 848x480@10 jpeg | depth | 9.99 | 58 | 17.6 / 18.1 / 18.2 | 22 / 67.4 / 86.5 | 23.8 / 63.9 / 87.1 | 0 | 24.3 |
| depth 848x480@10 raw | depth | 9.99 | 795 | 12.3 / 12.5 / 12.7 | 24.3 / 69.5 / 92.2 | 26.8 / 74.3 / 91.2 | 0 | 13.2 |
| depth 848x480@10 raw+zlib | depth | 9.99 | 321 | 26.6 / 28 / 28.3 | 34 / 74.8 / 102.2 | 36.9 / 86.9 / 106.1 | 0 | 27.4 |
| color+depth defaults jpeg | color | 9.99 | 126 | 1.6 / 1.7 / 1.8 | 7.3 / 50.1 / 69.2 | 9.7 / 58.1 / 76.8 | 0 | 25.7 |
|  | depth | 9.99 | 58 | 17.5 / 17.9 / 18.2 | 22.2 / 65.8 / 83 | 23.6 / 67.3 / 87 | 0 |  |
| color+depth defaults raw | color | 9.99 | 127 | 1.7 / 1.8 / 1.8 | 5.1 / 52.8 / 71.7 | 8 / 58.6 / 75.5 | 0 | 14.9 |
|  | depth | 9.99 | 795 | 13.9 / 14.4 / 14.6 | 24 / 60.8 / 84 | 28.5 / 76.4 / 109.3 | 0 |  |
| color+depth defaults jpeg q50 | color | 9.99 | 126 | 1.6 / 1.7 / 1.7 | 7.3 / 55.2 / 70 | 8.5 / 57.2 / 73 | 0 | 26.1 |
|  | depth | 9.99 | 34 | 17.5 / 17.9 / 18.2 | 21.6 / 60.5 / 83.5 | 23.3 / 63.5 / 84.3 | 0 |  |
| color+depth defaults jpeg q95 | color | 9.99 | 126 | 1.6 / 1.7 / 1.8 | 7.5 / 56.4 / 72.8 | 8.3 / 56.1 / 69.7 | 0 | 26.8 |
|  | depth | 9.99 | 122 | 18.4 / 18.7 / 18.8 | 24 / 67.2 / 86.1 | 24.9 / 68.3 / 89.7 | 0 |  |
| color+depth defaults jpeg 2 clients | color | 9.99 | 126 | 2 / 2.3 / 2.4 | 8.4 / 54.7 / 71.9 | 9.4 / 52.2 / 70.1 | 0 | 27.3 |
|  | depth | 9.99 | 58 | 18 / 18.4 / 18.6 | 21.9 / 68.5 / 86 | 23.8 / 71.2 / 89.2 | 0 |  |
| color+depth defaults raw 2 clients | color | 9.99 | 126 | 2.2 / 2.7 / 2.8 | 7.9 / 63 / 74.7 | 8.5 / 56.7 / 77.2 | 0 | 17.5 |
|  | depth | 9.99 | 795 | 17.6 / 18.7 / 19.5 | 32.5 / 82.5 / 99.6 | 37.8 / 81.6 / 109.5 | 0 |  |
| color+depth defaults jpeg + IMU 200Hz | color | 9.99 | 126 | 1.6 / 1.9 / 1.9 | 7.1 / 56.2 / 72.5 | 7.9 / 55.4 / 74.6 | 0 | 31.9 |
|  | depth | 9.99 | 58 | 17.5 / 18.2 / 18.6 | 21.9 / 61.6 / 87.6 | 23.6 / 62.9 / 88.9 | 0 |  |
| color+depth defaults frame_sync jpeg | color | 9.99 | 126 | 72.3 / 72.6 / 72.7 | 77.8 / 114.9 / 140.9 | 79.2 / 119.6 / 143 | 0 | 26.2 |
|  | depth | 9.99 | 58 | 18.3 / 18.7 / 18.8 | 22.5 / 64.7 / 87.4 | 24.4 / 67.3 / 88.7 | 0 |  |
| color+depth aligned jpeg | color | 9.99 | 126 | 80.2 / 81.1 / 81.3 | 85.6 / 130.6 / 150.8 | 86.8 / 132.6 / 154.9 | 0 | 42.4 |
|  | depth | 9.99 | 104 | 35.4 / 37.2 / 37.7 | 40.1 / 82.8 / 97.9 | 41.6 / 88.1 / 107.5 | 0 |  |
| color+depth aligned raw | color | 9.99 | 126 | 80.5 / 81.4 / 81.5 | 84.3 / 126.9 / 147.9 | 85.6 / 133.6 / 150.3 | 0 | 23.9 |
|  | depth | 9.99 | 1800 | 26.6 / 27.8 / 28.1 | 44.5 / 96.1 / 106.4 | 52.8 / 117.5 / 122 | 0 |  |
| all four defaults jpeg | color | 9.99 | 126 | 1.7 / 1.8 / 1.9 | 7.2 / 52.2 / 69.7 | 8.1 / 55.5 / 76.2 | 0 | 36.5 |
|  | depth | 9.99 | 58 | 17.8 / 18.2 / 18.5 | 21.9 / 66.1 / 88.1 | 24.5 / 61.9 / 84.3 | 0 |  |
|  | ir_left | 9.99 | 66 | 7.7 / 8 / 8.1 | 12 / 53.6 / 74.6 | 13.7 / 55.6 / 79.1 | 0 |  |
|  | ir_right | 9.99 | 67 | 5.7 / 6.4 / 6.4 | 10.2 / 49.5 / 72.5 | 12.4 / 50.8 / 73.2 | 0 |  |
| all four defaults raw | color | 9.99 | 126 | 1.8 / 1.9 / 1.9 | 5.7 / 56.6 / 70.8 | 7.6 / 62.8 / 67.5 | 0 | 24.2 |
|  | depth | 9.99 | 795 | 13.9 / 14.4 / 14.5 | 24.8 / 80.1 / 91.1 | 28.6 / 84.5 / 98.7 | 0 |  |
|  | ir_left | 9.99 | 398 | 5.8 / 6 / 6.1 | 14 / 56.5 / 77 | 16.3 / 64 / 89.4 | 0 |  |
|  | ir_right | 9.99 | 398 | 5.6 / 7.1 / 7.2 | 10.7 / 52.8 / 75.7 | 14.4 / 62.4 / 86.8 | 0 |  |
| color+depth 30fps jpeg | color | 29.96 | 47 | 1.8 / 1.9 / 2 | 5.5 / 50.1 / 68.4 | 7.5 / 51.2 / 72.1 | 0 | 66.9 |
|  | depth | 29.96 | 68 | 14.2 / 14.7 / 15 | 18.3 / 60.7 / 81.5 | 20.2 / 66.3 / 82.7 | 0 |  |
| color+depth 30fps raw | color | 29.96 | 47 | 1.9 / 2 / 2 | 6.2 / 52.2 / 71 | 7.5 / 51.7 / 71.9 | 0 | 25.5 |
|  | depth | 29.96 | 500 | 6.2 / 6.3 / 6.4 | 14.9 / 64.1 / 84.1 | 16 / 63.3 / 83.9 | 0 |  |
| all four 30fps jpeg | color | 29.97 | 47 | 3.2 / 5.5 / 6.1 | 6.6 / 53.1 / 70.4 | 8.4 / 54.2 / 73 | 0 | 92.7 |
|  | depth | 29.96 | 68 | 15.3 / 16.3 / 16.7 | 19.2 / 68.5 / 84.1 | 21 / 73.8 / 86.9 | 0 |  |
|  | ir_left | 29.96 | 43 | 5.9 / 7.5 / 15.8 | 8.9 / 53.6 / 73.3 | 10.2 / 57.7 / 76.9 | 0 |  |
|  | ir_right | 29.96 | 43 | 6.2 / 6.8 / 7.6 | 10.3 / 54.4 / 74.6 | 11.2 / 59.4 / 77 | 0 |  |
| depth 480x270@60 jpeg | depth | 59.89 | 28 | 7.1 / 7.8 / 7.9 | 10.9 / 56 / 74.8 | 12.1 / 55.9 / 76.1 | 0 | 73.7 |
| depth 480x270@60 raw | depth | 59.9 | 253 | 4.3 / 4.6 / 4.7 | 10.8 / 61.2 / 76.4 | 12.1 / 60.2 / 78.6 | 0 | 25.8 |

### Where the time goes (medians, ms)

`sensor→capture` = `t_capture − t_sensor` (camera timestamp, mapped to host by the SDK, to SDK
receipt), `SDK wait` = receipt → `wait_for_frames` returned, `align/copy` = owner processing,
`encode` = publish → payload ready, `queue` = payload ready → handed to the socket,
`send→recv` = network + client read. From the localhost run except the last column.

| case | stream | sensor→capture | SDK wait | align/copy | encode | queue | send→recv localhost / LAN / Tailscale |
|---|---|---|---|---|---|---|---|
| color 720p10 | color | 28.5 | 0.2 | 0.2 | 0 | 0.2 | 0.8 / 6.4 / 7.4 |
| depth 848x480@10 jpeg | depth | 90.8 | 8.2 | 0.7 | 7.9 | 0.2 | 0.6 / 4.8 / 6.5 |
| depth 848x480@10 raw | depth | 90.1 | 8.4 | 0.7 | 0 | 0.3 | 2.9 / 15.1 / 17.3 |
| depth 848x480@10 raw+zlib | depth | 89.8 | 8.4 | 0.7 | 15.9 | 0.2 | 1.6 / 9.1 / 11.7 |
| color+depth defaults jpeg | color | 29.1 | 0.2 | 0.2 | 0 | 0.2 | 0.9 / 6.6 / 8.9 |
|  | depth | 90.4 | 8.4 | 0.7 | 7.6 | 0.2 | 0.6 / 4.9 / 6.4 |
| color+depth defaults raw | color | 28.7 | 0.3 | 0.3 | 0 | 0.3 | 0.9 / 4.5 / 7.2 |
|  | depth | 90 | 8.3 | 0.7 | 0 | 0.3 | 4.6 / 14.6 / 19.1 |
| color+depth defaults jpeg q50 | color | 28.8 | 0.2 | 0.2 | 0 | 0.2 | 0.9 / 6.6 / 7.8 |
|  | depth | 90.1 | 8.4 | 0.7 | 7.7 | 0.2 | 0.5 / 4.6 / 6.3 |
| color+depth defaults jpeg q95 | color | 29.5 | 0.2 | 0.3 | 0 | 0.2 | 0.9 / 6.8 / 7.5 |
|  | depth | 90.9 | 8.4 | 0.7 | 8.3 | 0.2 | 0.9 / 6.6 / 7.3 |
| color+depth defaults jpeg 2 clients | color | 28.9 | 0.3 | 0.2 | 0 | 0.4 | 0.9 / 7.5 / 8.4 |
|  | depth | 90.2 | 8.4 | 0.7 | 8 | 0.4 | 0.6 / 4.5 / 6.4 |
| color+depth defaults raw 2 clients | color | 28.5 | 0.3 | 0.3 | 0 | 0.5 | 1.1 / 6.9 / 7.3 |
|  | depth | 89.8 | 8.4 | 0.7 | 0 | 1.1 | 7 / 22.5 / 27.6 |
| color+depth defaults jpeg + IMU 200Hz | color | 29 | 0.2 | 0.2 | 0 | 0.2 | 0.9 / 6.5 / 7.3 |
|  | depth | 90.3 | 8.4 | 0.7 | 7.7 | 0.2 | 0.6 / 4.7 / 6.5 |
| color+depth defaults frame_sync jpeg | color | 28.5 | 70 | 0.8 | 0 | 0.3 | 1.2 / 6.5 / 7.7 |
|  | depth | 89.8 | 8.3 | 0.8 | 8.4 | 0.2 | 0.6 / 4.7 / 6.5 |
| color+depth aligned jpeg | color | 29 | 70.1 | 8.5 | 0 | 0.3 | 1.2 / 6.5 / 7.5 |
|  | depth | 90.3 | 8.4 | 8.5 | 17.3 | 0.2 | 0.7 / 5.6 / 6.6 |
| color+depth aligned raw | color | 28.6 | 70.1 | 8.5 | 0 | 0.3 | 1.4 / 5.1 / 6.4 |
|  | depth | 90 | 8.4 | 8.5 | 0 | 0.6 | 9 / 26.7 / 35.1 |
| all four defaults jpeg | color | 29.3 | 0.3 | 0.3 | 0 | 0.2 | 0.9 / 6.5 / 7.3 |
|  | depth | 90.6 | 9.1 | 0.6 | 7.2 | 0.2 | 0.6 / 4.8 / 6.8 |
|  | ir_left | 88.9 | 0.1 | 0.4 | 6 | 0.3 | 0.8 / 5.3 / 6.9 |
|  | ir_right | 88 | 0.1 | 0.7 | 3.6 | 0.3 | 1 / 5.6 / 7.6 |
| all four defaults raw | color | 28.8 | 0.3 | 0.3 | 0 | 0.3 | 1 / 5 / 6.8 |
|  | depth | 90.6 | 9.9 | 0.7 | 0 | 0.2 | 3.1 / 15.1 / 17.2 |
|  | ir_left | 88.5 | 0.1 | 0.8 | 0 | 0.8 | 4 / 12.5 / 14.7 |
|  | ir_right | 87.6 | 0.1 | 0.7 | 0 | 0.7 | 4.1 / 9.6 / 12.5 |
| color+depth 30fps jpeg | color | 32 | 0.5 | 0.2 | 0 | 0.3 | 0.8 / 4.6 / 6.5 |
|  | depth | 32 | 3.2 | 0.5 | 9.6 | 0.2 | 0.7 / 4.9 / 6.6 |
| color+depth 30fps raw | color | 31.6 | 0.5 | 0.3 | 0 | 0.3 | 0.9 / 5.3 / 6.5 |
|  | depth | 31.5 | 3.2 | 0.5 | 0 | 0.2 | 2.2 / 11.3 / 12 |
| all four 30fps jpeg | color | 32.7 | 1 | 0.1 | 0 | 0.4 | 1 / 5 / 6.6 |
|  | depth | 32.5 | 3.7 | 0.7 | 9.9 | 0.3 | 0.7 / 5.2 / 6.5 |
|  | ir_left | 31.8 | 1.2 | 0.5 | 3.1 | 0.6 | 0.6 / 3.8 / 5.1 |
|  | ir_right | 31.7 | 0.3 | 0.8 | 3.6 | 0.7 | 0.6 / 4.9 / 5.7 |
| depth 480x270@60 jpeg | depth | 14.8 | 2.2 | 0.4 | 3.8 | 0.2 | 0.5 / 4.3 / 5.5 |
| depth 480x270@60 raw | depth | 15.7 | 2.2 | 0.4 | 0 | 0.3 | 1.4 / 8 / 9.2 |

### Settings changes

`POST /settings` round trip (it returns once every enabled stream delivered a frame with the new
configuration) and POST start → first new frame received by a client, ms. Changes that do not touch
the pipeline (display options, mode only) answer in 3–50 ms.

| settings change | localhost POST / first frame | LAN | Tailscale |
|---|---|---|---|
| → color 720p10 | 926 / 946.2 | 1060 / 1082.3 | 1008 / 1039.9 |
| → depth 848x480@10 jpeg | 454 / 474.1 | 450 / 481.5 | 476 / 515.4 |
| → color+depth defaults jpeg | 699 / 701.7 | 710 / 730.4 | 716 / 739.6 |
| → color+depth defaults frame_sync jpeg | 1078 / 1080.4 | 1138 / 1161.8 | 1145 / 1173.4 |
| → color+depth aligned jpeg | 1104 / 1105.6 | 1012 / 1033.2 | 1088 / 1120.9 |
| → all four defaults jpeg | 1022 / 1024.5 | 991 / 1012 | 947 / 1041.2 |
| → color+depth 30fps jpeg | 1122 / 1124.3 | 1070 / 1084.9 | 1098 / 1117.7 |
| → all four 30fps jpeg | 792 / 794.5 | 764 / 779.9 | 760 / 777.5 |
| → depth 480x270@60 jpeg | 297 / 306 | 374 / 396.2 | 325 / 341.2 |

### IMU

Per-sample latency `t_recv_server − t_capture`; "newest in batch" is the freshest sample of each
record.

| IMU | location | recv Hz | dropped / gaps | latency med / p95 / p99 ms | newest-in-batch med | sensor→capture | Pi CPU % |
|---|---|---|---|---|---|---|---|
| imu 200Hz | localhost | 202.8 | 0 / 0 | 0.6 / 0.8 / 1 | 0.6 | 2.5 | 7.3 |
| imu 1kHz | localhost | 1014.2 | 0 / 0 | 10.8 / 11 / 11 | 0.8 | 4.2 | 11.6 |
| imu 200Hz | LAN | 202.8 | 0 / 0 | 2.3 / 47.1 / 67.5 | 2.3 | 2.4 | 8.2 |
| imu 1kHz | LAN | 1014.2 | 0 / 0 | 12.8 / 58.2 / 77.4 | 2.9 | 4.8 | 10.9 |
| imu 200Hz | Tailscale | 202.8 | 0 / 0 | 2.7 / 48.4 / 68.7 | 2.7 | 2.5 | 9.0 |
| imu 1kHz | Tailscale | 1014.2 | 0 / 0 | 12.8 / 58.3 / 78.4 | 2.9 | 4.1 | 11.9 |

### Interpretation

- **The camera dominates, not the node.** `t_capture − t_sensor` is ~29 ms for MJPG color at any
  rate, but ~90 ms for depth and IR at 10 fps, ~32 ms at 30 fps and ~15 ms at 60 fps: the depth/IR
  device timestamp sits about **one frame period** before the frame reaches the Pi (exposure +
  on-camera stereo processing + paced USB 2 transfer). For control loops, run depth at the highest
  fps you can afford; 480x270@60 halves-to-quarters that delay. True photon-to-client latency is
  roughly `sensor→capture` + the table above (e.g. color ≈ 31 ms, depth 848x480@10 ≈ 105 ms on
  localhost).
- **Frame sync / alignment cost ~70 ms of color latency.** With frame sync (needed for D2C
  alignment) the SDK holds each color frame until its depth arrives. Leave `frame_sync` off unless
  you need paired frames; use `t_sensor` to pair them on the client instead (same camera clock).
  Alignment also costs ~9 ms of CPU per frame in the owner and doubles the raw depth size
  (aligned depth has the color resolution: 1.8 MB/frame).
- **Inside the node it is single-digit ms.** The SDK keeps depth ~8 ms after receipt at 10 fps
  (~3 ms at 30 fps; depth post-processing in the SDK), monitoring encode is ~8 ms for depth
  848x480 (OpenCV colormap + JPEG) and ~3–6 ms for IR, color is a zero-cost passthrough, raw is
  zero-copy, and the send queue is < 1 ms. On localhost color arrives **1.5 ms** after SDK receipt
  and raw depth 12–14 ms (of which 8 ms is the SDK).
- **JPEG quality barely matters** for latency (q50 vs q95: <1 ms, 34 vs 122 KB/frame); it matters
  for bandwidth. **2 clients** add ~0.4 ms (encode is shared); raw depth to two clients adds ~4 ms
  locally and ~8 ms over Wi-Fi.
- **raw + zlib is not worth it on this link**: zlib costs ~16 ms of Pi CPU per depth frame and
  saved less on Wi-Fi than it cost (34 vs 24 ms median). It may pay off on a much slower link.
- **Network (Mac on Wi-Fi):** medians add ~4–7 ms for JPEG-sized frames and ~10–20 ms for raw
  depth (65–150 Mbit/s all went through without drops). The p95/p99 (~50–60 / ~70–90 ms) are the
  same for a 120-byte IMU record as for an 800 KB depth frame, so they are Wi-Fi latency spikes
  (power save / scanning on the Mac), not bandwidth. Tailscale (direct, WireGuard) adds ~1–3 ms to
  the LAN numbers. A wired client should get close to the localhost column plus ~1 ms.
- **Pi CPU** (node process, % of one core of 4): 2.5 % for color passthrough, ~13–15 % raw
  color+depth, ~25 % with depth JPEG at 10 fps, ~67 % color+depth JPEG at 30 fps, ~93 % for all
  four streams JPEG at 30 fps, ~74 % depth JPEG at 60 fps. Raw delivery is cheaper on the Pi than
  monitoring JPEG. IMU adds ~6 % at 200 Hz.
- **IMU:** every sample arrives (no drops, no gaps), 0.6 ms after SDK receipt on localhost at
  200 Hz. At 1 kHz the SDK delivers samples in ~10 ms bursts (newest sample 0.8 ms, oldest ~11 ms),
  so per-sample latency is ~11 ms by construction. `t_capture − t_sensor` is 2.5 ms (200 Hz) to
  4–5 ms (1 kHz). Rates measured on the Pi clock are 1.4 % above nominal (202.8 / 1014 Hz).
- **Settings:** a pipeline restart takes 0.3–1.1 s until the first new frame (depth only
  ~0.45 s, color ~0.9–1 s, frame sync ~1.1 s).

Skipped / not measured: uncompressed color profiles in the matrix (only checked by hand:
640x480@30 RGB works at 30 fps), 1280x800 depth, and a wired Mac (the Mac was on Wi-Fi for both
"LAN" and Tailscale).

## SDK and FastAPI notes (found while building this)

pyorbbecsdk2 2.1.3 / libOrbbecSDK 2.10.6, Gemini 336L fw 1.4.60 on a Pi 5 USB 2.1 port:

- **Frame aggregation decides latency.** With frame sync on (the SDK default) color is held
  ~69 ms until its depth frame arrives. With frame sync off but any aggregation mode other than
  `DISABLE` (`ANY_SITUATION`, `COLOR_FRAME_REQUIRE`, `FULL_FRAME_REQUIRE`), depth is held ~140 ms
  for the *next* color frame and paired with it (wrong pairing, one frame late). The node uses
  `DISABLE` (one frame per frameset) unless frame sync or alignment is requested.
- **IMU needs its own pipeline.** In the video pipeline, accel/gyro samples ride along with the
  video framesets: about half of them arrive, ~125 ms late. A second `Pipeline(dev)` with only
  accel+gyro works while video streams (a second pipeline for video does not) and delivers every
  sample ~0.1 ms after SDK receipt. Sensor callbacks (`dev.get_sensor(...).start(profile, cb)`)
  also work, for IMU and video, but run on SDK threads.
- **Global timestamps.** `device.is_global_timestamp_supported()` is true; after
  `enable_global_timestamp(True)`, `frame.get_global_timestamp_us()` is the device timestamp in
  host epoch µs. The device clock (`get_timestamp_us`) is shared by video and IMU.
- **Units.** Accelerometer in m/s² (≈9.8 at rest), gyroscope in rad/s (the SDK converts; the
  library carries the ROS note "rotational velocity should be in rad/sec"). IMU options on this
  unit: 50 Hz–1 kHz, ±4 g, ±1000 °/s only.
- **Uncompressed color on USB 2** is not uniformly broken: `640x480@30 RGB` with depth
  `848x480@10` delivers 30 fps; heavier combinations (RGB color + 1280x800 depth + both IR +
  alignment) start without an error but deliver nothing, which the node detects (no frames within
  8 s) and reverts.
- Settings changes take 0.4–1 s (pipeline stop/start, first frames) and up to 8 s plus a restart
  for a silent configuration.
- FastAPI/uvicorn: streaming responses never end, so `timeout_graceful_shutdown=2` keeps Ctrl-C
  quick; encode futures shared between clients must be `asyncio.shield`ed or one client
  disconnecting cancels everyone's encode; `/time` is an `async def` so it is answered on the
  event loop, not in the thread pool (less jitter).

## Dependencies

Server: `fastapi` and `uvicorn` (plus the camera stack already in the robot env: `pyorbbecsdk2`,
`numpy`, `opencv-python-headless` or Pillow). On the Pi they were installed with
`pip install -c constraints.txt fastapi uvicorn` (fastapi 0.162, uvicorn 0.54, starlette 1.10,
pydantic 2.14; prebuilt wheels, no conflicts). Client: `httpx`, `numpy` (+ OpenCV or Pillow to
decode JPEG). Tests: `pytest` (the end-to-end test also needs fastapi + uvicorn).
