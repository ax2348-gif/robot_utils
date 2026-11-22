# Episode browser (`robot_utils.collect browse`)

Review recorded teleop sessions and episodes (docs/collect.md) in a local web page: the robot in 3D with the WBC's EE
references, the operator's headset and controllers with the clutch anchors, the camera videos, signal plots and a
timeline; replay at any rate, scrub and step; export the replay as an MP4 with a size estimate or a target size. It
runs on the Mac or erid, next to the data, never on the robot.

```bash
python -m robot_utils.collect browse ~/data                       # every session under ~/data
python -m robot_utils.collect browse ~/data/20261122-143000       # one session (its episodes, cut episodes)
python -m robot_utils.collect browse ~/data/20261122-143000/episodes/20261122-0003   # one cut episode
# on erid, viewed from the Mac: --host 0.0.0.0, then http://erid:8770/ (or ssh -L 8770:localhost:8770 erid)
python -m robot_utils.collect browse ~/data --host 0.0.0.0
# headless: one camera of one episode to an MP4 of 20 MB, twice as fast as real time
python -m robot_utils.collect browse ~/data/20261122-143000 --export-camera camera.color \
    --episode 20261122-0003 --rate 2 --target-mb 20
```

| option | default | |
|---|---|---|
| `path` | | a session (`session.json`), a cut episode (`episode.json`) or a directory holding sessions |
| `--host` / `--port` | `127.0.0.1` / `8770` | where the page is served |
| `--description` | (below) | robot description: JSON file, description node URL, or description data dir |
| `--ffmpeg` | (below) | the ffmpeg binary for export |
| `--export-dir` | `<path>/exports` | where videos are written |
| `--cache` | `~/.cache/robot_utils/browse` | the index cache (`$ROBOT_UTILS_BROWSE_CACHE` too) |
| `--open` | off | open the page in the default browser |
| `--export-camera STREAM` | | headless export of one camera (with `--episode`, `--out`, `--rate`, `--fps`, `--size`, `--preset`, `--crf`, `--target-mb`), then exit |

| file | what |
|---|---|
| `collect/browse/index.py` | `StreamIndex`: the time index of one stream's files, cached memory-mapped; lazy column reads |
| `collect/browse/oplog.py` | the operator stream as tables (`oplog.headset`, `oplog.sent`), the clutch anchors |
| `collect/browse/dataset.py` | `Recording` (a session or an episode), `Library` (what a path holds) |
| `collect/browse/model.py` | the description and the arm's kinematic chain for the 3D view |
| `collect/browse/export.py` | ffmpeg discovery, presets, size estimate / target, the encode jobs |
| `collect/browse/server.py` / `page.py` | the HTTP API and the page |
| `collect/browse/synth.py` | a synthetic session in the recorder's exact layout (tests, performance) |

## The page

- **Library** (left; ☰ hides it): sessions with their episodes (kept / aborted, duration) and any cut episode
  directories. Choosing an episode in a session shows its window plus 1 s each side; time reads relative to the
  episode's start. The URL hash keeps the selection.
- **Robot · world** (large): the robot at the WBC's base pose (`base_x/y/yaw`, the pose the WBC used; the
  estimator's when there is no WBC stream), wheels at their steering angles, the arm from the **measured** joints
  (`j1..6_q`, gripper `g_gap`; green while armed) and the WBC's **commanded** joints (`<ee>_q_cmd1..6`) as a faint
  ghost. The **EE references** (world): target `<ee>_tgt_*` (blue, ringed), command `<ee>_cmd_*` (orange),
  measured `<ee>_meas_*` (ink), each a small triad with its last 2 s of path. The base path over the whole
  recording (faint) and the last 5 s. While engaged (body-frozen): the **base hint** target from the `sent`
  datagrams (dashed footprint), the **frozen platform frame** and the **EE anchor** (dashed). Corner text: armed /
  WBC state / limit, `cmd-tgt` and `meas-tgt` errors. Drag to orbit, shift-drag to pan, wheel to zoom, double-click
  to reset; `follow` keeps the robot centred; `3D` / `top` / `side`.
- **Cameras**: every JPEG stream (`camera.color`, `wrist_camera.color`), the frame shown is the newest at or before the
  playhead (`t_sensor`); the label gives the fps and the stream's dropped count.
- **Operator · headset frame**: the headset LOCAL frame in operator FLU axes (`frames.C`; origin = where tracking
  started, the grid ~1.55 m below the first head pose): the headset, both controllers (blue and thicker while that
  hand's clutch is engaged), the head's last 4 s on the floor; while engaged, the **operator frame** captured at the
  engagement (dashed axes at the head, X = its gravity-aligned yaw), the **hand anchor** (dashed controller) and the
  delta from it to the hand (dashed line). Corner text: grip, trigger, engagement id, buttons; what was sent (armed,
  base mode).
- **Transport**: play / pause (space), step one camera frame (← →; shift: 1 s), rate 0.05× to 16× (`[` `]`), loop,
  in / out marks (I, O) for export.
- **Timeline**: the shown range; episodes as brackets (green kept, amber aborted), armed as a band, events as ticks
  (taller for episode events, red for e-stop / fault / abort; hover for details). Click or drag to scrub.
- **Plots**: stacked, sharing the time axis, centred on the playhead (wheel: window 1 s to 10 min; drag: scrub);
  the values at the playhead beside it; armed as a band at the bottom. Defaults: EE errors, EE z (target / command /
  measured), the base twist, the joints, the gripper, the operator's grip / trigger / engaged. "add a plot": any
  stream, any fields (comma-separated; the list filters as you type). The plot set is remembered (localStorage).
- **View**: show / hide each panel and the commanded-arm ghost.

## Data access (performance)

- **Index by time.** Opening a recording reads each stream file once (headers; for rows records also the time
  column) into arrays per record: file, header and payload offsets, type, Pi time span, row count, schema. They are
  saved as `.npy` in the cache, keyed by the files' paths, sizes and mtimes, and opened memory-mapped afterwards: a
  reopened recording is ready at once and never sits in memory. Records are kept in Pi-time order; a window lookup is
  a binary search on monotone envelopes of the records' start and end times.
- **Rows on demand.** A window `[t0, t1]` of chosen columns is gathered straight from the memory-mapped files
  (vectorised per schema and file); the page asks for 4 s chunks of the columns its views need, keeps ~90 per
  stream, and prefetches ahead of the playhead (further at higher rates).
- **Frames on demand.** The page fetches the frame list of a 4 s chunk, then single JPEGs by record index (served as
  stored, cacheable forever); the browser decodes them lazily (`createImageBitmap`), keeps ~150 per camera and
  prefetches the next ~0.5 s of frames (scaled by the rate). A frame not decoded yet leaves the previous one up.
- **The operator stream** is JSON, so its records are parsed once into the two tables (cached like the index).

## Clutch anchors (body-frozen)

The Pi's client (tools/quest_teleop/body_frozen.py) captures, on the frame an engagement starts: the operator frame
(origin = head position, X = its gravity-aligned yaw), the controller pose in it (hand anchor), the platform frame
(the WBC row's base x, y, yaw) and the commanded EE pose (EE anchor). None of this is recorded as such. The browser
reconstructs it (`oplog.reconstruct_anchors`):

- **Operator side, exact:** the capture frame is the first headset frame of the engagement (the native intent's
  `engagement_id` changes with `engaged` true); its head and hand poses give the operator frame and the hand anchor
  with the client's own formulas (tested against `frames.OperatorFrame`).
- **Robot side:** the first `sent` datagram with `base.mode = "hint"` after it: its hint pose is the platform frame
  (head motion and stick offset are still zero) and its EE target is the EE anchor (`target_raw` = the command at
  capture). Exact when that client step handled one frame (the usual case at 72 Hz); when a step drained several
  frames, the datagram already carries the later frames' hand motion (about a frame of motion off).
- An engagement the client refused (no fresh WBC row, head not tracked, release needed) sends no hint: it is kept
  with `accepted: false` and not drawn.

**What is missing** for an exact, assumption-free record: which headset frame each datagram came from, and the
capture itself. **Smallest addition (the owner's call; robot side, deployed separately):** one event line per
capture, in the existing contract, so it also lands in the Pi's event log:

```json
{"kind": "engaged", "detail": {"hand": "right", "engagement_id": 7, "frame_seq": 81234,
  "operator": {"origin": [x, y, z], "yaw": 0.12}, "hand_anchor": [x, y, z, qx, qy, qz, qw],
  "robot_frame": [x, y, yaw], "ee_anchor": [x, y, z, qx, qy, qz, qw], "ee_source": "cmd", "est_reset": 3}}
```

(every value is a field of `body_frozen.Engagement` at capture), plus `{"kind": "engagement_refused", "detail":
{"hand", "engagement_id", "reason"}}`, and optionally `frame_seq` (the newest frame of the step) on each `sent`
record. The browser would then read the anchors from the events and keep the reconstruction for older recordings.

## Model

`session.json` records neither the robot description nor a model version, so the 3D view takes the description
from, in order: `--description` (a JSON file of the description node's `GET /description`, a description node URL,
or a description data dir such as the sim's `~/.local/share/robot_utils/sim/description`); `<session>/description.json`;
the package default (nominal camera mounts and arm mount: labelled "package default" in the header). The arm's
chain (joint placements and axes, `end_link`, the TCP) comes from the arm node's pinocchio model
(`arm.kin.ArmKinematics`, rebot_ctl's URDF) and the page runs the forward kinematics (tested equal to
`ArmKinematics.frames`). Without rebot_ctl the arm is drawn as its EE poses only.

**Decision for the owner:** have `collect run` save, once per session at start, `GET /nodes/description/description`
as `<session>/description.json` (the browser already reads it) and `GET /nodes/description/model`'s `version` in
`session.json` (so a model change between sessions is visible). Client-side only; about ten lines in `session.py`.

## Export

**Source**: "replay" renders the page's own views frame by frame at the export clock (robot 3D left; cameras and
the operator view right; up to three plots below; a header with the episode, the time and the rate) into a canvas
of the chosen resolution and posts each frame as a JPEG; "`<camera>` only" takes that camera's recorded JPEGs as
they are (no rendering; much faster; also `--export-camera`). Range: the episode / shown range, in–out, or the whole
recording. Rate: video time = data time / rate (0.5× = slow motion), at the chosen fps; each output frame shows the
data at its own time.

**Presets** (codec, x264/x265 speed, CRF): draft (H.264, VideoToolbox when present else ultrafast, CRF 28), balanced
(H.264 veryfast 23), quality (H.264 slow 18), small (HEVC medium 28, `hvc1`; plays on Apple and modern players).
Every MP4 is `yuv420p` with `+faststart`.

**Size.** "by quality": a CRF; the estimate shown is a bits-per-pixel model (per codec, halving every +6 CRF),
and **measure** encodes a real sample (three 1 s pieces spread over the range, rendered or camera frames, same
settings) to replace the model by the measured bits per pixel. "target size": `bitrate = 8 · size · (1 − 1.5 %
container) / video duration`, encoded **two-pass** (exact to a few percent) or one-pass ABR (faster, looser); a
warning shows when the target leaves under 0.01 bit per pixel (blocks: lower the resolution or fps instead).

**ffmpeg** (a binary, not a Python package): `--ffmpeg`, `$FFMPEG`, `ffmpeg` on PATH, else the binary bundled by the
`imageio-ffmpeg` pip package in the running environment or any conda environment on the machine. Neither the Mac
nor erid has `ffmpeg` on PATH today; both have an `imageio-ffmpeg` binary (Mac: miniconda base, ffmpeg 9.0 with
libx264, libx265 and VideoToolbox; erid: several envs, e.g. `lightnav`). Without one, export is off and the rest
works. **Decision for the owner:** add `imageio-ffmpeg` to the robot_utils environment (a pip wheel with a static
ffmpeg; nothing system-wide), or install ffmpeg system-wide (brew / apt).

## Performance

Measured on the Mac (2026-11-22, heavily loaded by other jobs) with `browse/synth.py`: a 10 min session, 2 cameras ×
640×480 JPEG at 30 fps (92 KB a frame), arm / swerve / estimator / WBC rows at 100 Hz, the operator stream at
72 Hz, files rotated at 512 MB: 3.6 GB in 13 files, 28 episodes. (The generator reuses 60 distinct JPEGs, so frame
reads hit a warm page cache.)

| what | time |
|---|---|
| first open: index build of every stream + operator tables | 5.0 s (peak RSS 332 MB; cache 46 MB on disk) |
| reopen from the cache + the recording summary | 20 ms |
| WBC rows, a 4 s window × 60 columns (400 rows) | 0.4 ms median, 0.5 ms p95 |
| operator headset table, 4 s, all columns | 0.02 ms |
| one column over the whole session, stride 20 | 7 ms |
| camera frame lookup by time + read | 0.01 ms median |
| page: a 4 s WBC chunk over HTTP (fetch + parse) | 1.2 ms |
| page: frame fetch + JPEG decode, one at a time / 8 in flight | 2.7 ms / 0.56 ms a frame (≈ 1800 frames/s; 2 cameras at 16× need 960) |
| export: compose one 1280×720 replay frame / JPEG it | 2 ms / 10 ms; ≈ 20 frames/s rendered and posted |
| export: encode 20 s of 720p30 camera, balanced / two-pass to 4 MB / draft at 2× (VideoToolbox) | 14 s / 12 s (3.95 MB, −1.3 %) / 1.9 s |
| export: 6 s at 0.5× of the replay, 720p30, two-pass to 3 MB | 18 s render + 5 s encode; 2.97 MB (−1.0 %) |

Memory stays flat with session length: the index and the operator tables are memory-mapped, rows and frames are
read per window, and the page keeps a bounded number of chunks and decoded frames.

## Known limits

- The 3D views are orthographic wireframes on a 2D canvas (no WebGL, no meshes): light and dependency-free, but
  the arm is boxes and the robot has no textures.
- The export renders in the page: keep the tab open while it runs (it does not need to be in front).
- The WBC's pose fields are in the estimator's world; after an estimator reset (`est_reset` changes) the world
  jumps and so do the trails.
- Depth or raw camera streams are listed but not shown (JPEG only).
- One EE (the first `<ee>_tgt_x` in the WBC stream) is drawn.
