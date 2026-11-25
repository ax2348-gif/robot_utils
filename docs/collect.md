# Data collection script (`robot_utils.collect`)

The client side of teleop data collection (docs/plans/data_collection_v1.md, "Architecture"): it runs on
erid (real collection) or the Mac (the sim), never on the Pi. Teleop itself runs in the Pi's `quest_teleop`
node; this script sets the task, optionally starts / stops teleop, records every stream continuously as raw
bytes, and indexes the episodes from the operator stream's events. Episodes are cut out of the continuous
recording afterwards, by Pi time.

```bash
python -m robot_utils.collect run --robot http://botcore --out ~/data --task "pick the cup" --start-teleop
#   ... the operator collects episodes with A (start) / B tap (stop); Ctrl-C ends the session ...
python -m robot_utils.collect cut ~/data/20261121-143000 --pre-roll 0.5 --post-roll 0.5
```

| file | what |
|---|---|
| `collect/session.py` | `Session`: task, teleop start / stop, the recorders, the reconcile loop, the status line, shutdown |
| `collect/streams.py` | stream specs, `StreamRecorder` (raw bytes, rotation, reconnect, gap sidecar) |
| `collect/episodes.py` | `EpisodeIndex`: events in seq order, holes, episodes |
| `collect/records.py` | `RawSplitter` (exact record bytes), the Pi time of a record |
| `collect/cut.py` | `cut_session` |
| `collect/scenes.py` | `SceneSaver`: each rigid scene a pose stream names, saved once per `scene_id` |
| `collect/local.py` | `--robot local`: the nodes of this machine over their unix sockets (`NodeSocketTransport`) |
| `nodes/quest_teleop/recording.py` | `SimRecorder`: the sim's quest_teleop node runs `collect run` from Start to Stop |
| `replay/` | `python -m robot_utils.replay`: the operator's view of a recording as an mp4 (below) |
| `tests/collect_fakes.py` | a fake robot serving the contract (rows, camera, operator stream, task / start / stop / eventlog) |

## `run`

| option | default | |
|---|---|---|
| `--robot` | (required) | the frontend, e.g. `http://botcore` (every path is `<robot>/nodes/...`); `local`: the nodes of this machine over their unix sockets, no frontend (session.json says `http://localhost`) |
| `--out` | (required) | parent directory; the session is `<out>/<YYYYMMDD-HHMMSS>` (`--name` overrides) |
| `--task` | (required) | `POST /nodes/quest_teleop/task {"text"}` before anything else: the task of the next episodes |
| `--streams` | `camera/color,wrist_camera/color,camera/imu/stream,arm/stream,swerve/stream,estimator/stream,wbc/stream,quest_teleop/oplog/stream` | what to record (`camera/imu/stream`: the chassis camera's ~100 Hz IMU records, attitude and mean acceleration; cheap) |
| `--batch-ms` | `50` | the row streams (`<node>/stream`: arm, swerve, estimator, wbc) in batches this long: the same rows and timestamps, far fewer records, packets and wakeups on the robot (measured 2026-11-22: the Pi near its CPU limit with the full stack plus a recorder). `0` = every row on its own, for a consumer that acts on them live (a policy run) |
| `--start-teleop` | off | `POST /nodes/quest_teleop/start` once recording runs (a 409 ends the session with the WBC's reason) |
| `--stop-teleop` | off | `POST /nodes/quest_teleop/stop` on Ctrl-C (the node aborts an open episode itself) |
| `--rotate` | `512M` | start the stream's next file past this size |
| `--read-timeout` | 10 s | no bytes for this long (every stream pings after 1 s idle) = a dead connection: reconnect |
| `--settle` | 1 s | keep recording this long after Ctrl-C, so the last events and frames arrive |
| `--status-every` | 5 s | status line period |
| `--scene3d [SIM]` | off | also record the sim's rigid scene: `<SIM>/scene3d/stream` (default SIM `sim`) and each scene once per `scene_id` (below) |
| `--no-set-task` | off | leave the node's task as it is (`--task` is then only the session's label) |
| `--baseline-seq N` | the log's newest | index the events after seq N |
| `--status-json` | off | status lines on stdout as `{"status": ...}` (`Session.stats()`), for a supervising process |

**Stream specs** (relative to `<robot>/nodes/`): `<node>/.../stream` is that endpoint (`arm/stream`,
`quest_teleop/oplog/stream`); `<camera node>/<camera stream>` is the Orbbec node's
`/stream?streams=<stream>&mode=jpeg&every=true` (every frame in order from a bounded per-client queue; `camera/color`: colour as the camera's MJPG bytes, untouched; no
re-encoding anywhere); a spec with `?` is used verbatim (`camera/stream?streams=depth&mode=raw`); `<sim>/scene3d/stream` is the rigid
scene's pose stream (`format=bin`, below).

**Startup:** the task is set (failure = exit 1), the Pi event log's newest seq is read
(`GET /eventlog/files`, the max `last_seq`): events after it are this session's. If the node's `/state` shows
an open `episode` (`{id, task, t_start, start_seq}`), the index starts at its `start_seq` instead and its events
are fetched from the Pi log, so it is indexed from its start (if the log no longer has the start, it is opened
from `/state`'s fields). Then every stream is opened, and teleop is started if asked.

**Status** (stderr, every `--status-every` s, and at the end):

```
[collect 00:12:31] | in episode 20261121-0007 8.3 s | 6 kept 1 aborted | camera.color 10/s, wrist_camera.color 10/s, arm 100/s, swerve 100/s, estimator 100/s, wbc 100/s, quest_teleop.oplog 144/s gaps 1 | 2301 MB
```

`recording` instead of `in episode` between episodes; rates are rows/s for rows streams, records/s otherwise;
a stream name prefixed `!` is disconnected; `gaps` / `dropped` appear once non-zero. Each finished episode
prints a line too.

**Ctrl-C** (SIGINT or SIGTERM; a second one kills): teleop stop (with `--stop-teleop`), `--settle` s of
recording, a last reconcile against the Pi log, then every file is flushed, fsynced and closed (whole
records only), an episode still open is closed `aborted` / `session ended` (its `t_stop` = the newest Pi time
seen on the operator stream), and `session.json` gets the stop time, reason and per-stream totals.

## Session layout

```
<out>/<YYYYMMDD-HHMMSS>/
  session.json                    robot, task, streams (spec, name, url, params), started / stopped wall,
                                  stop_reason, eventlog_baseline_seq, open_episode_at_start, files per stream, totals, skipped seqs
  episodes.jsonl                  one line per finished episode (below), appended + fsynced as it finishes
  events.jsonl                    every event of the session in seq order, with "via": stream | eventlog;
                                  {"missing": [seqs], "why"} where the Pi log did not have them either
  <name>.000.bin, .001.bin ...    each stream's raw record bytes, rotated at record boundaries
  <name>.gaps.jsonl               one line per connection gap (only if there was one)
  scene3d/<scene_id>.json, .blob  each rigid scene the pose stream named (with --scene3d), scenes.jsonl
  episodes/<id>/                  after `cut` (below)
```

`<name>` is the spec with a trailing `/stream` dropped and `/` -> `.`: `camera.color`, `wrist_camera.color`,
`camera.imu`, `arm`, `swerve`, `estimator`, `wbc`, `quest_teleop.oplog`, `sim.scene3d`.

**Stream files** are what the node sent, byte for byte (record framing of docs/nodes.md "Streams"), so
`common.protocol.iter_records` / `iter_rows` read them, concatenated or one by one. Two additions, both whole
records the node itself sent: every rotated file after the first starts with the stream's schema records (a
rows file decodes on its own), and after a reconnect the node's new schema record follows in the stream as
usual. A partial record at a dropped connection is never written (its size is in the gap line).

**Gap line** (`<name>.gaps.jsonl`): `wall`, `mono` (this machine's `time.monotonic()`) and `error` of the
failure; `pi_t_last` (the Pi time of the last record before it); `partial_bytes` discarded; `reconnect`:
`{wall, mono, attempts, pi_t_first}` (`pi_t_first`: the first record's Pi time after it), null if the
session ended first; `retry_errors` (the failed attempts). The gap spans Pi time (`pi_t_last`, `pi_t_first`).

**Episode line** (`episodes.jsonl`): `id`, `task` (from the `episode_start` event: what the node used),
`t_start`, `t_stop` (Pi monotonic: the `episode_start` and `episode_stop` / `episode_abort` events' `t`),
`wall` `{start, stop}`, `outcome` `kept` | `aborted`, `reason` (the abort's `detail.reason`; `session ended`;
null when kept), `seq_start`, `seq_stop` (null when closed by the session), `events` (the seqs of every event
while it was open, its start and stop included), `reconciled` (some of its events came from the Pi log or `/state`).

## Episodes from events; reconciliation

The operator stream's `event` records (the contract's event line) are applied strictly in `seq` order:
`episode_start` opens an episode, `episode_stop` closes it `kept`, `episode_abort` closes it `aborted` with
its reason. The Pi log is authoritative:

- **A seq gap on the stream** (an event arrives ahead of a missing seq): the later events are held, and the
  script fetches `GET /nodes/quest_teleop/eventlog?since_seq=<newest applied>&limit=1000`, paging on from each
  page's last seq until a page comes back short; what it returns is applied in order, duplicates ignored.
- **A reconnect of the operator stream**: the same fetch, always (the events during the outage never come on
  the stream).
- A seq the log does not have either (pruned) is skipped and noted (`events.jsonl`, `session.json`
  `skipped_event_seqs`); a fetch that fails is retried with backoff and once more at shutdown.
- Episode times are the events' own Pi times, so a late or recovered event cuts exactly where the Pi says.
- A while an episode is open: the node emits `episode_stop` (kept) for it, then `episode_start` for the next.
  An `episode_start` with no stop before it (not expected) closes the open one `aborted` / `no stop event before
  the next episode_start`.
- `event` records carry the event line flat in the header (the contract); a nested `event` object is accepted.

## `cut`

`python -m robot_utils.collect cut SESSION [--out DIR] [--pre-roll S] [--post-roll S] [--episodes ID,...]`
(default out `SESSION/episodes`, rolls 0). For each episode, every stream's records whose **Pi time** falls
in `[t_start - pre_roll, t_stop + post_roll]` go to `<out>/<id>/<name>.bin`, in order; a rows record that
straddles an edge is cut to its rows inside (header kept, `n` updated); every file starts with the schema
records its rows need; pings are left out. `<out>/<id>/episode.json`: the episode line's fields plus
`pre_roll`, `post_roll`, `window`, `event_seqs`, `session`, `robot`, `streams.<name>` `{spec, file, records,
rows, by_type, dropped (the sum of the records' own dropped counts), time_fields, first_t, last_t}` and
`gaps.<name>` (the gap lines whose Pi span intersects the window), and `scenes` (`[{scene_id, saved}]`, in order of
appearance): the rigid scenes its `scene3d` records name, copied (hard-linked) to `<out>/<id>/scene3d/` with their
`scenes.jsonl` lines, so an episode directory replays on its own.

### The time field per stream (all Pi `time.monotonic()` s)

| stream | record | field |
|---|---|---|
| `arm`, `swerve`, `wbc` | `rows` | `t_tick` (the control tick's start; per row) |
| `estimator` | `rows` | `t` (the sample time, docs/estimator_node.md) |
| other rows streams | `rows` / `imu` | the first of `t_tick`, `t`, `t_sensor`, `t_end`, `t_capture` the schema has (NaN falls back to the next) |
| `camera.color`, `wrist_camera.color` | `frame` | `t_sensor` (camera timestamp mapped by the SDK ≈ capture instant, docs/orbbec_node.md); `t_capture` (SDK receipt) when `t_sensor` is null |
| `quest_teleop.oplog` | `headset` | `t_rx` (the Pi's arrival stamp) |
| | `sent` | `t_send` |
| | `event` | `t` |
| `sim.scene3d` | `scene3d` | `t_rx` (the recorder's receipt; below) |

Reviewing sessions and episodes (replay, plots, 3D, video export): `python -m robot_utils.collect browse`, docs/browse.md.

Training conversion (e.g. LeRobot), the arm's `t_meas` latency compensation and interpolation onto a master
clock are offline steps on these files, not part of the cut.

## The robot description

Every session saves the robot as it was when recording started, unconditionally: `description.json` (the
description node's `GET /description`: frames, cameras with their active calibrations, conventions), `model.json`
(`GET /model`: `model_version`, components, joints) and `model.urdf` (the assembled URDF), with
`session.json` `description` (`model_version`, the node URL). The recorder refuses to start without them: a
recording without the robot it was made on can't be replayed with the right mounts.

## The rigid scene (`--scene3d`)

What a replay of the sim needs beyond the robot's own streams (docs/scene3d.md):

- **The pose stream** `<sim>/scene3d/stream?format=bin`: RSF1 frames back to back (no record framing). The
  recorder wraps each frame in one `scene3d` record whose payload is the frame's exact bytes and whose header carries
  `t_rx` (this machine's `time.monotonic()` at receipt), `seq`, `t_sim` (the frame's own `t`: sim time, which stops on
  a pause and restarts on a reset, so not a time base), `reset_seq`, `scene_id`, `n`, `m`
  (`records.Rsf1Splitter`). So the file is read, rotated, gap-tracked and cut like every other stream. `t_rx` is the
  robot clock only when the recorder runs on the sim's machine: the quest_teleop node's recorder always does;
  `collect run --scene3d` against a sim on another machine would cut it by that machine's clock.
- **The scenes**: at each `scene_id` not saved yet (the first frame; an object added or removed; a ghost declared;
  a fixed object moved) `GET <sim>/scene3d` and `/scene3d/blob?scene_id=` with the stream's `cell` / `crease_deg`,
  saved as `scene3d/<scene_id>.json` and `.blob`, and a line in `scene3d/scenes.jsonl`: `scene_id`, `t_rx`, `wall`,
  `view` (`GET /scene3d/view`: the headset's style and geometry role then), `files`, `bytes`. The sim serves only its
  current scene: one it has already moved past is gone, noted `{scene_id, missed: why}` (the one it served instead
  is saved under its own id). `session.json` `scenes`: saved, missed, pending; the status line counts them.
- **The operator stream** (`quest_teleop/oplog/stream`, a default stream) carries every headset record as received:
  the head (OpenXR VIEW located in LOCAL) and both controllers (grip spaces in LOCAL), and, from a headset with the
  logging AR view (quest_env `ROBOT_AR.md`, "Logged placement"), the `robot_ar` block: where the sim world was drawn
  (`sim_in_local`, `scale`, the floor source, each re-anchoring) and both eyes (pose relative to the head, field of
  view). Nothing in robot_utils filters it: the node passes the frame through as the headset sent it.

## Sim teleop records itself

In the sim stack, Start on the teleop page records: the `quest_teleop` node (with `--sim`) runs this script as a
child process for each teleop run (`nodes/quest_teleop/recording.py`) and ends it at Stop, whatever stopped teleop.
Nothing to remember and nothing else to run:

```
python -m robot_utils.collect run --robot local --out ~/data/sim --name <YYYYMMDD-HHMMSS> --task <the node's task>
    --no-set-task --teleop quest_teleop --scene3d sim --baseline-seq <the seq before teleop_start>
    --batch-ms 50 --settle 1 --status-every 1 --status-json
```

| node option | default | |
|---|---|---|
| `--record DIR` | `~/data/sim` | sessions go to `DIR/<YYYYMMDD-HHMMSS>` (`-2`, `-3` ... if taken) |
| `--no-record` | off | do not record |
| `--record-streams` | collect's defaults | the stream specs (the rigid scene is added) |
| `--record-robot` | `local` | where the child reaches the nodes: their unix sockets, or a frontend URL |

Design:

- **The same code.** It is `collect run`: the Session, its recorders, the index, the reconcile, the shutdown, the
  layout; a sim session cuts and converts like any other.
- **A child process, not a thread in the node.** The recorder parses every camera frame, row batch and headset
  record; in the node's process that work would share the GIL with the teleop loop, whose input gate disarms on
  150 ms of lateness. A crash, a full disk or a stuck write in the recorder can then never reach teleop either.
  Start spawns it and returns (a spawn failure is an event and `recording.error`, never a refused Start); Stop sends
  SIGTERM after the `teleop_stop` event (so it is recorded) and does not wait: the child settles 1 s, reconciles,
  closes its files, closes an open episode `session ended`, and exits; SIGKILL only if it has not after 16 s. The
  node waits for it only at its own shutdown.
- **Over the unix sockets** (`--robot local`): no dependency on the frontend's port (the stack's `--port`), and the
  time base of every record (`t_rx`, `t_tick`, ...) is this machine's monotonic clock, the one the scene records are
  stamped with.
- **From `teleop_start` on**: `--baseline-seq` is the event log's seq just before `teleop_start`, so the session's
  events start with it. The streams connect within about a second of Start; Y and the first A come later.
- **The node's task** stands (`--no-set-task`).
- **Status**: the child prints `Session.stats()` once a second as JSON; the node keeps the newest for `/state`
  `recording` (and the page's Episode card: recording, MB, streams down, scenes, the directory) and turns the child's
  episode lines into node events.
- **Two stacks on one machine** (a second sim with `--runtime-dir`): give each quest_teleop node its own
  `--data-dir`, or both write the default event log and each recorder's reconcile reads the other's events.

## Operator view replay

`python -m robot_utils.replay SESSION|EPISODE_DIR [-o OUT.mp4] [--episode ID]` renders what the operator saw in the
headset as an mp4: the rigid scene from the operator's eye through the AR placement, both controllers where they
were, the goal ghost in its frame colour; no HUD, no status panel, no passthrough (a plain background). It writes
`OUT.mp4` (default `<path>/operator_view[_<episode>].mp4`; H.264, yuv420p) and `OUT.mp4.json` (the summary and each
frame's eye pose in LOCAL and camera in the sim world).

| option | default | |
|---|---|---|
| `--episode ID` | the whole session | one episode of a session (its window from `episodes.jsonl`); a cut episode directory replays as is |
| `--eye` | `center` | `center`: the VIEW space (between the eyes) with the binocular field (the left eye's left edge, the right eye's right edge); `left` / `right`: that eye as it was (its offset and asymmetric field) |
| `--fps`, `--width` | 30, 960 | the height follows the field of view |
| `--controllers` | `overlay` | `overlay`: drawn on top of the scene; `depth`: hidden behind what is in front of them; `off` |
| `--role` | the saved view's | which geoms: `auto` (the robot's and the ghosts' visual geoms, everything else's collision geoms, as the headset), `visual`, `collision` |
| `--bg R,G,B` | 0.12,0.12,0.13 | the background |
| `--start`, `--duration`, `--max-frames` | | a part |
| `--ar-offset`, `--ar-yaw`, `--ar-scale` | 0, 0, 1 | old logs only (reconstructed placement): the headset's AR config |
| `--ffmpeg PATH` | found | else `$IMAGEIO_FFMPEG_EXE`, `imageio_ffmpeg`, `ffmpeg` on the PATH, or the conda base env's `imageio_ffmpeg` binary |

**The chain** (quest_env `native/scene_view.h`, `robot_ar.h`): a sim point `x` is drawn in OpenXR LOCAL at
`R (scale · x) + p`, `(p, R)` = `sim_in_local` of the record's `robot_ar` block (`questToBase()ᵀ · world()`: the
origin 1.6 m ahead of the head along its gravity-aligned yaw at the floor, plus `--ar-offset` / `--ar-yaw`). The
camera is the record's head pose (VIEW in LOCAL) times the eye's pose in VIEW, mapped into the sim world by the
inverse (the position divided by the scale: the same picture), with the eye's frustum (OpenXR FOV tangents, near
0.04 m as on the headset). The controllers' grip poses (LOCAL) take the same map. Per output frame at time `t`: the
newest headset record at or before `t` (the newest tracked head), the newest scene frame received at or before `t`
and its scene.

**Rendering** is MuJoCo offscreen (no other package): each scene becomes an `MjSpec` (dynamic bodies on mocap bodies,
static ones fixed, meshes with one vertex per triangle corner and the face normal, primitives as such, planes left
out, two controller-like shapes), and `mujoco.Renderer` draws it with the camera's frustum set directly
(`mjvGLCamera`). Flat shading as on the headset (`mesh_renderer.h`'s `0.45 + 0.55 |n · L|`, `L` fixed in LOCAL):
two opposite directional lights on a body turned with the placement. Measured on the Mac (loaded): 22 frames/s at
960 x 754 with the sim robot scene.

**What is logged and what is reconstructed.** With a headset build that logs `robot_ar` (quest_env
`feat/ar-replay-log`), nothing is reconstructed: placement, scale, offset, yaw, floor and both eyes' poses and
fields of view are in every record. For older logs (no `robot_ar`; the summary says `"placement": "reconstructed"`
and the tool says so on stderr) the placement is rebuilt by a port of the headset's (`replay/ar.py`): placed at the
**first tracked head in the log** (the real one happened when the AR view started, possibly before the recording)
and again at each `reset_seq` change of the recorded scene frames (taken at the recorder's receipt, not the
headset's) and each XR session change, with the floor **estimated** 1.6 m below the head (the headset used
LOCAL_FLOOR / STAGE) and offset / yaw / scale from the options (the headset's config is not logged there either);
the eyes are a nominal Quest 3 pair (`ar.DEFAULT_EYE_FOV`, 63 mm apart). Always approximate: the scene frame the
headset drew (it lags the recorder by a Wi-Fi hop) and the head pose of the displayed frame (the record is the
input sample, predicted to the same display time).

## Known limits

- Video is every frame from a bounded per-client queue on the camera node (`every=true`): a stall longer than
  the queue still skips frames, counted in the frames' `dropped` and in `episode.json`.
- Pi time is the Pi's monotonic clock: a Pi reboot inside a session breaks the time base of the cut.
