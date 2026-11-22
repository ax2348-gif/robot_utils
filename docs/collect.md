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
| `tests/collect_fakes.py` | a fake robot serving the contract (rows, camera, operator stream, task / start / stop / eventlog) |

## `run`

| option | default | |
|---|---|---|
| `--robot` | (required) | the frontend, e.g. `http://botcore` (every path is `<robot>/nodes/...`) |
| `--out` | (required) | parent directory; the session is `<out>/<YYYYMMDD-HHMMSS>` (`--name` overrides) |
| `--task` | (required) | `POST /nodes/quest_teleop/task {"text"}` before anything else: the task of the next episodes |
| `--streams` | `camera/color,wrist_camera/color,arm/stream,swerve/stream,estimator/stream,wbc/stream,quest_teleop/oplog/stream` | what to record |
| `--start-teleop` | off | `POST /nodes/quest_teleop/start` once recording runs (a 409 ends the session with the WBC's reason) |
| `--stop-teleop` | off | `POST /nodes/quest_teleop/stop` on Ctrl-C (the node aborts an open episode itself) |
| `--rotate` | `512M` | start the stream's next file past this size |
| `--read-timeout` | 10 s | no bytes for this long (every stream pings after 1 s idle) = a dead connection: reconnect |
| `--settle` | 1 s | keep recording this long after Ctrl-C, so the last events and frames arrive |
| `--status-every` | 5 s | status line period |

**Stream specs** (relative to `<robot>/nodes/`): `<node>/.../stream` is that endpoint (`arm/stream`,
`quest_teleop/oplog/stream`); `<camera node>/<camera stream>` is the Orbbec node's
`/stream?streams=<stream>&mode=jpeg&every=true` (every frame in order from a bounded per-client queue; `camera/color`: colour as the camera's MJPG bytes, untouched; no
re-encoding anywhere); a spec with `?` is used verbatim (`camera/stream?streams=depth&mode=raw`).

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
  episodes/<id>/                  after `cut` (below)
```

`<name>` is the spec with a trailing `/stream` dropped and `/` -> `.`: `camera.color`, `wrist_camera.color`,
`arm`, `swerve`, `estimator`, `wbc`, `quest_teleop.oplog`.

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
`gaps.<name>` (the gap lines whose Pi span intersects the window).

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

Reviewing sessions and episodes (replay, plots, 3D, video export): `python -m robot_utils.collect browse`, docs/browse.md.

Training conversion (e.g. LeRobot), the arm's `t_meas` latency compensation and interpolation onto a master
clock are offline steps on these files, not part of the cut.

## Known limits

- Video is every frame from a bounded per-client queue on the camera node (`every=true`): a stall longer than
  the queue still skips frames, counted in the frames' `dropped` and in `episode.json`.
- Pi time is the Pi's monotonic clock: a Pi reboot inside a session breaks the time base of the cut.
