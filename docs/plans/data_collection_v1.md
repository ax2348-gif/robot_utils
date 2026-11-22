# Teleop data collection, v1: draft

Status (2026-11-21): draft; **recording deferred** until the teleop integration (WBC + sim + Quest) is done. The
operator controls and the reset semantics are decided (below) and are built **now, as part of the integration**,
without recording: Y/X enable/disable, A = reset (sim: the safe reset + AR re-anchor; robot: AR re-anchor only),
B inert (the headset's 2 s hold still stops telemetry). Recording later adds "then record" to A and "stop" to the
B tap, for the sim and the robot alike. The recorder's location, format and contents are open decisions.

## Goal

Episodes of whole-body teleop (Quest, body-frozen scheme, WBC) recorded with no laptop in the loop: the operator
starts, stops and (in the sim) resets from the controllers, and every episode holds everything needed to train a
policy and to diagnose the run afterwards. The same client, recorder and episode format in the sim and on the robot.

## Architecture (decided 2026-11-21, later; supersedes the client-side parts below where they differ)

**Hybrid: teleop on the robot, everything else client-side.**
- **The Pi's `quest_teleop` node is the teleop implementation** in production as in testing: it reads the headset
  (one consumer), runs the client core (body-frozen, the armed latch, Y = arm / X = disable, A = start episode and
  the sim reset sequence, B tap = stop and keep), and sends datagrams to the WBC over the local unix socket. The
  control loop never crosses erid / the Mac: a recording machine that crashes, sleeps or loses Wi-Fi changes nothing
  about teleop or its safety.
- **A client-side script (erid for real collection, the Mac for the sim) controls everything else:** starting a
  session through the node's API, the task text, which streams to record, and writing the files. The Pi never
  records streams or video.
- **The node publishes what it does as a full-rate stream:** each headset frame (stamped on arrival in Pi time) and
  each datagram it sent, plus the episode events. Episode boundaries are DATA in that ordered stream, so the script
  cuts episodes exactly even when it lags or reconnects; recording continuously and cutting afterwards gives
  pre-roll for free.
- **An event log on the Pi** (a deliberate small exception to "the Pi never records", like agent_interface's
  transcripts): button edges (A/B/X/Y), arm / disarm (reason, arm_seq), enable / disable, episode start / stop /
  abort (reason), e-stops, faults, holds, staleness / latency trips, teleop start / stop, sim resets — events only,
  never frames — with the Pi's monotonic and wall time. One append-only JSONL file per day under
  `~/.local/share/robot_utils/quest_teleop/`, each line flushed and fsynced, pruned at a size cap (oldest first).
  `GET /eventlog?since=` and `GET /eventlog/files`: the script retrieves what a Wi-Fi drop cost it and reconciles the
  boundaries. Built after the safety build merges.
- No headset HUD: the headset is an input device; recording state shows on the script's / node's page.

**Recommendations standing (2026-11-21):** raw stream bytes + an episode index, converted offline (e.g. LeRobot);
MJPG as sent with an every-frame camera stream mode for recorders (gaps counted per episode); the Pi clock as the
time base (WBC tick as master, alignment offline); seeded per-object position / yaw randomisation in the sim scene,
the seed in the episode; the review page later. Episode outcome labels: open, out of scope for now (abort reasons
are recorded automatically).

## Decided (2026-11-21)

**Body-frozen is the standard scheme** for the robot from now on. The headset's `standard` scheme stays in
quest_env for its own demo, not for the robot.

**Controller buttons** (all are in the telemetry stream already, so everything except the AR placement is the
Mac client's choice; quest_env changes are marked *headset*):

| button | action | sim | robot |
|---|---|---|---|
| **Y** | arm (enable if disabled): the armed latch, a deliberate press only (docs/wbc.md) | same | same |
| **X** | disable (level-triggered: the arm floats, the gripper damps, the base coasts) | same | same |
| **A** | **start an episode** | reset, re-anchor the AR scene, then record | record (no reset; re-anchor the AR scene) |
| **B** tap (released < 2 s) | **stop the episode** (keep) | same | same |
| **B** hold 2 s | stop telemetry (unchanged headset behaviour); an open episode ends with it | same | same |

- *Headset:* in body-frozen mode **A no longer realigns forward**. The body-frozen client derives its operator frame
  from the head at every engagement and never uses the realigned forward; the realignment only bumped the frame
  revision (a needless discontinuity) and set the AR heading. A keeps realigning in the `standard` scheme.
- *Headset:* **Y no longer re-places the AR scene.** The scene is anchored once at start and **re-anchored on every
  reset**: in front of the operator, heading = the head's gravity-aligned yaw at that moment (not the realigned
  forward). The headset learns of a reset from a counter in the `robot_state.v1` lines (`reset_seq`, below), so
  reset and re-anchoring are one event with one source of truth. On the robot (no reset) the scene re-anchors when
  an episode starts.
- B tap vs hold: the client acts on B's **release** if it was held under 2 s; the headset's 2 s hold still stops
  telemetry. A tap only flashes the hold arc.

**Reset (sim) is one event, owned by the sim:**

- `POST /nodes/sim/scene/reset` (exists): time 0, the robot at its start pose, objects at their scene poses. New:
  `reset_seq` incremented in `/state` and in every `robot_state.v1` line; objects optionally **randomised** within
  per-object ranges in the scene file (seeded; the seed recorded in the episode).
- **A reset must not fight the nodes' commands.** The world's arm is kinematic: if the arm node still holds its old
  `q_cmd`, the next tick drives the reset arm straight back. So the client's A sequence is: stop sending targets ->
  WBC disable (arm and swerve disabled, the joint-channel session closed) -> sim reset -> wait for the estimator's
  reset to show in the WBC stream (`est_reset` changed) -> WBC enable (re-seeded from the measured joints) -> start
  recording. The operator sees one press; the robot is never enabled across the jump.
- Engagements: the estimator reset already makes the client drop any engagement and require a release.

## Episode lifecycle

- `idle` -> A -> (`resetting`, sim only) -> `recording` -> B tap -> `saving` -> `idle`.
- **Disable (X), the e-stop, a WBC fault, telemetry loss over the stale timeout or B hold** while recording: the
  episode ends and is marked `aborted` with the reason (kept, not deleted; training filters on it).
- A while recording: ends the current episode (kept) and starts the next.
- Recording runs only while enabled; A while disabled is refused with a cue.
- Feedback to the operator: the headset HUD shows the client's state (`REC 00:12`, `RESETTING`, `SAVED 17`,
  `ABORTED: estop`). Needs a small *headset* change: a status line the client can set (the AR state stream's
  overlays already reach the HUD; a `status` text overlay is the simplest route).

## What an episode holds (proposal)

| stream | source | rate | why |
|---|---|---|---|
| WBC rows (targets, commands, `limited*`, base twist, `est_reset`) | wbc `/stream` | 100 Hz | actions, limits |
| arm rows (`q`, `q_cmd`, `t_meas`, `t_meas_mot`, gripper) | arm `/stream` | 100 Hz | proprioception |
| swerve rows | swerve `/stream` | 100 Hz | base state |
| estimator rows (pose, twist) | estimator `/stream` | 100 Hz | base pose |
| head colour (+ depth?) | camera | 30 fps | observation |
| wrist colour (left / both) | wrist_camera | 30 fps | observation |
| head-camera IMU | camera `/imu/stream` | 100 Hz | optional |
| Quest frames (raw) | headset 9391 | ~70 Hz | operator input, for replay and diagnosis |
| datagrams sent to the WBC | client | per frame | the exact commands |
| sim: `robot_state.v1`, scene, seed | sim | 30 Hz | ground truth (sim only) |
| meta | client | once | task text, operator, success label, versions, calibration ids, `model_version` |

## Open decisions

1. **Where the recorder runs:** the Mac (where the teleop client runs; simplest) or erid (disk, GPU, wired-quality
   Wi-Fi; the client could run there too). The Pi never records (nodes.md).
2. **Format:** raw stream bytes per stream (what `NodeClient.record` writes today; lossless, already parseable) plus
   an episode index, converted to a training format (e.g. LeRobot) by a separate step; or write the training format
   directly. Proposal: raw + index, conversion offline.
3. **Video:** the camera streams are latest-frame-wins, so a slow recorder silently drops frames. Record the MJPG
   bytes as they arrive and count gaps per episode (no re-encoding on the recorder); or an every-frame stream mode.
4. **Time base:** every stream carries Pi-monotonic stamps (`t_tick`, `t_meas`, camera `t_sensor`); alignment
   offline with one master stream (WBC tick) and interpolation, as the estimator does. Quest frames are on the
   headset's clock: stamp them on arrival with the Pi clock offset the client already estimates?
5. **Labels:** success/failure and the task text: set on the Mac before a session, per episode from the headset
   (e.g. B tap = success, B double tap = failure?), or afterwards in a review page.
6. **Randomisation** of the sim scene on reset: ranges per object in the scene file; camera/lighting later.
7. **Episode review page:** list, play back synchronised video and plots, relabel, mark bad. v1 or later?

## Order of work (proposed)

1. Sim: `reset_seq` in `/state` and `robot_state.v1`; scene randomisation with a seed.
2. Headset (quest_env): body-frozen A without realignment; Y without AR re-placement; AR re-anchors on `reset_seq`
   change (and on an episode start on the robot) with the head's yaw; a client status line on the HUD.
3. Client: the button state machine (Y/X/A/B), the safe reset sequence, the HUD status.
4. Recorder: episode directories, the streams above, meta, abort reasons; tests against the sim stack.
5. Conversion to the training format; the review page if wanted.
