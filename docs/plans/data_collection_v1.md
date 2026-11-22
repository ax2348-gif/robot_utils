# Teleop data collection, v1: draft

Status (2026-11-21): draft. The operator controls and the reset semantics are decided (below); the recorder's
location, format and contents are open decisions. Built after the WBC and the sim are on main; the sim is the
first target, the robot the same code with no reset.

## Goal

Episodes of whole-body teleop (Quest, body-frozen scheme, WBC) recorded with no laptop in the loop: the operator
starts, stops and (in the sim) resets from the controllers, and every episode holds everything needed to train a
policy and to diagnose the run afterwards. The same client, recorder and episode format in the sim and on the robot.

## Decided (2026-11-21)

**Body-frozen is the standard scheme** for the robot from now on. The headset's `standard` scheme stays in
quest_env for its own demo, not for the robot.

**Controller buttons** (all are in the telemetry stream already, so everything except the AR placement is the
Mac client's choice; quest_env changes are marked *headset*):

| button | action | sim | robot |
|---|---|---|---|
| **Y** | enable (WBC lease + enable), as the existing teleop's Y | same | same |
| **X** | disable, as the existing teleop's X | same | same |
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
