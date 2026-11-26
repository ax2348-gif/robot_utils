# Collaborator updates

What changed for the simulator + Quest teleop path in each `collab-vN` release, and how to update. The setup itself
is in [quest_setup.md](quest_setup.md); the pinned versions are at the top of `tools/quest_teleop/quest_setup.py`.

## collab-v4 (2026-11-26)

### Updating from collab-v3

1. **robot_utils:** `git pull` on branch `collab` (or `git checkout collab-v4`), then
   `source .venv/bin/activate && pip install -e '.[sim]'` (new package data: the OpenArm's URDFs and meshes).
2. **Runtime 1.7.0** (body tracking), with the headset on USB: `python3 tools/quest_teleop/quest_setup.py`. The
   wizard sees 1.6.0, installs 1.7.0 from quest_dev `collab-v4` (SHA-256 checked) and reports the body-tracking
   permission. It is the same release key as 1.6.0, so it updates in place: your keys, host key, address and
   workspace stay. A runtime from another build is migrated as before (backup in `~/quest-backups/`, reinstall,
   restore).
3. **Headset app:** nothing to do. The next `--quest` start pulls quest_env's branch `collab` (collab-v4: body
   tracking and controller haptics) and restarts the headset app. The launcher now starts it with body tracking
   (`--body core`), which an older quest_env refuses: if quest up fails with "--body needs quest_env with body
   tracking", rerun the wizard.
4. To check: `python3 tools/quest_teleop/quest_setup.py --check` (runtime 1.7.0, body tracking granted).

### What's new

**OpenArm bimanual** (`--robot openarm_bimanual`; the default robot is still the one-arm reBot): two 7-DOF arms on
the swerve base at human scale (shoulders 1.074 m, work surface 0.70 m), every task and the dynamic arms
(`deploy/sim/sim_openarm_dynamic.yaml`).
- **The arms:** Enactic's OpenArm v2.0 (Apache-2.0) with the **Anvil OpenArm 2.0's joint ranges**, J6 (the wrist)
  left −45..+70°, right −70..+45°; J1 keeps Enactic's window (200° of forward flexion).
- **The gripper:** Anvil's Customizable Gripper: 0-100 mm, its meshes, mass and wrist camera.
- `src/robot_utils/robots/openarm/README.md` has the sources, the limits and what is excluded (Anvil's arm model is
  not distributed).
- **The start pose:** upper arms down, forearms forward, tools forward with the wrist cameras up.

**Two hands, two arms.** Each hand's grip drives one arm, its trigger that arm's gripper. **Mirror pairing**
(`--mirror auto`, the default): the hands swap arms when the robot faces you; decided at the first grip, locked
until both hands release; the teleop page switches it.

**Body tracking and the elbow hint.** The headset streams Meta body tracking (runtime 1.7.0). With `--elbow-hint`
(on in the launcher) each engaged arm's elbow swivels with yours: lift your elbow, the robot's elbow lifts. The
swivel is held when you release, so it accumulates across grips; base turns do not change it. The WBC's `swivel_dir`
picks the model's rest direction (`head`, the default; `down` keeps the elbow hanging). Optional **chest base hint**
(`--hint-source chest`, or the teleop page): the base follows your chest instead of the headset, so looking around
does not steer it.

**Hold and re-grip.** A short dropout (the stream late or slow, a glitch, a reconnect of the same headset session,
the WBC's own input timeout) no longer disarms: the robot **holds**, and releasing and gripping again resumes, no Y.
A hold keeps the recording episode. Longer than `--resume-grace` (4 s in the launcher) it disarms as before (Y).

**Haptics.** The controllers vibrate on armed, disarmed, held, resumed, disabled, a refused Y (three quick taps) and
the gripper closing on something; optional continuous cues for limits and grip force. The teleop page's Haptics card
switches and tests each.

**Stick shaping.** While gripping, the stick moves the robot's reference at the base's own acceleration limits
(ramped at 0.75 x the WBC's `max_accel` / `max_alpha`, full deflection 0.85 x the speed caps), so the arm no longer
runs ahead of a base that starts late. The stick tether is off by default (`--tether`).

**The launcher** starts the teleop as the maintainer runs it: `--body core --elbow-hint --max-latency-ms 300
--stale-ms 350 --min-rate 15 --resume-grace 4` (`stack_nodes.yaml`), the WBC with `--hold-timeout 0.4`, and makes
sure the sim runs after its start reset.

**Sim fixes:**
- The base stopped for ~0.5 s every ~10 s (the swerve's encoder rebase never confirmed in the sim, so the estimator
  dropped its odometry and the WBC disarmed): fixed.
- The cart-and-door task has a second pull bar at a person's handle height (1.05 m).
- A crash of the headset gate ("deque mutated during iteration") that stopped teleop and disabled the WBC.

### Known issues

- **A released arm near a singularity blocks the base** (`docs/wbc.md`, "Known bugs"): with the hands released and an
  arm nearly straight, a stick translation barely moves the base, which turns slowly instead. Workaround: grip, bend
  that elbow, release.
- **Load:** the OpenArm with dynamic arms, a task scene and the headset's mesh view load a Mac heavily (the sim alone
  about two cores); a busy Mac holds more often (late input). Close other heavy work.
- Body tracking needs the headset worn (or around the neck, the torso visible to it); without it the elbow hint
  fades out and the base hint falls back to the headset.

## collab-v3 (2026-11-25)

### Updating from collab-v2

1. **robot_utils:** `git pull` on branch `collab` (or `git checkout collab-v3`), then
   `source .venv/bin/activate && pip install -e '.[sim]'` (no new dependencies, but it is cheap).
2. **Headset:** nothing to do. A headset set up by the collab-v2 wizard tracks quest_env's branch `collab`, and the
   next `--quest` start (`quest up`) pulls it and restarts the headset app on the new code. To check:
   `python3 tools/quest_teleop/quest_setup.py --check`.
3. **Runtime:** unchanged (1.6.0, from quest_dev `collab-v2`).

### What's new

**The sim in the headset is the real scene now.** The headset draws the simulator's world from its meshes (the robot
from its URDF meshes, scene objects, articulated objects), not boxes: `docs/scene3d.md` (the rigid-scene format,
`rigid_scene.v1`) and quest_env's mesh renderer. Measured budget on a Quest 3S: about 400 k triangles at 72 Hz.
- **Styles:** flat shading (the default), flat with edges, hidden-line wireframe, plain wireframe; and which
  geometry (visual or collision). The "Headset view" button on the sim page or the teleop page cycles them
  (`/scene3d/view`).
- **Placement:** the sim world is placed so that the robot and the task start in front of you (the scene's view
  anchor).
- **Goal ghost:** a see-through copy of the gripper at its commanded pose, so tracking error and deflection are
  visible. Optional **auto pause**: sim time stops while the clutch is released and the base is not commanded
  (teleop page, "sim aids").

**Tasks.** The sim page has a task picker (`GET /scene/tasks`, `POST /scene/task`) and a list of saved scenes:
- **rings:** two counter-rotating rings with red and blue objects; sort red onto the outer ring, blue onto the inner.
  Objects slowly creep outward on the turning rings and fall off the outer edge after ~20 min (kept on purpose).
- **revolving door:** 5 wings in an enclosure; walk through it (it takes nearly a full turn).
- **cart and door:** pull a castered cart with a U-handle through a door that opens toward you, against a gentle
  closer.
- **cabinet**, **tabletop** (as before).

Each task has its own config in `deploy/sim/` (e.g. `ROBOT_SIM_CONFIG=deploy/sim/sim_rings_dynamic.yaml`).

**A dynamic arm.** `fidelity: dynamic` (the `*_dynamic.yaml` configs): the arm is driven as the real one is, joint
torques from its motors' control law, with the gravity fitted to the real arm, so it sags, holds and complies like
it. Grasps hold (the jaws' contact was fixed; grasped objects no longer slip out).

**Recording, built in.** Sim teleop records itself: every A starts an episode (the sim resets first), B ends it,
into `~/data/sim/<session>/` (`docs/collect.md`): the robot streams, the operator stream and the rigid scene.
- **Replay from the operator's eyes:** `python -m robot_utils.replay <session> [--episode ID]` renders what the
  headset showed (without the HUD) to an MP4.
- **Data browser:** `python -m robot_utils.collect browse <session>`: replay in 3D (with the recorded scene) and 2D,
  scrub, change the rate, and export MP4s (`docs/browse.md`).

**More forgiving on a Mac.** A Mac doesn't schedule the nodes in real time, and a busy one used to fault them ("arm
control process reported dead", the WBC's "arm state stale", the sim's motor watchdog). On macOS the sim stack now
uses lenient stall limits (`--timing`, `timing.lenient`; never on the robot).

**Fixes you may have hit:**
- After A (sim reset) the arm stopped following while the base still moved: the reset now keeps the arm motors on,
  and resets the estimator too (the WBC waited forever for it).
- The sim cameras froze after a reset.
- The base slowly rolled away (~2 cm/s) whenever it was not commanded: the wheels now have rolling resistance.
- The headset kept running the old app after an update (`quest up` now restarts it when the code changed).

**For developers:** `tools/agent_rings/` runs a sandboxed coding agent against the rings task through the robot's
API only (no simulator ground truth), with a scorer and a request log; its README explains the containment.

### Known issues

- **Load:** the rings and door scenes with the dynamic arm use about two cores for the sim alone; on a busy Mac the
  teleop can stutter. Close other heavy work.
- **Tests:** a few real-time tests (`arm_proc`, `wbc_proc`, `wbc_overrun`, the sim integration cameras) can fail on
  a loaded machine; they pass on an idle one.
- The replay renders see-through walls opaque.
