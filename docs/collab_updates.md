# Collaborator updates

What changed for the simulator + Quest teleop path in each `collab-vN` release, and how to update. The setup itself
is in [quest_setup.md](quest_setup.md); the pinned versions are at the top of `tools/quest_teleop/quest_setup.py`.

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
