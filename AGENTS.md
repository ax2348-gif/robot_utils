# AGENTS.md: context for coding agents

robot_utils is the software of a mobile manipulator (swerve base + 6-DoF arm + cameras) and its MuJoCo
simulator. The same code runs on the robot (a Raspberry Pi 5) and, as the simulator stack, on a Mac or Linux
computer. This file is what you need to help someone run the **simulator with Meta Quest teleop**, and to debug it.

## Shape of the system

- **One process ("node") per hardware bus,** each a FastAPI app on a unix socket. A landing page (the
  `robot_utils.frontend` module) indexes them and proxies `/nodes/<name>/`. Conventions are in `docs/nodes.md`.
- **The simulator stack** (`python -m robot_utils.nodes.sim.stack`, `docs/sim.md`) runs the landing page on port
  8080 and these nodes against a MuJoCo world instead of hardware: description, sim, arm, swerve, camera,
  wrist_camera, power, estimator, pose_servo, wbc. Same instance names and APIs as on the robot. The world is a
  task picked on the sim page (rings, revolving door, cart and door, cabinet, tabletop; `deploy/sim/*.yaml`), and the
  headset draws it from its meshes (`docs/scene3d.md`). Sim teleop records itself (`docs/collect.md`); what changed
  per release: `docs/collab_updates.md`.
- **Teleop chain:**
  1. **headset:** the quest_env app sends JSON frames on TCP 9391;
  2. **`quest_teleop` node** (`src/robot_utils/nodes/quest_teleop/`; its core logic is in `tools/quest_teleop/`):
     arming, clutch, body-frozen mapping;
  3. **`wbc` node:** a QP moving the arm and base together, fed by datagrams on a unix socket;
  4. **arm and swerve nodes,** which drive the sim world.
- **The headset side** lives in two other repositories, both on branch `collab` (quest_dev tag `collab-v4`: runtime
  1.7.0; quest_env tag `collab-v4`):
  - [quest_dev](https://github.com/ax2348-gif/quest_dev): an Android app giving the Quest an SSH server (port 8022)
    and a native XR host;
  - [quest_env](https://github.com/ax2348-gif/quest_env): the headset app, its `quest` script and native telemetry
    library.

  Each has its own AGENTS.md.
- **Network:** Tailscale.
  - The computer reaches the headset as `ssh quest` (port 8022) and reads its stream at the same address on
    port 9391.
  - The headset fetches the AR view (the simulated robot) from the computer on port 8080. So the stack must serve
    on all interfaces (`--host 0.0.0.0`, implied by `--quest`), and the computer's firewall must allow it.

## Setup and daily use (the collaborator path)

`docs/quest_setup.md` is the walkthrough: checklist, install, the wizard, the one-step launcher, troubleshooting.
In short:
- **Install:** `./install.sh --no-service && source .venv/bin/activate && pip install -e '.[sim]'`. This needs
  Python 3.12 and git submodules (`git clone --recursive`).
- **Headset:** `python3 tools/quest_teleop/quest_setup.py`, with the headset on USB. It's an idempotent wizard:
  each step checks, fixes only what is missing, and is safe to rerun. `--check` changes nothing. Its pins
  (runtime version and SHA-256, quest_env tag) are at the top of the file. Its tests are in
  `tests/test_quest_setup.py` (a simulated headset).
- **Run:** `python -m robot_utils.nodes.sim.stack --quest --no-pull`. It starts the stack plus quest_teleop (its
  args in `src/robot_utils/nodes/sim/stack_nodes.yaml`: `--body core --elbow-hint`, the Mac's freshness limits,
  `--resume-grace 4`), runs `POST /nodes/quest_teleop/quest/up` (retried while the headset sleeps), then
  `POST /nodes/quest_teleop/start` (the sim resets; the launcher makes sure it runs afterwards). The operator presses
  Y to arm. The default robot is the one-arm reBot; `--robot openarm_bimanual` runs the OpenArm (two 7-DOF arms:
  Enactic's OpenArm v2.0 with Anvil's joint ranges and gripper, `src/robot_utils/robots/openarm/README.md`).

## Diagnosing

| look at | how |
|---|---|
| the whole setup | `python3 tools/quest_teleop/quest_setup.py --check` |
| the headset app | `tools/quest_teleop/quest_status`, `quest_up`, `quest_down` (ssh, with logs). In the headset: `ssh quest 'sh workspace/quest_env/quest status'` |
| the stream | `python3 -c "import sys; sys.path.insert(0,'tools/quest_teleop'); import quest_ctl as q; print(q.probe_stream('<headset 100.x>', 9391).describe())"` |
| the runtime (USB) | `adb shell content read --uri content://dev.quest.runtime.files/status` (also `/log`, `/autostart`) |
| the teleop node | its page `/nodes/quest_teleop/`; `GET /nodes/quest_teleop/state` (`teleop`, `headset`, `ctl` = the last quest up/down with its log lines, `error`) |
| a node that did not start | the landing page tile's **logs**, or `GET /api/nodes/<name>/logs?lines=50` |
| Tailscale | `tailscale status`, `tailscale ping <headset address>` |

**Common causes:**
- **The headset is asleep:** SSH and the stream time out. Press its power button.
- **The XR session is not focused** (the headset not worn, the privacy lock, another app in front): the stream
  says NOT focused and tracking is invalid.
- **The two devices are on different tailnets.**
- **The macOS firewall** blocks port 8080, so there is no AR view.
- **An existing `Host quest` entry** in `~/.ssh/config` points elsewhere. The wizard refuses to edit an entry it
  did not write; use `--alias`, then start the node with `--ssh-host`.
- **A runtime from another build** (another signing key) cannot be updated in place. The wizard migrates it:
  backup to `~/quest-backups/`, reinstall, restore with the same host key (docs/quest_setup.md, "Migrating a
  runtime"). The backup is the recovery path if anything goes wrong after the uninstall.
- **The headset was provisioned by another computer:** `Permission denied (publickey)`. The wizard asks before
  replacing the keys.

## Working in this repo

- **Tests:** `python -m pytest tests/...` in the venv (`pip install pytest`). The suite is large; run the files you
  touch, e.g. `tests/test_quest_*.py` and `tests/test_sim_stack.py`.
- **Docs** are in `docs/` and per tool. Update them with behaviour changes. They are terse and specific; keep
  that style.
- **This repo also runs a real robot.** Code paths must keep working on the Pi (Python 3.12, Ubuntu 24.04,
  aarch64). Never point anything at a real robot's hosts unless you're asked to; the simulator needs none.
- **Integration fixes for the collaborator path** belong on the `collab` branches of the three repositories. If
  you change the runtime APK or quest_env, the pins in `quest_setup.py` must follow, together with new tags.
