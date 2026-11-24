# Quest teleop in the simulator: setup

From a new Meta Quest and a Mac or Linux computer to whole-body teleop of the simulated robot, with the robot drawn
over passthrough in the headset. About 30 minutes the first time; afterwards one command.

What runs where:
- **The headset:** the Quest developer runtime (an SSH server) and the quest_env app (sends head and controller
  poses on port 9391).
- **Your computer:** the simulator stack: the MuJoCo world, the robot's nodes, and the teleop node that reads the
  headset.
- **Between them:** Tailscale (a private network). It works on any Wi-Fi, without a cable, once set up.

## Before you start: have these ready

- [ ] **The headset**, charged, and a **USB-C data cable** (some charging cables carry no data).
- [ ] **Developer mode** on the headset:
  1. Create a Meta developer organisation (free) at
     <https://developers.meta.com/horizon/manage/organizations/create/>, with the same Meta account as the headset.
  2. In the **Meta Horizon** phone app: Devices → your headset → Headset settings → **Developer mode** on.
  3. Reboot the headset.

  Meta's own guide, if the menus have moved:
  <https://developers.meta.com/horizon/documentation/native/android/mobile-device-setup/>.
- [ ] **A computer:**
  - macOS on Apple Silicon, or Linux x86-64 / arm64. Windows is not supported yet, and neither are Intel Macs.
  - Git and Python 3.12. On a Mac: `brew install python@3.12`.
- [ ] **adb:**
  - macOS: `brew install android-platform-tools`
  - Ubuntu: `sudo apt install adb`
- [ ] **Tailscale** (free account) on the computer, signed in and Connected: <https://tailscale.com/download>. The
  wizard installs it on the headset; there you sign in with the **same** account.
- [ ] **macOS:** when the firewall asks whether Python may accept incoming connections, allow it. The headset
  loads the simulated robot from your computer.

## 1. Install robot_utils with the simulator

```bash
git clone --recursive -b collab https://github.com/ax2348-gif/robot_utils.git
cd robot_utils
./install.sh --no-service          # creates .venv (Python 3.12); --no-service: no robot services on this machine
source .venv/bin/activate
pip install -e '.[sim]'            # MuJoCo
```

Check: `python -m robot_utils.nodes.sim.stack --no-pull` starts the simulator. Open <http://localhost:8080/>,
then press Ctrl-C to stop it.

## 2. Set up the headset (the wizard)

Plug the headset into the computer, wake it, and run:

```bash
python3 tools/quest_teleop/quest_setup.py
```

It goes through these steps, asking before anything that cannot be undone:
1. **adb:** finds the headset. Put the headset on once and choose **Allow** on "Allow USB debugging?" (tick
   *Always allow*).
2. **Tailscale on this computer:** checks that it is connected.
3. **Tailscale on the headset:**
   - installs it over USB if it is missing;
   - opens it in the headset: **sign in** with the same account and **allow the VPN** connection;
   - the wizard waits until the headset has its Tailscale address.
4. **The Quest developer runtime:** installs the APK, checked by SHA-256. Under Tailscale's own settings in the
   headset, *Always-on VPN* keeps the headset reachable after a reboot. A runtime from another build cannot be
   updated in place; if it answers over SSH, the wizard offers to migrate it (see below), otherwise it asks
   before uninstalling. Declining keeps the installed runtime.
5. **SSH:**
   - creates `~/.ssh/quest_ed25519` and authorises it on the headset;
   - adds `Host quest` to `~/.ssh/config`;
   - checks that `ssh quest` works over Tailscale. An existing `Host quest` entry of your own is kept; if it
     does not offer the wizard's key, the wizard offers to add one `IdentityFile` line to it;
   - `--extra-keys FILE` (repeatable): public keys of other machines that should reach the headset (a robot, a
     second computer). Added when missing on every run, never removed. Those machines record the headset's host
     key with the `known_hosts` line the wizard prints at the end.
6. **quest_env:** copies the headset app to the headset, installs Git there (quest_env's developer tools), and
   makes it a Git clone tracking branch `collab` (`--quest-env-branch` picks another), so quest up can update it.
   Then it verifies it.
7. **Optional check:** starts the headset app once, reads its stream, then stops it.

**Migrating a runtime from another build.** The wizard backs up the runtime's data over SSH into
`~/quest-backups/runtime-<serial>-<time>.tar`: its home (workspace, dotfiles, installed tools), authorised keys, SSH
host key, address and settings. It refuses to continue if the backup looks incomplete, and only then uninstalls,
installs the new runtime, restores everything except the BlackBox container, and restarts SSH with the old host
key, so every machine that knew the headset still trusts it. The backup is kept; delete it once the headset
works. If a restore is interrupted, rerun the wizard, or restore by hand once `ssh quest` works:
`ssh quest 'cd "$HOME/../.." && tar -xpf - --exclude "blackbox/*"' < ~/quest-backups/<file>.tar`, then restart the
runtime (`adb shell content call --uri content://dev.quest.runtime.files --method stop`, then `--method start`).

**Stopped, or something failed?** Fix what it says and run it again: finished steps are skipped.
`python3 tools/quest_teleop/quest_setup.py --check` reports the state without changing anything. Unplug the cable
when the wizard reports *Done*.

## 3. Try it

```bash
source .venv/bin/activate
python -m robot_utils.nodes.sim.stack --quest --no-pull
```

One command:
1. It starts the simulator on your network (so the headset can load it) and the teleop node.
2. It wakes the headset app over Tailscale with the robot's AR view. Press the headset's power button if it sleeps;
   the launcher retries.
3. It starts teleop.

Then put the headset on (or wear it around your neck: it tracks your torso), stand where you want the robot to
start, and:

| control | does |
|---|---|
| **Y** (left) | arm. Nothing moves before it. **X** disarms. |
| **right grip**, held | the arm follows your hand; release to let go |
| **right trigger** | the gripper (pull to close) |
| **left stick / right stick** | drive the base / turn it (the left stick follows the direction you face) |
| walking and turning, grip held | the robot walks with you |
| **right stick click** | re-register: face where the robot faces, then click |
| **A / B** (right) | start / stop a recorded episode (the simulator resets first) |
| **B** held 2 s | stop the headset app |

Open <http://localhost:8080/nodes/quest_teleop/> for its state, the event log and a top-down view, and
<http://localhost:8080/nodes/sim/> for the 3D world. Ctrl-C stops everything, including the headset app.

**Every later session:** wake the headset, then run `python -m robot_utils.nodes.sim.stack --quest --no-pull`.
No cable is needed.

## When something goes wrong

| symptom | likely cause | fix |
|---|---|---|
| wizard: no headset on adb | developer mode off, a charge-only cable, headset asleep | the checklist above; another cable; reboot the headset |
| wizard: `unauthorized` | the USB debugging prompt is waiting in the headset | put it on, Allow, tick Always allow |
| wizard waits for the headset's Tailscale address | not signed in, VPN not allowed, a different tailnet | open Tailscale in the headset (Library → Unknown sources), sign in with the computer's account |
| `ssh quest` times out | the headset sleeps, or Tailscale is off on either side | press the power button; both Tailscale apps Connected; `tailscale ping <headset address>` |
| wizard: "signed by another build" | the runtime came from another computer's build | answer yes to migrate (backup, reinstall, restore); no keeps it |
| another machine: `Permission denied` or "host identification has changed" | its key is not authorised, or the runtime was reinstalled | rerun with `--extra-keys <its .pub>`; on it, `ssh-keygen -R "[<headset>]:8022"` and add the printed `known_hosts` line |
| `Permission denied (publickey)` | the headset was set up by someone else's computer | rerun the wizard; it asks before replacing the headset's keys |
| launcher: "the headset is not up yet", forever | asleep, offline, or the XR app lost focus | wake it; `python3 tools/quest_teleop/quest_setup.py --check` |
| stream "NOT focused" / no tracking | the headset is not worn, the privacy lock is on, another app is in front | put it on once; close other apps |
| no robot in the headset (AR) | the computer's firewall, or the stack not on the network | allow Python incoming (macOS); start with `--quest` (it serves on all interfaces) |
| `install.sh` fails on Python | Python 3.12 missing | `brew install python@3.12` / your distribution's python3.12 + venv |
| the robot does not move after Y | teleop not started, or a refusal | the teleop page shows why (`/nodes/quest_teleop/`) |

**Using an AI coding agent to help:** each repository has an `AGENTS.md` with the context it needs: this one, plus
[quest_env](https://github.com/ax2348-gif/quest_env/blob/collab/AGENTS.md) and
[quest_dev](https://github.com/ax2348-gif/quest_dev/blob/collab/AGENTS.md).

## Pinned versions

The wizard installs fixed versions:
- the runtime 1.6.0 from [quest_dev](https://github.com/ax2348-gif/quest_dev/tree/collab-v2) tag `collab-v2`,
  checked by SHA-256;
- the headset app from [quest_env](https://github.com/ax2348-gif/quest_env/tree/collab-v2) tag `collab-v2`,
  which then tracks branch `collab`.

They are set at the top of `tools/quest_teleop/quest_setup.py`. A headset whose quest_env is already a Git clone
tracking a branch keeps it.
