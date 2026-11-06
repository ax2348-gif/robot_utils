# robot_utils

One place to set up the robot's software: the motor-control repos as submodules, installed
together into a single Python environment alongside [LeRobot](https://github.com/huggingface/lerobot).

```bash
sudo apt install python3.12-venv  # once, on Ubuntu (the robot ships Python 3.12 without venv support)
git clone --recursive https://github.com/ax2348-gif/robot_utils.git
cd robot_utils
./install.sh                     # creates .venv; see ./install.sh --help
source .venv/bin/activate
```

| Submodule | Tracks | Python package | What it is |
|---|---|---|---|
| `rev_ctl` | `main` | `sparkcan` (+ `sparkcan_py`) | REV SPARK Flex/MAX over CAN; swerve base |
| `rebot_ctl` | `robot_utils` | `reBotArm_control_py` | reBot arm kinematics, dynamics and control |
| `motorbridge_2ec` | `feat/nonblocking-register-read` | `motorbridge` | Rust motor ABI + Python SDK (Damiao/Robstride); fork with fast non-blocking shaft-encoder reads |

`rebot_ctl`'s `robot_utils` branch is `feat/joint-safety-and-diagnostics` (motor CAN watchdog,
joint-limit check, output-shaft reads) plus the packaging fix that makes it pip-installable on
Python ≥ 3.10. Submodules clone over HTTPS, so the robot needs no SSH key.

## Command-line tools

With the venv active, `rebot_ctl` provides (details: `<tool> --help`, `rebot_ctl/README.md`):

- `rebot_setup` — read-only check of the arm's one-time setup (motor CAN watchdog, feedback,
  joint limits); `--store-watchdog` writes the watchdog to motor flash.
- `rebot_zero` — set the current pose as every joint's zero (motors stay disabled).
- `rebot_vis` — all joints slack (MIT, zero gains), live pose in MeshCat reachable from other
  devices on the network; `-g` adds gravity compensation (still zero stiffness).

The Damiao configs find the arm's serial bridge by its stable name
(`serial_by_id: usb-HDSC_CDC_Device_00000000050C-if00`): `/dev/serial/by-id/...` on the robot,
matched by USB serial number on macOS. No `--channel` needed; it remains available to override.

## The environment

**One env, Python ≥ 3.12.** LeRobot ≥ 0.5 requires Python 3.12 and is not capped. The robot
(Ubuntu 24.04, aarch64) ships Python 3.12.3, so `install.sh` uses a plain `venv`.

**`robot_utils` is itself a package** (`src/robot_utils`: the Quest client and the generic
teleop framework, see [docs/teleop.md](docs/teleop.md)). Its dependencies are
the three submodule packages and `lerobot` (no extras), so installing it defines the
environment, and it is the home for shared robot code later.

**Nothing compiles on the robot.** `install.sh`:

1. installs `motorbridge` from the `motorbridge_2ec` fork's prebuilt wheels
   ([releases](https://github.com/ax2348-gif/motorbridge_2ec/releases), chosen to match the
   interpreter and platform);
2. installs `rev_ctl[slcan]` and `rebot_ctl` editable from the submodules (pure Python);
3. installs `robot_utils` editable, which pulls `lerobot`, `torch`, `pin`, … from PyPI wheels;
4. installs the Orbbec camera SDK (`pyorbbecsdk2`, imported as `pyorbbecsdk`) with `--no-deps`:
   it needs only numpy, and its declared dependencies serve its examples (`pynput` → `evdev`
   would compile; `opencv-python` would overwrite lerobot's `opencv-python-headless`);
5. runs `pip check` (ignoring the SDK's skipped example dependencies) and an import check that
   the fork's motorbridge is the one installed.

On Linux the camera also needs Orbbec's udev rule, once per machine; `install.sh` prints the
`sudo` commands if it is missing.

Every step runs under `constraints.txt`, which pins `motorbridge==0.5.6+2ec.1`. That version
exists only on the fork's releases, so nothing can replace it with PyPI's `motorbridge 0.5.6`,
which has a similar number but lacks the fast non-blocking shaft-encoder reads.

`rebot_ctl` must stay editable: it reads `config/` and `urdf/` relative to its repo.

## Updating

- **Submodules:** `git submodule update --remote`, test, commit the new pointers here.
- **motorbridge fork:** bump `bindings/python/src/motorbridge/_version.py` (e.g. `0.5.6+2ec.2`)
  in `motorbridge_2ec`, push a tag `2ec-<version>`; its *Fork wheels* workflow builds Linux
  aarch64/x86_64 and macOS arm64 wheels (cp312, cp313) into a GitHub release. Then update
  `constraints.txt` here to the same version.
