# robot_utils

One place to set up the robot's software: the motor-control repos as submodules, installed
together into a single Python environment alongside [LeRobot](https://github.com/huggingface/lerobot).

```bash
git clone --recursive https://github.com/ax2348-gif/robot_utils.git
```

| Submodule | Tracks | Python package | What it is |
|---|---|---|---|
| `rev_ctl` | `main` | `sparkcan` (+ `sparkcan_py`) | REV SPARK Flex/MAX over CAN; swerve base |
| `rebot_ctl` | `main` | `reBotArm_control_py` | reBot arm kinematics, dynamics and control |
| `motorbridge_2ec` | `feat/nonblocking-register-read` | `motorbridge` | Rust motor ABI + Python SDK (Damiao/Robstride); fork with fast non-blocking shaft-encoder reads |

## Environment plan

**One env, Python 3.12.** LeRobot ≥ 0.5 requires Python ≥ 3.12 and is not capped; the robot
(Ubuntu 24.04, aarch64) ships Python 3.12.3, so a plain `venv` on the system Python is enough.

**`robot_utils` is itself a package** (`src/robot_utils`, empty for now) whose dependencies are
the three submodule packages and `lerobot` (no extras). Installing it therefore defines the
environment, and it is the home for shared robot code later. Install order:

1. the `motorbridge_2ec` wheel (prebuilt, from a fork release);
2. `rev_ctl` and `rebot_ctl`, editable from the submodules;
3. `robot_utils` itself, editable, which pulls in `lerobot` from PyPI;

all under `constraints.txt`, which pins `motorbridge` to the fork's wheel so no step can swap in
PyPI's copy.

**Nothing compiles on the robot.** Every dependency installs from a prebuilt aarch64 wheel:

| Package | Source on the robot | Notes |
|---|---|---|
| `motorbridge` | prebuilt wheel from a `motorbridge_2ec` GitHub release | **must** be the fork: PyPI's `motorbridge 0.5.6` has the same version number but lacks the fast non-blocking shaft-encoder reads the arm needs to keep pace with `rev_ctl` |
| `sparkcan` | `pip install -e rev_ctl[slcan]` | pure Python |
| `reBotArm_control_py` | `pip install -e rebot_ctl` | pure Python; must stay *editable* — it reads `config/` and `urdf/` relative to the repo |
| `lerobot`, `torch`, `pin`, `opencv`, … | PyPI | all publish manylinux aarch64 wheels |

### Verified (macOS arm64, Python 3.12, 2026-11-04)

Installed into one env: fork `motorbridge` 0.5.6 (built from the submodule), `sparkcan` 0.1.0,
`rebot_ctl`'s dependencies (`pin` 4.1.0, `meshcat`, `matplotlib`), `lerobot` 0.6.1
(`torch` 2.11.0, `numpy` 2.2.6). `pip check` is clean, everything imports together, the fork's
motorbridge survives the LeRobot install, and both test suites pass (`rebot_ctl` 16, `rev_ctl` 151).

### Changes needed before the robot install works

1. **`rebot_ctl`: make it installable.** Its `pyproject.toml` has no build system or package
   list (setuptools refuses: *multiple top-level packages: urdf, config, reBotArm_control_py*),
   and declares `requires-python <3.12` for no stated reason — it runs fine on 3.12. Add a
   setuptools build section that packages `reBotArm_control_py*` and relax the bound to `>=3.10`.
2. **`motorbridge_2ec`: publish wheels from the fork.** The inherited `pypi-publish.yml` already
   builds manylinux aarch64/x86_64 and macOS wheels with cibuildwheel, but targets PyPI and has
   never run on the fork. Point it at GitHub Releases, and give the fork a distinct version
   (e.g. `0.5.6+2ec.1`) so it can never be confused with PyPI's `0.5.6`.
3. **`robot_utils`: add `install.sh` and `constraints.txt`.** The script creates the venv and
   runs the install order above, then a smoke check (`pip check`, imports, the fork's API).
   The package skeleton is already here; until (1) lands, `pip check` reports only
   `rebotarm-control-py` missing.

The robot's existing `~/sparkcan` copy is left as it is for now.
