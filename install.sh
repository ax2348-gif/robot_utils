#!/usr/bin/env bash
# Set up the robot's Python environment. Nothing is compiled: motorbridge comes from the
# motorbridge_2ec fork's prebuilt wheels, everything else from PyPI wheels or pure Python.
#
#   ./install.sh                 # creates ./.venv with python3.12
#   ./install.sh --venv ~/robot  # somewhere else
#   PYTHON=python3.13 ./install.sh
#   MOTORBRIDGE_WHEEL=dist/motorbridge-...whl ./install.sh   # use a local wheel (offline)
#
# Safe to re-run: it updates the existing venv in place.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$ROOT/.venv"
PYTHON="${PYTHON:-python3.12}"
FORK_REPO="ax2348-gif/motorbridge_2ec"
export PIP_NO_CACHE_DIR=1  # don't keep a second copy of every wheel in ~/.cache/pip

# On Linux aarch64 (the robot), constraints.txt pins CPU-only torch, served from PyTorch's
# index. --extra-index-url (not --index-url): that index also hosts stale copies of common
# packages, so pip must still see PyPI.
EXTRA_INDEX=()
if [[ "$(uname -s)" == Linux && "$(uname -m)" == aarch64 ]]; then
  EXTRA_INDEX=(--extra-index-url https://download.pytorch.org/whl/cpu)
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --venv) VENV="$2"; shift 2 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

step() { printf '\n==> %s\n' "$*"; }

step "Submodules"
# Only fetch submodules that are not checked out yet; never reset one that is (it may have
# local work). To move them to the recorded commits: git submodule update --recursive
for sub in rev_ctl rebot_ctl motorbridge_2ec; do
  if [[ ! -f "$ROOT/$sub/README.md" ]]; then
    [[ -e "$ROOT/.git" ]] || { echo "submodule $sub is missing; clone with --recursive" >&2; exit 1; }
    git -C "$ROOT" submodule update --init --recursive -- "$sub"
  fi
  echo "  $sub @ $(git -C "$ROOT/$sub" describe --always --dirty 2>/dev/null || echo '?')"
done

step "Virtual environment ($VENV)"
command -v "$PYTHON" >/dev/null || { echo "$PYTHON not found (LeRobot needs Python >= 3.12)" >&2; exit 1; }
"$PYTHON" -c 'import sys; sys.exit(sys.version_info < (3, 12))' \
  || { echo "$PYTHON is $("$PYTHON" -V 2>&1); LeRobot needs Python >= 3.12" >&2; exit 1; }
if [[ ! -x "$VENV/bin/python" ]]; then
  # Debian/Ubuntu ship venv without ensurepip unless python3.X-venv is installed.
  "$PYTHON" -c 'import ensurepip' 2>/dev/null || {
    v="$("$PYTHON" -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')"
    echo "$PYTHON cannot create virtual environments. On Ubuntu: sudo apt install python$v-venv" >&2
    exit 1
  }
  "$PYTHON" -m venv "$VENV"
fi
PY="$VENV/bin/python"
"$PY" -m pip install --quiet --upgrade pip
"$PY" -m pip install --quiet packaging  # for wheel selection; no --upgrade (LeRobot bounds it)

step "motorbridge (motorbridge_2ec fork wheel)"
MB_VERSION="$(sed -nE 's/^motorbridge==(.*)$/\1/p' "$ROOT/constraints.txt")"
WHEEL="${MOTORBRIDGE_WHEEL:-}"
if [[ -z "$WHEEL" ]]; then
  # Pick the release asset whose wheel tags match this interpreter and platform.
  WHEEL="$("$PY" - "$FORK_REPO" "$MB_VERSION" <<'PYEOF'
import json, sys, urllib.parse, urllib.request
from packaging.tags import sys_tags
from packaging.utils import parse_wheel_filename

repo, version = sys.argv[1], sys.argv[2]
tag = urllib.parse.quote(f"2ec-{version}", safe="")
url = f"https://api.github.com/repos/{repo}/releases/tags/{tag}"
with urllib.request.urlopen(url, timeout=30) as r:
    assets = json.load(r)["assets"]
ours = list(sys_tags())  # most preferred first
best = None
for a in assets:
    if not a["name"].endswith(".whl"):
        continue
    _, _, _, wheel_tags = parse_wheel_filename(a["name"])
    rank = min((ours.index(t) for t in wheel_tags if t in ours), default=None)
    if rank is not None and (best is None or rank < best[0]):
        best = (rank, a["browser_download_url"])
if best is None:
    sys.exit(f"no motorbridge {version} wheel for this platform in {repo} release 2ec-{version}")
print(best[1])
PYEOF
)"
fi
echo "$WHEEL"
"$PY" -m pip install --quiet -c "$ROOT/constraints.txt" "$WHEEL"

step "rev_ctl, rebot_ctl (editable), robot_utils and LeRobot"
"$PY" -m pip install --quiet -c "$ROOT/constraints.txt" ${EXTRA_INDEX[@]+"${EXTRA_INDEX[@]}"} \
  -e "$ROOT/rev_ctl[slcan]" \
  -e "$ROOT/rebot_ctl" \
  -e "$ROOT"

step "pyorbbecsdk2 (Orbbec camera SDK, without its example dependencies)"
# --no-deps: see constraints.txt. Its only runtime dependency, numpy, is already installed.
"$PY" -m pip install --quiet --no-deps -c "$ROOT/constraints.txt" pyorbbecsdk2
if [[ "$(uname -s)" == Linux && ! -e /etc/udev/rules.d/99-obsensor-libusb.rules ]]; then
  echo "  Orbbec udev rule not installed; the camera will fail to open as a normal user. Run once:"
  echo "    sudo cp $("$PY" -c 'import importlib.util, os; print(os.path.dirname(importlib.util.find_spec("pyorbbecsdk").origin))')/shared/99-obsensor-libusb.rules /etc/udev/rules.d/"
  echo "    sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

step "Check"
# pyorbbecsdk2's example dependencies are skipped on purpose; anything else is a real problem.
problems="$("$PY" -m pip check 2>&1 | grep -v '^pyorbbecsdk2 [0-9.]* requires ' | grep -v '^No broken requirements' || true)"
[[ -z "$problems" ]] || { echo "$problems" >&2; exit 1; }
echo "  pip check: OK"
"$PY" - <<'PYEOF'
import importlib.metadata as md

import lerobot, motorbridge, pyorbbecsdk, reBotArm_control_py, robot_utils, sparkcan, torch  # noqa: F401
from motorbridge.core import Motor

assert "+2ec" in motorbridge.__version__, f"motorbridge {motorbridge.__version__} is not the 2ec fork"
assert hasattr(Motor, "get_cached_register_f32"), "motorbridge lacks the fork's non-blocking reads"
for name in ("robot-utils", "sparkcan", "rebotarm-control-py", "motorbridge", "lerobot", "torch", "pyorbbecsdk2"):
    print(f"  {name:20} {md.version(name)}")
print(f"  motorbridge ABI      {motorbridge.abi_version()}")
PYEOF
printf '\nDone. Activate with: source %s/bin/activate\n' "$VENV"
