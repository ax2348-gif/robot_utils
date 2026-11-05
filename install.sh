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
"$PY" -m pip install --quiet --upgrade pip packaging

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
"$PY" -m pip install --quiet -c "$ROOT/constraints.txt" \
  -e "$ROOT/rev_ctl[slcan]" \
  -e "$ROOT/rebot_ctl" \
  -e "$ROOT"

step "Check"
"$PY" -m pip check
"$PY" - <<'PYEOF'
import importlib.metadata as md

import lerobot, motorbridge, reBotArm_control_py, robot_utils, sparkcan, torch  # noqa: F401
from motorbridge.core import Motor

assert "+2ec" in motorbridge.__version__, f"motorbridge {motorbridge.__version__} is not the 2ec fork"
assert hasattr(Motor, "get_cached_register_f32"), "motorbridge lacks the fork's non-blocking reads"
for name in ("robot-utils", "sparkcan", "rebotarm-control-py", "motorbridge", "lerobot", "torch"):
    print(f"  {name:20} {md.version(name)}")
print(f"  motorbridge ABI      {motorbridge.abi_version()}")
PYEOF
printf '\nDone. Activate with: source %s/bin/activate\n' "$VENV"
