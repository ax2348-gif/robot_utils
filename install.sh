#!/usr/bin/env bash
# Set up the robot's Python environment. Nothing is compiled: motorbridge comes from the
# motorbridge_2ec fork's prebuilt wheels, everything else from PyPI wheels or pure Python.
#
#   ./install.sh                 # creates ./.venv with python3.12
#   ./install.sh --venv ~/robot  # somewhere else
#   PYTHON=python3.13 ./install.sh
#   MOTORBRIDGE_WHEEL=dist/motorbridge-...whl ./install.sh   # use a local wheel (offline)
#   ./install.sh --no-service    # don't install/restart the landing page and port-80 proxy services
#
# Safe to re-run: it updates the existing venv in place, and on Linux (re)installs and
# restarts the landing page and the port-80 proxy as systemd user services.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$ROOT/.venv"
PYTHON="${PYTHON:-python3.12}"
FORK_REPO="ax2348-gif/motorbridge_2ec"
export PIP_NO_CACHE_DIR=1  # don't keep a second copy of every wheel in ~/.cache/pip
SERVICE=1                 # install the landing page as a systemd user service (Linux)

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
    --no-service) SERVICE=0; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
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
PORTS=/proc/sys/net/ipv4/ip_unprivileged_port_start
if [[ "$(uname -s)" == Linux && -r $PORTS && "$(cat $PORTS)" -gt 80 ]]; then
  echo "  Port 80 needs root; the frontend (python -m robot_utils.frontend) serves on it. Run once:"
  echo "    sudo $ROOT/tools/enable_low_ports.sh"
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
step "Landing page and port-80 proxy (systemd user units robot-frontend, robot-proxy)"
# Like the robot's other user services (e.g. ups-dashboard): user units, no root needed, started
# at boot when the user has lingering enabled. Re-running install.sh rewrites and restarts them.
#   robot-proxy.service    nginx on port 80, the only network-facing process: / -> the landing
#                          page, /nodes/<name>/ -> that node instance's unix socket (docs/nodes.md)
#   robot-frontend.service the landing page, on its unix socket frontend.sock
# Without a usable nginx the landing page listens on port 80 itself (--port 80) meanwhile.
FRONTEND=robot-frontend.service
PROXY=robot-proxy.service
if [[ $SERVICE == 0 ]]; then
  echo "  skipped (--no-service)"
elif [[ "$(uname -s)" != Linux ]] || ! command -v systemctl >/dev/null \
     || ! systemctl --user show-environment >/dev/null 2>&1; then
  echo "  skipped: no systemd user session here; run it by hand: python -m robot_utils.frontend --port 8080"
else
  UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
  NGINX_CONF_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/robot_utils/nginx"
  NGINX_STATE="${XDG_STATE_HOME:-$HOME/.local/state}/robot_utils/nginx"
  mkdir -p "$UNIT_DIR"

  NGINX_BIN="$(command -v nginx || true)"
  [[ -z "$NGINX_BIN" && -x /usr/sbin/nginx ]] && NGINX_BIN=/usr/sbin/nginx
  PROXY_OK=1
  if [[ -z "$NGINX_BIN" ]]; then
    PROXY_OK=0
    echo "  nginx is not installed; the landing page serves port 80 itself until it is. Run once:"
    echo "    sudo systemctl mask nginx.service      # the system nginx must never take port 80"
    echo "    sudo apt install -y nginx"
    echo "  then re-run ./install.sh"
  elif [[ "$(systemctl is-enabled nginx.service 2>/dev/null)" =~ ^(enabled|enabled-runtime|static|alias)$ ]] \
       || systemctl is-active --quiet nginx.service 2>/dev/null; then
    PROXY_OK=0
    echo "  The system nginx.service is enabled or running and would hold port 80. Run once:"
    echo "    sudo systemctl disable --now nginx.service && sudo systemctl mask nginx.service"
    echo "  then re-run ./install.sh (the landing page serves port 80 itself meanwhile)"
  fi

  FRONTEND_ARGS=""
  [[ $PROXY_OK == 1 ]] || FRONTEND_ARGS=" --port 80"
  cat > "$UNIT_DIR/$FRONTEND" <<UNITEOF
# Written by $ROOT/install.sh; re-run it to update. Logs: journalctl --user -u $FRONTEND
[Unit]
Description=robot_utils landing page (python -m robot_utils.frontend; behind $PROXY on port 80)

[Service]
WorkingDirectory=$ROOT
ExecStart=$PY -m robot_utils.frontend$FRONTEND_ARGS
Environment=PYTHONUNBUFFERED=1
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
UNITEOF

  # Stop the proxy before restarting the frontend: an old frontend on port 80 must let go of it
  # before nginx binds, and a frontend on --port 80 must not race a running nginx.
  systemctl --user stop "$PROXY" 2>/dev/null || true
  systemctl --user daemon-reload
  systemctl --user enable --quiet "$FRONTEND"
  systemctl --user restart "$FRONTEND"
  sleep 3
  if systemctl --user is-active --quiet "$FRONTEND"; then
    echo "  $FRONTEND: active; logs: journalctl --user -u $FRONTEND"
  else
    echo "  $FRONTEND did not start; see: journalctl --user -u $FRONTEND -n 30" >&2
  fi

  # nodes with boot: true in nodes.yaml (e.g. power): persistent units, started at boot
  (cd "$ROOT" && "$PY" -m robot_utils.frontend.boot | sed 's/^/  boot: /') \
    || echo "  some boot nodes did not start (above)" >&2

  if [[ $PROXY_OK == 1 ]]; then
    "$PY" -m robot_utils.frontend.proxy --conf-dir "$NGINX_CONF_DIR" --state-dir "$NGINX_STATE" >/dev/null
    NGINX_ARGS="-p $NGINX_STATE -e $NGINX_STATE/error.log -c $NGINX_CONF_DIR/nginx.conf"
    if ! "$NGINX_BIN" $NGINX_ARGS -t -q; then
      echo "  nginx rejected $NGINX_CONF_DIR/nginx.conf (above); $PROXY not started" >&2
    else
      cat > "$UNIT_DIR/$PROXY" <<UNITEOF
# Written by $ROOT/install.sh; re-run it to update. Logs: journalctl --user -u $PROXY,
# $NGINX_STATE/error.log. Config: $NGINX_CONF_DIR/nginx.conf (generated; docs/nodes.md)
[Unit]
Description=robot_utils port-80 proxy (nginx: / -> landing page, /nodes/<name>/ -> node sockets)
After=$FRONTEND

[Service]
ExecStart=$NGINX_BIN $NGINX_ARGS -g 'daemon off;'
ExecReload=$NGINX_BIN $NGINX_ARGS -s reload
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
UNITEOF
      systemctl --user daemon-reload
      systemctl --user enable --quiet "$PROXY"
      systemctl --user restart "$PROXY"
      sleep 2
      if systemctl --user is-active --quiet "$PROXY"; then
        echo "  $PROXY: active (http://$(hostname)/); logs: journalctl --user -u $PROXY"
      else
        echo "  $PROXY did not start; see: journalctl --user -u $PROXY -n 30" >&2
      fi
    fi
  else
    systemctl --user disable --quiet "$PROXY" 2>/dev/null || true
    echo "  $FRONTEND serves http://$(hostname)/ directly until nginx is set up (above)"
  fi
  if [[ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != yes ]]; then
    echo "  They start at boot only with lingering enabled for $USER. Run once:"
    echo "    sudo loginctl enable-linger $USER"
  fi
fi

printf '\nDone. Activate with: source %s/bin/activate\n' "$VENV"
