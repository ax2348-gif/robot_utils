#!/usr/bin/env bash
# Let normal users listen on ports >= 80, so the robot frontend can serve http://<robot>/
# without root. Run once per machine (Linux only):
#
#   sudo tools/enable_low_ports.sh
#
# Sets net.ipv4.ip_unprivileged_port_start=80 now (sysctl -w) and persists it in
# /etc/sysctl.d/60-robot-utils-ports.conf. Safe to re-run. Despite the "ipv4" in its name,
# the setting covers IPv6 too. Ports 1-79 (ssh is 22) stay privileged.
set -euo pipefail

PORT=80
KEY=net.ipv4.ip_unprivileged_port_start
CONF=/etc/sysctl.d/60-robot-utils-ports.conf
PROC=/proc/sys/net/ipv4/ip_unprivileged_port_start

[[ "$(uname -s)" == Linux ]] || { echo "Linux only (this is $(uname -s))." >&2; exit 1; }
[[ -e "$PROC" ]] || { echo "This kernel has no $KEY (needs Linux >= 4.11)." >&2; exit 1; }
if [[ "$(id -u)" -ne 0 ]]; then
  echo "Needs root, to change a kernel setting. Run: sudo $0" >&2
  exit 1
fi

before="$(cat "$PROC")"
if (( before > PORT )); then
  sysctl -q -w "$KEY=$PORT"
  echo "Set $KEY: $before -> $PORT (now)."
else
  echo "$KEY is already $before (<= $PORT); left as is."
fi

line="$KEY = $PORT"
if [[ -f "$CONF" ]] && grep -qxF "$line" "$CONF"; then
  echo "$CONF already persists it."
else
  printf '# Written by robot_utils tools/enable_low_ports.sh: lets the robot frontend use port 80\n%s\n' \
    "$line" > "$CONF"
  echo "Wrote $CONF (persists across reboots)."
fi
echo "Normal users can now listen on ports >= $(cat "$PROC")."
