#!/bin/bash
# Install the robot remote-access proxy on erid (run on erid as root: sudo ./install.sh).
# nginx (installed if missing) with robot-remote.conf in place of the default site, and robot-whois.service
# (Tailscale identity for the robot's who's-connected view, run as the invoking user). Checks, then reloads.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
RUN_AS=${SUDO_USER:-$(id -un)}
command -v nginx >/dev/null || { apt-get update && apt-get install -y nginx; }
install -d /usr/local/lib/robot-remote
install -m 755 "$HERE/robot-whois.py" /usr/local/lib/robot-remote/robot-whois.py
sed "s/@USER@/$RUN_AS/" "$HERE/robot-whois.service" > /etc/systemd/system/robot-whois.service
systemctl daemon-reload
systemctl enable --now robot-whois.service
systemctl restart robot-whois.service
install -m 644 "$HERE/robot-remote-proxy.inc" /etc/nginx/robot-remote-proxy.inc
install -m 644 "$HERE/robot-remote.conf" /etc/nginx/sites-available/robot-remote.conf
ln -sf /etc/nginx/sites-available/robot-remote.conf /etc/nginx/sites-enabled/robot-remote.conf
rm -f /etc/nginx/sites-enabled/default
nginx -t
systemctl enable nginx
systemctl reload-or-restart nginx
echo "robot remote proxy: http://$(tailscale status --json 2>/dev/null | python3 -c 'import json,sys;print(json.load(sys.stdin)["Self"]["DNSName"].rstrip("."))' 2>/dev/null || hostname)/"
