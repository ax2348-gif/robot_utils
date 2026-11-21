#!/bin/sh
# One-time install (needs sudo) of the power node's CPU-governor helper:
#   sudo sh ~/robot_utils/deploy/power/install.sh
# Installs /usr/local/sbin/robot-cpu-governor, the robot-cpu-governor@.service template, and a polkit rule that
# lets botcore start only that unit. Undo: sudo sh install.sh --remove
set -eu
D=$(cd "$(dirname "$0")" && pwd)
if [ "${1:-}" = "--remove" ]; then
    rm -f /usr/local/sbin/robot-cpu-governor /etc/systemd/system/robot-cpu-governor@.service \
          /etc/polkit-1/rules.d/51-botcore-cpu-governor.rules
    systemctl daemon-reload
    echo "removed"; exit 0
fi
install -m 0755 -o root -g root "$D/robot-cpu-governor" /usr/local/sbin/robot-cpu-governor
install -m 0644 -o root -g root "$D/robot-cpu-governor@.service" /etc/systemd/system/robot-cpu-governor@.service
install -m 0644 -o root -g root "$D/51-botcore-cpu-governor.rules" /etc/polkit-1/rules.d/51-botcore-cpu-governor.rules
systemctl daemon-reload
echo "installed; test as botcore (no sudo): systemctl start robot-cpu-governor@schedutil.service && cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
