# Real-time control loops

The loops that talk to motors or feed them run with real-time scheduling (SCHED_FIFO), so that
other work on the robot's computer cannot hold them up:

| node | thread | priority |
|---|---|---|
| arm | `arm-control` | 40 |
| swerve | `swerve` | 40 |
| estimator | `fusion` | 30 |
| pose_servo | `control` | 20 |

Why: the arm's motors stop if no command reaches them for 100 ms (their own CAN watchdog). On
2026-11-13 a full-resolution `/look` in another node held the arm loop for 105 ms, and every
motor faulted with "communication lost (CAN watchdog)" although the bus and cables were fine.

The code is `nodes/common/realtime.py`. Each thread asks for its priority when it starts. If
Linux refuses, the loop runs as before and the node says so: an event in its log, and `realtime`
in `GET /state` (`policy`, `priority`, `requested`, `error`).

## The one-time setup (root)

A user may not use real-time priorities by default (`ulimit -r` prints 0). The nodes run under
the user's systemd manager, which gets its limits from PAM (`pam_limits`), so allow the user
there and restart the manager:

```bash
echo "$USER - rtprio 50" | sudo tee /etc/security/limits.d/90-robot-rtprio.conf && sudo systemctl restart "user@$(id -u).service"
```

`LimitRTPRIO=` in a `user@.service` drop-in is not enough: systemd applies it, then `pam_limits`
(in `/usr/lib/pam.d/systemd-user`) sets the limit back to 0 before the manager starts (seen on
the robot, 2026-11-13). Check what the manager really has with
`grep -i realtime /proc/$(pgrep -u $USER -f "systemd --user" | head -1)/limits`.

The restart stops every node (a reboot does the same); start them again from the landing page.
Do it with the robot disabled and the arm supported. Afterwards each of the four nodes reports
`"policy": "SCHED_FIFO"`.

## Limits

- It only orders threads: a loop still waits for Python's interpreter lock when an HTTP handler
  of the same node holds it in one long C call. If late ticks remain, that is the cause.
- `ROBOT_UTILS_RT=0` in a node's environment switches it off.
- The kernel keeps 5 % of each second for normal threads (`sched_rt_runtime_us`), so a loop that
  spins cannot lock the machine up.
