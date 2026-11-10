# Swerve node

`python -m robot_utils.nodes.swerve` runs the MAXSwerve base's 100 Hz control loop and serves it
over HTTP: a twist command, a lease so only one client controls the base, an e-stop, and every
control tick as a binary telemetry stream. Shared conventions (time, framing, lease, e-stop,
`/meta`, the browser toolkit): [nodes.md](nodes.md).

- Page: `http://<robot>/nodes/swerve/` (lease, enable, jog pad with W A S D / Q E, e-stop, live plot)
- Interactive API docs: `http://<robot>/nodes/swerve/docs`
- `<pi>` = `botcore-desktop`, `192.168.8.209` (LAN) or `100.80.153.120` (Tailscale)

## Quick start

On the robot (SPARKs configured once with `python -m robot_utils.nodes.swerve.setup_sparks`):

```bash
cd ~/robot_utils && .venv/bin/python -m robot_utils.nodes.swerve      # instance 'swerve': /nodes/swerve/ (--port 8102 to debug)
```

It finds the CANable under `/dev/serial/by-id` (or `$CAN`, or `--interface slcan:/dev/ttyACM0`).
Without hardware, `--sim` runs the same node against 8 simulated SPARKs on a virtual bus.

From any machine:

```python
from robot_utils.nodes.swerve.client import SwerveClient

with SwerveClient("http://botcore/nodes/swerve") as base:   # on the robot: SwerveClient("swerve")
    base.sync()                                    # clock sync for the stream's timestamps
    with base.control("my-experiment"):            # lease + enable; disable + release on exit
        base.set_limits(max_speed=0.3, max_omega=1.0)
        base.drive_for((0.2, 0.0, 0.0), seconds=1.0)   # resends every 50 ms, then zero
    for header, rows in base.rows():               # every 100 Hz tick
        print(rows["t_tick"][-1], rows["FL_drive_vel"][-1])
        break
```

## Control

| request | who | what |
|---|---|---|
| `POST /lease {"client": "name"}` | anyone | → `{"token", "lease_id", "heartbeat": 1.0}`; 409 if held |
| `POST /lease/renew` | holder | keeps the lease (any accepted command also does) |
| `DELETE /lease` | holder | release: stops, disables, resets soft limits |
| `POST /enable`, `POST /disable` | holder | commands only move the base while enabled |
| `POST /command {"twist": [vx, vy, omega], "seq": 0}` | holder | robot frame: m/s forward, m/s left, rad/s counter-clockwise |
| `GET /limits`, `POST /limits {"max_speed", "max_omega"}` | anyone / holder | hard limits 2 m/s, 4 rad/s (fixed); soft limits within them, per lease |
| `POST /estop` (no body needed) | **anyone** | stops at once, latches; `{"latched": true}` |
| `POST /estop/clear` | holder | unlatch; stays disabled until `POST /enable` |
| `POST /reset` | holder | rebuild the drive after a loop error (ends disabled) |
| `GET /encoders`, `POST /encoders/rebase`, `GET`/`POST /encoders/auto` | anyone | drive-encoder rebase (below) |

Holder requests carry the lease token as the `X-Lease-Token` header.

**Timing.** A command holds for **0.25 s**: resend at least that often while moving (the page and
`drive_for` send at 20 Hz). The lease lapses after **1 s** without a renew or a command; a lapse
stops and disables the base like a release. Both are checked on the control thread every tick.
E-stop, lease lapse, command timeout and the SPARKs' own heartbeat watchdog are independent
stops.

**Clamping.** Translation is scaled down to `max_speed` keeping its direction; `omega` is clipped
to `max_omega`. The response says what was applied:
`{"seq": 0, "applied": [0.06, 0.08, 0.5], "clamped": true, "soft_limits": {...}}`.

**`seq`** is optional and only for debugging: it is echoed in the stream (`cmd_seq`) so a
recording shows which command each tick applied. Leave it 0 otherwise; it is never used for
ordering or rejection. Commands are latest-wins: if two arrive within one tick, only the newer
is applied (seen as a `seq` that never appears in the stream).

### curl

```bash
N=http://botcore/nodes/swerve
T=$(curl -s -X POST $N/lease -H 'content-type: application/json' -d '{"client":"curl"}' | python3 -c 'import json,sys;print(json.load(sys.stdin)["token"])')
curl -s -X POST $N/enable -H "X-Lease-Token: $T"
for i in $(seq 20); do curl -s -X POST $N/command -H "X-Lease-Token: $T" -H 'content-type: application/json' \
  -d '{"twist":[0.1,0,0]}' > /dev/null; sleep 0.05; done      # 1 s forward at 0.1 m/s
curl -s -X DELETE $N/lease -H "X-Lease-Token: $T"
curl -s -X POST $N/estop                                      # anyone, any time
```

### JavaScript (from another page or a dashboard; CORS is open)

```js
const N = "http://botcore/nodes/swerve/";
const {token} = await (await fetch(N + "lease", {method: "POST", headers: {"content-type": "application/json"},
                                                  body: JSON.stringify({client: "my-page"})})).json();
const H = {"content-type": "application/json", "X-Lease-Token": token};
await fetch(N + "enable", {method: "POST", headers: H});
const timer = setInterval(() => fetch(N + "command", {method: "POST", headers: H,
                                     body: JSON.stringify({twist: [0, 0, 0.3]})}), 50);
setTimeout(async () => { clearInterval(timer); await fetch(N + "lease", {method: "DELETE", headers: H}); }, 2000);
```

On the node's own page, `common/ui.js` provides all of this (`RU.leaseWidget`, `RU.api`,
`RU.rows`); see [nodes.md](nodes.md#browser-toolkit-commonuijs).

## Telemetry stream

`GET /stream[?batch_ms=0&max_backlog_s=20]`: one `schema` record (kind `"tick"`, 77 fields, units),
then a `rows` record per control tick (float64). `GET /fields` lists the columns with units and
notes. All times are the Pi's `time.monotonic()` seconds.

| columns | meaning |
|---|---|
| `t_tick`, `tick` | start of the control tick; tick counter (consecutive: gaps = lost rows) |
| `motors_enabled`, `enabled`, `estop`, `lease_id`, `loop_ok` | flags |
| `enc_epoch`, `enc_rebasing`, `rebase_age` | drive-encoder rebases confirmed since start; 1 while one is in flight (`drive_pos` may jump); s since the last (below) |
| `cmd_seq`, `cmd_source`, `cmd_age` | the command in force: its seq, writer (1 slow, 2 fast), age |
| `req_vx`, `req_vy`, `req_omega` | twist as requested |
| `cmd_vx`, `cmd_vy`, `cmd_omega` | twist after the soft-limit clamp, as sent (NaN when stopped) |
| `soft_max_speed`, `soft_max_omega` | soft limits in force |
| `<M>_steer_angle`, `<M>_steer_vel` | wheel angle (rad, 0 = forward, CCW +) and its rate, from the absolute encoder (STATUS_5) |
| `<M>_drive_vel`, `<M>_drive_pos` | wheel speed (m/s) and travel since the SPARK booted or the last rebase (m; float32 on the wire) (STATUS_2) |
| `<M>_steer_sp`, `<M>_drive_sp` | setpoints sent this tick, after kinematics and the 180° flip (NaN if none) |
| `<M>_steer_out`, `<M>_steer_current`, `<M>_drive_out`, `<M>_drive_current` | applied duty cycle and current (STATUS_0) |
| `<M>_t_steer`, `<M>_t_drive`, `<M>_t_steer_s0`, `<M>_t_drive_s0` | arrival time of each status frame (per-signal sample time) |

`<M>` is `FL`, `FR`, `RR`, `RL`. Status frames come every 10 ms, so a signal's own timestamp can
lag `t_tick` by up to about 10 ms; use it, not `t_tick`, when the exact sample time matters.
Odometry and other estimates are not computed here (an estimator node's job).

Record by saving the stream (`base.record("/stream", "run.bin", seconds=60)`) and read it with
`common.protocol.iter_rows(iter_records(open("run.bin", "rb")))`.

## Drive-encoder rebase

**Why.** `drive_pos` is a float32 on the CAN wire, so its resolution falls as travel accumulates:
about 0.06 mm at 1 km and 1 mm at 10 km of wheel travel (the estimator integrates its per-tick
differences). Setting the four drive encoders back to 0 now and then keeps it fine. The drive
SPARKs run velocity closed loop, so redefining their position changes nothing in control; the
steering (absolute encoder) is never touched.

**How.** A rebase is only done while the base is **stationary**: every |`drive_vel`| below
0.01 m/s from fresh STATUS_2 frames, and no non-zero command in force (stopped, timed out,
disabled, or enabled and holding zero are all fine). The control thread sends
`SET_PRIMARY_ENCODER_POSITION 0` to the four drives with the tick's other frames (one batched
write), then waits for STATUS_2 frames that arrive at least 15 ms after the send and read
|`drive_pos`| ≤ 1 mm on all four. Only then does `enc_epoch` increment (about 20-30 ms in the
simulator). Not confirmed within 0.5 s → the rebase failed (`last_error`, a warning event; the
epoch stays). While a rebase is in flight `enc_rebasing` = 1 and a fresh command is **held at
zero** (it goes through once the rebase ends), so the base never moves while `drive_pos` jumps.

| request | who | what |
|---|---|---|
| `GET /encoders` | anyone | `{epoch, last_rebase_t, last_rebase_age, rebasing, last_error, auto, max_abs_drive_pos, stationary_s, motion}` (times: Pi monotonic s); also `encoders` in `/state` |
| `POST /encoders/rebase` | anyone | rebase now; needs the base stationary for 0.5 s, else **409** with the reason (`motion`); → `/encoders` with the new epoch; 503 if not confirmed |
| `GET /encoders/auto` | anyone | `{enabled, interval_s, still_s}` |
| `POST /encoders/auto {"enabled", "interval_s", "still_s"}` | anyone | change any of them (runtime only: the defaults return when the node restarts) |

Anyone may rebase (no lease): it moves nothing, and the estimator re-references across it (a
rebase is invisible in the estimator's pose, frame and history; docs/estimator_node.md).

**Automatic (default on).** When the base has been stationary for `still_s` (default 1 s) and
the last rebase is older than `interval_s` (default 600 s), the node rebases by itself; with no
rebase yet, that is once at start as soon as the base is still. A failed automatic rebase is
retried after 10 s at the earliest.

Not confirmed on hardware yet: that a SPARK applies `SET_PRIMARY_ENCODER_POSITION` while
disabled and that the next STATUS_2 already carries the new position (the 15 ms settle and the
0.5 s timeout allow for some delay; a failure is reported, never silently counted).

## Measured latency

`examples/swerve_node_profile.py` on the robot (wheels off the ground, 0.1 m/s, 15 s), node at
commit b3c256c; JSON in [swerve_node_profile/](swerve_node_profile/). Clocks synced with
`/time`; "applied" = first tick whose row carries the command's `seq`.

| | sync ± | loop | tick → client p50 / p95 / p99 | command RTT p50 / p99 | command → applied p50 / p99 |
|---|---|---|---|---|---|
| Pi localhost | 0.5 ms | 100.0 Hz | 1.7 / 2.8 / 4.1 ms | 3.2 / 5.1 ms | 4.5 / 10.3 ms |
| Mac → Pi, LAN (Mac on Wi-Fi) | 2.1 ms | 100.0 Hz | 3.9 / 49 / 70 ms | 8.2 / 77 ms | 7.0 / 60 ms |
| Mac → Pi, Tailscale (direct) | 2.4 ms | 100.0 Hz | 4.5 / 49 / 70 ms | 8.6 / 76 ms | 7.4 / 67 ms |

- The control loop holds 100 Hz with period p99 about 10.5 ms, no dropped rows, while streaming.
- Command → applied is at most one tick (10 ms) plus the request time: the tick reads the
  command slot at its start.
- The tails off the robot are the Mac's Wi-Fi, not the node (the Pi-local numbers stay under
  5 ms at p99).
- CPU on the Pi 5: the node process uses about 15% of one core idle (control loop, telemetry,
  CAN receive thread) and 18% with one client streaming every tick.

## Notes

- The node sets the SPARKs' status frames at runtime when it starts (STATUS_0 and STATUS_2 or
  STATUS_5 every 10 ms, STATUS_1 every 250 ms); `setup_sparks` stores 20 ms for after a power cycle.
- A module whose steering reading goes stale (older than 0.1 s) is left out of the heartbeat
  (disabled) for that tick; its drive gets 0.
- After a control-loop error the node reports `state: "error"` with the exception; `POST /reset`
  rebuilds the drive.
- Not yet: the streaming fast path for commands (plug-in point: [nodes.md](nodes.md#command-paths-actuated-nodes)),
  and the direction + magnitude command (open direction in `src/robot_utils/nodes/swerve/README.md`).
