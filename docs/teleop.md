# Quest teleoperation: `robot_utils.quest` and `robot_utils.teleop`

Two layers, neither of which knows anything about a particular robot:

- **`robot_utils.quest`**: a client for the Quest app's `teleop.quest_state.v1` JSON-lines
  stream. It provides typed and validated frames.
- **`robot_utils.teleop`**: a fixed-rate loop that turns those frames into safe, generic
  inputs and drives a robot adapter through coarse callbacks.

A robot is supported by writing one `TeleopApp` subclass.
[`examples/rebot_quest_teleop.py`](../examples/rebot_quest_teleop.py) is the reBot arm's,
rewritten from rebot_ctl's `tools/quest_teleop.py`.
[`examples/rebot_quest_teleop_meshcat.py`](../examples/rebot_quest_teleop_meshcat.py) is the
same adapter on a simulated arm in MeshCat. It needs no hardware, drives a virtual base
with the twist channel, and draws through a `Monitor`.

## The Quest client

```python
from robot_utils.quest import QuestClient

with QuestClient(host) as quest:           # port 9391; background thread, reconnects with backoff
    frame = quest.latest()                 # newest QuestFrame or None
    quest.age(frame)                       # seconds since it arrived, on this machine's clock
    quest.stats                            # connected, frames, out_of_order, bad_lines, last_error
```

The app serves the stream on every interface, port 9391 (quest_env 447e80f and later), so
connect to the headset's address directly; no relay is needed.

**Compared with the old `QuestStream` in rebot_ctl:**
- `close()` and a context manager;
- frames are parsed once on the reader thread, and the snapshot is lock-protected;
- line framing is bounded;
- `frame_seq` ordering is enforced per connection and session;
- a stats counter;
- validation:
  - the top-level schema is checked;
  - the intent schema is checked; a bad intent leaves `frame.intent = None`;
  - non-finite numbers, short vectors, non-rotations and an out-of-range trigger make that
    hand `valid=False`;
  - rotations are re-orthonormalised.

**Freshness is not decided by the client.** The last frame stays in place when the
connection dies, so compare `received_at` with your clock. `TeleopRunner` does this.

**Generic names for two wire fields:**

| Generic name | Wire field |
|---|---|
| `HandIntent.trigger` | `gripper` |
| `OperatorIntent.twist` | `base_velocity` |

The raw record is always available as `frame.raw`, e.g. for buttons.

## The teleop framework

```python
from robot_utils.teleop import TeleopApp, TeleopRunner, PoseClutch, FaultAction

class MyRobot(TeleopApp):
    rate_hz = 100.0
    def start(self, ctx): ...                 # acquire hardware, leave it holding
    def engage(self, ctx, inputs): ...        # idle -> active (enable, seed commands)
    def active_step(self, ctx, inputs): ...   # follow targets, send commands
    def disengage(self, ctx, inputs): ...     # active -> idle
    def hold_step(self, ctx, inputs): ...     # every idle tick: hold, feed watchdogs
    def stop(self, ctx, result): ...          # always called once: release hardware
    # optional:
    def wants_active(self, ctx, inputs): return inputs.any_engaged
    def on_fault(self, ctx, fault): return FaultAction.STOP
    def emergency_stop(self, reason): ...     # any thread, immediately on an e-stop

with QuestClient(host, port) as quest:
    result = TeleopRunner(MyRobot(), quest).run()   # RunResult: kind, reason, fault, stats
```

### What a callback receives

`Inputs` is built once per tick and is already safe to act on.

| Field | Meaning |
|---|---|
| `live` / `status` | `ok`, or why not: `no-frame`, `stale` (older than `input_timeout_s`, default 0.2 s, on this machine's clock), `unfocused` or `no-intent`. |
| `hands[side]` | A `HandState` per controller, after the robot-side clutch. Its fields are below. |
| `twist` | The base velocity request `[fwd m/s, left m/s, yaw rad/s]`. It is zero unless `live`. |
| `frame` | The raw parsed frame when fresh, otherwise `None`. |

`HandState` fields:
- `status`: `no-input`, `invalid`, `released`, `needs-release` or `engaged`;
- `engaged`, `just_engaged` and `just_released`;
- `engagement`: a local id that changes with every accepted engagement;
- `delta_position` and `delta_rotation`: zero/identity unless engaged;
- `trigger`: `None` unless valid.

`Context` carries:
- `rate_hz`, `period_s` and `tick`;
- `active` and `stats`;
- `estop` and `log`;
- `request_stop(reason)` and `require_release(side)`.

### The clutch: the consumer contract, robot side

A hand drives nothing until it is released and gripped again in each of these cases:
- a gap in fresh input;
- unfocus;
- `valid=False`;
- a session change;
- an operator-frame realignment;
- a `HOLD` fault;
- `require_release()`.

An engagement already in progress is never adopted: its delta was not measured from this
robot's pose.

`PoseClutch` turns an engaged hand into an absolute target:
- It calls `measure()` once per new engagement to get the anchor.
- It returns `(anchor.p + R·dp, (R·dR·Rᵀ)·anchor.R)`, where `R = operator_to_robot`.
- `max_offset_m` and `max_angle_rad` optionally clamp the target.

**Live vs frozen reference frame.** For a robot whose measuring frame moves (a mobile
base), pass `frame=` the frame's world pose on every update:
- **Live** (no `frame`, the `standard` scheme): the target moves with the base.
- **Frozen** (`frame` given): the target is fixed in the world from engagement and
  re-expressed in the current base frame. This is what the Quest's `body-frozen` walking
  scheme expects: walking with the base leaves the arm's world target alone. Check
  `inputs.frame.intent.scheme`; in that scheme `HandIntent.operator_base_*` carries each
  engagement's captured operator frame.

Pose targets are one use. A hand can equally drive a velocity, a camera or nothing at all,
and `trigger` can map to any 0–1 actuator.

### Lifecycle and guarantees

- **One step callback per tick.** `wants_active` is asked only while input is live:
  - `False → True` calls `engage` and then `active_step`;
  - `True → False` (release, input loss) calls `disengage` and then `hold_step`.
- **`stop` runs exactly once** whatever ended the run: a request, a fault, a signal, or a
  failure in `start`. Its `result.kind` is `requested`, `fault` or `estop`. A `stop` that
  raises is reported in `result.stop_error`.
- **Faults.** An exception from a callback goes to `on_fault`:
  - `STOP` ends the run;
  - `HOLD` disengages and forces a re-grip on every hand;
  - `IGNORE` continues.

  The rest of the faulting tick is skipped. Faults in `start` or `disengage` always stop,
  and so does `max_consecutive_faults` (default 3) faults in a row. A bad return from
  `on_fault`, or an exception in it, also stops.
- **Timing.**
  - Deadlines advance by one period, so the loop does not drift.
  - Late ticks are counted in `stats.overruns` / `max_late_s`.
  - A tick later than `overrun_fault_s` (default 50 ms) is a `timing` fault (`Overrun`).
  - After falling a full period behind, the schedule restarts instead of bursting.
- **E-stop.** Signal handling is on with `install_signals=True`, from the main thread
  only. It covers SIGINT, SIGTERM, SIGHUP, SIGQUIT and SIGTSTP.
  - The signal reaches a watcher thread through a wakeup pipe. The watcher trips the stop
    and calls `emergency_stop` at once, even while the main thread is blocked in a native
    call.
  - The loop wakes and starts no further step. It waits up to 1 s for
    `emergency_stop` to return, then calls `stop`.
  - A second signal, or a `stop` still running after `estop_grace_s` (default 3 s),
    calls `os._exit` (code 130 or 1).
  - `ctx.estop.trip(reason)` does the same from any thread.

### Keep the steps fast: `Monitor`

**Steps should only read, compute and command.** Anything that can block (MeshCat, a
terminal, network, disk) goes through `Monitor(fn, rate_hz)`. A step calls
`monitor.publish(snapshot)`, which never blocks. A worker thread calls `fn` with the
newest snapshot at most `rate_hz` times a second. A failing `fn` is logged and then
disabled, and never affects the loop. Publish copies, and give the renderer its own model
data. An overrun fault is a real signal that a step blocked; don't ignore it.

**Clutch threshold.** The Quest app clutches, not this package: grip ≥ 0.6 engages and
≤ 0.4 releases. The robot side only enforces freshness and release-before-engage.

### Assumed safety model

**The hardware must stop on its own when commands stop arriving**, e.g. a motor-side CAN
watchdog. After an e-stop the loop sends nothing more, so that watchdog completes the
stop. `emergency_stop` and `stop` can add direct actions on top. Run the loop several
times faster than the watchdog timeout.

**The framework does not replace hardware limits.** Per-robot limits stay in the adapter
and the driver: joint and setpoint limits, slew rates, workspace bounds.

## Tests

```bash
PYTHONPATH=src:tests python -m unittest discover -s tests
```

The tests cover:
- message validation;
- the client against a local TCP server: ordering, partial lines, reconnects, close;
- the clutch contract and the pose helper;
- the runner: lifecycle, stale/unfocused input, input loss mid-engagement, every fault
  path, overruns, a cross-thread e-stop, KeyboardInterrupt and rate;
- real signals in a subprocess: a clean SIGINT, SIGTERM while the main thread is blocked,
  a hung `stop` hitting the grace exit, and a second signal hard-exiting.
