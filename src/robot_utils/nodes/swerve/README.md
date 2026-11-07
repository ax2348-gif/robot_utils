# `robot_utils.nodes.swerve`: MAXSwerve drive base library

The swerve-drive code from the `rev_ctl` submodule's `examples/`, ported at **rev_ctl commit
`cfcfc32`** (the pointer robot_utils' `main` records) into an importable package, so a future
HTTP swerve node can own it. Behaviour is unchanged: same constants (geometry, CAN IDs,
steering offsets, gains, current limits), same 10 ms control loop, same 0.25 s command
timeout, same error handling. Only imports, docstrings and file names changed, and the pure
kinematics functions moved into their own module.

Hardware usage (setup, zeroing, bench test, teleop, troubleshooting) is documented in
`rev_ctl/docs/swerve.md`; substitute the `python -m` commands below for the example scripts.

## Module map

| Here | From `rev_ctl` @ cfcfc32 | What |
|---|---|---|
| `constants.py` | `examples/swerve_constants.py` | Geometry, `MODULES` (CAN IDs, steer offsets), gear ratio, gains, limits, `COMMAND_TIMEOUT` |
| `kinematics.py` | `examples/swerve_drive.py` (top half) | Pure functions: `wrap_angle`, `wheel_velocities`, `module_command`, `robot_twist`, `MIN_SPEED` |
| `drive.py` | `examples/swerve_drive.py` | `SwerveDrive` (re-exports the kinematics names); CLI `python -m robot_utils.nodes.swerve.drive <iface> vx vy omega --seconds 2` |
| `setup_sparks.py` | `examples/swerve_setup.py` | SPARK configuration; `python -m robot_utils.nodes.swerve.setup_sparks <iface> [--apply]` |
| `steering_calibration.py` | `examples/steering_calibration.py` | `Steering` (measure / zero / blink) + its own web page; `python -m robot_utils.nodes.swerve.steering_calibration <iface>` |
| `tests/swerve/test_swerve.py` | `tests/test_swerve.py` | All but `test_bench_commands` |
| `tests/swerve/test_steering_calibration.py` | `tests/test_steering_calibration.py` | All |
| `tests/swerve/conftest.py` | `tests/conftest.py` | `iface` fixture only (`wait_for` moved into `test_swerve.py`) |

Tests run against `sparkcan.sim.SimulatedSpark` on a `virtual:` bus, no hardware:

```bash
python -m pytest tests/swerve
```

## Left in rev_ctl

They are clients of `SwerveDrive`, not part of it; a node replaces them.

- `examples/teleop.py`: web joystick page with its own client-claim / enable logic (tests: `tests/test_teleop.py`).
- `examples/quest_teleop.py`: Quest controller stream to `drive()`.
- `examples/bench.py`: interactive bench helper (tests: `test_bench_commands` in `tests/test_swerve.py`).
- `docs/swerve.md`: the hardware procedure documentation.

They still import the flat `examples/` copies, so there are now two copies of the library code.

## Changes since the port (loop timing)

Measured on the robot (wheels off the ground, slow twists), `SwerveDrive`'s loop ran at about
60 Hz, not 100: it slept 10 ms *after* each tick's work, and the work took about 6 ms, nearly
all of it 9 separate serial writes to the CANable (about 0.6 ms each, one USB transfer per
write). Three changes, each measured on the robot:

| | loop rate | period p99 | tick work p50 | frames received/s | CPU |
|---|---|---|---|---|---|
| as ported | 62 Hz | 19.1 ms | 5.9 ms | 2,990 | 24% |
| + fixed-rate schedule | 100 Hz | 10.1 ms | 5.2 ms | 2,990 | 28% |
| + trimmed status frames | 100 Hz | 10.0 ms | 6.5 ms | 1,750 | 22% |
| + one serial write per tick | 100 Hz | 10.0 ms | 1.4 ms | 1,750 | 17% |
| library as committed | 100 Hz | 10.3 ms | 0.7-1.1 ms (write) | 1,690 | 15-16% |

- **Fixed-rate schedule**: the loop sleeps until the next 10 ms deadline; after an overrun it
  restarts from now instead of bursting.
- **Status frames, set at runtime by `SwerveDrive` (not persisted)**: STATUS_0 and the one
  signal frame each SPARK needs (drives STATUS_2, steers STATUS_5) at 10 ms, the other signal
  frame off, STATUS_1 at 250 ms. All status signals then arrive at 100 Hz (p99 about 11 ms;
  rare single late frames), at about 40% bus load with 100 Hz control. setup_sparks still
  stores 20 ms, which applies after a power cycle until the next `SwerveDrive` starts.
- **Batched writes**: frames the loop thread sends during a tick are written in one
  `ser.write` (slcan only; other transports send them one at a time).
- Not adopted: `sys.setswitchinterval(0.001)` made no difference (wake-up lateness was already
  about 0.06 ms).

## Notes for the swerve node

`SwerveDrive` already is the hardware-owner thread: the constructor opens the bus, waits up
to 2 s per module for a steering reading (raises `RuntimeError` otherwise) and starts a daemon
thread named `swerve` that sends setpoints plus the enable heartbeat every 10 ms. The node's
API would wrap:

- `drive(vx, vy, omega)`: robot-frame twist (m/s, m/s, rad/s). Raises `ValueError` on
  non-finite values. Holds for `COMMAND_TIMEOUT` (0.25 s); the client must re-send at least
  that often (teleop sends at 50 Hz). Clamping to sensible limits is the caller's job
  (teleop clamps to 2 m/s and 3 rad/s); `SwerveDrive` only scales wheels to `MAX_WHEEL_SPEED`.
- `drive_wheels(angle, speed)`: bench mode, all wheels to one angle.
- `stop()`: disables immediately (one all-disabled heartbeat, then silence); the next
  `drive()` re-enables. There is no separate enable/disable latch: "disabled" is "no recent
  command". A node-level enable flag (as teleop.py has) must gate `drive()` itself.
- State: `measured_twist()` (or `None` if a steering reading is stale), `encoder_reading(i)`,
  `wheel_angle(reading, i)`, and the raw SPARK objects `drives[i]` / `steers[i]` for currents
  and faults (see `bench.py`'s `status()`).
- Error: `error` holds the exception that killed the control loop; the loop then disables
  the bus and exits. `drive()` keeps accepting commands silently afterwards, so the node must
  check `error` (on every request and/or in a watchdog) and report it. Recovery means
  `close()` and constructing a new `SwerveDrive`.
- `close()`: stops the loop, sends a disabling heartbeat and unregisters the SPARKs. It does
  **not** close the `CANBus` (sparkcan shares one bus per interface name in
  `CANBus._instances`), so the serial port stays open until the process exits or
  `drive.bus.close()` is called.

## Open direction: commanding a twist as direction + magnitude

The node's command will be a plain twist `(vx, vy, omega)` for now. A twist has only 3
degrees of freedom, and it leaves the wheels' steering undefined when the robot is not
moving: at zero twist `module_command` keeps each wheel's current heading, so a client cannot
pre-steer the modules before it starts moving (or hold a heading while stopped).

A more complete command separates the twist into a **direction** and a **magnitude**:

- a unit twist `u = (vx, vy, omega) / |(vx, vy, omega)|` fixes every module's steering angle
  uniquely, up to the sign of each module's drive direction (which the 180° flip in
  `module_command` already resolves by choosing the nearer heading);
- a scalar magnitude `s` (which may be 0) sets the actual twist `s * u`, i.e. the drive
  speeds.

So `(u, s)` is a well-defined command at every speed, including steering while stopped:
`s = 0` points the wheels for the direction `u` without moving the robot. A plain twist is
the special case `u = twist / |twist|`, `s = |twist|`.

Things to settle before implementing it:

- **Units in the norm.** `vx, vy` are m/s and `omega` is rad/s, so "unit" needs a length
  scale, e.g. normalise `(vx, vy, L * omega)` with `L` the wheel radius from the centre
  (about 0.23 m), so that pure translation and pure rotation at the same wheel speed have the
  same magnitude.
- **Transitions.** When `u` changes while `s` is 0, the modules turn in place; when `s > 0`
  the existing cosine scaling (`speed * cos(angle - heading)`) keeps a wheel from driving
  while it is still far from its target angle.
- **Limits.** Hard and soft limits apply to the resulting twist `s * u` (per component).
- **API shape.** For example `{"direction": [ux, uy, uw], "magnitude": s}` beside the plain
  `{"twist": [vx, vy, omega]}` form, both carried by the same command model.

