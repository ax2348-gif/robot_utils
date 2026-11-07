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
