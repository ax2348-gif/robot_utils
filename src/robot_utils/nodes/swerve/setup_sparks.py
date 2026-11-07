#!/usr/bin/env python3
"""Configure the swerve SPARKs: steering for position control, drive for velocity in m/s.

    python -m robot_utils.nodes.swerve.setup_sparks slcan:/dev/ttyACM0          # show current vs. intended
    python -m robot_utils.nodes.swerve.setup_sparks slcan:/dev/ttyACM0 --apply  # write and save

Settings follow REV's MAXSwerve template, in rotations and m/s, softened for first tests
(see constants.py). Run it before zeroing with steering_calibration: inverting the
steering encoder changes its readings.

Writes parameters only (saved to flash with --apply); never sends a heartbeat or a setpoint.

Ported from rev_ctl ``examples/swerve_setup.py`` (commit cfcfc32).
"""
from __future__ import annotations

import argparse
import math

from sparkcan import (FeedbackSensor, IdleMode, PersistMode, ResetMode, REVLibError, SparkFlex,
                      SparkFlexConfig)

from . import constants as c


def common_config(current_limit: int) -> SparkFlexConfig:
    config = SparkFlexConfig()
    config.smart_current_limit(current_limit)
    config.secondary_current_limit(c.SECONDARY_CURRENT_LIMIT)
    config.voltage_compensation(c.NOMINAL_VOLTAGE)
    config.set_idle_mode(IdleMode.COAST)   # wheels and modules can be moved by hand
    return config


def steering_config() -> SparkFlexConfig:
    config = common_config(c.STEER_CURRENT_LIMIT)
    config.absolute_encoder.inverted(True)        # the module's output turns opposite to the motor
    # Average 1 sample, not the default 128 (~65 ms of lag): the position loop closes on this
    # reading, and the lag made steering overshoot and chatter after abrupt changes.
    config.absolute_encoder.average_depth(1)
    loop = config.closed_loop
    loop.set_feedback_sensor(FeedbackSensor.ABSOLUTE_ENCODER)
    loop.p(c.STEER_P).i(0).d(0)
    loop.output_range(-c.STEER_MAX_OUTPUT, c.STEER_MAX_OUTPUT)
    loop.position_wrapping_enabled(True)          # 0.95 -> 0.05 goes through 0, not the long way
    loop.position_wrapping_input_range(0, 1)
    # Encoder frames every 20 ms instead of 200, so the drive can follow the real wheel angle.
    config.signals.absolute_encoder_position_period_ms(20)
    config.signals.duty_cycle_period_ms(20)
    return config


def drive_config() -> SparkFlexConfig:
    config = common_config(c.DRIVE_CURRENT_LIMIT)
    config.open_loop_ramp_rate(c.DRIVE_RAMP)
    config.closed_loop_ramp_rate(c.DRIVE_RAMP)
    config.encoder.position_conversion_factor(c.DRIVE_MPS_PER_RPM * 60)   # m per motor rotation
    config.encoder.velocity_conversion_factor(c.DRIVE_MPS_PER_RPM)        # m/s per motor RPM
    loop = config.closed_loop
    loop.set_feedback_sensor(FeedbackSensor.PRIMARY_ENCODER)
    loop.p(c.DRIVE_P).i(0).d(0)
    loop.feed_forward.v(c.DRIVE_KV)
    loop.output_range(-c.DRIVE_MAX_OUTPUT, c.DRIVE_MAX_OUTPUT)
    return config


def targets() -> list:
    """(label, CAN ID, config) for every SPARK."""
    steering, drive = steering_config(), drive_config()
    return ([(f"{m.name} steering", m.steer_id, steering) for m in c.MODULES]
            + [(f"{m.name} drive", m.drive_id, drive) for m in c.MODULES])


def matches(current, wanted) -> bool:
    if isinstance(wanted, float) and current is not None:
        return math.isclose(current, wanted, rel_tol=1e-6, abs_tol=1e-6)
    return current == wanted


def main() -> None:
    ap = argparse.ArgumentParser(description="Configure the swerve SPARKs")
    ap.add_argument("interface", help='e.g. can0 or "slcan:/dev/ttyACM0"')
    ap.add_argument("--apply", action="store_true", help="write the settings and save them to flash")
    args = ap.parse_args()

    for label, can_id, config in targets():
        spark = SparkFlex(args.interface, can_id)
        try:
            print(f"{label} (CAN ID {can_id}):")
            for param, value in config.flatten():
                current = spark.get_parameter(param)
                mark = "" if matches(current, value) else "  <- differs"
                print(f"  {param.name:32} now {current!s:12} want {value}{mark}")
            if args.apply:
                err = spark.configure(config, ResetMode.NO_RESET_SAFE_PARAMETERS, PersistMode.PERSIST_PARAMETERS)
                print("  applied and saved" if err == REVLibError.OK else f"  FAILED: {err}")
        finally:
            spark.close()
    if not args.apply:
        print("\nnothing written; run with --apply to write and save these settings")


if __name__ == "__main__":
    main()
