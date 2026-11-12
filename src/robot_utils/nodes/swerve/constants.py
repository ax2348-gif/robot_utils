"""Swerve drive base: every robot-specific number, in one place.

Frame: robot x forward, y left, angles counter-clockwise from x (seen from above).
Modules are REV MAXSwerve: SPARK Flex + NEO Vortex drive, SPARK Flex + Flex Dock + NEO 550
steering, REV Through Bore encoder on the steering output.

Ported unchanged from rev_ctl ``examples/swerve_constants.py`` (commit cfcfc32).
"""
import math
from dataclasses import dataclass

INCH = 0.0254

# --- Geometry -------------------------------------------------------------------------
# 17 in square frame; a MAXSwerve module is a 3.96 in square at the frame corner with its
# steering axis at the centre, so wheels sit 1.98 in in from both outer edges.
FRAME = 17.0 * INCH
WHEEL_INSET = 3.96 / 2 * INCH
HALF_WHEELBASE = FRAME / 2 - WHEEL_INSET   # x, centre to wheel (0.1656 m)
HALF_TRACK = FRAME / 2 - WHEEL_INSET       # y, centre to wheel
# Heights above the floor (REV drawings): the 2x1 frame tubes stand 2 in tall with their bottom face
# 2.75 in up; the top rails (1x1 tube, same 17 in square), on which the arm's mount plate sits, have
# their top face 6 in above that bottom face. Between the two frames is the robot's interior volume.
FRAME_BOTTOM_Z = 2.75 * INCH                # 0.0699 m
FRAME_TOP_Z = FRAME_BOTTOM_Z + 2.0 * INCH   # 0.1207 m
TOP_Z = FRAME_BOTTOM_Z + 6.0 * INCH         # 0.2223 m: the robot's top surface
TOP_RAIL_BOTTOM_Z = TOP_Z - 1.0 * INCH      # 0.1969 m


@dataclass(frozen=True)
class Module:
    name: str
    drive_id: int
    steer_id: int
    x: float              # m, wheel position
    y: float
    steer_offset: float   # encoder reading (rotations) when the wheel points forward


# CAN IDs: drive 1-4 counter-clockwise from front left, steering = drive + 4.
# steer_offset: the encoders read 0 in the calibration jig; each module is mounted turned
# another 90 degrees, so "forward" is a quarter turn apart (bevel gears then face left).
MODULES = (
    Module("FL", 1, 5, +HALF_WHEELBASE, +HALF_TRACK, 0.75),
    Module("FR", 4, 8, +HALF_WHEELBASE, -HALF_TRACK, 0.0),
    Module("RR", 3, 7, -HALF_WHEELBASE, -HALF_TRACK, 0.25),
    Module("RL", 2, 6, -HALF_WHEELBASE, +HALF_TRACK, 0.5),
)

# --- Drive train ----------------------------------------------------------------------
WHEEL_DIAMETER = 3.0 * INCH
DRIVE_PINION = 13                                  # teeth
DRIVE_REDUCTION = (45 * 22) / (DRIVE_PINION * 15)  # MAXSwerve: 5.08 for 13T
DRIVE_MPS_PER_RPM = math.pi * WHEEL_DIAMETER / DRIVE_REDUCTION / 60  # motor RPM -> wheel m/s
VORTEX_FREE_RPM = 6784
DRIVE_FREE_SPEED = VORTEX_FREE_RPM * DRIVE_MPS_PER_RPM  # m/s at 12 V (5.3)

# --- Gains (soft, for the first tests) --------------------------------------------------
# Units: steering error in encoder rotations, drive velocity in wheel m/s; P output is duty
# cycle, feedforward is in volts (firmware 26: parameter 16 is kV, not a duty-cycle F gain).
STEER_P = 3.0                # duty per rotation; REV's template is 2*pi (P = 1 per radian)
STEER_MAX_OUTPUT = 0.3       # duty cycle (about 1.2 wheel rotations/s at the NEO 550's free speed)
# Steering is plain position PID on the absolute encoder (no MAXMotion: a profile restarts
# on every new setpoint, so with a goal that changes every loop the wheels barely moved).
DRIVE_P = 0.02               # duty per m/s; REV's template uses 0.04
DRIVE_KV = 12.0 / DRIVE_FREE_SPEED         # V per m/s (2.25): 12 V at free speed, as REV's template
# Low-speed friction (hand-tuned on the floor; initial values, 2026-11-11). Without them the loop gave ~1 % duty at
# 0.05 m/s and never more while a wheel stood still, so the base stalled below ~0.06 m/s (e.g. slow turns in place).
# kS: volts added in the direction of any non-zero speed command: the KINETIC friction (rolling), below the
#     breakaway voltage (~0.15-0.2 V seen: 0.06 m/s stalls, 0.08 moves), so the base doesn't creep or lurch.
# I: covers the rest of the breakaway while a wheel is stuck, then stays small: duty per (m/s of error x ms), as the
#    SPARK accumulates every 1 ms; reset outside DRIVE_I_ZONE (m/s of error) and capped at DRIVE_I_MAX_ACCUM.
#    0.05 m/s stuck for 0.3 s adds ~0.09 V. The cap (I x cap = 0.01 duty = 0.12 V) stays BELOW the breakaway: at a
#    zero speed command kS adds nothing but a leftover accumulator still pushes, and must not move a stopped wheel.
#    Starting: kS + the full I = 0.22 V, above the breakaway.
DRIVE_KS = 0.10              # V
DRIVE_I = 0.0005             # duty per (m/s x ms)
DRIVE_I_ZONE = 0.10          # m/s of error
DRIVE_I_MAX_ACCUM = 20.0     # m/s x ms
DRIVE_MAX_OUTPUT = 0.35      # duty cycle (about 1.8 m/s)
DRIVE_RAMP = 0.5             # s from 0 to full output

# --- Limits for the first tests -------------------------------------------------------
# Smart current limits as REV's MAXSwerve template (applied at every speed). The factory 80 A
# let four nearly stalled steering motors brown out the supply at the start of a big turn; a
# very low limit (e.g. 3 A, below a Vortex's free current) stops the SPARKs driving at all.
STEER_CURRENT_LIMIT = 20                # A, NEO 550
DRIVE_CURRENT_LIMIT = 40                # A, NEO Vortex (REV's template: 50)
SECONDARY_CURRENT_LIMIT = 115.0         # A: firmware 26 default
NOMINAL_VOLTAGE = 12.0       # voltage compensation: same response at any battery voltage
MAX_WHEEL_SPEED = 5.0        # m/s, just under DRIVE_FREE_SPEED; all wheels scale down together above it
COMMAND_TIMEOUT = 0.25       # s without a drive() call -> motors disabled
