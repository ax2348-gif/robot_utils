"""Gripper types: a parallel gripper on an arm's end_link (two prismatic fingers along end_link y, the right one
mimicking the left; one motor through a pinion and racks). What the arm node's gripper law, the sim's finger joints
and contacts, the URDF, the block views and the WBC need of one.

- ``rebot``: the reBot arm's own gripper (arm/config.py, arm/geometry.py: unchanged; the reBot arm keeps it).
- ``anvil``: Anvil's Customizable Gripper, as in Anvil's OpenArm v2 URDF (robots/openarm/README.md; its geometry in
  robots/openarm/gripper/): 0-100 mm (the URDF), linear direct drive, a DM-J4310-2EC motor (below: what is Anvil's, what is derived).

The frame is the reBot's end_link convention: origin at the fingertips midway between the jaws, x out of the
fingertips (the approach), y the jaw travel (the left jaw at +y), z completing it. Boxes are arm/geometry.py's
format ``[frame, cx, cy, cz, sx, sy, sz]`` (mm, axis-aligned): frame ``end`` (on end_link) or ``finger`` (on the left
finger at gap 0, its inner face at y = 0; the right finger is its mirror in y).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

REBOT = "rebot"


@dataclass(frozen=True)
class GripperType:
    name: str
    boxes: tuple                 # [frame, cx, cy, cz, sx, sy, sz] mm
    box_names: tuple
    travel: float                # m, each finger (the URDF's finger_left upper limit)
    pitch_radius: float          # m, the pinion (gap per motor rad = 2 r)
    open_rad: float              # motor rad when fully open (0 = closed; negative opens)
    default_max_force: float     # N squeeze (the arm node's default force limit)
    hard_max_force: float        # N squeeze (the most a request may set)
    motor_torque_max: float      # N m, the gripper motor
    g_kp: float                  # the motor's MIT gains (N m / rad, N m s / rad)
    g_kd: float
    length: float                # m, flange face to fingertips
    tcp: tuple                   # m in end_link: the grasp point (the WBC / teleop TCP)
    jaw_height_root: float       # m, the pad's extent along end_link z at its root / tip
    jaw_height_tip: float
    end_mass: float              # kg, end_link (the motor block, the housing, the rail)
    end_com: tuple               # m in end_link
    end_inertia: tuple           # kg m^2, ixx iyy izz about the COM (end_link axes)
    finger_mass: float
    finger_effort: float         # N, the URDF finger joint's effort limit
    finger_velocity: float       # m/s, each finger
    doc: str = ""

    @property
    def gap_per_rad(self) -> float:
        return 2 * self.pitch_radius

    @property
    def gap_max(self) -> float:
        """m: the gap at the fully open motor angle (<= 2 x travel)."""
        return -self.open_rad * self.gap_per_rad

    def public(self) -> dict:
        return {**asdict(self), "gap_max": self.gap_max, "gap_per_rad": self.gap_per_rad}


def _rebot() -> GripperType:
    from ..nodes.arm.geometry import REBOT_BOX_NAMES, REBOT_BOXES
    sel = [(b, n) for b, n in zip(REBOT_BOXES, REBOT_BOX_NAMES) if b[0] in ("end", "finger")]
    return GripperType(
        name=REBOT, boxes=tuple(tuple(b) for b, _ in sel), box_names=tuple(n for _, n in sel),
        travel=0.0285, pitch_radius=0.008, open_rad=-3.1, default_max_force=50.0, hard_max_force=120.0,
        motor_torque_max=7.0, g_kp=1.0, g_kd=0.05, length=0.1603, tcp=(-0.15, 0.0, 0.0),
        jaw_height_root=0.033, jaw_height_tip=0.008, end_mass=0.5, end_com=(-0.0738, 0.0, 0.0),
        end_inertia=(0.00037354, 0.00015830, 0.00045353), finger_mass=0.03, finger_effort=8.0, finger_velocity=0.08,
        doc="the reBot arm's gripper (rebot_ctl): 49.6 mm of gap at the calibrated open angle, 57 mm finger travel")


# anvil: Anvil's gripper as on Anvil's OpenArm v2 bimanual URDF (read 2026-11-25; robots/openarm/README.md "The
# gripper"; its hand and fingers in our end_link convention: robots/openarm/gripper/). Geometry, mass and inertia are
# Anvil's URDF's, measured in end_link (at the fine tips):
# - stroke: each finger 0..0.05 m (the URDF's finger_joint1 / 2), 100 mm of gap; the fine tips meet at q = 0 (their
#   pads overlap 1 mm at the very tip, 3 mm apart at the root). Anvil's docs (docs.anvil.bot/products/customizable-
#   gripper, read 2026-11-25) say 0-110 mm: the URDF is authoritative here (the docs' figure may be the standard
#   fingers'; the URDF carries the fine tips);
# - the motor law (stated by Anvil's docs: a DM-J4310-2EC, linear direct drive, 0 rad fully closed to -2.858 rad fully
#   open): the URDF's 100 mm over the docs' 2.858 rad, 34.99 mm of gap per motor rad, an equivalent pinion radius of
#   17.50 mm (with the docs' 110 mm it would be 19.25 mm);
# - force (an INFERENCE, unchanged): Anvil's 150 N rated / 350 N max gripping force is both fingers' sum (the motor's
#   ~3 / 7 N m over the pinion is 171 / 400 N of total squeeze at 17.5 mm); the arm node's squeeze is each jaw's: its
#   default 75 N and hard cap 175 N. The URDF's finger effort (10 N) and velocity (1 m/s) are placeholders, no limit
#   anywhere in our stack (the sim lifts the URDF's finger effort);
# - gains: the reBot gripper's MIT gains x (17.50 / 8)^2, the same squeeze per mm of gap error;
# - masses: the hand 0.7 kg (with the wrist camera; Anvil's docs: 0.72 kg), each finger 0.036 kg; inertias the URDF's
#   (end_inertia: the diagonal in end_link axes; the URDF carries the full tensors);
# - flange to fingertips 0.152 m (Anvil's link7 flange face to the fine tips), joint-7 axis to fingertips 0.2026 m
#   (the mount on Enactic's arm keeps that: robots/openarm/build_urdf.py END_FROM_J7); Anvil's
#   docs' 480 / 650 mm reach without / with the gripper implies 170 mm;
# - the TCP: Anvil's hand_tcp, 22.5 mm behind the fingertips;
# - boxes: bounded from Anvil's visual meshes (the sim's jaw contact uses them: world.py ``_jaws``). The pad's inner
#   face is taken flat at y = 0 (the fine tip's leans in: 0 at the front 40 mm, 1-3 mm behind); the jaw is the front
#   40 mm (inner face at y = 0, 9 mm thick, the tip's height), the jaw back the rear 38 mm (1 mm off the face, the
#   pad's full 31.7 mm thickness).
ANVIL = GripperType(
    name="anvil",
    boxes=(("end", -130.85, -1.1, 0.2, 41.7, 57.0, 57.0),     # the hand's housing (behind the guide)
           ("end", -97.55, -1.1, 0.2, 24.9, 108.0, 57.0),     # the finger guide (the carriages' rail)
           ("end", -140.5, 0.0, 45.5, 67.0, 38.0, 41.0),      # the wrist camera's arm (toward link7)
           ("end", -102.5, 0.0, 86.5, 41.0, 38.0, 41.0),      # the wrist camera (Anvil's mesh: an Orbbec-size body)
           ("finger", -88.5, 16.25, 4.95, 20.2, 32.5, 35.5),  # carriage (on the guide)
           ("finger", -20.0, 4.5, -5.5, 40.0, 9.0, 38.6),     # the fine tip's front (inner face at y = 0)
           ("finger", -59.2, 16.85, -0.3, 38.4, 31.7, 49.0)), # the fine tip's back
    box_names=("gripper housing", "gripper guide", "wrist camera arm", "wrist camera", "jaw root", "jaw", "jaw back"),
    travel=0.05, pitch_radius=0.100 / 2.858 / 2, open_rad=-2.858, default_max_force=75.0, hard_max_force=175.0,
    motor_torque_max=7.0, g_kp=4.78, g_kd=0.239, length=0.152, tcp=(-0.0225, 0.0, 0.0),
    jaw_height_root=0.049, jaw_height_tip=0.016, end_mass=0.7, end_com=(-0.0725, -0.002, 0.0),
    end_inertia=(2.521e-4, 1.763e-5, 2.473e-4), finger_mass=0.036025, finger_effort=10.0, finger_velocity=1.0,
    doc="Anvil's gripper (Anvil's OpenArm v2 URDF, fine tips): 0-100 mm (the URDF; Anvil's docs say 110), "
        "150 N rated / 350 N max (both fingers, Anvil's docs), DM-J4310-2EC; geometry and inertia from Anvil's URDF")
assert abs(ANVIL.gap_max - 0.100) < 1e-9 and abs(2 * ANVIL.travel - ANVIL.gap_max) < 1e-9


def get(name: str | None) -> GripperType:
    if name in (None, REBOT):
        return _rebot()
    if name == ANVIL.name:
        return ANVIL
    raise ValueError(f"unknown gripper {name!r} (have rebot, anvil)")


def of_public(d: dict | None) -> GripperType:
    """A GripperType from ``public()`` (the arm node's environment), or the reBot's."""
    if not d:
        return _rebot()
    keys = {f for f in GripperType.__dataclass_fields__}
    return GripperType(**{k: (tuple(tuple(x) if isinstance(x, list) else x for x in v) if isinstance(v, list) else v)
                          for k, v in d.items() if k in keys})
