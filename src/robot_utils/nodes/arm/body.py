"""The arm's body as numbers: gripper and finger geometry, and every box of geometry.py, at a configuration.

Pure numpy, from the frames ``kin.frames(q)`` returns (joint1..6, end_link, tcp; 4x4 in the arm frame)
and the gripper gap. The node adds the arm frame's pose in the robot base frame (from the description
node) so every point is reported in both frames. Read-only geometry: it plans nothing and moves nothing.

end_link (URDF) is the gripper's own frame: origin at the fingertips, midway between the jaws; x points out
between the jaws (the approach direction), y is the jaw travel (left jaw at +y), z completes the frame.
The TCP that motion commands target sits at ``tcp_offset`` in end_link (default 0.15 m behind the
fingertips, near the wrist).
"""
from __future__ import annotations

import numpy as np

from .geometry import ARM_BOX_NAMES, ARM_BOXES, GRIPPER



def frame_of(frames: list, name: str) -> np.ndarray:
    """A box frame's 4x4 among ``frames`` (joint1..jointN, end_link, tcp): ``linkN`` = jointN's, ``end`` = end_link's,
    ``base`` = the arm frame."""
    if name == "base":
        return np.eye(4)
    if name == "end":
        return frames[-2]
    return frames[int(name[4:]) - 1]

# The finger prong (geometry.py, "jaw" box, finger frame = end_link moved gap/2 along y): its inner face
# is at y = 0 of the finger frame, it runs from the fingertip (x = 0) back to x = -58.9 mm, 13.6 mm thick,
# and tapers from +-16.5 mm (z) at the root to +-4 mm at the tip.
_JAW = next(b for b, n in zip(ARM_BOXES, ARM_BOX_NAMES) if n == "jaw")
JAW_LENGTH = _JAW[4] / 1000            # m, along end_link x
JAW_THICKNESS = _JAW[5] / 1000         # m, along end_link y (outward from the inner face)
JAW_HEIGHT_ROOT, JAW_HEIGHT_TIP = GRIPPER.jaw_height_root, GRIPPER.jaw_height_tip   # m, along end_link z
# Usable grasp depth: from the tips back to the jaw carriage's front face (the jaw-root box front, x = -59 mm).
_ROOT = next(b for b, n in zip(ARM_BOXES, ARM_BOX_NAMES) if n == "jaw root")
GRASP_DEPTH = -(_ROOT[1] + _ROOT[4] / 2) / 1000


def _box_corners(T: np.ndarray, b: list, sy: float = 1.0) -> np.ndarray:
    """8 corners (8x3) of a geometry.py box [frame, cx, cy, cz, sx, sy, sz] (mm) placed by T (4x4)."""
    c = np.array(b[1:4], float) / 1000
    s = np.array(b[4:7], float) / 1000
    P = np.array([[c[0] + i * s[0] / 2, sy * (c[1] + j * s[1] / 2), c[2] + k * s[2] / 2]
                  for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return P @ T[:3, :3].T + T[:3, 3]


def _box_pose(T: np.ndarray, b: list, sy: float = 1.0) -> dict:
    c = np.array([b[1], sy * b[2], b[3]], float) / 1000
    return {"center": (T[:3, :3] @ c + T[:3, 3]), "axes": T[:3, :3], "size": np.array(b[4:7], float) / 1000}


def finger_frames(E: np.ndarray, gap: float) -> dict:
    """The two finger frames (4x4): end_link moved +-gap/2 along its y (right = mirrored in y)."""
    out = {}
    for name, sgn in (("left", 1.0), ("right", -1.0)):
        F = E.copy()
        F[:3, 3] = E[:3, 3] + E[:3, 1] * sgn * gap / 2
        out[name] = (F, sgn)
    return out


def body(frames: list, gap: float) -> dict:
    """Gripper points, finger geometry and every arm box at one configuration, in the arm frame.

    frames: N + 2 4x4 arrays (joint1..N, end_link, tcp) in the arm frame; gap: m (clipped to [0, 2 x the finger
    travel]: 57 mm for the reBot's gripper).
    Returns numpy values; the node turns them into JSON in both frames.
    """
    g = float(min(max(gap, 0.0), 2 * GRIPPER.travel))
    E, TCP = frames[-2], frames[-1]          # joint1..N, end_link, tcp
    x, y, z = E[:3, 0], E[:3, 1], E[:3, 2]
    jaws = {}
    for name, (F, sgn) in finger_frames(E, g).items():
        o = F[:3, 3]                                   # inner face at the fingertip, mid height
        inward = -sgn * y                              # from this jaw's inner face toward the other jaw
        jaws[name] = {
            "tip": o,
            "inner_face_center": o - x * JAW_LENGTH / 2,
            "inner_face_root": o - x * JAW_LENGTH,
            "outer_face_tip": o - inward * JAW_THICKNESS,
            "inward": inward,
        }
    boxes = []
    for b, n in zip(ARM_BOXES, ARM_BOX_NAMES):
        if b[0] == "finger":
            for side, (F, sgn) in finger_frames(E, g).items():
                boxes.append((f"{side} {n}", "finger", _box_corners(F, b, sgn), _box_pose(F, b, sgn)))
            continue
        T = frame_of(frames, b[0])
        boxes.append((n, b[0], _box_corners(T, b), _box_pose(T, b)))
    return {"gap": g, "end_link": E, "tcp": TCP, "approach": x, "jaw_axis": y, "up": z, "jaws": jaws, "boxes": boxes}
