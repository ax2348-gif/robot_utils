"""The default tabletop scene (docs/sim.md, "The default scene"): a fixed table in front of the robot's start pose
(base at the origin facing +x), graspable objects on it, two objects on the floor for walking tasks.

Derived from the robot's model, not guessed (``swerve/constants.py``, where the description's base dimensions come
from, and ``arm/config.py``):

- **table top height** = the arm's mount (the base's top surface ``TOP_Z`` = 0.222 m, where the arm's mount plate
  sits) + ``TOP_ABOVE_MOUNT`` (0.08 m) = 0.302 m: above the base top, so the table's edge clears the chassis, and in
  the height band where the arm reaches farthest pointing straight down (sampled from the model's joint ranges:
  top-down grasps at the mount's y reach x 0.55–0.6 m for table heights 0.15–0.40 m);
- **front edge** = the base's front (``FRAME`` / 2 = 0.216 m) + ``EDGE_GAP`` (0.04 m), so the arm reaches the first
  ~0.3 m of the table top from the start pose; the table is long in y (1.6 m), so driving sideways along it brings
  the rest in reach (it is fixed and the robot cannot pass under it);
- **graspable objects** fit the gripper's opening (``GAP_MAX`` = 49.6 mm): each has a horizontal width <= 0.8 x it
  (``cube_view`` 45 mm); the near ones are in the arm's reach (and the wrist camera's view), ``cube_view`` near the
  far edge is in the head camera's (which, at its calibrated mount ~1.0 m up and ~19 deg down, sees the table top
  only beyond x ~0.68 m).
"""
from __future__ import annotations

TOP_ABOVE_MOUNT = 0.08      # m, table top above the arm's mount (the base's top surface)
EDGE_GAP = 0.04             # m, between the base's front and the table's edge
TABLE_DEPTH = 0.60          # m (x)
TABLE_WIDTH = 1.60          # m (y)
TABLE_THICKNESS = 0.03      # m
LEG = 0.04                  # m, square legs


def tabletop() -> list[dict]:
    """The scene's objects (POST /scene/objects fields), poses in the world with the base at its start pose."""
    from ..arm.config import GAP_MAX
    from ..swerve.constants import FRAME, TOP_Z
    top = TOP_Z + TOP_ABOVE_MOUNT
    x0 = FRAME / 2 + EDGE_GAP                       # front edge
    xc = x0 + TABLE_DEPTH / 2
    wood, legc = [0.62, 0.50, 0.38, 1.0], [0.40, 0.33, 0.26, 1.0]
    objs = [{"name": "table", "shape": "box", "fixed": True, "size": [TABLE_DEPTH, TABLE_WIDTH, TABLE_THICKNESS],
             "xyz": [xc, 0.0, top - TABLE_THICKNESS / 2], "rgba": wood, "friction": [0.8, 0.005, 0.0001]}]
    leg_h = top - TABLE_THICKNESS
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, -1), (-1, 1)), 1):
        objs.append({"name": f"table_leg{i}", "shape": "box", "fixed": True, "size": [LEG, LEG, leg_h],
                     "xyz": [xc + sx * (TABLE_DEPTH / 2 - LEG), sy * (TABLE_WIDTH / 2 - LEG), leg_h / 2], "rgba": legc})
    w = 0.8 * GAP_MAX                               # widest graspable dimension (~40 mm)

    def on_top(name, x, y, h, **kw):
        return {"name": name, "xyz": [x0 + x, y, top + h / 2 + 0.002], **kw}

    objs += [
        on_top("cube", 0.10, -0.15, 0.04, shape="box", size=[0.04, 0.04, 0.04], mass=0.06, rgba=[0.85, 0.20, 0.15, 1]),
        on_top("can", 0.16, 0.05, 0.10, shape="cylinder", radius=w / 2, length=0.10, mass=0.15,
               rgba=[0.20, 0.45, 0.85, 1]),
        on_top("block", 0.20, -0.32, 0.05, shape="box", size=[0.03, 0.08, 0.05], mass=0.08, yaw=0.4,
               rgba=[0.25, 0.65, 0.30, 1]),
        on_top("cube_far", 0.14, 0.55, 0.04, shape="box", size=[0.04, 0.04, 0.04], mass=0.06,
               rgba=[0.90, 0.75, 0.15, 1]),                    # drive left along the table to reach it
        # in the head camera's view: on the robot it sits ~1.0 m up, pitched ~19 deg down (its calibrated mount), so it
        # sees the table top only beyond x ~0.68 m; the near objects are for the wrist camera (gripper looking down)
        on_top("cube_view", 0.48, 0.0, 0.045, shape="box", size=[0.045, 0.045, 0.045], mass=0.07,
               rgba=[0.92, 0.92, 0.90, 1]),
        # on the floor, for walking tasks
        {"name": "crate", "shape": "box", "size": [0.30, 0.30, 0.30], "mass": 3.0, "xyz": [1.2, -1.4, 0.15],
         "rgba": [0.80, 0.50, 0.20, 1]},
        {"name": "bin", "shape": "cylinder", "radius": 0.15, "length": 0.40, "mass": 2.0, "xyz": [-1.0, 1.3, 0.20],
         "rgba": [0.55, 0.35, 0.70, 1]},
    ]
    return objs
