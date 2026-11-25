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
        on_top("can", 0.12, 0.0, 0.08, shape="cylinder", radius=w / 2, length=0.08, mass=0.12,
               rgba=[0.20, 0.45, 0.85, 1]),
        on_top("block", 0.14, -0.30, 0.05, shape="box", size=[0.03, 0.08, 0.05], mass=0.08, yaw=0.4,
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


# -- a cabinet: articulated furniture for drawer / door tasks ----------------------------------------------------

CAB_DEPTH, CAB_WIDTH, CAB_HEIGHT = 0.40, 0.50, 0.62     # m, outer x, y, z
CAB_WALL = 0.018                                         # m, panel thickness
CAB_PLINTH = 0.06                                        # m, kick plinth under the lowest compartment
CAB_GAP = 0.004                                          # m, between a front and its opening, each side
HANDLE_STANDOFF = 0.035                                  # m, from the front's face to the bar's centre
DRAWER_TRAVEL = 0.28                                     # m
CABINET_Y = -1.25                                        # m, beside the table (it ends at y -0.8)


def _handle(x_face: float, z: float, horizontal: bool, y: float = 0.0, length: float = 0.12) -> list[dict]:
    """A bar handle on a front whose face is at ``x_face`` (the front faces -x): the bar and its two posts. The bar
    (12 mm square) leaves 35 mm to the front, inside the gripper's 49.6 mm opening around it."""
    bar, post = 0.012, 0.010
    xb = x_face - HANDLE_STANDOFF
    size = [bar, length, bar] if horizontal else [bar, bar, length]
    parts = [{"shape": "box", "size": size, "pos": [xb, y, z], "mass": 0.05, "rgba": [0.75, 0.75, 0.78, 1]}]
    for s in (-1, 1):
        off = s * (length / 2 - post)
        parts.append({"shape": "box", "size": [HANDLE_STANDOFF, post, post],
                      "pos": [x_face - HANDLE_STANDOFF / 2, y + (off if horizontal else 0), z + (0 if horizontal else off)],
                      "mass": 0.02, "rgba": [0.75, 0.75, 0.78, 1]})
    return parts


def cabinet(name: str = "cabinet", x_front: float | None = None, y: float = CABINET_Y, yaw: float = 0.0) -> dict:
    """A fixed cabinet (POST /scene/objects fields, shape articulated) whose front faces -x: a door compartment at the
    bottom (hinged on its +y edge, opening outward up to ~110 deg) and two drawers above it (sliding out along -x up
    to ``DRAWER_TRAVEL``), each with a bar handle the gripper closes around. ``x_front``: where its front is
    (default: in line with the table's front edge, so the robot faces it from its start x after driving to ``y``).
    Its origin is the centre of its footprint on the floor."""
    from ..swerve.constants import FRAME
    x_front = FRAME / 2 + EDGE_GAP if x_front is None else x_front
    D, W, H, t = CAB_DEPTH, CAB_WIDTH, CAB_HEIGHT, CAB_WALL
    body_c, front_c = [0.86, 0.84, 0.80, 1.0], [0.55, 0.62, 0.70, 1.0]
    fx = -D / 2                                            # the front plane (carcass edge), local x
    # compartments, bottom up: (z0, z1) of each opening
    z_door = (CAB_PLINTH + t, 0.34)
    z_d1, z_d2 = (0.34 + t, 0.48), (0.48 + t, H - t)
    panels = [
        {"shape": "box", "size": [D, W, t], "pos": [0, 0, H - t / 2]},                       # top
        {"shape": "box", "size": [D, W, t], "pos": [0, 0, CAB_PLINTH + t / 2]},              # bottom
        {"shape": "box", "size": [D - t, W - 2 * t, CAB_PLINTH], "pos": [t / 2, 0, CAB_PLINTH / 2]},  # plinth
        {"shape": "box", "size": [D, t, H], "pos": [0, W / 2 - t / 2, H / 2]},               # sides
        {"shape": "box", "size": [D, t, H], "pos": [0, -W / 2 + t / 2, H / 2]},
        {"shape": "box", "size": [t, W - 2 * t, H - 2 * t], "pos": [D / 2 - t / 2, 0, H / 2]},  # back
        {"shape": "box", "size": [D - t, W - 2 * t, t], "pos": [t / 2, 0, 0.34 + t / 2]},     # dividers
        {"shape": "box", "size": [D - t, W - 2 * t, t], "pos": [t / 2, 0, 0.48 + t / 2]},
    ]
    for p in panels:
        p.update(mass=1.0, rgba=body_c)
    links = [{"name": "body", "geoms": panels}]

    def drawer(n: str, z0: float, z1: float) -> dict:
        """A drawer box behind a front panel; its frame is the carcass's, so q = 0 is closed."""
        inner_w, inner_h = W - 2 * t - 2 * CAB_GAP, z1 - z0 - 2 * CAB_GAP
        depth = D - 2 * t - 0.02
        x0 = fx + t                                        # behind the front panel
        zc, side_h = (z0 + z1) / 2, inner_h - 0.02
        geoms = [
            {"shape": "box", "size": [t, W - 2 * t - 2 * CAB_GAP, z1 - z0 - 2 * CAB_GAP],     # front, flush
             "pos": [fx + t / 2, 0, zc], "rgba": front_c},
            {"shape": "box", "size": [depth, inner_w - 2 * t, 0.010], "pos": [x0 + depth / 2, 0, z0 + CAB_GAP + 0.005]},
            {"shape": "box", "size": [depth, 0.010, side_h], "pos": [x0 + depth / 2, inner_w / 2 - t, z0 + CAB_GAP + side_h / 2]},
            {"shape": "box", "size": [depth, 0.010, side_h], "pos": [x0 + depth / 2, -inner_w / 2 + t, z0 + CAB_GAP + side_h / 2]},
            {"shape": "box", "size": [0.010, inner_w - 2 * t, side_h], "pos": [x0 + depth - 0.005, 0, z0 + CAB_GAP + side_h / 2]},
            *_handle(fx, zc, horizontal=True),
        ]
        for g in geoms:
            g.setdefault("mass", 0.25)
        return {"name": n, "parent": "body",
                "joint": {"type": "slide", "axis": [-1, 0, 0], "range": [0.0, DRAWER_TRAVEL], "damping": 3.0,
                          "frictionloss": 0.4},
                "geoms": geoms}

    links += [drawer("drawer_low", *z_d1), drawer("drawer_high", *z_d2)]
    # the door: hinged on the +y edge of the front, rotating about -z so it opens toward -x
    dz0, dz1 = z_door
    dw, dh = W - 2 * CAB_GAP, dz1 - dz0 + t - 2 * CAB_GAP
    hinge = [fx - t / 2, W / 2 - CAB_GAP, 0.0]
    links.append({"name": "door", "parent": "body", "pos": hinge,
                  "joint": {"type": "hinge", "axis": [0, 0, -1], "range": [0.0, 1.9], "damping": 0.6,
                            "frictionloss": 0.15},
                  "geoms": [{"shape": "box", "size": [t, dw, dh], "pos": [0, -dw / 2, (dz0 + dz1) / 2],
                             "mass": 0.8, "rgba": front_c},
                            *_handle(-t / 2, (dz0 + dz1) / 2 + 0.05, horizontal=False, y=-dw + 0.05)]})
    return {"name": name, "shape": "articulated", "fixed": True, "xyz": [x_front + D / 2, y, 0.0],
            "rpy": [0.0, 0.0, yaw], "rgba": body_c, "links": links}


def cabinet_scene() -> list[dict]:
    """The cabinet beside the table, with a cube in its upper drawer (to take out after opening it)."""
    cab = cabinet()
    x_c, y = cab["xyz"][0], cab["xyz"][1]
    return [cab, {"name": "drawer_cube", "shape": "box", "size": [0.035, 0.035, 0.035], "mass": 0.05,
                  "xyz": [x_c - 0.02, y, 0.48 + CAB_WALL + CAB_GAP + 0.010 + 0.0175 + 0.002],
                  "rgba": [0.90, 0.30, 0.55, 1]}]
