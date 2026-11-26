"""The cart-and-door scene (docs/sim.md, "The cart and door scene"): pull a cart into a room through a door that
opens toward the robot.

The robot starts at the origin facing +x (its start pose), the door straight ahead in the near wall of a closed room
(the only way in), a castered cart beside the robot on its right (the arm's side), its U-handle toward the door. The
door is a **pull** door from the robot's side: it swings toward the robot (-x) and a stop on the latch jamb keeps it
from opening the other way; from inside the room it opens by a push. A light closer (a parameter) returns it.

Every number is derived from the robot, not guessed (``swerve/constants.py`` ``FRAME``, the arm's reach and
``arm/config.py`` ``GAP_MAX``); the reasoning is in docs/sim.md. Two objects (POST /scene/objects fields):

- ``cart_room``: fixed, articulated: the room's walls (root link ``walls``) and the door (link ``door``, a hinge);
- ``cart``: free, articulated: the deck with its U-handle (root link ``deck``), four swivel casters (``swivel_*``,
  vertical hinges) each with its wheel (``wheel_*``, a hinge on the axle, which trails the swivel axis).
"""
from __future__ import annotations

import math

# -- the room and its door (the door's frame: x into the room, the doorway centred on y = 0) --------------------

WALL_GAP = 1.10             # m, from the base's front at the start pose to the wall's near face (clear of the swing)
WALL_T = 0.10               # m, wall thickness (the doorway's depth)
WALL_H = 2.20               # m
ROOM_D, ROOM_W = 3.00, 4.00  # m, the room's inside (x, y): closed, so the door is the only way in
LEAF_W = 0.90               # m, the door leaf's width (a standard 36 in interior door, ~0.91 m)
LEAF_T = 0.04               # m, its thickness (flush with the wall's near face when shut)
LEAF_H = 2.00               # m
LEAF_MASS = 12.0            # kg: a hollow-core interior door
LEAF_GAP = 0.003            # m, between the leaf and the jambs / head
FLOOR_GAP = 0.010           # m, under the leaf
STOP = 0.015                # m, the latch jamb's door stop (square): the leaf shuts against it, from the near side
OPEN_MAX = math.radians(110)  # rad, the overhead stop (a closer arm's limit)
HINGE_FRICTION = 0.20       # N m, the hinges' dry friction
HINGE_DAMPING = 0.20        # N m s/rad
DOOR_CLOSER = 3.0           # N m, the closer's moment holding the door shut (0: no closer)
CLOSER_PRELOAD = 1.5        # rad, the closer spring's relaxed angle is this far past shut: its moment grows from
                            # DOOR_CLOSER shut to 2.05 x at 90 deg (a near-constant-moment closer)
CLOSER_DAMPING = 12.0       # N m s/rad, the closer's closing-speed valve (it resists closing only, as the check
                            # valve of a hydraulic closer does): ~5 s from 90 deg to shut
CLOSER_MAX = 30.0           # N m, the valve's most (slamming it shut by hand overrides it)
PULL_BAR_R = 0.0125         # m, the door's pull bar (25 mm tube; GAP_MAX 49.6 mm)
PULL_BAR_L = 0.40           # m, vertical
PULL_BAR_Z = 0.60           # m, its centre above the floor: the arm's band (a forward grasp reaches x 0.8 m at z
                            # <= 0.55, 0.78 m at 0.65, 0.73 m at 0.70), not a person's ~1.0 m
PULL_BAR_Z_STD = 1.05       # m, a second bar at a person's standard handle height (ADA 0.86-1.22 m; the owner's ask:
                            # the human-scale OpenArm reaches there)
PULL_BARS_Z = (PULL_BAR_Z, PULL_BAR_Z_STD)   # both bars, on the same axis (0.40-0.80 and 0.85-1.25 m)
PULL_STANDOFF = 0.065       # m, the leaf's face to the bar's axis: 52 mm behind the bar for the fingers
PULL_EDGE = 0.07            # m, the bar's axis in from the leaf's free edge

# -- the cart (its frame: origin on the floor under the deck's centre, x toward the handle end) -----------------

CART_L, CART_W = 0.60, 0.42  # m, the deck (the robot's base is 0.432 m square)
DECK_T = 0.03               # m
WHEEL_R = 0.05              # m, 100 mm casters
WHEEL_W = 0.03              # m, the hub
TREAD_W, TREAD_RISE = 0.004, 0.003  # m: the crowned tread, the only part on the floor. MuJoCo touches a flat
                            # cylinder at both rim edges: the friction couple across its width is the tyre's scrub as
                            # its caster swivels (~0.06 N m per wheel at 4 mm; the 30 mm hub would be 0.4 N m)
CART_CONDIM = 3             # the cart's contacts: no torsional friction on top of the scrub (the objects' condim 4
                            # adds the floor's 5 mm x the load, 0.15 N m per wheel: no swivel at 15 N sideways)
TRAIL = 0.035               # m, the axle behind the swivel axis (horizontal), so the wheel trails the motion
SWIVEL_PLATE = 0.006        # m
CASTER_H = 0.14             # m, floor to the swivel plate's top (the deck's underside)
CASTER_INSET = (0.06, 0.06)  # m, swivel axes in from the deck's ends and sides
SWIVEL_FRICTION = 0.01      # N m: the swivel's ball race
SWIVEL_DAMPING = 0.002      # N m s/rad
WHEEL_FRICTION = 0.02       # N m: rolling resistance (0.4 N at the rim, ~1.4 % of the cart's weight in all)
WHEEL_DAMPING = 0.001       # N m s/rad
HANDLE_R = 0.0125          # m, the U-handle's tube (25 mm)
HANDLE_Z = 0.55             # m, the handle's top bar above the floor: in the arm's forward band, above the deck
HANDLE_SPAN = 0.38          # m, between the U's posts (centres)
HANDLE_X = CART_L / 2 - 0.02  # m, the U's plane, in from the deck's end
CART_MASS = 12.0            # kg in all: a small steel-and-plastic utility cart, empty
WHEEL_MASS, SWIVEL_MASS = 0.30, 0.20     # kg, each
POST_MASS, BAR_MASS = 0.35, 0.40          # kg, the handle's tubes
CART_POS = (0.0, -0.80)     # m: beside the robot on its right, clear of the door's swing, handle toward the door

WALL_C = [0.80, 0.80, 0.78, 0.35]          # see-through, as the revolving door's facade
DOOR_C, METAL = [0.62, 0.47, 0.33, 1.0], [0.70, 0.70, 0.72, 1.0]
CART_C, WHEEL_C = [0.25, 0.42, 0.62, 1.0], [0.15, 0.15, 0.15, 1.0]


def door_geometry() -> dict:
    """The doorway's numbers (m, world, the robot at its start pose): the wall's near face x, the opening's y range
    (between the jambs), the hinge axis (x, y), the latch stop's face y, the leaf's y range when shut, the swing's
    radius, and the clear width a straight pass has with the door open 90 deg (the latch stop to the open leaf)."""
    from ...swerve.constants import FRAME
    xn = FRAME / 2 + WALL_GAP
    half = LEAF_W / 2 + LEAF_GAP
    hinge_y = LEAF_W / 2
    stop_y = -half + STOP
    return {"near_face": xn, "opening": (-half, half), "hinge": (xn, hinge_y), "stop_face": stop_y,
            "leaf": (hinge_y - LEAF_W, hinge_y), "swing_r": math.hypot(LEAF_W, PULL_STANDOFF + PULL_BAR_R),
            "clear_open": (hinge_y - LEAF_T) - stop_y}


def cart_room(name: str = "cart_room", closer: float = DOOR_CLOSER) -> dict:
    """The room (POST /scene/objects fields, shape articulated, fixed): four walls closing a ``ROOM_D`` x ``ROOM_W``
    room whose near wall faces the robot ``WALL_GAP`` ahead of its front, the doorway centred on the robot's x axis,
    and the door: hinged on the doorway's left (+y) jamb at the wall's near face, opening toward the robot (about -z,
    0 = shut, up to ``OPEN_MAX``), shutting against the stop on the right jamb, with two vertical pull bars near its free
    edge on the near face (``PULL_BARS_Z``: the arm's band and a person's standard height). ``closer`` (N m holding it shut, 0: none): a spring toward ``CLOSER_PRELOAD`` past shut
    (``stiffness`` / ``springref``) and a closing-speed valve (a ``ratchet`` that resists closing only). Its origin is
    the doorway's centre at the wall's near face, on the floor."""
    g = door_geometry()
    half = -g["opening"][0]
    h_open = FLOOR_GAP + LEAF_H + LEAF_GAP
    y_out = ROOM_W / 2 + WALL_T                       # the walls' outer faces
    depth = WALL_T + ROOM_D + WALL_T
    walls = []
    for s in (1, -1):                                 # the near wall either side of the doorway
        walls.append({"shape": "box", "size": [WALL_T, y_out - half, WALL_H],
                      "pos": [WALL_T / 2, s * (half + y_out) / 2, WALL_H / 2]})
        walls.append({"shape": "box", "size": [depth, WALL_T, WALL_H],     # the side walls
                      "pos": [depth / 2, s * (y_out - WALL_T / 2), WALL_H / 2]})
    walls += [{"shape": "box", "size": [WALL_T, 2 * half, WALL_H - h_open],          # the head over the doorway
               "pos": [WALL_T / 2, 0.0, (h_open + WALL_H) / 2]},
              {"shape": "box", "size": [WALL_T, 2 * y_out, WALL_H],                  # the far wall
               "pos": [depth - WALL_T / 2, 0.0, WALL_H / 2]},
              {"shape": "box", "size": [STOP, STOP, h_open], "rgba": DOOR_C,          # the latch jamb's stop
               "pos": [LEAF_T + 0.001 + STOP / 2, -half + STOP / 2, h_open / 2]}]
    for w in walls:
        w["mass"] = 1.0
    zc = FLOOR_GAP + LEAF_H / 2
    yb = -LEAF_W + PULL_EDGE                          # the pull bar, in the door's frame (origin on the hinge axis)
    door = [{"shape": "box", "size": [LEAF_T, LEAF_W, LEAF_H], "pos": [LEAF_T / 2, -LEAF_W / 2, zc],
             "mass": LEAF_MASS, "rgba": DOOR_C},
            ]
    for bz in PULL_BARS_Z:                            # each pull bar and its two posts, along x
        door.append({"shape": "cylinder", "radius": PULL_BAR_R, "length": PULL_BAR_L, "pos": [-PULL_STANDOFF, yb, bz],
                     "mass": 0.3, "rgba": METAL})
        for s in (1, -1):
            door.append({"shape": "cylinder", "radius": 0.008, "length": PULL_STANDOFF, "rpy": [0.0, math.pi / 2, 0.0],
                         "pos": [-PULL_STANDOFF / 2, yb, bz + s * (PULL_BAR_L / 2 - 0.02)], "mass": 0.05,
                         "rgba": METAL})
    joint = {"type": "hinge", "axis": [0, 0, -1], "range": [0.0, OPEN_MAX], "damping": HINGE_DAMPING,
             "frictionloss": HINGE_FRICTION}
    if closer:
        joint.update(stiffness=closer / CLOSER_PRELOAD, springref=-CLOSER_PRELOAD,
                     ratchet={"direction": 1, "max_torque": CLOSER_MAX, "gain": CLOSER_DAMPING})
    return {"name": name, "shape": "articulated", "fixed": True, "xyz": [g["near_face"], 0.0, 0.0],
            "rgba": WALL_C, "links": [{"name": "walls", "geoms": walls},
                                      {"name": "door", "parent": "walls", "pos": [0.0, g["hinge"][1], 0.0],
                                       "joint": joint, "geoms": door}]}


CASTERS = {"fl": (1, 1), "fr": (1, -1), "rr": (-1, -1), "rl": (-1, 1)}    # the handle end is the front (+x)


def cart(name: str = "cart", xy=CART_POS, yaw: float = 0.0, mass: float = CART_MASS) -> dict:
    """A small platform cart (POST /scene/objects fields, shape articulated, free): a ``CART_L`` x ``CART_W`` deck
    on four swivel casters, a U-handle (two posts and a top bar, 25 mm tube) rising from its front end (+x). Each
    caster swivels on a vertical hinge; its wheel turns on a horizontal axle ``TRAIL`` behind the swivel axis, so it
    swings round to trail whichever way the cart moves. At q = 0 every wheel trails toward -x (rolling handle-first).
    ``mass``: kg in all (the deck takes what the handle and casters leave)."""
    top = CASTER_H + DECK_T
    handle_h = HANDLE_Z - top
    deck_mass = mass - 4 * (WHEEL_MASS + SWIVEL_MASS) - 2 * POST_MASS - BAR_MASS
    if not deck_mass > 0:
        raise ValueError(f"the cart's mass must exceed its handle and casters' "
                         f"{mass - deck_mass:.2f} kg")
    deck = [{"shape": "box", "size": [CART_L, CART_W, DECK_T], "pos": [0.0, 0.0, CASTER_H + DECK_T / 2],
             "mass": deck_mass, "rgba": CART_C},
            {"shape": "cylinder", "radius": HANDLE_R, "length": HANDLE_SPAN + 2 * HANDLE_R, "rpy": [math.pi / 2, 0, 0],
             "pos": [HANDLE_X, 0.0, HANDLE_Z], "mass": BAR_MASS, "rgba": METAL}]
    for s in (1, -1):
        deck.append({"shape": "cylinder", "radius": HANDLE_R, "length": handle_h,
                     "pos": [HANDLE_X, s * HANDLE_SPAN / 2, top + handle_h / 2], "mass": POST_MASS, "rgba": METAL})
    links = [{"name": "deck", "geoms": deck}]
    leg_h = CASTER_H - SWIVEL_PLATE - (WHEEL_R - 0.01)
    for tag, (sx, sy) in CASTERS.items():
        links.append({"name": f"swivel_{tag}", "parent": "deck",
                      "pos": [sx * (CART_L / 2 - CASTER_INSET[0]), sy * (CART_W / 2 - CASTER_INSET[1]), 0.0],
                      "joint": {"type": "hinge", "axis": [0, 0, 1], "damping": SWIVEL_DAMPING,
                                "frictionloss": SWIVEL_FRICTION},
                      "geoms": [{"shape": "box", "size": [0.06, 0.05, SWIVEL_PLATE],
                                 "pos": [0.0, 0.0, CASTER_H - SWIVEL_PLATE / 2], "mass": SWIVEL_MASS * 0.5,
                                 "rgba": METAL},
                                *({"shape": "box", "size": [0.04, 0.004, leg_h],     # the fork's legs
                                   "pos": [-TRAIL / 2, s * (WHEEL_W / 2 + 0.004), WHEEL_R - 0.01 + leg_h / 2],
                                   "mass": SWIVEL_MASS * 0.25, "rgba": METAL} for s in (1, -1))]})
        links.append({"name": f"wheel_{tag}", "parent": f"swivel_{tag}", "pos": [-TRAIL, 0.0, WHEEL_R],
                      "joint": {"type": "hinge", "axis": [0, 1, 0], "damping": WHEEL_DAMPING,
                                "frictionloss": WHEEL_FRICTION},
                      "geoms": [{"shape": "cylinder", "radius": WHEEL_R - TREAD_RISE, "length": WHEEL_W,   # hub
                                 "rpy": [math.pi / 2, 0.0, 0.0], "mass": WHEEL_MASS * 0.8, "rgba": WHEEL_C},
                                {"shape": "cylinder", "radius": WHEEL_R, "length": TREAD_W,           # tread
                                 "rpy": [math.pi / 2, 0.0, 0.0], "mass": WHEEL_MASS * 0.2, "rgba": WHEEL_C}]})
    return {"name": name, "shape": "articulated", "xyz": [xy[0], xy[1], 0.001], "rpy": [0.0, 0.0, yaw],
            "rgba": CART_C, "condim": CART_CONDIM, "links": links}


def cart_door_scene(closer: float = DOOR_CLOSER, cart_mass: float = CART_MASS) -> list[dict]:
    """The room with its pull door ahead of the robot (``cart_room()``) and the cart beside the robot (``cart()``)."""
    return [cart_room(closer=closer), cart(mass=cart_mass)]
