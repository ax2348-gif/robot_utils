"""The cart and door scene (docs/sim.md, "The cart and door scene"): a castered cart to pull into a closed room through
a door that opens toward the robot. The casters trail and the cart rolls under a few newtons at its handle; the door
holds against a push the wrong way, opens by its pull bar and its closer shuts it; the doorway fits the robot and the
cart with margins; reset restores everything; the joint spring (``stiffness`` / ``springref``) and the object
``condim`` it is built on are validated and survive save / load."""
import math

import mujoco
import numpy as np
import pytest
from fastapi.testclient import TestClient
from sim_helpers import make_world, robot_footprint

from robot_utils.nodes.sim.scenes import cart_door as C

pytest.importorskip("scipy")

GRASP = np.array([-0.05, 0.0, 0.0])     # between the finger pads, in arm_end_link (its +x points out of the gripper)


@pytest.fixture(scope="module")
def world():
    w = make_world(scene={"tabletop": False, "cart_door": True})
    yield w
    w.close()


@pytest.fixture(scope="module")
def free_door():
    """The scene without the closer (the door stays where it is left)."""
    w = make_world(scene={"tabletop": False, "cart_door": True, "cart_door_setup": {"closer": 0}})
    yield w
    w.close()


def door_q(world):
    return world.obj_joints["cart_room"]["door"]          # (qpos adr, dof adr)


def bar_point(world, z=C.PULL_BAR_Z):
    """The door's pull bar (on its axis, at height z) in the world, as the door is now."""
    b = world.m.body("obj_cart_room/door").id
    return world.d.xpos[b] + world.d.xmat[b].reshape(3, 3) @ np.array([-C.PULL_STANDOFF, -C.LEAF_W + C.PULL_EDGE, z])


def handle_point(world):
    """The middle of the cart's U-handle top bar in the world."""
    b = world.m.body("obj_cart").id
    return world.d.xpos[b] + world.d.xmat[b].reshape(3, 3) @ np.array([C.HANDLE_X, 0.0, C.HANDLE_Z])


def force_at(world, body: str, f, point_fn, steps: int) -> None:
    """``f`` (N, world) applied at ``point_fn()`` on ``body`` for ``steps`` physics steps (f may be a function of the
    world: called every step)."""
    b = world.m.body(body).id
    d = world.d
    for _ in range(steps):
        ff = np.asarray(f(world) if callable(f) else f, float)
        d.xfrc_applied[b, :3] = ff
        d.xfrc_applied[b, 3:] = np.cross(point_fn(world) - d.xipos[b], ff)
        world.step(1)
    d.xfrc_applied[b] = 0


def cart_speed(world):
    va = world.obj_idx["cart"][2]
    return world.d.qvel[va:va + 3].copy()


def reach(world, p, tool, jaw, base_xy, tries=8, yaw=0.0, data=None):
    """Can the grasp point reach world point ``p`` with the gripper's axis along ``tool`` and its jaws closing along
    ``jaw`` (either sign), the base at ``base_xy`` facing ``yaw``? (Damped least squares on the model's arm joints.)
    Returns the joint angles (``data``, if given, is left at that pose), or None."""
    m = world.m
    d = mujoco.MjData(m) if data is None else data
    d.qpos[:] = world.d.qpos
    d.qpos[world.base_q:world.base_q + 3] = [*base_xy, world.d.qpos[world.base_q + 2]]
    d.qpos[world.base_q + 3:world.base_q + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    e = m.body("arm_end_link").id
    ids = [m.joint(f"arm_joint{i}").id for i in range(1, 7)]
    lo, hi = m.jnt_range[ids, 0], m.jnt_range[ids, 1]
    rng = np.random.default_rng(0)
    tool, jaw = np.asarray(tool, float), np.asarray(jaw, float)
    for t in range(tries):
        q = np.array(world.cfg["components"]["arm"]["start_q"] if t == 0 else rng.uniform(lo, hi), float)
        for _ in range(300):
            d.qpos[world.arm_q] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            R = d.xmat[e].reshape(3, 3)
            g = d.xpos[e] + R @ GRASP
            ep = p - g
            j = jaw * (1.0 if R[:, 1] @ jaw >= 0 else -1.0)
            er = np.cross(R[:, 0], tool) + np.cross(R[:, 1], j)
            if np.linalg.norm(ep) < 0.005 and np.linalg.norm(er) < math.sin(math.radians(10)):
                return q
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, jr, g, e)
            J = np.vstack([jp[:, world.arm_v], 0.3 * jr[:, world.arm_v]])
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), np.concatenate([ep, 0.3 * er])), lo, hi)
    return None


# -- selection and layout ------------------------------------------------------------------------------------------

def test_preset_selects_the_scene_robot_facing_the_door_cart_beside_it():
    import pathlib

    import yaml
    from robot_utils.nodes.sim.world import World, load_config
    path = pathlib.Path(__file__).parents[1] / "deploy/sim/sim_cart_door_dynamic.yaml"
    cfg = load_config(str(path))
    assert cfg["components"]["arm"]["fidelity"] == "dynamic" and cfg["scene"]["view_anchor"] == [0.6, 0.0, 0.0]
    assert cfg["scene"]["tabletop"] is False and "timing" not in yaml.safe_load(path.read_text())
    objs = World.start_objects(type("W", (), {"cfg": cfg})())
    assert [o["name"] for o in objs] == ["cart_room", "cart"]
    room, cart = objs
    g = C.door_geometry()
    assert room["fixed"] and "fixed" not in cart                       # the room is welded, the cart free
    assert room["xyz"][1] == 0.0 and cfg["base"]["start"] == {"xyz": [0.0, 0.0, 0.0], "yaw": 0.0}   # straight ahead
    assert g["near_face"] == pytest.approx(0.216 + C.WALL_GAP, abs=1e-3)
    assert cart["xyz"][:2] == [0.0, -0.8] and cart["rpy"] == [0.0, 0.0, 0.0]    # on the robot's right, handle to +x


def test_doorway_fits_the_robot_and_the_cart_with_margins(world):
    """Open 90 deg, the doorway's clear width (the latch stop to the open leaf) leaves >= 15 cm each side of the robot
    (its footprint at the start pose, the arm included) and of the cart; the swing clears both at their starts; the
    room is closed (the door is the only way in)."""
    world.reset()
    x0, x1, y0, y1 = robot_footprint(world)
    assert (x1 - x0, y1 - y0) == pytest.approx((0.576, 0.454), abs=0.005)
    g = C.door_geometry()
    assert g["clear_open"] == pytest.approx(0.848, abs=0.002)
    assert (g["clear_open"] - (y1 - y0)) / 2 >= 0.19                   # 19.7 cm each side of the robot
    cart_w = C.CART_W                                                  # the deck is the widest part (casters inside)
    assert (g["clear_open"] - cart_w) / 2 >= 0.21                      # 21.4 cm each side of the cart
    assert C.HANDLE_SPAN + 2 * C.HANDLE_R < cart_w
    hx, hy = g["hinge"]
    def gap(xa, xb, ya, yb):                                            # the hinge axis to a rectangle
        return math.hypot(max(xa - hx, 0.0, hx - xb), max(ya - hy, 0.0, hy - yb))

    assert gap(x0, x1, y0, y1) >= g["swing_r"] + 0.05                   # the swing misses the robot by 8 cm
    cx, cy = C.CART_POS
    assert gap(cx - C.CART_L / 2, cx + C.CART_L / 2, cy - C.CART_W / 2, cy + C.CART_W / 2) >= g["swing_r"] + 0.3
    assert cy + C.CART_W / 2 < y0 - 0.3                                 # 35 cm right of the robot
    # the room: the near wall runs past the side walls, which with the far wall close it
    room = next(o for o in C.cart_door_scene() if o["name"] == "cart_room")
    walls = room["links"][0]["geoms"]
    ys = [abs(w["pos"][1]) + w["size"][1] / 2 for w in walls]
    assert max(ys) == pytest.approx(C.ROOM_W / 2 + C.WALL_T)
    assert C.ROOM_D >= C.CART_L + (x1 - x0) + 1.0                      # the robot and the cart fit inside with room


def closer_torque(q: float, closer: float = C.DOOR_CLOSER) -> float:
    """N m: the closer spring's moment shutting the door at angle q (rad)."""
    return closer / C.CLOSER_PRELOAD * (q + C.CLOSER_PRELOAD)


def test_arm_pulls_the_door_open_comfortably_over_its_whole_swing():
    """The closer is gentle: over the whole swing (0-110 deg) the pull it takes at the bar, along the bar's path, is
    under a third of what the arm can exert there (gripping the bar head-on, the base 0.6 m back, facing the leaf: the
    grasp the robot keeps while it backs round the arc), against the motor torque limits (27 N m joints 1-3, 7 N m
    joints 4-6, arm/config.py) less gravity and each joint's dry friction, and of what the jaws hold by friction at
    the gripper's 50 N squeeze before the bar slips. It still swings shut on its own, slowly (test below)."""
    from robot_utils.nodes.arm.config import DEFAULT_MAX_FORCE, motor_torque_limits
    w = make_world(scene={"tabletop": False, "cart_door": True}, components={"arm": {"fidelity": "dynamic"}})
    try:
        lim = motor_torque_limits()[0]
        assert list(lim) == [27.0, 27.0, 27.0, 7.0, 7.0, 7.0]
        coulomb = np.array(w.cfg["components"]["arm"]["dynamic"]["coulomb"])
        mu = float(w.cfg["contact"]["fingers"]["friction"][0])
        hold = 2 * mu * DEFAULT_MAX_FORCE                                 # two pads: 150 N before the bar slips
        g = C.door_geometry()
        hx, hy = g["hinge"]
        lever = math.hypot(C.LEAF_W - C.PULL_EDGE, C.PULL_STANDOFF)       # the hinge axis to the bar: 0.83 m
        mount = np.array([0.1159, -0.1459])                               # the arm's mount on the base
        rows = []
        m = w.m
        e = m.body("arm_end_link").id
        for deg in (0, 30, 60, 90, 110):
            q = math.radians(deg)
            n = np.array([-math.cos(q), math.sin(q)])                     # the leaf's near-face normal: toward the robot
            along = np.array([math.sin(q), math.cos(q)])                  # the leaf, horizontally
            r0 = np.array([-C.PULL_STANDOFF, -(C.LEAF_W - C.PULL_EDGE)])
            bar = np.array([hx, hy]) + np.array([[math.cos(q), math.sin(q)], [-math.sin(q), math.cos(q)]]) @ r0
            yaw = math.atan2(-n[1], -n[0])                                 # facing the leaf
            c, s = math.cos(yaw), math.sin(yaw)
            base = bar + 0.6 * n - np.array([[c, -s], [s, c]]) @ [0.0, mount[1]]
            d = mujoco.MjData(m)
            p = np.array([*bar, C.PULL_BAR_Z])
            qa = reach(w, p, [-n[0], -n[1], 0.0], [along[0], along[1], 0.0], base, yaw=yaw, data=d)
            assert qa is not None, deg
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)
            grav = d.qfrc_bias[w.arm_v]                                    # gravity at that pose
            jp = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, None, d.xpos[e] + d.xmat[e].reshape(3, 3) @ GRASP, e)
            per_n = np.abs(jp[:, w.arm_v].T @ np.array([*n, 0.0]))          # N m per N of pull toward the robot
            arm = float(np.min((lim - np.abs(grav) - coulomb) / np.maximum(per_n, 1e-9)))
            need = (closer_torque(q) + C.HINGE_FRICTION) / lever
            rows.append((deg, closer_torque(q), need, arm))
        print("\n".join(f"{dg:3d} deg: closer {t:.2f} N m, pull {nd:.2f} N, arm {a:.1f} N, jaws {hold:.0f} N"
                        for dg, t, nd, a in rows))
        for deg, _, need, arm in rows:
            assert need * 3 < min(arm, hold), (deg, need, arm, hold)
        assert closer_torque(0.0) == pytest.approx(C.DOOR_CLOSER) and closer_torque(C.OPEN_MAX) < 7.0
    finally:
        w.close()


def test_handles_fit_the_gripper_and_are_in_the_arms_reach(world):
    from robot_utils.nodes.arm.config import GAP_MAX
    assert 2 * C.PULL_BAR_R < 0.6 * GAP_MAX and 2 * C.HANDLE_R < 0.6 * GAP_MAX      # 25 mm tubes, 49.6 mm opening
    assert C.PULL_STANDOFF - C.PULL_BAR_R >= 0.05                      # finger room behind the door's bar
    world.reset()
    mount_y = -0.1459
    bar = bar_point(world)
    for dz in (-0.1, 0.0, 0.1):                                         # along the vertical bar, approaching it head-on
        p = bar + [0, 0, dz]
        assert reach(world, p, [1, 0, 0], [0, 1, 0], (p[0] - 0.6, p[1] - mount_y)) is not None, dz
    h = handle_point(world)                                             # the cart's top bar, gripper level, jaws vertical
    assert reach(world, h, [1, 0, 0], [0, 0, 1], (h[0] - 0.55, h[1] - mount_y)) is not None


# -- the cart ------------------------------------------------------------------------------------------------------

def test_casters_swivel_to_trail_the_pull(world):
    """Pulled sideways (+y, at the deck's centre of mass) or backward (-x), every caster swings its wheel behind its
    swivel axis: trailing the motion."""
    for f, want in (([0, 12.0, 0], math.pi / 2), ([-8.0, 0, 0], math.pi)):
        world.reset()
        world.teleport_base([-3.0, 3.0, 0.0])                          # the robot out of the way
        world.move_object("cart", [-1.5, -1.5, 0.001], yaw=0.0)        # open floor
        force_at(world, "obj_cart", f, lambda w: w.d.xipos[w.m.body("obj_cart").id], 2000)
        st = world.object_state("cart")
        v = cart_speed(world)
        assert np.linalg.norm(v[:2]) > 0.1, v
        heading = math.atan2(v[1], v[0]) - st["rpy"][2]                  # the motion, in the cart's frame
        assert abs((heading - want + math.pi) % (2 * math.pi) - math.pi) < 0.5         # it yaws a little
        for tag in C.CASTERS:
            q = st["joints"][f"swivel_{tag}"]
            assert abs((q - heading + math.pi) % (2 * math.pi) - math.pi) < 0.4, (f, tag, q, heading)
            m, d = world.m, world.d
            axis = d.xpos[m.body(f"obj_cart/swivel_{tag}").id]
            wheel = d.xpos[m.body(f"obj_cart/wheel_{tag}").id]
            assert (wheel - axis)[:2] @ (v[:2] / np.linalg.norm(v[:2])) < -0.03, tag      # behind the axis


def test_cart_rolls_under_a_few_newtons_at_its_handle(world):
    """Pulled at the handle bar toward +x (the casters trailing already): 1 N does not move it (the wheels' rolling
    resistance), 3 N rolls it; rolling steadily at ~0.3 m/s takes ~2.1 N."""
    moved = {}
    for f in (1.0, 3.0):
        world.reset()
        world.teleport_base([-3.0, 3.0, 0.0])
        world.move_object("cart", [-1.5, -1.5, 0.001], yaw=0.0)
        x0 = world.object_state("cart")["xyz"][0]
        force_at(world, "obj_cart", [f, 0, 0], handle_point, 2000)
        moved[f] = world.object_state("cart")["xyz"][0] - x0
    assert moved[1.0] < 0.01 and moved[3.0] > 0.15, moved               # 0.002 and 0.205 m in 2 s measured
    world.reset()
    world.teleport_base([-3.0, 3.0, 0.0])
    world.move_object("cart", [-2.0, -1.5, 0.001], yaw=0.0)            # 1.4 m to roll in 4 s, clear of the wall
    fs = []

    def servo(w):                                                       # hold 0.35 m/s
        fs.append(min(max(60.0 * (0.35 - cart_speed(w)[0]), 0.0), 30.0))
        return [fs[-1], 0, 0]

    force_at(world, "obj_cart", servo, handle_point, 4000)
    assert 0.29 < cart_speed(world)[0] < 0.36                         # 0.31 m/s: the servo's droop
    assert 1.5 < np.mean(fs[-1000:]) < 3.0, np.mean(fs[-1000:])        # 2.2 N measured
    st = world.object_state("cart")
    assert abs(st["rpy"][0]) < 0.01 and abs(st["rpy"][1]) < 0.01       # all four wheels down


# -- the door ------------------------------------------------------------------------------------------------------

def test_door_holds_against_a_push_the_wrong_way(world):
    world.reset()
    qa, _ = door_q(world)
    assert abs(world.d.qpos[qa]) < 1e-3                                  # shut, held by the closer
    force_at(world, "obj_cart_room/door", [100.0, 0, 0], bar_point, 1000)   # 100 N at the bar, into the room
    assert world.d.qpos[qa] > -0.002                                      # -0.6 mrad measured


def test_door_opens_by_its_pull_bar_and_the_closer_shuts_it(world):
    world.reset()
    qa, va = door_q(world)
    f = 0.0
    while world.d.qpos[qa] < 0.02 and f < 20:                            # a slow ramp, 0.5 N/s, toward the robot
        f += 0.01
        force_at(world, "obj_cart_room/door", [-f, 0, 0], bar_point, 20)
    # the closer's 3 N m + the hinges' 0.2 N m over the bar's 0.83 m lever: 3.9 N (4.4 N measured, the ramp's lag)
    assert 3.8 < f < 5.0, f
    world.reset()
    fs = []

    def pull(w):                                                          # along the bar's path, 0.5 rad/s
        q = w.d.qpos[qa]
        fs.append(min(max(40.0 * (0.5 - w.d.qvel[va]), 0.0), 30.0))
        return fs[-1] * np.array([-math.cos(q), math.sin(q), 0.0])

    n = 0
    while world.d.qpos[qa] < math.pi / 2 and n < 8000:
        force_at(world, "obj_cart_room/door", pull, bar_point, 10)
        n += 10
    assert world.d.qpos[qa] >= math.pi / 2 and np.median(fs) < 8.0, (n, np.median(fs))   # 6.2 N measured
    t, fastest = 0, 0.0
    while world.d.qpos[qa] > 0.01 and t < 12000:
        world.step(10)
        t += 10
        fastest = max(fastest, -float(world.d.qvel[va]))
    assert 3000 < t < 8000, t                                             # shut again in 4.9 s
    print(f"closed from 90 deg in {t / 1000:.2f} s, at most {fastest:.2f} rad/s, {-world.d.qvel[va]:.2f} at the stop")
    assert fastest < 0.6 and -world.d.qvel[va] < 0.4                      # slowly: the valve, not a slam
    world.step(1000)
    assert abs(world.d.qpos[qa]) < 2e-3


def test_door_without_a_closer_stays_where_it_is_left(free_door):
    qa, _ = door_q(free_door)
    free_door.reset()
    j = next(ln for ln in free_door.objects["cart_room"]["links"] if ln["name"] == "door")["joint"]
    assert "stiffness" not in j and "ratchet" not in j
    free_door.d.qpos[qa] = 1.2
    free_door.step(3000)
    assert free_door.d.qpos[qa] == pytest.approx(1.2, abs=0.01)
    free_door.reset()
    f = 0.0
    while free_door.d.qpos[qa] < 0.02 and f < 5:
        f += 0.01
        force_at(free_door, "obj_cart_room/door", [-f, 0, 0], bar_point, 20)
    assert f < 1.0, f                                     # the hinges: 0.2 N m / 0.83 m = 0.24 N (0.75 N measured: the ramp)


# -- the task, physically --------------------------------------------------------------------------------------------

def tow_through(world, door_open: float, steps: int = 15000):
    """The cart, handle first, 1 m in front of the doorway and centred on it, towed by a force at its handle (a speed
    servo at 0.4 m/s plus centring) until its rear end is past the wall: (steps taken, forces, door angles)."""
    world.reset()
    world.teleport_base([-3.0, 3.0, 0.0])
    g = C.door_geometry()
    world.move_object("cart", [g["near_face"] - 1.0, 0.0, 0.001], yaw=0.0)
    qa, _ = door_q(world)
    world.d.qpos[qa] = door_open
    fs, qs = [], []

    def tow(w):
        v, h = cart_speed(w), handle_point(w)
        f = np.array([min(max(60.0 * (0.4 - v[0]), 0.0), 40.0), -40.0 * h[1] - 20.0 * v[1], 0.0])
        fs.append(float(np.linalg.norm(f)))
        qs.append(float(w.d.qpos[qa]))
        return f

    b = world.m.body("obj_cart").id
    for n in range(0, steps, 100):
        force_at(world, "obj_cart", tow, handle_point, 100)
        rear = world.d.xpos[b] + world.d.xmat[b].reshape(3, 3) @ [-C.CART_L / 2, 0, 0]
        if rear[0] > g["near_face"] + C.WALL_T + 0.05:
            return n + 100, np.array(fs), np.array(qs)
    return None, np.array(fs), np.array(qs)


def test_cart_towed_by_its_handle_through_the_open_doorway(free_door):
    n, fs, qs = tow_through(free_door, math.pi / 2)
    assert n is not None and n < 6000, n                                  # through in 4.2 s
    assert np.median(fs) < 3.0, np.median(fs)                             # 2.2 N rolling at 0.4 m/s
    st = free_door.object_state("cart")
    assert abs(st["xyz"][1]) < 0.05 and abs(st["rpy"][2]) < 0.05 and np.all(np.abs(qs - math.pi / 2) < 0.01)
    assert abs(st["rpy"][0]) < 0.01 and abs(st["rpy"][1]) < 0.01


def test_closer_shuts_the_door_onto_the_cart_which_still_gets_through(world):
    """With the closer, a door let go at 90 deg swings onto the cart's side as it passes: it scrapes along and the
    cart still gets in (the robot can hold the door with its body instead)."""
    n, fs, qs = tow_through(world, math.pi / 2)
    assert n is not None, n                                               # 4.4 s measured
    assert qs.min() < 1.4 and qs[-1] > 0.1                                # it closed onto the cart, which held it
    st = world.object_state("cart")
    assert abs(st["rpy"][0]) < 0.02 and abs(st["rpy"][1]) < 0.02


# -- reset, export, the joint fields ---------------------------------------------------------------------------------

def test_reset_restores_the_door_and_the_cart(world):
    world.reset()
    st0 = world.object_state("cart")
    qa, _ = door_q(world)
    world.d.qpos[qa] = 1.0
    world.teleport_base([-3.0, 3.0, 0.0])
    force_at(world, "obj_cart", [0, 12.0, 0], handle_point, 1500)
    moved = world.object_state("cart")
    assert abs(moved["xyz"][1] - st0["xyz"][1]) > 0.1 and max(abs(v) for v in moved["joints"].values()) > 0.5
    world.reset()
    st = world.object_state("cart")
    assert st["xyz"] == pytest.approx(st0["xyz"], abs=1e-4) and st["rpy"] == pytest.approx(st0["rpy"], abs=1e-4)
    assert all(abs(v) < 1e-3 for v in st["joints"].values()), st["joints"]
    assert abs(world.d.qpos[qa]) < 1e-3 and world.base_state()["xyz"][:2] == pytest.approx([0, 0], abs=1e-3)
    world.reset(start_scene=True)
    assert abs(world.d.qpos[door_q(world)[0]]) < 1e-3 and world.object_state("cart")["xyz"][:2] == [0.0, -0.8]


def test_door_and_cart_parts_are_dynamic_bodies_in_the_rigid_scene(world):
    from robot_utils.nodes.sim import rigid
    world.reset()
    idx, _ = rigid.build(world)
    by = {b["name"]: b for b in idx["bodies"]}
    assert not by["cart_room"]["dynamic"]                                # the walls: static
    assert by["cart_room/door"]["dynamic"] and by["cart_room/door"]["joint"]["type"] == "hinge"
    assert by["cart"]["dynamic"]
    for tag in C.CASTERS:
        assert by[f"cart/swivel_{tag}"]["dynamic"] and by[f"cart/wheel_{tag}"]["dynamic"]
        assert by[f"cart/wheel_{tag}"]["joint"]["type"] == "hinge"


def test_spring_and_condim_validation(world):
    from robot_utils.nodes.sim.world import SimError
    base = {"name": "a", "geoms": [{"shape": "box"}]}

    def obj(**joint):
        return {"name": "bad", "shape": "articulated", "links": [
            base, {"name": "b", "parent": "a", "joint": {"type": "hinge", "axis": [0, 0, 1], **joint},
                   "geoms": [{"shape": "box"}]}]}

    for v in ("stiff", float("nan"), float("inf"), True, -1.0, [1.0]):
        with pytest.raises(SimError, match="stiffness"):
            world.add_object(obj(stiffness=v))
    for v in ("x", float("nan"), False):
        with pytest.raises(SimError, match="springref"):
            world.add_object(obj(stiffness=1.0, springref=v))
    with pytest.raises(SimError, match="needs a stiffness"):
        world.add_object(obj(springref=0.5))
    for c in (0, 2, 5, True, "3"):
        with pytest.raises(SimError, match="condim"):
            world.add_object({"name": "bad", "shape": "box", "condim": c})
    assert "bad" not in world.objects


def test_http_spring_and_condim_round_trip_through_save_and_load(world, tmp_path):
    from robot_utils.nodes.sim.node import SimNode
    from robot_utils.nodes.sim.server import create_app
    world.reset()
    node = SimNode(world, "cd", str(tmp_path / "scenes"))
    flap = {"name": "flap", "parent": "base", "pos": [0, 0, 0.12],
            "joint": {"type": "hinge", "axis": [0, 0, 1], "damping": 0.5, "range": [-0.5, 2.0],
                      "stiffness": 2.0, "springref": -0.5},
            "geoms": [{"shape": "box", "size": [0.4, 0.05, 0.02], "pos": [0.2, 0, 0]}]}
    obj = {"name": "flapper", "shape": "articulated", "fixed": True, "xyz": [-2, -2, 0], "condim": 3,
           "links": [{"name": "base", "geoms": [{"shape": "box", "size": [0.1, 0.1, 0.1], "pos": [0, 0, 0.05]}]},
                     flap]}

    def springs_back():
        qa, _ = world.obj_joints["flapper"]["flap"]
        world.d.qpos[qa] = 1.5
        world.step(3000)
        return float(world.d.qpos[qa])

    with TestClient(create_app(node, start=False)) as c:
        r = c.post("/scene/objects", json=obj)
        assert r.status_code == 200, r.text
        assert r.json()["condim"] == 3
        j = r.json()["links"][1]["joint"]
        assert j["stiffness"] == 2.0 and j["springref"] == -0.5
        assert springs_back() == pytest.approx(-0.5, abs=0.02)              # to its lower stop, past which it is relaxed
        for bad in ({"stiffness": -1}, {"stiffness": "x"}, {"springref": "x"}):
            o = {**obj, "name": "flap2", "links": [obj["links"][0], {**flap, "joint": {**flap["joint"], **bad}}]}
            assert c.post("/scene/objects", json=o).status_code == 422, bad
        assert c.post("/scene/objects", json={**obj, "name": "flap3", "condim": 2}).status_code == 422
        assert c.post("/scene/save", json={"name": "flapped"}).status_code == 200
        saved = (tmp_path / "scenes" / "flapped.yaml").read_text()
        assert "stiffness: 2.0" in saved and "springref: -0.5" in saved and "condim: 3" in saved
        assert c.post("/scene/load", json={"name": "flapped"}).status_code == 200
        assert world.objects["flapper"]["condim"] == 3 and world.objects["cart"]["condim"] == C.CART_CONDIM
        assert springs_back() == pytest.approx(-0.5, abs=0.02)
        g = world.m.body("obj_flapper").id
        assert all(world.m.geom_condim[k] == 3 for k in range(world.m.ngeom) if world.m.geom_bodyid[k] == g)
        assert c.delete("/scene/objects/flapper").status_code == 200
    world.reset(start_scene=True)


def test_the_door_has_a_pull_bar_in_the_arms_band_and_one_at_standard_height():
    door = next(l for l in C.cart_room()["links"] if l["name"] == "door")
    bars = sorted(g["pos"][2] for g in door["geoms"]
                  if g["shape"] == "cylinder" and g["radius"] == C.PULL_BAR_R and g["length"] == C.PULL_BAR_L)
    assert bars == [C.PULL_BAR_Z, C.PULL_BAR_Z_STD] == [0.60, 1.05]
    assert C.PULL_BAR_Z + C.PULL_BAR_L / 2 < C.PULL_BAR_Z_STD - C.PULL_BAR_L / 2      # apart, not one long bar
