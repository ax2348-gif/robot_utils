"""Shared helpers of the sim tests: the model bundle (built once) and a GL check for rendering."""
import functools

import pytest

mujoco = pytest.importorskip("mujoco")


@functools.lru_cache(maxsize=None)
def bundle(variant: str = "full") -> bytes:
    from robot_utils.nodes.sim.node import fetch_bundle
    return fetch_bundle("local", variant)[0]


def make_world(variant: str = "full", **over):
    from robot_utils.nodes.sim.world import World, load_config, unzip_bundle
    urdf, assets, info = unzip_bundle(bundle(variant))
    over.setdefault("scene", {"tabletop": False})                 # an empty world unless a test asks
    # stepped worlds: the robot's watchdogs on every host (deterministic); real-time ones: the host's default, as
    # the stack runs (lenient on macOS, where scheduling stalls trip the 0.1 s watchdog)
    over.setdefault("timing", {"lenient": "auto" if (over.get("time") or {}).get("run") else False})
    cfg = load_config(overrides={"time": {"run": False}, **over})
    return World(urdf, assets, cfg, {"source": "test", "model_version": info.get("model_version")})


def robot_footprint(world) -> tuple:
    """(x0, x1, y0, y1): the robot's extent seen from above at its start pose (base at the origin facing +x, the arm
    at its start pose), from every robot geom's box."""
    import numpy as np
    m, d = world.m, world.d
    pts = []
    for g in range(m.ngeom):
        name = m.body(m.geom_bodyid[g]).name
        if name == "world" or name.startswith("obj_"):
            continue
        c, half = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + c
        pts.append(corners @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    p = np.concatenate(pts)
    return float(p[:, 0].min()), float(p[:, 0].max()), float(p[:, 1].min()), float(p[:, 1].max())


@functools.lru_cache(maxsize=None)
def gl_available() -> bool:
    try:
        m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
        r = mujoco.Renderer(m, 8, 8)
        r.render()
        r.close()
        return True
    except Exception:  # noqa: BLE001 - no GL context here (e.g. a headless Linux without EGL/OSMesa)
        return False


needs_gl = pytest.mark.skipif("not __import__('sim_helpers').gl_available()",
                              reason="no OpenGL context for offscreen rendering")
