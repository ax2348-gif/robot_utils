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
    cfg = load_config(overrides={"time": {"run": False}, **over})
    return World(urdf, assets, cfg, {"source": "test", "model_version": info.get("model_version")})


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
