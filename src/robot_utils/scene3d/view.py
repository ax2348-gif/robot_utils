"""What a viewer draws of a rigid scene (docs/scene3d.md): the one geometry rule the headset uses
(``scene_view.h``, ``SceneView::rebuild``), shared by the offscreen replay (``replay.render``) and the episode
browser (``collect.browse``).

``role``: ``visual`` or ``collision`` picks those geoms (``both`` counts as either) for every body; ``auto`` (the
headset's default) draws the robot's and the ghosts' visual geoms and everything else's collision geoms, so the
operator sees what the physics touches. A body without any geom of the wanted role keeps all of its geoms. Planes
are never drawn.
"""
from __future__ import annotations

ROBOT_LIKE = ("robot", "ghost")


def _wants(geom_role: str, want: str) -> bool:
    return geom_role == want or geom_role == "both"


def wanted_role(body: dict, role: str) -> str:
    return role if role in ("visual", "collision") else ("visual" if body.get("kind") in ROBOT_LIKE else "collision")


def drawn_geoms(body: dict, role: str = "auto") -> list[int]:
    """Indices into ``body["geoms"]`` of the geoms drawn under ``role``."""
    want = wanted_role(body, role)
    geoms = body.get("geoms") or []
    have = any(_wants(g.get("role", "both"), want) for g in geoms)
    return [k for k, g in enumerate(geoms)
            if g.get("type") != "plane" and (not have or _wants(g.get("role", "both"), want))]


def select_geoms(body: dict, role: str = "auto") -> list[dict]:
    """The geoms of a body drawn under ``role`` (``drawn_geoms``, as dicts)."""
    geoms = body.get("geoms") or []
    return [geoms[k] for k in drawn_geoms(body, role)]
