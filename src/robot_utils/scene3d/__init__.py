"""Rigid scenes for remote viewers (docs/scene3d.md): any backend's world as rigid bodies with geometry, sent once,
plus every moving body's world pose, streamed. Backend-agnostic: the sim's MuJoCo adapter is one producer
(``robot_utils.nodes.sim.rigid``); the Quest's AR view is the main consumer."""
from .format import (FRAME_MAGIC, SCHEMA, Frame, SceneBuilder, decode_frame, encode_frame, outline_edges,
                     scene_hash)

__all__ = ["FRAME_MAGIC", "SCHEMA", "Frame", "SceneBuilder", "decode_frame", "encode_frame", "outline_edges",
           "scene_hash"]
