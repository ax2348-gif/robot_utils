"""HTTP API of the sim node (FastAPI). Guide: docs/sim.md. No lease, no e-stop (it substitutes for hardware)."""
from __future__ import annotations

import asyncio
import gzip
import json
import time
from contextlib import asynccontextmanager
from typing import Any, Literal, Optional

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from ..common.server import Event, add_base_routes, add_page_routes, node_meta, setup_app, sse
from ...scene3d import compat as s3compat
from ...scene3d import encode_frame
from . import __version__, rigid, viz
from .node import SimNode
from .world import SimError

TAGS = [
    {"name": "time", "description": "Run, pause, step, real-time factor."},
    {"name": "scene", "description": "Objects in the world: add, move, remove, reset, save, load."},
    {"name": "physics", "description": "**Privileged, sim-only** controls (never exposed by a hardware node): "
                                       "apply a wrench, teleport the base."},
    {"name": "cameras", "description": "Rendered camera images (previews; the camera nodes stream them)."},
    {"name": "view", "description": "The 3D view's data: the world's geometry, body poses (~30 Hz stream)."},
    {"name": "markers", "description": "Frames, points, lines and labels any client posts to be drawn in the 3D "
                                       "view (e.g. the WBC's end-effector targets and commanded poses)."},
    {"name": "state", "description": "Status, clock sync, events."},
]


class _In(BaseModel):
    model_config = ConfigDict(extra="forbid")


class RunIn(_In):
    rtf: Optional[float] = Field(None, gt=0, le=100, description="Real-time factor (1 = real time); unchanged if absent.")


class StepIn(_In):
    n: Optional[int] = Field(None, ge=1, le=1_000_000, description="Physics steps (1 ms each by default).")
    seconds: Optional[float] = Field(None, gt=0, le=600, description="Or sim seconds to advance.")


class PartIn(_In):
    shape: Literal["box", "cylinder"] = Field("box", description="box or cylinder (axis z).")
    size: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="m, box full extents x, y, z.")
    radius: Optional[float] = Field(None, gt=0, description="m, cylinder radius.")
    length: Optional[float] = Field(None, gt=0, description="m, cylinder length.")
    pos: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3, description="m, in the link frame.")
    rpy: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3, description="rad, in the link frame.")
    mass: float = Field(0.2, gt=0, description="kg.")
    rgba: Optional[list[float]] = Field(None, min_length=4, max_length=4, description="Colour (default: the object's).")


class DriveIn(_In):
    velocity: float = Field(description="rad/s (hinge) or m/s (slide), held by a velocity servo.")
    max_torque: float = Field(gt=0, description="N m (hinge) or N (slide): the servo's cap.")
    gain: Optional[float] = Field(None, gt=0, description="N m s/rad or N s/m of torque per unit of speed error "
                                                          "(default 1000).")


class RatchetIn(_In):
    direction: Literal[1, -1] = Field(description="The way it turns (slides) freely, along its axis: 1 or -1.")
    max_torque: float = Field(gt=0, description="N m (hinge) or N (slide): the most it holds against the other way.")
    gain: Optional[float] = Field(None, gt=0, description="N m s/rad or N s/m of hold per unit of reverse speed "
                                                          "(default 1e5: 100 N m against it creeps 1 mrad/s).")


class JointIn(_In):
    type: Literal["slide", "hinge"] = Field(description="slide (m along axis) or hinge (rad about axis).")
    axis: list[float] = Field([1.0, 0.0, 0.0], min_length=3, max_length=3, description="In the link frame.")
    range: Optional[list[float]] = Field(None, min_length=2, max_length=2, description="[lo, hi]; absent: unlimited.")
    pos: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3, description="m, the hinge axis' point.")
    damping: float = Field(2.0, ge=0, description="N s/m or N m s/rad.")
    frictionloss: float = Field(0.0, ge=0, description="N or N m: dry friction holding it in place.")
    drive: Optional[DriveIn] = Field(None, description="A motor holding the joint at a velocity (e.g. a turntable); "
                                                       "absent: passive.")
    ratchet: Optional[RatchetIn] = Field(None, description="A one-way clutch (e.g. a revolving door's anti-reverse); "
                                                           "not with a drive.")
    qvel0: Optional[float] = Field(None, allow_inf_nan=False,
                                   description="rad/s or m/s: its speed when the object is loaded and after every "
                                               "reset (e.g. a door given a spin); absent: at rest.")
    stiffness: Optional[float] = Field(None, ge=0, allow_inf_nan=False,
                                       description="N m/rad or N/m: a spring pulling the joint toward springref (e.g. "
                                                   "a door closer); absent: none.")
    springref: Optional[float] = Field(None, allow_inf_nan=False,
                                       description="rad or m: where the spring is relaxed (default 0; beyond the "
                                                   "range for a preload). Needs a stiffness.")
    mimic: Optional[str] = Field(None, description="An earlier link (same joint type) whose joint position this one "
                                                   "is held at: parts that move as one without sharing a body.")


class LinkIn(_In):
    name: str = Field(description="Unique in the object: letters, digits, _ or -.")
    parent: Optional[str] = Field(None, description="An earlier link; absent for the first (root) link only.")
    pos: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3, description="m, in the parent's frame.")
    rpy: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3, description="rad, in the parent's frame.")
    joint: Optional[JointIn] = Field(None, description="How it moves against its parent (absent: welded).")
    geoms: list[PartIn] = Field(min_length=1, description="Its primitives (they collide and are drawn).")


class ObjectIn(_In):
    name: str = Field(description="Unique name: letters, digits, _ or -.")
    shape: Literal["box", "cylinder", "articulated"] = Field(
        "box", description="box or cylinder (axis z), or articulated (`links`: a tree of parts joined by slide / "
                           "hinge joints, e.g. a cabinet with drawers).")
    links: Optional[list[LinkIn]] = Field(None, description="articulated: the links, root first.")
    size: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="m, box full extents x, y, z.")
    radius: Optional[float] = Field(None, gt=0, description="m, cylinder radius.")
    length: Optional[float] = Field(None, gt=0, description="m, cylinder length.")
    mass: float = Field(0.5, gt=0, description="kg (ignored when fixed).")
    fixed: bool = Field(False, description="Welded to the world (furniture): never moves, no mass.")
    xyz: list[float] = Field([0.5, 0.0, 0.2], min_length=3, max_length=3, description="m, world position of its centre.")
    rpy: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="rad, orientation (URDF rpy).")
    yaw: Optional[float] = Field(None, description="rad, heading (when rpy is absent).")
    rgba: Optional[list[float]] = Field(None, min_length=4, max_length=4, description="Colour, 0..1.")
    condim: Optional[Literal[1, 3, 4, 6]] = Field(None, description="Its contacts' dimensions (MuJoCo condim; the "
                                                                  "larger of the pair's applies); absent: "
                                                                  "contact.objects.condim (4).")


class MoveIn(_In):
    xyz: list[float] = Field(min_length=3, max_length=3, description="m, world position.")
    rpy: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="rad.")
    yaw: Optional[float] = Field(None, description="rad (when rpy is absent).")


class SceneIn(_In):
    name: Optional[str] = Field(None, description="A saved scene's name (POST /scene/save).")
    objects: Optional[list[ObjectIn]] = Field(None, description="Or the objects themselves.")


class ResetIn(_In):
    start: bool = Field(False, description="Reload the scene the world started with (the default tabletop and the "
                                           "config's objects) instead of putting the current objects back.")


class TaskIn(_In):
    name: str = Field(description="A task from GET /scene/tasks.")


class TasksOut(BaseModel):
    tasks: list[dict] = Field(description="[{name, title}]: the scenes an operator picks from.")
    current: Optional[str] = Field(None, description="The task last switched to (null: the start scene).")


class TaskOut(BaseModel):
    task: str = Field(description="The task now loaded.")
    objects: int = Field(description="Objects in the scene.")
    view: dict = Field(description="The headset view now ({style, role, anchor}).")


class SavedOut(BaseModel):
    names: list[str] = Field(description="Saved scenes, newest first (POST /scene/load {name}).")


class SaveIn(_In):
    name: str = Field(description="Scene name: letters, digits, - and _.")


class WrenchIn(_In):
    body: str = Field(description="`base`, an object's name, or a model body (link) name.")
    force: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3, description="N, world frame.")
    torque: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3, description="N m, world frame.")
    duration: float = Field(0.1, gt=0, le=60, description="s of sim time.")


class TeleportIn(_In):
    xyz: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="m, base_link in the world.")
    yaw: Optional[float] = Field(None, description="rad.")
    rpy: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="rad (instead of yaw).")


class MarkerIn(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(description="Unique name (letters, digits, _ . : / -); posting it again replaces it.")
    kind: Literal["frame", "point", "line", "text"] = Field(
        "frame", description="frame: axes at pos + quat; point: a dot at pos; line: a polyline through points; "
                             "text: a label at pos.")
    frame: str = Field("world", description="Coordinates are in: `world`, `base` (base_link) or a model body / "
                                            "object name (e.g. `arm_end_link`).")
    pos: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="m (frame, point, text).")
    quat: Optional[list[float]] = Field(None, min_length=4, max_length=4,
                                        description="Orientation [x, y, z, w] (frame); identity if absent.")
    points: Optional[list[list[float]]] = Field(None, description="m, [[x, y, z], ...] (line).")
    color: Optional[str] = Field(None, max_length=40, description="CSS colour, e.g. `#e0a030`; default: the "
                                                                  "theme's signal colour.")
    size: Optional[float] = Field(None, gt=0, le=5, description="m: axis length (frame, default 0.1), dot "
                                                                "diameter (point, default 0.02).")
    label: Optional[str] = Field(None, max_length=80, description="Text drawn next to it (the text of `text`).")
    ttl: Optional[float] = Field(None, gt=0, le=86400, description="s: removed unless posted again within this "
                                                                   "(a live target: post at 10 Hz with ttl 0.5).")


class MarkersIn(_In):
    markers: list[MarkerIn] = Field(description="Markers to add or replace (by name).")
    source: Optional[str] = Field(None, max_length=40, description="Who posts them (e.g. `wbc`), for "
                                                                    "`DELETE /markers?source=` and `replace`.")
    replace: bool = Field(False, description="Remove this source's other markers first.")


# -- replies --------------------------------------------------------------------------------------------

class BaseState(BaseModel):
    xyz: list[float] = Field(description="m, base_link in the world (z 0 = on the floor).")
    rpy: list[float] = Field(description="rad.")
    twist: list[float] = Field(description="Body twist vx, vy (m/s, base frame), omega (rad/s).")


class ModuleState(BaseModel):
    name: str = Field(description="FL | FR | RR | RL.")
    steer_on: bool = Field(description="Its steering SPARK is enabled and commanding.")
    drive_on: bool = Field(description="Its drive SPARK is enabled and commanding.")
    steer_goal: float = Field(description="rad, commanded module angle.")
    speed_goal: float = Field(description="m/s, commanded wheel speed.")
    steer: float = Field(description="rad, module angle (0 = forward).")
    steer_rate: float = Field(description="rad/s.")
    wheel: float = Field(description="rad, wheel angle.")
    speed: float = Field(description="m/s, wheel surface speed.")


class ObjectState(BaseModel):
    name: str = Field(description="Object name.")
    shape: str = Field(description="box | cylinder | articulated.")
    size: Optional[list[float]] = Field(None, description="m, box extents.")
    radius: Optional[float] = Field(None, description="m.")
    length: Optional[float] = Field(None, description="m.")
    mass: float = Field(description="kg.")
    fixed: Optional[bool] = Field(None, description="Welded to the world (present only when true).")
    rgba: Optional[list[float]] = Field(None, description="Colour, 0..1.")
    condim: Optional[int] = Field(None, description="Its contacts' dimensions, when set.")
    links: Optional[list[dict]] = Field(None, description="articulated: the links as given (defaults filled in).")
    joints: Optional[dict[str, float]] = Field(None, description="articulated: each jointed link's position (m | rad).")
    xyz: list[float] = Field(description="m.")
    rpy: list[float] = Field(description="rad.")
    speed: float = Field(description="m/s.")


class ArmState(BaseModel):
    enabled: bool = Field(description="Arm motors on (the arm backend enabled them).")
    gripper_enabled: bool = Field(description="Gripper motor on.")
    fault: int = Field(description="Damiao status code of a fault (0xD = CAN watchdog: no frame for 0.1 s); 0 if none.")
    q: list[float] = Field(description="rad, joints 1..6.")
    q_ref: list[float] = Field(description="rad, the kinematic reference the servo holds.")
    gap_mm: float = Field(description="mm, jaw gap.")
    frames: int = Field(description="MIT frame batches received.")
    gravity_fit: Optional[dict] = Field(None, description="Dynamic arm: the link masses fitted to the node's calibrated "
                                                       "gravity model (g x tau_scale), with the residuals (N m).")


class CameraState(BaseModel):
    active: bool = Field(description="Rendering (a backend asked within 2 s).")
    streams: list[str] = Field(description="Streams being rendered.")
    rendered: dict[str, int] = Field(description="Frames rendered per stream.")
    ms_per_frame: dict[str, Optional[float]] = Field(description="Mean render + encode ms per frame.")


class BackendState(BaseModel):
    socket: str = Field(description="Unix socket path.")
    clients: int = Field(description="Connected backends.")
    requests: int = Field(description="Requests served.")


class ModelSource(BaseModel):
    source: Optional[str] = Field(description="Where the model came from.")
    model_version: Optional[str] = Field(description="The description node's model_version.")
    camera_serials: dict[str, str] = Field({}, description="Serial each sim camera reports: the robot's (from the description snapshot) or the sim config's.")
    camera_mounts: dict[str, Any] = Field({}, description="Per camera, what its mount came from at model load (the description's `from`: nominal, ground / yaw calibration ids).")


class Fidelity(BaseModel):
    base: str = Field(description="The base's model.")
    arm: str = Field(description="kinematic (dynamic: not built yet).")


class SimState(BaseModel):
    state: Literal["running", "paused"] = Field(description="running (real time x rtf) or paused.")
    error: Optional[str] = Field(description="The last rendering error; null if fine.")
    model: ModelSource
    fidelity: Fidelity
    limits: dict[str, Any] = Field(description="The actuator limits, from the robot config (docs/sim.md, \"Limits come "
                                               "from the robot config\"): wheel / steer speed, acceleration, torque; "
                                               "arm joint speeds; finger force.")
    t: float = Field(description="s, sim clock.")
    running: bool = Field(description="Real-time mode.")
    rtf: float = Field(description="Target real-time factor.")
    reset_seq: int = Field(description="Resets since start (`POST /scene/reset`), as in every robot_state.v1 record.")
    rtf_measured: float = Field(description="Achieved real-time factor over the last second.")
    step_rate: float = Field(description="Physics steps per wall second over the last second.")
    steps: int = Field(description="Physics steps since start.")
    behind: int = Field(description="Times the physics fell behind real time and dropped its backlog.")
    base: BaseState
    modules: list[ModuleState] = Field(description="Swerve modules.")
    objects: list[ObjectState] = Field(description="Scene objects.")
    arm: Optional[ArmState] = Field(None, description="The arm (absent without one).")
    cameras: dict[str, CameraState] = Field(description="Rendered cameras by name.")
    backends: dict[str, BackendState] = Field(description="Backend sockets: arm, swerve, camera-<name>.")
    view: Optional[dict] = Field(None, description="How headsets draw /scene3d: {style, role} (GET /scene3d/view).")
    timing: Optional[dict] = Field(None, description="The watchdogs enforced: {lenient, why, arm_watchdog, "
                                                  "swerve_heartbeat} (s); lenient on macOS by default.")
    events: list[Event] = Field(description="Latest events, oldest first.")


class TimeOut(BaseModel):
    t: float = Field(description="s, sim clock.")
    running: bool = Field(description="Real-time mode.")
    rtf: float = Field(description="Target real-time factor.")
    reset_seq: int = Field(description="Resets since start (`POST /scene/reset` increments it).")


class SceneOut(BaseModel):
    objects: list[ObjectState] = Field(description="Every object with its current pose.")


class SavedScene(BaseModel):
    name: str = Field(description="Scene name.")
    path: str = Field(description="File it was written to.")
    objects: list[dict] = Field(description="The objects as saved (ObjectIn fields).")


class Marker(MarkerIn):
    model_config = ConfigDict(extra="forbid")
    source: Optional[str] = Field(None, description="Who posted it.")


class MarkersOut(BaseModel):
    markers: list[Marker] = Field(description="Every live marker.")


class MarkersPut(BaseModel):
    names: list[str] = Field(description="The markers just posted.")
    count: int = Field(description="Live markers in all.")


class MarkersRemoved(BaseModel):
    removed: int = Field(description="Markers removed.")


class VizBody(BaseModel):
    name: str = Field(description="Body name (an object's without its `obj_` prefix).")
    body: str = Field(description="The model body's name.")
    kind: Literal["world", "robot", "object"] = Field(description="What it belongs to.")
    parent: int = Field(description="Parent body index.")


class VizGeom(BaseModel):
    body: int = Field(description="Body index.")
    type: Literal["box", "sphere", "capsule", "cylinder", "ellipsoid", "mesh"] = Field(
        description="MuJoCo geom type; sizes as MuJoCo's (half extents, radius + half length along z).")
    size: list[float] = Field(description="m, MuJoCo geom_size.")
    pos: list[float] = Field(description="m, in the body.")
    quat: list[float] = Field(description="w, x, y, z in the body.")
    rgba: list[float] = Field(description="Colour 0..1.")
    visual: bool = Field(description="A visual geom (no contacts).")
    mesh: Optional[int] = Field(None, description="Index into meshes (type mesh).")


class VizMesh(BaseModel):
    name: str = Field(description="Mesh name.")
    nvert: int = Field(description="Vertices (float32 x, y, z).")
    nface: int = Field(description="Triangles.")
    vert_offset: int = Field(description="Byte offset of the vertices in /viz/meshes.bin.")
    face_offset: int = Field(description="Byte offset of the triangle indices.")
    index: Literal["u16", "u32"] = Field(description="Index type (little endian).")


class VizModel(BaseModel):
    generation: int = Field(description="Bumps when the model recompiles (objects added / removed): fetch again.")
    model_version: Optional[str] = Field(description="The description node's model_version.")
    camera_serials: dict[str, str] = Field({}, description="Serial each sim camera reports: the robot's (from the description snapshot) or the sim config's.")
    camera_mounts: dict[str, Any] = Field({}, description="Per camera, what its mount came from at model load (the description's `from`: nominal, ground / yaw calibration ids).")
    cell: float = Field(description="m, mesh simplification cell (0 = as compiled).")
    bodies: list[VizBody] = Field(description="Bodies by index (pose stream order).")
    geoms: list[VizGeom] = Field(description="Drawable geoms.")
    meshes: list[VizMesh] = Field(description="Meshes in /viz/meshes.bin.")
    blob_bytes: int = Field(description="Size of /viz/meshes.bin.")
    base_body: str = Field(description="The robot's base body.")


class VizBase(BaseModel):
    xyz: list[float] = Field(description="m, base_link in the world.")
    yaw: float = Field(description="rad.")
    twist: list[float] = Field(description="vx, vy (m/s, base frame), omega (rad/s).")


class VizFrame(BaseModel):
    t: float = Field(description="s, sim clock.")
    generation: int = Field(description="Model generation the poses index (refetch /viz/model when it changes).")
    running: bool = Field(description="Real-time mode.")
    rtf: float = Field(description="Target real-time factor.")
    rtf_measured: float = Field(description="Achieved real-time factor.")
    pos: list[float] = Field(description="m, every body's world position, 3 per body.")
    quat: list[float] = Field(description="Every body's world orientation, 4 per body (w, x, y, z).")
    base: VizBase
    markers_version: int = Field(description="Bumps when markers change.")
    markers: Optional[list[Marker]] = Field(None, description="Every live marker (in the stream: only when changed).")


class ViewIn(_In):
    style: Optional[Literal["hidden_line", "wire", "flat_edges", "flat"]] = Field(
        None, description="hidden_line: surfaces hide the lines behind them (see-through to passthrough); wire: every "
                          "line; flat: shaded surfaces; flat_edges: both.")
    role: Optional[Literal["auto", "visual", "collision"]] = Field(
        None, description="auto: the robot's visual meshes (the URDF's), everything else's collision geometry (what the "
                          "physics touches); or one role everywhere.")
    anchor: Optional[list[float]] = Field(None, min_length=3, max_length=3,
                                          description="[x, y, yaw_deg] in the sim world: placed in front of the operator.")
    clear_anchor: bool = Field(False, description="Back to the world origin.")


class ViewOut(BaseModel):
    style: str = Field(description="hidden_line | wire | flat_edges | flat.")
    role: str = Field(description="auto | visual | collision.")
    anchor: Optional[list[float]] = Field(None, description="[x, y, yaw_deg]: the sim-world point the headset places in "
                                                            "front of the operator; null: the world origin.")


class GhostIn(_In):
    name: str = Field(description="Ghost name (posting it again replaces it).")
    body: str = Field(description="The model body it copies, e.g. `arm_end_link` (the gripper with `subtree`).")
    subtree: bool = Field(True, description="The body's whole subtree (fingers included), or the body alone.")
    pose: Optional[list[float]] = Field(None, min_length=7, max_length=7,
                                        description="The body's goal in the sim world [x, y, z, qx, qy, qz, qw].")
    relative: Optional[list[float]] = Field(None, min_length=7, max_length=7,
                                            description="Or the goal in the body's own frame now (drift-free when the "
                                                        "poster's world is an estimate).")
    rgba: list[float] = Field([0.3, 0.9, 0.5, 0.4], min_length=4, max_length=4,
                              description="Its colour, alpha < 1 see-through (the viewer draws it as given).")
    ttl: Optional[float] = Field(None, gt=0, description="s: hidden unless posted again within it (null: until "
                                                         "removed).")


class GhostsIn(_In):
    ghosts: list[GhostIn] = Field(description="Ghosts to add or replace (one bad ghost refuses the post).")
    source: Optional[str] = Field(None, description="Who posts them (for replace / DELETE ?source=).")
    replace: bool = Field(False, description="Remove the source's other ghosts first.")


class GhostsOut(BaseModel):
    ghosts: list[dict] = Field(description="Every ghost: name, body, subtree, pose | relative, rgba, ttl, source, "
                                           "age (s since posted), live.")


class GhostsPut(BaseModel):
    names: list[str] = Field(description="Ghosts posted.")
    count: int = Field(description="Ghosts now.")


class S3Geom(BaseModel):
    model_config = ConfigDict(extra="forbid")
    type: Literal["box", "sphere", "capsule", "cylinder", "ellipsoid", "plane", "mesh"] = Field(description="Shape.")
    pos: list[float] = Field(description="m, in the body.")
    quat: list[float] = Field(description="x, y, z, w in the body.")
    rgba: list[float] = Field(description="Colour 0..1.")
    role: Literal["collision", "visual", "both"] = Field(description="collision: what the physics touches; visual: "
                                                                     "drawn only; both.")
    half_extents: Optional[list[float]] = Field(None, description="m, box (3) or plane (2; 0 = unbounded).")
    radius: Optional[float] = Field(None, description="m, sphere / capsule / cylinder.")
    half_length: Optional[float] = Field(None, description="m, capsule / cylinder straight part along z, half.")
    radii: Optional[list[float]] = Field(None, description="m, ellipsoid.")
    mesh: Optional[int] = Field(None, description="Index into meshes.")


class S3Body(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: int = Field(description="Body id (index).")
    name: str = Field(description="Name (an object's without `obj_`; an articulated part `object/link`).")
    kind: Literal["world", "robot", "object", "ghost"] = Field(description="What it belongs to (ghost: a copy at a "
                                                                          "goal, POST /ghosts).")
    parent: int = Field(description="Parent body id (-1: none). Informational: place bodies by their own poses.")
    dynamic: bool = Field(description="Moves (streamed); false: its `pose` is fixed.")
    pose: list[float] = Field(description="World pose [x, y, z, qx, qy, qz, qw] when the scene was made.")
    geoms: list[S3Geom]
    joint: Optional[dict] = Field(None, description="{name, type: hinge | slide, range?}: its joint to its parent.")


class S3Mesh(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(description="Mesh name (`.hull`: MuJoCo's convex hull of the mesh, what it collides as).")
    nvert: int = Field(description="Vertices (float32 x, y, z).")
    ntri: int = Field(description="Triangles.")
    nedge: int = Field(description="Outline edges (vertex pairs: boundary, non-manifold, crease).")
    vert_offset: int = Field(description="Byte offset in the blob.")
    tri_offset: int = Field(description="Byte offset of the triangles.")
    edge_offset: int = Field(description="Byte offset of the edges.")
    index: Literal["u16", "u32"] = Field(description="Index type (little endian).")
    bounds: list[list[float]] = Field(description="[[min x, y, z], [max x, y, z]] in the mesh frame.")
    error: Optional[float] = Field(description="m, how far its vertices may be from the model's after "
                                               "simplification (null: unknown).")


class S3Scene(BaseModel):
    model_config = ConfigDict(extra="forbid")
    schema_: str = Field(alias="schema", serialization_alias="schema", description="rigid_scene.v1.")
    scene_id: str = Field(description="16 hex digits over the index and blob: equal scenes, equal ids (cache key).")
    source: str = Field(description="Who made it (sim).")
    conventions: dict = Field(description="{up: +z, units: m, quat: xyzw}.")
    crease_deg: float = Field(description="Outline edges: faces more than this apart.")
    bodies: list[S3Body]
    dynamic: list[int] = Field(description="Body ids in the pose stream's order.")
    meshes: list[S3Mesh]
    blob_bytes: int = Field(description="Size of /scene3d/blob.")
    generation: int = Field(description="The sim model's generation.")
    cell: float = Field(description="m, visual mesh simplification (vertex clustering).")
    model_version: Optional[str] = Field(description="The description's model_version.")
    ghosts: list[dict] = Field(description="Per declared ghost: {name, body, subtree, bodies (scene ids), sources "
                                           "(the bodies they copy)}.")


class S3Frame(BaseModel):
    seq: int = Field(description="Frame counter (per stream).")
    t: float = Field(description="s, sim time.")
    reset_seq: int = Field(description="The world's resets since start.")
    scene_id: str = Field(description="The scene these poses index (fetch /scene3d again on a change).")
    poses: list[list[float]] = Field(description="Per dynamic body, in `dynamic` order: [x, y, z, qx, qy, qz, qw].")
    colors: Optional[dict[str, list[float]]] = Field(None, description="Slot (index in `dynamic`) -> rgba 0..1: this "
                                                                       "frame's colour for that body (alpha 0: hidden).")


class S3CompatModel(BaseModel):
    model_version: str = Field(description="The scene_id (the headset refetches the model when it changes).")
    root: str = Field(description="world.")
    links: dict = Field(description="{world: {visuals: []}}: every body comes in the state lines.")
    joints: list = Field(description="None.")


class StateObject(BaseModel):
    pose: list[float] = Field(description="World pose [x, y, z, qx, qy, qz, qw] of its centre.")
    geom: dict | list[dict] = Field(description="{type: box | cylinder, size (box full extents) | radius + length "
                                                "(along z), rgba, origin?}; a list for an articulated object's link "
                                                "(each part at its origin in the link frame).")
    fixed: bool = Field(description="Welded to the world (furniture).")


class RobotStateOverlays(BaseModel):
    model_config = ConfigDict(extra="allow")
    markers: list[Marker] = Field(description="Every live marker (POST /markers): the WBC's targets, commanded poses...")
    sim: dict = Field(description="{running, rtf}.")
    arm: Optional[dict] = Field(None, description="{enabled, fault}: the arm motors as the world sees them.")


class RobotState(BaseModel):
    schema_: str = Field(alias="schema", serialization_alias="schema", description="robot_state.v1.")
    source: str = Field(description="Who publishes it: sim (the robot: later).")
    seq: int = Field(description="Record counter (per server).")
    t: float = Field(description="s, the publisher's clock (`clock`): sim time here.")
    clock: str = Field(description="What `t` is: sim (sim seconds) | monotonic (the Pi's clock, on the robot).")
    reset_seq: int = Field(description="Resets of the world since start (`POST /scene/reset`): the headset re-anchors "
                                       "its AR scene when it changes.")
    t_host: float = Field(description="s, the server's monotonic clock when the record was made.")
    model_version: Optional[str] = Field(description="The description node's model_version of the model simulated "
                                                     "(GET /model/tree?... gives that tree).")
    frame: str = Field(description="Frame of `base` and object poses: world (z up, m).")
    base: list[float] = Field(description="base_link's world pose [x, y, z, qx, qy, qz, qw].")
    joints: dict[str, float] = Field(description="Every movable joint by its model name: rad or m (arm joints, "
                                                 "fingers, each module's steer and wheel).")
    objects: dict[str, StateObject] = Field(description="Scene objects (not the robot); an articulated object as one "
                                                        "entry per link, `object/link`.")
    overlays: RobotStateOverlays


class WrenchOut(BaseModel):
    body: str = Field(description="The model body the wrench acts on.")
    until: float = Field(description="s, sim time it ends.")


def create_app(node: SimNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        node.close()

    app = FastAPI(
        title="Sim world node",
        description=(
            "The MuJoCo sim world (docs/sim.md): the robot model from the description node, the floor and "
            "scene objects, rendered cameras. The arm, swerve and camera nodes attach to it through their "
            "hardware backends over Unix sockets; **no lease, no e-stop** here (use the nodes').\n\n"
            "**Time**: `POST /run {rtf}`, `/pause`, `/step {n | seconds}`. **Scene**: `GET /scene`, "
            "`POST /scene/objects`, `PUT|DELETE /scene/objects/{name}`, `POST /scene/reset`, `/scene/save`, "
            "`/scene/load`. **Privileged (sim-only)**: `POST /physics/wrench`, `/physics/teleport`."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)
    add_base_routes(app, lambda: {"node": "sim", "state": "running" if node.world.running else "paused"})
    add_page_routes(app, node_meta("sim", "Sim world", __version__, "sim", actuated=False))

    def bad(e: Exception, code: int = 400):
        return HTTPException(code, str(e))

    async def call(fn, *a, **kw):
        try:
            return await asyncio.to_thread(fn, *a, **kw)
        except KeyError as e:
            raise bad(e.args[0] if e.args else e, 404)
        except (SimError, ValueError) as e:
            raise bad(e)

    def time_out():
        w = node.world
        node.hub.state_changed()
        return {"t": w.t, "running": w.running, "rtf": w.rtf, "reset_seq": w.reset_seq}

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", response_model=SimState, response_model_exclude_unset=True, tags=["state"],
             summary="Sim clock, base, modules, arm, objects, cameras, backends")
    async def get_state():
        return await asyncio.to_thread(node.state)

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}}, "description": "Each `data:` line is /state."}})
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    # -- time ----------------------------------------------------------------------------------------

    @app.post("/run", response_model=TimeOut, tags=["time"], summary="Run in real time (x rtf)")
    async def run(body: Optional[RunIn] = None):
        await call(node.world.run, body.rtf if body else None)
        node.events.add(f"running (rtf {node.world.rtf:g})")
        return time_out()

    @app.post("/pause", response_model=TimeOut, tags=["time"], summary="Pause the sim clock")
    async def pause():
        node.world.pause()
        node.events.add("paused")
        return time_out()

    @app.post("/step", response_model=TimeOut, tags=["time"], summary="Advance n steps or a time (meant for paused)")
    async def step(body: StepIn):
        if body.seconds is not None:
            await call(node.world.step_for, body.seconds)
        else:
            await call(node.world.step, body.n or 1)
        return time_out()

    # -- scene ---------------------------------------------------------------------------------------

    @app.get("/scene", response_model=SceneOut, response_model_exclude_unset=True, tags=["scene"],
             summary="Objects and their poses")
    async def scene():
        return node.world.scene()

    @app.post("/scene/objects", response_model=ObjectState, response_model_exclude_unset=True, tags=["scene"],
              summary="Add an object (box, cylinder or articulated)")
    async def add_object(body: ObjectIn):
        out = await call(node.world.add_object, body.model_dump(exclude_none=True))
        node.events.add(f"object {body.name!r} added")
        return out

    @app.put("/scene/objects/{name}", response_model=ObjectState, response_model_exclude_unset=True,
             tags=["scene"], summary="Move an object (teleport: velocity zeroed)")
    async def move_object(name: str, body: MoveIn):
        return await call(node.world.move_object, name, body.xyz, body.rpy, body.yaw)

    @app.delete("/scene/objects/{name}", response_model=SceneOut, response_model_exclude_unset=True,
                tags=["scene"], summary="Remove an object")
    async def remove_object(name: str):
        await call(node.world.remove_object, name)
        node.events.add(f"object {name!r} removed")
        return node.world.scene()

    @app.post("/scene/reset", response_model=TimeOut, tags=["scene"],
              summary="Time 0: robot at its start pose, objects where added (or the start scene), commands dropped")
    async def reset(body: Optional[ResetIn] = None):
        start = bool(body and body.start)
        seq = await call(node.world.reset, start)
        node.events.add("reset" + (" to the start scene" if start else "") + f" (reset_seq {seq})")
        return time_out()

    @app.get("/scene/tasks", response_model=TasksOut, tags=["scene"], summary="The tasks an operator picks from")
    async def scene_tasks():
        return node.tasks()

    @app.post("/scene/task", response_model=TaskOut, tags=["scene"],
              summary="Switch to a task: its objects and headset view (the world is not reset: press A)")
    async def scene_task(body: TaskIn):
        return await call(node.set_task, body.name)

    @app.get("/scene/saved", response_model=SavedOut, tags=["scene"], summary="Saved scenes (for /scene/load)")
    async def scene_saved():
        return {"names": node.saved_scenes()}

    @app.post("/scene/save", response_model=SavedScene, tags=["scene"], summary="Save the objects (current poses)")
    async def save(body: SaveIn):
        return await call(node.save_scene, body.name)

    @app.post("/scene/load", response_model=SceneOut, response_model_exclude_unset=True, tags=["scene"],
              summary="Replace every object: a saved scene or a list")
    async def load(body: SceneIn):
        objs = None if body.objects is None else [o.model_dump(exclude_none=True) for o in body.objects]
        if objs is None and not body.name:
            raise bad(ValueError("give name or objects"))
        return await call(node.load_scene, body.name, objs)

    # -- privileged physics ------------------------------------------------------------------------------

    @app.post("/physics/wrench", response_model=WrenchOut, tags=["physics"],
              summary="SIM-ONLY: push a body (world-frame force / torque for a duration)")
    async def wrench(body: WrenchIn):
        return await call(node.world.apply_wrench, body.body, body.force, body.torque, body.duration)

    @app.post("/physics/teleport", response_model=BaseState, tags=["physics"],
              summary="SIM-ONLY: put the base somewhere (velocity zeroed); objects: PUT /scene/objects/{name}")
    async def teleport(body: TeleportIn):
        out = await call(node.world.teleport_base, body.xyz, body.yaw, body.rpy)
        node.events.add(f"base teleported to {out['xyz']}")
        return out

    # -- cameras -------------------------------------------------------------------------------------

    @app.get("/cameras/{name}/{stream}.jpg", tags=["cameras"], response_class=Response,
             summary="One frame rendered now (depth colourised); works paused",
             responses={200: {"content": {"image/jpeg": {}}, "description": "JPEG."}})
    async def camera_jpeg(name: str, stream: Literal["color", "depth"]):
        data = await call(node.cameras.snapshot_jpeg, name, stream)
        return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    # -- 3D view -------------------------------------------------------------------------------------

    @app.get("/viz/model", response_model=VizModel, response_model_exclude_unset=True, tags=["view"],
             summary="The world's drawable geometry (bodies, geoms, mesh index)")
    async def viz_model(cell: float = Query(viz.DEFAULT_CELL, ge=0, le=0.05,
                                            description="m, mesh simplification (0: as compiled)")):
        info, _ = await asyncio.to_thread(viz.model, node.world, cell)
        return info

    @app.get("/viz/meshes.bin", tags=["view"], response_class=Response,
             summary="The meshes of /viz/model (float32 vertices, uint16/uint32 triangles)",
             responses={200: {"content": {"application/octet-stream": {}}, "description": "Mesh blob."}})
    async def viz_meshes(request: Request, cell: float = Query(viz.DEFAULT_CELL, ge=0, le=0.05),
                         generation: Optional[int] = Query(None, description="409 if the model has moved on")):
        info, blob = await asyncio.to_thread(viz.model, node.world, cell)
        if generation is not None and generation != info["generation"]:
            raise HTTPException(409, f"model generation is {info['generation']}; fetch /viz/model again")
        if "gzip" in request.headers.get("accept-encoding", ""):
            gz = getattr(node, "_mesh_gz", None)
            if gz is None or gz[0] is not blob:
                gz = (blob, await asyncio.to_thread(gzip.compress, blob, 6))
                node._mesh_gz = gz
            return Response(gz[1], media_type="application/octet-stream",
                            headers={"Content-Encoding": "gzip", "Cache-Control": "no-cache"})
        return Response(blob, media_type="application/octet-stream", headers={"Cache-Control": "no-cache"})

    @app.get("/viz/frame", response_model=VizFrame, response_model_exclude_unset=True, tags=["view"],
             summary="Every body's pose now, the clock, the markers")
    async def viz_frame():
        return node.viz_frame()

    @app.get("/viz/stream", tags=["view"], response_class=StreamingResponse,
             summary="Server-Sent Events: /viz/frame at `hz` (markers only when they change)",
             responses={200: {"content": {"text/event-stream": {}}, "description": "Each `data:` line is a frame."}})
    async def viz_stream(hz: float = Query(30.0, ge=1, le=60, description="Frames per second")):
        async def gen():
            have = None
            period = 1.0 / hz
            while True:
                t0 = time.monotonic()
                f = node.viz_frame(have)
                have = f["markers_version"]
                yield f"data: {json.dumps(f, separators=(',', ':'))}\n\n"
                await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))
        return StreamingResponse(gen(), media_type="text/event-stream", headers={"Cache-Control": "no-store"})

    # -- ghosts --------------------------------------------------------------------------------------------

    @app.get("/ghosts", response_model=GhostsOut, tags=["markers"], summary="Every ghost (a body copy at a goal)")
    async def ghosts():
        return {"ghosts": node.ghosts.list()}

    @app.post("/ghosts", response_model=GhostsPut, tags=["markers"],
              summary="Add or replace ghosts: a body (and its subtree) drawn at a goal pose in a colour you pick")
    async def post_ghosts(body: GhostsIn):
        gs = [g.model_dump(exclude_none=True) for g in body.ghosts]
        return await call(node.put_ghosts, gs, body.source, body.replace)

    @app.delete("/ghosts", response_model=MarkersRemoved, tags=["markers"],
                summary="Remove every ghost, or one source's (`?source=`)")
    async def clear_ghosts(source: Optional[str] = Query(None, description="Only this source's ghosts")):
        return {"removed": node.ghosts.remove(source=source)}

    @app.delete("/ghosts/{name:path}", response_model=MarkersRemoved, tags=["markers"], summary="Remove a ghost")
    async def delete_ghost(name: str):
        return {"removed": node.ghosts.remove(name)}

    # -- rigid scene (rigid_scene.v1, docs/scene3d.md) ---------------------------------------------------

    def s3(cell: float, crease: float):
        return rigid.build(node.world, cell, crease, node.ghosts.declared())

    CELL = Query(rigid.DEFAULT_CELL, ge=0, le=0.05, description="m, visual mesh simplification (0: as compiled)")
    CREASE = Query(30.0, ge=0, le=180, description="deg, outline edges: faces at least this far apart")

    @app.get("/scene3d", response_model=S3Scene, response_model_exclude_unset=True, response_model_by_alias=True,
             tags=["view"], summary="The world as rigid bodies (rigid_scene.v1): bodies, geoms, mesh index")
    async def scene3d(cell: float = CELL, crease_deg: float = CREASE):
        index, _ = await asyncio.to_thread(s3, cell, crease_deg)
        return index

    @app.get("/scene3d/blob", tags=["view"], response_class=Response,
             summary="The meshes of /scene3d: float32 vertices, u16/u32 triangles and outline edges",
             responses={200: {"content": {"application/octet-stream": {}}, "description": "Mesh blob."}})
    async def scene3d_blob(request: Request, cell: float = CELL, crease_deg: float = CREASE,
                           scene_id: Optional[str] = Query(None, description="409 if the scene has changed")):
        index, blob = await asyncio.to_thread(s3, cell, crease_deg)
        if scene_id is not None and scene_id != index["scene_id"]:
            raise HTTPException(409, f"scene is {index['scene_id']}; fetch /scene3d again")
        headers = {"Cache-Control": "no-cache", "ETag": index["scene_id"]}
        if "gzip" in request.headers.get("accept-encoding", ""):
            gz = getattr(node, "_s3_gz", None)
            if gz is None or gz[0] is not blob:
                gz = (blob, await asyncio.to_thread(gzip.compress, blob, 6))
                node._s3_gz = gz
            return Response(gz[1], media_type="application/octet-stream", headers={**headers, "Content-Encoding": "gzip"})
        return Response(blob, media_type="application/octet-stream", headers=headers)

    def s3_poses(cell: float, crease: float, have):
        """(Poses for the current scene, its index): a new Poses when the scene changed."""
        index, _ = s3(cell, crease)
        if have is None or have.index["scene_id"] != index["scene_id"]:
            have = rigid.Poses(index)
        return have

    @app.get("/scene3d/view", response_model=ViewOut, tags=["view"],
             summary="How headsets draw the scene (style, which geometry); they read it every second")
    async def scene3d_view():
        return node.view

    @app.post("/scene3d/view", response_model=ViewOut, tags=["view"], summary="Set the headset view (style, role)")
    async def set_scene3d_view(body: ViewIn):
        anchor = None if body.clear_anchor else (body.anchor if body.anchor is not None else False)
        return await call(node.set_view, body.style, body.role, False, anchor)

    @app.post("/scene3d/view/next", response_model=ViewOut, tags=["view"], summary="Cycle the headset's style")
    async def next_scene3d_view():
        return await call(node.set_view, next_style=True)

    @app.get("/scene3d/frame", response_model=S3Frame, response_model_exclude_unset=True, tags=["view"],
             summary="The dynamic bodies' world poses now (JSON)")
    async def scene3d_frame(cell: float = CELL, crease_deg: float = CREASE):
        p = s3_poses(cell, crease_deg, None)
        return p.frame(node.world, node.ghosts.live()).to_json()

    @app.get("/scene3d/stream", tags=["view"], response_class=StreamingResponse,
             summary="Pose frames at `hz`: binary RSF1 frames back to back (format=bin), or JSON lines",
             responses={200: {"content": {"application/octet-stream": {}, "application/x-ndjson": {}},
                              "description": "bin: a 32-byte header (RSF1, seq, t, reset_seq, n, scene id) + n x 7 "
                                             "float32; a new scene_id means fetch /scene3d again."}})
    async def scene3d_stream(hz: float = Query(30.0, ge=1, le=120, description="Frames per second"),
                             format: Literal["bin", "jsonl"] = Query("bin"), cell: float = CELL,
                             crease_deg: float = CREASE):
        async def gen():
            p, period = None, 1.0 / hz
            while True:
                t0 = time.monotonic()
                p = await asyncio.to_thread(s3_poses, cell, crease_deg, p)
                f = p.frame(node.world, node.ghosts.live())
                yield encode_frame(f) if format == "bin" else json.dumps(f.to_json(), separators=(",", ":")) + "\n"
                await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))
        return StreamingResponse(gen(), media_type="application/octet-stream" if format == "bin"
                                 else "application/x-ndjson", headers={"Cache-Control": "no-store",
                                                                       "X-Accel-Buffering": "no"})

    @app.get("/scene3d/compat/model", response_model=S3CompatModel, tags=["view"],
             summary="For today's Quest AR view (--ar-model): a description with an empty root link")
    async def scene3d_compat_model(cell: float = CELL, crease_deg: float = CREASE):
        index, _ = await asyncio.to_thread(s3, cell, crease_deg)
        return s3compat.model(index)

    @app.get("/scene3d/compat/stream", tags=["view"], response_class=StreamingResponse,
             summary="For today's Quest AR view (--ar-state): robot_state.v1-shaped lines with every body as an "
                     "object of primitives",
             responses={200: {"content": {"application/x-ndjson": {}}, "description": "One JSON object per line."}})
    async def scene3d_compat_stream(hz: float = Query(30.0, ge=1, le=100, description="Lines per second"),
                                    role: Literal["collision", "visual"] = Query(
                                        "collision", description="Which geoms to draw (falls back per body)"),
                                    cell: float = CELL, crease_deg: float = CREASE,
                                    format: Literal["jsonl"] = Query("jsonl", description="jsonl (the view's)")):
        async def gen():
            p, lines, period = None, None, 1.0 / hz
            while True:
                t0 = time.monotonic()
                p = await asyncio.to_thread(s3_poses, cell, crease_deg, p)
                if lines is None or lines.scene_id != p.index["scene_id"]:
                    lines = s3compat.StateLines(p.index, role)
                    base_i = next((k for k, i in enumerate(p.index["dynamic"])
                                   if p.index["bodies"][i]["name"] == "base_link"), None)
                f = p.frame(node.world, node.ghosts.live())
                base = None if base_i is None else f.poses[base_i]
                yield json.dumps(lines.line(f, base, node.overlays()), separators=(",", ":")) + "\n"
                await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))
        return StreamingResponse(gen(), media_type="application/x-ndjson",
                                 headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})

    # -- robot_state.v1 ------------------------------------------------------------------------------

    @app.get("/robot_state", response_model=RobotState, response_model_exclude_unset=True, tags=["view"],
             summary="robot_state.v1: base pose, every joint, objects, overlays (one record)")
    async def robot_state():
        return node.robot_state()

    @app.get("/robot_state/stream", tags=["view"], response_class=StreamingResponse,
             summary="robot_state.v1 records as JSON lines at `hz` (plain chunked HTTP: a TCP client reads lines)",
             responses={200: {"content": {"application/x-ndjson": {}}, "description": "One JSON object per line."}})
    async def robot_state_stream(hz: float = Query(30.0, ge=1, le=100, description="Records per second"),
                                 format: Literal["jsonl"] = Query("jsonl", description="jsonl (one JSON per line)")):
        async def gen():
            period = 1.0 / hz
            while True:
                t0 = time.monotonic()
                yield json.dumps(node.robot_state(), separators=(",", ":")) + "\n"
                await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))
        return StreamingResponse(gen(), media_type="application/x-ndjson",
                                 headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})

    # -- markers -------------------------------------------------------------------------------------

    @app.get("/markers", response_model=MarkersOut, response_model_exclude_unset=True, tags=["markers"],
             summary="Every live marker")
    async def markers():
        return {"markers": node.markers.list()}

    @app.post("/markers", response_model=MarkersPut, tags=["markers"],
              summary="Add or replace markers (frames, points, lines, labels) in the 3D view")
    async def post_markers(body: MarkersIn):
        mks = [m.model_dump(exclude_none=True) for m in body.markers]
        return await call(node.put_markers, mks, body.source, body.replace)

    @app.delete("/markers", response_model=MarkersRemoved, tags=["markers"],
                summary="Remove every marker, or one source's (`?source=`)")
    async def clear_markers(source: Optional[str] = Query(None, description="Only this source's markers")):
        return {"removed": node.markers.remove(source=source)}

    @app.delete("/markers/{name:path}", response_model=MarkersRemoved, tags=["markers"], summary="Remove a marker")
    async def delete_marker(name: str):
        return {"removed": await call(node.markers.remove, name)}

    return app
