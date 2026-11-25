# Rigid scenes (rigid_scene.v1)

A simulated (or real) world for a remote viewer, mainly the Quest's AR / VR teleop view: **rigid bodies with
geometry, sent once, plus every moving body's world pose, streamed**. The viewer never runs kinematics: the robot's
links, a drawer, a door and a loose cube are all just bodies with poses. Any physics backend has those (MuJoCo
`xpos`/`xquat`, PhysX / Isaac `get_world_poses`, SAPIEN link poses), so one viewer serves them all.

Code: `src/robot_utils/scene3d/` (the format, backend-agnostic), `src/robot_utils/nodes/sim/rigid.py` (the MuJoCo
exporter), the sim node's `/scene3d` endpoints. Tests: `tests/test_scene3d.py`.

## Why this shape

- **Rendering stays on the headset.** Head motion is drawn at the display rate from local geometry; only poses cross
  the network (~0.9 KB per frame for 30 bodies, ~26 KB/s at 30 Hz). No video encoding, and Wi-Fi spikes delay
  objects, never the operator's view.
- **Crude graphics, real physics.** The operator needs geometry that is faithful to what the physics touches, not
  photorealism: the policy's camera images are rendered by the simulator, independent of the viewer (and a
  logged pose stream can be re-rendered later at any fidelity).
- **Geometry only.** Vertices, triangles, outline edges and a colour per geom: every backend can export that without
  material conversion. Wireframe, hidden-line, flat-shaded or X-ray are styles a viewer picks over the same data.

## Scene (`GET /scene3d` + `GET /scene3d/blob`)

The JSON index:

| field | |
|---|---|
| `schema` | `rigid_scene.v1` |
| `scene_id` | 16 hex digits over the index and the blob: equal scenes, equal ids. Cache key; the frames carry it |
| `conventions` | `{up: +z, units: m, quat: xyzw}` |
| `bodies` | `{id, name, kind: world \| robot \| object, parent, dynamic, pose, geoms, joint?}` |
| `dynamic` | body ids in the pose stream's order |
| `meshes` | `{name, nvert, ntri, nedge, vert_offset, tri_offset, edge_offset, index: u16 \| u32, bounds, error}` |
| `crease_deg` | outline edges keep faces at least this far apart |

- **Static bodies** (`dynamic: false`, MuJoCo: welded to the world) keep their `pose` for good; moving one (a fixed
  object teleported) makes a new scene.
- **Geoms**: `{type, pos, quat, rgba, role}` plus `half_extents` (box 3, plane 2 with 0 = unbounded), `radius`,
  `half_length` (capsule / cylinder, along local z), `radii` (ellipsoid) or `mesh`.
- **Roles**: `collision` (what the physics touches), `visual` (drawn only), `both` (scene objects: what is drawn is
  what collides). Draw `collision` for anything the robot can touch: the operator then sees what the physics sees.
  MuJoCo collides a mesh as its **convex hull**, so a colliding mesh is exported as its hull (`<name>.hull`, MuJoCo's
  own) and the mesh as `visual`; the finger meshes are the known case where the two differ.
- **Blob**: per mesh, float32 vertices, then triangles, then outline edges (vertex pairs: boundary, non-manifold and
  crease edges), each section 4-byte aligned. `?scene_id=` answers 409 if the scene has moved on; gzip on request.
- `joint` (`{name, type: hinge | slide, range?}`) and `parent` are informational (cues, labels).

## Frames (`GET /scene3d/stream`, `GET /scene3d/frame`)

`format=bin` (default): frames back to back, each a 32-byte little-endian header (`RSF1`, u32 seq, f64 t, u32
reset_seq, u32 n, the first 8 bytes of scene_id) and n x 7 float32 `[x, y, z, qx, qy, qz, qw]` in `dynamic` order.
`format=jsonl` and `/scene3d/frame` give the same as JSON. A new `scene_id` means: fetch the scene again (an object
added or removed, a fixed one moved). `reset_seq` changes on a sim reset (the AR view re-anchors).

## Simplification

What is drawn may differ from what is simulated; the question is by how much. Each mesh carries `error`: how far
(m) its surface may be from the simulated one. Today the sim's visual meshes are simplified by vertex clustering
(`cell`, default 2 mm), whose bound is the cell's diagonal on vertex displacement; hulls are exported as computed
(0). The plan for heavy scenes (Isaac kitchens: 1.2 M triangles for 27 objects alone):

1. simplify to an **error target**, not a triangle count (quadric error, e.g. meshoptimizer);
2. **measure** the two-sided surface distance against the collision surface (not the visual mesh: PhysX uses
   convex decompositions, SDFs or bounding boxes) and store it as `error`;
3. allowed error = the smaller of **what the eye resolves** (~0.05 deg per pixel on a Quest 3S: ~1 mm at 1 m) and
   **what the task needs** (handle thickness, drawer gaps): coarse far away, exact where the gripper works; a few
   levels of detail per mesh.

The bound covers geometry only. Contact cues (highlights, controller vibration) stay the backstop.

## Today's Quest AR view, without a headset update (`/scene3d/compat/...`)

The view (quest_env `ROBOT_AR.md`) draws a description by forward kinematics plus robot_state.v1 lines whose
`objects` are rigid bodies of primitives. The compat endpoints map a rigid scene onto that: the description is an
empty root link, and every body (static ones too) comes in each line as an object. Meshes become their bounding
boxes and planes are left out (the view draws primitives only); `role=collision` (default) or `visual` picks the
geoms; `base` (base_link's pose) and the overlays (markers, sim, arm) are kept, so targets and the velocity arrow
still draw. Static geometry is resent in every line (~9 KB per line with the cabinet scene): fine for a prototype,
the reason for the native format.

```sh
# the sim stack on the Mac with the cabinet beside the table
ROBOT_SIM_CONFIG=deploy/sim/sim_cabinet.yaml python -m robot_utils.nodes.sim.stack --quest --no-pull
# on the headset (URL = the frontend the stack serves; quest_ctl up passes --ar=URL already)
qr --ar=URL --ar-model=URL/nodes/sim/scene3d/compat/model --ar-state='URL/nodes/sim/scene3d/compat/stream?hz=30'
```

## Articulated scene objects (the sim)

`POST /scene/objects` with `shape: articulated` and `links`: a tree of rigid parts, root first, each a list of
primitives (box / cylinder at a pose in the link frame) joined to its parent by a `slide` or `hinge` joint (axis,
range, damping, frictionloss) or welded. The object is fixed (furniture) or free as any other. `/scene` reports
each jointed link's position (`joints`); robot_state.v1 lists one object per link (`cabinet/drawer_high`); a reset
puts every joint back to 0; saved scenes keep the links (joint positions restart at 0).

`scene.cabinet: true` (sim config) adds `scene.py`'s cabinet beside the table (front in line with the table's
edge at y -1.25: drive right to it): two drawers sliding out 28 cm and a door hinged on its +y edge opening to
~110 deg, each with a 12 mm bar handle 35 mm off the front (inside the gripper's 49.6 mm), and a cube in the upper
drawer.

## Next

- **Headset renderer** (quest_env): fetch and cache the scene by `scene_id`, meshes and edges in GPU buffers, one
  matrix per body per frame, a depth pass for hidden-line wireframe, multiview. Today's renderer rebuilds and clips
  every line segment on the CPU per eye per frame, which caps it at tens of thousands of segments; the benchmark
  (quest_env `tools/render_bench`) measures the headset's budget.
- **Measured error** in place of the clustering bound; levels of detail.
- **Other producers**: the real robot (its links from the arm node's kinematics, no physics); an Isaac Lab exporter
  (USD: bake each rigid body's mesh subtree into body-local meshes once, stream `RigidPrim.get_world_poses()`;
  about 130 dynamic bodies per SimReady kitchen).
- **Side channels**: contacts (point, normal, force) and joint values for cues.
