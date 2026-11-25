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
reset_seq, u16 n, u16 m, the first 8 bytes of scene_id), n x 7 float32 `[x, y, z, qx, qy, qz, qw]` in `dynamic`
order, then m colour overrides (u32 slot, 4 x u8 rgba): this frame, every geom of dynamic body `slot` in that
colour; alpha < 1 see-through, 0 hidden. Frames are stateless: no override, the geoms' own colours (whose alpha
can be < 1 too: see-through objects). `format=jsonl` and `/scene3d/frame` give the same as JSON (`colors`: slot ->
rgba 0..1). A new `scene_id` means: fetch the scene again (an object
added or removed, a fixed one moved). `reset_seq` changes on a sim reset (the AR view re-anchors).

## Ghosts (`/ghosts`)

A copy of a body, by default with its subtree (the gripper: `arm_end_link` and its fingers), drawn at a goal pose in
a colour the poster picks: `POST /ghosts {source, replace, ghosts: [{name, body, subtree, pose | relative, rgba,
ttl}]}`. `pose`: the body's goal in the sim world; `relative`: the goal in the body's own frame now (the sim applies
it to its own body, so a poster whose world is an estimate adds no drift). The first post declares the ghost (its
bodies join the scene, kind `ghost`, named `ghost:<name>/<body>`: a scene change); after that each frame carries
their poses and colour, alpha 0 once `ttl` lapses without a new post. `GET /ghosts`, `DELETE /ghosts[?source=]`,
`DELETE /ghosts/{name}`. robot_state.v1 lists live ghosts as objects too, so today's AR view draws them (lines in
the ghost's colour; it has no see-through).

The teleop node posts the **end effector's goal** this way in the sim (docs/quest_teleop_node.md, "Sim aids"):
green on target, yellow at half, red from 5 cm / 20°. Checked on the sim stack: a goal 5 cm along world x puts the
ghost's TCP 5.0 cm from the real one (the WBC's estimate agreed with the sim within 0.2 mm).

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
puts every joint back to 0; saved scenes keep the links (joint positions restart at 0). A joint may carry a `drive`
(a velocity servo, e.g. a turntable) or `mimic` another link's joint (docs/sim.md, "The rings scene").

`scene.cabinet: true` (sim config) adds `scene.py`'s cabinet beside the table (front in line with the table's
edge at y -1.25: drive right to it): two drawers sliding out 28 cm and a door hinged on its +y edge opening to
~110 deg, each with a 12 mm bar handle 35 mm off the front (inside the gripper's 49.6 mm), and a cube in the upper
drawer.

## Measured headset budget (Quest 3S, 2026-11-24)

quest_env `render_bench` (raw: quest_env `bench_results/quest3s-20261124-194313.jsonl`): 100 moving bodies over
passthrough, stereo, 1280 x 1340 per eye, 72 Hz, GPU time of our draws per frame (both eyes):

| triangles (edges) | wire | hidden-line | flat |
|---|---|---|---|
| 96 k (79 k) | 1.3 ms | 1.8 ms | 0.6 ms |
| 198 k (176 k) | 3.0 ms | 4.0 ms | 1.2 ms |
| 391 k (354 k) | 5.0 ms | 5.9 ms | 2.4 ms |
| 783 k (715 k) | 8.5 ms, **26 % frames missed** | 11.6 ms, **44 % missed** | (run invalid) |
| 1.59 M (1.45 M) | 16 ms, 63 % missed | 22 ms, 69 % missed | 6.0 ms, 0 missed |

- **Lines are the cost, not triangles**: 1.6 M flat-shaded triangles take 6 ms; 1.45 M lines take 16. The budget is
  set by outline edges: ~350 k edges (hidden-line, ~400 k triangles) runs with no missed frames.
- **Frames go missing from ~8 ms of our GPU time**, well under the 13.9 ms frame: the compositor and passthrough
  share the GPU. Budget ~6 ms of drawing.
- **Draw calls**: ~2 µs of CPU each (1000 per eye: 3.2 ms, 4000 per eye: 8.6 ms, still no misses). Past ~1000
  bodies, batch or instance them.
- The test meshes are noisy spheres with an edge for nearly every triangle; decimated CAD at a 30° crease has far
  fewer (the sim's robot: 0.6 edges per triangle), so real scenes fit more triangles per edge budget. 90 Hz is not
  measured (the frame there is 11.1 ms).

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
