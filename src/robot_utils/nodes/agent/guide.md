# agent: driving the robot as an agent

You are about to drive a real robot: a swerve-drive base with an RGB-D camera, in a room with
people and things. This guide is all you need. Every call is plain HTTP + JSON. `N` below is
this node's base URL, e.g. `http://botcore/nodes/agent` (all paths in replies are relative to it).

An arm stands on the base's front-right corner. You move it through the arm node
(`http://botcore/nodes/arm`, its own `/docs`), not here; its `GET /geometry` says where its
fingertips and every part are, in its own frame and in the base frame below. Here, `/plan/check`
draws the arm with the base, and anything the robot holds or reaches past its front edge counts
as an obstacle for the stop reflex (keep a carried object behind the bumper while driving).

## Principles (read these)

1. **You decide; the tools don't.** Perception and planning calls are read-only. They return
   numbers, proposals and images. Nothing moves unless you call `POST /goal`. No call turns
   perception into motion by itself.
2. **Stopping is the only automatic action.** The stop reflex brakes if depth shows an
   obstacle within the braking distance ahead of the motion. It only sees what the camera sees
   (forward, beyond the blind zone: `GET /robot`, about 0.8 m from the base centre). It is a guardrail, not your obstacle avoidance: **you** avoid
   obstacles. It counts depth points ahead of the bumper, whatever they are, so its stops can
   include false positives; each stop's event has a `mask` image of the points it counted.
3. **Look before you move, and look at the images.** Every perception and planning reply has an
   overlay image and a bird's-eye view (BEV). Fetch and inspect them. They show what the tool
   actually selected, fitted or checked. If the overlay is not what you meant, your numbers are wrong.
   **Select only the pixels you mean** (step 2): a 3D measurement is only as good as its selection.
4. **Path checks are loud, not binding.** `/goal` and `/plan/check` report obstacle points in the
   swept footprint, clearance and the share of the path that was never seen. Read the `warnings`.
   An `UNVERIFIED` part of the path, such as the blind zone in front of the robot, is unknown, not clear.
5. **Move in legs you have verified.** Prefer legs of ≲1–1.5 m, then look again. The camera can't
   see the floor within about 0.8 m of the base centre (about 0.6 m beyond the front edge): rely on
   what you saw from earlier positions.
6. **Work asynchronously.** `POST /goal` returns at once. Wait for it in the background
   (`GET /goals/{id}/wait`), and meanwhile look, think, or stop / replace the goal.
7. **Explain yourself.** Every POST accepts an optional `"comment"`. It goes into the transcript
   that humans read. Say why, briefly.
8. **The human is in charge.** If they disable or e-stop on the page, you get `423` until they
   release it. Ask, don't work around. Never send the `X-Operator: human` header: it is for the
   page / a person.

## Workflow

```bash
N=http://botcore/nodes/agent
curl -s $N/guide.md                                                    # this file
curl -s -X POST $N/session -H 'content-type: application/json' -d '{"agent": "Claude", "avatar": "🦉"}'
curl -s -X POST $N/enable  -H 'content-type: application/json' -d '{"comment": "starting the task"}'
```

**0. Know your body.** `GET $N/robot` returns the footprint, the wheels, the frames, the limits
and the camera: where it is (about 1.13 m up), where it points (pitch and yaw), its field of view,
and the floor it can and cannot see. It also returns `diagram`, a side and top view to scale.
Read it once per session: it is what you look *from*.

```bash
curl -s $N/robot                                    # JSON; then: curl -s -o robot.png $N/images/<diagram>
```

**1. Look.** It returns a snapshot id, the colour image, a BEV and the pose at capture.

```bash
curl -s -X POST $N/look -H 'content-type: application/json' -d '{"grid": true}'
# -> {"snapshot": "s1", "image": "images/s1.jpg", "bev": "images/s1_bev.png", "pose_world": [x, y, θ], ...}
curl -s -o s1.jpg $N/images/s1.jpg                                      # then view the file
```

The colour image's size is `image_size` (e.g. 640x480); selections are in its pixels. `grid: true`
draws labelled pixel lines every 100 px, so you can name coordinates; `{"grid": {"step": 50,
"values": "xz"}}` sets the spacing and also writes the depth's base-frame values at each grid
intersection: `xz` (distance ahead and height: with the pixel, a full 3D point), or `x`, `y`, `z`,
`range`, `xyz`. They are returned as numbers too (`grid.samples`, always x, y, z). The depth grid is half the colour size by default; `{"scale": 1.0}` gives
full-resolution depth for fine work. The BEV is top-down in the robot's frame at capture: the robot at the bottom,
x forward (up), y left, 0.5 m grid. Floor points are grey, obstacles orange/yellow by height, the
blue line is the nearest visible floor.

**Zoom** to read pixels precisely. It enlarges part of a snapshot you already have (nothing is
captured), with grid labels in the **full image's** pixel coordinates: a point you read off a zoom
is a query coordinate as it is, on the same snapshot.

```bash
curl -s -X POST $N/zoom -H 'content-type: application/json' \
     -d '{"snapshot": "s1", "box": [380, 260, 500, 350], "step": 10, "values": "xz", "comment": "the block"}'
# -> {"zoom": "z1", "image": "images/z1.jpg", "magnification": 8, "samples": [{u, v, base: [x, y, z]}, ...]}
```

Zoom enlarges the pixels the camera captured; it adds no detail, but it lets you place selections
on the right pixels. With `values`, every number is drawn on the surface it belongs to; read them
there, not just from `samples`, so you know which surface each one is.

**2. Query what you see.** Select pixels with a point, disk, box or polygon, and optionally exclude shapes.

```bash
curl -s -X POST $N/query/plane -H 'content-type: application/json' -d '{
  "snapshot": "s1", "select": {"polygon": [[190,10],[350,10],[345,75],[195,75]]},
  "exclude": [{"box": [300,0,350,20]}], "comment": "the whiteboard, not the curtain"}'
# -> centre / normal / width / height in base and world frames, touches_image_edge, overlay, bev
```

- `/query/region`: 3D extent (5/50/95 percentiles), centroid, and the nearest point in base and world frames.
- `/query/plane`: robust plane fit: centre, normal (toward the camera), facing heading, width, height,
  rms, inliers. `touches_image_edge: true` means the surface may continue out of view, so its
  centre and width may be biased. Look again from elsewhere, or account for it.
- `/query/line`: robust 3D line through **points on an edge** (`"points": [[u, v], ...]`, 2 or
  more, each read as a small disk) or a thin selection. It returns the direction, `heading`
  (base and world), length, ends and tilt. Use it for anything you want to drive *along* or
  park *parallel to*: a table edge, a wall's base, a row of legs, a cable.

**Planes give normals; lines give headings.**
- A plane fit of a strip along an edge returns whichever surface dominates the strip (often the
  tabletop, whose normal is vertical and says nothing about the edge's direction), or a mixed fit.
- For a direction, put 3–5 points on the edge itself and use `/query/line`.
- For a face's orientation, select an area of that face (the table's side panel, the wall),
  not its edge.

Check the overlay: magenta is your selection, red your exclusions, green the fitted plane. In
the BEV, the selection's footprint has a thick magenta border.

**Pixel coordinates belong to one snapshot.** Once the robot moves (even a small turn), the same
(u, v) is a different place. Pick pixels anew on each new image. To carry a 3D location across
snapshots, use its coordinates in a `snapshot:<id>` frame or the world frame, or save a place.

**Selections must contain only the thing you are measuring.** This is not 2D object detection:
a loose bounding box that "contains" the object is fine for a 2D detector, but in 3D every stray
pixel is a real point somewhere else.
- **Why it matters:** the floor below an object, the wall behind it and the edge of a neighbour
  lie at very different distances. A few of them pull a centroid, a nearest point, an extent or a
  plane far off, worst when the surface is seen at an angle.
- **The rule:** accept a box measurement only if the box contains *only* desired pixels. Check
  the overlay and the BEV footprint: a selection that spills onto the floor or the background
  shows up as a smeared or oversized magenta outline.
- **What to use instead**, in order of preference:
  - **A polygon** traced just inside the object's outline. This is the tool for anything that
    isn't a screen-aligned rectangle: a tilted board, a chair, a doorway.
  - **A small box or disk well inside** the object, when you only need its distance or its plane.
  - **A sequence of point or disk queries** on distinct parts (corners, edges, legs). You combine
    the points yourself: for example, two points on a doorframe give its width and position.
  - **`exclude`** shapes to cut out what doesn't belong (a curtain, a cable, a person).
- **Sanity checks:**
  - `depth_fraction` well below 1 means much of the selection has no depth (glass, black surfaces,
    the image edge).
  - For a plane, a low `inlier_fraction` or a high `rms_m` means the selection mixes surfaces.
  - A region's 5–95 % spread much larger than the object means outliers. Shrink the selection
    and query again.

**3. Plan.** Read-only.

```bash
curl -s -X POST $N/plan/approach -H 'content-type: application/json' -d '{"query": "q1", "standoff": 1.2}'
# -> proposal {frame: "snapshot:s1", x, y, theta}, path check, BEV with the swept footprint
curl -s -X POST $N/plan/check -H 'content-type: application/json' -d '{"frame": "local", "x": 0.8, "y": 0.2}'
```

In the check BEV: green footprint = clear, red = obstacle points inside (red dots), grey = never
seen. Proposals are suggestions. Verify them and adjust them yourself.

**Imagine the whole robot.** `/plan/check` also draws the robot at the goal into the snapshot
(`robot.view`: the base in cyan, the arm in orange, the fingertips L/R in magenta), with the arm as
it is now, or at `"arm": {"pose": {pos, rpy}, "gap_mm": 40}` (a TCP pose in the arm frame, solved as
the arm node's `POST /ik`) or `{"q": [...]}`. It returns the fingertips in the goal's base frame,
the snapshot's and the world, how far the arm reaches past the frame, and the depth points that
would be inside the arm (`depth_points_in_arm`: something in the way, or the arm itself where it
is now). One call checks a base move, an arm pose, or both; nothing moves. Only what lies in the
snapshot's view is drawn.

**When the drawing and the image disagree.** With no move (`{"frame": "local"}`) the arm is drawn
where it is now, so its drawn fingertips should sit on the real ones. If they are consistently
off, the camera or arm-mount placement is slightly off: correct it for this session with `POST
/session/geometry {"camera": {"xyz": [dx, dy, dz], "rpy_deg": [...]}, "comment": why}` (≤ 5 cm,
≤ 3°; `{}` removes it), and check again. It changes the base and world coordinates of your
looks, zoom values, queries (held snapshots included) and drawings, for this session only. The
stop reflex, goals and arm commands never use it.

**Reading the warnings.** They are meant to be rare, so act on them:
- `OBSTACLE`: points above the floor inside the swept footprint.
- `tight`: under 15 cm of clearance.
- `UNVERIFIED`: part of the swept area was not seen in the snapshot checked. Look from a position
  that sees it, move in a shorter leg, or name an earlier snapshot of this session that saw it in
  `also_use` (only views that are still true: the arm and people may have moved since).
- `STALE`: the robot moved since that snapshot, or it is minutes old. Look again.
- A turn in place carries a `note` instead: its corners sweep a 0.31 m radius around the base
  that the camera can't see. Make sure nothing low stands beside the robot.

**4. Move.** Explicitly. Then wait in the background.

```bash
curl -s -X POST $N/goal -H 'content-type: application/json' \
     -d '{"frame": "snapshot:s1", "x": 1.4, "y": 0.2, "theta": -0.5, "comment": "first leg toward the board"}'
# -> {"goal_id": 7, "warnings": [...], "check": {...}, "bev": "...", "wait": "goals/7/wait"}
curl -s "$N/goals/7/wait?timeout=60"          # run this in the background; it returns when the goal ends
```

The result is one of:
- `reached`;
- `stopped` (someone called `/stop`, or the reflex braked: the transcript's event has the points it
  counted, `points_base`, and a `mask` image);
- `stalled` (no progress: the base isn't following, so look at why);
- `aborted` (the estimator was lost or reset, or someone disabled).

While a goal runs you may `POST /look`, `POST /stop`, or post a new `/goal` (it replaces the
current one smoothly).

**Frames** for goals, checks and places:
- `local` (default): x forward, y left, relative to now; `theta` is literal, so `6.28` = one full turn;
- `global`: the estimator's world frame; `theta` takes the shortest turn;
- `snapshot:<id>`: relative to the robot's pose when that snapshot was taken, so coordinates
  read from a snapshot stay valid after you move;
- `place:<name>`.

Goal options (each within the caps):
- `speed`, `omega`, `accel`, `alpha`, `jerk`, `ang_jerk`;
- `pos_tol` (0.02 m), `heading_tol` (0.035 rad);
- `mode`: `synchronized` (default) or `independent`;
- `on_reach`: `terminate` or `hold`;
- `reflex_margin_m` (default 0.12 m, any value > 0): the gap the stop reflex leaves in front of
  what it sees, for this goal. Braking and latency are added as always, so a small margin (e.g.
  0.02) lets the robot end close to a surface, even flush against a wall, and still stop before
  touching it.

**Patterns that work.**
- **Scan:** turn in place in steps of about 90° (`theta` 1.57, `local`), looking after each, to
  find something not in view. Prefer the direction that keeps any cable unwound if the human
  says so.
- **Approach:** take legs of at most about 1.3 m toward the target, looking and measuring again
  after each. Plan the final leg from the newest snapshot.
- **Park parallel to an edge at a gap:**
  1. Face the edge and look.
  2. Get the edge's line and heading from `/query/line` on 3–5 points along it.
  3. Compute the goal pose: the line's heading, offset by your half-width (0.216 m) plus the gap
     along the line's normal.
  4. Check it, move, look again, and measure the same edge with `/query/line` on the new image.
  5. Correct with a small `local` goal (a sideways crab and a small turn) if the gap or angle is off.
- **Find a thin thing on the floor (a cable):** depth doesn't see it, but a point or small disk on
  its pixels returns the floor under it, which is its position. Trace it with a sequence of disks.

**5. Remember.** `POST /memory/note {"text"}` and `POST /memory/place {"name", ...}` (default: here),
then `GET /memory`. Places are lost when the estimator resets.

**6. Finish.** `POST /disable`, then `DELETE /session`.

## Limits and caps

- **Caps:** the human picks the safety profile (`cautious` by default: 0.15 m/s, 0.5 rad/s,
  0.2 m/s², goals ≤ 2 m / 3.2 rad away). You may lower caps (`POST /enable {"caps": {...}}`) but
  never raise them.
- **Goals that don't start (nothing moves):** too far or turning too much for the caps → `422`
  (split it into legs). Not enabled → `409`. Human hold → `423`. Estimator unusable → `503`.
- **What the camera sees:** forward only, about 93°x61°, pitched about 23° down, with the floor
  visible from about 0.8 m out (exact numbers: `GET /robot`).
  The reflex and the path checks see nothing beside or behind the robot, so turn and look.
- **Depth:** unreliable on glass, mirrors, black or shiny surfaces and thin legs or cables.
  Treat thin things you see in the image as real even when depth misses them.
- **Snapshot age:** snapshots belong to your session (a new session starts with none) and are kept briefly (the last 30). A snapshot from an earlier position is
  still valid in its own frame, but things may have moved since.

## Reference

| call | what |
|---|---|
| `GET /guide.md` | this guide |
| `POST /session {agent, avatar, comment}` / `DELETE /session` | open / close (closing disables) |
| `GET /robot` | body, wheels, frames, limits, camera pose / field of view / blind zone, diagram |
| `POST /enable {caps?, comment}` / `POST /disable` | motion on (pose_servo + swerve lease) / off at once |
| `POST /look {scale?, grid?, settle?}` | snapshot: image, BEV, pose at capture (`grid`: true or {step, values}) |
| `POST /zoom {snapshot?, box?, step?, values?}` | enlarge part of a snapshot, grid in full-image pixels, depth values |
| `POST /query/region`, `POST /query/plane` `{snapshot?, select, exclude?}` | read-only geometry with overlays |
| `POST /query/line {snapshot?, points \| select, exclude?}` | 3D line: direction, heading, length (edges, alignment) |
| `POST /plan/check {frame, x, y, theta, snapshot?, arm?, view?}` | path check (loud, advisory) + the robot drawn at the goal |
| `GET`/`POST /session/geometry` | camera / arm-mount placement: described, session correction, effective |
| `POST /plan/approach {query, standoff, face}` | proposed goal in front of a surface |
| `POST /goal {frame, x, y, theta, options…, comment}` | **move**; returns at once |
| `GET /goals/{id}`, `GET /goals/{id}/wait?timeout=` | goal state / long-poll until done |
| `POST /stop`, `POST /estop {reason}` | brake / e-stop (latches for the human) |
| `GET /images/{name}`, `GET /images/{snapshot}/grid.jpg` | rendered images |
| `GET /memory`, `POST /memory/note`, `POST /memory/place` | per-session memory |
| `GET /state`, `GET /log?since=`, `GET /sessions` | status, live transcript, stored transcripts |

Selections: `{"point": [u, v]}`, `{"disk": [u, v, r]}`, `{"box": [u0, v0, u1, v1]}` or
`{"polygon": [[u, v], …]}`, in colour-image pixels (`image_size`, origin top left).
