# agent_nav node

`python -m robot_utils.nodes.agent_nav` is the boundary between an AI agent and the robot. The agent
looks, measures and plans through read-only calls, and moves the base only with explicit goals. A
human can watch, and take part in, the exchange on the node's page.

- **For agents:** `GET http://<robot>/nodes/agent_nav/guide.md` is the whole briefing. Its
  source is `src/robot_utils/nodes/agent_nav/guide.md`. Give an agent that URL before a
  session; no agent-specific wrapper is needed.
- Page: `http://<robot>/nodes/agent_nav/`: the dialogue, the live view (image + BEV), controls, a
  command line for humans.
- API docs: `http://<robot>/nodes/agent_nav/docs`

## Principles (agreed 2026-11-08)

1. **Agent nav means agent nav.** No call ingests perception and moves the base on its own.
   - Perception (`/look`, `/query/*`) and planning (`/plan/*`) are read-only: they return data,
     proposals and overlay images.
   - Only `POST /goal` moves the robot, plus `/stop` and `/estop`, which stop it, and only when
     the agent or a human issues them.
   - There is no "go to what I clicked" endpoint: the agent queries, decides and issues the goal.
2. **Stopping is the only automatic action.** The stop reflex brakes, through pose_servo
   `/stop`, when registered depth shows obstacle points within the braking distance ahead of
   the motion. It is a guardrail, not obstacle avoidance; avoidance is the agent's job.
3. **Path checks are loud, never binding.** Goals and `/plan/check` report:
   - obstacle points in the swept footprint;
   - the clearance;
   - the share of the swept area the snapshot never saw (the blind zone);

   as `OBSTACLE` / `UNVERIFIED` / `STALE` warnings. The goal still executes as issued.
4. **Code for math and reflexes; the agent for decisions.**
   - Code does the unprojection, plane fits, transforms, the 50 Hz servo, the limits and the
     stops.
   - The agent decides what matters, where to go, and whether a path is acceptable (cardboard on
     the floor, a cable, a person).
   - Every tool shows its work: overlays and BEVs let the agent check that a selection or fit is
     what it meant.
5. **Asynchronous by default.** `POST /goal` returns at once, and `GET /goals/{id}/wait` is a
   long-poll the agent runs in the background. Meanwhile it can look, think, stop or replace the
   goal.
6. **The human is in charge.** A disable or e-stop from the page latches: the agent gets `423`
   until the human releases it. The human picks the safety profile, and the agent can only
   lower caps.

## Architecture

```
agent (curl) ──HTTP──► agent_nav ──► 3d_perception  /frame, /calibration, /settings   (snapshots)
                          │     ──► estimator      /stream                          (pose at capture)
                          │     ──► pose_servo     /enable /goal /stop /estop /state (motion)
                          │                         └─► swerve (lease, twists)
                          └── data dir: <session>/session.json, log.jsonl, thumbs/*.jpg
```

- **Snapshot:** one fused frame from 3d_perception, with the colour JPEG at 1280x720 and the
  registered depth (640x360 by default; `scale` up to 1.0). It carries the estimator pose
  interpolated at `t_color`, the camera-in-base transform, and the estimator `reset_id`. The
  last 30 are kept in memory at full resolution.
  - `look {scale}` changes 3d_perception's `scale` for one frame and restores it, so other
    viewers briefly see that resolution too.
- **Frames for goals, checks and places:**
  - `local`: x forward, y left; theta literal;
  - `global`: the estimator's world frame; theta takes the shortest turn;
  - `snapshot:<id>`: the base at that capture; refused after an estimator reset;
  - `place:<name>`.

  agent_nav resolves every goal to a literal local goal for pose_servo, so multi-turn rotations
  in snapshot frames are kept.
- **Geometry** (`geometry.py`, pure):
  - Selections are point, disk, box or polygon in colour pixels, with exclusions. They are
    rasterised onto the depth grid.
  - Unprojection uses rays with the lens distortion removed.
  - `region` gives percentiles and the centroid in the base and world frames.
  - `plane` is a RANSAC + SVD fit. It reports the centre at the middle of its extent, a normal
    toward the camera, width and height for vertical planes, and `touches_image_edge` (with a
    warning that the centre and width may be biased).
  - Path check: the footprint (0.43 m + 0.05 m margin) is swept along the straight move with
    the heading interpolated. It counts points 0.04–1.6 m above the floor inside the sweep.
    Coverage is the share of floor samples in the sweep whose projection into the snapshot sees
    the floor (or something in front of it). The robot's own footprint is excluded.
  - BEV: top-down in the snapshot's base frame. Floor points are grey; obstacles orange to
    yellow by height; the nearest-visible-floor line is blue; the selection footprint has a thick
    magenta border; the swept path is green or red; never-seen samples are grey.
- **Stop reflex:** active while pose_servo is moving and a goal is armed.
  - About 6 times a second it registers a fresh frame and looks along the commanded velocity's
    direction for points in the band beyond the front edge. The band's length is
    `v²/(2·max_accel) + 0.35 s·v + 0.12 m`, and its width the footprint plus 3 cm.
  - 25 or more points there → pose_servo `/stop` (it brakes within the caps), a `REFLEX STOP`
    turn in the dialogue, and the goal ends `stopped`. It re-arms on the next goal.
  - It is blind when the motion points more than 40° off the camera's view (it says so in
    `/state`), and within the camera's near blind zone.
- **Profiles** (server side; the human chooses, the agent may only lower):
  - `cautious` (default): 0.15 m/s, 0.5 rad/s, 0.2 m/s², 0.8 rad/s², jerk 0.8 / 3.0, goals ≤ 2 m / 3.2 rad.
  - `normal`: pose_servo's defaults, goals ≤ 3 m.
- **Transcript:** every call is a dialogue turn: caller (agent, human or robot), kind, request
  (selections stay vector), reply, comment, images. It goes to memory (the last 1000) and to
  `<data dir>/<session>/log.jsonl`, with a 320 px thumbnail per image in `thumbs/`. Robot events
  are turns too: goal results, reflex stops, external disables, estimator resets. The page shows
  the live session at full resolution (images in memory) and falls back to thumbnails.
- **Human identity:** the page sends `X-Operator: human`. It is an honour-system header on a
  trusted network: the guide tells agents never to send it. It is not an authentication
  mechanism.

## Sessions on the page

The bar above the dialogue shows the live session and offers:
- **New session** (the human drives, avatar 🧑) and **End session**;
- **Clear view**: hides the turns shown so far, deleting nothing;
- **Export**: a zip of `session.json`, `log.jsonl` and `thumbs/`;
- **Sessions ▾**: the stored transcripts, newest first, with turns and size, each with **View**
  (read-only replay from the thumbnails, with a "Back to live" banner), **Export** and
  **Delete** (human only, confirmed, never the live session).

Every turn carries its `session` id, so a new session starts a fresh dialogue. The API side is
`GET /sessions`, `GET /sessions/{id}` (metadata + turns), `GET /sessions/{id}/export.zip`, and
`DELETE /sessions/{id}` (human).

## Chat commands (page, human)

`help` lists them:
- `look [scale] [grid]`
- `region <shape> [exclude <shape>…] [on sN]`
- `plane …`
- `approach qN [standoff]`
- `check x y [θ] [frame]`
- `goal x y [θ] [frame] [key=value…]`
- `stop`, `enable`, `disable`, `release`, `estop [reason]`
- `note …`
- `place name [x y θ [frame]]`

Shapes are `point u v`, `disk u v r`, `box u0 v0 u1 v1` or `poly u,v u,v u,v…`. Angles take
radians, or degrees as `90d`. Commands run exactly like the agent's calls and appear in the
dialogue as the human's turns.

## Endpoints

See `guide.md` (Reference) and `/docs`. In addition:
- `GET /log?since=` (live turns), `GET /events` (SSE);
- `GET /sessions`, `GET /sessions/{id}/log.jsonl`, `GET /sessions/{id}/thumbs/{file}`;
- human only (`X-Operator: human`): `POST /release`, `POST /profile/{name}`, `POST /reflex?enabled=`.

## Options

`--pose-servo`, `--perception`, `--estimator` (instance names or URLs), `--profile cautious|normal`,
`--data-dir`, `--no-require-imu` (simulation only: lets goals proceed without an IMU-fused heading).

## Simulation

`robot_utils.nodes.agent_nav.sim` holds a synthetic scene: floor, vertical rectangles (whiteboard)
and boxes in the world frame, rendered through the robot's real colour calibration and camera
mount. It also has `fake_perception_app(scene, pose_fn)`, which serves `/calibration` and `/frame`
like 3d_perception. tests/test_agent_nav.py runs the whole stack on it: swerve sim, estimator
(odometry only), pose_servo, the fake perception and agent_nav. Tests cover the plane fit to the
board, loud path checks, an approach plus goal reached, human holds and chat, and the reflex
stopping before a box.

## Known limits

- The reflex and path checks see only the camera's forward view, beyond about 0.7–0.9 m.
  Depth misses glass, black or shiny surfaces and thin objects.
- Images other than the session thumbnails live in memory only (the last 200 renders); a
  restart keeps only the transcript and thumbnails.
- `look {scale: 1.0}` costs 3d_perception about 4× the registration time while it lasts.
