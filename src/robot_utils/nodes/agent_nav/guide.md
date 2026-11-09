# agent_nav: driving the robot as an agent

You are about to drive a real robot: a swerve-drive base with an RGB-D camera, in a room with
people and things. This guide is all you need. Every call is plain HTTP + JSON. `N` below is
this node's base URL, e.g. `http://botcore/nodes/agent_nav` (all paths in replies are relative to it).

## Principles (read these)

1. **You decide; the tools don't.** Perception and planning calls are read-only. They return
   numbers, proposals and images. Nothing moves unless you call `POST /goal`. No call turns
   perception into motion by itself.
2. **Stopping is the only automatic action.** The stop reflex brakes if depth shows an
   obstacle within the braking distance ahead of the motion. It only sees what the camera sees
   (forward, beyond ~0.7–0.9 m). It is a guardrail, not your obstacle avoidance: **you** avoid
   obstacles.
3. **Look before you move, and look at the images.** Every perception and planning reply has an
   overlay image and a bird's-eye view (BEV). Fetch and inspect them. They show what the tool
   actually selected, fitted or checked. If the overlay is not what you meant, your numbers are wrong.
4. **Path checks are loud, not binding.** `/goal` and `/plan/check` report obstacle points in the
   swept footprint, clearance and the share of the path that was never seen. Read the `warnings`.
   An `UNVERIFIED` part of the path, such as the blind zone in front of the robot, is unknown, not clear.
5. **Move in legs you have verified.** Prefer legs of ≲1–1.5 m, then look again. The camera can't
   see the floor within ~0.7–0.9 m of the robot's front: rely on what you saw from earlier positions.
6. **Work asynchronously.** `POST /goal` returns at once. Wait for it in the background
   (`GET /goals/{id}/wait`), and meanwhile look, think, or stop / replace the goal.
7. **Explain yourself.** Every POST accepts an optional `"comment"`. It goes into the transcript
   that humans read. Say why, briefly.
8. **The human is in charge.** If they disable or e-stop on the page, you get `423` until they
   release it. Ask, don't work around. Never send the `X-Operator: human` header: it is for the
   page / a person.

## Workflow

```bash
N=http://botcore/nodes/agent_nav
curl -s $N/guide.md                                                    # this file
curl -s -X POST $N/session -H 'content-type: application/json' -d '{"agent": "Claude", "avatar": "🦉"}'
curl -s -X POST $N/enable  -H 'content-type: application/json' -d '{"comment": "starting the task"}'
```

**1. Look.** It returns a snapshot id, the colour image, a BEV and the pose at capture.

```bash
curl -s -X POST $N/look -H 'content-type: application/json' -d '{"grid": true}'
# -> {"snapshot": "s1", "image": "images/s1.jpg", "bev": "images/s1_bev.png", "pose_world": [x, y, θ], ...}
curl -s -o s1.jpg $N/images/s1.jpg                                      # then view the file
```

The image is 1280x720 colour. `grid: true` draws labelled pixel lines every 100 px, so you can
name coordinates. The depth grid is 640x360 by default; `{"scale": 1.0}` gives full-resolution
depth for fine work. The BEV is top-down in the robot's frame at capture: the robot at the bottom,
x forward (up), y left, 0.5 m grid. Floor points are grey, obstacles orange/yellow by height, the
blue line is the nearest visible floor.

**2. Query what you see.** Select pixels with a point, disk, box or polygon, and optionally exclude shapes.

```bash
curl -s -X POST $N/query/plane -H 'content-type: application/json' -d '{
  "snapshot": "s1", "select": {"polygon": [[380,10],[700,10],[690,150],[390,150]]},
  "exclude": [{"box": [600,0,700,40]}], "comment": "the whiteboard, not the curtain"}'
# -> centre / normal / width / height in base and world frames, touches_image_edge, overlay, bev
```

- `/query/region`: 3D extent (5/50/95 percentiles), centroid, and the nearest point in base and world frames.
- `/query/plane`: robust plane fit: centre, normal (toward the camera), facing heading, width, height,
  rms, inliers. `touches_image_edge: true` means the surface may continue out of view, so its
  centre and width may be biased. Look again from elsewhere, or account for it.

Check the overlay: magenta is your selection, red your exclusions, green the fitted plane. In
the BEV, the selection's footprint has a thick magenta border.

**3. Plan.** Read-only.

```bash
curl -s -X POST $N/plan/approach -H 'content-type: application/json' -d '{"query": "q1", "standoff": 1.2}'
# -> proposal {frame: "snapshot:s1", x, y, theta}, path check, BEV with the swept footprint
curl -s -X POST $N/plan/check -H 'content-type: application/json' -d '{"frame": "local", "x": 0.8, "y": 0.2}'
```

In the check BEV: green footprint = clear, red = obstacle points inside (red dots), grey = never
seen. Proposals are suggestions. Verify them and adjust them yourself.

**4. Move.** Explicitly. Then wait in the background.

```bash
curl -s -X POST $N/goal -H 'content-type: application/json' \
     -d '{"frame": "snapshot:s1", "x": 1.4, "y": 0.2, "theta": -0.5, "comment": "first leg toward the board"}'
# -> {"goal_id": 7, "warnings": [...], "check": {...}, "bev": "...", "wait": "goals/7/wait"}
curl -s "$N/goals/7/wait?timeout=60"          # run this in the background; it returns when the goal ends
```

The result is one of:
- `reached`;
- `stopped` (someone called `/stop`, or the reflex braked);
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
- `on_reach`: `terminate` or `hold`.

**5. Remember.** `POST /memory/note {"text"}` and `POST /memory/place {"name", ...}` (default: here),
then `GET /memory`. Places are lost when the estimator resets.

**6. Finish.** `POST /disable`, then `DELETE /session`.

## Limits and caps

- **Caps:** the human picks the safety profile (`cautious` by default: 0.15 m/s, 0.5 rad/s,
  0.2 m/s², goals ≤ 2 m / 3.2 rad away). You may lower caps (`POST /enable {"caps": {...}}`) but
  never raise them.
- **Rejected goals:** too far or turning too much → `422`. Not enabled → `409`. Human hold → `423`.
  Estimator unusable → `503`.
- **What the camera sees:** forward only, about 82°x66°, with the floor visible from ~0.7–0.9 m out.
  The reflex and the path checks see nothing beside or behind the robot, so turn and look.
- **Depth:** unreliable on glass, mirrors, black or shiny surfaces and thin legs or cables.
  Treat thin things you see in the image as real even when depth misses them.
- **Snapshot age:** snapshots are kept briefly (the last 30). A snapshot from an earlier position is
  still valid in its own frame, but things may have moved since.

## Reference

| call | what |
|---|---|
| `GET /guide.md` | this guide |
| `POST /session {agent, avatar, comment}` / `DELETE /session` | open / close (closing disables) |
| `POST /enable {caps?, comment}` / `POST /disable` | motion on (pose_servo + swerve lease) / off at once |
| `POST /look {scale?, grid?, settle?}` | snapshot: image, BEV, pose at capture |
| `POST /query/region`, `POST /query/plane` `{snapshot?, select, exclude?}` | read-only geometry with overlays |
| `POST /plan/check {frame, x, y, theta, snapshot?}` | path check (loud, advisory) |
| `POST /plan/approach {query, standoff, face}` | proposed goal in front of a surface |
| `POST /goal {frame, x, y, theta, options…, comment}` | **move**; returns at once |
| `GET /goals/{id}`, `GET /goals/{id}/wait?timeout=` | goal state / long-poll until done |
| `POST /stop`, `POST /estop {reason}` | brake / e-stop (latches for the human) |
| `GET /images/{name}`, `GET /images/{snapshot}/grid.jpg` | rendered images |
| `GET /memory`, `POST /memory/note`, `POST /memory/place` | per-session memory |
| `GET /state`, `GET /log?since=`, `GET /sessions` | status, live transcript, stored transcripts |

Selections: `{"point": [u, v]}`, `{"disk": [u, v, r]}`, `{"box": [u0, v0, u1, v1]}` or
`{"polygon": [[u, v], …]}`, in colour-image pixels (1280x720, origin top left).
