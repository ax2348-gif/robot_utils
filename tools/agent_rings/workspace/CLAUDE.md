# Rings sorting: your task

You operate a mobile manipulator (a swerve-drive base with one 6-DOF arm and a parallel gripper) through its HTTP API,
and write whatever code you need to make it do the task below. This is a long session: build up tools, test them,
keep notes, and keep going until the task is done or you have truly exhausted your ideas.

## The task

In front of the robot are **two concentric horizontal rings, turning in opposite directions**, with **red and blue
objects** scattered on both. **Sort them: every red object onto the outer ring, every blue object onto the inner
ring.** Objects already on their right ring can stay. An object that falls off a ring (to the floor, or into the
hollow centre) is lost: be careful, but losing some is not the end.

You know nothing else about the scene: find out the geometry, speeds and everything else by looking with the robot's
cameras and by trying things.

## The robot

- **The robot is HTTP over a unix socket:** `/tmp/ara/robot.sock` (also in `$ROBOT_SOCKET`), the only thing your
  sandbox connects to besides PyPI. Use any host name in URLs, e.g. `curl --unix-socket /tmp/ara/robot.sock
  http://robot/api/nodes`, or in Python `httpx.Client(transport=httpx.HTTPTransport(uds="/tmp/ara/robot.sock"),
  base_url="http://robot")`. Where the docs say `http://botcore/...`, use this instead. Every robot node is at
  `/nodes/<name>/`; each node serves its API documentation at `/nodes/<name>/openapi.json` and a page at
  `/nodes/<name>/`.
- **Nodes:**
  - `agent_interface`: built for agents. Read its briefing first: `GET /nodes/agent_interface/guide.md`.
  - `arm`: the arm and gripper: servo, track and stream motions, IK, state, the lease.
  - `pose_servo`: base motion goals.
  - `swerve`: the base's wheels; low level, mostly for reading.
  - `estimator`: the base's pose.
  - `camera`: a head RGB-D camera; `wrist_camera`: an RGB-D camera on the wrist.
  - `3d_perception`: point clouds and geometry queries from the cameras.
  - `description`: the robot's model.
- **Docs:** copies of the robot's node documentation are in `docs/` here.
- **Recovery:** faults are normal while learning. The arm and base nodes have reset / clear endpoints (see their
  docs); the landing page (`/`) can stop and start robot nodes (`POST /api/nodes/<name>/stop|start`).

## Your machine

- This workspace is yours: write code, notes, logs and images here.
- **Python:** use the virtualenv `.venv` here (`.venv/bin/python`, `.venv/bin/pip install ...`). Install what you need
  (numpy, opencv-python, scipy, httpx, ...).
- **Keep a running log** in `NOTES.md`: what you tried, what you learned, what you're doing next. Commit nothing
  anywhere; there is no git remote.

## Rules (your transcript and every request you make are audited afterwards)

1. **No ground truth.** Use only what the robot itself provides: its cameras, its own state (joints, base pose,
   gripper), its nodes' APIs. The simulator behind it is off limits: don't try to reach its API (`/nodes/sim`), its
   processes, sockets, files, configuration or source code, and don't look for them.
2. **No source code of the robot software or the task.** Don't search the disk for it, and don't read files outside
   this workspace. Use the APIs and the docs in `docs/`.
3. **No neural perception.** No learned detectors, segmenters, depth or pose networks (no torch / tensorflow / onnx
   models, YOLO, SAM, CLIP, ...). Classical vision only: OpenCV, numpy, geometry, colour thresholds, the RGB-D data,
   the `3d_perception` node.
4. **This Mac only.** No other machines, no ssh. Use the internet only to install Python packages.
5. **No reset as a shortcut.** Don't restart the world to undo a mistake. If something you need breaks beyond the
   robot's own recovery, write it in `NOTES.md` and carry on with what you can.

When you think you are done, say so in `NOTES.md` with how many objects you believe are on their right ring, then keep
improving if you can.
