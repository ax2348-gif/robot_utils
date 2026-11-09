# Locks: chocks and covers

The robot's equivalent of a wheel chock (and a lens cap), modelled on industrial **lockout/tagout**.
A lock is a small file on the robot (`~/.local/state/robot_utils/locks/<id>.json`): what it covers,
who placed it, why and when. Files survive restarts and reboots, and the nodes themselves enforce
them.

| kind | targets | effect |
|---|---|---|
| **chock** | `swerve`, `arm`, an instance name, `all` | Stops the node at once and refuses `/enable` (HTTP 423, "chocked by …"). |
| **cover** | `camera`, an instance name, `all` | Captures no images: colour, depth and IR serve procedural frames. |

**What keeps working while locked:**
- **chock:** telemetry, the API and the page. Everything that drives through the swerve is also
  covered: pose_servo, agent_nav, and the description node's yaw calibration.
- **cover:** every endpoint, page and consumer works on the procedural frames, at the configured
  sizes and rates. Frame headers and `/state` carry `covered: true`, and 3d_perception passes the
  flag on. The **IMU keeps running**, because the estimator's heading depends on it.

**How enforcement works:**
- **chock:** the swerve node checks for chocks on every control tick, stopping and disabling even
  if the web server stalls. The check costs at most a directory stat every 0.2 s.
- **cover:** the camera's capture thread stops the video pipeline and publishes the procedural
  frames in its place. Its calibration stays available, so consumers keep working.

**Several people can each place their own lock.** A target is free again only when every lock on
it has been removed.

## Placing

- **Landing page, open to anyone:** the **Chock all actuators** and **Cover camera** buttons ask for
  a reason, as does `POST /api/locks {kind, target, reason}`. The placer is recorded by Tailscale
  identity. To keep a runaway agent from spamming locks:
  - an identical active lock is reused;
  - each client may place one lock per 30 s and 10 per day;
  - at most 5 web-placed locks can be active at once.
- **Over SSH, unlimited:**

```bash
.venv/bin/python -m robot_utils.admin.locks chock swerve --reason "battery swap"
.venv/bin/python -m robot_utils.admin.locks cover camera --reason "privacy: meeting in the lab"
```

## Removing (admin = anyone with SSH)

```bash
.venv/bin/python -m robot_utils.admin.locks status
.venv/bin/python -m robot_utils.admin.locks remove <id>
.venv/bin/python -m robot_utils.admin.locks remove --all [--kind chock|cover]
```

`DELETE /api/locks/<id>` always answers 403 with this command. Removal is deliberately not possible
from the web.
