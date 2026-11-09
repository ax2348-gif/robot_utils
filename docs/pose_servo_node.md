# Pose servo node

`python -m robot_utils.nodes.pose_servo` drives the swerve base to a pose (x, y, heading) with
smooth, jerk-limited motion. It reads the **estimator node's** pose and the **swerve node's**
telemetry and sends twists to the swerve node. Shared conventions (time, framing, `/meta`, the
browser toolkit): [nodes.md](nodes.md).

- Page: `http://<robot>/nodes/pose_servo/`: map (click to place a goal), controls, goal and config forms, plots of the twist sent, its rate and the error
- Interactive API docs: `http://<robot>/nodes/pose_servo/docs`

## Quick start

On the robot, with the swerve and estimator nodes running:

```bash
cd ~/robot_utils && .venv/bin/python -m robot_utils.nodes.pose_servo     # instance 'pose_servo'
```

Options: `--swerve`, `--estimator` (instance names or URLs; defaults `swerve` / `estimator`),
`--name`, `--port` (debugging).

```bash
N=http://botcore/nodes/pose_servo
curl -s -X POST $N/enable -H 'content-type: application/json' \
     -d '{"client": "me", "config": {"max_speed": 0.15, "max_accel": 0.2, "max_distance": 1.0}}'
curl -s -X POST $N/goal -H 'content-type: application/json' -d '{"x": 0.5, "theta": 1.57}'   # local
curl -s $N/state | python -m json.tool | head -40          # status, goal.result, ...
curl -s -X POST $N/disable                                  # stops at once, releases the swerve
```

```python
from robot_utils.nodes.common.client import NodeClient
with NodeClient("http://botcore/nodes/pose_servo") as ps:          # on the robot: NodeClient("pose_servo")
    ps.post("/enable", {"client": "script", "config": {"max_speed": 0.2}})
    ps.post("/goal", {"x": 0.3, "y": 0.1, "theta": 0.5, "speed": 0.1})
    # poll ps.get("/state")["goal"]["result"]: None while moving, then reached / stopped / aborted
```

Without hardware: swerve node with `--sim`, estimator with `--camera none` (the fake camera's IMU
never turns) and `"require_imu": false` in the enable config.

## Control and ownership

| request | what |
|---|---|
| `POST /enable {client, config?}` | Acquire the swerve lease (holder `pose_servo (<client>)`), set the swerve's soft limits to the caps, `POST /enable` the swerve. 409 if already enabled, the lease is held by someone else, or the swerve's e-stop is latched. The config (defaults + `config`) lasts until disable. |
| `POST /disable` | Immediate: no braking. Swerve `/disable` (it stops) and lease released. |
| `POST /goal {frame, x, y, theta, …}` | Start a motion, or replace the one in progress (it continues from the current velocity and acceleration, never assuming rest). |
| `POST /stop` | Brake to rest within the caps, drop the goal, stay enabled. |
| `POST /estop {reason?, source?}` | Anyone: forwarded to the swerve (it latches and stops at once), then disable. |
| `GET/POST /config` | The config in force and the hard limits. Changes are allowed only while enabled and idle. |
| `GET /state`, `/events` | `status` (disabled / idle / moving / holding / braking), `goal` (resolved goal, motion, `result` = null while active, then reached / stopped / aborted with a `reason`), `config`, `estimator` health, swerve state, `violations`, loop rate, latest tick, events. |
| `GET /stream`, `/fields` | Every control tick (50 Hz) as binary float64 rows. |
| `GET /meta`, `/health`, `/time`, `/docs`, `/` | As every node. |

pose_servo has no lease of its own. It holds the swerve's lease while enabled: it sends a twist
every tick while moving and renews the lease while idle. To take over, disable it, or e-stop the
swerve. The swerve's own `/enable` / `/disable` are unchanged. If the swerve reports an e-stop,
another lease, or disabled, pose_servo disables itself.

### Goals

- `frame` defaults to **`local`**: x forward, y left, relative to the pose when the request
  arrives; `theta` is the turn, taken **literally** (`2π` = one full turn).
- `global` uses the estimator's world frame; `theta` is reached by the **shortest** turn.
- **Rejected with 422:**
  - a goal farther than `max_distance` from the current pose;
  - a goal turning more than `max_rotation`;
  - a per-goal option above its cap;
  - an unknown option.
- **Other refusals:** 409 if not enabled; 503 if the estimator pose is unusable.
- **Per-goal options** override the config's motion defaults for this goal only:
  - `mode`, `pos_tol`, `heading_tol`, `on_reach`, `hold_band`;
  - limits `speed`, `omega`, `accel`, `alpha`, `jerk`, `ang_jerk`, each ≤ its cap. They shape
    the motion; the caps stay the hard bound. Lowering `speed` mid-motion decelerates within the
    limits rather than jumping.

### Config (per enable; defaults restored on disable)

| key | default | |
|---|---|---|
| `max_speed`, `max_omega` | 0.3 m/s, 0.8 rad/s | caps on every twist sent (≤ 2.0 / 4.0, the swerve's hard limits); also set as the swerve's soft limits |
| `max_accel`, `max_alpha` | 0.5 m/s², 1.5 rad/s² | caps on the twist's rate of change |
| `max_jerk`, `max_ang_jerk` | 2.0 m/s³, 6.0 rad/s³ | caps on the rate of that |
| `max_distance`, `max_rotation` | 3.0 m, π rad | goal rejection |
| `require_imu` | true | abort when the estimator's heading is not IMU-fused |
| `mode` | `synchronized` | or `independent` |
| `pos_tol`, `heading_tol` | 0.02 m, 0.035 rad | acceptance region |
| `on_reach` | `terminate` | or `hold` |
| `hold_band` | 2 | once inside its tolerance, an axis moves again only beyond `hold_band` × the tolerance |

**Fixed in code:**
- every wheel ≤ **3 m/s**;
- every wheel's steering ≤ **4π rad/s**;
- `max_speed + max_omega × 0.234 m ≤ 3 m/s` is required, so the wheel-speed limit always holds.

## How it works

The controller is `planner.py`: pure code, tested without I/O.

**What is constrained.** The servo integrates its own commanded body twist `u = (vx, vy, ω)` and
its rate `a`. The constraints apply to the twists actually sent, step by step, at the 50 Hz
period:
- `|(vx, vy)| ≤ max_speed`, `|ω| ≤ max_omega`;
- `|Δu/dt|` ≤ the acceleration caps;
- `|Δa/dt|` ≤ the jerk caps;
- each wheel's speed;
- each wheel's steering rate: the change of direction of its velocity, mod π (a module may flip
  instead of turning), counted only for wheels moving at least 2 cm/s in both twists. Slower than
  that a wheel has no defined direction; the swerve then keeps its angle or turns it at hardware
  speed while scaling the drive by cos(error).

The base's real motion lags these commands; the constraints are on what is sent.

**Planning, every tick (receding horizon).**
1. **Translation:** a desired world velocity toward the goal, at the highest speed from which a
   jerk-limited stop ends at the goal. The stop is planned with 80 % of the motion's
   acceleration and jerk, keeping the rest as headroom for tracking. Close to the goal the speed
   is limited to `distance / 0.4 s`, a linear final approach.
2. **Heading:** the same, in 1-D, on the continuous heading.
3. **Synchronized mode:** each axis's time to the acceptance region is predicted in closed form
   (S-curve plus the final approach, from its current velocity). The faster axis's speed,
   acceleration and jerk are scaled by s, s², s³ (a time scaling) so both arrive together. Both
   finish within about 0.1 s of each other in simulation. In independent mode, each axis goes
   as fast as its limits allow.
4. **Tracking:** the desired body twist is tracked by a jerk-limited velocity law. The
   acceleration aims at the velocity error left after unwinding the current acceleration
   (`a|a|/2J`), capped by the acceleration, `√(2J|e|)` and `|e| / 0.1 s`.
5. **Steering shaping:** the part of the planned acceleration perpendicular to the current twist
   turns the wheels (wheel velocities are linear in the twist). It is scaled down so no wheel
   would steer faster than half the limit. The check looks ahead at the lowest speed each wheel
   reaches while the current acceleration unwinds; for example, the turning centre may sweep past
   a wheel while translating and rotating.

**Limiter: the guarantee.**
1. The planned jerk is clipped to the jerk ball, and the resulting acceleration is projected
   onto the acceleration ball. The projection cannot increase the jerk.
2. The step is accepted only if it keeps the speed, wheel-speed and steering limits, **and**
   unwinding its acceleration afterwards would keep them all the way to constant velocity. That
   unwinding is the fallback the limiter takes when nothing else passes.
3. Otherwise the largest share of the step between the fallback (share 0) and the plan that
   passes is taken.

Every accepted state therefore has an allowed next step, and the sequence cannot be driven into
a violation. Each sent step is verified again and any violation is counted (`violations`,
expected 0; none has occurred in testing).

**Acceptance region.**
- Each axis latches *done* inside its tolerance; it then only brakes and holds.
- It is released only beyond `hold_band` × the tolerance, so there is no hunting at the goal.
- With both axes done, `terminate` brakes to an exact zero twist, ends the goal (`reached`) and
  stops sending.
- `hold` keeps sending a zero twist, and an axis pushed out of its band moves back.

**Pose and robustness.**
- The pose used is the estimator's latest, extrapolated to now with the commanded twist (at most
  0.1 s; the estimator lags by about one tick).
- A motion is **aborted** (`result` = aborted with the reason) when any of these happens:
  - the estimator stream is older than 0.2 s;
  - its `reset_id` changes (a `POST /reset` moved the world frame);
  - the pose jumps by more than 0.15 m or 0.3 rad between samples;
  - wheel odometry is missing;
  - with `require_imu`, the heading has not been IMU-fused for 0.2 s.
- An abort brakes to rest within the caps; braking needs no pose.
- New goals are refused with 503 while the pose is unusable.
- If swerve commands fail for 0.2 s, or the lease renew fails, the node disables. The swerve
  itself stops 0.25 s after the last command.

## Known limits

- The hard steering limit (4π rad/s) and wheel limit (3 m/s) are above what the base currently
  does: steering is capped at 0.3 duty (about 7.5 rad/s) and drive at 0.35 duty (about 1.8 m/s).
  In practice the caps you configure are what bound the motion.
- Planar and kinematic. It assumes the base follows the twist; tracking error shows up as pose
  error, which the next ticks correct. Nothing else (obstacles, other robots) is considered.
- The final approach is exponential; the time to reach the tolerance grows slowly as
  `pos_tol` shrinks below about 1 cm.

## Measured (simulation)

- **Kinematic simulation** (tests/test_pose_servo_planner.py):
  - 2,000 random goal sequences: 1–5 goals, each replaced mid-motion, from a random moving
    start, in both modes, across 5 sets of caps from (0.1 m/s, 0.2 m/s²) to (2 m/s, 5 m/s², 50 m/s³).
  - Zero violations; every final goal reached. The test suite runs 300 of them.
  - Speed, acceleration and jerk reach their caps exactly.
  - Synchronized axes finish within 0.12 s of each other.
  - Planner step about 0.02 ms on a Mac.
- **Full stack** (tests/test_pose_servo_node.py): simulated swerve node, estimator (odometry
  only) and pose_servo, as real servers.
  - Local goal (0.4, 0.2, 0.6 rad) reached within 3 cm and 0.05 rad.
  - Every twist the swerve accepted re-checked against the caps: no violation.
  - The control loop runs at about 50 Hz.
