# Arm fast path, v1: draft

Status (2026-11-21): decided (see "Local only" below); being built as the arm's internal joint channel, first
client the WBC (`wbc_v1.md`).

## Goal

A low-latency, joint-space command path into the arm node, for controllers that close the loop faster than HTTP
allows: a neural "system 0" on the Pi (measured: MLPs up to ~2M parameters run in 0.3–2 ms per 10 ms tick under the
core load), a corrective layer on erid over Wi-Fi (measured round trip with HTTP `/track`: 20 ms = two ticks), and
later a dedicated radio link (ELRS or similar). The same commands from all three; the arm node does not care which
transport carried them.

Out of scope for v1: the base (same pattern later), torque-level commands, replacing `/chunk` (policies at 20–30 Hz
keep using timestamped chunks over HTTP).

## Local only (decided 2026-11-21; supersedes the transport and session parts below)

The arm's joint-space input is a **low-level, robot-internal interface**. The principle that the public API does not
expose joint (qpos) control to the outside world stays pinned, and holds structurally:

- **Transport: a Unix datagram socket only**, in the robot's runtime directory, owner-only permissions. **No UDP, no
  HTTP route** (nothing through nginx or any page, nothing in OpenAPI or the agent guide). Only processes on the Pi
  running as the robot user can reach it.
- **Session over the same socket:** a hello message carrying the arm lease token opens it (holding the lease is
  necessary, being on the robot is too); no `/fast/open` HTTP endpoints. Bye, lease end, e-stop, disable or node
  shutdown close it.
- **Clients are internal components:** the WBC now, an on-Pi system 0 later, a future radio relay (a deliberately
  installed local process). Documented in the developer docs as an internal interface, not the public API.
- **The fast path for outside clients is one level up, at the WBC:** end-effector targets, hints and chunks in the
  world frame, over datagrams from the Mac or erid. External clients drive the robot through the WBC or the existing
  pose-level arm API, never through joints.
- **Clean inside the arm node:** a generic joint-space source with the review's constraints (below), plus one small
  transport module. The packet layouts below stay; the "UDP on the robot LAN" and HTTP session parts do not apply.

## Decisions already made

- **Datagrams, not WebSockets/TCP.** Newest command wins; a lost packet costs one command, never a stall (TCP would
  hold newer commands behind a retransmission). Each packet is self-contained.
- **Control plane over HTTP, data plane over datagrams.** Lease, enable, opening and closing a stream session stay
  HTTP requests with the existing checks (lease, e-stop, chocks, enabled). Only the per-tick commands and state are
  datagrams.
- **One packet format, several transports:** a Unix datagram socket on the Pi (on-Pi system 0, the radio relay),
  UDP on the robot LAN (erid). A radio relay process on the Pi forwards radio frames into the Unix socket.
- **Joint space, no IK** in the hot path. The node keeps its safety envelope: every goal goes through
  `_command_step` (joint speed and acceleration limits, joint limits) exactly as track and planned motions do.
- **Stale stream = hold.** No fresh command within the staleness timeout: the arm holds where its command is, one
  event, the session stays open; the next fresh command resumes.
- **Plain sockets and `struct`**, no new dependencies (no ROS 2/DDS, ZeroMQ, protobuf).

## Protocol (v1)

Little-endian, fixed size. Unknown `version` or bad `magic` is dropped (counted).

**Command, client -> node, 88 bytes** (`<4sBBHIIdd6f6fff`):

| field | type | meaning |
|---|---|---|
| magic | 4s | `b"RUC1"` |
| version | u8 | 1 |
| mode | u8 | 0 = keep-alive (renews, no goal change), 1 = joint goal |
| flags | u16 | reserved (0) |
| session | u32 | from `POST /fast/open` |
| seq | u32 | strictly increasing per session; older or equal is dropped (counted, not an error) |
| t_send | f64 | sender's own clock (diagnostics only) |
| t_basis | f64 | `t_tick` of the state packet the command was computed from (Pi clock), or NaN: lets the node measure command latency in its own clock |
| q | 6×f32 | joint position goal (rad, output side) |
| qd | 6×f32 | velocity feedforward (rad/s), bounded by the soft joint speed |
| gap | f32 | gripper gap goal (m), NaN = unchanged |
| reserved | f32 | 0 (candidate: stiffness scale, see open decisions) |

**State, node -> client, every tick while a session is open** (`<4sBBHIIdIBB2x6f6f6f6fff`, 136 bytes): magic
`b"RUS1"`, version, flags (servoing, stale, jam, limited), session, tick, `t_tick`, seq in force, source, state,
q (output encoders), qd, τ estimate, q_cmd, gripper gap, read_age. The radio relay may send a subset.

## Session lifecycle

- `POST /fast/open` (lease token, enabled) -> `{session, unix_socket, udp_port, stale_ms, limits}`. One session at a
  time (the lease is exclusive already); opening again replaces it.
- State packets go to the address the latest valid command came from (Unix: the client's bound socket path; UDP:
  its source address), so a client must send (a keep-alive is enough) before it receives.
- Every valid command renews the lease, like HTTP commands do today. Lease end, e-stop, disable, `POST /fast/close`
  or node shutdown close the session; later packets are dropped (counted).
- Access: the session id comes from the lease (same trust level as the lease token in HTTP headers today); the UDP
  socket binds only the robot LAN address, never Tailscale or 0.0.0.0.

## Arm node integration

- **Receiver thread:** one thread, `selectors` over the Unix and UDP sockets; per packet `struct.unpack`, session and
  seq check, bounds check (finite; goal within the hard limits plus a small tolerance, else drop and count), then
  under `_lock` write the newest command into one slot. Microseconds of work: minimal GIL time beside the 100 Hz loop.
- **A new source `stream`** in `SOURCES`:
  - `_has_source()` includes a stream slot; `_arm_engaged()` therefore covers a stream accepted but not yet engaged
    (the race fixed in `bf1e3ce` must not come back).
  - `_source()` returns `("stream", q, qd, gap)` from the slot; precedence with track and planned motions: the newest
    accepted command preempts (`_preempt("stream")`), and track/servo/chunk preempt the stream, as today.
  - Staleness checked in `_source()`: past `stale_ms` the goal becomes the current `q_cmd` (hold), `stale` flag set,
    one event; cleared on the next fresh command.
  - `_drop_sources()` (disable, e-stop, lease end, fault) clears the stream slot.
- **State out:** sent from `_publish()` (after the tick, same data as the row), non-blocking `sendto`; failures are
  counted, never raised into the tick.
- **Observability:** new row columns `stream_seq`, `stream_age` (s since the command in force arrived) and
  `stream_lat` (tick time minus `t_basis`); `/state` gains the session, counters (accepted, stale, dropped by reason)
  and the client address.

## Constraints from the code review (2026-11-21)

From a review of `nodes/arm` against this draft; each needs a decision or a fix before or with the fast path.

- **`SOURCES`: append `stream` at the end** (the row's `source` column is an index; inserting renumbers recordings).
  `_source_name()` duplicates `_source()`'s precedence and must change with it.
- **Gate at write time, under `_lock`:** the receiver writes the slot only if `enabled and motors_on and not estop`,
  as `track_cmd` does; otherwise a datagram landing after a disable engages on the next enable.
- **Validate in the receiver,** not on engage: finite values, hard limits, and the engage limit check
  (`_limit_problem`). A refusal inside `_engage` adds a warn event per tick at 100 Hz (flushes the 50-entry event log,
  makes every SSE client re-serialise `state_dict`). Never `events.add` per packet; rate-limit.
- **A lead bound:** `max |q_goal − q_meas|` per joint (track has `TRACK_MAX_JUMP` 20°, planned motions `_supervise`),
  else `q_cmd` can run the full range away from an obstructed arm. Plus a finite-value backstop in `_command_step`
  (a NaN goal would make `q_cmd` NaN permanently).
- **Feedforward `qd` is not acceleration-limited** (only scaled to `joint_vel`): limit its rate of change for
  network outputs.
- **Stale = hold is an instant brake** at the current speed; use a short ramp (decay `qd`, ease the goal) instead.
- **Jam:** treat `stream` as report-only like `track`; don't bump `_cmd_count` per datagram.
- **Lease:** a token-free `renew_holder()` for the receiver; catch the renew race (expiry between check and update).
  `_lease_ended`/`_disable` close the session and may run on any thread.
- **Coexistence with HTTP motions:** decide explicitly; at least `/stow`, `/home`, e-stop must not be preempted by
  a stream (proposal: a planned motion pauses the session until the client re-arms it).
- **`seq` lives in the session,** not the command slot (survives hold and cancel).
- **State packets every tick, including fault/connecting:** `_publish` is skipped on fault and enable ticks today,
  so a client would see silence instead of `state=fault`. Send right after `bus.read()` (saves several ms vs after
  `bus.send`); non-blocking, errors counted (it runs on the SCHED_FIFO thread).
- **Timestamps:** `t_tick` is the tick start, before a pipelined read whose output-encoder replies were requested
  about one tick earlier (motor-side feedback is newer). `t_basis`/`stream_lat` inherit that bias. The row now has
  `t_meas` (the request time the output-encoder replies answer) and `t_meas_mot` (the motor feedback's arrival):
  carry them in the state packet and use them, not `t_tick`, before training anything on these timings.
- **Output vs motor side:** goals are output-encoder space, `q_cmd` is seeded from the motor side; a static per-joint
  offset is visible to a policy closing its loop on `q`.
- **Sim:** `SimBus` reads synchronously with zero delay; a pipelined fake bus is needed for realistic fast-path tests.

## Measurement plan

- Unit tests with the sim bus and manual ticks: preemption both ways, staleness hold and resume, out-of-order and
  duplicate seq, bad session, lease end closing the session, e-stop, bounds rejection, the engaged-before-tick case.
- A UDP variant of `erid_loop_probe.py`: the same closed loop as the HTTP `/track` test (20 ms p50, 24 ms p99), to
  measure what dropping HTTP and IK buys over Wi-Fi.
- An on-Pi client pinned to its own core running the 2M MLP at 100 Hz through the Unix socket, under the colour-only
  core load: arm 100 Hz, zero overruns is the bar.

## Open decisions

- `stale_ms` default (proposal: 30 ms = 3 ticks).
- Allow a stiffness scale per command (bounded to the soft profile)? Useful for system 0, more to verify.
- Timestamped commands (`t_apply`: apply at a Pi-clock time) for latency compensation, or keep "next tick" in v1?
- Torque feedforward from system 0: not in v1?
- Per-packet keyed signature (a few µs) or session id only?
- Gripper in the stream, or gripper stays on HTTP?

## Order of work (proposed)

1. Protocol module (`nodes/common/fastpath.py`): pack/unpack, constants, tests.
2. Arm node: session endpoints, receiver thread, `stream` source, staleness, state out, row columns; sim tests.
3. Clients: a small Python client (Unix and UDP) and the UDP loop probe; on-Pi MLP client.
4. Robot tests (stationary first, as before), then the radio relay when a model needs erid's GPU.
