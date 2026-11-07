# Robot nodes: shared conventions

The robot runs one **node** per hardware bus: a process that owns that hardware and serves
an HTTP API. Shared code lives in `robot_utils.nodes.common`; each node adds its own
endpoints. There is no ROS and no message broker; clients talk to nodes directly over HTTP.

| node | package | default port | actuated | guide |
|---|---|---|---|---|
| camera (Orbbec Gemini 336L + IMU) | `robot_utils.nodes.orbbec` | 8101 | no | [orbbec_node.md](orbbec_node.md) |
| swerve base | `robot_utils.nodes.swerve` | 8102 | yes | [swerve_node.md](swerve_node.md) |
| arm (planned) | | 8103 | yes | |
| estimator (pose: wheel odometry + IMU heading) | `robot_utils.nodes.estimator` | 8110 | no | [estimator_node.md](estimator_node.md) |

Ports: nodes use **81xx**; **80xx** (and 80) is left for a frontend dashboard. Port 8081 on
the Pi is taken by the UPS dashboard. Every node binds `0.0.0.0` by default.

## Design rules

- **One hardware-owner thread per node.** Only that thread calls the hardware library; the
  async web server never does, and never blocks its event loop. Owner → web handoff:
  `common.hub` (thread-safe, wakes the event loop, no polling).
- **One uvicorn worker.** Several workers would be several processes fighting over the device.
- **No recording on the Pi** (small, slow disk). Clients record the streams they receive; the
  stream bytes are the recording format.
- **Processing of one device's data stays in its node** (e.g. the camera's IMU attitude filter);
  fusion across nodes (odometry + IMU, ...) belongs to an estimator node.
- **Commands are idempotent state, not events**: the latest command wins (see the swerve
  CommandSlot). Optional `seq` numbers are for debugging only.

## Time

Every timestamp a node publishes is on the **Pi's `time.monotonic()`** (CLOCK_MONOTONIC):
system wide, so all nodes on the Pi share one clock, and it never steps. Clients on other
machines synchronise with `GET /time` (`{mono, wall, epoch_minus_mono}`) NTP-style:

```
t0 = client_now(); ts = GET /time .mono; t1 = client_now()
offset = ts - (t0 + t1) / 2        # server = client + offset
uncertainty = (t1 - t0) / 2        # hard bound
```

Take ~16 samples over a kept-alive connection and intersect the bounds of the fastest ones:
`common.client.NodeClient.sync()` (Python), `RU.TimeSync` (browser). `common.timesync.ClockSync`
also fits clock drift for long sessions.

## Endpoints every node has

| endpoint | what |
|---|---|
| `GET /time` | server clocks for sync |
| `GET /health` | `200 {"ok": true, ...}` |
| `GET /meta` | what the node is (below) |
| `GET /state` | JSON with at least `state` (string) and `error` (string or null) |
| `GET /events` | Server-Sent Events: `/state` on every change and every second |
| `GET /` | the node's own page (relative URLs, uses the toolkit) |
| `GET /docs` | interactive OpenAPI docs |
| `GET /common/ui.js`, `/common/ui.css` | the shared browser toolkit |

CORS is open on every route (including preflight), so a dashboard on another port can call any
node from the browser. Errors are `{"detail": "..."}` with a meaningful status (400 bad input,
401/403 lease, 409 conflict / e-stop / not enabled, 503 hardware not running, 422 validation).

### `GET /meta`

```json
{"name": "swerve", "title": "Swerve base", "version": "0.1.0", "kind": "swerve",
 "has_page": true, "page": "./", "actuated": true,
 "endpoints": {"health": "health", "state": "state", "events": "events", "estop": "estop"}}
```

All paths are **relative to the node root**; `estop` is `null` for nodes that can't move
anything. Extra keys are allowed. Build it with `common.server.node_meta(...)` and serve it with
`common.server.add_page_routes(app, meta)` (which also serves the toolkit).

### Actuated nodes: lease and e-stop

`GET /state` of an actuated node also has `"lease": {"held": bool, "holder": str|null, ...}` and
`"estop": {"latched": bool, "reason", "source", "t"}`.

- **Lease** (`common.lease.Lease`): one controller at a time. `POST /lease {"client": "name"}` →
  `{"token", "heartbeat"}` (409 if held). Send the token as the `X-Lease-Token` header with every
  controlling request. It lapses after `heartbeat` seconds (1 s) without `POST /lease/renew` or
  an accepted command; `DELETE /lease` releases it. When a lease ends for any reason the node
  stops and disables the hardware and resets per-lease settings (e.g. soft limits). The node
  checks for a lapse on its control thread every tick, so a stuck web server can't keep a dead
  client in control.
- **E-stop** (`common.estop.EStop`): `POST /estop` (no body, no lease, any client; optional
  `{"reason", "source"}`) → `200 {"latched": true, ...}`. It stops at once and stays latched until
  the lease holder calls `POST /estop/clear`; clearing does not re-enable (`POST /enable`). The
  lease lapse is a second, independent stop; the hardware's own watchdogs are a third.

## Streams (binary records)

Streaming endpoints return one long-lived HTTP response: a sequence of records

```
[u32 big-endian header length N][N bytes JSON header][header.len bytes payload]
```

`common.protocol` (Python) and `RU.RecordReader` (browser) parse it in any chunking.
Record `type`s:

| type | payload | header |
|---|---|---|
| `ping` | none | `t_send`; sent after ~1 s without data |
| `schema` | none | `schema` (id), `fields`, `units`, `dtype` `"<f8"`, optional `kind`; first record of a rows stream |
| `rows` | little-endian float64 `(n, len(fields))` | `n`, `schema`, `dropped`, `t_send`, optional `kind` |
| `frame` / `imu` | camera-specific (see orbbec_node.md); `imu` records carry `fields` inline | |

Every-sample streams (swerve ticks, camera IMU) deliver **every** sample; a client that falls
too far behind loses the oldest rows and the next record's `dropped` says how many. `batch_ms`
(query parameter) coalesces rows per client. Video is latest-frame-wins instead.

Record on the client by writing the response bytes to a file; read it back with
`common.protocol.iter_records` / `iter_rows`:

```python
from robot_utils.nodes.common.client import NodeClient
from robot_utils.nodes.common.protocol import iter_records, iter_rows
NodeClient("http://botcore-desktop:8102").record("/stream", "run1.bin", seconds=30)
for header, rows in iter_rows(iter_records(open("run1.bin", "rb"))):
    print(rows["t_tick"][0], rows["FL_drive_vel"].mean())
```

## Command paths (actuated nodes)

**Slow path (implemented):** one JSON `POST` per command, e.g. `POST /command
{"twist": [vx, vy, omega], "seq": 0}`. Easy from `curl`, a browser or a script; resend it faster
than the command timeout (0.25 s on the swerve) to keep moving.

**Fast path (not implemented yet; where it plugs in):** a streaming endpoint for scripts, e.g.
`POST /command/stream` with a chunked body of newline-delimited JSON commands (same model as
`/command`). Its handler would:

1. check the lease token once, when the stream opens (`X-Lease-Token`);
2. for every received line, call the same `node.command(token, twist, seq, source="fast")` the
   slow path calls, which writes the same **CommandSlot** (a latest-command register) and renews
   the lease. Reading the body as it arrives and only ever keeping the newest command means a
   burst after a network hiccup collapses to its last command ("drain");
3. end the response (closing the stream) as soon as the lease ends or lapses, or the e-stop
   latches (`node.command` raises then).

The control tick doesn't care which path wrote the slot: last write wins, and the same lease
holder may use both paths at once (e.g. a script streaming while a browser jogs). The source of
the command in force is in every telemetry row (`cmd_source`: 1 slow, 2 fast) next to its `seq`.

## Browser toolkit (`common/ui.js`)

Node pages include `<link rel="stylesheet" href="common/ui.css">` and
`<script src="common/ui.js"></script>` and use **relative URLs only** (`"state"`, not `"/state"`),
so a page works on the node's own port and behind a reverse proxy under `/nodes/<name>/`.
`window.RU` provides:

| | |
|---|---|
| `RU.api(path, {method, body, headers})` | JSON call; throws `Error("status: detail")` |
| `new RU.TimeSync().sync()` | NTP-style sync: `.offset`, `.uncertainty`, `.serverNow()` |
| `RU.RecordReader`, `RU.RowsDecoder` | stream parsing (`decoded.col(name)`, `.last(name)`) |
| `RU.stream(path, onRecord)`, `RU.rows(path, onRows)` | read a binary stream, reconnecting |
| `RU.sse(path, onJson)` | Server-Sent Events of JSON |
| `RU.leaseWidget(el)` | take / release / auto-renew the lease; `.headers()`, `.update(state)` |
| `RU.estopButton(el)` | e-stop button showing the latched state; `.update(state)` |
| `new RU.Plot(canvas, {seconds, series})` | rolling line plot: `.push(t, values)`, `.draw()` |

## Adding a node

1. A hardware-owner thread (or a library that runs one, like `SwerveDrive`) plus a small core
   class with the node's state; no web code in it.
2. `create_app`: `setup_app(app)` (CORS, error mapping), `add_base_routes(app, health)`,
   `add_page_routes(app, node_meta(...))`, `/state`, `/events` (`common.server.sse`), streams
   (`common.server.rows_stream` for every-sample data), and for actuated nodes the lease and
   e-stop endpoints from `common.lease` / `common.estop`.
3. `python -m robot_utils.nodes.<name>` with `--host 0.0.0.0 --port 81xx`; a `--sim`/`--fake`
   mode so the API can be tried and tested without hardware.
4. A page at `/` using the toolkit and relative URLs; docs `docs/<name>_node.md`.
