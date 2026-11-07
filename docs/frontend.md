# Robot frontend (landing page)

`python -m robot_utils.frontend` serves one page at `http://<robot>/` that indexes the robot's
HTTP nodes: a tile per node (up/down, title and kind, state, error, lease holder, e-stop) and a
global **E-STOP ALL** button.

**It is not on the control path.** Each node (camera 8101, swerve 8102, …) is its own process
with its own page and its own e-stop at `http://<robot>:<port>/`. The frontend holds no state
and relays nothing in normal use: if it is stopped or crashes, nodes, their pages and their
e-stops keep working, and `curl -X POST http://<robot>:<port>/estop` still stops a node.

## Running

```bash
source .venv/bin/activate
python -m robot_utils.frontend                      # 0.0.0.0:80, packaged nodes.yaml
python -m robot_utils.frontend --port 8080          # no port-80 setup needed
python -m robot_utils.frontend --config ~/nodes.yaml
```

Port 80 as a normal user needs a one-time kernel setting (below). Without it the frontend
exits with a message naming the script; `--port 8080` works meanwhile.

### As a service (the robot)

`./install.sh` installs and starts the landing page as a **systemd user service**,
`robot-frontend.service` (unit in `~/.config/systemd/user/`, like the robot's
`ups-dashboard.service`): it runs `.venv/bin/python -m robot_utils.frontend` from the checkout,
restarts on failure, and starts at boot when lingering is enabled for the user
(`loginctl show-user $USER -p Linger`; `install.sh` prints `sudo loginctl enable-linger $USER`
if not). Re-running `install.sh` rewrites and restarts it, so it always serves this
checkout's code; `./install.sh --no-service` leaves it alone.

```bash
systemctl --user status robot-frontend        # running?
journalctl --user -u robot-frontend -f        # logs
systemctl --user restart robot-frontend       # after editing nodes.yaml or the code
systemctl --user disable --now robot-frontend # stop it and don't start at boot
```

To run it by hand instead (e.g. with `--config`), stop the service first: both want port 80.

## Node registry (`nodes.yaml`)

The default is packaged at `src/robot_utils/frontend/nodes.yaml`; `--config PATH` replaces it.

```yaml
nodes:
  - {name: camera, port: 8101, start: {module: robot_utils.nodes.orbbec}}
  - {name: swerve, port: 8102, start: {module: robot_utils.nodes.swerve}}
  - {name: arm, port: 8103, enabled: false}        # future
  - {name: estimator, port: 8110, enabled: false}  # future
```

| key | | |
|---|---|---|
| `name` | required | short id; the tile's title until the node's `/meta` answers |
| `port` | required | the node's HTTP port |
| `host` | optional | omit for "same machine as the frontend": the page then uses the hostname the browser reached the frontend by, so links work from any machine. Set it only for a node on another machine (never `localhost`) |
| `enabled` | optional, default `true` | `false`: listed under "Planned", not polled, not e-stopped |
| `start` | optional | `{module: ..., args: [...]}`: the landing page can start/stop this node (below). Local nodes only (no `host`) |

Adding a node is adding a line; no frontend code changes. Restart the frontend to reload.

## How it works

- `GET /` — the page (inline HTML/CSS/JS, no CDN).
- `GET /api/nodes` — the configured list: `{"nodes": [{"name", "port", "host", "enabled"}]}`
  (`host: null` = the browser's own hostname).
- `POST /api/estop` — the same e-stop fan-out as the button, run from the frontend (for
  scripts, or a browser that can reach port 80 but not the node ports). 200 with `"ok": true`
  only if every actuated node confirmed `latched`; otherwise 502 with per-node results.
- `GET /api/health` — `{"ok": true}`.
- `/nodes/<name>/<path>` — **proxy** to the node over the robot's loopback (GET, POST,
  DELETE, PUT, PATCH; forwards only `Content-Type`, `Accept`, `X-Lease-Token`,
  `Last-Event-ID`; streams the reply, so `/events` works too). 502 if the node is down, 404
  for a node that is not enabled in `nodes.yaml`.
- `GET /api/tools`, `GET /tools/<name>.html` — the tool pages (below).

**Why a proxy.** The first version polled and e-stopped the nodes straight from the browser
(nodes allow CORS). In practice browsers may refuse or stall cross-port requests to
local-network addresses even with CORS allowed: the tiles showed DOWN while the nodes answered
and their pages opened. So every request the landing page and the tools make is same-origin,
to this frontend, which forwards it. The proxy is for the nodes' small JSON API (and SSE);
node pages and their video/raw streams are still opened directly on the node port (the tile
link), so no bulk data passes through the frontend.

**Status** is polled through the proxy about once a second (`nodes/<name>/meta` once per
(re)connection, then `nodes/<name>/<endpoints.state>`). A node that does not answer in 0.9 s
shows **DOWN**, with the reason (`ConnectError` = nothing listening on that port).

**Tile link** = `http://` + `window.location.hostname` (or the configured `host`) + `:port/`
+ `meta.page`. Hidden when `meta.has_page` is `false`.

**E-STOP ALL** posts, in parallel, through the proxy, to every enabled node whose last `/meta`
says it is actuated (or names an `estop` endpoint). Nodes whose `/meta` was never read are
tried at `estop` anyway: a stray POST is better than a missed stop. Each node gets a line:
`LATCHED` (the reply said `{"latched": true}`), `FAILED` (HTTP error, or a reply without
`latched: true`), `UNREACHABLE` (no reply within 1.5 s), or `SKIPPED` (not actuated). The
banner turns red unless every actuated node latched. The button never asks to confirm.
Releasing an e-stop is done on each node's own page (or a tool page, by the lease holder).
If the frontend itself is down, each node's own page and `POST /estop` still work.

## Starting and stopping nodes

A node with a `start:` recipe gets a control row on its tile: a status pill, **Start** or
**Stop**, and **logs**. The frontend runs each such node as a **systemd user unit**,
`robot-node-<name>.service`, started with
`systemd-run --user --unit robot-node-<name> ... <frontend's python> -m <module> <args>`
(the frontend's interpreter is the robot venv; working directory = the frontend's).

The scope is deliberately small (`frontend/manager.py`):

- **Start** is refused (409) if that unit is already running, or if **anything already listens
  on the node's port**, e.g. a node started by hand in a terminal. The frontend never looks for
  such processes; the tile just shows **running (not managed)** with Start disabled.
- **Stop** stops that unit (`systemctl --user stop`: SIGTERM, so the node shuts down cleanly
  and the swerve node disables the base; SIGKILL after 10 s). It works on the unit whoever
  started it, and refuses (409) if the unit isn't running. Hand-started nodes are stopped by
  hand.
- **No restart policy.** A node that exits stays down; a failed unit shows **managed: failed**
  with its systemd result, and its logs open on the tile. The next Start clears it.
- **Managed nodes outlive the frontend**: each unit has its own lifetime, so restarting the
  frontend service (as `install.sh` does) leaves running nodes alone, and the frontend picks
  them up again from systemd.
- **Anyone** who can reach the landing page can start and stop nodes (for now).

Pill states: `stopped`, `managed: starting…`, `managed: running`, `managed: stopping…`,
`managed: failed (<result>)`, `running (not managed)`. Starting the camera shows a reminder
to keep the robot still for ~2 s (startup IMU calibration).

Endpoints: `GET /api/managed` (every node: `startable`, `unit`, `managed.state`, `pid`, `since`,
`result`, `port_in_use`), `POST /api/nodes/<name>/start`, `POST /api/nodes/<name>/stop`,
`GET /api/nodes/<name>/logs?lines=40` (the unit's journal). Errors are `{"detail": ...}` with
400 (no recipe), 404 (unknown or disabled node), 409 (already running / not running).

On the robot, by hand:

```bash
systemctl --user list-units 'robot-node-*'      # what the landing page started
journalctl --user -u robot-node-swerve -f       # a node's log
systemctl --user stop robot-node-swerve         # same as the Stop button
```

Without a systemd user session (development on macOS) the same controls run nodes as plain
child processes of the frontend instead; those stop when the frontend stops.

## Tools

Static pages in `src/robot_utils/frontend/tools/*.html`, listed in a **Tools** section on the
landing page (title and description from the page's `<title>` and
`<meta name="description">`). They call nodes only through the proxy, with URLs relative to
the page (`../nodes/<name>/...`), so they need no CORS and work under any host or port. To add
one, drop an `.html` file in that folder (names: lowercase letters, digits, `_`, `-`).

- **Swerve teleop** (`tools/swerve_teleop.html`), ported from rev_ctl `examples/teleop.py`:
  move joystick, rotate slider, page speed limits (capped at the node's soft limits), Enable,
  Disable, E-STOP and Clear e-stop. It takes the swerve node's lease ("Take control" if someone
  else holds it), POSTs the twist 20 times a second while enabled (each command renews the
  lease; the node stops 0.25 s after the last one), renews the lease twice a second while
  disabled, disables when the page is hidden and releases the lease when it is closed (the
  node's 1 s lease lapse is the backstop). The orange "achieved" ring is computed in the page
  from the per-module wheel angles and speeds, with the same least-squares fit as
  `robot_twist()` (the node does not serve a measured twist).

## Node contract

What a node serves to appear on the page (paths relative to the node's root; nodes must send
permissive CORS headers, since the browser calls them from the frontend's origin):

- `GET /meta` → `{"name", "title", "version", "kind", "has_page", "page", "actuated",
  "endpoints": {"health", "state", "events", "estop"}}`. `page` is relative to the node root
  (usually `"./"`); `endpoints.estop` is `null` for non-actuated nodes (camera). Missing
  fields are tolerated: the tile falls back to the config name, `state`, `estop` and `./`.
- `GET /health` → `200 {"ok": true, ...}` (not used by the page; for scripts and monitors).
- `GET /state` → at least `{"state": str, "error": str|null}`; actuated nodes also
  `"lease": {"held": bool, "holder": str|null}` and `"estop": {"latched": bool, ...}`.
  A tile is **UP** when this returns 2xx JSON.
- `POST /estop` (no body, no auth) → `200 {"latched": true}`.
- Its own page at `meta.page`, using relative URLs (served via `http://<robot>:<port>/`).

If a node does not send CORS headers, its e-stop POST may still reach it but the browser
cannot read the reply, so it shows as `UNREACHABLE`; its tile will show `LATCHED` on the next
poll only if `/state` is readable, which it is not without CORS either. CORS is required.

## Port 80 without root

Once per machine, as the user (the frontend never runs sudo, and neither does `install.sh`,
which only prints this when needed):

```bash
sudo tools/enable_low_ports.sh
```

It sets `net.ipv4.ip_unprivileged_port_start=80` immediately (`sysctl -w`) and persists it in
`/etc/sysctl.d/60-robot-utils-ports.conf`. Idempotent; refuses to run without root; prints
what it changed. Ports 1–79 (ssh, …) stay privileged. Undo: delete that file and
`sudo sysctl -w net.ipv4.ip_unprivileged_port_start=1024`.

## Tests

```bash
PYTHONPATH=src python -m pytest tests/test_frontend.py
```

No hardware or network: fake nodes are `httpx.MockTransport` handlers.
