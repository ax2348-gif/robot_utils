# Robot frontend (landing page)

`python -m robot_utils.frontend` serves one page at `http://<robot>/` that indexes the robot's
HTTP nodes: a tile per node (up/down, title and kind, state, error, lease holder, e-stop) and a
global **E-STOP ALL** button.

On the robot it sits behind the port-80 proxy (nginx, `robot-proxy.service`): `/` is this
page (unix socket `frontend.sock`), `/nodes/<name>/` each node instance straight from its own
socket (docs/nodes.md, Routing).

**It is not on the control path.** Each node is its own process with its own page and its own
e-stop at `http://<robot>/nodes/<name>/`, served by nginx straight from the node's socket. The
frontend holds no state: if it is stopped or crashes, nodes, their pages and their e-stops keep
working, and `curl -X POST http://<robot>/nodes/swerve/estop` still stops a node.

## Running

```bash
source .venv/bin/activate
python -m robot_utils.frontend                      # unix socket frontend.sock (behind nginx)
python -m robot_utils.frontend --port 8080          # also TCP 8080: without nginx / debugging
python -m robot_utils.frontend --config ~/nodes.yaml
```

Without nginx, `--port` makes the landing page reachable on its own, and its built-in
`/nodes/<name>/` proxy then reaches the nodes' sockets (all traffic through Python: fine for
debugging, not for heavy streams). `--port 80` as a normal user needs a one-time kernel setting
(below).

### As a service (the robot)

`./install.sh` installs and starts two **systemd user services** (units in
`~/.config/systemd/user/`, like the robot's `ups-dashboard.service`; restart on failure; start at
boot when lingering is enabled for the user, `install.sh` prints `sudo loginctl enable-linger
$USER` if not):

- `robot-frontend.service`: `.venv/bin/python -m robot_utils.frontend` from the checkout, on
  `frontend.sock`.
- `robot-proxy.service`: nginx on port 80 (config generated into
  `~/.config/robot_utils/nginx/nginx.conf` by `python -m robot_utils.frontend.proxy`; pid, logs,
  temp under `~/.local/state/robot_utils/nginx`). It needs the `nginx` package and the system
  `nginx.service` disabled; if either is missing, `install.sh` prints the `sudo` commands and
  runs the landing page with `--port 80` instead, so the robot stays reachable meanwhile.

Re-running `install.sh` rewrites and restarts both (frontend first, then the proxy), so they
always serve this checkout's code; `./install.sh --no-service` leaves them alone.

```bash
systemctl --user status robot-frontend        # running?
journalctl --user -u robot-frontend -f        # logs
systemctl --user restart robot-frontend       # after editing nodes.yaml or the code
systemctl --user status robot-proxy           # nginx; logs: journalctl --user -u robot-proxy
systemctl --user disable --now robot-frontend # stop it and don't start at boot
```

To run the landing page by hand instead (e.g. with `--config`), stop the service first: both
want `frontend.sock` (a second one refuses: "already running").

The landing page's **Restart landing page** button (next to the hostname) does the same as
`systemctl --user restart robot-frontend` (e.g. after editing `nodes.yaml` or pulling new code):
`POST /api/frontend/restart` replies 202, then asks systemd for a non-blocking restart of its own
unit; the page waits until the new process answers (`GET /api/frontend` reports its start time)
and reloads. It refuses (409) when the frontend was started by hand rather than as the service.
Nodes keep running: managed ones are separate units.

## Node registry (`nodes.yaml`)

The default is packaged at `src/robot_utils/frontend/nodes.yaml`; `--config PATH` replaces it.

Entries are **instances**: the same kind can run twice under different names (e.g. two cameras).

```yaml
nodes:
  - {name: camera, kind: camera, start: {module: robot_utils.nodes.orbbec}}
  - {name: swerve, kind: swerve, start: {module: robot_utils.nodes.swerve}}
  - {name: arm, kind: arm, enabled: false}        # future
  - {name: estimator, kind: estimator, start: {module: robot_utils.nodes.estimator}}
  - {name: description, kind: description, start: {module: robot_utils.nodes.description}}
  - {name: pose_servo, kind: pose_servo, start: {module: robot_utils.nodes.pose_servo}}
  - {name: 3d_perception, kind: 3d_perception, start: {module: robot_utils.nodes.perception3d}}
  - {name: agent_nav, kind: agent_nav, start: {module: robot_utils.nodes.agent_nav}}
  # a second camera would be e.g.
  # - {name: camera2, kind: camera, start: {module: robot_utils.nodes.orbbec, args: [...]}}
```

| key | | |
|---|---|---|
| `name` | required | instance name: socket `<runtime dir>/robot/<name>.sock`, URL `/nodes/<name>/`, tile title until `/meta` answers |
| `kind` | optional, default = `name` | what the node is (camera, swerve, ...) |
| `port` | optional | a local instance's TCP port, used only while its socket doesn't exist (a node not on the `serve` helper), and as an "already up" check |
| `host` | optional, with `port` | a node on another machine, reached through the frontend's own proxy |
| `enabled` | optional, default `true` | `false`: listed under "Planned", not polled, not e-stopped |
| `start` | optional | `{module: ..., args: [...]}`: the landing page can start/stop this node (below). Local nodes only (no `host`) |

Adding a node is adding a line; no frontend code changes. Restart the frontend to reload.

## How it works

- `GET /` — the page (inline HTML/CSS/JS, no CDN).
- `GET /api/nodes` — the configured list: `{"nodes": [{"name", "kind", "port", "host", "enabled", "start"}]}`.
- `POST /api/estop` — the same e-stop fan-out as the button, run from the frontend (for
  scripts, or a browser that can reach port 80 but not the node ports). 200 with `"ok": true`
  only if every actuated node confirmed `latched`; otherwise 502 with per-node results.
- `GET /api/health` — `{"ok": true}`.
- `/nodes/<name>/<path>` — the frontend's own **proxy** to the node's socket (or `host:port`)
  (GET, POST, DELETE, PUT, PATCH; forwards only `Content-Type`, `Accept`, `X-Lease-Token`,
  `Last-Event-ID`, adds `X-Forwarded-Prefix`; streams the reply). Behind nginx it only sees what
  nginx hands it: requests for a node whose socket does not answer (→ JSON 502 "not running")
  and nodes on other machines. Without nginx (`--port`) it carries all node traffic. 404 for a
  node that is not enabled in `nodes.yaml`.
- `GET /api/tools`, `GET /tools/<name>.html` — the tool pages (below).

**Why one origin.** Browsers may refuse or stall cross-port requests to local-network
addresses even with CORS allowed (the first version's tiles showed DOWN while the nodes
answered), so every request the page and the tools make is same-origin: `nodes/<name>/...`.
nginx serves those straight from the node's socket, so node pages and their video/raw streams
cost the frontend nothing.

**Status** is polled through the proxy about once a second (`nodes/<name>/meta` once per
(re)connection, then `nodes/<name>/<endpoints.state>`). A node that does not answer in 0.9 s
shows **DOWN**, with the reason ("not running" = its socket does not answer).

**Tile link** = `nodes/<name>/` + `meta.page` on the same origin. Hidden when
`meta.has_page` is `false`.

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
`systemd-run --user --unit robot-node-<name> ... <frontend's python> -m <module> --name <name> <args>`
(the frontend's interpreter is the robot venv; working directory = the frontend's).

The scope is deliberately small (`frontend/manager.py`):

- **Start** is refused (409) if that unit is already running, or if **the instance's socket
  already answers** (or its configured `port` is in use), e.g. a node started by hand in a
  terminal. The frontend never looks for
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
`result`, `listening`, `socket`), `POST /api/nodes/<name>/start`, `POST /api/nodes/<name>/stop`,
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

- **Remote teleop** (`tools/remote_teleop.html`): the swerve teleop's controls and logic over the
  camera's live view, for driving without line of sight. The background is the camera node's
  active stream (color if on, else depth, then IR), shown with `<img src="http://<host>:<port>/mjpeg/<stream>">`
  straight from the camera's port (from `api/nodes`), so the video never passes through the
  frontend's proxy; it follows the camera's settings (polled every 2 s) and shows "camera: …"
  in the corner. Controls are hollow (thick coloured outlines, transparent, shadowed text) and the
  layout fits the screen without scrolling (sizes scale with the viewport; checked from 320×568 to
  1920×1080, portrait and landscape).
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

What a node serves to appear on the page (paths relative to the node's root, which is
`/nodes/<name>/` behind nginx):

- `GET /meta` → `{"name", "title", "version", "kind", "has_page", "page", "actuated",
  "endpoints": {"health", "state", "events", "estop"}}`. `page` is relative to the node root
  (usually `"./"`); `endpoints.estop` is `null` for non-actuated nodes (camera). Missing
  fields are tolerated: the tile falls back to the config name, `state`, `estop` and `./`.
- `GET /health` → `200 {"ok": true, ...}` (not used by the page; for scripts and monitors).
- `GET /state` → at least `{"state": str, "error": str|null}`; actuated nodes also
  `"lease": {"held": bool, "holder": str|null}` and `"estop": {"latched": bool, ...}`.
  A tile is **UP** when this returns 2xx JSON.
- `POST /estop` (no body, no auth) → `200 {"latched": true}`.
- Its own page at `meta.page`, using relative URLs (served at `http://<robot>/nodes/<name>/`).
- It listens on its unix socket via `robot_utils.nodes.common.serve` (`--name`), which also sets
  `/meta`'s `name` to the instance.

Everything is same-origin now, so CORS is no longer needed for the landing page (nodes still
send it, for scripts and other tools).

## Who's here: observability (read-only)

The landing page has a **Who's here** panel and a matching API; admin stays on SSH.

- **Access log:** nginx writes one JSON object per finished request to
  `~/.local/state/robot_utils/nginx/access.log`: time, client address, method, URI, status, bytes,
  duration, the node, and the user agent. Requests are logged when they *end*, so a stream that's
  still open shows up in the live columns below.
- **Not logged:** successful status polls (`GET .../state`, `health`, `meta`, `time`, `fields`,
  `calibration`, and the landing page's own `/api/...` polls). Pages make these every second, and
  they were about 80 % of the volume (about 385 MB/day). Who is connected and how much they pull
  still shows live, and streams, pages, every POST and every error are logged.
- **Identity:** Tailscale addresses can't be forged over the tailnet, and `tailscale whois` maps them
  to the device and its owner (cached 10 min). LAN addresses are shown as they are, with no
  identity.
- **Live:** every 2 s, `ss` on port 80 gives each client's open connections, current send rate and
  send-queue backlog. A large backlog means the client can't keep up with its stream.
- **Uplink:** ping to 1.1.1.1 and to the gateway every 10 s, plus interface rates. The panel shows a
  banner when the 1-minute median is over 150 ms or pings are lost.
- **Retention:** the frontend rotates `access.log` daily (or past 100 MB) and `error.log` past
  20 MB, has nginx reopen them (SIGUSR1), and deletes rotated logs older than **7 days**.

| request | what |
|---|---|
| `GET /api/observe` | clients (identity, live connections / send rate / backlog, last hour's requests, bytes, nodes, last request), uplink, log info |
| `GET /api/observe/requests?limit=&ip=&node=` | the most recent finished requests, newest first, with identities |

From a terminal on the robot:

```bash
.venv/bin/python -m robot_utils.frontend.observe report --hours 24    # per client: requests, MB, nodes, top paths
.venv/bin/python -m robot_utils.frontend.observe rotate                # rotate now (normally automatic)
tail -f ~/.local/state/robot_utils/nginx/access.log | jq -c '{t, ip, uri, status, bytes}'
```

`--no-observe` starts the frontend without the observer.

## Editing pages

Pages are HTML and JavaScript strings, isolated from the nodes' logic:
- node pages in `nodes/<node>/page.py` (and `page_viz.py`);
- the shared toolkit in `nodes/common/ui_assets.py`;
- the landing page and tools in `frontend/*.html`.

The robot's guards are all server-side, so no page edit can defeat them: speed limits, the 0.25 s
command timeout, leases, e-stop latches, chocks, pose_servo's caps, agent_nav's profiles. Some page
code still matters for safety, though. When restyling:
- **Restyle through CSS and markup.** Keep element ids and the wiring the contracts check:
  - `#estop` and `RU.estopButton($("#estop"))` on the swerve, pose_servo and description pages;
  - agent_nav's and the teleop tools' own e-stop handlers;
  - the landing page's E-STOP ALL (`estopAll` / `estopOne`) and lock buttons.
- **Treat the toolkit's functions as an API.** Change how `RU.estopButton`, `RU.leaseWidget` (and its
  300 ms renewal) and `RU.api` look, not what they do.
- **Never drop a teleop stop path:**
  - the swerve page sends commands only while a key is held, and releases on key-up, window blur and
    pointer-up/leave;
  - the teleop tools zero a stick on pointer-up, cancel or lost capture, and disable the base when
    the tab is hidden or the page closes.
- **Run `tests/test_pages_contract.py`.** It pins all of the above and checks that every page's
  JavaScript parses (`node --check`; install Node.js in the test environment with
  `conda install -c conda-forge nodejs`, or the check is skipped).

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
PYTHONPATH=src python -m pytest tests/test_frontend.py tests/test_frontend_manager.py tests/test_frontend_proxy.py
```

No hardware or network: fake nodes are `httpx.MockTransport` handlers; the manager's end-to-end
test runs a stand-in node through `serve`; the nginx config test also runs `nginx -t` when an
nginx binary is available (on `PATH` or `NGINX_BIN`).
