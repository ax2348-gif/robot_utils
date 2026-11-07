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

## Node registry (`nodes.yaml`)

The default is packaged at `src/robot_utils/frontend/nodes.yaml`; `--config PATH` replaces it.

```yaml
nodes:
  - {name: camera, port: 8101}
  - {name: swerve, port: 8102}
  - {name: arm, port: 8103, enabled: false}        # future
  - {name: estimator, port: 8110, enabled: false}  # future
```

| key | | |
|---|---|---|
| `name` | required | short id; the tile's title until the node's `/meta` answers |
| `port` | required | the node's HTTP port |
| `host` | optional | omit for "same machine as the frontend": the page then uses the hostname the browser reached the frontend by, so links work from any machine. Set it only for a node on another machine (never `localhost`) |
| `enabled` | optional, default `true` | `false`: listed under "Planned", not polled, not e-stopped |

Adding a node is adding a line; no frontend code changes. Restart the frontend to reload.

## How it works

- `GET /` — the page (inline HTML/CSS/JS, no CDN).
- `GET /api/nodes` — the configured list: `{"nodes": [{"name", "port", "host", "enabled"}]}`
  (`host: null` = the browser's own hostname).
- `POST /api/estop` — the same e-stop fan-out as the button, run from the frontend (for
  scripts, or a browser that can reach port 80 but not the node ports). 200 with `"ok": true`
  only if every actuated node confirmed `latched`; otherwise 502 with per-node results.
- `GET /api/health` — `{"ok": true}`.

**Status** is polled by the browser, straight from each node, about once a second
(`GET /meta` once per (re)connection, then `GET <endpoints.state>`). The frontend never
contacts a node for status. A node that does not answer in 0.9 s shows **DOWN**.

**Tile link** = `http://` + `window.location.hostname` (or the configured `host`) + `:port/`
+ `meta.page`. Hidden when `meta.has_page` is `false`.

**E-STOP ALL** posts, from the browser, in parallel, to every enabled node whose last `/meta`
says it is actuated (or names an `estop` endpoint). Nodes whose `/meta` was never read are
tried at `estop` anyway: a stray POST is better than a missed stop. Each node gets a line:
`LATCHED` (the reply said `{"latched": true}`), `FAILED` (HTTP error, or a reply without
`latched: true`), `UNREACHABLE` (no reply within 1.5 s), or `SKIPPED` (not actuated). The
banner turns red unless every actuated node latched. The button never asks to confirm.
Releasing an e-stop is done on each node's own page.

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
