"""The frontend's FastAPI app: landing page, tools, node start/stop, and a server-side E-STOP fan-out.

On the robot everything is one origin on port 80: nginx (``robot-proxy.service``) serves this
app (unix socket ``frontend.sock``) at ``/`` and each node instance at ``/nodes/<name>/``
straight from its own socket (docs/nodes.md). The page and the tools therefore only make
same-origin requests (``nodes/<name>/...``), and heavy node streams never pass through Python.

This app also has its own ``/nodes/<name>/<path>`` proxy. nginx uses it as a fallback when a
node's socket does not answer (so a down node gets a JSON 502 instead of nginx's error page),
and it serves nodes on other machines (``host:`` in nodes.yaml). When the frontend runs alone
(``--port``, no nginx), it is the only route to the nodes.

``POST /api/estop`` is the same e-stop fan-out as the button, for scripts
(``curl -X POST http://<robot>/api/estop``).
"""

from __future__ import annotations

import asyncio
import html
import os
import re
import subprocess
import time
from contextlib import asynccontextmanager
from importlib import resources
from urllib.parse import urljoin

import httpx
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from starlette.background import BackgroundTask

from ..nodes.common.serve import socket_path
from .config import NodeEntry
from .manager import ManagerError, NodeManager, default_runner

META_TIMEOUT_S = 0.75
ESTOP_TIMEOUT_S = 1.5
# Proxy: connecting must be quick; no read timeout, because /events and /stream never end
# (the browser aborts slow requests itself, which closes the upstream request too).
PROXY_TIMEOUT = httpx.Timeout(connect=1.0, read=None, write=2.0, pool=2.0)
PROXY_REQUEST_HEADERS = ("content-type", "accept", "x-lease-token", "last-event-id")
PROXY_RESPONSE_HEADERS = ("content-type", "cache-control")
TOOL_NAME = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
SERVICE_UNIT = "robot-frontend.service"   # installed by install.sh


async def running_as_service() -> bool:
    """True if this process is the main process of robot-frontend.service (systemd user unit)."""
    try:
        proc = await asyncio.create_subprocess_exec(
            "systemctl", "--user", "show", SERVICE_UNIT, "-p", "MainPID", "--value",
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        out, _ = await asyncio.wait_for(proc.communicate(), 5)
    except (OSError, asyncio.TimeoutError):
        return False
    return out.strip().isdigit() and int(out.strip()) == os.getpid()


async def restart_service() -> None:
    """Ask systemd to restart our own unit; --no-block returns at once, then systemd stops us."""
    proc = await asyncio.create_subprocess_exec("systemctl", "--user", "restart", "--no-block", SERVICE_UNIT)
    await proc.wait()


def _via_socket(node: NodeEntry) -> bool:
    """A local node is reached over its unix socket; one whose socket doesn't exist but has a
    debug ``port`` in nodes.yaml (e.g. a node not yet on the serve helper) over 127.0.0.1:port."""
    return node.host is None and (node.port is None or socket_path(node.name).exists())


def node_base_url(node: NodeEntry) -> str:
    """``http://<name>/`` for a node on this machine (over its unix socket); otherwise
    ``http://host:port/`` (``host`` defaults to 127.0.0.1 for a local node with only a port)."""
    if _via_socket(node):
        return f"http://{node.name}/"
    host = node.host or "127.0.0.1"
    if ":" in host and not host.startswith("["):  # bare IPv6 literal
        host = f"[{host}]"
    return f"http://{host}:{node.port}/"

class NodeLinks:
    """HTTP clients for reaching nodes: one per local node (its unix socket) plus one for remote
    nodes. With ``transport`` (tests) a single client on that transport serves every node."""

    def __init__(self, transport: httpx.AsyncBaseTransport | None = None):
        self._transport = transport
        self._clients: dict[str, httpx.AsyncClient] = {}

    def client(self, node: NodeEntry) -> httpx.AsyncClient:
        sock = self._transport is None and _via_socket(node)
        key = node.name if sock else "*"
        c = self._clients.get(key)
        if c is None:
            transport = self._transport or (httpx.AsyncHTTPTransport(uds=str(socket_path(node.name)))
                                            if sock else None)
            c = self._clients[key] = httpx.AsyncClient(transport=transport, trust_env=False)
        return c

    async def aclose(self) -> None:
        for c in self._clients.values():
            await c.aclose()


def estop_path(meta: dict | None) -> str | None:
    """Where to POST a node's e-stop, or None when its /meta says it has nothing to stop.

    Unknown meta (node unreachable, or /meta missing fields) errs towards trying ``estop``.
    The page's JavaScript (``estopPath`` in page.html) mirrors this.
    """
    if not isinstance(meta, dict):
        return "estop"
    eps = meta.get("endpoints") if isinstance(meta.get("endpoints"), dict) else {}
    if isinstance(eps.get("estop"), str) and eps["estop"]:
        return eps["estop"]
    if meta.get("actuated") is True:
        return "estop"
    if meta.get("actuated") is False or "estop" in eps:
        return None
    return "estop"


async def estop_node(links: NodeLinks, node: NodeEntry) -> dict:
    base, client = node_base_url(node), links.client(node)
    result = {"name": node.name, "url": None, "status": "unreachable", "detail": ""}
    meta = None
    try:
        r = await client.get(urljoin(base, "meta"), timeout=META_TIMEOUT_S)
        if r.status_code == 200:
            meta = r.json()
    except (httpx.HTTPError, ValueError):
        pass  # still try the default path: better a stray POST than a missed stop
    path = estop_path(meta)
    if path is None:
        result.update(status="skipped", detail="not actuated")
        return result
    url = result["url"] = urljoin(base, path)
    try:
        r = await client.post(url, timeout=ESTOP_TIMEOUT_S)
    except httpx.TimeoutException:
        result["detail"] = "timed out"
        return result
    except httpx.HTTPError as e:
        result["detail"] = type(e).__name__
        return result
    if r.status_code >= 400:
        result.update(status="failed", detail=f"HTTP {r.status_code}")
        return result
    try:
        latched = r.json().get("latched")
    except (ValueError, AttributeError):
        latched = None
    if latched is True:
        result.update(status="latched", detail="")
    else:
        result.update(status="failed", detail="reply did not confirm latched")
    return result


async def estop_all(links: NodeLinks, nodes: list[NodeEntry]) -> list[dict]:
    """E-stop every enabled node in parallel; one result dict per node, in config order."""
    targets = [n for n in nodes if n.enabled]
    return list(await asyncio.gather(*(estop_node(links, n) for n in targets)))


def load_page() -> str:
    return resources.files(__package__).joinpath("page.html").read_text(encoding="utf-8")


def list_tools() -> list[dict]:
    """Static tool pages shipped in ``frontend/tools/*.html`` (title and description from the
    page's <title> and <meta name="description">)."""
    out = []
    folder = resources.files(__package__).joinpath("tools")
    if not folder.is_dir():
        return out
    for f in sorted(folder.iterdir(), key=lambda f: f.name):
        if not f.name.endswith(".html") or not TOOL_NAME.match(f.name[:-5]):
            continue
        text = f.read_text(encoding="utf-8")
        title = re.search(r"<title>(.*?)</title>", text, re.S)
        desc = re.search(r'<meta\s+name="description"\s+content="(.*?)"', text, re.S)
        out.append({"name": f.name[:-5], "href": f"tools/{f.name}",
                    "title": html.unescape(title.group(1).strip()) if title else f.name[:-5],
                    "description": html.unescape(desc.group(1).strip()) if desc else ""})
    return out


def create_app(
    nodes: list[NodeEntry],
    *,
    transport: httpx.AsyncBaseTransport | None = None,
    manager: NodeManager | None = None,
    is_service=running_as_service,
    restart=restart_service,
) -> FastAPI:
    """``transport``: one transport for every node (tests); otherwise unix sockets for local
    nodes and TCP for nodes with a ``host``."""
    page = load_page()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        app.state.manager = manager or NodeManager(nodes, default_runner())
        app.state.links = links = NodeLinks(transport)
        try:
            yield
        finally:
            await links.aclose()
        close = getattr(app.state.manager.runner, "close", None)
        if close is not None:   # only the subprocess fallback: its children die with us
            await close()

    app = FastAPI(title="robot frontend", lifespan=lifespan, docs_url=None, redoc_url=None)
    no_cache = {"Cache-Control": "no-store"}

    @app.get("/", response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        return HTMLResponse(page, headers=no_cache)

    @app.get("/api/nodes")
    async def list_nodes() -> JSONResponse:
        return JSONResponse({"nodes": [n.as_dict() for n in nodes]}, headers=no_cache)

    @app.get("/api/health")
    async def health() -> dict:
        return {"ok": True}

    @app.post("/api/estop")
    async def estop() -> JSONResponse:
        results = await estop_all(app.state.links, nodes)
        attempted = [r for r in results if r["status"] != "skipped"]
        ok = bool(attempted) and all(r["status"] == "latched" for r in attempted)
        return JSONResponse({"ok": ok, "results": results}, status_code=200 if ok else 502)

    # -- starting and stopping nodes (manager.py: systemd user units robot-node-<name>) ------

    def manager_error(e: ManagerError) -> JSONResponse:
        return JSONResponse({"detail": e.detail}, status_code=e.status)

    @app.get("/api/managed")
    async def managed() -> JSONResponse:
        return JSONResponse({"nodes": await app.state.manager.status_all()}, headers=no_cache)

    @app.post("/api/nodes/{name}/start")
    async def start_node(name: str) -> JSONResponse:
        try:
            return JSONResponse(await app.state.manager.start(name))
        except ManagerError as e:
            return manager_error(e)

    @app.post("/api/nodes/{name}/stop")
    async def stop_node(name: str) -> JSONResponse:
        try:
            return JSONResponse(await app.state.manager.stop(name))
        except ManagerError as e:
            return manager_error(e)

    @app.get("/api/nodes/{name}/logs")
    async def node_logs(name: str, lines: int = 40) -> JSONResponse:
        try:
            return JSONResponse({"lines": await app.state.manager.logs(name, lines)}, headers=no_cache)
        except ManagerError as e:
            return manager_error(e)

    # -- restarting this frontend (reloads nodes.yaml and the code) ----------------------------

    started = time.time()

    @app.get("/api/frontend")
    async def frontend_info() -> JSONResponse:
        return JSONResponse({"service": await is_service(), "unit": SERVICE_UNIT, "pid": os.getpid(),
                             "started": started}, headers=no_cache)

    @app.post("/api/frontend/restart")
    async def frontend_restart() -> JSONResponse:
        """Restart robot-frontend.service. Only works when this process is that service (started
        by install.sh), not when run by hand. Managed nodes are separate units and keep running."""
        if not await is_service():
            return JSONResponse({"detail": f"this frontend is not running as {SERVICE_UNIT} "
                                           "(started by hand?); restart it where it runs"}, status_code=409)

        async def later():
            await asyncio.sleep(0.3)   # let this reply reach the browser first
            await restart()
        return JSONResponse({"restarting": True, "pid": os.getpid(), "started": started}, status_code=202,
                            background=BackgroundTask(later))

    @app.get("/api/tools")
    async def tools() -> JSONResponse:
        return JSONResponse({"tools": list_tools()}, headers=no_cache)

    @app.get("/tools/{name}.html", response_class=HTMLResponse)
    async def tool(name: str) -> HTMLResponse:
        f = resources.files(__package__).joinpath("tools", f"{name}.html")
        if not TOOL_NAME.match(name) or not f.is_file():
            raise HTTPException(404, f"no tool {name!r}")
        return HTMLResponse(f.read_text(encoding="utf-8"), headers=no_cache)

    by_name = {n.name: n for n in nodes if n.enabled}

    @app.get("/nodes/{name}")
    async def node_root(name: str):
        from fastapi.responses import RedirectResponse
        return RedirectResponse(f"{name}/", status_code=301)

    @app.api_route("/nodes/{name}/{path:path}", methods=["GET", "POST", "DELETE", "PUT", "PATCH"])
    async def proxy(name: str, path: str, request: Request):
        node = by_name.get(name)
        if node is None:
            return JSONResponse({"detail": f"no enabled node {name!r} in nodes.yaml"}, status_code=404)
        url = urljoin(node_base_url(node), path)
        headers = {k: v for k, v in request.headers.items() if k.lower() in PROXY_REQUEST_HEADERS}
        headers["x-forwarded-prefix"] = f"/nodes/{name}"
        client: httpx.AsyncClient = app.state.links.client(node)
        upstream = client.build_request(request.method, url, params=request.query_params,
                                        headers=headers, content=await request.body(),
                                        timeout=PROXY_TIMEOUT)
        try:
            r = await client.send(upstream, stream=True)
        except httpx.HTTPError as e:
            where = "its socket" if node.host is None else f"{node.host}:{node.port}"
            return JSONResponse({"detail": f"node {name} is not running ({where} does not answer: "
                                           f"{type(e).__name__})"}, status_code=502)
        out_headers = {k: v for k, v in r.headers.items() if k.lower() in PROXY_RESPONSE_HEADERS}
        return StreamingResponse(r.aiter_bytes(), status_code=r.status_code, headers=out_headers,
                                 background=BackgroundTask(r.aclose))

    return app
