"""The frontend's FastAPI app: landing page, tools, node proxy and a server-side E-STOP fan-out.

Browser pages never call node ports cross-origin: some browsers refuse or stall cross-port
requests to local-network addresses even with CORS allowed (the landing tiles showed DOWN while
the nodes answered). Instead ``/nodes/<name>/<path>`` forwards to the node over the robot's
own loopback, so the landing page, its E-STOP and the tool pages only make same-origin
requests. The proxy is meant for the nodes' small JSON API (and SSE); node pages and their
video/raw streams are still opened directly on the node port ("Open page"), so no bulk data
passes through here. This process still holds no state and is not on the control path.

``POST /api/estop`` is the same e-stop fan-out run from here, for scripts
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


def node_base_url(node: NodeEntry, default_host: str) -> str:
    host = node.host or default_host
    if ":" in host and not host.startswith("["):  # bare IPv6 literal
        host = f"[{host}]"
    return f"http://{host}:{node.port}/"


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


async def estop_node(client: httpx.AsyncClient, node: NodeEntry, default_host: str) -> dict:
    base = node_base_url(node, default_host)
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


async def estop_all(client: httpx.AsyncClient, nodes: list[NodeEntry], default_host: str) -> list[dict]:
    """E-stop every enabled node in parallel; one result dict per node, in config order."""
    targets = [n for n in nodes if n.enabled]
    return list(await asyncio.gather(*(estop_node(client, n, default_host) for n in targets)))


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
    default_host: str = "127.0.0.1",
    transport: httpx.AsyncBaseTransport | None = None,
    manager: NodeManager | None = None,
    is_service=running_as_service,
    restart=restart_service,
) -> FastAPI:
    """``default_host``: how this process reaches nodes without a ``host`` (server-side e-stop
    only; the browser uses its own hostname). ``transport``: for tests."""
    page = load_page()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        app.state.manager = manager or NodeManager(nodes, default_runner())
        async with httpx.AsyncClient(transport=transport, trust_env=False) as client:
            app.state.client = client
            yield
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
        # host null = "the hostname you reached me by"; the page fills it in.
        return JSONResponse({"nodes": [n.as_dict() for n in nodes]}, headers=no_cache)

    @app.get("/api/health")
    async def health() -> dict:
        return {"ok": True}

    @app.post("/api/estop")
    async def estop() -> JSONResponse:
        results = await estop_all(app.state.client, nodes, default_host)
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

    @app.api_route("/nodes/{name}/{path:path}", methods=["GET", "POST", "DELETE", "PUT", "PATCH"])
    async def proxy(name: str, path: str, request: Request):
        node = by_name.get(name)
        if node is None:
            raise HTTPException(404, f"no enabled node {name!r} in nodes.yaml")
        url = urljoin(node_base_url(node, default_host), path)
        headers = {k: v for k, v in request.headers.items() if k.lower() in PROXY_REQUEST_HEADERS}
        client: httpx.AsyncClient = app.state.client
        upstream = client.build_request(request.method, url, params=request.query_params,
                                        headers=headers, content=await request.body(),
                                        timeout=PROXY_TIMEOUT)
        try:
            r = await client.send(upstream, stream=True)
        except httpx.HTTPError as e:
            return JSONResponse({"detail": f"node {name} unreachable: {type(e).__name__}"}, status_code=502)
        out_headers = {k: v for k, v in r.headers.items() if k.lower() in PROXY_RESPONSE_HEADERS}
        return StreamingResponse(r.aiter_bytes(), status_code=r.status_code, headers=out_headers,
                                 background=BackgroundTask(r.aclose))

    return app
