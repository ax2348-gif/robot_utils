"""The frontend's FastAPI app: landing page, node list, and a server-side E-STOP fan-out.

The page polls nodes and sends its E-STOP straight from the browser (nodes allow CORS), so
this process holds no state and relays nothing during normal use. ``POST /api/estop`` is the
same fan-out run from here, for scripts (``curl -X POST http://<robot>/api/estop``) and for
browsers that can reach port 80 but not the node ports.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from importlib import resources
from urllib.parse import urljoin

import httpx
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse

from .config import NodeEntry

META_TIMEOUT_S = 0.75
ESTOP_TIMEOUT_S = 1.5


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


def create_app(
    nodes: list[NodeEntry],
    *,
    default_host: str = "127.0.0.1",
    transport: httpx.AsyncBaseTransport | None = None,
) -> FastAPI:
    """``default_host``: how this process reaches nodes without a ``host`` (server-side e-stop
    only; the browser uses its own hostname). ``transport``: for tests."""
    page = load_page()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        async with httpx.AsyncClient(transport=transport, trust_env=False) as client:
            app.state.client = client
            yield

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

    return app
