"""The agent's only door to the robot: a reverse proxy in front of the agent stack's landing page that forwards the
robot's own nodes and refuses the simulator's privileged API, logging every request (tools/agent_rings/README.md).

    python tools/agent_rings/proxy.py --upstream unix:/tmp/ar/robot/frontend.sock --port 8091 --log LOG.jsonl

Refused (403, logged as such):
- ``/nodes/sim`` and everything under it: object and robot ground truth (``/scene``, ``/state``, ``/scene3d``,
  ``/robot_state``, ``/viz``), teleport, wrenches, markers, ghosts, resets;
- the landing page's ``/api/nodes/sim/*`` (logs, start, stop), ``/api/nodes/description/{start,stop}``,
  ``/api/frontend/restart`` and ``/api/observe*`` (it would show the operator's own requests).

Everything else (``/nodes/<robot node>/...``, the landing page, its other APIs) is forwarded as is, streams
included. The log has one JSON line per request: wall time, method, path with the query, status, bytes, ms,
``blocked``. The agent never sees it.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import posixpath
import re
import time

import httpx
import uvicorn
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import JSONResponse, Response, StreamingResponse
from starlette.routing import Route

BLOCKED = [
    re.compile(r"^/nodes/sim(/|$)"),
    re.compile(r"^/api/nodes/sim(/|$)"),
    re.compile(r"^/api/nodes/description/(start|stop)$"),
    re.compile(r"^/api/frontend/restart$"),
    re.compile(r"^/api/observe"),
]
SESSION = re.compile(r"^/nodes/wbc/session/?$")
HOP = {"connection", "keep-alive", "proxy-authenticate", "proxy-authorization", "te", "trailers",
       "transfer-encoding", "upgrade", "host", "content-length"}


def normalise(path: str) -> str:
    """The path as the robot will route it: percent-decoded (Starlette already did), slashes collapsed, dot segments
    resolved, so ``/nodes//sim``, ``/nodes/./sim`` or ``/x/../nodes/sim`` can't slip past the rules."""
    norm = posixpath.normpath("/" + path.lstrip("/"))
    return norm + ("/" if path.endswith("/") and norm != "/" else "")


def blocked(path: str) -> bool:
    return any(p.search(path.lower()) for p in BLOCKED)


def make_app(upstream: str, log_path: str, rewrite_session: str | None = None) -> Starlette:
    if upstream.startswith("unix:"):
        transport = httpx.AsyncHTTPTransport(uds=upstream[5:])
        base = "http://robot"
    else:
        transport, base = None, upstream.rstrip("/")
    client = httpx.AsyncClient(transport=transport, base_url=base, timeout=httpx.Timeout(30.0, read=None))
    log = open(log_path, "a", buffering=1)

    def write(rec: dict) -> None:
        log.write(json.dumps(rec, separators=(",", ":")) + "\n")

    async def handle(request: Request):
        t0 = time.time()
        path = normalise(request.url.path)
        target = path + (("?" + request.url.query) if request.url.query else "")
        rec = {"t": round(t0, 3), "method": request.method, "path": target}
        if blocked(path):
            write({**rec, "status": 403, "blocked": True})
            return JSONResponse({"detail": "not available to the agent (the simulator's privileged API)"}, 403)
        body = await request.body()
        headers = {k: v for k, v in request.headers.items() if k.lower() not in HOP}
        try:
            req = client.build_request(request.method, target, headers=headers, content=body)
            up = await client.send(req, stream=True)
        except httpx.HTTPError as e:
            write({**rec, "status": 502, "error": str(e)[:200]})
            return JSONResponse({"detail": f"robot unreachable: {e}"}, 502)
        out_headers = {k: v for k, v in up.headers.items() if k.lower() not in HOP}
        if rewrite_session and SESSION.match(path) and up.status_code == 200:
            # the WBC's session names its own target socket (out of the agent's reach): name the relay's instead
            raw = await up.aread()
            await up.aclose()
            try:
                body = json.loads(raw)
                body["unix_socket"] = rewrite_session
                body["udp_port"] = None
                raw = json.dumps(body).encode()
            except ValueError:
                pass
            write({**rec, "status": up.status_code, "bytes": len(raw), "ms": round((time.time() - t0) * 1e3, 1)})
            return Response(raw, status_code=up.status_code, media_type="application/json")
        size = [0]

        async def body_iter():
            try:
                async for chunk in up.aiter_raw():
                    size[0] += len(chunk)
                    yield chunk
            except (httpx.HTTPError, asyncio.CancelledError):
                pass
            finally:
                await up.aclose()
                write({**rec, "status": up.status_code, "bytes": size[0], "ms": round((time.time() - t0) * 1e3, 1)})
        return StreamingResponse(body_iter(), status_code=up.status_code, headers=out_headers)

    methods = ["GET", "POST", "PUT", "DELETE", "PATCH", "HEAD", "OPTIONS"]
    return Starlette(routes=[Route("/{path:path}", handle, methods=methods)])


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--upstream", required=True, help="unix:/path/to/frontend.sock or http://127.0.0.1:PORT")
    p.add_argument("--port", type=int, default=8091)
    p.add_argument("--host", default="::1", help="listen address (default ::1: the agent's sandbox allows that host "
                                                  "only, so other stacks on 127.0.0.1 stay out of its reach)")
    p.add_argument("--uds", default=None, help="listen on this unix socket instead of TCP (the agent's sandbox then "
                                              "allows that one socket: no loopback TCP, so no other stack, is reachable)")
    p.add_argument("--log", required=True)
    p.add_argument("--wbc-socket", default=None,
                   help="the target socket to name in the WBC's POST /session reply (dgram_relay.py's)")
    a = p.parse_args()
    app = make_app(a.upstream, a.log, a.wbc_socket)
    if a.uds:
        import os
        if os.path.exists(a.uds):
            os.unlink(a.uds)
        uvicorn.run(app, uds=a.uds, log_level="warning")
    else:
        uvicorn.run(app, host=a.host, port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
