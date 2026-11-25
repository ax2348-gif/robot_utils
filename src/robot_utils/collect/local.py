"""``--robot local``: record the nodes of this machine over their unix sockets, without the frontend.

Every collect path is ``<robot>/nodes/<instance>/...``; ``NodeSocketTransport`` sends such a request to the
instance's own socket (``nodes.common.serve.socket_path``, the mechanism nodes use to reach each other) as
``/...``, and anything else (an instance without a socket here) through plain HTTP as addressed. The quest_teleop
node's sim recorder uses it: it needs no frontend port, and the time base is this machine's clock.
"""
from __future__ import annotations

from typing import Optional

import httpx

from robot_utils.nodes.common.serve import socket_path

LOCAL = "local"
LOCAL_BASE = "http://localhost"       # what ``--robot local`` stands for in URLs (and session.json)


class NodeSocketTransport(httpx.AsyncBaseTransport):
    def __init__(self, fallback: Optional[httpx.AsyncBaseTransport] = None):
        self._sockets: dict[str, httpx.AsyncHTTPTransport] = {}
        self._fallback = fallback or httpx.AsyncHTTPTransport()

    def _route(self, request: httpx.Request) -> Optional[httpx.AsyncHTTPTransport]:
        parts = request.url.path.split("/", 3)              # "", "nodes", <instance>, <rest>
        if len(parts) < 3 or parts[1] != "nodes" or not parts[2]:
            return None
        name = parts[2]
        try:
            sock = socket_path(name)
        except ValueError:
            return None
        if not sock.exists():
            return None
        t = self._sockets.get(name)
        if t is None:
            t = self._sockets[name] = httpx.AsyncHTTPTransport(uds=str(sock))
        request.url = request.url.copy_with(host=name, port=None, path="/" + (parts[3] if len(parts) > 3 else ""))
        request.headers["host"] = name
        return t

    async def handle_async_request(self, request: httpx.Request) -> httpx.Response:
        t = self._route(request)
        return await (t or self._fallback).handle_async_request(request)

    async def aclose(self) -> None:
        for t in self._sockets.values():
            await t.aclose()
        await self._fallback.aclose()
