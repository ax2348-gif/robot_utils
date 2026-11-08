"""Serving a node: instance name, unix socket, opt-in TCP port. Shared by every node and the frontend.

Every node instance listens on a unix socket ``$XDG_RUNTIME_DIR/robot/<name>.sock`` (on the
robot ``/run/user/<uid>/robot/<name>.sock``; lingering keeps that directory). The only
network-facing process is nginx on port 80 (``robot-proxy.service``, written by install.sh),
which maps ``http://<robot>/nodes/<name>/<path>`` to ``<path>`` on that socket and the landing
page (``frontend.sock``) to ``/``. So one origin serves everything and a new instance needs no
proxy change. ``--port`` additionally listens on TCP, for debugging only.

    p = argparse.ArgumentParser(...)
    add_serve_args(p, "swerve")              # --name (default "swerve"), --uds, --port, --host
    args = p.parse_args()
    serve(create_app(...), args)             # blocks; Ctrl-C / SIGTERM stops it

Behind nginx a node's pages and API docs live under ``/nodes/<name>/``: nginx sends
``X-Forwarded-Prefix: /nodes/<name>`` and the node uses it as the request's ``root_path`` (so
``/docs`` finds its ``openapi.json``). Direct TCP access (``--port``) has no prefix. Node pages
use relative URLs only, so they work either way.

Inter-node clients name their peer instead of a URL: ``connect_target("swerve")`` resolves to
that instance's socket; anything containing ``://`` is used as a URL (remote / debugging).
"""
from __future__ import annotations

import argparse
import os
import re
import socket
import stat
import sys
from pathlib import Path
from typing import Optional

NAME = re.compile(r"^[A-Za-z0-9_.-]+$")
PREFIX_HEADER = b"x-forwarded-prefix"


def socket_dir() -> Path:
    """``$XDG_RUNTIME_DIR/robot`` (``/run/user/<uid>/robot``); a private temp dir as a fallback."""
    run = os.environ.get("XDG_RUNTIME_DIR") or (f"/run/user/{os.getuid()}" if os.path.isdir(f"/run/user/{os.getuid()}") else None)
    if run is None:   # macOS and friends: no runtime dir
        import tempfile
        run = os.path.join(tempfile.gettempdir(), f"robot-{os.getuid()}")
    return Path(run) / "robot"


def socket_path(name: str) -> Path:
    check_name(name)
    return socket_dir() / f"{name}.sock"


def check_name(name: str) -> str:
    if not isinstance(name, str) or not NAME.match(name):
        raise ValueError(f"instance name must match [A-Za-z0-9_.-]+, got {name!r}")
    return name


def add_serve_args(parser: argparse.ArgumentParser, kind: str, *, default_port: Optional[int] = None) -> None:
    """``--name`` (instance, default ``kind``), ``--uds`` (socket path), ``--port``/``--host``
    (opt-in TCP for debugging; ``default_port`` only if a node must keep a TCP port by default)."""
    g = parser.add_argument_group("serving")
    g.add_argument("--name", default=kind,
                   help=f"instance name (default {kind!r}): socket <runtime dir>/robot/<name>.sock, "
                        "URL http://<robot>/nodes/<name>/")
    g.add_argument("--uds", default=None, help="unix socket path (default: from --name)")
    g.add_argument("--port", type=int, default=default_port,
                   help="also listen on this TCP port (debugging; not needed behind the proxy)")
    g.add_argument("--host", default="0.0.0.0", help="address for --port (default: all interfaces)")
    parser.set_defaults(_serve_kind=kind)


def _sock_alive(path: Path) -> bool:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        s.connect(str(path))
        return True
    except OSError:
        return False
    finally:
        s.close()


def bind_uds(path: Path) -> socket.socket:
    """Bind a listening unix socket; remove a stale socket file, refuse if one is live."""
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if path.exists() or path.is_symlink():
        if not stat.S_ISSOCK(path.lstat().st_mode):
            raise RuntimeError(f"{path} exists and is not a socket; not touching it")
        if _sock_alive(path):
            raise RuntimeError(f"already running: something is listening on {path}")
        path.unlink()   # stale: left behind by a node that died
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(str(path))
    os.chmod(path, 0o660)
    sock.listen(2048)
    sock.set_inheritable(True)
    return sock


def bind_tcp(host: str, port: int) -> socket.socket:
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    sock = socket.socket(family, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(2048)
    sock.set_inheritable(True)
    return sock


class PrefixMiddleware:
    """Use nginx's ``X-Forwarded-Prefix`` (``/nodes/<name>``) as the request's ``root_path``."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] in ("http", "websocket"):
            for k, v in scope.get("headers", ()):
                if k == PREFIX_HEADER:
                    prefix = v.decode("latin-1").rstrip("/")
                    if prefix.startswith("/nodes/") and NAME.match(prefix[len("/nodes/"):]):
                        scope = dict(scope, root_path=prefix)
                    break
        await self.app(scope, receive, send)


def serve(app, args: argparse.Namespace, *, banner: Optional[str] = None, public_path: Optional[str] = None,
          **uvicorn_kwargs) -> None:
    """Run ``app`` on the instance's unix socket (+ ``--port`` if given) until SIGINT/SIGTERM.

    Sets the instance name in the app's ``/meta`` (``add_page_routes`` keeps it in
    ``app.state.node_meta``). ``public_path``: where the proxy serves it, for the startup line
    (default ``/nodes/<name>/``). ``uvicorn_kwargs`` go to ``uvicorn.Config`` (log level, timeouts).
    """
    import uvicorn

    name = check_name(args.name)
    meta = getattr(app.state, "node_meta", None)
    if isinstance(meta, dict):
        meta["name"] = name
    path = Path(args.uds) if args.uds else socket_path(name)
    try:
        socks = [bind_uds(path)]
    except (RuntimeError, OSError) as e:
        sys.exit(f"error: {e}")
    if args.port:
        try:
            socks.append(bind_tcp(args.host, args.port))
        except OSError as e:
            socks[0].close()
            path.unlink(missing_ok=True)
            hint = ""
            if isinstance(e, PermissionError) and args.port < 1024:
                hint = ("\n  ports below 1024 need a one-time setting: sudo tools/enable_low_ports.sh"
                        "\n  (or use a high port, e.g. --port 8080)")
            sys.exit(f"error: cannot listen on {args.host}:{args.port}: {e}{hint}")
    print(banner or f"{getattr(args, '_serve_kind', 'node')} node {name!r}", flush=True)
    print(f"  socket {path}  ->  http://<robot>{public_path or f'/nodes/{name}/'}", flush=True)
    if args.port:
        print(f"  debug  http://{args.host}:{args.port}/", flush=True)
    uvicorn_kwargs.setdefault("log_level", "warning")
    config = uvicorn.Config(PrefixMiddleware(app), workers=1, lifespan="on", **uvicorn_kwargs)

    def remove_socket():
        for s in socks:
            s.close()
        try:
            if path.is_socket() and not _sock_alive(path):
                path.unlink()
        except OSError:
            pass

    class Server(uvicorn.Server):
        # uvicorn re-raises SIGINT/SIGTERM after shutting down, so a ``finally`` would not run;
        # remove the socket as part of the shutdown instead (a stale one is replaced at start).
        async def shutdown(self, sockets=None):
            await super().shutdown(sockets)
            remove_socket()

    try:
        Server(config).run(sockets=socks)
    finally:
        remove_socket()


def connect_target(spec: str) -> tuple[str, Optional[str]]:
    """(base_url, unix socket path or None) for an instance name or a URL.

    ``"swerve"`` -> ``("http://swerve", "<runtime dir>/robot/swerve.sock")``;
    ``"http://botcore/nodes/swerve"`` -> that URL, no socket."""
    if "://" in spec:
        return spec.rstrip("/"), None
    return f"http://{check_name(spec)}", str(socket_path(spec))
