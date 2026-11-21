"""Helpers shared by the test modules (test modules never import from each other; they import from here).

- ``wait_for`` polls a predicate; ``waiter`` makes one with a module's own default timeout / interval.
- ``serve`` runs an ASGI app with uvicorn on a thread (a free local port); ``stop`` ends servers it started.
- ``same`` compares JSON losslessly: same key order (unless ``order=False``) and the same types (1 != 1.0).
- ``Clock`` is a hand-advanced clock: ``clock()`` returns ``clock.t``.
"""
import socket
import threading
import time

import uvicorn


def wait_for(predicate, timeout=5.0, interval=0.02):
    """Poll until ``predicate()`` is truthy; return its value, or fail after ``timeout`` s."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
    raise AssertionError("condition not met in time")


def waiter(timeout, interval):
    """``wait_for`` with other defaults (a module's own), same signature."""
    def wait(predicate, timeout=timeout, interval=interval):
        return wait_for(predicate, timeout, interval)
    return wait


def serve(app):
    """Serve ``app`` on 127.0.0.1 (a free port) from a daemon thread; returns (base url, server, thread)
    once it is accepting."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning",
                                           timeout_graceful_shutdown=1))
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    wait_for(lambda: server.started, 10)
    return f"http://127.0.0.1:{port}", server, t


def stop(*servers):
    """End servers from ``serve``, given as (server, thread) pairs."""
    for srv, t in servers:
        srv.should_exit = True
        t.join(10)


def same(a, b, path="$", order=True):
    """Equal as JSON, including key order (with ``order``) and int / float / bool types (1 and 1.0 differ)."""
    assert type(a) is type(b), f"{path}: {a!r} ({type(a).__name__}) != {b!r} ({type(b).__name__})"
    if isinstance(a, dict):
        assert (list(a) if order else set(a)) == (list(b) if order else set(b)), f"{path}: keys {list(a)} != {list(b)}"
        for k in a:
            same(a[k], b[k], f"{path}.{k}", order)
    elif isinstance(a, list):
        assert len(a) == len(b), f"{path}: length {len(a)} != {len(b)}"
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, f"{path}[{i}]", order)
    else:
        assert a == b, f"{path}: {a!r} != {b!r}"


class Clock:
    """A clock the test advances by hand (``clock.t += dt``)."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t
