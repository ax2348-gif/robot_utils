"""Clients of the sim world for the nodes' hardware backends (docs/sim.md, "Backends"): ``arm.WorldArmBus``
(an ArmBus), ``swerve.start_world_sim`` (world-coupled SimulatedSparks on a virtual CAN bus) and
``camera.WorldCameraBackend`` (the orbbec backend interface). Each talks to the world over its own socket.

The nodes select them with ``--sim world[=NAME|DIR]`` (``world_socket`` resolves it): ``world`` = the sim
node instance ``sim``, ``world=NAME`` another instance, ``world=DIR`` (contains a ``/``) the directory holding
the backend sockets (``<runtime dir>/robot/sim/<name>/`` by default)."""
from __future__ import annotations

import socket
import time
from pathlib import Path
from typing import Optional


def is_world(spec: Optional[str]) -> bool:
    """``--sim`` values that mean the sim world (``world``, ``world=...``)."""
    return isinstance(spec, str) and (spec == "world" or spec.startswith("world="))


def world_socket(spec: str, backend: str) -> Path:
    """The backend socket for ``--sim world[=NAME|DIR]``: ``backend`` is an arm's name (``arm``; robots/), ``swerve``
    or ``camera-<name>``."""
    from ..protocol import backend_socket
    if not is_world(spec):
        raise ValueError(f"--sim {spec!r}: expected world, world=NAME (sim instance) or world=DIR (socket directory)")
    target = spec[len("world="):] if "=" in spec else "sim"
    if "/" in target:
        return Path(target).expanduser() / f"{backend}.sock"
    return backend_socket(target or "sim", backend)


def wait_for_world(path: Path, timeout: float = 60.0) -> bool:
    """Wait until the world answers on ``path`` (the sim node started after this node); True if it does."""
    end = time.monotonic() + timeout
    while True:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(str(path))
            return True
        except OSError:
            if time.monotonic() > end:
                return False
            time.sleep(0.25)
        finally:
            s.close()
