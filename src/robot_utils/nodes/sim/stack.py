"""python -m robot_utils.nodes.sim.stack [--port 8080] [--host 127.0.0.1] [--nodes a,b,...] [--runtime-dir DIR]

The whole robot stack on the MuJoCo sim world, on this machine (docs/sim.md, "The sim stack"): the landing page
(``robot_utils.frontend``) with the sim registry (``stack_nodes.yaml``: every node in its sim mode), which then
starts the nodes itself, in order: the description node, the sim node, then arm, swerve, cameras, power,
estimator, pose_servo (``--nodes`` changes the list). So the landing page works as on the robot: tiles, pages,
Start / Stop. Open http://localhost:8080/ (the frontend's own /nodes/ proxy; no nginx needed).

Ctrl-C (or SIGTERM) stops every node (in reverse order), then the landing page. Node logs: the landing page's
"logs" per tile, or ``GET /api/nodes/<name>/logs``. Never on the robot: it would start nodes on hardware names
in sim mode next to the real ones.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from importlib import resources
from typing import Optional

DEFAULT_NODES = "description,sim,arm,swerve,camera,wrist_camera,power,estimator,pose_servo"
START_TIMEOUT = {"sim": 90.0, "description": 30.0}   # s until a node's socket answers (the sim loads meshes)


def registry_path() -> str:
    return str(resources.files(__package__).joinpath("stack_nodes.yaml"))


class Frontend:
    """The landing page's API on its TCP port."""

    def __init__(self, port: int):
        self.base = f"http://127.0.0.1:{port}"

    def call(self, method: str, path: str, timeout: float = 30.0):
        req = urllib.request.Request(self.base + path, method=method, data=b"" if method == "POST" else None)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.status, json.loads(r.read() or b"null")
        except urllib.error.HTTPError as e:
            try:
                detail = json.loads(e.read()).get("detail")
            except Exception:  # noqa: BLE001
                detail = str(e)
            return e.code, {"detail": detail}

    def up(self) -> bool:
        try:
            return self.call("GET", "/api/health", timeout=1.0)[0] == 200
        except OSError:
            return False

    def managed(self, name: str) -> Optional[dict]:
        st, body = self.call("GET", "/api/managed", timeout=10.0)
        if st != 200:
            return None
        return next((n for n in body if n.get("name") == name), None) if isinstance(body, list) else \
            next((n for n in body.get("nodes", []) if n.get("name") == name), None)


def log(msg: str) -> None:
    print(f"[sim stack] {msg}", flush=True)


def start_node(fe: Frontend, name: str) -> bool:
    st, body = fe.call("POST", f"/api/nodes/{name}/start", timeout=40.0)
    if st != 200:
        log(f"{name}: start refused: {body.get('detail') if isinstance(body, dict) else body}")
        return False
    end = time.monotonic() + START_TIMEOUT.get(name, 30.0)
    while time.monotonic() < end:
        m = fe.managed(name) or {}
        if m.get("listening"):
            log(f"{name}: up")
            return True
        state = (m.get("managed") or {}).get("state")
        if state in ("failed", "stopped"):
            break
        time.sleep(0.3)
    st, lines = fe.call("GET", f"/api/nodes/{name}/logs?lines=15")
    tail = "\n    ".join(lines.get("lines", []) if isinstance(lines, dict) else lines or [])
    log(f"{name}: did not come up; last log lines:\n    {tail}")
    return False


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m robot_utils.nodes.sim.stack", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8080, help="the landing page's TCP port (default 8080)")
    p.add_argument("--host", default="127.0.0.1",
                   help="address for --port (default 127.0.0.1: this machine only; 0.0.0.0 for the LAN / a headset)")
    p.add_argument("--nodes", default=DEFAULT_NODES, help=f"instances to start, in order (default {DEFAULT_NODES}); "
                                                          "the others in the registry get a Start button")
    p.add_argument("--config", default=None, help="node registry (default: the packaged stack_nodes.yaml)")
    p.add_argument("--runtime-dir", default=None,
                   help="run the stack under this XDG_RUNTIME_DIR (sockets in <dir>/robot/), e.g. to run two stacks; "
                        "keep it short: unix socket paths are limited to ~100 characters")
    a = p.parse_args(argv)

    env = dict(os.environ, PYTHONUNBUFFERED="1")
    if a.runtime_dir:
        env["XDG_RUNTIME_DIR"] = os.path.abspath(os.path.expanduser(a.runtime_dir))
        os.environ["XDG_RUNTIME_DIR"] = env["XDG_RUNTIME_DIR"]
    from ..common.serve import socket_dir
    sock_dir = socket_dir()
    if len(str(sock_dir / "sim" / "sim" / "camera-wrist_camera.sock")) > 103:
        p.error(f"socket directory {sock_dir} is too long for unix sockets; use a shorter --runtime-dir")

    fe = Frontend(a.port)
    if fe.up():
        p.error(f"something already answers on port {a.port} (another stack?); stop it or use --port")
    cfg = a.config or registry_path()
    cmd = [sys.executable, "-m", "robot_utils.frontend", "--config", cfg, "--port", str(a.port), "--host", a.host,
           "--no-observe"]
    log(f"landing page: {' '.join(cmd[1:])}")
    front = subprocess.Popen(cmd, env=env, start_new_session=True)

    stopping = {"flag": False}

    def on_signal(signum, frame):
        stopping["flag"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    started: list[str] = []
    try:
        end = time.monotonic() + 20
        while not fe.up():
            if front.poll() is not None or time.monotonic() > end:
                log("the landing page did not start")
                return 1
            time.sleep(0.2)
        names = [n.strip() for n in a.nodes.split(",") if n.strip()]
        for name in names:
            if stopping["flag"]:
                break
            if start_node(fe, name):
                started.append(name)
            elif name in ("description", "sim"):
                log("the world is needed by the other nodes: stopping")
                stopping["flag"] = True
        if not stopping["flag"]:
            shown = "localhost" if a.host in ("127.0.0.1", "0.0.0.0", "::") else a.host
            url = (os.environ.get("ROBOT_STACK_URL") or f"http://{shown}:{a.port}/").rstrip("/")   # behind a proxy
            log(f"up: {', '.join(started)}")
            log(f"landing page  {url}/")
            log(f"sim world     {url}/nodes/sim/")
            log(f"sockets       {sock_dir}")
            log("Ctrl-C stops everything")
        while not stopping["flag"] and front.poll() is None:
            time.sleep(0.3)
        if front.poll() is not None and not stopping["flag"]:
            log(f"the landing page exited ({front.returncode}); its nodes went with it")
    finally:
        if front.poll() is None:
            # stop what the landing page runs (also what was started from its tiles), newest first
            st, body = fe.call("GET", "/api/managed", timeout=10.0) if fe.up() else (0, None)
            running = [n["name"] for n in (body if isinstance(body, list) else (body or {}).get("nodes", []))
                       if (n.get("managed") or {}).get("state") in ("running", "starting")]
            order = [n for n in reversed(started) if n in running] + [n for n in running if n not in started]
            for name in order:
                log(f"stopping {name}")
                fe.call("POST", f"/api/nodes/{name}/stop", timeout=30.0)
            front.terminate()
            try:
                front.wait(15)
            except subprocess.TimeoutExpired:
                front.kill()
        log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
