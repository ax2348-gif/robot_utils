"""Start and stop nodes from the landing page, as systemd user units.

Scope, on purpose: the frontend controls exactly one systemd user unit per node,
``robot-node-<name>.service``, and treats it as its own (whoever started it). It does not look
for node processes started any other way (e.g. in a terminal); such a node only shows up as
"something already listens on its port", which blocks Start. So:

- Start: refused if the unit is already active, or if anything listens on the node's port.
  Otherwise ``systemd-run --user --unit robot-node-<name> ...`` runs
  ``<this interpreter> -m <module> <args>`` (``start:`` in nodes.yaml).
- Stop: ``systemctl --user stop robot-node-<name>`` (SIGTERM, so the node shuts down cleanly;
  SIGKILL after TimeoutStopSec). Refused if that unit is not running.

Why systemd units and not child processes: each node gets its own cgroup and lifetime, so it
keeps running when the frontend service restarts (``install.sh`` restarts it) or crashes;
which nodes are "ours" is recovered from systemd, not from memory; logs go to the journal
(``journalctl --user -u robot-node-<name>``). No restart policy: a node that exits stays down
(a failed unit keeps its result until the next Start, which clears it).

``SubprocessRunner`` is a fallback for machines without a systemd user session (development
on macOS): plain child processes owned by this frontend process.
"""

from __future__ import annotations

import asyncio
import os
import shutil
import socket
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Protocol

from .config import NodeEntry

UNIT_PREFIX = "robot-node-"
STOP_TIMEOUT_S = 10


class ManagerError(Exception):
    def __init__(self, status: int, detail: str):
        super().__init__(detail)
        self.status = status
        self.detail = detail


@dataclass
class UnitStatus:
    state: str                 # "running" | "starting" | "stopping" | "stopped" | "failed"
    pid: Optional[int] = None
    since: Optional[float] = None   # unix time the unit entered its state (systemd) / started
    result: Optional[str] = None    # systemd Result (e.g. "exit-code", "signal") when failed

    def as_dict(self) -> dict:
        return {"state": self.state, "pid": self.pid, "since": self.since, "result": self.result}


def unit_name(node: NodeEntry) -> str:
    return f"{UNIT_PREFIX}{node.name}.service"


def port_in_use(port: int, host: str = "127.0.0.1", timeout: float = 0.3) -> bool:
    """True if something accepts TCP connections on host:port (the generic "already up" test)."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


class Runner(Protocol):
    kind: str
    async def status(self, node: NodeEntry) -> UnitStatus: ...
    async def start(self, node: NodeEntry, argv: list[str], cwd: str) -> UnitStatus: ...
    async def stop(self, node: NodeEntry) -> None: ...
    async def logs(self, node: NodeEntry, lines: int) -> list[str]: ...


async def _run(*argv: str, timeout: float = 15.0) -> tuple[int, str, str]:
    proc = await asyncio.create_subprocess_exec(*argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        out, err = await asyncio.wait_for(proc.communicate(), timeout)
    except asyncio.TimeoutError:
        proc.kill()
        await proc.wait()
        return 124, "", f"{argv[0]} timed out after {timeout:.0f} s"
    return proc.returncode, out.decode(errors="replace"), err.decode(errors="replace")


class SystemdRunner:
    """Transient systemd user units (``systemd-run --user``)."""

    kind = "systemd"

    def __init__(self, run=_run):
        self._run = run   # injectable for tests

    @staticmethod
    def available() -> bool:
        if not (shutil.which("systemd-run") and shutil.which("systemctl")):
            return False
        try:
            return subprocess.run(["systemctl", "--user", "show-environment"], capture_output=True,
                                  timeout=5).returncode == 0
        except (OSError, subprocess.SubprocessError):
            return False

    async def status(self, node: NodeEntry) -> UnitStatus:
        code, out, _ = await self._run(
            "systemctl", "--user", "show", unit_name(node), "--no-pager",
            "-p", "LoadState", "-p", "ActiveState", "-p", "MainPID", "-p", "Result",
            "-p", "StateChangeTimestampMonotonic", timeout=5)
        props = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
        load, active = props.get("LoadState", "not-found"), props.get("ActiveState", "inactive")
        if code != 0 or load == "not-found":
            return UnitStatus("stopped")
        pid = int(props.get("MainPID", "0") or 0) or None
        since = None
        mono = int(props.get("StateChangeTimestampMonotonic", "0") or 0)
        if mono:
            since = time.time() - (time.monotonic() - mono / 1e6)
        state = {"active": "running", "activating": "starting", "reloading": "running",
                 "deactivating": "stopping", "failed": "failed"}.get(active, "stopped")
        result = props.get("Result") if state == "failed" else None
        return UnitStatus(state, pid, since, result)

    async def start(self, node: NodeEntry, argv: list[str], cwd: str) -> UnitStatus:
        unit = unit_name(node)
        await self._run("systemctl", "--user", "reset-failed", unit, timeout=5)  # clear a failed run
        code, out, err = await self._run(
            "systemd-run", "--user", f"--unit={unit}",
            f"--description=robot_utils node {node.name} (started by the landing page)",
            f"--working-directory={cwd}", "--setenv=PYTHONUNBUFFERED=1",
            f"--property=TimeoutStopSec={STOP_TIMEOUT_S}", "--property=KillMode=mixed",
            "--", *argv)
        if code != 0:
            raise ManagerError(500, f"systemd-run failed: {(err or out).strip()}")
        return await self.status(node)

    async def stop(self, node: NodeEntry) -> None:
        code, out, err = await self._run("systemctl", "--user", "stop", unit_name(node),
                                         timeout=STOP_TIMEOUT_S + 10)
        if code != 0:
            raise ManagerError(500, f"systemctl stop failed: {(err or out).strip()}")

    async def logs(self, node: NodeEntry, lines: int) -> list[str]:
        _, out, _ = await self._run("journalctl", "--user", "-u", unit_name(node), "-n", str(lines),
                                    "--no-pager", "-o", "cat", timeout=5)
        return out.splitlines()[-lines:]


class SubprocessRunner:
    """Child processes of this frontend (no systemd user session, e.g. development on macOS)."""

    kind = "subprocess"

    def __init__(self):
        self._procs: dict[str, asyncio.subprocess.Process] = {}
        self._nodes: dict[str, NodeEntry] = {}
        self._started: dict[str, float] = {}
        self._logs: dict[str, deque] = {}
        self._failed: dict[str, int] = {}

    async def status(self, node: NodeEntry) -> UnitStatus:
        proc = self._procs.get(node.name)
        if proc is None:
            return UnitStatus("failed", result=f"exit-code {self._failed[node.name]}") \
                if node.name in self._failed else UnitStatus("stopped")
        if proc.returncode is None:
            return UnitStatus("running", proc.pid, self._started[node.name])
        del self._procs[node.name]
        if proc.returncode not in (0, -15):
            self._failed[node.name] = proc.returncode
            return UnitStatus("failed", result=f"exit-code {proc.returncode}")
        return UnitStatus("stopped")

    async def start(self, node: NodeEntry, argv: list[str], cwd: str) -> UnitStatus:
        self._failed.pop(node.name, None)
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        proc = await asyncio.create_subprocess_exec(*argv, cwd=cwd, env=env, stdout=subprocess.PIPE,
                                                    stderr=subprocess.STDOUT, start_new_session=True)
        self._procs[node.name], self._started[node.name] = proc, time.time()
        self._nodes[node.name] = node
        buf = self._logs[node.name] = deque(maxlen=200)

        async def pump():
            async for line in proc.stdout:
                buf.append(line.decode(errors="replace").rstrip())
        asyncio.get_running_loop().create_task(pump())
        return await self.status(node)

    async def stop(self, node: NodeEntry) -> None:
        proc = self._procs.get(node.name)
        if proc is None or proc.returncode is not None:
            return
        proc.terminate()
        try:
            await asyncio.wait_for(proc.wait(), STOP_TIMEOUT_S)
        except asyncio.TimeoutError:
            proc.kill()
            await proc.wait()
        self._procs.pop(node.name, None)

    async def logs(self, node: NodeEntry, lines: int) -> list[str]:
        return list(self._logs.get(node.name, ()))[-lines:]

    async def close(self) -> None:
        """Child processes cannot outlive us cleanly: stop them with the frontend."""
        for name in list(self._procs):
            await self.stop(self._nodes[name])


class NodeManager:
    """The policy (what may be started/stopped) on top of a runner."""

    def __init__(self, nodes: list[NodeEntry], runner: Runner, *, cwd: str | None = None,
                 python: str = sys.executable, port_check=port_in_use):
        self.nodes = {n.name: n for n in nodes if n.enabled}
        self.runner = runner
        self.cwd = cwd or os.getcwd()
        self.python = python
        self._port_in_use = port_check
        self._locks = {name: asyncio.Lock() for name in self.nodes}

    def _node(self, name: str) -> NodeEntry:
        node = self.nodes.get(name)
        if node is None:
            raise ManagerError(404, f"no enabled node {name!r} in nodes.yaml")
        return node

    async def status(self, name: str) -> dict:
        node = self._node(name)
        st = await self.runner.status(node) if node.start else UnitStatus("stopped")
        listening = await asyncio.to_thread(self._port_in_use, node.port)
        return {"name": name, "startable": node.start is not None, "unit": unit_name(node),
                "runner": self.runner.kind, "managed": st.as_dict(), "port_in_use": listening}

    async def status_all(self) -> list[dict]:
        return list(await asyncio.gather(*(self.status(n) for n in self.nodes)))

    async def start(self, name: str) -> dict:
        node = self._node(name)
        if node.start is None:
            raise ManagerError(400, f"node {name!r} has no start recipe in nodes.yaml")
        async with self._locks[name]:
            st = await self.runner.status(node)
            if st.state in ("running", "starting", "stopping"):
                raise ManagerError(409, f"{unit_name(node)} is already {st.state}")
            if await asyncio.to_thread(self._port_in_use, node.port):
                raise ManagerError(409, f"port {node.port} is already in use: node {name!r} is "
                                        "running outside the landing page (e.g. started in a "
                                        "terminal); stop it there first")
            await self.runner.start(node, node.start.argv(self.python), self.cwd)
        return await self.status(name)

    async def stop(self, name: str) -> dict:
        node = self._node(name)
        async with self._locks[name]:
            st = await self.runner.status(node)
            if st.state not in ("running", "starting"):
                raise ManagerError(409, f"{unit_name(node)} is not running; the landing page only "
                                        "stops nodes it started")
            await self.runner.stop(node)
        return await self.status(name)

    async def logs(self, name: str, lines: int = 40) -> list[str]:
        node = self._node(name)
        return await self.runner.logs(node, max(1, min(lines, 500)))


def default_runner() -> Runner:
    return SystemdRunner() if SystemdRunner.available() else SubprocessRunner()
