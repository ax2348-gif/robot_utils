"""Built-in recording for the sim stack (docs/collect.md, "Sim teleop records itself"): Start records, Stop ends it.

In a sim stack (``--sim``) the node starts the collect script (``python -m robot_utils.collect run``, the same
Session and recorders an operator runs by hand) as a **child process** when teleop starts, and sends it SIGTERM
(its Ctrl-C: settle, a last reconcile, files closed, the open episode closed) when teleop stops, whatever stopped
it. A process, not a thread: recording parses every camera frame, row batch and headset record, and in this
process that work would share the GIL with the teleop loop, whose input gate disarms on 150 ms of lateness; a
crash, a full disk or a stuck write in the recorder can then never reach teleop either.

The child records the nodes of this machine over their unix sockets (``--robot local``: no frontend, and the
time base is this machine's monotonic clock, the one the stream records carry), with the default streams plus the
sim's rigid scene (``--scene3d <sim>``: its pose stream and each scene once per scene_id), indexes episodes from
the events after the seq just before ``teleop_start`` (``--baseline-seq``) and leaves the task alone
(``--no-set-task``: it is this node's). Its JSON status lines (``--status-json``) feed ``/state`` ``recording``.
Nothing here ever blocks or fails a start or a stop: a recorder that cannot start or dies is an event and
``recording.error``, and teleop runs on.
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

log = logging.getLogger("quest_teleop.recording")

DEFAULT_OUT = "~/data/sim"


@dataclass
class RecordConfig:
    out: str = DEFAULT_OUT            # sessions go to <out>/<YYYYMMDD-HHMMSS>
    robot: str = "local"              # the nodes over their unix sockets; or a frontend URL
    streams: Optional[str] = None     # comma-separated specs (default: collect's defaults)
    robot_profile: Optional[str] = None   # robots/: the default streams record each of its arms (arm_left, arm_right)
    batch_ms: float = 50.0
    settle: float = 1.0
    stop_timeout: float = 15.0        # after SIGTERM: then SIGKILL


class SimRecorder:
    """One child recorder per teleop run. ``start`` and ``stop`` return at once and never raise."""

    def __init__(self, node, sim: str, cfg: Optional[RecordConfig] = None,
                 popen: Callable[..., subprocess.Popen] = subprocess.Popen):
        self.node = node
        self.sim = sim
        self.cfg = cfg or RecordConfig()
        self.popen = popen
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.dir: Optional[Path] = None
        self.started_wall: Optional[float] = None
        self.stopped_wall: Optional[float] = None
        self.exit_code: Optional[int] = None
        self.error: Optional[str] = None
        self.status: Optional[dict] = None
        self.lines: deque = deque(maxlen=40)
        self.sessions = 0

    def out_dir(self) -> Path:
        return Path(os.path.expanduser(self.cfg.out))

    def _name(self) -> str:
        base = datetime.now().strftime("%Y%m%d-%H%M%S")
        name, k = base, 1
        while (self.out_dir() / name).exists():
            k += 1
            name = f"{base}-{k}"
        return name

    def command(self, name: str, baseline_seq: Optional[int], task: str) -> list[str]:
        c = self.cfg
        cmd = [sys.executable, "-m", "robot_utils.collect", "run", "--robot", c.robot, "--out", str(self.out_dir()),
               "--name", name, "--task", task or "", "--no-set-task", "--teleop", self.node.name,
               "--batch-ms", f"{c.batch_ms:g}", "--settle", f"{c.settle:g}", "--status-every", "1", "--status-json"]
        if c.streams:
            cmd += ["--streams", c.streams]
        elif c.robot_profile:
            cmd += ["--robot-profile", c.robot_profile]
        if self.sim and "://" not in self.sim:
            cmd += ["--scene3d", self.sim]
        if baseline_seq is not None:
            cmd += ["--baseline-seq", str(int(baseline_seq))]
        return cmd

    # -- start / stop ----------------------------------------------------------------------------------------

    def start(self, baseline_seq: Optional[int], task: str) -> None:
        try:
            with self.lock:
                if self.proc is not None and self.proc.poll() is None:
                    self._signal(self.proc)            # a previous one still settling: let it finish on its own
                self.out_dir().mkdir(parents=True, exist_ok=True)
                name = self._name()
                cmd = self.command(name, baseline_seq, task)
                env = dict(os.environ, PYTHONUNBUFFERED="1")
                src = str(Path(__file__).resolve().parents[3])      # this checkout's src/, first on the path
                env["PYTHONPATH"] = src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
                proc = self.popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL,
                                  env=env, text=True, bufsize=1)
                self.proc, self.dir = proc, self.out_dir() / name
                self.started_wall, self.stopped_wall, self.exit_code = time.time(), None, None
                self.error, self.status = None, None
                self.lines.clear()
                self.sessions += 1
            for stream, fn in ((proc.stdout, self._read_stdout), (proc.stderr, self._read_stderr)):
                threading.Thread(target=fn, args=(proc, stream), name="sim-recorder-io", daemon=True).start()
            threading.Thread(target=self._wait, args=(proc,), name="sim-recorder-wait", daemon=True).start()
            self.node.events.add(f"recording to {self.dir}")
        except Exception as e:  # noqa: BLE001 (never fails a teleop start)
            self.error = f"recording not started: {type(e).__name__}: {e}"
            log.exception("sim recorder start")
            self.node.events.add(self.error, "warn")
        self.node.hub.state_changed()

    def stop(self) -> None:
        """SIGTERM: the recorder settles, closes its files and exits on its own; SIGKILL after ``stop_timeout``."""
        with self.lock:
            proc = self.proc
        if proc is None or proc.poll() is not None:
            return
        self._signal(proc)

    def _signal(self, proc: subprocess.Popen) -> None:
        try:
            proc.send_signal(signal.SIGTERM)
        except OSError:
            return

        def reaper():
            try:
                proc.wait(self.cfg.stop_timeout + self.cfg.settle)
            except subprocess.TimeoutExpired:
                log.warning("sim recorder did not stop in time: killed")
                proc.kill()
        threading.Thread(target=reaper, name="sim-recorder-reaper", daemon=True).start()

    def close(self, timeout: Optional[float] = None) -> None:
        """Node shutdown: stop and wait for the recorder to finish its files."""
        with self.lock:
            proc = self.proc
        if proc is None:
            return
        self.stop()
        try:
            proc.wait(timeout if timeout is not None else self.cfg.stop_timeout + self.cfg.settle)
        except subprocess.TimeoutExpired:
            proc.kill()

    # -- the child's output ----------------------------------------------------------------------------------

    def _read_stdout(self, proc, stream) -> None:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            if line.startswith("{"):
                try:
                    st = json.loads(line).get("status")
                except ValueError:
                    st = None
                if isinstance(st, dict):
                    if proc is self.proc:
                        self.status = st
                        self.node.hub.state_changed()
                    continue
            self._line(proc, line, "info")

    def _read_stderr(self, proc, stream) -> None:
        for line in stream:
            line = line.rstrip()
            if line:
                self._line(proc, line, "warn" if (" WARNING " in line or " ERROR " in line) else "debug")

    def _line(self, proc, line: str, level: str) -> None:
        self.lines.append(line)
        log.log(logging.INFO if level == "info" else logging.WARNING if level == "warn" else logging.DEBUG,
                "recorder: %s", line)
        if line.startswith("collect: episode"):
            self.node.events.add("recorded " + line[len("collect: "):])

    def _wait(self, proc: subprocess.Popen) -> None:
        rc = proc.wait()
        if proc is not self.proc:
            return
        self.exit_code, self.stopped_wall = rc, time.time()
        if rc != 0:
            last = next((x for x in reversed(self.lines) if x.startswith("collect: ")), None) or \
                (self.lines[-1] if self.lines else "")
            self.error = f"the recorder exited with {rc}" + (f": {last}" if last else "")
            self.node.events.add(self.error, "warn")
        else:
            self.node.events.add(f"recording finished: {self.dir}")
        self.node.hub.state_changed()

    # -- state -----------------------------------------------------------------------------------------------

    def info(self) -> dict:
        proc = self.proc
        active = proc is not None and proc.poll() is None
        st = self.status or {}
        return {"enabled": True, "out": str(self.out_dir()), "active": active, "dir": str(self.dir) if self.dir else None,
                "pid": proc.pid if proc is not None else None, "started_wall": self.started_wall,
                "stopped_wall": self.stopped_wall, "exit_code": self.exit_code, "error": self.error,
                "sessions": self.sessions, "bytes": st.get("bytes"), "episodes": st.get("episodes"),
                "streams": st.get("streams"), "scenes": st.get("scenes"), "status": st.get("status")}
