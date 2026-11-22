"""The Quest teleop node's core: the teleop client loop (tools/quest_teleop, through ``client.py``) against the local
WBC, the headset link monitor, the WBC echo, and the headset control (quest_ctl up / down over this machine's ssh).
No web code here (server.py); nothing is written to disk (only log lines, to the journal).

Threads:
- ``quest-teleop`` (the loop): while teleop runs, ``TeleopClient.step()`` (headset frames in, one datagram per batch
  out); otherwise it drains the headset frames for the link monitor. Publishes a row every ``ROW_PERIOD``.
- ``wbc-poll``: ``GET /state`` of the WBC once a second (e-stop, lease, enabled, fault): stops teleop when the WBC is
  disabled, faults, e-stops or the lease is no longer ours.
- the reader threads of ``QuestReader`` and ``WbcStream`` (the client's own), and one thread per headset command.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import Callable, Optional

import numpy as np

from ..common.client import NodeClient
from ..common.hub import RowHub
from ..common.lease import RequestError
from ..common.server import EventLog
from .client import (DEFAULT_PORT, STALE_S, Config, LocalWbcClient, QuestReader, TeleopClient, WbcStream, close_wbc,
                     open_wbc, quest_ctl)

log = logging.getLogger("quest_teleop")

ROW_PERIOD = 0.05            # 20 Hz rows
CLIENT_NAME = "quest_teleop node"   # the WBC lease holder (distinct from the CLI's "quest_teleop")
CLIENT_STATES = ["stopped", "starting", "waiting", "released", "engaged", "needs_release", "stale", "refused",
                 "stopping"]
XR_STATES = ["UNKNOWN", "IDLE", "READY", "SYNCHRONIZED", "VISIBLE", "FOCUSED", "STOPPING", "LOSS_PENDING", "EXITING"]
WBC_STATES = ["disabled", "enabling", "idle", "tracking", "holding", "fault"]
WBC_MODES = ["free", "locked", "hint", "commanded"]
WBC_LIMITED = ["none", "reach", "manipulability", "fence", "joint_limit", "base_limit"]
nan = float("nan")

_COLS = [
    ("t", "s", "row time (this host's monotonic clock)"),
    ("client_state", "enum", "0 stopped, 1 starting, 2 waiting, 3 released, 4 engaged, 5 needs_release, 6 stale, "
                             "7 refused (the last engagement was refused; needs a release), 8 stopping"),
    ("sending", "bool", "1 if the last step sent a datagram to the WBC"),
    ("quest_connected", "bool", "1 while the TCP stream from the headset is open"),
    ("quest_rate", "Hz", "headset frames received over the last second"),
    ("quest_age", "s", "since the newest headset frame arrived (our clock)"),
    ("quest_focused", "bool", "input_focused of the newest frame"),
    ("quest_xr", "enum", "XR session state: " + ", ".join(f"{i} {s}" for i, s in enumerate(XR_STATES))),
    ("quest_discontinuities", "count", "session / revision / reconnect / gap discontinuities seen by the client"),
    ("left_valid", "bool", "the left hand's intent is valid"),
    ("left_engaged", "bool", "the left grip clutch is engaged (headset intent)"),
    ("left_gripper", "", "the left trigger, 0 open .. 1 closed"),
    ("right_valid", "bool", "the right hand's intent is valid"),
    ("right_engaged", "bool", "the right grip clutch is engaged (headset intent)"),
    ("right_gripper", "", "the right trigger, 0 open .. 1 closed"),
    ("engagements", "count", "engagements accepted since teleop started"),
    ("sent", "count", "target datagrams sent since teleop started"),
    ("wbc_state", "enum", "the WBC's state (its stream): " + ", ".join(f"{i} {s}" for i, s in enumerate(WBC_STATES))),
    ("wbc_base_mode", "enum", "the WBC's base mode: " + ", ".join(f"{i} {s}" for i, s in enumerate(WBC_MODES))),
    ("wbc_limited", "bool", "1 if a hard limit bound the WBC's motion"),
    ("wbc_limited_by", "enum", ", ".join(f"{i} {s}" for i, s in enumerate(WBC_LIMITED))),
    ("wbc_age", "s", "since the newest WBC stream row arrived (our clock)"),
    ("base_x", "m", "the WBC's base pose (world)"), ("base_y", "m", ""), ("base_yaw", "rad", ""),
    ("ee_cmd_x", "m", "the WBC's commanded EE (world)"), ("ee_cmd_y", "m", ""), ("ee_cmd_z", "m", ""),
    ("ee_tgt_x", "m", "the WBC's EE target in force (world)"), ("ee_tgt_y", "m", ""), ("ee_tgt_z", "m", ""),
    ("sent_x", "m", "the EE target this node sent last (world)"), ("sent_y", "m", ""), ("sent_z", "m", ""),
    ("hint_x", "m", "engaged: the base hint pose sent (world); NaN when released"), ("hint_y", "m", ""),
    ("hint_yaw", "rad", "engaged: the base hint heading sent"),
    ("head_x", "m", "engaged: the operator's head mapped into the world through the frozen frames (the base pose "
                    "at engagement + the head's travel); NaN when released"),
    ("head_y", "m", ""), ("head_yaw", "rad", ""),
    ("offset_x", "m", "engaged: the stick offset of the base hint (world)"), ("offset_y", "m", ""),
    ("offset_yaw", "rad", ""),
]
FIELDS = [c[0] for c in _COLS]
UNITS = [c[1] for c in _COLS]
FIELD_DOC = {c[0]: c[2] for c in _COLS if c[2]}


def _f(x) -> float:
    try:
        x = float(x)
    except (TypeError, ValueError):
        return nan
    return x


def _xr_index(v) -> int:
    if isinstance(v, bool) or v is None:
        return 0
    if isinstance(v, (int, float)):
        return int(v) if 0 <= int(v) < len(XR_STATES) else 0
    s = str(v).upper()
    return XR_STATES.index(s) if s in XR_STATES else 0


class HeadsetMonitor:
    """A tap over ``QuestReader`` (same ``get`` / ``drain``): link health from every frame that passes through."""

    def __init__(self, reader, clock: Callable[[], float] = time.monotonic):
        self.reader, self.clock = reader, clock
        self.arrivals: deque = deque(maxlen=400)
        self.last: Optional[dict] = None
        self.last_t: Optional[float] = None
        self.frames = 0
        self._lock = threading.Lock()

    def _seen(self, f) -> None:
        if f is None:
            return
        with self._lock:
            self.arrivals.append(f.arrived)
            self.last, self.last_t = f.data, f.arrived
            self.frames += 1

    def get(self, timeout=None):
        f = self.reader.get(timeout=timeout)
        self._seen(f)
        return f

    def drain(self) -> list:
        out = self.reader.drain()
        for f in out:
            self._seen(f)
        return out

    def hands(self) -> dict:
        d = self.last or {}
        intent = d.get("control_intent") if isinstance(d.get("control_intent"), dict) else {}
        hands = intent.get("hands") if isinstance(intent.get("hands"), dict) else {}
        out = {}
        for k in ("left", "right"):
            h = hands.get(k) if isinstance(hands.get(k), dict) else {}
            out[k] = {"valid": bool(h.get("valid")), "engaged": bool(h.get("engaged")),
                      "needs_release": bool(h.get("needs_release")),
                      "gripper": _f(h.get("gripper")) if h.get("gripper") is not None else None}
        return out

    def snapshot(self, now: float) -> dict:
        with self._lock:
            n = sum(1 for t in self.arrivals if now - t <= 1.0)
            d, t = self.last or {}, self.last_t
        intent = d.get("control_intent") if isinstance(d.get("control_intent"), dict) else {}
        r = self.reader
        return {"connected": bool(getattr(r, "connected", False)), "rate_hz": float(n),
                "age": None if t is None else round(now - t, 4), "frames": self.frames,
                "focused": d.get("input_focused") if d else None,
                "xr_state": d.get("xr_session_state") if d else None,
                "session_id": d.get("session_id") if d else None,
                "scheme": intent.get("scheme"),
                "connects": getattr(r, "counts", {}).get("connects"),
                "last_error": getattr(r, "last_error", None),
                "hands": self.hands() if d else None}


class DeadlineRunner:
    """quest_ctl's ssh runner under one overall deadline: each command's timeout is cut to what is left."""

    def __init__(self, inner, deadline: float, clock: Callable[[], float] = time.monotonic):
        self.inner, self.deadline, self.clock = inner, deadline, clock

    def __call__(self, command: str, timeout: float = 60.0):
        left = self.deadline - self.clock()
        if left <= 0:
            return quest_ctl.Result(124, "", "the headset command ran out of time (the node's --ctl-timeout)")
        return self.inner(command, min(timeout, left))


class QuestTeleopNode:
    def __init__(self, wbc: str = "wbc", *, quest_host: Optional[str] = None, quest_port: int = DEFAULT_PORT,
                 ssh_host: str = "quest", cfg: Optional[Config] = None, stale_s: float = STALE_S,
                 ar: str = ":80", scheme: str = "body-frozen", ctl_timeout: float = 240.0, wait: float = 30.0,
                 runner_factory: Optional[Callable] = None, probe: Optional[Callable] = None,
                 wbc_client_factory: Callable = LocalWbcClient, name: str = "quest_teleop",
                 clock: Callable[[], float] = time.monotonic):
        self.name = name
        self.wbc_target = wbc
        self.ssh_host = ssh_host
        self.quest_host = quest_host or quest_ctl.ssh_hostname(ssh_host)   # `ssh -G`: local, no network
        self.quest_port = quest_port
        self.cfg = cfg or Config()
        self.stale_s = stale_s
        self.ar = ar
        self.scheme = scheme
        self.ctl_timeout, self.wait = ctl_timeout, wait
        self.runner_factory = runner_factory or (lambda host: quest_ctl.SshRunner(host))
        self.probe = probe or quest_ctl.probe_stream
        self.wbc_client_factory = wbc_client_factory
        self.clock = clock
        self.hub = RowHub({"tick": len(FIELDS)})
        self.events = EventLog(50, on_add=self.hub.state_changed)
        self.reader = QuestReader(self.quest_host, quest_port)
        self.monitor = HeadsetMonitor(self.reader, clock)
        self.stream = WbcStream(wbc)
        self.lock = threading.RLock()
        self.step_lock = threading.Lock()
        self.client: Optional[TeleopClient] = None
        self.wbc: Optional[LocalWbcClient] = None
        self.phase = "stopped"            # stopped | starting | running | stopping
        self.stop_reason: Optional[str] = None
        self.error: Optional[str] = None
        self.started_at: Optional[float] = None
        self.last_msg: Optional[dict] = None
        self.last_sent_at: Optional[float] = None
        self.wbc_state: dict = {"reachable": False, "error": "not polled yet"}
        self.wbc_estop: Optional[dict] = None
        self.ctl: dict = {"busy": False, "command": None, "rc": None, "started": None, "finished": None,
                          "timed_out": False, "lines": deque(maxlen=120)}
        self.latest: Optional[dict] = None
        self._client_state = "stopped"
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # -- lifecycle -------------------------------------------------------------------------------------------

    def start(self) -> None:
        self.reader.start()
        self.stream.start()
        for target, name in ((self._loop, "quest-teleop"), (self._poll_wbc, "wbc-poll")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        self.events.add(f"headset stream {self.quest_host}:{self.quest_port}, ssh host {self.ssh_host!r}, "
                        f"WBC {self.wbc_target}")

    def close(self) -> None:
        self.stop_teleop("node shutting down")
        self._stop.set()
        for t in self._threads:
            t.join(2)
        self.reader.close()
        self.stream.close()

    # -- teleop ----------------------------------------------------------------------------------------------

    def start_teleop(self) -> dict:
        """Lease, session-default hints, enable, a datagram session (teleop.open_wbc), then the loop sends."""
        with self.lock:
            if self.phase != "stopped":
                raise RequestError(409, f"teleop is {self.phase}")
            self.phase, self.error, self.stop_reason = "starting", None, None
        self.hub.state_changed()
        wbc = self.wbc_client_factory(self.wbc_target, client_name=CLIENT_NAME)
        try:
            info = open_wbc(wbc, self.cfg)
        except Exception as e:  # noqa: BLE001 (any refusal: undo and report it)
            close_wbc(wbc, log=lambda s: None)
            wbc.close()
            msg = getattr(e, "detail", None) or str(e)
            with self.lock:
                self.phase, self.error = "stopped", f"start refused: {msg}"
            self.events.add(f"start refused: {msg}", "warn")
            raise RequestError(409, f"the WBC refused: {msg}") from None
        client = TeleopClient(self.cfg, self.monitor, self.stream, wbc, stale_s=self.stale_s, clock=self.clock)
        self.monitor.drain()            # frames from before the start are not this client's
        with self.lock:
            self.wbc, self.client = wbc, client
            self.phase, self.started_at = "running", self.clock()
            self.last_msg = None
        self.events.add(f"teleop started: WBC session {info['session']} over {wbc.transport}; "
                        f"the {self.cfg.hand} grip clutches the arm")
        return self.teleop_info()

    def stop_teleop(self, reason: str = "requested") -> dict:
        with self.lock:
            if self.phase != "running":
                return {"stopped": False, "phase": self.phase}
            self.phase = "stopping"
            client, wbc = self.client, self.wbc
            self.client = None
        self.hub.state_changed()
        with self.step_lock:            # the loop's current step finishes; nothing is sent after this
            pass
        problems = []
        close_wbc(wbc, log=problems.append)
        wbc.close()
        with self.lock:
            self.wbc, self.phase, self.stop_reason = None, "stopped", reason
            if problems:
                self.error = "stop: " + "; ".join(p.strip() for p in problems)
        self.events.add(f"teleop stopped ({reason})" + (f": {'; '.join(problems)}" if problems else ""),
                        "warn" if reason != "requested" else "info")
        return {"stopped": True, "reason": reason, "engagements": client.core.status.engagements}

    def _auto_stop(self, reason: str) -> None:
        threading.Thread(target=self.stop_teleop, args=(reason,), name="quest-teleop-stop", daemon=True).start()

    def estop(self, reason: str = "", source: str = "") -> dict:
        """Forward the e-stop to the WBC (which forwards it to the arm and the swerve), then stop teleop."""
        forwarded, err, downstream = False, None, None
        try:   # the WBC forwards to the arm and the swerve (0.5 s each) before it answers
            with NodeClient.for_instance(self.wbc_target, timeout=2.5) as c:
                r = c.post("/estop", {"reason": reason or "quest_teleop e-stop", "source": source or self.name})
            forwarded = bool(isinstance(r, dict) and r.get("latched"))
            downstream = r.get("forwarded") if isinstance(r, dict) else None
        except Exception as e:  # noqa: BLE001
            err = f"{type(e).__name__}: {e}"[:200]
        if forwarded:
            self.wbc_estop = {"latched": True, "reason": reason or "quest_teleop e-stop", "source": source or self.name,
                              "t": self.clock()}
        self.events.add("e-stop forwarded to the WBC" if forwarded else f"e-stop forward to the WBC FAILED: {err}",
                        "warn" if forwarded else "error")
        running = self.phase == "running"
        if running:
            self._auto_stop("e-stop")
        out = {"latched": forwarded, "forwarded": forwarded, "wbc_forwarded": downstream, "teleop_stopping": running}
        if err:
            out["error"] = err
        return out

    # -- headset control -------------------------------------------------------------------------------------

    def ar_url(self) -> Optional[str]:
        """The robot AR view the headset opens: ``--ar`` resolved like quest_ctl's (``:PORT`` = this machine's
        address on the route to the headset); None when off or there is no route."""
        if not self.ar or self.ar == "off":
            return None
        try:
            return quest_ctl.resolve_ar(self.ar, self.quest_host)
        except OSError as e:
            self._ctl_log(f"AR view off: no route to the headset at {self.quest_host} ({e})")
            return None

    def _ctl_log(self, line: str) -> None:
        for s in str(line).splitlines() or [""]:
            self.ctl["lines"].append({"t": self.clock(), "wall": time.time(), "line": s})
            log.info("quest %s: %s", self.ctl["command"], s)
        self.hub.state_changed()

    def quest_command(self, command: str) -> dict:
        """``up`` or ``down`` (quest_ctl), in the background with an overall timeout. Only ever on request."""
        if command not in ("up", "down"):
            raise RequestError(404, f"unknown headset command {command!r}")
        with self.lock:
            if self.ctl["busy"]:
                raise RequestError(409, f"quest {self.ctl['command']} is still running")
            self.ctl.update(busy=True, command=command, rc=None, started=self.clock(), finished=None,
                            timed_out=False)
            self.ctl["lines"].clear()
        threading.Thread(target=self._run_ctl, args=(command,), name=f"quest-{command}", daemon=True).start()
        self.events.add(f"quest {command} (ssh {self.ssh_host})")
        return self.ctl_info()

    def _run_ctl(self, command: str) -> None:
        deadline = self.clock() + self.ctl_timeout
        runner = DeadlineRunner(self.runner_factory(self.ssh_host), deadline, self.clock)
        ctl = quest_ctl.QuestCtl(runner, self.quest_host, port=self.quest_port, probe=self.probe, log=self._ctl_log,
                                 clock=self.clock)
        rc = None
        try:
            if command == "up":
                ar = self.ar_url()
                self._ctl_log(f"robot AR: {ar or 'off'}")
                wait = max(1.0, min(self.wait, deadline - self.clock()))
                rc = ctl.up(self.scheme, False, False, wait, ar)
            else:
                rc = ctl.down()
        except Exception as e:  # noqa: BLE001
            self._ctl_log(f"error: {type(e).__name__}: {e}")
            rc = 1
        timed_out = self.clock() >= deadline
        with self.lock:
            self.ctl.update(busy=False, rc=rc, finished=self.clock(), timed_out=timed_out)
        self._ctl_log(f"quest {command}: " + ("ok" if rc == 0 else f"failed (exit {rc})") +
                      (" — timed out" if timed_out else ""))
        self.events.add(f"quest {command} " + ("ok" if rc == 0 else f"failed (exit {rc})"),
                        "info" if rc == 0 else "warn")

    def ctl_info(self) -> dict:
        c = self.ctl
        return {"busy": c["busy"], "command": c["command"], "rc": c["rc"], "timed_out": c["timed_out"],
                "started": c["started"], "finished": c["finished"], "lines": list(c["lines"])}

    # -- the loop --------------------------------------------------------------------------------------------

    def _loop(self) -> None:
        next_row = 0.0
        while not self._stop.is_set():
            client = self.client
            if client is not None:
                with self.step_lock:
                    if self.client is client:
                        try:
                            msg = client.step(wait=0.02)
                        except Exception as e:  # noqa: BLE001 (a bug or a closed socket: stop, never spin)
                            log.exception("teleop step")
                            self.error = f"step: {type(e).__name__}: {e}"
                            self._auto_stop("client error")
                            msg = None
                            time.sleep(0.05)
                        if msg is not None:
                            self.last_msg, self.last_sent_at = msg, self.clock()
            else:
                self.monitor.get(timeout=0.02)
                self.monitor.drain()
            now = self.clock()
            if now >= next_row:
                next_row = now + ROW_PERIOD
                self._publish(now, client)
            if client is not None and self.phase == "running":
                self._watch(now)

    def _watch(self, now: float) -> None:
        """Stop when the WBC stream is gone for 2 s (the WBC node is down)."""
        _, t = self.stream.latest()
        if self.started_at is not None and now - self.started_at > 2.0 and (t is None or now - t > 2.0):
            self._auto_stop("the WBC stream is gone")

    def client_state(self) -> str:
        if self.phase in ("starting", "stopping"):
            return self.phase
        c = self.client
        if c is None:
            return "stopped"
        s = c.core.status
        if s.state == "needs_release" and s.reason.startswith("engagement refused"):
            return "refused"
        return s.state if s.state in CLIENT_STATES else "waiting"

    def _publish(self, now: float, client: Optional[TeleopClient]) -> None:
        row = dict.fromkeys(FIELDS, nan)
        cs = self.client_state()
        row.update(t=now, client_state=CLIENT_STATES.index(cs), sending=float(bool(client and client.sending)))
        h = self.monitor.snapshot(now)
        d = self.monitor.last or {}
        row.update(quest_connected=float(h["connected"]), quest_rate=h["rate_hz"],
                   quest_age=nan if h["age"] is None else h["age"],
                   quest_focused=nan if h["focused"] is None else float(bool(h["focused"])),
                   quest_xr=float(_xr_index(d.get("xr_session_state"))) if d else nan)
        for k, v in (h["hands"] or {}).items():
            row[f"{k}_valid"], row[f"{k}_engaged"] = float(v["valid"]), float(v["engaged"])
            row[f"{k}_gripper"] = nan if v["gripper"] is None else v["gripper"]
        wrow, wt = self.stream.latest()
        if wrow is not None:
            ee = self.cfg.ee
            row.update(wbc_state=wrow.get("state", nan), wbc_base_mode=wrow.get("base_mode", nan),
                       wbc_limited=wrow.get("limited", nan), wbc_limited_by=wrow.get("limited_by", nan),
                       wbc_age=now - wt, base_x=wrow.get("base_x", nan), base_y=wrow.get("base_y", nan),
                       base_yaw=wrow.get("base_yaw", nan))
            for a in "xyz":
                row[f"ee_cmd_{a}"] = wrow.get(f"{ee}_cmd_{a}", nan)
                row[f"ee_tgt_{a}"] = wrow.get(f"{ee}_tgt_{a}", nan)
        if client is not None:
            core = client.core
            row["quest_discontinuities"] = float(client.gate.counts.get("discontinuities", 0))
            row["engagements"] = float(core.status.engagements)
            row["sent"] = float(client.wbc.sent.get("target", 0))
            msg = self.last_msg
            try:
                pos = msg["ees"][self.cfg.ee]["pos"] if msg and msg.get("type") == "target" else None
            except (KeyError, TypeError):
                pos = None
            if pos is not None:
                row.update(sent_x=pos[0], sent_y=pos[1], sent_z=pos[2])
            eng, hint = core.eng, core.hint
            if eng is not None and hint is not None:
                row.update(hint_x=hint["pose"]["x"], hint_y=hint["pose"]["y"], hint_yaw=hint["heading"]["yaw"])
                o = core.offset
                row.update(offset_x=float(o[0]), offset_y=float(o[1]), offset_yaw=float(o[2]))
                try:
                    hd = eng.robot.r[:2, :2] @ np.asarray(eng.head_d, float)[:2]
                    row.update(head_x=float(eng.robot.p[0] + hd[0]), head_y=float(eng.robot.p[1] + hd[1]),
                               head_yaw=float(eng.robot.yaw + eng.head_dyaw))
                except (AttributeError, TypeError, ValueError):
                    pass
        values = [_f(row[k]) for k in FIELDS]
        self.hub.publish_rows(np.array([values]), "tick")
        self.latest = {k: (v if math.isfinite(v) else None) for k, v in zip(FIELDS, values)}
        if cs != self._client_state:
            self._client_state = cs
            self.hub.state_changed()

    # -- the WBC echo ----------------------------------------------------------------------------------------

    def _poll_wbc(self) -> None:
        c: Optional[NodeClient] = None
        lost = 0
        while not self._stop.wait(1.0):
            t0 = self.clock()
            try:
                if c is None:
                    c = NodeClient.for_instance(self.wbc_target, timeout=1.0)
                s = c.get("/state")
                e = s.get("estop") or {}
                lease = s.get("lease") or {}
                self.wbc_estop = e if e.get("latched") else None
                self.wbc_state = {
                    "reachable": True, "error": None, "state": s.get("state"), "enabled": s.get("enabled"),
                    "holder": s.get("holder"), "fault": s.get("fault"), "hold_reason": s.get("hold_reason"),
                    "disabled_reason": s.get("disabled_reason"), "limited_by": s.get("limited_by"),
                    "base_mode": ((s.get("hints") or {}).get("base") or {}).get("mode"),
                    "lease": {"held": bool(lease.get("held")), "holder": lease.get("holder")},
                    "estop": {"latched": bool(e.get("latched")), "reason": e.get("reason"), "source": e.get("source")},
                    "dropped": (s.get("session") or {}).get("dropped")}
            except Exception as ex:  # noqa: BLE001
                self.wbc_state = {"reachable": False, "error": f"{type(ex).__name__}: {ex}"[:200]}
                if c is not None:
                    c.close()
                    c = None
            # the watchdog: only polls that began after the start finished count
            if self.phase == "running" and self.started_at is not None and t0 > self.started_at + 0.5:
                w = self.wbc_state
                why = None
                if w.get("reachable"):
                    if w["estop"]["latched"]:
                        why = "the WBC e-stop is latched"
                    elif w.get("fault"):
                        why = f"WBC fault: {w['fault']}"
                    elif not w.get("enabled"):
                        why = f"the WBC was disabled ({w.get('disabled_reason') or 'no reason'})"
                    elif w["lease"]["holder"] != CLIENT_NAME:
                        why = f"the WBC lease is no longer ours ({w['lease']['holder'] or 'free'})"
                lost = lost + 1 if why else 0
                if lost >= 2 or (why and "e-stop" in why):
                    lost = 0
                    self._auto_stop(why)
            self.hub.state_changed()
        if c is not None:
            c.close()

    # -- state -----------------------------------------------------------------------------------------------

    def teleop_info(self) -> dict:
        c, w = self.client, self.wbc
        core = c.core if c else None
        return {"phase": self.phase, "state": self.client_state(),
                "sending": bool(c and c.sending), "reason": core.status.reason if core else None,
                "refused": dict(core.status.refused) if core else {},
                "engagements": core.status.engagements if core else 0,
                "sent": dict(w.sent) if w else None, "renew_errors": w.renew_errors if w else None,
                "transport": w.transport if w else None, "session": w.session if w else None,
                "last_sent_age": None if self.last_sent_at is None or c is None
                else round(self.clock() - self.last_sent_at, 3),
                "stop_reason": self.stop_reason, "started_at": self.started_at if c else None}

    def state(self) -> dict:
        now = self.clock()
        w = self.wbc_state
        lease = w.get("lease") or {"held": False, "holder": None}
        e = self.wbc_estop or {}
        _, wt = self.stream.latest()
        cfg = self.cfg
        return {
            "state": self.client_state(),
            "error": self.error,
            "teleop": self.teleop_info(),
            "headset": self.monitor.snapshot(now),
            "wbc": {**w, "stream_age": None if wt is None else round(now - wt, 4), "stream_error": self.stream.last_error},
            "lease": {"held": bool(lease.get("held")), "holder": lease.get("holder"),
                      "ours": self.phase == "running" and lease.get("holder") == CLIENT_NAME},
            "estop": {"latched": bool(e.get("latched")), "reason": e.get("reason"), "source": e.get("source"),
                      "t": e.get("t"), "on": "wbc"},
            "ctl": self.ctl_info(),
            "config": {"wbc": self.wbc_target, "quest_host": self.quest_host, "quest_port": self.quest_port,
                       "ssh_host": self.ssh_host, "ar": self.ar, "scheme": self.scheme, "hand": cfg.hand, "ee": cfg.ee,
                       "head_walk": cfg.head_walk, "gripper": cfg.gripper, "stale_ms": round(self.stale_s * 1e3, 1),
                       "ctl_timeout": self.ctl_timeout},
            "latest": self.latest,
            "events": self.events.list()[-30:],
        }
