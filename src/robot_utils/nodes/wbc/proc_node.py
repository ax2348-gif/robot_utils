"""The WBC node with its control loop in its own process (docs/wbc.md "Processes"): the server process's side.

``ProcNode`` has ``WbcNode``'s request API (server.py serves either). Here, in the server process: HTTP, the page, SSE,
the stream (rows from the control process's ring), the estimator and swerve stream readers (their rows into rings for
the control process), the swerve command sender (posting the twist the control process writes to a slot; muted on
e-stop / disable, as in-process), the client lease (renewed for every datagram the control process counts), the arm
and swerve leases (``open_peers`` at enable, released at disable), the e-stop fan-out, and the requests, which reach
the control process as commands on a datagram socket (answered: an HTTP reply waits for the control process's).

Python's GIL is per process: whatever runs here (uvicorn, SSE, the 100 Hz stream readers, the sender) never delays
the control tick, which runs alone in the control process (control_proc.py) at SCHED_FIFO on the robot.

Failure handling: the control process's heartbeat missing for ``ctl_heartbeat`` or the process gone -> a fault here
(the base muted, the swerve and the arm disabled and released, an event, /state ``fault``); nothing is restarted: the
operator restarts the node. The control process holds (disarms) when the server's heartbeat is missing.
"""
from __future__ import annotations

import logging
import math
import multiprocessing
import os
import select
import shutil
import threading
import time
from typing import Optional

from ..common.estop import EStop, EStopInfo
from ..common.hub import RowHub
from ..common.lease import Lease, RequestError
from ..common.server import EventLog
from . import ipc
from .arm_channel import joint_socket_path
from .model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from .node import (POSTURES, STRUCTURAL, config_doc, estop_fanout, forward_estop, hints_doc, model_doc, open_peers,
                   release_peers, state_document, stream_fields, validate_config)
from .spec import check_lead_torque, default_arm_gains, make_config, parse_hints

log = logging.getLogger("wbc")

PUMP_S = 0.002                  # the server's pump: the twist to the swerve sender, rows to the hub, the heartbeat
SLOW_S = 0.01                   # lease expiry, the server slot, the control heartbeat watch


class _RingFeed:
    """An ``Upstream``'s output queue: the estimator's / the swerve's rows straight into the control process's
    rings (on this host's clock), from the reader's own thread (one writer per ring)."""

    def __init__(self, shm: ipc.Shared):
        self.shm = shm

    def put(self, item, block=True, timeout=None) -> None:
        key, rows, offset = item
        names = rows.dtype.names
        if key == "estimator":
            for r in rows:
                get = lambda k, d: float(r[k]) if k in names else d      # noqa: E731
                self.shm.est.write([float(r["t"]) - offset, float(r["x"]), float(r["y"]), float(r["theta"]),
                                    get("reset_id", 0.0), get("imu_ok", 0.0), get("odom_ok", 1.0)])
        elif key == "swerve" and len(rows):
            r = rows[-1]
            self.shm.sw.write([float(r["t_tick"]) - offset, float(r["enabled"]), float(r["estop"]),
                               float(r["lease_id"])])

    put_nowait = put


class ProcNode:
    STRUCTURAL = STRUCTURAL
    ESTOP_JOIN_S = 2.5          # the fan-out waits this long for the arm's and the swerve's answers
    ESTOP_ACK_S = 0.2           # the e-stop's first step (the control process stops tracking) waits this long
    START_TIMEOUT_S = 60.0      # the control process's start (imports, the model, the QP)

    def __init__(self, model: Optional[WbcModel] = None, *, model_source: str = "description",
                 arm: Optional[str] = "arm", swerve: Optional[str] = "swerve", estimator: Optional[str] = "estimator",
                 config: Optional[dict] = None, udp_host: Optional[str] = "127.0.0.1", udp_port: int = 9410,
                 unix_socket: Optional[str] = None, arm_socket: Optional[str] = None, name: str = "wbc",
                 log_level: int = logging.INFO):
        self.config_in = dict(config or {})
        self.cfg = make_config(config)
        kp0, ml0, _ = default_arm_gains()
        check_lead_torque(self.cfg.lead_torque, kp0, ml0)
        self.clock = time.monotonic                 # one clock for both processes
        self.name = name
        self.arm_target, self.swerve_target, self.estimator_target = arm, swerve, estimator
        self.model_source = model_source
        self.model: Optional[WbcModel] = model
        self.model_info = "given" if model is not None else None
        self.model_error: Optional[str] = None
        self.ees: list = []
        self.fields: list = []
        self.hub: Optional[RowHub] = None
        self.lock = threading.RLock()
        self.events = EventLog()
        self.lease = Lease(self.cfg.lease_heartbeat, on_end=self._client_lease_ended, clock=self.clock)
        self.estop = EStop(on_trigger=self._estop_triggered, clock=self.clock)
        self.inputs: dict = {}
        if unix_socket is None:
            from ..common.serve import socket_dir
            unix_socket = str(socket_dir() / f"{name}.target.sock")
        self.udp_host, self.udp_port, self.unix_socket = udp_host, udp_port, unix_socket
        self.arm_socket = arm_socket or (str(joint_socket_path(arm)) if arm and "://" not in arm else None)
        self.log_level = log_level
        self.session_token: Optional[str] = None
        self.arm_http = None
        self.base = None
        self.gen = 0                                 # enables so far: the twist slot's base generation
        self._enabling = False
        self.proc = None
        self.shm: Optional[ipc.Shared] = None
        self.port: Optional[ipc.Port] = None
        self.dir: Optional[str] = None
        self.ready: dict = {}
        self.proc_fault: Optional[str] = None
        self._rid = 0
        self._waits: dict = {}
        self._stop = threading.Event()
        self._threads: list = []
        self._stats = {"pump_max_ms": 0.0, "rows": 0}
        self._srv_lock = threading.Lock()
        self._closed = False
        self.freeze_heartbeat = False                # tests: a stalled server process, as the control process sees it
        if self.model is not None:
            self._setup_model()

    # -- model ---------------------------------------------------------------------------------------------

    def _setup_model(self) -> None:
        self.ees = list(self.model.ees)
        self.fields, self.units, self.field_doc = stream_fields(self.ees)
        self.hub = RowHub({"tick": len(self.fields)}, recent=500)
        self.events.on_add = self.hub.state_changed

    def load_model(self) -> None:
        urdf, how = fetch_urdf(self.model_source)
        self.model = WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))
        self.model_info = how
        self._setup_model()

    # -- lifecycle -----------------------------------------------------------------------------------------

    def start(self, loop: bool = True) -> None:
        """Load the model, spawn the control process (waits until it runs), start the stream readers and the
        server's threads."""
        if self.model is None:
            self.load_model()
        from .control_proc import control_main
        self.dir = ipc.runtime_dir()
        self.shm = ipc.Shared(os.path.join(self.dir, "shm"), len(self.fields), create=True)
        self.shm.beats[ipc.SRV_HB] = time.monotonic()
        self._write_srv(force=True)
        srv_sock, ctl_sock = os.path.join(self.dir, "srv.sock"), os.path.join(self.dir, "ctl.sock")
        self.port = ipc.Port(srv_sock, ctl_sock)
        m = self.model
        params = {"shm": self.shm.path, "width": len(self.fields), "srv_sock": srv_sock, "ctl_sock": ctl_sock,
                  "urdf": m.urdf, "ees": m.ee_specs, "margin": m.margin, "fields": self.fields,
                  "arm": self.arm_target, "swerve": self.swerve_target, "estimator": self.estimator_target,
                  "config": self.config_in, "udp_host": self.udp_host, "udp_port": self.udp_port,
                  "unix_socket": self.unix_socket, "arm_socket": self.arm_socket, "name": self.name,
                  "ppid": os.getpid(), "log_level": self.log_level}
        ctx = multiprocessing.get_context("spawn")
        self.proc = ctx.Process(target=control_main, args=(params,), name=f"{self.name}-control", daemon=True)
        self.proc.start()
        try:
            self._wait_ready()
        except BaseException:
            self._cleanup()
            raise
        self._threads = [threading.Thread(target=self._rx, name="wbc-ipc-rx", daemon=True),
                         threading.Thread(target=self._pump, name="wbc-ipc-pump", daemon=True)]
        for t in self._threads:
            t.start()
        from ..estimator.node import Upstream
        feed = _RingFeed(self.shm)
        if self.estimator_target:
            self.inputs["estimator"] = Upstream("estimator", self.estimator_target, "/stream", feed, self.events)
        if self.swerve_target:
            self.inputs["swerve"] = Upstream("swerve", self.swerve_target, "/stream", feed, self.events)
        for u in self.inputs.values():
            u.start()
        self.events.add(f"wbc started (model: {self.model_info}; arm {self.arm_target}, swerve {self.swerve_target}, "
                        f"estimator {self.estimator_target}; targets udp {self.ready.get('udp_port')}, "
                        f"unix {self.ready.get('unix_socket')}; control process pid {self.ready.get('pid')})")

    def _wait_ready(self) -> None:
        end = time.monotonic() + self.START_TIMEOUT_S
        while time.monotonic() < end:
            if not self.proc.is_alive():
                raise RuntimeError(f"the WBC control process exited at start (code {self.proc.exitcode})")
            self.shm.beats[ipc.SRV_HB] = time.monotonic()
            r, _, _ = select.select([self.port.sock], [], [], 0.05)
            if not r:
                continue
            for m in self.port.recv_all():
                if m[0] == "ready":
                    if not m[1]:
                        raise RuntimeError(f"the WBC control process failed to start: {m[2]}")
                    self.ready = m[2]
                    return
                self._dispatch(m)
        raise RuntimeError(f"the WBC control process did not start within {self.START_TIMEOUT_S:g} s")

    def close(self) -> None:
        if self.proc is None or self._closed:
            return
        self._closed = True
        for u in self.inputs.values():
            u.stop.set()
        if self.proc_fault is None and (self.enabled or self.base is not None or self.arm_http is not None):
            self.disable("node shutting down")
        self._stop.set()
        if self.port is not None:
            self.port.send(("stop", None))
        self.proc.join(2.0)
        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(1.0)
        if self.proc.is_alive():
            self.proc.kill()
            self.proc.join(1.0)
        for t in self._threads:
            t.join(1.0)
        with self.lock:
            base, arm_http, self.base, self.arm_http = self.base, self.arm_http, None, None
        release_peers(arm_http, base, background=True)
        self._cleanup()

    def _cleanup(self) -> None:
        if self.proc is not None and self.proc.is_alive():
            self.proc.kill()
            self.proc.join(1.0)
        if self.port is not None:
            self.port.close()
        if self.shm is not None:
            self.shm.close()
        if self.dir:
            shutil.rmtree(self.dir, ignore_errors=True)

    # -- the server's threads ------------------------------------------------------------------------------

    def _rx(self) -> None:
        """Messages from the control process: replies, events, the base's mute / close, cross-bus e-stops."""
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([self.port.sock], [], [], 0.2)
            except (OSError, ValueError):
                return
            if r:
                for m in self.port.recv_all():
                    try:
                        self._dispatch(m)
                    except Exception:  # noqa: BLE001
                        log.exception("control message %r", m[:1])

    def _dispatch(self, m) -> None:
        kind = m[0]
        if kind == "reply":
            w = self._waits.get(m[1])
            if w is not None:
                w[1] = (m[2], m[3])
                w[0].set()
        elif kind == "ev":
            self.events.add(m[1], m[2])
        elif kind == "base_mute":
            b = self.base
            if b is not None and m[1] == self.gen:
                b.mute()
        elif kind == "base_close":
            with self.lock:
                b = self.base if m[1] == self.gen else None
                if b is not None:
                    self.base = None
            if b is not None:
                b.close(disable=True)
        elif kind == "fwd_estop":
            forward_estop(self.arm_http, self.swerve_target, m[1], self.events, base=m[2], arm=m[3])

    def _pump(self) -> None:
        """Every PUMP_S: the heartbeat, the twist to the swerve sender, the rows to the hub, the lease renewals;
        every SLOW_S: the lease's expiry, the server slot, the control process's heartbeat."""
        shm = self.shm
        last_tw = None
        last_rx = 0.0
        slow = 0.0
        while not self._stop.is_set():
            t0 = time.monotonic()
            if not self.freeze_heartbeat:
                shm.beats[ipc.SRV_HB] = t0
            try:
                tw = shm.twist.read()
                if tw is not None and (tw["gen"], tw["seq"]) != last_tw:
                    last_tw = (tw["gen"], tw["seq"])
                    b = self.base
                    if b is not None and tw["gen"] == self.gen:
                        if tw["muted"]:
                            b.mute()
                        else:
                            b.send(tw["twist"])
                rows = shm.rows.read()
                if len(rows):
                    self.hub.publish_rows(rows, "tick")
                    self._stats["rows"] += len(rows)
                rx = shm.beats[ipc.RX_COUNT]
                if rx != last_rx:
                    last_rx = rx
                    tok = self.session_token
                    if tok is not None:
                        try:
                            self.lease.renew(tok)
                        except Exception:  # noqa: BLE001 - lapsed: the control process holds
                            pass
                if t0 - slow >= SLOW_S:
                    slow = t0
                    self.lease.expire_if_due()
                    self._write_srv()
                    self._watch_control(t0)
            except Exception:  # noqa: BLE001 - never let the pump die
                log.exception("wbc pump")
            dt = time.monotonic() - t0
            self._stats["pump_max_ms"] = max(self._stats["pump_max_ms"], dt * 1e3)
            time.sleep(max(0.0, PUMP_S - dt))

    _srv_last = None
    _srv_t = -math.inf

    def _write_srv(self, force: bool = False) -> None:
        b = self.base
        f = getattr(b, "_fail_since", None) if b is not None else None
        d = {"lease_held": self.lease.held, "gen": self.gen, "fail_since": f,
             "last_error": getattr(b, "last_error", None) if b is not None else None}
        with self._srv_lock:                         # one writer at a time (the pump, an enable)
            now = time.monotonic()
            if force or d != self._srv_last or now - self._srv_t > 0.1:
                self._srv_last, self._srv_t = d, now
                self.shm.srv.write(d)

    def _watch_control(self, now: float) -> None:
        if self.proc_fault is not None or self._stop.is_set():
            return
        if not self.proc.is_alive():
            self._proc_failed(f"the control process exited (code {self.proc.exitcode})")
            return
        age = now - self.shm.beats[ipc.CTL_HB]
        if age > self.cfg.ctl_heartbeat:
            self._proc_failed(f"no control process heartbeat for {age:.2f} s")

    def _proc_failed(self, reason: str) -> None:
        """The control process is lost: a fault (the base muted, the swerve and the arm disabled and released),
        latched until the node restarts."""
        with self.lock:
            if self.proc_fault is not None:
                return
            self.proc_fault = reason
            base, arm_http, self.base, self.arm_http = self.base, self.arm_http, None, None
        if base is not None:
            base.mute()
        log.error("control process: %s", reason)
        self.events.add(f"FAULT: {reason}: the swerve and the arm are disabled and released; restart the node",
                        "error")
        self.port.send(("disable", None, f"server: {reason}"))       # if it is only stalled: it lets go too
        release_peers(arm_http, base, background=True)

    # -- commands ------------------------------------------------------------------------------------------

    def _call(self, kind: str, *args, timeout: float = 1.0):
        if self.proc_fault is not None:
            raise RequestError(503, f"the WBC control process is down ({self.proc_fault}): restart the node")
        if self.port is None:
            raise RequestError(503, "the WBC control process is not running")
        with self.lock:
            self._rid += 1
            rid = self._rid
            w = self._waits[rid] = [threading.Event(), None]
        try:
            if not self.port.send((kind, rid, *args)):
                raise RequestError(503, f"the control process's command channel refused {kind}")
            if not w[0].wait(timeout):
                raise RequestError(503, f"the control process did not answer {kind} within {timeout:g} s")
        finally:
            self._waits.pop(rid, None)
        ok, payload = w[1]
        if not ok:
            raise RequestError(payload[0], payload[1])
        return payload

    def snap(self) -> dict:
        return (self.shm.snap.read() if self.shm is not None else None) or {}

    @property
    def enabled(self) -> bool:
        return bool(self.snap().get("enabled")) and self.proc_fault is None

    @property
    def state(self) -> str:
        if self.proc_fault is not None:
            return "fault"
        if self._enabling:
            return "enabling"
        return self.snap().get("state", "disabled")

    @property
    def armed(self) -> bool:
        return bool(self.snap().get("armed")) and self.proc_fault is None

    def acquire(self, client: str):
        token, info = self.lease.acquire(client)
        self.events.add(f"lease {info.lease_id} acquired by {info.client}")
        return token, info

    def renew(self, token):
        return self.lease.renew(token)

    def release(self, token) -> None:
        self.lease.release(token)

    def _client_lease_ended(self, reason: str) -> None:
        self.session_token = None
        if self.port is not None:
            self.port.send(("lease_ended", None, reason))
        en = self.enabled
        self.events.add(f"client lease {reason}" + (": holding (POST /disable releases the arm and base)"
                                                    if en else ""), "warn" if en else "info")

    def _estop_triggered(self, info: EStopInfo) -> None:
        self.events.add(f"E-STOP ({info.source}: {info.reason})", "error")

    def enable(self, token: Optional[str]) -> dict:
        """The arm and swerve leases together (rolled back on any failure), both enabled; then the control
        process opens the arm's joint channel and seeds the commands. Blocking."""
        self.lease.check(token)
        if self.estop.active:
            raise RequestError(409, "e-stop is latched; clear it first (POST /estop/clear)")
        if self.proc_fault is not None:
            raise RequestError(503, f"the WBC control process is down ({self.proc_fault}): restart the node")
        with self.lock:
            st = self.state
            if self._enabling or self.enabled or st == "enabling":
                raise RequestError(409, f"already {st}; disable first")
            self._enabling = True
            holder = self.lease.info.client if self.lease.info else "anonymous"
        try:
            peers = open_peers(self.arm_target, self.swerve_target, self.cfg, f"wbc ({holder})",
                               len(self.model.arm_q["arm"]), self.events,
                               self.snap().get("kp_source") or "the arm's default profile")
            try:
                with self.lock:
                    self.gen += 1
                    self.base, self.arm_http = peers.base, peers.arm_http
                    gen = self.gen
                self._write_srv(force=True)
                out = self._call("enable", {
                    "arm_token": peers.arm_http.token if peers.arm_http is not None else None,
                    "arm_lease": peers.arm_lease, "tcp": peers.tcp,
                    "kp": None if peers.kp is None else [float(x) for x in peers.kp], "kp_source": peers.kp_source,
                    "base_lease_id": peers.base.lease_id if peers.base is not None else None, "gen": gen,
                    "holder": holder}, timeout=3.0)
            except BaseException:
                with self.lock:
                    self.base = self.arm_http = None
                release_peers(peers.arm_http, peers.base, background=False)
                raise
            if peers.tcp is not None:
                self.model.set_tcp("arm", peers.tcp)              # this side's copy (/state's model)
        finally:
            self._enabling = False
        return out["status"]

    def disable(self, reason: str = "requested", token: Optional[str] = None, check: bool = False) -> dict:
        if check:
            self.lease.check(token)
        with self.lock:
            base, arm_http = self.base, self.arm_http
            self.base = self.arm_http = None
        if base is not None:
            base.mute()              # disable coasts: no further twist (no zero twist), then the swerve disables
        try:
            out = self._call("disable", reason, timeout=1.0)   # a last hold to the arm, its channel closed
        except RequestError:
            out = {"enabled": False, "was_enabled": base is not None or arm_http is not None}
        release_peers(arm_http, base, background=True)
        return out

    def trigger_estop(self, reason: str, source: str) -> dict:
        """Anyone. The base muted here at once; the control process stops tracking (disarm, the arm held at its
        commanded joints); then the e-stop to the arm and the swerve IN PARALLEL; then disable."""
        info = self.estop.trigger(reason, source)
        b = self.base
        if b is not None:
            b.mute()                                 # the base gets NOTHING more (no zero twist: it coasts)
        try:
            self._call("estop", reason, timeout=self.ESTOP_ACK_S)
        except RequestError as e:
            self.events.add(f"e-stop: the control process did not confirm ({e}); forwarding anyway", "error")
        fw = estop_fanout(self.arm_http, self.arm_target, self.swerve_target, reason, source, self.events,
                          self.ESTOP_JOIN_S)
        out = {"latched": True, "reason": info.reason, "source": info.source, "forwarded": fw}
        self.disable(f"e-stop ({reason or 'no reason'})")
        return out

    def clear_estop(self, token: Optional[str]) -> None:
        self.lease.check(token)
        self.estop.clear()
        if self.proc_fault is None:
            self._call("estop_clear")
        self.events.add("e-stop cleared (the arm's and the swerve's are cleared there)")

    def open_session(self, token: Optional[str]) -> dict:
        self.lease.check(token)
        out = self._call("session", token)
        self.session_token = token
        return {"session": out["session"], "udp_port": self.ready.get("udp_port"),
                "unix_socket": self.ready.get("unix_socket"), "arm_seq": out["arm_seq"]}

    def disarm(self, reason: str = "", source: str = "") -> dict:
        why = "POST /disarm" + (f" ({source})" if source else "") + (f": {reason}" if reason else "")
        return self._call("disarm", why)

    def set_hints(self, token: Optional[str], doc: dict) -> dict:
        self.lease.check(token)
        h = parse_hints(doc, self.cfg, self.ees, POSTURES)
        return self._call("hints", h)

    def get_hints(self) -> dict:
        sn = self.snap()
        return hints_doc(sn["hints"], sn["in_force"])

    def get_config(self) -> dict:
        return config_doc(self.cfg, self.base)

    def set_config(self, changes: dict) -> dict:
        """As ``WbcNode.set_config``: validated here (the control state from its snapshot), the swerve's limits
        pushed, then applied by the control process (which checks again that the base is still)."""
        changes = dict(changes or {})
        sn = self.snap()
        with self.lock:
            base = self.base
            hard = getattr(base, "hard", None) or {} if base is not None else {}
            new = validate_config(self.cfg, changes, bool(sn.get("enabled")), sn.get("base_twist", (0, 0, 0)),
                                  sn.get("kp"), sn.get("max_lead"), hard)
            old = self.cfg
            push = bool(sn.get("enabled")) and base is not None and (
                (new.max_speed, new.max_omega) != (old.max_speed, old.max_omega))
        if push:
            base.client.set_limits(new.max_speed, new.max_omega)
        try:
            out = self._call("config", changes)
        except RequestError as e:
            if push and base is not None and base.client is not None:
                try:
                    base.client.set_limits(old.max_speed, old.max_omega)
                except Exception:  # noqa: BLE001
                    pass
            if e.status == 422:
                raise ValueError(str(e)) from None
            raise
        self.cfg = out["cfg"]
        return self.get_config()

    # -- state ---------------------------------------------------------------------------------------------

    def status(self) -> dict:
        sn = self.snap()
        return {"state": self.state, "enabled": self.enabled, "fault": self.proc_fault or sn.get("fault"),
                "armed": self.armed}

    def state_dict(self) -> dict:
        now = self.clock()
        sn = dict(self.snap())
        if not sn:
            raise RequestError(503, "the WBC control process has not published its state yet")
        if self.proc_fault is not None:
            sn.update(state="fault", enabled=False, armed=False, fault=self.proc_fault,
                      disarm_reason=f"fault: {self.proc_fault}")
        elif self._enabling:
            sn["state"] = "enabling"
        base = self.base
        last = self.hub.recent(1, "tick") if self.hub else []
        srv = {"estop": self.estop.info, "lease": self.lease.info, "arm_target": self.arm_target,
               "swerve_target": self.swerve_target, "model": model_doc(self.model, self.model_info, self.model_error),
               "base": None if base is None else {"lease_id": base.lease_id, "sent": base.sent,
                                                  "errors": base.errors},
               "inputs": {k: u.status() for k, u in self.inputs.items()},
               "latest": ({k: (v if math.isfinite(v) else None) for k, v in zip(self.fields, last[0].tolist())}
                          if len(last) else None),
               "events": self.events.list()[-30:]}
        doc = state_document(sn, now, srv)
        shm = self.shm
        doc["process"] = {
            "mode": "process", "pid": self.ready.get("pid"),
            "alive": bool(self.proc is not None and self.proc.is_alive()), "fault": self.proc_fault,
            "heartbeat_age": None if shm is None else round(time.monotonic() - shm.beats[ipc.CTL_HB], 4),
            "snapshot_age": round(now - sn["t"], 4),
            "rows_dropped": None if shm is None else shm.rows.dropped,
            "commands_dropped": None if self.port is None else self.port.dropped,
            "pump_max_ms": round(self._stats["pump_max_ms"], 3)}
        return doc
