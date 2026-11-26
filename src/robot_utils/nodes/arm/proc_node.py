"""The arm node with its control loop and bus in their own process (docs/arm.md "Processes"): the server process's
side.

``ProcNode`` has ``ArmNode``'s request API (server.py serves either). Here, in the server process: HTTP and the page,
SSE, ``/stream`` (the rows from the control process's rings), the lease (its holder and a digest of its token
published to the control process; renewed for every joint-channel datagram the control process counts), the e-stop
latch, the chocks' check at enable, the track IK and the planning (``ArmRequests``, the same code as in-process,
on the latest snapshot of the control state), the read-only lookups (kinematics, geometry, IK checks: this side's
own model, kept in step with the control process's TCP offset). Everything that changes the control state reaches
the control process as a command on a datagram socket and is answered (the HTTP reply waits for it): enable
(the control process runs the gripper check; its heartbeat is excused meanwhile), disable, float, the solved track,
the planned motion, gripper, cancel, reset, profile, TCP, limits, jam, the e-stop.

Python's GIL is per process: whatever runs here (uvicorn, SSE, the stream clients, IK, the planner's results) never
delays the control tick, which runs alone in the control process (control_proc.py) at SCHED_FIFO 40 on the robot.

Failure handling: the control process's heartbeat missing for CTL_HEARTBEAT (outside an announced blocking step) or
the process gone -> a fault here (an event; /state ``fault``, the motors reported off; every request needing the
control process answers 503 "restart the node"). The motors go to damping: if the control process is gone, by their
own 100 ms CAN watchdog; if it is only stalled it is told to fault (motors off) and, still silent KILL_AFTER_S later,
killed (the port is released; the watchdog again). Nothing restarts automatically: the operator restarts the node.
"""
from __future__ import annotations

import logging
import multiprocessing
import os
import pickle
import select
import shutil
import threading
import time
from typing import Optional

import numpy as np

from ..common import ipc
from ..common.estop import EStop, EStopInfo
from ..common.fastpath import unlink_socket
from ..common.hub import RowHub
from ..common.lease import Lease, LeaseError, RequestError
from ..common.locks import LockStore, describe as describe_locks
from ..common.server import EventLog
from . import config as C
from .control_proc import (BULK_MIN, CTL_BUSY, CTL_HB, RX_COUNT, RX_LEASE, SERVER_CHECKED, SRV_HB, control_main,
                           shared, token_digest)
from .kin import ArmKinematics
from .node import ENABLE_TIMEOUT, LEASE_HEARTBEAT, compose_state
from .planner import Planner
from .requests import ArmRequests
from .telemetry import FIELDS, JAC_FIELDS

log = logging.getLogger("arm")

SERVICE_S = 0.01                # the server's service runs on every control tick's "t" message, at least this often


class ProcNode(ArmRequests):
    KILL_AFTER_S = 1.0          # s after the fault a still-silent control process is killed
    ESTOP_ACK_S = 0.2           # s the e-stop waits for the control process (the motors off)
    CALL_S = 2.0                # s a command waits for its answer
    START_TIMEOUT_S = 60.0      # the control process's start (imports, the bus object, the joint channel)

    def __init__(self, bus_factory, profile: str = C.DEFAULT_PROFILE, settings: Optional[C.NodeSettings] = None,
                 locks: Optional[LockStore] = None, description_node: str = "description", joint_socket=None,
                 log_level: int = logging.INFO, urdf: Optional[str] = None, arm_frame: str = "arm",
                 robot: Optional[str] = None):
        """``bus_factory``: picklable (it is called in the control process), e.g. a ``functools.partial`` of a bus
        class. ``urdf``, ``arm_frame``, ``robot``: as ArmNode's."""
        self.bus_factory = bus_factory
        self.urdf, self.arm_frame, self.robot = urdf, arm_frame, robot
        self.profile0 = C.load_profile(profile).name
        self._profiles = {self.profile0: C.load_profile(profile)}
        self.settings = settings or C.NodeSettings()
        self.ctl_heartbeat = self.settings.ctl_heartbeat     # s: the control process silent this long -> fault
        self.description_node = description_node
        self.clock = time.monotonic                 # one clock for both processes
        self.locks = locks or LockStore()
        self.lock_names = {"arm"}
        self.hub = RowHub({"tick": len(FIELDS), "jacobian": len(JAC_FIELDS)})
        self.events = EventLog(on_add=self.hub.state_changed)
        self.lease = Lease(LEASE_HEARTBEAT, on_end=self._lease_ended, clock=self.clock)
        self.estop = EStop(on_trigger=self._estop_triggered)
        self.joint_socket = None if joint_socket is None else str(joint_socket)
        self.log_level = log_level
        self.kin: Optional[ArmKinematics] = None
        self._kin_key = None
        self._aib: Optional[dict] = None
        self._plan_lock = threading.Lock()
        self._lease_digest: Optional[tuple] = None   # (lease id, digest of its token)
        self.proc = None
        self.shm = None
        self.port: Optional[ipc.Port] = None
        self.dir: Optional[str] = None
        self.ready: dict = {}
        self.proc_fault: Optional[str] = None
        self._fault_t = 0.0
        self.busy: Optional[str] = None
        self._rid = 0
        self._waits: dict = {}
        self._wlock = threading.Lock()
        self._bulk_lock = threading.Lock()          # one large command in the bulk slot at a time, until answered
        self._srv_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list = []
        self._stats = {"service_max_ms": 0.0}
        self._last_rx = 0.0
        self._closed = False
        self.freeze_heartbeat = False                # tests: a stalled server process, as the control process sees it

    # -- lifecycle -----------------------------------------------------------------------------------------

    def start(self) -> None:
        """Spawn the control process (waits until it runs: the bus object built, the joint channel bound) and start
        the server's IPC thread. Connecting to the arm happens in the control loop (state ``connecting``)."""
        self.dir = ipc.runtime_dir("arm-")
        try:
            self.shm = shared(os.path.join(self.dir, "shm"), create=True)
            self.shm.beats[SRV_HB] = time.monotonic()
            self._write_srv()
            srv_sock, ctl_sock = os.path.join(self.dir, "srv.sock"), os.path.join(self.dir, "ctl.sock")
            self.port = ipc.Port(srv_sock, ctl_sock)
            params = {"shm": self.shm.path, "srv_sock": srv_sock, "ctl_sock": ctl_sock,
                      "bus_factory": self.bus_factory, "profile": self.profile0, "settings": self.settings,
                      "locks_path": str(self.locks.path), "lock_names": sorted(self.lock_names),
                      "joint_socket": self.joint_socket, "description_node": self.description_node,
                      "urdf": self.urdf, "arm_frame": self.arm_frame, "robot": self.robot,
                      "ppid": os.getpid(), "log_level": self.log_level}
            ctx = multiprocessing.get_context("spawn")
            self.proc = ctx.Process(target=control_main, args=(params,), name="arm-control", daemon=True)
            self.proc.start()
            self._wait_ready()
        except BaseException:
            self._cleanup()
            raise
        self._threads = [threading.Thread(target=self._rx, name="arm-ipc", daemon=True)]
        for t in self._threads:
            t.start()
        self.events.add(f"control loop in its own process (pid {self.ready.get('pid')})")

    def _wait_ready(self) -> None:
        end = time.monotonic() + self.START_TIMEOUT_S
        while time.monotonic() < end:
            if not self.proc.is_alive():
                raise RuntimeError(f"the arm's control process exited at start (code {self.proc.exitcode})")
            self.shm.beats[SRV_HB] = time.monotonic()
            r, _, _ = select.select([self.port.sock], [], [], 0.05)
            if not r:
                continue
            for m in self.port.recv_all():
                if m[0] == "ready":
                    if not m[1]:
                        raise RuntimeError(f"the arm's control process failed to start: {m[2]}")
                    self.ready = m[2]
                    return
                self._dispatch(m)
        raise RuntimeError(f"the arm's control process did not start within {self.START_TIMEOUT_S:g} s")

    def close(self) -> None:
        if self.proc is None or self._closed:
            return
        self._closed = True
        self._stop.set()
        if self.port is not None:
            self.port.send(("stop", None))       # the control process: the joint channel's bye, the motors off
        self.proc.join(5.0)
        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(1.0)
        if self.proc.is_alive():
            self.proc.kill()
            self.proc.join(1.0)
        for t in self._threads:
            t.join(1.0)
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
        if self.joint_socket and self.proc is not None and not self.proc.is_alive():
            unlink_socket(self.joint_socket)     # left behind by a killed control process

    # -- the server's IPC thread ---------------------------------------------------------------------------

    def _rx(self) -> None:
        """Messages from the control process (replies, events, a "t" after every control tick); on each tick (and at
        least every SERVICE_S): ``_service``. Event-driven: no polling faster than the control loop."""
        last = 0.0
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([self.port.sock], [], [], max(0.0, last + SERVICE_S - time.monotonic()))
            except (OSError, ValueError):
                return
            tick = False
            if r:
                for m in self.port.recv_all():
                    if m[0] == "t":
                        tick = True
                        continue
                    try:
                        self._dispatch(m)
                    except Exception:  # noqa: BLE001
                        log.exception("control message %r", m[:1])
            now = time.monotonic()
            if tick or now - last >= SERVICE_S:
                last = now
                self._service(now)

    def _dispatch(self, m) -> None:
        kind = m[0]
        if kind == "reply":
            w = self._waits.get(m[1])
            if w is not None:
                w[1] = (m[2], m[3])
                w[0].set()
        elif kind == "ev":
            self.events.add(m[1], m[2])
        elif kind == "sc":
            self.hub.state_changed()
        elif kind == "busy":
            self.busy = m[1]

    def _service(self, t0: float) -> None:
        """The heartbeat, the rows to the hub, the lease renewals and expiry, the model, the control heartbeat."""
        shm = self.shm
        if not self.freeze_heartbeat:
            shm.beats[SRV_HB] = t0
        try:
            rows = shm.rows.read()
            if len(rows):
                self.hub.publish_rows(rows, "tick")
            jac = shm.jac.read()
            if len(jac):
                self.hub.publish_rows(jac, "jacobian")
            rx = shm.beats[RX_COUNT]
            if rx != self._last_rx:
                self._last_rx = rx
                self.lease.renew_holder(int(shm.beats[RX_LEASE]))
            self.lease.expire_if_due()
            self._sync_model()
            self._watch_control(t0)
        except Exception:  # noqa: BLE001 - never let the service die
            log.exception("arm service")
        self._stats["service_max_ms"] = max(self._stats["service_max_ms"], (time.monotonic() - t0) * 1e3)

    def _sync_model(self) -> None:
        """This side's model: built from the control process's setpoint limits once it has connected (and the planner
        warmed), its TCP offset kept equal to the control process's."""
        sn = self.snap()
        lim = sn.get("setpoint_limits")
        if lim is None:
            return
        key = tuple(sorted((k, tuple(v)) for k, v in lim.items()))
        if self.kin is None or key != self._kin_key:
            self.kin, self._kin_key = ArmKinematics(lim, urdf=self.urdf), key
            Planner.warm(self.kin)
        tcp = (sn.get("core") or {}).get("tcp_offset")
        if tcp is not None and list(tcp) != list(self.kin.tcp_offset):
            self.kin.set_tcp_offset(tcp)

    def _watch_control(self, now: float) -> None:
        if self._stop.is_set():
            return
        if self.proc_fault is not None:
            hb_age = now - self.shm.beats[CTL_HB]
            if (self.proc.is_alive() and now - self._fault_t > self.KILL_AFTER_S and hb_age > self.ctl_heartbeat
                    and now > self.shm.beats[CTL_BUSY]):
                log.error("the arm's control process is still silent: killing it")
                self.proc.kill()
            return
        if not self.proc.is_alive():
            self._proc_failed(f"the control process exited (code {self.proc.exitcode})")
            return
        age = now - self.shm.beats[CTL_HB]
        if age > self.ctl_heartbeat and now > self.shm.beats[CTL_BUSY]:
            self._proc_failed(f"no control process heartbeat for {age:.2f} s")

    def _proc_failed(self, reason: str) -> None:
        """The control process is lost: a fault, latched until the node restarts."""
        with self._wlock:
            if self.proc_fault is not None:
                return
            self.proc_fault, self._fault_t = reason, time.monotonic()
            waits = list(self._waits.values())
        down = (False, ("request", 503, f"the arm's control process is down ({reason}): restart the node", {}))
        for w in waits:
            w[1] = down
            w[0].set()
        log.error("control process: %s", reason)
        self.events.add(f"FAULT: {reason}: the motors go to damping (their own CAN watchdog; a stalled control process "
                        "is told to switch them off, then killed); restart the node", "error")
        self.port.send(("off", None, f"the server process lost this process ({reason})"))

    # -- the lease (held here) -----------------------------------------------------------------------------

    def _write_srv(self) -> None:
        with self._srv_lock:
            info, dg = self.lease.info, self._lease_digest
            lease = None
            if info is not None and dg is not None and dg[0] == info.lease_id:
                lease = {"lease_id": info.lease_id, "client": info.client, "acquired": info.acquired,
                         "digest": dg[1]}
            self.shm.srv.write({"lease": lease})

    def acquire(self, client: str):
        token, info = self.lease.acquire(client)
        self._lease_digest = (info.lease_id, token_digest(token))
        self._write_srv()
        self.events.add(f"lease {info.lease_id} acquired by {info.client}")
        return token, info

    def renew(self, token):
        return self.lease.renew(token)

    def release(self, token) -> None:
        self.lease.release(token)

    def _lease_ended(self, reason: str) -> None:
        if self.shm is not None:
            self._write_srv()                     # the joint channel refuses a hello with the old token at once
        self._send(("lease_ended", None, reason))

    # -- commands ------------------------------------------------------------------------------------------

    def _live(self) -> None:
        if self.proc_fault is not None:
            raise RequestError(503, f"the arm's control process is down ({self.proc_fault}): restart the node")
        if self.port is None:
            raise RequestError(503, "the arm's control process is not running")

    def _send(self, msg) -> None:
        if self.port is not None and self.proc_fault is None:
            self.port.send(msg)

    def _call(self, kind: str, *args, timeout: Optional[float] = None, timeout_msg: Optional[str] = None):
        self._live()
        with self._wlock:
            self._rid += 1
            rid = self._rid
            w = self._waits[rid] = [threading.Event(), None]
        timeout = self.CALL_S if timeout is None else timeout
        msg = (kind, rid, *args)
        data = pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL)
        bulk = len(data) >= BULK_MIN
        try:
            if bulk:
                self._bulk_lock.acquire()
                self.shm.bulk.write(msg)
                data = pickle.dumps(("bulk", rid), protocol=pickle.HIGHEST_PROTOCOL)
            if not self.port.send_bytes(data):
                raise RequestError(503, f"the arm's control process's command channel refused {kind}")
            if not w[0].wait(timeout):
                raise RequestError(503, timeout_msg or f"the arm's control process did not answer {kind} within "
                                                       f"{timeout:g} s")
        finally:
            self._waits.pop(rid, None)
            if bulk:
                self._bulk_lock.release()
        ok, payload = w[1]
        if ok:
            return payload
        if payload[0] == "value":
            raise ValueError(payload[1])
        cls = LeaseError if payload[0] == "lease" else RequestError
        raise cls(payload[1], payload[2], payload[3])

    def _fwd(self, name: str, token, *args, **kwargs):
        """An ``ArmNode`` request that takes the lease token: the lease checked (and renewed) here first."""
        self.lease.renew(token)
        return self._call("call", name, (SERVER_CHECKED,) + args, kwargs)

    # ArmRequests' hooks (requests.py) --------------------------------------------------------------------

    def _ready(self, token) -> None:
        self.lease.renew(token)
        if self.estop.active:
            raise RequestError(409, "e-stop is latched")
        self._live()
        if not self.enabled or not self.motors_on:
            raise RequestError(409, f"not enabled ({self.state}; POST /enable)")

    def _new_command(self) -> None:
        self._send(("bump", None))

    def _arm_start_from(self, sn: dict) -> np.ndarray:
        r = sn.get("reading")
        if r is None:
            raise RequestError(503, "no reading yet")
        self._need_kin()
        why = self._limit_problem(r.q_mot)
        if why:
            raise RequestError(409, why)
        return r.q.copy()

    def _track_context(self):
        sn = self.snap()
        return sn.get("track_seq"), self._arm_start_from(sn)

    def _track_apply(self, q, pose, twist, twist_frame, seq, g_args) -> None:
        self._call("track", q, pose, twist, twist_frame, seq, g_args)

    def _arm_start_now(self) -> np.ndarray:
        return self._arm_start_from(self.snap())

    def _start(self, traj, by: str, q0) -> dict:
        return self._call("start", traj, by, q0)

    def _stowed(self, why: str) -> None:
        self._call("stowed", why)

    def _servo_cmd(self) -> Optional[np.ndarray]:
        q = self.snap().get("q_cmd")
        return None if q is None else q.copy()

    def _bus_names(self) -> Optional[list]:
        return self.snap().get("names")

    # requests --------------------------------------------------------------------------------------------

    def chocks(self) -> list:
        return self.locks.matching("chock", self.lock_names)

    def enable(self, token, timeout: float = ENABLE_TIMEOUT) -> dict:
        """Blocking (call from a worker thread): the control process switches the motors on (the first time after
        connecting, the gripper check first)."""
        self.lease.renew(token)
        ch = self.chocks()
        if ch:
            raise RequestError(423, f"chocked: {describe_locks(ch)}. An admin removes it on the robot: "
                                    "python -m robot_utils.admin.locks remove <id>")
        if self.estop.active:
            raise RequestError(409, "e-stop is latched; clear it first (POST /estop/clear)")
        return self._call("enable", timeout=timeout,
                          timeout_msg="enable did not finish in time (is the control loop running?)")

    def disable(self, token) -> dict:
        return self._fwd("disable", token)

    def float_arm(self, token) -> dict:
        return self._fwd("float_arm", token)

    def trigger_estop(self, reason: str, source: str) -> EStopInfo:
        return self.estop.trigger(reason, source)

    def _estop_triggered(self, info: EStopInfo) -> None:
        """The e-stop to the control process at once (it switches the motors off on receipt, before anything else
        queued); its confirmation is awaited in the background (the caller, maybe the event loop, never waits)."""
        if self.proc_fault is not None or self.port is None:
            return                                # down: the motors are off (their own watchdog) or going there
        with self._wlock:
            self._rid += 1
            rid = self._rid
            w = self._waits[rid] = [threading.Event(), None]
        sent = self.port.send(("estop", rid, info.reason, info.source))

        def confirm():
            try:
                if not sent or not w[0].wait(self.ESTOP_ACK_S) or not w[1][0]:
                    self.events.add("e-stop: the control process did not confirm within "
                                    f"{self.ESTOP_ACK_S * 1e3:.0f} ms", "error")
            finally:
                self._waits.pop(rid, None)
        threading.Thread(target=confirm, name="arm-estop-ack", daemon=True).start()

    def clear_estop(self, token) -> None:
        self.lease.renew(token)
        self.estop.clear()
        if self.proc_fault is None:
            self._fwd("clear_estop", token)

    def reset(self, token) -> None:
        self._fwd("reset", token)

    def cancel(self, token) -> dict:
        return self._fwd("cancel", token)

    def gripper_cmd(self, token, gap_mm: Optional[float] = None, max_force_n: Optional[float] = None) -> dict:
        return self._fwd("gripper_cmd", token, gap_mm, max_force_n)

    def set_profile(self, token, name: str) -> dict:
        return self._fwd("set_profile", token, name)

    def set_tcp(self, token, offset) -> dict:
        out = self._fwd("set_tcp", token, offset)
        if self.kin is not None:
            self.kin.set_tcp_offset(offset)           # this side's model at once (the snapshot confirms it)
        return out

    def set_limits(self, token, **kw) -> dict:
        return self._fwd("set_limits", token, **kw)

    def set_jam(self, changes: dict) -> dict:
        return self._call("call", "set_jam", (changes,), {})

    # -- state ---------------------------------------------------------------------------------------------

    def snap(self) -> dict:
        return (self.shm.snap.read() if self.shm is not None else None) or {}

    def _core(self) -> dict:
        return self.snap().get("core") or {}

    @property
    def reading(self):
        return self.snap().get("reading")

    @property
    def soft(self) -> C.Limits:
        return self.snap().get("soft") or C.DEFAULT_SOFT

    @property
    def _limited(self) -> bool:
        return bool(self.snap().get("limited"))

    @property
    def profile(self) -> C.Profile:
        name = self._core().get("profile", self.profile0)
        p = self._profiles.get(name)
        if p is None:
            p = self._profiles[name] = C.load_profile(name)
        return p

    @property
    def state(self) -> str:
        return "fault" if self.proc_fault is not None else self._core().get("state", "connecting")

    status = state

    @property
    def error(self) -> Optional[str]:
        return self.proc_fault if self.proc_fault is not None else self._core().get("error")

    def _flag(self, k: str) -> bool:
        return self.proc_fault is None and bool(self._core().get(k))

    enabled = property(lambda self: self._flag("enabled"))
    motors_on = property(lambda self: self._flag("motors_on"))
    servo = property(lambda self: self._flag("servo"))
    initialized = property(lambda self: bool(self._core().get("initialized")))

    def gripper_state(self) -> dict:
        return self._core().get("gripper") or {}

    def jam_state(self) -> dict:
        return self._core().get("jam") or {}

    def stream_state(self) -> dict:
        return self._core().get("joint_channel") or {}

    def state_dict(self) -> dict:
        now = self.clock()
        sn = self.snap()
        core = dict(sn.get("core") or {})
        if not core:
            raise RequestError(503, "the arm's control process has not published its state yet")
        tr = core.get("track")
        if tr is not None:                            # its age now, not at the snapshot
            core["track"] = {**tr, "age": round(tr["age"] + now - sn["t"], 3)}
        if self.proc_fault is not None:
            core.update(state="fault", error=self.proc_fault, enabled=False, motors_on=False, servo=False,
                        gripper_engaged=False)
        shm = self.shm
        proc = {"mode": "process", "pid": self.ready.get("pid"),
                "alive": bool(self.proc is not None and self.proc.is_alive()), "fault": self.proc_fault,
                "heartbeat_age": None if shm is None else round(time.monotonic() - shm.beats[CTL_HB], 4),
                "snapshot_age": round(now - sn["t"], 4), "busy": self.busy,
                "rows_dropped": None if shm is None else shm.rows.dropped + shm.jac.dropped,
                "commands_dropped": None if self.port is None else self.port.dropped,
                "service_max_ms": round(self._stats["service_max_ms"], 3)}
        return compose_state(core, now, self.settings, self.lease.info, self.estop.info, self.chocks(), self.hub,
                             self.events, proc)
