"""The arm's control process (docs/arm.md "Processes"): ONE thread, the bus and the 100 Hz control loop, alone with
its GIL.

``control_main`` is the spawned process's entry. It runs a ``ControlCore``: the very same ``ArmNode`` (connect, the
tick, the gripper check at enable, servo / track / stream / hold, the gripper loop, jam detection, faults, the
internal joint channel) with its surroundings swapped for the inter-process ones (common/ipc.py):

- the bus (``RebotArm`` and its serial port, or the simulation): built and owned here, as before by the control
  thread alone;
- the joint channel (``<name>.joint.sock``, the WBC's goals and state packets): owned here, no receiver thread: the
  loop waits for its tick on the command socket and the joint socket together and handles each datagram as it
  arrives (and once more right before the command, for goals that arrived during the bus read);
- the lease: the server process holds it; here a view of it from the server's slot (who holds it and a digest of
  its token, for the joint channel's hello); every joint-channel datagram that renews it is counted (the server
  renews the lease); the server sends ``lease_ended``;
- events: buffered, sent to the server's log after the snapshot they belong to; the tick and Jacobian rows: rings
  the server publishes; /state: a snapshot slot (``ArmNode.core_state`` and the pieces the server's requests need),
  every SNAPSHOT_EVERY ticks and after every command.

Commands from the server (the requests after the server checked the lease, the e-stop, a solved track, a planned
motion, ...) arrive on a datagram socket and are applied between ticks, the e-stop first and at once (the motors go
off on receipt, not at the next tick). Every tick writes a heartbeat; a blocking step (connecting, enabling: the
gripper check, the MIT mode switches) announces how long it may take instead. The server's heartbeat is watched:
missing for SRV_HEARTBEAT -> the arm holds where its command is (planned motion, track and stream dropped; the
gripper keeps its goal) and accepts no new motion (joint-channel goals dropped: ``server``) until it is back. The
server's process gone -> the joint channel's bye, the motors off (damping), the process ends.
"""
from __future__ import annotations

import gc
import hashlib
import logging
import math
import os
import secrets
import select
import signal
import time
from typing import Optional

import numpy as np

from ..common import ipc, realtime
from ..common.lease import LeaseError, LeaseInfo, RequestError
from ..common.locks import LockStore
from .node import DT, ENABLE_TIMEOUT, LEASE_HEARTBEAT, ArmNode
from .telemetry import FIELDS, JAC_FIELDS

log = logging.getLogger("arm")

# Beats cells
SRV_HB = 0          # the server process's heartbeat (its IPC thread, on every tick message)
CTL_HB = 1          # the control process's heartbeat (every tick)
RX_COUNT = 2        # joint-channel datagrams that renewed the lease (the server renews it)
RX_LEASE = 3        # ... of this lease id
CTL_PID = 4
CTL_BUSY = 5        # a blocking step in progress may last until then (monotonic s; 0: none)
N_BEATS = 16

SNAPSHOT_EVERY = 2          # ticks between snapshots (and after every command)
BULK_MIN = 32 * 1024        # bytes: a command this large goes through the bulk slot (a long planned motion: 3000
BULK_BYTES = 4 << 20        # samples are ~340 KB, over Linux's default datagram limit), its datagram only says so
SRV_HEARTBEAT = 0.5         # s without the server's heartbeat -> hold, no new motion
BUSY_CONNECT_S = 30.0       # s a connect attempt may block (the bus's own checks: watchdogs, joint readings)
BUSY_ENABLE_S = ENABLE_TIMEOUT + 2.0   # s an enable may block (the gripper check, the MIT mode switches)
BUSY_CLOSE_S = 5.0          # s closing the bus may block (a reset)
SERVER_CHECKED = "\x00server-checked"  # the token of a command the server checked the lease for (never a real token:
                                       # the server checks every HTTP token against its own lease first)
CALLS = ("disable", "float_arm", "clear_estop", "reset", "cancel", "gripper_cmd", "set_profile", "set_tcp",
         "set_limits", "set_jam")      # ArmNode requests the server forwards as they are
_DEFERRED = object()


def shared(path: str, create: bool) -> ipc.SharedFile:
    return ipc.SharedFile(path, [("beats", "beats", N_BEATS),
                                 ("srv", "slot", 4096),                     # server -> control: the lease
                                 ("bulk", "slot", BULK_BYTES),              # server -> control: a large command
                                 ("snap", "slot", 1 << 19),                 # control -> server: the snapshot
                                 ("rows", "ring", (2048, len(FIELDS))),     # control -> server: tick rows
                                 ("jac", "ring", (2048, len(JAC_FIELDS)))],  # control -> server: Jacobian rows
                          create)


def token_digest(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


class _Events:
    """The control process's event log: buffered until the snapshot they describe is written (``ControlCore.flush``),
    then sent to the server's log."""

    def __init__(self):
        self.pending: list = []

    def add(self, msg: str, level: str = "info") -> None:
        self.pending.append((msg, level))

    def list(self) -> list:
        return []


class _Hub:
    """The rows into the shared rings (the server publishes them)."""
    subscribers = 0

    def __init__(self, core: "ControlCore"):
        self.core = core

    def publish_rows(self, rows, channel=None) -> None:
        ring = self.core.shm.jac if channel == "jacobian" else self.core.shm.rows
        for r in np.atleast_2d(rows):
            ring.write(r)

    def state_changed(self) -> None:
        self.core.flush(snapshot=True)
        self.core.port.send(("sc",))

    def recent(self, n, channel=None):
        return np.empty((0, len(FIELDS)))


class _LeaseView:
    """The lease as the control process sees it: the server holds it and publishes who holds it (and a digest of the
    token) in its slot; commands the server checked carry SERVER_CHECKED; joint-channel datagrams are counted."""
    heartbeat = LEASE_HEARTBEAT

    def __init__(self, core: "ControlCore"):
        self.core = core
        self.rx = 0

    def _slot(self) -> Optional[dict]:
        return (self.core.shm.srv.read() or {}).get("lease")

    @staticmethod
    def _info(d: Optional[dict]) -> Optional[LeaseInfo]:
        return None if d is None else LeaseInfo(d["client"], d["acquired"], math.inf, d["lease_id"])

    @property
    def info(self) -> Optional[LeaseInfo]:
        return self._info(self._slot())

    @property
    def held(self) -> bool:
        return self._slot() is not None

    def check(self, token: Optional[str]) -> LeaseInfo:
        d = self._slot()
        if token != SERVER_CHECKED:
            if not token:
                raise LeaseError(401, "this request needs the lease token (X-Lease-Token header)")
        if d is None:
            raise LeaseError(403, "no lease is held (acquire one first; it may have lapsed)")
        if token != SERVER_CHECKED and not secrets.compare_digest(token_digest(token), d["digest"]):
            raise LeaseError(403, f"wrong lease token (the lease is held by {d['client']!r}, or yours lapsed)")
        return self._info(d)

    renew = check

    def renew_holder(self, lease_id: Optional[int] = None) -> Optional[LeaseInfo]:
        info = self.info
        if info is None or (lease_id is not None and info.lease_id != lease_id):
            return None
        self.rx += 1
        beats = self.core.shm.beats
        beats[RX_LEASE] = float(info.lease_id)
        beats[RX_COUNT] = float(self.rx)
        return info

    def expire_if_due(self) -> bool:
        return False                       # the server expires it (and sends lease_ended)


class _Waiters:
    """An enable in progress: ``_do_enable`` sets it when done; each waiting request gets its reply."""

    def __init__(self, core: "ControlCore", req: dict):
        self.core, self.req = core, req
        self.rids: list = []

    def set(self) -> None:
        core = self.core
        core.flush(snapshot=True)
        ok, msg, status = self.req["result"]
        for rid in self.rids:
            if ok:
                core.port.send(("reply", rid, True, {"enabled": True, "state": core.state,
                                                     "initialized": core.initialized}))
            else:
                core.port.send(("reply", rid, False, ("request", status, f"enable failed: {msg}", {})))
        self.rids = []


class ControlCore(ArmNode):
    """``ArmNode`` in the control process (module docstring)."""
    warm_planner = False                    # the server process plans
    joint_channel_thread = False            # the loop polls the joint socket

    def __init__(self, shm: ipc.SharedFile, port: ipc.Port, bus_factory, *, profile: str, settings, locks_path,
                 lock_names, joint_socket, description_node: str = "description", urdf=None, arm_frame: str = "arm",
                 robot=None):
        super().__init__(bus_factory, profile=profile, settings=settings,
                         locks=LockStore(locks_path) if locks_path else None, description_node=description_node,
                         joint_socket=joint_socket, urdf=urdf, arm_frame=arm_frame, robot=robot)
        self.lock_names = set(lock_names)
        self.shm, self.port = shm, port
        self.hub = _Hub(self)
        self.events = _Events()
        self.lease = _LeaseView(self)
        self._ticks = 0
        self._snap_due = False
        self.snap_max_ms = 0.0
        self.running = True

    # -- the loop ------------------------------------------------------------------------------------------

    def run(self, ppid: int) -> None:
        self.realtime = realtime.make_realtime(realtime.PRIO_BUS, "arm control process")
        self.events.add(realtime.describe(self.realtime) + " (its own process)",
                        "info" if self.realtime["error"] is None else "warn")
        socks = [self.port.sock] + ([self.joint_channel.sock] if self.joint_channel is not None else [])
        nxt = time.monotonic()
        while self.running:
            nxt += DT
            delay = nxt - time.monotonic()
            if delay > 0:
                while delay > 0 and self.running:       # wait for the tick; commands and goals meanwhile at once
                    r, _, _ = select.select(socks, [], [], delay)
                    if r:
                        self._inputs(r)
                    delay = nxt - time.monotonic()
            elif delay < -0.05:
                nxt = time.monotonic()                   # far behind (a blocking step): start afresh
            if not self.running:
                break
            now = time.monotonic()
            self.shm.beats[CTL_HB] = now
            if os.getppid() != ppid:                    # the server process is gone: end (bye, motors off)
                log.error("the server process is gone: the arm's control process ends (motors off)")
                break
            try:
                self.handle_commands()
                self._watch_server(now)
                self.step()
            except Exception as e:  # noqa: BLE001 - never let the loop die silently
                log.exception("control tick")
                self._fault(f"control loop: {type(e).__name__}: {e}")
            self._ticks += 1
            self.flush(snapshot=self._ticks % SNAPSHOT_EVERY == 0)
            self.port.send(("t",))                      # the server publishes the rows and beats now
        self.shutdown()

    def _inputs(self, readable) -> None:
        for s in readable:
            if s is self.port.sock:
                self.handle_commands()
            elif self.joint_channel is not None:
                self.joint_channel.poll()

    def _pre_control(self) -> None:
        if self.joint_channel is not None:              # goals that arrived during the read: this tick's command
            self.joint_channel.poll()

    def shutdown(self) -> None:
        try:
            self.stop_joint_channel()                    # the session's bye
        except Exception as e:  # noqa: BLE001
            log.warning("joint channel: %s", e)
        self._close_bus()                                # motors off (damping), the port released
        self.flush(snapshot=True)

    # -- blocking steps: the heartbeat excused for a while ------------------------------------------------

    def _busy(self, what: Optional[str], seconds: float = 0.0) -> None:
        self.shm.beats[CTL_BUSY] = 0.0 if what is None else time.monotonic() + seconds
        self.port.send(("busy", what))

    def _connect(self) -> None:
        self._busy("connecting", BUSY_CONNECT_S)
        try:
            super()._connect()
        finally:
            self._busy(None)
        if self.kin is not None:
            self.flush(snapshot=True)                    # the server builds its model from the setpoint limits

    def _do_enable(self, req: dict) -> None:
        self._busy("enabling", BUSY_ENABLE_S)
        try:
            super()._do_enable(req)
        finally:
            self._busy(None)

    def _close_bus(self) -> None:
        if self.bus is None:
            return
        self._busy("closing the bus", BUSY_CLOSE_S)
        try:
            super()._close_bus()
        finally:
            self._busy(None)

    # -- the server ----------------------------------------------------------------------------------------

    def _watch_server(self, now: float) -> None:
        age = now - self.shm.beats[SRV_HB]
        if age > SRV_HEARTBEAT:
            if not self._srv_lost:
                self._srv_lost = True
                self._hold(f"the server process is lost (no heartbeat for {age:.2f} s): holding")
                log.error("server process heartbeat lost (%.2f s): holding, no new motion", age)
        elif self._srv_lost:
            self._srv_lost = False
            self.events.add("server process heartbeat back: new motion accepted again", "warn")

    def _hold(self, why: str) -> None:
        """Hold where the command is (servoing) or keep floating: the planned motion, track and stream dropped; the
        gripper keeps what it is doing."""
        with self._lock:
            m = self.motion
            self.track = None
            self.stream = None
            if self.servo and self.q_cmd is not None:
                self.hold_q = self.q_cmd.copy()
            if m is not None:
                m.phase, m.reason = "aborted", why
                self.last_motion, self.motion = m, None
        self.events.add(why, "error")

    # -- the snapshot and the events ----------------------------------------------------------------------

    def flush(self, snapshot: bool = False) -> None:
        """The snapshot (if due), then the events it explains (the server's /state never lags its own events)."""
        ev = self.events.pending
        if snapshot or ev or self._snap_due:
            self.write_snapshot()
        if ev:
            self.events.pending = []
            for msg, level in ev:
                self.port.send(("ev", msg, level))

    def write_snapshot(self) -> None:
        t0 = time.perf_counter()
        now = self.clock()
        try:
            core = self.core_state(now)
            with self._lock:
                q_cmd = self.q_cmd.copy() if (self.servo and self.q_cmd is not None) else None
                prev = self.track if self._arm_engaged() else None
            b = self.bus
            self.shm.snap.write({
                "t": now, "core": core, "reading": self.reading, "q_cmd": q_cmd,
                "track_seq": None if prev is None else prev.seq, "limited": bool(self._limited), "soft": self.soft,
                "names": self._bus_names(), "setpoint_limits": None if b is None else dict(b.setpoint_limits),
                "estop": self.estop.active, "srv_lost": self._srv_lost, "snap_max_ms": round(self.snap_max_ms, 3)})
            self._snap_due = False
        except Exception as e:  # noqa: BLE001 - never into the tick
            log.warning("snapshot: %s", e)
        self.snap_max_ms = max(self.snap_max_ms, (time.perf_counter() - t0) * 1e3)

    # -- commands ------------------------------------------------------------------------------------------

    def handle_commands(self) -> None:
        msgs = self.port.recv_all()
        if not msgs:
            return
        msgs.sort(key=lambda m: m[0] != "estop")        # the e-stop first
        for m in msgs:
            if m[0] == "bulk":                          # the command itself is in the bulk slot
                m = self.shm.bulk.read()
                if m is None:
                    continue
            kind, rid, args = m[0], m[1], m[2:]
            try:
                out = getattr(self, "cmd_" + kind)(rid, *args)
                if out is _DEFERRED:
                    continue                             # answered later (an enable)
                self.flush(snapshot=True)
                if rid is not None:
                    self.port.send(("reply", rid, True, out))
            except RequestError as e:
                self.flush(snapshot=True)
                self.port.send(("reply", rid, False, ("lease" if isinstance(e, LeaseError) else "request", e.status,
                                                      str(e), e.extra)))
            except ValueError as e:
                self.flush(snapshot=True)
                self.port.send(("reply", rid, False, ("value", str(e))))
            except Exception as e:  # noqa: BLE001
                log.exception("command %s", kind)
                self.flush(snapshot=True)
                self.port.send(("reply", rid, False, ("request", 500, f"{type(e).__name__}: {e}", {})))

    def _cut_now(self) -> None:
        """Motors off right away (an e-stop, a stow, a fault from the server), not at the next tick: commands run in
        the bus's own thread here."""
        if self._cut is not None and self.bus is not None:
            why, self._cut = self._cut, None
            self._motors_off(why)

    def cmd_stop(self, rid):
        self.running = False

    def cmd_estop(self, rid, reason: str, source: str):
        self.estop.trigger(reason, source)               # -> _off (disabled, stream session closed)
        self._cut_now()
        return {"motors_on": self.motors_on}

    def cmd_off(self, rid, why: str):
        """The server lost this process (a stall it may come back from): fault, motors off."""
        self._fault(why)

    def cmd_lease_ended(self, rid, reason: str):
        self._lease_ended(reason)

    def cmd_bump(self, rid):
        self._new_command()

    def cmd_enable(self, rid):
        if self._base in ("connecting", "fault") or self.bus is None:
            raise RequestError(503, f"arm not ready ({self.state}: {self.error}); POST /reset after a fault")
        if self.enabled and self.motors_on:
            return {"enabled": True, "state": self.state, "initialized": self.initialized}
        with self._lock:
            req = self._enable_req
            if req is None or req.get("result") is not None:
                req = self._enable_req = {"result": None}
                req["event"] = _Waiters(self, req)
            req["event"].rids.append(rid)
        return _DEFERRED

    def cmd_call(self, rid, name: str, args: tuple, kwargs: dict):
        if name not in CALLS:
            raise RequestError(500, f"not a forwarded request: {name}")
        return getattr(self, name)(*args, **kwargs)

    def cmd_track(self, rid, q, pose, twist, twist_frame, seq, g_args):
        self._track_apply(q, pose, twist, twist_frame, seq, g_args)
        return {"limited": bool(self._limited)}

    def cmd_start(self, rid, traj, by: str, q0):
        return self._start(traj, by, q0)

    def cmd_stowed(self, rid, why: str):
        self._stowed(why)
        self._cut_now()
        return {}


def control_main(params: dict) -> None:
    """The control process's entry (multiprocessing 'spawn')."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)          # the server process stops it (Ctrl-C goes to the group)
    logging.basicConfig(level=params.get("log_level", logging.INFO),
                        format="%(asctime)s %(levelname)s %(name)s[control]: %(message)s", datefmt="%H:%M:%S")
    shm = shared(params["shm"], create=False)
    port = ipc.Port(params["ctl_sock"], params["srv_sock"])
    try:
        core = ControlCore(shm, port, params["bus_factory"], profile=params["profile"], settings=params["settings"],
                           locks_path=params["locks_path"], lock_names=params["lock_names"],
                           joint_socket=params["joint_socket"], description_node=params["description_node"],
                           urdf=params.get("urdf"), arm_frame=params.get("arm_frame", "arm"), robot=params.get("robot"))
        core.start_joint_channel()
    except Exception as e:  # noqa: BLE001
        log.exception("control process start")
        port.send(("ready", False, f"{type(e).__name__}: {e}"))
        port.close()
        return
    shm.beats[CTL_PID] = os.getpid()
    shm.beats[CTL_HB] = time.monotonic()
    core.write_snapshot()
    port.send(("ready", True, {"pid": os.getpid()}))
    gc.collect()
    gc.freeze()                                           # the setup's objects never cost a collection again
    try:
        core.run(params["ppid"])
    finally:
        port.close()
        log.info("arm control process ended")
