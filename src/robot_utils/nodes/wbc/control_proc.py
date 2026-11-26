"""The WBC's control process (docs/wbc.md "Processes"): ONE thread, the 100 Hz control loop, alone with its GIL.

``control_main`` is the spawned process's entry. It builds the model from the URDF the server loaded and runs a
``ControlCore``: the very same ``WbcNode`` tick (the latch, staleness, holds, the QP, integration, the outputs) with
its surroundings swapped for the inter-process ones (ipc.py):

- the target datagrams: the ``Receiver``'s sockets, polled every tick (no thread); armed / arm_seq / the latch are
  applied here as in-process;
- the arms: their joint channels, owned here (hello with each arm's lease token the server took);
- the estimator and swerve rows: rings the server's stream readers fill;
- the base twist: a slot the server's sender posts from; ``mute`` / a lost base are flags / messages;
- the client lease: ``held`` from the server's slot; every accepted datagram counts (the server renews the lease);
- events: messages to the server's log; the stream rows: a ring the server publishes; /state: a snapshot slot.

Commands from the server (enable, disable, e-stop, session, hints, config, disarm, ...) arrive on a datagram socket
and are applied between ticks (the loop waits on that socket, so an e-stop is applied at once, not at the next tick).
Every tick writes a heartbeat; the server's is watched: missing for ``srv_heartbeat`` -> disarm and hold (the arm
channel stays alive). The server's process gone -> the control process ends (the arm channel's bye).
"""
from __future__ import annotations

import gc
import logging
import os
import select
import signal
import time
from typing import Optional

import numpy as np

from ..common import realtime
from ..common.lease import RequestError
from . import ipc
from .arm_channel import ArmChannel, ChannelError
from .node import WbcNode
from .spec import make_config

log = logging.getLogger("wbc")

SNAPSHOT_EVERY = 2          # ticks between /state snapshots (and after every command)


class _Events:
    """The control process's event log: each event goes to the server's log."""

    def __init__(self, port: ipc.Port):
        self.port = port
        self.on_add = None

    def add(self, msg: str, level: str = "info") -> None:
        self.port.send(("ev", msg, level))

    def list(self) -> list:
        return []


class _Hub:
    """The stream rows into the shared ring (the server publishes them)."""

    def __init__(self, ring: ipc.Ring):
        self.ring = ring
        self.last = None

    def publish_rows(self, rows, channel=None) -> None:
        for r in np.atleast_2d(rows):
            self.ring.write(r)
            self.last = r

    def state_changed(self) -> None:
        pass

    def recent(self, n, channel=None):
        return np.empty((0, 0)) if self.last is None else np.atleast_2d(self.last)


class _Lease:
    """The client lease as the control process sees it: held (the server's slot); datagrams counted (renewals)."""

    def __init__(self, core: "ControlCore", heartbeat: float):
        self.core = core
        self.heartbeat = heartbeat
        self.rx = 0

    @property
    def held(self) -> bool:
        return bool(self.core.srv().get("lease_held"))

    @property
    def info(self):
        return None

    def renew(self, token) -> None:
        self.rx += 1
        self.core.shm.beats[ipc.RX_COUNT] = float(self.rx)

    def expire_if_due(self) -> bool:
        return False                       # the server expires it (and sends lease_ended)

    def check(self, token) -> None:
        pass                               # the server checks tokens


class _EStop:
    def __init__(self):
        self.active = False
        self.info = None


class _Base:
    """The swerve base as the control process sees it: the twist goes to a shared slot the server's sender posts
    from; ``mute`` is sticky; the sender's health comes back in the server's slot."""

    def __init__(self, core: "ControlCore", gen: int, lease_id):
        self.core, self.gen, self.lease_id = core, gen, lease_id
        self.seq = 0
        self.muted = False
        self.hard: dict = {}
        self.sent = self.errors = 0

    def _write(self, twist) -> None:
        self.seq += 1
        self.core.shm.twist.write({"gen": self.gen, "seq": self.seq, "twist": tuple(float(v) for v in twist),
                                   "muted": self.muted})

    def send(self, twist) -> None:
        if not self.muted:
            self._write(twist)

    def mute(self) -> None:
        if not self.muted:
            self.muted = True
            self._write((0.0, 0.0, 0.0))
            self.core.port.send(("base_mute", self.gen))

    def failing_for(self) -> float:
        s = self.core.srv()
        f = s.get("fail_since") if s.get("gen") == self.gen else None
        return 0.0 if f is None else time.monotonic() - f

    @property
    def last_error(self) -> Optional[str]:
        return self.core.srv().get("last_error")

    def close(self, disable: bool = True) -> None:
        self.mute()
        self.core.port.send(("base_close", self.gen))


class ControlCore(WbcNode):
    """``WbcNode``'s tick in the control process (module docstring)."""

    def __init__(self, model, shm: ipc.Shared, port: ipc.Port, *, arm, swerve, estimator, config, udp_host,
                 udp_port, unix_socket, arm_socket, name, arms=None, arm_sockets=None):
        super().__init__(model, arm=arm, swerve=swerve, estimator=estimator, config=config, udp_host=udp_host,
                         udp_port=udp_port, unix_socket=unix_socket, arm_socket=arm_socket, name=name, arms=arms,
                         arm_sockets=arm_sockets)
        self.shm, self.port = shm, port
        self.events = _Events(port)
        self.hub = _Hub(shm.rows)
        self.lease = _Lease(self, self.cfg.lease_heartbeat)
        self.estop = _EStop()
        self._pending: Optional[dict] = None       # an enable in progress (the joint channel's welcome, a state)
        self._srv_lost = False
        self._ticks = 0
        self.running = True

    def srv(self) -> dict:
        return self.shm.srv.read() or {}

    # -- the surroundings ----------------------------------------------------------------------------------

    def _drain_inputs(self, now: float) -> None:
        for r in self.shm.est.read():
            self.feed_estimator(dict(zip(ipc.EST_COLS, (float(v) for v in r))), now)
        sw = self.shm.sw.read()
        if len(sw):
            t, en, es, lid = (float(v) for v in sw[-1])
            self.sw = {"t": t, "enabled": en, "estop": es, "lease_id": lid, "rx": now}

    def _forward_estop(self, reason: str, *, base: bool, arm: bool, skip=None) -> None:
        self.port.send(("fwd_estop", reason, base, arm, skip))

    def _live(self) -> bool:
        return super()._live() and not self._srv_lost

    # -- the loop ------------------------------------------------------------------------------------------

    def run(self, ppid: int) -> None:
        self.realtime = realtime.make_realtime(realtime.PRIO_CONTROL, "wbc control process")
        self.events.add(realtime.describe(self.realtime) + " (its own process)",
                        "info" if self.realtime["error"] is None else "warn")
        nxt = time.monotonic()
        while self.running:
            nxt += self.cfg.dt
            delay = nxt - time.monotonic()
            if delay > 0:
                while delay > 0 and self.running:       # wait for the tick; commands meanwhile are applied at once
                    r, _, _ = select.select([self.port.sock], [], [], delay)
                    if r:
                        self.handle_commands()
                    delay = nxt - time.monotonic()
            else:
                nxt = time.monotonic()
            if not self.running:
                break
            self.shm.beats[ipc.CTL_HB] = time.monotonic()
            if os.getppid() != ppid:                    # the server process is gone: end (the arm channel's bye)
                log.error("the server process is gone: the control process ends")
                break
            try:
                self.handle_commands()
                self.receiver.poll()
                self._progress_enable()
                self._watch_server()
                self.step()
            except Exception as e:  # noqa: BLE001 - never let the loop die
                log.exception("control tick")
                self.events.add(f"control error: {type(e).__name__}: {e}", "error")
                with self.lock:
                    if self.enabled:
                        self._fault_locked(f"control error: {e}", True, True)
            self._ticks += 1
            if self._ticks % SNAPSHOT_EVERY == 0:
                self.write_snapshot()
            self.port.send(("t",))                      # the server posts the twist and publishes the row now
        self.shutdown()

    def write_snapshot(self) -> None:
        now = self.clock()
        with self.lock:
            sn = self._snapshot_locked(now)
        try:
            self.shm.snap.write(sn)
        except Exception as e:  # noqa: BLE001
            log.warning("snapshot: %s", e)

    def _watch_server(self) -> None:
        age = time.monotonic() - self.shm.beats[ipc.SRV_HB]
        if age > self.cfg.srv_heartbeat:
            if not self._srv_lost:
                self._srv_lost = True
                with self.lock:
                    self._disarm_locked(f"server process heartbeat lost ({age:.2f} s)")
                log.error("server process heartbeat lost (%.2f s): disarmed, holding", age)
        elif self._srv_lost:
            self._srv_lost = False
            self.events.add("server process heartbeat back (still disarmed: a new press arms)", "warn")

    def shutdown(self) -> None:
        with self.lock:
            r = self._disable_locked("node shutting down") if self.enabled else None
        if r is not None:
            for channel in r[1].values():
                channel.close()
        for channel in ((self._pending or {}).get("channels") or {}).values():
            channel.close()
        self.receiver.close()

    # -- commands ------------------------------------------------------------------------------------------

    def handle_commands(self) -> None:
        msgs = self.port.recv_all()
        if not msgs:
            return
        msgs.sort(key=lambda m: m[0] != "estop")        # the e-stop first
        for m in msgs:
            kind, rid, args = m[0], m[1], list(m[2:])
            if kind == "enable":
                args[0]["rid"] = rid
            try:
                out = getattr(self, "cmd_" + kind)(*args)
                if kind == "enable" and out is None:
                    continue                             # answered when the joint channel is up
                self.write_snapshot()
                if rid is not None:
                    self.port.send(("reply", rid, True, out))
            except RequestError as e:
                self.write_snapshot()
                self.port.send(("reply", rid, False, (e.status, str(e))))
            except ValueError as e:
                self.write_snapshot()
                self.port.send(("reply", rid, False, (422, str(e))))
            except Exception as e:  # noqa: BLE001
                log.exception("command %s", kind)
                self.port.send(("reply", rid, False, (500, f"{type(e).__name__}: {e}")))

    def cmd_stop(self):
        self.running = False

    def cmd_estop(self, reason: str):
        self.estop.active = True
        with self.lock:
            self._estop_stop_locked(reason)
        return {"held": True}

    def cmd_estop_clear(self):
        self.estop.active = False
        return {}

    def cmd_disarm(self, why: str):
        with self.lock:
            was = self.armed
            self._disarm_locked(why)
            return {"armed": False, "was_armed": was, "disarm_reason": self.disarm_reason}

    def cmd_session(self, token):
        with self.lock:
            sid, floor = self._open_session_locked(token)
        return {"session": sid, "arm_seq": floor}

    def cmd_lease_ended(self, reason: str):
        self.receiver.close_session()
        self.session_token = None
        with self.lock:
            self._disarm_locked(f"client lease {reason}")

    def cmd_hints(self, h):
        with self.lock:
            self._set_hints_locked(h)
            from .node import hints_doc
            return hints_doc(self.hints, self.effective_hints())

    def cmd_config(self, changes: dict):
        with self.lock:
            self._caps_check_locked(changes)                # the base started moving meanwhile
            new = make_config(changes, base=self.cfg)
            self._apply_config_locked(changes, new)
        return {"cfg": new}

    def cmd_disable(self, reason: str):
        if self._pending is not None:
            self._abort_enable(RequestError(409, f"disabled while enabling: {reason}"))
        with self.lock:
            r = self._disable_locked(reason)
        if r is None:
            return {"enabled": False, "was_enabled": False}
        was, channels, base, _ = r
        for channel in channels.values():
            channel.close()
        return {"enabled": False, "was_enabled": was}

    def cmd_enable(self, d: dict):
        """The server took the leases and enabled the peers; here, per arm (``d["arms"]``: EE -> its lease token,
        lease id, TCP, gains): the TCP and gains, the joint channel (the welcome and a first state are awaited over
        the next ticks, ``_progress_enable``), the seed."""
        with self.lock:
            if self.enabled or self.state == "enabling":
                raise RequestError(409, f"already {self.state}; disable first")
            self.state = "enabling"
        channels: dict = {}
        try:
            for e, a in (d.get("arms") or {}).items():
                if a.get("tcp") is not None:
                    self.model.set_tcp(e, a["tcp"])
                if a.get("kp") is not None:
                    self.ctl.kp[e], self.ctl.kp_source = np.asarray(a["kp"], float).copy(), a["kp_source"]
            base = None if d.get("base_lease_id") is None else _Base(self, d["gen"], d["base_lease_id"])
            p = {"rid": d["rid"], "d": d, "base": base, "channels": channels, "welcome": {}, "st": {},
                 "deadline": {}}
            for e, a in (d.get("arms") or {}).items():
                if a.get("token") is None:
                    continue
                ch = ArmChannel(self.arm_sockets.get(e))
                try:
                    ch.begin_open(a["token"])
                except ChannelError as ex:
                    raise RequestError(503, f"{self._label(e)} joint channel: {ex}")
                channels[e] = ch
                p["deadline"][e] = time.monotonic() + 1.0
            self._pending = p
        except BaseException:
            for ch in channels.values():
                ch.close()
            with self.lock:
                self.state = "disabled"
            raise
        self._progress_enable()
        return None

    def _abort_enable(self, err: RequestError) -> None:
        p, self._pending = self._pending, None
        for ch in p["channels"].values():
            ch.close()
        with self.lock:
            self.state = "disabled"
        self.write_snapshot()
        self.port.send(("reply", p["rid"], False, (err.status, str(err))))

    def _progress_enable(self) -> None:
        p = self._pending
        if p is None:
            return
        now = time.monotonic()
        waiting = False
        lab = "arm"
        try:
            for e, ch in list(p["channels"].items()):          # each arm: its welcome, then a first state
                lab = self._label(e)
                if not p["welcome"].get(e):
                    if ch.poll_open() is None:
                        if now > p["deadline"][e]:
                            ch.close(send_bye=False)
                            del p["channels"][e]
                            raise RequestError(503, f"{lab} joint channel: no welcome from the {lab} joint channel "
                                                    "within 1 s")
                        waiting = True
                        continue
                    p["welcome"][e], p["deadline"][e] = True, now + 1.0
                st = ch.poll(self.clock())
                if st is not None and np.all(np.isfinite(st.q)):
                    p["st"][e] = st
                if p["st"].get(e) is None:
                    if now > p["deadline"][e]:
                        raise RequestError(503, f"no state from the {lab} joint channel within 1 s")
                    waiting = True
            if waiting:
                return
            for e, ch in p["channels"].items():
                self._apply_arm_limits(ch.limits, e)
        except ChannelError as ex:
            p["channels"].pop(e, None)
            self._abort_enable(RequestError(503, f"{lab} joint channel: {ex}"))
            return
        except RequestError as ex:
            self._abort_enable(ex)
            return
        self._pending = None
        d = p["d"]
        self.receiver.poll()               # datagrams sent before the enable count as seen, never arm (as in-process)
        with self.lock:
            self._enable_finish_locked({}, {e: a.get("lease") for e, a in (d.get("arms") or {}).items()},
                                       p["channels"], p["base"], p["st"], d["holder"])
        self._tick_prev = None             # the enable's ticks were not late
        self.write_snapshot()
        limits = {e: dict(ch.limits) for e, ch in p["channels"].items()}
        self.port.send(("reply", p["rid"], True, {"status": self.status(),
                                                  "limits": limits.get(self.primary), "arms_limits": limits}))


def control_main(params: dict) -> None:
    """The control process's entry (multiprocessing 'spawn')."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)          # the server process stops it (Ctrl-C goes to the group)
    logging.basicConfig(level=params.get("log_level", logging.INFO),
                        format="%(asctime)s %(levelname)s %(name)s[control]: %(message)s", datefmt="%H:%M:%S")
    shm = ipc.Shared(params["shm"], params["width"], create=False)
    port = ipc.Port(params["ctl_sock"], params["srv_sock"])
    try:
        from .model import WbcModel
        model = WbcModel(params["urdf"], params["ees"], params["margin"])
        core = ControlCore(model, shm, port, arm=params["arm"], swerve=params["swerve"],
                           estimator=params["estimator"], config=params["config"], udp_host=params["udp_host"],
                           udp_port=params["udp_port"], unix_socket=params["unix_socket"],
                           arm_socket=params["arm_socket"], name=params["name"], arms=params.get("arms"),
                           arm_sockets=params.get("arm_sockets"))
        if core.fields != params["fields"]:
            raise RuntimeError("the control process's stream fields differ from the server's")
    except Exception as e:  # noqa: BLE001
        log.exception("control process start")
        port.send(("ready", False, f"{type(e).__name__}: {e}"))
        port.close()
        return
    shm.beats[ipc.CTL_PID] = os.getpid()
    shm.beats[ipc.CTL_HB] = time.monotonic()
    core.write_snapshot()
    port.send(("ready", True, {"udp_port": core.receiver.udp_port, "unix_socket": core.receiver.unix_path,
                               "pid": os.getpid()}))
    gc.collect()
    gc.freeze()                                           # the setup's objects never cost a collection again
    try:
        core.run(params["ppid"])
    finally:
        port.close()
        log.info("control process ended")

