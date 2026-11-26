"""The Quest teleop node's core: the teleop client loop (tools/quest_teleop, through ``client.py``) against the local
WBC, the headset link monitor, the WBC echo, and the headset control (quest_ctl up / down over this machine's ssh).
No web code here (server.py). On disk: only the event log (``eventlog.py``, ``--data-dir``); log lines go to the
journal. Episodes (``episodes.py``): A / B on the right controller; every event also goes to the operator stream
(``oplog.py``) with each headset frame and each datagram sent.

Threads:
- ``quest-teleop`` (the loop): while teleop runs, ``TeleopClient.step()`` (headset frames in, one datagram per batch
  out); otherwise it drains the headset frames for the link monitor. Publishes a row every ``ROW_PERIOD``.
- ``wbc-poll``: ``GET /state`` of the WBC once a second (e-stop, lease, enabled, fault): stops teleop when the WBC is
  disabled, faults, e-stops or the lease is no longer ours.
- the reader threads of ``QuestReader`` and ``WbcStream`` (the client's own), and one thread per headset command.
"""
from __future__ import annotations

import dataclasses
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
from .client import (DEFAULT_PORT, MAX_LATENCY_S, MIN_RATE_HZ, STALE_S, Config, EpisodeButtons, LocalWbcClient,
                     QuestReader, TeleopClient, WbcStream, assert_caps, close_wbc, open_wbc, quest_ctl,
                     MIRROR_MODES, TRANSIENT_CLASSES, ResumeConfig, input_class, set_top_speed, top_speed_caps, x_pressed)
from .episodes import RESET_TIMEOUT, Episodes
from .haptics import Haptics, NodeSources
from .sim_aids import AidsConfig, SimAids
from .eventlog import CAP_BYTES, TeleopEventLog
from .oplog import OpLog
from .recording import RecordConfig, SimRecorder

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
    # appended (append-only from here)
    ("armed", "bool", "1 while the client is armed (Y); 0 disarmed (X, stale input, ...: teleop state reason)"),
    ("wbc_armed", "bool", "the WBC's own armed flag (its stream)"),
    ("quest_latency", "s", "the newest headset frame's latency over the session's best (arrival - sample time)"),
    ("quest_rate_short", "Hz", "valid headset frames over the last 0.5 s (below 30 Hz: stale, disarms)"),
    ("battery_pct", "%", "the headset battery (its telemetry's device block; NaN when not reported)"),
    ("tier", "enum", "the client's recovery tier: 0 disarmed (Y arms), 1 hold (a transient: release and re-grip "
                     "resumes), 2 armed"),
]
TIERS = ["disarmed", "hold", "armed"]
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
        self.body = None                             # the newest parsed body block (quest_teleop body.py)
        self.body_t: Optional[float] = None          # its arrival (our clock)
        self.body_frames = 0
        self.on_frame: Optional[Callable] = None     # (t_rx, frame) for every frame (the operator stream)
        self._lock = threading.Lock()

    def _seen(self, f) -> None:
        if f is None:
            return
        with self._lock:
            self.arrivals.append(f.arrived)
            self.last, self.last_t = f.data, f.arrived
            self.frames += 1
            b = getattr(f, "body", None)
            if b is not None:
                self.body, self.body_t = b, f.arrived
                self.body_frames += 1
        if self.on_frame is not None:
            self.on_frame(f.arrived, f.data)

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

    def battery(self) -> dict:
        """The headset's ``device`` block (newer telemetry builds): battery %, charging / plugged, temperature, and a
        line formatted as quest_ctl's (``QuestCtl.battery``); "not reported" when the block is absent."""
        d = self.last or {}
        dev = d.get("device") if isinstance(d.get("device"), dict) else None
        pct = dev.get("battery_pct") if dev else None
        if not isinstance(pct, (int, float)) or isinstance(pct, bool):
            return {"pct": None, "charging": None, "plugged": None, "temperature_c": None, "text": "not reported"}
        from types import SimpleNamespace
        return {"pct": float(pct), "charging": bool(dev.get("charging")), "plugged": bool(dev.get("plugged")),
                "temperature_c": dev.get("temperature_c") if isinstance(dev.get("temperature_c"), (int, float))
                else None, "text": quest_ctl.QuestCtl.battery(SimpleNamespace(device=dev, live=True))}

    def body_info(self, now: float) -> dict:
        """The body tracking block (quest_env BODY_TRACKING.md), summarised: ``present`` (a valid block arrived in
        the last second), the newest block's availability / fidelity / joint counts, its ``age`` on our clock,
        and the reader's count of malformed blocks. Information only: nothing uses the body yet."""
        with self._lock:
            b, t, n = self.body, self.body_t, self.body_frames
        counts = getattr(self.reader, "counts", {}) or {}
        out = {"present": b is not None and t is not None and now - t <= 1.0,
               "age": None if t is None else round(now - t, 4), "frames": n,
               "bad": counts.get("body_bad", 0), "last_error": getattr(self.reader, "last_body_error", None)}
        if b is not None:
            out.update(b.summary())
        return out

    def snapshot(self, now: float) -> dict:
        with self._lock:
            n = sum(1 for t in self.arrivals if now - t <= 1.0)
            d, t = self.last or {}, self.last_t
        intent = d.get("control_intent") if isinstance(d.get("control_intent"), dict) else {}
        r = self.reader
        return {"connected": bool(getattr(r, "connected", False)), "rate_hz": float(n),
                "battery": self.battery(),
                "age": None if t is None else round(now - t, 4), "frames": self.frames,
                "focused": d.get("input_focused") if d else None,
                "xr_state": d.get("xr_session_state") if d else None,
                "session_id": d.get("session_id") if d else None,
                "scheme": intent.get("scheme"),
                "connects": getattr(r, "counts", {}).get("connects"),
                "last_error": getattr(r, "last_error", None),
                "hands": self.hands() if d else None,
                "body": self.body_info(now)}


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
    def __init__(self, wbc: str = "wbc", *, arm: Optional[str] = None, swerve: Optional[str] = None,
                 quest_host: Optional[str] = None, quest_port: int = DEFAULT_PORT,
                 ssh_host: str = "quest", cfg: Optional[Config] = None, stale_s: float = STALE_S,
                 max_latency: float = MAX_LATENCY_S, min_rate: float = MIN_RATE_HZ,
                 ar: str = ":80", scheme: str = "body-frozen", body: str = "off", ctl_timeout: float = 240.0,
                 wait: float = 30.0,
                 runner_factory: Optional[Callable] = None, probe: Optional[Callable] = None,
                 wbc_client_factory: Callable = LocalWbcClient, name: str = "quest_teleop",
                 clock: Callable[[], float] = time.monotonic, data_dir: Optional[str] = None,
                 log_cap_bytes: int = CAP_BYTES, sim: Optional[str] = None, reset_timeout: float = RESET_TIMEOUT,
                 aids: Optional[AidsConfig] = None, autostart: bool = False, estimator: Optional[str] = "estimator",
                 record: Optional[RecordConfig] = None,
                 recorder_factory: Callable = SimRecorder, resume: Optional[ResumeConfig] = None):
        """``data_dir``: the event log's directory (``__main__``: ``~/.local/share/robot_utils/quest_teleop``; None
        keeps it in memory only); ``sim``: the sim node (instance or URL) when the stack is a sim: A resets it;
        ``record``: with ``sim``, record every teleop run (recording.py; None: off)."""
        self.name = name
        self.wbc_target = wbc
        # the e-stop goes to these directly too (``arm``: one arm node, or several, comma-separated / a list)
        arms = [a for a in (arm.split(",") if isinstance(arm, str) else (arm or [])) if a]
        self.arm_targets = arms
        self.arm_target = ",".join(arms) or None
        self.swerve_target = swerve
        self.ssh_host = ssh_host
        self.quest_host = quest_host or quest_ctl.ssh_hostname(ssh_host)   # `ssh -G`: local, no network
        self.quest_port = quest_port
        self.cfg = cfg or Config()
        # the tiered recovery (arming.py): shared with each client, so a change applies at once
        self.resume = resume if resume is not None else ResumeConfig()
        self.stale_s = stale_s
        self.max_latency, self.min_rate = max_latency, min_rate
        self.ar = ar
        self.scheme = scheme
        self.body = body            # quest up: Meta body tracking off / core / all (the hint sources, the elbow hint)
        self.ctl_timeout, self.wait = ctl_timeout, wait
        self.runner_factory = runner_factory or (lambda host: quest_ctl.SshRunner(host))
        self.probe = probe or quest_ctl.probe_stream
        self.wbc_client_factory = wbc_client_factory
        self.clock = clock
        self.hub = RowHub({"tick": len(FIELDS)})
        self.events = EventLog(50, on_add=self.hub.state_changed)
        self.reader = QuestReader(self.quest_host, quest_port)
        self.monitor = HeadsetMonitor(self.reader, clock)
        self.oplog = OpLog()
        self.haptics = Haptics(NodeSources(self), clock=clock)    # controller vibration (haptics.py)
        self.monitor.on_frame = self._on_headset_frame
        self.data_dir = data_dir
        self.eventlog = TeleopEventLog(data_dir, log_cap_bytes, clock=clock)
        self.eventlog.listeners.append(self.oplog.event)
        self.eventlog.listeners.append(lambda line: self.hub.state_changed())
        self.eventlog.listeners.append(self.haptics.on_event)
        self.episodes = Episodes(self, sim, reset_timeout, estimator=estimator)
        self.autostart = bool(autostart and sim)     # the sim: start teleop by itself once the WBC answers
        self.sim = sim
        self.aids = SimAids(self, sim, aids) if sim else None   # the goal ghost, auto pause (sim_aids.py)
        self.recorder = recorder_factory(self, sim, record) if sim and record is not None else None
        self.buttons: Optional[EpisodeButtons] = None
        self.wbc_enabled: Optional[bool] = None   # as this node knows it (its own requests, the 1 Hz poll)
        self._known_lock = threading.Lock()
        self._fresh_prev: Optional[bool] = None
        self._connected_prev: Optional[bool] = None
        self._fault_known = False
        self._estop_known = False
        self._poll_prev: dict = {}
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
        if self.aids is not None:
            self.aids.start()
        if self.autostart:
            t = threading.Thread(target=self._autostart, name="quest-autostart", daemon=True)
            t.start()
            self._threads.append(t)
        self.events.add(f"headset stream {self.quest_host}:{self.quest_port}, ssh host {self.ssh_host!r}, "
                        f"WBC {self.wbc_target}")

    def _autostart(self) -> None:
        """--autostart (sim only): Start as soon as the WBC accepts it, retrying every 2 s; once, at node start (a
        later Stop is the operator's)."""
        while not self._stop.wait(2.0):
            if self.phase != "stopped":
                return
            try:
                self.start_teleop()
                self.events.add("teleop started automatically (--autostart)")
                return
            except Exception as e:  # noqa: BLE001 (the WBC or the sim not up yet: try again)
                log.info("autostart: %s", getattr(e, "detail", None) or e)

    def close(self) -> None:
        self.stop_teleop("node shutting down")
        if self.recorder is not None:
            self.recorder.close()           # its files closed before the node goes
        if self.aids is not None:
            self.aids.close()
        self._stop.set()
        for t in self._threads:
            t.join(2)
        self.haptics.close()
        self.reader.close()
        self.stream.close()
        self.eventlog.close()               # everything queued is written first

    # -- events ----------------------------------------------------------------------------------------------

    _NO_EPISODE = object()

    def emit(self, kind: str, detail: Optional[dict] = None, episode=_NO_EPISODE) -> dict:
        """One event (the contract's line) to the log and the operator stream; ``episode`` defaults to the open
        one."""
        if episode is self._NO_EPISODE:
            episode = self.episodes.episode_id
        return self.eventlog.emit(kind, detail, episode)

    def set_enabled(self, enabled: bool, reason: str) -> None:
        """The WBC became enabled / disabled (our request succeeded, or the poll saw it): one event per change; a
        disable aborts the open episode."""
        with self._known_lock:
            if self.wbc_enabled is enabled:
                return
            self.wbc_enabled = enabled
        if enabled:
            self.emit("enabled", {"by": reason})
        else:
            self.emit("disabled", {"reason": reason})
            self.episodes.abort("disabled", reason)

    def wbc_disabled(self) -> bool:
        """The WBC is disabled (its stream row's state, else what this node knows)."""
        row, _ = self.stream.latest()
        st = (row or {}).get("state")
        if st is not None and math.isfinite(st):
            return int(st) == 0
        return self.wbc_enabled is False

    def set_task(self, text: str) -> dict:
        return self.episodes.set_task(text)

    def _on_headset_frame(self, t_rx: float, frame: dict) -> None:
        self.oplog.headset(t_rx, frame)
        self.haptics.on_frame(t_rx, frame)

    # -- the client's observers (the loop's thread) ----------------------------------------------------------

    def _hook(self, client: TeleopClient) -> None:
        self.buttons = EpisodeButtons()
        self._fresh_prev = None
        self._connected_prev = None
        client.on_frame = lambda *a: self._on_frame(client, *a)
        client.on_step = self._on_step
        client.on_event = self._on_client_event

    def _on_frame(self, client: TeleopClient, frame: dict, verdict, now: float, fresh: bool) -> None:
        b = self.buttons
        b.on_frame(frame, verdict, now, fresh)
        for name, pressed in b.edges:
            self.emit("button", {"button": name, "pressed": pressed})
        b.edges.clear()
        if verdict.accept and x_pressed(frame):
            self.episodes.abort("x", "X: disabled")
        evs, b.events = b.events, []
        for ev in evs:
            self.episodes.on_button(ev, client)

    def _on_step(self, now: float, fresh: bool, why: str) -> None:
        c = self.client
        if fresh:
            self._fresh_prev = True
        elif self._fresh_prev and c is not None:
            self._fresh_prev = False
            g = c.gate
            if g.last_valid is None or now - g.last_valid > g.stale_s:
                kind = "staleness"
            elif "rate" in (g.timing_reason or ""):
                kind = "rate"
            else:
                kind = "latency"
            self.emit("stale", {"reason": kind, "why": why})
            # a transient the latch holds through keeps the episode (the hold); its escalation aborts it
            if not (c.latch.tier in ("armed", "hold") and self.resume.transient(input_class(why))):
                self.episodes.abort("stale", why)
        connected = bool(getattr(self.reader, "connected", False))
        if self._connected_prev and not connected:
            # a blackout the latch holds through (the reader reopens a silent connection after 1 s: the same headset
            # session is a gap) keeps the episode; its escalation aborts it
            if not (c is not None and c.latch.tier in ("armed", "hold") and self.resume.transient("input_gap")):
                self.episodes.abort("telemetry_loss", "the headset stream closed")
        self._connected_prev = connected
        row, _ = self.stream.latest()
        st = (row or {}).get("state")
        if st is not None and math.isfinite(st) and int(st) == 5:
            self._fault("the WBC stream shows a fault")

    def _on_client_event(self, ev: tuple) -> None:
        if ev[0] == "armed":
            self.emit("armed", {"arm_seq": ev[1]})
        elif ev[0] == "disarmed":
            self.emit("disarmed", {"reason": ev[1]})
            self.episodes.abort("x" if str(ev[1]).startswith("X:") else "disarm", ev[1])
        elif ev[0] in ("enable", "disable") and ev[1]:
            self.set_enabled(ev[0] == "enable", "Y" if ev[0] == "enable" else "X")
        elif ev[0] == "hold":                     # a transient: disarmed on the wire, a re-grip resumes
            info = ev[2] if len(ev) > 2 else {}
            self.emit("held", {"reason": ev[1], "class": info.get("class"), "cue": "hold: re-grip to resume"})
            self.events.add(f"hold: {ev[1]} (release and re-grip to resume)", "warn")
        elif ev[0] == "resumed":
            info = ev[2] if len(ev) > 2 else {}
            self.emit("resumed", {"arm_seq": ev[1], **info})
            self.events.add(f"resumed by a re-grip ({info.get('hand')}) after {info.get('after_s')} s: "
                            f"{info.get('cause')}")
        elif ev[0] == "y_refused":                # a Y press that did not arm: why (the page only showed it)
            self.emit("y_refused", {"reason": ev[1]})
            self.events.add(f"Y refused: {ev[1]}", "warn")
        elif ev[0] == "resume_refused":
            info = ev[2] if len(ev) > 2 else {}
            self.emit("resume_refused", {"arm_seq": ev[1], **info})
            self.events.add("the WBC refused the resume (still holding: release and re-grip)", "warn")
        elif ev[0] == "escalated":                # the hold became a disarm: Y re-arms
            info = ev[2] if len(ev) > 2 else {}
            self.emit("escalated", {"reason": ev[1], **info, "cue": "disarmed: press Y"})
            self.episodes.abort("disarm", ev[1])  # the episode survived the hold; a disarm ends it
            self.events.add(f"hold escalated to a disarm: {ev[1]} (press Y)", "warn")
        elif ev[0] == "pairing":                  # the mirror pairing locked at an engagement
            d = ev[1]
            self.emit("pairing", d)
            diff = d.get("heading_diff_deg")
            self.events.add(f"pairing: {d['pairing']} ({d['mode']}"
                            + (f", headings {abs(diff):.0f} deg apart" if diff is not None else "") + "): "
                            + ", ".join(f"{h} -> {e}" for h, e in d["hands"].items()))

    def _fault(self, reason: str) -> None:
        with self._known_lock:
            if self._fault_known:
                return
            self._fault_known = True
        self.emit("fault", {"reason": reason})
        self.episodes.abort("fault", reason)

    # -- teleop ----------------------------------------------------------------------------------------------

    def start_teleop(self) -> dict:
        """Lease, session-default hints, enable, a datagram session (teleop.open_wbc), then the loop sends."""
        with self.lock:
            if self.phase != "stopped":
                raise RequestError(409, f"teleop is {self.phase}")
            self.phase, self.error, self.stop_reason = "starting", None, None
        self.hub.state_changed()
        if self.sim:                       # the sim: every run starts from the start scene's state (and is recorded so)
            try:
                from .episodes import reset_world
                with NodeClient.for_instance(self.sim, timeout=3.0) as sim:
                    out = reset_world(sim, self.episodes.estimator)
                self.events.add(f"sim reset for the start (reset_seq {out.get('reset_seq')})")
            except Exception as e:  # noqa: BLE001 (no reset, no start: the run would not start where it should)
                msg = getattr(e, "detail", None) or str(e)
                with self.lock:
                    self.phase, self.error = "stopped", f"start refused: sim reset failed: {msg}"
                self.events.add(f"start refused: sim reset failed: {msg}", "warn")
                raise RequestError(409, f"the sim reset failed: {msg}") from None
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
        client = TeleopClient(self.cfg, self.monitor, self.stream, wbc, stale_s=self.stale_s, clock=self.clock,
                              max_latency=self.max_latency, min_rate=self.min_rate, resume=self.resume)
        client.session_opened(info)
        self._hook(client)
        wbc.on_sent = self.oplog.sent
        # the base hint's three candidates (head, chest, hips; raw and smoothed) per frame, for offline analysis
        client.core.on_hint = lambda rec: self.oplog.publish({"type": "hint", **rec})
        self.monitor.drain()            # frames from before the start are not this client's
        with self.lock:
            self.wbc, self.client = wbc, client
            self.phase, self.started_at = "running", self.clock()
            self.last_msg = None
        if self.recorder is not None:      # the sim records every run: from the events after this seq on
            self.recorder.start(self.eventlog.seq, self.episodes.task)
        self.emit("teleop_start", {"session": info.get("session"), "transport": getattr(wbc, "transport", None)})
        self.set_enabled(True, "teleop start")
        self.events.add(f"teleop started: WBC session {info['session']} over {wbc.transport}; nothing moves until "
                        f"Y (left) arms, X disables; " + self.hands_text())
        return self.teleop_info()

    def hands_text(self) -> str:
        """Which grip clutches which arm (for events and the page)."""
        hands = self.cfg.hands
        if hands:
            return ", ".join(f"the {h} grip clutches {e}" for h, e in hands.items())
        return f"the {self.cfg.hand} grip clutches the arm" + (f" ({self.cfg.ee})" if self.cfg.ee != "arm" else "")

    def set_mode(self, mode: str) -> dict:
        """Third or first person: where the sticks enter the frame chain (tools/quest_teleop/README.md). Read at
        each engagement, so it applies from the next grip; an engagement in progress keeps its mode."""
        if mode not in ("third", "first"):
            raise ValueError(f"mode must be 'third' or 'first', not {mode!r}")
        old, self.cfg.mode = self.cfg.mode, mode
        if mode != old:
            self.events.add(f"mode: {mode} person (from the next grip)")
        self.hub.state_changed()
        return {"mode": self.cfg.mode}

    def set_mirror(self, mode: str) -> dict:
        """The hand-to-arm pairing mode (two hands; ``off`` / ``auto`` / ``straight`` / ``mirror``, body_frozen.py
        "Mirror pairing"): decided at each engagement from all released. Changed only while no hand is engaged
        (409 otherwise); off / straight / mirror apply at once (released), auto at the next grip. ValueError for an
        unknown mode."""
        if mode not in MIRROR_MODES:
            raise ValueError(f"mirror must be one of {', '.join(MIRROR_MODES)}, not {mode!r}")
        with self.step_lock:                      # between the loop's steps: no engagement starts meanwhile
            c = self.client
            core = c.core if c is not None else None
            if hasattr(core, "set_mirror"):
                try:
                    core.set_mirror(mode)
                except ValueError as e:
                    raise RequestError(409, str(e)) from None
            elif mode != "off" and not self.cfg.hands:
                raise RequestError(409, "mirror pairing needs two hands (a two-arm robot)")
            old, self.cfg.mirror = self.cfg.mirror, mode
        if mode != old:
            self.events.add(f"mirror pairing: {mode}" + (" (from the next grip)" if mode == "auto" else ""))
        self.hub.state_changed()
        return {"mirror": self.cfg.mirror, "pairing": self.pairing_info()}

    def pairing_info(self) -> Optional[dict]:
        """The hand-to-arm pairing in force (two hands; MultiCore.pairing_info), else None. Not running: the
        configured one (straight unless the mode forces mirror)."""
        c = self.client
        core = c.core if c is not None else None
        if hasattr(core, "pairing_info"):
            return core.pairing_info()
        hands = self.cfg.hands
        if not hands:
            return None
        m = self.cfg.mirror == "mirror" and set(hands) == {"left", "right"}
        pair = {"left": hands["right"], "right": hands["left"]} if m else dict(hands)
        return {"mode": self.cfg.mirror, "pairing": "mirror" if m else "straight", "hands": pair, "locked": False,
                "mirror": {e: m for e in hands.values()},
                "decision": None}

    def set_elbow_hint(self, on: bool) -> dict:
        """Send each engaged arm's elbow-swivel input (tools/quest_teleop/swivel.py) in the datagrams. On applies
        from each hand's next grip (the WBC anchors the robot's swivel at a new seq), off at once."""
        old, self.cfg.elbow_hint = self.cfg.elbow_hint, bool(on)
        if self.cfg.elbow_hint != old:
            self.events.add("elbow hint: " + ("on (from the next grip)" if on else "off"))
        self.hub.state_changed()
        return {"elbow_hint": self.cfg.elbow_hint, "swivel": self.swivel_info()}

    def swivel_info(self) -> Optional[dict]:
        """Per EE: the elbow-swivel input and hint (``TeleopCore.swivel_info``), while running; else None."""
        c = self.client
        core = c.core if c is not None else None
        if core is None or not hasattr(core, "swivel_info"):
            return None
        sw = core.swivel_info()
        return {core.cfg.ee: sw} if "hand" in sw else sw

    def set_hint_source(self, source: str) -> dict:
        """What moves the base hint while engaged: ``head`` (the headset), ``chest`` or ``hips`` (body tracking,
        smoothed, falling back to the headset). Read at each engagement: applies from the next grip."""
        if source not in ("head", "chest", "hips"):
            raise ValueError(f"hint source must be head, chest or hips, not {source!r}")
        old, self.cfg.hint_source = self.cfg.hint_source, source
        if source != old:
            self.events.add(f"base hint source: {source} (from the next grip)")
        self.hub.state_changed()
        return {"hint_source": self.cfg.hint_source}

    def set_tether(self, on: bool) -> dict:
        """The stick tether (tools/quest_teleop/body_frozen.py): engaged, the stick offset stays within
        ``tether_pos`` / ``tether_yaw`` of the WBC's commanded base. Read at each engagement: from the next grip."""
        old, self.cfg.tether = self.cfg.tether, bool(on)
        if self.cfg.tether != old:
            self.events.add(f"stick tether {'on' if on else 'off'} (from the next grip)")
        self.hub.state_changed()
        return {"tether": self.cfg.tether}

    def set_headless(self, on: bool) -> dict:
        """Third person, registered: the left stick moves the robot along the operator's facing (applies at once)."""
        old, self.cfg.headless = self.cfg.headless, bool(on)
        if self.cfg.headless != old:
            self.events.add(f"headless left stick {'on' if on else 'off'}")
        self.hub.state_changed()
        return {"headless": self.cfg.headless}

    def set_resume(self, enabled: Optional[bool] = None, grace_s: Optional[float] = None,
                   hold_max_s: Optional[float] = None, classes: Optional[dict] = None) -> dict:
        """The tiered recovery (arming.py ``ResumeConfig``): on / off, the grace window, the hold's maximum age and
        each transient class on / off. Applies at once (a hold in force is judged by the new values)."""
        r = self.resume
        if grace_s is not None and not (0.0 <= float(grace_s) <= 30.0):
            raise ValueError("grace_s must be within 0 .. 30 s")
        if hold_max_s is not None and not (0.0 < float(hold_max_s) <= 3600.0):
            raise ValueError("hold_max_s must be within 0 .. 3600 s")
        unknown = set(classes or {}) - set(TRANSIENT_CLASSES)
        if unknown:
            raise ValueError(f"unknown classes {sorted(unknown)}; known: {list(TRANSIENT_CLASSES)}")
        if enabled is not None:
            r.enabled = bool(enabled)
        if grace_s is not None:
            r.grace_s = float(grace_s)
        if hold_max_s is not None:
            r.hold_max_s = float(hold_max_s)
        for k, v in (classes or {}).items():
            r.classes[k] = bool(v)
        self.events.add("resume: " + ("on" if r.enabled else "off (every disarm needs Y)")
                        + f", grace {r.grace_s:g} s, hold max {r.hold_max_s:g} s, transient: "
                        + (", ".join(k for k, v in r.classes.items() if v) or "none"))
        self.hub.state_changed()
        return r.info()

    def set_top_speed(self, max_speed: Optional[float] = None, max_omega: Optional[float] = None) -> dict:
        """The top speed (m/s, rad/s; None keeps one): the sticks' full deflection, the hint clamp and, while teleop
        runs, the WBC caps this lease asserts (restored at stop). Running, the WBC takes them only while the base is
        still (disarmed, or the sticks at rest): otherwise 409 and nothing changes. ValueError beyond the bounds."""
        trial = set_top_speed(dataclasses.replace(self.cfg), max_speed, max_omega)   # validates; nothing changed yet
        with self.lock:
            wbc = self.wbc if self.phase == "running" else None
        if wbc is not None:
            try:
                assert_caps(wbc, top_speed_caps(trial))
            except Exception as e:  # noqa: BLE001
                msg = getattr(e, "detail", None) or str(e)
                raise RequestError(409, f"the WBC refused the caps (disarm first: they change only while the base "
                                        f"is still): {msg}") from None
        set_top_speed(self.cfg, max_speed, max_omega)
        self.events.add(f"top speed {self.cfg.stick_max_speed:g} m/s, {self.cfg.stick_max_omega:g} rad/s"
                        + (" (WBC caps too)" if wbc is not None else " (the WBC caps at the next Start)"))
        self.hub.state_changed()
        return {"max_speed": self.cfg.stick_max_speed, "max_omega": self.cfg.stick_max_omega}

    def stop_teleop(self, reason: str = "requested") -> dict:
        with self.lock:
            if self.phase != "running":
                return {"stopped": False, "phase": self.phase}
            self.phase = "stopping"
            client, wbc = self.client, self.wbc
            self.client = None
        self.episodes.abort("teleop_stop", reason)
        self.hub.state_changed()
        with self.step_lock:            # the loop's current step finishes; nothing is sent after this
            pass
        if client is not None:
            client.close()
        problems = []
        close_wbc(wbc, log=problems.append)
        wbc.close()
        with self.lock:
            self.wbc, self.phase, self.stop_reason = None, "stopped", reason
            if problems:
                self.error = "stop: " + "; ".join(p.strip() for p in problems)
        self.set_enabled(False, "teleop stop")
        self.emit("teleop_stop", {"reason": reason})
        if self.recorder is not None:
            self.recorder.stop()            # it settles and closes on its own; never waited for here
        self.events.add(f"teleop stopped ({reason})" + (f": {'; '.join(problems)}" if problems else ""),
                        "warn" if reason != "requested" else "info")
        return {"stopped": True, "reason": reason, "engagements": client.core.status.engagements}

    def _auto_stop(self, reason: str) -> None:
        threading.Thread(target=self.stop_teleop, args=(reason,), name="quest-teleop-stop", daemon=True).start()

    ESTOP_TIMEOUTS = {"wbc": 2.5, "arm": 1.0, "swerve": 1.0}   # the WBC answers after its own fan-out

    def estop(self, reason: str = "", source: str = "") -> dict:
        """E-stop the arm, the swerve and the WBC directly and IN PARALLEL (one thread each: a slow or dead one
        never delays the others; each latches and coasts), then stop teleop. ``latched``: any of them latched."""
        why, src = reason or "quest_teleop e-stop", source or self.name
        res: dict = {}

        def post(name: str, target: str) -> None:
            try:
                with NodeClient.for_instance(target, timeout=self.ESTOP_TIMEOUTS.get(name, self.ESTOP_TIMEOUTS["arm"])) as c:
                    r = c.post("/estop", {"reason": why, "source": src})
                res[name] = {"latched": bool(isinstance(r, dict) and r.get("latched", True)), "error": None,
                             "reply": r if isinstance(r, dict) else None}
            except Exception as e:  # noqa: BLE001
                res[name] = {"latched": False, "error": f"{type(e).__name__}: {e}"[:200], "reply": None}

        arms = [("arm" if len(self.arm_targets) == 1 else a, a) for a in self.arm_targets]
        targets = [(n, t) for n, t in (*arms, ("swerve", self.swerve_target), ("wbc", self.wbc_target)) if t]
        threads = [threading.Thread(target=post, args=nt, name=f"quest-estop-{nt[0]}", daemon=True) for nt in targets]
        for t in threads:
            t.start()
        with self._known_lock:
            self._estop_known = True
        self.emit("estop", {"reason": why, "source": src})
        self.episodes.abort("estop", why)
        deadline = time.monotonic() + max(self.ESTOP_TIMEOUTS.values()) + 0.5
        for t in threads:
            t.join(max(0.0, deadline - time.monotonic()))
        out_t = {n: {k: v for k, v in res.get(n, {"latched": False, "error": "no answer in time"}).items()
                     if k != "reply"} for n, _ in targets}
        wbc = out_t.get("wbc", {})
        if wbc.get("latched"):
            self.wbc_estop = {"latched": True, "reason": why, "source": src, "t": self.clock()}
        for n, r in out_t.items():
            self.events.add(f"e-stop to the {n}: " + ("latched" if r["latched"] else f"FAILED: {r['error']}"),
                            "warn" if r["latched"] else "error")
        running = self.phase == "running"
        if running:
            self._auto_stop("e-stop")
        wr = (res.get("wbc") or {}).get("reply")
        out = {"latched": any(r["latched"] for r in out_t.values()), "targets": out_t,
               "forwarded": bool(wbc.get("latched")), "wbc_forwarded": wr.get("forwarded") if wr else None,
               "teleop_stopping": running}
        errs = [f"{n}: {r['error']}" for n, r in out_t.items() if r["error"]]
        if errs:
            out["error"] = "; ".join(errs)
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
                if self.body != "off":
                    self._ctl_log(f"body tracking: {self.body}")
                rc = ctl.up(self.scheme, False, False, wait, ar, self.body)
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
            try:
                self.haptics.update(now)
            except Exception:  # noqa: BLE001 (feedback never stops the loop)
                log.exception("haptics")
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
        bp = h["battery"]["pct"]
        row["battery_pct"] = nan if bp is None else bp
        wrow, wt = self.stream.latest()
        if wrow is not None:
            ee = self.cfg.ee
            row["wbc_armed"] = wrow.get("armed", nan)
            row.update(wbc_state=wrow.get("state", nan), wbc_base_mode=wrow.get("base_mode", nan),
                       wbc_limited=wrow.get("limited", nan), wbc_limited_by=wrow.get("limited_by", nan),
                       wbc_age=now - wt, base_x=wrow.get("base_x", nan), base_y=wrow.get("base_y", nan),
                       base_yaw=wrow.get("base_yaw", nan))
            for a in "xyz":
                row[f"ee_cmd_{a}"] = wrow.get(f"{ee}_cmd_{a}", nan)
                row[f"ee_tgt_{a}"] = wrow.get(f"{ee}_tgt_{a}", nan)
        if client is not None:
            core = client.core
            row["armed"] = float(client.latch.armed)
            row["tier"] = float(TIERS.index(client.latch.tier)) if client.latch.tier in TIERS else nan
            g = client.gate
            row["quest_latency"] = nan if g.latency is None else g.latency
            row["quest_rate_short"] = g.rate(now)
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
                    "armed": s.get("armed"), "disarm_reason": s.get("disarm_reason"),
                    "holder": s.get("holder"), "fault": s.get("fault"), "hold_reason": s.get("hold_reason"),
                    "disabled_reason": s.get("disabled_reason"), "limited_by": s.get("limited_by"),
                    "base_mode": ((s.get("hints") or {}).get("base") or {}).get("mode"),
                    "lease": {"held": bool(lease.get("held")), "holder": lease.get("holder")},
                    "estop": {"latched": bool(e.get("latched")), "reason": e.get("reason"), "source": e.get("source")},
                    "dropped": (s.get("session") or {}).get("dropped")}
                self._poll_events(self.wbc_state)
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
                    elif not w.get("enabled") and not self._disabled_by_x():
                        # (disabled by our own X: the session stays open, Y re-enables)
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

    def _poll_events(self, w: dict) -> None:
        """Events from what the 1 Hz poll sees change (between two polls, so a poll that raced our own request is not
        mistaken for someone else's): an e-stop or a fault not seen yet, a hold, an enable / disable."""
        prev = self._poll_prev
        cur = self._poll_prev = {"latched": w["estop"]["latched"], "fault": w.get("fault"),
                                 "hold": w.get("hold_reason"), "enabled": bool(w.get("enabled"))}
        if cur["latched"] and not prev.get("latched"):
            with self._known_lock:
                new, self._estop_known = not self._estop_known, True
            if new:
                e = w["estop"]
                self.emit("estop", {"reason": e.get("reason") or "", "source": e.get("source") or "wbc"})
                self.episodes.abort("estop", e.get("reason") or "the WBC e-stop is latched")
        elif not cur["latched"]:
            self._estop_known = False
        if cur["fault"]:
            self._fault(str(cur["fault"]))
        else:
            row, _ = self.stream.latest()
            st = (row or {}).get("state")
            if st is None or not math.isfinite(st) or int(st) != 5:
                self._fault_known = False
        if cur["hold"] and cur["hold"] != prev.get("hold"):
            self.emit("hold", {"reason": cur["hold"]})
        if prev and cur["enabled"] != prev.get("enabled") and self.phase == "running":
            self.set_enabled(cur["enabled"], "seen enabled" if cur["enabled"]
                             else (w.get("disabled_reason") or "seen disabled"))

    def _disabled_by_x(self) -> bool:
        c = self.client
        return bool(c is not None and c.latch.disabled_by_client)

    # -- state -----------------------------------------------------------------------------------------------

    def teleop_info(self) -> dict:
        c, w = self.client, self.wbc
        core = c.core if c else None
        lt = c.latch if c else None
        now = self.clock()
        return {"phase": self.phase, "state": self.client_state(),
                "armed": bool(lt and lt.armed), "arm_seq": lt.arm_seq if lt else None,
                "disarm_reason": (None if lt.armed else lt.reason) if lt else None,
                "disabled_by_x": bool(lt and lt.disabled_by_client), "presses": lt.presses if lt else 0,
                "tier": lt.tier_info(now) if lt else {"tier": "disarmed", "why": "teleop not running", "hold": None,
                                                     "resumes": 0, "config": self.resume.info()},
                "input": None if c is None else {
                    "fresh": c.gate.fresh(now), "why": None if c.gate.fresh(now) else c.gate.why_stale(now),
                    "latency_ms": None if c.gate.latency is None else round(c.gate.latency * 1e3, 1),
                    "rate_hz": round(c.gate.rate(now), 1), "late": c.gate.counts.get("late", 0),
                    "slow": c.gate.counts.get("slow", 0)},
                "sending": bool(c and c.sending), "reason": core.status.reason if core else None,
                "refused": dict(core.status.refused) if core else {},
                "engagements": core.status.engagements if core else 0,
                "registered": bool(core and core.status.registered),
                "registration": core.status.registration if core else "",
                "hands": core.hands_status() if core is not None and hasattr(core, "hands_status") else None,
                "pairing": self.pairing_info(),
                "hint": {"selected": self.cfg.hint_source,
                         "engaged_source": core.eng.hint_source if core and core.eng else None,
                         "in_force": core.status.hint_in_force if core else None,
                         "confidence": core.status.hint_confidence if core else None,
                         "why_not": (core.status.hint_reason or None) if core else None,
                         "switches": core.status.hint_switches if core else 0},
                "tether": core.status.tether if core else None,
                "swivel": self.swivel_info(),
                "sent": dict(w.sent) if w else None, "renew_errors": w.renew_errors if w else None,
                "transport": w.transport if w else None, "session": w.session if w else None,
                "wbc_caps": dict(w.caps) if w else None,
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
            "battery_pct": self.monitor.battery()["pct"],
            "config": {"wbc": self.wbc_target, "arm": self.arm_target, "swerve": self.swerve_target,
                       "max_latency_ms": round(self.max_latency * 1e3, 1), "min_rate": self.min_rate,
                       "quest_host": self.quest_host, "quest_port": self.quest_port,
                       "ssh_host": self.ssh_host, "ar": self.ar, "scheme": self.scheme, "body": self.body, "hand": cfg.hand, "ee": cfg.ee,
                       "hands": dict(cfg.hands) if cfg.hands else None,
                       "head_walk": cfg.head_walk, "mode": cfg.mode, "hint_source": cfg.hint_source, "headless": cfg.headless,
                       "mirror": cfg.mirror, "elbow_hint": cfg.elbow_hint, "tether": cfg.tether,
                       "max_speed": cfg.stick_max_speed, "max_omega": cfg.stick_max_omega, "gripper": cfg.gripper, "stale_ms": round(self.stale_s * 1e3, 1),
                       "ctl_timeout": self.ctl_timeout, "data_dir": self.data_dir, "sim": self.sim,
                       "reset_timeout": self.episodes.reset_timeout, "resume": self.resume.info()},
            "latest": self.latest,
            "sim_aids": None if self.aids is None else self.aids.info(),
            "haptics": self.haptics.info(now),
            "recording": {"enabled": False} if self.recorder is None else self.recorder.info(),
            "episode": self.episodes.open_episode(),
            "episodes": self.episodes.info(),
            "task": self.episodes.task,
            "last_event": self.eventlog.last,
            "eventlog": self.eventlog.info(),
            "oplog": {"subscribers": self.oplog.subscribers, "records": dict(self.oplog.counts)},
            "events": self.events.list()[-30:],
        }
