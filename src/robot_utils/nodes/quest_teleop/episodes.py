"""Episodes in the teleop node (docs/quest_teleop_node.md, "Episodes"; the plan: docs/plans/data_collection_v1.md).

The node only says what happened, as events (``episode_start`` / ``episode_stop`` / ``episode_abort``, and in the
sim ``sim_reset``); recording is the client-side script's job. States: ``idle`` -> A -> (``resetting``, sim only)
-> ``recording`` -> B tap (kept) or an abort -> ``idle``.

- **A** (the right controller, ``tools/quest_teleop/episodes.py`` for the press rules) while armed: start an
  episode (one recording ends first, kept). Disarmed, disabled or mid-reset: refused (a ``button`` event with
  ``action: "refused"`` and the reason). In the sim (``--sim``) first the reset: disarm -> WBC disable -> ``POST
  /scene/reset`` on the sim -> the estimator's reset in the WBC stream (``est_reset`` changed, within
  ``reset_timeout``) -> WBC enable, which leaves it **disarmed** (the operator arms with Y as always; nothing here
  ever arms) -> ``episode_start``. Y presses are refused while it runs (the latch's ``inhibit``).
- **B** tap: stop and keep. B held 2 s: abort (the headset stops its telemetry).
- **Aborts** (``reason`` code, ``why`` text): ``x`` (X), ``disarm`` (any disarm), ``disabled``, ``estop``,
  ``fault`` (the WBC's), ``stale`` (staleness / latency / rate), ``telemetry_loss`` (the headset stream closed),
  ``b_hold``, ``teleop_stop``; during a sim reset the same cancel the reset (the pending episode is aborted, the
  WBC stays disabled: Y enables and arms as after X), and ``reset`` when the reset itself fails or times out.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Optional

from ..common.client import NodeClient

log = logging.getLogger("quest_teleop.episodes")

RESET_TIMEOUT = 5.0
# aborts that a sim reset causes itself (its own disarm and disable): they never cancel it
RESET_OWN = ("disarm", "disabled")


class _Cancelled(Exception):
    pass


def _num(v) -> Optional[float]:
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def reset_world(sim, estimator: Optional[str] = "estimator", timeout: float = 3.0) -> dict:
    """The sim's world reset (``POST /scene/reset``), then the estimator's (``POST /reset`` at the base's start pose):
    the estimator does not see a sim reset as one (the wheel encoders jumping back to 0 are skipped as an odometry
    discontinuity), so without this its pose and ``reset_id`` stay where they were and nothing waiting for a new
    estimator frame (the WBC's ``est_reset``) ever sees one. ``sim``: an open NodeClient of the sim. Returns the
    sim's reply (``reset_seq``)."""
    out = sim.post("/scene/reset", {}) or {}
    if estimator:
        b = (sim.get("/state") or {}).get("base") or {}
        xyz, rpy = b.get("xyz") or [0.0, 0.0, 0.0], b.get("rpy") or [0.0, 0.0, 0.0]
        with NodeClient.for_instance(estimator, timeout=timeout) as est:
            est.post("/reset", {"x": float(xyz[0]), "y": float(xyz[1]), "theta": float(rpy[2])})
    return out


class Episodes:
    def __init__(self, node, sim: Optional[str] = None, reset_timeout: float = RESET_TIMEOUT,
                 estimator: Optional[str] = "estimator"):
        self.node = node
        self.sim = sim
        self.estimator = estimator
        self.reset_timeout = reset_timeout
        self.lock = threading.RLock()
        self.state = "idle"                    # idle | resetting | recording
        self.current: Optional[dict] = None    # the recording episode: id, task, t_start, wall_start
        self.pending: Optional[str] = None     # the episode id a sim reset is preparing
        self.cancel: Optional[tuple] = None    # (code, why): stop the running sim reset
        self.task = ""
        self.last: Optional[dict] = None       # the newest finished episode: id, outcome, reason, why
        self.counts = {"kept": 0, "aborted": 0, "refused": 0}
        self.refused: Optional[str] = None     # why the last A was refused
        self._thread: Optional[threading.Thread] = None

    @property
    def episode_id(self) -> Optional[str]:
        c = self.current
        return c["id"] if c else None

    def set_task(self, text: str) -> dict:
        with self.lock:
            self.task = str(text)
        self.node.events.add(f"task: {self.task!r}" if self.task else "task cleared")
        return {"task": self.task}

    def open_episode(self) -> Optional[dict]:
        """/state's ``episode`` (the contract): the open episode ``{id, task, t_start, start_seq}``, else None."""
        c = self.current
        return None if c is None else {"id": c["id"], "task": c["task"], "t_start": c["t_start"],
                                       "start_seq": c["start_seq"]}

    def info(self) -> dict:
        c = self.current
        now = self.node.clock()
        return {"state": self.state, "id": c["id"] if c else None, "task": self.task,
                "episode_task": c["task"] if c else None,
                "elapsed": round(now - c["t_start"], 1) if c else None, "pending": self.pending,
                "last": self.last, "refused": self.refused, "counts": dict(self.counts), "sim": self.sim}

    # -- the buttons (the teleop loop's thread) --------------------------------------------------------------

    def on_button(self, ev: str, client) -> None:
        if ev == "A":
            self._a(client)
        elif ev == "B":
            with self.lock:
                if self.state == "recording":
                    self._stop_kept()
                elif self.state == "resetting":
                    self.cancel = ("b_tap", "B: stopped before the episode started")
        elif ev == "B_hold":
            self.abort("b_hold", "B held 2 s: the headset stops its telemetry")

    def _refuse(self, why: str) -> None:
        self.counts["refused"] += 1
        self.refused = why
        self.node.emit("button", {"button": "A", "pressed": True, "action": "refused", "reason": why})
        self.node.events.add(f"A refused: {why}", "warn")

    def _a(self, client) -> None:
        lt = client.latch
        with self.lock:
            if self.state == "resetting":
                return self._refuse("a sim reset is in progress")
            if not lt.armed:                   # (armed implies enabled: the latch disarms when the WBC is not)
                return self._refuse("disabled (Y enables and arms)" if self.node.wbc_disabled() else
                                    "disarmed (Y arms)")
            self.refused = None
            if self.state == "recording":
                self._stop_kept()
            eid = self.node.eventlog.next_episode()
            if not self.sim:
                self._start(eid)
                return
            self.state, self.pending, self.cancel = "resetting", eid, None
            lt.inhibit = "sim reset in progress"
            lt.disarm("sim reset")             # sent as armed: false by the loop right after this frame
            self.node.events.add(f"episode {eid}: resetting the sim")
            self._thread = threading.Thread(target=self._reset, args=(eid, client, self.node.wbc),
                                            name="quest-sim-reset", daemon=True)
            self._thread.start()

    # -- transitions (under the lock) ------------------------------------------------------------------------

    def _start(self, eid: str) -> None:
        now = self.node.clock()
        self.current = {"id": eid, "task": self.task, "t_start": now, "wall_start": time.time(), "start_seq": None}
        self.state, self.pending = "recording", None
        line = self.node.emit("episode_start", {"task": self.task}, episode=eid)
        self.current.update(t_start=line["t"], start_seq=line["seq"])
        self.node.events.add(f"episode {eid} started" + (f": {self.task}" if self.task else ""))

    def _finish(self, outcome: str, code: Optional[str] = None, why: str = "") -> None:
        c = self.current
        self.current, self.state = None, "idle"
        self.last = {"id": c["id"], "outcome": outcome, "reason": code, "why": why or None,
                     "duration": round(self.node.clock() - c["t_start"], 2)}
        self.counts[outcome] += 1

    def _stop_kept(self) -> None:
        eid = self.episode_id
        self.node.emit("episode_stop", {}, episode=eid)
        self._finish("kept")
        self.node.events.add(f"episode {eid} stopped (kept)")

    def abort(self, code: str, why: str = "") -> None:
        """End the recording episode as aborted; during a sim reset: cancel it (except for its own disarm and
        disable). Idle: nothing."""
        with self.lock:
            if self.state == "recording":
                eid = self.episode_id
                self.node.emit("episode_abort", {"reason": code, "why": why}, episode=eid)
                self._finish("aborted", code, why)
                self.node.events.add(f"episode {eid} aborted: {code}" + (f" ({why})" if why else ""), "warn")
            elif self.state == "resetting" and code not in RESET_OWN and self.cancel is None:
                self.cancel = (code, why)

    # -- the sim reset (its own thread) ----------------------------------------------------------------------

    def _check(self) -> None:
        if self.cancel is not None:
            raise _Cancelled()

    def _reset(self, eid: str, client, wbc) -> None:
        node = self.node
        lt = client.latch
        was_disabled = lt.disabled_by_client
        enabled = disabling = False
        try:
            if wbc is None:
                raise RuntimeError("teleop is not running")
            with NodeClient.for_instance(self.sim, timeout=3.0) as sim:
                try:
                    sim.get("/state")
                except Exception as e:  # noqa: BLE001
                    raise RuntimeError(f"the sim ({self.sim}) does not answer: {e}") from None
                row, _ = node.stream.latest()
                est0 = _num((row or {}).get("est_reset"))
                self._check()
                # from here the WBC is disabled by us: the session stays open (as after X; the watchdog lets it
                # be) and, if the reset does not complete, Y enables and arms as after X
                lt.disabled_by_client = disabling = True
                wbc.disable()
                node.set_enabled(False, "sim reset")
                self._check()
                out = reset_world(sim, self.estimator)
            node.emit("sim_reset", {"reset_seq": (out or {}).get("reset_seq")})
            deadline = node.clock() + self.reset_timeout
            while True:
                row, _ = node.stream.latest()
                v = _num((row or {}).get("est_reset"))
                if v is not None and v != est0:
                    break
                if node.clock() > deadline:
                    raise RuntimeError(f"the estimator's reset did not show in the WBC stream within "
                                       f"{self.reset_timeout:g} s (est_reset still {est0})")
                self._check()
                time.sleep(0.02)
            self._check()
            wbc.enable()                       # re-seeded from the measured joints; DISARMED (Y arms)
            enabled = True
            with self.lock:
                self._check()
                lt.disabled_by_client = False
                node.set_enabled(True, "sim reset")
                self._start(eid)
        except _Cancelled:
            code, why = self.cancel
            self._fail(eid, code, why or "cancelled during the sim reset", wbc if enabled else None)
        except Exception as e:  # noqa: BLE001 (any failure: abort the pending episode, stay disabled)
            log.warning("sim reset: %s", e)
            self._fail(eid, "reset", str(getattr(e, "detail", None) or e)[:200], wbc if enabled else None)
        finally:
            if not disabling:
                lt.disabled_by_client = was_disabled
            lt.inhibit = None

    def _fail(self, eid: str, code: str, why: str, enabled_wbc) -> None:
        if enabled_wbc is not None:            # cancelled after our enable: disabled again
            try:
                enabled_wbc.disable()
            except Exception as e:  # noqa: BLE001
                log.warning("sim reset: disable after a cancel: %s", e)
        with self.lock:
            self.node.emit("episode_abort", {"reason": code, "why": why}, episode=eid)
            self.counts["aborted"] += 1
            self.last = {"id": eid, "outcome": "aborted", "reason": code, "why": why, "duration": 0.0}
            self.state, self.pending, self.cancel = "idle", None, None
        self.node.events.add(f"episode {eid} aborted before it started: {code} ({why})", "warn")
