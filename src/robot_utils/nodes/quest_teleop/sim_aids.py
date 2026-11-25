"""Sim-only aids of the teleop node (``--sim``; docs/quest_teleop_node.md, "Sim aids"), one thread at 30 Hz:

- **Goal ghost**: the gripper drawn at the end effector's goal (the WBC's target), coloured by how far the hand is
  from it, so error and deflection are visible in the AR view. Posted to the sim as a ghost (``POST /ghosts``)
  **relative** to the hand: goal in the measured end link's frame = tcp * meas^-1 * tgt * tcp^-1 (the WBC's
  target and measured TCP poses from its stream, its TCP offset from ``/state``), which the sim applies to its own
  end link. Both poses come from the same estimate, so estimator drift cancels. Colour: severity s = max(|tgt -
  meas| / ``ghost_pos_red``, angle / ``ghost_rot_red``): green at 0, yellow at 0.5, red from 1; see-through
  (``ghost_alpha``). Hidden (its ttl lapses) while there is no target (disarmed, released, disabled).
- **Auto pause** (optional, off by default): sim time stops while the operator is neither clutched (grip engaged)
  nor commanding the base (both sticks inside their dead zone) for ``pause_hold`` s, and runs again at the first
  grip or stick motion. Only while teleop runs, never during an episode's sim reset; the node only resumes what it
  paused. Every pause and resume is an event (``sim_pause`` / ``sim_resume``), so recordings can drop the frozen
  spans. Measured on the sim stack (2026-11-24): 4 s paused leaves the WBC enabled, the estimator healthy (pose age
  ~0.12 s) and the arm servoing; nothing faults.
"""
from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ...scene3d.pose import angle, compose, inverse
from ..common.client import NodeClient

log = logging.getLogger("quest_teleop.sim_aids")

GHOST = "ee_goal"
SOURCE = "quest_teleop"
GREEN, YELLOW, RED = (0.25, 0.9, 0.45), (1.0, 0.85, 0.2), (1.0, 0.25, 0.2)


@dataclass
class AidsConfig:
    ghost: bool = True
    ghost_alpha: float = 0.35
    ghost_pos_red: float = 0.05            # m: target - measured at which the ghost is fully red
    ghost_rot_red: float = math.radians(20)  # rad
    ghost_ttl: float = 0.3                 # s: hidden when not posted again within it
    auto_pause: bool = False
    pause_hold: float = 0.25               # s idle before the sim pauses
    hz: float = 30.0


def severity_color(pos_err: float, rot_err: float, cfg: AidsConfig) -> list:
    """rgba for an error: green -> yellow (s = 0.5) -> red (s >= 1)."""
    s = max(pos_err / cfg.ghost_pos_red, rot_err / cfg.ghost_rot_red)
    s = min(max(s, 0.0), 1.0)
    a, b, k = (GREEN, YELLOW, s / 0.5) if s < 0.5 else (YELLOW, RED, (s - 0.5) / 0.5)
    return [round(a[i] + (b[i] - a[i]) * k, 3) for i in range(3)] + [cfg.ghost_alpha]


def goal_in_link(row: dict, ee: str, tcp) -> Optional[tuple]:
    """(the goal of the end link in its measured frame [7], position error m, rotation error rad) from a WBC row,
    or None without a finite target and measured pose."""
    keys = ("x", "y", "z", "qx", "qy", "qz", "qw")
    tgt = np.array([row.get(f"{ee}_tgt_{k}", math.nan) for k in keys], float)
    meas = np.array([row.get(f"{ee}_meas_{k}", math.nan) for k in keys], float)
    if not (np.all(np.isfinite(tgt)) and np.all(np.isfinite(meas))) or np.linalg.norm(tgt[3:]) < 1e-6:
        return None
    t = np.array([*tcp, 0, 0, 0, 1], float)
    delta = compose(compose(compose(t, inverse(meas)), tgt), inverse(t))
    err_rot = angle(compose(inverse(meas), tgt)[3:])
    return delta, float(np.linalg.norm(tgt[:3] - meas[:3])), err_rot


class SimAids:
    def __init__(self, node, sim: str, cfg: Optional[AidsConfig] = None):
        self.node = node
        self.sim = sim
        self.cfg = cfg or AidsConfig()
        self.paused_by_us = False
        self.last_active = 0.0
        self.ee_link: Optional[str] = None
        self.tcp: Optional[list] = None
        self.error: Optional[str] = None
        self.posted = 0
        self.pauses = 0
        self._sim: Optional[NodeClient] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="quest-sim-aids", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(2)
        self._resume("node shutting down")
        if self._sim is not None:
            self._sim.close()

    def info(self) -> dict:
        c = self.cfg
        return {"ghost": c.ghost, "auto_pause": c.auto_pause, "paused": self.paused_by_us, "pauses": self.pauses,
                "ee_link": self.ee_link, "tcp": self.tcp, "ghosts_posted": self.posted, "error": self.error,
                "ghost_pos_red": c.ghost_pos_red, "ghost_rot_red_deg": round(math.degrees(c.ghost_rot_red), 1),
                "pause_hold": c.pause_hold}

    def set_auto_pause(self, on: bool) -> dict:
        self.cfg.auto_pause = bool(on)
        self.node.events.add(f"sim auto pause {'on' if on else 'off'}")
        if not on:
            self._resume("auto pause off")
        self.node.hub.state_changed()
        return self.info()

    def set_ghost(self, on: bool) -> dict:
        self.cfg.ghost = bool(on)
        self.node.events.add(f"goal ghost {'on' if on else 'off'}")
        if not on:
            self._call("delete", f"/ghosts/{GHOST}")
        self.node.hub.state_changed()
        return self.info()

    # -- the loop --------------------------------------------------------------------------------------------

    def _client(self) -> NodeClient:
        if self._sim is None:
            self._sim = NodeClient.for_instance(self.sim, timeout=1.0)
        return self._sim

    def _call(self, method: str, path: str, body=None):
        try:
            c = self._client()
            out = c.post(path, body or {}) if method == "post" else c.get(path) if method == "get" else \
                c.delete(path)
            self.error = None
            return out
        except Exception as e:  # noqa: BLE001 (the sim restarting: try again next tick)
            self.error = f"{method} {path}: {type(e).__name__}: {e}"[:200]
            if self._sim is not None:
                self._sim.close()
                self._sim = None
            return None

    def _run(self) -> None:
        period = 1.0 / self.cfg.hz
        while not self._stop.wait(period):
            try:
                self.tick(self.node.clock())
            except Exception:  # noqa: BLE001 (never take the node down)
                log.exception("sim aids")

    def tick(self, now: float) -> None:
        node = self.node
        running = node.phase == "running" and node.client is not None
        if self.cfg.ghost and running:
            self._ghost()
        if not self.cfg.auto_pause or not running or node.episodes.state == "resetting":
            # never paused meanwhile: a sim reset waits on the estimator, which needs sim time
            self._resume("auto pause off" if not self.cfg.auto_pause else
                         "teleop stopped" if not running else "sim reset")
            self.last_active = now
            return
        if self._active():
            self.last_active = now
            self._resume("operator active")
        elif not self.paused_by_us and now - self.last_active >= self.cfg.pause_hold:
            st = self._call("get", "/state")
            if st is not None and st.get("state") == "running" and self._call("post", "/pause") is not None:
                self.paused_by_us = True
                self.pauses += 1
                node.emit("sim_pause", {"reason": "idle", "t_sim": (st or {}).get("t")})
                node.hub.state_changed()

    def _active(self) -> bool:
        """Clutched, or a stick out of its dead zone (a missing frame counts as active: never pause blind)."""
        from .client import sticks_neutral
        c = self.node.client
        if c is not None and c.core.status.state == "engaged":
            return True
        frame = self.node.monitor.last
        return not frame or not sticks_neutral(frame)

    def _resume(self, why: str) -> None:
        if not self.paused_by_us:
            return
        if self._call("post", "/run") is not None:
            self.paused_by_us = False
            self.node.emit("sim_resume", {"reason": why})
            self.node.hub.state_changed()

    def _ghost(self) -> None:
        if self.tcp is None:
            self._read_tcp()
            if self.tcp is None:
                return
        row, _ = self.node.stream.latest()
        g = goal_in_link(row or {}, self.node.cfg.ee, self.tcp)
        if g is None:
            return                                   # no target: the posted ghost lapses (hidden)
        delta, ep, er = g
        body = {"source": SOURCE, "ghosts": [{"name": GHOST, "body": self.ee_link, "subtree": True,
                                              "relative": [round(float(x), 6) for x in delta],
                                              "rgba": severity_color(ep, er, self.cfg), "ttl": self.cfg.ghost_ttl}]}
        if self._call("post", "/ghosts", body) is not None:
            self.posted += 1

    def _read_tcp(self) -> None:
        """The WBC's end link and TCP offset for this EE (its /state model), and the sim's name for that link."""
        try:
            with NodeClient.for_instance(self.node.wbc_target, timeout=1.0) as w:
                ee = ((w.get("/state").get("model") or {}).get("ees") or {}).get(self.node.cfg.ee) or {}
            link, tcp = ee.get("link"), ee.get("tcp")
            if not link or tcp is None:
                self.error = "the WBC's model has no EE link / TCP yet"
                return
            self.ee_link, self.tcp = link, [float(x) for x in tcp]
        except Exception as e:  # noqa: BLE001
            self.error = f"WBC /state: {type(e).__name__}: {e}"[:200]
