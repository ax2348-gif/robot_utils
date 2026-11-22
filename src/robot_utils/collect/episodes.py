"""The episode index, built from the operator stream's ``event`` records and the Pi event log.

Events are applied strictly in ``seq`` order. An event that arrives ahead of a missing ``seq``
is held and a reconcile is requested (the session then fetches ``GET /eventlog?since_seq=N``
and ``add``s what it returns); the Pi log is authoritative, and a ``seq`` the log does not
have either is skipped (``skip_missing``) and noted in ``events.jsonl``.

Episode boundaries (the contract's event line, docs/plans/data_collection_v1.md):
``episode_start`` opens ``episode`` with ``detail.task``; ``episode_stop`` closes it ``kept``;
``episode_abort`` closes it ``aborted`` with ``detail.reason``. Every event while it is open is
listed in its ``events`` (seqs). An episode still open when the session ends is closed
``aborted`` / ``session ended`` (``close_open``).
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Callable, Optional

log = logging.getLogger("collect")


class EpisodeIndex:
    def __init__(self, episodes_path: Path, events_path: Path, applied_seq: Optional[int] = None,
                 on_episode: Optional[Callable[[dict], None]] = None):
        """``applied_seq``: the newest seq already handled before this session (None: take the
        first event seen as the start)."""
        self.episodes_path = Path(episodes_path)
        self.events_path = Path(events_path)
        self.applied_seq = applied_seq
        self.held: dict[int, tuple[dict, str]] = {}
        self.open: Optional[dict] = None
        self.finished: list[dict] = []
        self.skipped: list[int] = []
        self.on_episode = on_episode
        self.last_event_t: Optional[float] = None

    # -- intake --------------------------------------------------------------------------------

    def add(self, event: dict, via: str) -> bool:
        """Take an event (``via``: "stream" or "eventlog"); apply what is now in order. Returns
        True when a seq is missing ahead of held events (reconcile from the Pi log)."""
        seq = int(event["seq"])
        if self.applied_seq is None:
            self.applied_seq = seq - 1
        if seq > self.applied_seq and seq not in self.held:
            self.held[seq] = (event, via)
        self.advance()
        return self.has_hole()

    def advance(self):
        while self.applied_seq + 1 in self.held:
            ev, via = self.held.pop(self.applied_seq + 1)
            self.applied_seq += 1
            self._apply(ev, via)

    def has_hole(self) -> bool:
        return bool(self.held)

    def skip_missing(self, up_to: Optional[int] = None, why: str = "not in the Pi event log") -> list[int]:
        """Give up on the missing seqs below the held events (only those <= ``up_to`` if given)
        and apply what follows. Returns the skipped seqs."""
        skipped = []
        while self.held:
            nxt = self.applied_seq + 1
            if nxt in self.held:
                self.advance()
                continue
            if up_to is not None and nxt > up_to:
                break
            skipped.append(nxt)
            self.applied_seq = nxt
        if skipped:
            self.skipped += skipped
            self._log_event({"missing": skipped, "why": why})
            log.warning("events %s..%s missing (%s)", skipped[0], skipped[-1], why)
        return skipped

    # -- episodes ------------------------------------------------------------------------------

    def _log_event(self, line: dict):
        with open(self.events_path, "a") as f:
            f.write(json.dumps(line) + "\n")

    def _apply(self, ev: dict, via: str):
        self._log_event({**ev, "via": via})
        self.last_event_t = ev.get("t", self.last_event_t)
        kind = ev.get("kind")
        detail = ev.get("detail") or {}
        if kind == "episode_start":
            if self.open is not None:
                self._close("aborted", "no stop event before the next episode_start", ev, include=False)
            self.open = {"id": ev.get("episode"), "task": detail.get("task"), "t_start": ev.get("t"),
                         "wall_start": ev.get("wall"), "seq_start": ev["seq"], "events": [ev["seq"]],
                         "via": [via]}
            return
        if self.open is None:
            if kind in ("episode_stop", "episode_abort"):
                log.warning("%s for episode %s, which started before this session: not indexed",
                            kind, ev.get("episode"))
            return
        if kind in ("episode_stop", "episode_abort") and ev.get("episode") not in (None, self.open["id"]):
            log.warning("%s for %s while %s is open: ignored", kind, ev.get("episode"), self.open["id"])
            self.open["events"].append(ev["seq"])
            return
        self.open["events"].append(ev["seq"])
        self.open["via"].append(via)
        if kind == "episode_stop":
            self._close("kept", None, ev)
        elif kind == "episode_abort":
            self._close("aborted", detail.get("reason"), ev)

    def _close(self, outcome: str, reason: Optional[str], ev: Optional[dict], *, include: bool = True,
               t: Optional[float] = None, wall: Optional[float] = None):
        o = self.open
        self.open = None
        stop = ev if (ev is not None and include) else None
        episode = {
            "id": o["id"], "task": o["task"], "t_start": o["t_start"],
            "t_stop": stop["t"] if stop else (ev["t"] if ev is not None else t),
            "wall": {"start": o["wall_start"],
                     "stop": stop["wall"] if stop else (ev["wall"] if ev is not None else wall)},
            "outcome": outcome, "reason": reason,
            "seq_start": o["seq_start"], "seq_stop": stop["seq"] if stop else None,
            "events": o["events"],
            "reconciled": "eventlog" in o["via"],
        }
        self.finished.append(episode)
        with open(self.episodes_path, "a") as f:
            f.write(json.dumps(episode) + "\n")
            f.flush()
            os.fsync(f.fileno())
        if self.on_episode is not None:
            self.on_episode(episode)

    def close_open(self, reason: str, t: Optional[float], wall: float):
        """End of session: the open episode (if any) -> aborted with ``reason``, stopped at ``t``."""
        if self.open is not None:
            self._close("aborted", reason, None, t=t, wall=wall)


def read_episodes(path: Path) -> list[dict]:
    if not Path(path).exists():
        return []
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]
