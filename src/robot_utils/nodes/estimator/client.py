"""Python client for the estimator node (needs httpx).

    from robot_utils.nodes.estimator.client import EstimatorClient

    with EstimatorClient("estimator") as est:        # off the robot: EstimatorClient("http://botcore/nodes/estimator")
        t0 = est.pose()["t"]                         # Pi monotonic s
        ...
        rel = est.relative(t0)                       # how the robot moved since t0, in its frame at t0
        print(rel["dx"], rel["dy"], rel["dtheta"])
        p = est.pose_at([t0, t0 + 0.5])              # {"reset_id", "frame", "poses": [...]}

Times are on the Pi's ``time.monotonic()`` (the clock of the stream's ``t``); off the robot,
``est.sync()`` then ``est.clock.to_server(time.monotonic())`` maps a local time to it.
A time the history can't answer raises ``HistoryUnavailable`` (status 404: before the window or
the last reset; 409: not yet) with ``available = {t_start, t_end, reset_id}``.
"""
from __future__ import annotations

from typing import Iterable, Optional, Union

import httpx

from ..common.client import NodeClient, NodeError

Time = Union[float, str]     # Pi monotonic s, or "now"


class HistoryUnavailable(NodeError):
    def __init__(self, status: int, detail: str, available: Optional[dict]):
        super().__init__(status, detail)
        self.available = available


class EstimatorClient(NodeClient):
    def __init__(self, target: str = "estimator", timeout: float = 5.0):
        super().__init__(target, timeout)

    def _check(self, r: httpx.Response):
        if r.status_code in (404, 409):
            try:
                body = r.json()
            except ValueError:
                body = None
            if isinstance(body, dict) and "available" in body:
                raise HistoryUnavailable(r.status_code, str(body.get("detail")), body["available"])
        return super()._check(r)

    def state(self) -> dict:
        return self.get("/state")

    def pose(self) -> Optional[dict]:
        """The latest pose row (as /stream), or None before the first."""
        return self.get("/pose")

    def pose_at(self, ts: Union[Time, Iterable[Time]]) -> dict:
        """Poses at one or more times: ``{"reset_id", "frame", "poses": [...]}`` (all one reset_id)."""
        ts = [ts] if isinstance(ts, (int, float, str)) else list(ts)
        return self.get("/pose_at", t=[str(t) for t in ts])

    def relative(self, t_from: Time, t_to: Time = "now") -> dict:
        """The robot at ``t_to`` in the robot frame at ``t_from``: ``{dx, dy, dtheta, ...}``."""
        return self.get("/relative", **{"from": str(t_from), "to": str(t_to)})

    def history(self) -> dict:
        return self.get("/history")

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> dict:
        """A new world frame at this pose; clears the history. Not while someone relies on the frame."""
        return self.post("/reset", {"x": x, "y": y, "theta": theta})
