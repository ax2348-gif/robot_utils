"""Planning and heavy kinematics in a separate process.

The control thread shares the node's Python interpreter with request threads. A servo plan
(hundreds of IK solves) or a stream of IK / kinematics lookups holds the interpreter for long
enough to delay control ticks past the motors' watchdog (measured: 70-120 ms tick gaps on a Mac
during one plan). So that work runs here, in one worker process (a spawned interpreter of its
own), one job at a time; the node only waits for the result in a request thread, which costs the
control thread nothing.

Each job carries what the worker needs to rebuild the same model: the setpoint limits and the
TCP offset (the worker caches its ``ArmKinematics`` per pair). Results and ``PlanError`` pickle.
"""
from __future__ import annotations

import atexit
import multiprocessing as mp
import threading
from concurrent.futures import ProcessPoolExecutor
from typing import Optional

import numpy as np

from .kin import ArmKinematics

JOB_TIMEOUT = 30.0     # s

_kin: Optional[ArmKinematics] = None
_key = None


def _get(limits: dict, tcp: tuple) -> ArmKinematics:
    global _kin, _key
    key = (tuple(sorted((k, tuple(v)) for k, v in limits.items())), tuple(tcp))
    if _kin is None or key != _key:
        _kin, _key = ArmKinematics(limits, tcp_offset=tcp), key
    return _kin


def _job(kind: str, limits: dict, tcp: tuple, args: tuple, kwargs: dict):
    from . import motion
    kin = _get(limits, tcp)
    if kind == "servo":
        return motion.plan_servo(kin, *args, **kwargs)
    if kind == "joint":
        return motion.plan_joint(*args, kin=kin, **kwargs)
    if kind == "chunk":
        return motion.plan_chunk(kin, *args, **kwargs)
    if kind == "ik":
        return kin.ik(*args, **kwargs)
    if kind == "kinematics":
        frame = kwargs.get("frame", "base")
        out = []
        for q in args[0]:
            from .kin import pose_public
            J = kin.jacobian(q, frame)
            out.append({"q": [float(v) for v in q], "within_limits": kin.within_limits(q),
                        "tcp": pose_public(kin.fk(q)), "frame": frame, "jacobian": J.round(9).tolist(),
                        **kin.manipulability(J)})
        return out
    raise ValueError(f"unknown job {kind!r}")


class Planner:
    """One worker process, shared by every node in this process (jobs carry their own model key)."""
    _pool: Optional[ProcessPoolExecutor] = None
    _lock = threading.Lock()

    @classmethod
    def pool(cls) -> ProcessPoolExecutor:
        with cls._lock:
            if cls._pool is None:
                cls._pool = ProcessPoolExecutor(max_workers=1, mp_context=mp.get_context("spawn"))
                atexit.register(cls.shutdown)
            return cls._pool

    @classmethod
    def shutdown(cls) -> None:
        with cls._lock:
            if cls._pool is not None:
                cls._pool.shutdown(wait=False, cancel_futures=True)
                cls._pool = None

    @classmethod
    def warm(cls, kin: ArmKinematics) -> None:
        """Start the worker and build its model in the background (the first job otherwise
        waits a couple of seconds for the interpreter and pinocchio)."""
        limits = {n: (float(lo), float(hi)) for n, lo, hi in zip(kin.joint_names, kin.lo, kin.hi)}
        cls.pool().submit(_job, "ik", limits, tuple(kin.tcp_offset), (kin.fk(np.zeros(len(kin.lo))), np.zeros(len(kin.lo))), {})

    @classmethod
    def run(cls, job: str, kin: ArmKinematics, *args, **kwargs):
        limits = {n: (float(lo), float(hi)) for n, lo, hi in zip(kin.joint_names, kin.lo, kin.hi)}
        fut = cls.pool().submit(_job, job, limits, tuple(kin.tcp_offset), args, kwargs)
        return fut.result(timeout=JOB_TIMEOUT)
