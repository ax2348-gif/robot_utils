"""Real-time scheduling for a control loop's thread.

A control loop that talks to motors must not wait behind ordinary work: the arm's motors drop out
if no command arrives for 100 ms (their own CAN watchdog), and on the robot's 4 cores a burst of
image processing in another node was enough to hold the arm loop for 105 ms. ``make_realtime()``,
called by the loop's thread on itself, asks Linux for SCHED_FIFO so the loop runs before every
normal thread on the machine.

It never raises. Without the permission (RLIMIT_RTPRIO is 0 by default: see docs/realtime.md), on
another OS, or with ``ROBOT_UTILS_RT=0``, the loop runs as before and the returned record, which
each node shows in its state as ``realtime``, says why.

What it does not do: the loop shares its Python process with the HTTP handlers, so a handler that
holds the interpreter lock in one long C call still delays it.
"""
from __future__ import annotations

import logging
import os
from typing import Optional

from pydantic import BaseModel, Field

log = logging.getLogger(__name__)

# Below the kernel's threaded interrupt handlers (50), so USB and CAN interrupts still run first.
PRIO_BUS = 40        # loops that own a motor bus (arm, swerve): a late tick trips a motor watchdog
PRIO_ESTIMATE = 30   # the estimator: the controllers above it wait on its poses
PRIO_CONTROL = 20    # loops that command another node over HTTP (pose_servo)


class RealtimeOut(BaseModel):
    """How the control loop's thread is scheduled (``realtime`` in a node's state)."""
    policy: str = Field(description="`SCHED_FIFO` (real time: the loop runs before every normal thread) or "
                                    "`SCHED_OTHER` (the default time sharing: other work can delay the loop).")
    priority: int = Field(description="Real-time priority, 1-99 (0 with `SCHED_OTHER`).")
    requested: int = Field(description="The priority the loop asked for (0: none, real time is switched off).")
    error: Optional[str] = Field(description="Why the request was not granted; null if it was, or before the loop "
                                             "has started.")
    cpus: Optional[list[int]] = Field(None, description="CPUs the loop's thread is pinned to (null: not pinned, "
                                                        "it runs wherever the scheduler puts it).")


def not_started(requested: int) -> dict:
    """The record before the loop's thread has run."""
    return {"policy": "SCHED_OTHER", "priority": 0, "requested": requested, "error": None, "cpus": None}


def make_realtime(priority: int, name: str = "control loop", cpus: Optional[set] = None) -> dict:
    """Give the CALLING thread SCHED_FIFO at ``priority``; returns a ``RealtimeOut`` record.

    Call it first thing in the loop's thread. The policy is reset in anything the thread starts
    (SCHED_RESET_ON_FORK), so a planner process or a helper thread is not real time too.

    ``cpus``: also pin the calling thread to these CPUs (independent of real time; a failure is
    logged and reported in ``error``, the loop runs unpinned). Unlike the policy, the affinity IS
    inherited by threads the loop's thread starts (e.g. motorbridge's bus poller). Pin a SCHED_FIFO
    loop only to CPUs no other real-time loop runs on: a FIFO thread is never preempted by another
    of the same priority, and a pinned one cannot move away from it.
    """
    out = not_started(priority)
    pin_error = _pin(cpus, name, out) if cpus else None
    out = _make_realtime(priority, name, out)
    if pin_error is not None:
        out["error"] = pin_error if out["error"] is None else f"{out['error']}; {pin_error}"
    return out


def _pin(cpus: set, name: str, out: dict) -> Optional[str]:
    if not hasattr(os, "sched_setaffinity"):
        return "CPU pinning not available on this OS"
    try:
        os.sched_setaffinity(0, set(cpus))                                # 0: the calling thread
        out["cpus"] = sorted(os.sched_getaffinity(0))
        log.info("%s: pinned to CPUs %s", name, out["cpus"])
        return None
    except (OSError, ValueError) as e:
        log.warning("%s: not pinned to CPUs %s (%s)", name, sorted(cpus), e)
        return f"not pinned to CPUs {sorted(cpus)}: {type(e).__name__}: {e}"


def _make_realtime(priority: int, name: str, out: dict) -> dict:
    if os.environ.get("ROBOT_UTILS_RT", "1").strip().lower() in ("0", "off", "false", "no"):
        out.update(requested=0, error="switched off (ROBOT_UTILS_RT=0)")
        return out
    if not hasattr(os, "sched_setscheduler") or not hasattr(os, "SCHED_FIFO"):
        out["error"] = "not available on this OS"
        return out
    try:
        policy = os.SCHED_FIFO | getattr(os, "SCHED_RESET_ON_FORK", 0)
        os.sched_setscheduler(0, policy, os.sched_param(priority))       # 0: the calling thread
        out.update(policy="SCHED_FIFO", priority=priority)
        log.info("%s: real time (SCHED_FIFO %d)", name, priority)
    except PermissionError:
        out["error"] = ("not permitted: this user's real-time priority limit (RLIMIT_RTPRIO) is below "
                        f"{priority}; see docs/realtime.md")
        log.warning("%s: no real-time priority (%s)", name, out["error"])
    except (OSError, ValueError) as e:
        out["error"] = f"{type(e).__name__}: {e}"
        log.warning("%s: no real-time priority (%s)", name, out["error"])
    return out


def describe(rt: dict) -> str:
    """One line for a node's event log."""
    pinned = f", pinned to CPUs {','.join(map(str, rt['cpus']))}" if rt.get("cpus") else ""
    if rt["policy"] == "SCHED_FIFO":
        return f"control loop is real time (SCHED_FIFO {rt['priority']}){pinned}" + (
            f"; {rt['error']}" if rt.get("error") else "")
    return f"control loop is NOT real time: {rt['error']}{pinned}"
