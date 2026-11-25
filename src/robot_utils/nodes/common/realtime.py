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
import sys
from typing import Optional

from pydantic import BaseModel, Field

log = logging.getLogger(__name__)

# Below the kernel's threaded interrupt handlers (50), so USB and CAN interrupts still run first.
PRIO_BUS = 40        # loops that own a motor bus (arm, swerve): a late tick trips a motor watchdog
PRIO_BUS_IO = 39     # a bus driver's own native reader thread (motorbridge's polling worker): just below its loop
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
    bus_threads: Optional[list[dict]] = Field(None, description="The bus driver's own native threads started when the "
                                                                "bus connected (the arm: motorbridge's polling worker), "
                                                                "each `{tid, name, policy, priority, error}`: raised to "
                                                                "real time just below the loop; null before a connect "
                                                                "or where not applicable (macOS, a simulated bus).")


def not_started(requested: int) -> dict:
    """The record before the loop's thread has run."""
    return {"policy": "SCHED_OTHER", "priority": 0, "requested": requested, "error": None}


def make_realtime(priority: int, name: str = "control loop") -> dict:
    """Give the CALLING thread SCHED_FIFO at ``priority``; returns a ``RealtimeOut`` record.

    Call it first thing in the loop's thread. The policy is reset in anything the thread starts
    (SCHED_RESET_ON_FORK), so a planner process or a helper thread is not real time too.
    """
    out = not_started(priority)
    if os.environ.get("ROBOT_UTILS_RT", "1").strip().lower() in ("0", "off", "false", "no"):
        out.update(requested=0, error="switched off (ROBOT_UTILS_RT=0)")
        return out
    if not hasattr(os, "sched_setscheduler") or not hasattr(os, "SCHED_FIFO"):
        out["error"] = "not available on this OS"
        if sys.platform == "darwin":                 # the dev Mac (sim): the closest macOS offers, see _macos_latency
            out["error"] += f" (macOS: {_macos_latency(name)})"
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


_MAC_ACTIVITY = None   # the process's NSProcessInfo activity (held for its lifetime)


def _macos_latency(name: str) -> str:
    """macOS has no SCHED_FIFO for us. Ask for what it offers, process-locally, no permissions needed: the calling
    thread's QoS class user-interactive, and (once per process) an NSProcessInfo activity "user initiated + latency
    critical", which turns off App Nap and timer coalescing: without it a node whose terminal or browser window is
    out of view gets its timers delayed by ~100 ms (the sim arm's 0.1 s motor watchdog then trips)."""
    global _MAC_ACTIVITY
    import ctypes
    import ctypes.util
    notes = []
    try:
        libsys = ctypes.CDLL("/usr/lib/libSystem.B.dylib")
        rc = libsys.pthread_set_qos_class_self_np(ctypes.c_uint(0x21), ctypes.c_int(0))   # QOS_CLASS_USER_INTERACTIVE
        notes.append("thread QoS user-interactive" if rc == 0 else f"thread QoS refused ({rc})")
    except Exception as e:  # noqa: BLE001 - best effort, never raises
        notes.append(f"thread QoS: {type(e).__name__}")
    if _MAC_ACTIVITY is None:
        try:
            objc = ctypes.CDLL(ctypes.util.find_library("objc"))
            ctypes.CDLL(ctypes.util.find_library("Foundation"))
            objc.objc_getClass.restype = ctypes.c_void_p
            objc.objc_getClass.argtypes = [ctypes.c_char_p]
            objc.sel_registerName.restype = ctypes.c_void_p
            objc.sel_registerName.argtypes = [ctypes.c_char_p]
            send = objc.objc_msgSend

            def msg(obj, sel, *args, argtypes=()):
                send.restype = ctypes.c_void_p
                send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, *argtypes]
                return send(obj, objc.sel_registerName(sel), *args)
            info = msg(objc.objc_getClass(b"NSProcessInfo"), b"processInfo")
            reason = msg(objc.objc_getClass(b"NSString"), b"stringWithUTF8String:", f"robot_utils {name}".encode(),
                         argtypes=(ctypes.c_char_p,))
            options = 0x00FFFFFF | 0xFF00000000           # NSActivityUserInitiated | NSActivityLatencyCritical
            act = msg(info, b"beginActivityWithOptions:reason:", ctypes.c_uint64(options), reason,
                      argtypes=(ctypes.c_uint64, ctypes.c_void_p))
            if not act:
                raise RuntimeError("no activity")
            msg(act, b"retain")
            _MAC_ACTIVITY = act
        except Exception as e:  # noqa: BLE001
            notes.append(f"App Nap not disabled ({type(e).__name__}: {e})")
            return ", ".join(notes)
    notes.append("App Nap and timer coalescing off")
    return ", ".join(notes)


def task_ids() -> Optional[set]:
    """This process's thread ids (Linux ``/proc/self/task``), or None where there is no such listing (macOS)."""
    try:
        return {int(t) for t in os.listdir("/proc/self/task")}
    except (OSError, ValueError):
        return None


def _policy_name(tid: int) -> str:
    try:
        pol = os.sched_getscheduler(tid) & ~getattr(os, "SCHED_RESET_ON_FORK", 0)
    except OSError as e:
        return f"unknown ({e.strerror})"
    return {getattr(os, "SCHED_FIFO", -1): "SCHED_FIFO", getattr(os, "SCHED_RR", -1): "SCHED_RR",
            getattr(os, "SCHED_OTHER", -2): "SCHED_OTHER"}.get(pol, str(pol))


def _thread_name(tid: int) -> str:
    try:
        with open(f"/proc/self/task/{tid}/comm") as f:
            return f.read().strip()
    except OSError:
        return "?"


def raise_native_threads(before: Optional[set], priority: int = PRIO_BUS_IO, what: str = "bus reader") -> dict:
    """Give SCHED_FIFO ``priority`` to the native thread a driver started since ``before`` (``task_ids()`` taken
    just before the bus was opened): the threads now in this process that were not there then and are not Python
    threads (``threading``'s native ids). Exactly one is expected; none or several: nothing is changed (several could
    be anything another thread of this process started meanwhile). Never raises.

    Returns ``{"threads": [{tid, name, policy, priority, error}], "error": why nothing was set or None,
    "level": "info" | "warn"}``."""
    import threading
    out = {"threads": [], "error": None, "level": "warn"}
    try:
        if os.environ.get("ROBOT_UTILS_RT", "1").strip().lower() in ("0", "off", "false", "no"):
            out["error"] = "real time switched off (ROBOT_UTILS_RT=0)"
            return out
        now = task_ids()
        if before is None or now is None or not hasattr(os, "SCHED_FIFO"):
            out["error"] = "not available on this OS"
            return out
        py = {t.native_id for t in threading.enumerate()}
        new = sorted(now - before - py)
        out["threads"] = [{"tid": t, "name": _thread_name(t), "policy": _policy_name(t), "priority": 0,
                           "error": None} for t in new]
        if len(new) != 1:
            out["error"] = (f"no new native {what} thread found" if not new else
                            f"{len(new)} new native threads ({', '.join(map(str, new))}): not knowing which is the "
                            f"{what}, none changed")
            return out
        th = out["threads"][0]
        try:
            os.sched_setscheduler(new[0], os.SCHED_FIFO | getattr(os, "SCHED_RESET_ON_FORK", 0),
                                  os.sched_param(priority))
            th["priority"] = priority
            out["level"] = "info"
        except PermissionError:
            th["error"] = out["error"] = (f"not permitted: this user's real-time priority limit (RLIMIT_RTPRIO) is "
                                          f"below {priority}; see docs/realtime.md")
        except (OSError, ValueError) as e:
            th["error"] = out["error"] = f"{type(e).__name__}: {e}"
        th["policy"] = _policy_name(new[0])
    except Exception as e:  # noqa: BLE001 - never into connect
        out["error"] = f"{type(e).__name__}: {e}"
    return out


def describe_threads(rec: dict, what: str = "bus reader") -> str:
    """One line for a node's event log about ``raise_native_threads``' result."""
    ths = ", ".join(f"tid {t['tid']} ({t['name']}) {t['policy']}" + (f" {t['priority']}" if t["priority"] else "")
                    for t in rec["threads"]) or "none"
    if rec["error"] is None:
        return f"{what} thread is real time: {ths}"
    return f"{what} thread NOT made real time ({rec['error']}); new threads: {ths}"


def describe(rt: dict) -> str:
    """One line for a node's event log."""
    if rt["policy"] == "SCHED_FIFO":
        return f"control loop is real time (SCHED_FIFO {rt['priority']})"
    return f"control loop is NOT real time: {rt['error']}"
