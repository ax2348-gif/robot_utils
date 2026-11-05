"""Signal-driven emergency stop that does not depend on the main thread.

The safety model assumed here: the hardware stops on its own when commands stop arriving
(e.g. a motor-side CAN watchdog), so the fastest reliable stop is to stop commanding. A
control loop that checks ``EStop.tripped`` on every tick and sends nothing once it is set
achieves that whatever else is happening; ``on_trip`` hooks can add a direct action
(e.g. a disable command) on top.

Python runs signal handlers only in the main thread, between bytecodes, so a plain handler
would be delayed while the main thread is blocked in a native call (a bus read, a driver's
shutdown). Instead the C-level handler writes the signal number to a pipe
(``signal.set_wakeup_fd``) and a watcher thread trips the stop as soon as it arrives. The
Python-level handler then raises KeyboardInterrupt so blocking calls unwind and the main
thread runs its cleanup (``quiet()`` stops that once cleanup has begun).

Escalation (only with ``grace_s`` set): a second signal, or a shutdown still running
``grace_s`` after the stop tripped, ends the process with ``os._exit`` after the
``on_hard_exit`` hooks (e.g. restoring the terminal).
"""
from __future__ import annotations

import logging
import os
import signal
import threading
from typing import Callable, List, Optional

log = logging.getLogger(__name__)

STOP_SIGNALS = tuple(s for s in (getattr(signal, n, None) for n in
                                 ("SIGINT", "SIGTERM", "SIGHUP", "SIGQUIT", "SIGTSTP"))
                     if s is not None)


class EStop:
    def __init__(self, grace_s: Optional[float] = 3.0):
        self.grace_s = grace_s
        self.reason: Optional[str] = None
        self._tripped = threading.Event()
        self._hooks_done = threading.Event()
        self._signals = 0
        self._lock = threading.Lock()
        self._timer: Optional[threading.Timer] = None
        self._trip_hooks: List[Callable[[str], None]] = []
        self._hard_exit_hooks: List[Callable[[], None]] = []
        self._previous: dict = {}
        self._old_wakeup_fd: Optional[int] = None
        self._rfd = self._wfd = -1
        self._installed = False
        self._raise = True

    # -- state -------------------------------------------------------------

    @property
    def tripped(self) -> bool:
        return self._tripped.is_set()

    @property
    def installed(self) -> bool:
        return self._installed

    def wait(self, timeout: Optional[float]) -> bool:
        """Sleep up to ``timeout`` s, returning early (True) if the stop trips."""
        return self._tripped.wait(timeout)

    def trip(self, reason: str) -> None:
        """Stop (idempotent; the first reason is kept). Safe from any thread."""
        with self._lock:
            if self._tripped.is_set():
                return
            self.reason = reason
            self._tripped.set()
            if self.grace_s is not None and self._installed:
                self._timer = threading.Timer(self.grace_s, self._hard_exit,
                                              args=(1, f"shutdown took over {self.grace_s:g} s"))
                self._timer.daemon = True
                self._timer.start()
        try:
            for fn in list(self._trip_hooks):
                try:
                    fn(reason)
                except Exception:
                    log.exception("e-stop hook failed")
        finally:
            self._hooks_done.set()

    def wait_hooks(self, timeout: Optional[float]) -> bool:
        """After a trip, wait for the ``on_trip`` hooks (they run on the tripping thread)."""
        return self._hooks_done.wait(timeout)

    def on_trip(self, fn: Callable[[str], None]) -> None:
        """Call ``fn(reason)`` once when the stop trips, on the tripping thread."""
        self._trip_hooks.append(fn)

    def on_hard_exit(self, fn: Callable[[], None]) -> None:
        self._hard_exit_hooks.append(fn)

    def _hard_exit(self, code: int, why: str) -> None:
        for fn in self._hard_exit_hooks:
            try:
                fn()
            except Exception:
                pass
        try:
            os.write(2, f"\n[e-stop] {why}; exiting now.\n".encode())
        finally:
            os._exit(code)

    # -- signals -----------------------------------------------------------

    def install(self) -> "EStop":
        """Install handlers for STOP_SIGNALS. Must be called from the main thread."""
        if self._installed:
            return self
        self._rfd, self._wfd = os.pipe()
        os.set_blocking(self._wfd, False)
        self._old_wakeup_fd = signal.set_wakeup_fd(self._wfd, warn_on_full_buffer=False)
        for sig in STOP_SIGNALS:
            self._previous[sig] = signal.signal(sig, self._python_handler)
        self._installed = True
        threading.Thread(target=self._watch, args=(self._rfd,), name="estop-watcher",
                         daemon=True).start()
        return self

    def uninstall(self) -> None:
        """Restore the previous handlers and cancel the grace timer (the stop stays tripped)."""
        if not self._installed:
            return
        self._installed = False
        for sig, prev in self._previous.items():
            signal.signal(sig, prev)
        self._previous.clear()
        signal.set_wakeup_fd(self._old_wakeup_fd if self._old_wakeup_fd is not None else -1)
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
        os.close(self._wfd)  # ends the watcher's read

    def _watch(self, rfd: int) -> None:
        try:
            while True:
                try:
                    data = os.read(rfd, 64)
                except OSError:
                    return
                if not data:
                    return
                for signum in data:
                    self._signals += 1
                    try:
                        name = signal.Signals(signum).name
                    except ValueError:
                        name = str(signum)
                    self.trip(f"{name} received")
                    if self._signals >= 2 and self.grace_s is not None:
                        self._hard_exit(130, f"second signal ({name})")
        finally:
            os.close(rfd)

    def quiet(self) -> None:
        """Stop raising KeyboardInterrupt (e.g. during cleanup); signals still trip and escalate."""
        self._raise = False

    def _python_handler(self, signum, frame) -> None:
        # The watcher has already tripped the stop; unwind the main thread.
        if self._raise:
            raise KeyboardInterrupt
