"""``TeleopRunner``: the fixed-rate loop that drives a ``TeleopApp`` from Quest input.

Per tick it reads the newest frame from the source, applies the consumer contract
(freshness on this machine's clock, focus, per-hand validity, session and operator-frame
changes, release before engage) through one ``HandClutch`` per controller, decides
active/idle, and calls exactly one step callback. Around that:

- **timing**: deadlines advance by one period (no drift); a late tick is counted, and one
  later than ``overrun_fault_s`` becomes a ``timing`` fault. After falling behind by more
  than a period the schedule restarts from now instead of bursting to catch up.
- **faults**: an exception from a callback (or an overrun) goes to ``app.on_fault``;
  STOP ends the run, HOLD disengages and makes every hand re-grip, IGNORE continues. The
  rest of the faulting tick is skipped. ``start``/``disengage`` faults, and
  ``max_consecutive_faults`` faults in a row, always stop.
- **e-stop**: with ``install_signals`` (main thread only) SIGINT/SIGTERM/... trip the
  ``EStop`` from a watcher thread, which calls ``app.emergency_stop`` at once; the loop
  wakes, starts no further step and calls ``stop``. A second signal, or a ``stop`` that
  hangs for ``estop_grace_s``, hard-exits the process.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import replace
from typing import Callable, Iterable, Optional, Protocol

import numpy as np

from ..quest.messages import SIDES, QuestFrame
from .app import (ESTOP, FAULT, REQUESTED, Context, Fault, FaultAction, Inputs, Overrun,
                  RunResult, TeleopApp)
from .clutch import HandClutch
from .estop import EStop

log = logging.getLogger("robot_utils.teleop")

_ZERO3 = np.zeros(3)
_ZERO3.flags.writeable = False


class FrameSource(Protocol):
    def latest(self) -> Optional[QuestFrame]: ...


class _Stop(Exception):
    def __init__(self, kind: str, reason: str, fault: Optional[Fault] = None):
        super().__init__(reason)
        self.kind, self.reason, self.fault = kind, reason, fault


class TeleopRunner:
    def __init__(self, app: TeleopApp, source: FrameSource, *,
                 rate_hz: Optional[float] = None,
                 input_timeout_s: float = 0.2,
                 sides: Iterable[str] = SIDES,
                 max_consecutive_faults: int = 3,
                 overrun_fault_s: Optional[float] = 0.05,
                 install_signals: bool = True,
                 estop_grace_s: Optional[float] = 3.0,
                 estop: Optional[EStop] = None,
                 clock: Callable[[], float] = time.monotonic,
                 logger: logging.Logger = log):
        self.app = app
        self.source = source
        self.rate_hz = float(rate_hz if rate_hz is not None else app.rate_hz)
        if not self.rate_hz > 0:
            raise ValueError("rate_hz must be positive")
        self.input_timeout_s = input_timeout_s
        self.max_consecutive_faults = max_consecutive_faults
        self.overrun_fault_s = overrun_fault_s
        self.install_signals = install_signals
        self.estop = estop if estop is not None else EStop(grace_s=estop_grace_s)
        self._clock = clock
        self.log = logger
        self.clutches = {side: HandClutch(side) for side in sides}
        self.ctx = Context(self, self.rate_hz, self.estop, logger)
        self._stop_request: Optional[str] = None
        self._consecutive = 0
        self._ran = False
        self._last_status: Optional[str] = None

    # -- controls (any thread) ---------------------------------------------

    def request_stop(self, reason: str = "requested") -> None:
        if self._stop_request is None:
            self._stop_request = reason

    def require_release(self, side: Optional[str] = None) -> None:
        for s, c in self.clutches.items():
            if side is None or s == side:
                c.disarm()

    # -- run ---------------------------------------------------------------

    def run(self) -> RunResult:
        """Run until stopped; returns why. Not re-entrant; one run per runner."""
        if self._ran:
            raise RuntimeError("a TeleopRunner runs once")
        self._ran = True
        ctx, app, estop = self.ctx, self.app, self.estop
        installed_here = False
        if self.install_signals and not estop.installed:
            if threading.current_thread() is threading.main_thread():
                estop.install()
                installed_here = True
            else:
                self.log.warning("not on the main thread: signal e-stop not installed")
        estop.on_trip(self._on_trip)

        stop = _Stop(REQUESTED, "requested")
        try:
            try:
                app.start(ctx)
            except (KeyboardInterrupt, _Stop):
                raise
            except Exception as e:
                f = self._record_fault("start", e)
                raise _Stop(FAULT, f"start failed: {f}", f) from e
            self._loop()
        except _Stop as s:
            stop = s
        except KeyboardInterrupt:
            if estop.installed:
                estop.wait(0.1)   # a signal's watcher trip may land just after the handler raised
            estop.trip("KeyboardInterrupt")
            stop = _Stop(ESTOP, estop.reason or "KeyboardInterrupt")
        except Exception as e:   # a bug in the runner, or in a fault handler path
            self.log.exception("teleop loop failed")
            f = Fault("runner", e, ctx.tick, self._consecutive)
            stop = _Stop(FAULT, f"runner failed: {f}", f)
        finally:
            estop.quiet()
            if estop.tripped and stop.kind != ESTOP:
                stop = _Stop(ESTOP, estop.reason or "e-stop", stop.fault)
            result = RunResult(stop.kind, stop.reason, stop.fault, stats=ctx.stats)
            self.log.info("stopping (%s: %s)", result.kind, result.reason)
            try:
                app.stop(ctx, result)
            except BaseException as e:   # incl. a late KeyboardInterrupt
                self.log.exception("app.stop failed")
                result = replace(result, stop_error=e)
            if installed_here:
                estop.uninstall()
        return result

    def _on_trip(self, reason: str) -> None:
        self.log.warning("E-STOP: %s", reason)
        try:
            self.app.emergency_stop(reason)
        except Exception:
            self.log.exception("app.emergency_stop failed")

    def _loop(self) -> None:
        ctx, stats, period = self.ctx, self.ctx.stats, 1.0 / self.rate_hz
        deadline = self._clock()
        while True:
            # Sleep to the tick, waking early on an e-stop.
            delay = deadline - self._clock()
            if delay > 0:
                self.estop.wait(delay)
            self._check_stop()
            t0 = self._clock()
            deadline += period
            ctx.tick += 1
            stats.ticks += 1

            if self._tick(t0):
                self._consecutive = 0

            t1 = self._clock()
            stats.max_tick_s = max(stats.max_tick_s, t1 - t0)
            late = t1 - deadline
            if late > 0:
                stats.overruns += 1
                stats.max_late_s = max(stats.max_late_s, late)
                if late > period:
                    deadline = t1   # fell a whole period behind: restart the schedule
                if self.overrun_fault_s is not None and late > self.overrun_fault_s:
                    self._fault("timing", Overrun(f"tick {ctx.tick} finished {late * 1e3:.1f} ms late"))

    def _check_stop(self) -> None:
        if self.estop.tripped:
            raise _Stop(ESTOP, self.estop.reason or "e-stop")
        if self._stop_request is not None:
            raise _Stop(REQUESTED, self._stop_request)

    def _tick(self, now: float) -> bool:
        """One tick; True if it ran without a fault."""
        ctx, app = self.ctx, self.app
        try:
            inputs = self._inputs(now)
        except Exception as e:
            self._fault("input", e)
            return False
        if inputs.live:
            ctx.stats.live_ticks += 1
        if inputs.status != self._last_status:
            self.log.info("input %s", inputs.status)
            self._last_status = inputs.status

        phase = "wants_active"
        try:
            want = inputs.live and bool(app.wants_active(ctx, inputs))
            if want and not ctx.active:
                phase = "engage"
                self._check_stop()
                ctx.active = True   # set first: a failed engage gets a disengage
                ctx.stats.engagements += 1
                self.log.info("engage")
                app.engage(ctx, inputs)
            elif not want and ctx.active:
                phase = "disengage"
                self._disengage(inputs)
            self._check_stop()
            if ctx.active:
                phase = "active_step"
                ctx.stats.active_ticks += 1
                app.active_step(ctx, inputs)
            else:
                phase = "hold_step"
                app.hold_step(ctx, inputs)
            return True
        except _Stop:
            raise
        except Exception as e:
            self._fault(phase, e, inputs)
            return False

    def _disengage(self, inputs: Inputs) -> None:
        self.log.info("disengage")
        self.ctx.active = False
        try:
            self.app.disengage(self.ctx, inputs)
        except Exception as e:
            f = self._record_fault("disengage", e)
            raise _Stop(FAULT, f"disengage failed: {f}", f) from e

    def _inputs(self, now: float) -> Inputs:
        frame = self.source.latest()
        age = float("inf") if frame is None else now - frame.received_at
        if frame is None:
            status = "no-frame"
        elif age > self.input_timeout_s:
            status = "stale"
        elif not frame.input_focused:
            status = "unfocused"
        elif frame.intent is None:
            status = "no-intent"
        else:
            status = "ok"
        live = status == "ok"
        intent = frame.intent if live else None
        key = (intent.session_id, intent.operator_frame_revision) if intent else None
        hands = {side: c.update(intent.hands.get(side) if intent else None, key)
                 for side, c in self.clutches.items()}
        twist = intent.twist if intent is not None and intent.twist_valid else _ZERO3
        return Inputs(tick=self.ctx.tick, t=now, live=live, status=status, age_s=age, hands=hands,
                      twist=twist, frame=frame if age <= self.input_timeout_s else None)

    # -- faults ------------------------------------------------------------

    def _record_fault(self, phase: str, error: BaseException) -> Fault:
        self._consecutive += 1
        self.ctx.stats.faults += 1
        f = Fault(phase, error, self.ctx.tick, self._consecutive)
        if isinstance(error, Overrun):
            self.log.warning("fault %s", f)
        else:
            self.log.error("fault %s", f, exc_info=error)
        return f

    def _fault(self, phase: str, error: BaseException, inputs: Optional[Inputs] = None) -> None:
        f = self._record_fault(phase, error)
        try:
            action = self.app.on_fault(self.ctx, f)
        except Exception:
            self.log.exception("app.on_fault failed; stopping")
            action = FaultAction.STOP
        if not isinstance(action, FaultAction):
            self.log.error("app.on_fault returned %r; stopping", action)
            action = FaultAction.STOP
        if f.consecutive >= self.max_consecutive_faults:
            raise _Stop(FAULT, f"{f.consecutive} consecutive faults, last {f}", f)
        if action is FaultAction.STOP:
            raise _Stop(FAULT, str(f), f)
        if action is FaultAction.HOLD:
            self.require_release()
            if self.ctx.active:
                self._disengage(inputs if inputs is not None else self._no_inputs())

    def _no_inputs(self) -> Inputs:
        """Inputs with nothing live, for a disengage that has no tick inputs to hand over."""
        hands = {side: c.update(None) for side, c in self.clutches.items()}
        return Inputs(tick=self.ctx.tick, t=self._clock(), live=False, status="no-frame",
                      age_s=float("inf"), hands=hands, twist=_ZERO3)
