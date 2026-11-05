"""The interface a robot implements to be teleoperated: ``TeleopApp`` and what it is given.

A robot adapter subclasses ``TeleopApp`` and fills in a handful of coarse callbacks; the
``TeleopRunner`` owns the loop, the input contract, the clutches, faults and the e-stop,
and calls them in this order::

    start ─┬─> [hold_step]* ─> engage ─> [active_step]* ─> disengage ─> [hold_step]* ...
           └─────────────────────────────────────────────────────────────────────────> stop

``stop`` always runs exactly once after ``start`` was entered, whatever ended the run
(a request, a fault, a signal, an exception in ``start`` itself). ``emergency_stop`` runs
on whichever thread tripped the stop, possibly while another callback is still running.

Nothing here knows what the robot is. The inputs are generic channels: per-controller
clutched pose deltas and an analog trigger (``Inputs.hands``), and a planar velocity
request (``Inputs.twist``). Mapping them to joints, end effectors, tools or a base, and
deciding what "hold" means, is the adapter's job.
"""
from __future__ import annotations

import enum
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Mapping, Optional

import numpy as np

from .clutch import HandState

if TYPE_CHECKING:
    from ..quest.messages import QuestFrame
    from .estop import EStop


class FaultAction(enum.Enum):
    STOP = "stop"       # end the run (stop() is called)
    HOLD = "hold"       # leave the active state (disengage()), require every hand to re-grip
    IGNORE = "ignore"   # carry on with the next tick


class Overrun(RuntimeError):
    """A tick finished later than the runner's ``overrun_fault_s`` past its deadline."""


@dataclass(frozen=True)
class Fault:
    phase: str          # "start", "engage", "active_step", "hold_step", "disengage", "wants_active", "input", "timing"
    error: BaseException
    tick: int
    consecutive: int    # faults in a row including this one; reset by a clean tick

    def __str__(self) -> str:
        return f"{self.phase}: {type(self.error).__name__}: {self.error}"


# RunResult.kind values
REQUESTED = "requested"   # ctx.request_stop() or runner.request_stop()
ESTOP = "estop"           # signal, KeyboardInterrupt or estop.trip()
FAULT = "fault"


@dataclass
class Stats:
    ticks: int = 0
    active_ticks: int = 0
    live_ticks: int = 0
    engagements: int = 0
    faults: int = 0
    overruns: int = 0         # ticks that finished after their deadline
    max_tick_s: float = 0.0   # longest callback work in one tick
    max_late_s: float = 0.0   # worst finish past deadline


@dataclass(frozen=True)
class RunResult:
    kind: str
    reason: str
    fault: Optional[Fault] = None
    stop_error: Optional[BaseException] = None   # raised by app.stop()
    stats: Stats = field(default_factory=Stats)

    @property
    def ok(self) -> bool:
        return self.kind == REQUESTED and self.stop_error is None


@dataclass(frozen=True)
class Inputs:
    """Everything the operator asks for on one tick, already safe to act on.

    ``live`` is False when there is no fresh, focused intent (``status`` says why); then no
    hand is engaged, triggers are None and ``twist`` is zero. ``frame`` is the raw parsed
    record (for buttons etc.) when fresh, else None.
    """

    tick: int
    t: float
    live: bool
    status: str             # "ok", "no-frame", "stale", "unfocused", "no-intent"
    age_s: float            # since the newest frame arrived (inf if none)
    hands: Mapping[str, HandState]
    twist: np.ndarray = field(repr=False)   # [forward m/s, left m/s, yaw rad/s] in the operator frame
    frame: Optional["QuestFrame"] = field(default=None, repr=False)

    @property
    def any_engaged(self) -> bool:
        return any(h.engaged for h in self.hands.values())


class Context:
    """Handed to every callback: loop timing, statistics and the controls an app may use."""

    def __init__(self, runner, rate_hz: float, estop: "EStop", log: logging.Logger):
        self._runner = runner
        self.rate_hz = rate_hz
        self.period_s = 1.0 / rate_hz
        self.estop = estop
        self.log = log
        self.stats = Stats()
        self.tick = 0
        self.active = False

    def request_stop(self, reason: str = "requested") -> None:
        """End the run after the current tick (stop() is called)."""
        self._runner.request_stop(reason)

    def require_release(self, side: Optional[str] = None) -> None:
        """Make one hand (default: all) release and grip again before driving anything."""
        self._runner.require_release(side)


class TeleopApp:
    """Subclass and override. Only ``active_step`` and ``hold_step`` are required."""

    #: loop rate; keep it well above whatever the hardware's command watchdog needs
    rate_hz: float = 100.0

    def start(self, ctx: Context) -> None:
        """Acquire and prepare the hardware, leaving it safe (holding / not driving)."""

    def engage(self, ctx: Context, inputs: Inputs) -> None:
        """Idle -> active: e.g. enable outputs, seed commands from the measured state."""

    def active_step(self, ctx: Context, inputs: Inputs) -> None:
        """One tick while active: read state, follow the targets, send commands."""
        raise NotImplementedError

    def disengage(self, ctx: Context, inputs: Inputs) -> None:
        """Active -> idle (release, input loss, HOLD fault): bring the robot to a hold.
        Also called after an ``engage`` that failed part-way, so it must tolerate that."""

    def hold_step(self, ctx: Context, inputs: Inputs) -> None:
        """One tick while idle: keep the robot safe (hold position, zero velocity, ...).
        Runs every tick, so it is also where a robot keeps its watchdogs fed."""
        raise NotImplementedError

    def stop(self, ctx: Context, result: RunResult) -> None:
        """Release the hardware. Always called once; after an e-stop (``result.kind ==
        "estop"``) prefer not to command anything beyond making it safe."""

    def wants_active(self, ctx: Context, inputs: Inputs) -> bool:
        """Whether this tick should be active; only asked while inputs are live."""
        return inputs.any_engaged

    def on_fault(self, ctx: Context, fault: Fault) -> FaultAction:
        """Decide what a failed callback or an overrun means. ``start`` and ``disengage``
        faults always stop, and so does reaching the runner's consecutive-fault limit."""
        return FaultAction.STOP

    def emergency_stop(self, reason: str) -> None:
        """Immediate, thread-safe reaction to an e-stop (signal or ``estop.trip``), called
        on the tripping thread. The loop has already stopped commanding; keep this short."""
