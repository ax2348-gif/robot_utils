"""Between the WBC's two processes (docs/wbc.md "Processes"): the shared-memory layout and the cells of
``common/ipc.py`` (slots, rings, beats, the command ports)."""
from __future__ import annotations

from ..common.ipc import ALIGN, Beats, Port, Ring, SharedFile, Slot  # noqa: F401  (re-exported)
from ..common.ipc import runtime_dir as _runtime_dir

# Beats cells
SRV_HB = 0        # the server process's heartbeat (time.monotonic, written every few ms)
CTL_HB = 1        # the control process's heartbeat (every tick)
RX_COUNT = 2      # datagrams of the session in force (each renews the client lease: the server renews it)
CTL_PID = 3
N_BEATS = 16

# the estimator and swerve rows the server forwards to the control process (the fields the control tick uses)
EST_COLS = ("t", "x", "y", "theta", "reset_id", "imu_ok", "odom_ok")
SW_COLS = ("t", "enabled", "estop", "lease_id")


def runtime_dir() -> str:
    return _runtime_dir("wbc-")


class Shared(SharedFile):
    """The WBC's shared-memory file: beats, the slots and the rings."""

    def __init__(self, path: str, width: int, create: bool):
        super().__init__(path, [("beats", "beats", N_BEATS),
                                ("twist", "slot", 1024),           # control -> server: the base twist command
                                ("srv", "slot", 8192),             # server -> control: lease held, the sender's health
                                ("snap", "slot", 1 << 19),         # control -> server: the state snapshot (/state)
                                ("rows", "ring", (2048, width)),   # control -> server: the stream rows
                                ("est", "ring", (1024, len(EST_COLS))),     # server -> control: estimator rows
                                ("sw", "ring", (256, len(SW_COLS)))],       # server -> control: swerve rows
                         create)
