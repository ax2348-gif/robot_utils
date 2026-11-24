"""The thin adapter between this node and the Quest teleop client in ``tools/quest_teleop`` (its README.md).

The client core (``quest_stream``, ``body_frozen``, ``wbc_client``, ``teleop``) and the headset control
(``quest_ctl``) still live in the repository's ``tools/`` folder; they are imported from there, unchanged, as the
top-level package ``quest_teleop`` (the name the tests use too). The one difference here: target datagrams go to the
WBC's **unix datagram socket** (``POST /session`` names it), not UDP, since the WBC runs on the same machine.
"""
from __future__ import annotations

import os
import socket
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[4] / "tools"

try:
    import quest_teleop  # noqa: F401
except ImportError:
    if not (TOOLS / "quest_teleop").is_dir():
        raise ImportError(f"the Quest teleop client is not at {TOOLS / 'quest_teleop'}: this node runs from a source "
                          "checkout (pip install -e .)") from None
    sys.path.insert(0, str(TOOLS))

from quest_teleop import quest_ctl  # noqa: E402,F401
from quest_teleop.arming import x_pressed  # noqa: E402,F401
from quest_teleop.body_frozen import Config, set_top_speed  # noqa: E402,F401
from quest_teleop.episodes import EpisodeButtons  # noqa: E402,F401
from quest_teleop.quest_stream import DEFAULT_PORT, MAX_LATENCY_S, MIN_RATE_HZ, STALE_S, QuestReader  # noqa: E402,F401
from quest_teleop.teleop import TeleopClient, assert_caps, close_wbc, open_wbc, top_speed_caps  # noqa: E402,F401
from quest_teleop.wbc_client import WbcClient, WbcStream  # noqa: E402,F401


class LocalWbcClient(WbcClient):
    """``WbcClient`` whose datagrams go to the WBC's unix socket when it exists on this machine (else UDP to the
    WBC's host at the session's port, as the CLI does). ``transport`` says which."""

    transport: str = ""
    on_sent = None          # called with (t_send, datagram) after each datagram (the node's operator stream)

    def _send(self, msg: dict) -> dict:
        out = super()._send(msg)
        cb = self.on_sent
        if cb is not None:
            cb(time.monotonic(), out)
        return out

    def open_session(self) -> dict:
        # not super(): its int(udp_port) fails for a WBC without UDP (udp_port null)
        out = self.http.post("/session", {})
        path, port = out.get("unix_socket"), out.get("udp_port")
        if path and os.path.exists(path):
            self._udp.close()
            self._udp = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._addr = path
            self.transport = f"unix {path}"
        elif port:
            self._addr = (self.udp_host, int(port))
            self.transport = f"udp {self.udp_host}:{port}"
        else:
            raise RuntimeError("the WBC session offers no datagram socket (no unix socket here, no UDP port)")
        self.session = int(out["session"])
        self.udp_port = None if port is None else int(port)
        self.seq = 0
        self.arm_seq_floor = int(out.get("arm_seq") or 0)
        return out
