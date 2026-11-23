#!/usr/bin/env python3
"""Quest whole-body teleop through the WBC node (runs on the Mac or erid, never the Pi).

    python tools/quest_teleop/teleop.py --quest 100.72.77.83 --wbc http://botcore-desktop.local/nodes/wbc
    python tools/quest_teleop/teleop.py --quest 127.0.0.1 --wbc http://127.0.0.1:8150   # fake headset + sim

Reads the headset's ``teleop.quest_state.v1`` stream (TCP 9391), applies the remote-consumer rules and the
body-frozen scheme (README.md), and drives the WBC: lease, hints, enable, a datagram session; then one datagram per
batch of new headset frames. **Nothing moves until Y (left controller) arms; X disarms and disables** (README
"Arming: Y and X"). Disarmed it sends keep-alives with ``armed: false``; stale, late or invalid input disarms.
Ctrl-C: stop sending, disable, release the lease.
"""
from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

if __package__ in (None, ""):
    _here = Path(__file__).resolve()
    sys.path.insert(0, str(_here.parent.parent))
    try:
        import robot_utils  # noqa: F401
    except ImportError:
        sys.path.insert(0, str(_here.parents[2] / "src"))
    from quest_teleop.arming import ArmLatch, LatchConfig   # noqa: E402
    from quest_teleop.body_frozen import Config, RobotState, TeleopCore   # noqa: E402
    from quest_teleop.quest_stream import (DEFAULT_PORT, MAX_LATENCY_S, MIN_RATE_HZ, STALE_S, QuestGate,   # noqa: E402
                                           QuestReader)
    from quest_teleop.wbc_client import WbcClient, WbcStream   # noqa: E402
else:
    from .arming import ArmLatch, LatchConfig
    from .body_frozen import Config, RobotState, TeleopCore
    from .quest_stream import DEFAULT_PORT, MAX_LATENCY_S, MIN_RATE_HZ, STALE_S, QuestGate, QuestReader
    from .wbc_client import WbcClient, WbcStream


class Actions:
    """The latch's HTTP requests (``enable`` / ``disable`` on the WBC client), one at a time off the loop (a slow
    enable never delays X); ``drain()`` returns the results ``(what, ok, detail)``. ``sync``: run inline (tests)."""

    def __init__(self, wbc, sync: bool = False):
        self.wbc, self.sync = wbc, sync
        self._q: queue.Queue = queue.Queue()
        self._results: queue.Queue = queue.Queue()
        self.busy: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        if not sync:
            self._thread = threading.Thread(target=self._loop, name="quest-wbc-actions", daemon=True)
            self._thread.start()

    def request(self, what: str) -> None:
        if self.sync:
            self._run(what)
        else:
            self._q.put(what)

    def _run(self, what: str) -> None:
        self.busy = what
        try:
            getattr(self.wbc, what)()
            self._results.put((what, True, ""))
        except Exception as e:  # noqa: BLE001 (reported to the latch)
            self._results.put((what, False, str(getattr(e, "detail", None) or e)[:200]))
        finally:
            self.busy = None

    def _loop(self) -> None:
        while True:
            what = self._q.get()
            if what is None:
                return
            self._run(what)

    def drain(self) -> list:
        out = []
        while True:
            try:
                out.append(self._results.get_nowait())
            except queue.Empty:
                return out

    def close(self) -> None:
        if self._thread is not None:
            self._q.put(None)


def default_hints(cfg: Config) -> dict:
    """Session defaults set over HTTP at start; each datagram overrides ``base`` with the live state."""
    return {"base": {"mode": "locked"},
            "ees": {cfg.ee: {"weights": {"position": 1.0, "orientation": 0.3}}}}


class TeleopClient:
    """The loop: headset frames in, one datagram out per batch of new frames; the armed latch (``arming.py``)
    decides whether that is a target (armed) or a keep-alive with ``armed: false``."""

    def __init__(self, cfg: Config, reader: QuestReader, stream: WbcStream, wbc: WbcClient, *,
                 stale_s: float = STALE_S, clock: Callable[[], float] = time.monotonic,
                 max_latency: float = MAX_LATENCY_S, min_rate: float = MIN_RATE_HZ,
                 latch: LatchConfig = LatchConfig(), sync_http: bool = False):
        self.cfg = cfg
        self.reader, self.stream, self.wbc = reader, stream, wbc
        self.gate = QuestGate(stale_s, max_latency=max_latency, min_rate=min_rate)
        self.core = TeleopCore(cfg)
        self.core.on_disarm("not armed yet")       # nothing live until a Y press arms
        self.latch = ArmLatch(latch, getattr(wbc, "arm_seq_floor", 0) or 0)
        self.actions = Actions(wbc, sync_http)
        self.clock = clock
        self.sending = False
        self.sent_log: Optional[list] = None   # tests: every datagram sent
        # observers (the teleop node's event log and episodes; all called on the loop's thread):
        # on_frame(frame, verdict, now, fresh) after the latch and the core saw a frame, before its latch events;
        # on_step(now, fresh, why_stale) once per step before the latch's tick;
        # on_event(event) for each latch event (("armed", arm_seq) / ("disarmed", reason)) and HTTP result
        # (("enable" | "disable", ok, detail))
        self.on_frame: Optional[Callable] = None
        self.on_step: Optional[Callable] = None
        self.on_event: Optional[Callable] = None

    def session_opened(self, info: dict) -> None:
        """After ``POST /session``: number presses above the WBC's arm_seq floor."""
        self.latch.set_floor(int((info or {}).get("arm_seq") or 0))

    def robot(self) -> Optional[RobotState]:
        row, t = self.stream.latest()
        return None if row is None else RobotState.from_row(row, t, self.cfg.ee)

    def _send(self, kind: str, body: Optional[dict] = None) -> Optional[dict]:
        armed, seq = self.latch.armed, self.latch.arm_seq
        try:
            msg = self.wbc.send_alive(armed, seq) if kind == "alive" else self.wbc.send_target(body, armed, seq)
        except (RuntimeError, OSError):          # no session (yet), or the socket is gone
            return None
        if armed and kind == "target":
            self.latch.sent_arm(msg["seq"])
        if self.sent_log is not None:
            self.sent_log.append(msg)
        return msg

    def _latch_events(self, frame: Optional[dict]) -> None:
        """The latch's requests go to the HTTP worker; a disarm is sent at once (``armed: false``) and resets the
        core; an arm resets the core's gates (after the arming frame went through it)."""
        for what in self.latch.requests:
            self.actions.request(what)
        self.latch.requests.clear()
        for ev in self.latch.events:
            if ev[0] == "disarmed":
                self.core.on_disarm(f"disarmed: {ev[1]}")
                self._send("alive")
            else:
                self.core.on_arm(frame)
            if self.on_event is not None:
                self.on_event(ev)
        self.latch.events.clear()
        for what, ok, detail in self.actions.drain():
            if what == "enable":                 # the seq sent so far: a row that reflects the enable comes later
                self.latch.enable_done(ok, detail, seq=getattr(self.wbc, "seq", None))
            else:
                self.latch.disable_done(ok, detail)
            if self.on_event is not None:
                self.on_event((what, ok, detail))

    def step(self, wait: float = 0.05) -> Optional[dict]:
        self._latch_events(None)                 # HTTP results that arrived meanwhile
        first = self.reader.get(timeout=wait)
        frames = ([first] + self.reader.drain()) if first is not None else []
        robot = self.robot()
        row, t_row = self.stream.latest()
        for f in frames:
            v = self.gate.check(f)
            now = self.clock()
            self.latch.on_frame(f.data, v, row, t_row, now, self.gate.fresh(now))
            self.core.on_frame(f.data, v, robot, now)
            if self.on_frame is not None:
                self.on_frame(f.data, v, now, self.gate.fresh(now))
            self._latch_events(f.data)
        now = self.clock()
        fresh = self.gate.fresh(now)
        if self.on_step is not None:
            self.on_step(now, fresh, "" if fresh else self.gate.why_stale(now))
        self.latch.tick(now, fresh, self.gate.why_stale(now), row, t_row)
        self._latch_events(None)
        msg = None
        if self.latch.armed:
            cmd = self.core.command(now, fresh, robot)
            if cmd is None:                      # nothing to send (hand invalid, ...): disarm explicitly
                self.latch.disarm("no command: " + (self.core.status.reason or self.core.status.state))
                self._latch_events(None)
            elif frames:                         # at the stream rate
                msg = self._send("alive") if cmd == "alive" else self._send("target", cmd)
        else:
            if not fresh and (self.core.frame is not None or self.core.eng is not None):
                self.core.stale()
            if fresh and frames:
                msg = self._send("alive")        # disarmed: keep-alives with armed: false
        self.sending = msg is not None
        return msg

    def close(self) -> None:
        self.actions.close()

    def run(self, stop: threading.Event, status: Optional[Callable[["TeleopClient"], None]] = None,
            status_period: float = 0.2) -> None:
        next_status = 0.0
        while not stop.is_set():
            self.step()
            if status is not None and self.clock() >= next_status:
                next_status = self.clock() + status_period
                status(self)


def open_wbc(wbc: WbcClient, cfg: Config) -> dict:
    """Lease (renewed in the background), session-default hints, enable, a datagram session."""
    wbc.acquire()
    wbc.set_hints(default_hints(cfg))
    wbc.enable()
    return wbc.open_session()


def close_wbc(wbc: WbcClient, log: Callable[[str], None] = print) -> None:
    """Disable and release, each attempted even if the other fails."""
    for f in (wbc.disable, wbc.release):
        try:
            f()
        except Exception as e:  # noqa: BLE001
            log(f"  {f.__name__}: {e}")


def status_line(c: TeleopClient) -> str:
    """Armed or not (and why), engagement state, sending, headset latency and rate, the WBC row's age, the stick
    offset (world x, y, yaw) and the base hint sent."""
    s = c.core.status
    _, t = c.stream.latest()
    now = c.clock()
    age = f"{(now - t) * 1e3:4.0f}ms" if t is not None else "  none"
    quest = "ok" if c.gate.fresh(now) else "--"
    lat = f"{c.gate.latency * 1e3:3.0f}ms" if c.gate.latency is not None else "  --"
    o, h = c.core.offset, c.core.hint
    hint = f"({h['pose']['x']:+.2f},{h['pose']['y']:+.2f},{h['heading']['yaw']:+.2f})" if h else "none"
    lt = c.latch
    arm = f"ARMED#{lt.arm_seq}" if lt.armed else "disarmed"
    why = (s.reason if lt.armed else lt.reason) or ""
    return (f"[{arm:9}|{s.state:13}] {'SENDING' if c.sending else 'silent '} quest={quest} lat={lat}"
            f" rate={c.gate.rate(now):3.0f}Hz wbc_row={age} eng={s.engagements} sent={c.wbc.sent.get('target', 0)}"
            f" offset=({o[0]:+.2f},{o[1]:+.2f},{o[2]:+.2f}) hint={hint} {why[:40]:40}")


def print_status(c: TeleopClient) -> None:
    print("\r" + status_line(c), end="", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quest", default="100.72.77.83", help="headset address (default %(default)s)")
    ap.add_argument("--quest-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--wbc", default="http://botcore-desktop.local/nodes/wbc",
                    help="WBC node base URL: robot LAN (default), the erid proxy, or the local sim stack")
    ap.add_argument("--udp-host", default=None, help="datagram destination host (default: the --wbc host)")
    ap.add_argument("--hand", choices=("left", "right"), default="right", help="the hand that drives the arm")
    ap.add_argument("--ee", default="arm", help="end-effector name in the WBC (default %(default)s)")
    ap.add_argument("--no-head-walk", action="store_true",
                    help="engaged: the base hint follows only the stick offset, not the head's travel and yaw")
    ap.add_argument("--stale-ms", type=float, default=STALE_S * 1e3, help="headset freshness timeout")
    ap.add_argument("--mode", choices=("third", "first"), default="third",
                    help="third person (default: the sticks move the robot, directions stay room-aligned) or "
                         "first person (the sticks shift the anchor)")
    ap.add_argument("--headless", action="store_true",
                    help="third person, registered: the left stick moves the robot along the operator's facing")
    ap.add_argument("--no-gripper", action="store_true")
    ap.add_argument("--max-latency-ms", type=float, default=MAX_LATENCY_S * 1e3,
                    help="headset frames later than this over the session's best count as stale (disarm)")
    ap.add_argument("--min-rate", type=float, default=MIN_RATE_HZ,
                    help="headset frame rate (over 0.5 s) below this counts as stale (disarm)")
    args = ap.parse_args(argv)

    cfg = Config(hand=args.hand, ee=args.ee, head_walk=not args.no_head_walk, mode=args.mode, headless=args.headless, gripper=not args.no_gripper)
    wbc = WbcClient(args.wbc, udp_host=args.udp_host)
    stream = WbcStream(args.wbc).start()
    reader = QuestReader(args.quest, args.quest_port).start()
    client = TeleopClient(cfg, reader, stream, wbc, stale_s=args.stale_ms / 1e3,
                          max_latency=args.max_latency_ms / 1e3, min_rate=args.min_rate)
    stop = threading.Event()
    try:
        info = open_wbc(wbc, cfg)
        client.session_opened(info)
        print(f"WBC session {info['session']} -> udp {wbc.udp_host}:{wbc.udp_port}; headset {args.quest}:"
              f"{args.quest_port}, {args.hand} grip clutches the arm. Nothing moves until Y (left) arms; X disables."
              " Ctrl-C to stop.", flush=True)
        client.run(stop, print_status)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print("\nstopping: disable, release", flush=True)
        client.close()
        close_wbc(wbc)
        reader.close()
        stream.close()
        wbc.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
