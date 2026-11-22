#!/usr/bin/env python3
"""Headset-side control for the Quest teleop: ``up``, ``down``, ``status`` (also as ``quest_up`` / ``quest_down`` /
``quest_status``). Mac or erid; stdlib only. Automates only the headset, not the teleop workflow.

    quest_up [--scheme body-frozen|standard] [--demo] [--ar [URL|:PORT]] [--restart] [--wait 30]
    quest_down
    quest_status

The Quest runs an SSH server (the quest-dev-runtime APK); ``~/.ssh/config`` has ``Host quest`` (``--ssh-host``).
Commands go through quest_env's own script, ``sh "$QUEST_ENV_ROOT/quest" ...`` (what ``q`` does in an interactive
shell; profile.sh defines ``q`` only for interactive shells, so it is not used over SSH). The root comes from
``~/.config/quest-env/root`` as in profile.sh, else ``~/workspace/quest_env``.

``up``:
1. SSH reachability (``BatchMode``, short timeout): a clear message if the headset is asleep or offline.
2. ``git fetch`` then compare with the upstream; ``quest pull`` only if behind. ``pull`` fast-forwards only and
   refuses a dirty tree: a dirty tree is reported and left alone, never forced.
3. Telemetry, headless: ``quest run --no-viewer --scheme=<scheme>`` (``-s`` too with ``--demo``, which renders the
   headset's own kinematic demo; ``--ar=URL`` with ``--ar``: the robot AR view, quest_env ROBOT_AR.md; bare ``--ar``
   is this machine's sim stack at its address toward the headset). The scheme works without ``-s``: the script
   writes ``off body-frozen`` to ``~/.config/quest-env/demo-mode`` and native reads the second word into
   ``operatorIntent_.bodyFrozen`` regardless of the demo. ``run`` restarts the XR process, so it is skipped when the stream already answers, the
   mode file and the AR URL (``~/.config/quest-env/robot-ar``) match and nothing was pulled (``--restart`` forces
   it).
4. Waits until port 9391 answers with ``teleop.quest_state.v1`` frames, then prints the status.

``down``: ``quest stop``. ``status``: ``quest status``, the mode file, the stream's liveness and the battery.
Nothing destructive is ever run on the headset (read-only queries, ``fetch``, ``pull --ff-only`` via the script,
``run``, ``stop``).
"""
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

STREAM_PORT = 9391
SCHEMA = "teleop.quest_state.v1"
ROOT = 'R=$(cat "$HOME/.config/quest-env/root" 2>/dev/null || echo "$HOME/workspace/quest_env"); '
MODE_FILE = '"$HOME/.config/quest-env/demo-mode"'
AR_FILE = '"$HOME/.config/quest-env/robot-ar"'
XR_STATES = {"0": "UNKNOWN", "1": "IDLE (headset not worn?)", "2": "READY", "3": "SYNCHRONIZED", "4": "VISIBLE",
             "5": "FOCUSED", "6": "STOPPING", "7": "LOSS_PENDING", "8": "EXITING"}
UNREACHABLE = ("the headset does not answer SSH ({detail}). Is it asleep, off, or off the network, or is the "
               "quest-dev-runtime SSH server not running? Wake it (put it on / press power) and retry.")


@dataclass
class Result:
    rc: int
    out: str = ""
    err: str = ""


class SshRunner:
    """Runs one shell command on the headset: ``ssh -T -o BatchMode=yes -o ConnectTimeout=N <host> <command>``."""

    def __init__(self, host: str = "quest", connect_timeout: int = 5):
        self.host = host
        self.connect_timeout = connect_timeout

    def __call__(self, command: str, timeout: float = 60.0) -> Result:
        # -T: never a tty (with one, `quest run` would start its curses viewer and block)
        argv = ["ssh", "-T", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={self.connect_timeout}", self.host,
                command]
        try:
            p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return Result(124, "", f"timed out after {timeout:.0f} s")
        except FileNotFoundError:
            return Result(127, "", "ssh not found")
        return Result(p.returncode, p.stdout, p.stderr)


SIM_STACK_PORT = 8080   # python -m robot_utils.nodes.sim.stack's default


def local_address_toward(host: str) -> str:
    """This machine's address on the route to ``host`` (the Tailscale address when the headset is reached over
    Tailscale, the LAN address on a shared LAN): a UDP connect sends nothing, it only picks the route."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((host, 9))
        return s.getsockname()[0]


def resolve_ar(value: str, headset: str, local=local_address_toward) -> str:
    """``--ar`` value -> the URL the headset opens: ``auto`` (bare ``--ar``) or ``:PORT`` = this machine's sim stack
    at its address toward the headset; ``host[:port]`` gets ``http://``; a full URL is kept."""
    if "://" in value:
        return value.rstrip("/")
    if value == "auto" or value.startswith(":"):
        port = value[1:] if value.startswith(":") else str(SIM_STACK_PORT)
        return f"http://{local(headset)}:{port}"
    return f"http://{value}".rstrip("/")


def ssh_hostname(host: str) -> str:
    """The address ``ssh <host>`` connects to (``ssh -G``, local only), for the stream probe."""
    try:
        out = subprocess.run(["ssh", "-G", host], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return host
    for line in out.splitlines():
        k, _, v = line.partition(" ")
        if k == "hostname" and v:
            return v.strip()
    return host


# -- the stream -----------------------------------------------------------------------------------

@dataclass
class StreamInfo:
    live: bool
    frames: int = 0
    rate_hz: float = 0.0
    session_id: Optional[str] = None
    focused: Optional[bool] = None
    xr_state: Optional[str] = None
    scheme: Optional[str] = None
    error: str = ""
    device: Optional[dict] = None     # the newest record's ``device`` block (battery), when the headset sends one

    def describe(self) -> str:
        if not self.live:
            return f"stream: not answering ({self.error or 'no frames'})"
        return (f"stream: live, {self.rate_hz:.0f} Hz, session {self.session_id}, "
                f"{'focused' if self.focused else 'NOT focused'} ({self.xr_state}), scheme {self.scheme}")


def probe_stream(host: str, port: int = STREAM_PORT, seconds: float = 1.0) -> StreamInfo:
    """Read the stream for ``seconds``: frame rate (our clock), session, focus, the intent's scheme."""
    try:
        sock = socket.create_connection((host, port), timeout=2)
    except OSError as e:
        return StreamInfo(False, error=f"{type(e).__name__}: {e}")
    n, first, last, t_first, t_last = 0, None, None, None, None
    deadline = time.monotonic() + seconds
    try:
        sock.settimeout(0.5)
        f = sock.makefile("rb")
        while time.monotonic() < deadline:
            try:
                line = f.readline()
            except OSError:
                break
            if not line:
                break
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if not isinstance(d, dict) or d.get("schema") != SCHEMA:
                continue
            t = time.monotonic()
            n += 1
            first = first or d
            t_first = t_first or t
            last, t_last = d, t
    finally:
        sock.close()
    if n == 0:
        return StreamInfo(False, error="connected, no frames")
    rate = (n - 1) / (t_last - t_first) if n > 1 and t_last > t_first else 0.0
    intent = last.get("control_intent") or {}
    dev = last.get("device")
    return StreamInfo(True, n, rate, last.get("session_id"), last.get("input_focused"), last.get("xr_session_state"),
                      intent.get("scheme"), device=dev if isinstance(dev, dict) else None)


# -- commands -------------------------------------------------------------------------------------

class QuestCtl:
    def __init__(self, run: Callable[..., Result], stream_host: str, *, port: int = STREAM_PORT,
                 probe: Callable[..., StreamInfo] = probe_stream, log: Callable[[str], None] = print,
                 sleep: Callable[[float], None] = time.sleep, clock: Callable[[], float] = time.monotonic):
        self.run, self.stream_host, self.port = run, stream_host, port
        self.probe, self.log, self.sleep, self.clock = probe, log, sleep, clock

    def q(self, args: str, timeout: float = 60.0) -> Result:
        return self.run(ROOT + f'sh "$R/quest" {args}', timeout)

    def reachable(self) -> bool:
        r = self.run("echo quest-ok", 15)
        if r.rc == 0 and "quest-ok" in r.out:
            return True
        self.log(UNREACHABLE.format(detail=(r.err.strip().splitlines() or [f"exit {r.rc}"])[-1]))
        return False

    def repo_state(self) -> Optional[tuple[int, int]]:
        """(commits behind upstream, dirty files) after a fetch; None if it could not be determined."""
        r = self.run(ROOT + 'git -C "$R" fetch --quiet && '
                     'echo "behind=$(git -C "$R" rev-list --count HEAD..@{u})" && '
                     'echo "dirty=$(git -C "$R" status --porcelain | wc -l)"', 90)
        vals = {}
        for line in r.out.splitlines():
            k, _, v = line.strip().partition("=")
            if k in ("behind", "dirty") and v.strip().isdigit():
                vals[k] = int(v)
        if r.rc != 0 or len(vals) != 2:
            self.log(f"could not check for updates (exit {r.rc}): {(r.err.strip() or r.out.strip())[-200:]}")
            return None
        return vals["behind"], vals["dirty"]

    def mode(self) -> str:
        r = self.run(f"cat {MODE_FILE} 2>/dev/null", 15)
        return " ".join(r.out.split()) if r.rc == 0 else ""

    def ar_url(self) -> Optional[str]:
        """The robot AR URL the running telemetry was launched with (``url=`` in the AR file), None without one."""
        r = self.run(f"cat {AR_FILE} 2>/dev/null", 15)
        for line in r.out.splitlines() if r.rc == 0 else []:
            if line.startswith("url="):
                return line[4:].strip()
        return None

    def xr_state(self) -> str:
        """The telemetry process's last OpenXR session state, from ``quest status`` (its pid) and logcat (read-only).
        The stream only flows once the session runs (FOCUSED/VISIBLE): IDLE means the headset is not being worn."""
        r = self.q("status", 30)
        try:
            st = json.loads(r.out)
            pid = st["last_load"]["pid"]
            alive = st.get("last_load_process_alive")
        except (ValueError, KeyError, TypeError):
            return "unknown"
        if not alive:
            return "not running"
        r = self.run(f"logcat -d 2>/dev/null | grep ' {int(pid)} ' | grep 'OpenXR session state' | tail -1", 30)
        word = r.out.strip().split(" ")[-1] if r.rc == 0 and r.out.strip() else ""
        return XR_STATES.get(word, f"unknown ({word or 'no log line'})")

    @staticmethod
    def battery(info: Optional[StreamInfo]) -> str:
        """The headset battery from the telemetry stream's ``device`` block. Over SSH it cannot be read: the shell
        runs in an Android app sandbox (sysfs power_supply and the battery service are denied), so only the
        telemetry app (Android's battery broadcast) reports it, and only while telemetry runs."""
        d = info.device if info is not None else None
        if d and isinstance(d.get("battery_pct"), (int, float)):
            extra = ", charging" if d.get("charging") else (", plugged" if d.get("plugged") else "")
            t = d.get("temperature_c")
            return f"{int(d['battery_pct'])}%{extra}" + (f", {t:.0f} C" if isinstance(t, (int, float)) else "")
        if info is not None and info.live:
            return "not reported (the headset's telemetry predates the battery field: quest pull + restart)"
        return "unavailable while telemetry is stopped (the SSH shell cannot read it)"

    def wait_stream(self, wait: float) -> StreamInfo:
        deadline = self.clock() + wait
        info = self.probe(self.stream_host, self.port, 1.0)
        while not info.live and self.clock() < deadline:
            self.sleep(0.5)
            info = self.probe(self.stream_host, self.port, 1.0)
        return info

    def up(self, scheme: str = "body-frozen", demo: bool = False, restart: bool = False, wait: float = 30.0,
           ar: Optional[str] = None) -> int:
        if not self.reachable():
            return 2
        pulled = False
        state = self.repo_state()
        if state is not None:
            behind, dirty = state
            if behind and dirty:
                self.log(f"headset clone is {behind} commit(s) behind but has {dirty} local change(s): not pulling "
                         "(quest pull refuses a dirty tree; resolve it on the headset). Using the current version.")
            elif behind:
                self.log(f"headset clone is {behind} commit(s) behind: quest pull")
                r = self.q("pull", 180)
                if r.rc != 0:
                    self.log(f"quest pull failed (exit {r.rc}): {(r.err.strip() or r.out.strip())[-300:]}")
                    return 3
                pulled = True
            else:
                self.log("headset clone is up to date" + (f" ({dirty} local change(s))" if dirty else ""))
        want = f"{'live' if demo else 'off'}{' body-frozen' if scheme == 'body-frozen' else ''}"
        current, current_ar = self.mode(), self.ar_url()
        live = self.probe(self.stream_host, self.port, 1.0)
        if live.live and current == want and current_ar == ar and not pulled and not restart:
            self.log(f"telemetry already running ({want}{', AR ' + ar if ar else ''}); not restarting")
        else:
            why = ("--restart" if restart else "updated" if pulled else
                   "stream not answering" if not live.live else
                   f"mode {current or 'unset'!r} -> {want!r}" if current != want else
                   f"AR {current_ar or 'off'} -> {ar or 'off'}")
            args = f"run --no-viewer --scheme={scheme}" + (" -s" if demo else "") + (f" --ar={ar}" if ar else "")
            self.log(f"starting telemetry: quest {args} ({why})")
            r = self.q(args, 120)
            if r.rc != 0:
                self.log(f"quest run failed (exit {r.rc}): {(r.err.strip() or r.out.strip())[-300:]}")
                return 4
        info = self.wait_stream(wait)
        if not info.live:
            self.log(f"port {self.port} on {self.stream_host} did not answer within {wait:.0f} s: {info.error}. "
                     f"XR session: {self.xr_state()}. The stream flows only while the XR session runs: put the "
                     "headset on (or `q focus` on the headset), then quest_status.")
            return 5
        self.log(info.describe())
        if info.scheme and info.scheme != scheme:
            self.log(f"warning: the stream reports scheme {info.scheme!r}, wanted {scheme!r}")
        self.log(f"battery: {self.battery(info)}")
        return 0

    def down(self) -> int:
        if not self.reachable():
            return 2
        r = self.q("stop", 60)
        self.log((r.out.strip() or r.err.strip() or "stopped") if r.rc == 0 else f"quest stop failed (exit {r.rc}): "
                 f"{(r.err.strip() or r.out.strip())[-300:]}")
        return 0 if r.rc == 0 else 4

    def status(self) -> int:
        if not self.reachable():
            return 2
        r = self.q("status", 30)
        self.log("native: " + (r.out.strip() or r.err.strip() or f"exit {r.rc}"))
        self.log(f"mode file: {self.mode() or 'unset'}")
        self.log(f"XR session: {self.xr_state()}")
        info = self.probe(self.stream_host, self.port, 1.0)
        self.log(info.describe())
        self.log(f"battery: {self.battery(info)}")
        return 0


def main(argv=None, prog: Optional[str] = None) -> int:
    prog = prog or Path(sys.argv[0]).name
    implied = {"quest_up": "up", "quest_down": "down", "quest_status": "status"}.get(prog)
    ap = argparse.ArgumentParser(prog=prog, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    if implied is None:
        ap.add_argument("command", choices=("up", "down", "status"))
    ap.add_argument("--ssh-host", default="quest", help="ssh config host of the headset (default %(default)s)")
    ap.add_argument("--stream-host", default=None, help="stream address (default: the ssh host's HostName)")
    ap.add_argument("--port", type=int, default=STREAM_PORT)
    ap.add_argument("--scheme", choices=("body-frozen", "standard"), default="body-frozen")
    ap.add_argument("--demo", action="store_true", help="also the headset's own kinematic demo (-s)")
    ap.add_argument("--ar", nargs="?", const="auto", metavar="URL|:PORT",
                    help="robot AR view of a robot frontend (plain http). Bare --ar: this machine's sim stack "
                    f"(port {SIM_STACK_PORT}) at its address toward the headset; :PORT another port; or a URL / host")
    ap.add_argument("--restart", action="store_true", help="restart telemetry even if it is already running")
    ap.add_argument("--wait", type=float, default=30.0, help="seconds to wait for the stream")
    args = ap.parse_args(argv)
    command = implied or args.command
    stream_host = args.stream_host or ssh_hostname(args.ssh_host)
    ctl = QuestCtl(SshRunner(args.ssh_host), stream_host, port=args.port, log=lambda s: print(s, flush=True))
    if command == "up":
        ar = None
        if args.ar:
            try:
                ar = resolve_ar(args.ar, stream_host)
            except OSError as e:
                print(f"--ar: no route to the headset at {stream_host} ({e}); pass the URL", flush=True)
                return 2
            print(f"robot AR: {ar}", flush=True)
        return ctl.up(args.scheme, args.demo, args.restart, args.wait, ar)
    return ctl.down() if command == "down" else ctl.status()


if __name__ == "__main__":
    sys.exit(main())
