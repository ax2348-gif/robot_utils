"""Observability for the robot's port-80 traffic: who is connected, what they use, uplink health.

Read-only (admin stays on SSH). Runs as threads inside the landing page:

- **Access log tail**: nginx writes one JSON object per finished request to ``access.log``
  (frontend/proxy.py). The tail follows rotation and keeps the recent entries plus per-client
  totals for the last hour.
- **Identity**: Tailscale addresses (100.64.0.0/10, fd7a:115c:a1e0::/48) are unforgeable over the
  tailnet; ``tailscale whois`` maps them to the device and its owner (cached 10 min). LAN
  addresses are reported as they are.
- **Live peers**: every 2 s, ``ss`` on port 80 gives each client's open connections, send rate
  and send-queue backlog. This shows streams while they are open: nginx logs a request only when
  it ends.
- **Uplink**: ping to the internet (1.1.1.1) and the gateway every 10 s, plus interface rates.
- **Rotation**: daily (or past 100 MB), ``access.log`` and ``error.log`` are renamed and nginx is
  told to reopen them (SIGUSR1); rotated files older than 7 days are deleted.

    python -m robot_utils.frontend.observe report [--hours 24] [--top 15]   # terminal summary (over SSH)
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
import signal
import subprocess
import threading
import time
from collections import Counter, defaultdict, deque
from pathlib import Path
from typing import Callable, Optional

STATE_DIR = Path(os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state"))) / "robot_utils" / "nginx"
RETENTION_DAYS = 7
ROTATE_BYTES = 100 * 1024 * 1024
ERROR_ROTATE_BYTES = 20 * 1024 * 1024
RECENT = 2000                 # recent log entries kept in memory
WINDOW_S = 3600.0             # per-client totals over this window
WHOIS_TTL = 600.0
WHOIS_FAIL_TTL = 60.0
PEER_EVERY = 2.0
UPLINK_EVERY = 10.0
UPLINK_HISTORY = 360          # samples (1 h at 10 s)
SATURATED_MS = 150.0          # internet RTT above this = saturated
TS_V4 = ipaddress.ip_network("100.64.0.0/10")
TS_V6 = ipaddress.ip_network("fd7a:115c:a1e0::/48")

Runner = Callable[[list[str], float], Optional[str]]   # argv, timeout -> stdout (None on failure)


def run(argv: list[str], timeout: float) -> Optional[str]:
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout if r.returncode == 0 else None


def is_tailscale(ip: str) -> bool:
    try:
        a = ipaddress.ip_address(ip)
    except ValueError:
        return False
    return a in TS_V4 if a.version == 4 else a in TS_V6


# ----------------------------------------------------------------------------------------------
# identity
# ----------------------------------------------------------------------------------------------

def parse_whois(text: str) -> Optional[dict]:
    try:
        d = json.loads(text)
    except ValueError:
        return None
    node, prof = d.get("Node") or {}, d.get("UserProfile") or {}
    name = (node.get("ComputedName") or node.get("Name") or "").split(".")[0] or None
    return {"device": name, "os": (node.get("Hostinfo") or {}).get("OS"),
            "user": prof.get("LoginName"), "name": prof.get("DisplayName"),
            "tags": node.get("Tags") or None}


class Identities:
    """IP -> identity (Tailscale whois for tailnet addresses; LAN / other: the address itself)."""

    def __init__(self, runner: Runner = run, clock=time.monotonic):
        self.runner, self.clock = runner, clock
        self._cache: dict[str, tuple[float, dict]] = {}
        self._lock = threading.Lock()

    def get(self, ip: str) -> dict:
        if not is_tailscale(ip):
            kind = "local" if ip in ("127.0.0.1", "::1", "unix:") else "lan"
            return {"ip": ip, "kind": kind, "label": ip}
        now = self.clock()
        with self._lock:
            hit = self._cache.get(ip)
        if hit and now < hit[0]:
            return hit[1]
        out = self.runner(["tailscale", "whois", "--json", ip], 3.0)
        info = parse_whois(out) if out else None
        if info:
            ident = {"ip": ip, "kind": "tailscale", **info,
                     "label": f"{info.get('user') or '?'} · {info.get('device') or ip}"}
            ttl = WHOIS_TTL
        else:
            ident = {"ip": ip, "kind": "tailscale", "label": f"{ip} (unknown to whois)"}
            ttl = WHOIS_FAIL_TTL
        with self._lock:
            self._cache[ip] = (now + ttl, ident)
        return ident


# ----------------------------------------------------------------------------------------------
# access log
# ----------------------------------------------------------------------------------------------

def parse_line(line: str) -> Optional[dict]:
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        e = json.loads(line)
    except ValueError:
        return None
    if "ip" not in e or "ts" not in e:
        return None
    return e


class LogTail:
    """Follows access.log across rotation; keeps recent entries and 1 h per-client totals."""

    def __init__(self, path: Path, warm_bytes: int = 2_000_000):
        self.path = path
        self.recent: deque = deque(maxlen=RECENT)
        self._ino = None
        self._pos = 0
        self._warm = warm_bytes
        self.lock = threading.Lock()

    def poll(self) -> int:
        """Read new complete lines; returns how many entries were added."""
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return 0
        skip_partial = False
        if self._ino != st.st_ino or st.st_size < self._pos:          # new / rotated / truncated file
            first = self._ino is None
            self._ino = st.st_ino
            self._pos = max(0, st.st_size - self._warm) if first else 0
            skip_partial = self._pos > 0                               # warm start mid-file
        if st.st_size <= self._pos:
            return 0
        with open(self.path, "rb") as f:
            f.seek(self._pos)
            data = f.read(st.st_size - self._pos)
        cut = data.rfind(b"\n")
        if cut < 0:
            return 0                                                   # wait for the line to complete
        chunk = data[: cut + 1]
        self._pos += cut + 1
        if skip_partial:
            nl = chunk.find(b"\n")
            chunk = chunk[nl + 1:]
        entries = [e for e in (parse_line(l) for l in chunk.decode("utf-8", "replace").splitlines()) if e]
        with self.lock:
            self.recent.extend(entries)
        return len(entries)

    def window(self, since: float) -> list[dict]:
        with self.lock:
            return [e for e in self.recent if e.get("ts", 0) >= since]


# ----------------------------------------------------------------------------------------------
# live peers (ss) and uplink
# ----------------------------------------------------------------------------------------------

_PEER = re.compile(r"^\S+\s+(\d+)\s+(\d+)\s+(\S+)\s+(\S+)")


def parse_ss(text: str, port: int = 80) -> dict[tuple, dict]:
    """``ss -tniH state established`` text -> {(local, peer): {"acked": bytes, "queue": bytes}}."""
    out, lines = {}, text.splitlines()
    i = 0
    while i < len(lines):
        head = lines[i].strip()
        info = lines[i + 1] if i + 1 < len(lines) and lines[i + 1].startswith((" ", "\t")) else ""
        i += 2 if info else 1
        parts = head.split()
        if len(parts) < 4:
            continue
        # without -H state column omitted when filtering by state: Recv-Q Send-Q Local Peer
        try:
            q = int(parts[1])
        except ValueError:
            continue
        local, peer = parts[2], parts[3]
        if not local.endswith(f":{port}"):
            continue
        m = re.search(r"bytes_acked:(\d+)", info)
        out[(local, peer)] = {"acked": int(m.group(1)) if m else 0, "queue": q}
    return out


def peer_ip(peer: str) -> str:
    host = peer.rsplit(":", 1)[0]
    return host.strip("[]").replace("::ffff:", "")


def parse_ping(text: Optional[str]) -> Optional[float]:
    if not text:
        return None
    m = re.search(r"time[=<]([\d.]+)\s*ms", text)
    return float(m.group(1)) if m else None


def read_netdev(path: str = "/proc/net/dev") -> dict[str, tuple[int, int]]:
    out = {}
    try:
        with open(path) as f:
            for line in f.readlines()[2:]:
                name, rest = line.split(":", 1)
                v = rest.split()
                out[name.strip()] = (int(v[0]), int(v[8]))
    except (OSError, ValueError, IndexError):
        pass
    return out


# ----------------------------------------------------------------------------------------------
# rotation
# ----------------------------------------------------------------------------------------------

def rotate(state_dir: Path, now: Optional[float] = None, send_signal=os.kill) -> list[str]:
    """Rename access.log (daily or > 100 MB) and error.log (> 20 MB), have nginx reopen them, and
    delete rotated logs older than RETENTION_DAYS. Returns the actions taken."""
    now = time.time() if now is None else now
    actions = []
    acc, err = state_dir / "access.log", state_dir / "error.log"
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    renamed = False
    if acc.exists():
        st = acc.stat()
        day_old = time.strftime("%Y%m%d", time.localtime(st.st_mtime if st.st_size == 0 else _first_ts(acc, st.st_mtime))) \
            != time.strftime("%Y%m%d", time.localtime(now))
        if st.st_size > 0 and (day_old or st.st_size > ROTATE_BYTES):
            acc.rename(state_dir / f"access-{stamp}.log")
            actions.append(f"rotated access.log ({st.st_size} bytes)")
            renamed = True
    if err.exists() and err.stat().st_size > ERROR_ROTATE_BYTES:
        err.rename(state_dir / f"error-{stamp}.log")
        actions.append("rotated error.log")
        renamed = True
    if renamed:
        pid_file = state_dir / "nginx.pid"
        try:
            send_signal(int(pid_file.read_text().strip()), signal.SIGUSR1)   # nginx: reopen the log files
            actions.append("nginx reopened its logs")
        except (OSError, ValueError) as e:
            actions.append(f"could not signal nginx: {e}")
    cutoff = now - RETENTION_DAYS * 86400
    for f in list(state_dir.glob("access-*.log")) + list(state_dir.glob("error-*.log")):
        if f.stat().st_mtime < cutoff:
            f.unlink()
            actions.append(f"deleted {f.name}")
    return actions


def _first_ts(path: Path, default: float) -> float:
    try:
        with open(path) as f:
            e = parse_line(f.readline())
        return float(e["ts"]) if e else default
    except (OSError, ValueError, KeyError):
        return default


# ----------------------------------------------------------------------------------------------
# the observer
# ----------------------------------------------------------------------------------------------

class Observer:
    def __init__(self, state_dir: Path = STATE_DIR, runner: Runner = run, netdev=read_netdev,
                 targets=("1.1.1.1",), port: int = 80):
        self.state_dir, self.runner, self.netdev, self.port = state_dir, runner, netdev, port
        self.tail = LogTail(state_dir / "access.log")
        self.ids = Identities(runner)
        self.targets = targets
        self.lock = threading.Lock()
        self.peers: dict[str, dict] = {}        # ip -> {conns, bps, queue}
        self._last_ss: Optional[tuple[float, dict]] = None
        self.uplink: deque = deque(maxlen=UPLINK_HISTORY)
        self.iface: dict[str, dict] = {}
        self._last_dev: Optional[tuple[float, dict]] = None
        self.gateway: Optional[str] = None
        self.rotations: deque = deque(maxlen=50)
        self.stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # -- sampling ----------------------------------------------------------------------------

    def sample_peers(self, now: Optional[float] = None):
        now = time.monotonic() if now is None else now
        text = self.runner(["ss", "-tniH", "state", "established", f"( sport = :{self.port} )"], 3.0)
        if text is None:
            return
        cur = parse_ss(text, self.port)
        peers: dict[str, dict] = defaultdict(lambda: {"conns": 0, "bps": 0.0, "queue": 0})
        prev = self._last_ss
        for key, v in cur.items():
            ip = peer_ip(key[1])
            p = peers[ip]
            p["conns"] += 1
            p["queue"] += v["queue"]
            if prev and key in prev[1] and now > prev[0]:
                p["bps"] += max(0, v["acked"] - prev[1][key]["acked"]) / (now - prev[0])
        self._last_ss = (now, cur)
        with self.lock:
            self.peers = dict(peers)

    def sample_uplink(self, now: Optional[float] = None):
        now_wall = time.time() if now is None else now
        if self.gateway is None:
            out = self.runner(["ip", "route", "show", "default"], 2.0) or ""
            m = re.search(r"via (\S+)", out)
            self.gateway = m.group(1) if m else ""
        rec = {"t": now_wall}
        for name, host in [("internet", t) for t in self.targets] + ([("gateway", self.gateway)] if self.gateway else []):
            rec[f"{name}_ms"] = parse_ping(self.runner(["ping", "-n", "-c", "1", "-W", "2", host], 3.5))
        dev = self.netdev()
        mono = time.monotonic()
        if self._last_dev and mono > self._last_dev[0]:
            dt = mono - self._last_dev[0]
            iface = {}
            for k, (rx, tx) in dev.items():
                if k in self._last_dev[1] and k != "lo":
                    prx, ptx = self._last_dev[1][k]
                    iface[k] = {"rx_bps": max(0, rx - prx) / dt, "tx_bps": max(0, tx - ptx) / dt}
            with self.lock:
                self.iface = iface
        self._last_dev = (mono, dev)
        with self.lock:
            self.uplink.append(rec)

    # -- threads -----------------------------------------------------------------------------

    def start(self):
        def loop(fn, every):
            while not self.stop.is_set():
                try:
                    fn()
                except Exception:          # never let observation take the frontend down
                    pass
                self.stop.wait(every)

        def rot():
            acts = rotate(self.state_dir)
            if acts:
                self.rotations.append({"t": time.time(), "actions": acts})

        for fn, every in ((self.tail.poll, 1.0), (self.sample_peers, PEER_EVERY),
                          (self.sample_uplink, UPLINK_EVERY), (rot, 600.0)):
            t = threading.Thread(target=loop, args=(fn, every), daemon=True, name=f"observe-{fn.__name__}")
            t.start()
            self._threads.append(t)

    def close(self):
        self.stop.set()

    # -- views -------------------------------------------------------------------------------

    def clients(self, now: Optional[float] = None) -> list[dict]:
        now = time.time() if now is None else now
        hour = self.tail.window(now - WINDOW_S)
        stats: dict[str, dict] = defaultdict(lambda: {"requests": 0, "bytes": 0, "errors": 0,
                                                      "nodes": Counter(), "last": 0.0, "last_uri": None})
        for e in hour:
            s = stats[e["ip"]]
            s["requests"] += 1
            s["bytes"] += int(e.get("bytes") or 0)
            s["errors"] += int(int(e.get("status") or 0) >= 400)
            s["nodes"][e.get("node") or "(landing)"] += 1
            if e["ts"] >= s["last"]:
                s["last"], s["last_uri"] = e["ts"], e.get("uri")
        with self.lock:
            peers = dict(self.peers)
        out = []
        for ip in set(stats) | set(peers):
            s, p = stats.get(ip), peers.get(ip, {"conns": 0, "bps": 0.0, "queue": 0})
            out.append({"identity": self.ids.get(ip), "live": {"connections": p["conns"],
                        "send_bps": round(p["bps"]), "send_queue": p["queue"]},
                        "hour": None if not s else {"requests": s["requests"], "bytes": s["bytes"],
                                                    "errors": s["errors"], "nodes": dict(s["nodes"].most_common(6)),
                                                    "last_seen": s["last"], "last_uri": s["last_uri"]}})
        out.sort(key=lambda c: (-c["live"]["send_bps"], -c["live"]["connections"],
                                -((c["hour"] or {}).get("bytes") or 0)))
        return out

    def uplink_view(self) -> dict:
        with self.lock:
            hist = list(self.uplink)
            iface = dict(self.iface)
        last = hist[-1] if hist else {}
        recent = [h.get("internet_ms") for h in hist[-6:] if h.get("internet_ms") is not None]
        lost = sum(1 for h in hist[-6:] if h.get("internet_ms") is None)
        med = sorted(recent)[len(recent) // 2] if recent else None
        return {"internet_ms": last.get("internet_ms"), "gateway_ms": last.get("gateway_ms"),
                "internet_ms_median_1min": med, "lost_1min": lost,
                "saturated": bool(med is not None and med > SATURATED_MS) or lost >= 3,
                "interfaces": {k: {kk: round(vv) for kk, vv in v.items()} for k, v in iface.items()},
                "history": [{"t": h["t"], "internet_ms": h.get("internet_ms"), "gateway_ms": h.get("gateway_ms")}
                            for h in hist]}

    def snapshot(self) -> dict:
        return {"t": time.time(), "clients": self.clients(), "uplink": self.uplink_view(),
                "log": {"file": str(self.tail.path), "retention_days": RETENTION_DAYS,
                        "entries_in_memory": len(self.tail.recent)},
                "rotations": list(self.rotations)[-5:],
                "note": "requests are logged when they end: open streams show as live connections / send rate"}

    def requests(self, limit: int = 100, ip: Optional[str] = None, node: Optional[str] = None) -> list[dict]:
        with self.tail.lock:
            es = list(self.tail.recent)
        es = [e for e in es if (ip is None or e["ip"] == ip) and (node is None or e.get("node") == node)]
        es = sorted(es, key=lambda e: e.get("ts", 0), reverse=True)[:limit]       # newest first
        return [{**e, "who": self.ids.get(e["ip"])["label"]} for e in es]


# ----------------------------------------------------------------------------------------------
# terminal report (admin over SSH)
# ----------------------------------------------------------------------------------------------

def report(state_dir: Path, hours: float, top: int, ids: Optional[Identities] = None, now: Optional[float] = None) -> str:
    now = time.time() if now is None else now
    since = now - hours * 3600
    ids = ids or Identities()
    files = sorted(state_dir.glob("access-*.log")) + [state_dir / "access.log"]
    per = defaultdict(lambda: {"requests": 0, "bytes": 0, "errors": 0, "dur": 0.0, "nodes": Counter(),
                               "paths": Counter(), "first": None, "last": None})
    n = 0
    for f in files:
        if not f.exists() or f.stat().st_mtime < since:
            continue
        with open(f, errors="replace") as fh:
            for line in fh:
                e = parse_line(line)
                if not e or e["ts"] < since:
                    continue
                n += 1
                s = per[e["ip"]]
                s["requests"] += 1
                s["bytes"] += int(e.get("bytes") or 0)
                s["errors"] += int(int(e.get("status") or 0) >= 400)
                s["dur"] += float(e.get("dur") or 0)
                s["nodes"][e.get("node") or "(landing)"] += 1
                s["paths"][re.sub(r"\?.*", "", e.get("uri") or "")] += 1
                s["first"] = e["ts"] if s["first"] is None else min(s["first"], e["ts"])
                s["last"] = e["ts"] if s["last"] is None else max(s["last"], e["ts"])
    lines = [f"port-80 access, last {hours:g} h: {n} requests from {len(per)} clients"]
    for ip, s in sorted(per.items(), key=lambda kv: -kv[1]["bytes"])[:top]:
        who = ids.get(ip)["label"]
        span = f"{time.strftime('%m-%d %H:%M', time.localtime(s['first']))} - {time.strftime('%H:%M', time.localtime(s['last']))}"
        lines.append(f"\n{who}   [{ip}]")
        lines.append(f"  {s['requests']} requests, {s['bytes'] / 1e6:.1f} MB sent, {s['errors']} errors, "
                     f"{s['dur'] / 3600:.2f} h connected, {span}")
        lines.append("  nodes: " + ", ".join(f"{k} {v}" for k, v in s["nodes"].most_common(6)))
        lines.append("  top:   " + ", ".join(f"{k} {v}" for k, v in s["paths"].most_common(4)))
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.frontend.observe", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["report", "rotate"])
    ap.add_argument("--hours", type=float, default=24.0)
    ap.add_argument("--top", type=int, default=15)
    ap.add_argument("--state-dir", type=Path, default=STATE_DIR)
    a = ap.parse_args(argv)
    if a.command == "report":
        print(report(a.state_dir, a.hours, a.top))
    else:
        for act in rotate(a.state_dir) or ["nothing to rotate"]:
            print(act)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
