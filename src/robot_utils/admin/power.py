"""Power node from SSH (nodes/power, docs/power.md): status, shutdown with a countdown, cancel. Run on the robot.

    python -m robot_utils.admin.power status
    python -m robot_utils.admin.power shutdown --in 60 --reason "end of day"   # asks you to type "shutdown"
    python -m robot_utils.admin.power shutdown --in 30 --reason "..." --yes     # no prompt (scripts)
    python -m robot_utils.admin.power cancel
    python -m robot_utils.admin.power log

Talks to the running power node over its unix socket (--name, default "power"); the node holds the
countdown, so a shutdown requested here shows on the page (and can be cancelled there), and one
requested on the page can be cancelled here. At the end of the countdown the node e-stops every
actuated node, arms the UPS power-off and runs `systemctl poweroff`.
"""
from __future__ import annotations

import argparse
import sys
import time

import httpx

from ..nodes.common.serve import socket_path
from .locks import who_am_i


def connect(name: str) -> httpx.Client:
    path = socket_path(name)
    if not path.exists():
        raise SystemExit(f"error: the {name!r} node is not running (no socket {path})")
    return httpx.Client(base_url=f"http://{name}", transport=httpx.HTTPTransport(uds=str(path)), timeout=10)


def _call(c: httpx.Client, method: str, path: str, body=None) -> tuple[int, dict]:
    r = c.request(method, path, json=body)
    try:
        data = r.json()
    except ValueError:
        data = {"detail": r.text}
    return r.status_code, data


def fmt_status(st: dict) -> str:
    out = [f"state: {st['state']}" + ("   (FAKE UPS)" if st.get("fake") else "")]
    if st.get("error"):
        out.append(f"error: {st['error']}")
    for s in st["sources"]:
        if not s["telemetry"]:
            out.append(f"  {s['id']:5s} {s['type']}: {s.get('note', 'no telemetry')}")
            continue
        ok = "ok" if s["ok"] else "STALE" if s["stale"] else "NO READING"
        line = f"  {s['id']:5s} {s['type']} [{ok}]"
        if s.get("percent") is not None:
            rt = f", ~{s['runtime_min']} min left ({s['runtime_source']})" if s.get("runtime_min") is not None else ""
            line += (f" {s['percent']} % {s['pack_v']:.2f} V {s['current_a']:+.2f} A, {s['charge_state']}, "
                     f"input {'present' if s.get('input_present') else 'absent'}{rt}")
            if s.get("cells_v"):
                line += "\n        cells " + " ".join(f"{v:.3f}" for v in s["cells_v"]) + " V"
        out.append(line)
    lv = st["low_voltage"]
    out.append(f"low-voltage shutdown: {lv['status']} (threshold {lv['volts']:.2f} V, hold {lv['hold_s']:.0f} s)")
    out.append(fmt_shutdown(st["shutdown"]))
    return "\n".join(out)


def fmt_shutdown(sd: dict) -> str:
    if sd["pending"]:
        when = f"in {sd['in_s']:.0f} s" if sd["phase"] == "countdown" else sd["phase"]
        return (f"SHUTDOWN {when}: requested by {sd['by']} [{sd['client']}, {sd['source']}]: {sd['reason']}\n"
                "  cancel: python -m robot_utils.admin.power cancel")
    last = sd.get("last")
    if not last:
        return "shutdown: none pending"
    t = time.strftime("%H:%M:%S", time.localtime(last["requested_at"]))
    extra = f", cancelled by {last['cancelled_by']}" if last["phase"] == "cancelled" else f" ({last.get('error')})" if last.get("error") else ""
    return f"shutdown: none pending (last: {t} by {last['by']}: {last['reason']} -> {last['phase']}{extra})"


def main(argv=None, client: httpx.Client | None = None, ask=input, isatty=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.admin.power", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="power", help="power node instance (default: power)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    s = sub.add_parser("shutdown", help="power the Pi off after a countdown")
    s.add_argument("--in", dest="delay", type=float, default=30.0, help="countdown in s (10-300, default 30)")
    s.add_argument("--reason", default="requested over SSH", help="why (logged, shown on the page)")
    s.add_argument("--as", dest="by", default=None, help="your name (default: login and SSH client)")
    s.add_argument("--yes", action="store_true", help="don't ask to type 'shutdown'")
    c = sub.add_parser("cancel", help="cancel a pending shutdown")
    c.add_argument("--reason", default="")
    c.add_argument("--as", dest="by", default=None)
    lg = sub.add_parser("log", help="the audit log (requests, cancellations, outcomes)")
    lg.add_argument("-n", type=int, default=20)
    a = ap.parse_args(argv)
    http = client or connect(a.name)
    try:
        if a.cmd == "status":
            code, st = _call(http, "GET", "/state")
            if code != 200:
                print(f"error: {code}: {st.get('detail')}", file=sys.stderr)
                return 1
            print(fmt_status(st))
            return 0
        if a.cmd == "log":
            code, d = _call(http, "GET", f"/log?n={max(1, min(100, a.n))}")
            for r in d.get("log", []):
                bits = [r.get("time", ""), r.get("event", "")]
                bits += [f"by {r['by']}"] if r.get("by") else []
                bits += [f"[{r['client']}]"] if r.get("client") else []
                bits += [f": {r['reason']}"] if r.get("reason") else []
                bits += [f"({r.get('detail') or r.get('error')})"] if r.get("detail") or r.get("error") else []
                print(" ".join(bits))
            return 0
        by = a.by or who_am_i()
        if a.cmd == "cancel":
            code, d = _call(http, "POST", "/shutdown/cancel", {"by": by, "reason": a.reason})
            if code != 200:
                print(f"error: {code}: {d.get('detail')}", file=sys.stderr)
                return 2
            print("cancelled. " + fmt_shutdown(d))
            return 0
        # shutdown
        confirm = "shutdown"
        if not a.yes:
            tty = sys.stdin.isatty() if isatty is None else isatty
            if not tty:
                print("error: not a terminal; add --yes to confirm", file=sys.stderr)
                return 2
            print(f"This powers the Pi off in {a.delay:.0f} s: every actuated node is e-stopped, then the UPS cuts\n"
                  "power and the Pi shuts down. The robot is unreachable until it is powered on again.")
            try:
                confirm = ask("Type 'shutdown' to confirm: ").strip()
            except EOFError:
                confirm = ""
            if confirm != "shutdown":
                print("not confirmed; nothing scheduled")
                return 1
        code, d = _call(http, "POST", "/shutdown", {"confirm": confirm, "reason": a.reason, "by": by, "delay_s": a.delay})
        if code != 202:
            print(f"error: {code}: {d.get('detail')}", file=sys.stderr)
            return 2
        print(fmt_shutdown(d))
        return 0
    finally:
        if client is None:
            http.close()


if __name__ == "__main__":
    sys.exit(main())
