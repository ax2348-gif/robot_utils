"""Latency profile of the swerve node (WHEELS OFF THE GROUND: it drives slowly).

    python examples/swerve_node_profile.py --url http://botcore-desktop:8102 --seconds 15

Measures, with the client clock synchronised to the node's (GET /time):
- tick -> client: client receive time (mapped to the server clock) minus the row's t_tick;
- POST /command round trip;
- command -> applied: from sending a command (server clock) to the first tick row that carries
  its seq (cmd_seq), i.e. the tick that sent it to the motors;
- the control loop rate and dropped rows.
Commands a 0.1 m/s forward twist at 20 Hz (seq = 1, 2, ...), then disables and releases.
"""
from __future__ import annotations

import argparse
import json
import statistics
import threading
import time

from robot_utils.nodes.common.protocol import RowsDecoder
from robot_utils.nodes.swerve.client import SwerveClient


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(len(xs) * p))] if xs else float("nan")


def summary(xs, scale=1e3):
    return {"n": len(xs), "p50": round(pct(xs, .5) * scale, 2), "p95": round(pct(xs, .95) * scale, 2),
            "p99": round(pct(xs, .99) * scale, 2), "max": round(max(xs) * scale, 2) if xs else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://localhost:8102")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--speed", type=float, default=0.1)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    base = SwerveClient(args.url)
    est = base.sync(32)
    stream_client = SwerveClient(args.url)
    stream_client.clock = base.clock
    tick_lat, ticks, dropped, first_seen = [], [], [0], {}
    stop = threading.Event()

    def reader():
        dec = RowsDecoder()
        for h, p, t_rx in stream_client.records("/stream"):
            rows = dec.decode(h, p)
            dropped[0] += int(h.get("dropped", 0) or 0)
            if rows is not None:
                t_srv = base.clock.to_server(t_rx)
                for r in rows:
                    tick_lat.append(t_srv - r["t_tick"])
                    ticks.append(r["t_tick"])
                    s = int(r["cmd_seq"])
                    if s and s not in first_seen and r["cmd_source"] == 1 and r["motors_enabled"] == 1:
                        first_seen[s] = r["t_tick"]
            if stop.is_set():
                return
    th = threading.Thread(target=reader, daemon=True)
    th.start()
    time.sleep(1.0)

    sent, rtt = {}, []
    with base.control(f"profile {args.label}".strip()):
        seq, end = 0, time.monotonic() + args.seconds
        while time.monotonic() < end:
            seq += 1
            t0 = time.monotonic()
            base.command(args.speed, 0.0, 0.0, seq=seq)
            t1 = time.monotonic()
            sent[seq] = base.clock.to_server(t0)
            rtt.append(t1 - t0)
            time.sleep(max(0.0, 0.05 - (time.monotonic() - t0)))
        base.command(0.0, 0.0, 0.0)
    time.sleep(0.5)
    stop.set()

    apply = [first_seen[s] - sent[s] for s in sent if s in first_seen]
    dt = [b - a for a, b in zip(ticks, ticks[1:]) if b - a < 0.5]
    out = {
        "label": args.label, "url": args.url,
        "sync_uncertainty_ms": round(est.uncertainty * 1e3, 3), "sync_rtt_min_ms": round(est.rtt_min * 1e3, 3),
        "loop_hz": round(1 / statistics.median(dt), 2) if dt else None,
        "tick_period_ms": summary(dt),
        "tick_to_client_ms": summary(tick_lat),
        "command_rtt_ms": summary(rtt),
        "command_to_applied_ms": summary(apply),
        "commands_seen": f"{len(apply)}/{len(sent)}",
        "rows": len(ticks), "dropped": dropped[0],
    }
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
