#!/usr/bin/env python3
"""Control-stream latency from a remote machine (run on erid, over the robot LAN), for a "system 0" there:

  erid_loop_probe.py PASSIVE_S ACTIVE_S [OUT.json]

1. Passive: subscribes to arm/stream, swerve/stream and estimator/stream at full rate and, per row, measures its age
   on arrival: local arrival (mapped to the Pi's clock through each node's GET /time) minus the row's own time.
2. Active (stationary; supervised): takes the arm's lease, enables it (the gripper check closes the gripper), reads
   the TCP pose, then for ACTIVE_S at 100 Hz: reads the newest arm row, POSTs /track with that same pose and the next
   seq (the arm does not move), and watches the arm stream for the first tick where that seq is in force.
   Loop latency = t_tick of that tick - t_tick of the row the command was sent after (both on the Pi's clock: no
   sync needed). Also the POST round trip and how many commands were superseded before a tick took them. Stops if
   any joint drifts > 0.05 rad; leaves the arm disabled and the lease released.
Read-only except the arm lease and the stationary track.
"""
import http.client, json, os, sys, threading, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))
import numpy as np  # noqa: E402
from robot_utils.nodes.common.protocol import RecordReader  # noqa: E402

HOST, PORT, PREFIX = "192.168.8.209", 80, "/nodes"
PASSIVE = float(sys.argv[1]) if len(sys.argv) > 1 else 15
ACTIVE = float(sys.argv[2]) if len(sys.argv) > 2 else 30
OUT = sys.argv[3] if len(sys.argv) > 3 else None
stop = threading.Event()
lat = {n: [] for n in ("arm", "swerve", "estimator")}
gaps = {n: 0.0 for n in lat}
latest = {"row": None, "fields": None}
seq_seen: dict = {}          # seq -> first t_tick it was in force
lock = threading.Lock()


def conn():
    return http.client.HTTPConnection(HOST, PORT, timeout=10)


def req(c, method, path, body=None, token=None):
    h = {"Content-Type": "application/json"}
    if token:
        h["X-Lease-Token"] = token
    c.request(method, PREFIX + path, body=None if body is None else json.dumps(body), headers=h)
    r = c.getresponse()
    data = r.read()
    if r.status >= 400:
        raise RuntimeError(f"{method} {path}: {r.status} {data[:200]!r}")
    return json.loads(data) if data else None


def offset(node):
    """Pi monotonic - local monotonic (best of 15 round trips)."""
    c, best = conn(), None
    for _ in range(15):
        t0 = time.monotonic()
        m = req(c, "GET", f"/{node}/time")["mono"]
        t1 = time.monotonic()
        if best is None or t1 - t0 < best[0]:
            best = (t1 - t0, m - (t0 + t1) / 2)
    return best[1], best[0]


def follow(node, off):
    c = conn()
    c.request("GET", f"{PREFIX}/{node}/stream")
    r = c.getresponse()
    rr, schemas, last = RecordReader(), {}, None
    while not stop.is_set():
        chunk = r.read1(1 << 16)
        if not chunk:
            break
        now = time.monotonic()
        if last is not None:
            gaps[node] = max(gaps[node], now - last)
        last = now
        for h, payload in rr.feed(chunk):
            if h.get("type") == "schema":
                schemas[h["schema"]] = h["fields"]
                continue
            if h.get("type") != "rows":
                continue
            fields = h.get("fields") or schemas.get(h.get("schema"))
            if not fields:
                continue
            a = np.frombuffer(bytes(payload), "<f8").reshape(-1, len(fields))
            tcol = next(i for i, f in enumerate(fields) if f.startswith("t"))
            for row in a:
                lat[node].append(now + off - row[tcol])
            if node == "arm":
                si, ti = fields.index("seq"), fields.index("t_tick")
                with lock:
                    latest["row"], latest["fields"] = a[-1], fields
                    for row in a:
                        s = int(row[si])
                        if s and s not in seq_seen:
                            seq_seen[s] = row[ti]


def pct(a, q):
    a = sorted(a)
    return round(1e3 * a[min(len(a) - 1, int(q / 100 * len(a)))], 1) if a else None


def stats(a):
    return {"n": len(a), "p50": pct(a, 50), "p95": pct(a, 95), "p99": pct(a, 99), "max": pct(a, 100)}


def active():
    c = conn()
    tok = req(c, "POST", "/arm/lease", {"client": "erid loop probe (stationary track)"})["token"]
    sent, rtt, drift_stop = [], [], False
    try:
        req(c, "POST", "/arm/enable", {}, tok)
        tcp = req(c, "GET", "/arm/kinematics")["tcp"]
        pose = {"pos": tcp["pos"], "quat": tcp["quat"]}
        with lock:
            f = latest["fields"]
            q0 = np.array([latest["row"][f.index(f"j{i}_q")] for i in range(1, 7)])
        k, t_end, period = 0, time.monotonic() + ACTIVE, 0.01
        nxt = time.monotonic()
        while time.monotonic() < t_end:
            with lock:
                row, f = latest["row"], latest["fields"]
            q = np.array([row[f.index(f"j{i}_q")] for i in range(1, 7)])
            if np.max(np.abs(q - q0)) > 0.05:
                drift_stop = True
                print("WATCHDOG: joint drift > 0.05 rad; stopping", flush=True)
                break
            k += 1
            basis = row[f.index("t_tick")]
            t0 = time.monotonic()
            r = req(c, "POST", "/arm/track", {"pose": pose, "seq": k}, tok)
            rtt.append(time.monotonic() - t0)
            if r.get("ok"):
                sent.append((k, basis))
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
        time.sleep(0.3)                                   # the last commands reach a tick
    finally:
        for p in ("/arm/disable",):
            try:
                req(c, "POST", p, {}, tok)
            except Exception as e:  # noqa: BLE001
                print("cleanup", p, e, flush=True)
        try:
            req(c, "DELETE", "/arm/lease", None, tok)
        except Exception as e:  # noqa: BLE001
            print("cleanup lease", e, flush=True)
    with lock:
        loop = [seq_seen[k] - b for k, b in sent if k in seq_seen]
        superseded = sum(1 for k, _ in sent if k not in seq_seen)
    return {"commands": len(sent), "rate_hz": round(len(sent) / ACTIVE, 1), "superseded": superseded,
            "loop_ms": stats(loop), "post_rtt_ms": stats(rtt), "drift_stop": drift_stop}


def main():
    offs = {}
    for n in lat:
        offs[n] = offset(n)
        print(f"{n} clock offset {offs[n][0]:.4f} s (best round trip {offs[n][1] * 1e3:.1f} ms)", flush=True)
    ths = [threading.Thread(target=follow, args=(n, offs[n][0]), daemon=True) for n in lat]
    for t in ths:
        t.start()
    time.sleep(PASSIVE)
    passive = {n: {**stats(v), "max_gap_ms": round(1e3 * gaps[n], 1)} for n, v in lat.items()}
    for n in lat:
        lat[n].clear()
    act = active() if ACTIVE > 0 else None
    report = {"passive_row_age_ms": passive,
              "active_row_age_ms": {n: stats(v) for n, v in lat.items()},
              "active_loop": act,
              "clock_sync_rtt_ms": {n: round(o[1] * 1e3, 1) for n, o in offs.items()}}
    stop.set()
    print(json.dumps(report, indent=1), flush=True)
    if OUT:
        json.dump(report, open(OUT, "w"), indent=1)


if __name__ == "__main__":
    main()
