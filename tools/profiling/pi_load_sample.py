#!/usr/bin/env python3
"""Profile the Pi's load for one condition: run ON THE PI with the robot venv, e.g.

  ~/robot_utils/.venv/bin/python pi_load_sample.py 60 baseline > baseline.json

Read-only: subscribes to the arm, swerve and estimator streams like any consumer, sends GET /health to every
node, and reads /proc. Commands nothing. Reports, for the window:
  - system: CPU busy/iowait per core, PSI (cpu/io/memory "some"/"full" %), SD card IO (/proc/diskstats),
    temperature, CPU clock;
  - per node: CPU % (all threads), and GET /health latency p50/p95/max (event-loop responsiveness);
  - streams: source loop jitter (gaps between rows' t), delivery lag (receipt - t, same clock), and for the
    arm read_age / read_misses / overruns, for the swerve each motor's reply age (t_tick - t_<motor>).
"""
import http.client
import json
import os
import socket
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.protocol import RecordReader, RowsDecoder  # noqa: E402

RUN = f"/run/user/{os.getuid()}/robot"
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LABEL = sys.argv[2] if len(sys.argv) > 2 else "sample"
TCK = os.sysconf("SC_CLK_TCK")
STOP = threading.Event()


class UConn(http.client.HTTPConnection):
    def __init__(self, sock_path, timeout=5.0):
        super().__init__("localhost", timeout=timeout)
        self.sock_path = sock_path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.sock_path)


def nodes():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        if b"robot_utils.nodes." in b" ".join(cmd) and b"--name" in cmd:
            out[cmd[cmd.index(b"--name") + 1].decode()] = int(pid)
    return out


def proc_cpu(pid):
    try:
        f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
        return (int(f[11]) + int(f[12])) / TCK
    except OSError:
        return None


def cpu_times():
    out = {}
    for line in open("/proc/stat"):
        if line.startswith("cpu"):
            p = line.split()
            v = list(map(int, p[1:8]))       # user nice system idle iowait irq softirq
            out[p[0]] = v
    return out


def psi():
    out = {}
    for k in ("cpu", "io", "memory"):
        for line in open(f"/proc/pressure/{k}"):
            p = line.split()
            out[f"{k}_{p[0]}"] = int(p[-1].split("=")[1])      # total, microseconds
    return out


def disk():
    for line in open("/proc/diskstats"):
        p = line.split()
        if p[2] == "mmcblk0":
            # reads, sectors read, writes, sectors written, ms doing IO, weighted ms
            return [int(p[3]), int(p[5]), int(p[7]), int(p[9]), int(p[12]), int(p[13])]
    return [0] * 6


def pct(a, q):
    return round(float(np.percentile(a, q)), 4) if len(a) else None


def stream(name, params, sink):
    """Collect (receipt monotonic, rows) from GET /stream until STOP."""
    try:
        c = UConn(f"{RUN}/{name}.sock", timeout=10)
        c.request("GET", "/stream?" + "&".join(f"{k}={v}" for k, v in params.items()))
        r = c.getresponse()
        rr, dec = RecordReader(), RowsDecoder()
        while not STOP.is_set():
            chunk = r.read1(65536)
            if not chunk:
                break
            now = time.monotonic()
            for h, payload in rr.feed(chunk):
                rows = dec.decode(h, payload)
                if rows is not None:
                    sink.append((now, rows))
        c.close()
    except Exception as e:  # noqa: BLE001
        sink.append(("error", f"{type(e).__name__}: {e}"))


def prober(names, lat):
    i = 0
    while not STOP.is_set():
        n = names[i % len(names)]
        i += 1
        t = time.monotonic()
        try:
            c = UConn(f"{RUN}/{n}.sock", timeout=5)
            c.request("GET", "/health")
            c.getresponse().read()
            c.close()
            lat.setdefault(n, []).append(time.monotonic() - t)
        except Exception:  # noqa: BLE001
            lat.setdefault(n, []).append(5.0)
        STOP.wait(0.1)


def summarize_stream(sink, tfield):
    errs = [x[1] for x in sink if x[0] == "error"]
    parts = [x for x in sink if x[0] != "error"]
    if not parts:
        return {"error": errs[0] if errs else "no rows"}, None
    recv = np.concatenate([np.full(len(r), t) for t, r in parts])
    rows = np.concatenate([r for _, r in parts])
    t = rows[tfield].astype(float)
    gaps = np.diff(t)
    lag = recv - t
    out = {"rows": int(len(rows)), "rate_hz": round(len(rows) / max(t[-1] - t[0], 1e-9), 1),
           "gap_ms": {"p50": pct(gaps * 1e3, 50), "p99": pct(gaps * 1e3, 99), "max": pct(gaps * 1e3, 100),
                      "n_over_50": int((gaps > 0.05).sum()), "n_over_100": int((gaps > 0.1).sum())},
           "lag_ms": {"p50": pct(lag * 1e3, 50), "p95": pct(lag * 1e3, 95), "p99": pct(lag * 1e3, 99),
                      "max": pct(lag * 1e3, 100)}}
    if errs:
        out["errors"] = errs
    return out, rows


def main():
    nd = nodes()
    t0 = time.monotonic()
    c0, p0, d0 = cpu_times(), psi(), disk()
    n0 = {n: proc_cpu(p) for n, p in nd.items()}
    sinks = {"arm": [], "swerve": [], "estimator": []}
    ths = [threading.Thread(target=stream, args=("arm", {"max_hz": 100}, sinks["arm"]), daemon=True),
           threading.Thread(target=stream, args=("swerve", {"max_hz": 100}, sinks["swerve"]), daemon=True),
           threading.Thread(target=stream, args=("estimator", {}, sinks["estimator"]), daemon=True)]
    lat = {}
    ths.append(threading.Thread(target=prober, args=(sorted(nd), lat), daemon=True))
    for th in ths:
        th.start()
    temps, freqs = [], []
    while time.monotonic() - t0 < DUR:
        try:
            temps.append(int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000)
            freqs.append(int(open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").read()) / 1e6)
        except OSError:
            pass
        time.sleep(1.0)
    STOP.set()
    dt = time.monotonic() - t0
    c1, p1, d1 = cpu_times(), psi(), disk()
    n1 = {n: proc_cpu(p) for n, p in nd.items()}
    cores = {}
    for k in c1:
        a, b = c0[k], c1[k]
        tot = sum(b) - sum(a)
        busy = tot - (b[3] - a[3]) - (b[4] - a[4])
        cores[k] = {"busy_pct": round(100 * busy / tot, 1), "iowait_pct": round(100 * (b[4] - a[4]) / tot, 1),
                    "irq_pct": round(100 * ((b[5] - a[5]) + (b[6] - a[6])) / tot, 1)}
    dd = [b - a for a, b in zip(d0, d1)]
    out = {"label": LABEL, "seconds": round(dt, 1), "wall": time.strftime("%Y-%m-%d %H:%M:%S"),
           "cpu": cores, "psi_pct": {k: round(100 * (p1[k] - p0[k]) / 1e6 / dt, 2) for k in p1},
           "disk": {"reads_s": round(dd[0] / dt, 1), "read_kB_s": round(dd[1] / 2 / dt, 1),
                    "writes_s": round(dd[2] / dt, 1), "write_kB_s": round(dd[3] / 2 / dt, 1),
                    "util_pct": round(100 * dd[4] / 1000 / dt, 1),
                    "avg_wait_ms": round(dd[5] / max(dd[0] + dd[2], 1), 1)},
           "temp_c_max": max(temps) if temps else None, "ghz_min": min(freqs) if freqs else None,
           "node_cpu_pct": {n: round(100 * (n1[n] - n0[n]) / dt, 1) for n in nd
                            if n0.get(n) is not None and n1.get(n) is not None},
           "health_ms": {n: {"n": len(v), "p50": pct(np.array(v) * 1e3, 50), "p95": pct(np.array(v) * 1e3, 95),
                             "max": pct(np.array(v) * 1e3, 100)} for n, v in sorted(lat.items())}}
    out["node_cpu_total_pct"] = round(sum(out["node_cpu_pct"].values()), 1)
    s, rows = summarize_stream(sinks["arm"], "t_tick")
    if rows is not None:
        td, ra = rows["tick_dt"] * 1e3, rows["read_age"] * 1e3
        s.update({"tick_dt_ms": {"p50": pct(td, 50), "p99": pct(td, 99), "max": pct(td, 100)},
                  "read_age_ms": {"p99": pct(ra, 99), "max": pct(ra, 100)},
                  "read_misses": int(rows["read_misses"][-1] - rows["read_misses"][0]),
                  "overruns": int(rows["overruns"][-1] - rows["overruns"][0]),
                  "state": sorted({int(x) for x in rows["state"]}), "enabled": int(rows["enabled"].max())})
    out["arm"] = s
    s, rows = summarize_stream(sinks["swerve"], "t_tick")
    if rows is not None:
        ages = []
        for f in rows.dtype.names:
            if f.startswith(("FL_t_", "FR_t_", "RL_t_", "RR_t_")) and not f.endswith("_s0"):
                ages.append(rows["t_tick"] - rows[f])
        if ages:
            a = np.nanmax(np.vstack(ages), axis=0) * 1e3
            s["motor_reply_age_ms"] = {"p50": pct(a, 50), "p99": pct(a, 99), "max": pct(a, 100),
                                       "n_over_50": int((a > 50).sum())}
        s["loop_ok_min"] = float(rows["loop_ok"].min())
        s["enabled"] = int(rows["enabled"].max())
    out["swerve"] = s
    out["estimator"], _ = summarize_stream(sinks["estimator"], "t")
    print(json.dumps(out))


if __name__ == "__main__":
    main()
