#!/usr/bin/env python3
"""Where the Pi's CPU goes besides user code, for one window (run on the Pi; read-only, no privileges):

  python3 pi_io_sample.py 30

- per node: user vs kernel (system) CPU %, voluntary / involuntary context switches per second;
- per thread of each node: CPU % and run-queue wait % (from /proc/<pid>/task/<tid>/schedstat: time runnable
  but not running, i.e. waiting for a core), scheduling policy, and the top threads;
- system: CPU split user / nice / system / irq / softirq / idle / iowait, context switches/s,
  interrupts/s by source (/proc/interrupts), softirqs/s by type (/proc/softirqs).
"""
import json
import os
import sys
import time

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
TCK = os.sysconf("SC_CLK_TCK")


def nodes():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        if b"--name" in cmd and any(b"robot_utils" in c for c in cmd):
            out[cmd[cmd.index(b"--name") + 1].decode()] = int(pid)
    return out


def pstat(path):
    f = open(path).read().rsplit(")", 1)[1].split()
    return int(f[11]), int(f[12]), int(f[38])          # utime, stime, policy (0 other, 1 FIFO, 2 RR)


def status_ctx(path):
    v = n = 0
    for line in open(path):
        if line.startswith("voluntary_ctxt_switches"):
            v = int(line.split()[1])
        elif line.startswith("nonvoluntary_ctxt_switches"):
            n = int(line.split()[1])
    return v, n


def threads(pid):
    out = {}
    for tid in os.listdir(f"/proc/{pid}/task"):
        b = f"/proc/{pid}/task/{tid}"
        try:
            run_ns, wait_ns, slices = map(int, open(f"{b}/schedstat").read().split())
            ut, st, pol = pstat(f"{b}/stat")
            comm = open(f"{b}/comm").read().strip()
            v, n = status_ctx(f"{b}/status")
            out[int(tid)] = (run_ns, wait_ns, slices, ut, st, pol, comm, v, n)
        except OSError:
            pass
    return out


def snap(nd):
    s = {"stat": {}, "irq": {}, "softirq": {}, "nodes": {}}
    for line in open("/proc/stat"):
        p = line.split()
        if p[0] == "cpu":
            s["stat"]["cpu"] = list(map(int, p[1:8]))
        elif p[0] in ("ctxt", "intr"):
            s["stat"][p[0]] = int(p[1])
    lines = open("/proc/interrupts").read().splitlines()
    ncpu = len(lines[0].split())
    for line in lines[1:]:
        p = line.split()
        key = p[0].rstrip(":")
        cnt = sum(int(x) for x in p[1:1 + ncpu] if x.isdigit())
        name = " ".join(p[1 + ncpu:])[-48:] if len(p) > 1 + ncpu else key
        s["irq"][f"{key} {name}".strip()] = cnt
    for line in open("/proc/softirqs").read().splitlines()[1:]:
        p = line.split()
        s["softirq"][p[0].rstrip(":")] = sum(int(x) for x in p[1:])
    for n, pid in nd.items():
        try:
            ths = threads(pid)
            s["nodes"][n] = {"stat": pstat(f"/proc/{pid}/stat"), "threads": ths,
                             "ctx": (sum(t[7] for t in ths.values()), sum(t[8] for t in ths.values()))}
        except OSError:
            pass
    return s


def main():
    nd = nodes()
    a = snap(nd)
    t0 = time.monotonic()
    time.sleep(DUR)
    dt = time.monotonic() - t0
    b = snap(nd)
    c0, c1 = a["stat"]["cpu"], b["stat"]["cpu"]
    tot = sum(c1) - sum(c0)
    split = {k: round(100 * (c1[i] - c0[i]) / tot, 1)
             for i, k in enumerate(["user", "nice", "system", "idle", "iowait", "irq", "softirq"])}
    out = {"seconds": round(dt, 1), "cpu_split_pct_of_4_cores": split,
           "ctx_switches_s": round((b["stat"]["ctxt"] - a["stat"]["ctxt"]) / dt),
           "interrupts_s": round((b["stat"]["intr"] - a["stat"]["intr"]) / dt),
           "irq_top_s": dict(sorted(((k, round((b["irq"][k] - a["irq"].get(k, 0)) / dt)) for k in b["irq"]),
                                    key=lambda kv: -kv[1])[:10]),
           "softirq_s": {k: round((b["softirq"][k] - a["softirq"][k]) / dt) for k in b["softirq"]},
           "nodes": {}}
    for n in sorted(b["nodes"]):
        if n not in a["nodes"]:
            continue
        (u0, s0, _), (u1, s1, _) = a["nodes"][n]["stat"], b["nodes"][n]["stat"]
        common = set(a["nodes"][n]["threads"]) & set(b["nodes"][n]["threads"])
        v0 = sum(a["nodes"][n]["threads"][t][7] for t in common); v1 = sum(b["nodes"][n]["threads"][t][7] for t in common)
        i0 = sum(a["nodes"][n]["threads"][t][8] for t in common); i1 = sum(b["nodes"][n]["threads"][t][8] for t in common)
        ths = []
        for tid, t1 in b["nodes"][n]["threads"].items():
            t_0 = a["nodes"][n]["threads"].get(tid)
            if t_0 is None:
                continue
            run = (t1[0] - t_0[0]) / 1e9 / dt
            wait = (t1[1] - t_0[1]) / 1e9 / dt
            sl = (t1[2] - t_0[2]) / dt
            if run > 0.005 or wait > 0.005:
                ths.append({"tid": tid, "comm": t1[6], "policy": {0: "other", 1: "FIFO", 2: "RR"}.get(t1[5], t1[5]),
                            "cpu_pct": round(100 * run, 1), "runq_wait_pct": round(100 * wait, 1),
                            "wait_per_slice_ms": round(1e3 * wait / sl, 2) if sl else None,
                            "slices_s": round(sl), "vol_s": round((t1[7] - t_0[7]) / dt),
                            "invol_s": round((t1[8] - t_0[8]) / dt)})
        ths.sort(key=lambda x: -x["cpu_pct"])
        out["nodes"][n] = {"user_pct": round(100 * (u1 - u0) / TCK / dt, 1),
                           "system_pct": round(100 * (s1 - s0) / TCK / dt, 1),
                           "vol_ctx_s": round((v1 - v0) / dt), "invol_ctx_s": round((i1 - i0) / dt),
                           "threads_total": len(b["nodes"][n]["threads"]),
                           "runq_wait_pct_sum": round(sum(t["runq_wait_pct"] for t in ths), 1),
                           "top_threads": ths[:6]}
    print(json.dumps(out))


if __name__ == "__main__":
    main()
