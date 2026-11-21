#!/usr/bin/env python3
"""Summarise a usbmon text capture of the arm's adapter (bus 2): python3 parse_usbmon.py root_LABEL_usbmon2.txt

usbmon 'u' format: URB-tag timestamp(us) event(S/C/E) type:bus:dev:ep status length ...
For bulk OUT (Bo) URBs: submit->complete time (the adapter's ACK + host completion; stop-and-wait shows as one
URB at a time), how many were outstanding at once, bytes per URB. For bulk IN (Bi): completion intervals and bytes
(how replies arrive). Prints the distributions and the max concurrency.
"""
import sys
from collections import defaultdict

import statistics as st


def pct(a, q):
    if not a:
        return None
    a = sorted(a)
    return round(a[min(len(a) - 1, int(q / 100 * len(a)))], 1)


def main(path):
    submit, out_lat, out_len, in_done, in_len = {}, [], [], [], []
    inflight = maxin = 0
    gaps_between_out_submits, last_out_submit = [], None
    for line in open(path):
        p = line.split()
        if len(p) < 5:
            continue
        tag, ts, ev, addr = p[0], int(p[1]), p[2], p[3]
        kind = addr.split(":")[0]
        if kind == "Bo":
            if ev == "S":
                submit[tag] = ts
                inflight += 1
                maxin = max(maxin, inflight)
                if last_out_submit is not None:
                    gaps_between_out_submits.append(ts - last_out_submit)
                last_out_submit = ts
                try:
                    out_len.append(int(p[5]))
                except (IndexError, ValueError):
                    pass
            elif ev in ("C", "E") and tag in submit:
                out_lat.append(ts - submit.pop(tag))
                inflight = max(0, inflight - 1)
        elif kind == "Bi" and ev == "C":
            in_done.append(ts)
            try:
                in_len.append(int(p[5]))
            except (IndexError, ValueError):
                pass
    in_gaps = [b - a for a, b in zip(in_done, in_done[1:])]
    dur_s = (in_done[-1] - in_done[0]) / 1e6 if len(in_done) > 1 else None
    print(f"bulk OUT URBs: {len(out_lat)}  ({len(out_lat) / dur_s:.0f}/s)" if dur_s else f"bulk OUT URBs: {len(out_lat)}")
    print(f"  submit->complete us: p10 {pct(out_lat, 10)} p50 {pct(out_lat, 50)} p90 {pct(out_lat, 90)} "
          f"p99 {pct(out_lat, 99)} max {max(out_lat) if out_lat else None}")
    print(f"  max outstanding at once: {maxin}   (1 = strict stop-and-wait)")
    print(f"  bytes per URB: {sorted(set(out_len))[:8]}")
    print(f"  gap between OUT submits us: p50 {pct(gaps_between_out_submits, 50)} p90 {pct(gaps_between_out_submits, 90)}")
    print(f"bulk IN completions: {len(in_done)}  bytes: p50 {pct(in_len, 50)} max {max(in_len) if in_len else None}")
    print(f"  interval between IN completions us: p50 {pct(in_gaps, 50)} p90 {pct(in_gaps, 90)} max {max(in_gaps) if in_gaps else None}")


if __name__ == "__main__":
    main(sys.argv[1])
