#!/usr/bin/env python3
"""Wait for the Quest's telemetry stream, record it, exit when the headset goes away (run on the Mac or erid).

  python3 tools/quest_teleop/quest_probe.py OUT_DIR [--host 100.72.77.83] [--port 9391] [--max-s 7200]
  python3 tools/quest_teleop/quest_probe.py OUT_DIR --summarize

Never starts anything on the headset: it only connects to the stream port when telemetry is already running (start
it on the headset, or with quest_up, while wearing it). Writes OUT_DIR/records.jsonl (every record, as received,
with our arrival time) and OUT_DIR/summary.json (the field tree with types, rate, schemes seen, session / frame_seq
continuity, grip engagements, focus losses). Exits once the stream has been gone for 60 s after data was seen (a
headset that is asleep or off its head still answers SSH, so SSH is no signal), on Ctrl-C, or after --max-s; the
summary is written on every exit. `--summarize OUT_DIR` rebuilds summary.json from an existing records.jsonl.
"""
import argparse, json, os, socket, time
from collections import Counter

GONE_S = 60.0


def ssh_host():
    import subprocess
    try:
        out = subprocess.run(["ssh", "-G", "quest"], capture_output=True, text=True, timeout=5).stdout
        cfg = dict(l.split(" ", 1) for l in out.splitlines() if " " in l)
        return cfg.get("hostname")
    except Exception:  # noqa: BLE001
        return None


def walk(obj, prefix, out):
    if isinstance(obj, dict):
        for k, v in obj.items():
            walk(v, f"{prefix}.{k}" if prefix else k, out)
    elif isinstance(obj, list):
        out[prefix + "[]"] += 1
        if obj and isinstance(obj[0], (dict, list)):
            walk(obj[0], prefix + "[]", out)
    else:
        out[f"{prefix}:{type(obj).__name__}"] += 1


class Stats:
    def __init__(self):
        self.fields, self.schemes, self.sessions = Counter(), Counter(), Counter()
        self.n, self.first, self.last, self.connects = 0, None, None, 0
        self.seq_gaps = self.seq_back = self.focus_losses = self.engagements = 0
        self.prev_seq, self.prev_focus, self.prev_engaged = None, None, {}
        self.max_gap_s = 0.0

    def add(self, r, rx):
        self.n += 1
        if self.last is not None:
            self.max_gap_s = max(self.max_gap_s, rx - self.last)
        self.first = self.first or rx
        self.last = rx
        walk(r, "", self.fields)
        ci = r.get("control_intent") or {}
        self.schemes[str(ci.get("scheme"))] += 1
        self.sessions[str(r.get("session_id"))] += 1
        fs = r.get("frame_seq")
        if isinstance(fs, int) and isinstance(self.prev_seq, int):
            if fs > self.prev_seq + 1:
                self.seq_gaps += fs - self.prev_seq - 1
            elif fs <= self.prev_seq:
                self.seq_back += 1
        if isinstance(fs, int):
            self.prev_seq = fs
        foc = r.get("input_focused")
        if self.prev_focus and not foc:
            self.focus_losses += 1
        self.prev_focus = foc
        for side, h in (ci.get("hands") or {}).items():
            e = bool((h or {}).get("engaged"))
            if e and not self.prev_engaged.get(side):
                self.engagements += 1
            self.prev_engaged[side] = e

    def summary(self, host):
        dur = (self.last - self.first) if self.n > 1 else 0.0
        return {"host": host, "records": self.n, "seconds": round(dur, 1),
                "rate_hz": round((self.n - 1) / dur, 1) if dur else None, "max_gap_s": round(self.max_gap_s, 3),
                "connects": self.connects, "schemes": dict(self.schemes), "sessions": len(self.sessions),
                "frame_seq_gaps": self.seq_gaps, "frame_seq_backwards": self.seq_back,
                "focus_losses": self.focus_losses, "engagements": self.engagements,
                "fields": dict(sorted(self.fields.items()))}


def write_summary(out, st, host):
    summary = st.summary(host)
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k != "fields"}), flush=True)


def summarize(out):
    st = Stats()
    with open(os.path.join(out, "records.jsonl")) as f:
        for line in f:
            try:
                d = json.loads(line)
            except ValueError:
                continue  # a torn last line from a killed run
            st.add(d["record"], d["rx"])
    write_summary(out, st, None)


def record(out, host, port, max_s):
    st = Stats()
    t0 = time.monotonic()
    gone_since = time.monotonic()
    print(f"waiting for the Quest stream at {host}:{port}", flush=True)
    with open(os.path.join(out, "records.jsonl"), "a") as rec:
        try:
            while time.monotonic() - t0 < max_s:
                try:
                    s = socket.create_connection((host, port), timeout=2.0)
                except OSError:
                    if st.n and time.monotonic() - gone_since > GONE_S:
                        print(f"stream gone for {GONE_S:.0f} s: done", flush=True)
                        break
                    time.sleep(2.0)
                    continue
                st.connects += 1
                print(f"connected ({st.connects}); recording", flush=True)
                s.settimeout(5.0)
                buf = b""
                try:
                    while time.monotonic() - t0 < max_s:
                        chunk = s.recv(65536)
                        if not chunk:
                            break
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            if not line.strip():
                                continue
                            try:
                                r = json.loads(line)
                            except ValueError:
                                continue
                            now = time.time()
                            rec.write(json.dumps({"rx": now, "record": r}) + "\n")
                            st.add(r, now)
                            if st.n % 50 == 0:
                                rec.flush()
                except OSError:
                    pass
                finally:
                    s.close()
                print("stream closed", flush=True)
                gone_since = time.monotonic()
        except KeyboardInterrupt:
            print("interrupted", flush=True)
    write_summary(out, st, host)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--host")
    ap.add_argument("--port", type=int, default=9391)
    ap.add_argument("--max-s", type=float, default=7200)
    ap.add_argument("--summarize", action="store_true", help="rebuild summary.json from OUT/records.jsonl and exit")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.summarize:
        summarize(a.out)
    else:
        record(a.out, a.host or ssh_host() or "100.72.77.83", a.port, a.max_s)


if __name__ == "__main__":
    main()
