#!/usr/bin/env python3
"""No-motion soak of the whole control core with a live motion watchdog (run on the Pi).

    ~/robot_utils/.venv/bin/python tools/profiling/wbc_soak.py [--duration 600] [--baseline 20] [--scenario idle]

Everything the teleop stack runs is live, but the WBC is never armed, so it holds: the arm servos at its measured
pose, the swerve gets a zero twist every tick (motors energized, steering held). The run:

1. preflight: the WBC disabled with no lease, nothing e-stopped, no other soak running;
2. baseline (--baseline s, nothing enabled): the watchdog runs and its snapshot is taken; motion here = something
   else is moving the robot: abort;
3. a fake headset (tools/quest_teleop/fake_headset.py, never presses Y) and a second quest_teleop instance
   (--teleop, default ``quest_soak``) pointed at it; POST <teleop>/start: the WBC enables (arm + swerve) and
   takes ~72 Hz datagrams (armed false) from the client, as in teleop;
4. the soak (--duration s): a status line every --status s (WBC tick timing, overruns, late ticks, input age,
   the WBC processes' CPU); the recorder (python -m robot_utils.collect run --teleop quest_soak ...) can run
   on erid / the Mac meanwhile;
5. POST <teleop>/stop (the WBC disables), the watchdog keeps going --post s, then the verdict.

The watchdog reads the arm, swerve, estimator and WBC streams on their Unix sockets (every row, 20 ms batches) and
ABORTS (POST /estop to the WBC, the arm and the swerve at once, then checks each latched) on:

- measured motion: any arm joint off its snapshot by > --q-drift or faster than --qd-max; the gripper gap off by
  > --gap-drift (outside the enable window, when the arm's first enable runs its gripper check); any wheel's
  drive speed > --drive-vel or drive position off by > --drive-pos, steer angle off by > --steer or steer
  speed > --steer-vel; the estimator's x / y off by > --xy or theta_odom off by > --steer, or speed > --drive-vel;
- commanded motion (before it becomes motion): the WBC armed, its base twist or the swerve's requested /
  commanded twist non-zero (> --cmd-eps), the arm's commanded joints off the snapshot by > --q-drift while servoing;
- a watched stream silent for > --stale s, or an estimator reset.

Measured checks need --debounce consecutive rows over the threshold; command checks trip on one row. A WBC fault
or late tick is a soak FAILURE (reported), not an abort: the WBC already disarms and holds on those.
Exit code 0 pass, 1 fail (no motion; the core misbehaved), 2 abort (motion / stale / e-stop sent), 3 preflight.
"""
from __future__ import annotations

import argparse
import http.client
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(os.environ.get("ROBOT_UTILS_ROOT") or Path(__file__).resolve().parents[2])   # the checkout to run
if not (ROOT / "src" / "robot_utils").is_dir():
    ROOT = Path.home() / "robot_utils"
sys.path.insert(0, str(ROOT / "src"))
from robot_utils.nodes.common.protocol import RecordReader, RowsDecoder   # noqa: E402
from robot_utils.nodes.common.serve import socket_path                     # noqa: E402

WATCHED = ("arm", "swerve", "estimator", "wbc")


# -- Unix-socket HTTP ------------------------------------------------------------------------------

class UConn(http.client.HTTPConnection):
    def __init__(self, path, timeout=5.0):
        super().__init__("localhost", timeout=timeout)
        self.path_ = str(path)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path_)


def call(node, method, path, body=None, timeout=5.0):
    c = UConn(socket_path(node), timeout)
    try:
        c.request(method, path, json.dumps(body) if body is not None else None, {"Content-Type": "application/json"})
        r = c.getresponse()
        data = r.read()
    finally:
        c.close()
    out = json.loads(data) if data else {}
    if r.status >= 400:
        raise RuntimeError(f"{node} {method} {path}: {r.status} {out}")
    return out


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def log(msg):
    print(f"[soak {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# -- the watchdog ----------------------------------------------------------------------------------

class Watchdog:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        self.last = {n: None for n in WATCHED}          # monotonic time of the latest rows per stream
        self.rows = {n: 0 for n in WATCHED}
        self.snap: dict = {}                            # signal -> reference value
        self.taking_snap = True
        self.grip_ignore = False                        # the enable window (gripper check)
        self.over: dict = {}                            # signal -> consecutive rows over
        self.peak: dict = {}                            # signal -> max |deviation| seen (the noise report)
        self.tripped = None                             # (reason, detail) once
        self.trip_ev = threading.Event()
        self.stop_ev = threading.Event()
        self.wbc = {}                                   # the latest WBC row (dict)
        self.wbc_max_period = 0.0
        self.wbc_fault = False
        self.est_reset = None
        self.estop_seen = {}
        self.slack = a.baseline_slack                   # measured limits x this until the baseline is done
        self.raised: dict = {}                          # signal -> threshold raised over the baseline noise

    # one signal: deviation `dev` against threshold `lim`; measured ones debounced
    def _chk(self, name, dev, lim, measured=True):
        if not math.isfinite(dev):
            return
        dev = abs(dev)
        lim = self.raised.get(name, lim) * (self.slack if measured else 1.0)
        if dev > self.peak.get(name, 0.0):
            self.peak[name] = dev
        if dev > lim:
            n = self.over.get(name, 0) + 1
            self.over[name] = n
            if not measured or n >= self.a.debounce:
                self.trip("motion" if measured else "command", f"{name} {dev:.4g} > {lim:.4g}")
        else:
            self.over[name] = 0

    def trip(self, reason, detail):
        with self.lock:
            if self.tripped is not None:
                return
            self.tripped = (reason, detail)
        self.trip_ev.set()

    def _ref(self, name, v):
        """The snapshot of `name` (taken from the first row seen while taking_snap; retaken while unset)."""
        if name not in self.snap or (self.taking_snap and not math.isfinite(self.snap[name])):
            self.snap[name] = v
        return self.snap[name]

    def on_rows(self, node, rows, t_conn):
        a = self.a
        tcol = "t" if node == "estimator" else "t_tick"   # the node's monotonic clock (this machine's)
        if tcol in rows.dtype.names:
            rows = rows[rows[tcol] >= t_conn - 0.2]       # a stream replays its backlog on connect: not now
            if not len(rows):
                return
        with self.lock:
            self.last[node] = time.monotonic()
            self.rows[node] += len(rows)
        names = rows.dtype.names
        for r in rows:
            g = lambda k: float(r[k]) if k in names else float("nan")   # noqa: E731
            if node == "arm":
                for j in range(1, 7):
                    q = g(f"j{j}_q")
                    q0 = self._ref(f"arm.q{j}", q)
                    self._chk(f"arm.q{j}", q - q0, a.q_drift)
                    self._chk(f"arm.qd{j}", g(f"j{j}_qd"), a.qd_max)
                    if g("servo") >= 0.5:
                        self._chk(f"arm.q_cmd{j}", g(f"j{j}_q_cmd") - q0, a.q_drift, measured=False)
                gap = g("g_gap")
                if self.grip_ignore:
                    self.snap["arm.gap"] = gap
                else:
                    self._chk("arm.gap", gap - self._ref("arm.gap", gap), a.gap_drift)
                if g("estop") >= 0.5:
                    self.estop_seen["arm"] = True
            elif node == "swerve":
                for k in ("req_vx", "req_vy", "req_omega", "cmd_vx", "cmd_vy", "cmd_omega"):
                    self._chk(f"swerve.{k}", g(k), a.cmd_eps, measured=False)
                for f in names:
                    if f.endswith("_drive_vel"):
                        m = f[:-len("_drive_vel")]
                        self._chk(f"swerve.{m}.drive_vel", g(f), a.drive_vel)
                        p = g(f"{m}_drive_pos")
                        self._chk(f"swerve.{m}.drive_pos", p - self._ref(f"swerve.{m}.drive_pos", p), a.drive_pos)
                        s = g(f"{m}_steer_angle")
                        self._chk(f"swerve.{m}.steer", wrap(s - self._ref(f"swerve.{m}.steer", s)), a.steer)
                        self._chk(f"swerve.{m}.steer_vel", g(f"{m}_steer_vel"), a.steer_vel)
                if g("estop") >= 0.5:
                    self.estop_seen["swerve"] = True
            elif node == "estimator":
                rid = g("reset_id")
                if self.est_reset is None:
                    self.est_reset = rid
                elif rid != self.est_reset:
                    self.trip("estimator", f"reset_id {self.est_reset:g} -> {rid:g}")
                for k in ("x", "y"):
                    v = g(k)
                    self._chk(f"est.{k}", v - self._ref(f"est.{k}", v), a.xy)
                th = g("theta_odom")
                self._chk("est.theta_odom", wrap(th - self._ref("est.theta_odom", th)), a.steer)
                self._chk("est.speed", g("speed"), a.drive_vel)
            elif node == "wbc":
                d = {k: float(r[k]) for k in names}
                self.wbc = d
                if d.get("armed", 0) >= 0.5:
                    self.trip("command", "the WBC armed")
                for k in ("base_vx", "base_vy", "base_w"):
                    self._chk(f"wbc.{k}", d.get(k, float("nan")), a.cmd_eps, measured=False)
                self.wbc_max_period = max(self.wbc_max_period, d.get("tick_period", 0.0))
                if d.get("state") == 5:
                    self.wbc_fault = True

    def reader(self, node):
        dec, rr = RowsDecoder(), RecordReader()
        while not self.stop_ev.is_set():
            try:
                c = UConn(socket_path(node), timeout=2.0)
                t_conn = time.monotonic()
                c.request("GET", "/stream?batch_ms=20")
                resp = c.getresponse()
                if resp.status != 200:
                    raise RuntimeError(f"HTTP {resp.status}")
                while not self.stop_ev.is_set():
                    chunk = resp.read1(1 << 16)
                    if not chunk:
                        break
                    for h, p in rr.feed(chunk):
                        rows = dec.decode(h, p)
                        if rows is not None and len(rows):
                            self.on_rows(node, rows, t_conn)
            except Exception as e:  # noqa: BLE001  (staleness is the judge: reconnect)
                if not self.stop_ev.is_set():
                    log(f"{node} stream: {e}; reconnecting")
                    time.sleep(0.2)
                dec, rr = RowsDecoder(), RecordReader()

    def freshness(self, t_start):
        while not self.stop_ev.wait(0.02):
            now = time.monotonic()
            for n in WATCHED:
                last = self.last[n]
                ref = last if last is not None else t_start + 2.0   # 2 s to connect
                if now - ref > self.a.stale:
                    self.trip("stale", f"{n} stream silent {now - ref:.2f} s")

    def start(self):
        t0 = time.monotonic()
        for n in WATCHED:
            threading.Thread(target=self.reader, args=(n,), daemon=True, name=f"watch-{n}").start()
        threading.Thread(target=self.freshness, args=(t0,), daemon=True, name="watch-fresh").start()


def estop_all(reason):
    """E-stop the WBC, the arm and the swerve at once (no lease needed), then report each latch."""
    body = {"reason": f"wbc_soak watchdog: {reason}"[:200], "source": "wbc_soak"}
    res = {}

    def one(n):
        try:
            call(n, "POST", "/estop", body, timeout=1.0)
            res[n] = "sent"
        except Exception as e:  # noqa: BLE001
            res[n] = f"FAILED {e}"
    ths = [threading.Thread(target=one, args=(n,)) for n in ("swerve", "arm", "wbc")]
    for t in ths:
        t.start()
    for t in ths:
        t.join(2.0)
    for n in ("swerve", "arm", "wbc"):
        try:
            st = call(n, "GET", "/state", timeout=2.0)
            e = st.get("estop")
            latched = e.get("latched") if isinstance(e, dict) else bool(e)
            res[n] += f", latched={latched}"
        except Exception as e:  # noqa: BLE001
            res[n] += f", state unreadable: {e}"
    return res


# -- CPU of the WBC process tree (Linux /proc) -----------------------------------------------------

def wbc_pids():
    pids, parent = [], {}
    for d in os.listdir("/proc") if os.path.isdir("/proc") else []:
        if not d.isdigit():
            continue
        try:
            cmd = open(f"/proc/{d}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
            st = open(f"/proc/{d}/stat").read()
            parent[int(d)] = int(st.rsplit(")", 1)[1].split()[1])
            if "robot_utils.nodes.wbc" in cmd:
                pids.append(int(d))
        except OSError:
            continue
    out = set(pids)
    grew = True
    while grew:
        grew = False
        for p, pp in parent.items():
            if pp in out and p not in out:
                out.add(p)
                grew = True
    return sorted(out)


def cpu_ticks(pids):
    tot = 0
    for p in pids:
        try:
            f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()
            tot += int(f[11]) + int(f[12])
        except OSError:
            pass
    return tot


# -- the run ---------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=600.0, help="s of the enabled soak")
    ap.add_argument("--baseline", type=float, default=20.0, help="s watched with nothing enabled first")
    ap.add_argument("--post", type=float, default=5.0, help="s watched after the stop")
    ap.add_argument("--status", type=float, default=10.0, help="status line period, s")
    ap.add_argument("--teleop", default="quest_soak", help="the soak's quest_teleop instance name")
    ap.add_argument("--fake-port", type=int, default=9399)
    ap.add_argument("--scenario", default="idle", help="fake headset scenario (idle | demo; neither presses Y)")
    ap.add_argument("--no-headset", action="store_true", help="no fake headset (the client sends no datagrams)")
    ap.add_argument("--enable-window", type=float, default=12.0, help="s after /start with the gripper unwatched")
    ap.add_argument("--q-drift", type=float, default=0.009, help="rad, arm joint off its snapshot (also q_cmd; 0.5 deg)")
    ap.add_argument("--qd-max", type=float, default=0.1, help="rad/s, arm joint speed")
    ap.add_argument("--gap-drift", type=float, default=0.004, help="m, gripper gap off its snapshot")
    ap.add_argument("--drive-vel", type=float, default=0.005, help="m/s, wheel speed / estimator speed")
    ap.add_argument("--drive-pos", type=float, default=0.002, help="m, wheel travel")
    ap.add_argument("--steer", type=float, default=0.0087, help="rad, steer angle / theta_odom off its snapshot")
    ap.add_argument("--steer-vel", type=float, default=0.1, help="rad/s, steer speed")
    ap.add_argument("--xy", type=float, default=0.003, help="m, estimator position off its snapshot")
    ap.add_argument("--cmd-eps", type=float, default=1e-3, help="any |commanded twist| above this trips")
    ap.add_argument("--noise-factor", type=float, default=3.0,
                    help="a measured threshold below this x its baseline peak is raised to it (reported)")
    ap.add_argument("--baseline-slack", type=float, default=5.0,
                    help="during the baseline (noise not yet known) measured limits are this x larger")
    ap.add_argument("--debounce", type=int, default=2, help="consecutive measured rows over a threshold")
    ap.add_argument("--stale", type=float, default=0.5, help="s of silence on a watched stream")
    ap.add_argument("--out", default=None, help="the verdict JSON (default ~/soak/<time>.json)")
    a = ap.parse_args()
    py = sys.executable
    procs = []
    verdict = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "args": vars(a)}
    out = Path(a.out or Path.home() / "soak" / f"{time.strftime('%Y%m%d-%H%M%S')}.json")
    out.parent.mkdir(parents=True, exist_ok=True)

    # 1. preflight
    try:
        w = call("wbc", "GET", "/state")
        if w.get("enabled") or (w.get("lease") or {}).get("held"):
            raise RuntimeError(f"the WBC is enabled or leased ({w.get('state')}, holder "
                               f"{(w.get('lease') or {}).get('holder')}): stop teleop first")
        for n in ("wbc", "arm", "swerve"):
            e = call(n, "GET", "/state").get("estop")
            if (e.get("latched") if isinstance(e, dict) else e):
                raise RuntimeError(f"{n} e-stop latched: clear it first")
        if socket_path(a.teleop).exists():
            try:
                call(a.teleop, "GET", "/state", timeout=1.0)
                raise RuntimeError(f"a {a.teleop} instance is already running")
            except (ConnectionError, OSError):
                pass
        call("estimator", "GET", "/state")
    except Exception as e:  # noqa: BLE001
        log(f"PREFLIGHT FAILED: {e}")
        return 3
    log(f"preflight ok: WBC {w.get('state')}; arm/swerve/WBC not e-stopped")

    wd = Watchdog(a)
    wd.start()
    rc, phase = 0, "baseline"
    pids = wbc_pids()
    hz = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
    started = False

    def aborting():
        return wd.trip_ev.is_set()

    def sleep_watch(s):
        return wd.trip_ev.wait(s)

    try:
        # 2. baseline
        log(f"baseline {a.baseline:g} s, nothing enabled (watchdog live)")
        if sleep_watch(a.baseline):
            raise SystemExit
        wd.taking_snap = False
        wd.slack = 1.0
        verdict["baseline_peak"] = dict(wd.peak)
        lims = [(r"arm\.q\d$", a.q_drift), (r"arm\.qd\d$", a.qd_max), (r"arm\.gap$", a.gap_drift),
                (r"swerve\.\w+\.drive_vel$", a.drive_vel), (r"swerve\.\w+\.drive_pos$", a.drive_pos),
                (r"swerve\.\w+\.steer$", a.steer), (r"swerve\.\w+\.steer_vel$", a.steer_vel),
                (r"est\.[xy]$", a.xy), (r"est\.theta_odom$", a.steer), (r"est\.speed$", a.drive_vel)]
        for k, pk in wd.peak.items():
            base = next((v for pat, v in lims if re.match(pat, k)), None)     # measured signals only
            if base is not None and a.noise_factor * pk > base:
                wd.raised[k] = a.noise_factor * pk
        wd.peak = {}                                    # from here: the soak's own peaks
        if wd.raised:
            log(f"thresholds raised over the baseline noise: { {k: round(v, 5) for k, v in wd.raised.items()} }")
        verdict["raised"] = dict(wd.raised)
        verdict["snapshot"] = dict(wd.snap)
        noisy = {k: round(v, 5) for k, v in sorted(wd.peak.items(), key=lambda kv: -kv[1])[:8]}
        log(f"baseline ok: rows {wd.rows}; largest deviations {noisy}")

        # 3. fake headset + the soak teleop instance
        phase = "start"
        if not a.no_headset:
            procs.append(subprocess.Popen([py, str(ROOT / "tools/quest_teleop/fake_headset.py"), "--host", "127.0.0.1",
                                           "--port", str(a.fake_port), "--scenario", a.scenario, "--loop"],
                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT))
        targs = [py, "-m", "robot_utils.nodes.quest_teleop", "--name", a.teleop, "--quest", "127.0.0.1",
                 "--quest-port", str(a.fake_port), "--ar", "off"]
        hlp = subprocess.run(targs[:3] + ["--help"], cwd=str(ROOT), capture_output=True, text=True,
                             env=dict(os.environ, PYTHONPATH=str(ROOT / "src"))).stdout
        if "--data-dir" in hlp:                          # the event log (data collection builds only)
            targs += ["--data-dir", str(out.with_suffix("")) + ".events"]
        procs.append(subprocess.Popen(targs,
                                      cwd=str(ROOT), env=dict(os.environ, PYTHONPATH=str(ROOT / "src")),
                                      stdout=open(out.with_suffix(".teleop.log"), "w"), stderr=subprocess.STDOUT))
        t_w = time.monotonic()
        while True:
            try:
                call(a.teleop, "GET", "/state", timeout=1.0)
                break
            except Exception:  # noqa: BLE001
                if time.monotonic() - t_w > 20 or procs[-1].poll() is not None:
                    raise RuntimeError(f"{a.teleop} did not come up (see {out.with_suffix('.teleop.log')})")
                if sleep_watch(0.3):
                    raise SystemExit
        log(f"{a.teleop} up (headset {'none' if a.no_headset else a.scenario + ' on :' + str(a.fake_port)}); "
            "POST /start: the WBC enables (arm + swerve hold)")
        wd.grip_ignore = True
        call(a.teleop, "POST", "/task", {"text": "wbc soak (no motion)"})
        call(a.teleop, "POST", "/start", timeout=30.0)
        started = True
        threading.Timer(a.enable_window, lambda: setattr(wd, "grip_ignore", False)).start()

        # 4. the soak
        phase = "soak"
        t0 = time.monotonic()
        w0 = call("wbc", "GET", "/state")
        tick0 = w0.get("tick") or {}
        pids = wbc_pids() or pids
        c0, tc0 = cpu_ticks(pids), time.monotonic()
        verdict["wbc_pids"] = pids
        log(f"enabled: WBC {w0.get('state')}, armed {w0.get('armed', False)}; soak {a.duration:g} s; "
            f"record with: python -m robot_utils.collect run --robot http://botcore-desktop.local --teleop {a.teleop} "
            "--out DIR --task soak")
        cpu = []
        while time.monotonic() - t0 < a.duration:
            if sleep_watch(a.status):
                raise SystemExit
            st = call("wbc", "GET", "/state")
            tk = st.get("tick") or {}
            c1, tc1 = cpu_ticks(pids), time.monotonic()
            pc = 100.0 * (c1 - c0) / hz / max(tc1 - tc0, 1e-3)
            cpu.append(pc)
            c0, tc0 = c1, tc1
            r = wd.wbc
            log(f"{time.monotonic() - t0:5.0f}s WBC {st.get('state')} hz {st.get('loop_hz', 0):.1f} "
                f"period max {tk.get('max_period_ms', float('nan')):.1f} ms, overruns {tk.get('overruns')}, late "
                f"{tk.get('late')}, input_age {r.get('input_age', float('nan')):.3f}, est_age "
                f"{r.get('est_age', float('nan')):.3f}, cpu {pc:.0f}% ({len(pids)} procs); "
                f"rows {wd.rows}")
            if st.get("state") in ("fault", "disabled") or wd.wbc_fault:
                verdict["fail"] = f"the WBC {st.get('state')}: {st.get('fault') or st.get('disabled_reason')}"
                log(f"FAIL: {verdict['fail']}")
                break
        w1 = call("wbc", "GET", "/state")
        tick1 = w1.get("tick") or {}
        verdict["wbc"] = {"tick_start": tick0, "tick_end": tick1, "latch": w1.get("latch"),
                          "disarm_reason": w1.get("disarm_reason"), "loop_hz": w1.get("loop_hz"),
                          "solve_failures": w1.get("solve_failures"), "max_period_stream_ms": wd.wbc_max_period,
                          "cpu_pct": {"mean": sum(cpu) / len(cpu) if cpu else None, "max": max(cpu, default=None)}}
        d_over = (tick1.get("overruns") or 0) - (tick0.get("overruns") or 0)
        d_late = (tick1.get("late") or 0) - (tick0.get("late") or 0)
        verdict["wbc"].update(overruns_during=d_over, late_during=d_late)
        if d_late and "fail" not in verdict:
            verdict["fail"] = f"{d_late} late tick(s) (each disarms)"

        # 5. stop
        phase = "stop"
        call(a.teleop, "POST", "/stop", timeout=10.0)
        started = False
        log(f"stopped; watching {a.post:g} s more")
        if sleep_watch(a.post):
            raise SystemExit
    except SystemExit:
        pass
    except Exception as e:  # noqa: BLE001
        verdict.setdefault("fail", f"{phase}: {e}")
        log(f"ERROR in {phase}: {e}")
    finally:
        if aborting():
            reason, detail = wd.tripped
            res = estop_all(f"{reason}: {detail}")
            verdict["abort"] = {"phase": phase, "reason": reason, "detail": detail, "estop": res}
            log(f"ABORT in {phase}: {reason}: {detail}; e-stop {res}")
            rc = 2
        elif started:
            try:
                call(a.teleop, "POST", "/stop", timeout=10.0)
            except Exception as e:  # noqa: BLE001
                log(f"stop failed: {e}; e-stopping")
                verdict["abort"] = {"phase": phase, "reason": "stop failed", "estop": estop_all("soak stop failed")}
                rc = 2
        wd.stop_ev.set()
        for p in reversed(procs):
            p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
        verdict["peak"] = {k: round(v, 6) for k, v in wd.peak.items()}
        verdict["rows"] = wd.rows
        if rc == 0 and "fail" in verdict:
            rc = 1
        verdict["result"] = {0: "pass", 1: "fail", 2: "abort"}[rc]
        out.write_text(json.dumps(verdict, indent=1, default=str))
        log(f"RESULT {verdict['result'].upper()}" + (f": {verdict.get('fail')}" if rc == 1 else "")
            + f"; verdict {out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
