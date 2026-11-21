#!/usr/bin/env python3
"""Damiao serial-bridge send timing bench (run on the Pi, robot venv, with the arm node STOPPED and the motors off).

  MOTORBRIDGE_TX_GAP_US=<us> ~/robot_utils/.venv/bin/python pi_dm_bench.py LABEL SECONDS [MODE]

READ-ONLY on the motors: it never enables a motor and never sends MIT. Each 10 ms tick it does what the arm node's
read() does: request XOUT (register 81) and a feedback refresh (0x7FF) from every motor, then drains replies.
MODE: both (default: XOUT + refresh, 14 frames/tick) | xout (7 frames/tick: the 2-pair tick's request half).
It times every send call (perf_counter) and the whole tick's work, on a SCHED_FIFO 40 thread like the control
loop, and counts replies. Prints one JSON line.
"""
import json, os, sys, threading, time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from motorbridge import Controller  # noqa: E402
from reBotArm_control_py.actuator import rebotarm as rb  # noqa: E402

LABEL, SEC = sys.argv[1], float(sys.argv[2])
MODE = sys.argv[3] if len(sys.argv) > 3 else "both"
PORT = "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00"
DT = 0.01
XOUT = rb.DM_RID_XOUT


def main():
    cfg = rb.load_cfg()
    joints = [j for j in cfg["joints"] if j.vendor == "damiao"]
    ctrl = Controller.from_dm_serial(PORT, 921600)
    motors = {j.name: ctrl.add_damiao_motor(j.motor_id, j.feedback_id, j.model) for j in joints}
    names = list(motors)
    send_t, tick_work, lateness = [], [], []
    xout_ok = state_ok = ticks = 0
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(40))
        rt = "FIFO 40"
    except OSError as e:
        rt = f"normal ({e})"
    t_end = time.monotonic() + SEC
    nxt = time.monotonic()
    while time.monotonic() < t_end:
        nxt += DT
        t0 = time.perf_counter()
        ctrl.poll_feedback_once()                                  # replies to last tick's requests
        for n in names:
            r = motors[n].get_cached_register_f32(XOUT)            # fresh-only: a reply to the last request
            xout_ok += r is not None
            state_ok += motors[n].get_state() is not None
        for n in names:                                            # this tick's requests, timed one by one
            a = time.perf_counter()
            motors[n].request_register(XOUT)
            send_t.append(time.perf_counter() - a)
        if MODE == "both":
            for n in names:
                a = time.perf_counter()
                motors[n].request_feedback()
                send_t.append(time.perf_counter() - a)
        tick_work.append(time.perf_counter() - t0)
        ticks += 1
        now = time.monotonic()
        if nxt > now:
            time.sleep(nxt - now)
        else:
            lateness.append(now - nxt)
            nxt = now
    ctrl.close() if hasattr(ctrl, "close") else None
    s = np.array(send_t) * 1e3
    w = np.array(tick_work) * 1e3
    p = lambda a, q: round(float(np.percentile(a, q)), 3)   # noqa: E731
    print(json.dumps({
        "label": LABEL, "mode": MODE, "tx_gap_us_env": os.environ.get("MOTORBRIDGE_TX_GAP_US", "auto(120)"),
        "thread": rt, "ticks": ticks, "frames_per_tick": len(names) * (2 if MODE == "both" else 1),
        "send_ms": {"p10": p(s, 10), "p50": p(s, 50), "p90": p(s, 90), "p99": p(s, 99), "max": round(float(s.max()), 3)},
        "tick_work_ms": {"p50": p(w, 50), "p90": p(w, 90), "p99": p(w, 99), "max": round(float(w.max()), 3)},
        "late_ticks": len(lateness), "xout_reply_rate": round(xout_ok / max(1, (ticks - 1) * len(names)), 3),
        "state_seen_rate": round(state_ok / max(1, ticks * len(names)), 3)}), flush=True)


if __name__ == "__main__":
    main()
