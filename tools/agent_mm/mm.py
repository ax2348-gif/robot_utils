#!/usr/bin/env python3
"""Tiny mobile-manipulation helper: logs every robot call and saves every image.

  mm.py nav  METHOD PATH [JSON]     call agent_nav; downloads images it references into img/
  mm.py arm  METHOD PATH [JSON]     call the arm node (adds the lease token if held)
  mm.py keeper                      hold the arm lease (renew every 1 s); run in background
  mm.py tcp                         measured TCP pose + gripper + last motion
  mm.py servo X Y Z [R P Y] [--v V] planned arm move (arm base frame), waits, reports error
  mm.py step DX DY DZ [--v V]       servo relative to the last *commanded* goal
  mm.py grip GAP_MM [FORCE_N]

Every call is appended to $MM_RUN/calls.jsonl (default: the current folder). Run record.py
alongside for high-rate streams and camera frames.
"""
import json, os, sys, time, urllib.request, urllib.error

HOST = os.environ.get("ROBOT_NODES", "http://botcore-desktop.local/nodes")   # robot LAN (mDNS), not Tailscale
RUN = os.environ.get("MM_RUN") or os.getcwd()  # the episode folder (logs, img/, lease token)
IMG = os.path.join(RUN, "img")
LOG = os.path.join(RUN, "calls.jsonl")
TOKEN = os.path.join(RUN, ".lease_token")
GOAL = os.path.join(RUN, ".last_goal.json")


def call(node, method, path, body=None, log=True):
    url = f"{HOST}/{'agent_nav' if node == 'nav' else node}/{path.lstrip('/')}"
    data = json.dumps(body).encode() if body is not None else (b"{}" if method.upper() == "POST" else None)
    req = urllib.request.Request(url, data=data, method=method.upper())
    req.add_header("content-type", "application/json")
    if node == "arm" and os.path.exists(TOKEN):
        req.add_header("X-Lease-Token", open(TOKEN).read().strip())
    t = time.time()
    try:
        with urllib.request.urlopen(req, timeout=90) as r:
            status, raw = r.status, r.read()
    except urllib.error.HTTPError as e:
        status, raw = e.code, e.read()
    try:
        resp = json.loads(raw)
    except Exception:
        resp = raw.decode(errors="replace")
    if log:
        with open(LOG, "a") as f:
            f.write(json.dumps({"t": t, "node": node, "method": method.upper(), "path": path,
                                "body": body, "status": status, "resp": resp}) + "\n")
    return status, resp


def fetch_images(resp):
    """Download every images/... path found in a nav reply."""
    out = []
    def walk(o):
        if isinstance(o, dict):
            for v in o.values(): walk(v)
        elif isinstance(o, list):
            for v in o: walk(v)
        elif isinstance(o, str) and o.startswith("images/"):
            out.append(o)
    walk(resp)
    os.makedirs(IMG, exist_ok=True)
    saved = []
    for p in dict.fromkeys(out):
        dst = os.path.join(IMG, p[len("images/"):].replace("/", "_"))
        try:
            with urllib.request.urlopen(f"{HOST}/agent_nav/{p}", timeout=30) as r:
                open(dst, "wb").write(r.read())
            saved.append(dst)
        except Exception as e:
            saved.append(f"{dst} (failed: {e})")
    return saved


def tcp():
    _, k = call("arm", "GET", "kinematics", log=False)
    _, s = call("arm", "GET", "state", log=False)
    g = s["gripper"]
    m = s.get("motion") or {}
    return {"pos": k["tcp"]["pos"], "rpy": k["tcp"]["rpy"], "q": [round(x, 3) for x in k["q"]],
            "w": round(k["w"], 4), "state": s["state"],
            "gripper": {"gap_mm": g["gap_mm"], "force_n": g["force_n"], "limiting": g["limiting"]},
            "motion": {k2: m.get(k2) for k2 in ("id", "phase", "reason", "goal")}}


def servo(pos, rpy, v=0.05, comment=None):
    body = {"pose": {"pos": pos, "rpy": rpy}, "lin_vel": v, "ang_vel": 0.4}
    st, r = call("arm", "POST", "servo", body)
    if st != 200:
        return {"status": st, "resp": r}
    json.dump({"pos": pos, "rpy": rpy}, open(GOAL, "w"))
    for _ in range(300):
        time.sleep(0.2)
        _, s = call("arm", "GET", "state", log=False)
        ph = (s.get("motion") or {}).get("phase")
        if ph not in ("running", "planned", "moving", "settling", None) and s["motion"].get("progress", 0) >= 1.0:
            break
    time.sleep(0.3)
    now = tcp()
    err = [round(a - b, 4) for a, b in zip(now["pos"], pos)]
    out = {"goal": {"pos": pos, "rpy": rpy}, "measured": now["pos"], "rpy": now["rpy"], "err_m": err,
           "phase": now["motion"]["phase"], "reason": now["motion"]["reason"], "gripper": now["gripper"]}
    with open(LOG, "a") as f:
        f.write(json.dumps({"t": time.time(), "node": "arm", "method": "RESULT", "path": "servo", "resp": out}) + "\n")
    return out


def keeper():
    st, r = call("arm", "POST", "lease", {"client": "claude-mm"})
    if st != 200:
        print("lease refused", st, r); sys.exit(1)
    tok = r.get("token") or r.get("lease_token") or r.get("lease_id")
    open(TOKEN, "w").write(str(tok))
    print("lease held", r, flush=True)
    try:
        while True:
            st, r = call("arm", "POST", "lease/renew", log=False)
            if st != 200:
                print("renew failed", st, r, flush=True)
                break
            time.sleep(1.0)
    finally:
        if os.path.exists(TOKEN):
            os.remove(TOKEN)


def main(a):
    flags = {}
    if "--v" in a:
        i = a.index("--v"); flags["v"] = float(a[i + 1]); a = a[:i] + a[i + 2:]
    cmd = a[0]
    if cmd in ("nav", "arm"):
        body = json.loads(a[3]) if len(a) > 3 else None
        st, r = call(cmd, a[1], a[2], body)
        res = {"status": st, "resp": r}
        if cmd == "nav":
            res["saved"] = fetch_images(r)
        print(json.dumps(res, indent=1)[:6000])
    elif cmd == "keeper":
        keeper()
    elif cmd == "tcp":
        print(json.dumps(tcp()))
    elif cmd == "servo":
        x = [float(v) for v in a[1:]]
        rpy = x[3:6] if len(x) >= 6 else json.load(open(GOAL))["rpy"] if os.path.exists(GOAL) else [0, 0, 0]
        print(json.dumps(servo(x[:3], rpy, **flags)))
    elif cmd == "step":
        g = json.load(open(GOAL)) if os.path.exists(GOAL) else {"pos": tcp()["pos"], "rpy": [0, 0, 0]}
        d = [float(v) for v in a[1:4]]
        print(json.dumps(servo([round(p + q, 4) for p, q in zip(g["pos"], d)], g["rpy"], **flags)))
    elif cmd == "grip":
        body = {"gap_mm": float(a[1])}
        if len(a) > 2: body["max_force_n"] = float(a[2])
        st, r = call("arm", "POST", "gripper", body)
        time.sleep(1.5)
        print(st, json.dumps(tcp()["gripper"]))


if __name__ == "__main__":
    main(sys.argv[1:])
