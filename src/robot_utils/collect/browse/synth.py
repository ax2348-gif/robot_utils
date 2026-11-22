"""A synthetic session in the recorder's exact layout (docs/collect.md "Session layout"), for the browser's tests and
performance numbers without a robot:

    python -m robot_utils.collect.browse.synth OUT [--seconds 600] [--cam-fps 30] [--cam-size 640x480]

Every stream uses its node's real schema (arm, swerve, estimator, WBC ``stream_fields``), framed as the nodes send
it; cameras are JPEG ``frame`` records (a textured test image with the frame number, so a JPEG is camera-sized);
the operator stream has native-shaped ``headset`` frames (``tools/quest_teleop/fake_headset.Headset`` playing its
``episodes`` scenario), ``sent`` datagrams that follow the body-frozen rules (a hint at the engagement's base pose
and the EE command at capture on the first engaged datagram), and the episode events. Motion is smooth and
plausible, not physical. Pi time starts at ``T0``.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from ...nodes.common.protocol import pack_record, rows_record, schema_record

T0 = 100_000.0


class _Writer:
    """``<name>.NNN.bin`` with rotation at record boundaries (a new file repeats the schema records)."""

    def __init__(self, d: Path, name: str, rotate: int):
        self.d, self.name, self.rotate = d, name, rotate
        self.k, self.size, self.f = 0, 0, None
        self.schemas: list[bytes] = []
        self._open()

    def _open(self):
        if self.f:
            self.f.close()
        self.f = open(self.d / f"{self.name}.{self.k:03d}.bin", "wb")
        self.size = 0
        for s in self.schemas:
            self._w(s)

    def _w(self, b: bytes):
        self.f.write(b)
        self.size += len(b)

    def schema(self, b: bytes):
        self.schemas.append(b)
        self._w(b)

    def write(self, b: bytes):
        if self.rotate and self.size > self.rotate:
            self.k += 1
            self._open()
        self._w(b)

    def close(self):
        self.f.close()


def _jpeg(w: int, h: int, k: int, cam: int, rng) -> bytes:
    import cv2
    y, x = np.mgrid[0:h, 0:w]
    img = np.zeros((h, w, 3), np.uint8)
    img[..., 0] = (128 + 90 * np.sin(x / 37.0 + k * 0.05 + cam)).astype(np.uint8)
    img[..., 1] = (128 + 90 * np.sin(y / 23.0 - k * 0.03)).astype(np.uint8)
    img[..., 2] = ((x + y + 3 * k) % 256).astype(np.uint8)
    img = cv2.add(img, rng.integers(0, 40, img.shape, dtype=np.uint8))
    cv2.putText(img, f"{['head', 'wrist'][cam]} {k}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 3)
    return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


def make_session(out: Path, seconds: float = 60.0, cam_fps: float = 30.0, cam_size=(640, 480), rows_hz: float = 100.0,
                 rotate: int = 0, seed: int = 0, jpeg_pool: int = 0) -> dict:
    """Write a session to ``out``; returns ``{"episodes": [...], "anchors": [the true clutch anchors]}``."""
    from ...nodes.arm.telemetry import FIELDS as ARM_F, UNITS as ARM_U
    from ...nodes.estimator.fusion import FIELDS as EST_F, UNITS as EST_U
    from ...nodes.swerve.node import FIELDS as SW_F, UNITS as SW_U
    from ...nodes.wbc.node import stream_fields
    tools = Path(__file__).resolve().parents[4] / "tools"
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    from quest_teleop import fake_headset as FH  # noqa: E402

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    WB_F, WB_U, _ = stream_fields(["arm"])
    names = {"camera.color": "camera/color", "wrist_camera.color": "wrist_camera/color", "arm": "arm/stream",
             "swerve": "swerve/stream", "estimator": "estimator/stream", "wbc": "wbc/stream",
             "quest_teleop.oplog": "quest_teleop/oplog/stream"}
    W = {n: _Writer(out, n, rotate) for n in names}
    for n, (f, u) in {"arm": (ARM_F, ARM_U), "swerve": (SW_F, SW_U), "estimator": (EST_F, EST_U),
                      "wbc": (WB_F, WB_U)}.items():
        W[n].schema(schema_record(1, f, u))
    col = {n: {k: i for i, k in enumerate(f)} for n, f in (("arm", ARM_F), ("swerve", SW_F), ("estimator", EST_F),
                                                            ("wbc", WB_F))}

    # -- the operator: the fake headset's episodes scenario, looped -------------------------------
    keys = FH.episodes_scenario()
    period = keys[-1][0]
    hs = FH.Headset()
    hs_hz = 72.0

    def base(t):                       # a slow loop around the room
        u = (t - T0) * 0.05
        return 1.2 * math.sin(u), 0.8 * math.sin(2 * u), 0.6 * math.sin(u * 0.7)

    def q_arm(t):
        u = t - T0
        return np.array([0.6 * math.sin(0.3 * u), -0.8 + 0.3 * math.sin(0.5 * u), 1.2 + 0.4 * math.sin(0.4 * u),
                         0.3 * math.sin(0.7 * u), -0.5 + 0.2 * math.sin(0.6 * u), 0.8 * math.sin(0.2 * u)])

    # EE poses in world from the arm model when available (else a plausible point ahead of the base)
    try:
        from .model import arm_chain, chain_fk, load_description
        chain, _ = arm_chain()
        desc, _ = load_description(None)
        T_arm = np.asarray(desc["frames_in_base"]["arm"])
    except Exception:  # noqa: BLE001
        chain = None

    def ee_world(t, q):
        bx, by, byaw = base(t)
        if chain is not None:
            T = T_arm @ chain_fk(chain, q)[-1]
        else:
            T = np.eye(4)
            T[:3, 3] = [0.4, 0.0, 0.5]
        Tb = np.eye(4)
        Tb[:3, :3] = [[math.cos(byaw), -math.sin(byaw), 0], [math.sin(byaw), math.cos(byaw), 0], [0, 0, 1]]
        Tb[:3, 3] = [bx, by, 0]
        Tw = Tb @ T
        from .oplog import quat_from_matrix
        return Tw[:3, 3], quat_from_matrix(Tw[:3, :3])

    # -- rows streams ------------------------------------------------------------------------------
    n_rows = int(seconds * rows_hz)
    tt = T0 + np.arange(n_rows) / rows_hz
    for i, t in enumerate(tt):
        q = q_arm(t)
        bx, by, byaw = base(t)
        a = np.full(len(ARM_F), np.nan)
        a[col["arm"]["t_tick"]] = t
        a[col["arm"]["tick"]] = i
        for j in range(6):
            a[col["arm"][f"j{j + 1}_q"]] = q[j]
            a[col["arm"][f"j{j + 1}_q_cmd"]] = q[j] + 0.01
        a[col["arm"]["g_gap"]] = 0.03 + 0.025 * math.sin(0.4 * (t - T0))
        a[col["arm"]["g_gap_cmd"]] = a[col["arm"]["g_gap"]] + 0.002
        W["arm"].write(rows_record(a[None, :], 1))
        e = np.full(len(EST_F), np.nan)
        e[[col["estimator"][k] for k in ("t", "x", "y", "theta")]] = [t - 0.002, bx, by, byaw]
        W["estimator"].write(rows_record(e[None, :], 1))
        s = np.full(len(SW_F), np.nan)
        s[col["swerve"]["t_tick"]] = t
        for m in ("FL", "FR", "RL", "RR"):
            if f"{m}_steer_angle" in col["swerve"]:
                s[col["swerve"][f"{m}_steer_angle"]] = 0.5 * math.sin(0.1 * (t - T0))
                s[col["swerve"][f"{m}_drive_vel"]] = 0.2
        W["swerve"].write(rows_record(s[None, :], 1))
        w = np.full(len(WB_F), np.nan)
        c = col["wbc"]
        w[[c["t_tick"], c["base_x"], c["base_y"], c["base_yaw"], c["state"], c["armed"]]] = [t, bx, by, byaw, 3, 1]
        pm, qm = ee_world(t, q)
        pc, qc = ee_world(t, q + 0.01)
        pt, qt = ee_world(t, q + 0.02)
        for k, (p, qq) in {"meas": (pm, qm), "cmd": (pc, qc), "tgt": (pt, qt)}.items():
            w[[c[f"arm_{k}_{x}"] for x in ("x", "y", "z")]] = p
            w[[c[f"arm_{k}_{x}"] for x in ("qx", "qy", "qz", "qw")]] = qq
        for j in range(6):
            w[c[f"arm_q_cmd{j + 1}"]] = q[j] + 0.01
        w[c["arm_err_pos"]] = float(np.linalg.norm(pt - pc))
        w[c["arm_err_meas_pos"]] = float(np.linalg.norm(pt - pm))
        w[c["base_vx"]] = 0.05
        W["wbc"].write(rows_record(w[None, :], 1))

    # -- cameras -----------------------------------------------------------------------------------
    cw, ch = cam_size
    for cam, name in enumerate(("camera.color", "wrist_camera.color")):
        pool = [_jpeg(cw, ch, k, cam, rng) for k in range(jpeg_pool)] if jpeg_pool else None
        nf = int(seconds * cam_fps)
        for k in range(nf):
            t = T0 + k / cam_fps + 0.003 * cam
            jp = pool[k % jpeg_pool] if pool else _jpeg(cw, ch, k, cam, rng)
            W[name].write(pack_record({"type": "frame", "stream": "color", "seq": k, "t_sensor": t, "t_capture": t + 0.004,
                                       "width": cw, "height": ch, "source_format": "MJPG", "dropped": 0}, jp))

    # -- the operator stream ---------------------------------------------------------------------------
    events, eps, anchors = [], [], []
    seq = [0]

    def emit(t, kind, episode=None, **detail):
        seq[0] += 1
        ev = {"seq": seq[0], "t": t, "wall": 1.79e9 + t - T0, "kind": kind, "episode": episode, "detail": detail}
        events.append(ev)
        W["quest_teleop.oplog"].write(pack_record({"type": "event", **ev}))
        return ev

    n_hs = int(seconds * hs_hz)
    prev_btn = {}
    open_ep = None
    eng_prev = False
    eng_id_prev = None
    hint_frame = None
    hold = None
    ep_count = 0
    for k in range(n_hs):
        t = T0 + k / hs_hz
        st = FH.sample(keys, (t - T0) % period)
        fr = hs.frame(st, t)
        W["quest_teleop.oplog"].write(pack_record({"type": "headset", "t_rx": t, "frame": fr}))
        btn = {"A": st.right.primary, "B": st.right.secondary, "X": st.left.primary, "Y": st.left.secondary}
        for b, v in btn.items():
            if v != prev_btn.get(b, False):
                emit(t, "button", open_ep, button=b, pressed=v)
                if v and b == "A":
                    if open_ep:
                        emit(t, "episode_stop", open_ep)
                        eps[-1].update(t_stop=t, outcome="kept", reason=None)
                    ep_count += 1
                    open_ep = f"20261122-{ep_count:04d}"
                    emit(t, "episode_start", open_ep, task="synthetic")
                    eps.append({"id": open_ep, "task": "synthetic", "t_start": t, "t_stop": None,
                                "wall": {"start": 1.79e9 + t - T0, "stop": None}, "outcome": None, "reason": None})
                if v and b == "B" and open_ep:
                    emit(t, "episode_stop", open_ep)
                    eps[-1].update(t_stop=t, outcome="kept")
                    open_ep = None
                if v and b == "X" and open_ep:
                    emit(t, "episode_abort", open_ep, reason="disabled (X)")
                    eps[-1].update(t_stop=t, outcome="aborted", reason="disabled (X)")
                    open_ep = None
        prev_btn = btn
        ih = fr["control_intent"]["hands"]["right"]
        eng, eid = bool(ih["engaged"]), ih["engagement_id"]
        i_row = min(n_rows - 1, int((t - T0) * rows_hz))
        bx, by, byaw = base(tt[i_row])
        pc, qc = ee_world(tt[i_row], q_arm(tt[i_row]) + 0.01)
        if eng and (not eng_prev or eid != eng_id_prev):
            hint_frame = (bx, by, byaw, pc, qc)
            head = fr["head"]["pose"]
            hand = fr["hands"]["right"]["pose"]
            anchors.append({"t": t, "robot_frame": [bx, by, byaw], "ee_anchor": list(pc) + list(qc),
                            "head16": head, "hand16": hand, "engagement_id": eid})
        if eng:
            hx, hy, hyaw, p0, q0 = hint_frame
            first = not eng_prev or eid != eng_id_prev
            p = p0 if first else p0 + 0.001 * (k % 7)
            dg = {"type": "target", "session": 1, "seq": k, "t": None, "armed": True, "arm_seq": 1,
                  "ees": {"arm": {"frame": "world", "pos": list(map(float, p)), "quat": list(map(float, q0))}},
                  "base": {"mode": "hint", "target": {"pose": {"x": hx, "y": hy, "frame": "world"},
                                                      "heading": {"yaw": hyaw, "frame": "world"}}}}
        else:
            hold = (pc, qc)
            dg = {"type": "target", "session": 1, "seq": k, "t": None, "armed": True, "arm_seq": 1,
                  "ees": {"arm": {"frame": "world", "pos": list(map(float, hold[0])), "quat": list(map(float, hold[1]))}},
                  "base": {"mode": "locked"}}
        eng_prev, eng_id_prev = eng, eid
        W["quest_teleop.oplog"].write(pack_record({"type": "sent", "t_send": t + 0.0005, "datagram": dg}))
    if open_ep:
        eps[-1].update(t_stop=T0 + seconds, outcome="aborted", reason="session ended")
    for w in W.values():
        w.close()
    eps = [e for e in eps if e["t_stop"] is not None]
    with open(out / "episodes.jsonl", "w") as f:
        for e in eps:
            f.write(json.dumps(e) + "\n")
    with open(out / "events.jsonl", "w") as f:
        for e in events:
            f.write(json.dumps({**e, "via": "stream"}) + "\n")
    meta = {"robot": "synthetic", "task": "synthetic", "started_wall": 1.79e9, "stopped_wall": 1.79e9 + seconds,
            "stop_reason": "synthetic", "streams": [{"spec": s, "name": n, "url": None, "params": {}}
                                                    for n, s in names.items()]}
    (out / "session.json").write_text(json.dumps(meta, indent=1))
    return {"episodes": eps, "anchors": anchors}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("out")
    p.add_argument("--seconds", type=float, default=120.0)
    p.add_argument("--cam-fps", type=float, default=30.0)
    p.add_argument("--cam-size", default="640x480")
    p.add_argument("--rotate", default="0", help="rotate files at this many MB")
    p.add_argument("--pool", type=int, default=0, help="reuse this many distinct JPEGs (faster to generate)")
    a = p.parse_args(argv)
    w, h = (int(v) for v in a.cam_size.split("x"))
    r = make_session(Path(a.out), a.seconds, a.cam_fps, (w, h), rotate=int(float(a.rotate) * 1e6), jpeg_pool=a.pool)
    print(f"{a.out}: {len(r['episodes'])} episodes, {len(r['anchors'])} engagements")
    return 0


if __name__ == "__main__":
    sys.exit(main())
