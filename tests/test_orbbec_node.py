"""Tests for the Orbbec node that need no hardware.

    PYTHONPATH=src python -m pytest tests/test_orbbec_node.py -q

Pure logic (framing, time sync, settings validation) plus one end-to-end run of the real server
and client against the synthetic camera (skipped if fastapi/uvicorn are missing).
"""
import io
import json
import random
import socket
import threading
import time
import zlib

import numpy as np
import pytest

from robot_utils.nodes.orbbec.protocol import (IMU_FIELDS, RecordReader, decode_payload, imu_payload,
                                               iter_records, pack_record)
from robot_utils.nodes.orbbec.timesync import (ClockSync, SyncEstimate, SyncSample, collect_samples,
                                               epoch_minus_mono, estimate_offset, fit_drift)

pydantic = pytest.importorskip("pydantic")
from robot_utils.nodes.orbbec.models import (Catalog, ImuCatalog, ProfileOption, Settings,  # noqa: E402
                                             SettingsError, SettingsPatch, apply_patch, fix_settings)


# -- framing -----------------------------------------------------------------------------------

def test_pack_and_parse_roundtrip_any_chunking():
    depth = (np.arange(12, dtype=np.uint16).reshape(3, 4) * 1000)
    recs = [({"type": "frame", "stream": "depth", "encoding": "raw", "format": "Y16", "dtype": "<u2",
              "width": 4, "height": 3, "seq": 1}, depth.tobytes()),
            ({"type": "ping"}, b""),
            ({"type": "frame", "stream": "color", "encoding": "jpeg", "format": "MJPG", "width": 2, "height": 2},
             b"\xff\xd8fakejpeg\xff\xd9")]
    blob = b"".join(pack_record(h, p) for h, p in recs)
    # header length is a big-endian u32 and header.len is the payload size
    n = int.from_bytes(blob[:4], "big")
    assert json.loads(blob[4:4 + n])["len"] == depth.nbytes
    for chunk in (1, 3, 7, 64, len(blob)):
        r, got = RecordReader(), []
        for i in range(0, len(blob), chunk):
            got += r.feed(blob[i:i + chunk])
        assert [h["type"] for h, _ in got] == ["frame", "ping", "frame"]
        assert got[0][1] == depth.tobytes() and got[2][1] == recs[2][1] and r.pending == 0
    out = decode_payload(*got[0])
    assert out.dtype == np.uint16 and out.shape == (3, 4) and np.array_equal(out, depth)
    assert decode_payload(*got[1]) is None
    assert decode_payload(*got[2], decode_jpeg_images=False) == recs[2][1]


def test_zlib_and_ir_and_bad_size():
    ir = np.arange(6, dtype=np.uint8)
    h = {"type": "frame", "encoding": "raw", "format": "Y8", "width": 3, "height": 2, "compression": "zlib"}
    assert np.array_equal(decode_payload(h, zlib.compress(ir.tobytes())), ir.reshape(2, 3))
    with pytest.raises(ValueError):
        decode_payload({**h, "compression": None}, b"\x00" * 5)


def test_imu_record_and_file_iteration():
    rows = np.random.default_rng(0).normal(size=(5, len(IMU_FIELDS)))
    h = {"type": "imu", "n": 5, "fields": list(IMU_FIELDS)}
    f = io.BytesIO(pack_record(h, imu_payload(rows)) * 3)
    recs = list(iter_records(f, chunk_size=10))
    assert len(recs) == 3
    s = decode_payload(*recs[0])
    assert np.allclose(s.az, rows[:, IMU_FIELDS.index("az")]) and s.seq.shape == (5,)
    with pytest.raises(ValueError):
        list(iter_records(io.BytesIO(pack_record(h, imu_payload(rows))[:-3])))


def test_memoryview_payload_is_not_copied_wrong():
    a = np.arange(8, dtype=np.uint16).reshape(2, 4)
    rec = pack_record({"type": "frame"}, memoryview(a).cast("B"))
    (h, p), = RecordReader().feed(rec)
    assert p == a.tobytes() and h["len"] == 16


# -- time sync ---------------------------------------------------------------------------------

def _samples(offset, n=40, base_delay=0.002, jitter=0.01, seed=1):
    rnd = random.Random(seed)
    out, t = [], 100.0
    for _ in range(n):
        up, down, srv = base_delay + rnd.expovariate(1 / jitter), base_delay + rnd.expovariate(1 / jitter), 1e-4
        t0 = t
        ts = t0 + up + offset
        t1 = t0 + up + srv + down
        out.append(SyncSample(t0, ts, t1))
        t = t1 + 0.01
    return out


def test_sample_math():
    s = SyncSample(10.0, 1010.003, 10.004)
    assert s.rtt == pytest.approx(0.004)
    assert s.offset == pytest.approx(1000.001)
    lo, hi = s.bounds
    assert lo == pytest.approx(1000.0 - 0.001) and hi == pytest.approx(1000.003)


def test_estimate_contains_truth_and_beats_single_sample():
    true = 1234.5678
    samples = _samples(true)
    est = estimate_offset(samples, keep=8)
    assert abs(est.offset - true) <= est.uncertainty + 1e-12
    assert est.uncertainty <= min(s.rtt for s in samples) / 2 + 1e-12
    assert est.method == "intersection" and est.n == len(samples)


def test_estimate_falls_back_when_intervals_disagree():
    # two samples whose bound intervals do not overlap (clock stepped mid-burst)
    a, b = SyncSample(0.0, 5.0, 0.002), SyncSample(1.0, 7.0, 1.004)
    est = estimate_offset([a, b], keep=2)
    assert est.method == "min_rtt" and est.offset == pytest.approx(a.offset)
    with pytest.raises(ValueError):
        estimate_offset([])


def test_drift_fit_and_inverse():
    hist = [SyncEstimate(offset=5.0 + 2e-5 * t, uncertainty=1e-4, rtt_min=2e-4, rtt_median=3e-4, n=16,
                         t_client=t, method="intersection") for t in (0.0, 30.0, 60.0, 90.0)]
    assert fit_drift(hist) == pytest.approx(2e-5, rel=1e-6)
    assert fit_drift(hist[:1]) == 0.0
    assert fit_drift(hist[:2], min_span=100) == 0.0
    cs = ClockSync()
    for e in hist:
        cs.add(e)
    t_s = cs.to_server(100.0)
    assert t_s == pytest.approx(100.0 + 5.0 + 2e-5 * 100.0)
    assert cs.to_client(t_s) == pytest.approx(100.0)
    with pytest.raises(RuntimeError):
        ClockSync().to_server(1.0)


def test_collect_samples_with_fake_server_clock():
    clock = iter(np.arange(0, 100, 0.001)).__next__
    samples = collect_samples(lambda: 42.0 + clock(), n=5, clock=clock)
    est = estimate_offset(samples)
    assert abs(est.offset - 42.0) <= est.uncertainty + 1e-9


def test_epoch_minus_mono_is_consistent():
    assert abs(epoch_minus_mono() - (time.time() - time.monotonic())) < 0.01


# -- settings ----------------------------------------------------------------------------------

def _cat(**kw):
    return Catalog(
        available=["color", "depth", "ir_left", "ir_right"],
        color=[ProfileOption(key="1280x720@10 MJPG", width=1280, height=720, fps=10, format="MJPG", default=True),
               ProfileOption(key="640x480@30 MJPG", width=640, height=480, fps=30, format="MJPG")],
        stereo=[ProfileOption(key="848x480@10", width=848, height=480, fps=10, default=True),
                ProfileOption(key="640x400@30", width=640, height=400, fps=30)],
        imu=ImuCatalog(available=True, sample_rates=["100Hz", "200Hz", "1kHz"], default_rate="100Hz",
                       accel_ranges=["4g"], gyro_ranges=["1000dps"]),
        colormaps={"turbo": [], "jet": []}, **kw)


def test_fix_settings_fills_defaults_and_drops_missing_streams():
    cat = _cat()
    cat.available = ["color", "depth"]
    s = fix_settings(Settings(streams={"color": True, "depth": True, "ir_left": True, "ir_right": False},
                              color_profile="nope"), cat)
    assert s.color_profile == "1280x720@10 MJPG" and s.stereo_mode == "848x480@10"
    assert s.streams["ir_left"] is False and s.imu.sample_rate == "100Hz" and s.imu.gyro_range == "1000dps"


def test_apply_patch_restart_flags():
    cat = _cat()
    cur = fix_settings(Settings(), cat)
    new, restart, rimu = apply_patch(cur, SettingsPatch(display={"colormap": "jet"}), cat)
    assert new.display.colormap == "jet" and not restart and not rimu
    assert new.color_profile == cur.color_profile
    new, restart, rimu = apply_patch(cur, SettingsPatch(stereo_mode="640x400@30", streams={"ir_left": True}), cat)
    assert restart and not rimu and new.streams["ir_left"] and new.streams["color"]
    new, restart, rimu = apply_patch(cur, SettingsPatch(imu={"sample_rate": "1kHz"}), cat)
    assert rimu and not restart
    new, restart, rimu = apply_patch(cur, SettingsPatch(imu={"filter": {"kp": 0.2}}), cat)
    assert not rimu and not restart and new.imu.filter.kp == 0.2 and new.imu.filter.ki == cur.imu.filter.ki
    _, restart, _ = apply_patch(cur, SettingsPatch(color_profile=cur.color_profile), cat)
    assert not restart


@pytest.mark.parametrize("patch, msg", [
    ({"color_profile": "1920x1080@30 MJPG"}, "unknown color profile"),
    ({"stereo_mode": "1x1@1"}, "unknown depth/IR mode"),
    ({"display": {"colormap": "nope"}}, "unknown colormap"),
    ({"display": {"pct_lo": 90, "pct_hi": 10}}, "pct_lo < pct_hi"),
    ({"display": {"min_mm": 3000, "max_mm": 100}}, "min_mm < max_mm"),
    ({"display": {"jpeg_quality": 5}}, "jpeg_quality"),
    ({"display": {"invalid_color": "red"}}, "invalid_color"),
    ({"imu": {"sample_rate": "7Hz"}}, "unknown IMU sample_rate"),
    ({"imu": {"filter": {"kp": -1}}}, "kp"),
])
def test_apply_patch_rejects(patch, msg):
    cat = _cat()
    with pytest.raises(SettingsError, match=msg):
        apply_patch(fix_settings(Settings(), cat), SettingsPatch.model_validate(patch), cat)


def test_patch_model_rejects_unknown_keys_and_streams():
    with pytest.raises(pydantic.ValidationError):
        SettingsPatch.model_validate({"bogus": 1})
    with pytest.raises(pydantic.ValidationError):
        SettingsPatch.model_validate({"streams": {"thermal": True}})
    empty = Catalog()
    with pytest.raises(SettingsError, match="not connected"):
        apply_patch(Settings(), SettingsPatch(streams={"color": True}), empty)
    new, restart, _ = apply_patch(Settings(), SettingsPatch(streams={"color": False}), empty)
    assert restart and not new.streams["color"]


# -- end to end with the synthetic camera --------------------------------------------------------

def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def fake_node(tmp_path_factory):
    pytest.importorskip("fastapi")
    uvicorn = pytest.importorskip("uvicorn")
    pytest.importorskip("httpx")
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.orbbec.server import create_app
    hub = Hub()
    cal = str(tmp_path_factory.mktemp("cal") / "orbbec_imu.json")
    app = create_app(Owner(FakeBackend(), hub, Settings(), startup_s=1.5, calibration_file=cal), hub)
    port = _free_port()
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning",
                                           timeout_graceful_shutdown=1))
    th = threading.Thread(target=server.run, daemon=True)
    th.start()
    deadline = time.time() + 10
    while not server.started and time.time() < deadline:
        time.sleep(0.05)
    yield f"http://127.0.0.1:{port}"
    server.should_exit = True
    th.join(timeout=5)


def test_end_to_end_fake(fake_node, tmp_path):
    from robot_utils.nodes.orbbec.client import OrbbecClient, OrbbecError, read_recording, record
    with OrbbecClient(fake_node) as c:
        est = c.sync()
        cal = c.get("/calibration")
        assert {"depth", "color", "depth_to_color"} <= set(cal) and cal["depth"]["fx"] > 0 and cal["aligned_depth"] is False
        page = c.http.get("/").text
        assert "Point cloud" in page and "readStream" in page and page.count("<script>") == 2
        m = c.get("/meta")
        assert m["name"] == "camera" and m["page"] == "./" and m["actuated"] is False
        assert m["endpoints"] == {"health": "health", "state": "state", "events": "events", "estop": None}
        assert abs(est.offset) < 0.01 and est.uncertainty < 0.01   # same machine, same clock
        got = {}
        for fr in c.frames("color,depth", mode="raw"):
            got.setdefault(fr.stream, fr)
            if len(got) == 2:
                break
        d = got["depth"]
        assert d.image.dtype == np.uint16 and d.header["depth_scale"] == 1.0 and d.header["t_capture"] > 0
        assert got["color"].image.ndim == 3 and got["color"].header["format"] == "MJPG"
        assert d.latency is not None and 0 <= d.latency < 1.0
        b = next(c.imu(raw=True))
        assert b.header["type"] == "imu" and b.header["kind"] == "raw" and b.samples.ay[0] == pytest.approx(-9.81)
        b = next(r for r in c.imu() if r.samples.n[0] >= 5)      # filter output (startup records may be short)
        assert b.header["kind"] == "state" and abs(np.linalg.norm([b.samples.qw[0], b.samples.qx[0],
                                                                    b.samples.qy[0], b.samples.qz[0]]) - 1) < 1e-6
        with pytest.raises(OrbbecError) as ei:
            c.update_settings({"color_profile": "1920x1080@30 MJPG"})      # fake device refuses it
        assert ei.value.status == 409
        assert c.settings()["color_profile"] == "1280x720@10 MJPG"       # previous config restored
        with pytest.raises(OrbbecError, match="no frames from color") as ei:
            c.update_settings({"color_profile": "640x480@30 RGB"})         # starts, delivers nothing
        assert ei.value.status == 409 and c.settings()["color_profile"] == "1280x720@10 MJPG"
        r = c.update_settings({"stereo_mode": "640x400@30"})
        assert r["restarted"]
        fr = next(f for f in c.frames("depth", mode="raw") if f.header["t_capture"] > r["t_applied"])
        assert fr.image.shape == (400, 640)
        with pytest.raises(OrbbecError) as ei:
            c.update_settings({"stereo_mode": "nope"})
        assert ei.value.status == 400
    counts = record(fake_node, str(tmp_path), streams="color,depth", duration=1.0, progress=False)
    assert counts["frames"] > 5
    hs = [h for h, _ in read_recording(str(tmp_path / "video.rec"), decode_jpeg=False)]
    assert {h["stream"] for h in hs} == {"color", "depth"} and all("t_recv" in h for h in hs)
    imu = list(read_recording(str(tmp_path / "imu_raw.rec")))
    assert imu and imu[0][1].dtype.names == IMU_FIELDS
    from robot_utils.nodes.orbbec.imu_filter import STATE_FIELDS
    imu = list(read_recording(str(tmp_path / "imu.rec")))
    assert imu and imu[0][1].dtype.names == STATE_FIELDS


def test_colorize_depth_lut():
    from robot_utils.nodes.orbbec.encode import COLORMAPS, colorize_depth
    from robot_utils.nodes.orbbec.models import DisplaySettings
    ds = DisplaySettings(range_mode="manual", min_mm=1000, max_mm=2000, invalid_color="#ff0000", colormap="gray")
    raw = np.array([[0, 1000, 1500, 2000, 4000]], np.uint16)
    rgb, lo, hi, valid = colorize_depth(raw, 1.0, ds)
    assert (lo, hi) == (1000.0, 2000.0)
    assert tuple(rgb[0, 0]) == (255, 0, 0)                          # invalid
    assert tuple(rgb[0, 1]) == tuple(COLORMAPS["gray"][0]) and tuple(rgb[0, 3]) == tuple(COLORMAPS["gray"][255])
    assert tuple(rgb[0, 4]) == tuple(COLORMAPS["gray"][255])        # clipped
    bgr, *_ = colorize_depth(raw, 1.0, ds, bgr=True)
    assert tuple(bgr[0, 0]) == (0, 0, 255)
    inv, *_ = colorize_depth(raw, 0.5, ds.model_copy(update={"invert": True}))   # scale: 2000 units = 1000 mm
    assert tuple(inv[0, 3]) == tuple(COLORMAPS["gray"][255])



def test_meta_and_shared_toolkit_served(fake_node):
    import httpx
    meta = httpx.get(fake_node + "/meta").json()
    assert meta["name"] == "camera" and meta["actuated"] is False and meta["page"] == "./"
    assert meta["endpoints"] == {"health": "health", "state": "state", "events": "events", "estop": None}
    assert "RU.TimeSync" in httpx.get(fake_node + "/common/ui.js").text
    st = httpx.get(fake_node + "/state").json()
    assert isinstance(st["state"], str) and "error" in st
