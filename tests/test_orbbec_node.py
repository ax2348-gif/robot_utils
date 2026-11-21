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


def test_the_node_starts_at_the_device_colour_default():
    cat = _cat()
    cat.color.append(ProfileOption(key="640x480@15 MJPG", width=640, height=480, fps=15, format="MJPG"))
    assert Settings().color_profile is None
    assert fix_settings(Settings(), cat).color_profile == "1280x720@10 MJPG"      # the device's own default


def test_an_uncompressed_device_default_starts_as_mjpg():
    """The Gemini 305 defaults to uncompressed colour: start at the same size and rate in MJPG instead."""
    P = lambda k, d=False: ProfileOption(key=k, width=int(k.split("x")[0]), height=int(k.split("x")[1].split("@")[0]),
                                         fps=int(k.split("@")[1].split()[0]), format=k.split()[1], default=d)
    cat = _cat()
    cat.color = [P("1280x720@30 MJPG"), P("640x480@30 RGB", True), P("640x480@15 MJPG"), P("640x480@30 MJPG")]
    assert fix_settings(Settings(), cat).color_profile == "640x480@30 MJPG"              # same size and rate
    cat.color = [P("1280x720@30 MJPG"), P("848x480@30 MJPG"), P("640x480@30 YUYV", True), P("640x480@15 MJPG")]
    assert fix_settings(Settings(), cat).color_profile == "848x480@30 MJPG"              # closest size at that rate
    cat.color = [P("640x480@30 YUYV", True), P("640x480@15 MJPG")]
    assert fix_settings(Settings(), cat).color_profile == "640x480@30 YUYV"              # no MJPG at that rate
    cat.color = [P("640x480@30 RGB", True), P("640x480@30 MJPG")]
    assert fix_settings(Settings(color_profile="640x480@30 RGB"), cat).color_profile == "640x480@30 RGB"   # chosen


def test_usb2_link():
    from robot_utils.nodes.orbbec.models import usb2_link
    assert usb2_link("USB2.1") and usb2_link("USB2.0") and usb2_link("usb 2.0")
    assert not usb2_link("USB3.2") and not usb2_link("none") and not usb2_link("")


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
        desc = c.get("/description")
        assert desc["root"] == "depth_optical" and desc["device"]["serial"] and "color_optical" in desc["frames"]
        col = desc["frames"]["color_optical"]
        assert col["parent"] == "depth_optical" and col["translation"] == pytest.approx([0.025, 0.0, 0.0])
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
    assert meta["title"] == "Orbbec camera" and meta["kind"] == "camera"
    assert meta["endpoints"] == {"health": "health", "state": "state", "events": "events", "estop": None}
    assert "RU.TimeSync" in httpx.get(fake_node + "/common/ui.js").text
    st = httpx.get(fake_node + "/state").json()
    assert isinstance(st["state"], str) and "error" in st


def test_imu_fields_describe_every_column(fake_node):
    import httpx
    from robot_utils.nodes.orbbec.imu_filter import STATE_FIELDS
    from robot_utils.nodes.orbbec.protocol import IMU_FIELDS
    for raw, names in ((False, STATE_FIELDS), (True, IMU_FIELDS)):
        d = httpx.get(fake_node + "/imu/fields", params={"raw": raw}).json()
        assert d["fields"] == list(names) and len(d["units"]) == len(names)       # payload column order
        assert all(u for u in d["units"]) and all(d["doc"][f] for f in names)
        latest = httpx.get(fake_node + "/imu/latest", params={"n": 1, "raw": raw}).json()
        assert latest["fields"] == d["fields"]                                  # same columns as the data


def test_sdk_backend_opens_only_the_pinned_serial():
    """--serial: the backend opens that camera whatever the SDK's device order, and refuses
    (NoDevice naming what it sees) rather than open another one."""
    from robot_utils.nodes.orbbec.owner import NoDevice
    from robot_utils.nodes.orbbec.sdk_backend import SdkBackend

    class Devices:
        def __init__(self, serials): self.serials = serials
        def get_count(self): return len(self.serials)
        def get_device_serial_number_by_index(self, i): return self.serials[i]
        def get_device_by_index(self, i): return ("dev", self.serials[i])
        def get_device_by_serial_number(self, s): return ("dev", s)

    class Opened(Exception):
        pass

    def backend(serial, serials):
        b = SdkBackend.__new__(SdkBackend)          # no pyorbbecsdk on the test machine
        b.serial, b.ctx, b.ob = serial, type("Ctx", (), {"query_devices": lambda self: Devices(serials)})(), None
        # stop right after the device is chosen
        b.__class__ = type("B", (SdkBackend,), {"dev": property(lambda s: s.__dict__.get("_d"),
                                                                 lambda s, v: (_ for _ in ()).throw(Opened(v)))})
        return b

    with pytest.raises(Opened) as e:
        backend("HEAD", ["WRIST", "HEAD"]).open()
    assert e.value.args[0] == ("dev", "HEAD")
    with pytest.raises(Opened) as e:
        backend(None, ["WRIST", "HEAD"]).open()
    assert e.value.args[0] == ("dev", "WRIST")
    with pytest.raises(NoDevice, match="no camera with serial HEAD .connected: WRIST."):
        backend("HEAD", ["WRIST"]).open()
    with pytest.raises(NoDevice):
        backend(None, []).open()


def test_depth_filter_setting_patch_and_device_properties():
    """depth_filter: hardware by default, a pipeline key (re-applied at start), and set on the device: the hardware
    filter on and the SDK's software one off; software where the device has no hardware filter; 'device' touches
    nothing."""
    from robot_utils.nodes.orbbec.models import PIPELINE_KEYS, Settings, SettingsPatch, apply_patch
    from robot_utils.nodes.orbbec.sdk_backend import HW_FILTER, SOFT_FILTER, SdkBackend

    assert Settings().depth_filter == "hardware" and "depth_filter" in PIPELINE_KEYS
    import types
    cat = types.SimpleNamespace(available=["color", "depth"], color=[], stereo=[], colormaps=None,
                                imu=types.SimpleNamespace(available=False, sample_rates=[], accel_ranges=[],
                                                          gyro_ranges=[]))
    new, restart, _ = apply_patch(Settings(), SettingsPatch(depth_filter="software"), cat)
    assert new.depth_filter == "software" and restart
    same, restart, _ = apply_patch(new, SettingsPatch(depth_filter="software"), cat)
    assert not restart

    class Ids:
        pass
    for pid in (HW_FILTER, SOFT_FILTER):
        setattr(Ids, pid, pid)

    class Dev:
        def __init__(self, props):
            self.props, self.set = props, {}
        def is_property_supported(self, prop, perm):
            return prop in self.props
        def set_bool_property(self, prop, v):
            self.set[prop] = v

    def backend(props):
        b = SdkBackend.__new__(SdkBackend)
        b.ob = type("OB", (), {"OBPropertyID": Ids, "OBPermissionType": type("P", (), {"PERMISSION_READ_WRITE": 3})})
        b.dev = Dev(props)
        return b

    b = backend({HW_FILTER, SOFT_FILTER})
    assert b._apply_depth_filter("hardware") == "hardware" and b.dev.set == {HW_FILTER: True, SOFT_FILTER: False}
    b = backend({HW_FILTER, SOFT_FILTER})
    b._apply_depth_filter("software")
    assert b.dev.set == {HW_FILTER: False, SOFT_FILTER: True}
    b = backend({SOFT_FILTER})                                   # no hardware filter: falls back to software
    assert "software" in b._apply_depth_filter("hardware") and b.dev.set == {SOFT_FILTER: True}
    b = backend({HW_FILTER, SOFT_FILTER})
    assert b._apply_depth_filter("device") == "device defaults" and b.dev.set == {}


# -- camera modes: dual colour (Gemini 305) -----------------------------------------------------

def test_camera_mode_patch_rules():
    from robot_utils.nodes.orbbec.models import SettingsError, SettingsPatch, apply_patch, fix_settings
    cat = _cat()
    cat.modes = ["depth", "dual_color"]
    cur = fix_settings(Settings(align=True), cat)
    new, restart, _ = apply_patch(cur, SettingsPatch(camera_mode="dual_color"), cat)
    assert restart and new.camera_mode == "dual_color" and new.align is False
    assert {n for n, on in new.streams.items() if on} == {"color", "color_right"}      # the mode's own streams
    back, _, _ = apply_patch(new, SettingsPatch(camera_mode="depth"), cat)
    assert {n for n, on in back.streams.items() if on} == {"color", "depth"}
    with pytest.raises(SettingsError, match="on its own"):
        apply_patch(cur, SettingsPatch(camera_mode="dual_color", streams={"color": True}), cat)
    ok, _, _ = apply_patch(cur, SettingsPatch(camera_mode="dual_color", display={"colormap": "jet"}), cat)
    assert ok.display.colormap == "jet"
    cat.modes = ["depth"]
    with pytest.raises(SettingsError, match="not available"):
        apply_patch(cur, SettingsPatch(camera_mode="dual_color"), cat)
    # a device without the mode (or opened in another one) puts the settings in the catalog's mode
    assert fix_settings(Settings(camera_mode="dual_color"), cat).camera_mode == "depth"


def _owner(backend):
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.owner import Owner
    hub = Hub()
    seen = []
    orig = hub.publish_frames
    hub.publish_frames = lambda frames: (seen.extend(frames), orig(frames))
    o = Owner(backend, hub, Settings(), startup_s=1.5, calibration_file=None)
    o.start()
    deadline = time.time() + 5
    while o.state != "streaming" and time.time() < deadline:
        time.sleep(0.02)
    return o, seen


def test_owner_switches_to_dual_color_and_back():
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.models import SettingsPatch
    b = FakeBackend(dual=True)
    o, seen = _owner(b)
    try:
        assert o.catalog.modes == ["depth", "dual_color"] and o.catalog.mode == "depth"
        old_cat = o.catalog.catalog_id
        r = o.request_settings(SettingsPatch(camera_mode="dual_color")).result(timeout=10)
        s = r["settings"]
        assert r["restarted"] and s.camera_mode == "dual_color" and b.mode == "dual_color"
        assert {n for n, on in s.streams.items() if on} == {"color", "color_right"}
        assert o.catalog.mode == "dual_color" and o.catalog.catalog_id > old_cat and "depth" not in o.catalog.available
        frames = [f for f in seen if f.t_publish >= r["t_applied"]]
        assert {"color", "color_right"} <= {f.stream for f in frames}
        right = next(f for f in frames if f.stream == "color_right")
        assert right.fmt == "YUYV" and (right.w, right.h) == (1280, 720)         # the left's size and rate
        st = o.status()["streams"]
        assert st["color"]["label"] == "Left color" and st["color_right"]["available"] and not st["depth"]["available"]
        cal = o.calibration
        assert "color_right" in cal and cal["color_to_color_right"]["translation_mm"][0] == -18.0
        from robot_utils.nodes.orbbec.encode import encode_frame
        from robot_utils.nodes.orbbec.models import DisplaySettings
        if pytest.importorskip("cv2"):
            jpeg, extra = encode_frame(right, "jpeg", DisplaySettings())
            assert bytes(jpeg)[:2] == b"\xff\xd8" and extra["encoding"] == "jpeg"
        raw, extra = encode_frame(right, "raw", DisplaySettings())
        assert extra["format"] == "YUYV" and len(raw) == 1280 * 720 * 2
        r = o.request_settings(SettingsPatch(camera_mode="depth")).result(timeout=10)
        assert r["settings"].camera_mode == "depth" and b.mode == "depth" and o.catalog.mode == "depth"
        assert {n for n, on in r["settings"].streams.items() if on} == {"color", "depth"}
    finally:
        o.close()


def test_owner_goes_back_when_dual_color_does_not_start():
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.models import SettingsError, SettingsPatch
    b = FakeBackend(dual=True)
    start = b.start

    def refuse_dual(s):
        if s.camera_mode == "dual_color":
            raise RuntimeError("USB bandwidth")
        start(s)
    b.start = refuse_dual
    o, _ = _owner(b)
    try:
        with pytest.raises(SettingsError, match="could not switch to camera mode dual_color.*back to depth"):
            o.request_settings(SettingsPatch(camera_mode="dual_color")).result(timeout=10)
        assert b.mode == "depth" and o.catalog.mode == "depth" and o.settings.camera_mode == "depth"
        deadline = time.time() + 5
        while o.state != "streaming" and time.time() < deadline:
            time.sleep(0.02)
        assert o.state == "streaming"
    finally:
        o.close()


def test_sdk_backend_modes_presets_and_right_profile(monkeypatch):
    """The SDK side against a stub: the dual mode exists only with the preset; every open puts the device in the
    mode's preset (Default first, for the depth -> color extrinsic) and each preset load reopens the SDK (it caches a
    device's sensor set per Context); the right sensor runs at the left's size and rate, MJPG if it has it, else
    the native uncompressed format."""
    from robot_utils.nodes.orbbec import sdk_backend as sb
    monkeypatch.setattr(sb, "PRESET_GRACE_S", 0.0)

    class T:   # sensor type enum
        COLOR_SENSOR, DEPTH_SENSOR, LEFT_IR_SENSOR, RIGHT_IR_SENSOR = "C", "D", "IL", "IR"
        LEFT_COLOR_SENSOR, RIGHT_COLOR_SENSOR = "LC", "RC"

    class Camera:        # the physical device: its preset survives SDK reopens
        def __init__(self, dual, lag=0):
            self.dual, self.loaded, self.lag, self.pending = dual, [], lag, None

    class Dev:           # an SDK device handle: its sensor set is fixed when first queried
        def __init__(self, cam):
            self.cam, self._sensors = cam, None
        def load_preset(self, name):
            self.cam.loaded.append(name)
            self.cam.pending = (name == sb.DUAL_PRESET, self.cam.lag)
        def get_sensor_list(self):
            if self._sensors is None:
                p = self.cam.pending
                if p is not None:
                    if p[1] > 0:
                        self.cam.pending = (p[0], p[1] - 1)       # the device has not switched yet
                    else:
                        self.cam.dual, self.cam.pending = p[0], None
                self._sensors = ["LC", "RC"] if self.cam.dual else ["C", "D", "IL", "IR"]
            t = self._sensors
            return type("SL", (), {"get_count": lambda s: len(t), "get_type_by_index": lambda s, j: t[j]})()
        def get_available_preset_list(self):
            names = [sb.DEFAULT_PRESET, sb.DUAL_PRESET]
            return type("PL", (), {"get_count": lambda s: 2, "get_name_by_index": lambda s, k: names[k]})()
        def get_device_info(self):
            return type("I", (), {"get_serial_number": lambda s: "SN", "get_name": lambda s: "Gemini 305",
                                  "get_firmware_version": lambda s: "1", "get_connection_type": lambda s: "USB2.1"})()

    def backend(cam):
        b = sb.SdkBackend.__new__(sb.SdkBackend)
        b.ob = type("OB", (), {"OBSensorType": T})
        b.serial, b._depth_to_color, b.global_ts, b.reopens = None, None, False, 0
        b._imu_thread, b.running = None, False
        b.mode_streams = {"depth": {"color": ("C", 0)}, "dual_color": {"color": ("LC", 4), "color_right": ("RC", 5)}}

        def acquire():
            b.dev, b.pipe = Dev(cam), "pipe"
            b.reopens += 1
        b._acquire = acquire
        b._cache_depth_to_color = lambda: setattr(b, "_depth_to_color", {"translation_mm": [0, 0, 0]})
        b._catalog = lambda: b.mode
        return b

    cam = Camera(dual=False)
    b = backend(cam)
    info, mode = b.open("depth")
    assert b.modes == ["depth", "dual_color"] and mode == "depth" and cam.loaded == [sb.DEFAULT_PRESET]
    assert b._depth_to_color is not None and info.name == "Gemini 305"
    assert b.set_mode("dual_color") == "dual_color" and cam.loaded[-2:] == [sb.DEFAULT_PRESET, sb.DUAL_PRESET]
    assert b.streams == b.mode_streams["dual_color"] and cam.dual
    assert b.set_mode("depth") == "depth" and not cam.dual and b.streams == b.mode_streams["depth"]
    # a camera left in dual colour by an earlier run: opened for depth, it comes back to Default
    cam2 = Camera(dual=True)
    assert backend(cam2).open("depth")[1] == "depth" and not cam2.dual
    # the sensor set follows a few reopens late: _load_preset keeps reopening until it does
    cam3 = Camera(dual=False, lag=2)
    b3 = backend(cam3)
    b3.dev = Dev(cam3)
    b3._load_preset(sb.DUAL_PRESET, dual=True, timeout_s=2)
    assert cam3.dual and b3.reopens == 3
    # a device that ignores the preset: give up after the timeout
    b3.dev.load_preset = lambda name: None
    b3._acquire = lambda: setattr(b3, "dev", Dev(cam3))
    with pytest.raises(RuntimeError, match="did not switch its sensors"):
        b3._load_preset(sb.DEFAULT_PRESET, dual=False, timeout_s=0.05)
    # no dual preset (a Gemini 336L): depth only, no preset is ever loaded
    cam4 = Camera(dual=False)
    b4 = backend(cam4)
    b4._presets = lambda: [sb.DEFAULT_PRESET]
    assert b4.open("dual_color")[1] == "depth" and b4.modes == ["depth"] and cam4.loaded == []
    # the right sensor's profile for a left profile
    b._color = {"848x480@15 MJPG": (848, 480, 15, "MJPG"), "848x480@15 YUYV": (848, 480, 15, "YUYV")}
    b._profiles = {("color_right", (848, 480, 15, "YUYV")): "right-yuyv",
                   ("color_right", (848, 480, 15, "RGB")): "right-rgb"}
    pick = lambda key: b._profile("color_right", Settings(color_profile=key))   # noqa: E731
    assert pick("848x480@15 MJPG") == "right-yuyv"     # no MJPG on the right: native YUYV, not the SDK's RGB
    b._profiles[("color_right", (848, 480, 15, "MJPG"))] = "right-mjpg"
    assert pick("848x480@15 MJPG") == "right-mjpg" and pick("848x480@15 YUYV") == "right-yuyv"
    b._color["1280x800@10 MJPG"] = (1280, 800, 10, "MJPG")
    with pytest.raises(ValueError, match="right colour sensor has no 1280x800@10"):
        pick("1280x800@10 MJPG")


def test_owner_refuses_a_mode_with_no_streams():
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.models import SettingsError, SettingsPatch
    b = FakeBackend(dual=True)
    cat = b._catalog

    def empty_in_dual():
        c = cat()
        if b.mode == "dual_color":
            c.available = []
        return c
    b._catalog = empty_in_dual
    o, _ = _owner(b)
    try:
        with pytest.raises(SettingsError, match="no video streams"):
            o.request_settings(SettingsPatch(camera_mode="dual_color")).result(timeout=10)
        assert b.mode == "depth" and o.settings.camera_mode == "depth"
    finally:
        o.close()
