"""robot_utils.nodes.swerve.steering_calibration against simulated SPARKs.

Ported from rev_ctl tests/test_steering_calibration.py (commit cfcfc32).
"""
import json
import threading
import urllib.error
import urllib.request
from http.server import HTTPServer

import pytest

from robot_utils.nodes.swerve import steering_calibration as cal
from sparkcan._params import SparkParameter as P
from sparkcan.sim import SimulatedSpark

JIG = {"FL": 0.75, "FR": 0.002, "RR": 0.25, "RL": 0.5}  # simulated raw positions


@pytest.fixture
def sims(iface):
    sims = {name: SimulatedSpark(iface, can_id).start() for name, can_id in cal.STEERING_IDS.items()}
    for name, s in sims.items():
        s.position = JIG[name]
    yield sims
    for s in sims.values():
        s.close()


@pytest.fixture
def steering(iface, sims):
    st = cal.Steering(iface, cal.STEERING_IDS)
    yield st
    st.close()


def never_enabled(sims):
    return all(not s.enabled and not s.setpoint and not s.frames("SECONDARY_HEARTBEAT")
               for s in sims.values())


def test_circular_mean_wraps():
    assert abs(cal.wrap(cal.circular_mean([0.99, 0.01, 0.0]))) < 1e-9
    assert cal.circular_mean([0.25, 0.25]) == pytest.approx(0.25)


def test_measure_and_wheel_angle(steering, sims):
    for name in JIG:
        x = steering.measure(name)
        assert x == pytest.approx(JIG[name], abs=1e-4)
    # at YOR's offsets the jig positions above are "straight ahead" (FR is 0.72° off)
    assert cal.wheel_angle_deg("FL", 0.75) == pytest.approx(0)
    assert cal.wheel_angle_deg("FR", 0.002) == pytest.approx(0.72)
    assert never_enabled(sims)


def test_missing_signal_is_an_error(steering, sims):
    sims["RR"].overrides[6] = {"DUTY_CYCLE_NO_SIGNAL": 1}
    with pytest.raises(cal.CalibrationError, match="no encoder signal"):
        steering.measure("RR")


def test_zero_replaces_old_offset_and_persists(steering, sims):
    sims["FR"].params[int(P.DUTY_CYCLE_OFFSET)] = 0.1
    for name in JIG:
        assert steering.zero(name) == pytest.approx(JIG[name], abs=1e-4)
        assert sims[name].persisted[int(P.DUTY_CYCLE_OFFSET)] == pytest.approx(JIG[name], abs=1e-4)
        assert abs(cal.wrap(steering.measure(name))) < 1e-4
    assert never_enabled(sims)


def test_zero_restores_old_offset_if_not_verified(steering, sims):
    sims["FL"].overrides[5] = {"DUTY_CYCLE_ENCODER_POSITION": 0.3}  # ignores the offset
    with pytest.raises(cal.CalibrationError, match="restored"):
        steering.zero("FL")
    assert sims["FL"].params[int(P.DUTY_CYCLE_OFFSET)] == 0.0
    assert not sims["FL"].frames("PERSIST_PARAMETERS")


def test_web_page_actions(steering, sims, tmp_path):
    cal.Handler.steering, cal.Handler.out_file = steering, str(tmp_path / "positions.json")
    server = HTTPServer(("127.0.0.1", 0), cal.Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"

    def post(path, body):
        req = urllib.request.Request(base + path, json.dumps(body).encode(), method="POST")
        return json.load(urllib.request.urlopen(req, timeout=10))["message"]

    try:
        assert "Steering calibration" in urllib.request.urlopen(base + "/").read().decode()
        state = json.load(urllib.request.urlopen(base + "/state"))
        assert state["FL"]["can_id"] == 5
        with pytest.raises(urllib.error.HTTPError):  # zeroing needs the typed confirmation
            post("/zero", {})
        assert "saved" in post("/record", {})
        saved = json.loads((tmp_path / "positions.json").read_text())
        assert saved["RR"]["encoder"] == pytest.approx(0.25, abs=1e-4)
        assert "FL: zero offset 0.75000, saved" in post("/zero", {"confirm": "ZERO"})
    finally:
        server.shutdown()
        server.server_close()
    assert never_enabled(sims)


def test_zero_with_inverted_encoder(steering, sims):
    # inverted: the offset formula changes sign (REVLib: offset is set as if not inverted)
    sims["FR"].params[int(P.DUTY_CYCLE_INVERTED)] = True
    new = steering.zero("FR")
    assert abs(cal.wrap(steering.measure("FR"))) < 1e-4
    assert sims["FR"].persisted[int(P.DUTY_CYCLE_OFFSET)] == pytest.approx(new)
