"""Typed responses of the description node: every JSON endpoint declares a response model, and that
model passes the handler's output through unchanged: same keys, same key order, same JSON types
(int stays int). response_model silently drops undeclared keys, so this compares the raw handler
output with what the client gets. Runs against a fake camera node and the simulated swerve node."""
import json
import socket
import threading
import time

import fastapi.routing
import pytest
import uvicorn
import yaml
from fastapi.routing import APIRoute
from fastapi.testclient import TestClient
from pydantic import TypeAdapter

from robot_utils.nodes.common.locks import LockStore
from robot_utils.nodes.description.node import DescriptionNode
from robot_utils.nodes.description.server import create_app

NON_JSON = {"/events", "/cameras/{name}/depth.jpg", "/jobs/{jid}/raw.npz", "/calibrations/{kind}/{serial}/{cid}/raw.npz"}


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        v = predicate()
        if v:
            return v
        time.sleep(0.05)
    raise AssertionError("condition not met in time")


def serve(app):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning",
                                           timeout_graceful_shutdown=1))
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    wait_for(lambda: server.started, 10)
    return f"http://127.0.0.1:{port}", server, t


def same(a, b, path="$", order=True):
    """Equal JSON with the same types (1 and 1.0 differ) and, with ``order``, the same key order."""
    assert type(a) is type(b), f"{path}: {a!r} vs {b!r}"
    if isinstance(a, dict):
        assert (list(a) if order else set(a)) == (list(b) if order else set(b)), f"{path}: keys {list(a)} vs {list(b)}"
        for k in a:
            same(a[k], b[k], f"{path}.{k}", order)
    elif isinstance(a, list):
        assert len(a) == len(b), f"{path}: length"
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, f"{path}[{i}]", order)
    else:
        assert a == b, f"{path}: {a!r} vs {b!r}"


@pytest.fixture
def api(monkeypatch, tmp_path):
    """(call, node, routes): ``call(method, path, ...)`` returns the response JSON after checking it."""
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.orbbec.server import create_app as camera_app
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as swerve_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim

    raw = []
    fields = set()                          # the description app's response fields (other servers run here too)
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        if id(kw.get("field")) in fields:
            raw.append(json.loads(json.dumps(kw["response_content"])))   # as plain JSON, before the model
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)

    sims, iface = start_sim()
    sw = SwerveNode(iface)
    sw_url, sw_srv, sw_t = serve(swerve_app(sw))
    wait_for(lambda: sw.status == "running", 10)
    hub = Hub()
    cam_url, cam_srv, cam_t = serve(camera_app(Owner(FakeBackend(), hub, Settings(), startup_s=1.5,
                                                     calibration_file=str(tmp_path / "imu.json")), hub))
    data = tmp_path / "desc"
    data.mkdir()
    (data / "description.yaml").write_text(yaml.safe_dump({"cameras": {"camera": {"url": cam_url}}}))
    node = DescriptionNode(str(data), sw_url)
    app = create_app(node)
    routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}
    fields.update(id(r.response_field) for r in routes.values() if r.response_field is not None)

    called = set()

    def call(method, path, route=None, order=True, **kw):
        called.add((method, route or path))
        raw.clear()
        r = client.request(method, path, **kw)
        assert r.status_code == 200, (path, r.text)
        route = routes[(method, route or path)]
        assert route.response_model is not None, f"{method} {path} has no response model"
        TypeAdapter(route.response_model).validate_python(r.json())
        assert len(raw) == 1, f"{method} {path}: handler output not captured"
        try:
            same(r.json(), raw[0], order=order)
        except AssertionError as e:
            raise AssertionError(f"{method} {path}: response differs from the handler's output: {e}") from None
        return r.json()

    with TestClient(app) as client:
        wait_for(lambda: client.get("/description").json()["cameras"]["camera"]["reachable"], 15)
        yield call, node, routes, client, called
    for s, th in ((cam_srv, cam_t), (sw_srv, sw_t)):
        s.should_exit = True
        th.join(10)
    stop_sim(sims, iface)


# A ground record as stored on the robot (meta.json of 2026-11-11, trimmed): the camera node's description
# with real intrinsics and distortion, ints and floats as written.
_K = {"k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0, "k5": 0.0, "k6": 0.0, "p1": 0.0, "p2": 0.0}
ROBOT_GROUND = {
    "params": {"polygons": [[[0.1, 0.7], [0.9, 0.7], [0.9, 0.98], [0.1, 0.98]]], "frames": 10, "max_tilt_deg": 30.0,
               "tol": 0.01},
    "result": {"normal": [0.0173, -0.9239, -0.3822], "height": 1.128, "rms": 0.0038, "inliers": 86231,
               "n_points": 120000, "angle_to_gravity_deg": 1.25},
    "gravity_up_depth": [0.17, -9.06, -3.75],
    "camera": {"kind": "orbbec", "root": "depth_optical",
               "device": {"name": "Orbbec Gemini 336L", "serial": "FAKE0001", "firmware": "1.4.60"},
               "conventions": {"optical": "x right, y down, z out of the lens", "units": "m",
                               "transform": "p_parent = rotation @ p_child + translation"},
               "frames": {"depth_optical": {"parent": None},
                          "color_optical": {"parent": "depth_optical",
                                            "rotation": [[0.999992, -0.003328, 0.002076], [0.003328, 0.999994, -0.000156],
                                                         [-0.002076, 0.000163, 0.999998]],
                                            "translation": [0.023692, 9.1e-05, -0.000437]},
                          "imu": {"parent": "depth_optical", "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                                  "translation": [0.007886, 0.001086, -0.014248]}},
               "intrinsics": {"depth": {"fx": 415.50421142578125, "fy": 415.50421142578125, "cx": 423.3375244140625,
                                        "cy": 238.67501831054688, "width": 848, "height": 480, "distortion": _K},
                              "color": {"fx": 365.7105712890625, "fy": 365.7886047363281, "cx": 320.1219177246094,
                                        "cy": 236.58285522460938, "width": 640, "height": 480,
                                        "distortion": {**_K, "k1": -0.03221883252263069, "k2": 0.03856267035007477,
                                                       "k3": -0.013476075604557991, "p1": -0.000343605614034459,
                                                       "p2": -0.00011862569954246283}}},
               "aligned_depth": False, "settings_version": 3},
    "frame_seqs": [101, 102], "version": "0.3.0", "summary": "as on the robot", "quality": {"ok": True, "problems": []}, "camera_name": "camera"}


def job_done(client, jid, timeout=40):
    return wait_for(lambda: (j := client.get(f"/jobs/{jid}").json())["state"] != "running" and j, timeout)


def test_every_json_endpoint_is_typed_and_lossless(api, tmp_locks_dir):
    call, node, routes, client, called = api
    untyped = {f"{m} {p}" for (m, p), r in routes.items()
               if r.include_in_schema and p not in NON_JSON and r.response_model is None}
    assert not untyped
    call("GET", "/time")
    assert call("GET", "/health")["state"] == "ready"
    assert call("GET", "/meta")["moves"]                                # an extra key survives
    st = call("GET", "/state")                                          # idle: no job
    assert st["job"] is None and st["cameras"]["camera"]["serial"] == "FAKE0001"
    assert call("GET", "/jobs") == []
    assert call("POST", "/jobs/abort") == {"detail": "no job running"}
    assert call("GET", "/calibrations") == []
    d = call("GET", "/description")
    assert {"base", "camera/depth_optical", "camera/color_optical"} <= set(d["frames"])
    assert d["cameras"]["camera"]["from"] == {"nominal": True} and d["cameras"]["camera"]["intrinsics"]
    call("GET", "/transform")
    call("GET", "/transform", params={"from": "camera/color_optical", "to": "base"})
    cam = call("POST", "/cameras/camera/nominal", route="/cameras/{name}/nominal",
               json={"xyz": [0.1, 0.0, 0.4], "rpy": [-1.5707963, 0, -1.5707963]})
    assert cam["nominal"]["rpy"][1] == 0.0                              # the body model made it a float

    # ground: the fake camera's depth is a ramp, so the fit may fail; a stored record covers the rest
    j = call("POST", "/calibrate/ground", json={"frames": 2, "polygons": [[[0, 0.5], [1, 0.5], [1, 1], [0, 1]]]})
    j = job_done(client, j["id"])
    j = call("GET", f"/jobs/{j['id']}", route="/jobs/{jid}")
    assert j["kind"] == "ground" and j["state"] in ("done", "failed")
    if j["state"] == "done":                                            # the job's own record
        call("GET", f"/calibrations/ground/FAKE0001/{j['result']['id']}", route="/calibrations/{kind}/{serial}/{cid}")
    gid = node.store.save("ground", "FAKE0001", {
        "params": {"polygons": None, "frames": 10, "max_tilt_deg": 30.0, "tol": 0.01},
        "result": {"normal": [0.0, -0.96, -0.28], "height": 0.5, "rms": 0.003, "inliers": 5000, "n_points": 9000,
                   "angle_to_gravity_deg": 0.4},
        "gravity_up_depth": [0.0, -9.4, -2.7], "camera": {"device": {"serial": "FAKE0001"}}, "frame_seqs": [1, 2],
        "version": "0", "summary": "s", "quality": {"ok": True, "problems": []}, "camera_name": "camera"}, {})
    rec = call("GET", f"/calibrations/ground/FAKE0001/{gid}", route="/calibrations/{kind}/{serial}/{cid}")
    assert rec["result"]["inliers"] == 5000 and rec["frame_seqs"] == [1, 2]
    old = node.store.save("ground", "FAKE0001", {"result": {"normal": [0.0, -1.0, 0.0], "height": 0.5}, "summary": "old"},
                          {}, activate=False)
    rec = call("GET", f"/calibrations/ground/FAKE0001/{old}", route="/calibrations/{kind}/{serial}/{cid}")
    assert rec["result"]["normal"] == [0.0, -1.0, 0.0] and "quality" not in rec   # a sparse (older) record
    new = node.store.save("ground", "FAKE0001", {"future_key": {"a": 1}, "summary": "new"}, {}, activate=False)
    rec = call("GET", f"/calibrations/ground/FAKE0001/{new}", route="/calibrations/{kind}/{serial}/{cid}",
               order=False)                                             # unknown keys kept (after the known ones)
    assert rec["future_key"] == {"a": 1}
    real = node.store.save("ground", "FAKE0001", ROBOT_GROUND, {}, activate=False)
    rec = call("GET", f"/calibrations/ground/FAKE0001/{real}", route="/calibrations/{kind}/{serial}/{cid}")
    assert rec["camera"]["intrinsics"]["color"]["distortion"]["p2"] == -0.00011862569954246283

    # yaw: a full run on the simulated base (refused by the quality gate: the fake IMU feels nothing)
    j = call("POST", "/calibrate/yaw", json={"amplitude": 0.04, "frequency": 2.0, "cycles": 2, "confirm": True})
    assert j["state"] == "running" and j["params"]["camera"] == "camera"
    st = call("GET", "/state")                                          # a running job
    assert st["state"] == "calibrating"
    j = job_done(client, j["id"])
    assert j["state"] == "done", j
    j = call("GET", f"/jobs/{j['id']}", route="/jobs/{jid}")
    assert j["result"]["vectors"] and "angle_0_1" in j["result"]["checks"]
    call("GET", "/jobs")
    st = call("GET", "/state")                                          # a finished job with its result
    assert st["job"]["result"]["id"] == j["result"]["id"]
    cals = call("GET", "/calibrations", params={"type": "yaw"})
    c = cals[0]
    rec = call("GET", f"/calibrations/yaw/{c['serial']}/{c['id']}", route="/calibrations/{kind}/{serial}/{cid}")
    assert rec["type"] == "yaw" and rec["limits"]["cycles_min"] == 2   # an int stays an int
    res = call("POST", f"/calibrations/yaw/{c['serial']}/{c['id']}/resolve",
               route="/calibrations/yaw/{serial}/{cid}/resolve", json={"accel_lowpass_hz": 3.0, "activate": False})
    assert res["accel_lowpass_hz"] == 3.0
    rec = call("GET", f"/calibrations/yaw/{c['serial']}/{res['id']}", route="/calibrations/{kind}/{serial}/{cid}")
    assert rec["resolved_from"] == c["id"]
    act = call("POST", "/calibrations/yaw/activate", route="/calibrations/{kind}/activate",
               json={"serial": c["serial"], "id": c["id"]})
    assert act["FAKE0001"] == {"ground": gid, "yaw": c["id"]}
    act = call("POST", "/calibrations/ground/activate", route="/calibrations/{kind}/activate",
               json={"serial": c["serial"], "id": None})           # none active
    assert act["FAKE0001"] == {"yaw": c["id"]}
    call("POST", "/calibrations/ground/activate", route="/calibrations/{kind}/activate",
         json={"serial": c["serial"], "id": gid})
    d = call("GET", "/description")                                     # composed from ground + yaw
    assert set(d["cameras"]["camera"]["from"]) == {"nominal", "ground", "yaw", "residual_deg"}
    assert d["frames"]["camera/depth_optical"]["source"]["yaw"] == c["id"]
    call("GET", "/calibrations")

    # abort a running yaw job; then the e-stop
    j = call("POST", "/calibrate/yaw", json={"amplitude": 0.04, "frequency": 1.0, "cycles": 10, "confirm": True})
    wait_for(lambda: node.swerve_io(node.swerve_url).state()["lease"]["held"], 10)
    assert call("POST", "/jobs/abort")["id"] == j["id"]
    job_done(client, j["id"])
    wait_for(lambda: client.get(f"/jobs/{j['id']}").json()["raw"])   # saved just after the state turns aborted
    j = call("GET", f"/jobs/{j['id']}", route="/jobs/{jid}")
    assert j["state"] == "aborted" and j["result"] is None and j["raw"] == f"jobs/{j['id']}/raw.npz"
    assert call("POST", "/estop", json={"reason": "test", "source": "pytest"})["latched"]
    st = call("GET", "/state")                                          # e-stop latched
    assert st["estop"]["latched"]
    assert call("POST", "/estop/clear") == {"latched": False}

    store = LockStore(tmp_locks_dir)
    store.place("chock", "swerve", "pytest", "test")
    store.place("cover", "camera", "pytest", "test")
    st = wait_for(lambda: (s := call("GET", "/state"))["chocked"] and s["covered"] and s)
    assert len(st["locks"]) == 2

    json_routes = {(m, p) for (m, p), r in routes.items() if r.include_in_schema and p not in NON_JSON}
    assert json_routes <= called, f"not exercised: {json_routes - called}"

    transform = node.transform                                          # the check catches a dropped key
    node.transform = lambda a, b: {**transform(a, b), "undeclared": 1}
    with pytest.raises(AssertionError, match="differs"):
        call("GET", "/transform")


def test_openapi_documents_shapes_and_media_types(tmp_path):
    spec = create_app(DescriptionNode(str(tmp_path)), start=False).openapi()
    ok = {(m.upper(), p): op["responses"]["200"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    assert set(ok[("GET", "/events")]["content"]) == {"text/event-stream"}
    assert set(ok[("GET", "/cameras/{name}/depth.jpg")]["content"]) == {"image/jpeg"}
    for p in ("/jobs/{jid}/raw.npz", "/calibrations/{kind}/{serial}/{cid}/raw.npz"):
        assert set(ok[("GET", p)]["content"]) == {"application/octet-stream"}
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            schema = resp["content"]["application/json"]["schema"]
            assert "$ref" in json.dumps(schema) or "additionalProperties" in schema, (m, p)
    schemas = spec["components"]["schemas"]
    assert "p_from" in spec["paths"]["/transform"]["get"]["summary"]
    assert "frames_in_base" in schemas["DescriptionOut"]["properties"]
    assert "from" in schemas["CameraOut"]["properties"]
    assert schemas["YawRecord"]["additionalProperties"] is True
    # this node's own models: every field described, nothing free-form (the shared models are common's)
    shared = {"EStopState", "EStopTriggered", "EStopCleared", "Event", "LockOut", "MetaEndpoints", "NodeMeta",
              "Health", "TimeResponse", "HTTPValidationError", "ValidationError"}
    for name, sch in schemas.items():
        if name in shared:
            continue
        for f, p in (sch.get("properties") or {}).items():
            assert p.get("description") or "$ref" in p, f"{name}.{f} has no description"
            for alt in [p] + p.get("anyOf", []):
                assert not (alt.get("type") == "object" and not alt.get("properties")
                            and alt.get("additionalProperties") in (None, True, {})), f"{name}.{f} is free-form"
    for p in ("/jobs/{jid}", "/calibrations/{kind}/{serial}/{cid}", "/calibrations/{kind}/activate"):
        method = "post" if p.endswith("activate") else "get"
        assert "$ref" in spec["paths"][p][method]["responses"]["200"]["content"]["application/json"]["schema"], p
