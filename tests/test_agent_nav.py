"""agent_nav: geometry on a synthetic scene, chat parsing, and end to end with the simulated swerve,
estimator (odometry only), pose_servo, a fake 3d_perception rendering the scene from the estimated
pose, and agent_nav itself (real servers)."""
import json
import math
import os
import socket
import threading
import time

import httpx
import numpy as np
import pytest
import uvicorn

from robot_utils.nodes.agent_nav import chat
from robot_utils.nodes.agent_nav import geometry as G
from robot_utils.nodes.agent_nav.sim import CALIBRATION, Box, Rect, Scene, demo_scene, fake_perception_app

CAM = G.Camera.from_calibration(CALIBRATION)
T_BC = np.array(CALIBRATION["base_to_color"])


def board_px(scene_rect, pose=(0, 0, 0), z=None):
    T = G.pose_matrix(*pose)
    pw = np.array([scene_rect.x, scene_rect.y, z if z is not None else (scene_rect.z0 + scene_rect.z1) / 2, 1.0])
    pb = np.linalg.inv(T) @ pw
    return G.project(CAM, ((pb[:3] - T_BC[:3, 3]) @ T_BC[:3, :3])[None])[0]


# -- geometry ------------------------------------------------------------------------------------------

def test_from_color_matches_the_live_grid_intrinsics():
    c = G.Camera.from_color(CALIBRATION["color"], 640, 360)
    g = CALIBRATION["depth_grid"]
    assert (c.fx, c.cx, c.cy) == pytest.approx((g["fx"], g["cx"], g["cy"]), abs=1e-6)


def test_plane_fit_recovers_the_board_and_flags_the_image_edge():
    sc = demo_scene()
    dm, _ = sc.render((0, 0, 0))
    cl = G.Cloud(dm, CAM, T_BC, (0, 0, 0))
    u, v = board_px(sc.rects[0])
    p = G.fit_plane(cl, G.selection_mask({"box": [u - 150, v - 50, u + 150, v + 50]}, [], CAM))
    assert p["base"]["centre"][:2] == pytest.approx([3.0, 0.3], abs=0.03)
    assert p["base"]["normal"][0] < -0.99 and p["orientation"] == "vertical"
    assert p["world"]["facing_heading"] == pytest.approx(math.pi, abs=0.02) or \
        p["world"]["facing_heading"] == pytest.approx(-math.pi, abs=0.02)
    assert not p["touches_image_edge"]
    # a board running off the image edge is flagged
    sc2 = Scene(rects=[Rect(2.0, 1.2, math.pi, 1.9, 0.75, 1.75)])
    dm2, _ = sc2.render((0, 0, 0))
    cl2 = G.Cloud(dm2, CAM, T_BC, (0, 0, 0))
    p2 = G.fit_plane(cl2, G.selection_mask({"box": [0, 0, 400, 360]}, [], CAM))
    assert p2["touches_image_edge"] and "warning" in p2


def test_exclusion_and_polygon_selections():
    m_box = G.selection_mask({"box": [100, 100, 300, 300]}, [], CAM)
    m_ex = G.selection_mask({"box": [100, 100, 300, 300]}, [{"disk": [200, 200, 50]}], CAM)
    m_poly = G.selection_mask({"polygon": [[100, 100], [300, 100], [100, 300]]}, [], CAM)
    assert m_ex.sum() < m_box.sum() and abs(m_poly.sum() - m_box.sum() / 2) < 0.05 * m_box.sum()
    with pytest.raises(ValueError):
        G.check_shape({"box": [1, 2, 3]}, CAM)
    with pytest.raises(ValueError):
        G.check_shape({"box": [1, 2, 3, 4], "point": [1, 2]}, CAM)


def test_path_check_is_loud_about_obstacles_and_the_blind_zone():
    sc = demo_scene()
    dm, _ = sc.render((0, 0, 0))
    cl = G.Cloud(dm, CAM, T_BC, (0, 0, 0))
    clear = G.path_check(cl, (0, 0, 0), (2.0, 0.3, 0))
    assert clear["obstacle_points_in_path"] == 0 and clear["min_clearance_m"] > 0.2
    assert clear["verified_fraction"] < 0.95 and any("UNVERIFIED" in w for w in clear["warnings"])
    blocked = G.path_check(cl, (0, 0, 0), (1.8, -0.75, 0))
    assert blocked["obstacle_points_in_path"] > 100 and any("OBSTACLE" in w for w in blocked["warnings"])
    png = G.bev_image(cl, path=blocked, goal_base=(1.8, -0.75, 0))
    assert png[:4] == b"\x89PNG"


def test_chat_parsing():
    assert chat.parse("look grid 1.0") == ("look", {"grid": True, "scale": 1.0})
    m, kw = chat.parse("plane poly 10,10 100,10 50,90 exclude disk 50 40 5 on s2")
    assert m == "query" and kw["select"]["polygon"][2] == [50.0, 90.0] and kw["exclude"] == [{"disk": [50.0, 40.0, 5.0]}]
    assert kw["snapshot"] == "s2" and kw["kind"] == "plane"
    m, kw = chat.parse("goal 0.5 0 90d snapshot:s3 speed=0.1")
    assert m == "goal" and kw["theta"] == pytest.approx(math.pi / 2) and kw["frame"] == "snapshot:s3"
    assert kw["options"] == {"speed": 0.1}
    with pytest.raises(ValueError):
        chat.parse("fly 3")


# -- end to end -----------------------------------------------------------------------------------------

def wait_for(pred, timeout=10.0, interval=0.05):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = pred()
        if v:
            return v
        time.sleep(interval)
    raise AssertionError("condition not met in time")


def serve(app):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    srv = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning", timeout_graceful_shutdown=1))
    t = threading.Thread(target=srv.run, daemon=True)
    t.start()
    wait_for(lambda: srv.started, 10)
    return f"http://127.0.0.1:{port}", srv, t


@pytest.fixture
def stack(tmp_path, request):
    from robot_utils.nodes.agent_nav.node import AgentNav
    from robot_utils.nodes.agent_nav.server import create_app as nav_app
    from robot_utils.nodes.estimator.node import EstimatorNode
    from robot_utils.nodes.estimator.server import create_app as est_app
    from robot_utils.nodes.pose_servo.node import PoseServoNode
    from robot_utils.nodes.pose_servo.server import create_app as ps_app
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as sw_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim

    scene = getattr(request, "param", None) or demo_scene()
    sims, iface = start_sim()
    sw = SwerveNode(iface)
    servers = []
    sw_url, *s = serve(sw_app(sw)); servers.append(s)
    wait_for(lambda: sw.status == "running", 10)
    est = EstimatorNode(sw_url, None)
    est_url, *s = serve(est_app(est)); servers.append(s)

    def pose():
        p = est.state().get("pose")
        return (p["x"], p["y"], p["theta"]) if p else (0.0, 0.0, 0.0)

    per_url, *s = serve(fake_perception_app(scene, pose)); servers.append(s)
    ps = PoseServoNode(sw_url, est_url)
    ps_url, *s = serve(ps_app(ps)); servers.append(s)
    nav = AgentNav(ps_url, per_url, est_url, str(tmp_path / "nav"), "cautious", require_imu=False, description=None)
    url, *s = serve(nav_app(nav)); servers.append(s)
    wait_for(lambda: nav.pose_now() is not None, 15)
    yield {"url": url, "nav": nav, "swerve": sw_url, "est": est_url, "data": str(tmp_path / "nav")}
    for srv, t in reversed(servers):
        srv.should_exit = True
        t.join(10)
    stop_sim(sims, iface)


def post(st, path, body=None, human=False):
    return httpx.post(st["url"] + path, json=body if body is not None else {}, timeout=20,
                      headers={"X-Operator": "human"} if human else {})


def get(st, path, **kw):
    return httpx.get(st["url"] + path, timeout=20, params=kw)


def test_full_agent_session(stack):
    st = stack
    assert "You decide" in get(st, "/guide.md").text
    assert post(st, "/look").status_code == 409                       # no session
    r = post(st, "/session", {"agent": "Tester", "avatar": "🦉", "comment": "hello"})
    assert r.status_code == 200 and r.json()["profile"] == "cautious"
    assert post(st, "/enable", {"caps": {"max_speed": 0.5}}).status_code == 422          # cannot raise
    assert post(st, "/enable", {"caps": {"max_speed": 0.12}, "comment": "slow"}).json()["caps"]["max_speed"] == 0.12
    look = post(st, "/look", {"grid": True}).json()
    assert "snapshot" in look, look
    assert look["snapshot"] == "s1" and look["pose_world"] is not None and look["depth_fraction"] > 0.5
    img = get(st, "/" + look["image"])
    assert img.status_code == 200 and img.headers["content-type"] == "image/jpeg"
    assert get(st, "/" + look["bev"]).content[:4] == b"\x89PNG"
    u, v = board_px(demo_scene().rects[0])
    q = post(st, "/query/plane", {"select": {"polygon": [[u - 150, v - 50], [u + 150, v - 50], [u + 150, v + 50], [u - 150, v + 50]]},
                                  "comment": "the board"}).json()
    assert q["world"]["centre"][:2] == pytest.approx([3.0, 0.3], abs=0.05)
    reg = post(st, "/query/region", {"select": {"point": [u, v]}}).json()
    assert reg["base"]["centroid"][0] == pytest.approx(3.0, abs=0.05)
    ap = post(st, "/plan/approach", {"query": q["query"], "standoff": 2.4}).json()
    assert ap["goal"]["world"][:2] == pytest.approx([0.6, 0.3], abs=0.06) and "nothing moves" in ap["note"]
    assert abs(abs(ap["goal"]["world"][2])) < 0.05                    # facing the board (heading ~0)
    # nothing moved from planning
    assert get(st, "/state").json()["pose_servo"]["status"] == "idle"
    # move to the proposal (explicit), wait asynchronously
    g = post(st, "/goal", {**ap["proposal"], "comment": "approach the board"}).json()
    assert g["goal_id"] and isinstance(g["warnings"], list)
    res = get(st, f"/goals/{g['goal_id']}/wait", timeout=40).json()
    assert res["done"] and res["result"]["result"] == "reached", res
    p = get(st, "/state").json()["pose_world"]
    assert p[:2] == pytest.approx([0.6, 0.3], abs=0.05)
    # memory
    post(st, "/memory/place", {"name": "near_board"})
    post(st, "/memory/note", {"text": "board is 3 m from the start"})
    mem = get(st, "/memory").json()
    assert "near_board" in mem["places"] and mem["notes"][0]["text"].startswith("board")
    # transcript: memory and disk (thumbnails)
    log = get(st, "/log").json()
    kinds = [t["kind"] for t in log["turns"]]
    for k in ("session", "enable", "look", "query_plane", "plan_approach", "goal", "event"):
        assert k in kinds, kinds
    assert any(t["comment"] == "approach the board" for t in log["turns"])
    sid = log["session"]["id"]
    lines = open(os.path.join(st["data"], sid, "log.jsonl")).read().splitlines()
    first = min(t["n"] for t in log["turns"] if t["kind"] == "session")
    assert len(lines) == log["last"] - first + 1                       # the session's turns, all on disk
    thumbs = os.listdir(os.path.join(st["data"], sid, "thumbs"))
    assert thumbs and get(st, f"/sessions/{sid}/thumbs/{thumbs[0]}").status_code == 200
    assert get(st, "/sessions").json()[0]["id"] == sid
    assert post(st, "/disable").status_code == 200


def test_loud_but_not_binding_path_check(stack):
    st = stack
    post(st, "/session", {"agent": "T"})
    post(st, "/enable")
    post(st, "/look")
    chk = post(st, "/plan/check", {"frame": "local", "x": 1.4, "y": -0.75}).json()
    assert chk["check"]["obstacle_points_in_path"] > 0 and any("OBSTACLE" in w for w in chk["check"]["warnings"])
    g = post(st, "/goal", {"frame": "local", "x": 0.3, "y": -0.2, "comment": "short, checked"}).json()
    assert g["goal_id"]                                                  # accepted, warnings or not
    assert get(st, f"/goals/{g['goal_id']}/wait", timeout=30).json()["result"]["result"] == "reached"


def test_human_hold_and_chat(stack):
    st = stack
    post(st, "/session", {"agent": "T"})
    assert post(st, "/enable").status_code == 200
    assert post(st, "/disable", human=True).json()["latched"]
    r = post(st, "/enable")
    assert r.status_code == 423 and "human" in r.json()["detail"]
    assert post(st, "/goal", {"x": 0.1}).status_code == 423
    assert post(st, "/release").status_code == 403                       # agents cannot release
    assert post(st, "/release", human=True).json()["released"]
    assert post(st, "/enable").status_code == 200
    assert post(st, "/chat", {"text": "look grid"}).json()["snapshot"]
    r = post(st, "/chat", {"text": "region box 400 20 700 160"})
    assert r.status_code == 200 and r.json()["kind"] == "region"
    g = post(st, "/chat", {"text": "goal 0.2 0 10d"}).json()
    assert get(st, f"/goals/{g['goal_id']}/wait", timeout=30).json()["result"]["result"] == "reached"
    assert post(st, "/chat", {"text": "fly away"}).status_code == 422
    turns = get(st, "/log").json()["turns"]
    assert any(t["actor"] == "human" and t["kind"] == "goal" for t in turns)
    assert any(t["actor"] == "human" and "error" in t["reply"] for t in turns)


WALL_AHEAD = Scene(boxes=[Box(1.05, -0.6, 1.35, 0.6, 0.8)])


def _reflex_event(st, timeout=3.0):
    """The latest reflex-stop event: logged just after the stop is sent (with its mask), so it can land a
    moment after the goal's result."""
    deadline = time.monotonic() + timeout
    while True:
        turns = get(st, "/log").json()["turns"]
        ev = [t for t in turns if t["actor"] == "robot" and t["reply"].get("message", "").startswith("Reflex stop")]
        if ev or time.monotonic() > deadline:
            assert ev, turns
            return ev[-1]["reply"]
        time.sleep(0.1)


@pytest.mark.parametrize("stack", [WALL_AHEAD], indirect=True)
def test_reflex_stops_before_an_obstacle(stack):
    st = stack
    post(st, "/session", {"agent": "T"})
    post(st, "/enable", {"caps": {"max_distance": 2.0}})
    post(st, "/look")
    g = post(st, "/goal", {"x": 1.5, "comment": "deliberately into the box: the reflex must stop"}).json()
    assert any("OBSTACLE" in w for w in g["warnings"])                   # loud
    res = get(st, f"/goals/{g['goal_id']}/wait", timeout=40).json()
    assert res["result"]["result"] == "stopped", res
    x = get(st, "/state").json()["pose_world"][0]
    assert x + G.HALF < 1.05, x                                          # front edge short of the box face
    r = _reflex_event(st)
    assert r["margin_m"] == 0.12 and r["points"] >= 25 and set(r["points_base"]) == {"x", "y", "z"}
    assert get(st, "/" + r["mask"]).headers["content-type"] == "image/jpeg"     # the mask of what it counted


@pytest.mark.parametrize("stack", [WALL_AHEAD], indirect=True)
def test_reflex_margin_brings_the_robot_closer_without_contact(stack):
    """A small per-goal margin on a slow final leg ends a few cm from the box face, never touching it
    (the default 0.12 m margin leaves ~0.15 m at full speed in the sim)."""
    st = stack
    post(st, "/session", {"agent": "T"})
    post(st, "/enable", {"caps": {"max_distance": 2.0}})
    post(st, "/look")
    assert post(st, "/goal", {"x": 0.5, "reflex_margin_m": 0}).status_code == 422          # must be > 0
    g = post(st, "/goal", {"x": 1.5, "speed": 0.05, "reflex_margin_m": 0.02, "comment": "close to the box face"}).json()
    assert g["reflex_margin_m"] == 0.02
    res = get(st, f"/goals/{g['goal_id']}/wait", timeout=40).json()
    assert res["result"]["result"] == "stopped", res
    gap = 1.05 - (get(st, "/state").json()["pose_world"][0] + G.HALF)
    assert 0 < gap < 0.08, gap                       # measured 0.042-0.048 m
    assert _reflex_event(st)["margin_m"] == 0.02


def test_page_meta_and_refusals(stack):
    st = stack
    page = get(st, "/").text
    assert 'src="common/ui.js"' in page and 'api("/' not in page and "innerHTML = r" not in page
    meta = get(st, "/meta").json()
    assert meta["name"] == "agent_nav" and meta["guide"] == "guide.md"
    post(st, "/session", {"agent": "T"})
    assert post(st, "/query/plane", {"select": {"box": [0, 0, 10, 10]}}).status_code == 409   # no snapshot yet
    post(st, "/look")
    assert post(st, "/query/plane", {"snapshot": "s9", "select": {"box": [0, 0, 10, 10]}}).status_code == 404
    assert post(st, "/query/region", {"select": {"triangle": [1]}}).status_code == 422
    assert post(st, "/session", {"agent": "T", "avatar": "x" * 40}).status_code == 422


def test_session_management(stack):
    import io
    import zipfile
    st = stack
    post(st, "/session", {"agent": "First", "avatar": "🦉"})
    post(st, "/look")
    first = get(st, "/state").json()["session"]["id"]
    time.sleep(1.1)                                                      # session ids are per second
    post(st, "/session", {"agent": "operator", "avatar": "🧑"}, human=True)      # replaces (ends) the first
    second = get(st, "/state").json()["session"]["id"]
    assert second != first
    turns = get(st, "/log").json()["turns"]
    assert {t["session"] for t in turns if t["kind"] == "look"} == {first}
    rows = get(st, "/sessions").json()
    assert [r["id"] for r in rows][:2] == [second, first] and rows[0]["live"] and not rows[1]["live"]
    assert rows[1]["ended"] and rows[1]["turns"] >= 3
    stored = get(st, f"/sessions/{first}").json()
    assert stored["session"]["agent"] == "First" and any(t["kind"] == "look" for t in stored["turns"])
    z = zipfile.ZipFile(io.BytesIO(get(st, f"/sessions/{first}/export.zip").content))
    names = z.namelist()
    assert f"agent_nav_{first}/log.jsonl" in names and any("/thumbs/" in n for n in names)
    assert httpx.delete(st["url"] + f"/sessions/{first}").status_code == 403                 # agents cannot delete
    assert httpx.delete(st["url"] + f"/sessions/{second}", headers={"X-Operator": "human"}).status_code == 409  # live
    assert httpx.delete(st["url"] + f"/sessions/{first}", headers={"X-Operator": "human"}).json()["deleted"] == first
    assert get(st, f"/sessions/{first}").status_code == 404
    assert get(st, "/sessions/..%2Fetc").status_code == 404


def test_robot_description(stack):
    st = stack
    r = post(st, "/session", {"agent": "T"}).json()
    assert "camera" in r["camera_summary"] and "nearest visible floor" in r["camera_summary"]
    d = get(st, "/robot").json()
    cam = d["camera"]
    assert cam["calibrated"] and cam["height_m"] == pytest.approx(1.127, abs=0.01)
    assert cam["pitch_down_deg"] == pytest.approx(22.7, abs=0.5) and cam["hfov_deg"] == pytest.approx(93, abs=1.5)
    assert 0.7 < cam["nearest_visible_floor_m"] < 0.95 and "blind" in cam["blind_zone"] or "never visible" in cam["blind_zone"]
    assert len(d["body"]["wheels"]) == 4 and d["body"]["footprint_m"][0] == pytest.approx(0.432, abs=0.01)
    assert d["warnings"] and "description node" in d["warnings"][0]          # none configured: built-in body
    assert get(st, "/" + d["diagram"]).content[:4] == b"\x89PNG"
    assert any(t["kind"] == "robot" for t in get(st, "/log").json()["turns"])
    assert post(st, "/chat", {"text": "robot"}).json()["camera"]["calibrated"]


def test_loose_boxes_are_flagged_as_mixed():
    """A box around the board that also takes in the wall-less background / floor: loud MIXED warnings;
    a polygon inside the board: clean."""
    sc = demo_scene()
    dm, _ = sc.render((0, 0, 0))
    cl = G.Cloud(dm, CAM, T_BC, (0, 0, 0))
    u, v = board_px(sc.rects[0])
    loose = G.selection_mask({"box": [u - 260, v - 60, u + 260, v + 160]}, [], CAM)    # spills onto the floor
    tight = G.selection_mask({"polygon": [[u - 150, v - 45], [u + 150, v - 45], [u + 150, v + 45], [u - 150, v + 45]]}, [], CAM)
    r_loose, r_tight = G.region_stats(cl, loose), G.region_stats(cl, tight)
    assert any("MIXED" in w for w in r_loose.get("warnings", [])) and not r_tight.get("warnings")
    # the median centroid is robust, but the nearest point and the extent are not
    # (here the spill is the floor seen under and beyond the board: the far percentile jumps)
    assert r_tight["base"]["x"][2] < 3.05 and r_loose["base"]["x"][2] > 3.3
    assert r_loose["range_spread_m"] > 0.4 > r_tight["range_spread_m"]
    p_loose, p_tight = G.fit_plane(cl, loose), G.fit_plane(cl, tight)
    assert "MIXED" in p_loose.get("warning", "") and "warning" not in p_tight


def _edge_px(pts_world, pose=(0, 0, 0)):
    T = np.linalg.inv(G.pose_matrix(*pose))
    out = []
    for p in pts_world:
        pb = (T @ np.array([*p, 1.0]))[:3]
        out.append([float(x) for x in G.project(CAM, ((pb - T_BC[:3, 3]) @ T_BC[:3, :3])[None])[0]])
    return out


def test_line_fit_from_points_and_from_a_thin_selection():
    sc = demo_scene()
    dm, _ = sc.render((0, 0, 0))
    cl = G.Cloud(dm, CAM, T_BC, (0, 0, 0))
    # four points just above the board's bottom edge (x = 3.0 face, z = 0.78), left to right
    px = _edge_px([(3.0, 0.3 + y, 0.78) for y in (0.8, 0.3, -0.2, -0.7)])
    ln = G.fit_line(cl, points_px=px)
    assert ln["base"]["heading"] == pytest.approx(-math.pi / 2, abs=0.03)        # along -y (left to right)
    assert ln["length_m"] == pytest.approx(1.5, abs=0.05) and ln["tilt_from_horizontal_deg"] < 2
    assert "warning" not in ln and len(ln["per_point_base"]) == 4
    # a thin polygon along the same edge band
    a, b = px[0], px[-1]
    poly = [[a[0], a[1] - 4], [b[0], b[1] - 4], [b[0], b[1] + 4], [a[0], a[1] + 4]]
    ln2 = G.fit_line(cl, G.selection_mask({"polygon": poly}, [], CAM))
    assert abs(math.sin(ln2["base"]["heading"] - ln["base"]["heading"])) < 0.05


def test_turn_in_place_and_coverage_from_earlier_snapshots():
    sc = demo_scene()
    near = G.Cloud(sc.render((0, 0, 0))[0], CAM, T_BC, (0, 0, 0))
    back = G.Cloud(sc.render((-1.0, 0, 0))[0], CAM, T_BC, (-1.0, 0, 0))       # seen from 1 m further back
    turn = G.path_check(near, (0, 0, 0), (0, 0, 1.5))
    assert turn["verified_fraction"] is None and "turn in place" in turn["note"]
    assert not any("UNVERIFIED" in w for w in turn["warnings"])
    alone = G.path_check(near, (0, 0, 0), (1.0, 0.3, 0))
    both = G.path_check(near, (0, 0, 0), (1.0, 0.3, 0), others=[back])
    assert alone["verified_fraction"] < 0.8 and both["verified_fraction"] > 0.9 and both["snapshots_used"] == 2
    assert any("UNVERIFIED" in w for w in alone["warnings"]) and not any("UNVERIFIED" in w for w in both["warnings"])


def test_stale_means_moved_and_enable_adopts_the_pages_hold(stack):
    st = stack
    post(st, "/session", {"agent": "T"})
    assert post(st, "/enable", human=True).status_code == 200          # a human enables from the page
    r = post(st, "/enable", {"caps": {"max_speed": 0.1}}).json()           # the agent adopts it, with its caps
    assert "adopted" in r and r["caps"]["max_speed"] == 0.1
    post(st, "/look")
    time.sleep(1.0)
    g = post(st, "/goal", {"x": 0.3}).json()                               # no motion since the look: not stale
    assert not any("STALE" in w for w in g["warnings"])
    assert get(st, f"/goals/{g['goal_id']}/wait", timeout=30).json()["result"]["result"] == "reached"
    g2 = post(st, "/goal", {"x": 0.1}).json()                              # moved 0.3 m since s1
    assert any("STALE" in w and "moved" in w for w in g2["warnings"])
    get(st, f"/goals/{g2['goal_id']}/wait", timeout=30)
    ln = post(st, "/query/line", {"points": [[400, 120], [500, 125], [600, 130]]})
    assert ln.status_code == 200 and "heading" in ln.json()["base"]
    assert post(st, "/query/line", {"points": [[400, 120]]}).status_code == 422
    assert post(st, "/chat", {"text": "line 400,120 500,125 600,130"}).json()["kind"] == "line"


def test_look_grid_values_and_zoom(stack):
    """look grid {step, values}; zoom crops a held snapshot with full-image grid labels and depth values."""
    st = stack
    post(st, "/session", {"agent": "Tester"})
    look = post(st, "/look", {"grid": {"step": 50, "values": "x"}}).json()
    assert "snapshot" in look, look
    g = look["grid"]
    assert g["step"] == 50 and g["values"] == "x" and g["magnification"] == 1
    hit = [s for s in g["samples"] if s["base"] is not None]
    assert hit and all(len(s["base"]) == 3 for s in hit)
    u, v = board_px(demo_scene().rects[0])
    # zoom on the board: labels in full-image pixels, values = the board's distance (3.0 m)
    b = [u - 40, v - 20, u + 40, v + 20]
    z = post(st, "/zoom", {"snapshot": look["snapshot"], "box": b, "step": 10, "values": "x", "comment": "board"}).json()
    assert z["zoom"] == "z1" and z["snapshot"] == "s1" and z["magnification"] > 4
    assert z["box"] == [int(round(c)) for c in b]
    xs = [s["base"][0] for s in z["samples"] if s["base"] is not None]
    assert xs and all(abs(x - 3.0) < 0.05 for x in xs), xs
    assert all(s["u"] % 10 == 0 and b[0] <= s["u"] <= b[2] for s in z["samples"])
    img = get(st, "/" + z["image"])
    assert img.status_code == 200 and img.headers["content-type"] == "image/jpeg"
    # whole-image zoom with the default step; bad requests
    assert post(st, "/zoom", {}).json()["box"][0] == 0
    assert post(st, "/zoom", {"box": [10, 10, 12, 12]}).status_code == 422
    assert post(st, "/zoom", {"step": 2, "values": "z"}).status_code == 422          # too many samples
    assert post(st, "/zoom", {"snapshot": "s99"}).status_code == 404


def test_plan_check_draws_the_robot_and_counts_depth_in_the_arm(stack):
    """plan/check draws the base (and the arm, from the arm node's geometry) at the goal into the snapshot."""
    st = stack
    nav = st["nav"]
    post(st, "/session", {"agent": "Tester"})
    post(st, "/look")
    # no arm node in the sim: the base alone, said loudly
    r = post(st, "/plan/check", {"frame": "local", "x": 0.5, "y": 0.0, "theta": 0.0}).json()
    rv = r["robot"]
    assert rv["view"].startswith("images/") and rv["goal_in_snapshot_base"][:2] == pytest.approx([0.5, 0.0], abs=0.02)
    assert any("arm node unavailable" in w for w in rv["warnings"])
    assert get(st, "/" + rv["view"]).headers["content-type"] == "image/jpeg"
    # a stand-in arm: one box (the "jaw") reaching 2.5 m ahead of the arm frame, i.e. into the board 3 m ahead
    T = np.eye(4); T[:3, 3] = [0.1159, -0.1459, 0.22225]
    seen = {}

    def fake_geometry(arm):
        seen["arm"] = arm
        return {"which": "ik" if arm else "measured", "q": [0.0] * 6, "gap_mm": 40.0, "ik": {"ok": True, "exact": True} if arm else None,
                "frames": {"arm_in_base": {"matrix": T.tolist(), "xyz": [0.1159, -0.1459, 0.22225], "source": "test"}},
                "boxes": [{"name": "jaw", "link": "finger", "center": {"arm": [2.8, 0.3, 0.6], "base": [0, 0, 0]},
                           "size_m": [0.3, 0.3, 0.3], "axes_base": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}],
                "fingertips": {"left": {"arm": [0.4, 0.02, 0.3]}, "right": {"arm": [0.4, -0.02, 0.3]},
                               "center": {"arm": {"pos": [0.4, 0.0, 0.3]}}},
                "extent_base": {"past_footprint_m": {"front": 0.1, "rear": -0.1, "left": -0.2, "right": 0.0}}}

    nav._arm_geometry = fake_geometry
    r = post(st, "/plan/check", {"frame": "local", "x": 0.0, "y": 0.0, "theta": 0.0,
                                 "arm": {"pose": {"pos": [0.3, 0, 0.4], "rpy": [0, 0, 0]}, "gap_mm": 40}}).json()
    rv = r["robot"]
    assert seen["arm"]["gap_mm"] == 40 and rv["arm"]["which"] == "ik"
    assert rv["fingertips"]["left"]["base"] == pytest.approx([0.5159, -0.1259, 0.52225], abs=1e-3)
    assert rv["depth_points_in_arm"] and rv["depth_points_in_arm"][0]["box"] == "jaw"
    assert any("depth points inside the arm" in w for w in rv["warnings"])
    # a bad arm request is the agent's to fix (422), not a warning
    assert post(st, "/plan/check", {"frame": "local", "arm": {"q": [0, 0, 0]}}).status_code == 422


def test_session_geometry_corrects_perception_not_motion(stack):
    """Session placement corrections move base/world coordinates of held snapshots; they reset per session."""
    st = stack
    post(st, "/session", {"agent": "Tester"})
    g0 = get(st, "/session/geometry").json()
    assert g0["corrections"]["camera"]["xyz"] == [0, 0, 0] and g0["described"]["camera"] == g0["effective"]["camera"]
    assert "reflex" in g0["never_applies_to"]
    look = post(st, "/look").json()
    u, v = board_px(demo_scene().rects[0])
    x0 = post(st, "/query/region", {"select": {"disk": [u, v, 10]}}).json()["base"]["centroid"]
    g = post(st, "/session/geometry", {"camera": {"xyz": [0.02, -0.01, 0.0]}, "comment": "drawn tips 2 cm short"}).json()
    assert g["corrections"]["camera"]["xyz"] == [0.02, -0.01, 0.0]
    assert g["effective"]["camera"]["xyz"][0] == pytest.approx(g["described"]["camera"]["xyz"][0] + 0.02, abs=1e-5)
    x1 = post(st, "/query/region", {"snapshot": look["snapshot"], "select": {"disk": [u, v, 10]}}).json()["base"]["centroid"]
    assert x1[0] - x0[0] == pytest.approx(0.02, abs=2e-3) and x1[1] - x0[1] == pytest.approx(-0.01, abs=2e-3)
    assert any("session camera correction" in w for w in post(st, "/look").json()["warnings"])
    assert post(st, "/session/geometry", {"camera": {"xyz": [0.2, 0, 0]}}).status_code == 422       # beyond the limit
    assert post(st, "/session/geometry", {"arm": {"rpy_deg": [0, 5, 0]}}).status_code == 422
    post(st, "/session/geometry", {"camera": {}})                                                    # back to none
    x2 = post(st, "/query/region", {"snapshot": look["snapshot"], "select": {"disk": [u, v, 10]}}).json()["base"]["centroid"]
    assert x2 == pytest.approx(x0, abs=1e-6)
    post(st, "/session/geometry", {"camera": {"xyz": [0.01, 0, 0]}})
    post(st, "/session", {"agent": "Next"})                                                          # a new session starts clean
    assert get(st, "/session/geometry").json()["corrections"]["camera"]["xyz"] == [0, 0, 0]
