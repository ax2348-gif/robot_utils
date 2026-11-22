"""The robot's description snapshot for the sim (nodes/sim/snapshot.py; docs/sim.md "The robot's calibrations in the
sim"): only description.yaml, active.json and the active calibrations are copied; the sim cameras take the robot's
serials, so the description node applies the active calibrations (mounts calibrated); without a snapshot they stay
nominal."""
import numpy as np

from robot_utils.nodes.description.node import DescriptionNode
from robot_utils.nodes.sim import snapshot
from robot_utils.nodes.sim.node import apply_camera_serials
from robot_utils.nodes.sim.world import load_config

SERIAL = "CPC64630004G"
LOCAL = ["sh", "-c"]                 # "ssh" to this machine: run the remote command locally


def robot_store(path):
    n = DescriptionNode(str(path), None)
    nrm = np.array([0.05, -0.9, -0.3])
    nrm /= np.linalg.norm(nrm)
    old = n.store.save("ground", SERIAL, {"result": {"normal": nrm.tolist(), "height": 0.5}, "summary": "old"}, {})
    active = n.store.save("ground", SERIAL, {"result": {"normal": nrm.tolist(), "height": 0.48, "rms": 0.001,
                                                        "inliers": 5000}, "summary": "active"}, {"x": [1, 2]})
    n.store.set_override(["cameras", "camera", "last_serial"], SERIAL)
    return old, active


def test_pull_copies_the_active_state_and_the_sim_takes_the_robot_serial(tmp_path):
    old, active = robot_store(tmp_path / "robot")
    dest = tmp_path / "sim" / "description"
    info = snapshot.pull("robot", str(dest), command=LOCAL, remote_dir=str(tmp_path / "robot"))
    assert info["active"] == {SERIAL: {"ground": active}} and info["last_serials"] == {"camera": SERIAL}
    assert (dest / "calibrations" / "ground" / SERIAL / active / "raw.npz").exists()
    assert not (dest / "calibrations" / "ground" / SERIAL / old).exists()           # active ones only
    assert active in snapshot.describe(str(dest)) and SERIAL in snapshot.describe(str(dest))
    # the sim's description node, on the snapshot: the camera's mount is calibrated
    n = DescriptionNode(str(dest), None)
    d = n.description()
    assert d["cameras"]["camera"]["serial"] == SERIAL and d["cameras"]["camera"]["from"]["ground"] == active
    assert n.frames()["camera/depth_optical"]["source"]["ground"] == active
    # the sim cameras report the robot's serial (the wrist camera has none: its sim serial)
    cfg = load_config()
    serials = apply_camera_serials(cfg, d)
    assert serials == {"camera": SERIAL, "wrist_camera": "SIMWRIST0001"} and cfg["cameras"]["camera"]["serial"] == SERIAL
    # a second pull replaces the snapshot (the sim's own writes do not survive it)
    (dest / "stray.txt").write_text("x")
    snapshot.pull("robot", str(dest), command=LOCAL, remote_dir=str(tmp_path / "robot"))
    assert not (dest / "stray.txt").exists() and (dest / "active.json").exists()


def test_no_snapshot_means_nominal_mounts_and_sim_serials(tmp_path):
    dest = tmp_path / "sim" / "description"
    assert "NONE" in snapshot.describe(str(dest)) and "NOMINAL" in snapshot.describe(str(dest))
    n = DescriptionNode(str(dest), None)
    d = n.description()
    assert d["cameras"]["camera"]["from"] == {"nominal": True}
    assert n.frames()["camera/depth_optical"]["source"] == {"nominal": True}
    cfg = load_config()
    assert apply_camera_serials(cfg, d) == {"camera": "SIMHEAD0001", "wrist_camera": "SIMWRIST0001"}


def test_unreachable_robot_keeps_the_previous_snapshot(tmp_path):
    robot_store(tmp_path / "robot")
    dest = tmp_path / "sim" / "description"
    snapshot.pull("robot", str(dest), command=LOCAL, remote_dir=str(tmp_path / "robot"))
    before = snapshot.info(str(dest))
    try:
        snapshot.pull("robot", str(dest), command=["sh", "-c", "exit 255", "x"], timeout=2)
        raise AssertionError("expected SnapshotError")
    except snapshot.SnapshotError:
        pass
    assert snapshot.info(str(dest)) == before
