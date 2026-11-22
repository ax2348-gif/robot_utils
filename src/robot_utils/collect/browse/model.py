"""The robot model for the 3D view: the description (frames, base geometry, cameras) and the arm's kinematic chain.

The recorder does not save the model (``session.json`` has no description or model version; docs/browse.md
"Model"), so the description comes from, in order:

1. ``--description`` given: a JSON file (the description node's ``GET /description``), a description node URL
   (``http://botcore/nodes/description``: fetched once), or a description data directory (a sim's
   ``~/.local/share/robot_utils/sim/description``: its overrides and active calibrations);
2. ``<recording>/description.json`` (what a future recorder could save once per session);
3. the package default (``default_description.yaml``, nominal mounts): labelled as such.

The arm chain (``arm_chain``) is what the page needs to draw the arm from recorded joint angles: per joint its
placement in the parent joint frame at q = 0 and its axis, then ``end_link`` and the ``tcp`` in joint 6, from the
arm node's own pinocchio model (``arm.kin.ArmKinematics``, rebot_ctl's URDF). Forward kinematics then runs in the
page (``chain_fk`` here is the same product, for the tests).
"""
from __future__ import annotations

import json
import tempfile
import urllib.request
from pathlib import Path
from typing import Optional

import numpy as np


def load_description(spec: Optional[str], recording_dir: Optional[Path] = None) -> tuple[Optional[dict], str]:
    """(description, where it came from)."""
    if spec:
        if spec.startswith(("http://", "https://")):
            url = spec.rstrip("/")
            if not url.endswith("/description"):
                url += "/description"
            with urllib.request.urlopen(url, timeout=5) as r:
                return json.loads(r.read()), f"fetched from {url}"
        p = Path(spec).expanduser()
        if p.is_file():
            return json.loads(p.read_text()), f"file {p}"
        if p.is_dir():
            return _from_data_dir(p), f"description data dir {p}"
        raise ValueError(f"--description {spec!r}: not a URL, a JSON file or a directory")
    if recording_dir is not None:
        for d in (recording_dir, recording_dir.parent.parent):        # an episode dir: its session's
            f = d / "description.json"
            if f.is_file():
                return json.loads(f.read_text()), f"recorded {f}"
    try:
        return _from_data_dir(Path(tempfile.mkdtemp(prefix="ru-browse-desc-"))), \
            "package default (nominal mounts; not saved with the recording)"
    except Exception as e:  # noqa: BLE001
        return None, f"none ({type(e).__name__}: {e})"


def _from_data_dir(d: Path) -> dict:
    from ...nodes.description.node import DescriptionNode
    return DescriptionNode(str(d), swerve_url=None).description()


def _T(se3) -> list:
    T = np.eye(4)
    T[:3, :3] = np.asarray(se3.rotation)
    T[:3, 3] = np.asarray(se3.translation)
    return np.round(T, 9).tolist()


def arm_chain() -> tuple[Optional[dict], str]:
    """``({joints: [{name, T, axis}], end: T, tcp: T}, how)`` or ``(None, why not)``."""
    try:
        from ...nodes.arm.kin import ArmKinematics
        k = ArmKinematics()
    except Exception as e:  # noqa: BLE001 - rebot_ctl / pinocchio missing
        return None, f"no arm model ({type(e).__name__}: {e}); the arm is drawn as its EE poses only"
    m = k.model
    joints = []
    for j in range(1, 7):
        jm = m.joints[j]
        short = jm.shortname()
        if short.endswith(("RX", "RY", "RZ")):
            axis = {"RX": [1, 0, 0], "RY": [0, 1, 0], "RZ": [0, 0, 1]}[short[-2:]]
        else:
            axis = [float(v) for v in np.asarray(jm.extract().axis)]
        joints.append({"name": m.names[j], "T": _T(m.jointPlacements[j]), "axis": axis,
                       "lo": float(m.lowerPositionLimit[jm.idx_q]), "hi": float(m.upperPositionLimit[jm.idx_q])})
    end = m.frames[k._ee]
    tcp = m.frames[k.tcp]
    return {"joints": joints, "end": _T(end.placement), "tcp": _T(tcp.placement)}, "arm node model (rebot_ctl URDF)"


def _rot(axis, q) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def chain_fk(chain: dict, q) -> list[np.ndarray]:
    """The 6 joint frames, end_link and tcp (4x4, arm base frame) at ``q``: the page's FK, in numpy."""
    T = np.eye(4)
    out = []
    for jd, qi in zip(chain["joints"], q):
        R = np.eye(4)
        R[:3, :3] = _rot(jd["axis"], qi)
        T = T @ np.asarray(jd["T"]) @ R
        out.append(T.copy())
    out.append(T @ np.asarray(chain["end"]))
    out.append(T @ np.asarray(chain["tcp"]))
    return out
