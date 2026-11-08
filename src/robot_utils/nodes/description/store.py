"""On-disk state of the description node.

Layout under ``data_dir`` (default ``~/.local/share/robot_utils/description``):

    description.yaml                        live overrides (e.g. measured nominal camera poses)
    active.json                             {"<camera serial>": {"<type>": "<calibration id>"}}
    calibrations/<type>/<serial>/<id>/      one directory per run:
        meta.json                           inputs, results, quality, camera description, version
        raw.npz                             raw data (compressed)
    aborted/<time>-<type>-job<n>.npz        what an aborted run recorded (newest ``keep``), for diagnosis

Calibrations are kept per (type, camera serial): the newest ``keep`` plus the active one; older
ones are deleted when a new one is saved (FIFO). The active one is never deleted.
"""
from __future__ import annotations

import copy
import json
import os
import shutil
import threading
import time
from importlib import resources
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

DEFAULT_DATA_DIR = os.path.expanduser("~/.local/share/robot_utils/description")
TYPES = ("ground", "yaw")
KEEP = 5


def _merge(a: dict, b: dict) -> dict:
    out = copy.deepcopy(a)
    for k, v in (b or {}).items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else copy.deepcopy(v)
    return out


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    raise TypeError(type(o))


class Store:
    def __init__(self, data_dir: str = DEFAULT_DATA_DIR, keep: int = KEEP):
        self.dir = Path(data_dir)
        self.keep = keep
        self.lock = threading.RLock()
        self.dir.mkdir(parents=True, exist_ok=True)

    # -- description --------------------------------------------------------------------------

    def default(self) -> dict:
        text = resources.files(__package__).joinpath("default_description.yaml").read_text()
        return yaml.safe_load(text)

    def overrides(self) -> dict:
        p = self.dir / "description.yaml"
        return (yaml.safe_load(p.read_text()) or {}) if p.exists() else {}

    def description(self) -> dict:
        return _merge(self.default(), self.overrides())

    def set_override(self, path: list[str], value) -> None:
        with self.lock:
            o = self.overrides()
            d = o
            for k in path[:-1]:
                d = d.setdefault(k, {})
            d[path[-1]] = value
            tmp = self.dir / "description.yaml.tmp"
            tmp.write_text(yaml.safe_dump(o, sort_keys=False))
            os.replace(tmp, self.dir / "description.yaml")

    # -- calibrations -------------------------------------------------------------------------

    def _active_file(self) -> Path:
        return self.dir / "active.json"

    def active(self) -> dict:
        p = self._active_file()
        return json.loads(p.read_text()) if p.exists() else {}

    def _write_active(self, a: dict) -> None:
        tmp = self.dir / "active.json.tmp"
        tmp.write_text(json.dumps(a, indent=1))
        os.replace(tmp, self._active_file())

    def save(self, kind: str, serial: str, meta: dict, raw: dict, activate: bool = True) -> str:
        """Write a calibration run; returns its id. Activates it if asked, then prunes (FIFO)."""
        if kind not in TYPES:
            raise ValueError(f"unknown calibration type {kind!r}")
        with self.lock:
            base = self.dir / "calibrations" / kind / serial
            base.mkdir(parents=True, exist_ok=True)
            cid = time.strftime("%Y%m%d-%H%M%S")
            n = 0
            while (base / (cid if n == 0 else f"{cid}-{n}")).exists():
                n += 1
            cid = cid if n == 0 else f"{cid}-{n}"
            d = base / cid
            tmp = base / f".{cid}.tmp"
            tmp.mkdir()
            meta = {**meta, "id": cid, "type": kind, "serial": serial, "t_saved": time.time()}
            (tmp / "meta.json").write_text(json.dumps(meta, indent=1, default=_json_default))
            if raw:
                np.savez_compressed(tmp / "raw.npz", **{k: np.asarray(v) for k, v in raw.items()})
            os.replace(tmp, d)
            if activate:
                a = self.active()
                a.setdefault(serial, {})[kind] = cid
                self._write_active(a)
            self.prune(kind, serial)
            return cid

    def list(self, kind: Optional[str] = None, serial: Optional[str] = None) -> list[dict]:
        out = []
        act = self.active()
        root = self.dir / "calibrations"
        for k in ([kind] if kind else TYPES):
            kdir = root / k
            if not kdir.is_dir():
                continue
            for sdir in sorted(kdir.iterdir()):
                if serial and sdir.name != serial or not sdir.is_dir():
                    continue
                for cdir in sorted(sdir.iterdir(), reverse=True):
                    if cdir.name.startswith(".") or not (cdir / "meta.json").exists():
                        continue
                    meta = json.loads((cdir / "meta.json").read_text())
                    out.append({"id": cdir.name, "type": k, "serial": sdir.name,
                                "active": act.get(sdir.name, {}).get(k) == cdir.name,
                                "t_saved": meta.get("t_saved"), "summary": meta.get("summary"),
                                "quality": meta.get("quality"),
                                "has_raw": (cdir / "raw.npz").exists()})
        return out

    def get(self, kind: str, serial: str, cid: str) -> dict:
        p = self.dir / "calibrations" / kind / serial / cid / "meta.json"
        if not p.exists():
            raise KeyError(f"no {kind} calibration {cid!r} for camera {serial!r}")
        return json.loads(p.read_text())

    def raw_path(self, kind: str, serial: str, cid: str) -> Path:
        return self.dir / "calibrations" / kind / serial / cid / "raw.npz"

    def save_aborted(self, kind: str, job_id: int, raw: dict) -> Path:
        """Keep an aborted run's recording (newest ``keep`` of them)."""
        d = self.dir / "aborted"
        with self.lock:
            d.mkdir(parents=True, exist_ok=True)
            p = d / f"{time.strftime('%Y%m%d-%H%M%S')}-{kind}-job{job_id}.npz"
            np.savez_compressed(p, **raw)
            for old in sorted(d.glob("*.npz"), key=lambda f: f.stat().st_mtime)[:-self.keep]:
                old.unlink(missing_ok=True)
        return p

    def activate(self, kind: str, serial: str, cid: Optional[str]) -> None:
        """Make ``cid`` the active calibration of its type for this camera (None: none active)."""
        with self.lock:
            if cid is not None:
                self.get(kind, serial, cid)
            a = self.active()
            if cid is None:
                a.get(serial, {}).pop(kind, None)
            else:
                a.setdefault(serial, {})[kind] = cid
            self._write_active(a)

    def prune(self, kind: str, serial: str) -> list[str]:
        with self.lock:
            sdir = self.dir / "calibrations" / kind / serial
            if not sdir.is_dir():
                return []
            act = self.active().get(serial, {}).get(kind)
            ids = sorted((d.name for d in sdir.iterdir() if d.is_dir() and not d.name.startswith(".")), reverse=True)
            removed = [i for i in ids[self.keep:] if i != act]
            for i in removed:
                shutil.rmtree(sdir / i, ignore_errors=True)
            return removed
