"""``WorldCameraBackend``: the orbbec node's camera backend on the sim world (docs/sim.md, "Camera backend").

    Owner(WorldCameraBackend("sim", "camera"), hub, Settings(), ...)

Same interface as the orbbec ``SdkBackend`` / ``FakeBackend``. The catalog has one colour profile (MJPG)
and one depth mode from the world's camera config, depth only (no IR, no dual colour mode) and no IMU (not
simulated yet). Frames come from the world's renderer (``camera-<name>.sock``: ``info``, ``start``,
``stop``, ``frames``): colour as the camera sends it (JPEG = MJPG), depth as Y16 millimetres, each with
``t_sensor`` (the rendered state's host time) and ``t_capture`` = ``t_sensor`` + the configured
camera-to-host delay; ``wait`` hands a frame over no earlier than its ``t_capture``.
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np

from ...orbbec.hub import Frame
from ...orbbec.models import Catalog, DeviceInfo, ImuCatalog, ProfileOption, Settings
from ...orbbec.owner import NoDevice
from ..protocol import IpcClient, IpcError, backend_socket


class WorldCameraBackend:
    def __init__(self, sim: str = "sim", camera: str = "camera", socket_path=None):
        self.camera = camera
        self.client = IpcClient(socket_path or backend_socket(sim, f"camera-{camera}"), timeout=3.0)
        self.modes = ["depth"]
        self.mode = "depth"
        self.is_open = False
        self.running = False
        self.info: Optional[dict] = None
        self._streams: list = []
        self._seq: dict = {}
        self._held: list = []          # frames received before their t_capture
        self._index = 0

    def _keys(self) -> tuple[str, str]:
        c, d = self.info["color"], self.info["depth"]
        return (f"{c['width']}x{c['height']}@{int(c['fps'])} MJPG", f"{d['width']}x{d['height']}@{int(d['fps'])}")

    def open(self, mode: str = "depth"):
        try:
            self.info = self.client.call("info")[0]
        except IpcError as e:
            raise NoDevice(f"sim world camera {self.camera!r} not reachable: {e}") from None
        self.is_open = True
        return DeviceInfo(name=f"Sim camera ({self.camera})", serial=self.info["serial"], firmware="sim",
                          connection="sim", global_timestamp=True), self._catalog()

    def _catalog(self) -> Catalog:
        ck, dk = self._keys()
        c, d = self.info["color"], self.info["depth"]
        return Catalog(modes=["depth"], mode="depth", available=["color", "depth"],
                       color=[ProfileOption(key=ck, width=c["width"], height=c["height"], fps=int(c["fps"]),
                                            format="MJPG", default=True)],
                       stereo=[ProfileOption(key=dk, width=d["width"], height=d["height"], fps=int(d["fps"]),
                                             default=True)],
                       imu=ImuCatalog(available=False))

    def set_mode(self, mode: str) -> Catalog:
        if mode != "depth":
            raise ValueError(f"this device has no {mode!r} mode")
        return self._catalog()

    def start(self, s: Settings):
        self.stop()
        ck, dk = self._keys()
        if s.streams.get("color") and s.color_profile not in (None, ck):
            raise RuntimeError(f"the sim camera has only {ck}")
        if s.streams.get("depth") and s.stereo_mode not in (None, dk):
            raise RuntimeError(f"the sim camera has only {dk}")
        self._streams = [n for n in ("color", "depth") if s.streams.get(n)]
        self.client.call("start", streams=self._streams)
        self._seq = {}
        self._held = []
        self.running = bool(self._streams)

    def calibration(self, s):
        out = {k: dict(v) for k, v in self.info["intrinsics"].items()}
        out["depth_to_color"] = self.info["depth_to_color"]
        return out

    def stop(self):
        if self.running:
            try:
                self.client.call("stop")
            except IpcError:
                pass
        self.running = False

    def wait(self, timeout_ms):
        deadline = time.monotonic() + timeout_ms / 1000
        if not self._held:
            try:
                reply, data = self.client.call("frames", after=self._seq, wait=timeout_ms / 1000,
                                               timeout=timeout_ms / 1000 + 2.0)
            except IpcError as e:
                raise RuntimeError(f"sim world: {e}") from None
            off = 0
            for m in reply["frames"]:
                blob = data[off:off + m["len"]]
                off += m["len"]
                self._seq[m["stream"]] = m["seq"]
                self._held.append((m, blob))
        if not self._held:
            return None
        due = min(m["t_capture"] for m, _ in self._held)
        wait = due - time.monotonic()
        if wait > 0:
            if time.monotonic() + wait > deadline + 0.1:
                time.sleep(max(0.0, deadline - time.monotonic()))
                return None
            time.sleep(wait)
        now = time.monotonic()
        ready = [(m, b) for m, b in self._held if m["t_capture"] <= now + 1e-4]
        self._held = [(m, b) for m, b in self._held if m["t_capture"] > now + 1e-4]
        out = []
        for m, blob in ready:
            self._index += 1
            if m["stream"] == "depth":
                data = np.frombuffer(blob, "<u2").reshape(m["height"], m["width"]).copy()
                fr = Frame("depth", "Y16", m["width"], m["height"], data, m["t_capture"], m["t_sensor"],
                           int(m["t_sensor"] * 1e6), self._index, depth_scale=float(m.get("depth_scale", 1.0)))
            else:
                fr = Frame(m["stream"], m["format"], m["width"], m["height"], blob, m["t_capture"], m["t_sensor"],
                           int(m["t_sensor"] * 1e6), self._index)
            fr.t_owner = time.monotonic()
            out.append(fr)
        return out or None

    def close(self):
        self.stop()
        self.is_open = False
        self.client.close()

    def properties(self, all_props=False):
        return []

    def set_property(self, pid, value):
        raise ValueError(f"the sim camera has no property {pid!r}")

    @property
    def imu_running(self):
        return False

    def imu_start(self, s, sink, on_error, batch_ms):
        on_error("the sim camera has no IMU (not simulated yet)")

    def imu_stop(self):
        pass
