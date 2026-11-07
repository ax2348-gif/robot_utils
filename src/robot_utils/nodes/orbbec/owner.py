"""The hardware-owner thread: the only code that drives the camera backend (and so the SDK).

It opens the device (retrying while there is none), starts the video pipeline, waits for
framesets, stamps and publishes frames to the Hub, and executes commands (settings changes,
property reads/writes) that the API hands it through a queue with a concurrent.futures.Future
per command. The IMU runs on its own SDK pipeline in a second owner-side thread started and
stopped from here (see the backends).
"""
from __future__ import annotations

import concurrent.futures as cf
import logging
import os
import queue
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from .encode import colormap_stops
from .hub import Frame, Hub
from .models import (STREAM_LABELS, STREAM_NAMES, Catalog, DeviceInfo, Settings, SettingsError,
                     SettingsPatch, apply_patch, describe, fix_settings)

log = logging.getLogger("orbbec_node")

STALL_S = 4.0         # no frames for this long while streaming -> treat the camera as lost
STARTUP_S = 8.0       # allowance for the first frame after a (re)start
RETRY_S = 2.0         # between attempts to (re)open the camera
WAIT_MS = 50          # wait_for_frames timeout (bounds command latency while streaming)


class NoDevice(Exception):
    """No camera connected (already reported)."""


class DeviceError(Exception):
    """Property / device access failed or no device (HTTP 503 / 500)."""


class _Rate:
    def __init__(self, window=3.0):
        self.t = deque()
        self.n = deque()
        self.window = window

    def add(self, t, n=1):
        self.t.append(t)
        self.n.append(n)
        while self.t and t - self.t[0] > self.window:
            self.t.popleft()
            self.n.popleft()

    def rate(self, now):
        if len(self.t) < 2 or now - self.t[-1] > 2.0:
            return 0.0
        return (sum(self.n) - self.n[0]) / max(self.t[-1] - self.t[0], 1e-6)


class Owner:
    def __init__(self, backend, hub: Hub, settings: Optional[Settings] = None, startup_s: float = STARTUP_S):
        self.backend = backend
        self.startup_s = startup_s
        self.hub = hub
        self.settings = settings or Settings()
        self.settings_version = 0
        self.catalog = Catalog(colormaps=colormap_stops())
        self.info: Optional[DeviceInfo] = None
        self.lock = threading.RLock()
        self.cmds: queue.Queue = queue.Queue()
        self.stopping = threading.Event()
        self.state, self.error = "starting", None
        self.events: deque = deque(maxlen=20)
        self.seq = {n: 0 for n in STREAM_NAMES}
        self.frameset = 0
        self.rates = {n: _Rate() for n in STREAM_NAMES}
        self.shape = {n: (0, 0, "") for n in STREAM_NAMES}
        self.last_capture = {n: None for n in STREAM_NAMES}
        self.last_frame_t = 0.0
        self.last_pub = {n: 0.0 for n in STREAM_NAMES}
        self.video_started = 0.0
        self._silent: set = set()
        self._silent_check = 0.0
        self.imu = {"running": False, "samples": 0, "gaps": 0, "error": None, "last": None}
        self.imu_seq = 0
        self.imu_rate = _Rate()
        self._cpu = (time.monotonic(), sum(os.times()[:2]), 0.0)
        self.thread = threading.Thread(target=self._run, name="orbbec-owner", daemon=True)

    # -- lifecycle ---------------------------------------------------------------------------

    def start(self):
        self.thread.start()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=8)

    # -- API side (any thread; never touches the backend) --------------------------------------

    def submit(self, kind: str, arg=None) -> cf.Future:
        fut: cf.Future = cf.Future()
        if self.stopping.is_set():
            fut.set_exception(DeviceError("node is stopping"))
        else:
            self.cmds.put((kind, arg, fut))
        return fut

    def request_settings(self, patch: SettingsPatch) -> cf.Future:
        """Display-only patches (and IMU batch size) apply at once; anything that needs the
        device goes to the owner thread. Raises SettingsError for invalid patches."""
        p = patch.model_dump(exclude_unset=True)
        imu = p.get("imu") or {}
        if set(p) <= {"display", "imu"} and set(imu) <= {"batch_ms"}:
            with self.lock:
                new, _, _ = apply_patch(self.settings, patch, self.catalog)
                self._set_settings(new)
                res = self._result(False)
            fut: cf.Future = cf.Future()
            fut.set_result(res)
            return fut
        with self.lock:   # validate early for a quick 400; the owner re-validates when applying
            apply_patch(self.settings, patch, self.catalog)
        return self.submit("settings", patch)

    def _set_settings(self, new: Settings):
        self.settings = new
        self.settings_version += 1
        self.hub.state_changed()

    def _result(self, restarted, t_applied=None):
        return {"ok": True, "settings": self.settings, "settings_version": self.settings_version,
                "restarted": restarted, "t_applied": time.monotonic() if t_applied is None else t_applied}

    # -- status ------------------------------------------------------------------------------

    def _event(self, msg, error=False):
        (log.warning if error else log.info)(msg)
        with self.lock:
            self.events.append({"t": time.monotonic(), "wall": time.time(), "msg": msg, "error": error})
            if error:
                self.error = msg
        self.hub.state_changed()

    def _set_state(self, state, clear_error=False):
        with self.lock:
            changed = state != self.state or (clear_error and self.error)
            self.state = state
            if clear_error:
                self.error = None
        if changed:
            self.hub.state_changed()

    def cpu(self):
        now, total = time.monotonic(), sum(os.times()[:2])
        t0, c0, pct = self._cpu
        if now - t0 >= 1.0:
            pct = 100.0 * (total - c0) / (now - t0)
            self._cpu = (now, total, pct)
        return {"process_pct": round(pct, 1), "load1": round(os.getloadavg()[0], 2), "cores": os.cpu_count() or 1}

    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            s = self.settings
            st = {"state": self.state, "error": self.error, "events": list(self.events),
                  "device": self.info, "settings": s, "settings_version": self.settings_version,
                  "catalog_id": self.catalog.catalog_id}
            streams = {}
            for n in STREAM_NAMES:
                w, h, fmt = self.shape[n]
                tc = self.last_capture[n]
                streams[n] = {"label": STREAM_LABELS[n], "available": n in self.catalog.available,
                              "enabled": bool(s.streams.get(n)), "fps": round(self.rates[n].rate(now), 2),
                              "width": w, "height": h, "format": fmt, "seq": self.seq[n],
                              "age_ms": round((now - tc) * 1000, 1) if tc is not None else None}
            st["streams"] = streams
            st["imu"] = {"available": self.catalog.imu.available, "enabled": s.imu.enabled,
                         "running": self.imu["running"], "rate_hz": round(self.imu_rate.rate(now), 1),
                         "samples": self.imu["samples"], "gaps": self.imu["gaps"], "last": self.imu["last"],
                         "error": self.imu["error"]}
        st.update(clients=self.hub.clients, cpu=self.cpu(), server_mono=now, server_wall=time.time())
        return st

    # -- owner thread ------------------------------------------------------------------------

    def _run(self):
        while not self.stopping.is_set():
            try:
                if not self.backend.is_open:
                    self._open()
                self._serve()
            except NoDevice:
                pass
            except Exception as e:  # camera lost / SDK error: report, drop everything, retry
                if not self.stopping.is_set():
                    log.debug("owner error", exc_info=True)
                    self._event(f"camera error: {e}", error=True)
                    self._set_state("reconnecting")
            self._teardown()
            deadline = time.monotonic() + RETRY_S
            while not self.stopping.is_set() and time.monotonic() < deadline:
                self._handle_cmds(block=0.2)
        self._teardown()
        self._set_state("stopped")
        self._fail_pending("node stopped")

    def _open(self):
        try:
            info, cat = self.backend.open()
        except NoDevice:
            with self.lock:
                if self.state != "no device":
                    self._event("no camera found; waiting for one to be plugged in", error=True)
                self.state = "no device"
            raise
        cat.colormaps = colormap_stops()
        with self.lock:
            cat.catalog_id = self.catalog.catalog_id + 1
            self.catalog = cat
            self.info = info
            self._set_settings(fix_settings(self.settings, cat))
            want = self.settings
        self._event(f"opened {info.name} ({info.serial}, fw {info.firmware}, {info.connection})")
        self._start_video(want, prev=None)
        self._start_imu(want)

    def _teardown(self):
        try:
            self.backend.close()
        except Exception as e:
            log.debug("backend close: %s", e)
        self.hub.drop_frames()
        with self.lock:
            self.imu["running"] = False

    def _serve(self):
        while not self.stopping.is_set():
            self._handle_cmds(block=0 if self.backend.running else 0.5)
            if not self.backend.running:
                continue
            frames = self.backend.wait(WAIT_MS)
            now = time.monotonic()
            if not frames:
                if now - self.last_frame_t > STALL_S:
                    raise RuntimeError(f"no frames for {STALL_S:.0f} s (unplugged or USB error?)")
                continue
            self.last_frame_t = now
            self._publish(frames)
            if now - self._silent_check > 1.0:
                self._check_silent(now)

    def _check_silent(self, now):
        """Report enabled streams that stopped delivering while others still do."""
        self._silent_check = now
        s = self.settings
        for n in STREAM_NAMES:
            last = max(self.last_pub[n], self.video_started)
            silent = bool(s.streams.get(n)) and now - last > STALL_S
            if silent and n not in self._silent:
                self._silent.add(n)
                self._event(f"{STREAM_LABELS[n]} delivers no frames (for {STALL_S:.0f} s)", error=True)
            elif not silent and n in self._silent:
                self._silent.discard(n)
                self._event(f"{STREAM_LABELS[n]} frames again")

    def _publish(self, frames: list[Frame]):
        now = time.monotonic()
        self.frameset += 1
        with self.lock:
            for f in frames:
                self.seq[f.stream] += 1
                f.seq = self.seq[f.stream]
                f.frameset = self.frameset
                f.t_publish = now
                self.rates[f.stream].add(now)
                self.shape[f.stream] = (f.w, f.h, f.fmt)
                self.last_capture[f.stream] = f.t_capture
                self.last_pub[f.stream] = now
            if self.state != "streaming":
                self.state = "streaming"
                self.hub.state_changed()
        self.hub.publish_frames(frames)

    def _imu_sink(self, rows: np.ndarray, gaps: int = 0):
        """Called by the IMU thread with (n, len(IMU_FIELDS)) rows; column 0 (seq) is set here."""
        n = len(rows)
        rows[:, 0] = np.arange(self.imu_seq + 1, self.imu_seq + 1 + n)
        self.imu_seq += n
        now = time.monotonic()
        with self.lock:
            self.imu["samples"] += n
            self.imu["gaps"] += gaps
            self.imu_rate.add(now, n)
            r = rows[-1]
            self.imu["last"] = {"t_capture": float(r[1]), "accel": [round(float(x), 4) for x in r[4:7]],
                                "gyro": [round(float(x), 5) for x in r[7:10]], "temperature": round(float(r[10]), 2)}
        self.hub.publish_imu(rows)

    def _imu_error(self, msg):
        with self.lock:
            self.imu["error"] = msg
            self.imu["running"] = False
        self._event(f"IMU: {msg}", error=True)

    def _start_imu(self, s: Settings):
        self.backend.imu_stop()
        with self.lock:
            self.imu["running"] = False
            self.imu["error"] = None
        if not (s.imu.enabled and self.catalog.imu.available):
            return
        try:
            self.backend.imu_start(s.imu, self._imu_sink, self._imu_error,
                                   lambda: self.settings.imu.batch_ms)
            with self.lock:
                self.imu["running"] = True
            self._event(f"IMU running at {s.imu.sample_rate} (accel {s.imu.accel_range}, gyro {s.imu.gyro_range})")
        except Exception as e:
            self._imu_error(f"could not start: {e}")

    def _start_video(self, new: Settings, prev: Optional[Settings]) -> float:
        """(Re)start the video pipeline. For a change (prev given), wait until every enabled
        stream delivered a frame; if the device refuses the config or a stream stays silent
        (e.g. uncompressed color on USB 2), restore prev and raise SettingsError.
        Returns the server monotonic time the new pipeline was started."""
        self.hub.drop_frames()
        self._set_state("starting" if any(new.streams.values()) else "idle")
        try:
            self.backend.start(new)
            t_started = self.video_started = time.monotonic()
            self._silent.clear()
            self.last_frame_t = t_started + self.startup_s - STALL_S
            if prev is not None and self.backend.running:
                want = {n for n, on in new.streams.items() if on}
                seen: set = set()
                while seen != want and time.monotonic() - t_started < self.startup_s and not self.stopping.is_set():
                    frames = self.backend.wait(WAIT_MS)
                    if frames:
                        self.last_frame_t = time.monotonic()
                        self._publish(frames)
                        seen |= {f.stream for f in frames}
                if seen != want and not self.stopping.is_set():
                    raise RuntimeError(f"no frames from {', '.join(sorted(want - seen))} within {self.startup_s:.0f} s")
        except Exception as e:
            if prev is None or self.stopping.is_set():
                raise
            msg = f"could not start {describe(new)}: {e}; restored the previous settings"
            self._event(msg, error=True)
            self.hub.drop_frames()
            self.backend.start(prev)          # failure here -> camera error -> reconnect loop
            self.video_started = time.monotonic()
            self.last_frame_t = time.monotonic() + self.startup_s - STALL_S
            self._set_state("starting" if self.backend.running else "idle")
            raise SettingsError(msg) from None
        with self.lock:
            self.error = None
        self._event(("started " + describe(new)) if self.backend.running else "all video streams off")
        if not self.backend.running:
            self._set_state("idle")
        return t_started

    def _handle_cmds(self, block: float = 0):
        while True:
            try:
                kind, arg, fut = self.cmds.get(timeout=block) if block else self.cmds.get_nowait()
            except queue.Empty:
                return
            block = 0
            if not fut.set_running_or_notify_cancel():
                continue
            try:
                fut.set_result(self._command(kind, arg))
            except BaseException as e:   # noqa: BLE001 - handed back to the API
                fut.set_exception(e)

    def _command(self, kind, arg):
        if kind == "settings":
            with self.lock:
                new, restart, restart_imu = apply_patch(self.settings, arg, self.catalog)
                prev = self.settings
            t_applied = None
            if restart and self.backend.is_open:
                t_applied = self._start_video(new, prev)   # raises SettingsError after restoring prev
            with self.lock:
                self._set_settings(new)
            if restart_imu and self.backend.is_open:
                self._start_imu(new)
            return self._result(t_applied is not None, t_applied)
        if not self.backend.is_open:
            raise DeviceError("camera not connected")
        if kind == "properties":
            return self.backend.properties(all_props=bool(arg))
        if kind == "set_property":
            pid, value = arg
            self.backend.set_property(pid, value)
            self._event(f"set {pid} = {value}")
            return self.backend.properties()
        raise ValueError(f"unknown command {kind}")

    def _fail_pending(self, msg):
        while True:
            try:
                _, _, fut = self.cmds.get_nowait()
            except queue.Empty:
                return
            if fut.set_running_or_notify_cancel():
                fut.set_exception(DeviceError(msg))
