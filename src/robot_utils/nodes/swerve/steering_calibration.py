#!/usr/bin/env python3
"""Steering calibration for MAXSwerve modules: a web page served from the robot.

    python -m robot_utils.nodes.swerve.steering_calibration slcan:/dev/ttyACM0
    # then open http://<robot>:8080 from any computer on the network

Conventions (constants.py):
  * An encoder reads 0 when its module is locked in the calibration jig from the kit.
  * Each module's steer_offset turns that into the wheel angle: wheel = encoder - offset,
    0 = straight ahead, counter-clockwise positive.

The page shows each encoder and wheel angle live, and has three actions:
  * Blink: flash one module's LED to find it.
  * Zero in jig: store each module's current position as its encoder zero (saved in the SPARK).
  * Record: save the current readings to a JSON file on the robot.

Nothing here sends a heartbeat or a setpoint, so no motor can move.
Anyone who can reach the port can press the buttons: use a trusted network.

Ported from rev_ctl ``examples/steering_calibration.py`` (commit cfcfc32).
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

from sparkcan import PeriodicFrame, PersistMode, ResetMode, REVLibError, SparkFlex, SparkFlexConfig
from sparkcan._params import SparkParameter

from .constants import MODULES

STEERING_IDS = {m.name: m.steer_id for m in MODULES}
ROTATION_OFFSETS = {m.name: m.steer_offset for m in MODULES}
FRAMES = 5              # encoder frames averaged per measurement
FRAME_TIMEOUT = 0.5     # s; the encoder frames (STATUS_5/6) come every 200 ms (firmware 26 default)
MAX_SPREAD = 0.002      # rotations (0.7°): more than this means the wheel is moving


class CalibrationError(Exception):
    pass


def wrap(x: float) -> float:
    """A difference in rotations, folded into [-0.5, 0.5)."""
    return (x + 0.5) % 1.0 - 0.5


def circular_mean(xs: list) -> float:
    """Average of positions on a 0-1 circle (0.99 and 0.01 average to 0, not 0.5)."""
    s = sum(math.sin(2 * math.pi * x) for x in xs)
    c = sum(math.cos(2 * math.pi * x) for x in xs)
    return (math.atan2(s, c) / (2 * math.pi)) % 1.0


class Steering:
    """The steering SPARKs, read through their absolute (Through Bore) encoders."""

    def __init__(self, interface: str, can_ids: dict):
        self.sparks = {name: SparkFlex(interface, can_id) for name, can_id in can_ids.items()}

    def close(self) -> None:
        for spark in self.sparks.values():
            spark.close()

    def _latest(self, name: str) -> tuple:
        """(receive time, position) of the newest encoder frame; position None = no signal.

        Receive time is None until both STATUS_5 (position) and STATUS_6 (signal) have arrived.
        Reading them also enables them (they are off by default; not saved).
        """
        spark = self.sparks[name]
        position = spark.get_periodic_status(PeriodicFrame.STATUS_5)
        signal = spark.get_periodic_status(PeriodicFrame.STATUS_6)
        if position.timestamp is None or signal.timestamp is None:
            return None, None
        if signal["DUTY_CYCLE_NO_SIGNAL"]:
            return position.timestamp, None
        return position.timestamp, position["DUTY_CYCLE_ENCODER_POSITION"]

    def read(self, name: str):
        """Latest encoder position in rotations, or None if missing, stale or unplugged."""
        received, x = self._latest(name)
        if received is None or time.monotonic() - received > FRAME_TIMEOUT:
            return None
        return x

    def measure(self, name: str) -> float:
        """Average of FRAMES new encoder frames; raises if the encoder is missing or the wheel moves.

        Only frames received after this call starts are used, and the first of those is skipped:
        it may have been computed before a setting written just before took effect.
        """
        readings, last = [], time.monotonic()
        deadline = last + (FRAMES + 1) * FRAME_TIMEOUT
        while len(readings) < FRAMES + 1:
            if time.monotonic() > deadline:
                raise CalibrationError(f"{name}: no encoder data")
            received, x = self._latest(name)
            if received is not None and received > last:
                if x is None:
                    raise CalibrationError(f"{name}: no encoder signal")
                readings.append(x)
                last = received
            time.sleep(0.01)
        readings = readings[1:]
        mean = circular_mean(readings)
        if max(abs(wrap(x - mean)) for x in readings) > MAX_SPREAD:
            raise CalibrationError(f"{name}: reading is not steady; is the wheel moving?")
        return mean

    def zero_offset(self, name: str) -> float:
        value = self.sparks[name].get_parameter(SparkParameter.DUTY_CYCLE_OFFSET)
        if value is None:
            raise CalibrationError(f"{name}: no answer reading the zero offset")
        return value

    def _write_zero_offset(self, name: str, offset: float, persist: bool) -> None:
        config = SparkFlexConfig()
        config.absolute_encoder.zero_offset(offset)
        mode = PersistMode.PERSIST_PARAMETERS if persist else PersistMode.NO_PERSIST_PARAMETERS
        err = self.sparks[name].configure(config, ResetMode.NO_RESET_SAFE_PARAMETERS, mode)
        if err != REVLibError.OK:
            raise CalibrationError(f"{name}: writing the zero offset failed ({err})")

    def zero(self, name: str) -> float:
        """Make the current position read 0, check it does, then save it. Returns the new offset.

        REVLib defines the offset as the encoder's position at zero "as if the zero offset was
        set to 0 and inverted was set to false", so the SPARK reports raw - offset, negated when
        inverted. If the check fails, the old offset is put back.
        """
        old = self.zero_offset(name)
        inverted = self.sparks[name].get_parameter(SparkParameter.DUTY_CYCLE_INVERTED)
        if inverted is None:
            raise CalibrationError(f"{name}: no answer reading the inversion setting")
        reading = self.measure(name)
        new = ((old - reading) if inverted else (old + reading)) % 1.0
        self._write_zero_offset(name, new, persist=False)
        try:
            after = self.measure(name)
        except CalibrationError:
            after = None
        if after is None or abs(wrap(after)) > MAX_SPREAD:
            self._write_zero_offset(name, old, persist=False)
            raise CalibrationError(f"{name}: did not read 0 after the change; old offset restored")
        self._write_zero_offset(name, new, persist=True)
        return new

    def blink(self, name: str) -> None:
        self.sparks[name].identify()


def wheel_angle_deg(name: str, encoder: float) -> float:
    return wrap(encoder - ROTATION_OFFSETS[name]) * 360


class Handler(BaseHTTPRequestHandler):
    steering: Steering   # set in main()
    out_file: str

    def do_GET(self):
        if self.path == "/":
            self.reply(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/state":
            state = {}
            for name in self.steering.sparks:
                x = self.steering.read(name)
                state[name] = {"can_id": self.steering.sparks[name].get_device_id(), "encoder": x,
                               "wheel_deg": None if x is None else wheel_angle_deg(name, x)}
            self.reply_json(state)
        else:
            self.reply(404, "not found", "text/plain")

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        try:
            if self.path == "/blink":
                self.steering.blink(body["name"])
                message = f"{body['name']} is blinking"
            elif self.path == "/zero" and body.get("confirm") == "ZERO":
                message = "\n".join(f"{name}: zero offset {self.steering.zero(name):.5f}, saved"
                                    for name in self.steering.sparks)
            elif self.path == "/record":
                record = {}
                for name in self.steering.sparks:
                    x = self.steering.measure(name)
                    record[name] = {"can_id": self.steering.sparks[name].get_device_id(),
                                    "encoder": round(x, 5), "wheel_deg": round(wheel_angle_deg(name, x), 2)}
                with open(self.out_file, "w") as f:
                    json.dump(record, f, indent=2)
                message = f"saved {self.out_file}\n" + json.dumps(record, indent=2)
            else:
                return self.reply(400, "unknown or unconfirmed action", "text/plain")
        except CalibrationError as e:
            message = f"FAILED: {e}"
        print(message, flush=True)
        self.reply_json({"message": message})

    def reply_json(self, obj) -> None:
        self.reply(200, json.dumps(obj), "application/json")

    def reply(self, code: int, text: str, content_type: str) -> None:
        data = text.encode()
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):  # no per-request logging
        pass


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Steering Calibration</title>
<style>
  :root { color-scheme: light dark; font-family: system-ui, sans-serif; }
  body { max-width: 880px; margin: 0 auto; padding: 16px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }
  .card { border: 1px solid #8884; border-radius: 10px; padding: 12px; }
  .angle { font: 600 28px ui-monospace, monospace; margin: 6px 0; }
  .muted { opacity: .7; font-size: 13px; }
  button { font: inherit; padding: 6px 12px; margin: 8px 8px 0 0; cursor: pointer; }
  pre { border: 1px solid #8884; border-radius: 10px; padding: 12px; white-space: pre-wrap; }
</style></head>
<body>
<h1>Steering calibration</h1>
<div class="grid" id="modules"></div>
<p>
  <button onclick="zero()">Zero in jig…</button>
  <button onclick="act('/record')">Record positions</button>
</p>
<ol class="muted">
  <li>Lock every module in its jig, then press <b>Zero in jig</b>: each encoder then reads 0 there.</li>
  <li>Remove the jigs. Turn a wheel by hand to 0°: it should point straight ahead.</li>
  <li><b>Record positions</b> saves the readings to a file on the robot.</li>
</ol>
<pre id="log">Nothing here can make a motor move.</pre>
<script>
async function refresh() {
  const state = await (await fetch("/state")).json();
  document.getElementById("modules").innerHTML = Object.entries(state).map(([name, m]) => `
    <div class="card"><b>${name}</b> <span class="muted">CAN ID ${m.can_id}</span>
      <div class="angle">${m.wheel_deg === null ? "no signal" : m.wheel_deg.toFixed(1) + "°"}</div>
      <div class="muted">encoder ${m.encoder === null ? "-" : m.encoder.toFixed(4)} rot</div>
      <button onclick="act('/blink', {name: '${name}'})">Blink</button></div>`).join("");
}
async function act(path, body = {}) {
  document.getElementById("log").textContent = "working...";
  const r = await fetch(path, {method: "POST", body: JSON.stringify(body)});
  document.getElementById("log").textContent = r.ok ? (await r.json()).message : await r.text();
}
function zero() {
  if (prompt("All modules locked in their jigs? This saves a new zero in each SPARK. Type ZERO:") === "ZERO")
    act("/zero", {confirm: "ZERO"});
}
setInterval(() => refresh().catch(() => {}), 250);
</script>
</body></html>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description="MAXSwerve steering calibration page")
    ap.add_argument("interface", help='e.g. can0 or "slcan:/dev/ttyACM0"')
    ap.add_argument("--ids", help="steering CAN IDs as FL,FR,RR,RL (default 5,8,7,6)")
    ap.add_argument("--out", default="steering_positions.json", help="file written by Record")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    can_ids = STEERING_IDS
    if args.ids:
        ids = [int(x) for x in args.ids.split(",")]
        if len(ids) != len(STEERING_IDS):
            ap.error("--ids needs four CAN IDs: FL,FR,RR,RL")
        can_ids = dict(zip(STEERING_IDS, ids))
    Handler.steering = Steering(args.interface, can_ids)
    Handler.out_file = args.out
    # One request at a time: an action (zeroing, recording) never overlaps another.
    server = HTTPServer(("0.0.0.0", args.port), Handler)
    print(f"open http://{socket.gethostname()}:{args.port}  (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        Handler.steering.close()


if __name__ == "__main__":
    main()
