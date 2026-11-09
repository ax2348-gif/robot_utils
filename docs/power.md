# Power node

`robot_utils.nodes.power` manages the robot's power. Today:
- it reads the Pi's **UPS HAT (E)**: battery, cells, input, charger;
- it shows the **main supply**, which is a bench tether with no telemetry;
- it can **shut the Pi down remotely**, with a countdown anyone can cancel;
- it has a **low-voltage auto-shutdown**, ported from the old `~/ups-dashboard` service.

Later it will also read a main and an auxiliary battery (see Future batteries below).

```bash
python -m robot_utils.nodes.power                 # on the robot: I2C bus 1, address 0x2d
python -m robot_utils.nodes.power --fake          # simulated HAT; never powers anything off
```

The page is at `http://<robot>/nodes/power/`. The node is not actuated, so E-STOP ALL skips it.
It does run an E-STOP ALL of its own before it powers the Pi off.

| file | what |
|---|---|
| `ups.py` | register map, `decode()`, `UpsHat` (block reads, plus the one power-off write), `FakeBus` (a captured reading), `SimBus` (`--fake`) |
| `sources.py` | `Source` classes: `UpsSource`, `TetherSource`; the shared source envelope |
| `shutdown.py` | `ShutdownController`: confirm, countdown, cancel, e-stop then power-off, audit log, rate limit |
| `lowvolt.py` | `LowVoltage`: settings and the check |
| `node.py` | `PowerNode`: owner thread, history, `estop_robot()`, `system_power_off()` |
| `server.py`, `page.py` | the HTTP API and the page |
| `admin/power.py` | the SSH CLI |

## API

Paths are relative to the node root. The common endpoints (`/time`, `/health`, `/meta`, `/docs`,
`common/...`) are described in [nodes.md](nodes.md).

| request | what |
|---|---|
| `GET /state` | sources, low-voltage status, the shutdown, last 15 events (stable shape below) |
| `GET /events` | SSE: `/state` on every change and every second |
| `GET /history?source=ups&seconds=86400` | one row per 30 s, up to 24 h: `{fields, units, interval_s, rows}` |
| `GET /shutdown` | the shutdown (same as in `/state`) plus `log`, the audit log |
| `POST /shutdown` | `{confirm: "shutdown", reason, by, delay_s}`: schedule a shutdown (202) |
| `POST /shutdown/cancel` | `{by, reason}`: cancel during the countdown |
| `GET /low_voltage`, `POST /low_voltage` | `{enabled, volts, hold_s, confirm, by}` |
| `GET /log?n=50` | the audit log |

`/meta` reports `kind: "power"`, `actuated: false` and `has_page: true`.

### Stable contract (`/state`)

Other pages build on this shape, such as the landing page's robot overview, so it is kept stable.
- **Registry entry:** `{name: power, kind: power}` in `nodes.yaml`, listed after swerve with the
  hardware nodes.
- **`state`**: a short string, shown on the landing tile. Examples: `"UPS 59 % · charging"`,
  `"SHUTDOWN in 22 s"`, `"UPS stale"`, `"UPS unavailable"`.
- **`error`**: the UPS read error, or `null`.
- **`sources`**: a list, one object per power source. The keys are described under Sources below.
- **`shutdown`**: one flat object with the same keys whether or not a shutdown is pending.
  - `pending`: bool.
  - `phase`: `idle`, `countdown`, `executing`, `powering_off`.
  - `at`: when the power-off is due, in epoch seconds.
  - `in_s`: seconds until then.
  - `requested_at`: epoch seconds.
  - `by`: the name the requester typed.
  - `client`: the identity the node resolved.
  - `source`: `web`, `local` or `low_voltage`.
  - `reason`, `delay_s`, `estop` (the e-stop results), `error`.
  - `last`: the last shutdown that ended, or `null`. Its `phase` is `cancelled`, `done` or `failed`.

  Every field except `pending`, `phase`, `confirm`, `delay_bounds_s` and `last` is `null` when idle.
- **`low_voltage`**: the settings and the status (see Low-voltage shutdown below).

The whole object is about 2 KB.

```json
{
 "state": "UPS 59 % · charging", "error": null, "fake": false, "t": 1000.0,
 "sources": [
  {"id": "ups", "type": "ups", "role": "pi", "telemetry": true, "ok": true, "stale": false, "error": null,
   "age_s": 0.0, "model": "Waveshare UPS HAT (E)",
   "percent": 59, "pack_v": 15.587, "cells_v": [3.92, 3.874, 3.877, 3.916], "current_a": 0.512, "power_w": 7.98,
   "charge_state": "charging", "charging": true, "fast_charge": true, "input_present": true,
   "runtime_min": null, "runtime_source": null, "time_to_full_min": 250, "remaining_mah": 2815,
   "input": {"present": true, "v": 8.675, "a": 2.09, "w": 18.17},
   "charger_state": "constant current", "links": {"fuel_gauge": true, "charger": true}, "firmware": "2.0",
   "power_off_pending": false,
   "control": {"auto_power_on": true, "watchdog": false, "address": 45, "wd_timeout_s": 255, "wd_delay_s": 255}},
  {"id": "main", "type": "tether", "role": "main", "telemetry": false, "ok": null, "stale": false, "error": null,
   "age_s": null, "note": "bench supply on a tether; no battery installed"}
 ],
 "low_voltage": {"enabled": false, "volts": 13.2, "hold_s": 10.0, "status": "off", "low": false, "pack_v": 15.587,
                 "range_v": [11.0, 16.0], "range_hold_s": [5.0, 120.0], "first_run": true, "source": "ups"},
 "shutdown": {"pending": true, "phase": "countdown", "by": "avery", "client": "avery · laptop", "source": "web",
              "reason": "end of day", "delay_s": 30.0, "requested_at": 1794215560.42, "at": 1794215590.42,
              "in_s": 22.4, "estop": null, "error": null, "confirm": "shutdown",
              "delay_bounds_s": {"default": 30.0, "min": 10.0, "max": 300.0}, "last": null},
 "events": [{"t": 990.2, "wall": 1794215550.1, "msg": "...", "level": "info"}]
}
```

### Sources

Every source has the same envelope:
- `id`: `"ups"`, `"main"`, later `"aux"`.
- `type`: `"ups"`, `"tether"`, later `"battery"`.
- `role`: what it powers. `pi` keeps the Pi up; `main` powers the robot.
- `telemetry`: `false` when there is nothing to read.
- `ok`: the last read succeeded and is fresh. `null` without telemetry.
- `stale`: the last good reading is older than 5 s.
- `error` and `age_s`.

A source with a battery adds the **common battery fields**. Each is `null` when unknown:
- `percent`, `pack_v`, `cells_v`;
- `current_a` (+ = charging) and `power_w`;
- `charge_state`: `charging`, `full`, `discharging` or `idle`;
- `charging` and `input_present`;
- `runtime_min`, with `runtime_source`: `gauge` (the fuel gauge's time to empty) or `estimate`
  (remaining mAh divided by the current);
- `time_to_full_min` and `remaining_mah`.

The UPS also reports `input` (USB-C V / A / W), `charger_state`, `links` (fuel gauge and charger
I2C), `firmware`, `control` (auto power-on, watchdog) and `power_off_pending` (0x01 reads 0x55).

`RU.viz.Battery.fromSource(src)` (`common/viz.js`) turns any of these into the battery gauge's
data. The page uses the full gauge on each source card and the compact one (`{compact: true}`,
24 px) in the header.

## Shutdown

Losing the Pi loses the robot for everyone until someone powers it on again, so a shutdown has
to be deliberate.

1. **Request.** `POST /shutdown` with the exact string `confirm: "shutdown"` and a non-empty
   `reason`. The countdown `delay_s` is 10 to 300 s (default 30). The node refuses:
   - a missing or different confirm string (400);
   - no reason, or a countdown out of range (422);
   - a request while another one is pending (409);
   - a web client's second request within 60 s, or more than 5 in an hour (429). Requests on
     the unix socket (the SSH CLI) aren't rate-limited.

   Every refusal is logged.
2. **Countdown.** The pending shutdown is in `/state`, on the landing tile (`state:
   "SHUTDOWN in 22 s"`) and as a sticky banner on the page. The page shows the banner on load,
   with a large **Cancel shutdown** button.
3. **Cancel.** `POST /shutdown/cancel` (no confirm needed), from the page, the CLI or any
   client, any time before the deadline. Once the countdown has ended it is too late (409).
4. **At the deadline.** A worker thread runs, in this order:
   1. The node **e-stops every actuated node** with the landing page's own fan-out
      (`frontend.app.estop_all`, reused). It covers the nodes in `nodes.yaml` (`--config`) plus
      any other node socket on the Pi, and records the result per node. Power-off goes ahead
      even if an e-stop fails, since the power is going anyway.
   2. The node calls **power-off**:
      1. It writes the HAT's power-off trigger (0x01 ← 0x55). The HAT then cuts power about
         30 s later, and **this can't be undone**.
      2. It runs `systemctl poweroff`. The polkit rule
         `/etc/polkit-1/rules.d/50-botcore-poweroff.rules` lets `botcore` do this without sudo.

**Power-off is injected.** `PowerNode(power_off=...)` defaults to `refuse_power_off`, which
raises. Only `__main__` passes the real `system_power_off(ups)`, and only without `--fake`. Under
`--fake` the e-stop still runs against the nodes it can reach, and the power-off only logs "would
power off". Tests inject fakes and fail on any subprocess call.

**After the cut.** The HAT's auto power-on (0x40 bit 0) is on. With USB-C input present, the HAT
may power the Pi back up after it cuts power. This hasn't been verified on this robot. Without
input, the Pi stays off until input returns or someone power-cycles it by hand. The confirm step
on the page says which case applies right now.

**Identity.** Requests through nginx carry `X-Forwarded-For`. The node takes the **last** hop,
which is the address nginx itself saw. Earlier hops come from the client and can be forged. The
node resolves that address with the frontend's Tailscale identities (`tailscale whois`, cached),
for example `avery · laptop`. The landing page's fallback proxy now appends the peer the same
way. Requests straight on the unix socket are `local (unix socket)`. Each record keeps both the
typed `by` and the resolved `client`.

**Audit log.** Requests, refusals, cancellations, e-stop results, outcomes and low-voltage
setting changes go to three places:
- the journal: `journalctl --user -u robot-node-power`;
- the event list on the page;
- an append-only, fsynced JSON-lines file, `~/.local/state/robot_utils/power/shutdown.jsonl`.
  It survives the shutdown, so after a reboot it shows who shut the robot down and why.

### From SSH

```bash
.venv/bin/python -m robot_utils.admin.power status
.venv/bin/python -m robot_utils.admin.power shutdown --in 60 --reason "end of day"   # asks you to type "shutdown"
.venv/bin/python -m robot_utils.admin.power shutdown --in 30 --reason "..." --yes     # scripts: no prompt
.venv/bin/python -m robot_utils.admin.power cancel
.venv/bin/python -m robot_utils.admin.power log
```

The CLI talks to the running node over its socket. It identifies you as `<login> via ssh from
<Tailscale identity>`, the same as `admin.locks` (`--as` overrides it). Without a terminal it
refuses unless you pass `--yes`.

## Low-voltage shutdown

The node schedules a shutdown through the same path as a manual one when all of these hold:
- the feature is enabled;
- the UPS pack stays below `volts` for `hold_s` seconds;
- the pack is not charging.

The countdown is 10 s, so you can cancel it.

A cancelled low-voltage shutdown re-arms after 2 min if the pack is still low. To stop it for
good, turn the feature off.

Missing, stale or invalid readings never trigger it. A wrong chip ID counts as invalid, and an
I2C failure is not a low battery.

Settings live in `~/.config/robot_utils/power.json`: `enabled`, `volts` (11 to 16 V; 13.2 V is
3.3 V per cell), and `hold_s` (5 to 120 s, default 10). Change them on the page or with
`POST /low_voltage`. Enabling it while the pack is already below the threshold and not charging
needs `confirm: true`.

The defaults are those of the old service, except for two changes:
- **It is off on first run.** The old dashboard is still doing the same job.
- **It adds a cancellable 10 s countdown** after the 10 s hold. The old service powered off
  straight after the hold.

## Cutover from `~/ups-dashboard`

The old service, `ups-dashboard.service` (port 8081, system `python3`), still runs on the robot.
Its low-voltage shutdown is **enabled at 13.2 V** (`~/ups-dashboard/low_voltage.json`). Running
both would mean two monitors racing to power off, so the node's version is off until the cutover.
On the robot:

1. **Install the I2C library in the venv.** The robot venv has neither `smbus2` nor `smbus`, and
   doesn't see system site packages:
   ```bash
   ~/robot_utils/.venv/bin/pip install smbus2      # or: pip install -e '.[power]'
   ```
2. **Retire the old dashboard:**
   ```bash
   systemctl --user disable --now ups-dashboard
   ```
3. **Start the node and enable its low-voltage shutdown.** Add the start recipe to `nodes.yaml`:
   ```yaml
   - {name: power, kind: power, start: {module: robot_utils.nodes.power}}
   ```
   Restart the landing page, then press Start on the power tile. Tick **enabled** in the page's
   Low-voltage shutdown panel at 13.2 V, which matches the old setting. You can also run
   `curl -X POST localhost/nodes/power/low_voltage -H 'Content-Type: application/json' -d '{"enabled": true, "volts": 13.2, "by": "cutover"}'`.
4. **Check** the page and run `python -m robot_utils.admin.power status`.

The old dashboard also offered writes to 0x40 to 0x43: auto power-on, watchdog, I2C address and
watchdog timing. The node leaves those registers alone.

## Future batteries

A main or auxiliary battery is one more `Source` subclass in `sources.py`. For example, it could
read a BMS over CAN in its `poll()`. It sets `type = "battery"`, `role = "main"` or `"aux"`, fills
the common battery fields in `public()`, returns a `history_row()`, and goes into the source list
in `__main__`. Nothing else needs to change:
- `/state`, `/history?source=<id>` and the page's source cards are generic;
- `RU.viz.Battery.fromSource` works on any source with the common fields.

Two decisions remain open:
- whether the low-voltage check should watch the main battery too (today it watches the UPS
  only);
- what the robot should do on a low main battery: stop driving, or shut down.

## Tests

```bash
PYTHONPATH=src python -m pytest tests/test_power.py
```

The tests cover:
- register decoding against two captured readings;
- the sources' shape;
- shutdown: confirm, countdown, cancel, a second request, the rate limit, forged hops, and
  e-stop running before power-off (with fakes);
- low-voltage logic: hold, charging, stale readings, re-arm, confirm;
- the API and the CLI.

No test touches I2C, a process or another node.
