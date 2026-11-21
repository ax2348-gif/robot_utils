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
| `cpu.py` | `CpuGovernor`: read, switch and keep the CPU frequency governor (`deploy/power/`: the root helper) |
| `powerloss.py` | `PowerLoss`: time until the Pi loses power, and whether that is a clean shutdown or a UPS cutoff |
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
| `GET /history?source=ups&seconds=86400` | one row per 30 s, up to 24 h: `{fields, units, limits, interval_s, rows}` (see History below) |
| `GET /shutdown` | the shutdown (same as in `/state`) plus `log`, the audit log |
| `POST /shutdown` | `{confirm: "shutdown", reason, by, delay_s}`: schedule a shutdown (202) |
| `POST /shutdown/cancel` | `{by, reason}`: cancel during the countdown |
| `GET /low_voltage`, `POST /low_voltage` | `{enabled, volts, hold_s, confirm, by}` |
| `GET /cpu`, `POST /cpu` | the CPU governor; `{governor, by}` switches it (CPU governor below) |
| `GET /log?n=50` | the audit log |

`/meta` reports `kind: "power"`, `actuated: false` and `has_page: true`.

### Stable contract (`/state`)

Other pages build on this shape, such as the landing page's robot overview, so it is kept stable.
- **Registry entry:** `{name: power, kind: power}` in `nodes.yaml`, listed after swerve with the
  hardware nodes.
- **`state`**: a short string, shown on the landing tile. Examples: `"UPS 59 % · charging"`,
  `"UPS 46 % · draining on input"` (instead of the charger's `charging` while the pack drains on input,
  held as in the watch below), `"SHUTDOWN in 22 s"`, `"UPS stale"`, `"UPS unavailable"`. While the pack drains
  and the power-loss estimate has one (below), it ends with when and how the Pi goes down:
  `"UPS 46 % · draining on input · auto-shutdown in ≈ 34 min"` or `"... · power cut in ≈ 2 h 10 min"`.
- **`cpu`** (added; additive): the CPU governor, the same object as `GET /cpu` (read at most once a second).
  Absent when the node runs without CPU control.
- **`sources[ups].power_loss`** (added; additive): the time until the Pi loses power and how. See Time until
  power loss below for the fields and the method. Show this, not `runtime_min`.
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
- **`conditions`** (added; additive): the latest 10 conditions the watch reported (Power log and watch below),
  oldest first, as spans: `{key, label, active, start, end, detail}`. `key` is `input`, `drain`, `cell_low` or
  `spread`; `start` and `end` are epoch seconds (`end` is `null` while `active`); `detail` is the reading when it
  started, e.g. `"cell 3 3.25 V"`. In memory only: empty after a restart.

The whole object is about 2 KB (up to ~3 KB with conditions).

```json
{
 "state": "UPS 59 % · charging", "error": null, "fake": false, "t": 1000.0,
 "sources": [
  {"id": "ups", "type": "ups", "role": "pi", "telemetry": true, "ok": true, "stale": false, "error": null,
   "age_s": 0.0, "model": "Waveshare UPS HAT (E)",
   "percent": 59, "pack_v": 15.587, "cells_v": [3.92, 3.874, 3.877, 3.916], "current_a": 0.512, "power_w": 7.98,
   "charge_state": "charging", "charging": true, "fast_charge": true, "input_present": true,
   "runtime_min": null, "runtime_source": null, "time_to_full_min": 250, "remaining_mah": 2815,
   "input": {"present": true, "v": 8.675, "a": 2.09, "w": 18.17}, "draining_on_input": false,
   "charger_state": "constant current", "links": {"fuel_gauge": true, "charger": true}, "firmware": "2.0",
   "power_off_pending": false,
   "control": {"auto_power_on": true, "watchdog": false, "address": 45, "wd_timeout_s": 255, "wd_delay_s": 255},
   "power_loss": {"status": "not_draining", "in_min": null, "range_min": null, "at": null, "event": null,
                  "clean": null, "limit": null, "cutoff_in_min": null, "blocked_reason": null, "method": "...",
                  "confidence": null, "note": null, "cell": null, "cell_v": null, "slope_mv_min": null,
                  "current_a": 0.512, "on_input": true, "window_min": null, "text": "not draining (charging)"}},
  {"id": "main", "type": "tether", "role": "main", "telemetry": false, "ok": null, "stale": false, "error": null,
   "age_s": null, "note": "bench supply on a tether; no battery installed"}
 ],
 "low_voltage": {"enabled": false, "volts": 13.2, "hold_s": 10.0, "status": "off", "low": false, "pack_v": 15.587,
                 "range_v": [11.0, 16.0], "range_hold_s": [5.0, 120.0], "first_run": true, "source": "ups"},
 "shutdown": {"pending": true, "phase": "countdown", "by": "avery", "client": "avery · laptop", "source": "web",
              "reason": "end of day", "delay_s": 30.0, "requested_at": 1794215560.42, "at": 1794215590.42,
              "in_s": 22.4, "estop": null, "error": null, "confirm": "shutdown",
              "delay_bounds_s": {"default": 30.0, "min": 10.0, "max": 300.0}, "last": null},
 "events": [{"t": 990.2, "wall": 1794215550.1, "msg": "...", "level": "info"}],
 "conditions": [{"key": "drain", "label": "draining on input", "active": false, "start": 1794201230.0,
                 "end": 1794203030.0, "detail": "-0.13 A, -2.0 W, input 14.6 W"}]
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
  (remaining mAh divided by the current). Only while the charger bit is off, and from the gauge's idea of the
  pack's capacity, so it never covers a drain on input and doesn't see a weak cell (on 2026-11-20 the UPS cut
  power at 45 %). Kept unchanged for compatibility; the UPS's `power_loss` is the time to show;
- `time_to_full_min` and `remaining_mah`.

The UPS also reports `input` (USB-C V / A / W), `charger_state`, `links` (fuel gauge and charger
I2C), `firmware`, `control` (auto power-on, watchdog), `power_off_pending` (0x01 reads 0x55) and
`draining_on_input`: input present but the pack net discharging (current below -0.05 A) in this reading, while
`charging` (the charger bit) can still be true. It has no hold; `conditions` has the held span.

The net battery power is `power_w` (+ charging, - draining). The load is not measured: input `input.w` -
`power_w` is an upper bound, since converter losses count as load.

`RU.viz.Battery.fromSource(src)` (`common/viz.js`) turns any of these into the battery gauge's
data. The page uses the full gauge on each source card and the compact one (`{compact: true}`,
24 px) in the header.

## Time until power loss

`sources[ups].power_loss` says when the Pi will lose power and whether that is a **clean automatic shutdown**
(the node's low-voltage shutdown) or an **abrupt UPS cutoff**. It is refreshed every 5 s.

**Why the weakest cell.** On 2026-11-20 the pack drained about 2 W on input from midnight (62 % at 12:30, 45 % at
18:21) with the charger bit saying "charging". Cell 3 fell away from the others (3.73 V at 16:40, 3.38 V at 18:03)
and the UPS cut power at 18:21:25 with cell 3 at 3.05 V under 0.22 A, the pack at 13.32 V and the gauge at 45 %.
So the limit is the weakest cell reaching the UPS's own cutoff, `CELL_CUTOFF_V = 3.05` V, an estimate from that
one observed cutoff. The gauge's percent and time to empty can't see it.

**Method** (`powerloss.py`):
1. Every 1 Hz reading goes into a 35 min ring (refilled from the disk log at startup; `--fake` back-fills it from
   the simulated curve). Readings are reduced to per-minute medians, which drops load spikes.
2. Only the trailing run of draining minutes (median net current below -0.05 A) with the same input state as now
   is used, up to 30 min. One odd minute is skipped; two, a gap of more than 3 min or a change of input end it.
3. A least-squares line through the weakest cell's medians gives its slope at the middle of the window.
4. The knee: a cell falls faster as it nears empty (cell 3 fell 0.4 mV/min on the plateau, 3.5 at 3.65 V, 9 at
   3.32 V, ~40 in its last minute). The model is that the slope grows as `exp(dv / λ)` as the voltage falls by
   `dv`, with `λ = 0.15` V. The measured slope is moved from the window's middle to now with that rule, and the
   time to reach a target is `λ / slope · (1 − exp(−(v − target) / λ))`. A straight line is optimistic near the
   end and, on the plateau, says days.
5. Limits: the UPS cutoff (weakest cell → 3.05 V) always; with the low-voltage shutdown enabled also its
   `cell_volts` (weakest cell) and `volts` (the pack's trend, with `4λ`: errs towards later, i.e. towards calling it
   a cutoff). The auto-shutdown's time is its threshold + `hold_s` + the 10 s countdown. It is **clean** when that
   plus 30 s for the power-off itself still comes before the cutoff, judged with the gentler knee `λ = 0.25` V (the
   steep one squeezes the last 0.1 V into too little time: cell 3 took about 2.5 min from 3.15 V to the cutoff).
6. `range_min` reruns it with `λ = 0.10` and `0.25` V. `confidence` is `low` while the weakest cell is above
   3.65 V (on the plateau the knee isn't visible yet), `medium` below.

**Backtest** on the 2026-11-20 log (1 Hz, replayed up to each point; auto-shutdown off, so the event is the
cutoff; `tests/test_power.py::test_power_loss_backtest_2026_11_20` repeats it on a 30 s extract,
`tests/data/ups-2026-11-20-cutoff.csv.gz`):

| before the cutoff | weakest cell | estimate | range | confidence | estimate / real |
|---|---|---|---|---|---|
| 6 h | cell 2, 3.836 V | 6 h 16 min | 4 h 07 – 10 h 12 | low | 1.04 (by luck) |
| 3 h | cell 3, 3.759 V | 4 h 08 min | 2 h 42 – 6 h 42 | low | 1.38 |
| 2 h | cell 2, 3.732 V | 7 h 02 min | 4 h 40 – 11 h 14 | low | 3.5 |
| 1 h 30 | cell 3, 3.713 V | 3 h 37 min | 2 h 22 – 5 h 50 | low | 2.4 |
| 1 h | cell 3, 3.618 V | 34 min | 20 – 59 min | medium | 0.57 |
| 30 min | cell 3, 3.471 V | 18 min | 10 – 31 min | medium | 0.59 |
| 20 min | cell 3, 3.395 V | 12 min | 6.5 – 22 min | medium | 0.62 |
| 10 min | cell 3, 3.290 V | 7.1 min | 3.6 – 12.7 min | medium | 0.71 |
| 5 min | cell 3, 3.209 V | 4.1 min | 2.0 – 7.4 min | medium | 0.82 |
| 2 min | cell 3, 3.133 V | 1.9 min | 0.9 – 3.6 min | medium | 0.95 |

With the auto-shutdown on (`cell_volts` 3.15 V) it reports a clean auto-shutdown from 6 h out to the end, about
1 min before the cutoff estimate near the end (e.g. 17 min at 30 min out, 6 min at 10 min out).

**How far to trust it.** One event set `λ`, the 3.65 V plateau line and the 3.05 V cutoff; a straight line, a
generic Li-ion voltage-to-charge table (this pack's cells stay near 3.8 V far longer than one says) and windows of
10 to 60 min did worse. In the last hour it was 0.55–0.95x the real time, on the short side, and the real time was
inside the range (the top of the range an hour out was 59.4 min at 1 Hz). On the plateau it is a guess: it said
3.5x too long 2 h out, before the knee showed, which is why it says `confidence: low` there. A load step shortens
the end (the last 3 min here ran at 0.22 A instead of 0.14 A). The log has no stretch on battery longer than 3 min,
so the on-battery case is untested on real data: the same method applies (the 30 min window restarts when the
input goes) and the node shows it there too, not the gauge's `time_to_empty_min` (`runtime_min`), which comes from
a 4800 mAh capacity the pack no longer has (it gave about 2.5 Ah from full before cell 3 cut out).

Fields (null when not applicable):
- `status`: `draining` (an estimate), `not_draining` (net current at or above -0.05 A, or the weakest cell isn't
  falling), `estimating` (draining, but under 10 min of it), `no_data` (no reading in the last 2 min),
  `shutdown_pending` (a shutdown is scheduled: it comes first; `event: "shutdown"`, `in_min` its countdown).
- `in_min`: minutes until the Pi goes down: the end of a clean auto-shutdown's countdown (power-off then takes
  ~30 s), or the UPS cutoff. `at`: the same as epoch seconds. `range_min`: `[short, long]`.
- `event`: `auto_shutdown` or `ups_cutoff` (or `shutdown`); `clean`: true for the first.
- `limit`: `{kind: "cutoff" | "cell" | "pack", cell, target_v, v, label}`, e.g. `cell 3 → 3.05 V`.
- `cutoff_in_min`: minutes until the weakest cell reaches the cutoff, whatever the auto-shutdown does.
- `blocked_reason`: why it ends in a cutoff: `auto-shutdown is off`, or `auto-shutdown (cell 3 → 3.15 V) would come
  too late` (a threshold too close to the cutoff, or a long hold).
- `method`, `confidence` (`low`, `medium`; `high` for a pending shutdown), `note`.
- `cell`, `cell_v`, `slope_mv_min` (+ = falling), `current_a`, `on_input`, `window_min`: what it was computed from.
- `text`: one line, e.g. `power loss in ≈ 34 min: clean auto-shutdown (cell 3 → 3.15 V)` or `power loss in ≈ 2 h
  10 min: UPS cutoff (cell 3 → 3.05 V); auto-shutdown is off (rough: knee not visible yet)`.

`RU.viz.Battery.fromSource` takes `power_loss.in_min` as the gauge's runtime when the source has it (and doesn't
draw a pack draining on input as charging), so the landing tile's gauge says `~34 min left` too. The SSH CLI's
`status` prints `text` with its range and confidence.

## CPU governor

The Pi 5 has one cpufreq policy for all four cores, and its governor trades latency against power. Measured on
2026-11-20 at a light robot load:
- `ondemand` (the kernel's default here): about 1.9 GHz median and ~62 frequency changes/s (each a firmware
  mailbox call), costing ~16 % more CPU time and ~6 ms more estimator lag;
- `performance`: 2.4 GHz fixed, about 0.45 W more drain on the UPS pack (which matters while the input can't keep
  up: Time until power loss above);
- `schedutil`: a middle ground, untested.

`GET /cpu` (and `/state` `cpu`): `{available, governor, governors, ghz, ghz_range, transitions_s, policies, saved,
helper_installed, error}`; without cpufreq only `{available: false, governor: null, governors: [], error}`.
`transitions_s` is the change rate since the previous read (at least 1 s earlier). `POST /cpu {governor, by}` runs
`systemctl start robot-cpu-governor@<governor>.service`; 422 for a governor the kernel doesn't offer, 503 without
cpufreq or the helper, 502 if the switch failed. Every switch (and failure) goes to the audit log
(`cpu_governor`, with `before` and `after`), the journal and the page's events. The choice is saved in
`power.json` (`cpu`) and re-applied at startup, since the kernel boots on its default. `--fake` reads the local
cpufreq but never switches it.

Writing the governor needs root, so it goes through a oneshot unit that validates the name and writes every policy,
and a polkit rule that lets `botcore` start only that unit. Install it once on the robot:

```bash
sudo sh ~/robot_utils/deploy/power/install.sh
```

The page's CPU governor panel shows the governor, the clock, the changes per second, the saved choice, whether the
helper is installed (with that line if not) and the last error, with a select and Apply.

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

The node schedules a shutdown through the same path as a manual one when all of these hold for
`hold_s` seconds:
- the feature is enabled;
- the pack is **net draining** (current below -20 mA), whatever the charger's "charging" bit says;
- the pack is below `volts`, **or** its weakest cell is below `cell_volts`.

Why net current and the weakest cell (2026-11-20): an undersized input (5 V at its 3 A limit) left the
charger reporting "charging" while the pack drained about 2 W for hours. One weak cell then reached the
UPS's own cutoff (about 3.05 V under load) while the pack still read 13.32 V, above `volts`. The UPS cut
the Pi's power with no shutdown. The old rule (pack below `volts` and not charging) could not fire in
either case. A net-charging pack never triggers, so a Pi that boots with its nodes off and charges cannot
shut down again in a loop.

The countdown is 10 s, so you can cancel it.

A cancelled low-voltage shutdown re-arms after 2 min if the pack is still low. To stop it for
good, turn the feature off.

Missing, stale or invalid readings never trigger it. A wrong chip ID counts as invalid, and an
I2C failure is not a low battery.

Settings live in `~/.config/robot_utils/power.json`: `enabled`, `volts` (11 to 16 V; 13.2 V is
3.3 V per cell), `cell_volts` (2.9 to 3.6 V, default 3.15: above the observed cutoff, with a few minutes'
margin at the drain seen that day), and `hold_s` (5 to 120 s, default 10). Change them on the page or with
`POST /low_voltage` (`cell_volts` is optional there). Enabling it while the pack is already low and
draining needs `confirm: true`. `GET /low_voltage` (and `/state`) also give `reason` (why it is low, e.g.
"cell 3 at 3.12 V below 3.15 V, draining 0.21 A although the charger reports charging") and `min_cell_v`.

The defaults are those of the old service, except for two changes:
- **It is off on first run.** The old dashboard is still doing the same job.
- **It adds a cancellable 10 s countdown** after the 10 s hold. The old service powered off
  straight after the hold.

## Power log and watch

Every UPS poll (1 Hz) is appended to `~/.local/state/robot_utils/power/log/ups-YYYY-MM-DD.csv` (local date):
`wall, time, percent, pack_v, cell1_v..cell4_v, current_a (+ charging), power_w, input_v, input_a,
input_present, charging, fast_charge, charger_state, remaining_mah, error`. A failed read is a row with only the
time and the error. Rows are flushed as written and the file is fsynced every 5 s, so a brownout or power cut
loses at most the last ~5 s (the journal can lose its last minutes). Files older than 30 days are deleted. At
startup the node refills its 24 h history (`GET /history`) from the log, so a reboot no longer wipes it.

The node also writes a journal line (and a page event) when one of these starts or ends:

| Condition | Starts | Ends |
|---|---|---|
| input lost | input not present | present again |
| draining on input | input present, current below -50 mA for 10 s | current >= 0 for 10 s |
| low cell | any cell below 3.30 V for 5 s | all above 3.40 V |
| unbalanced | cells more than 0.20 V apart for 5 s | within 0.15 V |

Each condition is also kept as a span, from when it first held (before its hold ran out) to when its end first
held: `/state` `conditions`, shown on the page's Events panel as "since 14:02 (ongoing, 1 h 12 min)" or
"14:02–15:10 (1 h 08 min)". They are in memory only; the history charts below rebuild their spans from the history.

"Charging" in `/state` is the charger's status bit: it stays set while the input is present even when the pack
is net draining (the load is more than the input supplies). Read `current_a` or `power_w` for the net flow, or
`draining_on_input`; `state` says `draining on input` while the watch holds that condition.

## History

`GET /history` keeps one row per 30 s for 24 h, per source with telemetry (in memory; refilled from the power
log at startup). `fields` (append-only, so readers that index the first seven keep working) and `units`:

| field | unit |
|---|---|
| `t`, `wall` | server monotonic s, epoch s |
| `percent`, `pack_v`, `current_a` | %, V, A (+ charging) |
| `input_present`, `charging` | 0/1 (`charging` is the charger bit) |
| `input_w` | W, the USB-C input (0 without input) |
| `battery_w` | W, the pack's net power: + charging, - draining |
| `load_w` | W, **derived**: `input_w - battery_w`. Neither the Pi's draw nor the charger's output is measured, so converter losses count as load |
| `cell_min_v`, `cell_spread_v` | V, the lowest cell and max - min |

Rows restored from the log compute `input_w` as `input_v x input_a` (the log has no input W column). A value a
row lacks is `null`. `limits` gives the thresholds the page marks spans with: `{drain_a: -0.05, cell_low_v: 3.30,
spread_v: 0.20}` (the watch's, without its holds).

## Page

- **Header:** the battery gauge, `state` (caution while it says `draining on input`), and a power-flow pill:
  `in 14.6 W · pack −2.1 W draining on input · load ≈16.7 W`, caution while `draining_on_input`. While the pack
  drains on input the gauges drop the charging mark the charger bit would give them.
- **UPS card:** `state` says `draining on input (charger bit: charging)`; the pack's current and power carry their
  sign; a `load (derived)` row; a `power loss` row with the estimate's `text`, and under it its range, confidence,
  the cutoff time when a clean shutdown comes first, and the cell's slope. The gauge's own runtime, when it has one,
  is a separate `gauge runtime` row.
- **Header:** a power-loss pill with the same `text`, caution under 30 min.
- **History** (window 1 / 6 / 24 h; one crosshair shared by every chart, with a readout line of every value at it):
  - **Power flow** (W): input, pack net and the derived load (dashed) on one axis with a zero line. The pack net
    is filled against zero: a faint tint above it (charging), hatching below it (draining). Below it, the energy
    into and out of the pack, net, and input and load energy over the window.
  - **Cells** (V): the lowest cell with the 3.30 V line, and the spread with the 0.20 V line. While the pack
    drains and the lowest cell is within 0.5 V of the limit the estimate runs to, that limit is a second line,
    labelled `UPS cutoff 3.05 V · ≈ 34 min` or `auto-shutdown 3.15 V · ≈ 33 min`.
  - Charge, pack voltage and current, as before.
  - **Anomaly spans** from the history rows: draining on input (input present, current below -0.05 A) on the power
    flow, charge, voltage and current charts; a low cell on the cells chart; a spread above 0.20 V on the spread
    chart. Each is a thin hatched caution strip at the top of the chart with dotted edges; on the power-flow chart
    it is labelled with the pack's mean power in it (`draining on input: pack −1.6 W, input 14.6 W`, shortened to
    fit). A list under the power-flow chart gives the spans' times and durations.
- **Events:** the watched conditions as spans above the event lines.

The charts are page-local canvases (not `RU.Plot`, which has no zero fill, spans or time axis); colours come
from the theme tokens (`--plot-*`, `--caution`, `--ink`, `--muted`, `--rule`), so they follow light, dark and the
packaged themes, and redraw when the theme changes. `--fake` simulates a 30 min drain on input with cell 3
sagging below 3.3 V in each 4 h cycle, so the marking shows.

```
tail -f ~/.local/state/robot_utils/power/log/ups-$(date +%F).csv
journalctl --user -u robot-node-power -p warning
```

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
   - {name: power, kind: power, boot: true, start: {module: robot_utils.nodes.power}}
   ```
   Restart the landing page, then run `python -m robot_utils.frontend.boot` (install.sh does this):
   the node gets a persistent unit and starts at every boot, so low-voltage protection doesn't
   depend on someone pressing Start (docs/frontend.md, "Nodes that start at boot"). Tick **enabled** in the page's
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
- the history's power-flow and cell columns, live and restored from the log; draining on input in `/state`
  (`state`, `draining_on_input`, `conditions`); the simulated drain stretch; the page's charts (light check);
- the API and the CLI;
- the power-loss estimate: not draining, too little history, draining on input (cutoff with the auto-shutdown off,
  clean auto-shutdown with it on, a threshold too close to the cutoff), on battery, load spikes, `/state` and the
  tile string, a pending shutdown, and the 2026-11-20 backtest (a 5 KB gzipped extract of the log);
- the CPU governor (`tests/test_power_cpu.py`, a fake sysfs and runner): read, switch, save, re-apply at start,
  `GET`/`POST /cpu`, the audit record and event, no helper, and `/state` reading it at most once a second.

No test touches I2C, a process or another node.
