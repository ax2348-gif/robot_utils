"""Power node: the Pi's UPS HAT today (battery, input, remote shutdown); main / aux batteries later.

Guide: docs/power.md. Stable contract for other pages (the landing page's overview polls it):

- registry entry ``{name: power, kind: power}`` (nodes.yaml, with the hardware nodes);
- ``GET /state`` -> ``sources``: a list of ``{id, type, role, telemetry, ok, stale, error, age_s}``
  (``id`` "ups" / "main" / later "aux"; ``type`` "ups" / "tether" / later "battery") plus, for a
  source with a battery, ``percent, pack_v, cells_v, current_a, power_w, charge_state, charging,
  input_present, runtime_min, runtime_source, time_to_full_min, remaining_mah`` (null = unknown);
- ``GET /state`` -> ``shutdown``: flat, same keys whether pending or not: ``{pending, phase, at,
  in_s, requested_at, by, client, source, reason, delay_s, estop, error, confirm, delay_bounds_s,
  last}`` (times in epoch s; null when idle);
- ``GET /state`` -> ``state`` (short string for tiles), ``error``, ``low_voltage``, ``events``.

``RU.viz.Battery.fromSource(src)`` (common/viz.js) draws any battery source.
"""

__version__ = "0.1.0"
