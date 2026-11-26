"""Haptic feedback: Quest controller vibration for the operator (docs/quest_teleop_node.md "Haptics").

The node turns what it knows into **cues** and serves the headset a compact **haptics state**, schema
``teleop.quest_haptics.v1`` (quest_env HAPTICS.md), at ``GET /haptics``:

    {"schema", "epoch", "seq", "ttl_s",
     "hands": {"left" | "right": {"level": 0..1, "frequency_hz": 0, "pulses": [{"id", "group", "amplitude",
                                   "duration_s", "frequency_hz", "delay_s", "age_s"}]}}}

``GET /haptics?stream=1`` is the headset's: one such line per change (``publish``), and at least every
``KEEPALIVE_S`` while anything is on, ``IDLE_S`` otherwise (chunked NDJSON, the AR state stream's pattern). A line is
pushed the moment a cue is made, so event -> buzz is the one-way network time plus at most one XR frame, not a poll
period.

- **pulses** carry ids increasing for the node's life (``epoch`` names the life: a new epoch resets the headset's
  memory), so the headset plays each once, ``delay_s`` after it first sees it; pulses created at once share a
  ``group`` (a pattern), and a newer group on a hand replaces that hand's pulses still waiting. Each line repeats the
  pulses of the last ``KEEP_S`` (a reconnecting reader gets them again; the ids keep it from replaying); ``age_s``
  lets the headset skip a pulse that arrives too late to mean anything (quest_env: 0.5 s).
- **level**: a continuous vibration while > 0, valid for ``ttl_s`` after the line arrives (lines come every
  ``KEEPALIVE_S`` while any level is on, so a lost link stops the buzz within ``ttl_s``). Pulses play over it.
- ``frequency_hz`` 0 = the runtime's default.

A headset build that plays these says so with a ``haptics`` block in its frames (``{"schema", "epoch", "last_id",
"connected"}``: the newest pulse id it accepted, its stream up): ``headset`` in ``info``, and the round trip
(``ack_ms``: pulse made -> the first frame acknowledging it). An older build has no block and never opens the stream.

Cues (``CUES``; each on / off with its amplitude, ``POST /haptics/config``; a master switch):

- **state changes**, from the node's events by kind (``EVENT_CUES``: a new kind slots in there): armed (one short,
  both hands), disarmed / held (two short), resumed (one short), disabled (one long). A cue does not repeat a cue of
  the same or higher priority on that hand within ``DEDUPE_S`` (X both disarms and disables: one long).
- **limit** (continuous, off by default): while the WBC reports a limit (``LIMIT_KINDS``) on an arm, a low buzz on the
  engaged hand driving it (the mirror pairing in force). The WBC's per-arm column ``<ee>_limited_by``; a WBC without
  it: its global ``limited_by`` on every engaged hand.
- **contact** (a pulse) and **grip** (continuous, off by default), from each arm node's stream (``ArmWatch``): the
  gripper closing on something (the arm's force limit binding, ``g_limiting``, or the jaws stalled short of their
  goal) gives one pulse on the hand whose trigger drives that gripper, and while in contact a faint level growing
  with the squeeze force (``g_force / g_force_max``).
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

SCHEMA = "teleop.quest_haptics.v1"
HANDS = ("left", "right")
LIMITED = ["none", "reach", "manipulability", "fence", "joint_limit", "base_limit", "base_band", "lead"]  # the WBC's
LIMIT_KINDS = ("reach", "manipulability", "joint_limit")
KEEP_S = 1.0             # pulses repeated in every line for this long after they are made
TTL_S = 0.5              # a level holds this long after its line arrives (the headset's clock)
KEEPALIVE_S = 0.2        # a stream line at least this often while a level or a recent pulse is on
IDLE_S = 1.0             # ... and this often otherwise (the reader's stall check: quest_env reconnects after 2 s)
DEDUPE_S = 0.2          # one action's events (X: disarmed + disabled) arrive within milliseconds
LIMIT_HOLD_S = 0.15      # a limit keeps its level on this long after the last limited row (no chatter)
WBC_STALE_S = 0.5
ARM_STALE_S = 0.5
STALL_VEL = 0.5          # rad/s (motor side): the jaws are not moving (the arm's GripperContact.stall_vel)
STALL_ERR = 0.001        # m: the goal at least this far past the jaws ...
STALL_S = 0.05           # ... for this long: in contact
LEVEL_STEP = 0.02        # a level change smaller than this does not send a line by itself

SHORT = ((0.0, 0.06),)                   # (delay_s, duration_s) per pulse
TWO_SHORT = ((0.0, 0.06), (0.14, 0.06))
LONG = ((0.0, 0.4),)
TAP = ((0.0, 0.04),)
THREE_TAPS = ((0.0, 0.03), (0.08, 0.03), (0.16, 0.03))


@dataclass(frozen=True)
class CueSpec:
    kind: str                        # pulse | level
    pattern: tuple = ()
    priority: Optional[int] = None   # pulses: the dedupe order (None: never deduplicated)
    enabled: bool = True             # the default
    amplitude: float = 0.5           # the default, 0..1
    doc: str = ""


CUES = {
    "armed": CueSpec("pulse", SHORT, 1, True, 0.6, "armed (Y): one short pulse, both hands"),
    "resumed": CueSpec("pulse", SHORT, 1, True, 0.6, "resumed after a hold: one short pulse, both hands"),
    "disarmed": CueSpec("pulse", TWO_SHORT, 2, True, 0.6, "disarmed (X, stale input, the WBC's latch): two short"),
    "held": CueSpec("pulse", TWO_SHORT, 2, True, 0.6, "held (the WBC holds: input stale, no lease, ...): two short"),
    "disabled": CueSpec("pulse", LONG, 3, True, 0.8, "the WBC disabled (X, a stop, an e-stop, a fault): one long"),
    "refused": CueSpec("pulse", THREE_TAPS, 2, True, 0.7, "a Y press that did not arm (the reason is in the event "
                       "log and on the page): three quick taps"),
    "contact": CueSpec("pulse", TAP, None, True, 0.5, "a gripper closed on something: one tap on the hand driving it"),
    "limit": CueSpec("level", enabled=False, amplitude=0.15,
                     doc="while the WBC limits an arm (reach, manipulability, joint limit): a low buzz on the hand "
                         "driving it"),
    "grip": CueSpec("level", enabled=False, amplitude=0.3,
                    doc="while a gripper is in contact: a faint buzz growing with the squeeze force"),
}

# event kind -> (cue, a filter on the event's detail or None). The WBC's own hold while disarmed ("disarmed: ...")
# echoes the disarm the operator already felt.
# Kinds from feat/teleop-resume (held, resumed, resume_refused, escalated) map here too; a kind this node never
# emits costs nothing.
EVENT_CUES: dict = {
    "armed": ("armed", None),
    "disarmed": ("disarmed", None),
    "hold": ("held", lambda d: not str((d or {}).get("reason") or "").startswith("disarmed")),   # the WBC's hold
    "held": ("held", None),                    # the node's hold (release and re-grip to resume)
    "resume_refused": ("held", None),          # still holding
    "resumed": ("resumed", None),
    "escalated": ("disarmed", None),           # a hold escalated to a disarm (press Y)
    "disabled": ("disabled", None),
    "estop": ("disabled", None),
    "y_refused": ("refused", None),            # a Y press that did not arm
}


def _num(row: Optional[dict], key: str) -> float:
    v = (row or {}).get(key)
    try:
        v = float(v)
    except (TypeError, ValueError):
        return math.nan
    return v


class GripContact:
    """Contact of one gripper from its arm's rows: the force limit binding (``g_limiting``), or the jaws stalled
    short of their goal (``g_gap_goal`` below ``g_gap`` by ``STALL_ERR``, ``|g_qd| < STALL_VEL``) for ``STALL_S``."""

    def __init__(self):
        self.on = False
        self.stall_since: Optional[float] = None

    def update(self, row: Optional[dict], now: float) -> bool:
        if row is None:
            self.on, self.stall_since = False, None
            return False
        lim = _num(row, "g_limiting")
        gap, goal, qd = _num(row, "g_gap"), _num(row, "g_gap_goal"), _num(row, "g_qd")
        stalled = (math.isfinite(gap) and math.isfinite(goal) and math.isfinite(qd)
                   and goal < gap - (STALL_ERR / 2 if self.on else STALL_ERR) and abs(qd) < STALL_VEL)
        if stalled:
            if self.stall_since is None:
                self.stall_since = now
        else:
            self.stall_since = None
        self.on = bool((math.isfinite(lim) and lim >= 0.5)
                       or (stalled and (self.on or now - self.stall_since >= STALL_S)))
        return self.on

    @staticmethod
    def force_fraction(row: Optional[dict]) -> float:
        f, fmax = _num(row, "g_force"), _num(row, "g_force_max")
        if not (math.isfinite(f) and math.isfinite(fmax) and fmax > 0):
            return 0.5
        return min(1.0, max(0.0, f / fmax))


class ArmWatch:
    """The newest row of one arm node's stream, thinned (``max_hz``): the gripper columns. Reconnects on its own."""

    def __init__(self, target: str, max_hz: float = 30.0, clock: Callable[[], float] = time.monotonic):
        self.target, self.max_hz, self.clock = target, max_hz, clock
        self.row: Optional[dict] = None
        self.t: Optional[float] = None
        self.last_error: Optional[str] = None
        self._client = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"haptics-arm-{target}", daemon=True)

    def start(self) -> "ArmWatch":
        self._thread.start()
        return self

    def latest(self) -> tuple:
        return self.row, self.t

    def close(self) -> None:
        self._stop.set()
        c = self._client
        if c is not None:
            try:
                c.close()
            except Exception:  # noqa: BLE001
                pass
        self._thread.join(2)

    def _run(self) -> None:
        from ..common.client import NodeClient
        keys = ("g_gap", "g_gap_goal", "g_qd", "g_force", "g_force_max", "g_limiting")
        while not self._stop.is_set():
            try:
                self._client = NodeClient(self.target)
                for _, rows in self._client.rows("/stream", max_hz=self.max_hz):
                    if self._stop.is_set():
                        return
                    last = rows[-1]
                    names = rows.dtype.names
                    self.row = {k: float(last[k]) for k in keys if k in names}
                    self.t = self.clock()
            except Exception as e:  # noqa: BLE001 (any transport error: reconnect)
                self.last_error = f"{type(e).__name__}: {e}"[:200]
            finally:
                c, self._client = self._client, None
                if c is not None:
                    try:
                        c.close()
                    except Exception:  # noqa: BLE001
                        pass
            self._stop.wait(1.0)


class Sources:
    """What the cues read (the node's view; tests substitute their own)."""

    def running(self) -> bool:
        return False

    def armed(self) -> bool:
        return False

    def pairing(self) -> dict:
        """{hand: ee} in force."""
        return {}

    def engaged(self) -> dict:
        """{hand: bool}."""
        return {}

    def wbc_row(self) -> tuple:
        """(the newest WBC row, its arrival on our clock)."""
        return None, None

    def arm_target(self, ee: str) -> Optional[str]:
        """The arm node driving this EE (its stream has the gripper), or None."""
        return None


class NodeSources(Sources):
    def __init__(self, node):
        self.node = node

    def running(self) -> bool:
        return self.node.phase == "running" and self.node.client is not None

    def armed(self) -> bool:
        c = self.node.client
        return bool(c is not None and c.latch.armed)

    def pairing(self) -> dict:
        p = self.node.pairing_info()
        if p is not None:
            return dict(p["hands"])
        return {self.node.cfg.hand: self.node.cfg.ee}

    def engaged(self) -> dict:
        c = self.node.client
        core = c.core if c is not None else None
        if core is None:
            return {}
        if hasattr(core, "cores"):
            return {h: cc.status.state == "engaged" for h, cc in core.cores.items()}
        return {self.node.cfg.hand: core.status.state == "engaged"}

    def wbc_row(self) -> tuple:
        return self.node.stream.latest()

    def arm_target(self, ee: str) -> Optional[str]:
        arms = self.node.arm_targets
        if ee in arms:
            return ee
        if len(arms) == 1 and not self.node.cfg.hands:
            return arms[0]
        return None



class Haptics:
    def __init__(self, sources: Sources, *, clock: Callable[[], float] = time.monotonic,
                 epoch: Optional[str] = None, watch_factory: Callable = ArmWatch):
        self.src = sources
        self.clock = clock
        self.enabled = True
        self.cues = {k: {"enabled": c.enabled, "amplitude": c.amplitude} for k, c in CUES.items()}
        self.limit_kinds = list(LIMIT_KINDS)
        self.epoch = epoch or format(int(time.time() * 1000), "x")
        self.watch_factory = watch_factory
        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)   # a new line published (the stream readers wait on it)
        self._id = 0
        self._seq = 0
        self.pulses: deque = deque(maxlen=64)      # recent pulses (dicts with "t" on our clock, "hand", "cue")
        self.levels = {h: 0.0 for h in HANDS}
        self.level_cue = {h: None for h in HANDS}
        self._sent_levels = {h: 0.0 for h in HANDS}
        self._line: Optional[dict] = None          # the newest published line (seq = self._seq)
        self.readers = 0                           # open GET /haptics?stream=1 (the headset's)
        self._last_cue: dict = {}                  # hand -> (t, priority)
        self._limit_seen: dict = {}                # ee -> the last time a limited row was seen
        self._contact: dict = {}                   # ee -> GripContact
        self.watches: dict = {}                    # arm target -> ArmWatch
        self._acked = 0                            # the newest pulse id acknowledged
        self.ack_ms: deque = deque(maxlen=50)
        self.headset: dict = {"capable": False, "last_id": None, "epoch": None, "connected": None}
        self.counts = {"cues": {k: 0 for k in CUES}, "suppressed": 0, "published": 0, "streamed": 0}
        self.last_cue: Optional[dict] = None

    # -- config ------------------------------------------------------------------------------------------------

    def configure(self, enabled: Optional[bool] = None, cues: Optional[dict] = None,
                  limit_kinds: Optional[list] = None) -> dict:
        """Change the master switch, cues ({name: {enabled, amplitude}}) or the limit kinds. ValueError on an
        unknown cue / kind or an amplitude outside 0..1; nothing changes then."""
        cues = cues or {}
        for k, v in cues.items():
            if k not in CUES:
                raise ValueError(f"unknown cue {k!r} (cues: {', '.join(CUES)})")
            if not isinstance(v, dict) or set(v) - {"enabled", "amplitude"}:
                raise ValueError(f"cue {k}: {{enabled, amplitude}} only")
            a = v.get("amplitude")
            if a is not None and not (isinstance(a, (int, float)) and 0.0 <= float(a) <= 1.0):
                raise ValueError(f"cue {k}: amplitude must be within 0..1")
        if limit_kinds is not None:
            bad = [k for k in limit_kinds if k not in LIMITED or k == "none"]
            if bad:
                raise ValueError(f"unknown limit kinds {bad} (the WBC's: {', '.join(LIMITED[1:])})")
        with self._lock:
            if enabled is not None:
                self.enabled = bool(enabled)
            for k, v in cues.items():
                if v.get("enabled") is not None:
                    self.cues[k]["enabled"] = bool(v["enabled"])
                if v.get("amplitude") is not None:
                    self.cues[k]["amplitude"] = float(v["amplitude"])
            if limit_kinds is not None:
                self.limit_kinds = list(limit_kinds)
        self.update(self.clock(), force=True)
        return self.config()

    def config(self) -> dict:
        return {"enabled": self.enabled, "limit_kinds": list(self.limit_kinds),
                "cues": {k: {**v, "kind": CUES[k].kind, "doc": CUES[k].doc} for k, v in self.cues.items()}}

    def _on(self, cue: str) -> bool:
        return self.enabled and self.cues[cue]["enabled"]

    # -- pulses ------------------------------------------------------------------------------------------------

    def on_event(self, line: dict) -> list:
        """An event line (eventlog listener): its cue by ``EVENT_CUES``, on both hands."""
        entry = EVENT_CUES.get(line.get("kind"))
        if entry is None:
            return []
        cue, keep = entry
        if keep is not None and not keep(line.get("detail")):
            return []
        return self.cue(cue, HANDS, source=line.get("kind"))

    def cue(self, name: str, hands=HANDS, *, test: bool = False, source: Optional[str] = None) -> list:
        """Make a cue's pulses on these hands and send them at once. ``test``: play it even when off (the page's
        test buttons; a level cue plays as a 1 s pulse at its amplitude). Returns the pulses made."""
        spec = CUES[name]
        if not test and (spec.kind != "pulse" or not self._on(name)):
            self.counts["suppressed"] += 1
            return []
        now = self.clock()
        amp = self.cues[name]["amplitude"]
        pattern = spec.pattern if spec.kind == "pulse" else ((0.0, 1.0),)
        made = []
        with self._lock:
            for h in hands:
                if not test and spec.priority is not None:
                    last = self._last_cue.get(h)
                    if last is not None and now - last[0] < DEDUPE_S and last[1] >= spec.priority:
                        continue
                    self._last_cue[h] = (now, spec.priority)
                group = None
                for delay, dur in pattern:
                    self._id += 1
                    group = group or self._id
                    p = {"id": self._id, "group": group, "hand": h, "amplitude": round(amp, 3),
                         "duration_s": dur, "frequency_hz": 0.0, "delay_s": delay, "t": now, "cue": name}
                    self.pulses.append(p)
                    made.append(p)
            if made:
                self.counts["cues"][name] += 1
                self.last_cue = {"cue": name, "hands": sorted({p["hand"] for p in made}), "t": now,
                                 "source": source, "test": test}
            else:
                self.counts["suppressed"] += 1
        if made:
            self.publish(now)
        return made

    # -- levels ------------------------------------------------------------------------------------------------

    def _limited(self, row: dict, ees: list) -> dict:
        """{ee: kind} limited this row (the per-arm columns; else the global limited_by on every EE given)."""
        out = {}
        glob = None
        for e in ees:
            v = _num(row, f"{e}_limited_by")
            if math.isfinite(v):
                k = LIMITED[int(v)] if 0 <= int(v) < len(LIMITED) else "none"
            else:
                if glob is None:
                    g = _num(row, "limited_by")
                    glob = LIMITED[int(g)] if math.isfinite(g) and 0 <= int(g) < len(LIMITED) else "none"
                k = glob
            if k in self.limit_kinds:
                out[e] = k
        return out

    def _levels(self, now: float) -> dict:
        levels = {h: (0.0, None) for h in HANDS}

        def bump(h, v, cue):
            if h in levels and v > levels[h][0]:
                levels[h] = (v, cue)
        running = self.src.running()
        pairing = self.src.pairing() if running else {}
        hand_of = {e: h for h, e in pairing.items()}
        if running and self._on("limit") and self.src.armed():
            row, t = self.src.wbc_row()
            if row is not None and t is not None and now - t <= WBC_STALE_S:
                eng = self.src.engaged()
                ees = [e for h, e in pairing.items() if eng.get(h)]
                for e in self._limited(row, ees):
                    self._limit_seen[e] = now
            for e, t_seen in self._limit_seen.items():
                h = hand_of.get(e)
                if h is not None and now - t_seen <= LIMIT_HOLD_S:
                    bump(h, self.cues["limit"]["amplitude"], "limit")
        want_arms = running and (self._on("contact") or self._on("grip"))
        self._watch(pairing if want_arms else {})
        for e, h in hand_of.items():
            target = self.src.arm_target(e) if want_arms else None
            w = self.watches.get(target) if target else None
            st = self._contact.setdefault(e, GripContact())
            row, t = w.latest() if w is not None else (None, None)
            fresh = row if (row is not None and t is not None and now - t <= ARM_STALE_S) else None
            was = st.on
            on = st.update(fresh, now)
            if on and not was:
                self.cue("contact", (h,), source=e)
            if on and self._on("grip"):
                bump(h, self.cues["grip"]["amplitude"] * max(0.3, GripContact.force_fraction(fresh)), "grip")
        return levels

    def _watch(self, pairing: dict) -> None:
        """The arm streams the contact cues need (started on demand, closed when no longer needed)."""
        need = {t for t in (self.src.arm_target(e) for e in pairing.values()) if t}
        for t in list(self.watches):
            if t not in need:
                self.watches.pop(t).close()
        for t in need - set(self.watches):
            self.watches[t] = self.watch_factory(t).start()

    def update(self, now: Optional[float] = None, force: bool = False) -> None:
        """Recompute the levels (the node's loop, every step) and publish a line when one changed (the stream's
        keepalives come from ``wait``)."""
        now = self.clock() if now is None else now
        lv = self._levels(now)
        with self._lock:
            for h, (v, cue) in lv.items():
                self.levels[h], self.level_cue[h] = round(v, 3), cue
            changed = any((self.levels[h] > 0) != (self._sent_levels[h] > 0)
                          or abs(self.levels[h] - self._sent_levels[h]) >= LEVEL_STEP for h in HANDS)
        if force or changed:
            self.publish(now)

    def busy(self, now: Optional[float] = None) -> bool:
        """A level on or a pulse still repeated: the stream's keepalive is ``KEEPALIVE_S`` (else ``IDLE_S``)."""
        now = self.clock() if now is None else now
        with self._lock:
            return any(v > 0 for v in self.levels.values()) or any(now - p["t"] <= KEEP_S for p in self.pulses)

    # -- the line ------------------------------------------------------------------------------------------------

    def message(self, now: Optional[float] = None) -> dict:
        now = self.clock() if now is None else now
        with self._lock:
            hands = {}
            for h in HANDS:
                ps = [{"id": p["id"], "group": p["group"], "amplitude": p["amplitude"],
                       "duration_s": p["duration_s"], "frequency_hz": p["frequency_hz"], "delay_s": p["delay_s"],
                       "age_s": round(max(0.0, now - p["t"]), 3)}
                      for p in self.pulses if p["hand"] == h and now - p["t"] <= KEEP_S]
                hands[h] = {"level": self.levels[h], "frequency_hz": 0.0, "pulses": ps}
            return {"schema": SCHEMA, "epoch": self.epoch, "seq": self._seq, "ttl_s": TTL_S, "hands": hands}

    def publish(self, now: Optional[float] = None) -> dict:
        """A new line (a cue made, a level changed): the stream readers write it at once."""
        now = self.clock() if now is None else now
        with self._cond:
            self._seq += 1
            self._line = self.message(now)
            self._sent_levels = dict(self.levels)
            self.counts["published"] += 1
            self._cond.notify_all()
            return self._line

    def wait(self, after_seq: int, timeout: float) -> Optional[dict]:
        """The newest line once its seq is past ``after_seq``, or None after ``timeout`` s (a blocking call: the
        stream runs it in a thread)."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > after_seq, timeout):
                return None
            return self._line

    def stream_lines(self, stop: Callable[[], bool] = lambda: False):
        """The headset's stream (blocking generator, one JSON line each): the current state at once, every
        published line, and a fresh line after ``KEEPALIVE_S`` / ``IDLE_S`` without one (levels stay alive, the
        reader sees the link is up)."""
        with self._lock:
            self.readers += 1
        try:
            seq = -1
            while not stop():
                line = self.wait(seq, KEEPALIVE_S if self.busy() else IDLE_S) if seq >= 0 else None
                if line is None:
                    line = self.message()
                seq = line["seq"]
                self.counts["streamed"] += 1
                yield json.dumps(line, separators=(",", ":")) + "\n"
        finally:
            with self._lock:
                self.readers -= 1

    # -- the headset's side ----------------------------------------------------------------------------------------

    def on_frame(self, t_rx: float, frame: dict) -> None:
        """A headset frame (every frame): its ``haptics`` block, the ack of our pulses."""
        hb = frame.get("haptics") if isinstance(frame, dict) else None
        if not isinstance(hb, dict):
            if self.headset["capable"]:
                self.headset = {"capable": False, "last_id": None, "epoch": None, "connected": None}
            return
        last = hb.get("last_id")
        self.headset = {"capable": hb.get("schema") == SCHEMA, "last_id": last, "epoch": hb.get("epoch"),
                        "connected": hb.get("connected")}
        if hb.get("epoch") != self.epoch or not isinstance(last, int) or last <= self._acked:
            return
        with self._lock:
            for p in self.pulses:
                if self._acked < p["id"] <= last:
                    self.ack_ms.append(round((t_rx - p["t"]) * 1e3, 1))
            self._acked = last

    def close(self) -> None:
        for w in self.watches.values():
            w.close()
        self.watches.clear()

    def info(self, now: Optional[float] = None) -> dict:
        now = self.clock() if now is None else now
        acks = sorted(self.ack_ms)
        return {**self.config(), "epoch": self.epoch, "levels": dict(self.levels), "level_cue": dict(self.level_cue),
                "headset": dict(self.headset), "readers": self.readers, "last_cue": self.last_cue,
                "counts": {**self.counts, "cues": dict(self.counts["cues"])},
                "ack_ms": None if not acks else {"last": self.ack_ms[-1], "median": acks[len(acks) // 2],
                                                 "max": acks[-1], "n": len(acks)},
                "arms": {t: {"connected": w.t is not None and now - w.t <= ARM_STALE_S, "error": w.last_error}
                         for t, w in self.watches.items()},
                "contact": {e: c.on for e, c in self._contact.items()}}
