"""The client's side of the WBC's armed latch: the left controller's X and Y (README "Arming: Y and X"). Pure: no
sockets, the clock is passed in; ``teleop.TeleopClient`` runs the HTTP requests it asks for and sends the datagrams.

- **X** (``hands.left.buttons.primary``) is LEVEL-triggered and means **disable**: any accepted frame with X down
  disarms at once (the client sends ``armed: false``) and asks for ``POST /disable`` (again every
  ``disable_retry_s`` while X is held and the WBC row still shows it enabled). The teleop session stays open.
- **Y** (``hands.left.buttons.secondary``) **arms** (enabling first through ``POST /enable`` when the WBC is
  disabled) only on a deliberate press: Y seen released in a valid frame since the last disarm / disable /
  discontinuity, then held for ``y_frames`` (3) consecutive fresh valid frames with X up, both grips below
  ``grip_max`` (0.4), both sticks inside the native dead zone (0.18), and a fresh WBC row showing it enabled (or our
  enable just succeeded) without a fault. One arm per press: ``arm_seq`` += 1; Y must be released before the next.
  Anything failing during the press spoils it (release Y and press again).
- **Disarm** (``armed: false`` at once, then in every datagram): X; any discontinuity (``QuestGate``: reconnect,
  session, revision, gap); an invalid frame (unfocused, ...); stale, late or slow input (``QuestGate.fresh``); the
  LEFT controller inactive (the stop button gone); no command to send; the WBC's rows stale (newest older than
  ``wbc_stale_s``, 0.5 s); the WBC's own row showing it disarmed after it processed our arming datagram, or not
  enabled. Nothing re-arms on its own: every arm is a new Y press.
- ``inhibit`` (a reason; the teleop node's sim reset sets it): every Y press is refused with it (no arm, no enable).

**Tiered recovery** (``ResumeConfig``; README "Hold and re-grip"). Every disarm while armed is one of three tiers:

- **hold** (transient: stale / late / slow input, a gap, an invalid or unfocused frame, the left controller briefly
  inactive, the WBC's rows briefly stale, the WBC's own short-dropout disarm: a control tick overrun, its
  ``hold_timeout``, an unusable (not reset) estimator pose). The client disarms exactly as before (``armed: false``,
  the robot holds) but the state is ``hold: re-grip to resume``. Once the cause has cleared, **releasing and
  re-gripping a clutch hand resumes**: the engagement edge is the arming press (a new ``arm_seq`` above the floor, so
  the WBC's latch is unchanged and its ``arm_target_pos/rot`` check still refuses a jump; the core anchors that
  engagement to the commanded state). The edge must follow a release seen in a valid frame during the hold (a grip
  held through the dropout never resumes), with fresh input, X up, the left controller active, a fresh enabled WBC
  row without a fault and no ``inhibit``; the engagement itself must be accepted by the core (``confirm_resume``).
  A cause still present after ``grace_s`` (2 s) escalates to a disarm; so does a hold older than ``hold_max_s``.
- **disarmed** (serious: longer than the window, a reconnect / new session / operator-frame revision, an estimator
  reset (the WBC's reason, or ``est_reset`` changing in its rows during a hold) or pose jump, a fault, the WBC
  disabled (it never disables itself on a transient), any WBC disarm reason not known to be transient, a refused Y
  arm, ``max_refusals`` refused resumes). A Y press re-arms, as before (Y also arms from a hold).
  An invalid frame or the left controller inactive stays a present cause until a valid frame with the left
  controller active (``FRAME_CLASSES``): frames that keep arriving unfocused escalate after ``grace_s`` too.
- **deliberate** (X, ``inhibit`` such as a sim reset, ``disarm(..., deliberate=True)`` from the page): as before,
  ``disarmed``, never resumed.

``ResumeConfig.classes`` turns each transient class on or off (off: that cause disarms as before);
``ResumeConfig.enabled`` False restores "every disarm needs Y".  Events (``events``): ``("hold", reason, info)``,
``("resumed", arm_seq, info)``, ``("escalated", reason, info)`` in addition to ``("armed", seq)`` and
``("disarmed", reason)`` (a hold is not also reported as ``disarmed``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

WBC_DISABLED, WBC_ENABLING, WBC_FAULT = 0, 1, 5
WBC_ENABLED = (2, 3, 4)          # idle, tracking, holding (the WBC stream's state enum)
DEAD_ZONE = 0.18                 # the native stick dead zone (quest_env teleop::Intent)


def _hand(frame: dict, name: str) -> dict:
    h = (frame.get("hands") or {}).get(name)
    return h if isinstance(h, dict) else {}


def button(frame: dict, hand: str, name: str) -> bool:
    b = _hand(frame, hand).get("buttons")
    return isinstance(b, dict) and b.get(name) is True


def x_pressed(frame: dict) -> bool:
    return button(frame, "left", "primary")


def y_pressed(frame: dict) -> bool:
    return button(frame, "left", "secondary")


def _num(x) -> Optional[float]:
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) else None


def sticks_neutral(frame: dict, dead_zone: float = DEAD_ZONE) -> bool:
    """Both controllers' raw joysticks inside the dead zone (a missing or malformed stick is not neutral)."""
    for name in ("left", "right"):
        js = _hand(frame, name).get("joystick")
        if not isinstance(js, (list, tuple)) or len(js) < 2:
            return False
        v = [_num(x) for x in js[:2]]
        if any(x is None or abs(x) > dead_zone for x in v):
            return False
    return True


def grips_released(frame: dict, grip_max: float) -> bool:
    for name in ("left", "right"):
        g = _num(_hand(frame, name).get("grip"))
        if g is None or g >= grip_max:
            return False
    return True


@dataclass
class LatchConfig:
    y_frames: int = 3                 # Y held this many consecutive fresh valid frames
    grip_max: float = 0.4             # both grips below this to arm
    dead_zone: float = DEAD_ZONE
    robot_stale_s: float = 0.3        # the WBC row must be younger than this to arm
    wbc_stale_s: float = 0.5          # armed: the newest WBC row older than this disarms (0.3 tripped on WBC process
                                      # stalls on the Pi, 2026-11-21)
    disable_retry_s: float = 1.0      # X held: POST /disable again this often while the WBC still shows enabled
    enable_timeout_s: float = 2.0     # a press on a disabled WBC: cancelled if not enabled (and armed) by then


# the transient classes (ResumeConfig.classes): each cause that may hold instead of disarming
TRANSIENT_CLASSES = ("input_stale", "input_late", "input_gap", "input_invalid", "left_inactive", "wbc_stale",
                     "wbc_disarm", "no_command")
# the WBC's own disarm reasons (its /state disarm_reason) that are short dropouts: everything else is serious
WBC_TRANSIENT_PREFIXES = ("control tick overrun", "input stale", "estimator: ", "client: armed false")
SERIOUS_DISCONTINUITIES = ("session", "revision")   # a "reconnect" alone (the same headset session: the reader
# reopens a connection silent for 1 s) is a gap, transient: a Wi-Fi blackout over a second was a disarm before
# the classes judged per frame: present from the frame that showed them until a valid frame with the left controller
# active (between frames the tick cannot see them)
FRAME_CLASSES = ("input_invalid", "left_inactive")


def wbc_reason_transient(reason: Optional[str]) -> bool:
    """A WBC ``disarm_reason`` that is a short dropout (tick overrun, its hold_timeout, an unusable but not reset
    estimator pose). Unknown or missing: serious."""
    r = (reason or "").strip()
    return bool(r) and r.startswith(WBC_TRANSIENT_PREFIXES)


def input_class(why: str) -> str:
    """The class of a ``QuestGate.why_stale`` text: ``input_stale`` (no frame) or ``input_late`` (latency, rate,
    recovering, not judged yet)."""
    return "input_stale" if "stale" in (why or "") and "ms)" in (why or "") else "input_late"


@dataclass
class ResumeConfig:
    enabled: bool = True              # False: every disarm needs Y (the behaviour before the tiers)
    grace_s: float = 2.0              # a transient cause present longer than this escalates to a disarm
    hold_max_s: float = 60.0          # a hold not resumed within this (from its start) escalates
    max_refusals: int = 3             # resumes the WBC refuses (arm_far, estimator unusable, ...) before escalating
    classes: dict = field(default_factory=lambda: {c: True for c in TRANSIENT_CLASSES})

    def transient(self, cls: Optional[str]) -> bool:
        return bool(self.enabled and cls and self.classes.get(cls, False))

    def info(self) -> dict:
        return {"enabled": self.enabled, "grace_s": self.grace_s, "hold_max_s": self.hold_max_s,
                "max_refusals": self.max_refusals, "classes": dict(self.classes)}


class ArmLatch:
    """``on_frame`` per accepted frame, ``tick`` per control step; read ``armed`` / ``arm_seq``; drain ``events``
    (("armed", arm_seq) / ("disarmed", reason)) and ``requests`` ("enable" / "disable") after each call; report the
    HTTP results with ``enable_done`` / ``disable_done``; ``sent_arm`` with the seq of the arming datagram."""

    def __init__(self, cfg: LatchConfig = LatchConfig(), arm_seq: int = 0, resume: Optional[ResumeConfig] = None,
                 clutch_hands: tuple = ("left", "right")):
        self.cfg = cfg
        self.resume = resume if resume is not None else ResumeConfig()
        self.clutch_hands = tuple(clutch_hands)   # the hands whose grip engagement may resume a hold
        self.tier = "disarmed"            # armed | hold | disarmed (the README's tiers; X / inhibit: disarmed)
        # a hold: {"t", "cause" (the reason text), "class", "bad_since" (the cause present since; None: cleared),
        # "released" (clutch hands seen released in a valid frame since the hold began), "classifying" (waiting for
        # the WBC's disarm reason), "resumes" (resume attempts), "arm_seq" (the resume's seq waiting for the WBC)}
        self.hold: Optional[dict] = None
        self.resume_candidate: Optional[str] = None   # a hand whose engagement edge on this frame may resume
        self.resumes = 0                  # resumes completed (confirmed by the WBC row)
        self.wbc_seen_armed = False       # the WBC row showed armed since our current arm
        self.resuming = False             # the current arm is a resume (a refusal returns to the hold)
        self.armed = False
        self.arm_seq = arm_seq            # the newest press's number (the WBC arms only above its floor)
        self.reason = "not armed: press Y"  # the current state: armed, enabling, why not armed / why cancelled
        self.y_ready = False              # Y seen released in a valid frame since the last disarm / discontinuity
        self.y_count = 0
        # a deliberate press on a disabled WBC: {"t": when, "enabled_seq": None | the datagram seq when our enable
        # was reported done}; it waits through enabling and completes on a fresh enabled row after that seq
        self.pending: Optional[dict] = None
        self.just_enabled = False         # our POST /enable succeeded (until a disable / fault / disarm)
        self.x_down = False
        self.disabled_by_client = False   # the WBC is disabled because of X (the teleop node keeps running)
        self.arm_dgram_seq: Optional[int] = None   # the datagram seq that carried the arm
        self.events: list = []
        self.requests: list = []
        self.presses = 0                  # arms (one per press)
        self.inhibit: Optional[str] = None   # set: no press arms (or enables), e.g. "sim reset in progress"
        self._last_disable = -math.inf

    # -- transitions ----------------------------------------------------------------------------------------

    def set_floor(self, floor: int) -> None:
        """The WBC's arm_seq floor (``POST /session``): presses are numbered above it."""
        self.arm_seq = max(self.arm_seq, int(floor or 0))

    @property
    def pending_arm(self) -> bool:
        return self.pending is not None

    def _cancel(self, why: str) -> None:
        if self.pending is not None:
            self.pending = None
            self.reason = f"arm cancelled: {why}"
            self.events.append(("y_refused", why))   # a Y press that enabled but did not arm

    def _spoil(self, why: str = "") -> None:
        """The current press is spoiled (Y must be released and pressed again); a pending arm is cancelled."""
        self.y_ready, self.y_count = False, 0
        self._cancel(why or "the press was interrupted")

    def disarm(self, reason: str, cls: Optional[str] = None, now: Optional[float] = None,
               deliberate: bool = False) -> None:
        """Disarm now. ``cls``: the transient class of the cause (None: serious); with resume on and the class on,
        an armed latch goes to **hold** (one ("hold", reason, info) event) instead of disarmed (one ("disarmed",
        reason) event). A hold already in force: a transient cause updates it, a serious or deliberate one
        escalates it (("escalated", reason, info)). A pending arm is cancelled with this reason."""
        self._spoil(reason)
        if deliberate:
            cls = None
        if self.armed:
            self.armed = False
            self.arm_dgram_seq = None
            self.reason = reason
            if self.resuming and not self.wbc_seen_armed and self.hold is not None:
                # a resume the WBC has not confirmed yet, interrupted: a transient keeps the same hold, anything
                # else escalates it
                self.resuming = False
                if self.resume.transient(cls) and now is not None:
                    self.tier = "hold"
                    self._hold_cause(reason, cls, now)
                else:
                    self._escalate(reason)
                return
            self.resuming = False
            if self.resume.transient(cls) and now is not None:
                self._enter_hold(reason, cls, now)
            else:
                self.hold = None
                self.tier = "disarmed"
                self.events.append(("disarmed", reason))
            return
        if self.hold is not None:
            if self.resume.transient(cls) and now is not None:
                self._hold_cause(reason, cls, now)
            else:
                self._escalate(reason)

    # -- the hold tier --------------------------------------------------------------------------------------

    def _enter_hold(self, reason: str, cls: str, now: float) -> None:
        self.tier = "hold"
        self.hold = {"t": now, "cause": reason, "class": cls, "bad_since": now, "released": set(),
                     "classifying": False, "resumes": 0, "arm_seq": None, "refusals": 0, "est_reset": None,
                     "frame_bad": (reason, cls) if cls in FRAME_CLASSES else None}
        self.reason = f"hold: {reason} (re-grip to resume)"
        self.events.append(("hold", reason, self.hold_info(now)))

    def _hold_cause(self, reason: str, cls: str, now: Optional[float]) -> None:
        """A transient cause during a hold: the hold continues; its cause is present (again) from now."""
        h = self.hold
        h["cause"], h["class"] = reason, cls
        if cls in FRAME_CLASSES:
            h["frame_bad"] = (reason, cls)        # present until a valid frame with the left controller active
        if h["bad_since"] is None and now is not None:
            h["bad_since"] = now
        self.reason = f"hold: {reason} (re-grip to resume)"

    def _escalate(self, reason: str) -> None:
        """A hold becomes a disarm (Y re-arms)."""
        h, self.hold = self.hold, None
        self.tier = "disarmed"
        self.resuming = False
        self.reason = f"disarmed: {reason} (press Y)"
        self.events.append(("escalated", reason, {"cause": h["cause"] if h else None,
                                                  "class": h["class"] if h else None}))

    def hold_info(self, now: float) -> Optional[dict]:
        h = self.hold
        if h is None:
            return None
        return {"cause": h["cause"], "class": h["class"], "since_s": round(now - h["t"], 3),
                "cause_present_s": None if h["bad_since"] is None else round(now - h["bad_since"], 3),
                "resumable": h["bad_since"] is None and not h["classifying"],
                "released": sorted(h["released"]), "resumes": h["resumes"], "refusals": h["refusals"],
                "grace_s": self.resume.grace_s, "last_refused": h.get("refused")}

    def tier_info(self, now: float) -> dict:
        """``/state``: the tier, why, and the hold (None unless holding)."""
        return {"tier": self.tier, "why": self.reason, "hold": self.hold_info(now), "resumes": self.resumes,
                "config": self.resume.info()}

    def wbc_disarm_reason(self, reason: Optional[str], now: float) -> None:
        """The WBC's ``disarm_reason`` for a disarm we saw in its row (asked with the "why" request): a short
        dropout keeps the hold; anything else (or no answer) escalates."""
        h = self.hold
        if h is None or not h["classifying"]:
            return
        h["classifying"] = False
        if not (reason or "").strip() and self.resume.transient("wbc_disarm"):
            # no reason: the WBC already moved on (armed again, or its /state not answering on a lossy link). Still
            # a hold: a serious cause is caught on its own (an estimator reset in the rows, the WBC disabled or
            # faulted), and the re-grip still passes the WBC's arming checks
            h["cause"] = "the WBC disarmed (no reason reported)"
            self.reason = f"hold: {h['cause']} (re-grip to resume)"
        elif wbc_reason_transient(reason) and self.resume.transient("wbc_disarm"):
            h["cause"] = f"the WBC disarmed: {reason}"
            self.reason = f"hold: {h['cause']} (re-grip to resume)"
        else:
            self._escalate(f"the WBC disarmed: {reason or 'reason unknown'}")

    def confirm_resume(self, engaged: bool, now: float) -> None:
        """After the core saw the frame that made ``resume_candidate``: resume if it accepted the engagement."""
        hand, self.resume_candidate = self.resume_candidate, None
        if hand is None or self.hold is None:
            return
        if not engaged:
            self.reason = f"hold: {self.hold['cause']} (the engagement was refused: release and re-grip)"
            return
        h = self.hold
        h["resumes"] += 1
        h["released"].discard(hand)
        self.arm_seq += 1
        self.presses += 1
        self.armed, self.tier = True, "armed"
        self.reason = "armed (resumed)"
        self.arm_dgram_seq = None
        self.resuming, self.wbc_seen_armed = True, False
        h["arm_seq"] = self.arm_seq
        self.events.append(("resumed", self.arm_seq, {"hand": hand, "after_s": round(now - h["t"], 3),
                                                     "cause": h["cause"], "class": h["class"]}))

    def _resume_check(self, frame: dict, row, row_t, now: float, fresh: bool) -> None:
        """A valid frame during a hold (X up, the left controller active): note released clutch hands; an
        engagement edge after a release, with the cause cleared and the arming preconditions met, is a resume
        candidate (``confirm_resume`` after the core saw the frame)."""
        h = self.hold
        hands = ((frame.get("control_intent") or {}).get("hands")) or {}
        edge = None
        for name in self.clutch_hands:
            ih = hands.get(name) or {}
            if not ih.get("valid"):
                continue
            if not ih.get("engaged"):
                h["released"].add(name)
            elif name in h["released"] and edge is None:
                edge = name
        if edge is None:
            return
        h["released"].discard(edge)       # this edge is used, whatever follows
        why = None
        if h["bad_since"] is not None or h["classifying"]:
            why = "the cause has not cleared yet"
        elif self.inhibit:
            why = self.inhibit
        elif not fresh:
            why = "headset input not fresh"
        else:
            st, w = self._wbc(row, row_t, now)
            if w:
                why = w
            elif st != WBC_FAULT and st in WBC_ENABLED:
                why = None
            else:
                why = "the WBC is not enabled"
        if why:
            h["refused"] = why
            self.reason = f"hold: {h['cause']} (resume refused: {why}; release and re-grip)"
            return
        h["refused"] = None
        self.resume_candidate = edge

    def _arm(self) -> None:
        self.arm_seq += 1
        self.presses += 1
        self.armed, self.reason = True, "armed"
        self.tier, self.hold = "armed", None
        self.resuming, self.wbc_seen_armed = False, False
        self.arm_dgram_seq = None
        self.pending = None
        self._spoil()                          # one arm per press
        self.events.append(("armed", self.arm_seq))

    def sent_arm(self, seq: int) -> None:
        """The datagram ``seq`` that first carried this arm (the WBC row's ``seq`` tells when it was processed)."""
        if self.armed and self.arm_dgram_seq is None:
            self.arm_dgram_seq = seq

    def enable_done(self, ok: bool, detail: str = "", seq: Optional[int] = None) -> None:
        """Our ``POST /enable`` answered. ``seq``: the newest datagram seq sent when the answer was seen: a pending arm
        completes only on a WBC row that processed a later datagram (so it reflects the enable)."""
        if ok:
            self.just_enabled, self.disabled_by_client = True, False
            if self.pending is not None:
                self.pending["enabled_seq"] = -1 if seq is None else int(seq)
        elif self.pending is not None:
            self._cancel(f"enable refused: {detail}")
        else:
            self.reason = f"enable refused: {detail}"

    def disable_done(self, ok: bool, detail: str = "") -> None:
        if not ok:
            self.reason = f"disable failed: {detail}"

    def _request(self, what: str) -> None:
        if what not in self.requests:
            self.requests.append(what)

    # -- inputs ---------------------------------------------------------------------------------------------

    def _wbc(self, row: Optional[dict], row_t: Optional[float], now: float) -> tuple[Optional[int], Optional[str]]:
        """(the WBC state from a fresh row, None) or (None, why not usable)."""
        if row is None or row_t is None:
            return None, "no WBC state"
        if now - row_t > self.cfg.robot_stale_s:
            return None, f"WBC state stale ({now - row_t:.2f} s)"
        s = _num(row.get("state"))
        if s is None:
            return None, "no WBC state"
        return int(s), None

    def _preconditions(self, frame: dict, row, row_t, now: float, fresh: bool) -> Optional[str]:
        if self.inhibit:
            return self.inhibit
        if not fresh:
            return "headset input not fresh"
        if not grips_released(frame, self.cfg.grip_max):
            return "a grip is pressed"
        if not sticks_neutral(frame, self.cfg.dead_zone):
            return "a stick is deflected"
        st, why = self._wbc(row, row_t, now)
        if why:
            return why
        if st == WBC_FAULT:
            return "the WBC is in a fault (X disables and clears it)"
        if st == WBC_ENABLING:
            return "the WBC is enabling"
        return None

    def on_frame(self, frame: dict, verdict, row: Optional[dict], row_t: Optional[float], now: float,
                 fresh: bool) -> None:
        """One accepted-or-not frame from ``QuestGate.check`` (in order)."""
        if not verdict.accept:
            return
        self.resume_candidate = None
        if x_pressed(frame):                       # level-triggered: any accepted frame, valid or not
            self.disarm("X: disabled", deliberate=True)   # (a pending arm: "arm cancelled: X: disabled"; a hold
                                                          # escalates: deliberate, never resumed)
            if not self.reason.startswith("arm cancelled"):
                self.reason = "X: disabled (Y re-enables and arms)"
            self.just_enabled = False
            st, _ = self._wbc(row, row_t, now)
            if not self.x_down or (now - self._last_disable >= self.cfg.disable_retry_s and st != WBC_DISABLED):
                self._request("disable")
                self._last_disable = now
            self.x_down = True
            self.disabled_by_client = True
            return
        self.x_down = False
        serious = bool(set((verdict.reason or "").split(",")) & set(SERIOUS_DISCONTINUITIES))
        if verdict.discontinuity:
            self.disarm(f"headset discontinuity ({verdict.reason})", None if serious else "input_gap", now)
        if not verdict.valid:
            self.disarm(f"headset input invalid ({verdict.reason})", None if serious else "input_invalid", now)
            return
        if not _hand(frame, "left").get("active"):
            self.disarm("the left controller is not active (X / Y unavailable)", "left_inactive", now)
            return
        if self.hold is not None:                  # a valid frame, X up, the left controller active: a re-grip?
            self.hold["frame_bad"] = None
            self._resume_check(frame, row, row_t, now, fresh)
        if self.pending is not None:               # a press already made: Y released or held, it waits
            if not y_pressed(frame):
                self.y_ready, self.y_count = True, 0
            self._pending_step(frame, row, row_t, now, fresh)
            return
        if not y_pressed(frame):
            self.y_ready, self.y_count = True, 0
            return
        if not self.y_ready:                       # held since before a disarm, or already used for an arm
            return
        why = self._preconditions(frame, row, row_t, now, fresh)
        if why:
            self.reason = f"not armed: {why} (release Y and press again)"
            self._spoil()
            self.events.append(("y_refused", why))   # logged and felt (the operator can't see the page)
            return
        self.y_count += 1
        if self.y_count < self.cfg.y_frames:
            return
        st, _ = self._wbc(row, row_t, now)
        self.y_ready, self.y_count = False, 0     # this press is used, whatever follows
        if st in WBC_ENABLED:
            self._arm()
        elif st == WBC_DISABLED:                  # enable, then arm when the WBC shows it enabled
            self.pending = {"t": now, "enabled_seq": None}
            self.reason = "enabling the WBC…"
            self._request("enable")

    def _pending_step(self, frame: Optional[dict], row, row_t, now: float, fresh: bool) -> None:
        """A pending arm: cancelled on a timeout or a fault row; completed on the first fresh row that processed a
        datagram sent after our enable succeeded and shows the WBC enabled, if the other preconditions hold then."""
        p = self.pending
        if self.inhibit:                           # e.g. a sim reset started: never complete an arm during it
            self._cancel(self.inhibit)
            return
        if now - p["t"] > self.cfg.enable_timeout_s:
            self._cancel(f"the WBC did not enable within {self.cfg.enable_timeout_s:g} s")
            return
        st, why = self._wbc(row, row_t, now)
        if st == WBC_FAULT:
            self._cancel("the WBC is in a fault (X disables and clears it)")
            return
        if p["enabled_seq"] is None or why is not None or st not in WBC_ENABLED:
            return                                 # still enabling (or the row is not usable yet): wait
        seq = _num(row.get("seq"))
        if seq is not None and seq <= p["enabled_seq"]:
            return                                 # this row predates our enable
        if frame is None:
            return                                 # complete on a frame: its input is checked
        if not fresh:
            self._cancel("headset input not fresh")
        elif not grips_released(frame, self.cfg.grip_max):
            self._cancel("a grip is pressed")
        elif not sticks_neutral(frame, self.cfg.dead_zone):
            self._cancel("a stick is deflected")
        else:
            self._arm()

    def tick(self, now: float, fresh: bool, why_stale: str, row: Optional[dict], row_t: Optional[float]) -> None:
        """Per control step: stale / late / slow input disarms (or holds); so does the WBC's own row disagreeing. A
        hold: its cause cleared or still present (escalating after ``grace_s``), its age (``hold_max_s``)."""
        bad = None                                 # (reason, class) of a transient cause present now
        if not fresh:
            why = why_stale or "headset input stale"
            if self.armed:
                self.disarm(why, input_class(why), now)
            else:
                self._spoil(why)
            bad = (why, input_class(why))
        else:
            if self.pending is not None:           # the timeout and a fault row, between frames too
                self._pending_step(None, row, row_t, now, fresh)
            if row is None or row_t is None or now - row_t > self.cfg.wbc_stale_s:
                why = ("WBC state stale" if row is None or row_t is None else f"WBC state stale ({now - row_t:.2f} s)")
                if self.armed:                     # the WBC stream is gone or late: we cannot see what it does
                    self.disarm(why, "wbc_stale", now)
                bad = (why, "wbc_stale")
            else:
                bad = self._row_step(row, now)
        self._hold_step(now, bad, row, row_t)

    def _row_step(self, row: dict, now: float) -> Optional[tuple]:
        """A fresh WBC row: fault / disabled / its own disarm. Returns a transient cause present now, or None."""
        st = _num(row.get("state"))
        if st is not None and int(st) == WBC_FAULT:
            self.just_enabled = False
        if self.hold is not None and st is not None and int(st) not in WBC_ENABLED:
            self._escalate("the WBC is " + {WBC_DISABLED: "disabled", WBC_FAULT: "in a fault"}.get(int(st),
                                                                                                   "not enabled"))
            return None
        if not self.armed or self.arm_dgram_seq is None:
            return None
        # only rows that reflect our arming datagram (their seq: the newest datagram the WBC processed) count: an
        # older row may predate our enable
        a, seq = _num(row.get("armed")), _num(row.get("seq"))
        if seq is None or seq < self.arm_dgram_seq:
            return None
        if st is not None and int(st) not in WBC_ENABLED:
            self.disarm("the WBC is " + {WBC_DISABLED: "disabled", WBC_FAULT: "in a fault"}.get(int(st), "not enabled"))
            return None
        if a is not None and a == 1.0:
            self.wbc_seen_armed = True
            if self.resuming:
                self.resuming = False
                self.resumes += 1
                self.hold = None
            return None
        if a is not None and a == 0.0:
            if not self.wbc_seen_armed:            # our arm never took: the WBC refused it (too far, not live, ...)
                if self.resuming and self.hold is not None:
                    h = self.hold
                    self.armed, self.resuming = False, False
                    self.arm_dgram_seq = None
                    self.tier = "hold"
                    h["refusals"] += 1
                    self.reason = (f"hold: {h['cause']} (the WBC refused the resume: release and re-grip)")
                    self.events.append(("resume_refused", self.arm_seq, {"cause": h["cause"],
                                                                         "refusals": h["refusals"]}))
                    if h["refusals"] >= self.resume.max_refusals:
                        self._escalate(f"the WBC refused {h['refusals']} resumes (its /state: arm_far, estimator, "
                                       "...)")
                    return None
                self.disarm("the WBC refused the arm (its /state: arm_far, estimator, ...)")
                return None
            self.disarm("the WBC disarmed (its /state disarm_reason)", "wbc_disarm", now)
            if self.hold is not None:              # the reason decides: a short dropout holds, anything else not
                self.hold["classifying"] = True
                self._request("why")
            return None
        return None

    def _hold_step(self, now: float, bad: Optional[tuple], row, row_t) -> None:
        h = self.hold
        if h is None:
            return
        if self.inhibit:
            self._escalate(self.inhibit)
            return
        er = _num((row or {}).get("est_reset"))   # the WBC's estimator epoch: a reset during a hold is serious
        if er is not None and row_t is not None and now - row_t <= self.cfg.wbc_stale_s:
            if h["est_reset"] is None:
                h["est_reset"] = er
            elif er != h["est_reset"]:
                self._escalate("estimator reset (world frame changed) during the hold")
                return
        if bad is None and h["frame_bad"] is not None:
            bad = h["frame_bad"]                   # invalid / unfocused frames or the left controller inactive
        if bad is not None and self.resume.transient(bad[1]):
            if h["bad_since"] is None:
                h["bad_since"] = now
            h["cause"], h["class"] = bad
            self.reason = f"hold: {bad[0]} (re-grip to resume)"
        elif bad is not None:                      # a cause whose class is off: as before, a disarm
            self._escalate(bad[0])
            return
        elif h["bad_since"] is not None:
            h["bad_since"] = None                  # cleared: a release and a re-grip resume
            self.reason = f"hold: {h['cause']} (cleared: re-grip to resume)"
        if h["bad_since"] is not None and now - h["bad_since"] > self.resume.grace_s:
            self._escalate(f"{h['cause']} for over {self.resume.grace_s:g} s")
        elif h["classifying"] and now - h["t"] > max(self.resume.grace_s, 0.0):
            self._escalate("the WBC disarmed: its reason not known in time")
        elif now - h["t"] > self.resume.hold_max_s:
            self._escalate(f"not resumed within {self.resume.hold_max_s:g} s")
