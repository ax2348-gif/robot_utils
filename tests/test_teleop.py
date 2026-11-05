import os
import signal
import subprocess
import sys
import textwrap
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from robot_utils.quest import parse_frame
from robot_utils.teleop import (ENGAGED, ESTOP, FAULT, INVALID, NEEDS_RELEASE, NO_INPUT, RELEASED,
                                REQUESTED, FaultAction, HandClutch, Monitor, Pose, PoseClutch, TeleopApp,
                                TeleopRunner, clamp_rotation, rotation_from_vector, rotation_vector)

from _quest_records import hand, record, rot_z9

SRC = str(Path(__file__).resolve().parents[1] / "src")
TESTS = str(Path(__file__).resolve().parent)


def H(**kw):
    return parse_frame(record(1, right=hand(**kw)), 0.0).intent.hands["right"]


class HandClutchTest(unittest.TestCase):
    def test_release_then_grip_engages(self):
        c = HandClutch("right")
        self.assertEqual(c.update(None).status, NO_INPUT)
        self.assertEqual(c.update(H(), "k").status, RELEASED)
        s = c.update(H(engaged=True, engagement_id=1, dp=(0.1, 0, 0), gripper=0.4), "k")
        self.assertEqual(s.status, ENGAGED)
        self.assertTrue(s.engaged and s.just_engaged)
        self.assertEqual(s.trigger, 0.4)
        s2 = c.update(H(engaged=True, engagement_id=1), "k")
        self.assertTrue(s2.engaged and not s2.just_engaged)
        self.assertEqual(s2.engagement, s.engagement)
        s3 = c.update(H(), "k")
        self.assertTrue(s3.just_released and not s3.engaged)

    def test_mid_engagement_attach_needs_release(self):
        c = HandClutch("right")
        s = c.update(H(engaged=True, engagement_id=7, dp=(0.3, 0, 0)), "k")
        self.assertEqual(s.status, NEEDS_RELEASE)
        np.testing.assert_array_equal(s.delta_position, 0)
        self.assertEqual(c.update(H(engaged=True, engagement_id=8), "k").status, NEEDS_RELEASE)
        c.update(H(), "k")
        self.assertEqual(c.update(H(engaged=True, engagement_id=9), "k").status, ENGAGED)

    def test_gap_invalid_and_key_change_disarm(self):
        for interrupt in ("gap", "invalid", "key"):
            with self.subTest(interrupt):
                c = HandClutch("right")
                c.update(H(), "k")
                first = c.update(H(engaged=True, engagement_id=1), "k")
                self.assertTrue(first.engaged)
                if interrupt == "gap":
                    self.assertTrue(c.update(None, "k").just_released)
                elif interrupt == "invalid":
                    s = c.update(H(valid=False, engaged=True, engagement_id=1), "k")
                    self.assertEqual(s.status, INVALID)
                    self.assertIsNone(s.trigger)
                key = "k2" if interrupt == "key" else "k"
                s = c.update(H(engaged=True, engagement_id=1), key)
                self.assertEqual(s.status, NEEDS_RELEASE)   # same engagement is not re-adopted
                c.update(H(), key)
                s = c.update(H(engaged=True, engagement_id=2), key)
                self.assertTrue(s.engaged)
                self.assertNotEqual(s.engagement, first.engagement)

    def test_needs_release_flag_from_app_disarms(self):
        c = HandClutch("right")
        self.assertEqual(c.update(H(needs_release=True), "k").status, NEEDS_RELEASE)
        self.assertEqual(c.update(H(engaged=True, engagement_id=1), "k").status, NEEDS_RELEASE)

    def test_disarm(self):
        c = HandClutch("right")
        c.update(H(), "k")
        self.assertTrue(c.update(H(engaged=True, engagement_id=1), "k").engaged)
        c.disarm()
        self.assertEqual(c.update(H(engaged=True, engagement_id=1), "k").status, NEEDS_RELEASE)


class PoseClutchTest(unittest.TestCase):
    def engaged(self, c, eid=1, **kw):
        c.update(H(), "k")
        return c.update(H(engaged=True, engagement_id=eid, **kw), "k")

    def test_anchor_measured_once_per_engagement(self):
        hc, pc = HandClutch("right"), PoseClutch()
        calls = []
        anchor = Pose(np.array([1.0, 2, 3]), np.eye(3))

        def measure():
            calls.append(1)
            return anchor

        s = self.engaged(hc, dp=(0.1, 0, 0), dr=rot_z9(0.2))
        t = pc.update(s, measure)
        np.testing.assert_allclose(t.position, [1.1, 2, 3])
        np.testing.assert_allclose(t.rotation, np.array(rot_z9(0.2)).reshape(3, 3), atol=1e-12)
        pc.update(hc.update(H(engaged=True, engagement_id=1, dp=(0.2, 0, 0)), "k"), measure)
        self.assertEqual(len(calls), 1)
        self.assertIsNone(pc.update(hc.update(H(), "k"), measure))
        self.assertIsNone(pc.anchor)
        pc.update(hc.update(H(engaged=True, engagement_id=2), "k"), measure)
        self.assertEqual(len(calls), 2)

    def test_operator_to_robot_and_clamps(self):
        rz90 = np.array(rot_z9(np.pi / 2)).reshape(3, 3)
        pc = PoseClutch(rz90, max_offset_m=0.05, max_angle_rad=0.1)
        s = self.engaged(HandClutch("right"), dp=(1.0, 0, 0), dr=rot_z9(0.5))
        t = pc.update(s, Pose.identity())
        np.testing.assert_allclose(t.position, [0, 0.05, 0], atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(rotation_vector(t.rotation)), 0.1)
        with self.assertRaises(ValueError):
            PoseClutch(np.diag([1.0, 1, -1]))

    def test_bad_measure_raises(self):
        s = self.engaged(HandClutch("right"))
        with self.assertRaises(ValueError):
            PoseClutch().update(s, lambda: Pose(np.array([np.nan, 0, 0]), np.eye(3)))

    def test_frozen_frame_keeps_world_target(self):
        def planar(x, y, yaw):
            return Pose(np.array([x, y, 0.0]), np.array(rot_z9(yaw)).reshape(3, 3))
        hc, pc = HandClutch("right"), PoseClutch()
        anchor = Pose(np.array([0.3, 0.0, 0.2]), np.eye(3))
        s = self.engaged(hc)
        t0 = pc.update(s, anchor, frame=planar(0, 0, 0))
        np.testing.assert_allclose(t0.position, anchor.position)
        # base drives 0.1 m forward and turns 90 deg; no operator motion
        s = hc.update(H(engaged=True, engagement_id=1), "k")
        base = planar(0.1, 0, np.pi / 2)
        t = pc.update(s, anchor, frame=base)
        np.testing.assert_allclose((base @ t).position, anchor.position, atol=1e-12)   # same world point
        np.testing.assert_allclose(t.position, [0, -0.2, 0.2], atol=1e-12)
        with self.assertRaises(ValueError):          # frame dropped mid-engagement
            pc.update(s, anchor)
        live = PoseClutch()
        live.update(self.engaged(HandClutch("right")), anchor)
        np.testing.assert_allclose(live.target.position, anchor.position)   # live: base-relative

    def test_pose_algebra(self):
        a = Pose(np.array([1.0, 2, 3]), np.array(rot_z9(0.4)).reshape(3, 3))
        b = Pose(np.array([-0.5, 0.2, 0]), np.array(rot_z9(-1.0)).reshape(3, 3))
        np.testing.assert_allclose((a @ b).matrix(), a.matrix() @ b.matrix(), atol=1e-12)
        np.testing.assert_allclose((a.inverse() @ a).matrix(), np.eye(4), atol=1e-12)

    def test_rotation_helpers(self):
        for rv in ([0, 0, 0], [0.3, -0.2, 0.1], [0, np.pi - 1e-9, 0], [1e-9, 0, 0]):
            np.testing.assert_allclose(rotation_vector(rotation_from_vector(np.array(rv))), rv, atol=1e-6)
        r = rotation_from_vector(np.array([0, 0, 2.0]))
        np.testing.assert_allclose(rotation_vector(clamp_rotation(r, 0.5)), [0, 0, 0.5], atol=1e-12)


class MonitorTest(unittest.TestCase):
    def test_latest_only_rate_limited_and_never_blocks(self):
        seen, gate = [], threading.Event()

        def slow(x):
            seen.append(x)
            gate.wait(1.0)

        with Monitor(slow, rate_hz=1000) as m:
            t = time.monotonic()
            for i in range(1000):
                m.publish(i)
            self.assertLess(time.monotonic() - t, 0.1)   # publish does not wait for fn
            time.sleep(0.05)
            gate.set()
            time.sleep(0.05)
        self.assertLessEqual(len(seen), 3)
        self.assertEqual(seen[-1], 999)
        self.assertEqual(m.stats.published, 1000)

    def test_errors_disable_without_raising(self):
        def bad(x):
            raise RuntimeError("render")
        m = Monitor(bad, rate_hz=1000, max_errors=3).start()
        for i in range(20):
            m.publish(i)
            time.sleep(0.005)
        m.close()
        self.assertTrue(m.stats.failed)
        self.assertEqual(m.stats.errors, 3)

    def test_close_with_stuck_fn_is_bounded(self):
        m = Monitor(lambda x: time.sleep(5), rate_hz=100).start()
        m.publish(1)
        time.sleep(0.02)
        t = time.monotonic()
        m.close(timeout_s=0.1)
        self.assertLess(time.monotonic() - t, 0.5)


class Source:
    """Scripted frame source: ``script(tick)`` returns (record or None, age_s)."""

    def __init__(self, script):
        self.script, self.tick, self.calls = script, 0, 0

    def latest(self):
        self.calls += 1
        rec, age = self.script(self.calls)
        return None if rec is None else parse_frame(rec, time.monotonic() - age)


def grip_script(engage_at=3, release_at=None, eid=1, fresh=True):
    def script(n):
        if not fresh:
            return record(n), 1.0
        engaged = n >= engage_at and (release_at is None or n < release_at)
        return record(n, right=hand(engaged=engaged, engagement_id=eid if engaged else 0)), 0.0
    return script


class Recorder(TeleopApp):
    rate_hz = 500.0

    def __init__(self, stop_at=20, raise_in=None, raise_times=1, fault_action=FaultAction.STOP):
        self.events, self.stop_at = [], stop_at
        self.raise_in, self.raise_times, self.fault_action = raise_in, raise_times, fault_action
        self.faults, self.emergency = [], []

    def _maybe_raise(self, phase):
        if self.raise_in == phase and self.raise_times > 0:
            self.raise_times -= 1
            raise RuntimeError(f"boom in {phase}")

    def _log(self, ctx, name):
        if not self.events or self.events[-1] != name:
            self.events.append(name)
        if ctx.tick >= self.stop_at:
            ctx.request_stop("done")

    def start(self, ctx):
        self.events.append("start")
        self._maybe_raise("start")

    def engage(self, ctx, inputs):
        self.events.append("engage")
        self._maybe_raise("engage")

    def disengage(self, ctx, inputs):
        self.events.append("disengage")
        self._maybe_raise("disengage")

    def active_step(self, ctx, inputs):
        self.last_inputs = inputs
        self._log(ctx, "active")
        self._maybe_raise("active_step")

    def hold_step(self, ctx, inputs):
        self.last_inputs = inputs
        self._log(ctx, "hold")
        self._maybe_raise("hold_step")

    def stop(self, ctx, result):
        self.events.append("stop")
        self.result_seen = result

    def on_fault(self, ctx, fault):
        self.faults.append(fault)
        return self.fault_action

    def emergency_stop(self, reason):
        self.emergency.append((reason, threading.current_thread().name))


def run(app, script, **kw):
    kw.setdefault("install_signals", False)
    kw.setdefault("overrun_fault_s", None)
    return TeleopRunner(app, Source(script), **kw).run()


class RunnerTest(unittest.TestCase):
    def test_lifecycle(self):
        app = Recorder(stop_at=30)
        res = run(app, grip_script(engage_at=5, release_at=15))
        self.assertEqual(app.events, ["start", "hold", "engage", "active", "disengage", "hold", "stop"])
        self.assertTrue(res.ok)
        self.assertEqual((res.kind, res.reason), (REQUESTED, "done"))
        self.assertEqual(res.stats.engagements, 1)
        self.assertEqual(res.stats.active_ticks, 10)
        self.assertIs(app.result_seen.kind, REQUESTED)

    def test_stale_input_never_engages_and_zeroes(self):
        app = Recorder(stop_at=10)
        run(app, lambda n: (record(n, right=hand(engaged=True, engagement_id=1, gripper=1.0),
                                   base_velocity=(1, 0, 0)), 1.0))
        self.assertEqual(app.events, ["start", "hold", "stop"])
        i = app.last_inputs
        self.assertEqual(i.status, "stale")
        self.assertFalse(i.live)
        self.assertIsNone(i.frame)
        self.assertIsNone(i.hands["right"].trigger)
        np.testing.assert_array_equal(i.twist, 0)

    def test_unfocused_and_no_frame(self):
        for status, script in (("unfocused", lambda n: (record(n, focused=False, base_velocity=(1, 0, 0)), 0.0)),
                               ("no-frame", lambda n: (None, 0.0)),
                               ("no-intent", lambda n: (record(n, intent=False), 0.0))):
            with self.subTest(status):
                app = Recorder(stop_at=5)
                run(app, script)
                self.assertEqual(app.last_inputs.status, status)
                np.testing.assert_array_equal(app.last_inputs.twist, 0)

    def test_input_loss_mid_engagement_disengages_and_needs_regrip(self):
        def script(n):
            if 10 <= n < 13:
                return None, 0.0                               # stream drops
            engaged = n >= 3
            return record(n, right=hand(engaged=engaged, engagement_id=1 if engaged else 0)), 0.0
        app = Recorder(stop_at=25)
        run(app, script)
        # same engagement continues after the gap: must not resume
        self.assertEqual(app.events, ["start", "hold", "engage", "active", "disengage", "hold", "stop"])
        self.assertEqual(app.last_inputs.hands["right"].status, NEEDS_RELEASE)

    def test_twist_passes_when_live(self):
        app = Recorder(stop_at=5)
        run(app, lambda n: (record(n, base_velocity=(0.3, -0.1, 0.2)), 0.0))
        np.testing.assert_allclose(app.last_inputs.twist, [0.3, -0.1, 0.2])

    def test_fault_stop(self):
        app = Recorder(raise_in="active_step")
        res = run(app, grip_script())
        self.assertEqual(res.kind, FAULT)
        self.assertEqual(res.fault.phase, "active_step")
        self.assertEqual(app.events[-1], "stop")
        self.assertNotIn("disengage", app.events)   # stop() handles shutdown directly

    def test_fault_hold_disengages_and_requires_regrip(self):
        app = Recorder(stop_at=20, raise_in="active_step", fault_action=FaultAction.HOLD)
        res = run(app, grip_script(engage_at=3))
        self.assertEqual(res.kind, REQUESTED)
        self.assertEqual(app.events, ["start", "hold", "engage", "active", "disengage", "hold", "stop"])
        self.assertEqual(app.last_inputs.hands["right"].status, NEEDS_RELEASE)

    def test_fault_ignore_then_consecutive_limit(self):
        app = Recorder(stop_at=50, raise_in="hold_step", raise_times=2, fault_action=FaultAction.IGNORE)
        self.assertEqual(run(app, grip_script(engage_at=99)).kind, REQUESTED)
        app = Recorder(stop_at=50, raise_in="hold_step", raise_times=99, fault_action=FaultAction.IGNORE)
        res = run(app, grip_script(engage_at=99), max_consecutive_faults=3)
        self.assertEqual(res.kind, FAULT)
        self.assertIn("3 consecutive", res.reason)

    def test_start_fault_still_stops(self):
        app = Recorder(raise_in="start", fault_action=FaultAction.IGNORE)
        res = run(app, grip_script())
        self.assertEqual(res.kind, FAULT)
        self.assertEqual(app.events, ["start", "stop"])

    def test_engage_fault_hold_calls_disengage(self):
        app = Recorder(stop_at=15, raise_in="engage", fault_action=FaultAction.HOLD)
        run(app, grip_script(engage_at=3))
        self.assertEqual(app.events[:4], ["start", "hold", "engage", "disengage"])
        self.assertNotIn("active", app.events)

    def test_disengage_fault_stops(self):
        app = Recorder(stop_at=40, raise_in="disengage", fault_action=FaultAction.IGNORE)
        res = run(app, grip_script(engage_at=3, release_at=6))
        self.assertEqual((res.kind, res.fault.phase), (FAULT, "disengage"))

    def test_on_fault_bad_return_stops(self):
        app = Recorder(raise_in="hold_step", fault_action="hold")
        self.assertEqual(run(app, grip_script(engage_at=99)).kind, FAULT)

    def test_stop_error_reported(self):
        class Bad(Recorder):
            def stop(self, ctx, result):
                raise OSError("bus gone")
        res = run(Bad(stop_at=3), grip_script())
        self.assertIsInstance(res.stop_error, OSError)
        self.assertFalse(res.ok)

    def test_overrun_fault(self):
        class Slow(Recorder):
            def hold_step(self, ctx, inputs):
                super().hold_step(ctx, inputs)
                if ctx.tick == 4:
                    time.sleep(0.05)
        app = Slow(stop_at=30)
        res = run(app, grip_script(engage_at=99), overrun_fault_s=0.02)
        self.assertEqual(res.kind, FAULT)
        self.assertEqual(res.fault.phase, "timing")
        self.assertGreaterEqual(res.stats.overruns, 1)

    def test_estop_from_other_thread(self):
        class Tripper(Recorder):
            def hold_step(self, ctx, inputs):
                super().hold_step(ctx, inputs)
                if ctx.tick == 5:
                    threading.Thread(target=ctx.estop.trip, args=("button",), name="ui").start()
        app = Tripper(stop_at=10_000)
        t = time.monotonic()
        res = run(app, grip_script(engage_at=99))
        self.assertLess(time.monotonic() - t, 1.0)
        self.assertEqual((res.kind, res.reason), (ESTOP, "button"))
        self.assertEqual(app.emergency, [("button", "ui")])
        self.assertEqual(app.events[-1], "stop")

    def test_keyboard_interrupt_in_step(self):
        class KI(Recorder):
            def active_step(self, ctx, inputs):
                raise KeyboardInterrupt
        app = KI()
        res = run(app, grip_script(engage_at=2))
        self.assertEqual(res.kind, ESTOP)
        self.assertEqual(app.emergency[0][0], "KeyboardInterrupt")
        self.assertEqual(app.events[-1], "stop")

    def test_rate(self):
        app = Recorder(stop_at=50)
        app.rate_hz = 200.0
        t = time.monotonic()
        res = run(app, grip_script(engage_at=99))
        self.assertAlmostEqual(time.monotonic() - t, 50 / 200, delta=0.08)
        self.assertEqual(res.stats.ticks, 50)


SIGNAL_CHILD = textwrap.dedent("""
    import sys, time, threading
    from robot_utils.quest import parse_frame
    from robot_utils.teleop import TeleopApp, TeleopRunner
    from _quest_records import record

    MODE = sys.argv[1]

    class Src:
        def latest(self):
            return parse_frame(record(1), time.monotonic())

    class App(TeleopApp):
        rate_hz = 200.0
        def hold_step(self, ctx, inputs):
            if ctx.tick == 3:
                print("READY", flush=True)
            if MODE == "blocked" and ctx.tick == 5:
                time.sleep(30)        # stuck in a 'native call': the watcher must still trip
        def active_step(self, ctx, inputs):
            pass
        def emergency_stop(self, reason):
            print("EMERGENCY", reason, threading.current_thread().name, flush=True)
        def stop(self, ctx, result):
            print("STOP", result.kind, flush=True)
            if MODE == "hang":
                time.sleep(30)

    res = TeleopRunner(App(), Src(), estop_grace_s=0.5).run()
    print("RESULT", res.kind, res.reason, flush=True)
""")


class SignalTest(unittest.TestCase):
    def spawn(self, mode):
        env = {**os.environ, "PYTHONPATH": os.pathsep.join([SRC, TESTS])}
        p = subprocess.Popen([sys.executable, "-c", SIGNAL_CHILD, mode], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True, env=env)
        self.assertEqual(p.stdout.readline().strip(), "READY")
        return p

    def finish(self, p, timeout=10):
        out, err = p.communicate(timeout=timeout)
        return p.returncode, out, err

    def test_sigint_stops_cleanly(self):
        p = self.spawn("normal")
        p.send_signal(signal.SIGINT)
        code, out, err = self.finish(p)
        self.assertEqual(code, 0, err)
        self.assertIn("EMERGENCY SIGINT received estop-watcher", out)
        self.assertIn("STOP estop", out)
        self.assertIn("RESULT estop SIGINT received", out)

    def test_sigterm_while_main_thread_blocked(self):
        p = self.spawn("blocked")
        time.sleep(0.2)
        t = time.monotonic()
        p.send_signal(signal.SIGTERM)
        code, out, err = self.finish(p)
        self.assertIn("EMERGENCY SIGTERM received estop-watcher", out)
        # the sleep is interrupted by KeyboardInterrupt from the Python handler
        self.assertIn("STOP estop", out)
        self.assertLess(time.monotonic() - t, 5)

    def test_hung_stop_hard_exits_after_grace(self):
        p = self.spawn("hang")
        p.send_signal(signal.SIGINT)
        code, out, err = self.finish(p)
        self.assertEqual(code, 1)
        self.assertIn("STOP estop", out)
        self.assertIn("shutdown took over 0.5 s", err)

    def test_second_signal_hard_exits(self):
        p = self.spawn("hang")
        p.send_signal(signal.SIGINT)
        self.assertTrue(p.stdout.readline().startswith("EMERGENCY"))
        p.send_signal(signal.SIGINT)
        code, out, err = self.finish(p)
        self.assertEqual(code, 130)
        self.assertIn("second signal", err)


if __name__ == "__main__":
    unittest.main()
