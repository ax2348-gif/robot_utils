import json
import socket
import threading
import time
import unittest

import numpy as np

from robot_utils.quest import ProtocolError, QuestClient, parse_frame

from _quest_records import EYE9, hand, record, rot_z9


class ParseTest(unittest.TestCase):
    def test_valid_record(self):
        f = parse_frame(record(5, right=hand(engaged=True, engagement_id=3, dp=(0.1, 0, 0),
                                             dr=rot_z9(0.5), gripper=0.7),
                               base_velocity=(0.2, 0, 0.1)), 12.0)
        self.assertEqual((f.frame_seq, f.session_id, f.input_focused, f.received_at), (5, "s1", True, 12.0))
        r = f.intent.hands["right"]
        self.assertTrue(r.valid and r.engaged)
        self.assertEqual(r.engagement_id, 3)
        np.testing.assert_allclose(r.delta_position, [0.1, 0, 0])
        np.testing.assert_allclose(r.delta_rotation, np.array(rot_z9(0.5)).reshape(3, 3), atol=1e-12)
        self.assertAlmostEqual(r.trigger, 0.7)
        np.testing.assert_allclose(f.intent.twist, [0.2, 0, 0.1])
        self.assertTrue(f.intent.twist_valid)
        self.assertFalse(r.delta_position.flags.writeable)

    def test_rejects_wrong_schema_and_sequencing(self):
        for bad in ({**record(1), "schema": "other"}, {**record(1), "frame_seq": -1},
                    {**record(1), "frame_seq": 1.5}, {**record(1), "frame_seq": True},
                    {**record(1), "session_id": None}, [1, 2]):
            with self.assertRaises(ProtocolError):
                parse_frame(bad, 0.0)

    def test_missing_or_bad_intent_keeps_frame(self):
        f = parse_frame(record(1, intent=False), 0.0)
        self.assertIsNone(f.intent)
        rec = record(1)
        rec["control_intent"]["schema"] = "teleop.operator_intent.v2"
        f = parse_frame(rec, 0.0)
        self.assertIsNone(f.intent)
        self.assertIn("schema", f.intent_error)

    def test_bad_hand_numbers_make_hand_invalid(self):
        cases = {"null": hand(dp=(None, 0, 0)), "nan-rot": hand(dr=[None] * 9),
                 "not-rotation": hand(dr=[2, 0, 0, 0, 1, 0, 0, 0, 1]),
                 "reflection": hand(dr=[-1, 0, 0, 0, 1, 0, 0, 0, 1]),
                 "short": hand(dp=(0, 0)), "trigger": hand(gripper=None),
                 "trigger-range": hand(gripper=3.0), "bool": hand(dp=(True, 0, 0))}
        for name, h in cases.items():
            h["engaged"] = True
            with self.subTest(name):
                r = parse_frame(record(1, right=h), 0.0).intent.hands["right"]
                self.assertFalse(r.valid)
                self.assertIsNotNone(r.invalid_reason)
                self.assertTrue(r.engaged)   # flags kept for diagnostics, but invalid
                np.testing.assert_array_equal(r.delta_position, 0)

    def test_missing_hand_and_reported_invalid(self):
        rec = record(1)
        del rec["control_intent"]["hands"]["left"]
        rec["control_intent"]["hands"]["right"]["valid"] = False
        hands = parse_frame(rec, 0.0).intent.hands
        self.assertFalse(hands["left"].valid)
        self.assertFalse(hands["right"].valid)

    def test_rotation_reorthonormalised(self):
        r = parse_frame(record(1, right=hand(dr=[round(v, 4) for v in rot_z9(0.3)])), 0.0).intent.hands["right"]
        m = r.delta_rotation
        np.testing.assert_allclose(m.T @ m, np.eye(3), atol=1e-12)

    def test_body_frozen_operator_base(self):
        h = hand(engaged=True, engagement_id=1)
        h.update(operator_base_in_quest_position=[0.1, 1.5, -0.2], operator_base_in_quest_rotation=rot_z9(0.3))
        rec = record(1, right=h)
        rec["control_intent"].update(scheme="body-frozen", translation_limit_m=None)
        f = parse_frame(rec, 0.0)
        self.assertEqual(f.intent.scheme, "body-frozen")
        self.assertIsNone(f.intent.translation_limit_m)
        np.testing.assert_allclose(f.intent.hands["right"].operator_base_position, [0.1, 1.5, -0.2])
        self.assertIsNotNone(f.intent.hands["right"].operator_base_rotation)
        self.assertIsNone(f.intent.hands["left"].operator_base_position)

    def test_bad_twist_is_zero(self):
        f = parse_frame(record(1, base_velocity=(None, 0, 0)), 0.0)
        self.assertFalse(f.intent.twist_valid)
        np.testing.assert_array_equal(f.intent.twist, 0)


class _Server:
    """One-connection-at-a-time TCP server whose lines the test pushes."""

    def __init__(self):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen()
        self.port = self.srv.getsockname()[1]
        self.conn = None
        self.accepted = threading.Event()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            self.conn = c
            self.accepted.set()

    def send(self, *chunks):
        for c in chunks:
            self.conn.sendall(c if isinstance(c, bytes) else (json.dumps(c) + "\n").encode())

    def drop(self):
        self.accepted.clear()
        self.conn.close()

    def close(self):
        self.srv.close()
        if self.conn:
            self.conn.close()


def _wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


class ClientTest(unittest.TestCase):
    def setUp(self):
        self.server = _Server()
        self.client = QuestClient("127.0.0.1", self.server.port, backoff_s=(0.05, 0.1)).start()
        self.assertTrue(self.server.accepted.wait(3))

    def tearDown(self):
        self.client.close()
        self.server.close()

    def test_receives_orders_and_counts(self):
        line = (json.dumps(record(3)) + "\n").encode()
        self.server.send(record(1), line[:10], line[10:], record(2), b"not json\n",
                         {"schema": "x"}, record(4), b"\n")
        self.assertTrue(_wait(lambda: self.client.stats.frames == 3 and self.client.stats.bad_lines == 2))
        f = self.client.latest()
        self.assertEqual(f.frame_seq, 4)
        self.assertEqual(self.client.stats.out_of_order, 1)   # seq 2 after 3
        self.assertLess(self.client.age(), 1.0)

    def test_new_session_resets_order(self):
        self.server.send(record(100, session="a"))
        self.assertIsNotNone(self.client.wait_for_frame(2.0))
        self.server.send(record(1, session="b"))
        self.assertTrue(_wait(lambda: self.client.latest().session_id == "b"))

    def test_reconnects_and_keeps_last_frame(self):
        self.server.send(record(10))
        self.assertTrue(_wait(lambda: self.client.stats.frames == 1))
        self.server.drop()
        self.assertTrue(_wait(lambda: not self.client.stats.connected))
        self.assertEqual(self.client.latest().frame_seq, 10)   # kept; age() grows
        self.assertTrue(self.server.accepted.wait(3))
        self.assertTrue(_wait(lambda: self.client.stats.connects == 2))
        self.server.send(record(0))   # provider restarted the count: new connection, accepted
        self.assertTrue(_wait(lambda: self.client.latest().frame_seq == 0))

    def test_close_is_prompt(self):
        t = time.monotonic()
        self.client.close()
        self.assertLess(time.monotonic() - t, 1.0)
        self.assertFalse(self.client._thread.is_alive())


class ClientNoServerTest(unittest.TestCase):
    def test_no_server_backs_off_and_closes(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        with QuestClient("127.0.0.1", port, backoff_s=(0.01, 0.05)) as c:
            self.assertIsNone(c.wait_for_frame(0.2))
            self.assertIn("connect", c.stats.last_error)
            self.assertEqual(c.age(), float("inf"))


if __name__ == "__main__":
    unittest.main()
