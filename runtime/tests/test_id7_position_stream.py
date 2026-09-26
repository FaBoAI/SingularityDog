import math
import unittest

from singularitydog_hw.id7_position_stream import (
    REAR_PORT, collect, continuity_metrics, validate_plan,
)


class PositionStreamTests(unittest.TestCase):
    def setUp(self):
        self.uids = {str(i): f"{i:016x}" for i in range(1, 13)}

    def test_requires_verified_rear_port_and_finite_short_window(self):
        self.assertEqual(validate_plan(REAR_PORT, 15, self.uids)["allowed_can_types"], [0, 17])
        for port in ("/dev/ttyUSB0", "/dev/robstride-usb2can", "relative/path"):
            with self.assertRaises(ValueError):
                validate_plan(port, 15, self.uids)
        for seconds in (0, 20.1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                validate_plan(REAR_PORT, seconds, self.uids)

    def test_reports_raw_adjacent_step_without_unwrapping(self):
        rows = [
            {"sample": 0, "monotonic_ns": 0, "position_rad": 0.0},
            {"sample": 1, "monotonic_ns": 20_000_000, "position_rad": math.radians(0.2)},
            {"sample": 2, "monotonic_ns": 40_000_000, "position_rad": math.radians(47.2)},
        ]
        result = continuity_metrics(rows)
        self.assertEqual(result["adjacent_changes_over_5deg"], 1)
        self.assertAlmostEqual(result["largest_adjacent_change"]["delta_deg"], 47)
        self.assertEqual(result["largest_adjacent_change"]["to_sample"], 2)

    def test_collection_transmits_only_id7_position_reads(self):
        class FakeCan:
            timeout_s = .01

            def __init__(self):
                self.now = 0
                self.requests = []
                self.parser = type("Parser", (), {"discarded_bytes": 0, "buffer": b""})()

            def query(self, motor_id, parameter):
                self.requests.append((motor_id, parameter))
                self.now += 20_000_000
                return {"ok": True, "value": len(self.requests) * .001,
                        "monotonic_ns": self.now, "raw_value_hex": "00000000",
                        "round_trip_ms": 1.0}

        can = FakeCan()
        events = []
        rows = collect(can, 1.0, events.append, clock=lambda: can.now,
                       sleep=lambda _: None)
        self.assertGreater(len(rows), 1)
        self.assertTrue(all(req == (7, "position") for req in can.requests))
        self.assertLessEqual(can.now, 1_000_000_000)
        self.assertEqual(len(events), len(rows))


if __name__ == "__main__":
    unittest.main()
