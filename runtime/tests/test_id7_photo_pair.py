import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from singularitydog_hw import id7_photo_pair as module


class FakeCAN:
    calls = []
    uid_override = None

    def __init__(self, *, port, event_sink):
        self.port = port
        self.sink = event_sink
        self.parser = SimpleNamespace(discarded_bytes=0, buffer=b"")
        self.counter = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def query(self, motor_id, parameter=None):
        self.calls.append((motor_id, parameter))
        self.counter += 1
        if parameter is None:
            uid = (self.uid_override if motor_id == 8 and self.uid_override
                   else f"{motor_id:016x}")
            return {"ok": True, "mcu_uid_hex": uid,
                    "monotonic_ns": self.counter * 1_000_000}
        if parameter != "position" or motor_id != 7:
            raise AssertionError("Unexpected CAN query")
        pose = "A" if self.counter <= 6 else "B"
        values = {"A": [1.0, 1.002, 1.001], "B": [1.4, 1.402, 1.401]}
        sample = (self.counter - 4) % 6
        value = values[pose][sample]
        return {"ok": True, "value": value,
                "monotonic_ns": self.counter * 1_000_000,
                "raw_value_hex": "00000000"}


class ID7PhotoPairTests(unittest.TestCase):
    def setUp(self):
        self.uids = {str(i): f"{i:016x}" for i in range(1, 13)}
        FakeCAN.calls = []
        FakeCAN.uid_override = None

    def test_requires_verified_rear_port_and_distinct_full_uid_inventory(self):
        self.assertEqual(module.validate_port(module.REAR_PORT), module.REAR_PORT)
        for port in ("/dev/ttyUSB1", "/dev/robstride-usb2can", "relative"):
            with self.assertRaises(ValueError):
                module.validate_port(port)
        self.assertIs(module.validate_expected_uids(self.uids), self.uids)
        for invalid in ({"7": self.uids["7"]},
                        {**self.uids, "8": self.uids["7"]},
                        {**self.uids, "8": "NOT-HEX"}):
            with self.assertRaises(ValueError):
                module.validate_expected_uids(invalid)

    def test_capture_verifies_three_uids_and_only_reads_id7_position(self):
        can = FakeCAN(port=module.REAR_PORT, event_sink=lambda _: None)
        events = []
        snap = module.capture_pose(can, "A", self.uids, events.append,
                                   lambda: None, sleep=lambda _: None)
        self.assertEqual(FakeCAN.calls, [(7, None), (8, None), (9, None)] +
                         [(7, "position")] * 3)
        self.assertEqual(len([e for e in events if e["kind"] == "photo_pair_position"]), 3)
        self.assertAlmostEqual(snap["median_position_rad"], 1.001)
        self.assertAlmostEqual(snap["position_range_deg"], math.degrees(.002))
        self.assertFalse(snap["stationarity_certified"])

    def test_uid_mismatch_stops_before_any_position_read(self):
        FakeCAN.uid_override = "f" * 16
        can = FakeCAN(port=module.REAR_PORT, event_sink=lambda _: None)
        with self.assertRaisesRegex(RuntimeError, "ID8 UID mismatch"):
            module.capture_pose(can, "A", self.uids, lambda _: None,
                                lambda: None, sleep=lambda _: None)
        self.assertEqual(FakeCAN.calls, [(7, None), (8, None)])

    def test_photo_ack_records_ref_and_two_timestamps_or_quits(self):
        ack = module.photo_ack("A", lambda: None, ask=lambda _: "IMG_0448.JPG",
                               clock_ns=lambda: 123, wall_ns=lambda: 456)
        self.assertEqual(ack, {"pose": "A", "photo_reference": "IMG_0448.JPG",
                               "ack_monotonic_ns": 123, "ack_wall_time_ns": 456})
        with self.assertRaises(InterruptedError):
            module.photo_ack("B", lambda: None, ask=lambda _: "q")

    def test_main_pairs_both_photos_and_is_never_runtime_approved(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            uid_file = root / "uids.json"
            uid_file.write_text(json.dumps(self.uids))
            output = root / "pair"
            argv = ["--execute-readonly", "--expected-uids", str(uid_file),
                    "--output", str(output)]
            def ack(pose, check):
                check()
                return {"pose": pose, "photo_reference": f"photo-{pose}.jpg",
                        "ack_wall_time_ns": 100 if pose == "A" else 200,
                        "ack_monotonic_ns": 10 if pose == "A" else 20}
            with patch.object(module, "ReadOnlyCAN", FakeCAN), \
                 patch.object(module, "read_boot_id", return_value="same-boot"), \
                 patch.object(module, "photo_ack", side_effect=ack), \
                 patch.object(module, "SAMPLE_GAP_S", 0), \
                 patch.object(Path, "home", return_value=root), \
                 patch("sys.stdin.isatty", return_value=True), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(module.main(argv), 0)
            summary = json.loads((output / "summary.json").read_text())
            events = [json.loads(line) for line in
                      (output / "events.jsonl").read_text().splitlines()]
        self.assertEqual(summary["status"], "RECORDED_REVIEW_REQUIRED")
        self.assertFalse(summary["approved_for_runtime"])
        self.assertFalse(summary["calibration_applied"])
        self.assertEqual(summary["boot_id"], "same-boot")
        self.assertEqual(summary["poses"]["A"]["photo_reference"], "photo-A.jpg")
        self.assertEqual(summary["poses"]["B"]["photo_reference"], "photo-B.jpg")
        self.assertAlmostEqual(summary["raw_position_change_deg"], math.degrees(.4))
        self.assertEqual(len([e for e in events if e["kind"] == "photo_pair_position"]), 6)
        self.assertEqual(FakeCAN.calls, ([(7, None), (8, None), (9, None)] +
                                          [(7, "position")] * 3) * 2)

    def test_dry_run_prints_plan_without_opening_bus(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            uid_file = root / "uids.json"
            uid_file.write_text(json.dumps(self.uids))
            with patch.object(module, "ReadOnlyCAN") as bus, \
                 contextlib.redirect_stdout(io.StringIO()) as stream:
                self.assertEqual(module.main(["--expected-uids", str(uid_file),
                                              "--output", str(root / "pair")]), 0)
            bus.assert_not_called()
            self.assertFalse((root / "pair").exists())
            self.assertFalse(json.loads(stream.getvalue())["motor_output_available"])


if __name__ == "__main__":
    unittest.main()
