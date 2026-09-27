"""No-hardware checks for the post-charge, read-only box-pose comparison."""

import contextlib
import io
import json
import math
import os
from pathlib import Path
import stat
import tempfile
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import post_charge_box_pose_check as check
from singularitydog_hw.can_readonly import PARAMETERS, read_request


BOOT = "test-boot"
UIDS = {str(i): f"{i:016x}" for i in range(1, 13)}


def baseline():
    return {"status": check.BASELINE_STATUS, "boot_id": BOOT,
            "created_ns": 100, "motor_output_allowed": False,
            "rows": {str(i): {"uid": UIDS[str(i)], "run_mode": 0,
                              "current_A": 0.0, "median_position_rad": i / 10,
                              "span_deg": .01} for i in range(1, 13)}}


class FakeCAN:
    opened = []
    wrong_uid = False
    wrong_parameter = False

    def __init__(self, port, event_sink):
        self.bus = "front" if port.endswith("front") else "rear"
        self.sink = event_sink
        self.parser = type("Parser", (), {"buffer": b"", "discarded_bytes": 0})()
        self.serial = self
        self.fd = None
        self.calls = []
        self.position_counts = {}
        FakeCAN.opened.append(self)

    def __enter__(self):
        self.fd = os.open("/dev/null", os.O_RDONLY)
        return self

    def __exit__(self, *_):
        os.close(self.fd)

    def fileno(self):
        return self.fd

    def query(self, mid, parameter=None):
        self.calls.append((mid, parameter))
        self.sink({"kind": "can_tx", "motor_id": mid,
                   "parameter": parameter or "identity",
                   "hex": read_request(mid, parameter).hex()})
        before = time.monotonic_ns()
        row = {"ok": True, "request_monotonic_ns": before,
               "monotonic_ns": before + 1}
        if parameter is None:
            row["mcu_uid_hex"] = ("f" * 16 if self.wrong_uid and mid == 7
                                  else UIDS[str(mid)])
        else:
            index, _, unit = PARAMETERS[parameter]
            row.update(index=index + (1 if self.wrong_parameter and mid == 7 else 0),
                       unit=unit)
            if parameter == "position":
                count = self.position_counts.get(mid, 0)
                self.position_counts[mid] = count + 1
                row["value"] = mid / 10 + (2 * math.pi if mid == 3 else 0) + count * .001
            else:
                row["value"] = {"run_mode": 0, "current": 0., "voltage": 40.}[parameter]
        return row


class PostChargeTests(unittest.TestCase):
    def setUp(self):
        FakeCAN.opened = []
        FakeCAN.wrong_uid = False
        FakeCAN.wrong_parameter = False

    def _run(self, directory, *, execute=True):
        root = Path(directory)
        source = root / "baseline.json"
        source.write_text(json.dumps(baseline()))
        output = root / "private-result.json"
        binding = {"path": "/dev/front", "resolved": "/dev/null",
                   "st_rdev": os.stat("/dev/null").st_rdev}
        bindings = {"front": dict(binding),
                    "rear": {**binding, "path": "/dev/rear"}}
        argv = ["--baseline", str(source), "--front-port", "/dev/front",
                "--rear-port", "/dev/rear", "--output", str(output)]
        if execute:
            argv.append("--execute-readonly")
        with patch.object(check, "_boot_id", return_value=BOOT), \
             patch.object(check.dual, "validate_ports", return_value=bindings), \
             patch.object(check.dual, "binding_matches", return_value=True), \
             patch.object(check.dual, "port_lock", return_value=contextlib.nullcontext()), \
             patch.object(check, "ownership_locks", return_value=contextlib.nullcontext()), \
             patch.object(check, "ReadOnlyCAN", FakeCAN), \
             contextlib.redirect_stdout(io.StringIO()):
            code = check.main(argv)
        return code, output

    def test_complete_read_keeps_direct_360_degree_branch_and_private_file(self):
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory)
            self.assertEqual(code, 0)
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
            result = json.loads(output.read_text())
        self.assertEqual(result["status"], "RECORDED_REVIEW_REQUIRED")
        self.assertFalse(result["angle_wrap_applied"])
        self.assertFalse(result["approved_for_runtime"])
        self.assertEqual(set(result["identities"]), check.ALL_IDS)
        self.assertEqual(set(result["telemetry"]["rows"]), check.ALL_IDS)
        self.assertAlmostEqual(result["direct_delta_by_id"]["3"]["direct_delta_deg"],
                               360 + math.degrees(.001), places=6)
        self.assertEqual(len(result["telemetry"]["rows"]["3"]["position_samples"]), 3)
        for can in FakeCAN.opened:
            allowed = check.IDS_BY_BUS[can.bus]
            self.assertEqual(len(can.calls), 6 + 6 * (3 + 3))
            self.assertTrue(all(mid in allowed and parameter in (None, *check.READS)
                                for mid, parameter in can.calls))
            self.assertTrue(all(parameter is None for _, parameter in can.calls[:6]))

    def test_uid_mismatch_stops_before_type17(self):
        FakeCAN.wrong_uid = True
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory)
            self.assertEqual(code, 1)
            result = json.loads(output.read_text())
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIsNone(result["telemetry"])
        self.assertIn("UID mismatch", result["errors"][0])
        self.assertTrue(all(parameter is None for can in FakeCAN.opened
                            for _, parameter in can.calls))

    def test_parameter_mismatch_fails_closed(self):
        FakeCAN.wrong_parameter = True
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory)
            self.assertEqual(code, 1)
            result = json.loads(output.read_text())
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("parameter mismatch", result["errors"][0])
        self.assertIsNone(result["direct_delta_by_id"])

    def test_plan_does_not_open_can_or_write(self):
        with tempfile.TemporaryDirectory() as directory:
            code, output = self._run(directory, execute=False)
            self.assertEqual(code, 0)
            self.assertFalse(output.exists())
            self.assertEqual(FakeCAN.opened, [])

    def test_baseline_and_tx_guard_reject_invalid_data(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "baseline.json"
            data = baseline()
            data["rows"]["9"]["span_deg"] = .2
            source.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "invalid raw position"):
                check.load_baseline(source)
            source.write_text('{"status":1,"status":2}')
            with self.assertRaisesRegex(ValueError, "Duplicate key"):
                check.load_baseline(source)
        with self.assertRaisesRegex(RuntimeError, "Unexpected CAN transmission"):
            check.guard_event("front", {"kind": "can_tx", "motor_id": 1,
                                        "parameter": "enable", "hex": ""})
        with self.assertRaisesRegex(RuntimeError, "Noncanonical CAN transmission"):
            check.guard_event("front", {"kind": "can_tx", "motor_id": 1,
                                        "parameter": "position", "hex": "bad"})


if __name__ == "__main__":
    unittest.main()
