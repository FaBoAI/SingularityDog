"""No-hardware tests for the current-boot raw full-body pose recorder."""

import contextlib
import io
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from singularitydog_hw import fixed_stance_readonly_capture as capture


EXPECTED = {i: f"{i:016x}" for i in range(1, 13)}


class Parser:
    buffer = b""
    discarded_bytes = 0


class FakeCAN:
    def __init__(self, bus, *, corrupt=None, move=0.):
        self.bus, self.corrupt, self.move = bus, corrupt, move
        self.parser, self.calls = Parser(), []
        self.samples = {mid: 0 for mid in capture.IDS_BY_BUS[bus]}

    def query(self, mid, parameter=None):
        self.calls.append((mid, parameter))
        if self.corrupt and (mid, parameter) == self.corrupt:
            raise TimeoutError("No fresh reply")
        before = time.monotonic_ns()
        result = {"ok": True, "request_monotonic_ns": before,
                  "monotonic_ns": before + 1}
        if parameter is None:
            result["mcu_uid_hex"] = EXPECTED[mid]
        else:
            if parameter == "position":
                self.samples[mid] += 1
                result.update(index=0x7019, unit="rad_output_shaft")
            result["value"] = ({"position": mid / 10 + self.move * self.samples[mid],
                                "velocity": 0., "current": 0., "voltage": 40.,
                                "run_mode": 0.}[parameter])
        return result


def buses(**changes):
    return {bus: FakeCAN(bus, **changes) for bus in capture.IDS_BY_BUS}


class CaptureTests(unittest.TestCase):
    def test_exact_two_bus_identity_and_fresh_pose(self):
        cans = buses()
        identities = capture.capture_identities(cans, EXPECTED, lambda: None)
        self.assertEqual(set(identities), {str(i) for i in range(1, 13)})
        enter = time.monotonic_ns()
        pose = capture.capture_pose(cans, lambda: None,
                                    enter_monotonic_ns=enter)
        self.assertEqual(set(pose["raw_rad_by_id"]), set(identities))
        self.assertEqual(pose["sampling_issues"], [])
        self.assertTrue(pose["sampling_stability_heuristic_passed"])
        self.assertFalse(pose["stationarity_verified"])
        self.assertEqual(len(pose["samples"]["7"]), 3)
        self.assertEqual(len(cans["front"].calls), 6 + 3 * 6 * len(capture.PARAMETERS))
        self.assertEqual(len(cans["rear"].calls), 6 + 3 * 6 * len(capture.PARAMETERS))
        self.assertTrue(all(mid <= 6 for mid, _ in cans["front"].calls))
        self.assertTrue(all(mid >= 7 for mid, _ in cans["rear"].calls))

    def test_wrong_uid_or_missing_reply_fails(self):
        wrong = dict(EXPECTED)
        wrong[7] = "f" * 16
        with self.assertRaisesRegex(RuntimeError, "ID7 UID mismatch"):
            capture.capture_identities(buses(), wrong, lambda: None)
        cans = {"front": FakeCAN("front"),
                "rear": FakeCAN("rear", corrupt=(7, "position"))}
        with self.assertRaises(TimeoutError):
            capture.capture_pose(cans, lambda: None,
                                 enter_monotonic_ns=time.monotonic_ns())

    def test_motion_is_reported_and_no_physical_or_stop_claims(self):
        pose = capture.capture_pose(buses(move=.02), lambda: None,
                                    enter_monotonic_ns=time.monotonic_ns())
        self.assertEqual(len(pose["sampling_issues"]), 12)
        summary = {"status": "RECORDED_REVIEW_REQUIRED", "boot_id": "boot",
                   "identities": {str(i): {"mcu_uid_hex": EXPECTED[i]}
                                  for i in EXPECTED}, "pose": pose}
        draft = capture.draft_manifest(summary, "a" * 64)
        self.assertEqual(draft["stop_state"], "UNVERIFIED_BY_READ_ONLY_PROTOCOL")
        self.assertFalse(draft["simultaneous_physical_stance_verified"])
        self.assertFalse(draft["all_segment_sweeps_physically_reviewed"])
        self.assertFalse(draft["same_boot_12_axis_hold_passed"])
        self.assertFalse(draft["approved_for_runtime"])
        self.assertFalse(draft["output_allowed"])
        self.assertFalse(draft["self_supported_standing_verified"])

    def test_bad_scope_or_stale_timestamp_fails_before_claims(self):
        cans = buses()
        cans["rear"] = cans["front"]
        with self.assertRaisesRegex(RuntimeError, "Buses must not share"):
            capture.capture_identities(cans, EXPECTED, lambda: None)
        with self.assertRaisesRegex(RuntimeError, "predates operator confirmation"):
            capture.capture_pose(buses(), lambda: None,
                                 enter_monotonic_ns=time.monotonic_ns() + 10**10)

    def test_default_cli_is_plan_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            uid_file = Path(tmp) / "uids.json"
            uid_file.write_text(__import__("json").dumps({str(i): EXPECTED[i]
                                                            for i in EXPECTED}))
            out = Path(tmp) / "out"
            with patch.object(capture, "ReadOnlyCAN") as io_open, \
                 contextlib.redirect_stdout(io.StringIO()):
                status = capture.main(["--front-port", "/dev/front",
                    "--rear-port", "/dev/rear", "--expected-uids", str(uid_file),
                    "--output", str(out)])
            self.assertEqual(status, 0)
            io_open.assert_not_called()
            self.assertFalse(out.exists())


if __name__ == "__main__":
    unittest.main()
