"""Offline before/after tests; all positions and identities are synthetic."""

import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import stat
import tempfile
import unittest

from singularitydog_hw.joint_observation import (
    ObservationError, ObservationLimits, compare_captures, compare_jsonl_files, main,
)


def capture(start=1_000_000_000, changes=None, bases=None):
    changes, bases = changes or {}, bases or {}
    records = []
    def add(record):
        moment = start + len(records) * 1_000_000
        record.update(kind="motor_parameter", ok=True, monotonic_ns=moment,
                      wall_time_ns=1_790_000_000_000_000_000 + moment)
        records.append(record)
    for mid in range(1, 13):
        add({"motor_id": mid, "parameter": "identity", "mcu_uid_hex": "%016x" % mid})
    for jitter in (-0.002, 0.0, 0.001, 0.002):
        for mid in range(1, 13):
            add({"motor_id": mid, "parameter": "position", "index": 0x7019,
                 "status": 0, "unit": "rad_output_shaft",
                 "value": bases.get(mid, mid * 0.1) + changes.get(mid, 0) + jitter})
    return records


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.before = capture()
        self.after = capture(3_000_000_000, {2: 0.2})

    def test_single_change_is_only_a_candidate_and_input_is_unchanged(self):
        originals = copy.deepcopy((self.before, self.after))
        report = compare_captures(self.before, self.after, 2)
        self.assertEqual(report["classification"], "expected_change_candidate")
        self.assertEqual(report["changed_motor_ids"], [2])
        self.assertAlmostEqual(report["motors"]["2"]["median_change_rad"], 0.2)
        self.assertAlmostEqual(report["motors"]["2"]["mean_change_rad"], 0.2)
        self.assertAlmostEqual(report["motors"]["2"]["before"]["peak_to_peak_rad"], 0.004)
        for motor in report["motors"].values():
            self.assertTrue(math.isfinite(motor["mean_change_rad"]))
            self.assertTrue(math.isfinite(motor["median_change_rad"]))
        self.assertTrue(report["requires_physical_confirmation"])
        for key in ("approved_for_runtime", "semantic_joint_role_inferred", "direction_sign_inferred",
                    "zero_inferred", "angle_wrapping_applied", "commissioning_modified"):
            self.assertFalse(report[key])
        self.assertEqual((self.before, self.after), originals)

    def test_no_change_and_other_joint_changes_are_not_unambiguous(self):
        report = compare_captures(self.before, capture(3_000_000_000), 2)
        self.assertEqual(report["classification"], "no_change")
        report = compare_captures(self.before, capture(3_000_000_000, {2: 0.2, 5: -0.1}), 2)
        self.assertEqual(report["classification"], "ambiguous")
        self.assertEqual(report["other_changed_motor_ids"], [5])
        report = compare_captures(self.before, capture(3_000_000_000, {5: 0.1}), 2)
        self.assertEqual(report["classification"], "ambiguous")

    def test_wrap_like_discontinuity_is_preserved_and_flagged(self):
        before = capture(bases={2: 6.25})
        after = capture(3_000_000_000, bases={2: 0.15})
        report = compare_captures(before, after, 2)
        self.assertAlmostEqual(report["motors"]["2"]["median_change_rad"], -6.1)
        self.assertTrue(report["motors"]["2"]["large_raw_change_or_discontinuity"])
        self.assertEqual(report["classification"], "ambiguous")
        self.assertFalse(report["angle_wrapping_applied"])

    def test_turn_counted_positions_are_not_reduced_modulo(self):
        before = capture(bases={2: 7 * math.pi})
        after = capture(3_000_000_000, {2: 0.2}, bases={2: 7 * math.pi})
        report = compare_captures(before, after, 2)
        self.assertGreater(report["motors"]["2"]["before"]["median_rad"], 6 * math.pi)
        self.assertAlmostEqual(report["motors"]["2"]["median_change_rad"], 0.2)
        self.assertEqual(report["classification"], "expected_change_candidate")

    def test_unsettled_captures_and_adjacent_jumps_are_ambiguous(self):
        positions = [r for r in self.before if r["parameter"] == "position" and r["motor_id"] == 8]
        positions[0]["value"] += 0.2
        report = compare_captures(self.before, self.after, 2)
        self.assertTrue(report["motors"]["8"]["unsettled_capture"])
        self.assertEqual(report["classification"], "ambiguous")
        positions[0]["value"] += 2 * math.pi
        report = compare_captures(self.before, self.after, 2)
        self.assertTrue(report["motors"]["8"]["large_raw_change_or_discontinuity"])

    def test_identity_mapping_mismatch_missing_and_duplicate_are_rejected(self):
        for change in ("mismatch", "missing", "duplicate", "malformed", "bad_id"):
            records = copy.deepcopy(self.after)
            if change == "mismatch":
                records[0]["mcu_uid_hex"] = "ffffffffffffffff"
            elif change == "missing":
                del records[0]
            elif change == "duplicate":
                records[0]["mcu_uid_hex"] = records[1]["mcu_uid_hex"]
            elif change == "malformed":
                records[0]["mcu_uid_hex"] = "0011"
            else:
                records[0]["motor_id"] = True
            with self.subTest(change=change), self.assertRaises(ObservationError):
                compare_captures(self.before, records, 2)

    def test_each_id_requires_three_positions(self):
        removed = 0
        after = []
        for record in self.after:
            if record["parameter"] == "position" and record["motor_id"] == 12 and removed < 2:
                removed += 1
            else:
                after.append(record)
        with self.assertRaisesRegex(ObservationError, "ID12.*three"):
            compare_captures(self.before, after, 2)

    def test_bad_values_units_and_status_fail_closed(self):
        for field, bad in (("value", math.nan), ("value", math.inf), ("value", True),
                           ("value", "1"), ("unit", "degrees"), ("index", 0x701A),
                           ("status", 1), ("ok", False)):
            records = copy.deepcopy(self.after)
            records[12][field] = bad
            with self.subTest(field=field, bad=bad), self.assertRaises(ObservationError):
                compare_captures(self.before, records, 2)
        for records in (self.before, self.after):
            for record in records:
                if record["parameter"] == "position":
                    record["value"] = 1e308
        with self.assertRaisesRegex(ObservationError, "statistics"):
            compare_captures(self.before, self.after, 2)

    def test_errors_timeouts_and_malformed_records_are_not_ignored(self):
        for extra in (None, {"kind": ""}, {"kind": "can_timeout"}, {"kind": "other", "error": "failed"},
                      {"kind": "other", "errors": ["failed"]}, {"kind": "motor_feedback", "fault_bits": 1}):
            records = self.after + [extra]
            with self.subTest(extra=extra), self.assertRaises(ObservationError):
                compare_captures(self.before, records, 2)
        self.after.append({"kind": "imu", "monotonic_ns": 3_100_000_000,
                           "wall_time_ns": 1_790_000_003_100_000_000})
        self.assertEqual(compare_captures(self.before, self.after, 2)["classification"],
                         "expected_change_candidate")

    def test_clocks_must_be_present_increasing_and_both_intervals_nonoverlapping(self):
        for field in ("monotonic_ns", "wall_time_ns"):
            for bad in (None, True, math.nan, 0, self.after[11][field]):
                records = copy.deepcopy(self.after)
                records[12][field] = bad
                with self.subTest(field=field, bad=bad), self.assertRaises(ObservationError):
                    compare_captures(self.before, records, 2)
            records = copy.deepcopy(self.after)
            for record, prior in zip(records, self.before):
                record[field] = prior[field]
            with self.assertRaisesRegex(ObservationError, "overlap"):
                compare_captures(self.before, records, 2)
        with self.assertRaisesRegex(ObservationError, "reversed"):
            compare_captures(self.after, self.before, 2)

    def test_expected_id_and_limits_are_validated(self):
        for bad in (0, 13, True, "2", 2.0):
            with self.assertRaises(ObservationError):
                compare_captures(self.before, self.after, bad)
        for bad in (0, -1, math.nan, math.inf, True, "1"):
            with self.assertRaises(ValueError):
                ObservationLimits(movement_threshold_rad=bad)
        with self.assertRaises(ValueError):
            ObservationLimits(movement_threshold_rad=4)


class FileTests(unittest.TestCase):
    def write(self, directory, name, records):
        path = Path(directory) / name
        path.write_text("".join(json.dumps(r) + "\n" for r in records))
        return path

    def test_private_output_hashes_and_no_overwrite_or_input_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            before = self.write(directory, "before.jsonl", capture())
            after = self.write(directory, "after.jsonl", capture(3_000_000_000, {2: 0.2}))
            originals = (before.read_bytes(), after.read_bytes())
            output = Path(directory) / "candidate.json"
            report = compare_jsonl_files(before, after, 2, output)
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
            self.assertEqual(report["sources"]["before"]["sha256"], hashlib.sha256(originals[0]).hexdigest())
            self.assertEqual(report["sources"]["after"]["sha256"], hashlib.sha256(originals[1]).hexdigest())
            self.assertEqual((before.read_bytes(), after.read_bytes()), originals)
            saved = output.read_bytes()
            with self.assertRaises(FileExistsError):
                compare_jsonl_files(before, after, 2, output)
            self.assertEqual(output.read_bytes(), saved)

    def test_bad_json_duplicate_keys_same_capture_and_git_output_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            before = self.write(directory, "before.jsonl", capture())
            after = self.write(directory, "after.jsonl", capture(3_000_000_000, {2: 0.2}))
            output = Path(directory) / "candidate.json"
            for invalid in ('{"x":1,"x":2}', '{"x":NaN}', '[]', '{bad'):
                after.write_text(invalid)
                with self.assertRaises(ObservationError):
                    compare_jsonl_files(before, after, 2, output)
                self.assertFalse(output.exists())
            with self.assertRaisesRegex(ObservationError, "identical"):
                compare_jsonl_files(before, before, 2, output)
            (Path(directory) / ".git").mkdir()
            with self.assertRaisesRegex(ObservationError, "Git"):
                compare_jsonl_files(before, before, 2, output)

    def test_cli_is_offline_and_reports_candidate_only(self):
        with tempfile.TemporaryDirectory() as directory:
            before = self.write(directory, "before.jsonl", capture())
            after = self.write(directory, "after.jsonl", capture(3_000_000_000, {2: 0.2}))
            output = Path(directory) / "candidate.json"
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                self.assertEqual(main(["--before", str(before), "--after", str(after),
                                       "--expected-id", "2", "--output", str(output)]), 0)
            self.assertEqual(json.loads(stream.getvalue())["status"], "candidate")
            self.assertTrue(json.loads(stream.getvalue())["requires_physical_confirmation"])


if __name__ == "__main__":
    unittest.main()
