"""Offline replacement inventory: real synthetic complete snapshot files."""
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import replacement_inventory as inventory
from test_pose_record import snapshot, edit


class ReplacementInventoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.old = snapshot(self.root, "old")
        self.new = snapshot(self.root, "new", start=5_000_000_000)
        for mid in (6, 9, 10):
            self.uid(self.new, mid, f"f{mid:015x}")
        self.config = {"schema_version": 1, "expected_changed_ids": [6, 9, 10],
                       "previous": {"capture": str(self.old)}, "current": {"capture": str(self.new)}}
        self.config_path = self.root / "config.json"
        self.pin()

    @staticmethod
    def sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def pin(self):
        for label, path in (("previous", self.old), ("current", self.new)):
            self.config[label].update(events_sha256=self.sha(path / "events.jsonl"),
                                      summary_sha256=self.sha(path / "summary.json"))
        self.write_config()

    def write_config(self):
        self.config_path.write_text(json.dumps(self.config))

    @staticmethod
    def uid(path, mid, value):
        def change(records, meta):
            next(r for r in records if r["kind"] == "motor_parameter" and
                 r["motor_id"] == mid and r["parameter"] == "identity")["mcu_uid_hex"] = value
            meta["summary"]["identities"][str(mid)] = value
        edit(path, change)

    def values(self, mid, name, values):
        def change(records, meta):
            replies = [r for r in records if r["kind"] == "motor_parameter" and
                       r["motor_id"] == mid and r["parameter"] == name]
            self.assertEqual(len(replies), len(values))
            for reply, value in zip(replies, values):
                reply["value"] = value
            if name == "position":
                meta["summary"]["positions"][str(mid)] = {
                    "samples": len(values), "mean": sum(values)/len(values),
                    "min": min(values), "max": max(values), "last": values[-1]}
        edit(self.new, change)
        self.pin()

    def build(self):
        return inventory.build_inventory(self.config_path)

    def blocked_export(self):
        result = inventory.save_inventory(self.config_path, self.root / "result")
        self.assertEqual(result["status"], "BLOCKED_REVIEW_REQUIRED")
        self.assertFalse(result["inventory_candidate_validated"])
        self.assertTrue((self.root / "result" / "inventory.json").exists())
        self.assertFalse((self.root / "result" / "expected-uids.json").exists())
        return result

    def test_exact_three_changes_keep_id11_and_export_candidate_only(self):
        before = {p: p.read_bytes() for d in (self.old, self.new) for p in d.iterdir()}
        output = self.root / "out"
        result = inventory.save_inventory(self.config_path, output)
        self.assertEqual(result["status"], "CANDIDATE_INVENTORY_VALIDATED")
        self.assertEqual(result["observed_changed_ids"], [6, 9, 10])
        self.assertEqual(result["required_recalibration_ids"], [6, 9, 10])
        self.assertEqual(result["unchanged_ids"], [1, 2, 3, 4, 5, 7, 8, 11, 12])
        current = json.loads((output / "expected-uids.json").read_text())
        self.assertEqual(set(current), {str(i) for i in range(1, 13)})
        self.assertEqual(current["11"], f"{11:016x}")
        self.assertEqual(current["6"], f"f{6:015x}")
        self.assertEqual(result["expected_uids_export"]["sha256"], self.sha(output / "expected-uids.json"))
        self.assertTrue(all(q["samples"] == 3 for q in result["current_observation_quality"]))
        for flag in ("approved_for_runtime", "output_allowed", "motor_output_available", "live_readiness",
                     "zero_verified", "sign_verified", "calibration_copied", "prior_rr_overlay_inherited",
                     "motor_power_cycle_continuity_verified", "motor_enable_state_verified"):
            self.assertIs(result[flag], False)
        self.assertNotIn("offset", json.dumps(result))
        self.assertEqual(before, {p: p.read_bytes() for p in before})
        self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o700)
        for p in output.iterdir():
            self.assertEqual(stat.S_IMODE(p.stat().st_mode), 0o600)

    def test_extra_change_id11_rejects_uid_export(self):
        self.uid(self.new, 11, f"e{11:015x}")
        self.pin()
        result = self.blocked_export()
        self.assertFalse(result["identity_and_chronology_gates"]["other_identities_unchanged"])
        self.assertEqual(result["observed_changed_ids"], [6, 9, 10, 11])

    def test_missing_expected_change_rejects_uid_export(self):
        self.uid(self.new, 9, f"{9:016x}")
        self.pin()
        result = self.blocked_export()
        self.assertFalse(result["identity_and_chronology_gates"]["exact_expected_ids_changed"])
        self.assertIn(9, result["retired_uid_present_at_ids"])

    def test_retired_uid_reassigned_to_different_changed_id_rejected(self):
        self.uid(self.new, 9, f"{6:016x}")
        self.pin()
        result = self.blocked_export()
        self.assertEqual(result["observed_changed_ids"], [6, 9, 10])
        self.assertTrue(result["identity_and_chronology_gates"]["exact_expected_ids_changed"])
        self.assertEqual(result["retired_uid_present_at_ids"], [9])

    def test_swapping_retired_motors_among_replacement_ids_rejected(self):
        for mid, prior in ((6, 9), (9, 10), (10, 6)):
            self.uid(self.new, mid, f"{prior:016x}")
        self.pin()
        self.assertEqual(self.blocked_export()["retired_uid_present_at_ids"], [6, 9, 10])

    def test_duplicate_uids_in_either_capture_reject_before_output(self):
        for capture in (self.old, self.new):
            with self.subTest(capture=capture.name):
                saved = {p: p.read_bytes() for p in capture.iterdir()}
                self.uid(capture, 1, f"{2:016x}")
                self.pin()
                with self.assertRaises(ValueError):
                    inventory.save_inventory(self.config_path, self.root / "invalid")
                self.assertFalse((self.root / "invalid").exists())
                for path, raw in saved.items():
                    path.write_bytes(raw)

    def test_mixed_summary_and_event_identity_is_rejected(self):
        edit(self.new, lambda r, m: m["summary"]["identities"].update({"6": f"{6:016x}"}))
        self.pin()
        with self.assertRaisesRegex(ValueError, "identities"):
            self.build()

    def test_missing_unknown_or_repeated_identity_records_rejected(self):
        baseline = {p: p.read_bytes() for p in self.new.iterdir()}
        for kind in ("missing", "unknown", "repeated"):
            def change(records, meta):
                event = next(r for r in records if r["kind"] == "motor_parameter" and
                             r["parameter"] == "identity")
                if kind == "missing":
                    records.remove(event)
                elif kind == "unknown":
                    event["motor_id"] = 13
                else:
                    event["motor_id"] = 2
            edit(self.new, change)
            self.pin()
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.build()
            for path, raw in baseline.items():
                path.write_bytes(raw)

    def test_both_hashes_pinned_for_both_captures(self):
        original = copy.deepcopy(self.config)
        for label in ("previous", "current"):
            for name in ("events_sha256", "summary_sha256"):
                self.config = copy.deepcopy(original)
                self.config[label][name] = "0" * 64
                self.write_config()
                with self.subTest(label=label, name=name), self.assertRaisesRegex(ValueError, "hash mismatch"):
                    inventory.save_inventory(self.config_path, self.root / "invalid")
                self.assertFalse((self.root / "invalid").exists())

    def test_reboot_monotonic_reset_permitted_but_wall_clock_must_follow(self):
        def reset_monotonic(records, _):
            for r in records:
                r["monotonic_ns"] -= 4_500_000_000
                if "request_monotonic_ns" in r:
                    r["request_monotonic_ns"] -= 4_500_000_000
        edit(self.new, reset_monotonic)
        self.pin()
        self.assertTrue(self.build()["inventory_candidate_validated"])
        edit(self.new, lambda records, _: [r.update(wall_time_ns=r["wall_time_ns"]-5_000_000_000)
                                           for r in records])
        self.pin()
        self.assertFalse(self.blocked_export()["identity_and_chronology_gates"]["current_capture_after_previous"])

    def test_unstable_new_position_blocks_without_clipping(self):
        self.values(6, "position", [.6, .6, .63])
        result = self.blocked_export()
        row = next(q for q in result["current_observation_quality"] if q["motor_id"] == 6)
        self.assertAlmostEqual(row["position_span_rad"], .03)
        self.assertFalse(row["gates"]["position_span"])
        self.assertFalse(row["gates"]["adjacent_step"])

    def test_current_outlier_on_unchanged_motor_blocks_export(self):
        self.values(11, "current", [0, 0, -.051])
        result = self.blocked_export()
        self.assertIn({"motor_id": 11, "gate": "current"}, result["blockers"])

    def test_velocity_voltage_are_preserved_without_inventing_drive_permission(self):
        self.values(9, "velocity", [0, .12, 0])
        self.values(9, "voltage", [34, 35, 36])
        result = self.build()
        self.assertTrue(result["inventory_candidate_validated"])
        row = next(q for q in result["current_observation_quality"] if q["motor_id"] == 9)
        self.assertEqual(row["sampling_issues"], ["velocity"])
        self.assertEqual(row["voltage_minmax_V"], [34, 36])
        self.assertFalse(result["approved_for_runtime"])

    def test_missing_core_evidence_and_incomplete_capture_rejected(self):
        original = {p: p.read_bytes() for p in self.new.iterdir()}
        for failure in ("current", "stability", "incomplete"):
            def change(records, meta):
                if failure == "current":
                    records.remove(next(r for r in records if r["kind"] == "motor_parameter"
                                        and r["parameter"] == "current"))
                elif failure == "stability":
                    del meta["summary"]["positions"]["6"]["samples"]
                else:
                    meta["status"] = "INCOMPLETE"
            edit(self.new, change)
            self.pin()
            with self.subTest(failure=failure), self.assertRaises(ValueError):
                self.build()
            for path, raw in original.items():
                path.write_bytes(raw)

    def test_nonfinite_and_wrong_unit_current_rejected(self):
        original = {p: p.read_bytes() for p in self.new.iterdir()}
        for field, value in (("value", float("nan")), ("value", float("inf")), ("unit", "mA")):
            def change(records, _):
                next(r for r in records if r["kind"] == "motor_parameter"
                     and r["parameter"] == "current")[field] = value
            edit(self.new, change)
            self.pin()
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.build()
            for path, raw in original.items():
                path.write_bytes(raw)

    def test_strict_config_rejects_duplicates_missing_boolean_and_permission(self):
        original = copy.deepcopy(self.config)
        for field, value in (("schema_version", True), ("expected_changed_ids", [6, 6, 9, 10]),
                             ("expected_changed_ids", [9, 6, 10]), ("expected_changed_ids", [True, 9, 10]),
                             ("expected_changed_ids", []), ("approved_for_runtime", True)):
            self.config = {**original, field: value}
            self.write_config()
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.build()
        self.config = copy.deepcopy(original)
        self.config["current"].pop("summary_sha256")
        self.write_config()
        with self.assertRaises(ValueError):
            self.build()
        self.config_path.write_text(json.dumps(original)[:-1]+',"schema_version":1}')
        with self.assertRaises(ValueError):
            self.build()

    def test_existing_output_symlink_and_git_rejected(self):
        output = self.root / "out"
        inventory.save_inventory(self.config_path, output)
        before = {p: p.read_bytes() for p in output.iterdir()}
        with self.assertRaises(FileExistsError):
            inventory.save_inventory(self.config_path, output)
        self.assertEqual(before, {p: p.read_bytes() for p in before})
        git = self.root / "git"
        git.mkdir()
        (git / ".git").write_text("gitdir: elsewhere")
        link = self.root / "link"
        link.symlink_to(git, target_is_directory=True)
        for path in (git / "result", link / "result", link):
            with self.assertRaises(ValueError):
                inventory.save_inventory(self.config_path, path)

    def test_cli_opens_no_devices_and_stdout_has_no_uids(self):
        stdout = io.StringIO()
        with patch("singularitydog_hw.can_readonly.ReadOnlyCAN") as can, contextlib.redirect_stdout(stdout):
            self.assertEqual(inventory.main(["--config", str(self.config_path),
                                           "--output", str(self.root / "out")]), 0)
        can.assert_not_called()
        parsed = json.loads(stdout.getvalue())
        self.assertTrue(parsed["expected_uids_exported"])
        self.assertFalse(parsed["output_allowed"])
        self.assertNotIn("000000000000", stdout.getvalue())


if __name__ == "__main__":
    unittest.main()
