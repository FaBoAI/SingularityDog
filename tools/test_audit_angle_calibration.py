"""CLI replay tests use synthetic files and never import a hardware transport."""

import contextlib
from dataclasses import asdict
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("audit_angle_calibration", Path(__file__).with_name("audit_angle_calibration.py"))
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def profile_fixture(root):
    evidence = root / "physical-review.txt"
    evidence.write_text("Synthetic review fixture, not real robot calibration\n")
    digest = hashlib.sha256(evidence.read_bytes()).hexdigest()
    axes = []
    for mid in tool.IDS:
        row = asdict(tool.AxisCalibration(
            mid, f"{mid:016x}", 1, 0., -.5, .5, .01, "a" * 64,
            True, True, True, digest, digest, digest))
        del row["calibration_sha256"]
        axes.append(row)
    profile = {"schema": tool.PROFILE_SCHEMA, "assembly_revision": "fixture-1",
               "physical_uncertainty_known": True, "axes": axes,
               "evidence_files": {digest: evidence.name},
               "approved_for_runtime": False, "motor_output_available": False}
    path = root / "profile.json"
    path.write_text(json.dumps(profile))
    return path, evidence


def capture_fixture(root):
    capture = {"status": "RECORDED_REVIEW_REQUIRED", "errors": [],
               "boot_id": "fixture-boot", "motor_power_epoch": "NOT_INFERRED_FROM_JETSON_BOOT",
               "motor_output_allowed": False, "angle_wrap_applied": False,
               "plan": {"allowed_can_types": [0, 17]},
               "identities": {str(mid): {"mcu_uid_hex": f"{mid:016x}"} for mid in tool.IDS},
               "telemetry": {"rows": {str(mid): {"run_mode": 0, "current": 0.,
                   "position_span_deg": .01, "median_position_rad": .1} for mid in tool.IDS}}}
    path = root / "capture.json"
    path.write_text(json.dumps(capture))
    return path


class AuditCLITests(unittest.TestCase):
    def test_profile_roundtrip_and_output_still_not_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            capture = capture_fixture(root)
            output = root / "report.json"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(["--profile", str(profile), "--capture", str(capture),
                                            "--output", str(output)]), 0)
            result = json.loads(output.read_text())
            self.assertEqual(result["status"], "EVIDENCE_READY_FOR_EPOCH_BINDING")
            self.assertFalse(result["epoch_binding_created"])
            self.assertFalse(result["approved_for_runtime"])
            self.assertEqual(result["batch_review_plan"]["direction_without_historical_record_ids"], [])
            self.assertEqual(output.stat().st_mode & 0o777, 0o600)

    def test_mutated_evidence_or_unknown_zero_error_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, evidence = profile_fixture(root)
            evidence.write_text("changed")
            with self.assertRaisesRegex(ValueError, "evidence file changed"):
                tool.load_profile(profile)
            profile, _ = profile_fixture(root)
            data = json.loads(profile.read_text())
            data["physical_uncertainty_known"] = False
            profile.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "error bound"):
                tool.load_profile(profile)

    def test_duplicate_json_and_nonfinite_rejected(self):
        for source in ('{"a":1,"a":2}', '{"a":NaN}'):
            with self.assertRaises(ValueError):
                tool.strict_json(source)

    def test_new_output_does_not_replace_a_previous_record(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "record.json"
            tool.write_private(output, {"old": 1})
            with self.assertRaises(FileExistsError):
                tool.write_private(output, {"new": 2})
            self.assertEqual(json.loads(output.read_text()), {"old": 1})

    def test_uid_mismatch_is_reported_for_only_changed_axis(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            capture = capture_fixture(root)
            data = json.loads(capture.read_text())
            data["identities"]["11"]["mcu_uid_hex"] = "f" * 16
            capture.write_text(json.dumps(data))
            output = root / "report.json"
            with contextlib.redirect_stdout(io.StringIO()):
                tool.main(["--profile", str(profile), "--capture", str(capture), "--output", str(output)])
            report = json.loads(output.read_text())
            self.assertEqual(report["uid_changed_ids"], [11])
            self.assertTrue(report["rows_by_id"]["1"]["evidence_ready_for_epoch_binding"])

    def test_diagnostic_policy_candidate_embeds_current_branch_without_approval(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile_path, _ = profile_fixture(root)
            profile = json.loads(profile_path.read_text())
            # Diagnostics may use nominal zero/sign/range hypotheses. They remain unapproved.
            for row in profile["axes"]:
                row.update(zero_reviewed=False, direction_reviewed=False, physical_limits_reviewed=False)
            profile_path.write_text(json.dumps(profile))
            _, contracts = tool.load_profile(profile_path)
            current = json.loads(capture_fixture(root).read_text())
            current["telemetry"]["rows"]["3"]["median_position_rad"] += 2 * math.pi
            candidate = tool.policy_input_candidate(contracts, current, capture_sha256="e" * 64)
            row = candidate["candidates"][2]
            self.assertEqual(row["diagnostic_branch_turns_embedded_in_offset"], 1)
            self.assertAlmostEqual(row["offset_candidate_rad"], -2 * math.pi)
            self.assertAlmostEqual(candidate["model_rad_at_source_capture_by_id"]["3"], .1)
            self.assertFalse(candidate["approved_for_runtime"])
            self.assertFalse(candidate["epoch_binding_created"])
            self.assertFalse(candidate["physical_joint_limits_verified"])
            self.assertEqual(candidate["source_capture_sha256"], "e" * 64)
            from singularitydog_hw.policy_shadow import validate_calibration
            self.assertEqual(len(validate_calibration(candidate)), 12)

    def test_diagnostic_policy_candidate_rejects_uid_mismatch_and_out_of_range(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile_path, _ = profile_fixture(root)
            _, contracts = tool.load_profile(profile_path)
            current = json.loads(capture_fixture(root).read_text())
            current["telemetry"]["rows"]["3"]["median_position_rad"] = 1.
            with self.assertRaisesRegex(ValueError, "unique in-range"):
                tool.policy_input_candidate(contracts, current, capture_sha256="e" * 64)
            current["identities"]["11"]["mcu_uid_hex"] = "f" * 16
            with self.assertRaisesRegex(ValueError, "UID mismatch"):
                tool.policy_input_candidate(contracts, current, capture_sha256="e" * 64)

    def test_batch_plan_separates_numeric_history_and_physical_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile_path, _ = profile_fixture(root)
            profile = json.loads(profile_path.read_text())
            for axis in profile["axes"]:
                axis.update(zero_reviewed=False, direction_reviewed=False,
                            physical_limits_reviewed=False)
            profile["physical_uncertainty_known"] = False
            profile_path.write_text(json.dumps(profile))
            _, contracts = tool.load_profile(profile_path)
            current = json.loads(capture_fixture(root).read_text())
            current["telemetry"]["rows"]["3"]["median_position_rad"] += 2 * math.pi
            raw, uids = tool.current_values(current)
            report = tool.audit_twelve_axes(contracts, raw, uids)
            report["motor_power_epoch"] = current["motor_power_epoch"]
            report["physical_uncertainty_known"] = False
            report["historical_direction_support_by_id"] = {
                str(mid): {"matches_current_hypothesis": mid != 10} for mid in tool.IDS}
            plan = tool.batch_review_plan(report)
            self.assertEqual(plan["numeric_branch_screen_pass_ids"], list(tool.IDS))
            self.assertEqual(plan["numeric_branch_turns_by_id"]["3"], 1)
            self.assertEqual(plan["historical_direction_conflicts_ids"], [10])
            self.assertEqual(len(plan["historical_direction_record_review_ids"]), 11)
            self.assertEqual(plan["physical_zero_error_review_ids"], list(tool.IDS))
            self.assertEqual(plan["physical_limit_review_ids"], list(tool.IDS))
            self.assertTrue(plan["motor_power_epoch_label_missing"])
            self.assertFalse(plan["motor_power_epoch_attestation_verified_by_this_audit"])
            self.assertTrue(plan["power_epoch_manifest_link_needed"])
            self.assertFalse(plan["approved_for_runtime"])
            self.assertFalse(plan["output_allowed"])
            report["motor_power_epoch"] = "operator-attested:unverified-label"
            plan_with_label = tool.batch_review_plan(report)
            self.assertFalse(plan_with_label["motor_power_epoch_label_missing"])
            self.assertFalse(plan_with_label["motor_power_epoch_attestation_verified_by_this_audit"])

    def test_batch_plan_requires_complete_historical_support_if_any(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile_path, _ = profile_fixture(root)
            _, contracts = tool.load_profile(profile_path)
            current = json.loads(capture_fixture(root).read_text())
            raw, uids = tool.current_values(current)
            report = tool.audit_twelve_axes(contracts, raw, uids)
            report["historical_direction_support_by_id"] = {
                "10": {"matches_current_hypothesis": False}}
            with self.assertRaisesRegex(ValueError, "all twelve"):
                tool.batch_review_plan(report)


if __name__ == "__main__":
    unittest.main()
