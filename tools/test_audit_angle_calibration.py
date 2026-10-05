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
    def test_current_capture_rejects_boolean_protocol_and_quiet_state_values(self):
        with tempfile.TemporaryDirectory() as folder:
            baseline = json.loads(capture_fixture(Path(folder)).read_text())
            for field, value in (("allowed_can_types", [False, 17]),
                                 ("allowed_can_types", [0., 17]),
                                 ("run_mode", False), ("run_mode", 0.),
                                 ("current", False), ("position_span_deg", False),
                                 ("median_position_rad", True)):
                changed = json.loads(json.dumps(baseline))
                if field == "allowed_can_types":
                    changed["plan"][field] = value
                else:
                    changed["telemetry"]["rows"]["5"][field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    tool.current_values(changed)

    def test_current_capture_rejects_malformed_or_nonfinite_measurements(self):
        with tempfile.TemporaryDirectory() as folder:
            baseline = json.loads(capture_fixture(Path(folder)).read_text())
            for field in ("current", "position_span_deg", "median_position_rad"):
                for value in (None, "0.0", [], float("nan"), float("inf"), 10**400):
                    changed = json.loads(json.dumps(baseline))
                    changed["telemetry"]["rows"]["5"][field] = value
                    with self.subTest(field=field, kind=type(value).__name__), self.assertRaises(ValueError):
                        tool.current_values(changed)
            for field in ("plan", "telemetry"):
                changed = json.loads(json.dumps(baseline))
                changed[field] = []
                with self.subTest(field=field), self.assertRaises(ValueError):
                    tool.current_values(changed)
            for branch in ("identities", "telemetry"):
                changed = json.loads(json.dumps(baseline))
                rows = changed[branch] if branch == "identities" else changed[branch]["rows"]
                rows["5"] = []
                with self.subTest(branch=branch), self.assertRaises(ValueError):
                    tool.current_values(changed)

    def test_reference_template_cannot_transfer_invalid_raw_or_uid(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, contracts = tool.load_profile(profile)
            baseline = json.loads(capture_fixture(root).read_text())
            for value in ("0.2", True, None, float("inf")):
                changed = json.loads(json.dumps(baseline))
                changed["telemetry"]["rows"]["4"]["median_position_rad"] = value
                with self.subTest(value=value), self.assertRaises(ValueError):
                    tool.reference_observation_template([(baseline, "a"*64), (changed, "b"*64)], contracts, [4])
            for value in (None, "", "a"*15, "A"*16, False):
                changed = json.loads(json.dumps(baseline))
                changed["identities"]["4"]["mcu_uid_hex"] = value
                with self.subTest(uid=value), self.assertRaises(ValueError):
                    tool.current_values(changed)

    def test_observed_quiet_rejects_invalid_history_current(self):
        tool.observed_quiet({"run_mode": 0, "current_A": 0.}, "fixture", current_key="current_A")
        for row in ([], {"run_mode": False, "current_A": 0.},
                    {"run_mode": 0, "current_A": False},
                    {"run_mode": 0, "current_A": "0"}):
            with self.subTest(row=row), self.assertRaises(ValueError):
                tool.observed_quiet(row, "fixture", current_key="current_A")

    def test_reference_template_reuses_selected_axis_raws_and_leaves_external_measurements_blank(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            baseline = capture_fixture(root)
            moved_data = json.loads(baseline.read_text())
            moved_data["telemetry"]["rows"]["4"]["median_position_rad"] += math.radians(10)
            moved = root / "moved.json"
            moved.write_text(json.dumps(moved_data))
            template = root / "references.json"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(["--profile", str(profile), "--capture", str(baseline),
                    "--reference-capture", str(moved), "--reference-id", "4",
                    "--reference-template-output", str(template), "--output", str(root / "audit.json")]), 0)
            refs = json.loads(template.read_text())
            self.assertEqual(set(refs), {"4"})
            self.assertEqual(len(refs["4"]), 2)
            self.assertEqual(refs["4"][0]["raw_rad"], .1)
            self.assertAlmostEqual(refs["4"][1]["raw_rad"], .1 + math.radians(10))
            self.assertEqual(refs["4"][1]["source_sha256"], hashlib.sha256(moved.read_bytes()).hexdigest())
            for row in refs["4"]:
                for key in ("model_rad", "uncertainty_rad", "relative_output_shaft_observed", "physical_angle_method"):
                    self.assertIsNone(row[key])
            # External measurements make the same file consumable by the existing fit.
            for index, row in enumerate(refs["4"]):
                row.update(model_rad=math.radians(index * 10), uncertainty_rad=math.radians(.5),
                    relative_output_shaft_observed=True, physical_angle_method="angle_gauge",
                    motor_power_epoch="synthetic-known-power-epoch")
            template.write_text(json.dumps(refs))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(["--profile", str(profile), "--capture", str(baseline),
                    "--references", str(template), "--output", str(root / "fit-audit.json")]), 0)
            report = json.loads((root / "fit-audit.json").read_text())
            fit = report["reference_fits_by_id"]["4"]
            self.assertTrue(fit["profile_sign_matches"])
            self.assertAlmostEqual(fit["offset_difference_rad_unwrapped"], -.1)
            self.assertFalse(fit["profile_changed"])
            self.assertEqual(report["batch_review_plan"]["external_reference_fit_review_ids"], [4])
            self.assertEqual(report["batch_review_plan"]["external_reference_not_supplied_ids"], [1,2,3,5,6,7,8,9,10,11,12])
            self.assertFalse(report["batch_review_plan"]["dynamic_type2_scale_verified_by_this_audit"])
            self.assertFalse(report["approved_for_runtime"])

    def test_reference_template_rejects_duplicate_source_uid_boot_and_epoch_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile, _ = profile_fixture(root)
            _, contracts = tool.load_profile(profile)
            baseline = json.loads(capture_fixture(root).read_text())
            with self.assertRaisesRegex(ValueError, "repeat one capture"):
                tool.reference_observation_template([(baseline, "a"*64), (baseline, "a"*64)], contracts)
            for field, value, pattern in (("boot_id", "other-boot", "share one boot"),
                ("motor_power_epoch", "other-power", "epoch label changed"),
                ("uid", "f"*16, "UID changed")):
                changed = json.loads(json.dumps(baseline))
                if field == "uid":
                    changed["identities"]["4"]["mcu_uid_hex"] = value
                else:
                    changed[field] = value
                with self.subTest(field=field), self.assertRaisesRegex(ValueError, pattern):
                    tool.reference_observation_template([(baseline, "a"*64), (changed, "b"*64)], contracts, [4])

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

    def test_fresh_capture_hash_replaces_current_without_losing_profile_provenance(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile_path, _ = profile_fixture(root)
            profile = json.loads(profile_path.read_text())
            original_sources = {"current": "a" * 64, "candidate": "b" * 64,
                                "camera_l_by_leg": {"FR": "c" * 64}}
            profile["source_sha256"] = original_sources
            profile_path.write_text(json.dumps(profile))
            original_profile_bytes = profile_path.read_bytes()
            capture = capture_fixture(root)
            current_hash = hashlib.sha256(capture.read_bytes()).hexdigest()
            output, exported = root / "report.json", root / "profile-copy.json"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.main(["--profile", str(profile_path), "--capture", str(capture),
                                            "--output", str(output), "--profile-output", str(exported)]), 0)
            report = json.loads(output.read_text())
            self.assertEqual(report["source_sha256"]["current"], current_hash)
            self.assertEqual(report["current_capture_sha256"], current_hash)
            self.assertEqual(report["profile_source_sha256"], original_sources)
            self.assertEqual(report["source_sha256"]["candidate"], original_sources["candidate"])
            self.assertEqual(report["source_sha256"]["camera_l_by_leg"], original_sources["camera_l_by_leg"])
            self.assertEqual(json.loads(exported.read_text()), profile)
            self.assertEqual(profile_path.read_bytes(), original_profile_bytes)

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
