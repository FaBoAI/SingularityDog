"""Synthetic reviewed-input integration and tamper rejection; no hardware I/O."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import imu_calibration_review as review
from singularitydog_hw import policy_live_profile as profiles
from singularitydog_hw.policy_output_model import LivePolicyModel
from singularitydog_hw.policy_motion_envelope import MotionSample
import test_imu_calibration as capture_fixtures
from singularitydog_hw.imu_calibration import GRAVITY
from test_policy_observer import bias_candidate, mount, make, snapshot
from test_policy_live_profile import synthetic_fixture, _write

try:
    import torch
except ImportError:
    torch = None


class ReviewedAccelerationTests(unittest.TestCase):
    def setUp(self):
        # Reuse the full raw/readback/trim capture fixture, rather than mock the
        # evidence audit or assert correctness from the implementation formula.
        self.captures = capture_fixtures.AuditedCaptureTests("test_cli_audits_directories_without_runtime_approval")
        self.captures.setUp()
        self.addCleanup(self.captures.doCleanups)
        self.root = self.captures.root
        self.candidate = self.captures.calibrate()
        self.candidate_path = self.root / "candidate.json"
        self.mount = mount()
        self.bias = bias_candidate()
        (self.root / "bias.json").write_text(json.dumps(self.bias))
        (self.root / "mount.json").write_text(json.dumps(self.mount))
        self.template = review.write_template(self.root / "bias.json", self.candidate_path,
            self.root / "mount.json", self.root / "bias-review-template.json")
        self.accepted = copy.deepcopy(self.template)
        document = self.accepted["accel_calibration_review"]
        document.update({key: True for key in review.PHYSICAL_KEYS})
        document.update(external_reference_uncertainty_rad=.005,
            corrected_norm_min_m_s2=9.4, corrected_norm_max_m_s2=10.2,
            review={"reviewer": "SYNTHETIC UNIT TEST ONLY", "reviewed_at": "2026-10-05T12:00:00+09:00",
                    "decision": review.DECISION, "rationale": "Synthetic fixture, no real robot."})

    def load(self, bias=None):
        return review.reviewed_acceleration(bias or self.accepted,
            self.mount["R_body_from_sensor"], enabled=True)

    def measured(self):
        return [(force / self.candidate["accel"]["scale"][i])
                + self.candidate["accel"]["bias_m_s2"][i]
                for i, force in enumerate((0., 0., -GRAVITY))]

    def test_template_is_unreviewed_and_default_does_not_apply(self):
        self.assertFalse(self.template["approved_for_runtime"])
        self.assertTrue(all(self.template["accel_calibration_review"][key] is False
                            for key in review.PHYSICAL_KEYS))
        self.assertIsNone(review.reviewed_acceleration(self.template, self.mount["R_body_from_sensor"]))
        with self.assertRaisesRegex(ValueError, "reviewer|scope"):
            self.load(self.template)
        with self.assertRaises(ValueError):
            review.write_template(self.root / "bias.json", self.candidate_path,
                self.root / "mount.json", self.root / "bias-review-template.json")

    def test_fixed_diagonal_correction_preserves_input_and_actual_magnitude(self):
        loaded = self.load()
        measured = self.measured()
        original = list(measured)
        corrected, norm = loaded.correct(measured)
        for actual, expected in zip(corrected, (0., 0., -GRAVITY)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(norm, GRAVITY)
        self.assertEqual(measured, original)
        # A physical magnitude change must remain visible, not normalize to g.
        changed = list(measured)
        changed[2] -= .1 / loaded.scale[2]
        self.assertAlmostEqual(loaded.correct(changed)[1], GRAVITY + .1)
        with self.assertRaisesRegex(ValueError, "outside reviewed range"):
            loaded.correct([0., 0., -20.])

    def test_physical_review_mount_and_instrument_uncertainty_required(self):
        for key in review.PHYSICAL_KEYS:
            changed = copy.deepcopy(self.accepted)
            changed["accel_calibration_review"][key] = False
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "physical review"):
                self.load(changed)
        for uncertainty in (None, 0, False, -.001, math.radians(3.1)):
            changed = copy.deepcopy(self.accepted)
            changed["accel_calibration_review"]["external_reference_uncertainty_rad"] = uncertainty
            with self.subTest(uncertainty=uncertainty), self.assertRaisesRegex(ValueError, "reference uncertainty"):
                self.load(changed)
        with self.assertRaisesRegex(ValueError, "mount differs"):
            review.reviewed_acceleration(self.accepted, [[1,0,0],[0,1,0],[0,0,1]], enabled=True)

    def test_mount_booleans_cannot_impersonate_numeric_rotation(self):
        changed = copy.deepcopy(self.accepted)
        changed["accel_calibration_review"]["R_body_from_sensor"][0] = [True, False, False]
        with self.assertRaisesRegex(ValueError, "mount entries"):
            self.load(changed)

    def test_candidate_recomputation_preserves_json_types(self):
        variants = (("schema_version", True), ("requires_physical_validation", 1),
                    ("gyro", {**self.candidate["gyro"], "scale_estimated": 0}),
                    ("validation", {**self.candidate["validation"],
                                    "heldout_is_independent_physical_validation": 0}))
        for key, value in variants:
            changed = copy.deepcopy(self.candidate)
            changed[key] = value
            self.candidate_path.write_text(json.dumps(changed))
            self.accepted["accel_calibration_review"]["candidate"]["sha256"] = hashlib.sha256(self.candidate_path.read_bytes()).hexdigest()
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "recomputation differs"):
                self.load()

    def test_candidate_change_after_reaudit_is_rejected_before_final_load(self):
        original = review.baseline._load_capture
        calls = 0
        def concurrent_change(*args, **kwargs):
            nonlocal calls
            loaded = original(*args, **kwargs)
            calls += 1
            # First twelve calls recompute the candidate, next six check the
            # independent samples against the named review's norm bounds.
            if calls == 18:
                self.candidate_path.write_bytes(self.candidate_path.read_bytes() + b" ")
            return loaded
        with patch.object(review.baseline, "_load_capture", side_effect=concurrent_change), \
                self.assertRaisesRegex(ValueError, "candidate changed during final"):
            self.load()

    def test_ground_reconstruction_preserves_raw_guard_and_checks_both_norms(self):
        from singularitydog_hw import ground_trial_review as ground
        from singularitydog_hw.policy_observer import _bias
        from test_ground_trial_review import fixture
        base = self.root / "ground-fixture"
        base.mkdir()
        profile, documents, _, report = fixture(base)
        runtime = report["runtime_report"]
        rotation = self.mount["R_body_from_sensor"]
        bias = _bias(self.accepted)["bias_sensor_rad_s"]
        selected = self.load()
        expected_body = [sum(row[j] * (0., 0., -GRAVITY)[j] for j in range(3)) for row in rotation]
        expected_gyro = [sum(row[j] * -bias[j] for j in range(3)) for row in rotation]
        raw_norm = math.hypot(*self.measured())
        for cycle in runtime["cycles"]:
            cycle["imu"]["accel_m_s2"] = self.measured()
            cycle["imu_body"] = dict(frame="body", source_monotonic_ns=cycle["imu"]["read_started_monotonic_ns"],
                accel_m_s2=expected_body, gyro_rad_s=expected_gyro, tilt_rad=0., accel_norm_m_s2=raw_norm,
                raw_accel_sensor_m_s2=self.measured(), raw_accel_norm_m_s2=raw_norm,
                corrected_accel_norm_m_s2=GRAVITY, accel_bias_subtracted=True, accel_scale_corrected=True,
                reviewed_accel_calibration=selected.provenance())
        feedback, voltages = ground._journal(runtime, profile,
            tested_firmware=documents["hardware_review"]["device_watchdog"])
        def cycles():
            return ground._cycles(runtime, profile, feedback, voltages, rotation, bias, selected)
        with self.assertRaisesRegex(ValueError, "acceleration norm limit"):
            cycles()
        profile["imu_accel_norm_min_m_s2"], profile["imu_accel_norm_max_m_s2"] = 10.4, 10.8
        self.assertEqual(cycles()["cycles"], 111)
        saved = copy.deepcopy(runtime["cycles"][0])
        for field, value in (("raw_accel_norm_m_s2", GRAVITY), ("corrected_accel_norm_m_s2", raw_norm),
                             ("accel_bias_subtracted", 1), ("reviewed_accel_calibration", {**selected.provenance(), "grants_motor_output": 0})):
            runtime["cycles"][0] = copy.deepcopy(saved)
            runtime["cycles"][0]["imu_body"][field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "provenance mismatch"):
                cycles()
        runtime["cycles"][0] = copy.deepcopy(saved)
        del runtime["cycles"][0]["imu_body"]
        with self.assertRaisesRegex(ValueError, "both norm records"):
            cycles()
        runtime["cycles"][0] = copy.deepcopy(saved)
        runtime["cycles"][0]["imu"]["accel_scale_corrected"] = True
        with self.assertRaisesRegex(ValueError, "raw frame"):
            cycles()

    def test_candidate_tampering_is_rejected_even_if_new_hash_is_claimed(self):
        changed = copy.deepcopy(self.candidate)
        changed["accel"]["scale"][0] += .01
        self.candidate_path.write_text(json.dumps(changed))
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.load()
        self.accepted["accel_calibration_review"]["candidate"]["sha256"] = hashlib.sha256(self.candidate_path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, "recomputation differs"):
            self.load()

    def test_original_independent_capture_tampering_is_rejected(self):
        path = self.captures.validation_paths["x+"]
        metadata, rows = self.captures.fixtures[path]
        rows[0]["accel_m_s2"][0] += .1
        self.captures.save()
        with self.assertRaisesRegex(ValueError, "raw/SI"):
            self.load()

    def test_runtime_input_source_change_is_rejected(self):
        changed = review.source_hashes()
        changed["singularitydog_hw/policy_observer.py"] = "0" * 64
        with patch.object(review, "source_hashes", return_value=changed), \
                self.assertRaisesRegex(ValueError, "input source SHA256"):
            self.load()

    def test_consistently_edited_raw_capture_still_fails_pinned_recomputation(self):
        path = self.captures.validation_paths["x+"]
        metadata, rows = self.captures.fixtures[path]
        rows[0]["raw_accel"][0] += 1
        rows[0]["accel_m_s2"][0] = rows[0]["raw_accel"][0] * metadata["configuration"]["accel_m_s2_per_lsb"]
        capture_fixtures.refresh_summary(metadata, rows)
        self.captures.save()
        with self.assertRaisesRegex(ValueError, "recomputation differs"):
            self.load()

    def test_unapproved_same_capture_holdout_cannot_replace_independent_set(self):
        changed = copy.deepcopy(self.candidate)
        changed["validation"]["independent_capture_gates_passed"] = False
        self.candidate_path.write_text(json.dumps(changed))
        self.accepted["accel_calibration_review"]["candidate"]["sha256"] = hashlib.sha256(self.candidate_path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, "independent captures"):
            self.load()

    def test_observer_explicit_application_records_both_norms_and_owns_configuration(self):
        run = make(gyro_bias_candidate=self.accepted, apply_reviewed_accel_calibration=True)
        run.reset_run(1_000_000_000, warmup_completed=True)
        source = snapshot()
        source["imu"]["accel_m_s2"] = self.measured()
        original = copy.deepcopy(source)
        # Mutating caller metadata after construction cannot change the fit.
        self.accepted["accel_calibration_review"]["corrected_norm_max_m_s2"] = 9.81
        result = run.consume(source)
        self.assertEqual(source, original)
        provenance = result["provenance"]
        self.assertAlmostEqual(provenance["raw_accel_norm_m_s2"], math.hypot(*self.measured()))
        self.assertAlmostEqual(provenance["corrected_accel_norm_m_s2"], GRAVITY)
        self.assertTrue(provenance["accel_bias_subtracted"])
        self.assertEqual(provenance["reviewed_accel_calibration"]["corrected_norm_bounds_m_s2"], [9.4, 10.2])
        self.assertFalse(result["approved_for_runtime"])
        self.assertEqual(result["inputs"]["gravity_body_unit"], [0., 0., -1.])

    def test_observer_missing_review_and_bad_timestamp_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "review required"):
            make(gyro_bias_candidate=self.bias, apply_reviewed_accel_calibration=True)
        run = make(gyro_bias_candidate=self.accepted, apply_reviewed_accel_calibration=True)
        run.reset_run(1_000_000_000, warmup_completed=True)
        source = snapshot()
        source["imu"]["read_finished_ns"] = source["tick_ns"] + 1
        with self.assertRaisesRegex(ValueError, "Noncausal"):
            run.consume(source)

    def test_profile_load_binds_reviewed_bias_and_diagnostic_selection(self):
        base = self.root / "profile-fixture"
        base.mkdir()
        data, documents, pins = synthetic_fixture(base)
        data.update(schema=profiles.SCHEMA_V3, telemetry_cadence=profiles.CADENCE_PRE_ENABLE,
                    cadence_source_sha256=profiles.cadence_source_hashes(), apply_reviewed_accel_calibration=True)
        documents["bias"] = self.accepted
        data["artifacts"]["bias"] = _write(base / "bias.json", documents["bias"])
        timing = documents["pipeline_diagnostic"]
        timing["input_sha256"]["gyro_bias"] = data["artifacts"]["bias"]["sha256"]
        timing["plan"].update(apply_reviewed_accel_calibration=True,
            telemetry_cadence=data["telemetry_cadence"], cadence_source_sha256=data["cadence_source_sha256"])
        # Existing V3 fixture helper defines the exact legacy cadence contract.
        from test_policy_live_profile import ProfileTests
        fixture = ProfileTests("test_template_only_is_not_permission_to_output")
        fixture.base, fixture.data, fixture.docs = base, data, documents
        fixture.save(bind_review=True)
        with patch.object(profiles.shadow, "SOURCE_HASHES", pins):
            loaded = profiles.load_profile(base / "profile.json")
            self.assertTrue(loaded["output_allowed"])
            self.assertEqual(loaded["artifacts"]["bias"]["sha256"], data["artifacts"]["bias"]["sha256"])
            documents["pipeline_diagnostic"]["plan"]["apply_reviewed_accel_calibration"] = False
            fixture.save(bind_review=True)
            with self.assertRaisesRegex(ValueError, "acceleration calibration selection"):
                profiles.load_profile(base / "profile.json")

    @unittest.skipIf(torch is None, "CPU PyTorch is unavailable")
    def test_live_model_preserves_raw_norm_guard_and_uses_corrected_vector(self):
        from test_policy_output_model import TensorPolicy
        base = self.root / "model-fixture"
        base.mkdir()
        data, documents, _ = synthetic_fixture(base)
        data.update(schema=profiles.SCHEMA_V3, output_allowed=True, apply_reviewed_accel_calibration=True)
        documents["bias"] = self.accepted
        data["artifacts"]["bias"] = _write(base / "bias.json", self.accepted)
        for value in data["artifacts"].values():
            value["path"] = str(base / value["path"])
        model = LivePolicyModel(data, policy=TensorPolicy(), torch_module=torch)
        imu = dict(frame="sensor", accel_m_s2=self.measured(), gyro_rad_s=[.01,.02,.03],
                   read_started_monotonic_ns=998_000_000, read_finished_monotonic_ns=999_000_000)
        sample = MotionSample(tuple(-.8 if i % 3 == 1 else .4 if i % 3 == 2 else 0.
                                    for i in range(1, 13)), (0.,)*12, (0.,)*12, (25.,)*12, .998)
        with self.assertRaisesRegex(ValueError, "gravity-proxy norm outside"):
            model.validate_inputs(sample, imu, 1_000_000_000)
        # Raw monitoring remains explicit and separately reviewed.
        data["imu_accel_norm_min_m_s2"], data["imu_accel_norm_max_m_s2"] = 10.4, 10.8
        values = model.validate_inputs(sample, imu, 1_000_000_000)
        self.assertEqual(values[1], [0., 0., -1.])
        self.assertAlmostEqual(model.last_validation["corrected_accel_norm_m_s2"], GRAVITY)
        self.assertAlmostEqual(model.last_validation["raw_accel_norm_m_s2"], math.hypot(*self.measured()))
        for flag in ("calibration_applied", "accel_bias_subtracted", "accel_scale_corrected"):
            changed = {**imu, flag: True}
            with self.subTest(flag=flag), self.assertRaisesRegex(ValueError, "Uncorrected"):
                model.validate_inputs(sample, changed, 1_000_000_000)
        # Final model construction rechecks both wrapper SHA and raw sources.
        (base / "bias.json").write_text("{}")
        with self.assertRaisesRegex(ValueError, "bias changed"):
            LivePolicyModel(data, policy=TensorPolicy(), torch_module=torch)


if __name__ == "__main__":
    unittest.main()
