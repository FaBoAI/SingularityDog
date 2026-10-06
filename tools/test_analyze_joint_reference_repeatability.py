"""Synthetic file-only checks; no robot calibration or hardware access."""

import contextlib
from dataclasses import asdict
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import analyze_joint_reference_repeatability as tool


class RepeatabilityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.axes = {i: tool.AxisCalibration(
            i, f"{i:016x}", 1, 0., -.5, .5, 0., "a" * 64) for i in tool.IDS}
        self.profile = {"schema": "singularitydog.angle-calibration-review-profile.v1",
                        "assembly_revision": "SYNTHETIC", "approved_for_runtime": False,
                        "motor_output_available": False, "axes": []}
        for axis in self.axes.values():
            row = asdict(axis)
            del row["calibration_sha256"]
            self.profile["axes"].append(row)
        self.manifest = {"schema": tool.INPUT_SCHEMA, "references": [], "return_trials": []}

    def tearDown(self):
        self.temp.cleanup()

    def save(self, name, data):
        path = self.root / name
        path.write_text(json.dumps(data, allow_nan=False))
        return {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    def capture(self, name, raw=.1, boot="A", time=10, epoch="NOT_INFERRED_FROM_JETSON_BOOT"):
        data = {"status": "RECORDED_REVIEW_REQUIRED", "errors": [], "boot_id": boot,
                "motor_power_epoch": epoch, "motor_output_allowed": False, "angle_wrap_applied": False,
                "plan": {"allowed_can_types": [0, 17]},
                "identities": {str(i): {"mcu_uid_hex": f"{i:016x}"} for i in tool.IDS},
                "telemetry": {"rows": {str(i): {"run_mode": 0, "current": 0., "position_span_deg": .01,
                    "median_position_rad": raw, "position_samples": [
                        {"rad": raw, "request_monotonic_ns": time, "reply_monotonic_ns": time + 1}]}
                    for i in tool.IDS}}}
        return self.save(name, data)

    def reference(self, pin, ids=None, method="bubble"):
        return {"label": "SYNTHETIC", "method": method, "ids": ids or [3], "capture": pin}

    def analyze(self):
        return tool.analyze(self.manifest, self.profile, self.axes, tool.Sources(self.root))

    def trial(self, back=.12, boot="A", epoch="NOT_INFERRED_FROM_JETSON_BOOT"):
        trial = {"label": "SYNTHETIC", "motor_id": 3,
                 "baseline": self.capture("base.json", .1, time=10),
                 "moved": self.capture("move.json", .3, time=20),
                 "returned": self.capture("return.json", back, boot=boot, time=30, epoch=epoch)}
        self.manifest["return_trials"] = [trial]
        return trial

    def test_only_explicit_reference_axes_are_counted(self):
        self.manifest["references"] = [self.reference(self.capture("a.json"))]
        report = self.analyze()
        self.assertEqual([r["motor_id"] for r in report["reference_rows"]], [3])
        self.assertIsNone(report["reference_groups"][0]["observed_reference_span_deg"])

    def test_same_boot_spread_is_direct_and_not_absolute_accuracy(self):
        self.manifest["references"] = [self.reference(self.capture("a.json", .1)),
                                       self.reference(self.capture("b.json", .12))]
        group = self.analyze()["reference_groups"][0]
        self.assertAlmostEqual(group["observed_reference_span_deg"], math.degrees(.02))
        self.assertIsNone(group["absolute_accuracy_bound_rad"])
        self.assertIsNone(group["pairs"][0]["orientation_delta_deg"])

    def test_cross_boot_turn_is_displayed_separately(self):
        self.manifest["references"] = [self.reference(self.capture("a.json", .1)),
                                       self.reference(self.capture("b.json", .12 + 2 * math.pi, "B"))]
        group = self.analyze()["reference_groups"][0]
        self.assertEqual(group["pairs"][0]["full_turn_difference"], 1)
        self.assertAlmostEqual(group["observed_reference_span_deg"], math.degrees(.02))
        self.assertFalse(group["pairs"][0]["epoch_branch_binding_created"])

    def test_same_boot_turn_jump_is_retained_not_wrapped(self):
        self.manifest["references"] = [self.reference(self.capture("a.json", .1)),
                                       self.reference(self.capture("b.json", .12 + 2 * math.pi))]
        group = self.analyze()["reference_groups"][0]
        self.assertIsNone(group["observed_reference_span_deg"])
        self.assertGreater(group["pairs"][0]["direct_raw_delta_deg"], 360)
        self.assertTrue(group["ambiguity_or_discontinuity_present"])

    def test_methods_are_not_pooled(self):
        self.manifest["references"] = [self.reference(self.capture("a.json", .1)),
                                       self.reference(self.capture("b.json", .2), method="camera")]
        self.assertEqual(len(self.analyze()["reference_groups"]), 2)

    def test_duplicate_capture_is_not_independent_repeat(self):
        pin = self.capture("a.json")
        self.manifest["references"] = [self.reference(pin), self.reference(pin)]
        with self.assertRaisesRegex(ValueError, "independent reference"):
            self.analyze()

    def test_cross_boot_half_turn_is_ambiguous(self):
        self.manifest["references"] = [self.reference(self.capture("a.json", math.pi / 2)),
                                       self.reference(self.capture("b.json", -math.pi / 2, "B"))]
        group = self.analyze()["reference_groups"][0]
        self.assertTrue(group["ambiguity_or_discontinuity_present"])
        self.assertIsNone(group["observed_reference_span_deg"])

    def test_wide_cross_boot_group_is_ambiguous_in_every_anchor_order(self):
        pins = [self.capture("zero.json", 0., "A"),
                self.capture("plus.json", math.radians(170), "B"),
                self.capture("minus.json", math.radians(-170), "C")]
        for order in ([0, 1, 2], [1, 2, 0], [2, 0, 1]):
            self.manifest["references"] = [self.reference(pins[i]) for i in order]
            group = self.analyze()["reference_groups"][0]
            self.assertTrue(group["ambiguity_or_discontinuity_present"])
            self.assertIsNone(group["observed_reference_span_deg"])
            self.assertEqual(len(group["pairs"]), 3)

    def test_direct_return_retains_failed_return(self):
        self.trial(back=.2)
        row = self.analyze()["return_trials"][0]
        self.assertAlmostEqual(row["direct_raw_return_deg_by_id"]["3"], math.degrees(.1))
        self.assertFalse(row["return_tolerance_or_approval_created"])

    def test_return_full_turn_is_not_hidden(self):
        self.trial(back=.1 + 2 * math.pi)
        row = self.analyze()["return_trials"][0]
        self.assertAlmostEqual(row["direct_raw_return_deg_by_id"]["3"], 360)
        self.assertTrue(row["half_turn_or_larger_present"])

    def test_return_to_moved_half_turn_is_retained(self):
        trial = self.trial(back=math.radians(-100))
        trial["baseline"] = self.capture("zero.json", 0., time=10)
        trial["moved"] = self.capture("plus.json", math.radians(100), time=20)
        row = self.analyze()["return_trials"][0]
        self.assertAlmostEqual(row["direct_raw_recovery_deg_by_id"]["3"], -200)
        self.assertTrue(row["half_turn_or_larger_present"])

    def test_return_arithmetic_overflow_rejected(self):
        trial = self.trial(back=1e308)
        trial["baseline"] = self.capture("negative.json", -1e308, time=10)
        with self.assertRaisesRegex(ValueError, "Finite numeric"):
            self.analyze()

    def test_return_rejects_changed_boot(self):
        self.trial(boot="B")
        with self.assertRaisesRegex(ValueError, "boot changed"):
            self.analyze()

    def test_return_rejects_mixed_unknown_and_known_power_epoch(self):
        self.trial(epoch="known-1")
        with self.assertRaisesRegex(ValueError, "power epoch changed"):
            self.analyze()

    def test_return_rejects_bad_time_order(self):
        trial = self.trial()
        trial["moved"] = self.capture("move2.json", .2, time=5)
        with self.assertRaisesRegex(ValueError, "timestamp order"):
            self.analyze()

    def test_numeric_gap_is_minimum_touch_not_origin_bound(self):
        self.manifest["references"] = [self.reference(self.capture("a.json", 0.))]
        self.manifest["current"] = self.capture("current.json", .6)
        report = self.analyze()
        row = report["current_by_id"]["3"]
        self.assertAlmostEqual(row["minimum_offset_change_rad_to_touch_numeric_interval"], -.1)
        self.assertIsNone(row["range_failure_explained_by_absolute_origin_error"])
        self.assertEqual(row["existing_periodic_branch_candidates"], [])
        self.assertFalse(report["output_allowed"])

    def test_uid_change_rejected(self):
        pin = self.capture("a.json")
        data = json.loads((self.root / pin["path"]).read_text())
        data["identities"]["3"]["mcu_uid_hex"] = "b" * 16
        self.manifest["references"] = [self.reference(self.save("b.json", data))]
        with self.assertRaisesRegex(ValueError, "UID differs"):
            self.analyze()

    def test_source_sha_mismatch_rejected(self):
        pin = self.capture("a.json")
        pin["sha256"] = "b" * 64
        self.manifest["references"] = [self.reference(pin)]
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            self.analyze()

    def test_symlink_rejected(self):
        pin = self.capture("a.json")
        (self.root / "link.json").symlink_to("a.json")
        pin["path"] = "link.json"
        self.manifest["references"] = [self.reference(pin)]
        with self.assertRaisesRegex(ValueError, "Regular source"):
            self.analyze()

    def test_source_mutation_recheck_rejected(self):
        pin = self.capture("a.json")
        sources = tool.Sources(self.root)
        sources.capture(pin, self.axes)
        (self.root / "a.json").write_text("{}")
        with self.assertRaisesRegex(ValueError, "changed during analysis"):
            sources.recheck()

    def test_second_source_read_cannot_replace_prior_path_pin(self):
        pin = self.capture("a.json")
        sources = tool.Sources(self.root)
        sources.capture(pin, self.axes)
        new_pin = self.capture("a.json", .2)
        with self.assertRaisesRegex(ValueError, "conflicting hashes"):
            sources.capture(new_pin, self.axes)

    def test_invalid_ids_and_empty_selection_rejected(self):
        pin = self.capture("a.json")
        for ids in ([], [True], [3, 3], [13]):
            self.manifest["references"] = [{**self.reference(pin), "ids": ids}]
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                self.analyze()

    def test_operator_observation_must_bind_actual_capture(self):
        pin = self.capture("a.json")
        obs = self.save("obs.json", {"l_pose_observed": True, "power_event": {
            "inferred_from_boot": False, "boot_id": "A", "capture_sha256": {"l": "b" * 64}}})
        self.manifest["references"] = [{**self.reference(pin), "observation": obs}]
        with self.assertRaisesRegex(ValueError, "does not bind"):
            self.analyze()

    def test_operator_observation_cannot_swap_trial_phases(self):
        trial = self.trial()
        trial["observation"] = self.save("swapped.json", {"power_event": {
            "inferred_from_boot": False, "boot_id": "A", "capture_sha256": {
                "baseline": trial["baseline"]["sha256"], "moved": trial["returned"]["sha256"],
                "return": trial["moved"]["sha256"]}}})
        with self.assertRaisesRegex(ValueError, "does not bind"):
            self.analyze()

    def test_cli_private_output_and_no_overwrite(self):
        self.manifest["references"] = [self.reference(self.capture("a.json"))]
        self.save("input.json", self.manifest)
        self.save("profile.json", self.profile)
        output = self.root / "result.json"
        args = ["--profile", str(self.root / "profile.json"), "--input", str(self.root / "input.json"), "--output", str(output)]
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tool.main(args), 0)
        self.assertEqual(output.stat().st_mode & 0o777, 0o600)
        data = json.loads(output.read_text())
        self.assertFalse(data["profile_changed"])
        self.assertFalse(data["calibration_approved_for_runtime"])
        before = output.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            tool.main(args)
        self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
