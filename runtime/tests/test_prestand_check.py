"""Pure saved-capture screens: synthetic identities, no serial or policy use."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw import prestand_check as check
from singularitydog_hw.pose_record import ObservationError
from test_pose_record import snapshot, edit


class PrestandTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.capture = snapshot(self.root, "snapshot")
        self.cal = {"status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
                    "formula": "q_model = sign * raw + offset; rad; no wrapping",
                    "model_can_order_candidate": list(shadow.CAN_ORDER),
                    "identities": {str(i): f"{i:016x}" for i in range(1, 13)},
                    "candidates": []}
        for index, mid in enumerate(shadow.CAN_ORDER):
            sign = 1 if mid == 9 or mid % 2 else -1
            self.cal["candidates"].append({"motor_id": mid, "sign_candidate": sign,
                "offset_candidate_rad": check.NOMINAL[index] - sign * mid * .1})
        self.cal_path = self.root / "calibration.json"
        self.write_cal()
        row = next(r for r in self.cal["candidates"] if r["motor_id"] == 9)
        self.overlay = {"schema": "singularitydog.manual-zero-overlay-candidate.v1",
            "status": "OFFLINE_CANDIDATE_ONLY", "motor_id": 9, "leg": "RR",
            "model_joint": "RR_hip_joint", "identity": self.cal["identities"]["9"],
            "base_calibration_sha256": self.hash(self.cal_path), "candidate_sign": 1,
            "base_offset_rad": row["offset_candidate_rad"],
            "candidate_offset_rad": row["offset_candidate_rad"] + .1,
            "formula": "q_model = raw + candidate_offset_rad; radians; no wrapping",
            "nominal_model_angle_rad": 0, "reference_external_measurement": False,
            **dict.fromkeys(check.OVERLAY_FALSE_FLAGS, False)}
        self.overlay_path = self.root / "overlay.json"
        self.write_overlay()

    @staticmethod
    def hash(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def write_cal(self):
        self.cal_path.write_text(json.dumps(self.cal))

    def write_overlay(self, value=None):
        self.overlay_path.write_text(json.dumps(self.overlay if value is None else value))

    def report(self, overlay=False):
        return check.build_report(self.capture, self.cal_path,
                                  self.overlay_path if overlay else None)

    @staticmethod
    def joint(report, mid):
        return next(r for r in report["joint_table"] if r["motor_id"] == mid)

    def values(self, mid, parameter, values):
        def change(records, meta):
            replies = [r for r in records if r["kind"] == "motor_parameter"
                       and r["motor_id"] == mid and r["parameter"] == parameter]
            self.assertEqual(len(replies), len(values))
            for reply, value in zip(replies, values):
                reply["value"] = value
            if parameter == "position":
                meta["summary"]["positions"][str(mid)] = {
                    "samples": len(values), "mean": sum(values) / len(values),
                    "min": min(values), "max": max(values), "last": values[-1]}
        edit(self.capture, change)

    def test_nominal_screen_order_sign_and_permanent_no_output(self):
        result = self.report()
        self.assertEqual(result["status"], "OFFLINE_SCREEN_PASS")
        self.assertEqual(result["model_can_order"], shadow.CAN_ORDER)
        for i, row in enumerate(result["joint_table"]):
            self.assertAlmostEqual(row["q_candidate_median_rad"], check.NOMINAL[i])
            self.assertAlmostEqual(row["nominal_minus_candidate_rad"], 0)
            self.assertEqual(row["model_lower_rad"], shadow.LOWER[i])
            self.assertEqual(row["model_upper_rad"], shadow.UPPER[i])
            self.assertTrue(all(row["gates"].values()))
        for flag in ("motor_output_available", "motor_output_allowed", "output_allowed",
                     "approved_for_runtime", "live_readiness", "fresh_identity_match_verified",
                     "calibration_verified", "zero_verified", "sign_verified",
                     "controller_config_changed", "encoder_zero_written", "angle_wrapping_applied"):
            self.assertIs(result[flag], False)
        self.assertTrue(result["capture_uid_match_all12"])
        text = json.dumps(result)
        for forbidden in ('"mcu_uid_hex"', '"identity"', '"identities"', '"wire_hex"'):
            self.assertNotIn(forbidden, text)

    def test_overlay_changes_id9_only_and_preserves_original(self):
        original_bytes = self.cal_path.read_bytes()
        original, updated = self.report(), self.report(True)
        for before, after in zip(original["joint_table"], updated["joint_table"]):
            if before["motor_id"] == 9:
                self.assertAlmostEqual(after["q_candidate_median_rad"] - before["q_candidate_median_rad"], .1)
                self.assertEqual(after["q_original_median_rad"], before["q_original_median_rad"])
                self.assertAlmostEqual(after["nominal_minus_candidate_rad"], -.1)
            else:
                self.assertEqual(before, after)
        self.assertEqual(self.cal_path.read_bytes(), original_bytes)
        self.assertEqual(updated["sources"]["rr_overlay"]["sha256"], self.hash(self.overlay_path))

    def test_overlay_identity_hash_location_and_sign_rejected(self):
        changes = {"motor_id": [8, True], "leg": ["FR"], "model_joint": ["RR_thigh_joint"],
                   "identity": ["ffffffffffffffff"], "base_calibration_sha256": ["0" * 64],
                   "candidate_sign": [-1, True], "base_offset_rad": [0, True],
                   "formula": ["q = wrapped(raw)"], "nominal_model_angle_rad": [True, .1],
                   "status": ["APPROVED"], "schema": ["other"]}
        for key, values in changes.items():
            for value in values:
                with self.subTest(key=key, value=value):
                    bad = {**self.overlay, key: value}
                    self.write_overlay(bad)
                    with self.assertRaises(ValueError):
                        self.report(True)

    def test_every_overlay_approval_flag_must_be_explicitly_false(self):
        for key in (*check.OVERLAY_FALSE_FLAGS, "reference_external_measurement"):
            for value in (True, 0, None):
                with self.subTest(key=key, value=value):
                    self.write_overlay({**self.overlay, key: value})
                    with self.assertRaises(ValueError):
                        self.report(True)
            bad = dict(self.overlay)
            del bad[key]
            self.write_overlay(bad)
            with self.assertRaises(ValueError):
                self.report(True)

    def test_overlay_stays_bound_to_exact_original_calibration_bytes(self):
        self.cal_path.write_text(json.dumps(self.cal, indent=2))
        with self.assertRaisesRegex(ValueError, "hash"):
            self.report(True)

    def test_uid_mismatch_any_of_twelve_rejects_without_output(self):
        for mid in (1, 9, 12):
            original = self.cal["identities"][str(mid)]
            self.cal["identities"][str(mid)] = "ffffffffffffffff"
            self.write_cal()
            with self.assertRaisesRegex(ValueError, "UID mismatch"):
                check.save_report(self.capture, self.cal_path, self.root / "rejected.json")
            self.assertFalse((self.root / "rejected.json").exists())
            self.cal["identities"][str(mid)] = original

    def test_nonfinite_boolean_and_overflow_values_rejected(self):
        for value in (float("nan"), float("inf"), True, "0.1", None):
            self.write_overlay({**self.overlay, "candidate_offset_rad": value})
            with self.assertRaises((ValueError, OverflowError)):
                self.report(True)
        self.write_overlay({**self.overlay, "extra": {"nonfinite": 1e309}})
        with self.assertRaises(ValueError):
            self.report(True)
        # JSON permits 1e309 lexically, but parsed infinity must still be rejected.
        self.overlay_path.write_text(json.dumps(self.overlay)[:-1] + ',"extra":1e309}')
        with self.assertRaises(ValueError):
            self.report(True)
        self.cal["candidates"][0]["offset_candidate_rad"] = float("nan")
        self.write_cal()
        with self.assertRaises(ValueError):
            self.report()

    def test_duplicate_json_keys_rejected(self):
        self.overlay_path.write_text(json.dumps(self.overlay)[:-1] + ',"motor_id":9}')
        with self.assertRaises(ValueError):
            self.report(True)

    def test_out_of_range_is_reported_without_clamping_or_wrapping(self):
        self.overlay["candidate_offset_rad"] += 7
        self.write_overlay()
        result = self.report(True)
        row = self.joint(result, 9)
        self.assertGreater(row["q_candidate_median_rad"], 7)
        self.assertFalse(row["gates"]["model_range_all_samples"])
        self.assertEqual(result["status"], "OFFLINE_SCREEN_BLOCKED")
        self.assertIs(result["angle_wrapping_applied"], False)

    def test_all_sample_bounds_not_just_median(self):
        cal_row = next(r for r in self.cal["candidates"] if r["motor_id"] == 6)
        cal_row["offset_candidate_rad"] = .499 + .6  # sign -1
        self.write_cal()
        self.values(6, "position", [.6, .6, .598])
        row = self.joint(self.report(), 6)
        self.assertAlmostEqual(row["q_candidate_median_rad"], .499)
        self.assertAlmostEqual(row["q_candidate_minmax_rad"][1], .501)
        self.assertFalse(row["gates"]["model_range_all_samples"])
        self.assertTrue(row["gates"]["sampling_heuristics"])

    def test_voltage_bounds_and_current_outliers_not_averaged(self):
        self.values(4, "voltage", [35, 39, 43])
        self.values(4, "current", [0, -.05, .05])
        self.assertTrue(self.joint(self.report(), 4)["gates"]["voltage_all_samples"])
        self.assertTrue(self.joint(self.report(), 4)["gates"]["current_all_samples"])
        for volts in ([34.99, 39, 39], [39, 39, 43.01]):
            self.values(4, "voltage", volts)
            self.assertFalse(self.joint(self.report(), 4)["gates"]["voltage_all_samples"])
        self.values(4, "current", [0, 0, -.051])
        self.assertFalse(self.joint(self.report(), 4)["gates"]["current_all_samples"])

    def test_sampling_warnings_retained_and_block(self):
        self.values(8, "velocity", [0, .101, 0])
        self.values(8, "position", [.8, .8, .83])
        result = self.report()
        row = self.joint(result, 8)
        self.assertIn("velocity", row["sampling_issues"])
        self.assertIn("position_spread", row["sampling_issues"])
        self.assertFalse(row["gates"]["sampling_heuristics"])
        self.assertEqual(result["status"], "OFFLINE_SCREEN_BLOCKED")

    def test_existing_strict_snapshot_validation_is_used(self):
        edit(self.capture, lambda records, meta: meta.update(status="INCOMPLETE"))
        with self.assertRaises(ObservationError):
            self.report()

    def test_metadata_cannot_grant_permission(self):
        edit(self.capture, lambda records, meta: meta.update(output_allowed=True,
                         approved_for_runtime=True, live_readiness=True))
        self.overlay.update(output_allowed=True, live_readiness=True)
        self.write_overlay()
        result = self.report(True)
        self.assertIs(result["output_allowed"], False)
        self.assertIs(result["approved_for_runtime"], False)
        self.assertIs(result["live_readiness"], False)

    def test_private_new_output_hashes_and_no_overwrite(self):
        out = self.root / "private" / "report.json"
        result = check.save_report(self.capture, self.cal_path, out)
        self.assertEqual(json.loads(out.read_text()), result)
        self.assertEqual(stat.S_IMODE(out.stat().st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(out.parent.stat().st_mode), 0o700)
        self.assertEqual(result["sources"]["original_calibration_sha256"], self.hash(self.cal_path))
        self.assertEqual(result["sources"]["checker_sha256"], self.hash(Path(check.__file__)))
        before = out.read_bytes()
        with self.assertRaises(FileExistsError):
            check.save_report(self.capture, self.cal_path, out)
        self.assertEqual(before, out.read_bytes())
        dangling = self.root / "dangling"
        dangling.symlink_to(self.root / "missing")
        with self.assertRaises(FileExistsError):
            check.save_report(self.capture, self.cal_path, dangling)

    def test_git_output_and_symlink_into_git_rejected(self):
        git = self.root / "repo"
        git.mkdir()
        (git / ".git").write_text("gitdir: elsewhere")
        alias = self.root / "alias"
        alias.symlink_to(git, target_is_directory=True)
        for parent in (git, alias):
            with self.assertRaisesRegex(ValueError, "outside Git"):
                check.save_report(self.capture, self.cal_path, parent / "report.json")
        self.assertFalse((git / "report.json").exists())

    def test_cli_is_offline_and_has_no_force(self):
        args = ["--capture", str(self.capture), "--calibration", str(self.cal_path),
                "--output", str(self.root / "out.json")]
        stdout = io.StringIO()
        with patch("singularitydog_hw.can_readonly.ReadOnlyCAN") as can, \
                patch.object(shadow, "load_policy") as policy, contextlib.redirect_stdout(stdout):
            self.assertEqual(check.main(args), 0)
        can.assert_not_called()
        policy.assert_not_called()
        result = json.loads(stdout.getvalue())
        self.assertIs(result["output_allowed"], False)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
            check.main(args + ["--force"])
        self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
