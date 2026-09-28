"""Fixed-mount evidence replay and failed-claim checks; synthetic sensor I/O only."""
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import imu_commissioning_audit as audit
from singularitydog_hw import imu_commissioning_capture as capture
from singularitydog_hw.imu_fixed_mount_baseline import BaselineError
from test_imu_fixed_mount_baseline import synthetic_capture, refresh_summary


class CommissioningAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.write("a", *synthetic_capture(11, 100))
        self.write("b", *synthetic_capture(22, 140))
        self.manifest = {"schema_version": 1, "stationary": {"a": "a", "b": "b", "operator_confirmed": True},
                         "movements": []}
        for index, name in enumerate(audit.MOVEMENTS):
            meta, rows = synthetic_capture(33+index, 180+index*40)
            sensor_axis, sign = {"nose_up": (0, -1), "left_side_up": (1, 1), "turn_left": (2, -1)}[name]
            for i, row in enumerate(rows):
                t = i*.01
                delta = sign*1048 if 1 <= t < 3 else -sign*1048 if 5 <= t < 7 else 0
                row["raw_gyro"][sensor_axis] += delta
                row["gyro_rad_s"] = [v*meta["configuration"]["gyro_rad_s_per_lsb"] for v in row["raw_gyro"]]
            refresh_summary(meta, rows)
            self.write(name, meta, rows)
            self.manifest["movements"].append({"movement": name, "capture": name, "outbound_s": [1, 3],
                                                "return_s": [5, 7], "operator_direction_confirmed": True})

    def write(self, name, meta, rows):
        path = self.root/name
        path.mkdir(exist_ok=True)
        (path/"summary.json").write_text(json.dumps(meta))
        (path/"events.jsonl").write_text("\n".join(json.dumps(row) for row in
                                                  [{"kind": "capture_metadata", **meta["plan"]}, *rows])+"\n")

    def run_audit(self):
        return audit.audit_manifest(self.manifest, root=self.root)

    def test_correct_body_axis_signs_and_bias_order_with_norm_anomaly(self):
        result = self.run_audit()
        for name in audit.MOVEMENTS:
            self.assertTrue(result["checks"][name+"_axis_sign"])
        self.assertTrue(result["checks"]["gyro_bias_candidate"])
        self.assertFalse(result["checks"]["raw_accel_norm_within_3_percent"])
        self.assertFalse(result["checks"]["accel_bias_and_scale_identified"])
        self.assertFalse(result["checks"]["absolute_level_and_heading_validated"])
        for flag in ("approved_for_runtime", "automatically_applied", "hardware_opened", "motor_output_available"):
            self.assertIs(result[flag], False)
        expected = audit.rotate(audit.MOUNT_ROTATION, result["stationary"]["comparison"]["gyro_b_residual_after_subtracting_a_mean_rad_s"])
        for a, b in zip(expected, result["heldout_body_diagnostic"]["gyro_body_corrected_mean_rad_s"]):
            self.assertAlmostEqual(a, b)
        self.assertLess(result["heldout_body_diagnostic"]["gravity_direction_candidate_body"][2], -.99)

    def test_reflection_is_rejected_before_audit(self):
        self.manifest["mount_candidate"] = audit.mount_candidate()
        self.manifest["mount_candidate"]["R_body_from_sensor"][2][2] = 1.
        with self.assertRaisesRegex(ValueError, "determinant"):
            self.run_audit()

    def test_heldout_gyro_max_is_per_sample_norm_not_norm_of_mean(self):
        _, rows, _, _ = audit.baseline._load_capture(self.root/"b")
        result = self.run_audit()
        bias = result["gyro_bias_candidate_sensor_rad_s"]
        expected = max(math.hypot(*(v-b for v, b in zip(row["gyro_rad_s"], bias))) for row in rows)
        heldout = result["heldout_body_diagnostic"]
        self.assertAlmostEqual(heldout["gyro_body_corrected_norm_max_rad_s"], expected)
        self.assertGreater(expected, math.hypot(*heldout["gyro_body_corrected_mean_rad_s"]))

    def test_opposite_direction_yields_failure_not_axis_flip(self):
        self.manifest["movements"][0]["outbound_s"] = [5, 7]
        self.manifest["movements"][0]["return_s"] = [8, 10]
        r = self.run_audit()["movements"][0]
        self.assertFalse(r["axis_sign_supported"])
        self.assertIn("outbound_sign", r["failed_gates"])

    def test_unconfirmed_stationarity_cannot_emit_bias_or_motion_pass(self):
        self.manifest["stationary"]["operator_confirmed"] = False
        r = self.run_audit()
        self.assertIsNone(r["gyro_bias_candidate_sensor_rad_s"])
        self.assertIsNone(r["heldout_body_diagnostic"])
        self.assertFalse(any(m["axis_sign_supported"] for m in r["movements"]))

    def test_missing_or_empty_summary_is_reported_and_not_reconstructed(self):
        (self.root/"turn_left"/"summary.json").write_text("")
        r = self.run_audit()
        self.assertFalse(r["checks"]["turn_left_axis_sign"])
        self.assertTrue(r["checks"]["nose_up_axis_sign"])
        self.assertEqual((self.root/"turn_left"/"summary.json").read_text(), "")

    def test_duplicate_movement_labels_rejected(self):
        self.manifest["movements"].append(copy.deepcopy(self.manifest["movements"][0]))
        with self.assertRaisesRegex(BaselineError, "duplicate"):
            self.run_audit()

    def test_missing_direction_assertion_is_not_pass(self):
        del self.manifest["movements"][0]["operator_direction_confirmed"]
        r = self.run_audit()["movements"][0]
        self.assertFalse(r["axis_sign_supported"])
        self.assertIn("operator_direction_confirmed", r["failed_gates"])

    def test_overlapping_or_out_of_capture_motion_windows_rejected(self):
        for windows in (([1, 3], [2, 4]), ([1, 3], [10, 14])):
            self.manifest["movements"][0]["outbound_s"], self.manifest["movements"][0]["return_s"] = windows
            r = self.run_audit()["movements"][0]
            self.assertFalse(r["axis_sign_supported"])
            self.assertIn("error", r)

    def test_readback_conversion_corruption_never_passes(self):
        p = self.root/"turn_left"/"events.jsonl"
        data = [json.loads(line) for line in p.read_text().splitlines()]
        data[101]["gyro_rad_s"][0] += 1
        p.write_text("\n".join(json.dumps(row) for row in data)+"\n")
        r = self.run_audit()["movements"][2]
        self.assertFalse(r["axis_sign_supported"])
        self.assertIn("raw/SI", r["error"])

    def test_gross_constant_rotation_is_not_accepted_as_bias(self):
        meta, rows = synthetic_capture(11, 100)
        for row in rows:
            row["raw_gyro"][0] += 2000
            row["gyro_rad_s"][0] = row["raw_gyro"][0]*meta["configuration"]["gyro_rad_s_per_lsb"]
        refresh_summary(meta, rows)
        self.write("a", meta, rows)
        self.assertFalse(self.run_audit()["checks"]["gyro_bias_candidate"])

    def test_output_exclusive_and_non_approving(self):
        p = self.root/"manifest.json"
        p.write_text(json.dumps(self.manifest))
        out = self.root/"audit.json"
        audit.write_audit(p, out)
        self.assertEqual(out.stat().st_mode & 0o777, 0o600)
        with self.assertRaises(FileExistsError):
            audit.write_audit(p, out)


class GuidedCaptureTests(unittest.TestCase):
    def test_yaw_only_plan_never_opens_device_or_marks_other_axes(self):
        with tempfile.TemporaryDirectory() as root:
            target = Path(root)/"not-created"
            with patch.object(capture.subprocess, "Popen") as proc, contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(capture.main(["--output", str(target), "--yaw-only"]), 0)
                proc.assert_not_called()
            self.assertFalse(target.exists())
        plan = json.loads(out.getvalue())
        self.assertEqual(plan["captures"], ["static-a", "static-b", "turn_left"])
        self.assertEqual(plan["minimum_recording_seconds"], 69)
        self.assertFalse(plan["runtime_approval"])

    def test_yaw_only_session_keeps_pitch_and_roll_unresolved(self):
        calls = []
        def fake_capture(path, *, movement=None):
            calls.append(path.name)
            meta, rows = synthetic_capture(10+len(calls), 100+40*len(calls))
            if movement == "turn_left":
                for i, row in enumerate(rows):
                    t = i*.01
                    delta = -1048 if 1 <= t < 3 else 1048 if 5 <= t < 7 else 0
                    row["raw_gyro"][2] += delta
                    row["gyro_rad_s"] = [v*meta["configuration"]["gyro_rad_s_per_lsb"]
                                         for v in row["raw_gyro"]]
                refresh_summary(meta, rows)
            path.mkdir()
            (path/"summary.json").write_text(json.dumps(meta))
            (path/"events.jsonl").write_text("\n".join(json.dumps(row) for row in
                [{"kind": "capture_metadata", **meta["plan"]}, *rows])+"\n")
            if movement:
                return {"movement": movement, "capture": path.name,
                        "outbound_s": [1, 3], "return_s": [5, 7],
                        "operator_direction_confirmed": False}
        with tempfile.TemporaryDirectory() as root, patch.object(capture, "capture_one", side_effect=fake_capture), \
                patch("builtins.input", side_effect=["", "", "y", "", "y"]), \
                contextlib.redirect_stdout(io.StringIO()):
            result = capture.run_session(Path(root)/"session", movements=("turn_left",))
            self.assertEqual(calls, ["static-a", "static-b", "turn_left"])
            self.assertTrue(result["checks"]["gyro_bias_candidate"])
            self.assertTrue(result["checks"]["turn_left_axis_sign"])
            self.assertFalse(result["checks"]["nose_up_axis_sign"])
            self.assertFalse(result["checks"]["left_side_up_axis_sign"])
            self.assertFalse(result["approved_for_runtime"])

    def test_plan_and_missing_assertions_never_spawn_device_process(self):
        with patch.object(capture.subprocess, "Popen") as proc, contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(capture.main(["--output", "/tmp/not-created-imu-test"]), 0)
            with self.assertRaises(SystemExit):
                capture.main(["--output", "/tmp/not-created-imu-test", "--execute"])
            proc.assert_not_called()

    def test_actual_prompt_times_and_late_prompt_rejection(self):
        start = 100_000_000_000
        times = {key: start+int((target+.01)*1e9) for target, key, _ in capture.PROMPTS}
        result = capture.movement_windows(times, start)
        self.assertAlmostEqual(result["outbound_s"][0], 3.01)
        times["return_start"] += 500_000_000
        with self.assertRaisesRegex(RuntimeError, "Late"):
            capture.movement_windows(times, start)
        del times["outbound_start"]
        with self.assertRaisesRegex(RuntimeError, "Not all"):
            capture.movement_windows(times, start)

    def test_partial_json_tail_does_not_become_first_sample(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root)/"events.jsonl"
            p.write_text('{"kind":"capture_metadata"}\n{"kind":"imu"')
            self.assertIsNone(capture._first_sample(p))
            p.write_text('{"kind":"capture_metadata"}\n{"kind":"imu","monotonic_ns":123}\n')
            self.assertEqual(capture._first_sample(p), 123)

    def test_interruption_terminates_child_and_waits_for_restoration_path(self):
        proc = Mock()
        proc.poll.return_value = None
        with tempfile.TemporaryDirectory() as root, patch.object(capture.subprocess, "Popen", return_value=proc), \
                patch.object(capture.time, "sleep", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                capture.capture_one(Path(root)/"recording")
            proc.terminate.assert_called_once()
            proc.wait.assert_called_once_with(timeout=10)
            proc.kill.assert_not_called()

    def test_failed_child_does_not_parse_or_approve_measurements(self):
        proc = Mock(returncode=2)
        proc.poll.return_value = 2
        with tempfile.TemporaryDirectory() as root, patch.object(capture.subprocess, "Popen", return_value=proc), \
                patch.object(capture.baseline, "_load_capture") as load:
            with self.assertRaisesRegex(RuntimeError, "capture failed"):
                capture.capture_one(Path(root)/"recording")
            load.assert_not_called()

    def test_static_session_exports_existing_observer_bias_schema(self):
        from singularitydog_hw.policy_observer import _bias
        calls = []
        def fake_capture(path):
            calls.append(path.name)
            meta, rows = synthetic_capture(10+len(calls), 100+40*len(calls))
            path.mkdir()
            (path/"summary.json").write_text(json.dumps(meta))
            (path/"events.jsonl").write_text("\n".join(json.dumps(row) for row in
                [{"kind": "capture_metadata", **meta["plan"]}, *rows])+"\n")
        with tempfile.TemporaryDirectory() as root, patch.object(capture, "capture_one", side_effect=fake_capture), \
                patch("builtins.input", side_effect=["", "", "y"]), contextlib.redirect_stdout(io.StringIO()):
            destination = Path(root)/"session"
            result = capture.run_session(destination, static_only=True)
            self.assertEqual(calls, ["static-a", "static-b"])
            self.assertTrue(result["checks"]["gyro_bias_candidate"])
            bias = _bias(json.loads((destination/"gyro-bias-candidate.json").read_text()))
            self.assertEqual(bias["bias_sensor_rad_s"], result["gyro_bias_candidate_sensor_rad_s"])
            self.assertFalse(result["checks"]["turn_left_axis_sign"])


if __name__ == "__main__":
    unittest.main()
