"""Offline tests for operator-paced yaw retakes; no device process is started."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import imu_yaw_retake as retake
from singularitydog_hw.imu_fixed_mount_baseline import BaselineError
from test_imu_fixed_mount_baseline import synthetic_capture, refresh_summary


def write_capture(path, metadata, rows):
    path.mkdir()
    (path / "summary.json").write_text(json.dumps(metadata))
    (path / "events.jsonl").write_text("\n".join(json.dumps(row) for row in
        [{"kind": "capture_metadata", **metadata["plan"]}, *rows]) + "\n")


class YawRetakeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name, seed, start in (("static-a", 11, 100), ("static-b", 22, 140)):
            write_capture(self.root / name, *synthetic_capture(seed, start))
        self.source = self.root / "prior-manifest.json"
        self.manifest = {"schema_version": 1,
                         "stationary": {"a": "static-a", "b": "static-b", "operator_confirmed": True},
                         "operator_assertions": {"qdd_power_off": True, "body_supported": True},
                         "movements": []}
        self.save_source()

    def save_source(self):
        self.source.write_text(json.dumps(self.manifest))

    def fake_yaw(self, path, *, capture_seconds=35, opposite=False, rebooted=False):
        metadata, rows = synthetic_capture(33, 10 if rebooted else 180)
        for i, row in enumerate(rows):
            t = i * .01
            direction = 1 if opposite else -1
            delta = direction * 1048 if 1 <= t < 3 else -direction * 1048 if 5 <= t < 7 else 0
            row["raw_gyro"][2] += delta
            row["gyro_rad_s"] = [v * metadata["configuration"]["gyro_rad_s_per_lsb"]
                                   for v in row["raw_gyro"]]
        refresh_summary(metadata, rows)
        write_capture(path, metadata, rows)
        return {"movement": "turn_left", "capture": path.name,
                "outbound_s": [1, 3], "return_s": [5, 7],
                "operator_direction_confirmed": False,
                "marker_monotonic_ns": {name: rows[0]["monotonic_ns"] + int(t * 1e9)
                                         for name, t in zip((n for n, _ in retake.MARKERS), (1, 3, 5, 7))},
                "marker_method": "operator_enter_at_motion_boundaries"}

    def test_plan_is_yaw_only_and_never_opens_device_or_static_source(self):
        target = self.root / "retake"
        with patch.object(retake.subprocess, "Popen") as proc, contextlib.redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(retake.main(["--reuse-static-from", str(self.root / "missing.json"),
                                          "--output", str(target)]), 0)
            proc.assert_not_called()
        plan = json.loads(stdout.getvalue())
        self.assertEqual(plan["captures"], ["turn_left"])
        self.assertEqual(plan["yaw_recording_seconds"], 60)
        self.assertEqual(len(plan["operator_boundary_markers"]), 4)
        self.assertFalse(target.exists())
        self.assertFalse(plan["runtime_approval"])

    def test_source_stationary_provenance_is_retained_without_copying(self):
        stationary, _, source = retake._stationary_source(self.source)
        self.assertEqual(stationary["a"], str((self.root / "static-a").resolve()))
        self.assertEqual(stationary["b"], str((self.root / "static-b").resolve()))
        self.assertEqual(source["static_a"]["directory"], stationary["a"])
        self.assertEqual(len(source["static_b"]["events_sha256"]), 64)

    def test_rejects_unconfirmed_or_ineligible_static(self):
        self.manifest["stationary"]["operator_confirmed"] = False
        self.save_source()
        with self.assertRaisesRegex(BaselineError, "stationarity"):
            retake._stationary_source(self.source)
        self.manifest["stationary"]["operator_confirmed"] = True
        self.manifest["operator_assertions"]["qdd_power_off"] = False
        self.save_source()
        with self.assertRaisesRegex(BaselineError, "power-off"):
            retake._stationary_source(self.source)

    def test_actual_markers_set_windows_and_bad_boundaries_are_rejected(self):
        first = 100_000_000_000
        names = [name for name, _ in retake.MARKERS]
        marks = {name: first + int(t * 1e9) for name, t in zip(names, (1, 4, 7, 10))}
        windows = retake.marker_windows(marks, first, first + 20_000_000_000)
        self.assertEqual(windows, {"outbound_s": [1, 4], "return_s": [7, 10]})
        late = dict(marks)
        late["return_end"] = first + 17_000_000_000
        with self.assertRaisesRegex(ValueError, "still tail after return_end 0.00s"):
            retake.marker_windows(late, first, first + 17_000_000_000)
        reversed_marks = dict(marks)
        reversed_marks["return_start"] = first + 2_000_000_000
        with self.assertRaisesRegex(ValueError, "ordered"):
            retake.marker_windows(reversed_marks, first, first + 20_000_000_000)
        too_long = dict(marks)
        too_long["outbound_end"] = first + 17_000_000_000
        too_long["return_start"] = first + 18_000_000_000
        too_long["return_end"] = first + 19_000_000_000
        with self.assertRaisesRegex(ValueError, "outbound window 16.00s"):
            retake.marker_windows(too_long, first, first + 20_000_000_000)

    def test_first_outbound_prompt_waits_for_initial_still_period(self):
        proc = Mock()
        proc.poll.return_value = None
        first = 100_000_000_000
        with patch.object(retake.time, "monotonic_ns", side_effect=[first + 100_000_000,
                                                                    first + 700_000_000,
                                                                    first + 1_000_000_000]), \
             patch.object(retake.time, "monotonic", return_value=0), \
             patch.object(retake.time, "sleep") as sleep:
            retake._wait_still_after(first, retake.INITIAL_STILL_SECONDS, proc, 10)
        self.assertEqual(sleep.call_count, 2)

    def test_retake_preserves_static_and_passes_signed_yaw_only(self):
        with patch.object(retake, "capture_yaw", side_effect=self.fake_yaw), \
             patch("builtins.input", side_effect=["", "y"]), contextlib.redirect_stdout(io.StringIO()):
            review = retake.run_session(self.root / "retake", reuse_static_from=self.source)
        self.assertTrue(review["turn_left_axis_sign"])
        self.assertTrue(review["gyro_bias_candidate"])
        self.assertFalse(review["operator_motion_audit_discrepancy"])
        self.assertFalse(review["approved_for_runtime"])
        manifest = json.loads((self.root / "retake" / "manifest.json").read_text())
        self.assertEqual(manifest["stationary"]["a"], str((self.root / "static-a").resolve()))
        self.assertEqual(manifest["movements"][0]["marker_method"], "operator_enter_at_motion_boundaries")
        audit = json.loads((self.root / "retake" / "audit.json").read_text())
        self.assertFalse(audit["checks"]["nose_up_axis_sign"])
        self.assertFalse(audit["checks"]["left_side_up_axis_sign"])
        self.assertEqual(audit["stationary"]["provenance"]["a"]["events_sha256"],
                         manifest["stationary_source"]["static_a"]["events_sha256"])

    def test_operator_yes_with_opposite_gyro_is_explicit_discrepancy(self):
        with patch.object(retake, "capture_yaw", side_effect=lambda path, **kw: self.fake_yaw(path, opposite=True)), \
             patch("builtins.input", side_effect=["", "y"]), contextlib.redirect_stdout(io.StringIO()):
            review = retake.run_session(self.root / "opposite", reuse_static_from=self.source)
        self.assertFalse(review["turn_left_axis_sign"])
        self.assertTrue(review["operator_motion_audit_discrepancy"])
        self.assertIn("outbound_sign", review["failed_gates"])

    def test_rebooted_motion_cannot_claim_same_static_bias_epoch(self):
        with patch.object(retake, "capture_yaw", side_effect=lambda path, **kw: self.fake_yaw(path, rebooted=True)), \
             patch("builtins.input", side_effect=["", "y"]), contextlib.redirect_stdout(io.StringIO()):
            review = retake.run_session(self.root / "rebooted", reuse_static_from=self.source)
        self.assertFalse(review["turn_left_axis_sign"])
        self.assertTrue(review["operator_motion_audit_discrepancy"])
        self.assertIn("movement must follow static B", review["audit_error"])

    def test_interruption_terminates_child_without_emitting_complete_manifest(self):
        proc = Mock()
        proc.poll.return_value = None
        with patch.object(retake.subprocess, "Popen", return_value=proc), \
             patch.object(retake.guided, "_first_sample", side_effect=KeyboardInterrupt), \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(KeyboardInterrupt):
                retake.capture_yaw(self.root / "interrupt")
        proc.terminate.assert_called_once()
        proc.wait.assert_called_once_with(timeout=10)

    def test_invalid_marker_timing_still_persists_all_pressed_times(self):
        first = 100_000_000_000
        proc = Mock(returncode=0)
        proc.poll.side_effect = [None, 0, 0]
        names = [name for name, _ in retake.MARKERS]
        marks = [(name, first + int(t * 1e9)) for name, t in zip(names, (1, 17, 18, 19))]
        rows = [{"monotonic_ns": first}, {"monotonic_ns": first + 20_000_000_000}]
        with patch.object(retake.subprocess, "Popen", return_value=proc), \
             patch.object(retake.guided, "_first_sample", return_value=first), \
             patch.object(retake, "_wait_still_after") as still, \
             patch.object(retake, "_mark", side_effect=marks), \
             patch.object(retake.baseline, "_load_capture", return_value=({"restore_status": "restored"}, rows, {}, {})), \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, "outbound window 16.00s"):
                retake.capture_yaw(self.root / "bad-timing", capture_seconds=60)
        sidecar = [json.loads(line) for line in (self.root / "yaw-markers.jsonl").read_text().splitlines()]
        self.assertEqual([row["kind"] for row in sidecar],
                         ["first_sample", *["operator_marker"] * 4, "last_sample"])
        self.assertEqual([row["name"] for row in sidecar[1:5]], names)
        self.assertEqual(sidecar[-1]["elapsed_from_first_sample_s"], 20)
        self.assertEqual(still.call_count, 2)
        self.assertEqual(still.call_args_list[1].args[1], retake.BETWEEN_MOVEMENTS_STILL_SECONDS)


if __name__ == "__main__":
    unittest.main()
