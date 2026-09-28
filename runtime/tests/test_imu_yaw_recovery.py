"""File-only recovery tests for a complete yaw capture rejected by the old 10 s limit."""
import copy
import hashlib
import json
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import imu_commissioning_audit as audit
from singularitydog_hw import imu_yaw_recovery as recovery
from singularitydog_hw import imu_yaw_retake as retake
from singularitydog_hw.imu_fixed_mount_baseline import BaselineError
from test_imu_fixed_mount_baseline import synthetic_capture, refresh_summary


def write_capture(path, metadata, rows):
    path.mkdir()
    (path / "summary.json").write_text(json.dumps(metadata) + "\n")
    (path / "events.jsonl").write_text("\n".join(json.dumps(row) for row in
        [{"kind": "capture_metadata", **metadata["plan"]}, *rows]) + "\n")


class YawRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.static = self.root / "static"
        self.static.mkdir()
        for name, seed, start in (("static-a", 11, 100), ("static-b", 22, 140)):
            write_capture(self.static / name, *synthetic_capture(seed, start))
        self.static_manifest = self.static / "manifest.json"
        self.static_manifest.write_text(json.dumps({
            "schema_version": 1, "mount_candidate": audit.mount_candidate(),
            "stationary": {"a": "static-a", "b": "static-b", "operator_confirmed": True},
            "operator_assertions": {"qdd_power_off": True, "body_supported": True},
            "movements": [], "runtime_approval": False}) + "\n")
        stationary, mount, source = retake._stationary_source(self.static_manifest)
        self.session = self.root / "incomplete-r5"
        self.session.mkdir()
        metadata, original_rows = synthetic_capture(33, 180)
        rows = []
        first = original_rows[0]["monotonic_ns"]
        for index in range(3000):
            row = copy.deepcopy(original_rows[index % len(original_rows)])
            stamp = first + index * 10_000_000
            row.update(sequence=index+1, monotonic_ns=stamp,
                       wall_time_ns=1_790_000_000_000_000_000+stamp+500_000,
                       read_started_monotonic_ns=stamp-500_000,
                       read_finished_monotonic_ns=stamp+500_000)
            t = index * .01
            row["raw_gyro"][2] += -270 if 2 <= t < 14.4 else 880 if 20 <= t < 23.8 else 0
            row["gyro_rad_s"] = [v*metadata["configuration"]["gyro_rad_s_per_lsb"]
                                  for v in row["raw_gyro"]]
            rows.append(row)
        metadata["plan"]["capture_seconds"] = 30
        refresh_summary(metadata, rows)
        write_capture(self.session / "turn_left", metadata, rows)
        markers = {name: first + int(t*1e9) for (name, _), t in
                   zip(retake.MARKERS, (2, 14.4, 20, 23.8))}
        records = [{"kind": "first_sample", "monotonic_ns": first,
                    "capture_seconds_requested": 30}]
        records += [{"kind": "operator_marker", "name": name, "monotonic_ns": stamp,
                     "elapsed_from_first_sample_s": (stamp-first)/1e9}
                    for name, stamp in markers.items()]
        records += [{"kind": "last_sample", "monotonic_ns": rows[-1]["monotonic_ns"],
                     "elapsed_from_first_sample_s": (rows[-1]["monotonic_ns"]-first)/1e9}]
        self.marker_path = self.session / "yaw-markers.jsonl"
        self.marker_path.write_text("\n".join(json.dumps(record) for record in records) + "\n")
        self.receipt = self.session / "session-incomplete.json"
        self.receipt.write_text(json.dumps({
            "error": "ValueError('outbound window 12.40s: allowed 0.50..10.00s')",
            "status": "INCOMPLETE", "approved_for_runtime": False,
            "manifest_so_far": {"schema_version": 1, "mount_candidate": mount,
                "stationary": stationary, "stationary_source": source, "movements": [],
                "hardware_scope": "IMU only; no CAN",
                "operator_assertions": {"qdd_power_off": True, "body_supported": True,
                                        "imu_mount_unchanged_since_static": True},
                "runtime_approval": False}}) + "\n")

    def test_complete_capture_recovers_with_explicit_direction_but_never_approves_runtime(self):
        output = self.root / "recovered"
        with patch.object(retake.subprocess, "Popen") as process:
            result = recovery.recover(self.session, self.static_manifest, output,
                                      operator_direction_confirmed=True)
            process.assert_not_called()
        self.assertTrue(result["checks"]["turn_left_axis_sign"])
        self.assertEqual(result["movements"][0]["failed_gates"], [])
        self.assertEqual(result["axis_limits"]["maximum_window_s"], 15)
        self.assertFalse(result["approved_for_runtime"])
        self.assertFalse(result["hardware_opened"])
        self.assertFalse(result["motor_output_available"])
        manifest_path, audit_path = output / "manifest.json", output / "audit.json"
        manifest = json.loads(manifest_path.read_text())
        self.assertEqual(result["manifest_sha256"], hashlib.sha256(manifest_path.read_bytes()).hexdigest())
        self.assertEqual(manifest["recovery_evidence"]["marker_sidecar_sha256"],
                         hashlib.sha256(self.marker_path.read_bytes()).hexdigest())
        self.assertEqual(stat.S_IMODE(manifest_path.stat().st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(audit_path.stat().st_mode), 0o600)
        with self.assertRaisesRegex(BaselineError, "new output directory"):
            recovery.recover(self.session, self.static_manifest, output,
                             operator_direction_confirmed=True)

    def test_direction_unconfirmed_is_explicit_failed_gate(self):
        _, result, _ = recovery.evaluate(self.session, self.static_manifest,
                                         operator_direction_confirmed=False)
        self.assertFalse(result["checks"]["turn_left_axis_sign"])
        self.assertIn("operator_direction_confirmed", result["movements"][0]["failed_gates"])
        with self.assertRaisesRegex(BaselineError, "explicit operator"):
            recovery.evaluate(self.session, self.static_manifest,
                              operator_direction_confirmed=None)

    def test_rejects_marker_tamper_and_nonrestored_capture(self):
        original = self.marker_path.read_bytes()
        records = [json.loads(line) for line in original.splitlines()]
        records[1]["monotonic_ns"] += 1_000_000_000
        self.marker_path.write_text("\n".join(json.dumps(record) for record in records) + "\n")
        with self.assertRaisesRegex(BaselineError, "elapsed timestamp"):
            recovery.evaluate(self.session, self.static_manifest, operator_direction_confirmed=True)
        self.marker_path.write_bytes(original)
        summary = self.session / "turn_left" / "summary.json"
        metadata = json.loads(summary.read_text())
        metadata["restore_status"] = "failed"
        summary.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(BaselineError, "restoration"):
            recovery.evaluate(self.session, self.static_manifest, operator_direction_confirmed=True)

    def test_rejects_changed_static_evidence_and_git_output(self):
        summary = self.static / "static-a" / "summary.json"
        summary.write_bytes(summary.read_bytes() + b" ")
        with self.assertRaisesRegex(BaselineError, "static a summary_sha256"):
            recovery.evaluate(self.session, self.static_manifest, operator_direction_confirmed=True)
        # Restore the exact file before checking the output location.
        summary.write_bytes(summary.read_bytes()[:-1])
        git = self.root / "git"
        git.mkdir()
        (git / ".git").mkdir()
        with self.assertRaisesRegex(ValueError, "outside Git"):
            recovery.recover(self.session, self.static_manifest, git / "report",
                             operator_direction_confirmed=True)


if __name__ == "__main__":
    unittest.main()
