"""Synthetic capture integrity and non-approval tests; no hardware access."""
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import random
import stat
import statistics
import tempfile
import unittest

from singularitydog_hw.imu_fixed_mount_baseline import (
    BaselineError, GRAVITY, compare_captures, main, write_baseline,
)


def synthetic_capture(seed, start_s):
    rng = random.Random(seed)
    scale_a, scale_g = GRAVITY/16384, math.pi/180/131
    plan = {"face_label": "unverified", "face_label_source": "operator_argument",
            "orientation_verified_by_software": False, "settle_seconds": 10,
            "capture_seconds": 12, "bus": "/dev/i2c-7", "address": 104,
            "can_opened": False, "calibration_applied": False, "poll_hz": 100}
    config = {"who_am_i": 234, "address": 104, "frame": "sensor", "axis_order": ["x", "y", "z"],
              "orientation_applied": False, "magnetometer_initialized": False,
              "accel_range_g": 2, "gyro_range_dps": 250,
              "accel_m_s2_per_lsb": scale_a, "gyro_rad_s_per_lsb": scale_g,
              "registers": {"bank2:0x14": 33, "bank2:0x01": 33, "bank0:0x06": 1}}
    audit = {"raw_registers": {f"bank{bank}:0x{address:02X}": 0 for bank, addresses in {
        1: (2, 3, 4, 14, 15, 16, 20, 21, 23, 24, 26, 27, 40),
        2: (3, 4, 5, 6, 7, 8)}.items() for address in addresses},
        "offset_registers_written": False, "self_test_executed": False}
    rows = []
    for i in range(1200):
        raw_a = [rng.randint(-15, 15), rng.randint(-15, 15), -17898+rng.randint(-20, 20)]
        raw_g = [36+rng.randint(-5, 5), 128+rng.randint(-5, 5), -31+rng.randint(-5, 5)]
        mono = int(start_s*1e9)+i*10_000_000
        raw_temp = 2400+rng.randint(-20, 20)
        rows.append({"kind": "imu", "frame": "sensor", "sequence": i+1,
                     "monotonic_ns": mono, "wall_time_ns": 1_790_000_000_000_000_000+mono+500_000,
                     "read_started_monotonic_ns": mono-500_000,
                     "read_finished_monotonic_ns": mono+500_000, "data_ready_status": 1,
                     "raw_accel": raw_a, "raw_gyro": raw_g,
                     "accel_m_s2": [v*scale_a for v in raw_a], "gyro_rad_s": [v*scale_g for v in raw_g],
                     "raw_temperature": raw_temp, "temperature_c": raw_temp/333.87+21})
    metadata = {"status": "RECORDED_NOT_CALIBRATED", "restore_status": "restored", "errors": [],
                "plan": plan, "configuration": config, "source_sha256": {"imu.py": "a"*64, "imu_capture.py": "b"*64},
                "register_audit_before": copy.deepcopy(audit), "register_audit_after": copy.deepcopy(audit)}
    refresh_summary(metadata, rows)
    return metadata, rows


def refresh_summary(metadata, rows):
    duration = (rows[-1]["monotonic_ns"]-rows[0]["monotonic_ns"])/1e9
    temps = [r["temperature_c"] for r in rows]
    norm = statistics.fmean(math.sqrt(sum(v*v for v in r["accel_m_s2"])) for r in rows)
    metadata["summary"] = {"samples": len(rows), "duration_s": duration,
        "rate_hz": (len(rows)-1)/duration, "accel_norm_mean_m_s2": norm,
        "gravity_norm_deviation_percent": 100*(norm/GRAVITY-1),
        "temperature_c": {"mean": statistics.fmean(temps), "min": min(temps), "max": max(temps),
                          "first": temps[0], "last": temps[-1]}, "sensor_frame": True, "calibration_applied": False}
    for key, source in (("accel", "accel_m_s2"), ("gyro", "gyro_rad_s")):
        unit = "m_s2" if key == "accel" else "rad_s"
        metadata["summary"][key+"_mean_"+unit] = [statistics.fmean(r[source][i] for r in rows) for i in range(3)]
        metadata["summary"][key+"_std_"+unit] = [statistics.pstdev(r[source][i] for r in rows) for i in range(3)]


class FixedMountTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.a, self.b = self.root/"a", self.root/"b"
        self.a.mkdir(); self.b.mkdir()
        self.ma, self.ra = synthetic_capture(11, 100)
        self.mb, self.rb = synthetic_capture(22, 140)
        self.save()

    def save(self):
        for path, meta, rows in ((self.a, self.ma, self.ra), (self.b, self.mb, self.rb)):
            (path/"summary.json").write_text(json.dumps(meta)+"\n")
            records = [{"kind": "capture_metadata", **meta["plan"]}]+rows
            (path/"events.jsonl").write_text("\n".join(json.dumps(r) for r in records)+"\n")

    def compare(self, confirmed=True):
        return compare_captures(self.a, self.b, operator_confirmed_stationary=confirmed)

    def assertRejected(self, regex=None):
        self.save()
        with self.assertRaisesRegex(BaselineError, regex or "."):
            self.compare()

    def test_gyro_candidate_uses_a_only_and_never_claims_acceleration_calibration(self):
        result = self.compare()
        self.assertEqual(result["gyro_bias_candidate_rad_s"], self.ma["summary"]["gyro_mean_rad_s"])
        self.assertTrue(result["gyro_bias_candidate_eligible"])
        self.assertGreater(result["captures"]["a"]["gravity_norm_deviation_percent"], 9.)
        self.assertEqual(len(result["warnings"]), 2)
        for flag in ("approved_for_runtime", "automatically_applied", "hardware_opened", "motor_output_available",
                     "calibration_verified", "accel_bias_estimated", "accel_scale_estimated", "mount_rotation_applied"):
            self.assertIs(result[flag], False)
        self.assertLess(result["captures"]["a"]["specific_force_direction_sensor_unit"][2], -.99)
        for row in self.rb:
            row["raw_gyro"][0] += 10
            row["gyro_rad_s"][0] = row["raw_gyro"][0]*self.mb["configuration"]["gyro_rad_s_per_lsb"]
        refresh_summary(self.mb, self.rb); self.save()
        changed = self.compare()
        self.assertEqual(changed["gyro_bias_candidate_rad_s"], result["gyro_bias_candidate_rad_s"])
        self.assertGreater(changed["comparison"]["gyro_b_residual_after_subtracting_a_mean_rad_s"][0], .001)

    def test_unconfirmed_stationarity_yields_diagnostics_not_candidate(self):
        result = self.compare(False)
        self.assertEqual(result["status"], "DIAGNOSTIC_ONLY")
        self.assertIsNone(result["gyro_bias_candidate_rad_s"])
        self.assertFalse(result["gyro_bias_candidate_eligible"])
        self.assertTrue(all(result["diagnostic_gates"].values()))

    def test_large_repeatability_change_cannot_be_approved_by_operator(self):
        for row in self.rb:
            row["raw_gyro"][0] += 300
            row["gyro_rad_s"][0] = row["raw_gyro"][0]*self.mb["configuration"]["gyro_rad_s_per_lsb"]
        refresh_summary(self.mb, self.rb); self.save()
        result = self.compare()
        self.assertFalse(result["gyro_bias_candidate_eligible"])
        self.assertIn("gyro_repeatability", result["failed_diagnostic_gates"])

    def test_direction_change_is_reported_without_mount_rotation(self):
        for row in self.rb:
            row["raw_accel"][0] += 1700
            row["accel_m_s2"][0] = row["raw_accel"][0]*self.mb["configuration"]["accel_m_s2_per_lsb"]
        refresh_summary(self.mb, self.rb); self.save()
        result = self.compare()
        self.assertGreater(result["comparison"]["specific_force_direction_change_deg"], 5.)
        self.assertIn("direction_repeatability", result["failed_diagnostic_gates"])
        self.assertFalse(result["gyro_bias_candidate_eligible"])

    def test_duplicate_capture_and_retimed_measurements_rejected(self):
        with self.assertRaisesRegex(BaselineError, "reused"):
            compare_captures(self.a, self.a)
        for source, target in zip(self.ra, self.rb):
            for key in ("raw_accel", "raw_gyro", "accel_m_s2", "gyro_rad_s"):
                target[key] = copy.deepcopy(source[key])
        refresh_summary(self.mb, self.rb)
        self.assertRejected("reused")

    def test_overlapping_capture_intervals_rejected(self):
        for r in self.rb:
            for key in ("monotonic_ns", "wall_time_ns", "read_started_monotonic_ns", "read_finished_monotonic_ns"):
                r[key] -= 40_000_000_000
        self.assertRejected("overlapping")

    def test_duplicate_timestamp_missing_sequence_and_gap_rejected(self):
        original = copy.deepcopy(self.rb)
        for mutation in ("duplicate", "sequence", "gap"):
            self.rb = copy.deepcopy(original)
            if mutation == "duplicate":
                self.rb[1]["wall_time_ns"] = self.rb[0]["wall_time_ns"]
            elif mutation == "sequence":
                self.rb[1]["sequence"] += 1
            else:
                for row in self.rb[1:]:
                    for key in ("monotonic_ns", "wall_time_ns", "read_started_monotonic_ns", "read_finished_monotonic_ns"):
                        row[key] += 500_000_000
            with self.subTest(mutation=mutation):
                self.assertRejected()

    def test_short_capture_rejected_even_if_summary_is_consistent(self):
        self.rb = self.rb[:300]
        refresh_summary(self.mb, self.rb)
        self.assertRejected("duration")

    def test_incomplete_restore_trim_and_source_mismatch_rejected(self):
        original = copy.deepcopy(self.mb)
        for mutation in ("incomplete", "restore", "trim", "source", "configuration"):
            self.mb = copy.deepcopy(original)
            if mutation == "incomplete": self.mb["status"] = "INCOMPLETE"
            elif mutation == "restore": self.mb["restore_status"] = "failed"
            elif mutation == "trim": self.mb["register_audit_after"]["raw_registers"]["bank1:0x14"] = 4
            elif mutation == "source": self.mb["source_sha256"]["imu.py"] = "c"*64
            else: self.mb["configuration"]["magnetometer_initialized"] = True
            with self.subTest(mutation=mutation): self.assertRejected()

    def test_different_trim_between_captures_rejected(self):
        for key in ("register_audit_before", "register_audit_after"):
            self.mb[key]["raw_registers"]["bank1:0x14"] = 4
        self.assertRejected("trim differs")

    def test_corrupt_scale_raw_conversion_and_previous_correction_rejected(self):
        original_row, original_meta = copy.deepcopy(self.rb[0]), copy.deepcopy(self.mb)
        for mutation in ("scale", "raw", "temperature", "frame", "already_corrected", "summary", "nan"):
            self.rb[0], self.mb = copy.deepcopy(original_row), copy.deepcopy(original_meta)
            if mutation == "scale": self.mb["configuration"]["accel_m_s2_per_lsb"] *= .9154
            elif mutation == "raw": self.rb[0]["accel_m_s2"][2] *= .9154
            elif mutation == "temperature": self.rb[0]["temperature_c"] += 1
            elif mutation == "frame": self.rb[0]["frame"] = "body"
            elif mutation == "already_corrected": self.rb[0]["gyro_bias_subtracted"] = True
            elif mutation == "summary": self.mb["summary"]["gyro_mean_rad_s"][0] += 1
            else: self.rb[0]["accel_m_s2"][0] = float("nan")
            with self.subTest(mutation=mutation): self.assertRejected()

    def test_temperature_drift_is_not_silently_accepted(self):
        for index, row in enumerate(self.rb):
            row["raw_temperature"] += index
            row["temperature_c"] = row["raw_temperature"]/333.87+21
        refresh_summary(self.mb, self.rb); self.save()
        result = self.compare()
        self.assertFalse(result["gyro_bias_candidate_eligible"])
        self.assertIn("b_temperature_drift", result["failed_diagnostic_gates"])

    def test_private_new_output_preserves_inputs_and_refuses_git(self):
        before = {p: p.read_bytes() for d in (self.a, self.b) for p in d.iterdir()}
        output = self.root/"report.json"
        result = write_baseline(self.a, self.b, output, operator_confirmed_stationary=True)
        self.assertTrue(result["gyro_bias_candidate_eligible"])
        self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
        self.assertEqual(before, {p: p.read_bytes() for p in before})
        with self.assertRaises(FileExistsError): write_baseline(self.a, self.b, output)
        git = self.root/"repository"; git.mkdir(); (git/".git").write_text("gitdir: elsewhere\n")
        with self.assertRaisesRegex(BaselineError, "outside Git"):
            write_baseline(self.a, self.b, git/"report.json")
        alias = self.root/"alias"; alias.symlink_to(git, target_is_directory=True)
        with self.assertRaisesRegex(BaselineError, "outside Git"):
            write_baseline(self.a, self.b, alias/"report.json")

    def test_cli_without_assertion_is_diagnostic_and_invalid_capture_creates_no_output(self):
        output = self.root/"cli.json"
        argv = ["--base-a", str(self.a), "--base-b", str(self.b), "--output", str(output)]
        with contextlib.redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(main(argv), 0)
        self.assertFalse(json.loads(stream.getvalue())["gyro_bias_candidate_eligible"])
        self.assertFalse(json.loads(output.read_text())["approved_for_runtime"])
        output.unlink()
        self.mb["errors"] = ["interrupted"]; self.save()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            main(argv+["--operator-confirmed-stationary"])
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
