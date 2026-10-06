import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from analyze_imu_acceleration_consistency import G, analyze, audit_rows, audit_capture, audit_range_crosscheck, hypotheses


def config(range_g=2):
    index = (2, 4, 8, 16).index(range_g)
    return {"frame": "sensor", "orientation_applied": False, "who_am_i": 234,
            "registers": {"bank2:0x14": 33 | (index << 1)}, "accel_range_g": range_g,
            "accel_m_s2_per_lsb": G / (16384, 8192, 4096, 2048)[index]}


def rows(counts=None, range_g=2):
    counts = counts or [[0, 0, -17800], [1, 0, -17801], [-1, 0, -17799]]
    scale = config(range_g)["accel_m_s2_per_lsb"]
    return [{"frame": "sensor", "monotonic_ns": (i+1)*10000000,
             "raw_accel": r, "accel_m_s2": [v*scale for v in r],
             "data_ready_during_read": i == 1, "temperature_c": 28.0}
            for i, r in enumerate(counts)]


def capture(path, sample_rows=None):
    sample_rows = sample_rows or rows()
    path.mkdir()
    summary = {"status": "RECORDED_NOT_CALIBRATED", "errors": [], "restore_status": "restored",
               "plan": {"calibration_applied": False}, "configuration": config(),
               "summary": {"samples": len(sample_rows)},
               "register_audit_before": {"offset_registers_written": False, "raw_registers": {"z": 10}},
               "register_audit_after": {"offset_registers_written": False, "raw_registers": {"z": 10}}}
    (path / "summary.json").write_text(json.dumps(summary))
    (path / "events.jsonl").write_text("\n".join(map(json.dumps, sample_rows)))
    return summary


class AccelerationConsistencyTests(unittest.TestCase):
    def test_constant_norm_excess_is_not_assigned_to_random_noise(self):
        r = audit_rows(rows([[0, 0, -17800]]*4), config())
        self.assertGreater(r["norm_excess_percent"], 8)
        self.assertEqual(r["observed_norm_averaging_gap_m_s2"], 0)
        self.assertEqual(r["raw_to_si_max_abs_error_m_s2"], 0)

    def test_transverse_variation_has_positive_jensen_gap(self):
        r = audit_rows(rows([[1000, 0, -16384], [-1000, 0, -16384]]), config())
        self.assertAlmostEqual(r["norm_of_mean_vector_m_s2"], G)
        self.assertGreater(r["observed_norm_averaging_gap_m_s2"], 0)

    def test_fullscale_readback_controls_conversion(self):
        for fs in (2, 4, 8, 16):
            with self.subTest(fs=fs):
                self.assertEqual(audit_rows(rows(range_g=fs), config(fs))["accel_range_g"], fs)
        bad = config(); bad["accel_range_g"] = 4
        with self.assertRaisesRegex(ValueError, "range/readback"):
            audit_rows(rows(), bad)

    def test_bad_scale_rejected(self):
        c = config(); c["accel_m_s2_per_lsb"] *= 0.92
        with self.assertRaisesRegex(ValueError, "SI scale"):
            audit_rows(rows(), c)

    def test_raw_si_mismatch_rejected_instead_of_silently_refit(self):
        r = rows(); r[0]["accel_m_s2"][2] *= 0.92
        with self.assertRaisesRegex(ValueError, "raw/SI"):
            audit_rows(r, config())

    def test_invalid_measurements_rejected(self):
        cases = [("raw_accel", [True, 0, 1]), ("raw_accel", [0, 0, 32768]),
                 ("accel_m_s2", [0, 0, float("nan")]), ("monotonic_ns", False),
                 ("data_ready_during_read", "false"), ("calibration_applied", True)]
        for key, value in cases:
            with self.subTest(key=key, value=value):
                r = rows(); r[0][key] = value
                with self.assertRaises(ValueError):
                    audit_rows(r, config())

    def test_repeated_timestamps_rejected(self):
        r = rows(); r[1]["monotonic_ns"] = r[0]["monotonic_ns"]
        with self.assertRaisesRegex(ValueError, "increase"):
            audit_rows(r, config())

    def test_ready_races_are_retained_not_filtered(self):
        result = audit_rows(rows(), config())
        self.assertEqual(result["samples"], 3)
        self.assertEqual(result["data_ready_race_groups"]["true"]["samples"], 1)
        self.assertEqual(result["data_ready_race_groups"]["false"]["samples"], 2)
        self.assertTrue(result["data_ready_race_is_not_proof_of_torn_sample"])

    def test_hypotheses_fit_current_pose_but_predict_different_inverted_norm(self):
        result = hypotheses([0, 0, -G*1.09])
        p = result["prediction_by_orientation"]
        self.assertAlmostEqual(p[0]["predicted_norm_separation_m_s2"], 0)
        self.assertAlmostEqual(p[-1]["predicted_norm_separation_m_s2"], G*0.18)
        self.assertFalse(result["real_bias_or_scale_identified"])
        self.assertAlmostEqual(p[-2]["radial_bias_normalized_direction_error_deg"], math.degrees(math.atan(0.09)))
        self.assertTrue(result["coefficients_must_not_be_used_by_runtime"])

    def test_no_bias_identification_from_many_same_pose_captures(self):
        with tempfile.TemporaryDirectory() as t:
            a, b = Path(t)/"a", Path(t)/"b"
            capture(a)
            second = rows(); second[0]["monotonic_ns"] += 1
            capture(b, second)
            result = analyze([a, b])
            self.assertLess(result["maximum_mean_direction_separation_deg"], 1e-5)
            for key in ("approved_for_runtime", "output_allowed", "hardware_opened", "profile_changed", "calibration_created"):
                self.assertFalse(result[key])

    def test_duplicate_measurements_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            p = Path(t)/"a"; capture(p)
            with self.assertRaisesRegex(ValueError, "duplicate"):
                analyze([p, p])

    def test_bad_capture_state_and_changed_trim_rejected(self):
        mutations = [lambda s: s.update(status="INCOMPLETE"),
                     lambda s: s.update(restore_status="failed"),
                     lambda s: s["summary"].update(samples=4),
                     lambda s: s["register_audit_after"]["raw_registers"].update(z=11)]
        for change in mutations:
            with tempfile.TemporaryDirectory() as t:
                p = Path(t)/"capture"; s = capture(p); change(s)
                (p/"summary.json").write_text(json.dumps(s))
                with self.assertRaises(ValueError):
                    audit_capture(p)

    def test_missing_trim_retained_as_unknown(self):
        with tempfile.TemporaryDirectory() as t:
            p = Path(t)/"capture"; s = capture(p)
            del s["register_audit_before"]; del s["register_audit_after"]
            (p/"summary.json").write_text(json.dumps(s))
            r = analyze([p])
            self.assertIsNone(r["trim_consistent_across_available_audits"])
            self.assertEqual(r["captures_without_trim_audit"], 1)

    def test_range_comparison_checks_each_raw_session_not_stored_norm(self):
        with tempfile.TemporaryDirectory() as t:
            p = Path(t); all_rows, sessions = [], []
            for i, fs in enumerate((2, 4, 2)):
                counts = -17800 if fs == 2 else -8900
                sample_rows = rows([[0, 0, counts]]*3, fs)
                for r in sample_rows:
                    r["range_g"] = fs
                    r["monotonic_ns"] += i*100000000
                all_rows.extend(sample_rows)
                sessions.append({"requested_range_g": fs, "configuration": config(fs),
                                 "restore_status": "restored", "summary": {
                                     "sample_count": 3,
                                     "first_monotonic_ns": sample_rows[0]["monotonic_ns"],
                                     "last_monotonic_ns": sample_rows[-1]["monotonic_ns"],
                                     "accel_norm_mean_m_s2": -999}})
            report = {"status": "CONFIG_CONSISTENT_ACCEL_NORM_UNRESOLVED", "errors": [], "sessions": sessions}
            (p/"report.json").write_text(json.dumps(report))
            (p/"samples.jsonl").write_text("\n".join(map(json.dumps, all_rows)))
            result = audit_range_crosscheck(p)
            self.assertEqual(result["four_vs_bracket_two_relative_difference_percent"], 0)
            self.assertTrue(result["common_gain_or_bias_not_excluded"])
            sessions[0]["summary"]["last_monotonic_ns"] = sessions[1]["summary"]["last_monotonic_ns"]
            (p/"report.json").write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "range mismatch"):
                audit_range_crosscheck(p)

    def test_range_comparison_rejects_unknown_extra_sample(self):
        with tempfile.TemporaryDirectory() as t:
            p = Path(t); r = rows()
            for x in r: x["range_g"] = 2
            session = {"requested_range_g": 2, "configuration": config(), "restore_status": "restored",
                       "summary": {"sample_count": 2, "first_monotonic_ns": r[0]["monotonic_ns"],
                                   "last_monotonic_ns": r[1]["monotonic_ns"]}}
            (p/"report.json").write_text(json.dumps({"status": "CONFIG_CONSISTENT_ACCEL_NORM_UNRESOLVED",
                                                     "errors": [], "sessions": [session]}))
            (p/"samples.jsonl").write_text("\n".join(map(json.dumps, r)))
            with self.assertRaisesRegex(ValueError, "unassigned"):
                audit_range_crosscheck(p)


if __name__ == "__main__":
    unittest.main()
