"""The fixed-pose range comparison diagnoses conversion, not calibration."""
import unittest

from singularitydog_hw import imu_range_crosscheck as probe


def session(range_g, norm, raw, *, std=0.02, restore="restored"):
    return {"requested_range_g": range_g,
            "configuration": {"accel_range_g": range_g},
            "restore_status": restore,
            "original_registers": {"bank2:0x14": 7, "REG_BANK_SEL": 0x30},
            "summary": {"sample_count": 400, "accel_norm_mean_m_s2": norm,
                        "raw_count_norm_mean": raw,
                        "accel_axis_std_m_s2": [std, std, std],
                        "gyro_axis_mean_rad_s": [0.001, 0.002, 0.001]}}


class RangeCrosscheckTests(unittest.TestCase):
    def test_consistent_ranges_do_not_falsely_approve_large_norm_bias(self):
        result = probe.evaluate([session(2, 10.68, 17845),
                                 session(4, 10.67, 8920),
                                 session(2, 10.68, 17845)])
        self.assertEqual(result["status"], "CONFIG_CONSISTENT_ACCEL_NORM_UNRESOLVED")
        self.assertTrue(result["checks"]["raw_count_ratio_two_to_four_near_two"])
        self.assertFalse(result["checks"]["raw_si_norm_within_3_percent_of_g"])
        self.assertFalse(result["approved_for_runtime"])

    def test_range_dependent_result_requires_review(self):
        result = probe.evaluate([session(2, 10.68, 17845),
                                 session(4, 21.34, 17845),
                                 session(2, 10.68, 17845)])
        self.assertEqual(result["status"], "REVIEW_REQUIRED")
        self.assertFalse(result["checks"]["si_norm_range_invariant"])

    def test_motion_or_failed_restore_requires_review(self):
        result = probe.evaluate([session(2, 9.8, 16384),
                                 session(4, 9.8, 8192, std=0.3),
                                 session(2, 9.8, 16384, restore="failed")])
        self.assertEqual(result["status"], "REVIEW_REQUIRED")
        self.assertFalse(result["checks"]["stationary"])
        self.assertFalse(result["checks"]["all_sessions_restored"])


if __name__ == "__main__":
    unittest.main()
