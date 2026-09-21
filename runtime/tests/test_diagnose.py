import unittest

from singularitydog_hw.can_readonly import PARAMETERS
from singularitydog_hw.diagnose import coverage_errors, optional_read_rejection


class DiagnosticCoverageTests(unittest.TestCase):
    def baseline(self):
        return {"imu": {"samples": 100}, "motors": {
            str(i): {"identities": [f"uid{i}"], "parameters": {
                name: {"last": 0} for name in PARAMETERS}}
            for i in range(1, 13)}}

    def test_zero_is_a_valid_reading(self):
        self.assertEqual(coverage_errors(self.baseline()), [])

    def test_early_end_cannot_report_complete(self):
        result = self.baseline()
        result["imu"]["samples"] = 1
        result["motors"]["12"]["parameters"]["voltage"]["last"] = None
        self.assertEqual(len(coverage_errors(result)), 2)

    def test_inconsistent_or_duplicate_identities(self):
        result = self.baseline()
        result["motors"]["12"]["identities"] = ["uid1"]
        self.assertTrue(coverage_errors(result))

    def test_only_explicit_optional_negative_reply_can_relax_coverage(self):
        result = self.baseline()
        result["motors"]["6"]["parameters"]["can_timeout"]["last"] = None
        self.assertTrue(coverage_errors(result))
        self.assertEqual(coverage_errors(result, {(6, "can_timeout")}), [])
        self.assertTrue(optional_read_rejection({"parameter":"can_timeout", "error":"parameter_status_nonzero"}))
        self.assertFalse(optional_read_rejection({"parameter":"voltage", "error":"parameter_status_nonzero"}))
        self.assertFalse(optional_read_rejection({"parameter":"can_timeout", "error":"reserved_bytes_nonzero"}))
        result = self.baseline()
        result["motors"]["12"]["identities"] = ["uid12", "different"]
        self.assertTrue(coverage_errors(result))


if __name__ == "__main__":
    unittest.main()
