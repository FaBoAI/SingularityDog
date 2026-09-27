"""Saved two-hypothesis comparison tests; synthetic source files only."""

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from singularitydog_hw import dual_policy_once as once
from test_dual_policy_once import captured
from test_policy_observer import FakeTorch, Policy
from tools import compare_dual_policy_saved as comparison


class SensitivePolicy(Policy):
    def __call__(self, *tensors):
        result = super().__call__(*tensors)
        result.values[0][0] += 0.01 * tensors[-1].tolist()[0][0]
        return result


def fixture(root):
    dual_report, events, imu, calibration = captured()
    calibration.update({"calibration_verified": False, "output_allowed": False,
                        "motor_output_available": False})
    for row in calibration["candidates"]:
        row.update({"approved_for_runtime": False,
                    "physical_angle_accuracy_verified": False})
    from test_policy_observer import mount
    mount_candidate = mount()
    original = once.observe_one(dual_report, events, imu, calibration, mount_candidate,
                                SensitivePolicy(), FakeTorch, h_hypothesis=0,
                                boot_id="synthetic-boot", motor_power_epoch="UNVERIFIED")
    root = Path(root)
    content = {"dual-report.json": dual_report, "events-front.json": events["front"],
               "events-rear.json": events["rear"], "events-imu.json": imu,
               "calibration.json": calibration, "mount.json": mount_candidate}
    hashes = {}
    for name, data in content.items():
        raw = json.dumps(data, allow_nan=False).encode()
        (root/name).write_bytes(raw)
        hashes[name] = hashlib.sha256(raw).hexdigest()
    summary = {**once.FLAGS, "status": "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT",
               "boot_id": "synthetic-boot",
               "plan": {"motor_power_epoch_label": "UNVERIFIED"},
               "motor_power_epoch_attested": False,
               "failure": None,
               "observation": original,
               "capture_file_sha256": {name: hashes[name]
                                       for name in comparison.CAPTURE_FILES},
               "input_sha256": {"calibration": hashes["calibration.json"],
                                "imu_mount_candidate": hashes["mount.json"]},
               "model_source": {"sha256": {"synthetic": "fixed"}}}
    (root/"summary.json").write_text(json.dumps(summary, allow_nan=False))
    return summary


class SavedDualPolicyComparisonTests(unittest.TestCase):
    def test_exact_h0_replay_and_independent_h1_sensitivity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fixture(root)
            summary, loaded, calibration, mount, hashes = comparison.load_saved(
                root, root/"calibration.json", root/"mount.json")
            result = comparison.compare_saved(summary, loaded, calibration, mount,
                SensitivePolicy, FakeTorch, model_sha256={"synthetic": "fixed"})
        self.assertEqual(result["status"], "OFFLINE_DUAL_H_SENSITIVITY_NO_OUTPUT")
        self.assertTrue(result["h0_exact_reproduction"])
        self.assertAlmostEqual(result["max_h1_minus_h0_target_abs_rad"], 0.01)
        self.assertEqual(len(result["rows_model_can_order"]), 12)
        self.assertTrue(all(result[key] is False for key in comparison.FALSE_FLAGS))
        self.assertEqual(hashes["capture_file_sha256"], summary["capture_file_sha256"])

    def test_changed_capture_bytes_rejected_before_policy_loading(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fixture(root)
            front = root/"events-front.json"
            front.write_bytes(front.read_bytes() + b" ")
            with self.assertRaisesRegex(ValueError, "differs from original capture"):
                comparison.load_saved(root, root/"calibration.json", root/"mount.json")

    def test_tampered_saved_baseline_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fixture(root)
            summary, loaded, calibration, mount, _ = comparison.load_saved(
                root, root/"calibration.json", root/"mount.json")
            changed = copy.deepcopy(summary)
            changed["observation"]["observer_tick"]["q_target_rad_diagnostic_only"][0] += .001
            with self.assertRaisesRegex(ValueError, "did not reproduce"):
                comparison.compare_saved(changed, loaded, calibration, mount,
                    SensitivePolicy, FakeTorch, model_sha256={"synthetic": "fixed"})

    def test_cross_cpu_float32_actor_roundoff_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fixture(root)
            summary, loaded, calibration, mount, _ = comparison.load_saved(
                root, root/"calibration.json", root/"mount.json")
            rounded = copy.deepcopy(summary)
            rounded["observation"]["observer_tick"]["actor_residual12"][0] += 5e-7
            result = comparison.compare_saved(rounded, loaded, calibration, mount,
                SensitivePolicy, FakeTorch, model_sha256={"synthetic": "fixed"})
            self.assertFalse(result["h0_exact_reproduction"])
            self.assertTrue(result["h0_non_actor_fields_exact"])
            self.assertAlmostEqual(result["h0_actor_max_abs_difference"], 5e-7)
            rounded["observation"]["observer_tick"]["actor_residual12"][0] += 1e-5
            with self.assertRaisesRegex(ValueError, "did not reproduce"):
                comparison.compare_saved(rounded, loaded, calibration, mount,
                    SensitivePolicy, FakeTorch, model_sha256={"synthetic": "fixed"})

    def test_unapproved_flags_and_model_hash_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fixture(root)
            summary, loaded, calibration, mount, _ = comparison.load_saved(
                root, root/"calibration.json", root/"mount.json")
            with self.assertRaisesRegex(ValueError, "Policy bundle differs"):
                comparison.compare_saved(summary, loaded, calibration, mount,
                    SensitivePolicy, FakeTorch, model_sha256={"synthetic": "other"})
            data = json.loads((root/"summary.json").read_text())
            data["learned_target_sent"] = True
            (root/"summary.json").write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "no-output/unapproved"):
                comparison.load_saved(root, root/"calibration.json", root/"mount.json")


if __name__ == "__main__":
    unittest.main()
