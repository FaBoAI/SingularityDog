"""Offline tests with known biases/scales and adverse six-face datasets."""
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import random
import tempfile
import unittest

from singularitydog_hw.imu_calibration import (
    CalibrationError, CalibrationLimits, FACES, GRAVITY, calibrate_jsonl_files,
    estimate_six_face, main,
)


TRUE_BIAS = [0.02 * GRAVITY, -0.015 * GRAVITY, -0.09 * GRAVITY]
TRUE_SCALE = [1.02, 0.97, 0.995]
TRUE_GYRO_BIAS = [0.008, 0.018, -0.004]


def fake_faces(*, count=400, bias=TRUE_BIAS, scale=TRUE_SCALE, gyro_bias=TRUE_GYRO_BIAS):
    random_source = random.Random(12345)
    result = {}
    for face_index, label in enumerate(FACES):
        axis = "xyz".index(label[0])
        sign = 1 if label[1] == "+" else -1
        expected = [0.0, 0.0, 0.0]
        expected[axis] = sign * GRAVITY
        records = []
        for index in range(count):
            accel = [expected[i] / scale[i] + bias[i] + random_source.gauss(0, GRAVITY * 0.002)
                     for i in range(3)]
            gyro = [v + random_source.gauss(0, 0.0005) for v in gyro_bias]
            timestamp = 1_000_000_000 + face_index * 10_000_000_000 + index * 10_000_000
            records.append({
                "kind": "imu", "frame": "sensor", "monotonic_ns": timestamp,
                "wall_time_ns": 1_780_000_000_000_000_000 + timestamp,
                "raw_accel": [round(v * 16384 / GRAVITY) for v in accel],
                "raw_gyro": [round(v * 131 * 180 / math.pi) for v in gyro],
                "accel_m_s2": accel, "gyro_rad_s": gyro,
            })
        result[label] = records
    return result


class EstimatorTests(unittest.TestCase):
    def test_recovers_known_bias_scale_and_validates_heldout(self):
        datasets = fake_faces()
        original = copy.deepcopy(datasets)
        candidate = estimate_six_face(datasets)
        for estimated, true in zip(candidate["accel"]["bias_m_s2"], TRUE_BIAS):
            self.assertAlmostEqual(estimated, true, delta=0.003)
        for estimated, true in zip(candidate["accel"]["scale"], TRUE_SCALE):
            self.assertAlmostEqual(estimated, true, delta=0.001)
        for estimated, true in zip(candidate["gyro"]["bias_rad_s"], TRUE_GYRO_BIAS):
            self.assertAlmostEqual(estimated, true, delta=0.0001)
        for label in FACES:
            heldout = candidate["validation"]["heldout"][label]
            self.assertEqual(heldout["samples"], 100)
            self.assertLess(heldout["norm_rms_error_m_s2"], 0.01 * GRAVITY)
            self.assertLess(heldout["vector_rms_error_m_s2"], 0.01 * GRAVITY)
        self.assertEqual(candidate["status"], "candidate")
        self.assertTrue(candidate["requires_physical_validation"])
        self.assertFalse(candidate["approved_for_runtime"])
        self.assertFalse(candidate["automatically_applied"])
        self.assertEqual(datasets, original)

    def test_nine_percent_scale_error_is_allowed_and_recovered(self):
        candidate = estimate_six_face(fake_faces(bias=[0, 0, 0], scale=[1, 1, 1 / 1.09]))
        self.assertAlmostEqual(candidate["accel"]["scale"][2], 1 / 1.09, delta=0.001)

    def test_heldout_does_not_participate_in_fit(self):
        datasets = fake_faces()
        initial = estimate_six_face(datasets)
        for record in datasets["z+"][300:]:
            record["accel_m_s2"][2] += 0.02 * GRAVITY
        result = estimate_six_face(datasets)
        self.assertEqual(result["accel"], initial["accel"])
        self.assertGreater(result["validation"]["heldout"]["z+"]["norm_rms_error_m_s2"],
                           0.015 * GRAVITY)

    def test_heldout_drift_fails_gravity_validation(self):
        datasets = fake_faces()
        for record in datasets["z+"][300:]:
            record["accel_m_s2"][2] += 0.05 * GRAVITY
        with self.assertRaisesRegex(CalibrationError, "held-out.*gravity norm"):
            estimate_six_face(datasets)

    def test_six_distinct_labels_required(self):
        datasets = fake_faces()
        del datasets["z-"]
        with self.assertRaisesRegex(CalibrationError, "six distinct"):
            estimate_six_face(datasets)

    def test_count_and_duration_gates(self):
        with self.assertRaisesRegex(CalibrationError, "insufficient samples"):
            estimate_six_face(fake_faces(count=50))
        datasets = fake_faces()
        for index, record in enumerate(datasets["x+"]):
            record["monotonic_ns"] = 1_000_000 + index * 1_000_000
        with self.assertRaisesRegex(CalibrationError, "duration"):
            estimate_six_face(datasets)

    def test_nonfinite_nonnumeric_and_wrong_frame_are_rejected(self):
        for bad in (float("nan"), float("inf"), -float("inf"), True, "1"):
            datasets = fake_faces()
            datasets["x+"][0]["accel_m_s2"][0] = bad
            with self.subTest(bad=bad), self.assertRaises(CalibrationError):
                estimate_six_face(datasets)
        for field, value in (("frame", "body"), ("calibration_applied", True),
                             ("orientation_applied", 0)):
            datasets = fake_faces()
            datasets["x+"][0][field] = value
            with self.subTest(field=field), self.assertRaises(CalibrationError):
                estimate_six_face(datasets)

    def test_corrupt_raw_counts_are_rejected(self):
        datasets = fake_faces()
        datasets["x+"][0]["raw_accel"][0] = 32768
        with self.assertRaisesRegex(CalibrationError, "signed16"):
            estimate_six_face(datasets)

    def test_timing_must_increase_without_large_gaps(self):
        for change in (0, -1, 1_000_000_000):
            datasets = fake_faces()
            datasets["x+"][1]["monotonic_ns"] = datasets["x+"][0]["monotonic_ns"] + change
            with self.subTest(change=change), self.assertRaisesRegex(CalibrationError, "timing"):
                estimate_six_face(datasets)

    def test_overlapping_acquisition_intervals_are_rejected(self):
        datasets = fake_faces()
        for original, repeated in zip(datasets["x+"], datasets["x-"]):
            repeated["wall_time_ns"] = original["wall_time_ns"]
        with self.assertRaisesRegex(CalibrationError, "overlapping/reused"):
            estimate_six_face(datasets)

    def test_no_wall_time_requires_one_nonoverlapping_monotonic_timebase(self):
        datasets = fake_faces()
        for records in datasets.values():
            for record in records:
                del record["wall_time_ns"]
        self.assertEqual(estimate_six_face(datasets)["acquisition_interval_clock"],
                         "monotonic_ns_single_timebase")
        for original, repeated in zip(datasets["x+"], datasets["x-"]):
            repeated["monotonic_ns"] = original["monotonic_ns"]
        with self.assertRaisesRegex(CalibrationError, "overlapping/reused"):
            estimate_six_face(datasets)

    def test_motion_and_gyro_variation_are_rejected(self):
        for field, magnitude in (("accel_m_s2", 0.2 * GRAVITY), ("gyro_rad_s", 0.1)):
            datasets = fake_faces()
            for index, record in enumerate(datasets["x+"]):
                record[field][1] += magnitude * (1 if index % 2 else -1)
            with self.subTest(field=field), self.assertRaisesRegex(CalibrationError, "variation"):
                estimate_six_face(datasets)

    def test_holdout_motion_cannot_be_hidden_by_quiet_training_samples(self):
        datasets = fake_faces()
        for index, record in enumerate(datasets["x+"][300:]):
            record["gyro_rad_s"][1] += 0.04 * (1 if index % 2 else -1)
        with self.assertRaisesRegex(CalibrationError, "held-out gyro variation"):
            estimate_six_face(datasets)

    def test_constant_gyro_cannot_prove_stationarity(self):
        candidate = estimate_six_face(fake_faces(gyro_bias=[0.08, 0, 0]))
        self.assertAlmostEqual(candidate["gyro"]["bias_rad_s"][0], 0.08, delta=0.0001)
        self.assertTrue(any("constant rotation" in item for item in candidate["limitations"]))
        self.assertTrue(candidate["requires_physical_validation"])
        with self.assertRaisesRegex(CalibrationError, "gyro mean"):
            estimate_six_face(fake_faces(gyro_bias=[0.3, 0, 0]))

    def test_face_labels_and_axis_dominance_are_enforced(self):
        datasets = fake_faces()
        datasets["x+"], datasets["y+"] = datasets["y+"], datasets["x+"]
        with self.assertRaisesRegex(CalibrationError, "axis does not dominate"):
            estimate_six_face(datasets)

    def test_tilt_cannot_hide_behind_corrected_norm_only(self):
        datasets = fake_faces(bias=[0, 0, 0], scale=[1, 1, 1])
        angle = math.radians(12)
        for label in ("x+", "x-"):
            for record in datasets[label]:
                x = record["accel_m_s2"][0]
                record["accel_m_s2"][0] = x * math.cos(angle)
                record["accel_m_s2"][2] += x * math.sin(angle)
        with self.assertRaisesRegex(CalibrationError, "orientation"):
            estimate_six_face(datasets)

    def test_poor_span_and_unreasonable_bias_are_rejected(self):
        with self.assertRaisesRegex(CalibrationError, "span"):
            estimate_six_face(fake_faces(scale=[1.4, 1, 1]))
        with self.assertRaisesRegex(CalibrationError, "bias correction"):
            estimate_six_face(fake_faces(bias=[0.25 * GRAVITY, 0, 0]))

    def test_gyro_bias_must_be_consistent_across_poses(self):
        datasets = fake_faces()
        for index, label in enumerate(FACES):
            for record in datasets[label]:
                record["gyro_rad_s"][0] += 0.08 if index % 2 else -0.08
        with self.assertRaisesRegex(CalibrationError, "gyro bias is inconsistent"):
            estimate_six_face(datasets)


class FileAndCLITests(unittest.TestCase):
    def write_inputs(self, directory):
        paths = {}
        for label, records in fake_faces().items():
            path = Path(directory) / (label + ".jsonl")
            events = [{"kind": "can_status", "message": "ignored"}] + records
            path.write_text("\n".join(json.dumps(event) for event in events) + "\n")
            paths[label] = path
        return paths

    def test_file_provenance_and_output_exclusive_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self.write_inputs(directory)
            before = {label: path.read_bytes() for label, path in paths.items()}
            output = Path(directory) / "candidate.json"
            result = calibrate_jsonl_files(paths, output)
            self.assertTrue(output.exists())
            self.assertEqual(json.loads(output.read_text())["status"], "candidate")
            for label in FACES:
                self.assertEqual(result["provenance"][label]["sha256"], hashlib.sha256(before[label]).hexdigest())
                self.assertEqual(paths[label].read_bytes(), before[label])
            original_output = output.read_bytes()
            with self.assertRaises(FileExistsError):
                calibrate_jsonl_files(paths, output)
            self.assertEqual(output.read_bytes(), original_output)

    def test_duplicate_file_content_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self.write_inputs(directory)
            paths["x-"].write_bytes(paths["x+"].read_bytes())
            output = Path(directory) / "candidate.json"
            with self.assertRaisesRegex(CalibrationError, "reused input content hash"):
                calibrate_jsonl_files(paths, output)
            self.assertFalse(output.exists())

    def test_invalid_json_and_nonobject_json_are_rejected_without_output(self):
        for invalid in ("{broken", "[]", "null"):
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory() as directory:
                paths = self.write_inputs(directory)
                paths["x+"].write_text(invalid)
                output = Path(directory) / "candidate.json"
                with self.assertRaises(CalibrationError):
                    calibrate_jsonl_files(paths, output)
                self.assertFalse(output.exists())

    def test_rotated_configuration_event_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self.write_inputs(directory)
            path = paths["x+"]
            event = {"kind": "imu_configured", "configuration": {"frame": "body", "orientation_applied": True}}
            path.write_text(json.dumps(event) + "\n" + path.read_text())
            with self.assertRaisesRegex(CalibrationError, "configuration event"):
                calibrate_jsonl_files(paths, Path(directory) / "candidate.json")

    def test_cli_accepts_six_labels_and_never_marks_candidate_approved(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self.write_inputs(directory)
            output = Path(directory) / "candidate.json"
            arguments = [arg for label, path in paths.items() for arg in ("--face", label + "=" + str(path))]
            arguments += ["--output", str(output)]
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                self.assertEqual(main(arguments), 0)
            self.assertFalse(json.loads(stream.getvalue())["automatically_applied"])
            self.assertFalse(json.loads(output.read_text())["approved_for_runtime"])

    def test_cli_rejects_repeated_label(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            main(["--face", "x+=a", "--face", "x+=b", "--output", "unused"])
        self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
