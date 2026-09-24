"""Offline integration conventions, held-out replay and validation boundaries."""
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import imu_orientation_replay as replay
from singularitydog_hw.imu_fixed_mount_baseline import BaselineError
from test_imu_fixed_mount_baseline import refresh_summary, synthetic_capture


class GyroIntegrationTests(unittest.TestCase):
    def test_constant_positive_and_negative_45_and_90_degree_axes(self):
        # Irregular actual times also exercise use of dt instead of nominal Hz.
        times = [1_000_000_000+int(v*1e9) for v in (0, .07, .25, .41, .6, .75, 1)]
        for axis in range(3):
            for angle in (-90, -45, 45, 90):
                with self.subTest(axis=axis, angle=angle):
                    rate = [0., 0., 0.]
                    rate[axis] = math.radians(angle)
                    result = replay.integrate_gyro(times, [rate]*len(times))
                    q = result["time_series"][-1]["quaternion_wxyz"]
                    expected = [math.cos(math.radians(angle)/2), 0., 0., 0.]
                    expected[axis+1] = math.sin(math.radians(angle)/2)
                    for actual, target in zip(q, expected):
                        self.assertAlmostEqual(actual, target, places=12)
                    self.assertAlmostEqual(result["final_relative_shortest_rotation_deg"], abs(angle), places=12)
                    for row in result["time_series"]:
                        self.assertAlmostEqual(math.hypot(*row["quaternion_wxyz"]), 1., places=14)

    def test_sensor_frame_composition_is_right_multiplied_and_scalar_first(self):
        # Endpoint rates make the interval midpoint rotations exactly +90 X,
        # then +90 current-sensor Y. q = q_x * q_y, not q_y * q_x.
        result = replay.integrate_gyro([1_000_000_000, 1_100_000_000, 1_200_000_000],
                                      [[10*math.pi, 0., 0.], [0., 0., 0.], [0., 10*math.pi, 0.]])
        q = result["time_series"][-1]["quaternion_wxyz"]
        for component in q:
            self.assertAlmostEqual(component, .5, places=14)
        # q maps the current sensor's +Z to initial sensor +X.
        x_from_z = 2*(q[1]*q[3]+q[0]*q[2])
        y_from_z = 2*(q[2]*q[3]-q[0]*q[1])
        z_from_z = 1-2*(q[1]**2+q[2]**2)
        for actual, expected in zip((x_from_z, y_from_z, z_from_z), (1., 0., 0.)):
            self.assertAlmostEqual(actual, expected, places=14)

    def test_shortest_rotation_is_not_accumulated_path_length(self):
        times = [1_000_000_000+i*100_000_000 for i in range(41)]
        result = replay.integrate_gyro(times, [[0., 0., math.pi/2]]*len(times))
        self.assertAlmostEqual(result["final_relative_shortest_rotation_deg"], 0., places=12)
        self.assertAlmostEqual(result["max_relative_shortest_rotation_deg"], 180., places=12)

    def test_invalid_direct_integration_inputs_rejected(self):
        for times, rates, bias in (
            ([1, 1], [[0., 0., 0.]]*2, (0., 0., 0.)),
            ([1, 300_000_001], [[0., 0., 0.]]*2, (0., 0., 0.)),
            ([1, 2], [[0., float("nan"), 0.]]*2, (0., 0., 0.)),
            ([1, 2], [[0., 0., 0.]]*2, (0., float("inf"), 0.)),
            ([1, 2], [[0., 0., 0.]], (0., 0., 0.)),
        ):
            with self.subTest(times=times, bias=bias), self.assertRaises(BaselineError):
                replay.integrate_gyro(times, rates, bias_rad_s=bias)


class OrientationReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.a, self.b = self.root/"a", self.root/"b"
        self.a.mkdir()
        self.b.mkdir()
        self.ma, self.ra = synthetic_capture(11, 100)
        self.mb, self.rb = synthetic_capture(22, 140)
        self.save()

    def save(self):
        for path, metadata, rows in ((self.a, self.ma, self.ra), (self.b, self.mb, self.rb)):
            (path/"summary.json").write_text(json.dumps(metadata)+"\n")
            records = [{"kind": "capture_metadata", **metadata["plan"]}]+rows
            (path/"events.jsonl").write_text("\n".join(json.dumps(r) for r in records)+"\n")

    def run_replay(self):
        return replay.replay_captures(self.a, self.b, operator_confirmed_stationary=True)

    def test_bias_fits_only_a_and_b_retains_independent_residual(self):
        scale = self.ma["configuration"]["gyro_rad_s_per_lsb"]
        for rows, raw_x in ((self.ra, 40), (self.rb, 50)):
            for row in rows:
                row["raw_gyro"] = [raw_x, 0, 0]
                row["gyro_rad_s"] = [raw_x*scale, 0., 0.]
        refresh_summary(self.ma, self.ra)
        refresh_summary(self.mb, self.rb)
        self.save()
        result = self.run_replay()
        self.assertEqual(result["gyro_bias_candidate_rad_s"], [40*scale, 0., 0.])
        duration = result["actual_duration_s"]
        self.assertAlmostEqual(result["raw"]["final_relative_shortest_rotation_deg"],
                               math.degrees(50*scale*duration), places=11)
        self.assertAlmostEqual(result["subtract_a_bias"]["final_relative_shortest_rotation_deg"],
                               math.degrees(10*scale*duration), places=11)
        # B alone changes; its residual changes while the A-only candidate does not.
        for row in self.rb:
            row["raw_gyro"][0] = 40
            row["gyro_rad_s"][0] = 40*scale
        refresh_summary(self.mb, self.rb)
        self.save()
        changed = self.run_replay()
        self.assertEqual(changed["gyro_bias_candidate_rad_s"], result["gyro_bias_candidate_rad_s"])
        self.assertAlmostEqual(changed["subtract_a_bias"]["final_relative_shortest_rotation_deg"], 0., places=12)

    def test_original_measurements_and_relative_frame_are_preserved(self):
        result = self.run_replay()
        self.assertEqual(result["actual_duration_s"], (self.rb[-1]["monotonic_ns"]-self.rb[0]["monotonic_ns"])/1e9)
        self.assertEqual(result["temperature_c"], self.mb["summary"]["temperature_c"])
        self.assertEqual(result["quaternion_convention"]["maps_from"], "sensor_current")
        self.assertEqual(result["quaternion_convention"]["maps_to"], "sensor_at_t0")
        for flag in ("approved_for_runtime", "automatically_applied", "hardware_opened", "motor_output_available",
                     "calibration_verified", "accel_bias_estimated", "accel_scale_estimated",
                     "mount_rotation_applied", "absolute_level_verified", "sensor_fusion_applied"):
            self.assertIs(result[flag], False)
        monitor = result["acceleration_monitor"]
        self.assertFalse(monitor["used_for_orientation_correction"])
        self.assertGreater(monitor["norm_deviation_from_1g_percent"], 9.)
        for actual, original in zip(monitor["time_series"], self.rb):
            self.assertEqual(actual["uncorrected_accel_m_s2"], original["accel_m_s2"])
            self.assertEqual(actual["raw_accel_lsb"], original["raw_accel"])
            self.assertAlmostEqual(math.hypot(*actual["specific_force_direction_sensor_unit"]), 1.)
        self.assertEqual(result["raw"]["time_series"][0]["quaternion_wxyz"], [1., 0., 0., 0.])

    def test_no_implicit_stationary_approval(self):
        for value in (False, None, 1, "yes"):
            with self.subTest(value=value), self.assertRaisesRegex(BaselineError, "explicit"):
                replay.replay_captures(self.a, self.b, operator_confirmed_stationary=value)
        with self.assertRaisesRegex(BaselineError, "explicit"):
            replay.replay_captures(self.a, self.b)

    def test_missing_nonfinite_corrected_and_discontinuous_samples_use_validator(self):
        original = copy.deepcopy(self.rb)
        for change in ("missing", "nan", "nonfinite_json_number", "corrected", "gap", "configuration"):
            self.rb = copy.deepcopy(original)
            if change == "missing":
                del self.rb[5]["gyro_rad_s"]
            elif change == "nan":
                self.rb[5]["gyro_rad_s"][0] = float("nan")
            elif change == "nonfinite_json_number":
                self.rb[5]["gyro_rad_s"][0] = float("inf")
            elif change == "corrected":
                self.rb[5]["gyro_bias_subtracted"] = True
            elif change == "gap":
                for row in self.rb[5:]:
                    for key in ("monotonic_ns", "wall_time_ns", "read_started_monotonic_ns", "read_finished_monotonic_ns"):
                        row[key] += 500_000_000
            else:
                self.mb["configuration"]["gyro_rad_s_per_lsb"] *= 2
            self.save()
            with self.subTest(change=change), self.assertRaises(BaselineError):
                self.run_replay()

    def test_failed_independent_validation_rejects_replay(self):
        for row in self.rb:
            row["raw_gyro"][0] += 300
            row["gyro_rad_s"][0] = row["raw_gyro"][0]*self.mb["configuration"]["gyro_rad_s_per_lsb"]
        refresh_summary(self.mb, self.rb)
        self.save()
        with self.assertRaisesRegex(BaselineError, "gyro_repeatability"):
            self.run_replay()

    def test_changed_b_after_comparison_is_rejected(self):
        original_compare = replay.baseline.compare_captures

        def compare_then_change(*args, **kwargs):
            result = original_compare(*args, **kwargs)
            # Valid bytes changed, so the second validation succeeds but provenance differs.
            path = self.b/"summary.json"
            path.write_text(path.read_text()+"\n")
            return result

        with patch.object(replay.baseline, "compare_captures", side_effect=compare_then_change):
            with self.assertRaisesRegex(BaselineError, "changed during replay"):
                self.run_replay()

    def test_private_new_output_refuses_existing_git_and_nonfinite_results(self):
        before = {p: p.read_bytes() for d in (self.a, self.b) for p in d.iterdir()}
        output = self.root/"replay.json"
        result = replay.write_replay(self.a, self.b, output, operator_confirmed_stationary=True)
        self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
        self.assertEqual(json.loads(output.read_text()), result)
        self.assertEqual(before, {p: p.read_bytes() for p in before})
        with self.assertRaises(FileExistsError):
            replay.write_replay(self.a, self.b, output, operator_confirmed_stationary=True)
        git = self.root/"repository"
        git.mkdir()
        (git/".git").write_text("gitdir: elsewhere\n")
        alias = self.root/"alias"
        alias.symlink_to(git, target_is_directory=True)
        for parent in (git, alias):
            with self.assertRaisesRegex(BaselineError, "outside Git"):
                replay.write_replay(self.a, self.b, parent/"replay.json", operator_confirmed_stationary=True)
        nonfinite = self.root/"nonfinite.json"
        with patch.object(replay, "replay_captures", return_value={"value": float("nan")}):
            with self.assertRaises(ValueError):
                replay.write_replay(self.a, self.b, nonfinite, operator_confirmed_stationary=True)
        self.assertFalse(nonfinite.exists())

    def test_cli_requires_stationary_flag_and_rejects_invalid_input_without_output(self):
        output = self.root/"cli.json"
        argv = ["--base-a", str(self.a), "--base-b", str(self.b), "--output", str(output)]
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            replay.main(argv)
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(output.exists())
        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(replay.main(argv+["--operator-confirmed-stationary"]), 0)
        self.assertFalse(json.loads(stdout.getvalue())["approved_for_runtime"])
        output.unlink()
        del self.rb[4]["raw_accel"]
        self.save()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            replay.main(argv+["--operator-confirmed-stationary"])
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
