"""Offline record-validation tests; all numeric calibrations here are fixtures."""

from contextlib import redirect_stdout
import copy
import io
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from singularitydog_hw.commissioning import (
    load_commissioning, main, validate_commissioning,
)


EXAMPLE = Path(__file__).resolve().parents[1] / "config" / "commissioning.example.json"


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        self.example = load_commissioning(EXAMPLE)

    def complete_fixture(self):
        data = copy.deepcopy(self.example)
        for index, motor in enumerate(data["motors"]):
            motor["joint"].update(model_joint_name="fixture_joint_%d" % motor["id"],
                                  semantic_role=("hip_pitch", "knee", "hip_roll")[index % 3],
                                  verified=True, evidence=["fixture mapping evidence"])
            motor["calibration"].update(verified=True, offset_rad=0.0, sign=1,
                                        measured_rom_rad={"min": -1.0, "max": 1.0},
                                        evidence=["fixture measured calibration"])
            motor["communication_loss_stop"].update(verified=True, measured_stop_latency_s=0.1,
                                                     evidence=["fixture measured loss-of-communication test"])
        data["imu"]["rotation"].update(verified=True,
            matrix_sensor_to_body=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            artifact="fixture-mount.json", evidence=["fixture reviewed mounting evidence"])
        data["imu"]["bias_and_scale"].update(verified=True,
            accel_bias_m_s2=[0, 0, 0], accel_scale=[1, 1, 1], gyro_bias_rad_s=[0, 0, 0],
            artifact="fixture-six-face.json", evidence=["fixture reviewed calibration evidence"])
        for item in data["physical"].values():
            item.update(verified=True, evidence=["fixture reviewed physical test"])
        return data

    def test_example_defaults_deny_and_preserves_known_unknown_timeout_values(self):
        blockers = validate_commissioning(self.example)
        self.assertTrue(blockers)
        self.assertTrue(any("semantic_role" in b for b in blockers))
        self.assertTrue(any("communication_loss_stop" in b for b in blockers))
        self.assertTrue(any("physical.power_cut" in b for b in blockers))
        unknown = {m["id"] for m in self.example["motors"]
                   if m["observed_can_timeout"]["value_ticks"] is None}
        self.assertEqual(unknown, {6, 9, 10})
        self.assertEqual(sum(m["observed_can_timeout"]["value_ticks"] == 0
                             for m in self.example["motors"]), 9)
        for motor in self.example["motors"]:
            self.assertIsNone(motor["joint"]["semantic_role"])
            self.assertIsNone(motor["calibration"]["offset_rad"])

    def test_complete_record_fixture_and_joint_role_order_is_not_assumed(self):
        data = self.complete_fixture()
        self.assertEqual(validate_commissioning(data), ())
        data["motors"][0]["joint"]["semantic_role"] = "knee"
        data["motors"][1]["joint"]["semantic_role"] = "hip_pitch"
        self.assertEqual(validate_commissioning(data), ())

    def test_false_verification_and_missing_evidence_fail_closed(self):
        for value in (False, None, 1, "true"):
            data = self.complete_fixture()
            data["physical"]["power_cut"]["verified"] = value
            self.assertTrue(validate_commissioning(data))
        for value in ([], None, [""], ["  "], [1], "not a list"):
            data = self.complete_fixture()
            data["motors"][0]["calibration"]["evidence"] = value
            self.assertTrue(validate_commissioning(data))
        data = self.complete_fixture()
        data["imu"]["bias_and_scale"]["artifact"] = None
        self.assertTrue(validate_commissioning(data))

    def test_timeout_values_never_replace_individual_measured_stop_proof(self):
        for ticks in (None, 0, 20000):
            data = self.complete_fixture()
            timeout = data["motors"][0]["observed_can_timeout"]
            timeout.update(value_ticks=ticks, read_status=1 if ticks is None else 0)
            data["motors"][0]["communication_loss_stop"]["verified"] = False
            self.assertTrue(any("communication_loss_stop" in b for b in validate_commissioning(data)))
        data = self.complete_fixture()
        data["motors"][0]["communication_loss_stop"]["measured_stop_latency_s"] = None
        self.assertTrue(validate_commissioning(data))
        data = self.complete_fixture()
        data["motors"][5]["observed_can_timeout"]["value_ticks"] = 0
        self.assertTrue(any("null/unknown" in b for b in validate_commissioning(data)))

    def test_id_joint_name_and_leg_role_uniqueness(self):
        mutations = (
            lambda d: d["motors"].pop(),
            lambda d: d["motors"][0].update(id=2),
            lambda d: d["motors"][0].update(id=True),
            lambda d: d["legs"].update(FR=[3, 2, 1]),
            lambda d: d["motors"][0]["joint"].update(model_joint_name="fixture_joint_2"),
            lambda d: d["motors"][0]["joint"].update(semantic_role="knee"),
        )
        for mutate in mutations:
            data = self.complete_fixture()
            mutate(data)
            self.assertTrue(validate_commissioning(data))

    def test_nonfinite_sign_rom_and_bias_validation(self):
        for value in (math.nan, math.inf, -math.inf, True, "0", None):
            data = self.complete_fixture()
            data["motors"][0]["calibration"]["offset_rad"] = value
            self.assertTrue(validate_commissioning(data))
        for sign in (0, 2, True, 1.0, None):
            data = self.complete_fixture()
            data["motors"][0]["calibration"]["sign"] = sign
            self.assertTrue(validate_commissioning(data))
        for rom in ({"min": 1, "max": 1}, {"min": 2, "max": 1}, {"min": None, "max": 1}):
            data = self.complete_fixture()
            data["motors"][0]["calibration"]["measured_rom_rad"] = rom
            self.assertTrue(validate_commissioning(data))
        for field, value in (("gyro_bias_rad_s", [0, math.nan, 0]),
                             ("accel_bias_m_s2", [0, 0]), ("accel_scale", [1, 0, 1])):
            data = self.complete_fixture()
            data["imu"]["bias_and_scale"][field] = value
            self.assertTrue(validate_commissioning(data))

    def test_rotation_rejects_reflection_scaling_shear_and_malformed_values(self):
        matrices = (
            [[-1, 0, 0], [0, 1, 0], [0, 0, 1]],
            [[2, 0, 0], [0, 1, 0], [0, 0, 1]],
            [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]],
            [[1, 0, 0], [0, 1, 0]],
            [[True, 0, 0], [0, 1, 0], [0, 0, 1]],
            [[math.nan, 0, 0], [0, 1, 0], [0, 0, 1]],
        )
        for matrix in matrices:
            data = self.complete_fixture()
            data["imu"]["rotation"]["matrix_sensor_to_body"] = matrix
            self.assertTrue(validate_commissioning(data))
        data = self.complete_fixture()
        data["imu"]["rotation"]["matrix_sensor_to_body"] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        self.assertEqual(validate_commissioning(data), ())

    def test_missing_wrong_schema_and_unknown_fields_fail_closed(self):
        for value in (None, [], {}, {"schema_version": True}):
            self.assertTrue(validate_commissioning(value))
        data = self.complete_fixture()
        data["schema_version"] = 2
        self.assertTrue(validate_commissioning(data))
        data = self.complete_fixture()
        data["motors"][0]["joint"]["verifyed"] = True
        self.assertTrue(validate_commissioning(data))

    def test_strict_json_rejects_duplicate_keys_nonfinite_and_missing_files(self):
        for text in ('{"schema_version":1,"schema_version":1}', '{"x":NaN}', '{"x":Infinity}'):
            with patch.object(Path, "read_text", return_value=text), self.assertRaises(ValueError):
                load_commissioning("unused.json")
        out = io.StringIO()
        with patch.object(Path, "read_text", side_effect=OSError("missing")), redirect_stdout(out):
            self.assertEqual(main(["--config", "missing.json"]), 1)
        self.assertTrue(json.loads(out.getvalue())["blockers"])

    def test_cli_reports_blockers_only_and_cannot_arm(self):
        out = io.StringIO()
        with redirect_stdout(out):
            result = main(["--config", str(EXAMPLE)])
        self.assertEqual(result, 1)
        self.assertEqual(set(json.loads(out.getvalue())), {"blockers"})
        out = io.StringIO()
        with patch.object(Path, "read_text", return_value=json.dumps(self.complete_fixture())), redirect_stdout(out):
            self.assertEqual(main(["--config", "fixture.json"]), 0)
        self.assertEqual(json.loads(out.getvalue()), {"blockers": []})


if __name__ == "__main__":
    unittest.main()
