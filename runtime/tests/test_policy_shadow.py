"""Pure-file diagnostic tests; synthetic measurements and fake tensor backend."""
import ast
import contextlib
import copy
import inspect
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_shadow as shadow


def calibration():
    return {"status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
            "formula": "q_model = sign * raw + offset; rad; no wrapping",
            "model_can_order_candidate": list(shadow.CAN_ORDER),
            "identities": {str(i): f"{i:016x}" for i in range(1, 13)},
            "approved_for_runtime": False, "zero_verified": False, "sign_verified": False,
            "motor_power_cycle_continuity_verified": False,
            "candidates": [{"motor_id": i, "sign_candidate": -1 if i % 2 else 1,
                            "offset_candidate_rad": .01 * i, "approved_for_runtime": False,
                            "zero_verified": False, "sign_verified": False}
                           for i in range(1, 13)]}


def imu(stamp=2_000_000_000, gyro=None, accel=None):
    return {"kind": "imu", "frame": "sensor", "monotonic_ns": stamp,
            "read_started_monotonic_ns": stamp - 100_000,
            "read_finished_monotonic_ns": stamp + 100_000,
            "accel_m_s2": accel or [0., 0., 8.9],
            "gyro_rad_s": gyro or [.01, .02, .03]}


def records():
    data = [{"kind": "motor_parameter", "motor_id": i, "parameter": "identity", "ok": True,
             "mcu_uid_hex": f"{i:016x}", "monotonic_ns": 1_000_000_000 + i * 1000}
            for i in range(1, 13)]
    data.append(imu())
    for i in range(1, 13):
        for n, value in (("position", .1 * i), ("velocity", .01 * i)):
            data.append({"kind": "motor_parameter", "motor_id": i, "parameter": n,
                         "ok": True, "status": 0, "unit": shadow.PARAMETERS[n][2], "value": value,
                         "monotonic_ns": 2_000_000_000 + (2 * i - (n == "position")) * 1_000_000})
    return data


def samples(data=None, candidate=None, **kwargs):
    return shadow.build_samples(records() if data is None else data,
                                calibration() if candidate is None else candidate,
                                assume_sensor_aligned=True, **kwargs)


def wire(kind, source, dest, payload):
    cid = (kind << 24) | (source << 8) | dest
    return (b"AT" + ((cid << 3) | 4).to_bytes(4, "big") + b"\x08" + payload + b"\r\n").hex()


def capture_fixture(directory):
    summary = {"status": "COMPLETE", "errors": [], "warnings": [], "plan": {
        "motor_output_available": False, "allowed_can_types": [0, 17],
        "ids_bottom_to_top": {"FR": [1, 2, 3], "FL": [4, 5, 6], "RR": [7, 8, 9], "RL": [10, 11, 12]}}}
    events = []
    sequence = 0
    for original in records():
        if original["kind"] == "imu":
            events.append(original)
            continue
        event = copy.deepcopy(original)
        sequence += 1
        i, name = event["motor_id"], event["parameter"]
        requested = event["monotonic_ns"] - 500
        event.update(sequence=sequence, request_monotonic_ns=requested)
        if name == "identity":
            kind, request, reply, dest = 0, bytes(8), bytes.fromhex(event["mcu_uid_hex"]), 0xFE
        else:
            kind, dest = 17, 0xFD
            index = struct.pack("<H", shadow.PARAMETERS[name][0])
            request = index + bytes(6)
            packed = struct.pack("<f", event["value"])
            reply = index + bytes(2) + packed
            event["value"] = struct.unpack("<f", packed)[0]
        events.extend([
            {"kind": "can_tx", "sequence": sequence, "motor_id": i, "parameter": name,
             "monotonic_ns": requested, "hex": wire(kind, 0xFD, i, request)},
            {"kind": "can_rx_frame", "monotonic_ns": requested + 250,
             "wire_hex": wire(kind, i, dest, reply)}, event])
    write_capture(directory, summary, events)
    return summary, events


def write_capture(directory, summary, events):
    (Path(directory) / "summary.json").write_text(json.dumps(summary))
    (Path(directory) / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))


class Tensor:
    def __init__(self, values):
        self.values = values
    def detach(self):
        return self
    def cpu(self):
        return self
    def tolist(self):
        return self.values


class FakeTorch:
    long, float32 = "long", "float32"
    @staticmethod
    def tensor(values, dtype=None):
        return Tensor(values)
    @staticmethod
    def inference_mode():
        return contextlib.nullcontext()


class Policy:
    def __init__(self):
        self.resets, self.calls, self.ready = 0, [], False
    def reset(self, ids):
        assert ids.tolist() == [0]
        self.resets += 1
        self.ready = True
    def __call__(self, gyro, gravity, command, q, dq, h):
        assert self.ready, "each hypothesis must cold-reset"
        self.ready = False
        values = [v.tolist()[0] for v in (gyro, gravity, command, q, dq, h)]
        self.calls.append(values)
        self.last_actor_output = Tensor([[.1] * 12])
        self.last_observation = Tensor([[.2] * 74])
        return Tensor([[x * .01 for x in values[-1]]])


class SampleTests(unittest.TestCase):
    def test_explicit_hypothesis_is_mandatory(self):
        with self.assertRaises(ValueError):
            shadow.build_samples(records(), calibration())
        with self.assertRaises(ValueError):
            shadow.build_samples(records(), calibration(), assume_sensor_aligned=1)

    def test_exact_order_units_signs_and_out_of_range_values_are_preserved(self):
        raw = records()
        next(e for e in raw if e.get("motor_id") == 6 and e.get("parameter") == "position")["value"] = 2.
        before = copy.deepcopy(raw)
        row = samples(raw)[0]
        self.assertEqual(raw, before)
        self.assertAlmostEqual(row["q_model_rad"][0], 2.06)
        self.assertIn(6, [v["motor_id"] for v in row["violations"]])
        for index, i in enumerate(shadow.CAN_ORDER):
            sign = -1 if i % 2 else 1
            self.assertAlmostEqual(row["dq_model_rad_s"][index], sign * .01 * i)
        self.assertEqual(len(row["can_sample_times_ns"]), 24)
        self.assertAlmostEqual(row["can_skew_s"], .023)
        self.assertFalse(row["motor_output_available"])

    def test_missing_identity_or_wrong_replacement_uid_is_rejected(self):
        for missing in (True, False):
            raw = records()
            entry = next(e for e in raw if e.get("motor_id") == 11 and e.get("parameter") == "identity")
            if missing:
                raw.remove(entry)
            else:
                entry["mcu_uid_hex"] = "f" * 16
            with self.assertRaises(ValueError):
                samples(raw)

    def test_missing_or_duplicate_joint_measurement_is_not_zero_filled(self):
        for duplicate in (False, True):
            raw = records()
            index = next(i for i, e in enumerate(raw) if e.get("motor_id") == 6 and e.get("parameter") == "position")
            if duplicate:
                raw.insert(index, copy.deepcopy(raw[index]))
            else:
                del raw[index]
            with self.assertRaises(ValueError):
                samples(raw)

    def test_bad_units_nonfinite_and_boolean_measurements_are_rejected(self):
        for field, value in (("unit", "degree"), ("value", math.nan), ("value", math.inf), ("value", True)):
            raw = records()
            next(e for e in raw if e.get("parameter") == "velocity")[field] = value
            with self.assertRaises(ValueError):
                samples(raw)
        for value in (math.nan, True):
            candidate = calibration()
            candidate["candidates"][0]["offset_candidate_rad"] = value
            with self.assertRaises(ValueError):
                samples(candidate=candidate)

    def test_imu_uses_causal_read_completion_not_future_or_midpoint_only(self):
        raw = records()
        raw.append(imu(2_023_000_000, gyro=[.4, .5, .6]))
        future = imu(2_023_900_000, gyro=[9., 9., 9.])
        future["read_finished_monotonic_ns"] = 2_025_000_000
        raw.append(future)
        row = samples(raw)[0]
        self.assertEqual(row["gyro_rad_s"], [.4, .5, .6])
        self.assertAlmostEqual(row["imu_age_s"], .001)
        self.assertLessEqual(row["imu_read_finished_monotonic_ns"], row["frame_monotonic_ns"])
        no_past = [e for e in raw if e.get("kind") != "imu"] + [future]
        with self.assertRaises(ValueError):
            samples(no_past)

    def test_raw_acceleration_bias_and_unknown_alignment_remain_visible(self):
        row = samples()[0]
        self.assertEqual(row["raw_accel_norm_m_s2"], 8.9)
        self.assertEqual(row["gravity_body_unit"], [-0., -0., -1.])
        self.assertIn("hypothesis", row["assumptions"]["sensor_to_body_rotation"])
        for key in ("gyro_bias_subtracted", "gravity_fusion_verified", "calibration_verified", "exposure_h_available"):
            self.assertFalse(row["assumptions"][key])

    def test_bad_imu_vectors_or_intervals_are_rejected(self):
        for field, value in (("accel_m_s2", [0., 0., 0.]), ("gyro_rad_s", [0., math.nan, 0.]),
                             ("read_finished_monotonic_ns", 1), ("frame", "body")):
            raw = records()
            next(e for e in raw if e["kind"] == "imu")[field] = value
            with self.assertRaises(ValueError):
                samples(raw)


class InferenceTests(unittest.TestCase):
    def test_each_frame_and_exposure_scenario_resets_without_hidden_sensor_zeros(self):
        frames = samples() * 2
        policy = Policy()
        output = shadow.evaluate_samples(frames, policy, FakeTorch)
        self.assertEqual(policy.resets, 4)
        self.assertEqual([call[-1] for call in policy.calls], [[0.] * 12, [1.] * 12] * 2)
        for result in output:
            self.assertFalse(result["motor_output_available"])
            for scenario in result["scenarios"]:
                self.assertFalse(scenario["h_measured"])
                self.assertTrue(scenario["independent_cold_reset"])
        for call in policy.calls:
            self.assertEqual(call[0], frames[0]["gyro_rad_s"])
            self.assertEqual(call[2], [0., 0., 0.])  # Explicit stop command, not missing sensor data.
            self.assertEqual(call[3], frames[0]["q_model_rad"])
            self.assertEqual(call[4], frames[0]["dq_model_rad_s"])

    def test_nonfinite_policy_output_is_rejected(self):
        class BadPolicy(Policy):
            def __call__(self, *args):
                value = super().__call__(*args)
                value.values[0][0] = math.nan
                return value
        with self.assertRaises(ValueError):
            shadow.evaluate_samples(samples(), BadPolicy(), FakeTorch)

    def test_modified_model_bundle_is_rejected_before_loading_backend(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in shadow.SOURCE_HASHES:
                (Path(tmp) / name).write_bytes(b"altered source")
            with self.assertRaisesRegex(ValueError, "hash"):
                shadow.load_policy(tmp)

    def test_module_imports_no_hardware_or_process_transport(self):
        tree = ast.parse(inspect.getsource(shadow))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports += [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                imports.append(node.module or "")
        forbidden = {"serial", "socket", "subprocess", "can_readonly", "rs05_joint_trial", "rs05_leg_trial", "imu"}
        self.assertFalse({name.split(".")[-1] for name in imports} & forbidden)


class FileTests(unittest.TestCase):
    def test_canonical_capture_redecodes_every_identity_position_and_velocity(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture_fixture(tmp)
            events, report = shadow.load_capture(tmp)
            self.assertEqual(report["raw_redecoded_parameter_count"], 36)
            self.assertEqual(report["events_sha256"], shadow.sha(Path(tmp) / "events.jsonl"))
            self.assertFalse(report["incomplete_source_accepted_for_diagnostics_only"])
            self.assertEqual(len(samples(events)), 1)

    def test_actuating_wire_and_disagreement_between_raw_and_decoded_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            for modification in ("enable_wire", "value", "unit", "correlation"):
                summary, events = capture_fixture(tmp)
                if modification == "enable_wire":
                    events[0]["hex"] = wire(3, 0xFD, 1, bytes(8))
                else:
                    event = next(e for e in events if e["kind"] == "motor_parameter" and e["parameter"] == "position")
                    if modification == "value":
                        event["value"] += .001
                    elif modification == "unit":
                        event["unit"] = "degrees"
                    else:
                        event["request_monotonic_ns"] -= 1
                write_capture(tmp, summary, events)
                with self.subTest(modification=modification), self.assertRaises(ValueError):
                    shadow.load_capture(tmp)

    def test_incomplete_capture_requires_opt_in_and_excludes_timeout_request_and_late_reply(self):
        with tempfile.TemporaryDirectory() as tmp:
            summary, events = capture_fixture(tmp)
            complete_count = len(events)
            tx = {"kind": "can_tx", "sequence": 37, "motor_id": 6, "parameter": "position",
                  "monotonic_ns": 3_000_000_000, "hex": wire(17, 0xFD, 6, struct.pack("<H", 0x7019) + bytes(6))}
            events.extend([tx, {"kind": "can_timeout", "monotonic_ns": 3_100_000_000},
                           {"kind": "can_rx_frame", "wire_hex": "late untrusted bytes"}, imu(3_200_000_000)])
            summary.update(status="INCOMPLETE", errors=[{"component": "can", "error": "TimeoutError('No fresh response: ID6')"}])
            write_capture(tmp, summary, events)
            with self.assertRaises(ValueError):
                shadow.load_capture(tmp)
            prefix, report = shadow.load_capture(tmp, allow_incomplete_capture=True)
            self.assertEqual(len(prefix), complete_count)
            self.assertEqual(report["source_status"], "INCOMPLETE")
            self.assertEqual(report["source_errors"], summary["errors"])
            self.assertEqual(report["excluded_tail_record_count"], 4)
            self.assertTrue(report["incomplete_source_accepted_for_diagnostics_only"])
            self.assertEqual(len(samples(prefix)), 1)
            summary["errors"] = [{"component": "can", "error": "Unexpected serial corruption"}]
            write_capture(tmp, summary, events)
            with self.assertRaises(ValueError):
                shadow.load_capture(tmp, allow_incomplete_capture=True)

    def test_unanswered_request_and_missing_timeout_boundary_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            summary, events = capture_fixture(tmp)
            events.append({"kind": "can_tx", "sequence": 37, "motor_id": 6, "parameter": "position",
                           "monotonic_ns": 3_000_000_000,
                           "hex": wire(17, 0xFD, 6, struct.pack("<H", 0x7019) + bytes(6))})
            write_capture(tmp, summary, events)
            with self.assertRaisesRegex(ValueError, "Unanswered"):
                shadow.load_capture(tmp)
            summary.update(status="INCOMPLETE", errors=[{"component": "can", "error": "TimeoutError('No fresh response: ID6')"}])
            write_capture(tmp, summary, events)
            with self.assertRaisesRegex(ValueError, "timeout boundary"):
                shadow.load_capture(tmp, allow_incomplete_capture=True)

    def test_duplicate_keys_and_nonfinite_json_are_rejected(self):
        for text in ('{"x": 1, "x": 2}', '{"x": NaN}', '{"x": Infinity}'):
            with self.assertRaises(ValueError):
                shadow._json(text)

    def test_cli_rejects_git_output_before_model_or_capture_load(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "repo"
            (repo / ".git").mkdir(parents=True)
            args = ["--bundle", str(repo), "--capture", str(repo), "--calibration", str(repo / "x.json"),
                    "--output", str(repo / "private"), "--assume-sensor-aligned"]
            with patch.object(shadow, "load_capture") as capture, patch.object(shadow, "load_policy") as policy:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    shadow.main(args)
                capture.assert_not_called()
                policy.assert_not_called()
            self.assertFalse((repo / "private").exists())


if __name__ == "__main__":
    unittest.main()
