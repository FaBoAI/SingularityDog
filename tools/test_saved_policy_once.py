"""Saved-only inference gate tests with a fake policy; no device or model files."""

import contextlib
import copy
import io
import json
import struct
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw.telemetry_snapshot import TelemetrySnapshotBuffer
from singularitydog_hw import fast_policy_inputs as fast
from tools import saved_policy_once as saved


class Tensor:
    def __init__(self, values):
        self.values = copy.deepcopy(values)

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return copy.deepcopy(self.values)


class FakeTorch:
    long, float32 = "long", "float32"
    tensor = staticmethod(lambda values, dtype=None: Tensor(values))
    inference_mode = staticmethod(contextlib.nullcontext)


class Policy:
    def __init__(self, target=None):
        self.calls = 0
        self.resets = 0
        self.target = target or [0., .4, -.8] * 4

    def reset(self, ids):
        self.resets += 1

    def __call__(self, *inputs):
        self.calls += 1
        self.last_actor_output = Tensor([[0.] * 12])
        self.last_observation = Tensor([[0.] * 74])
        return Tensor([self.target])


def calibration():
    return {"status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
            "formula": "q_model = sign * raw + offset; rad; no wrapping",
            "model_can_order_candidate": [6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7],
            "identities": {str(i): f"{i:016x}" for i in range(1, 13)},
            "approved_for_runtime": False, "physical_angle_accuracy_verified": False,
            "candidates": [{"motor_id": i, "sign_candidate": 1,
                            "offset_candidate_rad": 0., "approved_for_runtime": False}
                           for i in range(1, 13)]}


def mount():
    return {"schema_version": 1, "status": "IMU_MOUNT_CANDIDATE_ONLY",
            "input_frame": "sensor", "output_frame": "body_x_forward_y_left_z_up",
            "R_body_from_sensor": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "raw_driver_axes_verified": False, "approved_for_runtime": False,
            "provenance": {"source": "synthetic unit test"}}


def report():
    tick = 1_000_000_000
    buffer = TelemetrySnapshotBuffer(max_age_ns=10_000_000, max_spread_ns=5_000_000)
    order = calibration()["model_can_order_candidate"]
    for index, mid in enumerate(order):
        q = [0., .4, -.8][index % 3]
        for name, value, unit in (("position", q, "rad"), ("velocity", 0., "rad_s")):
            buffer.ingest_motor(can_type=17, motor_id=mid, parameter=name,
                                value=value, unit=unit, request_ns=tick-3_000_000,
                                received_ns=tick-1_000_000)
    buffer.ingest_imu(accel_m_s2=[0., 0., -10.], gyro_rad_s=[0., 0., 0.],
                      read_started_ns=tick-2_000_000, read_finished_ns=tick-1_500_000)
    return {"status": "COMPLETE_INTEGRATED_STOP_PROXY", "snapshot": buffer.snapshot(tick).as_dict()}


def reconstructed_fixture(root):
    """Write synthetic saved Type2 and IMU rows with a real encoded raw frame."""
    root = Path(root)
    tick = 1_000_000_000
    candidate = calibration()
    target = [0., .4, -.8] * 4
    for index, mid in enumerate(candidate["model_can_order_candidate"]):
        candidate["candidates"][mid-1]["offset_candidate_rad"] = target[index]
    for bus, ids in (("front", range(1, 7)), ("rear", range(7, 13))):
        rows = []
        for mid in ids:
            rows.append({"kind": "pipeline_reply", "cycle": 0, "parameter": "identity",
                         "motor_id": mid, "ok": True,
                         "result": {"ok": True, "mcu_uid_hex": f"{mid:016x}"}})
            data = struct.pack(">4H", 32768, 32768, 32768, 300)
            can_id = (2 << 24) | (mid << 8) | 0xFD
            wire = b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + data + b"\r\n"
            p, v, torque, temp = struct.unpack(">4H", data)
            scale = lambda value, limit: value*(2.*limit)/65535.-limit
            result = dict.fromkeys(fast.UNVERIFIED_FLAGS, False)
            result.update({"ok": True, "motor_id": mid, "parameter": "stop_feedback",
                           "mode_state": 0, "fault_bits": 0, "position_u16": p,
                           "velocity_u16": v, "torque_u16": torque, "temperature_u16": temp,
                           "position_rad_candidate": scale(p, 12.57),
                           "velocity_rad_s_candidate": scale(v, 50.),
                           "torque_nm_candidate": scale(torque, 5.5),
                           "temperature_c": temp/10.,
                           "position_velocity_in_one_reply": True,
                           "raw_frame": {"can_id": can_id, "type": 2, "source_id": mid,
                                         "destination_id": 0xFD, "flags": 4,
                                         "data_hex": data.hex(), "wire_hex": wire.hex()}})
            rows.append({"kind": "pipeline_reply", "cycle": 1,
                         "motor_id": mid, "parameter": "stop_feedback", "ok": True,
                         "write_call_entered": True, "write_expected_bytes": 17,
                         "write_returned_bytes": 17,
                         "write_started_monotonic_ns": tick-20_000_000+mid,
                         "write_finished_monotonic_ns": tick-19_000_000+mid,
                         "received_monotonic_ns": tick-5_000_000+mid,
                         "deadline_monotonic_ns": tick+100_000_000,
                         "result": result})
        (root/(bus + ".jsonl")).write_text("".join(json.dumps(row)+"\n" for row in rows))
    saved_report = {"status": "INCOMPLETE", "cycles": 1, "output_allowed": False,
                    "learned_target_sent": False, "calibration_verified": False,
                    "h_measured": False,
                    "errors": [{"scope": "coordinator", "error":
                                "ObserverError('Calibrated position outside registered joint range; no clipping')"}]
                              + [{"scope": name, "error":
                                  "InterruptedError('Integrated collection cancelled')"}
                                 for name in ("front", "rear", "imu")],
                    "stamps": {"inputs_ready_ns": tick-500_000,
                               "snapshot_start_ns": tick,
                               "snapshot_end_ns": tick+1_000_000},
                    "imu_sample": {"kind": "imu", "frame": "sensor",
                                   "accel_m_s2": [0., 0., -9.8],
                                   "gyro_rad_s": [0., 0., 0.],
                                   "read_started_monotonic_ns": tick-3_000_000,
                                   "read_finished_monotonic_ns": tick-2_000_000,
                                   "available_monotonic_ns": tick-1_000_000}}
    (root/"report.json").write_text(json.dumps(saved_report))
    return candidate


class SavedPolicyOnceTests(unittest.TestCase):
    def test_one_saved_input_after_reset_with_unverified_output(self):
        policy = Policy()
        result = saved.infer_once(report(), calibration(), mount(), policy, FakeTorch,
                                  h_hypothesis=0)
        self.assertEqual(policy.calls, 4)  # Three labelled synthetic warmups, one saved input.
        self.assertEqual(policy.resets, 1)
        self.assertEqual(result["saved_input_policy_calls"], 1)
        self.assertEqual(len(result["tick"]["observation74"]), 74)
        self.assertFalse(result["output_allowed"])
        self.assertFalse(result["calibration_verified"])
        self.assertFalse(result["live_50hz_verified"])

    def test_calibrated_range_blocks_before_saved_input_call(self):
        source = report()
        next(row for row in source["snapshot"]["motors"]
             if row["motor_id"] == 6 and row["parameter"] == "position")["value"] = 2.
        policy = Policy()
        with self.assertRaisesRegex(ValueError, "Calibrated position outside"):
            saved.infer_once(source, calibration(), mount(), policy, FakeTorch,
                             h_hypothesis=1)
        self.assertEqual(policy.calls, 0)

    def test_policy_target_range_is_checked(self):
        target = [0., .4, -.8] * 4
        target[0] = 2.
        policy = Policy(target)
        with self.assertRaisesRegex(ValueError, "Policy target outside"):
            saved.infer_once(report(), calibration(), mount(), policy, FakeTorch,
                             h_hypothesis=0)
        self.assertEqual(policy.calls, 4)

    def test_incomplete_report_and_unverified_candidate_are_required(self):
        source = report()
        source["status"] = "INCOMPLETE"
        policy = Policy()
        with self.assertRaisesRegex(ValueError, "completed report"):
            saved.infer_once(source, calibration(), mount(), policy, FakeTorch,
                             h_hypothesis=0)
        self.assertEqual(policy.calls, 0)
        source["status"] = "COMPLETE_INTEGRATED_STOP_PROXY"
        candidate = calibration()
        candidate["physical_angle_accuracy_verified"] = True
        with self.assertRaisesRegex(ValueError, "claims verified"):
            saved.infer_once(source, candidate, mount(), policy, FakeTorch,
                             h_hypothesis=0)
        self.assertEqual(policy.calls, 0)

    def test_cli_rejects_bad_saved_report_before_model_load(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = report()
            source["status"] = "INCOMPLETE"
            for name, data in (("report.json", source), ("calibration.json", calibration()),
                               ("mount.json", mount())):
                (root/name).write_text(json.dumps(data))
            output = io.StringIO()
            with patch.object(saved.shadow, "load_policy", side_effect=AssertionError("model loaded")):
                with contextlib.redirect_stdout(output):
                    code = saved.main(["--report", str(root/"report.json"),
                                       "--calibration", str(root/"calibration.json"),
                                       "--imu-mount-candidate", str(root/"mount.json"),
                                       "--bundle", str(root), "--h-hypothesis", "0"])
            self.assertEqual(code, 2)
            self.assertEqual(json.loads(output.getvalue())["status"],
                             "OFFLINE_SAVED_POLICY_BLOCKED")

    def test_reconstruct_range_stopped_source_then_observe_one_saved_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = reconstructed_fixture(tmp)
            source, hashes = saved.reconstruct_range_stopped_capture(tmp, candidate)
        self.assertEqual(source["source_capture_status"], "INCOMPLETE")
        self.assertFalse(source["reconstruction"]["original_tick_known"])
        self.assertEqual(source["snapshot"]["tick_ns"], 1_001_000_000)
        self.assertEqual(len(source["snapshot"]["motors"]), 24)
        self.assertEqual(set(hashes), {"report_sha256", "front_jsonl_sha256",
                                       "rear_jsonl_sha256"})
        policy = Policy()
        result = saved.infer_once(source, candidate, mount(), policy, FakeTorch,
                                  h_hypothesis=0)
        self.assertEqual(policy.calls, 4)
        self.assertTrue(result["source_wire_revalidated_by_this_tool"])
        self.assertEqual(result["source_capture_status"], "INCOMPLETE")
        self.assertFalse(result["reconstruction"]["current_pose_after_capture_verified"])

    def test_reconstructed_capture_wire_and_error_scope_are_rechecked(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = reconstructed_fixture(tmp)
            front = Path(tmp)/"front.jsonl"
            rows = [json.loads(line) for line in front.read_text().splitlines()]
            rows[1]["result"]["raw_frame"]["wire_hex"] = "00" * 17
            front.write_text("".join(json.dumps(row)+"\n" for row in rows))
            with self.assertRaisesRegex(ValueError, "Noncanonical raw Type2 wire"):
                saved.reconstruct_range_stopped_capture(tmp, candidate)
            reconstructed_fixture(tmp)
            saved_report = json.loads((Path(tmp)/"report.json").read_text())
            saved_report["errors"][0]["error"] = "TimeoutError('CAN source missing')"
            (Path(tmp)/"report.json").write_text(json.dumps(saved_report))
            with self.assertRaisesRegex(ValueError, "did not stop solely"):
                saved.reconstruct_range_stopped_capture(tmp, candidate)

    def test_reconstructed_out_of_range_still_blocks_before_model_call(self):
        with tempfile.TemporaryDirectory() as tmp:
            candidate = reconstructed_fixture(tmp)
            source, _ = saved.reconstruct_range_stopped_capture(tmp, candidate)
        candidate["candidates"][5]["offset_candidate_rad"] = 2.
        policy = Policy()
        with self.assertRaisesRegex(ValueError, "motor IDs 6"):
            saved.infer_once(source, candidate, mount(), policy, FakeTorch,
                             h_hypothesis=0)
        self.assertEqual(policy.calls, 0)


if __name__ == "__main__":
    unittest.main()
