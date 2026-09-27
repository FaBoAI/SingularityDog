"""Two-bus capture adapter tests use saved-shaped synthetic data, no devices."""

import copy
import contextlib
import io
import json
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import dual_policy_once as once
from singularitydog_hw import policy_shadow as shadow
from test_policy_observer import FakeTorch, Policy, calibration, mount


def captured(calib=None):
    calib = calib or calibration()
    positions = {}
    nominal = [0., .4, -.8]*4
    for index, mid in enumerate(shadow.CAN_ORDER):
        row = next(row for row in calib["candidates"] if row["motor_id"] == mid)
        positions[mid] = (nominal[index]-row["offset_candidate_rad"])/row["sign_candidate"]
    events = {"front": [], "rear": []}
    start = 1_000_000_000
    for bus, ids in (("front", range(1, 7)), ("rear", range(7, 13))):
        for mid in ids:
            events[bus].append({"kind": "pipeline_reply", "cycle": 0,
                "motor_id": mid, "parameter": "identity", "ok": True,
                "result": {"ok": True, "mcu_uid_hex": calib["identities"][str(mid)]}})
        for cycle in range(1, 21):
            for mid in ids:
                for slot, parameter in enumerate(("position", "velocity")):
                    request = start + cycle*20_000_000 + mid*100_000 + slot*20_000
                    value = positions[mid] if parameter == "position" else 0.
                    events[bus].append({"kind": "pipeline_reply", "cycle": cycle,
                        "motor_id": mid, "parameter": parameter, "ok": True,
                        "write_started_monotonic_ns": request,
                        "write_finished_monotonic_ns": request+10_000,
                        "received_monotonic_ns": request+30_000,
                        "result": {"ok": True, "motor_id": mid,
                                   "parameter": parameter, "status": 0,
                                   "index": 0x7019 if parameter == "position" else 0x701B,
                                   "unit": "rad_output_shaft" if parameter == "position"
                                           else "rad_s_output_shaft", "value": value}})
    imu = []
    for sequence in range(1, 43):
        request = start + sequence*10_000_000
        imu.append({"kind": "imu", "frame": "sensor", "sequence": sequence,
                    "accel_m_s2": [0., 0., -10.], "gyro_rad_s": [0., 0., 0.],
                    "read_started_monotonic_ns": request,
                    "read_finished_monotonic_ns": request+100_000})
    report = {"status": "COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT",
              "output_allowed": False, "producer_threads_exited": True,
              "common_lock_released": True, "errors": []}
    return report, events, imu, calib


class DualPolicyOnceTests(unittest.TestCase):
    def test_complete_two_bus_capture_calls_policy_once_after_warmup(self):
        report, events, imu, calib = captured()
        policy = Policy()
        result = once.observe_one(report, events, imu, calib, mount(), policy,
            FakeTorch, h_hypothesis=0, boot_id="synthetic-boot",
            motor_power_epoch="UNVERIFIED")
        self.assertEqual(result["status"], "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT")
        self.assertEqual(result["inference_calls_on_captured_input"], 1)
        self.assertEqual(len(policy.calls), 4)  # Three synthetic warmups + one real input.
        self.assertEqual(len(result["observer_tick"]["observation74"]), 74)
        self.assertEqual(len(result["observer_tick"]["q_target_rad_diagnostic_only"]), 12)
        self.assertFalse(result["output_allowed"])
        self.assertFalse(result["learned_target_sent"])
        self.assertFalse(result["live_50hz_verified"])
        self.assertFalse(result["motor_power_epoch_attested"])

    def test_uid_mismatch_blocks_before_model_call(self):
        report, events, imu, calib = captured()
        events["rear"][0]["result"]["mcu_uid_hex"] = "f"*16
        policy = Policy()
        with self.assertRaisesRegex(ValueError, "identity does not match"):
            once.observe_one(report, events, imu, calib, mount(), policy,
                FakeTorch, h_hypothesis=0, boot_id="synthetic-boot",
                motor_power_epoch="operator-epoch-1")
        self.assertEqual(policy.calls, [])

    def test_missing_or_stale_imu_and_incomplete_bus_block_inference(self):
        report, events, imu, calib = captured()
        with self.assertRaisesRegex(ValueError, "bounded IMU"):
            once.observe_one(report, events, [], calib, mount(), Policy(),
                FakeTorch, h_hypothesis=0, boot_id="synthetic-boot",
                motor_power_epoch="operator-epoch-1")
        stale = copy.deepcopy(imu)
        stale[:] = stale[:4]
        with self.assertRaisesRegex(ValueError, "incomplete or stale"):
            once.observe_one(report, events, stale, calib, mount(), Policy(),
                FakeTorch, h_hypothesis=0, boot_id="synthetic-boot",
                motor_power_epoch="operator-epoch-1")
        events["front"].pop()
        with self.assertRaisesRegex(ValueError, "Incomplete twelve-axis"):
            once.observe_one(report, events, imu, calib, mount(), Policy(),
                FakeTorch, h_hypothesis=0, boot_id="synthetic-boot",
                motor_power_epoch="operator-epoch-1")
        report, events, imu, calib = captured()
        for row in events["front"]:
            if row.get("parameter") == "position" and row.get("motor_id") == 3:
                row["result"]["value"] += 2*math.pi
        policy = Policy()
        with self.assertRaisesRegex(ValueError, "outside registered joint range"):
            once.observe_one(report, events, imu, calib, mount(), policy,
                FakeTorch, h_hypothesis=0, boot_id="synthetic-boot",
                motor_power_epoch="UNVERIFIED", warmup_completed=True)
        self.assertEqual(policy.calls, [])

    def test_imu_worker_is_stopped_if_can_collector_fails(self):
        active = []
        audit = {}
        def fake_imu(bus, deadline_ns, **kwargs):
            active.append(bus)
            bus.stop.wait(2)
            bus.producer_status["imu"] = {"exited": True, "restore_status": "restored"}
        with patch.object(once.live, "imu_producer", fake_imu), \
             patch.object(once.dual, "collect_dual", side_effect=RuntimeError("CAN failed")):
            with self.assertRaisesRegex(RuntimeError, "CAN failed"):
                once.collect_once({"front": {}, "rear": {}}, {}, audit_status=audit)
        self.assertEqual(len(active), 1)
        self.assertTrue(active[0].stop.is_set())
        self.assertEqual(audit, {"imu_worker_exited": True,
                                 "imu_restore_status": "restored"})

    def test_receive_mode_reaches_both_bus_workers(self):
        def fake_imu(bus, deadline_ns, **kwargs):
            bus.stop.wait(2)
            bus.producer_status["imu"] = {"exited": True, "restore_status": "restored"}
        with patch.object(once.live, "imu_producer", fake_imu), \
             patch.object(once.dual, "collect_dual", return_value={}) as collect:
            audit = {}
            once.collect_once({"front": {}, "rear": {}}, {}, audit_status=audit)
            self.assertEqual(collect.call_args.kwargs["receive_mode"], "select")
            self.assertIs(collect.call_args.kwargs["paired_cycle_sync"], True)
            self.assertEqual(audit, {"imu_worker_exited": True,
                                     "imu_restore_status": "restored"})
            once.collect_once({"front": {}, "rear": {}}, {}, receive_mode="serial")
            self.assertEqual(collect.call_args.kwargs["receive_mode"], "serial")

    def test_dry_run_prints_plan_without_opening_devices(self):
        argv = ["--expected-uids", "/abs/never-read", "--front-port", "/abs/front",
                "--rear-port", "/abs/rear", "--calibration", "/abs/calibration",
                "--imu-mount-candidate", "/abs/mount", "--bundle", "/abs/bundle",
                "--output", "/abs/output", "--h-hypothesis", "0",
                "--current-motor-power-epoch", "UNVERIFIED"]
        output = io.StringIO()
        with patch.object(once.dual, "validate_ports", side_effect=AssertionError("device opened")), \
             contextlib.redirect_stdout(output):
            self.assertEqual(once.main(argv), 0)
        self.assertEqual(json.loads(output.getvalue())["receive_mode"], "select")
        self.assertIs(json.loads(output.getvalue())["paired_cycle_sync"], True)


if __name__ == "__main__":
    unittest.main()
