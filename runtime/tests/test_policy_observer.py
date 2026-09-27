"""Stateful diagnostic contracts with fake tensors; no hardware or model weights."""
import ast
import contextlib
import copy
import hashlib
import inspect
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_observer as observer
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.angle_branch_comparison import StaticBranchComparison, TWO_PI
from singularitydog_hw.telemetry_snapshot import TelemetrySnapshotBuffer


def calibration():
    return {"status": "MANUAL_NOMINAL_CANDIDATES_ONLY",
            "formula": "q_model = sign * raw + offset; rad; no wrapping",
            "model_can_order_candidate": list(shadow.CAN_ORDER),
            "identities": {str(i): f"{i:016x}" for i in range(1, 13)},
            "approved_for_runtime": False, "sign_verified": False,
            "candidates": [{"motor_id": i, "sign_candidate": -1 if i % 2 else 1,
                            "offset_candidate_rad": .01*i, "zero_verified": False}
                           for i in range(1, 13)]}


def mount(rotation=None):
    return {"schema_version": 1, "status": "IMU_MOUNT_CANDIDATE_ONLY",
            "input_frame": "sensor", "output_frame": "body_x_forward_y_left_z_up",
            "R_body_from_sensor": rotation or [[0, 1, 0], [1, 0, 0], [0, 0, -1]],
            "raw_driver_axes_verified": False, "approved_for_runtime": False,
            "provenance": {"source": "synthetic test candidate"}}


def bias_candidate():
    return {"schema_version": 1, "kind": "fixed_mount_baseline", "status": "GYRO_BIAS_CANDIDATE",
            "frame": "sensor", "axis_order": ["x", "y", "z"],
            "gyro_bias_candidate_eligible": True, "operator_confirmed_stationary": True,
            "approved_for_runtime": False, "automatically_applied": False,
            "mount_rotation_applied": False, "stationarity_verified_by_software": False,
            "gyro_bias_candidate_rad_s": [.01, .02, .03],
            "captures": {"a": {"gyro_mean_rad_s": [.01, .02, .03]}},
            "provenance": {"a": {"summary_sha256": "a"*64, "events_sha256": "b"*64},
                           "b": {"summary_sha256": "c"*64, "events_sha256": "d"*64}}}


def snapshot(tick=1_000_000_000, *, calib=None, buffer=None):
    c = calib or calibration()
    rows = {r["motor_id"]: r for r in c["candidates"]}
    target_q = [0., .4, -.8]*4
    target_dq = [.02*(i+1) for i in range(12)]
    buffer = buffer or TelemetrySnapshotBuffer(max_age_ns=10_000_000, max_spread_ns=5_000_000)
    for idx, mid in enumerate(shadow.CAN_ORDER):
        row = rows[mid]
        raw_q = (target_q[idx]-row["offset_candidate_rad"])/row["sign_candidate"]
        for name, val, unit in (("position", raw_q, "rad"),
                                ("velocity", target_dq[idx]/row["sign_candidate"], "rad_s")):
            buffer.ingest_motor(can_type=17, motor_id=mid, parameter=name, value=val, unit=unit,
                                request_ns=tick-3_000_000, received_ns=tick-1_000_000)
    buffer.ingest_imu(accel_m_s2=[0., 0., -10.], gyro_rad_s=[.11, .22, .33],
                      read_started_ns=tick-2_000_000, read_finished_ns=tick-1_500_000)
    return buffer.snapshot(tick).as_dict()


def reviewed_branch_snapshot(tick=1_000_000_000, *, current_raw=6.262798309326172,
                             reference_raw=-.04319906234741211, calib=None):
    """Synthetic read-only tick with caller-attested quiet power-epoch evidence."""
    s = snapshot(tick, calib=calib)
    position = {str(row["motor_id"]): row["value"] for row in s["motors"]
                if row["parameter"] == "position"}
    position["3"] = current_raw
    next(row for row in s["motors"] if row["motor_id"] == 3
         and row["parameter"] == "position")["value"] = current_raw
    reference = dict(position)
    reference["3"] = reference_raw
    uids = (calib or calibration())["identities"]
    def capture(epoch, digit, raw):
        return {"boot_id": "synthetic-jetson-boot", "motor_power_epoch": epoch,
                "uid_read_boot_id": "synthetic-jetson-boot",
                "uid_read_motor_power_epoch": epoch,
                "capture_sha256": digit*64, "uid_capture_sha256": digit*63+"f",
                "uids_by_id": dict(uids), "raw_rad_by_id": raw,
                "disabled_zero_current_by_id": {str(i): True for i in range(1, 13)},
                "motor_output_allowed": False}
    old_capture = capture("observed-motor-epoch-1", "a", reference)
    new_capture = capture("observed-motor-epoch-2", "b", position)
    no_turn = {"motor_supply_off_on_observed": True,
               "reference_capture_sha256": old_capture["capture_sha256"],
               "current_capture_sha256": new_capture["capture_sha256"],
               "evidence_sha256": "e"*64,
               "no_full_physical_turn_by_id": {"3": True},
               "physical_pose_observation_by_id": {"3": "marked shaft did not make a full turn"}}
    review = StaticBranchComparison(old_capture, new_capture, no_turn)
    bound = review.validated_current_binding()
    s["source_flags"] = {"power_epoch_branch_capture": {
        **bound, "uid_read_boot_id": bound["boot_id"],
        "uid_read_motor_power_epoch": bound["motor_power_epoch"],
        "motor_supply_off_on_observed": True,
        "disabled_zero_current_by_id": {str(i): True for i in range(1, 13)}}}
    return s, review


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
    def __init__(self, bad=None):
        self.resets, self.calls, self.bad = 0, [], bad
        self.previous = [99.]*12  # A caller warmup could have left arbitrary state.
    def reset(self, ids):
        assert ids.tolist() == [0]
        self.resets += 1
        self.previous = [0.]*12
    def __call__(self, *tensors):
        values = [v.tolist()[0] for v in tensors]
        self.calls.append(values)
        gyro, gravity, command, q, dq, h = values
        observation = ([x*.25 for x in gyro]+gravity+command+q+[x*.05 for x in dq]
                       +self.previous+h+[0.]*9+[0., 1., 0.]+[0., 0., 0., 0., 1.])
        raw = [.1*len(self.calls)]*12
        self.last_observation = Tensor([observation])
        self.last_actor_output = Tensor([raw])
        self.previous = raw
        result = Tensor([[0., .4, -.8]*4])
        if self.bad == "actor_nan":
            self.last_actor_output.values[0][0] = math.nan
        elif self.bad == "observation_batch":
            self.last_observation = Tensor([[0.]*37, [0.]*37])
        elif self.bad == "target_shape":
            result = Tensor([[0.]*11])
        elif self.bad == "target_bounds":
            result.values[0][0] = 2.
        elif self.bad == "interrupt":
            raise KeyboardInterrupt()
        return result


def make(policy=None, **options):
    return observer.StatefulPolicyObserver(policy or Policy(), options.pop("calibration", calibration()),
        imu_mount_candidate=options.pop("imu_mount_candidate", mount()),
        h_hypothesis=options.pop("h_hypothesis", 0), command=options.pop("command", [0., 0., 0.]),
        max_ticks=options.pop("max_ticks", 2), max_age_ns=options.pop("max_age_ns", 10_000_000),
        max_spread_ns=options.pop("max_spread_ns", 5_000_000), torch_module=FakeTorch, **options)


class ObserverTests(unittest.TestCase):
    def test_static_digest_is_prepared_once_but_live_snapshot_digest_is_always_current(self):
        with patch.object(observer, "_digest", wraps=observer._digest) as digest:
            o = make()
            calibration_calls = [call for call in digest.call_args_list
                                 if call.args[0] is o._calibration]
            self.assertEqual(len(calibration_calls), 1)
            expected_calibration_digest = observer._digest(calibration())
            digest.reset_mock()
            o.reset_run(1_000_000_000, warmup_completed=True)
            hashes = []
            for index in range(2):
                source = snapshot(1_000_000_000+index*observer.DT_NS)
                source["source_flags"] = {"sequence": index, "nested": ["live"]}
                source["imu"]["gyro_rad_s"][0] += index*.01
                result = o.consume(source)
                self.assertEqual(result["provenance"]["calibration_canonical_json_sha256"],
                                 expected_calibration_digest)
                hashes.append(result["provenance"]["snapshot_canonical_json_sha256"])
                self.assertEqual(digest.call_count, index+1)
                self.assertEqual(digest.call_args.args[0], source)
                self.assertIsNot(digest.call_args.args[0], source)
            self.assertNotEqual(*hashes)
            self.assertNotEqual(o._policy.calls[0][0], o._policy.calls[1][0])

    def test_constructor_inputs_and_returned_provenance_cannot_mutate_fixed_configuration(self):
        cal, imu_mount, bias, command = calibration(), mount(), bias_candidate(), [.12, 0., 0.]
        cal["source_notes"] = {"nested": ["original", {"unverified": True}]}
        original_cal = copy.deepcopy(cal)
        baseline = make(calibration=cal, imu_mount_candidate=imu_mount,
                        gyro_bias_candidate=bias, command=command)
        actual = make(calibration=cal, imu_mount_candidate=imu_mount,
                      gyro_bias_candidate=bias, command=command)
        cal["candidates"][0]["offset_candidate_rad"] = 123
        cal["source_notes"]["nested"][1]["unverified"] = False
        imu_mount["R_body_from_sensor"][0][0] = 9
        imu_mount["provenance"]["source"] = "changed"
        bias["gyro_bias_candidate_rad_s"][0] = 10
        bias["provenance"]["a"]["events_sha256"] = "e"*64
        command[0] = 0
        for run in (actual, baseline): run.reset_run(1_000_000_000, warmup_completed=True)
        source = snapshot(calib=original_cal)
        source["source_flags"] = {"nested": ["upstream"]}
        first = actual.consume(source)
        self.assertEqual(first, baseline.consume(source))
        original_first = copy.deepcopy(first)
        first["provenance"]["calibration_source_flags"]["source_notes"]["nested"].clear()
        first["provenance"]["imu_mount_candidate"]["R_body_from_sensor"][0][0] = 5
        first["provenance"]["gyro_bias_hypothesis"]["bias_sensor_rad_s"][0] = 5
        first["provenance"]["gyro_bias_hypothesis"]["source"]["a"].clear()
        first["provenance"]["model_can_order_candidate"].clear()
        first["provenance"]["snapshot_source_flags"]["nested"].clear()
        first["inputs"]["command"][0] = 999
        first["inputs"]["h_hypothesis12"][0] = 999
        source["imu"]["gyro_rad_s"][0] = 999
        source["source_flags"]["nested"].append("mutated later")
        second_source = snapshot(1_020_000_000, calib=original_cal)
        second = actual.consume(second_source)
        self.assertEqual(second, baseline.consume(second_source))
        # Returned records own their provenance, also across later ticks.
        second["provenance"]["calibration_source_flags"]["source_notes"]["nested"].clear()
        self.assertEqual(original_first["provenance"]["calibration_source_flags"]["source_notes"],
                         {"nested": ["original", {"unverified": True}]})
        self.assertEqual(actual._calibration, original_cal)

    def test_saved_json_sequence_matches_frozen_preoptimization_records_for_both_hypotheses(self):
        # Full-record digests from the unchanged observer source SHA256
        # f180436674257cc5e8783f28a8760e23401e165aa1c058d2e50690defa9a6be4.
        # This includes all dynamic inputs, evolving actor history, output flags
        # and complete static/live provenance, not only the policy target.
        golden = {0: "65652dddfc2182787a3189c2f5317a2b71e84a1d884023d10db7c0dead5bc4cc",
                  1: "883b630dbc09fb023fc6a411c0b37a42ff213d75b82421736ab71f5ac3ccd79c"}
        cal = calibration()
        cal["source_notes"] = {"nested": ["captured fixture", {"unverified": True}]}
        for h in (0, 1):
            with self.subTest(h=h):
                run = make(calibration=cal, gyro_bias_candidate=bias_candidate(),
                           h_hypothesis=h, command=[.12, 0., 0.], max_ticks=3)
                run.reset_run(1_000_000_000, warmup_completed=True)
                output = []
                for index in range(3):
                    source = snapshot(1_000_000_000+index*observer.DT_NS, calib=cal)
                    source["source_flags"] = {"input_record": index,
                        "nested": [False, {"schema": "synthetic saved JSON"}]}
                    source["imu"]["gyro_rad_s"][0] += .01*index
                    next(row for row in source["motors"] if row["motor_id"] == 2
                         and row["parameter"] == "position")["value"] += .01*index
                    output.append(run.consume(json.loads(json.dumps(source))))
                encoded = json.dumps(output, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
                self.assertEqual(hashlib.sha256(encoded).hexdigest(), golden[h])
                self.assertEqual(run.finish()["status"], "COMPLETE_NO_OUTPUT_DIAGNOSTIC")

    def test_profiling_is_opt_in_and_default_does_not_read_the_clock(self):
        clock = Mock(side_effect=AssertionError("clock must not be read"))
        o = make(monotonic_ns=clock)
        o.reset_run(1_000_000_000, warmup_completed=True)
        result = o.consume(snapshot())
        self.assertNotIn("consume_profile", result)
        self.assertNotIn("last_consume_profile", o.summary())
        clock.assert_not_called()
        for value in (1, None, "yes"):
            with self.assertRaisesRegex(observer.ObserverError, "boolean"):
                make(profile_consume=value)
        with self.assertRaisesRegex(observer.ObserverError, "callable clock"):
            make(profile_consume=True, monotonic_ns=123)

    def test_section_profile_preserves_two_tick_results_state_and_input_immutability(self):
        ticks = iter(range(100, 1000, 10))
        profiled = make(profile_consume=True, monotonic_ns=lambda: next(ticks))
        baseline = make()
        for o in (profiled, baseline):
            o.reset_run(1_000_000_000, warmup_completed=True)
        stages = ["snapshot_copy", "source_validation", "input_conversion",
                  "provenance_serialization", "tensor_conversion", "model_call",
                  "output_conversion_validation", "result_build"]
        for tick in (1_000_000_000, 1_020_000_000):
            source = snapshot(tick)
            original = copy.deepcopy(source)
            actual, expected = profiled.consume(source), baseline.consume(source)
            timing = actual.pop("consume_profile")
            self.assertEqual(actual, expected)
            self.assertEqual(source, original)
            self.assertEqual(list(timing["durations_ns"]), stages)
            self.assertEqual(timing["measured_total_ns"], sum(timing["durations_ns"].values()))
            self.assertEqual(timing["measured_total_ns"], 80)
            self.assertTrue(timing["complete"])
            self.assertIsNone(timing["failed_stage"])
            self.assertFalse(timing["wall_clock_timing_verified"])
            self.assertFalse(timing["output_allowed"])
            self.assertIn("caller_json_serialization_and_io", timing["excludes"])
            self.assertIn("device_acquisition", timing["excludes"])
            self.assertIn("motor_command_transmission", timing["excludes"])
        self.assertEqual(profiled.ticks_completed, baseline.ticks_completed)

    def test_profile_is_independent_of_returned_records_and_cleared_by_reset(self):
        clock = iter(range(100, 1000))
        o = make(profile_consume=True, monotonic_ns=lambda: next(clock))
        o.reset_run(1_000_000_000, warmup_completed=True)
        result = o.consume(snapshot())
        result["consume_profile"]["durations_ns"]["model_call"] = 999
        summary = o.summary()
        self.assertEqual(summary["last_consume_profile"]["durations_ns"]["model_call"], 1)
        summary["last_consume_profile"]["durations_ns"].clear()
        self.assertEqual(len(o.summary()["last_consume_profile"]["durations_ns"]), 8)
        o.invalidate("caller cancelled")
        o.prepare_run(warmup_completed=True)
        self.assertIsNone(o.summary()["last_consume_profile"])

    def test_failed_validation_and_model_call_retain_partial_profile_and_do_not_accept_tick(self):
        for bad in ("missing_input", "interrupt"):
            with self.subTest(bad=bad):
                clock = iter(range(100, 1000))
                p = Policy(bad="interrupt" if bad == "interrupt" else None)
                o = make(p, profile_consume=True, monotonic_ns=lambda: next(clock))
                o.reset_run(1_000_000_000, warmup_completed=True)
                s = snapshot()
                if bad == "missing_input":
                    s["imu"] = None
                with self.assertRaises(KeyboardInterrupt if bad == "interrupt" else observer.ObserverError):
                    o.consume(s)
                timing = o.summary()["last_consume_profile"]
                self.assertFalse(timing["complete"])
                self.assertEqual(timing["failed_stage"],
                                 "model_call" if bad == "interrupt" else "source_validation")
                self.assertNotIn("result_build", timing["durations_ns"])
                self.assertEqual(o.ticks_completed, 0)
                self.assertEqual(o.status, "INCOMPLETE")
                self.assertEqual(len(p.calls), 1 if bad == "interrupt" else 0)

    def test_invalid_profile_clock_never_accepts_a_tick_or_masks_model_failure(self):
        for samples in ([100, 90], [100, -1], [100, True], [100, 1.5],
                        [100, 101, 102, 103, 104, 105, 106, 107, 99]):
            with self.subTest(samples=samples):
                clock = iter(samples)
                o = make(profile_consume=True, monotonic_ns=lambda: next(clock))
                o.reset_run(1_000_000_000, warmup_completed=True)
                with self.assertRaises(observer.ObserverError):
                    o.consume(snapshot())
                self.assertEqual(o.ticks_completed, 0)
                self.assertEqual(o.status, "INCOMPLETE")
                self.assertFalse(o.summary()["last_consume_profile"]["complete"])
        # The model raises KeyboardInterrupt, then timing the failed section
        # raises StopIteration: the original interruption must still escape.
        clock = iter(range(6))
        o = make(Policy(bad="interrupt"), profile_consume=True, monotonic_ns=lambda: next(clock))
        o.reset_run(1_000_000_000, warmup_completed=True)
        with self.assertRaises(KeyboardInterrupt):
            o.consume(snapshot())
        self.assertIn("StopIteration", o.summary()["last_consume_profile"]["measurement_error"])
        self.assertIn("KeyboardInterrupt", o.failure)

    def test_prepare_then_arm_resets_once_and_cannot_consume_or_rearm_early(self):
        p, o = Policy(), None
        o = make(p)
        with self.assertRaisesRegex(observer.ObserverError, "Prepare"):
            o.arm_run(1_000_000_000)
        o.prepare_run(warmup_completed=True)
        self.assertEqual(o.status, "PREPARED")
        self.assertIsNone(o._next_tick_ns)
        with self.assertRaises(observer.ObserverError): o.consume(snapshot())
        with self.assertRaises(observer.ObserverError): o.prepare_run(warmup_completed=True)
        o.arm_run(1_000_000_000)
        with self.assertRaises(observer.ObserverError): o.arm_run(1_020_000_000)
        result = o.consume(snapshot())
        self.assertEqual(p.resets, 1)
        self.assertEqual(o.reset_count, 1)
        self.assertEqual(result['tick_ns'], 1_000_000_000)
        self.assertEqual(result['observation74'][33:45], [0.]*12)

    def test_continuous_previous_actor_state_and_one_reset_after_warmup(self):
        p = Policy()
        o = make(p)
        o.reset_run(1_000_000_000, warmup_completed=True)
        first = o.consume(snapshot())
        second = o.consume(snapshot(1_020_000_000))
        self.assertEqual(p.resets, 1)
        self.assertEqual(first["observation74"][33:45], [0.]*12)
        self.assertEqual(second["observation74"][33:45], [.1]*12)
        self.assertEqual(o.finish()["status"], "COMPLETE_NO_OUTPUT_DIAGNOSTIC")
        self.assertFalse(o.summary()["live_50hz_verified"])
        self.assertFalse(second["output_allowed"])
        self.assertFalse(second["approved_for_runtime"])
        self.assertFalse(second["fresh_identity_match_verified"])

    def test_requires_explicit_start_and_caller_warmup(self):
        o = make()
        with self.assertRaises(observer.ObserverError):
            o.consume(snapshot())
        with self.assertRaises(observer.ObserverError):
            o.reset_run(1_000_000_000, warmup_completed=False)
        self.assertEqual(o.reset_count, 0)

    def test_sign_offset_order_no_wrap_and_bias_before_rotation(self):
        p = Policy()
        o = make(p, gyro_bias_candidate=bias_candidate())
        s = snapshot()
        before = copy.deepcopy(s)
        o.reset_run(s["tick_ns"], warmup_completed=True)
        result = o.consume(s)
        self.assertEqual(s, before)
        gyro, gravity, _, q, dq, h = p.calls[0]
        for actual, expected in zip(gyro, [.2, .1, -.3]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(gravity, [0., 0., -1.])
        for actual, expected in zip(q, [0., .4, -.8]*4):
            self.assertAlmostEqual(actual, expected)
        for idx, actual in enumerate(dq):
            self.assertAlmostEqual(actual, .02*(idx+1))
        self.assertEqual(h, [0.]*12)
        self.assertEqual(result["provenance"]["raw_accel_norm_m_s2"], 10.)
        self.assertFalse(result["provenance"]["accel_scale_corrected"])
        self.assertTrue(result["provenance"]["gyro_bias_hypothesis"]["applied_as_hypothesis"])

    def test_reviewed_power_epoch_branch_changes_only_observation_input(self):
        source, review = reviewed_branch_snapshot()
        unchanged = copy.deepcopy(source)
        baseline = make(max_ticks=1)
        baseline.reset_run(source["tick_ns"], warmup_completed=True)
        with self.assertRaisesRegex(observer.ObserverError, "joint range"):
            baseline.consume(source)
        policy = Policy()
        run = make(policy, max_ticks=1, power_epoch_branch_comparison=review)
        run.reset_run(source["tick_ns"], warmup_completed=True)
        result = run.consume(source)
        self.assertEqual(source, unchanged)
        q_index = shadow.CAN_ORDER.index(3)
        equivalent_raw = 6.262798309326172-TWO_PI
        self.assertAlmostEqual(policy.calls[0][3][q_index], -equivalent_raw+.03)
        self.assertAlmostEqual(result["inputs"]["q_model_rad"][q_index], -equivalent_raw+.03)
        self.assertTrue(shadow.LOWER[q_index] <= result["inputs"]["q_model_rad"][q_index]
                        <= shadow.UPPER[q_index])
        branch = result["provenance"]["power_epoch_branch_overlay"]
        self.assertEqual(branch["raw_position_rad_by_id"]["3"], 6.262798309326172)
        self.assertAlmostEqual(branch["comparison_position_rad_by_id"]["3"], equivalent_raw)
        self.assertEqual(branch["reviewed_capture_binding"]["reviewed_branch_turns_by_id"],
                         {"3": 1})
        self.assertEqual(branch["reviewed_capture_binding"]["motor_power_epoch"],
                         "observed-motor-epoch-2")
        self.assertFalse(result["output_allowed"])
        self.assertFalse(result["motor_output_available"])
        self.assertFalse(result["approved_for_runtime"])
        self.assertFalse(branch["motor_output_allowed"])
        self.assertEqual(result["q_target_rad_diagnostic_only"], [0., .4, -.8]*4)

    def test_power_epoch_branch_profile_and_returned_provenance_are_independent(self):
        source, review = reviewed_branch_snapshot()
        run = make(power_epoch_branch_comparison=review)
        run.reset_run(source["tick_ns"], warmup_completed=True)
        first = run.consume(source)
        first_branch = first["provenance"]["power_epoch_branch_overlay"]
        first_branch["reviewed_capture_binding"]["uids_by_id"]["3"] = "f"*16
        review.validated_current_binding()["raw_rad_by_id"]["3"] = 123.
        later, _ = reviewed_branch_snapshot(source["tick_ns"]+observer.DT_NS)
        second = run.consume(later)
        self.assertEqual(second["provenance"]["power_epoch_branch_overlay"]
                         ["reviewed_capture_binding"]["uids_by_id"]["3"],
                         calibration()["identities"]["3"])
        self.assertAlmostEqual(second["provenance"]["power_epoch_branch_overlay"]
                               ["comparison_position_rad_by_id"]["3"],
                               6.262798309326172-TWO_PI)

    def test_power_epoch_branch_rejects_missing_or_mismatched_capture_proof(self):
        changes = {
            "missing": lambda s: s["source_flags"].clear(),
            "boot": lambda s: s["source_flags"]["power_epoch_branch_capture"].update(
                boot_id="other-boot"),
            "epoch": lambda s: s["source_flags"]["power_epoch_branch_capture"].update(
                motor_power_epoch="other-epoch"),
            "capture": lambda s: s["source_flags"]["power_epoch_branch_capture"].update(
                capture_sha256="c"*64),
            "uid_capture": lambda s: s["source_flags"]["power_epoch_branch_capture"].update(
                uid_capture_sha256="c"*64),
            "evidence": lambda s: s["source_flags"]["power_epoch_branch_capture"].update(
                evidence_sha256="c"*64),
            "uid": lambda s: s["source_flags"]["power_epoch_branch_capture"]
                ["uids_by_id"].update({"3": "f"*16}),
            "stale_uid_read": lambda s: s["source_flags"]["power_epoch_branch_capture"].update(
                uid_read_motor_power_epoch="observed-motor-epoch-1"),
            "not_quiet": lambda s: s["source_flags"]["power_epoch_branch_capture"]
                ["disabled_zero_current_by_id"].update({"3": False}),
            "no_off_on": lambda s: s["source_flags"]["power_epoch_branch_capture"].update(
                motor_supply_off_on_observed=False),
        }
        for label, change in changes.items():
            with self.subTest(label=label):
                source, review = reviewed_branch_snapshot()
                change(source)
                policy = Policy()
                run = make(policy, max_ticks=1, power_epoch_branch_comparison=review)
                run.reset_run(source["tick_ns"], warmup_completed=True)
                with self.assertRaises(observer.ObserverError):
                    run.consume(source)
                self.assertEqual(policy.calls, [])
                self.assertEqual(run.ticks_completed, 0)
                self.assertEqual(run.status, "INCOMPLETE")
        source, review = reviewed_branch_snapshot()
        wrong_calibration = calibration()
        wrong_calibration["identities"]["3"] = "f"*16
        with self.assertRaisesRegex(observer.ObserverError, "UIDs do not match"):
            make(calibration=wrong_calibration, power_epoch_branch_comparison=review)
        with self.assertRaisesRegex(observer.ObserverError, "StaticBranchComparison"):
            make(power_epoch_branch_comparison=review.comparison())

    def test_power_epoch_branch_rejects_raw_discontinuity_and_retains_range_gate(self):
        source, review = reviewed_branch_snapshot()
        policy = Policy()
        run = make(policy, power_epoch_branch_comparison=review)
        run.reset_run(source["tick_ns"], warmup_completed=True)
        run.consume(source)
        later, _ = reviewed_branch_snapshot(source["tick_ns"]+observer.DT_NS)
        next(row for row in later["motors"] if row["motor_id"] == 3
             and row["parameter"] == "position")["value"] += TWO_PI
        with self.assertRaisesRegex(observer.ObserverError, "discontinuity|static capture"):
            run.consume(later)
        self.assertEqual(len(policy.calls), 1)
        self.assertEqual(run.ticks_completed, 1)
        self.assertEqual(run.status, "INCOMPLETE")

        current_raw, reference_raw = TWO_PI-.46, -.46
        source, review = reviewed_branch_snapshot(current_raw=current_raw,
                                                   reference_raw=reference_raw)
        policy = Policy()
        run = make(policy, power_epoch_branch_comparison=review)
        run.reset_run(source["tick_ns"], warmup_completed=True)
        self.assertAlmostEqual(run.consume(source)["inputs"]["q_model_rad"]
                               [shadow.CAN_ORDER.index(3)], .49)
        later, _ = reviewed_branch_snapshot(source["tick_ns"]+observer.DT_NS,
                                             current_raw=current_raw,
                                             reference_raw=reference_raw)
        next(row for row in later["motors"] if row["motor_id"] == 3
             and row["parameter"] == "position")["value"] -= .03
        with self.assertRaisesRegex(observer.ObserverError, "joint range"):
            run.consume(later)
        self.assertEqual(len(policy.calls), 1)

    def test_h_instances_are_separate_and_not_measurements(self):
        a, b = make(h_hypothesis=0), make(h_hypothesis=1)
        for o in (a, b):
            o.reset_run(1_000_000_000, warmup_completed=True)
        ar, br = a.consume(snapshot()), b.consume(snapshot())
        self.assertEqual(ar["observation74"][45:57], [0.]*12)
        self.assertEqual(br["observation74"][45:57], [1.]*12)
        self.assertFalse(br["h_measured"])
        with self.assertRaises(observer.ObserverError):
            make(a._policy, h_hypothesis=1)

    def test_source_order_and_equal_receive_times_are_allowed(self):
        s = snapshot()
        s["motors"].reverse()
        o = make()
        o.reset_run(s["tick_ns"], warmup_completed=True)
        o.consume(s)
        self.assertEqual(o.ticks_completed, 1)

    def test_missing_nan_bounds_future_and_false_ready_all_block_before_forward(self):
        def stale(s):
            for row in s["motors"]:
                row["request_ns"] -= 50_000_000
                row["age_upper_bound_ns"] += 50_000_000
            s["oldest_observation_age_ns"] += 50_000_000
            s["acquisition_spread_ns"] += 50_000_000
        modifications = [
            lambda s: s.update(status="BLOCKED", blocked_reasons=["missing"]),
            lambda s: s.update(imu=None),
            lambda s: s["motors"].pop(),
            lambda s: s["motors"][0].update(value=math.nan),
            lambda s: s["imu"]["gyro_rad_s"].__setitem__(1, math.inf),
            lambda s: s["motors"][0].update(value=100.),
            lambda s: s["motors"][0].update(received_ns=s["tick_ns"]+1),
            lambda s: s["imu"].update(read_finished_ns=s["tick_ns"]+1),
            lambda s: s.update(output_allowed=True),
            lambda s: s.update(max_age_ns=500_000_000),
            lambda s: s["motors"][0].update(unit="degrees"), stale]
        for modify in modifications:
            with self.subTest(modification=modify):
                s, p = snapshot(), Policy()
                modify(s)
                o = make(p)
                o.reset_run(1_000_000_000, warmup_completed=True)
                with self.assertRaises(observer.ObserverError):
                    o.consume(s)
                self.assertEqual(len(p.calls), 0)
                self.assertEqual(o.status, "INCOMPLETE")
                with self.assertRaises(observer.ObserverError):
                    o.consume(snapshot())

    def test_tick_gap_or_repeat_requires_explicit_reset_not_catchup(self):
        for tick in (1_000_000_000, 1_040_000_000):
            p = Policy()
            o = make(p)
            o.reset_run(1_000_000_000, warmup_completed=True)
            o.consume(snapshot())
            with self.assertRaisesRegex(observer.ObserverError, "20ms tick"):
                o.consume(snapshot(tick))
            self.assertEqual(len(p.calls), 1)
            self.assertEqual(p.resets, 1)
            with self.assertRaises(observer.ObserverError):
                o.consume(snapshot(1_020_000_000))
            o.reset_run(2_000_000_000, warmup_completed=True)
            self.assertEqual(p.resets, 2)
            fresh = o.consume(snapshot(2_000_000_000))
            self.assertEqual(fresh["observation74"][33:45], [0.]*12)

    def test_shape_nonfinite_out_of_range_and_interrupt_invalidate_after_one_forward(self):
        for bad in ("actor_nan", "observation_batch", "target_shape", "target_bounds", "interrupt"):
            p = Policy(bad)
            o = make(p)
            o.reset_run(1_000_000_000, warmup_completed=True)
            with self.subTest(bad=bad), self.assertRaises((ValueError, KeyboardInterrupt)):
                o.consume(snapshot())
            self.assertEqual(len(p.calls), 1)
            self.assertEqual(o.ticks_completed, 0)
            self.assertEqual(o.status, "INCOMPLETE")
            self.assertTrue(o.finish()["incomplete"])

    def test_float32_endpoints_pass_but_one_float32_step_outside_fails(self):
        f32 = lambda value: struct.unpack("<f", struct.pack("<f", value))[0]
        for values in (shadow.LOWER, shadow.UPPER):
            class EndpointPolicy(Policy):
                def __call__(self, *args):
                    super().__call__(*args)
                    return Tensor([[f32(v) for v in values]])
            p = EndpointPolicy()
            o = make(p)
            o.reset_run(1_000_000_000, warmup_completed=True)
            self.assertEqual(o.consume(snapshot())["q_target_rad_diagnostic_only"], [f32(v) for v in values])
        for endpoint in (shadow.LOWER[2], shadow.UPPER[1]):
            packed = struct.unpack("<I", struct.pack("<f", endpoint))[0]
            outside = struct.unpack("<f", struct.pack("<I", packed+1))[0]
            class OutsidePolicy(Policy):
                def __call__(self, *args):
                    result = super().__call__(*args)
                    result.values[0][2 if endpoint < 0 else 1] = outside
                    return result
            o = make(OutsidePolicy())
            o.reset_run(1_000_000_000, warmup_completed=True)
            with self.assertRaisesRegex(observer.ObserverError, "target outside"):
                o.consume(snapshot())

    def test_hold_is_explicitly_aged_but_retrograde_or_changed_same_time_is_rejected(self):
        for mode in ("hold", "motor_changed", "imu_changed", "retrograde"):
            p = Policy()
            o = make(p, max_age_ns=50_000_000)
            first = snapshot(buffer=TelemetrySnapshotBuffer(max_age_ns=50_000_000,
                                                            max_spread_ns=5_000_000))
            o.reset_run(first["tick_ns"], warmup_completed=True)
            o.consume(first)
            held = copy.deepcopy(first)
            held["tick_ns"] += observer.DT_NS
            for row in held["motors"]:
                row["age_upper_bound_ns"] += observer.DT_NS
            held["imu"]["age_upper_bound_ns"] += observer.DT_NS
            held["oldest_observation_age_ns"] += observer.DT_NS
            if mode == "motor_changed":
                held["motors"][0]["value"] += .01
            elif mode == "imu_changed":
                held["imu"]["gyro_rad_s"][0] += .01
            elif mode == "retrograde":
                # All intervals shift backwards, leaving a self-consistent
                # single-snapshot timing summary that only history detects.
                for row in held["motors"]:
                    row["request_ns"] -= 1_000
                    row["received_ns"] -= 1_000
                    row["age_upper_bound_ns"] += 1_000
                held["imu"]["read_started_ns"] -= 1_000
                held["imu"]["read_finished_ns"] -= 1_000
                held["imu"]["age_upper_bound_ns"] += 1_000
                held["oldest_observation_age_ns"] += 1_000
            if mode == "hold":
                result = o.consume(held)
                self.assertEqual(result["provenance"]["oldest_observation_age_ns"], 23_000_000)
            else:
                with self.subTest(mode=mode), self.assertRaises(observer.ObserverError):
                    o.consume(held)
                self.assertEqual(len(p.calls), 1)

    def test_early_finish_and_tick_budget_do_not_claim_completion(self):
        o = make()
        o.reset_run(1_000_000_000, warmup_completed=True)
        o.consume(snapshot())
        self.assertTrue(o.finish()["incomplete"])
        p = make(max_ticks=1)
        p.reset_run(1_000_000_000, warmup_completed=True)
        p.consume(snapshot())
        with self.assertRaises(observer.ObserverError):
            p.consume(snapshot(1_020_000_000))
        self.assertTrue(p.finish()["incomplete"])

    def test_profiles_are_explicit_finite_and_not_runtime_approval(self):
        for value in (None, True, .5, math.nan):
            with self.subTest(h=value), self.assertRaises(observer.ObserverError):
                make(h_hypothesis=value)
        for count in (0, True, 30_001):
            with self.assertRaises(observer.ObserverError):
                make(max_ticks=count)
        for command in ([.1, .1, 0.], [.1, 0., .1], [math.nan, 0., 0.]):
            with self.assertRaises(observer.ObserverError):
                make(command=command)
        s = snapshot()
        s["source_flags"] = {"some_upstream_claim": True}
        o = make()
        o.reset_run(1_000_000_000, warmup_completed=True)
        result = o.consume(s)
        self.assertEqual(result["provenance"]["snapshot_source_flags"], s["source_flags"])
        self.assertFalse(result["output_allowed"])

    def test_mount_file_provenance_and_bad_rotation(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/"mount.json"
            p.write_text(json.dumps(mount()))
            o = make(imu_mount_candidate=p)
            self.assertEqual(o._mount["source"]["sha256"], shadow.sha(p))
        with self.assertRaises(ValueError):
            make(imu_mount_candidate=mount([[1, 0, 0], [0, 1, 0], [0, 0, -1]]))

    def test_bias_must_be_A_fit_sensor_candidate_not_an_approval(self):
        for change in (lambda c: c.update(approved_for_runtime=True),
                       lambda c: c.update(mount_rotation_applied=True),
                       lambda c: c.update(gyro_bias_candidate_rad_s=[.2, .3, .4]),
                       lambda c: c["provenance"].pop("a")):
            c = bias_candidate()
            change(c)
            with self.assertRaises(observer.ObserverError):
                make(gyro_bias_candidate=c)

    def test_active_run_cannot_reset_away_history(self):
        o = make()
        o.reset_run(1_000_000_000, warmup_completed=True)
        with self.assertRaises(observer.ObserverError):
            o.reset_run(1_020_000_000, warmup_completed=True)
        self.assertEqual(o.reset_count, 1)

    def test_module_has_no_device_or_process_interfaces(self):
        tree = ast.parse(inspect.getsource(observer))
        forbidden = {"serial", "socket", "subprocess", "can_readonly", "can_pipeline_probe",
                     "imu", "control", "actuation", "os"}
        imports = {alias.name for n in ast.walk(tree) if isinstance(n, ast.Import) for alias in n.names}
        imports |= {n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        self.assertFalse(imports & forbidden)


if __name__ == "__main__":
    unittest.main()
