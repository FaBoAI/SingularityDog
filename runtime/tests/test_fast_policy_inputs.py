"""Safety and ownership contracts for saved-input CPU preparation; no hardware."""
import copy
from dataclasses import FrozenInstanceError, replace
import hashlib
import json
import math
import struct
import unittest

from singularitydog_hw import fast_policy_inputs as fast
from singularitydog_hw.fast_policy_replay import benchmark_capture
from singularitydog_hw import policy_observer as observer
from singularitydog_hw import policy_shadow as shadow
from test_policy_observer import make, calibration, Policy

TICK = 1_000_000_000


def reply(mid):
    data = struct.pack(">4H", 32767+mid, 32767-mid, 32767, 300)
    can_id = (2 << 24) | (mid << 8) | 0xFD
    wire = b"AT"+((can_id << 3) | 4).to_bytes(4, "big")+b"\x08"+data+b"\r\n"
    p, v, torque, temp = struct.unpack(">4H", data)
    scale = lambda value, limit: value*(2.*limit)/65535.-limit
    result = dict.fromkeys(fast.UNVERIFIED_FLAGS, False)
    result.update(ok=True, motor_id=mid, parameter="stop_feedback", mode_state=0, fault_bits=0,
        position_u16=p, velocity_u16=v, torque_u16=torque, temperature_u16=temp,
        position_rad_candidate=scale(p, 12.57), velocity_rad_s_candidate=scale(v, 50.),
        torque_nm_candidate=scale(torque, 5.5), temperature_c=temp/10.,
        position_velocity_in_one_reply=True,
        raw_frame={"can_id": can_id, "type": 2, "source_id": mid, "destination_id": 0xFD,
                   "flags": 4, "data_hex": data.hex(), "wire_hex": wire.hex()})
    return dict(kind="pipeline_reply", sequence=mid+6, cycle=1, motor_id=mid,
        parameter="stop_feedback", ok=True, write_expected_bytes=17, write_returned_bytes=17,
        write_call_entered=True, write_started_monotonic_ns=TICK-20_000_000+mid,
        write_finished_monotonic_ns=TICK-19_000_000+mid,
        received_monotonic_ns=TICK-5_000_000+mid, deadline_monotonic_ns=TICK+200_000_000,
        result=result)


def imu():
    return dict(kind="imu", frame="sensor", accel_m_s2=[0., 0., -9.80665],
        gyro_rad_s=[.1, .2, .3], read_started_monotonic_ns=TICK-3_000_000,
        read_finished_monotonic_ns=TICK-2_000_000, available_monotonic_ns=TICK-1_000_000)


class PreparedInputTests(unittest.TestCase):
    def setUp(self):
        self.rows, self.imu = [reply(i) for i in range(1, 13)], imu()

    def test_original_values_timestamps_and_evidence_are_preserved(self):
        prepared = fast.prepare_cycle(list(reversed(self.rows)), self.imu)
        snapshot = prepared.snapshot(TICK)
        for row in self.rows:
            position, velocity = snapshot["motors"][(row["motor_id"]-1)*2:row["motor_id"]*2]
            self.assertEqual(position["value"], row["result"]["position_rad_candidate"])
            self.assertEqual(velocity["value"], row["result"]["velocity_rad_s_candidate"])
            self.assertEqual(position["request_ns"], row["write_started_monotonic_ns"])
            self.assertEqual(velocity["received_ns"], row["received_monotonic_ns"])
        digest = lambda x: hashlib.sha256(json.dumps(x, sort_keys=True,
            separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        flags = snapshot["source_flags"]
        self.assertEqual(flags["source_reply_rows_canonical_json_sha256"], digest(self.rows))
        self.assertEqual(flags["source_imu_event_canonical_json_sha256"], digest(self.imu))
        self.assertEqual(flags["raw_type2_frames_canonical_json_sha256"],
                         digest([r["result"]["raw_frame"] for r in self.rows]))
        self.assertFalse(snapshot["output_allowed"])
        self.assertFalse(flags["source_timestamps_changed"])
        self.assertFalse(flags["full_pipeline_20ms_verified"])

    def test_prepared_source_and_returned_snapshots_have_separate_ownership(self):
        original = copy.deepcopy((self.rows, self.imu))
        prepared = fast.prepare_cycle(self.rows, self.imu)
        expected = prepared.snapshot(TICK)
        altered = prepared.snapshot(TICK)
        self.assertEqual((self.rows, self.imu), original)
        self.rows[0]["result"]["raw_frame"]["wire_hex"] = "broken"
        self.imu["gyro_rad_s"][0] = 123
        altered["motors"][0]["value"] = 999
        altered["imu"]["accel_m_s2"].clear()
        altered["source_flags"]["position_range_rad_candidate"].clear()
        self.assertEqual(prepared.snapshot(TICK), expected)
        with self.assertRaises(FrozenInstanceError):
            prepared.motors[0].position = 3

    def test_each_new_source_is_prepared_and_hashed_independently(self):
        a = fast.prepare_cycle(self.rows, self.imu)
        self.imu["gyro_rad_s"][0] += .01
        b = fast.prepare_cycle(self.rows, self.imu)
        self.assertNotEqual(a.evidence_hashes[-1], b.evidence_hashes[-1])
        self.assertNotEqual(a.snapshot(TICK)["imu"], b.snapshot(TICK)["imu"])

    def test_materialization_rechecks_age_and_never_retimes_old_sources(self):
        prepared = fast.prepare_cycle(self.rows, self.imu)
        later = prepared.snapshot(TICK+20_000_000)
        self.assertEqual(later["motors"][0]["request_ns"], self.rows[0]["write_started_monotonic_ns"])
        self.assertEqual(later["motors"][0]["age_upper_bound_ns"], 39_999_999)
        with self.assertRaisesRegex(ValueError, "Stale"):
            prepared.snapshot(TICK+100_000_000)
        for bad_tick in (True, -1, 1., math.nan, 2**63):
            with self.subTest(tick=bad_tick), self.assertRaises(ValueError):
                prepared.snapshot(bad_tick)

    def test_materialization_keeps_id_fault_and_range_checks(self):
        prepared = fast.prepare_cycle(self.rows, self.imu)
        for change in ({"motor_id": 2}, {"can_id": prepared.motors[0].can_id | (1 << 16)},
                       {"position": 13.}, {"velocity": math.inf}, {"flags": 0}):
            # Even a deliberately constructed replacement record cannot bypass
            # the dynamic checks at materialization.
            bad = replace(prepared, motors=(replace(prepared.motors[0], **change), *prepared.motors[1:]))
            with self.subTest(change=change), self.assertRaises(ValueError):
                bad.snapshot(TICK)

    def test_missing_duplicate_wrong_cycle_and_partial_write_rejected(self):
        variants = [self.rows[:-1], self.rows+[reply(1)], self.rows[:-1]+[reply(1)]]
        for key, value in (("motor_id", True), ("cycle", 2), ("ok", False),
                           ("write_returned_bytes", 16), ("write_call_entered", False)):
            rows = copy.deepcopy(self.rows)
            rows[0][key] = value
            variants.append(rows)
        for rows in variants:
            with self.subTest(count=len(rows)), self.assertRaises(ValueError):
                fast.prepare_cycle(rows, self.imu)

    def test_all_used_decoded_fields_must_match_raw(self):
        changes = (("motor_id", 2), ("mode_state", 1), ("fault_bits", 1),
                   ("position_u16", True), ("position_rad_candidate", math.nan),
                   ("velocity_rad_s_candidate", 0.), ("torque_nm_candidate", 0.),
                   ("temperature_c", 0.), ("velocity_scale_verified", True))
        for key, value in changes:
            rows = copy.deepcopy(self.rows)
            rows[0]["result"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                fast.prepare_cycle(rows, self.imu)

    def test_raw_fault_mode_source_host_version_and_malformed_wire_rejected(self):
        for mode in ("fault", "mode", "source", "host", "version", "malformed"):
            rows = copy.deepcopy(self.rows)
            raw = rows[0]["result"]["raw_frame"]
            wire = bytearray.fromhex(raw["wire_hex"])
            cid = raw["can_id"]
            if mode == "fault": cid |= 1 << 16
            elif mode == "mode": cid |= 1 << 22
            elif mode == "source": cid = (cid & ~(255 << 8)) | (2 << 8)
            elif mode == "host": cid = (cid & ~255) | 254
            elif mode == "version": wire[7:10] = b"\x00\xc4\x56"
            else: wire[-1] = 0
            wire[2:6] = ((cid << 3) | 4).to_bytes(4, "big")
            raw.update(can_id=cid, source_id=(cid >> 8) & 255, destination_id=cid & 255,
                       wire_hex=wire.hex(), data_hex=wire[7:15].hex())
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                fast.prepare_cycle(rows, self.imu)

    def test_noncausal_late_and_unavailable_sources_rejected(self):
        for key, value in (("received_monotonic_ns", True),
                           ("write_started_monotonic_ns", TICK-1),
                           ("deadline_monotonic_ns", TICK-6_000_000),
                           ("available_monotonic_ns", TICK+1)):
            rows = copy.deepcopy(self.rows)
            rows[0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                fast.prepare_cycle(rows, self.imu).snapshot(TICK)

    def test_invalid_imu_and_excessive_spread_rejected(self):
        for key, value in (("frame", "body"), ("gyro_rad_s", [0., math.inf, 0.]),
                           ("accel_m_s2", [0., 0.]), ("available_monotonic_ns", TICK+1),
                           ("read_started_monotonic_ns", TICK-200_000_000)):
            sample = copy.deepcopy(self.imu)
            sample[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                fast.prepare_cycle(self.rows, sample).snapshot(TICK)

    def test_replay_reports_full_preparation_cost_and_reuses_original_tick(self):
        expected = fast.prepare_cycle(self.rows, self.imu).snapshot(TICK)
        ticks = []
        def baseline(rows, imu_sample, tick, cycle):
            ticks.append(tick)
            return copy.deepcopy(expected)
        report = benchmark_capture((self.rows, self.imu, TICK, expected), baseline,
                                   repeats=4, warmup=0)
        self.assertEqual(set(ticks), {TICK})
        self.assertTrue(report["snapshots_exactly_equal"])
        self.assertEqual(set(report["timings"]), {"baseline_assembly", "prepare_immutable_cycle",
                         "prepared_assembly_only", "prepare_and_assemble"})
        self.assertTrue(all(t["wall"]["samples"] == 4 for t in report["timings"].values()))

    def test_replay_fails_if_saved_result_does_not_match(self):
        expected = fast.prepare_cycle(self.rows, self.imu).snapshot(TICK)
        bad = copy.deepcopy(expected)
        bad["motors"][0]["value"] = 999
        with self.assertRaisesRegex(ValueError, "differ"):
            benchmark_capture((self.rows, self.imu, TICK, bad), lambda *args, **kwargs: expected,
                              repeats=1, warmup=0)

    def observer(self, **kwargs):
        cal = calibration()
        target_by_id = dict(zip(shadow.CAN_ORDER, [0., .4, -.8]*4))
        for row in cal["candidates"]:
            raw = self.rows[row["motor_id"]-1]["result"]["position_rad_candidate"]
            row["offset_candidate_rad"] = target_by_id[row["motor_id"]]-row["sign_candidate"]*raw
        run = make(calibration=cal, max_age_ns=fast.MAX_AGE_NS,
                   max_spread_ns=fast.MAX_SPREAD_NS, **kwargs)
        run.reset_run(TICK, warmup_completed=True)
        return run

    def test_existing_observer_consumes_prepared_snapshot_without_guard_changes(self):
        prepared = fast.prepare_cycle(self.rows, self.imu)
        run = self.observer()
        result = run.consume(prepared.snapshot(TICK))
        self.assertFalse(result["output_allowed"])
        for actual, expected in zip(result["inputs"]["q_model_rad"], [0., .4, -.8]*4):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(result["provenance"]["snapshot_source_flags"],
                         prepared.snapshot(TICK)["source_flags"])

    def test_existing_calibrated_input_range_check_still_blocks(self):
        snapshot = fast.prepare_cycle(self.rows, self.imu).snapshot(TICK)
        next(r for r in snapshot["motors"] if r["parameter"] == "position")["value"] = 100
        with self.assertRaisesRegex(observer.ObserverError, "registered joint range"):
            self.observer().consume(snapshot)

    def test_existing_target_range_check_still_blocks(self):
        snapshot = fast.prepare_cycle(self.rows, self.imu).snapshot(TICK)
        with self.assertRaisesRegex(observer.ObserverError, "target outside"):
            self.observer(policy=Policy(bad="target_bounds")).consume(snapshot)

    def test_changed_value_with_repeated_source_times_still_blocks(self):
        run = self.observer()
        run.consume(fast.prepare_cycle(self.rows, self.imu).snapshot(TICK))
        self.imu["gyro_rad_s"][0] += .01
        altered = fast.prepare_cycle(self.rows, self.imu).snapshot(TICK+20_000_000)
        with self.assertRaisesRegex(observer.ObserverError, "Held source timestamps changed"):
            run.consume(altered)


if __name__ == "__main__":
    unittest.main()
