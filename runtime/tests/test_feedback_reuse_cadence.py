"""Pure timestamp/protocol fault injection; no devices or motor eligibility."""
import copy
from dataclasses import FrozenInstanceError, replace
import struct
import unittest

from singularitydog_hw import feedback_reuse_cadence as cadence


def wire(can_id, data):
    return b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + data + b"\r\n"


def context(**changes):
    values = dict(boot_id="synthetic-boot", power_epoch="synthetic-power",
        source_sha256="a" * 64, profile_sha256="b" * 64,
        identity_evidence_sha256="c" * 64,
        expected_uids=tuple((i, f"{i:016x}") for i in cadence.IDS), transport_generation=1)
    values.update(changes)
    return cadence.FeedbackContext(**values)


def original_records(*, first=1_010_000_000, gap=900_000, mode=2, kind=1):
    rows = {}
    for bus, ids in cadence.BUSES.items():
        rows[bus] = []
        for slot, mid in enumerate(ids):
            start = first + slot * gap
            cid = kind << 24 | (32767 if kind == 1 else cadence.HOST_ID) << 8 | mid
            tx = wire(cid, struct.pack(">4H", 32768, 32767, 0, 0) if kind == 1 else bytes(8))
            rx = wire(2 << 24 | mode << 22 | mid << 8 | cadence.HOST_ID,
                      struct.pack(">4H", 32768, 32768, 32768, 330))
            rows[bus].append(dict(tx_hex=tx.hex(), rx_hex=rx.hex(), start_ns=start,
                finish_ns=start + 10_000, received_ns=start + 2_000_000,
                deadline_ns=start + 20_000_000, written=17, received=17))
    return rows


def snapshot(*, records=None, ctx=None, generation=1):
    records = records or original_records()
    return cadence.freeze_type2_snapshot(records, context=ctx or context(),
        generation=generation,
        published_ns=max(r["received_ns"] for v in records.values() for r in v))


def select(snap=None, **changes):
    ctx = context()
    values = dict(context=ctx, release_ns=1_020_000_000, now_ns=1_021_000_000,
        imu=cadence.IMUInterval(1_020_100_000, 1_020_900_000, 2,
                               (0., 0., -9.81), (0., 0., 0.)),
        work_to_last_write_budget_ns=8_000_000, last_consumed_generation=0,
        previous_imu_sequence=1, bootstrap_complete=True, opt_in=True)
    values.update(changes)
    return cadence.select_feedback_acquisition(snap, **values)


class FeedbackReuseTests(unittest.TestCase):
    def test_reuse_is_explicit_immutable_and_never_hardware_authorization(self):
        snap = snapshot()
        result = select(snap)
        self.assertEqual(result.mode, "REUSE_14")
        self.assertEqual(result.requests_per_cycle, 14)
        self.assertFalse(result.output_allowed)
        self.assertEqual(result.oldest_input_request_ns, 1_010_000_000)
        self.assertEqual(result.final_write_deadline_ns, 1_030_000_000)
        self.assertEqual(select(snap, opt_in=False).mode, "REFRESH_26")
        with self.assertRaises(FrozenInstanceError):
            snap.samples[0].request_ns = 1_021_000_000

    def test_snapshot_copies_raw_records_and_rows_keep_original_timestamps(self):
        records = original_records()
        snap = snapshot(records=records)
        original = snap.samples[0].tx_wire
        records["front"][0]["tx_hex"] = "00"
        records["front"][0]["start_ns"] = 1
        self.assertEqual(snap.samples[0].tx_wire, original)
        rows = cadence.feedback_rows(snap)
        sample, request, received = rows[1, "feedback"]
        self.assertEqual(request, sample.request_ns)
        self.assertEqual(received, sample.received_ns)
        self.assertNotEqual(request, received)
        self.assertEqual(sample.protocol_position_rad, sample.position_rad)

    def test_rows_are_compatible_with_existing_calibrated_feedback_validator(self):
        from singularitydog_hw.policy_output_runtime import feedback_sample
        snap = snapshot()
        rows = cadence.feedback_rows(snap)
        profile = {"max_sample_age_ms": 20., "axes": {
            str(mid): {"sign": -1 if mid % 2 else 1, "max_measured_velocity_rad_s": 1.}
            for mid in cadence.IDS}}
        offsets = {mid: .1 for mid in cadence.IDS}
        sample = feedback_sample(rows, profile, offsets, now_ns=1_021_000_000)
        for mid in cadence.IDS:
            sign = profile["axes"][str(mid)]["sign"]
            self.assertEqual(sample.q_model_rad[mid - 1], sign * snap.samples[mid - 1].position_rad + .1)
            self.assertEqual(sample.velocity_rad_s[mid - 1], sign * snap.samples[mid - 1].velocity_rad_s)

    def test_budget_includes_every_final_write_and_cannot_use_received_as_source(self):
        snap = snapshot()
        result = select(snap, work_to_last_write_budget_ns=10_000_000)
        self.assertEqual(result.mode, "REFRESH_26")
        self.assertIn("original_input_to_final_write_budget_exceeded", result.reasons)
        # A wrong receive-based bound would admit 21 + 10 <= 12 + 20 ms.
        self.assertLessEqual(result.projected_final_write_ns,
                             min(row.received_ns for row in snap.samples) + 20_000_000)
        self.assertGreater(result.projected_final_write_ns,
                           min(row.request_ns for row in snap.samples) + 20_000_000)

    def test_equality_allowed_but_actual_one_nanosecond_miss_retained(self):
        result = select(snapshot(), work_to_last_write_budget_ns=9_000_000)
        self.assertEqual(result.mode, "REUSE_14")
        deadline = cadence.recheck_before_dispatch(result, now_ns=1_024_000_000,
            remaining_write_budget_ns=6_000_000, context=context(), generation=1)
        self.assertEqual(deadline, result.final_write_deadline_ns)
        age = cadence.validate_final_write(result, final_host_write_ns=deadline,
                                          context=context(), generation=1)
        self.assertEqual(age, 20_000_000)
        with self.assertRaises(TimeoutError):
            cadence.validate_final_write(result, final_host_write_ns=deadline + 1,
                                         context=context(), generation=1)
        with self.assertRaises(TimeoutError):
            cadence.recheck_before_dispatch(result, now_ns=1_024_000_001,
                remaining_write_budget_ns=6_000_000, context=context(), generation=1)

    def test_current_release_deadline_not_extended_by_newer_feedback(self):
        snap = snapshot(records=original_records(first=1_020_000_000))
        result = select(snap, now_ns=1_028_000_000,
                        work_to_last_write_budget_ns=13_000_000)
        self.assertEqual(result.mode, "REFRESH_26")
        self.assertEqual(result.final_write_deadline_ns, 1_040_000_000)

    def test_no_bootstrap_partial_snapshot_or_generation_replay(self):
        self.assertEqual(select(None).mode, "REFRESH_26")
        self.assertEqual(select(snapshot(), bootstrap_complete=False).mode, "REFRESH_26")
        self.assertEqual(select(snapshot(), last_consumed_generation=1).mode, "BLOCK")
        result = select(snapshot(generation=3))
        self.assertEqual(result.mode, "REFRESH_26")
        self.assertIn("feedback_generation_gap_requires_refresh", result.reasons)
        rows = original_records()
        rows["rear"].pop()
        with self.assertRaises(ValueError):
            snapshot(records=rows)

    def test_context_boot_power_source_profile_uid_descriptor_reset_blocks(self):
        changes = [dict(boot_id="new"), dict(power_epoch="new"),
            dict(source_sha256="d" * 64), dict(profile_sha256="d" * 64),
            dict(identity_evidence_sha256="d" * 64), dict(transport_generation=2),
            dict(expected_uids=tuple((i, f"{i+100:016x}") for i in cadence.IDS))]
        for change in changes:
            with self.subTest(change=change):
                self.assertEqual(select(snapshot(), context=context(**change)).mode, "BLOCK")

    def test_fault_and_mode_cannot_be_skipped_by_refresh(self):
        for fault, mode in ((1, 2), (0, 0), (63, 2), (0, 3)):
            rows = original_records(mode=mode)
            rx = bytes.fromhex(rows["front"][0]["rx_hex"])
            cid = int.from_bytes(rx[2:6], "big") >> 3 | fault << 16
            rows["front"][0]["rx_hex"] = wire(cid, rx[7:15]).hex()
            with self.subTest(fault=fault, mode=mode):
                self.assertEqual(select(snapshot(records=rows)).mode, "BLOCK")
        result = select(snapshot(records=original_records(mode=0, kind=4)), required_mode=0)
        self.assertEqual(result.mode, "REUSE_14")

    def test_original_deadline_noncausal_duplicates_crossbus_and_id_mismatch(self):
        mutations = [lambda r: r["front"][0].update(received=0),
            lambda r: r["front"][0].update(finish_ns=2_000_000_000),
            lambda r: r["front"][0].update(deadline_ns=r["front"][0]["received_ns"]),
            lambda r: r["front"].__setitem__(1, copy.deepcopy(r["front"][0])),
            lambda r: (r["front"].__setitem__(0, copy.deepcopy(r["rear"][0]))),
            lambda r: r["front"][0].update(rx_hex=r["front"][1]["rx_hex"]),
            lambda r: r["front"][0].update(tx_hex="00")]
        for mutation in mutations:
            rows = original_records()
            mutation(rows)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                snapshot(records=rows)

    def test_version_bytes_never_accepted_as_feedback(self):
        rows = original_records()
        rx = bytes.fromhex(rows["front"][0]["rx_hex"])
        rows["front"][0]["rx_hex"] = wire(int.from_bytes(rx[2:6], "big") >> 3,
            b"\x00\xc4\x56" + bytes(5)).hex()
        with self.assertRaises(ValueError):
            snapshot(records=rows)

    def test_bad_imu_stale_future_repeated_and_nonfinite_rejected(self):
        self.assertEqual(select(snapshot(), previous_imu_sequence=2).mode, "BLOCK")
        stale = cadence.IMUInterval(1_000_000_000, 1_000_900_000, 2,
                                   (0., 0., -9.81), (0., 0., 0.))
        self.assertEqual(select(snapshot(), imu=stale).mode, "BLOCK")
        with self.assertRaises(ValueError):
            select(snapshot(), now_ns=1_020_000_000)
        with self.assertRaises(ValueError):
            cadence.IMUInterval(1, 2, 1, (float("nan"), 0., 1.), (0., 0., 0.))

    def test_spread_and_delay_need_fresh_feedback(self):
        result = select(snapshot(), max_acquisition_spread_ns=10_000_000)
        self.assertEqual(result.mode, "REFRESH_26")
        self.assertEqual(result.reasons, ("original_input_interval_spread_exceeded",))
        self.assertEqual(select(snapshot(), now_ns=1_030_000_001).mode, "REFRESH_26")

    def test_invalid_boolean_or_relaxed_20ms_budget_rejected(self):
        for values in (dict(work_to_last_write_budget_ns=20_000_001),
                       dict(max_acquisition_spread_ns=20_000_001),
                       dict(work_to_last_write_budget_ns=True), dict(opt_in=1),
                       dict(bootstrap_complete=1), dict(required_mode=True)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                select(snapshot(), **values)
        result = select(snapshot())
        with self.assertRaises(ValueError):
            cadence.validate_final_write(result, final_host_write_ns=1_029_000_000,
                                         context=context(), generation=True)

    def test_policy_consumption_generation_is_distinct_from_output_validation(self):
        first = snapshot(generation=1)
        self.assertEqual(select(first, last_consumed_generation=0).mode, "REUSE_14")
        self.assertEqual(select(first, last_consumed_generation=1).mode, "BLOCK")
        later = snapshot(generation=2)
        self.assertEqual(select(later, last_consumed_generation=1).mode, "REUSE_14")

    def test_fixed_phase_14_request_50hz_does_not_establish_steady_reuse(self):
        self.assertEqual(select(snapshot()).mode, "REUSE_14")
        # The first late->early reuse can fit. Its new commands occur earlier;
        # at the next 20 ms release those original requests are already too old
        # to finish another six-write 0.9 ms bus train within their 20 ms age.
        next_snapshot = snapshot(records=original_records(first=1_024_000_000), generation=2)
        next_imu = cadence.IMUInterval(1_040_100_000, 1_040_900_000, 3,
                                      (0., 0., -9.81), (0., 0., 0.))
        next_result = select(next_snapshot, release_ns=1_040_000_000,
            now_ns=1_041_000_000, imu=next_imu, last_consumed_generation=1,
            previous_imu_sequence=2)
        self.assertEqual(next_result.mode, "REFRESH_26")
        self.assertEqual(next_result.reasons, ("original_input_to_final_write_budget_exceeded",))

    def test_decision_deadline_cannot_be_constructed_or_replaced_as_later(self):
        result = select(snapshot())
        for change in (dict(final_write_deadline_ns=result.final_write_deadline_ns + 1),
                       dict(oldest_input_request_ns=result.oldest_input_request_ns + 1),
                       dict(input_source_deadline_ns=result.input_source_deadline_ns + 1),
                       dict(projected_final_write_ns=result.final_write_deadline_ns + 1)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                replace(result, **change)

    def test_stop_mode_and_received_count_type_cannot_be_forged(self):
        bad_count = original_records()
        bad_count["front"][0]["received"] = 17.0
        for rows in (original_records(mode=2, kind=4), bad_count):
            with self.assertRaises(ValueError):
                snapshot(records=rows)


if __name__ == "__main__":
    unittest.main()
