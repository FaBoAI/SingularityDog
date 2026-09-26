import dataclasses
import math
import struct
import unittest

from singularitydog_hw.active_report_receiver import ReceiverError, Sample, StreamReceiver
from singularitydog_hw.can_readonly import ATParser


MS = 1_000_000


def feedback(mid=1, kind=2, *, raw=(0, 32768, 65535, 250), mode=0,
             fault=0, destination=0xFD, flags=4, payload=None):
    data = struct.pack(">4H", *raw) if payload is None else payload
    can_id = kind << 24 | mode << 22 | fault << 16 | mid << 8 | destination
    return (b"AT" + ((can_id << 3) | flags).to_bytes(4, "big") +
            bytes([len(data)]) + data + b"\r\n")


class ActiveReportReceiverTests(unittest.TestCase):
    def test_constructor_requires_exact_selected_ids_and_type(self):
        for ids in ((), (0,), (13,), (True,), (1.0,), ("1",), (2, 1),
                    (1, 1), [1], {1}, None):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                StreamReceiver(ids, 2)
        for kind in (True, False, 2.0, "2", 0, 21, 25):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                StreamReceiver((1,), kind)
        for capacity in (0, -1, True, 1.5, None):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                StreamReceiver((1,), 2, capacity)
        self.assertEqual(StreamReceiver((1, 12), 24).ids, (1, 12))

    def test_sample_is_frozen_and_preserves_wire_raw_values(self):
        wire = feedback()
        receiver = StreamReceiver((1,), 2)
        sample, = receiver.feed(wire, 10, 30)
        self.assertIsInstance(sample, Sample)
        self.assertEqual(sample.raw_u16, (0, 32768, 65535, 250))
        self.assertEqual((sample.motor_id, sample.kind, sample.mode_state,
                          sample.fault_bits, sample.host_sequence), (1, 2, 0, 0, 1))
        self.assertEqual(sample.wire, ATParser().feed(wire)[0].wire)
        self.assertEqual((sample.first_seen_ns, sample.received_ns), (10, 30))
        with self.assertRaises(dataclasses.FrozenInstanceError):
            sample.received_ns = 99
        self.assertFalse(hasattr(sample, "velocity_rad_s"))

    def test_all_split_points_preserve_first_contributing_read_start(self):
        wire = feedback(raw=(0x4154, 0x0D0A, 65535, 0))
        for cut in range(len(wire) + 1):
            with self.subTest(cut=cut):
                receiver = StreamReceiver((1,), 2)
                got = receiver.feed(wire[:cut], 0, 10)
                got += receiver.feed(wire[cut:], 20, 30)
                self.assertEqual(len(got), 1)
                self.assertEqual(got[0].wire, wire)
                self.assertEqual(got[0].first_seen_ns, 0 if cut else 20)
                self.assertEqual(got[0].received_ns, 10 if cut == len(wire) else 30)
                self.assertEqual(receiver.parser.discarded_bytes, 0)
                self.assertEqual(receiver.parser.buffer, b"")

    def test_byte_fragments_and_empty_reads_do_not_reset_partial_start(self):
        receiver = StreamReceiver((1,), 2)
        wire = feedback()
        got = []
        for i, byte in enumerate(wire):
            got += receiver.feed(bytes([byte]), i * 4, i * 4 + 1)
            self.assertEqual(receiver.feed(b"", i * 4 + 2, i * 4 + 3), [])
        self.assertEqual(len(got), 1)
        self.assertEqual((got[0].first_seen_ns, got[0].received_ns), (0, 65))

    def test_partial_then_coalesced_frames_get_independent_start_times(self):
        receiver = StreamReceiver((1, 2), 2)
        one, two = feedback(1), feedback(2)
        receiver.feed(one[:8], 1, 2)
        got = receiver.feed(one[8:] + two + one[:3], 3, 4)
        self.assertEqual([(s.motor_id, s.first_seen_ns, s.received_ns) for s in got],
                         [(1, 1, 4), (2, 3, 4)])
        last, = receiver.feed(one[3:], 5, 6)
        self.assertEqual((last.first_seen_ns, last.received_ns), (3, 6))

    def test_identical_values_and_same_timestamp_are_distinct_observations(self):
        receiver = StreamReceiver((1, 2), 2)
        got = receiver.feed(feedback(1) * 3 + feedback(2) * 2, MS, 2 * MS)
        got += receiver.feed(feedback(1), 2 * MS, 2 * MS)
        self.assertEqual([s.host_sequence for s in got], [1, 2, 3, 1, 2, 4])
        self.assertEqual(len(receiver.samples), 6)
        summary = receiver.summary(0, 3 * MS)
        self.assertTrue(summary["ok"])
        self.assertEqual(summary["ids"]["1"]["interarrival_ms"]["max"], 0)

    def test_discovery_is_per_id_but_cannot_change_kind(self):
        receiver = StreamReceiver((1, 2), None)
        receiver.feed(feedback(1, 2) + feedback(2, 24), 0, 1)
        self.assertEqual(receiver.detected_kinds, {1: 2, 2: 24})
        with self.assertRaisesRegex(ReceiverError, "per_id_frame_type_changed"):
            receiver.feed(feedback(1, 24), 2, 3)
        self.assertEqual(receiver.detected_kinds, {1: 2, 2: 24})
        self.assertEqual(len(receiver.samples), 2)
        self.assertFalse(receiver.summary(0, 4)["ok"])

    def test_explicit_kind_does_not_silently_discover_another(self):
        for expected, observed in ((2, 24), (24, 2)):
            receiver = StreamReceiver((1,), expected)
            with self.subTest(expected=expected), self.assertRaisesRegex(
                    ReceiverError, "expected_frame_type_mismatch"):
                receiver.feed(feedback(kind=observed), 0, 1)
            self.assertEqual(receiver.detected_kinds, {})

    def test_metadata_rejections_preserve_rejected_wire_and_poison(self):
        cases = [
            (feedback(mid=2), "unknown_motor_id"),
            (feedback(mid=0), "unknown_motor_id"),
            (feedback(destination=0xFE), "unexpected_destination"),
            (feedback(kind=21), "unexpected_frame_type"),
            (feedback(kind=17), "unexpected_frame_type"),
            (feedback(flags=0), "noncanonical_flags_or_dlc"),
            (feedback(payload=bytes(7)), "noncanonical_flags_or_dlc"),
            (feedback(payload=b"\x00\xc4\x56" + bytes(5)),
             "version_reply_is_not_active_feedback"),
        ]
        cases += [(feedback(kind=kind, mode=mode, fault=fault), "nonzero_mode_or_fault")
                  for kind in (2, 24) for mode, fault in ((1, 0), (2, 0), (3, 0),
                                                        (0, 1), (0, 32), (0, 63))]
        for wire, reason in cases:
            with self.subTest(reason=reason, wire=wire.hex()):
                receiver = StreamReceiver((1,), None)
                with self.assertRaisesRegex(ReceiverError, reason):
                    receiver.feed(wire, 0, 1)
                self.assertEqual(receiver.samples, [])
                self.assertTrue(receiver.poisoned)
                self.assertEqual(receiver.rejected_wire_hex, wire.hex())
                with self.assertRaisesRegex(ReceiverError, "already_poisoned"):
                    receiver.feed(feedback(), 2, 3)

    def test_malformed_framing_fails_closed_and_retains_accepted_prefix(self):
        wire = feedback()
        cases = (b"noise", b"AT\x00\x00\x00\x04\xff", wire[:-2] + b"xx")
        for malformed in cases:
            with self.subTest(malformed=malformed):
                receiver = StreamReceiver((1,), 2)
                with self.assertRaisesRegex(ReceiverError, "malformed_or_discarded_bytes"):
                    receiver.feed(wire + malformed + wire, 0, 1)
                self.assertEqual(len(receiver.samples), 1)
                self.assertGreater(receiver.parser.discarded_bytes, 0)
                result = receiver.summary(0, 2)
                self.assertFalse(result["ok"])
                self.assertLess(len(receiver.parser.buffer), 17)
                self.assertEqual(result["received_bytes"], 2 * len(wire) + len(malformed))

    def test_record_capacity_is_not_a_ring_buffer_or_silent_drop(self):
        receiver = StreamReceiver((1,), 2, max_frames=2)
        wire = feedback()
        with self.assertRaisesRegex(ReceiverError, "frame_capacity_exceeded"):
            receiver.feed(wire * 4, 0, 1)
        self.assertEqual([s.host_sequence for s in receiver.samples], [1, 2])
        self.assertEqual(receiver.unprocessed_bytes, len(wire))
        self.assertEqual(receiver.rejected_wire_hex, wire.hex())
        self.assertFalse(receiver.summary(0, 2)["ok"])

    def test_time_inversion_and_overlapping_reads_poison_even_empty_feed(self):
        for start, end in ((9, 11), (11, 10), (0, 1), (True, 12),
                           (11, math.nan), (11.0, 12), (-1, 12)):
            with self.subTest(start=start, end=end):
                receiver = StreamReceiver((1,), 2)
                receiver.feed(feedback(), 5, 10)
                with self.assertRaises(ReceiverError):
                    receiver.feed(b"", start, end)
                self.assertTrue(receiver.poisoned)
                self.assertEqual(len(receiver.samples), 1)
        receiver = StreamReceiver((1,), 2)
        with self.assertRaisesRegex(ReceiverError, "chunk must be bytes"):
            receiver.feed(bytearray(feedback()), 0, 1)

    def test_interarrival_statistics_and_window_edges(self):
        receiver = StreamReceiver((1,), 2)
        for t in (1, 2, 5, 10):
            receiver.feed(feedback(), t * MS, t * MS)
        result = receiver.summary(0, 12 * MS)
        self.assertTrue(result["ok"])
        channel = result["ids"]["1"]
        self.assertEqual(channel["frame_count"], 4)
        self.assertEqual(channel["initial_gap_ms"], 1)
        self.assertEqual(channel["tail_gap_ms"], 2)
        self.assertEqual(channel["max_silence_ms"], 5)
        self.assertEqual(channel["interarrival_ms"]["p50"], 3)
        self.assertAlmostEqual(channel["interarrival_ms"]["p95"], 4.8)
        self.assertAlmostEqual(channel["interarrival_ms"]["p99"], 4.96)
        self.assertEqual(channel["interarrival_ms"]["max"], 5)

    def test_idle_tail_and_late_first_channel_fail_despite_short_interarrival(self):
        receiver = StreamReceiver((1,), 2)
        receiver.feed(feedback(), 0, 0)
        receiver.feed(feedback(), 10 * MS, 10 * MS)
        receiver.feed(b"", 11 * MS, 50 * MS)
        result = receiver.summary(0, 50 * MS)
        self.assertFalse(result["ok"])
        self.assertEqual(result["ids"]["1"]["max_silence_ms"], 40)
        receiver = StreamReceiver((1, 2), 2)
        receiver.feed(feedback(1), MS, MS)
        receiver.feed(feedback(2) * 2, 25 * MS, 25 * MS)
        result = receiver.summary(0, 30 * MS)
        self.assertFalse(result["ok"])
        self.assertIn("host_silence_limit_exceeded:2", result["errors"])
        self.assertEqual(result["ids"]["2"]["initial_gap_ms"], 25)

    def test_missing_and_empty_channels_fail_even_in_short_window(self):
        receiver = StreamReceiver((1, 2), 2)
        self.assertFalse(receiver.summary(0, MS)["ok"])
        receiver.feed(feedback(1), 0, 0)
        result = receiver.summary(0, MS)
        self.assertFalse(result["ok"])
        self.assertIn("missing_motor_id:2", result["errors"])
        missing = result["ids"]["2"]
        self.assertEqual(missing["frame_count"], 0)
        self.assertEqual(missing["max_silence_ms"], 1)
        self.assertEqual(missing["interarrival_ms"], dict.fromkeys(("p50", "p95", "p99", "max")))

    def test_silence_limit_includes_equality_but_rejects_one_nanosecond_late(self):
        receiver = StreamReceiver((1,), 24)
        receiver.feed(feedback(kind=24), 0, 0)
        self.assertTrue(receiver.summary(0, 20 * MS)["ok"])
        self.assertFalse(receiver.summary(0, 20 * MS + 1)["ok"])

    def test_residual_partial_frame_is_visible_and_invalidates_its_window(self):
        receiver = StreamReceiver((1,), 2)
        wire = feedback()
        receiver.feed(wire + wire[:4], 0, MS)
        result = receiver.summary(0, 2 * MS)
        self.assertFalse(result["ok"])
        self.assertIn("partial_frame_at_end", result["errors"])
        self.assertEqual(result["residual_hex"], wire[:4].hex())
        self.assertTrue(result["residual_in_window"])
        self.assertEqual(result["residual_first_seen_ns"], 0)

    def test_window_excludes_warmup_crossing_start_and_post_window_samples(self):
        receiver = StreamReceiver((1,), 2)
        wire = feedback()
        receiver.feed(wire + wire[:4], MS, 2 * MS)
        receiver.feed(wire[4:], 10 * MS, 11 * MS)
        receiver.feed(wire, 12 * MS, 13 * MS)
        receiver.feed(wire, 15 * MS, 16 * MS)
        result = receiver.summary(10 * MS, 14 * MS)
        self.assertTrue(result["ok"])
        self.assertEqual(result["samples_received"], 1)
        self.assertEqual(result["total_samples_received"], 4)
        self.assertEqual(result["excluded_before_window"], 2)
        self.assertEqual(result["excluded_crossing_start"], 1)
        self.assertEqual(result["excluded_after_window"], 1)
        self.assertEqual(result["excluded_crossing_end"], 0)
        self.assertEqual(result["ids"]["1"]["initial_gap_ms"], 3)
        self.assertEqual(result["ids"]["1"]["tail_gap_ms"], 1)
        self.assertEqual(len(receiver.samples), 4)

    def test_frame_completed_by_drain_is_reported_at_window_end(self):
        receiver = StreamReceiver((1,), 2)
        wire = feedback()
        receiver.feed(wire + wire[:5], MS, 2 * MS)
        receiver.feed(wire[5:], 11 * MS, 12 * MS)
        result = receiver.summary(0, 10 * MS)
        self.assertTrue(result["ok"])
        self.assertEqual(result["samples_received"], 1)
        self.assertEqual(result["excluded_after_window"], 1)
        self.assertEqual(result["excluded_crossing_end"], 1)
        self.assertEqual(result["residual_bytes"], 0)

    def test_partial_frame_started_after_window_does_not_invalidate_it(self):
        receiver = StreamReceiver((1,), 2)
        receiver.feed(feedback(), MS, 2 * MS)
        receiver.feed(feedback()[:5], 11 * MS, 12 * MS)
        result = receiver.summary(0, 10 * MS)
        self.assertTrue(result["ok"])
        self.assertFalse(result["residual_in_window"])
        self.assertEqual(result["residual_bytes"], 5)

    def test_summary_never_promotes_host_observation_to_control_proof(self):
        receiver = StreamReceiver((1,), 2)
        receiver.feed(feedback() * 3, 0, 0)
        result = receiver.summary(0, MS)
        self.assertTrue(result["ok"])
        self.assertTrue(result["host_observation_only"])
        for name in ("physical_packet_loss_proven", "motor_sample_time_proven",
                     "control_20ms_proven", "scaled_feedback_verified"):
            self.assertIs(result[name], False)

    def test_invalid_summary_arguments_do_not_corrupt_receiver(self):
        receiver = StreamReceiver((1,), 2)
        for start, end, gap in ((0, 0, 20), (1, 0, 20), (-1, 1, 20),
                                (0, True, 20), (0, 1, 0), (0, 1, -1),
                                (0, 1, math.nan), (0, 1, math.inf), (0, 1, True)):
            with self.subTest(start=start, end=end, gap=gap), self.assertRaises(ValueError):
                receiver.summary(start, end, gap)
        self.assertFalse(receiver.poisoned)


class DeactivationPrefixTests(unittest.TestCase):
    def activated(self):
        receiver = StreamReceiver((1,), 24, activation_type2_prefix=True)
        receiver.expect_activation(1, 0, 5)
        receiver.feed(feedback(), 0, 1)
        return receiver

    def test_split_off_prefix_with_coalesced_type24_tails(self):
        off, tail = feedback(kind=2), feedback(kind=24)
        for cut in range(len(off) + 1):
            with self.subTest(cut=cut):
                receiver = self.activated()
                receiver.expect_deactivation(1, 10, 30)
                got = receiver.feed(tail + off[:cut], 10, 15)
                got += receiver.feed(off[cut:] + tail, 20, 25)
                self.assertEqual([s.host_sequence for s in got], [1, 2])
                self.assertEqual([s.kind for s in got], [24, 24])
                self.assertEqual(receiver.deactivation_pending, {})
                record = receiver.deactivation_feedback[1]
                self.assertEqual((record.kind, record.host_sequence), (2, 0))
                self.assertEqual(record.wire, off)
                self.assertEqual(record.first_seen_ns, 10 if cut else 20)
                self.assertEqual(record.received_ns, 15 if cut == len(off) else 25)
                self.assertEqual(len(receiver.samples), 2)
                result = receiver.summary(0, 30)
                self.assertTrue(result["ok"])
                self.assertEqual(result["deactivation_feedback"]["1"]["wire_hex"], off.hex())
                self.assertFalse(result["deactivation_stop_ack_proven"])
                self.assertFalse(result["deactivation_reporting_off_proven"])

    def test_duplicate_or_unarmed_off_type2_fails(self):
        receiver = self.activated()
        with self.assertRaisesRegex(ReceiverError, "activation_type2_prefix_duplicate"):
            receiver.feed(feedback(), 10, 11)
        receiver = self.activated()
        receiver.expect_deactivation(1, 10, 20)
        with self.assertRaisesRegex(ReceiverError, "deactivation_type2_prefix_duplicate"):
            receiver.feed(feedback() * 2, 10, 11)
        self.assertEqual(receiver.samples, [])
        self.assertEqual(len(receiver.deactivation_feedback), 1)

    def test_deactivation_requires_activation_and_cannot_rearm(self):
        for activate in (False, True):
            receiver = StreamReceiver((1,), 24, activation_type2_prefix=True)
            if activate:
                receiver.expect_activation(1, 0, 10)
            with self.subTest(activate=activate), self.assertRaisesRegex(
                    ReceiverError, "deactivation_requires_activation_prefix"):
                receiver.expect_deactivation(1, 10, 20)
        for consumed in (False, True):
            receiver = self.activated()
            receiver.expect_deactivation(1, 10, 20)
            if consumed:
                receiver.feed(feedback(), 10, 11)
            with self.subTest(consumed=consumed), self.assertRaisesRegex(
                    ReceiverError, "deactivation_already_armed"):
                receiver.expect_deactivation(1, 12, 20)

    def test_missing_off_prefix_stays_pending_despite_type24_then_expires(self):
        receiver = self.activated()
        receiver.expect_deactivation(1, 10, 20)
        receiver.feed(feedback(kind=24), 10, 20)
        result = receiver.summary(0, 20)
        self.assertFalse(result["ok"])
        self.assertIn("deactivation_prefix_pending:1", result["errors"])
        self.assertEqual(result["deactivation_pending"]["1"],
                         {"not_before_ns": 10, "deadline_ns": 20})
        with self.assertRaisesRegex(ReceiverError, "deactivation_prefix_deadline_exceeded:1"):
            receiver.feed(b"", 20, 21)

    def test_off_prefix_must_be_wholly_inside_interval(self):
        receiver = self.activated()
        receiver.expect_deactivation(1, 10, 20)
        receiver.feed(feedback()[:4], 9, 10)
        with self.assertRaisesRegex(ReceiverError, "deactivation_type2_prefix_outside_interval"):
            receiver.feed(feedback()[4:], 10, 11)
        receiver = self.activated()
        receiver.expect_deactivation(1, 10, 20)
        with self.assertRaisesRegex(ReceiverError, "deactivation_type2_prefix_outside_interval"):
            receiver.feed(feedback(), 10, 21)

    def test_invalid_off_feedback_still_fails_all_common_checks(self):
        for wire in (feedback(mode=1), feedback(fault=1), feedback(2),
                     feedback(kind=21), feedback(destination=0xFE),
                     feedback(flags=0), feedback(payload=bytes(7)),
                     feedback()[:-2] + b"xx",
                     feedback(payload=b"\x00\xc4\x56" + bytes(5))):
            receiver = self.activated()
            receiver.expect_deactivation(1, 10, 20)
            with self.subTest(wire=wire.hex()), self.assertRaises(ReceiverError):
                receiver.feed(wire, 10, 11)
            self.assertEqual(receiver.deactivation_pending, {1: (10, 20)})
            self.assertEqual(receiver.deactivation_feedback, {})


class ActivationPrefixTests(unittest.TestCase):
    def receiver(self, ids=(1,), max_frames=100000):
        return StreamReceiver(ids, 24, max_frames, activation_type2_prefix=True)

    def test_option_is_explicit_and_requires_type24(self):
        for kind in (None, 2):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                StreamReceiver((1,), kind, activation_type2_prefix=True)
        for flag in (1, 0, "true", None):
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                StreamReceiver((1,), 24, activation_type2_prefix=flag)
        receiver = StreamReceiver((1,), 24)
        with self.assertRaisesRegex(ReceiverError, "expected_frame_type_mismatch"):
            receiver.feed(feedback(kind=2), 0, 1)
        receiver = StreamReceiver((1,), 24)
        with self.assertRaisesRegex(ReceiverError, "activation_type2_prefix_not_selected"):
            receiver.expect_activation(1, 0, 10)

    def test_split_prefix_coalesced_periodic_keeps_separate_records(self):
        prefix, periodic = feedback(kind=2), feedback(kind=24)
        for cut in range(len(prefix) + 1):
            with self.subTest(cut=cut):
                receiver = self.receiver()
                receiver.expect_activation(1, 10, 30)
                self.assertEqual(receiver.activation_pending, {1: (10, 30)})
                self.assertEqual(receiver.feed(prefix[:cut], 10, 15), [])
                samples = receiver.feed(prefix[cut:] + periodic * 2, 20, 25)
                self.assertEqual(receiver.activation_pending, {})
                activation = receiver.activation_feedback[1]
                self.assertEqual(activation.host_sequence, 0)
                self.assertEqual(activation.kind, 2)
                self.assertEqual(activation.wire, prefix)
                self.assertEqual(activation.first_seen_ns, 10 if cut else 20)
                self.assertEqual(activation.received_ns, 15 if cut == len(prefix) else 25)
                self.assertEqual([s.host_sequence for s in samples], [1, 2])
                self.assertEqual([s.kind for s in receiver.samples], [24, 24])
                self.assertEqual(receiver.detected_kinds, {1: 24})
                result = receiver.summary(0, 30)
                self.assertTrue(result["ok"])
                self.assertEqual(result["samples_received"], 2)
                self.assertEqual(result["activation_feedback"]["1"]["wire_hex"], prefix.hex())
                self.assertFalse(result["activation_stop_ack_proven"])
                self.assertFalse(result["activation_configuration_ack_proven"])

    def test_prefix_only_is_not_periodic_evidence(self):
        receiver = self.receiver()
        receiver.expect_activation(1, 10, 20)
        self.assertEqual(receiver.feed(feedback(), 10, 20), [])
        self.assertEqual(receiver.samples, [])
        self.assertEqual(receiver.detected_kinds, {})
        result = receiver.summary(0, 30)
        self.assertFalse(result["ok"])
        self.assertIn("missing_motor_id:1", result["errors"])
        self.assertEqual(result["samples_received"], 0)

    def test_type24_before_prefix_and_duplicate_type2_fail_closed(self):
        receiver = self.receiver()
        receiver.expect_activation(1, 0, 10)
        with self.assertRaisesRegex(ReceiverError, "activation_type2_prefix_required"):
            receiver.feed(feedback(kind=24), 0, 1)
        self.assertEqual(receiver.activation_pending, {1: (0, 10)})
        self.assertEqual(receiver.activation_feedback, {})
        receiver = self.receiver()
        receiver.expect_activation(1, 0, 10)
        with self.assertRaisesRegex(ReceiverError, "activation_type2_prefix_duplicate"):
            receiver.feed(feedback() * 2, 0, 1)
        self.assertEqual(len(receiver.activation_feedback), 1)
        self.assertEqual(receiver.samples, [])

    def test_unarmed_ids_cannot_supply_either_kind(self):
        for kind in (2, 24):
            receiver = self.receiver((1, 2))
            receiver.expect_activation(1, 0, 10)
            with self.subTest(kind=kind), self.assertRaisesRegex(ReceiverError, "activation_not_armed"):
                receiver.feed(feedback(2, kind), 0, 1)

    def test_invalid_activation_frame_does_not_consume_expectation(self):
        cases = [
            (feedback(2), "unknown_motor_id"),
            (feedback(destination=0xFE), "unexpected_destination"),
            (feedback(kind=21), "unexpected_frame_type"),
            (feedback(flags=0), "noncanonical_flags_or_dlc"),
            (feedback(payload=bytes(7)), "noncanonical_flags_or_dlc"),
            (feedback(payload=b"\x00\xc4\x56" + bytes(5)), "version_reply_is_not_active_feedback"),
            (feedback(mode=1), "nonzero_mode_or_fault"),
            (feedback(fault=1), "nonzero_mode_or_fault"),
            (feedback()[:-2] + b"xx", "malformed_or_discarded_bytes"),
        ]
        for wire, reason in cases:
            with self.subTest(reason=reason):
                receiver = self.receiver()
                receiver.expect_activation(1, 0, 10)
                with self.assertRaisesRegex(ReceiverError, reason):
                    receiver.feed(wire, 0, 1)
                self.assertEqual(receiver.activation_pending, {1: (0, 10)})
                self.assertEqual(receiver.activation_feedback, {})
                self.assertTrue(receiver.poisoned)

    def test_early_partial_and_late_complete_prefix_rejected(self):
        receiver = self.receiver()
        receiver.expect_activation(1, 10, 20)
        receiver.feed(feedback()[:5], 9, 10)
        with self.assertRaisesRegex(ReceiverError, "activation_type2_prefix_outside_interval"):
            receiver.feed(feedback()[5:], 11, 12)
        receiver = self.receiver()
        receiver.expect_activation(1, 10, 20)
        receiver.feed(feedback()[:5], 10, 11)
        with self.assertRaisesRegex(ReceiverError, "activation_type2_prefix_outside_interval"):
            receiver.feed(feedback()[5:], 20, 21)

    def test_missing_prefix_is_pending_then_expires_on_empty_read(self):
        receiver = self.receiver()
        receiver.expect_activation(1, 10, 20)
        self.assertEqual(receiver.feed(b"", 10, 20), [])
        result = receiver.summary(0, 20)
        self.assertFalse(result["ok"])
        self.assertIn("activation_prefix_pending:1", result["errors"])
        self.assertEqual(result["activation_pending"]["1"],
                         {"not_before_ns": 10, "deadline_ns": 20})
        with self.assertRaisesRegex(ReceiverError, "activation_prefix_deadline_exceeded:1"):
            receiver.feed(b"", 20, 21)
        self.assertTrue(receiver.poisoned)
        self.assertEqual(receiver.activation_pending, {1: (10, 20)})

    def test_repeated_arm_invalid_bounds_and_wrong_id_are_rejected(self):
        for consumed in (False, True):
            receiver = self.receiver()
            receiver.expect_activation(1, 0, 10)
            if consumed:
                receiver.feed(feedback(), 0, 1)
            with self.subTest(consumed=consumed), self.assertRaisesRegex(
                    ReceiverError, "activation_already_armed"):
                receiver.expect_activation(1, 2, 10)
        for mid, start, end in ((True, 0, 10), (2, 0, 10), (1, True, 10),
                                (1, 0, True), (1, -1, 10), (1, 10, 10),
                                (1, 10, 9), (1, 0, math.inf)):
            receiver = self.receiver()
            with self.subTest(mid=mid, start=start, end=end), self.assertRaises(ReceiverError):
                receiver.expect_activation(mid, start, end)
            self.assertEqual(receiver.activation_pending, {})
        receiver = self.receiver()
        receiver.feed(b"", 0, 5)
        with self.assertRaisesRegex(ReceiverError, "activation_interval_precedes_last_read"):
            receiver.expect_activation(1, 4, 10)

    def test_sequential_activation_allows_prior_id_periodic_stream(self):
        receiver = self.receiver((1, 2), max_frames=2)
        receiver.expect_activation(1, 0, 10)
        receiver.feed(feedback(1), 0, 1)
        receiver.expect_activation(2, 2, 12)
        samples = receiver.feed(feedback(1, 24) + feedback(2) + feedback(2, 24), 3, 4)
        self.assertEqual([(s.motor_id, s.host_sequence) for s in samples], [(1, 1), (2, 1)])
        self.assertEqual(set(receiver.activation_feedback), {1, 2})
        self.assertEqual(receiver.activation_pending, {})
        self.assertEqual(receiver.detected_kinds, {1: 24, 2: 24})
        self.assertTrue(receiver.summary(0, 5)["ok"])
        with self.assertRaisesRegex(ReceiverError, "frame_capacity_exceeded"):
            receiver.feed(feedback(1, 24), 5, 6)
        self.assertEqual(len(receiver.samples), 2)
        self.assertEqual(len(receiver.activation_feedback), 2)


if __name__ == "__main__":
    unittest.main()
