"""Pure native six-feedback codec equivalence; no sessions, FD or hardware."""
import ctypes as C
import importlib.util
import random
import struct
import threading
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.can_readonly import read_request
from singularitydog_hw.motor_version_probe import version_request
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from test_native_active_transport import ROOT, frame, reply, version_reply


class NativeFeedbackBatchDecodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('build_feedback_batch_native', ROOT / 'build.py')
        builder = importlib.util.module_from_spec(spec); spec.loader.exec_module(builder)
        cls.lib = native.load_library(builder.build())

    def setUp(self):
        self.decoder = native.NativeFeedbackBatchDecoder(self.lib)

    def records(self, first=1, order=None, values=None, mode=0, fault=0, kinds=None):
        records = (Record * 6)()
        order = list(range(first, first + 6)) if order is None else order
        values = [(32767, 32767, 32767, 250)] * 6 if values is None else values
        kinds = [4] * 6 if kinds is None else kinds
        for index, (motor, data, kind) in enumerate(zip(order, values, kinds)):
            r = records[index]
            r.start_ns = 100 + index; r.finish_ns = 150 + index
            r.read_start_ns = 175 + index; r.received_ns = 200 + index; r.deadline_ns = 300 + index
            r.written = r.received = 17
            # Legacy feedback decoding does not impose command payload/gain
            # limits: transport validation is separate. Preserve that split.
            tx = frame((kind << 24) | (0xfd << 8) | motor, bytes(8))
            rx = frame((2 << 24) | (mode << 22) | (fault << 16) | (motor << 8) | 0xfd,
                       struct.pack('>4H', *data))
            r.tx[:] = tx; r.rx[:] = rx
        return records

    def legacy(self, records):
        return runtime.decode_records((records, native.Stats()))

    def same(self, records, first=1):
        raw = bytes(records)
        expected = self.legacy(records)
        actual = self.decoder.decode(records, first)
        self.assertEqual(list(actual), list(expected))
        self.assertEqual(actual, expected)
        for key in expected:
            left, right = actual[key][0], expected[key][0]
            left_bits = struct.pack('>4d', left.protocol_position_rad, left.velocity_rad_s,
                                    left.torque_nm, left.temperature_c)
            right_bits = struct.pack('>4d', right.protocol_position_rad, right.velocity_rad_s,
                                     right.torque_nm, right.temperature_c)
            self.assertEqual(left_bits, right_bits)
        self.assertEqual(bytes(records), raw)

    def test_all_uint16_values_are_bit_exact_for_every_feedback_numeric_field(self):
        # Each of p/v/torque/temp exhausts its entire 16-bit encoding range.
        # This also verifies the exact multiply/divide/add rounding order.
        for start in range(0, 65536, 6):
            values = [min(65535, start + i) for i in range(6)]
            records = self.records(values=[(v, v, v, v) for v in values])
            self.same(records)

    def test_native_codec_abi_matches_every_owned_record_and_output_field_offset(self):
        self.assertEqual(self.lib.sda_feedback_decode_abi(), 1)
        self.assertEqual(C.sizeof(Record), 88)
        self.assertEqual({key: getattr(Record, key).offset for key, _ in Record._fields_}, {
            'start_ns': 0, 'finish_ns': 8, 'read_start_ns': 16, 'received_ns': 24,
            'deadline_ns': 32, 'tx': 40, 'rx': 57, 'written': 76, 'received': 80})
        self.assertEqual(C.sizeof(native.FeedbackDecoded), 64)
        self.assertEqual({key: getattr(native.FeedbackDecoded, key).offset
                          for key, _ in native.FeedbackDecoded._fields_}, {
            'motor_id': 0, 'mode_state': 4, 'fault_bits': 8, 'position_u16': 12,
            'protocol_position_rad': 16, 'velocity_rad_s': 24, 'torque_nm': 32,
            'temperature_c': 40, 'request_started_ns': 48, 'received_ns': 56})

    def test_random_payloads_bus_order_modes_faults_and_command_types_match(self):
        rng = random.Random(890)
        for first in (1, 7):
            for _ in range(150):
                order = list(range(first, first + 6)); rng.shuffle(order)
                values = [tuple(rng.randrange(65536) for _ in range(4)) for _ in range(6)]
                self.same(self.records(first, order, values, mode=rng.randrange(3),
                    fault=rng.randrange(64), kinds=[rng.choice((1, 3, 4, 18)) for _ in range(6)]), first)

    def test_faults_and_accepted_mode_one_are_preserved_without_motion_admission(self):
        self.same(self.records(mode=1, fault=63, kinds=[1, 3, 4, 18, 1, 4]))
        self.assertEqual(self.decoder.decode(self.records(mode=1, fault=63), 1)[(1, 'feedback')][0].fault_bits, 63)

    def test_original_maximum_uint64_timestamps_survive_unchanged(self):
        records = self.records()
        for r in records:
            r.start_ns = 2**64 - 400; r.finish_ns = 2**64 - 300
            r.read_start_ns = 2**64 - 250; r.received_ns = 2**64 - 200; r.deadline_ns = 2**64 - 1
        self.same(records)

    def test_invalid_supported_records_fall_back_to_exact_original_errors(self):
        def change_wire(record, field, index, value):
            getattr(record, field)[index] = value
        cases = [
            lambda a: setattr(a[0], 'written', 16),
            lambda a: setattr(a[0], 'received', 16),
            lambda a: setattr(a[0], 'start_ns', 0),
            lambda a: setattr(a[0], 'finish_ns', a[0].start_ns - 1),
            lambda a: setattr(a[0], 'received_ns', a[0].finish_ns - 1),
            lambda a: setattr(a[0], 'received_ns', a[0].deadline_ns),
            lambda a: change_wire(a[0], 'tx', 0, ord('X')),
            lambda a: change_wire(a[0], 'rx', 5, a[0].rx[5] ^ 1),
            lambda a: change_wire(a[0], 'rx', 6, 7),
            lambda a: change_wire(a[0], 'rx', 16, 0),
            lambda a: a[0].rx.__setitem__(slice(None), reply(bytes(a[0].tx), host=0xfc)),
            lambda a: a[0].rx.__setitem__(slice(None), reply(bytes(a[0].tx), source=2)),
            lambda a: a[0].rx.__setitem__(slice(None), reply(bytes(a[0].tx), mode=3)),
            lambda a: a[0].rx.__setitem__(slice(None), version_reply(bytes(a[0].tx))),
            lambda a: C.memmove(C.addressof(a[1]), C.addressof(a[0]), C.sizeof(Record)),
        ]
        for mutate in cases:
            with self.subTest(mutate=mutate):
                records = self.records(); mutate(records); raw = bytes(records)
                with self.assertRaises((ValueError, RuntimeError)) as expected: self.legacy(records)
                self.assertIsNone(self.decoder.decode(records, 1))
                with self.assertRaises(type(expected.exception)) as actual: self.legacy(records)
                self.assertEqual(str(actual.exception), str(expected.exception))
                self.assertEqual(bytes(records), raw)

    def test_mixed_identity_parameter_and_version_keep_authoritative_legacy_rows(self):
        for tx in (read_request(1), frame((0 << 24) | (0xfd << 8) | 1, bytes(8)), version_request(1)):
            records = self.records(); records[0].tx[:] = tx
            records[0].rx[:] = version_reply(tx) if tx == version_request(1) else reply(tx)
            raw = bytes(records); expected = self.legacy(records)
            self.assertIsNone(self.decoder.decode(records, 1))
            self.assertEqual(self.legacy(records), expected); self.assertEqual(bytes(records), raw)

    def test_only_exact_root_owned_six_record_arrays_are_eligible(self):
        records = self.records(); copied_list = list(records)
        alias = (Record * 6).from_address(C.addressof(records))
        pointer_view = C.cast(records, C.POINTER(Record * 6)).contents
        subclass = type('OtherRecordArray', (Record * 6,), {})()
        for unsupported in (copied_list, alias, pointer_view, subclass, (Record * 5)(), (Record * 7)()):
            self.assertIsNone(self.decoder.decode(unsupported, 1))
        for first in (True, 0, 2, 12, 1.0, '1'):
            self.assertIsNone(self.decoder.decode(records, first))
        self.assertIsNone(self.decoder.decode(records, 7))
        self.same(records)

    def test_absent_optional_codec_preserves_fallback_and_partial_codec_fails_closed(self):
        with patch.multiple(self.lib, **dict.fromkeys(native._FEEDBACK_SYMBOLS)):
            decoder = native.NativeFeedbackBatchDecoder(self.lib)
            self.assertFalse(decoder.available); self.assertIsNone(decoder.decode(self.records(), 1))
        with patch.object(self.lib, 'sda_feedback_decode_batch', None):
            with self.assertRaisesRegex(ValueError, 'Incomplete'):
                native.NativeFeedbackBatchDecoder(self.lib)
        with patch.object(self.lib, 'sda_feedback_decode_abi', Mock(return_value=1)):
            with self.assertRaisesRegex(ValueError, 'Exact GIL-releasing'):
                native.NativeFeedbackBatchDecoder(self.lib)

    def test_changed_function_signature_or_declared_scale_cannot_silently_decode(self):
        with patch.object(self.lib, 'sda_feedback_decode_batch', Mock(return_value=0)):
            with self.assertRaisesRegex(ValueError, 'Exact GIL-releasing'):
                self.decoder.decode(self.records(), 1)
        with patch.object(protocol, 'POSITION_MIN', -1.):
            self.assertIsNone(self.decoder.decode(self.records(), 1))

    def test_scratch_reuse_never_publishes_previous_generation_after_failure(self):
        first = self.records(values=[(0, 0, 0, 0)] * 6); self.same(first)
        bad = self.records(); bad[0].written = 0
        self.assertIsNone(self.decoder.decode(bad, 1))
        last = self.records(values=[(65535, 65535, 65535, 65535)] * 6); self.same(last)
        self.assertEqual(self.decoder.decode(last, 1)[(1, 'feedback')][0].position_u16, 65535)

    def test_reuse_on_another_thread_is_allowed_and_contention_uses_legacy_fallback(self):
        records = self.records(); self.same(records)
        results, failures = [], []
        def decode():
            try: results.append(self.decoder.decode(records, 1))
            except BaseException as error: failures.append(error)
        thread = threading.Thread(target=decode); thread.start(); thread.join(.5)
        self.assertFalse(thread.is_alive()); self.assertFalse(failures)
        self.assertEqual(results, [self.legacy(records)])
        with self.decoder._busy: self.assertIsNone(self.decoder.decode(records, 1))
        self.same(records)


if __name__ == '__main__': unittest.main()
