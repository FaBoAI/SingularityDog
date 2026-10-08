"""File-only replay guards; native equivalence has a separate exhaustive test."""
import ctypes
import importlib.util
from pathlib import Path
import struct
import unittest

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import rs05_trial_protocol as protocol

spec = importlib.util.spec_from_file_location('verify_native_feedback_decode',
                                             Path(__file__).with_name('verify_native_feedback_decode.py'))
subject = importlib.util.module_from_spec(spec)
spec.loader.exec_module(subject)


def saved_row(mid, parameter=False):
    tx = codec.read_request(mid, 'voltage') if parameter else native.encode_motion(mid, .1, 3., .15)
    can_id = ((17 if parameter else 2) << 24) | (mid << 8) | 0xfd
    payload = b'\x1c\x70\x00\x00' + struct.pack('<f', 40.) if parameter else struct.pack('>4H', 32767, 32767, 32767, 250)
    return {'tx_hex': tx.hex(), 'rx_hex': protocol._wire(can_id, payload).hex(),
            'start_ns': 1, 'finish_ns': 2, 'read_start_ns': 3, 'received_ns': 4,
            'deadline_ns': 5, 'written': 17, 'received': 17}


def saved_cycle():
    return {phase: {bus: {'records': [saved_row(mid, phase == 'voltage') for mid in ids]}
                    for bus, ids in (('front', [1] if phase == 'voltage' else range(1, 7)),
                                     ('rear', [7] if phase == 'voltage' else range(7, 13)))}
            for phase in ('acquired', 'voltage', 'output')}


class Decoder:
    def decode(self, records, first):
        return None if len(records) != 6 else runtime.decode_records((records, None))


class SavedFeedbackDecodeTests(unittest.TestCase):
    def test_original_records_and_parameter_fallback_counts_are_preserved(self):
        result = subject.verify([saved_cycle()], Decoder())
        self.assertEqual(result['native_batches'], 4)
        self.assertEqual(result['native_feedback_rows'], 24)
        self.assertEqual(result['legacy_batches'], 2)
        self.assertEqual(result['original_records'], 26)
        self.assertFalse(result['runtime_qualification'])
        self.assertFalse(result['hardware_opened'])

    def test_uint32_uint64_and_hex_boundaries_reject_wrapping_and_coercion(self):
        for field in subject.FIELDS:
            limit = 2**32 if field in ('written', 'received') else 2**64
            for value in (True, -1, limit, 1.0, '1'):
                row = saved_row(1); row[field] = value
                with self.assertRaises(ValueError): subject.owned_records([row])
            row = saved_row(1); row[field] = limit - 1
            self.assertEqual(getattr(subject.owned_records([row])[0], field), limit - 1)
        for value in ('00' * 16, '00' * 18, 'AA' * 17, ' 0' * 17, None):
            row = saved_row(1); row['rx_hex'] = value
            with self.assertRaises(ValueError): subject.owned_records([row])

    def test_eligible_feedback_cannot_silently_fall_back(self):
        class Unavailable(Decoder):
            def decode(self, records, first): return None
        with self.assertRaisesRegex(AssertionError, 'Eligible'):
            subject.verify([saved_cycle()], Unavailable())

    def test_order_or_raw_mutation_cannot_be_reported_as_equal(self):
        class Reordered(Decoder):
            def decode(self, records, first):
                result = super().decode(records, first)
                return None if result is None else dict(reversed(list(result.items())))
        with self.assertRaisesRegex(AssertionError, 'order differ'):
            subject.verify([saved_cycle()], Reordered())
        class Mutated(Decoder):
            def decode(self, records, first):
                result = super().decode(records, first)
                records[0].read_start_ns += 1
                return result
        with self.assertRaisesRegex(AssertionError, 'modified'):
            subject.verify([saved_cycle()], Mutated())

    def test_missing_phase_bus_or_oversized_batch_cannot_be_skipped(self):
        for saved in ([], [{}], [dict(acquired=saved_cycle()['acquired'])]):
            with self.assertRaises(ValueError): subject.verify(saved, Decoder())
        with self.assertRaises(ValueError): subject.owned_records([saved_row(1)] * 13)


if __name__ == '__main__': unittest.main()
