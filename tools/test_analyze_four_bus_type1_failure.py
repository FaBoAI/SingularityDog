"""File-only tests of physical-stage classification and original-evidence gates."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest

import analyze_four_bus_type1_failure as tool


def frame(kind, mid=3, *, reply=False, mode=0, kp=0):
    if reply:
        cid = (kind << 24) | (mode << 22) | (mid << 8) | 253
        data = bytes(8)
    elif kind == 1:
        cid = (1 << 24) | (32767 << 8) | mid
        data = struct.pack('>4H', 32767, 32767, kp, 0)
    else:
        cid, data = (kind << 24) | (253 << 8) | mid, bytes(8)
    return (b'AT'+((cid << 3) | 4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n').hex()


def fixture(label='enable', kind=3, budget=30_000_000):
    begin = 1_000_000_000
    record = {'tx_hex': frame(kind), 'rx_hex': '00'*17, 'start_ns': begin+100,
              'finish_ns': begin+200, 'read_start_ns': 0, 'received_ns': 0,
              'deadline_ns': begin+budget, 'written': 17, 'received': 0}
    batch = {'records': [record], 'stats': {'begin_ns': begin, 'end_ns': begin+budget+100,
              'writes': 1, 'reads': 0, 'bytes': 0}, 'rejected_total_bytes': 0,
             'rejected_hex': '', 'rejected_truncated': False}
    report = {'schema': tool.SCHEMAS[1], 'first_release_monotonic_ns': None, 'completed_cycles': 0,
              'control_clock': {'policy_hz': 30, 'inference_tx_budget_ns': 20_000_000,
                                'cycle_budget_ns': 33_333_334},
              'topology_by_port': {'port0': [7, 8, 9], 'port1': [10, 11, 12],
                                   'port2': [4, 5, 6], 'port3': [1, 2, 3]},
              'measurement': {'raw_journal': {p: [] for p in tool.PORTS},
                  'zero_gain_enable_transition': {'complete': False,
                       'current_axis': {'port': 'port3', 'motor_id': 3},
                       'current_stage': 'zero_gain' if label == 'zero_gain' else 'enable'}},
              'terminal_stop': {'stop_confirmed': False,
                  'confirmed_ids': [mid for mid in range(1, 13) if mid != 3],
                  'unconfirmed_ids': [3], 'ambiguous_ids': [3], 'physical_cutoff_required': True}}
    report['measurement']['raw_journal']['port3'] = [[label, batch]]
    return report


class ClassificationTests(unittest.TestCase):
    def test_enable_timeout_is_setup30_not_inference20(self):
        source = fixture()
        before = copy.deepcopy(source)
        result = tool.analyze(source)
        failed = result['failed_requests'][0]
        self.assertEqual((failed['stage'], failed['native_reply_budget_ms']), ('SETUP_ENABLE_TYPE3', 30))
        self.assertEqual(failed['state'], 'FULL_WRITE_NO_HOST_RX')
        self.assertFalse(result['first_cycle_started'])
        self.assertEqual(result['inference_tx_budget_ms'], 20)
        self.assertFalse(result['physical_layer_root_cause_confirmed'])
        self.assertFalse(result['output_allowed'])
        self.assertEqual(source, before)

    def test_zero_gain20_is_not_a_failed_cycle(self):
        result = tool.analyze(fixture('zero_gain', 1, 20_000_000))
        self.assertEqual(result['failed_requests'][0]['stage'], 'SETUP_SINGLE_AXIS_ZERO_GAIN')
        self.assertEqual(result['failed_requests'][0]['native_reply_budget_ms'], 20)
        self.assertFalse(result['first_cycle_started'])

    def test_initial_hold_and_cyclic_output_are_separate(self):
        source = fixture('output', 1)
        self.assertEqual(tool.analyze(source)['failed_requests'][0]['stage'], 'SETUP_INITIAL_ZERO_GAIN_HOLD')
        source['first_release_monotonic_ns'] = 999_999_999
        self.assertEqual(tool.analyze(source)['failed_requests'][0]['stage'], 'CYCLE_TYPE1_OUTPUT')
        source['measurement']['raw_journal']['port3'][0][0] = 'feedback_hold'
        self.assertEqual(tool.analyze(source)['failed_requests'][0]['stage'], 'CYCLE_FEEDBACK_HOLD')

    def test_later_mode_zero_never_clears_sticky_ambiguity(self):
        source = fixture()
        late = copy.deepcopy(source['measurement']['raw_journal']['port3'][0][1])
        row = late['records'][0]
        row.update(tx_hex=frame(4), rx_hex=frame(2, reply=True), received=17,
                   read_start_ns=row['finish_ns']+100, received_ns=row['finish_ns']+200)
        late['stats'].update(reads=1, bytes=17)
        source['measurement']['raw_journal']['port3'].append(['stop_subset', late])
        result = tool.analyze(source)
        self.assertEqual(result['terminal_STOP_owner_summary']['mode_zero_bytes_observed_for_unconfirmed_ids'], [3])
        self.assertEqual(result['terminal_STOP_owner_summary']['ambiguous_ids'], [3])
        self.assertFalse(result['sticky_STOP_ambiguity_cleared'])
        self.assertFalse(result['stop_confirmation_created'])

    def test_label_and_native_request_type_mismatch_rejected(self):
        source = fixture(kind=1)
        with self.assertRaisesRegex(ValueError, 'type differs'):
            tool.analyze(source)

    def test_transition_axis_mismatch_rejected(self):
        source = fixture()
        source['measurement']['zero_gain_enable_transition']['current_axis']['motor_id'] = 2
        with self.assertRaisesRegex(ValueError, 'Transition stage differs'):
            tool.analyze(source)

    def test_noncanonical_enable_payload_rejected(self):
        source = fixture()
        row = source['measurement']['raw_journal']['port3'][0][1]['records'][0]
        raw = bytearray.fromhex(row['tx_hex'])
        raw[7] = 1
        row['tx_hex'] = raw.hex()
        with self.assertRaisesRegex(ValueError, 'Noncanonical setup/control'):
            tool.analyze(source)

    def test_positive_gain_disguised_as_zero_rejected(self):
        source = fixture('zero_gain', 1)
        source['measurement']['raw_journal']['port3'][0][1]['records'][0]['tx_hex'] = frame(1, kp=1)
        with self.assertRaisesRegex(ValueError, 'positive gains'):
            tool.analyze(source)

    def test_noncausal_reply_rejected(self):
        source = fixture()
        batch = source['measurement']['raw_journal']['port3'][0][1]
        row = batch['records'][0]
        row.update(rx_hex=frame(2, reply=True), received=17,
                   read_start_ns=row['finish_ns']-1, received_ns=row['finish_ns']+1)
        batch['stats'].update(reads=1, bytes=17)
        with self.assertRaisesRegex(ValueError, 'Noncausal complete reply'):
            tool.analyze(source)

    def test_native_counter_mismatch_rejected(self):
        source = fixture()
        source['measurement']['raw_journal']['port3'][0][1]['stats']['bytes'] = 1
        with self.assertRaisesRegex(ValueError, 'Native counters differ'):
            tool.analyze(source)

    def test_wrong_clock_cannot_relabel_a20ms_trial(self):
        source = fixture()
        source['control_clock']['cycle_budget_ns'] = 20_000_000
        with self.assertRaisesRegex(ValueError, 'Explicit 30Hz'):
            tool.analyze(source)


class PinnedFileTests(unittest.TestCase):
    def test_sha_mismatch_duplicate_json_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp).resolve()
            path = directory/'report.json'
            raw = json.dumps(fixture()).encode()
            path.write_bytes(raw)
            digest = hashlib.sha256(raw).hexdigest()
            self.assertEqual(tool.load_report(path, digest), fixture())
            with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
                tool.load_report(path, '0'*64)
            link = directory/'link.json'
            link.symlink_to(path)
            with self.assertRaises(OSError):
                tool.load_report(link, digest)
            raw = b'{"schema": 1, "schema": 2}'
            path.write_bytes(raw)
            with self.assertRaisesRegex(ValueError, 'Duplicate JSON key'):
                tool.load_report(path, hashlib.sha256(raw).hexdigest())

    def test_output_is_fresh_private_and_never_overwrites_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp).resolve()/'analysis.json'
            tool.write_fresh(path, {'output_allowed': False})
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            original = path.read_bytes()
            with self.assertRaises(FileExistsError):
                tool.write_fresh(path, {'output_allowed': True})
            self.assertEqual(path.read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
