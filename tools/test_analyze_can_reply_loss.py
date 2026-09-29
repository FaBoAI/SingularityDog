import copy
import unittest

from tools.analyze_can_reply_loss import analyze


def wire(cid):
    return (b'AT' + ((cid << 3) | 4).to_bytes(4, 'big') + b'\x08' + bytes(8) + b'\r\n').hex()


def sample():
    record = {'tx_hex': wire((1 << 24) | (32767 << 8) | 12),
              'rx_hex': '00' * 17, 'written': 17, 'received': 0,
              'start_ns': 1_000_000, 'finish_ns': 1_100_000, 'read_start_ns': 0,
              'received_ns': 0, 'deadline_ns': 20_000_000}
    prefix = wire((2 << 24) | (2 << 22) | (12 << 8) | 0xfd)[:-4]
    return {'status': 'ABORTED', 'cycles': [], 'journal': [
        {'bus': 'rear', 'phase': 'policy_output', 'error': 'deadline', 'records': [record],
         'stats': {'bytes': 15, 'reads': 1, 'end_ns': 20_100_000}, 'rejected_hex': prefix}],
        'stop_reports': {'rear': {'attempts': [{'evidence': {
            'stats': {'begin_ns': 20_200_000}, 'rejected_hex': '0d0a'}}]}}}


class LossTests(unittest.TestCase):
    def test_partial_tail_is_candidate_not_recovered_ack(self):
        report = sample()
        original = copy.deepcopy(report)
        r = analyze(report)
        self.assertEqual(report, original)
        f = r['failed_batches'][0]
        self.assertEqual(f['late_tail_candidate']['id'], 12)
        self.assertEqual(f['unresolved'][0]['write_to_deadline_ms'], 18.9)
        self.assertFalse(f['skip_recovery_proven'])
        self.assertFalse(f['late_tail_candidate']['arrival_within_one_extra_cycle_proven'])
        self.assertFalse(r['runtime_skip_approved'])

    def test_later_tail_needs_matching_id_mode_and_untruncated_evidence(self):
        for modifier in (
            lambda d: d['journal'][0].update(rejected_truncated=True),
            lambda d: d['stop_reports']['rear']['attempts'][0]['evidence'].update(rejected_hex='ffff'),
            lambda d: d['stop_reports']['rear']['attempts'][0]['evidence'].update(rejected_truncated=True),
            lambda d: d['journal'][0].update(rejected_hex=wire((2 << 24) | (2 << 22) | (11 << 8) | 0xfd)[:-4]),
            lambda d: d['journal'][0].update(rejected_hex=wire((2 << 24) | (12 << 8) | 0xfd)[:-4]),
            lambda d: d['stop_reports']['rear']['attempts'][0]['evidence']['stats'].update(begin_ns=100)):
            d = sample()
            modifier(d)
            self.assertIsNone(analyze(d)['failed_batches'][0]['late_tail_candidate'])

    def test_complete_absence_does_not_prove_can_loss(self):
        d = sample()
        d['journal'][0].update(rejected_hex='')
        d['journal'][0]['stats']['bytes'] = 0
        r = analyze(d)
        self.assertIsNone(r['failed_batches'][0]['late_tail_candidate'])
        self.assertEqual(r['request_groups'][0]['incomplete_records'], 1)
        self.assertTrue(r['root_cause'].startswith('UNRESOLVED'))

    def test_success_latency_and_wrong_bus(self):
        d = sample()
        b = d['journal'][0]
        b.update(error=None, rejected_hex='')
        b['records'][0].update(received=17, read_start_ns=3_000_000, received_ns=3_100_000,
                               rx_hex=wire((2 << 24) | (2 << 22) | (12 << 8) | 0xfd))
        r = analyze(d)
        self.assertEqual(r['failed_batches'], [])
        self.assertEqual(r['request_groups'][0]['write_return_to_reply_ms']['median'], 2)
        b['bus'] = 'front'
        with self.assertRaises(ValueError):
            analyze(d)


if __name__ == '__main__':
    unittest.main()
