"""Synthetic scheduling checks; no CAN, IMU, or motor is opened."""

import unittest

from analyze_cycle_overlap import build_profiles, simulate, summarize


MS = 1_000_000


def batch(start, write, reply, end):
    frame = {'start_ns': start, 'finish_ns': write, 'received_ns': reply,
             'written': 17, 'received': 17}
    return {'records': [dict(frame) for _ in range(6)],
            'stats': {'begin_ns': start, 'end_ns': end}}


def fixture(cycle_tail_ms=3.5):
    records = []
    measurements = []
    for cycle in (1, 2):
        release = 1_000_000_000 + (cycle - 1) * 30 * MS
        imu_start = release + MS // 10
        front_start = release + MS // 5
        rear_start = release + 3 * MS // 10
        front_reply = release + 18 * MS
        rear_reply = release + 19 * MS
        output_end = rear_reply
        cycle_end = output_end + int(cycle_tail_ms * MS)
        records.append({'cycle': cycle,
                        'imu': {'read_started_monotonic_ns': imu_start,
                                'read_finished_monotonic_ns': release + 4 * MS},
                        'acquired': {
                            'front': batch(front_start, release + 4 * MS,
                                           release + 7 * MS, release + 7 * MS),
                            'rear': batch(rear_start, release + 4 * MS,
                                          release + 7 * MS, release + 7 * MS)},
                        'output': {
                            'front': batch(release + 11 * MS, release + 15 * MS,
                                           front_reply, front_reply),
                            'rear': batch(release + 11 * MS, release + 15 * MS,
                                          rear_reply, rear_reply)}})
        measurements.append({'release_ns': release,
                             'oldest_input_start_ns': imu_start,
                             'gather_end_ns': release + 7 * MS,
                             'infer_end_ns': release + 11 * MS,
                             'final_host_write_ns': release + 15 * MS,
                             'last_proxy_reply_ns': rear_reply,
                             'cycle_end_ns': cycle_end,
                             'whole_iteration_ms': 19 + cycle_tail_ms})
    report = {'status': 'COMPLETE_DIAGNOSTIC', 'mode': 'stop-proxy',
              'motor_enable_sent': False, 'learned_targets_sent': False,
              'cycles_requested': 2, 'cycles_completed': 2,
              'measurements': measurements}
    return report, records


class CycleOverlapTests(unittest.TestCase):
    def test_reply_gate_hides_cleanup_and_early_lookahead(self):
        report, records = fixture()
        profiles = build_profiles(report, records, expected_cycles=2)
        serial = simulate(profiles, overlap=False)
        overlap = simulate(profiles, overlap=True)
        early = simulate(profiles, overlap=True, lead_ns=2 * MS)
        self.assertEqual(serial[1]['launch'], int(22.5 * MS))
        self.assertEqual(overlap[1]['launch'], 20 * MS)
        self.assertEqual(early[1]['launch'], 19 * MS)
        self.assertGreaterEqual(early[1]['launch'], early[0]['output_end'])
        self.assertEqual(overlap[1]['write'] - overlap[1]['oldest'],
                         serial[1]['write'] - serial[1]['oldest'])
        self.assertEqual(early[1]['write'] - early[1]['oldest'],
                         serial[1]['write'] - serial[1]['oldest'])
        self.assertEqual(summarize(early)['oldest_input_to_host_write_over_20ms'], 0)

    def test_main_cleanup_can_make_prefetched_input_stale(self):
        report, records = fixture(cycle_tail_ms=14)
        profiles = build_profiles(report, records, expected_cycles=2)
        early = simulate(profiles, overlap=True, lead_ns=2 * MS)
        self.assertEqual(early[1]['launch'], 19 * MS)
        self.assertGreater(early[1]['write'] - early[1]['oldest'], 20 * MS)
        self.assertEqual(summarize(early)['oldest_input_to_host_write_over_20ms'], 1)

    def test_incomplete_output_rejected(self):
        report, records = fixture()
        records[0]['output']['rear']['records'][0]['received'] = 0
        with self.assertRaisesRegex(ValueError, 'incomplete or noncausal'):
            build_profiles(report, records, expected_cycles=2)

    def test_enabled_run_rejected(self):
        report, records = fixture()
        report['motor_enable_sent'] = True
        with self.assertRaisesRegex(ValueError, 'disabled STOP-proxy'):
            build_profiles(report, records, expected_cycles=2)


if __name__ == '__main__':
    unittest.main()
