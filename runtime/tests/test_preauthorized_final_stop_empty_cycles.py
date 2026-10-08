"""Synthetic protocol boundaries; no hardware observations or output approval."""
import struct
import unittest

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import rs05_trial_protocol as protocol


def synthetic_report():
    report = {'cycles': [{'end_ns': 100}], 'stop_reports': {}}
    for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13))):
        records = []
        for mid in ids:
            tx = protocol.stop_request(phase=protocol.TrialPhase.STOP, motor_id=mid)
            rx = protocol._wire((2 << 24) | (mid << 8) | 0xFD,
                                struct.pack('>4H', 32767, 32767, 32767, 300))
            records.append(dict(tx_hex=tx.hex(), rx_hex=rx.hex(),
                                start_ns=100 + mid, finish_ns=101 + mid,
                                received_ns=102 + mid, written=17, received=17))
        report['stop_reports'][bus] = dict(complete=True, confirmed_ids=list(ids),
            unconfirmed_ids=[], ambiguous_ids=[], evidence={'records': records})
    return report


class FinalStopBoundaryTests(unittest.TestCase):
    def test_complete_causal_stop_is_unchanged(self):
        self.assertEqual(live._preauthorized_boxed_sequence_raw_stop(synthetic_report()), 114)

    def test_zero_or_missing_cycles_cannot_qualify_a_successor(self):
        for value in ([], None, (), {}, 'cycle'):
            with self.subTest(value=value):
                report = synthetic_report()
                report['cycles'] = value
                with self.assertRaisesRegex(live.ProfileError, 'at least one recorded cycle'):
                    live._preauthorized_boxed_sequence_raw_stop(report)
        report = synthetic_report()
        del report['cycles']
        with self.assertRaisesRegex(live.ProfileError, 'at least one recorded cycle'):
            live._preauthorized_boxed_sequence_raw_stop(report)

    def test_old_truncated_or_enabled_stop_remains_rejected(self):
        for mutation in ('old', 'truncated', 'enabled'):
            with self.subTest(mutation=mutation):
                report = synthetic_report()
                row = report['stop_reports']['front']['evidence']['records'][0]
                if mutation == 'old':
                    row['start_ns'] = 99
                elif mutation == 'truncated':
                    row['rx_hex'] = '00'
                else:
                    row['rx_hex'] = protocol._wire(
                        (2 << 24) | (2 << 22) | (1 << 8) | 0xFD,
                        struct.pack('>4H', 32767, 32767, 32767, 300)).hex()
                with self.assertRaises(live.ProfileError):
                    live._preauthorized_boxed_sequence_raw_stop(report)


if __name__ == '__main__':
    unittest.main()
