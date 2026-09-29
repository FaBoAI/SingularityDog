"""Deterministic fault injection, no device or live runtime imports."""
import unittest
from experiments.receive_gap.contract import ReceiveGap, State

BASE = 1_000_000_000


def reply(mid, *, mode=2, fault=0, host=0xfd):
    cid = (2 << 24) | (mode << 22) | (fault << 16) | (mid << 8) | host
    return b'AT' + ((cid << 3) | 4).to_bytes(4, 'big') + b'\x08' + bytes(8) + b'\r\n'


def gap(**kwargs):
    return ReceiveGap(cycle_start_ns=BASE, requests=[
        ('front' if i <= 6 else 'rear', i, BASE + i * 100_000) for i in range(1, 13)], **kwargs)


def almost_complete(g):
    g.feed('front', b''.join(reply(i) for i in range(1, 7)), BASE + 8_000_000)
    g.feed('rear', b''.join(reply(i) for i in range(7, 12)), BASE + 9_000_000)


class GapTests(unittest.TestCase):
    def test_on_time_all_axes(self):
        g = gap(); almost_complete(g)
        self.assertEqual(g.feed('rear', reply(12), BASE + 10_000_000), State.COMPLETE)
        self.assertEqual(g.result()['skipped_slots'], 0)
        self.assertFalse(g.result()['live_resume_approved'])

    def test_one_late_reply_requires_new_fresh_state_without_new_commands(self):
        g = gap(); almost_complete(g)
        self.assertEqual(g.tick(BASE + 20_000_000), State.SKIP)
        self.assertEqual(g.feed('rear', reply(12), BASE + 26_000_000), State.RESAMPLE)
        self.assertEqual(g.result()['skipped_slots'], 1)
        self.assertFalse(g.result()['policy_inference_allowed'])
        self.assertFalse(g.result()['new_motor_writes_allowed'])

    def test_fragmented_tail_attaches_only_to_original_request(self):
        g = gap(); almost_complete(g)
        self.assertEqual(g.feed('rear', reply(12)[:15], BASE + 19_000_000), State.WAIT)
        self.assertEqual(g.feed('rear', reply(12)[15:], BASE + 22_000_000), State.RESAMPLE)

    def test_missing_does_not_reset_hard_deadline(self):
        g = gap(); almost_complete(g)
        for ms in (20, 29, 39):
            self.assertEqual(g.tick(BASE + ms * 1_000_000), State.SKIP)
        self.assertEqual(g.tick(BASE + 40_000_000), State.STOP)
        self.assertEqual(g.feed('rear', reply(12), BASE + 41_000_000), State.STOP)

    def test_deadline_boundary_is_not_on_time(self):
        g = gap(); almost_complete(g)
        self.assertEqual(g.feed('rear', reply(12), BASE + 20_000_000), State.RESAMPLE)
        g = gap(); almost_complete(g)
        self.assertEqual(g.feed('rear', reply(12), BASE + 40_000_000), State.STOP)

    def test_startup_and_consecutive_gaps_stop(self):
        for kwargs in ({'phase': 'startup'}, {'previous_slot_skipped': True},
                       {'skips_in_previous_99_slots': 1}):
            g = gap(**kwargs); almost_complete(g)
            self.assertEqual(g.tick(BASE + 20_000_000), State.STOP)

    def test_fault_mode_host_and_malformed_are_not_skips(self):
        for raw in (reply(12, fault=1), reply(12, mode=0), reply(12, host=0),
                    b'xx' + reply(12)[2:]):
            g = gap(); almost_complete(g)
            self.assertEqual(g.feed('rear', raw, BASE + 21_000_000), State.STOP)

    def test_old_or_duplicate_cannot_satisfy_new_exchange(self):
        g = gap(); almost_complete(g)
        self.assertEqual(g.feed('front', reply(1), BASE + 21_000_000), State.STOP)
        self.assertEqual(g.feed('rear', reply(12), BASE + 22_000_000), State.STOP)
        g = gap(); almost_complete(g)
        g.feed('rear', reply(12), BASE + 22_000_000)
        self.assertEqual(g.feed('rear', reply(12), BASE + 23_000_000), State.STOP)

    def test_clock_disconnect_and_new_write_stop(self):
        g = gap(); almost_complete(g)
        self.assertEqual(g.tick(BASE), State.STOP)
        g = gap()
        self.assertEqual(g.transport_error('EOF'), State.STOP)
        g = gap(); g.tick(BASE + 20_000_000)
        self.assertEqual(g.attempt_new_command(), State.STOP)

    def test_partial_extra_or_foreign_frame_is_rejected(self):
        g = gap(); almost_complete(g)
        self.assertEqual(g.feed('rear', reply(12) + b'A', BASE + 21_000_000), State.STOP)
        g = gap(); almost_complete(g)
        self.assertEqual(g.feed('rear', reply(1), BASE + 21_000_000), State.STOP)

    def test_request_must_be_written_before_deadline_and_unique(self):
        for requests in ([('front', 1, BASE + 20_000_000)],
                         [('front', 1, BASE), ('front', 1, BASE)],
                         [('rear', 1, BASE)], []):
            with self.assertRaises(ValueError):
                ReceiveGap(cycle_start_ns=BASE, requests=requests)


if __name__ == '__main__':
    unittest.main()
