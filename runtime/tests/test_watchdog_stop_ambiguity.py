"""Synthetic owned transactions; no serial device, motor or model is used."""
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import serial_deadline_reader
from singularitydog_hw import watchdog_commissioning as watchdog


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(1, round(seconds*1e9))


class BrokenMessageError(OSError):
    def __str__(self):
        raise RuntimeError('formatter must not run during STOP')


class Port:
    def __init__(self, clock):
        self.clock = clock
        self.writes = []
        self.in_waiting = 0
        self.last = None
        self.delivered = False
        self.drop_enable = False
        self.drop_stops = set()
        self.partial_stops = set()
        self.read_errors = {}
        self.faults = {}
        self.read_calls = 0

    def write(self, raw):
        frame, = codec.ATParser().feed(raw)
        self.writes.append(frame)
        self.last, self.delivered = frame, False
        self.clock.now += 10_000
        return 16 if frame.kind == 4 and frame.destination in self.partial_stops else len(raw)


class Reader:
    def __init__(self, port):
        self.port = port

    def read_until(self, wake, hard):
        port, frame = self.port, self.port.last
        port.read_calls += 1
        mid = frame.destination
        if mid in port.read_errors:
            error = port.read_errors.pop(mid)
            raise error
        if (port.delivered or (frame.kind == 3 and port.drop_enable)
                or (frame.kind == 4 and mid in port.drop_stops)):
            port.clock.now = hard
            raise TimeoutError('synthetic absolute receive deadline')
        port.delivered = True
        port.in_waiting = 0
        port.clock.now += 500_000
        mode = 2 if frame.kind == 3 else 0
        cid = (2 << 24) | (mode << 22) | (port.faults.get(mid, 0) << 16) | (mid << 8) | 0xFD
        raw = (b'AT'+((cid << 3) | 4).to_bytes(4, 'big')+b'\x08'
               +struct.pack('>4H', 32767, 32767, 32767, 250)+b'\r\n')
        return raw, port.clock.now


class WatchdogStopAmbiguityTests(unittest.TestCase):
    def make_channel(self):
        clock = Clock()
        port = Port(clock)
        channel = watchdog.Channel(port, watchdog.BUSES['rear'], clock=clock, reader=Reader(port))
        self.addCleanup(patch.stopall)
        patch.object(serial_deadline_reader, 'DeadlineSerialReader',
                     side_effect=lambda port, **_: Reader(port)).start()
        patch.object(watchdog.time, 'sleep', side_effect=clock.sleep).start()
        return channel, port

    def assert_one_stop_round(self, frames):
        self.assertEqual([frame.destination for frame in frames], list(range(7, 13)))
        for frame in frames:
            self.assertEqual((frame.kind, frame.data), (4, bytes(8)))

    def test_id10_no_enable_reply_stays_ambiguous_after_two_fully_observed_stop_rounds(self):
        channel, port = self.make_channel()
        port.drop_enable = True
        with self.assertRaises(TimeoutError):
            channel.exchange(10, 'enable')
        self.assertTrue(channel.failed)
        self.assertEqual(channel.pending, {10: 'enable'})
        for _ in range(2):
            before_reads = port.read_calls
            result = channel.stop_all()
            self.assertFalse(result['complete'])
            self.assertEqual(result['ambiguous_ids'], [10])
            self.assertEqual(result['unconfirmed_ids'], [10])
            self.assertEqual(result['observed_reset_ids'], list(range(7, 13)))
            self.assertEqual(result['unresolved_state_steps'], {'10': 'enable'})
            self.assertTrue(result['ambiguity_preserved'])
            self.assertEqual(port.read_calls-before_reads, 6)
        self.assertEqual(port.writes[0].kind, 3)
        self.assert_one_stop_round(port.writes[1:7])
        self.assert_one_stop_round(port.writes[7:13])

    def test_missing_stop_reply_latches_even_when_next_round_receives_all_reset_frames(self):
        channel, port = self.make_channel()
        port.drop_stops = {10}
        first = channel.stop_all()
        self.assertEqual(first['ambiguous_ids'], [10])
        self.assertEqual(first['unresolved_state_steps'], {'10': 'stop'})
        port.drop_stops.clear()
        second = channel.stop_all()
        self.assertEqual(second['observed_reset_ids'], list(range(7, 13)))
        self.assertEqual(second['ambiguous_ids'], [10])
        self.assertNotIn(10, second['confirmed_ids'])
        self.assertFalse(second['complete'])
        self.assert_one_stop_round(port.writes[:6])
        self.assert_one_stop_round(port.writes[6:])

    def test_partial_stop_write_is_never_repaired_by_another_mode_zero_frame(self):
        channel, port = self.make_channel()
        port.partial_stops = {10}
        first = channel.stop_all()
        self.assertEqual(first['ambiguous_ids'], [10])
        self.assertTrue(any('Partial write' in error for error in first['errors']))
        port.partial_stops.clear()
        second = channel.stop_all()
        self.assertEqual(second['ambiguous_ids'], [10])
        self.assertFalse(second['complete'])
        self.assertEqual(len(port.writes), 12)  # one attempt per axis per explicit call

    def test_partial_boundary_is_preserved_and_cannot_be_cleaned_into_success(self):
        channel, port = self.make_channel()
        channel.parser.feed(b'AT\x00')
        first = channel.stop_all()
        self.assertEqual(first['receive_boundary_evidence']['partial_hex'], '415400')
        self.assertTrue(first['sticky_boundary_uncertain'])
        self.assertEqual(first['confirmed_ids'], [])
        second = channel.stop_all()
        self.assertEqual(second['receive_boundary_evidence']['partial_hex'], '')
        self.assertEqual(second['observed_reset_ids'], list(range(7, 13)))
        self.assertEqual(second['confirmed_ids'], [])
        self.assertTrue(second['sticky_boundary_uncertain'])
        self.assertFalse(second['complete'])
        self.assertEqual(len(port.writes), 12)

    def test_prior_discarded_bytes_and_backlog_each_prevent_false_clean_boundary(self):
        for kind in ('discarded', 'backlog'):
            with self.subTest(kind=kind):
                channel, port = self.make_channel()
                if kind == 'discarded':
                    channel.parser.discarded_bytes = 2
                else:
                    port.in_waiting = 17
                first = channel.stop_all()
                evidence = first['receive_boundary_evidence']
                self.assertEqual(evidence['discarded_bytes'], 2 if kind == 'discarded' else 0)
                self.assertEqual(evidence['backlogged_bytes'], 17 if kind == 'backlog' else 0)
                self.assertFalse(first['complete'])
                second = channel.stop_all()
                self.assertTrue(second['sticky_boundary_uncertain'])
                self.assertEqual(second['confirmed_ids'], [])

    def test_broken_error_formatter_does_not_skip_later_stops(self):
        channel, port = self.make_channel()
        port.read_errors = {10: BrokenMessageError('receive failed')}
        result = channel.stop_all()
        self.assert_one_stop_round(port.writes)
        self.assertTrue(any('BrokenMessageError: receive failed' in error for error in result['errors']))
        self.assertFalse(result['complete'])
        self.assertEqual(result['ambiguous_ids'], [10])

    def test_event_budget_failure_still_attempts_every_stop_and_retains_uncertainty(self):
        channel, port = self.make_channel()
        channel.events = [{}]*2048
        first = channel.stop_all()
        self.assert_one_stop_round(port.writes)
        self.assertEqual(first['ambiguous_ids'], list(range(7, 13)))
        channel.events = []
        second = channel.stop_all()
        self.assertEqual(second['observed_reset_ids'], list(range(7, 13)))
        self.assertEqual(second['confirmed_ids'], [])
        self.assertFalse(second['complete'])

    def test_normal_stop_remains_complete_but_never_rearms_active_commands(self):
        channel, port = self.make_channel()
        result = channel.stop_all()
        self.assertTrue(result['complete'])
        self.assertEqual(result['ambiguous_ids'], [])
        self.assertEqual(result['confirmed_ids'], list(range(7, 13)))
        self.assertEqual(result['unresolved_state_steps'], {})
        self.assertFalse(result['sticky_boundary_uncertain'])
        with self.assertRaisesRegex(RuntimeError, 'Failed channel'):
            channel.exchange(10, 'enable')
        self.assert_one_stop_round(port.writes)

    def test_faulted_reset_frame_is_not_an_ack_and_still_stops_all_siblings(self):
        channel, port = self.make_channel()
        port.faults = {10: 1}
        first = channel.stop_all()
        self.assert_one_stop_round(port.writes)
        self.assertNotIn(10, first['observed_reset_ids'])
        self.assertNotIn(10, first['confirmed_ids'])
        self.assertFalse(first['complete'])
        port.faults.clear()
        second = channel.stop_all()
        self.assertEqual(second['ambiguous_ids'], [10])
        self.assertFalse(second['complete'])


if __name__ == '__main__':
    unittest.main()
