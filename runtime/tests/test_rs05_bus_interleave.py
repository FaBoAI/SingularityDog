"""Arrival processing between paced Type1 writes; entirely fake serial I/O."""
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_bus_transport import BusTrialTransport, EXCHANGE_TIMEOUT_S
from singularitydog_hw.rs05_trial_protocol import TrialPhase, enable_request
from test_rs05_bus_transport import BUSES, CLOCK_PATCH, BusFixture
from test_rs05_joint_trial import FakeClock
from test_rs05_leg_pacing import TimedSerial
from test_rs05_leg_transport import Serial, feedback, motion, wire


class ArrivingSerial(TimedSerial):
    """Deliver each reply 3ms after its write, during the 5ms TX wait."""
    def __init__(self, clock):
        super().__init__(clock, duration=.001)
        self.arrivals = []

    def write(self, data):
        count = super().write(data)
        self.arrivals.append((self.clock()+.003, self.rx))
        self.rx = b''
        return count

    @property
    def in_waiting(self):
        ready = [item for item in self.arrivals if item[0] <= self.clock()]
        self.arrivals = [item for item in self.arrivals if item[0] > self.clock()]
        self.rx += b''.join(item[1] for item in ready)
        return len(self.rx)


class InterleavedBusTests(BusFixture):
    def run_batch(self, transport):
        return transport.feedback_many([motion(mid, active=True) for mid in transport.ids],
                                       transport.ids)

    def test_reply_arriving_during_pacing_reaches_guard_before_next_write(self):
        for interleave in (False, True):
            clock = FakeClock()
            port = ArrivingSerial(clock)
            t = BusTrialTransport(port, lambda _: None, ids=BUSES['front'], wait=clock.wait,
                                  interleave_feedback=interleave)
            published, seen = {}, []
            t.feedback_guard = lambda value, when, mid: published.update({mid: when})
            t.pre_send_guard = lambda: seen.append(tuple(sorted(published)))
            with patch(CLOCK_PATCH, clock):
                found = self.run_batch(t)
            self.assertEqual(set(found), set(t.ids))
            self.assertEqual(seen, [tuple(range(1, mid)) if interleave else ()
                                    for mid in t.ids])
            self.assertEqual([f.destination for f in port.sent], list(t.ids))
            self.assert_gaps(port)

    def test_both_buses_preserve_completion_pacing_and_actual_rx_stamp(self):
        for bus, ids in BUSES.items():
            clock, port, t = self.fixture(bus, timed=True)
            t.interleave_feedback = True
            publications = []
            t.feedback_guard = lambda value, when, mid: publications.append((mid, when))
            with patch(CLOCK_PATCH, clock):
                found = self.run_batch(t)
            self.assertEqual([mid for mid, _ in publications], list(ids))
            self.assertEqual([when for _, when in publications], [found[mid][1] for mid in ids])
            self.assertEqual(len(port.sent), 6)
            self.assert_gaps(port)
            for index, mid in enumerate(ids):
                self.assertAlmostEqual(found[mid][1], port.finishes[index]+.001)

    def test_unsent_duplicate_wrong_host_and_non_type2_abort_before_next_write(self):
        wrong_host = ATParser().feed(feedback(1))[0]
        cases = (feedback(2), feedback(1)+feedback(1),
                 wire(wrong_host.can_id ^ 1, wrong_host.data),
                 wire((17 << 24) | (1 << 8) | 0xFD, bytes(8)),
                 wire((21 << 24) | (1 << 8) | 0xFD, bytes(8)),
                 feedback(7), b'garbage'+feedback(1))
        for reply in cases:
            clock, port, t = self.fixture(responder=lambda _: reply)
            t.interleave_feedback = True
            published = []
            t.feedback_guard = lambda value, when, mid: published.append(mid)
            with self.subTest(reply=reply.hex()), patch(CLOCK_PATCH, clock):
                with self.assertRaises(RuntimeError):
                    self.run_batch(t)
                self.assertEqual(len(port.sent), 1)
                self.assertEqual(published, [1] if reply == feedback(1)+feedback(1) else [])
                self.assertIsNotNone(t.fault_latched)
                port.responder = lambda frame: feedback(frame.destination, mode=0)
                reports = t.stop_all()
            self.assertEqual([f.destination for f in port.sent if f.kind == 4], list(t.ids))
            # Malformed input may deliberately prevent STOP confirmation, but
            # never prevents attempting all six physical STOP writes.
            self.assertEqual(set(reports), set(t.ids))

    def test_missing_reply_has_original_last_write_deadline_and_no_retry(self):
        clock, port, t = self.fixture(timed=True,
            responder=lambda frame: b'' if frame.destination == 3 else feedback(frame.destination))
        t.interleave_feedback = True
        with patch(CLOCK_PATCH, clock):
            with self.assertRaises(TimeoutError) as caught:
                self.run_batch(t)
            result = caught.exception.diagnostics
            self.assertEqual(result['missing_ids'], [3])
            self.assertEqual(result['received_ids'], [1, 2, 4, 5, 6])
            self.assertEqual(result['batch_completed_monotonic_s'], port.finishes[-1])
            self.assertAlmostEqual(result['effective_deadline_monotonic_s'],
                                   port.finishes[-1]+EXCHANGE_TIMEOUT_S)
            self.assertEqual(len(port.sent), 6)
            port.responder = lambda frame: feedback(frame.destination, mode=0)
            self.assertTrue(all(row['confirmed'] for row in t.stop_all().values()))
        self.assert_gaps(port)

    def test_partial_frame_blocks_next_command_but_complete_fragments_work(self):
        clock, port, t = self.fixture(responder=lambda _: b'AT')
        t.interleave_feedback = True
        with patch(CLOCK_PATCH, clock), self.assertRaisesRegex(RuntimeError, 'Partial'):
            self.run_batch(t)
        self.assertEqual(len(port.sent), 1)
        clock, port, t = self.fixture()
        t.interleave_feedback = True
        original_read = port.read
        port.read = lambda count: original_read(min(count, 3))
        with patch(CLOCK_PATCH, clock):
            self.assertEqual(set(self.run_batch(t)), set(t.ids))

    def test_slow_logger_does_not_refresh_received_timestamp(self):
        clock, port, t = self.fixture()
        t.interleave_feedback = True
        t.emit = lambda row: clock.wait(.11) if row['kind'] == 'can_rx_bytes' else None
        with patch(CLOCK_PATCH, clock), self.assertRaisesRegex(RuntimeError, 'stale'):
            self.run_batch(t)
        self.assertEqual(len(port.sent), 1)
        self.assertFalse(t.latest)

    def test_hard_deadline_and_feedback_guard_failure_prevent_further_writes(self):
        for cause in ('deadline', 'guard'):
            clock, port, t = self.fixture(timed=True)
            t.interleave_feedback = True
            if cause == 'deadline':
                t.active_deadline = clock()+.006
            else:
                def reject(*_):
                    raise RuntimeError('injected feedback guard rejection')
                t.feedback_guard = reject
            with self.subTest(cause=cause), patch(CLOCK_PATCH, clock):
                with self.assertRaises(RuntimeError):
                    self.run_batch(t)
                self.assertEqual(len(port.sent), 1)
                port.responder = lambda frame: feedback(frame.destination, mode=0)
                self.assertTrue(all(row['confirmed'] for row in t.stop_all().values()))
            self.assertEqual([f.destination for f in port.sent if f.kind == 4], list(t.ids))

    def test_opt_in_does_not_change_two_wire_enable_neutral_sequence(self):
        clock, port, t = self.fixture()
        t.interleave_feedback = True
        command = enable_request(phase=TrialPhase.ENABLE, motor_id=1)
        with patch(CLOCK_PATCH, clock):
            found = t.feedback_many([command, motion(1)], (1,))
        self.assertEqual(set(found), {1})
        self.assertEqual([f.kind for f in port.sent], [3, 1])

    def test_constructor_rejects_truthy_non_boolean_opt_in(self):
        for bad in (1, 'true', None):
            clock = FakeClock()
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                BusTrialTransport(Serial(clock), lambda _: None, ids=BUSES['front'],
                                  interleave_feedback=bad)


if __name__ == '__main__':
    unittest.main()
