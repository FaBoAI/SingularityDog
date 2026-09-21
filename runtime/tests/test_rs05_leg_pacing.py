"""The provisional five-millisecond transport spacing is tested without hardware."""
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import read_request
from singularitydog_hw.rs05_joint_trial import check_feedback
from singularitydog_hw.rs05_leg_trial import LegTrialTransport, MIN_TX_INTERVAL_S
from singularitydog_hw.rs05_trial_protocol import (
    TrialPhase, Type2Feedback, enable_request, stop_request, watchdog_setup_request)
from test_rs05_joint_trial import FakeClock
from test_rs05_leg_transport import Serial, motion


class TimedSerial(Serial):
    def __init__(self, clock, *, failure=None, duration=.003):
        super().__init__(clock)
        self.failure, self.duration = failure, duration
        self.starts, self.finishes, self.read_starts = [], [], []

    def write(self, data):
        self.starts.append(self.clock())
        try:
            self.clock.wait(self.duration)
            if self.failure == 'exception' and len(self.starts) == 1:
                raise IOError('first write uncertain')
            count = super().write(data)
            if self.failure == 'partial' and len(self.starts) == 1:
                return count-1
            return count
        finally:
            self.finishes.append(self.clock())

    def read(self, size):
        self.read_starts.append(self.clock())
        return super().read(size)


class PacingTests(unittest.TestCase):
    def fixture(self, **kwargs):
        clock=FakeClock()
        port=TimedSerial(clock, **kwargs)
        return clock,port,LegTrialTransport(port,lambda _:None,ids=(1,2,3),wait=clock.wait)

    def assert_gaps(self, port):
        for before,after in zip(port.finishes,port.starts[1:]):
            self.assertGreaterEqual(after-before, MIN_TX_INTERVAL_S-1e-12)

    def test_every_frame_class_waits_from_previous_write_completion(self):
        clock,port,t=self.fixture()
        wires=[read_request(1),read_request(2,'position'),
               watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP,motor_id=3),
               enable_request(phase=TrialPhase.ENABLE,motor_id=1),motion(2,active=True),
               stop_request(phase=TrialPhase.STOP,motor_id=3)]
        with patch('singularitydog_hw.rs05_leg_trial.time.monotonic',clock):
            for wire in wires:t.send(wire)
        self.assertEqual(len(port.starts),6)
        self.assert_gaps(port)
        self.assertAlmostEqual(port.starts[1]-port.starts[0],.008)

    def test_early_wakeup_is_not_assumed_to_satisfy_interval(self):
        clock,port,t=self.fixture()
        waits=[]
        def early_once(seconds):
            waits.append(seconds)
            clock.wait(seconds/2 if len(waits)==1 else seconds)
        t.wait=early_once
        with patch('singularitydog_hw.rs05_leg_trial.time.monotonic',clock):
            t.send(read_request(1));t.send(read_request(2))
        self.assertEqual(len(waits),2)
        self.assert_gaps(port)

    def test_deadline_expiring_during_pacing_blocks_enable_and_motion(self):
        for wire in (enable_request(phase=TrialPhase.ENABLE,motor_id=2),motion(2,active=True)):
            with self.subTest(wire=wire.hex()):
                clock,port,t=self.fixture()
                with patch('singularitydog_hw.rs05_leg_trial.time.monotonic',clock):
                    t.send(read_request(1));t.active_deadline=clock()+.004
                    with self.assertRaisesRegex(RuntimeError,'deadline'):t.send(wire)
                self.assertEqual(len(port.starts),1)

    def test_feedback_aging_during_pacing_blocks_next_active_write(self):
        clock,port,t=self.fixture()
        with patch('singularitydog_hw.rs05_leg_trial.time.monotonic',clock):
            t.send(read_request(1))
            latest={i:(Type2Feedback(2,0,32767,0,0,0,30),clock()-.098) for i in (1,2,3)}
            def guard():
                for fb,when in latest.values():check_feedback(fb,0,when,clock())
            t.pre_send_guard=guard
            with self.assertRaisesRegex(RuntimeError,'Stale'):t.send(motion(2,active=True))
        self.assertEqual(len(port.starts),1)

    def test_latched_interrupt_or_fault_during_pacing_blocks_next_write(self):
        for cause in ('interrupt','fault'):
            with self.subTest(cause=cause):
                clock,port,t=self.fixture()
                interrupted=[]
                def check():
                    if interrupted:raise InterruptedError('operator')
                t.check_interrupt=check
                def wait(seconds):
                    clock.wait(seconds)
                    if cause=='interrupt':interrupted.append(True)
                    else:t.fault_latched='fault while pacing'
                t.wait=wait
                with patch('singularitydog_hw.rs05_leg_trial.time.monotonic',clock):
                    t.send(read_request(1))
                    with self.assertRaises((InterruptedError,RuntimeError)):t.send(motion(2,active=True))
                self.assertEqual(len(port.starts),1)

    def test_failed_and_partial_writes_still_space_all_remaining_stops(self):
        for failure in ('exception','partial'):
            with self.subTest(failure=failure):
                clock,port,t=self.fixture(failure=failure)
                t.active_deadline=clock()-.01
                t.check_interrupt=lambda:(_ for _ in ()).throw(InterruptedError('operator'))
                with patch('singularitydog_hw.rs05_leg_trial.time.monotonic',clock):
                    reports=t.stop_all((1,2,3))
                self.assertEqual(len(port.starts),3)
                self.assert_gaps(port)
                self.assertGreaterEqual(port.read_starts[0],port.finishes[-1])
                self.assertFalse(reports[1]['confirmed'])
                self.assertTrue(reports[2]['confirmed'] and reports[3]['confirmed'])

    def test_stop_ignores_interrupt_latched_while_it_paces(self):
        clock,port,t=self.fixture()
        interrupted=[]
        def wait(seconds):
            clock.wait(seconds);interrupted.append(True)
        t.wait=wait
        def check():
            if interrupted:raise InterruptedError('operator')
        t.check_interrupt=check
        t.active_deadline=clock()-.01
        with patch('singularitydog_hw.rs05_leg_trial.time.monotonic',clock):
            reports=t.stop_all((1,2,3))
        self.assertEqual(len(port.starts),3)
        self.assertTrue(all(r['confirmed'] for r in reports.values()))
        self.assert_gaps(port)


if __name__=='__main__':unittest.main()
