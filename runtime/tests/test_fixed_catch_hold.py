"""Terminal and deadline behavior; these tests never open motor hardware."""
import os
import pty
import unittest

from singularitydog_hw.fixed_catch_hold import FixedCatchExecution


class FixedCatchHoldTests(unittest.TestCase):
    def setUp(self):
        self.master,self.slave=pty.openpty()
        self.addCleanup(os.close,self.master)
        self.addCleanup(os.close,self.slave)
        self.now=1_000_000_000
        self.execution=FixedCatchExecution(self.slave,clock=lambda:self.now)
        self.addCleanup(self.execution.close)
        self.cancelled=[]
        self.execution.connect_cancel(lambda:self.cancelled.append(True))
        self.execution.on_start(self.now)

    def _step(self,seconds,phase='active'):
        self.now=1_000_000_000+int(seconds*1e9)
        self.execution.after_cycle_validated(self.now-19_000_000,self.now,phase)

    def test_cue_only_after_validated_active_hold_and_ack_is_timestamped(self):
        self._step(1.5,'starting')
        self.assertIsNone(self.execution.cue_ns)
        self._step(2.1)
        self.assertEqual(self.execution.cue_ns,self.now)
        os.write(self.master,b'\n')
        self.now+=20_000_000
        self.assertFalse(self.execution.before_cycle(self.now+20_000_000))
        self.assertIsNotNone(self.execution.ack_ns)
        self.assertGreater(self.execution.ack_ns,self.execution.cue_ns)
        self._step(26.1)
        self.assertTrue(self.execution.warned)
        self.assertEqual(sum(x['key']=='STOP_APPROACHING' for x in self.execution.events),1)

    def test_missing_ack_requests_early_normal_stop(self):
        self._step(2.1)
        self.assertFalse(self.execution.before_cycle(self.now+20_000_000))
        self.assertTrue(self.execution.before_cycle(1_000_000_000+8_000_000_000))
        self.assertEqual(self.cancelled,[])

    def test_operator_q_cancels_and_cue_is_not_repeated(self):
        self._step(2.1)
        self._step(2.5)
        self.assertEqual(sum(x['key']=='UPPER_SUPPORT_WITHDRAWAL_CUE' for x in self.execution.events),1)
        os.write(self.master,b'q\n')
        with self.assertRaisesRegex(RuntimeError,'Operator requested'):
            self.execution.before_cycle(self.now+20_000_000)
        self.assertTrue(self.cancelled)

    def test_non_tty_cannot_arm_cue(self):
        read,write=os.pipe()
        try:
            with self.assertRaisesRegex(ValueError,'visible local Jetson terminal'):
                FixedCatchExecution(read)
        finally:
            os.close(read);os.close(write)


if __name__=='__main__': unittest.main()
