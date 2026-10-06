"""Active timeout arithmetic regression with every transport syscall simulated."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tempfile
import unittest

EXP = Path(__file__).resolve().parents[1]/'experiments/native_active_transport_fresh_wait'
SOURCE = EXP.parent/'native_active_transport/transport.cpp'
LEGACY = EXP/'fixtures/transport-before-fresh-wait.cpp'
LEGACY_SHA256 = '9f61d07d08aa2c904cef5686716ee7f91c80903bfe81562d74f5e882cbbb01e2'
BEFORE = '        const uint64_t wait=wake>t?wake-t:0;\n'
AFTER = '''        // Boot checking may consume time after loop-entry t.
        // Keep the original absolute wake/deadline; never extend either one.
        const uint64_t before_wait=now();
        if(!before_wait||before_wait>=deadline)return fail("Active exchange deadline exceeded; no retry");
        const uint64_t wait=wake>before_wait?wake-before_wait:0;
'''


class NativeActiveTransportFreshWaitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if compiler is None:
            raise unittest.SkipTest('C++ compiler needed for simulated syscalls')
        original = LEGACY.read_bytes()
        if hashlib.sha256(original).hexdigest() != LEGACY_SHA256:
            raise AssertionError('Legacy fixture changed')
        if original.decode().count(BEFORE) != 1:
            raise AssertionError('Legacy wait block must be unique')
        if SOURCE.read_text() != original.decode().replace(BEFORE, AFTER):
            raise AssertionError('Only the reviewed timeout block may differ')
        cls.temporary = tempfile.TemporaryDirectory(prefix='active-fresh-wait-')
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.executables = {}
        for label, source in [('original', LEGACY), ('fresh', SOURCE)]:
            executable = Path(cls.temporary.name)/label
            subprocess.run([compiler, '-std=c++17', '-O2',
                f'-DSOURCE_FILE="{source}"', str(EXP/'fake_transport_harness.cpp'),
                '-o', str(executable)], check=True, capture_output=True, timeout=30)
            cls.executables[label] = executable

    def run_case(self, label, mode):
        result = subprocess.run([str(self.executables[label]), str(mode)],
            check=True, capture_output=True, text=True, timeout=2)
        return json.loads(result.stdout)

    def test_exact_single_block_preserves_abi_stop_and_matching(self):
        self.assertEqual(SOURCE.read_text().replace(AFTER, BEFORE), LEGACY.read_text())
        self.assertEqual(SOURCE.read_text().count(AFTER), 1)

    def test_delayed_boot_is_subtracted_without_lowering_request_gap(self):
        old, new = (self.run_case(label, 0) for label in ('original', 'fresh'))
        for result in (old, new):
            self.assertEqual((result['status'], result['writes'], result['reads']), (0, 2, 2))
        self.assertEqual(old['first_write_ns'], new['first_write_ns'])
        # The separate mandatory pre-write boot check still costs 2ms.
        self.assertEqual(new['second_write_ns']-new['first_write_ns'], 7_000_000)
        self.assertEqual(old['second_write_ns']-old['first_write_ns'], 9_000_000)
        self.assertEqual(old['deadline_ns'], new['deadline_ns'])
        for key in ('tx0_hex', 'tx1_hex'):
            self.assertEqual(old[key], new[key])

    def test_no_reply_stops_at_same_deadline_and_preserves_ambiguity(self):
        old, new = (self.run_case(label, 1) for label in ('original', 'fresh'))
        self.assertEqual(new['end_ns'], new['deadline_ns'])
        self.assertEqual(old['end_ns']-old['deadline_ns'], 2_000_000)
        for result in (old, new):
            self.assertEqual(result['status'], -1)
            self.assertEqual(result['writes'], 1)
            self.assertTrue(result['poisoned'])
            self.assertEqual(result['ambiguous_mask'], 1)
            self.assertEqual((result['retry_status'], result['extra_retry_writes']), (-1, 0))

    def test_eintr_rechecks_expired_original_deadline_without_another_wait(self):
        old, new = (self.run_case(label, 2) for label in ('original', 'fresh'))
        self.assertEqual(old['deadline_ns'], new['deadline_ns'])
        # The interrupted wait and next boot read themselves consume 0.1ms beyond
        # the deadline. The correction does not add a further stale relative wait.
        self.assertEqual(new['end_ns']-new['deadline_ns'], 100_000)
        self.assertLess(new['exchange_waits'], old['exchange_waits'])
        for result in (old, new):
            self.assertEqual((result['status'], result['writes']), (-1, 1))
            self.assertEqual((result['ambiguous_mask'], result['extra_retry_writes']), (1, 0))

    def test_cancel_before_exchange_sends_nothing(self):
        for label in ('original', 'fresh'):
            result = self.run_case(label, 3)
            self.assertEqual((result['status'], result['writes']), (-1, 0))
            self.assertEqual(result['error'], 'Cancelled before active exchange')

    def test_cancel_after_first_write_retains_pending_axis(self):
        for label in ('original', 'fresh'):
            result = self.run_case(label, 9)
            self.assertEqual((result['status'], result['writes']), (-1, 1))
            self.assertEqual(result['error'], 'Cancelled active exchange')
            self.assertEqual((result['ambiguous_mask'], result['extra_retry_writes']), (1, 0))

    def test_partial_type1_write_is_not_retried_and_stop_remains_ambiguous(self):
        for label in ('original', 'fresh'):
            result = self.run_case(label, 4)
            self.assertEqual((result['status'], result['writes']), (-1, 1))
            self.assertEqual(result['error'], 'Partial/failed active write; no retransmission')
            self.assertTrue(result['poisoned'])
            self.assertEqual((result['ambiguous_mask'], result['extra_retry_writes']), (1, 0))
            self.assertEqual(result['emergency_attempted'], 63)
            self.assertEqual(result['emergency_confirmed'], 62)
            self.assertEqual(result['emergency_ambiguous'], 1)
            self.assertEqual(result['emergency_status'], 1)

    def test_no_boot_delay_preserves_stop_and_type1_results_and_packets(self):
        for mode in (5, 10):
            old, new = (self.run_case(label, mode) for label in ('original', 'fresh'))
            self.assertEqual(old, new)
            self.assertEqual(new['status'], 0)
            self.assertFalse(new['poisoned'])
            self.assertEqual(new['second_write_ns']-new['first_write_ns'], 5_000_000)

    def test_eintr_limit_unchanged(self):
        for label in ('original', 'fresh'):
            result = self.run_case(label, 6)
            self.assertEqual((result['status'], result['writes'], result['exchange_waits']), (-1, 0, 33))
            self.assertEqual(result['error'], 'Active pselect failed')

    def test_expired_during_boot_fails_before_another_wait_or_write(self):
        result = self.run_case('fresh', 7)
        self.assertEqual((result['status'], result['writes'], result['exchange_waits']), (-1, 0, 0))
        self.assertTrue(result['poisoned'])
        self.assertEqual(result['ambiguous_mask'], 0)
        self.assertEqual(result['error'], 'Active exchange deadline exceeded; no retry')

    def test_invalid_type1_position_and_torque_rejected_before_wait(self):
        for label in ('original', 'fresh'):
            for mode in (8, 11):
                result = self.run_case(label, mode)
                self.assertEqual((result['status'], result['writes'], result['exchange_waits']), (-1, 0, 0))
                self.assertEqual(result['error'], 'Disallowed/noncanonical/out-of-bounds active command')

    def test_send_only_stays_forbidden(self):
        for label in ('original', 'fresh'):
            result = self.run_case(label, 12)
            self.assertEqual((result['status'], result['writes'], result['exchange_waits']), (-1, 0, 0))
            self.assertEqual(result['error'], 'Invalid active exchange arguments/FD binding')


if __name__ == '__main__':
    unittest.main()
