"""Per-thread diagnostic slack contracts; all prctl operations are mocked."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import errno
import threading
import types
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import thread_timer_slack as slack


class FakePrctl:
    """Parent readbacks and inherited worker values, with no OS setting changes."""
    def __init__(self, *, original=73_000, inherited=1_000, parent_readbacks=(), fail_set_at=None):
        self.parent_tid = threading.get_native_id()
        self.current = original
        self.inherited = inherited
        self.parent_readbacks = deque(parent_readbacks)
        self.fail_set_at = fail_set_at
        self.calls = []
        self.set_count = 0
        self.lock = threading.Lock()

    def get(self):
        tid = threading.get_native_id()
        with self.lock:
            self.calls.append(('get', tid))
            if tid != self.parent_tid:
                return self.inherited
            if self.parent_readbacks:
                value = self.parent_readbacks.popleft()
                if isinstance(value, BaseException):
                    raise value
                return value
            return self.current

    def set(self, value):
        tid = threading.get_native_id()
        with self.lock:
            self.calls.append(('set', tid, value))
            self.set_count += 1
            if self.set_count == self.fail_set_at:
                raise slack.TimerSlackError('Injected prctl set failure')
            if tid != self.parent_tid:
                raise AssertionError('Worker attempted to change timer slack')
            self.current = value


class ThreadTimerSlackTests(unittest.TestCase):
    def test_default_is_noop_even_off_linux_and_never_loads_libc(self):
        with patch.object(slack.sys, 'platform', 'darwin'), \
                patch.object(slack, '_load_prctl') as loader, \
                patch.object(slack, 'require_supported_platform') as supported:
            scope = slack.TimerSlack()
            with scope:
                scope.worker_initializer()
                scope.verify_workers()
        loader.assert_not_called(); supported.assert_not_called()
        self.assertFalse(scope.report['enabled'])
        self.assertIsNone(scope.report['requested_ns'])
        self.assertEqual(scope.report['workers'], [])
        self.assertTrue(all(value is None for value in scope.report['parent'].values()))

    def test_opt_in_off_linux_fails_without_libc_or_mutation(self):
        scope = slack.TimerSlack(1_000)
        with patch.object(slack.sys, 'platform', 'darwin'), patch.object(slack, '_load_prctl') as loader:
            with self.assertRaisesRegex(slack.TimerSlackError, 'requires Linux'):
                with scope:
                    self.fail('Unsupported scope entered')
        loader.assert_not_called()
        self.assertIsNone(scope.report['parent']['original_ns'])
        self.assertIsNone(scope.report['parent']['restored'])
        self.assertIn('requires Linux', scope.report['errors'][0])

    def test_requested_values_are_strict_and_bounded_before_loading(self):
        with patch.object(slack, '_load_prctl') as loader:
            for value in (0, 999, 1_001, 49_999, 50_001, -1, True, False, 1_000., '1000'):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    slack.TimerSlack(value)
        loader.assert_not_called()

    def test_normal_scope_restores_exact_original_and_verifies_three_distinct_workers(self):
        for requested in slack.CHOICES_NS:
            with self.subTest(requested=requested):
                backend = FakePrctl(inherited=requested)
                scope = slack.TimerSlack(requested)
                barrier = threading.Barrier(3)
                with patch.object(slack.sys, 'platform', 'linux'), \
                        patch.object(slack, '_load_prctl', return_value=backend) as loader:
                    with scope:
                        self.assertEqual(backend.current, requested)
                        with ThreadPoolExecutor(max_workers=3, initializer=scope.worker_initializer) as pool:
                            futures = [pool.submit(barrier.wait, .5) for _ in range(3)]
                            for future in futures:
                                future.result(timeout=1)
                        scope.verify_workers()
                loader.assert_called_once_with()
                parent = scope.report['parent']
                self.assertEqual(parent, {'native_tid': backend.parent_tid, 'original_ns': 73_000,
                                         'during_ns': requested, 'after_ns': 73_000, 'restored': True})
                self.assertEqual(backend.current, 73_000)
                self.assertEqual([row[2] for row in backend.calls if row[0] == 'set'], [requested, 73_000])
                self.assertTrue(all(row[1] == backend.parent_tid for row in backend.calls if row[0] == 'set'))
                self.assertEqual(len(scope.report['workers']), 3)
                self.assertEqual(len({row['native_tid'] for row in scope.report['workers']}), 3)
                self.assertTrue(all(row['current_ns'] == requested and row['verified'] for row in scope.report['workers']))
                self.assertTrue(scope.report['worker_verification_complete'])

    def test_collection_exception_still_restores_exact_original(self):
        backend = FakePrctl()
        scope = slack.TimerSlack(1_000)
        with patch.object(slack.sys, 'platform', 'linux'), patch.object(slack, '_load_prctl', return_value=backend):
            with self.assertRaisesRegex(InterruptedError, 'cancelled'):
                with scope:
                    raise InterruptedError('cancelled')
        self.assertTrue(scope.report['parent']['restored'])
        self.assertEqual(backend.current, 73_000)
        self.assertEqual([row[2] for row in backend.calls if row[0] == 'set'], [1_000, 73_000])

    def test_apply_or_parent_readback_failure_restores_before_propagating(self):
        fixtures = (
            (FakePrctl(fail_set_at=1), 'set failure'),
            (FakePrctl(parent_readbacks=(73_000, 50_000, 73_000)), 'readback mismatch'),
            (FakePrctl(parent_readbacks=(73_000, slack.TimerSlackError('get failure'), 73_000)), 'get failure'),
        )
        for backend, message in fixtures:
            with self.subTest(message=message):
                scope = slack.TimerSlack(1_000)
                with patch.object(slack.sys, 'platform', 'linux'), patch.object(slack, '_load_prctl', return_value=backend):
                    with self.assertRaisesRegex(slack.TimerSlackError, message):
                        with scope:
                            self.fail('Unverified setting entered')
                self.assertTrue(scope.report['parent']['restored'])
                self.assertEqual(backend.current, 73_000)
                self.assertEqual([row[2] for row in backend.calls if row[0] == 'set'], [1_000, 73_000])

    def test_restore_failure_or_unconfirmed_readback_is_never_success(self):
        fixtures = (
            (FakePrctl(fail_set_at=2), 'set failure'),
            (FakePrctl(parent_readbacks=(73_000, 1_000, 50_000)), 'restoration unconfirmed'),
            (FakePrctl(parent_readbacks=(73_000, 1_000, slack.TimerSlackError('get failure'))), 'get failure'),
        )
        for backend, message in fixtures:
            with self.subTest(message=message):
                scope = slack.TimerSlack(1_000)
                with patch.object(slack.sys, 'platform', 'linux'), patch.object(slack, '_load_prctl', return_value=backend):
                    with self.assertRaisesRegex(slack.TimerSlackError, message):
                        with scope:
                            pass
                self.assertFalse(scope.report['parent']['restored'])
                self.assertIn(message, scope.report['errors'][-1])
                self.assertNotIn(0, [row[2] for row in backend.calls if row[0] == 'set'])

    def test_unreadable_or_zero_original_never_writes_zero_or_any_setting(self):
        for original in (0, slack.TimerSlackError('original read failed')):
            with self.subTest(original=original):
                backend = FakePrctl(parent_readbacks=(original,))
                scope = slack.TimerSlack(1_000)
                with patch.object(slack.sys, 'platform', 'linux'), patch.object(slack, '_load_prctl', return_value=backend):
                    with self.assertRaises(slack.TimerSlackError):
                        with scope:
                            self.fail('Unknown original entered')
                self.assertEqual([row for row in backend.calls if row[0] == 'set'], [])
                self.assertIsNone(scope.report['parent']['restored'])

    def test_worker_mismatch_is_recorded_and_parent_restored(self):
        backend = FakePrctl(inherited=50_000)
        scope = slack.TimerSlack(1_000)
        errors = []
        def start():
            try:
                scope.worker_initializer()
            except slack.TimerSlackError as error:
                errors.append(error)
        with patch.object(slack.sys, 'platform', 'linux'), patch.object(slack, '_load_prctl', return_value=backend):
            with scope:
                worker = threading.Thread(target=start)
                worker.start(); worker.join(timeout=1)
                self.assertFalse(worker.is_alive())
                with self.assertRaisesRegex(slack.TimerSlackError, 'Three distinct'):
                    scope.verify_workers()
        self.assertEqual(len(errors), 1)
        self.assertIn('readback mismatch', str(errors[0]))
        self.assertEqual(scope.report['workers'][0]['current_ns'], 50_000)
        self.assertFalse(scope.report['workers'][0]['verified'])
        self.assertTrue(scope.report['parent']['restored'])

    def test_libc_binding_uses_unsigned_long_arguments_and_checks_errno(self):
        function = Mock(side_effect=(50_000, 0, -1, 0))
        ctypes = types.SimpleNamespace(CDLL=Mock(return_value=types.SimpleNamespace(prctl=function)),
            c_int=object(), c_ulong=object(), get_errno=Mock(return_value=errno.EPERM))
        with patch.dict('sys.modules', {'ctypes': ctypes}):
            backend = slack._load_prctl()
            self.assertEqual(backend.get(), 50_000)
            backend.set(1_000)
            with self.assertRaisesRegex(slack.TimerSlackError, 'Operation not permitted'):
                backend.get()
            with self.assertRaisesRegex(slack.TimerSlackError, 'positive'):
                backend.set(0)
        ctypes.CDLL.assert_called_once_with(None, use_errno=True)
        self.assertEqual(function.argtypes, [ctypes.c_int]+[ctypes.c_ulong]*4)
        self.assertIs(function.restype, ctypes.c_int)
        self.assertEqual([call.args for call in function.call_args_list],
                         [(30, 0, 0, 0, 0), (29, 1_000, 0, 0, 0), (30, 0, 0, 0, 0)])


if __name__ == '__main__':
    unittest.main()
