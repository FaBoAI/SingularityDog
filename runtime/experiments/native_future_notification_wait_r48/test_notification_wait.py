"""File-only synthetic Futures/pipes; C++ is compiled locally, no device I/O."""
from concurrent.futures import Future
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

if __package__:
    from . import candidate as c
    from . import baseline_waiter as baseline
else:
    import candidate as c
    import baseline_waiter as baseline


def ready(value=1):
    f = Future(); f.set_result(value); return f


class NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='notification-file-only-',
                                                     dir=Path(__file__).parent)
        directory = Path(cls.directory.name).resolve()
        raw = (Path(__file__).parent / 'notify_wait.cpp').read_bytes()
        (directory / 'notify_wait.cpp').write_bytes(raw)
        binary = directory / 'notification_wait.so'
        result = subprocess.run(['c++', '-std=c++17', '-O2', '-shared', '-fPIC',
                                 str(directory / 'notify_wait.cpp'), '-o', str(binary)],
                                capture_output=True, timeout=30)
        if result.returncode:
            raise RuntimeError(result.stderr.decode())
        record = dict(schema='private.notification-wait-build.v1',
                      source_sha256=hashlib.sha256(raw).hexdigest(),
                      binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
        (directory / 'build-record.json').write_text(json.dumps(record))
        cls.library = c.load_library(binary)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.cancel_read, self.cancel_write = os.pipe()
        self.scope = c.NotificationScope(self.library)
        self.scope.start(self.cancel_read)
        self.threads = []

    def tearDown(self):
        for thread in self.threads:
            thread.join(2)
            self.assertFalse(thread.is_alive(), 'synthetic writer not reaped')
        self.scope.close()
        for fd in (self.cancel_read, self.cancel_write):
            if fd is not None:
                os.close(fd)

    def later(self, delay, callback):
        def run():
            time.sleep(delay); callback()
        thread = threading.Thread(target=run)
        self.threads.append(thread); thread.start()

    def test_completed_at_registration_notifies_before_tick(self):
        self.scope.register((ready(), ready()))
        start = time.monotonic_ns()
        result = self.scope.wait(start + 100_000_000, start + 500_000_000)
        self.assertEqual(result['kind'], 'NOTIFIED')
        self.assertLess(result['actual_ns'], start + 100_000_000)
        self.assertEqual(self.scope.statistics()['notifications'], 2)

    def test_future_completion_wakes_incomplete_pair_without_claiming_complete(self):
        one, two = Future(), Future()
        self.scope.register((one, two))
        self.later(.005, lambda: one.set_result(1))
        start = time.monotonic_ns()
        result = self.scope.wait(start + 100_000_000, start + 500_000_000)
        self.assertEqual(result['kind'], 'NOTIFIED')
        self.assertLess(result['actual_ns'], start + 100_000_000)
        self.assertFalse(two.done())

    def test_join_waits_for_both_and_keeps_guards(self):
        one, two = Future(), Future()
        self.later(.004, lambda: one.set_result(1))
        self.later(.008, lambda: two.set_result(2))
        checks = []
        result = c.await_ready({'front': one, 'rear': two}, None, scope=self.scope,
                               phase='Proxy output', deadline_ns=time.monotonic_ns()+500_000_000,
                               check=lambda: checks.append(1))
        self.assertTrue(one.done() and two.done())
        self.assertGreaterEqual(result['notification_returns'], 1)
        self.assertGreater(len(checks), 2)
        self.assertFalse(result['active_output_eligible'])

    def test_ready_error_precedes_unfinished_owner(self):
        one, two = Future(), Future(); error = ValueError('original owner failure')
        one.set_exception(error)
        with self.assertRaises(ValueError) as raised:
            c.await_ready({'front': one, 'rear': two}, None, scope=self.scope,
                          phase='Voltage', deadline_ns=time.monotonic_ns()+500_000_000)
        self.assertIs(raised.exception, error)
        self.assertFalse(two.done())

    def test_cancelled_future_does_not_wait_for_other(self):
        one, two = Future(), Future(); one.cancel()
        with self.assertRaisesRegex(RuntimeError, 'owner future cancelled'):
            c.await_ready({'front': one, 'rear': two}, None, scope=self.scope,
                          phase='Voltage', deadline_ns=time.monotonic_ns()+500_000_000)

    def test_cancel_fd_wins_over_simultaneous_notification(self):
        self.scope.register((ready(), ready()))
        os.write(self.cancel_write, b'!')
        now = time.monotonic_ns()
        with self.assertRaisesRegex(c.WaitError, 'cancelled'):
            self.scope.wait(now+100_000_000, now+500_000_000)

    def test_original_cancel_fd_closed_still_uses_owned_dup(self):
        self.scope.register((Future(), Future()))
        os.close(self.cancel_read); self.cancel_read = None
        os.write(self.cancel_write, b'!')
        now = time.monotonic_ns()
        with self.assertRaisesRegex(c.WaitError, 'cancelled'):
            self.scope.wait(now+100_000_000, now+500_000_000)

    def test_notification_pipe_full_still_wakes_and_drains(self):
        one, two = Future(), Future(); self.scope.register((one, two))
        filled = 0
        for _ in range(1024):
            try: filled += os.write(self.scope._write_fd, bytes(4096))
            except BlockingIOError: break
        else: self.fail('nonblocking pipe did not saturate within finite cap')
        self.assertGreater(filled, 0)
        one.set_result(1)
        self.assertEqual(self.scope.statistics()['saturated'], 1)
        now = time.monotonic_ns()
        self.assertEqual(self.scope.wait(now+100_000_000, now+500_000_000)['kind'], 'NOTIFIED')
        with self.assertRaises(BlockingIOError): os.read(self.scope._read_fd, 1)

    def test_drained_then_later_callback_is_not_lost(self):
        one, two = Future(), Future(); self.scope.register((one, two))
        one.set_result(1); now = time.monotonic_ns()
        self.assertEqual(self.scope.wait(now+100_000_000, now+500_000_000)['kind'], 'NOTIFIED')
        self.later(.005, lambda: two.set_result(2)); now = time.monotonic_ns()
        self.assertEqual(self.scope.wait(now+100_000_000, now+500_000_000)['kind'], 'NOTIFIED')

    def test_callback_after_close_never_writes_reused_fd(self):
        one, two = Future(), Future(); self.scope.register((one, two))
        old = self.scope._write_fd; self.scope.close()
        read_fd, write_fd = os.pipe(); os.set_blocking(read_fd, False)
        duplicated = old != write_fd
        try:
            if duplicated: os.dup2(write_fd, old)
            one.set_result(1)
            with self.assertRaises(BlockingIOError): os.read(read_fd, 1)
            self.assertEqual(self.scope.statistics()['late_callbacks_skipped'], 1)
        finally:
            if duplicated and old not in (read_fd, write_fd): os.close(old)
            os.close(read_fd); os.close(write_fd)

    def test_close_callback_race_retains_fds_until_writer_returns(self):
        one, two = Future(), Future(); self.scope.register((one, two))
        entered, release = threading.Event(), threading.Event()
        real_write = os.write
        def blocked_write(fd, value):
            entered.set()
            if not release.wait(1): raise RuntimeError('fixture writer release timeout')
            return real_write(fd, value)
        with patch.object(c.os, 'write', side_effect=blocked_write):
            thread = threading.Thread(target=lambda: one.set_result(1))
            self.threads.append(thread); thread.start(); self.assertTrue(entered.wait(1))
            try:
                with self.assertRaises(c.CloseBusy): self.scope.close()
                self.assertIsNotNone(self.scope._write_fd)
                self.assertFalse(self.scope.statistics()['cleanup_complete'])
            finally: release.set(); thread.join(1)
        self.scope.close(); self.assertTrue(self.scope.statistics()['cleanup_complete'])

    def test_contended_callback_uses_tick_fallback_not_false_readiness(self):
        one, two = Future(), Future(); self.scope.register((one, two))
        with self.scope._lock: one.set_result(1)
        self.assertEqual(self.scope.statistics()['contended_fallback'], 1)
        now = time.monotonic_ns()
        result = self.scope.wait(now+1_000_000, now+500_000_000)
        self.assertEqual(result['kind'], 'TICK'); self.assertFalse(two.done())

    def test_expired_hard_deadline_not_success(self):
        self.scope.register((ready(), ready()))
        now = time.monotonic_ns()
        with self.assertRaises(TimeoutError): self.scope.wait(now-100, now-1)

    def test_no_notification_returns_actual_tick(self):
        self.scope.register((Future(), Future())); now = time.monotonic_ns()
        result = self.scope.wait(now+1_000_000, now+500_000_000)
        self.assertEqual(result['kind'], 'TICK')
        self.assertGreaterEqual(result['actual_ns'], now+1_000_000)

    def test_write_end_closed_is_fatal_not_ready(self):
        self.scope.register((Future(), Future()))
        os.close(self.scope._write_fd); self.scope._write_fd = None
        now = time.monotonic_ns()
        with self.assertRaisesRegex(c.WaitError, 'writer closed'):
            self.scope.wait(now+100_000_000, now+500_000_000)

    def test_native_regular_and_closed_fd_rejected(self):
        self.scope.register((Future(), Future()))
        with tempfile.TemporaryFile() as file:
            error = C.create_string_buffer(256); value = C.c_uint64(); now = time.monotonic_ns()
            result = self.library.nw_wait(file.fileno(), self.scope._cancel_fd,
                                          now+100_000_000, now+500_000_000,
                                          C.byref(value), error, 256)
            self.assertEqual(result, -1); self.assertEqual(value.value, 0)
        result = self.library.nw_wait(-1, self.scope._cancel_fd, now+100_000_000,
                                      now+500_000_000, C.byref(value), error, 256)
        self.assertEqual(result, -1)

    def test_nonowner_and_reentrant_wait_rejected(self):
        self.scope.register((Future(), Future())); errors = []
        def other():
            try: self.scope.wait(1, 2)
            except BaseException as error: errors.append(error)
        thread = threading.Thread(target=other); thread.start(); thread.join(1)
        self.assertEqual(len(errors), 1); self.assertIsInstance(errors[0], c.WaitError)
        self.scope._busy.acquire()
        try:
            with self.assertRaisesRegex(c.WaitError, 'Reentrant'): self.scope.wait(1, 2)
        finally: self.scope._busy.release()

    def test_duplicate_registration_or_future_rejected(self):
        f = Future()
        with self.assertRaises(ValueError): self.scope.register((f, f))
        self.scope.register((f, Future()))
        with self.assertRaises(ValueError): self.scope.register((Future(), Future()))
        for tick, hard in ((True, 5), (1, True), (3, 2), (0, 1)):
            with self.assertRaises(ValueError): self.scope.wait(tick, hard)

    def test_build_binary_and_source_pins_checked_before_loading(self):
        directory = Path(self.directory.name).resolve()
        original = (directory/'build-record.json').read_bytes()
        record = json.loads(original); record['binary_sha256'] = '0'*64
        try:
            (directory/'build-record.json').write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, 'pin mismatch'):
                c.load_library(directory/'notification_wait.so')
        finally: (directory/'build-record.json').write_bytes(original)

    def test_delayed_callback_does_not_prevent_actual_future_readiness(self):
        one, two = Future(), Future(); release = threading.Event(); entered = threading.Event()
        original_notify = self.scope._callback_fn
        def delayed_notify(future):
            if future is one:
                entered.set()
                if not release.wait(1): raise RuntimeError('fixture callback release timeout')
            original_notify(future)
        self.scope._callback_fn = delayed_notify
        self.later(.003, lambda: one.set_result(1))
        self.later(.006, lambda: two.set_result(2))
        try:
            result = c.await_ready({'front':one,'rear':two},None,scope=self.scope,
                                  phase='Voltage',deadline_ns=time.monotonic_ns()+500_000_000)
            self.assertTrue(entered.is_set()); self.assertTrue(one.done() and two.done())
            self.assertFalse(result['active_output_eligible'])
            # A Future becomes done before its callbacks return. Keep the
            # first deliberately delayed, but settle the ordinary second
            # worker before testing close followed by the first late callback.
            self.threads[-1].join(1)
            self.assertFalse(self.threads[-1].is_alive())
            self.scope.close()
        finally: release.set()
        for thread in self.threads: thread.join(1)
        self.assertEqual(self.scope.statistics()['late_callbacks_skipped'],1)

    def test_callback_broken_pipe_reports_failure(self):
        one,two=Future(),Future();self.scope.register((one,two))
        os.close(self.scope._read_fd);self.scope._read_fd=None
        one.set_result(1)
        with self.assertRaisesRegex(c.WaitError,'callback failed') as raised:
            self.scope.check_callback()
        self.assertIsInstance(raised.exception.__cause__,BrokenPipeError)

    def test_close_error_is_sticky_unknown_never_fabricated_restoration(self):
        saved=self.scope._write_fd;real_close=os.close
        def failed_close(fd):
            if fd==saved:raise OSError('fixture close unknown')
            return real_close(fd)
        try:
            with patch.object(c.os,'close',side_effect=failed_close):
                with self.assertRaisesRegex(c.WaitError,'close failed'):self.scope.close()
            self.assertFalse(self.scope.statistics()['cleanup_complete'])
            self.scope.close()
            self.assertFalse(self.scope.statistics()['cleanup_complete'])
        finally:real_close(saved)

    def test_allocation_failure_releases_duplicate_and_restores_signal_mask(self):
        other=c.NotificationScope(self.library)
        mask_before=signal.pthread_sigmask(signal.SIG_BLOCK,set())
        method='pipe2' if hasattr(os,'pipe2') else 'pipe'
        with patch.object(c.os,method,side_effect=RuntimeError('pipe allocation failed')):
            with self.assertRaisesRegex(RuntimeError,'allocation failed'):other.start(self.cancel_read)
        self.assertIsNone(other._cancel_fd)
        self.assertTrue(other.statistics()['cleanup_complete'])
        self.assertEqual(signal.pthread_sigmask(signal.SIG_BLOCK,set()),mask_before)

    def test_real_signal_eintr_retries_keep_absolute_tick(self):
        self.scope.register((Future(),Future()))
        original_handler=signal.getsignal(signal.SIGALRM)
        original_timer=signal.getitimer(signal.ITIMER_REAL)
        signal.signal(signal.SIGALRM,lambda *_:None)
        now=time.monotonic_ns()
        try:
            signal.setitimer(signal.ITIMER_REAL,.004,.004)
            result=self.scope.wait(now+20_000_000,now+500_000_000)
            self.assertEqual(result['kind'],'TICK')
            self.assertGreaterEqual(result['actual_ns'],now+20_000_000)
        finally:
            signal.setitimer(signal.ITIMER_REAL,*original_timer)
            signal.signal(signal.SIGALRM,original_handler)

    def test_real_repeated_eintr_is_finite_failure(self):
        self.scope.register((Future(),Future()))
        original_handler=signal.getsignal(signal.SIGALRM)
        original_timer=signal.getitimer(signal.ITIMER_REAL)
        signal.signal(signal.SIGALRM,lambda *_:None)
        now=time.monotonic_ns()
        try:
            signal.setitimer(signal.ITIMER_REAL,.001,.001)
            with self.assertRaisesRegex(c.WaitError,'interrupted too often'):
                self.scope.wait(now+150_000_000,now+500_000_000)
        finally:
            signal.setitimer(signal.ITIMER_REAL,*original_timer)
            signal.signal(signal.SIGALRM,original_handler)

    def test_real_late_callback_io_error_after_last_guard_is_not_hidden(self):
        one,two=Future(),Future();scope=c.NotificationScope(self.library)
        callback_entered,release_callback=threading.Event(),threading.Event()
        original_notify=scope._callback_fn
        def delayed_notify(future):
            if future is one:
                callback_entered.set()
                if not release_callback.wait(1):raise RuntimeError('fixture release timeout')
            original_notify(future)
        scope._callback_fn=delayed_notify
        self.later(.003,lambda:one.set_result(1));self.later(.006,lambda:two.set_result(2))
        damaged=False
        def guard():
            nonlocal damaged
            if not damaged and callback_entered.is_set() and two.done():
                # Deliberate private-FD fault exactly after the preceding
                # callback-error check, while the native wait is not running.
                damaged=True;os.close(scope._read_fd);scope._read_fd=None
                release_callback.set();self.threads[-2].join(1)
        try:
            with patch.object(c,'NotificationScope',return_value=scope):
                with self.assertRaisesRegex(c.WaitError,'callback failed') as raised:
                    c.join_once({'front':one,'rear':two},None,library=self.library,
                                cancel_fd=self.cancel_read,phase='Voltage',
                                deadline_ns=time.monotonic_ns()+500_000_000,check=guard)
            self.assertTrue(damaged)
            self.assertTrue(raised.exception.notification_cleanup_complete)
            self.assertTrue(scope.statistics()['cleanup_complete'])
            self.assertIsInstance(raised.exception.__cause__,BrokenPipeError)
        finally:release_callback.set();scope.close()


class ScratchTests(unittest.TestCase):
    def fixture(self,callback):
        fn=C.CFUNCTYPE(C.c_int,*c.ARGTYPES)(callback)
        fn.argtypes=list(c.ARGTYPES);fn.restype=C.c_int
        library=SimpleNamespace(nw_wait=fn,nw_abi=lambda:1)
        read_fd,write_fd=os.pipe();scope=c.NotificationScope(library);scope.start(read_fd)
        scope.register((Future(),Future()))
        self.addCleanup(os.close,read_fd);self.addCleanup(os.close,write_fd)
        self.addCleanup(scope.close)
        return scope,library

    def test_native_status_error_and_timestamp_contracts_and_scratch_reset(self):
        for status,actual,error in ((4,0,b''),(0,9,b''),(1,0,b''),(1,20,b''),(0,10,b'error')):
            def callback(n,cancel,tick,hard,out,err,size,s=status,a=actual,e=error):
                out[0]=a
                if e:C.memmove(err,e+b'\0',len(e)+1)
                return s
            scope,library=self.fixture(callback)
            with self.assertRaises(c.WaitError):scope.wait(10,20)
            self.assertEqual(scope._actual.value,0);self.assertEqual(scope._error.value,b'')
            scope.close()

    def test_function_replacement_and_changed_abi_are_rejected(self):
        def callback(n,cancel,tick,hard,out,err,size):out[0]=tick;return 0
        scope,library=self.fixture(callback)
        real=library.nw_wait
        library.nw_wait=lambda *_:0
        with self.assertRaisesRegex(ValueError,'GIL-releasing'):scope.wait(10,20)
        library.nw_wait=real;real.restype=C.c_uint32
        with self.assertRaisesRegex(ValueError,'GIL-releasing'):scope.wait(10,20)
        real.restype=C.c_int


class FakeScope:
    def __init__(self, step=None): self.step = step; self.calls = 0; self.owners = None
    def register(self, owners): self.owners = owners
    def check_callback(self): pass
    def wait(self, tick, deadline):
        self.calls += 1
        if self.step: self.step(self, tick, deadline)
        return dict(kind='NOTIFIED', actual_ns=1)


class Clock:
    def __init__(self, values): self.values = iter(values)
    def __call__(self): return next(self.values)


class ContractTests(unittest.TestCase):
    def test_bounded_regular_source_rejection_closes_directory_fd(self):
        captured=[];real_open=os.open
        def opening(*args,**kwargs):
            fd=real_open(*args,**kwargs);captured.append(fd);return fd
        with patch.object(c.os,'open',side_effect=opening):
            with self.assertRaisesRegex(ValueError,'Bounded nonempty regular'):
                c.read_regular(Path(__file__).parent.resolve())
        self.assertEqual(len(captured),1)
        with self.assertRaises(OSError):os.fstat(captured[0])

    def test_bounded_fifo_source_rejection_closes_fd(self):
        captured=[];real_open=os.open
        def opening(*args,**kwargs):
            fd=real_open(*args,**kwargs);captured.append(fd);return fd
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            path=Path(directory).resolve()/'fifo';os.mkfifo(path)
            with patch.object(c.os,'open',side_effect=opening):
                with self.assertRaisesRegex(ValueError,'Bounded nonempty regular'):c.read_regular(path)
        self.assertEqual(len(captured),1)
        with self.assertRaises(OSError):os.fstat(captured[0])

    def test_source_stream_wrapper_exception_closes_fd(self):
        captured=[];real_open=os.open
        def opening(*args,**kwargs):
            fd=real_open(*args,**kwargs);captured.append(fd);return fd
        with patch.object(c.os,'open',side_effect=opening),patch.object(c.os,'fdopen',side_effect=RuntimeError('fixture wrapper failure')):
            with self.assertRaisesRegex(RuntimeError,'wrapper failure'):c.read_regular(Path(__file__).resolve())
        self.assertEqual(len(captured),1)
        with self.assertRaises(OSError):os.fstat(captured[0])

    def test_one_shot_cleanup_does_not_replace_original_owner_error(self):
        original=ValueError('owner');cleanup=c.CloseBusy('callback pending')
        scope=SimpleNamespace(start=lambda _:None,close=lambda:(_ for _ in ()).throw(cleanup),_cleanup_complete=False)
        with patch.object(c,'NotificationScope',return_value=scope),patch.object(c,'await_ready',side_effect=original):
            with self.assertRaises(ValueError) as raised:
                c.join_once({},None,library=object(),cancel_fd=0)
        self.assertIs(raised.exception,original)
        self.assertIs(original.notification_scope,scope)
        self.assertFalse(original.notification_cleanup_complete)

    def test_one_shot_success_with_cleanup_failure_never_returns_complete(self):
        cleanup=c.CloseBusy('callback pending')
        scope=SimpleNamespace(start=lambda _:None,close=lambda:(_ for _ in ()).throw(cleanup),_cleanup_complete=False)
        with patch.object(c,'NotificationScope',return_value=scope),patch.object(c,'await_ready',return_value={}):
            with self.assertRaises(c.CloseBusy) as raised:
                c.join_once({},None,library=object(),cancel_fd=0)
        self.assertIs(raised.exception,cleanup)
        self.assertFalse(cleanup.notification_cleanup_complete)

    def test_postclose_callback_error_rejects_success_with_truthful_fd_cleanup(self):
        error=c.WaitError('late callback I/O failure')
        scope=SimpleNamespace(start=lambda _:None,close=lambda:None,
            check_callback=lambda:(_ for _ in ()).throw(error),_cleanup_complete=True)
        with patch.object(c,'NotificationScope',return_value=scope),patch.object(c,'await_ready',return_value={}):
            with self.assertRaises(c.WaitError) as raised:
                c.join_once({},None,library=object(),cancel_fd=0)
        self.assertIs(raised.exception,error)
        self.assertTrue(error.notification_cleanup_complete)

    def test_postclose_callback_error_does_not_replace_primary_owner_error(self):
        original=ValueError('original owner');late=c.WaitError('late callback')
        scope=SimpleNamespace(start=lambda _:None,close=lambda:None,
            check_callback=lambda:(_ for _ in ()).throw(late),_cleanup_complete=True)
        with patch.object(c,'NotificationScope',return_value=scope),patch.object(c,'await_ready',side_effect=original):
            with self.assertRaises(ValueError) as raised:
                c.join_once({},None,library=object(),cancel_fd=0)
        self.assertIs(raised.exception,original)
        self.assertIn('late callback',original.notification_cleanup_error)
        self.assertTrue(original.notification_cleanup_complete)

    def test_original_poll_targets_exactly_preserved(self):
        for remaining in (1,2,3,4,49_999,50_000,99_999,100_000,399_999,400_000,400_001,1_000_000):
            self.assertEqual(c.poll_target(100,100+remaining),
                             baseline._readiness_poll_target(100,100+remaining))

    def test_original_ready_result_clocks_guard_order_and_no_takeout(self):
        results, checks = [], []
        for func in ('original', 'notification'):
            futures = {'front': ready(), 'rear': ready()}
            values = Clock([100,200,300]); cpu = Clock([10,20]); seen = []
            if func == 'original':
                result = baseline._await_owned_ready(futures,None,phase='Proxy output',deadline_ns=500,
                            deadline_wait=lambda _:None,clock=values,thread_clock=cpu,check=lambda:seen.append(1))
            else:
                result = c.await_ready(futures,None,scope=FakeScope(),phase='Proxy output',deadline_ns=500,
                                      clock=values,thread_clock=cpu,check=lambda:seen.append(1))
            results.append({k:result[k] for k in ('begin_ns','end_ns','thread_cpu_begin_ns','thread_cpu_end_ns','wait_calls','future_results_taken_only_after_ready')})
            checks.append(seen)
        self.assertEqual(results[0],results[1]); self.assertEqual(checks,[[1],[1]])

    def test_notification_does_not_make_unfinished_owner_ready(self):
        futures={'front':Future(),'rear':Future()}; checks=[]
        with self.assertRaises(TimeoutError):
            c.await_ready(futures,None,scope=FakeScope(),phase='Voltage',deadline_ns=500,
                          clock=Clock([100,200,300,500]),thread_clock=lambda:10,
                          check=lambda:checks.append(1))
        self.assertEqual(checks,[1,1]); self.assertFalse(futures['rear'].done())

    def test_ready_at_deadline_and_completion_crossing_deadline_rejected(self):
        for values in ([100,500],[100,200,500]):
            with self.assertRaises(TimeoutError):
                c.await_ready({'front':ready(),'rear':ready()},None,scope=FakeScope(),phase='Voltage',
                              deadline_ns=500,clock=Clock(values),thread_clock=Clock([1,2]))

    def test_owner_error_precedes_native_cancel_exception(self):
        f=Future(); other=Future(); original=ValueError('owner')
        def step(scope,tick,deadline): f.set_exception(original); raise c.WaitError('cancel')
        with self.assertRaises(ValueError) as raised:
            c.await_ready({'front':f,'rear':other},None,scope=FakeScope(step),phase='Voltage',
                          deadline_ns=500,clock=Clock([100,200]),thread_clock=lambda:1)
        self.assertIs(raised.exception,original)

    def test_boot_guard_failure_precedes_ready_success(self):
        def check(): raise RuntimeError('boot changed')
        with self.assertRaisesRegex(RuntimeError,'boot changed'):
            c.await_ready({'front':ready(),'rear':ready()},None,scope=FakeScope(),phase='Voltage',
                          deadline_ns=500,clock=Clock([100]),thread_clock=lambda:1,check=check)

    def test_noncausal_clock_and_notification_return_rejected(self):
        with self.assertRaisesRegex(ValueError,'Noncausal join clock'):
            c.await_ready({'front':ready(),'rear':ready()},None,scope=FakeScope(),phase='Voltage',
                          deadline_ns=500,clock=Clock([100,99]),thread_clock=lambda:1)
        with self.assertRaisesRegex(ValueError,'native notification return clock'):
            c.await_ready({'front':Future(),'rear':Future()},None,scope=FakeScope(),phase='Voltage',
                          deadline_ns=500,clock=Clock([100,200,0]),thread_clock=lambda:1)

    def test_callback_write_error_preserved_but_done_owner_error_first(self):
        # This is tested via exact ordinary Future and a scope with real ABI
        # in NativeTests; here assert ready owner exception order explicitly.
        f=Future();original=ValueError('owner');f.set_exception(original)
        scope=FakeScope()
        scope.check_callback=lambda:(_ for _ in ()).throw(c.WaitError('callback'))
        with self.assertRaises(ValueError) as raised:
            c.await_ready({'front':f,'rear':Future()},None,scope=scope,phase='Voltage',
                          deadline_ns=500,clock=Clock([100]),thread_clock=lambda:1)
        self.assertIs(raised.exception,original)


if __name__ == '__main__': unittest.main()
