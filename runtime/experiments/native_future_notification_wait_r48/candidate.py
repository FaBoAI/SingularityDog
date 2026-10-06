"""Isolated Future notification experiment; no runtime import or live selection.

Future callbacks only wake a private pipe. Ready/error/boot checks and real
decision clocks remain on the coordinator, with the original <=200us tick
fallback and <=50us tail. No notification timestamp certifies completion.
"""
from concurrent.futures import Future
import ctypes as C
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import threading
import time

ARGTYPES = (C.c_int, C.c_int, C.c_uint64, C.c_uint64,
            C.POINTER(C.c_uint64), C.POINTER(C.c_char), C.c_uint32)
FLAGS = dict(hardware_opened=False, source_runtime_changed=False,
             timing_admission_eligible=False, active_output_eligible=False,
             approved_for_runtime=False, motor_enable_sent=False,
             learned_targets_sent=False)


def read_regular(path, limit=4_000_000):
    path = Path(path)
    if (not path.is_absolute() or '..' in path.parts or
            any(p.is_symlink() for p in (path, *path.parents))):
        raise ValueError('Absolute non-symlink file required')
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= limit:
            raise ValueError('Bounded nonempty regular file required')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            value = stream.read(before.st_size + 1)
            after = os.fstat(fd)
    finally:
        os.close(fd)
    identity = lambda x: (x.st_dev, x.st_ino, x.st_size, x.st_mtime_ns)
    if len(value) != before.st_size or identity(before) != identity(after):
        raise ValueError('File changed during bounded read')
    return value


def load_library(path):
    """Explicit local file-only build; PLAN/import never calls this."""
    path = Path(path)
    source = path.parent / 'notify_wait.cpp'
    record_path = path.parent / 'build-record.json'
    record_raw = read_regular(record_path)
    record = json.loads(record_raw)
    source_raw, binary_raw = read_regular(source), read_regular(path)
    if (record.get('schema') != 'private.notification-wait-build.v1' or
            record.get('source_sha256') != hashlib.sha256(source_raw).hexdigest() or
            record.get('binary_sha256') != hashlib.sha256(binary_raw).hexdigest()):
        raise ValueError('Notification build source/binary pin mismatch')
    library = C.CDLL(str(path))
    library.nw_abi.argtypes = []; library.nw_abi.restype = C.c_uint32
    if library.nw_abi() != 1:
        raise ValueError('Notification ABI mismatch')
    library.nw_now_ns.argtypes = []; library.nw_now_ns.restype = C.c_uint64
    before = time.monotonic_ns(); actual = library.nw_now_ns(); after = time.monotonic_ns()
    if not before - 1000 <= actual <= after + 1000:
        raise ValueError('Native notification and Python monotonic clocks differ')
    library.nw_wait.argtypes = list(ARGTYPES); library.nw_wait.restype = C.c_int
    if (read_regular(source) != source_raw or read_regular(path) != binary_raw or
            read_regular(record_path) != record_raw):
        raise ValueError('Notification build changed while loading')
    return library


def poll_target(now, deadline_ns):
    remaining = deadline_ns - now
    step = min(50_000, max(1, remaining // 2)) if remaining <= 400_000 else 200_000
    return min(deadline_ns, now + step)


class WaitError(RuntimeError):
    pass


class CloseBusy(WaitError):
    """FD ownership retained; cleanup is unknown until a later close succeeds."""


class NotificationScope:
    """Retain this empty object before start; a single coordinator owns it.

    Callback writes and close share a lock. Callbacks never block on that lock;
    a contended notification is safely missed and the existing tick checks the
    Future. close is nonblocking and fails honestly if a callback owns the lock.
    No FD is closed/reused while a callback writes. Late callbacks skip after
    close. The caller may retry close only, never retry a failed join.
    """
    def __init__(self, library):
        self._library = library
        self._waiter = library.nw_wait
        self._verify()
        if library.nw_abi() != 1:
            raise ValueError('Notification ABI mismatch')
        self._owner = threading.current_thread()
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._read_fd = self._write_fd = self._cancel_fd = None
        self._started = self._closed = self._registered = False
        self._cleanup_complete = False
        self._cleanup_failed = False
        self._owners = None
        self._callback_error = None
        self._notifications = self._saturated = self._contended = self._late = 0
        self._actual = C.c_uint64()
        self._error = C.create_string_buffer(256)
        self._actual_ptr = C.byref(self._actual)
        self._callback_fn = self._notify

    def _verify(self):
        fn = self._waiter
        if (getattr(self._library, 'nw_wait', None) is not fn or
                not isinstance(fn, C._CFuncPtr) or fn._flags_ != C._FUNCFLAG_CDECL or
                tuple(fn.argtypes or ()) != ARGTYPES or fn.restype is not C.c_int or
                getattr(fn, 'errcheck', None) is not None):
            raise ValueError('Exact GIL-releasing notification ABI required')

    def _check_owner(self):
        if threading.current_thread() is not self._owner:
            raise WaitError('Notification scope used by another coordinator')

    def start(self, cancel_fd):
        self._check_owner()
        if self._started or self._closed or type(cancel_fd) is not int:
            raise ValueError('Fresh notification scope and cancellation FD required')
        # Normal OS signal interruption is deferred until all newly created FDs
        # are retained, or cleanup completed. Arbitrary injected async exceptions
        # at every interpreter bytecode are not certified by this experiment.
        signals = {signal.SIGINT, signal.SIGTERM, signal.SIGHUP}
        prior = signal.pthread_sigmask(signal.SIG_BLOCK, signals)
        try:
            self._cancel_fd = fcntl.fcntl(cancel_fd, fcntl.F_DUPFD_CLOEXEC, 0)
            if hasattr(os, 'pipe2'):
                self._read_fd, self._write_fd = os.pipe2(os.O_NONBLOCK | os.O_CLOEXEC)
            else:
                self._read_fd, self._write_fd = os.pipe()
                for fd in (self._read_fd, self._write_fd):
                    os.set_blocking(fd, False); os.set_inheritable(fd, False)
            self._started = True
        except BaseException:
            self.close()
            raise
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, prior)

    def register(self, owners):
        self._check_owner()
        owners = tuple(owners)
        if (not self._started or self._closed or self._registered or
                not 2 <= len(owners) <= 3 or len({id(f) for f in owners}) != len(owners) or
                any(type(f) is not Future for f in owners)):
            raise ValueError('Exactly two/three distinct ordinary Futures required once')
        self._registered = True; self._owners = owners
        for future in owners:
            future.add_done_callback(self._callback_fn)

    def _notify(self, future):
        if not self._lock.acquire(blocking=False):
            self._contended += 1
            return
        try:
            if self._write_fd is None:
                self._late += 1
                return
            try:
                count = os.write(self._write_fd, b'!')
                if count != 1:
                    raise WaitError('Partial notification write')
                self._notifications += 1
            except BlockingIOError:
                # Existing bytes make the pipe readable: no readiness loss.
                self._saturated += 1
            except BaseException as error:
                self._callback_error = error
        finally:
            self._lock.release()

    def check_callback(self):
        if self._callback_error is not None:
            raise WaitError('Notification callback failed') from self._callback_error

    def wait(self, tick_ns, hard_ns):
        self._check_owner()
        if (not self._started or self._closed or not self._registered or
                type(tick_ns) is not int or type(hard_ns) is not int or
                not 0 < tick_ns <= hard_ns < 2**64):
            raise ValueError('Live notification scope and absolute tick/hard deadline required')
        if not self._busy.acquire(blocking=False):
            raise WaitError('Reentrant notification wait')
        try:
            self._verify(); self.check_callback()
            self._actual.value = 0; self._error.raw = bytes(256)
            status = self._waiter(self._read_fd, self._cancel_fd, tick_ns, hard_ns,
                                  self._actual_ptr, self._error, 256)
            actual, error = self._actual.value, self._error.value
            if status == -2:
                raise TimeoutError(error.decode('utf-8', 'replace'))
            if status not in (0, 1):
                raise WaitError(error.decode('utf-8', 'replace') or 'Notification native error')
            if error or actual <= 0 or actual >= hard_ns or (status == 0 and actual < tick_ns):
                raise WaitError('Noncausal/error notification native success')
            self.check_callback()
            return dict(kind='NOTIFIED' if status == 1 else 'TICK', actual_ns=actual)
        finally:
            self._actual.value = 0; self._error.raw = bytes(256)
            self._busy.release()

    def close(self):
        self._check_owner()
        if not self._lock.acquire(blocking=False):
            raise CloseBusy('Callback still owns FD; cleanup incomplete')
        prior = None
        try:
            if self._busy.locked():
                raise CloseBusy('Native wait still owns FD; cleanup incomplete')
            prior = signal.pthread_sigmask(signal.SIG_BLOCK,
                                          {signal.SIGINT, signal.SIGTERM, signal.SIGHUP})
            errors = []
            for name in ('_write_fd', '_read_fd', '_cancel_fd'):
                fd = getattr(self, name)
                if fd is not None:
                    try: os.close(fd)
                    except BaseException as error: errors.append(error)
                    finally: setattr(self, name, None)
            self._owners = None; self._closed = True
            if errors: self._cleanup_failed = True
            self._cleanup_complete = not self._cleanup_failed
            if errors: raise WaitError('Notification close failed') from errors[0]
        finally:
            self._lock.release()
            if prior is not None:
                signal.pthread_sigmask(signal.SIG_SETMASK, prior)

    def statistics(self):
        return dict(notifications=self._notifications, saturated=self._saturated,
                    contended_fallback=self._contended, late_callbacks_skipped=self._late,
                    cleanup_complete=self._cleanup_complete,
                    callback_timestamp_is_completion_proof=False, **FLAGS)


def await_ready(futures, third, *, scope, phase, deadline_ns,
                clock=time.monotonic_ns, check=lambda: None,
                thread_clock=time.thread_time_ns):
    """One-shot isolated equivalent coordinator, receives a retained live scope.

    No result is taken here. A notification only requests the original owner
    readiness/error/boot/clock checks. Hard deadlines and error-before-timeout
    order remain; the original native waiter itself is never monkeypatched.
    """
    if (type(futures) is not dict or set(futures) != {'front', 'rear'} or
            any(type(f) is not Future for f in futures.values()) or
            (third is not None and type(third) is not Future) or
            type(deadline_ns) is not int or deadline_ns <= 0):
        raise ValueError('Exact owner Futures and absolute deadline required')
    owners = tuple(futures.values()) + (() if third is None else (third,))
    scope.register(owners)
    owner_count = len(owners); ready_flags = bytearray(owner_count)
    begin = clock(); cpu_begin = thread_clock(); calls = notifications = 0
    if type(begin) is not int or begin <= 0:
        raise ValueError('Causal join clock required')
    last_clock = begin
    while True:
        ready_count = 0
        for index, future in enumerate(owners):
            ready_flags[index] = bool(future.done()); ready_count += ready_flags[index]
        for index, future in enumerate(owners):
            if not ready_flags[index]: continue
            if future.cancelled(): raise RuntimeError(phase + ' owner future cancelled')
            error = future.exception()
            if error is not None: raise error
        scope.check_callback()
        check()
        now = clock()
        if type(now) is not int or now < last_clock:
            raise ValueError('Noncausal join clock')
        last_clock = now
        if now >= deadline_ns: raise TimeoutError(phase + ' hard join deadline reached')
        if ready_count == owner_count:
            cpu_end = thread_clock(); end = clock()
            if type(end) is not int or end < now or cpu_end < cpu_begin:
                raise ValueError('Noncausal join completion clock')
            if end >= deadline_ns: raise TimeoutError(phase + ' hard join completion deadline reached')
            return dict(mode='native_notification_tick_fallback_experiment_v1',
                        begin_ns=begin, end_ns=end, thread_cpu_begin_ns=cpu_begin,
                        thread_cpu_end_ns=cpu_end, wait_calls=calls,
                        notification_returns=notifications, native_tick_max_us=200,
                        native_tail_window_us=400, native_tail_tick_max_us=50,
                        future_results_taken_only_after_ready=True, **FLAGS)
        tick = poll_target(now, deadline_ns); calls += 1
        try:
            result = scope.wait(tick, deadline_ns)
        except BaseException:
            for future in owners:
                if future.done() and not future.cancelled():
                    error = future.exception()
                    if error is not None: raise error
            raise
        returned = clock()
        if (type(returned) is not int or returned < result['actual_ns'] or
                (result['kind'] == 'TICK' and returned < tick)):
            raise ValueError('Noncausal native notification return clock')
        if result['kind'] == 'NOTIFIED': notifications += 1
        last_clock = returned


def join_once(futures, third, *, library, cancel_fd, **options):
    """Retain resources before allocation; preserve primary error on cleanup.

    Failed/busy cleanup retains the scope on the raised exception. It is never
    reported as closed. Only cleanup may be retried after callbacks finish;
    the failed join itself has no retry path.
    """
    scope = NotificationScope(library)
    primary = None
    try:
        scope.start(cancel_fd)
        result = await_ready(futures, third, scope=scope, **options)
    except BaseException as error:
        primary = error
        raise
    finally:
        try:
            scope.close()
            scope.check_callback()
        except BaseException as cleanup_error:
            retained = primary if primary is not None else cleanup_error
            retained.notification_cleanup_complete = scope._cleanup_complete
            retained.notification_cleanup_error = type(cleanup_error).__name__ + ': ' + str(cleanup_error)
            retained.notification_scope = scope
            if primary is None: raise
    result['notification_scope'] = scope.statistics()
    return result
