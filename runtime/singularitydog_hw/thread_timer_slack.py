"""Opt-in Linux timer slack for a diagnostic collector and its new workers.

The default scope does not load libc or read/change OS settings. A selected
scope changes only its calling thread; new workers inherit that value and only
read it back. Leaving the scope restores and verifies the exact original value.
"""
import os
import sys
import threading


CHOICES_NS = (1_000, 50_000)
PR_SET_TIMERSLACK = 29
PR_GET_TIMERSLACK = 30


class TimerSlackError(RuntimeError):
    pass


def require_supported_platform():
    if sys.platform != 'linux':
        raise TimerSlackError('Diagnostic timer slack requires Linux')


def _load_prctl():
    # Import and symbol lookup stay inside the explicitly selected Linux scope.
    import ctypes
    libc = ctypes.CDLL(None, use_errno=True)
    prctl = libc.prctl
    prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong,
                     ctypes.c_ulong, ctypes.c_ulong]
    prctl.restype = ctypes.c_int

    class LinuxPrctl:
        def call(self, operation, value=0):
            result = prctl(operation, value, 0, 0, 0)
            if result == -1:
                error = ctypes.get_errno()
                raise TimerSlackError('prctl timer slack failed: '+os.strerror(error))
            return result

        def get(self):
            value = self.call(PR_GET_TIMERSLACK)
            if value <= 0:
                raise TimerSlackError('Positive timer slack readback required')
            return value

        def set(self, value):
            if type(value) is not int or value <= 0:
                raise TimerSlackError('Restoring timer slack requires an exact positive value')
            if self.call(PR_SET_TIMERSLACK, value) != 0:
                raise TimerSlackError('Timer slack write was not confirmed')

    return LinuxPrctl()


class TimerSlack:
    def __init__(self, requested_ns=None):
        if requested_ns is not None and (type(requested_ns) is not int or requested_ns not in CHOICES_NS):
            raise ValueError('Diagnostic timer slack must be 1000 or 50000 ns')
        self.requested_ns = requested_ns
        self.report = {
            'requested_ns': requested_ns, 'enabled': requested_ns is not None,
            'scope': 'diagnostic_collect_only',
            'parent': {'native_tid': None, 'original_ns': None, 'during_ns': None,
                       'after_ns': None, 'restored': None},
            'workers': [], 'worker_verification_complete': False,
            'errors': [],
        }
        self._lock = threading.Lock()
        self._prctl = None
        self._active = False
        self._entered = False

    def _error(self, error):
        self.report['errors'].append(type(error).__name__+': '+str(error))

    def _restore(self):
        parent = self.report['parent']
        parent['restored'] = False
        try:
            if threading.get_native_id() != parent['native_tid']:
                raise TimerSlackError('Timer slack must be restored by its original thread')
            self._prctl.set(parent['original_ns'])
            parent['after_ns'] = self._prctl.get()
            if parent['after_ns'] != parent['original_ns']:
                raise TimerSlackError('Exact original timer slack restoration unconfirmed')
            parent['restored'] = True
        except BaseException as error:
            self._error(error)
            raise
        finally:
            self._active = False

    def __enter__(self):
        if self._entered:
            raise TimerSlackError('Timer slack scope cannot be reused')
        self._entered = True
        if self.requested_ns is None:
            return self
        try:
            require_supported_platform()
            self._prctl = _load_prctl()
            parent = self.report['parent']
            parent['native_tid'] = threading.get_native_id()
            parent['original_ns'] = self._prctl.get()
            if type(parent['original_ns']) is not int or parent['original_ns'] <= 0:
                raise TimerSlackError('Exact positive original timer slack required')
            self._prctl.set(self.requested_ns)
            parent['during_ns'] = self._prctl.get()
            if parent['during_ns'] != self.requested_ns:
                raise TimerSlackError('Requested parent timer slack readback mismatch')
            self._active = True
            return self
        except BaseException as error:
            self._error(error)
            if type(self.report['parent']['original_ns']) is int and self.report['parent']['original_ns'] > 0:
                self._restore()
            raise

    def worker_initializer(self):
        """Read inherited slack once, before a worker may reach the startup barrier."""
        if self.requested_ns is None:
            return
        row = {'native_tid': threading.get_native_id(), 'current_ns': None, 'verified': False}
        error = None
        try:
            if not self._active:
                raise TimerSlackError('Worker started outside its diagnostic timer slack scope')
            row['current_ns'] = self._prctl.get()
            if row['current_ns'] != self.requested_ns:
                raise TimerSlackError('Worker inherited timer slack readback mismatch')
            row['verified'] = True
        except BaseException as failure:
            error = failure
            row['error'] = type(failure).__name__+': '+str(failure)
        with self._lock:
            self.report['workers'].append(row)
            workers = self.report['workers']
            self.report['worker_verification_complete'] = (
                len(workers) == 3 and len({worker['native_tid'] for worker in workers}) == 3
                and all(worker['verified'] for worker in workers))
        if error is not None:
            raise error

    def verify_workers(self):
        if self.requested_ns is not None and not self.report['worker_verification_complete']:
            raise TimerSlackError('Three distinct diagnostic worker timer slack readbacks required')

    def __exit__(self, error_type, error, traceback):
        if self.requested_ns is not None:
            self._restore()
        return False
