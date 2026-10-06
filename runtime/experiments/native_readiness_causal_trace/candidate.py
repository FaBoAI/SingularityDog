"""Opt-in, file-only readiness observation prototype; no device/CLI execution.

The fixed baseline function is cloned for explicit diagnostic observation only.
``trace=None`` delegates to that original function and adds no timing calls.
No production caller selects this prototype. Export only after all owners join.
"""
from array import array
import ast
from concurrent.futures import Future
import hashlib
import os
from pathlib import Path
import stat
import threading

BASELINE_SHA256 = '0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'
DEFAULT_SOURCE = Path(__file__).resolve().parents[2] / 'singularitydog_hw/native_pipeline_benchmark.py'
FIELDS = (
    'readiness_begin_ns', 'readiness_begin_thread_cpu_ns',
    'readiness_and_error_end_ns', 'readiness_and_error_end_thread_cpu_ns',
    'guard_before_ns', 'guard_before_thread_cpu_ns',
    'guard_after_ns', 'guard_after_thread_cpu_ns',
    'decision_ns', 'ready_count', 'planned_wake_ns',
    'native_call_before_ns', 'native_call_before_thread_cpu_ns',
    'native_woke_return_value_ns', 'python_return_ns', 'python_return_thread_cpu_ns',
    'completion_ns',
)
OWNER_FIELDS = ('registration_before_ns', 'registration_before_thread_cpu_ns',
                'registration_after_ns', 'registration_after_thread_cpu_ns',
                'callback_observed_ns', 'callback_thread_cpu_ns',
                'already_done_at_registration', 'callback_count')
UNKNOWN = -1


def _read_source(path):
    path = Path(path)
    if (not path.is_absolute() or '..' in path.parts or
            any(p.is_symlink() for p in (path, *path.parents))):
        raise ValueError('Absolute regular source without symlinks required')
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(fd, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > 256_000:
            raise ValueError('Bounded regular source required')
        raw = stream.read(256_001)
    if len(raw) > 256_000 or hashlib.sha256(raw).hexdigest() != BASELINE_SHA256:
        raise ValueError('Exact K37 adaptive readiness source required')
    return raw


class FixedReadinessTrace:
    """One helper invocation, fixed numeric slots and three owner callbacks.

    A callback observes an already-completed Future. Its timestamp is neither
    the native exchange end nor the exact internal Future publication time.
    Clock/overflow errors only invalidate trace evidence, never alter a result.
    There are no per-poll lists/dicts/records or I/O. Python clock and integer
    operations still have a cost; this is not an allocation-free native tracer.
    """
    def __init__(self, capacity=256):
        if type(capacity) is not int or not 1 <= capacity <= 2048:
            raise ValueError('Bounded trace capacity 1..2048 required')
        self.capacity = capacity
        self._rows = array('q', [UNKNOWN]) * (capacity * len(FIELDS))
        self._owners = array('q', [UNKNOWN]) * (3 * len(OWNER_FIELDS))
        self._owner = threading.current_thread()
        self._clock = self._thread_clock = None
        self._futures = None
        self._callbacks = None
        self._row = -1
        self.polls_seen = self.overflow = self.trace_errors = 0
        self.finished = 0  # 0 incomplete, 1 returned, 2 original helper raised

    def bind(self, futures, clock, thread_clock):
        if (threading.current_thread() is not self._owner or self._futures is not None or
                not 2 <= len(futures) <= 3 or
                any(type(f) is not Future for f in futures) or
                len({id(f) for f in futures}) != len(futures)):
            raise ValueError('Fresh owner-thread trace and exact distinct Futures required')
        self._clock, self._thread_clock, self._futures = clock, thread_clock, tuple(futures)
        # All callbacks/closures and owner slots are allocated during binding.
        self._callbacks = tuple(self._make_callback(index) for index in range(len(futures)))
        for index, future in enumerate(futures):
            offset = index * len(OWNER_FIELDS)
            self._stamp(self._owners, offset)
            self._number(self._owners, offset + 6, int(future.done()))
            future.add_done_callback(self._callbacks[index])
            self._stamp(self._owners, offset + 2)

    def _make_callback(self, index):
        offset = index * len(OWNER_FIELDS)
        def observed(future):
            self._stamp(self._owners, offset + 4)
            self._number(self._owners, offset + 7, 1)
        return observed

    def _number(self, target, index, value):
        if type(value) is int and 0 <= value < 2**63:
            target[index] = value
        else:
            self.trace_errors += 1

    def _stamp(self, target, index):
        try:
            self._number(target, index, self._clock())
            self._number(target, index + 1, self._thread_clock())
        except Exception:
            self.trace_errors += 1

    def next_poll(self):
        self.polls_seen += 1
        if self.polls_seen <= self.capacity:
            self._row = (self.polls_seen - 1) * len(FIELDS)
            self.stamp(0)
        else:
            self._row = -1
            self.overflow += 1

    def stamp(self, column):
        if self._row >= 0:
            self._stamp(self._rows, self._row + column)

    def number(self, column, value):
        if self._row >= 0:
            self._number(self._rows, self._row + column, value)

    def return_cpu(self):
        if self._row >= 0:
            try:
                self.number(15, self._thread_clock())
            except Exception:
                self.trace_errors += 1

    def native_value(self, value):
        # None/custom return values have always been allowed by the baseline;
        # retain UNKNOWN rather than rejecting or inventing a native timestamp.
        if value is not None:
            self.number(13, value)

    def export(self):
        """Serialize outside the measured helper, after every owner settled."""
        if (threading.current_thread() is not self._owner or self._futures is None or
                not self.finished or any(not f.done() for f in self._futures) or
                any(self._owners[i * len(OWNER_FIELDS) + 7] != 1
                    for i in range(len(self._futures)))):
            raise ValueError('Wait for original helper and all owner callbacks before export')
        rows = [list(self._rows[i * len(FIELDS):(i + 1) * len(FIELDS)])
                for i in range(min(self.polls_seen, self.capacity))]
        owners = [list(self._owners[i * len(OWNER_FIELDS):(i + 1) * len(OWNER_FIELDS)])
                  for i in range(len(self._futures))]
        native_rows = [row for row in rows if row[10] != UNKNOWN]
        native_available = all(row[13] != UNKNOWN and row[14] != UNKNOWN for row in native_rows)
        native_causal = native_available and all(row[10] <= row[13] <= row[14] for row in native_rows)
        return dict(schema='singularitydog.readiness-causal-trace-prototype.v1',
                    baseline_sha256=BASELINE_SHA256, fields=list(FIELDS), rows=rows,
                    owner_fields=list(OWNER_FIELDS), owners=owners,
                    capacity=self.capacity, polls_seen=self.polls_seen, overflow=self.overflow,
                    trace_errors=self.trace_errors, helper_returned=self.finished == 1,
                    trace_complete=self.overflow == self.trace_errors == 0 and native_causal,
                    native_wake_values_available=native_available,
                    native_wake_values_causal=native_causal,
                    owner_observation='callback after internal Future completion; not exact publication',
                    native_woke_source='callback return value, not independently authenticated here',
                    unknown_sentinel=UNKNOWN, added_trace_cost_measured=False,
                    diagnostic_only=True, hardware_opened=False,
                    timing_admission_eligible=False, active_output_eligible=False)


def _replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError('Exact source instrumentation anchor required')
    return text.replace(before, after, 1)


def build_traced_helper(baseline_module, source=DEFAULT_SOURCE):
    """Build an isolated function from exact source; imports/opens no device.

    This is an explicit experiment API, not a monkeypatch. Source and imported
    baseline bytecode must match. Normal function objects/globals are unchanged.
    """
    raw = _read_source(source)
    tree = ast.parse(raw)
    selected = {}
    for name in ('_await_owned_ready', '_readiness_poll_target'):
        node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
        original = getattr(baseline_module, name)
        space = dict(original.__globals__)
        exec(compile(ast.Module(body=[node], type_ignores=[]), original.__code__.co_filename, 'exec'), space)
        if space[name].__code__ != original.__code__:
            raise ValueError('Imported baseline bytecode differs from fixed source')
        selected[name] = (node, original)
    node, baseline = selected['_await_owned_ready']
    text = ast.get_source_segment(raw.decode(), node)
    text = _replace_once(text, 'def _await_owned_ready(', 'def _await_owned_ready_observed(')
    text = _replace_once(text, 'thread_clock=time.thread_time_ns):', 'thread_clock=time.thread_time_ns,trace=None):')
    text = _replace_once(text, "    if set(futures)!=set(dual.SCOPES)",
        "    if trace is None:\n"
        "        return _original_ready(futures,validation_future,phase=phase,deadline_ns=deadline_ns,\n"
        "            deadline_wait=deadline_wait,clock=clock,check=check,thread_clock=thread_clock)\n"
        "    if type(trace) is not _trace_class:raise ValueError('Exact fixed trace required')\n"
        "    if set(futures)!=set(dual.SCOPES)")
    text = _replace_once(text, '    owner_count=len(owners)',
        '    owner_count=len(owners)\n    trace.bind(owners,clock,thread_clock)')
    text = _replace_once(text, '        while True:\n', '        while True:\n            trace.next_poll()\n')
    text = _replace_once(text, "            stage='guard_check'\n            check()",
        "            trace.stamp(2)\n            stage='guard_check'\n            trace.stamp(4)\n"
        "            try:check()\n            finally:trace.stamp(6)")
    text = _replace_once(text, '            decision_ns=now\n',
        '            decision_ns=now\n            trace.number(8,now);trace.number(9,ready_count)\n')
    text = _replace_once(text, '                decision_ns=end\n',
        '                decision_ns=end\n                trace.number(16,end)\n')
    text = _replace_once(text, "                return {'mode':", "                trace.finished=1\n                return {'mode':")
    text = _replace_once(text, '                try:deadline_wait(wake)',
        '                trace.number(10,wake);trace.stamp(11)\n                try:trace.native_value(deadline_wait(wake))')
    text = _replace_once(text, '                returned=clock()\n',
        '                returned=clock()\n                trace.number(14,returned);trace.return_cpu()\n')
    text = _replace_once(text, '    except BaseException as error:\n',
        '    except BaseException as error:\n        trace.finished=2\n')
    namespace = dict(baseline.__globals__)
    namespace.update(_original_ready=baseline, _trace_class=FixedReadinessTrace)
    exec(compile(text, str(Path(__file__).resolve()) + ':isolated-observed', 'exec'), namespace)
    return namespace['_await_owned_ready_observed']


def plan(source=DEFAULT_SOURCE):
    _read_source(source)
    return dict(schema='singularitydog.readiness-causal-trace-plan.v1', status='PLAN_ONLY',
                baseline_sha256=BASELINE_SHA256, default_delegates_to_original=True,
                existing_callers_changed=False, deployed=False, library_loaded=False,
                hardware_opened=False, timing_admission_eligible=False, active_output_eligible=False,
                proposed_fields=list(FIELDS), owner_fields=list(OWNER_FIELDS),
                limitation='Added clock and callback overhead must be measured separately; this is not a cause finding.')


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    args = parser.parse_args()
    print(json.dumps(plan(args.source), sort_keys=True, allow_nan=False))
