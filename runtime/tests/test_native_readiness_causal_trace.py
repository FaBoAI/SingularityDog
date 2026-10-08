"""Fake clocks and Futures only. No library, model, CAN, IMU or SSH."""
from concurrent.futures import Future
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

FILE = Path(__file__).resolve().parents[1] / 'experiments/native_readiness_causal_trace/candidate.py'
SPEC = importlib.util.spec_from_file_location('readiness_causal_candidate', FILE)
candidate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(candidate)
BASELINE_SOURCE = Path(__file__).resolve().parent / 'fixtures/native_readiness_causal_trace/runtime/singularitydog_hw/native_pipeline_benchmark.py'
BASELINE_BYTES = BASELINE_SOURCE.read_bytes()
if hashlib.sha256(BASELINE_BYTES).hexdigest() != candidate.BASELINE_SHA256:
    raise ValueError('Exact historical K37 test fixture required')
BASELINE_SPEC = importlib.util.spec_from_file_location('singularitydog_hw._test_readiness_k37_baseline', BASELINE_SOURCE)
baseline = importlib.util.module_from_spec(BASELINE_SPEC)
exec(compile(BASELINE_BYTES, str(BASELINE_SOURCE), 'exec'), baseline.__dict__)


class Clock:
    def __init__(self, now=850_000):
        self.now = now
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.now


class CausalTraceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.helper = staticmethod(candidate.build_traced_helper(baseline, BASELINE_SOURCE))

    def test_imported_helpers_use_authenticated_historical_file(self):
        self.assertEqual(Path(baseline.__file__),BASELINE_SOURCE)
        self.assertEqual(Path(baseline._await_owned_ready.__code__.co_filename),BASELINE_SOURCE)
        self.assertEqual(hashlib.sha256(BASELINE_BYTES).hexdigest(),candidate.BASELINE_SHA256)
        self.assertIsNot(baseline,sys.modules.get('singularitydog_hw.native_pipeline_benchmark'))

    def owners(self, ready=False):
        result = {name: Future() for name in ('front', 'rear')}
        if ready:
            for f in result.values():
                f.set_result('owned result')
        return result

    def call(self, owners, clock, native=None, trace=None, validation=None,
             check=lambda: None, cpu=lambda: 100):
        return self.helper(owners, validation, phase='Proxy output', deadline_ns=1_000_000,
                           deadline_wait=native, clock=clock, check=check,
                           thread_clock=cpu, trace=trace)

    def ready_native(self, owners, clock, native_wake_delta=0, python_delta=0):
        def native(wake):
            actual = wake + native_wake_delta
            clock.now = actual + python_delta
            for f in owners.values():
                f.set_result('owned result')
            return actual
        return native

    def row(self, trace, index=0):
        proof = trace.export()
        return dict(zip(proof['fields'], proof['rows'][index]))

    def test_none_delegates_exact_original_clocks_arguments_and_result(self):
        owners = self.owners(True)
        clock = Clock()
        cpu = Mock(return_value=100)
        original = baseline._await_owned_ready(owners, None, phase='Proxy output',
            deadline_ns=1_000_000, clock=clock, thread_clock=cpu)
        original_calls = (clock.calls, cpu.call_count)
        clock.calls = 0
        cpu.reset_mock()
        with patch.object(candidate.FixedReadinessTrace, 'bind', side_effect=AssertionError('trace selected')):
            actual = self.call(owners, clock, cpu=cpu)
        self.assertEqual(actual, original)
        self.assertEqual((clock.calls, cpu.call_count), original_calls)

    def test_opt_in_records_native_wake_python_return_and_owner_callback(self):
        owners = self.owners()
        clock = Clock()
        trace = candidate.FixedReadinessTrace(8)
        result = self.call(owners, clock, self.ready_native(owners, clock, 5_000, 17_000), trace)
        row = self.row(trace)
        self.assertEqual(row['planned_wake_ns'], 900_000)
        self.assertEqual(row['native_woke_return_value_ns'], 905_000)
        self.assertEqual(row['python_return_ns'], 922_000)
        self.assertEqual(result['end_ns'], 922_000)
        proof = trace.export()
        self.assertEqual(proof['polls_seen'], 2)
        self.assertTrue(proof['trace_complete'])
        self.assertEqual([r[4] for r in proof['owners']], [922_000, 922_000])
        self.assertEqual([r[6] for r in proof['owners']], [0, 0])
        self.assertIn('not exact publication', proof['owner_observation'])

    def test_three_futures_including_validation_separately_observed(self):
        owners = self.owners(True)
        validation = Future()
        clock = Clock()
        trace = candidate.FixedReadinessTrace()
        def native(wake):
            clock.now = wake
            validation.set_result('proof')
            return wake
        self.call(owners, clock, native, trace, validation)
        proof = trace.export()
        self.assertEqual(len(proof['owners']), 3)
        self.assertEqual([r[6] for r in proof['owners']], [1, 1, 0])
        self.assertEqual(proof['owners'][2][4], 900_000)

    def test_guard_wall_and_thread_cpu_are_independent_fields(self):
        owners = self.owners(True)
        clock = Clock()
        cpu_clock = Clock(5_000)
        trace = candidate.FixedReadinessTrace()
        def check():
            clock.now += 90_000
            cpu_clock.now += 3_000
        self.call(owners, clock, trace=trace, check=check, cpu=cpu_clock)
        row = self.row(trace)
        self.assertEqual(row['guard_after_ns'] - row['guard_before_ns'], 90_000)
        self.assertEqual(row['guard_after_thread_cpu_ns'] - row['guard_before_thread_cpu_ns'], 3_000)
        self.assertEqual(row['decision_ns'], 940_000)

    def test_deadline_equality_and_late_wake_are_still_rejected(self):
        for actual in (1_000_000, 1_000_001):
            owners = self.owners()
            clock = Clock(960_000)
            trace = candidate.FixedReadinessTrace()
            def native(wake):
                clock.now = actual
                for f in owners.values(): f.set_result('late')
                return actual
            with self.subTest(actual=actual), self.assertRaises(TimeoutError) as caught:
                self.call(owners, clock, native, trace)
            self.assertEqual(caught.exception.readiness_poll_failure['decision_ns'], actual)
            self.assertFalse(trace.export()['helper_returned'])

    def test_r37_measured_overshoot_is_preserved_as_failure(self):
        owners = self.owners()
        clock = Clock(952_681)
        trace = candidate.FixedReadinessTrace()
        def native(wake):
            self.assertEqual(wake, 976_340)
            clock.now = 1_016_074
            for f in owners.values(): f.set_result('before eventual decision')
            return 977_000  # Synthetic: actual R37 native wake was not saved.
        with self.assertRaises(TimeoutError):
            self.call(owners, clock, native, trace)
        row = self.row(trace)
        self.assertEqual(row['native_woke_return_value_ns'], 977_000)
        self.assertEqual(row['python_return_ns'], 1_016_074)
        self.assertFalse(trace.export()['timing_admission_eligible'])

    def test_original_ready_error_identity_wins_over_pending_and_deadline(self):
        owners = self.owners()
        error = OSError('owner failed')
        owners['front'].set_exception(error)
        clock = Clock(1_000_001)
        trace = candidate.FixedReadinessTrace()
        native = Mock()
        with self.assertRaises(OSError) as caught:
            self.call(owners, clock, native, trace)
        self.assertIs(caught.exception, error)
        native.assert_not_called()
        owners['rear'].set_result('cleanup')
        self.assertFalse(trace.export()['helper_returned'])

    def test_original_owner_error_wins_over_native_cancel(self):
        owners = self.owners()
        trace = candidate.FixedReadinessTrace()
        error = OSError('owner error')
        def native(wake):
            owners['front'].set_exception(error)
            raise KeyboardInterrupt('native cancel')
        with self.assertRaises(OSError) as caught:
            self.call(owners, Clock(), native, trace)
        self.assertIs(caught.exception, error)
        owners['rear'].set_result('cleanup')
        self.assertFalse(trace.export()['helper_returned'])

    def test_guard_error_and_cancel_identity_preserved(self):
        for error in (RuntimeError('guard changed'), KeyboardInterrupt('cancel')):
            trace = candidate.FixedReadinessTrace()
            with self.subTest(error=type(error).__name__), self.assertRaises(type(error)) as caught:
                self.call(self.owners(True), Clock(), trace=trace, check=Mock(side_effect=error))
            self.assertIs(caught.exception, error)
            row = self.row(trace)
            self.assertEqual(row['guard_before_ns'], row['guard_after_ns'])
            self.assertEqual(row['decision_ns'], -1)

    def test_cancelled_owner_is_not_success(self):
        owners = self.owners(True)
        owners['rear'] = Future()
        owners['rear'].cancel()
        trace = candidate.FixedReadinessTrace()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            self.call(owners, Clock(), trace=trace)
        self.assertFalse(trace.export()['helper_returned'])

    def test_noncausal_native_return_still_rejects(self):
        owners = self.owners()
        clock = Clock()
        trace = candidate.FixedReadinessTrace()
        def native(wake):
            clock.now = wake - 1
            for f in owners.values(): f.set_result('early')
            return wake
        with self.assertRaisesRegex(ValueError, 'before requested wake'):
            self.call(owners, clock, native, trace)
        self.assertEqual(self.row(trace)['python_return_ns'], 899_999)

    def test_native_none_return_is_unknown_and_does_not_change_legacy_semantics(self):
        owners = self.owners()
        clock = Clock()
        trace = candidate.FixedReadinessTrace()
        def native(wake):
            clock.now = wake
            for f in owners.values(): f.set_result('ready')
        self.call(owners, clock, native, trace)
        self.assertEqual(self.row(trace)['native_woke_return_value_ns'], -1)
        self.assertEqual(trace.trace_errors, 0)
        self.assertFalse(trace.export()['native_wake_values_available'])
        self.assertFalse(trace.export()['trace_complete'])

    def test_noncausal_callback_claim_is_flagged_without_changing_control_result(self):
        for claimed in (899_999, 900_001):
            owners = self.owners()
            clock = Clock()
            trace = candidate.FixedReadinessTrace()
            def native(wake):
                clock.now = wake
                for f in owners.values(): f.set_result('ready')
                return claimed
            self.call(owners, clock, native, trace)
            self.assertTrue(trace.export()['helper_returned'])
            self.assertTrue(trace.export()['native_wake_values_available'])
            self.assertFalse(trace.export()['native_wake_values_causal'])
            self.assertFalse(trace.export()['trace_complete'])

    def test_invalid_native_return_evidence_never_replaces_original_acceptance(self):
        owners = self.owners()
        clock = Clock()
        trace = candidate.FixedReadinessTrace()
        def native(wake):
            clock.now = wake
            for f in owners.values(): f.set_result('ready')
            return float('nan')
        self.call(owners, clock, native, trace)
        proof = trace.export()
        self.assertTrue(proof['helper_returned'])
        self.assertFalse(proof['trace_complete'])
        json.dumps(proof, allow_nan=False)

    def test_overflow_keeps_fixed_buffers_and_original_result(self):
        owners = self.owners()
        clock = Clock(1_000)
        trace = candidate.FixedReadinessTrace(1)
        buffers = (id(trace._rows), len(trace._rows), id(trace._owners), len(trace._owners))
        count = [0]
        def native(wake):
            count[0] += 1
            clock.now = wake
            if count[0] == 3:
                for f in owners.values(): f.set_result('ready')
            return wake
        result = self.call(owners, clock, native, trace)
        self.assertEqual(result['wait_calls'], 3)
        self.assertEqual(buffers, (id(trace._rows), len(trace._rows), id(trace._owners), len(trace._owners)))
        self.assertEqual(trace.export()['overflow'], 3)
        self.assertEqual(len(trace.export()['rows']), 1)

    def test_trace_clock_error_invalidates_only_trace_and_does_not_mask_guard_error(self):
        trace = candidate.FixedReadinessTrace()
        original_stamp = trace._stamp
        def stamp(target, index):
            original_clock = trace._clock
            trace._clock = Mock(side_effect=OSError('measurement failed'))
            try: original_stamp(target, index)
            finally: trace._clock = original_clock
        with patch.object(trace, '_stamp', side_effect=stamp):
            error = RuntimeError('real guard error')
            with self.assertRaises(RuntimeError) as caught:
                self.call(self.owners(True), Clock(), trace=trace, check=Mock(side_effect=error))
        self.assertIs(caught.exception, error)
        self.assertFalse(trace.export()['trace_complete'])

    def test_export_pending_owner_and_reuse_are_rejected(self):
        owners = self.owners()
        trace = candidate.FixedReadinessTrace()
        with self.assertRaises(RuntimeError):
            self.call(owners, Clock(), trace=trace, check=Mock(side_effect=RuntimeError('fail')))
        with self.assertRaises(ValueError): trace.export()
        for f in owners.values(): f.set_result('cleanup')
        trace.export()
        with self.assertRaises(ValueError): self.call(self.owners(True), Clock(), trace=trace)

    def test_owner_thread_rejection_does_not_start_polling(self):
        trace = candidate.FixedReadinessTrace()
        caught = []
        def other():
            try: self.call(self.owners(True), Clock(), trace=trace)
            except ValueError as error: caught.append(error)
        thread = threading.Thread(target=other)
        thread.start(); thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(caught), 1)
        self.assertEqual(trace.polls_seen, 0)

    def test_export_is_owned_copy(self):
        trace = candidate.FixedReadinessTrace()
        self.call(self.owners(True), Clock(), trace=trace)
        first = trace.export()
        first['rows'][0][0] = -999
        first['owners'][0][0] = -999
        self.assertNotEqual(trace.export()['rows'][0][0], -999)
        self.assertNotEqual(trace.export()['owners'][0][0], -999)

    def test_incompatible_source_or_imported_function_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'changed.py'
            p.write_bytes(BASELINE_BYTES + b'\n# changed\n')
            with self.assertRaises(ValueError): candidate.build_traced_helper(baseline, p)
        with patch.object(baseline, '_readiness_poll_target', lambda a, b: b):
            with self.assertRaises(ValueError): candidate.build_traced_helper(baseline, BASELINE_SOURCE)

    def test_plan_opens_no_native_library_and_cli_has_no_execute(self):
        source_before = hashlib.sha256(BASELINE_SOURCE.read_bytes()).hexdigest()
        with patch.object(candidate, 'build_traced_helper', side_effect=AssertionError('must not build')):
            plan = candidate.plan(BASELINE_SOURCE)
        self.assertEqual(plan['status'], 'PLAN_ONLY')
        self.assertFalse(plan['hardware_opened'])
        result = subprocess.run([sys.executable, '-B', str(FILE), '--execute'],
            capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(hashlib.sha256(BASELINE_SOURCE.read_bytes()).hexdigest(), source_before)

    def test_invalid_capacity_rejected(self):
        for value in (0, 2049, True, 1.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                candidate.FixedReadinessTrace(value)


if __name__ == '__main__': unittest.main()
