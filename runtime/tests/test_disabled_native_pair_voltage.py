"""Explicit full-charge STOP-only selection; synthetic files and buffers only.

No native library, model, socket, serial device or hardware approval is used.
The lifecycle evidence below is mocked and cannot establish real placement.
"""
import contextlib
import hashlib
import io
import struct
import threading
import time
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import native_pipeline_benchmark as benchmark
from singularitydog_hw import policy_active_fk
import test_disabled_native_pair_candidate as candidate_fixture
from test_native_pipeline_benchmark import Observer
from test_native_voltage_fast_pipeline import PreparedSession, WaitForVoltageDevice
from test_native_voltage_overlap import OverlapObserver


class NativePairVoltagePlanTests(unittest.TestCase):
    def setUp(self):
        self.recipe = candidate_fixture.DisabledNativePairCLITests()
        self.recipe.setUp()
        self.addCleanup(self.recipe.doCleanups)

    def select_voltage(self, maximum):
        self.recipe.fixture.data['voltage_max_v'] = maximum
        self.recipe.fixture.path = self.recipe.fixture.fixture.fixture.seal()
        argv = self.recipe.replace('--active-fk-profile-sha256', hashlib.sha256(
            self.recipe.fixture.path.read_bytes()).hexdigest())
        return argv + ['--voltage-max-v', str(maximum)]

    def assert_plan_without_load(self, argv):
        with patch.object(active, 'load_library') as native, \
             patch.object(benchmark.native, 'load_library') as diagnostic, \
             patch.object(policy_active_fk, 'diagnostic_load') as model, \
             patch.object(benchmark.imu, 'ICM20948') as device:
            result, plan = self.recipe.invoke(argv)
        self.assertEqual(result, 0)
        for loader in (native, diagnostic, model, device):loader.assert_not_called()
        self.assertFalse(plan['output_allowed'])
        self.assertFalse(plan['enable_available'])
        self.assertFalse(plan['learned_targets_sent'])
        self.assertEqual(plan['type1_requests_per_cycle'], 0)
        self.assertEqual(plan['requests_per_cycle'], 26)
        self.assertEqual(plan['native_phase_pair_plan']['paired_phases'], 'output_stop_proxy_only')
        return plan

    def test_default_stays_42_without_explicit_full_charge_selection(self):
        plan = self.assert_plan_without_load(self.recipe.argv)
        self.assertEqual(plan['voltage_max_v'], 42)
        self.assertEqual(plan['voltage_range_v'], [35., 42.])

    def test_explicit_43_plan_supports_only_the_same_5_or_501_recipe(self):
        argv = self.select_voltage(43)
        for cycles in (5, 501):
            with self.subTest(cycles=cycles):
                selected = argv[:]
                selected[selected.index('--cycles') + 1] = str(cycles)
                plan = self.assert_plan_without_load(selected)
                self.assertEqual(plan['cycles'], cycles)
                self.assertEqual(plan['startup_cycle_allowance'], 1)
                self.assertEqual(plan['voltage_max_v'], 43)
                self.assertEqual(plan['voltage_range_v'], [35., 43.])
                self.assertEqual(plan['request_gap_us'], 900)
                self.assertEqual(plan['window'], 3)

    def test_cli_43_cannot_select_a_pinned_42_profile(self):
        with contextlib.redirect_stderr(io.StringIO()), \
             patch.object(active, 'load_library') as native, \
             patch.object(benchmark.imu, 'ICM20948') as device, \
             self.assertRaises(SystemExit):
            self.recipe.invoke(self.recipe.argv + ['--voltage-max-v', '43'])
        native.assert_not_called(); device.assert_not_called()

    def test_cli_rejects_unsupported_and_non_numeric_voltage_before_load(self):
        for value in ('44', 'True', 'nan'):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()), \
                 patch.object(active, 'load_library') as native, \
                 self.assertRaises(SystemExit):
                self.recipe.invoke(self.recipe.argv + ['--voltage-max-v', value])
            native.assert_not_called()


def mocked_candidate(sessions):
    # Preserve the production exact-class/owned-session admission check without
    # running the constructor or loading its native library.
    candidate = object.__new__(benchmark._DisabledNativePairCandidate)
    candidate.sessions = sessions
    candidate.configure_owners = Mock()
    candidate.close = Mock()
    candidate.exchange_stop_proxy = Mock(side_effect=AssertionError('Unexpected proxy STOP'))
    candidate.evidence = Mock(return_value={
        'journal': [], 'errors': [], 'all_phases_joined': True,
        'owner_placement_verified': False, 'owner_settings_restored': False,
        'coordinator_placement_verified': False, 'coordinator_settings_restored': False,
        'source_files_unchanged': True})
    return candidate


def pair_options(candidate, **extra):
    return dict(mode='stop-proxy', cycles=5, native_phase_pair_candidate=candidate,
        record_storage='trace', main_thread_cpu=4, output_dispatch_trace=True,
        defer_gc_during_cycles=True, pre_cycle_policy_prepare=lambda: None,
        post_pin_policy_prepare=lambda: None, v3_voltage_proxy=True,
        v3_voltage_overlap=True, v3_voltage_validation_overlap=True,
        v3_voltage_fast_pipeline=True, prepare_voltage_before_feedback_publication=True,
        inference_thread_cpu_trace=True, absolute_epoch_cadence=True,
        exclude_policy_cpu_from_workers=True, startup_cycle_allowance=1,
        deadline_wait=lambda deadline: time.sleep(max(0., (deadline - time.monotonic_ns()) / 1e9)),
        worker_initializer=lambda: None, **extra)


class NativePairVoltageCollectorTests(unittest.TestCase):
    def test_43_passes_pair_admission_for_5_and_501_before_worker_startup(self):
        for cycles in (5, 501):
            sessions = {'front': object(), 'rear': object()}
            candidate = mocked_candidate(sessions)
            options = pair_options(candidate, voltage_max_v=43)
            options['cycles'] = cycles
            with self.subTest(cycles=cycles), patch.object(benchmark, '_RecordTrace',
                    side_effect=RuntimeError('SYNTHETIC stop after admission')) as trace:
                report, raw = benchmark.collect(sessions, None, Observer(), **options)
            trace.assert_called_once()
            self.assertIn('SYNTHETIC stop after admission', ' '.join(report['errors']))
            self.assertEqual(report['voltage_range_v'], [35., 43])
            self.assertEqual(raw, [])
            candidate.close.assert_called_once()
            candidate.configure_owners.assert_not_called()
            candidate.exchange_stop_proxy.assert_not_called()
            self.assertNotIn('native_phase_pair_proof', report)

    def test_invalid_collect_voltage_is_rejected_before_storage_or_any_output(self):
        sessions = {'front': object(), 'rear': object()}
        candidate = mocked_candidate(sessions)
        for maximum in (44, True, float('nan')):
            with self.subTest(maximum=maximum), patch.object(benchmark, '_RecordTrace') as trace, \
                 self.assertRaisesRegex(ValueError, 'Voltage maximum'):
                benchmark.collect(sessions, None, Observer(),
                    **pair_options(candidate, voltage_max_v=maximum))
            trace.assert_not_called()
        candidate.configure_owners.assert_not_called()
        candidate.exchange_stop_proxy.assert_not_called()

    def test_raw_high_voltage_aborts_before_proxy_stop_with_failed_buffers_retained(self):
        for maximum, value in ((42, 42.10703659057617), (43, 43.01)):
            with self.subTest(maximum=maximum, value=value):
                started = (threading.Event(), threading.Event())
                release = threading.Event()
                sessions = {scope: PreparedSession(started[index], release,
                    voltage_v=value if scope == 'front' else 40.)
                    for index, scope in enumerate(('front', 'rear'))}
                observer = OverlapObserver(started, release)
                candidate = mocked_candidate(sessions)
                masks = {}
                def get_affinity(pid):
                    return set(masks.setdefault(threading.get_native_id(), {0, 1, 2, 3, 4}))
                def set_affinity(pid, cpus):
                    masks[threading.get_native_id()] = set(cpus)
                with patch.object(benchmark.os, 'sched_getaffinity', get_affinity, create=True), \
                     patch.object(benchmark.os, 'sched_setaffinity', set_affinity, create=True):
                    report, raw = benchmark.collect(sessions, WaitForVoltageDevice(started), observer,
                        **pair_options(candidate, voltage_max_v=maximum))
                self.assertEqual(report['status'], 'ABORTED')
                self.assertEqual(report['cycles_completed'], 0)
                self.assertIn(f'Voltage outside 35..{maximum} V before proxy STOP', ' '.join(report['errors']))
                self.assertFalse(report['motor_enable_sent']); self.assertFalse(report['learned_targets_sent'])
                candidate.configure_owners.assert_called_once()
                candidate.exchange_stop_proxy.assert_not_called()
                candidate.close.assert_called_once()
                self.assertEqual(len(raw), 1)
                row = benchmark._serialize(raw)[0]
                self.assertEqual(row['output'], {})
                self.assertNotIn('observed', row)
                self.assertTrue(observer.invalid)
                self.assertEqual(set(row['voltage']), {'front', 'rear'})
                for scope in ('front', 'rear'):
                    self.assertEqual(len(row['acquired'][scope]['records']), 6)
                    self.assertEqual(len(row['voltage'][scope]['records']), 1)
                    self.assertTrue(all(r['written'] == r['received'] == 17
                        for phase in ('acquired', 'voltage') for r in row[phase][scope]['records']))
                self.assertTrue(all(session.phases == ['feedback', 'voltage'] for session in sessions.values()))
                # Read the actual float32 reply, keeping the failed raw bytes.
                record = row['voltage']['front']['records'][0]
                self.assertEqual(record['written'], 17); self.assertEqual(record['received'], 17)
                self.assertEqual(bytes.fromhex(record['rx_hex'])[11:15], struct.pack('<f', value))


if __name__ == '__main__':unittest.main()
