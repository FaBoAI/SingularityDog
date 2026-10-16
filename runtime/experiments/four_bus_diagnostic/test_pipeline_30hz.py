"""30Hz20ms writes/33ms proxy replies; factory-only mocks, no devices/model."""
from dataclasses import replace
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_model_30hz as model30
from singularitydog_hw.policy_clock import PolicyClock, control_clock_payload, validate_control_clock
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.policy_output_runtime import decode_records
from . import pipeline
from . import test_pipeline as legacy


def inline_model_plan():
    # Explicit mock paths; the pipeline only validates inline contract fields.
    # Foreground load must separately reverify the real seven files. This mock
    # plan never opens a library/model and makes no live timing qualification.
    return model30._plan(model30.bundle_artifacts('/file-only-mock30'),
        {'source_manifest': dict.fromkeys(model30._SOURCE_NAMES),
         'export': {'evaluation_all_intervals_completed': False}})


class Pipeline30Tests(unittest.TestCase):
    def setUp(self):
        self.fixture = legacy.PipelineTests('test_default_plan_does_not_call_factories_or_load_models')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.clock_payload = control_clock_payload(30)
        self.model_plan = inline_model_plan()
        self.fixture.config = replace(self.fixture.config, control_clock=self.clock_payload,
                                      policy30_model=self.model_plan)
        self.original_factory = self.fixture.factory
        self.fixture.factory = self.selected_factory

    def selected_factory(self, group):
        adapter = self.original_factory(group)
        is30 = pipeline._control(self.fixture.config).policy_hz == 30
        adapter.separate_write_deadline = is30
        ordinary = adapter.output_stop
        def output(*, deadline_ns, check, write_deadline_ns=None):
            self.assertIs(write_deadline_ns is not None, is30)
            batch = ordinary(deadline_ns=deadline_ns, check=check)
            if not is30:
                return batch
            self.assertLessEqual(write_deadline_ns, deadline_ns-13_333_334)
            controls = self.fixture.controls
            if controls.get('proxy_partial_write'):
                last = batch.records[-1]
                last.start_ns = time.monotonic_ns()
                last.finish_ns = last.start_ns+1
                last.written, last.received = 8, 0
                raise RuntimeError('File-only partial proxy write failure')
            offset = controls.get('proxy_reply_from_release_ns')
            if offset is not None:
                reply = deadline_ns-33_333_334+offset
                while time.monotonic_ns() < reply:
                    time.sleep(max(0., (reply-time.monotonic_ns())/1e9))
                for row in batch.records:
                    row.received_ns = time.monotonic_ns()
            if controls.get('proxy_late_write'):
                # Original-looking raw evidence at the exact forbidden host
                # cutoff; this mock does not claim the native write ABI ran.
                for row in batch.records:
                    row.finish_ns = write_deadline_ns
                    row.received_ns = max(row.received_ns, write_deadline_ns+1)
            batch.stats.end_ns = max(time.monotonic_ns(), max(row.received_ns for row in batch.records))
            batch = replace(batch, rows=decode_records((batch.records, batch.stats)),
                record_image=bytes(batch.records), stats_image=bytes(batch.stats),
                completed_ns=batch.stats.end_ns, write_deadline_ns=write_deadline_ns)
            adapter.batches[-1] = batch
            return batch
        adapter.output_stop = output
        return adapter

    def run_case(self, observer=None, *, cycles=1):
        self.fixture.config = replace(self.fixture.config, cycles=cycles)
        def release(when):
            delay = (when-time.monotonic_ns())/1e9
            if delay > 0:
                time.sleep(delay)
            return time.monotonic_ns()
        return pipeline.run(self.fixture.config, factory=self.fixture.factory,
            imu_read=self.fixture.imu, observer=observer or legacy.Observer(),
            snapshot_builder=legacy.snapshot_builder,
            check_current=self.fixture.check, cancel_io=self.fixture.canceled.set,
            model_setup=lambda o, **counts: dict(counts, reset_verified=True, file_only_mock_model=True),
            worker_scope=self.fixture.worker_scope, main_scope=self.fixture.main_scope,
            backend_usage={'kind': 'injected_file_only_mock'}, release_wait=release, execute=True)

    def test_new_plan_preserves20ms_writes_and_explicit33ms_proxy_reply_budget(self):
        result = pipeline.run(self.fixture.config,
                              factory=lambda _: self.fail('PLAN opened device'))
        self.assertEqual(result['schema'], pipeline.SCHEMA_30HZ)
        self.assertEqual(result['control_clock'], self.clock_payload)
        self.assertEqual(result['policy30_model'], self.model_plan)
        self.assertEqual(result['absolute_deadline_ns'], 33_333_334)
        self.assertEqual(result['input_to_stop_proxy_tx_budget_ns'], 20_000_000)
        self.assertEqual(result['acquisition_budget_ns'], 20_000_000)
        self.assertEqual(result['proxy_exchange_budget_ns'], 33_333_334)
        self.assertEqual(result['proxy_reply_budget_ns'], 33_333_334)
        self.assertEqual(result['proxy_write_deadline_abi_required'], 1)
        self.assertFalse(result['diagnostic_reply_budget_unchanged'])
        self.assertFalse(result['diagnostic_type1_33ms_reply_qualification'])
        self.assertEqual(result['request_gap_ns'], 900_000)
        self.assertEqual(result['request_window'], 3)
        self.assertEqual(result['per_cycle_requests'], 28)

    def test_clock_and_new_model_must_be_selected_together(self):
        for clock, model in ((None, self.model_plan), (control_clock_payload(50), self.model_plan),
                             (self.clock_payload, None)):
            with self.subTest(clock=clock), self.assertRaises(ValueError):
                pipeline.plan(replace(self.fixture.config, control_clock=clock, policy30_model=model))
        wrong = dict(self.model_plan, output_allowed=True)
        with self.assertRaisesRegex(ValueError, 'Inline30Hz'):
            pipeline.plan(replace(self.fixture.config, policy30_model=wrong))

    def test_run_explicit_options_cannot_override_a_different_config(self):
        with self.assertRaisesRegex(ValueError, 'selections differ'):
            pipeline.run(self.fixture.config, control_clock=control_clock_payload(50),
                         policy30_model=self.model_plan)
        legacy = replace(self.fixture.config, control_clock=None, policy30_model=None)
        result = pipeline.run(legacy, control_clock=self.clock_payload, policy30_model=self.model_plan)
        self.assertEqual(result['schema'], pipeline.SCHEMA_30HZ)

    def test_four_cycles_use_exact_rational_schedule_and_zero_output_gains(self):
        result = self.run_case(cycles=4)
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])
        self.assertEqual(result['completed_cycles'], 4)
        clock = validate_control_clock(self.clock_payload)
        epoch = result['records'][0]['scheduled_release_ns']
        for index, row in enumerate(result['records']):
            self.assertEqual(row['scheduled_release_ns'], clock.release_ns(epoch, index))
            self.assertEqual(row['scheduled_slot_deadline_ns'], clock.release_ns(epoch, index+1))
            self.assertEqual(row['original_deadline_ns'], row['release_ns']+33_333_334)
            self.assertLessEqual(row['input_to_stop_proxy_tx_deadline_ns'], row['release_ns']+20_000_000)
            self.assertLess(row['final_proxy_host_write_ns'], row['input_to_stop_proxy_tx_deadline_ns'])
            self.assertEqual(row['proxy_reply_deadline_ns'], row['release_ns']+33_333_334)
            self.assertEqual(set(row['proxy_host_write_records']), set(pipeline.PORTS))
            self.assertFalse(row['retained_proxy_host_writes_are_admission'])
            self.assertTrue(row['input_to_stop_proxy_tx_20ms_met'])
            self.assertEqual(row['actual_request_count'], 28)
        releases = [row['scheduled_release_ns'] for row in result['records']]
        self.assertEqual([b-a for a, b in zip(releases, releases[1:])],
                         [33_333_333, 33_333_333, 33_333_334])
        self.assertFalse(result['motor_enable_sent'])
        self.assertFalse(result['positive_gain_sent'])
        self.assertFalse(result['learned_targets_sent'])
        for adapter in self.fixture.adapters.values():
            for wire in adapter.wires:
                self.assertIn(ATParser().feed(wire)[0].kind, (4, 17))

    def test_longer_cadence_does_not_allow_inference_over20ms(self):
        result = self.run_case(legacy.Observer(lambda _: time.sleep(.021)))
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(result['completed_cycles'], 0)
        self.assertFalse(self.fixture.controls.get('output_ports'))
        self.assertTrue(result['failure_retained'])

    def test_stale_input_21ms_rejected_even_before33ms_cycle_end(self):
        original = self.fixture.imu
        def stale():
            value = original()
            value['read_started_monotonic_ns'] -= 21_000_000
            value['read_finished_monotonic_ns'] -= 21_000_000
            return value
        self.fixture.imu = stale
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(self.fixture.controls.get('output_ports'))
        self.assertIn('IMU', result['primary_error']['message'])

    def test_completed_proxy_replies_may_leave_postprocessing_until33ms(self):
        original = pipeline._take
        completed_outputs = 0
        def coordinator_cost(*args, **kwargs):
            nonlocal completed_outputs
            result = original(*args, **kwargs)
            if getattr(result, 'label', None) == 'output_stop':
                completed_outputs += 1
                if completed_outputs == 4:
                    # Coordinator cost remains bounded by the33ms cycle.
                    time.sleep(.021)
            return result
        with patch.object(pipeline, '_take', side_effect=coordinator_cost):
            result = self.run_case()
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])
        row = result['records'][0]
        self.assertGreater(row['cycle_end_ns']-row['release_ns'], 20_000_000)
        self.assertLess(row['cycle_end_ns']-row['release_ns'], 33_333_334)
        self.assertTrue(row['input_to_stop_proxy_tx_20ms_met'])

    def test_proxy_ack_at21ms_passes_with_all_raw_host_writes_inside20ms(self):
        self.fixture.controls['proxy_reply_from_release_ns'] = 21_000_000
        result = self.run_case()
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])
        row = result['records'][0]
        for batch in row['output'].values():
            for raw in batch.records:
                self.assertGreaterEqual(raw.received_ns-row['release_ns'], 21_000_000)
                self.assertLess(raw.finish_ns, row['input_to_stop_proxy_tx_deadline_ns'])
                self.assertEqual(raw.deadline_ns, row['proxy_reply_deadline_ns'])
        self.assertFalse(result['diagnostic_type1_33ms_reply_qualification'])

    def test_proxy_raw_write_at20ms_rejected_even_if_reply_is_inside33ms(self):
        self.fixture.controls['proxy_late_write'] = True
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(result['completed_cycles'], 0)
        self.assertIn('host-write exceeded', result['primary_error']['message'])
        row = result['records'][0]
        self.assertEqual(row['final_proxy_host_write_ns'], row['input_to_stop_proxy_tx_deadline_ns'])
        self.assertFalse(row.get('input_to_stop_proxy_tx_20ms_met', False))
        self.assertTrue(row['retained_proxy_host_write_records'])

    def test_raw_batch_native_write_cutoff_must_equal_the_actual_dispatch_cutoff(self):
        group = self.fixture.groups[0]
        reply = time.monotonic_ns()+33_333_334
        write = reply-13_333_334
        batch = replace(legacy.raw_batch(group, 'output_stop', group.ids, reply),
                        write_deadline_ns=write+1)
        with self.assertRaisesRegex(TimeoutError, 'raw write proof differs'):
            pipeline._validate_proxy_host_writes(batch, write, reply)

    def test_failed_partial_write_retains_last_call_finish_separately_from_last_complete_write(self):
        self.fixture.controls['proxy_partial_write'] = True
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('partial proxy write failure', result['primary_error']['message'])
        row = result['records'][0]
        retained = [raw for batches in row['retained_proxy_host_write_records'].values()
                    for batch in batches for raw in batch]
        self.assertEqual(row['final_proxy_host_write_ns'], max(raw['finish_ns'] for raw in retained))
        self.assertEqual(row['final_complete_proxy_host_write_ns'],
                         max(raw['finish_ns'] for raw in retained if raw['written'] == 17))
        self.assertGreater(row['final_proxy_host_write_ns'], row['final_complete_proxy_host_write_ns'])
        self.assertFalse(row.get('input_to_stop_proxy_tx_20ms_met', False))
        self.assertFalse(row['retained_proxy_host_writes_are_admission'])

    def test_proxy_unfinished_owner_at33ms_is_rejected_and_write_evidence_retained(self):
        original_factory = self.fixture.factory
        def delayed_factory(group):
            adapter = original_factory(group)
            original = adapter.output_stop
            def output(**kwargs):
                value = original(**kwargs)
                time.sleep(.034)
                return value
            adapter.output_stop = output
            return adapter
        self.fixture.factory = delayed_factory
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(result['completed_cycles'], 0)
        self.assertFalse(result['diagnostic_type1_33ms_reply_qualification'])
        self.assertTrue(all(value['complete'] for value in result['cleanup'].values()))
        self.assertTrue(result['records'][0]['retained_proxy_host_write_records'])
        self.assertLess(result['records'][0]['final_proxy_host_write_ns'],
                        result['records'][0]['input_to_stop_proxy_tx_deadline_ns'])

    def test_30hz_rejects_old_adapter_before_acquisition_or_proxy_dispatch(self):
        self.fixture.factory = self.original_factory
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('requires separate', result['primary_error']['message'])
        self.assertFalse(self.fixture.controls.get('output_ports'))
        self.assertEqual(result['records'], [])

    def test_50hz_keeps20ms_takeout_and_never_supplies_split_write_argument(self):
        self.fixture.config = replace(self.fixture.config, control_clock=None, policy30_model=None)
        ordinary_factory = self.fixture.factory
        def delayed_factory(group):
            adapter = ordinary_factory(group)
            original = adapter.output_stop
            def output(**kwargs):
                self.assertNotIn('write_deadline_ns', kwargs)
                value = original(**kwargs)
                time.sleep(.021)
                return value
            adapter.output_stop = output
            return adapter
        self.fixture.factory = delayed_factory
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(result['completed_cycles'], 0)
        self.assertNotIn('proxy_reply_budget_ns', result)

    def test_default50hz_plan_schema_and_budget_are_unchanged(self):
        result = pipeline.plan(replace(self.fixture.config, control_clock=None, policy30_model=None))
        self.assertEqual(result['schema'], pipeline.SCHEMA)
        self.assertEqual(result['absolute_deadline_ns'], 20_000_000)
        self.assertNotIn('control_clock', result)
        self.assertNotIn('policy30_model', result)

    def run_advertised(self, observer, *, backend='injected_file_only_mock', model_setup=None):
        return pipeline.run(self.fixture.config, factory=self.fixture.factory,
            imu_read=self.fixture.imu, observer=observer, snapshot_builder=legacy.snapshot_builder,
            check_current=self.fixture.check, cancel_io=self.fixture.canceled.set,
            model_setup=model_setup or (lambda o, **counts: dict(counts, reset_verified=True,
                                                                 file_only_mock_model=True)),
            worker_scope=self.fixture.worker_scope, main_scope=self.fixture.main_scope,
            backend_usage={'kind': backend},
            release_wait=lambda when: max(when, time.monotonic_ns()), execute=True)

    def test_known_legacy_observer_cannot_execute_inside_30hz_config(self):
        observer = legacy.Observer()
        observer._control_clock = None
        with self.assertRaisesRegex(ValueError, 'observer and outer control clock differ'):
            self.run_advertised(observer)
        self.assertEqual(observer.calls, 0)
        self.assertEqual(self.fixture.adapters, {})
        self.assertEqual(self.fixture.events, [])

    def test_known_30hz_observer_cannot_execute_inside_legacy_config(self):
        self.fixture.config = replace(self.fixture.config, control_clock=None, policy30_model=None)
        observer = legacy.Observer()
        observer._control_clock = PolicyClock(30)
        with self.assertRaisesRegex(ValueError, 'observer and outer control clock differ'):
            self.run_advertised(observer)
        self.assertEqual(observer.calls, 0)
        self.assertEqual(self.fixture.adapters, {})
        self.assertEqual(self.fixture.events, [])

    def test_unknown_mock_clock_is_allowed_but_real_backend_requires_advertised_clock(self):
        observer = legacy.Observer()
        with self.assertRaisesRegex(ValueError, 'Real diagnostic observer must advertise'):
            self.run_advertised(observer, backend='genuine_original_active_subset_cpp')
        self.assertEqual(self.fixture.adapters, {})
        self.assertEqual(self.fixture.events, [])
        result = self.run_advertised(observer)
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])

    def test_invalid_advertised_clock_type_rejects_before_device_factory(self):
        observer = legacy.Observer()
        for claimed in (True, 30, self.clock_payload):
            observer._control_clock = claimed
            with self.subTest(claimed=claimed), self.assertRaisesRegex(ValueError, 'immutable PolicyClock'):
                self.run_advertised(observer)
        self.assertEqual(observer.calls, 0)
        self.assertEqual(self.fixture.adapters, {})
        self.assertEqual(self.fixture.events, [])

    def test_matching_advertised_30hz_and_legacy_clocks_remain_usable(self):
        observer = legacy.Observer()
        observer._control_clock = PolicyClock(30)
        result = self.run_advertised(observer)
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])
        # Legacy path with an explicit None retains exactly its former cadence.
        self.fixture.adapters.clear()
        self.fixture.canceled.clear()
        self.fixture.config = replace(self.fixture.config, control_clock=None, policy30_model=None)
        observer = legacy.Observer()
        observer._control_clock = None
        result = self.run_advertised(observer)
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])

    def test_clock_changed_during_warmup_stops_before_any_model_consumption(self):
        observer = legacy.Observer()
        observer._control_clock = PolicyClock(30)
        def mutate(o, **counts):
            o._control_clock = None
            return dict(counts, reset_verified=True, file_only_mock_model=True)
        result = self.run_advertised(observer, model_setup=mutate)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('observer and outer control clock differ', result['primary_error']['message'])
        self.assertEqual(observer.calls, 0)
        self.assertFalse(self.fixture.controls.get('output_ports'))
        self.assertTrue(all(row['complete'] for row in result['cleanup'].values()))


if __name__ == '__main__':
    unittest.main()
