"""Opt-in four-request acquisition: genuine local sockets and factory guards.

No device, model artifact, current physical state or timing approval is used.
All positive native acquisitions keep the original absolute 20ms deadline.
"""
from concurrent.futures import Future
from contextlib import contextmanager
import ctypes as C
from dataclasses import replace
import os
import select
import socket
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from singularitydog_hw import native_active_transport as active
from singularitydog_hw.policy_output_runtime import decode_records
from . import pipeline
from .transport_adapter import Batch, Group, ThreeAxisTransport, PORTS
from .test_pipeline import MockTransport, Observer, profile, raw_batch
from . import test_subset_stop as native_fixture

BOOT, reply = native_fixture.BOOT, native_fixture.reply


def mixed_batch(group, mid, deadline, *, voltage=40., fault=0):
    """A truthful file-only mock Record*4, not a native three/one exchange."""
    feedback = raw_batch(group, 'mock_source_feedback', group.ids, deadline, fault=fault)
    value = raw_batch(group, 'mock_source_voltage', (mid,), deadline, voltage=voltage)
    records = (Record*4)()
    for index, original in enumerate((*feedback.records, *value.records)):
        records[index] = original
    stats = active.Stats()
    stats.begin_ns = feedback.stats.begin_ns
    stats.end_ns = value.stats.end_ns
    stats.writes = stats.reads = 4; stats.bytes = 68
    rows = decode_records((records, stats))
    return Batch(group, 'acquisition_combined4', records, stats, rows,
                 bytes(records), bytes(stats), stats.end_ns)


def combined_snapshot(feedback, imu, now):
    values = []
    for port in PORTS:
        batch = feedback[port]
        for (mid, kind), row in batch.verify().items():
            if kind == 'feedback':
                values.append((mid, row[0].protocol_position_rad, row[0].velocity_rad_s,
                               row[1], row[2]))
    return {'tick_ns': now, 'motors': values, 'imu': dict(imu),
            'physical_record_count_per_bus': 4, 'projected_feedback_count_per_bus': 3,
            'voltage_pending': False, 'output_allowed': False}


class MockCombinedTransport(MockTransport):
    combined_acquisition = True

    def acquire_combined(self, mid, future, *, deadline_ns, check):
        self._owner(); self.busy = True
        try:
            check()
            delay = self.controls.get('voltage_delay', 0)
            if delay: time.sleep(delay)
            if self.controls.get('fail_voltage') == self.group.port:
                raise ValueError('Injected original combined voltage failure')
            batch = mixed_batch(self.group, mid, deadline_ns,
                                voltage=self.controls.get('voltage_value', 40.))
            self.journal.append((batch.label, (batch.records, batch.stats)))
            self.batches.append(batch)
            self.wires.extend(bytes(record.tx) for record in batch.records)
            check()
            if future.done(): raise ValueError('Foreign completed prefix')
            self.voltage_joined = True
            future.set_result(batch)
            check()
            delay = self.controls.get('owner_post_publish_delay', 0)
            if delay: time.sleep(delay)
            if self.controls.get('foreign_full') == self.group.port:
                return replace(batch)
            return batch
        finally:
            self.busy = False

    def verify_combined_batch(self, batch, mid):
        rows = self.verify_batch(batch, 'acquisition_combined4')
        if (len(batch.records) != 4 or
                tuple(bytes(record.tx) for record in batch.records) !=
                tuple(stop_wire(axis) for axis in self.group.ids)+(read_request(mid, 'voltage'),) or
                set(rows) != {(axis, 'feedback') for axis in self.group.ids} | {(mid, 'voltage')}):
            raise ValueError('Original mock combined image differs')
        return rows


class CombinedPipelineTests(unittest.TestCase):
    def setUp(self):
        # Deliberately use the actual reported physical port permutation.
        self.groups = tuple(Group(port, ids) for port, ids in zip(PORTS,
            ((7, 8, 9), (10, 11, 12), (4, 5, 6), (1, 2, 3))))
        self.topology = {'motor_power_epoch': 'file-only-test',
                         'ids_by_port': {group.port: list(group.ids) for group in self.groups}}
        self.config = pipeline.Config(self.groups, profile(), {mid:0. for mid in range(1, 13)},
                                      self.topology, 1, combined_acquisition=True)
        self.controls, self.adapters = {}, {}
        self.cancel = threading.Event()
        self.topology_patch = patch('experiments.four_bus_diagnostic.topology.validate_topology',
                                    side_effect=lambda document: document)
        self.topology_patch.start(); self.addCleanup(self.topology_patch.stop)

    def factory(self, group):
        adapter = MockCombinedTransport(group, self.controls)
        self.adapters[group.port] = adapter
        return adapter

    @contextmanager
    def scope(self, port, mask):
        yield {'native_tid':threading.get_native_id(), 'cpu_mask':list(mask),
               'timer_slack_ns':1000, 'file_only_mock_readback':True}

    @contextmanager
    def main_scope(self): yield {'file_only_mock_readback':True}

    def imu(self):
        now = time.monotonic_ns()
        return {'read_started_monotonic_ns':now, 'read_finished_monotonic_ns':now,
                'accel_m_s2':(0., 0., 9.81), 'gyro_rad_s':(0., 0., 0.)}

    def check(self):
        if self.cancel.is_set(): raise ValueError('Injected current cancellation')

    def run_case(self, observer=None, factory=None):
        return pipeline.run(self.config, factory=factory or self.factory, imu_read=self.imu,
            observer=observer or Observer(), snapshot_builder=combined_snapshot,
            check_current=self.check, cancel_io=self.cancel.set,
            model_setup=lambda observer, **kw:dict(kw, reset_verified=True, file_only_mock_model=True),
            worker_scope=self.scope, main_scope=self.main_scope,
            backend_usage={'kind':'injected_file_only_mock'},
            release_wait=lambda scheduled:max(scheduled, time.monotonic_ns()), execute=True)

    def test_default_false_and_true_plan_open_nothing_and_are_different_contracts(self):
        false = pipeline.run(replace(self.config, combined_acquisition=False))
        true = pipeline.run(self.config, factory=lambda group:self.fail('PLAN opened a bus'))
        self.assertNotIn('combined_acquisition_selected', false)
        self.assertTrue(true['combined_acquisition_selected'])
        self.assertNotEqual(false['schema'], true['schema'])
        self.assertEqual(true['per_cycle_requests'], 28)
        self.assertFalse(true['voltage_pending_during_inference'])
        self.assertFalse(true['timing_admission_eligible']); self.assertFalse(true['output_allowed'])

    def test_non_bool_selection_rejects_before_factory(self):
        for value in (1, 0, None, 'true'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'exact bool'):
                pipeline.run(replace(self.config, combined_acquisition=value))

    def test_config_and_adapter_selection_must_match_before_native_owner_calls(self):
        result = self.run_case(factory=lambda group:MockTransport(group, self.controls))
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('selection must match', result['primary_error']['message'])
        self.assertFalse(result['setup_voltage'])

    def test_full4_original_join_voltage_and_five_workers_before_model_then_28_requests(self):
        def during(snapshot):
            self.assertTrue(all(adapter.voltage_joined and not adapter.busy
                                for adapter in self.adapters.values()))
            self.assertEqual(len(snapshot['motors']), 12)
            self.assertFalse(snapshot['voltage_pending'])
            self.assertFalse(self.controls.get('output_ports'))
        result = self.run_case(Observer(during))
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])
        record = result['records'][0]
        self.assertEqual(record['actual_request_count'], 28)
        self.assertEqual(record['feedback'], {}); self.assertEqual(record['voltage'], {})
        self.assertLess(record['voltage_join_end_ns'], record['gather_end_ns'])
        self.assertEqual(len({row['native_tid'] for row in result['worker_settings'].values()}), 5)
        for port, batch in record['combined_acquisition'].items():
            self.assertEqual(len(batch.records), 4); self.assertEqual(batch.stats.writes, 4)
            self.assertIs(batch, self.adapters[port].batches[3])
        saved = pipeline.evidence_report(result)
        for port in PORTS:
            raw = saved['records'][0]['combined_acquisition'][port]
            self.assertEqual(raw['original_record_count'], 4)
            self.assertEqual(raw['stats_scope'], 'original_four_request_exchange')
            self.assertEqual(raw['codec'], 'original_python_parser_exact_four_mixed_requests')
            self.assertEqual(raw['projected_feedback_record_indices'], [0, 1, 2])
            self.assertEqual(raw['voltage_record_index'], 3)
            self.assertEqual(saved['records'][0]['output'][port]['codec'],
                             'original_python_parser_exact_three_or_one')
            self.assertTrue(all(value['codec'] == 'original_python_parser_exact_three_or_one'
                                for value in saved['setup_voltage'][port]))
        self.assertFalse(saved['active_controller_qualification'])

    def test_current_bad_voltage_rejects_before_model_and_proxy_all4_cleanup(self):
        self.controls['voltage_value'] = 34.99
        observer = Observer(); result = self.run_case(observer)
        self.assertEqual(result['status'], 'ABORTED'); self.assertEqual(observer.calls, 0)
        self.assertIn('voltage', result['primary_error']['message'])
        self.assertFalse(self.controls.get('output_ports'))
        self.assertEqual(set(result['cleanup']), set(PORTS))

    def test_native_owner_exception_keeps_primary_and_no_model_or_proxy(self):
        self.controls['fail_voltage'] = 'port2'
        observer = Observer(); result = self.run_case(observer)
        self.assertEqual(result['status'], 'ABORTED'); self.assertEqual(observer.calls, 0)
        self.assertIn('original combined voltage failure', result['primary_error']['message'])
        self.assertTrue(all(row['admission_eligible'] is False for row in result['owner_settlement']))

    def test_foreign_full_future_result_cannot_certify_prefix_before_model(self):
        self.controls['foreign_full'] = 'port1'
        observer = Observer(); result = self.run_case(observer)
        self.assertEqual(result['status'], 'ABORTED'); self.assertEqual(observer.calls, 0)
        self.assertIn('combined/full Future binding', result['primary_error']['message'])

    def test_prefix_success_cannot_substitute_pending_original_full_owner_at20ms(self):
        self.controls['owner_post_publish_delay'] = .021
        observer = Observer(); result = self.run_case(observer)
        self.assertEqual(result['status'], 'ABORTED'); self.assertEqual(observer.calls, 0)
        self.assertEqual(result['completed_cycles'], 0)
        self.assertFalse(self.controls.get('output_ports'))
        self.assertEqual(result['primary_error']['type'], 'TimeoutError')
        self.assertTrue(all(row['cleanup_only'] and not row['admission_eligible']
                            for row in result['owner_settlement']))

    def test_final_raw_guard_detects_model_time_tamper_before_output(self):
        def tamper(snapshot):
            self.adapters['port0'].batches[3].records[3].rx[10] ^= 1
        result = self.run_case(Observer(tamper))
        self.assertEqual(result['status'], 'ABORTED'); self.assertFalse(self.controls.get('output_ports'))
        self.assertIn('raw batch', result['primary_error']['message'])

    def test_model_crossing20ms_rejects_even_all4_full_futures_were_successful(self):
        result = self.run_case(Observer(lambda snapshot:time.sleep(.021)))
        self.assertEqual(result['status'], 'ABORTED'); self.assertEqual(result['completed_cycles'], 0)
        self.assertFalse(self.controls.get('output_ports'))
        self.assertTrue(result['failure_retained'])

    def test_current_cancel_after_model_no_proxy_keeps_all4_cleanup_and_restore(self):
        result = self.run_case(Observer(lambda snapshot:self.cancel.set()))
        self.assertEqual(result['status'], 'ABORTED'); self.assertFalse(self.controls.get('output_ports'))
        self.assertEqual(set(result['cleanup']), set(PORTS))
        self.assertTrue(all(result['restoration'][port] is True for port in (*PORTS, 'imu', 'main')))


class BoundaryCurrentCheckTests(CombinedPipelineTests):
    """Selected branch: full guard only at three cycle boundaries."""
    def setUp(self):
        super().setUp()
        self.config = replace(self.config, boundary_current_checks=True)
        self.full_calls, self.light_calls = [], []

    def check(self):
        self.full_calls.append(threading.get_native_id())
        super().check()

    def light(self):
        self.light_calls.append(threading.get_native_id())
        if self.cancel.is_set(): raise ValueError('Injected light cancellation')

    def run_case(self, observer=None, factory=None):
        return pipeline.run(self.config, factory=factory or self.factory, imu_read=self.imu,
            observer=observer or Observer(), snapshot_builder=combined_snapshot,
            check_current=self.check, check_cancelled=self.light, cancel_io=self.cancel.set,
            model_setup=lambda observer, **kw:dict(kw, reset_verified=True, file_only_mock_model=True),
            worker_scope=self.scope, main_scope=self.main_scope,
            backend_usage={'kind':'injected_file_only_mock'},
            release_wait=lambda scheduled:max(scheduled, time.monotonic_ns()), execute=True)

    def test_default_false_and_true_plan_open_nothing_and_are_different_contracts(self):
        # The unselected-combined case is invalid with boundary selection; see below.
        true = pipeline.run(self.config, factory=lambda group:self.fail('PLAN opened a bus'))
        self.assertTrue(true['combined_acquisition_selected']); self.assertFalse(true['output_allowed'])

    def test_plan_records_selection_and_requires_combined(self):
        value = pipeline.run(self.config)
        self.assertTrue(value['boundary_current_checks_selected'])
        self.assertEqual(len(value['full_current_check_points']), 3)
        self.assertEqual(value['per_cycle_requests'], 28)
        self.assertNotIn('boundary_current_checks_selected', pipeline.run(replace(self.config,
            boundary_current_checks=False)))
        separate = pipeline.plan(replace(self.config, combined_acquisition=False))
        self.assertTrue(separate['boundary_current_checks_selected'])
        self.assertNotIn('combined_acquisition_selected', separate)
        for value in (1, None, 'true'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'exact bool'):
                pipeline.plan(replace(self.config, boundary_current_checks=value))

    def test_missing_cancel_check_rejects_before_factory(self):
        with self.assertRaisesRegex(ValueError, 'cancellation check'):
            pipeline.run(self.config, factory=lambda group:self.fail('opened'), imu_read=self.imu,
                observer=Observer(), snapshot_builder=combined_snapshot, check_current=self.check,
                cancel_io=self.cancel.set, model_setup=lambda *a, **k:None, worker_scope=self.scope,
                main_scope=self.main_scope, backend_usage={'kind':'injected_file_only_mock'},
                release_wait=lambda scheduled:scheduled, execute=True)

    def test_cycle_uses_three_full_checks_and_light_hot_path(self):
        marks = {}
        def during(snapshot):
            marks['before_inference'] = len(self.full_calls)
        result = self.run_case(Observer(during))
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result.get('primary_error'))
        self.assertEqual(result['records'][0]['actual_request_count'], 28)
        # Setup (main/factory/prime) checks precede the cycle; count cycle-only.
        setup = marks['before_inference'] - 2
        self.assertEqual(len(self.full_calls) - setup, 3)
        self.assertGreater(len(self.light_calls), 0)
        workers = {row['native_tid'] for port, row in result['worker_settings'].items() if port in PORTS}
        self.assertTrue(workers & set(self.light_calls))
        self.assertFalse(workers & set(self.full_calls[setup:]))


class FinalGateInputIdentityTests(BoundaryCurrentCheckTests):
    """Selected branch: no post-inference snapshot rebuild."""
    def setUp(self):
        super().setUp()
        self.config = replace(self.config, final_gate_input_identity=True)
        self.builds = 0

    def run_case(self, observer=None, factory=None):
        original = globals()['combined_snapshot']
        def counted(*args):
            self.builds += 1
            return original(*args)
        with patch(__name__+'.combined_snapshot', counted):
            return super().run_case(observer, factory)

    def test_plan_records_identity_selection(self):
        value = pipeline.run(self.config)
        self.assertTrue(value['final_gate_input_identity_selected'])
        self.assertTrue(pipeline.plan(replace(self.config, combined_acquisition=False,
            boundary_current_checks=False))['final_gate_input_identity_selected'])
        with self.assertRaisesRegex(ValueError, 'exact bool'):
            pipeline.plan(replace(self.config, final_gate_input_identity=1))

    def test_one_snapshot_build_per_cycle(self):
        result = self.run_case()
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result.get('primary_error'))
        self.assertEqual(self.builds, 1)

    def test_imu_mutation_during_inference_rejects_before_output(self):
        def tamper(snapshot):
            self.imu_value['accel_m_s2'] = (0., 0., 9.8)
        self.imu_value = None
        imu = self.imu
        def shared():
            self.imu_value = imu(); return self.imu_value
        self.imu = shared
        result = self.run_case(Observer(tamper))
        self.assertEqual(result['status'], 'ABORTED'); self.assertFalse(self.controls.get('output_ports'))
        self.assertIn('IMU image changed', result['primary_error']['message'])


class CombinedNativeTests(unittest.TestCase):
    # Reuse the existing real local socket/build fixture without inheriting its
    # test cases or rebuilding a shared library. The environment can select an
    # already qualified genuine subset library on the target.
    setUpClass = classmethod(native_fixture.SubsetStopTests.setUpClass.__func__)
    tearDownClass = classmethod(native_fixture.SubsetStopTests.tearDownClass.__func__)
    setUp = native_fixture.SubsetStopTests.setUp
    tearDown = native_fixture.SubsetStopTests.tearDown
    peer_loop = native_fixture.SubsetStopTests.peer_loop

    def adapter(self, ids=(1, 2, 3), *, selected=True):
        group = Group('port0', ids)
        half = range(group.first_id, group.first_id+6)
        adapter = ThreeAxisTransport.create(self.lib, self.host.fileno(), group=group,
            cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
            raw_lower_by_id={mid:-1. for mid in half}, raw_upper_by_id={mid:1. for mid in half},
            combined_acquisition=selected)
        self.adapters.append(adapter); return adapter

    def acquire(self, adapter, mid=None, future=None, check=lambda:None):
        future = future or Future(); future.set_running_or_notify_cancel()
        batch = adapter.acquire_combined(adapter.group.ids[0] if mid is None else mid, future,
                    deadline_ns=time.monotonic_ns()+20_000_000, check=check)
        return batch, future

    def test_original_native_accepts4mixed_sameid_keys_original_bytes_stats_and_times(self):
        adapter = self.adapter((10, 11, 12)); self.peer_loop(4, fragmented=True)
        batch, future = self.acquire(adapter, 11)
        self.assertIs(future.result(), batch); self.assertEqual(len(batch.records), 4)
        self.assertEqual([value.kind for value in self.seen], [4, 4, 4, 17])
        self.assertEqual([value.destination for value in self.seen], [10, 11, 12, 11])
        self.assertEqual(batch.stats.writes, 4); self.assertEqual(batch.stats.bytes, 68)
        self.assertEqual(batch.record_image, bytes(batch.records))
        self.assertEqual(batch.stats_image, bytes(batch.stats))
        self.assertEqual(batch.rows, decode_records((batch.records, batch.stats)))
        self.assertEqual(adapter.verify_combined_batch(batch, 11), batch.rows)
        self.assertTrue(all(row.deadline_ns == batch.records[0].deadline_ns for row in batch.records))
        self.assertTrue(all(row.received == row.written == 17 for row in batch.records))
        self.assertTrue(all(b.start_ns-a.start_ns >= 900_000
                            for a, b in zip(batch.records, batch.records[1:])))

    def test_original_native_duplicate_type2_key_still_rejects_but_mixed_keys_allowed(self):
        adapter = self.adapter()
        with self.assertRaisesRegex(ValueError, 'exact three'):
            adapter.validate_wires((stop_wire(1), stop_wire(1), stop_wire(3), read_request(1, 'voltage')))
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_default_unselected_and_wrongselection_type_do_not_admit_mixed_wire(self):
        adapter = self.adapter(selected=False)
        with self.assertRaisesRegex(ValueError, 'exact three'):
            adapter.validate_wires(adapter._stop_wires+(read_request(1, 'voltage'),))
        with self.assertRaisesRegex(ValueError, 'Explicit combined'):
            adapter.acquire_combined(1, Future(), deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        with self.assertRaisesRegex(ValueError, 'exact bool'):
            self.adapter(selected=1)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_exact_whitelist_rejects_foreign_axis_order_flags_data_and_type1_before_write(self):
        adapter = self.adapter((4, 5, 6))
        good = adapter._stop_wires+(read_request(4, 'voltage'),)
        self.assertEqual(adapter.validate_wires(good), good)
        variants = [good[:3]+(read_request(1, 'voltage'),),
                    good[:3]+(read_request(4, 'position'),),
                    (good[1], good[0], good[2], good[3]),
                    (active.encode_motion(4, 0., 0., 0.),)+good[1:],
                    (good[0][:6]+b'\x00'+good[0][7:],)+good[1:],
                    good[:3]+(good[3][:11]+b'\x01'+good[3][12:],)]
        for wires in variants:
            with self.subTest(wires=wires), self.assertRaises(ValueError): adapter.validate_wires(wires)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_allfour_join_needed_when_voltage_delayed_no_early_prefix(self):
        adapter = self.adapter(); future = Future(); future.set_running_or_notify_cancel()
        saw_voltage = threading.Event()
        def response(wire):
            if ATParser().feed(wire)[0].kind == 17:
                saw_voltage.set(); self.assertFalse(future.done()); time.sleep(.002)
            return reply(wire)
        self.peer_loop(4, mutate=response)
        batch = adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertTrue(saw_voltage.is_set()); self.assertIs(future.result(), batch)
        self.assertGreaterEqual(batch.completed_ns, max(row.received_ns for row in batch.records))

    def test_voltage_can_reply_before_last_STOP_without_reordering_original_slots(self):
        adapter = self.adapter((7, 8, 9)); delayed = []
        def response(wire):
            request = ATParser().feed(wire)[0]
            if request.kind == 4 and request.destination == 9:
                delayed.append(reply(wire)); return None
            if request.kind == 17:
                return reply(wire)+delayed.pop()
            return reply(wire)
        self.peer_loop(4, mutate=response)
        batch, future = self.acquire(adapter, 8)
        self.assertIs(future.result(), batch)
        self.assertLessEqual(batch.records[3].received_ns, batch.records[2].received_ns)
        self.assertLess(batch.records[3].start_ns, batch.records[2].received_ns)
        self.assertEqual(tuple(bytes(row.tx) for row in batch.records),
                         adapter._stop_wires+(read_request(8, 'voltage'),))
        self.assertEqual(adapter.verify_combined_batch(batch, 8), batch.rows)

    def test_missing_voltage_keeps_original4_partial_and_no_prefix_success(self):
        adapter = self.adapter(); future = Future(); future.set_running_or_notify_cancel()
        self.peer_loop(4, mutate=lambda wire:None if ATParser().feed(wire)[0].kind == 17 else reply(wire))
        with self.assertRaises(active.ExchangeError) as caught:
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertIs(future.exception(), caught.exception)
        self.assertEqual(len(caught.exception.records), 4)
        self.assertEqual([row.received for row in caught.exception.records], [17, 17, 17, 0])
        self.assertEqual(len(adapter.journal), 1); self.assertEqual(adapter.journal[0][0], 'acquisition_combined4')
        self.assertIs(adapter.journal[0][1][0], caught.exception.records)
        self.assertTrue(adapter._session.poisoned)

    def test_mode2_feedback_rejected_by_native_exactSTOP_match_with_raw_and_no_prefix(self):
        adapter = self.adapter(); future = Future(); future.set_running_or_notify_cancel()
        self.peer_loop(4, mutate=lambda wire:reply(wire, mode=2) if ATParser().feed(wire)[0].kind == 4 else reply(wire))
        with self.assertRaises(active.ExchangeError) as caught:
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertIs(future.exception(), caught.exception)
        self.assertEqual(len(caught.exception.records), 4)
        self.assertGreater(caught.exception.stats.rejected_total, 0)

    def test_faulty_feedback_keeps_full4raw_and_prefix_exception(self):
        adapter = self.adapter(); future = Future(); future.set_running_or_notify_cancel()
        self.peer_loop(4, mutate=lambda wire:reply(wire, fault=4) if ATParser().feed(wire)[0].destination == 2 else reply(wire))
        with self.assertRaisesRegex(active.ExchangeError, 'STOP reported a fault') as caught:
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertIs(future.exception(), caught.exception)
        self.assertEqual(len(adapter.journal), 1)
        self.assertIs(adapter.journal[0][1][0], caught.exception.records)
        self.assertEqual(len(caught.exception.records), 4)
        self.assertEqual(caught.exception.records[1].received, 17)
        self.assertEqual(int.from_bytes(bytes(caught.exception.records[1].rx)[2:6], 'big') >> 19 & 63, 4)
        self.assertTrue(adapter._session.poisoned)

    def test_foreign_or_precompleted_future_cannot_pass_owner_publication(self):
        adapter = self.adapter(); future = Future(); future.set_result('foreign')
        with self.assertRaisesRegex(ValueError, 'unpublished'):
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_cancel_before_native_no_requests_original_failure_and_prefix_exception(self):
        adapter = self.adapter(); future = Future(); future.set_running_or_notify_cancel()
        os.write(self.cancel_write, b'x')
        with self.assertRaises(active.ExchangeError) as caught:
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertIs(future.exception(), caught.exception)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])
        self.assertTrue(all(row.written == 0 for row in caught.exception.records))

    def test_native_cancel_during4keeps_partial_original_and_never_success(self):
        adapter = self.adapter(); future = Future(); future.set_running_or_notify_cancel()
        def response(wire):
            request = ATParser().feed(wire)[0]
            if request.destination == 2 and request.kind == 4:
                os.write(self.cancel_write, b'x'); return None
            return reply(wire)
        self.peer_loop(4, mutate=response)
        with self.assertRaisesRegex(active.ExchangeError, 'Cancelled') as caught:
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertIs(future.exception(), caught.exception)
        self.assertEqual(len(caught.exception.records), 4)
        self.assertLess(caught.exception.stats.writes, 4)

    def test_external_completion_during_native_cannot_replace_actual_full4_identity(self):
        adapter = self.adapter(); future = Future(); future.set_running_or_notify_cancel()
        foreign = object()
        def response(wire):
            if ATParser().feed(wire)[0].kind == 17:
                future.set_result(foreign)
            return reply(wire)
        self.peer_loop(4, mutate=response)
        with self.assertRaisesRegex(ValueError, 'externally completed'):
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertIs(future.result(), foreign)
        self.assertEqual(len(adapter.journal), 1)
        self.assertEqual(len(adapter.journal[0][1][0]), 4)
        self.assertTrue(all(row.received == 17 for row in adapter.journal[0][1][0]))

    def test_rebound_original_fd_fails_before_writing_replacement(self):
        adapter = self.adapter(); old = self.host.detach(); os.close(old)
        reader, writer = os.pipe(); self.extra_fds.extend((reader, writer))
        if reader != old:
            os.dup2(reader, old); self.extra_fds.append(old)
        future = Future(); future.set_running_or_notify_cancel()
        with self.assertRaises(active.ExchangeError) as caught:
            adapter.acquire_combined(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertIs(future.exception(), caught.exception)
        self.assertTrue(all(row.written == 0 for row in caught.exception.records))

    def test_selection_source_and_whitelist_mutation_reject_before_write(self):
        adapter = self.adapter(); adapter.combined_acquisition = False
        with self.assertRaisesRegex(ValueError, 'binding changed'):
            adapter.acquire_combined(1, Future(), deadline_ns=time.monotonic_ns()+20_000_000, check=lambda:None)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_allfour_raw_and_stats_tamper_or_foreign_clone_is_rejected(self):
        adapter = self.adapter(); self.peer_loop(4)
        batch, future = self.acquire(adapter)
        with self.assertRaisesRegex(ValueError, 'current owner batch'):
            adapter.verify_combined_batch(replace(batch), 1)
        original = batch.stats.writes; batch.stats.writes = 3
        try:
            with self.assertRaisesRegex(ValueError, 'raw batch'): adapter.verify_combined_batch(batch, 1)
        finally: batch.stats.writes = original
        batch.records[3].rx[11] ^= 1
        with self.assertRaisesRegex(ValueError, 'raw batch'): adapter.verify_combined_batch(batch, 1)

    def test_four_genuine_buses_combined28_full_raw_and_original_subsetSTOP_cleanup(self):
        groups = tuple(Group(port, ids) for port, ids in zip(PORTS,
            ((7, 8, 9), (10, 11, 12), (4, 5, 6), (1, 2, 3))))
        hosts, peers, threads, made = {}, {}, [], {}
        seen, errors = {port:[] for port in PORTS}, []
        for port in PORTS:
            host, peer = socket.socketpair(); host.setblocking(False)
            hosts[port] = host; peers[port] = peer
            ready = threading.Event()
            def serve(port=port, peer=peer, ready=ready):
                parser = ATParser(); ready.set()
                try:
                    while len(seen[port]) < 13:
                        if not select.select([peer], [], [], 1)[0]: return
                        raw = peer.recv(4096)
                        if not raw: return
                        for request in parser.feed(raw):
                            seen[port].append(request)
                            peer.sendall(reply(request.wire))
                except OSError: pass
                except BaseException as error: errors.append(error)
            thread = threading.Thread(target=serve, name='combined4-local-peer-'+port)
            thread.start(); threads.append(thread); self.assertTrue(ready.wait(timeout=1))
        def factory(group):
            half = range(group.first_id, group.first_id+6)
            adapter = ThreeAxisTransport.create(self.lib, hosts[group.port].fileno(), group=group,
                cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
                raw_lower_by_id={mid:-1. for mid in half}, raw_upper_by_id={mid:1. for mid in half},
                combined_acquisition=True)
            made[group.port] = adapter; return adapter
        @contextmanager
        def scope(port, mask):
            yield {'native_tid':threading.get_native_id(), 'cpu_mask':list(mask),
                   'timer_slack_ns':1000, 'file_only_mock_readback':True}
        @contextmanager
        def main_scope(): yield {'file_only_mock_readback':True}
        def imu():
            now = time.monotonic_ns()
            return {'read_started_monotonic_ns':now, 'read_finished_monotonic_ns':now,
                    'accel_m_s2':(0.,0.,9.81), 'gyro_rad_s':(0.,0.,0.)}
        topology = {'motor_power_epoch':'file-only-test',
                    'ids_by_port':{group.port:list(group.ids) for group in groups}}
        config = pipeline.Config(groups, profile(), {mid:0. for mid in range(1,13)},
                                 topology, 1, combined_acquisition=True)
        try:
            with patch('experiments.four_bus_diagnostic.topology.validate_topology', side_effect=lambda x:x):
                result = pipeline.run(config, factory=factory, imu_read=imu, observer=Observer(),
                    snapshot_builder=combined_snapshot, check_current=lambda:None,
                    cancel_io=lambda:os.write(self.cancel_write, b'x'),
                    model_setup=lambda o, **kw:dict(kw, reset_verified=True, file_only_mock_model=True),
                    worker_scope=scope, main_scope=main_scope,
                    backend_usage={'kind':'injected_file_only_mock', 'transport':'genuine_cpp_local_sockets'},
                    release_wait=lambda ns:max(ns,time.monotonic_ns()), execute=True)
            self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result['primary_error'])
            row = result['records'][0]
            self.assertEqual(row['actual_request_count'], 28)
            self.assertLess(row['voltage_join_end_ns'], row['gather_end_ns'])
            for group in groups:
                batch = row['combined_acquisition'][group.port]
                self.assertEqual(len(batch.records), 4); self.assertEqual(batch.stats.writes, 4)
                self.assertEqual([request.kind for request in seen[group.port]],
                                 [17,17,17,4,4,4,17,4,4,4,4,4,4])
                self.assertEqual({request.destination for request in seen[group.port]}, set(group.ids))
                self.assertTrue(result['cleanup'][group.port]['complete'])
                self.assertIs(made[group.port]._batches[3], batch)
                self.assertEqual(tuple(bytes(record.tx) for record in batch.records),
                                 tuple(stop_wire(mid) for mid in group.ids)+(read_request(group.ids[0], 'voltage'),))
            self.assertEqual(len({value['native_tid'] for value in result['worker_settings'].values()}), 5)
            self.assertFalse(result['learned_targets_sent']); self.assertFalse(result['positive_gain_sent'])
        finally:
            for adapter in made.values(): adapter.close()
            for sock in (*hosts.values(), *peers.values()): sock.close()
            for thread in threads: thread.join(timeout=1)
        if errors: raise errors[0]


if __name__ == '__main__': unittest.main()
