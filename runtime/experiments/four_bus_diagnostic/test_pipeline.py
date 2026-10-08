"""Factory-only causal tests. No library, serial descriptor, CAN or real model."""
from concurrent.futures import Future
from contextlib import contextmanager
from dataclasses import replace
import ctypes as C
import math
import struct
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw import native_active_transport as active
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from singularitydog_hw.policy_output_runtime import decode_records
from . import pipeline
from .transport_adapter import Batch, Group, GROUPS, PORTS, ThreeAxisTransport


def frame(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def raw_batch(group, label, mids, deadline, *, voltage=None, fault=0):
    records = (Record*len(mids))()
    stats = active.Stats()
    stats.begin_ns = time.monotonic_ns()
    for record, mid in zip(records, mids):
        if voltage is None:
            tx = stop_wire(mid)
            rx = frame((2 << 24)|(fault << 16)|(mid << 8)|0xfd,
                       struct.pack('>4H', 32767, 32767, 32767, 250))
        else:
            tx = read_request(mid, 'voltage')
            rx = frame((17 << 24)|(mid << 8)|0xfd, tx[7:11]+struct.pack('<f', voltage))
        now = time.monotonic_ns()
        record.tx[:] = tx; record.rx[:] = rx
        record.start_ns = now; record.finish_ns = now+1
        record.read_start_ns = now+2; record.received_ns = now+3
        record.deadline_ns = deadline; record.written = record.received = 17
    stats.end_ns = time.monotonic_ns()+4
    stats.writes = len(mids); stats.reads = len(mids); stats.bytes = 17*len(mids)
    rows = decode_records((records, stats))
    return Batch(group, label, records, stats, rows, bytes(records), bytes(stats), stats.end_ns)


class MockTransport:
    def __init__(self, group, controls):
        self.group, self.controls = group, controls
        self.journal, self.batches, self.wires = [], [], []
        self.owner = None; self.closed = False; self.busy = False; self.voltage_joined = False
        self.recover_calls = 0

    def _owner(self):
        tid = threading.get_native_id()
        if self.owner is None: self.owner = tid
        if self.owner != tid: raise RuntimeError('Mock physical owner changed')

    def _batch(self, label, mids, deadline, voltage=None):
        self._owner()
        batch = raw_batch(self.group, label, mids, deadline, voltage=voltage)
        self.journal.append((label, (batch.records, batch.stats)))
        self.batches.append(batch)
        self.wires.extend(bytes(record.tx) for record in batch.records)
        return batch

    def read_voltage(self, mid, *, deadline_ns, check):
        check(); return self._batch('setup_voltage', (mid,), deadline_ns, 40.)

    def acquire(self, mid, future, *, deadline_ns, check):
        self._owner(); self.busy = True
        try:
            check()
            feedback = self._batch('feedback', self.group.ids, deadline_ns)
            future.set_result(feedback)
            if self.controls.get('prefix_event'): self.controls['prefix_event'].set()
            delay = self.controls.get('voltage_delay', 0)
            if delay: time.sleep(delay)
            check()
            if self.controls.get('fail_voltage') == self.group.port:
                error = RuntimeError('Injected current voltage owner failure')
                raise error
            voltage = self._batch('voltage', (mid,), deadline_ns,
                                  self.controls.get('voltage_value', 40.))
            self.voltage_joined = True
            return feedback, voltage
        finally:
            self.busy = False

    def verify_batch(self, batch, label):
        if type(batch) is not Batch or not any(batch is value for value in self.batches) or batch.label != label:
            raise ValueError('Foreign mock batch')
        rows = batch.verify()
        if any(value[0].mode_state != 0 or value[0].fault_bits != 0
               for key, value in rows.items() if key[1] == 'feedback'):
            raise ValueError('Mock STOP mode/fault')
        return rows

    def output_stop(self, *, deadline_ns, check):
        check(); self._owner()
        if not self.voltage_joined: raise RuntimeError('Output before full owner join')
        self.controls.setdefault('output_ports', []).append(self.group.port)
        return self._batch('output_stop', self.group.ids, deadline_ns)

    def recover_subset(self):
        self._owner()
        if self.busy: raise RuntimeError('Recovery raced existing owner')
        self.recover_calls += 1
        self.controls.setdefault('recovery_order', []).append(self.group.port)
        return {'complete': self.controls.get('failed_stop') != self.group.port,
                'selected_ids': list(self.group.ids), 'cleanup_only': True,
                'active_deadline_extended': False}

    def close(self):
        if self.busy: raise RuntimeError('Close raced existing owner')
        self.closed = True
        if self.controls.get('failed_close') == self.group.port:
            raise RuntimeError('Injected session close failure')


def profile():
    return {'max_sample_age_ms': 20,
            'axes': {str(mid): {'sign': 1, 'lower_rad': -1., 'upper_rad': 1.,
                    'max_measured_velocity_rad_s': 5., 'max_measured_torque_nm': 1.,
                    'max_temperature_c': 50., 'max_displacement_from_start_rad': .1}
                     for mid in range(1, 13)}}


def snapshot_builder(feedback, imu, now):
    values = []
    for port in PORTS:
        batch = feedback[port]
        values.extend((mid, rows[0].protocol_position_rad, rows[0].velocity_rad_s,
                       rows[1], rows[2]) for (mid, _), rows in batch.verify().items())
    return {'tick_ns': now, 'motors': values, 'imu': dict(imu), 'output_allowed': False}


class Observer:
    def __init__(self, consume=None): self.calls = 0; self.callback = consume
    def consume(self, snapshot):
        self.calls += 1
        if self.callback: self.callback(snapshot)
        return {'file_only_mock_model': True, 'call': self.calls, 'output_allowed': False}


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.groups = tuple(Group(port, group) for port, group in zip(PORTS, GROUPS))
        self.topology = {'motor_power_epoch': 'file-only-test',
                         'ids_by_port': {g.port: list(g.ids) for g in self.groups}}
        self.config = pipeline.Config(self.groups, profile(), {mid: 0. for mid in range(1, 13)}, self.topology, 1)
        self.topology_patch = patch('experiments.four_bus_diagnostic.topology.validate_topology',
                                    side_effect=lambda value: value)
        self.topology_patch.start(); self.addCleanup(self.topology_patch.stop)
        self.controls = {}; self.adapters = {}; self.canceled = threading.Event(); self.events = []

    def factory(self, group):
        value = MockTransport(group, self.controls); self.adapters[group.port] = value; return value

    @contextmanager
    def worker_scope(self, port, mask):
        self.events.append(('worker-enter', port, threading.get_native_id()))
        yield {'native_tid': threading.get_native_id(), 'cpu_mask': list(mask),
               'timer_slack_ns': 1000, 'file_only_mock_readback': True}
        self.events.append(('worker-exit', port, threading.get_native_id()))
        if self.controls.get('failed_restore') == port:
            raise RuntimeError('Injected worker restore failure')

    @contextmanager
    def main_scope(self):
        self.events.append(('main-enter',))
        yield {'file_only_mock_readback': True}
        self.events.append(('main-exit',))

    def check(self):
        if self.canceled.is_set(): raise RuntimeError('Injected cancellation')

    def imu(self):
        now = time.monotonic_ns()
        return {'read_started_monotonic_ns': now, 'read_finished_monotonic_ns': now,
                'accel_m_s2': (0., 0., 9.81), 'gyro_rad_s': (0., 0., 0.)}

    def run_case(self, observer=None):
        return pipeline.run(self.config, factory=self.factory, imu_read=self.imu,
            observer=observer or Observer(), snapshot_builder=snapshot_builder,
            check_current=self.check, cancel_io=self.canceled.set,
            model_setup=lambda o, **kw: dict(kw, reset_verified=True, file_only_mock_model=True),
            worker_scope=self.worker_scope, main_scope=self.main_scope,
            backend_usage={'kind': 'injected_file_only_mock'},
            release_wait=lambda when: max(when, time.monotonic_ns()), execute=True)

    def test_default_plan_does_not_call_factories_or_load_models(self):
        result = pipeline.run(self.config, factory=lambda _: self.fail('PLAN opened factory'))
        self.assertEqual(result['status'], 'PLAN'); self.assertFalse(result['opens_devices'])
        self.assertEqual(result['per_cycle_requests'], 28)

    def test_one_cycle_has_four_physical_3_1_3_raw_and_separate_terminal_stop(self):
        result = self.run_case()
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result.get('primary_error'))
        self.assertEqual(result['completed_cycles'], 1)
        record = result['records'][0]
        self.assertEqual(record['actual_request_count'], 28)
        self.assertEqual(len({e[2] for e in self.events if e[0] == 'worker-enter'}), 5)
        for port, adapter in self.adapters.items():
            self.assertTrue(adapter.closed); self.assertEqual(adapter.recover_calls, 1)
            self.assertEqual([len(record[phase][port].records) for phase in ('feedback', 'voltage', 'output')], [3, 1, 3])
            for wire in adapter.wires:
                request = ATParser().feed(wire)[0]
                self.assertIn(request.destination, adapter.group.ids)
                self.assertIn(request.kind, (4, 17))
        evidence = pipeline.evidence_report(result)
        self.assertEqual(evidence['records'][0]['feedback']['port0']['original_record_count'], 3)
        self.assertEqual(self.events[-1], ('main-exit',))

    def test_feedback_prefix_allows_inference_but_voltage_full_join_precedes_output(self):
        self.controls['voltage_delay'] = .002
        def during(_):
            self.assertTrue(any(adapter.busy for adapter in self.adapters.values()))
            self.assertFalse(self.controls.get('output_ports'))
        result = self.run_case(Observer(during))
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result.get('primary_error'))
        self.assertEqual(set(self.controls['output_ports']), set(PORTS))

    def test_voltage_exception_no_output_all_four_cleanup_original_failure_kept(self):
        self.controls['fail_voltage'] = 'port1'
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('current voltage owner failure', result['primary_error']['message'])
        self.assertFalse(self.controls.get('output_ports'))
        self.assertEqual(set(result['cleanup']), set(PORTS))
        self.assertTrue(all(row['admission_eligible'] is False for row in result['records'][0]['owner_settlement'].values()))

    def test_low_voltage_no_output_and_no_threshold_expansion(self):
        self.controls['voltage_value'] = 34.99
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED'); self.assertFalse(self.controls.get('output_ports'))
        self.assertIn('voltage', result['primary_error']['message'])

    def test_inference_crossing_original_deadline_cannot_use_late_done_futures(self):
        result = self.run_case(Observer(lambda _: time.sleep(.021)))
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(result['completed_cycles'], 0)
        self.assertFalse(self.controls.get('output_ports'))
        self.assertTrue(result['failure_retained'])
        self.assertLess(result['records'][0]['original_deadline_ns'], time.monotonic_ns())

    def test_snapshot_tamper_rejected_before_output(self):
        result = self.run_case(Observer(lambda snapshot: snapshot['motors'].clear()))
        self.assertEqual(result['status'], 'ABORTED'); self.assertFalse(self.controls.get('output_ports'))
        self.assertIn('mutated', result['primary_error']['message'])

    def test_current_cancel_no_output_all_four_stop_and_restore(self):
        result = self.run_case(Observer(lambda _: self.canceled.set()))
        self.assertEqual(result['status'], 'ABORTED'); self.assertFalse(self.controls.get('output_ports'))
        self.assertEqual(set(self.controls['recovery_order']), set(PORTS))
        self.assertTrue(all(result['restoration'][port] is True for port in (*PORTS, 'imu', 'main')))

    def test_unconfirmed_subset_stop_keeps_physical_cutoff_required(self):
        self.controls['failed_stop'] = 'port2'
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED'); self.assertTrue(result['physical_cutoff_required'])
        self.assertFalse(result['output_allowed'])

    def test_whitelist_requires_full_canonical_wires_and_exact_physical_ids(self):
        adapter = object.__new__(ThreeAxisTransport)
        adapter._stop_wires = tuple(stop_wire(mid) for mid in (1, 2, 3))
        adapter._voltage_wires = tuple(read_request(mid, 'voltage') for mid in (1, 2, 3))
        self.assertEqual(adapter.validate_wires(adapter._stop_wires), adapter._stop_wires)
        rejected = [tuple(stop_wire(mid) for mid in (4, 5, 6)),
                    (read_request(4, 'voltage'),), (read_request(1),),
                    (active.encode_motion(1, 0., 0., 0.),),
                    (bytes([adapter._voltage_wires[0][0]^1])+adapter._voltage_wires[0][1:],)]
        for wires in rejected:
            with self.subTest(wires=wires), self.assertRaisesRegex(ValueError, 'exact three'):
                adapter.validate_wires(wires)

    def test_raw_image_or_decoded_value_tamper_is_rejected(self):
        group = self.groups[0]
        batch = raw_batch(group, 'feedback', group.ids, time.monotonic_ns()+20_000_000)
        batch.records[0].rx[10] ^= 1
        with self.assertRaisesRegex(ValueError, 'raw batch'): batch.verify()

    def test_half_crossing_and_duplicate_groups_are_rejected_before_factory(self):
        with self.assertRaises(ValueError): Group('port0', (5, 6, 7))
        bad = pipeline.Config((self.groups[0],)*4, profile(), self.config.offsets, self.topology)
        with self.assertRaises(ValueError): pipeline.run(bad)

    def test_actual_port_permutation_keeps_all_twelve_logical_axes_and_rotating_ids(self):
        groups = tuple(Group(port, ids) for port, ids in zip(PORTS,
                       ((7, 8, 9), (10, 11, 12), (4, 5, 6), (1, 2, 3))))
        topology = dict(self.topology, ids_by_port={group.port: list(group.ids) for group in groups})
        self.config = pipeline.Config(groups, profile(), self.config.offsets, topology, 1)
        result = self.run_case()
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result.get('primary_error'))
        self.assertEqual({mid for batch in result['records'][0]['feedback'].values()
                          for mid, kind in batch.rows}, set(range(1, 13)))
        self.assertEqual([next(iter(result['records'][0]['voltage'][port].rows))[0]
                          for port in PORTS], [7, 10, 4, 1])

    def test_ready_owner_error_has_priority_over_pending_peer_deadline(self):
        ready, pending = Future(), Future()
        error = ValueError('genuine ready owner error'); ready.set_exception(error)
        def check():
            if ready.done(): ready.result()
        with self.assertRaises(ValueError) as caught:
            pipeline._take(pending, time.monotonic_ns()-1, time.monotonic_ns, check)
        self.assertIs(caught.exception, error)

    def test_worker_restore_failure_aborts_otherwise_successful_cycle(self):
        self.controls['failed_restore'] = 'port2'
        result = self.run_case()
        self.assertEqual(result['completed_cycles'], 1)
        self.assertEqual(result['status'], 'ABORTED'); self.assertTrue(result['failure_retained'])
        self.assertIn('worker restore failure', result['primary_error']['message'])
        self.assertTrue(result['restoration']['port0']); self.assertTrue(result['restoration']['imu'])

    def test_session_close_failure_aborts_otherwise_successful_cycle(self):
        self.controls['failed_close'] = 'port3'
        result = self.run_case()
        self.assertEqual(result['completed_cycles'], 1)
        self.assertEqual(result['status'], 'ABORTED'); self.assertTrue(result['failure_retained'])
        self.assertIn('session close failure', result['primary_error']['message'])
        self.assertTrue(result['restoration']['main'])

    def test_cleanup_failure_cannot_replace_original_voltage_failure(self):
        self.controls.update(fail_voltage='port1', failed_restore='port2')
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('current voltage owner failure', result['primary_error']['message'])

    def test_real_backend_rejects_incomplete_os_readback_before_factory(self):
        with self.main_scope() as value:
            with self.assertRaisesRegex(ValueError, 'before devices'):
                pipeline._real_operating_readback(value)

    def test_failed_initializer_settles_delayed_imu_and_restores_its_same_thread(self):
        original = self.worker_scope
        @contextmanager
        def selected(port, mask):
            if port == 'port0':
                raise RuntimeError('Injected early scope setup failure')
            if port == 'imu': time.sleep(.03)
            with original(port, mask) as value: yield value
        self.worker_scope = selected
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('early scope setup failure', result['primary_error']['message'])
        self.assertTrue(result['restoration']['imu'])
        starts = [value[2] for value in self.events if value[:2] == ('worker-enter', 'imu')]
        ends = [value[2] for value in self.events if value[:2] == ('worker-exit', 'imu')]
        self.assertEqual(starts, ends)
        self.assertEqual(len(result['owner_settlement']), 5)

    def test_pending_abort_owner_is_not_misreported_as_post_settlement_success(self):
        self.controls['voltage_delay'] = .57
        result = self.run_case()
        self.assertEqual(result['status'], 'ABORTED')
        pending = result['owner_settlement'][0]
        self.assertEqual(pending['state'], 'PENDING')
        self.assertFalse(pending['done_observed']); self.assertNotIn('post_settlement_ns', pending)
        self.assertLess(pending['observation_monotonic_ns'], result['all_original_workers_joined_monotonic_ns'])
        self.assertFalse(result['records'][0]['completed'])
        self.assertTrue(all(adapter.closed and not adapter.busy for adapter in self.adapters.values()))

    def test_worker_scope_rollback_preserves_setup_error_and_restores_cpu_even_if_slack_cleanup_fails(self):
        import sys
        from singularitydog_hw import thread_timer_slack
        original_cpu, requested = {0, 1, 2, 3}, {0}
        current = [original_cpu]
        writes = []
        def set_cpu(pid, mask):
            writes.append(set(mask)); current[0] = set(mask)
        primary = RuntimeError('Injected timer slack setup failure')
        class Slack:
            def __init__(self, requested_ns): pass
            def __enter__(self): raise primary
            def __exit__(self, *args): raise RuntimeError('Injected timer slack cleanup failure')
        with patch.object(sys, 'platform', 'linux'), \
                patch.object(pipeline.os, 'sched_getaffinity', side_effect=lambda pid:set(current[0]), create=True), \
                patch.object(pipeline.os, 'sched_setaffinity', side_effect=set_cpu, create=True), \
                patch.object(thread_timer_slack, 'TimerSlack', Slack):
            with self.assertRaises(RuntimeError) as caught:
                pipeline.LinuxWorkerScope('port0', (0,)).__enter__()
        self.assertIs(caught.exception, primary)
        self.assertEqual(writes, [requested, original_cpu]); self.assertEqual(current[0], original_cpu)
        self.assertTrue(any('slack cleanup failure' in note for note in primary.__notes__))


if __name__ == '__main__':
    unittest.main()


class SeparateBoundaryIdentityTests(PipelineTests):
    """Every separate-branch case also passes with both explicit selections."""
    def setUp(self):
        super().setUp()
        self.config = replace(self.config, boundary_current_checks=True, final_gate_input_identity=True)
        self.cycle_full = None

    def check(self):
        if self.cycle_full is not None:
            self.cycle_full.append(threading.get_native_id())
        super().check()

    def light(self):
        if self.canceled.is_set(): raise RuntimeError('Injected light cancellation')

    def release(self, when):
        self.cycle_full = []
        return max(when, time.monotonic_ns())

    def run_case(self, observer=None):
        return pipeline.run(self.config, factory=self.factory, imu_read=self.imu,
            observer=observer or Observer(), snapshot_builder=snapshot_builder,
            check_current=self.check, check_cancelled=self.light, cancel_io=self.canceled.set,
            model_setup=lambda o, **kw: dict(kw, reset_verified=True, file_only_mock_model=True),
            worker_scope=self.worker_scope, main_scope=self.main_scope,
            backend_usage={'kind': 'injected_file_only_mock'},
            release_wait=self.release, execute=True)

    def test_plan_records_both_selections_on_separate_branch(self):
        value = pipeline.run(self.config)
        self.assertTrue(value['boundary_current_checks_selected'])
        self.assertTrue(value['final_gate_input_identity_selected'])
        self.assertNotIn('combined_acquisition_selected', value)

    def test_cycle_has_exactly_three_main_full_checks(self):
        result = self.run_case()
        self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result.get('primary_error'))
        self.assertEqual(result['records'][0]['actual_request_count'], 28)
        workers = {row['native_tid'] for port, row in result['worker_settings'].items() if port in PORTS}
        self.assertEqual(len(self.cycle_full), 3)
        self.assertFalse(workers & set(self.cycle_full))
