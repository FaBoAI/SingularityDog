"""No devices or model weights: real observer contracts and deterministic clocks."""
import contextlib
import copy
import io
import json
from pathlib import Path
import queue
import signal
import tempfile
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_observer_live as live
from test_policy_observer import Policy, FakeTorch, calibration, mount, snapshot


class Clock:
    def __init__(self, now=1_000_000_000):
        self.now = now
    def __call__(self):
        return self.now
    def wait(self, seconds):
        self.now += max(1, round(seconds*1e9))


def plan(**changes):
    values = dict(max_ticks=3, max_seconds=10., max_age_ns=100_000_000,
                  max_spread_ns=100_000_000, max_lateness_ns=5_000_000)
    values.update(changes)
    return live.plan_for(**values)


def seed(bus, tick=1_000_000_000, *, identity=True, omit=None, imu_sequence=1):
    if identity:
        bus.publish({'kind': 'identity_all12_verified', 'ids': list(range(1, 13))}, telemetry_input=True)
    data = snapshot(tick)
    for row in data['motors']:
        if (row['motor_id'], row['parameter']) == omit:
            continue
        bus.publish({'kind': 'motor_parameter', 'motor_id': row['motor_id'],
            'parameter': row['parameter'], 'ok': True, 'status': 0, 'value': row['value'],
            'unit': row['unit'], 'request_monotonic_ns': row['request_ns'],
            'monotonic_ns': row['received_ns']}, telemetry_input=True)
    im = data['imu']
    bus.publish({'kind': 'imu', 'frame': 'sensor', 'sequence': imu_sequence,
        'accel_m_s2': im['accel_m_s2'], 'gyro_rad_s': im['gyro_rad_s'],
        'read_started_monotonic_ns': im['read_started_ns'],
        'read_finished_monotonic_ns': im['read_finished_ns']}, telemetry_input=True)


def logs(bus):
    result = []
    while not bus.logs.empty(): result.append(bus.logs.get_nowait())
    return result


class SchedulerTests(unittest.TestCase):
    def test_enabled_profile_reaches_live_tick_results_and_final_summary(self):
        clock = Clock()
        bus = live.SessionBus(clock=clock)
        options = plan(hypotheses=(1,), profile_consume=True, max_ticks=2)
        profiling_clock = iter(range(1000, 2000, 10))
        with patch.object(live.observer.time, 'monotonic_ns', side_effect=lambda: next(profiling_clock)):
            runs = live.prepare_observers([Policy()], calibration(), mount(), None,
                                          options, FakeTorch, 2)
            seed(bus)
            result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        self.assertEqual(result['status'], 'COMPLETE_NO_OUTPUT_DIAGNOSTIC')
        ticks = [r for r in logs(bus) if r['kind'] == 'live_policy_tick']
        self.assertEqual(len(ticks), 2)
        for tick in ticks:
            record = tick['result']
            profile = record['consume_profile']
            self.assertEqual(record['h_hypothesis'], 1)
            self.assertTrue(profile['complete'])
            self.assertIn('model_call', profile['durations_ns'])
            self.assertEqual(profile['measured_total_ns'], sum(profile['durations_ns'].values()))
            self.assertFalse(profile['wall_clock_timing_verified'])
            self.assertFalse(record['output_allowed'])
            self.assertFalse(record['approved_for_runtime'])
        self.assertEqual(result['hypotheses'][0]['last_consume_profile'],
                         ticks[-1]['result']['consume_profile'])
        self.assertFalse(result['output_allowed'])
        self.assertFalse(result['live_50hz_verified'])

    def test_single_h1_keeps_labels_inputs_and_state(self):
        clock, bus = Clock(), live.SessionBus(clock=Clock())
        bus.clock = clock
        options = plan(hypotheses=(1,))
        policy = Policy()
        runs = live.prepare_observers([policy], calibration(), mount(), None, options, FakeTorch, 2)
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        self.assertEqual(result['status'], 'COMPLETE_NO_OUTPUT_DIAGNOSTIC')
        self.assertEqual(result['diagnostic_hypotheses'], [1])
        self.assertFalse(result['h_measured'])
        self.assertFalse(result['live_50hz_verified'])
        self.assertFalse(result['output_allowed'])
        ticks = [r for r in logs(bus) if r['kind'] == 'live_policy_tick']
        self.assertEqual([r['h_hypothesis'] for r in ticks], [1]*3)
        self.assertTrue(all(r['result']['inputs']['h_hypothesis12'] == [1.]*12 for r in ticks))
        self.assertEqual(policy.resets, 1)
        self.assertEqual(ticks[1]['result']['observation74'][33:45], ticks[0]['result']['actor_residual12'])

    def test_single_condition_does_not_relabel_failed_combined_budget(self):
        for hypotheses, expected in (((0, 1), 'INCOMPLETE'), ((0,), 'COMPLETE_NO_OUTPUT_DIAGNOSTIC'),
                                     ((1,), 'COMPLETE_NO_OUTPUT_DIAGNOSTIC')):
            with self.subTest(hypotheses=hypotheses):
                clock = Clock()
                class TwelveMsPolicy(Policy):
                    def __call__(self, *args):
                        answer = super().__call__(*args)
                        if self.resets: clock.now += 12_000_000
                        return answer
                options, bus = plan(max_ticks=1, hypotheses=hypotheses), live.SessionBus(clock=clock)
                runs = live.prepare_observers([TwelveMsPolicy() for _ in hypotheses], calibration(),
                                               mount(), None, options, FakeTorch, 2)
                seed(bus)
                result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
                self.assertEqual(result['status'], expected)
                self.assertEqual(options['max_tick_execution_window_ns'], 20_000_000)
                if len(hypotheses) == 2:
                    self.assertIn('exceeded20ms', result['first_blocked_tick']['reason'])

    def test_invalid_hypothesis_selection_or_mismatch_rejected(self):
        for selection in ((), (0, 0), (1, 0), (2,), (True,), (0.,), '0', None):
            with self.subTest(selection=selection), self.assertRaises(ValueError):
                plan(hypotheses=selection)
        with self.assertRaises(ValueError):
            live.prepare_observers([Policy(), Policy()], calibration(), mount(), None,
                                   plan(hypotheses=(1,)), FakeTorch, 2)
        options = plan(hypotheses=(1,))
        runs = live.prepare_observers([Policy()], calibration(), mount(), None, options, FakeTorch, 2)
        options['diagnostic_hypotheses'] = [0]
        clock = Clock()
        result = live.observe_live(live.SessionBus(clock=clock), runs, options,
                                  clock()+10**10, wait=clock.wait)
        self.assertIn('Observers do not match', result['first_blocked_tick']['reason'])
        self.assertEqual(runs[0].reset_count, 0)

    def test_slow_resets_finish_before_schedule_is_armed(self):
        clock = Clock()
        class SlowReset(Policy):
            def reset(self, ids):
                super().reset(ids)
                clock.now += 30_000_000
        bus, options = live.SessionBus(clock=clock), plan(max_ticks=1)
        policies = [SlowReset(), SlowReset()]
        runs = live.prepare_observers(policies, calibration(), mount(), None, options, FakeTorch, 2)
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        self.assertEqual(result['status'], 'COMPLETE_NO_OUTPUT_DIAGNOSTIC')
        self.assertEqual(result['first_tick_ns'], 1_080_000_000)
        self.assertEqual([p.resets for p in policies], [1, 1])
        rows = logs(bus)
        ends = [r for r in rows if r['kind'] == 'live_policy_reset_end']
        self.assertEqual([r['reset_duration_ns'] for r in ends], [30_000_000]*2)
        armed = next(r for r in rows if r['kind'] == 'live_schedule_armed')
        check = next(r for r in rows if r['kind'] == 'live_tick_start_check')
        self.assertEqual(armed['monotonic_ns'], ends[-1]['monotonic_ns'])
        self.assertEqual(check['start_lateness_ns'], 0)
        self.assertEqual(check['scheduled_tick_ns'], armed['first_tick_ns'])

    def test_reset_preparation_still_obeys_total_deadline(self):
        clock = Clock()
        class SlowReset(Policy):
            def reset(self, ids):
                super().reset(ids)
                clock.now += 30_000_000
        bus, options, policies = live.SessionBus(clock=clock), plan(max_ticks=1), [SlowReset(), Policy()]
        runs = live.prepare_observers(policies, calibration(), mount(), None, options, FakeTorch, 2)
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+25_000_000, wait=clock.wait)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIsNone(result['first_tick_ns'])
        self.assertEqual([p.resets for p in policies], [1, 0])
        self.assertFalse(any(r['kind'] == 'live_schedule_armed' for r in logs(bus)))

    def setup_run(self, *, options=None, policies=None):
        clock = Clock()
        bus = live.SessionBus(clock=clock)
        options = options or plan()
        policies = policies or [Policy(), Policy()]
        runs = live.prepare_observers(policies, calibration(), mount(), None, options, FakeTorch, 2)
        return clock, bus, options, policies, runs

    def test_three_real_observer_ticks_independent_state_and_one_reset(self):
        clock, bus, options, policies, runs = self.setup_run()
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        self.assertEqual(result['status'], 'COMPLETE_NO_OUTPUT_DIAGNOSTIC')
        self.assertTrue(result['identity_all12_verified'])
        self.assertFalse(result['output_allowed'])
        self.assertFalse(result['live_50hz_verified'])
        ticks = [r for r in logs(bus) if r['kind'] == 'live_policy_tick']
        self.assertEqual(len(ticks), 6)
        for h, p in enumerate(policies):
            self.assertEqual(p.resets, 1)
            self.assertEqual(len(p.calls), 5)  # two synthetic warmups, three sensor ticks
            records = [r for r in ticks if r['h_hypothesis'] == h]
            self.assertEqual([r['scheduled_tick_ns'] for r in records],
                             [1_020_000_000, 1_040_000_000, 1_060_000_000])
            self.assertEqual(records[0]['result']['observation74'][33:45], [0.]*12)
            self.assertEqual(records[1]['result']['observation74'][33:45],
                             records[0]['result']['actor_residual12'])
            self.assertEqual(records[0]['result']['inputs']['h_hypothesis12'], [float(h)]*12)
            self.assertEqual(records[0]['actual_started_ns'], records[0]['scheduled_tick_ns'])
            self.assertEqual(records[0]['result']['provenance']['oldest_observation_age_ns'], 23_000_000)
        self.assertTrue(bus.stop.is_set())

    def test_missing_source_times_out_without_fill_or_policy_reset(self):
        clock, bus, options, policies, runs = self.setup_run()
        seed(bus, omit=(12, 'velocity'))
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIsNone(result['first_tick_ns'])
        self.assertEqual([p.resets for p in policies], [0, 0])
        self.assertEqual([len(p.calls) for p in policies], [2, 2])

    def test_stale_first_tick_keeps_actual_source_time_and_blocks(self):
        clock, bus, options, policies, runs = self.setup_run(options=plan(max_age_ns=10_000_000))
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        blocked = result['first_blocked_tick']
        self.assertEqual(blocked['tick_index'], 0)
        self.assertEqual(blocked['snapshot']['motors'][0]['request_ns'], 997_000_000)
        self.assertEqual(blocked['snapshot']['oldest_observation_age_ns'], 23_000_000)
        self.assertEqual([len(p.calls) for p in policies], [2, 2])

    def test_late_arrival_with_old_source_time_cannot_repair_earlier_tick(self):
        clock, bus, options, policies, runs = self.setup_run(options=plan(max_ticks=2))
        seed(bus)
        delivered = False
        def wait(seconds):
            nonlocal delivered
            clock.wait(seconds)
            if clock() >= 1_020_000_000 and not delivered:
                delivered = True
                clock.now += 1_000_000
                bus.publish({'kind': 'motor_parameter', 'motor_id': 1, 'parameter': 'position',
                    'ok': True, 'status': 0, 'value': 100., 'unit': 'rad',
                    'request_monotonic_ns': 1_019_000_000, 'monotonic_ns': 1_020_000_000}, telemetry_input=True)
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=wait)
        self.assertEqual(result['first_blocked_tick']['tick_index'], 1)
        self.assertEqual([len(p.calls) for p in policies], [3, 3])
        ticks = [r for r in logs(bus) if r['kind'] == 'live_policy_tick']
        self.assertEqual([r['tick_index'] for r in ticks], [0, 0])

    def test_scheduler_lateness_never_catches_up(self):
        clock, bus, options, policies, runs = self.setup_run()
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+10**10,
            wait=lambda seconds: clock.wait(.030))
        self.assertEqual(result['first_blocked_tick']['tick_index'], 0)
        self.assertIn('deadline missed', result['first_blocked_tick']['reason'])
        self.assertEqual([len(p.calls) for p in policies], [2, 2])

    def test_slow_first_policy_prevents_second_h_and_next_tick(self):
        clock = Clock()
        class Slow(Policy):
            def __call__(self, *args):
                answer = super().__call__(*args)
                if self.resets: clock.now += 20_000_000
                return answer
        bus, options, policies = live.SessionBus(clock=clock), plan(), [Slow(), Policy()]
        runs = live.prepare_observers(policies, calibration(), mount(), None, options, FakeTorch, 2)
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        self.assertIn('exceeded20ms', result['first_blocked_tick']['reason'])
        self.assertEqual([len(p.calls) for p in policies], [3, 2])
        self.assertEqual([r.summary()['status'] for r in runs], ['INCOMPLETE', 'INCOMPLETE'])

    def test_range_failure_stops_both_without_clamping(self):
        clock, bus, options, policies, runs = self.setup_run()
        seed(bus)
        policies[0].bad = 'target_bounds'
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertEqual([len(p.calls) for p in policies], [3, 2])
        self.assertTrue(all(r.summary()['status'] == 'INCOMPLETE' for r in runs))

    def test_total_deadline_during_forward_cannot_be_reported_complete(self):
        clock = Clock()
        class Slow(Policy):
            def __call__(self, *args):
                result = super().__call__(*args)
                if self.resets: clock.now += 10_000_000
                return result
        bus, options, policies = live.SessionBus(clock=clock), plan(max_ticks=1), [Slow(), Policy()]
        runs = live.prepare_observers(policies, calibration(), mount(), None, options, FakeTorch, 2)
        seed(bus)
        result = live.observe_live(bus, runs, options, clock()+25_000_000, wait=clock.wait)
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertIn('deadline', result['first_blocked_tick']['reason'])
        self.assertEqual([len(p.calls) for p in policies], [3, 2])

    def test_boot_failure_stops_before_reset(self):
        clock, bus, options, policies, runs = self.setup_run()
        seed(bus)
        def changed(): raise RuntimeError('Boot changed')
        result = live.observe_live(bus, runs, options, clock()+10**10, wait=clock.wait, check_external=changed)
        self.assertIn('Boot changed', result['first_blocked_tick']['reason'])
        self.assertEqual([p.resets for p in policies], [0, 0])


class QueueTests(unittest.TestCase):
    def test_availability_timestamp_is_after_queue_insertion_not_before(self):
        clock, bus = Clock(90), live.SessionBus(clock=lambda: clock())
        original = bus.logs.put_nowait
        def preempted(row):
            original(row)
            clock.now = 110
        bus.logs.put_nowait = preempted
        source = {'kind': 'imu', 'read_finished_monotonic_ns': 80}
        bus.publish(source, telemetry_input=True)
        self.assertEqual(list(bus.available(100)), [])
        row = list(bus.available(110))[0]
        self.assertEqual(row['available_monotonic_ns'], 110)
        self.assertEqual(row['read_finished_monotonic_ns'], 80)
        self.assertNotIn('available_monotonic_ns', source)

    def test_overflow_poison_stops_instead_of_dropping_and_continuing(self):
        bus = live.SessionBus(input_capacity=1)
        bus.publish({'kind': 'a'}, telemetry_input=True)
        with self.assertRaisesRegex(RuntimeError, 'overflow'):
            bus.publish({'kind': 'b'}, telemetry_input=True)
        self.assertTrue(bus.stop.is_set())
        self.assertEqual(bus.dropped_events, 1)

    def test_bad_identity_barrier_future_nan_and_missing_imu_sequence_rejected(self):
        for mutation in ('identity', 'future', 'nan', 'sequence'):
            with self.subTest(mutation=mutation):
                clock, bus = Clock(), live.SessionBus(clock=lambda: clock())
                seed(bus)
                rows = list(bus.available(clock()))
                if mutation == 'identity': rows[0]['ids'].pop()
                elif mutation == 'future': rows[1]['monotonic_ns'] = clock()+1
                elif mutation == 'nan': rows[1]['value'] = float('nan')
                else: rows[-1]['sequence'] = 2
                inputs = live.LiveInputs(plan())
                with self.assertRaises(ValueError):
                    for row in rows: inputs.ingest(row)

    def test_live_ingest_cannot_erase_imu_correction_state(self):
        flags=('calibration_applied','orientation_applied','mount_rotation_applied',
               'gyro_bias_subtracted','accel_bias_subtracted','accel_scale_corrected',
               'mount_correction_applied','gyro_bias_correction_applied')
        for flag in flags:
            for value in (True,None,0,'false'):
                with self.subTest(flag=flag,value=value):
                    clock=Clock();bus=live.SessionBus(clock=clock);seed(bus)
                    rows=list(bus.available(clock()));rows[-1][flag]=value
                    inputs=live.LiveInputs(plan())
                    for row in rows[:-1]:inputs.ingest(row)
                    with self.assertRaisesRegex(ValueError,'raw IMU correction'):
                        inputs.ingest(rows[-1])
                    self.assertEqual(inputs.last_imu_sequence,0)
                    self.assertFalse(inputs.ready())

    def test_audit_writer_serializes_committed_times_and_flushes(self):
        bus = live.SessionBus(clock=Clock())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'events.jsonl'
            writer = live.AuditWriter(path, bus)
            writer.start()
            for n in range(20): bus.publish({'kind': 'test', 'n': n})
            writer.close()
            self.assertTrue(writer.flushed)
            self.assertEqual(bus.errors, [])
            records = [json.loads(row) for row in path.read_text().splitlines()]
            self.assertEqual([r['n'] for r in records], list(range(20)))
            self.assertTrue(all(r['available_monotonic_ns'] == 1_000_000_000 for r in records))
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_writer_start_failure_closes_fd_without_joining_unstarted_thread(self):
        bus = live.SessionBus()
        with tempfile.TemporaryDirectory() as directory:
            writer = live.AuditWriter(Path(directory)/'events.jsonl', bus)
            with patch.object(writer.thread, 'start', side_effect=RuntimeError('no thread')):
                with self.assertRaisesRegex(RuntimeError, 'no thread'): writer.start()
            self.assertIsNone(writer.descriptor)
            writer.close()
            self.assertFalse(writer.flushed)


class ProducerTests(unittest.TestCase):
    def run_can(self, *, wrong_id=None, close_error=False, query_error=None):
        clock, bus = Clock(), live.SessionBus(clock=lambda: clock())
        calls, order = [], []
        @contextlib.contextmanager
        def locks():
            order.append('locks_acquired')
            try: yield
            finally: order.append('locks_released')
        class CAN:
            parser = type('Parser', (), {'buffer': b'', 'discarded_bytes': 0})()
            def __init__(self, event_sink): self.sink = event_sink
            def __enter__(self): order.append('opened'); return self
            def __exit__(self, *args):
                order.append('close_attempt')
                if close_error: raise OSError('close failed')
                order.append('closed')
            def query(self, mid, parameter=None):
                calls.append((mid, parameter))
                # Verify every generated request belongs to the existing read-only codec.
                wire = live.codec.read_request(mid, parameter)
                self.sink({'kind': 'can_tx', 'hex': wire.hex(), 'monotonic_ns': clock()})
                if query_error and len(calls) == query_error: raise TimeoutError('request timeout')
                if parameter is None:
                    return {'ok': True, 'mcu_uid_hex': 'wrong' if mid == wrong_id else calibration()['identities'][str(mid)]}
                if len(calls) == 36: bus.stop.set()
                return {'ok': True}
        live.can_producer(bus, calibration()['identities'], clock()+10**10,
                          can_factory=CAN, lock_factory=locks)
        return bus, calls, order

    def test_single_can_owner_only_twelve_identities_then_24_read_queries(self):
        bus, calls, order = self.run_can()
        self.assertEqual(calls, [(i, None) for i in range(1, 13)]+
                         [(i, p) for i in range(1, 13) for p in ('position', 'velocity')])
        self.assertEqual(order, ['locks_acquired', 'opened', 'close_attempt', 'closed', 'locks_released'])
        self.assertEqual(bus.errors, [])
        self.assertTrue(bus.producer_status['can']['device_context_exited'])

    def test_identity_mismatch_and_query_timeout_never_retry_or_start_telemetry(self):
        for kw in ({'wrong_id': 6}, {'query_error': 6}):
            bus, calls, order = self.run_can(**kw)
            self.assertEqual(calls, [(i, None) for i in range(1, 7)])
            self.assertTrue(bus.errors)
            self.assertNotIn('identity_all12_verified', [r['kind'] for r in logs(bus)])
            self.assertLess(order.index('closed'), order.index('locks_released'))

    def test_close_failure_after_normal_stop_is_not_suppressed(self):
        bus, calls, order = self.run_can(close_error=True)
        self.assertEqual(len(calls), 36)
        self.assertTrue(bus.stop.is_set())
        self.assertIn('close failed', bus.errors[0]['error'])
        self.assertFalse(bus.producer_status['can']['device_context_exited'])

    def test_blocked_can_thread_keeps_locks_until_actual_close(self):
        bus, entered, release = live.SessionBus(), threading.Event(), threading.Event()
        order = []
        @contextlib.contextmanager
        def locks():
            order.append('locks_acquired')
            try: yield
            finally: order.append('locks_released')
        class CAN:
            parser = type('Parser', (), {'buffer': b'', 'discarded_bytes': 0})()
            def __init__(self, **kw): pass
            def __enter__(self): return self
            def __exit__(self, *args): order.append('closed')
            def query(self, mid):
                entered.set(); release.wait(1.)
                return {'ok': True, 'mcu_uid_hex': calibration()['identities'][str(mid)]}
        worker = threading.Thread(target=live.can_producer,
            args=(bus, calibration()['identities'], bus.clock()+10**10),
            kwargs={'can_factory': CAN, 'lock_factory': locks})
        worker.start()
        try:
            self.assertTrue(entered.wait(.5))
            bus.stop.set(); worker.join(.01)
            self.assertTrue(worker.is_alive())
            self.assertEqual(order, ['locks_acquired'])
            self.assertNotIn('can', bus.producer_status)
        finally:
            release.set(); worker.join(1.)
        self.assertEqual(order, ['locks_acquired', 'closed', 'locks_released'])

    def test_imu_normal_stop_restores_and_close_failure_remains_error(self):
        for failed in (False, True):
            bus, order = live.SessionBus(), []
            class IMU:
                original_registers = {'test': 1}
                restore_status = 'not_started'
                def start(self): return {'source': 'fake'}
                def read_sample(self): bus.stop.set(); return None
                def close(self):
                    order.append('close')
                    self.restore_status = 'failed' if failed else 'restored'
                    if failed: raise OSError('restore failed')
            live.imu_producer(bus, bus.clock()+10**10, imu_factory=IMU,
                              lock_factory=contextlib.nullcontext)
            self.assertEqual(order, ['close'])
            self.assertEqual(bool(bus.errors), failed)
            self.assertEqual(bus.producer_status['imu']['restore_status'], 'failed' if failed else 'restored')

    def test_fault_or_running_feedback_aborts_after_logging(self):
        for event in ({'type': 21}, {'type': 2, 'fault_bits': 1}, {'type': 2, 'mode_state': 2}):
            bus = live.SessionBus()
            with self.assertRaises(RuntimeError): live._can_event(bus, {'kind': 'motor_feedback', **event})
            self.assertEqual(len(logs(bus)), 1)


class CLITests(unittest.TestCase):
    def test_single_h_plan_and_profiling_are_explicit_without_device_access(self):
        with patch.object(live, 'run_acquisition', side_effect=AssertionError('hardware forbidden')):
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(live.main(self.args('/missing/untouched')+
                    ['--hypothesis', '1', '--profile-consume']), 0)
        data = json.loads(output.getvalue())
        self.assertEqual(data['diagnostic_hypotheses'], [1])
        self.assertTrue(data['profile_consume'])
        self.assertFalse(data['output_allowed'])
        self.assertFalse(data['h_measured'])

    def args(self, output, execute=False):
        return (['--execute-no-output'] if execute else [])+[
            '--calibration', '/missing/calibration', '--bundle', '/missing/bundle',
            '--imu-mount-candidate', '/missing/mount', '--max-age-ms', '100',
            '--max-spread-ms', '100', '--output', str(output)]

    def test_default_plan_does_not_touch_files_or_hardware(self):
        with patch.object(live, 'run_acquisition', side_effect=AssertionError('hardware forbidden')):
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(live.main(self.args('/missing/untouched')), 0)
        data = json.loads(output.getvalue())
        self.assertEqual(data['allowed_can_types'], [0, 17])
        self.assertFalse(data['output_allowed'])

    def test_preflight_failure_saves_incomplete_private_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'new'
            with patch.object(live, 'run_acquisition', side_effect=AssertionError('hardware forbidden')):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(live.main(self.args(output, True)), 2)
            summary = json.loads((output/'summary.json').read_text())
            self.assertEqual(summary['status'], 'INCOMPLETE')
            self.assertEqual(summary['failure_phase'], 'input_validation')
            self.assertIn('policy_observer_live.py', summary['source_sha256'])
            self.assertEqual((output/'summary.json').stat().st_mode & 0o777, 0o600)
            self.assertFalse(summary['output_allowed'])
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                live.main(self.args(output, True))

    def test_cli_warmed_fake_run_saves_hashes_and_summary_after_writer_start_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cal, im = root/'cal.json', root/'mount.json'
            cal.write_text(json.dumps(calibration())); im.write_text(json.dumps(mount()))
            args = self.args(root/'new', True)
            args[args.index('--calibration')+1] = str(cal)
            args[args.index('--imu-mount-candidate')+1] = str(im)
            original = Path.read_text
            def read(path, *a, **kw):
                if str(path) == '/proc/sys/kernel/random/boot_id': return 'fake-boot'
                return original(path, *a, **kw)
            with patch.object(Path, 'read_text', read), \
                 patch.object(live.shadow, 'load_policy', side_effect=lambda _: (Policy(), {'test': True})), \
                 patch.dict('sys.modules', {'torch': FakeTorch}), \
                 patch.object(threading.Thread, 'start', side_effect=RuntimeError('thread unavailable')), \
                 patch.object(live, 'run_acquisition', side_effect=AssertionError('hardware forbidden')), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(live.main(args), 2)
            report = json.loads((root/'new'/'summary.json').read_text())
            self.assertEqual(report['status'], 'INCOMPLETE')
            self.assertIn('thread unavailable', report['failure'])
            self.assertEqual(report['failure_phase'], 'acquisition')
            self.assertFalse(report['log_flush_confirmed'])
            self.assertEqual(report['boot_id'], 'fake-boot')
            self.assertEqual(len(report['input_sha256']['calibration']), 64)

    def test_cli_complete_run_and_signal_during_cleanup_have_distinct_status(self):
        for interrupt_cleanup in (False, True):
            with self.subTest(interrupt_cleanup=interrupt_cleanup), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                cal, im = root/'cal.json', root/'mount.json'
                cal.write_text(json.dumps(calibration())); im.write_text(json.dumps(mount()))
                args = self.args(root/'new', True)
                args[args.index('--calibration')+1] = str(cal)
                args[args.index('--imu-mount-candidate')+1] = str(im)
                original_read, original_close = Path.read_text, live.AuditWriter.close
                def read(path, *a, **kw):
                    if str(path) == '/proc/sys/kernel/random/boot_id': return 'fake-boot'
                    return original_read(path, *a, **kw)
                def close(writer):
                    if interrupt_cleanup: signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
                    return original_close(writer)
                def acquire(bus, runs, calibration, options, **kw):
                    bus.publish({'kind': 'fake_acquisition'})
                    return {'status': 'COMPLETE_NO_OUTPUT_DIAGNOSTIC'}
                with patch.object(Path, 'read_text', read), \
                     patch.object(live.shadow, 'load_policy', side_effect=lambda _: (Policy(), {'test': True})), \
                     patch.dict('sys.modules', {'torch': FakeTorch}), \
                     patch.object(live.AuditWriter, 'close', close), \
                     patch.object(live, 'run_acquisition', acquire), \
                     contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(live.main(args), 2 if interrupt_cleanup else 0)
                report = json.loads((root/'new'/'summary.json').read_text())
                self.assertTrue(report['log_flush_confirmed'])
                self.assertEqual(report['signals'], [signal.SIGTERM] if interrupt_cleanup else [])
                self.assertEqual(report['status'], 'INCOMPLETE' if interrupt_cleanup else 'COMPLETE_NO_OUTPUT_DIAGNOSTIC')

    def test_finite_budget_and_git_output_rejected(self):
        for changes in ({'max_ticks': 501}, {'max_seconds': float('nan')},
                        {'max_seconds': 11}, {'max_lateness_ns': 20_000_000},
                        {'max_age_ns': 0}, {'max_ticks': 100, 'max_seconds': 1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError): plan(**changes)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root/'.git').mkdir()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                live.main(self.args(root/'new', True))
            self.assertFalse((root/'new').exists())


if __name__ == '__main__':
    unittest.main()
