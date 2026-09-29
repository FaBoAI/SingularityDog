"""Opt-in cadence/source contracts using fake buses; never robot evidence."""
from contextlib import redirect_stdout
import copy
import io
import json
import struct
import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import patch

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_output as cli
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import ground_trial_review as ground
import test_policy_output_runtime as runtime_tests
import test_policy_live_profile as profile_tests
from test_policy_output_runtime import FakeSession, wire, quantize
from test_policy_live_profile import _write
from test_ground_trial_review import fixture as ground_fixture, native_record


def profile():
    # This fixture bypasses the file approval loader and is fake-bus only.
    # OS scheduling in unit tests does not prove or approve hardware timing.
    value = runtime_tests.profile()
    value['max_consecutive_20ms_misses'] = 100
    return value


def new_profile():
    value = profile()
    value.update(schema=live.SCHEMA_V3, telemetry_cadence=live.CADENCE_PRE_ENABLE,
                 cadence_source_sha256=live.cadence_source_hashes())
    # These fake-bus tests validate bytes and safety branches, not OS timing.
    # Keep unrelated host scheduling pauses out of the protocol assertions.
    value.update(hard_cycle_ms=500.,max_sample_age_ms=500.,max_sample_gap_ms=500.)
    return value


def parameter_name(frame):
    if frame.kind != 17:
        return None
    return next(name for name, entry in codec.PARAMETERS.items()
                if entry[0] == int.from_bytes(frame.data[:2], 'little'))


class MonitoringSession(FakeSession):
    """Inject responses only; the runtime still emits real protocol bytes."""
    def __init__(self, first_id, *, timeout_ticks=4000, missing_timeout=False,
                 drift_after_enable=False, acquisition_failure=None,
                 pre_enable_voltage_failure=None,initial_voltage_failure=None,
                 after_enable_voltage_failure=None):
        super().__init__(first_id)
        self.timeout_ticks = timeout_ticks
        self.missing_timeout = missing_timeout
        self.drift_after_enable = drift_after_enable
        self.acquisition_failure = acquisition_failure
        self.pre_enable_voltage_failure = pre_enable_voltage_failure
        self.initial_voltage_failure = initial_voltage_failure
        self.after_enable_voltage_failure = after_enable_voltage_failure

    def _exchange(self, wires, timeout_ns, send_only):
        wires = tuple(wires)
        frames = [codec.ATParser().feed(w)[0] for w in wires]
        acquisition = any(parameter_name(f) == 'voltage' for f in frames) and any(f.kind == 1 for f in frames)
        voltage_refresh = len(frames) == 6 and all(parameter_name(f) == 'voltage' for f in frames)
        initial_voltage = (not voltage_refresh and not acquisition and
                           any(parameter_name(f) == 'run_mode' for f in frames) and
                           any(parameter_name(f) == 'voltage' for f in frames))
        result = super()._exchange(wires, timeout_ns, send_only)
        for record in result[0]:
            tx = codec.ATParser().feed(bytes(record.tx))[0]
            name = parameter_name(tx)
            if name == 'can_timeout' and tx.destination == self.ids[-1]:
                if self.missing_timeout:
                    record.received = 0
                    record.received_ns = 0
                else:
                    ticks = 0 if self.drift_after_enable and self.enabled else self.timeout_ticks
                    body = tx.data[:4] + struct.pack('<I', ticks)
                    record.rx[:] = wire((17 << 24) | (tx.destination << 8) | codec.HOST_ID, body)
            voltage_failure=(self.after_enable_voltage_failure if voltage_refresh and self.enabled else
                             self.pre_enable_voltage_failure if voltage_refresh else
                             self.initial_voltage_failure if initial_voltage else None)
            if name == 'voltage' and tx.destination == self.ids[-1] and voltage_failure:
                if voltage_failure == 'missing':
                    record.received = 0;record.received_ns = 0
                elif voltage_failure == 'stale':
                    record.received_ns = record.start_ns - 1
                elif voltage_failure == 'low':
                    record.rx[:] = wire((17 << 24) | (tx.destination << 8) | codec.HOST_ID,
                                        tx.data[:4] + struct.pack('<f', 34.))
            if not acquisition or self.acquisition_failure is None:
                continue
            failure = self.acquisition_failure
            if name == 'voltage':
                if failure == 'voltage_missing':
                    record.received = 0
                    record.received_ns = 0
                elif failure == 'voltage_stale':
                    record.received_ns = record.start_ns - 1
                elif failure in ('voltage_low', 'voltage_nonfinite'):
                    value = 34. if failure == 'voltage_low' else float('nan')
                    record.rx[:] = wire((17 << 24) | (tx.destination << 8) | codec.HOST_ID,
                                        tx.data[:4] + struct.pack('<f', value))
            if tx.kind == 1 and tx.destination == self.ids[0]:
                frame = codec.ATParser().feed(bytes(record.rx))[0]
                cid = frame.can_id
                values = list(struct.unpack('>4H', frame.data))
                if failure == 'fault':
                    cid |= 1 << 16
                elif failure == 'disabled_mode':
                    cid &= ~(3 << 22)
                elif failure == 'torque':
                    values[2] = quantize(4., -5.5, 5.5)
                elif failure == 'velocity':
                    values[1] = quantize(2., -50., 50.)
                elif failure == 'temperature':
                    values[3] = 650
                record.rx[:] = wire(cid, struct.pack('>4H', *values))
        return result


class CadenceRuntimeTests(unittest.TestCase):
    run_case = runtime_tests.OutputRuntimeTests.run_case

    def test_v3_two_all_axis_timeout_reads_before_enable_and_no_cyclic_queries(self):
        announced = []
        # The fake bus proves request structure, not host scheduling. Separate
        # tests below exercise the actual 126 ms voltage-age boundary.
        with patch.object(runtime,'V3_VOLTAGE_MAX_AGE_NS',1_000_000_000):
            report, sessions = self.run_case(profile_data=new_profile(),
                                             announce=lambda: announced.append(time.monotonic_ns()))
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertTrue(report['after_announcement_watchdog_verified'])
        self.assertEqual(set(report['after_announcement_watchdog_readback_by_id']),
                         {str(i) for i in range(1, 13)})
        self.assertEqual(report['telemetry_cadence']['total_requests_per_cycle_including_output'], 26)
        self.assertFalse(report['telemetry_cadence']['timeout_parameter_drift_monitored_during_cycles'])
        for session in sessions.values():
            enabled = min(call[0] for call in session.calls if call[1] == 3)
            for mid in session.ids:
                reads = [call for call in session.calls if call[2] == mid and
                         parameter_name(codec.ATParser().feed(call[3])[0]) == 'can_timeout']
                self.assertEqual(len(reads), 2)
                self.assertLess(reads[0][0], announced[0])
                self.assertGreaterEqual(reads[1][0], announced[0])
                self.assertLess(reads[1][0], enabled)
        journals = report['journal']
        for bus, ids in runtime.BUSES.items():
            batches = [b for b in journals if b['phase'] == 'feedback_hold' and b['bus'] == bus]
            self.assertGreaterEqual(len(batches), 6)
            for index, batch in enumerate(batches):
                frames = [codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0] for row in batch['records']]
                self.assertEqual(len(frames), 7)
                self.assertEqual([f.destination for f in frames if f.kind == 1], list(ids))
                self.assertEqual([(f.destination, parameter_name(f)) for f in frames if f.kind == 17],
                                 [(ids[index % 6], 'voltage')])
            self.assertEqual({codec.ATParser().feed(bytes.fromhex(b['records'][-1]['tx_hex']))[0].destination
                              for b in batches[:6]}, set(ids))
        final = [b for b in journals if b['phase'] == 'watchdog_pre_enable_readback']
        pose = [b for b in journals if b['phase'] == 'pre_enable_pose']
        refresh = [b for b in journals if b['phase'] == 'voltage_pre_enable_refresh']
        after_enable = [b for b in journals if b['phase'] == 'voltage_after_enable_refresh']
        self.assertEqual(len(refresh),2)
        self.assertEqual(len(after_enable),2)
        self.assertEqual({codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].destination
                          for b in refresh for row in b['records']},set(range(1,13)))
        self.assertEqual({codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].destination
                          for b in after_enable for row in b['records']},set(range(1,13)))
        for batch in after_enable:
            for row in batch['records']:
                mid = codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].destination
                voltage = codec.decode_reply(
                    codec.ATParser().feed(bytes.fromhex(row['rx_hex']))[0],mid,'voltage')['value']
                self.assertEqual(report['voltage_guard']['after_enable_refresh_by_id'][str(mid)],
                                 {'value_v':voltage,'received_ns':row['received_ns']})
        self.assertLess(max(r['received_ns'] for b in final for r in b['records']),
                        min(r['start_ns'] for b in pose for r in b['records']))
        self.assertLess(max(r['received_ns'] for b in pose for r in b['records']),
                        min(r['start_ns'] for b in refresh for r in b['records']))
        self.assertLess(max(r['received_ns'] for b in refresh for r in b['records']),
                        min(c[0] for session in sessions.values() for c in session.calls if c[1] == 3))
        last_transition = max(c[0] for session in sessions.values()
                              for c in session.calls if c[1] == 3)
        self.assertLess(last_transition,
                        min(r['start_ns'] for b in after_enable for r in b['records']))
        after_enable_end = max(r['received_ns'] for b in after_enable for r in b['records'])
        final_zero_hold = [b for b in journals if b['phase'] == 'preflight' and
                           len(b['records']) == 6 and all(
                               codec.ATParser().feed(bytes.fromhex(r['tx_hex']))[0].kind == 1
                               for r in b['records']) and
                           min(r['start_ns'] for r in b['records']) > after_enable_end]
        self.assertEqual(len(final_zero_hold),2)
        self.assertLess(after_enable_end,
                        min(r['start_ns'] for b in final_zero_hold for r in b['records']))
        self.assertLess(max(r['received_ns'] for b in final_zero_hold for r in b['records']),
                        min(r['start_ns'] for b in journals if b['phase'] == 'feedback_hold'
                            for r in b['records']))
        self.assertEqual(set(report['voltage_guard']['pre_enable_refresh_by_id']),
                         {str(i) for i in range(1,13)})
        self.assertGreater(report['voltage_guard']['checks_before_type1'],0)
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['telemetry_cadence']['post_stop_timeout_parameter_readback'])
        for session in sessions.values():
            self.assertLess(max(c[0] for c in session.calls), session.stop_times[0])

    def test_initial_bad_or_missing_timeout_on_either_bus_aborts_before_enable(self):
        for first in (1, 7):
            for missing in (False, True):
                session = MonitoringSession(first, timeout_ticks=0, missing_timeout=missing)
                kwargs = {'front' if first == 1 else 'rear': session}
                with self.subTest(first=first, missing=missing):
                    report, sessions = self.run_case(profile_data=new_profile(), **kwargs)
                    self.assertEqual(report['status'], 'ABORTED')
                    self.assertFalse(report['motor_enable_sent'])
                    self.assertTrue(all(all(c[1] != 3 for c in s.calls) for s in sessions.values()))
                    self.assertTrue(report['stop_confirmed'])

    def test_pre_enable_voltage_refresh_rejects_missing_stale_and_low(self):
        for failure in ('missing','stale','low'):
            with self.subTest(failure=failure):
                report,sessions=self.run_case(profile_data=new_profile(),
                    rear=MonitoringSession(7,pre_enable_voltage_failure=failure))
                self.assertEqual(report['status'],'ABORTED',report['errors'])
                self.assertFalse(report['motor_enable_attempted'])
                self.assertFalse(report['motor_enable_sent'])
                self.assertTrue(report['stop_confirmed'])
                self.assertTrue(any('voltage' in error.lower() or 'noncausal' in error.lower()
                                    for error in report['errors']))
                self.assertFalse(any(call[1] in (1,3) for session in sessions.values()
                                     for call in session.calls))

    def test_after_enable_voltage_refresh_rejects_missing_stale_and_low_before_cycles(self):
        for failure in ('missing','stale','low'):
            with self.subTest(failure=failure):
                report,sessions=self.run_case(profile_data=new_profile(),
                    rear=MonitoringSession(7,after_enable_voltage_failure=failure))
                self.assertEqual(report['status'],'ABORTED',report['errors'])
                self.assertTrue(report['motor_enable_sent'])
                self.assertFalse(report['cycles'])
                self.assertEqual(report['voltage_guard']['after_enable_refresh_by_id'],{})
                self.assertTrue(report['stop_confirmed'])
                self.assertTrue(any('voltage' in error.lower() or 'noncausal' in error.lower()
                                    for error in report['errors']))
                self.assertFalse(any(batch['phase']=='feedback_hold' for batch in report['journal']))
                self.assertFalse(any(batch['phase']=='preflight' and len(batch['records'])==6
                                     and all(codec.ATParser().feed(bytes.fromhex(r['tx_hex']))[0].kind==1
                                             for r in batch['records'])
                                     for batch in report['journal']))

    def test_initial_preflight_voltage_rejects_missing_stale_and_low(self):
        for failure in ('missing','stale','low'):
            with self.subTest(failure=failure):
                report,sessions=self.run_case(profile_data=new_profile(),
                    front=MonitoringSession(1,initial_voltage_failure=failure))
                self.assertEqual(report['status'],'ABORTED',report['errors'])
                self.assertFalse(report['motor_enable_attempted'])
                self.assertFalse(report['motor_enable_sent'])
                self.assertTrue(report['stop_confirmed'])
                self.assertFalse(any(call[1] in (1,3) for session in sessions.values()
                                     for call in session.calls))
                self.assertFalse(any(batch['phase']=='voltage_pre_enable_refresh'
                                     for batch in report['journal']))

    def test_all_axis_voltage_cache_has_an_exact_126ms_boundary(self):
        now=1_000_000_000
        cache={i:(40.,now-runtime.V3_VOLTAGE_MAX_AGE_NS) for i in runtime.IDS}
        self.assertEqual(runtime.checked_voltage_cache(cache,new_profile(),now),
                         (runtime.V3_VOLTAGE_MAX_AGE_NS,40.))
        for failure in ('missing','stale','low','nonfinite'):
            broken=dict(cache)
            if failure=='missing':del broken[12]
            elif failure=='stale':broken[12]=(40.,now-runtime.V3_VOLTAGE_MAX_AGE_NS-1)
            elif failure=='low':broken[12]=(34.9,now)
            else:broken[12]=(float('nan'),now)
            with self.subTest(failure=failure),self.assertRaisesRegex(RuntimeError,'voltage'):
                runtime.checked_voltage_cache(broken,new_profile(),now)

    def test_stale_other_axis_aborts_live_cycle_before_another_type1(self):
        # Accelerate the fake bus's age boundary so OS timer jitter cannot
        # decide this test; the exact production boundary is asserted above.
        with patch.object(runtime,'V3_VOLTAGE_MAX_AGE_NS',80_000_000):
            report,sessions=self.run_case(profile_data=new_profile())
        self.assertEqual(report['status'],'ABORTED',report['errors'])
        self.assertTrue(any('voltage stale' in error for error in report['errors']))
        self.assertTrue(report['motor_enable_sent'])
        self.assertTrue(report['stop_confirmed'])
        self.assertGreater(report['voltage_guard']['checks_before_type1'],0)
        self.assertTrue(any(session.positive_gain_writes for session in sessions.values()))

    def test_timeout_change_during_announcement_is_rejected_before_enable(self):
        rear = MonitoringSession(7)
        report, sessions = self.run_case(profile_data=new_profile(), rear=rear,
                                         announce=lambda: setattr(rear, 'timeout_ticks', 0))
        self.assertEqual(report['status'], 'ABORTED')
        self.assertTrue(any('pre-enable watchdog readback' in e for e in report['errors']))
        self.assertFalse(report['motor_enable_sent'])
        self.assertTrue(all(all(c[1] != 3 for c in s.calls) for s in sessions.values()))
        self.assertTrue(report['stop_confirmed'])

    def test_all_physical_feedback_and_voltage_faults_abort_before_inference(self):
        for failure in ('fault', 'disabled_mode', 'torque', 'velocity', 'temperature',
                        'voltage_missing', 'voltage_stale', 'voltage_low', 'voltage_nonfinite'):
            calls = []
            with self.subTest(failure=failure):
                report, _ = self.run_case(profile_data=new_profile(),
                    front=MonitoringSession(1, acquisition_failure=failure),
                    policy=lambda *args: calls.append(args) or (.04,) * 12)
                self.assertEqual(report['status'], 'ABORTED', report['errors'])
                self.assertEqual(calls, [])
                self.assertTrue(report['stop_confirmed'])

    def test_legacy_v2_still_queries_rotating_timeout_and_reports_28_requests(self):
        report, sessions = self.run_case(profile_data=profile())
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertEqual(report['telemetry_cadence']['total_requests_per_cycle_including_output'], 28)
        self.assertTrue(report['telemetry_cadence']['timeout_parameter_drift_monitored_during_cycles'])
        self.assertFalse(report['after_announcement_watchdog_verified'])
        batches = [b for b in report['journal'] if b['phase'] == 'feedback_hold']
        self.assertTrue(all(len(b['records']) == 8 for b in batches))
        for session in sessions.values():
            self.assertEqual(sum(parameter_name(codec.ATParser().feed(c[3])[0]) == 'can_timeout'
                                 for c in session.calls), 6 + len(report['cycles']))

    def test_contract_explicitly_distinguishes_config_drift_detection(self):
        # A changed static parameter is intentionally no longer cyclically read.
        # The device watchdog loss-proof and mode/fault checks are separate gates.
        for value, expect_complete in ((profile(), False), (new_profile(), True)):
            with self.subTest(schema=value['schema']):
                with patch.object(runtime,'V3_VOLTAGE_MAX_AGE_NS',1_000_000_000):
                    report, _ = self.run_case(profile_data=value,
                        front=MonitoringSession(1, drift_after_enable=True))
                self.assertEqual(report['status'] == 'COMPLETE_SUPPORTED_OUTPUT', expect_complete, report['errors'])
                self.assertEqual(report['telemetry_cadence']['timeout_parameter_drift_monitored_during_cycles'],
                                 not expect_complete)


class CadenceProfileTests(unittest.TestCase):
    setUp = profile_tests.ProfileTests.setUp
    save = profile_tests.ProfileTests.save

    def select_v3(self):
        self.data.update(schema=live.SCHEMA_V3, telemetry_cadence=live.CADENCE_PRE_ENABLE,
                         cadence_source_sha256=live.cadence_source_hashes())

    def test_v3_cannot_reuse_a_v2_hardware_review_digest(self):
        old_digest = live.reviewed_settings_sha256(self.data)
        self.select_v3()
        self.assertNotEqual(live.reviewed_settings_sha256(self.data), old_digest)
        with self.assertRaisesRegex(live.ProfileError, 'exact gains, limits'):
            live.load_profile(self.save())
        parsed = live.load_profile(self.save(bind_review=True))
        self.assertEqual(live.telemetry_settings(parsed)['total_requests_per_cycle_including_output'], 26)
        self.assertFalse(parsed['actual_policy_output_20ms_verified'])

    def test_invalid_cadence_or_stale_source_pin_is_rejected_even_after_review_rebind(self):
        self.select_v3()
        self.data['telemetry_cadence'] = 'pretend_all_monitors_identical'
        with self.assertRaisesRegex(live.ProfileError, 'telemetry cadence'):
            live.load_profile(self.save())
        self.data['telemetry_cadence'] = live.CADENCE_PRE_ENABLE
        first = live.CADENCE_SOURCE_PATHS[0]
        self.data['cadence_source_sha256'][first] = '0' * 64
        with self.assertRaisesRegex(live.ProfileError, 'Cadence source SHA256 mismatch'):
            live.load_profile(self.save(bind_review=True))

    def test_all_source_pin_names_and_hashes_are_required(self):
        self.select_v3()
        original = copy.deepcopy(self.data['cadence_source_sha256'])
        for name in live.CADENCE_SOURCE_PATHS:
            self.data['cadence_source_sha256'] = dict(original)
            del self.data['cadence_source_sha256'][name]
            with self.subTest(name=name), self.assertRaisesRegex(live.ProfileError, 'Complete cadence source pins'):
                live.load_profile(self.save())

    def test_legacy_schema_rejects_a_new_cadence_field(self):
        self.data['telemetry_cadence'] = live.CADENCE_PRE_ENABLE
        with self.assertRaisesRegex(live.ProfileError, 'Unsupported profile fields'):
            live.load_profile(self.save())

    def test_plan_reports_cadence_and_opens_no_hardware(self):
        candidate = live.template(schema=live.SCHEMA_V3)
        path = self.base/'cadence-plan.json'
        _write(path, candidate)
        stdout = io.StringIO()
        with patch('singularitydog_hw.native_active_transport.load_library') as native, \
             patch('singularitydog_hw.imu.ICM20948') as imu, \
             patch.object(cli.subprocess, 'run') as audio, redirect_stdout(stdout):
            self.assertEqual(cli.main(['--profile', str(path)]), 0)
        plan = json.loads(stdout.getvalue())
        self.assertFalse(plan['output_allowed'])
        self.assertFalse(plan['hardware_opened'])
        self.assertEqual(plan['telemetry_cadence']['total_requests_per_cycle_including_output'], 26)
        self.assertFalse(plan['telemetry_cadence']['timeout_parameter_drift_monitored_during_cycles'])
        for method in (native, imu, audio):
            method.assert_not_called()

    def test_default_template_remains_v2_unapproved(self):
        default = live.template()
        explicit = live.template(schema=live.SCHEMA_V3)
        self.assertEqual(default['schema'], live.SCHEMA_V2)
        self.assertNotIn('telemetry_cadence', default)
        self.assertFalse(default['approved_for_supported_policy_output'])
        self.assertFalse(explicit['approved_for_supported_policy_output'])
        self.assertIsNone(explicit['review'])
        self.assertTrue(explicit['blockers'])


def v3_ground_fixture(base):
    profile_data, docs, plan, report = ground_fixture(base)
    profile_data.update(schema=live.SCHEMA_V3, telemetry_cadence=live.CADENCE_PRE_ENABLE,
                        cadence_source_sha256=live.cadence_source_hashes())
    profile_data['watchdog_by_id'] = docs['hardware_review']['device_watchdog']
    result = report['runtime_report']
    result.update(telemetry_cadence=live.telemetry_settings(profile_data),
                  cadence_source_sha256=profile_data['cadence_source_sha256'],
                  announcement_completed_ns=1_990_000_000,
                  pre_enable_imu={'read_started_monotonic_ns':1_997_000_000,
                                  'read_finished_monotonic_ns':1_997_100_000})
    additions = []
    for bus, ids in runtime.BUSES.items():
        setup = []
        final = []
        pose = []
        refresh = []
        for mid in ids:
            tx = runtime.protocol.watchdog_setup_request(phase=runtime.protocol.TrialPhase.WATCHDOG_SETUP,
                                                        motor_id=mid)
            t = 1_930_000_000 + mid * 10_000
            setup.append(native_record(tx,wire((2<<24)|(mid<<8)|codec.HOST_ID,
                                              struct.pack('>4H',32767,32767,32767,250)),t))
            tx = codec.read_request(mid, 'can_timeout')
            body = codec.PARAMETERS['can_timeout'][0].to_bytes(2, 'little') + b'\x00\x00' + struct.pack('<I', 4000)
            rx = wire((17 << 24) | (mid << 8) | codec.HOST_ID, body)
            final.append(native_record(tx, rx, 1_992_000_000 + mid * 10_000))
            payload = next(bytes.fromhex(row['rx_hex'])[7:15]
                           for batch in result['journal'] if batch['phase'] == 'policy_output'
                           for row in batch['records']
                           if codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].destination == mid)
            tx = runtime.protocol.stop_request(phase=runtime.protocol.TrialPhase.STOP, motor_id=mid)
            pose.append(native_record(tx, wire((2 << 24) | (mid << 8) | codec.HOST_ID, payload),
                                      1_996_000_000 + mid * 10_000))
            body=struct.pack('<H',codec.PARAMETERS['voltage'][0])+b'\x00\x00'+struct.pack('<f',39.)
            refresh.append(native_record(codec.read_request(mid,'voltage'),
                wire((17<<24)|(mid<<8)|codec.HOST_ID,body),1_998_000_000+mid*10_000))
        for phase, records in (('watchdog_setup', setup), ('watchdog_pre_enable_readback', final),
                               ('pre_enable_pose', pose),('voltage_pre_enable_refresh',refresh)):
            additions.append({'bus':bus, 'phase':phase, 'error':None, 'rejected_total':0,
                              'rejected_hex':'', 'records':records})
    initial = []
    for batch in result['journal']:
        if batch['phase'] == 'preflight':
            rows = [row for row in batch['records'] if
                    parameter_name(codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0]) == 'can_timeout']
            batch['records'] = [row for row in batch['records'] if row not in rows and
                                codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].kind != 18]
            initial.append({'bus':batch['bus'], 'phase':'watchdog_initial_readback', 'error':None,
                            'rejected_total':0, 'rejected_hex':'', 'records':rows})
    result['journal'] = ([batch for batch in additions if batch['phase']=='watchdog_setup'] + initial +
                         [batch for batch in additions if batch['phase']!='watchdog_setup'] + result['journal'])
    indexes = dict.fromkeys(runtime.BUSES, 0)
    for batch in result['journal']:
        if batch['phase'] != 'feedback_hold':
            continue
        bus = batch['bus']; wanted = runtime.BUSES[bus][indexes[bus] % 6]
        indexes[bus] += 1
        batch['records'] = [row for row in batch['records'] if
            codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].kind == 1 or
            codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].destination == wanted]
    refresh_by_id={str(codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0].destination):
                   {'value_v':39.,'received_ns':row['received_ns']}
                   for batch in result['journal'] if batch['phase']=='voltage_pre_enable_refresh'
                   for row in batch['records']}
    latest_by_id=dict(refresh_by_id)
    for batch in result['journal']:
        if batch['phase']!='feedback_hold':continue
        for row in batch['records']:
            frame=codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0]
            if parameter_name(frame)=='voltage':
                latest_by_id[str(frame.destination)]={'value_v':39.,'received_ns':row['received_ns']}
    result['voltage_guard']={'maximum_age_ms':126.,'pre_enable_refresh_by_id':refresh_by_id,
        'latest_by_id':latest_by_id,'checks_before_type1':13+2*len(result['cycles']),
        'maximum_checked_age_ms':100.,'minimum_checked_voltage_v':39.}
    return profile_data, result


class CadenceGroundReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.profile, self.runtime = v3_ground_fixture(Path(self.temp.name))

    def test_all_setup_reads_precede_pose_and_output_with_exact_rotating_voltage(self):
        feedback, voltages = ground._journal(self.runtime, self.profile)
        self.assertTrue(feedback)
        self.assertEqual(len(voltages), 24 + 2*111)

    def test_missing_or_invalid_watchdog_setup_ack_rejects(self):
        for failure in ('missing','send_only','partial','wrong_source','active','fault','version'):
            with self.subTest(failure=failure):
                value=copy.deepcopy(self.runtime)
                batch=next(b for b in value['journal'] if b['phase']=='watchdog_setup')
                row=batch['records'][0]
                frame=codec.ATParser().feed(bytes.fromhex(row['rx_hex']))[0]
                if failure=='missing': batch['records'].pop(0)
                elif failure=='send_only': row.update(rx_hex='',received=0,read_start_ns=0,received_ns=0)
                elif failure=='partial': row.update(rx_hex=row['rx_hex'][:-2],received=16)
                elif failure=='version':
                    row['rx_hex']=wire(frame.can_id,bytes.fromhex('00c45605001300a5')).hex()
                else:
                    can_id=(2<<24)|((2 if failure=='active' else 0)<<22)|((1 if failure=='fault' else 0)<<16)
                    can_id|=((2 if failure=='wrong_source' else 1)<<8)|codec.HOST_ID
                    row['rx_hex']=wire(can_id,frame.data).hex()
                with self.assertRaises(ValueError):
                    ground._journal(value,self.profile)

    def test_initial_watchdog_readback_waits_for_complete_setup_ack(self):
        for delta in (-1,0):
            with self.subTest(readback_minus_ack_ns=delta):
                value=copy.deepcopy(self.runtime)
                setup=max((row for batch in value['journal'] if batch['phase']=='watchdog_setup'
                           for row in batch['records']),key=lambda row:row['received_ns'])
                mid=codec.ATParser().feed(bytes.fromhex(setup['tx_hex']))[0].destination
                readback=next(row for batch in value['journal'] if batch['phase']=='watchdog_initial_readback'
                              for row in batch['records'] if row['tx_hex']==codec.read_request(mid,'can_timeout').hex())
                shift=setup['received_ns']+delta-readback['start_ns']
                for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
                    readback[key]+=shift
                self.assertGreater(readback['start_ns'],setup['finish_ns'])
                with self.assertRaises(ground.EvidenceError):
                    ground._journal(value,self.profile)

    def test_missing_axis_recheck_or_read_after_enable_rejects(self):
        for bad in ('missing', 'late'):
            value = copy.deepcopy(self.runtime)
            batch = next(b for b in value['journal'] if b['phase'] == 'watchdog_pre_enable_readback')
            if bad == 'missing':
                batch['records'].pop()
            else:
                row = batch['records'][0]
                for key in ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns'):
                    row[key] += 30_000_000
            with self.subTest(bad=bad), self.assertRaises(ground.EvidenceError):
                ground._journal(value, self.profile)

    def test_forged_cadence_or_source_binding_rejects(self):
        for key in ('telemetry_cadence', 'cadence_source_sha256'):
            value = copy.deepcopy(self.runtime); value[key] = {}
            with self.subTest(key=key), self.assertRaises(ground.EvidenceError):
                ground._journal(value, self.profile)

    def test_unreviewed_cyclic_timeout_query_is_rejected(self):
        value = copy.deepcopy(self.runtime)
        batch = next(b for b in value['journal'] if b['phase'] == 'feedback_hold')
        source = next(b for b in value['journal'] if b['phase'] == 'watchdog_pre_enable_readback')
        batch['records'].append(copy.deepcopy(source['records'][0]))
        with self.assertRaisesRegex(ground.EvidenceError, 'unexpected timeout-parameter readback phase'):
            ground._journal(value, self.profile)

    def test_voltage_rotation_cannot_omit_an_axis_or_add_requests(self):
        value = copy.deepcopy(self.runtime)
        batch = next(b for b in value['journal'] if b['phase'] == 'feedback_hold')
        batch['records'].pop()
        with self.assertRaisesRegex(ground.EvidenceError, 'six Type1 feedback replies plus one rotating voltage'):
            ground._journal(value, self.profile)

    def test_pre_enable_voltage_refresh_missing_or_out_of_order_rejects(self):
        for failure in ('missing_axis','before_imu','after_enable','wrong_parameter'):
            value=copy.deepcopy(self.runtime)
            batch=next(b for b in value['journal'] if b['phase']=='voltage_pre_enable_refresh')
            if failure=='missing_axis':batch['records'].pop()
            elif failure in ('before_imu','after_enable'):
                delta=-3_000_000 if failure=='before_imu' else 10_000_000
                for part in value['journal']:
                    if part['phase']=='voltage_pre_enable_refresh':
                        for row in part['records']:
                            for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
                                row[key]+=delta
            else:
                row=batch['records'][0];frame=codec.ATParser().feed(bytes.fromhex(row['tx_hex']))[0]
                row['tx_hex']=codec.read_request(frame.destination,'position').hex()
                body=struct.pack('<H',codec.PARAMETERS['position'][0])+b'\x00\x00'+struct.pack('<f',0.)
                row['rx_hex']=wire((17<<24)|(frame.destination<<8)|codec.HOST_ID,body).hex()
            with self.subTest(failure=failure),self.assertRaises(ground.EvidenceError):
                ground._journal(value,self.profile)

    def test_pre_enable_guard_and_latest_values_must_match_raw_can(self):
        for failure in ('refresh_value','refresh_time','latest_time','check_count','minimum_value'):
            value=copy.deepcopy(self.runtime);guard=value['voltage_guard']
            if failure=='refresh_value':guard['pre_enable_refresh_by_id']['1']['value_v']=40.
            elif failure=='refresh_time':guard['pre_enable_refresh_by_id']['1']['received_ns']+=1
            elif failure=='latest_time':guard['latest_by_id']['1']['received_ns']+=1
            elif failure=='check_count':guard['checks_before_type1']-=1
            else:guard['minimum_checked_voltage_v']=40.
            with self.subTest(failure=failure),self.assertRaises(ground.EvidenceError):
                ground._journal(value,self.profile)

    def test_type1_cannot_use_a_voltage_older_than_126ms(self):
        value=copy.deepcopy(self.runtime)
        batch=next(b for b in value['journal'] if b['phase']=='policy_output')
        row=batch['records'][0]
        for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns'):
            row[key]+=3_000_000_000
        with self.assertRaisesRegex(ground.EvidenceError,'stale all-axis voltage'):
            ground._journal(value,self.profile)


if __name__ == '__main__':
    unittest.main()
