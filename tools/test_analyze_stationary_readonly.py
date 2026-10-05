"""Synthetic fixed50 raw receipts; no private identifiers or hardware."""

from contextlib import redirect_stdout
import copy
import hashlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from tools import analyze_stationary_readonly as analysis


B = 1_000_000_000
BOOT = '00000000-0000-4000-8000-000000000001'
EXPECTED = {str(i): f'{i:016x}' for i in range(1, 13)}
UID_BYTES = json.dumps(EXPECTED, sort_keys=True).encode()
UID_SHA = hashlib.sha256(UID_BYTES).hexdigest()


def fixture(values=None):
    values = values or (lambda mid, sweep, parameter: {
        'position': mid/10, 'velocity': .06 if sweep % 2 else -.06,
        'current': 0., 'run_mode': 0, 'voltage': 40.}[parameter])
    report = dict(schema=analysis.SCHEMA, status='COMPLETE_STATIONARY_READONLY_CAPTURE',
                  duration_s=10, sweeps=50, period_ms=200, parameters=list(analysis.PARAMETERS),
                  allowed_can_types=[0, 17], requests_per_bus=1506, catchup_available=False,
                  max_reply_timeout_ms=250, stop_state='UNVERIFIED_BY_READ_ONLY_PROTOCOL',
                  boot_id=BOOT, expected_boot_id=BOOT, expected_uids_sha256=UID_SHA,
                  source_sha256={name: 'a'*64 for name in analysis.SOURCE_NAMES},
                  errors=[], queries_sent=3012, buses={}, **dict.fromkeys(analysis.FALSE_FLAGS, False))
    for n, (bus, ids) in enumerate(analysis.BUSES.items(), 1):
        data = dict(errors=[], identities={}, sweeps=[], events=[], owner_thread_id=n,
                    capture_begin_ns=B, capture_end_ns=B+analysis.DURATION_NS,
                    queries_sent=1506, rx_bytes=1506*17, parser_discarded_bytes=0,
                    parser_residual_hex='')
        report['buses'][bus] = data
        def query(mid, parameter, sequence, start, sweep=None):
            if parameter == 'identity':
                can_id, payload = mid << 8 | 0xfe, bytes.fromhex(EXPECTED[str(mid)])
            else:
                index, fmt, _ = analysis.codec.PARAMETERS[parameter]
                packed = struct.pack('<'+fmt, values(mid, sweep, parameter))
                payload = struct.pack('<H', index)+bytes(2)+packed+bytes(4-len(packed))
                can_id = 17 << 24 | mid << 8 | 0xfd
            wire = b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+b'\x08'+payload+b'\r\n'
            frame = analysis.codec.ATParser().feed(wire)[0]
            result = analysis.codec.decode_reply(frame, mid, None if parameter == 'identity' else parameter)
            result.update(kind='motor_parameter', sequence=sequence, request_monotonic_ns=start,
                          monotonic_ns=start+1_000_000, round_trip_ms=1.)
            data['events'].extend([
                dict(kind='can_tx', sequence=sequence, motor_id=mid, parameter=parameter,
                     hex=analysis.codec.read_request(mid, None if parameter == 'identity' else parameter).hex(),
                     monotonic_ns=start),
                dict(kind='can_rx_bytes', hex=wire.hex(), monotonic_ns=start+999_999),
                dict(kind='can_rx_frame', **frame.record(), monotonic_ns=start+999_999), result])
            return result
        for k, mid in enumerate(ids):
            data['identities'][str(mid)] = query(mid, 'identity', k+1, B-20_000_000+k*2_000_000)
        for sweep in range(50):
            begin = B+sweep*analysis.PERIOD_NS
            row = dict(index=sweep, requested_release_ns=begin, begin_ns=begin, end_ns=begin+59_000_000,
                       complete=True, samples={})
            for k, mid in enumerate(ids):
                row['samples'][str(mid)] = {}
                for p, parameter in enumerate(analysis.PARAMETERS):
                    offset = k*5+p
                    row['samples'][str(mid)][parameter] = query(mid, parameter, 7+sweep*30+offset,
                                                               begin+offset*2_000_000, sweep)
            data['sweeps'].append(row)
    return report


def analyze(report):
    return analysis.analyze(report, EXPECTED, UID_SHA, BOOT)


def partial(report, count=10):
    report['status'], report['errors'] = 'ABORTED_READONLY_CAPTURE', ['PRIVATE ORIGINAL FAILURE']
    for data in report['buses'].values():
        last_sequence = 6+30*count
        data['sweeps'] = data['sweeps'][:count]
        data['events'] = data['events'][:last_sequence*4]
        data.pop('capture_end_ns')
        data.update(queries_sent=last_sequence, rx_bytes=last_sequence*17, errors=['PRIVATE BUS FAILURE'])
    report['queries_sent'] = 2*last_sequence
    return report


class StationaryFileAnalysisTests(unittest.TestCase):
    def test_full_raw_replay_retains_all_points_and_grants_no_approval(self):
        result = analyze(fixture())
        self.assertEqual(result['status'], 'COMPLETE_FILE_DIAGNOSTIC')
        self.assertTrue(result['capture_integrity_verified'])
        self.assertEqual(result['queries_replayed'], 3012)
        self.assertEqual(set(result['per_motor']), set(EXPECTED))
        self.assertTrue(result['external_boot_binding_verified'])
        for row in result['per_motor'].values():
            self.assertEqual(len(row['velocity_samples']), 50)
            self.assertEqual(len(row['position_samples']), 50)
            self.assertEqual(row['missing_parameter_sweep_indices']['velocity'], [])
            self.assertEqual(row['position_span_rad'], 0)
            self.assertEqual(row['position_OLS_slope_rad_s_host_midpoints'], 0)
            self.assertEqual(row['statistics']['velocity']['mean'], 0)
            self.assertAlmostEqual(row['statistics']['velocity']['rms'], .06, places=8)
            self.assertEqual(len(row['velocity_absolute_local_peaks']), 50)  # ties retained
        for flag, value in analysis.SCOPE_FLAGS.items():
            self.assertIs(value, False)
            self.assertIs(result[flag], False)
        self.assertNotIn('passed', result)

    def test_position_creep_is_measured_with_host_times_without_velocity_replacement(self):
        def values(mid, sweep, parameter):
            return {'position': .003*(sweep*.2), 'velocity': -.06 if sweep % 2 else .06,
                    'current': 0., 'run_mode': 0, 'voltage': 40.}[parameter]
        result = analyze(fixture(values)); row = result['per_motor']['1']
        self.assertAlmostEqual(row['position_OLS_slope_rad_s_host_midpoints'], .003, places=9)
        self.assertAlmostEqual(row['position_span_rad'], .0294, places=8)
        self.assertAlmostEqual(row['statistics']['velocity']['rms'], .06, places=8)
        self.assertFalse(result['physical_stationarity_proven'])

    def test_large_isolated_peaks_are_not_trimmed_and_all_peak_times_are_retained(self):
        def values(mid, sweep, parameter):
            return {'position': 0., 'velocity': .3 if sweep in (7, 19) else .01,
                    'current': 0., 'run_mode': 0, 'voltage': 40.}[parameter]
        result = analyze(fixture(values)); row = result['per_motor']['2']
        self.assertEqual([p['sweep_index'] for p in row['velocity_absolute_maximum_samples']], [7, 19])
        self.assertTrue({7, 19}.issubset(p['sweep_index'] for p in row['velocity_absolute_local_peaks']))
        self.assertEqual(row['statistics']['velocity']['samples'], 50)
        self.assertAlmostEqual(row['statistics']['velocity']['rms'], ((2*.3**2+48*.01**2)/50)**.5, places=8)
        peak = row['velocity_absolute_maximum_samples'][0]
        self.assertEqual(peak['host_midpoint_ns'], (peak['request_monotonic_ns']+peak['reply_monotonic_ns'])//2)
        self.assertEqual(row['position_velocity_read_separation_ns'][0]['velocity_minus_position_host_midpoint_ns'], 2_000_000)

    def test_partial_capture_preserves_every_present_sample_and_exposes_missing_window(self):
        result = analyze(partial(fixture())); row = result['per_motor']['1']
        self.assertEqual(result['status'], 'INCOMPLETE_FILE_DIAGNOSTIC')
        self.assertFalse(result['capture_integrity_verified'])
        self.assertEqual(row['statistics']['velocity']['samples'], 10)
        self.assertEqual(row['missing_parameter_sweep_indices']['velocity'], list(range(10, 50)))
        self.assertEqual(result['buses']['front']['missing_sweep_indices'], list(range(10, 50)))
        self.assertNotIn('PRIVATE', json.dumps(result))

    def test_missing_saved_cell_does_not_hide_a_successful_raw_velocity_receipt(self):
        report = fixture(); del report['buses']['front']['sweeps'][7]['samples']['1']['velocity']
        result = analyze(report)
        self.assertFalse(result['capture_integrity_verified'])
        self.assertEqual(result['per_motor']['1']['statistics']['velocity']['samples'], 50)
        self.assertIn(7+7*30+1, result['buses']['front']['missing_saved_parameter_sequences'])

    def test_missing_bus_retains_all_twelve_ids_and_explicit_missing_points(self):
        report = fixture(); del report['buses']['rear']; result = analyze(report)
        self.assertFalse(result['capture_integrity_verified'])
        self.assertEqual(set(result['per_motor']), set(EXPECTED))
        self.assertIsNone(result['per_motor']['12']['statistics']['velocity'])
        self.assertEqual(result['per_motor']['12']['missing_parameter_sweep_indices']['velocity'], list(range(50)))

    def test_current_and_run_mode_nonzero_values_are_not_stop_or_motion_approval(self):
        def values(mid, sweep, parameter):
            return {'position': 0., 'velocity': 0., 'current': .1 if sweep == 8 else 0.,
                    'run_mode': 1 if sweep == 9 else 0, 'voltage': 40.}[parameter]
        result = analyze(fixture(values)); row = result['per_motor']['1']
        self.assertEqual(row['current_nonzero_samples'][0]['sweep_index'], 8)
        self.assertEqual(row['run_mode_nonzero_samples'][0]['sweep_index'], 9)
        self.assertFalse(row['observed_current_all_zero'] or row['observed_run_mode_all_zero'])
        self.assertFalse(result['capture_integrity_verified'] or result['stop_confirmed'])

    def test_schema_boot_uid_source_and_scope_mismatches_rejected(self):
        report = fixture()
        for name in ('schema', 'boot', 'external_boot', 'uid', 'uid_hash', 'source', 'approve', 'bool_plan'):
            with self.subTest(name=name):
                value = copy.deepcopy(report)
                if name == 'schema': value['schema'] += '.wrong'
                elif name == 'boot': value['expected_boot_id'] = '00000000-0000-4000-8000-000000000002'
                elif name == 'external_boot': value['boot_id'] = value['expected_boot_id'] = '00000000-0000-4000-8000-000000000002'
                elif name == 'uid': value['buses']['front']['identities']['1']['mcu_uid_hex'] = 'f'*16
                elif name == 'uid_hash': value['expected_uids_sha256'] = 'b'*64
                elif name == 'source': value['source_sha256'].pop('can_readonly.py')
                elif name == 'approve': value['approved_for_runtime'] = True
                else: value['duration_s'] = True
                with self.assertRaises(ValueError): analyze(value)

    def test_raw_value_wire_rtt_and_receive_frame_tampering_rejected(self):
        report = fixture()
        for name in ('value', 'tx', 'rx', 'frame', 'rtt', 'receipt_time'):
            with self.subTest(name=name):
                value = copy.deepcopy(report); data = value['buses']['front']
                if name == 'value': data['sweeps'][0]['samples']['1']['position']['value'] += .1
                elif name == 'tx': data['events'][24]['hex'] = '00'*17
                elif name == 'rx': data['events'][25]['hex'] = data['events'][25]['hex'].replace('ffff', 'fffe') if 'ffff' in data['events'][25]['hex'] else '00'*17
                elif name == 'frame': data['events'][26]['data_hex'] = '00'*8
                elif name == 'rtt': data['events'][27]['round_trip_ms'] = .5
                else: data['events'][27]['request_monotonic_ns'] -= 1
                with self.assertRaises(ValueError): analyze(value)

    def test_sequence_or_schedule_errors_rejected_and_counter_errors_remain_incomplete(self):
        report = fixture()
        for name in ('sequence', 'duplicate_sweep', 'catchup', 'epoch', 'receipt_outside_sweep'):
            with self.subTest(name=name):
                value = copy.deepcopy(report); data = value['buses']['front']
                if name == 'sequence': data['events'][24]['sequence'] = 8
                elif name == 'duplicate_sweep': data['sweeps'][1]['index'] = 0
                elif name == 'catchup': data['sweeps'][1]['requested_release_ns'] -= 1
                elif name == 'epoch': data['capture_begin_ns'] += 1
                else: data['sweeps'][0]['end_ns'] = B+500_000
                with self.assertRaises(ValueError): analyze(value)
        report['buses']['front']['rx_bytes'] -= 1
        result = analyze(report)
        self.assertFalse(result['capture_integrity_verified'])
        self.assertIn('front:reported_rx_bytes_mismatch_or_missing', result['validation_errors'])

    def test_gap_indices_are_exposed_without_reselecting_a_passing_window(self):
        report = fixture(); del report['buses']['front']['sweeps'][17]
        result = analyze(report)
        self.assertEqual(result['buses']['front']['missing_sweep_indices'], [17])
        self.assertEqual(len(result['per_motor']['1']['position_samples']), 50)  # all raw points retained
        self.assertFalse(result['capture_integrity_verified'])

    def test_fragmented_receive_bytes_and_unrequested_type2_never_create_stop_proof(self):
        report = fixture(); data = report['buses']['front']
        event = data['events'][1]
        data['events'][1:2] = [dict(event, hex=event['hex'][:2], monotonic_ns=event['monotonic_ns']-1),
                              dict(event, hex=event['hex'][2:])]
        can_id = 2 << 24 | 1 << 8 | 0xfd
        raw = b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+b'\x08'+bytes(8)+b'\r\n'
        frame = analysis.codec.ATParser().feed(raw)[0]
        when = B-21_000_000
        data['events'][:0] = [dict(kind='can_rx_bytes', hex=raw.hex(), monotonic_ns=when),
                              dict(kind='can_rx_frame', monotonic_ns=when, **frame.record()),
                              dict(kind='motor_feedback', monotonic_ns=when, **analysis.codec.feedback_metadata(frame))]
        data['rx_bytes'] += 17
        result = analyze(report)
        self.assertTrue(result['capture_integrity_verified'])
        self.assertEqual(result['buses']['front']['unsolicited_feedback_count'], 1)
        self.assertFalse(result['stop_confirmed'] or result['type2_comparison_verified'])

    def test_timeout_partial_receive_boundary_is_recorded_without_retry(self):
        report = partial(fixture()); data = report['buses']['front']
        sequence = data['queries_sent']+1; when = B+10*analysis.PERIOD_NS
        data['events'].extend([
            dict(kind='can_tx', sequence=sequence, motor_id=1, parameter='position',
                 hex=analysis.codec.read_request(1, 'position').hex(), monotonic_ns=when),
            dict(kind='can_rx_bytes', hex='41', monotonic_ns=when+1),
            dict(kind='can_timeout', motor_id=1, parameter='position', monotonic_ns=when+250_000_000)])
        data.update(queries_sent=sequence, rx_bytes=data['rx_bytes']+1, parser_residual_hex='41')
        report['queries_sent'] += 1
        result = analyze(report); bus = result['buses']['front']
        self.assertEqual(bus['timeout_count'], 1)
        self.assertEqual(bus['parser_residual_hex'], '41')
        self.assertIn('request_without_parameter_receipt', bus['issues'])
        self.assertFalse(result['capture_integrity_verified'] or result['automatic_retry'])

    def test_raw_enabled_fault_feedback_cannot_be_hidden_by_missing_metadata(self):
        for type_id, mode, fault in ((2, 1, 0), (2, 0, 1), (21, 0, 0), (2, 0, 0)):
            with self.subTest(type_id=type_id, mode=mode, fault=fault):
                report = fixture(); data = report['buses']['front']
                can_id = type_id << 24 | mode << 22 | fault << 16 | 1 << 8 | 0xfd
                raw = b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+b'\x08'+bytes(8)+b'\r\n'
                frame = analysis.codec.ATParser().feed(raw)[0]; when = B-21_000_000
                data['events'][:0] = [dict(kind='can_rx_bytes', hex=raw.hex(), monotonic_ns=when),
                                      dict(kind='can_rx_frame', monotonic_ns=when, **frame.record())]
                data['rx_bytes'] += 17
                result = analyze(report); bus = result['buses']['front']
                self.assertEqual(bus['unsolicited_feedback_count'], 1)
                self.assertEqual(bus['missing_feedback_metadata_count'], 1)
                self.assertIn('missing_feedback_metadata_events', bus['issues'])
                if type_id == 21 or mode or fault:
                    self.assertIn('unsolicited_enabled_or_fault_feedback', bus['issues'])
                self.assertFalse(result['capture_integrity_verified'] or result['stop_confirmed'])

    def test_shared_owner_or_different_common_epoch_cannot_be_complete(self):
        report = fixture(); report['buses']['rear']['owner_thread_id'] = report['buses']['front']['owner_thread_id']
        result = analyze(report)
        self.assertIn('missing_or_shared_bus_owner', result['validation_errors'])
        self.assertFalse(result['capture_integrity_verified'])

    def test_file_loader_rejects_duplicate_nonfinite_symlink_directory_and_wrong_sha(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'data.json'
            for raw in (b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":1e999}'):
                path.write_bytes(raw)
                with self.assertRaises(ValueError): analysis.load_json(path)
            path.write_text('{}')
            with self.assertRaises(ValueError): analysis.load_json(path, '0'*64)
            link = Path(temp)/'link.json'; link.symlink_to(path)
            with self.assertRaises(OSError): analysis.load_json(link)
            with self.assertRaises((ValueError, OSError)): analysis.load_json(Path(temp))

    def test_cli_exit_codes_are_integrity_only_and_no_device_is_opened(self):
        with tempfile.TemporaryDirectory() as temp, \
                patch.object(analysis.codec, 'ReadOnlyCAN', side_effect=AssertionError('No hardware')):
            path, uid_path = Path(temp)/'capture.json', Path(temp)/'uids.json'
            uid_path.write_bytes(UID_BYTES)
            for report, expected_exit in ((fixture(), 0), (partial(fixture()), 1), ({'schema':'wrong'}, 2)):
                path.write_text(json.dumps(report))
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                with redirect_stdout(io.StringIO()) as output:
                    code = analysis.main(['--report', str(path), '--expected-sha256', digest,
                                          '--expected-uids', str(uid_path), '--expected-boot-id', BOOT])
                self.assertEqual(code, expected_exit)
                result = json.loads(output.getvalue())
                self.assertFalse(result['automatic_approval'] or result['motor_output_allowed'])
                if code != 2: self.assertEqual(result['source_report_sha256'], digest)


if __name__ == '__main__':
    unittest.main()
