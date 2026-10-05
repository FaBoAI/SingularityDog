"""Synthetic file-only probes. No hardware, network or private live fixtures."""
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest

import analyze_stationary_velocity_probe as a
from singularitydog_hw import stationary_velocity_probe as probe

BOOT = '11111111-2222-4333-8444-555555555555'
UIDS = {i: f'{i:016x}' for i in range(1, 13)}


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def wait(self, seconds): self.now += round(seconds*1e9)


class Serial:
    def __init__(self, clock, delay, fail):
        self.clock, self.delay, self.fail = clock, delay, fail
        self.buffer, self.parameters = bytearray(), 0
    @property
    def in_waiting(self): return len(self.buffer)
    def close(self): pass
    def write(self, raw):
        frame = probe.codec.ATParser().feed(raw)[0]; mid = frame.destination
        if frame.kind == 0:
            can_id, data = (mid<<8)|0xFE, bytes.fromhex(UIDS[mid])
        else:
            self.parameters += 1
            if self.fail == self.parameters: return len(raw)
            index = int.from_bytes(frame.data[:2], 'little')
            value = ((-1 if self.parameters % 2 else 1)*.08+self.parameters*.00001) if index == 0x701B else 2.5+(self.parameters % 4)*.00005
            can_id, data = (17<<24)|(mid<<8)|0xFD, struct.pack('<HHf', index, 0, value)
        self.buffer.extend(b'AT'+((can_id<<3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n')
        return len(raw)
    def read(self, count):
        self.clock.now += self.delay
        raw = bytes(self.buffer[:count]); del self.buffer[:count]; return raw


class SavedFixtures:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.uid_path, self.source_path = self.directory/'uids.json', self.directory/'source.json'
        self.uid_path.write_text(json.dumps({str(k): v for k, v in UIDS.items()}))
        self.source_path.write_text(json.dumps(probe.source_hashes()))
        self.runs = []
    def create(self, period, count, delay=2_000_000, fail=None, reserve_abort=False):
        clock, events, plan = Clock(), [], probe.make_plan(5, count, period)
        def emit(bus, event):
            events.append({'bus': bus, **event})
            if reserve_abort and bus == 'front' and event['kind'] == 'can_tx' and event['sequence'] == 10:
                clock.now += (count-1)*period*1_000_000-40_000_000
        cans = {b: probe.ProbeCAN(b, plan, lambda e, b=b: emit(b, e),
                                 clock=clock, serial_port=Serial(clock, delay, fail)) for b in probe.IDS}
        with cans['front'], cans['rear']:
            captured = probe.collect(cans, UIDS, plan, clock=clock, wait=clock.wait)
        paths = {k: self.directory/f'{period}-{k}.json' for k in ('report', 'receipt', 'trace')}
        report = {'schema': probe.SCHEMA, 'status': 'INCOMPLETE', **probe.FLAGS, 'plan': plan, 'errors': [],
            'samples': [], 'hardware_opened': True, 'hardware_open_attempted': True,
            'expected_boot_id': BOOT, 'boot_id': BOOT,
            'expected_uids_sha256': hashlib.sha256(self.uid_path.read_bytes()).hexdigest(),
            'source_sha256': probe.source_hashes(), 'started_at': '2026-01-01T00:00:00+00:00',
            'completed_at': '2026-01-01T00:00:10+00:00',
            'motor_power_epoch': 'NOT_INFERRED_FROM_JETSON_BOOT', 'stop_state': 'UNVERIFIED_BY_READ_ONLY_PROTOCOL',
            'all_device_contexts_closed': True, 'resource_leases_retained_until_process_exit': False,
            'report_finalization_claimed': False,
            'persistence_scope': 'This report cannot certify its own finalization. Require the separate final CLI receipt with matching report SHA256.'}
        report.update(captured)
        report['data_collection_status'] = report['status']
        report['status'] = 'RECORDED_REVIEW_REQUIRED' if report['status'] == 'COMPLETE_READONLY_PROBE_CAPTURE' else 'INCOMPLETE'
        receipt = {'status': 'SAVED_READONLY_PROBE_REVIEW_REQUIRED' if not report['errors'] else 'INCOMPLETE',
            'output': str(paths['report']), 'report_sha256': None, 'errors': list(report['errors']), **probe.FLAGS,
            **{k: report[k] for k in ('requested_slots', 'complete_triplets', 'dropped_slot_count', 'incomplete_triplets',
                                     'unacquired_slot_count', 'full_requested_slot_coverage')}}
        item = {'paths': paths, 'report': report, 'receipt': receipt, 'events': events}
        self.runs.append(item); self.save(item); return item
    def save(self, item):
        trace = b''.join((json.dumps(e, separators=(',', ':'))+'\n').encode() for e in item['events'])
        item['paths']['trace'].write_bytes(trace)
        item['report']['trace_events'] = {'path': str(item['paths']['trace']), 'sha256': hashlib.sha256(trace).hexdigest(),
            'event_count': len(item['events']), 'attempted_event_count': len(item['events']), 'byte_count': len(trace),
            'complete': True, 'errors': [], 'status': 'COMPLETE_EVENT_TRACE'}
        raw = (json.dumps(item['report'], indent=2)+'\n').encode(); item['paths']['report'].write_bytes(raw)
        if item['receipt']['status'] == 'SAVED_READONLY_PROBE_REVIEW_REQUIRED':
            item['receipt']['report_sha256'] = hashlib.sha256(raw).hexdigest()
        item['paths']['receipt'].write_text(json.dumps(item['receipt']))
    def compare(self):
        return a.compare([r['paths'] for r in self.runs], expected_uids=self.uid_path, expected_sources=self.source_path,
                         expected_boot_id=BOOT, motor_id=5)


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.f = SavedFixtures(self.temp.name)
    def pair(self, count20=50, count200=5, **kwargs):
        return self.f.create(20, count20, **kwargs), self.f.create(200, count200)
    def test_replay_complete_recording_keeps_requested_drop_denominator_and_all_phases(self):
        fast, slow = self.pair(); result = self.f.compare()
        self.assertEqual(result['status'], 'DESCRIPTIVE_COMPARISON_REVIEW_REQUIRED')
        self.assertEqual(result['runs']['20']['complete_triplets'], 48)
        self.assertEqual(result['runs']['200']['complete_triplets'], 5)
        self.assertTrue(result['runs']['20']['recording_complete'])
        self.assertFalse(result['runs']['20']['data_coverage'])
        self.assertFalse(result['all_requested_slot_coverage'])
        self.assertEqual(result['runs']['20']['unacquired_slots'], [48, 49])
        phases = result['twenty_ms_all_ten_phase_cohorts']
        self.assertEqual(len(phases), 10)
        self.assertEqual(sum(p['requested_slot_count'] for p in phases), 50)
        self.assertFalse(phases[8]['data_coverage']); self.assertEqual(phases[8]['dropped_slot_indices'], [48])
        self.assertTrue(all(result[k] is False for k in probe.FLAGS))
        self.assertFalse(result['velocity_ground_truth'] or result['hardware_opened'])
        self.assertEqual(result['runs']['20']['trace_counts']['can_tx'], 12+3*48)
        self.assertEqual(result['runs']['20']['raw_velocity']['unique_raw_words'], 48)
        self.assertGreater(result['runs']['20']['raw_position']['minimum_positive_observed_value_spacing'], 0)
    def test_all500_and50_bounded_slots_and_count_aware_window(self):
        self.pair(500, 50); result = self.f.compare()
        self.assertEqual(result['common_nominal_elapsed_window_ns'], 10_000_000_000)
        self.assertEqual(result['runs']['20']['complete_triplets'], 498)
        self.assertEqual(sum(p['requested_slot_count'] for p in result['twenty_ms_all_ten_phase_cohorts']), 500)
        self.assertTrue(result['runs']['200']['data_coverage'])
        self.assertEqual(result['runs']['200']['raw_velocity']['count'], 50)
    def test_empty_phases_and_no_complete_samples_never_success(self):
        self.pair(1, 1); result = self.f.compare()
        self.assertEqual(result['status'], 'INCOMPLETE_COMPARISON_RECORDS')
        self.assertIsNone(result['runs']['20']['reported_velocity_rad_s'])
        for phase in result['twenty_ms_all_ten_phase_cohorts'][1:]:
            self.assertEqual(phase['requested_slot_count'], 0)
            self.assertFalse(phase['has_complete_measurements'] or phase['data_coverage'])
    def test_partial_timeout_keeps_all_trace_and_excludes_partial_velocity_from_complete_stats(self):
        fast, _ = self.pair(fail=3); result = self.f.compare(); run = result['runs']['20']
        self.assertEqual(result['status'], 'INCOMPLETE_COMPARISON_RECORDS')
        self.assertEqual(run['complete_triplets'], 0); self.assertEqual(run['incomplete_triplets'], 1)
        self.assertIsNone(run['reported_velocity_rad_s'])
        self.assertEqual(run['all_valid_velocity_receipts_including_partial_triplets']['count'], 1)
        self.assertEqual(run['trace_counts']['can_timeout'], 1)
        self.assertEqual(len(run['requests_without_receipts']), 1)
        self.assertFalse(run['final_report_binding_verified'])
    def test_period_misses_and_long_reply_gaps_all_reported(self):
        self.pair(count20=10, delay=9_000_000); result = self.f.compare(); run = result['runs']['20']
        self.assertTrue(run['period_deadline_missed_slots'])
        self.assertGreater(run['velocity_reply_gap_ms']['minimum'], 20)
        self.assertTrue(run['dropped_slots'])
        self.assertEqual(len(run['all_reply_timing_rows']), run['trace_counts']['motor_parameter'])
    def test_unwritten_request_intent_is_reported_separately_from_physical_write(self):
        self.pair(count20=10, reserve_abort=True); result = self.f.compare(); run = result['runs']['20']
        self.assertEqual(result['status'], 'DESCRIPTIVE_COMPARISON_REVIEW_REQUIRED')
        self.assertEqual(run['complete_triplets'], 1)
        self.assertEqual(run['request_intents_without_physical_write'], 1)
        self.assertEqual(run['trace_counts']['can_tx'], 16)
        self.assertEqual(run['trace_counts']['probe_write_timing'], 15)
        self.assertFalse(run['requests_without_receipts'][0]['physical_write_recorded'])
        self.assertEqual(run['unacquired_slots'], list(range(1, 10)))
    def test_report_close_failure_cannot_self_certify_with_saved_complete_json(self):
        fast, _ = self.pair()
        fast['receipt'].update(status='INCOMPLETE', report_sha256=None, errors=['Synthetic report close failure'])
        self.f.save(fast); result = self.f.compare()
        self.assertTrue(result['runs']['20']['recording_complete'])
        self.assertFalse(result['runs']['20']['final_report_binding_verified'])
        self.assertEqual(result['status'], 'INCOMPLETE_COMPARISON_RECORDS')
    def test_integrity_and_schema_mutations_rejected_even_when_report_rehashed(self):
        mutations = {
            'coverage': lambda x: x['report'].__setitem__('full_requested_slot_coverage', True),
            'periodmiss': lambda x: x['report'].__setitem__('period_deadline_missed_slots', [0]),
            'uid': lambda x: x['report']['identities']['5'].__setitem__('mcu_uid_hex', 'a'*16),
            'rawdecoded': lambda x: x['report']['samples'][0]['velocity'].__setitem__('value', 9.),
            'chronology': lambda x: x['report']['samples'][0].__setitem__('begin_ns', x['report']['samples'][0]['end_ns']+1),
            'requesttiming': lambda x: x['report']['requests'][-1].__setitem__('received_monotonic_ns', 1),
            'flags': lambda x: x['receipt'].__setitem__('stop_sent', True),
            'unknown': lambda x: x['report'].__setitem__('unknown', False),
            'boolcount': lambda x: x['report'].__setitem__('complete_triplets', True),
            'replybytes': lambda x: x['events'][2].__setitem__('hex', x['events'][2]['hex'][:-2]+'00'),
            'nonreadonly': lambda x: x['events'][0].__setitem__('hex', '00'),
            'source': lambda x: x['report']['source_sha256'].__setitem__('can_readonly.py', 'a'*64),
            'boot': lambda x: x['report'].__setitem__('boot_id', '99999999-2222-4333-8444-555555555555'),
            'id': lambda x: x['report']['plan'].__setitem__('motor_id', 6),
            'collectionstatus': lambda x: x['report'].__setitem__('data_collection_status', 'ANYTHING'),
            'openstate': lambda x: x['report'].__setitem__('hardware_opened', False),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                f = SavedFixtures(directory); fast = f.create(20, 5); f.create(200, 1)
                mutate(fast); f.save(fast)
                with self.assertRaises((ValueError, KeyError, TypeError)): f.compare()
    def test_final_sha_trace_count_and_nonfinite_duplicate_json_rejected(self):
        fast, _ = self.pair()
        for kind, raw in [('report', b'{"x":1,"x":2}'), ('receipt', b'{"x":NaN}'), ('trace', b'{}')]:
            path = fast['paths'][kind]; saved = path.read_bytes(); path.write_bytes(raw)
            with self.assertRaises((ValueError, KeyError)): self.f.compare()
            path.write_bytes(saved)
        fast['receipt']['report_sha256'] = 'a'*64
        fast['paths']['receipt'].write_text(json.dumps(fast['receipt']))
        with self.assertRaises(ValueError): self.f.compare()
        self.f.save(fast); fast['report']['trace_events']['event_count'] += 1
        fast['paths']['report'].write_text(json.dumps(fast['report']))
        with self.assertRaises(ValueError): self.f.compare()
    def test_symlink_and_loaded_decoder_mismatch_refused(self):
        fast, _ = self.pair(); original = fast['paths']['report']
        link = self.f.directory/'link.json'; link.symlink_to(original); fast['paths']['report'] = link
        with self.assertRaises(ValueError): self.f.compare()
        fast['paths']['report'] = original
        source = json.loads(self.f.source_path.read_text()); source['can_readonly.py'] = 'a'*64
        self.f.source_path.write_text(json.dumps(source))
        with self.assertRaises(ValueError): self.f.compare()
    def test_period_labels_and_independent_expected_context_refused(self):
        self.pair()
        for override in ({'motor_id': 6}, {'expected_boot_id': '99999999-2222-4333-8444-555555555555'}):
            kwargs = dict(expected_uids=self.f.uid_path, expected_sources=self.f.source_path, expected_boot_id=BOOT, motor_id=5)
            kwargs.update(override)
            with self.assertRaises(ValueError): a.compare([r['paths'] for r in self.f.runs], **kwargs)
        with self.assertRaises(ValueError):
            a.compare([r['paths'] for r in reversed(self.f.runs)], expected_uids=self.f.uid_path,
                      expected_sources=self.f.source_path, expected_boot_id=BOOT, motor_id=5)
        changed = json.loads(self.f.uid_path.read_text()); changed['5'] = 'aaaaaaaaaaaaaaaa'
        self.f.uid_path.write_text(json.dumps(changed))
        with self.assertRaises(ValueError): self.f.compare()
    def test_failed_trace_finalization_remains_unverified_even_when_raw_replay_is_valid(self):
        fast, _ = self.pair()
        fast['receipt'].update(status='INCOMPLETE', report_sha256=None, errors=['Synthetic trace fsync failure'])
        self.f.save(fast)
        report = fast['report']; report['trace_events'].update(complete=False, sha256=None, status='INCOMPLETE_EVENT_TRACE', errors=['Synthetic trace fsync failure'])
        report.update(data_collection_status='INCOMPLETE', status='INCOMPLETE', errors=['Synthetic trace fsync failure'])
        fast['paths']['report'].write_text(json.dumps(report))
        result = self.f.compare(); self.assertEqual(result['status'], 'INCOMPLETE_COMPARISON_RECORDS')
        self.assertFalse(result['runs']['20']['trace_complete'])
        self.assertFalse(result['runs']['20']['trace_manifest_binding_verified'])
    def test_incomplete_timeout_overrun_is_described_without_success_or_cap_claim(self):
        fast, _ = self.pair(fail=3)
        end = fast['report']['segment_deadline_ns']+3_000_000
        fast['events'][-1]['monotonic_ns'] = end
        self.assertEqual(fast['events'][-1]['kind'], 'can_timeout')
        fast['report']['samples'][-1]['end_ns'] = end
        self.f.save(fast); result = self.f.compare()
        self.assertEqual(result['status'], 'INCOMPLETE_COMPARISON_RECORDS')
        self.assertEqual(result['runs']['20']['segment_overrun_ns'], 3_000_000)
        self.assertFalse(result['runs']['20']['final_report_binding_verified'])
    def test_precollection_empty_trace_has_all_slots_unavailable_and_unknown_boot(self):
        fast, _ = self.pair()
        report = {k: fast['report'][k] for k in a.STARTUP_FIELDS}
        report.update(status='INCOMPLETE', data_collection_status='INCOMPLETE', errors=['Synthetic lock acquisition failure'],
                      samples=[], hardware_opened=False, hardware_open_attempted=False)
        fast.update(report=report, events=[])
        fast['receipt'].update(status='INCOMPLETE', report_sha256=None, errors=report['errors'], complete_triplets=0,
                              incomplete_triplets=0, dropped_slot_count=0, unacquired_slot_count=50, full_requested_slot_coverage=False)
        self.f.save(fast); result = self.f.compare(); run = result['runs']['20']
        self.assertEqual(result['status'], 'INCOMPLETE_COMPARISON_RECORDS')
        self.assertEqual(run['acquisition_state'], 'UNACQUIRED_STARTUP_RECORD')
        self.assertFalse(run['boot_context_verified'] or run['has_complete_measurements'] or run['data_coverage'])
        self.assertEqual(run['unacquired_slots'], list(range(50)))
        self.assertEqual(sum(p['requested_slot_count'] for p in result['twenty_ms_all_ten_phase_cohorts']), 50)
    def test_low_four_bits_only_descriptive_value_change_upper_bound(self):
        receipts = [{'value': struct.unpack('<f', w.to_bytes(4, 'little'))[0], 'raw_value_hex': w.to_bytes(4, 'little').hex()}
                    for w in (0x3dccccc1, 0x3dccccc7, 0xbdcccccf)]
        result = a.raw_value_stats(receipts)
        self.assertEqual(sum(result['low_four_bit_histogram'].values()), 3)
        self.assertGreater(result['low_four_bit_mask_max_abs_value_change'], 0)
        self.assertLess(result['two_endpoint_change_upper_bound_from_low_four_bit_mask'], .000001)
    def test_cli_invalid_input_returns_false_scope_only_and_changes_no_files(self):
        self.pair(); before = {p: p.read_bytes() for p in self.f.directory.iterdir()}
        args = ['--expected-uids', str(self.f.uid_path), '--expected-source-hashes', str(self.f.source_path),
                '--expected-boot-id', BOOT, '--id', '5']
        for run in self.f.runs:
            for key, path in run['paths'].items(): args.extend([f'--{key}-{run["report"]["plan"]["period_ms"]}', str(path)])
        with redirect_stdout(io.StringIO()) as out: self.assertEqual(a.main(args), 0)
        self.assertEqual(json.loads(out.getvalue())['status'], 'DESCRIPTIVE_COMPARISON_REVIEW_REQUIRED')
        self.assertEqual(before, {p: p.read_bytes() for p in self.f.directory.iterdir()})
        args[args.index('--id')+1] = '6'
        with redirect_stdout(io.StringIO()) as out: self.assertEqual(a.main(args), 1)
        result = json.loads(out.getvalue()); self.assertEqual(result['status'], 'INVALID_ARTIFACTS')
        self.assertFalse(result['motor_output_allowed'] or result['automatic_approval'] or result['hardware_opened'])
        self.assertEqual(before, {p: p.read_bytes() for p in self.f.directory.iterdir()})


if __name__ == '__main__':
    unittest.main()
