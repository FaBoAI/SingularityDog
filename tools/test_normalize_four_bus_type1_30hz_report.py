"""Strict synthetic compatibility tests; no device/model execution or qualification."""
import copy
import hashlib
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.append(str(Path(__file__).resolve().parents[1]/'runtime'))
from experiments.four_bus_type1 import test_type1_profile as fixtures, type1_profile as P
from experiments.four_bus_type1.test_type1_30hz import BUNDLE, model_selection
from singularitydog_hw.policy_clock import control_clock_payload

import normalize_four_bus_type1_30hz_report as tool
import analyze_four_bus_type1_failure as audit


def wire(kind, mid, *, receive=False):
    if kind == 1:
        cid = (kind << 24) | (32767 << 8) | mid
        data = struct.pack('>4H', 32767, 32767, 0, 0)
    elif receive and kind == 2:
        cid = (kind << 24) | (2 << 22) | (mid << 8) | 253
        data = struct.pack('>4H', 32767, 32767, 32767, 250)
    else:
        cid = (kind << 24) | ((mid if receive else 253) << 8) | (253 if receive else mid)
        data = bytes.fromhex('1c700000')+(struct.pack('<f', 40.) if receive else bytes(4))
    return (b'AT'+((cid << 3) | 4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n').hex()


def batch(ids, phase, begin, at, reply, deadline):
    chosen = ids[:1] if phase == 'voltage' else ids
    kind = 17 if phase == 'voltage' else 1
    rows = [{'tx_hex': wire(kind, mid), 'rx_hex': wire(17 if kind == 17 else 2, mid, receive=True),
             'start_ns': at+slot*1000, 'finish_ns': at+slot*1000+10,
             'read_start_ns': reply-10, 'received_ns': reply, 'deadline_ns': deadline,
             'written': 17, 'received': 17} for slot, mid in enumerate(chosen)]
    return {'physical_ids': ids, 'phase': phase, 'original_record_count': len(rows), 'records': rows,
            'stats': {'begin_ns': begin, 'end_ns': reply+10, 'writes': len(rows), 'reads': 1, 'bytes': len(rows)*17},
            'rejected_hex': '', 'rejected_total_bytes': 0, 'rejected_truncated': False}


@unittest.skipUnless(BUNDLE.exists(), 'Pinned 30Hz release is unavailable')
class ProjectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.world = fixtures.World(Path(cls.temp.name).resolve())
        clock, model = control_clock_payload(30), model_selection()
        contract = P.build_contract(cls.world.plan, cls.world.lineage_refs(), cls.world.geometry_doc,
                                   cls.world.geometry, control_clock=clock, policy30_model=model)
        diagnostic = {'report': {'path': '/synthetic/diag.json', 'sha256': 'e'*64},
            'pinned_capture': cls.world.current, 'completed_cycles': 501,
            'first_release_monotonic_ns': 60_000_000_000, 'workers_joined_monotonic_ns': 77_000_000_000,
            'control_clock': clock, 'policy30_model_canonical_sha256': P.canonical_sha256(model)}
        cls.profile = P.assemble_profile(contract, 'zero_gain_timing', 2,
            {'current_capture': cls.world.capture_evidence(), 'stop_proxy_diagnostic': diagnostic, 'predecessor': None},
            prepared_at='2026-10-14T09:00:00+09:00')
        epoch = 100_000_000_000
        cls.report = fixtures.type1_report(contract, 'zero_gain_timing', 2, epoch, epoch+2_000_000_000)
        cls.report.update(schema=P.REPORT_SCHEMA_30, control_clock=clock, policy30_model=model,
            source_and_input_pins_unchanged=True, completed_cycles=1,
            profile_canonical_sha256=P.profile_canonical_sha256(cls.profile),
            input_sha256={contract['source_manifest']['path']: contract['source_manifest']['sha256'],
                          '/synthetic/library': '7'*64, '/synthetic/receipt': '8'*64})
        cls.report['input_sha256'].update({ref['path']: ref['sha256'] for ref in model['artifacts'].values()})
        m = cls.report['measurement'] = {key: copy.deepcopy(cls.report[key]) for key in
            ('mode', 'duration_s', 'boot_id', 'motor_power_epoch', 'contract_sha256', 'pacing', 'status',
             'completed_cycles', 'motor_enable_sent', 'type1_sent', 'positive_gain_sent', 'learned_targets_attempted',
             'control_clock', 'policy30_model')}
        m.update(epoch_ns=epoch, normal_ramp_completed=True, errors=[], failure_retained=False,
                 inference_tx_budget_ns=20_000_000, cycle_budget_ns=33_333_334,
                 backend_usage={'kind': 'genuine_subset_active_cpp', 'source_manifest_sha256': contract['source_manifest']['sha256'],
                                'library_sha256': '7'*64, 'build_record_sha256': '8'*64},
                 raw_journal={}, write_deadline_journal={})
        cap, end = epoch+20_000_000, epoch+33_333_334
        cycle = {'index': 0, 'slot': 0, 'release_ns': epoch, 'begin_ns': epoch, 'first_ns': epoch+1_000_000,
                 'hard_end_ns': end, 'write_deadline_ns': cap, 'cycle_end_ns': epoch+23_000_000,
                 'infer_end_ns': epoch+10_000_000, 'request_count': 28, 'completed': True,
                 'decision': {'accepted': True, 'checked_ns': epoch+23_000_000},
                 'imu': {'read_started_monotonic_ns': epoch+2_000_000,
                         'read_finished_monotonic_ns': epoch+3_000_000},
                 'hold': {}, 'voltage': {}, 'output': {}}
        for port, ids in contract['topology_by_port'].items():
            hold = batch(ids, 'feedback_hold', epoch, epoch+1_000_000, epoch+5_000_000, end)
            voltage = batch(ids, 'voltage', epoch+5_000_001, epoch+6_000_000, epoch+8_000_000, end)
            output = batch(ids, 'output', epoch+15_000_000, epoch+16_000_000, epoch+22_000_000, end)
            for name, value in (('hold', hold), ('voltage', voltage), ('output', output)):
                value['physical_port'] = port
                cycle[name][port] = value
            m['raw_journal'][port] = [[b['phase'], copy.deepcopy(b)] for b in (hold, voltage, output)]
            m['write_deadline_journal'][port] = [[2, end, cap, epoch+14_000_000]]
        cycle.update(final_write_ns=max(r['finish_ns'] for b in cycle['output'].values() for r in b['records']),
                     output_last_reply_ns=epoch+22_000_000)
        m['cycles'] = [cycle]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def normalized(self, report=None, **kw):
        return tool.normalize(report or self.report, self.profile,
            original_reference={'path': '/synthetic/report.json', 'sha256': 'a'*64},
            profile_reference={'path': '/synthetic/profile.json', 'sha256': 'b'*64},
            expected_source_sha256=kw.get('source', self.profile['contract']['source_manifest']['sha256']))

    def test_only_existing_aliases_and_derivation_are_added(self):
        before = copy.deepcopy(self.report)
        result = self.normalized()
        self.assertEqual({k: v for k, v in result.items() if k not in (*tool.FIELDS, tool.METADATA)}, before)
        self.assertEqual(self.report, before)
        self.assertEqual((result['inference_tx_budget_ns'], result['cycle_budget_ns']), (20_000_000, 33_333_334))
        self.assertFalse(result[tool.METADATA]['output_approval_granted_here'])
        self.assertFalse(result[tool.METADATA]['sticky_STOP_ambiguity_cleared'])
        self.assertEqual(result[tool.METADATA]['raw_validation']['original_Type1_output_records_checked'], 12)

    def test_missing_contradictory_nested_or_existing_top_fields_rejected(self):
        for name in tool.FIELDS:
            for change in ('missing', 'different', 'existing'):
                source = copy.deepcopy(self.report)
                if change == 'missing':
                    source['measurement'].pop(name)
                elif change == 'different':
                    source['measurement'][name] += 1
                else:
                    source[name] = source['measurement'][name]
                with self.subTest(field=name, change=change), self.assertRaises(ValueError):
                    self.normalized(source)

    def test_source_contract_model_and_flags_cannot_be_requalified(self):
        changes = [lambda r: r.update(source_manifest_sha256='0'*64),
                   lambda r: r.update(profile_canonical_sha256='0'*64),
                   lambda r: r['input_sha256'].pop(next(iter(model_selection()['artifacts'].values()))['path']),
                   lambda r: r['measurement']['backend_usage'].update(source_manifest_sha256='0'*64),
                   lambda r: r['measurement']['policy30_model'].update(training_updates=499),
                   lambda r: r['measurement'].update(positive_gain_sent=True),
                   lambda r: r.update(restoration_complete=False),
                   lambda r: r.update(source_and_input_pins_unchanged=False),
                   lambda r: r['terminal_stop'].update(ambiguous_ids=[3], stop_confirmed=False)]
        for change in changes:
            source = copy.deepcopy(self.report); change(source)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.normalized(source)
        with self.assertRaisesRegex(ValueError, 'source manifest differs'):
            self.normalized(source='0'*64)

    def test_raw_late_missing_or_substituted_output_rejected(self):
        def both(source):
            m = source['measurement']
            return m['cycles'][0]['output']['port0']['records'][0], m['raw_journal']['port0'][2][1]['records'][0]
        for change in ('late_write', 'late_reply', 'missing', 'wrong_id'):
            source = copy.deepcopy(self.report)
            for raw in both(source):
                if change == 'late_write':
                    raw['finish_ns'] = source['measurement']['cycles'][0]['write_deadline_ns']
                    raw['read_start_ns'] = raw['finish_ns']+1
                elif change == 'late_reply':
                    raw['received_ns'] = raw['deadline_ns']
                elif change == 'missing':
                    raw.update(received=0, read_start_ns=0, received_ns=0, rx_hex='00'*17)
                else:
                    raw['tx_hex'] = wire(1, 12)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.normalized(source)

    def test_missing_native_cutoff_or_original_journal_match_rejected(self):
        for change in ('cutoff', 'journal', 'clock'):
            source = copy.deepcopy(self.report)
            if change == 'cutoff':
                source['measurement']['write_deadline_journal']['port0'] = []
            elif change == 'journal':
                source['measurement']['raw_journal']['port0'].pop()
            else:
                source['measurement']['cycles'][0]['first_ns'] += 1
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.normalized(source)

    def test_file_pin_rejects_an_unrecorded_change(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp).resolve()/'report.json'
            path.write_bytes(b'{}')
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            path.write_bytes(b'{"changed": true}')
            with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
                audit.load_report(path, digest)


if __name__ == '__main__':
    unittest.main()
