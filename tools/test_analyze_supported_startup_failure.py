"""Toy original receipts only; no device, calibration or approval generation."""
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import analyze_supported_startup_failure as tool


def wire(kind, mid, mode=0, fault=0):
    can = ((kind << 24) | (mode << 22) | (fault << 16) | (mid << 8) | 253
           if kind == 2 else (kind << 24) | ((32767 if kind == 1 else 253) << 8) | mid)
    payload = bytes(2)+b'\x7f\xff'+bytes(4) if kind == 1 else bytes(8)
    return (b'AT'+((can << 3) | 4).to_bytes(4, 'big')+b'\x08'+payload+b'\r\n').hex()


def receipt(kind, mid, *, begin=1_000_000_000, reply=True, mode=0, fault=0):
    return dict(tx_hex=wire(kind, mid), rx_hex=wire(2, mid, mode, fault) if reply else bytes(17).hex(),
        written=17, received=17 if reply else 0, start_ns=begin+1000, finish_ns=begin+2000,
        read_start_ns=begin+2_000_000 if reply else 0,
        received_ns=begin+2_001_000 if reply else 0, deadline_ns=begin+30_000_000)


def entry(mid, phase, *, reply=True):
    bus = 'front' if mid < 7 else 'rear'
    return dict(bus=bus, phase=phase, error=None if reply else 'saved native timeout',
        records=[receipt(3 if phase == 'startup_enable' else 1, mid, reply=reply,
                         mode=0 if phase == 'startup_enable' else 2)],
        stats=dict(begin_ns=1_000_000_000, end_ns=1_030_001_000))


def fixture(success=False):
    ordered = [mid for pair in zip(range(1, 7), range(7, 13)) for mid in pair]
    journal = []
    for mid in (ordered if success else ordered[:3]):
        journal.extend((entry(mid, 'startup_enable'), entry(mid, 'startup_zero_gain')))
    if not success:
        journal.append(entry(8, 'startup_enable', reply=False))
    stops = {}
    for bus, ids in tool.BUSES.items():
        ambiguous = [] if success or bus == 'front' else [8]
        confirmed = sorted(ids-set(ambiguous))
        attempt = dict(complete=not ambiguous, confirmed_ids=confirmed, unconfirmed_ids=ambiguous,
            ambiguous_ids=ambiguous, fault_by_id={str(mid):0 for mid in ids},
            evidence=dict(records=[receipt(4, mid) for mid in sorted(ids)],
                stats=dict(begin_ns=1_000_000_000, end_ns=1_003_000_000)))
        stops[bus] = {**copy.deepcopy(attempt),
                      'attempts':[copy.deepcopy(attempt) for _ in range(3 if ambiguous else 1)]}
    return dict(status='COMPLETE_SUPPORTED_OUTPUT' if success else 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED',
        journal=journal, zero_gain_enable_transition=dict(complete=success,
            completed_axes=ordered if success else ordered[:3],
            current_axis=None if success else dict(bus='rear', motor_id=8),
            current_stage=None if success else 'enable'), stop_reports=stops, stop_confirmed=success,
        actual_model_calls=68 if success else 0, cycles=[{}]*95 if success else [],
        learned_targets_sent=success, secret_uid='DO_NOT_COPY_PRIVATE_UID')


class StartupFailureTests(unittest.TestCase):
    def analyze(self, report):
        raw = json.dumps(report).encode()
        return tool.analyze_report(report, hashlib.sha256(raw).hexdigest())

    def test_original_failed_type3_full_write_reply_zero_and_deadline(self):
        result = self.analyze(fixture())
        context = result['failed_request_context']
        self.assertEqual((context['bus'], context['motor_id'], context['phase'], context['request_type']),
                         ('rear', 8, 'startup_enable', 3))
        self.assertTrue(context['full_write']); self.assertTrue(context['no_reply_read'])
        self.assertEqual(context['reply_bytes'], 0)
        self.assertEqual(context['original_timestamps_ns']['deadline_ns'], 1_030_000_000)
        self.assertEqual(context['deadline_from_request_ms'], 29.999)
        self.assertEqual(result['actual_model_calls'], 0)
        self.assertEqual(result['cycles_recorded'], 0)
        self.assertEqual(result['failure_cause'], 'UNKNOWN_FROM_SAVED_HOST_RECEIPTS')
        self.assertFalse(result['automatic_retry']); self.assertFalse(result['hardware_access'])
        self.assertNotIn('DO_NOT_COPY_PRIVATE_UID', json.dumps(result))

    def test_three_mode_zero_rounds_preserve_outstanding_type3_ambiguity(self):
        result = self.analyze(fixture())['stop']
        self.assertEqual(result['mode_zero_observed_ids'], list(range(1, 13)))
        self.assertEqual(result['ambiguous_ids'], [8])
        self.assertEqual(result['unconfirmed_ids'], [8])
        self.assertEqual(result['confirmation_status'], 'UNCONFIRMED')
        self.assertEqual(result['buses']['rear']['raw_STOP_records_validated'], 18)
        self.assertEqual(len(result['matching_reported_confirmed_ids']), 11)

    def test_erasing_reported_ambiguity_cannot_promote_pending_request(self):
        report = fixture(); report['stop_confirmed'] = True
        stop = report['stop_reports']['rear']
        stop.update(complete=True, confirmed_ids=list(range(7, 13)), unconfirmed_ids=[], ambiguous_ids=[])
        result = self.analyze(report)['stop']
        self.assertEqual(result['ambiguous_ids'], [8])
        self.assertNotIn(8, result['matching_reported_confirmed_ids'])
        self.assertEqual(result['confirmation_status'], 'UNCONFIRMED')
        self.assertTrue(result['data_quality_errors'])

    def test_successful_startup_requires_all_original_axis_receipts(self):
        result = self.analyze(fixture(success=True))
        self.assertEqual(result['startup_classification'], 'NO_STARTUP_FAILURE_OBSERVED_IN_SAVED_REPORT')
        self.assertIsNone(result['failed_request_context'])
        self.assertEqual(result['stop']['confirmation_status'], 'CONFIRMED_AS_REPORTED_WITH_MATCHING_RAW')
        self.assertFalse(result['trial_success_or_extension_eligibility_inferred'])

    def test_duplicate_axis_or_nonzero_startup_gain_cannot_claim_complete_startup(self):
        report=fixture(success=True)
        report['journal'][2]=copy.deepcopy(report['journal'][0])
        self.assertEqual(self.analyze(report)['startup_classification'],'UNKNOWN')
        report=fixture(success=True)
        row=report['journal'][1]['records'][0]
        row['tx_hex']=row['tx_hex'][:22]+'0001'+row['tx_hex'][26:]
        self.assertEqual(self.analyze(report)['failed_request_context']['evidence_status'],'UNKNOWN')

    def test_no_journal_or_rows_is_unknown_even_with_success_flags(self):
        for change in (lambda r:r.pop('journal'), lambda r:r.update(journal=[]),
                       lambda r:r['journal'][-1].update(records=[]),
                       lambda r:r['zero_gain_enable_transition'].update(completed_axes=None)):
            report=fixture(success=True);change(report)
            with self.subTest(change=change):
                result=self.analyze(report)
                self.assertEqual(result['startup_classification'], 'UNKNOWN')
                self.assertIsNotNone(result['failed_request_context'])

    def test_missing_stop_receipts_or_summary_cannot_confirm(self):
        for change in (lambda r:r.pop('stop_reports'),
                       lambda r:r['stop_reports']['rear'].update(attempts=[]),
                       lambda r:r['stop_reports']['rear']['attempts'][0].pop('evidence')):
            report=fixture(success=True);change(report)
            with self.subTest(change=change):
                self.assertEqual(self.analyze(report)['stop']['confirmation_status'], 'UNKNOWN')

    def test_boolean_missing_late_and_noncausal_timestamps_stay_unknown(self):
        for key,value in (('deadline_ns',True),('start_ns',None),('finish_ns',0),('received',False)):
            report=fixture();report['journal'][-1]['records'][0][key]=value
            with self.subTest(key=key):
                context=self.analyze(report)['failed_request_context']
                self.assertEqual(context['evidence_status'],'UNKNOWN')
                self.assertIsNone(context['full_write'])
        for key,value in (('received_ns',1_030_000_000),('read_start_ns',0),('finish_ns',1_031_000_000)):
            report=fixture(success=True);report['stop_reports']['rear']['attempts'][0]['evidence']['records'][1][key]=value
            with self.subTest(key=key):
                self.assertNotIn(8,self.analyze(report)['stop']['mode_zero_observed_ids'])

    def test_wrong_id_noncanonical_and_version_reply_cannot_be_stop_proof(self):
        for kind in ('wrong_id','bad_framing','version'):
            report=fixture(success=True)
            row=report['stop_reports']['rear']['attempts'][0]['evidence']['records'][1]
            if kind=='wrong_id':row['rx_hex']=wire(2,9)
            elif kind=='bad_framing':row['rx_hex']='00'+row['rx_hex'][2:]
            else:row['rx_hex']=row['rx_hex'][:14]+'00c456'+row['rx_hex'][20:]
            with self.subTest(kind=kind):
                self.assertNotIn(8,self.analyze(report)['stop']['mode_zero_observed_ids'])

    def test_faulted_mode_zero_is_observed_but_not_healthy_confirmation(self):
        report=fixture(success=True)
        report['stop_reports']['rear']['attempts'][0]['evidence']['records'][1]['rx_hex']=wire(2,8,fault=1)
        result=self.analyze(report)['stop']
        self.assertIn(8,result['mode_zero_observed_ids'])
        self.assertNotIn(8,result['mode_zero_fault_zero_observed_ids'])
        self.assertNotIn(8,result['matching_reported_confirmed_ids'])

    def test_missing_boolean_or_contradictory_fault_summary_cannot_confirm(self):
        for change in (lambda s:s.pop('fault_by_id'),
                       lambda s:s['fault_by_id'].update({'8':1}),
                       lambda s:s['fault_by_id'].update({'8':False})):
            report=fixture(success=True);change(report['stop_reports']['rear'])
            with self.subTest(change=change):
                result=self.analyze(report)['stop']
                self.assertEqual(result['confirmation_status'],'UNKNOWN')
                self.assertTrue(result['data_quality_errors'])
        report=fixture(success=True)
        summary=report['stop_reports']['rear']
        faulty=copy.deepcopy(summary['attempts'][0])
        faulty['evidence']['records'][1]['rx_hex']=wire(2,8,fault=1)
        summary['attempts'].insert(0,faulty)
        summary['fault_by_id']['8']=1
        result=self.analyze(report)['stop']
        self.assertIn(8,result['mode_zero_observed_ids'])
        self.assertNotIn(8,result['mode_zero_fault_zero_observed_ids'])
        self.assertEqual(result['buses']['rear']['retained_raw_fault_bits_by_id']['8'],1)

    def test_cli_pins_report_and_source_refuses_overwrite_mismatch_and_nan(self):
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory);source=base/'report.json';out=base/'analysis.json'
            source.write_text(json.dumps(fixture()))
            sha=hashlib.sha256(source.read_bytes()).hexdigest()
            args=['--report',str(source),'--report-sha256',sha,'--output',str(out)]
            with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(tool.main(args),0)
                before=out.read_bytes();self.assertEqual(tool.main(args),1)
                self.assertEqual(out.read_bytes(),before)
                self.assertEqual(tool.main(['--report',str(source),'--report-sha256','f'*64,
                                            '--output',str(base/'wrong.json')]),1)
                self.assertFalse((base/'wrong.json').exists())
                source.write_text('{"journal":NaN}')
                self.assertEqual(tool.main(['--report',str(source),'--output',str(base/'nan.json')]),1)
            result=json.loads(out.read_text())
            self.assertEqual(result['source_report_sha256'],sha)
            self.assertEqual(result['tool_source_sha256'],hashlib.sha256(Path(tool.__file__).read_bytes()).hexdigest())


if __name__ == '__main__':
    unittest.main()
