"""Synthetic serial evidence, including unresolved Enable/STOP attribution."""
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from singularitydog_hw import rs05_trial_protocol as protocol
from tools.analyze_motor_reply_failure import READER_COUNTERS, analyze, load_report, main

B = 1_000_000_000
MS = 1_000_000


def response(mid, mode=0):
    can_id=(2 << 24) | (mode << 22) | (mid << 8) | 0xfd
    return b'AT'+((can_id << 3) | 4).to_bytes(4,'big')+b'\x08'+bytes(8)+b'\r\n'


def tx(mid, step, when):
    wire=(protocol.enable_request(phase=protocol.TrialPhase.ENABLE,motor_id=mid)
          if step=='enable' else protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=mid))
    return dict(kind='tx',motor_id=mid,step=step,hex=wire.hex(),start_ns=when,
                finish_ns=when+10_000,returned_bytes=17)


def rx(raw, when, kind='rx_bytes'):
    return dict(kind=kind,hex=raw.hex(),received_ns=when)


def fixture():
    stops={bus:dict(confirmed_ids=list(range(lo,hi)),unconfirmed_ids=[],ambiguous_ids=[],
                    complete=True,errors=[]) for bus,lo,hi in (('front',1,7),('rear',7,13))}
    stops['rear'].update(confirmed_ids=[7,8,9,11,12],unconfirmed_ids=[10],ambiguous_ids=[10],complete=False)
    return dict(stop_confirmed=False,boot_id='SYNTHETIC_UNPUBLISHED_BOOT',
                motor_power_epoch='SYNTHETIC_UNPUBLISHED_POWER',
                expected_uids_sha256='1'*64,
                events_by_bus={'front':[], 'rear':[
                    tx(10,'enable',B),tx(10,'stop',B+260*MS),rx(response(10),B+263*MS)]},
                stop_reports=stops)


def failed_exchange(*, before_write=False):
    before = {key: 0 for key in READER_COUNTERS}
    after = dict(before, read_until_calls=1, select_calls=1,
                 select_wait_ns=250*MS, hard_expiries=1)
    return dict(kind='exchange_failure',motor_id=10,step='enable',
                event_start_index=0,event_end_index=0 if before_write else 1,
                request_start_ns=None if before_write else B,
                hard_deadline_ns=None if before_write else B+250*MS,
                failed_at_ns=B+250*MS,error='PRIVATE_ORIGINAL_EXCEPTION_TEXT',
                receive_boundary_evidence=dict(partial_hex='',discarded_bytes=0,backlogged_bytes=0),
                reader_counters_before=before,reader_counters_after=after,reader_counters_delta=after.copy(),
                diagnostic_errors=[],counter_scope='host_serial_reader_not_CAN_or_motor_delivery',
                automatic_retry=False)


class ReplyFailureAnalysisTests(unittest.TestCase):
    def test_failure_boundary_counters_retained_without_can_root_cause_inference(self):
        report=fixture();event=failed_exchange()
        report['events_by_bus']['rear'].insert(1,event)
        result=analyze(report);row=result['buses']['rear']['suspicious_exchanges'][0]
        observation=row['failure_observation']
        self.assertEqual(observation['request_deadline_ms'],250)
        self.assertEqual(observation['failed_after_deadline_ms'],0)
        self.assertEqual(observation['reader_counters_delta']['select_wait_ns'],250*MS)
        self.assertEqual(observation['host_receive_condition'],'HOST_DEADLINE_WITHOUT_RECORDED_READ_BYTES')
        self.assertFalse(result['root_cause_established']);self.assertTrue(result['unresolved_stop_evidence'])
        self.assertNotIn('PRIVATE_ORIGINAL_EXCEPTION_TEXT',json.dumps(result))

    def test_pre_write_failure_is_not_attached_to_previous_successful_transaction(self):
        report=fixture();event=failed_exchange(before_write=True)
        event.update(event_start_index=2,event_end_index=2)
        report['events_by_bus']['rear']=[tx(9,'stop',B),rx(response(9),B+MS),event,
                                       tx(10,'enable',B+255*MS),tx(10,'stop',B+260*MS),
                                       rx(response(10),B+263*MS)]
        bus=analyze(report)['buses']['rear']
        self.assertEqual(len(bus['pre_write_failures']),1)
        self.assertFalse(bus['pre_write_failures'][0]['request_written'])
        self.assertNotIn('failure_observation',bus['suspicious_exchanges'][0])

    def test_reader_eof_bytes_transients_and_unavailable_counters_are_separate_facts(self):
        cases=[('eof_events',1,'HOST_READER_EOF_RECORDED'),
               ('bytes_received',17,'HOST_READER_BYTES_RECORDED'),
               ('select_eintr',1,'HOST_READER_TRANSIENT_EVENTS_RECORDED'),
               (None,None,'READER_COUNTERS_UNAVAILABLE')]
        for key,value,condition in cases:
            with self.subTest(condition=condition):
                report=fixture();event=failed_exchange()
                if key is None:
                    for name in ('reader_counters_before','reader_counters_after','reader_counters_delta'):event[name]=None
                else:
                    event['reader_counters_after'][key]=value;event['reader_counters_delta'][key]=value
                report['events_by_bus']['rear'].insert(1,event)
                obs=analyze(report)['buses']['rear']['suspicious_exchanges'][0]['failure_observation']
                self.assertEqual(obs['host_receive_condition'],condition)

    def test_corrupt_counter_delta_scope_or_transaction_binding_rejected(self):
        for mutation in ('delta','boolean','scope','index','request','retry','backlog','deadline'):
            with self.subTest(mutation=mutation):
                report=fixture();event=failed_exchange()
                if mutation=='delta':event['reader_counters_delta']['hard_expiries']=2
                elif mutation=='boolean':event['reader_counters_after']['read_calls']=True
                elif mutation=='scope':event['counter_scope']='CAN_delivered'
                elif mutation=='index':event['event_end_index']=2
                elif mutation=='request':event['request_start_ns']=B+1
                elif mutation=='retry':event['automatic_retry']=True
                elif mutation=='deadline':event['hard_deadline_ns']=B+5*MS
                else:event['receive_boundary_evidence']['backlogged_bytes']=-1
                report['events_by_bus']['rear'].insert(1,event)
                with self.assertRaises(ValueError):analyze(report)

    def test_incomplete_diagnostic_storage_flag_is_not_silently_ignored(self):
        report=fixture()
        report['diagnostic_storage_by_bus']={bus:dict(unlogged_receive_failure=False,unlogged_exchange_failure=False)
                                            for bus in ('front','rear')}
        self.assertTrue(analyze(report)['failure_diagnostic_storage_complete'])
        report['diagnostic_storage_by_bus']['rear']['unlogged_exchange_failure']=True
        self.assertFalse(analyze(report)['failure_diagnostic_storage_complete'])
        self.assertIsNone(analyze(fixture())['failure_diagnostic_storage_complete'])
        report['diagnostic_storage_by_bus']['front']['unlogged_receive_failure']=0
        with self.assertRaises(ValueError):analyze(report)

    def test_full_enable_write_no_rx_not_repaired_by_later_reset_observation(self):
        result=analyze(fixture());bus=result['buses']['rear']
        failed=bus['suspicious_exchanges'][0]
        self.assertEqual(failed['motor_id'],10)
        self.assertEqual(failed['step'],'enable')
        self.assertEqual(failed['classification'],'FULL_HOST_WRITE_WITHOUT_RECORDED_RX')
        self.assertTrue(failed['host_write_complete'])
        self.assertEqual(failed['received_bytes'],0)
        self.assertEqual(bus['reset_observed_during_stop_ids'],[10])
        self.assertEqual(bus['ambiguous_stop_ids_from_owner'],[10])
        self.assertTrue(result['unresolved_stop_evidence'])
        self.assertFalse(result['stop_confirmation_created'])
        self.assertFalse(result['root_cause_established'])
        self.assertFalse(result['output_allowed'])

    def test_summary_success_conflicting_with_owner_is_exposed(self):
        report=fixture();report['stop_confirmed']=True
        result=analyze(report)
        self.assertTrue(result['contradictory_stop_summary'])
        self.assertTrue(result['unresolved_stop_evidence'])

    def test_split_receive_counts_matching_frame_without_claiming_causality(self):
        report=fixture();raw=response(10,2)
        report['events_by_bus']['rear'].insert(1,rx(raw[:15],B+MS))
        report['events_by_bus']['rear'].insert(2,rx(raw[15:],B+2*MS))
        result=analyze(report)
        self.assertEqual(result['buses']['rear']['suspicious_exchanges'],[])
        self.assertTrue(result['unresolved_stop_evidence'])

    def test_partial_rx_and_bad_bytes_remain_distinct_from_absent_rx(self):
        for raw in (response(10)[:15],b'bad'):
            with self.subTest(raw=raw):
                report=fixture();report['events_by_bus']['rear'].insert(1,rx(raw,B+MS))
                bus=analyze(report)['buses']['rear']
                first=bus['suspicious_exchanges'][0]
                self.assertEqual(first['classification'],'RX_BYTES_WITHOUT_MATCHING_COMPLETE_REPLY')
                self.assertEqual(first['received_bytes'],len(raw))
                self.assertTrue(bus['parser_partial_bytes_at_end'] or bus['parser_discarded_bytes'])

    def test_wrong_axis_complete_reply_does_not_match_enable(self):
        report=fixture();report['events_by_bus']['rear'].insert(1,rx(response(9,2),B+MS))
        first=analyze(report)['buses']['rear']['suspicious_exchanges'][0]
        self.assertEqual(first['reply_candidate_count'],0)
        self.assertEqual(first['received_bytes'],17)

    def test_partial_host_write_is_preserved(self):
        report=fixture();report['events_by_bus']['rear'][0]['returned_bytes']=15
        first=analyze(report)['buses']['rear']['suspicious_exchanges'][0]
        self.assertFalse(first['host_write_complete'])
        self.assertEqual(first['classification'],'HOST_WRITE_INCOMPLETE')

    def test_rejected_receive_with_unknown_timestamp_is_never_candidate(self):
        report=fixture();report['events_by_bus']['rear'].insert(1,rx(response(10,2),None,'rx_rejected'))
        bus=analyze(report)['buses']['rear']
        self.assertEqual(bus['receive_chunks_without_valid_time'],1)
        self.assertEqual(bus['rejected_receive_chunks'],1)
        self.assertEqual(bus['suspicious_exchanges'][0]['reply_candidate_count'],0)

    def test_private_metadata_and_raw_hex_not_emitted(self):
        report=fixture();report['extra']='PRIVATE_LOCATION_MARKER'
        text=json.dumps(analyze(report))
        for marker in ('SYNTHETIC_UNPUBLISHED_BOOT','SYNTHETIC_UNPUBLISHED_POWER',
                       'PRIVATE_LOCATION_MARKER',response(10).hex()):
            self.assertNotIn(marker,text)

    def test_noncausal_or_boolean_timestamp_rejected(self):
        for mutation in ('write','receive','boolean'):
            with self.subTest(mutation=mutation):
                report=fixture()
                if mutation=='write':report['events_by_bus']['rear'][0]['finish_ns']=B-1
                elif mutation=='receive':report['events_by_bus']['rear'][-1]['received_ns']=B
                else:report['events_by_bus']['rear'][0]['start_ns']=True
                with self.assertRaises(ValueError):analyze(report)

    def test_cross_bus_or_forged_step_and_wire_rejected(self):
        for field,value in (('motor_id',1),('step','zero'),('hex','00'),('returned_bytes',True)):
            with self.subTest(field=field):
                report=fixture();report['events_by_bus']['rear'][0][field]=value
                with self.assertRaises(ValueError):analyze(report)

    def test_positive_gain_mislabeled_as_zero_rejected(self):
        report=fixture();event=report['events_by_bus']['rear'][0]
        wire=protocol.motion_request(phase=protocol.TrialPhase.ZERO_GAIN,center_rad=0.,motor_id=10)
        event.update(step='zero',hex=wire.hex())
        self.assertEqual(analyze(report)['buses']['rear']['transactions'],2)
        raw=bytearray(wire);raw[11]=1;event['hex']=raw.hex()
        with self.assertRaises(ValueError):analyze(report)

    def test_malformed_stop_id_accounting_rejected(self):
        for field,value in (('confirmed_ids',[7,8,9,10,11,12]),('ambiguous_ids',[7]),
                            ('unconfirmed_ids',[10,10]),('unconfirmed_ids',[True]),
                            ('complete',1),('confirmed_ids',[[7]])):
            with self.subTest(field=field):
                report=fixture();report['stop_reports']['rear'][field]=value
                with self.assertRaises(ValueError):analyze(report)

    def test_zero_reply_without_rx_timestamp_rejected(self):
        report=fixture();report['events_by_bus']['rear'][-1]['received_ns']=None
        with self.assertRaises(ValueError):analyze(report)

    def test_duplicate_nonfinite_json_bad_hash_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'report.json'
            raw=json.dumps(fixture()).encode();path.write_bytes(raw)
            report,sha=load_report(path,hashlib.sha256(raw).hexdigest())
            self.assertEqual(report,fixture())
            with self.assertRaises(ValueError):load_report(path,'0'*64)
            link=Path(folder)/'alias.json';link.symlink_to(path)
            with self.assertRaises(OSError):load_report(link)
            for raw in (b'{"stop_confirmed":true,"stop_confirmed":false}',
                        b'{"x":NaN}',b'{"x":1e999}'):
                path.write_bytes(raw)
                with self.assertRaises(ValueError):load_report(path)

    def test_cli_reports_only_summary_and_rejects_private_error_text(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'report.json';path.write_text(json.dumps(fixture()))
            output=io.StringIO()
            with contextlib.redirect_stdout(output):code=main(['--report',str(path)])
            result=json.loads(output.getvalue())
            self.assertEqual(code,0);self.assertFalse(result['output_allowed'])
            self.assertEqual(len(result['report_sha256']),64)
            output=io.StringIO()
            with contextlib.redirect_stdout(output):code=main(['--report',str(path)+'-PRIVATE_MISSING'])
            self.assertEqual(code,2);self.assertNotIn('PRIVATE_MISSING',output.getvalue())


if __name__=='__main__':
    unittest.main()
