"""Bounded diagnostic record storage; no hardware access."""
import contextlib
import io
import json
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device, Observer, Session


def equipment():
    return {scope: Session() for scope in ('front', 'rear')}, Device()


class RecordStorageTests(unittest.TestCase):
    def test_trace_preserves_full_rows_after_native_buffers_change(self):
        class RetainedSession(Session):
            def __init__(self):super().__init__();self.results=[]
            def exchange(self,wires):
                rows,stats=super().exchange(wires)
                stats.rejected[:3]=b'xyz';stats.rejected_size=3
                self.results.append((rows,stats))
                return rows,stats

        sessions={scope:RetainedSession() for scope in ('front','rear')}
        originals=[];capture=bench._RecordTrace.capture
        def capture_original(storage,cycle_index,row):
            originals.append(bench._serialize([row])[0])
            return capture(storage,cycle_index,row)

        with patch.object(bench._RecordTrace,'capture',capture_original):
            report,records=bench.collect(sessions,Device(),Observer(),
                                         mode='stop-proxy',cycles=2,record_storage='trace')
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(report['record_storage']['completed_trace_rows'],2)
        self.assertEqual(report['record_storage']['capacity_cycles'],2)
        self.assertGreater(report['record_storage']['allocated_bytes'],0)
        for session in sessions.values():
            for rows,stats in session.results:
                rows[0].tx[0]=0;rows[0].rx[0]=0
                stats.rejected[0]=0;stats.writes=0
        saved=bench._serialize(records)
        self.assertEqual(saved,originals)
        self.assertEqual([row['cycle'] for row in saved],[1,2])
        self.assertTrue(all(row['observed']=={'q_target_rad_diagnostic_only':[.1]*12}
                            for row in saved))
        self.assertTrue(all(exchange['rejected_hex']=='78797a'
                            for row in saved for phase in ('acquired','output')
                            for exchange in row[phase].values()))
        json.dumps(saved,allow_nan=False)

    def test_trace_copy_cost_is_inside_whole_cycle(self):
        sessions,device=equipment()
        capture=bench._RecordTrace.capture
        def slow_capture(storage,cycle_index,row):
            time.sleep(.002)
            return capture(storage,cycle_index,row)

        with patch.object(bench._RecordTrace,'capture',slow_capture):
            report,records=bench.collect(sessions,device,Observer(),
                                         mode='stop-proxy',cycles=1,record_storage='trace')
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(len(records),1)
        row=report['measurements'][0]
        self.assertGreaterEqual(row['cycle_end_ns']-row['last_proxy_reply_ns'],2_000_000)
        self.assertEqual(row['whole_iteration_ms'],
                         (row['cycle_end_ns']-row['release_ns'])/1e6)

    def test_trace_avoids_json_and_native_evidence_conversion_during_collection(self):
        sessions,device=equipment()
        with patch.object(bench.native,'exchange_evidence',side_effect=AssertionError('hot evidence conversion')), \
                patch.object(bench.json,'dumps',side_effect=AssertionError('hot JSON encoding')):
            report,records=bench.collect(sessions,device,Observer(),
                                         mode='stop-proxy',cycles=1,record_storage='trace')
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual(len(bench._serialize(records)),1)

    def test_trace_invalid_observation_keeps_all_native_evidence(self):
        class Invalid(Observer):
            def consume(self,snapshot):return {'diagnostic_only':float('nan')}

        sessions,device=equipment()
        report,records=bench.collect(sessions,device,Invalid(),
                                     mode='stop-proxy',cycles=1,record_storage='trace')
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        saved,failures=bench._encoded_records_for_output(records)
        self.assertEqual(len(saved),len(failures),1)
        self.assertEqual(saved[0]['status'],'RECORD_STORAGE_FAILED')
        self.assertIn('diagnostic_only',failures[0]['invalid_json_fields'][0]['path'])
        self.assertEqual(set(failures[0]['native_evidence']),{'acquired','output'})
        self.assertTrue(all(len(exchange['records'])==6
                            for phase in ('acquired','output')
                            for exchange in failures[0]['native_evidence'][phase].values()))
        json.dumps({'records':saved,'failures':failures},allow_nan=False)

    def test_trace_copy_failure_keeps_inflight_raw_row(self):
        sessions,device=equipment()
        with patch.object(bench._RecordTrace,'capture',side_effect=RuntimeError('trace failed')):
            report,records=bench.collect(sessions,device,Observer(),
                                         mode='stop-proxy',cycles=2,record_storage='trace')
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(report['cycles_completed'],0)
        self.assertEqual(report['record_storage_failure']['cycle'],1)
        self.assertEqual(type(records[0]),dict)
        saved,failures=bench._encoded_records_for_output(records,report['record_storage_failure'])
        self.assertEqual(len(saved),len(failures),1)
        self.assertEqual(set(failures[0]['native_evidence']),{'acquired','output'})
        self.assertEqual([s.calls for s in sessions.values()],[2,2])

    def test_trace_native_failure_keeps_completed_other_bus(self):
        class FailedRear(Session):
            def exchange(self,wires):
                self.calls+=1
                raise bench.native.ExchangeError('injected rear failure',[],bench.native.Stats())

        sessions={'front':Session(),'rear':FailedRear()}
        report,rows=bench.collect(sessions,Device(),Observer(),
                                  mode='stop-proxy',cycles=2,record_storage='trace')
        self.assertEqual(report['status'],'ABORTED')
        saved,failures=bench._encoded_records_for_output(rows)
        self.assertEqual(failures,[])
        self.assertEqual(len(saved),2)
        self.assertEqual(saved[0]['failure_scope'],'rear')
        self.assertEqual(len(saved[1]['acquired']['front']['records']),6)
        self.assertEqual(saved[1]['output'],{})

    def test_trace_type17_uses_two_slots_and_keeps_all_twelve_inputs_per_bus(self):
        sessions,device=equipment()
        report,records=bench.collect(sessions,device,None,
                                     mode='type17',cycles=1,record_storage='trace')
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertIsNone(report['measurements'][0]['last_proxy_reply_ns'])
        self.assertIsNone(report['host_deadline_misses'])
        self.assertEqual(report['record_storage']['allocated_bytes'],
                         bench.C.sizeof(bench._TraceExchange)*2)
        saved=bench._serialize(records)
        self.assertEqual(saved[0]['output'],{})
        self.assertNotIn('observed',saved[0])
        self.assertTrue(all(len(exchange['records'])==12
                            for exchange in saved[0]['acquired'].values()))

    def test_encoded_row_preserves_serialized_values_and_bus_order(self):
        sessions, device = equipment()
        originals = []
        serialize = bench._serialize

        def capture(rows):
            result = serialize(rows)
            if rows and type(rows[0]) is dict and rows[0].get('cycle'):
                originals.append(result)
            return result

        with patch.object(bench, '_serialize', side_effect=capture):
            report, records = bench.collect(sessions, device, Observer(),
                                            mode='stop-proxy', cycles=2,
                                            record_storage='encoded')
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC')
        self.assertEqual(report['record_storage']['completed_encoded_rows'], 2)
        self.assertEqual(len(records), len(originals))
        self.assertTrue(all(type(row) is str for row in records))
        self.assertEqual([json.loads(row) for row in records], originals)
        self.assertEqual(bench._serialize(records), [row[0] for row in originals])
        self.assertEqual([r['cycle'] for r in bench._serialize(records)], [1, 2])
        self.assertTrue(all(set(r['acquired']) == set(r['output']) == {'front', 'rear'}
                            for r in bench._serialize(records)))
        self.assertTrue(all(len(exchange['records']) == 6
                            for r in bench._serialize(records)
                            for phase in ('acquired', 'output')
                            for exchange in r[phase].values()))

    def test_encoding_cost_is_inside_whole_cycle(self):
        sessions, device = equipment()
        dumps = bench.json.dumps

        def slow_dumps(*args, **kwargs):
            time.sleep(.002)
            return dumps(*args, **kwargs)

        with patch.object(bench.json, 'dumps', side_effect=slow_dumps):
            report, _ = bench.collect(sessions, device, Observer(),
                                      mode='stop-proxy', cycles=1,
                                      record_storage='encoded')
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC')
        row = report['measurements'][0]
        self.assertGreaterEqual(row['cycle_end_ns']-row['last_proxy_reply_ns'], 2_000_000)
        self.assertEqual(row['whole_iteration_ms'],
                         (row['cycle_end_ns']-row['release_ns'])/1e6)

    def test_invalid_json_aborts_and_preserves_native_evidence(self):
        class Invalid(Observer):
            def consume(self, snapshot):
                return {'diagnostic_only': float('nan')}

        sessions, device = equipment()
        report, records = bench.collect(sessions, device, Invalid(),
                                        mode='stop-proxy', cycles=3,
                                        record_storage='encoded')
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(report['record_storage_failure']['cycle'], 1)
        self.assertEqual(len(records), 1)
        self.assertEqual([s.calls for s in sessions.values()], [2, 2])
        saved, failures = bench._encoded_records_for_output(records,
                                                report['record_storage_failure'])
        self.assertEqual(len(saved), len(failures), 1)
        self.assertEqual(saved[0]['status'], 'RECORD_STORAGE_FAILED')
        self.assertEqual(failures[0]['invalid_json_fields'][0]['type'], 'float')
        self.assertIn('diagnostic_only', failures[0]['invalid_json_fields'][0]['path'])
        self.assertEqual(set(failures[0]['native_evidence']), {'acquired', 'output'})
        self.assertTrue(all(len(v['records']) == 6
                            for phase in ('acquired', 'output')
                            for v in failures[0]['native_evidence'][phase].values()))
        json.dumps({'records': saved, 'failures': failures}, allow_nan=False)

    def test_default_retains_native_records_and_no_record_storage_field(self):
        sessions, device = equipment()
        report, records = bench.collect(sessions, device, Observer(),
                                        mode='stop-proxy', cycles=1)
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC')
        self.assertNotIn('record_storage', report)
        self.assertEqual(type(records[0]), dict)
        self.assertEqual(len(records[0]['acquired']['front'][0]), 6)

    def test_native_failure_retains_completed_other_bus_evidence(self):
        class FailedRear(Session):
            def exchange(self, wires):
                self.calls += 1
                raise bench.native.ExchangeError('injected rear failure', [],
                                                 bench.native.Stats())

        sessions = {'front': Session(), 'rear': FailedRear()}
        report, rows = bench.collect(sessions, Device(), Observer(),
                                     mode='stop-proxy', cycles=2,
                                     record_storage='encoded')
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles_completed'], 0)
        self.assertTrue(any('native_failure' in row for row in rows))
        saved, failures = bench._encoded_records_for_output(rows)
        self.assertEqual(failures, [])
        self.assertEqual(len(saved), 2)
        self.assertEqual(saved[0]['failure_scope'], 'rear')
        self.assertEqual(len(saved[1]['acquired']['front']['records']), 6)
        self.assertEqual(saved[1]['output'], {})
        json.dumps(saved, allow_nan=False)

    def test_feedback_combination_rejected_before_output_or_hardware(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                bench.main(['--execute', '--record-storage', 'encoded',
                            '--compare-feedback', '--mode', 'stop-proxy',
                            '--acquisition-only', '--output', '/tmp/should-not-exist'])
        self.assertEqual(raised.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
