"""File-only/socket tests for default-off selected disabled codec producer.

The exact existing ordinary active library is supplied by R21_ACTIVE_LIBRARY.
No test builds or opens a physical device/model or grants output admission.
"""
import contextlib
from concurrent.futures import Future,ThreadPoolExecutor
import ctypes as C
import copy
import hashlib
import io
import json
from pathlib import Path
import struct
import time
import os
import threading
import unittest
from unittest.mock import patch

import test_unpaired_native_feedback_codec_integration as fixtures
from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import native_active_transport as active
from singularitydog_hw import policy_live_profile as profiles
from singularitydog_hw import unpaired_native_feedback_codec as codec
from singularitydog_hw import diagnostic_runtime_output as adapter


class ProducerTests(unittest.TestCase):
    setUpClass=classmethod(fixtures.TwoBusCodecTests.setUpClass.__func__)
    tearDownClass=classmethod(fixtures.TwoBusCodecTests.tearDownClass.__func__)
    setUp=fixtures.TwoBusCodecTests.setUp
    tearDown=fixtures.TwoBusCodecTests.tearDown
    start_peers=fixtures.TwoBusCodecTests.start_peers

    def decoders(self):
        return codec.prepare_unpaired_decoders(self.sessions,self.selected)[0]

    def raw(self,scope,*,codes=None):
        rows=(active.Record*6)()
        for j,mid in enumerate(fixtures.runtime.BUSES[scope]):
            wire=bench._STOP_WIRES[mid];answer=fixtures.reply(wire)
            if codes is not None:
                answer=answer[:7]+struct.pack('>4H',*codes[j])+answer[15:]
            rows[j].tx[:]=wire;rows[j].rx[:]=answer
            rows[j].start_ns=1000+j*100;rows[j].finish_ns=1050+j*100
            rows[j].read_start_ns=1080+j*100;rows[j].received_ns=1090+j*100
            rows[j].deadline_ns=10000;rows[j].written=rows[j].received=17
        return rows

    def sample(self,begin=900,end=950):
        return {'read_started_monotonic_ns':begin,'read_finished_monotonic_ns':end,
                'accel_m_s2':[0.,0.,9.80665],'gyro_rad_s':[0.,0.,0.]}

    def parity(self,records,sample,tick,decoders):
        before={s:bytes(r) for s,r in records.items()}
        try:old=bench.snapshot_from_records(records,sample,tick)
        except Exception as error:
            with self.assertRaises(type(error)) as new:
                bench.snapshot_from_records(records,sample,tick,feedback_decoders=decoders)
            self.assertEqual(str(error),str(new.exception))
        else:
            new=bench.snapshot_from_records(records,sample,tick,feedback_decoders=decoders)
            self.assertEqual(old,new)
            for left,right in zip(old['motors'],new['motors']):
                self.assertEqual(struct.pack('>d',left['value']),struct.pack('>d',right['value']))
            self.assertEqual(json.dumps(old,sort_keys=True),json.dumps(new,sort_keys=True))
        self.assertEqual(before,{s:bytes(r) for s,r in records.items()})

    def test_exact_snapshot_and_double_bits_across_code_edges(self):
        decoders=self.decoders();codes=(0,1,32767,32768,65534,65535)
        rows={scope:self.raw(scope,codes=[(c,c,65535-c,c) for c in codes]) for scope in fixtures.runtime.BUSES}
        self.parity(rows,self.sample(),2000,decoders)

    def test_genuine_two_bus_prepared_feedback_voltage_and_final_stop_same_deadline(self):
        self.zero_sessions();decoders=self.decoders();self.start_peers()
        with ThreadPoolExecutor(max_workers=3) as pool:
            workers=self.borrowed(pool)
            ready={s:Future() for s in self.sessions}
            for _ in range(3):pool.submit(lambda:None).result(timeout=1)
            started=time.monotonic_ns();deadline=started+20_000_000
            def exchange(scope,wires,**kwargs):
                return self.sessions[scope].exchange(wires,deadline_ns=deadline,**kwargs)
            voltage={scope:pool.submit(bench._feedback_then_voltage,exchange,scope,
                tuple(bench._STOP_WIRES[mid] for mid in fixtures.runtime.BUSES[scope]),
                bench._READ_WIRES[fixtures.runtime.BUSES[scope][0],'voltage'],ready[scope],
                publish_before_native=True) for scope in self.sessions}
            raw={scope:future.result(timeout=max(0,(deadline-time.monotonic_ns())/1e9)) for scope,future in ready.items()}
            sample=self.sample(started,started+1);tick=time.monotonic_ns()
            snapshot,proof=bench._validated_feedback_for_voltage(raw,sample,tick,feedback_decoders=decoders)
            self.assertTrue(all(type(f) is Future for f in (*ready.values(),*voltage.values())))
            electric={scope:future.result(timeout=max(0,(deadline-time.monotonic_ns())/1e9)) for scope,future in voltage.items()}
            full,_=bench._verify_voltage_after_inference(raw,electric,sample,snapshot,{'front':1,'rear':7},
                time.monotonic_ns,42,proof)
            checked=bench._verify_voltage_final_freshness(raw,electric,sample,snapshot,full,
                {'front':1,'rear':7},time.monotonic_ns,42,proof)
            self.assertLess(checked,deadline)
            outputs=workers.submit_decoded(workers._stop_batches,deadline_ns=deadline,
                label='diagnostic_policy_output')
            waiter=active.make_owned_waiter(self.lib,self.cancel_read,spin_us=500)
            decorated=workers.collect_output(outputs,deadline_ns=deadline,native_deadline_ns=deadline,
                deadline_wait=waiter,timing=fixtures.runtime._PendingCycleTiming())
            results={scope:row[0] for scope,row in decorated.items()}
            self.assertLess(time.monotonic_ns(),deadline)
            self.assertEqual(workers.output_notification_groups,1)
            self.assertTrue(all(row is outputs[scope].result() for scope,row in decorated.items()))
            workers.close()
        self.assertEqual(sum(map(len,self.seen.values())),26)
        for scope,(records,stats) in results.items():
            self.assertEqual(len(records),6)
            self.assertTrue(all(bytes(r.rx)[:7]==bench._STOP_REPLY_HEADERS[mid] for mid,r in zip(fixtures.runtime.BUSES[scope],records)))
            self.assertEqual([request.kind for request in self.seen[scope]],[4]*6+[17]+[4]*6)

    def test_all_incomplete_causal_age_and_imu_failures_keep_legacy_priority(self):
        decoders=self.decoders()
        for change in ('written','received','received_ns','deadline_ns','start_ns'):
            rows={s:self.raw(s) for s in self.sessions}
            setattr(rows['front'][0],change,0 if change!='deadline_ns' else 1000)
            self.parity(rows,self.sample(),2000,decoders)
        self.parity({s:self.raw(s) for s in self.sessions},self.sample(),100_002_000,decoders)
        self.parity({s:self.raw(s) for s in self.sessions},self.sample(begin=2001),2000,decoders)
        sample=self.sample();sample['gyro_rad_s'][1]=float('nan')
        self.parity({s:self.raw(s) for s in self.sessions},sample,2000,decoders)

    def test_mode_fault_sentinel_and_bad_frame_retain_original_errors(self):
        decoders=self.decoders()
        for kind in ('mode','fault','sentinel','framing','cross_bus','duplicate'):
            rows={s:self.raw(s) for s in self.sessions};row=rows['front'][1]
            if kind in ('mode','fault'):
                row.rx[:]=fixtures.reply(bytes(row.tx),mode=2 if kind=='mode' else 0,fault=1 if kind=='fault' else 0)
            elif kind=='sentinel':row.rx[7:10]=b'\x00\xc4\x56'
            elif kind=='framing':row.rx[16]=0
            elif kind=='cross_bus':row.tx[:]=bench._STOP_WIRES[7];row.rx[:]=fixtures.reply(bytes(row.tx))
            else:row.tx[:]=bench._STOP_WIRES[1];row.rx[:]=fixtures.reply(bytes(row.tx))
            self.parity(rows,self.sample(),2000,decoders)

    def test_owned_buffer_and_busy_legacy_fallback(self):
        decoders=self.decoders();owned={s:self.raw(s) for s in self.sessions}
        aliased={s:(active.Record*6).from_buffer_copy(bytes(r)) for s,r in owned.items()}
        view={s:(active.Record*6).from_buffer(r) for s,r in aliased.items()}
        self.parity(view,self.sample(),2000,decoders)
        decoders['front']._busy.acquire()
        try:self.parity(owned,self.sample(),2000,decoders)
        finally:decoders['front']._busy.release()

    def test_changed_codec_function_rejected_before_snapshot(self):
        decoders=self.decoders();function=self.lib.sda_feedback_decode_batch
        self.lib.sda_feedback_decode_batch=lambda *args:0
        try:
            with self.assertRaisesRegex(ValueError,'Exact GIL-releasing'):
                bench.snapshot_from_records({s:self.raw(s) for s in self.sessions},self.sample(),2000,feedback_decoders=decoders)
        finally:self.lib.sda_feedback_decode_batch=function

    def test_final_dynamic_gate_function_is_exact_frozen_r21(self):
        # Source equivalence is portable, unlike hashing interpreter-version AST.
        import inspect
        source=inspect.getsource(bench._verify_voltage_final_freshness)
        self.assertIn('_check_feedback_proof',source);self.assertIn('_check_voltage_proof',source)
        self.assertEqual(hashlib.sha256(source.encode()).hexdigest(),'b88c96ca3f003557fa9fcda425fa48db3fb122157692affbb60c60a8026eb74e')
        self.assertIn('row.start_ns',source);self.assertIn('row.received_ns',source)
        self.assertNotIn('feedback_decoders',source)

    def test_source_provenance_pins_selected_helper_on_both_boundaries(self):
        p=bench._start_source_provenance(profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,'synthetic-power',
             native_target_fk_cache=True,native_feedback_batch_decode=True)
        self.assertTrue(p['native_feedback_batch_decode'])
        self.assertIn(codec.SOURCE_PATH,p['cadence_source_sha256'])
        report={'status':'COMPLETE_DIAGNOSTIC','errors':[]}
        bench._finish_source_provenance(report,p)
        self.assertTrue(p['source_files_unchanged']);self.assertEqual(report['errors'],[])
        p['cadence_source_sha256'][codec.SOURCE_PATH]='0'*64
        bench._finish_source_provenance(report,p)
        self.assertEqual(report['status'],'ABORTED');self.assertFalse(p['source_files_unchanged'])

    def test_nonbool_or_inactive_collector_selection_rejected(self):
        for value in (None,1,'true'):
            with self.assertRaisesRegex(ValueError,'boolean'):
                bench.collect({},None,None,mode='type17',cycles=1,native_feedback_batch_decode=value)
        with self.assertRaisesRegex(ValueError,'Inactive'):
            bench.collect({},None,None,mode='type17',cycles=1,native_feedback_codec_selection=self.selected)

    def test_selected_collector_rejects_generic_or_missing_unpaired_owners(self):
        with self.assertRaisesRegex(ValueError,'actual diagnostic BusWorkers output'):
            bench.collect({},None,None,mode='type17',cycles=1,native_feedback_batch_decode=True,
                native_feedback_codec_selection=self.selected)

    def test_cli_default_help_and_complete_selection_before_device(self):
        out=io.StringIO()
        with contextlib.redirect_stdout(out),self.assertRaises(SystemExit) as exit:
            bench.main(['--help'])
        self.assertEqual(exit.exception.code,0);self.assertIn('--native-feedback-batch-decode',out.getvalue())
        for args in (['--native-feedback-batch-decode'],
                     ['--native-feedback-codec-selection','unused']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as exit:
                bench.main(args)
            self.assertEqual(exit.exception.code,2)

    def test_selected_native_pair_or_split_rejected_before_source_open(self):
        for branch in ('split7',None):
            args=['--native-feedback-batch-decode','--native-feedback-codec-selection','unused',
                  '--native-feedback-codec-selection-sha256','0'*64]
            if branch is not None:args+=['--private-seven-request-experiment',branch,
                '--private-seven-request-library','unused','--private-seven-request-library-sha256','0'*64,
                '--private-seven-request-source-inventory','unused','--private-seven-request-source-inventory-sha256','0'*64]
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as exit:bench.main(args)
            self.assertEqual(exit.exception.code,2)

    def zero_sessions(self):
        for scope,old in tuple(self.sessions.items()):
            old.close();ids=fixtures.runtime.BUSES[scope]
            self.sessions[scope]=active.ActiveSession(self.lib,self.sockets[scope][0].fileno(),
                first_id=ids[0],cancel_fd=self.cancel_read,boot_fd=self.boot.fileno(),boot_id=self.boot_id,
                raw_lower_by_id={mid:-1. for mid in ids},raw_upper_by_id={mid:1. for mid in ids},
                kp_max_by_id={mid:0. for mid in ids},kd_max_by_id={mid:0. for mid in ids},
                gap_ns=900_000,window=3)

    def borrowed(self,pool,*,codec_on=True,notify=True):
        bench._prestart_workers(pool,lambda:None)
        return adapter.make_borrowed_output(pool,self.sessions,lambda:os.write(self.cancel_write,b'x'),
            clock=time.monotonic_ns,native_feedback_batch_decode=codec_on,
            native_feedback_codec_selection=self.selected if codec_on else None,
            unpaired_output_future_notifications=notify)

    def test_borrowed_pool_actual_runtime_output_keeps_original_three_owners_and_raw(self):
        self.zero_sessions();self.start_peers()
        pool=ThreadPoolExecutor(max_workers=3);workers=self.borrowed(pool)
        try:
            threads=set(pool._threads);trace=__import__('array').array('Q',[0])*15
            workers.set_dispatch_trace(trace,0,lambda:None)
            deadline=time.monotonic_ns()+20_000_000
            futures=workers.submit_decoded({s:tuple(bench._STOP_WIRES[mid] for mid in fixtures.runtime.BUSES[s])
                for s in self.sessions},deadline_ns=deadline,label='diagnostic_policy_output')
            waiter=active.make_owned_waiter(self.lib,self.cancel_read,spin_us=500)
            result=workers.collect_output(futures,deadline_ns=deadline,native_deadline_ns=deadline,
                deadline_wait=waiter,timing=fixtures.runtime._PendingCycleTiming())
            self.assertEqual(set(pool._threads),threads);self.assertEqual(len(threads),3)
            self.assertTrue(all(type(f) is Future for f in futures.values()))
            for scope,row in result.items():
                self.assertIs(row,futures[scope].result());raw,decoded,lastwrite,lastreply=row
                self.assertEqual(decoded,fixtures.runtime.decode_records(raw))
                self.assertEqual(len(raw[0]),6);self.assertTrue(all(r.received_ns<deadline for r in raw[0]))
                self.assertEqual(lastwrite,max(r.finish_ns for r in raw[0]));self.assertEqual(lastreply,max(r.received_ns for r in raw[0]))
                self.assertGreater(trace[5 if scope=='front' else 9],0)
            self.assertTrue(workers.evidence()['actual_busworkers_collect_output'])
            workers.close();self.assertFalse(pool._shutdown);self.assertEqual(pool.submit(lambda:123).result(timeout=1),123)
            self.assertTrue(all(session._handle for session in self.sessions.values()))
        finally:
            pool.shutdown(wait=True,cancel_futures=False)
            if not workers._closed:workers.close()

    def test_borrowed_both_switches_default_false_keep_original_output_parser(self):
        self.zero_sessions();self.start_peers()
        with ThreadPoolExecutor(max_workers=3) as pool:
            workers=self.borrowed(pool,codec_on=False,notify=False)
            self.assertIsNone(workers.native_feedback_decoders)
            deadline=time.monotonic_ns()+20_000_000
            originals=workers.submit_decoded(workers._stop_batches,deadline_ns=deadline,label='diagnostic')
            workers.collect_output(originals,deadline_ns=deadline)
            self.assertEqual(workers.output_notification_groups,0);self.assertEqual(workers.output_notification_waits,0)
            workers.close();self.assertFalse(pool._shutdown)

    def test_borrowed_rejects_positive_gain_sessions_and_type1_before_io(self):
        with ThreadPoolExecutor(max_workers=3) as pool:
            bench._prestart_workers(pool,lambda:None)
            with self.assertRaisesRegex(ValueError,'zero-gain'):
                adapter.make_borrowed_output(pool,self.sessions,lambda:None,clock=time.monotonic_ns)
        self.zero_sessions()
        with ThreadPoolExecutor(max_workers=3) as pool:
            workers=self.borrowed(pool)
            wires={s:tuple(active.encode_motion(mid,0.,0.,0.) for mid in fixtures.runtime.BUSES[s]) for s in self.sessions}
            with self.assertRaisesRegex(ValueError,'no Type1'):
                workers.submit_decoded(wires,deadline_ns=time.monotonic_ns()+20_000_000,label='not_allowed')
            with self.assertRaisesRegex(RuntimeError,'only exact decoded STOP'):workers.submit(workers._stop_batches)
            self.assertEqual(self.seen,{'front':[],'rear':[]});workers.close()

    def test_borrowed_foreign_readiness_function_rejected_before_owner_enqueue(self):
        self.zero_sessions()
        other=active.load_library(self.library_path,expected_sha256=fixtures.ref(self.library_path)['sha256'])
        original=self.lib.sda_wait_future_ready;self.lib.sda_wait_future_ready=other.sda_wait_future_ready
        try:
            with ThreadPoolExecutor(max_workers=3) as pool:
                bench._prestart_workers(pool,lambda:None)
                with self.assertRaisesRegex(ValueError,'function|source binding|authenticated'):
                    adapter.make_borrowed_output(pool,self.sessions,lambda:None,clock=time.monotonic_ns)
                self.assertEqual(self.seen,{'front':[],'rear':[]})
        finally:self.lib.sda_wait_future_ready=original

    def test_borrowed_foreign_future_cross_thread_and_pending_detach_rejected(self):
        self.zero_sessions()
        with ThreadPoolExecutor(max_workers=3) as pool:
            workers=self.borrowed(pool);gate=threading.Event()
            blockers=[pool.submit(gate.wait) for _ in range(3)]
            deadline=time.monotonic_ns()+20_000_000
            futures=workers.submit_decoded(workers._stop_batches,deadline_ns=deadline,label='diagnostic')
            with self.assertRaisesRegex(RuntimeError,'join all output'):workers.close()
            with self.assertRaisesRegex(RuntimeError,'Exact current original'):
                workers.collect_output({'front':Future(),'rear':Future()},deadline_ns=deadline)
            failures=[]
            thread=threading.Thread(target=lambda:self._cross_close(workers,failures));thread.start();thread.join()
            self.assertEqual(len(failures),1);self.assertIsInstance(failures[0],RuntimeError)
            os.write(self.cancel_write,b'x');gate.set()
            for f in (*blockers,*futures.values()):
                try:f.result(timeout=1)
                except BaseException:pass
            # Emergency STOPs queued by the actual runtime are authoritative.
            # Joining the root pool also joins STOPs scheduled during owner failure.
            pool.shutdown(wait=True,cancel_futures=False)
            workers.close();self.assertTrue(pool._shutdown)

    @staticmethod
    def _cross_close(workers,failures):
        try:workers.close()
        except BaseException as error:failures.append(error)

    def test_borrowed_plain_decode_failure_retains_exact_six_raw_before_queued_stop(self):
        self.zero_sessions();self.start_peers()
        pool=ThreadPoolExecutor(max_workers=3);workers=self.borrowed(pool)
        original=workers._decode_bus_records
        def reject(scope,result):
            if scope=='front':raise ValueError('Injected Python decode failure after complete raw')
            return original(scope,result)
        workers._decode_bus_records=reject;deadline=time.monotonic_ns()+20_000_000
        originals=workers.submit_decoded(workers._stop_batches,deadline_ns=deadline,label='diagnostic_policy_output')
        try:
            with self.assertRaisesRegex(ValueError,'Injected Python decode') as failure:
                workers.collect_output(originals,deadline_ns=deadline)
        finally:
            pool.shutdown(wait=True,cancel_futures=False)
        error=originals['front'].exception()
        self.assertIsInstance(error,ValueError);self.assertEqual(len(error.records),6)
        raw=next(row[1] for row in workers.journal if row[0]=='front' and row[3]=='diagnostic_policy_output')
        self.assertIs(error.records,raw[0]);self.assertIs(error.stats,raw[1])
        self.assertTrue(all(r.written==r.received==17 for r in error.records))
        self.assertTrue(all(f.done() for f in (workers.stop_futures or {}).values()))
        self.assertEqual(set(workers.stop_futures),set(self.sessions));workers.close()

    def test_second_cycle_pre_native_failure_never_attaches_previous_same_label_raw(self):
        self.zero_sessions();self.start_peers();pool=ThreadPoolExecutor(max_workers=3)
        workers=self.borrowed(pool,notify=False);deadline=time.monotonic_ns()+20_000_000
        originals=workers.submit_decoded(workers._stop_batches,deadline_ns=deadline,label='diagnostic_policy_output')
        workers.collect_output(originals,deadline_ns=deadline)
        def reject():raise ValueError('Current second-cycle guard rejected before native I/O')
        trace=__import__('array').array('Q',[0])*15;workers.set_dispatch_trace(trace,0,reject)
        second=workers.submit_decoded(workers._stop_batches,deadline_ns=time.monotonic_ns()+20_000_000,
            label='diagnostic_policy_output')
        pool.shutdown(wait=True,cancel_futures=False)
        failures=[f.exception() for f in second.values()]
        self.assertTrue(any('second-cycle guard' in str(e) for e in failures))
        self.assertTrue(all(not hasattr(e,'records') for e in failures))
        self.assertEqual(sum(1 for row in workers.journal if row[3]=='diagnostic_policy_output'),2)
        self.assertTrue(all(f.done() for f in workers.stop_futures.values()));workers.close()

    def test_combined_notification_timing_cannot_admit_codec_only_live_selector(self):
        # This synthetic software fixture exercises only the proof-setting
        # boundary. It is never exported as an actual review/admission artifact.
        p=fixtures.SourceAdmissionTests().profile();p['cadence_source_sha256']=profiles.cadence_source_hashes(p)
        checks=('numeric_double_bits_exact','raw_bytes_order_timestamps_exact',
            'mode_fault_and_voltage_guards_unchanged','invalid_and_mixed_legacy_fallback',
            'partial_or_changed_abi_rejected','genuine_two_bus_original_futures',
            'prepared_voltage_deadline_cancellation','full_output_then_all12_raw_stop',
            'source_owner_and_busy_rejections')
        validation=dict(schema='singularitydog.unpaired-native-feedback-codec-source-validation.v1',
            status='PASS_CODEC_SOCKET_AND_SAVED_PARITY',source_sha256=p['cadence_source_sha256'],errors=[],skips=0,
            tests_passed=20,saved_transactions_compared=1000,hardware_opened=False,output_allowed=False,
            approved_for_runtime=False,whole_loop_timing_qualified=False,checks={k:True for k in checks},
            selection=self.selected,test_output=fixtures.ref(__file__),review=dict(reviewer='synthetic unit fixture',
            reviewed_at='2026-10-08T00:00:00Z',decision='ACCEPT_SOURCE_BOUND_UNPAIRED_FEEDBACK_CODEC',rationale='fixture only'))
        proof=dict(enabled=True,selected_buses=['front','rear'],scope='ordinary_unpaired_owners.v1',
            source_binding=dict(schema=codec.PROOF_SCHEMA,references=codec.verify_source_selection(self.selected),
            feedback_abi=1,active_abi=1,adds_owner_or_future_or_request=False,
            timestamps_and_deadlines_unchanged=True,unsupported_or_invalid_uses_legacy_codec=True))
        owner=dict(schema='singularitydog.diagnostic-runtime-output-owner.v1',scope='disabled_stop_proxy_only',
            actual_busworkers_submit_decoded=True,actual_busworkers_collect_output=True,
            genuine_original_output_futures=True,native_feedback_batch_decode_selected=True,
            original_absolute_deadlines_unchanged=True,collector_worker_count=3,extra_worker_or_reader_count=0,
            borrowed_executor_owner='original_collector',owns_or_closes_borrowed_executor=False,type1_sent=False,
            active_controller_qualification=False,output_allowed=False,approved_for_runtime=False,
            output_future_notifications_selected=True)
        docs={'native_feedback_codec_source_validation':validation,'pipeline_diagnostic':dict(
            native_feedback_batch_decode=proof,cadence_source_sha256=p['cadence_source_sha256'],
            native_phase_pair={'enabled':False},diagnostic_runtime_output=owner)}
        with self.assertRaisesRegex(profiles.ProfileError,'Own actual runtime'):
            profiles._unpaired_native_feedback_codec_evidence(docs,p)
        owner['output_future_notifications_selected']=False
        profiles._unpaired_native_feedback_codec_evidence(docs,p)
        self.assertEqual(p['_unpaired_native_feedback_codec_selection'],self.selected)

    def test_borrowed_native_fault_preserves_partial_raw_and_separate_stop_evidence(self):
        self.zero_sessions();self.behavior['front']=lambda wire:fixtures.reply(wire,fault=1)
        self.start_peers();pool=ThreadPoolExecutor(max_workers=3);workers=self.borrowed(pool)
        deadline=time.monotonic_ns()+20_000_000
        originals=workers.submit_decoded(workers._stop_batches,deadline_ns=deadline,label='diagnostic_policy_output')
        try:
            with self.assertRaisesRegex(Exception,'fault|cancel|aborted'):
                workers.collect_output(originals,deadline_ns=deadline)
        finally:pool.shutdown(wait=True,cancel_futures=False)
        error=originals['front'].exception()
        self.assertTrue(hasattr(error,'records'));self.assertEqual(len(error.records),6)
        self.assertTrue(any(r.received==17 for r in error.records))
        self.assertTrue(all(f.done() for f in (workers.stop_futures or {}).values()))
        workers.close()

if __name__=='__main__':unittest.main()
