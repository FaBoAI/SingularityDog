"""Offline genuine C++/socket ownership tests; OS controls explicitly mocked.

Collector tests inject a tiny observer to exercise scheduling/failure paths.
They do not qualify learned-model inference or physical hardware timing. The
target's separate saved501 replay uses the unchanged pinned real FK model.
"""
import contextlib, dataclasses, hashlib, importlib.util, json, os, pathlib, socket, sys, threading, time, unittest
from concurrent.futures import Future,ThreadPoolExecutor
from unittest.mock import patch
from singularitydog_hw import private_seven_request_bridge as bridge
from singularitydog_hw import private_seven_request_candidate as candidate
from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import thread_timer_slack
from test_native_pipeline_benchmark import Device,Observer

ROOT=pathlib.Path(bridge.__file__).parents[1]/'experiments/private_seven_request'
sys.modules['seven_request_candidate']=candidate
spec=importlib.util.spec_from_file_location('private_native_fixture',ROOT/'original_native_tests.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)

class FakePrctl:
    def __init__(self):self.values={}
    def get(self):return self.values.get(threading.get_native_id(),1000)
    def set(self,value):self.values[threading.get_native_id()]=value

class NativeBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixture.SplitTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixture.SplitTests.tearDownClass()
    def setUp(self):
        self.f=fixture.SplitTests('test_cpp_generation_mismatch_before_any_writes');self.f.lib=fixture.SplitTests.lib
        self.f._testMethodName=self._testMethodName;self.f.setUp()
        # The reusable fixture reserves front FD during setup. Release its
        # unused session before testing the runtime's genuine exclusive owner.
        self.f.session.close()
        self.rear,self.rear_peer=socket.socketpair();self.rear.setblocking(False);self.rear_peer.setblocking(False)
        self.stack=contextlib.ExitStack();self.masks={}
        def get_affinity(_):return set(self.masks.get(threading.get_native_id(),{0,1,2,3,4}))
        def set_affinity(_,mask):self.masks[threading.get_native_id()]=set(mask)
        self.stack.enter_context(patch.object(os,'sched_getaffinity',side_effect=get_affinity,create=True))
        self.stack.enter_context(patch.object(os,'sched_setaffinity',side_effect=set_affinity,create=True))
        self.stack.enter_context(patch.object(thread_timer_slack,'require_supported_platform'))
        self.stack.enter_context(patch.object(thread_timer_slack,'_load_prctl',return_value=FakePrctl()))
        self.timer=self.stack.enter_context(thread_timer_slack.TimerSlack(1000));self.runtime=None;self.original_pool=None
    def tearDown(self):
        try:
            if self.runtime is not None and not self.runtime.closed:
                try:os.write(self.f.cancel_write,b'x')
                except OSError:pass
                self.runtime.close()
        finally:
            
            if self.original_pool is not None:self.original_pool.shutdown(wait=True)
            self.stack.close();self.f.tearDown();self.rear.close();self.rear_peer.close()
    def runtime_for(self,mode):
        self.runtime=bridge.UnpairedRuntime(mode=mode,library=self.f.lib,
            fd_by_scope={'front':self.f.host.fileno(),'rear':self.rear.fileno()},
            boot_fd_by_scope={'front':self.f.boot.fileno(),'rear':self.f.boot.fileno()},
            cancel_fd=self.f.cancel_read,boot_id=fixture.BOOT,
            source_refs={str(ROOT/'transport.cpp'):bridge.CORE_SHA})
        return self.runtime
    def start(self,mode='split7'):
        runtime=self.runtime_for(mode);self.original_pool=ThreadPoolExecutor(max_workers=3,initializer=self.timer.worker_initializer)
        bench._prestart_workers(self.original_pool,lambda:None);runtime.start_workers(self.original_pool,self.timer.worker_initializer)
        runtime.configure_owners({0,1,2,3,4},4);runtime.begin_cycle(time.monotonic_ns()+20_000_000)
        return runtime
    def proof(self):return {**{n+'_ns_by_bus':{} for n in ('feedback_dispatch','feedback_reply_end','feedback_ready','feedback_published','voltage_dispatch','voltage_reply_end')},'private_seven_request':{'buses':{}}}
    def split(self,runtime,proof=None):
        proof=self.proof() if proof is None else proof;feedback={s:Future() for s in bridge.SCOPES}
        complete=runtime.start_split({'front':1,'rear':7},feedback,proof,0)
        return proof,feedback,complete
    def test_genuine_two_bus_prefix_then_original_full_main_close_fence(self):
        runtime=self.start();self.f.peer_loop(delay_ns=100_000,delays={6:4_000_000});self.f.peer_loop(io=self.rear_peer.fileno(),delay_ns=100_000,delays={6:4_000_000})
        proof,feedback,complete=self.split(runtime);runtime.pump_feedback(feedback)
        for s,future in complete.items():
            self.assertIs(future,runtime.phases[s].full_future);self.assertEqual(len(feedback[s].result()[0]),6)
            self.assertIs(runtime.phases[s]._caller,threading.current_thread());self.assertIs(runtime.phases[s].executor,self.original_pool)
        record={'voltage':{}};self.assertFalse(runtime.fd_release_safe());self.assertEqual(runtime.settle_voltage(complete,record),[])
        for s in bridge.SCOPES:
            row=proof['private_seven_request']['buses'][s];self.assertTrue(row['prefix_full_record_bits_exact']);self.assertTrue(row['closed_before_output'])
            self.assertEqual(row['prefix6_records_hex'],row['full7_records_hex'][:6]);self.assertFalse(row['prefix_native_owner_joined'])
            self.assertEqual(len(record['voltage'][s][0]),1);self.assertGreaterEqual(runtime.phases[s].full_stats.begin_ns,max(r['constructor_end_ns'] for r in proof['private_seven_request']['buses'].values()))
        self.assertTrue(runtime.fd_release_safe());self.assertEqual(runtime.closed_phase_count,2)
    def test_missing_seventh_preserves_prefix_full7_and_rejects_main_takeout(self):
        runtime=self.start();self.f.peer_loop(delay_ns=100_000,missing=(6,));self.f.peer_loop(io=self.rear_peer.fileno(),delay_ns=100_000)
        proof,feedback,complete=self.split(runtime);runtime.pump_feedback(feedback)
        self.assertEqual(len(feedback['front'].result()[0]),6);record={'voltage':{}};self.assertTrue(runtime.settle_voltage(complete,record))
        row=proof['private_seven_request']['buses']['front'];self.assertEqual(len(row['full7_records_hex']),7);self.assertTrue(row['closed_before_output']);self.assertFalse(row['full_owner_joined'])
        self.assertNotIn('prefix_full_record_bits_exact',row);self.assertTrue(runtime.fd_release_safe())
    def test_wrong_mode_never_publishes_healthy_collector_bridge(self):
        runtime=self.start();self.f.peer_loop(delay_ns=100_000,modes={2:1});self.f.peer_loop(io=self.rear_peer.fileno(),delay_ns=100_000)
        proof,feedback,complete=self.split(runtime)
        with self.assertRaises(Exception):runtime.pump_feedback(feedback)
        with self.assertRaises(Exception):feedback['front'].result()
        self.assertTrue(runtime.settle_voltage(complete,{'voltage':{}}));self.assertEqual(len(proof['private_seven_request']['buses']['front']['full7_records_hex']),7)
    def test_prefix_full_mutation_is_rejected(self):
        prefix=candidate.Prefix((b'a',)*6,b'',1,1,63)
        full=candidate.Full((b'a',)*5+(b'b',b'v'),b'',1,2,3,40.)
        with self.assertRaisesRegex(ValueError,'record bits'):bridge.verify_prefix_full(prefix,full)
    def test_no_type1_or_enable_api_on_disabled_adapter(self):
        runtime=self.start()
        with self.assertRaisesRegex(ValueError,'Type0/17/zero STOP'):runtime.sessions['front'].exchange((bytes(17),))
        self.assertFalse(hasattr(runtime.sessions['front'],'send_only'));self.assertFalse(hasattr(runtime.sessions['front'],'emergency_stop'))
    def test_close_failure_blocks_normal_stop_and_cycle_reuse(self):
        runtime=self.start();self.f.peer_loop(delay_ns=100_000);self.f.peer_loop(io=self.rear_peer.fileno(),delay_ns=100_000)
        proof,feedback,complete=self.split(runtime);runtime.pump_feedback(feedback)
        with patch.object(candidate.SevenRequestPhase,'close',side_effect=RuntimeError('injected close failure')):
            self.assertTrue(runtime.settle_voltage(complete,{'voltage':{}}))
        self.assertFalse(runtime.fd_release_safe())
        with self.assertRaisesRegex(ValueError,'reuse prohibited'):runtime.begin_cycle(time.monotonic_ns()+20_000_000)
        with self.assertRaisesRegex(ValueError,'must close before STOP'):runtime.sessions['front'].exchange((bench.native.stop_wire(1),))
        self.assertTrue(proof['private_seven_request']['buses']['front']['full_owner_joined'])
        # SAME original main caller retries cleanup; no foreign worker close.
        runtime.cleanup_phases();self.assertTrue(runtime.fd_release_safe())
    def test_source_mutation_is_rejected_before_new_cycle(self):
        runtime=self.start()
        with patch.object(pathlib.Path,'read_bytes',return_value=b'changed'):
            with self.assertRaisesRegex(ValueError,'source/build pin'):runtime.verify_sources()
    def test_completed_native_raw_survives_late_python_takeout(self):
        rows=(bench.native.Record*1)();stats=bench.native.Stats();future=Future();future.set_result((rows,stats))
        with self.assertRaises(bench.native.ExchangeError) as caught:bridge.owned_result(future,time.monotonic_ns()-1)
        self.assertIs(caught.exception.records,rows);self.assertIs(caught.exception.stats,stats)
    def test_native_raw_survives_original_direct_outer_takeout_deadline(self):
        runtime=self.start('baseline6plus1');self.f.peer_loop(delay_ns=100_000,total=6)
        original=runtime.native_sessions['front'].exchange;saved=[]
        def late_publication(*args,**kwargs):
            value=original(*args,**kwargs);saved.append(value);time.sleep(.03);return value
        with patch.object(runtime.native_sessions['front'],'exchange',side_effect=late_publication),self.assertRaises(bench.native.ExchangeError) as caught:
            runtime.sessions['front'].exchange(tuple(bench.native.stop_wire(i) for i in range(1,7)))
        self.assertEqual(len(caught.exception.records),6);self.assertEqual([bridge.image(r) for r in caught.exception.records],[bridge.image(r) for r in saved[0][0]])
        self.assertTrue(runtime.fd_release_safe())
    def test_missing_seventh_collector_keeps_fullraw_and_sends_no_cycle_output(self):
        runtime=self.runtime_for('split7')
        # Six administrative prime requests precede measured seven inputs.
        self.f.peer_loop(delay_ns=100_000,total=13,missing=(12,));self.f.peer_loop(io=self.rear_peer.fileno(),delay_ns=100_000,total=13)
        report,raw=bench.collect(runtime.sessions,Device(),Observer(),mode='stop-proxy',cycles=5,
            private_seven_request_runtime=runtime,record_storage='trace',main_thread_cpu=4,
            worker_initializer=self.timer.worker_initializer,output_dispatch_trace=True,defer_gc_during_cycles=True,
            pre_cycle_policy_prepare=lambda:None,post_pin_policy_prepare=lambda:None,
            v3_voltage_proxy=True,v3_voltage_overlap=True,v3_voltage_validation_overlap=True,
            v3_voltage_fast_pipeline=True,prepare_voltage_before_feedback_publication=True,
            inference_thread_cpu_trace=True,absolute_epoch_cadence=True,exclude_policy_cpu_from_workers=True,
            startup_cycle_allowance=1,deadline_wait=lambda deadline:time.sleep(max(0.,(deadline-time.monotonic_ns())/1e9)))
        self.assertEqual(report['status'],'ABORTED');self.assertEqual(report['cycles_completed'],0)
        rows=bench._serialize(raw);self.assertEqual(rows[0]['output'],{})
        self.assertEqual(len(rows[0]['voltage_fast_pipeline']['private_seven_request']['buses']['front']['full7_records_hex']),7)
        self.assertTrue(report['private_seven_request_experiment']['fd_release_safe'])
    def test_both_unpaired_branches_complete_five_collector_cycles_with_same_controls(self):
        # Timing is genuine native protocol over socket, inference is an injected
        # test observer. Actual model/IMU performance must be measured separately.
        for mode in bridge.MODES:
            with self.subTest(mode=mode):
                if self.runtime is not None:self.tearDown();self.setUp()
                runtime=self.runtime_for(mode)
                self.f.peer_loop(delay_ns=100_000,total=71);self.f.peer_loop(io=self.rear_peer.fileno(),delay_ns=100_000,total=71)
                def deadline_wait(deadline):
                    remaining=(deadline-time.monotonic_ns())/1e9
                    if remaining>0:time.sleep(remaining)
                report,raw=bench.collect(runtime.sessions,Device(),Observer(),mode='stop-proxy',cycles=5,
                    private_seven_request_runtime=runtime,record_storage='trace',main_thread_cpu=4,
                    worker_initializer=self.timer.worker_initializer,output_dispatch_trace=True,defer_gc_during_cycles=True,
                    pre_cycle_policy_prepare=lambda:None,post_pin_policy_prepare=lambda:None,
                    v3_voltage_proxy=True,v3_voltage_overlap=True,v3_voltage_validation_overlap=True,
                    v3_voltage_fast_pipeline=True,prepare_voltage_before_feedback_publication=True,
                    inference_thread_cpu_trace=True,absolute_epoch_cadence=True,exclude_policy_cpu_from_workers=True,
                    startup_cycle_allowance=1,deadline_wait=deadline_wait)
                self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors']);self.assertEqual(report['cycles_completed'],5)
                self.assertEqual(report['private_seven_request_experiment']['original_collector_worker_count'],3);self.assertEqual(report['private_seven_request_experiment']['extra_native_owner_worker_count'],0);self.assertTrue(report['worker_affinity']['restored'])
                self.assertTrue(self.timer.report['worker_verification_complete']);self.assertEqual(len(self.timer.report['workers']),3)
                rows=bench._serialize(raw);json.dumps(rows,allow_nan=False)
                self.assertEqual(sum(len(row[p][s]['records']) for row in rows for p in ('acquired','voltage','output') for s in bridge.SCOPES),130)
                if mode=='split7':
                    for i,row in enumerate(rows):
                        for s in bridge.SCOPES:
                            proof=row['voltage_fast_pipeline']['private_seven_request']['buses'][s]
                            self.assertTrue(proof['closed_before_output']);self.assertTrue(proof['prefix_full_record_bits_exact'])
                            self.assertLess(proof['close_end_ns'],min(r['start_ns'] for r in row['output'][s]['records']))
                            self.assertLess(proof['close_end_ns'],report['measurements'][i]['cycle_end_ns'])

class AdmissionTests(unittest.TestCase):
    def test_default_import_does_not_load_native_library(self):
        # Import above did not register/load the optional C++ library.
        self.assertNotIn('torch',sys.modules)
    def test_option_without_complete_source_library_proof_rejects_before_device(self):
        with patch.object(bench.native,'load_library') as lib,patch.object(bench.imu,'ICM20948') as imu,\
             contextlib.redirect_stderr(__import__('io').StringIO()),self.assertRaises(SystemExit):
            bench.main(['--private-seven-request-experiment','split7'])
        lib.assert_not_called();imu.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
