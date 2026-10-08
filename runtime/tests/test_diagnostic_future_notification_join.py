"""Diagnostic join hints: original Futures, anonymous pipes and local sockets only."""
from concurrent.futures import Future
import contextlib
import ctypes as C
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import native_pipeline_benchmark as bench
import test_disabled_native_pair_candidate as candidate_fixture


class DiagnosticNotificationCausalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory=tempfile.TemporaryDirectory()
        base=Path(cls.directory.name)
        source=Path(active.__file__).resolve().parents[1]/'experiments/native_active_transport'
        for name in ('transport.cpp','build.py'):shutil.copyfile(source/name,base/name)
        spec=importlib.util.spec_from_file_location('build_diagnostic_notifications',base/'build.py')
        builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
        cls.library_path=builder.build()
        cls.library=active.load_library(cls.library_path,
            expected_sha256=hashlib.sha256(cls.library_path.read_bytes()).hexdigest())
    @classmethod
    def tearDownClass(cls):cls.directory.cleanup()
    def setUp(self):
        self.cancel=os.pipe()
        for fd in self.cancel:os.set_blocking(fd,False)
        self.waiter=active.make_owned_waiter(self.library,self.cancel[0],spin_us=500)
        # This test's clock scheduler owns only an anonymous cancellation pipe.
        self.legacy=lambda target:active.wait_until(self.library,self.cancel[0],target,spin_us=500)
    def tearDown(self):
        if self.waiter._readiness_group is not None:self.waiter._readiness_group.close()
        for fd in self.cancel:os.close(fd)
    def call(self,futures,extra=None,*,deadline_ns=None,**kwargs):
        return bench._await_owned_ready(futures,extra,phase='Acquisition',
            deadline_ns=deadline_ns or time.monotonic_ns()+20_000_000,
            deadline_wait=self.legacy,native_readiness_waiter=self.waiter,**kwargs)
    def later(self,function):
        def execute():
            active.wait_until(self.library,self.cancel[0],time.monotonic_ns()+1_000_000,spin_us=200)
            function()
        thread=threading.Thread(target=execute);thread.start();self.addCleanup(thread.join);return thread
    def test_original_owner_error_wins_over_pending_peer(self):
        front,rear=Future(),Future();error=RuntimeError('original owner error');front.set_exception(error)
        with self.assertRaises(RuntimeError) as caught:self.call({'front':front,'rear':rear})
        self.assertIs(caught.exception,error);self.assertFalse(rear.done())
    def test_original_owner_error_wins_over_simultaneous_cancel(self):
        front,rear=Future(),Future();error=RuntimeError('original owner error');front.set_exception(error)
        os.write(self.cancel[1],b'c')
        with self.assertRaises(RuntimeError) as caught:self.call({'front':front,'rear':rear})
        self.assertIs(caught.exception,error)
    def test_in_flight_owner_error_wins_over_native_cancel_wake(self):
        front,rear=Future(),Future();error=OSError('in-flight owner error')
        def fail():front.set_exception(error);os.write(self.cancel[1],b'c')
        thread=self.later(fail)
        try:
            with self.assertRaises(OSError) as caught:self.call({'front':front,'rear':rear})
        finally:thread.join()
        self.assertIs(caught.exception,error);self.assertIsNone(self.waiter._readiness_group)
    def test_cancel_before_hint_never_certifies_original_readiness(self):
        os.write(self.cancel[1],b'c');front,rear=Future(),Future()
        with self.assertRaises(active.ActiveWaitError):self.call({'front':front,'rear':rear})
        self.assertIsNone(self.waiter._readiness_group);self.assertFalse(front.done());self.assertFalse(rear.done())
    def test_hint_alone_keeps_both_originals_pending_until_original_deadline(self):
        front,rear=Future(),Future();create=self.waiter.readiness_group
        def hint_only(owners):
            group=create(owners);os.write(group._fds[1],b'h');return group
        with patch.object(self.waiter,'readiness_group',side_effect=hint_only):
            with self.assertRaises(TimeoutError) as caught:
                self.call({'front':front,'rear':rear},deadline_ns=time.monotonic_ns()+2_000_000)
        self.assertFalse(front.done());self.assertFalse(rear.done())
        proof=caught.exception.readiness_poll_failure
        self.assertEqual(proof['native_notification_wait_calls'],2)
        self.assertIsNone(proof['native_tick_max_us'])
    def test_absolute_deadline_is_preserved_in_failure_proof(self):
        front,rear=Future(),Future();deadline=time.monotonic_ns()+1_000_000
        with self.assertRaises(TimeoutError) as caught:self.call({'front':front,'rear':rear},deadline_ns=deadline)
        self.assertGreaterEqual(time.monotonic_ns(),deadline)
        self.assertEqual(caught.exception.readiness_poll_failure['deadline_ns'],deadline)
        self.assertIsNone(self.waiter._readiness_group)
    def test_cleanup_duration_counts_against_original_deadline(self):
        front,rear=Future(),Future();create=self.waiter.readiness_group
        def provider(owners):
            group=create(owners);original_close=group.close
            def close_late():
                active.wait_until(self.library,self.cancel[0],time.monotonic_ns()+4_000_000,spin_us=200)
                original_close()
            group.close=close_late;return group
        thread=self.later(lambda:(front.set_result('front'),rear.set_result('rear')))
        deadline=time.monotonic_ns()+3_000_000
        try:
            with patch.object(self.waiter,'readiness_group',side_effect=provider):
                with self.assertRaises(TimeoutError) as caught:self.call({'front':front,'rear':rear},deadline_ns=deadline)
        finally:thread.join()
        proof=caught.exception.readiness_poll_failure
        self.assertEqual(proof['stage'],'completion_clock');self.assertGreaterEqual(proof['decision_ns'],deadline)
    def test_already_ready_avoids_pipe_retains_original_values_and_truthful_mode(self):
        originals={scope:Future() for scope in ('front','rear')};imu=Future();value=object()
        for future in (*originals.values(),imu):future.set_result(value)
        with patch.object(self.waiter,'readiness_group',side_effect=AssertionError('unexpected pipe')):
            proof=self.call(originals,imu)
        self.assertEqual(proof['mode'],'native_future_notification_v1')
        self.assertTrue(proof['native_future_notification_selected'])
        self.assertFalse(proof['native_notification_group_created']);self.assertEqual(proof['wait_calls'],0)
        self.assertIsNone(proof['native_tick_max_us'])
        self.assertTrue(all(f.result() is value for f in (*originals.values(),imu)))
    def test_explicit_group_binding_failure_cannot_fall_back_to_poll(self):
        with patch.object(self.waiter,'readiness_group',return_value=None),patch.object(self,'legacy',side_effect=AssertionError('fallback')):
            with self.assertRaisesRegex(ValueError,'could not bind'):
                self.call({scope:Future() for scope in ('front','rear')})
    def test_imu_and_validation_future_are_part_of_exact_original_group(self):
        for join in (bench._await_acquisition_ready,bench._await_voltage_ready):
            with self.subTest(join=join.__name__):
                originals={scope:Future() for scope in ('front','rear')};extra=Future();value=object()
                for future in originals.values():future.set_result(value)
                thread=self.later(lambda:extra.set_result(value))
                create=self.waiter.readiness_group
                with patch.object(self.waiter,'readiness_group',wraps=create) as factory:
                    proof=join(originals,extra,deadline_ns=time.monotonic_ns()+20_000_000,
                        deadline_wait=self.legacy,native_readiness_waiter=self.waiter)
                thread.join();factory.assert_called_once_with((*originals.values(),extra))
                self.assertTrue(proof['native_notification_group_created']);self.assertEqual(proof['original_future_count'],3)
                self.assertIs(extra.result(),value);self.assertIsNone(self.waiter._readiness_group)
    def test_late_original_callback_ignores_closed_reused_pipe(self):
        pending=Future();group=self.waiter.readiness_group((pending,));old=group._fds;group.close()
        reused=os.pipe()
        try:
            self.assertEqual(reused,old)
            for fd in reused:os.set_blocking(fd,False)
            pending.set_result('late')
            with self.assertRaises(BlockingIOError):os.read(reused[0],1)
        finally:
            for fd in reused:os.close(fd)
    def test_future_subclass_is_not_eligible_for_explicit_notifications(self):
        class OtherFuture(Future):pass
        with self.assertRaisesRegex(ValueError,'original Futures'):
            self.call({'front':OtherFuture(),'rear':Future()})
    def test_changed_selected_capability_fails_even_for_ready_originals(self):
        originals={scope:Future() for scope in ('front','rear')}
        for future in originals.values():future.set_result('ready')
        with patch.object(self.library,'sda_wait_future_ready',None):
            with self.assertRaisesRegex(ValueError,'capability changed'):self.call(originals)
    def test_legacy_wait_proof_has_no_notification_fields(self):
        originals={scope:Future() for scope in ('front','rear')}
        for future in originals.values():future.set_result('ready')
        proof=bench._await_output_ready(originals,deadline_ns=time.monotonic_ns()+20_000_000,deadline_wait=self.legacy)
        self.assertEqual(proof['mode'],'native_readiness_poll_v1');self.assertEqual(proof['native_tick_max_us'],200)
        self.assertFalse(any('notification' in name for name in proof))
    def test_missing_partial_or_wrong_abi_library_is_rejected_before_group(self):
        self.assertTrue(bench._require_future_notification_library(self.library))
        for names in (('sda_future_readiness_abi',),('sda_wait_future_ready',),('sda_future_readiness_abi','sda_wait_future_ready')):
            with self.subTest(names=names),contextlib.ExitStack() as stack:
                for name in names:stack.enter_context(patch.object(self.library,name,None))
                with self.assertRaisesRegex(ValueError,'exact active ABI1'):
                    bench._require_future_notification_library(self.library)
        abi=self.library.sda_future_readiness_abi
        # ctypes signature attributes are descriptors; patch.object can restore
        # their class default instead of the selected ABI's explicit signature.
        original_restype=abi.restype
        try:
            abi.restype=C.c_int
            with self.assertRaises(ValueError):bench._require_future_notification_library(self.library)
        finally:abi.restype=original_restype


class DiagnosticNotificationScopeTests(unittest.TestCase):
    def test_flag_without_pair_fails_before_loading_or_opening_any_device(self):
        with patch.object(bench.native,'load_library',side_effect=AssertionError('unexpected load')) as load:
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as caught:
                bench.main(['--native-future-notification-joins'])
        self.assertEqual(caught.exception.code,2);load.assert_not_called()
    def test_default_plan_does_not_select_or_claim_notifications(self):
        output=io.StringIO()
        with contextlib.redirect_stdout(output):self.assertEqual(bench.main([]),0)
        plan=json.loads(output.getvalue());self.assertNotIn('native_future_notification_joins',plan)
    def test_selected_flag_cannot_bypass_pair_best20_scope(self):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            bench.main(['--native-future-notification-joins','--native-phase-pair'])
    def test_collect_requires_boolean_and_verified_pair(self):
        for value in (True,1,'yes'):
            with self.subTest(value=value),self.assertRaises(ValueError):
                bench.collect({},None,None,mode='stop-proxy',cycles=5,native_future_notification_joins=value)
    def test_counters_include_failed_original_join_and_keep_raw_metadata(self):
        okay={'native_future_notification_selected':True,'native_notification_group_created':True,
            'native_notification_wait_calls':2}
        failed={**okay,'native_notification_wait_calls':3}
        row={'voltage_fast_pipeline':{'acquisition_join_wait':okay,'voltage_join_failure_proof':failed,
            'output_join_failure_proof':{'readiness_poll_failure':failed}}}
        before=json.dumps(row,sort_keys=True)
        value=bench._notification_join_evidence([row])
        self.assertEqual(value['acquisition'],dict(attempts=1,groups_created=1,wait_calls=2,failed_joins=0))
        for phase in ('voltage','output'):
            self.assertEqual(value[phase],dict(attempts=1,groups_created=1,wait_calls=3,failed_joins=1))
        self.assertEqual(json.dumps(row,sort_keys=True),before)
    def test_late_takeout_failure_retains_successful_join_counter_evidence(self):
        joined={'native_future_notification_selected':True,'native_notification_group_created':True,
            'native_notification_wait_calls':2}
        row={'voltage_fast_pipeline':{'output_join_wait':joined,
            'output_join_failure_proof':{'stage':'result_takeout','reason':'original deadline exceeded'}}}
        value=bench._notification_join_evidence([row])
        self.assertEqual(value['output'],dict(attempts=1,groups_created=1,wait_calls=2,failed_joins=1))


class DiagnosticNotificationPairCollectionTests(unittest.TestCase):
    setUpClass=classmethod(candidate_fixture.DisabledNativePairCandidateTests.setUpClass.__func__)
    tearDownClass=classmethod(candidate_fixture.DisabledNativePairCandidateTests.tearDownClass.__func__)
    setUp=candidate_fixture.DisabledNativePairCandidateTests.setUp
    tearDown=candidate_fixture.DisabledNativePairCandidateTests.tearDown
    create=candidate_fixture.DisabledNativePairCandidateTests.create
    device=candidate_fixture.DisabledNativePairCandidateTests.device
    batches=candidate_fixture.DisabledNativePairCandidateTests.batches
    collect_candidate=candidate_fixture.DisabledNativePairCandidateTests.collect_candidate
    def test_selected_candidate_preserves_prepared_publication_trace_stop_and_native_join_proof(self):
        # Exercise the existing explicit prime outside the measured cycles:
        # this is a publication/trace contract test, not a cold-start timing
        # guarantee on the host OS. The actual 20 ms cycle deadlines stay exact.
        report,raw=self.collect_candidate(prime=True,
            extra_options={'native_future_notification_joins':True})
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['cycles_completed'],5)
        for field in ('input_acquisition_wait','voltage_join_wait','output_join_wait'):
            self.assertEqual(report[field],'native_future_notification.v1')
        proof=report['native_future_notification_joins'];self.assertTrue(proof['enabled']);self.assertEqual(proof['abi'],1)
        self.assertFalse(proof['type1_speedup_verified']);self.assertFalse(proof['source_timestamps_changed'])
        for phase in ('acquisition','voltage','output'):self.assertEqual(proof['joins'][phase]['attempts'],5)
        self.assertTrue(report['native_phase_pair_proof']['all_phases_joined'])
        self.assertFalse(report['motor_enable_sent']);self.assertFalse(report['learned_targets_sent'])
        for row in raw:
            serialized=row.serialize();pipeline=serialized['voltage_fast_pipeline']
            self.assertEqual(pipeline['stop_reply_count'],12)
            self.assertEqual(pipeline['acquisition_join_wait']['original_future_count'],3)
            for phase in ('acquisition','voltage','output'):
                join=pipeline[phase+'_join_wait'];self.assertIsNone(join['native_tick_max_us'])
    def test_waiter_retains_exact_selected_active_library_and_cancel_descriptor(self):
        candidate=self.create();waiter=candidate.future_readiness_waiter()
        self.assertIs(waiter._library,candidate._DisabledNativePairCandidate__library)
        self.assertEqual(waiter._cancel_fd,self.cancel_read)
        self.assertTrue(waiter.future_readiness_available)
        candidate.close()
        with self.assertRaisesRegex(RuntimeError,'closed'):candidate.future_readiness_waiter()


class DiagnosticNotificationPlanTests(unittest.TestCase):
    setUp=candidate_fixture.DisabledNativePairCLITests.setUp
    tearDown=candidate_fixture.DisabledNativePairCLITests.tearDown
    invoke=candidate_fixture.DisabledNativePairCLITests.invoke
    replace=candidate_fixture.DisabledNativePairCLITests.replace
    def test_file_only_plan_selects_flag_without_manufacturing_abi_or_runtime_qualification(self):
        with patch.object(active,'load_library',side_effect=AssertionError('PLAN must not load')):
            result,plan=self.invoke(self.argv+['--native-future-notification-joins'])
        self.assertEqual(result,0);self.assertTrue(plan['native_future_notification_joins'])
        self.assertTrue(plan['native_phase_pair']);self.assertFalse(plan['output_allowed'])
        self.assertNotIn('native_phase_pair_proof',plan);self.assertEqual(plan['type1_requests_per_cycle'],0)


if __name__=='__main__':unittest.main()
