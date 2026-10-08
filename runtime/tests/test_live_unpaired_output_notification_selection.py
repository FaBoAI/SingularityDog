"""Pure admission and genuine anonymous sockets; no physical device or approval."""
from concurrent.futures import Future
import copy
from contextlib import redirect_stdout,redirect_stderr
import ctypes as C
import hashlib
import io
import math
import os
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import policy_live_profile as profiles
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import unpaired_output_future_notifications as notification
from singularitydog_hw.native_diagnostic_transport import stop_wire
import test_unpaired_native_feedback_codec_integration as codec_fixture
from test_unpaired_native_feedback_codec_integration import reply,ref


def profile(selected=True):
    p=profiles.template(schema=profiles.SCHEMA_V3)
    for mid,partial in codec_fixture.SourceAdmissionTests().profile()['axes'].items():p['axes'][mid].update(partial)
    p.update({k:v for k,v in codec_fixture.SourceAdmissionTests().profile().items() if k!='axes'})
    p.update(native_feedback_batch_decode=False,unpaired_output_future_notifications=selected,
             period_ms=20,max_sample_gap_ms=21,command=[0.,0.,0.],voltage_min_v=35.,voltage_max_v=42.,
             startup_duration_s=.4,policy_ramp_s=.4,stop_duration_s=.4,h_hypothesis=0.)
    p['cadence_source_sha256']=profiles.cadence_source_hashes(p)
    p['artifacts']={name:{'path':None,'sha256':None} for name in profiles.artifact_names(p)}
    return p


class ProfileNotificationTests(unittest.TestCase):
    def test_old_absent_and_explicit_false_retain_default(self):
        self.assertFalse(profiles.unpaired_output_future_notifications_settings({}))
        self.assertFalse(profiles.unpaired_output_future_notifications_settings({'unpaired_output_future_notifications':False}))
    def test_types_legacy_pair_duration_and_caps_rejected(self):
        for value in (1,None,'yes'):
            p=profile();p['unpaired_output_future_notifications']=value
            with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings(p,require_approved=False)
        for key,value in (('schema',profiles.SCHEMA_V2),('native_phase_pair',True),('duration_s',20.),('hard_cycle_ms',21),('request_gap_us',890),('max_sample_age_ms',21),('voltage_max_v',43.)):
            p=profile();p[key]=value
            with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings(p,require_approved=False)
        for mid in profiles.IDS:
            p=profile();p['axes'][mid]['kp']=3.01
            with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings(p,require_approved=False)
    def test_candidate_is_plan_only_and_missing_real_evidence_rejects(self):
        p=profile();self.assertTrue(profiles.unpaired_output_future_notifications_settings(p,require_approved=False))
        with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings(p)
        with self.assertRaisesRegex(profiles.ProfileError,'Own-source'):
            profiles._unpaired_output_future_notifications_evidence({'pipeline_diagnostic':{}},p)
    def test_new_artifact_and_actual_source_are_explicitly_bound(self):
        p=profile();profiles._structure(p);profiles._settings(p)
        self.assertIn('unpaired_output_notification_source_validation',p['artifacts'])
        self.assertIn(notification.SOURCE_PATH,p['cadence_source_sha256'])
        old=profiles.reviewed_settings_sha256(p);p['unpaired_output_future_notifications']=False
        p['cadence_source_sha256']=profiles.cadence_source_hashes(p)
        self.assertNotEqual(old,profiles.reviewed_settings_sha256(p))
    def test_seal_rejects_false_true_and_source_epoch_cap_mutations(self):
        p=profile();p.update(output_allowed=True,_unpaired_output_notification_token=profiles._UNPAIRED_OUTPUT_NOTIFICATION_TOKEN,
                            _unpaired_output_notification_selection={'negative_fixture':True})
        # Internal token fixture only; no serialized approval, qualification or device action.
        p['_unpaired_output_notification_binding']=profiles._unpaired_output_future_notifications_binding(p)
        self.assertTrue(profiles.unpaired_output_future_notifications_settings(p))
        for key,value in (('unpaired_output_future_notifications',False),('motor_power_epoch','different'),('boot_id','different')):
            changed=copy.deepcopy(p);changed['_unpaired_output_notification_token']=profiles._UNPAIRED_OUTPUT_NOTIFICATION_TOKEN;changed[key]=value
            with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings(changed)
        changed=copy.deepcopy(p);changed['_unpaired_output_notification_token']=profiles._UNPAIRED_OUTPUT_NOTIFICATION_TOKEN
        changed['cadence_source_sha256']['singularitydog_hw/policy_output_runtime.py']='0'*64
        with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings(changed)
    def test_prepared_binding_changes_when_notification_selection_changes(self):
        p=profile();old=profiles._prepared_voltage_publication_binding(p)
        p['unpaired_output_future_notifications']=False
        p['cadence_source_sha256']=profiles.cadence_source_hashes(p)
        self.assertNotEqual(old,profiles._prepared_voltage_publication_binding(p))
    def test_foreign_or_inactive_private_tokens_are_rejected(self):
        for name in ('_unpaired_output_notification_token','_unpaired_output_notification_binding','_unpaired_output_notification_selection'):
            with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings({name:object()})
        p=profile();p.update(output_allowed=True,_unpaired_output_notification_token=profiles._NATIVE_PHASE_PAIR_TOKEN)
        with self.assertRaises(profiles.ProfileError):profiles.unpaired_output_future_notifications_settings(p)
    def test_diagnostic_notification_must_match_profile_both_directions(self):
        for selected in (True,False):
            p=profile(selected)
            profiles._unpaired_output_notification_diagnostic_selection({'diagnostic_runtime_output':{'output_future_notifications_selected':selected}},p)
            with self.assertRaises(profiles.ProfileError):profiles._unpaired_output_notification_diagnostic_selection({'diagnostic_runtime_output':{'output_future_notifications_selected':not selected}},p)
        profiles._unpaired_output_notification_diagnostic_selection({},profile(False))
        with self.assertRaises(profiles.ProfileError):profiles._unpaired_output_notification_diagnostic_selection({},profile(True))


class CLINotificationTests(unittest.TestCase):
    def test_help_exposes_only_explicit_new_flag(self):
        from singularitydog_hw import policy_output as cli
        text=io.StringIO()
        with redirect_stdout(text),self.assertRaises(SystemExit) as e:cli.main(['--help'])
        self.assertEqual(e.exception.code,0);self.assertIn('--unpaired-output-future-notifications',text.getvalue())
    def test_plan_true_does_not_load_library_open_io_or_authorize(self):
        from singularitydog_hw import policy_output as cli
        p=profile();p.update(output_allowed=False,profile_sha256='0'*64);text=io.StringIO()
        with patch.object(cli,'load_profile',return_value=p),patch.object(active,'load_library',side_effect=AssertionError('native load')),redirect_stdout(text):
            self.assertEqual(cli.main(['--profile','unused','--unpaired-output-future-notifications']),0)
        self.assertIn('"unpaired_output_future_notifications": true',text.getvalue());self.assertIn('"hardware_opened": false',text.getvalue())
    def test_cli_mismatch_both_directions_rejected_before_native_load(self):
        from singularitydog_hw import policy_output as cli
        for selected in (False,True):
            p=profile(selected);p.update(output_allowed=False,profile_sha256='0'*64)
            args=['--profile','unused']+([] if selected else ['--unpaired-output-future-notifications'])
            with patch.object(cli,'load_profile',return_value=p),patch.object(active,'load_library',side_effect=AssertionError('native load')),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as e:cli.main(args)
            self.assertEqual(e.exception.code,2)
    def test_fake_active_profile_cannot_bypass_loader_token(self):
        from singularitydog_hw import policy_output as cli
        p=profile();p.update(output_allowed=True,profile_sha256='0'*64)
        with patch.object(cli,'load_profile',return_value=p),patch.object(active,'load_library',side_effect=AssertionError('native load')),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as e:
            cli.main(['--profile','unused','--execute-supported','--unpaired-output-future-notifications'])
        self.assertEqual(e.exception.code,2)
    def test_runtime_keyword_mismatch_rejected_before_workers_or_enable(self):
        p=profile();p['output_allowed']=True
        with patch.object(runtime,'BusWorkers',side_effect=AssertionError('workers')),self.assertRaises(ValueError):
            runtime.run_supported_policy(p,{},lambda:None,lambda _:None,cancel_io=lambda:None,
                unpaired_output_future_notifications=True)


class GenuineNotificationTests(unittest.TestCase):
    setUpClass=classmethod(codec_fixture.TwoBusCodecTests.setUpClass.__func__)
    tearDownClass=classmethod(codec_fixture.TwoBusCodecTests.tearDownClass.__func__)
    setUp=codec_fixture.TwoBusCodecTests.setUp
    tearDown=codec_fixture.TwoBusCodecTests.tearDown
    start_peers=codec_fixture.TwoBusCodecTests.start_peers
    def setup_notification(self):
        self.waiter=active.make_owned_waiter(self.lib,self.cancel_read,spin_us=500)
        self.selection=notification.selection_for_authenticated_library(self.lib)
        return notification.prepare_notifications(self.sessions,self.waiter,self.selection)
    def workers_selected(self):
        self.setup_notification();self.start_peers()
        self.workers=runtime.BusWorkers(self.sessions,lambda:None,unpaired_output_future_notifications=True)
        return self.workers
    def stop_batch(self):return {s:tuple(stop_wire(mid) for mid in ids) for s,ids in runtime.BUSES.items()}
    def test_actual_source_bound_two_bus_original_futures_complete_raw_stop(self):
        proof=self.setup_notification();self.assertEqual(proof['future_readiness_abi'],1);self.assertFalse(proof['adds_owner_or_future_or_request'])
        self.start_peers();self.workers=runtime.BusWorkers(self.sessions,lambda:None,unpaired_output_future_notifications=True)
        deadline=time.monotonic_ns()+20_000_000
        originals=self.workers.submit_decoded(self.stop_batch(),deadline_ns=deadline,label='genuine')
        result=self.workers.collect_output(originals,deadline_ns=deadline,deadline_wait=self.waiter)
        self.assertTrue(all(originals[s] is self.workers._output_notification_current_futures[s] for s in runtime.BUSES))
        for scope in runtime.BUSES:
            self.assertEqual(len(result[scope][0][0]),6);self.assertEqual(len(result[scope][1]),6)
        self.assertGreaterEqual(self.workers.output_notification_groups,1)
    def test_selected_collect_without_submit_rejects_foreign_futures(self):
        self.workers_selected();a=Future();b=Future();a.set_result(None);b.set_result(None)
        with self.assertRaisesRegex(RuntimeError,'current original'):
            self.workers.collect_output({'front':a,'rear':b},
                deadline_ns=time.monotonic_ns()+20_000_000,deadline_wait=self.waiter)
        self.assertIsNone(self.workers._output_notification_current_futures)
    def test_returned_mapping_substitution_cannot_replace_original_future_seal(self):
        self.workers_selected();deadline=time.monotonic_ns()+20_000_000
        originals=self.workers.submit_decoded(self.stop_batch(),deadline_ns=deadline,label='mapping')
        forged=Future();forged.set_result(None);originals['front']=forged
        with self.assertRaisesRegex(RuntimeError,'current original'):
            self.workers.collect_output(originals,deadline_ns=deadline,deadline_wait=self.waiter)
        self.assertIsNot(self.workers._output_notification_current_futures['front'],forged)
        with self.assertRaises(TypeError):self.workers._output_notification_current_futures['front']=forged
    def test_stale_generation_and_swapped_futures_rejected(self):
        self.workers_selected();deadline=time.monotonic_ns()+20_000_000
        first=self.workers.submit_decoded(self.stop_batch(),deadline_ns=deadline,label='first')
        self.workers.collect_output(first,deadline_ns=deadline,deadline_wait=self.waiter)
        deadline=time.monotonic_ns()+20_000_000
        second=self.workers.submit_decoded(self.stop_batch(),deadline_ns=deadline,label='second')
        with self.assertRaisesRegex(RuntimeError,'current original'):
            self.workers.collect_output(first,deadline_ns=deadline,deadline_wait=self.waiter)
        # Failure preserved; inherited emergency cancellation prevents further active reuse.
        for f in second.values():
            try:f.result(timeout=.5)
            except BaseException:pass
    def test_foreign_unpinned_cdll_and_changed_wait_symbol_rejected_before_setup(self):
        self.setup_notification();foreign=active.load_library(self.library_path)
        with self.assertRaises(ValueError):notification.verify_loaded_library(foreign,self.selection)
        original=self.lib.sda_wait_future_ready
        other=active.load_library(self.library_path,expected_sha256=ref(self.library_path)['sha256'])
        self.lib.sda_wait_future_ready=other.sda_wait_future_ready
        try:
            with self.assertRaisesRegex(ValueError,'capability changed|authenticated'):notification.prepare_notifications(self.sessions,self.waiter,self.selection)
        finally:self.lib.sda_wait_future_ready=original
    def test_busy_pair_cancel_or_cross_thread_binding_rejected(self):
        self.setup_notification();s=self.sessions['front'];s.busy.acquire()
        try:
            with self.assertRaisesRegex(ValueError,'idle'):notification.prepare_notifications(self.sessions,self.waiter,self.selection)
        finally:s.busy.release()
        old=s._cancel_fd;s._cancel_fd=self.cancel_write
        try:
            with self.assertRaisesRegex(ValueError,'cancellation'):notification.prepare_notifications(self.sessions,self.waiter,self.selection)
        finally:s._cancel_fd=old
        errors=[]
        def call():
            try:notification.prepare_notifications(self.sessions,self.waiter,self.selection)
            except BaseException as e:errors.append(e)
        t=threading.Thread(target=call);t.start();t.join();self.assertEqual(len(errors),1)
    def test_mutated_session_and_waiter_cancel_claims_do_not_replace_original_creation(self):
        self.setup_notification();read,write=os.pipe()
        original=self.sessions['front']._cancel_fd
        self.sessions['front']._cancel_fd=read;self.sessions['rear']._cancel_fd=read
        other=active.make_owned_waiter(self.lib,read,spin_us=500)
        try:
            with self.assertRaisesRegex(ValueError,'creation'):
                notification.prepare_notifications(self.sessions,other,self.selection)
        finally:
            self.sessions['front']._cancel_fd=original;self.sessions['rear']._cancel_fd=original
            os.close(read);os.close(write)
    def test_mutated_waiter_cancel_is_rejected_before_any_group_or_io(self):
        self.setup_notification();read,write=os.pipe();old=self.waiter._cancel_fd
        self.waiter._cancel_fd=read
        try:
            with self.assertRaisesRegex(ValueError,'creation'):
                self.waiter.readiness_group((Future(),Future()))
        finally:self.waiter._cancel_fd=old;os.close(read);os.close(write)
    def test_closed_and_reused_original_cancel_descriptor_cannot_be_resealed(self):
        self.setup_notification();number=self.cancel_read;os.close(number)
        read,write=os.pipe()
        if read!=number:os.dup2(read,number);os.close(read)
        try:
            with self.assertRaisesRegex(ValueError,'creation'):
                notification.prepare_notifications(self.sessions,self.waiter,self.selection)
        finally:os.close(write)
    def test_destroyed_session_cannot_restore_old_handle_creation_authority(self):
        self.setup_notification();session=self.sessions['front'];original=session._handle
        session.close();session._handle=original
        try:
            with self.assertRaisesRegex(ValueError,'creation'):
                active.verified_active_session_creation(session)
        finally:
            # The saved native handle is already destroyed; no second destroy.
            session._handle=None
    def test_borrowed_diagnostic_forwards_verified_waiter_and_real_source_evidence(self):
        from concurrent.futures import ThreadPoolExecutor
        from singularitydog_hw import diagnostic_runtime_output as producer
        for scope,ids in runtime.BUSES.items():
            self.sessions[scope].close()
            self.sessions[scope]=active.ActiveSession(self.lib,self.sockets[scope][0].fileno(),
                first_id=ids[0],cancel_fd=self.cancel_read,boot_fd=self.boot.fileno(),boot_id=self.boot_id,
                raw_lower_by_id={mid:-1. for mid in ids},raw_upper_by_id={mid:1. for mid in ids},
                kp_max_by_id={mid:0. for mid in ids},kd_max_by_id={mid:0. for mid in ids},gap_ns=900_000,window=3)
        proof=self.setup_notification();self.start_peers();pool=ThreadPoolExecutor(max_workers=3)
        barrier=threading.Barrier(4);starts=[pool.submit(barrier.wait) for _ in range(3)];barrier.wait()
        for f in starts:f.result()
        adapter=None
        try:
            adapter=producer.make_borrowed_output(pool,self.sessions,lambda:None,clock=time.monotonic_ns,
                unpaired_output_future_notifications=True,notification_waiter=self.waiter)
            deadline=time.monotonic_ns()+20_000_000
            originals=adapter.submit_decoded(self.stop_batch(),deadline_ns=deadline,label='proof')
            adapter.collect_output(originals,deadline_ns=deadline,deadline_wait=self.waiter)
            evidence=adapter.evidence();self.assertEqual(evidence['notification_source_binding'],proof)
            self.assertTrue(evidence['genuine_original_output_futures']);self.assertFalse(evidence['type1_sent'])
        finally:
            pool.shutdown(wait=True,cancel_futures=False)
            if adapter is not None:adapter.close()
    def test_cancellation_priority_keeps_original_raw_and_owner_settlement(self):
        self.workers_selected();os.write(self.cancel_write,b'x')
        deadline=time.monotonic_ns()+20_000_000
        originals=self.workers.submit_decoded(self.stop_batch(),deadline_ns=deadline,label='cancelled')
        with self.assertRaises(BaseException):self.workers.collect_output(originals,deadline_ns=deadline,deadline_wait=self.waiter)
        self.assertTrue(self.workers.aborted.is_set())
    def test_owner_error_precedes_simultaneous_host_deadline(self):
        self.workers_selected();a=Future();b=Future();error=RuntimeError('original owner failure')
        a.set_exception(error);b.set_result(None);current={'front':a,'rear':b}
        self.workers._output_notification_current_futures=current
        with self.assertRaisesRegex(RuntimeError,'original owner failure'):
            self.workers.collect_output(current,deadline_ns=time.monotonic_ns()-1,deadline_wait=self.waiter)
    def test_hint_is_not_readiness_and_original_deadline_is_not_extended(self):
        self.workers_selected();a=Future();b=Future();current={'front':a,'rear':b}
        self.workers._output_notification_current_futures=current
        deadline=time.monotonic_ns()+1_000_000
        with self.assertRaises(TimeoutError):self.workers.collect_output(current,deadline_ns=deadline,deadline_wait=self.waiter)
        self.assertFalse(a.done());self.assertFalse(b.done());self.assertGreaterEqual(time.monotonic_ns(),deadline)

class CombinedSourceSelectionTests(unittest.TestCase):
    def test_foreground_command_forwards_independent_checked_codec_and_notification_flags(self):
        from types import SimpleNamespace
        import dog_supported_fk_trial as foreground
        args=SimpleNamespace(profile='/SYNTHETIC/profile',profile_sha256='1'*64,
            library='/SYNTHETIC/library',library_sha256='2'*64,audio='/SYNTHETIC/audio',
            audio_sha256='3'*64,front_port='/SYNTHETIC/front',rear_port='/SYNTHETIC/rear',
            audio_device='SYNTHETIC',power_epoch='SYNTHETIC',request_gap_us=900,release_spin_us=500)
        for codec_flag in (False,True):
            for checked_flag in (False,True):
                for notification_flag in (False,True):
                    with self.subTest(codec=codec_flag,checked=checked_flag,notification=notification_flag):
                        draft={'native_feedback_batch_decode':codec_flag,
                            'native_checked_policy_dispatch':checked_flag,
                            'unpaired_output_future_notifications':notification_flag,
                            'artifacts':{'checked_model_manifest':{'path':'/SYNTHETIC/checked','sha256':'4'*64}}}
                        command=foreground.build_command(args,draft,'/SYNTHETIC/output')
                        self.assertIs('--native-feedback-batch-decode' in command,codec_flag)
                        self.assertIs('--checked-model-manifest' in command,checked_flag)
                        self.assertIs('--unpaired-output-future-notifications' in command,notification_flag)
                        self.assertNotIn('--native-phase-pair',command)
    def test_diagnostic_profile_notification_mismatch_rejects_before_input_or_io(self):
        from types import SimpleNamespace
        from singularitydog_hw import native_pipeline_benchmark as benchmark
        args=SimpleNamespace(active_fk_profile='/SYNTHETIC/profile.json',active_fk_profile_sha256='1'*64,
            mode='stop-proxy',supported_disabled=True,acquisition_only=False,compare_feedback=False,
            view_cache_manifest=None,view_cache_manifest_sha256=None,retain_gil_trace_copy=False,
            native_boot_guard_artifact=None,native_boot_guard_artifact_sha256=None,
            unpaired_output_future_notifications=True)
        draft=profile(False);draft['native_target_fk_cache']=True
        with patch.object(profiles,'_read_json',return_value=({},'1'*64)), \
                patch.object(profiles,'load_profile',return_value=draft), \
                patch.object(profiles,'execution_settings') as settings:
            with self.assertRaisesRegex(ValueError,'notification selection'):
                benchmark._active_fk_diagnostic_context(args)
            settings.assert_not_called()
    def test_all_three_selectors_are_independent_in_start_and_finish_source_seal(self):
        from singularitydog_hw import native_pipeline_benchmark as benchmark
        flags=dict(accel_input_hypothesis=True,native_target_fk_cache=True,native_phase_pair=False,
            native_feedback_batch_decode=True,native_checked_policy_dispatch=True,
            unpaired_output_future_notifications=True)
        mode=profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER
        proof=benchmark._start_source_provenance(mode,'SYNTHETIC file-only epoch',**flags)
        expected={k:v for k,v in flags.items() if v};expected.update(schema=profiles.SCHEMA_V3,
            diagnostic_timing_acceptance=mode)
        self.assertEqual(proof['cadence_source_sha256'],profiles.cadence_source_hashes(expected))
        self.assertNotIn('native_phase_pair',proof)
        report={'status':'COMPLETE_DIAGNOSTIC'}
        benchmark._finish_source_provenance(report,proof)
        self.assertTrue(proof['source_files_unchanged']);self.assertNotIn('errors',report)
        paths=profiles.cadence_source_paths(expected)
        self.assertIn('singularitydog_hw/policy_checked_dispatch.py',paths)
        self.assertIn('singularitydog_hw/unpaired_output_future_notifications.py',paths)
        self.assertIn('singularitydog_hw/unpaired_native_feedback_codec.py',paths)
    def test_codec_checked_poll_has_no_notification_selection_or_cadence_requirement(self):
        from singularitydog_hw import native_pipeline_benchmark as benchmark
        mode=profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER
        proof=benchmark._start_source_provenance(mode,'SYNTHETIC file-only epoch',
            accel_input_hypothesis=True,native_target_fk_cache=True,native_feedback_batch_decode=True,
            native_checked_policy_dispatch=True,unpaired_output_future_notifications=False)
        self.assertNotIn('unpaired_output_future_notifications',proof)
        self.assertNotIn('singularitydog_hw/unpaired_output_future_notifications.py',proof['cadence_source_sha256'])
        report={'status':'COMPLETE_DIAGNOSTIC'};benchmark._finish_source_provenance(report,proof)
        self.assertTrue(proof['source_files_unchanged']);self.assertNotIn('errors',report)

if __name__=='__main__':unittest.main()
