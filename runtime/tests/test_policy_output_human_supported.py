"""Human-catch integration boundaries on in-memory buses; no motor devices."""
from contextlib import ExitStack, nullcontext, redirect_stderr, redirect_stdout
import copy
import io
import hashlib
import math
import os
from pathlib import Path
import pty
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import wave
from types import SimpleNamespace
from unittest.mock import patch, Mock

from singularitydog_hw import policy_output as cli
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import human_supported_hold as human
from test_policy_output_runtime import FakeSession, FakeIMU, SimulatedClock, encode_motion, wire
from test_human_supported_hold import ParkedThread, FakeAudio
from test_human_supported_partial_profile import human_fixture, seal_human
from test_policy_output_cli import MockSerialPort


def loaded_synthetic_profile(base):
    """Use real loader gates; all receipts/models/ports remain synthetic."""
    data,docs,pins,receipts=human_fixture(base)
    with patch.object(live.shadow,'SOURCE_HASHES',pins):
        seal_human(base,data,docs,receipts)
        return live.load_profile(base/'profile.json',require_approved=True),docs


class EmergencyRecoveryOrderingTests(unittest.TestCase):
    def workers(self, notify):
        events=[]
        class Session(FakeSession):
            def emergency_stop(self):
                events.append(('stop',self.ids[0]))
                return super().emergency_stop()
        sessions={'front':Session(1),'rear':Session(7)}
        workers=runtime.BusWorkers(sessions,lambda:events.append(('cancel',None)),
            before_emergency_stop=lambda reason:notify(events,reason))
        self.addCleanup(workers.close)
        return workers,events

    def test_recovery_notification_precedes_cancellation_and_both_stop_owners(self):
        workers,events=self.workers(lambda rows,reason:rows.append(('resupport',reason)))
        workers.emergency('Synthetic missing Type1 reply')
        result=workers.finish_stops()
        self.assertEqual(events[:2],[('resupport','Synthetic missing Type1 reply'),('cancel',None)])
        self.assertEqual(sorted(event[1] for event in events if event[0]=='stop'),[1,7])
        self.assertEqual(sum(event[0]=='resupport' for event in events),1)
        self.assertTrue(all(row['complete'] for row in result.values()))

    def test_terminal_notification_failure_never_suppresses_stop_dispatch(self):
        def failing(rows,reason):
            rows.append(('resupport',reason))
            raise OSError('Synthetic terminal disappearance')
        workers,events=self.workers(failing)
        workers.emergency('USB fault')
        result=workers.finish_stops()
        self.assertTrue(all(row['complete'] for row in result.values()))
        self.assertEqual(sorted(event[1] for event in events if event[0]=='stop'),[1,7])
        self.assertEqual(workers.emergency_errors[0]['stage'],'recovery_notification')

    def test_normal_completion_does_not_invent_a_fault_recovery_cue(self):
        workers,events=self.workers(lambda rows,reason:rows.append(('resupport',reason)))
        workers.finish_stops()
        self.assertFalse(any(event[0]=='resupport' for event in events))
        self.assertEqual(sorted(event[1] for event in events if event[0]=='stop'),[1,7])

    def test_native_owner_failure_notifies_without_waiting_for_main_collection(self):
        workers,events=self.workers(lambda rows,reason:rows.append(('resupport',reason)))
        worker_returned=threading.Event()
        def fault():
            try:workers._exchange('front',[b'not-a-valid-request'])
            except BaseException:worker_returned.set()
        thread=threading.Thread(target=fault);thread.start();thread.join(1.)
        self.assertTrue(worker_returned.is_set())
        self.assertTrue(workers.aborted.is_set())
        self.assertEqual(events[0][0],'resupport')
        workers.finish_stops()


class HumanCLIAdmissionTests(unittest.TestCase):
    def quiet(self,args):
        with redirect_stderr(io.StringIO()),redirect_stdout(io.StringIO()):
            return cli.main(args)

    def physical_flags(self):
        return ['--two-operators-full-weight-catch','--slight-ease-only',
                '--resupport-before-stop','--off-power-rehearsal',
                '--side-view-video-ready','--paws-floor','--cutoff-ready',
                '--absolute-epoch-cadence']

    def test_every_fresh_operator_confirmation_is_required_before_profile_or_devices(self):
        flags=self.physical_flags()
        for removed in flags:
            with self.subTest(removed=removed),patch.object(cli,'load_profile') as load:
                with self.assertRaises(SystemExit):
                    self.quiet(['--profile','DO_NOT_READ','--execute-human-supported-partial',
                                *(flag for flag in flags if flag!=removed)])
                load.assert_not_called()

    def test_human_execution_cannot_be_mixed_with_other_output_modes(self):
        for mode in ('--execute-supported','--execute-supported-preload','--execute-fixed-catch'):
            with self.subTest(mode=mode),patch.object(cli,'load_profile') as load:
                with self.assertRaises(SystemExit):
                    self.quiet(['--profile','DO_NOT_READ','--execute-human-supported-partial',mode,
                                *self.physical_flags()])
                load.assert_not_called()

    def test_operator_flags_cannot_authorize_the_legacy_ground_or_supported_runner(self):
        for flag in self.physical_flags()[:-2]:
            with self.subTest(flag=flag),patch.object(cli,'load_profile') as load:
                with self.assertRaises(SystemExit):
                    self.quiet(['--profile','DO_NOT_READ','--execute-supported',flag])
                load.assert_not_called()


class OwnedStageAudioTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.base=Path(self.tmp.name)
        self.stages={}
        for stage in cli.HUMAN_AUDIO_STAGES:
            path=self.base/(stage+'.wav')
            with wave.open(str(path),'wb') as audio:
                audio.setnchannels(1);audio.setsampwidth(2);audio.setframerate(1000)
                audio.writeframes(b'\x00\x00'*60)
            digest=hashlib.sha256(path.read_bytes()).hexdigest()
            self.stages[stage]=cli.pinned_audio_file(path,digest,go=stage=='go')

    def player(self,popen):
        player=cli.HumanStageAudioPlayer(self.stages,'DO_NOT_PLAY',popen=popen)
        self.addCleanup(player.close)
        return player

    class Process:
        def __init__(self,returncode=0):self.returncode=returncode;self.owners=[]
        def poll(self):return self.returncode
        def terminate(self):self.owners.append(threading.get_ident());self.returncode=-15
        def wait(self,timeout=None):return self.returncode
        def kill(self):self.owners.append(threading.get_ident());self.returncode=-9

    def test_real_pcm_bounds_reject_zero_frames_long_go_and_truncated_data(self):
        path=self.base/'invalid.wav'
        for frames in (0,121):
            with wave.open(str(path),'wb') as audio:
                audio.setnchannels(1);audio.setsampwidth(2);audio.setframerate(1000)
                audio.writeframes(b'\0\0'*frames)
            with self.assertRaises(ValueError):
                cli.pinned_audio_file(path,hashlib.sha256(path.read_bytes()).hexdigest(),go=True)
        raw=Path(self.stages['go']['path']).read_bytes()[:-2];path.write_bytes(raw)
        with self.assertRaisesRegex(ValueError,'truncated'):
            cli.pinned_audio_file(path,hashlib.sha256(raw).hexdigest())

    def test_actor_plays_anonymous_approved_bytes_and_reports_real_start_and_exit(self):
        complete=threading.Event();calls=[];started=[];finished=[]
        expected=Path(self.stages['go']['path']).read_bytes()
        def popen(args,**kw):
            calls.append((threading.get_ident(),args,kw['stdin'].read()))
            return self.Process()
        player=self.player(popen)
        player.start('go',lambda *row:(finished.append(row),complete.set()),
                     lambda *row:self.fail(str(row)),on_started=lambda *row:started.append(row))
        self.assertTrue(complete.wait(1.))
        self.assertEqual(calls[0][1],['aplay','-D','DO_NOT_PLAY'])
        self.assertEqual(calls[0][2],expected)
        self.assertNotEqual(calls[0][0],threading.get_ident())
        self.assertEqual(started[0][0],'go')
        self.assertLessEqual(started[0][1],finished[0][2])
        self.assertEqual(player.events[0]['status'],'complete')

    def test_file_mutation_before_owned_playback_prevents_any_process(self):
        popen_calls=[];failed=threading.Event()
        player=self.player(lambda *args,**kw:popen_calls.append(args))
        Path(self.stages['go']['path']).write_bytes(b'unreviewed replacement')
        player.start('go',lambda *args:self.fail('Mutated audio completed'),
                     lambda *args:failed.set())
        self.assertTrue(failed.wait(1.));self.assertEqual(popen_calls,[])

    def test_cancel_before_queueing_rejects_go_but_allows_abort_recovery(self):
        complete=threading.Event();calls=[]
        def popen(args,**kw):calls.append(args);return self.Process()
        player=self.player(popen);player.cancel()
        with self.assertRaises(RuntimeError):
            player.start('go',lambda *a:None,lambda *a:None)
        player.start('abort',lambda *a:complete.set(),lambda *a:self.fail(str(a)))
        self.assertTrue(complete.wait(1.));self.assertEqual(len(calls),1)

    def test_cancel_during_spawn_suppresses_late_go_callback_and_closes_on_owner(self):
        spawning=threading.Event();release=threading.Event();failed=threading.Event()
        started=[];completed=[];process=self.Process(None);owner=[]
        def popen(*args,**kw):
            owner.append(threading.get_ident());spawning.set()
            if not release.wait(1.):raise TimeoutError('Synthetic spawn did not release')
            return process
        player=self.player(popen)
        player.start('go',lambda *a:completed.append(a),lambda *a:failed.set(),
                     on_started=lambda *a:started.append(a))
        self.assertTrue(spawning.wait(1.));player.cancel();release.set()
        self.assertTrue(failed.wait(1.));self.assertEqual(started,[]);self.assertEqual(completed,[])
        self.assertEqual(process.owners,owner)

    def test_running_preparation_cancels_before_nonoverlapping_recovery(self):
        running=threading.Event();done=threading.Event();process=self.Process(None);calls=[]
        def popen(*args,**kw):
            calls.append(threading.get_ident())
            if len(calls)==1:running.set();return process
            self.assertIsNotNone(process.returncode)
            return self.Process()
        player=self.player(popen)
        player.start('prepare_ease',lambda *a:self.fail('Cancelled preparation completed'),lambda *a:None)
        self.assertTrue(running.wait(1.));player.cancel()
        player.start('resupport',lambda *a:done.set(),lambda *a:self.fail(str(a)))
        self.assertTrue(done.wait(1.));self.assertEqual(len(calls),2)
        self.assertEqual(len(set(calls+process.owners)),1)

    def test_nonzero_exit_and_callback_failure_never_complete_a_stage(self):
        for returncode,broken_callback in ((4,False),(0,True)):
            with self.subTest(returncode=returncode,broken_callback=broken_callback):
                failed=threading.Event();completed=[]
                player=self.player(lambda *a,**kw:self.Process(returncode))
                def started(*args):
                    if broken_callback:raise RuntimeError('Supervisor rejected stale Go')
                player.start('go',lambda *a:completed.append(a),lambda *a:failed.set(),on_started=started)
                self.assertTrue(failed.wait(1.));self.assertEqual(completed,[])

    def test_audio_timeout_terminates_child_and_reports_failure(self):
        failed=threading.Event();process=self.Process(None)
        player=self.player(lambda *args,**kw:process)
        player.start('prepare_ease',lambda *a:self.fail('Hung audio completed'),lambda *a:failed.set())
        self.assertTrue(failed.wait(1.));self.assertIsNotNone(process.returncode)
        self.assertIn('completion deadline',player.events[0]['error'])

    def test_post_stop_close_drains_urgent_audio_without_terminating_it(self):
        running=threading.Event();completed=threading.Event();process=self.Process(None)
        player=self.player(lambda *args,**kw:(running.set(),process)[1])
        player.start('abort',lambda *a:completed.set(),lambda *a:self.fail(str(a)))
        self.assertTrue(running.wait(1.))
        # close belongs to post-STOP cleanup, never to the emergency path.
        closer=threading.Thread(target=player.close);closer.start()
        time.sleep(.02);self.assertTrue(closer.is_alive())
        process.returncode=0;closer.join(1.)
        self.assertFalse(closer.is_alive());self.assertTrue(completed.is_set())
        self.assertEqual(process.owners,[])

    def test_post_stop_recovery_failure_cannot_silently_report_success(self):
        failed=threading.Event()
        player=self.player(lambda *args,**kw:self.Process(5))
        player.start('abort',lambda *a:self.fail('Failed recovery completed'),lambda *a:failed.set())
        self.assertTrue(failed.wait(1.))
        with self.assertRaisesRegex(RuntimeError,'recovery audio playback unconfirmed'):
            player.close()
        self.assertTrue(player.closed)

    def test_recovery_process_receipt_is_distinct_from_stop_and_audibility(self):
        supervision=SimpleNamespace(audio_requests={'resupport':1})
        for rows,confirmed in (([],False),([dict(stage='resupport',status='failed',error='exit5')],False),
                               ([dict(stage='resupport',status='complete')],True)):
            with self.subTest(rows=rows):
                evidence=cli.human_audio_report(supervision,SimpleNamespace(events=rows))
                self.assertTrue(evidence['human_audio_recovery_requested'])
                self.assertEqual(evidence['human_audio_recovery_confirmed'],confirmed)
                self.assertEqual(bool(evidence['human_audio_recovery_errors']),not confirmed)
                self.assertFalse(evidence['human_audio_completion_proves_audibility'])
                self.assertNotIn('stop_confirmed',evidence)


class HumanRuntimeIntegrationTests(unittest.TestCase):
    """Real profile admission/supervision with causal in-memory wire exchanges."""
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.base=Path(self.tmp.name)
        self.data,self.docs=loaded_synthetic_profile(self.base)
        self.clock=SimulatedClock()
        self.master,self.slave=pty.openpty()
        self.addCleanup(os.close,self.master);self.addCleanup(os.close,self.slave)
        self.audio=FakeAudio(self.clock)
        self.supervisor=human.HumanSupportedHoldExecution(
            self.slave,clock=self.clock,thread_factory=ParkedThread,stage_audio=self.audio)
        self.addCleanup(self.supervisor.close)
        self.cancelled=threading.Event();self.supervisor.connect_cancel(self.cancelled.set)
        self.supervisor.bind_profile(self.data,active=True)
        data=self.data;positions=self.docs['local_reference_capture']['telemetry']['rows']
        class Session(FakeSession):
            def __init__(owned,first_id):
                super().__init__(first_id,clock=self.clock)
                owned.positions={mid:positions[str(mid)]['median_position_rad'] for mid in owned.ids}
            def _exchange(owned,wires,timeout_ns,send_only):
                result=super()._exchange(wires,timeout_ns,send_only)
                for record in result[0]:
                    tx=runtime.codec.ATParser().feed(bytes(record.tx))[0]
                    if tx.kind==0:
                        record.rx[:]=wire((tx.destination<<8)|0xFE,
                            bytes.fromhex(data['axes'][str(tx.destination)]['uid']))
                return result
        self.sessions={'front':Session(1),'rear':Session(7)}
        self.validations=[]
        class ValidationOnly:
            def validate_inputs(inner,*args):self.validations.append(args[-1])
            def __call__(inner,*args):raise AssertionError('No inference in partial current hold')
        self.policy=ValidationOnly()

    def run_hold(self,*,auto_ack=True,after_valid=None,policy=None,supervision=None,announce=lambda:None):
        supervisor=self.supervisor if supervision is None else supervision
        before=supervisor.before_cycle;after=supervisor.after_cycle_validated
        original_wait=runtime.wait
        def fixture_wait(futures,*,timeout,return_when):
            # These real owner threads use shared synthetic time. OS scheduling
            # is not simulated bus latency: allow a finite readiness handshake
            # without advancing that clock. Keep real Future states/errors and
            # FIRST_EXCEPTION; runtime still checks its unchanged clock deadline.
            self.assertTrue(math.isfinite(timeout) and timeout>0)
            return original_wait(futures,timeout=2.,return_when=return_when)
        clips=live.human_supported_audio_settings(self.data)['clips'];ack_sent=False
        def drive_audio_and_input(begun_ns,**kwargs):
            nonlocal ack_sent
            for stage,entry in list(self.audio.pending.items()):
                started,_,_=entry
                if self.clock()-started>=round(clips[stage]['duration_s']*1e9):
                    self.audio.finish(stage)
            if auto_ack and supervisor.ack_prompt_ns is not None and not ack_sent:
                os.write(self.master,b'\n');ack_sent=True
            return before(begun_ns,**kwargs)
        def completed(*args,**kwargs):
            after(*args,**kwargs)
            if after_valid is not None:after_valid(*args,**kwargs)
        with patch.object(supervisor,'before_cycle',side_effect=drive_audio_and_input), \
             patch.object(supervisor,'after_cycle_validated',side_effect=completed), \
             patch.object(runtime,'wait',side_effect=fixture_wait):
            report=runtime.run_supported_policy(self.data,self.sessions,FakeIMU(clock=self.clock),
                self.policy if policy is None else policy,cancel_io=self.cancelled.set,
                clock=self.clock,sleep=self.clock.sleep,encode_motion=encode_motion,
                supervision=supervisor,absolute_epoch_cadence=True,announce=announce)
        return supervisor.decorate_report(report)

    def test_current_hold_full_gain_audio_ack_and_all_axis_stop(self):
        full_gain_cycles=[]
        report=self.run_hold(after_valid=lambda *a,**k:full_gain_cycles.append((a[2],k['full_gain'])))
        self.assertEqual(report['status'],'COMPLETE_HUMAN_SUPPORTED_PARTIAL_HOLD',report['errors'])
        self.assertTrue(report['motor_enable_sent']);self.assertTrue(report['motion_gain_sent'])
        self.assertFalse(report['learned_targets_sent']);self.assertTrue(report['cyclic_inference_skipped'])
        self.assertTrue(report['stop_confirmed']);self.assertTrue(report['normal_ramp_completed'])
        self.assertEqual(len(self.validations),len(report['cycles'])+1)
        self.assertGreater(self.supervisor.ack_ns,self.supervisor.resupport_audio_finished_ns)
        self.assertIn(('active',True),full_gain_cycles)
        self.assertTrue(all(not full for phase,full in full_gain_cycles if phase!='active'))
        self.assertEqual(self.audio.requests,['prepare_ease','go','resupport'])
        self.assertFalse(report['full_controller_50Hz_verified'])
        for session in self.sessions.values():
            positions={mid:set() for mid in session.ids}
            for _,kind,mid,request in session.calls:
                if kind==1:positions[mid].add(struct.unpack('>4H',request[7:15])[0])
            self.assertTrue(all(len(values)==1 for values in positions.values()))

    def test_missing_fresh_full_support_ack_stops_without_normal_gain_down(self):
        report=self.run_hold(auto_ack=False)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(report['stop_confirmed']);self.assertFalse(report['normal_ramp_completed'])
        self.assertIsNone(self.supervisor.ack_ns);self.assertIsNotNone(self.supervisor.failed)
        self.assertTrue(all(row['command']['phase']!='stopping' for row in report['cycles']))
        self.assertFalse(report['learned_targets_sent'])

    def test_async_audio_failure_latches_before_next_output_and_no_ease(self):
        positive_writes_at_fault=[]
        def fault(*args,**kwargs):
            if self.supervisor.prepare_request_ns is not None and not positive_writes_at_fault:
                positive_writes_at_fault.append(sum(s.positive_gain_writes for s in self.sessions.values()))
                self.supervisor._audio_failure('prepare_ease','Synthetic audio device lost')
        report=self.run_hold(after_valid=fault)
        self.assertEqual(report['status'],'ABORTED',report['errors']);self.assertTrue(report['stop_confirmed'])
        self.assertIsNone(self.supervisor.cue_ns)
        self.assertEqual(sum(s.positive_gain_writes for s in self.sessions.values()),positive_writes_at_fault[0])
        self.assertIn('AUDIO_FAILURE',str(report['errors']))

    def test_supervision_work_is_included_in_hard_cycle_deadline(self):
        injected=[]
        def slow(*args,**kwargs):
            if not injected:
                injected.append(True);self.clock.advance(20_000_001)
        report=self.run_hold(after_valid=slow)
        self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
        self.assertIn('supervision exceeded hard deadline',str(report['errors']))
        self.assertEqual(report['failed_cycle_timing']['stage'],'human_hold_supervision')
        self.assertEqual(report['deadline20ms_misses'],1)

    def test_untrusted_exact_type_binding_and_timing_must_fail_before_any_bus(self):
        cases=[('no supervisor',None),('generic ground',object())]
        for name,supervisor in cases:
            with self.subTest(name=name),self.assertRaises(RuntimeError):
                runtime.run_supported_policy(self.data,self.sessions,FakeIMU(clock=self.clock),self.policy,
                    cancel_io=self.cancelled.set,supervision=supervisor,absolute_epoch_cadence=True)
            self.assertTrue(all(not s.calls for s in self.sessions.values()))
        self.supervisor.cancel=None
        with self.assertRaisesRegex(RuntimeError,'connected emergency'):
            runtime.run_supported_policy(self.data,self.sessions,FakeIMU(clock=self.clock),self.policy,
                cancel_io=self.cancelled.set,supervision=self.supervisor,absolute_epoch_cadence=True)
        self.assertTrue(all(not s.calls for s in self.sessions.values()))

    def test_mutated_loaded_token_contract_fails_before_any_bus(self):
        for mutation in (lambda p:p.update(_human_supported_token=True),
                         lambda p:p.update(startup_cycle_allowance=live.FIRST_CYCLE_POST_REPLY),
                         lambda p:p.update(post_reply_deadline_policy={}),
                         lambda p:p['axes']['1'].update(kp=12.),
                         lambda p:p.update(policy_weight=.01)):
            data=dict(self.data);data['axes']=copy.deepcopy(self.data['axes']);mutation(data)
            with self.assertRaises(live.ProfileError):
                runtime.run_supported_policy(data,self.sessions,FakeIMU(clock=self.clock),self.policy,
                    cancel_io=self.cancelled.set,supervision=self.supervisor,absolute_epoch_cadence=True)
            self.assertTrue(all(not s.calls for s in self.sessions.values()))

    def test_started_supervisor_cannot_be_reused_after_an_earlier_trial(self):
        self.supervisor.on_start(self.clock())
        with self.assertRaisesRegex(RuntimeError,'Cannot rebind'):
            runtime.run_supported_policy(self.data,self.sessions,FakeIMU(clock=self.clock),self.policy,
                cancel_io=self.cancelled.set,supervision=self.supervisor,absolute_epoch_cadence=True)
        self.assertTrue(all(not s.calls for s in self.sessions.values()))

    def test_active_but_pd_derated_commands_never_issue_go(self):
        original=runtime.PolicyMotionEnvelope.step;active=[]
        def derated(envelope,*args,**kwargs):
            from dataclasses import replace
            command=original(envelope,*args,**kwargs)
            if command.phase=='active':
                active.append(True)
                return replace(command,kp=tuple(k/2 for k in command.kp),
                    kd=tuple(k/2 for k in command.kd),gain_scale=.5)
            return command
        with patch.object(runtime.PolicyMotionEnvelope,'step',new=derated):
            report=self.run_hold()
        self.assertTrue(active);self.assertIsNone(self.supervisor.cue_ns)
        self.assertNotIn('go',self.audio.requests);self.assertTrue(report['stop_confirmed'])
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('shutdown reserve',str(report['errors']))

    def test_model_input_fault_keeps_current_hold_validation_and_stops_before_gain(self):
        class InvalidInput:
            def validate_inputs(inner,*args):raise ValueError('Synthetic current-hold IMU tilt fault')
            def __call__(inner,*args):raise AssertionError('Inference must stay disabled')
        report=self.run_hold(policy=InvalidInput())
        self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
        self.assertIn('IMU tilt fault',str(report['errors']))
        self.assertFalse(report['motion_gain_sent']);self.assertFalse(report['learned_targets_sent'])
        self.assertIsNone(self.supervisor.cue_ns)
        self.assertEqual(self.audio.requests,['abort'])

    def test_pre_enable_spoken_brief_failure_stops_without_enable_or_gain(self):
        def lost_audio():raise OSError('Synthetic initial brief device lost')
        report=self.run_hold(announce=lost_audio)
        self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['motor_enable_sent']);self.assertFalse(report['motion_gain_sent'])
        self.assertFalse(report['learned_targets_sent']);self.assertFalse(report['cycles'])
        self.assertTrue(all(not any(kind==3 for _,kind,_,_ in s.calls) for s in self.sessions.values()))
        self.assertIsNone(self.supervisor.cue_ns)


class HumanCLIConcreteGatingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.base=Path(self.tmp.name)
        self.data,_=loaded_synthetic_profile(self.base)
        self.clips=live.human_supported_audio_settings(self.data)['clips']

    def args(self):
        brief=self.clips['brief']
        arguments=['--profile',str(self.base/'profile.json'),'--execute-human-supported-partial',
            *HumanCLIAdmissionTests.physical_flags(self),
            '--front-port','DO_NOT_OPEN_FRONT','--rear-port','DO_NOT_OPEN_REAR',
            '--library','DO_NOT_LOAD','--output',str(self.base/'report'),
            '--audio',brief['path'],'--audio-sha256',brief['sha256'],
            '--audio-device','DO_NOT_PLAY','--power-epoch',self.data['motor_power_epoch']]
        for stage in cli.HUMAN_AUDIO_STAGES:
            option='--human-'+stage.replace('_','-')+'-audio';clip=self.clips[stage]
            arguments.extend((option,clip['path'],option+'-sha256',clip['sha256']))
        return arguments

    def quiet(self,args):
        with redirect_stderr(io.StringIO()),redirect_stdout(io.StringIO()):return cli.main(args)

    def test_unapproved_profile_cannot_construct_actor_model_or_devices(self):
        with patch.object(cli,'load_profile',side_effect=live.ProfileError('Synthetic approval absent')), \
             patch.object(cli,'HumanStageAudioPlayer') as actor, \
             patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model:
            with self.assertRaises(live.ProfileError):self.quiet(self.args())
            actor.assert_not_called();model.assert_not_called()

    def test_missing_changed_or_manifest_mismatched_audio_never_constructs_actor(self):
        for condition in ('missing','changed','mismatched pin'):
            with self.subTest(condition=condition):
                args=self.args();clip=self.clips['go'];path=Path(clip['path']);raw=path.read_bytes()
                if condition=='missing':path.unlink()
                elif condition=='changed':path.write_bytes(raw+b'unreviewed tail')
                else:
                    replacement=self.base/'unapproved-copy.wav';replacement.write_bytes(raw)
                    args[args.index('--human-go-audio')+1]=str(replacement)
                try:
                    with patch.object(cli,'load_profile',return_value=self.data), \
                         patch.object(cli,'HumanStageAudioPlayer') as actor, \
                         patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model:
                        with self.assertRaises((FileNotFoundError,ValueError,SystemExit)):
                            self.quiet(args)
                        actor.assert_not_called();model.assert_not_called()
                finally:path.write_bytes(raw)

    def test_non_tty_returns_before_model_or_native_device_and_closes_actor(self):
        actor=Mock(events=[])
        with patch.object(cli,'load_profile',return_value=self.data), \
             patch.object(cli,'HumanStageAudioPlayer',return_value=actor), \
             patch('singularitydog_hw.human_supported_hold.os.isatty',return_value=False), \
             patch('singularitydog_hw.policy_output_model.LivePolicyModel') as model, \
             patch('singularitydog_hw.native_active_transport.load_library') as native:
            code=self.quiet(self.args())
        self.assertEqual(code,2);model.assert_not_called();native.assert_not_called()
        actor.close.assert_called_once_with()

    def test_cli_uses_exact_supervisor_and_one_real_length_pre_enable_brief(self):
        master,slave=pty.openpty();self.addCleanup(os.close,master);self.addCleanup(os.close,slave)
        actor=Mock(events=[]);original_supervisor=human.HumanSupportedHoldExecution
        executions=[];ports=[];sessions=[];real_open=os.open
        boot=self.base/'fake-boot';boot.write_text(self.data['boot_id'])
        guard=Mock(boot_id=self.data['boot_id'])
        device=Mock(restore_status='restored');device.start.return_value={}
        model=SimpleNamespace(calls=0,provenance={'kind':'synthetic-model'})
        played=[]
        def supervisor(*args,**kwargs):
            e=original_supervisor(slave,stage_audio=kwargs['stage_audio']);executions.append(e)
            return e
        def serial_port(**kwargs):
            p=MockSerialPort(self.base/('fake-port-'+str(len(ports))));ports.append(p);return p
        def session(*args,**kwargs):
            s=Mock();sessions.append(s);return s
        def open_regular(path,flags,*args,**kwargs):
            if os.fspath(path)=='/proc/sys/kernel/random/boot_id':path=boot
            return real_open(path,flags,*args,**kwargs)
        def playback(args,**kwargs):
            played.append((args,kwargs['stdin'].read(),kwargs['timeout']))
        def run(profile,owned,imu,policy,**kwargs):
            self.assertIs(profile,self.data);self.assertIs(policy,model)
            self.assertIs(type(kwargs['supervision']),original_supervisor)
            self.assertTrue(kwargs['supervision']._active_bound)
            self.assertTrue(kwargs['absolute_epoch_cadence'])
            # The real runtime checks/handles speech before any Enable; its
            # separate integration test covers initial announcement failure.
            kwargs['announce']()
            return dict(status='COMPLETE_HUMAN_SUPPORTED_PARTIAL_HOLD',errors=[],
                motor_enable_sent=False,learned_targets_sent=False,stop_confirmed=True)
        bindings={scope:dict(path='DO_NOT_OPEN',resolved='DO_NOT_OPEN',st_rdev=0)
                  for scope in ('front','rear')}
        with ExitStack() as stack:
            stack.enter_context(patch.object(cli,'load_profile',return_value=self.data))
            stack.enter_context(patch.object(cli,'HumanStageAudioPlayer',return_value=actor))
            stack.enter_context(patch.object(human,'HumanSupportedHoldExecution',side_effect=supervisor))
            stack.enter_context(patch.dict(sys.modules,{'torch':SimpleNamespace(
                set_num_threads=Mock(),set_num_interop_threads=Mock()),
                'serial':SimpleNamespace(Serial=serial_port)}))
            stack.enter_context(patch.object(cli.os,'open',side_effect=open_regular))
            stack.enter_context(patch.object(cli.signal,'signal',return_value=None))
            stack.enter_context(patch('singularitydog_hw.policy_output_model.LivePolicyModel',return_value=model))
            stack.enter_context(patch('singularitydog_hw.native_active_transport.load_library',return_value=object()))
            stack.enter_context(patch('singularitydog_hw.native_active_transport.ActiveSession',side_effect=session))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.validate_ports',return_value=bindings))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.binding_matches',return_value=True))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.BootIdentityGuard',return_value=guard))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.pipeline.ownership_locks',side_effect=nullcontext))
            stack.enter_context(patch('singularitydog_hw.dual_can_pipeline_benchmark.port_lock',side_effect=lambda _:nullcontext()))
            stack.enter_context(patch('singularitydog_hw.policy_observer_live.imu_ownership_lock',side_effect=nullcontext))
            stack.enter_context(patch('singularitydog_hw.imu.ICM20948',return_value=device))
            stack.enter_context(patch('singularitydog_hw.policy_output_runtime.run_supported_policy',side_effect=run))
            stack.enter_context(patch.object(cli.subprocess,'run',side_effect=playback))
            code=self.quiet(self.args())
        self.assertEqual(code,0);self.assertEqual(len(played),1)
        self.assertEqual(played[0][0],['aplay','-D','DO_NOT_PLAY'])
        self.assertEqual(hashlib.sha256(played[0][1]).hexdigest(),self.clips['brief']['sha256'])
        self.assertEqual(played[0][2],self.clips['brief']['duration_s']+.25)
        self.assertTrue(executions[0].closed);self.assertTrue(all(p.closed for p in ports))
        self.assertTrue(all(s.close.call_count==1 for s in sessions))



if __name__=='__main__':unittest.main()
