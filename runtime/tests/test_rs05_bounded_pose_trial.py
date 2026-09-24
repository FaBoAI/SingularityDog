"""Offline absolute raw target/arrival/stop tests with synthetic feedback."""
from dataclasses import replace
import math
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import bounded_pose_plan as pose
from singularitydog_hw import rs05_leg_trial as trial
from singularitydog_hw.can_readonly import ATParser
from test_rs05_leg_position_gate import SelectedWindowTransport
from test_position_response_evidence import evidence_file


class PoseTransport(SelectedWindowTransport):
    def __init__(self, leg='FR', failure=None, reply_delay_s=.007):
        super().__init__(leg)
        self.failure = failure
        self.commanded = {}
        self.reply_delay_s=reply_delay_s

    def send(self, wire):
        super().send(wire)
        frame=ATParser().feed(wire)[0]
        if frame.kind==1:
            position,_,kp,_=struct.unpack('>4H',frame.data)
            if kp:self.commanded[frame.destination]=position*25.14/65535.-12.57

    def parameter(self, mid, name=None):
        result=super().parameter(mid,name)
        if name is None and self.failure=='identity' and mid==self.ids[1]:
            return {'mcu_uid_hex':'f'*16}
        return result

    def value(self, mid):
        fb=super().value(mid)
        if mid in self.enabled and mid in self.commanded:
            target=self.commanded[mid]
            if self.failure=='not_tracking':target=self.centers[mid]
            if self.failure=='partial_tracking':target=self.centers[mid]+.13*(target-self.centers[mid])
            if self.failure=='hold_drift' and self.clock()-self.enable_time>4.35 and mid==self.ids[1]:
                target+=math.radians(1.2)
            fb=replace(fb,protocol_position_rad=target)
        return fb

    def feedback_many(self,wires,expected_ids):
        result=super().feedback_many(wires,expected_ids)
        extra=self.reply_delay_s-.007
        self.clock.wait(extra)
        # Three positive, distinct source latencies within the7ms fake batch.
        return {mid:(value,received+extra-(2-index)*.001)
                for index,(mid,(value,received)) in enumerate(result.items())}


def execute(t, targets=None, *, emit=lambda row:None, interrupt=lambda:None, evidence_leg=None, gain_profile='kp3', **kwargs):
    if targets is None:
        targets={i:t.centers[i]+d for i,d in zip(t.ids,(math.radians(4),math.radians(-2),0.))}
    leg=next(name for name,ids in trial.LEGS.items() if ids==t.ids)
    with evidence_file(evidence_leg or leg) as path:
        return trial.run_bounded_pose_trial(t,{i:f'{i:016x}' for i in t.ids},interrupt,emit,
            absolute_targets=targets,clock=t.clock,wait=t.clock.wait,
            position_response_evidence=path,gain_profile=gain_profile,**kwargs)


def execute_rr_hip(t, **kwargs):
    references=kwargs.pop('matched_start_positions',dict(t.centers))
    targets=dict(references);targets[9]-=math.radians(4)
    return execute(t,targets,matched_start_positions=references,
                   gain_profile='rr_hip_kp6_diagnostic',**kwargs)


class SettlingTransport(PoseTransport):
    """Declared synthetic error about fixed targets, separate from wire codec."""
    def __init__(self, errors=None, **kwargs):
        super().__init__('RR',**kwargs)
        self.errors=errors or (lambda mid,index: 1.4 if mid==9 else 0.)

    def value(self,mid):
        value=super().value(mid)
        if mid in self.enabled and self.commanded:
            index=len(self.motion_batches)-81
            if index>=0:
                target=self.centers[mid]-(math.radians(4) if mid==9 else 0.)
                value=replace(value,protocol_position_rad=target+math.radians(self.errors(mid,index)))
            elif mid==9:
                value=replace(value,protocol_position_rad=value.protocol_position_rad+math.radians(1.8))
        return value


class SettlingObservationRunnerTests(unittest.TestCase):
    def execute(self,t,**kwargs):
        return execute_rr_hip(t,observation_profile='rr_settling_1s',**kwargs)

    def test_unreached_target_can_complete_observation_without_hold_success(self):
        t=SettlingTransport();result=self.execute(t)
        self.assertEqual(result['status'],'RR_SETTLING_OBSERVATION_COMPLETE_RESET_CONFIRMED',result['errors'])
        self.assertTrue(result['settling_observation_completed'] and result['stop_confirmed'])
        self.assertFalse(result['arrival_candidate_met'] or result['hold_candidate_met'])
        self.assertEqual(result['hold_evaluation']['arrival_error_candidate_rad'],math.radians(1))
        self.assertEqual(len(result['settling_observation_samples']),20)
        evaluation=result['settling_observation_evaluation']
        self.assertTrue(evaluation['data_complete'])
        self.assertAlmostEqual(evaluation['first_hold_abs_error_rad']['9'],math.radians(1.4))
        self.assertAlmostEqual(result['settling_first_hold_batch']['abs_target_error_rad'][9],math.radians(1.4))
        for mid in t.ids:
            self.assertEqual(evaluation['motors'][str(mid)]['accepted_samples'],20)
            self.assertGreaterEqual(evaluation['motors'][str(mid)]['observed_span_ns'],900_000_000)
        self.assertEqual(len(t.motion_batches),100)
        self.assertTrue(5. <= t.clock()-t.enable_time <5.1)
        self.assertEqual(sum(f.kind==3 for _,f in t.frames),3)
        self.assertFalse(t.enabled)
        for flag in pose.FLAGS:self.assertFalse(result[flag])

    def test_no_switch_retains_immediate_one_degree_abort_and_no_auto_retry(self):
        t=SettlingTransport();result=execute_rr_hip(t)
        self.assertEqual(result['status'],'ABORTED')
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['motion_completed'])
        self.assertLess(t.clock()-t.enable_time,4.2)
        self.assertNotIn('settling_observation_completed',result)
        self.assertTrue(any('1-degree' in e for e in result['errors']))

    def test_scope_bad_switch_and_missing_fixed_refs_fail_before_io(self):
        for bad in (False,True,1,0,'','rr_settling','RR_SETTLING_1S','rr_settling_1s ',{},[]):
            t=PoseTransport('RR')
            with self.assertRaises(ValueError):execute_rr_hip(t,observation_profile=bad)
            self.assertFalse(t.calls or t.frames or t.stop_calls)
        for gain in ('kp3','kp4_diagnostic'):
            t=PoseTransport('RR')
            with self.assertRaises(ValueError):execute(t,gain_profile=gain,observation_profile='rr_settling_1s')
            self.assertFalse(t.calls or t.frames or t.stop_calls)
        for leg in trial.LEGS:
            t=PoseTransport(leg)
            with self.assertRaises(ValueError):
                execute(t,gain_profile='rr_hip_kp6_diagnostic',observation_profile='rr_settling_1s')
            self.assertFalse(t.calls or t.frames or t.stop_calls)
        import inspect
        self.assertIsNone(inspect.signature(trial.run_bounded_pose_trial).parameters['observation_profile'].default)
        self.assertNotIn('observation_profile',inspect.signature(trial.run_leg_trial).parameters)
        self.assertNotIn('observation',inspect.getsource(trial.main))

    def test_each_axis_aborts_excess_error_worsening_overshoot_and_nonfinite(self):
        for mid in (7,8,9):
            for failure in ('2deg','worsening','overshoot','nan'):
                def errors(i,index):
                    if i!=mid:return 0.
                    if failure=='2deg':return 2.00001
                    if failure=='nan':return math.nan
                    first=1.4
                    return first if index<3 else (-1.65001 if failure=='overshoot' else 1.65001)
                t=SettlingTransport(errors);result=self.execute(t)
                self.assertEqual(result['status'],'ABORTED',(mid,failure,result['errors']))
                self.assertFalse(result['settling_observation_completed'])
                self.assertTrue(result['stop_confirmed'])
                self.assertFalse(t.enabled)
                self.assertLess(t.clock()-t.enable_time,4.3)
        # A falling error never changes the first baseline to a rolling minimum.
        t=SettlingTransport(lambda mid,index: (1.4 if index==0 else (.7 if index<5 else 1.6)) if mid==9 else 0.)
        result=self.execute(t)
        self.assertTrue(result['settling_observation_completed'],result['errors'])
        self.assertAlmostEqual(result['settling_first_hold_batch']['abs_target_error_rad'][9],math.radians(1.4))

    def test_first_batch_is_atomic_and_stale_ramp_values_never_set_baseline(self):
        for fault in ('missing','ramp_source'):
            class BadFirst(SettlingTransport):
                def feedback_many(self,wires,expected_ids):
                    found=super().feedback_many(wires,expected_ids)
                    if len(self.motion_batches)==81:
                        if fault=='missing':found.pop(8)
                        else:
                            value,received=found[8];found[8]=(value,received-.02)
                    return found
            t=BadFirst();result=self.execute(t)
            self.assertEqual(result['status'],'ABORTED')
            self.assertFalse(result['settling_observation_completed'])
            self.assertNotIn('settling_first_hold_batch',result)
            self.assertTrue(result['stop_confirmed']);self.assertFalse(t.enabled)

    def test_twenty_three_ms_batches_complete_but_late_log_or_missing_last_does_not(self):
        t=SettlingTransport(reply_delay_s=.023);result=self.execute(t)
        self.assertTrue(result['settling_observation_completed'],result['errors'])
        class MissingLast(SettlingTransport):
            def feedback_many(self,wires,expected_ids):
                found=super().feedback_many(wires,expected_ids)
                if len(self.motion_batches)==100:found.pop(9)
                return found
        t=MissingLast();result=self.execute(t)
        self.assertFalse(result['settling_observation_completed'])
        self.assertEqual(result['status'],'ABORTED');self.assertTrue(result['stop_confirmed'])
        t=SettlingTransport()
        def emit(row):
            if row['kind']=='leg_trial_motion_sample' and len(t.motion_batches)==100:t.clock.wait(.08)
        result=self.execute(t,emit=emit)
        self.assertFalse(result['settling_observation_completed'])
        self.assertEqual(result['status'],'ABORTED');self.assertTrue(result['stop_confirmed'])

    def test_observation_keeps_torque_feedback_and_final_stop_guards(self):
        for fault in ('torque','stop'):
            class Fault(SettlingTransport):
                def value(self,mid):
                    value=super().value(mid)
                    if fault=='torque' and mid==8 and len(self.motion_batches)>=84:value=replace(value,torque_nm=.50001)
                    return value
                def stop_all(self,ids):
                    reports=super().stop_all(ids)
                    if fault=='stop' and self.enable_time is not None:reports[9].update(confirmed=False,feedback=None)
                    return reports
            t=Fault();result=self.execute(t)
            self.assertFalse(result['settling_observation_completed'])
            self.assertEqual(result['status'],'ABORTED')
            self.assertEqual(t.stop_calls[-1],(7,8,9))
            self.assertFalse(t.enabled)

    def test_successful_tracking_still_uses_observation_specific_status(self):
        t=PoseTransport('RR');result=self.execute(t)
        self.assertTrue(result['arrival_candidate_met'] and result['hold_candidate_met'])
        self.assertTrue(result['settling_observation_completed'])
        self.assertEqual(result['status'],'RR_SETTLING_OBSERVATION_COMPLETE_RESET_CONFIRMED')


class BoundedRunnerTests(unittest.TestCase):
    def test_rr_hip_kp6_changes_only_hip_gain_and_logs_every_axis(self):
        old=PoseTransport('RR');references=dict(old.centers);targets=dict(references)
        targets[9]-=math.radians(4)
        previous=execute(old,targets,gain_profile='kp4_diagnostic',matched_start_positions=references)
        new=PoseTransport('RR');logs=[];validated=[]
        codec=trial.motion_request
        def record_codec(**kwargs):
            if kwargs['phase'] is not trial.TrialPhase.ZERO_GAIN:
                validated.append((new.enable_time,kwargs['motor_id'],kwargs['phase']))
            return codec(**kwargs)
        with patch.object(trial,'motion_request',side_effect=record_codec):
            result=execute_rr_hip(new,emit=logs.append)
        self.assertEqual(result['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',result['errors'])
        self.assertEqual((old.calls,old.stop_calls),(new.calls,new.stop_calls))
        self.assertEqual(len(old.frames),len(new.frames))
        for (ot,of),(nt,nf) in zip(old.frames,new.frames):
            self.assertEqual((ot,of.can_id),(nt,nf.can_id))
            if of.kind==1 and struct.unpack('>4H',of.data)[2] and of.destination==9:
                self.assertEqual(struct.unpack('>4H',nf.data)[2:],(786,1966))
                self.assertEqual(of.data[:4]+of.data[6:],nf.data[:4]+nf.data[6:])
            else:self.assertEqual(of.wire,nf.wire)
        for report in (result,result['bounded_pose_plan']):
            self.assertIsNone(report['Kp'])
            self.assertEqual(report['Kp_by_motor'],{7:4.,8:4.,9:6.})
            self.assertEqual(report['Kd'],.15)
            self.assertEqual(report['torque_feedforward_nm'],0.)
            self.assertEqual(report['max_abs_torque_feedback_candidate_nm'],.5)
            self.assertFalse(report['physical_torque_cap_verified'] or report['automatic_gain_increase'])
        for mid in new.ids:
            expected_phase=(trial.TrialPhase.POSITION_STEP5_RR_HIP_KP6 if mid==9
                            else trial.TrialPhase.POSITION_STEP5_KP4)
            used=[(when,phase) for when,i,phase in validated if i==mid]
            self.assertGreaterEqual(sum(when is None for when,_ in used),3)
            self.assertTrue(all(phase is expected_phase for _,phase in used))
            self.assertEqual(result['motors'][mid]['Kp'],6. if mid==9 else 4.)
            rows=[r for r in logs if r['kind']=='leg_trial_motion_sample' and r['motor_id']==mid]
            self.assertEqual(len(rows),100)
            self.assertTrue(all(r['Kp']==(6. if mid==9 else 4.) and r['motion_phase']==expected_phase.value
                                and r['Kd']==.15 and r['torque_feedforward_nm']==0. for r in rows))
        self.assertEqual(previous['Kp'],4.)
        self.assertTrue(result['matched_start_check']['passed'] and result['stop_confirmed'])
        self.assertEqual(result['hold_evaluation']['motors']['9']['accepted_samples'],20)
        self.assertEqual(sum(f.kind==3 for _,f in new.frames),3)
        self.assertFalse(new.enabled)

    def test_rr_hip_kp6_scope_targets_evidence_and_profile_required_before_io(self):
        for leg in ('FR','FL','RL'):
            t=PoseTransport(leg)
            with self.assertRaises(ValueError):
                execute(t,gain_profile='rr_hip_kp6_diagnostic',matched_start_positions=t.centers)
            self.assertFalse(t.calls or t.frames or t.stop_calls)
        for failure in ('no_reference','old_targets','id7','id8','hip_plus','hip_other','wrap',
                        'legacy','wrong_evidence','relaxed_tolerance'):
            t=PoseTransport('RR');refs=dict(t.centers);targets=dict(refs);targets[9]-=math.radians(4)
            kwargs={'matched_start_positions':refs}
            if failure=='no_reference':kwargs.clear()
            if failure=='old_targets':targets={i:refs[i]+math.radians(d) for i,d in zip(t.ids,(4,4,-4))}
            if failure in ('id7','id8'):targets[int(failure[2:])]=math.nextafter(refs[int(failure[2:])],math.inf)
            if failure=='hip_plus':targets[9]=refs[9]+math.radians(4)
            if failure=='hip_other':targets[9]=math.nextafter(targets[9],math.inf)
            if failure=='wrap':targets[9]+=2*math.pi
            if failure=='legacy':kwargs['profile']='legacy-rms-v1'
            if failure=='wrong_evidence':kwargs['evidence_leg']='FL'
            if failure=='relaxed_tolerance':kwargs['matched_start_tolerance_rad']=math.radians(.6)
            with self.subTest(failure=failure),self.assertRaises(ValueError):
                execute(t,targets,gain_profile='rr_hip_kp6_diagnostic',**kwargs)
            self.assertFalse(t.calls or t.frames or t.stop_calls)
        t=PoseTransport('RR');refs=dict(t.centers);targets=dict(refs);targets[9]-=math.radians(4)
        with self.assertRaises(ValueError):
            trial.run_bounded_pose_trial(t,{i:f'{i:016x}' for i in t.ids},lambda:None,lambda r:None,
                absolute_targets=targets,matched_start_positions=refs,gain_profile='rr_hip_kp6_diagnostic')
        self.assertFalse(t.calls or t.frames or t.stop_calls)
        import inspect
        self.assertEqual(inspect.signature(trial.run_bounded_pose_trial).parameters['gain_profile'].default,'kp3')
        self.assertNotIn('rr_hip_kp6',inspect.getsource(trial.main))

    def test_rr_hip_kp6_matched_failure_stops_before_enable_and_never_recenters(self):
        for mid in (7,8,9):
            for delta in (math.radians(.501),-math.radians(.501),2*math.pi):
                t=PoseTransport('RR');refs=dict(t.centers);refs[mid]+=delta
                result=execute_rr_hip(t,matched_start_positions=refs)
                self.assertEqual(result['status'],'ABORTED')
                self.assertTrue(result['stop_confirmed'])
                self.assertFalse(any(f.kind==3 or (f.kind==1 and struct.unpack('>4H',f.data)[2]) for _,f in t.frames))
                self.assertEqual(result['matched_start_positions_rad'],refs)
                self.assertFalse(result['automatic_repositioning'])

    def test_rr_hip_kp6_torque_monitor_at_initial_settled_enabled_and_motion(self):
        for stage in ('initial','settled','enabled','motion'):
            for mid in (7,8,9):
                for bad in (.500001,-.500001,math.nan,True):
                    class TorqueTransport(PoseTransport):
                        def stop_all(self,ids):
                            reports=super().stop_all(ids)
                            if stage=='initial' and len(self.stop_calls)==1:reports[mid]['feedback']['torque_nm']=bad
                            return reports
                        def value(self,i):
                            value=super().value(i)
                            fail=(stage=='settled' and self.window_count>0 and not self.enabled
                                  or stage=='enabled' and self.enabled and not self.commanded
                                  or stage=='motion' and self.commanded)
                            return replace(value,torque_nm=bad) if fail and i==mid else value
                    t=TorqueTransport('RR');result=execute_rr_hip(t)
                    self.assertEqual(result['status'],'ABORTED',(stage,mid,bad))
                    self.assertTrue(result['stop_confirmed'])
                    self.assertFalse(t.enabled)
                    self.assertTrue(any('torque feedback' in e for e in result['errors']),result['errors'])
                    if stage in ('initial','settled'):self.assertFalse(any(f.kind==3 for _,f in t.frames))
                    self.assertLessEqual(len(t.motion_batches),1)
        for boundary in (-.5,.5):
            class Boundary(PoseTransport):
                def value(self,i):return replace(super().value(i),torque_nm=boundary)
            self.assertEqual(execute_rr_hip(Boundary('RR'))['status'],
                             'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED')

    def test_rr_hip_kp6_prewrite_monitor_catches_sibling_before_nonzero_tx(self):
        for mid in (7,8,9):
            class BeforeWrite(PoseTransport):
                def send(self,wire):
                    frame=ATParser().feed(wire)[0]
                    if frame.kind==1 and struct.unpack('>4H',frame.data)[2] and self.pre_send_guard:
                        value,received=self.latest[mid]
                        self.latest[mid]=(replace(value,torque_nm=.51),received)
                        self.pre_send_guard()
                    super().send(wire)
            t=BeforeWrite('RR');result=execute_rr_hip(t)
            self.assertEqual(result['status'],'ABORTED')
            self.assertFalse(any(f.kind==1 and struct.unpack('>4H',f.data)[2] for _,f in t.frames))
            self.assertTrue(result['stop_confirmed']);self.assertFalse(t.enabled)

    def test_rr_hip_kp6_preserves_arrival_stop_and_no_retry_on_undertracking(self):
        for failure in ('not_tracking','partial_tracking'):
            t=PoseTransport('RR',failure=failure);result=execute_rr_hip(t)
            self.assertEqual(result['status'],'ABORTED')
            self.assertTrue(result['stop_confirmed'])
            self.assertFalse(result['motion_completed'] or result['hold_candidate_met'])
            self.assertLess(t.clock()-t.enable_time,4.2)
            self.assertEqual(sum(f.kind==3 for _,f in t.frames),3)
            self.assertTrue(any('1-degree' in e for e in result['errors']))
            self.assertFalse(t.enabled)

    def test_rr_hip_kp6_third_watchdog_read_rejection_never_writes_settings(self):
        class RejectedWatchdog(PoseTransport):
            def parameter(self,mid,name=None):
                if mid==9 and name=='can_timeout':raise RuntimeError('synthetic ID9 watchdog read rejected')
                return super().parameter(mid,name)
        t=RejectedWatchdog('RR');result=execute_rr_hip(t)
        self.assertEqual(result['status'],'ABORTED')
        self.assertFalse(any(f.kind in (3,18) for _,f in t.frames))
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(t.stop_calls[-1],(7,8,9))
        self.assertEqual(result['motors'][7]['watchdog_previous_ticks'],0)
        self.assertEqual(result['motors'][8]['watchdog_previous_ticks'],0)

    def test_rr_hip_kp6_failed_final_stop_cannot_report_success(self):
        class FailedStop(PoseTransport):
            def stop_all(self,ids):
                reports=super().stop_all(ids)
                if self.enable_time is not None:
                    reports[9].update(confirmed=False,feedback=None,error='synthetic missing stop reply')
                return reports
        t=FailedStop('RR');result=execute_rr_hip(t)
        self.assertEqual(result['status'],'ABORTED')
        self.assertFalse(result['stop_confirmed'])
        self.assertTrue(any('STOP_UNCONFIRMED' in e for e in result['errors']))
        self.assertEqual([f.destination for _,f in t.frames[-3:]], [7,8,9])
        self.assertTrue(all(f.kind==4 for _,f in t.frames[-3:]))
        self.assertEqual(sum(f.kind==3 for _,f in t.frames),3)

    def test_matched_start_hip_only_fixed_targets_preserve_other_axes(self):
        t=PoseTransport('RR');references=dict(t.centers);targets=dict(references)
        targets[9]-=math.radians(4)
        result=execute(t,targets,gain_profile='kp4_diagnostic',matched_start_positions=references)
        self.assertEqual(result['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',result['errors'])
        self.assertTrue(result['matched_start_check']['passed'])
        self.assertEqual(result['matched_start_positions_rad'],references)
        self.assertEqual(result['bounded_pose_plan']['matched_start_positions_rad'],references)
        self.assertEqual(result['matched_start_tolerance_rad'],math.radians(.5))
        self.assertFalse(result['automatic_repositioning'])
        for mid in (7,8):self.assertEqual(result['motors'][mid]['target_final_offset_rad'],0.)
        self.assertAlmostEqual(result['motors'][9]['target_final_offset_rad'],-math.radians(4))

    def test_matched_start_rejects_any_axis_before_all_enable_without_repositioning(self):
        for mid in (7,8,9):
            for offset in (math.radians(.501),-math.radians(.501),2*math.pi):
                t=PoseTransport('RR');references=dict(t.centers);references[mid]-=offset
                result=execute(t,matched_start_positions=references)
                self.assertEqual(result['status'],'ABORTED')
                self.assertFalse(result['matched_start_check']['passed'])
                self.assertTrue(result['stop_confirmed'])
                self.assertFalse(any(f.kind==3 or (f.kind==1 and struct.unpack('>4H',f.data)[2]) for _,f in t.frames))

    def test_matched_start_uses_final_type2_not_initial_type17_and_copies_reference(self):
        class FinalType2(PoseTransport):
            def value(self,mid):
                value=super().value(mid)
                return replace(value,protocol_position_rad=value.protocol_position_rad+.009) if mid==8 else value
        t=FinalType2('RR');references=dict(t.centers)
        result=execute(t,matched_start_positions=references)
        self.assertTrue(result['settled_window']['passed'])
        self.assertEqual(result['status'],'ABORTED')
        self.assertAlmostEqual(result['matched_start_check']['motors']['8']['delta_rad'],.009)
        self.assertFalse(any(f.kind==3 for _,f in t.frames))
        t=PoseTransport('RR');references=dict(t.centers)
        def emit(row):
            if row['kind']=='leg_trial_settled_window':references[8]+=1.
        result=execute(t,matched_start_positions=references,emit=emit)
        self.assertTrue(result['matched_start_check']['passed'])
        self.assertEqual(result['matched_start_positions_rad'][8],t.centers[8])

    def test_matched_start_validation_rejects_bad_inputs_before_io(self):
        for bad in ({}, {7:0.,8:0.},{1:0.,2:0.,3:0.},{7:0.,'7':0.,8:0.,9:0.},
                    {7:True,8:0.,9:0.},{7:math.nan,8:0.,9:0.},{7:math.inf,8:0.,9:0.}):
            t=PoseTransport('RR')
            with self.assertRaises(ValueError):execute(t,matched_start_positions=bad)
            self.assertFalse(t.calls or t.frames or t.stop_calls)
        for tolerance in (math.radians(.6),0.,True,None,math.nan):
            t=PoseTransport('RR')
            with self.assertRaises(ValueError):
                execute(t,matched_start_positions=t.centers,matched_start_tolerance_rad=tolerance)
            self.assertFalse(t.calls or t.frames or t.stop_calls)

    def test_matched_start_rechecked_at_every_enable_and_does_not_reset_guard(self):
        class GuardedEnable(PoseTransport):
            def send(self,wire):
                frame=ATParser().feed(wire)[0]
                if frame.kind==3:self.pre_enable_guard()
                super().send(wire)
        t=GuardedEnable('RR')
        with patch.object(pose,'evaluate_matched_start',wraps=pose.evaluate_matched_start) as check:
            result=execute(t,matched_start_positions=t.centers)
        self.assertTrue(result['matched_start_check']['passed'])
        self.assertEqual(check.call_count,4) # final plan + immediately before each of3 enables

    def test_explicit_kp4_only_changes_gain_and_records_monitor_in_plan(self):
        old=PoseTransport('RR');previous=execute(old)
        new=PoseTransport('RR');result=execute(new,gain_profile='kp4_diagnostic')
        self.assertEqual(result['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED')
        self.assertEqual(old.calls,new.calls)
        self.assertEqual(old.stop_calls,new.stop_calls)
        self.assertEqual(len(old.frames),len(new.frames))
        for (ot,of),(nt,nf) in zip(old.frames,new.frames):
            self.assertEqual((ot,of.can_id),(nt,nf.can_id))
            if of.kind==1 and struct.unpack('>4H',of.data)[2]:
                self.assertEqual(struct.unpack('>4H',nf.data)[2:],(524,1966))
                self.assertEqual(of.data[:4]+of.data[6:],nf.data[:4]+nf.data[6:])
            else:self.assertEqual(of.wire,nf.wire)
        for report in (result,result['bounded_pose_plan']):
            self.assertEqual((report['gain_profile'],report['Kp'],report['Kd']),('kp4_diagnostic',4.,.15))
            self.assertEqual(report['torque_feedforward_nm'],0.)
            self.assertEqual(report['max_abs_torque_feedback_candidate_nm'],.5)
            self.assertFalse(report['physical_torque_cap_verified'] or report['automatic_gain_increase'])
        self.assertEqual(previous['gain_profile'],'kp3')
        self.assertIsNone(previous['max_abs_torque_feedback_candidate_nm'])

    def test_invalid_gain_profiles_fail_before_any_io(self):
        for profile in (None,True,4,4.,'kp4','KP4_DIAGNOSTIC','kp4_diagnostic ',{},[]):
            t=PoseTransport()
            with self.assertRaises(ValueError):execute(t,gain_profile=profile)
            self.assertFalse(t.calls or t.frames or t.stop_calls)
        import inspect
        self.assertNotIn('gain_profile',inspect.signature(trial.run_leg_trial).parameters)
        self.assertNotIn('--gain-profile',inspect.getsource(trial.main))

    def test_kp4_feedback_monitor_rejects_excess_nonfinite_and_preserves_kp3(self):
        for torque in (.500001,-.500001,math.nan,math.inf,-math.inf,True):
            class TorqueTransport(PoseTransport):
                def value(self,mid):
                    value=super().value(mid)
                    return replace(value,torque_nm=torque) if self.commanded and mid==self.ids[1] else value
            t=TorqueTransport('RR');result=execute(t,gain_profile='kp4_diagnostic')
            self.assertEqual(result['status'],'ABORTED')
            self.assertTrue(result['stop_confirmed'])
            self.assertFalse(t.enabled or result['hold_candidate_met'])
            self.assertLess(t.clock()-t.enable_time,.1)
            self.assertTrue(any('torque feedback' in error for error in result['errors']))
        for torque in (-.5,.5):
            class BoundaryTransport(PoseTransport):
                def value(self,mid):return replace(super().value(mid),torque_nm=torque)
            self.assertEqual(execute(BoundaryTransport(),gain_profile='kp4_diagnostic')['status'],
                             'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED')
        class OldTorqueTransport(PoseTransport):
            def value(self,mid):return replace(super().value(mid),torque_nm=.51)
        self.assertEqual(execute(OldTorqueTransport())['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED')

    def test_kp4_prewrite_guard_catches_changed_sibling_torque_before_tx(self):
        class BeforeWrite(PoseTransport):
            def send(self,wire):
                frame=ATParser().feed(wire)[0]
                if frame.kind==1 and struct.unpack('>4H',frame.data)[2] and self.pre_send_guard:
                    value,received=self.latest[self.ids[-1]]
                    self.latest[self.ids[-1]]=(replace(value,torque_nm=.51),received)
                    self.pre_send_guard() # Real transport checks after pacing, before UART write.
                super().send(wire)
        t=BeforeWrite('RR');result=execute(t,gain_profile='kp4_diagnostic')
        self.assertEqual(result['status'],'ABORTED')
        self.assertFalse(any(f.kind==1 and struct.unpack('>4H',f.data)[2] for _,f in t.frames))
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(t.enabled)

    def test_kp4_disabled_torque_failure_prevents_enable(self):
        class DisabledTorque(PoseTransport):
            def value(self,mid):return replace(super().value(mid),torque_nm=.51)
        t=DisabledTorque();result=execute(t,gain_profile='kp4_diagnostic')
        self.assertEqual(result['status'],'ABORTED')
        self.assertFalse(any(f.kind==3 for _,f in t.frames))
        self.assertTrue(result['stop_confirmed'])

    def test_twenty_three_ms_batch_completion_with_fixed_start_schedule_can_pass(self):
        t=PoseTransport(reply_delay_s=.023);result=execute(t)
        self.assertEqual(result['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',result['errors'])
        self.assertTrue(result['hold_candidate_met'] and result['stop_confirmed'])

    def test_exact_five_degree_endpoint_is_codec_checked_before_enable(self):
        t=PoseTransport();t.centers={i:1. for i in t.ids}
        targets={1:1.-pose.MAX_DELTA_RAD,2:1.+pose.MAX_DELTA_RAD,3:1.}
        result=execute(t,targets)
        self.assertEqual(result['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',result['errors'])
        self.assertEqual(result['motors'][1]['target_final_offset_rad'],-pose.MAX_DELTA_RAD)
        self.assertEqual(result['motors'][2]['target_final_offset_rad'],pose.MAX_DELTA_RAD)

    def test_each_leg_tracks_explicit_target_hold_candidate_then_stops(self):
        for leg in trial.LEGS:
            with self.subTest(leg=leg):
                t=PoseTransport(leg);result=execute(t)
                self.assertEqual(result['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',result['errors'])
                self.assertTrue(result['trajectory_elapsed'] and result['arrival_candidate_met'] and result['hold_candidate_met'])
                self.assertTrue(result['motion_completed'] and result['stop_confirmed'])
                self.assertEqual(t.stop_calls[-1],t.ids)
                self.assertFalse(t.enabled)
                for flag in pose.FLAGS:self.assertFalse(result[flag])
                for row in result['hold_evaluation']['motors'].values():
                    self.assertEqual(row['accepted_samples'],20)
                    self.assertGreaterEqual(row['observed_span_ns'],900_000_000)
                    self.assertLess(row['observed_span_ns'],1_000_000_000)
                self.assertEqual(result['directions'],None)

    def test_static_or_partial_tracking_is_not_success_and_stops_at_first_hold(self):
        for failure in ('not_tracking','partial_tracking'):
            t=PoseTransport(failure=failure);result=execute(t)
            self.assertEqual(result['status'],'ABORTED')
            self.assertFalse(result['motion_completed'] or result['hold_candidate_met'])
            self.assertTrue(result['stop_confirmed'])
            self.assertEqual(t.stop_calls[-1],t.ids)
            self.assertLess(t.clock()-t.enable_time,4.2)
            self.assertTrue(any('1-degree' in e for e in result['errors']))

    def test_arrives_then_loses_hold_stops_without_retry(self):
        t=PoseTransport(failure='hold_drift');result=execute(t)
        self.assertEqual(result['status'],'ABORTED')
        self.assertTrue(result['arrival_candidate_met'])
        self.assertFalse(result['hold_candidate_met'])
        self.assertTrue(result['stop_confirmed'])
        self.assertLess(t.clock()-t.enable_time,4.5)
        self.assertEqual(sum(f.kind==3 for _,f in t.frames),3)

    def test_invalid_target_structure_none_nonfinite_wrong_leg_fail_before_io(self):
        for targets in (None,{}, {4:0.,5:0.,6:0.}, {1:float('nan'),2:0.,3:0.},
                        {1:0.,'1':0.,2:0.,3:0.}, {1:True,2:0.,3:0.}):
            t=PoseTransport()
            with self.assertRaises(ValueError):
                trial.run_bounded_pose_trial(t,{i:f'{i:016x}' for i in t.ids},lambda:None,lambda r:None,
                    absolute_targets=targets,clock=t.clock,wait=t.clock.wait)
            self.assertFalse(t.calls or t.frames or t.stop_calls)

    def test_far_targets_and_one_revolution_fail_before_watchdog_or_enable(self):
        for offset in (math.radians(5.1),2*math.pi):
            t=PoseTransport();targets=dict(t.centers);targets[1]+=offset
            result=execute(t,targets)
            self.assertEqual(result['status'],'ABORTED')
            self.assertFalse(any(f.kind in (3,18) for _,f in t.frames))
            self.assertTrue(result['stop_confirmed'])

    def test_missing_wrong_scope_or_changed_evidence_blocks_before_io(self):
        t=PoseTransport('RR')
        with self.assertRaises(ValueError):execute(t,evidence_leg='FL')
        self.assertFalse(t.calls or t.frames or t.stop_calls)
        with self.assertRaises(ValueError):
            trial.run_bounded_pose_trial(t,{i:f'{i:016x}' for i in t.ids},lambda:None,lambda r:None,
                absolute_targets=t.centers,clock=t.clock,wait=t.clock.wait)
        self.assertFalse(t.calls or t.frames or t.stop_calls)

    def test_guard_communication_partial_enable_and_signal_failures_always_stop(self):
        for failure in ('identity','headroom','watchdog','prior_watchdog_rejected','partial_enable',
                        'communication','overspeed','fault','stale'):
            t=PoseTransport(failure=failure);result=execute(t)
            self.assertEqual(result['status'],'ABORTED',failure)
            self.assertTrue(t.stop_calls,failure)
            self.assertFalse(t.enabled,failure)
            self.assertTrue(result['stop_confirmed'],failure)
        t=PoseTransport()
        def interrupt():
            if t.enable_time is not None and t.clock()-t.enable_time>.4:raise KeyboardInterrupt('synthetic')
        result=execute(t,interrupt=interrupt)
        self.assertEqual(result['status'],'ABORTED');self.assertTrue(result['stop_confirmed'])

    def test_stop_failure_never_reports_pose_success(self):
        t=PoseTransport(failure='stop');result=execute(t)
        self.assertEqual(result['status'],'ABORTED')
        self.assertFalse(result['stop_confirmed'])
        self.assertTrue(any('STOP_UNCONFIRMED' in e for e in result['errors']))

    def test_stale_after_settled_window_blocks_every_enable(self):
        t=PoseTransport()
        def emit(row):
            if row['kind']=='leg_trial_settled_window':t.clock.wait(.11)
        result=execute(t,emit=emit)
        self.assertEqual(result['status'],'ABORTED')
        self.assertFalse(any(f.kind==3 for _,f in t.frames))
        self.assertTrue(result['stop_confirmed'])

    def test_stale_request_with_still_fresh_reply_blocks_each_enable(self):
        for delayed_id in (1,2,3):
            class DelayedEnable(PoseTransport):
                def send(self,wire):
                    frame=ATParser().feed(wire)[0]
                    if frame.kind==3:
                        if frame.destination==delayed_id:self.clock.wait(.096)
                        # Real transport applies this after pacing, before write.
                        if self.pre_enable_guard:self.pre_enable_guard()
                    super().send(wire)
            t=DelayedEnable();result=execute(t)
            self.assertEqual(result['status'],'ABORTED')
            self.assertTrue(any('request/receive aged before enable' in e for e in result['errors']))
            self.assertEqual([f.destination for _,f in t.frames if f.kind==3],list(range(1,delayed_id)))
            self.assertFalse(t.motion_batches or t.enabled)
            self.assertTrue(result['stop_confirmed'])

    def test_diagnostic_exception_or_log_failure_cannot_skip_stop(self):
        t=PoseTransport()
        with patch.object(pose,'evaluate_hold',side_effect=ValueError('synthetic diagnostic failure')):
            result=execute(t)
        self.assertEqual(result['status'],'ABORTED');self.assertTrue(result['stop_confirmed'])
        self.assertFalse(t.enabled)
        t=PoseTransport()
        def emit(row):
            if row['kind']=='leg_trial_motion_sample':raise IOError('log failed')
        result=execute(t,emit=emit)
        self.assertEqual(result['status'],'ABORTED');self.assertTrue(result['stop_confirmed'])

    def test_candidate_limits_are_not_changed_and_existing_cli_has_no_absolute_switch(self):
        self.assertEqual((trial.DURATION_S,trial.ACTIVE_BUDGET_S,trial.CYCLE_S,trial.MIN_TX_INTERVAL_S),(5.,6.,.05,.005))
        self.assertAlmostEqual(trial.MAX_DRIFT_RAD,math.radians(7))
        import inspect
        self.assertNotIn('absolute_targets',inspect.signature(trial.run_leg_trial).parameters)
        self.assertNotIn('--absolute',inspect.getsource(trial.main))


if __name__=='__main__':unittest.main()
