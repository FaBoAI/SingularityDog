"""Offline fixed-reference FR Kp3/12/12 scope, wire and guard tests."""
from dataclasses import replace
import math
import struct
import unittest

from singularitydog_hw import rs05_leg_trial as trial
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import TrialPhase, motion_request
from test_position_response_evidence import evidence_file
from test_rs05_serial_enable import SequentialEnableTransport
from test_rs05_bounded_pose_trial import PoseTransport


PHASES = {1: TrialPhase.POSITION_STEP5,
          2: TrialPhase.POSITION_CURRENT_FR_THIGH_KP12,
          3: TrialPhase.POSITION_CURRENT_FR_HIP_KP12}


def run(transport, references=None, **changes):
    references = dict(transport.centers) if references is None else dict(references)
    options = dict(absolute_targets=dict(references), matched_start_positions=references,
        profile='position-v2', gain_profile='fr_current_kp12_diagnostic',
        enable_profile='sequential-confirmed')
    with evidence_file('FR') as path:
        options['position_response_evidence'] = path
        options.update(changes)
        return trial.run_bounded_pose_trial(transport, {i:f'{i:016x}' for i in transport.ids},
            lambda:None, lambda _:None, clock=transport.clock, wait=transport.clock.wait, **options)


class FRCurrentKp12Tests(unittest.TestCase):
    def test_codec_restricts_each_phase_to_its_motor_and_exact_zero_offset(self):
        for selected in (2,3):
            for mid in range(1,13):
                with self.subTest(selected=selected,mid=mid):
                    if mid == selected:
                        frame=ATParser().feed(motion_request(phase=PHASES[selected],center_rad=5.,motor_id=mid))[0]
                        self.assertEqual(struct.unpack('>4H',frame.data)[1:],(32767,1572,1966))
                    else:
                        with self.assertRaises(ValueError):
                            motion_request(phase=PHASES[selected],center_rad=5.,motor_id=mid)
            for offset in (-math.radians(5),-1e-12,1e-12,math.radians(5)):
                with self.subTest(selected=selected,offset=offset),self.assertRaises(ValueError):
                    motion_request(phase=PHASES[selected],center_rad=5.,offset_rad=offset,motor_id=selected)

    def test_hundred_constant_reference_packets_even_at_half_degree_fresh_center_mismatch(self):
        for shift in (0.,math.radians(.5),-math.radians(.5)):
            t=SequentialEnableTransport()
            references={i:q+shift for i,q in t.centers.items()}
            with self.subTest(shift=shift):
                r=run(t,references)
                self.assertEqual(r['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',r['errors'])
                self.assertEqual(r['Kp_by_motor'],{1:3.,2:12.,3:12.})
                self.assertTrue(r['current_position_only']);self.assertFalse(r['target_ramp_applied'])
                self.assertEqual(r['bounded_pose_plan']['ramp_ns'],0)
                self.assertEqual(r['bounded_pose_plan']['hold_ns'],5_000_000_000)
                self.assertEqual(r['bounded_pose_plan']['final_evaluation_start_ns'],4_000_000_000)
                self.assertEqual(r['sequential_enable_confirmed_ids'],[1,2,3])
                self.assertTrue(r['hold_candidate_met']);self.assertTrue(r['stop_confirmed'])
                self.assertFalse(r['continuous_hold_proven'])
                for mid in t.ids:
                    commands=[(when,f.wire) for when,f in t.frames
                              if f.destination==mid and f.kind==1 and f.data[4:8]!=bytes(4)]
                    expected=motion_request(phase=PHASES[mid],center_rad=references[mid],motor_id=mid)
                    self.assertEqual([wire for _,wire in commands],[expected]*100)
                    self.assertLess(commands[-1][0]-commands[0][0],5.)
                self.assertEqual(t.stop_calls[-1],(1,2,3))

    def test_any_added_target_motion_or_wrong_scope_or_missing_evidence_rejects_before_io(self):
        changes=[{'matched_start_positions':None},{'profile':'legacy-rms-v1'},
                 {'profile':'position-v2-all'},{'enable_profile':'legacy-burst'},
                 {'position_response_evidence':None}]
        for mid in (1,2,3):
            t=SequentialEnableTransport();targets=dict(t.centers);targets[mid]+=1e-12
            changes.append({'absolute_targets':targets})
        for change in changes:
            t=SequentialEnableTransport()
            with self.subTest(change=change),self.assertRaises(ValueError):run(t,**change)
            self.assertEqual((t.calls,t.frames,t.stop_calls),([],[],[]))
        for leg in ('FL','RR','RL'):
            t=PoseTransport(leg)
            with self.subTest(leg=leg),self.assertRaises(ValueError):run(t)
            self.assertEqual((t.calls,t.frames,t.stop_calls),([],[],[]))

    def test_outside_half_degree_match_stops_without_any_enable(self):
        t=SequentialEnableTransport()
        r=run(t,{i:q+math.radians(.5001) for i,q in t.centers.items()})
        self.assertEqual(r['status'],'ABORTED',r['errors'])
        self.assertFalse(any(f.kind==3 for _,f in t.frames))
        self.assertTrue(r['stop_confirmed']);self.assertEqual(t.stop_calls[-1],(1,2,3))

    def test_all_torque_channels_keep_half_nm_abort_and_all_stop(self):
        for mid in (1,2,3):
            for torque in (.5001,float('nan')):
                class TorqueTransport(SequentialEnableTransport):
                    def value(self,motor_id):
                        value=super().value(motor_id)
                        return replace(value,torque_nm=torque) if motor_id==mid and self.commanded else value
                t=TorqueTransport()
                with self.subTest(mid=mid,torque=torque):
                    r=run(t)
                    self.assertEqual(r['status'],'ABORTED')
                    self.assertTrue(any('0.5Nm' in e for e in r['errors']))
                    self.assertTrue(r['stop_confirmed']);self.assertEqual(t.stop_calls[-1],(1,2,3))

    def test_mode_fault_stale_and_final_one_degree_error_remain_fail_closed(self):
        for failure in ('mode0','fault','stale','hold_drift'):
            t=SequentialEnableTransport(failure if failure!='hold_drift' else None)
            if failure=='hold_drift':t.failure='hold_drift'
            with self.subTest(failure=failure):
                r=run(t)
                self.assertEqual(r['status'],'ABORTED',r['errors'])
                self.assertTrue(r['stop_confirmed']);self.assertEqual(t.stop_calls[-1],(1,2,3))
                if failure=='hold_drift':self.assertTrue(any('1-degree' in e for e in r['errors']))
                else:self.assertFalse(any(f.kind==1 and f.data[4:8]!=bytes(4) for _,f in t.frames))


if __name__=='__main__':unittest.main()
