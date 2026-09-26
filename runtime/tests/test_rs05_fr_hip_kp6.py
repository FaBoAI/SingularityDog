"""Offline tests for explicitly scoped FR hip-only Kp6 diagnostics."""
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


def run(transport, **changes):
    options = dict(absolute_targets={i:q+math.radians(4) for i,q in transport.centers.items()},
                   matched_start_positions=dict(transport.centers), profile='position-v2',
                   gain_profile='fr_hip_kp6_diagnostic', enable_profile='sequential-confirmed')
    options.update(changes)
    with evidence_file('FR') as path:
        return trial.run_bounded_pose_trial(transport, {i:f'{i:016x}' for i in transport.ids},
            lambda:None, lambda _:None, clock=transport.clock, wait=transport.clock.wait,
            position_response_evidence=path, **options)


class FRHipKp6Tests(unittest.TestCase):
    def test_codec_phase_is_id3_only_and_changes_only_kp_from_existing_profile(self):
        for mid in range(1,13):
            if mid != 3:
                with self.subTest(mid=mid), self.assertRaises(ValueError):
                    motion_request(phase=TrialPhase.POSITION_STEP5_FR_HIP_KP6, center_rad=5., motor_id=mid)
        for offset in (-math.radians(5),0.,math.radians(5)):
            old=ATParser().feed(motion_request(phase=TrialPhase.POSITION_STEP5, center_rad=5.,offset_rad=offset,motor_id=3))[0]
            new=ATParser().feed(motion_request(phase=TrialPhase.POSITION_STEP5_FR_HIP_KP6, center_rad=5.,offset_rad=offset,motor_id=3))[0]
            self.assertEqual(old.can_id,new.can_id)
            self.assertEqual(old.data[:4]+old.data[6:],new.data[:4]+new.data[6:])
            self.assertEqual(struct.unpack('>4H',new.data)[2],786)

    def test_success_exact_mixed_gains_all_hundred_commands_and_existing_hold_coverage(self):
        t=SequentialEnableTransport();r=run(t)
        self.assertEqual(r['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',r['errors'])
        self.assertIsNone(r['Kp'])
        self.assertEqual(r['Kp_by_motor'],{1:3.,2:3.,3:6.})
        self.assertEqual(r['max_abs_torque_feedback_candidate_nm'],.5)
        self.assertEqual(r['sequential_enable_confirmed_ids'],[1,2,3])
        for mid in t.ids:
            active=[(when,f) for when,f in t.frames if f.kind==1 and f.destination==mid and f.data[4:8]!=bytes(4)]
            self.assertEqual(len(active),100)
            self.assertLess(active[-1][0]-active[0][0],5.)
            for _,f in active:
                _,velocity,kp,kd=struct.unpack('>4H',f.data)
                self.assertEqual((velocity,kp,kd),(32767,786 if mid==3 else 393,1966))
                self.assertEqual(f.can_id, (1<<24)|(32767<<8)|mid)
        self.assertTrue(r['hold_candidate_met'])
        self.assertTrue(r['stop_confirmed'])
        self.assertEqual(t.stop_calls[-1],(1,2,3))

    def test_wrong_leg_profile_unmatched_burst_or_over4point5_reference_delta_reject_before_io(self):
        changes=[{'profile':'legacy-rms-v1'},{'profile':'position-v2-all'},
                 {'matched_start_positions':None},{'enable_profile':'legacy-burst'}]
        for mid in (1,2,3):
            t=SequentialEnableTransport();targets=dict(t.centers);targets[mid]+=math.radians(4.5001)
            changes.append({'absolute_targets':targets})
        for change in changes:
            t=SequentialEnableTransport()
            with self.subTest(change=change),self.assertRaises(ValueError):run(t,**change)
            self.assertEqual((t.calls,t.frames,t.stop_calls),([],[],[]))
        for leg in ('FL','RR','RL'):
            t=PoseTransport(leg)
            with self.subTest(leg=leg),self.assertRaises(ValueError):run(t)
            self.assertEqual((t.calls,t.frames,t.stop_calls),([],[],[]))

    def test_all_three_torque_channels_abort_and_stop_on_overlimit_or_nonfinite(self):
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

    def test_enable_fault_and_unchanged_one_degree_arrival_error_abort_and_stop(self):
        for failure in ('fault','hold_drift'):
            t=SequentialEnableTransport('fault' if failure=='fault' else None)
            if failure=='hold_drift':t.failure='hold_drift'
            with self.subTest(failure=failure):
                r=run(t)
                self.assertEqual(r['status'],'ABORTED',r['errors'])
                self.assertTrue(r['stop_confirmed']);self.assertEqual(t.stop_calls[-1],(1,2,3))
                if failure=='fault':
                    self.assertFalse(any(f.kind==1 and f.data[4:8]!=bytes(4) for _,f in t.frames))
                else:self.assertTrue(any('1-degree' in e for e in r['errors']))


if __name__=='__main__':unittest.main()
