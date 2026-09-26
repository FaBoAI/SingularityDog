"""Offline explicit FR Kp3/12/12 four-degree ramp tests."""
from dataclasses import replace
import math
import struct
import unittest

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import TrialPhase, motion_request
from test_rs05_serial_enable import SequentialEnableTransport
from test_rs05_bounded_pose_trial import PoseTransport
from test_rs05_fr_current_kp12 import run as hold_run


PHASES = {1:TrialPhase.POSITION_STEP5, 2:TrialPhase.POSITION_STEP4_FR_THIGH_KP12,
          3:TrialPhase.POSITION_STEP4_FR_HIP_KP12}


def run(t, references=None, direction=1, **changes):
    refs = dict(t.centers) if references is None else dict(references)
    options = {'gain_profile':'fr_step4_kp12_diagnostic',
               'absolute_targets':{i:q+direction*math.radians(4) for i,q in refs.items()}}
    options.update(changes)
    return hold_run(t,refs,**options)


class FRStep4Kp12Tests(unittest.TestCase):
    def test_codec_is_selected_axis_only_and_bounded_to_four_point_five_degrees(self):
        for selected in (2,3):
            for mid in range(1,13):
                if mid != selected:
                    with self.subTest(selected=selected,mid=mid),self.assertRaises(ValueError):
                        motion_request(phase=PHASES[selected],center_rad=5.,motor_id=mid)
            for offset in (-math.radians(4.5),0.,math.radians(4.5)):
                frame=ATParser().feed(motion_request(phase=PHASES[selected],center_rad=5.,offset_rad=offset,motor_id=selected))[0]
                self.assertEqual(struct.unpack('>4H',frame.data)[1:],(32767,1572,1966))
            for offset in (-math.radians(4.5001),math.radians(4.5001)):
                with self.assertRaises(ValueError):
                    motion_request(phase=PHASES[selected],center_rad=5.,offset_rad=offset,motor_id=selected)

    def test_actual_encoded_commands_follow_four_second_cosine_then_twenty_fixed_targets(self):
        for direction in (-1,1):
            t=SequentialEnableTransport();r=run(t,direction=direction)
            self.assertEqual(r['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',r['errors'])
            self.assertEqual(r['Kp_by_motor'],{1:3.,2:12.,3:12.})
            self.assertTrue(r['target_ramp_applied']);self.assertFalse(r['current_position_only'])
            self.assertEqual(r['bounded_pose_plan']['ramp_ns'],4_000_000_000)
            self.assertEqual(r['bounded_pose_plan']['hold_ns'],1_000_000_000)
            for mid in t.ids:
                commands=[(when,f) for when,f in t.frames if f.destination==mid and f.kind==1 and f.data[4:8]!=bytes(4)]
                self.assertEqual(len(commands),100)
                start=commands[0][0]
                for when,frame in commands:
                    elapsed=when-start
                    expected=t.centers[mid]+direction*math.radians(4)*.5*(1-math.cos(math.pi*min(elapsed,4)/4))
                    position,velocity,kp,kd=struct.unpack('>4H',frame.data)
                    decoded=position*25.14/65535-12.57
                    self.assertLessEqual(abs(decoded-expected),25.14/65535+1e-10)
                    self.assertEqual((velocity,kp,kd),(32767,393 if mid==1 else 1572,1966))
                self.assertEqual(len({frame.wire for _,frame in commands[-20:]}),1)
                self.assertGreater(len({frame.wire for _,frame in commands[:80]}),50)
                self.assertLess(commands[-1][0]-start,5.)
            self.assertTrue(r['hold_candidate_met']);self.assertTrue(r['stop_confirmed'])
            self.assertEqual(t.stop_calls[-1],(1,2,3))

    def test_matched_half_degree_margin_keeps_fresh_delta_at_four_point_five(self):
        for direction in (-1,1):
            t=SequentialEnableTransport()
            refs={i:q+direction*math.radians(.5) for i,q in t.centers.items()}
            r=run(t,refs,direction=direction)
            self.assertEqual(r['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',r['errors'])
            for i in t.ids:
                self.assertAlmostEqual(abs(r['motors'][i]['target_final_offset_rad']),math.radians(4.5))
            self.assertTrue(r['stop_confirmed'])

    def test_wrong_scope_evidence_or_more_than_four_degree_reference_target_rejects_before_io(self):
        changes=[{'matched_start_positions':None},{'profile':'legacy-rms-v1'},
                 {'profile':'position-v2-all'},{'enable_profile':'legacy-burst'},
                 {'position_response_evidence':None},{'gain_profile':'fr_step4_kp24_diagnostic'}]
        for mid in (1,2,3):
            for direction in (-1,1):
                t=SequentialEnableTransport();targets=dict(t.centers);targets[mid]+=direction*math.radians(4.0001)
                changes.append({'absolute_targets':targets})
        for change in changes:
            t=SequentialEnableTransport()
            with self.subTest(change=change),self.assertRaises(ValueError):run(t,**change)
            self.assertEqual((t.calls,t.frames,t.stop_calls),([],[],[]))
        for leg in ('FL','RR','RL'):
            t=PoseTransport(leg)
            with self.subTest(leg=leg),self.assertRaises(ValueError):run(t)
            self.assertEqual((t.calls,t.frames,t.stop_calls),([],[],[]))

    def test_torque_each_axis_and_mode_fault_or_final_one_degree_error_stop_all(self):
        for selected in (1,2,3):
            class TorqueTransport(SequentialEnableTransport):
                def value(self,mid):
                    value=super().value(mid)
                    return replace(value,torque_nm=.5001) if mid==selected and self.commanded else value
            t=TorqueTransport();r=run(t)
            self.assertEqual(r['status'],'ABORTED');self.assertTrue(r['stop_confirmed'])
            self.assertTrue(any('0.5Nm' in e for e in r['errors']))
            self.assertEqual(t.stop_calls[-1],(1,2,3))
        for failure in ('mode0','fault','hold_drift'):
            t=SequentialEnableTransport(failure if failure!='hold_drift' else None)
            if failure=='hold_drift':t.failure='hold_drift'
            r=run(t)
            self.assertEqual(r['status'],'ABORTED');self.assertTrue(r['stop_confirmed'])
            self.assertEqual(t.stop_calls[-1],(1,2,3))
            if failure=='hold_drift':self.assertTrue(any('1-degree' in e for e in r['errors']))

    def test_current_only_profile_still_rejects_moves_and_keeps_one_constant_target(self):
        t=SequentialEnableTransport()
        with self.assertRaises(ValueError):run(t,gain_profile='fr_current_kp12_diagnostic')
        self.assertEqual(t.calls,[])
        t=SequentialEnableTransport();r=hold_run(t)
        self.assertEqual(r['status'],'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED',r['errors'])
        self.assertFalse(r['target_ramp_applied'])
        for mid in t.ids:
            self.assertEqual(len({f.wire for _,f in t.frames if f.destination==mid and f.kind==1 and f.data[4:8]!=bytes(4)}),1)


if __name__=='__main__':unittest.main()
