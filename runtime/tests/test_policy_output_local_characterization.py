"""Local-mode coordinator regressions with in-memory buses, never hardware.

The explicit private token below models a profile already accepted by the
loader. Profile artifact/review validation is covered by test_policy_live_profile;
these tests exercise actual target blending, envelopes, quantization and STOP.
"""
import math
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live_profile
from singularitydog_hw import policy_output_runtime as runtime
import test_policy_output_runtime as runtime_fixtures
from test_policy_output_runtime import FakeSession, FakeIMU, profile


def local_profile(*, half_width=math.radians(3), hip_center=-1.,sign=1):
    data=profile()
    data.update(schema=live_profile.SCHEMA_V3,
        telemetry_cadence=live_profile.CADENCE_PRE_ENABLE,
        cadence_source_sha256=live_profile.cadence_source_hashes(),
        voltage_overlap=True,
        local_characterization=live_profile.LOCAL_RELATIVE_SUPPORTED,
        _local_validation_token=live_profile._LOCAL_VALIDATION_TOKEN,
        policy_weight=.01,request_gap_us=800,duration_s=1.2,
        startup_duration_s=.2,policy_ramp_s=.2,stop_duration_s=.2)
    margin=live_profile.LOCAL_NUMERICAL_MARGIN_RAD
    for mid in runtime.IDS:
        center=hip_center if mid in (1,4,7,10) else 0.
        axis=data['axes'][str(mid)]
        axis.update(sign=sign,offset_rad=center,uncertainty_rad=None,
            physical_lower_rad=center-half_width,physical_upper_rad=center+half_width,
            lower_rad=center-half_width+margin,upper_rad=center+half_width-margin,
            kp=3.,kd=.15,max_command_velocity_rad_s=math.radians(1),
            max_command_acceleration_rad_s2=math.radians(5),
            max_tracking_error_rad=math.radians(2),max_measured_velocity_rad_s=.25,
            max_measured_torque_nm=1.,max_estimated_pd_torque_nm=.1,
            max_displacement_from_start_rad=math.radians(1),max_temperature_c=45.)
    return data


def targets(data,delta=.2):
    return tuple(data['axes'][str(mid)]['offset_rad']+delta for mid in runtime.IDS)


class LocalCharacterizationRuntimeTests(unittest.TestCase):
    def run_case(self,*,profile_data,policy):
        # Blending/quantization tests use one causal clock for every fake source;
        # host scheduling is not simulated voltage age or transport latency.
        clock=runtime_fixtures.SimulatedClock()
        original_wait=runtime.wait
        def fixture_wait(futures,*,timeout,return_when):
            self.assertTrue(math.isfinite(timeout) and timeout>0)
            return original_wait(futures,timeout=2.,return_when=return_when)
        # Keep actual Future results/errors and the unchanged runtime clock gates.
        with patch.object(runtime,'wait',side_effect=fixture_wait):
            return runtime_fixtures.OutputRuntimeTests.run_case(self,
                profile_data=profile_data,policy=policy,
                front=FakeSession(1,clock=clock),rear=FakeSession(7,clock=clock),
                imu=FakeIMU(clock=clock),clock=clock,sleep=clock.sleep)

    def assert_actual_wires_within_local_limits(self,data,report,sessions):
        for session in sessions.values():
            for _,kind,mid,wire in session.calls:
                if kind!=1:
                    continue
                axis=data['axes'][str(mid)]
                raw=int.from_bytes(wire[7:9],'big')*25.14/65535-12.57
                q=axis['sign']*raw+report['fixed_offsets_rad_by_id'][mid]
                self.assertGreaterEqual(q,axis['lower_rad'])
                self.assertLessEqual(q,axis['upper_rad'])
                # The offset cancels. This check uses actual quantized encoder
                # displacement rather than an unverified absolute zero.
                self.assertLessEqual(abs(raw-report['initial_raw_rad_by_id'][mid]),
                                     axis['max_displacement_from_start_rad'])

    def test_model_valid_target_outside_local_range_can_blend_to_safe_actual_target(self):
        data=local_profile();wanted=targets(data)
        self.assertTrue(all(q>data['axes'][str(mid)]['upper_rad']
                            for mid,q in enumerate(wanted,1)))
        report,sessions=self.run_case(profile_data=data,policy=lambda *args:wanted)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(report['learned_targets_sent'])
        self.assertTrue(report['stop_confirmed'])
        self.assertIsNone(report['local_characterization']['absolute_zero_uncertainty_rad'])
        self.assertTrue(all(axis['uncertainty_rad'] is None for axis in data['axes'].values()))
        self.assert_actual_wires_within_local_limits(data,report,sessions)

    def test_raw_target_outside_actual_can_mapped_model_range_aborts_before_blending(self):
        data=local_profile();wanted=list(targets(data))
        wanted[0]=runtime.MODEL_TARGET_LIMITS_BY_ID[1][1]+1e-6
        report,sessions=self.run_case(profile_data=data,policy=lambda *args:wanted)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('ID1 learned target outside model range' in error for error in report['errors']),report['errors'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(all(session.positive_gain_writes==0 for session in sessions.values()))
        self.assertTrue(report['stop_confirmed'])

    def test_relative_displacement_guard_is_unchanged_with_negative_sign_and_unknown_zero(self):
        data=local_profile(sign=-1)
        report,sessions=self.run_case(profile_data=data,policy=lambda *args:targets(data))
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(report['learned_targets_sent'])
        self.assertIsNone(report['local_characterization']['absolute_zero_uncertainty_rad'])
        self.assert_actual_wires_within_local_limits(data,report,sessions)

    def test_blended_target_outside_narrow_reviewed_local_window_is_not_sent(self):
        data=local_profile(half_width=.0015)
        report,sessions=self.run_case(profile_data=data,policy=lambda *args:targets(data))
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('blended target outside local physical range' in error for error in report['errors']),report['errors'])
        self.assertTrue(report['stop_confirmed'])
        self.assert_actual_wires_within_local_limits(data,report,sessions)

    def test_model_valid_blend_inside_local_range_cannot_bypass_one_degree_displacement(self):
        data=local_profile(hip_center=-.15)
        wanted=list(targets(data,0.))
        for mid in (1,4,7,10):
            wanted[mid-1]=-2.19
        report,sessions=self.run_case(profile_data=data,policy=lambda *args:wanted)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('target outside joint/supported displacement envelope' in error
                            for error in report['errors']),report['errors'])
        self.assertTrue(report['stop_confirmed'])
        self.assert_actual_wires_within_local_limits(data,report,sessions)

    def test_untrusted_mode_and_boolean_or_dictionary_proof_never_open_bus_workers(self):
        for fake_proof in (None,True,{'validated':True}):
            with self.subTest(fake_proof=fake_proof):
                data=local_profile()
                data['_local_validation_token']=fake_proof
                sessions={'front':FakeSession(1),'rear':FakeSession(7)}
                with self.assertRaisesRegex(live_profile.ProfileError,'validated loader proof'):
                    runtime.run_supported_policy(data,sessions,FakeIMU(),lambda *args:targets(data),
                        cancel_io=threading.Event().set)
                self.assertTrue(all(not session.calls for session in sessions.values()))

    def test_local_profile_cannot_be_used_with_ground_supervision(self):
        sessions={'front':FakeSession(1),'rear':FakeSession(7)}
        with self.assertRaisesRegex(RuntimeError,'supported-only runner'):
            runtime.run_supported_policy(local_profile(),sessions,FakeIMU(),lambda *args:(),
                cancel_io=threading.Event().set,supervision=object())
        self.assertTrue(all(not session.calls for session in sessions.values()))


if __name__=='__main__':
    unittest.main()
