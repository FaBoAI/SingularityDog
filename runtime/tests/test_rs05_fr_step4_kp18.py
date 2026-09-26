"""The FR thigh Kp18 trial is explicit and preserves the other two gains."""
import math
import struct
import unittest

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import TrialPhase, motion_request
from test_rs05_fr_step4_kp12 import run as step_run
from test_rs05_serial_enable import SequentialEnableTransport


PROFILE = 'fr_step4_thigh_kp18_peak_burst_diagnostic'


class FRStep4ThighKp18Tests(unittest.TestCase):
    def test_codec_requires_id2_and_retains_four_point_five_degree_envelope(self):
        for mid in range(1, 13):
            if mid != 2:
                with self.assertRaises(ValueError):
                    motion_request(phase=TrialPhase.POSITION_STEP4_FR_THIGH_KP18,
                                   center_rad=5., motor_id=mid)
        for offset in (-math.radians(4.5), 0., math.radians(4.5)):
            wire = motion_request(phase=TrialPhase.POSITION_STEP4_FR_THIGH_KP18,
                                  center_rad=5., offset_rad=offset, motor_id=2)
            frame = ATParser().feed(wire)[0]
            self.assertEqual(struct.unpack('>4H', frame.data)[1:], (32767, 2359, 1966))
        with self.assertRaises(ValueError):
            motion_request(phase=TrialPhase.POSITION_STEP4_FR_THIGH_KP18,
                           center_rad=5., offset_rad=math.radians(4.5001), motor_id=2)

    def test_only_id2_gain_changes_and_all_axes_stop_after_finite_observation(self):
        transport = SequentialEnableTransport()
        result = step_run(transport, gain_profile=PROFILE, observation_profile='fr_settling_1s')
        self.assertEqual(result['status'], 'FR_SETTLING_OBSERVATION_COMPLETE_RESET_CONFIRMED',
                         result['errors'])
        self.assertEqual(result['Kp_by_motor'], {1: 3., 2: 18., 3: 12.})
        self.assertEqual(result['Kd'], .15)
        self.assertEqual(result['max_abs_torque_feedback_candidate_nm_by_motor'],
                         {1: .5, 2: 1.5, 3: .5})
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['physical_torque_cap_verified'])
        self.assertEqual(len(transport.motion_batches), 100)
        self.assertEqual(transport.stop_calls[-1], (1, 2, 3))
        self.assertFalse(transport.enabled)
        for mid, encoded_kp in ((1, 393), (2, 2359), (3, 1572)):
            frames = [f for _, f in transport.frames if f.destination == mid and f.kind == 1
                      and f.data[4:8] != bytes(4)]
            self.assertEqual(len(frames), 100)
            self.assertTrue(all(struct.unpack('>4H', f.data)[2] == encoded_kp for f in frames))

    def test_other_profile_cannot_pick_the_new_thigh_phase(self):
        transport = SequentialEnableTransport()
        result = step_run(transport, gain_profile='fr_step4_kp12_peak_burst_diagnostic',
                          observation_profile='fr_settling_1s')
        self.assertEqual(result['Kp_by_motor'], {1: 3., 2: 12., 3: 12.})
        self.assertTrue(all(struct.unpack('>4H', f.data)[2] == 1572 for _, f in transport.frames
                            if f.destination == 2 and f.kind == 1 and f.data[4:8] != bytes(4)))


if __name__ == '__main__':
    unittest.main()
