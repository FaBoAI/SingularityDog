"""Feedback validation boundaries and exact arithmetic without device access."""
import copy
from dataclasses import replace
import math
import struct
from types import SimpleNamespace
import unittest

from singularitydog_hw import policy_output_runtime as runtime


NOW = 1_000_000_000


class FeedbackHotPathTests(unittest.TestCase):
    def setUp(self):
        self.profile = {'max_sample_age_ms': 20., 'axes': {}}
        self.rows, self.previous, self.offsets = {}, {}, {}
        for mid in runtime.IDS:
            self.profile['axes'][str(mid)] = {
                'sign': -1 if mid % 2 else 1, 'lower_rad': -2., 'upper_rad': 2.,
                'max_measured_velocity_rad_s': .35, 'max_measured_torque_nm': 1.,
                'max_temperature_c': 50., 'max_displacement_from_start_rad': math.radians(1)}
            frame = SimpleNamespace(protocol_position_rad=(mid-6)/20.,
                velocity_rad_s=(mid-6)/100., torque_nm=(mid-6)/50.,
                temperature_c=30.+mid, mode_state=2, fault_bits=0)
            start = NOW-5_000_000-mid*1000
            end = NOW-2_000_000+mid*1000
            self.rows[mid, 'feedback'] = frame, start, end
            self.previous[mid, 'feedback'] = copy.copy(frame), start-5_000_000, end-5_000_000
            self.offsets[mid] = mid/1000.

    def sample(self, **kwargs):
        return runtime.feedback_sample(self.rows, self.profile, self.offsets,
            now_ns=NOW, previous=self.previous, **kwargs)

    def test_exact_values_timestamps_and_inputs_remain_unchanged(self):
        saved = copy.deepcopy((self.rows, self.previous, self.profile, self.offsets))
        got = self.sample()
        expected = tuple(self.profile['axes'][str(i)]['sign'] *
            self.rows[i, 'feedback'][0].protocol_position_rad+self.offsets[i]
            for i in runtime.IDS)
        self.assertEqual([struct.pack('>d', x) for x in got.q_model_rad],
                         [struct.pack('>d', x) for x in expected])
        self.assertEqual(got.monotonic_s,
                         min(row[1] for row in self.rows.values())/1e9)
        self.assertEqual((self.rows, self.previous, self.profile, self.offsets), saved)
        runtime.validate_measured(got, self.profile, initial=got)

    def test_all_axes_missing_mode_fault_repeated_and_discontinuity(self):
        for mid in runtime.IDS:
            original = self.rows[mid, 'feedback']
            for reason in ('missing feedback', 'fault/mode', 'repeated feedback',
                           'raw position discontinuity'):
                with self.subTest(mid=mid, reason=reason):
                    frame, start, end = original
                    frame = copy.copy(frame)
                    if reason == 'missing feedback':
                        del self.rows[mid, 'feedback']
                    elif reason == 'fault/mode':
                        frame.fault_bits = 1
                        self.rows[mid, 'feedback'] = frame, start, end
                    elif reason == 'repeated feedback':
                        old_end = self.previous[mid, 'feedback'][2]
                        self.rows[mid, 'feedback'] = frame, old_end-1, old_end
                    else:
                        frame.protocol_position_rad += .1
                        self.rows[mid, 'feedback'] = frame, start, end
                    with self.assertRaisesRegex(RuntimeError, f'^ID{mid} {reason}$'):
                        self.sample()
                    self.rows[mid, 'feedback'] = original

    def test_age_boundary_and_observed_post_reply_overrun_stay_rejected(self):
        for mid in runtime.IDS:
            original = self.rows[mid, 'feedback']
            frame, _, end = original
            self.rows[mid, 'feedback'] = frame, NOW-20_000_000, end
            self.sample()
            for age in (20_000_001, 20_128_909):
                self.rows[mid, 'feedback'] = frame, NOW-age, end
                with self.assertRaisesRegex(RuntimeError, f'^ID{mid} stale feedback$'):
                    self.sample()
            self.rows[mid, 'feedback'] = original

    def test_noncausal_or_wrong_required_mode_is_not_accepted(self):
        frame, start, end = self.rows[1, 'feedback']
        for new_start, new_end in ((0, end), (end+1, end), (start, NOW+1)):
            self.rows[1, 'feedback'] = frame, new_start, new_end
            with self.assertRaisesRegex(RuntimeError, 'stale feedback'):
                self.sample()
        self.rows[1, 'feedback'] = frame, start, end
        with self.assertRaisesRegex(RuntimeError, 'fault/mode'):
            self.sample(required_mode=0)

    def test_every_measured_limit_and_nan_keeps_its_error(self):
        sample = self.sample()
        cases = [('q_model_rad', 3., 'measured joint limit'),
                 ('torque_nm', 1.000001, 'measured torque'),
                 ('velocity_rad_s', .350001, 'measured velocity'),
                 ('temperature_c', 50.000001, 'measured temperature')]
        for mid in runtime.IDS:
            for field, value, reason in cases:
                for bad in (value, float('nan')):
                    with self.subTest(mid=mid, field=field, bad=bad):
                        row = list(getattr(sample, field));row[mid-1] = bad
                        candidate = SimpleNamespace(**vars(sample))
                        setattr(candidate, field, tuple(row))
                        with self.assertRaisesRegex(RuntimeError, f'^ID{mid} {reason}$'):
                            runtime.validate_measured(candidate, self.profile)
            row = list(sample.q_model_rad)
            row[mid-1] += math.radians(1)+1e-8
            with self.assertRaisesRegex(RuntimeError, f'^ID{mid} measured trial displacement$'):
                runtime.validate_measured(replace(sample, q_model_rad=tuple(row)), self.profile, initial=sample)


if __name__ == '__main__':
    unittest.main()
