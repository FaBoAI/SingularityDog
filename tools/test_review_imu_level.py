import math
import unittest
from review_imu_level import direction_error_deg


class LevelDirectionTests(unittest.TestCase):
    R = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    def test_uniform_scale_preserves_direction_but_tilt_does_not(self):
        angle = math.radians(4)
        a = [0, math.sin(angle)*9.80665, -math.cos(angle)*9.80665]
        self.assertAlmostEqual(direction_error_deg(a, self.R), 4)
        self.assertAlmostEqual(direction_error_deg([v*1.09 for v in a], self.R), 4)
        self.assertGreater(direction_error_deg([1, 0, -10.68], self.R), 5)

    def test_inverted_mount_is_not_silently_accepted(self):
        self.assertEqual(direction_error_deg([0, 0, 10.68], self.R), 180)

    def test_invalid_or_directionless_data_rejected(self):
        for a in ([0, 0, 0], [0, float('nan'), 1], [0, float('inf'), 1], [0, True, 1], [0, 1]):
            with self.assertRaises(ValueError):
                direction_error_deg(a, self.R)


if __name__ == '__main__':
    unittest.main()
