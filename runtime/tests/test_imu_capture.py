import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from singularitydog_hw.imu_capture import main,capture_summary


class CaptureTests(unittest.TestCase):
    def test_dry_run_does_not_open_hardware_or_create_output(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"not-created"
            with mock.patch("singularitydog_hw.imu_capture.ICM20948") as device, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--output",str(path)]),0)
            device.assert_not_called()
            self.assertFalse(path.exists())

    def test_summary_does_not_turn_static_reading_into_calibration(self):
        samples=[{"monotonic_ns":i*10_000_000,"accel_m_s2":[0,0,-10.69],
                  "gyro_rad_s":[0,.01,0],"temperature_c":26+i*.01} for i in range(100)]
        result=capture_summary(samples)
        self.assertFalse(result["calibration_applied"])
        self.assertFalse(result["stillness_or_orientation_confirmed"])
        self.assertAlmostEqual(result["rate_hz"],100)
        self.assertGreater(result["gravity_norm_deviation_percent"],8.9)


if __name__=="__main__":
    unittest.main()
