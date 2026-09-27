import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from singularitydog_hw.motor_epoch_readonly_capture import (
    _check_quiet, _expected_uids,
)


class MotorEpochReadOnlyCaptureTest(unittest.TestCase):
    def test_expected_uid_file_requires_unique_twelve_id_mapping(self):
        values = {str(mid): f"{mid:016x}" for mid in range(1, 13)}
        with TemporaryDirectory() as directory:
            path = Path(directory) / "uids.json"
            path.write_text(json.dumps(values))
            actual, digest = _expected_uids(path)
            self.assertEqual(actual[3], values["3"])
            self.assertEqual(len(digest), 64)
            path.write_text('{"1":"0000000000000001","1":"0000000000000001"}')
            with self.assertRaisesRegex(ValueError, "Duplicate key"):
                _expected_uids(path)

    def test_quiet_check_rejects_motion_and_current(self):
        rows = {str(mid): {"run_mode": 0, "current": 0.0,
                           "position_span_deg": 0.01} for mid in range(1, 13)}
        _check_quiet({"rows": rows})
        rows["7"]["current"] = 0.2
        with self.assertRaisesRegex(RuntimeError, "ID7"):
            _check_quiet({"rows": rows})
        rows["7"]["current"] = 0.0
        rows["7"]["position_span_deg"] = 0.11
        with self.assertRaisesRegex(RuntimeError, "ID7"):
            _check_quiet({"rows": rows})


if __name__ == "__main__":
    unittest.main()
