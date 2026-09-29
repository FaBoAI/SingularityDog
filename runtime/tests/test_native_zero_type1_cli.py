"""Missing explicit native diagnostic inputs fail before file or device access."""

from contextlib import redirect_stderr
import io
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_zero_type1_stress as stress


class NativeZeroType1CliTests(unittest.TestCase):
    physical_flags = ("support-in-place", "cutoff-ready", "rs05-model-confirmed")
    inputs = {"condition": "800us-window3", "front-port": "/unused/front",
              "rear-port": "/unused/rear", "library": "/unused/library.so",
              "expected-uids": "/unused/uids.json", "boot-id": "unused-boot",
              "power-epoch": "unused-power", "output": "/unused/output",
              "audio": "/unused/audio.wav", "audio-sha256": "unused-hash",
              "audio-device": "unused-device"}

    def execution_args(self, missing=None):
        args = ["--execute-supported-zero-gain"]
        args.extend("--"+name for name in self.physical_flags if name != missing)
        for name, value in self.inputs.items():
            if name != missing:
                args.extend(("--"+name, value))
        return args

    def assert_rejected_before_open(self, args, message):
        serial = SimpleNamespace(Serial=Mock(side_effect=AssertionError("Serial opened")))
        error_output = io.StringIO()
        with patch.dict(sys.modules, {"serial": serial}), \
             patch.object(stress.native, "load_library") as load, \
             patch.object(stress.Path, "read_bytes") as read, \
             redirect_stderr(error_output):
            with self.assertRaises(SystemExit) as raised:
                stress.main(args)
        self.assertEqual(raised.exception.code, 2)
        self.assertIn(message, error_output.getvalue())
        load.assert_not_called()
        read.assert_not_called()
        serial.Serial.assert_not_called()

    def test_each_explicit_execution_input_is_required_before_open(self):
        for name in self.inputs:
            with self.subTest(missing=name):
                self.assert_rejected_before_open(self.execution_args(name),
                    "One explicit condition, native library, ports, epoch, output and audio inputs required")

if __name__ == "__main__":
    unittest.main()
