"""In-memory opt-in wiring; no device or motor transport is opened."""

import threading
import unittest
from unittest.mock import patch
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_policy_output_runtime import FakeIMU, FakeSession, SimulatedClock, encode_motion, profile
from singularitydog_hw import native_policy_batch_encode as batch
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_output_runtime as runtime


class _VerifiedModule:
    binary_sha256 = 'a' * 64

    def __init__(self, *, fail_encode=False):
        self.bound_specs = None
        self.calls = 0
        self.fail_encode = fail_encode

    def bind(self, axis_specs):
        self.bound_specs = axis_specs

        def encode(command):
            self.calls += 1
            if self.fail_encode:
                raise RuntimeError('Injected native batch rejection')
            return batch._reference_wires(command, axis_specs)

        return encode


class NativeBatchRuntimeOptInTests(unittest.TestCase):
    def make_profile(self, *, include_derived=True):
        data = profile()
        data.update(schema=live.SCHEMA_V3,
                    telemetry_cadence=live.CADENCE_PRE_ENABLE,
                    cadence_source_sha256=live.cadence_source_hashes(),
                    voltage_overlap=True, voltage_pipeline=True,
                    native_batch_encoder={'path': 'native/sdbe_native.so',
                                          'sha256': 'a' * 64})
        if include_derived:
            data['_native_batch_encoder_path'] = '/file-only/pinned/sdbe_native.so'
        return data

    def run_case(self, data, module):
        clock = SimulatedClock()
        sessions = {'front': FakeSession(1, clock=clock),
                    'rear': FakeSession(7, clock=clock)}

        with patch.object(batch, 'load_verified_module', return_value=module) as load:
            report = runtime.run_supported_policy(
                data, sessions, FakeIMU(clock=clock), lambda *_: (.04,) * 12,
                cancel_io=lambda: None, encode_motion=encode_motion,
                clock=clock, sleep=clock.sleep)
        return report, sessions, load

    def test_explicit_opt_in_uses_native_bytes_and_still_stops(self):
        module = _VerifiedModule()
        report, sessions, load = self.run_case(self.make_profile(), module)
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertGreater(module.calls, 0)
        self.assertEqual(len(module.bound_specs), 12)
        self.assertTrue(report['native_batch_encoder']['enabled'])
        self.assertEqual(report['native_batch_encoder']['binary_sha256'], 'a' * 64)
        load.assert_called_once_with('/file-only/pinned/sdbe_native.so',
                                     expected_binary_sha256='a' * 64)
        self.assertTrue(all(len(session.stop_times) == 1 for session in sessions.values()))

    def test_missing_reviewed_resolved_path_fails_before_any_bus_use(self):
        data = self.make_profile(include_derived=False)
        sessions = {'front': FakeSession(1), 'rear': FakeSession(7)}
        with self.assertRaisesRegex(RuntimeError, 'Reviewed V3 native batch'):
            runtime.run_supported_policy(
                data, sessions, FakeIMU(), lambda *_: (.04,) * 12,
                cancel_io=lambda: None, encode_motion=encode_motion)
        self.assertTrue(all(not session.calls and not session.enabled for session in sessions.values()))

    def test_native_batch_works_with_original_voltage_order(self):
        data = self.make_profile()
        data['voltage_pipeline'] = False
        module = _VerifiedModule()
        report, sessions, _ = self.run_case(data, module)
        self.assertEqual(report['status'], 'COMPLETE_SUPPORTED_OUTPUT', report['errors'])
        self.assertGreater(module.calls, 0)
        self.assertTrue(all(len(session.stop_times) == 1 for session in sessions.values()))

    def test_pin_failure_fails_before_any_bus_use(self):
        data = self.make_profile()
        sessions = {'front': FakeSession(1), 'rear': FakeSession(7)}
        with patch.object(batch, 'load_verified_module',
                          side_effect=ValueError('Native encoder binary hash differs')):
            with self.assertRaisesRegex(ValueError, 'binary hash differs'):
                runtime.run_supported_policy(
                    data, sessions, FakeIMU(), lambda *_: (.04,) * 12,
                    cancel_io=lambda: None, encode_motion=encode_motion)
        self.assertTrue(all(not session.calls and not session.enabled for session in sessions.values()))

    def test_native_rejection_faults_and_sends_stop_without_fallback(self):
        module = _VerifiedModule(fail_encode=True)
        report, sessions, _ = self.run_case(self.make_profile(), module)
        self.assertEqual(report['status'], 'ABORTED')
        self.assertGreater(module.calls, 0)
        self.assertTrue(any('Injected native batch rejection' in item
                            for item in report['errors']), report['errors'])
        self.assertFalse(report['learned_targets_sent'])
        self.assertTrue(all(len(session.stop_times) == 1 for session in sessions.values()))


if __name__ == '__main__':
    unittest.main()
