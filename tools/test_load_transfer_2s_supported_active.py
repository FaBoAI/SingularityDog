"""Offline wire-gate and refusal tests; never opens a physical port."""
import json
import math
from pathlib import Path
import tempfile
import unittest

from load_transfer_2s_active_wrapper import ExactWirePort
from build_load_transfer_2s_supported_active import build
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw import rs05_trial_protocol as P


class FakeRaw:
    def __init__(self):
        self.writes = []
        self.port = '/dev/fake'
        self.in_waiting = 0

    def write(self, wire):
        self.writes.append(wire)
        return len(wire)

    def read(self, count):
        return b''


class SupportedActiveTests(unittest.TestCase):
    def gate(self, ids=(1, 4)):
        raw = FakeRaw()
        review = {'supported_floor_start_raw_rad_by_id': {str(mid): 0. for mid in ids}}
        review.update(wrap_equivalence_motor_ids=[3, 9], physical_full_turn_excluded=True)
        return raw, ExactWirePort(raw, ids, review, parser_type=ATParser,
                                  read_request=read_request, protocol=P)

    def test_exact_wire_gate_rejects_nonreviewed_gain_and_moving_target(self):
        raw, gate = self.gate()
        gate.write(read_request(1))
        gate.write(P.watchdog_setup_request(phase=P.TrialPhase.WATCHDOG_SETUP, motor_id=1))
        gate.write(P.motion_request(phase=P.TrialPhase.ZERO_GAIN, center_rad=0., motor_id=1))
        gate.set_centers({1: 0., 4: 0.})
        gate.write(P.enable_request(phase=P.TrialPhase.ENABLE, motor_id=1))
        gate.write(P.motion_request(phase=P.TrialPhase.POSITION_STEP5, center_rad=0., motor_id=1))
        gate.write(P.motion_request(phase=P.TrialPhase.POSITION_STEP5_KP4, center_rad=0., motor_id=4))
        prior = len(raw.writes)
        for wire in (
            P.motion_request(phase=P.TrialPhase.POSITION_STEP5_KP4, center_rad=0., motor_id=1),
            P.motion_request(phase=P.TrialPhase.POSITION_STEP5, center_rad=.02, motor_id=1),
            P.motion_request(phase=P.TrialPhase.POSITION_STEP5_RR_HIP_KP6,
                             center_rad=0., motor_id=9),
            P.enable_request(phase=P.TrialPhase.ENABLE, motor_id=7),
        ):
            with self.assertRaises(RuntimeError):
                gate.write(wire)
        self.assertEqual(len(raw.writes), prior)
        gate.write(P.stop_request(phase=P.TrialPhase.STOP, motor_id=1))
        self.assertEqual(len(raw.writes), prior + 1)
        raw2, unseeded = self.gate()
        unseeded.write(P.enable_request(phase=P.TrialPhase.ENABLE, motor_id=1))
        with self.assertRaises(RuntimeError):
            unseeded.write(P.motion_request(phase=P.TrialPhase.POSITION_STEP5,
                                            center_rad=0., motor_id=1))
        self.assertEqual(len(raw2.writes), 1)

    def test_builder_refuses_missing_r8_success(self):
        source = Path('/private/tmp/fabo-stance-20260927-private/load-transfer-2s-preflight-r8')
        failed = Path('/private/tmp/fabo-stance-20260927-private/load-transfer-2s-preflight-20260927-r6')
        prior_package = Path('/private/tmp/fabo-stance-20260927-private/load-transfer-2s-supported-active-r1')
        prior_log = Path('/private/tmp/fabo-stance-20260927-private/load-transfer-2s-supported-active-20260927-r1')
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / 'active'
            with self.assertRaises(ValueError):
                build(source, failed / 'summary.json', failed / 'events.jsonl',
                      prior_package, prior_log / 'summary.json', prior_log / 'events.jsonl', out)
            self.assertFalse(out.exists())

    def test_exact_wire_gate_allows_reviewed_encoder_branch_only(self):
        raw, gate = self.gate((3, 9))
        gate.write(P.motion_request(phase=P.TrialPhase.ZERO_GAIN,
                                    center_rad=-2 * math.pi, motor_id=3))
        gate.write(P.motion_request(phase=P.TrialPhase.ZERO_GAIN,
                                    center_rad=2 * math.pi + .004, motor_id=9))
        gate.set_centers({3: -2 * math.pi, 9: 2 * math.pi + .004})
        gate.write(P.enable_request(phase=P.TrialPhase.ENABLE, motor_id=3))
        gate.write(P.motion_request(phase=P.TrialPhase.POSITION_STEP5,
                                    center_rad=-2 * math.pi, motor_id=3))
        self.assertEqual(len(raw.writes), 4)
        _, bad = self.gate((3, 9))
        with self.assertRaises(RuntimeError):
            bad.set_centers({3: 2 * math.pi + math.radians(10), 9: 0.})
        _, other = self.gate((1, 4))
        with self.assertRaises(RuntimeError):
            other.set_centers({1: 2 * math.pi, 4: 0.})


if __name__ == '__main__':
    unittest.main()
