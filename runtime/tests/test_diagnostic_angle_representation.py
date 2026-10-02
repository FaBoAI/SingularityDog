"""File-only periodic-angle audit; synthetic buses never open a device.

This is a candidate regression suite, not an output permission or calibration
certificate.  Diagnostic startup normalization and active continuity remain
separate: an unexpected full turn during a run must still stop the run.
"""
import copy
import importlib.util
import math
from pathlib import Path
import statistics
import struct
from types import SimpleNamespace
import unittest


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    import test_diagnostic_angle_branch as fixtures
except ImportError:
    fixtures = load('branch_fixtures', Path(__file__).with_name(
        'test_diagnostic_capture_branch_derivation_20261002.py'))
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import policy_output_runtime as runtime
import test_policy_output_runtime as simulated

TAU = 2 * math.pi


def set_samples(capture, mid, values):
    row = capture['telemetry']['rows'][str(mid)]
    for sample, value in zip(row['position_samples'], values):
        sample['rad'] = value
    row['median_position_rad'] = statistics.median(values)
    row['position_span_deg'] = math.degrees(max(values) - min(values))


class AngleRepresentationTests(unittest.TestCase):
    def test_unsigned_and_signed_startup_representations_match_all_axes_and_signs(self):
        for sign in (-1, 1):
            base, signed, uids = fixtures.fixtures()
            unsigned = copy.deepcopy(signed)
            wanted = {}
            for mid in range(1, 13):
                index = fixtures.tool.shadow.CAN_ORDER.index(mid)
                lower, upper = fixtures.tool.shadow.LOWER[index], fixtures.tool.shadow.UPPER[index]
                # Avoid a representation boundary inside the three-sample
                # window; the explicit boundary rejection is tested below.
                q = (lower + upper) / 2
                if abs(q) < .01:
                    q = .04
                base['candidates'][mid - 1]['sign_candidate'] = sign
                raw = sign * q
                signed_raw = (raw + math.pi) % TAU - math.pi
                unsigned_raw = raw % TAU
                set_samples(signed, mid, [signed_raw + x for x in (-1e-5, 0., 1e-5)])
                set_samples(unsigned, mid, [unsigned_raw + x for x in (-1e-5, 0., 1e-5)])
                wanted[str(mid)] = q
            before = copy.deepcopy((base, signed, unsigned, uids))
            a = fixtures.derive(base, signed, uids)
            b = fixtures.derive(base, unsigned, uids)
            for mid in range(1, 13):
                key = str(mid)
                with self.subTest(sign=sign, mid=mid):
                    self.assertAlmostEqual(a['model_rad_at_source_capture_by_id'][key], wanted[key], places=12)
                    self.assertAlmostEqual(b['model_rad_at_source_capture_by_id'][key], wanted[key], places=12)
                    for out, capture in ((a, signed), (b, unsigned)):
                        row = out['candidates'][mid - 1]
                        # The unchanged formula and its inverse use the same
                        # selected branch.  Read-only modulo alone is not enough.
                        reconstructed = (wanted[key] - row['offset_candidate_rad']) / sign
                        self.assertAlmostEqual(reconstructed,
                            capture['telemetry']['rows'][key]['median_position_rad'], places=12)
                        self.assertEqual(out['source_raw_rad_by_id'][key],
                            capture['telemetry']['rows'][key]['median_position_rad'])
                        for flag in ('output_allowed', 'motor_output_available',
                                     'motor_targets_generated', 'calibration_verified',
                                     'approved_for_runtime', 'live_50hz_verified'):
                            self.assertIs(out[flag], False)
            self.assertEqual((base, signed, unsigned, uids), before)

    def test_three_quiet_samples_crossing_zero_boundary_are_not_silently_wrapped(self):
        base, capture, uids = fixtures.fixtures()
        set_samples(capture, 3, [TAU - 1e-5, 1e-5, TAU - 2e-5])
        before = copy.deepcopy(capture)
        with self.assertRaisesRegex(ValueError, 'ID3 position median/span inconsistent or not static'):
            fixtures.derive(base, capture, uids)
        self.assertEqual(capture, before)

    def test_three_quiet_samples_crossing_signed_boundary_are_not_silently_wrapped(self):
        base, capture, uids = fixtures.fixtures()
        # A legitimate model position can have a raw encoder representation
        # near pi because the original mechanical zero need not be zero.
        index = fixtures.tool.shadow.CAN_ORDER.index(3)
        q = (fixtures.tool.shadow.LOWER[index] + fixtures.tool.shadow.UPPER[index]) / 2
        base['candidates'][2]['offset_candidate_rad'] = q - math.pi
        set_samples(capture, 3, [math.pi - 1e-5, -math.pi + 1e-5, math.pi - 2e-5])
        with self.assertRaisesRegex(ValueError, 'ID3 position median/span inconsistent or not static'):
            fixtures.derive(base, capture, uids)

    def test_forged_circular_span_does_not_bypass_raw_linear_stability(self):
        base, capture, uids = fixtures.fixtures()
        set_samples(capture, 3, [TAU - 1e-5, 1e-5, TAU - 2e-5])
        capture['telemetry']['rows']['3']['position_span_deg'] = math.degrees(3e-5)
        with self.assertRaisesRegex(ValueError, 'ID3 position median/span inconsistent or not static'):
            fixtures.derive(base, capture, uids)

    def test_type17_type2_one_turn_mismatch_aborts_before_enable_or_targets(self):
        for delta in (-TAU, TAU):
            with self.subTest(delta=delta):
                clock = simulated.SimulatedClock()

                class DifferentParameterBranch(simulated.FakeSession):
                    def _exchange(self, wires, timeout_ns, send_only):
                        result = super()._exchange(wires, timeout_ns, send_only)
                        for record in result[0]:
                            tx = codec.ATParser().feed(bytes(record.tx))[0]
                            if (tx.kind == 17 and tx.destination == 3
                                    and int.from_bytes(tx.data[:2], 'little') == 0x7019):
                                record.rx[11:15] = struct.pack('<f', self.positions[3] + delta)
                        return result

                front = DifferentParameterBranch(1, clock=clock)
                rear = simulated.FakeSession(7, clock=clock)

                class Pair:
                    def exchange(self, wires, **kwargs):
                        kwargs.pop('label', None)
                        return {scope: session.exchange(wires[scope], **kwargs)
                                for scope, session in (('front', front), ('rear', rear))}

                with self.assertRaisesRegex(RuntimeError, 'ID3 Type17/Type2 branch or scale mismatch'):
                    runtime.preflight(Pair(), simulated.profile())
                self.assertFalse(front.enabled or rear.enabled)
                self.assertFalse(any(kind in (1, 3) for session in (front, rear)
                                     for _, kind, _, _ in session.calls))

    def test_active_full_turn_jump_rejected_for_every_axis_and_both_signs(self):
        for sign in (-1, 1):
            for mid in runtime.IDS:
                for jump in (-TAU, TAU):
                    with self.subTest(sign=sign, mid=mid, jump=jump):
                        profile = simulated.profile()
                        profile['axes'][str(mid)]['sign'] = sign
                        old = {i: SimpleNamespace(mode_state=2, fault_bits=0,
                            protocol_position_rad=.1, velocity_rad_s=0., torque_nm=0.,
                            temperature_c=25.) for i in runtime.IDS}
                        new = {i: SimpleNamespace(**vars(v)) for i, v in old.items()}
                        new[mid].protocol_position_rad += jump
                        previous = {(i, 'feedback'): (old[i], 1_000_000_000, 1_000_000_100)
                                    for i in runtime.IDS}
                        current = {(i, 'feedback'): (new[i], 1_020_000_000, 1_020_000_100)
                                   for i in runtime.IDS}
                        with self.assertRaisesRegex(RuntimeError, f'ID{mid} raw position discontinuity'):
                            runtime.feedback_sample(current, profile, {i: 0. for i in runtime.IDS},
                                now_ns=1_020_000_200, previous=previous)

    def test_active_full_turn_jump_causes_both_bus_stops_in_real_coordinator(self):
        case = simulated.OutputRuntimeTests()
        for sign in (-1, 1):
            for jump in (-TAU, TAU):
                clock = simulated.SimulatedClock()
                profile = simulated.profile()
                profile['axes']['3']['sign'] = sign

                class JumpSession(simulated.FakeSession):
                    injected = False
                    def _exchange(self, wires, timeout_ns, send_only):
                        result = super()._exchange(wires, timeout_ns, send_only)
                        for record in result[0]:
                            tx = codec.ATParser().feed(bytes(record.tx))[0]
                            if (not self.injected and tx.kind == 1 and tx.destination == 3
                                    and int.from_bytes(tx.data[4:6], 'big') > 0):
                                raw = int.from_bytes(record.rx[7:9], 'big') * 25.14 / 65535 - 12.57
                                record.rx[7:9] = simulated.quantize(raw + jump, -12.57, 12.57).to_bytes(2, 'big')
                                self.injected = True
                        return result

                front = JumpSession(1, clock=clock)
                report, sessions = case.run_case(front=front,
                    rear=simulated.FakeSession(7, clock=clock), profile_data=profile,
                    imu=simulated.FakeIMU(clock=clock), clock=clock, sleep=clock.sleep)
                with self.subTest(sign=sign, jump=jump):
                    self.assertTrue(front.injected)
                    self.assertEqual(report['status'], 'ABORTED')
                    self.assertIn('ID3 raw position discontinuity', str(report['errors']))
                    self.assertTrue(report['stop_confirmed'])
                    self.assertTrue(all(not s.enabled and len(s.stop_times) == 1 for s in sessions.values()))


if __name__ == '__main__':
    unittest.main()
