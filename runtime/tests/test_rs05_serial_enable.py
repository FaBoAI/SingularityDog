"""Offline opt-in sequential enable tests with fault-injected replies."""
from dataclasses import replace
import inspect
import math
import unittest

from singularitydog_hw import rs05_leg_trial as trial
from singularitydog_hw.can_readonly import ATParser
from test_rs05_bounded_pose_trial import PoseTransport


class SequentialEnableTransport(PoseTransport):
    def __init__(self, failure=None):
        super().__init__('FR')
        self.enable_failure = failure
        self.exchanges, self.confirmed = [], set()
        self.enable_receipts = []

    def send(self, wire):
        frame = ATParser().feed(wire)[0]
        if frame.kind == 3 and getattr(self, 'pre_enable_guard', None) is not None:
            self.pre_enable_guard()
        if frame.kind == 1 and frame.data[4:8] != bytes(4):
            if self.confirmed != set(self.ids):
                raise AssertionError('Nonzero output preceded all three selected mode2 confirmations')
            if getattr(self, 'pre_send_guard', None) is not None:
                self.pre_send_guard()
        super().send(wire)

    def feedback_many(self, wires, expected_ids):
        wires, expected_ids = tuple(wires), tuple(expected_ids)
        frames = [ATParser().feed(wire)[0] for wire in wires]
        self.exchanges.append((tuple((f.kind, f.destination) for f in frames), expected_ids))
        if len(frames) != 1 or frames[0].kind != 3:
            found = super().feedback_many(wires, expected_ids)
            if any(f.kind == 3 for f in frames):
                self.confirmed.update(mid for mid, (value, _) in found.items() if value.mode_state == 2)
            return found
        mid = frames[0].destination
        if expected_ids != (mid,):
            raise AssertionError('Sequential enable must request exactly its own reply')
        self.send(wires[0])
        self.clock.wait(.06 if self.enable_failure == 'slow' else .007)
        if mid == 2 and self.enable_failure == 'missing':
            return {}
        value, received = self.value(mid), self.clock()
        if mid == 2:
            if self.enable_failure == 'mode0': value = replace(value, mode_state=0)
            if self.enable_failure == 'mode1': value = replace(value, mode_state=1)
            if self.enable_failure == 'fault': value = replace(value, fault_bits=1)
            if self.enable_failure == 'stale': received -= .101
            if self.enable_failure == 'mode0_then2' and self.feedback_guard is not None:
                self.feedback_guard(replace(value, mode_state=0), received, mid)
        if self.feedback_guard is not None:
            self.feedback_guard(value, received, mid)
        self.latest[mid] = value, received
        self.enable_receipts.append((mid, value.mode_state, received))
        if value.mode_state == 2 and value.fault_bits == 0:
            self.confirmed.add(mid)
        return {mid: (value, received)}


def run(transport, *, enable_profile='sequential-confirmed'):
    references = dict(transport.centers)
    return trial.run_bounded_pose_trial(transport, {mid: f'{mid:016x}' for mid in transport.ids},
        lambda: None, lambda _: None, absolute_targets={mid: ref + math.radians(4)
        for mid, ref in references.items()}, matched_start_positions=references,
        clock=transport.clock, wait=transport.clock.wait, profile='legacy-rms-v1',
        gain_profile='kp3', enable_profile=enable_profile)


class SequentialEnableTests(unittest.TestCase):
    def assert_stopped(self, transport, result):
        self.assertTrue(result['stop_confirmed'], result['errors'])
        self.assertEqual(transport.stop_calls[-1], (1, 2, 3))
        self.assertEqual([f.destination for _, f in transport.frames[-3:]], [1, 2, 3])
        self.assertTrue(all(f.kind == 4 for _, f in transport.frames[-3:]))
        self.assertFalse(transport.enabled)

    def test_three_individual_confirmations_precede_exactly_hundred_active_commands_per_axis(self):
        transport = SequentialEnableTransport()
        result = run(transport)
        self.assertEqual(result['status'], 'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['sequential_enable_confirmed_ids'], [1, 2, 3])
        enable_exchanges = [exchange for exchange in transport.exchanges if any(kind == 3 for kind, _ in exchange[0])]
        self.assertEqual(enable_exchanges, [(((3, mid),), (mid,)) for mid in (1, 2, 3)])
        self.assertEqual([row[:2] for row in transport.enable_receipts], [(1, 2), (2, 2), (3, 2)])
        first_enable = next(index for index, (_, f) in enumerate(transport.frames) if f.kind == 3)
        first_active = next(index for index, (_, f) in enumerate(transport.frames)
                            if f.kind == 1 and f.data[4:8] != bytes(4))
        self.assertFalse(any(f.kind == 1 for _, f in transport.frames[first_enable:first_active]))
        active_times = []
        for mid in transport.ids:
            times = [when for when, f in transport.frames if f.destination == mid
                     and f.kind == 1 and f.data[4:8] != bytes(4)]
            self.assertEqual(len(times), 100)
            self.assertLess(times[-1] - times[0], 5.)
            self.assertGreater(times[0], transport.enable_receipts[-1][2] - 1e-12)
            active_times.extend(times)
        stop_time = next(when for when, f in transport.frames[first_active:] if f.kind == 4)
        self.assertLessEqual(stop_time - min(active_times), 5.000000001)
        self.assertTrue(result['hold_candidate_met'])
        self.assert_stopped(transport, result)

    def test_transient_reset_mode_is_allowed_but_final_selected_reply_must_be_motor_mode(self):
        transport = SequentialEnableTransport('mode0_then2')
        result = run(transport)
        self.assertEqual(result['status'], 'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED', result['errors'])
        self.assert_stopped(transport, result)

    def test_reset_missing_fault_invalid_mode_or_stale_selected_reply_aborts_without_nonzero(self):
        for failure in ('mode0', 'missing', 'fault', 'mode1', 'stale', 'slow'):
            transport = SequentialEnableTransport(failure)
            with self.subTest(failure=failure):
                result = run(transport)
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertTrue(result['errors'])
                self.assertFalse(any(f.kind == 1 and f.data[4:8] != bytes(4) for _, f in transport.frames))
                enabled_ids = [f.destination for _, f in transport.frames if f.kind == 3]
                self.assertEqual(len(enabled_ids), len(set(enabled_ids)))
                self.assert_stopped(transport, result)

    def test_legacy_default_retains_the_single_enable_and_neutral_burst(self):
        self.assertEqual(inspect.signature(trial.run_bounded_pose_trial).parameters['enable_profile'].default,
                         'legacy-burst')
        transport = SequentialEnableTransport()
        result = run(transport, enable_profile='legacy-burst')
        self.assertEqual(result['status'], 'BOUNDED_POSE_CANDIDATE_HOLD_RESET_CONFIRMED', result['errors'])
        exchanges = [row for row in transport.exchanges if any(kind == 3 for kind, _ in row[0])]
        self.assertEqual(exchanges, [(tuple(item for mid in (1, 2, 3) for item in ((3, mid), (1, mid))), (1, 2, 3))])
        self.assert_stopped(transport, result)

    def test_unknown_or_nonstring_selection_is_rejected_before_any_transport_io(self):
        for profile in ('sequential', 'retry', '', True, None, 1):
            transport = SequentialEnableTransport()
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                run(transport, enable_profile=profile)
            self.assertEqual((transport.calls, transport.frames, transport.stop_calls), ([], [], []))

    def test_sequential_enable_requires_explicit_matched_start_before_io(self):
        transport = SequentialEnableTransport()
        with self.assertRaises(ValueError):
            trial.run_bounded_pose_trial(transport, {mid: f'{mid:016x}' for mid in transport.ids},
                lambda: None, lambda _: None, absolute_targets=dict(transport.centers),
                profile='legacy-rms-v1', enable_profile='sequential-confirmed',
                clock=transport.clock, wait=transport.clock.wait)
        self.assertEqual((transport.calls, transport.frames, transport.stop_calls), ([], [], []))


if __name__ == '__main__':
    unittest.main()
