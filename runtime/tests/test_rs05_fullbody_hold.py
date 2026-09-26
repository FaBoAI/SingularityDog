"""Two-worker full-body hold tests with virtual time and no serial hardware."""
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import asdict, replace
import math
from pathlib import Path
import struct
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.rs05_trial_protocol import (
    TrialPhase, Type2Feedback, motion_request, stop_request)
from singularitydog_hw import rs05_fullbody_hold as fullbody
from test_current_hold_review import BOOT_ID, review_data
from test_rs05_leg_settled import CENTERS, samples


BUS_IDS = {'front': (1, 2, 3, 4, 5, 6), 'rear': (7, 8, 9, 10, 11, 12)}
UIDS = {mid: f'{mid:016x}' for mid in range(1, 13)}
REAL_BARRIER = threading.Barrier


def id3_noisy_velocity_replay():
    """21 readings with today's observed +.03375 mean and .0634 RMS."""
    mean, rms = .03375, .0634
    deviation = math.sqrt((rms*rms - mean*mean) * 21 / 20)
    return [mean + deviation if n % 2 == 0 else mean - deviation
            for n in range(20)] + [mean]


def fullbody_review(*, active=False):
    return {'schema': 'rs05-fullbody-current-hold-review-v1',
            'scope': 'supported-fullbody-current-reference-hold-5s',
            'motor_ids': list(range(1, 13)),
            'motor_uids': {str(mid): uid for mid, uid in UIDS.items()},
            'reviewed_motor_ids': [5, 6, 8], 'boot_id': BOOT_ID,
            'firmware': '0.5.0.13', 'sha256': 'a' * 64,
            'review_complete': True, 'source_files_verified': True,
            'calibration_verified': False, 'learned_policy_allowed': False,
            'standing_allowed': False, 'l_target_replay_allowed': False,
            'supported_hold_authorized': active,
            'motor_reviews': {str(mid): review_data(motor_id=mid) for mid in (5, 6, 8)}}


class WorkerClock:
    """Each worker advances independently; real barriers synchronize epochs."""
    def __init__(self):
        self.main_thread = threading.get_ident()
        self.times = {}
        self.lock = threading.Lock()

    def __call__(self):
        ident = threading.get_ident()
        with self.lock:
            if ident == self.main_thread:
                return max(self.times.values(), default=10.)
            return self.times.setdefault(ident, 10.)

    def wait(self, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            raise AssertionError('Virtual wait must be finite and nonnegative')
        ident = threading.get_ident()
        with self.lock:
            self.times[ident] = self.times.get(ident, 10.) + seconds

    def synchronize(self):
        with self.lock:
            now = max(self.times.values(), default=10.)
            for ident in self.times:
                self.times[ident] = now

    def barrier(self, parties, action=None, timeout=None):
        def synchronize_then_act():
            self.synchronize()
            if action is not None:
                action()
        return REAL_BARRIER(parties, action=synchronize_then_act, timeout=timeout)


class FakeBus:
    """Records every transport operation and invokes the real coordinator guards."""
    def __init__(self, name, clock, *, failure=None, noise_ids=(), ledger=None):
        self.bus_name, self.ids = name, BUS_IDS[name]
        self.clock, self.failure, self.noise_ids = clock, failure, noise_ids
        self.serial = object()
        self.calls, self.frames, self.stop_calls = [], [], []
        self.ledger = [] if ledger is None else ledger
        self.owner_threads = set()
        self.centers = {mid: .5 + .1 * mid for mid in self.ids}
        self.watchdogs = {mid: 0 for mid in self.ids}
        self.latest, self.enabled, self.active_mode, self.commanded = {}, set(), set(), {}
        self.feedback_guard = self.pre_send_guard = self.pre_enable_guard = None
        self.active_deadline = None
        self.fault_latched = None
        self.batch_count, self.active_batches, self.disabled_batches = 0, 0, 0
        self.failed_once = False
        self.guard_observations = None
        self.guard_checks = []

    def own(self, operation, *args):
        ident = threading.get_ident()
        self.owner_threads.add(threading.current_thread())
        self.calls.append((operation, *args, ident, self.clock()))
        self.ledger.append({'operation': operation, 'bus': self.bus_name, 'args': args})
        if ident == self.clock.main_thread:
            raise AssertionError('Main thread touched a transport')
        if len(self.owner_threads) != 1:
            raise AssertionError('A bus acquired more than one worker owner')

    def parameter(self, mid, name=None):
        self.own('parameter', mid, name)
        if mid not in self.ids:
            raise AssertionError('Cross-bus parameter request')
        if self.failure == 'uid' and mid == self.ids[-1] and name is None:
            return {'mcu_uid_hex': 'f' * 16}
        if name is None:
            return {'mcu_uid_hex': UIDS[mid], 'ok': True}
        values = {'run_mode': 0, 'position': self.centers[mid], 'current': 0.,
                  'velocity': 0., 'voltage': 40., 'can_timeout': self.watchdogs[mid]}
        value = values[name]
        if mid == self.ids[-1]:
            failures = {'run_mode': ('run_mode', 2), 'current': ('current', .051),
                        'voltage': ('voltage', 34.9), 'velocity': ('velocity', .501),
                        'nonfinite': ('position', float('nan')),
                        'position_mismatch': ('position', self.centers[mid] + .021)}
            if self.failure in failures and name == failures[self.failure][0]:
                value = failures[self.failure][1]
            if self.failure == 'watchdog_readback' and name == 'can_timeout' and value == 4000:
                value = 0
        return {'value': value, 'ok': True}

    def fresh_boundary(self):
        self.own('fresh_boundary')
        if self.fault_latched:
            raise RuntimeError(self.fault_latched)

    def guarded(self, phase, callback):
        self.guard_observations = []
        try:
            callback()
        finally:
            self.guard_checks.append((phase, tuple(self.guard_observations)))
            self.guard_observations = None

    def send(self, command):
        frame = ATParser().feed(command)[0]
        mid = frame.destination
        self.own('send', frame.kind, mid)
        if mid not in self.ids:
            raise AssertionError('Cross-bus command')
        if frame.kind != 4:
            if self.fault_latched:
                raise RuntimeError(self.fault_latched)
            if (frame.kind in (1, 3) and self.active_deadline is not None
                    and self.clock() >= self.active_deadline):
                raise RuntimeError('Active cycle deadline')
            if frame.kind == 3 and self.pre_enable_guard is not None:
                self.guarded('enable', self.pre_enable_guard)
            if frame.kind == 1 and frame.data[4:8] != bytes(4) and self.pre_send_guard is not None:
                self.guarded('motion', self.pre_send_guard)
        if (self.failure == 'write' and frame.kind == 1 and frame.data[4:8] != bytes(4)
                and not self.failed_once):
            self.failed_once = True
            self.fault_latched = 'Injected active write failure'
            raise IOError(self.fault_latched)
        self.frames.append((self.clock(), frame, threading.get_ident()))
        if frame.kind == 18:
            self.watchdogs[mid] = struct.unpack_from('<I', frame.data, 4)[0]
        if frame.kind == 3:
            self.enabled.add(mid)
            # The physical RS05 replies with mode2 immediately after Enable.
            self.active_mode.add(mid)
        if frame.kind == 4:
            self.enabled.discard(mid)
        if frame.kind == 1 and frame.data[4:8] != bytes(4):
            self.active_mode.add(mid)
            self.commanded[mid] = struct.unpack_from('>H', frame.data)[0] * 25.14 / 65535 - 12.57
        if frame.kind == 4:
            self.active_mode.discard(mid)

    def value(self, mid):
        return Type2Feedback(2 if mid in self.active_mode else 0, 0, 32767,
                             self.commanded.get(mid, self.centers[mid]), 0., 0., 30.)

    def feedback_many(self, commands, expected_ids):
        commands = tuple(commands)
        expected_ids = tuple(expected_ids)
        self.own('feedback_many', tuple(expected_ids))
        if not expected_ids or len(set(expected_ids)) != len(expected_ids) or not set(expected_ids) <= set(self.ids):
            raise AssertionError('Feedback must use a unique subset of its bus')
        active_cycle = any(ATParser().feed(command)[0].kind == 1
                           and ATParser().feed(command)[0].data[4:8] != bytes(4)
                           for command in commands)
        for command in commands:
            self.send(command)
        self.batch_count += 1
        active = bool(self.enabled)
        if active_cycle:
            self.active_batches += 1
        if not active:
            self.disabled_batches += 1
        found = {mid: (self.value(mid), self.clock()) for mid in expected_ids}
        if active_cycle and self.failure == 'final_error' and 10 in found:
            value, when = found[10]
            found[10] = replace(value, protocol_position_rad=self.centers[10] + math.radians(1.23)), when
        if active_cycle and self.failure in ('id4_stable_bias', 'id4_large_bias') and 4 in found:
            value, when = found[4]
            bias_deg = 1.14 if self.failure == 'id4_stable_bias' else 1.6
            found[4] = replace(value, protocol_position_rad=self.centers[4] + math.radians(bias_deg)), when
        if active_cycle and self.failure in ('id9_stable_bias', 'id9_large_bias') and 9 in found:
            value, when = found[9]
            bias_deg = -1.04 if self.failure == 'id9_stable_bias' else -1.6
            found[9] = replace(value, protocol_position_rad=self.centers[9] + math.radians(bias_deg)), when
        if not active:
            for mid in self.noise_ids:
                if mid in found:
                    value, when = found[mid]
                    found[mid] = (replace(value, velocity_rad_s=.06 if self.disabled_batches % 2 else -.06), when)
        if active_cycle and self.active_batches >= 3 and self.ids[-1] in found:
            mid = self.ids[-1]
            value, when = found[mid]
            if self.failure == 'fault':
                found[mid] = replace(value, fault_bits=1), when
            if self.failure == 'stale':
                found[mid] = value, when - .101
            if self.failure == 'deadline':
                self.clock.wait(.051)
            if self.failure == 'missing':
                del found[mid]
        self.latest.update(found)
        if self.feedback_guard is not None:
            for mid, (value, when) in found.items():
                self.feedback_guard(value, when, mid)
        return found

    def stop_all(self, ids=None):
        ids = self.ids if ids is None else tuple(ids)
        self.own('stop_all', ids)
        self.stop_calls.append((ids, self.clock(), threading.get_ident()))
        if ids != self.ids:
            raise AssertionError('Worker final STOP must cover all six bus axes')
        reports = {}
        for mid in ids:
            self.send(stop_request(phase=TrialPhase.STOP, motor_id=mid))
            value = self.value(mid)
            reports[mid] = {'confirmed': True, 'feedback': asdict(value), 'error': None}
            self.latest[mid] = value, self.clock()
        if self.failure == 'initial_stop' and len(self.stop_calls) == 1:
            reports[ids[-1]]['confirmed'] = False
        if self.failure == 'final_stop' and len(self.stop_calls) > 1:
            reports[ids[-1]]['confirmed'] = False
        return reports


@contextmanager
def boot_file():
    original_read = Path.read_text

    def read_text(path, *args, **kwargs):
        if str(path) == '/proc/sys/kernel/random/boot_id':
            return BOOT_ID + '\n'
        return original_read(path, *args, **kwargs)

    with patch.object(Path, 'read_text', autospec=True, side_effect=read_text):
        yield


class FullbodyHoldTests(unittest.TestCase):
    def fixture(self, *, failure=None, failure_bus='front', noise_ids=()):
        clock = WorkerClock()
        ledger = []
        transports = {name: FakeBus(name, clock,
                                    failure=failure if name == failure_bus else None,
                                    noise_ids=noise_ids, ledger=ledger) for name in BUS_IDS}
        return clock, transports

    def run_hold(self, clock, transports, *, preflight_only=True, review=None, events=None, **kwargs):
        if review is None:
            review = fullbody_review(active=not preflight_only)
        with boot_file(), patch.object(fullbody.threading, 'Barrier', side_effect=clock.barrier):
            return fullbody.run_fullbody_hold(
                transports, UIDS, lambda: None, events.append if events is not None else lambda _: None,
                validated_review=review, preflight_only=preflight_only,
                clock=clock, wait=clock.wait, **kwargs)

    def assert_worker_ownership_and_stops(self, transports):
        owners = []
        for name, transport in transports.items():
            self.assertEqual(len(transport.owner_threads), 1, name)
            owner = next(iter(transport.owner_threads)); owners.append(owner)
            self.assertNotEqual(owner.ident, threading.get_ident())
            self.assertTrue(transport.stop_calls, name)
            self.assertEqual(transport.stop_calls[-1][0], transport.ids)
            self.assertEqual(transport.stop_calls[-1][2], owner.ident)
            self.assertFalse(transport.enabled)
        self.assertEqual(len(set(owners)), 2)

    def test_preflight_runs_twenty_cycles_without_enable_or_nonzero_gain(self):
        clock, transports = self.fixture(noise_ids=(5, 6, 8))
        result = self.run_hold(clock, transports)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['preflight_completed'])
        self.assertFalse(result['motion_completed'])
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(result['reviewed_motor_ids'], [5, 6, 8])
        for name, transport in transports.items():
            self.assertEqual(result['workers'][name]['cycle_count'], 20)
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in transport.frames))
            self.assertTrue(all(frame.data[4:8] == bytes(4)
                                for _, frame, _ in transport.frames if frame.kind == 1))
            self.assertAlmostEqual(transport.stop_calls[-1][1] -
                                   result['workers'][name]['start_monotonic_s'], 1.)
            for leg, window in result['workers'][name]['settled_windows'].items():
                self.assertTrue(window['passed'], leg)
                self.assertTrue(all(motor['sample_count'] == 21 for motor in window['motors'].values()))
                self.assertFalse(window['absolute_rest_proven'])
        self.assert_worker_ownership_and_stops(transports)

    def test_preflight_skips_wait_when_time_crosses_due_after_loop_condition(self):
        class CrossingClock(WorkerClock):
            def __init__(self):
                super().__init__()
                self.armed_thread = None
                self.read_count = self.jump_count = 0
                self.wait_arguments = []

            def __call__(self):
                now = super().__call__()
                if threading.get_ident() == self.armed_thread:
                    self.read_count += 1
                    # After arming: due = clock() + .015; while clock() < due;
                    # then check() reads the preparation deadline. Cross due
                    # on that third read, before until() computes remaining.
                    if self.read_count == 3:
                        with self.lock:
                            self.times[self.armed_thread] += .016
                            now = self.times[self.armed_thread]
                        self.jump_count += 1
                        self.armed_thread = None
                return now

            def wait(self, seconds):
                self.wait_arguments.append(seconds)
                if seconds <= 0:
                    raise AssertionError('Coordinator requested a nonpositive wait')
                super().wait(seconds)

        clock = CrossingClock()
        ledger = []
        transports = {name: FakeBus(name, clock, ledger=ledger) for name in BUS_IDS}
        original_send = transports['front'].send

        def arm_after_last_front_watchdog(command):
            original_send(command)
            frame = ATParser().feed(command)[0]
            if frame.kind == 18 and frame.destination == 6:
                clock.armed_thread = threading.get_ident()

        transports['front'].send = arm_after_last_front_watchdog
        result = self.run_hold(clock, transports)
        self.assertEqual(clock.jump_count, 1)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['preflight_completed'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(clock.wait_arguments)
        self.assertTrue(all(seconds > 0 for seconds in clock.wait_arguments))
        for name, transport in transports.items():
            self.assertEqual(result['workers'][name]['cycle_count'], 20)
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in transport.frames))
            self.assertTrue(all(frame.data[4:8] == bytes(4)
                                for _, frame, _ in transport.frames if frame.kind == 1))
        self.assert_worker_ownership_and_stops(transports)

    def test_all_identity_and_previous_watchdog_reads_finish_before_stage_writes(self):
        clock, transports = self.fixture()
        result = self.run_hold(clock, transports)
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        ledger = transports['front'].ledger
        first_stop = next(index for index, event in enumerate(ledger)
                          if event['operation'] == 'stop_all')
        identities = {event['args'][0] for event in ledger[:first_stop]
                      if event['operation'] == 'parameter' and event['args'][1] is None}
        self.assertEqual(identities, set(range(1, 13)))
        first_watchdog_write = next(index for index, event in enumerate(ledger)
                                    if event['operation'] == 'send' and event['args'][0] == 18)
        previous_reads = {event['args'][0] for event in ledger[:first_watchdog_write]
                          if event['operation'] == 'parameter' and event['args'][1] == 'can_timeout'}
        self.assertEqual(previous_reads, set(range(1, 13)))

    def test_active_hold_runs_one_hundred_fixed_kp3_cycles_then_stops_all_twelve(self):
        clock, transports = self.fixture(noise_ids=(5, 6, 8))
        result = self.run_hold(clock, transports, preflight_only=False)
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['motion_completed'])
        self.assertTrue(result['stop_confirmed'])
        for name, transport in transports.items():
            self.assertEqual(result['workers'][name]['cycle_count'], 100)
            self.assertEqual(sum(frame.kind == 3 for _, frame, _ in transport.frames), 6)
            active = [frame for _, frame, _ in transport.frames
                      if frame.kind == 1 and frame.data[4:8] != bytes(4)]
            self.assertEqual(len(active), 600)
            for frame in active:
                self.assertEqual(frame.wire, motion_request(
                    phase=TrialPhase.POSITION_STEP5, center_rad=transport.centers[frame.destination],
                    motor_id=frame.destination))
            self.assertAlmostEqual(transport.stop_calls[-1][1] -
                                   result['workers'][name]['start_monotonic_s'], 5.)
            cycle_times = sorted({when for when, frame, _ in transport.frames
                                  if frame.kind == 1 and frame.data[4:8] != bytes(4)})
            self.assertEqual(len(cycle_times), 100)
            hold_times = cycle_times
            for earlier, later in zip(hold_times, hold_times[1:]):
                self.assertAlmostEqual(later - earlier, .05)
            self.assertTrue(all(len(samples) == 20 for samples in
                                result['workers'][name]['final_hold_samples'].values()))
        self.assert_worker_ownership_and_stops(transports)

    def test_serialized_activation_is_tick_zero_within_hundred_packet_five_second_budget(self):
        clock, transports = self.fixture()
        exchange_barrier = REAL_BARRIER(2, action=clock.synchronize)
        events = []
        first_active = []
        for transport in transports.values():
            original = transport.feedback_many

            def timed_feedback(commands, expected_ids, original=original):
                commands = tuple(commands)
                frames = [ATParser().feed(command)[0] for command in commands]
                active = any(frame.kind == 1 and frame.data[4:8] != bytes(4) for frame in frames)
                if active:
                    self.assertTrue(all(bus.enabled == set(bus.ids) for bus in transports.values()))
                    # Advance both independent virtual workers together so
                    # one peer's receipt cannot appear to be in the future.
                    exchange_barrier.wait(timeout=1.)
                    first_active.append(clock())
                found = original(commands, expected_ids)
                if active:
                    # Six single-axis reply/quiet exchanges cost 36 ms;
                    # following six-axis batches cost 31 ms.
                    clock.wait(.006 if len(commands) == 1 else .031)
                    exchange_barrier.wait(timeout=1.)
                    found = {mid: (value, clock()) for mid, (value, _) in found.items()}
                return found

            transport.feedback_many = timed_feedback
        result = self.run_hold(clock, transports, preflight_only=False, events=events)
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        for name, transport in transports.items():
            worker = result['workers'][name]
            cycles = [event for event in events if event['kind'] == 'fullbody_cycle' and event['bus'] == name]
            self.assertEqual([row['tick'] for row in cycles], list(range(100)))
            self.assertAlmostEqual(cycles[0]['completed_monotonic_s'] - worker['start_monotonic_s'], .036)
            self.assertTrue(all(row['completed_monotonic_s'] < row['deadline_monotonic_s'] for row in cycles))
            for mid in transport.ids:
                times = [when for when, frame, _ in transport.frames
                         if frame.destination == mid and frame.kind == 1 and frame.data[4:8] != bytes(4)]
                self.assertEqual(len(times), 100)
                self.assertTrue(all(0 <= when - min(first_active) < 5. for when in times))
                self.assertAlmostEqual(times[-1] - worker['start_monotonic_s'], 4.95)
                samples = worker['final_hold_samples'][mid]
                self.assertEqual(len(samples), 20)
                self.assertAlmostEqual(samples[-1][0] - samples[0][0], .95)
            self.assertAlmostEqual(transport.stop_calls[-1][1] - worker['start_monotonic_s'], 5.)
        self.assert_worker_ownership_and_stops(transports)

    def test_serialized_tick_zero_over_fifty_ms_aborts_both_without_extra_commands(self):
        clock, transports = self.fixture()
        transport = transports['front']
        original = transport.feedback_many

        def slow_activation(commands, expected_ids):
            commands = tuple(commands)
            frames = [ATParser().feed(command)[0] for command in commands]
            found = original(commands, expected_ids)
            if any(frame.kind == 1 and frame.data[4:8] != bytes(4) for frame in frames):
                clock.wait(.011)
                found = {mid: (value, clock()) for mid, (value, _) in found.items()}
            return found

        transport.feedback_many = slow_activation
        result = self.run_hold(clock, transports, preflight_only=False)
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertFalse(result['motion_completed'])
        self.assertTrue(result['stop_confirmed'])
        for bus in transports.values():
            active = [frame for _, frame, _ in bus.frames if frame.kind == 1 and frame.data[4:8] != bytes(4)]
            self.assertLessEqual(len(active), 6)
            self.assertTrue(all(sum(frame.destination == mid for frame in active) <= 1 for mid in bus.ids))
        self.assert_worker_ownership_and_stops(transports)

    def test_active_transition_confirms_enable_before_nonzero_gain(self):
        clock, transports = self.fixture()
        result = self.run_hold(clock, transports, preflight_only=False)
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        for transport in transports.values():
            enables = [index for index, (_, frame, _) in enumerate(transport.frames)
                       if frame.kind == 3]
            first_active = next(index for index, (_, frame, _) in enumerate(transport.frames)
                                if frame.kind == 1 and frame.data[4:8] != bytes(4))
            self.assertEqual(len(enables), 6)
            self.assertTrue(enables[-1] < first_active)
            transition = [frame for _, frame, _ in
                          transport.frames[max(enables) + 1:first_active]
                          if frame.kind == 1]
            self.assertEqual(transition, [])

    def test_explicit_id10_kp4_profile_changes_only_its_gain_word_for_all_hundred_commands(self):
        clock, transports = self.fixture()
        review = {**fullbody_review(active=True), 'gain_profile': 'id10-kp4-only'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id10-kp4-only')
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertIsNone(result['Kp'])
        self.assertEqual(result['gain_profile'], 'id10-kp4-only')
        self.assertEqual(result['Kp_by_motor_id'], {mid: 4. if mid == 10 else 3. for mid in range(1, 13)})
        self.assertEqual((result['Kd'], result['torque_feedforward_nm']), (.15, 0.))
        for name, transport in transports.items():
            for mid in transport.ids:
                active = [frame for _, frame, _ in transport.frames
                          if frame.destination == mid and frame.kind == 1 and frame.data[4:8] != bytes(4)]
                expected = motion_request(phase=(TrialPhase.POSITION_STEP5_KP4 if mid == 10
                    else TrialPhase.POSITION_STEP5), center_rad=transport.centers[mid], motor_id=mid)
                self.assertEqual([frame.wire for frame in active], [expected] * 100)
                old = ATParser().feed(motion_request(phase=TrialPhase.POSITION_STEP5,
                    center_rad=transport.centers[mid], motor_id=mid))[0]
                for frame in active:
                    self.assertEqual(frame.can_id, old.can_id)
                    self.assertEqual(frame.data[:4] + frame.data[6:], old.data[:4] + old.data[6:])
                samples = result['workers'][name]['final_hold_samples'][mid]
                self.assertEqual(len(samples), 20)
                self.assertAlmostEqual(samples[-1][0] - samples[0][0], .95)
            self.assertAlmostEqual(transport.stop_calls[-1][1]
                                   - result['workers'][name]['start_monotonic_s'], 5.)
        self.assert_worker_ownership_and_stops(transports)

    def test_explicit_id4_kp4_profile_changes_only_id4_and_still_stops_every_axis(self):
        clock, transports = self.fixture()
        review = {**fullbody_review(active=True), 'gain_profile': 'id4-kp4-only'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id4-kp4-only')
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['Kp_by_motor_id'], {mid: 4. if mid == 4 else 3.
                                                     for mid in range(1, 13)})
        self.assertEqual((result['Kd'], result['torque_feedforward_nm']), (.15, 0.))
        for transport in transports.values():
            for mid in transport.ids:
                active = [frame for _, frame, _ in transport.frames
                          if frame.destination == mid and frame.kind == 1
                          and frame.data[4:8] != bytes(4)]
                expected = motion_request(phase=(TrialPhase.POSITION_STEP5_KP4 if mid == 4
                    else TrialPhase.POSITION_STEP5), center_rad=transport.centers[mid], motor_id=mid)
                self.assertEqual([frame.wire for frame in active], [expected] * 100)
        self.assert_worker_ownership_and_stops(transports)

    def test_explicit_id4_id10_kp4_profile_preserves_other_ten_axes_and_all_stops(self):
        clock, transports = self.fixture()
        review = {**fullbody_review(active=True), 'gain_profile': 'id4-id10-kp4'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id4-id10-kp4')
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['Kp_by_motor_id'], {mid: 4. if mid in (4, 10) else 3.
                                                     for mid in range(1, 13)})
        self.assertEqual((result['Kd'], result['torque_feedforward_nm']), (.15, 0.))
        for transport in transports.values():
            for mid in transport.ids:
                active = [frame for _, frame, _ in transport.frames
                          if frame.destination == mid and frame.kind == 1
                          and frame.data[4:8] != bytes(4)]
                expected = motion_request(phase=(TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10)
                    else TrialPhase.POSITION_STEP5), center_rad=transport.centers[mid], motor_id=mid)
                self.assertEqual([frame.wire for frame in active], [expected] * 100)
        self.assert_worker_ownership_and_stops(transports)

    def test_fixed_gain_profile_requires_matching_explicit_review_before_io(self):
        candidates = [('id10-kp4-only', fullbody_review(active=True)),
                      ('id4-kp4-only', fullbody_review(active=True)),
                      ('id4-id10-kp4', fullbody_review(active=True)),
                      ('kp3', {**fullbody_review(active=True), 'gain_profile': 'id10-kp4-only'})]
        for selection in ('kp4', 'id9-kp4-only', 'id10-kp6-only', None, True, 4., {10: 4.}):
            candidates.append((selection, {**fullbody_review(active=True), 'gain_profile': selection}))
        for selection, review in candidates:
            clock, transports = self.fixture()
            with self.subTest(selection=selection), self.assertRaises(ValueError):
                self.run_hold(clock, transports, preflight_only=False, review=review, gain_profile=selection)
            self.assertTrue(all(not transport.calls for transport in transports.values()))

    def test_id10_kp4_profile_preflight_still_sends_no_enable_or_nonzero_gain(self):
        clock, transports = self.fixture()
        review = {**fullbody_review(), 'gain_profile': 'id10-kp4-only'}
        result = self.run_hold(clock, transports, review=review, gain_profile='id10-kp4-only')
        self.assertEqual(result['status'], 'PREFLIGHT_PASSED_RESET_CONFIRMED', result['errors'])
        self.assertEqual(result['Kp'], 0.)
        self.assertEqual(set(result['Kp_by_motor_id'].values()), {0.})
        for transport in transports.values():
            self.assertFalse(any(frame.kind == 3 for _, frame, _ in transport.frames))
            self.assertTrue(all(frame.data[4:8] == bytes(4)
                                for _, frame, _ in transport.frames if frame.kind == 1))
        self.assert_worker_ownership_and_stops(transports)

    def test_id10_kp4_profile_preserves_fault_deadline_and_final_one_degree_abort(self):
        for bus, failure in [(bus, failure) for bus in BUS_IDS
                             for failure in ('fault', 'stale', 'missing', 'write', 'deadline')] + [('rear', 'final_error')]:
            clock, transports = self.fixture(failure=failure, failure_bus=bus)
            review = {**fullbody_review(active=True), 'gain_profile': 'id10-kp4-only'}
            with self.subTest(bus=bus, failure=failure):
                result = self.run_hold(clock, transports, preflight_only=False, review=review,
                                       gain_profile='id10-kp4-only')
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertFalse(result['motion_completed'])
                self.assertTrue(result['stop_confirmed'])
                if failure == 'final_error':
                    self.assertTrue(any('ID10 final hold error exceeds1-degree' in error for error in result['errors']))
                    self.assertEqual(result['workers']['rear']['cycle_count'], 80)
                self.assert_worker_ownership_and_stops(transports)

    def test_id4_stable_one_point_one_four_degree_bias_passes_only_dual_kp4_hold(self):
        clock, transports = self.fixture(failure='id4_stable_bias', failure_bus='front')
        review = {**fullbody_review(active=True), 'gain_profile': 'id4-id10-kp4'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id4-id10-kp4')
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assert_worker_ownership_and_stops(transports)

        clock, transports = self.fixture(failure='id4_stable_bias', failure_bus='front')
        review = {**fullbody_review(active=True), 'gain_profile': 'kp3'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='kp3')
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID4 final hold error exceeds1-degree' in error for error in result['errors']))
        self.assertTrue(result['stop_confirmed'])

    def test_id4_one_point_six_degree_bias_still_aborts_and_stops_every_axis(self):
        clock, transports = self.fixture(failure='id4_large_bias', failure_bus='front')
        review = {**fullbody_review(active=True), 'gain_profile': 'id4-id10-kp4'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id4-id10-kp4')
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID4 final hold error exceeds1.5-degree' in error for error in result['errors']))
        self.assertTrue(result['stop_confirmed'])
        self.assert_worker_ownership_and_stops(transports)

    def test_id9_stable_negative_one_point_zero_four_degree_bias_passes_only_dual_kp4_hold(self):
        clock, transports = self.fixture(failure='id9_stable_bias', failure_bus='rear')
        review = {**fullbody_review(active=True), 'gain_profile': 'id4-id10-kp4'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id4-id10-kp4')
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(result['workers']['rear']['cycle_count'], 100)
        samples = result['workers']['rear']['final_hold_samples'][9]
        self.assertEqual(len(samples), 20)
        for _, position in samples:
            self.assertAlmostEqual(math.degrees(position - transports['rear'].centers[9]), -1.04)
        self.assert_worker_ownership_and_stops(transports)

        clock, transports = self.fixture(failure='id9_stable_bias', failure_bus='rear')
        review = {**fullbody_review(active=True), 'gain_profile': 'kp3'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='kp3')
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID9 final hold error exceeds1-degree' in error for error in result['errors']))
        self.assertEqual(result['workers']['rear']['cycle_count'], 80)
        self.assertTrue(result['stop_confirmed'])
        self.assert_worker_ownership_and_stops(transports)

    def test_id9_one_point_six_degree_bias_still_aborts_and_stops_every_axis(self):
        clock, transports = self.fixture(failure='id9_large_bias', failure_bus='rear')
        review = {**fullbody_review(active=True), 'gain_profile': 'id4-id10-kp4'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id4-id10-kp4')
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID9 final hold error exceeds1.5-degree' in error for error in result['errors']))
        self.assertEqual(result['workers']['rear']['cycle_count'], 80)
        self.assertTrue(result['stop_confirmed'])
        self.assert_worker_ownership_and_stops(transports)

    def test_dual_kp4_profile_keeps_id10_final_one_degree_gate(self):
        clock, transports = self.fixture(failure='final_error', failure_bus='rear')
        review = {**fullbody_review(active=True), 'gain_profile': 'id4-id10-kp4'}
        result = self.run_hold(clock, transports, preflight_only=False, review=review,
                               gain_profile='id4-id10-kp4')
        self.assertEqual(result['status'], 'ABORTED', result['errors'])
        self.assertTrue(any('ID10 final hold error exceeds1-degree' in error for error in result['errors']))
        self.assertEqual(result['workers']['rear']['cycle_count'], 80)
        self.assertTrue(result['stop_confirmed'])
        self.assert_worker_ownership_and_stops(transports)

    def test_every_enable_and_active_write_rechecks_all_twelve_axes(self):
        clock, transports = self.fixture()
        original_check = fullbody.check_feedback

        def observed_check(value, center, received, now, **kwargs):
            for transport in transports.values():
                if (threading.current_thread() in transport.owner_threads
                        and transport.guard_observations is not None):
                    transport.guard_observations.append(center)
            return original_check(value, center, received, now, **kwargs)

        with patch.object(fullbody, 'check_feedback', side_effect=observed_check):
            result = self.run_hold(clock, transports, preflight_only=False)
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        all_centers = {value for transport in transports.values() for value in transport.centers.values()}
        for transport in transports.values():
            self.assertEqual(sum(phase == 'enable' for phase, _ in transport.guard_checks), 6)
            self.assertEqual(sum(phase == 'motion' for phase, _ in transport.guard_checks), 600)
            for phase, centers in transport.guard_checks:
                self.assertEqual(len(centers), 12, phase)
                self.assertEqual(set(centers), all_centers, phase)

    def test_invalid_review_or_identity_mapping_rejects_before_any_transport_io(self):
        failures = []
        for key, value in (('schema', 'other'), ('scope', 'other'), ('review_complete', False),
                           ('source_files_verified', False), ('boot_id', 'previous-boot'),
                           ('firmware', '0.5.0.12'), ('sha256', 'z' * 64),
                           ('calibration_verified', True), ('learned_policy_allowed', True),
                           ('standing_allowed', True), ('l_target_replay_allowed', True),
                           ('reviewed_motor_ids', [6, 8]), ('motor_ids', list(range(1, 12)))):
            review = fullbody_review(); review[key] = value
            failures.append((key, review))
        for mid in (5, 6, 8):
            review = fullbody_review(); review['motor_reviews'][str(mid)]['motor_uid_hex'] = 'f' * 16
            failures.append((f'ID{mid}_uid', review))
            review = fullbody_review(); del review['motor_reviews'][str(mid)]
            failures.append((f'ID{mid}_missing', review))
        for label, review in failures:
            clock, transports = self.fixture()
            with self.subTest(failure=label), self.assertRaises(ValueError):
                self.run_hold(clock, transports, review=review)
            self.assertTrue(all(not transport.calls for transport in transports.values()))
        for expected in ({mid: uid for mid, uid in UIDS.items() if mid != 12},
                         {**UIDS, 12: UIDS[11]}, {**UIDS, '12': UIDS[12]}):
            clock, transports = self.fixture()
            with self.subTest(expected=expected), boot_file(), self.assertRaises(ValueError):
                fullbody.run_fullbody_hold(transports, expected, lambda: None, lambda _: None,
                    validated_review=fullbody_review(), clock=clock, wait=clock.wait)
            self.assertTrue(all(not transport.calls for transport in transports.values()))

    def test_active_hold_requires_explicit_authorization_before_io(self):
        clock, transports = self.fixture()
        with self.assertRaises(ValueError):
            self.run_hold(clock, transports, preflight_only=False, review=fullbody_review(active=False))
        self.assertTrue(all(not transport.calls for transport in transports.values()))

    def test_invalid_selector_or_nonboolean_mode_rejects_before_io(self):
        for reviewed in ((6, 8), (5, 6, 8, 9), (8, 6, 5), (5, 6, 8, 8),
                         ('5', '6', '8'), (5., 6., 8.)):
            clock, transports = self.fixture()
            with self.subTest(reviewed=reviewed), self.assertRaises(ValueError):
                self.run_hold(clock, transports, reviewed_motor_ids=reviewed)
            self.assertTrue(all(not transport.calls for transport in transports.values()))
        for mode in (0, 1, None, 'false'):
            clock, transports = self.fixture()
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                self.run_hold(clock, transports, preflight_only=mode)
            self.assertTrue(all(not transport.calls for transport in transports.values()))

    def test_invalid_bus_pair_or_shared_serial_rejects_before_io(self):
        for failure in ('missing', 'wrong_ids', 'same_serial'):
            clock, transports = self.fixture()
            original = dict(transports)
            if failure == 'missing': del transports['rear']
            if failure == 'wrong_ids': transports['rear'].ids = BUS_IDS['front']
            if failure == 'same_serial': transports['rear'].serial = transports['front'].serial
            with self.subTest(failure=failure), self.assertRaises(ValueError):
                self.run_hold(clock, transports)
            self.assertTrue(all(not transport.calls for transport in original.values()))

    def test_pre_enable_stage_failure_aborts_both_workers_without_enabling(self):
        for failure in ('uid', 'initial_stop', 'run_mode', 'current', 'voltage', 'velocity',
                        'nonfinite', 'position_mismatch', 'watchdog_readback'):
            clock, transports = self.fixture(failure=failure)
            with self.subTest(failure=failure):
                result = self.run_hold(clock, transports, preflight_only=False)
                self.assertEqual(result['status'], 'ABORTED', result['errors'])
                self.assertFalse(result['motion_completed'])
                self.assertTrue(all(not any(frame.kind == 3 for _, frame, _ in transport.frames)
                                    for transport in transports.values()))
                self.assert_worker_ownership_and_stops(transports)

    def test_all_axes_use_position_gate_when_velocity_field_is_noisy(self):
        for mid in (1, 4, 7, 9, 12):
            clock, transports = self.fixture(noise_ids=(mid,))
            with self.subTest(mid=mid):
                result = self.run_hold(clock, transports, preflight_only=False)
                self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
                self.assertTrue(all(any(frame.kind == 3 for _, frame, _ in transport.frames)
                                    for transport in transports.values()))
                warned = False
                for worker in result['workers'].values():
                    for window in worker['settled_windows'].values():
                        self.assertEqual(window['profile'], 'position-v2-all')
                        self.assertTrue(window['passed'])
                        warned |= bool(window['warnings'])
                self.assertTrue(warned)
                self.assert_worker_ownership_and_stops(transports)

    def test_quantized_stationary_id3_replay_accepts_only_bounded_velocity_bias(self):
        replay = id3_noisy_velocity_replay()
        rows = samples()
        for n, row in enumerate(rows[3]):
            row['feedback']['velocity_rad_s'] = replay[n]
        report = fullbody.evaluate_settled_window(
            rows, CENTERS, profile=fullbody.FULLBODY_POSITION_PROFILE)
        self.assertTrue(report['passed'], report['errors'])
        self.assertEqual(report['limits']['abs_velocity_mean_rad_s'], .025)
        self.assertEqual(report['motors'][3]['position_range_rad'], 0.)
        self.assertEqual(report['motors'][3]['OLS_slope_rad_s'], 0.)
        self.assertEqual(report['motors'][3]['tail_position_range_rad'], 0.)
        self.assertAlmostEqual(report['motors'][3]['velocity_mean_rad_s'], .03375)
        self.assertAlmostEqual(report['motors'][3]['velocity_RMS_rad_s'], .0634)
        self.assertTrue(report['motors'][3]['static_position_bias_exception_applied'])
        self.assertTrue(any('bounded Type2 velocity bias' in warning
                            for warning in report['warnings']))
        self.assertFalse(report['absolute_rest_proven'])

        def rejected(modify):
            changed = deepcopy(rows)
            modify(changed[3])
            result = fullbody.evaluate_settled_window(
                changed, CENTERS, profile=fullbody.FULLBODY_POSITION_PROFILE)
            self.assertFalse(result['passed'])
            return result

        # Even sub-threshold position creep is enough to deny the noise-only
        # exception. The original position/tail/feedback gates remain active.
        creep = rejected(lambda axis: [row['feedback'].__setitem__(
            'protocol_position_rad', CENTERS[3] + n*.00001)
            for n, row in enumerate(axis)])
        self.assertTrue(any('abs_velocity_mean_rad_s' in error for error in creep['errors']))
        tail = rejected(lambda axis: axis[-1]['feedback'].__setitem__(
            'protocol_position_rad', CENTERS[3] + .0011))
        self.assertTrue(any('tail_position_range_rad' in error for error in tail['errors']))
        changed_count = rejected(lambda axis: axis[10]['feedback'].__setitem__('position_u16', 32766))
        self.assertTrue(any('abs_velocity_mean_rad_s' in error for error in changed_count['errors']))
        for field, value in (('fault_bits', 1), ('mode_state', 2), ('velocity_rad_s', .51)):
            with self.subTest(field=field):
                self.assertFalse(rejected(lambda axis: axis[10]['feedback'].__setitem__(field, value))['passed'])
        stale = rejected(lambda axis: axis[10].__setitem__(
            'checked_monotonic_s', axis[10]['received_monotonic_s'] + .11))
        self.assertTrue(any('Stale' in error for error in stale['errors']))
        high_bias = rejected(lambda axis: [row['feedback'].__setitem__(
            'velocity_rad_s', .041) for row in axis])
        self.assertTrue(any('abs_velocity_mean_rad_s' in error for error in high_bias['errors']))
        mean, rms = .03375, .081
        deviation = math.sqrt((rms*rms - mean*mean) * 21 / 20)
        high_rms = rejected(lambda axis: [row['feedback'].__setitem__(
            'velocity_rad_s', mean + (deviation if n % 2 == 0 else -deviation)
            if n < 20 else mean) for n, row in enumerate(axis)])
        self.assertTrue(any('abs_velocity_mean_rad_s' in error for error in high_rms['errors']))

    def test_one_count_id8_chatter_and_net_shift_accept_bounded_bias(self):
        # Sanitized ID8 disabled-window replay: 21 samples toggle between two
        # adjacent Type2 counts and return to the first count. The independent
        # position slope is tiny despite a +.028229 rad/s reported mean.
        counts = [33200, 33199, 33200, 33199, 33200, 33200, 33200,
                  33200, 33199, 33200, 33199, 33199, 33200, 33199,
                  33200, 33199, 33199, 33199, 33200, 33200, 33200]
        velocities = [.112153811, .035858701, .025177386, .029755093,
                      .069428550, .041962310, .023651484, -.055695430,
                      .011444266, .119783322, .020599680, .040436408,
                      .045014115, -.048065919, .069428550, -.032806897,
                      .025177386, .008392462, -.019073777, .008392462,
                      .061799039]
        rows = samples()
        count_rad = 25.14 / 65535
        for row, count, velocity in zip(rows[2], counts, velocities):
            row['feedback'].update(position_u16=count,
                protocol_position_rad=CENTERS[2] + (count-counts[0])*count_rad,
                velocity_rad_s=velocity)
        rear_rows = {mid+6: axis for mid, axis in rows.items()}
        rear_centers = {mid+6: center for mid, center in CENTERS.items()}

        def evaluate(axis_rows):
            return fullbody.evaluate_settled_window(
                axis_rows, rear_centers, profile=fullbody.FULLBODY_POSITION_PROFILE)

        report = evaluate(rear_rows)
        self.assertTrue(report['passed'], report['errors'])
        self.assertEqual(report['motors'][8]['position_count_span'], 1)
        self.assertTrue(report['motors'][8]['position_count_returned_to_start'])
        self.assertTrue(report['motors'][8]['position_count_consistent'])
        self.assertTrue(report['motors'][8]['static_position_bias_exception_applied'])
        self.assertAlmostEqual(report['motors'][8]['velocity_mean_rad_s'], .02822919, places=7)

        one_count_net = deepcopy(rear_rows)
        for row in one_count_net[8][10:]:
            row['feedback'].update(position_u16=33199,
                protocol_position_rad=CENTERS[2]-count_rad)
        net_report = evaluate(one_count_net)
        self.assertTrue(net_report['passed'], net_report['errors'])
        self.assertEqual(net_report['motors'][8]['position_count_span'], 1)
        self.assertFalse(net_report['motors'][8]['position_count_returned_to_start'])
        self.assertTrue(net_report['motors'][8]['static_position_bias_exception_applied'])

        # Two-count movement, inconsistent count/position, and a fast late
        # change must still fail even with the same biased speed field.
        for change in ('two_count_net_drift', 'two_count_oscillation',
                       'count_position_mismatch', 'fast_tail_step'):
            changed = deepcopy(rear_rows)
            axis = changed[8]
            if change == 'two_count_net_drift':
                for row in axis[10:]:
                    row['feedback'].update(position_u16=33198,
                        protocol_position_rad=CENTERS[2]-2*count_rad)
            elif change == 'two_count_oscillation':
                axis[10]['feedback'].update(position_u16=33198,
                    protocol_position_rad=CENTERS[2]-2*count_rad)
            elif change == 'count_position_mismatch':
                axis[10]['feedback']['position_u16'] = 33200
            else:
                # At the shortest permitted tail span, a one-count step in
                # the final three samples exceeds the unchanged tail-slope
                # limit, even though the count-span exception would apply.
                for n, row in enumerate(axis):
                    if n >= 15:
                        row['received_monotonic_s'] = 11.5 + (n-15)*.09
                        row['checked_monotonic_s'] = row['received_monotonic_s'] + .004
                    count = 33199 if n >= 18 else 33200
                    row['feedback'].update(position_u16=count,
                        protocol_position_rad=CENTERS[2]+(count-33200)*count_rad)
            rejected = evaluate(changed)
            with self.subTest(change=change):
                self.assertFalse(rejected['passed'])
                if change == 'fast_tail_step':
                    self.assertTrue(any('tail_abs_OLS_slope_rad_s' in error
                                        for error in rejected['errors']))
                else:
                    self.assertFalse(rejected['motors'][8]['static_position_bias_exception_applied'])
                    self.assertTrue(any('abs_velocity_mean_rad_s' in error
                                        for error in rejected['errors']))

    def test_fullbody_hold_accepts_id3_replay_and_still_stops_every_axis(self):
        clock, transports = self.fixture()
        front = transports['front']
        original_value = front.value
        replay = id3_noisy_velocity_replay()

        def value(mid):
            feedback = original_value(mid)
            if mid == 3 and not front.active_mode:
                return replace(feedback,
                               velocity_rad_s=replay[front.disabled_batches % len(replay)])
            return feedback

        front.value = value
        result = self.run_hold(clock, transports, preflight_only=False)
        self.assertEqual(result['status'], 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED', result['errors'])
        self.assertTrue(result['workers']['front']['settled_windows']['FR']['motors'][3]
                        ['static_position_bias_exception_applied'])
        self.assertTrue(result['stop_confirmed'])
        self.assert_worker_ownership_and_stops(transports)

    def test_fullbody_biased_velocity_with_real_small_creep_never_enables(self):
        clock, transports = self.fixture()
        front = transports['front']
        original_value = front.value
        replay = id3_noisy_velocity_replay()

        def value(mid):
            feedback = original_value(mid)
            if mid == 3 and not front.active_mode:
                return replace(feedback,
                               protocol_position_rad=feedback.protocol_position_rad
                                   + front.disabled_batches * .00001,
                               velocity_rad_s=replay[front.disabled_batches % len(replay)])
            return feedback

        front.value = value
        result = self.run_hold(clock, transports, preflight_only=False)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertTrue(any('abs_velocity_mean_rad_s' in error for error in result['errors']))
        self.assertFalse(any(frame.kind == 3 for transport in transports.values()
                             for _, frame, _ in transport.frames))
        self.assert_worker_ownership_and_stops(transports)

    def test_one_bus_active_fault_stale_missing_write_or_fifty_ms_overrun_stops_both(self):
        for name in BUS_IDS:
            for failure in ('fault', 'stale', 'missing', 'write', 'deadline'):
                clock, transports = self.fixture(failure=failure, failure_bus=name)
                with self.subTest(bus=name, failure=failure):
                    result = self.run_hold(clock, transports, preflight_only=False)
                    self.assertEqual(result['status'], 'ABORTED', result['errors'])
                    self.assertFalse(result['motion_completed'])
                    self.assertTrue(result['errors'])
                    self.assertTrue(all(worker['cycle_count'] < 100 for worker in result['workers'].values()))
                    self.assert_worker_ownership_and_stops(transports)

    def test_failed_final_stop_cannot_report_success(self):
        clock, transports = self.fixture(failure='final_stop')
        result = self.run_hold(clock, transports)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['stop_confirmed'])
        self.assert_worker_ownership_and_stops(transports)


if __name__ == '__main__':
    unittest.main()
