"""Opt-in timing options F0-F4: default equivalence and each option.

File-only: mock or synthetic owners, virtual clocks, no device, library,
Torch or network. Nothing here is a timing qualification or output approval.
"""
import contextlib
import copy
import gc
import json
import os
from pathlib import Path
import shutil
import signal
import tempfile
import threading
import time
from types import MappingProxyType, SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw.policy_motion_envelope import PolicyMotionEnvelope
from experiments.four_bus_diagnostic.transport_adapter import Batch, PORTS
from . import timing_evidence as E, type1_foreground as F, type1_profile as P, type1_runner as R
from . import type1_transport as T
from . import test_type1_foreground as TF, test_type1_profile as TP
from .test_type1_runner import Harness, MockType1, Observer, admitted
from .test_type1_transport import ARMED, MockSession

# Pinned from the pre-option modules (canonical SHA256 of R.plan(admitted(mode)) and of PACING).
DEFAULT_PLAN_SHA256 = {'zero_gain_timing': 'a78201a72ea7df4ec009a9f80fd8cf29f087ef8d768eb3769af79a7f3a2e9552',
                       'learned_boxed': 'd8a532f051f039b45e770212b6df21b9c27ab020edac273c4e28fe6d95d33fb2'}
PACING_SHA256 = 'e1b5a934624831f9bb00c4aac50c618299aa7309636933dd973f82566f319349'
DEFAULT_ROW_KEYS = frozenset((
    'acquired_ns', 'acquisition_ms', 'begin_ns', 'blended_target_by_id', 'command', 'completed', 'cycle_end_ns',
    'decision', 'encode_end_ns', 'envelope_and_encode_ms', 'final_gate_ns', 'final_write_ns', 'first_ns', 'gather_ns',
    'hard_end_ns', 'hold', 'hold_first_write_ns', 'hold_last_reply_ns', 'imu', 'imu_limit_check', 'index',
    'infer_end_ns', 'inference_ms', 'input_to_last_output_reply_ms', 'iteration_ms', 'join_deadline_ns', 'label',
    'model_target_by_id', 'observed', 'output', 'output_exchange_ms', 'output_feedback', 'output_first_write_ns',
    'output_last_reply_ns', 'output_submit_ns', 'release_lateness_ms', 'release_ns', 'reply_return_ns',
    'request_count', 'sample', 'slot', 'voltage', 'voltage_join_ms', 'voltage_join_ns', 'voltage_last_reply_ns',
    'weight'))
DEFAULT_REPORT_KEYS = frozenset((
    'absolute_deadline_ns', 'after_announcement_watchdog_readback_by_id', 'after_enable_voltage_by_id',
    'all_workers_joined_monotonic_ns', 'announcement_completed_ns', 'approved_for_runtime', 'backend_usage', 'boot_id',
    'completed_cycles', 'contract_sha256', 'cycles', 'devices_opened', 'duration_s', 'enable_order', 'encoder',
    'envelope_gains', 'epoch_ns', 'errors', 'failure_retained', 'final_gate', 'firmware_versions_match_watchdog_review',
    'first_cycle_post_reply', 'fixed_branch_turns_by_id', 'fixed_offsets_rad_by_id', 'flag_semantics',
    'full_current_check_points', 'gc_enabled_during_cycles', 'host_watchdog_ns', 'host_watchdog_placement',
    'host_watchdog_reason', 'ids_by_port', 'imu_accel_input_hypothesis_selected', 'imu_limit_check_points',
    'imu_limits', 'initial_hold', 'initial_raw_rad_by_id', 'learned_targets_attempted', 'live_type1_qualified',
    'max_iteration_ms', 'max_stop_s', 'mode', 'model_setup', 'motor_enable_sent', 'motor_power_epoch',
    'motor_watchdog_ticks', 'normal_ramp_completed', 'opens_devices', 'operating_settings',
    'output_approval_granted_here', 'owner_settlement', 'pacing', 'per_cycle_requests', 'physical_cutoff_required',
    'physical_observations', 'positive_gain_sent', 'post_reply_deadline_allowance_uses', 'post_reply_late_cycles',
    'post_reply_policy', 'pre_enable_imu', 'pre_enable_imu_limit_check', 'pre_enable_voltage_by_id', 'preflight',
    'primary_error', 'q0_model_rad_by_id', 'raw_journal', 'request_gap_ns', 'request_schedule', 'request_window',
    'restoration', 'schema', 'startup_displacement_checks', 'status', 'stop_at_s', 'stop_confirmed',
    'stop_dispatch_errors', 'stop_latched_ns', 'stop_policy', 'stop_reason', 'stop_results',
    'timing_qualification_granted_here', 'trial_origin_model_rad_by_id', 'type1_sent', 'voltage_guard',
    'watchdog_settings', 'worker_settings', 'zero_gain_enable_transition', 'zero_gain_timing_outputs'))
K_US = 11250
LEAD_US = 1500
K_WITH_LEAD_US = 9200  # F1+F3 static check: K <= 10710 - max(L, 1250) us, 9210 at L = 1500.


def optioned(mode='zero_gain_timing', **options):
    value = admitted(mode)
    value['pacing'] = P.selected_pacing(options)
    return value


class DecodeOnceMock(MockType1):
    """Owner publishing read-only rows; verify_batch mirrors Type1Transport (hold fast path, voltage re-decoded)."""
    decode_once = True

    def __init__(self, group, harness):
        super().__init__(group, harness)
        self.published, self.fast, self.full = {}, [], []

    def _exchange(self, label, pairs, deadline):
        batch = super()._exchange(label, pairs, deadline)
        batch = Batch(batch.group, batch.label, batch.records, batch.stats, MappingProxyType(dict(batch.rows)),
                      batch.record_image, batch.stats_image, batch.completed_ns)
        self.last_batch = self.published[id(batch)] = batch
        return batch

    def verify_batch(self, batch, label):
        if self.published.get(id(batch)) is not batch or batch.label != {'feedback_hold': 'hold'}.get(label, label):
            raise ValueError('Genuine current owner batch required')
        if label == 'feedback_hold':
            batch.verify_images()
            self.fast.append(self.holds)
            return batch.rows
        self.full.append(label)
        return batch.verify()


class PrearmedMock(MockType1):
    """Native sda_subset_exchange_at stand-in: armed owners write nothing before the release or after a cancel."""
    prearmed_hold = True

    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check, not_before_ns=None):
        with self.h.lock:
            self.h.armed.append((self.group.port, not_before_ns, self.h.clock.now))
        if not_before_ns is not None:
            limit = time.monotonic()+2
            while self.h.clock.now < not_before_ns:
                if self.h.cancelled.is_set() or time.monotonic() > limit:
                    self.h.refused.append(self.group.port)
                    error = RuntimeError('Native cancel byte set before the pre-armed write')
                    if not prefix_future.done():
                        prefix_future.set_exception(error)
                    raise error
                time.sleep(.0001)
        return super().hold_then_voltage(wires, voltage_id, prefix_future, deadline_ns=deadline_ns, check=check)


class PrepCostMock(PrearmedMock):
    """Owner Python preparation (GIL-serialized) before the native call, which refuses at/after the release."""
    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check, not_before_ns=None):
        if not_before_ns is not None:
            with self.h.lock:
                now = self.h.clock.advance_to(self.h.clock.now+self.h.prep_ns)
            if now >= not_before_ns:
                self.h.refused.append(self.group.port)
                error = TimeoutError('Pre-armed release must be ahead (at most 5 ms) and before the deadline')
                prefix_future.set_exception(error)
                raise error
        return super().hold_then_voltage(wires, voltage_id, prefix_future, deadline_ns=deadline_ns, check=check,
                                         not_before_ns=not_before_ns)


class StallMock(MockType1):
    """r1 cycle-26 type: every owner's first hold write moved to release+offset in the selected cycle."""
    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check):
        offset = self.c.get('hold_at', {}).get(self.holds)
        if offset is not None:
            self.h.clock.advance_to(self.h.releases[-1]+offset)
        return super().hold_then_voltage(wires, voltage_id, prefix_future, deadline_ns=deadline_ns, check=check)


class FakeEvidence:
    """TimingEvidence-shaped reader with synthetic counters."""
    delta = staticmethod(E.delta)
    run_delta = staticmethod(E.TimingEvidence.run_delta)

    def __init__(self):
        self.samples = self.runs = 0
        self.bound, self.closed, self.threads = None, False, []

    def bind(self, threads):
        self.bound = dict(threads)
        return {'threads': dict(threads)}

    def sample(self):
        self.samples += 1
        self.threads.append(threading.get_native_id())
        return {name: {'run_delay_ns': 10*self.samples, 'nivcsw': self.samples, 'minflt': None} for name in self.bound}

    def run_counters(self):
        self.runs += 1
        return {'vmstat': {'thp_fault_alloc': self.runs, 'compact_stall': 0}, 'interrupts': {'7': [self.runs, 4], 'IPI0': [2, 2]}}

    def close(self):
        self.closed = True


class OptionHarness(Harness):
    mock = MockType1

    def setUp(self):
        super().setUp()
        self.lock = threading.Lock()
        self.releases, self.waits, self.armed, self.refused, self.ledger = [], [], [], [], []
        self.current_times = []
        self.prearm = False

    def factory(self, group):
        value = self.mock(group, self)
        self.adapters[group.port] = value
        return value

    def current(self):
        self.current_times.append((self.cycle_release, self.clock.now))
        self.ledger.append(('check', self.cycle_release))
        super().current()

    def release_wait(self, when):
        self.waits.append(when)
        if not self.prearm:
            self.releases.append(when)
            return super().release_wait(when)
        if len(self.waits) % 2:  # Pre-armed wake at release-lead.
            self.cycle_release += 1
            return self.clock.advance_to(when)
        self.releases.append(when)
        limit = time.monotonic()+2
        while (sum(1 for _, at, _ in list(self.armed) if at == when) < 4 and not self.cancelled.is_set() and
               time.monotonic() < limit):
            time.sleep(.0001)  # The four owners arm in native code before the release.
        hook = self.controls.get('release_hook')
        if hook is not None:
            hook(len(self.releases)-1, when)
        return self.clock.advance_to(when)

    def gate_observer(self, gates):
        """Natural final gate near release+gates[cycle] (call n is cycle n-1 before gain-down)."""
        def hook(calls, targets):
            gate = gates(calls-1) if callable(gates) else gates.get(calls-1)
            if gate is not None:
                self.clock.advance_to(self.releases[-1]+gate)
        return Observer(hook)

    @contextlib.contextmanager
    def recorded_steps(self):
        steps, original = [], PolicyMotionEnvelope.step
        def step(envelope, target, sample, *, now_s):
            steps.append(now_s)
            self.ledger.append(('step', len(steps)-1))
            return original(envelope, target, sample, now_s=now_s)
        with patch.object(PolicyMotionEnvelope, 'step', step):
            yield steps

    def waiter(self, *, cancel_at=None, raise_at=None, short=False):
        calls = []
        def wait(target):
            cycle = len(self.releases)-1
            calls.append((cycle, target))
            self.ledger.append(('wait', cycle))
            if raise_at == cycle:
                raise RuntimeError('Cancelled during command phase wait')
            if cancel_at == cycle:
                self.injected_cancel.set()
            return self.clock.advance_to(target)-(2_000 if short else 0)
        wait.calls = calls
        return wait

    def no_output_from(self, cycle):
        for port in PORTS:
            self.assertEqual(self.output_cycles(port), cycle, port)


# ----------------------------------------------------------------------------- default equivalence
class DefaultEquivalenceTests(OptionHarness, unittest.TestCase):
    def test_default_plan_contract_pacing_and_report_keys_unchanged(self):
        for mode in R.MODES:
            plan = R.plan(admitted(mode))
            self.assertEqual(P.canonical_sha256(plan), DEFAULT_PLAN_SHA256[mode], mode)
            self.assertEqual(R.plan(optioned(mode)), plan)  # selected_pacing() with nothing selected.
        self.assertEqual(P.canonical_sha256(P.PACING), PACING_SHA256)
        self.assertEqual(P.selected_pacing(), P.PACING)
        self.assertEqual(P.selected_pacing({'command_phase_offset_us': None, 'decode_once': False,
                                            'prearmed_hold_lead_us': None, 'gc_freeze': False}), P.PACING)
        self.assertEqual(P.pacing_options(copy.deepcopy(P.PACING)), {})
        result = self.run_case('zero_gain_timing')
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        self.assertEqual(set(result), DEFAULT_REPORT_KEYS)
        self.assertTrue(all(set(row) == DEFAULT_ROW_KEYS for row in result['cycles']))
        self.assertEqual(result['pacing'], P.PACING)

    def test_default_path_calls_and_kwargs_unchanged(self):
        seen, verified = [], []
        original_hold = MockType1.hold_then_voltage
        def hold(adapter, *args, **kwargs):
            seen.append(sorted(kwargs))
            return original_hold(adapter, *args, **kwargs)
        original_verify = Batch.verify
        def verify(batch):
            verified.append(batch.label)
            return original_verify(batch)
        admitted_begins, original_admit = [], R.PostReplyDeadlineBudget.admit
        def admit(budget, **kwargs):
            admitted_begins.append(kwargs['begin_ns'])
            return original_admit(budget, **kwargs)
        with patch.object(MockType1, 'hold_then_voltage', hold), patch.object(Batch, 'verify', verify), \
                patch.object(gc, 'freeze', side_effect=AssertionError('freeze')), \
                patch.object(R.PostReplyDeadlineBudget, 'admit', admit):
            result = self.run_case('zero_gain_timing')
        cycles = len(result['cycles'])
        self.assertTrue(seen and all(keys == ['check', 'deadline_ns'] for keys in seen))
        # Post-reply rules and the output join take main's actual wake as the cycle begin, as before.
        lateness = int(R.POST_REPLY_V1['max_lateness_ms']*1e6)
        self.assertEqual(admitted_begins, [row['begin_ns'] for row in result['cycles']])
        for row in result['cycles']:
            self.assertEqual(row['join_deadline_ns'], min(row['first_ns']+20_000_000,
                                                          row['begin_ns']+R.PERIOD_NS+lateness))
        # Gather + final gate re-decode every hold and voltage image, exactly as before.
        self.assertEqual(verified.count('hold'), 2*4*cycles)
        self.assertEqual(verified.count('voltage'), 2*4*cycles)
        self.assertEqual(len(self.waits), cycles)
        self.assertNotIn('gc_freeze', result)
        self.assertNotIn('timing_evidence', result)

    def test_command_neutral_options_keep_the_wire_bytes(self):
        def wires():
            return {port: [tuple(bytes(r.tx) for r in raw[0]) for label, raw in self.adapters[port].journal
                           if label in ('hold', 'voltage', 'output')] for port in PORTS}
        default = self.run_case('zero_gain_timing')
        reference = wires()
        self.setUp()
        self.mock = DecodeOnceMock
        selected = self.run_case('zero_gain_timing', value=optioned(decode_once=True, gc_freeze=True),
                                 timing_evidence=FakeEvidence())
        for result in (default, selected):
            self.assertTrue(result['status'].startswith('COMPLETE'), result['primary_error'])
        for port, sent in wires().items():
            common = min(len(sent), len(reference[port]))
            self.assertGreater(common, 200)
            self.assertEqual(sent[:common], reference[port][:common], port)

    def test_option_capabilities_are_injected_exactly_when_selected(self):
        with self.assertRaisesRegex(ValueError, 'Command phase waiter'):
            self.run_case('zero_gain_timing', command_wait=lambda target: target)
        with self.assertRaisesRegex(ValueError, 'Command phase waiter'):
            self.run_case('zero_gain_timing', value=optioned(command_phase_offset_us=K_US))
        with self.assertRaisesRegex(ValueError, 'Timing evidence reader'):
            self.run_case('zero_gain_timing', timing_evidence=object())
        for options, mock in (({'decode_once': True}, MockType1), ({}, DecodeOnceMock),
                              ({'prearmed_hold_lead_us': 1000}, MockType1), ({}, PrearmedMock)):
            with self.subTest(options=options, mock=mock.__name__):
                self.setUp()
                self.mock = mock
                result = self.run_case('zero_gain_timing', value=optioned(**options))
                # Rejected at the first factory call: unopened ports make the STOP unconfirmed.
                self.assertEqual(result['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
                self.assertIn('selections must equal the admitted contract', result['primary_error']['message'])
                self.assertFalse(result['motor_enable_sent'] or result['type1_sent'])
                self.assertTrue(self.adapters and all(adapter.closed for adapter in self.adapters.values()))


# ----------------------------------------------------------------------------- F1 command phase
class CommandPhaseTests(OptionHarness, unittest.TestCase):
    def run_paced(self, gates, mode='zero_gain_timing', offset=K_US, wait=None, **kw):
        value = optioned(mode, command_phase_offset_us=offset) if offset is not None else admitted(mode)
        wait = wait or (self.waiter() if offset is not None else None)
        with self.recorded_steps() as steps:
            result = self.run_case(mode, observer=self.gate_observer(gates), value=value, command_wait=wait, **kw)
        return result, steps, wait

    def test_step_uses_release_plus_k_unless_the_natural_gate_is_later(self):
        gates = {3: 10_000_000, 4: 12_000_000, 5: 10_500_000}
        result, steps, wait = self.run_paced(gates)
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        rows = result['cycles']
        for index in (3, 5):
            row, target = rows[index], rows[index]['release_ns']+K_US*1000
            self.assertLess(row['natural_gate_ns'], target)
            self.assertTrue(target <= row['command_ns'] <= target+50_000, index)
            self.assertEqual(steps[index], row['command_ns']/1e9)
            self.assertEqual(row['final_gate_ns'], row['natural_gate_ns'])
            self.assertAlmostEqual(row['command_wait_ms'], (row['command_ns']-row['natural_gate_ns'])/1e6)
        row = rows[4]
        self.assertGreaterEqual(row['natural_gate_ns'], row['release_ns']+12_000_000)
        self.assertEqual((row['command_ns'], steps[4]), (row['natural_gate_ns'], row['natural_gate_ns']/1e9))
        self.assertNotIn(4, [cycle for cycle, _ in wait.calls])
        self.assertTrue({3, 5} <= {cycle for cycle, _ in wait.calls})
        self.assertTrue(all(target == rows[cycle]['release_ns']+K_US*1000 for cycle, target in wait.calls))
        self.assertEqual(result['command_phase']['offset_us'], K_US)
        self.assertEqual(result['pacing']['command_phase_offset_us'], K_US)
        self.assertTrue(all(DEFAULT_ROW_KEYS < set(row) for row in rows))
        self.assertTrue(all(row['hard_end_ns']-row['release_ns'] <= 20_100_000 for row in rows))

    def test_wait_follows_boundary3_and_precedes_step(self):
        result, steps, wait = self.run_paced({2: 9_000_000})
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        for cycle, _ in wait.calls:
            events = [event for event in self.ledger if event in (('check', cycle+1), ('wait', cycle), ('step', cycle))]
            self.assertEqual(events, [('check', cycle+1)]*3+[('wait', cycle), ('step', cycle)], cycle)

    def test_cancel_or_failure_during_the_wait_sends_no_output(self):
        for name, wait in (('cancel', lambda: self.waiter(cancel_at=4)), ('raise', lambda: self.waiter(raise_at=4)),
                           ('backdated', lambda: self.waiter(short=True))):
            with self.subTest(name):
                self.setUp()
                result, steps, _ = self.run_paced({}, wait=wait())
                self.assertEqual(result['status'], 'ABORTED')
                cycle = 0 if name == 'backdated' else 4
                self.assertEqual(len(steps), cycle)
                self.no_output_from(cycle)
                self.assertEqual(len(result['cycles']), cycle+1)
                self.assertIsNone(result['cycles'][cycle]['command_ns'])
                self.assert_all_stopped(result)

    def test_r2_cycle70_replay_faults_today_and_passes_paced(self):
        natural = lambda cycle: 11_569_000 if cycle == 6 else 10_175_000
        today, _, _ = self.run_paced(natural, offset=None)
        self.assertEqual(today['status'], 'ABORTED')
        self.assertIn('Command gap exceeded', today['primary_error']['message'])
        self.assertEqual(len(today['cycles']), 7)
        self.setUp()
        paced, steps, _ = self.run_paced(natural)
        self.assertEqual(paced['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', paced['primary_error'])
        intervals = [(b-a)*1e3 for a, b in zip(steps[1:], steps[2:])]
        self.assertLess(max(intervals), 20.4)
        self.assertGreater(paced['cycles'][6]['natural_gate_ns']-paced['cycles'][6]['release_ns'], 11_569_000)

    def test_r1_cycle26_host_stall_replay_still_faults_on_the_sample_gap(self):
        self.mock = StallMock
        self.controls['hold_at'] = {5: 2_794_000}
        result, _, _ = self.run_paced(lambda cycle: 10_175_000)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('Sample gap exceeded', result['primary_error']['message'])
        self.no_output_from(5)
        self.assert_all_stopped(result)

    def test_static_offset_range(self):
        self.assertEqual(P.COMMAND_PHASE_MAX_US+sum(P.COMMAND_PHASE_BUDGET_US.values()), 20_000)
        for value in (P.COMMAND_PHASE_MIN_US, K_US, P.COMMAND_PHASE_MAX_US):
            self.assertEqual(P.pacing_options(P.selected_pacing({'command_phase_offset_us': value})),
                             {'command_phase_offset_us': value})
        for value in (P.COMMAND_PHASE_MIN_US-1, P.COMMAND_PHASE_MAX_US+1, 11250., True, '11250', 0, -1):
            with self.subTest(value=value), self.assertRaises(ValueError):
                P.selected_pacing({'command_phase_offset_us': value})
            with self.subTest(runner=value), self.assertRaises(ValueError):
                value_admitted = admitted()
                value_admitted['pacing'] = dict(P.PACING, command_phase_offset_us=value)
                R.plan(value_admitted)

    def test_static_offset_with_a_prearmed_lead_keeps_the_next_prearm_window(self):
        tail = sum(P.COMMAND_PHASE_BUDGET_US.values())+sum(P.PREARMED_TAIL_BUDGET_US.values())
        self.assertEqual((tail, P.PREARM_WORK_US), (9290, 1250))
        for lead in (P.PREARMED_LEAD_US[0], 1000, P.PREARM_WORK_US, LEAD_US, 1710):
            top = P.command_phase_max_with_lead_us(lead)
            self.assertEqual(top+tail+max(lead, P.PREARM_WORK_US), 20_000, lead)
            pair = {'command_phase_offset_us': top, 'prearmed_hold_lead_us': lead}
            self.assertEqual(P.pacing_options(P.selected_pacing(pair)), pair)
            for value in (top+1, K_US, P.COMMAND_PHASE_MAX_US):
                with self.subTest(lead=lead, offset=value), self.assertRaisesRegex(ValueError, 'pre-armed hold lead'):
                    P.selected_pacing({'command_phase_offset_us': value, 'prearmed_hold_lead_us': lead})
        self.assertEqual(P.command_phase_max_with_lead_us(LEAD_US), 9210)
        for lead in (1711, P.PREARMED_LEAD_US[1]):  # No admissible K is left at these leads.
            with self.subTest(lead=lead), self.assertRaises(ValueError):
                P.selected_pacing({'command_phase_offset_us': P.COMMAND_PHASE_MIN_US, 'prearmed_hold_lead_us': lead})
        # Each option alone keeps its full reviewed range.
        for alone in ({'command_phase_offset_us': P.COMMAND_PHASE_MAX_US}, {'prearmed_hold_lead_us': 2000}):
            self.assertEqual(P.pacing_options(P.selected_pacing(alone)), alone)
        # The reviewed F1-only K = 11250 with L = 1500 (finding's profile) fails the profile and the runner.
        rejected = dict(P.PACING, command_phase_offset_us=K_US, prearmed_hold_lead_us=LEAD_US)
        with self.assertRaisesRegex(ValueError, 'K <= 9210 us at L = 1500 us'):
            P.pacing_options(rejected)
        value = admitted()
        value['pacing'] = rejected
        with self.assertRaisesRegex(ValueError, 'pre-armed hold lead'):
            R.plan(value)
        plan = R.plan(optioned(command_phase_offset_us=K_WITH_LEAD_US, prearmed_hold_lead_us=LEAD_US))
        self.assertEqual(plan['command_phase']['prearmed_tail_budget_us'],
                         {'post_reply_admission': 730, 'timing_evidence': 100, 'next_prearm_window': LEAD_US})
        self.assertNotIn('prearmed_tail_budget_us', R.plan(optioned(command_phase_offset_us=K_US))['command_phase'])


# ----------------------------------------------------------------------------- F2b decode-once
class DecodeOnceTests(OptionHarness, unittest.TestCase):
    mock = DecodeOnceMock

    def test_takeouts_and_final_gate_reuse_the_publication(self):
        verified = []
        original = Batch.verify
        def verify(batch):
            verified.append(batch.label)
            return original(batch)
        with patch.object(Batch, 'verify', verify):
            result = self.run_case('learned_boxed', value=optioned('learned_boxed', decode_once=True))
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', result['primary_error'])
        cycles = len(result['cycles'])
        self.assertNotIn('hold', verified)  # No hold re-decode in any cycle.
        for port in PORTS:
            self.assertEqual(len(self.adapters[port].fast), 2*cycles)
            self.assertEqual(self.adapters[port].full.count('voltage'), 2*cycles)
        self.assertTrue(result['decode_once_selected'])
        self.assertEqual(result['takeout_check'], 'raw_byte_images_compared_publication_rows_reused')
        self.assertIn('raw_byte_images_compared', result['final_gate'])
        self.assertIs(result['pacing']['decode_once'], True)

    def test_raw_image_changed_after_publication_fails_the_final_gate(self):
        def tamper(calls, targets):
            if calls == 4:
                record = self.adapters['port1'].last_hold.records[0]
                record.rx[9] ^= 1
        result = self.run_case('zero_gain_timing', observer=Observer(tamper),
                               value=optioned(decode_once=True))
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('changed after publication', result['primary_error']['message'])
        self.no_output_from(3)
        self.assert_all_stopped(result)

    def test_published_rows_are_read_only(self):
        result = self.run_case('zero_gain_timing', value=optioned(decode_once=True))
        rows = result['cycles'][0]['hold']['port0'].rows
        self.assertIs(type(rows), MappingProxyType)
        with self.assertRaises(TypeError):
            rows[(7, 'feedback')] = None


# ----------------------------------------------------------------------------- F3 pre-armed hold
class PrearmedHoldTests(OptionHarness, unittest.TestCase):
    mock = PrearmedMock
    LEAD = 1000

    def setUp(self):
        super().setUp()
        self.prearm = True

    def run_armed(self, mode='zero_gain_timing', **kw):
        return self.run_case(mode, value=optioned(mode, prearmed_hold_lead_us=self.LEAD), **kw)

    def test_wake_gates_and_submit_before_release_writes_at_or_after_it(self):
        checked, original = [], R.checked_voltage_cache
        def voltage(cache, profile, now_ns):
            checked.append(now_ns)
            return original(cache, profile, now_ns)
        with patch.object(R, 'checked_voltage_cache', voltage):
            result = self.run_armed()
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        self.assertEqual(result['full_current_check_points'][0], 'before_release_before_prearmed_submit')
        self.assertEqual(result['prearmed_hold']['lead_us'], self.LEAD)
        rows = result['cycles']
        self.assertEqual(result['epoch_ns'], rows[0]['release_ns'])
        for row in rows:
            release, index = row['release_ns'], row['index']
            self.assertEqual(self.waits[2*index:2*index+2], [release-self.LEAD*1000, release])
            self.assertTrue(release-self.LEAD*1000 <= row['prearm_wake_ns'] < release)
            boundary1 = [at for cycle, at in self.current_times if cycle == index+1][0]
            self.assertLess(boundary1, release)
            armed = [(at, port) for port, nb, at in self.armed if nb == release]
            self.assertEqual(sorted(port for _, port in armed), sorted(PORTS))
            self.assertTrue(all(at < release for at, _ in armed))
            self.assertGreaterEqual(row['hold_first_write_ns'], release)
            self.assertGreaterEqual(row['first_ns'], release)
            self.assertGreaterEqual(row['imu']['read_started_monotonic_ns'], row['begin_ns'])
            self.assertTrue(release <= row['begin_ns'] <= row['hold_first_write_ns']+50_000)
            self.assertLessEqual(row['hard_end_ns'], release+20_000_000)
            self.assertEqual(row['request_count'], 28)
        self.assertTrue(all(nb is not None for _, nb, _ in self.armed))
        # The voltage-cache gate before each pre-armed hold is evaluated at that release.
        self.assertTrue({row['release_ns'] for row in rows} <= set(checked))
        self.assertEqual(result['voltage_guard']['checks_before_type1'], 25+2*len(rows))
        self.assert_all_stopped(result)

    def test_main_waking_after_the_armed_owners_wrote_keeps_the_post_reply_rules_causal(self):
        # Cycle 3: the owners write at release+2 us; main's own release wait returns at +30 us.
        original = OptionHarness.release_wait
        def late(when):
            if len(self.waits) % 2 == 0 or len(self.releases) != 3:
                return original(self, when)
            self.waits.append(when)
            self.releases.append(when)
            limit = time.monotonic()+2
            while sum(1 for _, at, _ in list(self.armed) if at == when) < 4 and time.monotonic() < limit:
                time.sleep(.0001)
            before = {port: adapter.holds for port, adapter in self.adapters.items()}
            self.clock.advance_to(when+2_000)
            while any(self.adapters[p].holds == before[p] for p in before) and time.monotonic() < limit:
                time.sleep(.0001)
            return self.clock.advance_to(when+30_000)
        admitted_begins, original_admit = [], R.PostReplyDeadlineBudget.admit
        def admit(budget, **kwargs):
            admitted_begins.append(kwargs['begin_ns'])
            return original_admit(budget, **kwargs)
        with patch.object(self, 'release_wait', late), patch.object(R.PostReplyDeadlineBudget, 'admit', admit):
            result = self.run_armed()
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        self.assertEqual(admitted_begins, [row['release_ns'] for row in result['cycles']])
        row = result['cycles'][3]
        release = row['release_ns']
        self.assertEqual(row['begin_ns'], release+30_000)
        self.assertEqual(row['first_ns'], row['hold_first_write_ns'])
        self.assertTrue(release <= row['first_ns'] < row['begin_ns'])
        self.assertTrue(row['decision']['accepted'])
        lateness = int(R.POST_REPLY_V1['max_lateness_ms']*1e6)
        for row in result['cycles']:
            self.assertEqual(row['join_deadline_ns'], min(row['first_ns']+20_000_000,
                                                          row['release_ns']+20_000_000+lateness))

    def test_cancel_between_prearm_and_release_writes_nothing(self):
        def hook(cycle, release):
            if cycle == 3:
                raise RuntimeError('Cancelled before active release')
        self.controls['release_hook'] = hook
        result = self.run_armed()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(sorted(self.refused), sorted(PORTS))
        for port in PORTS:
            self.assertEqual(self.adapters[port].holds, 3)
        self.no_output_from(3)
        self.assert_all_stopped(result)

    def test_late_wake_or_gates_at_the_release_send_no_hold(self):
        self.release_extra = {3: self.LEAD*1000}  # The pre-arm wake of cycle 2 returns at its release.
        def late(when):
            extra = self.release_extra.get(self.cycle_release, 0) if len(self.waits) % 2 == 0 else 0
            self.waits.append(when)
            if len(self.waits) % 2:
                self.cycle_release += 1
                return self.clock.advance_to(when+extra)
            self.releases.append(when)
            limit = time.monotonic()+2
            while sum(1 for _, at, _ in list(self.armed) if at == when) < 4 and time.monotonic() < limit:
                time.sleep(.0001)
            return self.clock.advance_to(when)
        with patch.object(self, 'release_wait', late):
            result = self.run_armed()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('Pre-armed wake missed', result['primary_error']['message'])
        self.assertEqual([adapter.holds for adapter in self.adapters.values()], [3]*4)
        self.no_output_from(3)
        self.setUp()
        original = self.current
        def slow():
            original()
            if self.cycle_release == 4 and len([c for c, _ in self.current_times if c == 4]) == 1:
                self.clock.advance_to(self.waits[-1]+self.LEAD*1000)
        with patch.object(self, 'current', slow):
            result = self.run_armed()
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('reached the release', result['primary_error']['message'])
        self.assertEqual([adapter.holds for adapter in self.adapters.values()], [3]*4)
        self.no_output_from(3)

    def test_hold_gates_are_evaluated_at_the_release_not_at_the_wake(self):
        # 19.5 ms of setup after the initial hold: the initial replies are 20.5 ms old at the
        # pre-armed cycle-0 release (reject) but only 19.5 ms at its wake; the default path passes.
        original = self.current
        def slow():
            original()
            if self.cycle_release == 0 and len(self.adapters) == 4 and \
                    all(len(a.outputs) == 1 for a in self.adapters.values()) and \
                    not self.controls.get('advanced'):
                self.controls['advanced'] = True
                self.clock.advance_to(self.clock.now+19_500_000)
        with patch.object(self, 'current', slow):
            result = self.run_armed()
        self.assertTrue(self.controls['advanced'])
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('stale feedback before hold', result['primary_error']['message'])
        self.assertEqual([adapter.holds for adapter in self.adapters.values()], [0]*4)
        self.assertEqual(self.armed, [])
        self.setUp()
        self.prearm, self.mock = False, MockType1
        original = self.current
        with patch.object(self, 'current', slow):
            default = self.run_case('zero_gain_timing')
        self.assertTrue(self.controls['advanced'])
        self.assertEqual(default['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', default['primary_error'])

    def run_cycle_end(self, end_ns, *, prearmed=True):
        """Cycle 3 ends at release+end_ns; Boundary 1 costs 160 us and each owner prepares 250 us (GIL-serialized)."""
        self.mock, self.prep_ns = PrepCostMock, 250_000
        original_admit, original_current = R.PostReplyDeadlineBudget.admit, self.current
        def admit(budget, **kwargs):
            if kwargs['index'] == 3:
                self.clock.advance_to(kwargs['begin_ns']+end_ns)
            return original_admit(budget, **kwargs)
        def current():
            original_current()
            self.clock.advance_to(self.clock.now+160_000)
        if not prearmed:
            self.prearm, self.mock = False, MockType1
        with patch.object(R.PostReplyDeadlineBudget, 'admit', admit), patch.object(self, 'current', current):
            if prearmed:
                return self.run_armed()
            return self.run_case('zero_gain_timing')

    def test_cycle_end_inside_the_combined_budget_leaves_the_prearm_window(self):
        self.LEAD = LEAD_US
        tail = sum(P.COMMAND_PHASE_BUDGET_US.values())+sum(P.PREARMED_TAIL_BUDGET_US.values())
        end_ns = (P.command_phase_max_with_lead_us(LEAD_US)+tail)*1000  # release+18.5 ms
        self.assertEqual(end_ns, 20_000_000-LEAD_US*1000)
        result = self.run_cycle_end(end_ns)
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        self.assertEqual(self.refused, [])
        rows = result['cycles']
        self.assertGreaterEqual(rows[4]['prearm_wake_ns']-rows[3]['release_ns'], end_ns)  # Woke after the late end.
        self.assertLess(rows[4]['prearm_wake_ns'], rows[4]['release_ns'])
        self.assertTrue(all(at < nb for _, nb, at in self.armed))

    def test_cycle_end_in_the_f1_only_tail_aborts_the_next_prearmed_cycle_before_any_hold(self):
        # K = 11250 with a +1.3 ms CH341 output tail and 0.73 ms post-reply admission ends
        # cycle 3 at release+18.9 ms, inside the F1-only budget (admitted before the
        # lead-aware check). Boundary 1 and the owner prep then cross the release.
        self.LEAD = LEAD_US
        end_ns = (K_US+470+5140+1300+730)*1000
        self.assertLess(end_ns, 20_000_000)
        result = self.run_cycle_end(end_ns)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('Pre-armed release must be ahead', result['primary_error']['message'])
        self.assertEqual(len(result['cycles']), 5)
        self.assertTrue(self.refused)
        self.assertEqual([adapter.holds for adapter in self.adapters.values()], [4]*4)
        self.no_output_from(4)
        self.assert_all_stopped(result)
        self.setUp()
        default = self.run_cycle_end(end_ns, prearmed=False)  # Same tail without F3: the next cycle starts late.
        self.assertEqual(default['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', default['primary_error'])

    def test_lead_range_and_missing_transport_selection(self):
        for value in (299, 2001, 1000., True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                P.selected_pacing({'prearmed_hold_lead_us': value})
        self.mock = MockType1
        result = self.run_armed()
        self.assertIn('selections must equal', result['primary_error']['message'])
        self.assertFalse(result['type1_sent'])


# ----------------------------------------------------------------------------- F0 evidence, F4 gc freeze
class EvidenceAndFreezeTests(OptionHarness, unittest.TestCase):
    def test_timing_evidence_rows_and_run_delta(self):
        reader = FakeEvidence()
        result = self.run_case('zero_gain_timing', timing_evidence=reader)
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', result['primary_error'])
        rows = result['cycles']
        self.assertEqual(reader.samples, len(rows)+1)
        self.assertEqual(set(reader.threads), {threading.get_native_id()})
        self.assertEqual(set(reader.bound), {'main', *PORTS, 'imu', 'host_watchdog'})
        self.assertEqual(reader.bound['port2'], result['worker_settings']['port2']['native_tid'])
        self.assertEqual(reader.bound['host_watchdog'], result['watchdog_settings']['native_tid'])
        for row in rows:
            self.assertEqual(set(row)-DEFAULT_ROW_KEYS, {'submit_done_ns', 'owner_entry_ns', 'evidence_cost_ns',
                                                         'thread_counter_delta'})
            self.assertTrue(row['begin_ns'] < row['submit_done_ns'] < row['acquired_ns'])
            self.assertEqual(set(row['owner_entry_ns']), set(PORTS))
            self.assertTrue(all(row['begin_ns'] < value < row['hold_last_reply_ns']
                                for value in row['owner_entry_ns'].values()))
            self.assertLess(row['evidence_cost_ns'], 100_000)
            self.assertEqual(row['thread_counter_delta']['main'], {'run_delay_ns': 10, 'nivcsw': 1, 'minflt': None})
        self.assertEqual(result['timing_evidence']['run'],
                         {'vmstat': {'thp_fault_alloc': 1, 'compact_stall': 0}, 'interrupts': {'7': [1, 0]}})
        self.assertIs(result['timing_evidence']['contract_input'], False)
        self.assertTrue(reader.closed)
        self.assertEqual(result['pacing'], P.PACING)

    def test_gc_freeze_after_warmup_and_unfrozen_at_restoration(self):
        seen = {}
        def hook(calls, targets):
            seen.setdefault('cycle', gc.get_freeze_count())
        setup = lambda o, **k: dict(k, reset_verified=True, freeze_count=gc.get_freeze_count())
        for inject in (None, ('port0', 4)):
            with self.subTest(inject=inject):
                self.setUp()
                seen.clear()
                if inject:
                    self.controls['hold_fail'] = inject
                result = self.run_case('zero_gain_timing', observer=Observer(hook), model_setup=setup,
                                       value=optioned(gc_freeze=True))
                self.assertEqual(result['status'], 'ABORTED' if inject else 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING')
                self.assertEqual(result['model_setup']['freeze_count'], 0)
                self.assertGreater(seen['cycle'], 0)
                self.assertGreater(result['gc_freeze']['frozen_before_first_release'], 0)
                self.assertIs(result['gc_freeze']['unfrozen_at_restoration'], True)
                self.assertEqual(gc.get_freeze_count(), 0)
                self.assertTrue(result['gc_freeze_selected'])

    def run_case(self, mode='learned_boxed', observer=None, value=None, model_setup=None, **kw):
        if model_setup is None:
            return super().run_case(mode, observer, value, **kw)
        kw.setdefault('clock', self.clock)
        return R.run(value or admitted(mode), factory=self.factory, imu_read=self.imu,
            observer=observer or Observer(), check_current=self.current, check_cancelled=self.check,
            cancel_io=self.cancel_io, model_setup=model_setup, worker_scope=self.worker_scope,
            main_scope=self.main_scope, release_wait=self.release_wait,
            announce=lambda: self.events.append(('announce',)), stop_requested=self.stop_requested,
            backend_usage={'kind': 'injected_file_only_mock'}, execute=True, **kw)


class TimingEvidenceReaderTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.root)

    def proc(self, tid, schedstat, minflt, nvcsw):
        task = self.root/'self'/'task'/str(tid)
        task.mkdir(parents=True, exist_ok=True)
        (task/'schedstat').write_text('%d %d %d\n' % (5_000_000, schedstat, 40))
        (task/'stat').write_text('%d (type1 (worker)) S 1 2 3 0 -1 4194560 %d 0 7 0 5 6\n' % (tid, minflt))
        (task/'status').write_text('Name:\tx\nvoluntary_ctxt_switches:\t%d\nnonvoluntary_ctxt_switches:\t3\n' % nvcsw)

    def test_fake_proc_counters_deltas_and_degradation(self):
        self.proc(101, 1000, 50, 9)
        (self.root/'vmstat').write_text('nr_free_pages 1\nthp_fault_alloc 4\ncompact_stall 2\npgfault 9\n')
        (self.root/'interrupts').write_text('       CPU0  CPU1\n 7:  10  20  GICv3 usb\nIPI0:  5  5  Rescheduling\nERR: 0\n')
        reader = E.TimingEvidence(self.root, rusage_thread=None)
        binding = reader.bind({'port0': 101, 'missing': 999})
        self.assertEqual(binding['threads']['port0']['proc_counters'], {'schedstat': True, 'stat': True, 'status': True})
        self.assertEqual(binding['threads']['missing']['proc_counters'], {'schedstat': False, 'stat': False, 'status': False})
        first = reader.sample()
        self.assertEqual(first['port0'], {'run_delay_ns': 1000, 'timeslices': 40, 'minflt': 50, 'majflt': 7,
                                          'nvcsw': 9, 'nivcsw': 3})
        self.assertTrue(all(value is None for value in first['missing'].values()))
        before = reader.run_counters()
        self.assertEqual(before['vmstat'], {'thp_fault_alloc': 4, 'compact_stall': 2})
        self.assertEqual(before['interrupts'], {'7': [10, 20], 'IPI0': [5, 5], 'ERR': [0]})
        self.proc(101, 1600, 53, 12)
        (self.root/'interrupts').write_text('       CPU0  CPU1\n 7:  12  20  GICv3 usb\nIPI0:  5  5  Rescheduling\nERR: 0\n')
        (self.root/'vmstat').write_text('thp_fault_alloc 6\ncompact_stall 2\n')
        change = reader.delta(first, reader.sample())
        self.assertEqual(change['port0'], {'run_delay_ns': 600, 'timeslices': 0, 'minflt': 3, 'majflt': 0,
                                           'nvcsw': 3, 'nivcsw': 0})
        self.assertEqual(reader.run_delta(before, reader.run_counters()),
                         {'vmstat': {'thp_fault_alloc': 2, 'compact_stall': 0}, 'interrupts': {'7': [2, 0]}})
        reader.close()
        self.assertTrue(all(fd is None for fds in reader.fds.values() for fd in fds.values()))
        absent = E.TimingEvidence(self.root/'absent')
        absent.bind({'main': threading.get_native_id()})
        self.assertEqual(absent.run_counters(), {'vmstat': None, 'interrupts': None})
        begin = time.perf_counter_ns()
        value = absent.sample()
        self.assertLess(time.perf_counter_ns()-begin, 1_000_000)
        self.assertEqual(set(value['main']), set(E.THREAD_FIELDS))
        absent.close()

    def test_run_counters_read_large_proc_files_whole(self):
        # Jetson-sized /proc/interrupts (>8 KB) with the IPI block last, read back one page per read().
        header = '      ' + ''.join('     CPU%d' % k for k in range(6)) + '\n'
        lines = [' %3d:' % irq + ''.join(' %9d' % (irq+k) for k in range(6)) + '  GICv3 %3d Level  dev%03d\n' % (irq, irq)
                 for irq in range(400)]
        ipis = ['IPI%d:' % k + ''.join(' %9d' % (k*10) for _ in range(6)) + '  Rescheduling interrupts\n' for k in range(7)]
        (self.root/'interrupts').write_text(header+''.join(lines)+''.join(ipis)+'Err:          0\n')
        (self.root/'vmstat').write_text(''.join('nr_counter_%d %d\n' % (k, k) for k in range(600)) +
                                        'thp_fault_alloc 7\ncompact_stall 3\n')
        self.assertGreater((self.root/'interrupts').stat().st_size, 8192*4)
        self.assertGreater((self.root/'vmstat').stat().st_size, 8192)
        reader = E.TimingEvidence(self.root, rusage_thread=None)
        read = os.read
        with patch.object(E.os, 'read', lambda fd, n: read(fd, min(n, 4096))):
            value = reader.run_counters()
        self.assertEqual(len(value['interrupts']), 400+7+1)
        self.assertEqual(value['interrupts']['399'], [399+k for k in range(6)])
        self.assertEqual(value['interrupts']['IPI6'], [60]*6)
        self.assertEqual(value['vmstat'], {'thp_fault_alloc': 7, 'compact_stall': 3})
        with patch.object(E, 'RUN_FILE_LIMIT', 8192):
            self.assertEqual(reader.run_counters(), {'vmstat': None, 'interrupts': None})


# ----------------------------------------------------------------------------- profile and chaining
class ProfileOptionTests(TP.Base):
    OPTIONS = {'command_phase_offset_us': K_WITH_LEAD_US, 'decode_once': True, 'prearmed_hold_lead_us': LEAD_US,
               'gc_freeze': True}

    def test_default_contract_unchanged_and_options_recorded_only_when_selected(self):
        default = self.world.contract()
        self.assertEqual(default['pacing'], P.PACING)
        self.assertEqual(P.canonical_sha256(default['pacing']), PACING_SHA256)
        explicit = P.build_contract(copy.deepcopy(self.world.plan), self.world.lineage_refs(),
            copy.deepcopy(self.world.geometry_doc), self.world.geometry,
            options={'command_phase_offset_us': None, 'decode_once': False, 'prearmed_hold_lead_us': None,
                     'gc_freeze': False})
        self.assertEqual(P.contract_sha256(explicit), P.contract_sha256(default))
        selected = P.build_contract(copy.deepcopy(self.world.plan), self.world.lineage_refs(),
            copy.deepcopy(self.world.geometry_doc), self.world.geometry, options=dict(self.OPTIONS))
        self.assertEqual(selected['pacing'], {**P.PACING, **self.OPTIONS})
        self.assertNotEqual(P.contract_sha256(selected), P.contract_sha256(default))
        self.assertEqual(P.validate_contract(json.loads(json.dumps(selected))), selected)
        self.assertEqual({key: value for key, value in selected.items() if key != 'pacing'},
                         {key: value for key, value in default.items() if key != 'pacing'})

    def test_option_tampering_rejected(self):
        cases = {'unknown': {'command_phase_offset': 11250}, 'false flag': {'decode_once': False},
                 'int flag': {'gc_freeze': 1}, 'offset type': {'command_phase_offset_us': 11250.},
                 'offset low': {'command_phase_offset_us': 8999}, 'offset high': {'command_phase_offset_us': 11541},
                 'lead low': {'prearmed_hold_lead_us': 299}, 'lead high': {'prearmed_hold_lead_us': 2001},
                 'base changed': {'request_gap_us': 890}}
        for name, change in cases.items():
            contract = copy.deepcopy(self.base_contract)
            contract['pacing'].update(change)
            with self.subTest(name), self.assertRaises(ValueError):
                P.validate_contract(contract)
        with self.assertRaisesRegex(ValueError, 'Unknown pacing option'):
            P.selected_pacing({'nice': -5})

    def test_chained_steps_require_identical_options(self):
        selected = P.build_contract(copy.deepcopy(self.world.plan), self.world.lineage_refs(),
            copy.deepcopy(self.world.geometry_doc), self.world.geometry, options={'decode_once': True})
        default = self.base_contract
        for contract, report_contract in ((default, selected), (selected, default)):
            report = TP.type1_report(report_contract, 'zero_gain_timing', 2, 20_000_000_000, 30_000_000_000)
            with self.assertRaisesRegex(ValueError, 'opt-in pacing options differ'):
                P.validate_type1_report(report, contract=contract, contract_digest=P.contract_sha256(contract),
                                        mode='zero_gain_timing')
        report = TP.type1_report(selected, 'zero_gain_timing', 2, 20_000_000_000, 30_000_000_000)
        summary = P.validate_type1_report(report, contract=selected, contract_digest=P.contract_sha256(selected),
                                          mode='zero_gain_timing')
        evidence = {'current_capture': self.world.capture_evidence(), 'stop_proxy_diagnostic': None,
                    'predecessor': {'report': {'path': str(self.root/'pred.json'), 'sha256': 'f'*64}, **summary}}
        profile = P.assemble_profile(copy.deepcopy(selected), 'learned_boxed', 2, evidence, prepared_at='now')
        admitted = P.admit(profile, self.conditions(profile))
        self.assertEqual(admitted.report_binding()['pacing'], selected['pacing'])
        with self.assertRaisesRegex(ValueError, 'required complete same-contract run'):
            P.assemble_profile(copy.deepcopy(default), 'learned_boxed', 2, evidence, prepared_at='now')

    def test_prepare_flags_select_options_and_default_omits_them(self):
        parsed = P.parser().parse_args(['prepare', '--mode', 'zero_gain_timing', '--duration', '2', '--power-epoch',
            'e', *sum((['--'+n, 'x', '--'+n+'-sha256', 'y'] for n in ('topology', 'events', 'calibration',
            'model-profile', 'source-binding', 'mount', 'bias', 'axis-geometry-profile', 'current-topology',
            'current-events')), []), '--output', '/x', '--command-phase-offset-us', str(K_WITH_LEAD_US),
            '--decode-once', '--prearmed-hold-lead-us', str(LEAD_US), '--gc-freeze'])
        self.assertEqual(P.selected_pacing(P.cli_options(parsed)), {**P.PACING, **self.OPTIONS})
        report = TP.type1_report(self.base_contract, 'zero_gain_timing', 2, 20_000_000_000, 30_000_000_000)
        path = TP.write(self.output('zero.json'), report)
        args = self.world.args('learned_boxed', 2, self.output('p.json'), predecessor_report=path['path'],
                               predecessor_report_sha256=path['sha256'])
        result = P.prepare(args)
        self.assertNotIn('pacing_options', result)
        args.decode_once = True
        with self.assertRaisesRegex(ValueError, 'opt-in pacing options differ'):
            P.prepare(args)
        selected = P.build_contract(copy.deepcopy(self.world.plan), self.world.lineage_refs(),
            copy.deepcopy(self.world.geometry_doc), self.world.geometry, options={'decode_once': True})
        path = TP.write(self.output('zero-once.json'),
                        TP.type1_report(selected, 'zero_gain_timing', 2, 20_000_000_000, 30_000_000_000))
        args = self.world.args('learned_boxed', 2, self.output('p2.json'), predecessor_report=path['path'],
                               predecessor_report_sha256=path['sha256'], decode_once=True, prepare=True)
        result = P.prepare(args)
        self.assertEqual(result['pacing_options'], {'decode_once': True})
        profile = P.read_json(result['output'], result['profile']['sha256'])
        self.assertIs(profile['contract']['pacing']['decode_once'], True)


# ----------------------------------------------------------------------------- foreground wiring
class ArmedMotors(TF.Motors):
    def __init__(self, world, clock):
        super().__init__(world, clock)
        self.armed = []

    def sda_subset_exchange_at(self, handle, mask, raw, count, send_only, not_before, deadline, records, stats,
                               error, size):
        with self.lock:
            self.armed.append((handle, not_before, self.clock.now))
        limit = time.monotonic()+2
        while self.clock.now < not_before:
            if self.cancelled(handle) or time.monotonic() > limit:
                with self.lock:
                    self.refused_after_cancel.append((handle, mask, None))
                error.value = b'Synthetic native cancelled before not_before'
                return -1
            time.sleep(.0001)
        return self.sda_subset_exchange(handle, mask, raw, count, send_only, deadline, records, stats, error, size)


class OptionEnvironment(TF.Environment):
    prearm = False

    def release_wait(self, library, cancel_fd, observer):
        self.cancel_fd, calls = cancel_fd, []
        def wait(when):
            calls.append(when)
            if self.prearm and not len(calls) % 2:
                limit = time.monotonic()+2
                while sum(1 for _, at, _ in list(self.motors.armed) if at == when) < 4 and time.monotonic() < limit:
                    time.sleep(.0001)
            return self.clock.advance_to(when)
        return wait

    def command_wait(self, library, cancel_fd):
        self.command_waits = []
        def wait(target):
            self.command_waits.append(target)
            return self.clock.advance_to(target)
        return wait

    def timing_evidence(self):
        self.reader = E.TimingEvidence()
        return self.reader


class ForegroundOptionTests(TF.Base):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        world = cls.world
        cls.lineage = {'topology': world.lineage, 'events': world.lineage_events,
                       **{name: world.refs[name] for name in ('calibration', 'model_profile', 'source_binding',
                                                             'mount', 'bias')}}
        # A receipt that records the optional pre-armed export.
        source = Path(world.type1_library['path']).parent
        armed = world.root/'t1lib-armed'
        armed.mkdir()
        for name in ('ordinary_transport.cpp', 'subset_stop.cpp', 'transport.cpp', 'libdog_four_bus_type1_transport.so'):
            shutil.copy2(source/name, armed/name)
        record = json.loads((source/'build-record.json').read_text())
        record['four_bus_subset_active']['exchange_at_abi'] = 1
        cls.armed_library = {'path': str(armed/'libdog_four_bus_type1_transport.so'),
                             'sha256': world.type1_library['sha256']}
        cls.armed_build = TP.write(armed/'build-record.json', record)

    def admission(self, name, mode, capture, options, **kw):
        contract = P.build_contract(copy.deepcopy(self.world.plan), self.lineage, copy.deepcopy(self.world.geometry_doc),
                                    self.world.geometry, options=options)
        with patch.object(self.world, 'contract', contract):
            return self.world.admission(name, mode, 2, capture, **kw), contract

    def run_options(self, admission, capture, mode, *, extra=(), armed=False, mutate=None):
        world = self.world
        with patch.object(world, 'type1_library', self.armed_library if armed else world.type1_library), \
                patch.object(world, 'type1_build', self.armed_build if armed else world.type1_build):
            argv = world.argv(*admission, capture, mode, 2, self.output(), *extra)
        args = F.parser().parse_args(argv+['--execute'])
        with TF.file_only_prepare():
            prepared = F.prepare(args)
        clock = TF.VirtualClock()
        motors = ArmedMotors(world, clock)
        if mutate is not None:
            mutate(motors)
        observer = TF.Observer(self.targets())
        environment = OptionEnvironment(motors, clock, observer)
        environment.prearm = armed
        seal = ARMED if armed else TF.SEAL
        MockSession.created = []
        with patch.object(T.active, 'ActiveSession', MockSession), patch.object(T, 'verify_library', lambda lib: seal), \
                patch.object(T.active, 'verified_active_session_creation', lambda session: ('c', 'b')), \
                patch.object(T, 'time', SimpleNamespace(monotonic_ns=clock)):
            report = F.execute(args, prepared, environment)
        self.assertEqual(motors.forbidden, [])
        written = prepared['output']/'report.json'
        self.assertEqual(json.loads(written.read_text()), json.loads(json.dumps(report)))
        return SimpleNamespace(report=report, motors=motors, environment=environment, path=written,
                               prepared=prepared)

    def test_default_foreground_plan_and_report_carry_no_option_fields(self):
        default = self.run_options(self.zero, self.current, 'zero_gain_timing')
        self.assertEqual(default.report['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', default.report['errors'])
        for key in ('pacing_options', 'timing_evidence_selected'):
            self.assertNotIn(key, default.report)
        self.assertTrue(all(set(row) == DEFAULT_ROW_KEYS for row in default.report['measurement']['cycles']))
        self.assertEqual(default.motors.armed, [])
        argv = self.world.argv(*self.zero, self.current, 'zero_gain_timing', 2, self.output())
        with TF.file_only_prepare():
            prepared = F.prepare(F.parser().parse_args(argv))
        for key in ('pacing_options', 'timing_evidence'):
            self.assertNotIn(key, prepared['plan'])
        self.assertNotIn('exchange_at_abi', prepared['plan']['type1_library_plan'])
        self.assertEqual(prepared['plan']['pacing'], P.PACING)
        self.assertEqual(prepared['plan']['runner_plan']['pacing'], P.PACING)

    def test_all_options_chain_zero_gain_then_learned_with_genuine_transport(self):
        options = {'command_phase_offset_us': K_WITH_LEAD_US, 'decode_once': True, 'prearmed_hold_lead_us': LEAD_US,
                   'gc_freeze': True}
        admission, contract = self.admission('opt-zero', 'zero_gain_timing', self.current, options)
        zero = self.run_options(admission, self.current, 'zero_gain_timing', extra=('--timing-evidence',), armed=True)
        report = zero.report
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', report['errors'])
        self.assertEqual(report['pacing_options'], options)
        self.assertTrue(report['timing_evidence_selected'])
        measured = report['measurement']
        self.assertTrue(measured['decode_once_selected'] and measured['gc_freeze_selected'])
        self.assertEqual(measured['full_current_check_points'][0], 'before_release_before_prearmed_submit')
        self.assertTrue(measured['gc_freeze']['unfrozen_at_restoration'])
        self.assertIn('binding', measured['timing_evidence'])
        rows = measured['cycles']
        self.assertTrue(all(row['hold_first_write_ns'] >= row['release_ns'] for row in rows))
        self.assertTrue(all(row['command_ns'] >= row['release_ns']+K_WITH_LEAD_US*1000 for row in rows))
        self.assertEqual(measured['command_phase']['prearmed_tail_budget_us'],
                         {'post_reply_admission': 730, 'timing_evidence': 100, 'next_prearm_window': LEAD_US})
        self.assertEqual(len(zero.environment.command_waits), len(rows))
        self.assertEqual(len(zero.motors.armed), 4*len(rows))
        self.assertTrue(all(at < nb for _, nb, at in zero.motors.armed))
        self.assertTrue(all(row['thread_counter_delta'] is not None for row in rows))
        self.assertTrue(all(session.lib is zero.motors for session in MockSession.created))
        summary = P.validate_type1_report(json.loads(zero.path.read_text()), contract=contract,
                                          contract_digest=P.contract_sha256(contract), mode='zero_gain_timing',
                                          duration_s=2)
        with self.assertRaisesRegex(ValueError, 'opt-in pacing options differ|contract/pacing'):
            P.validate_type1_report(json.loads(zero.path.read_text()), contract=self.world.contract,
                                    contract_digest=self.world.digest, mode='zero_gain_timing')
        later = self.world.capture('opt-after-zero', summary['terminal_stop_finished_monotonic_ns']+1_000_000_000)
        predecessor = {'report': {'path': str(zero.path), 'sha256': TF.sha(zero.path)}, **summary}
        learned_admission, _ = self.admission('opt-learned', 'learned_boxed', later, options, predecessor=predecessor)
        learned = self.run_options(learned_admission, later, 'learned_boxed', armed=True)
        self.assertEqual(learned.report['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', learned.report['errors'])
        self.assertNotIn('timing_evidence_selected', learned.report)
        P.validate_type1_report(json.loads(learned.path.read_text()), contract=contract,
                                contract_digest=P.contract_sha256(contract), mode='learned_boxed', duration_s=2)

    def test_prearmed_contract_rejects_a_receipt_without_exchange_at(self):
        admission, _ = self.admission('opt-armed', 'zero_gain_timing', self.current, {'prearmed_hold_lead_us': 1000})
        argv = self.world.argv(*admission, self.current, 'zero_gain_timing', 2, self.output())
        with TF.file_only_prepare(), self.assertRaisesRegex(ValueError, 'exchange_at_abi'):
            F.prepare(F.parser().parse_args(argv))
        with patch.object(self.world, 'type1_library', self.armed_library), \
                patch.object(self.world, 'type1_build', self.armed_build):
            argv = self.world.argv(*admission, self.current, 'zero_gain_timing', 2, self.output())
        with TF.file_only_prepare():
            self.assertEqual(F.prepare(F.parser().parse_args(argv))['plan']['type1_library_plan']['exchange_at_abi'], 1)

    def test_prearmed_cancel_before_release_writes_nothing_and_stops_all(self):
        admission, _ = self.admission('opt-cancel', 'zero_gain_timing', self.current, {'prearmed_hold_lead_us': 1500})
        def mutate(motors):
            original = motors.sda_subset_exchange_at
            def armed(handle, *args):
                releases = {row[1] for row in motors.armed}
                if args[4] not in releases and len(releases) == 4:  # First owner arming cycle 4.
                    os.kill(os.getpid(), signal.SIGTERM)
                    limit = time.monotonic()+2
                    while not motors.cancelled(handle) and time.monotonic() < limit:
                        time.sleep(.0001)
                return original(handle, *args)
            motors.sda_subset_exchange_at = armed
        result = self.run_options(admission, self.current, 'zero_gain_timing', armed=True, mutate=mutate)
        report = result.report
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cancel_requests'][0]['source'], 'signal_SIGTERM')
        self.assertTrue(report['terminal_stop']['stop_confirmed'])
        self.assertTrue(any(mask is None for _, _, mask in result.motors.refused_after_cancel))
        # Zero-gain handshake + initial hold + (hold + output) for cycles 0..3; nothing in cycle 4.
        self.assertEqual({len(wires) for wires in result.motors.type1.values()}, {10})
        self.assert_all_ports_stopped(result)


if __name__ == '__main__':
    unittest.main()
