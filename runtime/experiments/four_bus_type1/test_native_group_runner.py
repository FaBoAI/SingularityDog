"""File-only control-loop integration of the opt-in native group API.

Synthetic records exercise the original motion envelope and all final gates.
These are functional tests, not timing measurements or motor qualification.
"""
import json
import unittest

from singularitydog_hw import can_readonly as codec
from experiments.four_bus_diagnostic.transport_adapter import PORTS
from . import type1_profile as P, type1_runner as R
from .test_type1_runner import Harness, type2, type17
from .test_type1_30hz import Mock30, admitted30


def admitted_native(mode='learned_boxed'):
    value = admitted30(mode)
    value['pacing'] = P.selected_pacing({'native_group_cycle': True})
    return value


class FileOnlyGroup:
    """Single-thread deterministic stand-in for separately tested native owners."""
    def __init__(self, adapters, harness):
        self.adapters, self.h = adapters, harness
        self.ticket, self.closed, self.journal, self.pending = 0, False, [], None
        self.settings = [dict(native_tid=i+100, cpu_mask=1 << i, configured=1,
                              status=0, timer_slack_ns=1000, synthetic=True) for i in range(4)]
        self.close_report = None
        harness.sequence.append(('native-ready',))
        harness.clock.advance_to(harness.clock()+harness.controls.get('group_setup_ns', 0))

    def initial_hold(self, wires, deadline_ns, check):
        check()
        self.h.sequence.append(('native-initial-hold',))
        return self._output(wires, deadline_ns, deadline_ns)

    def start_hold(self, wires, voltage_ids, deadline_ns, check):
        check()
        self.ticket += 1
        self.h.sequence.append(('native-start', self.ticket))
        self.pending = (wires, voltage_ids, deadline_ns)
        if self.h.controls.get('group_hold_failure'):
            raise TimeoutError('Synthetic native partial hold failure')
        return self.ticket

    def wait_hold(self, ticket):
        if ticket != self.ticket:
            raise ValueError('Current native ticket required')
        wires, _, deadline_ns = self.pending
        batches = {}
        for port in PORTS:
            adapter = self.adapters[port]
            if tuple(wires[port]) != adapter.last_output:
                raise ValueError('Last exact validated wires required')
            batches[port] = adapter._exchange('hold', [(wire, type2(mid, adapter.raw(mid), mode=2))
                for wire, mid in zip(wires[port], adapter.group.ids)], deadline_ns)
        return batches

    def wait_voltage(self, ticket):
        if ticket != self.ticket:
            raise ValueError('Current native ticket required')
        _, ids, deadline_ns = self.pending
        return {port: self.adapters[port]._exchange('voltage', [(
            codec.read_request(ids[port], 'voltage'), type17(ids[port], 'voltage', 40.))], deadline_ns)
            for port in PORTS}

    def _output(self, wires, reply_deadline_ns, write_deadline_ns):
        self.journal.append({'kind': 'synthetic-group-output', 'ticket': self.ticket})
        if self.h.controls.get('group_late_write') and self.ticket:
            self.h.clock.advance_to(write_deadline_ns+1)
        if self.h.clock() >= write_deadline_ns:
            raise TimeoutError('Synthetic separate native host-write cutoff')
        batches = {}
        for port in PORTS:
            adapter = self.adapters[port]
            batches[port] = adapter._exchange('output', [(wire, type2(mid, adapter.raw(mid), mode=2))
                for wire, mid in zip(wires[port], adapter.group.ids)], reply_deadline_ns)
            if any(record.finish_ns > write_deadline_ns for record in batches[port].records):
                raise TimeoutError('Synthetic separate native host-write cutoff')
            adapter.outputs.append(tuple(wires[port])); adapter.last_output = tuple(wires[port])
        return batches

    def output(self, wires, reply_deadline_ns, write_deadline_ns, check):
        check()
        return self._output(wires, reply_deadline_ns, write_deadline_ns)

    def cancel(self):
        self.h.sequence.append(('native-cancel',))

    def close(self):
        if not self.closed:
            self.h.sequence.append(('native-close',))
            self.closed = True
            incomplete = self.h.controls.get('group_snapshot_error', False)
            self.close_report = {'closed': True, 'restored': True,
                'snapshot_complete': not incomplete,
                'snapshot_error': 'synthetic snapshot failure' if incomplete else None}
        return self.close_report


class FollowingGroup(FileOnlyGroup):
    """Synthetic motors that reach each commanded position one cycle later."""
    def _output(self, wires, reply_deadline_ns, write_deadline_ns):
        from . import type1_transport as T
        from .test_type1_runner import RAW
        batches = super()._output(wires, reply_deadline_ns, write_deadline_ns)
        drift = self.h.controls.setdefault('drift', {})
        for port in PORTS:
            for wire in wires[port]:
                mid, q, _, _ = T.type1_fields(wire)
                drift[mid] = T.decoded_q(q)-RAW[mid]
        return batches


class StopOrderMock(Mock30):
    def stop_repeated(self, **kwargs):
        if self.h.native is not None and not self.h.native.closed:
            raise AssertionError('Legacy STOP before native owner join')
        self.h.sequence.append(('legacy-terminal-stop', self.group.port))
        return super().stop_repeated(**kwargs)


class NativeRunnerTests(Harness, unittest.TestCase):
    def setUp(self):
        super().setUp()
        self.native = None

    def factory(self, group):
        self.adapters[group.port] = StopOrderMock(group, self)
        return self.adapters[group.port]

    def create_group(self, adapters):
        self.native = FileOnlyGroup(adapters, self)
        return self.native

    def run_native(self, **kwargs):
        return self.run_case(value=admitted_native(), cycle_group_factory=self.create_group, **kwargs)

    def test_native_plan_opens_nothing_and_is_explicit_30hz_only(self):
        result = R.plan(admitted_native())
        self.assertEqual(result['pacing']['native_group_cycle'], True)
        self.assertFalse(result['opens_devices'])
        from .test_type1_runner import admitted
        legacy = admitted()
        legacy['pacing'] = P.selected_pacing({'native_group_cycle': True})
        with self.assertRaisesRegex(ValueError, '30 Hz'):
            R.plan(legacy)
        self.assertNotIn('native_group_cycle', P.selected_pacing({'native_group_cycle': False}))

    def test_full_controller_uses_group_not_python_can_cycle_workers(self):
        report = self.run_native()
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', report['errors'])
        self.assertTrue(report['learned_targets_attempted'])
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(report['normal_ramp_completed'])
        self.assertTrue(report['native_group_close']['snapshot_complete'])
        for adapter in self.adapters.values():
            self.assertNotIn('hold', [name for name, _ in adapter.calls])
            self.assertEqual([name for name, _ in adapter.calls].count('output'), 1)
        self.assertTrue(all(row['request_count'] == 28 for row in report['cycles']))
        closed = self.sequence.index(('native-close',))
        self.assertTrue(all(i > closed for i, item in enumerate(self.sequence)
                            if item[0] == 'legacy-terminal-stop'))
        self.assert_all_stopped(report)
        encoded = json.dumps(R.evidence_report(report), allow_nan=False)
        self.assertIn('native_group_journal', encoded)

    def test_costly_ready_gets_actual_fresh_zero_hold_before_initial_clock(self):
        self.controls['group_setup_ns'] = 37_000_000
        report = self.run_native()
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', report['errors'])
        ready = report['native_group_setup']['ready_ns']
        old = report['before_native_group_initial_hold']
        fresh = report['initial_hold']
        self.assertLess(max(batch.stats.end_ns for batch in old.values()), ready)
        self.assertGreater(min(batch.stats.begin_ns for batch in fresh.values()), ready)
        self.assertLess(ready, report['epoch_ns'])

    def test_partial_hold_failure_joins_group_then_stops_all_legacy_ports(self):
        self.controls['group_hold_failure'] = True
        report = self.run_native()
        self.assertEqual(report['status'], 'ABORTED')
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(self.native.closed)
        self.assertEqual(report['completed_cycles'], 0)
        self.assert_all_stopped(report)

    def test_late_host_write_rejected_even_with33ms_reply_budget(self):
        self.controls['group_late_write'] = True
        report = self.run_native()
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('host-write cutoff', str(report['errors']))
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual(report['completed_cycles'], 0)

    def test_snapshot_failure_cannot_be_a_completed_predecessor_but_stop_still_runs(self):
        self.controls['group_snapshot_error'] = True
        report = self.run_native()
        self.assertEqual(report['status'], 'ABORTED')
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['native_group_close']['snapshot_complete'])
        self.assertIn('snapshot incomplete', str(report['errors']))

    def test_missing_group_or_accidental_factory_refused_before_can(self):
        with self.assertRaisesRegex(ValueError, 'exactly when selected'):
            self.run_case(value=admitted_native())
        self.assertEqual(self.adapters, {})
        with self.assertRaisesRegex(ValueError, 'exactly when selected'):
            self.run_case(value=admitted30(), cycle_group_factory=self.create_group)
        self.assertEqual(self.adapters, {})

    def test_identical_target_source_and_wire_limits_across_backends(self):
        legacy = self.run_case(value=admitted30())
        old_rows = legacy['cycles']
        self.setUp()
        native = self.run_native()
        self.assertEqual(legacy['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', legacy['errors'])
        self.assertEqual(native['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', native['errors'])
        # VirtualClock charges each Python timestamp access 1 us. The native
        # API intentionally uses fewer accesses, so its ramp end is not an
        # identical wall-clock sample. Compare all actual inference outputs;
        # both runs must separately finish the unchanged braking/gain ramp.
        old_inferred = [row for row in old_rows if row['observed'] is not None]
        new_inferred = [row for row in native['cycles'] if row['observed'] is not None]
        self.assertGreater(min(len(old_inferred), len(new_inferred)), 30)
        for old, new in zip(old_inferred, new_inferred):
            self.assertEqual(old['model_target_by_id'], new['model_target_by_id'])
        for new in native['cycles']:
            self.assertLessEqual(max(new['command']['kp']), 3.)
            for port in PORTS:
                for raw in new['output'][port].records:
                    self.assertEqual(raw.written, 17)
                    self.assertLessEqual(raw.finish_ns, new['first_ns']+20_000_000)
                    self.assertEqual(raw.deadline_ns, new['hard_end_ns'])
        self.assertTrue(native['normal_ramp_completed'])

    def late_command_at(self, call, late_ns):
        def hook(index, sample):
            if index == call:  # After the hold/IMU sample, before inference and command time.
                self.clock.advance_to(self.clock()+late_ns)
        self.controls['imu'] = hook

    def test_default_command_gap_stops_on_one_late_command_phase(self):
        self.late_command_at(4, 2_500_000)
        report = self.run_native()
        self.assertIn('Command gap exceeded', report['primary_error']['message'])
        self.assertNotIn('command_gap', report)  # Default report keys unchanged.
        self.assertTrue(report['stop_confirmed'])
        self.assert_all_stopped(report)

    def test_selected_command_gap_limit_records_and_continues(self):
        self.late_command_at(4, 2_500_000)
        value = admitted_native()
        value['pacing'] = P.selected_pacing({'native_group_cycle': True, 'command_gap_limit_us': 40_000})
        report = self.run_case(value=value, cycle_group_factory=self.create_group)
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', report['errors'])
        gaps = report['command_gap']
        self.assertEqual(gaps['stop_limit_ns'], 40_000_000)
        self.assertEqual(gaps['nominal_budget_ns'], 34_333_334)
        self.assertEqual([row['cycle_index'] for row in gaps['over_nominal']], [3])
        self.assertGreater(gaps['over_nominal'][0]['command_interval_ns'], gaps['nominal_budget_ns'])
        self.assertLess(gaps['max_command_interval_ns'], 40_000_000)

    def test_selected_command_gap_limit_still_stops_beyond_limit(self):
        self.late_command_at(4, 7_500_000)
        value = admitted_native()
        value['pacing'] = P.selected_pacing({'native_group_cycle': True, 'command_gap_limit_us': 40_000})
        report = self.run_case(value=value, cycle_group_factory=self.create_group)
        # Beyond 40 ms either the unchanged host watchdog or the selected limit stops first.
        self.assertRegex(report['primary_error']['message'], 'Command gap exceeded|Host output heartbeat expired')
        self.assertTrue(report['stop_confirmed'])
        self.assert_all_stopped(report)

    def test_command_gap_limit_range_and_30hz_only(self):
        for bad in (34_333, 40_001, 40_000.0, True):
            with self.assertRaisesRegex(ValueError, 'Command gap limit'):
                P.selected_pacing({'command_gap_limit_us': bad})
        self.assertNotIn('command_gap_limit_us', P.selected_pacing({'command_gap_limit_us': None}))
        from .test_type1_runner import admitted
        legacy = admitted()
        legacy['pacing'] = P.selected_pacing({'command_gap_limit_us': 40_000})
        with self.assertRaisesRegex(ValueError, '30 Hz'):
            R.plan(legacy)


    def stance_value(self):
        value = admitted_native()
        value['pacing'] = P.selected_pacing({'native_group_cycle': True, 'stance_transition': True})
        # The stance caps brake for 0.5 s; use the live profile's 0.4 s ramps.
        value['profile'].update(startup_duration_s=.4, policy_ramp_s=.4, stop_duration_s=.4)
        return value

    def test_stance_transition_sends_plan_targets_with_stance_caps(self):
        import math
        from . import stance
        report = self.run_case(value=self.stance_value(), cycle_group_factory=self.create_group)
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', report['errors'])
        plan = report['stance_transition']
        self.assertLessEqual(plan['max_joint_delta_deg'], 2.5+1e-9)
        self.assertGreater(plan['expected_body_rise_m'], -1.)
        labels = [row['label'] for row in report['cycles']]
        self.assertIn('stance_transition', labels)
        self.assertNotIn('policy_output', labels)
        commands = [row['command'] for row in report['cycles'] if row['label'] == 'stance_transition']
        self.assertTrue(any(max(c['kp']) == stance.KP for c in commands))
        self.assertTrue(all(max(c['kp']) <= stance.KP and max(c['kd']) <= stance.KD for c in commands))
        q0 = report['q0_model_rad_by_id']
        for c in commands:
            for mid in range(1, 13):
                self.assertLessEqual(abs(c['q_model_rad'][mid-1]-q0[str(mid)]), math.radians(2.5)+1e-9)

    def test_default_learned_run_keeps_boxed_caps_and_model_targets(self):
        report = self.run_native()
        self.assertNotIn('stance_transition', report)
        commands = [row['command'] for row in report['cycles'] if row['label'] == 'policy_output']
        self.assertTrue(commands)
        self.assertTrue(all(max(c['kp']) <= 3. for c in commands))

    def test_stance_transition_is_two_second_30hz_only(self):
        value = self.stance_value()
        value['duration_s'] = 10
        value['profile']['duration_s'] = 10
        with self.assertRaisesRegex(ValueError, '2 s, or 10/20 s'):
            R.plan(value)
        from .test_type1_runner import admitted
        legacy = admitted()
        legacy['pacing'] = P.selected_pacing({'stance_transition': True})
        with self.assertRaisesRegex(ValueError, '30 Hz'):
            R.plan(legacy)
        with self.assertRaisesRegex(ValueError, 'must be exactly true'):
            P.selected_pacing({'stance_transition': 1})

    def test_transport_stance_caps_only_when_selected(self):
        from experiments.four_bus_diagnostic.transport_adapter import Group
        from . import stance, type1_transport as T
        group = Group('port3', (1, 2, 3))
        bounds = {mid: (-.05, .05) for mid in group.ids}
        kp, kd = {mid: stance.KP for mid in group.ids}, {mid: stance.KD for mid in group.ids}
        with self.assertRaisesRegex(ValueError, 'kp cap must be 0..3'):
            T.session_limits(group, bounds, kp, kd)
        lower, upper, kps, kds = T.session_limits(group, bounds, kp, kd, stance_caps=True)
        self.assertEqual(kps[1], stance.KP)
        with self.assertRaisesRegex(ValueError, 'kp cap must be 0..'+str(stance.KP)):
            T.session_limits(group, bounds, {mid: stance.KP+.5 for mid in group.ids}, kd, stance_caps=True)


    def percent_value(self, percent=35):
        value = admitted_native()
        value['pacing'] = P.selected_pacing({'native_group_cycle': True, 'stance_transition': True,
                                             'stance_lift_percent': percent})
        value['duration_s'] = 10
        value['profile'].update(duration_s=10, startup_duration_s=.4, policy_ramp_s=.4, stop_duration_s=.4)
        return value

    def test_percent_lift_rises_holds_and_returns_within_widened_window(self):
        import math
        from experiments.stance_transition import plan as SP
        def following(adapters):
            self.native = FollowingGroup(adapters, self)
            return self.native
        report = self.run_case(value=self.percent_value(), cycle_group_factory=following)
        self.assertEqual(report['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', report['errors'])
        plan = report['stance_transition']
        self.assertEqual(plan['percent'], 35)
        self.assertTrue(plan['return_to_start'])
        self.assertLess(plan['max_foot_slide_m'], 1e-12)
        self.assertGreater(plan['max_joint_delta_deg'], 3.)  # Beyond the reviewed +-3 deg window.
        q0 = {int(k): v for k, v in report['q0_model_rad_by_id'].items()}
        commands = [row['command'] for row in report['cycles'] if row['label'] == 'stance_transition']
        heights = [SP.body_height_m({mid: c['q_model_rad'][mid-1] for mid in range(1, 13)}) for c in commands]
        start = SP.body_height_m(q0)
        self.assertGreater(max(heights)-start, plan['lift_m']*.9)
        self.assertAlmostEqual(heights[-1], start, places=6)  # Lowered back (micrometres) before the stop.

    def test_percent_requires_ten_seconds_and_stance(self):
        value = self.percent_value()
        value['duration_s'] = 2
        value['profile']['duration_s'] = 2
        report_plan = R.plan(value)  # 2 s with a percent contract keeps the small lift.
        self.assertEqual(report_plan['duration_s'], 2)
        value = self.percent_value()
        value['duration_s'] = 10
        value['pacing'] = P.selected_pacing({'native_group_cycle': True, 'stance_transition': True})
        with self.assertRaisesRegex(ValueError, '10/20 s with stance_lift_percent'):
            R.plan(value)
        for bad in (0, 66, 35.0, True):
            with self.assertRaisesRegex(ValueError, 'stance_lift_percent'):
                P.selected_pacing({'stance_transition': True, 'stance_lift_percent': bad})
        with self.assertRaisesRegex(ValueError, 'stance_lift_percent'):
            P.selected_pacing({'stance_lift_percent': 35})

    def test_foreground_transport_limits_match_runner_widened_axes(self):
        from . import stance, type1_foreground as F
        from experiments.four_bus_diagnostic.transport_adapter import Group
        from singularitydog_hw.policy_output_runtime import MODEL_TARGET_LIMITS_BY_ID
        value = self.percent_value()
        axes = {key: dict(axis, fixed_offset_rad=value['offsets_by_id'][key])
                for key, axis in value['profile']['axes'].items()}
        for key, axis in axes.items():
            raw = sorted(((axis['lower_rad']-axis['fixed_offset_rad'])/axis['sign'],
                          (axis['upper_rad']-axis['fixed_offset_rad'])/axis['sign']))
            axis['raw_lower_rad'], axis['raw_upper_rad'] = raw
        contract = {'axes': axes, 'pacing': value['pacing'],
                    'start_pose_bounds': value['profile']['start_pose_bounds']}
        spec = R.validate_admitted(value)
        group = Group('port3', (1, 2, 3))
        bounds, kp, kd = F.transport_limits(contract, group, 'learned_boxed', 10)
        for mid in group.ids:
            axis = spec['profile']['axes'][str(mid)]
            self.assertEqual(bounds[mid], (axis['raw_lower_rad'], axis['raw_upper_rad']))
            self.assertEqual(kp[mid], stance.KP)
        small, _, _ = F.transport_limits(contract, group, 'learned_boxed', 2)
        for mid in group.ids:
            self.assertEqual(small[mid], (contract['axes'][str(mid)]['raw_lower_rad'],
                                          contract['axes'][str(mid)]['raw_upper_rad']))


    def test_observer_guard_uses_the_same_widened_windows(self):
        from . import type1_foreground as F
        from experiments.four_bus_diagnostic import model_bridge as B
        value = self.percent_value()
        spec = R.validate_admitted(value)
        axes = {key: dict(axis, fixed_offset_rad=value['offsets_by_id'][key])
                for key, axis in value['profile']['axes'].items()}
        contract = {'axes': axes, 'pacing': value['pacing'],
                    'start_pose_bounds': value['profile']['start_pose_bounds']}
        windows = F.stance_windows(contract, 'learned_boxed', 10)
        for mid, (lower, upper) in windows.items():
            axis = spec['profile']['axes'][str(mid)]
            self.assertEqual((lower, upper), (axis['physical_lower_rad'], axis['physical_upper_rad']))
        self.assertIsNone(F.stance_windows(contract, 'learned_boxed', 2))
        self.assertIsNone(F.stance_windows(contract, 'zero_gain_timing', 2))
        plan_axes = B._plan_axes(value['model_plan'])
        widened = B.widen_position_windows(plan_axes, windows)
        self.assertTrue(any(widened[mid][4]-widened[mid][3] > plan_axes[mid][4]-plan_axes[mid][3]+1e-6
                            for mid in windows))
        narrow = {mid: (plan_axes[mid][3]+.001, plan_axes[mid][4]) for mid in windows}
        with self.assertRaisesRegex(ValueError, 'contain the reviewed window'):
            B.widen_position_windows(plan_axes, narrow)


if __name__ == '__main__':
    unittest.main()
