"""30 Hz file-only contracts and injected owner tests; never opens hardware."""
import copy
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
import struct
from unittest.mock import patch

from singularitydog_hw import policy_model_30hz as models
from singularitydog_hw.policy_clock import PolicyClock, control_clock_payload, validate_control_clock
from experiments.four_bus_diagnostic import model_bridge
from singularitydog_hw.native_diagnostic_transport import stop_wire
from . import type1_profile as profile, type1_runner as runner
from . import test_type1_profile as profiles, test_type1_runner as runners
from . import test_type1_foreground as foregrounds, type1_profile as P, type1_transport as T

BUNDLE = Path(__file__).resolve().parents[4] / 'RS05_30Hz_Model_20261013'


def model_selection():
    return models.plan(models.bundle_artifacts(BUNDLE))


def proxy30(capture_ref, by_port, start, clock_payload, model, *, split=False, source_sha='9'*64):
    """Complete synthetic wire/timestamp fixture, never an actual qualification."""
    report = profiles.stop_proxy(capture_ref, by_port, start)
    clock = validate_control_clock(clock_payload)
    report.update(schema='singularitydog.four-bus-foreground-stop-proxy.v2',
                  control_clock=clock_payload, policy30_model=model)
    m = report['measurement']
    m.update(schema='singularitydog.four-bus-stop-proxy-pipeline.v2', control_clock=clock_payload,
        policy30_model=model, absolute_deadline_ns=clock.cycle_budget_ns,
        input_to_stop_proxy_tx_budget_ns=20_000_000,
        proxy_exchange_budget_ns=clock.cycle_budget_ns if split else 20_000_000,
        diagnostic_reply_budget_unchanged=not split, diagnostic_type1_33ms_reply_qualification=False)
    if split:
        shared = dict(input_to_stop_proxy_tx_budget_ns=20_000_000, acquisition_budget_ns=20_000_000,
            proxy_exchange_budget_ns=clock.cycle_budget_ns, proxy_reply_budget_ns=clock.cycle_budget_ns,
            proxy_write_deadline_abi_required=1, diagnostic_reply_budget_unchanged=False,
            diagnostic_type1_33ms_reply_qualification=False)
        report.update(shared, proxy_write_deadline_abi_verified=1, plan_source_manifest_sha256=source_sha)
        m.update(shared, backend_usage=dict(kind='genuine_original_active_subset_cpp',
            library_sha256='8'*64, build_record_sha256='7'*64, source_manifest_sha256=source_sha,
            proxy_write_deadline_abi_verified=1))
        report['input_sha256'].update({'/synthetic/library': '8'*64,
            '/synthetic/build': '7'*64, '/synthetic/whole-source': source_sha})
    for index, row in enumerate(m['records']):
        release = clock.release_ns(start, index)
        prefix = release+20_000_000
        hard = clock.cycle_deadline_ns(release)
        cap = hard if split else prefix
        row.update(scheduled_release_ns=release, release_ns=release,
            cycle_end_ns=release+(23_000_000 if split else 19_000_000), original_deadline_ns=hard,
            input_to_stop_proxy_tx_deadline_ns=prefix, input_to_stop_proxy_tx_20ms_met=True, output={})
        if split:
            row.update(proxy_reply_deadline_ns=hard, proxy_exchange_budget_ns=clock.cycle_budget_ns,
                proxy_write_deadline_abi_required=1, diagnostic_reply_budget_unchanged=False)
        for port, ids in by_port.items():
            records = []
            for slot, mid in enumerate(ids):
                begin = release+13_000_000+slot*900_000
                can_id = (2 << 24)|(mid << 8)|0xfd
                rx = (b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+
                      struct.pack('>4H', 32767, 32767, 32767, 250)+b'\r\n')
                records.append(dict(tx_hex=stop_wire(mid).hex(), rx_hex=rx.hex(), start_ns=begin,
                    finish_ns=begin+10, read_start_ns=begin+11,
                    received_ns=release+(22_000_000 if split else 18_000_000),
                    deadline_ns=cap, written=17, received=17))
            batch = dict(physical_port=port, physical_ids=list(ids), phase='output_stop',
                         original_record_count=3, records=records)
            if split:
                batch.update(separate_write_deadline_monotonic_ns=prefix,
                             record_deadlines_are_reply_deadlines=True)
            row['output'][port] = batch
        row['final_proxy_host_write_ns'] = max(raw['finish_ns'] for batch in row['output'].values()
                                              for raw in batch['records'])
    m['all_original_workers_joined_monotonic_ns'] = clock.release_ns(start, 501)+500_000_000
    return report


def admitted30(mode='learned_boxed'):
    value = runners.admitted(mode)
    clock = control_clock_payload(30)
    model = model_selection()
    value.update(control_clock=clock, policy30_model=model)
    value['profile'].update(period_ms=1000/30, hard_cycle_ms=1000/30,
                            max_sample_gap_ms=1000/30+1)
    value['model_plan'] = model_bridge.reclock_model_plan(value['model_plan'], clock, model)
    return value


class Mock30(runners.MockType1):
    """Synthetic write-cutoff extension; old mock remains byte/API unchanged."""
    def output(self, wires, *, deadline_ns, check, write_deadline_ns=None):
        if type(write_deadline_ns) is not int or self.h.clock() >= write_deadline_ns:
            raise TimeoutError('Synthetic independent host-write cutoff')
        batch = super().output(wires, deadline_ns=deadline_ns, check=check)
        if any(record.finish_ns > write_deadline_ns for record in batch.records):
            raise TimeoutError('Synthetic independent host-write cutoff')
        return batch


class DelayedReply30(Mock30):
    """Write every port by20ms, then deliver synthetic replies after20ms."""
    def output(self, wires, *, deadline_ns, check, write_deadline_ns=None):
        if write_deadline_ns == deadline_ns:  # Initial zero-gain hold keeps20ms.
            return super().output(wires, deadline_ns=deadline_ns, check=check,
                                  write_deadline_ns=write_deadline_ns)
        self.h.clock.advance_to(write_deadline_ns-3_000_000)
        batch = super().output(wires, deadline_ns=deadline_ns, check=check,
                               write_deadline_ns=write_deadline_ns)
        self.h.ack_barrier.wait(timeout=1)  # All four ports have finished writing.
        received = self.h.clock.advance_to(write_deadline_ns+7_000_000)
        for record in batch.records:
            record.received_ns = received
        batch.stats.end_ns = received
        rows = runners.decode_records((batch.records, batch.stats))
        new = runners.Batch(batch.group, batch.label, batch.records, batch.stats, rows,
                            bytes(batch.records), bytes(batch.stats), received)
        self.last_batch = new
        return new


@unittest.skipUnless(BUNDLE.exists(), 'Pinned 30 Hz release is unavailable')
class Profile30Tests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.world = profiles.World(Path(self.temporary.name).resolve())
        self.clock = control_clock_payload(30)
        self.model = model_selection()

    def contract(self, **kwargs):
        return profile.build_contract(copy.deepcopy(self.world.plan), self.world.lineage_refs(),
            copy.deepcopy(self.world.geometry_doc), self.world.geometry,
            control_clock=self.clock, policy30_model=self.model, **kwargs)

    def test_new_clock_contract_keeps_every_physical_limit(self):
        old = self.world.contract()
        new = self.contract()
        self.assertEqual(new['schema'], profile.PROFILE_SCHEMA_30)
        for field in ('axes', 'start_pose_bounds', 'pacing', 'watchdog', 'enable', 'terminal_stop',
                      'imu_limits', 'ramps', 'lineage', 'source_manifest'):
            self.assertEqual(new[field], old[field], field)
        expected = copy.deepcopy(old['timing'])
        expected.update(period_ms=1000/30, hard_cycle_ms=1000/30, max_sample_gap_ms=1000/30+1)
        self.assertEqual(new['timing'], expected)
        self.assertNotEqual(profile.contract_sha256(new), profile.contract_sha256(old))
        self.assertFalse(new['policy30_model']['output_allowed'])
        profile.validate_contract(new)

    def test_partial_clock_or_model_selection_is_rejected(self):
        args = (self.world.plan, self.world.lineage_refs(), self.world.geometry_doc, self.world.geometry)
        for kw in ({'control_clock': self.clock}, {'policy30_model': self.model}):
            with self.subTest(selection=list(kw)), self.assertRaisesRegex(ValueError, 'selected together'):
                profile.build_contract(*args, **kw)

    def test_embedded_30hz_plan_cannot_be_wrapped_in_legacy_contract(self):
        plan = model_bridge.reclock_model_plan(self.world.plan, self.clock, self.model)
        with self.assertRaisesRegex(ValueError, 'Explicit 30 Hz selection'):
            profile.build_contract(plan, self.world.lineage_refs(),
                                   self.world.geometry_doc, self.world.geometry)
        contract = self.world.contract()
        contract['model_plan'] = plan
        contract['model_plan_canonical_sha256'] = plan['plan_canonical_sha256']
        with self.assertRaisesRegex(ValueError, 'Legacy 50 Hz contract'):
            profile.validate_contract(contract)

    def test_old_command_phase_and_prearm_are_not_implicitly_requalified(self):
        for options in ({'command_phase_offset_us': 10000}, {'prearmed_hold_lead_us': 1000}):
            with self.subTest(options=options), self.assertRaisesRegex(ValueError, 'historical 20 ms'):
                self.contract(options=options)
        new = self.contract(options={'decode_once': True, 'gc_freeze': True})
        self.assertTrue(new['pacing']['decode_once'])

    def test_new_clock_cannot_claim_old_20ms_proxy_evidence(self):
        report = profiles.stop_proxy(self.world.lineage, self.world.plan['topology_by_port'], 60_000_000_000)
        with self.assertRaisesRegex(ValueError, 'Complete four-bus STOP-proxy'):
            profile.validate_stop_proxy_report(report, capture_refs=[self.world.lineage],
                topology_by_port=self.world.plan['topology_by_port'],
                control_clock=self.clock, policy30_model=self.model)

    def test_new_clock_proxy_is_bound_to_same_model_and_prefix(self):
        report = proxy30(self.world.lineage, self.world.plan['topology_by_port'], 60_000_000_000,
                         self.clock, self.model)
        result = profile.validate_stop_proxy_report(report, capture_refs=[self.world.lineage],
            topology_by_port=self.world.plan['topology_by_port'],
            control_clock=self.clock, policy30_model=self.model)
        self.assertEqual(result['control_clock'], self.clock)
        report['measurement']['input_to_stop_proxy_tx_budget_ns'] = 33_333_334
        with self.assertRaisesRegex(ValueError, 'input-to-write'):
            profile.validate_stop_proxy_report(report, capture_refs=[self.world.lineage],
                topology_by_port=self.world.plan['topology_by_port'],
                control_clock=self.clock, policy30_model=self.model)

    def test_split_native_proxy_with22ms_ack_and20ms_writes_can_be_a_prerequisite_only(self):
        report = proxy30(self.world.lineage, self.world.plan['topology_by_port'], 60_000_000_000,
                         self.clock, self.model, split=True)
        result = profile.validate_stop_proxy_report(report, capture_refs=[self.world.lineage],
            topology_by_port=self.world.plan['topology_by_port'], control_clock=self.clock,
            policy30_model=self.model, source_manifest_sha256='9'*64)
        self.assertEqual(result['completed_cycles'], 501)
        self.assertEqual(result['control_clock'], self.clock)
        self.assertNotIn('output_allowed', result)
        self.assertFalse(report['measurement']['diagnostic_type1_33ms_reply_qualification'])

    def test_split_proxy_missing_late_wrong_abi_or_unpinned_evidence_never_qualifies(self):
        base = proxy30(self.world.lineage, self.world.plan['topology_by_port'], 60_000_000_000,
                       self.clock, self.model, split=True)
        row_at = lambda r: r['measurement']['records'][500]
        raw_at = lambda r: row_at(r)['output']['port3']['records'][2]
        cases = {
            'no output batch': lambda r: row_at(r)['output'].pop('port3'),
            'one missing reply': lambda r: raw_at(r).update(received=0),
            'one partial write': lambda r: raw_at(r).update(written=8),
            'read start missing': lambda r: raw_at(r).pop('read_start_ns'),
            'read start zero': lambda r: raw_at(r).update(read_start_ns=0),
            'read before write finish': lambda r: raw_at(r).update(read_start_ns=raw_at(r)['finish_ns']-1),
            'read after receive': lambda r: raw_at(r).update(read_start_ns=raw_at(r)['received_ns']+1),
            'write at20ms': lambda r: raw_at(r).update(finish_ns=row_at(r)['input_to_stop_proxy_tx_deadline_ns']),
            'ACK at33ms': lambda r: raw_at(r).update(received_ns=row_at(r)['original_deadline_ns']),
            'whole-cycle at33ms': lambda r: row_at(r).update(cycle_end_ns=row_at(r)['original_deadline_ns']),
            'old20ms raw ACKcap': lambda r: raw_at(r).update(deadline_ns=row_at(r)['input_to_stop_proxy_tx_deadline_ns']),
            'dispatch cutoff changed': lambda r: row_at(r)['output']['port3'].update(
                separate_write_deadline_monotonic_ns=row_at(r)['original_deadline_ns']),
            'clock relabeled': lambda r: r['measurement'].update(control_clock=control_clock_payload(50)),
            'verified ABI missing': lambda r: r.pop('proxy_write_deadline_abi_verified'),
            'verified ABI bool': lambda r: r.update(proxy_write_deadline_abi_verified=True),
            'mock backend': lambda r: r['measurement']['backend_usage'].update(kind='injected_file_only_mock'),
            'unbound source': lambda r: r['measurement']['backend_usage'].update(source_manifest_sha256='4'*64),
            'different last write': lambda r: row_at(r).update(final_proxy_host_write_ns=row_at(r)['release_ns']+19_000_000),
            'mode or fault': lambda r: raw_at(r).update(rx_hex=raw_at(r)['rx_hex'][:4]+'4801'+raw_at(r)['rx_hex'][8:]),
            'Type1 claimed': lambda r: r['measurement'].update(diagnostic_type1_33ms_reply_qualification=True),
        }
        for name, mutate in cases.items():
            value = copy.deepcopy(base)
            mutate(value)
            with self.subTest(name=name), self.assertRaises(ValueError):
                profile.validate_stop_proxy_report(value, capture_refs=[self.world.lineage],
                    topology_by_port=self.world.plan['topology_by_port'], control_clock=self.clock,
                    policy30_model=self.model, source_manifest_sha256='9'*64)
        with self.assertRaisesRegex(ValueError, 'current Type1 contract'):
            profile.validate_stop_proxy_report(base, capture_refs=[self.world.lineage],
                topology_by_port=self.world.plan['topology_by_port'], control_clock=self.clock,
                policy30_model=self.model, source_manifest_sha256='1'*64)

    def test_original30hz_proxy20_contract_cannot_use22ms_reply_evidence(self):
        report = proxy30(self.world.lineage, self.world.plan['topology_by_port'], 60_000_000_000,
                         self.clock, self.model)
        last = report['measurement']['records'][500]
        last['cycle_end_ns'] = last['release_ns']+23_000_000
        last['output']['port3']['records'][2]['received_ns'] = last['release_ns']+22_000_000
        with self.assertRaisesRegex(ValueError, 'per-axis write/reply deadlines'):
            profile.validate_stop_proxy_report(report, capture_refs=[self.world.lineage],
                topology_by_port=self.world.plan['topology_by_port'], control_clock=self.clock,
                policy30_model=self.model)

    def test_file_prepare30_pins_all_new_model_artifacts_and_accepts_same_source_split_proxy(self):
        source_sha = self.world.contract()['source_manifest']['sha256']
        diagnostic = proxy30(self.world.current, self.world.plan['topology_by_port'], 60_000_000_000,
                             self.clock, self.model, split=True, source_sha=source_sha)
        reference = profiles.write(self.world.root/'synthetic-split-proxy.json', diagnostic)
        args = self.world.args('zero_gain_timing', 2, self.world.root/'fresh-profile.json',
            stop_proxy_report=reference['path'], stop_proxy_report_sha256=reference['sha256'],
            policy30_bundle=str(BUNDLE))
        with patch('ctypes.CDLL', side_effect=AssertionError('File builder opened native library')), \
             patch.object(models, 'load', side_effect=AssertionError('File builder executed model')):
            result, pins = profile.build_from_files(args)
        self.assertEqual(result['schema'], profile.PROFILE_SCHEMA_30)
        self.assertFalse(result['output_allowed'])
        self.assertFalse(result['admitted'])
        self.assertEqual(result['evidence']['stop_proxy_diagnostic']['control_clock'], self.clock)
        for ref in self.model['artifacts'].values():
            self.assertEqual(pins.values[ref['path']], ref['sha256'])

    def test_relabelled_old_proxy_rows_are_rejected_as_wrong_cadence(self):
        report = profiles.stop_proxy(self.world.lineage, self.world.plan['topology_by_port'], 60_000_000_000)
        report.update(schema='singularitydog.four-bus-foreground-stop-proxy.v2',
                      control_clock=self.clock, policy30_model=self.model)
        report['measurement'].update(schema='singularitydog.four-bus-stop-proxy-pipeline.v2',
            control_clock=self.clock, policy30_model=self.model,
            absolute_deadline_ns=33_333_334, input_to_stop_proxy_tx_budget_ns=20_000_000,
            proxy_exchange_budget_ns=20_000_000, diagnostic_reply_budget_unchanged=True,
            diagnostic_type1_33ms_reply_qualification=False)
        with self.assertRaisesRegex(ValueError, 'diagnostic epoch'):
            profile.validate_stop_proxy_report(report, capture_refs=[self.world.lineage],
                topology_by_port=self.world.plan['topology_by_port'],
                control_clock=self.clock, policy30_model=self.model)

    def test_same_contract_predecessor_requires_new_clock_report(self):
        contract = self.contract()
        report = profiles.type1_report(contract, 'zero_gain_timing', 2, 100, 200)
        with self.assertRaisesRegex(ValueError, 'run report'):
            profile.validate_type1_report(report, contract=contract,
                contract_digest=profile.contract_sha256(contract), mode='zero_gain_timing')
        report.update(schema=profile.REPORT_SCHEMA_30, control_clock=self.clock,
            policy30_model=self.model, cycle_budget_ns=33_333_334,
            inference_tx_budget_ns=20_000_000, completed_cycles=55)
        profile.validate_type1_report(report, contract=contract,
            contract_digest=profile.contract_sha256(contract), mode='zero_gain_timing')
        report['policy30_model'] = dict(self.model, training_updates=499)
        with self.assertRaisesRegex(ValueError, 'new clock|clock/model'):
            profile.validate_type1_report(report, contract=contract,
                contract_digest=profile.contract_sha256(contract), mode='zero_gain_timing')


@unittest.skipUnless(BUNDLE.exists(), 'Pinned 30 Hz release is unavailable')
class Runner30Tests(runners.Harness, unittest.TestCase):
    def factory(self, group):
        result = Mock30(group, self)
        self.adapters[group.port] = result
        return result

    def run_bound_observer(self, observer, *, value=None, setup=None,
                           backend='injected_file_only_mock'):
        return runner.run(value or admitted30(), factory=self.factory, imu_read=self.imu,
            observer=observer, check_current=self.current, check_cancelled=self.check,
            cancel_io=self.cancel_io,
            model_setup=setup or (lambda o, **counts: dict(counts, reset_verified=True,
                                                          file_only_mock_model=True)),
            worker_scope=self.worker_scope, main_scope=self.main_scope,
            release_wait=self.release_wait, announce=lambda: self.events.append(('announce',)),
            stop_requested=self.stop_requested, backend_usage={'kind': backend},
            clock=self.clock, execute=True)

    def test_known_legacy_observer_cannot_execute_inside_30hz_runner(self):
        observer = runners.Observer()
        observer._control_clock = None
        with self.assertRaisesRegex(ValueError, 'observer and outer control clock differ'):
            self.run_bound_observer(observer)
        self.assertEqual(observer.calls, 0)
        self.assertEqual(self.adapters, {})
        self.assertEqual(self.events, [])

    def test_known_30hz_observer_cannot_execute_inside_legacy_runner(self):
        observer = runners.Observer()
        observer._control_clock = PolicyClock(30)
        with self.assertRaisesRegex(ValueError, 'observer and outer control clock differ'):
            self.run_bound_observer(observer, value=runners.admitted())
        self.assertEqual(observer.calls, 0)
        self.assertEqual(self.adapters, {})
        self.assertEqual(self.events, [])

    def test_unknown_observer_clock_only_remains_allowed_for_file_only_mock(self):
        observer = runners.Observer()
        with self.assertRaisesRegex(ValueError, 'Real diagnostic observer must advertise'):
            self.run_bound_observer(observer, backend='genuine_subset_active_cpp')
        self.assertEqual(self.adapters, {})
        self.assertEqual(self.events, [])
        result = self.run_bound_observer(observer)
        self.assertEqual(result['status'], profile.COMPLETE_STATUS['learned_boxed'], result['errors'])
        self.assert_all_stopped(result)

    def test_matching_advertised_30hz_observer_runs_and_stops_normally(self):
        observer = runners.Observer()
        observer._control_clock = PolicyClock(30)
        result = self.run_bound_observer(observer)
        self.assertEqual(result['status'], profile.COMPLETE_STATUS['learned_boxed'], result['errors'])
        self.assertGreater(observer.calls, 0)
        self.assertTrue(result['stop_confirmed'])
        self.assert_all_stopped(result)

    def test_clock_changed_during_warmup_is_rejected_before_enable_or_consumption(self):
        observer = runners.Observer()
        observer._control_clock = PolicyClock(30)
        def mutate(o, **counts):
            o._control_clock = None
            return dict(counts, reset_verified=True, file_only_mock_model=True)
        result = self.run_bound_observer(observer, setup=mutate)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('observer and outer control clock differ', result['primary_error']['message'])
        self.assertEqual(observer.calls, 0)
        self.assertFalse(result['motor_enable_sent'])
        self.assertFalse(result['type1_sent'])
        self.assert_all_stopped(result)

    def test_clock_changed_after_first_inference_is_rejected_before_next_consumption(self):
        observer = runners.Observer()
        observer._control_clock = PolicyClock(30)
        def mutate(call, _targets):
            if call == 1:
                observer._control_clock = None
        observer.hook = mutate
        result = self.run_bound_observer(observer)
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('observer and outer control clock differ', result['primary_error']['message'])
        self.assertEqual(observer.calls, 1)
        self.assert_all_stopped(result)

    def test_plan_keeps_send_prefix_and_stop_budgets_separate(self):
        result = runner.plan(admitted30())
        self.assertEqual(result['schema'], profile.REPORT_SCHEMA_30)
        self.assertEqual(result['cycle_budget_ns'], 33_333_334)
        self.assertEqual(result['inference_tx_budget_ns'], 20_000_000)
        self.assertEqual(result['host_watchdog_ns'], 40_000_000)
        self.assertEqual(result['motor_watchdog_ticks'], 4000)
        self.assertFalse(result['opens_devices'])
        self.assertFalse(result['output_approval_granted_here'])

    def test_embedded_30hz_plan_cannot_be_wrapped_in_legacy_runtime(self):
        value = admitted30()
        del value['control_clock'], value['policy30_model']
        value['profile'].update(period_ms=20, hard_cycle_ms=20, max_sample_gap_ms=21)
        with self.assertRaisesRegex(ValueError, 'Legacy 50 Hz runtime'):
            runner.validate_admitted(value)

    def test_finite_30hz_learned_run_uses_exact_rational_epochs(self):
        result = self.run_case(value=admitted30())
        self.assertEqual(result['status'], profile.COMPLETE_STATUS['learned_boxed'], result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['normal_ramp_completed'])
        self.assertEqual(result['input_to_tx_20ms_violations'], 0)
        self.assertLessEqual(result['completed_cycles'], 60)
        clock = validate_control_clock(result['control_clock'])
        for row in result['cycles']:
            self.assertEqual(row['release_ns'], clock.release_ns(result['epoch_ns'], row['index']))
            self.assertLessEqual(row['final_write_ns'], row['write_deadline_ns'])
            self.assertEqual(row['decision']['input_to_tx_budget_ns'], 20_000_000)
        self.assert_all_stopped(result)

    def test_finite_30hz_zero_gain_run_does_not_attempt_learned_targets(self):
        result = self.run_case(value=admitted30('zero_gain_timing'))
        self.assertEqual(result['status'], profile.COMPLETE_STATUS['zero_gain_timing'], result['errors'])
        self.assertFalse(result['learned_targets_attempted'])
        self.assertFalse(result['positive_gain_sent'])
        self.assert_all_stopped(result)

    def test_20ms_late_inference_is_rejected_before_new_output(self):
        def delay(_calls, _targets):
            self.clock.advance_to(self.clock()+21_000_000)
        result = self.run_case(value=admitted30(), observer=runners.Observer(delay))
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('Inference exceeded', result['primary_error']['message'])
        self.assertFalse(result['learned_targets_attempted'])
        self.assert_all_stopped(result)

    def test_reply_after20ms_is_allowed_only_after_all_writes_before20ms(self):
        self.ack_barrier = threading.Barrier(4)
        def factory(group):
            result = DelayedReply30(group, self)
            self.adapters[group.port] = result
            return result
        self.factory = factory
        result = self.run_case(value=admitted30())
        self.assertEqual(result['status'], profile.COMPLETE_STATUS['learned_boxed'], result['errors'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(all(row['output_last_reply_ns']-row['begin_ns'] > 20_000_000
                            for row in result['cycles']))
        self.assertTrue(all(row['final_write_ns'] <= row['write_deadline_ns']
                            for row in result['cycles']))
        self.assertEqual(result['input_to_tx_20ms_violations'], 0)
        self.assert_all_stopped(result)

    def test_missing_hold_reply_still_stops_all_ports(self):
        self.controls['hold_fail'] = ('port0', 1)
        result = self.run_case(value=admitted30())
        self.assertEqual(result['status'], 'ABORTED')
        self.assertIn('hold failure', result['primary_error']['message'])
        self.assertTrue(result['stop_confirmed'])
        self.assert_all_stopped(result)

    def test_freshness20ms_is_not_replaced_with_new_cycle33ms(self):
        def old_imu(call, sample):
            if call:
                sample['read_started_monotonic_ns'] -= 21_000_000
                sample['read_finished_monotonic_ns'] -= 21_000_000
        self.controls['imu'] = old_imu
        result = self.run_case(value=admitted30())
        self.assertEqual(result['status'], 'ABORTED')
        self.assertFalse(result['learned_targets_attempted'])
        self.assert_all_stopped(result)

    def test_reused_imu_timestamp_is_rejected_for_next_policy_decision(self):
        saved = []
        def old_imu(call, sample):
            if call == 1:
                saved.append(sample['read_started_monotonic_ns'])
            elif call > 1:
                sample['read_started_monotonic_ns'] = saved[0]
                sample['read_finished_monotonic_ns'] = saved[0]
        self.controls['imu'] = old_imu
        result = self.run_case(value=admitted30())
        self.assertEqual(result['status'], 'ABORTED')
        self.assert_all_stopped(result)

    def test_ambiguous_stop_still_requires_power_cutoff(self):
        self.controls['stop_ambiguous'] = 'port2'
        result = self.run_case(value=admitted30())
        self.assertEqual(result['status'], profile.STOP_UNCONFIRMED_STATUS)
        self.assertTrue(result['physical_cutoff_required'])
        self.assertFalse(result['stop_confirmed'])
        self.assert_all_stopped(result)


@unittest.skipUnless(BUNDLE.exists(), 'Pinned 30 Hz release is unavailable')
class Foreground30PublicationTests(foregrounds.Base):
    """Genuine F.execute publication -> genuine P consumer, no hand-added report fields."""
    @classmethod
    def setUpClass(cls):
        cls.root = Path(tempfile.mkdtemp()).resolve()
        cls.world = foregrounds.World(cls.root)
        cls.clock_payload, cls.model = control_clock_payload(30), model_selection()
        cls.world.contract = P.build_contract(copy.deepcopy(cls.world.plan),
            copy.deepcopy(cls.world.contract['lineage']), copy.deepcopy(cls.world.geometry_doc),
            cls.world.geometry, control_clock=cls.clock_payload, policy30_model=cls.model)
        cls.world.digest = P.contract_sha256(cls.world.contract)
        # Explicit synthetic receipt for the separate-write ABI. No dlopen.
        record = json.loads(Path(cls.world.type1_build['path']).read_text())
        record['four_bus_subset_active']['write_deadline_abi'] = 1
        cls.world.type1_build = foregrounds.write(Path(cls.world.type1_build['path']), record)
        cls.current = cls.world.capture('current30', 50_000_000_000)

    def admission30(self, name, mode, capture, predecessor=None):
        diagnostic = None
        if mode == 'zero_gain_timing':
            diagnostic = {'report': {'path': str(self.root/'synthetic-diag30.json'), 'sha256': 'e'*64},
                'pinned_capture': capture['topology'], 'completed_cycles': 501,
                'first_release_monotonic_ns': 60_000_000_000,
                'workers_joined_monotonic_ns': 77_000_000_000,
                'control_clock': self.clock_payload, 'policy30_model_canonical_sha256': P.canonical_sha256(self.model)}
        profile = P.assemble_profile(copy.deepcopy(self.world.contract), mode, 2,
            {'current_capture': capture, 'stop_proxy_diagnostic': diagnostic, 'predecessor': predecessor},
            prepared_at='2026-10-14T09:00:00+09:00')
        conditions = P.conditions_record(user_statement='SYNTHETIC current boxed conditions', user_reply_id=None,
            boot_id=foregrounds.BOOT, motor_power_epoch=foregrounds.EPOCH,
            contract_sha256=profile['contract_sha256'], authorized_modes=list(P.MODES),
            authorized_durations_s=[2], record_written_at='2026-10-14T09:00:01+09:00',
            **foregrounds.CONDITIONS)
        return (foregrounds.write(self.root/(name+'-profile.json'), profile),
                foregrounds.write(self.root/(name+'-conditions.json'), conditions))

    def run30(self, admission, capture, mode):
        def exchange(motors, handle, mask, raw, count, send_only, reply_cap, write_cap,
                     records, stats, error, size):
            self.assertLess(motors.clock(), write_cap)
            self.assertLessEqual(write_cap, reply_cap)
            status = motors.sda_subset_exchange(handle, mask, raw, count, send_only,
                reply_cap, records, stats, error, size)
            self.assertTrue(all(record.finish_ns < write_cap for record in records))
            return status
        seal = SimpleNamespace(write_deadline=('synthetic ABI', 'synthetic exchange'))
        with patch.object(foregrounds, 'SEAL', seal), \
             patch.object(foregrounds.Motors, 'sda_subset_exchange_write_deadline', exchange, create=True):
            return self.run_mode(admission, capture, mode)

    def test_actual_foreground30_report_is_consumed_directly_and_chains_to_learned(self):
        zero = self.run30(self.admission30('zero30', 'zero_gain_timing', self.current),
                          self.current, 'zero_gain_timing')
        original = json.loads(zero.path.read_text())
        self.assertEqual(original['status'], P.COMPLETE_STATUS['zero_gain_timing'], original['errors'])
        self.assertEqual(original['inference_tx_budget_ns'], original['measurement']['inference_tx_budget_ns'])
        self.assertEqual(original['cycle_budget_ns'], original['measurement']['cycle_budget_ns'])
        self.assertEqual((original['inference_tx_budget_ns'], original['cycle_budget_ns']), (20_000_000, 33_333_334))
        summary = P.validate_type1_report(original, contract=self.world.contract,
            contract_digest=self.world.digest, mode='zero_gain_timing', duration_s=2)
        # No report.update()/normalization: the production foreground serialized
        # the fields consumed by the next genuine admission.
        later = self.world.capture('after-zero30', summary['terminal_stop_finished_monotonic_ns']+1_000_000_000)
        predecessor = {'report': {'path': str(zero.path), 'sha256': foregrounds.sha(zero.path)}, **summary}
        learned = self.run30(self.admission30('learned30', 'learned_boxed', later, predecessor),
                             later, 'learned_boxed')
        written = json.loads(learned.path.read_text())
        self.assertEqual(written['status'], P.COMPLETE_STATUS['learned_boxed'], written['errors'])
        P.validate_type1_report(written, contract=self.world.contract,
            contract_digest=self.world.digest, mode='learned_boxed', duration_s=2)
        self.assertFalse(written['output_approval_granted_here'])
        self.assertEqual(written['backend_kind'], 'injected_file_only_mock')
        self.assert_all_ports_stopped(learned)


if __name__ == '__main__':
    unittest.main()
