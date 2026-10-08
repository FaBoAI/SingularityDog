"""Synthetic file-only fixtures: no robot, device, native library, Torch or network."""
import argparse
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as LP
from experiments.four_bus_diagnostic import test_model_bridge as mb
from experiments.four_bus_diagnostic import model_bridge
from experiments.four_bus_type1 import type1_profile as t

IDS = tuple(range(1, 13))


def write(path, document):
    raw = json.dumps(document, indent=2, sort_keys=True, allow_nan=False)+'\n'
    path.write_text(raw)
    return {'path': str(path), 'sha256': hashlib.sha256(raw.encode()).hexdigest()}


def write_capture(root, name, capture, document=None):
    document = capture.document() if document is None else document
    raw = ''.join(json.dumps(e, sort_keys=True, allow_nan=False)+'\n' for e in capture.events)
    events = root/(name+'-events.jsonl')
    events.write_text(raw)
    events_ref = {'path': str(events), 'sha256': hashlib.sha256(raw.encode()).hexdigest()}
    document['trace'] = {**events_ref, 'bytes': len(raw.encode()), 'events': raw.count('\n'),
                         'complete': True, 'errors': []}
    return write(root/(name+'-capture.json'), document), events_ref, document


def geometry_for(plan, *, duration=2):
    data = LP.template(schema=LP.SCHEMA_V3)
    data.update(duration_s=float(duration), startup_duration_s=.5, stop_duration_s=.5, policy_ramp_s=.5,
        policy_weight=.005, hard_cycle_ms=20, max_sample_age_ms=20, max_sample_gap_ms=21,
        voltage_min_v=35, voltage_max_v=42, request_gap_us=890, request_window=3,
        diagnostic_timing_acceptance=(LP.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER if duration == 2
                                      else LP.SUPPORTED_POLICY_PROBE_10S_AFTER_2S),
        native_phase_pair=True, native_target_fk_cache=True, model_backend=LP.SCALAR_BACKEND,
        local_characterization=LP.LOCAL_RELATIVE_SUPPORTED, watchdog_review_policy=LP.COMMAND_LOSS_ONLY_SUPPORTED,
        voltage_overlap=True, voltage_pipeline=True, prepare_voltage_before_feedback_publication=True,
        startup_damping_duration_s=.08, post_reply_deadline_policy=copy.deepcopy(t.POST_REPLY_POLICY),
        preauthorized_native_boxed_sequence=True, approved_for_supported_policy_output=True, blockers=[],
        assembly_id='SYNTHETIC-assembly', boot_id='11111111-2222-3333-4444-555555555555',
        motor_power_epoch='SYNTHETIC-old-two-bus-epoch',
        review={'reviewer': 'SYNTHETIC reviewer', 'reviewed_at': '2026-10-08T00:00:00+00:00',
                'decision': 'APPROVED_SUPPORTED_CHARACTERIZATION', 'rationale': 'SYNTHETIC'})
    data['start_pose_bounds'] = {}
    for mid in LP.IDS:
        axis, q = plan['axes'][mid], plan['provenance']['model_rad_by_id'][mid]
        data['axes'][mid] = dict(uid=axis['uid'], sign=axis['sign'], offset_rad=axis['nominal_offset_rad'],
            uncertainty_rad=None, physical_lower_rad=q-math.radians(3), physical_upper_rad=q+math.radians(3),
            kp=3., kd=.15, max_command_velocity_rad_s=math.radians(1),
            max_command_acceleration_rad_s2=math.radians(5), max_tracking_error_rad=math.radians(2),
            max_measured_velocity_rad_s=.35, max_measured_torque_nm=1., max_temperature_c=45.,
            max_estimated_pd_torque_nm=.1, max_displacement_from_start_rad=math.radians(1))
        data['start_pose_bounds'][mid] = [q-math.radians(.5), q+math.radians(.5)]
    data['artifacts'] = {name: {'path': '/synthetic/'+name, 'sha256': 'a'*64} for name in LP.artifact_names(data)}
    return data


def stop_proxy(capture_ref, by_port, start, *, cycles=501):
    records = [{'cycle': i, 'release_ns': start+i*20_000_000, 'cycle_end_ns': start+i*20_000_000+19_000_000,
                'completed': True, 'actual_request_count': 28} for i in range(cycles)]
    workers = {port: {'cpu_mask': [i], 'timer_slack_ns': 1000, 'file_only_mock_readback': False}
               for i, port in enumerate(t.PORTS)}
    workers['imu'] = {'cpu_mask': [0, 1, 2, 3], 'timer_slack_ns': 1000, 'file_only_mock_readback': False}
    measurement = {'schema': t.STOP_PROXY_PIPELINE_SCHEMA, 'status': t.STOP_PROXY_STATUS, 'cycles': cycles,
        'completed_cycles': cycles, 'primary_error': None, 'failure_retained': False,
        'physical_cutoff_required': False, 'per_cycle_requests': 28, 'request_gap_ns': 900_000,
        'request_window': 3, 'absolute_deadline_ns': 20_000_000, 'request_schedule': t.STOP_PROXY_SCHEDULE,
        'boundary_current_checks_selected': True, 'final_gate_input_identity_selected': True,
        'output_allowed': False, 'motor_enable_sent': False, 'learned_targets_sent': False,
        'positive_gain_sent': False, 'physical_groups': copy.deepcopy(by_port), 'worker_settings': workers,
        'operating_settings': {'main_cpu_mask': [4], 'nice': -10, 'timer_slack_ns': 1000,
            'switch_interval_s': 9.999999999999999e-05, 'single_thread_math_verified': True,
            'power_scope_verified': True, 'gc_deferred_during_cycles': True},
        'records': records, 'all_original_workers_joined_monotonic_ns': start+cycles*20_000_000+500_000_000,
        'cleanup': {port: {'complete': True, 'confirmed_ids': list(ids), 'ambiguous_ids': [],
                           'physical_cutoff_required': False, 'fault_by_id': {str(m): 0 for m in ids}}
                    for port, ids in by_port.items()}}
    return {'schema': t.STOP_PROXY_SCHEMA, 'status': t.STOP_PROXY_STATUS, 'errors': [], 'failure_retained': False,
            'source_and_input_pins_unchanged': True, 'physical_post_trial_observation': None,
            'output_allowed': False, 'approved_for_runtime': False, 'live_type1_qualified': False,
            'motor_enable_sent': False, 'learned_targets_sent': False, 'timing_admission_eligible': False,
            'input_sha256': {capture_ref['path']: capture_ref['sha256']}, 'measurement': measurement}


def type1_report(contract, mode, duration, first, finished):
    learned = mode == 'learned_boxed'
    return {'schema': t.REPORT_SCHEMA, 'status': t.COMPLETE_STATUS[mode], 'mode': mode, 'duration_s': duration,
        'contract_sha256': t.contract_sha256(contract), 'profile_canonical_sha256': 'c'*64,
        'conditions_sha256': 'd'*64, 'boot_id': contract['boot_id'],
        'motor_power_epoch': contract['motor_power_epoch'],
        'source_manifest_sha256': contract['source_manifest']['sha256'],
        'topology_by_port': copy.deepcopy(contract['topology_by_port']), 'pacing': copy.deepcopy(contract['pacing']),
        'errors': [], 'failure_retained': False, 'completed_cycles': duration*50-5, 'all_cycles_passed': True,
        'first_release_monotonic_ns': first, 'motor_enable_sent': True, 'type1_sent': True,
        'positive_gain_sent': learned, 'learned_targets_attempted': learned,
        'terminal_stop': {'stop_confirmed': True, 'confirmed_ids': list(IDS), 'unconfirmed_ids': [],
                          'ambiguous_ids': [], 'fault_by_id': {str(m): 0 for m in IDS},
                          'physical_cutoff_required': False, 'finished_monotonic_ns': finished},
        'physical_cutoff_required': False, 'restoration_complete': True, 'physical_post_trial_observation': None}


CONDITIONS = dict(motor_40v_on=True, box_supports_body=True, four_feet_touch_floor=True,
    all12_local_plus_minus3deg_clear=True, hands_off=True, immediate_40v_cutoff=True,
    other_drive_tools_stopped=True, box_will_remain=True, box_removal_allowed=False,
    load_transfer_allowed=False, standing_allowed=False, walking_allowed=False)


class World:
    """Pinned synthetic lineage/current captures, model inputs and reviewed geometry."""
    def __init__(self, root):
        self.root = root
        fixture = mb.Fixture()
        self.lineage, self.lineage_events, _ = write_capture(root, 'lineage', fixture.capture, fixture.topology)
        refs = {name: write(root/(name+'.json'), getattr(fixture, name)) for name in ('mount', 'bias')}
        refs['calibration'] = write(root/'calibration.json', fixture.nominal)
        fixture.profile['artifacts'] = dict(refs)
        refs['model_profile'] = write(root/'model-profile.json', fixture.profile)
        fixture.source['original_model_profile'] = refs['model_profile']
        refs['source_binding'] = write(root/'source-binding.json', fixture.source)
        self.refs = refs
        current = mb.Capture()
        current.clock_ns = 50_000_000_000
        self.current, self.current_events, self.current_document = write_capture(root, 'current', current)
        self.plan = model_bridge.prepare_model_plan(**self.envelopes(self.lineage, self.lineage_events))
        self.geometry_doc = geometry_for(self.plan)
        self.geometry = write(root/'geometry.json', self.geometry_doc)

    def envelopes(self, capture, events):
        load = lambda ref: {'reference': ref, 'raw_json': Path(ref['path']).read_text()}
        result = {name: load(self.refs[name]) for name in ('calibration', 'model_profile', 'source_binding',
                                                            'mount', 'bias')}
        result['topology'] = load(capture)
        result['events'] = {'reference': events, 'raw_jsonl': Path(events['path']).read_text()}
        return result

    def lineage_refs(self):
        return {'topology': self.lineage, 'events': self.lineage_events,
                **{name: self.refs[name] for name in ('calibration', 'model_profile', 'source_binding', 'mount', 'bias')}}

    def contract(self, geometry=None):
        return t.build_contract(copy.deepcopy(self.plan), self.lineage_refs(),
                                geometry or copy.deepcopy(self.geometry_doc), self.geometry)

    def args(self, mode, duration, output, **extra):
        values = dict(mode=mode, duration=duration, power_epoch='explicit-current-label', output=str(output),
            prepare=False, accel_hypothesis=None, accel_hypothesis_sha256=None, stop_proxy_report=None,
            stop_proxy_report_sha256=None, predecessor_report=None, predecessor_report_sha256=None,
            topology=self.lineage['path'], topology_sha256=self.lineage['sha256'],
            events=self.lineage_events['path'], events_sha256=self.lineage_events['sha256'],
            current_topology=self.current['path'], current_topology_sha256=self.current['sha256'],
            current_events=self.current_events['path'], current_events_sha256=self.current_events['sha256'],
            axis_geometry_profile=self.geometry['path'], axis_geometry_profile_sha256=self.geometry['sha256'])
        for name in ('calibration', 'model_profile', 'source_binding', 'mount', 'bias'):
            values[name], values[name+'_sha256'] = self.refs[name]['path'], self.refs[name]['sha256']
        values.update(extra)
        return argparse.Namespace(**values)

    def capture_evidence(self):
        current = model_bridge.prepare_model_plan(**self.envelopes(self.current, self.current_events))
        return {'topology': self.current, 'events': self.current_events,
                'started_monotonic_ns': self.current_document['started_monotonic_ns'],
                'finished_monotonic_ns': self.current_document['finished_monotonic_ns'],
                'model_rad_by_id': dict(current['provenance']['model_rad_by_id']),
                'fixed_offset_rad_by_id': {k: v['fixed_offset_rad'] for k, v in current['axes'].items()},
                'boot_id': 'current-boot', 'motor_power_epoch': 'explicit-current-label'}


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(tempfile.mkdtemp()).resolve()
        cls.world = World(cls.root)
        cls.base_contract = cls.world.contract()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root)

    def output(self, name):
        directory = Path(tempfile.mkdtemp()).resolve()
        self.addCleanup(shutil.rmtree, directory)
        return directory/name

    def zero_gain_profile(self):
        contract = copy.deepcopy(self.base_contract)
        diag = {'report': {'path': str(self.root/'diag.json'), 'sha256': 'e'*64}, 'pinned_capture': self.world.current,
                'completed_cycles': 501, 'first_release_monotonic_ns': 60_000_000_000,
                'workers_joined_monotonic_ns': 71_000_000_000}
        evidence = {'current_capture': self.world.capture_evidence(), 'stop_proxy_diagnostic': diag, 'predecessor': None}
        return t.assemble_profile(contract, 'zero_gain_timing', 2, evidence, prepared_at='2026-10-08T23:00:00+09:00')

    def conditions(self, profile, **changes):
        values = dict(CONDITIONS)
        values.update(changes)
        return t.conditions_record(user_statement='SYNTHETIC direct statement', user_reply_id=None,
            boot_id=profile['contract']['boot_id'], motor_power_epoch=profile['contract']['motor_power_epoch'],
            contract_sha256=profile['contract_sha256'], authorized_modes=['zero_gain_timing', 'learned_boxed'],
            authorized_durations_s=[2, 10], record_written_at='2026-10-08T23:01:00+09:00', **values)


class ContractTests(Base):
    def test_contract_reuses_lp_boxed_validators_and_records_fixed_pacing(self):
        with patch.object(LP, '_preauthorized_boxed_axis_caps', wraps=LP._preauthorized_boxed_axis_caps) as caps, \
             patch.object(LP, '_preauthorized_ordinary_boxed_sequence_scope',
                          wraps=LP._preauthorized_ordinary_boxed_sequence_scope) as scope:
            contract = self.world.contract()
        self.assertGreaterEqual(caps.call_count, 2)
        views = [call.args[0] for call in scope.call_args_list
                 if call.args[0].get('preauthorized_ordinary_boxed_sequence') is True]
        self.assertEqual(len(views), 1)  # Others are LP's own structure checks of the geometry.
        self.assertEqual(views[0]['request_gap_us'], 900)
        self.assertFalse(views[0]['native_phase_pair'])
        self.assertEqual(contract['pacing']['request_gap_us'], 900)
        self.assertEqual(contract['pacing']['release_spin_us'], 500)
        self.assertEqual(contract['pacing']['per_cycle_requests'], 28)
        self.assertEqual(contract['enable_order_ids'], [1, 4, 7, 10, 2, 5, 8, 11, 3, 6, 9, 12])
        self.assertEqual(contract['axis_geometry']['route'], 'native890')
        self.assertIsNone(contract['axis_geometry']['today_two_bus_boxed_profile'])
        self.assertFalse(contract['axis_geometry']['two_bus_timing_or_permission_reused'])
        self.assertEqual(contract['boot_id'], 'current-boot')
        for row in contract['axes'].values():
            self.assertEqual((row['kp'], row['kd']), (3., .15))
            self.assertLessEqual(row['physical_upper_rad']-row['physical_lower_rad'], 2*math.radians(3)+1e-12)
            self.assertAlmostEqual(row['lower_rad'], row['physical_lower_rad']+LP.LOCAL_NUMERICAL_MARGIN_RAD)
            self.assertLess(row['raw_lower_rad'], row['raw_upper_rad'])

    def test_json_round_trip_and_sha_excludes_mode_duration(self):
        contract = json.loads(json.dumps(self.base_contract, allow_nan=False))
        self.assertEqual(t.validate_contract(contract), self.base_contract)
        profile = self.zero_gain_profile()
        self.assertEqual(profile['contract_sha256'], t.contract_sha256(self.base_contract))
        self.assertNotIn('duration_s', profile['contract']); self.assertNotIn('mode', profile['contract'])

    def geometry(self, change):
        geometry = copy.deepcopy(self.world.geometry_doc)
        change(geometry)
        try:  # Keep LP's own artifact shape valid so the four-bus check itself is exercised.
            geometry['artifacts'] = {name: {'path': '/synthetic/'+name, 'sha256': 'a'*64}
                                     for name in LP.artifact_names(geometry)}
        except ValueError:
            pass
        return geometry

    def test_reviewed_geometry_rejections(self):
        cases = {
            'kp above boxed cap': lambda g: g['axes']['3'].update(kp=3.01),
            'kd above boxed cap': lambda g: g['axes']['3'].update(kd=.16),
            'displacement above 1deg': lambda g: g['axes']['5'].update(max_displacement_from_start_rad=math.radians(1.1)),
            'unapproved': lambda g: g.update(approved_for_supported_policy_output=False),
            'wrong review': lambda g: g['review'].update(decision='ACCEPT_SOMETHING_ELSE'),
            'uid differs': lambda g: g['axes']['7'].update(uid='ff'*8),
            'sign differs': lambda g: g['axes']['8'].update(sign=-1),
            'offset differs': lambda g: g['axes']['9'].update(offset_rad=.001),
            'wide start': lambda g: g['start_pose_bounds']['2'].__setitem__(1, g['start_pose_bounds']['2'][0]+math.radians(1.2)),
            'lineage pose outside start': lambda g: g['start_pose_bounds'].update(
                {'4': [g['start_pose_bounds']['4'][0]+math.radians(.6), g['start_pose_bounds']['4'][1]+math.radians(.2)]}),
            'extra field': lambda g: g.update(unreviewed_field=True),
        }
        for name, change in cases.items():
            with self.subTest(name), self.assertRaises(ValueError):
                self.world.contract(self.geometry(change))
        with self.assertRaisesRegex(ValueError, 'native or ordinary boxed'):
            self.world.contract(self.geometry(lambda g: g.update(preauthorized_native_boxed_sequence=False)))
        with self.assertRaisesRegex(ValueError, 'reviewed local boxed 2s/10s'):
            self.world.contract(self.geometry(lambda g: g.update(
                duration_s=20., diagnostic_timing_acceptance=LP.SUPPORTED_POLICY_PROBE_20S_AFTER_10S)))

    def test_contract_tampering_rejected(self):
        cases = {
            'gap890': lambda c: c['pacing'].update(request_gap_us=890),
            'spin200': lambda c: c['pacing'].update(release_spin_us=200),
            'weight': lambda c: c['timing'].update(policy_weight=.01),
            'post reply': lambda c: c['timing']['post_reply_deadline_policy'].update(max_lateness_ms=2.),
            'voltage': lambda c: c['timing'].update(voltage_max_v=43),
            'raw bounds': lambda c: c['axes']['1'].update(raw_upper_rad=c['axes']['1']['raw_upper_rad']+.01),
            'physical widened': lambda c: c['axes']['2'].update(physical_upper_rad=c['axes']['2']['physical_upper_rad']+.01),
            'kp': lambda c: c['axes']['6'].update(kp=3.5),
            'topology': lambda c: c['topology_by_port'].update(port0=c['topology_by_port']['port1'],
                                                               port1=c['topology_by_port']['port0']),
            'plan': lambda c: c['model_plan']['axes']['1'].update(fixed_offset_rad=.5),
            'source': lambda c: c['source_manifest'].update(sha256='0'*64),
            'modes': lambda c: c['authorized_modes'].append('walking'),
            'extra': lambda c: c.update(extra=True),
        }
        for name, change in cases.items():
            contract = copy.deepcopy(self.base_contract)
            change(contract)
            with self.subTest(name), self.assertRaises(ValueError):
                t.validate_contract(contract)


class WindowRebindingTests(Base):
    def shifted(self, degrees):
        geometry = copy.deepcopy(self.world.geometry_doc)
        delta = math.radians(degrees)
        for key in LP.IDS:
            axis = geometry['axes'][key]
            axis['physical_lower_rad'] += delta; axis['physical_upper_rad'] += delta
            geometry['start_pose_bounds'][key] = [v + delta for v in geometry['start_pose_bounds'][key]]
        return geometry

    def build(self, geometry, rebind):
        w = self.world
        return t.build_contract(copy.deepcopy(w.plan), w.lineage_refs(), geometry, w.geometry, rebind_window=rebind)

    def test_moved_pose_is_rejected_by_default_and_rebound_only_explicitly(self):
        geometry = self.shifted(10.)
        with self.assertRaisesRegex(ValueError, 'Intersected local bounds are empty'):
            self.build(copy.deepcopy(geometry), False)
        contract = self.build(copy.deepcopy(geometry), True)
        record = contract['axis_geometry']['window_rebound_to_current_capture']
        self.assertFalse(record['physical_clearance_inferred'])
        self.assertEqual(record['requires_direct_human'], 'all12_local_plus_minus3deg_clear')
        self.assertAlmostEqual(abs(record['max_abs_centre_shift_deg']), 10., delta=1.)
        plan = self.world.plan
        for key in LP.IDS:
            row, q = contract['axes'][key], plan['provenance']['model_rad_by_id'][key]
            self.assertEqual([row['reviewed_physical_lower_rad'], row['reviewed_physical_upper_rad']],
                             plan['axes'][key]['local_bounds_rad'])
            lo, hi = contract['start_pose_bounds'][key]
            self.assertTrue(lo < q < hi); self.assertAlmostEqual(hi - lo, math.radians(1.))
            for name in ('kp', 'kd', 'max_displacement_from_start_rad', 'max_estimated_pd_torque_nm'):
                self.assertEqual(row[name], self.world.geometry_doc['axes'][key][name])
        self.assertIs(t.validate_contract(contract), contract)

    def test_rebinding_record_tamper_and_non_bool_selection_rejected(self):
        contract = self.build(self.shifted(10.), True)
        bad = copy.deepcopy(contract)
        bad['axis_geometry']['window_rebound_to_current_capture']['physical_clearance_inferred'] = True
        with self.assertRaisesRegex(ValueError, 'rebinding record'):
            t.validate_contract(bad)
        bad = copy.deepcopy(contract)
        bad['start_pose_bounds']['1'] = [v + 1e-3 for v in bad['start_pose_bounds']['1']]
        with self.assertRaises(ValueError):
            t.validate_contract(bad)
        with self.assertRaisesRegex(ValueError, 'rebinding boolean'):
            self.build(self.shifted(10.), 1)

    def test_unshifted_rebinding_keeps_caps_and_validates(self):
        contract = self.build(copy.deepcopy(self.world.geometry_doc), True)
        self.assertIs(t.validate_contract(contract), contract)


class EvidenceTests(Base):
    def evidence(self, **changes):
        evidence = {'current_capture': self.world.capture_evidence(), 'stop_proxy_diagnostic': None, 'predecessor': None}
        evidence.update(changes)
        return evidence

    def predecessor(self, mode='zero_gain_timing', duration=2, first=20_000_000_000, finished=30_000_000_000):
        report = type1_report(self.base_contract, mode, duration, first, finished)
        summary = t.validate_type1_report(report, contract=self.base_contract,
            contract_digest=t.contract_sha256(self.base_contract), mode=mode)
        return {'report': {'path': str(self.root/'pred.json'), 'sha256': 'f'*64}, **summary}

    def diag(self, first, joined=None, pinned=None):
        return {'report': {'path': str(self.root/'diag.json'), 'sha256': 'e'*64},
                'pinned_capture': pinned or self.world.current, 'completed_cycles': 501,
                'first_release_monotonic_ns': first, 'workers_joined_monotonic_ns': joined or first+11_000_000_000}

    def assemble(self, mode, duration, evidence):
        return t.assemble_profile(copy.deepcopy(self.base_contract), mode, duration, evidence, prepared_at='now')

    def test_ladder_requirements(self):
        self.assertEqual(t.required_evidence('zero_gain_timing', 2), (None, True))
        self.assertEqual(t.required_evidence('learned_boxed', 2), (('zero_gain_timing', None), False))
        self.assertEqual(t.required_evidence('learned_boxed', 10), (('learned_boxed', 2), False))
        self.assertEqual(t.required_evidence('learned_boxed', 20), (('learned_boxed', 10), True))
        for mode, duration in (('learned_boxed', 30), ('walking', 2), ('zero_gain_timing', 5)):
            with self.subTest(mode=mode, duration=duration), self.assertRaises(ValueError):
                t.required_evidence(mode, duration)
        self.assemble('learned_boxed', 2, self.evidence(predecessor=self.predecessor()))
        self.assemble('learned_boxed', 10, self.evidence(predecessor=self.predecessor('learned_boxed', 2)))
        self.assemble('learned_boxed', 20, self.evidence(predecessor=self.predecessor('learned_boxed', 10),
                                                         stop_proxy_diagnostic=self.diag(35_000_000_000)))
        self.assemble('zero_gain_timing', 10, self.evidence(stop_proxy_diagnostic=self.diag(35_000_000_000)))

    def test_missing_or_misordered_evidence_rejected(self):
        cases = {
            'learned2 without zero gain': ('learned_boxed', 2, self.evidence()),
            'learned2 after learned2': ('learned_boxed', 2, self.evidence(predecessor=self.predecessor('learned_boxed', 2))),
            'learned10 after zero gain': ('learned_boxed', 10, self.evidence(predecessor=self.predecessor())),
            'learned20 without new diagnostic': ('learned_boxed', 20, self.evidence(
                predecessor=self.predecessor('learned_boxed', 10))),
            'learned20 diagnostic before 10s run': ('learned_boxed', 20, self.evidence(
                predecessor=self.predecessor('learned_boxed', 10), stop_proxy_diagnostic=self.diag(25_000_000_000))),
            'zero gain without diagnostic': ('zero_gain_timing', 2, self.evidence()),
            'zero gain with predecessor': ('zero_gain_timing', 2, self.evidence(
                predecessor=self.predecessor(), stop_proxy_diagnostic=self.diag(35_000_000_000))),
            'capture before predecessor STOP': ('learned_boxed', 2, self.evidence(
                predecessor=self.predecessor(finished=60_000_000_000))),
            'capture before diagnostic on other capture': ('zero_gain_timing', 2, self.evidence(
                stop_proxy_diagnostic=self.diag(55_000_000_000, pinned=self.world.lineage))),
            'diagnostic on foreign capture': ('zero_gain_timing', 2, self.evidence(
                stop_proxy_diagnostic=self.diag(35_000_000_000, pinned={'path': '/other/capture.json', 'sha256': '1'*64}))),
            'twenty without braking reserve': ('zero_gain_timing', 2, self.evidence(stop_proxy_diagnostic=self.diag(1))),
        }
        contract = copy.deepcopy(self.base_contract)
        contract['ramps']['startup_duration_s'] = 1.
        for name, (mode, duration, evidence) in cases.items():
            with self.subTest(name), self.assertRaises(ValueError):
                if name.startswith('twenty'):
                    t.assemble_profile(copy.deepcopy(contract), mode, duration, evidence, prepared_at='now')
                else:
                    self.assemble(mode, duration, evidence)

    def test_current_pose_outside_start_rejected(self):
        evidence = self.evidence(stop_proxy_diagnostic=self.diag(35_000_000_000))
        evidence['current_capture']['model_rad_by_id']['5'] += math.radians(.6)
        with self.assertRaises(ValueError):
            self.assemble('zero_gain_timing', 2, evidence)

    def test_stop_proxy_report_validator(self):
        by_port = self.base_contract['topology_by_port']
        report = stop_proxy(self.world.lineage, by_port, 2_000_000_000)
        summary = t.validate_stop_proxy_report(report, capture_refs=[self.world.lineage, self.world.current],
                                               topology_by_port=by_port)
        self.assertEqual(summary['completed_cycles'], 501)
        self.assertEqual(summary['pinned_capture'], self.world.lineage)
        cases = {
            '500 cycles': lambda r: r.update(stop_proxy(self.world.lineage, by_port, 2_000_000_000, cycles=500)),
            'schema': lambda r: r.update(schema='singularitydog.four-bus-stop-proxy-pipeline.v1'),
            'status': lambda r: r.update(status='ABORTED'),
            'motor enable': lambda r: r.update(motor_enable_sent=True),
            'positive gain': lambda r: r['measurement'].update(positive_gain_sent=True),
            'mock readback': lambda r: r['measurement']['worker_settings']['port2'].update(file_only_mock_readback=True),
            'cpu placement': lambda r: r['measurement']['worker_settings']['port1'].update(cpu_mask=[3]),
            'gap890': lambda r: r['measurement'].update(request_gap_ns=890_000),
            'combined acquisition': lambda r: r['measurement'].update(request_schedule='combined'),
            'ambiguous STOP': lambda r: r['measurement']['cleanup']['port3'].update(ambiguous_ids=[2]),
            'fault': lambda r: r['measurement']['cleanup']['port0']['fault_by_id'].update({'7': 1}),
            'incomplete cycle': lambda r: r['measurement']['records'][250].update(completed=False),
            'not bound': lambda r: r.update(input_sha256={self.world.lineage['path']: '0'*64}),
            'topology': lambda r: r['measurement'].update(physical_groups={**by_port, 'port0': by_port['port1']}),
        }
        for name, change in cases.items():
            value = copy.deepcopy(report)
            change(value)
            with self.subTest(name), self.assertRaises(ValueError):
                t.validate_stop_proxy_report(value, capture_refs=[self.world.lineage], topology_by_port=by_port)
        with self.assertRaises(ValueError):
            t.validate_stop_proxy_report(report, capture_refs=[self.world.lineage], topology_by_port=by_port,
                                         after_ns=3_000_000_000)

    def test_type1_report_validator(self):
        digest = t.contract_sha256(self.base_contract)
        report = type1_report(self.base_contract, 'learned_boxed', 2, 10, 20)
        self.assertEqual(t.validate_type1_report(report, contract=self.base_contract, contract_digest=digest,
            mode='learned_boxed', duration_s=2)['terminal_stop_finished_monotonic_ns'], 20)
        cases = {
            'status': lambda r: r.update(status=t.STOP_UNCONFIRMED_STATUS),
            'missing key': lambda r: r.pop('restoration_complete'),
            'contract': lambda r: r.update(contract_sha256='0'*64),
            'boot': lambda r: r.update(boot_id='other-boot'),
            'epoch': lambda r: r.update(motor_power_epoch='other-epoch'),
            'source': lambda r: r.update(source_manifest_sha256='0'*64),
            'pacing': lambda r: r['pacing'].update(request_gap_us=890),
            'positive gain not truthful': lambda r: r.update(positive_gain_sent=False),
            'learned not attempted': lambda r: r.update(learned_targets_attempted=False),
            'stop unconfirmed': lambda r: r['terminal_stop'].update(stop_confirmed=False),
            'stop ambiguous': lambda r: r['terminal_stop'].update(ambiguous_ids=[6]),
            'stop missing id': lambda r: r['terminal_stop'].update(confirmed_ids=list(range(1, 12))),
            'fault': lambda r: r['terminal_stop']['fault_by_id'].update({'4': 2}),
            'cutoff': lambda r: r.update(physical_cutoff_required=True),
            'cycles': lambda r: r.update(completed_cycles=0),
            'observation inferred': lambda r: r.update(physical_post_trial_observation={'ok': True}),
            'errors': lambda r: r.update(errors=['late']),
        }
        for name, change in cases.items():
            value = copy.deepcopy(report)
            change(value)
            with self.subTest(name), self.assertRaises(ValueError):
                t.validate_type1_report(value, contract=self.base_contract, contract_digest=digest,
                                        mode='learned_boxed', duration_s=2)
        with self.assertRaises(ValueError):
            t.validate_type1_report(report, contract=self.base_contract, contract_digest=digest,
                                    mode='learned_boxed', duration_s=10)
        zero = type1_report(self.base_contract, 'zero_gain_timing', 2, 10, 20)
        t.validate_type1_report(zero, contract=self.base_contract, contract_digest=digest, mode='zero_gain_timing')
        zero['positive_gain_sent'] = True
        with self.assertRaises(ValueError):
            t.validate_type1_report(zero, contract=self.base_contract, contract_digest=digest, mode='zero_gain_timing')


class AdmissionTests(Base):
    def test_conditions_only_from_explicit_arguments(self):
        profile = self.zero_gain_profile()
        record = self.conditions(profile)
        self.assertEqual(record['source'], 'direct_current_user_reply')
        self.assertIs(record['physical_condition_inferred'], False)
        values = dict(CONDITIONS); values.pop('hands_off')
        with self.assertRaises(TypeError):
            t.conditions_record(user_statement='x', user_reply_id=None, boot_id='b', motor_power_epoch='e',
                contract_sha256='a'*64, authorized_modes=['zero_gain_timing'], authorized_durations_s=[2],
                record_written_at='now', **values)
        for name, value in (('hands_off', False), ('hands_off', 1), ('box_supports_body', None),
                            ('walking_allowed', True), ('load_transfer_allowed', True), ('box_removal_allowed', True)):
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                self.conditions(profile, **{name: value})
        with self.assertRaises(ValueError):
            t.conditions_record(user_statement=' ', user_reply_id=None, boot_id='b', motor_power_epoch='e',
                contract_sha256='a'*64, authorized_modes=['zero_gain_timing'], authorized_durations_s=[2],
                record_written_at='now', **CONDITIONS)

    def test_admit_returns_immutable_bound_object(self):
        profile = self.zero_gain_profile()
        admitted = t.admit(profile, self.conditions(profile), expected_boot='current-boot',
                           expected_power_epoch='explicit-current-label')
        self.assertIs(t.verify_admitted(admitted), admitted)
        self.assertEqual(admitted.mode, 'zero_gain_timing')
        self.assertEqual(admitted.contract_sha256, profile['contract_sha256'])
        binding = admitted.report_binding()
        self.assertEqual(binding['pacing']['request_gap_us'], 900)
        self.assertEqual(admitted.plain_profile(), profile)
        with self.assertRaises(AttributeError):
            admitted.mode = 'learned_boxed'
        with self.assertRaises(TypeError):
            admitted.profile['mode'] = 'learned_boxed'
        with self.assertRaises(TypeError):
            admitted.contract['axes']['1']['kp'] = 30.
        with self.assertRaises(ValueError):
            t.Admitted(object(), profile, self.conditions(profile), {})
        with self.assertRaises(ValueError):
            t.verify_admitted(copy.copy(profile))

    def test_admission_rejections(self):
        profile = self.zero_gain_profile()
        record = self.conditions(profile)
        cases = {
            'boot': (lambda p, c: c.update(boot_id='other-boot'), {}),
            'epoch': (lambda p, c: c.update(motor_power_epoch='other'), {}),
            'contract': (lambda p, c: c.update(contract_sha256='0'*64), {}),
            'mode': (lambda p, c: c.update(authorized_modes=['learned_boxed']), {}),
            'duration': (lambda p, c: c.update(authorized_durations_s=[10]), {}),
            'synthetic': (lambda p, c: c.update(synthetic_interaction=True), {}),
            'not direct': (lambda p, c: c.update(direct_human=False), {}),
            'inferred': (lambda p, c: c.update(physical_condition_inferred=True), {}),
            'condition false': (lambda p, c: c.update(immediate_40v_cutoff=False), {}),
            'profile grants': (lambda p, c: p.update(output_allowed=True), {}),
            'profile claims sent': (lambda p, c: p.update(type1_sent=True), {}),
            'profile mode': (lambda p, c: p.update(mode='learned_boxed'), {}),
            'profile contract sha': (lambda p, c: p.update(contract_sha256='0'*64), {}),
            'live boot': (lambda p, c: None, {'expected_boot': 'new-boot'}),
            'live epoch': (lambda p, c: None, {'expected_power_epoch': 'new-epoch'}),
        }
        for name, (change, kwargs) in cases.items():
            p, c = copy.deepcopy(profile), copy.deepcopy(record)
            change(p, c)
            with self.subTest(name), self.assertRaises(ValueError):
                t.admit(p, c, **kwargs)


class FileTests(Base):
    def run_main(self, argv):
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            code = t.main(argv)
        return code, json.loads(stream.getvalue())

    def argv(self, args):
        result = ['prepare']
        for key, value in vars(args).items():
            if key == 'prepare':
                result += ['--prepare'] if value else []
            elif value is not None:
                result += ['--'+key.replace('_', '-'), str(value)]
        return result

    def test_plan_writes_nothing_and_prepare_writes_one_profile(self):
        diag = write(self.output('diag.json'), stop_proxy(self.world.current, self.base_contract['topology_by_port'],
                                                         60_000_000_000))
        output = self.output('profile.json')
        args = self.world.args('zero_gain_timing', 2, output, stop_proxy_report=diag['path'],
                               stop_proxy_report_sha256=diag['sha256'])
        with patch('ctypes.CDLL', side_effect=AssertionError('library opened')), \
             patch('serial.Serial', side_effect=AssertionError('device opened'), create=True):
            code, plan = self.run_main(self.argv(args))
        self.assertEqual((code, plan['status']), (0, 'PLAN_ONLY'))
        self.assertFalse(output.exists())
        self.assertFalse(plan['hardware_opened']); self.assertFalse(plan['output_allowed'])
        args.prepare = True
        code, result = self.run_main(self.argv(args))
        self.assertEqual((code, result['status']), (0, 'PREPARED_FILE_ONLY_PROFILE'))
        profile = t.read_json(output, result['profile']['sha256'])
        t.validate_profile(profile)
        self.assertEqual(profile['contract_sha256'], plan['contract_sha256'])
        self.assertEqual(profile['evidence']['stop_proxy_diagnostic']['pinned_capture'], self.world.current)
        code, again = self.run_main(self.argv(args))
        self.assertEqual(code, 2)
        self.assertEqual(again['status'], 'REJECTED_FILE_ONLY')
        # Direct-human record from explicit CLI flags, then file admission.
        record_path = self.output('conditions.json')
        flags = ['conditions', '--user-statement', 'SYNTHETIC statement', '--boot-id', 'current-boot',
                 '--power-epoch', 'explicit-current-label', '--contract-sha256', profile['contract_sha256'],
                 '--authorize-mode', 'zero_gain_timing', '--authorize-duration', '2', '--output', str(record_path)]
        for name, value in CONDITIONS.items():
            flags += ['--'+name.replace('_', '-'), 'true' if value else 'false']
        code, shown = self.run_main(flags)
        self.assertEqual((code, shown['status']), (0, 'PLAN_ONLY'))
        self.assertFalse(record_path.exists())
        code, recorded = self.run_main(flags+['--record'])
        self.assertEqual((code, recorded['status']), (0, 'RECORDED_DIRECT_HUMAN_CONDITIONS'))
        admitted = t.read_admitted(output, result['profile']['sha256'], record_path, recorded['written']['sha256'],
                                   expected_boot='current-boot')
        self.assertEqual(admitted.files['conditions']['sha256'], recorded['written']['sha256'])
        refused = list(flags)
        refused[refused.index('--hands-off')+1] = 'false'
        code, result = self.run_main(refused+['--record'])
        self.assertEqual(code, 2)

    def test_learned_two_seconds_from_files_needs_zero_gain_report(self):
        contract = self.base_contract
        report = write(self.output('zero.json'), type1_report(contract, 'zero_gain_timing', 2,
                                                              20_000_000_000, 30_000_000_000))
        output = self.output('profile.json')
        args = self.world.args('learned_boxed', 2, output, predecessor_report=report['path'],
                               predecessor_report_sha256=report['sha256'], prepare=True)
        result = t.prepare(args)
        self.assertEqual(result['evidence']['predecessor']['mode'], 'zero_gain_timing')
        late = write(self.output('late.json'), type1_report(contract, 'zero_gain_timing', 2,
                                                            20_000_000_000, 60_000_000_000))
        args = self.world.args('learned_boxed', 2, self.output('late-profile.json'), predecessor_report=late['path'],
                               predecessor_report_sha256=late['sha256'])
        with self.assertRaisesRegex(ValueError, 'follow the predecessor'):
            t.prepare(args)
        args = self.world.args('learned_boxed', 2, self.output('none.json'))
        with self.assertRaisesRegex(ValueError, 'Supply exactly'):
            t.prepare(args)

    def test_pin_and_epoch_mismatches_rejected(self):
        diag = write(self.output('diag.json'), stop_proxy(self.world.current, self.base_contract['topology_by_port'],
                                                         60_000_000_000))
        base = dict(stop_proxy_report=diag['path'], stop_proxy_report_sha256=diag['sha256'])
        for name, extra in (('geometry sha', {'axis_geometry_profile_sha256': '0'*64}),
                            ('epoch', {'power_epoch': 'other-epoch'}),
                            ('diagnostic bound elsewhere', {
                                'current_topology': self.world.lineage['path'],
                                'current_topology_sha256': self.world.lineage['sha256'],
                                'current_events': self.world.lineage_events['path'],
                                'current_events_sha256': self.world.lineage_events['sha256']})):
            args = self.world.args('zero_gain_timing', 2, self.output('p.json'), **{**base, **extra})
            with self.subTest(name), self.assertRaises(ValueError):
                t.prepare(args)
        by_port = self.base_contract['topology_by_port']
        for start, error in ((2_000_000_000, None), (60_000_000_000, 'follow the diagnostic')):
            lineage = write(self.output('lineage-diag.json'), stop_proxy(self.world.lineage, by_port, start))
            args = self.world.args('zero_gain_timing', 2, self.output('p.json'), stop_proxy_report=lineage['path'],
                                   stop_proxy_report_sha256=lineage['sha256'])
            if error is None:
                self.assertEqual(t.prepare(args)['evidence']['stop_proxy_diagnostic']['pinned_capture'],
                                 self.world.lineage)
            else:
                with self.assertRaisesRegex(ValueError, error):
                    t.prepare(args)


if __name__ == '__main__':
    unittest.main()
