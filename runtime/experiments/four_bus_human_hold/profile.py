"""Independent 30 Hz human-catch current hold: file-only plans, never boxed grants.

This namespace is deliberately separate from the installed R3 boxed runner.
Numerical plans and synthetic test results do not permit output. Preparation
authenticates primary source, model, raw transport and current operator evidence;
active use additionally requires the separately integrated runner source receipt.
No function here opens a device, native library, model, subprocess or terminal.
"""
import copy
import hashlib
import io
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import types
import wave
from collections import Counter

from experiments.four_bus_type1 import type1_profile as boxed
from experiments.four_bus_diagnostic import model_bridge, topology
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.motor_version_probe import version_request, decode_version
from singularitydog_hw.policy_clock import validate_control_clock

SCHEMA = 'singularitydog.four-can-human-hold30-profile.v1'
RUNTIME_SCHEMA = 'singularitydog.four-can-human-hold30-runtime.v1'
CONTRACT_SCHEMA = 'singularitydog.four-can-human-hold30-contract.v1'
CONDITIONS_SCHEMA = 'singularitydog.four-can-human-hold30-current-conditions.v1'
AUDIO_SCHEMA = 'singularitydog.four-can-human-hold30-audio.v1'
REPORT_SCHEMA = 'singularitydog.four-can-human-hold30-run.v1'
SOURCE_REVIEW_SCHEMA = 'singularitydog.four-can-human-hold30-source-review.v1'
IMPLEMENTATION_SCHEMA = 'singularitydog.four-can-human-hold30-integration.v1'
SCOPE = 'four_can_human_supported_partial_current_hold30_only'
ACCEPTANCE = 'human-supported-partial-current-hold30-audio-8s-v1'
MODES = ('human_zero_gain_timing', 'human_current_hold')
DURATIONS = {'human_zero_gain_timing': 2, 'human_current_hold': 8}
COMPLETE_STATUS = {'human_zero_gain_timing': 'COMPLETE_HUMAN_ZERO_GAIN_TIMING',
                   'human_current_hold': 'COMPLETE_HUMAN_CURRENT_HOLD'}
IDS = tuple(range(1, 13))
PORTS = ('port0', 'port1', 'port2', 'port3')
STAGES = ('brief', 'prepare_ease', 'go', 'resupport', 'abort')
SETTINGS = {'schema': 'singularitydog.human-supported-hold30.v1',
    'pose_kind': 'human_full_support', 'operator_count': 2,
    'continuous_body_catch': True, 'hands_remain_on_body': True,
    'slight_ease_max_duration_s': .5, 'cue_not_before_s': 1.2,
    'resupport_ack_window_s': .5}
CAPS = {'max_displacement_from_start_rad': math.radians(1),
    'max_measured_velocity_rad_s': .25, 'max_estimated_pd_torque_nm': .2,
    'max_command_velocity_rad_s': math.radians(1),
    'max_command_acceleration_rad_s2': math.radians(5),
    'max_measured_torque_nm': 1., 'max_temperature_c': 45.,
    'max_tracking_error_rad': math.radians(2)}
NO_GRANTS = dict(output_allowed=False, approved_for_runtime=False, admitted=False,
    eligibility=False, motor_enable_sent=False, type1_sent=False,
    positive_gain_sent=False, learned_targets_attempted=False,
    partial_load_started=False, walking_allowed=False, standing_allowed=False)
REQUIRED_CONDITIONS = ('motor_40v_on', 'four_feet_touch_floor',
    'all_axes_simultaneously_stationary', 'full_body_weight_supported',
    'continuous_body_catch', 'hands_remain_on_body', 'box_removed',
    'immediate_40v_cutoff', 'other_drive_tools_stopped',
    'all12_local_plus_minus3deg_clear', 'video_camera_fixed',
    'body_four_feet_supporting_hands_visible', 'audio_heard', 'local_terminal_visible',
    'off_power_remove_restore_rehearsal_completed', 'rehearsal_continuous_full_support',
    'no_power_operation_since_capture', 'pose_maintained_since_capture')
FORBIDDEN_CONDITIONS = ('unsupported_standing_allowed', 'walking_allowed',
    'hands_may_leave_body', 'physical_conditions_inferred', 'synthetic_interaction',
    'abnormal_noise_vibration_slip_sinking_contact')
EVIDENCE_KEYS = {'source_root', 'source_count', 'source_manifest', 'source_review',
    'model_inputs', 'stop_proxy_report', 'watchdog_report', 'watchdog_trace',
    'prior_boxed_profile', 'prior_boxed_report', 'prior_boxed_observation',
    'human_zero_gain_report', 'post_diagnostic_capture', 'post_diagnostic_events',
    'audio_manifest', 'current_conditions', 'implementation_receipt'}
_PREPARED_TOKEN = object()
_EXECUTION_PROOF_TOKEN = object()


def need(value, message):
    if not value:
        raise ValueError(message)


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _number(value, label):
    need(type(value) in (int, float) and math.isfinite(value), 'Finite ' + label + ' required')
    return float(value)


def _stamp(value, label):
    need(type(value) is int and value > 0, 'Positive monotonic ' + label + ' required')
    return value


def _text(value, label):
    need(type(value) is str and 0 < len(value) <= 4096 and value.strip() == value,
         'Explicit nonblank ' + label + ' required')
    return value


def _ref(value, label):
    return boxed._ref(value, label)


def _freeze(value):
    if type(value) is dict:
        return types.MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if type(value) is list:
        return tuple(_freeze(item) for item in value)
    return value


def _thaw(value):
    if type(value) is types.MappingProxyType:
        return {key: _thaw(item) for key, item in value.items()}
    if type(value) is tuple:
        return [_thaw(item) for item in value]
    return value


def build_contract(base_contract):
    """Pure numerical reuse only; never consumes a boxed Admitted/profile flag."""
    need(type(base_contract) is dict, 'A numerical boxed contract, not an admission, is required')
    base = boxed.validate_contract(copy.deepcopy(base_contract))
    need(base['schema'] == boxed.PROFILE_SCHEMA_30, 'Retrained 30 Hz contract required')
    clock = validate_control_clock(base['control_clock'])
    need(clock.policy_hz == 30 and clock.inference_tx_budget_ns == 20_000_000,
         'Independent 30 Hz cadence and unchanged 20 ms write budget required')
    axes = copy.deepcopy(base['axes'])
    for row in axes.values():
        row.update(CAPS, kp=6., kd=.15, offset_rad=row['nominal_offset_rad'])
    profile = dict(schema=RUNTIME_SCHEMA, scope=SCOPE, diagnostic_timing_acceptance=ACCEPTANCE,
        duration_s=8., startup_duration_s=1., startup_damping_duration_s=1., stop_duration_s=.4, policy_ramp_s=1.,
        policy_weight=0., hard_cycle_ms=clock.period_ms, period_ms=clock.period_ms,
        max_sample_age_ms=20., max_sample_gap_ms=clock.cadence_gap_ms,
        inference_tx_budget_ns=20_000_000, cycle_budget_ns=clock.cycle_budget_ns,
        reply_budget_ns=clock.cycle_budget_ns, post_reply_deadline_policy=None,
        startup_cycle_allowance=None, axes=axes, start_pose_bounds=copy.deepcopy(base['start_pose_bounds']),
        human_supported_hold=copy.deepcopy(SETTINGS), control_clock=copy.deepcopy(base['control_clock']),
        voltage_min_v=35., voltage_max_v=42., max_consecutive_20ms_misses=0,
        command=[0., 0., 0.], h_hypothesis=0., **copy.deepcopy(base['imu_limits']))
    human_supported_hold_settings(profile)
    return {'schema': CONTRACT_SCHEMA, 'scope': SCOPE, 'base_numerical_contract': base,
            'runtime_profile': profile, 'authorized_modes': list(MODES),
            'durations_by_mode': dict(DURATIONS), 'boxed_permissions_reused': False,
            'whole_processing_20ms_claimed': False, **copy.deepcopy(NO_GRANTS)}


def validate_contract(contract):
    need(type(contract) is dict and contract.get('schema') == CONTRACT_SCHEMA,
         'Independent human-hold30 contract required')
    rebuilt = build_contract(contract.get('base_numerical_contract'))
    need(contract == rebuilt, 'Human-hold30 contract differs from exact numerical construction')
    return contract


def human_supported_hold_settings(profile):
    """Exact finite caps only; copied flags never count as evidence/admission."""
    need(type(profile) is dict and profile.get('scope') == SCOPE and
         profile.get('schema') == RUNTIME_SCHEMA and profile.get('diagnostic_timing_acceptance') == ACCEPTANCE,
         'Exact new human-supported current-hold30 scope required')
    value = profile.get('human_supported_hold')
    need(type(value) is dict and set(value) == set(SETTINGS) and
         all(type(value[key]) is type(expected) and value[key] == expected
             for key, expected in SETTINGS.items()), 'Exact two-person continuous-catch settings required')
    clock = validate_control_clock(profile.get('control_clock'))
    need(clock.policy_hz == 30, 'Human hold requires explicit 30 Hz control clock')
    for key, expected in (('duration_s', 8.), ('startup_duration_s', 1.), ('startup_damping_duration_s', 1.),
        ('stop_duration_s', .4), ('policy_weight', 0.), ('hard_cycle_ms', clock.period_ms),
        ('max_sample_age_ms', 20.), ('inference_tx_budget_ns', 20_000_000),
        ('cycle_budget_ns', clock.cycle_budget_ns), ('reply_budget_ns', clock.cycle_budget_ns)):
        need(_number(profile.get(key), key) == expected, 'Exact human-hold30 ' + key + ' required')
    need(profile.get('post_reply_deadline_policy') is None and
         profile.get('startup_cycle_allowance') is None and
         not {'fixed_catch', 'fixed_catch_hold'}.intersection(profile),
         'No old timing allowance or fixed-catch scope may be transplanted')
    axes = profile.get('axes')
    need(type(axes) is dict and set(axes) == {str(i) for i in IDS}, 'All twelve axes required')
    for key, row in axes.items():
        need(type(row) is dict and _number(row.get('kp'), 'kp') == 6. and
             _number(row.get('kd'), 'kd') == .15, 'All human hold axes require Kp6/Kd0.15: ID' + key)
        for name, cap in CAPS.items():
            need(0 < _number(row.get(name), name) <= cap, 'Human hold cap exceeded: ' + name)
    brake = max(row['max_command_velocity_rad_s']/row['max_command_acceleration_rad_s2']
                for row in axes.values())
    latest_stop_s = profile['duration_s'] - brake - profile['stop_duration_s'] - 2*clock.cycle_budget_ns/1e9
    need(SETTINGS['cue_not_before_s'] + SETTINGS['slight_ease_max_duration_s'] +
         SETTINGS['resupport_ack_window_s'] + clock.period_s < latest_stop_s,
         'Easing, full-support acknowledgement and bounded fade do not fit')
    return {**SETTINGS, 'latest_stop_s': latest_stop_s}


def validate_conditions(record, contract, *, capture_sha256=None):
    """Direct statements are named, scoped facts; they are not measured contact."""
    base = contract['base_numerical_contract']
    need(type(record) is dict and record.get('schema') == CONDITIONS_SCHEMA and
         record.get('source') == 'direct_current_user_reply' and record.get('direct_human') is True,
         'Fresh direct current human condition record required')
    for name in ('user_statement', 'user_reply_id', 'record_written_at'):
        _text(record.get(name), name)
    need(record.get('contract_sha256') == canonical_sha256(contract) and
         record.get('boot_id') == base['boot_id'] and
         record.get('motor_power_epoch') == base['motor_power_epoch'],
         'Human condition record has another contract, boot or motor-power epoch')
    need(type(record.get('operator_count')) is int and record['operator_count'] == 2,
         'Exactly two current operators required')
    roles = record.get('roles')
    need(type(roles) is dict and set(roles) == {'body_catch', 'terminal_and_cutoff'} and
         all(_text(name, 'role holder') for name in roles.values()) and len(set(roles.values())) == 2,
         'Distinct continuous body-catch and terminal/cutoff role holders required')
    need(all(record.get(name) is True for name in REQUIRED_CONDITIONS) and
         all(record.get(name) is False for name in FORBIDDEN_CONDITIONS),
         'Current human full-support, audio/video/terminal/rehearsal/clearance conditions incomplete')
    need(record.get('local_clearance_rad') == math.radians(3), 'Exact current all-axis +/-3deg clearance required')
    if capture_sha256 is not None:
        need(record.get('capture_sha256') == capture_sha256,
             'Human conditions must name the fresh human-pose capture')
    return record


def build_review_plan(base_contract, conditions=None, evidence=None, *, mode='human_current_hold',
                      prepared_at=None):
    """Pure default PLAN; completeness lists are never an executable grant."""
    need(mode in MODES, 'Select one distinct human hold or zero-gain mode')
    contract = build_contract(base_contract)
    refs = {} if evidence is None else copy.deepcopy(evidence)
    need(type(refs) is dict and set(refs) <= EVIDENCE_KEYS, 'Unknown human-hold evidence item')
    blockers = ['PRIMARY_FILES_NOT_AUTHENTICATED', 'ACTIVE_RUNNER_INTEGRATION_NOT_AUTHENTICATED']
    for name in sorted(EVIDENCE_KEYS - {'human_zero_gain_report', 'post_diagnostic_capture',
                                        'post_diagnostic_events'}):
        if name not in refs:
            blockers.append('MISSING_' + name.upper())
    if mode == 'human_current_hold':
        for name in ('human_zero_gain_report', 'post_diagnostic_capture', 'post_diagnostic_events'):
            if name not in refs:
                blockers.append('MISSING_' + name.upper())
    if conditions is None:
        blockers.append('MISSING_DIRECT_CURRENT_HUMAN_CONDITIONS')
    else:
        validate_conditions(conditions, contract)
    return {'schema': SCHEMA, 'status': 'PLAN_ONLY', 'scope': SCOPE, 'mode': mode,
        'duration_s': DURATIONS[mode], 'contract': contract,
        'contract_sha256': canonical_sha256(contract), 'conditions': copy.deepcopy(conditions),
        'evidence': refs, 'prepared_at': prepared_at, 'blockers': blockers,
        'old_boxed_output_permission_reused': False, 'physical_contact_or_load_inferred': False,
        **copy.deepcopy(NO_GRANTS)}


class Pins:
    """Primary pinned regular bytes; a second pass detects replacement/change."""
    def __init__(self):
        self.values = {}

    def read(self, path, digest):
        path = str(Path(path))
        _ref({'path': path, 'sha256': digest}, 'file')
        need(path not in self.values or self.values[path] == digest, 'Conflicting pin: ' + path)
        raw = topology.read_pinned(path, digest)
        self.values[path] = digest
        return raw

    def raw(self, ref):
        ref = _ref(ref, 'artifact')
        return self.read(ref['path'], ref['sha256'])

    def json(self, ref, digest=None):
        if digest is not None:
            ref = {'path': str(ref), 'sha256': digest}
        raw = self.raw(ref)
        need(len(raw) <= 128*1024*1024, 'Bounded human-hold JSON required')
        return json.loads(raw, object_pairs_hook=topology.strict_pairs,
                          parse_constant=topology.bad_constant)

    def envelope(self, ref, *, events=False):
        return {'reference': _ref(ref, 'model input'),
                'raw_jsonl' if events else 'raw_json': self.raw(ref).decode('utf-8')}

    def verify(self):
        for path, digest in tuple(self.values.items()):
            self.read(path, digest)


def verify_manifest(root, manifest_ref, count, pins):
    """Original complete-kit primary inventory check plus human-entry origins."""
    from experiments.four_bus_diagnostic import foreground as fg
    manifest = pins.json(manifest_ref)
    fg.verify_manifest(root, manifest, count, pins)
    fg.verify_origins(root, manifest)
    runtime = Path(root)/'runtime'
    import sys
    for name, module in tuple(sys.modules.items()):
        if not (name.startswith('experiments.four_bus_human_hold') or
                name.startswith('experiments.four_bus_type1')):
            continue
        filename = getattr(module, '__file__', None)
        if filename is None:
            continue
        relative = name.replace('.', '/')
        expected = runtime/(relative + ('/__init__.py' if hasattr(module, '__path__') else '.py'))
        actual = Path(filename).absolute()
        need(actual == expected and str(actual.relative_to(root)) in manifest['files'],
             'Human/Type1 imported origin differs from complete selected kit: ' + name)
    for name in ('profile.py', 'supervision.py', 'runner.py', 'type1_runner.py'):
        need('runtime/experiments/four_bus_human_hold/' + name in manifest['files'],
             'Integrated independent human source is not frozen: ' + name)
    return manifest


def rebuild_current_model(base, model_inputs, pins, *, capture=None, events=None):
    required_inputs = {'topology', 'events', 'calibration', 'model_profile', 'source_binding', 'mount', 'bias'}
    need(type(model_inputs) is dict and required_inputs <= set(model_inputs) <= required_inputs | {'accel_hypothesis'},
         'Exact current capture/events plus model-only source dependencies required')
    selected = copy.deepcopy(model_inputs)
    if capture is not None:
        selected['topology'], selected['events'] = capture, events
    inputs = {key: pins.envelope(ref, events=key == 'events') for key, ref in selected.items()}
    plan = model_bridge.prepare_model_plan(**inputs)
    plan = model_bridge.reclock_model_plan(plan, base['control_clock'], base['policy30_model'])
    model_bridge._plan_axes(plan)
    need(plan['provenance']['boot_id'] == base['boot_id'] and
         plan['provenance']['motor_power_epoch'] == base['motor_power_epoch'] and
         plan['provenance']['uids_by_id'] == base['uids_by_id'] and
         plan['topology_by_port'] == base['topology_by_port'] and
         plan['source_binding']['four_bus_source_manifest'] == base['source_manifest'],
         'Rebuilt human pose has another current UID/boot/power/topology/source')
    for key, axis in base['axes'].items():
        need(plan['axes'][key]['fixed_offset_rad'] == axis['fixed_offset_rad'] and
             plan['axes'][key]['reference_turns'] == axis['reference_turns'] and
             plan['axes'][key]['sign'] == axis['sign'], 'Human capture branch changed: ID' + key)
        boxed._check_pose(base, plan['provenance']['model_rad_by_id'], 'fresh human capture')
    return plan, pins.json(selected['topology'])


def _one_frame(raw, label):
    need(type(raw) is str and len(raw) == 34 and all(c in '0123456789abcdef' for c in raw),
         'One canonical17-byte ' + label + ' required')
    parser = codec.ATParser()
    frames = parser.feed(bytes.fromhex(raw))
    need(len(frames) == 1 and not parser.buffer and not parser.discarded_bytes and
         frames[0].flags == 4 and len(frames[0].data) == 8, 'Malformed canonical ' + label)
    return frames[0]


def _feedback(raw, mid, *, mode, profile):
    frame = _one_frame(raw, 'Type2 feedback')
    result = protocol.decode_type2(frame, motor_id=mid)
    row = profile['axes'][str(mid)]
    model_angle = row['sign'] * result.protocol_position_rad + row['fixed_offset_rad']
    center = sum(profile['start_pose_bounds'][str(mid)])/2
    need(result.mode_state == mode and result.fault_bits == 0 and
         row['lower_rad'] <= model_angle <= row['upper_rad'] and
         abs(model_angle-center) <= row['max_displacement_from_start_rad'] and
         abs(result.velocity_rad_s) <= row['max_measured_velocity_rad_s'] and
         abs(result.torque_nm) <= row['max_measured_torque_nm'] and
         -10 <= result.temperature_c < row['max_temperature_c'],
         'Fault/mode/velocity/torque/temperature outside human hold: ID' + str(mid))
    return result


def _transaction(row, *, deadline_ns, write_deadline_ns=None):
    need(type(row) is dict and row.get('written') == 17 and type(row.get('written')) is int and
         row.get('received') == 17 and type(row.get('received')) is int,
         'Complete original17-byte write and reply required')
    start, finish, read, received = (_stamp(row.get(key), key) for key in
        ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns'))
    need(start <= finish <= read <= received < deadline_ns and row.get('deadline_ns') == deadline_ns,
         'Original write/read/reply clocks are noncausal or over deadline')
    if write_deadline_ns is not None:
        need(finish < write_deadline_ns, 'All human Type1 host writes must finish before20ms')
    return start, finish, received


def validate_zero_gain_report(report, contract, *, after_ns, source_manifest_sha256, pins=None):
    """Primary raw 30 Hz human-pose Type1 timing proof; never boxed or positive."""
    base, profile = contract['base_numerical_contract'], contract['runtime_profile']
    clock = validate_control_clock(base['control_clock'])
    need(type(report) is dict and report.get('schema') == REPORT_SCHEMA and
         report.get('mode') == 'human_zero_gain_timing' and
         report.get('status') == 'COMPLETE_HUMAN_ZERO_GAIN_TIMING' and
         report.get('duration_s') == 2 and report.get('contract_sha256') == canonical_sha256(contract),
         'Separate complete human-pose zero-gain timing report required')
    for key, expected in (('boot_id', base['boot_id']), ('motor_power_epoch', base['motor_power_epoch']),
        ('control_clock', base['control_clock']), ('policy30_model', base['policy30_model']),
        ('source_manifest_sha256', source_manifest_sha256), ('errors', []),
        ('failure_retained', False), ('normal_ramp_completed', True), ('motor_enable_sent', True),
        ('type1_sent', True), ('positive_gain_sent', False), ('learned_targets_attempted', False),
        ('physical_cutoff_required', False), ('restoration_complete', True),
        ('source_and_input_pins_unchanged', True)):
        need(report.get(key) == expected and (type(expected) is not bool or type(report[key]) is bool),
             'Human zero-gain primary binding differs: ' + key)
    m = report.get('measurement', report)
    need(type(m) is dict and m.get('backend_usage', {}).get('kind') == 'genuine_subset_active_cpp',
         'Genuine original C++ Type1 backend required; injected tests are not a prerequisite')
    backend, input_pins = m['backend_usage'], report.get('input_sha256')
    need(type(input_pins) is dict and backend.get('source_manifest_sha256') == source_manifest_sha256 and
         source_manifest_sha256 in input_pins.values(), 'Human zero-gain backend source must be input-pinned')
    for key in ('library_sha256', 'build_record_sha256'):
        boxed._sha(backend.get(key), 'human native ' + key)
        need(backend[key] in input_pins.values(), 'Human zero-gain native/source receipts are not input-pinned')
    cycles = m.get('cycles')
    need(type(cycles) is list and 0 < len(cycles) <= 60 and
         type(report.get('completed_cycles')) is int and report['completed_cycles'] == len(cycles),
         'Complete bounded two-second human zero-gain original cycles required')
    from experiments.four_bus_type1.type1_runner import imu_frame, check_imu_limits
    frame = imu_frame(base['model_plan'])
    correction = None
    hypothesis_ref = base['model_plan']['observer_kwargs'].get('accel_input_hypothesis')
    if hypothesis_ref is not None:
        need(type(pins) is Pins, 'Selected acceleration hypothesis requires primary file pins')
        from singularitydog_hw.imu_accel_input_hypothesis import load_accel_input_hypothesis
        _pin_reference_tree(hypothesis_ref, pins)
        correction = load_accel_input_hypothesis(hypothesis_ref, frame['rotation'])
    raw_journal = m.get('raw_journal')
    need(type(raw_journal) is dict and set(raw_journal) == set(PORTS), 'All four original Type1 journals required')
    journal_batches = {}
    for port, entries in raw_journal.items():
        need(type(entries) is list, 'Original owner journal list required')
        journal_batches[port] = Counter()
        for entry in entries:
            if type(entry) is list and len(entry) == 2 and type(entry[0]) is str and type(entry[1]) is dict:
                raw = entry[1]
            elif type(entry) is dict:
                raw = entry.get('exchange', entry)
            else:
                raise ValueError('Unsupported original Type1 journal entry')
            if type(raw.get('records')) is list:
                journal_batches[port][canonical_sha256(raw['records'])] += 1
    epoch, previous_begin, previous_imu = _stamp(cycles[0].get('release_ns'), 'epoch'), None, 0
    phases = []
    need(epoch >= after_ns, 'Human zero-gain report must follow fresh STOP501 diagnostics')
    for index, row in enumerate(cycles):
        begin, first, end = (_stamp(row.get(key), key) for key in ('begin_ns', 'first_ns', 'cycle_end_ns'))
        release, hard = clock.release_ns(epoch, index), begin+clock.cycle_budget_ns
        write = min(begin, first)+20_000_000
        need(row.get('index') == index and row.get('slot') == index and
             row.get('release_ns') == release and release <= begin <= release+1_000_000 and
             row.get('hard_end_ns') == hard and row.get('write_deadline_ns') == write and end < hard and
             row.get('completed') is True and row.get('request_count') == 28 and row.get('weight') == 0,
             'Human zero-gain original cycle missing, skipped or over deadline')
        edges = [_stamp(row.get(key), key) for key in ('begin_ns', 'first_ns', 'acquired_ns',
            'gather_ns', 'infer_end_ns', 'voltage_join_ns', 'final_gate_ns', 'encode_end_ns',
            'output_submit_ns', 'reply_return_ns', 'cycle_end_ns')]
        need(all(left <= right for left, right in zip(edges, edges[1:])),
             'Human acquisition/inference/encode/output/reply clocks are noncausal')
        command = row.get('command')
        need(type(command) is dict and command.get('phase') in ('starting', 'active', 'stopping', 'stopped') and
             command.get('monotonic_s') == row['final_gate_ns']/1e9,
             'Original human zero-gain envelope phase and clock required')
        phase = command['phase']; phases.append(phase)
        for key in ('kp', 'kd', 'velocity_reference_rad_s', 'feedforward_torque_nm', 'command_velocity_rad_s'):
            vector = command.get(key)
            need(type(vector) is list and len(vector) == 12 and
                 all(type(value) in (int, float) and value == 0 for value in vector),
                 'Human zero-gain command must retain exact all12 zero ' + key)
        need(type(command.get('q_model_rad')) is list and len(command['q_model_rad']) == 12 and
             all(type(q) in (int, float) and math.isfinite(q) for q in command['q_model_rad']),
             'Original all12 current-position command required')
        if previous_begin is not None:
            need(clock.minimum_start_separation_ns <= begin-previous_begin <= clock.command_gap_budget_ns,
                 'Human zero-gain cadence gap outside explicit30Hz contract')
        previous_begin = begin
        writes, replies, request_starts = [], [], []
        for phase, count in (('hold', 3), ('voltage', 1), ('output', 3)):
            batches = row.get(phase)
            need(type(batches) is dict and set(batches) == set(PORTS), 'All four raw ' + phase + ' batches required')
            for port, ids in base['topology_by_port'].items():
                batch = batches[port]
                records = batch.get('records') if type(batch) is dict else None
                need(type(records) is list and len(records) == count and
                     batch.get('physical_port') == port and batch.get('physical_ids') == ids and
                     batch.get('original_record_count') == count and
                     batch.get('rejected_total_bytes') == 0 and batch.get('rejected_truncated') is False,
                     'Complete unmodified raw batch required: ' + phase + '/' + port)
                digest = canonical_sha256(records)
                need(journal_batches[port][digest] > 0,
                     'Cycle raw batch absent or reused in original owner journal: ' + phase + '/' + port)
                journal_batches[port][digest] -= 1
                if phase == 'output':
                    need(batch.get('separate_write_deadline_monotonic_ns') == write and
                         batch.get('record_deadlines_are_reply_deadlines') is True,
                         'Native independent host-write cutoff absent')
                mids = []
                for raw in records:
                    cap = hard
                    started, finished, received = _transaction(raw, deadline_ns=cap,
                        write_deadline_ns=write if phase == 'output' else None)
                    need(started >= begin, 'Current human request predates this cycle release')
                    if phase != 'output':
                        need(received < write, 'Fresh acquisition/voltage must complete within unchanged20ms prefix')
                    tx = _one_frame(raw.get('tx_hex'), 'request')
                    mid = tx.can_id & 0xff
                    need(mid in ids and mid not in mids, 'Cross-port or duplicated request ID')
                    mids.append(mid)
                    request_starts.append(started)
                    if phase == 'voltage':
                        need(bytes.fromhex(raw['tx_hex']) == codec.read_request(mid, 'voltage'),
                             'Only rotating voltage Type17 is allowed between human Type1 phases')
                        rx = _one_frame(raw.get('rx_hex'), 'voltage reply')
                        result = codec.decode_reply(rx, mid, 'voltage')
                        need(result['ok'] is True and 35 <= _number(result.get('value'), 'voltage') <= 42,
                             'Human diagnostic voltage outside35..42V')
                    else:
                        p, v, kp, kd = struct.unpack('>4H', tx.data)
                        need(tx.kind == 1 and (tx.can_id >> 8)&0xffff == 32767 and
                             v == 32767 and kp == 0 and kd == 0,
                             'Zero-gain diagnostic may contain only exact Type1 Kp/Kd0 and zero v/FF')
                        q = p*25.14/65535.-12.57
                        axis = base['axes'][str(mid)]
                        need(axis['raw_lower_rad'] <= q <= axis['raw_upper_rad'],
                             'Raw zero-gain target outside current reviewed branch: ID' + str(mid))
                        _feedback(raw.get('rx_hex'), mid, mode=2, profile=profile)
                    if phase == 'output':
                        need(started >= row['encode_end_ns'], 'Human output write predates its own encoded command')
                        writes.append(finished); replies.append(received)
                if phase != 'voltage':
                    need(mids == ids, 'Human raw Type1 order differs from exact3 group')
        imu = row.get('imu')
        need(type(imu) is dict, 'Original fresh IMU metadata required')
        imu_start = _stamp(imu.get('read_started_monotonic_ns'), 'IMU read start')
        imu_finish = _stamp(imu.get('read_finished_monotonic_ns'), 'IMU read finish')
        need(imu_start <= imu_finish < write and first == min(imu_start, *request_starts),
             'Human oldest-input freshness origin differs from raw acquisition')
        from singularitydog_hw.policy_output_runtime import validate_imu_metadata
        try:
            previous_imu = validate_imu_metadata(imu, _stamp(row.get('acquired_ns'), 'acquisition completed'),
                                                 profile, previous=previous_imu)
        except RuntimeError as error:
            raise ValueError('Invalid original human IMU metadata') from error
        check_imu_limits(imu, {key: profile[key] for key in boxed.IMU_KEYS},
                         frame['rotation'], frame['gyro_bias'], correction)
        need(row.get('final_write_ns') == max(writes) and row.get('output_last_reply_ns') == max(replies),
             'Human write/reply summaries differ from all12 original raw records')
    reserve = max(row['max_command_velocity_rad_s']/row['max_command_acceleration_rad_s2']
                  for row in profile['axes'].values())+profile['stop_duration_s']+2*clock.cycle_budget_ns/1e9
    need(max(1.7, 2-reserve)*1e9 <= cycles[-1]['cycle_end_ns']-epoch < (2+clock.period_s)*1e9,
         'A short prefix cannot substitute for the bounded two-second zero-gain timing run')
    compact = [phase for index, phase in enumerate(phases) if index == 0 or phase != phases[index-1]]
    need(compact == ['starting', 'active', 'stopping', 'stopped'] and phases.count('stopped') == 1 and
         cycles[-1]['command'].get('stop_stage') == 'complete',
         'Original human startup/full-gain/graceful-stop/complete progression required')
    first_active = next(row for row in cycles if row['command']['phase'] == 'active')
    first_stopping = next(row for row in cycles if row['command']['phase'] == 'stopping')
    need(first_active['command']['monotonic_s'] >= epoch/1e9+1 and
         first_stopping['command']['monotonic_s'] >= epoch/1e9+2-reserve and
         cycles[-1]['command']['monotonic_s']-first_stopping['command']['monotonic_s'] >= .4-clock.period_s,
         'Human zero-gain startup or bounded fade clocks differ from the actual control contract')
    _validate_worker_and_stop(m, base, profile, after_ns=cycles[-1]['cycle_end_ns'])
    return {'completed_cycles': len(cycles), 'terminal_stop_finished_monotonic_ns':
            _stamp(m.get('all_workers_joined_monotonic_ns'), 'joined workers')}


def _validate_worker_and_stop(measurement, base, profile, *, after_ns):
    workers = measurement.get('worker_settings')
    need(type(workers) is dict and set(workers) == {*PORTS, 'imu'}, 'Five actual CAN/IMU owners required')
    tids = []
    for i, port in enumerate((*PORTS, 'imu')):
        row = workers[port]
        need(type(row) is dict and row.get('cpu_mask') == ([i] if i < 4 else [0, 1, 2, 3]) and
             row.get('timer_slack_ns') == 1000 and row.get('file_only_mock_readback') is False,
             'Real owner CPU/slack readback required: ' + port)
        tids.append(_stamp(row.get('native_tid'), 'owner tid'))
    need(len(set(tids)) == 5, 'Five independent original owner threads required')
    from experiments.four_bus_diagnostic.pipeline import _real_operating_readback
    _real_operating_readback(measurement.get('operating_settings'))
    restoration = measurement.get('restoration')
    need(type(restoration) is dict and all(restoration.get(key) is True for key in
         (*PORTS, 'imu', 'main', *(port+'_session' for port in PORTS))), 'Exact owner/session/main restoration required')
    need(measurement.get('stop_confirmed') is True and measurement.get('host_watchdog_reason') == 'normal completion',
         'Successful final STOP and no host watchdog failure required')
    journal, stops = measurement.get('raw_journal'), measurement.get('stop_results')
    need(type(journal) is dict and set(journal) == set(PORTS) and type(stops) is dict and
         set(stops) == set(PORTS), 'Separate all12 terminal STOP journals required')
    for port, ids in base['topology_by_port'].items():
        stop = stops[port]
        need(type(stop) is dict and stop.get('complete') is True and stop.get('confirmed_ids') == ids and
             stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
             stop.get('faults') == {str(mid): 0 for mid in ids}, 'No ambiguous terminal STOP: ' + port)
        entries = journal[port]
        need(type(entries) is list, 'Raw terminal STOP owner journal required')
        normalized = []
        for entry in entries:
            if type(entry) is list and len(entry) == 2 and type(entry[0]) is str and type(entry[1]) is dict:
                normalized.append({'label': entry[0], **entry[1]})
            elif type(entry) is dict:
                normalized.append(entry)
            else:
                raise ValueError('Original owner journal has unsupported record shape')
        # The authentic adapter also journals an earlier preflight exchange
        # labelled 'stop'. Only its final emergency subset-STOP primitive owns
        # all twelve terminal replies. Earlier setup STOPs cannot substitute.
        terminal = [entry for entry in normalized
                    if entry.get('label', entry.get('phase')) == 'stop_subset']
        need(terminal, 'Terminal subset STOP raw journal missing: ' + port)
        confirmed = set()
        for entry in terminal:
            records = entry.get('records', entry.get('exchange', {}).get('records'))
            need(type(records) is list and records, 'Terminal STOP raw records missing')
            for raw in records:
                if raw.get('written') == raw.get('received') == 0:
                    need(len(records) == 6 and all(raw.get(name) == 0 for name in
                         ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns')) and
                         raw.get('tx_hex') == raw.get('rx_hex') == '00'*17,
                         'Only untouched original subsetSTOP slots may be omitted from selected axes')
                    continue
                started, finished, received = _transaction(raw, deadline_ns=_stamp(raw.get('deadline_ns'), 'STOP deadline'))
                need(started >= after_ns, 'Earlier cycle STOP cannot substitute for final STOP')
                tx = _one_frame(raw.get('tx_hex'), 'terminal STOP')
                mid = tx.can_id & 0xff
                need(mid in ids and bytes.fromhex(raw['tx_hex']) ==
                     protocol.stop_request(phase=protocol.TrialPhase.STOP, motor_id=mid),
                     'Terminal journal contains an ambiguous or non-STOP command')
                _feedback(raw.get('rx_hex'), mid, mode=0, profile=profile)
                confirmed.add(mid)
        need(confirmed == set(ids), 'Every terminal STOP needs its own causal raw reply')
    need(_stamp(measurement.get('all_workers_joined_monotonic_ns'), 'joined workers') > after_ns,
         'Owner cleanup must follow the completed cycles')


def validate_watchdog(report, trace, contract, *, after_ns, source_manifest_sha256):
    """Fresh all12 command-loss semantics re-proved against original Channel events."""
    base, profile = contract['base_numerical_contract'], contract['runtime_profile']
    need(type(report) is dict and report.get('status') == 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC' and
         report.get('errors') == [] and report.get('selected_ids') == list(IDS) and
         report.get('configured_timeout_ms') == 200 and report.get('watchdog_ticks') == 4000 and
         report.get('motor_enable_sent') is True and report.get('stop_confirmed') is True and
         report.get('positive_gain_sent') is False and report.get('learned_targets_sent') is False and
         report.get('usb_disconnect_tested') is False and report.get('approved_for_runtime') is False and
         report.get('boot_id') == base['boot_id'] and report.get('motor_power_epoch') == base['motor_power_epoch'] and
         report.get('source_manifest_sha256') == source_manifest_sha256,
         'Fresh new-source current human-pose all12 zero-gain watchdog report required')
    need(type(trace) is dict and trace.get('schema') == 'singularitydog.four-can-watchdog-trace.v1' and
         trace.get('report_canonical_sha256') == canonical_sha256(report) and
         type(trace.get('events_by_port')) is dict and set(trace['events_by_port']) == set(PORTS),
         'Pinned original four-owner watchdog events must bind this report')
    axes = report.get('axes')
    need(type(axes) is dict and set(axes) == {str(i) for i in IDS}, 'All12 watchdog axes required')
    finishes = []
    for port, ids in base['topology_by_port'].items():
        events = trace['events_by_port'][port]
        need(type(events) is list and events, 'Original watchdog events required: ' + port)
        replies = _watchdog_transactions(events, ids, axes)
        for mid in ids:
            key, axis = str(mid), axes[str(mid)]
            need(type(axis) is dict and axis.get('uid') == base['uids_by_id'][key] and
                 axis.get('command_loss_tested') is True and axis.get('disabled_on_command_loss') is True and
                 axis.get('configured_timeout_ms') == 200 and axis.get('usb_disconnect_tested') is False and
                 axis.get('disable_upper_bound_origin') == 'last_zero_host_write_started_ns' and
                 axis.get('watchdog_readback', {}).get('value') == 4000 and
                 35 <= _number(axis.get('voltage_v'), 'watchdog voltage') <= 42,
                 'Watchdog UID/timeout/disable/voltage differs: ID' + key)
            rows = replies[mid]
            need(len(rows['identity']) == 1 and rows['identity'][0]['value']['mcu_uid_hex'] == axis['uid'] and
                 len(rows['can_timeout']) == 1 and rows['can_timeout'][0]['value']['value'] == 4000 and
                 len(rows['watchdog_write']) == 1 and len(rows['enable']) == 1 and len(rows['zero']) == 3 and
                 len(rows['stop']) >= 2 and len(rows['version']) == 1,
                 'Exact watchdog preflight/enable/silence/STOP sequence absent: ID' + key)
            need(rows['identity'][0]['start_ns'] >= after_ns, 'Watchdog must follow the fresh human capture')
            need(len(rows['run_mode']) == len(rows['voltage']) == 1 and
                 rows['run_mode'][0]['value'].get('value') == 0 and
                 rows['voltage'][0]['value'].get('value') == axis['voltage_v'] and
                 rows['version'][0]['value'].get('version_bytes_hex') == axis.get('version', {}).get('version_bytes_hex'),
                 'Original watchdog mode/voltage/FW bytes differ from semantic axis row')
            preflight = [rows[step][0] for step in ('identity', 'stop', 'version', 'run_mode', 'voltage',
                                                  'watchdog_write', 'can_timeout', 'zero', 'enable')]
            need(all(left['received_ns'] <= right['start_ns'] for left, right in zip(preflight, preflight[1:])) and
                 preflight[-1]['received_ns'] <= rows['zero'][1]['start_ns'],
                 'Watchdog per-axis preflight, zero-before-enable and active sequence was reordered')
            disabled, active, probe = rows['zero']
            for row, mode in ((disabled, 0), (active, 2), (probe, 0)):
                _feedback(row['rx_hex'], mid, mode=mode, profile=profile)
            silent_start, probe_start, probe_end = active['start_ns'], probe['start_ns'], probe['received_ns']
            need(210_000_000 <= probe_start-active['received_ns'] and
                 210_000_000 <= probe_end-silent_start <= 250_000_000 and
                 axis.get('last_zero_write_start_ns') == silent_start and
                 axis.get('last_zero_write_finish_ns') == active['finish_ns'] and
                 axis.get('stop_probe', {}).get('received_ns') == probe_end and
                 axis.get('disable_reply_upper_bound_ms') == (probe_end-silent_start)/1e6,
                 'Watchdog silence or conservative250ms upper bound not supported by raw clocks: ID' + key)
            # A silence interval is global, not a per-port idle interval.
            for other in trace['events_by_port'].values():
                need(not any(e.get('kind') == 'tx' and active['finish_ns'] < e.get('start_ns', 0) < probe_start
                             for e in other), 'CAN transmission occurred during watchdog silence')
            need(rows['stop'][-1]['start_ns'] >= probe_end, 'Final STOP must follow disabled-state probe')
            _feedback(rows['stop'][-1]['rx_hex'], mid, mode=0, profile=profile)
            finishes.append(rows['stop'][-1]['received_ns'])
        stop = report.get('stop_reports', {}).get(port)
        need(type(stop) is dict and stop.get('complete') is True and stop.get('confirmed_ids') == ids and
             stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and stop.get('errors') == [],
             'Watchdog terminal STOP semantic ambiguity must remain sticky: ' + port)
    return {'finished_monotonic_ns': max(finishes),
            'firmware_by_id': {key: axis['version']['version_bytes_hex'] for key, axis in axes.items()}}


def _watchdog_transactions(events, ids, axes):
    result = {mid: {step: [] for step in ('identity', 'stop', 'version', 'run_mode', 'voltage',
                                        'watchdog_write', 'can_timeout', 'zero', 'enable')} for mid in ids}
    pending, parser, received_raw = None, codec.ATParser(), bytearray()
    last = 0
    for event in events:
        need(type(event) is dict and event.get('kind') in ('tx', 'rx_bytes'),
             'Rejected or failed watchdog events cannot be hidden')
        if event['kind'] == 'tx':
            need(pending is None and not parser.buffer and not parser.discarded_bytes,
                 'Watchdog request has pending/partial preceding reply')
            mid, step = event.get('motor_id'), event.get('step')
            need(type(mid) is int and mid in ids and step in result[mid], 'Cross-owner/arbitrary watchdog command')
            start, finish = _stamp(event.get('start_ns'), 'watchdog write start'), _stamp(event.get('finish_ns'), 'watchdog write finish')
            need(last <= start <= finish and type(event.get('returned_bytes')) is int and
                 event['returned_bytes'] == 17, 'Partial or noncausal watchdog command')
            if step in ('identity', 'run_mode', 'voltage', 'can_timeout'):
                expected = codec.read_request(mid, None if step == 'identity' else step)
            elif step == 'version':
                expected = version_request(mid)
            elif step == 'stop':
                expected = protocol.stop_request(phase=protocol.TrialPhase.STOP, motor_id=mid)
            elif step == 'watchdog_write':
                expected = protocol.watchdog_setup_request(phase=protocol.TrialPhase.WATCHDOG_SETUP, motor_id=mid)
            elif step == 'enable':
                expected = protocol.enable_request(phase=protocol.TrialPhase.ENABLE, motor_id=mid)
            else:
                expected = protocol.motion_request(phase=protocol.TrialPhase.ZERO_GAIN,
                    center_rad=_number(axes[str(mid)].get('center_rad'), 'center'), motor_id=mid)
            need(event.get('hex') == expected.hex(), 'Watchdog original command differs from declared step')
            pending = dict(mid=mid, step=step, start_ns=start, finish_ns=finish)
            received_raw.clear()
            last = finish
        else:
            need(pending is not None, 'Unsolicited watchdog receive bytes')
            received = _stamp(event.get('received_ns'), 'watchdog receive')
            need(last <= received < pending['start_ns']+250_000_000,
                 'Noncausal/late watchdog reply')
            raw = bytes.fromhex(event.get('hex', ''))
            need(raw, 'Empty original watchdog receive event')
            received_raw.extend(raw)
            frames = parser.feed(raw)
            need(not parser.discarded_bytes and len(frames) <= 1, 'Malformed/duplicate watchdog reply')
            last = received
            if not frames:
                continue
            need(not parser.buffer and len(received_raw) == 17, 'Residual or ambiguous watchdog reply')
            mid, step, frame = pending['mid'], pending['step'], frames[0]
            need(frame.flags == 4 and len(frame.data) == 8, 'Noncanonical watchdog CAN reply')
            if step in ('identity', 'run_mode', 'voltage', 'can_timeout'):
                value = codec.decode_reply(frame, mid, None if step == 'identity' else step)
                need(value['ok'] is True, 'Failed original watchdog parameter reply')
            elif step == 'version':
                value = decode_version(frame, mid)
            else:
                feedback = protocol.decode_type2(frame, motor_id=mid)
                need(feedback.fault_bits == 0, 'Fault in original watchdog feedback')
                value = {'mode_state': feedback.mode_state}
            result[mid][step].append({**pending, 'received_ns': received,
                                     'rx_hex': bytes(received_raw).hex(), 'value': value})
            pending = None
    need(pending is None and not parser.buffer and not parser.discarded_bytes,
         'Incomplete original watchdog trace cannot qualify')
    return result


def _audio(document, profile, pins):
    need(type(document) is dict and document.get('schema') == AUDIO_SCHEMA and
         document.get('scope') == SCOPE and document.get('contract_sha256') is not None,
         'Separate human-hold30 pinned audio manifest required')
    meanings = ('prepare_ease_is_not_go', 'go_is_short_tone', 'resupport_starts_with_urgent_tone',
                'operator_must_resupport_before_voice_finishes',
                'audio_process_completion_is_not_proof_of_audibility',
                'physical_ease_duration_not_verified_by_audio')
    need(all(document.get(key) is True for key in meanings), 'Explicit human cue meaning and limitations required')
    clips = document.get('clips')
    need(type(clips) is dict and set(clips) == set(STAGES), 'Five pinned stage-specific clips required')
    checked = {}
    for stage, clip in clips.items():
        need(type(clip) is dict and set(clip) == {'path', 'sha256', 'duration_s', 'transcript'},
             'Exact audio stage reference/duration/transcript required')
        _text(clip['transcript'], 'audio transcript')
        raw = pins.raw({'path': clip['path'], 'sha256': clip['sha256']})
        need(0 < len(raw) <= 16*1024*1024, 'Bounded real audio bytes required')
        try:
            with wave.open(io.BytesIO(raw), 'rb') as wav:
                frames = wav.getnframes()
                need(wav.getcomptype() == 'NONE' and wav.getnchannels() == 2 and
                     wav.getsampwidth() == 2 and wav.getframerate() == 48000 and frames > 0 and
                     len(wav.readframes(frames)) == frames*4, 'Human cues require complete48kHz stereoPCM16 WAV')
                duration = frames/48000.
        except (wave.Error, EOFError) as error:
            raise ValueError('Malformed human audio WAV') from error
        need(_number(clip['duration_s'], 'clip duration') == duration and duration <= 30,
             'Audio manifest duration differs from the actual hashed WAV')
        if stage == 'go':
            need(duration <= .12, 'Human go tone must be at most120ms')
        checked[stage] = dict(path=clip['path'], sha256=clip['sha256'], duration_s=duration)
    settings = human_supported_hold_settings(profile)
    reserve = (SETTINGS['cue_not_before_s'] + checked['prepare_ease']['duration_s'] + .25 +
        checked['go']['duration_s'] + .25 + SETTINGS['slight_ease_max_duration_s'] +
        checked['resupport']['duration_s'] + .25 + SETTINGS['resupport_ack_window_s'] + 1/30)
    need(reserve < settings['latest_stop_s'], 'All spoken cues, easing, acknowledgement and fade must fit8s')
    return {'clips': checked, 'cue_reserve_s': reserve,
            'audio_completion_does_not_prove_audibility': True,
            'physical_ease_duration_not_verified_by_audio': True}


class Prepared:
    """Immutable primary review, distinct from any old boxed Admitted object."""
    __slots__ = ('_token', '_execution_proof', 'plan', 'pins', 'audio', 'firmware', 'digest')

    def __init__(self, token, plan, pins, audio, firmware, execution_proof=None):
        need(token is _PREPARED_TOKEN, 'Use prepare_from_files(); arbitrary Prepared objects are rejected')
        for key, value in (('_token', token), ('_execution_proof', execution_proof),
            ('plan', _freeze(copy.deepcopy(plan))), ('pins', _freeze(dict(pins.values))),
            ('audio', _freeze(copy.deepcopy(audio))), ('firmware', _freeze(copy.deepcopy(firmware))),
            ('digest', canonical_sha256(plan))):
            object.__setattr__(self, key, value)

    def __setattr__(self, key, value):
        raise AttributeError('Prepared human review is immutable')

    def __delattr__(self, key):
        raise AttributeError('Prepared human review is immutable')

    def plain_plan(self):
        return _thaw(self.plan)

    def report_binding(self):
        value = verify_prepared(self)
        plan = value.plain_plan()
        base = plan['contract']['base_numerical_contract']
        return {'schema': REPORT_SCHEMA, 'scope': SCOPE, 'mode': plan['mode'],
            'duration_s': plan['duration_s'], 'contract_sha256': plan['contract_sha256'],
            'profile_canonical_sha256': self.digest,
            'conditions_sha256': canonical_sha256(plan['conditions']),
            'source_manifest_sha256': base['source_manifest']['sha256'],
            'boot_id': base['boot_id'], 'motor_power_epoch': base['motor_power_epoch'],
            'topology_by_port': copy.deepcopy(base['topology_by_port']),
            'pacing': copy.deepcopy(base['pacing']), 'control_clock': copy.deepcopy(base['control_clock']),
            'policy30_model': copy.deepcopy(base['policy30_model'])}


def verify_prepared(value):
    need(type(value) is Prepared and value._token is _PREPARED_TOKEN,
         'Genuine independently prepared human review required; boxed/dict grants are rejected')
    plan = value.plain_plan()
    need(canonical_sha256(plan) == value.digest and all(plan.get(key) is expected for key, expected in NO_GRANTS.items()),
         'Prepared content changed or acquired public output flags')
    validate_contract(plan['contract'])
    pins = Pins()
    for path, digest in _thaw(value.pins).items():
        pins.read(path, digest)
    return value


def validated_audio(prepared):
    value = verify_prepared(prepared)
    need(type(value.audio) is types.MappingProxyType, 'Prepared token-bound audio is absent')
    return _thaw(value.audio)


def supervisor_view(prepared, *, active):
    need(type(active) is bool, 'Explicit active boolean required')
    value = verify_prepared(prepared)
    plan = value.plain_plan()
    need(plan['mode'] == 'human_current_hold', 'Zero-gain timing must not bind the partial-load supervisor')
    if active:
        _require_execution(value)
    view = copy.deepcopy(plan['contract']['runtime_profile'])
    if active:
        # Generated after the distinct private proof check; never accepted from
        # user JSON or from the boxed Admitted class.
        view['_genuine_primary_execution_proof'] = True
    return view, validated_audio(value)


def _require_execution(value):
    need(value._execution_proof is _EXECUTION_PROOF_TOKEN and value.plan['blockers'] == (),
         'Execution unavailable: fresh primary gates and integrated new-runner receipt are incomplete')


def runtime_spec(prepared):
    """Opaque proof only; returns owned runner data after primary pin recheck."""
    value = verify_prepared(prepared)
    _require_execution(value)
    plan = value.plain_plan()
    base, profile = plan['contract']['base_numerical_contract'], plan['contract']['runtime_profile']
    if plan['mode'] == 'human_zero_gain_timing':
        profile = copy.deepcopy(profile)
        profile['duration_s'] = 2.
        profile['policy_ramp_s'] = .01
        for axis in profile['axes'].values():
            axis['kp'] = axis['kd'] = 0.
    from experiments.four_bus_type1 import type1_runner as old_runner
    from experiments.four_bus_diagnostic.transport_adapter import Group
    clock = validate_control_clock(base['control_clock'])
    maximum_stop = max(row['max_command_velocity_rad_s']/row['max_command_acceleration_rad_s2']
                       for row in profile['axes'].values())+profile['stop_duration_s']
    imu = old_runner.imu_frame(base['model_plan'])
    imu['limits'] = {key: float(profile[key]) for key in old_runner.IMU_LIMIT_RANGES}
    return {'mode': plan['mode'], 'duration_s': plan['duration_s'], 'ids_by_port': copy.deepcopy(base['topology_by_port']),
        'groups': tuple(Group(port, tuple(base['topology_by_port'][port])) for port in PORTS),
        'boot_id': base['boot_id'], 'motor_power_epoch': base['motor_power_epoch'],
        'contract_sha256': plan['contract_sha256'], 'profile': profile,
        'source_manifest_sha256': base['source_manifest']['sha256'],
        'offsets_by_id': {key: row['fixed_offset_rad'] for key, row in base['axes'].items()},
        'reference_turns_by_id': {key: row['reference_turns'] for key, row in base['axes'].items()},
        'uids_by_id': copy.deepcopy(base['uids_by_id']), 'firmware_by_id': _thaw(value.firmware),
        'offsets': {int(key): row['fixed_offset_rad'] for key, row in base['axes'].items()},
        'turns': {int(key): row['reference_turns'] for key, row in base['axes'].items()},
        'uids': {int(key): uid for key, uid in base['uids_by_id'].items()},
        'firmware': {int(key): firmware for key, firmware in _thaw(value.firmware).items()},
        'max_stop_s': maximum_stop, 'stop_at_s': plan['duration_s']-maximum_stop-2*clock.cycle_budget_ns/1e9,
        'imu': imu, 'options': boxed.pacing_options(base['pacing']), 'clock': clock, 'is30': True,
        'pacing': copy.deepcopy(base['pacing']), 'post_reply_policy': None, 'first_cycle_post_reply': False,
        'model_plan': copy.deepcopy(base['model_plan']), 'control_clock': copy.deepcopy(base['control_clock']),
        'policy30_model': copy.deepcopy(base['policy30_model']), 'human_contract': copy.deepcopy(plan['contract']),
        'conditions_canonical_sha256': canonical_sha256(plan['conditions']), 'independent_human_proof_verified': True}


def verify_runner_source(prepared, filename):
    value = verify_prepared(prepared)
    _require_execution(value)
    evidence = value.plain_plan()['evidence']
    expected = Path(evidence['source_root'])/'runtime/experiments/four_bus_human_hold/type1_runner.py'
    need(Path(filename).absolute() == expected,
         'Loaded runner must be the independently frozen human Type1 implementation')
    pin = _thaw(value.pins).get(str(expected))
    need(pin is not None, 'Loaded human runner lacks a primary source pin')
    topology.read_pinned(expected, pin)
    pins = Pins()
    verify_manifest(evidence['source_root'], evidence['source_manifest'], evidence['source_count'], pins)
    return value


def _frozen_compatibility_tool(source_root, manifest, pins):
    """Load only the actual reviewed file-only auditor from this frozen source."""
    root = Path(source_root)
    selected = {}
    for name, relative in (
        ('analyze_four_bus_type1_failure', 'tools/analyze_four_bus_type1_failure.py'),
        ('robotdog_frozen_type1_30_report_normalizer', 'tools/normalize_four_bus_type1_30hz_report.py')):
        record = manifest['files'].get(relative)
        need(type(record) is dict, 'Frozen compatibility audit source is missing: ' + relative)
        path = root/relative
        pins.read(path, record['sha256'])
        module = sys.modules.get(name)
        if module is None:
            spec = importlib.util.spec_from_file_location(name, path)
            need(spec is not None and spec.loader is not None, 'File-only compatibility auditor is unavailable')
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            try:
                spec.loader.exec_module(module)
            except BaseException:
                sys.modules.pop(name, None)
                raise
        need(Path(module.__file__).absolute() == path.absolute(),
             'Compatibility auditor was loaded from another source tree')
        selected[name] = module
    normalizer = selected['robotdog_frozen_type1_30_report_normalizer']
    need(normalizer.raw_audit is selected['analyze_four_bus_type1_failure'],
         'Compatibility auditor dependency has another loaded origin')
    return normalizer


def verified_predecessor_observation_sha(report, profile, *, report_ref, profile_ref,
                                         source_root, manifest, pins):
    """Keep physical observation on original bytes through a strict file-only derivative.

    Only a replay of the frozen raw auditor may authenticate a compatibility
    projection. Original report/profile bytes remain separately pinned. No
    observation, runtime grant or altered clock/packet/status is created here.
    """
    metadata_key = 'compatibility_deadline_field_projection'
    if metadata_key not in report:
        return _ref(report_ref, 'boxed predecessor')['sha256']
    metadata = report[metadata_key]
    need(type(metadata) is dict and metadata.get('schema') ==
         'singularitydog.original-deadline-field-projection.v1',
         'Unknown boxed predecessor compatibility projection')
    original_ref = _ref(metadata.get('original_report'), 'original boxed report')
    original_profile_ref = _ref(metadata.get('original_profile'), 'original boxed profile')
    need(original_profile_ref == _ref(profile_ref, 'boxed profile'),
         'Compatibility projection names another original profile')
    original = pins.json(original_ref)
    original_profile = pins.json(original_profile_ref)
    need(original_profile == profile, 'Original boxed profile bytes differ')
    normalizer = _frozen_compatibility_tool(source_root, manifest, pins)
    recomputed = normalizer.normalize(original, original_profile,
        original_reference=original_ref, profile_reference=original_profile_ref,
        expected_source_sha256=original_profile['contract']['source_manifest']['sha256'])
    need(recomputed == report,
         'Boxed compatibility projection changed raw/status/deadline/metadata from its pinned original')
    pins.verify()
    return original_ref['sha256']


def prepare_from_files(plan):
    """Authenticate all primary gates. Missing integration remains a review blocker.

    Only an integrated new runner can satisfy its receipt; this implementation
    does not create such a receipt from tests, a PLAN, or an old R3 snapshot.
    """
    need(type(plan) is dict and plan.get('schema') == SCHEMA and plan.get('status') == 'PLAN_ONLY' and
         all(plan.get(key) is expected for key, expected in NO_GRANTS.items()), 'Ungranting default PLAN required')
    contract = validate_contract(copy.deepcopy(plan.get('contract')))
    need(plan.get('contract_sha256') == canonical_sha256(contract) and plan.get('mode') in MODES and
         plan.get('duration_s') == DURATIONS[plan['mode']], 'Plan contract/mode/duration differs')
    refs, base, pins = plan['evidence'], contract['base_numerical_contract'], Pins()
    need(type(refs) is dict and set(refs) <= EVIDENCE_KEYS, 'Exact named primary evidence references required')
    required = EVIDENCE_KEYS - {'implementation_receipt'}
    if plan['mode'] == 'human_zero_gain_timing':
        required -= {'human_zero_gain_report', 'post_diagnostic_capture', 'post_diagnostic_events'}
    need(required <= set(refs), 'Missing named primary human evidence: ' + ', '.join(sorted(required-set(refs))))
    need(refs['source_manifest'] == base['source_manifest'], 'New frozen source differs from model/contract source')
    manifest = verify_manifest(refs['source_root'], refs['source_manifest'], refs['source_count'], pins)
    source = pins.json(refs['source_review'])
    need(type(source) is dict and source.get('schema') == SOURCE_REVIEW_SCHEMA and source.get('scope') == SCOPE and
         source.get('source_manifest_sha256') == refs['source_manifest']['sha256'] and
         source.get('decision') == 'REVIEWED_HUMAN_HOLD30_SOURCE_ONLY' and
         source.get('old_boxed_or_two_bus_permission_reused') is False and
         source.get('tests_grant_output_permission') is False,
         'Separate new-source review required; tests/old output are not permission')
    for name in ('reviewer', 'reviewed_at', 'rationale'):
        _text(source.get(name), 'source review ' + name)
    current, capture = rebuild_current_model(base, refs['model_inputs'], pins)
    from singularitydog_hw import policy_model_30hz as new_models
    need(new_models.plan(base['policy30_model']['artifacts']) == base['policy30_model'],
         'Actual retrained30Hz model delivery changed after the numerical PLAN')
    for reference in base['policy30_model']['artifacts'].values():
        pins.raw(reference)
    need(current == base['model_plan'], 'Base human numerical model plan must be rebuilt from exact current files')
    condition = pins.json(refs['current_conditions'])
    need(condition == plan['conditions'], 'Pinned direct human conditions differ from review PLAN')
    validate_conditions(condition, contract, capture_sha256=refs['model_inputs']['topology']['sha256'])
    boxed_profile = pins.json(refs['prior_boxed_profile'])
    boxed.validate_profile(boxed_profile)
    old_contract = boxed_profile['contract']
    need(old_contract.get('schema') == boxed.PROFILE_SCHEMA_30 and
         old_contract['policy30_model'] == base['policy30_model'],
         'Positive boxed predecessor must be the actual retrained30Hz model')
    prior_report = pins.json(refs['prior_boxed_report'])
    observation_sha = verified_predecessor_observation_sha(prior_report, boxed_profile,
        report_ref=refs['prior_boxed_report'], profile_ref=refs['prior_boxed_profile'],
        source_root=refs['source_root'], manifest=manifest, pins=pins)
    prior = boxed.validate_type1_report(prior_report, contract=old_contract,
        contract_digest=boxed_profile['contract_sha256'], mode='learned_boxed', duration_s=2)
    observed = pins.json(refs['prior_boxed_observation'])
    need(type(observed) is dict and observed.get('source') == 'direct_current_user_reply' and
         observed.get('report_sha256') == observation_sha and
         observed.get('audio_heard') is True and observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
         observed.get('synthetic_interaction') is False, 'Direct physical positive boxed2 observation required')
    _text(observed.get('user_statement'), 'prior boxed observation')
    _text(observed.get('user_reply_id'), 'prior boxed user reply')
    started, finished = _stamp(capture.get('started_monotonic_ns'), 'human capture start'), _stamp(capture.get('finished_monotonic_ns'), 'human capture finish')
    need(old_contract['boot_id'] == base['boot_id'] and prior['terminal_stop_finished_monotonic_ns'] < started < finished,
         'Fresh human capture must follow boxed2 terminalSTOP in the same boot; prior source is supplemental only')
    watchdog = validate_watchdog(pins.json(refs['watchdog_report']), pins.json(refs['watchdog_trace']), contract,
        after_ns=finished, source_manifest_sha256=refs['source_manifest']['sha256'])
    from experiments.four_bus_type1.type1_foreground import reviewed_firmware
    need(watchdog['firmware_by_id'] == reviewed_firmware(base, pins),
         'Fresh watchdog firmware differs from the independently pinned tested firmware')
    proxy = boxed.validate_stop_proxy_report(pins.json(refs['stop_proxy_report']),
        capture_refs=[refs['model_inputs']['topology']], topology_by_port=base['topology_by_port'],
        after_ns=watchdog['finished_monotonic_ns'], control_clock=base['control_clock'],
        policy30_model=base['policy30_model'], source_manifest_sha256=refs['source_manifest']['sha256'])
    if plan['mode'] == 'human_current_hold':
        zero = validate_zero_gain_report(pins.json(refs['human_zero_gain_report']), contract,
            after_ns=proxy['workers_joined_monotonic_ns'], source_manifest_sha256=refs['source_manifest']['sha256'], pins=pins)
        post, post_capture = rebuild_current_model(base, refs['model_inputs'], pins,
            capture=refs['post_diagnostic_capture'], events=refs['post_diagnostic_events'])
        need(_stamp(post_capture.get('started_monotonic_ns'), 'post-zero capture') > zero['terminal_stop_finished_monotonic_ns'],
             'Positive hold needs a fresh same-branch human capture after zero-gain terminalSTOP')
    audio_doc = pins.json(refs['audio_manifest'])
    need(audio_doc.get('contract_sha256') == canonical_sha256(contract), 'Audio manifest binds another human contract')
    audio = _audio(audio_doc, contract['runtime_profile'], pins)
    blockers = []
    receipt_ref = refs.get('implementation_receipt')
    if receipt_ref is None:
        blockers.append('ACTIVE_RUNNER_INTEGRATION_NOT_AUTHENTICATED')
    else:
        receipt = pins.json(receipt_ref)
        paths = ('runtime/experiments/four_bus_human_hold/runner.py',
                 'runtime/experiments/four_bus_human_hold/type1_runner.py',
                 'runtime/experiments/four_bus_human_hold/profile.py',
                 'runtime/experiments/four_bus_human_hold/supervision.py')
        need(type(receipt) is dict and receipt.get('schema') == IMPLEMENTATION_SCHEMA and
             receipt.get('scope') == SCOPE and receipt.get('source_manifest_sha256') == refs['source_manifest']['sha256'] and
             receipt.get('runtime_entry') == paths[0] and receipt.get('independent_type1_runner') == paths[1] and
             receipt.get('source_sha256') == {path: manifest['files'][path]['sha256'] for path in paths} and
             receipt.get('decision') == 'REVIEWED_INDEPENDENT_HUMAN_HOLD30_INTEGRATION' and
             receipt.get('tests_grant_output_permission') is False and receipt.get('old_boxed_permission_reused') is False,
             'New integrated runner receipt must name exact independently frozen executable source')
        for name in ('reviewer', 'reviewed_at', 'rationale'):
            _text(receipt.get(name), 'integration review ' + name)
        _verify_native_integration(receipt, refs['source_root'], manifest, pins)
    reviewed = copy.deepcopy(plan)
    reviewed.update(status='PRIMARY_REVIEW_COMPLETE', blockers=blockers)
    pins.verify()
    return Prepared(_PREPARED_TOKEN, reviewed, pins, audio, watchdog['firmware_by_id'],
                    _EXECUTION_PROOF_TOKEN if not blockers else None)


def _verify_native_integration(receipt, source_root, manifest, pins):
    """Authenticate registered actual foreground/native builds without dlopen."""
    foreground = receipt.get('active_foreground')
    need(type(foreground) is dict and set(foreground) == {'relative_path', 'sha256'} and
         foreground['relative_path'] == 'runtime/experiments/four_bus_human_hold/foreground.py' and
         manifest['files'].get(foreground['relative_path'], {}).get('sha256') == foreground['sha256'],
         'The independently registered hardware-owning human foreground is not frozen')
    pins.read(Path(source_root)/foreground['relative_path'], foreground['sha256'])
    adapter = receipt.get('human_transport_adapter')
    need(type(adapter) is dict and set(adapter) == {'relative_path', 'sha256'} and
         adapter['relative_path'] == 'runtime/experiments/four_bus_human_hold/type1_transport.py' and
         manifest['files'].get(adapter['relative_path'], {}).get('sha256') == adapter['sha256'],
         'Independent Kp6 human adapter with a separate loaded-source seal is not frozen')
    pins.read(Path(source_root)/adapter['relative_path'], adapter['sha256'])
    native = receipt.get('native_type1')
    need(type(native) is dict and set(native) ==
         {'library', 'build_record', 'ordinary_source', 'subset_stop_source', 'extension_source'},
         'Exact independently registered Type1 library/build/source references required')
    for reference in native.values():
        pins.raw(reference)
    from experiments.four_bus_type1 import build as native_build
    directory = Path(native['library']['path']).parent
    need(native_build.receipt_problem(directory) is None,
         'Registered original subset-active build receipt is invalid')
    record = pins.json(native['build_record'])
    scope = record.get('four_bus_subset_active')
    need(type(scope) is dict and type(scope.get('write_deadline_abi')) is int and scope['write_deadline_abi'] == 1 and
         record.get('binary_sha256') == native['library']['sha256'],
         'Original Type1 native independent20ms write-cutoff ABI1 build required')
    paths = {'ordinary_source': 'runtime/experiments/native_active_transport/transport.cpp',
             'subset_stop_source': 'runtime/experiments/four_bus_diagnostic/subset_stop.cpp',
             'extension_source': 'runtime/experiments/four_bus_type1/subset_active.cpp'}
    for key, relative in paths.items():
        digest = native[key]['sha256']
        need(manifest['files'].get(relative, {}).get('sha256') == digest and
             scope.get(key+'_sha256') == digest, 'Registered native source differs from the fresh frozen kit: ' + key)
    guard = receipt.get('native_current_guard')
    need(type(guard) is dict and set(guard) == {'library', 'build_record', 'source'},
         'Registered original current guard library/build/source required')
    for reference in guard.values():
        pins.raw(reference)
    record = pins.json(guard['build_record'])
    source = pins.raw(guard['source'])
    need(record.get('schema') == 'singularitydog.four-bus-current-guard-build.v1' and
         type(record.get('abi')) is int and record['abi'] == 1 and
         record.get('binary_sha256') == guard['library']['sha256'] and
         record.get('source_sha256') == guard['source']['sha256'] and
         record.get('source_bytes') == len(source) and record.get('CAN_IO_available') is False and
         record.get('output_allowed') is False and record.get('timing_admission_eligible') is False and
         manifest['files'].get('runtime/experiments/four_bus_diagnostic/native_current_guard.cpp', {}).get('sha256') == guard['source']['sha256'],
         'Registered current-check guard differs from its current source/build/binary pins')


def _pin_reference_tree(reference, pins):
    """Pin nested saved calibration/source artifacts, without executing their code."""
    pending, seen = [reference], set()
    while pending:
        ref = _ref(pending.pop(), 'nested saved hypothesis')
        if ref['path'] in seen:
            continue
        need(len(seen) < 256, 'Bounded saved hypothesis reference closure required')
        seen.add(ref['path'])
        raw = pins.raw(ref)
        if not ref['path'].endswith('.json'):
            continue
        doc = json.loads(raw, object_pairs_hook=topology.strict_pairs, parse_constant=topology.bad_constant)
        stack = [doc]
        while stack:
            value = stack.pop()
            if type(value) is dict:
                if {'path', 'sha256'} <= set(value):
                    pending.append({'path': value['path'], 'sha256': value['sha256']})
                stack.extend(value.values())
            elif type(value) is list:
                stack.extend(value)
