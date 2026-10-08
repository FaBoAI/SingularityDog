"""File-only four-bus Type1 boxed run profile, predecessor validators and admission.

Default PLAN reads pinned inputs and writes nothing; it opens no device,
library, Torch or model. --prepare writes one fresh profile. A profile, test
pass, timing result or STOP-proxy diagnostic is never output permission:
admit() additionally requires the current direct-human condition record for
the same boot, power epoch and contract. The record is written only from
explicit arguments; nothing here infers a physical condition or observation.
"""
import argparse
import copy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import types

if not __package__:
    sys.path.insert(0, str(Path(__file__).absolute().parents[2]))
    __package__ = 'experiments.four_bus_type1'

from singularitydog_hw import policy_live_profile as LP
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.policy_output_runtime import V3_VOLTAGE_MAX_AGE_NS
from experiments.four_bus_diagnostic import model_bridge, topology
from experiments.four_bus_diagnostic.transport_adapter import Group, PORTS, IDS

PROFILE_SCHEMA = 'singularitydog.four-bus-type1-boxed-profile.v1'
CONDITIONS_SCHEMA = 'singularitydog.four-bus-type1-current-conditions.v1'
REPORT_SCHEMA = 'singularitydog.four-bus-type1-boxed-run.v1'
PREPARATION_SCHEMA = 'singularitydog.four-bus-type1-profile-preparation.v1'
STOP_PROXY_SCHEMA = 'singularitydog.four-bus-foreground-stop-proxy.v1'
STOP_PROXY_PIPELINE_SCHEMA = 'singularitydog.four-bus-stop-proxy-pipeline.v1'
STOP_PROXY_STATUS = 'COMPLETE_STOP_PROXY_DIAGNOSTIC'
STOP_PROXY_CYCLES = 501
STOP_PROXY_SCHEDULE = 'each_physical_3STOP_feedback_then_1voltage_then_3STOP_proxy'
SCOPE = 'four_bus_boxed_small_type1_only'
MODES = ('zero_gain_timing', 'learned_boxed')
DURATIONS = (2, 10, 20)
COMPLETE_STATUS = {'zero_gain_timing': 'COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING',
                   'learned_boxed': 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED'}
STOP_UNCONFIRMED_STATUS = 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED'
CONDITIONS_SOURCE = 'direct_current_user_reply'
TRUE_CONDITIONS = ('motor_40v_on', 'box_supports_body', 'four_feet_touch_floor',
                   'all12_local_plus_minus3deg_clear', 'hands_off', 'immediate_40v_cutoff',
                   'other_drive_tools_stopped', 'box_will_remain')
FALSE_CONDITIONS = ('box_removal_allowed', 'load_transfer_allowed', 'standing_allowed', 'walking_allowed')
# (mode, duration) -> required (mode, duration) of the completed Type1 predecessor; None = any.
PREDECESSOR = {('learned_boxed', 2): ('zero_gain_timing', None),
               ('learned_boxed', 10): ('learned_boxed', 2),
               ('learned_boxed', 20): ('learned_boxed', 10)}
MARGIN_RAD = LP.LOCAL_NUMERICAL_MARGIN_RAD
RAW_LIMIT_RAD = 12.57
CAP_KEYS = tuple(LP.LIMIT_CAPS)
IMU_KEYS = ('imu_tilt_limit_rad', 'imu_gyro_limit_rad_s', 'imu_accel_norm_min_m_s2', 'imu_accel_norm_max_m_s2')
RAMP_KEYS = ('startup_duration_s', 'policy_ramp_s', 'stop_duration_s')
# Today's reviewed two-bus boxed2/10 profiles (evidence/validation-execution-20261008.json).
# Informational only: a match is recorded truthfully, it is not an admission.
TODAY_TWO_BUS_BOXED_PROFILE_SHA256 = {
    '7b6a3452881aced187e2f8cb497a133da3d9703a6185e9ee3640169d0d1b27a2': 'native890_boxed_2s',
    'd04e19fe22e4b4dce14f0fc69ce024b189b27fdc85f14d3dceaf172509f40078': 'native890_boxed_10s'}
POST_REPLY_POLICY = {'mode': 'bounded_post_reply_v1', 'max_lateness_ms': 1., 'max_consecutive_misses': 1,
                     'rolling_window_cycles': 100, 'max_misses_per_window': 1}
TIMING = {'period_ms': 20, 'hard_cycle_ms': 20, 'max_sample_age_ms': 20, 'max_sample_gap_ms': 21,
          'max_consecutive_20ms_misses': 0, 'startup_damping_duration_s': .08,
          'post_reply_deadline_policy': POST_REPLY_POLICY, 'voltage_min_v': 35, 'voltage_max_v': 42,
          'voltage_max_age_ms': 126, 'policy_weight': .005, 'h_hypothesis': 0, 'command': [0., 0., 0.]}
PACING = {'request_gap_us': 900, 'request_window': 3, 'release_spin_us': 500, 'main_cpu_mask': [4],
          'can_owner_cpu_by_port': {'port0': [0], 'port1': [1], 'port2': [2], 'port3': [3]},
          'imu_mask': [0, 1, 2, 3], 'timer_slack_ns': 1000, 'switch_interval_us': 100, 'nice': -10,
          'single_thread_math': True, 'native_phase_pair': False, 'separate_acquisition': True,
          'hold_type1_per_port': 3, 'voltage_reads_per_port': 1, 'output_type1_per_port': 3,
          'per_cycle_requests': 28, 'voltage_overlapped_with_inference': True,
          'boundary_current_checks': True, 'final_gate_input_identity': True,
          'two_bus_890us_or_200us_spin_equivalence_claimed': False}
# Opt-in timing options (jitter fixes F1-F4). A key enters the contract pacing
# only when selected, so the default contract, its SHA256 and every report
# pacing stay the R8 PACING byte for byte. Chained steps share one contract
# SHA256 and therefore exactly the same options.
PACING_OPTIONS = ('command_phase_offset_us', 'decode_once', 'prearmed_hold_lead_us', 'gc_freeze')
# F1 static check: release+K, then encode/submit, the output exchange and its
# worst recorded tail (R9 +2.85 ms) must end inside the unchanged 20 ms deadline.
COMMAND_PHASE_BUDGET_US = {'encode_and_submit': 470, 'output_exchange': 5140, 'output_tail': 2850}
COMMAND_PHASE_MIN_US = 9000  # Below the recorded natural gate (min 9.97 ms) pacing would never engage.
COMMAND_PHASE_MAX_US = 20_000-sum(COMMAND_PHASE_BUDGET_US.values())
PREARMED_LEAD_US = (300, 2000)  # Owner Python preparation fits; transport cap PREARM_MAX_LEAD_NS is 5 ms.
# F1+F3 static check: with a pre-armed next cycle, cycle k's worst end (the F1 budget
# above, then post-reply admission and F0 sampling) must also leave the next cycle's
# whole pre-arm window before release+20 ms: max(L, Boundary 1 + gates + owner prep).
PREARMED_TAIL_BUDGET_US = {'post_reply_admission': 730,  # R9 max cycle end K+6.34 ms less 5.61 ms.
                           'timing_evidence': 100}  # F0 sampling at the cycle end (tested < 100 us).
PREARM_WORK_US = 1250  # Boundary 1 (~0.16 ms), hold gates and owner prep (0.65-0.95 ms submit->last native).
WATCHDOG = {'motor_can_timeout_ticks': 4000, 'motor_can_timeout_ms': 200,
            'host_output_watchdog_ms': 40, 'host_output_watchdog_poll_ms': 2}
ENABLE = {'serial_single_axis': True, 'per_reply_budget_ms': 30, 'total_budget_ms': 120, 'retry': False}
TERMINAL_STOP = {'rounds': 3, 'per_port_budget_ms': 1000, 'collect_deadline_ms': 1250,
                 'concurrent_across_ports': True, 'ambiguity_sticky': True}
NO_GRANTS = {'output_allowed': False, 'approved_for_runtime': False, 'admitted': False,
             'motor_enable_sent': False, 'type1_sent': False, 'positive_gain_sent': False,
             'learned_targets_attempted': False, 'physical_post_trial_observation': None}
CONTRACT_KEYS = {'schema', 'scope', 'boot_id', 'motor_power_epoch', 'assembly_id', 'topology_by_port',
    'uids_by_id', 'enable_order_ids', 'axes', 'start_pose_bounds', 'timing', 'pacing', 'watchdog',
    'enable', 'terminal_stop', 'imu_limits', 'ramps', 'model_plan', 'model_plan_canonical_sha256',
    'model_artifacts', 'lineage', 'source_manifest', 'axis_geometry', 'authorized_modes',
    'authorized_durations_s', 'lp_scope_checks'}
AXIS_FIELDS = {'uid', 'physical_port', 'sign', 'nominal_offset_rad', 'fixed_offset_rad', 'reference_turns',
    'reviewed_physical_lower_rad', 'reviewed_physical_upper_rad', 'plan_local_lower_rad',
    'plan_local_upper_rad', 'physical_lower_rad', 'physical_upper_rad', 'lower_rad', 'upper_rad',
    'raw_lower_rad', 'raw_upper_rad', *CAP_KEYS}
PROFILE_KEYS = {'schema', 'status', 'mode', 'duration_s', 'contract', 'contract_sha256', 'evidence',
                'prepared_at', *NO_GRANTS}
CONDITION_KEYS = {'schema', 'source', 'direct_human', 'synthetic_interaction', 'user_statement',
    'user_reply_id', 'boot_id', 'motor_power_epoch', 'contract_sha256', 'authorized_modes',
    'authorized_durations_s', 'record_written_at', 'physical_condition_inferred',
    *TRUE_CONDITIONS, *FALSE_CONDITIONS}
REPORT_REQUIRED_KEYS = ('schema', 'status', 'mode', 'duration_s', 'contract_sha256',
    'profile_canonical_sha256', 'conditions_sha256', 'boot_id', 'motor_power_epoch',
    'source_manifest_sha256', 'topology_by_port', 'pacing', 'errors', 'failure_retained',
    'completed_cycles', 'all_cycles_passed', 'first_release_monotonic_ns', 'motor_enable_sent',
    'type1_sent', 'positive_gain_sent', 'learned_targets_attempted', 'terminal_stop',
    'physical_cutoff_required', 'restoration_complete', 'physical_post_trial_observation')
TERMINAL_STOP_REQUIRED_KEYS = ('stop_confirmed', 'confirmed_ids', 'unconfirmed_ids', 'ambiguous_ids',
                               'fault_by_id', 'physical_cutoff_required', 'finished_monotonic_ns')
_ADMISSION_TOKEN = object()


def need(value, message):
    if not value:
        raise ValueError(message)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def canonical_sha256(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def _sha(value, label):
    need(type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value),
         'Exact lowercase SHA256 required: '+label)
    return value


def _text(value, label):
    need(type(value) is str and 0 < len(value) <= 4096 and value.strip() == value,
         'Explicit nonblank text required: '+label)
    return value


def _finite(value, label):
    need(type(value) in (int, float) and math.isfinite(value), 'Finite number required: '+label)
    return float(value)


def _stamp(value, label):
    need(type(value) is int and value > 0, 'Positive monotonic nanoseconds required: '+label)
    return value


def _ref(value, label):
    need(type(value) is dict and set(value) == {'path', 'sha256'} and type(value['path']) is str and
         Path(value['path']).is_absolute(), 'Absolute pinned reference required: '+label)
    _sha(value['sha256'], label)
    return {'path': value['path'], 'sha256': value['sha256']}


def contract_sha256(contract):
    """Everything except mode, duration and evidence lives in the contract."""
    return canonical_sha256(contract)


def profile_canonical_sha256(profile):
    return canonical_sha256(profile)


def command_phase_max_with_lead_us(lead):
    """Largest F1 offset whose worst cycle end still leaves the next cycle's F3 pre-arm window."""
    return (20_000-sum(COMMAND_PHASE_BUDGET_US.values())-sum(PREARMED_TAIL_BUDGET_US.values())-
            max(lead, PREARM_WORK_US))


def pacing_options(pacing, message='Fixed four-bus boxed pacing differs'):
    """Exact R8 PACING plus only explicitly selected opt-in keys; returns the selection."""
    need(type(pacing) is dict and set(pacing) <= set(PACING) | set(PACING_OPTIONS) and
         all(key in pacing and pacing[key] == value for key, value in PACING.items()), message)
    options = {key: pacing[key] for key in PACING_OPTIONS if key in pacing}
    offset, lead = options.get('command_phase_offset_us'), options.get('prearmed_hold_lead_us')
    need(offset is None or (type(offset) is int and COMMAND_PHASE_MIN_US <= offset <= COMMAND_PHASE_MAX_US),
         'Command phase offset must be an integer %d..%d us (K + %d us output budget <= 20 ms)' % (
             COMMAND_PHASE_MIN_US, COMMAND_PHASE_MAX_US, sum(COMMAND_PHASE_BUDGET_US.values())))
    need(lead is None or (type(lead) is int and PREARMED_LEAD_US[0] <= lead <= PREARMED_LEAD_US[1]),
         'Pre-armed hold lead must be an integer %d..%d us' % PREARMED_LEAD_US)
    need(offset is None or lead is None or offset <= command_phase_max_with_lead_us(lead),
         'Command phase offset with a pre-armed hold lead must keep K + %d us + max(L, %d us) <= 20 ms '
         '(K <= %d us at L = %d us)' % (sum(COMMAND_PHASE_BUDGET_US.values())+sum(PREARMED_TAIL_BUDGET_US.values()),
                                        PREARM_WORK_US, command_phase_max_with_lead_us(lead or 0), lead or 0))
    need(all(options[key] is True for key in ('decode_once', 'gc_freeze') if key in options),
         'A selected decode_once/gc_freeze option must be exactly true')
    return options


def selected_pacing(options=None):
    """Contract pacing: PACING plus only the selected options (None/False omitted)."""
    pacing = copy.deepcopy(PACING)
    need(options is None or type(options) is dict, 'Pacing options must be a dict')
    for key, value in (options or {}).items():
        need(key in PACING_OPTIONS, 'Unknown pacing option: '+str(key))
        if value is not None and value is not False:
            pacing[key] = value
    pacing_options(pacing)
    return pacing


def mapping(value):
    need(type(value) is dict and set(value) == set(PORTS), 'Four physical ports required')
    groups = {port: Group(port, tuple(value[port])) for port in PORTS}
    need(sorted(mid for g in groups.values() for mid in g.ids) == list(IDS), 'Exact3/3/3/3 ID topology required')
    return {port: list(groups[port].ids) for port in PORTS}


def enable_order(topology_by_port):
    """One axis at a time, interleaved across ports by group (1,4,7,10,2,...)."""
    groups = sorted(topology_by_port.values())
    return [group[k] for k in range(3) for group in groups]


def validate_geometry(geometry):
    """Pinned reviewed two-bus boxed LP profile: shape, review and pure boxed caps only."""
    need(type(geometry) is dict and geometry.get('schema') == LP.SCHEMA_V3 and
         geometry.get('approved_for_supported_policy_output') is True and geometry.get('blockers') == [],
         'Reviewed approved V3 boxed geometry profile required')
    LP._structure(copy.deepcopy(geometry))
    LP._review(geometry['review'], 'APPROVED_SUPPORTED_CHARACTERIZATION')
    native = LP.preauthorized_native_boxed_sequence_selected(geometry)
    ordinary = LP.preauthorized_ordinary_boxed_sequence_selected(geometry)
    need(native or ordinary, 'Geometry must come from a native or ordinary boxed preauthorized profile')
    labels = {2: LP.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, 10: LP.SUPPORTED_POLICY_PROBE_10S_AFTER_2S}
    need(geometry.get('local_characterization') == LP.LOCAL_RELATIVE_SUPPORTED and
         geometry.get('duration_s') in labels and
         geometry.get('diagnostic_timing_acceptance') == labels[geometry['duration_s']],
         'Geometry must be the reviewed local boxed 2s/10s profile')
    LP._preauthorized_boxed_axis_caps(geometry)
    for key in LP.IDS:
        axis = geometry['axes'][key]
        need(axis['uncertainty_rad'] is None, 'Local geometry keeps unknown absolute uncertainty: ID'+key)
        lo, hi = geometry['start_pose_bounds'][key]
        need(axis['physical_lower_rad']+MARGIN_RAD <= lo < hi <= axis['physical_upper_rad']-MARGIN_RAD,
             'Reviewed start bounds exceed effective local bounds: ID'+key)
    for key in (*IMU_KEYS, *RAMP_KEYS):
        need(_finite(geometry.get(key), key) > 0, 'Positive reviewed '+key+' required')
    _text(geometry.get('assembly_id'), 'geometry assembly ID')
    return 'native890' if native else 'ordinary900'


def _axis(mid, plan_axis, geometry):
    key = str(mid)
    g, start = geometry['axes'][key], geometry['start_pose_bounds'][key]
    need(g['uid'] == plan_axis['uid'] and g['sign'] == plan_axis['sign'] and
         g['offset_rad'] == plan_axis['nominal_offset_rad'],
         'Reviewed geometry UID/sign/nominal offset differs from current plan: ID'+key)
    local_lo, local_hi = plan_axis['local_bounds_rad']
    row = {'uid': g['uid'], 'physical_port': plan_axis['physical_port'], 'sign': g['sign'],
           'nominal_offset_rad': g['offset_rad'], 'fixed_offset_rad': plan_axis['fixed_offset_rad'],
           'reference_turns': plan_axis['reference_turns'],
           'reviewed_physical_lower_rad': g['physical_lower_rad'],
           'reviewed_physical_upper_rad': g['physical_upper_rad'],
           'plan_local_lower_rad': local_lo, 'plan_local_upper_rad': local_hi,
           **{name: g[name] for name in CAP_KEYS}}
    _derive_bounds(row)
    return row, list(start)


def _derive_bounds(row):
    """Intersect reviewed and current local +-3deg; effective = LP local margin inside."""
    row['physical_lower_rad'] = max(row['reviewed_physical_lower_rad'], row['plan_local_lower_rad'])
    row['physical_upper_rad'] = min(row['reviewed_physical_upper_rad'], row['plan_local_upper_rad'])
    row['lower_rad'] = row['physical_lower_rad']+MARGIN_RAD
    row['upper_rad'] = row['physical_upper_rad']-MARGIN_RAD
    need(row['lower_rad'] < row['upper_rad'], 'Intersected local bounds are empty')
    raw = sorted(((row['lower_rad']-row['fixed_offset_rad'])/row['sign'],
                  (row['upper_rad']-row['fixed_offset_rad'])/row['sign']))
    row['raw_lower_rad'], row['raw_upper_rad'] = raw
    need(-RAW_LIMIT_RAD <= raw[0] < raw[1] <= RAW_LIMIT_RAD, 'Raw bounds exceed the Type1 position field')
    return row


def _lp_view(contract):
    # LP route selections are copied only so the existing pure ordinary900
    # boxed scope re-proves the identical numeric contract on this dict (the
    # pattern of policy_checked_dispatch.scope). Duration is evaluated as the
    # 2s label because LP binds duration only to its two-bus diagnostic label;
    # the four-bus 2/10/20 ladder and braking reserve are validated separately.
    timing = contract['timing']
    view = {key: copy.deepcopy(timing[key]) for key in timing if key != 'voltage_max_age_ms'}
    view.update(schema=LP.SCHEMA_V3, scope='supported_characterization_only', duration_s=2.,
        diagnostic_timing_acceptance=LP.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
        preauthorized_ordinary_boxed_sequence=True, native_phase_pair=contract['pacing']['native_phase_pair'],
        local_characterization=LP.LOCAL_RELATIVE_SUPPORTED, watchdog_review_policy=LP.COMMAND_LOSS_ONLY_SUPPORTED,
        native_target_fk_cache=True, model_backend=LP.SCALAR_BACKEND, voltage_overlap=True,
        voltage_pipeline=True, prepare_voltage_before_feedback_publication=True,
        request_gap_us=contract['pacing']['request_gap_us'], request_window=contract['pacing']['request_window'],
        axes={key: {name: row[name] for name in (*CAP_KEYS, 'physical_lower_rad', 'physical_upper_rad')}
              for key, row in contract['axes'].items()},
        start_pose_bounds=copy.deepcopy(contract['start_pose_bounds']))
    return view


def rebind_window_to_current_capture(geometry, plan):
    """Explicit: re-centre only the reviewed +-3deg window and +-0.5deg start
    interval on the current lineage pose (jetson-best20-settings precedent).

    Origin, sign, gains and every cap stay the reviewed values. Physical
    clearance of the new window is not inferred here; admission still needs
    the direct-human all12_local_plus_minus3deg_clear record.
    """
    validate_geometry(copy.deepcopy(geometry))
    rebound = copy.deepcopy(geometry)
    half_start = math.radians(.5)
    record = {}
    for mid in IDS:
        key = str(mid)
        q = _finite(plan['provenance']['model_rad_by_id'][key], 'lineage q')
        lo, hi = (_finite(v, 'plan local bound') for v in plan['axes'][key]['local_bounds_rad'])
        need(lo < q < hi and hi-lo <= 2*math.radians(3)+1e-12, 'Plan local window must be the current +-3deg: ID'+key)
        old = geometry['axes'][key]
        rebound['axes'][key]['physical_lower_rad'], rebound['axes'][key]['physical_upper_rad'] = lo, hi
        rebound['start_pose_bounds'][key] = [q-half_start, q+half_start]
        record[key] = {'reviewed_window_rad': [old['physical_lower_rad'], old['physical_upper_rad']],
                       'reviewed_start_rad': list(geometry['start_pose_bounds'][key]),
                       'current_window_rad': [lo, hi], 'current_start_rad': [q-half_start, q+half_start],
                       'centre_shift_deg': math.degrees(q - (old['physical_lower_rad']+old['physical_upper_rad'])/2)}
    validate_geometry(rebound)
    return rebound, {'selected': True, 'source': 'current_lineage_capture_pose',
        'kept_reviewed': ['uid', 'sign', 'offset', 'kp', 'kd', 'all_caps', 'imu_limits', 'ramps'],
        'requires_direct_human': 'all12_local_plus_minus3deg_clear', 'physical_clearance_inferred': False,
        'axes': record, 'max_abs_centre_shift_deg': max(abs(r['centre_shift_deg']) for r in record.values())}


def build_contract(plan, lineage, geometry, geometry_ref, *, rebind_window=False, options=None):
    """Pure: current four-bus plan + reviewed boxed geometry -> fixed contract.

    ``options`` selects opt-in PACING_OPTIONS; None keeps the R8 contract.
    """
    need(type(rebind_window) is bool, 'Explicit window rebinding boolean required')
    pacing = selected_pacing(options)
    rebinding = None
    if rebind_window:
        geometry, rebinding = rebind_window_to_current_capture(geometry, plan)
    route = validate_geometry(geometry)
    model_bridge._plan_axes(plan)
    by_port = mapping(plan['topology_by_port'])
    provenance = plan['provenance']
    axes, start = {}, {}
    for mid in IDS:
        axes[str(mid)], start[str(mid)] = _axis(mid, plan['axes'][str(mid)], geometry)
    contract = {'schema': PROFILE_SCHEMA, 'scope': SCOPE,
        'boot_id': provenance['boot_id'], 'motor_power_epoch': provenance['motor_power_epoch'],
        'assembly_id': geometry['assembly_id'], 'topology_by_port': by_port,
        'uids_by_id': dict(provenance['uids_by_id']), 'enable_order_ids': enable_order(by_port),
        'axes': axes, 'start_pose_bounds': start, 'timing': copy.deepcopy(TIMING),
        'pacing': pacing, 'watchdog': dict(WATCHDOG), 'enable': dict(ENABLE),
        'terminal_stop': dict(TERMINAL_STOP),
        'imu_limits': {key: geometry[key] for key in IMU_KEYS},
        'ramps': {key: geometry[key] for key in RAMP_KEYS},
        'model_plan': copy.deepcopy(plan), 'model_plan_canonical_sha256': plan['plan_canonical_sha256'],
        'model_artifacts': copy.deepcopy(plan['model_profile'].get('artifacts', {})),
        'lineage': {name: _ref(ref, name) for name, ref in lineage.items()},
        'source_manifest': _ref(plan['source_binding']['four_bus_source_manifest'], 'four-bus source manifest'),
        'axis_geometry': {'profile': _ref(geometry_ref, 'axis geometry'), 'route': route,
            'reviewed_duration_s': geometry['duration_s'],
            'today_two_bus_boxed_profile': TODAY_TWO_BUS_BOXED_PROFILE_SHA256.get(geometry_ref['sha256']),
            'two_bus_timing_or_permission_reused': False,
            **({'window_rebound_to_current_capture': rebinding} if rebinding else {})},
        'authorized_modes': list(MODES), 'authorized_durations_s': list(DURATIONS),
        'lp_scope_checks': ['_preauthorized_boxed_axis_caps', '_preauthorized_ordinary_boxed_sequence_scope']}
    validate_contract(contract)
    return contract


def validate_contract(contract):
    need(type(contract) is dict and set(contract) == CONTRACT_KEYS and
         contract['schema'] == PROFILE_SCHEMA and contract['scope'] == SCOPE, 'Exact four-bus Type1 contract required')
    for key, expected in (('timing', TIMING), ('watchdog', WATCHDOG),
                          ('enable', ENABLE), ('terminal_stop', TERMINAL_STOP)):
        need(contract[key] == expected, 'Fixed four-bus boxed '+key+' differs')
    pacing_options(contract['pacing'])
    need(contract['timing']['voltage_max_age_ms']*1_000_000 == V3_VOLTAGE_MAX_AGE_NS,
         'Voltage cache age differs from the original runtime')
    need(contract['authorized_modes'] == list(MODES) and contract['authorized_durations_s'] == list(DURATIONS),
         'Fixed mode/duration ladder required')
    _text(contract['boot_id'], 'boot ID'); _text(contract['motor_power_epoch'], 'power epoch')
    need(contract['motor_power_epoch'] not in ('UNKNOWN', 'NOT_INFERRED_FROM_JETSON_BOOT'),
         'Explicit power epoch required')
    _text(contract['assembly_id'], 'assembly ID')
    by_port = mapping(contract['topology_by_port'])
    need(by_port == contract['topology_by_port'] and contract['enable_order_ids'] == enable_order(by_port),
         'Topology/enable order differs')
    plan = contract['model_plan']
    plan_axes = model_bridge._plan_axes(plan)
    need(plan['plan_canonical_sha256'] == contract['model_plan_canonical_sha256'] and
         plan['topology_by_port'] == by_port and plan['provenance']['boot_id'] == contract['boot_id'] and
         plan['provenance']['motor_power_epoch'] == contract['motor_power_epoch'] and
         plan['provenance']['uids_by_id'] == contract['uids_by_id'] and
         plan['model_profile'].get('artifacts', {}) == contract['model_artifacts'] and
         _ref(plan['source_binding']['four_bus_source_manifest'], 'source') == contract['source_manifest'],
         'Embedded model plan differs from contract')
    lineage = contract['lineage']
    need(type(lineage) is dict and {'topology', 'events'} <= set(lineage) and
         all(_ref(ref, name) == ref for name, ref in lineage.items()) and
         lineage['topology'] == plan['provenance']['topology_capture'] and
         lineage['events'] == plan['provenance']['raw_events'], 'Lineage capture differs from plan')
    geometry = contract['axis_geometry']
    need(type(geometry) is dict and _ref(geometry.get('profile'), 'geometry') == geometry['profile'] and
         geometry.get('route') in ('native890', 'ordinary900') and geometry.get('reviewed_duration_s') in (2, 10) and
         geometry.get('today_two_bus_boxed_profile') ==
         TODAY_TWO_BUS_BOXED_PROFILE_SHA256.get(geometry['profile']['sha256']) and
         geometry.get('two_bus_timing_or_permission_reused') is False, 'Axis geometry provenance differs')
    rebound = geometry.get('window_rebound_to_current_capture')
    if rebound is not None:
        need(type(rebound) is dict and rebound.get('selected') is True and
             rebound.get('physical_clearance_inferred') is False and
             rebound.get('requires_direct_human') == 'all12_local_plus_minus3deg_clear' and
             set(rebound.get('axes', {})) == set(LP.IDS) and
             all(contract['axes'][key]['reviewed_physical_lower_rad'] == row['current_window_rad'][0] and
                 contract['axes'][key]['reviewed_physical_upper_rad'] == row['current_window_rad'][1] and
                 contract['start_pose_bounds'][key] == row['current_start_rad']
                 for key, row in rebound['axes'].items()), 'Window rebinding record differs from contract')
    for key in IMU_KEYS:
        need(_finite(contract['imu_limits'].get(key), key) > 0, 'Positive IMU limit required')
    need(set(contract['imu_limits']) == set(IMU_KEYS) and set(contract['ramps']) == set(RAMP_KEYS) and
         all(_finite(contract['ramps'][key], key) > 0 for key in RAMP_KEYS), 'Ramp/IMU fields differ')
    axes, start = contract['axes'], contract['start_pose_bounds']
    need(type(axes) is dict and set(axes) == set(LP.IDS) and type(start) is dict and set(start) == set(LP.IDS),
         'Twelve axes and start bounds required')
    for mid in IDS:
        key, row = str(mid), axes[str(mid)]
        need(type(row) is dict and set(row) == AXIS_FIELDS, 'Exact four-bus axis fields required: ID'+key)
        port, sign, fixed, local_lo, local_hi, _ = plan_axes[mid]
        need(row['uid'] == contract['uids_by_id'][key] and row['physical_port'] == port and
             mid in by_port[port] and row['sign'] == sign and row['fixed_offset_rad'] == fixed and
             row['plan_local_lower_rad'] == local_lo and row['plan_local_upper_rad'] == local_hi and
             row['nominal_offset_rad'] == plan['axes'][key]['nominal_offset_rad'] and
             row['reference_turns'] == plan['axes'][key]['reference_turns'], 'Axis differs from plan: ID'+key)
        derived = _derive_bounds({name: row[name] for name in ('sign', 'fixed_offset_rad',
            'reviewed_physical_lower_rad', 'reviewed_physical_upper_rad', 'plan_local_lower_rad', 'plan_local_upper_rad')})
        need(all(row[name] == value for name, value in derived.items()), 'Derived bounds differ: ID'+key)
        index = shadow.CAN_ORDER.index(mid)
        need(shadow.LOWER[index] <= row['physical_lower_rad'] < row['physical_upper_rad'] <= shadow.UPPER[index],
             'Bounds exceed model range: ID'+key)
        need(type(start[key]) is list and len(start[key]) == 2, 'Start interval required: ID'+key)
        lo, hi = (_finite(v, 'start bound') for v in start[key])
        need(row['lower_rad'] <= lo < hi <= row['upper_rad'], 'Start bounds exceed effective bounds: ID'+key)
        q = _finite(plan['provenance']['model_rad_by_id'][key], 'lineage q')
        need(lo <= q <= hi, 'Lineage pose outside reviewed start bounds: ID'+key)
        need(row['max_estimated_pd_torque_nm'] <= row['max_measured_torque_nm'] and
             row['max_command_velocity_rad_s'] <= row['max_measured_velocity_rad_s'],
             'PD/velocity budget exceeds measured monitor: ID'+key)
    need(contract['lp_scope_checks'] == ['_preauthorized_boxed_axis_caps', '_preauthorized_ordinary_boxed_sequence_scope'],
         'LP scope check record differs')
    view = _lp_view(contract)
    LP._preauthorized_boxed_axis_caps(view)
    LP._preauthorized_ordinary_boxed_sequence_scope(view)
    return contract


def _brake_s(contract):
    return max(row['max_command_velocity_rad_s']/row['max_command_acceleration_rad_s2']
               for row in contract['axes'].values())


def _check_pose(contract, model_rad_by_id, label):
    need(type(model_rad_by_id) is dict and set(model_rad_by_id) == set(LP.IDS), 'Twelve current angles required')
    for key, row in contract['axes'].items():
        q = _finite(model_rad_by_id[key], label)
        lo, hi = contract['start_pose_bounds'][key]
        need(lo <= q <= hi and row['lower_rad'] <= q <= row['upper_rad'],
             label+' outside unchanged start/local bounds: ID'+key)


def required_evidence(mode, duration_s):
    need(mode in MODES and duration_s in DURATIONS, 'Mode/duration outside the four-bus boxed ladder')
    predecessor = PREDECESSOR.get((mode, duration_s))
    diagnostic = mode == 'zero_gain_timing' or duration_s == 20
    return predecessor, diagnostic


def validate_profile(profile):
    """Pure re-validation of a prepared profile; evidence files are not reread here."""
    need(type(profile) is dict and set(profile) == PROFILE_KEYS and profile['schema'] == PROFILE_SCHEMA and
         profile['status'] == 'PREPARED_FILE_ONLY_NOT_ADMITTED', 'Exact prepared four-bus profile required')
    need(all(profile[key] is value for key, value in NO_GRANTS.items()), 'Profile grants or claims nothing')
    contract = validate_contract(profile['contract'])
    need(profile['contract_sha256'] == contract_sha256(contract), 'Contract SHA256 differs')
    mode, duration = profile['mode'], profile['duration_s']
    need(type(duration) is int, 'Integer duration required')
    predecessor, diagnostic = required_evidence(mode, duration)
    ramps = contract['ramps']
    need(ramps['startup_duration_s']+ramps['policy_ramp_s']+ramps['stop_duration_s']+_brake_s(contract)+.04 < duration,
         'Duration does not reserve worst-case braking')
    _text(profile['prepared_at'], 'prepared time')
    evidence = profile['evidence']
    need(type(evidence) is dict and set(evidence) == {'current_capture', 'stop_proxy_diagnostic', 'predecessor'},
         'Exact evidence fields required')
    capture = evidence['current_capture']
    need(type(capture) is dict and set(capture) == {'topology', 'events', 'started_monotonic_ns',
         'finished_monotonic_ns', 'model_rad_by_id', 'fixed_offset_rad_by_id', 'boot_id', 'motor_power_epoch'},
         'Current capture summary required')
    _ref(capture['topology'], 'current capture'); _ref(capture['events'], 'current events')
    begin, end = _stamp(capture['started_monotonic_ns'], 'capture'), _stamp(capture['finished_monotonic_ns'], 'capture')
    need(begin < end and capture['boot_id'] == contract['boot_id'] and
         capture['motor_power_epoch'] == contract['motor_power_epoch'] and
         capture['fixed_offset_rad_by_id'] == {k: r['fixed_offset_rad'] for k, r in contract['axes'].items()},
         'Current capture boot/epoch/branch differs')
    _check_pose(contract, capture['model_rad_by_id'], 'Current capture pose')
    pred, diag = evidence['predecessor'], evidence['stop_proxy_diagnostic']
    need((pred is None) == (predecessor is None) and (diag is None) == (not diagnostic),
         'Required predecessor/diagnostic evidence differs for this mode and duration')
    if pred is not None:
        need(type(pred) is dict and set(pred) == {'report', 'mode', 'duration_s', 'status', 'contract_sha256',
             'boot_id', 'motor_power_epoch', 'source_manifest_sha256', 'first_release_monotonic_ns',
             'terminal_stop_finished_monotonic_ns'}, 'Predecessor summary required')
        _ref(pred['report'], 'predecessor report')
        need(pred['mode'] == predecessor[0] and (predecessor[1] is None or pred['duration_s'] == predecessor[1]) and
             pred['duration_s'] in DURATIONS and pred['status'] == COMPLETE_STATUS[pred['mode']] and
             pred['contract_sha256'] == profile['contract_sha256'] and pred['boot_id'] == contract['boot_id'] and
             pred['motor_power_epoch'] == contract['motor_power_epoch'] and
             pred['source_manifest_sha256'] == contract['source_manifest']['sha256'],
             'Predecessor is not the required complete same-contract run')
        after = _stamp(pred['terminal_stop_finished_monotonic_ns'], 'predecessor STOP')
        need(_stamp(pred['first_release_monotonic_ns'], 'predecessor release') < after < begin,
             'Current capture must follow the predecessor terminal STOP')
    if diag is not None:
        need(type(diag) is dict and set(diag) == {'report', 'pinned_capture', 'completed_cycles',
             'first_release_monotonic_ns', 'workers_joined_monotonic_ns'}, 'Diagnostic summary required')
        _ref(diag['report'], 'diagnostic report')
        pinned = _ref(diag['pinned_capture'], 'diagnostic capture')
        need(pinned in (contract['lineage']['topology'], capture['topology']) and
             diag['completed_cycles'] == STOP_PROXY_CYCLES, 'Diagnostic must be a 501-cycle run on this capture lineage')
        first = _stamp(diag['first_release_monotonic_ns'], 'diagnostic release')
        joined = _stamp(diag['workers_joined_monotonic_ns'], 'diagnostic join')
        need(first < joined, 'Diagnostic clocks differ')
        if pred is not None:
            need(first > pred['terminal_stop_finished_monotonic_ns'], 'Diagnostic must follow the predecessor run')
        if pinned != capture['topology']:
            need(begin > joined, 'Current capture must follow the diagnostic')
    return profile


def assemble_profile(contract, mode, duration_s, evidence, *, prepared_at):
    profile = {'schema': PROFILE_SCHEMA, 'status': 'PREPARED_FILE_ONLY_NOT_ADMITTED', 'mode': mode,
               'duration_s': duration_s, 'contract': copy.deepcopy(contract),
               'contract_sha256': contract_sha256(contract), 'evidence': copy.deepcopy(evidence),
               'prepared_at': prepared_at, **NO_GRANTS}
    return validate_profile(profile)


def validate_stop_proxy_report(report, *, capture_refs, topology_by_port, after_ns=None):
    """Four-bus R8-configuration STOP-proxy 501/501 as a timing prerequisite only.

    It never qualifies Type1; its own no-grant flags must remain false.
    """
    need(type(report) is dict and report.get('schema') == STOP_PROXY_SCHEMA and
         report.get('status') == STOP_PROXY_STATUS and report.get('errors') == [] and
         report.get('failure_retained') is False and report.get('source_and_input_pins_unchanged') is True and
         report.get('physical_post_trial_observation') is None, 'Complete four-bus STOP-proxy diagnostic required')
    need(all(report.get(key) is False for key in ('output_allowed', 'approved_for_runtime',
         'live_type1_qualified', 'motor_enable_sent', 'learned_targets_sent')),
         'STOP-proxy diagnostic must have sent and granted nothing')
    pins = report.get('input_sha256')
    need(type(pins) is dict, 'Diagnostic input pins required')
    matched = [ref for ref in capture_refs if pins.get(ref['path']) == ref['sha256']]
    need(len(matched) >= 1, 'Diagnostic is not bound to this capture lineage')
    m = report.get('measurement')
    need(type(m) is dict and m.get('schema') == STOP_PROXY_PIPELINE_SCHEMA and m.get('status') == STOP_PROXY_STATUS and
         m.get('cycles') == STOP_PROXY_CYCLES and m.get('completed_cycles') == STOP_PROXY_CYCLES and
         m.get('primary_error') is None and m.get('failure_retained') is False and
         m.get('physical_cutoff_required') is False and m.get('per_cycle_requests') == 28 and
         m.get('request_gap_ns') == 900_000 and m.get('request_window') == 3 and
         m.get('absolute_deadline_ns') == 20_000_000 and m.get('request_schedule') == STOP_PROXY_SCHEDULE and
         m.get('boundary_current_checks_selected') is True and m.get('final_gate_input_identity_selected') is True and
         all(m.get(key) is False for key in ('output_allowed', 'motor_enable_sent', 'learned_targets_sent',
                                             'positive_gain_sent')), 'R8-configuration 501-cycle STOP proxy required')
    need(m.get('physical_groups') == topology_by_port, 'Diagnostic topology differs')
    workers = m.get('worker_settings')
    need(type(workers) is dict and all(type(workers.get(p)) is dict and
         workers[p].get('cpu_mask') == PACING['can_owner_cpu_by_port'][p] and
         workers[p].get('timer_slack_ns') == 1000 and workers[p].get('file_only_mock_readback') is False
         for p in PORTS) and type(workers.get('imu')) is dict and workers['imu'].get('cpu_mask') == PACING['imu_mask'] and
         workers['imu'].get('file_only_mock_readback') is False, 'Actual R8 worker placement readback required')
    settings = m.get('operating_settings')
    need(type(settings) is dict and settings.get('main_cpu_mask') == [4] and settings.get('nice') == -10 and
         settings.get('timer_slack_ns') == 1000 and type(settings.get('switch_interval_s')) is float and
         abs(settings['switch_interval_s']-1e-4) <= 1e-12 and settings.get('single_thread_math_verified') is True and
         settings.get('power_scope_verified') is True and settings.get('gc_deferred_during_cycles') is True,
         'Actual R8 operating readback required')
    records = m.get('records')
    need(type(records) is list and len(records) == STOP_PROXY_CYCLES and
         [r.get('cycle') for r in records] == list(range(STOP_PROXY_CYCLES)) and
         all(r.get('completed') is True and r.get('actual_request_count') == 28 for r in records),
         'All 501 completed 28-request cycles required')
    first = _stamp(min(r.get('release_ns', 0) for r in records), 'diagnostic release')
    last = max(_stamp(r.get('cycle_end_ns'), 'diagnostic cycle end') for r in records)
    joined = _stamp(m.get('all_original_workers_joined_monotonic_ns'), 'diagnostic join')
    need(first < last <= joined, 'Diagnostic clocks differ')
    if after_ns is not None:
        need(first > after_ns, 'Diagnostic must start after the predecessor terminal STOP')
    cleanup = m.get('cleanup')
    need(type(cleanup) is dict and set(cleanup) == set(PORTS), 'Terminal STOP per port required')
    for port in PORTS:
        row, ids = cleanup[port], topology_by_port[port]
        need(type(row) is dict and row.get('complete') is True and row.get('confirmed_ids') == ids and
             row.get('ambiguous_ids') == [] and row.get('physical_cutoff_required') is False and
             row.get('fault_by_id') == {str(mid): 0 for mid in ids}, 'Diagnostic terminal STOP unconfirmed: '+port)
    return {'pinned_capture': matched[0], 'completed_cycles': STOP_PROXY_CYCLES,
            'first_release_monotonic_ns': first, 'workers_joined_monotonic_ns': joined}


def validate_type1_report(report, *, contract, contract_digest, mode, duration_s=None):
    """Runner report as predecessor: complete, same boot/epoch/source/contract, all-12 STOP."""
    need(type(report) is dict and all(key in report for key in REPORT_REQUIRED_KEYS) and
         report['schema'] == REPORT_SCHEMA, 'Four-bus Type1 run report required')
    need(report['mode'] == mode and report['status'] == COMPLETE_STATUS[mode] and
         report['duration_s'] in DURATIONS and (duration_s is None or report['duration_s'] == duration_s),
         'Predecessor mode/duration/status differs')
    # A chained step uses exactly its predecessor's opt-in options (also bound by the contract SHA256).
    need(type(report['pacing']) is dict and pacing_options(contract['pacing']) ==
         {key: report['pacing'][key] for key in PACING_OPTIONS if key in report['pacing']},
         'Predecessor opt-in pacing options differ from this contract')
    need(report['contract_sha256'] == contract_digest and report['boot_id'] == contract['boot_id'] and
         report['motor_power_epoch'] == contract['motor_power_epoch'] and
         report['source_manifest_sha256'] == contract['source_manifest']['sha256'] and
         report['topology_by_port'] == contract['topology_by_port'] and report['pacing'] == contract['pacing'],
         'Predecessor boot/epoch/source/contract/pacing differs')
    _sha(report['profile_canonical_sha256'], 'predecessor profile'); _sha(report['conditions_sha256'], 'conditions')
    learned = mode == 'learned_boxed'
    need(report['errors'] == [] and report['failure_retained'] is False and report['all_cycles_passed'] is True and
         report['physical_cutoff_required'] is False and report['restoration_complete'] is True and
         report['physical_post_trial_observation'] is None and report['motor_enable_sent'] is True and
         report['type1_sent'] is True and report['positive_gain_sent'] is learned and
         report['learned_targets_attempted'] is learned, 'Predecessor run is not complete and truthful')
    cycles = report['completed_cycles']
    need(type(cycles) is int and 0 < cycles <= report['duration_s']*50, 'Predecessor cycle count invalid')
    stop = report['terminal_stop']
    need(type(stop) is dict and all(key in stop for key in TERMINAL_STOP_REQUIRED_KEYS) and
         stop['stop_confirmed'] is True and stop['confirmed_ids'] == list(IDS) and stop['unconfirmed_ids'] == [] and
         stop['ambiguous_ids'] == [] and stop['physical_cutoff_required'] is False and
         stop['fault_by_id'] == {str(mid): 0 for mid in IDS}, 'Predecessor all-12 terminal STOP unconfirmed')
    first = _stamp(report['first_release_monotonic_ns'], 'predecessor release')
    finished = _stamp(stop['finished_monotonic_ns'], 'predecessor STOP')
    need(first < finished, 'Predecessor clocks differ')
    return {'mode': mode, 'duration_s': report['duration_s'], 'status': report['status'],
            'contract_sha256': report['contract_sha256'], 'boot_id': report['boot_id'],
            'motor_power_epoch': report['motor_power_epoch'],
            'source_manifest_sha256': report['source_manifest_sha256'],
            'first_release_monotonic_ns': first, 'terminal_stop_finished_monotonic_ns': finished}


def conditions_record(*, user_statement, user_reply_id, boot_id, motor_power_epoch, contract_sha256,
                      authorized_modes, authorized_durations_s, motor_40v_on, box_supports_body,
                      four_feet_touch_floor, all12_local_plus_minus3deg_clear, hands_off,
                      immediate_40v_cutoff, other_drive_tools_stopped, box_will_remain,
                      box_removal_allowed, load_transfer_allowed, standing_allowed, walking_allowed,
                      record_written_at):
    """Only explicit caller arguments; no defaults, no inference, no physical observation."""
    values = dict(motor_40v_on=motor_40v_on, box_supports_body=box_supports_body,
        four_feet_touch_floor=four_feet_touch_floor,
        all12_local_plus_minus3deg_clear=all12_local_plus_minus3deg_clear, hands_off=hands_off,
        immediate_40v_cutoff=immediate_40v_cutoff, other_drive_tools_stopped=other_drive_tools_stopped,
        box_will_remain=box_will_remain, box_removal_allowed=box_removal_allowed,
        load_transfer_allowed=load_transfer_allowed, standing_allowed=standing_allowed,
        walking_allowed=walking_allowed)
    need(all(type(value) is bool for value in values.values()), 'Every condition must be an explicit boolean')
    record = {'schema': CONDITIONS_SCHEMA, 'source': CONDITIONS_SOURCE, 'direct_human': True,
              'synthetic_interaction': False, 'user_statement': user_statement, 'user_reply_id': user_reply_id,
              'boot_id': boot_id, 'motor_power_epoch': motor_power_epoch, 'contract_sha256': contract_sha256,
              'authorized_modes': list(authorized_modes), 'authorized_durations_s': list(authorized_durations_s),
              'record_written_at': record_written_at, 'physical_condition_inferred': False, **values}
    return validate_conditions(record)


def validate_conditions(record):
    need(type(record) is dict and set(record) == CONDITION_KEYS and record['schema'] == CONDITIONS_SCHEMA and
         record['source'] == CONDITIONS_SOURCE and record['direct_human'] is True and
         record['synthetic_interaction'] is False and record['physical_condition_inferred'] is False,
         'Direct-human current condition record required')
    _text(record['user_statement'], 'verbatim user statement')
    need(record['user_reply_id'] is None or _text(record['user_reply_id'], 'user reply reference'), 'Reply reference')
    _text(record['boot_id'], 'boot ID'); _text(record['motor_power_epoch'], 'power epoch')
    _sha(record['contract_sha256'], 'conditions contract')
    _text(record['record_written_at'], 'record time')
    modes, durations = record['authorized_modes'], record['authorized_durations_s']
    need(type(modes) is list and modes and len(set(modes)) == len(modes) and all(m in MODES for m in modes) and
         type(durations) is list and durations and durations == sorted(set(durations)) and
         all(type(d) is int and d in DURATIONS for d in durations), 'Explicit authorized modes/durations required')
    need(all(record[key] is True for key in TRUE_CONDITIONS) and all(record[key] is False for key in FALSE_CONDITIONS),
         'Direct current boxed conditions incomplete; no record or admission')
    return record


def _freeze(value):
    if type(value) is dict:
        return types.MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if type(value) is list:
        return tuple(_freeze(item) for item in value)
    return value


def thaw(value):
    if isinstance(value, types.MappingProxyType):
        return {key: thaw(item) for key, item in value.items()}
    if type(value) is tuple:
        return [thaw(item) for item in value]
    return value


class Admitted:
    """Immutable admission; carries no output token beyond this exact record."""
    __slots__ = ('_token', 'profile', 'conditions', 'profile_canonical_sha256', 'conditions_sha256',
                 'contract_sha256', 'mode', 'duration_s', 'boot_id', 'motor_power_epoch', 'files')

    def __init__(self, token, profile, conditions, files):
        need(token is _ADMISSION_TOKEN, 'Use admit(); admissions cannot be constructed')
        for name, value in (('_token', token), ('profile', _freeze(copy.deepcopy(profile))),
                ('conditions', _freeze(copy.deepcopy(conditions))),
                ('profile_canonical_sha256', profile_canonical_sha256(profile)),
                ('conditions_sha256', canonical_sha256(conditions)),
                ('contract_sha256', profile['contract_sha256']), ('mode', profile['mode']),
                ('duration_s', profile['duration_s']), ('boot_id', profile['contract']['boot_id']),
                ('motor_power_epoch', profile['contract']['motor_power_epoch']), ('files', _freeze(copy.deepcopy(files)))):
            object.__setattr__(self, name, value)

    def __setattr__(self, name, value):
        raise AttributeError('Admitted profile is immutable')

    def __delattr__(self, name):
        raise AttributeError('Admitted profile is immutable')

    @property
    def contract(self):
        return self.profile['contract']

    def plain_profile(self):
        return thaw(self.profile)

    def report_binding(self):
        """Exact fields a runner report copies for predecessor validation."""
        contract = self.profile['contract']
        return {'mode': self.mode, 'duration_s': self.duration_s, 'contract_sha256': self.contract_sha256,
                'profile_canonical_sha256': self.profile_canonical_sha256,
                'conditions_sha256': self.conditions_sha256, 'boot_id': self.boot_id,
                'motor_power_epoch': self.motor_power_epoch,
                'source_manifest_sha256': contract['source_manifest']['sha256'],
                'topology_by_port': thaw(contract['topology_by_port']), 'pacing': thaw(contract['pacing'])}


def verify_admitted(value):
    need(type(value) is Admitted and value._token is _ADMISSION_TOKEN, 'Genuine four-bus admission required')
    profile, conditions = thaw(value.profile), thaw(value.conditions)
    need(profile_canonical_sha256(profile) == value.profile_canonical_sha256 and
         canonical_sha256(conditions) == value.conditions_sha256, 'Admission content changed')
    _admission_checks(profile, conditions)
    return value


def _admission_checks(profile, conditions, *, expected_boot=None, expected_power_epoch=None):
    validate_profile(profile)
    validate_conditions(conditions)
    contract = profile['contract']
    need(conditions['boot_id'] == contract['boot_id'] and conditions['motor_power_epoch'] == contract['motor_power_epoch'],
         'Condition record belongs to a different boot or power epoch')
    need(conditions['contract_sha256'] == profile['contract_sha256'], 'Condition record names a different contract')
    need(profile['mode'] in conditions['authorized_modes'] and profile['duration_s'] in conditions['authorized_durations_s'],
         'Condition record does not authorize this mode/duration')
    if expected_boot is not None:
        need(expected_boot == contract['boot_id'], 'Current boot differs from the admitted profile')
    if expected_power_epoch is not None:
        need(expected_power_epoch == contract['motor_power_epoch'], 'Current power epoch differs from the profile')


def admit(profile, conditions, *, expected_boot=None, expected_power_epoch=None, files=None):
    _admission_checks(profile, conditions, expected_boot=expected_boot, expected_power_epoch=expected_power_epoch)
    files = {} if files is None else {name: _ref(ref, name) for name, ref in files.items()}
    return Admitted(_ADMISSION_TOKEN, profile, conditions, files)


def read_json(path, sha256):
    return json.loads(topology.read_pinned(path, sha256), object_pairs_hook=topology.strict_pairs,
                      parse_constant=topology.bad_constant)


def read_admitted(profile_path, profile_sha256, conditions_path, conditions_sha256, **kwargs):
    profile, conditions = read_json(profile_path, profile_sha256), read_json(conditions_path, conditions_sha256)
    return admit(profile, conditions, files={'profile': {'path': str(profile_path), 'sha256': profile_sha256},
        'conditions': {'path': str(conditions_path), 'sha256': conditions_sha256}}, **kwargs)


class _Pins:
    def __init__(self):
        self.values = {}

    def raw(self, path, sha256):
        path = str(path)
        raw = topology.read_pinned(path, sha256)
        need(self.values.setdefault(path, sha256) == sha256, 'Conflicting pin: '+path)
        return raw

    def json(self, path, sha256):
        return json.loads(self.raw(path, sha256), object_pairs_hook=topology.strict_pairs,
                          parse_constant=topology.bad_constant)

    def envelope(self, path, sha256):
        return {'reference': {'path': str(path), 'sha256': sha256}, 'raw_json': self.raw(path, sha256).decode('utf-8')}

    def events(self, path, sha256):
        return {'reference': {'path': str(path), 'sha256': sha256}, 'raw_jsonl': self.raw(path, sha256).decode('utf-8')}

    def verify(self):
        for path, sha256 in self.values.items():
            topology.read_pinned(path, sha256)


def _plan_inputs(args, pins, capture, events):
    inputs = {'topology': pins.envelope(*capture), 'events': pins.events(*events),
              'calibration': pins.envelope(args.calibration, args.calibration_sha256),
              'model_profile': pins.envelope(args.model_profile, args.model_profile_sha256),
              'source_binding': pins.envelope(args.source_binding, args.source_binding_sha256),
              'mount': pins.envelope(args.mount, args.mount_sha256),
              'bias': pins.envelope(args.bias, args.bias_sha256)}
    need((args.accel_hypothesis is None) == (args.accel_hypothesis_sha256 is None), 'Accel path and SHA together')
    if args.accel_hypothesis is not None:
        inputs['accel_hypothesis'] = pins.envelope(args.accel_hypothesis, args.accel_hypothesis_sha256)
    return inputs


def cli_options(args):
    """Explicit prepare flags -> opt-in pacing options; absent flags select nothing."""
    values = {'command_phase_offset_us': getattr(args, 'command_phase_offset_us', None),
              'decode_once': getattr(args, 'decode_once', False),
              'prearmed_hold_lead_us': getattr(args, 'prearmed_hold_lead_us', None),
              'gc_freeze': getattr(args, 'gc_freeze', False)}
    need(type(values['decode_once']) is bool and type(values['gc_freeze']) is bool,
         'Decode-once/gc-freeze selections must be booleans')
    return values


def build_from_files(args):
    """Read pinned files only; returns (profile, pins). Opens no device/library/model."""
    need(args.mode in MODES and args.duration in DURATIONS, 'Explicit mode/duration required')
    predecessor, diagnostic = required_evidence(args.mode, args.duration)
    pins = _Pins()
    lineage_ref = {'path': str(args.topology), 'sha256': args.topology_sha256}
    inputs = _plan_inputs(args, pins, (args.topology, args.topology_sha256), (args.events, args.events_sha256))
    plan = model_bridge.prepare_model_plan(**inputs)
    need(plan['provenance']['motor_power_epoch'] == args.power_epoch, 'Explicit power epoch differs from capture')
    lineage = {name: inputs[name]['reference'] for name in ('topology', 'events', 'calibration',
               'model_profile', 'source_binding', 'mount', 'bias', *(('accel_hypothesis',) if
               'accel_hypothesis' in inputs else ()))}
    geometry_ref = {'path': str(args.axis_geometry_profile), 'sha256': args.axis_geometry_profile_sha256}
    contract = build_contract(plan, lineage, pins.json(args.axis_geometry_profile, args.axis_geometry_profile_sha256),
                              geometry_ref, rebind_window=getattr(args, 'rebind_window_to_current_capture', False),
                              options=cli_options(args))
    digest = contract_sha256(contract)
    current_ref = {'path': str(args.current_topology), 'sha256': args.current_topology_sha256}
    current_inputs = _plan_inputs(args, pins, (args.current_topology, args.current_topology_sha256),
                                  (args.current_events, args.current_events_sha256))
    current = model_bridge.prepare_model_plan(**current_inputs)
    capture_doc = pins.json(args.current_topology, args.current_topology_sha256)
    topology.validate_topology(capture_doc, expected_boot=contract['boot_id'],
                               expected_power_epoch=contract['motor_power_epoch'])
    need(current['topology_by_port'] == contract['topology_by_port'] and
         current['provenance']['uids_by_id'] == contract['uids_by_id'] and
         all(current['axes'][k]['reference_turns'] == r['reference_turns'] and
             current['axes'][k]['fixed_offset_rad'] == r['fixed_offset_rad'] for k, r in contract['axes'].items()),
         'Current capture topology/identity/branch differs from the contract lineage')
    evidence = {'current_capture': {'topology': current_ref,
        'events': {'path': str(args.current_events), 'sha256': args.current_events_sha256},
        'started_monotonic_ns': capture_doc['started_monotonic_ns'],
        'finished_monotonic_ns': capture_doc['finished_monotonic_ns'],
        'model_rad_by_id': dict(current['provenance']['model_rad_by_id']),
        'fixed_offset_rad_by_id': {k: v['fixed_offset_rad'] for k, v in current['axes'].items()},
        'boot_id': current['provenance']['boot_id'], 'motor_power_epoch': current['provenance']['motor_power_epoch']},
        'stop_proxy_diagnostic': None, 'predecessor': None}
    need((args.predecessor_report is not None) == (predecessor is not None) and
         (args.stop_proxy_report is not None) == diagnostic,
         'Supply exactly the predecessor/diagnostic reports this mode and duration require')
    after = None
    if predecessor is not None:
        report = pins.json(args.predecessor_report, args.predecessor_report_sha256)
        summary = validate_type1_report(report, contract=contract, contract_digest=digest,
                                        mode=predecessor[0], duration_s=predecessor[1])
        evidence['predecessor'] = {'report': {'path': str(args.predecessor_report),
                                              'sha256': args.predecessor_report_sha256}, **summary}
        after = summary['terminal_stop_finished_monotonic_ns']
    if diagnostic:
        report = pins.json(args.stop_proxy_report, args.stop_proxy_report_sha256)
        summary = validate_stop_proxy_report(report, capture_refs=[lineage_ref, current_ref],
                                             topology_by_port=contract['topology_by_port'], after_ns=after)
        evidence['stop_proxy_diagnostic'] = {'report': {'path': str(args.stop_proxy_report),
                                                        'sha256': args.stop_proxy_report_sha256}, **summary}
    profile = assemble_profile(contract, args.mode, args.duration, evidence,
                               prepared_at=datetime.datetime.now().astimezone().isoformat())
    pins.verify()
    return profile, pins


def _fresh_output(value):
    path = Path(value)
    need(path.is_absolute() and not path.exists() and path.parent.is_dir() and
         not any(p.is_symlink() for p in (path, *path.parents)), 'Fresh absolute nonsymlink output file required')
    need(not any((p/'.git').exists() for p in path.parents), 'Output must be outside Git')
    return path


def write_json(path, value):
    raw = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n').encode()
    with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'wb') as handle:
        handle.write(raw); handle.flush(); os.fsync(handle.fileno())
    return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}


def prepare(args):
    output = _fresh_output(args.output)
    profile, pins = build_from_files(args)
    result = {'schema': PREPARATION_SCHEMA, 'status': 'PLAN_ONLY', 'mode': args.mode, 'duration_s': args.duration,
              'contract_sha256': profile['contract_sha256'], 'output': str(output), 'input_sha256': dict(pins.values),
              'axis_geometry': profile['contract']['axis_geometry'], 'evidence': profile['evidence'],
              'hardware_opened': False, 'library_or_model_loaded': False, **NO_GRANTS}
    options = pacing_options(profile['contract']['pacing'])
    if options:
        result['pacing_options'] = options
    if not args.prepare:
        return result
    written = write_json(_fresh_output(args.output), profile)
    pins.verify()
    result.update(status='PREPARED_FILE_ONLY_PROFILE', profile=written)
    return result


def record_conditions(args):
    output = _fresh_output(args.output)
    flag = lambda name: {'true': True, 'false': False}[getattr(args, name)]
    record = conditions_record(user_statement=args.user_statement, user_reply_id=args.user_reply_id,
        boot_id=args.boot_id, motor_power_epoch=args.power_epoch, contract_sha256=args.contract_sha256,
        authorized_modes=args.authorize_mode, authorized_durations_s=sorted(set(args.authorize_duration)),
        record_written_at=datetime.datetime.now().astimezone().isoformat(),
        **{name: flag(name) for name in (*TRUE_CONDITIONS, *FALSE_CONDITIONS)})
    result = {'schema': CONDITIONS_SCHEMA, 'status': 'PLAN_ONLY', 'record': record, 'output': str(output)}
    if args.record:
        result.update(status='RECORDED_DIRECT_HUMAN_CONDITIONS', written=write_json(output, record))
    return result


def parser():
    result = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    commands = result.add_subparsers(dest='command', required=True)
    prep = commands.add_parser('prepare', allow_abbrev=False, help='File-only profile; PLAN unless --prepare')
    prep.add_argument('--mode', choices=MODES, required=True)
    prep.add_argument('--duration', type=int, choices=DURATIONS, required=True)
    prep.add_argument('--power-epoch', required=True)
    helps = {'topology': 'Lineage four-bus read-only capture.json (exact path the STOP-proxy report pinned)',
             'axis-geometry-profile': 'Pinned reviewed boxed LP V3 profile of the two-bus boxed 2s/10s runs, e.g. '
                 'the Jetson profile whose SHA256 is evidence current_boxed_native_output.two_second.profile_sha256',
             'current-topology': 'Fresh read-only capture taken after the predecessor terminal STOP'}
    for name in ('topology', 'events', 'calibration', 'model-profile', 'source-binding', 'mount', 'bias',
                 'axis-geometry-profile', 'current-topology', 'current-events'):
        prep.add_argument('--'+name, required=True, help=helps.get(name))
        prep.add_argument('--'+name+'-sha256', required=True)
    for name in ('accel-hypothesis', 'stop-proxy-report', 'predecessor-report'):
        prep.add_argument('--'+name)
        prep.add_argument('--'+name+'-sha256')
    prep.add_argument('--rebind-window-to-current-capture', action='store_true',
        help='Explicit: re-centre only the reviewed +-3deg window and +-0.5deg start on the current pose')
    # Opt-in timing options, recorded in the contract pacing only when given (default contract unchanged).
    prep.add_argument('--command-phase-offset-us', type=int,
        help='F1: command time release+K after the final gate (%d..%d; suggestion 11250; with '
             '--prearmed-hold-lead-us L at most %d - max(L, %d))' % (COMMAND_PHASE_MIN_US, COMMAND_PHASE_MAX_US,
             command_phase_max_with_lead_us(0)+PREARM_WORK_US, PREARM_WORK_US))
    prep.add_argument('--decode-once', action='store_true',
        help='F2b: owner-published read-only rows; takeouts and the final gate compare raw images')
    prep.add_argument('--prearmed-hold-lead-us', type=int,
        help='F3: wake at release-lead and pre-arm the hold natively at the release (%d..%d, e.g. 1000-1500; '
             'it must cover Boundary 1, the hold gates and all four owners\' preparation); '
             'needs a library whose receipt records exchange_at_abi 1' % PREARMED_LEAD_US)
    prep.add_argument('--gc-freeze', action='store_true',
        help='F4: gc.freeze() after warm-up before the first release, gc.unfreeze() at restoration')
    prep.add_argument('--output', required=True)
    prep.add_argument('--prepare', action='store_true')
    cond = commands.add_parser('conditions', allow_abbrev=False, help='Direct-human record; PLAN unless --record')
    cond.add_argument('--user-statement', required=True, help='Verbatim current user chat statement')
    cond.add_argument('--user-reply-id')
    cond.add_argument('--boot-id', required=True)
    cond.add_argument('--power-epoch', required=True)
    cond.add_argument('--contract-sha256', required=True)
    cond.add_argument('--authorize-mode', action='append', choices=MODES, required=True)
    cond.add_argument('--authorize-duration', action='append', type=int, choices=DURATIONS, required=True)
    for name in (*TRUE_CONDITIONS, *FALSE_CONDITIONS):
        cond.add_argument('--'+name.replace('_', '-'), dest=name, choices=('true', 'false'), required=True)
    cond.add_argument('--output', required=True)
    cond.add_argument('--record', action='store_true')
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == 'prepare':
            for name in ('accel_hypothesis', 'stop_proxy_report', 'predecessor_report'):
                need((getattr(args, name) is None) == (getattr(args, name+'_sha256') is None),
                     'Path and SHA256 must be given together: '+name)
            result = prepare(args)
        else:
            result = record_conditions(args)
    except (OSError, ValueError) as error:
        print(json.dumps({'status': 'REJECTED_FILE_ONLY', 'errors': [type(error).__name__+': '+str(error)],
                          'hardware_opened': False, **NO_GRANTS}, sort_keys=True))
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
