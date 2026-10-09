"""Default-PLAN four-bus boxed live Type1 runner; opens, loads and writes nothing itself.

Port-keyed equivalent of ``policy_output_runtime.run_supported_policy`` phases
A-J (OR:1296-2319) on four exact-three Type1 transports (DESIGN.md section 4).
Every device, model, operating scope and STOP capability is injected by the
foreground. A PLAN, test pass or COMPLETE status is never an output approval:
admission and the current direct-human condition record live elsewhere.
"""
from collections.abc import Mapping
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import asdict, replace
import copy
import gc
import math
import os
import struct
import threading
import time
from typing import Protocol

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw import rs05_trial_protocol as protocol
from singularitydog_hw.angle_calibration_audit import AngleEvidenceError, resolve_unique_numeric_branch
from singularitydog_hw.native_active_transport import encode_motion
from singularitydog_hw.native_diagnostic_transport import Record
from singularitydog_hw.policy_live_profile import _preauthorized_boxed_axis_caps
from singularitydog_hw.policy_motion_envelope import AxisLimits, PolicyMotionEnvelope
from singularitydog_hw.policy_observer import RAW_IMU_CORRECTION_FLAGS, _bias, _mount
from singularitydog_hw.policy_output_runtime import (
    IDS, MODEL_TARGET_LIMITS_BY_ID, PERIOD_NS, V3_VOLTAGE_MAX_AGE_NS, OutputWatchdog, _absolute_epoch_slot,
    _python_motion_wires, checked_voltage_cache, checked_voltage_rows, feedback_sample,
    validate_imu_metadata, validate_measured)
from singularitydog_hw.policy_post_reply_timing import PostReplyDeadlineBudget
from experiments.four_bus_diagnostic import model_bridge as bridge
from experiments.four_bus_diagnostic.pipeline import _real_operating_readback, _take, evidence_report
from experiments.four_bus_diagnostic.transport_adapter import GROUPS, PORTS, Group
from . import type1_profile as _profile

SCHEMA = 'singularitydog.four-bus-type1-boxed-run.v1'
MODES = ('zero_gain_timing', 'learned_boxed')
DURATIONS = (2, 10, 20)
PER_CYCLE_REQUESTS = 28
SETUP_EXCHANGE_NS = 100_000_000
VERSION_PROBE_NS = 250_000_000
ENABLE_REPLY_NS = 30_000_000
ENABLE_TOTAL_NS = 120_000_000
HOST_WATCHDOG_NS = 40_000_000  # OR:1456 PERIOD_NS + hard_cycle_ms
STOP_COLLECTION_S = 1.25       # OR:1073 shared collection deadline
SETUP_JOIN_GRACE_S = .05       # Host bookkeeping only; native deadlines are unchanged.
POSE_TOLERANCE_RAD = .02
BRANCH_MARGIN_RAD = bridge.MARGIN_RAD
POST_REPLY_V1 = copy.deepcopy(_profile.POST_REPLY_POLICY)  # bounded_post_reply_v1
# One pacing record shared with the admitted contract (type1_profile.PACING), so
# a run report's pacing equals the contract pacing its successor validates.
PACING = copy.deepcopy(_profile.PACING)
AXIS_KEYS = ('uid', 'sign', 'offset_rad', 'physical_lower_rad', 'physical_upper_rad',
             *AxisLimits.__dataclass_fields__)
REQUIRED_ADMITTED_KEYS = ('mode', 'duration_s', 'ids_by_port', 'boot_id', 'motor_power_epoch',
    'contract_sha256', 'profile', 'offsets_by_id', 'reference_turns_by_id', 'uids_by_id',
    'firmware_by_id', 'pacing', 'post_reply_policy', 'first_cycle_post_reply', 'model_plan')
TRANSPORT_METHODS = ('identify', 'stop', 'version_probe', 'read_params', 'write_watchdog',
                     'enable', 'zero_gain', 'hold_then_voltage', 'output', 'stop_repeated', 'close')
_FLAGS = {'learned_targets_attempted': False, 'motor_enable_sent': False,
          'type1_sent': False, 'positive_gain_sent': False}
_GRANTS = {'output_approval_granted_here': False, 'timing_qualification_granted_here': False,
           'live_type1_qualified': False, 'approved_for_runtime': False}
_PHYSICAL = {'motion_observed': None, 'box_support_observed': None,
             'clearance_observed': None, 'feet_contact_observed': None}
_VERSION_PREFIX = b'\x00\xc4\x56'
# LP:2083-2087 reviewed ranges for the OM.validate_inputs body limits.
IMU_LIMIT_RANGES = {'imu_tilt_limit_rad': (.01, .35), 'imu_gyro_limit_rad_s': (.01, 1.),
                    'imu_accel_norm_min_m_s2': (8.8, 11.2), 'imu_accel_norm_max_m_s2': (8.8, 11.2)}
IMU_CHECK_POINTS = ('pre_enable_after_metadata', 'every_cycle_after_metadata_including_gain_down')


class Type1TransportContract(Protocol):
    """Duck-typed DESIGN section 2 contract the runner relies on (no import).

    Every call runs on the port's single owner worker. Feedback maps are keyed
    ``mid`` or ``(mid, 'feedback')``; ``read_params`` maps ``(mid, name)``.
    Values are either ``decode_records`` rows ``(value, start_ns, received_ns)``
    or bare decoded values (Type2 Feedback / parameter value) whose timestamps
    come from the owner's ``last_batch`` (re-verified, equal decoded values).
    ``identify``/``version_probe`` map mid to bytes or lowercase hex (or a row
    whose value has ``mcu_uid_hex``/``version_bytes_hex``). ``enable`` and
    ``zero_gain`` return one Feedback (bare or as a row).
    ``hold_then_voltage``/``output`` return Batch objects with ``records``
    (Record*n), ``rows`` and ``verify()`` (four_bus_diagnostic Batch shape).
    ``stop_repeated`` returns {'complete', 'confirmed_ids', 'unconfirmed_ids',
    'ambiguous_ids', 'faults', 'rounds', 'physical_cutoff_required'}.
    Opt-in only: ``decode_once``/``prearmed_hold`` attributes must equal the
    contract selection; decode_once adds ``verify_batch(batch, label)`` and
    prearmed_hold adds ``hold_then_voltage(..., not_before_ns=release)``.
    """
    group: Group
    journal: list
    def identify(self, *, deadline_ns): ...
    def stop(self, *, deadline_ns): ...
    def version_probe(self, *, deadline_ns): ...
    def read_params(self, names, *, deadline_ns): ...
    def write_watchdog(self, ticks, *, deadline_ns): ...
    def enable(self, mid, *, deadline_ns): ...
    def zero_gain(self, mid, q_raw, *, deadline_ns): ...
    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check): ...
    def output(self, wires, *, deadline_ns, check): ...
    def stop_repeated(self, *, total_budget_ns=1_000_000_000, rounds=3): ...
    def close(self): ...


def need(condition, message):
    if not condition:
        raise ValueError(message)


def _plain(value):
    """Setup-only owned JSON-shaped copy of an immutable admitted object."""
    if isinstance(value, Mapping):
        need(all(type(key) is str for key in value), 'Admitted mappings need string keys')
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if value is None or type(value) in (bool, int, float, str):
        return value
    raise ValueError('Plain admitted data required: '+type(value).__name__)


def _finite(value, name):
    need(type(value) in (int, float) and math.isfinite(value), 'Finite '+name+' required')
    return float(value)


def _hex(value, length, name):
    need(type(value) is str and (length is None or len(value) == length) and value and
         all(c in '0123456789abcdef' for c in value), 'Lowercase hex '+name+' required')
    return value


def enable_order(groups):
    """OR:1725-1728 interleave, keyed by ID: 1,4,7,10,2,5,8,11,3,6,9,12."""
    ordered = sorted(groups, key=lambda group: group.ids[0])
    return tuple((group.port, group.ids[index]) for index in range(3) for group in ordered)


def validate_admitted(admitted):
    """Pure admitted-object recheck; returns an owned spec. Opens nothing."""
    need(isinstance(admitted, Mapping) and all(key in admitted for key in REQUIRED_ADMITTED_KEYS),
         'Admitted four-bus Type1 object with all required keys required')
    value = {key: _plain(admitted[key]) for key in REQUIRED_ADMITTED_KEYS}
    mode, duration = value['mode'], value['duration_s']
    need(mode in MODES, 'Explicit zero_gain_timing or learned_boxed mode required')
    need(type(duration) is int and duration in DURATIONS, 'Duration must be exactly 2, 10 or 20 s')
    mapping = value['ids_by_port']
    need(type(mapping) is dict and set(mapping) == set(PORTS), 'Exact four-port topology required')
    groups = tuple(Group(port, tuple(mapping[port])) for port in PORTS)
    need({group.ids for group in groups} == set(GROUPS), 'Four physical groups must cover twelve axes')
    for key in ('boot_id', 'motor_power_epoch'):
        need(type(value[key]) is str and value[key].strip() and
             value[key] not in ('UNKNOWN', 'NOT_INFERRED_FROM_JETSON_BOOT'), 'Explicit current '+key+' required')
    _hex(value['contract_sha256'], 64, 'contract sha256')
    options = _profile.pacing_options(value['pacing'], 'Exact R8-qualified four-bus pacing required')
    need(value['post_reply_policy'] == POST_REPLY_V1, 'Exact bounded_post_reply_v1 policy required')
    need(type(value['first_cycle_post_reply']) is bool, 'First-cycle post-reply selection must be a bool')
    profile = value['profile']
    need(type(profile) is dict and type(profile.get('axes')) is dict and
         set(profile['axes']) == {str(mid) for mid in IDS}, 'Twelve profile axes required')
    for key, expected in (('max_sample_age_ms', 20), ('max_sample_gap_ms', 21), ('hard_cycle_ms', 20),
                          ('period_ms', 20), ('max_consecutive_20ms_misses', 0), ('voltage_min_v', 35),
                          ('voltage_max_v', 42), ('startup_damping_duration_s', .08),
                          ('policy_weight', .005), ('duration_s', duration)):
        need(type(profile.get(key)) in (int, float) and profile[key] == expected,
             'Fixed boxed profile value differs: '+key)
    for key in ('startup_duration_s', 'stop_duration_s', 'policy_ramp_s'):
        need(_finite(profile.get(key), key) > 0, 'Positive '+key+' required')
    need(profile['startup_damping_duration_s'] <= profile['startup_duration_s'],
         'Damping ramp must not be slower than the position ramp')
    need(type(profile.get('start_pose_bounds')) is dict, 'Reviewed start pose bounds required')
    for key, (lower, upper) in IMU_LIMIT_RANGES.items():
        need(lower <= _finite(profile.get(key), key) <= upper, 'Reviewed IMU limit out of scope: '+key)
    need(profile['imu_accel_norm_min_m_s2'] < profile['imu_accel_norm_max_m_s2'] and
         profile['imu_accel_norm_max_m_s2']-profile['imu_accel_norm_min_m_s2'] <= 1.5,
         'Invalid reviewed IMU gravity-norm interval')
    _preauthorized_boxed_axis_caps(profile)
    for key in ('offsets_by_id', 'reference_turns_by_id', 'uids_by_id', 'firmware_by_id'):
        need(type(value[key]) is dict and set(value[key]) == {str(mid) for mid in IDS}, 'Twelve-axis '+key+' required')
    offsets, turns, uids, firmware = {}, {}, {}, {}
    for mid in IDS:
        key, axis = str(mid), profile['axes'][str(mid)]
        need(type(axis) is dict and all(name in axis for name in AXIS_KEYS), 'ID'+key+' axis keys missing')
        need(axis['sign'] in (-1, 1) and type(axis['sign']) in (int, float), 'ID'+key+' sign must be +/-1')
        for name in AXIS_KEYS[2:]:
            _finite(axis[name], 'ID'+key+' '+name)
        lo, hi = MODEL_TARGET_LIMITS_BY_ID[mid]
        need(lo <= axis['physical_lower_rad'] < axis['physical_upper_rad'] <= hi,
             'ID'+key+' physical envelope outside model range')
        # LP:2131-2132 local mode: effective bounds are the physical envelope shrunk by the margin.
        need(abs(axis['lower_rad']-(axis['physical_lower_rad']+BRANCH_MARGIN_RAD)) <= 1e-12 and
             abs(axis['upper_rad']-(axis['physical_upper_rad']-BRANCH_MARGIN_RAD)) <= 1e-12 and
             axis['lower_rad'] < axis['upper_rad'], 'ID'+key+' effective bounds must be physical -/+ local margin')
        bounds = profile['start_pose_bounds'].get(key)
        need(type(bounds) is list and len(bounds) == 2 and
             axis['lower_rad'] <= bounds[0] < bounds[1] <= axis['upper_rad'], 'ID'+key+' start pose bounds')
        offsets[mid] = _finite(value['offsets_by_id'].get(key), 'ID'+key+' fixed offset')
        turns[mid] = value['reference_turns_by_id'].get(key)
        need(type(turns[mid]) is int, 'ID'+key+' integer reference turns required')
        need(abs(axis['offset_rad']-axis['sign']*turns[mid]*2*math.pi-offsets[mid]) <= 1e-9,
             'ID'+key+' fixed offset differs from nominal offset and reference turns')
        uids[mid] = _hex(value['uids_by_id'].get(key), None, 'ID'+key+' UID')
        need(axis['uid'] == uids[mid], 'ID'+key+' profile UID differs from topology UID')
        firmware[mid] = _hex(value['firmware_by_id'].get(key), 8, 'ID'+key+' firmware fingerprint')
    limits = [AxisLimits(**{name: profile['axes'][str(mid)][name] for name in AxisLimits.__dataclass_fields__})
              for mid in IDS]
    max_stop_s = max(a.max_command_velocity_rad_s/a.max_command_acceleration_rad_s2 for a in limits)+profile['stop_duration_s']
    need(profile['startup_duration_s']+profile['policy_ramp_s']+max_stop_s+.04 < duration,
         'Duration must include startup, policy ramp, braking and gain ramp')  # OR:1699-1701
    plan_axes = bridge._plan_axes(value['model_plan'])
    for mid in IDS:
        port, sign, offset, _, _, _ = plan_axes[mid]
        need(mid in mapping[port] and sign == profile['axes'][str(mid)]['sign'] and
             abs(offset-offsets[mid]) <= 1e-9, 'ID%d model plan port/sign/branch differs' % mid)
    imu = imu_frame(value['model_plan'])
    imu['limits'] = {key: float(profile[key]) for key in IMU_LIMIT_RANGES}
    return {'mode': mode, 'duration_s': duration, 'groups': groups, 'profile': profile,
            'offsets': offsets, 'turns': turns, 'uids': uids, 'firmware': firmware,
            'max_stop_s': max_stop_s, 'stop_at_s': duration-max_stop_s-.04,
            'first_cycle_post_reply': value['first_cycle_post_reply'], 'model_plan': value['model_plan'],
            'boot_id': value['boot_id'], 'motor_power_epoch': value['motor_power_epoch'],
            'contract_sha256': value['contract_sha256'], 'imu': imu, 'options': options}


def imu_frame(model_plan):
    """The observer's own inline mount/bias candidates (MB:399-404); opens nothing."""
    kwargs = model_plan.get('observer_kwargs') if isinstance(model_plan, Mapping) else None
    need(isinstance(kwargs, Mapping) and kwargs.get('apply_reviewed_accel_calibration') is False,
         'Model plan observer IMU settings required')
    mount, bias = kwargs.get('imu_mount_candidate'), kwargs.get('gyro_bias_candidate')
    need(isinstance(mount, Mapping) and isinstance(bias, Mapping), 'Inline IMU mount and gyro bias candidates required')
    rotation = _mount(_plain(mount))['R_body_from_sensor']
    offset = _bias(_plain(bias))['bias_sensor_rad_s']
    rotation = [[_finite(x, 'IMU rotation') for x in row] for row in rotation]
    offset = [_finite(x, 'gyro bias') for x in offset]
    need(len(rotation) == len(offset) == 3, 'Three-axis IMU mount and bias required')
    return {'rotation': rotation, 'gyro_bias': offset,
            'accel_input_hypothesis': kwargs.get('accel_input_hypothesis') is not None}


def check_imu_limits(imu, limits, rotation, gyro_bias, correction=None):
    """IMU part of OM.validate_inputs (policy_output_model.py:243-271); rejects, never clips.

    Raw norm and raw tilt always; with the selected acceleration hypothesis its
    own norm bounds and the corrected tilt as well. Returns (tilt, raw_tilt,
    body_gyro_norm, raw_norm).
    """
    if (imu.get('frame') != 'sensor' or
            any(imu.get(key, False) is not False for key in RAW_IMU_CORRECTION_FLAGS)):
        raise ValueError('Uncorrected sensor-frame IMU required')
    accel, gyro = imu.get('accel_m_s2'), imu.get('gyro_rad_s')
    for vector in (accel, gyro):
        if (not isinstance(vector, (list, tuple)) or len(vector) != 3 or
                not all(type(v) in (int, float) and math.isfinite(v) for v in vector)):
            raise ValueError('Invalid IMU vector')
    norm = math.hypot(*accel)
    if not limits['imu_accel_norm_min_m_s2'] <= norm <= limits['imu_accel_norm_max_m_s2']:
        raise ValueError('IMU gravity-proxy norm outside reviewed range')
    up = rotation[2]
    raw_tilt = math.acos(max(-1., min(1., (up[0]*accel[0]+up[1]*accel[1]+up[2]*accel[2])/norm)))
    tilt = raw_tilt
    if correction is not None:
        corrected, corrected_norm = correction.correct(accel)
        tilt = math.acos(max(-1., min(1., (up[0]*corrected[0]+up[1]*corrected[1]+up[2]*corrected[2])/corrected_norm)))
        if not raw_tilt <= limits['imu_tilt_limit_rad']:
            raise ValueError('Raw body tilt exceeded with acceleration hypothesis')
    g0, g1, g2 = gyro[0]-gyro_bias[0], gyro[1]-gyro_bias[1], gyro[2]-gyro_bias[2]
    rate = math.hypot(*(row[0]*g0+row[1]*g1+row[2]*g2 for row in rotation))
    if not (tilt <= limits['imu_tilt_limit_rad'] and rate <= limits['imu_gyro_limit_rad_s']):
        raise ValueError('Body tilt/angular velocity exceeded')
    return tilt, raw_tilt, rate, norm


def plan(admitted):
    spec = validate_admitted(admitted)
    order = enable_order(spec['groups'])
    options = spec['options']
    result = {'schema': SCHEMA, 'status': 'PLAN', 'opens_devices': False, **_FLAGS, **_GRANTS,
            'physical_observations': dict(_PHYSICAL),
            'flag_semantics': 'set_before_first_corresponding_write_attempt_never_reset',
            'mode': spec['mode'], 'duration_s': spec['duration_s'],
            'boot_id': spec['boot_id'], 'motor_power_epoch': spec['motor_power_epoch'],
            'contract_sha256': spec['contract_sha256'],
            'ids_by_port': {group.port: list(group.ids) for group in spec['groups']},
            'enable_order': [{'port': port, 'motor_id': mid} for port, mid in order],
            'pacing': {**copy.deepcopy(PACING), **options}, 'request_gap_ns': 900_000, 'request_window': 3,
            'absolute_deadline_ns': PERIOD_NS, 'per_cycle_requests': PER_CYCLE_REQUESTS,
            'request_schedule': 'each_physical_3Type1hold_then_1voltage_then_3Type1_output',
            'post_reply_policy': dict(POST_REPLY_V1),
            'first_cycle_post_reply': spec['first_cycle_post_reply'],
            'stop_at_s': spec['stop_at_s'], 'max_stop_s': spec['max_stop_s'],
            'host_watchdog_ns': HOST_WATCHDOG_NS, 'motor_watchdog_ticks': protocol.WATCHDOG_TICKS,
            'host_watchdog_placement': 'created_on_imu_owner_inherits_cpu0_3_not_main_cpu4',
            'full_current_check_points': ['cycle_start_before_submit',
                'after_feedback_joins_before_inference', 'after_final_gate_before_output'],
            'final_gate': 'raw_bytes_redecoded_and_imu_equal_pre_inference_snapshot_not_retained',
            'imu_limits': dict(spec['imu']['limits']), 'imu_limit_check_points': list(IMU_CHECK_POINTS),
            'imu_accel_input_hypothesis_selected': spec['imu']['accel_input_hypothesis'],
            'stop_policy': 'repeated_subset_stop_each_own_owner_concurrent_shared_1250ms',
            'zero_gain_timing_outputs': 'kp_kd_zero_at_q0_model_targets_recorded_only'}
    # Opt-in options only add their own truthful fields; the default PLAN is unchanged.
    if 'command_phase_offset_us' in options:
        result['command_phase'] = {'offset_us': options['command_phase_offset_us'],
            'command_time': 'max(release_plus_offset, natural_final_gate_end)',
            'wait': 'after_boundary3_native_wait_until_cancel_fd_then_hot_check',
            'natural_gate_recorded_as': 'final_gate_ns', 'static_budget_us': dict(_profile.COMMAND_PHASE_BUDGET_US)}
        if 'prearmed_hold_lead_us' in options:
            result['command_phase']['prearmed_tail_budget_us'] = {**_profile.PREARMED_TAIL_BUDGET_US,
                'next_prearm_window': max(options['prearmed_hold_lead_us'], _profile.PREARM_WORK_US)}
    if options.get('decode_once') is True:
        result.update(decode_once_selected=True, takeout_check='raw_byte_images_compared_publication_rows_reused',
            decode_once_scope='frozen_type2_hold_rows_only_voltage_rows_redecoded',
            final_gate='hold_raw_byte_images_compared_publication_rows_reused_voltage_redecoded_'
                       'and_imu_equal_pre_inference_snapshot_not_retained')
    if 'prearmed_hold_lead_us' in options:
        result['prearmed_hold'] = {'lead_us': options['prearmed_hold_lead_us'],
            'main_wake': 'release_minus_lead', 'hold_write': 'native_not_before_release_cancel_fd_watched',
            'gates_before_release': ['boundary1_full_current_check', 'command_sample_gap', 'stale_feedback',
                                     'voltage_cache_at_release'],
            'hard_end': 'release_plus_20ms', 'imu_submit': 'after_actual_release_wait',
            'first_input_stamp': 'earliest_actual_request_start'}
        result['full_current_check_points'][0] = 'before_release_before_prearmed_submit'
    if options.get('gc_freeze') is True:
        result['gc_freeze_selected'] = True
    return result


def split_wires_by_port(encoded, groups, *, zero_gain):
    """Map the pinned {'front','rear'} encoder output to ports by destination ID.

    _python_motion_wires (OR:257-284) and VerifiedBatchEncoder have already
    encoded all twelve and run the quantized checks; nothing is clipped here.
    """
    if type(encoded) is not dict or encoded.keys() != {'front', 'rear'}:
        raise RuntimeError('Pinned twelve-axis front/rear encoder output required')
    by_id = {}
    for bus, base in (('front', 1), ('rear', 7)):
        wires = encoded[bus]
        if type(wires) not in (list, tuple) or len(wires) != 6:
            raise RuntimeError('Six encoder wires per half required')
        for index, wire in enumerate(wires):
            if (type(wire) is not bytes or len(wire) != 17 or wire[:2] != b'AT' or wire[6] != 8 or
                    wire[-2:] != b'\r\n' or wire[5] & 7 != 4 or
                    int.from_bytes(wire[2:6], 'big') >> 3 != (1 << 24) | (32767 << 8) | (base+index) or
                    wire[9:11] != b'\x7f\xff'):
                raise RuntimeError('Encoder wire is not the canonical Type1 for ID%d' % (base+index))
            if zero_gain and wire[11:15] != b'\x00\x00\x00\x00':
                raise RuntimeError('Zero-gain timing output carries a gain')
            by_id[base+index] = wire
    return {group.port: tuple(by_id[mid] for mid in group.ids) for group in groups}


def type1_hold_snapshot_builder(plan_document):
    """Setup-only plan copy; Type1-hold variant of model_bridge._snapshot (MB:473-535).

    Each record's tx must equal the exact previous validated Type1 wire and its
    reply must be a healthy mode-2 fault-0 Type2. Positions keep the frozen
    branch check (``_check_positions``) and the ordinary observer contract.
    """
    owned = _plain(plan_document)
    axes = bridge._plan_axes(owned)
    order = {port: tuple(owned['topology_by_port'][port]) for port in PORTS}
    replies = {mid: (((2 << 24) | (2 << 22) | (mid << 8) | codec.HOST_ID) << 3) | 4 for mid in IDS}
    record_type = Record*3

    def build(hold_by_port, wires_by_port, sample, tick_ns):
        tick_ns = bridge._stamp(tick_ns, 'observer tick')
        motors = []
        oldest = latest = earliest = None
        for port in PORTS:
            records, wires = hold_by_port[port].records, wires_by_port[port]
            if type(records) is not record_type or len(wires) != 3:
                raise ValueError('Original owned Record*3 Type1 hold required per physical port')
            for mid, record, wire in zip(order[port], records, wires):
                if not (record.written == record.received == 17 and
                        0 < record.start_ns <= record.finish_ns <= record.read_start_ns <=
                        record.received_ns < record.deadline_ns and record.received_ns <= tick_ns):
                    raise ValueError('Incomplete/noncausal Type1 hold record')
                if bytes(record.tx) != wire:
                    raise ValueError('Exact previous validated Type1 hold wire required')
                rx = bytes(record.rx)
                if (rx[:2] != b'AT' or rx[6] != 8 or rx[-2:] != b'\r\n' or
                        int.from_bytes(rx[2:6], 'big') != replies[mid] or rx[7:10] == _VERSION_PREFIX):
                    raise ValueError('Healthy mode-2 fault-0 Type1 hold reply required')
                p, v = struct.unpack_from('>2H', rx, 7)
                start, end = record.start_ns, record.received_ns
                motors.append({'motor_id': mid, 'parameter': 'position', 'value': p*(2.*12.57)/65535.-12.57,
                               'unit': 'rad', 'request_ns': start, 'received_ns': end,
                               'age_upper_bound_ns': tick_ns-start})
                motors.append({'motor_id': mid, 'parameter': 'velocity', 'value': v*100./65535.-50.,
                               'unit': 'rad_s', 'request_ns': start, 'received_ns': end,
                               'age_upper_bound_ns': tick_ns-start})
                oldest = start if oldest is None or start < oldest else oldest
                latest = end if latest is None or end > latest else latest
                earliest = end if earliest is None or end < earliest else earliest
        if type(sample) is not dict:
            raise ValueError('Original IMU sample required')
        a = bridge._stamp(sample.get('read_started_monotonic_ns'), 'IMU read start')
        b = bridge._stamp(sample.get('read_finished_monotonic_ns'), 'IMU read finish')
        if not a <= b <= tick_ns:
            raise ValueError('Noncausal original IMU interval')
        vectors = {}
        for name in ('accel_m_s2', 'gyro_rad_s'):
            vector = sample.get(name)
            if type(vector) is not list or len(vector) != 3:
                raise ValueError('Three original IMU components required')
            vectors[name] = [bridge._finite(item, 'IMU component') for item in vector]
        oldest, latest, earliest = min(oldest, a), max(latest, b), min(earliest, b)
        if tick_ns-oldest > PERIOD_NS or latest-oldest > PERIOD_NS:
            raise ValueError('Four-bus Type1 hold input age/spread exceeds 20ms')
        snapshot = {'status': 'DIAGNOSTIC_READY', 'output_allowed': False, 'blocked_reasons': [],
            'tick_ns': tick_ns, 'max_age_ns': bridge.OBSERVER_LIMIT_NS, 'max_spread_ns': bridge.OBSERVER_LIMIT_NS,
            'motors': motors, 'imu': {'frame': 'raw_sensor', **vectors,
                'read_started_ns': a, 'read_finished_ns': b, 'age_upper_bound_ns': tick_ns-a},
            'oldest_observation_age_ns': tick_ns-oldest, 'acquisition_spread_ns': latest-oldest,
            'receive_spread_ns': latest-earliest, 'voltage_by_bus': {},
            'source_flags': {'native_diagnostic_transport': True, 'sensor_type2_candidate': True,
                'type1_hold_feedback_mode2': True, 'stop_feedback_state_changing': False,
                'v3_voltage_overlap_pending_at_inference': True, 'velocity_scale_verified': False,
                'sensor_internal_sample_time_verified': False, 'fresh_identity_match_verified': True,
                'four_physical_buses_projected_to_original_full12_model': True,
                'physical_buses': list(PORTS), 'physical_record_count_by_port': {p: 3 for p in PORTS},
                'synthetic_six_axis_stats_created': False, 'approved_for_runtime': False,
                'output_allowed': False}}
        bridge._check_positions(axes, snapshot)
        return snapshot
    return build


def _timed_rows(transport, value, keys):
    """Exact decode rows ``(value, start_ns, received_ns)`` for one just-returned call.

    Call on the owner thread directly after the transport method. Row-shaped
    replies are used as given; a bare-value reply is bound to the owner's last
    Batch, which must cover exactly these keys with equal re-verified values.
    """
    need(isinstance(value, Mapping), 'Reply map required')
    given = {}
    for key, item in value.items():
        given[(key, 'feedback') if type(key) is int else key] = item
    need(set(given) == set(keys), 'Exact same-group reply keys required')
    if all(type(item) is tuple and len(item) == 3 and type(item[1]) is int and type(item[2]) is int
           for item in given.values()):
        return given
    batch = getattr(transport, 'last_batch', None)
    rows = batch.verify() if callable(getattr(batch, 'verify', None)) else None
    need(isinstance(rows, Mapping) and set(rows) == set(keys), 'Timestamped owner batch for this reply required')
    for key, item in given.items():
        decoded = rows[key][0]
        need(item == (decoded if key[1] == 'feedback' else decoded.get('value')),
             'ID%d returned value differs from its owner batch' % key[0])
    return dict(rows)


def _feedback_rows(transport, value, ids):
    return _timed_rows(transport, value, {(mid, 'feedback') for mid in ids})


def _param_rows(transport, value, ids, names):
    rows = _timed_rows(transport, value, {(mid, name) for mid in ids for name in names})
    for row in rows.values():
        need(isinstance(row[0], Mapping), 'Parameter rows need decoded values')
    return rows


def _single_feedback(value, mid):
    if isinstance(value, Mapping):
        need(set(value) in ({mid}, {(mid, 'feedback')}), 'Single-axis same-ID feedback required')
        value = next(iter(value.values()))
    if type(value) is tuple and len(value) == 3:
        value = value[0]
    need(type(getattr(value, 'mode_state', None)) is int and type(getattr(value, 'fault_bits', None)) is int,
         'Single-axis Type2 feedback required')
    return value


def _reply_hex(value, field):
    if type(value) is tuple and len(value) == 3:
        value = value[0]
    if isinstance(value, Mapping):
        value = value.get(field)
    if type(value) in (bytes, bytearray):
        value = bytes(value).hex()
    return value


def _stop_complete(result, ids):
    return (isinstance(result, Mapping) and result.get('complete') is True and
            sorted(result.get('confirmed_ids') or ()) == list(ids) and
            not result.get('unconfirmed_ids') and not result.get('ambiguous_ids') and
            result.get('physical_cutoff_required') is False and
            isinstance(result.get('faults'), Mapping) and not any(result['faults'].values()))


class _Supervisor:
    """Latched emergency: cancel once, then repeated subset STOP on each own owner.

    Mirrors OR:1027-1078. A single-worker FIFO pool means each queued STOP
    starts only after that port's in-flight exchange has returned.
    """
    def __init__(self, pools, adapters, cancel_io, clock, ids_by_port):
        self.pools, self.adapters, self.cancel_io, self.clock = pools, adapters, cancel_io, clock
        self.ids_by_port = {port: list(ids) for port, ids in ids_by_port.items()}
        self.lock = threading.Lock()
        self.aborted = threading.Event()
        self.reason = self.latched_ns = self.normal_completion = None
        self.stop_futures = None
        self.errors = []

    def emergency(self, reason, *, normal_completion=False):
        with self.lock:
            if self.stop_futures is not None:
                return
            self.reason, self.normal_completion = str(reason), normal_completion
            self.latched_ns = self.clock()
            self.aborted.set()
            self.stop_futures = {}
            try:
                self.cancel_io()
            except BaseException as error:
                self.errors.append({'stage': 'cancel_io', 'port': None,
                                    'error': type(error).__name__+': '+str(error)})
            for port in PORTS:
                adapter, pool = self.adapters.get(port), self.pools.get(port)
                if adapter is None or pool is None:
                    continue
                try:
                    self.stop_futures[port] = pool.submit(adapter.stop_repeated)
                except BaseException as error:
                    self.errors.append({'stage': 'stop_submission', 'port': port,
                                        'error': type(error).__name__+': '+str(error)})
                    failure = Future()
                    failure.set_exception(error)
                    self.stop_futures[port] = failure

    def finish(self):
        """Total: never raises; a failed or missing port is reported unconfirmed."""
        try:
            self.emergency('normal completion', normal_completion=True)
        except BaseException as error:
            self.errors.append({'stage': 'finish_emergency', 'port': None,
                                'error': type(error).__name__+': '+str(error)})
        result = {}
        deadline = time.monotonic()+STOP_COLLECTION_S
        for port, future in dict(self.stop_futures or {}).items():
            try:
                result[port] = future.result(timeout=max(0., deadline-time.monotonic()))
            except BaseException as error:
                result[port] = {'complete': False, 'confirmed_ids': [], 'error': repr(error),
                                'unconfirmed_ids': list(self.ids_by_port.get(port, ())),
                                'physical_cutoff_required': True}
        return result


def _create_watchdog(supervisor, clock):
    """Runs on the IMU owner: the 2 ms poller inherits its CPU0..3 mask, never main CPU4."""
    return OutputWatchdog(supervisor, HOST_WATCHDOG_NS, clock), threading.get_native_id()


def _watchdog_placement(watcher, creator, imu_tid, genuine):
    """Actual poller thread mask readback; OR creates it before pinning main (OR:1456/1633)."""
    tid = getattr(getattr(watcher, 'thread', None), 'native_id', None)
    need(type(tid) is int and type(creator) is int and creator == imu_tid and
         tid not in (creator, threading.get_native_id()), 'Host watchdog must be created on the IMU owner')
    mask = sorted(os.sched_getaffinity(tid)) if hasattr(os, 'sched_getaffinity') else None
    if genuine:
        need(mask == list(PACING['imu_mask']) and 4 not in mask, 'Host watchdog CPU0..3 placement readback required')
    return {'creator': 'imu_owner', 'creator_native_tid': creator, 'native_tid': tid, 'cpu_mask': mask,
            'timer_slack': 'inherited_from_imu_owner', 'poll_s': .002, 'timeout_ns': HOST_WATCHDOG_NS,
            'file_only_mock_readback': not genuine}


def _owned(supervisor, port, function, args, kwargs):
    """Owner-side wrapper: no new work after abort; a failure stops siblings now."""
    if supervisor.aborted.is_set():
        raise RuntimeError('Output cancelled before owner work: '+str(supervisor.reason))
    try:
        return function(*args, **kwargs)
    except BaseException as error:
        supervisor.emergency(port+' owner: '+type(error).__name__+': '+str(error))
        raise


def _owned_stamped(stamps, clock, supervisor, port, function, args, kwargs):
    """Timing evidence only: the owner entry time, then exactly _owned."""
    stamps[port] = clock()
    return _owned(supervisor, port, function, args, kwargs)


class _Cycle:
    """Raw per-cycle references and clocks; dictionaries are built after STOP."""
    __slots__ = ('index', 'slot', 'release_ns', 'begin_ns', 'hard_end_ns', 'first_ns', 'acquired_ns',
                 'gather_ns', 'infer_end_ns', 'voltage_join_ns', 'final_gate_ns', 'encode_end_ns',
                 'output_submit_ns', 'reply_return_ns', 'cycle_end_ns', 'join_deadline_ns', 'hold',
                 'voltage', 'output', 'imu', 'observed', 'model_target', 'blended_target', 'weight',
                 'label', 'command', 'sample', 'checked', 'decision', 'request_count', 'completed', 'imu_check',
                 # Opt-in only; materialized only when their option is selected.
                 'natural_gate_ns', 'command_ns', 'prearm_wake_ns', 'submit_done_ns', 'owner_entry_ns',
                 'thread_counters', 'evidence_cost_ns')

    def __init__(self, index, slot, release, begun):
        for name in self.__slots__:
            setattr(self, name, None)
        self.index, self.slot, self.release_ns, self.begin_ns = index, slot, release, begun
        self.completed = False

    def materialize(self, extra=()):
        def edge(batches, field, pick):
            if not batches:
                return None
            values = [getattr(record, field) for batch in batches.values() for record in batch.records]
            return pick(values) if values else None
        def ms(new, old):
            return None if new is None or old is None else (new-old)/1e6
        row = {name: getattr(self, name) for name in (
            'index', 'slot', 'release_ns', 'begin_ns', 'hard_end_ns', 'first_ns', 'acquired_ns',
            'gather_ns', 'infer_end_ns', 'voltage_join_ns', 'final_gate_ns', 'encode_end_ns',
            'output_submit_ns', 'reply_return_ns', 'cycle_end_ns', 'join_deadline_ns', 'weight', 'label',
            'request_count', 'completed', 'decision', 'observed', 'hold', 'voltage', 'output', 'imu')}
        row.update(model_target_by_id=None if self.model_target is None else list(self.model_target),
                   blended_target_by_id=None if self.blended_target is None else list(self.blended_target),
                   command=None if self.command is None else asdict(self.command),
                   sample=None if self.sample is None else asdict(self.sample),
                   output_feedback=None if self.checked is None else asdict(self.checked),
                   hold_first_write_ns=edge(self.hold, 'start_ns', min),
                   hold_last_reply_ns=edge(self.hold, 'received_ns', max),
                   voltage_last_reply_ns=edge(self.voltage, 'received_ns', max),
                   output_first_write_ns=edge(self.output, 'start_ns', min),
                   final_write_ns=edge(self.output, 'finish_ns', max),
                   output_last_reply_ns=edge(self.output, 'received_ns', max),
                   imu_limit_check=None if self.imu_check is None else dict(zip(
                       ('tilt_rad', 'raw_tilt_rad', 'body_gyro_norm_rad_s', 'raw_accel_norm_m_s2'), self.imu_check)))
        row.update(release_lateness_ms=ms(self.begin_ns, self.release_ns),
                   acquisition_ms=ms(self.acquired_ns, self.first_ns),
                   inference_ms=ms(self.infer_end_ns, self.gather_ns),
                   voltage_join_ms=ms(self.voltage_join_ns, self.infer_end_ns),
                   envelope_and_encode_ms=ms(self.encode_end_ns, self.final_gate_ns),
                   output_exchange_ms=ms(self.reply_return_ns, self.output_submit_ns),
                   iteration_ms=ms(self.cycle_end_ns, self.begin_ns),
                   input_to_last_output_reply_ms=ms(row['output_last_reply_ns'], self.first_ns))
        for name in extra:
            row[name] = getattr(self, name)
        if 'command_ns' in extra:
            row.update(command_wait_ms=ms(self.command_ns, self.natural_gate_ns),
                       envelope_and_encode_ms=ms(self.encode_end_ns, self.command_ns))
        if 'prearm_wake_ns' in extra:
            row['prearm_lead_actual_ms'] = ms(self.release_ns, self.prearm_wake_ns)
        return row


def run(admitted, *, factory=None, imu_read=None, observer=None, check_current=None,
        check_cancelled=None, cancel_io=None, model_setup=None, worker_scope=None,
        main_scope=None, release_wait=None, announce=None, encoder=None, stop_requested=None,
        backend_usage=None, accel_correction=None, clock=time.monotonic_ns, execute=False,
        command_wait=None, timing_evidence=None):
    """PLAN unless ``execute is True``; PLAN calls none of the capabilities.

    ``factory(group)`` returns a Type1TransportContract owner for that port.
    ``encoder`` is None (pinned Python ``_python_motion_wires``) or a verified
    native batch module with ``bind(axis_specs)`` (NBE VerifiedBatchModule).
    ``stop_requested`` (SIGUSR1) requests the graceful envelope ramp.
    ``accel_correction`` is the loaded acceleration input hypothesis (``correct``)
    exactly when the model plan selects one, else None.
    ``command_wait(target_ns)`` (GIL-released, cancel-watched; returns the actual
    wake time) is injected exactly when the contract selects
    command_phase_offset_us. ``timing_evidence`` (diagnostic, not contract) is
    None or a timing_evidence.TimingEvidence-shaped reader.
    """
    planned = plan(admitted)
    if execute is not True:
        return planned
    needed = (factory, imu_read, check_current, check_cancelled, cancel_io, model_setup,
              worker_scope, main_scope, release_wait, announce)
    if any(not callable(item) for item in needed) or not callable(getattr(observer, 'consume', None)):
        raise ValueError('Explicit source/current/owner/model/restore/announce capabilities required')
    if stop_requested is None:
        stop_requested = threading.Event()
    need(callable(getattr(stop_requested, 'is_set', None)), 'Graceful stop request must be an Event')
    need(encoder is None or callable(getattr(encoder, 'bind', None)), 'Pinned encoder module with bind required')
    need(isinstance(backend_usage, Mapping) and backend_usage.get('kind') in (
        'genuine_subset_active_cpp', 'injected_file_only_mock'), 'Truthful explicit backend usage required')
    genuine = backend_usage['kind'] == 'genuine_subset_active_cpp'
    spec = validate_admitted(admitted)
    need((accel_correction is not None) == spec['imu']['accel_input_hypothesis'] and
         (accel_correction is None or callable(getattr(accel_correction, 'correct', None))),
         'Acceleration correction must be injected exactly when the model plan selects it')
    options = spec['options']
    command_offset_ns = options['command_phase_offset_us']*1000 if 'command_phase_offset_us' in options else None
    prearm_ns = options['prearmed_hold_lead_us']*1000 if 'prearmed_hold_lead_us' in options else None
    decode_once, gc_freeze = options.get('decode_once') is True, options.get('gc_freeze') is True
    need((command_wait is None) == (command_offset_ns is None) and (command_wait is None or callable(command_wait)),
         'Command phase waiter must be injected exactly when the contract selects command_phase_offset_us')
    need(timing_evidence is None or all(callable(getattr(timing_evidence, name, None)) for name in
         ('bind', 'sample', 'run_counters', 'delta', 'run_delta', 'close')), 'Timing evidence reader incomplete')
    imu_limits, imu_rotation, imu_bias = spec['imu']['limits'], spec['imu']['rotation'], spec['imu']['gyro_bias']
    snapshot_builder = type1_hold_snapshot_builder(spec['model_plan'])
    groups, profile, offsets = spec['groups'], spec['profile'], spec['offsets']
    axes = profile['axes']
    axis_rows = tuple(axes[str(mid)] for mid in IDS)
    group_of = {group.port: group for group in groups}
    learned = spec['mode'] == 'learned_boxed'
    age_ns = int(profile['max_sample_age_ms']*1e6)
    gap_ns = int(profile['max_sample_gap_ms']*1e6)
    report = {**planned, 'status': 'STARTING', 'opens_devices': True,
              'backend_usage': _plain(backend_usage), 'devices_opened': False,
              'encoder': {'kind': 'python_motion_wires' if encoder is None else 'verified_native_batch',
                          'binary_sha256': getattr(encoder, 'binary_sha256', None)},
              'primary_error': None, 'errors': [], 'worker_settings': {}, 'restoration': {},
              'preflight': {}, 'startup_displacement_checks': [], 'owner_settlement': [],
              'voltage_guard': {'maximum_age_ns': V3_VOLTAGE_MAX_AGE_NS, 'checks_before_type1': 0,
                                'maximum_checked_age_ns': 0, 'minimum_checked_voltage_v': None},
              'cycles': [], 'completed_cycles': 0, 'normal_ramp_completed': False}
    pools, adapters, owner_scopes, in_flight = {}, {}, {}, []
    supervisor = _Supervisor(pools, adapters, cancel_io, clock, {group.port: group.ids for group in groups})
    voltage_cache = {}
    cycles = []
    guard = report['voltage_guard']
    watcher = main_context = created = None
    main_entered = False
    primary = None
    budget = PostReplyDeadlineBudget(POST_REPLY_V1)
    frozen = False
    evidence_before = evidence_baseline = None

    def owners():
        for owner in in_flight:
            if owner.done():
                if owner.cancelled():
                    raise RuntimeError('Owner Future was cancelled')
                error = owner.exception()
                if error is not None:
                    raise error

    def aborted():
        if supervisor.aborted.is_set():
            raise RuntimeError(supervisor.reason or 'Output aborted')

    def check():
        check_current()
        aborted()
        owners()

    def hot():
        check_cancelled()
        aborted()
        owners()

    def owner_check():
        check_cancelled()
        aborted()

    def submit(port, function, *args, **kwargs):
        return pools[port].submit(_owned, supervisor, port, function, args, kwargs)

    def setup_join(futures, timeout_s):
        in_flight[:] = list(futures.values())
        end = time.monotonic()+timeout_s
        try:
            result = {port: future.result(timeout=max(0., end-time.monotonic())) for port, future in futures.items()}
        except BaseException:
            aborted()  # The first latched owner failure wins over sibling cancellation.
            raise
        in_flight.clear()
        check()
        return result

    def all_ports(function, timeout_s=.5):
        return setup_join({port: submit(port, function, port) for port in PORTS}, timeout_s)

    def single(port, function, *args, deadline_ns):
        future = submit(port, function, *args, deadline_ns=deadline_ns)
        return setup_join({port: future}, max(0., (deadline_ns-clock())/1e9)+SETUP_JOIN_GRACE_S)[port]

    def require_voltage_before_type1(at_ns=None):
        # at_ns: the pre-armed release, when the hold is written (stricter than now).
        maximum_age, minimum = checked_voltage_cache(voltage_cache, profile, clock() if at_ns is None else at_ns)
        guard['checks_before_type1'] += 1
        if maximum_age > guard['maximum_checked_age_ns']:
            guard['maximum_checked_age_ns'] = maximum_age
        if guard['minimum_checked_voltage_v'] is None or minimum < guard['minimum_checked_voltage_v']:
            guard['minimum_checked_voltage_v'] = minimum

    def refresh_voltage(label):
        def read(port):
            transport = adapters[port]
            return _param_rows(transport, transport.read_params(('voltage',), deadline_ns=clock()+SETUP_EXCHANGE_NS),
                               group_of[port].ids, ('voltage',))
        rows = {}
        for value in all_ports(read).values():
            rows.update(value)
        voltage_cache.update(checked_voltage_rows(rows, IDS, profile, clock()))
        report[label] = {str(mid): {'value_v': voltage_cache[mid][0], 'received_ns': voltage_cache[mid][1]}
                         for mid in IDS}

    def initialize(port, mask):
        scope = worker_scope(port, mask)
        value = scope.__enter__()
        owner_scopes[port] = scope
        report['worker_settings'][port] = copy.deepcopy(value)
        return threading.get_native_id()

    def restore(port):
        scope = owner_scopes.pop(port, None)
        if scope is None:
            report['restoration'][port] = {'scope_not_entered': True}
            return
        scope.__exit__(None, None, None)
        report['restoration'][port] = True

    def cleanup_error(error):
        nonlocal primary
        if primary is None:
            primary = error
            report['primary_error'] = {'type': type(error).__name__, 'message': str(error)}
        else:
            primary.add_note('Cleanup failure: '+str(error))
        report['errors'].append('cleanup: '+type(error).__name__+': '+str(error))

    try:
        # A. Gates, owners, placement (PL:359-398; OR:1313-1460).
        main_context = main_scope()
        operating = main_context.__enter__()
        main_entered = True
        report['operating_settings'] = copy.deepcopy(operating)
        if genuine:
            _real_operating_readback(operating)
        check()
        if genuine:
            from .type1_transport import Type1Transport
        for group in groups:
            adapter = factory(group)
            adapters[group.port] = adapter  # Retain the returned resource even if rejected.
            report['devices_opened'] = True
            pools[group.port] = ThreadPoolExecutor(max_workers=1, thread_name_prefix='type1-can-'+group.port)
            if adapter.group != group or any(not callable(getattr(adapter, name, None)) for name in TRANSPORT_METHODS):
                raise ValueError('Exact-group Type1 transport with the full DESIGN section 2 contract required')
            if (getattr(adapter, 'decode_once', False) is not decode_once or
                    getattr(adapter, 'prearmed_hold', False) is not (prearm_ns is not None) or
                    (decode_once and not callable(getattr(adapter, 'verify_batch', None)))):
                raise ValueError('Transport decode-once/pre-armed selections must equal the admitted contract')
            if genuine:
                if type(adapter) is not Type1Transport:
                    raise ValueError('Real backend requires the genuine Type1Transport')
                verify = getattr(adapter, '_verify', None)
                if callable(verify):
                    verify()
        pools['imu'] = ThreadPoolExecutor(max_workers=1, thread_name_prefix='type1-imu')
        readbacks = [pools[port].submit(initialize, port, (index,)) for index, port in enumerate(PORTS)]
        readbacks.append(pools['imu'].submit(initialize, 'imu', (0, 1, 2, 3)))
        in_flight[:] = readbacks
        worker_tids = [future.result(timeout=1) for future in readbacks]
        in_flight.clear()
        if len({report['worker_settings'][port]['native_tid'] for port in (*PORTS, 'imu')}) != 5:
            raise ValueError('Five distinct persistent CAN/IMU workers required')
        if genuine:
            for index, port in enumerate((*PORTS, 'imu')):
                value = report['worker_settings'][port]
                if (value.get('cpu_mask') != ([index] if index < 4 else [0, 1, 2, 3]) or
                        value.get('timer_slack_ns') != 1000 or value.get('file_only_mock_readback') is not False):
                    raise ValueError('Actual five-worker placement/slack readback required')
        # Not from the CPU4-pinned main thread: a new thread inherits its creator's mask and slack.
        created = pools['imu'].submit(_create_watchdog, supervisor, clock)
        watcher, creator = created.result(timeout=1)
        report['watchdog_settings'] = _watchdog_placement(watcher, creator,
                                                          report['worker_settings']['imu']['native_tid'], genuine)

        # B. Preflight, ports concurrent and each port serial (OR:1170-1240).
        def preflight(port):
            transport, ids = adapters[port], group_of[port].ids
            uids = transport.identify(deadline_ns=clock()+SETUP_EXCHANGE_NS)
            need(isinstance(uids, Mapping) and set(uids) == set(ids), 'Exact same-group identity replies required')
            for mid in ids:
                need(_reply_hex(uids[mid], 'mcu_uid_hex') == spec['uids'][mid], 'ID%d UID mismatch' % mid)
            stopped = _feedback_rows(transport, transport.stop(deadline_ns=clock()+SETUP_EXCHANGE_NS), ids)
            for mid in ids:
                feedback = stopped[mid, 'feedback'][0]
                need(feedback.mode_state == 0 and feedback.fault_bits == 0, 'ID%d initial STOP/fault not clear' % mid)
            versions = transport.version_probe(deadline_ns=clock()+VERSION_PROBE_NS)
            need(isinstance(versions, Mapping) and set(versions) == set(ids), 'Exact same-group version replies required')
            firmware = {mid: _reply_hex(versions[mid], 'version_bytes_hex') for mid in ids}
            for mid in ids:
                need(firmware[mid] == spec['firmware'][mid], 'ID%d fresh firmware bytes differ from tested watchdog firmware' % mid)
            modes = _param_rows(transport, transport.read_params(('run_mode', 'voltage'),
                                deadline_ns=clock()+SETUP_EXCHANGE_NS), ids, ('run_mode', 'voltage'))
            for mid in ids:
                need(modes[mid, 'run_mode'][0].get('value') == 0, 'ID%d MIT mode is not configured' % mid)
                voltage = modes[mid, 'voltage'][0].get('value')
                need(type(voltage) in (int, float) and profile['voltage_min_v'] <= voltage <= profile['voltage_max_v'],
                     'ID%d supply voltage' % mid)
            acks = _feedback_rows(transport, transport.write_watchdog(protocol.WATCHDOG_TICKS,
                                  deadline_ns=clock()+SETUP_EXCHANGE_NS), ids)
            for mid in ids:
                feedback = acks[mid, 'feedback'][0]
                need(feedback.mode_state == 0 and feedback.fault_bits == 0, 'ID%d watchdog setup acknowledgement' % mid)
            timeouts = _param_rows(transport, transport.read_params(('can_timeout',),
                                   deadline_ns=clock()+SETUP_EXCHANGE_NS), ids, ('can_timeout',))
            for mid in ids:
                need(timeouts[mid, 'can_timeout'][0].get('value') == protocol.WATCHDOG_TICKS, 'ID%d watchdog readback' % mid)
            direct = _param_rows(transport, transport.read_params(('position', 'velocity'),
                                 deadline_ns=clock()+SETUP_EXCHANGE_NS), ids, ('position', 'velocity'))
            final = _feedback_rows(transport, transport.stop(deadline_ns=clock()+SETUP_EXCHANGE_NS), ids)
            return {'firmware': firmware, 'direct': direct, 'stopped': final, 'watchdog_readback': timeouts}
        found = all_ports(preflight, 1.5)
        starts, turns_seen = {}, {}
        for port, value in found.items():
            report['preflight'][port] = {'firmware_by_id': {str(mid): hexa for mid, hexa in value['firmware'].items()},
                                         'watchdog_ticks_by_id': {str(mid): row[0]['value']
                                             for (mid, _), row in value['watchdog_readback'].items()}}
            for mid in group_of[port].ids:
                axis = axes[str(mid)]
                raw = value['direct'][mid, 'position'][0].get('value')
                feedback = value['stopped'][mid, 'feedback'][0]
                need(feedback.mode_state == 0 and feedback.fault_bits == 0, 'ID%d preflight mode/fault' % mid)
                try:
                    branch = resolve_unique_numeric_branch(raw, sign=axis['sign'], offset_rad=axis['offset_rad'],
                        lower_rad=axis['physical_lower_rad'], upper_rad=axis['physical_upper_rad'],
                        uncertainty_rad=BRANCH_MARGIN_RAD)
                except (AngleEvidenceError, KeyError, TypeError, ValueError) as error:
                    raise RuntimeError('ID%d ambiguous/out-of-range initial encoder branch: %s' % (mid, error)) from error
                need(branch['turns'] == spec['turns'][mid], 'ID%d branch differs from admitted reference turns' % mid)
                need(abs(raw-feedback.protocol_position_rad) <= POSE_TOLERANCE_RAD, 'ID%d Type17/Type2 branch or scale mismatch' % mid)
                velocity = value['direct'][mid, 'velocity'][0].get('value')
                need(type(velocity) in (int, float) and abs(velocity) <= axis['max_measured_velocity_rad_s'],
                     'ID%d initial velocity' % mid)
                turns_seen[str(mid)] = branch['turns']
                starts[mid] = feedback.protocol_position_rad
        report['firmware_versions_match_watchdog_review'] = True
        report['initial_raw_rad_by_id'] = {str(mid): starts[mid] for mid in IDS}
        report['fixed_branch_turns_by_id'] = turns_seen
        report['fixed_offsets_rad_by_id'] = {str(mid): offsets[mid] for mid in IDS}

        # C. Announcement, post-speech watchdog readback, model, pose (OR:1599-1714).
        check()
        announce()
        check()
        announced = report['announcement_completed_ns'] = clock()
        def readback(port):
            transport = adapters[port]
            return _param_rows(transport, transport.read_params(('can_timeout',), deadline_ns=clock()+SETUP_EXCHANGE_NS),
                               group_of[port].ids, ('can_timeout',))
        report['after_announcement_watchdog_readback_by_id'] = {}
        for port, rows in all_ports(readback).items():
            for (mid, _), (value, started, received) in rows.items():
                need(value.get('value') == protocol.WATCHDOG_TICKS, 'ID%d pre-enable watchdog readback' % mid)
                need(started >= announced, 'ID%d watchdog read predates announcement' % mid)
                report['after_announcement_watchdog_readback_by_id'][str(mid)] = {
                    'value_ticks': value['value'], 'request_started_ns': started, 'received_ns': received}
        report['model_setup'] = model_setup(observer, pre_calls=10, post_calls=10)
        if (not isinstance(report['model_setup'], Mapping) or report['model_setup'].get('pre_calls') != 10 or
                report['model_setup'].get('post_calls') != 10 or report['model_setup'].get('reset_verified') is not True):
            raise ValueError('Selected-model 10+10 warmup and reset required')
        if genuine and (report['model_setup'].get('real_model_verified') is not True or
                report['model_setup'].get('selected_model_warmup_verified') is not True or
                not report['model_setup'].get('model_artifact_sha256') or
                not report['model_setup'].get('model_source_sha256')):
            raise ValueError('Current actual selected model/source/warmup binding required')
        if gc_freeze:
            # F4: right after model warm-up, while every motor is still disabled.
            # Freezing the whole heap takes tens of ms, so it must precede the
            # pose recapture, enable and the 21 ms command/sample gap window.
            freeze_begin = clock()
            gc.freeze()
            frozen = True
            report['gc_freeze'] = {'selected': True, 'frozen_before_first_release': gc.get_freeze_count(),
                                   'frozen_before_enable': True, 'freeze_ms': (clock()-freeze_begin)/1e6,
                                   'unfrozen_at_restoration': False}
        check()
        def pose(port):
            transport = adapters[port]
            return _feedback_rows(transport, transport.stop(deadline_ns=clock()+SETUP_EXCHANGE_NS), group_of[port].ids)
        current = {}
        for rows in all_ports(pose).values():
            current.update(rows)
        for mid in IDS:
            need(abs(current[mid, 'feedback'][0].protocol_position_rad-starts[mid]) <= POSE_TOLERANCE_RAD,
                 'ID%d moved during preparation; recapture required' % mid)
        initial_sample = feedback_sample(current, profile, offsets, now_ns=clock(), required_mode=0)
        validate_measured(initial_sample, profile)
        trial_origin = initial_sample  # Never re-based (OR:1676-1681).
        report['trial_origin_model_rad_by_id'] = {str(mid): trial_origin.q_model_rad[mid-1] for mid in IDS}
        def displacement(rows, stage):
            for (mid, _), row in rows.items():
                feedback = row[0] if type(row) is tuple else row
                axis = axes[str(mid)]
                q = axis['sign']*feedback.protocol_position_rad+offsets[mid]
                delta = q-trial_origin.q_model_rad[mid-1]
                report['startup_displacement_checks'].append(
                    {'motor_id': mid, 'stage': stage, 'q_model_rad': q, 'displacement_rad': delta})
                need(abs(delta) <= axis['max_displacement_from_start_rad'],
                     'ID%d startup trial displacement from pre-enable origin' % mid)
        imu = pools['imu'].submit(imu_read).result(timeout=.25)
        now = clock()
        last_imu = validate_imu_metadata(imu, now, profile)
        report['pre_enable_imu'] = copy.deepcopy(imu)
        report['pre_enable_imu_limit_check'] = dict(zip(
            ('tilt_rad', 'raw_tilt_rad', 'body_gyro_norm_rad_s', 'raw_accel_norm_m_s2'),
            check_imu_limits(imu, imu_limits, imu_rotation, imu_bias, accel_correction)))
        need(now/1e9-initial_sample.monotonic_s <= profile['max_sample_age_ms']/1000,
             'Initial motor samples became stale during IMU acquisition')
        for mid in IDS:
            lo, hi = profile['start_pose_bounds'][str(mid)]
            need(lo <= initial_sample.q_model_rad[mid-1] <= hi, 'ID%d outside reviewed starting posture' % mid)
        refresh_voltage('pre_enable_voltage_by_id')
        need(clock()/1e9-initial_sample.monotonic_s <= profile['max_sample_age_ms']/1000,
             'Initial motor samples became stale during voltage refresh')

        # D. Serial enable then zero gain per axis, 30 ms / 120 ms, no retry (OR:1720-1768).
        transition = report['zero_gain_enable_transition'] = {
            'enable_reply_budget_ms': 30., 'total_budget_ms': 120., 'motion_retry_allowed': False,
            'strategy': 'serial_axis_enable_then_zero_gain_four_port_v1',
            'ordered_axes': [{'port': port, 'motor_id': mid} for port, mid in enable_order(groups)],
            'completed_axes': [], 'current_axis': None, 'current_stage': None, 'complete': False}
        transition['begin_ns'] = begin = clock()
        transition['deadline_ns'] = transition_deadline = begin+ENABLE_TOTAL_NS
        def bounded(desired):
            remaining = transition_deadline-clock()
            need(remaining >= 1_000_000, 'Zero-gain enable sequence deadline exceeded')
            return clock()+min(desired, remaining)
        watcher.kick()
        for port, mid in enable_order(groups):
            check()
            require_voltage_before_type1()
            transition.update(current_axis={'port': port, 'motor_id': mid}, current_stage='enable')
            report['motor_enable_sent'] = True
            reply = _single_feedback(single(port, adapters[port].enable, mid, deadline_ns=bounded(ENABLE_REPLY_NS)), mid)
            need(reply.mode_state in (0, 2) and reply.fault_bits == 0, 'ID%d enable transition failed' % mid)
            displacement({(mid, 'feedback'): reply}, 'enable')
            watcher.kick()
            require_voltage_before_type1()
            transition['current_stage'] = 'zero_gain'
            report['type1_sent'] = True
            reply = _single_feedback(single(port, adapters[port].zero_gain, mid, starts[mid],
                                            deadline_ns=bounded(PERIOD_NS)), mid)
            need(reply.mode_state == 2 and reply.fault_bits == 0, 'ID%d zero-gain transition failed' % mid)
            displacement({(mid, 'feedback'): reply}, 'zero_gain')
            transition['completed_axes'].append(mid)
            watcher.kick()
        transition['end_ns'] = clock()
        need(transition['end_ns'] < transition_deadline, 'Zero-gain enable sequence deadline exceeded')
        transition.update(complete=True, current_axis=None, current_stage=None)
        check()
        refresh_voltage('after_enable_voltage_by_id')
        checked_voltage_cache(voltage_cache, profile, clock())

        # E. All-port zero-gain hold sets q0; envelope and encoder (OR:1784-1823).
        last_wires = {group.port: tuple(encode_motion(mid, starts[mid], 0., 0.) for mid in group.ids)
                      for group in groups}
        require_voltage_before_type1()
        deadline = clock()+PERIOD_NS
        futures = {port: submit(port, adapters[port].output, last_wires[port], deadline_ns=deadline,
                                check=owner_check) for port in PORTS}
        in_flight[:] = list(futures.values())
        initial_hold = {port: _take(futures[port], deadline, clock, hot) for port in PORTS}
        in_flight.clear()
        fresh_zero = {}
        for port, batch in initial_hold.items():
            need(tuple(bytes(record.tx) for record in batch.records) == last_wires[port] and
                 batch.rows.keys() == {(mid, 'feedback') for mid in group_of[port].ids},
                 'Exact initial zero-gain hold batch required')
            fresh_zero.update(batch.rows)
        report['initial_hold'] = initial_hold
        initial_sample = feedback_sample(fresh_zero, profile, offsets, now_ns=clock())
        displacement(fresh_zero, 'all_axis_zero_gain')
        validate_measured(initial_sample, profile, initial=trial_origin)
        q0 = initial_sample.q_model_rad
        report['q0_model_rad_by_id'] = {str(mid): q0[mid-1] for mid in IDS}
        limits = tuple(replace(AxisLimits(**{key: axis[key] for key in AxisLimits.__dataclass_fields__}),
                               lower_rad=max(axis['lower_rad'], q-axis['max_displacement_from_start_rad']),
                               upper_rad=min(axis['upper_rad'], q+axis['max_displacement_from_start_rad']),
                               **({} if learned else {'kp': 0., 'kd': 0.}))
                       for axis, q in zip(axis_rows, trial_origin.q_model_rad))
        last_command_ns = clock()
        last_sample_ns = min(row[1] for row in fresh_zero.values())
        envelope = PolicyMotionEnvelope(limits, initial_sample, now_s=last_command_ns/1e9,
            startup_duration_s=profile['startup_duration_s'], stop_duration_s=profile['stop_duration_s'],
            startup_damping_duration_s=profile['startup_damping_duration_s'],
            max_sample_age_s=profile['max_sample_age_ms']/1000,
            max_sample_gap_s=profile['max_sample_gap_ms']/1000)
        report['envelope_gains'] = 'profile_kp_kd' if learned else 'zero_kp_kd'
        if encoder is None:
            def wires_for(command):
                return _python_motion_wires(command, offsets, axes, trial_origin.q_model_rad,
                                            encode_motion=encode_motion)
        else:
            wires_for = encoder.bind(tuple((offsets[mid], axis['sign'], axis['lower_rad'], axis['upper_rad'],
                trial_origin.q_model_rad[mid-1], axis['max_displacement_from_start_rad'],
                axis['max_estimated_pd_torque_nm']) for mid, axis in zip(IDS, axis_rows)))
        watcher.kick()
        previous = fresh_zero
        if genuine:
            need(not gc.isenabled(), 'Main scope must defer automatic GC during cycles')
        report['gc_enabled_during_cycles'] = gc.isenabled()
        check()

        # F-H. Cycles at absolute 20 ms epoch slots; ramp-down; (OR:1825-2146).
        expected_keys = {group.port: frozenset((mid, 'feedback') for mid in group.ids) for group in groups}
        voltage_wires = {mid: codec.read_request(mid, 'voltage') for mid in IDS}
        lateness_ns = int(POST_REPLY_V1['max_lateness_ms']*1e6)
        weight_scale, startup_s, ramp_s = profile['policy_weight'], profile['startup_duration_s'], profile['policy_ramp_s']
        first_cycle_allowed = spec['first_cycle_post_reply']
        stop_at_ns = int(spec['stop_at_s']*1e9)
        max_run_ns = int((spec['duration_s']+.04)*1e9)
        can_order = tuple(shadow.CAN_ORDER)
        model_limits = tuple(MODEL_TARGET_LIMITS_BY_ID[mid] for mid in can_order)
        local_limits = tuple((axis['lower_rad'], axis['upper_rad']) for axis in axis_rows)
        output_limits = tuple((axis['lower_rad'], axis['upper_rad'], axis['max_measured_torque_nm'],
            axis['max_measured_velocity_rad_s'], axis['max_temperature_c'], axis['max_tracking_error_rad'],
            axis['max_displacement_from_start_rad'], axis['max_estimated_pd_torque_nm']) for axis in axis_rows)
        origin_q = trial_origin.q_model_rad
        if timing_evidence is not None:  # F0 diagnostic only; read outside every cycle window.
            threads = {'main': threading.get_native_id(), **dict(zip((*PORTS, 'imu'), worker_tids)),
                       'host_watchdog': report['watchdog_settings']['native_tid']}
            report['timing_evidence'] = {'selected': True, 'contract_input': False,
                'sampled': 'each_cycle_end_after_post_reply_admission_outside_release_to_output_window',
                'binding': timing_evidence.bind(threads)}
            evidence_before = timing_evidence.run_counters()
            evidence_baseline = timing_evidence.sample()
        start = clock()
        if prearm_ns is not None:
            start += prearm_ns  # Epoch one lead ahead, so cycle 0 is pre-armed as well.
        report['epoch_ns'] = start
        previous_slot = previous_begin = None
        stop_started = False
        report['cycles'] = cycles
        while clock()-start < max_run_ns:
            index = len(cycles)
            hot()
            if prearm_ns is None:
                slot, release = _absolute_epoch_slot(start, previous_slot, previous_begin, clock())
                if slot != index:
                    raise RuntimeError('Absolute-epoch cycle slot skipped; STOP before another hold')
                begun = release_wait(release)
                if type(begun) is not int or begun < release or begun > clock():
                    raise RuntimeError('Release waiter must return actual current monotonic time')
                if _absolute_epoch_slot(start, previous_slot, previous_begin, begun) != (slot, release):
                    raise RuntimeError('Absolute-epoch release missed its slot; STOP before another hold')
                hard_end = begun+PERIOD_NS
                row = _Cycle(index, slot, release, begun)
                cycles.append(row)
                check()  # Boundary 1: cycle_start_before_submit.
                hold_now = clock()
                prearmed = {}
            else:
                # F3: the slot is the next release this cycle can still pre-arm for.
                slot, release = _absolute_epoch_slot(start, previous_slot, previous_begin, clock()+prearm_ns)
                if slot != index:
                    raise RuntimeError('Absolute-epoch cycle slot skipped; STOP before another hold')
                woke = release_wait(release-prearm_ns)
                if type(woke) is not int or woke < release-prearm_ns or woke > clock():
                    raise RuntimeError('Release waiter must return actual current monotonic time')
                if (woke >= release or
                        _absolute_epoch_slot(start, previous_slot, previous_begin, woke+prearm_ns) != (slot, release)):
                    raise RuntimeError('Pre-armed wake missed its lead before the release; STOP before another hold')
                hard_end = release+PERIOD_NS
                row = _Cycle(index, slot, release, None)
                row.prearm_wake_ns = woke
                cycles.append(row)
                check()  # Boundary 1, pre-armed: before_release_before_prearmed_submit.
                hold_now = release  # Every hold gate is evaluated at the release, when the hold is written.
                prearmed = {'not_before_ns': release}
            if hold_now-last_command_ns > gap_ns or hold_now-last_sample_ns > gap_ns:
                raise RuntimeError('Command/sample gap exceeded before feedback hold')
            for mid in IDS:
                _, old_start, old_end = previous[mid, 'feedback']
                if not (0 < old_start <= old_end <= hold_now and hold_now-old_start <= age_ns):
                    raise RuntimeError('ID%d stale feedback before hold' % mid)
            if prearm_ns is None:
                require_voltage_before_type1()
            else:
                require_voltage_before_type1(release)
                if clock() >= release:
                    raise TimeoutError('Pre-armed hold gates reached the release; STOP before another hold')
            prefix, full, voltage_id = {}, {}, {}
            entries = None
            if timing_evidence is not None:
                entries = row.owner_entry_ns = {}
            in_flight.clear()
            for group in groups:
                port = group.port
                future = Future()
                future.set_running_or_notify_cancel()
                prefix[port] = future
                voltage_id[port] = group.ids[index % 3]
                if entries is None:
                    full[port] = pools[port].submit(_owned, supervisor, port, adapters[port].hold_then_voltage,
                        (last_wires[port], voltage_id[port], future),
                        {'deadline_ns': hard_end, 'check': owner_check, **prearmed})
                else:
                    full[port] = pools[port].submit(_owned_stamped, entries, clock, supervisor, port,
                        adapters[port].hold_then_voltage, (last_wires[port], voltage_id[port], future),
                        {'deadline_ns': hard_end, 'check': owner_check, **prearmed})
                in_flight.append(full[port])
            if prearm_ns is not None:
                # Main waits natively for the actual release; the armed owners write on their own.
                begun = release_wait(release)
                if type(begun) is not int or begun < release or begun > clock():
                    raise RuntimeError('Release waiter must return actual current monotonic time')
                if _absolute_epoch_slot(start, previous_slot, previous_begin, begun) != (slot, release):
                    raise RuntimeError('Absolute-epoch release missed its slot; STOP before another hold')
                row.begin_ns = begun
            # Post-reply rules assume every input after the cycle begin; pre-armed holds are
            # written natively at or after the release, possibly before main's own wake.
            rule_begin = begun if prearm_ns is None else release
            imu_future = pools['imu'].submit(imu_read)
            in_flight.append(imu_future)
            if timing_evidence is not None:
                row.submit_done_ns = clock()
            hold = {port: _take(prefix[port], hard_end, clock, hot) for port in PORTS}
            imu = _take(imu_future, hard_end, clock, hot)
            acquired = row.acquired_ns = clock()
            row.hold, row.imu = hold, imu
            last_imu = validate_imu_metadata(imu, acquired, profile, previous=last_imu)
            # Body limits hold on every cycle, gain-down included (OR:1957).
            row.imu_check = check_imu_limits(imu, imu_limits, imu_rotation, imu_bias, accel_correction)
            first = last_imu
            rows = {}
            for port in PORTS:
                if decode_once:  # F2b: the owner's own read-only publication, raw images compared.
                    current = adapters[port].verify_batch(hold[port], 'feedback_hold')
                else:
                    current = hold[port].verify()
                if current.keys() != expected_keys[port]:
                    raise RuntimeError('Exact-three Type1 hold feedback rows required')
                rows.update(current)
                for record in hold[port].records:
                    if record.start_ns < first:
                        first = record.start_ns
            row.first_ns = first
            if first+age_ns < hard_end:
                hard_end = first+age_ns
            row.hard_end_ns = hard_end
            if acquired >= hard_end:
                raise TimeoutError('Hold acquisition exceeded hard cycle or sample-age deadline')
            sample = row.sample = feedback_sample(rows, profile, offsets, now_ns=acquired, previous=previous)
            validate_measured(sample, profile, initial=trial_origin)
            checked_voltage_cache(voltage_cache, profile, acquired)
            snapshot = snapshot_builder(hold, last_wires, imu, acquired)
            imu_image = (imu['read_started_monotonic_ns'], imu['read_finished_monotonic_ns'],
                         tuple(imu['accel_m_s2']), tuple(imu['gyro_rad_s']))
            row.gather_ns = clock()
            check()  # Boundary 2: after_feedback_joins_before_inference.
            if clock() >= hard_end:
                raise TimeoutError('Deadline before inference')
            if not stop_started and (stop_requested.is_set() or begun-start >= stop_at_ns):
                envelope.request_stop()
                stop_started = True
            weight = 0.
            if stop_started:
                target = None  # No inference during gain-down; IMU metadata and body limits were checked.
            else:
                observed = row.observed = observer.consume(snapshot)
                model = observed.get('q_target_rad_diagnostic_only') if type(observed) is dict else None
                if type(model) not in (list, tuple) or len(model) != 12:
                    raise RuntimeError('Observer must return twelve CAN_ORDER targets')
                by_id = [0.]*12
                for value, mid, (lower, upper) in zip(model, can_order, model_limits):
                    if type(value) not in (int, float) or not lower <= value <= upper:
                        raise RuntimeError('ID%d learned target outside model range' % mid)
                    by_id[mid-1] = float(value)
                fraction = max(0., min(1., ((begun-start)/1e9-startup_s)/ramp_s))
                weight = weight_scale*fraction**3*(10.+fraction*(-15.+6.*fraction))
                blended = tuple(base+weight*(q-base) for q, base in zip(by_id, q0))
                for mid, (q, (lower, upper)) in enumerate(zip(blended, local_limits), 1):
                    if not lower <= q <= upper:
                        raise RuntimeError('ID%d blended target outside local physical range' % mid)
                row.model_target, row.blended_target = by_id, blended
                target = blended if learned else q0
            row.weight = weight
            row.infer_end_ns = clock()
            voltage = {}
            for port in PORTS:
                actual, batch = _take(full[port], hard_end, clock, hot)
                if actual is not hold[port]:
                    raise RuntimeError('Current exact hold/full Future binding failed')
                mid = voltage_id[port]
                records = batch.records
                current = adapters[port].verify_batch(batch, 'voltage') if decode_once else batch.verify()
                if len(records) != 1 or bytes(records[0].tx) != voltage_wires[mid] or current.keys() != {(mid, 'voltage')}:
                    raise RuntimeError('Current rotating same-group voltage required')
                voltage_cache.update(checked_voltage_rows(current, (mid,), profile, clock()))
                voltage[port] = batch
            row.voltage = voltage
            checked_voltage_cache(voltage_cache, profile, clock())
            row.voltage_join_ns = clock()
            # Final gate: every raw hold/voltage image re-decoded (decode-once: hold
            # images compared to the owner's publication), IMU unchanged.
            if decode_once:
                for port in PORTS:
                    adapters[port].verify_batch(hold[port], 'feedback_hold')
                    adapters[port].verify_batch(voltage[port], 'voltage')
            else:
                for port in PORTS:
                    hold[port].verify()
                    voltage[port].verify()
            if (imu['read_started_monotonic_ns'], imu['read_finished_monotonic_ns'],
                    tuple(imu['accel_m_s2']), tuple(imu['gyro_rad_s'])) != imu_image:
                raise RuntimeError('IMU image changed after inference')
            check()  # Boundary 3: after_final_gate_before_output.
            computed = row.final_gate_ns = clock()
            if command_offset_ns is not None:
                # F1: command time is release+K unless the natural gate is later. The
                # join, final gate and Boundary 3 stay before it; a cancel sends nothing.
                row.natural_gate_ns = computed
                target_ns = release+command_offset_ns
                if computed < target_ns:
                    woke = command_wait(target_ns)
                    if type(woke) is not int or woke < target_ns or woke > clock():
                        raise RuntimeError('Command phase waiter must return actual current monotonic time')
                    hot()
                    computed = clock()
                row.command_ns = computed
            if computed >= hard_end:
                raise TimeoutError('Inference exceeded hard cycle deadline')
            command = row.command = envelope.step(target, sample, now_s=computed/1e9)
            outgoing = split_wires_by_port(wires_for(command), groups, zero_gain=not learned)
            row.encode_end_ns = clock()
            require_voltage_before_type1()
            if clock() >= hard_end:
                raise TimeoutError('Encoded command exceeded hard cycle or sample-age deadline')
            # Mark intent before the first write, including a partial transaction (OR:2011-2013).
            if learned and weight > 0.:
                report['learned_targets_attempted'] = True
            if any(command.kp) or any(command.kd):
                report['positive_gain_sent'] = True
            row.label = ('graceful_stop' if stop_started else 'policy_output' if learned and weight > 0.
                         else 'startup_hold' if learned else 'zero_gain_timing')
            row.output_submit_ns = clock()
            output = {}
            in_flight.clear()
            for port in PORTS:
                output[port] = pools[port].submit(_owned, supervisor, port, adapters[port].output,
                    (outgoing[port],), {'deadline_ns': hard_end, 'check': owner_check})
                in_flight.append(output[port])
            join_deadline = row.join_deadline_ns = min(first+age_ns, rule_begin+PERIOD_NS+lateness_ns)
            batches = {port: _take(output[port], join_deadline, clock, hot) for port in PORTS}
            reply_return = row.reply_return_ns = clock()
            row.output = batches
            returned = {}
            final_write = last_reply = 0
            count = 0
            for port in PORTS:
                batch = batches[port]
                records = batch.records
                if len(records) != 3:
                    raise RuntimeError('Exactly three Type1 output replies per port required')
                for record, wire in zip(records, outgoing[port]):
                    if bytes(record.tx) != wire:
                        raise RuntimeError('Output record differs from the validated outgoing wire')
                    if record.finish_ns > final_write:
                        final_write = record.finish_ns
                    if record.received_ns > last_reply:
                        last_reply = record.received_ns
                if batch.rows.keys() != expected_keys[port]:
                    raise RuntimeError('Exact-three Type1 output feedback rows required')
                returned.update(batch.rows)
                count += len(hold[port].records)+len(voltage[port].records)+len(records)
            row.request_count = count
            if count != PER_CYCLE_REQUESTS:
                raise RuntimeError('Four-bus 28 transaction proof incomplete')
            # Validate returned limits before the next hold reuses them (OR:2040-2051).
            checked = row.checked = feedback_sample(returned, profile, offsets, now_ns=reply_return, previous=rows)
            for k, (lower, upper, torque, velocity, temperature, tracking, travel, pd) in enumerate(output_limits):
                q = checked.q_model_rad[k]
                if not lower <= q <= upper:
                    raise RuntimeError('ID%d joint limit' % (k+1))
                if abs(checked.torque_nm[k]) > torque:
                    raise RuntimeError('ID%d torque' % (k+1))
                if abs(checked.velocity_rad_s[k]) > velocity:
                    raise RuntimeError('ID%d velocity' % (k+1))
                if checked.temperature_c[k] > temperature:
                    raise RuntimeError('ID%d temperature' % (k+1))
                if abs(q-command.q_model_rad[k]) > tracking:
                    raise RuntimeError('ID%d tracking error' % (k+1))
                if abs(q-origin_q[k]) > travel:
                    raise RuntimeError('ID%d trial displacement' % (k+1))
                if abs(command.kp[k]*(command.q_model_rad[k]-q)-command.kd[k]*checked.velocity_rad_s[k]) > pd:
                    raise RuntimeError('ID%d estimated PD torque' % (k+1))
            output_sample_start = min(value[1] for value in returned.values())
            try:
                decision = budget.admit(index=index, begin_ns=rule_begin, oldest_input_ns=first,
                    final_write_ns=final_write, last_reply_ns=last_reply,
                    output_sample_start_ns=output_sample_start, checked_ns=clock(), sample_age_ns=age_ns,
                    startup_allowed=first_cycle_allowed and index == 0)
            except RuntimeError as error:
                row.decision = {'accepted': False, 'rejection': str(error)}
                raise
            row.decision = decision
            row.cycle_end_ns = decision['checked_ns']
            row.completed = True
            watcher.kick()
            previous, last_wires = returned, outgoing
            previous_begin, previous_slot = begun, slot
            last_command_ns = computed
            last_sample_ns = min(value[1] for value in rows.values())
            in_flight.clear()
            report['completed_cycles'] = index+1
            if timing_evidence is not None:  # After admission; outside the next release window.
                sampled = clock()
                row.thread_counters = timing_evidence.sample()
                row.evidence_cost_ns = clock()-sampled
            if command.phase == 'stopped':
                report['normal_ramp_completed'] = True
                break
        if not report['normal_ramp_completed']:
            raise RuntimeError('Finite run budget expired before normal stop')
        report['status'] = 'COMPLETE_FOUR_BUS_TYPE1_'+spec['mode'].upper()
    except BaseException as error:
        primary = error
        report['status'] = 'ABORTED'
        report['primary_error'] = {'type': type(error).__name__, 'message': str(error)}
        report['errors'].append(type(error).__name__+': '+str(error))
        supervisor.emergency(type(error).__name__+': '+str(error))
    finally:
        # I. Terminal/emergency repeated subset STOP on every opened owner, then J. restore.
        try:
            stops = supervisor.finish()
        except BaseException as cleanup:
            stops = {}
            cleanup_error(cleanup)
        report['stop_latched_ns'] = supervisor.latched_ns
        report['stop_reason'] = supervisor.reason
        report['stop_dispatch_errors'] = list(supervisor.errors)
        if evidence_before is not None:  # After STOP; diagnostic only.
            try:
                report['timing_evidence']['run'] = timing_evidence.run_delta(evidence_before,
                                                                             timing_evidence.run_counters())
            except BaseException as cleanup:
                report['timing_evidence']['run_error'] = type(cleanup).__name__+': '+str(cleanup)
        if timing_evidence is not None:
            try:
                timing_evidence.close()
            except BaseException as cleanup:
                cleanup_error(cleanup)
        if watcher is None and created is not None:
            try:  # Created but not yet returned to this thread: still close its poller.
                watcher = created.result(timeout=1)[0]
            except BaseException:
                pass
        if watcher is not None:
            try:
                watcher.close()
            except BaseException as cleanup:
                cleanup_error(cleanup)
        for index, future in enumerate(in_flight):
            value = {'cleanup_only': True, 'admission_eligible': False}
            try:
                future.result(timeout=.5)
                value['state'] = 'SUCCESS'
            except BaseException as cleanup:
                value.update(state='EXCEPTION' if future.done() else 'PENDING', error=str(cleanup))
            value['observation_monotonic_ns'] = clock()
            report['owner_settlement'].append(value)
        for port, pool in pools.items():
            # FIFO after STOP and every original exchange on this physical bus.
            restoration = pool.submit(restore, port)
            pool.shutdown(wait=True, cancel_futures=False)
            try:
                restoration.result()
            except BaseException as cleanup:
                report['restoration'][port] = {'error': str(cleanup)}
                cleanup_error(cleanup)
        report['all_workers_joined_monotonic_ns'] = clock()
        for port, adapter in adapters.items():
            try:
                adapter.close()
            except BaseException as cleanup:
                report['restoration'][port+'_session'] = {'error': str(cleanup)}
                cleanup_error(cleanup)
            else:
                report['restoration'][port+'_session'] = True
        report['raw_journal'] = {port: list(getattr(adapter, 'journal', ())) for port, adapter in adapters.items()}
        if frozen:
            gc.unfreeze()
            report['gc_freeze']['unfrozen_at_restoration'] = gc.get_freeze_count() == 0
        if main_entered:
            try:
                main_context.__exit__(type(primary) if primary else None, primary, None)
                report['restoration']['main'] = True
            except BaseException as cleanup:
                report['restoration']['main'] = {'error': str(cleanup)}
                cleanup_error(cleanup)
        report['stop_results'] = stops
        opened = set(adapters)
        if not opened:
            report['stop_confirmed'] = None
            report['physical_cutoff_required'] = False
        else:
            report['stop_confirmed'] = opened == set(PORTS) and all(
                _stop_complete(stops.get(port), group_of[port].ids) for port in PORTS)
            report['physical_cutoff_required'] = not report['stop_confirmed']
            if not report['stop_confirmed']:
                report['status'] = 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED'
                report['errors'].append('All-axis STOP not confirmed; physical power cutoff required')
        if primary is not None and report['status'].startswith('COMPLETE'):
            report['status'] = 'ABORTED'
        report['host_watchdog_reason'] = supervisor.reason
        report['post_reply_deadline_allowance_uses'] = budget.accepted_misses
        # Dictionaries and unit conversions only after every owner has stopped.
        try:
            extra = ((('natural_gate_ns', 'command_ns') if command_offset_ns is not None else ()) +
                     (('prearm_wake_ns',) if prearm_ns is not None else ()) +
                     (('submit_done_ns', 'owner_entry_ns', 'evidence_cost_ns') if timing_evidence is not None else ()))
            report['cycles'] = [row.materialize(extra) for row in cycles]
            if timing_evidence is not None:
                counters = evidence_baseline
                for row, value in zip(cycles, report['cycles']):
                    value['thread_counter_delta'] = (None if row.thread_counters is None or counters is None
                                                     else timing_evidence.delta(counters, row.thread_counters))
                    counters = row.thread_counters if row.thread_counters is not None else counters
            report['post_reply_late_cycles'] = sum(1 for row in report['cycles'] if row['decision'] and
                                                   row['decision'].get('lateness_ms', 0) > 0)
            report['max_iteration_ms'] = max((row['iteration_ms'] for row in report['cycles']
                                              if row['iteration_ms'] is not None), default=None)
        except BaseException as cleanup:
            cleanup_error(cleanup)
            if report['status'].startswith('COMPLETE'):
                report['status'] = 'ABORTED'
        report['failure_retained'] = primary is not None
    return report


__all__ = ('MODES', 'DURATIONS', 'IMU_LIMIT_RANGES', 'PACING', 'POST_REPLY_V1', 'REQUIRED_ADMITTED_KEYS',
           'SCHEMA', 'Type1TransportContract', 'check_imu_limits', 'enable_order', 'evidence_report',
           'imu_frame', 'plan', 'run', 'split_wires_by_port', 'type1_hold_snapshot_builder',
           'validate_admitted')
