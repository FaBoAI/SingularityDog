"""File-only reviewed profile for a short, supported learned-policy experiment.

Loading verifies pinned files and review consistency, not physical truth. A
candidate/template never grants output. The runner still owns fresh identity,
power-epoch, mode-0/watchdog readbacks, physical support and cancellation checks.
No transport, network, torch loading or automatic candidate promotion exists.
"""
import argparse
import copy
import difflib
from datetime import datetime
import hashlib
import json
import math
import statistics
from pathlib import Path
import uuid
import wave

from . import policy_shadow as shadow
from .policy_observer import _bias
from .imu_calibration_review import reviewed_acceleration

SCHEMA_V1 = 'singularitydog.supported-policy-profile.v1'
SCHEMA_V2 = 'singularitydog.supported-policy-profile.v2'
SCHEMA_V3 = 'singularitydog.supported-policy-profile.v3'
# V3 is opt-in; existing/default templates keep their original V2 contract.
SCHEMA = SCHEMA_V2
REVIEW_SCHEMA = 'singularitydog.supported-policy-hardware-review.v1'
IDS = tuple(str(i) for i in range(1, 13))
ARTIFACTS = ('calibration', 'mount', 'bias', 'model_manifest', 'pipeline_diagnostic', 'hardware_review')
LIMIT_CAPS = {
    'kp': 30., 'kd': 1., 'max_command_velocity_rad_s': .5,
    'max_command_acceleration_rad_s2': 2., 'max_tracking_error_rad': .25,
    'max_measured_velocity_rad_s': 1., 'max_measured_torque_nm': 3.,
    'max_temperature_c': 60., 'max_estimated_pd_torque_nm': 3.,
    'max_displacement_from_start_rad': math.radians(10),
}
# These are experiment-scope ceilings, not motor/structure safety ratings.
AXIS_KEYS = {'uid', 'sign', 'offset_rad', 'uncertainty_rad', 'physical_lower_rad',
             'physical_upper_rad', *LIMIT_CAPS}
TOP_KEYS_V1 = {'schema', 'scope', 'approved_for_supported_policy_output', 'blockers', 'review',
            'boot_id', 'motor_power_epoch', 'assembly_id', 'axes', 'artifacts', 'bundle_path',
            'duration_s', 'startup_duration_s', 'stop_duration_s', 'policy_ramp_s',
            'policy_weight', 'h_hypothesis', 'command', 'period_ms', 'hard_cycle_ms',
            'max_consecutive_20ms_misses', 'max_sample_age_ms', 'max_sample_gap_ms',
            'voltage_min_v', 'voltage_max_v', 'imu_tilt_limit_rad', 'imu_gyro_limit_rad_s',
            'imu_accel_norm_min_m_s2', 'imu_accel_norm_max_m_s2', 'start_pose_bounds'}
TRANSPORT_KEYS = {'request_gap_us', 'request_window'}
TOP_KEYS = TOP_KEYS_V1 | TRANSPORT_KEYS
CADENCE_KEYS = {'telemetry_cadence', 'cadence_source_sha256'}
TOP_KEYS_V3 = TOP_KEYS | CADENCE_KEYS
V3_EXECUTION_KEYS = {'model_backend', 'voltage_overlap', 'diagnostic_timing_acceptance',
                     'watchdog_review_policy', 'local_characterization', 'post_reply_deadline_policy',
                     'voltage_pipeline', 'native_batch_encoder', 'startup_damping_duration_s',
                     'startup_cycle_allowance', 'fixed_catch', 'human_supported_hold',
                     'apply_reviewed_accel_calibration', 'accel_input_hypothesis',
                     'prepare_voltage_before_feedback_publication'}
COMMAND_LOSS_ONLY_SUPPORTED = 'command_loss_only_supported_trial'
LOCAL_RELATIVE_SUPPORTED = 'bounded_relative_supported_v1'
LOCAL_NUMERICAL_MARGIN_RAD = 2*25.14/65535
_LOCAL_VALIDATION_TOKEN = object()
_ACCEL_INPUT_HYPOTHESIS_TOKEN = object()
_POST_REPLY_VALIDATION_TOKEN = object()
_PREPARED_VOLTAGE_PUBLICATION_TOKEN = object()
PREPARED_VOLTAGE_PUBLICATION_MODE = 'prepare_voltage_before_feedback_publication.v1'
SCALAR_BACKEND = 'scalar_step_cpp'
OBSERVED_R17_TIMING = 'observed-r17-cadence-20260928'
MEASURED_R17_STARTUP_TIMING = 'measured-r17-startup-20260929'
CURRENT_HOLD_PROBE = 'current-position-hold-probe-v1'
CURRENT_HOLD_AFTER_SUPPORTED_10S = 'current-position-hold-after-supported-10s-v1'
FIXED_CATCH_CURRENT_HOLD_30S = 'fixed-catch-current-position-hold-30s-v1'
FIXED_CATCH_SCOPE = 'fixed_catch_current_hold_only'
_FIXED_CATCH_TOKEN = object()
_FIXED_CATCH_ARTIFACTS = ('prior_current_hold_profile', 'prior_current_hold_report',
                         'prior_current_hold_observation', 'fixed_catch_source_review')
_FIXED_CATCH_NEW_SOURCE = 'singularitydog_hw/fixed_catch_hold.py'
_FIXED_CATCH_CHANGED_SOURCES = frozenset(('singularitydog_hw/policy_live_profile.py',
    'singularitydog_hw/policy_output_runtime.py', 'singularitydog_hw/policy_output.py',
    _FIXED_CATCH_NEW_SOURCE))
HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S = 'human-supported-partial-current-hold-audio-8s-v1'
HUMAN_SUPPORTED_PARTIAL_SCOPE = 'human_supported_partial_current_hold_only'
_HUMAN_SUPPORTED_TOKEN = object()
_HUMAN_SUPPORTED_ARTIFACTS = ('human_supported_preparation', 'human_supported_source_review',
                            'human_supported_audio_manifest',
                            'prior_current_hold_profile', 'prior_current_hold_report',
                            'prior_current_hold_observation')
_HUMAN_SUPPORTED_NEW_SOURCE = 'singularitydog_hw/human_supported_hold.py'
_HUMAN_SUPPORTED_CHANGED_SOURCES = frozenset(('singularitydog_hw/policy_live_profile.py',
    'singularitydog_hw/policy_output_runtime.py', 'singularitydog_hw/policy_output.py',
    'singularitydog_hw/native_pipeline_benchmark.py',
    _HUMAN_SUPPORTED_NEW_SOURCE))
SUPPORTED_POLICY_PROBE = 'supported-policy-probe-v1'
SUPPORTED_POLICY_PROBE_5S = 'supported-policy-probe-5s-v1'
SUPPORTED_POLICY_PROBE_2S_RARE_JITTER = 'supported-policy-probe-2s-rare-jitter-v1'
SUPPORTED_POLICY_PROBE_10S_AFTER_2S = 'supported-policy-probe-10s-after-2s-v1'
SUPPORTED_POLICY_PROBE_20S_AFTER_10S = 'supported-policy-probe-20s-after-10s-v1'
SUPPORTED_POLICY_GAIN_STEP_3S = 'supported-policy-gain-step-3s-v1'
SUPPORTED_POLICY_MIX_STEP_10PCT = 'supported-policy-mix-step-10pct-5s-v1'
_MIX_STEP_TOKEN = object()
_MIX_STEP_ARTIFACTS = ('saved_policy_target_sequence', 'policy_mixture_analysis',
                       'mix_step_clearance', 'mix_step_source_review')
_MIX_STEP_CHANGED_SOURCES = frozenset(('singularitydog_hw/policy_live_profile.py',
    'singularitydog_hw/native_pipeline_benchmark.py'))
SUPPORTED_PRELOAD_5S = 'supported-geometric-preload-5s-v1'
_PRELOAD_TOKEN = object()
_PRELOAD_ARTIFACTS = ('preload_source_profile', 'preload_path', 'preload_review')
_PRELOAD_SOURCE_PATH = 'singularitydog_hw/supported_preload_path.py'
FIRST_CYCLE_POST_REPLY = 'first-cycle-post-reply-v1'
_STARTUP_CYCLE_TOKEN = object()
_EXTENSION_ARTIFACTS = ('prior_supported_profile', 'prior_supported_report',
                        'prior_supported_observation')
_CURRENT_HOLD_TOKEN = object()
OBSERVED_R17_REPORT_SHA256 = frozenset((
    '1d0e49226ab095c007d5de63325b8e201467315234e43ed229546b3652cfb2d4',
    '7eaadb878a0ea6f98cfeae1b49312a2bcc4a63d71d4aeb8c9f43c6740afbe204',
))
CADENCE_PRE_ENABLE = 'feedback_voltage_pre_enable_timeout.v1'
CADENCE_SOURCE_PATHS = (
    'singularitydog_hw/policy_live_profile.py',
    'singularitydog_hw/policy_output.py',
    'singularitydog_hw/math_thread_startup.py',
    'singularitydog_hw/policy_output_runtime.py',
    'singularitydog_hw/active_output_timer_slack.py',
    'singularitydog_hw/thread_timer_slack.py',
    'singularitydog_hw/policy_post_reply_timing.py',
    'singularitydog_hw/ground_trial_output.py',
    'singularitydog_hw/ground_trial_review.py',
    'singularitydog_hw/native_active_transport.py',
    'singularitydog_hw/native_policy_batch_encode.py',
    'singularitydog_hw/native_pipeline_benchmark.py',
    'singularitydog_hw/can_readonly.py',
    'singularitydog_hw/rs05_trial_protocol.py',
    'experiments/native_active_transport/transport.cpp',
    'experiments/native_policy_batch_encode/batch_encode.cpp',
    'experiments/native_policy_batch_encode/batch_encode_py.cpp',
)
_ACCEL_INPUT_HYPOTHESIS_SOURCES = (
    'singularitydog_hw/imu_accel_input_hypothesis.py',
    'singularitydog_hw/imu_calibration.py',
    'singularitydog_hw/imu_fixed_mount_baseline.py',
    'singularitydog_hw/imu.py',
    'singularitydog_hw/imu_capture.py',
    'singularitydog_hw/policy_observer.py',
    'singularitydog_hw/policy_output_model.py',
)


class ProfileError(ValueError):
    pass


def _need(condition, message):
    if not condition:
        raise ProfileError(message)


def _number(value, label, lower, upper, *, positive=False):
    _need(type(value) in (int, float), 'Invalid number: '+label)
    try:
        value = float(value)
    except (OverflowError, ValueError) as error:
        raise ProfileError('Invalid number: '+label) from error
    _need(math.isfinite(value) and lower <= value <= upper and (not positive or value > 0),
          'Out-of-scope limit: '+label)
    return value


def _text(value, label):
    _need(type(value) is str and 0 < len(value) <= 1024 and value.strip() == value,
          'Missing/invalid '+label)
    return value


def _hash(value, label):
    _need(type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value),
          'Invalid SHA256: '+label)
    return value


def _review(value, expected_decision):
    _need(type(value) is dict and set(value) == {'reviewer', 'reviewed_at', 'decision', 'rationale'},
          'Explicit named review is required')
    _text(value['reviewer'], 'reviewer'); _text(value['rationale'], 'review rationale')
    try:
        stamp = datetime.fromisoformat(value['reviewed_at'].replace('Z', '+00:00'))
        _need(stamp.utcoffset() is not None, 'Review time must include timezone')
    except (TypeError, ValueError, AttributeError) as error:
        raise ProfileError('Invalid review date') from error
    _need(value['decision'] == expected_decision, 'Review has not approved this scope')


def _read_json(path, *, digest=None):
    path = Path(path)
    _need(path.is_file() and not path.is_symlink(), 'Regular nonsymlink file required: '+str(path))
    _need(path.stat().st_size <= 16*1024*1024, 'Profile/evidence JSON is too large')
    raw = path.read_bytes()
    actual = hashlib.sha256(raw).hexdigest()
    _need(digest is None or actual == digest, 'Artifact SHA256 mismatch: '+path.name)
    try:
        return shadow._json(raw.decode('utf-8')), actual
    except (ValueError, UnicodeError) as error:
        raise ProfileError('Invalid JSON: '+path.name) from error


def _artifact(reference, base):
    _need(type(reference) is dict and set(reference) == {'path', 'sha256'}, 'Invalid artifact reference')
    name = _text(reference['path'], 'artifact path')
    digest = _hash(reference['sha256'], name)
    path = Path(name).expanduser()
    if not path.is_absolute():
        path = base/path
    data, _ = _read_json(path, digest=digest)
    return data, {'path': str(path.absolute()), 'sha256': digest}


def _profile_keys(data):
    _need(type(data) is dict and type(data.get('schema')) is str,
          'Unsupported profile schema')
    _need(data['schema'] in (SCHEMA_V1, SCHEMA_V2, SCHEMA_V3), 'Unsupported profile schema')
    keys = {SCHEMA_V1: TOP_KEYS_V1, SCHEMA_V2: TOP_KEYS, SCHEMA_V3: TOP_KEYS_V3}[data['schema']]
    return keys | (V3_EXECUTION_KEYS.intersection(data) if data['schema'] == SCHEMA_V3 else set())


def execution_settings(profile):
    """Explicit reviewed V3 choices; old profiles retain their original route."""
    _profile_keys(profile)
    acceleration_calibration_selected(profile)
    accel_input_hypothesis_selected(profile)
    if profile['schema'] != SCHEMA_V3:
        _need(not V3_EXECUTION_KEYS.intersection(profile), 'Fast execution requires a V3 profile')
    backend = profile.get('model_backend', 'native_baseline')
    _need(backend in ('native_baseline', SCALAR_BACKEND), 'Unsupported model backend')
    overlap = profile.get('voltage_overlap', False)
    _need(type(overlap) is bool, 'voltage_overlap must be an explicit boolean')
    pipeline = profile.get('voltage_pipeline', False)
    _need(type(pipeline) is bool, 'voltage_pipeline must be an explicit boolean')
    _need(not pipeline or profile['schema'] == SCHEMA_V3 and overlap,
          'Voltage pipeline requires V3 voltage overlap')
    _prepared_voltage_publication_scope(profile)
    timing = profile.get('diagnostic_timing_acceptance')
    _need(timing in (None, OBSERVED_R17_TIMING, MEASURED_R17_STARTUP_TIMING,
                    CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S,
                    HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S,
                    SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S,
                    SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                    SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
                    SUPPORTED_POLICY_GAIN_STEP_3S, SUPPORTED_PRELOAD_5S, SUPPORTED_POLICY_MIX_STEP_10PCT),
          'Unsupported diagnostic timing acceptance')
    _need(profile.get('watchdog_review_policy') in (None, COMMAND_LOSS_ONLY_SUPPORTED),
          'Unsupported watchdog review policy')
    _need(profile.get('local_characterization') in (None, LOCAL_RELATIVE_SUPPORTED),
          'Unsupported local characterization')
    return {'model_backend': backend, 'voltage_overlap': overlap,
            'voltage_pipeline': pipeline,
            'diagnostic_timing_acceptance': timing}


def _prepared_voltage_publication_selected(profile):
    selected = profile.get('prepare_voltage_before_feedback_publication', False)
    _need(type(selected) is bool and (not selected or profile.get('schema') == SCHEMA_V3),
          'Prepared voltage publication requires an explicit V3 boolean selection')
    return selected


def _prepared_voltage_publication_scope(profile):
    if not _prepared_voltage_publication_selected(profile):
        return
    mode = profile.get('diagnostic_timing_acceptance')
    duration = profile.get('duration_s')
    extension = mode == SUPPORTED_POLICY_PROBE_10S_AFTER_2S
    post_reply = _post_reply_policy(profile) if mode == SUPPORTED_POLICY_PROBE_20S_AFTER_10S else None
    extension_20s = (post_reply is not None and
                     post_reply['mode'] == 'bounded_post_reply_input_age_v2')
    _need(profile.get('scope') == 'supported_characterization_only' and
          profile.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
          profile.get('model_backend') == SCALAR_BACKEND and
          profile.get('voltage_overlap') is True and profile.get('voltage_pipeline') is True and
          mode in (SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
                   SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                   *((SUPPORTED_POLICY_PROBE_20S_AFTER_10S,) if extension_20s else ())) and
          type(duration) in (int, float) and
          (2. < duration <= 10. if extension else duration == 20. if extension_20s else duration == 2.) and
          type(profile.get('policy_weight')) in (int, float) and
          0 < profile['policy_weight'] <= .005 and
          profile.get('hard_cycle_ms') == 20 and
          type(profile.get('max_sample_age_ms')) in (int, float) and
          profile['max_sample_age_ms'] <= 20 and
          type(profile.get('max_sample_gap_ms')) in (int, float) and
          profile['max_sample_gap_ms'] <= 21 and
          profile.get('max_consecutive_20ms_misses') == 0 and
          not {'fixed_catch', 'human_supported_hold'}.intersection(profile),
          'Prepared voltage publication requires a boxed two-second probe or its evidenced ten/twenty-second extension')
    caps = {'kp': 3., 'kd': .15, 'max_displacement_from_start_rad': math.radians(1),
            'max_estimated_pd_torque_nm': .1, 'max_measured_torque_nm': 1.,
            'max_command_velocity_rad_s': math.radians(1),
            'max_command_acceleration_rad_s2': math.radians(5),
            'max_tracking_error_rad': math.radians(2), 'max_temperature_c': 45.,
            'max_measured_velocity_rad_s': .35 if mode in
                (SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                 SUPPORTED_POLICY_PROBE_20S_AFTER_10S) else .25}
    for mid in IDS:
        for key, cap in caps.items():
            _number(profile['axes'][mid][key], 'prepared voltage '+key+' ID'+mid,
                    0, cap, positive=True)


def _prepared_voltage_publication_binding(profile):
    value = {'settings': reviewed_settings_sha256(profile), 'axes': profile['axes'],
             'artifacts': profile['artifacts'], 'sources': profile['cadence_source_sha256'],
             'boot_id': profile['boot_id'], 'motor_power_epoch': profile['motor_power_epoch'],
             'review': profile['review'], 'blockers': profile['blockers'],
             'approved_for_supported_policy_output': profile['approved_for_supported_policy_output']}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def prepared_voltage_publication_settings(profile):
    """Expose only a complete, immutable reviewed selection; defaults stay inactive."""
    if not _prepared_voltage_publication_selected(profile):
        _need('_prepared_voltage_publication_token' not in profile,
              'Prepared voltage publication selection changed after loading')
        return False
    _prepared_voltage_publication_scope(profile)
    _need(profile.get('_prepared_voltage_publication_token') is _PREPARED_VOLTAGE_PUBLICATION_TOKEN and
          profile.get('output_allowed') is True and
          profile.get('_prepared_voltage_publication_binding') ==
          _prepared_voltage_publication_binding(profile),
          'Prepared voltage publication requires immutable complete loader proof')
    return True


def _prepared_voltage_publication_predecessor(report, prior):
    """Verify the selected production edge against original voltage journal times."""
    from . import can_readonly as codec
    proof = report.get('prepared_voltage_publication')
    _need(type(proof) is dict and
          proof.get('schema') == 'singularitydog.active-prepared-voltage-publication.v1' and
          proof.get('mode') == 'validate_feedback_prepare_voltage_publish_then_native' and
          proof.get('transport_capability') == 'singularitydog.active-prepared-exchange.v1' and
          proof.get('selection_bound_to_reviewed_profile') is True and
          proof.get('cadence_source_sha256') == prior['cadence_source_sha256'] and
          proof.get('changes_deadline_or_cancellation_guards') is False,
          'Prepared voltage extension lacks the selected production-edge proof')
    cycles, rows, journal = report.get('cycles'), proof.get('records'), report.get('journal')
    _need(type(cycles) is list and type(rows) is list and len(rows) == 2*len(cycles) and
          type(journal) is list, 'Prepared voltage predecessor must prove both owners on every cycle')
    buses = {'front': 1, 'rear': 7}
    voltages = {bus: [] for bus in buses}
    for item in journal:
        _need(type(item) is dict, 'Invalid prepared voltage predecessor journal')
        if item.get('phase') == 'overlapped_voltage':
            _need(item.get('bus') in buses, 'Prepared voltage journal bus mismatch')
            voltages[item['bus']].append(item)
    _need(all(len(items) == len(cycles) for items in voltages.values()),
          'Prepared voltage predecessor journal coverage differs')
    seen = set()
    for row in rows:
        _need(type(row) is dict and row.get('bus') in buses and
              type(row.get('bus_cycle_index')) is int and
              0 <= row['bus_cycle_index'] < len(cycles) and
              row.get('status') == 'VALIDATED' and row.get('error') is None,
              'Prepared voltage predecessor has a failed or invalid owner record')
        bus, index = row['bus'], row['bus_cycle_index']
        _need((bus, index) not in seen, 'Duplicate prepared voltage predecessor owner')
        seen.add((bus, index))
        cycle, item = cycles[index], voltages[bus][index]
        _need(type(cycle) is dict and type(cycle.get('begin_ns')) is int and
              type(cycle.get('end_ns')) is int and 0 < cycle['begin_ns'] <= cycle['end_ns'],
              'Prepared voltage predecessor cycle timestamps invalid')
        mid = buses[bus]+index%6
        names = ('feedback_validated_ns', 'prepared_before_publish_ns',
                 'publication_checked_after_ns', 'voltage_native_begin_ns',
                 'voltage_first_request_ns', 'voltage_validated_ns')
        stamps = [row.get(name) for name in names]
        effective, submitted = row.get('effective_deadline_ns'), row.get('submitted_deadline_ns')
        _need(row.get('voltage_motor_id') == mid and
              all(type(v) is int and 0 < v < 2**63 for v in stamps+[effective, submitted]) and
              stamps == sorted(stamps) and cycle['begin_ns'] <= stamps[0] and
              stamps[-1] < effective <= submitted <= cycle['begin_ns']+20_000_000 and
              stamps[-1] <= cycle['end_ns'],
              'Prepared voltage predecessor publication/deadline order differs')
        records, stats = item.get('records'), item.get('stats')
        _need(item.get('error') is None and type(records) is list and len(records) == 1 and
              type(stats) is dict and stats.get('begin_ns') == row['voltage_native_begin_ns'],
              'Prepared voltage predecessor lacks matching native voltage evidence')
        raw = records[0]
        _need(type(raw) is dict and raw.get('written') == raw.get('received') == 17 and
              raw.get('start_ns') == row['voltage_first_request_ns'] and
              raw.get('deadline_ns') == effective and
              all(type(raw.get(name)) is int for name in ('start_ns', 'finish_ns', 'received_ns')) and
              0 < raw['start_ns'] <= raw['finish_ns'] <= raw['received_ns'] < effective and
              raw['received_ns'] <= stamps[-1],
              'Prepared voltage predecessor raw request/deadline differs')
        try:
            tx, rx = bytes.fromhex(raw['tx_hex']), bytes.fromhex(raw['rx_hex'])
            frames = codec.ATParser().feed(rx)
            _need(len(tx) == len(rx) == 17 and tx == codec.read_request(mid, 'voltage') and
                  len(frames) == 1 and frames[0].wire == rx,
                  'Prepared voltage predecessor wire differs')
            reply = codec.decode_reply(frames[0], mid, 'voltage')
        except (ValueError, TypeError, KeyError) as error:
            raise ProfileError('Invalid prepared voltage predecessor wire') from error
        _need(reply.get('ok') is True and type(reply.get('value')) in (int, float) and
              math.isfinite(reply['value']) and prior['voltage_min_v'] <= reply['value'] <= prior['voltage_max_v'],
              'Prepared voltage predecessor voltage proof is out of scope')


def acceleration_calibration_selected(profile):
    selected = profile.get('apply_reviewed_accel_calibration', False)
    _need(type(selected) is bool and (not selected or profile.get('schema') == SCHEMA_V3),
          'Reviewed acceleration calibration requires an explicit V3 boolean selection')
    return selected


def accel_input_hypothesis_selected(profile):
    """Explicit experiment input; never a formal acceleration calibration."""
    selected = profile.get('accel_input_hypothesis', False)
    _need(type(selected) is bool and (not selected or profile.get('schema') == SCHEMA_V3),
          'Acceleration input hypothesis requires an explicit V3 boolean selection')
    _need(not selected or not acceleration_calibration_selected(profile),
          'Acceleration input hypothesis and reviewed calibration are mutually exclusive')
    return selected


def _accel_input_hypothesis_scope(profile):
    if not accel_input_hypothesis_selected(profile):
        return
    mode = profile.get('diagnostic_timing_acceptance')
    duration = profile.get('duration_s')
    extension = mode == SUPPORTED_POLICY_PROBE_10S_AFTER_2S
    post_reply = _post_reply_policy(profile) if mode == SUPPORTED_POLICY_PROBE_20S_AFTER_10S else None
    extension_20s = (post_reply is not None and
                     post_reply['mode'] == 'bounded_post_reply_input_age_v2')
    current_hold = mode == CURRENT_HOLD_AFTER_SUPPORTED_10S
    bounded_duration = (type(duration) in (int, float) and
                        (2. < duration <= 10. if extension else
                         duration == 20. if extension_20s else
                         duration == 3. if current_hold else duration == 2.))
    _need(profile.get('scope') == 'supported_characterization_only' and
          profile.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
          mode in (SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
                   SUPPORTED_POLICY_PROBE_10S_AFTER_2S, CURRENT_HOLD_AFTER_SUPPORTED_10S,
                   *((SUPPORTED_POLICY_PROBE_20S_AFTER_10S,) if extension_20s else ())) and
          profile.get('model_backend') == SCALAR_BACKEND and
          profile.get('voltage_overlap') is True and
          bounded_duration and
          type(profile.get('policy_weight')) in (int, float) and
          (profile['policy_weight'] == 0 if current_hold else 0 < profile['policy_weight'] <= .005) and
          not {'fixed_catch', 'human_supported_hold'}.intersection(profile),
          'Acceleration input hypothesis requires a boxed two-second probe, its proven ten/twenty-second extension, or the proven three-second current hold')
    # Keep learned output within its small-policy limits. The separately
    # evidenced zero-mixture hold permits only its exact Kp6/Kd0.15 comparison;
    # neither route inherits a wider envelope from another timing mode.
    caps = {'kp': 6. if current_hold else 3., 'kd': .15,
            'max_displacement_from_start_rad': math.radians(1),
            'max_estimated_pd_torque_nm': .2 if current_hold else .1, 'max_measured_torque_nm': 1.,
            'max_command_velocity_rad_s': math.radians(1),
            'max_command_acceleration_rad_s2': math.radians(5),
            'max_tracking_error_rad': math.radians(2), 'max_temperature_c': 45.,
            'max_measured_velocity_rad_s': .35 if mode in
                (SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                 SUPPORTED_POLICY_PROBE_20S_AFTER_10S) else .25}
    for mid in IDS:
        if current_hold:
            _need(profile['axes'][mid]['kp'] == 6. and profile['axes'][mid]['kd'] == .15,
                  'Acceleration hypothesis current hold requires exact Kp6/Kd0.15')
        for key, cap in caps.items():
            _number(profile['axes'][mid][key], 'acceleration hypothesis '+key+' ID'+mid,
                    0, cap, positive=True)


def _load_accel_input_hypothesis(reference, rotation):
    from .imu_accel_input_hypothesis import load_accel_input_hypothesis
    return load_accel_input_hypothesis(reference, rotation)


def _accel_input_hypothesis_binding(profile):
    value = {'settings': reviewed_settings_sha256(profile), 'axes': profile['axes'],
             'artifacts': profile['artifacts'], 'sources': profile['cadence_source_sha256'],
             'boot_id': profile['boot_id'], 'motor_power_epoch': profile['motor_power_epoch'],
             'review': profile['review'], 'blockers': profile['blockers'],
             'approved_for_supported_policy_output': profile['approved_for_supported_policy_output'],
             'provenance': profile['_accel_input_hypothesis_provenance']}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def accel_input_hypothesis_settings(profile):
    """Return its exact input ref only after complete supported-profile review.

    A raw JSON selection, unapproved PLAN or altered loaded mapping cannot
    authorize this scope. The caller re-loads the immutable input before use.
    """
    if not accel_input_hypothesis_selected(profile):
        _need('_accel_input_hypothesis_token' not in profile,
              'Acceleration input hypothesis selection changed after loading')
        return None
    _accel_input_hypothesis_scope(profile)
    _need(profile.get('_accel_input_hypothesis_token') is _ACCEL_INPUT_HYPOTHESIS_TOKEN and
          profile.get('output_allowed') is True and
          profile.get('_accel_input_hypothesis_binding') == _accel_input_hypothesis_binding(profile),
          'Acceleration input hypothesis requires immutable complete loader proof')
    return copy.deepcopy(profile['artifacts']['accel_input_hypothesis'])


def current_position_hold_only(profile):
    """The probe omits inference, never input validation or active deadlines."""
    selected = profile.get('diagnostic_timing_acceptance') in (
        CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S,
        HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S)
    if selected:
        _need(profile.get('_current_hold_token') is _CURRENT_HOLD_TOKEN and
              profile['policy_weight'] == 0, 'Current-position hold requires loader proof')
        if profile.get('diagnostic_timing_acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S:
            human_supported_partial_current_hold_settings(profile)
    return selected


def _mix_step_selected(profile):
    return profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_MIX_STEP_10PCT


def _mix_step_binding(profile):
    payload = {'settings': reviewed_settings_sha256(profile), 'artifacts': profile['artifacts'],
               'sources': profile['cadence_source_sha256'], 'axes': profile['axes']}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _supported_duration_cap(profile):
    if _mix_step_selected(profile):
        return 5
    if profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_20S_AFTER_10S:
        return 20
    if profile.get('diagnostic_timing_acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S:
        return 8
    if profile.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S:
        return 30
    if profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_10S_AFTER_2S:
        return 10
    if profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER:
        return 2
    if profile.get('diagnostic_timing_acceptance') == SUPPORTED_PRELOAD_5S:
        return 5
    if profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_GAIN_STEP_3S:
        return 3
    return 5 if profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_5S else 3


def _fixed_catch_settings(profile):
    selected = profile.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S
    if not selected:
        _need('fixed_catch' not in profile and profile.get('scope') != FIXED_CATCH_SCOPE,
              'Fixed catch requires its dedicated current-hold mode')
        return None
    value = profile.get('fixed_catch')
    expected = dict(mode='fixed-full-weight-catch-v1', fixed=True,
        full_weight_capacity_reviewed=True, immediate_power_cutoff_ready=True,
        upper_support_withdrawal_allowed=True, walking_allowed=False,
        catch_must_remain=True, unsupported_trial_allowed=False,
        recovery_basis='passive_fixed_catch_prearmed')
    _need(profile['schema'] == SCHEMA_V3 and profile['scope'] == FIXED_CATCH_SCOPE and
          type(value) is dict and set(value) == {*expected, 'catch_gap_mm'},
          'Fixed catch requires the dedicated V3 scope and complete catch settings')
    for key, required in expected.items():
        _need(type(value[key]) is type(required) and value[key] == required,
              'Fixed catch setting differs: '+key)
    _number(value['catch_gap_mm'], 'fixed catch gap mm', 2., 3.)
    return dict(value)


def fixed_catch_current_hold_settings(profile):
    """A frozen, reviewed finite hold; never ground or unsupported permission."""
    value = _fixed_catch_settings(profile)
    if value is not None:
        _need(profile.get('_fixed_catch_token') is _FIXED_CATCH_TOKEN,
              'Fixed catch current hold requires validated loader proof')
    return value


def _human_supported_settings(profile):
    selected = profile.get('diagnostic_timing_acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S
    if not selected:
        _need('human_supported_hold' not in profile and
              profile.get('scope') != HUMAN_SUPPORTED_PARTIAL_SCOPE,
              'Human-supported partial hold requires its dedicated current-hold mode')
        return None
    # The supervisor owns the same pure, exact finite contract. Importing it
    # opens no hardware and grants no execution permission.
    from .human_supported_hold import human_supported_hold_settings
    try:
        human_supported_hold_settings(profile)
        return dict(profile['human_supported_hold'])
    except (ValueError, TypeError, KeyError) as error:
        raise ProfileError('Invalid human-supported partial hold settings: '+str(error)) from error


def human_supported_partial_current_hold_settings(profile):
    """Loader proof for continuous human catch; never fixed-catch/ground permission."""
    value = _human_supported_settings(profile)
    if value is not None:
        _need(profile.get('_human_supported_token') is _HUMAN_SUPPORTED_TOKEN and
              profile.get('_human_supported_binding') == _human_supported_binding(profile),
              'Human-supported partial current hold requires validated loader proof')
    return value


def _human_supported_binding(data):
    payload = dict(settings=reviewed_settings_sha256(data), axes=data['axes'],
        artifacts=data['artifacts'], boot_id=data['boot_id'],
        motor_power_epoch=data['motor_power_epoch'], bundle_path=data['bundle_path'],
        audio=data.get('_human_supported_audio'))
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def human_supported_audio_settings(profile):
    """Return detached loader-bound spoken clips, without playing any audio."""
    if _human_supported_settings(profile) is None:
        return None
    human_supported_partial_current_hold_settings(profile)
    _need(type(profile.get('_human_supported_audio')) is dict,
          'Human-supported audio requires validated loader proof')
    return copy.deepcopy(profile['_human_supported_audio'])


def _approval_decision(profile):
    if profile.get('diagnostic_timing_acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S:
        return 'APPROVED_HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD'
    return ('APPROVED_FIXED_CATCH_CURRENT_HOLD' if
            profile.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S
            else 'APPROVED_SUPPORTED_CHARACTERIZATION')


def _startup_cycle_policy(profile):
    selected = profile.get('startup_cycle_allowance')
    if selected is None:
        _need('startup_cycle_allowance' not in profile, 'Omit inactive startup allowance')
        return None
    _need(selected == FIRST_CYCLE_POST_REPLY and profile['schema'] == SCHEMA_V3 and
          profile['scope'] == 'supported_characterization_only' and
          profile.get('diagnostic_timing_acceptance') in (
              SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
              SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
              SUPPORTED_POLICY_GAIN_STEP_3S, SUPPORTED_POLICY_MIX_STEP_10PCT) and
          profile.get('model_backend') == SCALAR_BACKEND and profile.get('voltage_overlap') is True and
          profile['hard_cycle_ms'] == 20 and profile['max_sample_age_ms'] <= 20 and
          profile['max_consecutive_20ms_misses'] == 0 and _post_reply_policy(profile) is not None,
          'First-cycle allowance requires reviewed supported post-reply timing with hard20ms')
    return selected


def reviewed_startup_cycle_allowance(profile):
    selected = _startup_cycle_policy(profile)
    if selected is not None:
        _need(profile.get('_startup_cycle_token') is _STARTUP_CYCLE_TOKEN,
              'First-cycle allowance requires loader review proof')
    return selected is not None


def native_batch_encoder_settings(profile):
    """Validate an optional reviewed binary selection without loading code."""
    selection = profile.get('native_batch_encoder')
    if selection is None:
        return None
    _need(profile['schema'] == SCHEMA_V3 and type(selection) is dict and
          set(selection) == {'path', 'sha256'},
          'Native batch encoder requires a V3 path and SHA256')
    name = _text(selection['path'], 'native batch encoder path')
    relative = Path(name)
    _need(not relative.is_absolute() and len(relative.parts) == 1 and
          relative.name == name and name not in ('.', '..'),
          'Native batch encoder path must be one bundle-relative file name')
    _hash(selection['sha256'], 'native batch encoder')
    return selection


def artifact_names(profile):
    return ARTIFACTS + (('scalar_step_manifest',)
                        if execution_settings(profile)['model_backend'] == SCALAR_BACKEND else ()) + (
        ('operator_acceptance', 'command_loss_report')
        if profile.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED else ()) + (
        ('local_reference_capture',) if profile.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED else ()) + (
        _EXTENSION_ARTIFACTS if profile.get('diagnostic_timing_acceptance') in (
            SUPPORTED_POLICY_PROBE_10S_AFTER_2S, CURRENT_HOLD_AFTER_SUPPORTED_10S,
            SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
            SUPPORTED_POLICY_GAIN_STEP_3S, SUPPORTED_POLICY_MIX_STEP_10PCT) else ()) + (
        _MIX_STEP_ARTIFACTS if _mix_step_selected(profile) else ()) + (
        _FIXED_CATCH_ARTIFACTS if profile.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S else ()) + (
        _HUMAN_SUPPORTED_ARTIFACTS if profile.get('diagnostic_timing_acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S else ()) + (
        _PRELOAD_ARTIFACTS if profile.get('diagnostic_timing_acceptance') == SUPPORTED_PRELOAD_5S else ()) + (
        ('accel_input_hypothesis',) if accel_input_hypothesis_selected(profile) else ())


def _post_reply_policy(profile):
    from .policy_post_reply_timing import (POST_REPLY_POLICY, POST_REPLY_POLICY_V2,
                                          POST_REPLY_INPUT_AGE_BUDGET_KEY)
    value = profile.get('post_reply_deadline_policy')
    if value is None:
        _need('post_reply_deadline_policy' not in profile, 'Omit inactive post-reply deadline policy')
        return None
    _need(profile['schema'] == SCHEMA_V3 and
          profile.get('scope') == 'supported_characterization_only',
          'Post-reply deadline policy requires supported-only V3')
    input_age_v2 = type(value) is dict and value.get('mode') == POST_REPLY_POLICY_V2
    keys = {'mode', 'max_lateness_ms', 'max_consecutive_misses',
            'rolling_window_cycles', 'max_misses_per_window'}
    if input_age_v2:
        keys.add(POST_REPLY_INPUT_AGE_BUDGET_KEY)
    _need(type(value) is dict and set(value) == keys and
          value.get('mode') in (POST_REPLY_POLICY, POST_REPLY_POLICY_V2),
          'Invalid post-reply deadline policy')
    _number(value['max_lateness_ms'], 'post-reply lateness', 0, 1, positive=True)
    for key, expected in (('max_consecutive_misses', 1), ('rolling_window_cycles', 100),
                          ('max_misses_per_window', 1)):
        _need(type(value[key]) is int and value[key] == expected, 'Invalid post-reply '+key)
    _need(profile['hard_cycle_ms'] == 20 and profile['max_sample_age_ms'] <= 20 and
          profile['max_sample_gap_ms'] <= 21 and profile['max_consecutive_20ms_misses'] == 0 and
          profile['duration_s'] <= _supported_duration_cap(profile),
          'Post-reply policy preserves hard20ms, freshness and finite supported scope')
    if input_age_v2:
        _number(value[POST_REPLY_INPUT_AGE_BUDGET_KEY], 'post-reply input-age budget',
                0, value['max_lateness_ms'], positive=True)
        mode = profile.get('diagnostic_timing_acceptance')
        duration = profile.get('duration_s')
        bounded_duration = (type(duration) in (int, float) and
                            (duration == 2. if mode == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER else
                             duration == 10. if mode == SUPPORTED_POLICY_PROBE_10S_AFTER_2S else
                             duration == 20. if mode == SUPPORTED_POLICY_PROBE_20S_AFTER_10S else False))
        _need(bounded_duration and
              profile.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              profile.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED and
              0 < profile['policy_weight'] <= .005 and
              profile.get('model_backend') == SCALAR_BACKEND and
              profile.get('voltage_overlap') is True and profile.get('voltage_pipeline') is True and
              profile['max_sample_age_ms'] == 20 and profile['command'] == [0., 0., 0.] and
              not {'fixed_catch', 'human_supported_hold'}.intersection(profile),
              'Post-reply input-age v2 requires a boxed local two-second probe or its proven ten/twenty-second chain')
        caps = {'kp': 3., 'kd': .15, 'max_displacement_from_start_rad': math.radians(1),
                'max_estimated_pd_torque_nm': .1, 'max_measured_torque_nm': 1.,
                'max_measured_velocity_rad_s': .35}
        for mid in IDS:
            for key, cap in caps.items():
                _number(profile['axes'][mid][key], 'post-reply input-age v2 '+key+' ID'+mid,
                        0, cap, positive=True)
    return dict(value)


def _post_reply_review_limits(acceptance, settings):
    """V2 must describe its changed post-reply age rather than claim unchanged freshness."""
    from .policy_post_reply_timing import POST_REPLY_POLICY_V2, POST_REPLY_INPUT_AGE_BUDGET_KEY
    if settings is None or settings['mode'] != POST_REPLY_POLICY_V2:
        return acceptance.get('hard_output_and_freshness_limits_unchanged') is True
    budget = acceptance.get(POST_REPLY_INPUT_AGE_BUDGET_KEY)
    return ('hard_output_and_freshness_limits_unchanged' not in acceptance and
            acceptance.get('pre_send_input_and_native_output_limits_unchanged') is True and
            acceptance.get('output_feedback_sample_age_limit_unchanged') is True and
            type(budget) in (int, float) and math.isfinite(budget) and
            budget == settings[POST_REPLY_INPUT_AGE_BUDGET_KEY])


def _post_reply_input_age_binding(profile):
    # Same immutable contract as the prepared-voltage token, including exact
    # reviewed settings, sources, input artifacts, power and review decisions.
    return _prepared_voltage_publication_binding(profile)


def post_reply_deadline_settings(profile):
    """Only a fully reviewed loader result may enable post-reply tolerance."""
    value = _post_reply_policy(profile)
    if value is not None:
        _need(profile.get('_post_reply_validation_token') is _POST_REPLY_VALIDATION_TOKEN,
              'Post-reply deadline policy requires validated loader proof')
    from .policy_post_reply_timing import POST_REPLY_POLICY_V2
    if (value is not None and value['mode'] == POST_REPLY_POLICY_V2 or
            '_post_reply_input_age_binding' in profile):
        _need(value is not None and profile.get('output_allowed') is True and
              profile.get('_post_reply_input_age_binding') == _post_reply_input_age_binding(profile),
              'Post-reply input-age v2 selection changed or lacks bound loader proof')
    return value


def local_characterization_settings(profile):
    """Return a loader-derived local mode proof; raw JSON cannot mint the token."""
    if profile.get('local_characterization') is None:
        return None
    _need(profile.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
          profile.get('_local_validation_token') is _LOCAL_VALIDATION_TOKEN,
          'Local characterization requires validated loader proof')
    mix_step = _mix_step_selected(profile)
    if mix_step:
        _need(profile.get('_mix_step_token') is _MIX_STEP_TOKEN and
              profile.get('_mix_step_binding') == _mix_step_binding(profile),
              'Ten-percent mix step requires its immutable loader evidence binding')
    return {'mode': LOCAL_RELATIVE_SUPPORTED, 'numerical_position_margin_rad': LOCAL_NUMERICAL_MARGIN_RAD,
            'absolute_zero_uncertainty_rad': None,
            'max_displacement_rad': math.radians(6 if mix_step else 1)}


def _supported_command_loss_acceptance(acceptance, report, data):
    """An explicit bounded experiment choice; never fabricate a USB test result."""
    fixed_catch = _fixed_catch_settings(data)
    human_supported = _human_supported_settings(data)
    _need(data['schema'] == SCHEMA_V3 and
          (data['scope'] == 'supported_characterization_only' or fixed_catch is not None or
           human_supported is not None),
          'Command-loss-only review is limited to supported V3 characterization')
    _need(type(acceptance) is dict and
          acceptance.get('schema') == ('singularitydog.human-supported-hold-operator-acceptance.v1'
                                      if human_supported else
                                      'singularitydog.fixed-catch-hold-operator-acceptance.v1' if fixed_catch
                                      else 'singularitydog.supported-trial-operator-acceptance.v1') and
          acceptance.get('scope') == data['scope'] and
          acceptance.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED,
          'Explicit supported command-loss-only operator acceptance required')
    _review(acceptance.get('review'), 'ACCEPT_COMMAND_LOSS_ONLY_HUMAN_SUPPORTED_PARTIAL_TRIAL'
            if human_supported else 'ACCEPT_COMMAND_LOSS_ONLY_FIXED_CATCH_HOLD' if fixed_catch
            else 'ACCEPT_COMMAND_LOSS_ONLY_SUPPORTED_TRIAL')
    _text(acceptance.get('user_statement'), 'explicit user instruction to omit USB test')
    if fixed_catch:
        _need(acceptance.get('fixed_catch_must_remain') is True and
              acceptance.get('fixed_catch_settings') == fixed_catch and
              acceptance.get('upper_box_must_remain') is False and 'box_must_remain' not in acceptance,
              'Fixed catch operator acceptance must preserve the fixed full-weight catch')
    if human_supported:
        _need(acceptance.get('human_supported_hold_settings') == human_supported and
              acceptance.get('continuous_body_catch_required') is True and
              acceptance.get('hands_remain_on_body_required') is True and
              acceptance.get('two_operators_required') is True and
              acceptance.get('fixed_catch_authorized') is False and
              not {'box_must_remain', 'fixed_catch_settings', 'upper_box_must_remain'}.intersection(acceptance),
              'Human-supported acceptance requires continuous two-person catch without fixed-catch permission')
    _need(acceptance.get('usb_disconnect_test_waived') is True and
          (fixed_catch is not None or human_supported is not None or
           acceptance.get('box_must_remain') is True) and
          acceptance.get('immediate_40v_cutoff_required') is True and
          acceptance.get('ground_progression_allowed') is False,
          'Supported-only acceptance must preserve box/cutoff and prohibit ground progression')
    _need(acceptance.get('reviewed_settings_sha256') == reviewed_settings_sha256(data) and
          acceptance.get('uids_by_id') == {mid: data['axes'][mid]['uid'] for mid in IDS},
          'Operator acceptance settings or UID binding differs')
    _need(acceptance.get('artifact_sha256') == {
        k: data['artifacts'][k]['sha256'] for k in artifact_names(data)
        if k not in ('hardware_review', 'operator_acceptance')},
        'Operator acceptance must pin the exact diagnostic and command-loss report')
    _need(type(report) is dict and report.get('status') == 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC' and
          report.get('errors') == [] and report.get('selected_ids') == list(range(1,13)) and
          report.get('configured_timeout_ms') == 200 and report.get('watchdog_ticks') == 4000 and
          report.get('stop_confirmed') is True and report.get('positive_gain_sent') is False and
          report.get('learned_targets_sent') is False and report.get('usb_disconnect_tested') is False,
          'Complete zero-gain twelve-axis command-loss evidence required')
    _need(report.get('boot_id') == data['boot_id'] and
          report.get('motor_power_epoch') == data['motor_power_epoch'],
          'Command-loss evidence must match the current boot and motor-power epoch')
    _need(report.get('voltage_max_v', 42.) == data['voltage_max_v'],
          'Command-loss voltage envelope differs from reviewed profile')
    _need(report.get('voltage_range_v', [35., 42.]) == [35., data['voltage_max_v']],
          'Command-loss voltage range differs from reviewed profile')
    _need(type(report.get('axes')) is dict and set(report['axes']) == set(IDS),
          'Twelve command-loss axis records required')
    for scope, ids in (('front', list(range(1,7))), ('rear', list(range(7,13)))):
        stop = report.get('stop_reports', {}).get(scope, {})
        _need(stop.get('complete') is True and stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
              stop.get('errors') == [], 'Command-loss report has incomplete or ambiguous STOP evidence')
    for mid in IDS:
        row = report['axes'][mid]
        _number(row.get('voltage_v'), 'command-loss voltage ID'+mid,
                data['voltage_min_v'], data['voltage_max_v'])
        _need(type(row) is dict and row.get('uid') == data['axes'][mid]['uid'] and
              row.get('command_loss_tested') is True and row.get('disabled_on_command_loss') is True and
              row.get('usb_disconnect_tested') is False and row.get('configured_timeout_ms') == 200,
              'Per-UID command-loss evidence incomplete: ID'+mid)
        _number(row.get('disable_reply_upper_bound_ms'), 'command-loss disable bound ID'+mid, 0, 250, positive=True)
        _need(row.get('disable_upper_bound_origin') == 'last_zero_host_write_started_ns',
              'Command-loss disable bound must start before the final zero-command write: ID'+mid)
        probe = row.get('stop_probe', {})
        _need(probe.get('mode_state') == 0 and probe.get('fault_bits') == 0 and
              row.get('watchdog_readback', {}).get('value') == 4000,
              'Command-loss disabled/timeout readback invalid: ID'+mid)


def transport_settings(profile, *, request_gap_us=None, request_window=None):
    """Resolve reviewed pacing; v1 always retains its original 600us/window3."""
    _profile_keys(profile)
    if profile['schema'] == SCHEMA_V1:
        _need(not TRANSPORT_KEYS.intersection(profile), 'Unsupported v1 profile fields')
        gap, window = 600, 3
    else:
        gap, window = profile.get('request_gap_us'), profile.get('request_window')
    _need(type(gap) is int and 600 <= gap <= 5000, 'Invalid request_gap_us: integer 600..5000 required')
    _need(type(window) is int and 1 <= window <= 3, 'Invalid request_window: integer 1..3 required')
    for key, requested, reviewed in (('request_gap_us', request_gap_us, gap),
                                     ('request_window', request_window, window)):
        _need(requested is None or (type(requested) is int and requested == reviewed),
              key+' differs from reviewed profile')
    return {'request_gap_us': gap, 'request_window': window,
            'source_profile_schema': profile['schema'], 'emergency_stop_uses_same_gap': True}


def cadence_source_paths(profile=None):
    """Keep historical manifests stable; preload explicitly pins its extra runtime."""
    extra = (_PRELOAD_SOURCE_PATH,) if profile is not None and profile.get(
        'diagnostic_timing_acceptance') == SUPPORTED_PRELOAD_5S else ()
    if profile is not None and profile.get('diagnostic_timing_acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S:
        extra = (_HUMAN_SUPPORTED_NEW_SOURCE,)
    if profile is not None and profile.get('accel_input_hypothesis') is True:
        extra += _ACCEL_INPUT_HYPOTHESIS_SOURCES
    return tuple(dict.fromkeys(CADENCE_SOURCE_PATHS+extra))


def cadence_source_hashes(profile=None):
    """Read-only identity of cadence-related source files; not all kit dependencies."""
    root = Path(__file__).resolve().parents[1]
    values = {}
    for name in cadence_source_paths(profile):
        path = root/name
        _need(path.is_file() and not path.is_symlink(), 'Missing cadence source: '+name)
        values[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return values


def telemetry_settings(profile):
    """Resolve only an explicit cadence; V1/V2 retain rotating timeout monitoring."""
    _profile_keys(profile)
    new = profile['schema'] == SCHEMA_V3
    if new:
        _need(profile.get('telemetry_cadence') == CADENCE_PRE_ENABLE, 'Unsupported telemetry cadence')
        sources = profile.get('cadence_source_sha256')
        _need(type(sources) is dict and set(sources) == set(cadence_source_paths(profile)),
              'Complete cadence source pins required')
        for name, value in sources.items():
            _hash(value, 'cadence source '+name)
    else:
        _need(not CADENCE_KEYS.intersection(profile), 'Legacy profile cannot select a new cadence')
    return {'schema': 'singularitydog.policy-telemetry-cadence.v1',
        'source_profile_schema': profile['schema'],
        'cadence': CADENCE_PRE_ENABLE if new else 'feedback_rotating_voltage_and_timeout.v1',
        'feedback_requests_per_bus_per_cycle': 6,
        'voltage_requests_per_bus_per_cycle': 1,
        'voltage_rotation_length_cycles': 6,
        'timeout_requests_per_bus_per_cycle': 0 if new else 1,
        'total_requests_per_cycle_including_output': 26 if new else 28,
        'initial_all_axis_timeout_write_and_readback': True,
        'after_announcement_all_axis_timeout_readback': new,
        'timeout_parameter_drift_monitored_during_cycles': not new,
        'all_axis_feedback_monitored_on_acquisition_and_output': True,
        'stop_reply_confirmation_required': True,
        'post_stop_timeout_parameter_readback': False}


def validate_cadence_sources(profile):
    """Fail before hardware setup if a V3 cadence pin differs from this frozen kit."""
    telemetry_settings(profile)
    if profile['schema'] == SCHEMA_V3:
        _need(profile['cadence_source_sha256'] == cadence_source_hashes(profile),
              'Cadence source SHA256 mismatch; freeze and review a new profile')


def add_transport_arguments(parser, *, reviewed=True):
    """Bound CLI pacing; output overrides only confirm the selected profile."""
    def gap_us(value):
        try:
            number = int(value)
        except ValueError as error:
            raise argparse.ArgumentTypeError('request-gap-us must be an integer 600..5000') from error
        if not 600 <= number <= 5000:
            raise argparse.ArgumentTypeError('request-gap-us must be an integer 600..5000')
        return number
    context = 'must match the reviewed profile' if reviewed else 'for the unapproved candidate profile'
    parser.add_argument('--request-gap-us', type=gap_us, help='600..5000 us; '+context)
    parser.add_argument('--request-window', type=int, choices=(1, 2, 3), help='1..3; '+context)


def template(*, schema=SCHEMA):
    """No invented calibration, gains, identities or evidence in the template."""
    _profile_keys({'schema': schema})
    data = {'schema': schema, 'scope': 'supported_characterization_only',
        'approved_for_supported_policy_output': False,
        'blockers': ['12-axis zero/sign/physical range review including ID10',
                     'fixed-mount IMU direction/bias/gravity review',
                     'full diagnostic acquire/infer/12-STOP timing evidence',
                     'Type2 dynamic position/velocity/torque interpretation',
                     'each QDD actual communication-loss watchdog test'],
        'review': None, 'boot_id': None, 'motor_power_epoch': None, 'assembly_id': None,
        'axes': {mid: {key: None for key in sorted(AXIS_KEYS)} for mid in IDS},
        'artifacts': {key: {'path': None, 'sha256': None} for key in ARTIFACTS},
        'bundle_path': None, 'duration_s': 5., 'startup_duration_s': 1., 'stop_duration_s': 1.,
        'policy_ramp_s': 1., 'policy_weight': .1, 'h_hypothesis': 0., 'command': [0., 0., 0.],
        'period_ms': 20, 'hard_cycle_ms': 20., 'max_consecutive_20ms_misses': 0,
        'max_sample_age_ms': 20., 'max_sample_gap_ms': 21.,
        'voltage_min_v': 35., 'voltage_max_v': 42., 'imu_tilt_limit_rad': .2,
        'imu_gyro_limit_rad_s': .5, 'imu_accel_norm_min_m_s2': 9.4,
        'imu_accel_norm_max_m_s2': 10.2, 'start_pose_bounds': None}
    if schema in (SCHEMA_V2, SCHEMA_V3):
        data.update(request_gap_us=600, request_window=3)
    if schema == SCHEMA_V3:
        data.update(telemetry_cadence=CADENCE_PRE_ENABLE, cadence_source_sha256=cadence_source_hashes())
        data['blockers'].append('Review absence of cyclic timeout-parameter drift polling and pinned cadence source')
    return data


def reviewed_settings_sha256(profile):
    """Bind review to numerical settings; file relocation and fresh epoch are separate."""
    # Keep the exact original v1 digest input; derived legacy defaults are not
    # new reviewed fields. v2 binds its two explicit pacing settings.
    transport_settings(profile)
    telemetry_settings(profile)
    keys = _profile_keys(profile)-{'artifacts', 'review', 'blockers', 'approved_for_supported_policy_output',
                     'boot_id', 'motor_power_epoch', 'bundle_path'}
    settings = {key: copy.deepcopy(profile[key]) for key in keys}
    # load_profile adds effective limits, while the file stores physical limits.
    settings['axes'] = {mid: {key: row[key] for key in AXIS_KEYS}
                        for mid, row in settings['axes'].items()}
    return hashlib.sha256(json.dumps(settings, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _structure(data):
    keys = _profile_keys(data)
    _need(set(data) == keys, 'Unsupported profile fields')
    human_supported = _human_supported_settings(data)
    _need(data['scope'] == 'supported_characterization_only' or human_supported is not None or
          _fixed_catch_settings(data) is not None,
          'Only reviewed supported or fixed-catch characterization is supported')
    _need(type(data['approved_for_supported_policy_output']) is bool, 'Explicit approval boolean required')
    _need(type(data['blockers']) is list and all(type(v) is str and v for v in data['blockers']),
          'Invalid blockers')
    _need(type(data['axes']) is dict and set(data['axes']) == set(IDS), 'Exactly twelve CAN axes required')
    for row in data['axes'].values():
        _need(type(row) is dict and set(row) == AXIS_KEYS, 'Unsupported axis fields')
    _need(type(data['artifacts']) is dict and set(data['artifacts']) == set(artifact_names(data)),
          'Complete pinned artifact references required for the selected backend')


def _settings(data):
    transport_settings(data)
    validate_cadence_sources(data)
    execution_settings(data)
    _accel_input_hypothesis_scope(data)
    native_batch_encoder_settings(data)
    _startup_cycle_policy(data)
    _need(type(data['period_ms']) is int and data['period_ms'] == 20, 'Target period is exactly20ms')
    hard = _number(data['hard_cycle_ms'], 'hard_cycle_ms', 20, 60)
    _need(type(data['max_consecutive_20ms_misses']) is int and
          0 <= data['max_consecutive_20ms_misses'] <= 3, 'Invalid20ms miss budget')
    _number(data['max_sample_age_ms'], 'max_sample_age_ms', 1, hard)
    # Inter-sample scheduling jitter is separate from sample age and execution
    # deadlines. This1ms margin never changes either20ms computation criterion.
    _number(data['max_sample_gap_ms'], 'max_sample_gap_ms', 1, hard+1)
    fixed_catch = _fixed_catch_settings(data)
    human_supported = _human_supported_settings(data)
    if human_supported:
        _need(data['schema'] == SCHEMA_V3 and
              data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              data.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED and
              data.get('model_backend') == SCALAR_BACKEND and data.get('voltage_overlap') is True and
              not {'startup_damping_duration_s', 'startup_cycle_allowance',
                   'post_reply_deadline_policy', 'fixed_catch'}.intersection(data),
              'Human-supported partial hold requires separate local V3 proof and strict live deadlines')
    mix_step = _mix_step_selected(data)
    if mix_step:
        _need(data['schema'] == SCHEMA_V3 and data['scope'] == 'supported_characterization_only' and
              data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              data.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED and
              data.get('model_backend') == SCALAR_BACKEND and data.get('voltage_overlap') is True and
              data['policy_weight'] == .1 and
              data['duration_s'] == 5. and data['startup_duration_s'] == 1. and
              data['policy_ramp_s'] == 2. and data['stop_duration_s'] == .4 and
              hard == 20 and data['max_sample_age_ms'] == 20 and data['max_sample_gap_ms'] == 21 and
              data['max_consecutive_20ms_misses'] == 0 and
              not {'startup_damping_duration_s', 'fixed_catch', 'human_supported_hold'}.intersection(data),
              'Ten-percent mix step requires its exact five-second boxed contract and hard20ms')
    extension_20s = data.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_20S_AFTER_10S
    duration = _number(data['duration_s'], 'duration_s', .5, 30 if fixed_catch else 20 if extension_20s else 10)
    if extension_20s:
        _need(data['schema'] == SCHEMA_V3 and data['scope'] == 'supported_characterization_only' and
              data.get('model_backend') == SCALAR_BACKEND and data.get('voltage_overlap') is True and
              duration == 20.,
              'Twenty-second extension requires its exact supported V3 scalar/overlap mode and duration')
    if data.get('diagnostic_timing_acceptance') in (CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S):
        _need(data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              data['policy_weight'] == 0 and duration <= 3 and
              data['startup_duration_s'] >= 1 and hard == 20 and
              data['max_sample_age_ms'] <= 20 and data['max_sample_gap_ms'] <= 21 and
              data['max_consecutive_20ms_misses'] == 0,
              'Current-position probe requires zero mixture, >=1s ramp, <=3s and hard20ms')
    if data.get('diagnostic_timing_acceptance') == CURRENT_HOLD_AFTER_SUPPORTED_10S:
        _need(not {'startup_damping_duration_s', 'startup_cycle_allowance',
                   'post_reply_deadline_policy'}.intersection(data),
              'Current hold after supported run requires the full gain ramp and strict live deadline')
    if fixed_catch:
        _need(data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              data['policy_weight'] == 0 and duration == 30 and
              data['startup_duration_s'] >= 1 and hard == 20 and
              data['max_sample_age_ms'] <= 20 and data['max_sample_gap_ms'] <= 21 and
              data['max_consecutive_20ms_misses'] == 0 and
              not {'startup_damping_duration_s', 'startup_cycle_allowance',
                   'post_reply_deadline_policy'}.intersection(data),
              'Thirty-second fixed catch requires strict zero-mixture current hold')
    if data.get('diagnostic_timing_acceptance') == SUPPORTED_PRELOAD_5S:
        _need(data['schema'] == SCHEMA_V3 and data['scope'] == 'supported_characterization_only' and
              data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              data.get('model_backend') == SCALAR_BACKEND and data.get('voltage_overlap') is True and
              data['policy_weight'] == 0 and duration == 5 and data['startup_duration_s'] == 1 and
              .2 <= data['stop_duration_s'] <= .4 and hard == 20 and
              data['max_sample_age_ms'] <= 20 and data['max_sample_gap_ms'] <= 21 and
              data['max_consecutive_20ms_misses'] == 0 and
              not {'startup_damping_duration_s', 'startup_cycle_allowance',
                   'post_reply_deadline_policy', 'fixed_catch'}.intersection(data),
              'Preload requires supported five-second zero-mixture path and strict live deadlines')
    policy_probe_5s = data.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_5S
    rare_jitter_probe = data.get('diagnostic_timing_acceptance') in (
        SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
        SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
        SUPPORTED_POLICY_GAIN_STEP_3S)
    if data.get('diagnostic_timing_acceptance') in (
            SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            SUPPORTED_POLICY_PROBE_10S_AFTER_2S, SUPPORTED_POLICY_PROBE_20S_AFTER_10S):
        _need(data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              0 < data['policy_weight'] <= .005 and duration <= (
                  20 if extension_20s else
                  10 if data.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_10S_AFTER_2S
                  else 5 if policy_probe_5s else 2) and
              data['startup_duration_s'] >= .4 and hard == 20 and
              data['max_sample_age_ms'] <= 20 and data['max_sample_gap_ms'] <= 21 and
              data['max_consecutive_20ms_misses'] == 0,
              'Supported policy probe requires <=0.5percent mixture, >=0.4s ramp, bounded duration and hard20ms')
    if 'startup_damping_duration_s' in data:
        _need(policy_probe_5s or rare_jitter_probe,
              'Independent damping ramp requires the five-second or rare-jitter supported probe')
        _number(data['startup_damping_duration_s'], 'startup damping duration', .08,
                data['startup_duration_s'])
    if data.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_GAIN_STEP_3S:
        _need(data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              duration == 3 and data['startup_duration_s'] >= 1. and
              0 < data['policy_weight'] <= .01 and hard == 20 and
              data['max_sample_age_ms'] <= 20 and data['max_sample_gap_ms'] <= 21 and
              data['max_consecutive_20ms_misses'] == 0,
              'Gain-step probe requires a three-second supported run and slow gain ramp')
    if execution_settings(data)['voltage_pipeline']:
        _need(hard == 20 and data['max_sample_age_ms'] <= 20 and
              data['max_sample_gap_ms'] <= 21 and duration <= _supported_duration_cap(data),
              'Voltage pipeline preserves hard20ms, freshness and short supported scope')
    _post_reply_policy(data)
    start = _number(data['startup_duration_s'], 'startup_duration_s', .2, 2)
    stop = _number(data['stop_duration_s'], 'stop_duration_s', .2, 2)
    ramp = _number(data['policy_ramp_s'], 'policy_ramp_s', .2, 5)
    _need(start+ramp+stop <= duration, 'Run duration must include startup, policy ramp and stopping')
    _number(data['policy_weight'], 'policy_weight', 0, 1)
    _need(type(data['h_hypothesis']) in (int, float) and data['h_hypothesis'] in (0., 1.),
          'Explicit h=0 or h=1 hypothesis required')
    _need(type(data['command']) is list and len(data['command']) == 3 and
          all(type(v) in (int, float) and v == 0 for v in data['command']), 'Only zero locomotion command allowed')
    lo = _number(data['voltage_min_v'], 'voltage_min_v', 35, 42)
    # Full-charge RS05 characterization can explicitly select 43V telemetry.
    # The default stays42V; matching diagnostic/watchdog evidence is required.
    hi = _number(data['voltage_max_v'], 'voltage_max_v', 35,
                 43 if data['schema'] == SCHEMA_V3 else 42)
    _need(lo < hi, 'Invalid voltage interval')
    _number(data['imu_tilt_limit_rad'], 'imu_tilt_limit_rad', .01, .35)
    _number(data['imu_gyro_limit_rad_s'], 'imu_gyro_limit_rad_s', .01, 1.)
    lo = _number(data['imu_accel_norm_min_m_s2'], 'imu_accel_norm_min_m_s2', 8.8, 11.2)
    hi = _number(data['imu_accel_norm_max_m_s2'], 'imu_accel_norm_max_m_s2', 8.8, 11.2)
    _need(lo < hi and hi-lo <= 1.5, 'Invalid diagnostic gravity-norm interval')
    if data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED:
        _need(data['schema'] == SCHEMA_V3 and
              data.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED,
              'Local characterization requires explicit supported V3 review')
        _need(duration <= _supported_duration_cap(data) and data['policy_weight'] <= (.1 if mix_step else .01) and hard == 20 and
              data['max_consecutive_20ms_misses'] == 0,
              'Local characterization requires bounded duration, <=1percent mix and hard20ms')


def _axes(data, calibration):
    rows = shadow.validate_calibration(calibration)
    _need(calibration.get('approved_for_runtime') is False,
          'Keep original candidate provenance unchanged; use the separate hardware review')
    # A reviewed all-axis zero-gain comparison exercises the actual Type1
    # transport without applying learned targets or PD gains. Preserve the
    # existing positive-gain contract for every other profile and keep all
    # monitoring limits strictly positive. Native gain caps also become zero.
    zero_gain_comparison = data['policy_weight'] == 0 and all(
        data['axes'][mid][key] == 0 for mid in IDS for key in ('kp', 'kd'))
    # With exactly zero policy mixture the runtime anchors every target to the
    # initial measured position. Allow a separately reviewed gain comparison
    # there only; learned motion retains the original Kp3 / 0.1Nm ceilings.
    preload = data.get('diagnostic_timing_acceptance') == SUPPORTED_PRELOAD_5S
    current_position_hold = data['policy_weight'] == 0 and not preload
    for mid in IDS:
        row = data['axes'][mid]
        candidate = rows[int(mid)]
        _need(row['uid'] == calibration['identities'][mid], 'Calibrated UID mismatch: ID'+mid)
        _need(type(row['sign']) is int and row['sign'] in (-1, 1) and
              row['sign'] == candidate['sign_candidate'], 'Calibrated sign mismatch: ID'+mid)
        offset = _number(row['offset_rad'], 'offset_rad ID'+mid, -30, 30)
        _need(offset == candidate['offset_candidate_rad'], 'Calibrated offset mismatch: ID'+mid)
        local = data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED
        if local:
            _need(row['uncertainty_rad'] is None, 'Local absolute-zero uncertainty must remain unknown')
            uncertainty = LOCAL_NUMERICAL_MARGIN_RAD
        else:
            uncertainty = _number(row['uncertainty_rad'], 'uncertainty_rad ID'+mid, 1e-6, .0873)
        lower = _number(row['physical_lower_rad'], 'physical lower ID'+mid, -math.pi, math.pi)
        upper = _number(row['physical_upper_rad'], 'physical upper ID'+mid, -math.pi, math.pi)
        index = shadow.CAN_ORDER.index(int(mid))
        _need(shadow.LOWER[index] <= lower < upper <= shadow.UPPER[index],
              'Physical range exceeds model range: ID'+mid)
        _need(lower+uncertainty < upper-uncertainty, 'Uncertainty consumes joint range: ID'+mid)
        row['lower_rad'], row['upper_rad'] = lower+uncertainty, upper-uncertainty
        for key, cap in LIMIT_CAPS.items():
            _number(row[key], key+' ID'+mid, 0, cap,
                    positive=not (zero_gain_comparison and key in ('kp', 'kd')))
        if local:
            if current_position_hold and (row['kp'] > 3. or row['max_estimated_pd_torque_nm'] > .1):
                _need(data['startup_duration_s'] >= 1.,
                      'Higher-gain current-position hold requires at least1s gain ramp')
            gain_step = data.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_GAIN_STEP_3S
            mix_step = _mix_step_selected(data)
            for key, cap in {'kp':6. if preload else 12. if current_position_hold or gain_step else 3., 'kd':.15,
                    'max_command_velocity_rad_s':.12 if mix_step else math.radians(1),
                    'max_command_acceleration_rad_s2':.5 if mix_step else math.radians(5),
                    'max_tracking_error_rad':math.radians(3 if mix_step else 2),
                    # Retain the already bounded five-second probe's monitor
                    # for its shorter rare-jitter admission; gains/motion stay fixed.
                    'max_measured_velocity_rad_s':(.35 if data.get('diagnostic_timing_acceptance') in
                        (SUPPORTED_POLICY_PROBE_5S, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
                         SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                         SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
                         SUPPORTED_POLICY_GAIN_STEP_3S, SUPPORTED_POLICY_MIX_STEP_10PCT) else .25),
                    'max_measured_torque_nm':1.,
                    'max_estimated_pd_torque_nm':.2 if preload else .25 if mix_step else .5 if current_position_hold or gain_step else .1,
                    'max_temperature_c':45., 'max_displacement_from_start_rad':math.radians(6 if mix_step else 1)}.items():
                _need(row[key] <= cap, 'Local characterization limit exceeded: '+key+' ID'+mid)
        _need(row['max_estimated_pd_torque_nm'] <= row['max_measured_torque_nm'],
              'Estimated PD budget must not exceed hard measured torque monitor')
        _need(row['max_command_velocity_rad_s'] <= row['max_measured_velocity_rad_s'],
              'Command velocity exceeds measured velocity monitor')
    bounds = data['start_pose_bounds']
    brake_s = max(row['max_command_velocity_rad_s']/row['max_command_acceleration_rad_s2']
                  for row in data['axes'].values())
    _need(data['startup_duration_s']+data['policy_ramp_s']+data['stop_duration_s']+brake_s+.04 < data['duration_s'],
          'Duration does not reserve worst-case acceleration-limited braking')
    if bounds is not None:
        _need(type(bounds) is dict and set(bounds) == set(IDS), 'Start-pose bounds need all12axes')
        for mid, value in bounds.items():
            _need(type(value) is list and len(value) == 2, 'Start-pose interval required')
            lo = _number(value[0], 'start lower', -math.pi, math.pi)
            hi = _number(value[1], 'start upper', -math.pi, math.pi)
            _need(data['axes'][mid]['lower_rad'] <= lo < hi <= data['axes'][mid]['upper_rad'],
                  'Start-pose interval exceeds effective physical range')


def _timing(report, data):
    """Recompute evidence metrics from timestamps; STOP proxy is never active I/O."""
    if type(report) is dict:
        diagnostic_plan = report.get('plan')
        _need('sourced_boot_guard' not in report and
              (type(diagnostic_plan) is not dict or 'sourced_boot_guard' not in diagnostic_plan),
              'Experimental boot-guard diagnostic cannot qualify the unchanged active controller')
        _need('trace_copy_provenance' not in report and
              (type(diagnostic_plan) is not dict or 'trace_copy_provenance' not in diagnostic_plan),
              'Experimental trace-copy diagnostic cannot qualify the unchanged active controller')
    _need(type(report) is dict and report.get('status') == 'COMPLETE_DIAGNOSTIC' and
          report.get('mode') == 'stop-proxy' and report.get('errors') == [] and
          report.get('motor_enable_sent') is False and report.get('learned_targets_sent') is False and
          report.get('full_controller_50Hz_verified') is False and
          type(report.get('observer')) is dict, 'Full real-input/inference/STOP diagnostic required')
    prepared_publication = _prepared_voltage_publication_selected(data)
    for source in (report, report.get('plan', {})):
        _need(type(source) is dict, 'Diagnostic pacing differs from reviewed profile: invalid prepared voltage plan')
        value = source.get('prepare_voltage_before_feedback_publication', False)
        _need(type(value) is bool and value is prepared_publication,
              'Diagnostic prepared voltage publication selection differs from reviewed profile')
    if prepared_publication:
        provenance = report.get('source_provenance')
        _need(report.get('boot_id') == data['boot_id'] and
              report.get('motor_power_epoch') == data['motor_power_epoch'] and
              report.get('cadence_source_sha256') == data['cadence_source_sha256'] and
              type(provenance) is dict and provenance.get('source_files_unchanged') is True and
              provenance.get('cadence_source_sha256') == data['cadence_source_sha256'] and
              provenance.get('motor_power_epoch') == data['motor_power_epoch'],
              'Prepared voltage diagnostic requires exact current sources, boot and power epoch')
    if data['schema'] in (SCHEMA_V2, SCHEMA_V3):
        settings = transport_settings(data)
        plan = report.get('plan')
        _need(type(plan) is dict and type(plan.get('request_gap_us')) is int and
              type(plan.get('window')) is int and
              plan['request_gap_us'] == settings['request_gap_us'] and
              plan['window'] == settings['request_window'],
              'Diagnostic pacing differs from reviewed profile')
    _need(report.get('plan', {}).get('voltage_max_v', 42.) == data['voltage_max_v'],
          'Diagnostic voltage envelope differs from reviewed profile')
    for source in (report.get('plan', {}), report):
        _need(source.get('voltage_max_v', 42.) == data['voltage_max_v'],
              'Diagnostic voltage envelope differs from reviewed profile')
        _need(source.get('voltage_range_v', [35., 42.]) == [35., data['voltage_max_v']],
              'Diagnostic voltage range differs from reviewed profile')
    bindings = report.get('input_sha256', {})
    _need(report.get('plan', {}).get('apply_reviewed_accel_calibration', False)
          is acceleration_calibration_selected(data),
          'Diagnostic acceleration calibration selection differs from reviewed profile')
    hypothesis = accel_input_hypothesis_selected(data)
    reference = data['artifacts']['accel_input_hypothesis'] if hypothesis else None
    _need(report.get('plan', {}).get('accel_input_hypothesis') == reference and
          bindings.get('accel_input_hypothesis') == (reference['sha256'] if hypothesis else None),
          'Diagnostic acceleration hypothesis input differs from reviewed profile')
    provenance = data.get('_accel_input_hypothesis_provenance') if hypothesis else None
    observed = report.get('observer', {}).get('accel_input_hypothesis')
    _need((not hypothesis or type(provenance) is dict) and
          json.dumps(observed, sort_keys=True, separators=(',', ':'), allow_nan=False) ==
          json.dumps(provenance, sort_keys=True, separators=(',', ':'), allow_nan=False),
          'Diagnostic observer acceleration hypothesis provenance differs')
    if hypothesis:
        _need(report.get('motor_power_epoch') == data['motor_power_epoch'] and
              report.get('cadence_source_sha256') == data['cadence_source_sha256'],
              'Acceleration hypothesis diagnostic must bind current power epoch and exact sources')
    for source, key in (('calibration', 'calibration'), ('mount', 'mount'), ('bias', 'gyro_bias')):
        value = bindings.get(key, bindings.get('bias') if source == 'bias' else None)
        _need(value == data['artifacts'][source]['sha256'], 'Timing input mismatch: '+source)
        if source == 'bias' and 'bias' in bindings:
            _need(bindings['bias'] == value, 'Conflicting timing gyro-bias pins')
    execution = execution_settings(data)
    if 'voltage_overlap' in data:
        _need(report.get('plan', {}).get('v3_voltage_overlap',False) is execution['voltage_overlap'],
              'Diagnostic voltage overlap differs from reviewed profile')
        if execution['voltage_overlap']:
            _need(report.get('plan', {}).get('v3_voltage_validation_overlap') is True,
                  'Diagnostic worker voltage validation overlap was not measured')
    if execution['voltage_pipeline']:
        _need(report.get('plan', {}).get('v3_voltage_fast_pipeline') is True and
              report.get('plan', {}).get('v3_voltage_pipeline') is not True,
              'Immediate feedback-then-voltage pipeline requires its own disabled diagnostic')
    model_key = 'scalar_step_manifest' if execution['model_backend'] == SCALAR_BACKEND else 'model_manifest'
    _need(report.get('model_source', {}).get('manifest_sha256') == data['artifacts'][model_key]['sha256'],
          'Timing model-manifest mismatch')
    if model_key == 'scalar_step_manifest':
        _need(report.get('model_source', {}).get('baseline_provenance', {}).get('manifest_sha256') ==
              data['artifacts']['model_manifest']['sha256'], 'Scalar timing baseline differs')
    mix_step = _mix_step_selected(data)
    observed_r17 = execution['diagnostic_timing_acceptance'] == OBSERVED_R17_TIMING
    hold_after_supported = execution['diagnostic_timing_acceptance'] == CURRENT_HOLD_AFTER_SUPPORTED_10S
    fixed_catch_hold = execution['diagnostic_timing_acceptance'] == FIXED_CATCH_CURRENT_HOLD_30S
    human_supported_hold = execution['diagnostic_timing_acceptance'] == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S
    hold_probe = execution['diagnostic_timing_acceptance'] in (
        CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S,
        HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S)
    rare_jitter_probe = execution['diagnostic_timing_acceptance'] in (
        SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
        SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
        SUPPORTED_POLICY_GAIN_STEP_3S, CURRENT_HOLD_AFTER_SUPPORTED_10S,
        FIXED_CATCH_CURRENT_HOLD_30S, SUPPORTED_POLICY_MIX_STEP_10PCT)
    policy_probe = execution['diagnostic_timing_acceptance'] in (
        SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
        SUPPORTED_POLICY_PROBE_10S_AFTER_2S, SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
        SUPPORTED_POLICY_GAIN_STEP_3S, SUPPORTED_POLICY_MIX_STEP_10PCT)
    bounded_probe = hold_probe or policy_probe
    measured_r17 = execution['diagnostic_timing_acceptance'] in (
        MEASURED_R17_STARTUP_TIMING, CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S,
        FIXED_CATCH_CURRENT_HOLD_30S, HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S,
        SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S,
        SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
        SUPPORTED_POLICY_PROBE_20S_AFTER_10S,
        SUPPORTED_POLICY_GAIN_STEP_3S, SUPPORTED_PRELOAD_5S, SUPPORTED_POLICY_MIX_STEP_10PCT)
    accepted_r17 = observed_r17 or measured_r17
    if observed_r17:
        _need(data['artifacts']['pipeline_diagnostic']['sha256'] in OBSERVED_R17_REPORT_SHA256,
              'Observed cadence acceptance is limited to the two original R17 reports')
    if accepted_r17:
        _need(report.get('plan', {}).get('startup_cycle_allowance') == 1 and
              report.get('cycles_requested') == 501, 'R17 startup evidence differs')
    if measured_r17:
        plan = report['plan']
        schedule = report.get('absolute_epoch_schedule')
        _need(execution['model_backend'] == SCALAR_BACKEND and execution['voltage_overlap'] is True and
              data['hard_cycle_ms'] == 20 and data['max_sample_age_ms'] <= 20 and
              data['max_sample_gap_ms'] <= 21 and
              report.get('boot_id') == data['boot_id'] and
              report.get('approved_for_runtime') is False and
              report.get('imu_restore_status') in ('restored', 'not_needed') and
              type(plan.get('startup_cycle_allowance')) is int and
              type(plan.get('steady_cycles_requested')) is int and
              plan['steady_cycles_requested'] == 500 and
              plan.get('absolute_epoch_cadence') is True and
              type(schedule) is dict and schedule.get('enabled') is True and
              type(schedule.get('epoch_ns')) is int and schedule['epoch_ns'] > 0 and
              type(schedule.get('period_ns')) is int and schedule['period_ns'] == 20_000_000,
              'Fresh R17 requires a bound 1+500 disabled absolute-epoch diagnostic')
    if execution['diagnostic_timing_acceptance'] in (SUPPORTED_PRELOAD_5S, HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S, SUPPORTED_POLICY_MIX_STEP_10PCT):
        _need(report.get('motor_power_epoch') == data['motor_power_epoch'] and
              report.get('cadence_source_sha256') == data['cadence_source_sha256'],
              'Preload diagnostic must bind current power epoch and exact execution sources')
    rows = report.get('measurements')
    _need(type(rows) is list and 20 <= len(rows) <= 100000 and
          report.get('cycles_completed') == len(rows) == report.get('cycles_requested'),
          'At least20complete diagnostic cycles required')
    if execution['voltage_pipeline']:
        _voltage_fast_pipeline_trace(report, data, rows)
    observation = report['observer']
    _need(observation.get('status') == 'COMPLETE_NO_OUTPUT_DIAGNOSTIC' and
          observation.get('failure') is None and observation.get('incomplete') is False and
          observation.get('ticks_completed') == len(rows) == observation.get('ticks_requested') and
          observation.get('h_hypothesis') == data['h_hypothesis'] and
          observation.get('output_allowed') is False,
          'Model observer did not complete the exact reviewed hypothesis and tick count')
    maximum, misses, consecutive, longest, previous, previous_end = 0., 0, 0, 0, None, None
    late_intervals = 0
    names = ('release_ns', 'oldest_input_start_ns', 'input_latest_reply_ns', 'gather_end_ns',
             'prepare_end_ns', 'infer_end_ns', 'final_host_write_ns', 'last_proxy_reply_ns', 'cycle_end_ns')
    startup_elapsed = None
    probe_miss_indices = []
    for index, row in enumerate(rows):
        _need(type(row) is dict, 'Invalid timing row')
        stamps = [row.get(k) for k in names]
        _need(all(type(x) is int and 0 < x < 2**63 for x in stamps) and stamps == sorted(stamps),
              'Missing/noncausal full-pipeline timestamps')
        release, oldest, latest, gather, prepared, inferred, sent, replied, end = stamps
        _need(row.get('learned_targets_sent') is False and
              row.get('host_write_is_can_wire_completion') is False and inferred > prepared,
              'Diagnostic scope or real inference timestamps invalid')
        _need(previous_end is None or release >= previous_end, 'Overlapping diagnostic cycles')
        elapsed = (end-release)/1e6
        if human_supported_hold:
            _need(elapsed <= 20 and end-oldest <= 20_000_000 and
                  replied-release <= 20_000_000,
                  'Human-supported diagnostic must meet strict whole-cycle/reply/age20ms including startup')
        maximum = max(maximum, elapsed)
        startup = accepted_r17 and index == 0
        if accepted_r17:
            _need(row.get('timing_phase') == ('startup' if startup else 'steady'),
                  'R17 startup classification differs')
        if startup: startup_elapsed = elapsed
        if measured_r17:
            scheduled = schedule['epoch_ns'] + index*20_000_000
            _need(type(row.get('cadence_slot')) is int and row['cadence_slot'] == index and
                  type(row.get('scheduled_release_ns')) is int and
                  row['scheduled_release_ns'] == scheduled and release >= scheduled and
                  type(row.get('skipped_slots_before')) is int and
                  row['skipped_slots_before'] == 0 and
                  (index == 0 or 15_000_000 <= release-previous <= 21_000_000),
                  'Fresh R17 schedule skipped a slot or exceeded the release-gap budget')
            if bounded_probe:
                # Admission to a short supported commissioning probe is not
                # a successful20ms certificate. Both live paths retain the
                # hard20ms output/freshness guards. Only hold omits inference.
                _need(elapsed <= 21 and replied-oldest <= 21_000_000,
                      'Supported probe diagnostic exceeds21ms bound')
                if rare_jitter_probe:
                    _need(replied-oldest <= 20_000_000 and end-oldest <= 20_000_000 and
                          (startup or replied-release <= 20_000_000),
                          'Rare-jitter diagnostic exceeds hard reply or checked sample-age20ms')
            else:
                _need((elapsed <= 21 if startup else end <= scheduled+20_000_000) and
                      (replied-oldest) <= 20_000_000,
                      'Fresh R17 exceeds startup, scheduled, or STOP-reply deadline')
        _need((startup or elapsed <= (21 if bounded_probe else data['hard_cycle_ms'])) and (inferred-oldest)/1e6 <= data['max_sample_age_ms'],
              'Diagnostic exceeds the reviewed cycle/freshness budget')
        # Wakeup jitter has a distinct1ms tolerance; it is not included in the
        # actual20ms computation deadline. Never hide it by rounding timestamps.
        interval = (release-previous)/1e6 if previous is not None else None
        _need(accepted_r17 or interval is None or interval <= data['hard_cycle_ms']+1,
              'Diagnostic scheduling gap exceeds reviewed budget')
        late_intervals += int(interval is not None and interval > 21)
        missed = not startup and (elapsed > 20 or (sent-oldest)/1e6 > 20 or
            bounded_probe and not human_supported_hold and
            (end > scheduled+20_000_000 or replied-oldest > 20_000_000))
        if accepted_r17 and not startup and not bounded_probe:
            _need(elapsed <= 20 and (replied-oldest)/1e6 <= 20,
                  'Observed cadence acceptance does not waive steady20ms processing/reply limits')
        misses += int(missed); consecutive = consecutive+1 if missed else 0
        longest = max(longest, consecutive)
        if bounded_probe:
            if missed: probe_miss_indices.append(index)
            if rare_jitter_probe:
                # This is admission to a finite supported experiment, not a
                # change to the live one-miss-per100 post-reply STOP budget.
                # Count scheduled misses too, including startup carry-over.
                _need(longest <= 1 and sum(i > index-100 for i in probe_miss_indices) <= 2 and
                      misses <= 5,
                      'Rare-jitter diagnostic exceeds five per500, two per100 or isolated-miss budget')
            else:
                _need(longest <= 1 and sum(i > index-100 for i in probe_miss_indices) <= 1,
                      'Supported probe diagnostic exceeds one miss per100 cycles')
        else:
            _need(longest <= data['max_consecutive_20ms_misses'], 'Diagnostic exceeds20ms consecutive-miss budget')
        previous, previous_end = release, end
    return {'kind': ('supported_policy_mix_step_10pct_5s_admission_only' if mix_step else
                     'supported_preload_5s_diagnostic_admission_only'
                     if execution['diagnostic_timing_acceptance'] == SUPPORTED_PRELOAD_5S else
                     'human_supported_partial_current_hold_admission_only' if human_supported_hold else
                     'fixed_catch_current_hold_30s_admission_only' if fixed_catch_hold else
                     'current_hold_after_supported_10s_admission_only' if hold_after_supported else
                     'supported_policy_10s_after_2s_admission_only'
                     if execution['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_10S_AFTER_2S else
                     'supported_policy_20s_after_10s_admission_only'
                     if execution['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_20S_AFTER_10S else
                     'supported_policy_gain_step_3s_admission_only'
                     if execution['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_GAIN_STEP_3S else
                     'supported_policy_2s_rare_jitter_admission_only' if rare_jitter_probe else
                     'current_position_probe_admission_only' if hold_probe else
                     'supported_policy_probe_admission_only' if policy_probe else
                     'stop_proxy_diagnostic_only'), 'cycles': len(rows),
            'max_whole_iteration_ms': maximum, 'twenty_ms_misses': misses,
            'longest_consecutive_twenty_ms_misses': longest,
            'release_intervals_over_21ms': late_intervals,
            'diagnostic_timing_acceptance': execution['diagnostic_timing_acceptance'],
            'startup_whole_iteration_ms': startup_elapsed,
            'strict_start_interval_20ms_met': all(
                (b['release_ns']-a['release_ns']) <= 20_000_000 for a, b in zip(rows, rows[1:])),
            'actual_policy_output_20ms_verified': False}


def _voltage_pipeline_trace(report, data, measurements):
    """Recheck the separately hashed trace; a plan flag alone is no evidence."""
    from . import can_readonly as codec
    from . import rs05_trial_protocol as protocol
    proof = report.get('v3_voltage_pipeline')
    _need(type(proof) is dict and proof.get('enabled') is True and
          proof.get('schema') == 'feedback-then-voltage-proxy-v1' and
          proof.get('period_ns') == 20_000_000 and
          proof.get('feedback_gate_before_voltage') is True and
          proof.get('voltage_verified_before_proxy_stop') is True and
          proof.get('diagnostic_only') is True and
          proof.get('motor_output_allowed') is False and
          proof.get('learned_targets_sent') is False and
          proof.get('active_feedback_safety_equivalent') is False,
          'Voltage pipeline requires exact disabled trace provenance')
    overlap = report.get('v3_voltage_overlap')
    _need(type(overlap) is dict and overlap.get('enabled') is True and
          overlap.get('validation_overlap_enabled') is True and
          overlap.get('voltage_dispatch_schedule') == 'after_complete_feedback_imu_snapshot',
          'Voltage pipeline timing differs from feedback-gated schedule')
    digest = _hash(proof.get('records_sha256'), 'voltage pipeline records')
    records_path = Path(data['artifacts']['pipeline_diagnostic']['path']).with_name('records.json')
    records, _ = _read_json(records_path, digest=digest)
    _need(type(records) is list and len(records) == len(measurements),
          'Voltage pipeline trace count differs from timing rows')
    buses = {'front': set(range(1, 7)), 'rear': set(range(7, 13))}
    for index, (record, timing) in enumerate(zip(records, measurements)):
        _need(type(record) is dict and record.get('cycle') == index+1 and
              record.get('voltage_overlap', {}).get('status') == 'VALIDATED_BEFORE_PROXY_STOP',
              'Voltage pipeline trace is incomplete or out of order')
        row = record.get('voltage_pipeline')
        _need(type(row) is dict and row.get('status') == 'VALIDATED_BEFORE_PROXY_STOP' and
              row.get('output_allowed') is False and row.get('range_v') == [35., data['voltage_max_v']],
              'Voltage pipeline cycle lacks validated no-output proof')
        bus_names = ('feedback_dispatch_ns_by_bus', 'feedback_reply_end_ns_by_bus',
                     'feedback_ready_ns_by_bus', 'voltage_dispatch_ns_by_bus',
                     'voltage_reply_end_ns_by_bus')
        _need(all(type(row.get(name)) is dict and set(row[name]) == set(buses)
                  for name in bus_names), 'Voltage pipeline lacks both bus timestamp sets')
        join, gate, inferred, voltage_join, verified, deadline = (row.get(name) for name in
            ('feedback_join_ns', 'voltage_gate_set_ns', 'inference_end_ns',
             'voltage_join_ns', 'voltage_verified_ns', 'hard_deadline_ns'))
        _need(all(type(value) is int and 0 < value < 2**63 for value in
                  (join, gate, inferred, voltage_join, verified, deadline)) and
              join == timing['gather_end_ns'] and inferred == timing['infer_end_ns'] and
              join <= gate <= inferred <= verified <= timing['final_host_write_ns'] and
              voltage_join <= verified < deadline and
              deadline == min(timing['release_ns'], timing['oldest_input_start_ns'])+20_000_000,
              'Voltage pipeline coordinator timestamps are noncausal')
        for bus, expected_ids in buses.items():
            timestamps = tuple(row[name][bus] for name in bus_names)
            feedback_sent, feedback_replied, feedback_ready, voltage_sent, voltage_replied = timestamps
            _need(all(type(value) is int and 0 < value < 2**63 for value in timestamps) and
                  timing['release_ns'] <= feedback_sent <= feedback_replied <= feedback_ready <= join <=
                  gate <= voltage_sent <= voltage_replied <= voltage_join,
                  'Voltage pipeline feedback/gate/voltage order is invalid')
            _need(type(record.get('acquired')) is dict and type(record.get('voltage')) is dict and
                  type(record.get('output')) is dict and
                  set(record['acquired']) == set(buses) and
                  set(record['voltage']) == set(buses) and
                  set(record['output']) == set(buses),
                  'Voltage pipeline requires two-bus feedback, voltage and STOP')
            acquired = record['acquired'][bus].get('records')
            volts = record['voltage'][bus].get('records')
            output = record['output'][bus].get('records')
            _need(all(type(part) is list for part in (acquired, volts, output)) and
                  (len(acquired), len(volts), len(output)) == (6, 1, 6) and
                  min(part['start_ns'] for part in output) >= verified,
                  'Voltage pipeline frame counts or STOP ordering differ')
            ordered_ids = tuple(sorted(expected_ids))
            expected_stops = [protocol.stop_request(phase=protocol.TrialPhase.STOP,
                                                    motor_id=mid) for mid in ordered_ids]
            expected_voltage = codec.read_request(ordered_ids[index % 6], 'voltage')
            try:
                actual_acquired = [bytes.fromhex(part['tx_hex']) for part in acquired]
                actual_voltage = bytes.fromhex(volts[0]['tx_hex'])
                actual_output = [bytes.fromhex(part['tx_hex']) for part in output]
            except (ValueError, TypeError, KeyError) as error:
                raise ProfileError('Invalid voltage-pipeline trace request frame') from error
            _need(actual_acquired == expected_stops and actual_voltage == expected_voltage and
                  actual_output == expected_stops,
                  'Voltage pipeline requires exact feedback, voltage and STOP requests')


def _voltage_fast_pipeline_trace(report, data, measurements):
    """Bind the immediate, read-only voltage overlap to complete STOP evidence.

    Voltage can start after its own bus's six feedback replies, before the
    coordinator has checked both buses and IMU. This diagnostic never sends a
    learned target. The active runtime must complete those checks and both
    voltage checks before it may send Type1.
    """
    from . import can_readonly as codec
    from . import rs05_trial_protocol as protocol
    proof = report.get('v3_voltage_fast_pipeline')
    _need(type(proof) is dict and proof.get('enabled') is True and
          proof.get('schema') == 'immediate-feedback-voltage-proxy-v1' and
          proof.get('period_ns') == 20_000_000 and
          proof.get('voltage_dispatch_schedule') == 'after_each_bus_feedback' and
          proof.get('voltage_may_precede_global_feedback_validation') is True and
          proof.get('voltage_verified_before_proxy_stop') is True and
          proof.get('diagnostic_only') is True and
          proof.get('motor_output_allowed') is False and
          proof.get('learned_targets_sent') is False and
          proof.get('active_feedback_safety_equivalent') is False,
          'Fast voltage pipeline requires exact disabled trace provenance')
    overlap = report.get('v3_voltage_overlap')
    _need(type(overlap) is dict and overlap.get('enabled') is True and
          overlap.get('validation_overlap_enabled') is True and
          overlap.get('voltage_dispatch_schedule') == 'after_each_bus_feedback',
          'Fast voltage pipeline timing differs from immediate schedule')
    digest = _hash(proof.get('records_sha256'), 'fast voltage pipeline records')
    records_path = Path(data['artifacts']['pipeline_diagnostic']['path']).with_name('records.json')
    records, _ = _read_json(records_path, digest=digest)
    _need(type(records) is list and len(records) == len(measurements),
          'Fast voltage pipeline trace count differs from timing rows')
    buses = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
    for index, (record, timing) in enumerate(zip(records, measurements)):
        _need(type(record) is dict and record.get('cycle') == index+1 and
              type(record.get('voltage_overlap')) is dict and
              record['voltage_overlap'].get('status') == 'VALIDATED_BEFORE_PROXY_STOP',
              'Fast voltage pipeline trace is incomplete or out of order')
        row = record.get('voltage_fast_pipeline')
        _need(type(row) is dict and row.get('status') == 'VALIDATED_BEFORE_PROXY_STOP' and
              row.get('output_allowed') is False and row.get('range_v') == [35., data['voltage_max_v']] and
              row.get('stop_reply_count') == 12,
              'Fast voltage pipeline cycle lacks validated no-output proof')
        bus_names = ('feedback_dispatch_ns_by_bus', 'feedback_reply_end_ns_by_bus',
                     'feedback_ready_ns_by_bus', 'voltage_dispatch_ns_by_bus',
                     'voltage_reply_end_ns_by_bus', 'stop_reply_end_ns_by_bus')
        _need(all(type(row.get(name)) is dict and set(row[name]) == set(buses)
                  for name in bus_names), 'Fast voltage pipeline lacks both bus timestamp sets')
        join, snapshot_ok, inferred, voltage_join, post_verified, verified, deadline, stop_ok = (
            row.get(name) for name in ('feedback_join_ns', 'feedback_snapshot_validated_ns',
            'inference_end_ns', 'voltage_join_ns', 'post_inference_verified_ns',
            'voltage_verified_ns', 'hard_deadline_ns', 'stop_reply_verified_ns'))
        _need(all(type(value) is int and 0 < value < 2**63 for value in
                  (join, snapshot_ok, inferred, voltage_join, post_verified,
                   verified, deadline, stop_ok)) and
              join == timing['gather_end_ns'] and inferred == timing['infer_end_ns'] and
              join <= snapshot_ok <= inferred <= post_verified <= verified < deadline and
              voltage_join <= post_verified and verified <= timing['final_host_write_ns'] and
              stop_ok <= timing['cycle_end_ns'] and
              deadline == min(timing['release_ns'], timing['oldest_input_start_ns'])+20_000_000,
              'Fast voltage pipeline coordinator timestamps are noncausal')
        for bus, ids in buses.items():
            feedback_sent, feedback_replied, feedback_ready, voltage_sent, voltage_replied, stop_replied = (
                row[name][bus] for name in bus_names)
            _need(all(type(value) is int and 0 < value < 2**63 for value in
                      (feedback_sent, feedback_replied, feedback_ready,
                       voltage_sent, voltage_replied, stop_replied)) and
                  timing['release_ns'] <= feedback_sent <= feedback_replied <= feedback_ready <=
                  voltage_sent <= voltage_replied <= voltage_join and
                  feedback_ready <= join and
                  timing['final_host_write_ns'] <= stop_replied <= stop_ok,
                  'Fast voltage pipeline feedback/voltage/STOP order is invalid')
            _need(type(record.get('acquired')) is dict and type(record.get('voltage')) is dict and
                  type(record.get('output')) is dict and
                  set(record['acquired']) == set(buses) and
                  set(record['voltage']) == set(buses) and
                  set(record['output']) == set(buses),
                  'Fast voltage pipeline requires two-bus feedback, voltage and STOP')
            acquired = record['acquired'][bus].get('records')
            volts = record['voltage'][bus].get('records')
            output = record['output'][bus].get('records')
            _need(all(type(part) is list for part in (acquired, volts, output)) and
                  (len(acquired), len(volts), len(output)) == (6, 1, 6) and
                  min(part['start_ns'] for part in output) >= verified and
                  max(part['received_ns'] for part in output) == stop_replied,
                  'Fast voltage pipeline frame counts or STOP ordering differ')
            expected_stops = [protocol.stop_request(phase=protocol.TrialPhase.STOP,
                                                    motor_id=mid) for mid in ids]
            try:
                actual_acquired = [bytes.fromhex(part['tx_hex']) for part in acquired]
                actual_voltage = bytes.fromhex(volts[0]['tx_hex'])
                actual_output = [bytes.fromhex(part['tx_hex']) for part in output]
                reply_frames = [codec.ATParser().feed(bytes.fromhex(part['rx_hex']))
                                for part in output]
            except (ValueError, TypeError, KeyError) as error:
                raise ProfileError('Invalid fast voltage pipeline trace frame') from error
            _need(actual_acquired == expected_stops and
                  actual_voltage == codec.read_request(ids[index % 6], 'voltage') and
                  actual_output == expected_stops and
                  all(len(frames) == 1 and frames[0].flags == 4 and
                      frames[0].can_id == ((2 << 24) | (mid << 8) | 0xfd)
                      for mid, frames in zip(ids, reply_frames)),
                  'Fast voltage pipeline requires exact feedback, voltage and STOP replies')


def _local_reference(review, capture, data):
    """Bind a local relative envelope to a current, non-driving twelve-axis read."""
    local = review.get('local_characterization', {})
    clearance = math.radians(7 if _mix_step_selected(data) else 3)
    _need(type(local) is dict and local.get('schema') == 'singularitydog.local-relative-review.v1' and
          local.get('operator_confirmed_local_clearance') is True and
          local.get('local_clearance_rad') == clearance and
          'absolute_zero_uncertainty_rad' in local and local['absolute_zero_uncertainty_rad'] is None and
          local.get('absolute_calibration_not_certified') is True and
          local.get('full_dynamic_feedback_not_certified') is True,
          'Explicit relative local clearance review with unknown absolute calibration required')
    _need(type(capture) is dict and capture.get('schema') == 'singularitydog.readonly-12-angle-capture.v1' and
          capture.get('status') == 'RECORDED_REVIEW_REQUIRED' and capture.get('errors') == [] and
          capture.get('motor_output_allowed') is False and capture.get('boot_id') == data['boot_id'],
          'Current-boot read-only local reference capture required')
    turns = local.get('reference_turns_by_id')
    _need(type(turns) is dict and set(turns) == set(IDS), 'Local reference branches need all twelve IDs')
    identities, rows = capture.get('identities', {}), capture.get('telemetry', {}).get('rows', {})
    _need(set(identities) == set(rows) == set(IDS), 'Local reference must contain twelve identities and angles')
    for mid in IDS:
        a, row = data['axes'][mid], rows[mid]
        _need(identities[mid].get('mcu_uid_hex') == a['uid'] and row.get('run_mode') == 0,
              'Local reference UID/mode mismatch: ID'+mid)
        samples = row.get('position_samples')
        _need(type(samples) is list and len(samples) == 3, 'Local reference requires three position samples')
        values = [_number(s.get('rad'), 'reference raw ID'+mid, -1000, 1000) for s in samples]
        _need(max(values)-min(values) <= math.radians(.1) and
              row.get('median_position_rad') == statistics.median(values),
              'Local reference position median/span invalid: ID'+mid)
        _need(type(turns[mid]) is int and -20 <= turns[mid] <= 20, 'Invalid local branch: ID'+mid)
        q = a['sign']*(statistics.median(values)-turns[mid]*2*math.pi)+a['offset_rad']
        index = shadow.CAN_ORDER.index(int(mid))
        lower, upper = max(shadow.LOWER[index], q-clearance), min(shadow.UPPER[index], q+clearance)
        _need(abs(a['physical_lower_rad']-lower) <= 1e-12 and
              abs(a['physical_upper_rad']-upper) <= 1e-12 and
              lower+LOCAL_NUMERICAL_MARGIN_RAD < q < upper-LOCAL_NUMERICAL_MARGIN_RAD,
              'Local bounds differ from measured reference and model range: ID'+mid)


def _hardware(review, data, base, *, command_loss_report=None, local_reference_capture=None):
    _need(type(review) is dict and review.get('schema') == REVIEW_SCHEMA and
          review.get('scope') == data['scope'], 'Explicit supported hardware-review artifact required')
    _review(review.get('review'), _approval_decision(data))
    _need(review.get('reviewed_settings_sha256') == reviewed_settings_sha256(data),
          'Hardware review does not bind exact gains, limits and policy settings')
    _need(review.get('assembly_id') == data['assembly_id'] and
          review.get('uids_by_id') == {mid: data['axes'][mid]['uid'] for mid in IDS},
          'Hardware review assembly/UID mismatch')
    _need(review.get('artifact_sha256') == {k: data['artifacts'][k]['sha256'] for k in artifact_names(data) if k != 'hardware_review'},
          'Hardware review is not bound to all exact input artifacts')
    sources = review.get('source_captures')
    _need(type(sources) is list and bool(sources), 'Underlying hardware capture files required')
    for source in sources:
        _artifact(source, base)
    angles = review.get('angles')
    _need(type(angles) is dict and set(angles) == set(IDS), 'Every axis needs physical angle review')
    local_mode = data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED
    if local_mode:
        _local_reference(review, local_reference_capture, data)
    for mid, angle in angles.items():
        row = data['axes'][mid]
        keys = (('zero_reference_recorded', 'sign_evidence_reviewed', 'relative_local_clearance_verified',
                 'power_cycle_branch_method_verified') if local_mode else
                ('zero_and_sign_physically_verified', 'physical_range_and_clearance_verified',
                 'power_cycle_branch_method_verified'))
        _need(type(angle) is dict and all(angle.get(k) is True for k in keys),
              'Unresolved physical calibration: ID'+mid)
        _need(angle.get('sign') == row['sign'] and angle.get('offset_rad') == row['offset_rad'] and
              angle.get('physical_lower_rad') == row['physical_lower_rad'] and
              angle.get('physical_upper_rad') == row['physical_upper_rad'] and
              angle.get('uncertainty_rad') == row['uncertainty_rad'], 'Angle review values differ: ID'+mid)
    imu = review.get('imu', {})
    gravity_key = 'gravity_direction_compared_to_operator_level' if local_mode else 'gravity_direction_verified'
    _need(type(imu) is dict and all(imu.get(k) is True for k in
          ('right_handed_mount_physically_verified', 'nose_up_verified', 'left_up_verified',
           'yaw_left_verified', 'gyro_bias_independent_stationary_validation', gravity_key)),
          'IMU direction/bias/gravity review incomplete')
    if local_mode:
        _need('absolute_gravity_error_bound_rad' in imu and imu['absolute_gravity_error_bound_rad'] is None,
              'Local IMU comparison must preserve unknown absolute gravity accuracy')
    _number(imu.get('gravity_direction_max_error_rad'), 'IMU gravity direction error', 0, math.radians(3))
    _number(imu.get('corrected_static_gyro_max_rad_s'), 'IMU held-out gyro residual', 0, .02)
    normlo = _number(imu.get('raw_gravity_norm_min_m_s2'), 'review gravity norm low', 8.8, 11.2)
    normhi = _number(imu.get('raw_gravity_norm_max_m_s2'), 'review gravity norm high', 8.8, 11.2)
    _need(data['imu_accel_norm_min_m_s2'] <= normlo <= normhi <= data['imu_accel_norm_max_m_s2'],
          'Observed static gravity norm outside run bounds')
    _text(imu.get('norm_deviation_rationale'), 'gravity norm disposition (single-pose bias fitting prohibited)')
    feedback = review.get('type2_dynamic')
    _need(type(feedback) is dict and set(feedback) == set(IDS), 'Twelve-axis dynamic Type2 review required')
    for mid, row in feedback.items():
        dynamic_keys = ('output_shaft_position_verified', 'velocity_scale_and_sign_verified', 'torque_interpretation_verified')
        if local_mode:
            _need(type(row) is dict and row.get('limited_trial_reviewed') is True and
                  all(type(row.get(k)) is bool for k in dynamic_keys),
                  'Local Type2 characterization requires explicit pending/verified state: ID'+mid)
        else:
            _need(type(row) is dict and all(row.get(k) is True for k in dynamic_keys),
                  'Static-only Type2 comparison cannot validate dynamic feedback: ID'+mid)
        _need(row.get('position_range_rad') == [-12.57, 12.57] and
              row.get('velocity_range_rad_s') == [-50., 50.] and
              row.get('torque_range_nm') == [-5.5, 5.5], 'Unexpected fixed RS05 feedback codec')
    watchdogs = review.get('device_watchdog')
    _need(type(watchdogs) is dict and set(watchdogs) == set(IDS), 'Every motor needs watchdog-loss test evidence')
    result = {}
    command_loss_only = data.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED
    for mid, row in watchdogs.items():
        _need(type(row) is dict and row.get('motor_model') == 'RS05' and
              row.get('actual_command_loss_test_passed') is True and
              row.get('usb_disconnect_test_passed') is (False if command_loss_only else True) and
              row.get('disabled_after_loss_verified') is True,
              'STOP transmission/readback alone does not verify device watchdog: ID'+mid)
        timeout = _number(row.get('configured_timeout_ms'), 'device watchdog timeout', 200, 200)
        measured = _number(row.get('max_observed_disable_ms'), 'observed command-loss stop', 0, 250, positive=True)
        _need(timeout >= data['hard_cycle_ms']+20 and measured <= timeout+50,
              'Watchdog timing inconsistent with cycle budget')
        fingerprint=row.get('version_bytes_hex')
        _need(type(fingerprint) is str and len(fingerprint)==8 and
              all(c in '0123456789abcdef' for c in fingerprint),
              'Tested raw firmware version_bytes_hex required: ID'+mid)
        # Do not interpret these protocol bytes as a semantic release number.
        # Old string-only reviews cannot bind the version read before enabling.
        if row.get('firmware_version') is not None:
            _text(row['firmware_version'], 'informational tested firmware version')
        if command_loss_only:
            measured_row = command_loss_report['axes'][mid]
            _need(measured_row.get('version', {}).get('version_bytes_hex') == fingerprint and
                  measured_row['disable_reply_upper_bound_ms'] == measured,
                  'Watchdog review differs from measured command-loss evidence: ID'+mid)
        result[mid] = {'configured_timeout_ms': timeout, 'max_observed_disable_ms': measured,
                       'version_bytes_hex':fingerprint,
                       'firmware_version': row.get('firmware_version')}
        if command_loss_only:
            result[mid].update(usb_disconnect_test_passed=False,
                              usb_disconnect_test_waived_for_supported_trial=True)
    _need(review.get('mode0_readback_required_before_enable') is True,
          'Mode0 must be freshly read back before enable')
    _text(review.get('timing_budget_rationale'), 'reviewed timing budget rationale')
    return result


def _v2_supported_extension_context(documents, data, prior, report):
    from .policy_post_reply_timing import POST_REPLY_POLICY_V2
    settings = _post_reply_policy(data)
    if settings is None or settings['mode'] != POST_REPLY_POLICY_V2:
        return False
    _need(type(report) is dict and type(documents.get('pipeline_diagnostic')) is dict,
          'V2 extension needs original report and diagnostic mappings')
    _need(_post_reply_policy(prior) == settings and data['axes'] == prior['axes'] and
          data['start_pose_bounds'] == prior['start_pose_bounds'] and
          data['artifacts']['local_reference_capture']['sha256'] ==
              prior['artifacts']['local_reference_capture']['sha256'],
          'V2 extension must preserve the exact pose, physical limits and local capture')
    diagnostic = documents['pipeline_diagnostic']
    _need(diagnostic.get('motor_power_epoch') == data['motor_power_epoch'] and
          diagnostic.get('cadence_source_sha256') == data['cadence_source_sha256'],
          'V2 extension needs the exact current diagnostic sources and power')
    if prior['artifacts']['pipeline_diagnostic']['sha256'] != data['artifacts']['pipeline_diagnostic']['sha256']:
        rows = report.get('cycles')
        measurements = diagnostic.get('measurements')
        _need(type(measurements) is list and bool(measurements) and type(measurements[0]) is dict,
              'V2 extension diagnostic measurements missing')
        first = measurements[0].get('release_ns')
        last = rows[-1].get('end_ns') if type(rows) is list and rows else None
        _need(type(first) is int and type(last) is int and first > last,
              'Fresh V2 extension diagnostic must follow the completed predecessor')
    return True


def _post_reply_input_age_predecessor(report, prior):
    """Replay V2 admissions from original CAN/IMU times, never summary ages.

    This additional proof is opt-in. Legacy V1 predecessor contracts keep their
    original checks. Native writes/replies and returned feedback retain 20ms.
    """
    from . import can_readonly as codec
    from . import rs05_trial_protocol as protocol
    from .policy_post_reply_timing import POST_REPLY_POLICY_V2, PostReplyDeadlineBudget
    settings = _post_reply_policy(prior)
    if settings is None or settings['mode'] != POST_REPLY_POLICY_V2:
        return
    _need(report.get('post_reply_deadline_policy') == settings,
          'V2 predecessor policy differs from its reviewed profile')
    cycles, journal = report.get('cycles'), report.get('journal')
    _need(type(cycles) is list and bool(cycles) and type(journal) is list,
          'V2 predecessor needs original cycle and wire records')
    groups = {phase: {bus: [] for bus in ('front', 'rear')}
              for phase in ('input', 'output')}
    for item in journal:
        _need(type(item) is dict, 'V2 predecessor journal entry invalid')
        phase = item.get('phase')
        group = ('input' if phase == 'feedback_hold' else
                 'output' if phase in ('startup_hold', 'policy_output', 'graceful_stop') else None)
        if group is not None:
            _need(item.get('bus') in groups[group], 'V2 predecessor wire bus invalid')
            groups[group][item['bus']].append(item)
    _need(all(len(items) == len(cycles) for buses in groups.values() for items in buses.values()),
          'V2 predecessor original wire coverage incomplete')

    def records(item, bus, begin, end):
        rows = item.get('records')
        ids = list(range(1, 7)) if bus == 'front' else list(range(7, 13))
        _need(item.get('error') is None and type(rows) is list and len(rows) == 6 and
              type(item.get('rejected_total')) is int and item['rejected_total'] == 0 and
              item.get('rejected_hex') == '' and
              item.get('rejected_truncated') is False,
              'V2 predecessor has incomplete or rejected native wire')
        for mid, row in zip(ids, rows):
            _need(type(row) is dict and row.get('written') == row.get('received') == 17,
                  'V2 predecessor native wire is incomplete')
            stamps = [row.get(k) for k in ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns')]
            deadline = row.get('deadline_ns')
            _need(all(type(v) is int and 0 < v < 2**63 for v in stamps+[deadline]) and
                  begin <= stamps[0] <= stamps[1] <= stamps[3] <= end and
                  stamps[0] <= stamps[2] <= stamps[3] < deadline <= begin+20_000_000,
                  'V2 predecessor native write/reply deadline differs')
            try:
                tx, rx = bytes.fromhex(row['tx_hex']), bytes.fromhex(row['rx_hex'])
                tx_frames, rx_frames = codec.ATParser().feed(tx), codec.ATParser().feed(rx)
                _need(len(tx) == len(rx) == 17 and len(tx_frames) == len(rx_frames) == 1 and
                      tx_frames[0].wire == tx and rx_frames[0].wire == rx and
                      tx_frames[0].flags == 4 and tx_frames[0].kind == 1 and
                      tx_frames[0].destination == mid,
                      'V2 predecessor native wire type/ID differs')
                decoded = protocol.decode_type2(rx_frames[0], motor_id=mid)
            except (ValueError, TypeError, KeyError) as error:
                raise ProfileError('Invalid V2 predecessor original wire') from error
            _need(decoded.fault_bits == 0 and decoded.mode_state == 2,
                  'V2 predecessor original output feedback faulted or disabled')
        return rows

    budget = PostReplyDeadlineBudget(settings)
    startup = _startup_cycle_policy(prior) is not None
    startup_count, misses, previous_end = 0, 0, None
    for index, cycle in enumerate(cycles):
        _need(type(cycle) is dict and type(cycle.get('index')) is int and cycle['index'] == index,
              'V2 predecessor cycle sequence invalid')
        begin, end = cycle.get('begin_ns'), cycle.get('end_ns')
        _need(type(begin) is int and type(end) is int and 0 < begin <= end and
              (previous_end is None or begin >= previous_end), 'V2 predecessor cycle times invalid')
        inputs, outputs = [], []
        for bus in ('front', 'rear'):
            inputs += records(groups['input'][bus][index], bus, begin, end)
            outputs += records(groups['output'][bus][index], bus, begin, end)
        imu = cycle.get('imu', {})
        imu_start, imu_end = (imu.get(k) for k in
                              ('read_started_monotonic_ns', 'read_finished_monotonic_ns'))
        _need(type(imu_start) is int and type(imu_end) is int and begin <= imu_start <= imu_end <= end,
              'V2 predecessor original IMU interval invalid')
        oldest = min(imu_start, min(row['start_ns'] for row in inputs))
        final_write = max(row['finish_ns'] for row in outputs)
        replied = max(row['received_ns'] for row in outputs)
        _need(cycle.get('output_reply_end_ns') == replied and
              cycle.get('oldest_input_to_final_host_write_ms') == (final_write-oldest)/1e6,
              'V2 predecessor output reply/input age differs from original wire')
        try:
            decision = budget.admit(index=index, begin_ns=begin, oldest_input_ns=oldest,
                final_write_ns=final_write, last_reply_ns=replied,
                output_sample_start_ns=min(row['start_ns'] for row in outputs), checked_ns=end,
                sample_age_ns=int(prior['max_sample_age_ms']*1e6),
                startup_allowed=startup and index == 0)
        except RuntimeError as error:
            raise ProfileError('V2 predecessor '+str(error)) from error
        _need(cycle.get('post_reply_deadline') == decision and
              cycle.get('deadline20ms_missed') is (end-begin > 20_000_000) and
              cycle.get('steady_deadline20ms_missed') is decision['allowance_used'] and
              cycle.get('startup_20ms_allowance_used') is decision['startup_allowance_used'],
              'V2 predecessor admission proof differs from original timestamps')
        startup_count += int(decision['startup_allowance_used'])
        misses += int(end-begin > 20_000_000)
        previous_end = end
    for name, count in (('deadline20ms_misses', misses),
                        ('steady_deadline20ms_misses', budget.accepted_misses),
                        ('post_reply_deadline_allowance_uses', budget.accepted_misses),
                        ('startup_20ms_allowance_uses', startup_count)):
        _need(type(report.get(name)) is int and report[name] == count,
              'V2 predecessor admission counts differ')


def _supported_extension_evidence(documents, data):
    """Admit only a duration extension of a pinned, completed two-second run.

    Prior files retain their original bytes and loader hash. Never recursively
    load an old profile against the new admission-only loader source.
    """
    prior = documents['prior_supported_profile']
    report = documents['prior_supported_report']
    observed = documents['prior_supported_observation']
    _structure(prior)
    _need(prior['approved_for_supported_policy_output'] is True and prior['blockers'] == [] and
          prior['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER and
          prior['duration_s'] == 2. and 2. < data['duration_s'] <= 10.,
          'Extension requires an approved two-second predecessor and at most ten seconds')
    _review(prior['review'], 'APPROVED_SUPPORTED_CHARACTERIZATION')
    hypothesis_extension = (accel_input_hypothesis_selected(data) and
                            accel_input_hypothesis_selected(prior))
    v2_extension = _v2_supported_extension_context(documents, data, prior, report)
    # A hypothesis extension is measured again with the current source graph.
    # Only this diagnostic reference may change; the prior runtime, input
    # hypothesis, model, power and every live limit remain exact below.
    replaceable_artifacts = (*_EXTENSION_ARTIFACTS, 'hardware_review',
                             'operator_acceptance', 'local_reference_capture')
    if hypothesis_extension or v2_extension:
        replaceable_artifacts += ('pipeline_diagnostic',)
    def contract(profile):
        omitted = {'artifacts', 'review', 'blockers', 'approved_for_supported_policy_output',
                   'assembly_id', 'bundle_path', 'duration_s', 'diagnostic_timing_acceptance',
                   'start_pose_bounds', 'axes', 'cadence_source_sha256'}
        result = {k:v for k,v in profile.items() if k not in omitted}
        result['axes'] = {mid:{k:v for k,v in axis.items()
            if k not in ('physical_lower_rad', 'physical_upper_rad')} for mid,axis in profile['axes'].items()}
        result['sources'] = {k:v for k,v in profile['cadence_source_sha256'].items()
                            if k != 'singularitydog_hw/policy_live_profile.py'}
        result['artifacts'] = {k:v['sha256'] for k,v in profile['artifacts'].items()
            if k not in replaceable_artifacts}
        return result
    _need(contract(prior) == contract(data),
          'Extension changes prior execution, model, calibration, UID, boot, power or safety contract')
    if _prepared_voltage_publication_selected(data):
        _need(type(report) is dict and report.get('prepare_voltage_before_feedback_publication') is True,
              'Prepared voltage extension requires the selected actual predecessor')
        _prepared_voltage_publication_predecessor(report, prior)
    profile_sha = data['artifacts']['prior_supported_profile']['sha256']
    report_sha = data['artifacts']['prior_supported_report']['sha256']
    _need(type(report) is dict and report.get('profile_sha256') == profile_sha and
          report.get('boot_id') == data['boot_id'] and
          report.get('motor_power_epoch') == data['motor_power_epoch'] and
          report.get('cadence_source_sha256') == prior['cadence_source_sha256'] and
          report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == [] and
          report.get('scope') == data['scope'] and report.get('normal_ramp_completed') is True and
          report.get('learned_targets_sent') is True and report.get('stop_confirmed') is True and
          report.get('post_reply_deadline_allowance_uses') == 0 and
          report.get('post_reply_deadline_rejections') == [] and
          report.get('trial_displacement_origin') == 'final_pre_enable_feedback',
          'Extension requires successful same-session learned output with no deadline allowance')
    startup_enabled = prior.get('startup_cycle_allowance') == FIRST_CYCLE_POST_REPLY
    startup_misses = report.get('startup_20ms_allowance_uses', 0)
    _need(type(startup_misses) is int and 0 <= startup_misses <= int(startup_enabled) and
          report.get('deadline20ms_misses') == startup_misses and
          report.get('steady_deadline20ms_misses', 0) == 0 and
          report.get('startup_20ms_allowance_enabled', False) is startup_enabled,
          'Extension predecessor startup exception differs or steady timing failed')
    native = report.get('native_batch_encoder', {})
    _need(type(prior.get('native_batch_encoder')) is dict and native.get('enabled') is True and
          native.get('binary_sha256') == prior['native_batch_encoder']['sha256'] and
          report.get('execution_settings') == execution_settings(prior) and
          report.get('transport_settings', {}).get('request_gap_us') == data['request_gap_us'] and
          report.get('transport_settings', {}).get('request_window') == data['request_window'],
          'Extension predecessor encoder/backend/pacing differs')
    stops = report.get('stop_reports', {})
    for scope, ids in (('front', list(range(1,7))), ('rear', list(range(7,13)))):
        stop = stops.get(scope, {})
        _need(stop.get('complete') is True and stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
              stop.get('fault_by_id') == {str(mid):0 for mid in ids},
              'Extension predecessor STOP evidence incomplete or faulted')
    rows = report.get('cycles')
    _need(type(rows) is list and 80 <= len(rows) <= 102,
          'Extension requires at least eighty completed predecessor cycles')
    _post_reply_input_age_predecessor(report, prior)
    if hypothesis_extension:
        # The full loader has already checked the fresh diagnostic's selected
        # input and exact current sources in _timing. Bind the actual prior
        # run to that same audited hypothesis as well, including gain-down.
        provenance = documents['pipeline_diagnostic'].get('observer', {}).get('accel_input_hypothesis')
        _need(type(provenance) is dict and
              provenance.get('hypothesis_sha256') == data['artifacts']['accel_input_hypothesis']['sha256'] and
              provenance.get('formal_calibration_approved') is False and
              provenance.get('grants_motor_output') is False and
              report.get('current_position_hold_only') is False and
              report.get('cyclic_inference_skipped') is False and
              type(report.get('actual_model_calls')) is int and
              0 < report['actual_model_calls'] <= len(rows),
              'Hypothesis extension requires actual prior inference with the same unapproved input')
        for row in rows:
            imu = row.get('imu_body', {})
            _need(type(imu) is dict and imu.get('accel_bias_subtracted') is True and
                  imu.get('accel_scale_corrected') is True and
                  'reviewed_accel_calibration' not in imu and
                  json.dumps(imu.get('accel_input_hypothesis'), sort_keys=True, separators=(',', ':'), allow_nan=False) ==
                  json.dumps(provenance, sort_keys=True, separators=(',', ':'), allow_nan=False),
                  'Hypothesis extension predecessor actual input provenance differs')
        if (prior['artifacts']['pipeline_diagnostic']['sha256'] !=
                data['artifacts']['pipeline_diagnostic']['sha256']):
            first = documents['pipeline_diagnostic'].get('measurements', [{}])[0].get('release_ns')
            last = rows[-1].get('end_ns')
            _need(type(first) is int and type(last) is int and first > last,
                  'Fresh hypothesis extension diagnostic must follow the completed two-second run')
    previous_end = None
    for index, row in enumerate(rows):
        _need(type(row) is dict and row.get('index') == index, 'Extension predecessor cycle sequence invalid')
        stamps = [row.get(k) for k in ('begin_ns','output_reply_end_ns','end_ns')]
        _need(all(type(v) is int and v > 0 for v in stamps) and stamps == sorted(stamps),
              'Extension predecessor timestamps invalid')
        begin, replied, end = stamps
        timing = row.get('post_reply_deadline', {})
        startup_used = index == 0 and startup_enabled and end-begin > 20_000_000
        _need(end-begin <= (21_000_000 if startup_used else 20_000_000) and
              replied-begin <= 20_000_000 and (previous_end is None or begin >= previous_end) and
              timing.get('accepted') is True and timing.get('checked_ns') == end and
              timing.get('allowance_used') is False and timing.get('startup_allowance_used') is startup_used and
              (index != 0 or int(startup_used) == startup_misses),
              'Extension predecessor cycle exceeds hard deadline or uses allowance')
        previous_end = end
    _need(rows[0].get('phase') == 'starting' and rows[-1].get('phase') == 'stopped' and
          1_500_000_000 <= rows[-1]['end_ns']-rows[0]['begin_ns'] <= 2_040_000_000 and
          any(row.get('phase') == 'active' and row.get('effective_policy_weight') == prior['policy_weight']
              for row in rows), 'Extension predecessor did not complete its learned ramp and stop')
    _need(type(observed) is dict and observed.get('report_sha256') == report_sha and
          observed.get('observed_by') == 'operator' and observed.get('audio_heard') is True and
          observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          observed.get('box_support_maintained') is True and
          observed.get('autonomous_standing_or_walking_observed') is False,
          'Extension requires a matching operator observation of the completed supported run')
    _text(observed.get('user_statement'), 'predecessor physical observation')
    acceptance = documents['hardware_review'].get('supported_extension_acceptance', {})
    _need(type(acceptance) is dict and acceptance.get('mode') == SUPPORTED_POLICY_PROBE_10S_AFTER_2S and
          acceptance.get('scope') == data['scope'] and acceptance.get('only_duration_extended') is True and
          acceptance.get('live_limits_unchanged') is True and
          acceptance.get('prior_profile_sha256') == profile_sha and
          acceptance.get('prior_report_sha256') == report_sha and
          acceptance.get('prior_observation_sha256') == data['artifacts']['prior_supported_observation']['sha256'],
          'Explicit hash-bound supported extension review required')
    _review(acceptance.get('review'), 'ACCEPT_10S_SUPPORTED_AFTER_2S')


def _supported_20s_extension_evidence(documents, data, base):
    """Extend only a completed same-pose 2s -> 10s learned-output sequence.

    Historical profiles keep their original source pins. Only this admission
    loader may differ; neither numerical limits nor the physical reference may
    change. A supported run remains evidence of bounded output, not load bearing
    or a learned rise from the box.
    """
    prior = documents['prior_supported_profile']
    report = documents['prior_supported_report']
    observed = documents['prior_supported_observation']
    _structure(prior)
    _need(data['duration_s'] == 20. and prior['schema'] == SCHEMA_V3 and
          prior['scope'] == data['scope'] == 'supported_characterization_only' and
          prior['approved_for_supported_policy_output'] is True and prior['blockers'] == [] and
          prior.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_10S_AFTER_2S and
          prior['duration_s'] == 10. and prior['policy_weight'] > 0,
          'Twenty-second extension requires an approved ten-second box-supported learned predecessor')
    _review(prior['review'], 'APPROVED_SUPPORTED_CHARACTERIZATION')

    hypothesis_extension = (accel_input_hypothesis_selected(data) and
                            accel_input_hypothesis_selected(prior))
    v2_extension = _v2_supported_extension_context(documents, data, prior, report)
    replaceable_artifacts = (*_EXTENSION_ARTIFACTS, 'hardware_review', 'operator_acceptance')
    if hypothesis_extension or v2_extension:
        replaceable_artifacts += ('pipeline_diagnostic',)

    def contract(value):
        omitted = {'artifacts', 'review', 'blockers', 'approved_for_supported_policy_output',
                   'assembly_id', 'bundle_path', 'duration_s', 'diagnostic_timing_acceptance',
                   'cadence_source_sha256'}
        result = {k:v for k,v in value.items() if k not in omitted}
        _need(type(value['cadence_source_sha256']) is dict, 'Twenty-second source pins must be a mapping')
        result['sources'] = {k:v for k,v in value['cadence_source_sha256'].items()
                            if k != 'singularitydog_hw/policy_live_profile.py'}
        result['artifacts'] = {k:v['sha256'] for k,v in value['artifacts'].items()
            if k not in replaceable_artifacts}
        return result

    _need(contract(prior) == contract(data),
          'Twenty-second extension changes predecessor pose, execution, model, calibration, UID, boot, power or limits')
    # Verify the old ten-second review's hash graph without recursively loading
    # its historical loader bytes against the current frozen kit.
    _, prior_ref = _artifact(data['artifacts']['prior_supported_profile'], base)
    prior_base = Path(prior_ref['path']).parent
    nested = {key:_artifact(prior['artifacts'][key], prior_base)[0]
              for key in artifact_names(prior)}
    _supported_extension_evidence(nested, prior)
    earlier = nested['prior_supported_profile']
    _need(earlier['axes'] == prior['axes'] and
          earlier['start_pose_bounds'] == prior['start_pose_bounds'] and
          earlier['artifacts']['local_reference_capture']['sha256'] ==
              prior['artifacts']['local_reference_capture']['sha256'],
          'Twenty-second extension requires the same physical pose throughout two/ten/twenty-second trials')
    checked_prior = copy.deepcopy(prior)
    checked_prior['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = (
        data['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'])
    _settings(checked_prior)
    _axes(checked_prior, nested['calibration'])
    _supported_command_loss_acceptance(nested['operator_acceptance'], nested['command_loss_report'], prior)
    _hardware(nested['hardware_review'], prior,
              Path(_artifact(prior['artifacts']['hardware_review'], prior_base)[1]['path']).parent,
              command_loss_report=nested['command_loss_report'],
              local_reference_capture=nested['local_reference_capture'])
    prior_timing = copy.deepcopy(prior)
    if hypothesis_extension:
        # The outer loader audited this same pinned hypothesis, mount and bias.
        # Supply that audited provenance solely as comparison context for the
        # original ten-second diagnostic; its original graph remains intact.
        current_hypothesis_provenance = documents['pipeline_diagnostic'].get('observer', {}).get('accel_input_hypothesis')
        _need(type(current_hypothesis_provenance) is dict,
              'Twenty-second extension requires the already audited hypothesis provenance')
        prior_timing['_accel_input_hypothesis_provenance'] = current_hypothesis_provenance
    _timing(nested['pipeline_diagnostic'], prior_timing)

    if _prepared_voltage_publication_selected(data):
        _need(report.get('prepare_voltage_before_feedback_publication') is True,
              'Prepared voltage twenty-second extension requires the selected actual predecessor')
        _prepared_voltage_publication_predecessor(report, prior)

    profile_sha = data['artifacts']['prior_supported_profile']['sha256']
    report_sha = data['artifacts']['prior_supported_report']['sha256']
    _need(type(report) is dict and report.get('profile_sha256') == profile_sha and
          report.get('boot_id') == data['boot_id'] and
          report.get('motor_power_epoch') == data['motor_power_epoch'] and
          report.get('cadence_source_sha256') == prior['cadence_source_sha256'] and
          report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == [] and
          report.get('scope') == data['scope'] and report.get('normal_ramp_completed') is True and
          all(report.get(k) is True for k in ('motor_enable_sent', 'motion_gain_sent',
              'command_output_sent', 'learned_targets_sent', 'stop_confirmed')) and
          report.get('current_position_hold_only') is False and report.get('cyclic_inference_skipped') is False and
          report.get('post_reply_deadline_rejections') == [] and
          report.get('trial_displacement_origin') == 'final_pre_enable_feedback',
          'Twenty-second extension requires successful same-session learned output and normal STOP')
    native, provenance = report.get('native_batch_encoder', {}), report.get('model_provenance', {})
    pacing, stops = report.get('transport_settings'), report.get('stop_reports')
    _need(all(type(value) is dict for value in (native, provenance, pacing, stops)) and
          type(provenance.get('baseline_provenance')) is dict,
          'Twenty-second predecessor model/pacing/STOP mappings missing')
    _need(type(prior.get('native_batch_encoder')) is dict and native.get('enabled') is True and
          native.get('binary_sha256') == prior['native_batch_encoder']['sha256'] and
          report.get('execution_settings') == execution_settings(prior) and
          provenance.get('manifest_sha256') == data['artifacts']['scalar_step_manifest']['sha256'] and
          provenance.get('baseline_provenance', {}).get('manifest_sha256') ==
              data['artifacts']['model_manifest']['sha256'] and
          pacing.get('request_gap_us') == data['request_gap_us'] and
          pacing.get('request_window') == data['request_window'],
          'Twenty-second predecessor encoder/model/backend/pacing differs')
    for scope, ids in (('front', list(range(1,7))), ('rear', list(range(7,13)))):
        stop = stops.get(scope, {})
        _need(type(stop) is dict and stop.get('complete') is True and stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
              stop.get('fault_by_id') == {str(mid):0 for mid in ids},
              'Twenty-second predecessor STOP evidence incomplete or faulted')
    rows = report.get('cycles')
    _need(type(rows) is list and 475 <= len(rows) <= 502 and
          type(report.get('actual_model_calls')) is int and 400 <= report['actual_model_calls'] <= len(rows),
          'Twenty-second extension requires completed ten-second learned cycles')
    _post_reply_input_age_predecessor(report, prior)
    if hypothesis_extension:
        provenance = current_hypothesis_provenance
        for row in rows:
            imu = row.get('imu_body', {})
            _need(type(imu) is dict and imu.get('accel_bias_subtracted') is True and
                  imu.get('accel_scale_corrected') is True and
                  'reviewed_accel_calibration' not in imu and
                  json.dumps(imu.get('accel_input_hypothesis'), sort_keys=True, separators=(',', ':'), allow_nan=False) ==
                  json.dumps(provenance, sort_keys=True, separators=(',', ':'), allow_nan=False),
                  'Hypothesis twenty-second predecessor actual input provenance differs')
        if (prior['artifacts']['pipeline_diagnostic']['sha256'] !=
                data['artifacts']['pipeline_diagnostic']['sha256']):
            first = documents['pipeline_diagnostic'].get('measurements', [{}])[0].get('release_ns')
            last = rows[-1].get('end_ns')
            _need(type(first) is int and type(last) is int and first > last,
                  'Fresh hypothesis twenty-second diagnostic must follow the completed ten-second run')
    post_reply = _post_reply_policy(prior)
    _need(post_reply is not None and report.get('post_reply_deadline_policy') == post_reply,
          'Twenty-second predecessor post-reply policy differs')
    startup_enabled = _startup_cycle_policy(prior) is not None
    _need(report.get('startup_20ms_allowance_enabled', False) is startup_enabled,
          'Twenty-second predecessor startup policy differs')
    origin = report.get('trial_origin_model_rad_by_id')
    _need(type(origin) is dict and set(origin) == set(IDS),
          'Twenty-second predecessor requires twelve final pre-enable displacement origins')
    for mid, value in origin.items():
        axis = prior['axes'][mid]
        _number(value, 'predecessor origin ID'+mid, axis['physical_lower_rad'], axis['physical_upper_rad'])
    misses, startup_misses, previous_end, active_seen = [], 0, None, False
    for index, row in enumerate(rows):
        _need(type(row) is dict and row.get('index') == index, 'Twenty-second predecessor cycle sequence invalid')
        stamps = [row.get(k) for k in ('begin_ns', 'output_reply_end_ns', 'end_ns')]
        _need(all(type(v) is int and v > 0 for v in stamps) and stamps == sorted(stamps),
              'Twenty-second predecessor timestamps invalid')
        begin, replied, end = stamps
        missed = end-begin > 20_000_000
        startup_used = index == 0 and startup_enabled and missed
        steady_missed = missed and not startup_used
        if startup_used: startup_misses += 1
        if steady_missed: misses.append(index)
        rolling_misses = sum(i > index-100 for i in misses)
        timing = row.get('post_reply_deadline', {})
        _need(type(timing) is dict and end-begin <= 20_000_000+int(post_reply['max_lateness_ms']*1e6) and
              replied-begin <= 20_000_000 and (previous_end is None or begin >= previous_end) and
              (index == 0 or begin-rows[index-1]['begin_ns'] <= 21_000_000) and
              rolling_misses <= 1 and timing.get('accepted') is True and timing.get('checked_ns') == end and
              timing.get('allowance_used') is steady_missed and timing.get('startup_allowance_used') is startup_used and
              timing.get('rolling_misses') == rolling_misses and timing.get('consecutive_misses') == int(steady_missed) and
              row.get('deadline20ms_missed') is missed and row.get('steady_deadline20ms_missed') is steady_missed and
              row.get('startup_20ms_allowance_used') is startup_used,
              'Twenty-second predecessor exceeds its unchanged live timing policy')
        previous_end = end
        _number(row.get('oldest_input_to_final_host_write_ms'), 'predecessor input age', 0, prior['max_sample_age_ms'])
        _need(row.get('phase') in ('starting', 'active', 'stopping', 'stopped'),
              'Twenty-second predecessor phase invalid')
        mix = _number(row.get('effective_policy_weight'), 'predecessor policy mixture', 0, prior['policy_weight'])
        command, feedback = row.get('command', {}), row.get('feedback', {})
        _need(type(command) is dict and type(feedback) is dict and command.get('phase') == row['phase'],
              'Twenty-second predecessor command/feedback record missing')
        required = (('command', command, ('q_model_rad', 'kp', 'kd', 'velocity_reference_rad_s',
            'feedforward_torque_nm', 'command_velocity_rad_s', 'tracking_error_rad', 'estimated_pd_torque_nm')),
            ('feedback', feedback, ('q_model_rad', 'velocity_rad_s', 'torque_nm', 'temperature_c')))
        for label, vectors, names in required:
            _need(all(type(vectors.get(name)) is list and len(vectors[name]) == 12 for name in names),
                  'Twenty-second predecessor twelve-axis '+label+' vectors incomplete')
        for order, mid in enumerate(map(str, shadow.CAN_ORDER)):
            axis = prior['axes'][mid]
            lo, hi = axis['physical_lower_rad'], axis['physical_upper_rad']
            for vectors in (command, feedback):
                q = _number(vectors['q_model_rad'][order], 'predecessor position ID'+mid, lo, hi)
                _need(abs(q-origin[mid]) <= axis['max_displacement_from_start_rad'],
                      'Twenty-second predecessor exceeds displacement ID'+mid)
            for key in ('kp', 'kd'):
                _number(command[key][order], 'predecessor '+key, 0, axis[key])
            for key in ('velocity_reference_rad_s', 'feedforward_torque_nm'):
                _need(type(command[key][order]) in (int, float) and command[key][order] == 0.,
                      'Twenty-second predecessor has unreviewed velocity/torque feedforward')
            for key, limit in (('command_velocity_rad_s', 'max_command_velocity_rad_s'),
                               ('tracking_error_rad', 'max_tracking_error_rad'),
                               ('estimated_pd_torque_nm', 'max_estimated_pd_torque_nm')):
                _number(command[key][order], 'predecessor '+key, -axis[limit], axis[limit])
            for key, limit in (('velocity_rad_s', 'max_measured_velocity_rad_s'), ('torque_nm', 'max_measured_torque_nm')):
                _number(feedback[key][order], 'predecessor '+key, -axis[limit], axis[limit])
            _number(feedback['temperature_c'][order], 'predecessor temperature', 0, axis['max_temperature_c'])
        if row['phase'] == 'active' and mix == prior['policy_weight']:
            active_seen = active_seen or (command['kp'] == [prior['axes'][str(mid)]['kp'] for mid in shadow.CAN_ORDER] and
                                         command['kd'] == [prior['axes'][str(mid)]['kd'] for mid in shadow.CAN_ORDER])
    for key, count in (('deadline20ms_misses', len(misses)+startup_misses),
                       ('steady_deadline20ms_misses', len(misses)),
                       ('post_reply_deadline_allowance_uses', len(misses)),
                       ('startup_20ms_allowance_uses', startup_misses)):
        _need(type(report.get(key)) is int and report[key] == count,
              'Twenty-second predecessor deadline counts differ')
    _need(rows[0]['phase'] == 'starting' and rows[-1]['phase'] == 'stopped' and
          9_500_000_000 <= rows[-1]['end_ns']-rows[0]['begin_ns'] <= 10_040_000_000 and active_seen and
          all(value == 0 for key in ('kp', 'kd') for value in rows[-1]['command'][key]),
          'Twenty-second predecessor did not complete learned gain ramp and stop')
    _need(type(observed) is dict and observed.get('report_sha256') == report_sha and
          observed.get('observed_by') == 'operator' and observed.get('audio_heard') is True and
          observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          observed.get('box_support_maintained') is True and
          observed.get('autonomous_standing_or_walking_observed') is False,
          'Twenty-second extension requires a matching real operator observation of the ten-second supported run')
    _text(observed.get('user_statement'), 'ten-second physical observation')
    acceptance = documents['hardware_review'].get('supported_extension_acceptance', {})
    _need(type(acceptance) is dict and acceptance.get('mode') == SUPPORTED_POLICY_PROBE_20S_AFTER_10S and
          acceptance.get('scope') == data['scope'] and acceptance.get('only_duration_extended') is True and
          acceptance.get('live_limits_unchanged') is True and acceptance.get('support_must_remain') is True and
          acceptance.get('load_bearing_not_established') is True and acceptance.get('walking_allowed') is False and
          acceptance.get('prior_profile_sha256') == profile_sha and acceptance.get('prior_report_sha256') == report_sha and
          acceptance.get('prior_observation_sha256') == data['artifacts']['prior_supported_observation']['sha256'],
          'Explicit hash-bound twenty-second supported-only extension review required')
    _review(acceptance.get('review'), 'ACCEPT_20S_SUPPORTED_AFTER_10S')


def _supported_mix_step_operator_receipt(receipt, data):
    """Validate the pinned original statement, not a derived copy's flags.

    A named clearance review cannot turn an older-power statement or a denied
    support/path confirmation into current physical evidence. The file-only
    preparation tool checks this same receipt contract before making a draft;
    the loader independently enforces it for hand-assembled profiles as well.
    """
    _need(type(receipt) is dict and
          receipt.get('schema') == 'singularitydog.supported-mix-step-operator-receipt.v1' and
          receipt.get('boot_id') == data['boot_id'] and
          receipt.get('motor_power_epoch') == data['motor_power_epoch'],
          'Mix step source receipt is from another boot/power or has no original receipt schema')
    for key in ('source_id', 'question', 'answer'):
        _need(type(receipt.get(key)) is str and bool(receipt[key].strip()),
              'Mix step source receipt needs original operator '+key)
    _need(type(receipt.get('clearance_deg')) in (int, float) and receipt['clearance_deg'] == 7 and
          all(receipt.get(key) is True for key in ('all_twelve_current_clearance_confirmed',
              'box_supports_body', 'four_paws_floor', 'hands_clear', 'cutoff_ready',
              'power_and_pose_unchanged_since_prior_10s')) and
          all(receipt.get(key) is False for key in ('box_removal_authorized',
              'load_bearing_verified', 'standing_verified')),
          'Mix step source receipt lacks current seven-degree boxed physical confirmation')


def _supported_mix_step_evidence(documents, data, base):
    """A reviewed 10% target experiment, never standing or support-transfer proof.

    Validate original ten-second evidence without rewriting its source pins. The
    model replay uses that historical feedback, not feedback produced by 10%.
    Recompute all target excursions rather than trusting analysis summary flags.
    This screen is anchored to the recorded pre-enable trial origin. Live blending
    uses its fresh zero-gain sample and retains the separate displacement guard.
    """
    prior = documents['prior_supported_profile']
    report = documents['prior_supported_report']
    observed = documents['prior_supported_observation']
    _structure(prior)
    _need(prior['approved_for_supported_policy_output'] is True and prior['blockers'] == [] and
          prior['scope'] == data['scope'] == 'supported_characterization_only' and
          prior.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_10S_AFTER_2S and
          prior['duration_s'] == 10. and prior['policy_weight'] == .005,
          'Ten-percent mix step requires the approved boxed 0.5-percent ten-second predecessor')
    _review(prior['review'], 'APPROVED_SUPPORTED_CHARACTERIZATION')
    for key in ('schema', 'boot_id', 'motor_power_epoch', 'command', 'h_hypothesis',
                'start_pose_bounds', 'model_backend', 'voltage_overlap', 'voltage_pipeline',
                'native_batch_encoder', 'request_gap_us', 'request_window', 'telemetry_cadence',
                'period_ms', 'hard_cycle_ms', 'max_sample_age_ms', 'max_sample_gap_ms',
                'max_consecutive_20ms_misses', 'post_reply_deadline_policy', 'startup_cycle_allowance',
                'voltage_min_v', 'voltage_max_v',
                'imu_tilt_limit_rad', 'imu_gyro_limit_rad_s', 'imu_accel_norm_min_m_s2',
                'imu_accel_norm_max_m_s2', 'watchdog_review_policy', 'local_characterization'):
        _need(prior.get(key) == data.get(key), 'Mix step changes protected predecessor setting: '+key)
    for name in ('calibration', 'mount', 'bias', 'model_manifest', 'scalar_step_manifest',
                 'local_reference_capture'):
        _need(data['artifacts'][name]['sha256'] == prior['artifacts'][name]['sha256'],
              'Mix step changes pinned model, calibration or measured reference: '+name)
    for mid in IDS:
        for key in ('uid', 'sign', 'offset_rad', 'uncertainty_rad', 'kp', 'kd',
                    'max_measured_velocity_rad_s', 'max_measured_torque_nm', 'max_temperature_c'):
            _need(data['axes'][mid][key] == prior['axes'][mid][key],
                  'Mix step changes protected axis setting: ID'+mid+' '+key)
        _need(all(documents['hardware_review']['type2_dynamic'][mid].get(k) is False for k in
                  ('output_shaft_position_verified', 'velocity_scale_and_sign_verified',
                   'torque_interpretation_verified')),
              'Mix step must preserve unverified dynamic feedback rather than certify it')
    # The prior profile has a separate original 2s->10s graph. Verify it in place.
    _, prior_ref = _artifact(data['artifacts']['prior_supported_profile'], base)
    prior_base = Path(prior_ref['path']).parent
    nested = {key: _artifact(prior['artifacts'][key], prior_base)[0] for key in artifact_names(prior)}
    _supported_extension_evidence(nested, prior)
    sources = documents['mix_step_source_review']
    old, new = prior['cadence_source_sha256'], data['cadence_source_sha256']
    _need(set(old) == set(new), 'Mix step source set differs from the prior supported runtime')
    changed = {key: {'before': old[key], 'after': new[key]} for key in old if old[key] != new[key]}
    _need(set(changed) <= _MIX_STEP_CHANGED_SOURCES and
          type(sources) is dict and sources.get('schema') == 'singularitydog.supported-mix-step-source-review.v1' and
          sources.get('prior_source_sha256') == old and sources.get('new_source_sha256') == new and
          sources.get('changed_sources') == changed,
          'Mix step requires the exact narrow loader/diagnostic source delta')
    _review(sources.get('review'), 'ACCEPT_SUPPORTED_MIX_STEP_SOURCE_DELTA')
    checked_prior = copy.deepcopy(prior)
    checked_prior['cadence_source_sha256'] = copy.deepcopy(new)
    _settings(checked_prior)
    _axes(checked_prior, nested['calibration'])
    _supported_command_loss_acceptance(nested['operator_acceptance'], nested['command_loss_report'], prior)
    _hardware(nested['hardware_review'], prior,
              Path(_artifact(prior['artifacts']['hardware_review'], prior_base)[1]['path']).parent,
              command_loss_report=nested['command_loss_report'],
              local_reference_capture=nested['local_reference_capture'])
    _timing(nested['pipeline_diagnostic'], prior)
    profile_sha = data['artifacts']['prior_supported_profile']['sha256']
    report_sha = data['artifacts']['prior_supported_report']['sha256']
    _need(report.get('profile_sha256') == profile_sha and
          report.get('boot_id') == data['boot_id'] and report.get('motor_power_epoch') == data['motor_power_epoch'] and
          report.get('cadence_source_sha256') == old and report.get('scope') == data['scope'] and
          report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == [] and
          all(report.get(k) is True for k in ('motor_enable_sent', 'motion_gain_sent', 'command_output_sent',
              'learned_targets_sent', 'normal_ramp_completed', 'stop_confirmed')) and
          report.get('current_position_hold_only') is False and report.get('cyclic_inference_skipped') is False and
          all(type(report.get(k)) is int and report[k] == 0 for k in ('deadline20ms_misses',
              'steady_deadline20ms_misses', 'post_reply_deadline_allowance_uses', 'startup_20ms_allowance_uses')) and
          report.get('post_reply_deadline_rejections') == [] and
          report.get('trial_displacement_origin') == 'final_pre_enable_feedback' and
          report.get('execution_settings') == execution_settings(prior),
          'Mix step requires a complete original ten-second learned run without timing allowances')
    _need(report.get('native_batch_encoder', {}).get('binary_sha256') == prior['native_batch_encoder']['sha256'] and
          report.get('transport_settings', {}).get('request_gap_us') == prior['request_gap_us'] and
          report.get('transport_settings', {}).get('request_window') == prior['request_window'] and
          report.get('model_provenance', {}).get('manifest_sha256') == prior['artifacts']['scalar_step_manifest']['sha256'] and
          report.get('model_provenance', {}).get('baseline_provenance', {}).get('manifest_sha256') ==
              prior['artifacts']['model_manifest']['sha256'], 'Mix step predecessor model or transport differs')
    for bus, ids in (('front', list(range(1, 7))), ('rear', list(range(7, 13)))):
        stop = report.get('stop_reports', {}).get(bus, {})
        _need(stop.get('complete') is True and stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
              stop.get('fault_by_id') == {str(mid): 0 for mid in ids},
              'Mix step predecessor STOP is incomplete or faulted')
    rows, count = report.get('cycles'), report.get('actual_model_calls')
    _need(type(rows) is list and 475 <= len(rows) <= 502 and type(count) is int and
          400 <= count < len(rows), 'Mix step predecessor must contain the complete ten-second model-call sequence')
    origin = report.get('trial_origin_model_rad_by_id')
    _need(type(origin) is dict and set(origin) == set(IDS), 'Mix step requires twelve original displacement origins')
    previous_end = None
    active = False
    for index, row in enumerate(rows):
        _need(type(row) is dict and row.get('index') == index, 'Mix step predecessor cycle sequence invalid')
        begin, replied, end = (row.get(k) for k in ('begin_ns', 'output_reply_end_ns', 'end_ns'))
        _need(all(type(v) is int and v > 0 for v in (begin, replied, end)) and
              begin <= replied <= end <= begin+20_000_000 and
              (previous_end is None or begin >= previous_end) and
              (index == 0 or begin-rows[index-1]['begin_ns'] <= 21_000_000),
              'Mix step predecessor timing is noncausal or exceeds live deadlines')
        previous_end = end
        _number(row.get('oldest_input_to_final_host_write_ms'), 'prior sample age', 0, 20)
        timing = row.get('post_reply_deadline', {})
        _need(timing.get('accepted') is True and timing.get('checked_ns') == end and
              timing.get('allowance_used') is False and timing.get('startup_allowance_used') is False and
              row.get('deadline20ms_missed') is False and row.get('steady_deadline20ms_missed') is False and
              row.get('startup_20ms_allowance_used') is False and
              row.get('phase') in (('starting', 'active') if index < count else ('stopping', 'stopped')),
              'Mix step predecessor model-call prefix or deadline proof differs')
        command, feedback = row.get('command', {}), row.get('feedback', {})
        _need(command.get('phase') == row['phase'], 'Mix step predecessor command phase differs')
        for values, names in ((command, ('q_model_rad', 'kp', 'kd', 'command_velocity_rad_s',
              'tracking_error_rad', 'estimated_pd_torque_nm', 'velocity_reference_rad_s', 'feedforward_torque_nm')),
              (feedback, ('q_model_rad', 'velocity_rad_s', 'torque_nm', 'temperature_c'))):
            _need(all(type(values.get(k)) is list and len(values[k]) == 12 for k in names),
                  'Mix step predecessor twelve-axis feedback/command is incomplete')
        # Active-output records are ID1..12 order, unlike model tensors' CAN_ORDER.
        for order, mid in enumerate(IDS):
            axis = prior['axes'][mid]
            q0 = _number(origin[mid], 'prior origin ID'+mid, axis['physical_lower_rad'], axis['physical_upper_rad'])
            for values in (command, feedback):
                q = _number(values['q_model_rad'][order], 'prior q ID'+mid,
                            axis['physical_lower_rad'], axis['physical_upper_rad'])
                _need(abs(q-q0) <= axis['max_displacement_from_start_rad'], 'Mix step predecessor displacement invalid')
            for key in ('kp', 'kd'): _number(command[key][order], 'prior '+key, 0, axis[key])
            for key in ('command_velocity_rad_s', 'tracking_error_rad', 'estimated_pd_torque_nm'):
                _number(command[key][order], 'prior '+key, -axis['max_'+key], axis['max_'+key])
            for key in ('velocity_reference_rad_s', 'feedforward_torque_nm'):
                _need(type(command[key][order]) in (int, float) and command[key][order] == 0,
                      'Mix step predecessor contains unreviewed feedforward')
            for key in ('velocity_rad_s', 'torque_nm'):
                limit = axis['max_measured_velocity_rad_s' if key == 'velocity_rad_s' else 'max_measured_torque_nm']
                _number(feedback[key][order], 'prior measured '+key, -limit, limit)
            _number(feedback['temperature_c'][order], 'prior measured temperature', 0, axis['max_temperature_c'])
        mix = _number(row.get('effective_policy_weight'), 'prior mixture', 0, prior['policy_weight'])
        active |= row['phase'] == 'active' and mix == prior['policy_weight']
    _need(rows[0]['phase'] == 'starting' and rows[-1]['phase'] == 'stopped' and active and
          9_500_000_000 <= rows[-1]['end_ns']-rows[0]['begin_ns'] <= 10_040_000_000 and
          all(v == 0 for key in ('kp', 'kd') for v in rows[-1]['command'][key]),
          'Mix step predecessor did not complete the learned gain ramp and stop')
    _need(observed.get('report_sha256') == report_sha and observed.get('observed_by') == 'operator' and
          observed.get('audio_heard') is True and observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          observed.get('box_support_maintained') is True and observed.get('autonomous_standing_or_walking_observed') is False,
          'Mix step requires the real matching predecessor observation')
    _text(observed.get('user_statement'), 'ten-second operator observation')
    targets, analysis = documents['saved_policy_target_sequence'], documents['policy_mixture_analysis']
    target_sha = data['artifacts']['saved_policy_target_sequence']['sha256']
    _need(targets.get('schema') == 'singularitydog.saved-policy-target-sequence.v1' and
          targets.get('id_order') == list(range(1, 13)) and
          all(type(mid) is int for mid in targets['id_order']) and targets.get('profile_sha256') == profile_sha and
          targets.get('report_sha256') == report_sha and targets.get('historical_boot_id') == data['boot_id'] and
          targets.get('historical_motor_power_epoch') == data['motor_power_epoch'] and
          targets.get('model_backend') == SCALAR_BACKEND and targets.get('command') == data['command'] and
          targets.get('h_hypothesis') == data['h_hypothesis'] and targets.get('model_provenance') == report['model_provenance'] and
          all(targets.get(k) is False for k in ('output_allowed', 'hardware_accessed', 'closed_loop_prediction',
              'load_bearing_verified', 'standing_verified', 'box_removal_allowed')),
          'Mix step requires the pinned original-input model replay without output or standing claims')
    replay_manifest, replay_ref = _artifact(sources.get('replay_kit_manifest'), base)
    _need(replay_ref['sha256'] == targets.get('kit_manifest_sha256') and
          type(replay_manifest.get('files')) is dict and
          all(replay_manifest['files'].get('runtime/'+name) == digest for name, digest in old.items()),
          'Mix step replay kit must contain the exact predecessor runtime sources')
    replay_source = sources.get('replay_source')
    _need(type(replay_source) is dict and set(replay_source) == {'path', 'sha256'}, 'Pinned replay source required')
    replay_path = Path(_text(replay_source['path'], 'replay source path'))
    if not replay_path.is_absolute(): replay_path = base/replay_path
    _need(replay_path.is_file() and not replay_path.is_symlink() and replay_path.stat().st_size <= 512*1024 and
          hashlib.sha256(replay_path.read_bytes()).hexdigest() == _hash(replay_source['sha256'], 'replay source'),
          'Mix step replay source differs from the reviewed file-only program')
    _need(sources.get('target_sequence_sha256') == target_sha and sources.get('replay_input_report_sha256') == report_sha and
          all(sources.get(k) is True for k in ('model_values_unchanged', 'replay_uses_original_inputs',
              'recorded_feedback_not_new_mix_feedback')) and
          all(sources.get(k) is False for k in ('closed_loop_prediction', 'standing_prediction', 'output_allowed')),
          'Mix step source review must preserve original-input and no-prediction limitations')
    target_rows = targets.get('rows')
    _need(type(target_rows) is list and len(target_rows) == count, 'Mix step replay omitted or added model calls')
    initial = [origin[mid] for mid in IDS]
    peaks = [0.]*12
    for index, row in enumerate(target_rows):
        _need(type(row) is dict and type(row.get('cycle_index')) is int and row['cycle_index'] == rows[index]['index'] and
              type(row.get('raw_target_model_rad')) is list and len(row['raw_target_model_rad']) == 12,
              'Mix step target sequence differs from the original model-call prefix')
        for order, mid in enumerate(IDS):
            model_index = shadow.CAN_ORDER.index(int(mid))
            raw = _number(row['raw_target_model_rad'][order], 'replayed full target ID'+mid,
                          shadow.LOWER[model_index], shadow.UPPER[model_index])
            target = initial[order]+.1*(raw-initial[order])
            axis = data['axes'][mid]
            _need(axis['lower_rad'] <= target <= axis['upper_rad'] and
                  abs(target-initial[order]) <= axis['max_displacement_from_start_rad'],
                  'Ten-percent replayed target exceeds the reviewed physical/displacement envelope: ID'+mid)
            peaks[order] = max(peaks[order], abs(math.degrees(target-initial[order])))
    _need(analysis.get('schema') == 'singularitydog.saved-policy-target-mixture-analysis.v1' and
          analysis.get('input_sha256') == {'report': report_sha, 'targets': target_sha} and
          analysis.get('id_order') == list(range(1, 13)) and
          all(type(mid) is int for mid in analysis['id_order']) and type(analysis.get('target_rows')) is int and analysis['target_rows'] == count and
          analysis.get('initial_pose_source') == 'report_trial_origin_model_rad_by_id' and
          analysis.get('initial_q_model_rad_by_id') == initial and
          analysis.get('sequence_extent') == 'logged_model_call_count_matches' and
          analysis.get('first_cycle_index') == rows[0]['index'] and analysis.get('last_cycle_index') == rows[count-1]['index'] and
          analysis.get('cap_deg') == {'physical_clearance': [7.]*12, 'maximum_displacement': [6.]*12} and
          all(analysis.get(k) is False for k in ('physical_clearance_independently_verified',
              'model_replay_verified_by_this_tool', 'closed_loop_prediction', 'standing_prediction',
              'timestamps_predict_future_motion', 'hardware_accessed', 'model_executed', 'approvals_created', 'output_allowed')),
          'Mix step analysis must preserve exact inputs, numerical caps and limited algebraic scope')
    matches = [item for item in analysis.get('mixtures', []) if type(item) is dict and item.get('weight') == .1]
    _need(len(matches) == 1 and matches[0].get('output_allowed') is False and
          matches[0].get('physical_clearance_exceeded_ids') == [] and matches[0].get('maximum_displacement_exceeded_ids') == [] and
          type(matches[0].get('per_axis')) is list and len(matches[0]['per_axis']) == 12,
          'Ten-percent analysis reports an exceeded or incomplete numerical envelope')
    for order, axis in enumerate(matches[0]['per_axis']):
        _need(type(axis.get('id')) is int and axis['id'] == order+1 and type(axis.get('peak_absolute_delta_deg')) in (int, float) and
              abs(axis['peak_absolute_delta_deg']-peaks[order]) <= 1e-10 and
              axis.get('exceeds_supplied_physical_clearance') is False and
              axis.get('exceeds_supplied_maximum_displacement') is False,
              'Ten-percent analysis extrema differ from independent target recomputation')
    _need(type(matches[0].get('maximum_absolute_delta_deg')) in (int, float) and
          abs(matches[0]['maximum_absolute_delta_deg']-max(peaks)) <= 1e-10,
          'Ten-percent analysis maximum differs from the raw target sequence')
    clearance = documents['mix_step_clearance']
    expected = dict(schema='singularitydog.supported-mix-step-clearance.v1', mode=SUPPORTED_POLICY_MIX_STEP_10PCT,
        scope=data['scope'], boot_id=data['boot_id'], motor_power_epoch=data['motor_power_epoch'],
        capture_sha256=data['artifacts']['local_reference_capture']['sha256'],
        uids_by_id={mid: data['axes'][mid]['uid'] for mid in IDS},
        reference_turns_by_id=documents['hardware_review']['local_characterization']['reference_turns_by_id'],
        local_clearance_rad=math.radians(7))
    _need(all(clearance.get(k) == v for k, v in expected.items()) and
          all(clearance.get(k) is True for k in ('support_must_remain', 'four_paws_floor', 'hands_off',
              'cutoff_ready', 'current_pose_unchanged', 'current_power_unchanged')) and
          all(clearance.get(k) is False for k in ('box_removal_allowed', 'standing_allowed',
              'walking_allowed', 'load_bearing_verified')), 'Mix step requires current seven-degree operator clearance and support')
    source_statement, _ = _artifact(clearance.get('source_receipt'), base)
    _supported_mix_step_operator_receipt(source_statement, data)
    _need(clearance.get('user_statement') == source_statement['answer'],
          'Mix step clearance must retain the original pinned operator statement')
    _text(clearance.get('user_statement'), 'current seven-degree operator statement')
    _review(clearance.get('review'), 'ACCEPT_CURRENT_7DEG_SUPPORTED_MIX_STEP_CLEARANCE')
    acceptance = documents['hardware_review'].get('supported_mix_step_acceptance', {})
    refs = dict(prior_profile_sha256=profile_sha, prior_report_sha256=report_sha,
        prior_observation_sha256=data['artifacts']['prior_supported_observation']['sha256'],
        target_sequence_sha256=target_sha, mixture_analysis_sha256=data['artifacts']['policy_mixture_analysis']['sha256'],
        clearance_sha256=data['artifacts']['mix_step_clearance']['sha256'],
        source_review_sha256=data['artifacts']['mix_step_source_review']['sha256'],
        capture_sha256=data['artifacts']['local_reference_capture']['sha256'],
        diagnostic_sha256=data['artifacts']['pipeline_diagnostic']['sha256'])
    _need(acceptance.get('mode') == SUPPORTED_POLICY_MIX_STEP_10PCT and acceptance.get('scope') == data['scope'] and
          all(acceptance.get(k) == v for k, v in refs.items()) and acceptance.get('support_must_remain') is True and
          acceptance.get('load_bearing_not_established') is True and
          all(acceptance.get(k) is False for k in ('box_removal_allowed', 'standing_allowed', 'walking_allowed')),
          'Explicit hash-bound five-second ten-percent boxed review required')
    _review(acceptance.get('review'), 'ACCEPT_5S_SUPPORTED_LEARNED_MIX_STEP_10PCT')


def _supported_gain_step_evidence(documents, data):
    """Bind a three-second gain step to a completed learned run and fresh inputs.

    The predecessor proves only that learned output and STOP completed under
    support. It does not prove load bearing, ground clearance, or walking.
    """
    prior = documents['prior_supported_profile']
    report = documents['prior_supported_report']
    observed = documents['prior_supported_observation']
    _structure(prior)
    _need(prior['approved_for_supported_policy_output'] is True and
          prior['blockers'] == [] and
          prior['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER and
          prior['duration_s'] == 2. and prior['policy_weight'] == .005,
          'Gain step requires the reviewed 0.5-percent two-second predecessor')
    _review(prior['review'], 'APPROVED_SUPPORTED_CHARACTERIZATION')
    _need(data['schema'] == SCHEMA_V3 and
          data['scope'] == prior['scope'] == 'supported_characterization_only' and
          data['policy_weight'] <= 2*prior['policy_weight'] and
          data['h_hypothesis'] == prior['h_hypothesis'] and
          data['command'] == prior['command'] and
          data['request_gap_us'] == prior['request_gap_us'] and
          data['request_window'] == prior['request_window'] and
          data['model_backend'] == prior['model_backend'] and
          data['native_batch_encoder'] == prior['native_batch_encoder'] and
          data.get('post_reply_deadline_policy') == prior.get('post_reply_deadline_policy') and
          data.get('startup_cycle_allowance') == prior.get('startup_cycle_allowance') and
          data.get('voltage_pipeline') == prior.get('voltage_pipeline') and
          data.get('voltage_overlap') == prior.get('voltage_overlap'),
          'Gain step may change only the bounded learned mixture and gain schedule')
    for name in ('calibration', 'mount', 'bias', 'model_manifest',
                 'scalar_step_manifest'):
        _need(data['artifacts'][name]['sha256'] == prior['artifacts'][name]['sha256'],
              'Gain step changes pinned model or calibration: '+name)
    for source, digest in prior['cadence_source_sha256'].items():
        if source != 'singularitydog_hw/policy_live_profile.py':
            _need(data['cadence_source_sha256'][source] == digest,
                  'Gain step changes active runtime or transport source')
    for mid in IDS:
        old, new = prior['axes'][mid], data['axes'][mid]
        for key in ('uid', 'sign', 'offset_rad', 'uncertainty_rad', 'kd',
                    'max_command_velocity_rad_s', 'max_command_acceleration_rad_s2',
                    'max_tracking_error_rad', 'max_measured_velocity_rad_s',
                    'max_measured_torque_nm', 'max_temperature_c',
                    'max_displacement_from_start_rad'):
            _need(new[key] == old[key], 'Gain step changes protected axis setting: ID'+mid+' '+key)
        _need(old['kp'] == 3. and 3. < new['kp'] <= 12. and
              old['max_estimated_pd_torque_nm'] == .1 and
              .1 < new['max_estimated_pd_torque_nm'] <= .5,
              'Gain step requires an explicit bounded increase on every axis')
    _need(report.get('profile_sha256') ==
          data['artifacts']['prior_supported_profile']['sha256'] and
          report.get('boot_id') == prior['boot_id'] and
          report.get('motor_power_epoch') == prior['motor_power_epoch'] and
          report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and
          report.get('errors') == [] and
          report.get('motor_enable_sent') is True and
          report.get('learned_targets_sent') is True and
          type(report.get('actual_model_calls')) is int and
          report['actual_model_calls'] > 0 and
          report.get('normal_ramp_completed') is True and
          report.get('stop_confirmed') is True and
          report.get('post_reply_deadline_rejections') == [] and
          report.get('post_reply_deadline_allowance_uses') == 0 and
          report.get('steady_deadline20ms_misses') == 0 and
          report.get('startup_20ms_allowance_uses') in (0, 1) and
          report.get('deadline20ms_misses') == report.get('startup_20ms_allowance_uses'),
          'Gain step predecessor must have completed learned output and all deadlines')
    rows = report.get('cycles')
    _need(type(rows) is list and 80 <= len(rows) <= 102 and
          rows[0].get('phase') == 'starting' and
          rows[-1].get('phase') == 'stopped' and
          any(row.get('phase') == 'active' and
              row.get('effective_policy_weight') == prior['policy_weight']
              for row in rows),
          'Gain step predecessor lacks a full learned ramp and STOP')
    previous_end = None
    for index, row in enumerate(rows):
        begin = row.get('begin_ns')
        replied = row.get('output_reply_end_ns')
        end = row.get('end_ns')
        allowed_ns = 21_000_000 if index == 0 and \
            report['startup_20ms_allowance_uses'] else 20_000_000
        _need(row.get('index') == index and
              all(type(v) is int for v in (begin, replied, end)) and
              0 < begin <= replied <= end and
              end-begin <= allowed_ns and
              replied-begin <= 20_000_000 and
              (previous_end is None or begin >= previous_end) and
              row.get('post_reply_deadline', {}).get('accepted') is True,
              'Gain step predecessor cycle or reply deadline is invalid')
        previous_end = end
    stops = report.get('stop_reports', {})
    for scope, ids in (('front', list(range(1, 7))),
                       ('rear', list(range(7, 13)))):
        stop = stops.get(scope, {})
        _need(stop.get('complete') is True and
              stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and
              stop.get('ambiguous_ids') == [] and
              stop.get('fault_by_id') == {str(mid): 0 for mid in ids},
              'Gain step predecessor STOP is incomplete or faulted')
    _need(observed.get('report_sha256') ==
          data['artifacts']['prior_supported_report']['sha256'] and
          observed.get('observed_by') == 'operator' and
          observed.get('audio_heard') is True and
          observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          observed.get('box_support_maintained') is True and
          observed.get('autonomous_standing_or_walking_observed') is False,
          'Gain step requires the actual operator observation')
    _text(observed.get('user_statement'), 'gain step predecessor observation')
    acceptance = documents['hardware_review'].get('supported_gain_step_acceptance', {})
    _need(acceptance.get('mode') == SUPPORTED_POLICY_GAIN_STEP_3S and
          acceptance.get('scope') == data['scope'] and
          acceptance.get('prior_profile_sha256') ==
              data['artifacts']['prior_supported_profile']['sha256'] and
          acceptance.get('prior_report_sha256') ==
              data['artifacts']['prior_supported_report']['sha256'] and
          acceptance.get('prior_observation_sha256') ==
              data['artifacts']['prior_supported_observation']['sha256'] and
          acceptance.get('support_must_remain') is True and
          acceptance.get('load_bearing_not_established') is True and
          acceptance.get('walking_allowed') is False,
          'Exact prior evidence and supported-only gain-step review required')
    _review(acceptance.get('review'), 'ACCEPT_3S_SUPPORTED_LEARNED_GAIN_STEP')


def _current_hold_after_supported_evidence(documents, data):
    """Admit an existing <=3s hold using separately pinned supported evidence.

    A completed learned run is timing evidence, never a weight-bearing test.
    Keep historical bytes intact; only the admission loader may have changed.
    The new hold has no first-cycle or post-reply live deadline allowance.
    """
    prior = documents['prior_supported_profile']
    report = documents['prior_supported_report']
    observed = documents['prior_supported_observation']
    _structure(prior)
    _need(prior['approved_for_supported_policy_output'] is True and prior['blockers'] == [] and
          prior.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_10S_AFTER_2S and
          prior['duration_s'] == 10.,
          'Current hold requires an approved ten-second supported predecessor')
    _review(prior['review'], 'APPROVED_SUPPORTED_CHARACTERIZATION')
    hypothesis_hold = (accel_input_hypothesis_selected(data) and
                       accel_input_hypothesis_selected(prior))
    replaceable_artifacts = (*_EXTENSION_ARTIFACTS, 'hardware_review',
                             'operator_acceptance', 'local_reference_capture')
    if hypothesis_hold:
        replaceable_artifacts += ('pipeline_diagnostic',)

    def contract(value):
        omitted = {'artifacts', 'review', 'blockers', 'approved_for_supported_policy_output',
                   'assembly_id', 'bundle_path', 'duration_s', 'diagnostic_timing_acceptance',
                   'start_pose_bounds', 'axes', 'cadence_source_sha256', 'policy_weight',
                   'startup_duration_s', 'startup_damping_duration_s', 'startup_cycle_allowance',
                   'post_reply_deadline_policy'}
        result = {k:v for k,v in value.items() if k not in omitted}
        result['axes'] = {mid:{k:v for k,v in axis.items() if k not in (
            'physical_lower_rad', 'physical_upper_rad', 'kp', 'max_estimated_pd_torque_nm',
            'max_measured_velocity_rad_s')} for mid,axis in value['axes'].items()}
        result['sources'] = {k:v for k,v in value['cadence_source_sha256'].items()
                            if k != 'singularitydog_hw/policy_live_profile.py'}
        result['artifacts'] = {k:v['sha256'] for k,v in value['artifacts'].items()
            if k not in replaceable_artifacts}
        return result

    _need(contract(prior) == contract(data),
          'Current hold changes predecessor execution, model, calibration, UID, boot, power or safety contract')
    # Validate historical numerical caps against the current unchanged runtime,
    # without recursively loading prior review files or modifying the evidence.
    checked_prior = copy.deepcopy(prior)
    checked_prior['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'] = (
        data['cadence_source_sha256']['singularitydog_hw/policy_live_profile.py'])
    _settings(checked_prior)
    _axes(checked_prior, documents['calibration'])
    _need(all(data['axes'][mid]['max_measured_velocity_rad_s'] <=
              prior['axes'][mid]['max_measured_velocity_rad_s'] for mid in IDS),
          'Current hold cannot loosen the predecessor velocity monitor')
    profile_sha = data['artifacts']['prior_supported_profile']['sha256']
    report_sha = data['artifacts']['prior_supported_report']['sha256']
    _need(type(report) is dict and report.get('profile_sha256') == profile_sha and
          report.get('boot_id') == data['boot_id'] and
          report.get('motor_power_epoch') == data['motor_power_epoch'] and
          report.get('cadence_source_sha256') == prior['cadence_source_sha256'] and
          report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == [] and
          report.get('scope') == data['scope'] and report.get('normal_ramp_completed') is True and
          report.get('learned_targets_sent') is True and report.get('stop_confirmed') is True and
          report.get('current_position_hold_only') is False and report.get('cyclic_inference_skipped') is False and
          report.get('post_reply_deadline_rejections') == [] and
          report.get('trial_displacement_origin') == 'final_pre_enable_feedback',
          'Current hold requires successful same-session learned output and normal ramp/STOP')
    native = report.get('native_batch_encoder', {})
    provenance = report.get('model_provenance', {})
    _need(type(prior.get('native_batch_encoder')) is dict and native.get('enabled') is True and
          native.get('binary_sha256') == prior['native_batch_encoder']['sha256'] and
          report.get('execution_settings') == execution_settings(prior) and
          provenance.get('manifest_sha256') == data['artifacts']['scalar_step_manifest']['sha256'] and
          provenance.get('baseline_provenance', {}).get('manifest_sha256') ==
              data['artifacts']['model_manifest']['sha256'] and
          report.get('transport_settings', {}).get('request_gap_us') == data['request_gap_us'] and
          report.get('transport_settings', {}).get('request_window') == data['request_window'],
          'Current hold predecessor encoder/model/backend/pacing differs')
    for scope, ids in (('front', list(range(1,7))), ('rear', list(range(7,13)))):
        stop = report.get('stop_reports', {}).get(scope, {})
        _need(stop.get('complete') is True and stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
              stop.get('fault_by_id') == {str(mid):0 for mid in ids},
              'Current hold predecessor STOP evidence incomplete or faulted')
    rows = report.get('cycles')
    _need(type(rows) is list and 475 <= len(rows) <= 502 and
          type(report.get('actual_model_calls')) is int and 400 <= report['actual_model_calls'] <= len(rows),
          'Current hold requires completed ten-second learned cycles')
    if hypothesis_hold:
        # The new no-output measurement has passed _timing with the current
        # exact source/input pins. Every prior active and stopping cycle must
        # retain that same non-certifying correction, not merely its profile flag.
        accel_provenance = documents['pipeline_diagnostic'].get('observer', {}).get('accel_input_hypothesis')
        _need(type(accel_provenance) is dict and
              accel_provenance.get('hypothesis_sha256') == data['artifacts']['accel_input_hypothesis']['sha256'] and
              accel_provenance.get('formal_calibration_approved') is False and
              accel_provenance.get('grants_motor_output') is False,
              'Hypothesis current hold requires the same unapproved prior input')
        if (prior['artifacts']['pipeline_diagnostic']['sha256'] !=
                data['artifacts']['pipeline_diagnostic']['sha256']):
            first = documents['pipeline_diagnostic'].get('measurements', [{}])[0].get('release_ns')
            last = rows[-1].get('end_ns')
            _need(type(first) is int and type(last) is int and first > last,
                  'Fresh hypothesis current-hold diagnostic must follow the completed ten-second run')
    prior_post_reply = _post_reply_policy(prior)
    _need(prior_post_reply is not None and report.get('post_reply_deadline_policy') == prior_post_reply,
          'Current hold predecessor post-reply policy differs')
    startup_enabled = _startup_cycle_policy(prior) is not None
    _need(report.get('startup_20ms_allowance_enabled', False) is startup_enabled,
          'Current hold predecessor startup policy differs')
    misses, startup_misses, previous_end = [], 0, None
    for index, row in enumerate(rows):
        _need(type(row) is dict and row.get('index') == index, 'Current hold predecessor cycle sequence invalid')
        if hypothesis_hold:
            imu = row.get('imu_body', {})
            _need(type(imu) is dict and imu.get('accel_bias_subtracted') is True and
                  imu.get('accel_scale_corrected') is True and
                  'reviewed_accel_calibration' not in imu and
                  json.dumps(imu.get('accel_input_hypothesis'), sort_keys=True, separators=(',', ':'), allow_nan=False) ==
                  json.dumps(accel_provenance, sort_keys=True, separators=(',', ':'), allow_nan=False),
                  'Hypothesis current hold predecessor actual input provenance differs')
        stamps = [row.get(k) for k in ('begin_ns', 'output_reply_end_ns', 'end_ns')]
        _need(all(type(v) is int and v > 0 for v in stamps) and stamps == sorted(stamps),
              'Current hold predecessor timestamps invalid')
        begin, replied, end = stamps
        missed = end-begin > 20_000_000
        startup_used = index == 0 and startup_enabled and missed
        steady_missed = missed and not startup_used
        if startup_used: startup_misses += 1
        if steady_missed: misses.append(index)
        rolling_misses = sum(i > index-100 for i in misses)
        timing = row.get('post_reply_deadline', {})
        _need(end-begin <= 20_000_000+int(prior_post_reply['max_lateness_ms']*1e6) and
              replied-begin <= 20_000_000 and (previous_end is None or begin >= previous_end) and
              (index == 0 or begin-rows[index-1]['begin_ns'] <= 21_000_000) and
              rolling_misses <= 1 and
              timing.get('accepted') is True and timing.get('checked_ns') == end and
              timing.get('allowance_used') is steady_missed and
              timing.get('startup_allowance_used') is startup_used and
              timing.get('rolling_misses') == rolling_misses and
              timing.get('consecutive_misses') == int(steady_missed) and
              row.get('deadline20ms_missed') is missed and
              row.get('steady_deadline20ms_missed') is steady_missed and
              row.get('startup_20ms_allowance_used') is startup_used,
              'Current hold predecessor exceeds its original bounded live timing policy')
        previous_end = end
    for key, count in (('deadline20ms_misses', len(misses)+startup_misses),
                       ('steady_deadline20ms_misses', len(misses)),
                       ('post_reply_deadline_allowance_uses', len(misses)),
                       ('startup_20ms_allowance_uses', startup_misses)):
        _need(type(report.get(key)) is int and report[key] == count,
              'Current hold predecessor deadline counts differ')
    _need(rows[0].get('phase') == 'starting' and rows[-1].get('phase') == 'stopped' and
          9_500_000_000 <= rows[-1]['end_ns']-rows[0]['begin_ns'] <= 10_040_000_000 and
          any(row.get('phase') == 'active' and row.get('effective_policy_weight') == prior['policy_weight']
              for row in rows), 'Current hold predecessor did not complete its learned ramp and stop')
    _need(type(observed) is dict and observed.get('report_sha256') == report_sha and
          observed.get('observed_by') == 'operator' and observed.get('audio_heard') is True and
          observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          observed.get('box_support_maintained') is True and
          observed.get('autonomous_standing_or_walking_observed') is False,
          'Current hold requires a matching operator observation of the supported run')
    _text(observed.get('user_statement'), 'predecessor physical observation')
    acceptance = documents['hardware_review'].get('current_hold_after_supported_acceptance', {})
    _need(type(acceptance) is dict and acceptance.get('mode') == CURRENT_HOLD_AFTER_SUPPORTED_10S and
          acceptance.get('scope') == data['scope'] and acceptance.get('strict_50hz_not_established') is True and
          acceptance.get('support_must_remain') is True and acceptance.get('load_bearing_not_established') is True and
          acceptance.get('strict_current_hold_deadline') is True and
          acceptance.get('diagnostic_sha256') == data['artifacts']['pipeline_diagnostic']['sha256'] and
          acceptance.get('prior_profile_sha256') == profile_sha and acceptance.get('prior_report_sha256') == report_sha and
          acceptance.get('prior_observation_sha256') == data['artifacts']['prior_supported_observation']['sha256'],
          'Explicit hash-bound current hold after supported review required')
    _review(acceptance.get('review'), 'ACCEPT_CURRENT_HOLD_AFTER_SUPPORTED_10S')


def _fixed_catch_evidence(documents, data, base):
    """Bind the fixed catch run to the completed, supported three-second hold.

    The predecessor is evidence, not a loader-approved 30-second permission.
    Any changed executable source must be separately named and hash reviewed.
    """
    prior = documents['prior_current_hold_profile']
    report = documents['prior_current_hold_report']
    observed = documents['prior_current_hold_observation']
    source_review = documents['fixed_catch_source_review']
    _need(type(prior) is dict and prior.get('schema') == SCHEMA_V3 and
          prior.get('scope') == 'supported_characterization_only' and
          prior.get('diagnostic_timing_acceptance') == CURRENT_HOLD_AFTER_SUPPORTED_10S and
          prior.get('duration_s') == 3 and prior.get('policy_weight') == 0 and
          prior.get('approved_for_supported_policy_output') is True and prior.get('blockers') == [] and
          prior.get('boot_id') == data['boot_id'] and
          prior.get('motor_power_epoch') == data['motor_power_epoch'],
          'Thirty-second fixed catch requires same-session reviewed three-second hold')
    _review(prior.get('review'), 'APPROVED_SUPPORTED_CHARACTERIZATION')
    for key in ('bundle_path', 'period_ms', 'hard_cycle_ms',
                'max_consecutive_20ms_misses', 'max_sample_age_ms', 'max_sample_gap_ms',
                'voltage_min_v', 'voltage_max_v', 'imu_tilt_limit_rad', 'imu_gyro_limit_rad_s',
                'imu_accel_norm_min_m_s2', 'imu_accel_norm_max_m_s2', 'startup_duration_s',
                'stop_duration_s', 'policy_ramp_s', 'policy_weight', 'h_hypothesis',
                'command', 'request_gap_us', 'request_window', 'telemetry_cadence',
                'model_backend', 'voltage_overlap', 'voltage_pipeline', 'native_batch_encoder',
                'watchdog_review_policy', 'local_characterization'):
        _need(prior.get(key) == data.get(key), 'Fixed catch changes predecessor setting: '+key)
    for mid in IDS:
        old_axis, new_axis = prior['axes'][mid], data['axes'][mid]
        for key in AXIS_KEYS - {'physical_lower_rad', 'physical_upper_rad'}:
            _need(old_axis[key] == new_axis[key],
                  'Fixed catch changes predecessor gain/limit/calibration: ID'+mid+' '+key)
    prior_artifacts = prior.get('artifacts', {})
    for key, reference in prior_artifacts.items():
        if key not in ('hardware_review', 'operator_acceptance', 'local_reference_capture',
                       'prior_supported_profile', 'prior_supported_report',
                       'prior_supported_observation'):
            _need(data['artifacts'].get(key, {}).get('sha256') == reference['sha256'],
                  'Fixed catch changes predecessor evidence: '+key)
    old_hashes = prior.get('cadence_source_sha256')
    new_hashes = data.get('cadence_source_sha256')
    _need(type(old_hashes) is dict and type(new_hashes) is dict and
          set(old_hashes) == set(new_hashes) == set(CADENCE_SOURCE_PATHS),
          'Fixed catch requires matching pinned source set')
    changed = {path for path in old_hashes if old_hashes[path] != new_hashes[path]}
    _need(changed <= _FIXED_CATCH_CHANGED_SOURCES - {_FIXED_CATCH_NEW_SOURCE} and
          'singularitydog_hw/policy_live_profile.py' in changed,
          'Unreviewed predecessor executable source changed')
    _need(type(source_review) is dict and
          source_review.get('schema') == 'singularitydog.fixed-catch-source-review.v1' and
          source_review.get('prior_profile_sha256') ==
              data['artifacts']['prior_current_hold_profile']['sha256'] and
          source_review.get('changes') == [
              {'path': path, 'before_sha256': old_hashes[path], 'after_sha256': new_hashes[path]}
              for path in sorted(changed)] and
          source_review.get('new_source') == {'path': _FIXED_CATCH_NEW_SOURCE,
              'sha256': hashlib.sha256((base/'runtime'/_FIXED_CATCH_NEW_SOURCE).read_bytes()).hexdigest()},
          'Fixed catch source delta differs from the exact reviewed hashes')
    _review(source_review.get('review'), 'ACCEPT_FIXED_CATCH_SOURCE_DELTA')
    report_sha = data['artifacts']['prior_current_hold_report']['sha256']
    _need(type(report) is dict and report.get('profile_sha256') ==
              data['artifacts']['prior_current_hold_profile']['sha256'] and
          report.get('boot_id') == data['boot_id'] and
          report.get('motor_power_epoch') == data['motor_power_epoch'] and
          report.get('cadence_source_sha256') == old_hashes and
          report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == [] and
          report.get('normal_ramp_completed') is True and report.get('stop_confirmed') is True and
          report.get('current_position_hold_only') is True and
          report.get('learned_targets_sent') is False and report.get('actual_model_calls') == 0 and
          report.get('deadline20ms_misses') == 0 and report.get('startup_20ms_misses') == 0 and
          report.get('steady_deadline20ms_misses') == 0 and
          report.get('trial_displacement_origin') == 'final_pre_enable_feedback',
          'Fixed catch predecessor must be a complete strict current-position hold')
    rows = report.get('cycles')
    _need(type(rows) is list and 125 <= len(rows) <= 151 and
          rows[0].get('phase') == 'starting' and rows[-1].get('phase') == 'stopped',
          'Fixed catch predecessor three-second sequence is incomplete')
    for index, row in enumerate(rows):
        _need(row.get('index') == index and type(row.get('begin_ns')) is int and
              type(row.get('end_ns')) is int and row['begin_ns'] < row['end_ns'] and
              row['end_ns'] - row['begin_ns'] <= 20_000_000 and
              row.get('deadline20ms_missed') is False,
              'Fixed catch predecessor cycle exceeded strict20ms')
    for scope, ids in (('front', list(range(1,7))), ('rear', list(range(7,13)))):
        stop = report.get('stop_reports', {}).get(scope, {})
        _need(stop.get('complete') is True and stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
              stop.get('fault_by_id') == {str(mid):0 for mid in ids},
              'Fixed catch predecessor lacks all-axis fault-free STOP')
    _need(type(observed) is dict and observed.get('report_sha256') == report_sha and
          observed.get('observed_by') == 'operator' and observed.get('audio_heard') is True and
          observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          observed.get('box_support_maintained') is True and
          observed.get('autonomous_standing_or_walking_observed') is False,
          'Fixed catch requires matching no-anomaly operator observation')
    acceptance = documents['hardware_review'].get('fixed_catch_acceptance', {})
    _need(type(acceptance) is dict and acceptance.get('mode') == FIXED_CATCH_CURRENT_HOLD_30S and
          acceptance.get('scope') == FIXED_CATCH_SCOPE and
          acceptance.get('settings') == data['fixed_catch'] and
          acceptance.get('prior_profile_sha256') ==
              data['artifacts']['prior_current_hold_profile']['sha256'] and
          acceptance.get('prior_report_sha256') == report_sha and
          acceptance.get('prior_observation_sha256') ==
              data['artifacts']['prior_current_hold_observation']['sha256'] and
          acceptance.get('strict_current_hold_deadline') is True and
          acceptance.get('load_bearing_not_yet_observed') is True and
          acceptance.get('walking_allowed') is False,
          'Explicit 30-second fixed-catch engineering review required')
    _review(acceptance.get('review'), 'ACCEPT_FIXED_CATCH_CURRENT_HOLD_30S')


def _human_supported_audio_manifest(document, data, base):
    """Pin real WAV bytes and duration; a process exit never proves audibility."""
    _need(type(document) is dict and
          document.get('schema') == 'singularitydog.human-supported-audio-manifest.v1' and
          document.get('scope') == HUMAN_SUPPORTED_PARTIAL_SCOPE and
          document.get('acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S and
          document.get('prepare_ease_is_not_go') is True and
          document.get('go_is_short_tone') is True and
          document.get('resupport_starts_with_urgent_tone') is True and
          document.get('operator_must_resupport_before_voice_finishes') is True and
          document.get('audio_process_completion_is_not_proof_of_audibility') is True and
          document.get('physical_ease_duration_not_verified_by_audio') is True,
          'Explicit stage meaning and audibility limitations are required for human-supported audio')
    _review(document.get('review'), 'ACCEPT_HUMAN_SUPPORTED_SPOKEN_CUES')
    clips = document.get('clips')
    stages = ('brief','prepare_ease','go','resupport','abort')
    _need(type(clips) is dict and set(clips) == set(stages), 'Five pinned human-supported audio stages required')
    resolved = {}
    for stage in stages:
        clip = clips[stage]
        _need(type(clip) is dict and set(clip) == {'path','sha256','duration_s','transcript'},
              'Audio stage requires exact file, duration and reviewed transcript: '+stage)
        name = _text(clip['path'], 'audio file '+stage)
        _text(clip['transcript'], 'audio meaning '+stage)
        digest = _hash(clip['sha256'], 'audio stage '+stage)
        path = Path(name).expanduser()
        if not path.is_absolute(): path = base/path
        _need(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 16*1024*1024,
              'Regular bounded audio file required: '+stage)
        raw = path.read_bytes()
        _need(hashlib.sha256(raw).hexdigest() == digest, 'Audio file SHA256 mismatch: '+stage)
        # Parse the exact hashed bytes, avoiding a second file-open race.
        import io
        try:
            with wave.open(io.BytesIO(raw), 'rb') as recording:
                _need(recording.getcomptype() == 'NONE' and recording.getnchannels() == 2 and
                      recording.getsampwidth() == 2 and recording.getframerate() == 48000,
                      'Audio must be 48kHz stereo PCM16 WAV: '+stage)
                frames = recording.getnframes()
                _need(frames > 0 and len(recording.readframes(frames)) == frames*4,
                      'Truncated human-supported audio: '+stage)
                actual_duration = frames/48000.
        except (wave.Error, EOFError) as error:
            raise ProfileError('Invalid human-supported WAV: '+stage) from error
        duration = _number(clip['duration_s'], 'audio duration '+stage, 0, 30, positive=True)
        _need(abs(duration-actual_duration) <= 1e-9, 'Audio duration differs from hashed WAV: '+stage)
        if stage == 'go':
            _need(duration <= .12, 'Human-supported go tone must be at most120ms')
        resolved[stage] = dict(path=str(path.absolute()), sha256=digest, duration_s=actual_duration)
    settings = data['human_supported_hold']
    # Both spoken completion gates get a real-duration+.25s timeout. Fresh
    # feedback after each completion, ease, ACK and fading remain budgeted.
    brake_s = max(axis['max_command_velocity_rad_s']/axis['max_command_acceleration_rad_s2']
                  for axis in data['axes'].values())
    reserve = (settings['cue_not_before_s'] + resolved['prepare_ease']['duration_s']+.25 +
        resolved['go']['duration_s']+.25 + settings['slight_ease_max_duration_s'] +
        resolved['resupport']['duration_s']+.25 + settings['resupport_ack_window_s'] +
        brake_s + data['stop_duration_s'] + .06)
    _need(reserve < data['duration_s'], 'Spoken cues/ease/ACK/fade do not fit the bounded eight-second hold')
    return dict(clips=resolved, cue_reserve_s=reserve,
                audio_completion_does_not_prove_audibility=True,
                physical_ease_duration_not_verified_by_audio=True)


def _human_supported_evidence(documents, data, base):
    """Fresh human pose/session proof; previous box output is supplemental only.

    Named source receipts record operator statements, never manufacture their
    physical truth. No fixed receiver, unsupported stance or walking is granted.
    """
    settings = _human_supported_settings(data)
    prep = documents['human_supported_preparation']
    uids = {mid:data['axes'][mid]['uid'] for mid in IDS}
    capture_pin = data['artifacts']['local_reference_capture']['sha256']
    watchdog_pin = data['artifacts']['command_loss_report']['sha256']
    diagnostic_pin = data['artifacts']['pipeline_diagnostic']['sha256']
    _need(type(prep) is dict and prep.get('schema') == 'singularitydog.human-supported-preparation.v1' and
          prep.get('scope') == HUMAN_SUPPORTED_PARTIAL_SCOPE and prep.get('settings') == settings and
          prep.get('boot_id') == data['boot_id'] and prep.get('motor_power_epoch') == data['motor_power_epoch'] and
          prep.get('uids_by_id') == uids and
          prep.get('local_reference_capture_sha256') == capture_pin and
          prep.get('command_loss_report_sha256') == watchdog_pin and
          prep.get('pipeline_diagnostic_sha256') == diagnostic_pin and
          prep.get('reviewed_settings_sha256') == reviewed_settings_sha256(data) and
          prep.get('fixed_catch_authorized') is False and prep.get('ground_progression_allowed') is False and
          prep.get('absolute_calibration_certified') is False and prep.get('dynamic_feedback_certified') is False,
          'Fresh human-supported preparation must pin exact pose, power, diagnostic and bounded settings')
    _review(prep.get('review'), 'ACCEPT_HUMAN_SUPPORTED_PARTIAL_PREPARATION')
    receipt_keys = ('power', 'pose', 'rehearsal', 'video', 'clearance', 'physical_observation')
    references = prep.get('source_receipts')
    _need(type(references) is dict and set(references) == set(receipt_keys),
          'Human-supported preparation needs every pinned physical source receipt')
    receipts = {}
    for kind in receipt_keys:
        receipt, _ = _artifact(references[kind], base)
        _need(type(receipt) is dict and
              receipt.get('schema') == 'singularitydog.human-supported-operator-receipt.v1' and
              receipt.get('kind') == kind and receipt.get('observed_by') == 'operator',
              'Explicit operator source receipt required: '+kind)
        _text(receipt.get('source_message_id'), 'operator source message '+kind)
        _text(receipt.get('user_statement'), 'operator statement '+kind)
        _review(receipt.get('review'), 'ACCEPT_HUMAN_SUPPORTED_OPERATOR_RECEIPT')
        if kind in ('power', 'pose', 'clearance', 'physical_observation'):
            _need(receipt.get('boot_id') == data['boot_id'] and
                  receipt.get('motor_power_epoch') == data['motor_power_epoch'],
                  'Human-supported source receipt is from another boot/power epoch: '+kind)
        receipts[kind] = receipt
    power = receipts['power']
    _need(power.get('power_epoch_origin') == 'operator_statement' and
          power.get('off_on_confirmed') is True and power.get('motor_power_on') is True and
          power.get('no_power_operation_since_capture') is True,
          'Human-supported power generation must be operator confirmed, never inferred from boot')
    pose = receipts['pose']
    _need(pose.get('capture_sha256') == capture_pin and pose.get('pose_kind') == 'human_full_support' and
          pose.get('box_removed') is True and pose.get('operator_count') == 2 and
          type(pose.get('operator_count')) is int and pose.get('full_body_weight_supported') is True and
          pose.get('all_four_paws_on_floor') is True and pose.get('all_axes_simultaneously_stationary') is True and
          pose.get('continuous_body_catch') is True and pose.get('hands_remain_on_body') is True and
          pose.get('legs_and_wiring_contact_free') is True,
          'A fresh human full-support all-axis pose is required; box poses cannot be substituted')
    rehearsal = receipts['rehearsal']
    _need(type(rehearsal.get('operator_count')) is int and rehearsal['operator_count'] == 2 and
          rehearsal.get('motor_power_off') is True and rehearsal.get('body_full_support_continuous') is True and
          rehearsal.get('box_removed_and_restored') is True and rehearsal.get('cutoff_role_maintained') is True and
          rehearsal.get('abnormal_noise_vibration_slip_sinking_contact') is False,
          'Two-operator off-power remove/restore rehearsal is required')
    video = receipts['video']
    _need(video.get('side_view_recording_ready') is True and video.get('camera_fixed') is True and
          video.get('body_four_paws_and_supporting_hands_visible') is True,
          'Fixed side video must show the body, four paws and continuously supporting hands')
    clearance = receipts['clearance']
    _need(clearance.get('capture_sha256') == capture_pin and clearance.get('selected_ids') == list(range(1,13)) and
          clearance.get('local_clearance_rad') == math.radians(3) and
          clearance.get('legs_and_wiring_contact_free') is True and
          clearance.get('pose_maintained_since_capture') is True and clearance.get('immediate_40v_cutoff_ready') is True,
          'Current human pose requires all-axis local clearance and immediate cutoff')
    physical = receipts['physical_observation']
    _need(physical.get('report_sha256') == watchdog_pin and physical.get('audio_heard') is True and
          physical.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          physical.get('human_full_support_maintained') is True and
          physical.get('pose_maintained_since_capture') is True,
          'Fresh zero-gain observation must retain human full support without anomalies')
    capture = documents['local_reference_capture']
    _need(capture.get('approved_for_runtime') is False and
          capture.get('motor_power_epoch') in (data['motor_power_epoch'], 'NOT_INFERRED_FROM_JETSON_BOOT') and
          capture.get('stop_state') == 'UNVERIFIED_BY_READ_ONLY_PROTOCOL',
          'Read-only capture must preserve unverified STOP and separately bound operator power provenance')
    oldest, newest = [], []
    for mid in IDS:
        identity = capture['identities'][mid]
        row = capture['telemetry']['rows'][mid]
        _need(type(identity.get('request_monotonic_ns')) is int and
              type(identity.get('reply_monotonic_ns')) is int and
              0 < identity['request_monotonic_ns'] <= identity['reply_monotonic_ns'],
              'Human pose identity request/reply causality missing: ID'+mid)
        _need(type(row.get('current')) in (int,float) and row['current'] == 0,
              'Human pose must be captured in current-zero mode: ID'+mid)
        _number(row.get('voltage'), 'human pose voltage ID'+mid, data['voltage_min_v'], data['voltage_max_v'])
        previous = identity['reply_monotonic_ns']
        for sample in row['position_samples']:
            start, end = sample.get('request_monotonic_ns'), sample.get('reply_monotonic_ns')
            _need(type(start) is int and type(end) is int and 0 < previous <= start <= end and
                  end-start <= 30_000_000, 'Human pose position reply causality invalid: ID'+mid)
            oldest.append(start); newest.append(end); previous = end
    _need(max(newest)-min(oldest) <= 2_000_000_000,
          'Human pose all-axis read interval is too long to bind one stationary posture')
    watchdog = documents['command_loss_report']
    for mid in IDS:
        start = watchdog['axes'][mid].get('version', {}).get('request_start_ns')
        end = watchdog['axes'][mid].get('stop_probe', {}).get('received_ns')
        _need(type(start) is int and type(end) is int and max(newest) <= start < end,
              'Fresh human-pose watchdog evidence must follow the capture: ID'+mid)
    diagnostic = documents['pipeline_diagnostic']
    _need(diagnostic['measurements'][0]['release_ns'] >= max(
        watchdog['axes'][mid]['stop_probe']['received_ns'] for mid in IDS),
        'Human-pose diagnostic must follow the fresh all-axis watchdog')
    hardware = documents['hardware_review']
    for mid in IDS:
        dynamic = hardware['type2_dynamic'][mid]
        _need(all(dynamic.get(k) is False for k in ('output_shaft_position_verified',
              'velocity_scale_and_sign_verified', 'torque_interpretation_verified')) and
              data['axes'][mid]['uncertainty_rad'] is None,
              'Human-supported trial must preserve unknown absolute calibration and dynamic scales: ID'+mid)
    prior = documents['prior_current_hold_profile']
    report = documents['prior_current_hold_report']
    observed = documents['prior_current_hold_observation']
    _structure(prior)
    _need(type(prior) is dict and prior.get('schema') == SCHEMA_V3 and
          prior.get('scope') == 'supported_characterization_only' and
          prior.get('diagnostic_timing_acceptance') == CURRENT_HOLD_AFTER_SUPPORTED_10S and
          prior.get('approved_for_supported_policy_output') is True and prior.get('blockers') == [] and
          prior.get('duration_s') == 3 and prior.get('policy_weight') == 0 and
          prior.get('motor_power_epoch') != data['motor_power_epoch'],
          'Prior box hold is supplemental only; a separate human-pose power epoch is required')
    _review(prior.get('review'), 'APPROVED_SUPPORTED_CHARACTERIZATION')
    for key in ('calibration', 'mount', 'bias', 'model_manifest', 'scalar_step_manifest'):
        _need(prior.get('artifacts', {}).get(key, {}).get('sha256') == data['artifacts'][key]['sha256'],
              'Supplemental box hold changes model/calibration provenance: '+key)
    for mid in IDS:
        _need(all(prior.get('axes', {}).get(mid, {}).get(k) == data['axes'][mid][k]
                  for k in AXIS_KEYS-{'physical_lower_rad','physical_upper_rad'}),
              'Human hold must retain supplemental box-hold gains, calibration and monitor caps: ID'+mid)
    old_hashes, new_hashes = prior.get('cadence_source_sha256'), data['cadence_source_sha256']
    _need(type(old_hashes) is dict and set(old_hashes) == set(CADENCE_SOURCE_PATHS) and
          set(new_hashes) == {*CADENCE_SOURCE_PATHS, _HUMAN_SUPPORTED_NEW_SOURCE},
          'Human-supported source set must add only its dedicated supervisor')
    changed = {name for name in old_hashes if old_hashes[name] != new_hashes[name]}
    _need(changed <= _HUMAN_SUPPORTED_CHANGED_SOURCES and
          'singularitydog_hw/policy_live_profile.py' in changed,
          'Unreviewable unrelated source changes in human-supported hold')
    source_review = documents['human_supported_source_review']
    _need(type(source_review) is dict and
          source_review.get('schema') == 'singularitydog.human-supported-source-review.v1' and
          source_review.get('scope') == HUMAN_SUPPORTED_PARTIAL_SCOPE and
          source_review.get('prior_profile_sha256') == data['artifacts']['prior_current_hold_profile']['sha256'] and
          source_review.get('changes') == [{'path':name, 'before_sha256':old_hashes[name],
              'after_sha256':new_hashes[name]} for name in sorted(changed)] and
          source_review.get('new_source') == {'path':_HUMAN_SUPPORTED_NEW_SOURCE,
              'sha256':new_hashes[_HUMAN_SUPPORTED_NEW_SOURCE]},
          'Human-supported source review must bind the exact executable delta')
    _review(source_review.get('review'), 'ACCEPT_HUMAN_SUPPORTED_PARTIAL_SOURCE_DELTA')
    _need(type(report) is dict and report.get('profile_sha256') ==
          data['artifacts']['prior_current_hold_profile']['sha256'] and
          report.get('boot_id') == prior['boot_id'] and report.get('motor_power_epoch') == prior['motor_power_epoch'] and
          report.get('cadence_source_sha256') == old_hashes and
          report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == [] and
          report.get('normal_ramp_completed') is True and report.get('stop_confirmed') is True and
          report.get('current_position_hold_only') is True and report.get('cyclic_inference_skipped') is True and
          report.get('actual_model_calls') == 0 and report.get('learned_targets_sent') is False and
          report.get('deadline20ms_misses') == 0 and report.get('startup_20ms_misses') == 0 and
          report.get('steady_deadline20ms_misses') == 0,
          'Supplemental box current hold must be complete with strict timing and no learned output')
    cycles = report.get('cycles')
    _need(type(cycles) is list and 125 <= len(cycles) <= 151 and
          cycles[0].get('phase') == 'starting' and cycles[-1].get('phase') == 'stopped',
          'Supplemental box three-second hold sequence is incomplete')
    previous = None
    for index, row in enumerate(cycles):
        begin, replied, end = (row.get(k) for k in ('begin_ns','output_reply_end_ns','end_ns'))
        _need(row.get('index') == index and all(type(v) is int for v in (begin,replied,end)) and
              0 < begin <= replied <= end and end-begin <= 20_000_000 and
              (previous is None or begin >= previous) and row.get('deadline20ms_missed') is False,
              'Supplemental box hold timing sequence is invalid')
        previous = end
    for bus, ids in (('front',list(range(1,7))),('rear',list(range(7,13)))):
        stop = report.get('stop_reports', {}).get(bus, {})
        _need(stop.get('complete') is True and stop.get('confirmed_ids') == ids and
              stop.get('unconfirmed_ids') == [] and stop.get('ambiguous_ids') == [] and
              stop.get('fault_by_id') == {str(mid):0 for mid in ids},
              'Supplemental box hold must have all twelve fault-free STOP replies')
    _need(type(observed) is dict and observed.get('report_sha256') == data['artifacts']['prior_current_hold_report']['sha256'] and
          observed.get('observed_by') == 'operator' and observed.get('audio_heard') is True and
          observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
          observed.get('box_support_maintained') is True and
          observed.get('autonomous_standing_or_walking_observed') is False,
          'Supplemental box hold physical observation must remain limited to supported output')
    acceptance = hardware.get('human_supported_partial_acceptance', {})
    _need(type(acceptance) is dict and acceptance.get('mode') == data['diagnostic_timing_acceptance'] and
          acceptance.get('scope') == HUMAN_SUPPORTED_PARTIAL_SCOPE and acceptance.get('settings') == settings and
          acceptance.get('strict_current_hold_deadline') is True and
          acceptance.get('load_bearing_not_yet_observed') is True and
          acceptance.get('fixed_catch_authorized') is False and acceptance.get('ground_progression_allowed') is False and
          acceptance.get('artifact_sha256') == {name:data['artifacts'][name]['sha256'] for name in
              (*_HUMAN_SUPPORTED_ARTIFACTS, 'local_reference_capture','command_loss_report','pipeline_diagnostic')},
          'Dedicated bounded human-supported engineering acceptance must pin all fresh and supplemental evidence')
    _review(acceptance.get('review'), 'ACCEPT_HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD')


def supported_preload_template():
    """Incomplete opt-in plan; numerical success cannot authorize motor output."""
    data = template(schema=SCHEMA_V3)
    data.update(diagnostic_timing_acceptance=SUPPORTED_PRELOAD_5S,
        model_backend=SCALAR_BACKEND, voltage_overlap=True,
        watchdog_review_policy=COMMAND_LOSS_ONLY_SUPPORTED,
        local_characterization=LOCAL_RELATIVE_SUPPORTED, policy_weight=0.,
        duration_s=5., startup_duration_s=1., policy_ramp_s=.2, stop_duration_s=.4,
        request_gap_us=900, request_window=3)
    data['cadence_source_sha256'] = cadence_source_hashes(data)
    data['artifacts'] = {name: {'path': None, 'sha256': None} for name in artifact_names(data)}
    data['blockers'].extend(('Reviewed finite 0.25mm path and physical direction/corridor',
        'Pinned software fault/STOP validation and current-power full diagnostic',
        'Support must remain; standing, support removal and walking are not authorized'))
    return data


def _preload_binding(data, path):
    """Bind loader proof to all executable values, not just a copied token."""
    payload = dict(settings=reviewed_settings_sha256(data), axes=data['axes'],
        artifacts=data['artifacts'], boot_id=data['boot_id'],
        motor_power_epoch=data['motor_power_epoch'], path=path)
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def supported_preload_settings(profile):
    """Return admitted path data only; never fabricate a learned-policy target."""
    if profile.get('diagnostic_timing_acceptance') != SUPPORTED_PRELOAD_5S:
        _need('_preload_token' not in profile, 'Preload proof used with a different mode')
        return None
    _need(profile.get('_preload_token') is _PRELOAD_TOKEN and profile.get('output_allowed') is True,
          'Preload execution requires validated loader proof')
    path = profile.get('_preload_path')
    _need(type(path) is dict and profile.get('_preload_binding') == _preload_binding(profile, path),
          'Preload execution settings changed after review')
    from .supported_preload_path import validate_path, RETURN_COMPLETE_S
    try:
        validate_path(path, profile)
    except (ValueError, TypeError, KeyError) as error:
        raise ProfileError(str(error)) from error
    return {'path': copy.deepcopy(path),
            'path_sha256': profile['artifacts']['preload_path']['sha256'],
            'return_complete_s': RETURN_COMPLETE_S}


def _supported_preload_evidence(documents, data, base):
    """Separate reviewed disposition of an immutable file-only candidate.

    Historical candidate blockers/approval flags are retained. Numerical screen
    failures are never waivable. The software review is evidence of offline
    tests only, while fresh hardware, watchdog and diagnostic gates stay active.
    """
    from .supported_preload_path import validate_path
    path = documents['preload_path']
    prior = documents['preload_source_profile']
    review = documents['preload_review']
    _structure(prior)
    _need(prior['approved_for_supported_policy_output'] is True and prior['blockers'] == [] and
          prior['scope'] == 'supported_characterization_only',
          'Preload source must be a pinned historical supported profile')
    _review(prior['review'], 'APPROVED_SUPPORTED_CHARACTERIZATION')
    for name in ('calibration', 'mount', 'bias', 'model_manifest'):
        _need(prior['artifacts'][name]['sha256'] == data['artifacts'][name]['sha256'],
              'Preload historical calibration/model provenance differs: '+name)
    for mid in IDS:
        for name in ('uid', 'sign', 'offset_rad'):
            _need(prior['axes'][mid][name] == data['axes'][mid][name],
                  'Preload historical calibration differs: ID'+mid)
    _need(type(path) is dict and type(path.get('source_screen')) is dict,
          'Preload path/source screen required')
    screen = path['source_screen']
    _need(screen.get('profile_sha256') == data['artifacts']['preload_source_profile']['sha256'] and
          screen.get('capture_sha256') == data['artifacts']['local_reference_capture']['sha256'],
          'Preload path must pin its original source profile and current capture')
    try:
        numerical = validate_path(path, data)
    except (ValueError, TypeError, KeyError) as error:
        raise ProfileError(str(error)) from error
    capture = documents['local_reference_capture']
    turns = documents['hardware_review']['local_characterization']['reference_turns_by_id']
    for n, mid in enumerate(IDS):
        raw = capture['telemetry']['rows'][mid]['median_position_rad']
        axis = data['axes'][mid]
        q = axis['sign']*(raw-turns[mid]*2*math.pi)+axis['offset_rad']
        _need(abs(raw-numerical.initial_raw[n]) <= 1e-10 and
              abs(q-numerical.initial_model[n]) <= 1e-10,
              'Preload path origin differs from reviewed capture: ID'+mid)
    _need(type(review) is dict and review.get('schema') == 'singularitydog.supported-preload-review.v1',
          'Separate supported preload engineering review required')
    _review(review.get('review'), 'ACCEPT_SUPPORTED_GEOMETRIC_PRELOAD_5S')
    expected = dict(mode=SUPPORTED_PRELOAD_5S, scope=data['scope'],
        boot_id=data['boot_id'], motor_power_epoch=data['motor_power_epoch'],
        assembly_id=data['assembly_id'],
        path_sha256=data['artifacts']['preload_path']['sha256'],
        source_profile_sha256=data['artifacts']['preload_source_profile']['sha256'],
        capture_sha256=data['artifacts']['local_reference_capture']['sha256'],
        diagnostic_sha256=data['artifacts']['pipeline_diagnostic']['sha256'],
        command_loss_sha256=data['artifacts']['command_loss_report']['sha256'],
        source_sha256=data['cadence_source_sha256'])
    _need(all(review.get(k) == v for k, v in expected.items()),
          'Preload review must pin exact path, sources and current-epoch evidence')
    flags = dict(support_must_remain=True, low_catch_must_remain=True,
        immediate_power_cutoff_ready=True, four_foot_contact_observed=True,
        physical_corridor_and_direction_verified=True, encoder_branch_rechecked=True,
        original_candidate_not_promoted=True, absolute_accuracy_not_certified=True,
        load_bearing_not_established=True, standing_allowed=False,
        walking_allowed=False, box_removal_allowed=False)
    _need(all(review.get(k) is v for k, v in flags.items()),
          'Preload physical supported-only scope acknowledgements incomplete')
    direction = screen.get('up_direction_body_unit_vector')
    _need(type(direction) is list and len(direction) == 3 and
          all(type(v) in (int, float) and math.isfinite(v) for v in direction) and
          abs(sum(v*v for v in direction)-1) < 1e-9 and
          review.get('verified_up_direction_body_unit_vector') == direction,
          'Preload extension direction must be independently reviewed')
    _text(review.get('direction_and_contact_evidence'), 'preload direction/contact evidence')
    blockers = path.get('blockers')
    screen_blockers = screen.get('readiness_blockers')
    _need(type(blockers) is list and type(screen_blockers) is list and
          all(type(v) is str and bool(v) for v in blockers+screen_blockers) and
          set(screen_blockers) <= set(blockers), 'Keep original preload candidate blockers')
    dispositions = review.get('blocker_dispositions')
    _need(type(dispositions) is dict and set(dispositions) == set(blockers),
          'Every original preload blocker needs an explicit disposition')
    for blocker, disposition in dispositions.items():
        _text(disposition, 'preload blocker disposition: '+blocker)
    validation, _ = _artifact(review.get('software_validation'),
        Path(data['artifacts']['preload_review']['path']).parent)
    checks = ('normal_extend_return_and_stop', 'fault_stop_both_buses', 'cancellation_stop',
              'stale_input_stop', 'path_mutation_and_replay_rejected',
              'return_target_and_measured_confirmation')
    _need(type(validation) is dict and validation.get('schema') ==
          'singularitydog.supported-preload-source-validation.v1' and
          validation.get('status') == 'PASS_FILE_ONLY_TESTS' and
          validation.get('hardware_opened') is False and validation.get('errors') == [] and
          validation.get('source_sha256') == data['cadence_source_sha256'] and
          type(validation.get('checks')) is dict and
          all(validation['checks'].get(k) is True for k in checks),
          'Preload exact-source normal/fault/STOP software validation missing')
    _text(validation.get('test_command'), 'preload validation test command')
    _hash(validation.get('test_output_sha256'), 'preload test output')
    _need(type(validation.get('tests_passed')) is int and validation['tests_passed'] > 0,
          'Preload validation needs an actual passing test count')
    data['_preload_path'] = copy.deepcopy(path)
    data['_preload_binding'] = _preload_binding(data, path)
    data['_preload_token'] = _PRELOAD_TOKEN


def load_profile(path, *, require_approved=True):
    """Return a deep-copied plain mapping; resolves references but never opens hardware.

    An unapproved plan is returned only when require_approved=False. It contains
    placeholders and must not be passed to a runner. Approved loads also work in
    plan mode and receive exactly the same validation as an executable load.
    """
    _need(type(require_approved) is bool, 'Invalid approval requirement')
    path = Path(path).expanduser().absolute()
    original, digest = _read_json(path)
    _structure(original)
    data = copy.deepcopy(original)
    _settings(data)
    if data['approved_for_supported_policy_output'] is False:
        _need(not require_approved, 'Profile remains unapproved; review its blockers before real output')
        _need(data['review'] is None and bool(data['blockers']), 'Unapproved plan needs explicit blockers')
        return {**data, 'output_allowed': False, 'profile_path': str(path), 'profile_sha256': digest}
    _need(data['blockers'] == [], 'Approved profile still contains unresolved blockers')
    _review(data['review'], _approval_decision(data))
    try:
        _need(str(uuid.UUID(data['boot_id'])) == data['boot_id'], 'Noncanonical boot ID')
    except (ValueError, TypeError, AttributeError) as error:
        raise ProfileError('Invalid boot ID') from error
    _text(data['motor_power_epoch'], 'motor power epoch'); _text(data['assembly_id'], 'assembly ID')
    bundle = Path(_text(data['bundle_path'], 'bundle path')).expanduser()
    if not bundle.is_absolute():
        bundle = path.parent/bundle
    _need(bundle.is_dir(), 'Pinned policy bundle directory missing')
    for filename, sha in shadow.SOURCE_HASHES.items():
        source = bundle/filename
        _need(source.is_file() and not source.is_symlink() and shadow.sha(source) == sha,
              'Pinned bundle member mismatch: '+filename)
    data['bundle_path'] = str(bundle.absolute())
    encoder_selection = native_batch_encoder_settings(data)
    if encoder_selection is not None:
        encoder_path = bundle / encoder_selection['path']
        _need(encoder_path.is_file() and not encoder_path.is_symlink() and
              hashlib.sha256(encoder_path.read_bytes()).hexdigest() == encoder_selection['sha256'],
              'Pinned native batch encoder binary is missing or differs')
        data['_native_batch_encoder_path'] = str(encoder_path.absolute())
    documents = {}
    for key in artifact_names(data):
        documents[key], data['artifacts'][key] = _artifact(data['artifacts'][key], path.parent)
    command_loss_only = data.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED
    if command_loss_only:
        _supported_command_loss_acceptance(documents['operator_acceptance'], documents['command_loss_report'], data)
    _axes(data, documents['calibration'])
    shadow.validate_imu_mount_candidate(documents['mount'])
    _bias(documents['bias'])
    reviewed_acceleration(documents['bias'], documents['mount']['R_body_from_sensor'],
                         enabled=acceleration_calibration_selected(data))
    if accel_input_hypothesis_selected(data):
        value = _load_accel_input_hypothesis(data['artifacts']['accel_input_hypothesis'],
                                            documents['mount']['R_body_from_sensor'])
        data['_accel_input_hypothesis_provenance'] = value.provenance()
        provenance = data['_accel_input_hypothesis_provenance']
        _need(type(provenance) is dict and
              provenance.get('kind') == 'singularitydog.supported-accel-input-hypothesis.v1' and
              provenance.get('scope') == 'boxed_small_mix_only' and
              provenance.get('hypothesis_sha256') == data['artifacts']['accel_input_hypothesis']['sha256'] and
              provenance.get('formal_calibration_approved') is False and
              provenance.get('absolute_orientation_error_bound_rad') is None and
              provenance.get('grants_motor_output') is False and
              provenance.get('fit_and_independent_captures_reaudited') is True,
              'Acceleration input hypothesis must retain its audited unapproved scope')
        _need(json.dumps(provenance.get('raw_norm_bounds_m_s2'), allow_nan=False) ==
              json.dumps([data['imu_accel_norm_min_m_s2'], data['imu_accel_norm_max_m_s2']], allow_nan=False),
              'Acceleration input hypothesis raw norm bounds differ from existing profile monitors')
    manifest = documents['model_manifest']
    _need(type(manifest) is dict and manifest.get('schema') == 'native-policy-overnight-v1' and
          manifest.get('status') == 'VALIDATED_FILE_ONLY' and manifest.get('bundle_hashes') == shadow.SOURCE_HASHES and
          all(manifest.get(k) is False for k in ('output_allowed', 'approved_for_runtime', 'live_50hz_verified')),
          'Pinned native model equivalence manifest required; runtime loader checks ABI and library')
    if execution_settings(data)['model_backend'] == SCALAR_BACKEND:
        scalar = documents['scalar_step_manifest']
        _need(type(scalar) is dict and scalar.get('schema') == 'native-step-scalar-file-only-v1' and
              scalar.get('status') == 'PASS_FILE_ONLY_COMPARE' and
              scalar.get('baseline_manifest_sha256') == data['artifacts']['model_manifest']['sha256'] and
              all(scalar.get(k) is False for k in ('hardware_opened', 'output_allowed',
                  'approved_for_runtime', 'live_50hz_verified')),
              'Scalar equivalence manifest must retain file-only provenance and exact baseline')
    data['timing_review'] = _timing(documents['pipeline_diagnostic'], data)
    data['watchdog_by_id'] = _hardware(documents['hardware_review'], data,
                                     Path(data['artifacts']['hardware_review']['path']).parent,
                                     command_loss_report=documents.get('command_loss_report'),
                                     local_reference_capture=documents.get('local_reference_capture'))
    if execution_settings(data)['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_10S_AFTER_2S:
        _supported_extension_evidence(documents, original)
    if execution_settings(data)['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_20S_AFTER_10S:
        _supported_20s_extension_evidence(documents, original, path.parent)
    if execution_settings(data)['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_GAIN_STEP_3S:
        _supported_gain_step_evidence(documents, original)
    if _mix_step_selected(data):
        _supported_mix_step_evidence(documents, data, path.parent)
    if execution_settings(data)['diagnostic_timing_acceptance'] == SUPPORTED_PRELOAD_5S:
        _supported_preload_evidence(documents, data, path.parent)
    if execution_settings(data)['diagnostic_timing_acceptance'] == CURRENT_HOLD_AFTER_SUPPORTED_10S:
        _current_hold_after_supported_evidence(documents, original)
    if execution_settings(data)['diagnostic_timing_acceptance'] == FIXED_CATCH_CURRENT_HOLD_30S:
        _fixed_catch_evidence(documents, original, path.parent)
    if execution_settings(data)['diagnostic_timing_acceptance'] == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S:
        data['_human_supported_audio'] = _human_supported_audio_manifest(
            documents['human_supported_audio_manifest'], original,
            Path(data['artifacts']['human_supported_audio_manifest']['path']).parent)
        _human_supported_evidence(documents, original, path.parent)
    post_reply = _post_reply_policy(data)
    from .policy_post_reply_timing import POST_REPLY_POLICY_V2
    input_age_v2 = post_reply is not None and post_reply['mode'] == POST_REPLY_POLICY_V2
    if execution_settings(data)['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER:
        acceptance = documents['hardware_review'].get('rare_jitter_diagnostic_acceptance', {})
        _need(type(acceptance) is dict and
              acceptance.get('mode') == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER and
              acceptance.get('diagnostic_sha256') == data['artifacts']['pipeline_diagnostic']['sha256'] and
              acceptance.get('scope') == data['scope'] and
              acceptance.get('strict_50hz_not_established') is True and
              ((acceptance.get('live_deadline_policy_unchanged') is True) if not input_age_v2 else
               ('live_deadline_policy_unchanged' not in acceptance and
                _post_reply_review_limits(acceptance, post_reply))),
              'Explicit matching rare-jitter diagnostic acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_RARE_JITTER_DIAGNOSTIC_FOR_2S_SUPPORTED_PROBE')
    if execution_settings(data)['voltage_pipeline']:
        acceptance = documents['hardware_review'].get('voltage_pipeline_acceptance', {})
        _need(type(acceptance) is dict and
              acceptance.get('pipeline') == 'feedback_then_voltage.fast_v1' and
              acceptance.get('diagnostic_sha256') == data['artifacts']['pipeline_diagnostic']['sha256'] and
              acceptance.get('scope') == data['scope'] and
              _post_reply_review_limits(acceptance, post_reply),
              'Explicit matching voltage-pipeline acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_FEEDBACK_THEN_VOLTAGE')
    if _prepared_voltage_publication_selected(data):
        acceptance = documents['hardware_review'].get('prepared_voltage_publication_acceptance', {})
        _need(type(acceptance) is dict and
              acceptance.get('schema') == 'singularitydog.prepared-voltage-publication-review.v1' and
              acceptance.get('mode') == PREPARED_VOLTAGE_PUBLICATION_MODE and
              acceptance.get('diagnostic_sha256') == data['artifacts']['pipeline_diagnostic']['sha256'] and
              acceptance.get('scope') == data['scope'] and
              _post_reply_review_limits(acceptance, post_reply) and
              acceptance.get('stop_proxy_does_not_certify_active_api_latency') is True,
              'Explicit matching prepared voltage publication acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_PREPARED_VOLTAGE_PUBLICATION')
    if encoder_selection is not None:
        acceptance = documents['hardware_review'].get('native_batch_encoder_acceptance', {})
        _need(type(acceptance) is dict and
              acceptance.get('binary_sha256') == encoder_selection['sha256'] and
              acceptance.get('scope') == data['scope'] and
              _post_reply_review_limits(acceptance, post_reply),
              'Explicit matching native batch encoder acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_NATIVE_BATCH_ENCODER')
    if post_reply is not None:
        acceptance = documents['hardware_review'].get('post_reply_deadline_acceptance', {})
        _need(type(acceptance) is dict and acceptance.get('settings') == post_reply and
              acceptance.get('scope') == data['scope'] and
              acceptance.get('strict_50hz_not_established') is True and
              _post_reply_review_limits(acceptance, post_reply),
              'Explicit matching post-reply deadline acceptance required')
        if input_age_v2:
            _need(acceptance.get('schema') == 'singularitydog.post-reply-input-age-review.v2' and
                  acceptance.get('diagnostic_sha256') == data['artifacts']['pipeline_diagnostic']['sha256'],
                  'Explicit source-bound post-reply input-age v2 review required')
        _review(acceptance.get('review'), 'ACCEPT_BOUNDED_POST_REPLY_INPUT_AGE_V2' if input_age_v2
                else 'ACCEPT_BOUNDED_POST_REPLY_DEADLINE')
        data['_post_reply_validation_token'] = _POST_REPLY_VALIDATION_TOKEN
    if _startup_cycle_policy(data) is not None:
        acceptance = documents['hardware_review'].get('startup_cycle_acceptance', {})
        _need(type(acceptance) is dict and acceptance.get('mode') == FIRST_CYCLE_POST_REPLY and
              acceptance.get('scope') == data['scope'] and acceptance.get('first_cycle_only') is True and
              _post_reply_review_limits(acceptance, post_reply) and
              acceptance.get('steady_miss_budget_unchanged') is True,
              'Explicit first-cycle post-reply acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_FIRST_CYCLE_POST_REPLY')
        data['_startup_cycle_token'] = _STARTUP_CYCLE_TOKEN
    if _mix_step_selected(data):
        data['_mix_step_token'] = _MIX_STEP_TOKEN
        data['_mix_step_binding'] = _mix_step_binding(data)
    data['mode0_readback_required_before_enable'] = True
    if data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED:
        data['_local_validation_token'] = _LOCAL_VALIDATION_TOKEN
    if data.get('diagnostic_timing_acceptance') in (
            CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S,
            HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S):
        data['_current_hold_token'] = _CURRENT_HOLD_TOKEN
    if data.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S:
        data['_fixed_catch_token'] = _FIXED_CATCH_TOKEN
    if data.get('diagnostic_timing_acceptance') == HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S:
        data['_human_supported_token'] = _HUMAN_SUPPORTED_TOKEN
        data['_human_supported_binding'] = _human_supported_binding(data)
    if accel_input_hypothesis_selected(data):
        data['_accel_input_hypothesis_token'] = _ACCEL_INPUT_HYPOTHESIS_TOKEN
        data['_accel_input_hypothesis_binding'] = _accel_input_hypothesis_binding(data)
    if _prepared_voltage_publication_selected(data):
        data['_prepared_voltage_publication_token'] = _PREPARED_VOLTAGE_PUBLICATION_TOKEN
        data['_prepared_voltage_publication_binding'] = _prepared_voltage_publication_binding(data)
    if input_age_v2:
        data['_post_reply_input_age_binding'] = _post_reply_input_age_binding(data)
    return {**data, 'output_allowed': True, 'profile_path': str(path), 'profile_sha256': digest,
            'actual_policy_output_20ms_verified': False,
            'support_must_remain': data['scope'] in ('supported_characterization_only', HUMAN_SUPPORTED_PARTIAL_SCOPE),
            'human_body_catch_must_remain': data['scope'] == HUMAN_SUPPORTED_PARTIAL_SCOPE}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    options = parser.add_mutually_exclusive_group(required=True)
    options.add_argument('--write-template', type=Path)
    options.add_argument('--write-preload-template', type=Path,
                         help='Write an unapproved V3 five-second geometric preload plan')
    options.add_argument('--check', type=Path)
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--template-schema', choices=(SCHEMA_V1, SCHEMA_V2, SCHEMA_V3), default=SCHEMA,
                        help='V3 cadence is explicit and always starts unapproved')
    args = parser.parse_args(argv)
    target = args.write_preload_template or args.write_template
    if target:
        candidate = (supported_preload_template() if args.write_preload_template
                     else template(schema=args.template_schema))
        with target.open('x', encoding='utf-8') as stream:
            target.chmod(0o600)
            json.dump(candidate, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        print('UNAPPROVED_TEMPLATE_WRITTEN '+str(target))
        return 0
    data = load_profile(args.check, require_approved=not args.plan_only)
    print(json.dumps({'output_allowed': data['output_allowed'], 'scope': data['scope'],
                      'blockers': data['blockers'], 'profile_sha256': data['profile_sha256'],
                      'transport_settings': transport_settings(data),
                      'telemetry_cadence': telemetry_settings(data)}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
