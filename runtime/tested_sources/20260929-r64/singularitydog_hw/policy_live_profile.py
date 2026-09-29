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

from . import policy_shadow as shadow
from .policy_observer import _bias

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
                     'startup_cycle_allowance', 'fixed_catch'}
COMMAND_LOSS_ONLY_SUPPORTED = 'command_loss_only_supported_trial'
LOCAL_RELATIVE_SUPPORTED = 'bounded_relative_supported_v1'
LOCAL_NUMERICAL_MARGIN_RAD = 2*25.14/65535
_LOCAL_VALIDATION_TOKEN = object()
_POST_REPLY_VALIDATION_TOKEN = object()
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
SUPPORTED_POLICY_PROBE = 'supported-policy-probe-v1'
SUPPORTED_POLICY_PROBE_5S = 'supported-policy-probe-5s-v1'
SUPPORTED_POLICY_PROBE_2S_RARE_JITTER = 'supported-policy-probe-2s-rare-jitter-v1'
SUPPORTED_POLICY_PROBE_10S_AFTER_2S = 'supported-policy-probe-10s-after-2s-v1'
SUPPORTED_POLICY_GAIN_STEP_3S = 'supported-policy-gain-step-3s-v1'
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
    timing = profile.get('diagnostic_timing_acceptance')
    _need(timing in (None, OBSERVED_R17_TIMING, MEASURED_R17_STARTUP_TIMING,
                    CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S,
                    SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S,
                    SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                    SUPPORTED_POLICY_GAIN_STEP_3S),
          'Unsupported diagnostic timing acceptance')
    _need(profile.get('watchdog_review_policy') in (None, COMMAND_LOSS_ONLY_SUPPORTED),
          'Unsupported watchdog review policy')
    _need(profile.get('local_characterization') in (None, LOCAL_RELATIVE_SUPPORTED),
          'Unsupported local characterization')
    return {'model_backend': backend, 'voltage_overlap': overlap,
            'voltage_pipeline': pipeline,
            'diagnostic_timing_acceptance': timing}


def current_position_hold_only(profile):
    """The probe omits inference, never input validation or active deadlines."""
    selected = profile.get('diagnostic_timing_acceptance') in (
        CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S)
    if selected:
        _need(profile.get('_current_hold_token') is _CURRENT_HOLD_TOKEN and
              profile['policy_weight'] == 0, 'Current-position hold requires loader proof')
    return selected


def _supported_duration_cap(profile):
    if profile.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S:
        return 30
    if profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_10S_AFTER_2S:
        return 10
    if profile.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER:
        return 2
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


def _approval_decision(profile):
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
              SUPPORTED_POLICY_GAIN_STEP_3S) and
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
            SUPPORTED_POLICY_GAIN_STEP_3S) else ()) + (
        _FIXED_CATCH_ARTIFACTS if profile.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S else ())


def _post_reply_policy(profile):
    from .policy_post_reply_timing import POST_REPLY_POLICY
    value = profile.get('post_reply_deadline_policy')
    if value is None:
        _need('post_reply_deadline_policy' not in profile, 'Omit inactive post-reply deadline policy')
        return None
    _need(profile['schema'] == SCHEMA_V3 and
          profile.get('scope') == 'supported_characterization_only',
          'Post-reply deadline policy requires supported-only V3')
    _need(type(value) is dict and set(value) == {'mode', 'max_lateness_ms',
          'max_consecutive_misses', 'rolling_window_cycles', 'max_misses_per_window'} and
          value.get('mode') == POST_REPLY_POLICY, 'Invalid post-reply deadline policy')
    _number(value['max_lateness_ms'], 'post-reply lateness', 0, 1, positive=True)
    for key, expected in (('max_consecutive_misses', 1), ('rolling_window_cycles', 100),
                          ('max_misses_per_window', 1)):
        _need(type(value[key]) is int and value[key] == expected, 'Invalid post-reply '+key)
    _need(profile['hard_cycle_ms'] == 20 and profile['max_sample_age_ms'] <= 20 and
          profile['max_sample_gap_ms'] <= 21 and profile['max_consecutive_20ms_misses'] == 0 and
          profile['duration_s'] <= _supported_duration_cap(profile),
          'Post-reply policy preserves hard20ms, freshness and finite supported scope')
    return dict(value)


def post_reply_deadline_settings(profile):
    """Only a fully reviewed loader result may enable post-reply tolerance."""
    value = _post_reply_policy(profile)
    if value is not None:
        _need(profile.get('_post_reply_validation_token') is _POST_REPLY_VALIDATION_TOKEN,
              'Post-reply deadline policy requires validated loader proof')
    return value


def local_characterization_settings(profile):
    """Return a loader-derived local mode proof; raw JSON cannot mint the token."""
    if profile.get('local_characterization') is None:
        return None
    _need(profile.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
          profile.get('_local_validation_token') is _LOCAL_VALIDATION_TOKEN,
          'Local characterization requires validated loader proof')
    return {'mode': LOCAL_RELATIVE_SUPPORTED, 'numerical_position_margin_rad': LOCAL_NUMERICAL_MARGIN_RAD,
            'absolute_zero_uncertainty_rad': None, 'max_displacement_rad': math.radians(1)}


def _supported_command_loss_acceptance(acceptance, report, data):
    """An explicit bounded experiment choice; never fabricate a USB test result."""
    fixed_catch = _fixed_catch_settings(data)
    _need(data['schema'] == SCHEMA_V3 and
          (data['scope'] == 'supported_characterization_only' or fixed_catch is not None),
          'Command-loss-only review is limited to supported V3 characterization')
    _need(type(acceptance) is dict and
          acceptance.get('schema') == ('singularitydog.fixed-catch-hold-operator-acceptance.v1' if fixed_catch
                                      else 'singularitydog.supported-trial-operator-acceptance.v1') and
          acceptance.get('scope') == data['scope'] and
          acceptance.get('watchdog_review_policy') == COMMAND_LOSS_ONLY_SUPPORTED,
          'Explicit supported command-loss-only operator acceptance required')
    _review(acceptance.get('review'), 'ACCEPT_COMMAND_LOSS_ONLY_FIXED_CATCH_HOLD' if fixed_catch
            else 'ACCEPT_COMMAND_LOSS_ONLY_SUPPORTED_TRIAL')
    _text(acceptance.get('user_statement'), 'explicit user instruction to omit USB test')
    if fixed_catch:
        _need(acceptance.get('fixed_catch_must_remain') is True and
              acceptance.get('fixed_catch_settings') == fixed_catch and
              acceptance.get('upper_box_must_remain') is False and 'box_must_remain' not in acceptance,
              'Fixed catch operator acceptance must preserve the fixed full-weight catch')
    _need(acceptance.get('usb_disconnect_test_waived') is True and
          (fixed_catch is not None or acceptance.get('box_must_remain') is True) and
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


def cadence_source_hashes():
    """Read-only identity of cadence-related source files; not all kit dependencies."""
    root = Path(__file__).resolve().parents[1]
    values = {}
    for name in CADENCE_SOURCE_PATHS:
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
        _need(type(sources) is dict and set(sources) == set(CADENCE_SOURCE_PATHS),
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
        _need(profile['cadence_source_sha256'] == cadence_source_hashes(),
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
    _need(data['scope'] == 'supported_characterization_only' or
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
    duration = _number(data['duration_s'], 'duration_s', .5, 30 if fixed_catch else 10)
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
    policy_probe_5s = data.get('diagnostic_timing_acceptance') == SUPPORTED_POLICY_PROBE_5S
    rare_jitter_probe = data.get('diagnostic_timing_acceptance') in (
        SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
        SUPPORTED_POLICY_GAIN_STEP_3S)
    if data.get('diagnostic_timing_acceptance') in (
            SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            SUPPORTED_POLICY_PROBE_10S_AFTER_2S):
        _need(data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED and
              0 < data['policy_weight'] <= .005 and duration <= (
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
        _need(duration <= _supported_duration_cap(data) and data['policy_weight'] <= .01 and hard == 20 and
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
    current_position_hold = data['policy_weight'] == 0
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
            for key, cap in {'kp':12. if current_position_hold or gain_step else 3., 'kd':.15,
                    'max_command_velocity_rad_s':math.radians(1),
                    'max_command_acceleration_rad_s2':math.radians(5),
                    'max_tracking_error_rad':math.radians(2),
                    # Retain the already bounded five-second probe's monitor
                    # for its shorter rare-jitter admission; gains/motion stay fixed.
                    'max_measured_velocity_rad_s':(.35 if data.get('diagnostic_timing_acceptance') in
                        (SUPPORTED_POLICY_PROBE_5S, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
                         SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
                         SUPPORTED_POLICY_GAIN_STEP_3S) else .25),
                    'max_measured_torque_nm':1.,
                    'max_estimated_pd_torque_nm':.5 if current_position_hold or gain_step else .1,
                    'max_temperature_c':45., 'max_displacement_from_start_rad':math.radians(1)}.items():
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
    _need(type(report) is dict and report.get('status') == 'COMPLETE_DIAGNOSTIC' and
          report.get('mode') == 'stop-proxy' and report.get('errors') == [] and
          report.get('motor_enable_sent') is False and report.get('learned_targets_sent') is False and
          report.get('full_controller_50Hz_verified') is False and
          type(report.get('observer')) is dict, 'Full real-input/inference/STOP diagnostic required')
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
    observed_r17 = execution['diagnostic_timing_acceptance'] == OBSERVED_R17_TIMING
    hold_after_supported = execution['diagnostic_timing_acceptance'] == CURRENT_HOLD_AFTER_SUPPORTED_10S
    fixed_catch_hold = execution['diagnostic_timing_acceptance'] == FIXED_CATCH_CURRENT_HOLD_30S
    hold_probe = execution['diagnostic_timing_acceptance'] in (
        CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S)
    rare_jitter_probe = execution['diagnostic_timing_acceptance'] in (
        SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
        SUPPORTED_POLICY_GAIN_STEP_3S, CURRENT_HOLD_AFTER_SUPPORTED_10S,
        FIXED_CATCH_CURRENT_HOLD_30S)
    policy_probe = execution['diagnostic_timing_acceptance'] in (
        SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S, SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
        SUPPORTED_POLICY_PROBE_10S_AFTER_2S, SUPPORTED_POLICY_GAIN_STEP_3S)
    bounded_probe = hold_probe or policy_probe
    measured_r17 = execution['diagnostic_timing_acceptance'] in (
        MEASURED_R17_STARTUP_TIMING, CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S,
        FIXED_CATCH_CURRENT_HOLD_30S,
        SUPPORTED_POLICY_PROBE, SUPPORTED_POLICY_PROBE_5S,
        SUPPORTED_POLICY_PROBE_2S_RARE_JITTER, SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
        SUPPORTED_POLICY_GAIN_STEP_3S)
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
            bounded_probe and (end > scheduled+20_000_000 or replied-oldest > 20_000_000))
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
    return {'kind': ('fixed_catch_current_hold_30s_admission_only' if fixed_catch_hold else
                     'current_hold_after_supported_10s_admission_only' if hold_after_supported else
                     'supported_policy_10s_after_2s_admission_only'
                     if execution['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_10S_AFTER_2S else
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
    _need(type(local) is dict and local.get('schema') == 'singularitydog.local-relative-review.v1' and
          local.get('operator_confirmed_local_clearance') is True and
          local.get('local_clearance_rad') == math.radians(3) and
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
        lower, upper = max(shadow.LOWER[index], q-math.radians(3)), min(shadow.UPPER[index], q+math.radians(3))
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
            if k not in (*_EXTENSION_ARTIFACTS, 'hardware_review', 'operator_acceptance', 'local_reference_capture')}
        return result
    _need(contract(prior) == contract(data),
          'Extension changes prior execution, model, calibration, UID, boot, power or safety contract')
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
            if k not in (*_EXTENSION_ARTIFACTS, 'hardware_review', 'operator_acceptance', 'local_reference_capture')}
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
    prior_post_reply = _post_reply_policy(prior)
    _need(prior_post_reply is not None and report.get('post_reply_deadline_policy') == prior_post_reply,
          'Current hold predecessor post-reply policy differs')
    startup_enabled = _startup_cycle_policy(prior) is not None
    _need(report.get('startup_20ms_allowance_enabled', False) is startup_enabled,
          'Current hold predecessor startup policy differs')
    misses, startup_misses, previous_end = [], 0, None
    for index, row in enumerate(rows):
        _need(type(row) is dict and row.get('index') == index, 'Current hold predecessor cycle sequence invalid')
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
    if execution_settings(data)['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_GAIN_STEP_3S:
        _supported_gain_step_evidence(documents, original)
    if execution_settings(data)['diagnostic_timing_acceptance'] == CURRENT_HOLD_AFTER_SUPPORTED_10S:
        _current_hold_after_supported_evidence(documents, original)
    if execution_settings(data)['diagnostic_timing_acceptance'] == FIXED_CATCH_CURRENT_HOLD_30S:
        _fixed_catch_evidence(documents, original, path.parent)
    if execution_settings(data)['diagnostic_timing_acceptance'] == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER:
        acceptance = documents['hardware_review'].get('rare_jitter_diagnostic_acceptance', {})
        _need(type(acceptance) is dict and
              acceptance.get('mode') == SUPPORTED_POLICY_PROBE_2S_RARE_JITTER and
              acceptance.get('diagnostic_sha256') == data['artifacts']['pipeline_diagnostic']['sha256'] and
              acceptance.get('scope') == data['scope'] and
              acceptance.get('strict_50hz_not_established') is True and
              acceptance.get('live_deadline_policy_unchanged') is True,
              'Explicit matching rare-jitter diagnostic acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_RARE_JITTER_DIAGNOSTIC_FOR_2S_SUPPORTED_PROBE')
    if execution_settings(data)['voltage_pipeline']:
        acceptance = documents['hardware_review'].get('voltage_pipeline_acceptance', {})
        _need(type(acceptance) is dict and
              acceptance.get('pipeline') == 'feedback_then_voltage.fast_v1' and
              acceptance.get('diagnostic_sha256') == data['artifacts']['pipeline_diagnostic']['sha256'] and
              acceptance.get('scope') == data['scope'] and
              acceptance.get('hard_output_and_freshness_limits_unchanged') is True,
              'Explicit matching voltage-pipeline acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_FEEDBACK_THEN_VOLTAGE')
    if encoder_selection is not None:
        acceptance = documents['hardware_review'].get('native_batch_encoder_acceptance', {})
        _need(type(acceptance) is dict and
              acceptance.get('binary_sha256') == encoder_selection['sha256'] and
              acceptance.get('scope') == data['scope'] and
              acceptance.get('hard_output_and_freshness_limits_unchanged') is True,
              'Explicit matching native batch encoder acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_NATIVE_BATCH_ENCODER')
    post_reply = _post_reply_policy(data)
    if post_reply is not None:
        acceptance = documents['hardware_review'].get('post_reply_deadline_acceptance', {})
        _need(type(acceptance) is dict and acceptance.get('settings') == post_reply and
              acceptance.get('scope') == data['scope'] and
              acceptance.get('strict_50hz_not_established') is True and
              acceptance.get('hard_output_and_freshness_limits_unchanged') is True,
              'Explicit matching post-reply deadline acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_BOUNDED_POST_REPLY_DEADLINE')
        data['_post_reply_validation_token'] = _POST_REPLY_VALIDATION_TOKEN
    if _startup_cycle_policy(data) is not None:
        acceptance = documents['hardware_review'].get('startup_cycle_acceptance', {})
        _need(type(acceptance) is dict and acceptance.get('mode') == FIRST_CYCLE_POST_REPLY and
              acceptance.get('scope') == data['scope'] and acceptance.get('first_cycle_only') is True and
              acceptance.get('hard_output_and_freshness_limits_unchanged') is True and
              acceptance.get('steady_miss_budget_unchanged') is True,
              'Explicit first-cycle post-reply acceptance required')
        _review(acceptance.get('review'), 'ACCEPT_FIRST_CYCLE_POST_REPLY')
        data['_startup_cycle_token'] = _STARTUP_CYCLE_TOKEN
    data['mode0_readback_required_before_enable'] = True
    if data.get('local_characterization') == LOCAL_RELATIVE_SUPPORTED:
        data['_local_validation_token'] = _LOCAL_VALIDATION_TOKEN
    if data.get('diagnostic_timing_acceptance') in (
            CURRENT_HOLD_PROBE, CURRENT_HOLD_AFTER_SUPPORTED_10S, FIXED_CATCH_CURRENT_HOLD_30S):
        data['_current_hold_token'] = _CURRENT_HOLD_TOKEN
    if data.get('diagnostic_timing_acceptance') == FIXED_CATCH_CURRENT_HOLD_30S:
        data['_fixed_catch_token'] = _FIXED_CATCH_TOKEN
    return {**data, 'output_allowed': True, 'profile_path': str(path), 'profile_sha256': digest,
            'actual_policy_output_20ms_verified': False,
            'support_must_remain': data['scope'] == 'supported_characterization_only'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    options = parser.add_mutually_exclusive_group(required=True)
    options.add_argument('--write-template', type=Path)
    options.add_argument('--check', type=Path)
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--template-schema', choices=(SCHEMA_V1, SCHEMA_V2, SCHEMA_V3), default=SCHEMA,
                        help='V3 cadence is explicit and always starts unapproved')
    args = parser.parse_args(argv)
    if args.write_template:
        with args.write_template.open('x', encoding='utf-8') as stream:
            args.write_template.chmod(0o600)
            json.dump(template(schema=args.template_schema), stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        print('UNAPPROVED_TEMPLATE_WRITTEN '+str(args.write_template))
        return 0
    data = load_profile(args.check, require_approved=not args.plan_only)
    print(json.dumps({'output_allowed': data['output_allowed'], 'scope': data['scope'],
                      'blockers': data['blockers'], 'profile_sha256': data['profile_sha256'],
                      'transport_settings': transport_settings(data),
                      'telemetry_cadence': telemetry_settings(data)}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
