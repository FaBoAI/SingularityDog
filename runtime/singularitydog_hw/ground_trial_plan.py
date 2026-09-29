"""File-only gates for bounded supported, load-transfer, stand and walk trials.

A plan authorizes numerical scope only. Hashes bind reviewed files; they cannot
prove that a video depicts this robot or that a catcher is present now. Even a
validated plan returns ``output_allowed=False``: the live runner must separately
arm current support/catch, recording, cutoff and operator acknowledgement.

No model, clock, filesystem, transport or hardware API is called here. A caller
loads preceding evaluation files as exact bytes, checking their paths separately.
Synthetic fixtures are useful for testing these gates and never physical evidence.
"""

import copy
from datetime import datetime
import hashlib
import json
import math


SCHEMA = 'singularitydog.ground-trial-plan.v1'
SCOPE = 'bounded_ground_characterization'
EVALUATION_SCHEMA = 'singularitydog.ground-trial-evaluation.v1'
STAGES = ('supported_stance', 'partial_load', 'stand', 'walk')
IDS = tuple(str(i) for i in range(1, 13))
TOP_KEYS = {'schema', 'scope', 'stage', 'approved_for_ground_trial', 'blockers',
            'review', 'base_profile_sha256', 'assembly_id', 'boot_id',
            'motor_power_epoch', 'trajectory', 'catch', 'video_required',
            'prior_evaluations', 'maximum_measured_distance_m'}
TRAJECTORY_KEYS = {'stage', 'duration_s', 'initial_hold_s', 'active_duration_s',
                   'forward_velocity_m_s', 'ramp_up_s', 'ramp_down_s',
                   'final_stationary_s', 'resupport_window_s', 'shutdown_reserve_s'}
CATCH_KEYS = {'kind', 'operator_count', 'full_weight_capacity_reviewed',
              'motor_disabled_recovery_reviewed', 'roles_separated', 'review_note'}


class GroundPlanError(ValueError):
    """The file cannot authorize this bounded commissioning stage."""


def _need(condition, message):
    if not condition:
        raise GroundPlanError(message)


def _text(value, label):
    _need(type(value) is str and 0 < len(value) <= 4096 and value.strip() == value,
          'Missing/invalid ' + label)
    return value


def _number(value, label, lo, hi):
    _need(type(value) in (int, float), 'Invalid number: ' + label)
    try:
        value = float(value)
    except (ValueError, OverflowError) as error:
        raise GroundPlanError('Invalid number: ' + label) from error
    _need(math.isfinite(value) and lo <= value <= hi, 'Out of range: ' + label)
    return value


def _hash(value, label):
    _need(type(value) is str and len(value) == 64 and
          all(c in '0123456789abcdef' for c in value), 'Invalid SHA256: ' + label)
    return value


def _stamp(value, label):
    _text(value, label)
    try:
        stamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
        _need(stamp.utcoffset() is not None, label + ' needs timezone')
    except (ValueError, TypeError) as error:
        raise GroundPlanError('Invalid ' + label) from error


def prerequisites(stage):
    _need(stage in STAGES, 'Unknown ground stage')
    return STAGES[:STAGES.index(stage)]


def ground_plan_settings_sha256(plan):
    """Canonical review digest, including evidence pins, catch and fresh epoch.

    Changing approval/review text does not change the digest. Changing any motion
    setting, prior file reference, identity or catch arrangement does.
    """
    _need(type(plan) is dict and set(plan) == TOP_KEYS, 'Unsupported ground plan fields')
    payload = {key: plan[key] for key in TOP_KEYS -
               {'review', 'blockers', 'approved_for_ground_trial'}}
    try:
        raw = json.dumps(payload, sort_keys=True, separators=(',', ':'),
                         allow_nan=False).encode('utf-8')
    except (TypeError, ValueError, OverflowError) as error:
        raise GroundPlanError('Settings are not finite JSON values') from error
    return hashlib.sha256(raw).hexdigest()


def template_ground_plan(stage, base_profile=None):
    """Unapproved starting file; absent evidence and physical review stay absent.

    Timing values are suggestions only. In particular a supplied base duration is
    never increased to make these stages fit; validation may require a separately
    reviewed base profile with enough time for stationary recovery and shutdown.
    """
    prerequisites(stage)
    base = base_profile or {}
    start = base.get('startup_duration_s', 1.) + base.get('policy_ramp_s', 1.)
    reserve = None
    axes = base.get('axes', {})
    if axes and set(axes) == set(IDS):
        try:
            reserve = base['stop_duration_s'] + max(
                row['max_command_velocity_rad_s']/row['max_command_acceleration_rad_s2']
                for row in axes.values()) + .04
        except (KeyError, TypeError, ZeroDivisionError):
            pass
    return {'schema': SCHEMA, 'scope': SCOPE, 'stage': stage,
            'approved_for_ground_trial': False,
            'blockers': ['Review complete base profile and this stage timing',
                         'Review preceding genuine hardware result and synchronized video',
                         'Confirm current support/catch, recording and immediate cutoff'],
            'review': None, 'base_profile_sha256': base.get('profile_sha256'),
            'assembly_id': base.get('assembly_id'), 'boot_id': base.get('boot_id'),
            'motor_power_epoch': base.get('motor_power_epoch'),
            'trajectory': {'stage': stage, 'duration_s': base.get('duration_s'),
                           'initial_hold_s': start, 'active_duration_s': 1.,
                           'forward_velocity_m_s': .02 if stage == 'walk' else 0.,
                           'ramp_up_s': .5 if stage == 'walk' else 0.,
                           'ramp_down_s': .5 if stage == 'walk' else 0.,
                           'final_stationary_s': .5, 'resupport_window_s': 1.,
                           'shutdown_reserve_s': reserve},
            'catch': {'kind': 'support_in_place' if stage == 'supported_stance' else None,
                      'operator_count': None, 'full_weight_capacity_reviewed': False,
                      'motor_disabled_recovery_reviewed': False,
                      'roles_separated': False, 'review_note': None},
            'video_required': True,
            'maximum_measured_distance_m': .15 if stage == 'walk' else None,
            'prior_evaluations': {name: {'path': None, 'sha256': None}
                                  for name in prerequisites(stage)}}


def _structure(plan):
    _need(type(plan) is dict and set(plan) == TOP_KEYS, 'Unsupported ground plan fields')
    _need(plan['schema'] == SCHEMA and plan['scope'] == SCOPE, 'Invalid ground plan scope')
    prerequisites(plan['stage'])
    _need(type(plan['approved_for_ground_trial']) is bool, 'Explicit approval boolean required')
    _need(type(plan['blockers']) is list and
          all(type(v) is str and v.strip() for v in plan['blockers']), 'Invalid blockers')
    _need(type(plan['trajectory']) is dict and set(plan['trajectory']) == TRAJECTORY_KEYS,
          'Unsupported trajectory fields')
    _need(plan['trajectory']['stage'] == plan['stage'], 'Trajectory stage mismatch')
    _need(type(plan['catch']) is dict and set(plan['catch']) == CATCH_KEYS,
          'Unsupported catch fields')
    _need(plan['video_required'] is True, 'Synchronized video is required')
    _need(type(plan['prior_evaluations']) is dict and
          set(plan['prior_evaluations']) == set(prerequisites(plan['stage'])),
          'Every preceding stage needs a pinned evaluation')


def _base(plan, base):
    _need(type(base) is dict and base.get('output_allowed') is True and
          base.get('approved_for_supported_policy_output') is True and
          base.get('scope') == 'supported_characterization_only' and
          base.get('blockers') == [], 'Reviewed supported base profile is required')
    _need(_hash(plan['base_profile_sha256'], 'base profile') == base.get('profile_sha256'),
          'Base profile SHA256 mismatch')
    for key in ('assembly_id', 'boot_id', 'motor_power_epoch'):
        _need(_text(plan[key], key) == base.get(key), 'Base identity/epoch mismatch: ' + key)
    axes = base.get('axes')
    _need(type(axes) is dict and set(axes) == set(IDS), 'Base needs all twelve CAN axes')
    for mid, row in axes.items():
        _need(type(row) is dict, 'Invalid base axis: ' + mid)
        _text(row.get('uid'), 'UID ' + mid)
    _need(len({row['uid'] for row in axes.values()}) == 12, 'Duplicate base motor UID')
    if plan['stage'] in ('stand', 'walk'):
        _need(type(base.get('policy_weight')) in (int, float) and base['policy_weight'] == 1.,
              'Stand/walk characterization requires the reviewed full learned-policy weight')
    if plan['stage'] == 'walk':
        _need(type(base.get('hard_cycle_ms')) in (int, float) and base['hard_cycle_ms'] == 20. and
              type(base.get('max_consecutive_20ms_misses')) is int and
              base['max_consecutive_20ms_misses'] == 0,
              'Walk cannot reuse a relaxed diagnostic cycle deadline')
        _number(base.get('max_sample_age_ms'), 'walk sample age', 1., 20.)
        timing = base.get('timing_review')
        _need(type(timing) is dict and type(timing.get('twenty_ms_misses')) is int and
              timing['twenty_ms_misses'] == 0,
              'Walk requires zero misses in the pinned base diagnostic; actual MIT proof is also required')


def _trajectory(plan, base):
    row = plan['trajectory']
    duration = _number(row['duration_s'], 'duration_s', .5, 10)
    _need(duration == base.get('duration_s'), 'Trajectory duration must equal reviewed base duration')
    start = _number(row['initial_hold_s'], 'initial_hold_s', .2, 10)
    _need(start >= base['startup_duration_s'] + base['policy_ramp_s'],
          'Initial stationary hold must include gain startup and policy blending')
    active = _number(row['active_duration_s'], 'active_duration_s',
                     1. if plan['stage'] == 'walk' else .5, 5.)
    velocity = _number(row['forward_velocity_m_s'], 'forward_velocity_m_s', 0, .05)
    up = _number(row['ramp_up_s'], 'ramp_up_s', 0, 5.)
    down = _number(row['ramp_down_s'], 'ramp_down_s', 0, 5.)
    if plan['stage'] == 'walk':
        _need(velocity > 0 and up >= .5 and down >= .5 and up + down <= active,
              'Walk needs positive bounded forward speed and two >=0.5s ramps')
        _number(plan['maximum_measured_distance_m'], 'maximum measured walk distance', .05, .3)
    else:
        _need(velocity == up == down == 0, 'Only walk may command locomotion')
        _need(plan['maximum_measured_distance_m'] is None,
              'Measured travel distance applies only to the walking stage')
    stationary = _number(row['final_stationary_s'], 'final_stationary_s', .5, 10)
    window = _number(row['resupport_window_s'], 'resupport_window_s', 1., 10)
    reserve = _number(row['shutdown_reserve_s'], 'shutdown_reserve_s', .04, 10)
    brake = max(_number(axis['max_command_velocity_rad_s'], 'axis velocity', 1e-9, .5) /
                _number(axis['max_command_acceleration_rad_s2'], 'axis acceleration', 1e-9, 2.)
                for axis in base['axes'].values())
    expected = _number(base['stop_duration_s'], 'base stop duration', .2, 2.) + brake + .04
    _need(abs(reserve - expected) <= 1e-12,
          'Shutdown reserve must include reviewed gain ramp and worst-case joint braking')
    _need(start + active + stationary + window + reserve <= duration + 1e-12,
          'Duration does not fit stationary recovery window and bounded shutdown')


def _catch(plan):
    row = plan['catch']
    _need(type(row['operator_count']) is int and 1 <= row['operator_count'] <= 10,
          'Explicit operator count is required')
    for key in ('full_weight_capacity_reviewed', 'motor_disabled_recovery_reviewed', 'roles_separated'):
        _need(type(row[key]) is bool, 'Explicit catch review boolean required: ' + key)
    _text(row['review_note'], 'catch review note')
    if plan['stage'] == 'supported_stance':
        _need(row['kind'] == 'support_in_place', 'Supported stance must retain its support')
        return
    _need(row['kind'] in ('fixed_catch', 'two_operators'),
          'Load transfer/stand/walk need independent body catch or two operators')
    _need(row['full_weight_capacity_reviewed'] and row['motor_disabled_recovery_reviewed'],
          'Catch must support full weight and recover with motors disabled')
    if row['kind'] == 'two_operators':
        _need(row['operator_count'] >= 2 and row['roles_separated'],
              'Body catch and cutoff/support removal require two separate people')


def _review(plan):
    row = plan['review']
    _need(type(row) is dict and set(row) ==
          {'reviewer', 'reviewed_at', 'decision', 'rationale', 'settings_sha256'},
          'Named numerical and physical-arrangement review is required')
    _text(row['reviewer'], 'reviewer'); _text(row['rationale'], 'review rationale')
    _stamp(row['reviewed_at'], 'review time')
    _need(row['decision'] == 'ALLOW_BOUNDED_GROUND_TRIAL', 'Review has not approved this ground scope')
    _need(_hash(row['settings_sha256'], 'review settings') == ground_plan_settings_sha256(plan),
          'Settings/evidence changed after review')


def _json_bytes(raw):
    _need(type(raw) is bytes and 0 < len(raw) <= 16*1024*1024,
          'Prior evaluation must be exact bounded file bytes')
    def pairs(rows):
        out = {}
        for key, value in rows:
            _need(key not in out, 'Duplicate evaluation JSON key')
            out[key] = value
        return out
    def constant(value):
        raise GroundPlanError('Nonfinite evaluation JSON: ' + value)
    def finite_float(value):
        result = float(value)
        _need(math.isfinite(result), 'Nonfinite evaluation JSON number')
        return result
    try:
        return json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
                          parse_constant=constant, parse_float=finite_float)
    except (ValueError, UnicodeError) as error:
        raise GroundPlanError('Invalid prior evaluation JSON') from error


def _prior(plan, base, supplied):
    _need(type(supplied) is dict and set(supplied) == set(prerequisites(plan['stage'])),
          'Supply exact bytes for every preceding evaluation, without unrelated stages')
    for stage in prerequisites(plan['stage']):
        ref = plan['prior_evaluations'][stage]
        _need(type(ref) is dict and set(ref) == {'path', 'sha256'}, 'Invalid prior reference')
        _text(ref['path'], 'prior evaluation path')
        digest = _hash(ref['sha256'], 'prior evaluation')
        raw = supplied[stage]
        report = _json_bytes(raw)
        _need(hashlib.sha256(raw).hexdigest() == digest, 'Prior evaluation SHA256 mismatch: ' + stage)
        _need(type(report) is dict and report.get('schema') == EVALUATION_SCHEMA and
              report.get('stage') == stage and report.get('status') == 'PASS_REVIEWED_STAGE' and
              report.get('decision') == 'PASS_REVIEWED_HARDWARE_STAGE' and
              report.get('dependency_eligible') is True,
              'Preceding stage is not a reviewed hardware pass: ' + stage)
        for key in ('genuine_hardware_capture', 'fault_free', 'all_axis_stop_confirmed',
                    'physical_review_complete'):
            _need(report.get(key) is True, 'Prior hardware evidence missing: ' + key)
        for key in ('simulated', 'replayed'):
            _need(report.get(key) is False, 'Synthetic/replayed capture cannot release a stage')
        if stage != 'supported_stance':
            _need(report.get('independent_body_catch_used') is True, 'Prior catch evidence missing')
        if stage in ('stand', 'walk'):
            _need(report.get('learned_policy_used') is True, 'Real learned-policy output must precede walk')
        if plan['stage'] == 'walk' and stage == 'stand':
            _need(report.get('actual_controller_20ms_pass') is True,
                  'Walk needs prior stand actual acquire/infer/MIT-output 20ms evidence')
            _number(report.get('max_active_cycle_ms'), 'prior actual MIT cycle', 0.000001, 20.)
        _need(report.get('assembly_id') == base['assembly_id'] and
              report.get('base_profile_sha256') == base['profile_sha256'] and
              report.get('uids_by_id') == {mid: base['axes'][mid]['uid'] for mid in IDS},
              'Prior robot/profile/UID association mismatch: ' + stage)
        _text(report.get('reviewed_by'), 'prior reviewer')
        _stamp(report.get('reviewed_at'), 'prior review time')
        _hash(report.get('report_sha256'), 'prior report')
        _hash(report.get('video_sha256'), 'prior video')
        _hash(report.get('stage_plan_sha256'), 'prior stage plan')
        _hash(report.get('physical_review_sha256'), 'prior physical review')
        association = report.get('association')
        _need(type(association) is dict, 'Prior file/video association missing')
        references = association.get('references')
        _need(type(references) is dict and set(references) == {'report', 'plan', 'profile', 'video'},
              'Prior association must pin report, plan, profile and video')
        for key, top_key in (('report', 'report_sha256'), ('plan', 'stage_plan_sha256'),
                             ('profile', 'base_profile_sha256'), ('video', 'video_sha256')):
            reference = references[key]
            _need(type(reference) is dict and set(reference) == {'path', 'sha256'},
                  'Invalid prior source association: ' + key)
            _text(reference['path'], 'prior source path ' + key)
            _need(_hash(reference['sha256'], 'prior source ' + key) == report[top_key],
                  'Prior source association SHA256 mismatch: ' + key)
        sync = association.get('sync')
        _need(type(sync) is dict and set(sync) ==
              {'trial_start_ns', 'trial_end_ns', 'video_start_s', 'video_end_s', 'uncertainty_ms'},
              'Prior synchronized video interval missing')
        begin, end = sync['trial_start_ns'], sync['trial_end_ns']
        _need(type(begin) is int and type(end) is int and 0 <= begin < end,
              'Invalid prior capture time interval')
        start_s = _number(sync['video_start_s'], 'video start', 0, 86400)
        end_s = _number(sync['video_end_s'], 'video end', 0, 86400)
        _need(start_s < end_s, 'Invalid prior video interval')
        _number(sync['uncertainty_ms'], 'video sync uncertainty', 0, 1000)


def validate_ground_plan(plan, base_profile, prior_evaluations=None, *, require_approved=True):
    """Return a detached execution plan without granting current physical arming.

    ``base_profile`` must be the result of the supported profile loader, whose
    artifact checks this function does not duplicate. Prior files are passed as
    exact bytes, not trusted pre-parsed dictionaries. An unapproved PLAN template
    is inspectable with ``require_approved=False`` but is never execution-valid.
    """
    _structure(plan)
    result = copy.deepcopy(plan)
    result.update(output_allowed=False, physical_arming_required=True,
                  execution_plan_validated=False)
    if not plan['approved_for_ground_trial']:
        _need(not require_approved, 'Ground plan is unapproved')
        return result
    _need(plan['blockers'] == [], 'Ground plan has unresolved blockers')
    _need(type(base_profile) is dict and base_profile.get('watchdog_review_policy') is None,
          'Supported-only command-loss acceptance cannot authorize ground progression')
    _need(base_profile.get('post_reply_deadline_policy') is None,
          'Supported-only post-reply timing acceptance cannot authorize ground progression')
    _base(plan, base_profile)
    _trajectory(plan, base_profile)
    _catch(plan)
    _review(plan)
    _prior(plan, base_profile, {} if prior_evaluations is None else prior_evaluations)
    result['execution_plan_validated'] = True
    return result
