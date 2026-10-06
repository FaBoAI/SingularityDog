#!/usr/bin/env python3
"""Prepare a private UNAPPROVED boxed two-second input-hypothesis draft.

Saved files only: no SSH, model/library loading, audio, devices or approvals.
The successful old profile is historical evidence, never current permission.
The original norm diagnostic is re-audited by the runtime's pure template API.
Run with PYTHONPATH=runtime:tools; all input references require explicit SHA256.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import uuid

import audit_angle_calibration as angles
from analyze_stationary_velocity_probe import digest, exact, need, read_file, strict_json
from singularitydog_hw import imu_accel_input_hypothesis as hypothesis
from singularitydog_hw import policy_live_profile as live

SCHEMA = 'singularitydog.corrected-supported-trial-preparation.v1'
INPUT_SCHEMA = 'singularitydog.corrected-supported-trial-input.v1'
FILE_NAMES = ('prior_profile', 'prior_report', 'current_capture', 'calibration',
    'expected_uids', 'mount', 'gyro_bias', 'model_manifest', 'scalar_step_manifest',
    'accel_diagnostic_input', 'accel_diagnostic_report')
FILE_MAX = 64*1024*1024
FLAGS = dict.fromkeys(('output_allowed', 'approved_for_runtime', 'hardware_opened',
    'motor_enable_sent', 'learned_targets_sent', 'source_files_modified',
    'formal_calibration_approved', 'absolute_accuracy_verified',
    'load_transfer_verified', 'standing_verified', 'walking_verified'), False)
CAPS = {'kp': 3., 'kd': .15, 'max_displacement_from_start_rad': math.radians(1),
    'max_command_velocity_rad_s': math.radians(1),
    'max_command_acceleration_rad_s2': math.radians(5),
    'max_tracking_error_rad': math.radians(2), 'max_estimated_pd_torque_nm': .1,
    'max_measured_velocity_rad_s': .35, 'max_measured_torque_nm': 1.,
    'max_temperature_c': 45.}


def json_bytes(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False)+'\n').encode()


def reference(pin):
    return {key: pin[key] for key in ('path', 'sha256')}


def read_reference(value):
    need(type(value) is dict and set(value) == {'path', 'sha256'}, 'Exact path/SHA reference required')
    need(type(value['path']) is str and Path(value['path']).is_absolute(), 'Absolute input path required')
    digest(value['sha256'])
    raw, pin = read_file(value['path'], FILE_MAX)
    exact(pin['sha256'], value['sha256'], 'Input SHA')
    return raw, pin


def verify_pins(pins):
    for pin in pins:
        exact(read_file(pin['path'], FILE_MAX)[1], pin, 'Input/source changed before publication')


def validate_prior(profile, report, profile_sha):
    """Authenticate the saved success and its exact conservative numeric scope."""
    live._structure(profile)
    # Validate numeric settings using current code without pretending that the
    # historical source pins match it. The original bytes/pins stay untouched.
    numeric_only = copy.deepcopy(profile)
    numeric_only['cadence_source_sha256'] = live.cadence_source_hashes()
    live._settings(numeric_only)
    need(profile['approved_for_supported_policy_output'] is True and profile['blockers'] == []
         and type(profile['review']) is dict, 'Historical approved profile required')
    need(profile['schema'] == live.SCHEMA_V3 and profile['scope'] == 'supported_characterization_only'
         and profile.get('local_characterization') == live.LOCAL_RELATIVE_SUPPORTED
         and profile.get('diagnostic_timing_acceptance') == live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER
         and not profile.get('apply_reviewed_accel_calibration', False)
         and not profile.get('accel_input_hypothesis', False), 'Exact historical raw-input boxed probe required')
    for key, value in {'duration_s': 2., 'policy_weight': .005, 'period_ms': 20,
        'hard_cycle_ms': 20., 'max_sample_age_ms': 20., 'max_sample_gap_ms': 21.,
        'startup_duration_s': .4, 'startup_damping_duration_s': .08,
        'policy_ramp_s': .4, 'stop_duration_s': .4, 'request_gap_us': 900,
        'request_window': 3, 'model_backend': live.SCALAR_BACKEND,
        'voltage_overlap': True, 'voltage_pipeline': True}.items():
        need(type(profile.get(key)) is type(value) and profile[key] == value,
             'Historical two-second setting differs: '+key)
    for mid, row in profile['axes'].items():
        for key, value in CAPS.items():
            need(type(row[key]) in (int, float) and math.isfinite(row[key])
                 and row[key] == value, 'Historical cap differs: '+key+' ID'+mid)
    need(report.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and report.get('errors') == []
         and report.get('profile_sha256') == profile_sha
         and report.get('boot_id') == profile['boot_id']
         and report.get('motor_power_epoch') == profile['motor_power_epoch']
         and report.get('scope') == profile['scope']
         and all(report.get(k) is True for k in ('motor_enable_sent', 'learned_targets_sent',
             'motion_gain_sent', 'normal_ramp_completed', 'stop_confirmed')),
         'Successful prior report/profile binding required')
    for scope, ids in (('front', list(range(1, 7))), ('rear', list(range(7, 13)))):
        stop = report.get('stop_reports', {}).get(scope, {})
        need(stop.get('complete') is True and stop.get('confirmed_ids') == ids
             and stop.get('unconfirmed_ids') == stop.get('ambiguous_ids') == [],
             'Prior all-twelve STOP evidence required')


def rebind_axes(profile, calibration, capture, expected_uids, capture_sha, boot):
    rows = live.shadow.validate_calibration(calibration)
    raw, uids = angles.current_values(capture)
    exact(uids, expected_uids, 'Selected UID inventory')
    exact(calibration['identities'], uids, 'Calibration UID inventory')
    exact({mid: row['uid'] for mid, row in profile['axes'].items()}, uids, 'Historical UID inventory')
    exact(capture.get('boot_id'), boot, 'Selected capture boot')
    exact(calibration.get('source_current_boot_id'), boot, 'Calibration boot')
    exact(calibration.get('source_capture_sha256'), capture_sha, 'Calibration capture SHA')
    exact(calibration.get('source_raw_rad_by_id'), raw, 'Calibration raw angles')
    exact(calibration.get('approved_for_runtime'), False, 'Calibration remains unapproved')
    descriptions = {}
    profile['start_pose_bounds'] = {}
    for mid in live.IDS:
        row, selected = profile['axes'][mid], rows[int(mid)]
        need(selected['sign_candidate'] == row['sign'], 'Physical sign change is not a branch rebind')
        # Only the supplied, capture-bound integer-turn derivation is retained.
        turns = (selected['offset_candidate_rad']-row['offset_rad'])/(2*math.pi)
        need(abs(turns-round(turns)) <= 1e-10 and abs(turns) <= 20,
             'Nonintegral physical zero change is not a branch rebind')
        embedded = selected.get('diagnostic_branch_turns_embedded_in_offset')
        need(type(embedded) is int and -20 <= embedded <= 20, 'Explicit selected branch required')
        row['offset_rad'] = selected['offset_candidate_rad']
        q = row['sign']*raw[mid]+row['offset_rad']
        index = live.shadow.CAN_ORDER.index(int(mid))
        lower, upper = live.shadow.LOWER[index], live.shadow.UPPER[index]
        need(math.isfinite(q) and lower < q < upper, 'Current angle outside unchanged model range: ID'+mid)
        row['uncertainty_rad'] = None
        row['physical_lower_rad'] = max(lower, q-math.radians(3))
        row['physical_upper_rad'] = min(upper, q+math.radians(3))
        margin = math.radians(1)+live.LOCAL_NUMERICAL_MARGIN_RAD
        need(row['physical_lower_rad']+margin < q < row['physical_upper_rad']-margin,
             'Current numeric interval cannot contain one-degree probe: ID'+mid)
        profile['start_pose_bounds'][mid] = [q-math.radians(.5), q+math.radians(.5)]
        descriptions[mid] = {'raw_rad': raw[mid], 'model_rad': q,
            'embedded_branch_turns': embedded, 'offset_difference_integer_turns': round(turns),
            'proposed_numeric_local_interval_rad': [row['physical_lower_rad'], row['physical_upper_rad']],
            'physical_clearance_verified': None, 'absolute_zero_uncertainty_rad': None}
    live._axes(copy.deepcopy(profile), calibration)
    return descriptions


def prepare(input_path, output):
    output = Path(output).expanduser().absolute()
    need(not output.exists() and not output.is_symlink(), 'Fresh private output directory required')
    need(not any(p.is_symlink() for p in output.parents), 'Output ancestor symlink refused')
    need(not any((p/'.git').exists() for p in (output, *output.parents)), 'Output must be outside Git')
    raw_input, input_pin = read_file(input_path, 16384)
    request = strict_json(raw_input)
    need(type(request) is dict and set(request) == {'schema', 'assembly_id', 'expected_boot_id',
        'motor_power_epoch', 'files', 'bundle'}, 'Exact preparation input fields required')
    exact(request['schema'], INPUT_SCHEMA, 'Preparation input schema')
    need(type(request['assembly_id']) is str and 0 < len(request['assembly_id']) <= 128,
         'Explicit assembly ID required')
    boot = request['expected_boot_id']
    need(type(boot) is str and str(uuid.UUID(boot)) == boot, 'Canonical selected boot required')
    epoch = request['motor_power_epoch']
    need(epoch is None or type(epoch) is str and 0 < len(epoch) <= 256, 'Power epoch is null or an explicit label')
    need(type(request['files']) is dict and set(request['files']) == set(FILE_NAMES), 'Exact pinned input inventory required')
    documents, raw_files, pins = {}, {}, [input_pin]
    for name in FILE_NAMES:
        raw, pin = read_reference(request['files'][name])
        documents[name], raw_files[name] = strict_json(raw), raw
        pins.append(pin)
    validate_prior(documents['prior_profile'], documents['prior_report'], request['files']['prior_profile']['sha256'])
    profile = copy.deepcopy(documents['prior_profile'])
    axes = rebind_axes(profile, documents['calibration'], documents['current_capture'],
        documents['expected_uids'], request['files']['current_capture']['sha256'], boot)
    live.shadow.validate_imu_mount_candidate(documents['mount'])
    live._bias(documents['gyro_bias'])
    need('accel_calibration_review' not in documents['gyro_bias'], 'Keep formal acceleration review separate')
    manifest = documents['model_manifest']
    need(manifest.get('schema') == 'native-policy-overnight-v1' and manifest.get('status') == 'VALIDATED_FILE_ONLY'
         and manifest.get('bundle_hashes') == live.shadow.SOURCE_HASHES
         and all(manifest.get(k) is False for k in ('output_allowed', 'approved_for_runtime', 'live_50hz_verified')),
         'Pinned file-only model equivalence manifest required')
    scalar = documents['scalar_step_manifest']
    need(scalar.get('schema') == 'native-step-scalar-file-only-v1' and scalar.get('status') == 'PASS_FILE_ONLY_COMPARE'
         and scalar.get('baseline_manifest_sha256') == request['files']['model_manifest']['sha256']
         and all(scalar.get(k) is False for k in ('hardware_opened', 'output_allowed', 'approved_for_runtime', 'live_50hz_verified')),
         'Exact scalar/baseline file-only provenance required')
    bundle = Path(request['bundle']) if type(request['bundle']) is str else None
    need(bundle is not None and bundle.is_absolute() and bundle.is_dir()
         and not any(p.is_symlink() for p in (bundle, *bundle.parents)), 'Absolute regular bundle directory required')
    for name, expected in live.shadow.SOURCE_HASHES.items():
        _, pin = read_file(bundle/name, FILE_MAX)
        exact(pin['sha256'], expected, 'Policy bundle member SHA'); pins.append(pin)
    missing_encoder = False
    encoder = profile.get('native_batch_encoder')
    if encoder is not None:
        binary = bundle/encoder['path']
        if binary.exists() or binary.is_symlink():
            _, pin = read_file(binary, FILE_MAX)
            exact(pin['sha256'], encoder['sha256'], 'Historical native encoder SHA'); pins.append(pin)
        else:
            missing_encoder = True
    template = hypothesis.create_template(request['files']['accel_diagnostic_input'],
        request['files']['accel_diagnostic_report'], documents['mount']['R_body_from_sensor'])
    diagnostic = documents['accel_diagnostic_report']
    need(type(diagnostic.get('input_bindings')) is list and type(diagnostic.get('source_bindings')) is list,
         'Re-audited norm provenance inventory required')
    pins.extend(diagnostic['input_bindings']+diagnostic['source_bindings'])
    # Every live permission artifact is deliberately unresolved. Old true flags
    # are preserved only in the byte-identical historical copies below.
    profile.update(approved_for_supported_policy_output=False, review=None,
        boot_id=boot, motor_power_epoch=epoch or 'NOT_INFERRED_FROM_JETSON_BOOT',
        assembly_id=request['assembly_id'], bundle_path=str(bundle), accel_input_hypothesis=True)
    profile.pop('apply_reviewed_accel_calibration', None)
    profile['cadence_source_sha256'] = live.cadence_source_hashes(profile)
    profile['artifacts'] = {name: {'path': None, 'sha256': None} for name in live.artifact_names(profile)}
    for name, source in (('calibration', 'calibration'), ('mount', 'mount'), ('bias', 'gyro_bias'),
        ('model_manifest', 'model_manifest'), ('scalar_step_manifest', 'scalar_step_manifest'),
        ('local_reference_capture', 'current_capture')):
        profile['artifacts'][name] = dict(request['files'][source])
    template_raw = json_bytes(template)
    profile['artifacts']['accel_input_hypothesis'] = {'path': str(output/'accel-input-hypothesis-draft.json'),
        'sha256': hashlib.sha256(template_raw).hexdigest()}
    blockers = ['fresh_target_boot_power_pose_reconfirmation_pending',
        'current_all12_command_loss_watchdog_and_STOP_evidence_pending',
        'same_source_same_hypothesis_501_no_output_diagnostic_pending',
        'current_local_clearance_and_IMU_direction_observations_pending',
        'explicit_raw_and_corrected_norm_bounds_and_hypothesis_review_pending',
        'named_current_hardware_profile_and_command_loss_only_review_pending',
        'visible_foreground_spoken_launcher_current_source_binding_pending',
        'load_transfer_requires_separate_preparation_and_review']
    if epoch is None: blockers.append('current_motor_power_epoch_not_declared')
    if missing_encoder: blockers.append('native_batch_encoder_bytes_not_present_in_selected_bundle')
    profile['blockers'] = blockers
    live._structure(profile); live._settings(profile)
    source_paths = [Path(__file__), Path(angles.__file__), Path(hypothesis.__file__), Path(live.__file__)]
    import analyze_stationary_velocity_probe as reader
    source_paths.append(Path(reader.__file__))
    source_pins = [read_file(path, FILE_MAX)[1] for path in source_paths]
    runtime = Path(live.__file__).parent.parent
    for name, expected in profile['cadence_source_sha256'].items():
        _, pin = read_file(runtime/name, FILE_MAX)
        exact(pin['sha256'], expected, 'Cadence source SHA'); source_pins.append(pin)
    for name, expected in template['source_sha256'].items():
        _, pin = read_file(runtime/name, FILE_MAX)
        exact(pin['sha256'], expected, 'Hypothesis source SHA'); source_pins.append(pin)
    pins.extend(source_pins)
    result = {'schema': SCHEMA, 'status': 'UNAPPROVED_FILE_ONLY_DRAFT', **FLAGS,
        'selected_boot_id': boot, 'current_boot_verified_on_target': False,
        'motor_power_epoch_label': epoch, 'profile_sha256': hashlib.sha256(json_bytes(profile)).hexdigest(),
        'historical_success_is_current_permission': False, 'selected_new_input_hypothesis': True,
        'apply_reviewed_accel_calibration': False, 'blockers': blockers,
        'input_bindings': pins[:len(FILE_NAMES)+1], 'all_input_and_source_bindings': pins,
        'axis_diagnostic_descriptions': axes,
        'unresolved_measurements': {'absolute_zero_uncertainty_rad': None,
            'absolute_gravity_error_bound_rad': None, 'physical_local_clearance_verified': None,
            'dynamic_velocity_scale_and_sign_verified': None, 'dynamic_torque_interpretation_verified': None},
        'next_steps': ['Reconfirm target boot, motor-power epoch and unchanged supported pose.',
            'Review the separate input hypothesis and explicit raw/corrected norm bounds from saved evidence.',
            'Acquire current all-twelve zero-gain command-loss/STOP evidence and matching corrected-input 501 diagnostic.',
            'Attach actual observations and named bounded profile/hardware reviews; preserve raw origins and sign.',
            'Bind a new foreground spoken launcher to current kit/profile SHA; PLAN first, then separately authorized trial.',
            'Retain box through this two-second trial; load transfer needs its own preparation and review.']}
    files = {'input-manifest.json': raw_input, 'historical-profile.json': raw_files['prior_profile'],
        'historical-report.json': raw_files['prior_report'], 'profile.json': json_bytes(profile),
        'accel-input-hypothesis-draft.json': template_raw, 'preparation.json': json_bytes(result)}
    files['manifest.json'] = json_bytes({'schema': SCHEMA, 'status': result['status'], **FLAGS,
        'files': {name: {'sha256': hashlib.sha256(raw).hexdigest(), 'byte_count': len(raw)}
                  for name, raw in files.items()}})
    verify_pins(pins)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    for name, raw in files.items():
        with os.fdopen(os.open(output/name, os.O_WRONLY|os.O_CREAT|os.O_EXCL|getattr(os, 'O_NOFOLLOW', 0), 0o600), 'wb') as stream:
            stream.write(raw)
    plan = live.load_profile(output/'profile.json', require_approved=False)
    exact(plan['output_allowed'], False, 'Draft PLAN cannot authorize output')
    return {'status': result['status'], 'output': str(output), 'profile': str(output/'profile.json'),
        'profile_sha256': result['profile_sha256'], 'blockers': blockers, **FLAGS}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = prepare(args.input, args.output)
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, allow_nan=False)); return 0


if __name__ == '__main__':
    raise SystemExit(main())
