#!/usr/bin/env python3
"""Bind a disabled fast-voltage diagnostic to a fresh UNAPPROVED V3 draft.

This tool reads files only and writes a new private review directory. It never
opens hardware, changes a source assembly, or supplies a review decision.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'runtime'))
from singularitydog_hw import policy_live_profile as live


def _reference(path):
    path = Path(path).expanduser().absolute()
    value, digest = live._read_json(path)
    if type(value) is not dict:
        raise ValueError('JSON object required: '+str(path))
    return value, {'path': str(path.resolve()), 'sha256': digest}


def _output_path(path):
    path = Path(path).expanduser().absolute()
    if path.exists() or path.is_symlink():
        raise FileExistsError('Use a new private output directory')
    path = path.resolve()
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise ValueError('A new private draft needs an existing regular parent directory')
    if any((parent/'.git').exists() for parent in (path, *path.parents)):
        raise ValueError('Private profile draft must remain outside Git')
    return path


def _json_bytes(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False)+'\n').encode()


def _pending_hardware_review(profile):
    """Create empty review fields when an unapproved plan has no review file."""
    axes={}
    dynamic={}
    watchdog={}
    for mid,row in profile['axes'].items():
        axes[mid]={name:row[name] for name in
            ('sign','offset_rad','uncertainty_rad','physical_lower_rad','physical_upper_rad')}
        axes[mid].update({name:None for name in
            ('zero_and_sign_physically_verified','physical_range_and_clearance_verified',
             'zero_reference_recorded','sign_evidence_reviewed',
             'relative_local_clearance_verified','power_cycle_branch_method_verified')})
        dynamic[mid]={name:None for name in
            ('output_shaft_position_verified','velocity_scale_and_sign_verified',
             'torque_interpretation_verified','limited_trial_reviewed')}
        dynamic[mid].update(position_range_rad=[-12.57,12.57],
            velocity_range_rad_s=[-50.,50.],torque_range_nm=[-5.5,5.5])
        watchdog[mid]={'motor_model':'RS05','actual_command_loss_test_passed':None,
            'usb_disconnect_test_passed':None,'disabled_after_loss_verified':None,
            'configured_timeout_ms':None,'max_observed_disable_ms':None,
            'version_bytes_hex':None,'firmware_version':None}
    imu={name:None for name in
        ('right_handed_mount_physically_verified','nose_up_verified','left_up_verified',
         'yaw_left_verified','gyro_bias_independent_stationary_validation',
         'gravity_direction_verified','gravity_direction_compared_to_operator_level',
         'absolute_gravity_error_bound_rad','gravity_direction_max_error_rad',
         'corrected_static_gyro_max_rad_s','raw_gravity_norm_min_m_s2',
         'raw_gravity_norm_max_m_s2')}
    imu['norm_deviation_rationale']=''
    return {'schema':live.REVIEW_SCHEMA,'scope':profile['scope'],'review':None,
        'assembly_id':profile['assembly_id'],
        'uids_by_id':{mid:row['uid'] for mid,row in profile['axes'].items()},
        'source_captures':[copy.deepcopy(profile['artifacts']['calibration'])],
        'angles':axes,'type2_dynamic':dynamic,'device_watchdog':watchdog,'imu':imu,
        'mode0_readback_required_before_enable':True,'timing_budget_rationale':''}


def prepare(*, base_profile, pipeline_diagnostic, scalar_step_manifest, output):
    """Require a valid disabled timing proof, then leave all approvals pending."""
    out = _output_path(output)
    base_path = Path(base_profile).expanduser().absolute()
    base, base_ref = _reference(base_path)
    live._structure(base)
    live.transport_settings(base)
    old_sources=base.get('cadence_source_sha256')
    if (base.get('telemetry_cadence') != live.CADENCE_PRE_ENABLE or
            type(old_sources) is not dict or not old_sources or
            not set(old_sources) <= set(live.CADENCE_SOURCE_PATHS)):
        raise ValueError('An explicit prior V3 cadence source map is required')
    for name,digest in old_sources.items():live._hash(digest,'prior cadence source '+name)
    if (base['schema'] != live.SCHEMA_V3 or
            base['approved_for_supported_policy_output'] is not False or
            base['review'] is not None or not base['blockers']):
        raise ValueError('An unapproved V3 preparation profile with unresolved blockers is required')
    resolved_base=copy.deepcopy(base)
    for reference in resolved_base['artifacts'].values():
        if reference['path'] is not None:
            path=Path(reference['path']).expanduser()
            reference['path']=str((path if path.is_absolute() else base_path.parent/path).resolve())
    if resolved_base['bundle_path'] is not None:
        bundle=Path(resolved_base['bundle_path']).expanduser()
        resolved_base['bundle_path']=str((bundle if bundle.is_absolute() else
                                          base_path.parent/bundle).resolve())
    hardware_ref=base['artifacts']['hardware_review']
    if hardware_ref == {'path':None,'sha256':None}:
        original_hardware=_pending_hardware_review(resolved_base)
        generated_hardware_skeleton=True
    else:
        original_hardware, _ = live._artifact(hardware_ref, base_path.parent)
        if (type(original_hardware) is not dict or original_hardware.get('schema') != live.REVIEW_SCHEMA or
                original_hardware.get('review') is not None):
            raise ValueError('A pending hardware-review skeleton is required')
        hardware_path=Path(resolved_base['artifacts']['hardware_review']['path'])
        for reference in original_hardware.get('source_captures',[]):
            if type(reference) is dict and type(reference.get('path')) is str:
                path=Path(reference['path']).expanduser()
                reference['path']=str((path if path.is_absolute() else
                                       hardware_path.parent/path).resolve())
        generated_hardware_skeleton=False
    for name in live.ARTIFACTS:
        if name != 'hardware_review':
            live._artifact(base['artifacts'][name], base_path.parent)

    report, report_ref = _reference(pipeline_diagnostic)
    scalar, scalar_ref = _reference(scalar_step_manifest)
    if (scalar.get('schema') != 'native-step-scalar-file-only-v1' or
            scalar.get('status') != 'PASS_FILE_ONLY_COMPARE' or
            scalar.get('baseline_manifest_sha256') != base['artifacts']['model_manifest']['sha256'] or
            any(scalar.get(name) is not False for name in
                ('hardware_opened', 'output_allowed', 'approved_for_runtime', 'live_50hz_verified'))):
        raise ValueError('Pinned unapproved scalar-step equivalence manifest required')

    draft = copy.deepcopy(resolved_base)
    cadence_before=copy.deepcopy(draft['cadence_source_sha256'])
    draft['cadence_source_sha256']=live.cadence_source_hashes()
    changed_sources=[name for name in live.CADENCE_SOURCE_PATHS
                     if cadence_before.get(name)!=draft['cadence_source_sha256'][name]]
    draft.update(model_backend=live.SCALAR_BACKEND, voltage_overlap=True,
                 voltage_pipeline=True,
                 diagnostic_timing_acceptance=live.MEASURED_R17_STARTUP_TIMING)
    draft['artifacts']['pipeline_diagnostic'] = report_ref
    draft['artifacts']['scalar_step_manifest'] = scalar_ref
    draft['artifacts']['hardware_review'] = {
        'path': str(out/'hardware-review.json'), 'sha256': None}
    draft['approved_for_supported_policy_output'] = False
    draft['review'] = None
    live._settings(draft)
    if (type(draft['boot_id']) is not str or
            report.get('boot_id') != draft['boot_id'] or
            not draft['motor_power_epoch']):
        raise ValueError('Diagnostic and prepared profile need the same current boot and a bound motor-power epoch')
    # The timing validator reopens records.json beside the exact report and
    # verifies its report-pinned SHA and every cycle's STOP evidence.
    timing = live._timing(report, draft)
    draft['blockers'] = [item for item in draft['blockers']
                         if item != 'missing_measurements:full_pipeline_diagnostic_not_eligible']
    for item in ('named_review:v3_fast_voltage_diagnostic_and_schedule_pending',
                 'named_review:v3_fast_voltage_pipeline_acceptance_pending'):
        if item not in draft['blockers']:
            draft['blockers'].append(item)
    if changed_sources:
        draft['blockers'].append('named_review:v3_fast_voltage_updated_source_pins_pending')

    hardware = copy.deepcopy(original_hardware)
    hardware['review'] = None
    hardware['reviewed_settings_sha256'] = live.reviewed_settings_sha256(draft)
    hardware['artifact_sha256'] = {
        name: draft['artifacts'][name]['sha256'] for name in live.artifact_names(draft)
        if name != 'hardware_review'}
    hardware['voltage_pipeline_acceptance'] = {
        'pipeline': 'feedback_then_voltage.fast_v1',
        'diagnostic_sha256': report_ref['sha256'], 'scope': draft['scope'],
        'hard_output_and_freshness_limits_unchanged': None, 'review': None}
    hardware_bytes = _json_bytes(hardware)
    draft['artifacts']['hardware_review']['sha256'] = hashlib.sha256(hardware_bytes).hexdigest()
    live._structure(draft)
    live._settings(draft)
    draft_bytes = _json_bytes(draft)
    audit = {
        'schema': 'singularitydog.v3-fast-voltage-draft.v1',
        'status': 'UNAPPROVED_REVIEW_DRAFT', 'output_allowed': False,
        'hardware_opened': False, 'source_files_modified': False,
        'base_profile_sha256': base_ref['sha256'],
        'pipeline_diagnostic_sha256': report_ref['sha256'],
        'fast_records_sha256': report['v3_voltage_fast_pipeline']['records_sha256'],
        'scalar_step_manifest_sha256': scalar_ref['sha256'],
        'generated_hardware_review_skeleton': generated_hardware_skeleton,
        'cadence_source_sha256_before': cadence_before,
        'cadence_source_sha256_after': draft['cadence_source_sha256'],
        'cadence_sources_requiring_review': changed_sources,
        'timing_review_only': timing, 'blockers': draft['blockers'],
        'actual_policy_output_20ms_verified': False,
        'operator_acceptance_supplied': False}
    files = {'profile.json': draft_bytes, 'hardware-review.json': hardware_bytes,
             'draft-audit.json': _json_bytes(audit)}
    # Validate the unapproved file contract before publishing the directory.
    with tempfile.TemporaryDirectory(prefix='.v3-fast-draft-', dir=out.parent) as temporary:
        staged = Path(temporary)/'draft'
        staged.mkdir(mode=0o700)
        for name, raw in files.items():
            path = staged/name
            with os.fdopen(os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), 'wb') as stream:
                stream.write(raw)
        checked = live.load_profile(staged/'profile.json', require_approved=False)
        if checked['output_allowed'] is not False:
            raise AssertionError('Draft must never authorize output')
        os.replace(staged, out)
    return {'status': audit['status'], 'output_allowed': False,
            'hardware_opened': False, 'output': str(out),
            'profile_sha256': hashlib.sha256(draft_bytes).hexdigest(),
            'timing_review_only': timing, 'blockers': draft['blockers']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('base-profile', 'pipeline-diagnostic', 'scalar-step-manifest', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    result = prepare(**vars(parser.parse_args(argv)))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
