#!/usr/bin/env python3
"""Assemble pinned files into an UNAPPROVED supported-policy review directory.

File-only: no model/library loading, CAN, I2C, SSH, actuation or approvals.
Missing physical measurements remain null; evidence review is never inferred
from an arithmetic match. Existing source files are never modified.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'runtime'))
sys.path.insert(0, str(ROOT/'tools'))
import audit_angle_calibration as angles
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw.angle_calibration_audit import UNKNOWN_EPOCHS, equivalent_branch_candidates

SCHEMA = 'singularitydog.supported-profile-preparation.v1'
SETTINGS_SCHEMA = 'singularitydog.supported-group-settings.v1'
GROUPS = {'calf': (1,4,7,10), 'thigh': (2,5,8,11), 'hip': (3,6,9,12)}
RUN_KEYS = live.TOP_KEYS-{'schema','scope','approved_for_supported_policy_output','blockers','review',
    'boot_id','motor_power_epoch','assembly_id','axes','artifacts','bundle_path','start_pose_bounds'}
# Formal candidates can retain explicitly selected fast implementations. Local
# clearances, stage transitions and supported-only deadline/watchdog exceptions
# still belong to their separate preparation/review paths.
FAST_EXECUTION_KEYS = {'model_backend','voltage_overlap','voltage_pipeline','native_batch_encoder'}
RUN_KEYS |= FAST_EXECUTION_KEYS
IMU_PHYSICAL = ('right_handed_mount_physically_verified','nose_up_verified','left_up_verified',
    'yaw_left_verified','gyro_bias_independent_stationary_validation','gravity_direction_verified')


def settings_template():
    return {'schema': SETTINGS_SCHEMA, 'groups': {
        group: {key: None for key in sorted(live.LIMIT_CAPS)} for group in GROUPS},
        'run_settings': {}}


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _json_bytes(data):
    return (json.dumps(data, indent=2, sort_keys=True, allow_nan=False)+'\n').encode()


def _same(a,b,label):
    if json.dumps(a,sort_keys=True,allow_nan=False)!=json.dumps(b,sort_keys=True,allow_nan=False):
        raise ValueError(label+' mismatch')


def _apply_settings(profile, settings):
    if (type(settings) is not dict or set(settings)!={'schema','groups','run_settings'}
            or settings['schema']!=SETTINGS_SCHEMA or type(settings['groups']) is not dict
            or set(settings['groups'])!=set(GROUPS) or type(settings['run_settings']) is not dict
            or not set(settings['run_settings'])<=RUN_KEYS):
        raise ValueError('Invalid three-group settings schema')
    for group,ids in GROUPS.items():
        values=settings['groups'][group]
        if type(values) is not dict or not set(values)<=set(live.LIMIT_CAPS):
            raise ValueError('Invalid group fields: '+group)
        for key,value in values.items():
            if value is not None:live._number(value,group+'.'+key,0,live.LIMIT_CAPS[key],positive=True)
            for mid in ids:profile['axes'][str(mid)][key]=value
        for lower,upper in (('max_estimated_pd_torque_nm','max_measured_torque_nm'),
                            ('max_command_velocity_rad_s','max_measured_velocity_rad_s')):
            if values.get(lower) is not None and values.get(upper) is not None and values[lower]>values[upper]:
                raise ValueError('Inconsistent group monitor budgets: '+group)
    profile.update(copy.deepcopy(settings['run_settings']))
    live._settings(profile)


def prepare(*, calibration, angle_profile, mount, bias, model_manifest, pipeline_diagnostic,
            assembly_id, output, imu_fragment=None, group_settings=None, source_capture=None, bundle=None,
            power_epoch=None, request_gap_us=None, request_window=None, profile_schema=live.SCHEMA,
            scalar_step_manifest=None):
    """Pin source JSON and emit a review skeleton. Always returns output_allowed=False."""
    out=Path(output).expanduser().absolute()
    if out.exists() or out.is_symlink():raise FileExistsError('Use a new private output directory')
    out=out.resolve()
    if any((p/'.git').exists() for p in (out,*out.parents)):
        raise ValueError('Private assembly must remain outside Git')
    live._text(assembly_id,'assembly ID')
    requested_pacing = {'request_gap_us': request_gap_us, 'request_window': request_window}
    # Explicit preparation options select candidate settings only; they cannot
    # confer review or infer pacing from a diagnostic's outcome.
    live.transport_settings({'schema': live.SCHEMA_V2,
        'request_gap_us': 600 if request_gap_us is None else request_gap_us,
        'request_window': 3 if request_window is None else request_window})
    if power_epoch is not None and (type(power_epoch) is not str or not 0<len(power_epoch)<=128
            or power_epoch!=power_epoch.strip() or power_epoch in UNKNOWN_EPOCHS
            or not power_epoch.isprintable()):
        raise ValueError('Invalid operator-declared motor power epoch')
    pinned={};documents={};references={};bundle_members={}
    def pin(label,path):
        path=Path(path).expanduser().absolute()
        doc,digest=live._read_json(path)
        if type(doc) is not dict:raise ValueError('JSON object required: '+label)
        raw=path.read_bytes()
        if _sha(raw)!=digest:raise ValueError('Source changed while reading: '+label)
        path=path.resolve()
        pinned[label]=(path,raw);documents[label]=doc
        references[label]={'path':str(path),'sha256':digest}
        return doc
    for key,path in (('calibration',calibration),('angle_profile',angle_profile),('mount',mount),
                     ('bias',bias),('model_manifest',model_manifest),('pipeline_diagnostic',pipeline_diagnostic)):
        pin(key,path)
    ap,contracts=angles.load_profile(pinned['angle_profile'][0])
    _same(ap,documents['angle_profile'],'Angle profile reread')
    for digest,path in ap.get('evidence_files',{}).items():
        live._hash(digest,'angle evidence')
        path=Path(path).expanduser()
        path=path if path.is_absolute() else pinned['angle_profile'][0].parent/path
        pin('angle_evidence_'+digest,path)
        if references['angle_evidence_'+digest]['sha256']!=digest:
            raise ValueError('Angle evidence hash mismatch')
    cal=documents['calibration'];rows=live.shadow.validate_calibration(cal)
    if cal.get('approved_for_runtime') is not False:
        raise ValueError('Keep calibration candidate unapproved and unchanged')
    live.shadow.validate_imu_mount_candidate(documents['mount']);live._bias(documents['bias'])
    manifest=documents['model_manifest']
    if (manifest.get('schema')!='native-policy-overnight-v1' or manifest.get('status')!='VALIDATED_FILE_ONLY'
            or manifest.get('bundle_hashes')!=live.shadow.SOURCE_HASHES
            or any(manifest.get(k) is not False for k in ('output_allowed','approved_for_runtime','live_50hz_verified'))):
        raise ValueError('Pinned unapproved native-model equivalence manifest required')
    blockers={'missing_measurements':[],'file_assembly':[],'named_review':[]}
    def block(kind,code,ids=None):
        row={'code':code}
        if ids is not None:row['motor_ids']=list(ids)
        blockers[kind].append(row)
    if source_capture is not None:
        capture=pin('source_capture',source_capture);raw,uids=angles.current_values(capture)
        capture_sha=cal.get('source_capture_sha256',cal.get('source_sha256',{}).get('current_capture'))
        if capture_sha!=references['source_capture']['sha256']:
            raise ValueError('Calibration source-capture hash mismatch')
        _same(uids,cal['identities'],'Capture identities')
        _same(capture.get('boot_id'),cal.get('source_current_boot_id'),'Capture boot')
        if 'source_raw_rad_by_id' in cal:_same(raw,cal['source_raw_rad_by_id'],'Capture raw angles')
        if 'source_current_motor_power_epoch_label' in cal and capture.get('motor_power_epoch') is not None:
            _same(capture['motor_power_epoch'],cal['source_current_motor_power_epoch_label'],'Capture power epoch')
    else:
        capture=None;raw=cal.get('source_raw_rad_by_id')
        block('file_assembly','source_capture_file_not_pinned')
    if raw is not None and (type(raw) is not dict or set(raw)!=set(live.IDS)):
        raise ValueError('Embedded source raw angles need all twelve axes')
    source_contracts=cal.get('source_calibration_sha256_by_id')
    if source_contracts is not None:
        _same(source_contracts,{mid:references['angle_profile']['sha256'] for mid in live.IDS},
              'Source angle-profile hashes')
    elif ap.get('source_sha256',{}).get('candidate')!=references['calibration']['sha256']:
        block('file_assembly','calibration_to_angle_profile_source_binding_needs_review')
    profile=live.template(schema=profile_schema);profile['assembly_id']=assembly_id
    physical_status={}
    for mid in live.IDS:
        axis=contracts[int(mid)];candidate=rows[int(mid)]
        if axis.uid!=cal['identities'][mid] or axis.sign!=candidate['sign_candidate']:
            raise ValueError('Candidate/angle-profile UID or sign mismatch: ID'+mid)
        turns=candidate.get('diagnostic_branch_turns_embedded_in_offset',
                            candidate.get('reviewed_branch_turns_embedded_in_offset'))
        if type(turns) is not int:raise ValueError('Explicit integer candidate branch required: ID'+mid)
        expected=axis.offset_rad-axis.sign*turns*2*math.pi
        if not math.isclose(expected,candidate['offset_candidate_rad'],rel_tol=0,abs_tol=1e-10):
            raise ValueError('Candidate/angle-profile offset mismatch: ID'+mid)
        if raw is not None:
            options=equivalent_branch_candidates(raw[mid],axis)
            if (len(options)!=1 or options[0]['turns']!=turns
                    or not options[0]['whole_uncertainty_inside_limits']):
                raise ValueError('Source raw angle does not identify candidate branch: ID'+mid)
        else:block('file_assembly','source_raw_branch_not_recomputed',[int(mid)])
        row=profile['axes'][mid]
        row.update(uid=axis.uid,sign=axis.sign,offset_rad=candidate['offset_candidate_rad'])
        if axis.zero_reviewed:
            row['uncertainty_rad']=live._number(axis.uncertainty_rad,'physical uncertainty',1e-6,.0873)
        if axis.physical_limits_reviewed:
            index=live.shadow.CAN_ORDER.index(int(mid))
            if not live.shadow.LOWER[index]<=axis.lower_rad<axis.upper_rad<=live.shadow.UPPER[index]:
                raise ValueError('Reviewed physical range exceeds model bounds: ID'+mid)
            row.update(physical_lower_rad=axis.lower_rad,physical_upper_rad=axis.upper_rad)
        if (row['uncertainty_rad'] is not None and row['physical_lower_rad'] is not None
                and row['physical_lower_rad']+row['uncertainty_rad']>=row['physical_upper_rad']-row['uncertainty_rad']):
            raise ValueError('Uncertainty consumes reviewed physical range: ID'+mid)
        physical_status[mid]={k:getattr(axis,k+'_reviewed') for k in ('zero','direction','physical_limits')}
    for part in ('zero','direction','physical_limits'):
        ids=[int(mid) for mid in live.IDS if not physical_status[mid][part]]
        if ids:block('named_review','existing_'+part+'_evidence_needs_review',ids)
    # A missing recorded physical bound is distinct from a missing review checkbox.
    ids=[int(mid) for mid in live.IDS if profile['axes'][mid]['uncertainty_rad'] is None]
    if ids:block('missing_measurements','physical_zero_error_bound_not_available',ids)
    ids=[int(mid) for mid in live.IDS if profile['axes'][mid]['physical_lower_rad'] is None]
    if ids:block('missing_measurements','bounded_path_range_clearance_not_available',ids)
    settings=settings_template() if group_settings is None else pin('group_settings',group_settings)
    _apply_settings(profile,settings)
    for key,value in requested_pacing.items():
        if value is not None:
            if key in settings['run_settings'] and settings['run_settings'][key] != value:
                raise ValueError('Explicit pacing conflicts with group settings: '+key)
            profile[key]=value
    live._settings(profile)
    execution=live.execution_settings(profile)
    if execution['model_backend']==live.SCALAR_BACKEND:
        if scalar_step_manifest is None:
            raise ValueError('Explicit scalar-step manifest required for selected backend')
        scalar=pin('scalar_step_manifest',scalar_step_manifest)
        if (scalar.get('schema')!='native-step-scalar-file-only-v1'
                or scalar.get('status')!='PASS_FILE_ONLY_COMPARE'
                or scalar.get('baseline_manifest_sha256')!=references['model_manifest']['sha256']
                or any(scalar.get(k) is not False for k in
                    ('hardware_opened','output_allowed','approved_for_runtime','live_50hz_verified'))):
            raise ValueError('Pinned scalar file-only manifest must match the exact baseline')
    elif scalar_step_manifest is not None:
        raise ValueError('Scalar-step manifest requires explicit scalar backend selection')
    if all(value is not None for row in profile['axes'].values() for value in row.values()):
        # Validate complete numeric preparations against the same live contract,
        # without mutating its exact file schema or creating physical approval.
        live._axes(copy.deepcopy(profile),cal)
    for group,ids in GROUPS.items():
        missing=[k for k in live.LIMIT_CAPS if profile['axes'][str(ids[0])][k] is None]
        if missing:block('file_assembly','explicit_'+group+'_settings_missing:'+','.join(sorted(missing)),ids)
    for key in ('calibration','mount','bias','model_manifest','pipeline_diagnostic',
                *(('scalar_step_manifest',) if scalar_step_manifest is not None else ())):
        profile['artifacts'][key]=dict(references[key])
    report=documents['pipeline_diagnostic']
    for key,name in (('calibration','calibration'),('mount','mount'),('gyro_bias','bias')):
        actual=report.get('input_sha256',{}).get(key)
        if actual is not None and actual!=references[name]['sha256']:
            raise ValueError('Diagnostic input hash mismatch: '+key)
    model_hash=report.get('model_source',{}).get('manifest_sha256')
    model_key='scalar_step_manifest' if execution['model_backend']==live.SCALAR_BACKEND else 'model_manifest'
    if model_hash is not None and model_hash!=references[model_key]['sha256']:
        raise ValueError('Diagnostic model hash mismatch')
    if model_key=='scalar_step_manifest':
        baseline_hash=report.get('model_source',{}).get('baseline_provenance',{}).get('manifest_sha256')
        if baseline_hash is not None and baseline_hash!=references['model_manifest']['sha256']:
            raise ValueError('Diagnostic scalar baseline hash mismatch')
    try:timing=live._timing(report,profile)
    except (ValueError,TypeError,KeyError) as error:
        timing={'status':'NOT_ELIGIBLE','reason':str(error)}
        block('missing_measurements','full_pipeline_diagnostic_not_eligible')
    boot=report.get('boot_id')
    try:valid_boot=type(boot) is str and str(uuid.UUID(boot))==boot
    except ValueError:valid_boot=False
    diagnostic_context=(report.get('status')=='COMPLETE_DIAGNOSTIC' and report.get('mode')=='stop-proxy'
        and report.get('errors')==[] and report.get('motor_enable_sent') is False
        and report.get('learned_targets_sent') is False and not any(report.get(k) is True
        for k in ('simulated','simulation_only','replayed','synthetic','all_new_timestamps_are_simulated'))
        and report.get('hardware_opened') is not False)
    if valid_boot and diagnostic_context and boot==cal.get('source_current_boot_id'):
        profile['boot_id']=boot
    else:block('file_assembly','current_boot_binding_unavailable_or_inconsistent')
    labels=[v for v in (report.get('motor_power_epoch'),cal.get('source_current_motor_power_epoch_label'),
        capture.get('motor_power_epoch') if capture else None) if type(v) is str and v==v.strip() and v not in UNKNOWN_EPOCHS]
    if power_epoch is not None:
        if not profile['boot_id'] or capture is None or capture.get('boot_id')!=profile['boot_id']:
            raise ValueError('Operator-declared power epoch needs pinned capture/calibration and hardware diagnostic from the same boot')
        if any(label!=power_epoch for label in labels):
            raise ValueError('Operator-declared power epoch conflicts with source label')
        profile['motor_power_epoch']=power_epoch
    elif profile['boot_id'] and labels and len(set(labels))==1:profile['motor_power_epoch']=labels[0]
    else:block('file_assembly','explicit_motor_power_epoch_unavailable_or_inconsistent')
    epoch_binding={'label':profile['motor_power_epoch'],
        'source':'operator_declared' if power_epoch is not None else
            ('source_labels' if profile['motor_power_epoch'] is not None else 'unresolved'),
        'operator_declared_label':power_epoch,'source_labels':labels,
        'matched_boot_id':profile['boot_id'],
        'physical_power_transition_verified_by_this_tool':False,
        'note':'An operator label associates these files; it is not a measurement of motor power transitions or permission to drive.'}
    # A template's current source pins do not upgrade an old diagnostic. Preserve
    # its timing numbers for review while keeping missing/stale bindings explicit.
    diagnostic_binding={
        'boot_matches_capture_and_calibration': bool(profile['boot_id'] and capture is not None
            and capture.get('boot_id')==profile['boot_id']),
        'motor_power_epoch_matches': bool(profile['motor_power_epoch'] and
            report.get('motor_power_epoch')==profile['motor_power_epoch']),
        'cadence_sources_match': None,
        'freshness_or_physical_power_transition_verified': False,
    }
    binding_blockers=[]
    if not diagnostic_binding['boot_matches_capture_and_calibration']:
        binding_blockers.append('diagnostic_boot_not_bound_to_capture_and_calibration')
    if not diagnostic_binding['motor_power_epoch_matches']:
        binding_blockers.append('diagnostic_motor_power_epoch_unavailable_or_inconsistent')
    if profile['schema']==live.SCHEMA_V3:
        diagnostic_binding['cadence_sources_match']=(
            report.get('cadence_source_sha256')==profile['cadence_source_sha256'])
        if not diagnostic_binding['cadence_sources_match']:
            binding_blockers.append('diagnostic_cadence_sources_unavailable_or_inconsistent')
    for code in binding_blockers:block('missing_measurements',code)
    if binding_blockers and timing.get('status')!='NOT_ELIGIBLE':
        timing={'status':'NOT_ELIGIBLE','reason':'Diagnostic provenance is not bound to this preparation',
            'binding_blockers':binding_blockers,'timestamp_review':timing}
    if bundle is not None:
        bundle=Path(bundle).expanduser().absolute()
        for name,digest in live.shadow.SOURCE_HASHES.items():
            path=bundle/name
            if path.is_symlink() or not path.is_file() or _sha(path.read_bytes())!=digest:
                raise ValueError('Model bundle source mismatch: '+name)
            bundle_members[path.resolve()]=digest
        profile['bundle_path']=str(bundle.resolve())
    else:block('file_assembly','explicit_pinned_bundle_path_required')
    encoder=live.native_batch_encoder_settings(profile)
    if encoder is not None:
        if bundle is None:
            raise ValueError('Selected native batch encoder requires its explicit pinned bundle')
        path=bundle/encoder['path']
        if path.is_symlink() or not path.is_file() or _sha(path.read_bytes())!=encoder['sha256']:
            raise ValueError('Native batch encoder binary mismatch')
        bundle_members[path.resolve()]=encoder['sha256']
    imu={key:None for key in IMU_PHYSICAL}
    imu.update(gravity_direction_max_error_rad=None,corrected_static_gyro_max_rad_s=None,
        raw_gravity_norm_min_m_s2=None,raw_gravity_norm_max_m_s2=None,norm_deviation_rationale='')
    if imu_fragment is not None:
        fragment=pin('imu_fragment',imu_fragment)
        if (fragment.get('schema')!='singularitydog.imu-review-preparation.v1' or fragment.get('status')!='UNREVIEWED'
                or fragment.get('approved_for_runtime') is not False or fragment.get('dependency_eligible') is not False
                or fragment.get('hardware_opened') is not False):raise ValueError('Only an unreviewed IMU fragment is accepted')
        for key in ('mount','bias'):
            if fragment.get('references',{}).get(key,{}).get('sha256')!=references[key]['sha256']:
                raise ValueError('IMU fragment input mismatch: '+key)
        values=fragment.get('imu',{})
        for key in ('corrected_static_gyro_max_rad_s','raw_gravity_norm_min_m_s2','raw_gravity_norm_max_m_s2'):
            value=values.get(key)
            if value is not None:live._number(value,'IMU '+key,0,12)
            imu[key]=value
        # Physical declarations, rationale and direction-error claims stay pending.
    block('named_review','imu_physical_direction_bias_gravity_and_norm_review_pending')
    block('missing_measurements','type2_dynamic_evidence_to_attach',range(1,13))
    block('missing_measurements','actual_command_loss_and_usb_disconnect_evidence_to_attach',range(1,13))
    block('named_review','named_supported_profile_and_hardware_review_pending')
    if execution['voltage_pipeline']:
        block('named_review','feedback_then_voltage_pipeline_acceptance_pending')
    if encoder is not None:
        block('named_review','native_batch_encoder_acceptance_pending')
    hardware={'schema':live.REVIEW_SCHEMA,'scope':profile['scope'],'review':None,
        'assembly_id':assembly_id,'uids_by_id':dict(cal['identities']),
        'reviewed_settings_sha256':live.reviewed_settings_sha256(profile),
        'artifact_sha256':{k:profile['artifacts'][k]['sha256'] for k in live.artifact_names(profile) if k!='hardware_review'},
        'source_captures':list(references.values()),'angles':{},'type2_dynamic':{},'device_watchdog':{},
        'imu':imu,'mode0_readback_required_before_enable':True,'timing_budget_rationale':''}
    if execution['voltage_pipeline']:
        hardware['voltage_pipeline_acceptance']={'pipeline':'feedback_then_voltage.fast_v1',
            'diagnostic_sha256':references['pipeline_diagnostic']['sha256'],'scope':profile['scope'],
            'hard_output_and_freshness_limits_unchanged':None,'review':None}
    if encoder is not None:
        hardware['native_batch_encoder_acceptance']={'binary_sha256':encoder['sha256'],
            'scope':profile['scope'],'hard_output_and_freshness_limits_unchanged':None,'review':None}
    for mid,row in profile['axes'].items():
        hardware['angles'][mid]={key:row[key] for key in ('sign','offset_rad','uncertainty_rad','physical_lower_rad','physical_upper_rad')}
        hardware['angles'][mid].update(zero_and_sign_physically_verified=None,
            physical_range_and_clearance_verified=None,power_cycle_branch_method_verified=None)
        hardware['type2_dynamic'][mid]={'output_shaft_position_verified':None,'velocity_scale_and_sign_verified':None,
            'torque_interpretation_verified':None,'position_range_rad':[-12.57,12.57],
            'velocity_range_rad_s':[-50.,50.],'torque_range_nm':[-5.5,5.5]}
        hardware['device_watchdog'][mid]={'motor_model':'RS05','actual_command_loss_test_passed':None,
            'usb_disconnect_test_passed':None,'disabled_after_loss_verified':None,'configured_timeout_ms':None,
            'max_observed_disable_ms':None,'version_bytes_hex':None,'firmware_version':None}
    hardware_bytes=_json_bytes(hardware)
    profile['artifacts']['hardware_review']={'path':str(out/'hardware-review.json'),'sha256':_sha(hardware_bytes)}
    profile['blockers']=[kind+':'+row['code'] for kind,items in blockers.items() for row in items]
    live._structure(profile);live._settings(profile)
    result={'schema':SCHEMA,'status':'UNAPPROVED_ASSEMBLY','output_allowed':False,'approved_for_runtime':False,
        'hardware_opened':False,'source_files_modified':False,'references':references,
        'blockers_by_kind':blockers,'physical_evidence_status_by_id':physical_status,
        'timing_diagnostic_review_only':timing,'group_ids':GROUPS,
        'transport_settings':live.transport_settings(profile),
        'execution_settings':execution,
        'telemetry_cadence':live.telemetry_settings(profile),
        'motor_power_epoch_binding':epoch_binding,
        'diagnostic_binding':diagnostic_binding,
        'calibration_source_capture_pinned':source_capture is not None,
        'instructions':['Review existing evidence first; missing attachment is not proof a measurement was never taken.',
            'Fill physical review from observations, never replace null with true to bypass a missing test.',
            'Changing settings/evidence requires rebinding reviewed_settings_sha256 and artifact hashes.',
            'The original calibration stays unapproved; the separate named hardware review is required.']}
    for path,raw in pinned.values():
        if path.is_symlink() or path.read_bytes()!=raw:raise ValueError('Source changed during assembly: '+str(path))
    for path,digest in bundle_members.items():
        if path.is_symlink() or not path.is_file() or _sha(path.read_bytes())!=digest:
            raise ValueError('Model bundle source changed during assembly: '+str(path))
    # Exclusive directory publication, only after all validations and source rechecks.
    files={'profile.json':_json_bytes(profile),'hardware-review.json':hardware_bytes,
        'group-settings-template.json':_json_bytes(settings_template()),'preparation.json':_json_bytes(result)}
    out.mkdir(mode=0o700,parents=True,exist_ok=False);os.chmod(out,0o700)
    for name,raw in files.items():
        with os.fdopen(os.open(out/name,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'wb') as stream:stream.write(raw)
    # Public CLI loader must accept PLAN while retaining all null physical fields.
    loaded=live.load_profile(out/'profile.json',require_approved=False)
    if loaded['output_allowed'] is not False:raise AssertionError('Preparation must never authorize output')
    return {'status':result['status'],'output':str(out),'profile':str(out/'profile.json'),
        'profile_sha256':_sha(files['profile.json']),'output_allowed':False,'hardware_opened':False,
        'transport_settings':live.transport_settings(profile),
        'execution_settings':execution,
        'blockers_by_kind':blockers}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('calibration','angle-profile','mount','bias','model-manifest','pipeline-diagnostic','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--assembly-id',required=True)
    parser.add_argument('--power-epoch',help='Operator-declared current motor-power label (not a physical power measurement)')
    live.add_transport_arguments(parser,reviewed=False)
    parser.add_argument('--profile-schema',choices=(live.SCHEMA_V2,live.SCHEMA_V3),default=live.SCHEMA,
                        help='V3 cadence must be explicitly selected and remains unapproved')
    for name in ('imu-fragment','group-settings','source-capture','bundle','scalar-step-manifest'):
        parser.add_argument('--'+name,type=Path)
    args=vars(parser.parse_args(argv));print(json.dumps(prepare(**args),ensure_ascii=False,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
