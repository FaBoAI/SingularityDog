#!/usr/bin/env python3
"""Prepare exactly one file-only boxed native890 2 -> 10 -> 20 stage under direct permission.

Default PLAN writes nothing and never opens devices or loads a policy/library.
--prepare saves a fresh profile after current source, capture, watchdog/diagnostic
and original predecessor evidence checks. The existing runtime still checks the
live starting pose, UID, fault/mode, input freshness and final STOP independently.
Permission is not a future observation: audio/anomalies/support stay UNKNOWN.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
if not sys.dont_write_bytecode:
    raise RuntimeError('Launch the file-only entry with python -B (or PYTHONDONTWRITEBYTECODE=1)')
sys.path.insert(0, str(ROOT/'runtime'))
from singularitydog_hw import policy_live_profile as live

MODES = {2:live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
         10:live.SUPPORTED_POLICY_PROBE_10S_AFTER_2S,
         20:live.SUPPORTED_POLICY_PROBE_20S_AFTER_10S}


def read_pin(path, sha):
    path=Path(path).expanduser().absolute()
    live._need(type(sha) is str and len(sha)==64 and all(c in '0123456789abcdef' for c in sha),
               'Explicit lowercase SHA256 required')
    value, actual=live._read_json(path)
    live._need(actual==sha, 'Explicit input SHA256 mismatch: '+str(path))
    return value, dict(path=str(path),sha256=sha)


def write(path,value):
    raw=(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:stream.write(raw)
    return dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest())


def absolute_refs(value,base):
    if type(value) is dict:
        if set(value)=={'path','sha256'}:
            path=Path(value['path']);value['path']=str(path if path.is_absolute() else (base/path).absolute())
        else:
            for item in value.values():absolute_refs(item,base)
    elif type(value) is list:
        for item in value:absolute_refs(item,base)


def build(args):
    """Read/pin original files only; no inferred motor epoch or future outcomes."""
    profile, profile_ref=read_pin(args.profile,args.profile_sha256)
    live._structure(profile)
    live._need(profile['approved_for_supported_policy_output'] is True and profile['blockers']==[],
               'Current fully reviewed native boxed baseline/predecessor required')
    live._need(profile.get('native_phase_pair') is True and
               not live.preauthorized_boxed_sequence_selected(profile),
               'Legacy boxed authorization cannot become native authorization')
    kit=Path(args.kit_runtime).expanduser().absolute()
    live._need(kit==Path(live.__file__).resolve().parents[1], 'Use the explicit selected kit runtime for this entry')
    manifest, manifest_ref=read_pin(args.source_manifest,args.source_manifest_sha256)
    source=live.cadence_source_hashes(profile)
    live._need(profile['cadence_source_sha256']==source and type(manifest.get('files')) is dict and
               all(manifest['files'].get('runtime/'+name,{}).get('sha256')==sha for name,sha in source.items()),
               'Fresh diagnostic and current profile must pin this exact source kit')
    permission, permission_ref=read_pin(args.preauthorization,args.preauthorization_sha256)
    capture,capture_ref=read_pin(args.current_capture,args.current_capture_sha256)
    base=Path(profile_ref['path']).parent
    data=copy.deepcopy(profile);absolute_refs(data['artifacts'],base)
    if not Path(data['bundle_path']).is_absolute():data['bundle_path']=str((base/data['bundle_path']).absolute())
    docs={name:live._artifact(ref,base)[0] for name,ref in data['artifacts'].items()}
    duration=args.duration
    if duration == 20:
        live._need(args.pipeline_diagnostic and args.pipeline_diagnostic_sha256,
                   'Native twenty seconds requires a fresh post-ten-second diagnostic')
        diagnostic, diagnostic_ref=read_pin(args.pipeline_diagnostic,args.pipeline_diagnostic_sha256)
        data['artifacts']['pipeline_diagnostic']=diagnostic_ref
        docs['pipeline_diagnostic']=diagnostic
    live._need(duration in MODES, 'Only boxed native890 2/10/20 durations supported')
    original_duration=2 if duration==2 else 2 if duration==10 else 10
    live._need(profile['duration_s']==original_duration and profile['diagnostic_timing_acceptance']==MODES[original_duration],
               'Numeric predecessor must be the exact immediately preceding duration')
    data.update(duration_s=float(duration),diagnostic_timing_acceptance=MODES[duration],preauthorized_native_boxed_sequence=True)
    live._preauthorized_native_boxed_sequence_scope(data)
    data['review']={**data['review'], 'reviewer':'Codex file-only preauthorized boxed sequence admission',
        'reviewed_at':datetime.now(timezone.utc).isoformat(),
        'rationale':'Explicit original boxed native890 2/10/20 permission; same source/model/epoch/control contract. '
                    'Fresh current numeric pose, complete original predecessor/STOP/restore and loader remain required. '
                    'Future audio/anomaly/support observations remain UNKNOWN; no load transfer, standing or walking.'}
    inputs=[profile_ref,manifest_ref,permission_ref,capture_ref,*copy.deepcopy(list(data['artifacts'].values()))]
    if duration==2:
        live._need(permission.get('schema')==live.NATIVE_BOXED_SEQUENCE_CONTEXT_SCHEMA and
                   permission.get('post_trial_physical_observation_inferred') is False and
                   permission.get('physical_anomaly_after_new_trials','missing') is None and
                   permission.get('physical_audio_heard_after_new_trials','missing') is None,
                   'Original preserved sequence instruction required; future facts must remain unknown')
        live._need(permission.get('authorized_durations_s') == [2,10,20] and
                   permission.get('native_phase_pair') is True and permission.get('request_gap_us') == 890 and
                   permission.get('request_window') == 3 and
                   permission.get('automatic_continuation_explicitly_authorized') is True and
                   permission.get('post_trial_confirmation_questions_waived') is True and
                   all(permission.get(k) is False for k in
                       ('box_removal_allowed','load_transfer_allowed','standing_allowed','walking_allowed')),
                   'Explicit native890 boxed2/10/20 source context required')
        conditions,conditions_ref=live._artifact(permission.get('observation_reference'),Path(permission_ref['path']).parent)
        instruction=permission.get('sequence_authorization_source',{})
        live._need(instruction.get('kind')=='latest_direct_user_message_in_codex' and
                   type(instruction.get('exact_text')) is str and bool(instruction['exact_text'].strip()),
                   'Original direct human sequence text required')
        human=dict(source='direct_current_user_reply_in_codex',direct_human=True,synthetic_interaction=False,
            user_statement=instruction['exact_text'],user_reply_id=instruction.get('user_reply_id',permission_ref['sha256']),
            boot_id=data['boot_id'],motor_power_epoch=data['motor_power_epoch'],authorized_durations_s=[2,10,20],
            automatic_continuation_explicitly_authorized=True,post_trial_confirmation_questions_waived=True,
            source_context=permission_ref,
            interpretation='Current explicit native boxed 2,10,20 continuation permission; never post-trial physical evidence')
        review={**data['review'],'decision':'AUTHORIZE_NATIVE_BOXED_2_10_20_WITH_UNKNOWN_POST_OBSERVATIONS',
            'rationale':'Direct sequence instruction and present conditions; all future physical outcomes UNKNOWN'}
        auth=dict(schema=live.NATIVE_BOXED_SEQUENCE_AUTHORIZATION_SCHEMA,scope='native_boxed_small_sequence_only',
            authorized_durations_s=[2,10,20],boot_id=data['boot_id'],motor_power_epoch=data['motor_power_epoch'],
            assembly_id=data['assembly_id'],reference_capture_sha256=data['artifacts']['local_reference_capture']['sha256'],
            sequence_contract_sha256=live.preauthorized_boxed_sequence_contract_sha256(data),
            source_manifest=manifest_ref,source_manifest_sha256=manifest_ref['sha256'],
            human_authorization_source=None,current_conditions_source=conditions_ref,review=review,
            automatic_continuation_explicitly_authorized=True,post_trial_confirmation_questions_waived=True,
            future_physical_observations_must_remain_unknown=True,native_phase_pair=True,
            request_gap_us=890,request_window=3,box_removal_allowed=False,
            load_transfer_allowed=False,standing_allowed=False,walking_allowed=False)
        inputs.append(conditions_ref)
        receipt=None
    else:
        live._need(live.preauthorized_native_boxed_sequence_selected(profile) and
                   permission.get('schema')==live.NATIVE_BOXED_SEQUENCE_AUTHORIZATION_SCHEMA and
                   profile['artifacts']['boxed_sequence_authorization']['sha256']==permission_ref['sha256'],
                   'Retain the same original sequence permission; legacy observations cannot substitute')
        auth=permission;human=None
        live._need(args.prior_report and args.prior_report_sha256 and args.prior_execution_receipt and args.prior_execution_receipt_sha256,
                   'Original completed predecessor report and execution receipt pins required')
        report,report_ref=read_pin(args.prior_report,args.prior_report_sha256)
        receipt,receipt_ref=read_pin(args.prior_execution_receipt,args.prior_execution_receipt_sha256)
        inputs.extend([report_ref,receipt_ref])
        docs.update(prior_supported_profile=copy.deepcopy(profile),prior_supported_report=report,
            prior_supported_observation=dict(schema=live.NATIVE_BOXED_SEQUENCE_NUMERIC_RESULT_SCHEMA,
                source='authenticated_original_supported_report',synthetic_interaction=False,
                profile_sha256=profile_ref['sha256'],report_sha256=report_ref['sha256'],
                preauthorization_sha256=permission_ref['sha256'],boot_id=data['boot_id'],motor_power_epoch=data['motor_power_epoch'],
                numeric_result_only=True,physical_result_inferred=False,post_trial_audio_heard=None,
                post_trial_anomalies=None,post_trial_support_maintained=None,execution_receipt=receipt_ref))
        for name,ref in (('prior_supported_profile',profile_ref),('prior_supported_report',report_ref)):
            data['artifacts'][name]=ref
        docs['hardware_review']['supported_extension_acceptance']=dict(
            mode=MODES[duration],scope=data['scope'],only_duration_extended=True,live_limits_unchanged=True,
            support_must_remain=True,load_bearing_not_established=True,walking_allowed=False,
            review={**data['review'],'decision':'ACCEPT_10S_SUPPORTED_AFTER_2S' if duration==10 else
                'ACCEPT_20S_SUPPORTED_AFTER_10S',
                'rationale':'Original numeric completion/STOP/restore and new read-only pose; future physical observations UNKNOWN'})
    docs.update(boxed_sequence_authorization=auth,boxed_sequence_current_capture=capture)
    data['artifacts']['boxed_sequence_current_capture']=capture_ref
    return data,docs,human,permission_ref,inputs


def prepare(args):
    data,docs,human,permission_ref,inputs=build(args)
    out=Path(args.output).expanduser().absolute()
    live._need(not out.exists() and not any(path.is_symlink() for path in (out,*out.parents)) and
               not any((path/'.git').exists() for path in (out,*out.parents)), 'Fresh private output outside Git required')
    plan=dict(schema='singularitydog.preauthorized-native-boxed-stage-preparation.v1',status='PLAN_ONLY',
        duration_s=args.duration,inputs=inputs,output_directory=str(out),output_allowed=False,
        hardware_opened=False,motor_enable_sent=False,learned_targets_sent=False,
        post_trial_physical_observation=dict(audio_heard=None,anomalies=None,support_maintained=None),
        box_removal_allowed=False,load_transfer_verified=False,standing_verified=False,walking_verified=False)
    if not args.prepare:return plan
    out.mkdir(parents=True,mode=0o700)
    try:
        return _publish(args,data,docs,human,permission_ref,inputs,out,plan)
    except BaseException:
        # Only this fresh private attempt belongs to us. No original input or
        # pre-existing directory is removed if validation/publication fails.
        shutil.rmtree(out)
        raise


def _publish(args,data,docs,human,permission_ref,inputs,out,plan):
    if human is not None:
        docs['boxed_sequence_authorization']['human_authorization_source']=write(out/'original-human-instruction.json',human)
        data['artifacts']['boxed_sequence_authorization']=write(out/'boxed_sequence_authorization.json',docs['boxed_sequence_authorization'])
    else:data['artifacts']['boxed_sequence_authorization']=permission_ref
    if args.duration>2:
        ref=write(out/'prior_supported_observation.json',docs['prior_supported_observation'])
        data['artifacts']['prior_supported_observation']=ref
        acceptance=docs['hardware_review']['supported_extension_acceptance']
        for field,key in (('prior_profile_sha256','prior_supported_profile'),('prior_report_sha256','prior_supported_report'),
                          ('prior_observation_sha256','prior_supported_observation')):
            acceptance[field]=data['artifacts'][key]['sha256']
    # Every acceptance must bind the actual new diagnostic, without creating
    # diagnostic evidence or changing any numerical admission condition.
    if args.duration == 20:
        for key in ('post_reply_deadline_acceptance','rare_jitter_diagnostic_acceptance',
                    'voltage_pipeline_acceptance','native_batch_encoder_acceptance',
                    'startup_cycle_acceptance','prepared_voltage_publication_acceptance',
                    'native_phase_pair_acceptance'):
            if key in docs['hardware_review']:
                docs['hardware_review'][key]['diagnostic_sha256']=data['artifacts']['pipeline_diagnostic']['sha256']
    for name in ('operator_acceptance','hardware_review'):
        document=docs[name];absolute_refs(document,Path(args.profile).expanduser().absolute().parent)
        document['reviewed_settings_sha256']=live.reviewed_settings_sha256(data)
        excluded={'operator_acceptance','hardware_review'} if name=='operator_acceptance' else {'hardware_review'}
        document['artifact_sha256']={key:ref['sha256'] for key,ref in data['artifacts'].items() if key not in excluded}
        data['artifacts'][name]=write(out/(name+'.json'),document)
    profile_ref=write(out/'profile.json',data)
    loaded=live.load_profile(profile_ref['path'])
    for ref in inputs:read_pin(ref['path'],ref['sha256'])
    plan.update(status='PREPARED_PREAUTHORIZED_BOXED_STAGE',profile=profile_ref,
        profile_loader_accepted=loaded['output_allowed'],
        preauthorization=data['artifacts']['boxed_sequence_authorization'],
        sequence_contract_sha256=live.preauthorized_boxed_sequence_contract_sha256(data),
        recorded_at=datetime.now(timezone.utc).isoformat())
    write(out/'preparation-receipt.json',plan)
    return plan


def parser():
    result=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('profile','profile-sha256','preauthorization','preauthorization-sha256',
                 'current-capture','current-capture-sha256','source-manifest','source-manifest-sha256',
                 'kit-runtime','output'):
        result.add_argument('--'+name,required=True)
    result.add_argument('--duration',type=int,choices=(2,10,20),required=True)
    for name in ('prior-report','prior-report-sha256','prior-execution-receipt','prior-execution-receipt-sha256',
                 'pipeline-diagnostic','pipeline-diagnostic-sha256'):
        result.add_argument('--'+name)
    result.add_argument('--prepare',action='store_true')
    return result


def main(argv=None):
    args=parser().parse_args(argv)
    try:
        print(json.dumps(prepare(args),sort_keys=True,indent=2,allow_nan=False));return 0
    except (ValueError,OSError,RuntimeError) as error:
        print(json.dumps(dict(status='REJECTED_FILE_PREPARATION',errors=[str(error)],output_allowed=False,
            hardware_opened=False,motor_enable_sent=False,learned_targets_sent=False),allow_nan=False));return 2


if __name__=='__main__':raise SystemExit(main())
