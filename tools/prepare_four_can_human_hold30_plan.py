"""Build and inspect the separate human-hold30 PLAN from SHA-pinned actual files.

File only: no SSH, device, native library, model inference, motor command,
operator confirmation or physical observation is created. Current human facts
must already exist as a separately pinned direct-reply document. Missing facts
and missing fresh live records remain explicit blockers in the resulting PLAN.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys

RUNTIME = Path(__file__).resolve().parents[1]/'runtime'
if str(RUNTIME) not in sys.path:
    sys.path.insert(0, str(RUNTIME))
from experiments.four_bus_human_hold import profile as H
from experiments.four_bus_type1 import type1_profile as B

REFS_SCHEMA = 'singularitydog.four-can-human-hold30-evidence-inputs.v1'
REVIEW_SCHEMA = 'singularitydog.four-can-human-hold30-file-plan-review.v1'
FALSE_FLAGS = dict(hardware_opened=False, motor_enable_sent=False, type1_sent=False,
    positive_gain_sent=False, learned_targets_sent=False, output_allowed=False,
    physical_facts_created=False, human_conditions_created=False,
    live_timing_records_created=False, standing_or_walking_authorized=False)


def need(value, message):
    if not value:
        raise ValueError(message)


def reference(path, digest):
    return H._ref({'path':str(path), 'sha256':digest}, 'builder input')


def inspect_assets(plan, pins):
    """Recheck independent source/native/audio/prior bytes without current-fact inference."""
    evidence, contract = plan['evidence'], plan['contract']
    base = contract['base_numerical_contract']
    result = {'source':None, 'integration_native':None, 'audio':None,
        'prior_boxed':None, 'errors':[], 'missing':[], **FALSE_FLAGS}
    manifest = None
    source_keys = {'source_root','source_count','source_manifest','source_review'}
    if source_keys <= set(evidence):
        try:
            need(evidence['source_manifest']==base['source_manifest'],
                 'Evidence source inventory differs from the supplied numerical model contract')
            manifest=H.verify_manifest(evidence['source_root'],evidence['source_manifest'],
                                       evidence['source_count'],pins)
            document=pins.json(evidence['source_review'])
            need(document.get('schema')==H.SOURCE_REVIEW_SCHEMA and document.get('scope')==H.SCOPE and
                 document.get('source_manifest_sha256')==evidence['source_manifest']['sha256'] and
                 document.get('decision')=='REVIEWED_HUMAN_HOLD30_SOURCE_ONLY' and
                 document.get('old_boxed_or_two_bus_permission_reused') is False and
                 document.get('tests_grant_output_permission') is False,
                 'Independent source-only review is absent or has another frozen source')
            for key in ('reviewer','reviewed_at','rationale'):
                H._text(document.get(key),'source review '+key)
            result['source']={'status':'PINNED_SOURCE_ONLY_REVIEW_CHECKED',
                'source_manifest_sha256':evidence['source_manifest']['sha256']}
        except Exception as error:
            result['errors'].append('source: '+type(error).__name__+': '+str(error))
    else:
        result['missing'].extend(sorted(source_keys-set(evidence)))
    if manifest is not None and 'implementation_receipt' in evidence:
        try:
            receipt=pins.json(evidence['implementation_receipt'])
            paths=('runtime/experiments/four_bus_human_hold/runner.py',
                   'runtime/experiments/four_bus_human_hold/type1_runner.py',
                   'runtime/experiments/four_bus_human_hold/profile.py',
                   'runtime/experiments/four_bus_human_hold/supervision.py')
            need(receipt.get('schema')==H.IMPLEMENTATION_SCHEMA and receipt.get('scope')==H.SCOPE and
                 receipt.get('source_manifest_sha256')==evidence['source_manifest']['sha256'] and
                 receipt.get('runtime_entry')==paths[0] and receipt.get('independent_type1_runner')==paths[1] and
                 receipt.get('source_sha256')=={path:manifest['files'][path]['sha256'] for path in paths} and
                 receipt.get('decision')=='REVIEWED_INDEPENDENT_HUMAN_HOLD30_INTEGRATION' and
                 receipt.get('tests_grant_output_permission') is False and
                 receipt.get('old_boxed_permission_reused') is False,
                 'Exact independent integration receipt required')
            for key in ('reviewer','reviewed_at','rationale'):
                H._text(receipt.get(key),'integration review '+key)
            H._verify_native_integration(receipt,evidence['source_root'],manifest,pins)
            result['integration_native']={'status':'PINNED_SEPARATE_ADAPTER_AND_NATIVE_BUILDS_CHECKED'}
        except Exception as error:
            result['errors'].append('integration_native: '+type(error).__name__+': '+str(error))
    else:
        result['missing'].append('implementation_receipt_or_checked_source_inventory')
    if 'audio_manifest' in evidence:
        try:
            document=pins.json(evidence['audio_manifest'])
            need(document.get('contract_sha256')==plan['contract_sha256'],
                 'Audio manifest has another exact human contract')
            result['audio']={'status':'HASHED_FIVE_STAGE_PCM_AND_DURATIONS_CHECKED',
                **H._audio(document,contract['runtime_profile'],pins)}
        except Exception as error:
            result['errors'].append('audio: '+type(error).__name__+': '+str(error))
    else:
        result['missing'].append('audio_manifest')
    prior_keys={'prior_boxed_profile','prior_boxed_report','prior_boxed_observation'}
    if prior_keys <= set(evidence):
        try:
            profile=pins.json(evidence['prior_boxed_profile']); B.validate_profile(profile)
            need(profile['contract'].get('schema')==B.PROFILE_SCHEMA_30 and
                 profile['contract']['policy30_model']==base['policy30_model'],
                 'Boxed predecessor must use the actual independently retrained30Hz model')
            report=pins.json(evidence['prior_boxed_report'])
            observed_sha=H.verified_predecessor_observation_sha(report,profile,
                report_ref=evidence['prior_boxed_report'],profile_ref=evidence['prior_boxed_profile'],
                source_root=evidence.get('source_root'),manifest=manifest or {},pins=pins)
            summary=B.validate_type1_report(report,contract=profile['contract'],
                contract_digest=profile['contract_sha256'],mode='learned_boxed',duration_s=2)
            observed=pins.json(evidence['prior_boxed_observation'])
            need(observed.get('source')=='direct_current_user_reply' and observed.get('report_sha256')==observed_sha and
                 observed.get('audio_heard') is True and
                 observed.get('abnormal_noise_vibration_slip_sinking_contact') is False and
                 observed.get('synthetic_interaction') is False,
                 'Actual prior boxed observation must remain on its verified original bytes')
            H._text(observed.get('user_statement'),'prior boxed observation')
            H._text(observed.get('user_reply_id'),'prior boxed reply')
            result['prior_boxed']={'status':'SUPPLEMENTAL_ORIGINAL_BOXED2_CHECKED',
                'observation_original_report_sha256':observed_sha,'summary':summary,
                'new_human_permission_inferred':False}
        except Exception as error:
            result['errors'].append('prior_boxed: '+type(error).__name__+': '+str(error))
    else:
        result['missing'].extend(sorted(prior_keys-set(evidence)))
    pins.verify()
    return result


def build(base_profile_ref, evidence_ref, *, conditions_ref=None, mode='human_zero_gain_timing',
          review_primary=False):
    """Return a serialized PLAN and separate review; never an executable opaque proof."""
    pins=H.Pins()
    profile=pins.json(base_profile_ref); B.validate_profile(profile)
    document=pins.json(evidence_ref)
    need(type(document) is dict and set(document)=={'schema','evidence'} and document['schema']==REFS_SCHEMA and
         type(document['evidence']) is dict and set(document['evidence']) <= H.EVIDENCE_KEYS,
         'Evidence input must be a reference-only document with the exact schema')
    evidence=copy.deepcopy(document['evidence'])
    conditions=None
    if conditions_ref is not None:
        conditions=pins.json(conditions_ref)
        need(evidence.get('current_conditions')==conditions_ref,
             'Direct current conditions must be separately SHA-pinned in the evidence document')
    else:
        need('current_conditions' not in evidence,
             'Existing current_conditions requires an explicit --conditions reference; never infer confirmation')
    plan=H.build_review_plan(profile['contract'],conditions,evidence,mode=mode)
    assets=inspect_assets(plan,pins)
    review={'schema':REVIEW_SCHEMA,'status':'PLAN_ONLY_NOT_EXECUTABLE','mode':mode,
        'plan_contract_sha256':plan['contract_sha256'],'assets':assets,
        'blockers':list(plan['blockers']),'primary_review_requested':review_primary,
        'actual_current_human_conditions_supplied':conditions_ref is not None,
        'input_sha256':dict(pins.values),**FALSE_FLAGS}
    if conditions_ref is None:
        review['blockers'].append('CURRENT_TWO_PERSON_FULL_SUPPORT_VIDEO_AUDIO_TTY_FACTS_NOT_SUPPLIED')
    if review_primary:
        try:
            need(conditions_ref is not None,'Current human conditions are missing; primary review cannot proceed')
            prepared=H.prepare_from_files(plan)
            # Keep output PLAN_ONLY for the foreground to re-authenticate every
            # primary. The in-memory opaque proof is neither returned nor saved.
            review['primary_review']={'status':prepared.plain_plan()['status'],
                'blockers':prepared.plain_plan()['blockers']}
            review['blockers']=list(prepared.plain_plan()['blockers'])
            review['status']='PRIMARY_FILES_RECHECKED_NO_EXECUTION'
        except Exception as error:
            review['primary_review']={'status':'BLOCKED','error':type(error).__name__+': '+str(error)}
            review['blockers'].append('PRIMARY_FILES_REVIEW_BLOCKED')
    pins.verify()
    review['input_sha256']=dict(pins.values)
    return plan,review


def publish(directory, plan, review):
    path=Path(directory)
    need(path.is_absolute() and not path.exists() and
         not any((item/'.git').exists() for item in (path,*path.parents)),
         'Fresh private directory outside a Git checkout required')
    path.mkdir(mode=0o700)
    refs={}
    for name,value in (('plan.json',plan),('file-only-review.json',review)):
        raw=(json.dumps(value,indent=2,allow_nan=False)+'\n').encode()
        target=path/name
        with os.fdopen(os.open(target,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'wb') as handle:
            handle.write(raw);handle.flush();os.fsync(handle.fileno())
        refs[name]={'path':str(target),'sha256':hashlib.sha256(raw).hexdigest()}
    return refs


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('base-profile','evidence'):
        parser.add_argument('--'+name,type=Path,required=True)
        parser.add_argument('--'+name+'-sha256',required=True)
    parser.add_argument('--conditions',type=Path)
    parser.add_argument('--conditions-sha256')
    parser.add_argument('--mode',choices=H.MODES,default='human_zero_gain_timing')
    parser.add_argument('--review-primary',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    need((args.conditions is None)==(args.conditions_sha256 is None),'Conditions need both path and SHA')
    plan,review=build(reference(args.base_profile,args.base_profile_sha256),reference(args.evidence,args.evidence_sha256),
        conditions_ref=None if args.conditions is None else reference(args.conditions,args.conditions_sha256),
        mode=args.mode,review_primary=args.review_primary)
    output=publish(args.output,plan,review)
    print(json.dumps({'status':review['status'],'output':output,'blockers':review['blockers'],**FALSE_FLAGS},indent=2))
    return 2 if review_primary and review.get('primary_review',{}).get('status')=='BLOCKED' else 0


if __name__=='__main__':
    raise SystemExit(main())
