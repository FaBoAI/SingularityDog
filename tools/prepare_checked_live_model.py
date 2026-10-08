#!/usr/bin/env python3
"""File-only checked-model manifest and blocked draft; default PLAN, no Torch."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).absolute().parents[1]
if not sys.dont_write_bytecode:
    raise RuntimeError('Use python -B for this file-only producer')
sys.path.insert(0,str(ROOT/'runtime'))
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_checked_dispatch as checked

DEPENDENCIES_SCHEMA='singularitydog.checked-live-model-dependencies.v1'
BLOCKERS=(
    'Current selected-source normal-observer 5/501 raw timing and engineering review are pending.',
    'Current boot, power, UID, pose, voltage and watchdog graph must be rebound after any physical change.',
    'Complete current supported-output and ordinary2/10 predecessor admission are pending.',
    'CPU component parity grants no Type1, deadline, physical or motor-output qualification.',
)


def encoded(value):
    return (json.dumps(value,sort_keys=True,indent=2,ensure_ascii=False,allow_nan=False)+'\n').encode()


def read(path,digest,pins):
    return checked._read({'path':str(Path(path).expanduser().absolute()),'sha256':digest},pins)


def inspect(args):
    pins={};base=read(args.base_profile,args.base_profile_sha256,pins)
    live._structure(base)
    live._need(not checked.selected(base),'Fresh selection must start from an ordinary original-model profile')
    dependencies=read(args.dependencies,args.dependencies_sha256,pins)
    live._need(type(dependencies) is dict and set(dependencies)=={'schema','references'} and
        dependencies['schema']==DEPENDENCIES_SCHEMA and type(dependencies['references']) is dict and
        set(dependencies['references'])==set(checked.REFS),'Exact checked model dependencies document required')
    output=Path(args.output).expanduser().absolute()
    live._need(not output.exists() and output.parent.is_dir() and
        not any(item.is_symlink() for item in (output,*output.parents)) and
        not any((item/'.git').exists() for item in (output,*output.parents)),
        'Fresh absolute private destination outside Git required')
    draft=copy.deepcopy(base)
    draft.update(native_checked_policy_dispatch=True,approved_for_supported_policy_output=False,
                 review=None,blockers=list(BLOCKERS))
    draft['cadence_source_sha256']=live.cadence_source_hashes(draft)
    source_root=ROOT/'runtime'
    source_hashes={name:hashlib.sha256((source_root/name).read_bytes()).hexdigest()
                   for name in checked.source_paths()}
    manifest=dict(schema=checked.SCHEMA,status='FILE_ONLY_MODEL_BINDINGS_NO_OUTPUT_APPROVAL',
        variant='checked_r11',references=copy.deepcopy(dependencies['references']),
        original_fk_artifacts={key:copy.deepcopy(base['artifacts'][key]) for key in
            ('target_fk_manifest','scalar_step_manifest','model_manifest')},
        source_sha256=source_hashes,physical_future_observations=None,
        **dict.fromkeys(checked.FALSE_FLAGS,False))
    manifest_bytes=encoded(manifest)
    reference=dict(path=str(output/'checked-model-manifest.json'),
                   sha256=hashlib.sha256(manifest_bytes).hexdigest())
    draft['artifacts']['checked_model_manifest']=reference
    checked.scope(draft);live._structure(draft);live._settings(draft)
    proposal=checked._document_plan(draft,manifest,reference,pins)
    changed={key for key in set(base)|set(draft) if base.get(key)!=draft.get(key)}
    live._need(changed=={'native_checked_policy_dispatch','approved_for_supported_policy_output',
        'review','blockers','artifacts','cadence_source_sha256'} or changed<=
        {'native_checked_policy_dispatch','approved_for_supported_policy_output',
         'review','blockers','artifacts','cadence_source_sha256'},'Unexpected original profile field change')
    live._need({key:value for key,value in draft['artifacts'].items() if key!='checked_model_manifest'}==
        base['artifacts'],'Original source/current/model artifacts must remain byte-bound')
    plan=dict(schema='singularitydog.checked-live-model-draft-plan.v1',status='PLAN_ONLY',
        base_profile=dict(path=str(Path(args.base_profile).absolute()),sha256=args.base_profile_sha256),
        dependencies=dict(path=str(Path(args.dependencies).absolute()),sha256=args.dependencies_sha256),
        destination=str(output),proposed_manifest=reference,
        proposed_profile=dict(path=str(output/'profile.json'),sha256=hashlib.sha256(encoded(draft)).hexdigest()),
        proposed_file_plan=proposal,selected_manifest_published=False,file_only_loader_acceptance_pending=True,
        changed_profile_fields=sorted(changed),numerical_settings_changed=False,
        current_boot_or_power_observed=False,physical_future_observations=None,
        all_input_sha256=copy.deepcopy(pins),**dict.fromkeys(checked.FALSE_FLAGS,False))
    checked.verify_files({'input_sha256':pins})
    return plan,manifest,draft


def run(args):
    plan,manifest,draft=inspect(args)
    print(json.dumps(plan,indent=2,ensure_ascii=False,allow_nan=False),flush=True)
    if not args.prepare:
        return plan
    output=Path(plan['destination']);output.mkdir(mode=0o700)
    try:
        (output/'checked-model-manifest.json').write_bytes(encoded(manifest))
        (output/'profile.json').write_bytes(encoded(draft))
        admitted=live.load_profile(output/'profile.json',require_approved=False)
        live._need(admitted['output_allowed'] is False and admitted['review'] is None and
            admitted['approved_for_supported_policy_output'] is False and
            admitted['_checked_model_plan']['manifest']==plan['proposed_manifest'],
            'Fresh draft must remain unapproved after complete file-only loader')
        checked.verify_files({'input_sha256':plan['all_input_sha256']})
        result=dict(plan,status='UNAPPROVED_CHECKED_MODEL_DRAFT_CREATED',selected_manifest_published=True,
                    file_only_loader_acceptance_pending=False,complete_output_admission_pending=True)
        (output/'prepare-receipt.json').write_bytes(encoded(result))
        print(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False),flush=True)
        return result
    except BaseException:
        shutil.rmtree(output)
        raise


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('base-profile','base-profile-sha256','dependencies','dependencies-sha256','output'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--prepare',action='store_true')
    args=parser.parse_args(argv)
    try:run(args)
    except (OSError,ValueError,KeyError,TypeError) as error:parser.error(str(error))
    return 0


if __name__=='__main__':raise SystemExit(main())
