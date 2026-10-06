"""Pinned separate FK STOP diagnostic child; default PLAN changes no scheduler."""
import argparse
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import sys

BASELINE_SHA='0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'
KIT_SHA='f3d48a7cabae2e502863f5e87aed2ba1b076faa292d661f59572a79b81f896fe'
REPLAY_SHA='28a97b55284313ad61eeb9506f3cb45beae4a337d2323a005c785e7abb57badf'
REPLAY_MODULE='native_policy_overnight.model_call_fastpath.saved_input_profile'
NAMES={'fk_cache_diagnostic_benchmark.py','diagnostic_support.py','diagnostic_generate.py','diagnostic_child.py',
       'native_policy_overnight/target_tail_fk_cache/diagnostic_loader.py'} | {
       'native_policy_overnight/'+folder+'/'+name for folder in ('target_tail_fk_cache','target_tail_fusion')
       for name in ('__init__.py','target.cpp','generator.py','saved_profile.py')}


def read(path, limit=8_000_000):
    path=Path(path)
    if not path.is_absolute() or '..' in path.parts or any(p.is_symlink() for p in (path,*path.parents)):
        raise ValueError('Absolute non-symlink path required')
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    with os.fdopen(fd,'rb') as stream:
        before=os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size>limit:
            raise ValueError('Bounded regular source required')
        raw=stream.read(before.st_size+1);after=os.fstat(stream.fileno())
    if (len(raw)!=before.st_size or (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=
            (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns)):
        raise ValueError('Source changed during bounded read')
    return raw


def parse(raw):
    def unique(rows):
        result={}
        for key,value in rows:
            if key in result:raise ValueError('Duplicate JSON key')
            result[key]=value
        return result
    return json.loads(raw,object_pairs_hook=unique,
                      parse_constant=lambda value:(_ for _ in ()).throw(ValueError(value)))


def verify(bundle, expected):
    if type(expected) is not str or not re.fullmatch('[0-9a-f]{64}',expected):
        raise ValueError('Explicit bundle SHA256 required')
    bundle=Path(bundle)
    raw=read(bundle/'manifest.json')
    if hashlib.sha256(raw).hexdigest()!=expected:raise ValueError('Bundle manifest changed')
    manifest=parse(raw)
    if (manifest.get('schema')!='singularitydog.fk-stop-diagnostic-source-bundle.v1'
            or set(manifest.get('files',{}))!=NAMES or manifest.get('baseline_source_sha256')!=BASELINE_SHA
            or manifest.get('baseline_kit_manifest_sha256')!=KIT_SHA
            or manifest.get('active_controller_qualification') is not False
            or manifest.get('timing_admission_eligible') is not False
            or manifest.get('target_bundle_path')!=str(bundle)
            or Path(__file__).absolute()!=bundle/'diagnostic_child.py'):
        raise ValueError('Exact separate source bundle scope required')
    for name,pin in manifest['files'].items():
        if hashlib.sha256(read(bundle/name)).hexdigest()!=pin:raise ValueError('Bundle member changed: '+name)
    kit=Path(manifest['baseline_kit_path']);raw=read(kit/'kit-manifest.json')
    if hashlib.sha256(raw).hexdigest()!=KIT_SHA:raise ValueError('Original K37 manifest changed')
    inventory=parse(raw)['files']
    if len(inventory)!=650:raise ValueError('Original650 member kit required')
    for name,pin in inventory.items():
        if (type(name) is not str or Path(name).is_absolute() or '..' in Path(name).parts or
                hashlib.sha256(read(kit/name)).hexdigest()!=pin):
            raise ValueError('Original kit member changed: '+str(name))
    return manifest,kit


def execute_module(path, name, expected):
    raw=read(path)
    if hashlib.sha256(raw).hexdigest()!=expected:raise ValueError('Execution source changed')
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    exec(compile(raw,str(path),'exec'),module.__dict__)
    return module


def require_fresh_interpreter():
    if any(name=='singularitydog_hw' or name.startswith('singularitydog_hw.') or
           name=='native_policy_overnight' or name.startswith('native_policy_overnight.') or
           name=='diagnostic_support' or name=='torch' for name in sys.modules):
        raise ValueError('Fresh interpreter without preloaded runtime/model required')


def pinned_read(ref):
    if (type(ref) is not dict or set(ref)!={'path','sha256'} or type(ref['path']) is not str
            or type(ref['sha256']) is not str or not re.fullmatch('[0-9a-f]{64}',ref['sha256'])):
        raise ValueError('Exact dependency path/SHA256 required')
    raw=read(ref['path'])
    if hashlib.sha256(raw).hexdigest()!=ref['sha256']:
        raise ValueError('Pinned dependency source changed')
    return raw


def bind_replay(arguments):
    """Bind the fixed saved helper absent from sparse K37; never search paths."""
    parser=argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    parser.add_argument('--fk-cache-manifest',required=True)
    parser.add_argument('--fk-cache-manifest-sha256',required=True)
    for option in ('--fk-cache-manifest','--fk-cache-manifest-sha256'):
        if sum(token==option or token.startswith(option+'=') for token in arguments)!=1:
            raise ValueError('One explicit candidate dependency selection required')
    selected,_=parser.parse_known_args(arguments)
    data=parse(pinned_read({'path':selected.fk_cache_manifest,'sha256':selected.fk_cache_manifest_sha256}))
    if (data.get('schema')!='singularitydog.fk-cache-stop-diagnostic-artifact.v1'
            or data.get('status')!='PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL'
            or any(data.get(flag) is not False for flag in ('output_allowed','approved_for_runtime',
                'active_controller_qualification','timing_admission_eligible','live_50hz_verified'))):
        raise ValueError('Unapproved explicit diagnostic dependencies required')
    refs=data['references'];ref=refs['replay_helper']
    report=parse(pinned_read(refs['file_only_report']))
    if ref['sha256']!=REPLAY_SHA or report.get('replay_helper_sha256')!=REPLAY_SHA:
        raise ValueError('Exact frozen saved helper required')
    pinned_read(ref)
    package=importlib.import_module('native_policy_overnight.model_call_fastpath')
    if REPLAY_MODULE in sys.modules or hasattr(package,'saved_input_profile'):
        raise ValueError('Saved helper must be freshly bound')
    module=execute_module(Path(ref['path']),REPLAY_MODULE,REPLAY_SHA)
    sys.modules[REPLAY_MODULE]=module;package.saved_input_profile=module
    pinned_read(ref)
    return ref


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--bundle',type=Path,required=True);p.add_argument('--bundle-sha256',required=True)
    p.add_argument('--interval-us',type=int,choices=(100,),required=True)
    p.add_argument('arguments',nargs=argparse.REMAINDER);args=p.parse_args(argv)
    arguments=args.arguments[1:] if args.arguments[:1]==['--'] else args.arguments
    manifest,kit=verify(args.bundle,args.bundle_sha256)
    require_fresh_interpreter()
    before=sys.getswitchinterval();paths=list(sys.path);modules=set(sys.modules);changed=False;replay_ref=None
    try:
        sys.path[:0]=[str(args.bundle),str(kit/'runtime'),str(kit/'runtime/experiments')]
        package=importlib.import_module('native_policy_overnight')
        package.__path__.append(str(args.bundle/'native_policy_overnight'))
        replay_ref=bind_replay(arguments)
        generator=execute_module(args.bundle/'diagnostic_generate.py',
            'native_policy_overnight.target_tail_fk_cache._diagnostic_generate',
            manifest['files']['diagnostic_generate.py'])
        original=read(kit/'runtime/singularitydog_hw/native_pipeline_benchmark.py')
        derived,proof=generator.derive(original)
        if derived!=read(args.bundle/'fk_cache_diagnostic_benchmark.py') or proof!=manifest['derivation']:
            raise ValueError('Inverse source derivation differs')
        support=execute_module(args.bundle/'diagnostic_support.py','diagnostic_support',
                               manifest['files']['diagnostic_support.py'])
        sys.modules['diagnostic_support']=support
        module=execute_module(args.bundle/'fk_cache_diagnostic_benchmark.py',
            'singularitydog_hw._explicit_fk_stop_diagnostic',manifest['files']['fk_cache_diagnostic_benchmark.py'])
        # Parser abbreviations are disabled inside the separate module, so only
        # literal --execute selects collection and this scheduling scope.
        if '--execute' in arguments:
            changed=True;sys.setswitchinterval(args.interval_us/1_000_000)
        print(json.dumps({'kind':'fk_diagnostic_python_switch_scope','before_s':before,
                          'during_s':sys.getswitchinterval(),'selected_execute':'--execute' in arguments,
                          'active_controller_qualification':False}))
        return module.main(arguments)
    finally:
        sys.path[:]=paths
        if changed:sys.setswitchinterval(before)
        for name in set(sys.modules)-modules:
            if name=='diagnostic_support' or name=='native_policy_overnight' or name.startswith('native_policy_overnight.'):
                sys.modules.pop(name,None)
        unchanged=True
        try:
            verify(args.bundle,args.bundle_sha256)
            if replay_ref is not None:pinned_read(replay_ref)
        except Exception:unchanged=False
        print(json.dumps({'kind':'fk_diagnostic_python_switch_restore',
                          'restored':sys.getswitchinterval()==before,'source_files_unchanged':unchanged}))
        if not unchanged:raise RuntimeError('Source changed during diagnostic child')


if __name__=='__main__':raise SystemExit(main())
