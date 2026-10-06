"""Source-pinned separate STOP-proxy module runner with reversible Python scope."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys


def read_regular(path, limit=1_000_000):
    path = Path(path)
    if not path.is_absolute() or '..' in path.parts or any(p.is_symlink() for p in (path,*path.parents)):
        raise ValueError('Absolute non-symlink source path required')
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    with os.fdopen(fd,'rb') as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size>limit:
            raise ValueError('Bounded regular source required')
        value=stream.read(limit+1)
        after=os.fstat(stream.fileno())
    if (len(value)>limit or len(value)!=info.st_size or
            (info.st_dev,info.st_ino,info.st_size,info.st_mtime_ns)!=
            (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns)):
        raise ValueError('Oversized or concurrently changed source')
    return value


def verify(bundle, expected):
    bundle=Path(bundle)
    raw=read_regular(bundle/'manifest.json')
    if hashlib.sha256(raw).hexdigest()!=expected:raise ValueError('Experiment manifest changed')
    manifest=json.loads(raw)
    names={'candidate.py','readiness_cause_collector_support_r38.py',
           'native_readiness_cause_benchmark_r38.py','readiness_cause_child_r38.py'}
    if (manifest['schema']!='private.readiness-cause-bundle.v1' or set(manifest['files'])!=names or
            manifest['active_output_eligible'] is not False or manifest['timing_admission_eligible'] is not False or
            manifest['baseline_source_sha256']!='0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'):
        raise ValueError('Exact experiment member/scope manifest required')
    for name,digest in manifest['files'].items():
        if hashlib.sha256(read_regular(bundle/name)).hexdigest()!=digest:
            raise ValueError('Experiment member changed: '+name)
    if Path(__file__).resolve()!=bundle/'readiness_cause_child_r38.py':
        raise ValueError('Runner must be the pinned bundle member')
    kit=Path(manifest['baseline_kit_path'])
    raw=read_regular(kit/'kit-manifest.json')
    if hashlib.sha256(raw).hexdigest()!=manifest['baseline_kit_manifest_sha256']:
        raise ValueError('Original kit changed')
    original=json.loads(raw)
    if len(original['files'])!=650:raise ValueError('Exact original kit inventory required')
    for name,digest in original['files'].items():
        if Path(name).is_absolute() or '..' in Path(name).parts:raise ValueError('Invalid kit member path')
        if hashlib.sha256(read_regular(kit/name,8_000_000)).hexdigest()!=digest:
            raise ValueError('Original kit member changed: '+name)
    return manifest,kit


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--bundle-sha256',required=True)
    p.add_argument('--interval-us',type=int,choices=(100,),required=True)
    p.add_argument('arguments',nargs=argparse.REMAINDER)
    args=p.parse_args(argv)
    arguments=args.arguments[1:] if args.arguments[:1]==['--'] else args.arguments
    manifest,kit=verify(args.bundle,args.bundle_sha256)
    if ('readiness_cause_collector_support_r38' in sys.modules or
            any(name=='singularitydog_hw' or name.startswith('singularitydog_hw.') for name in sys.modules)):
        raise ValueError('Fresh interpreter without preloaded runtime/support required')
    before=sys.getswitchinterval()
    paths=list(sys.path)
    scope_changed=False
    try:
        sys.path[:0]=[str(args.bundle),str(kit/'runtime'),str(kit/'runtime/experiments')]
        support_path=args.bundle/'readiness_cause_collector_support_r38.py'
        support_raw=read_regular(support_path)
        if hashlib.sha256(support_raw).hexdigest()!=manifest['files'][support_path.name]:
            raise ValueError('Experiment support changed before execution')
        support_spec=importlib.util.spec_from_file_location('readiness_cause_collector_support_r38',support_path)
        support=importlib.util.module_from_spec(support_spec)
        exec(compile(support_raw,str(support_path),'exec'),support.__dict__)
        sys.modules['readiness_cause_collector_support_r38']=support
        spec=importlib.util.spec_from_file_location('singularitydog_hw._explicit_readiness_cause_r38',
            args.bundle/'native_readiness_cause_benchmark_r38.py')
        module=importlib.util.module_from_spec(spec)
        raw=read_regular(args.bundle/'native_readiness_cause_benchmark_r38.py')
        if hashlib.sha256(raw).hexdigest()!=manifest['files']['native_readiness_cause_benchmark_r38.py']:
            raise ValueError('Copied benchmark changed before execution')
        exec(compile(raw,str(args.bundle/'native_readiness_cause_benchmark_r38.py'),'exec'),module.__dict__)
        # PLAN performs no scheduling mutation. The copied module independently
        # rejects wrong duration, phase, command kind, gap, native mode and caps.
        if '--execute' in arguments:
            scope_changed=True
            sys.setswitchinterval(args.interval_us/1_000_000)
        print(json.dumps(dict(kind='experimental_python_switch_scope',before_s=before,
            during_s=sys.getswitchinterval(),selected_execute='--execute' in arguments,
            executing_module_sha256=manifest['files']['native_readiness_cause_benchmark_r38.py'])))
        return module.main(arguments)
    finally:
        sys.modules.pop('readiness_cause_collector_support_r38',None)
        sys.path[:]=paths
        if scope_changed:sys.setswitchinterval(before)
        unchanged=True
        try:verify(args.bundle,args.bundle_sha256)
        except Exception:unchanged=False
        print(json.dumps(dict(kind='experimental_python_switch_restore',
            restored=sys.getswitchinterval()==before,source_files_unchanged=unchanged)))
        if not unchanged:raise RuntimeError('Experiment source changed during child run')


if __name__=='__main__':raise SystemExit(main())
