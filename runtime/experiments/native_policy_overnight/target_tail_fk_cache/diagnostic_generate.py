"""Generate an inverse-verified separate main; never edit the ordinary benchmark."""
import argparse
import ast
import hashlib
import json
from pathlib import Path

from . import diagnostic_loader as loader

BASELINE_SHA = '0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'
KIT_SHA = 'f3d48a7cabae2e502863f5e87aed2ba1b076faa292d661f59572a79b81f896fe'


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Unique original source anchor required: ' + old[:70])
    return text.replace(old, new, 1)


def derive(raw):
    if hashlib.sha256(raw).hexdigest() != BASELINE_SHA:
        raise ValueError('Exact frozen K37 benchmark required')
    edits = []
    def apply(text, old, new):
        edits.append((old, new))
        return once(text, old, new)
    source = raw.decode()
    source = apply(source, 'PERIOD_NS = 20_000_000',
        'import diagnostic_support as _fk_diagnostic\n\nPERIOD_NS = 20_000_000')
    source = apply(source, '    p=argparse.ArgumentParser(description=__doc__)\n',
        "    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)\n"
        "    p.add_argument('--fk-cache-manifest',required=True)\n"
        "    p.add_argument('--fk-cache-manifest-sha256',required=True)\n")
    source = apply(source, '    args=p.parse_args(argv)\n',
        '    args=p.parse_args(argv)\n    fk_proof=_fk_diagnostic.validate_cli(args,p,__file__)\n')
    source = apply(source, "('pinned_scalar_cpp' if scalar_step_selected else",
        "('pinned_fk_cache_cpp' if scalar_step_selected else")
    source = apply(source, "    if not args.execute:\n        print(json.dumps(plan,indent=2));return 0",
        "    plan['experimental_fk_cache_diagnostic']=fk_proof\n"
        "    plan['active_controller_qualification']=False;plan['timing_admission_eligible']=False\n"
        "    if source_provenance is not None:source_provenance['source_map_role']='original dependency graph only'\n"
        "    if not args.execute:\n        print(json.dumps(plan,indent=2));return 0")
    source = apply(source, "Path(__file__).resolve().parents[1]/'experiments'",
                   "_fk_diagnostic.baseline_runtime()/'experiments'")
    source = apply(source,
        "                    from native_policy_overnight.model_call_fastpath.scalar_loader import load_file_only_verified\n"
        "                    policy,source=load_file_only_verified(args.scalar_step_manifest,\n"
        "                        expected_sha256=args.scalar_step_manifest_sha256,\n"
        "                        baseline_manifest=args.native_policy_manifest,\n"
        "                        baseline_sha=args.native_policy_manifest_sha256,bundle=args.bundle)\n"
        "                    report['scalar_step_model_source']=source\n",
        "                    from native_policy_overnight.target_tail_fk_cache.diagnostic_loader import load_diagnostic_verified\n"
        "                    policy,source=load_diagnostic_verified(args.fk_cache_manifest,\n"
        "                        expected_sha256=args.fk_cache_manifest_sha256,\n"
        "                        scalar_manifest=args.scalar_step_manifest,scalar_sha=args.scalar_step_manifest_sha256,\n"
        "                        baseline_manifest=args.native_policy_manifest,\n"
        "                        baseline_sha=args.native_policy_manifest_sha256,bundle=args.bundle)\n"
        "                    report['fk_cache_model_source']=source\n")
    source = apply(source, '        _finish_source_provenance(report,source_provenance)\n',
        '        _finish_source_provenance(report,source_provenance)\n'
        '        _fk_diagnostic.finalize_report(report,fk_proof)\n')
    original, changed = ast.parse(raw), ast.parse(source)
    def functions(tree):
        return {node.name: ast.dump(node, include_attributes=False) for node in tree.body
                if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
    before, after = functions(original), functions(changed)
    if before.keys() != after.keys() or [key for key in before if before[key] != after[key]] != ['main']:
        raise ValueError('Only main may differ; all collection/observer/wait/guard functions must match')
    restored = source
    for old, new in reversed(edits):
        restored = once(restored, new, old)
    if restored.encode() != raw or ast.dump(ast.parse(restored)) != ast.dump(original):
        raise ValueError('Inverse full-byte/AST identity failed')
    return source.encode(), {'inverse_bytes_exact': True, 'inverse_ast_exact': True,
                            'changed_functions': ['main'], 'collect_ast_unchanged': True}


def generate_bundle(baseline, destination, *, target_bundle, baseline_kit):
    """Publish new source files only; candidate/evidence references stay explicit."""
    baseline = Path(baseline)
    raw = loader.read({'path':str(baseline), 'sha256':BASELINE_SHA})
    generated, proof = derive(raw)
    out = Path(destination)
    for path in (out, Path(target_bundle), Path(baseline_kit)):
        if not path.is_absolute() or '..' in path.parts:
            raise ValueError('Absolute declaration required')
    if (out.exists() or not out.parent.is_dir() or any(p.is_symlink() for p in (out,*out.parents))
            or any((p/'.git').exists() for p in (out,*out.parents))):
        raise ValueError('Fresh private non-symlink source destination required')
    here = Path(__file__).absolute().parent
    members = {'fk_cache_diagnostic_benchmark.py':generated}
    for name in ('diagnostic_support.py','diagnostic_generate.py','diagnostic_child.py'):
        path = here/name
        members[name] = loader.read({'path':str(path), 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    for folder in ('target_tail_fusion', 'target_tail_fk_cache'):
        for name in ('__init__.py','target.cpp','generator.py','saved_profile.py'):
            path = here.parent/folder/name
            members['native_policy_overnight/'+folder+'/'+name] = loader.read(
                {'path':str(path), 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    path=here/'diagnostic_loader.py'
    members['native_policy_overnight/target_tail_fk_cache/diagnostic_loader.py'] = loader.read(
        {'path':str(path), 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    out.mkdir(mode=0o700)
    for name, value in members.items():
        path=out/name;path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('xb') as stream:stream.write(value)
    manifest={'schema':'singularitydog.fk-stop-diagnostic-source-bundle.v1',
        'files':{name:hashlib.sha256(value).hexdigest() for name,value in members.items()},
        'baseline_source_sha256':BASELINE_SHA,'baseline_kit_manifest_sha256':KIT_SHA,
        'baseline_kit_path':str(baseline_kit),'target_bundle_path':str(target_bundle),
        'derivation':proof,'active_controller_qualification':False,'timing_admission_eligible':False}
    with (out/'manifest.json').open('x') as stream:json.dump(manifest,stream,indent=2);stream.write('\n')
    return manifest


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('baseline','output','target-bundle','baseline-kit'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args(argv)
    manifest=generate_bundle(args.baseline,args.output,target_bundle=args.target_bundle,baseline_kit=args.baseline_kit)
    print(json.dumps({'status':'SOURCE_ONLY_DIAGNOSTIC_BUNDLE','members':len(manifest['files']),
                      'active_controller_qualification':False}))
    return 0


if __name__=='__main__':raise SystemExit(main())
