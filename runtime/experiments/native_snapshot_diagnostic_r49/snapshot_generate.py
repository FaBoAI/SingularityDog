"""Generate import-only observer and main-only FK diagnostic copies."""
import argparse
import ast
import json
from pathlib import Path

from . import snapshot_loader as loader

KIT_SHA = 'f3d48a7cabae2e502863f5e87aed2ba1b076faa292d661f59572a79b81f896fe'
FK_MANIFEST_SHA = '1e08148e7426172f6123c699e8ce52f10663adb2ab6adaf08a6623048aca9a17'
FK_BENCHMARK_SHA = 'f8060c85059cc3e23ca079bcb784f5fd9f57bc0fd465e77eed039c59caa5bc14'
FK_CHILD_SHA = '94d1b9978c59c6516f3b7d12317ea03aac6194d086b9987f1dde764d9b1aec01'
R47_FILES = {
 'fk_cache_diagnostic_benchmark.py': FK_BENCHMARK_SHA,
 'diagnostic_child.py': FK_CHILD_SHA,
 'diagnostic_support.py': 'b00db2aa80c1065d874e80016d95932db087f2faf7b20f9f067ef2f591999077',
 'diagnostic_generate.py': 'e8ad9542721321f1bb7313ad035d5a1d25d8057f2d55e931b917ab0ad1537872',
 'native_policy_overnight/target_tail_fk_cache/diagnostic_loader.py': '34226609eb7a7b8b52a2eecf19f2156b5c611f866d5c27c3f8404f6a6b11232a',
 'native_policy_overnight/target_tail_fusion/__init__.py': 'e1acf2a1c36b1af4bb6e54b14c8fd453beeb0480614a5aa3a72d3025a9055d9f',
 'native_policy_overnight/target_tail_fusion/target.cpp': '2629bb99b6db4789c8a28101d31f65bab894337242060c71b5b8a9c229702b40',
 'native_policy_overnight/target_tail_fusion/generator.py': 'd4d554ed0953219b45b78212838daf0967fc49819f57b61bbd6cd94f60bcca51',
 'native_policy_overnight/target_tail_fusion/saved_profile.py': 'cf36d22ef538297ac781c5ec717e7a0ae4ad414f12e38f32619ef36ea1114a38',
 'native_policy_overnight/target_tail_fk_cache/__init__.py': '4c2e011ef5f7e94f0870986717431b4ab7b8245b50d147c9c0172d2d48b43e7f',
 'native_policy_overnight/target_tail_fk_cache/target.cpp': '61a7b873f049ab206a839ea94e62fabed979c4b9b2823511e466508b37b4c426',
 'native_policy_overnight/target_tail_fk_cache/generator.py': '8ef5cd705382a4a4b3d8a251e797e94cdab6139454f16d09119c4576ecc26852',
 'native_policy_overnight/target_tail_fk_cache/saved_profile.py': '15b84f72d928d145d717a0b1860f3316bd1aaa772e7903de8144b6d5c3fdbb40'}
EXTRA_NAMES = {'baseline_fk_benchmark.py', 'baseline_diagnostic_child.py', 'copy_observer.py',
               '_event_snapshot_native.c', 'build_native_event_copy.py'} | {
    'native_snapshot_diagnostic_r49/' + name for name in
    ('__init__.py', 'snapshot_generate.py', 'snapshot_loader.py', 'snapshot_support.py', 'snapshot_build.py')}
NAMES = set(R47_FILES) | EXTRA_NAMES


def once(text, old, new):
    loader.require(text.count(old) == 1, 'Unique source anchor required: ' + old[:70])
    return text.replace(old, new, 1)


def _derive(raw, pin, edits, changed_functions):
    loader.require(loader.sha(raw) == pin, 'Unknown baseline source rejected')
    source = raw.decode('utf-8')
    for old, new in edits:
        source = once(source, old, new)
    before, after = ast.parse(raw), ast.parse(source)
    functions = lambda tree: {n.name: ast.dump(n, include_attributes=False) for n in tree.body
                             if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    a, b = functions(before), functions(after)
    loader.require(a.keys() == b.keys() and [name for name in a if a[name] != b[name]] == changed_functions,
                   'Allowed function AST delta differs')
    restored = source
    for old, new in reversed(edits):
        restored = once(restored, new, old)
    loader.require(restored.encode() == raw and ast.dump(ast.parse(restored)) == ast.dump(before),
                   'Inverse full-byte/AST identity failed')
    return source.encode(), {'inverse_bytes_exact': True, 'inverse_ast_exact': True,
                            'changed_functions': changed_functions}


def derive_observer(raw):
    return _derive(raw, loader.OBSERVER_SHA, [
        ('from .event_snapshot import snapshot_event',
         'from _sd_snapshot_diag_r49._event_snapshot_native import snapshot_event')], [])


def derive_benchmark(raw):
    result, proof = _derive(raw, FK_BENCHMARK_SHA, [
        ('import diagnostic_support as _fk_diagnostic',
         'import diagnostic_support as _fk_diagnostic\n'
         'from native_snapshot_diagnostic_r49 import snapshot_support as _snapshot_diagnostic'),
        ("    p.add_argument('--fk-cache-manifest',required=True)\n",
         "    p.add_argument('--snapshot-manifest',required=True)\n"
         "    p.add_argument('--snapshot-manifest-sha256',required=True)\n"
         "    p.add_argument('--fk-cache-manifest',required=True)\n"),
        ('    fk_proof=_fk_diagnostic.validate_cli(args,p,__file__)\n',
         '    fk_proof=_fk_diagnostic.validate_cli(args,p,__file__)\n'
         '    snapshot_proof=_snapshot_diagnostic.validate_cli(args,p,__file__,observer)\n'),
        ("    plan['experimental_fk_cache_diagnostic']=fk_proof\n",
         "    plan['experimental_fk_cache_diagnostic']=fk_proof\n"
         "    plan['experimental_snapshot_copy_diagnostic']=snapshot_proof\n"),
        ('    try:\n        if guard_reference is not None:\n',
         '    snapshot_selection=None\n    try:\n'
         '        snapshot_selection=_snapshot_diagnostic.enter(args,globals(),snapshot_proof)\n'
         '        if guard_reference is not None:\n'),
        ('    finally:\n        for sig,handler in handlers.items():signal.signal(sig,handler)\n',
         '    finally:\n'
         '        _snapshot_diagnostic.restore(report,snapshot_proof,snapshot_selection)\n'
         '        for sig,handler in handlers.items():signal.signal(sig,handler)\n'),
        ('        _fk_diagnostic.finalize_report(report,fk_proof)\n',
         '        _fk_diagnostic.finalize_report(report,fk_proof)\n'
         '        _snapshot_diagnostic.finalize_report(report,snapshot_proof)\n')], ['main'])
    proof['collect_ast_unchanged'] = True
    proof['fk_model_selection_unchanged'] = True
    return result, proof


def derive_child(raw):
    names = repr(sorted(EXTRA_NAMES))
    return _derive(raw, FK_CHILD_SHA, [
        ("       for name in ('__init__.py','target.cpp','generator.py','saved_profile.py')}\n",
         "       for name in ('__init__.py','target.cpp','generator.py','saved_profile.py')}\n"
         'NAMES |= set(' + names + ')\n'),
        ("manifest.get('schema')!='singularitydog.fk-stop-diagnostic-source-bundle.v1'",
         "manifest.get('schema')!='singularitydog.snapshot-copy-stop-diagnostic-source-bundle.r49.v1'"),
        ("or manifest.get('active_controller_qualification') is not False\n",
         "or manifest.get('baseline_observer_sha256')!='" + loader.OBSERVER_SHA + "'\n"
         "            or manifest.get('active_controller_qualification') is not False\n"),
        ("           name=='diagnostic_support' or name=='torch' for name in sys.modules):",
         "           name=='diagnostic_support' or name=='torch' or\n"
         "           name=='native_snapshot_diagnostic_r49' or name.startswith('native_snapshot_diagnostic_r49.') or\n"
         "           name=='_sd_snapshot_diag_r49' or name.startswith('_sd_snapshot_diag_r49.') for name in sys.modules):"),
        ("        if derived!=read(args.bundle/'fk_cache_diagnostic_benchmark.py') or proof!=manifest['derivation']:\n"
         "            raise ValueError('Inverse source derivation differs')\n",
         "        if derived!=read(args.bundle/'baseline_fk_benchmark.py') or proof!=manifest['fk_derivation']:\n"
         "            raise ValueError('Inverse FK source derivation differs')\n"
         "        from native_snapshot_diagnostic_r49 import snapshot_generate as snapshot_generator\n"
         "        derived,proof=snapshot_generator.derive_benchmark(derived)\n"
         "        if derived!=read(args.bundle/'fk_cache_diagnostic_benchmark.py') or proof!=manifest['benchmark_derivation']:\n"
         "            raise ValueError('Inverse snapshot benchmark derivation differs')\n"
         "        copied,proof=snapshot_generator.derive_observer(read(kit/'runtime/singularitydog_hw/policy_observer.py'))\n"
         "        if copied!=read(args.bundle/'copy_observer.py') or proof!=manifest['observer_derivation']:\n"
         "            raise ValueError('Inverse observer derivation differs')\n"),
        ("        sys.path[:0]=[str(args.bundle),str(kit/'runtime'),str(kit/'runtime/experiments')]\n",
         "        sys.path[:0]=[str(args.bundle),str(kit/'runtime'),str(kit/'runtime/experiments')]\n"
         "        # Execute authenticated bytes directly; never consume a cached pyc.\n"
         "        snapshot_package=execute_module(args.bundle/'native_snapshot_diagnostic_r49/__init__.py',\n"
         "            'native_snapshot_diagnostic_r49',manifest['files']['native_snapshot_diagnostic_r49/__init__.py'])\n"
         "        sys.modules['native_snapshot_diagnostic_r49']=snapshot_package\n"
         "        for selected_name in ('snapshot_loader','snapshot_generate','snapshot_support','snapshot_build'):\n"
         "            source_name='native_snapshot_diagnostic_r49/'+selected_name+'.py'\n"
         "            selected=execute_module(args.bundle/source_name,'native_snapshot_diagnostic_r49.'+selected_name,\n"
         "                manifest['files'][source_name])\n"
         "            sys.modules['native_snapshot_diagnostic_r49.'+selected_name]=selected\n"
         "            setattr(snapshot_package,selected_name,selected)\n"),
        ("            if name=='diagnostic_support' or name=='native_policy_overnight' or name.startswith('native_policy_overnight.'):",
         "            if (name=='diagnostic_support' or name=='native_policy_overnight' or name.startswith('native_policy_overnight.') or\n"
         "                    name=='native_snapshot_diagnostic_r49' or name.startswith('native_snapshot_diagnostic_r49.')):")],
         ['verify', 'require_fresh_interpreter', 'main'])


def generate_bundle(r47, original_observer, native_folder, destination, *, target_bundle, baseline_kit):
    """Copy authenticated dependencies to a fresh directory; compile/load nothing."""
    r47 = Path(r47); native_folder = Path(native_folder)
    old = loader.parse(loader.read({'path': str(r47 / 'manifest.json'), 'sha256': FK_MANIFEST_SHA}))
    loader.require(old.get('files') == R47_FILES, 'Exact frozen R47 inventory required')
    members = {name: loader.read({'path': str(r47 / name), 'sha256': pin}) for name, pin in R47_FILES.items()}
    members['baseline_fk_benchmark.py'] = members['fk_cache_diagnostic_benchmark.py']
    members['baseline_diagnostic_child.py'] = members['diagnostic_child.py']
    members['fk_cache_diagnostic_benchmark.py'], benchmark_proof = derive_benchmark(members['baseline_fk_benchmark.py'])
    members['diagnostic_child.py'], child_proof = derive_child(members['baseline_diagnostic_child.py'])
    original = loader.read({'path': str(original_observer), 'sha256': loader.OBSERVER_SHA})
    members['copy_observer.py'], observer_proof = derive_observer(original)
    for name, pin in (('_event_snapshot_native.c', loader.NATIVE_SHA), ('build_native_event_copy.py', loader.BUILDER_SHA)):
        members[name] = loader.read({'path': str(native_folder / name), 'sha256': pin})
    here = Path(__file__).absolute().parent
    for name in EXTRA_NAMES:
        if name.startswith('native_snapshot_diagnostic_r49/'):
            path = here / Path(name).name
            members[name] = loader.read({'path': str(path), 'sha256': loader.sha(path.read_bytes())})
    out = Path(destination)
    for path in (out, Path(target_bundle), Path(baseline_kit)):
        loader.require(path.is_absolute() and '..' not in path.parts, 'Absolute declarations required')
    loader.require(not out.exists() and out.parent.is_dir() and
                   not any(p.is_symlink() or (p / '.git').exists() for p in (out, *out.parents)),
                   'Fresh private non-symlink source destination required')
    loader.require(set(members) == NAMES, 'Exact source inventory required')
    out.mkdir(mode=0o700)
    for name, value in members.items():
        path = out / name; path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as stream:
            stream.write(value)
    manifest = {'schema': loader.BUNDLE_SCHEMA, 'files': {name: loader.sha(raw) for name, raw in members.items()},
        'baseline_source_sha256': old['baseline_source_sha256'], 'baseline_kit_manifest_sha256': KIT_SHA,
        'baseline_kit_path': str(baseline_kit), 'target_bundle_path': str(target_bundle),
        'baseline_observer_sha256': loader.OBSERVER_SHA, 'baseline_fk_benchmark_sha256': FK_BENCHMARK_SHA,
        'fk_derivation': old['derivation'], 'benchmark_derivation': benchmark_proof,
        'observer_derivation': observer_proof, 'child_derivation': child_proof,
        **dict.fromkeys(loader.FLAGS, False)}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    return manifest


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('r47', 'original-observer', 'native-folder', 'output', 'target-bundle', 'baseline-kit'):
        p.add_argument('--' + name, type=Path, required=True)
    args = p.parse_args(argv)
    manifest = generate_bundle(args.r47, args.original_observer, args.native_folder, args.output,
                               target_bundle=args.target_bundle, baseline_kit=args.baseline_kit)
    print(json.dumps({'status': 'SOURCE_ONLY_DIAGNOSTIC_BUNDLE', 'members': len(manifest['files']),
                      'native_library_loaded': False, 'hardware_opened': False, 'output_allowed': False}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
