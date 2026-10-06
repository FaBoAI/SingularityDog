"""Explicit file-only native build; default PLAN starts no compiler or extension."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import sysconfig

from . import snapshot_loader as loader


def _fresh(path):
    path = Path(path)
    loader.require(path.is_absolute() and '..' not in path.parts and not path.exists() and
        path.parent.is_dir() and not any(p.is_symlink() or (p / '.git').exists() for p in (path, *path.parents)),
        'Fresh private non-symlink output required')
    return path


def build_plan(bundle, expected_sha256, original_observer, output):
    ref = {'path': str(bundle), 'sha256': expected_sha256}
    data = loader._bundle(ref)
    original = {'path': str(original_observer), 'sha256': loader.OBSERVER_SHA}
    loader.read(original)
    _fresh(output)
    return {'schema': 'singularitydog.snapshot-copy-native-build-plan.r49.v1',
        'source_bundle': ref, 'source_sha256': loader.NATIVE_SHA, 'builder_sha256': loader.BUILDER_SHA,
        'original_observer': original, 'output': str(output), 'compiler_started': False,
        'native_library_loaded': False, 'hardware_opened': False, **dict.fromkeys(loader.FLAGS, False)}


def build(bundle, *, expected_sha256, original_observer, output):
    plan = build_plan(bundle, expected_sha256, original_observer, output)
    folder = loader.reference(plan['source_bundle']).parent
    out = _fresh(output)
    sources = {name: loader.read({'path': str(folder / name), 'sha256': pin}) for name, pin in
        (('_event_snapshot_native.c', loader.NATIVE_SHA), ('build_native_event_copy.py', loader.BUILDER_SHA))}
    loader.require(sys.implementation.name == 'cpython', 'CPython native build required')
    out.mkdir(mode=0o700)
    for name, raw in sources.items():
        with (out / name).open('xb') as stream:
            stream.write(raw)
    result = subprocess.run([sys.executable, '-B', str(out / 'build_native_event_copy.py')],
                            capture_output=True, text=True, check=False)
    (out / 'compiler.stdout').write_text(result.stdout)
    (out / 'compiler.stderr').write_text(result.stderr)
    loader.require(result.returncode == 0, 'Explicit file-only native compiler failed')
    library = out / ('_event_snapshot_native' + sysconfig.get_config_var('EXT_SUFFIX'))
    library_ref = {'path': str(library), 'sha256': loader.sha(library.read_bytes())}
    loader.read(library_ref)
    for name, pin in (('_event_snapshot_native.c', loader.NATIVE_SHA), ('build_native_event_copy.py', loader.BUILDER_SHA)):
        loader.read({'path': str(out / name), 'sha256': pin})
        loader.read({'path': str(folder / name), 'sha256': pin})
    receipt = {'schema': 'singularitydog.snapshot-copy-native-build.r49.v1', 'status': 'BUILT_FILE_ONLY',
        'source_sha256': loader.NATIVE_SHA, 'builder_sha256': loader.BUILDER_SHA,
        'library': library_ref, 'implementation': sys.implementation.name, 'python_version': sys.version,
        'extension_suffix': sysconfig.get_config_var('EXT_SUFFIX'), 'compiler_exit_code': result.returncode,
        'compiler_command': result.stdout.splitlines()[0] if result.stdout else '',
        'compiler_stdout_sha256': loader.sha(result.stdout.encode()),
        'compiler_stderr_sha256': loader.sha(result.stderr.encode()), 'hardware_opened': False,
        'native_library_loaded': False, **dict.fromkeys(loader.FLAGS, False)}
    (out / 'build-receipt.json').write_text(json.dumps(receipt, indent=2, sort_keys=True) + '\n')
    loader._bundle(plan['source_bundle'])
    return receipt


def artifact(bundle, expected_sha256, original_observer, receipt_path):
    bundle_ref = {'path': str(bundle), 'sha256': expected_sha256}
    data = loader._bundle(bundle_ref)
    folder = loader.reference(bundle_ref).parent
    ref = lambda path: {'path': str(path), 'sha256': loader.sha(Path(path).read_bytes())}
    receipt_ref = ref(receipt_path); receipt = loader.parse(loader.read(receipt_ref))
    refs = {'source_bundle': bundle_ref, 'original_observer': ref(original_observer),
            'build_receipt': receipt_ref, 'library': receipt['library']}
    for key, name in {'copied_observer': 'copy_observer.py', 'benchmark': 'fk_cache_diagnostic_benchmark.py',
        'native_source': '_event_snapshot_native.c', 'native_builder': 'build_native_event_copy.py',
        'selector': 'native_snapshot_diagnostic_r49/snapshot_loader.py',
        'support': 'native_snapshot_diagnostic_r49/snapshot_support.py',
        'generator': 'native_snapshot_diagnostic_r49/snapshot_generate.py', 'child': 'diagnostic_child.py'}.items():
        refs[key] = {'path': str(folder / name), 'sha256': data['files'][name]}
    return loader.create_manifest(refs)


def rebind_fk_manifest(original, bundle_manifest, folder):
    """Only five explicit integration references change; model evidence stays exact."""
    integration = original.get('integration_sources')
    expected = {'diagnostic_loader.py', 'diagnostic_support.py', 'diagnostic_generate.py',
                'diagnostic_child.py', 'fk_cache_diagnostic_benchmark.py'}
    loader.require(type(integration) is dict and set(integration) == expected,
                   'Exact original FK integration inventory required')
    from . import snapshot_generate as generate
    for key in expected:
        loader.require(integration[key]['sha256'] == generate.R47_FILES[
                'native_policy_overnight/target_tail_fk_cache/' + key if key == 'diagnostic_loader.py' else key],
                       'Unknown original FK integration source rejected')
    result = loader.parse(json.dumps(original, allow_nan=False))
    result['integration_sources'] = {}
    for name in expected:
        key = 'native_policy_overnight/target_tail_fk_cache/' + name if name == 'diagnostic_loader.py' else name
        result['integration_sources'][name] = {'path': str(Path(folder) / key), 'sha256': bundle_manifest['files'][key]}
    before = dict(original); after = dict(result)
    del before['integration_sources']; del after['integration_sources']
    loader.require(before == after, 'FK model/evidence references changed')
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument('--source-bundle', type=Path, required=True)
    p.add_argument('--source-bundle-sha256', required=True)
    p.add_argument('--original-observer', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--artifact-output', type=Path)
    p.add_argument('--execute', action='store_true')
    args = p.parse_args(argv)
    plan = build_plan(args.source_bundle, args.source_bundle_sha256, args.original_observer, args.output)
    if not args.execute:
        print(json.dumps(plan, indent=2)); return 0
    if args.artifact_output is None:
        p.error('Explicit fresh artifact output required for build')
    path = _fresh(args.artifact_output)
    build(args.source_bundle, expected_sha256=args.source_bundle_sha256,
          original_observer=args.original_observer, output=args.output)
    data = artifact(args.source_bundle, args.source_bundle_sha256, args.original_observer, args.output / 'build-receipt.json')
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')
    loader.validate(path, loader.sha(path.read_bytes()))
    print(json.dumps({'status': 'BUILT_FILE_ONLY', 'artifact': str(path), 'hardware_opened': False,
                      'native_library_loaded': False, **dict.fromkeys(loader.FLAGS, False)}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
