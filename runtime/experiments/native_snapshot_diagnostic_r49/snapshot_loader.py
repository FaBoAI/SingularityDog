"""File-only PLAN and scoped observer selection; no native import at module load."""
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import sys
import sysconfig
from types import ModuleType, BuiltinFunctionType

OBSERVER_SHA = '21a18c24b19fc9eb38f2d9f172556f4928dec07a36eedc2d47a41b9cd990b0ad'
NATIVE_SHA = 'cafd55c92e4c61afabaf96ba7c02d6af70f02cb8581636658c7edb44c513b27a'
BUILDER_SHA = '7299ae7294bcebbffa000cc80bcbdfeb9413342c61809b544f8f133f34790df3'
SCHEMA = 'singularitydog.snapshot-copy-stop-diagnostic-artifact.r49.v1'
BUNDLE_SCHEMA = 'singularitydog.snapshot-copy-stop-diagnostic-source-bundle.r49.v1'
FLAGS = ('output_allowed', 'approved_for_runtime', 'active_controller_qualification',
         'timing_admission_eligible', 'live_50hz_verified')
REFS = {'original_observer', 'copied_observer', 'native_source', 'native_builder', 'library',
        'build_receipt', 'benchmark', 'selector', 'support', 'generator', 'child', 'source_bundle'}
PRIVATE_PACKAGE = '_sd_snapshot_diag_r49'
PRIVATE_NATIVE = PRIVATE_PACKAGE + '._event_snapshot_native'
PRIVATE_OBSERVER = 'singularitydog_hw._explicit_snapshot_observer_r49'
_ACTIVE = None


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def reference(ref):
    require(type(ref) is dict and set(ref) == {'path', 'sha256'} and
            type(ref['path']) is str and type(ref['sha256']) is str and
            re.fullmatch('[0-9a-f]{64}', ref['sha256']), 'Exact path/SHA256 required')
    path = Path(ref['path'])
    require(path.is_absolute() and '..' not in path.parts and
            not any(p.is_symlink() for p in (path, *path.parents)), 'Absolute non-symlink path required')
    return path


def read(ref):
    path = reference(ref)
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode) and before.st_size <= 64 * 1024 * 1024,
                'Bounded regular file required')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(before.st_size + 1)
        after = os.fstat(fd)
    finally:
        os.close(fd)
    require(len(raw) == before.st_size and
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) ==
            (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) and
            sha(raw) == ref['sha256'], 'Pinned file changed: ' + str(path))
    return raw


def parse(raw):
    def pairs(rows):
        result = {}
        for key, value in rows:
            require(key not in result, 'Duplicate JSON key')
            result[key] = value
        return result
    def invalid(value):
        raise ValueError('Nonfinite JSON constant')
    result = json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid)
    require(type(result) is dict, 'JSON object required')
    return result


def _bundle(ref):
    from . import snapshot_generate as generate
    data = parse(read(ref))
    require(data.get('schema') == BUNDLE_SCHEMA and
            set(data.get('files', {})) == generate.NAMES and
            data.get('baseline_kit_manifest_sha256') == generate.KIT_SHA and
            data.get('baseline_observer_sha256') == OBSERVER_SHA and
            data.get('baseline_fk_benchmark_sha256') == generate.FK_BENCHMARK_SHA and
            all(data.get(flag) is False for flag in FLAGS), 'Exact source bundle scope required')
    folder = reference(ref).parent
    require(data.get('target_bundle_path') == str(folder), 'Bundle location differs')
    for name, pin in data['files'].items():
        read({'path': str(folder / name), 'sha256': pin})
    baseline = read({'path': str(folder / 'baseline_fk_benchmark.py'),
                     'sha256': generate.FK_BENCHMARK_SHA})
    benchmark, proof = generate.derive_benchmark(baseline)
    require(read({'path': str(folder / 'fk_cache_diagnostic_benchmark.py'),
                  'sha256': data['files']['fk_cache_diagnostic_benchmark.py']}) == benchmark and
            data.get('benchmark_derivation') == proof, 'Benchmark inverse proof differs')
    child, proof = generate.derive_child(read({'path': str(folder / 'baseline_diagnostic_child.py'),
                                             'sha256': generate.FK_CHILD_SHA}))
    require(read({'path': str(folder / 'diagnostic_child.py'),
                  'sha256': data['files']['diagnostic_child.py']}) == child and
            data.get('child_derivation') == proof, 'Child inverse proof differs')
    require(data['files']['native_snapshot_diagnostic_r49/snapshot_loader.py'] ==
            sha(read({'path': str(Path(__file__).absolute()),
                      'sha256': data['files']['native_snapshot_diagnostic_r49/snapshot_loader.py']})),
            'Executing selector differs')
    for name, pin in generate.R47_FILES.items():
        if name not in ('fk_cache_diagnostic_benchmark.py', 'diagnostic_child.py'):
            require(data['files'][name] == pin, 'R47 dependency changed: ' + name)
    require(data['files']['_event_snapshot_native.c'] == NATIVE_SHA and
            data['files']['build_native_event_copy.py'] == BUILDER_SHA, 'Unknown native source rejected')
    return data


def validate(manifest, expected_sha256):
    from . import snapshot_generate as generate
    manifest_ref = {'path': str(manifest), 'sha256': expected_sha256}
    data = parse(read(manifest_ref))
    require(set(data) == {'schema', 'status', 'references', *FLAGS} and data['schema'] == SCHEMA and
            data['status'] == 'PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL' and
            all(data[flag] is False for flag in FLAGS), 'Unapproved diagnostic artifact required')
    refs = data['references']
    require(type(refs) is dict and set(refs) == REFS, 'Complete explicit snapshot references required')
    raw = {name: read(ref) for name, ref in refs.items()}
    require(refs['original_observer']['sha256'] == OBSERVER_SHA and
            refs['native_source']['sha256'] == NATIVE_SHA and
            refs['native_builder']['sha256'] == BUILDER_SHA, 'Unknown original/native source rejected')
    copied, observer_proof = generate.derive_observer(raw['original_observer'])
    require(copied == raw['copied_observer'], 'Copied observer inverse proof differs')
    bundle = _bundle(refs['source_bundle'])
    folder = reference(refs['source_bundle']).parent
    bindings = {'copied_observer': 'copy_observer.py', 'benchmark': 'fk_cache_diagnostic_benchmark.py',
        'native_source': '_event_snapshot_native.c', 'native_builder': 'build_native_event_copy.py',
        'selector': 'native_snapshot_diagnostic_r49/snapshot_loader.py',
        'support': 'native_snapshot_diagnostic_r49/snapshot_support.py',
        'generator': 'native_snapshot_diagnostic_r49/snapshot_generate.py', 'child': 'diagnostic_child.py'}
    for key, name in bindings.items():
        require(refs[key] == {'path': str(folder / name), 'sha256': bundle['files'][name]},
                'Executing source selection differs: ' + key)
    require(bundle.get('observer_derivation') == observer_proof, 'Observer derivation binding differs')
    receipt = parse(raw['build_receipt'])
    require(receipt.get('schema') == 'singularitydog.snapshot-copy-native-build.r49.v1' and
            receipt.get('status') == 'BUILT_FILE_ONLY' and receipt.get('source_sha256') == NATIVE_SHA and
            receipt.get('builder_sha256') == BUILDER_SHA and
            receipt.get('library') == refs['library'] and receipt.get('compiler_exit_code') == 0 and
            type(receipt.get('compiler_command')) is str and receipt['compiler_command'] and
            receipt.get('implementation') == 'cpython' and
            type(receipt.get('python_version')) is str and type(receipt.get('extension_suffix')) is str and
            all(receipt.get(flag) is False for flag in ('hardware_opened', 'output_allowed',
                'approved_for_runtime', 'active_controller_qualification')), 'Native build identity incomplete')
    read(manifest_ref)
    return data, bundle, receipt


def plan(manifest, *, expected_sha256):
    data, _, _ = validate(manifest, expected_sha256)
    return {'schema': 'singularitydog.snapshot-copy-stop-diagnostic-plan.r49.v1',
            'manifest': {'path': str(manifest), 'sha256': expected_sha256},
            'expected_executing_sources': data['references'], 'observer_import_only_change': True,
            'native_library_loaded': False, 'observer_selected': False, 'hardware_opened': False,
            **dict.fromkeys(FLAGS, False)}


@contextmanager
def select(benchmark, manifest, *, expected_sha256):
    """Select only the copied benchmark alias, and restore it on every exit."""
    global _ACTIVE
    require(_ACTIVE is None, 'Nested observer selection rejected')
    data, _, receipt = validate(manifest, expected_sha256)
    refs = data['references']
    require(Path(__file__).absolute() == reference(refs['selector']), 'Executing selector path differs')
    require(sys.implementation.name == 'cpython' and receipt['python_version'] == sys.version and
            receipt['extension_suffix'] == sysconfig.get_config_var('EXT_SUFFIX'), 'Native Python ABI differs')
    require(Path(benchmark.__file__).absolute() == reference(refs['benchmark']),
            'Copied benchmark selection differs')
    original = benchmark.observer
    require(type(original) is ModuleType and
            Path(original.__file__).absolute() == reference(refs['original_observer']) and
            original.__name__ == 'singularitydog_hw.policy_observer', 'Original observer module differs')
    read(refs['original_observer'])
    require(not any(name == PRIVATE_OBSERVER or name == PRIVATE_PACKAGE or
                    name.startswith(PRIVATE_PACKAGE + '.') for name in sys.modules),
            'Private snapshot namespace already bound')
    ordinary = dict(original.__dict__)
    aliases = []
    copied = None
    copied_globals = None
    primary_error = None
    try:
        parent = ModuleType(PRIVATE_PACKAGE); parent.__path__ = []
        sys.modules[PRIVATE_PACKAGE] = parent; aliases.append((PRIVATE_PACKAGE, parent))
        spec = importlib.util.spec_from_file_location(PRIVATE_NATIVE, reference(refs['library']))
        native = importlib.util.module_from_spec(spec)
        sys.modules[PRIVATE_NATIVE] = native; aliases.append((PRIVATE_NATIVE, native))
        spec.loader.exec_module(native)
        # Single-phase CPython extensions may recreate a module wrapper from
        # the import cache while its builtin still owns the first wrapper.
        owner = native.snapshot_event.__self__
        require(type(native.snapshot_event) is BuiltinFunctionType and
                native.snapshot_event.__name__ == 'snapshot_event' and
                type(owner) is ModuleType and owner.__dict__.get('__name__') == PRIVATE_NATIVE and
                owner.__dict__.get('snapshot_event') is native.snapshot_event and
                Path(owner.__dict__.get('__file__', '')).absolute() == reference(refs['library']) and
                Path(native.__file__).absolute() == reference(refs['library']), 'Native copier binding differs')
        copied = ModuleType(PRIVATE_OBSERVER)
        copied.__package__ = 'singularitydog_hw'; copied.__file__ = str(reference(refs['copied_observer']))
        exec(compile(read(refs['copied_observer']), copied.__file__, 'exec'), copied.__dict__)
        copied.ObserverError = original.ObserverError
        copied._OWNERS = original._OWNERS
        require(copied.snapshot_event is native.snapshot_event, 'Copied observer copier differs')
        sys.modules[PRIVATE_OBSERVER] = copied; aliases.append((PRIVATE_OBSERVER, copied))
        _ACTIVE = {'benchmark': benchmark, 'original': original, 'copied': copied,
                   'manifest': {'path': str(manifest), 'sha256': expected_sha256}}
        benchmark.observer = copied
        copied_globals = dict(copied.__dict__)
        yield copied
    except BaseException as error:
        primary_error = error
        raise
    finally:
        failures = []
        bindings_unchanged = False
        observer_restored = False
        ordinary_unchanged = False
        sources_unchanged = False
        try:
            bindings_unchanged = (copied_globals is None or
                (type(copied) is ModuleType and type(owner) is ModuleType and
                 benchmark.observer is copied and copied.__dict__.keys() == copied_globals.keys() and
                 all(copied.__dict__[key] is value for key, value in copied_globals.items()) and
                 owner.__dict__.get('snapshot_event') is copied.__dict__.get('snapshot_event') and
                 all(sys.modules.get(name) is module for name, module in aliases)))
            require(bindings_unchanged, 'Selected observer/native bindings changed')
        except BaseException as error:
            failures.append(error)
        try:
            benchmark.observer = original
            observer_restored = True
        except BaseException as error:
            failures.append(error)
        _ACTIVE = None
        # This namespace was absent at entry. Restore that absence even if a
        # callback replaced one of our entries; do not leave a foreign alias.
        for name in tuple(sys.modules):
            if name == PRIVATE_OBSERVER or name == PRIVATE_PACKAGE or name.startswith(PRIVATE_PACKAGE + '.'):
                sys.modules.pop(name, None)
        try:
            require(type(original) is ModuleType and original.__dict__.keys() == ordinary.keys() and
                    all(original.__dict__[key] is value for key, value in ordinary.items()),
                    'Ordinary observer globals changed')
            ordinary_unchanged = True
        except BaseException as error:
            failures.append(error)
        try:
            validate(manifest, expected_sha256)
            sources_unchanged = True
        except BaseException as error:
            failures.append(error)
        if failures:
            message = 'Snapshot selection cleanup: ' + '; '.join(type(e).__name__ + ': ' + str(e) for e in failures)
            if primary_error is not None:
                primary_error.add_note(message)
            else:
                error = ValueError(message)
                error.selection_restored = observer_restored and not any(
                    name == PRIVATE_OBSERVER or name == PRIVATE_PACKAGE or name.startswith(PRIVATE_PACKAGE + '.')
                    for name in sys.modules)
                error.ordinary_module_globals_unchanged = ordinary_unchanged
                error.selected_bindings_unchanged = bindings_unchanged
                error.sources_unchanged_after_run = sources_unchanged
                raise error from failures[0]


def create_manifest(references):
    require(type(references) is dict and set(references) == REFS, 'Complete explicit references required')
    for ref in references.values():
        reference(ref)
    return parse(json.dumps({'schema': SCHEMA, 'status': 'PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL',
        'references': references, **dict.fromkeys(FLAGS, False)}, allow_nan=False))
