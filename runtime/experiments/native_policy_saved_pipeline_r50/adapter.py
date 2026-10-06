"""INCOMPLETE code-only R50 saved pipeline candidate. PLAN reads files only.

Actual Observer/model replay has not been implemented or executed. EXEC rejects
before importing Torch, loading a native library, or creating a result directory.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
from types import ModuleType

NAMES = frozenset(('__init__.py', 'adapter.py', 'inputs.py', 'support.py', 'package.py',
                  'test_adapter.py', 'test_inputs.py', 'test_package.py', 'README.md'))
PRIVATE = '_sd_saved_pipeline_r50'


def _read(filename, pin):
    filename = Path(filename)
    if (not filename.is_absolute() or '..' in filename.parts or
            any(p.is_symlink() for p in (filename, *filename.parents)) or
            type(pin) is not str or not re.fullmatch('[0-9a-f]{64}', pin)):
        raise ValueError('Explicit absolute nonsymlink file/pin required')
    fd = os.open(filename, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or not 0 <= before.st_size <= 4 * 1024 * 1024:
            raise ValueError('Bounded regular bootstrap file required')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(before.st_size + 1)
        after = os.fstat(fd)
        named = filename.lstat()
        identity = lambda item: (item.st_dev, item.st_ino, item.st_size, item.st_mtime_ns)
        if (identity(before) != identity(after) or identity(after) != identity(named) or
                len(raw) != before.st_size or hashlib.sha256(raw).hexdigest() != pin):
            raise ValueError('Bootstrap pin changed')
        return raw
    finally:
        os.close(fd)


def bootstrap(manifest, pin):
    """Execute only exact pinned local package bytes; stale pyc cannot run."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('Duplicate bootstrap JSON key')
            result[key] = value
        return result
    document = json.loads(_read(manifest, pin), object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite bootstrap JSON')))
    folder = Path(manifest).parent
    if (document.get('target_bundle_path') != str(folder) or
            Path(__file__).absolute() != folder / 'adapter.py' or
            set(document.get('source_files', {})) != NAMES or
            {str(p.relative_to(folder)) for p in folder.rglob('*')} != NAMES | {'manifest.json'}):
        raise ValueError('Exact executing nine-source inventory required')
    raw = {name: _read(folder / name, expected) for name, expected in document['source_files'].items()}
    if any(name == PRIVATE or name.startswith(PRIVATE + '.') for name in sys.modules):
        raise ValueError('Fresh private bootstrap namespace required')
    sys.dont_write_bytecode = True
    package = ModuleType(PRIVATE)
    package.__package__ = PRIVATE
    package.__path__ = [str(folder)]
    package.__file__ = str(folder / '__init__.py')
    sys.modules[PRIVATE] = package
    try:
        exec(compile(raw['__init__.py'], package.__file__, 'exec'), package.__dict__)
        for name in ('inputs', 'support'):
            module = ModuleType(PRIVATE + '.' + name)
            module.__package__ = PRIVATE
            module.__file__ = str(folder / (name + '.py'))
            sys.modules[module.__name__] = module
            setattr(package, name, module)
            exec(compile(raw[name + '.py'], module.__file__, 'exec'), module.__dict__)
        return package.inputs, package.support
    except BaseException:
        for name in tuple(sys.modules):
            if name == PRIVATE or name.startswith(PRIVATE + '.'):
                sys.modules.pop(name, None)
        raise


def plan(inputs, support, manifest, pin, output):
    output = inputs.path(str(output))
    inputs.require(not output.exists() and output.parent.is_dir(), 'Fresh result path required even for PLAN')
    inputs.require(not output.is_relative_to(Path(manifest).parent) and
                   not Path(manifest).parent.is_relative_to(output), 'Result/source scopes must be separate')
    document, docs, inventories, kit, closure = inputs.verify_bundle(manifest, pin, executing_adapter=__file__)
    rows, partial = inputs.archived_rows(docs['report'], docs['records'], docs['calibration'], document['references'])
    closure.verify()
    return {'schema': 'singularitydog.saved-pipeline-code-only-plan.r50.v1',
        'status': 'PASS_FILE_PINS_ONLY_INCOMPLETE_EXEC',
        'candidate_implementation_status': support.IMPLEMENTATION_STATUS,
        'manifest': {'path': str(manifest), 'sha256': pin},
        'archived_report_status': docs['report']['status'],
        'complete_archived_ticks': len(rows), 'excluded_partial_cycle': partial['cycle'],
        'original_tick_ns': [row['observed']['tick_ns'] for row in rows],
        'verified_file_pins': dict(closure.pins), 'source_pins_unchanged_after': True,
        'planned_abba_labels_not_measurements': support.abba_schedule(),
        'measured_diagnostic_ticks_required': True, 'original_501_complete_loader_unchanged': True,
        'requested_model_load': False, 'model_loaded': False, 'native_library_loaded': False,
        'actual_observer_model_replay_executed': False, 'performance_measured': False,
        'target_run_permitted': False, 'output_directory_created': False,
        **dict.fromkeys(inputs.FALSE_FLAGS, False)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--mode', choices=('PLAN', 'EXEC'), default='PLAN')
    args = parser.parse_args(argv)
    # This candidate has no EXEC implementation. Reject even before bootstrap;
    # forged/missing input paths cannot accidentally cause a model import/load.
    if args.mode == 'EXEC':
        print(json.dumps({'status': 'INCOMPLETE_EXEC_REJECTED', 'candidate_implementation_status':
            'INCOMPLETE_ACTUAL_OBSERVER_MODEL_EXEC_NOT_IMPLEMENTED', 'requested_model_load': True,
            'model_loaded': False, 'native_library_loaded': False, 'hardware_opened': False,
            'output_directory_created': False, 'performance_measured': False, 'target_run_permitted': False}), file=sys.stderr)
        return 2
    inputs, support = bootstrap(args.manifest, args.manifest_sha256)
    result = plan(inputs, support, args.manifest, args.manifest_sha256, args.output_dir)
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
