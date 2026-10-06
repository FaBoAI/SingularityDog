"""Pre/post identities for the separate snapshot-copy STOP diagnostic."""
from pathlib import Path

from . import snapshot_loader as loader


class _BenchmarkBinding:
    def __init__(self, namespace):
        self.namespace = namespace
        self.__file__ = namespace['__file__']

    @property
    def observer(self):
        return self.namespace['observer']

    @observer.setter
    def observer(self, value):
        self.namespace['observer'] = value


def validate_cli(args, parser, executing_module, observer):
    try:
        proof = loader.plan(args.snapshot_manifest, expected_sha256=args.snapshot_manifest_sha256)
        refs = proof['expected_executing_sources']
        if (Path(executing_module).absolute() != loader.reference(refs['benchmark']) or
                Path(__file__).absolute() != loader.reference(refs['support']) or
                Path(loader.__file__).absolute() != loader.reference(refs['selector']) or
                Path(observer.__file__).absolute() != loader.reference(refs['original_observer']) or
                observer.__name__ != 'singularitydog_hw.policy_observer'):
            raise ValueError('Executing snapshot source binding differs')
        proof.update(selection_restored=None, error_class_shared=None, owner_registry_shared=None,
                     ordinary_module_globals_unchanged=None, selected_bindings_unchanged=None,
                     sources_unchanged_after_run=None)
        return proof
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.error(str(error))


def enter(args, namespace, proof):
    manager = loader.select(_BenchmarkBinding(namespace), args.snapshot_manifest,
                            expected_sha256=args.snapshot_manifest_sha256)
    copied = manager.__enter__()
    active = loader._ACTIVE
    proof.update(observer_selected=True, native_library_loaded=True,
        error_class_shared=copied.ObserverError is active['original'].ObserverError,
        owner_registry_shared=copied._OWNERS is active['original']._OWNERS)
    return manager


def restore(report, proof, manager):
    try:
        if manager is not None:
            manager.__exit__(None, None, None)
        proof.update(selection_restored=loader._ACTIVE is None,
                     ordinary_module_globals_unchanged=True if manager is not None else None,
                     selected_bindings_unchanged=True if manager is not None else None)
    except BaseException as error:
        proof.update(selection_restored=getattr(error, 'selection_restored', loader._ACTIVE is None),
            ordinary_module_globals_unchanged=getattr(error, 'ordinary_module_globals_unchanged', None),
            selected_bindings_unchanged=getattr(error, 'selected_bindings_unchanged', None))
        report['status'] = 'ABORTED'
        report.setdefault('errors', []).append('Snapshot selection restore: ' + type(error).__name__ + ': ' + str(error))


def finalize_report(report, proof):
    refs = proof['expected_executing_sources']
    changed = []
    for key, ref in {'manifest': proof['manifest'], **refs}.items():
        try:
            loader.read(ref)
        except (ValueError, OSError):
            changed.append(key)
    proof['sources_unchanged_after_run'] = not changed
    if (changed or not proof['observer_selected'] or proof['selection_restored'] is not True or
            proof['selected_bindings_unchanged'] is not True or
            proof['ordinary_module_globals_unchanged'] is not True):
        report['status'] = 'ABORTED'
        report.setdefault('errors', []).append('Snapshot diagnostic selection/source proof incomplete: ' + ','.join(changed))
    executing = {'singularitydog_hw/policy_observer.py': refs['copied_observer']['sha256'],
                 'singularitydog_hw/native_pipeline_benchmark.py': refs['benchmark']['sha256']}
    proof['actual_executing_sources'] = {key: refs[key] for key in
        ('copied_observer', 'benchmark', 'selector', 'support', 'generator', 'child', 'native_source', 'library', 'build_receipt')}
    for value in (report, report.get('source_provenance', {}), report.get('plan', {}).get('source_provenance', {})):
        value['executing_source_sha256'] = dict(executing)
        value['source_map_role'] = 'baseline dependency graph plus explicitly selected derived observer and benchmark'
    report['experimental_snapshot_copy_diagnostic'] = proof
    report.update(actual_output_qualification=False, full_controller_50Hz_verified=False,
                  **dict.fromkeys(loader.FLAGS, False))
