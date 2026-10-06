"""Seal a separate R50 source bundle and file-only command specifications.

This module uses only the standard library.  It never imports the adapter,
Torch, a source dependency, or a native library, and never runs a command.
"""
import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import tarfile


SCHEMA = 'singularitydog.saved-pipeline-source-bundle.r50.v1'
IMPLEMENTATION_STATUS = 'INCOMPLETE_ACTUAL_OBSERVER_MODEL_EXEC_NOT_IMPLEMENTED'
SOURCE_NAMES = frozenset(('__init__.py', 'adapter.py', 'inputs.py', 'support.py',
    'package.py', 'test_adapter.py', 'test_inputs.py', 'test_package.py', 'README.md'))
REFERENCE_PINS = {
    'report': '4369859814b0d864c67e42714187ce1b527771b70eeeb94cc7abd6ed0e5c6260',
    'records': 'cf1e75e54f119d78db5afdf463c37ac9fce78eba7da35a2398b36ca625f15d71',
    'calibration': '21eedf31f4d548f663e19669e48edc60f5f84ad355e32d49631a96e4ebfd7c0e',
    'mount': 'b9f6814e13141e82efca0fbb64772847ae7ad91434282b8c9b1e038b3c42b420',
    'gyro_bias': '2ade7e77705f51f077386154bbc70180f3cec614cc5ecaea2c439dac964cb8f8',
    'accel_hypothesis': '6cbc3426b5bd0b1f4daf090129704c5584af5b51efac661a4f6e83d8e4975415',
    'r48_source_manifest': 'df8f43f3d7627afb14e481569ce41a607e54780674018010f6d0db5defa729f4',
    'r47_source_manifest': '1e08148e7426172f6123c699e8ce52f10663adb2ab6adaf08a6623048aca9a17',
    'fk_candidate_original': 'd68c138ba5345010fd9020b3e49d89ffbccec6fed3515c30d19360b42c695220',
    'fk_candidate_rebound': '23b40c517b4351cd4c52458e142eb4580546379775afa5c5ced1d0e6db8a6be6',
    'r49_source_manifest': 'd665a415aa646662a339b98a00910f06e400ee0bbf4cb1f0b4bf0072aeeef6cc',
    'snapshot_artifact': '41552932cb23ef163c56df57f8abaa3ba9febd572bc869cc5b9f5f2cdad164ec',
}
KIT_PIN = 'f3d48a7cabae2e502863f5e87aed2ba1b076faa292d661f59572a79b81f896fe'
FALSE_FLAGS = ('hardware_opened', 'output_allowed', 'approved_for_runtime',
    'active_controller_qualification', 'timing_admission_eligible',
    'live_50hz_verified', 'whole_20ms_gain_proven', 'ordinary_runtime_changed')
INPUT_COPIES = {'calibration': 'calibration.json', 'gyro_bias': 'gyro-bias.json',
    'accel_hypothesis': 'accel-hypothesis.json',
    'fk_candidate_rebound': 'fk-candidate-rebound-r49.json'}


class MissingCounterpart(ValueError):
    pass


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def absolute(value):
    require(type(value) is str and value.startswith('/') and '\\' not in value,
            'Absolute path required')
    path = Path(value)
    require(str(path) == value and '..' not in path.parts, 'Canonical path required')
    return path


def member(value):
    require(type(value) is str and value != '' and '\\' not in value,
            'Relative member required')
    path = PurePosixPath(value)
    require(not path.is_absolute() and str(path) == value and
            '..' not in path.parts and '.' not in path.parts,
            'Canonical relative member required')
    return value


def pin(value):
    require(type(value) is str and re.fullmatch('[0-9a-f]{64}', value) is not None,
            'SHA256 pin required')
    return value


def no_symlinks(path):
    require(not any(p.is_symlink() for p in (path, *path.parents)),
            'Symlink path rejected: ' + str(path))


def read_pinned(path, expected, *, max_size=128 * 1024 * 1024):
    path = absolute(str(path)); pin(expected); no_symlinks(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode) and before.st_size <= max_size,
                'Bounded regular file required: ' + str(path))
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(max_size + 1)
        after = os.fstat(fd)
        require((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) ==
                (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) and
                len(raw) == before.st_size and sha(raw) == expected,
                'Pinned file changed: ' + str(path))
        no_symlinks(path)
        return raw
    finally:
        os.close(fd)


def parse(raw):
    def pairs(values):
        result = {}
        for key, value in values:
            require(key not in result, 'Duplicate JSON key rejected')
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON rejected')))


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()


def exact_source_files(source_dir, source_pins):
    source_dir = absolute(str(source_dir)); no_symlinks(source_dir)
    require(type(source_pins) is dict and set(source_pins) == SOURCE_NAMES,
            'Exact nine R50 source pins required')
    require({path.name for path in source_dir.iterdir()} == SOURCE_NAMES,
            'Exact nine regular R50 source files required; extra files rejected')
    return {name: read_pinned(source_dir / name, pin(source_pins[name]))
            for name in sorted(SOURCE_NAMES)}


def verify_references(references, local_references, path_mappings, path_overrides=None,
                      *, allow_nested_pending=False):
    """Authenticate core files, three source bundles, K37, and nested refs."""
    require(type(references) is dict and set(references) == set(REFERENCE_PINS),
            'Exact twelve historical reference roles required')
    require(type(local_references) is dict and set(local_references) == set(references),
            'Exact twelve explicit local counterparts required')
    require(type(path_mappings) is dict and type(path_overrides or {}) is dict,
            'Explicit local path mapping required')
    mappings = [(absolute(target), absolute(local)) for target, local in path_mappings.items()]
    overrides = {str(absolute(target)): absolute(local)
                 for target, local in (path_overrides or {}).items()}
    checked = {}
    pending = {}

    def resolve(target):
        target = absolute(target)
        if str(target) in overrides:
            return overrides[str(target)]
        choices = [(base, local) for base, local in mappings if target.is_relative_to(base)]
        if not choices:
            raise MissingCounterpart('Missing local counterpart: ' + str(target))
        base, local = max(choices, key=lambda item: len(item[0].parts))
        return local / target.relative_to(base)

    def check(target, expected, local=None):
        target = str(absolute(target)); pin(expected)
        if local is None and target in checked:
            local = checked[target]['local_path']
        local = absolute(str(local)) if local is not None else resolve(target)
        binding = {'local_path': str(local), 'sha256': expected}
        require(target not in checked or checked[target] == binding,
                'Conflicting target pin/local counterpart rejected: ' + target)
        raw = read_pinned(local, expected)
        checked[target] = binding
        return raw

    documents = {}
    for role, ref in references.items():
        require(type(ref) is dict and set(ref) == {'path', 'sha256'} and
                ref['sha256'] == REFERENCE_PINS[role], 'Historical reference differs: ' + role)
        raw = check(ref['path'], ref['sha256'], local_references[role])
        if role not in ('gyro_bias', 'accel_hypothesis', 'calibration', 'mount'):
            documents[role] = parse(raw)

    for role, key, count in (('r48_source_manifest', 'source_sha256', 21),
                            ('r47_source_manifest', 'files', 13),
                            ('r49_source_manifest', 'files', 23)):
        doc = documents[role]; inventory = doc[key]
        require(type(inventory) is dict and len(inventory) == count,
                'Exact frozen inventory required: ' + role)
        root = absolute(references[role]['path']).parent
        if 'target_bundle_path' in doc:
            require(doc['target_bundle_path'] == str(root), 'Declared source root differs')
        for name, expected in inventory.items():
            check(str(root / member(name)), expected)

    r49 = documents['r49_source_manifest']
    require(r49['baseline_kit_manifest_sha256'] == KIT_PIN, 'Original K37 pin required')
    kit = absolute(r49['baseline_kit_path'])
    kit_doc = parse(check(str(kit / 'kit-manifest.json'), KIT_PIN))
    require(type(kit_doc['files']) is dict and len(kit_doc['files']) == 650,
            'Exact K37 inventory required')
    for name, expected in kit_doc['files'].items():
        check(str(kit / member(name)), expected)

    for role in ('fk_candidate_original', 'fk_candidate_rebound', 'snapshot_artifact'):
        doc = documents[role]
        for key in ('references', 'source_references', 'generated_source_references', 'integration_sources'):
            for ref in doc.get(key, {}).values():
                require(type(ref) is dict and set(ref) == {'path', 'sha256'},
                        'Exact nested reference required')
                try:
                    check(ref['path'], ref['sha256'])
                except (MissingCounterpart, FileNotFoundError):
                    if not allow_nested_pending:
                        raise
                    target = str(absolute(ref['path'])); pin(ref['sha256'])
                    require(target not in pending or pending[target] == ref['sha256'],
                            'Conflicting pending target pin rejected')
                    pending[target] = ref['sha256']
    return checked, pending


def archive_bytes(members):
    """Deterministic archive: only exact pinned regular members, no directories."""
    raw = io.BytesIO()
    with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as zipped:
        with tarfile.open(fileobj=zipped, mode='w', format=tarfile.USTAR_FORMAT) as archive:
            for name, value in sorted(members.items()):
                info = tarfile.TarInfo(member(name)); info.size = len(value)
                info.mode = 0o600; info.mtime = 0; info.uid = info.gid = 0
                archive.addfile(info, io.BytesIO(value))
    return raw.getvalue()


def verify_archive(raw, member_pins):
    with tarfile.open(fileobj=io.BytesIO(raw), mode='r:gz') as archive:
        members = archive.getmembers()
        require(len(members) == len(member_pins) and
                {item.name for item in members} == set(member_pins),
                'Exact regular archive members required')
        for item in members:
            require(item.isfile() and item.name == member(item.name), 'Nonregular archive member')
            require(sha(archive.extractfile(item).read()) == member_pins[item.name],
                    'Archive member hash differs')


def prepare(source_dir, destination, config):
    """Prepare fresh files; a specification requests work but proves no execution."""
    source_dir = absolute(str(source_dir)); destination = absolute(str(destination))
    no_symlinks(destination)
    require(not destination.exists() and destination.parent.is_dir(),
            'Fresh private package destination required')
    target = absolute(config['target_bundle_path'])
    output = absolute(config['target_output_directory'])
    input_target = absolute(config['target_inputs_directory'])
    python = str(absolute(config['python']))
    require(not target.is_relative_to(output) and not output.is_relative_to(target),
            'Source and result directories must be distinct')
    require(not input_target.is_relative_to(target) and not target.is_relative_to(input_target),
            'Input copies must be outside the source inventory')
    require(not input_target.is_relative_to(output) and not output.is_relative_to(input_target),
            'Input copies and result directories must be distinct')
    members = exact_source_files(source_dir, config['source_sha256'])
    local_pins, pending = verify_references(config['references'], config['local_references'],
        config['local_path_mappings'], config.get('local_path_overrides'),
        allow_nested_pending=config.get('allow_nested_target_validation_pending') is True)
    for role in ('r48_source_manifest', 'r47_source_manifest', 'r49_source_manifest'):
        existing = absolute(config['references'][role]['path']).parent
        require(all(not path.is_relative_to(existing) and not existing.is_relative_to(path)
                    for path in (target, input_target, output)),
                'Separate inventory outside immutable source roots required')
    r49 = parse(read_pinned(config['local_references']['r49_source_manifest'],
                           REFERENCE_PINS['r49_source_manifest']))
    existing_kit = absolute(r49['baseline_kit_path'])
    require(all(not path.is_relative_to(existing_kit) and not existing_kit.is_relative_to(path)
                for path in (target, input_target, output)),
            'Separate experiment outside original K37 required')
    copied_inputs = {}
    for role, name in INPUT_COPIES.items():
        require(config['references'][role]['path'] == str(input_target / name),
                'Exact separate copied-input target required: ' + role)
        copied_inputs[name] = read_pinned(config['local_references'][role], REFERENCE_PINS[role])
    manifest = {'schema': SCHEMA, 'source_files': dict(config['source_sha256']),
        'references': config['references'], 'target_bundle_path': str(target),
        'mode': 'HISTORICAL_SAVED_PIPELINE_ONLY', **dict.fromkeys(FALSE_FLAGS, False)}
    members['manifest.json'] = json_bytes(manifest)
    member_pins = {name: sha(raw) for name, raw in members.items()}
    archive = archive_bytes(members); verify_archive(archive, member_pins)
    input_member_pins = {name: sha(raw) for name, raw in copied_inputs.items()}
    input_archive = archive_bytes(copied_inputs); verify_archive(input_archive, input_member_pins)
    common = [python, '-I', '-B', str(target / 'adapter.py'), '--manifest',
              str(target / 'manifest.json'), '--manifest-sha256', member_pins['manifest.json']]
    specs = {}
    for mode in ('PLAN', 'EXEC'):
        specs[mode] = {'schema': 'private.r50-saved-pipeline-command-spec.v1',
            'candidate_implementation_status': IMPLEMENTATION_STATUS,
            'target_run_permitted': False,
            'target_run_prohibited_until_finished_and_reviewed': True,
            'mode': mode, 'argv': common + ['--output-dir', str(output), '--mode', mode],
            'source_members': member_pins, 'target_source_directory': str(target),
            'target_output_directory': str(output),
            'input_files': {**{path: value['sha256'] for path, value in local_pins.items()}, **pending},
            'nested_target_reference_validation_pending': bool(pending),
            'nested_local_validation_scope': 'DIRECT_CANDIDATE_AND_ARTIFACT_REFERENCES_ONLY',
            'recursive_historical_target_reference_validation_pending': True,
            'target_input_directory': str(input_target), 'input_archive_members': input_member_pins,
            'environment': {'PYTHONDONTWRITEBYTECODE': '1'},
            'requested_model_load': mode == 'EXEC', 'model_loaded': False,
            'requested_native_library_load': mode == 'EXEC',
            'native_library_loaded': False, 'output_directory_created': False,
            'target_executed': False, **dict.fromkeys(FALSE_FLAGS, False)}
        if config.get('boot_id') is not None:
            require(type(config['boot_id']) is str and
                    re.fullmatch('[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', config['boot_id']),
                    'Explicit boot UUID required')
            specs[mode]['outer_scope_boot_id'] = config['boot_id']
            specs[mode]['adapter_requires_current_boot_for_historical_inputs'] = False

    # Re-read every authenticated input and source before creating any output.
    require(exact_source_files(source_dir, config['source_sha256']) ==
            {name: members[name] for name in SOURCE_NAMES}, 'R50 source drift')
    for binding in local_pins.values():
        read_pinned(binding['local_path'], binding['sha256'])
    destination.mkdir(mode=0o700)
    source_out = destination / 'source'; source_out.mkdir(mode=0o700)
    for name, raw in members.items():
        with (source_out / name).open('xb') as stream:
            stream.write(raw)
    input_out = destination / 'inputs'; input_out.mkdir(mode=0o700)
    for name, raw in copied_inputs.items():
        with (input_out / name).open('xb') as stream:
            stream.write(raw)
    outputs = {'r50-source.tar.gz': archive, 'r50-inputs.tar.gz': input_archive,
               'plan-spec.json': json_bytes(specs['PLAN']),
               'exec-spec.json': json_bytes(specs['EXEC'])}
    for name, raw in outputs.items():
        with (destination / name).open('xb') as stream:
            stream.write(raw)
    exact_source_files(source_dir, config['source_sha256'])
    for binding in local_pins.values():
        read_pinned(binding['local_path'], binding['sha256'])
    for name, expected in member_pins.items():
        read_pinned(source_out / name, expected)
    for name, expected in input_member_pins.items():
        read_pinned(input_out / name, expected)
    for name, raw in outputs.items():
        read_pinned(destination / name, sha(raw))
    receipt = {'schema': 'private.r50-saved-pipeline-package-receipt.v1',
        'status': 'PASS_INCOMPLETE_CODE_ONLY_SCAFFOLD_SOURCE_AND_INPUT_FILE_SEAL',
        'candidate_implementation_status': IMPLEMENTATION_STATUS,
        'target_run_permitted': False,
        'target_run_prohibited_until_finished_and_reviewed': True,
        'source_directory': str(source_dir), 'local_source_directory': str(source_out),
        'target_source_directory': str(target), 'manifest_sha256': member_pins['manifest.json'],
        'source_members': member_pins, 'archive_members': member_pins,
        'archive_sha256': sha(archive), 'local_reference_counterparts': config['local_references'],
        'input_archive_sha256': sha(input_archive), 'input_archive_members': input_member_pins,
        'local_input_directory': str(input_out), 'target_input_directory': str(input_target),
        'nested_target_reference_validation_pending': bool(pending),
        'nested_local_validation_scope': 'DIRECT_CANDIDATE_AND_ARTIFACT_REFERENCES_ONLY',
        'recursive_historical_target_reference_validation_pending': True,
        'pending_nested_target_references': pending,
        'verified_local_files': local_pins, 'source_and_input_pins_unchanged_after': True,
        'generated_files_sha256': {name: sha(raw) for name, raw in outputs.items()},
        'plan_spec_sha256': sha(outputs['plan-spec.json']),
        'exec_spec_sha256': sha(outputs['exec-spec.json']),
        'model_loaded': False, 'native_library_loaded': False,
        'target_executed': False, **dict.fromkeys(FALSE_FLAGS, False)}
    with (destination / 'package-receipt.json').open('xb') as stream:
        stream.write(json_bytes(receipt))
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--source-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--config-sha256', required=True)
    args = parser.parse_args(argv)
    config = parse(read_pinned(args.config, args.config_sha256))
    receipt = prepare(args.source_dir, args.output_dir, config)
    print(json.dumps({'status': receipt['status'], 'source_members': len(receipt['source_members']),
                     'manifest_sha256': receipt['manifest_sha256'], 'model_loaded': False}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
