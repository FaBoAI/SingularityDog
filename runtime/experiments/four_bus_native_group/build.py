"""Explicit fresh composite native group build; never hardware admission."""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess

from experiments.four_bus_type1 import build as active_build

HERE = Path(__file__).resolve().parent
ORDINARY = active_build.ORDINARY
SUBSET_STOP = active_build.SUBSET_STOP
SUBSET_ACTIVE = active_build.EXTENSION
EXTENSION = HERE/'native_group.cpp'
LIBRARY_NAME = active_build.LIBRARY_NAME
SCHEMA = 'singularitydog.four-bus-native-group-build.v1'
SCOPE = 'four_bus_native_group.v1'
SOURCE_NAMES = ('ordinary_transport.cpp', 'subset_stop.cpp', 'subset_active.cpp', 'transport.cpp')
SOURCE_KEYS = ('ordinary', 'subset_stop', 'subset_active', 'native_group')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(output):
    sources = tuple(p.resolve(strict=True) for p in (ORDINARY, SUBSET_STOP, SUBSET_ACTIVE, EXTENSION))
    originals = tuple(p.read_bytes() for p in sources)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, raw in zip(SOURCE_NAMES, originals):
        (output/name).write_bytes(raw)
    library = output/LIBRARY_NAME
    command = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++17', '-O2', '-Wall',
               '-Wextra', '-Werror', '-pthread', '-fPIC', '-shared',
               '-DFOUR_BUS_ORDINARY_SOURCE="ordinary_transport.cpp"',
               '-DFOUR_BUS_SUBSET_STOP_SOURCE="subset_stop.cpp"',
               '-DFOUR_BUS_ACTIVE_SOURCE="subset_active.cpp"',
               str(output/'transport.cpp'), '-o', str(library)]
    subprocess.run(command, check=True)
    if tuple(p.read_bytes() for p in sources) != originals or tuple(
            (output/n).read_bytes() for n in SOURCE_NAMES) != originals:
        raise ValueError('Composite group source changed during build')
    # These query-only symbols do not instantiate workers, Sessions or devices.
    lib = C.CDLL(str(library.resolve()))
    abi = {}
    for name in ('sda_abi', 'sda_subset_active_abi', 'sda_emergency_stop_subset_abi',
                 'sda_subset_exchange_at_abi', 'sda_subset_write_deadline_abi', 'sdg_abi',
                 'sdg_result_size', 'sdg_port_size'):
        function = lib[name]; function.argtypes = []; function.restype = C.c_uint32
        abi[name] = function()
    if any(value != 1 for name, value in abi.items() if name.endswith('_abi')):
        raise ValueError('Composite native group ABI mismatch')
    sha = dict(zip(SOURCE_KEYS, (digest(output/n) for n in SOURCE_NAMES)))
    sizes = dict(zip(SOURCE_KEYS, (len(raw) for raw in originals)))
    old_scope = {'schema': active_build.SCHEMA, 'scope': active_build.SCOPE,
        'type1_exchange_abi': 1, 'subset_stop_abi': 1, 'original_active_abi': 1,
        'exchange_at_abi': 1, 'write_deadline_abi': 1,
        'ordinary_source_sha256': sha['ordinary'], 'subset_stop_source_sha256': sha['subset_stop'],
        'extension_source_sha256': sha['native_group'],
        'ordinary_source_bytes': sizes['ordinary'], 'subset_stop_source_bytes': sizes['subset_stop'],
        'extension_source_bytes': sizes['native_group'],
        'sources': {'ordinary': str(sources[0]), 'subset_stop': str(sources[1]),
                    'extension': str(sources[3])},
        'library': LIBRARY_NAME, 'allowed_masks': active_build.ALLOWED_MASKS,
        'allowed_kinds': active_build.ALLOWED_KINDS, 'type1_batch_sizes': [1, 3],
        'output_allowed': False, 'timing_admission_eligible': False}
    record = {'abi': 1, 'source_sha256': sha['native_group'], 'binary_sha256': digest(library),
        'command': command, 'platform': platform.platform(), 'machine': platform.machine(),
        'four_bus_subset_active': old_scope,
        'four_bus_native_group': {'schema': SCHEMA, 'scope': SCOPE, 'abi': 1,
            'included_source_sha256': sha, 'included_source_bytes': sizes,
            'source_files': dict(zip(SOURCE_KEYS, SOURCE_NAMES)), 'queried_abi': abi,
            'library': LIBRARY_NAME, 'gap_ns': 900_000, 'window': 3,
            'session_count': 4, 'exact_motor_ids': list(range(1, 13)),
            'port_order': 'caller-bound-independent-of-enumeration',
            'hold_then_voltage': True, 'separate_output_write_deadline': True,
            'same_library_session_handles_only': True, 'unchanged_wire_kernel': True,
            'close_before_existing_emergency_stop': True,
            'output_allowed': False, 'timing_admission_eligible': False}}
    (output/'build-record.json').write_text(json.dumps(record, indent=2)+'\n')
    problem = receipt_problem(output)
    if problem:
        raise ValueError(problem)
    return library


def receipt_problem(directory):
    directory = Path(directory)
    problem = active_build.receipt_problem(directory)
    if problem:
        return problem
    try:
        record = json.loads((directory/'build-record.json').read_bytes())
        scope = record['four_bus_native_group']
        expected_files = dict(zip(SOURCE_KEYS, SOURCE_NAMES))
        if (type(scope) is not dict or scope.get('schema') != SCHEMA or scope.get('scope') != SCOPE or
                scope.get('abi') != 1 or scope.get('library') != LIBRARY_NAME or
                scope.get('source_files') != expected_files or scope.get('gap_ns') != 900_000 or
                scope.get('window') != 3 or scope.get('session_count') != 4 or
                scope.get('exact_motor_ids') != list(range(1, 13)) or
                scope.get('port_order') != 'caller-bound-independent-of-enumeration' or
                any(scope.get(name) is not True for name in (
                    'hold_then_voltage', 'separate_output_write_deadline',
                    'same_library_session_handles_only', 'unchanged_wire_kernel',
                    'close_before_existing_emergency_stop')) or
                scope.get('output_allowed') is not False or scope.get('timing_admission_eligible') is not False):
            return 'Native group receipt scope/configuration differs'
        for key, name in expected_files.items():
            raw = (directory/name).read_bytes()
            if (hashlib.sha256(raw).hexdigest() != scope['included_source_sha256'][key] or
                    len(raw) != scope['included_source_bytes'][key]):
                return 'Native group included source differs from authenticated receipt'
        if record['source_sha256'] != scope['included_source_sha256']['native_group']:
            return 'Native group compiled extension pin differs'
        queries = scope.get('queried_abi')
        if (type(queries) is not dict or set(queries) != {
                'sda_abi', 'sda_subset_active_abi', 'sda_emergency_stop_subset_abi',
                'sda_subset_exchange_at_abi', 'sda_subset_write_deadline_abi',
                'sdg_abi', 'sdg_result_size', 'sdg_port_size'} or
                any(type(value) is not int or (value != 1 if name.endswith('_abi') else value <= 0)
                    for name, value in queries.items())):
            return 'Native group ABI receipt differs'
    except (KeyError, OSError, TypeError, ValueError):
        return 'Native group receipt/source outputs incomplete'
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args(argv)
    if not args.build:
        print(json.dumps({'status': 'PLAN', 'opens_devices': False, 'builds_library': False,
            'output_allowed': False, 'scope': SCOPE,
            'included_source_sha256': dict(zip(SOURCE_KEYS, (digest(p) for p in
                (ORDINARY, SUBSET_STOP, SUBSET_ACTIVE, EXTENSION))))}))
        return 0
    print(build(args.output));return 0


if __name__ == '__main__':
    raise SystemExit(main())
