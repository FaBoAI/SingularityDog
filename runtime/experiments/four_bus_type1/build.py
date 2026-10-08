"""Fresh, explicit file-only build of the mask-bound four-bus Type1 extension.

A library or receipt is never an output approval (`output_allowed` false).
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess

HERE = Path(__file__).resolve().parent
ORDINARY = HERE.parent / 'native_active_transport' / 'transport.cpp'
SUBSET_STOP = HERE.parent / 'four_bus_diagnostic' / 'subset_stop.cpp'
EXTENSION = HERE / 'subset_active.cpp'
LIBRARY_NAME = 'libdog_four_bus_type1_transport.so'
SCOPE = 'four_bus_subset_active.v1'
SCHEMA = 'singularitydog.four-bus-subset-active-build.v1'
ALLOWED_MASKS = [7, 56]
ALLOWED_KINDS = [0, 1, 3, 4, 17, 18]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(output, *, ordinary_source=ORDINARY, subset_stop_source=SUBSET_STOP):
    """No shared output, installation, device open or network operation."""
    output = Path(output)
    sources = (Path(ordinary_source).resolve(strict=True),
               Path(subset_stop_source).resolve(strict=True), EXTENSION.resolve(strict=True))
    original, stop, extension = (path.read_bytes() for path in sources)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'ordinary_transport.cpp').write_bytes(original)
    (output / 'subset_stop.cpp').write_bytes(stop)
    (output / 'transport.cpp').write_bytes(extension)
    library = output / LIBRARY_NAME
    command = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++17', '-O2',
               '-Wall', '-Wextra', '-Werror', '-pthread', '-fPIC', '-shared',
               '-DFOUR_BUS_ORDINARY_SOURCE="ordinary_transport.cpp"',
               '-DFOUR_BUS_SUBSET_STOP_SOURCE="subset_stop.cpp"',
               str(output / 'transport.cpp'), '-o', str(library)]
    subprocess.run(command, check=True)
    if tuple(path.read_bytes() for path in sources) != (original, stop, extension):
        raise ValueError('Build input changed during compilation')
    copies = ('ordinary_transport.cpp', 'subset_stop.cpp', 'transport.cpp')
    if tuple((output / name).read_bytes() for name in copies) != (original, stop, extension):
        raise ValueError('Build output source copy changed during compilation')
    record = {'abi': 1, 'command': command, 'platform': platform.platform(),
              'machine': platform.machine(), 'source_sha256': digest(output / 'transport.cpp'),
              'binary_sha256': digest(library),
              'four_bus_subset_active': {'schema': SCHEMA, 'scope': SCOPE,
                  'type1_exchange_abi': 1, 'subset_stop_abi': 1, 'original_active_abi': 1,
                  'ordinary_source_sha256': digest(output / 'ordinary_transport.cpp'),
                  'subset_stop_source_sha256': digest(output / 'subset_stop.cpp'),
                  'extension_source_sha256': digest(output / 'transport.cpp'),
                  'ordinary_source_bytes': len(original), 'subset_stop_source_bytes': len(stop),
                  'extension_source_bytes': len(extension),
                  'sources': {'ordinary': str(sources[0]), 'subset_stop': str(sources[1]),
                              'extension': str(sources[2])},
                  'library': LIBRARY_NAME, 'allowed_masks': ALLOWED_MASKS,
                  'allowed_kinds': ALLOWED_KINDS, 'type1_batch_sizes': [1, 3],
                  'output_allowed': False, 'timing_admission_eligible': False}}
    (output / 'build-record.json').write_text(json.dumps(record, indent=2) + '\n')
    return library


def receipt_problem(directory):
    """None only for an exact subset-active receipt whose source copies still match.

    A STOP-only receipt (`four_bus_subset_stop`) is never accepted here, and
    this receipt carries no `four_bus_subset_stop` scope for the STOP loader.
    """
    directory = Path(directory)
    try:
        record = json.loads((directory / 'build-record.json').read_bytes())
    except (OSError, ValueError):
        return 'Unreadable four-bus Type1 build receipt'
    scope = record.get('four_bus_subset_active') if type(record) is dict else None
    if type(scope) is not dict or 'four_bus_subset_stop' in record:
        return 'Four-bus subset-active scope required; STOP-only receipt rejected'
    copies = (('ordinary_transport.cpp', 'ordinary'), ('subset_stop.cpp', 'subset_stop'),
              ('transport.cpp', 'extension'))
    try:
        for name, key in copies:
            raw = (directory / name).read_bytes()
            if (hashlib.sha256(raw).hexdigest() != scope.get(f'{key}_source_sha256') or
                    len(raw) != scope.get(f'{key}_source_bytes')):
                return 'Four-bus Type1 included source differs from its receipt'
        binary = digest(directory / LIBRARY_NAME)
    except OSError:
        return 'Four-bus Type1 build output incomplete'
    if (record.get('abi') != 1 or record.get('source_sha256') != scope.get('extension_source_sha256') or
            record.get('binary_sha256') != binary or scope.get('schema') != SCHEMA or
            scope.get('scope') != SCOPE or scope.get('type1_exchange_abi') != 1 or
            scope.get('subset_stop_abi') != 1 or scope.get('original_active_abi') != 1 or
            scope.get('allowed_masks') != ALLOWED_MASKS or scope.get('allowed_kinds') != ALLOWED_KINDS or
            scope.get('type1_batch_sizes') != [1, 3] or scope.get('library') != LIBRARY_NAME or
            scope.get('output_allowed') is not False or scope.get('timing_admission_eligible') is not False):
        return 'Four-bus Type1 receipt pins differ'
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args(argv)
    if not args.build:
        print(json.dumps({'status': 'PLAN', 'opens_devices': False,
                          'builds_library': False, 'output_allowed': False, 'scope': SCOPE,
                          'ordinary_source_sha256': digest(ORDINARY),
                          'subset_stop_source_sha256': digest(SUBSET_STOP),
                          'extension_source_sha256': digest(EXTENSION)}))
        return 0
    print(build(args.output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
