"""Fresh, explicit file-only build of the optional three-axis STOP extension."""
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


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(output, *, ordinary_source=ORDINARY):
    """No shared output, installation, device open or network operation."""
    output = Path(output)
    ordinary_source = Path(ordinary_source).resolve(strict=True)
    original = ordinary_source.read_bytes()
    extension = (HERE / 'subset_stop.cpp').read_bytes()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'ordinary_transport.cpp').write_bytes(original)
    (output / 'transport.cpp').write_bytes(extension)
    library = output / 'libdog_four_bus_transport.so'
    command = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++17', '-O2',
               '-Wall', '-Wextra', '-Werror', '-pthread', '-fPIC', '-shared',
               '-DFOUR_BUS_ORDINARY_SOURCE="ordinary_transport.cpp"',
               str(output / 'transport.cpp'), '-o', str(library)]
    subprocess.run(command, check=True)
    if ordinary_source.read_bytes() != original or (HERE / 'subset_stop.cpp').read_bytes() != extension:
        raise ValueError('Build input changed during compilation')
    record = {'abi': 1, 'command': command, 'platform': platform.platform(),
              'machine': platform.machine(), 'source_sha256': digest(output / 'transport.cpp'),
              'binary_sha256': digest(library),
              'four_bus_subset_stop': {'schema': 'singularitydog.four-bus-subset-build.v1',
                  'abi': 1, 'ordinary_source_sha256': digest(output / 'ordinary_transport.cpp'),
                  'extension_source_sha256': digest(output / 'transport.cpp'),
                  'ordinary_source_bytes': len(original), 'extension_source_bytes': len(extension),
                  'allowed_masks': [7, 56], 'original_active_abi': 1,
                  'output_allowed': False, 'timing_admission_eligible': False}}
    (output / 'build-record.json').write_text(json.dumps(record, indent=2) + '\n')
    return library


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args(argv)
    if not args.build:
        print(json.dumps({'status': 'PLAN', 'opens_devices': False,
                          'builds_library': False, 'output_allowed': False,
                          'ordinary_source_sha256': digest(ORDINARY),
                          'extension_source_sha256': digest(HERE / 'subset_stop.cpp')}))
        return 0
    print(build(args.output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
