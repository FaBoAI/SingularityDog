"""Fresh file-only C++ current-guard build; default PLAN creates nothing."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess

SOURCE = Path(__file__).resolve().with_name('native_current_guard.cpp')
SCHEMA = 'singularitydog.four-bus-current-guard-build.v1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(output):
    raw = SOURCE.read_bytes()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    source = output / SOURCE.name
    source.write_bytes(raw)
    library = output / 'libdog_current_guard.so'
    command = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++17', '-O2',
        '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared', str(source), '-o', str(library)]
    subprocess.run(command, check=True)
    if SOURCE.read_bytes() != raw or source.read_bytes() != raw:
        raise ValueError('Current guard source changed during build')
    record = {'schema': SCHEMA, 'abi': 1, 'source_sha256': digest(source),
        'source_bytes': len(raw),
        'binary_sha256': digest(library), 'command': command,
        'platform': platform.platform(), 'machine': platform.machine(),
        'CAN_IO_available': False, 'output_allowed': False, 'timing_admission_eligible': False}
    (output / 'build-record.json').write_text(json.dumps(record, indent=2)+'\n')
    return library


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--build', action='store_true')
    args = p.parse_args(argv)
    if not args.build:
        print(json.dumps({'status': 'PLAN', 'opens_devices': False, 'builds_library': False,
            'source_sha256': digest(SOURCE), 'output_allowed': False}))
        return 0
    print(build(args.output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
