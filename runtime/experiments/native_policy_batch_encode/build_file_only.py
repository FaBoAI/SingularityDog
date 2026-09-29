#!/usr/bin/env python3
"""Compile only the pinned CPython batch encoder; never grant output approval."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import sysconfig

PINNED_SOURCE_SHA256 = {
    'batch_encode.cpp': '73e3e28bf80800c8ea6a2b12320f783c247b706f1964b46122c3e9a5b78238ea',
    'batch_encode_py.cpp': '7210ee9cce727e5b31cac697893753d0355813fc17e5d9d34a709883c7fa3908',
}


def build(output, *, compiler='c++', includes=()):
    source_dir = Path(__file__).resolve().parent
    for name, digest in PINNED_SOURCE_SHA256.items():
        source = source_dir / name
        if not source.is_file() or source.is_symlink() or hashlib.sha256(source.read_bytes()).hexdigest() != digest:
            raise ValueError('Pinned batch encoder source differs: ' + name)
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    output = Path(output).absolute()
    if output.name != 'sdbe_native' + suffix or output.exists() or not output.parent.is_dir():
        raise ValueError('Output must be a new sdbe_native Python extension in an existing directory')
    include_dirs = includes or (sysconfig.get_path('include'),)
    if not all((Path(directory) / 'Python.h').is_file() for directory in include_dirs[:1]):
        raise ValueError('Python development headers missing')
    command = [compiler, '-std=c++17', '-O2', '-ffp-contract=off']
    if sys.platform == 'darwin':
        command += ['-dynamiclib', '-undefined', 'dynamic_lookup']
    else:
        command += ['-fPIC', '-shared']
    command += ['-I' + str(Path(directory)) for directory in include_dirs]
    command += ['-o', str(output), str(source_dir / 'batch_encode_py.cpp')]
    subprocess.run(command, check=True)
    return {'schema': 'singularitydog.file-only-batch-encoder-build.v1',
            'status': 'BUILT_UNAPPROVED_CANDIDATE', 'hardware_opened': False,
            'motor_commands_sent': False, 'output_approved': False,
            'compiler_command': command,
            'source_sha256': PINNED_SOURCE_SHA256,
            'binary_path': str(output),
            'binary_sha256': hashlib.sha256(output.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='c++')
    parser.add_argument('--include', action='append', default=[],
                        help='Python header directory; repeat for architecture include roots')
    args = parser.parse_args()
    print(json.dumps(build(args.output, compiler=args.compiler,
                           includes=tuple(args.include)), sort_keys=True))


if __name__ == '__main__':
    main()
