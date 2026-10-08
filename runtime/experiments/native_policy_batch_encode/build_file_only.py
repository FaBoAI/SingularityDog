#!/usr/bin/env python3
"""Compile only the pinned CPython batch encoder; never grant output approval."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import platform
import re
import subprocess
import sys
import sysconfig

PINNED_SOURCE_SHA256 = {
    'batch_encode.cpp': '73e3e28bf80800c8ea6a2b12320f783c247b706f1964b46122c3e9a5b78238ea',
    'batch_encode_py.cpp': '7210ee9cce727e5b31cac697893753d0355813fc17e5d9d34a709883c7fa3908',
}


def _macro_number(macros, name):
    value = macros.get(name)
    if value is None:
        raise ValueError('Python header probe omitted ' + name)
    value = value.strip().strip('()')
    if re.fullmatch(r'[0-9]+[uUlL]*', value) is None:
        raise ValueError('Noninteger Python header macro: ' + name)
    return int(value.rstrip('uUlL'))


def _header_contract(compiler, include_dirs):
    """Inspect the headers actually selected by the compiler, without a binary.

    Explicit --include is not a license to mix Python ABIs. Preprocessing also
    resolves distribution pyconfig.h indirections and architecture conditions;
    reading only the first Python.h would miss those configuration changes.
    """
    command = [compiler, '-std=c++17']
    command += ['-I' + str(Path(directory)) for directory in include_dirs]
    command += ['-E', '-dM', '-H', '-x', 'c++', '-']
    result = subprocess.run(command, input='#include <Python.h>\n', text=True,
                            capture_output=True, check=True, timeout=30)
    macros = {}
    for line in result.stdout.splitlines():
        match = re.fullmatch(r'#define ([A-Za-z_][A-Za-z_0-9]*)(?:\s+(.*))?', line)
        if match:
            macros[match.group(1)] = match.group(2) or ''
    version = tuple(_macro_number(macros, name) for name in
                    ('PY_MAJOR_VERSION', 'PY_MINOR_VERSION', 'PY_MICRO_VERSION'))
    target_version = tuple(sys.version_info[:3])
    if version[:2] != target_version[:2]:
        raise ValueError('Python development header major/minor differs from the running interpreter')
    pointer_bytes = ctypes.sizeof(ctypes.c_void_p)
    header_pointer = _macro_number(macros, 'SIZEOF_VOID_P')
    compiler_pointer = _macro_number(macros, '__SIZEOF_POINTER__')
    config_pointer = sysconfig.get_config_var('SIZEOF_VOID_P')
    if (type(config_pointer) is not int or
            not pointer_bytes == header_pointer == compiler_pointer == config_pointer):
        raise ValueError('Python header/compiler/interpreter pointer width differs')
    flags = {}
    for name in ('Py_DEBUG', 'Py_GIL_DISABLED', 'Py_TRACE_REFS'):
        # Python.h uses #ifdef for these ABI switches. Even an explicitly
        # defined zero changes the header layout/contract; absence is off.
        header = int(name in macros)
        target = sysconfig.get_config_var(name) or 0
        if header not in (0, 1) or target not in (0, 1) or header != target:
            raise ValueError('Python header/interpreter build flag differs: ' + name)
        flags[name] = header
    machine = platform.machine().lower()
    known_architectures = {
        'arm64': ('__aarch64__', '__arm64__'),
        'aarch64': ('__aarch64__', '__arm64__'),
        'x86_64': ('__x86_64__', '_M_X64'),
        'amd64': ('__x86_64__', '_M_X64'),
        'i386': ('__i386__', '_M_IX86'),
        'i686': ('__i386__', '_M_IX86'),
    }
    expected_architecture = known_architectures.get(machine)
    if expected_architecture is not None and not any(name in macros for name in expected_architecture):
        raise ValueError('Compiler target architecture differs from the running interpreter')
    headers = []
    seen = set()
    for line in result.stderr.splitlines():
        match = re.fullmatch(r'\.+\s+(.+?)(?:\s+\(framework directory\))?', line)
        if match is None:
            continue
        path = Path(match.group(1))
        if path.name not in ('Python.h', 'patchlevel.h', 'pyconfig.h'):
            continue
        path = path.resolve(strict=True)
        if not path.is_file() or path in seen:
            continue
        raw = path.read_bytes()
        seen.add(path)
        headers.append({'path': str(path), 'size_bytes': len(raw),
                        'sha256': hashlib.sha256(raw).hexdigest()})
    if {Path(row['path']).name for row in headers} != {'Python.h', 'patchlevel.h', 'pyconfig.h'}:
        raise ValueError('Compiler did not identify all Python ABI headers')
    return {'target_python': {'executable': sys.executable,
                             'version': list(target_version), 'machine': machine,
                             'extension_suffix': sysconfig.get_config_var('EXT_SUFFIX'),
                             'soabi': sysconfig.get_config_var('SOABI'),
                             'pointer_bytes': pointer_bytes, 'build_flags': flags},
            'header_probe': {'compiler_command': command, 'version': list(version),
                             'pointer_bytes': header_pointer, 'headers': headers,
                             'compiler_pointer_bytes': compiler_pointer,
                             'compiler_architecture_macros': sorted({name for names in known_architectures.values()
                                                                    for name in names if name in macros}),
                             'matching_major_minor_required': True,
                             'matching_patch_required': False,
                             'hardware_opened': False}}


def build(output, *, compiler='c++', includes=()):
    source_dir = Path(__file__).resolve().parent
    for name, digest in PINNED_SOURCE_SHA256.items():
        source = source_dir / name
        if not source.is_file() or source.is_symlink() or hashlib.sha256(source.read_bytes()).hexdigest() != digest:
            raise ValueError('Pinned batch encoder source differs: ' + name)
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    if type(suffix) is not str or not suffix:
        raise ValueError('Running Python extension suffix is unavailable')
    output = Path(output).absolute()
    if output.name != 'sdbe_native' + suffix or output.exists() or output.is_symlink() or not output.parent.is_dir():
        raise ValueError('Output must be a new sdbe_native Python extension in an existing directory')
    include_dirs = includes or (sysconfig.get_path('include'),)
    if not all((Path(directory) / 'Python.h').is_file() for directory in include_dirs[:1]):
        raise ValueError('Python development headers missing')
    contract = _header_contract(compiler, include_dirs)
    command = [compiler, '-std=c++17', '-O2', '-ffp-contract=off']
    if sys.platform == 'darwin':
        command += ['-dynamiclib', '-undefined', 'dynamic_lookup']
    else:
        command += ['-fPIC', '-shared']
    command += ['-I' + str(Path(directory)) for directory in include_dirs]
    command += ['-o', str(output), str(source_dir / 'batch_encode_py.cpp')]
    subprocess.run(command, check=True)
    for name, digest in PINNED_SOURCE_SHA256.items():
        source = source_dir / name
        if source.is_symlink() or not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != digest:
            raise ValueError('Pinned batch encoder source changed during build: ' + name)
    for row in contract['header_probe']['headers']:
        if hashlib.sha256(Path(row['path']).read_bytes()).hexdigest() != row['sha256']:
            raise ValueError('Python ABI header changed during build: ' + Path(row['path']).name)
    return {'schema': 'singularitydog.file-only-batch-encoder-build.v1',
            'status': 'BUILT_UNAPPROVED_CANDIDATE', 'hardware_opened': False,
            'motor_commands_sent': False, 'output_approved': False,
            'compiler_command': command,
            **contract,
            'source_and_header_pins_unchanged_after_build': True,
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
