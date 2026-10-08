"""Build a fresh, explicit file-only operator; never load it or open hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SOURCE_HASHES = {
    'target_candidate.cpp': '467c483d71fa3cd3146b4007cc97cbacd5872d0363810f0a3b173e56d6837cec',
    'target_baseline.cpp': '61a7b873f049ab206a839ea94e62fabed979c4b9b2823511e466508b37b4c426',
    'envelope_helpers.cpp.inc': 'a9464e94e9dc8c7418ea0f4cca70a8e5e7722e441309aa8fc3e7ca690c08e269',
}


def source_checks():
    """Authenticate the fixed numerical candidate before importing Torch."""
    for name, digest in SOURCE_HASHES.items():
        path = HERE / name
        if path.is_symlink() or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('Pinned experimental source differs: ' + name)
    path = HERE / 'source-receipt.json'
    if path.is_symlink() or not path.is_file():
        raise ValueError('Regular source receipt required')
    raw = path.read_bytes()
    receipt = json.loads(raw)
    if (receipt.get('schema') != 'singularitydog.target-envelope-scalar-source.v1'
            or receipt.get('candidate_sha256') != SOURCE_HASHES['target_candidate.cpp']
            or receipt.get('baseline_sha256') != SOURCE_HASHES['target_baseline.cpp']
            or receipt.get('helpers_sha256') != SOURCE_HASHES['envelope_helpers.cpp.inc']
            or receipt.get('new_namespace') != 'sd_target_envelope_scalar_fileonly_r11'
            or any(receipt.get(k) is not False for k in
                   ('hardware_opened', 'output_allowed', 'approved_for_runtime', 'selected_live_artifacts_changed'))):
        raise ValueError('Explicit file-only source contract differs')
    return hashlib.sha256(raw).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    if sys.flags.optimize:
        raise ValueError('Checks may not be suppressed')
    receipt_hash = source_checks()
    directory = Path(args.output_dir).absolute()
    if (directory.exists() or not directory.parent.is_dir()
            or any(p.is_symlink() for p in (directory, *directory.parents))):
        raise ValueError('Fresh nonsymlink output required')
    directory.mkdir(mode=0o700)
    import torch
    from torch.utils.cpp_extension import include_paths, library_paths
    source = HERE / 'target_candidate.cpp'
    library = directory / 'target_envelope_r11.so'
    command = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++20', '-O3',
               '-ffp-contract=off', '-fno-fast-math', '-shared', '-fPIC',
               '-D_GLIBCXX_USE_CXX11_ABI=' + str(int(torch._C._GLIBCXX_USE_CXX11_ABI)),
               str(source), '-o', str(library)]
    for item in include_paths():
        command += ['-I', item]
    for item in library_paths():
        command += ['-L', item, '-Wl,-rpath,' + item]
    command += ['-ltorch_cpu', '-lc10']
    before = time.perf_counter_ns()
    result = subprocess.run(command, capture_output=True, text=True, timeout=180)
    elapsed = time.perf_counter_ns() - before
    (directory / 'compiler.stdout').write_text(result.stdout)
    (directory / 'compiler.stderr').write_text(result.stderr)
    if result.returncode:
        raise RuntimeError('Compile failed; original compiler output retained')
    if source_checks() != receipt_hash:
        raise ValueError('Source receipt changed during build')
    record = dict(schema='singularitydog.target-envelope-scalar-build.v1',
                  source_sha256=SOURCE_HASHES['target_candidate.cpp'],
                  numerical_source_sha256=SOURCE_HASHES,
                  source_receipt_sha256=receipt_hash,
                  library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                  command=command, compile_ns=elapsed, torch_version=torch.__version__,
                  python_version=sys.version, system=platform.system(), machine=platform.machine(),
                  cxx11_abi=bool(torch._C._GLIBCXX_USE_CXX11_ABI), hardware_opened=False,
                  output_allowed=False, approved_for_runtime=False, namespace_loaded=False)
    (directory / 'build-record.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record))


if __name__ == '__main__':
    main()
