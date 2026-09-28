"""Build portable diagnostic C++ transport; no Python development headers needed."""
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent

def build():
    source, output = ROOT / 'transport.cpp', ROOT / 'libdog_transport.so'
    command = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++17', '-O2',
               '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared', str(source), '-o', str(output)]
    subprocess.run(command, check=True)
    report = {'command': command, 'platform': platform.platform(),
              'machine': platform.machine(), 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'binary_sha256': hashlib.sha256(output.read_bytes()).hexdigest()}
    (ROOT / 'build-record.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return output

if __name__ == '__main__':
    build()
