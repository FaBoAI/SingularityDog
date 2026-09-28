"""Explicit offline build; no installation, device access, or network."""
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent

def build():
    source, output = ROOT/'transport.cpp', ROOT/'libdog_active_transport.so'
    command = [*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++17', '-O2',
               '-Wall', '-Wextra', '-Werror', '-pthread', '-fPIC', '-shared', str(source), '-o', str(output)]
    subprocess.run(command, check=True)
    report = {'abi': 1, 'command': command, 'platform': platform.platform(),
              'machine': platform.machine(), 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'binary_sha256': hashlib.sha256(output.read_bytes()).hexdigest()}
    (ROOT/'build-record.json').write_text(json.dumps(report, indent=2)+'\n')
    return output

if __name__ == '__main__':
    print(build())
