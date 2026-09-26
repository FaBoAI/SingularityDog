"""Build locally with stdlib and a system compiler, no setuptools/network."""
import argparse
from pathlib import Path
import shlex
import subprocess
import sys
import sysconfig

parser = argparse.ArgumentParser()
parser.add_argument('--sanitize', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parent
if sys.implementation.name != 'cpython':
    raise SystemExit('This extension requires CPython')
command = shlex.split(sysconfig.get_config_var('LDSHARED'))
command += ['-std=c11', '-O3', '-fPIC', '-Wall', '-Wextra', '-Werror']
command += ['-I' + sysconfig.get_path('include')]
if args.sanitize:
    command += ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
command += [str(root / '_event_snapshot_native.c'), '-o',
            str(root / ('_event_snapshot_native' + sysconfig.get_config_var('EXT_SUFFIX')))]
print(shlex.join(command), flush=True)
subprocess.run(command, check=True)
