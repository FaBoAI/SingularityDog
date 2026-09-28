"""Explicit target-local build of the file-only fixed-record parser."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time

HERE=Path(__file__).resolve().parent


def sha(raw):return hashlib.sha256(raw).hexdigest()


def build(output,compiler='c++'):
    output=Path(output).expanduser().absolute()
    if platform.system() not in ('Darwin','Linux') or not output.parent.is_dir() or output.exists():
        raise ValueError('Require Darwin/Linux and a new output in an existing directory')
    if any((parent/'.git').exists() for parent in (output.parent,*output.parents)):
        raise ValueError('Native parser binary must remain outside Git')
    source=HERE/'snapshot.cpp'
    command=[compiler,'-std=c++20','-O3','-ffp-contract=off','-shared','-fPIC',
             str(source),'-o',str(output)]
    start=time.perf_counter_ns()
    result=subprocess.run(command,capture_output=True,text=True,timeout=30)
    if result.returncode:
        raise RuntimeError('Native parser compile failed: '+result.stderr[-4000:])
    return {'schema':'native-snapshot-fastpath-build-v1','source_sha256':sha(source.read_bytes()),
            'library_sha256':sha(output.read_bytes()),
            'elapsed_ms':(time.perf_counter_ns()-start)/1e6,
            'platform':platform.system()+'-'+platform.machine(),
            'hardware_opened':False,'output_allowed':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--compiler',default='c++')
    print(json.dumps(build(**vars(parser.parse_args())),indent=2))


if __name__=='__main__':main()
