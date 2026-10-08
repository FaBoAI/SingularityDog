"""File-only build of the CPU checked-dispatch operator. Default PLAN."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).absolute().parent

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--source-sha256',required=True)
    p.add_argument('--execute',action='store_true')
    a=p.parse_args(argv);source=HERE/'checked_dispatch.cpp'
    if not source.is_file() or any(x.is_symlink() for x in (source,*source.parents)):
        raise ValueError('Regular nonsymlink source required')
    raw=source.read_bytes();digest=hashlib.sha256(raw).hexdigest()
    if digest!=a.source_sha256:raise ValueError('Explicit source SHA256 mismatch')
    plan=dict(schema='experimental.live-checked-dispatch-build.v1',status='PLAN_ONLY',source_path=str(source),
              source_sha256=digest,hardware_opened=False,output_allowed=False,approved_for_runtime=False,
              flags=['-O3','-std=c++20','-ffp-contract=off','-fno-fast-math'])
    if not a.execute:
        print(json.dumps(plan,indent=2));return 0
    out=a.output.absolute()
    if out.exists() or not out.parent.is_dir() or any(x.is_symlink() for x in (out,*out.parents)):
        raise ValueError('Fresh build output required')
    out.mkdir(mode=0o700)
    import platform
    import torch
    from torch.utils.cpp_extension import include_paths,library_paths
    lib=out/'live_checked_dispatch.so'
    paths=library_paths();command=['c++','-shared','-fPIC','-O3','-std=c++20',
        '-ffp-contract=off','-fno-fast-math','-D_GLIBCXX_USE_CXX11_ABI='+str(int(torch._C._GLIBCXX_USE_CXX11_ABI))]
    command += ['-I'+x for x in include_paths()]
    command += [str(source),'-o',str(lib),*['-L'+x for x in paths],
                '-ltorch','-ltorch_cpu','-lc10',*['-Wl,-rpath,'+x for x in paths]]
    result=subprocess.run(command,capture_output=True,text=True)
    (out/'build.stdout').write_text(result.stdout);(out/'build.stderr').write_text(result.stderr)
    if result.returncode:raise RuntimeError('Build failed; original compiler stdout/stderr preserved')
    if hashlib.sha256(source.read_bytes()).hexdigest()!=digest:raise ValueError('Source changed while building')
    record={**plan,'status':'CPU_LIBRARY_BUILT_NO_RUNTIME_QUALIFICATION','command':command,
        'library_path':str(lib),'library_sha256':hashlib.sha256(lib.read_bytes()).hexdigest(),
        'torch_version':torch.__version__,'python_version':platform.python_version(),
        'machine':platform.machine(),'system':platform.system(),
        'cxx11_abi':bool(torch._C._GLIBCXX_USE_CXX11_ABI),'compiler_returncode':result.returncode}
    (out/'build-record.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
