"""Build CPython validation helper using the stdlib and a C++17 compiler."""
from pathlib import Path
import hashlib
import json
import os
import shlex
import subprocess
import sys
import sysconfig

root=Path(__file__).resolve().parent
compiler=shlex.split(os.environ.get('CXX') or sysconfig.get_config_var('CXX') or 'c++')
link=['-bundle','-undefined','dynamic_lookup'] if sys.platform=='darwin' else ['-shared']
source=root/'native_input_validation.cpp'
binary=root/('_native_input_validation'+sysconfig.get_config_var('EXT_SUFFIX'))
command=compiler+['-std=c++17','-O3','-fPIC','-Wall','-Wextra','-Werror','-ffp-contract=off']+link
command+=['-I'+p for p in sorted({sysconfig.get_path('include'),sysconfig.get_path('platinclude')}) if p]
command += [str(source),'-o',str(binary)]
print(shlex.join(command),flush=True)
subprocess.run(command,check=True)
record={'python':sys.version,'command':command,
        'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'library_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
(root/'build-record.json').write_text(json.dumps(record,indent=2)+'\n')
