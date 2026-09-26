"""Build the private CPython extension using stdlib + a C++17 compiler only."""
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import sysconfig

BASE = Path(__file__).resolve().parent
source = BASE / "native_boot_guard.cpp"
suffix = sysconfig.get_config_var("EXT_SUFFIX")
if not suffix:
    raise RuntimeError("CPython extension suffix unavailable")
compiler = shlex.split(os.environ.get("CXX") or sysconfig.get_config_var("CXX") or "g++")
includes = sorted({sysconfig.get_path("include"), sysconfig.get_path("platinclude")})
output = BASE / ("_native_boot_guard" + suffix)
link = ["-bundle", "-undefined", "dynamic_lookup"] if sys.platform == "darwin" else ["-shared"]
command = compiler + ["-std=c++17", "-O3", "-DNDEBUG", "-fPIC", "-Wall", "-Wextra"] + link
command += ["-I" + path for path in includes if path] + [str(source), "-o", str(output)]
subprocess.run(command, cwd=BASE, check=True)
record = dict(status="BUILT", python=sys.version, platform=platform.platform(), command=command,
    compiler_version=subprocess.run(compiler + ["--version"], text=True, capture_output=True, check=True).stdout,
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    library=str(output), library_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
(BASE / "build-record.json").write_text(json.dumps(record, indent=2) + "\n")
print(json.dumps(record, indent=2))
