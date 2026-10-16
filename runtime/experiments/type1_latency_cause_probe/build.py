"""Build/load only the independent CPU-only return probe; no transport capability."""
import argparse
import ctypes as C
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

SCHEMA='singularitydog.type1-latency-cpu-probe-build.v1'
SOURCE=Path(__file__).with_name('gil_reacquire.cpp')
THREAD_CPU,THREAD_RUSAGE,SCHEDSTAT,LINUX_PLACEMENT=1,2,4,8


class Snapshot(C.Structure):
    _fields_=[(name,C.c_uint64) for name in ('sample_started_ns','sample_finished_ns','thread_cpu_ns',
        'user_cpu_ns','system_cpu_ns','minflt','majflt','nvcsw','nivcsw','sched_cpu_ns',
        'sched_run_delay_ns','sched_timeslices','native_tid')]+[
        ('cpu',C.c_int32),('policy',C.c_int32),('valid_bits',C.c_uint32),('reserved',C.c_uint32)]


class Record(C.Structure):
    _fields_=[('abi',C.c_uint32),('status',C.c_int32)]+[(name,C.c_uint64) for name in
        ('call_started_ns','wait_requested_ns','spin_requested_ns','wait_started_ns',
         'wait_finished_ns','native_ready_to_return_ns')]+[('before',Snapshot),('after',Snapshot)]


def digest(raw):return hashlib.sha256(raw).hexdigest()


def build(directory, *, cxx=None):
    target=Path(directory)
    if not target.is_absolute() or target.exists():
        raise ValueError('Fresh absolute CPU-only build directory required')
    target.mkdir(mode=0o700,parents=True)
    source=SOURCE.read_bytes();(target/SOURCE.name).write_bytes(source)
    compiler=cxx or shutil.which('c++')
    if not compiler:raise ValueError('C++ compiler unavailable')
    extension='.dylib' if platform.system()=='Darwin' else '.so'
    binary=target/('libgil_reacquire_probe'+extension)
    command=[str(compiler),'-std=c++17','-O2','-fPIC','-pthread',
        '-dynamiclib' if extension=='.dylib' else '-shared',str(target/SOURCE.name),'-o',str(binary)]
    completed=subprocess.run(command,capture_output=True,text=True,check=False)
    (target/'compiler-stdout.txt').write_text(completed.stdout)
    (target/'compiler-stderr.txt').write_text(completed.stderr)
    if completed.returncode:raise RuntimeError('CPU-only probe compiler failed: '+completed.stderr[-2000:])
    record={'schema':SCHEMA,'abi':1,'source_name':SOURCE.name,'source_sha256':digest(source),
        'source_bytes':len(source),'binary_name':binary.name,'binary_sha256':digest(binary.read_bytes()),
        'binary_bytes':binary.stat().st_size,'compiler_command':command,'platform':platform.system(),
        'machine':platform.machine(),'snapshot_size':C.sizeof(Snapshot),'record_size':C.sizeof(Record),
        'counter_scope':'calling_native_thread_only','counter_sampling_windows_explicit':True,
        'native_wall_clock':'mach_absolute_time' if platform.system()=='Darwin' else 'CLOCK_MONOTONIC',
        'python_monotonic_domain_check_required':True,'actual_transport_probe':False,
        'CAN_IO_available':False,'serial_IO_available':False,'socket_IO_available':False,
        'foreign_function_pointer_forwarding':False,'motor_frames_created':False,
        'output_allowed':False,'runtime_qualified':False,'original_type1_source_modified':False}
    (target/'build-record.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def receipt(directory):
    target=Path(directory)
    record=json.loads((target/'build-record.json').read_text())
    if record.get('schema')!=SCHEMA or record.get('abi')!=1 or record.get('snapshot_size')!=120 or record.get('record_size')!=296:
        raise ValueError('CPU-only probe build/ABI receipt differs')
    for name in ('CAN_IO_available','serial_IO_available','socket_IO_available','foreign_function_pointer_forwarding',
                 'motor_frames_created','output_allowed','runtime_qualified','original_type1_source_modified'):
        if record.get(name) is not False:raise ValueError('CPU-only scope differs: '+name)
    for prefix in ('source','binary'):
        path=target/record[prefix+'_name']
        if path.parent!=target or path.is_symlink() or not path.is_file():
            raise ValueError('CPU-only artifact path is not its original regular file')
        raw=path.read_bytes()
        if digest(raw)!=record[prefix+'_sha256'] or len(raw)!=record[prefix+'_bytes']:
            raise ValueError('CPU-only artifact changed: '+prefix)
    return record


def snapshot_to_dict(value):
    result={name:int(getattr(value,name)) for name,_ in Snapshot._fields_}
    bits=result['valid_bits']
    groups={THREAD_CPU:('thread_cpu_ns',),THREAD_RUSAGE:('user_cpu_ns','system_cpu_ns','minflt','majflt','nvcsw','nivcsw'),
        SCHEDSTAT:('sched_cpu_ns','sched_run_delay_ns','sched_timeslices'),LINUX_PLACEMENT:('cpu','policy')}
    for bit,names in groups.items():
        if not bits&bit:
            for name in names:result[name]=None
    result['counter_scope']='calling_native_thread'
    result['zero_unavailable_counters_are_not_measurements']=True
    return result


def record_to_dict(value):
    return {**{name:int(getattr(value,name)) for name,_ in Record._fields_ if name not in ('before','after')},
        'before':snapshot_to_dict(value.before),'after':snapshot_to_dict(value.after)}


@dataclass(frozen=True)
class ProbeLibrary:
    cdll:object
    pydll:object
    receipt:dict
    build_record_sha256:str

    def snapshot(self):
        result=Snapshot();error=C.create_string_buffer(256)
        status=self.pydll.gp_snapshot(C.byref(result),error,len(error))
        if status:raise RuntimeError(error.value.decode())
        return result


def load_library(directory, *, expected_binary_sha256=None, expected_build_sha256=None):
    target=Path(directory);record=receipt(target)
    build_sha=digest((target/'build-record.json').read_bytes())
    if expected_binary_sha256 is not None and record['binary_sha256']!=expected_binary_sha256:
        raise ValueError('Expected CPU-only binary SHA differs')
    if expected_build_sha256 is not None and build_sha!=expected_build_sha256:
        raise ValueError('Expected CPU-only build SHA differs')
    raw=(target/record['binary_name']).read_bytes()
    with tempfile.TemporaryDirectory() as tmp:
        immutable=Path(tmp)/record['binary_name'];immutable.write_bytes(raw)
        cdll,pydll=C.CDLL(str(immutable)),C.PyDLL(str(immutable))
    for library in (cdll,pydll):
        for name in ('gp_abi','gp_snapshot_size','gp_record_size','gp_entered'):
            function=getattr(library,name);function.argtypes=[];function.restype=C.c_uint32
        library.gp_reset_entered.argtypes=[];library.gp_reset_entered.restype=None
        library.gp_snapshot.argtypes=[C.POINTER(Snapshot),C.POINTER(C.c_char),C.c_uint32]
        library.gp_snapshot.restype=C.c_int
        library.gp_wait.argtypes=[C.c_uint64,C.c_uint64,C.POINTER(Record),C.POINTER(C.c_char),C.c_uint32]
        library.gp_wait.restype=C.c_int
        if library.gp_abi()!=1 or library.gp_snapshot_size()!=C.sizeof(Snapshot) or library.gp_record_size()!=C.sizeof(Record):
            raise ValueError('Actual loaded CPU-only ABI differs')
    receipt(target)
    return ProbeLibrary(cdll,pydll,record,build_sha)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--cxx')
    args=parser.parse_args(argv)
    record=build(args.output,cxx=args.cxx)
    print(json.dumps({'directory':str(args.output),'build_record':record},indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
