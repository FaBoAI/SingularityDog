"""Local fake/pipe-only microbenchmark; never opens serial, IMU or model."""
import argparse
from pathlib import Path
import datetime
import gc
import hashlib
import importlib.util
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import time
from concurrent.futures import Future

DIRECTORY = Path(__file__).resolve().parent
RUNTIME = DIRECTORY.parents[1]
CANDIDATE = DIRECTORY/'candidate.py'
SOURCE = RUNTIME/'experiments/native_transport/transport.cpp'
CPP_SHA = '1f8aebaccf3cdbea84d1e8945cd4510b9fc27ce13ef998f3129b8ab6a80d8646'
CANDIDATE_SHA = 'ab299e1379131b4d9fa90817a4eb9303f695d39abbce25e774c896281dcec63b'


def initialize():
    global baseline, native, module, helper
    sys.path.insert(0,str(RUNTIME))
    from singularitydog_hw import native_pipeline_benchmark as baseline
    from singularitydog_hw import native_diagnostic_transport as native
    spec=importlib.util.spec_from_file_location('readiness_cause_candidate_local',CANDIDATE)
    module=importlib.util.module_from_spec(spec)
    raw=CANDIDATE.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=CANDIDATE_SHA:raise ValueError('Frozen candidate changed')
    exec(compile(raw,str(CANDIDATE),'exec'),module.__dict__)
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest()!=CPP_SHA:raise ValueError('Pinned native pipe source changed')
    helper=module.build_traced_helper(baseline)


def stats(values):
    s = sorted(values)
    return dict(count=len(s), min=s[0], median=statistics.median(s),
                p95=s[math.ceil(.95*len(s))-1], p99=s[math.ceil(.99*len(s))-1], max=s[-1])


def run_case(kind, traced, owned=None):
    futures = {key: Future() for key in ('front','rear')}
    trace = module.FixedReadinessTrace(128) if traced else None
    if kind == 'fake_already_ready':
        for f in futures.values(): f.set_result(None)
        clock = lambda: 1_000_000
        deadline = 21_000_000
        callback = None
    else:
        clock = time.monotonic_ns
        deadline = clock() + 20_000_000
        def callback(target):
            actual = owned(target)
            # This synthetic completion is on the coordinator, not a bus worker.
            for f in futures.values(): f.set_result(None)
            return actual
    begin_cpu = time.thread_time_ns()
    begin = time.perf_counter_ns()
    proof = helper(futures, None, phase='Proxy output', deadline_ns=deadline,
                   deadline_wait=callback, clock=clock, thread_clock=time.thread_time_ns,
                   check=lambda:None, trace=trace)
    end = time.perf_counter_ns()
    end_cpu = time.thread_time_ns()
    result = dict(wall_ns=end-begin, thread_cpu_ns=end_cpu-begin_cpu,
                  wait_calls=proof['wait_calls'])
    if trace:
        exported = trace.export()
        if not exported['trace_complete']:raise RuntimeError('Local trace incomplete')
        result['trace'] = exported
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--measure-local-pipe',action='store_true')
    p.add_argument('--output',type=Path)
    args=p.parse_args(argv)
    if not args.measure_local_pipe:
        print(json.dumps(dict(status='PLAN_ONLY',local_fake_and_cancel_pipe=True,
            blocks=['baseline','traced','traced','baseline'],samples_per_block=100,
            serial_or_imu_opened=False,model_loaded=False,library_loaded=False,
            hardware_opened=False,timing_admission_eligible=False,active_output_eligible=False)))
        return 0
    out=args.output
    if (out is None or not out.is_absolute() or '..' in out.parts or
        any(x.is_symlink() for x in (out,*out.parents)) or
        any((x/'.git').exists() for x in (out,*out.parents))):
        p.error('Explicit local measurement requires fresh private absolute output')
    initialize()
    out.mkdir(mode=0o700,parents=True,exist_ok=False)
    source = out/'transport.cpp'
    source.write_bytes(SOURCE.read_bytes())
    binary = out/'libdog_transport.so'
    command = ['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-fPIC','-shared',str(source),'-o',str(binary)]
    built = subprocess.run(command, capture_output=True, text=True, check=True, timeout=25)
    (out/'build-record.json').write_text(json.dumps(dict(
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()))+'\n')
    library = native.load_library(binary)
    read_fd, write_fd = os.pipe()
    before = dict(gc_enabled=gc.isenabled(), gc_threshold=list(gc.get_threshold()),
                  python_switch_interval_s=sys.getswitchinterval())
    pins={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in (Path(__file__),SOURCE,CANDIDATE,Path(baseline.__file__),Path(native.__file__))}
    rows=[]
    try:
        owned = native.make_owned_waiter(library, read_fd, spin_us=500)
        for kind in ('fake_already_ready','native_cancel_pipe_one_poll'):
            for traced in (False,True):
                for _ in range(8): run_case(kind,traced,owned)
            for block,traced in enumerate((False,True,True,False)):
                for index in range(100):
                    row=run_case(kind,traced,owned)
                    row.update(kind=kind,traced=traced,block=block,index=index)
                    rows.append(row)
    finally:
        os.close(write_fd);os.close(read_fd)
    after = dict(gc_enabled=gc.isenabled(), gc_threshold=list(gc.get_threshold()),
                 python_switch_interval_s=sys.getswitchinterval())
    if before!=after:raise RuntimeError('Local process settings changed')
    summary={}
    for kind in ('fake_already_ready','native_cancel_pipe_one_poll'):
        values={}
        for traced in (False,True):
            selected=[r for r in rows if r['kind']==kind and r['traced']==traced]
            values[str(traced)]=dict(wall_ns=stats([r['wall_ns']for r in selected]),
                                    thread_cpu_ns=stats([r['thread_cpu_ns']for r in selected]))
        values['median_wall_added_ns']=values['True']['wall_ns']['median']-values['False']['wall_ns']['median']
        values['median_cpu_added_ns']=values['True']['thread_cpu_ns']['median']-values['False']['thread_cpu_ns']['median']
        summary[kind]=values
    if any(hashlib.sha256(Path(path).read_bytes()).hexdigest()!=value for path,value in pins.items()):
        raise RuntimeError('Local source changed during measurements')
    report=dict(schema='private.readiness-cause-local-overhead.v1',
        status='LOCAL_FAKE_AND_CANCELLATION_PIPE_ONLY',created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        host=platform.platform(),python=sys.version,command=command,build_stdout=built.stdout,build_stderr=built.stderr,
        source_sha256=pins,sources_unchanged_after_measurement=True,
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),summary=summary,
        rows=rows,before=before,after=after,settings_restored=True,
        serial_or_imu_opened=False,model_loaded=False,ssh_used=False,
        target_jetson_measured=False,active_output_eligible=False,timing_admission_eligible=False,
        limitations=['Mac local overhead only, not Jetson or full pipeline timing.',
            'Preallocation and JSON export excluded; callback binding and all trace clocks included.',
            'Native case uses only a local cancellation pipe; synthetic Future completions occur on main thread.',
            'No bus-worker, GIL-contention or real boot-check cost is reproduced.',
            'Four ABBA blocks retain all observations, including outliers; no gain/stability inference.'])
    path=out/'report.json';path.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary,indent=2));print(path,hashlib.sha256(path.read_bytes()).hexdigest())


if __name__=='__main__':main()
