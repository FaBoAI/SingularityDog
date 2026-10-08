"""Compare bounded voltage/acquisition Future polling and hints, without motor I/O.

Only anonymous pipes and synthetic ThreadPool tasks are used. Completion
times are scheduled with the existing native release waiter. Pipe creation,
callback publication, result takeout and cleanup remain inside the measurement.
"""
import argparse
from concurrent.futures import Future, ThreadPoolExecutor
import gc
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import threading
import time

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime


def summary(values):
    ordered=sorted(values)
    return {'samples':len(values),'median_us':statistics.median(values)/1000,
            'p99_us':ordered[int((len(ordered)-1)*.99)]/1000,'max_us':max(values)/1000}


def run(library,*,samples=100,trials=4,warmup=10):
    if (type(samples) is not int or not 1<=samples<=1000 or
            type(trials) is not int or not 1<=trials<=10 or
            type(warmup) is not int or not 0<=warmup<=1000):
        raise ValueError('Bounded positive sample/trial counts required')
    cancellation=os.pipe()
    for fd in cancellation:os.set_blocking(fd,False)
    gc_before=gc.isenabled();rows=[]
    try:
        waiter=native.make_owned_waiter(library,cancellation[0],spin_us=500)
        if not waiter.future_readiness_available:
            raise ValueError('Native Future readiness ABI required')
        # Wrapping only disables optional hint admission. Both alternatives
        # call the exact same pinned C++ release/cancellation implementation.
        legacy=lambda deadline:waiter(deadline)
        workers=runtime.BusWorkers.__new__(runtime.BusWorkers)
        workers.clock=time.monotonic_ns;workers.aborted=threading.Event();workers.reason=None
        workers.voltage_notification_groups=0;workers.voltage_notification_waits=0
        workers.acquisition_notification_groups=0;workers.acquisition_notification_waits=0
        workers.native_pair=None
        def emergency(reason):
            workers.reason=reason;workers.aborted.set()
        workers.emergency=emergency
        def complete(release,scope):
            native.wait_until(library,cancellation[0],release,spin_us=200)
            return (scope,time.monotonic_ns())
        with ThreadPoolExecutor(max_workers=3,thread_name_prefix='synthetic-input') as pool:
            def measured(name,case,index):
                workers.aborted.clear();workers.reason=None
                begin=time.monotonic_ns()
                if case=='already_ready':
                    futures={scope:Future() for scope in ('front','rear')}
                    for scope,future in futures.items():future.set_result((scope,begin))
                else:
                    # A 1500 us lead lets both genuine worker Futures start.
                    # The tail varies over a 200 us polling period; reverse the
                    # last-finishing bus so neither method benefits from order.
                    tail=1_600_000+(index%8)*25_000
                    scopes=('front','rear')
                    delays=(1_500_000,tail) if index%2 else (tail,1_500_000)
                    if case=='delayed_acquisition':
                        scopes=(*scopes,'imu');delays=(*delays,1_550_000)
                        # Every original input must be the last one sometimes.
                        if index%3==0:delays=(1_500_000,1_550_000,tail)
                    futures={scope:pool.submit(complete,begin+delay,scope)
                             for scope,delay in zip(scopes,delays)}
                join_begin=time.monotonic_ns();cpu_begin=time.thread_time_ns()
                selected=waiter if name=='notification' else legacy
                if case=='delayed_acquisition':
                    bus_results,imu_result=workers.collect_acquisition(
                        {scope:futures[scope] for scope in ('front','rear')},futures['imu'],
                        deadline_ns=begin+20_000_000,deadline_wait=selected)
                    results={**bus_results,'imu':imu_result}
                else:
                    results=workers.collect_voltage(futures,deadline_ns=begin+20_000_000,
                        deadline_wait=selected)
                cpu_end=time.thread_time_ns();end=time.monotonic_ns()
                if set(results)!=set(futures) or not all(f.done() for f in futures.values()):
                    raise AssertionError('Original input Futures were not ready')
                for scope,future in futures.items():
                    if results[scope] is not future.result() or results[scope][0]!=scope:
                        raise AssertionError('Original result identity or bus order changed')
                latest=max(value[1] for value in results.values())
                if not begin<=latest<=end<begin+20_000_000 or workers.aborted.is_set():
                    raise AssertionError('Synthetic original deadline or timestamps violated')
                return {'join_ns':end-join_begin,'handoff_after_owner_ns':end-latest,
                        'main_cpu_ns':cpu_end-cpu_begin,'whole_phase_ns':end-begin}
            for index in range(warmup):
                for case in ('delayed','delayed_acquisition'):
                    for name in ('poll','notification'):measured(name,case,index)
            gc.collect();gc.disable()
            for case in ('already_ready','delayed','delayed_acquisition'):
                for trial in range(trials):
                    measured_rows={'poll':[],'notification':[]}
                    for index in range(samples):
                        order=('poll','notification') if (index+trial)%2==0 else ('notification','poll')
                        for name in order:measured_rows[name].append(measured(name,case,index))
                    fields=('join_ns','handoff_after_owner_ns','main_cpu_ns','whole_phase_ns')
                    row={'case':case,'trial':trial,'raw_samples':measured_rows}
                    for name in measured_rows:
                        row[name]={field:summary([sample[field] for sample in measured_rows[name]])
                                   for field in fields}
                    row['median_handoff_saved_us']=(
                        row['poll']['handoff_after_owner_ns']['median_us']-
                        row['notification']['handoff_after_owner_ns']['median_us'])
                    row['median_main_cpu_saved_us']=(row['poll']['main_cpu_ns']['median_us']-
                        row['notification']['main_cpu_ns']['median_us'])
                    rows.append(row)
        return {'schema':'singularitydog.voltage-future-readiness-benchmark.v1',
            'platform':platform.platform(),'machine':platform.machine(),
            'scope':'synthetic original Futures with anonymous notification/cancellation pipes',
            'measurement_order':'alternate method each sample; reverse first method each trial',
            'includes_group_creation_callbacks_takeout_cleanup':True,
            'absolute_deadline_ns':20_000_000,'original_future_results_checked':True,
            'notification_groups':workers.voltage_notification_groups,
            'notification_wait_calls':workers.voltage_notification_waits,
            'acquisition_notification_groups':workers.acquisition_notification_groups,
            'acquisition_notification_wait_calls':workers.acquisition_notification_waits,
            'hardware_opened':False,'motor_commands_sent':False,'network_used':False,
            'jetson_measured':False,'whole_robot_cycle_measured':False,'timing_admission_eligible':False,
            'trials':rows}
    finally:
        for fd in cancellation:os.close(fd)
        if gc_before:gc.enable()


def main():
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--library-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--samples',type=int,default=100)
    parser.add_argument('--trials',type=int,default=4)
    parser.add_argument('--warmup',type=int,default=10)
    args=parser.parse_args()
    if args.output.exists() or args.output.is_symlink():parser.error('Fresh output required')
    paths={'transport.cpp':args.library.parent/'transport.cpp',
           'native_active_transport.py':Path(native.__file__),
           'policy_output_runtime.py':Path(runtime.__file__),
           'benchmark_voltage_readiness.py':Path(__file__)}
    digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    before={name:digest(path) for name,path in paths.items()}
    gc_before=gc.isenabled()
    library=native.load_library(args.library,expected_sha256=args.library_sha256)
    report=run(library,samples=args.samples,trials=args.trials,warmup=args.warmup)
    after={name:digest(path) for name,path in paths.items()}
    if before!=after:raise RuntimeError('A measured source changed')
    report.update(source_sha256=before,source_unchanged=True,library_sha256=digest(args.library),
                  gc_restored=gc.isenabled()==gc_before)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({'output':str(args.output),'trials':[
        {key:row[key] for key in ('case','trial','median_handoff_saved_us','median_main_cpu_saved_us')}
        for row in report['trials']], 'gc_restored':report['gc_restored']}))


if __name__=='__main__':main()
