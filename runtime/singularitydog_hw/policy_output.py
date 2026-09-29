"""Explicit supported-only learned target runner. Default: file-only plan.

Requires a separately reviewed profile. No automatic arming, gain escalation,
remote reconnect, resumption or unsupported standing is available.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess

from . import math_thread_startup as math_threads
from .policy_live_profile import (ProfileError, add_transport_arguments, load_profile,
                                  transport_settings, telemetry_settings, SUPPORTED_PRELOAD_5S)


class SignalState:
    """Signal-safe Python state: no locks and never a blocking pipe write.

    Only primitive flag assignment and a nonblocking byte notification happen
    in a signal handler. Repeated cancellation remains level-triggered in C++.
    """
    def __init__(self,write_fd):
        self.write_fd=write_fd;self.cancelled=False;self.graceful=False
        os.set_blocking(write_fd,False)
    def cancel(self):
        self.cancelled=True
        try:os.write(self.write_fd,b'x')
        except BlockingIOError:pass  # Existing bytes already signal cancellation.
    def is_set(self):return self.graceful
    def handler(self,signum,frame):
        if signum==getattr(signal,'SIGUSR1',None):self.graceful=True
        else:self.cancel()


def main(argv=None,*,execution=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',required=True)
    p.add_argument('--execute-supported',action='store_true')
    p.add_argument('--execute-supported-preload',action='store_true',
                   help='Execute only the separately reviewed five-second geometric extend/return path')
    p.add_argument('--execute-fixed-catch',action='store_true')
    p.add_argument('--fixed-catch-ready',action='store_true')
    p.add_argument('--support-in-place',action='store_true');p.add_argument('--cutoff-ready',action='store_true')
    for key in ('front-port','rear-port','library','output','audio','audio-sha256','audio-device'):
        p.add_argument('--'+key)
    p.add_argument('--power-epoch',help='Explicit current motor-power epoch, matching the reviewed profile')
    p.add_argument('--pre-cycle-policy-warmup-calls',type=int,choices=(10,),metavar='10',
                   help='Opt-in R22 synthetic model warmup after worker startup, before CPU4 pin')
    p.add_argument('--main-thread-cpu',type=int,choices=(4,),metavar='4',
                   help='Opt-in R22 main-thread CPU4 pin; I/O workers keep their original CPU mask by default')
    p.add_argument('--exclude-policy-cpu-from-workers',action='store_true',
                   help='Experimental supported-only R22 comparison: exclude CPU4 from CAN/IMU workers, then restore')
    p.add_argument('--post-pin-policy-prime-calls',type=int,choices=(10,),metavar='10',
                   help='Optionally prime ten calls on reused model input buffers after CPU4 pin')
    p.add_argument('--defer-gc-during-cycles',action='store_true',
                   help='Temporarily defer automatic GC across the finite output cycle loop')
    p.add_argument('--absolute-epoch-cadence',action='store_true',
                   help='Opt-in fixed 20ms active release epochs; skip/STOP on a missed slot')
    p.add_argument('--release-spin-us',type=int,choices=(200,500),
                   help='Use the pinned active C++ cancellation-aware release wait')
    p.add_argument('--active-timer-slack-ns',type=int,choices=(1000,),
                   help='Set exactly 1us timer slack on the three active I/O workers, then restore')
    p.add_argument('--single-thread-math',action='store_true',
                   help='Opt in to OMP/OPENBLAS/MKL thread counts of 1 before NumPy/Torch import')
    add_transport_arguments(p)
    a=p.parse_args(argv)
    if a.execute_fixed_catch and (a.execute_supported_preload or a.execute_supported or execution is not None or
                                  not a.fixed_catch_ready):
        p.error('Fixed catch requires its dedicated execution and ready flags')
    if a.fixed_catch_ready and not a.execute_fixed_catch:
        p.error('Fixed-catch readiness may only accompany its dedicated execution')
    try:math_startup=math_threads.configure_single_thread_math(a.single_thread_math)
    except math_threads.MathThreadStartupError as error:p.error(str(error))
    r22=a.main_thread_cpu is not None or a.pre_cycle_policy_warmup_calls is not None or a.post_pin_policy_prime_calls is not None
    if r22 and (a.main_thread_cpu!=4 or a.pre_cycle_policy_warmup_calls!=10):
        p.error('R22 startup requires --pre-cycle-policy-warmup-calls 10 and --main-thread-cpu 4 together')
    if a.exclude_policy_cpu_from_workers and (not r22 or
                                                  execution is not None and not a.execute_fixed_catch):
        p.error('--exclude-policy-cpu-from-workers requires R22 supported-only output')
    if a.release_spin_us is not None and not a.absolute_epoch_cadence:
        p.error('--release-spin-us requires --absolute-epoch-cadence')
    if a.execute_supported_preload and (a.execute_supported or a.execute_fixed_catch or execution is not None):
        p.error('Geometric preload requires its dedicated supported-only execution')
    active=a.execute_supported or a.execute_fixed_catch or a.execute_supported_preload
    profile=load_profile(a.profile,require_approved=active)
    preload_mode=profile.get('diagnostic_timing_acceptance')==SUPPORTED_PRELOAD_5S
    if active and preload_mode!=a.execute_supported_preload:
        p.error('Geometric preload profile and --execute-supported-preload must match')
    if a.execute_supported_preload:
        if not a.absolute_epoch_cadence:
            p.error('Geometric preload requires --absolute-epoch-cadence')
        a.execute_supported=True
    if a.execute_fixed_catch:
        from .fixed_catch_hold import FixedCatchExecution
        try:
            # The CPU performance wrapper starts a new session. Its child has
            # the operator's terminal on inherited stdin/stdout, but no
            # controlling /dev/tty. Use those verified descriptors directly.
            execution=FixedCatchExecution(0,write_fd=1)
        except (ValueError,OSError) as error:p.error(str(error))
        a.execute_supported=True
    elif profile.get('scope')=='fixed_catch_current_hold_only':
        p.error('Fixed-catch profile requires the dedicated terminal execution path')
    try:
        pacing=transport_settings(profile,request_gap_us=a.request_gap_us,request_window=a.request_window)
    except ProfileError as error:
        p.error(str(error))
    if execution is not None:execution.bind_profile(profile,active=active)
    if not a.execute_supported:
        print(json.dumps({'status':'PLAN_ONLY','output_allowed':False,'profile_reviewed':profile['output_allowed'],
            'scope':profile['scope'],'duration_s':profile['duration_s'],
            'blockers':profile['blockers'],'profile_sha256':profile['profile_sha256'],
            'hardware_opened':False,'transport_settings':pacing,
            'telemetry_cadence':telemetry_settings(profile),
            'r22_startup_selected':r22,'post_pin_policy_prime_calls':a.post_pin_policy_prime_calls,
            'defer_gc_during_cycles':a.defer_gc_during_cycles,
            'exclude_policy_cpu_from_workers':a.exclude_policy_cpu_from_workers,
            'absolute_epoch_cadence':a.absolute_epoch_cadence,
            'release_spin_us':a.release_spin_us,
            'active_timer_slack_ns':a.active_timer_slack_ns,
            'math_thread_startup':math_startup,
            'actual_policy_output_20ms_verified':False},ensure_ascii=False,indent=2));return 0
    if (not a.support_in_place and not a.execute_fixed_catch) or not a.cutoff_ready:
        p.error('Reviewed support or fixed catch and immediate cutoff are required')
    if any(not getattr(a,k) for k in ('front_port','rear_port','library','output','audio','audio_sha256','audio_device','power_epoch')):
        p.error('Explicit ports, library, private output, pinned audio/device and current power epoch required')
    if a.power_epoch!=profile['motor_power_epoch']:p.error('Motor power epoch differs; review/capture again')
    audio=Path(a.audio).expanduser().resolve(strict=True)
    if hashlib.sha256(audio.read_bytes()).hexdigest()!=a.audio_sha256:p.error('Audio SHA256 mismatch')
    out=Path(a.output).expanduser().resolve()
    if any((x/'.git').exists() for x in (out,*out.parents)):p.error('Raw records must be outside Git')
    out.mkdir(parents=True,mode=0o700,exist_ok=False)
    report={'status':'ABORTED_BEFORE_OUTPUT','errors':[],'motor_enable_sent':False,'learned_targets_sent':False,
            'transport_settings':pacing,'telemetry_cadence':telemetry_settings(profile),
            'r22_startup_selected':r22,'post_pin_policy_prime_calls':a.post_pin_policy_prime_calls,
            'defer_gc_during_cycles':a.defer_gc_during_cycles,
            'exclude_policy_cpu_from_workers':a.exclude_policy_cpu_from_workers,
            'absolute_epoch_cadence':a.absolute_epoch_cadence,
            'release_spin_us':a.release_spin_us,
            'active_timer_slack_ns':a.active_timer_slack_ns,
            'math_thread_startup':math_startup}
    cr,cw=os.pipe();signals=SignalState(cw);handlers={}
    device=None
    try:
        from . import dual_can_pipeline_benchmark as dual
        from . import policy_observer_live as live
        from . import native_active_transport as native
        from . import imu
        from .policy_output_model import LivePolicyModel
        from .policy_output_runtime import run_supported_policy, BUSES
        if a.single_thread_math:
            math_startup['before_torch_import_env']=math_threads.verify_before_math_import()
        else:
            math_startup['before_torch_import_env']=math_threads.effective_math_thread_env()
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        # Load before opening serial. Default warmup runs here; R22 defers it
        # until the active runner's I/O workers are ready, still before enable.
        raw_model=(LivePolicyModel(profile,defer_warmup=True) if r22 else LivePolicyModel(profile))
        model=raw_model;lib=native.load_library(a.library)
        if execution is not None:
            execution.connect_cancel(signals.cancel)
            model=execution.wrap_model(model)
        bindings=dual.validate_ports(a.front_port,a.rear_port)
        with ExitStack() as stack:
            stack.enter_context(dual.pipeline.ownership_locks());stack.enter_context(live.imu_ownership_lock())
            guard=dual.BootIdentityGuard();stack.callback(guard.close)
            if guard.boot_id!=profile['boot_id']:raise RuntimeError('Jetson boot changed; reviewed profile is stale')
            def check():
                if signals.cancelled:raise InterruptedError('Emergency cancellation')
                guard.check()
            for sig in (signal.SIGINT,signal.SIGTERM,getattr(signal,'SIGUSR1',signal.SIGINT)):
                if sig not in handlers:handlers[sig]=signal.signal(sig,signals.handler)
            import serial
            sessions={}
            for scope,binding in bindings.items():
                stack.enter_context(dual.port_lock(binding['resolved']))
                port=serial.Serial(port=None,baudrate=921600,timeout=0,write_timeout=.1,exclusive=True)
                port.dtr=port.rts=False;port.port=binding['path'];port.open();stack.callback(port.close)
                if not dual.binding_matches(binding) or os.fstat(port.fileno()).st_rdev!=binding['st_rdev']:
                    raise RuntimeError('USB port binding changed')
                bootfd=os.open('/proc/sys/kernel/random/boot_id',os.O_RDONLY|os.O_CLOEXEC);stack.callback(os.close,bootfd)
                ids=BUSES[scope]
                sessions[scope]=native.ActiveSession(lib,port.fileno(),first_id=ids[0],cancel_fd=cr,
                    boot_fd=bootfd,boot_id=guard.boot_id,
                    raw_lower_by_id={i:-12.57 for i in ids},raw_upper_by_id={i:12.57 for i in ids},
                    kp_max_by_id={i:profile['axes'][str(i)]['kp'] for i in ids},
                    kd_max_by_id={i:profile['axes'][str(i)]['kd'] for i in ids},
                    gap_ns=pacing['request_gap_us']*1000,window=pacing['request_window'])
                stack.callback(sessions[scope].close)
            device=imu.ICM20948();stack.callback(device.close);configuration=device.start()
            def announce():
                check()
                subprocess.run(['aplay','-D',a.audio_device,str(audio)],check=True,timeout=8.,
                               stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            startup_options={}
            if r22:
                startup_options.update(startup_model=raw_model,main_thread_cpu=4,
                    pre_cycle_policy_warmup_calls=10,
                    post_pin_policy_prime_calls=a.post_pin_policy_prime_calls)
            if a.defer_gc_during_cycles:startup_options['defer_gc_during_cycles']=True
            if a.exclude_policy_cpu_from_workers:startup_options['exclude_policy_cpu_from_workers']=True
            if a.absolute_epoch_cadence:startup_options['absolute_epoch_cadence']=True
            if a.release_spin_us is not None:
                if getattr(lib,'sda_wait_until',None) is None:
                    raise RuntimeError('Pinned active library lacks bounded release wait')
                startup_options['deadline_wait']=lambda target: native.wait_until(
                    lib,cr,target,spin_us=a.release_spin_us)
            if a.active_timer_slack_ns is not None:
                startup_options['active_timer_slack_ns']=a.active_timer_slack_ns
            report=run_supported_policy(profile,sessions,device.read_sample,model,cancel_io=signals.cancel,
                check=check,announce=announce,stop_requested=signals,supervision=execution,
                **startup_options)
            report.update(profile_sha256=profile['profile_sha256'],boot_id=guard.boot_id,
                motor_power_epoch=a.power_epoch,model_provenance=model.provenance,
                actual_model_calls=model.calls,imu_configuration=configuration,
                r22_startup_selected=r22,post_pin_policy_prime_calls=a.post_pin_policy_prime_calls,
                defer_gc_during_cycles=a.defer_gc_during_cycles,
                exclude_policy_cpu_from_workers=a.exclude_policy_cpu_from_workers,
                absolute_epoch_cadence=a.absolute_epoch_cadence,
                release_spin_us=a.release_spin_us,
                active_timer_slack_ns=a.active_timer_slack_ns,
                math_thread_startup=math_startup)
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)
        report['imu_restore_status']=device.restore_status if device is not None else 'not_started'
        if device is not None and device.restore_status not in ('restored','not_needed'):
            report['errors'].append('IMU restoration unconfirmed')
            if report['status'] in ('COMPLETE_SUPPORTED_OUTPUT', 'COMPLETE_FIXED_CATCH_HOLD'):
                report['status']='ABORTED_RESTORE'
        report['transport_settings']=pacing
        report['telemetry_cadence']=telemetry_settings(profile)
        report['math_thread_startup']=math_startup
        report['actual_policy_output_20ms_verified']=False
        if execution is not None:report=execution.decorate_report(report)
        if a.execute_fixed_catch:execution.close()
        # The ground operator reader can signal cancellation. Join it before
        # closing its notification pipe, including setup/early-failure paths.
        os.close(cr);os.close(cw)
        with os.fdopen(os.open(out/'report.json',os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600),'w') as f:
            json.dump(report,f,ensure_ascii=False,allow_nan=False);f.write('\n')
        print(json.dumps({k:report.get(k) for k in ('status','errors','motor_enable_sent','learned_targets_sent','preload_targets_sent','output_kind','preload_return_commanded','preload_return_measured','stop_confirmed','deadline20ms_misses','transport_settings')},ensure_ascii=False))
    return 0 if report['status'] in ('COMPLETE_SUPPORTED_OUTPUT','COMPLETE_BOUNDED_GROUND_TRIAL',
                                    'COMPLETE_FIXED_CATCH_HOLD') else 2


if __name__=='__main__':raise SystemExit(main())
