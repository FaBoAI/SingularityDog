"""Explicit supported-only learned target runner. Default: file-only plan.

Requires a separately reviewed profile. No automatic arming, gain escalation,
remote reconnect, resumption or unsupported standing is available.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import tempfile
import threading
import time
import wave

from . import math_thread_startup as math_threads
from .policy_live_profile import (ProfileError, add_transport_arguments, load_profile,
                                  transport_settings, telemetry_settings, SUPPORTED_PRELOAD_5S,
                                  human_supported_partial_current_hold_settings,
                                  prepared_voltage_publication_settings)


HUMAN_AUDIO_STAGES=('prepare_ease','go','resupport','abort')


def pinned_audio_file(path,digest,*,go=False):
    """Pin real PCM metadata before opening hardware, never trust declared time."""
    candidate=Path(path).expanduser()
    if candidate.is_symlink():raise ValueError('Pinned audio must not be a symlink')
    target=candidate.resolve(strict=True)
    if not target.is_file() or hashlib.sha256(target.read_bytes()).hexdigest()!=digest:
        raise ValueError('Pinned stage audio SHA256 mismatch')
    with wave.open(str(target),'rb') as audio:
        frames,rate=audio.getnframes(),audio.getframerate()
        if (audio.getcomptype()!='NONE' or frames<=0 or rate<=0 or
                audio.getnchannels() not in (1,2) or audio.getsampwidth() not in (1,2,3,4)):
            raise ValueError('Finite uncompressed PCM stage audio is required')
        if len(audio.readframes(frames))!=frames*audio.getnchannels()*audio.getsampwidth():
            raise ValueError('Pinned PCM audio is truncated')
        duration=frames/rate
    if go and duration>.12:raise ValueError('Human-supported Go audio exceeds 0.12 seconds')
    return {'path':str(target),'sha256':digest,'duration_s':duration}


class HumanStageAudioPlayer:
    """One prestarted actor owns all playback processes; motor calls never wait.

    Cancellation prevents queued preparation/Go and interrupts running playback.
    Real process start/exit timestamps describe issued cues, not physical sound
    onset or human load transfer. Recovery audio remains allowed after cancel.
    """
    def __init__(self,stages,device,*,clock=time.monotonic_ns,popen=None):
        if set(stages)!=set(HUMAN_AUDIO_STAGES) or not device:
            raise ValueError('All four pinned human audio stages and device are required')
        self.stages={key:dict(value) for key,value in stages.items()}
        if any(type(source.get('duration_s')) not in (int,float) or
               not math.isfinite(source['duration_s']) or source['duration_s']<=0
               for source in self.stages.values()):
            raise ValueError('Real finite stage audio durations are required')
        self._snapshots={}
        try:
            for stage,source in self.stages.items():
                raw=Path(source['path']).read_bytes()
                if hashlib.sha256(raw).hexdigest()!=source['sha256']:
                    raise ValueError('Pinned human audio changed before worker setup')
                snapshot=tempfile.TemporaryFile()
                self._snapshots[stage]=snapshot
                snapshot.write(raw);snapshot.flush();snapshot.seek(0)
        except BaseException:
            for snapshot in self._snapshots.values():snapshot.close()
            raise
        self.device=device;self.clock=clock;self.popen=popen or subprocess.Popen
        self._pending=queue.Queue(maxsize=2)
        self._cancelled=threading.Event();self._closed=threading.Event()
        self._shutdown=threading.Event()
        self._ready=threading.Event();self.events=[];self.closed=False;self.cleanup_error=None
        self._thread=threading.Thread(target=self._run,name='human-hold-audio',daemon=True)
        try:self._thread.start()
        except BaseException:
            for snapshot in self._snapshots.values():snapshot.close()
            raise
        if not self._ready.wait(1.):
            self._closed.set()
            self._thread.join(timeout=.5)
            if not self._thread.is_alive():
                for snapshot in self._snapshots.values():snapshot.close()
            raise RuntimeError('Human audio worker did not start before enable')

    def start(self,stage,on_complete,on_failure,on_started=None):
        if (stage not in self.stages or not callable(on_complete) or
                not callable(on_failure) or on_started is not None and not callable(on_started)):
            raise ValueError('Invalid stage audio callback contract')
        if self._closed.is_set() or self._shutdown.is_set() or self._cancelled.is_set() and stage in ('prepare_ease','go'):
            raise RuntimeError('Human audio preparation/Go was cancelled')
        try:self._pending.put_nowait((stage,on_complete,on_failure,on_started))
        except queue.Full as error:raise RuntimeError('Human audio stage queue is full') from error

    def cancel(self):
        self._cancelled.set()

    @staticmethod
    def _terminate(process):
        if process.poll() is not None:return
        process.terminate()
        try:process.wait(timeout=.2)
        except subprocess.TimeoutExpired:
            process.kill();process.wait(timeout=.2)

    def _run(self):
        self._ready.set()
        while not self._closed.is_set():
            if self._shutdown.is_set() and self._pending.empty():return
            # Idle actors do not wake every control cycle and contend for the
            # GIL. close sends a sentinel; a full queue drains before exit.
            item=self._pending.get()
            if item is None:
                self._pending.task_done();return
            stage,complete,failure,started_callback=item
            process=None;started=None;row={'stage':stage,'status':'queued'}
            self.events.append(row)
            try:
                if self._cancelled.is_set() and stage in ('prepare_ease','go'):
                    raise RuntimeError('Stage cancelled before owned playback')
                source=self.stages[stage]
                # Verify the pinned bytes again at the actor boundary. A file
                # replaced during the hold must not turn into an unreviewed Go.
                if hashlib.sha256(Path(source['path']).read_bytes()).hexdigest()!=source['sha256']:
                    raise RuntimeError('Pinned stage audio changed before playback')
                if self._closed.is_set() or self._cancelled.is_set() and stage in ('prepare_ease','go'):
                    raise RuntimeError('Stage cancelled before process start')
                snapshot=self._snapshots[stage];snapshot.seek(0)
                # Play the anonymous approved byte snapshot through stdin. The
                # original path cannot race the hash check and child file open.
                process=self.popen(['aplay','-D',self.device],
                    stdin=snapshot,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                started=self.clock();row.update(status='started',started_ns=started)
                if self._closed.is_set() or self._cancelled.is_set() and stage in ('prepare_ease','go'):
                    raise RuntimeError('Stage cancelled during process start')
                if started_callback is not None:started_callback(stage,started)
                deadline=time.monotonic()+source['duration_s']+.25
                while process.poll() is None:
                    if self._closed.is_set() or self._cancelled.is_set() and stage in ('prepare_ease','go'):
                        raise RuntimeError('Stage playback cancelled')
                    if time.monotonic()>=deadline:raise TimeoutError('Stage playback completion deadline')
                    self._closed.wait(.005)
                if process.returncode!=0:raise RuntimeError('Stage playback exited '+str(process.returncode))
                finished=self.clock();row.update(status='complete',finished_ns=finished)
                if self._closed.is_set() or self._cancelled.is_set() and stage in ('prepare_ease','go'):
                    raise RuntimeError('Stage completion cancelled')
                complete(stage,started,finished)
            except BaseException as error:
                if process is not None:
                    try:self._terminate(process)
                    except BaseException as cleanup:
                        row['cleanup_error']=repr(cleanup);self.cleanup_error=repr(cleanup)
                        self._closed.set()  # Never overlap recovery playback with an unclosed process.
                row.update(status='failed',error=type(error).__name__+': '+str(error),finished_ns=self.clock())
                try:failure(stage,row['error'])
                except BaseException as callback:row['callback_error']=repr(callback)
            finally:self._pending.task_done()

    def close(self):
        if self.closed:return
        # Called only after STOP collection. Preparation/Go cancel immediately,
        # while urgent recovery already queued or playing drains to completion.
        # Nothing on the motor loop waits for speech or an operator ACK.
        self._cancelled.set();self._shutdown.set()
        try:self._pending.put_nowait(None)
        except queue.Full:pass
        recovery_max=max(self.stages[stage]['duration_s']+.25
                         for stage in ('resupport','abort'))
        self._thread.join(timeout=3*recovery_max+.7)
        if self._thread.is_alive():raise RuntimeError('Human audio worker cleanup unconfirmed')
        self.closed=True;self._closed.set()
        for snapshot in self._snapshots.values():snapshot.close()
        if self.cleanup_error is not None:raise RuntimeError('Stage process cleanup unconfirmed: '+self.cleanup_error)
        failed_recovery=[row for row in self.events
                         if row['stage'] in ('resupport','abort') and row['status']=='failed']
        if failed_recovery:
            raise RuntimeError('Human recovery audio playback unconfirmed: '+failed_recovery[-1]['error'])


def human_audio_report(execution,player):
    """Post-STOP process evidence; keep it separate from STOP and audibility."""
    events=[] if player is None else [dict(row) for row in player.events]
    requested=set() if execution is None else set(execution.audio_requests)&{'resupport','abort'}
    recovery=[row for row in events if row['stage'] in ('resupport','abort')]
    completed={row['stage'] for row in recovery if row['status']=='complete'}
    failed=[row['stage']+': '+row.get('error','Playback unconfirmed')
            for row in recovery if row['status']!='complete']
    missing=requested-{row['stage'] for row in recovery}
    errors=failed+['No owned playback completion evidence: '+stage for stage in sorted(missing)]
    return dict(human_audio_playback=events,human_audio_recovery_requested=bool(requested),
        human_audio_recovery_confirmed=bool(requested) and requested<=completed and not errors,
        human_audio_recovery_errors=errors,human_audio_completion_proves_audibility=False)


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
    p.add_argument('--execute-human-supported-partial',action='store_true',
                   help='Only the separately reviewed eight-second current hold with two continuous body-support operators and pinned spoken cues')
    human_flags=('two-operators-full-weight-catch','slight-ease-only','resupport-before-stop',
                 'off-power-rehearsal','side-view-video-ready','paws-floor')
    for flag in human_flags:p.add_argument('--'+flag,action='store_true')
    for stage in HUMAN_AUDIO_STAGES:
        option='--human-'+stage.replace('_','-')+'-audio'
        p.add_argument(option);p.add_argument(option+'-sha256')
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
                   help='Set exactly 1us timer slack on the control thread and three active I/O workers, then restore')
    p.add_argument('--prepare-voltage-before-feedback-publication',action='store_true',
                   help='Use only the matching reviewed V3 profile: prepare each voltage transaction before publishing validated feedback')
    p.add_argument('--single-thread-math',action='store_true',
                   help='Opt in to OMP/OPENBLAS/MKL thread counts of 1 before NumPy/Torch import')
    add_transport_arguments(p)
    a=p.parse_args(argv)
    if a.execute_fixed_catch and (a.execute_human_supported_partial or a.execute_supported_preload or a.execute_supported or execution is not None or
                                  not a.fixed_catch_ready):
        p.error('Fixed catch requires its dedicated execution and ready flags')
    if a.fixed_catch_ready and not a.execute_fixed_catch:
        p.error('Fixed-catch readiness may only accompany its dedicated execution')
    fresh_human_flags=[getattr(a,flag.replace('-','_')) for flag in human_flags]
    if a.execute_human_supported_partial:
        if (a.execute_supported or a.execute_supported_preload or a.execute_fixed_catch or
                execution is not None or not all(fresh_human_flags) or not a.cutoff_ready):
            p.error('Human-supported partial hold requires its dedicated execution, fresh two-operator confirmations and cutoff readiness')
        if not a.absolute_epoch_cadence:
            p.error('Human-supported partial hold requires --absolute-epoch-cadence')
    elif any(fresh_human_flags):
        p.error('Human-support confirmations may only accompany the dedicated partial-hold execution')
    human_audio_flags={stage:(getattr(a,'human_'+stage+'_audio'),
                             getattr(a,'human_'+stage+'_audio_sha256'))
                       for stage in HUMAN_AUDIO_STAGES}
    if a.execute_human_supported_partial and not all(all(values) for values in human_audio_flags.values()):
        p.error('All four pinned stage audio files and SHA256 values are required')
    if not a.execute_human_supported_partial and any(any(values) for values in human_audio_flags.values()):
        p.error('Human stage audio belongs only to its dedicated partial-hold execution')
    try:math_startup=math_threads.configure_single_thread_math(a.single_thread_math)
    except math_threads.MathThreadStartupError as error:p.error(str(error))
    r22=a.main_thread_cpu is not None or a.pre_cycle_policy_warmup_calls is not None or a.post_pin_policy_prime_calls is not None
    if r22 and (a.main_thread_cpu!=4 or a.pre_cycle_policy_warmup_calls!=10):
        p.error('R22 startup requires --pre-cycle-policy-warmup-calls 10 and --main-thread-cpu 4 together')
    if a.exclude_policy_cpu_from_workers and (not r22 or
                                                  execution is not None and not (a.execute_fixed_catch or a.execute_human_supported_partial)):
        p.error('--exclude-policy-cpu-from-workers requires R22 supported-only output')
    if a.release_spin_us is not None and not a.absolute_epoch_cadence:
        p.error('--release-spin-us requires --absolute-epoch-cadence')
    if a.execute_supported_preload and (a.execute_supported or a.execute_fixed_catch or a.execute_human_supported_partial or execution is not None):
        p.error('Geometric preload requires its dedicated supported-only execution')
    active=a.execute_supported or a.execute_fixed_catch or a.execute_supported_preload or a.execute_human_supported_partial
    profile=load_profile(a.profile,require_approved=active)
    try:
        prepared_voltage=prepared_voltage_publication_settings(profile)
    except ProfileError as error:p.error(str(error))
    if prepared_voltage is not a.prepare_voltage_before_feedback_publication:
        p.error('--prepare-voltage-before-feedback-publication must exactly match the reviewed profile selection')
    if prepared_voltage and (execution is not None or a.execute_fixed_catch or a.execute_supported_preload or a.execute_human_supported_partial):
        p.error('Prepared voltage publication requires the ordinary box-supported execution path')
    human_mode=human_supported_partial_current_hold_settings(profile)
    if bool(human_mode is not None)!=a.execute_human_supported_partial:
        p.error('Human-supported profile requires its dedicated terminal execution path')
    preload_mode=profile.get('diagnostic_timing_acceptance')==SUPPORTED_PRELOAD_5S
    if active and preload_mode!=a.execute_supported_preload:
        p.error('Geometric preload profile and --execute-supported-preload must match')
    if a.execute_supported_preload:
        if not a.absolute_epoch_cadence:
            p.error('Geometric preload requires --absolute-epoch-cadence')
        a.execute_supported=True
    if a.execute_fixed_catch:
        if profile.get('scope')!='fixed_catch_current_hold_only':
            p.error('Fixed-catch execution requires its dedicated reviewed profile')
        a.execute_supported=True
    elif profile.get('scope')=='fixed_catch_current_hold_only':
        p.error('Fixed-catch profile requires the dedicated terminal execution path')
    if a.execute_human_supported_partial:
        a.execute_supported=True
    try:
        pacing=transport_settings(profile,request_gap_us=a.request_gap_us,request_window=a.request_window)
    except ProfileError as error:
        p.error(str(error))
    if execution is not None:
        try:execution.bind_profile(profile,active=active)
        except BaseException:
            if a.execute_fixed_catch or a.execute_human_supported_partial:execution.close()
            raise
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
            'prepare_voltage_before_feedback_publication':prepared_voltage,
            'math_thread_startup':math_startup,
            'actual_policy_output_20ms_verified':False},ensure_ascii=False,indent=2));return 0
    if (not a.support_in_place and not (a.execute_fixed_catch or a.execute_human_supported_partial)) or not a.cutoff_ready:
        p.error('Reviewed support or fixed catch and immediate cutoff are required')
    if any(not getattr(a,k) for k in ('front_port','rear_port','library','output','audio','audio_sha256','audio_device','power_epoch')):
        p.error('Explicit ports, library, private output, pinned audio/device and current power epoch required')
    if a.power_epoch!=profile['motor_power_epoch']:p.error('Motor power epoch differs; review/capture again')
    audio=Path(a.audio).expanduser().resolve(strict=True)
    if hashlib.sha256(audio.read_bytes()).hexdigest()!=a.audio_sha256:p.error('Audio SHA256 mismatch')
    reviewed_human_audio=None;human_stages=None
    if a.execute_human_supported_partial:
        from .policy_live_profile import human_supported_audio_settings
        reviewed_human_audio=human_supported_audio_settings(profile)
        checked={'brief':pinned_audio_file(a.audio,a.audio_sha256)}
        checked.update({stage:pinned_audio_file(path,digest,go=stage=='go')
                        for stage,(path,digest) in human_audio_flags.items()})
        for stage,value in checked.items():
            approved=reviewed_human_audio['clips'][stage]
            if (value['path']!=str(Path(approved['path']).resolve(strict=True)) or value['sha256']!=approved['sha256'] or
                    abs(value['duration_s']-approved['duration_s'])>1e-9):
                p.error('Stage audio differs from reviewed manifest: '+stage)
        human_stages={stage:checked[stage] for stage in HUMAN_AUDIO_STAGES}
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
            'prepare_voltage_before_feedback_publication':prepared_voltage,
            'math_thread_startup':math_startup}
    cr,cw=os.pipe();signals=SignalState(cw);handlers={}
    device=None;stage_audio=None
    try:
        if a.execute_fixed_catch:
            from .fixed_catch_hold import FixedCatchExecution
            # Take ownership only after every file/flag/power/output check has
            # passed. Setup errors below reach finally and restore both TTY
            # descriptors before returning control to the operator's shell.
            # A CPU wrapper may leave no controlling /dev/tty, so retain the
            # inherited, same-visible-terminal stdin/stdout contract.
            execution=FixedCatchExecution(0,write_fd=1)
            execution.bind_profile(profile,active=True)
        if a.execute_human_supported_partial:
            from .human_supported_hold import HumanSupportedHoldExecution
            stage_audio=HumanStageAudioPlayer(human_stages,a.audio_device)
            execution=HumanSupportedHoldExecution(0,write_fd=1,stage_audio=stage_audio)
            execution.bind_profile(profile,active=True)
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
                if reviewed_human_audio is None:
                    subprocess.run(['aplay','-D',a.audio_device,str(audio)],check=True,timeout=8.,
                                   stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
                else:
                    raw=audio.read_bytes()
                    if hashlib.sha256(raw).hexdigest()!=a.audio_sha256:
                        raise RuntimeError('Pinned human pre-enable brief changed')
                    with tempfile.TemporaryFile() as brief:
                        brief.write(raw);brief.flush();brief.seek(0)
                        subprocess.run(['aplay','-D',a.audio_device],check=True,
                            timeout=reviewed_human_audio['clips']['brief']['duration_s']+.25,
                            stdin=brief,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
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
                startup_options['deadline_wait']=native.make_owned_waiter(
                    lib,cr,spin_us=a.release_spin_us)
            if a.active_timer_slack_ns is not None:
                startup_options['active_timer_slack_ns']=a.active_timer_slack_ns
            if prepared_voltage:
                startup_options['prepare_voltage_before_feedback_publication']=True
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
                prepare_voltage_before_feedback_publication=prepared_voltage,
                math_thread_startup=math_startup)
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)
        report['imu_restore_status']=device.restore_status if device is not None else 'not_started'
        if device is not None and device.restore_status not in ('restored','not_needed'):
            report['errors'].append('IMU restoration unconfirmed')
            if report['status'] in ('COMPLETE_SUPPORTED_OUTPUT', 'COMPLETE_FIXED_CATCH_HOLD',
                                    'COMPLETE_HUMAN_SUPPORTED_PARTIAL_HOLD'):
                report['status']='ABORTED_RESTORE'
        report['transport_settings']=pacing
        report['telemetry_cadence']=telemetry_settings(profile)
        report['math_thread_startup']=math_startup
        report['actual_policy_output_20ms_verified']=False
        if a.execute_human_supported_partial:
            try:
                if execution is not None:execution.close()
                if stage_audio is not None:stage_audio.close()
                if execution is not None and execution.failed is not None:
                    raise RuntimeError('Human supervisor recovery/cleanup failed: '+execution.failed)
            except BaseException as error:
                report['errors'].append('Human stage cleanup: '+type(error).__name__+': '+str(error))
                if report['status']=='COMPLETE_HUMAN_SUPPORTED_PARTIAL_HOLD':
                    report['status']='ABORTED_HUMAN_AUDIO_CLEANUP'
            report.update(human_audio_report(execution,stage_audio))
            if report['human_audio_recovery_requested'] and not report['human_audio_recovery_confirmed']:
                report['errors'].extend('Human recovery audio: '+error
                                        for error in report['human_audio_recovery_errors'])
                if report['status']=='COMPLETE_HUMAN_SUPPORTED_PARTIAL_HOLD':
                    report['status']='ABORTED_HUMAN_AUDIO_CLEANUP'
        if execution is not None:report=execution.decorate_report(report)
        if a.execute_fixed_catch and execution is not None:execution.close()
        # The ground operator reader can signal cancellation. Join it before
        # closing its notification pipe, including setup/early-failure paths.
        os.close(cr);os.close(cw)
        with os.fdopen(os.open(out/'report.json',os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600),'w') as f:
            json.dump(report,f,ensure_ascii=False,allow_nan=False);f.write('\n')
        print(json.dumps({k:report.get(k) for k in ('status','errors','motor_enable_sent','learned_targets_sent','preload_targets_sent','output_kind','preload_return_commanded','preload_return_measured','stop_confirmed','deadline20ms_misses','transport_settings')},ensure_ascii=False))
    return 0 if report['status'] in ('COMPLETE_SUPPORTED_OUTPUT','COMPLETE_BOUNDED_GROUND_TRIAL',
                                    'COMPLETE_FIXED_CATCH_HOLD','COMPLETE_HUMAN_SUPPORTED_PARTIAL_HOLD') else 2


if __name__=='__main__':raise SystemExit(main())
