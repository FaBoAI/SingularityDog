"""Separate, explicitly reviewed ground-trial entry point. Default is PLAN only.

One bounded stage per process. Physical load/contact/slip are reviewed from video;
the runtime only controls bounded commands and checks its available telemetry.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import sys
import threading
import termios
import time
import uuid

from .policy_live_profile import (ProfileError, add_transport_arguments, load_profile,
                                  transport_settings, telemetry_settings)
from .policy_shadow import _json


def sha(raw):return hashlib.sha256(raw).hexdigest()


def read_document(path):
    path=Path(path)
    if path.is_symlink() or not path.is_file() or not 0<path.stat().st_size<=16*1024*1024:
        raise ValueError('A bounded regular ground evidence file is required')
    raw=path.read_bytes()
    if len(raw)>16*1024*1024:raise ValueError('Ground evidence grew beyond the file bound')
    value=_json(raw.decode('utf-8'))
    if not isinstance(value,dict):raise ValueError('Ground evidence must be a JSON object')
    return raw,value


class GroundTrialExecution:
    """Adds reviewed timeline and fresh re-support acknowledgment to the runner.

    No serial/I2C calls and no automatic physical confirmation. A terminal reader
    records Enter times; stale confirmations cannot release the support gate.
    """
    def __init__(self,plan,plan_sha256,*,trial_id=None,execution_kind='hardware',
                 input_stream=None,emit=None,clock=time.monotonic_ns):
        from .ground_trial_trajectory import GroundTimeline
        if not plan.get('execution_plan_validated'):
            raise ValueError('A reviewed ground plan is required')
        if execution_kind not in ('hardware','simulation'):
            raise ValueError('Explicit execution kind required')
        self.plan=plan;self.plan_sha256=plan_sha256;self.timeline=GroundTimeline(**plan['trajectory'])
        self.trial_id=trial_id or str(uuid.uuid4());self.execution_kind=execution_kind
        self.input_stream=input_stream;self.emit=emit or (lambda text: print(text,flush=True));self.clock=clock
        self.started_ns=None;self.model=None;self.cancel=None;self.ack_ns=None
        self.events=[];self._closed=threading.Event();self._reader=None;self._normal_requested=False
        self._input_lock=threading.RLock();self._input_fd=None;self._input_was_blocking=None
        self._input_buffer=b'';self._reader_failed=None
        self._step_index=0;self._resupport_emitted_ns=None;self._resupport_emitted_step=None
        self._accepted_ack_ns=None

    def bind_profile(self,profile,*,active):
        for name in ('profile_sha256','assembly_id','boot_id','motor_power_epoch'):
            key='base_profile_sha256' if name=='profile_sha256' else name
            if profile.get(name)!=self.plan.get(key):raise ValueError('Ground/base profile mismatch: '+name)
        if active and not profile.get('output_allowed'):raise ValueError('Base profile is not reviewed')
        if abs(self.plan['trajectory']['duration_s']-profile['duration_s'])>1e-9:
            raise ValueError('Ground timeline must fit the exact reviewed base duration')
        reserve=max(a['max_command_velocity_rad_s']/a['max_command_acceleration_rad_s2']
                    for a in profile['axes'].values())+profile['stop_duration_s']+.04
        if self.plan['trajectory']['shutdown_reserve_s']<reserve:
            raise ValueError('Ground plan lacks the full braking/gain-down reserve')

    def connect_cancel(self,cancel):self.cancel=cancel
    def wrap_model(self,model):self.model=model;return self
    @property
    def provenance(self):return self.model.provenance
    @property
    def calls(self):return self.model.calls
    @property
    def last_validation(self):return self.model.last_validation
    def validate_inputs(self,*args):return self.model.validate_inputs(*args)
    def __call__(self,sample,imu,now_ns):
        if self.started_ns is None:raise RuntimeError('Ground timeline not started')
        command=self.timeline.command_at((now_ns-self.started_ns)/1e9)
        return self.model(sample,imu,now_ns,command_override=command)

    def _input(self):
        # Raw nonblocking reads avoid TextIO readahead assigning a new timestamp
        # to an old buffered Enter. The cue writer drains this same fd under the
        # same lock before opening a new acknowledgement generation.
        try:
            while not self._closed.is_set():
                ready,_,_=select.select([self._input_fd],[],[],.05)
                if not ready:continue
                with self._input_lock:
                    try:chunk=os.read(self._input_fd,4096)
                    except BlockingIOError:continue  # A concurrent cue flush consumed it.
                    if not chunk:
                        self._reader_failure('TERMINAL_CLOSED')
                        return
                    self._input_buffer+=chunk
                    if len(self._input_buffer)>4096:
                        self._reader_failure('TERMINAL_INPUT_TOO_LONG')
                        return
                    while b'\n' in self._input_buffer:
                        line,self._input_buffer=self._input_buffer.split(b'\n',1)
                        value=line.strip().lower()
                        if value in (b'q',b'stop'):
                            self._reader_failure('OPERATOR_EMERGENCY')
                            return
                        if value in (b'',b'resupported'):
                            self.acknowledge_resupport(self.clock())
        except Exception as error:
            self._reader_failure('TERMINAL_READER_FAILED',type(error).__name__)

    def _reader_failure(self,key,error_type=None):
        self._reader_failed=key
        self.events.append({'key':key,'monotonic_ns':self.clock(),'error_type':error_type})
        if self.cancel:self.cancel()

    def _flush_stale_input(self):
        """Discard queued confirmations, but never discard a queued emergency.

        Called under the input lock. The fd is already nonblocking, so draining
        cannot stall the motor loop if the reader/cue races a terminal event.
        """
        pending=self._input_buffer
        if self._input_fd is not None:
            while True:
                try:chunk=os.read(self._input_fd,4096)
                except BlockingIOError:break
                if not chunk:raise OSError('Terminal closed while establishing input boundary')
                pending+=chunk
                if len(pending)>4096:raise ValueError('Terminal input exceeds boundary buffer')
        if any(line.strip().lower() in (b'q',b'stop') for line in pending.split(b'\n')[:-1]):
            self._reader_failure('OPERATOR_EMERGENCY')
            raise RuntimeError('Queued operator emergency')
        if self._input_fd is not None:termios.tcflush(self._input_fd,termios.TCIFLUSH)
        self._input_buffer=b''

    def acknowledge_resupport(self,now_ns):
        if type(now_ns) is not int or now_ns<0:raise ValueError('Monotonic acknowledgment timestamp required')
        with self._input_lock:
            self.ack_ns=now_ns
            self.events.append({'key':'OPERATOR_RESUPPORT_ACK_RECEIVED','monotonic_ns':now_ns})

    def on_start(self,start_ns):
        if self.started_ns is not None:raise RuntimeError('Ground trial cannot restart')
        self.started_ns=start_ns
        self.events.append({'key':'GROUND_TIMELINE_STARTED','monotonic_ns':start_ns})
        self.emit('GROUND_TIMELINE_STARTED 胴体を支持し続け、合図を待ってください。')
        if self.input_stream is not None:
            try:
                self._input_fd=self.input_stream.fileno()
                self._input_was_blocking=os.get_blocking(self._input_fd)
                os.set_blocking(self._input_fd,False)
            except Exception as error:
                self._reader_failure('TERMINAL_READER_FAILED',type(error).__name__)
                raise RuntimeError('Terminal reader could not start') from error
            try:
                self._reader=threading.Thread(target=self._input,name='ground-operator-input',daemon=True)
                self._reader.start()
            except Exception as error:
                self._reader=None
                self._reader_failure('TERMINAL_READER_FAILED',type(error).__name__)
                try:os.set_blocking(self._input_fd,self._input_was_blocking)
                except OSError:pass
                raise RuntimeError('Terminal reader could not start') from error

    def before_cycle(self,now_ns,*,stop_requested=False):
        if self.started_ns is None:raise RuntimeError('Ground timeline not started')
        if self._reader_failed:raise RuntimeError(self._reader_failed)
        self._step_index+=1
        t=(now_ns-self.started_ns)/1e9
        reannounce=False
        if stop_requested and not self._normal_requested:
            old_timing=self.timeline.timing
            self.timeline.request_stop(t);self._normal_requested=True
            self.events.append({'key':'GRACEFUL_STOP_REQUESTED','monotonic_ns':now_ns})
            timing=self.timeline.timing
            if any(timing[key]!=old_timing[key] for key in
                   ('resupport_window_open_s','latest_stop_start_s')):
                # A replanned window requires an actually displayed new cue,
                # including when its opening time is unchanged but the deadline moved.
                with self._input_lock:
                    self._resupport_emitted_ns=None;self._resupport_emitted_step=None
                    self._accepted_ack_ns=None;self.ack_ns=None
                reannounce=timing['resupport_window_open_s']<=t
        # An acknowledgment racing this cycle's begin timestamp belongs to the
        # next cycle. More importantly, the cue must have finished displaying in
        # a previous cycle, and the operator event must be strictly later than
        # that actual timestamp. Scheduled time alone is never readiness evidence.
        with self._input_lock:
            ack_stamp=self.ack_ns
            cue_stamp=self._resupport_emitted_ns
            visible_before=(cue_stamp is not None and self._resupport_emitted_step<self._step_index)
            ack=(ack_stamp-self.started_ns)/1e9 if (visible_before and ack_stamp is not None and
                                                  cue_stamp<ack_stamp<=now_ns) else None
        decision=self.timeline.step(t,resupport_ack_s=ack)
        if decision.emergency_stop and self.cancel:self.cancel()
        if decision.resupport_ack_accepted and self._accepted_ack_ns is None:
            if ack is None:raise RuntimeError('Timeline accepted an acknowledgement without visible-cue evidence')
            self._accepted_ack_ns=ack_stamp
            self.events.append({'key':'OPERATOR_RESUPPORT_ACK','monotonic_ns':ack_stamp,
                                'cue_emitted_ns':cue_stamp,'accepted_ns':now_ns,
                                'cue_step':self._resupport_emitted_step,'accepted_step':self._step_index})
        cues=list(decision.cues)
        if reannounce and not any(c.key=='resupport_window_open' for c in cues):
            cues.extend(c for c in self.timeline.cues_between(None,t) if c.key=='resupport_window_open')
        for cue in cues:
            label=cue.label
            if cue.key=='initial_hold':label='初期保持。胴体を支持したまま待ってください。'
            elif cue.key=='active_window_open':
                label={'supported_stance':'支持を残して四足の接地・滑りを観察します。',
                       'partial_load':'落下受けを維持し、支える力を少しだけ緩めます。',
                       'stand':'落下受けを維持し、胴体の支持をゆっくり減らして静止立位を観察します。',
                       'walk':'落下受けと遮断担当を維持。短い前進指令を開始します。'}[self.plan['stage']]
            elif cue.key=='active_window_close':label='速度指令はゼロです。実物の停止を確認し、胴体の支えを準備してください。'
            elif cue.key=='resupport_window_open':label='胴体を再び全支持してください。実際に支えてからEnterで確認。落下受けは維持します。'
            elif cue.key=='normal_stop_deadline':label='再支持を照合し、保持力を下げて停止します。未確認なら異常停止します。'
            elif cue.key=='active_window_skipped':label='終了要求により荷重・歩行区間を省略しました。胴体を支持してください。'
            self.emit(cue.key+' '+label)
            with self._input_lock:
                if cue.key=='resupport_window_open':
                    # A key buffered before the displayed request is not a fresh
                    # confirmation. Use the fd directly; no TextIO buffer remains.
                    try:self._flush_stale_input()
                    except Exception as error:
                        if self._reader_failed is None:
                            self._reader_failure('TERMINAL_FLUSH_FAILED',type(error).__name__)
                        raise RuntimeError('Cannot establish fresh operator-input boundary') from error
                    self.ack_ns=None;self._accepted_ack_ns=None
                emitted_ns=self.clock()
                self.events.append({'key':cue.key,'label':cue.label,'scheduled_elapsed_s':cue.time_s,
                                    'scheduled_monotonic_ns':self.started_ns+round(cue.time_s*1e9),
                                    'monotonic_ns':now_ns,'emitted_ns':emitted_ns,
                                    'emitted_step':self._step_index,'display_text':label})
                if cue.key=='resupport_window_open':
                    self._resupport_emitted_ns=emitted_ns
                    self._resupport_emitted_step=self._step_index
        if decision.emergency_stop:
            raise RuntimeError(decision.reason)
        return decision.request_normal_stop

    def decorate_report(self,report):
        self._closed.set()
        if self._reader:self._reader.join(timeout=.2)
        reader_alive=self._reader is not None and self._reader.is_alive()
        if reader_alive:
            # Do not switch a still-running reader back to potentially blocking
            # I/O. Output has already stopped; retain this explicit cleanup fact.
            self.events.append({'key':'TERMINAL_READER_NOT_EXITED','monotonic_ns':self.clock()})
        if not reader_alive and self._input_fd is not None and self._input_was_blocking is not None:
            try:os.set_blocking(self._input_fd,self._input_was_blocking)
            except OSError:
                self.events.append({'key':'TERMINAL_MODE_RESTORE_FAILED','monotonic_ns':self.clock()})
        report=dict(report)
        report['scope']='bounded_ground_characterization'
        status=('COMPLETE_BOUNDED_GROUND_TRIAL' if report['status']=='COMPLETE_SUPPORTED_OUTPUT'
                else report['status'])
        return {'schema':'singularitydog.ground-trial-report.v1','status':status,
            'execution_kind':self.execution_kind,'simulation_only':self.execution_kind!='hardware',
            'stage':self.plan['stage'],'trial_id':self.trial_id,'scope':'bounded_ground_characterization',
            'stage_plan_sha256':self.plan_sha256,'profile_sha256':self.plan['base_profile_sha256'],
            'assembly_id':self.plan['assembly_id'],'boot_id':self.plan['boot_id'],
            'motor_power_epoch':self.plan['motor_power_epoch'],'errors':report.get('errors',[]),
            'motor_enable_sent':report.get('motor_enable_sent',False),
            'learned_targets_sent':report.get('learned_targets_sent',False),
            'stop_confirmed':report.get('stop_confirmed',False),
            'deadline20ms_misses':report.get('deadline20ms_misses'),
            'transport_settings':report.get('transport_settings'),
            'telemetry_cadence':report.get('telemetry_cadence'),
            'actual_policy_output_20ms_verified':False,
            'physical_result':'UNREVIEWED','dependency_eligible':False,
            'early_stop_requested':self._normal_requested,
            'planned_trajectory':self.plan['trajectory'],
            'effective_timing':self.timeline.timing,
            'resupport_ack':{'monotonic_ns':self._accepted_ack_ns,'last_received_ns':self.ack_ns,
                'accepted_elapsed_s':self.timeline.resupport_ack_s},
            'cues':list(self.events),'runtime_report':report}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--ground-plan',type=Path,required=True)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--execute-ground',action='store_true')
    parser.add_argument('--catch-ready',action='store_true')
    parser.add_argument('--video-ready',action='store_true')
    parser.add_argument('--roles-ready',action='store_true')
    parser.add_argument('--trial-id')
    # Keep a finite whitelist; arbitrary/abbreviated arguments must never be
    # forwarded into a different entry point with its own arming semantics.
    flags=('support-in-place','cutoff-ready')
    values=('front-port','rear-port','library','output','audio','audio-sha256','audio-device','power-epoch')
    for name in flags:parser.add_argument('--'+name,action='store_true')
    for name in values:parser.add_argument('--'+name)
    add_transport_arguments(parser)
    args=parser.parse_args(argv)
    rest=['--'+name for name in flags if getattr(args,name.replace('-','_'))]
    for name in values:
        value=getattr(args,name.replace('-','_'))
        if value is not None:rest.extend(('--'+name,value))
    for name in ('request-gap-us','request-window'):
        value=getattr(args,name.replace('-','_'))
        if value is not None:rest.extend(('--'+name,str(value)))
    from .ground_trial_plan import validate_ground_plan
    raw,plan=read_document(args.ground_plan)
    profile=load_profile(args.profile,require_approved=args.execute_ground)
    try:
        pacing=transport_settings(profile,request_gap_us=args.request_gap_us,request_window=args.request_window)
    except ProfileError as error:
        parser.error(str(error))
    priors={}
    for stage,artifact in plan.get('prior_evaluations',{}).items():
        if artifact and artifact.get('path'):
            priors[stage]=read_document(args.ground_plan.parent/artifact['path'])[0]
    valid=validate_ground_plan(plan,profile,priors,require_approved=args.execute_ground)
    if not args.execute_ground:
        print(json.dumps({'status':'PLAN_ONLY','stage':plan['stage'],'output_allowed':False,
            'hardware_opened':False,'execution_plan_validated':valid.get('execution_plan_validated',False),
            'blockers':valid.get('blockers',[]),'stage_plan_sha256':sha(raw),
            'transport_settings':pacing,'telemetry_cadence':telemetry_settings(profile),
            'actual_policy_output_20ms_verified':False},ensure_ascii=False,indent=2))
        return 0
    if not args.video_ready or not args.roles_ready:parser.error('Current recording and operator roles must be ready')
    if valid['stage']!='supported_stance' and not args.catch_ready:
        parser.error('An independent full-body catch or separate helper must be ready')
    if not sys.stdin.isatty():parser.error('Run locally in the visible Jetson terminal for fresh re-support acknowledgment')
    execution=GroundTrialExecution(valid,sha(raw),trial_id=args.trial_id,input_stream=sys.stdin)
    from .policy_output import main as output_main
    return output_main(['--profile',str(args.profile),'--execute-supported',*rest],execution=execution)


if __name__=='__main__':raise SystemExit(main())
