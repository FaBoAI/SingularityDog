"""Supported, finite learned-policy output coordinator.

This module does not open devices. Each bus has a single executor/FD owner.
During active operation Type1 replies provide telemetry; STOP is never used as
a polling instruction. An independent host timer cancels acquisition and queues
STOP on both owners if a model/IMU call stalls. Process/kernel failure still
requires the independently tested actuator watchdog and physical support.
"""
from concurrent.futures import FIRST_EXCEPTION, Future, ThreadPoolExecutor, wait
from dataclasses import asdict, replace
import gc
from types import MappingProxyType
import math
import os
import threading
import time

from . import can_readonly as codec
from . import rs05_trial_protocol as protocol
from . import motor_version_probe as versions
from . import policy_shadow as shadow
from .policy_live_profile import (SCHEMA_V3, SCALAR_BACKEND, MEASURED_R17_STARTUP_TIMING,
                                  telemetry_settings, validate_cadence_sources,
                                  execution_settings, local_characterization_settings,
                                  post_reply_deadline_settings, current_position_hold_only,
                                  reviewed_startup_cycle_allowance,
                                  fixed_catch_current_hold_settings, supported_preload_settings,
                                  human_supported_partial_current_hold_settings,
                                  prepared_voltage_publication_settings,
                                  native_phase_pair_settings,
                                  native_feedback_batch_decode_settings,
                                  unpaired_output_future_notifications_settings)
from .policy_post_reply_timing import PostReplyDeadlineBudget
from .policy_observer import _TARGET_LOWER, _TARGET_UPPER
from .native_diagnostic_transport import exchange_evidence
from .policy_motion_envelope import AxisLimits, MotionSample, PolicyMotionEnvelope
from .angle_calibration_audit import (AngleEvidenceError, UNKNOWN_EPOCHS,
                                      resolve_unique_numeric_branch)

IDS=tuple(range(1,13))
BUSES={'front':tuple(range(1,7)), 'rear':tuple(range(7,13))}
PERIOD_NS=20_000_000
ABSOLUTE_MIN_START_SEPARATION_NS=15_000_000
V3_VOLTAGE_MAX_AGE_NS=126_000_000
MODEL_TARGET_LIMITS_BY_ID={mid:(lower,upper) for mid,lower,upper in
                          zip(shadow.CAN_ORDER,_TARGET_LOWER,_TARGET_UPPER)}


def need(condition,message):
    if not condition:raise RuntimeError(message)


def _owned_future_readiness_group(deadline_wait,inputs):
    """Optional hint pipe owned by the exact pinned, GIL-releasing waiter.

    A notification cannot replace the current owner Futures or their original
    deadline. Generic callables, older libraries and nonstandard Futures keep
    the existing readiness-poll path.
    """
    from .native_active_transport import _OwnedActiveWaiter
    if type(deadline_wait) is not _OwnedActiveWaiter:return None
    if not deadline_wait.future_readiness_available:return None
    return deadline_wait.readiness_group(inputs)


def checked_voltage_rows(rows,ids,profile,now_ns):
    """Validate a complete set of direct voltage replies before caching it."""
    values={}
    for i in ids:
        need((i,'voltage') in rows,f'ID{i} voltage reply missing')
        reply,started,received=rows[i,'voltage']
        need(0<started<=received<=now_ns,f'ID{i} voltage reply noncausal')
        voltage=reply.get('value')
        need(type(voltage) in (int,float) and math.isfinite(voltage) and
             profile['voltage_min_v']<=voltage<=profile['voltage_max_v'],f'ID{i} voltage outside profile')
        values[i]=(voltage,received)
    return values


def checked_voltage_cache(cache,profile,now_ns):
    """Return the worst age/value, or fail before a V3 Type1 command."""
    need(set(cache)==set(IDS),'All-axis voltage cache incomplete')
    maximum_age=0;minimum_voltage=math.inf
    for i in IDS:
        voltage,received=cache[i]
        need(type(received) is int and 0<received<=now_ns,f'ID{i} voltage timestamp noncausal')
        age=now_ns-received
        need(age<=V3_VOLTAGE_MAX_AGE_NS,f'ID{i} voltage stale')
        need(type(voltage) in (int,float) and math.isfinite(voltage) and
             profile['voltage_min_v']<=voltage<=profile['voltage_max_v'],f'ID{i} voltage outside profile')
        maximum_age=max(maximum_age,age);minimum_voltage=min(minimum_voltage,voltage)
    return maximum_age,minimum_voltage


def _start_interval_metrics(cycles):
    intervals=[right['begin_ns']-left['begin_ns']
               for left,right in zip(cycles,cycles[1:])]
    return {'start_interval_count':len(intervals),
            'start_intervals_over_20ms':sum(value>PERIOD_NS for value in intervals),
            'start_intervals_over_21ms':sum(value>PERIOD_NS+1_000_000 for value in intervals),
            'max_start_interval_ms':max(intervals,default=0)/1e6,
            'strict_start_interval_20ms_met':bool(intervals) and all(value<=PERIOD_NS for value in intervals)}


def _absolute_epoch_slot(epoch_ns,previous_slot,previous_begin_ns,now_ns):
    """Choose one 20 ms slot; never replay missed work in a short burst.

    A late wake may advance to a later epoch slot. Active output rejects that
    skip before another hold command; this helper only identifies the slot.
    """
    if (type(epoch_ns) is not int or type(now_ns) is not int or now_ns<epoch_ns or
            (previous_slot is None)!=(previous_begin_ns is None) or
            (previous_slot is not None and
             (type(previous_slot) is not int or previous_slot<0 or
              type(previous_begin_ns) is not int or previous_begin_ns<epoch_ns))):
        raise ValueError('Invalid active absolute-epoch schedule state')
    minimum_slot=0 if previous_slot is None else previous_slot+1
    elapsed_slot=(now_ns-epoch_ns)//PERIOD_NS
    if previous_begin_ns is None:
        spaced_slot=0
    else:
        earliest=previous_begin_ns+ABSOLUTE_MIN_START_SEPARATION_NS
        spaced_slot=max(0,(earliest-epoch_ns+PERIOD_NS-1)//PERIOD_NS)
    slot=max(minimum_slot,elapsed_slot,spaced_slot)
    return slot,epoch_ns+slot*PERIOD_NS


class _PendingCycleTiming:
    """One reused scalar record; materialize failure evidence only after STOP."""
    _fields=('index','release_ns','begin_ns','previous_candidate_ns','previous_sample_start_ns',
             'hold_checked_ns','combined_acquisition_wait_begin_ns','combined_acquisition_wait_end_ns',
             'combined_acquisition_wait_cpu_begin_ns','combined_acquisition_wait_cpu_end_ns',
             'feedback_collect_begin_ns','feedback_collect_end_ns',
             'imu_wait_begin_ns','imu_wait_end_ns','imu_read_started_ns','imu_read_finished_ns',
             'acquisition_complete_ns','sample_start_ns','policy_call_begin_ns',
             'policy_call_return_ns','target_ready_ns','voltage_owner_validated_ns',
             'voltage_join_complete_ns','voltage_join_cpu_begin_ns','voltage_join_cpu_end_ns','candidate_ns',
             'output_submit_ns','output_join_begin_ns','output_join_ready_ns',
             'output_join_cpu_begin_ns','output_join_cpu_end_ns','output_takeout_end_ns',
             'output_return_ns','cycle_end_ns')
    __slots__=(*_fields,'active','stage')

    def __init__(self):
        self.active=False;self.stage=None
        for name in self._fields:setattr(self,name,None)

    def begin(self,index,release,begun,previous_candidate,previous_sample):
        for name in self._fields:setattr(self,name,None)
        self.index=index;self.release_ns=release;self.begin_ns=begun
        self.previous_candidate_ns=previous_candidate;self.previous_sample_start_ns=previous_sample
        self.stage='before_feedback_hold';self.active=True

    def snapshot(self,profile):
        result={name:getattr(self,name) for name in self._fields}
        result.update(schema='singularitydog.failed-policy-cycle-timing.v1',stage=self.stage,
            gap_limit_ms=profile.get('max_sample_gap_ms',profile['hard_cycle_ms']),
            sample_age_limit_ms=profile['max_sample_age_ms'],hard_cycle_limit_ms=profile['hard_cycle_ms'],
            command_gap_basis='validated_target_computation_not_transport_write')
        for name,new,old in (
                ('command_interval_ms',self.candidate_ns,self.previous_candidate_ns),
                ('sample_interval_ms',self.sample_start_ns,self.previous_sample_start_ns),
                ('candidate_sample_age_ms',self.candidate_ns,self.sample_start_ns),
                ('policy_call_ms',self.policy_call_return_ns,self.policy_call_begin_ns),
                ('acquisition_join_cpu_ms',self.combined_acquisition_wait_cpu_end_ns,self.combined_acquisition_wait_cpu_begin_ns),
                ('voltage_join_ms',self.voltage_join_complete_ns,self.target_ready_ns),
                ('voltage_join_cpu_ms',self.voltage_join_cpu_end_ns,self.voltage_join_cpu_begin_ns),
                ('voltage_owner_to_join_ms',self.voltage_join_complete_ns,self.voltage_owner_validated_ns),
                ('output_join_ms',self.output_takeout_end_ns,self.output_join_begin_ns),
                ('output_join_cpu_ms',self.output_join_cpu_end_ns,self.output_join_cpu_begin_ns)):
            result[name]=None if new is None or old is None else (new-old)/1e6
        return result


class _DeferredCycleEvidence:
    """Retain raw cycle values; build diagnostic dictionaries only after STOP.

    The live coordinator still validates every reply and applies its timing
    budget. Formatting and floating-point unit conversions are not required
    to decide whether another command may run.
    """
    __slots__=('index','phase','begin_ns','release_ns','previous_release_ns',
        'cadence_slot','scheduled_release_ns','acquired_ns','first_ns',
        'computed_ns','policy_computed_ns','encoded_ns','weight','command','imu',
        'imu_body','voltage_validated_ns','voltage_overlap',
        'acquisition_cpu_begin_ns','acquisition_cpu_end_ns',
        'voltage_cpu_begin_ns','voltage_cpu_end_ns','output_reply_end_ns',
        'output_exchange_return_ns','output_join_begin_ns','output_join_ready_ns',
        'output_takeout_end_ns','output_join_deadline_ns','output_native_deadline_ns',
        'output_cpu_begin_ns','output_cpu_end_ns','final_write_ns','feedback',
        'end_ns','deadline20ms_missed','steady_deadline20ms_missed',
        'startup_20ms_allowance_used','post_reply_deadline')

    def __init__(self,index,command,begun,release,previous_release,slot,
                 absolute_epoch_cadence,acquired,first,computed,policy_computed,
                 encoded,weight,imu,imu_body,voltage_validated_ns,voltage_overlap,timing):
        self.index=index;self.phase=command.phase;self.command=command
        self.begin_ns=begun;self.release_ns=release
        self.previous_release_ns=previous_release;self.cadence_slot=slot
        self.scheduled_release_ns=release if absolute_epoch_cadence else None
        self.acquired_ns=acquired;self.first_ns=first;self.computed_ns=computed
        self.policy_computed_ns=policy_computed;self.encoded_ns=encoded
        self.weight=weight;self.imu=imu;self.imu_body=imu_body
        self.voltage_validated_ns=voltage_validated_ns;self.voltage_overlap=voltage_overlap
        self.acquisition_cpu_begin_ns=timing.combined_acquisition_wait_cpu_begin_ns
        self.acquisition_cpu_end_ns=timing.combined_acquisition_wait_cpu_end_ns
        self.voltage_cpu_begin_ns=timing.voltage_join_cpu_begin_ns
        self.voltage_cpu_end_ns=timing.voltage_join_cpu_end_ns
        self.startup_20ms_allowance_used=False;self.post_reply_deadline=None

    def materialize(self):
        """Called after bus owners and IMU worker have stopped, never in a tick."""
        result={'index':self.index,'phase':self.phase,'begin_ns':self.begin_ns,
            'output_reply_end_ns':self.output_reply_end_ns,
            'output_exchange_return_ns':self.output_exchange_return_ns,
            'output_join_begin_ns':self.output_join_begin_ns,
            'output_join_ready_ns':self.output_join_ready_ns,
            'output_takeout_end_ns':self.output_takeout_end_ns,
            'output_join_deadline_ns':self.output_join_deadline_ns,
            'output_native_deadline_ns':self.output_native_deadline_ns,
            'output_join_cpu_ms':None if self.output_cpu_begin_ns is None else
                (self.output_cpu_end_ns-self.output_cpu_begin_ns)/1e6,
            'oldest_input_to_final_host_write_ms':(self.final_write_ns-self.first_ns)/1e6,
            'feedback':self.feedback,'imu_body':self.imu_body,
            'release_lateness_ms':max(0,self.begin_ns-self.release_ns)/1e6,
            'release_interval_ms':None if self.previous_release_ns is None else
                (self.begin_ns-self.previous_release_ns)/1e6,
            'cadence_slot':self.cadence_slot,'scheduled_release_ns':self.scheduled_release_ns,
            'acquisition_ms':(self.acquired_ns-self.first_ns)/1e6,
            'inference_ms':(self.computed_ns-self.acquired_ns)/1e6,
            'acquisition_join_cpu_ms':None if self.acquisition_cpu_begin_ns is None else
                (self.acquisition_cpu_end_ns-self.acquisition_cpu_begin_ns)/1e6,
            'policy_return_ns':self.policy_computed_ns,
            'overlapped_voltage_validated_ns':self.voltage_validated_ns,
            'voltage_join_ms':(self.computed_ns-self.policy_computed_ns)/1e6 if self.voltage_overlap else 0.,
            'voltage_join_cpu_ms':None if self.voltage_cpu_begin_ns is None else
                (self.voltage_cpu_end_ns-self.voltage_cpu_begin_ns)/1e6,
            'envelope_and_encode_ms':(self.encoded_ns-self.computed_ns)/1e6,
            'policy_and_envelope_ms':(self.encoded_ns-self.acquired_ns)/1e6,
            'effective_policy_weight':self.weight,'command':self.command,'imu':self.imu,
            'end_ns':self.end_ns,'iteration_ms':(self.end_ns-self.begin_ns)/1e6,
            'deadline20ms_missed':self.deadline20ms_missed,
            'post_output_processing_ms':(self.end_ns-self.output_exchange_return_ns)/1e6,
            'startup_20ms_allowance_used':self.startup_20ms_allowance_used,
            'steady_deadline20ms_missed':self.steady_deadline20ms_missed}
        if self.post_reply_deadline is not None:
            result['post_reply_deadline']=self.post_reply_deadline
        return result


def _fixed_record_frame(wire):
    """A native record is already framed; do not re-run stream resynchronization."""
    need(len(wire)==17 and wire[:2]==b'AT' and wire[6]==8 and
         wire[-2:]==b'\r\n' and wire[5]&7==4,'Invalid native frame')
    return codec.Frame(int.from_bytes(wire[2:6],'big') >> 3,4,wire[7:15],wire)


def _python_motion_wires(command,offsets,axes,trial_origin_q,*,encode_motion):
    """Encode and check all twelve quantized targets before publishing a batch.

    Canonical fixed-length bytes from encode_motion need no stream parser or
    Frame allocation. Anything outside that exact framing retains the original
    parser, including its first-frame/error behavior. Keep all twelve encodes
    ahead of the ordered quantized checks: no partially checked batch escapes.
    """
    raws={i:(command.q_model_rad[i-1]-offsets[i])/axes[str(i)]['sign'] for i in IDS}
    result={s:[encode_motion(i,raws[i],command.kp[i-1],command.kd[i-1]) for i in ids]
            for s,ids in BUSES.items()}
    for wires in result.values():
        for wire in wires:
            if (type(wire) is bytes and len(wire)==17 and wire[:2]==b'AT' and
                    wire[6]==8 and wire[-2:]==b'\r\n'):
                i=(int.from_bytes(wire[2:6],'big') >> 3)&255
                raw=int.from_bytes(wire[7:9],'big')*25.14/65535-12.57
            else:
                frame=codec.ATParser().feed(wire)[0];i=frame.destination
                raw=int.from_bytes(frame.data[:2],'big')*25.14/65535-12.57
            a=axes[str(i)]
            q=a['sign']*raw+offsets[i]
            need(a['lower_rad']<=q<=a['upper_rad'],f'ID{i} quantized target outside physical range')
            need(abs(q-trial_origin_q[i-1])<=a['max_displacement_from_start_rad'],
                 f'ID{i} quantized target outside trial displacement')
            estimated=command.kp[i-1]*(q-command.q_model_rad[i-1])+command.estimated_pd_torque_nm[i-1]
            need(abs(estimated)<=a['max_estimated_pd_torque_nm'],f'ID{i} quantized estimated PD torque')
    return result


def decode_records(result,*,feedback_decoder=None,first_id=None):
    records,_=result
    if feedback_decoder is not None:
        need(type(first_id) is int and first_id in (1,7),
             'Native feedback decoder requires an exact bus first ID')
        # The optional decoder accepts only the genuine fixed six-record ABI
        # and supported Type2 feedback transactions. Mixed/invalid records
        # return None, preserving the legacy parser's exact error contracts.
        decoded=feedback_decoder.decode(records,first_id)
        if decoded is not None:return decoded
    rows={}
    for r in records:
        need(r.written==r.received==17 and 0<r.start_ns<=r.finish_ns<=r.received_ns<r.deadline_ns,
             'Incomplete or noncausal motor transaction')
        tx,rx=_fixed_record_frame(bytes(r.tx)),_fixed_record_frame(bytes(r.rx))
        mid=tx.destination
        if tx.kind==4 and tx.data==versions.VERSION_PAYLOAD:
            value=versions.decode_version(rx,mid)
            value.update(request_started_monotonic_ns=r.start_ns,received_monotonic_ns=r.received_ns)
            key=(mid,'version')
        elif tx.kind in (1,3,4,18):
            value=protocol.decode_type2(rx,motor_id=mid)
            key=(mid,'feedback')
        else:
            name='identity' if tx.kind==0 else next((name for name,(idx,_,_) in codec.PARAMETERS.items()
                if tx.data[:2]==idx.to_bytes(2,'little')),None)
            need(name is not None,'Unexpected parameter')
            value=codec.decode_reply(rx,mid,None if name=='identity' else name)
            need(value['ok'],'Parameter or identity rejected')
            key=(mid,name)
        need(key not in rows,'Duplicate transaction')
        rows[key]=(value,r.start_ns,r.received_ns)
    return rows


class BusWorkers:
    """One owner per bus; emergency scheduling prevents subsequent active work."""
    def __init__(self,sessions,cancel_io,clock=time.monotonic_ns,*,before_emergency_stop=None,
                 prepare_voltage_before_feedback_publication=False,native_phase_pair=False,
                 native_feedback_batch_decode=False,native_feedback_codec_selection=None,
                 unpaired_output_future_notifications=False):
        need(type(unpaired_output_future_notifications) is bool,
             'Unpaired output notification selection must be a bool')
        need(not unpaired_output_future_notifications or native_phase_pair is False,
             'Unpaired output notification cannot select a native pair')
        need(set(sessions)==set(BUSES) and sessions['front'] is not sessions['rear'],'Two independent buses required')
        need(type(prepare_voltage_before_feedback_publication) is bool,
             'Prepared voltage publication selection must be a bool')
        if prepare_voltage_before_feedback_publication:
            from .native_active_transport import PREPARED_EXCHANGE_CAPABILITY
            need(all(getattr(session,'prepared_exchange_capability',None) is
                     PREPARED_EXCHANGE_CAPABILITY for session in sessions.values()),
                 'Both active transports must support the exact prepared exchange capability')
        need(type(native_feedback_batch_decode) is bool,
             'Unpaired native feedback codec selection must be a bool')
        need(native_feedback_batch_decode or native_feedback_codec_selection is None,
             'Inactive unpaired codec cannot carry a source selection')
        selected_decoders=None;codec_proof=None
        if native_feedback_batch_decode:
            from .unpaired_native_feedback_codec import prepare_unpaired_decoders
            need(native_phase_pair is False,'Unpaired feedback codec cannot select native phase owners')
            selected_decoders,codec_proof=prepare_unpaired_decoders(sessions,native_feedback_codec_selection)
        self.unpaired_native_feedback_codec_proof=codec_proof
        self.prepare_voltage_before_feedback_publication=prepare_voltage_before_feedback_publication
        self.prepared_voltage_publications=[] if prepare_voltage_before_feedback_publication else None
        self.prepared_voltage_counts={scope:0 for scope in BUSES} if prepare_voltage_before_feedback_publication else None
        self.sessions=sessions;self.cancel_io=cancel_io;self.clock=clock
        self.pools={s:ThreadPoolExecutor(max_workers=1,thread_name_prefix='policy-'+s) for s in BUSES}
        self.lock=threading.RLock();self.aborted=threading.Event();self.reason=None
        self.stop_futures=None;self.journal=[];self.emergency_errors=[]
        self.acquisition_notification_groups=0;self.acquisition_notification_waits=0
        self.voltage_notification_groups=0;self.voltage_notification_waits=0
        self.unpaired_output_future_notifications=unpaired_output_future_notifications
        self.output_notification_groups=0;self.output_notification_waits=0
        self._output_notification_current_futures=None
        self.before_emergency_stop=before_emergency_stop
        need(type(native_phase_pair) is bool,'Native phase pair selection must be a bool')
        self.native_pair=None
        self.native_feedback_decoders=selected_decoders
        if native_phase_pair:
            from .native_active_transport import ActivePhasePair,ActiveSession,NativeFeedbackBatchDecoder
            try:
                self.native_pair=ActivePhasePair(sessions['front'],sessions['rear'])
                # Keep generic sessions and the default route unchanged. The
                # native-pair opt-in already binds the actual active libraries.
                if all(isinstance(session,ActiveSession) for session in sessions.values()):
                    self.native_feedback_decoders={scope:NativeFeedbackBatchDecoder(session.lib)
                        for scope,session in sessions.items()}
            except BaseException as error:
                # A rejected optional ABI must not retain newly started
                # native owners, session bindings, or Python executors.
                cleanup_errors=[]
                if self.native_pair is not None:
                    try:self.native_pair.close()
                    except BaseException as cleanup:cleanup_errors.append(str(cleanup))
                for pool in self.pools.values():
                    try:pool.shutdown(wait=True,cancel_futures=False)
                    except BaseException as cleanup:cleanup_errors.append(str(cleanup))
                if cleanup_errors and hasattr(error,'add_note'):
                    error.add_note('Native codec construction cleanup: '+ '; '.join(cleanup_errors))
                raise

    def _decode_bus_records(self,scope,result):
        decoders=getattr(self,'native_feedback_decoders',None)
        return decode_records(result,feedback_decoder=None if decoders is None else decoders[scope],
                              first_id=BUSES[scope][0])

    def _transform_native_pair_results(self,raw,label,*,decoded,journaled,journal_lock):
        """Decorate stable native slots directly before original publication.

        The native pair has joined both writers before invoking this function.
        It must never call emergency here: a lost enqueue can make the caller
        join this coordinator while holding the STOP lock. Raw evidence is
        journaled once even if that caller subsequently receives an error.
        """
        results={}
        for scope in BUSES:
            result=raw[scope];error=None
            try:
                if isinstance(result,BaseException):raise result
                if decoded:
                    rows=self._decode_bus_records(scope,result)
                    need(all(mid in BUSES[scope] for mid,_ in rows),'Cross-bus response')
                    records=result[0]
                    result=(result,rows,max((r.finish_ns for r in records),default=0),
                            max((r.received_ns for r in records),default=0))
                results[scope]=result
            except BaseException as caught:
                error=caught;results[scope]=caught
            original=raw[scope]
            with journal_lock:
                if scope not in journaled:
                    if isinstance(original,tuple) and len(original)==2:
                        self.journal.append((scope,original,None if error is None else str(error),label))
                        journaled.add(scope)
                    elif hasattr(original,'records') and hasattr(original,'stats'):
                        self.journal.append((scope,(original.records,original.stats),str(original),label))
                        journaled.add(scope)
        return results

    def _submit_native_pair(self,wires,deadline_ns,label,*,decoded=False):
        need(set(wires)==set(BUSES),'Native phase requires both buses')
        need(type(deadline_ns) is int and self.clock()<deadline_ns,
             'Native pair exceeded absolute hard deadline')
        journaled=set();journal_lock=threading.Lock()
        def transform(raw):
            return self._transform_native_pair_results(raw,label,decoded=decoded,
                journaled=journaled,journal_lock=journal_lock)
        try:futures=self.native_pair.submit(wires,deadline_ns=deadline_ns,result_transform=transform)
        except BaseException as error:
            # A Python enqueue can raise after native writers actually ran.
            # The pair settles that generation and attaches only its raw slots.
            partial=getattr(error,'native_pair_bus_results',None) or {}
            with journal_lock:
                for scope,row in partial.items():
                    if scope in journaled:continue
                    if isinstance(row,BaseException) and hasattr(row,'records') and hasattr(row,'stats'):
                        self.journal.append((scope,(row.records,row.stats),str(row),label))
                        journaled.add(scope)
                    elif isinstance(row,tuple) and len(row)==2:
                        self.journal.append((scope,row,str(error),label));journaled.add(scope)
            self.emergency(type(error).__name__+': '+str(error))
            raise
        failure_handed_off=False;handoff_lock=threading.Lock()
        def handoff_failure(done):
            nonlocal failure_handed_off
            if done.cancelled():error=RuntimeError('Native pair publication cancelled')
            else:error=done.exception()
            if error is None:return
            with handoff_lock:
                if failure_handed_off:return
                failure_handed_off=True
            # Error-only work goes to an existing bus pool. Success has no
            # second enqueue or synthetic Future. STOP still joins native
            # writers; this callback never waits for the coordinator lock.
            try:self.pools['front'].submit(self.emergency,type(error).__name__+': '+str(error))
            except BaseException as enqueue_error:
                self.emergency_errors.append({'stage':'native_pair_failure_handoff','bus':None,
                    'error':type(enqueue_error).__name__+': '+str(enqueue_error)})
        for future in futures.values():future.add_done_callback(handoff_failure)
        return futures

    def _exchange(self,scope,wires,timeout_ns=100_000_000,send_only=False,label='preflight',
                  deadline_ns=None,before_native=None):
        need(not self.aborted.is_set(),'Output cancelled')
        try:
            if before_native is not None:
                need(self.prepare_voltage_before_feedback_publication and not send_only and
                     callable(before_native) and type(deadline_ns) is int and self.clock()<deadline_ns,
                     'Prepared exchange requires selected active absolute-deadline voltage I/O')
                result=self.sessions[scope].exchange(wires,deadline_ns=deadline_ns,
                                                     before_native=before_native)
            elif deadline_ns is None:
                result=(self.sessions[scope].send_only if send_only else self.sessions[scope].exchange)(
                    wires,timeout_ns=timeout_ns)
            else:
                need(not send_only and type(deadline_ns) is int and self.clock()<deadline_ns,
                     'Active exchange exceeded absolute hard deadline')
                result=self.sessions[scope].exchange(wires,deadline_ns=deadline_ns)
            self.journal.append((scope,result,None,label))
            return result
        except BaseException as error:
            if hasattr(error,'records') and hasattr(error,'stats'):
                self.journal.append((scope,(error.records,error.stats),str(error),label))
            # Notify the other owner immediately; do not wait for the main
            # thread to collect a slower future on the other bus first.
            self.emergency(type(error).__name__+': '+str(error))
            raise

    def submit(self,wires,*,timeout_ns=100_000_000,send_only=False,label='preflight',
               deadline_ns=None):
        with self.lock:
            need(not self.aborted.is_set(),'Output cancelled before submission')
            if self.native_pair is not None and set(wires)==set(BUSES) and not send_only:
                end=self.clock()+timeout_ns if deadline_ns is None else deadline_ns
                return self._submit_native_pair(wires,end,label)
            return {s:self.pools[s].submit(self._exchange,s,w,timeout_ns,send_only,label,
                                          deadline_ns) for s,w in wires.items()}

    def _exchange_decoded(self,scope,wires,deadline_ns,label):
        # Decode each bus's bounded reply set on its owner while the other bus
        # may still be waiting for I/O. An invalid reply schedules STOP here,
        # before the coordinator can reuse any returned feedback.
        try:
            result=self._exchange(scope,wires,label=label,deadline_ns=deadline_ns)
            rows=self._decode_bus_records(scope,result)
            need(all(mid in BUSES[scope] for mid,_ in rows),'Cross-bus response')
            records=result[0]
            return result,rows,max((r.finish_ns for r in records),default=0),max(
                (r.received_ns for r in records),default=0)
        except BaseException as error:
            self.emergency(type(error).__name__+': '+str(error))
            raise

    def submit_decoded(self,wires,*,deadline_ns,label):
        with self.lock:
            need(not self.aborted.is_set(),'Output cancelled before submission')
            if self.native_pair is not None:
                return self._submit_native_pair(wires,deadline_ns,label,decoded=True)
            if self.unpaired_output_future_notifications:
                previous=self._output_notification_current_futures
                need(previous is None or all(f.done() for f in previous.values()),
                     'Previous original output notification owners are still active')
            originals={s:self.pools[s].submit(self._exchange_decoded,s,w,deadline_ns,label)
                       for s,w in wires.items()}
            if self.unpaired_output_future_notifications:
                self._output_notification_current_futures=MappingProxyType(dict(originals))
            return originals

    def collect(self,futures):
        results={};failure=None
        for s,f in futures.items():
            try:results[s]=f.result()
            except BaseException as e:
                failure=failure or e
                self.emergency(type(e).__name__+': '+str(e))
        if failure:raise failure
        pair=getattr(self,'native_pair',None)
        if pair is not None and pair.owns_futures(futures):
            try:pair.wait_published(futures)
            except BaseException as error:
                self.emergency(type(error).__name__+': '+str(error));raise
        return results

    def collect_output(self,futures,*,deadline_ns,deadline_wait=None,timing=None,
                       native_deadline_ns=None):
        """Join both current decoded replies before taking either result.

        A selected native pair wakes on joined native completion and original
        Future publication, with 200 us ticks (50 us in the last 1 ms). Wake
        notifications are hints: genuine current Futures and both original
        deadlines are checked after each wake. Once both results are ready,
        the metadata publication fence is joined within the remaining original
        host deadline. Older native libraries retain
        the absolute waiter polling path. Neither route guarantees OS wake
        latency or grants another I/O or host completion allowance.
        The explicitly reviewed unpaired notification route is
        default-off. Its private pipe observes the same original Futures and
        includes result takeout and notification cleanup in the original host
        deadline. Capability alone cannot select it; old libraries and generic
        waiters retain the existing bounded readiness polling path.
        The fallback has one finite FIRST_EXCEPTION wait. No previous reply,
        unfinished Future, or late host proof is accepted.
        """
        if timing is not None:
            timing.output_join_cpu_begin_ns=time.thread_time_ns()
            timing.output_join_begin_ns=self.clock()
        notification=None;notification_checked=False
        try:
            need(set(futures)==set(BUSES),'Two-bus output proofs required')
            need(type(deadline_ns) is int and deadline_ns>0,'Integer output join deadline required')
            if native_deadline_ns is None:native_deadline_ns=deadline_ns
            need(type(native_deadline_ns) is int and 0<native_deadline_ns<=deadline_ns,
                 'Native output deadline must be no later than coordinator deadline')
            need(deadline_wait is None or callable(deadline_wait),'Callable native output wait required')
            inputs=tuple(futures.values())
            need(all(isinstance(future,Future) for future in inputs) and
                 len({id(future) for future in inputs})==len(BUSES),
                 'Distinct current output Future owners required')
            current=getattr(self,'_output_notification_current_futures',None)
            if getattr(self,'unpaired_output_future_notifications',False) is True:
                need(current is not None and set(current)==set(BUSES) and
                     all(futures[s] is current[s] for s in BUSES),
                     'Unpaired notification requires current original decoded output Futures')
            pair=getattr(self,'native_pair',None)
            paired=False
            notifier=None
            if pair is not None:
                need(pair.owns_futures(futures),'Native completion requires current original output Futures')
                paired=True
                if getattr(pair,'completion_notification_available',False) is True:
                    notifier=pair.wait_completion
            def ready_failure():
                for future in inputs:
                    if future.cancelled():future.result()
                    if future.done() and future.exception() is not None:future.result()
            while True:
                ready_failure()
                need(not self.aborted.is_set(),self.reason or 'Output aborted during output join')
                now=self.clock()
                if now>=deadline_ns:raise TimeoutError('Output join coordinator deadline')
                all_ready=all(future.done() for future in inputs)
                if all_ready and (not paired or pair.publication_complete(futures)):break
                if all_ready and paired:
                    # Native writers and both genuine results are complete;
                    # only the publisher's metadata fence remains. Join that
                    # Event directly instead of consuming a final hint and
                    # waiting another polling tick. Keep the original end.
                    try:pair.wait_published(futures,timeout=(deadline_ns-now)/1e9)
                    except BaseException:
                        ready_failure();raise
                    continue
                # This explicit default-off selection leaves the paired
                # route and all ordinary run/profile/CLI defaults unchanged.
                # Register only current original Futures and recheck errors,
                # cancellation and the same host deadline after registration.
                if (not paired and
                        getattr(self,'unpaired_output_future_notifications',False) is True and
                        deadline_wait is not None and not notification_checked):
                    try:notification=_owned_future_readiness_group(deadline_wait,inputs)
                    except BaseException:
                        ready_failure();raise
                    notification_checked=True
                    if notification is not None:
                        self.output_notification_groups=getattr(self,'output_notification_groups',0)+1
                        continue
                if notification is not None:
                    self.output_notification_waits=getattr(self,'output_notification_waits',0)+1
                    before=self.clock()
                    try:event=notification.wait(deadline_ns)
                    except BaseException:
                        ready_failure();raise
                    ready_failure()
                    after=self.clock()
                    need(type(event) is dict and event.get('kind') in ('NOTIFIED','DEADLINE') and
                         type(event.get('actual_ns')) is int and
                         before<=event['actual_ns']<=after,
                         'Invalid native unpaired output readiness notification')
                    need(event['kind']!='DEADLINE' or event['actual_ns']>=deadline_ns,
                         'Native unpaired output deadline notification returned early')
                elif notifier is None and deadline_wait is None:
                    _,unfinished=wait(inputs,timeout=(deadline_ns-now)/1e9,return_when=FIRST_EXCEPTION)
                    ready_failure()
                    if unfinished:raise TimeoutError('Output join coordinator deadline')
                    need(all(future.done() for future in inputs),'Incomplete output readiness wait')
                else:
                    # The host join can have a reviewed bookkeeping allowance.
                    # Its final readiness horizon follows the earlier native
                    # I/O deadline, while all rejection clocks remain unchanged.
                    quantum_ns=50_000 if now>=native_deadline_ns-1_000_000 else 200_000
                    wake=min(deadline_ns,now+quantum_ns)
                    try:
                        if notifier is not None:
                            event=notifier(futures,tick_ns=wake,deadline_ns=deadline_ns)
                            need(type(event) is dict and event.get('kind') in ('NOTIFIED','TICK') and
                                 type(event.get('actual_ns')) is int and
                                 now<=event['actual_ns']<=self.clock(),
                                 'Invalid native output completion notification')
                            if event['kind']=='TICK':
                                need(event['actual_ns']>=wake,'Native output completion tick returned early')
                        else:
                            deadline_wait(wake)
                            need(self.clock()>=wake,'Native output wait returned before its deadline')
                    except BaseException:
                        ready_failure()
                        raise
            if timing is not None:timing.output_join_ready_ns=self.clock()
            results=self.collect(futures)
            if timing is not None:timing.output_takeout_end_ns=self.clock()
            need(not self.aborted.is_set(),self.reason or 'Output aborted during output result takeout')
            if self.clock()>=deadline_ns:raise TimeoutError('Output result takeout coordinator deadline')
            if notification is not None:
                notification.close();notification=None
                need(not self.aborted.is_set(),self.reason or 'Output aborted during output readiness cleanup')
                if self.clock()>=deadline_ns:raise TimeoutError('Output readiness cleanup coordinator deadline')
            return results
        except BaseException as error:
            if notification is not None:
                try:notification.close()
                except BaseException as cleanup:
                    if hasattr(error,'add_note'):
                        error.add_note('Output readiness cleanup: '+repr(cleanup))
            self.emergency(type(error).__name__+': '+str(error))
            raise
        finally:
            if timing is not None:
                timing.output_join_cpu_end_ns=time.thread_time_ns()

    def collect_acquisition(self,futures,imu_future,*,deadline_ns,deadline_wait=None,timing=None):
        """Join the current CAN owners and current IMU before result takeout.

        The optional pinned native notification observes these three original
        Futures. A hint never certifies readiness or replaces the unchanged
        absolute deadline, including result takeout and notification cleanup.
        Older native libraries retain bounded 200 us readiness polling; the
        no-callback route retains its one finite FIRST_EXCEPTION wait.
        """
        if timing is not None:
            timing.combined_acquisition_wait_cpu_begin_ns=time.thread_time_ns()
        notification=None;notification_checked=False
        try:
            need(set(futures)==set(BUSES),'Two-bus acquisition required')
            need(type(deadline_ns) is int and deadline_ns>0,'Integer acquisition deadline required')
            need(deadline_wait is None or callable(deadline_wait),'Callable native acquisition wait required')
            inputs=(*futures.values(),imu_future)
            need(all(isinstance(future,Future) for future in inputs),
                 'Current acquisition Future owners required')
            need(len({id(future) for future in inputs})==3,
                 'Distinct CAN and IMU acquisition futures required')
            need(not self.aborted.is_set(),self.reason or 'Output aborted before acquisition wait')
            remaining=deadline_ns-self.clock()
            if remaining<=0:raise TimeoutError('Input acquisition hard deadline')
            # Cancellation before executor notification must not enter wait().
            for future in inputs:
                if future.cancelled():future.result()
            if timing is not None:timing.combined_acquisition_wait_begin_ns=self.clock()
            if deadline_wait is None:
                completed,unfinished=wait(inputs,timeout=remaining/1e9,return_when=FIRST_EXCEPTION)
                for future in inputs:
                    if future.cancelled():future.result()
                    if future in completed and future.exception() is not None:future.result()
                if unfinished or self.clock()>=deadline_ns:
                    raise TimeoutError('Input acquisition hard deadline')
                need(all(future.done() for future in inputs),'Incomplete acquisition readiness wait')
            else:
                def ready_failure():
                    for future in inputs:
                        if future.cancelled():future.result()
                        if future.done() and future.exception() is not None:future.result()
                while True:
                    ready_failure()
                    need(not self.aborted.is_set(),self.reason or 'Output aborted during acquisition wait')
                    now=self.clock()
                    if now>=deadline_ns:raise TimeoutError('Input acquisition hard deadline')
                    if all(future.done() for future in inputs):break
                    if not notification_checked:
                        try:notification=_owned_future_readiness_group(deadline_wait,inputs)
                        except BaseException:
                            ready_failure()
                            raise
                        notification_checked=True
                        if notification is not None:
                            self.acquisition_notification_groups=getattr(self,'acquisition_notification_groups',0)+1
                            # Registering callbacks may complete an owner or
                            # consume the remaining budget. Check again first.
                            continue
                    if notification is not None:
                        self.acquisition_notification_waits=getattr(self,'acquisition_notification_waits',0)+1
                        before=self.clock()
                        try:event=notification.wait(deadline_ns)
                        except BaseException:
                            ready_failure()
                            raise
                        # An original owner failure has priority over a bad
                        # hint, cancellation wake, or late hint timestamp.
                        ready_failure()
                        after=self.clock()
                        need(type(event) is dict and event.get('kind') in ('NOTIFIED','DEADLINE') and
                             type(event.get('actual_ns')) is int and
                             before<=event['actual_ns']<=after,
                             'Invalid native acquisition readiness notification')
                        need(event['kind']!='DEADLINE' or event['actual_ns']>=deadline_ns,
                             'Native acquisition deadline notification returned early')
                    else:
                        wake=min(deadline_ns,now+200_000)
                        try:deadline_wait(wake)
                        except BaseException:
                            ready_failure()
                            raise
                        need(self.clock()>=wake,'Native acquisition wait returned before its deadline')
            need(not self.aborted.is_set(),self.reason or 'Output aborted during acquisition')
            if timing is not None:
                timing.combined_acquisition_wait_end_ns=self.clock()
                timing.feedback_collect_begin_ns=self.clock()
            results=self.collect(futures)
            if timing is not None:
                timing.feedback_collect_end_ns=self.clock()
                timing.imu_wait_begin_ns=self.clock()
            imu_value=imu_future.result()
            if timing is not None:timing.imu_wait_end_ns=self.clock()
            # Result takeout and merger hooks are work, even for ready Futures.
            # A late/cancelled result must not reach input validation or policy.
            need(not self.aborted.is_set(),self.reason or 'Output aborted during acquisition result takeout')
            if self.clock()>=deadline_ns:raise TimeoutError('Input acquisition result takeout hard deadline')
            if notification is not None:
                notification.close();notification=None
                need(not self.aborted.is_set(),self.reason or 'Output aborted during acquisition readiness cleanup')
                if self.clock()>=deadline_ns:raise TimeoutError('Input acquisition readiness cleanup hard deadline')
            return results,imu_value
        except BaseException as error:
            if notification is not None:
                try:notification.close()
                except BaseException as cleanup:
                    if hasattr(error,'add_note'):
                        error.add_note('Acquisition readiness cleanup: '+repr(cleanup))
            self.emergency(type(error).__name__+': '+str(error))
            raise
        finally:
            if timing is not None:
                timing.combined_acquisition_wait_cpu_end_ns=time.thread_time_ns()

    def collect_voltage(self,futures,*,deadline_ns,deadline_wait=None,timing=None):
        """Join both original owner proofs without sequential blocking result calls.

        The optional pinned native notification wakes on a current original
        Future's callback and checks cancellation. Hints never certify
        readiness: both Futures and the original absolute deadline are checked
        after every wake and after takeout and notification cleanup. Older
        native libraries retain 200 us readiness polling; without a native
        waiter, use one finite FIRST_EXCEPTION wait. No unfinished result,
        previous voltage, or late proof may be reused.
        """
        if timing is not None:timing.voltage_join_cpu_begin_ns=time.thread_time_ns()
        notification=None;notification_checked=False
        try:
            need(set(futures)==set(BUSES),'Two-bus voltage proofs required')
            need(type(deadline_ns) is int,'Integer voltage join deadline required')
            need(deadline_wait is None or callable(deadline_wait),'Callable native voltage wait required')
            inputs=tuple(futures.values())
            need(len(set(inputs))==len(BUSES),'Distinct voltage owner futures required')
            def ready_failure():
                for future in inputs:
                    if future.cancelled():future.result()
                    if future.done() and future.exception() is not None:future.result()
            while True:
                ready_failure()
                need(not self.aborted.is_set(),self.reason or 'Output aborted during voltage join')
                now=self.clock()
                if now>=deadline_ns:raise TimeoutError('Voltage join hard cycle deadline')
                if all(future.done() for future in inputs):break
                if not notification_checked and deadline_wait is not None:
                    try:notification=_owned_future_readiness_group(deadline_wait,inputs)
                    except BaseException:
                        ready_failure()
                        raise
                    notification_checked=True
                    if notification is not None:
                        self.voltage_notification_groups=getattr(self,'voltage_notification_groups',0)+1
                        # Callback registration may finish a Future or consume
                        # the remaining budget. Recheck before the native wait.
                        continue
                if notification is not None:
                    self.voltage_notification_waits=getattr(self,'voltage_notification_waits',0)+1
                    before=self.clock()
                    try:event=notification.wait(deadline_ns)
                    except BaseException:
                        ready_failure()
                        raise
                    # Keep a simultaneous original owner failure ahead of a
                    # cancellation, hint error or late notification timestamp.
                    ready_failure()
                    after=self.clock()
                    need(type(event) is dict and event.get('kind') in ('NOTIFIED','DEADLINE') and
                         type(event.get('actual_ns')) is int and
                         before<=event['actual_ns']<=after,
                         'Invalid native voltage readiness notification')
                    need(event['kind']!='DEADLINE' or event['actual_ns']>=deadline_ns,
                         'Native voltage deadline notification returned early')
                elif deadline_wait is None:
                    _,unfinished=wait(inputs,timeout=(deadline_ns-now)/1e9,return_when=FIRST_EXCEPTION)
                    ready_failure()
                    if unfinished:raise TimeoutError('Voltage join hard cycle deadline')
                    need(all(future.done() for future in inputs),'Incomplete voltage readiness wait')
                else:
                    wake=min(deadline_ns,now+200_000)
                    try:deadline_wait(wake)
                    except BaseException:
                        # Cancellation may have come from a failed owner. Keep
                        # that original error when its Future is already ready.
                        ready_failure()
                        raise
                    need(self.clock()>=wake,'Native voltage wait returned before its deadline')
            results={scope:future.result() for scope,future in futures.items()}
            need(not self.aborted.is_set(),self.reason or 'Output aborted during voltage result takeout')
            if self.clock()>=deadline_ns:raise TimeoutError('Voltage result takeout hard cycle deadline')
            if notification is not None:
                notification.close();notification=None
                need(not self.aborted.is_set(),self.reason or 'Output aborted during voltage readiness cleanup')
                if self.clock()>=deadline_ns:raise TimeoutError('Voltage readiness cleanup hard cycle deadline')
            return results
        except BaseException as error:
            if notification is not None:
                try:notification.close()
                except BaseException as cleanup:
                    if hasattr(error,'add_note'):
                        error.add_note('Voltage readiness cleanup: '+repr(cleanup))
            self.emergency(type(error).__name__+': '+str(error))
            raise
        finally:
            if timing is not None:timing.voltage_join_cpu_end_ns=time.thread_time_ns()

    def _voltage(self,scope,wires,ids,profile,timeout_ns=None,deadline_ns=None,before_native=None):
        """The existing bus owner receives and validates before returning.

        A bad voltage aborts the other owner while inference is still running;
        the main thread must join both proofs before sending another Type1.
        """
        try:
            if before_native is None:
                result=self._exchange(scope,wires,100_000_000 if timeout_ns is None else timeout_ns,
                                      label='overlapped_voltage',deadline_ns=deadline_ns)
            else:
                result=self._exchange(scope,wires,100_000_000 if timeout_ns is None else timeout_ns,
                    label='overlapped_voltage',deadline_ns=deadline_ns,before_native=before_native)
            checked=checked_voltage_rows(self._decode_bus_records(scope,result),ids,profile,self.clock())
            return result,checked,self.clock()
        except BaseException as error:
            self.emergency(type(error).__name__+': '+str(error))
            raise

    def submit_voltage(self,ids_by_bus,profile,*,timeout_ns=None,deadline_ns=None):
        with self.lock:
            need(not self.aborted.is_set(),'Output cancelled before voltage submission')
            return {scope:self.pools[scope].submit(self._voltage,scope,
                [codec.read_request(mid,'voltage')],(mid,),profile,timeout_ns,deadline_ns)
                for scope,mid in ids_by_bus.items()}

    def _feedback_then_voltage(self,scope,wires,mid,profile,deadline_ns,feedback_ready,previous):
        """Precheck this bus's six replies, then start its read-only voltage I/O."""
        publication=None
        try:
            remaining=deadline_ns[0]-self.clock()
            need(remaining>0,'Feedback exceeded hard cycle deadline')
            result=self._exchange(scope,wires,label='feedback_hold',deadline_ns=deadline_ns[0])
            current=self._decode_bus_records(scope,result)
            expected={(axis,'feedback') for axis in BUSES[scope]}
            need(set(current)==expected,'Incomplete or cross-bus feedback response')
            checked_at=self.clock()
            for axis in BUSES[scope]:
                feedback,started,received=current[axis,'feedback']
                need(0<started<=received<=checked_at and
                     checked_at-started<=profile['max_sample_age_ms']*1e6,
                     f'ID{axis} stale feedback before voltage')
                need(feedback.mode_state==2 and feedback.fault_bits==0,
                     f'ID{axis} fault/mode before voltage')
                if previous is not None:
                    old,_,old_end=previous[axis,'feedback']
                    need(received>old_end,f'ID{axis} repeated feedback before voltage')
                    dt=(received-old_end)/1e9
                    need(abs(feedback.protocol_position_rad-old.protocol_position_rad)<=
                         profile['axes'][str(axis)]['max_measured_velocity_rad_s']*dt+.01,
                         f'ID{axis} raw position discontinuity before voltage')
            if self.prepare_voltage_before_feedback_publication:
                submitted_deadline=deadline_ns[0]
                published=False;last_checked_ns=None
                self.prepared_voltage_counts[scope]+=1
                # Allocate audit storage before transport preparation/publication.
                # Reuse the existing cancellation/deadline check clocks below.
                publication={'bus':scope,'bus_cycle_index':self.prepared_voltage_counts[scope]-1,
                    'voltage_motor_id':mid,'feedback_validated_ns':checked_at,
                    'submitted_deadline_ns':submitted_deadline,'effective_deadline_ns':None,
                    'prepared_before_publish_ns':None,'publication_checked_after_ns':None,
                    'voltage_native_begin_ns':None,'voltage_first_request_ns':None,
                    'voltage_validated_ns':None,'status':'PREPARING','error':None}
                self.prepared_voltage_publications.append(publication)
                def checked_deadline():
                    nonlocal last_checked_ns
                    need(not self.aborted.is_set(),'Output cancelled around prepared feedback publication')
                    current_deadline=deadline_ns[0]
                    need(type(current_deadline) is int and 0<current_deadline<=submitted_deadline,
                         'Shared voltage deadline must not extend during preparation')
                    last_checked_ns=self.clock()
                    need(last_checked_ns<current_deadline,
                         'Voltage deadline expired around prepared feedback publication')
                    return current_deadline
                def publish_prepared():
                    nonlocal published
                    checked_deadline()
                    need(not feedback_ready.done(),'Prepared feedback must be published exactly once')
                    publication['prepared_before_publish_ns']=last_checked_ns
                    feedback_ready.set_result((result,current))
                    published=True
                    narrowed=checked_deadline()
                    publication['publication_checked_after_ns']=last_checked_ns
                    publication['effective_deadline_ns']=narrowed
                    return narrowed
                checked_deadline()
                voltage=self._voltage(scope,[codec.read_request(mid,'voltage')],(mid,),profile,
                    deadline_ns=submitted_deadline,before_native=publish_prepared)
                need(published and feedback_ready.done() and not feedback_ready.cancelled() and
                     feedback_ready.exception() is None,
                     'Prepared transport returned without publishing valid feedback')
                raw,_,validated=voltage
                publication.update(voltage_native_begin_ns=raw[1].begin_ns,
                    voltage_first_request_ns=raw[0][0].start_ns,voltage_validated_ns=validated)
                need(publication['feedback_validated_ns']<=publication['prepared_before_publish_ns']<=
                     publication['publication_checked_after_ns']<=publication['voltage_native_begin_ns']<=
                     publication['voltage_first_request_ns']<=validated and
                     all(row.deadline_ns==publication['effective_deadline_ns'] for row in raw[0]),
                     'Prepared publication/native voltage proof is noncausal or has changed deadline')
                publication['status']='VALIDATED'
                return voltage
            feedback_ready.set_result((result,current))
            need(not self.aborted.is_set(),'Output cancelled before voltage read')
            remaining=deadline_ns[0]-self.clock()
            need(remaining>0,'Voltage read exceeded hard cycle or sample-age deadline')
            # A later IMU or full twelve-axis failure may find this read in
            # flight. emergency() cancels it before queuing STOP on this owner.
            return self._voltage(scope,[codec.read_request(mid,'voltage')],(mid,),profile,
                                 deadline_ns=deadline_ns[0])
        except BaseException as error:
            if publication is not None:
                publication.update(status='FAILED',error=type(error).__name__+': '+str(error))
            if not feedback_ready.done():feedback_ready.set_exception(error)
            self.emergency(type(error).__name__+': '+str(error))
            raise

    def submit_feedback_then_voltage(self,wires,ids_by_bus,profile,*,deadline_ns,previous=None):
        """Queue one ordered feedback/voltage task per FD; STOP cancels I/O."""
        with self.lock:
            need(not self.aborted.is_set(),'Output cancelled before feedback submission')
            feedback={scope:Future() for scope in wires}
            voltage={scope:self.pools[scope].submit(self._feedback_then_voltage,scope,
                batch,ids_by_bus[scope],profile,deadline_ns,feedback[scope],previous)
                for scope,batch in wires.items()}
            return feedback,voltage

    def exchange(self,wires,*,timeout_ns=100_000_000,send_only=False,label='preflight'):
        return self.collect(self.submit(wires,timeout_ns=timeout_ns,send_only=send_only,label=label))

    def emergency(self,reason,*,normal_completion=False):
        with self.lock:
            if self.stop_futures is not None:return
            self.reason=str(reason);self.aborted.set()
            # Latch before invoking the external cancellation hook, including
            # reentrant hooks. A cancellation error must not suppress either
            # STOP owner or escape the coordinator's final evidence report.
            self.stop_futures={}
            if not normal_completion and self.before_emergency_stop is not None:
                try:self.before_emergency_stop(self.reason)
                except BaseException as error:
                    self.emergency_errors.append({'stage':'recovery_notification','bus':None,
                        'error':type(error).__name__+': '+str(error)})
            try:self.cancel_io()
            except BaseException as error:
                self.emergency_errors.append({'stage':'cancel_io','bus':None,
                        'error':type(error).__name__+': '+str(error)})
            pair=getattr(self,'native_pair',None)
            pair_joined=True
            if pair is not None:
                try:
                    pair.cancel();pair.wait_idle()
                except BaseException as error:
                    pair_joined=False
                    self.emergency_errors.append({'stage':'native_pair_join','bus':None,
                        'error':type(error).__name__+': '+str(error)})
            for scope in BUSES:
                try:
                    need(pair_joined,'Native owners not joined; concurrent STOP forbidden')
                    session=self.sessions[scope]
                    stop=getattr(session,'emergency_stop_repeated',None)
                    if stop is None:stop=session.emergency_stop
                    # Queued on the same owner: never race a writer on its FD.
                    self.stop_futures[scope]=self.pools[scope].submit(stop)
                except BaseException as error:
                    self.emergency_errors.append({'stage':'stop_submission','bus':scope,
                        'error':type(error).__name__+': '+str(error)})
                    failure=Future();failure.set_exception(error)
                    self.stop_futures[scope]=failure

    def finish_stops(self):
        self.emergency('normal completion',normal_completion=True)
        result={}
        # Both owners run concurrently. Allow the one-second STOP budget plus
        # cancellation/dispatch overhead, with one shared collection deadline.
        # A per-owner one-second wait would race the extended native cleanup.
        deadline=time.monotonic()+1.25
        for s,f in self.stop_futures.items():
            try:result[s]=f.result(timeout=max(0.,deadline-time.monotonic()))
            except BaseException as error:
                result[s]={'confirmed_ids':[],'unconfirmed_ids':list(BUSES[s]),'error':repr(error)}
        return result

    def close(self):
        try:
            if self.native_pair is not None:self.native_pair.close()
        finally:
            for pool in self.pools.values():pool.shutdown(wait=True,cancel_futures=False)


class OutputWatchdog:
    """Host supervision is separate from the model and IMU worker."""
    def __init__(self,workers,timeout_ns,clock=time.monotonic_ns):
        self.workers=workers;self.timeout_ns=timeout_ns;self.clock=clock
        self.lock=threading.Lock();self.deadline=None;self.closed=threading.Event()
        self.thread=threading.Thread(target=self._run,name='policy-output-watchdog',daemon=True)
        self.thread.start()
    def kick(self):
        with self.lock:self.deadline=self.clock()+self.timeout_ns
    def _run(self):
        while not self.closed.wait(.002):
            with self.lock:deadline=self.deadline
            if deadline is not None and self.clock()>=deadline:
                self.workers.emergency('Host output heartbeat expired');return
    def close(self):self.closed.set();self.thread.join()


def _worker_affinities(workers,imu_pool,check):
    """Read each existing I/O owner's mask from that thread, not the caller."""
    if not hasattr(os,'sched_getaffinity'):raise RuntimeError('Thread affinity is unavailable')
    futures={scope:workers.pools[scope].submit(lambda:(threading.get_native_id(),sorted(os.sched_getaffinity(0))))
             for scope in BUSES}
    futures['imu']=imu_pool.submit(lambda:(threading.get_native_id(),sorted(os.sched_getaffinity(0))))
    values={scope:future.result(timeout=1.) for scope,future in futures.items()}
    check()
    need(len({tid for tid,_ in values.values()})==3,'I/O worker threads are not distinct')
    return {scope:{'native_tid':tid,'cpus':mask} for scope,(tid,mask) in values.items()}


def _transition_worker_affinities(workers,imu_pool,originals,target=None):
    """Apply/read back on each same owner; restoration must work after abort.

    This submits only affinity work directly to the existing pools, never motor
    work. A partial setup failure therefore still restores every original owner.
    """
    def transition(scope):
        row={'native_tid':threading.get_native_id(),'cpus':None,'error':None}
        try:
            need(row['native_tid']==originals[scope]['native_tid'],
                 scope+' I/O owner changed during affinity transition')
            wanted=set(originals[scope]['cpus']) if target is None else set(target)
            os.sched_setaffinity(0,wanted)
            row['cpus']=sorted(os.sched_getaffinity(0))
            need(set(row['cpus'])==wanted,scope+' I/O worker affinity readback differs')
        except BaseException as error:row['error']=type(error).__name__+': '+str(error)
        return row
    pools={**workers.pools,'imu':imu_pool};futures={};rows={}
    for scope,pool in pools.items():
        try:futures[scope]=pool.submit(transition,scope)
        except BaseException as error:
            rows[scope]={'native_tid':None,'cpus':None,'error':type(error).__name__+': '+str(error)}
    for scope,future in futures.items():
        try:rows[scope]=future.result(timeout=1.)
        except BaseException as error:
            rows[scope]={'native_tid':None,'cpus':None,'error':type(error).__name__+': '+str(error)}
    return rows


def rows_from_pair(pair):
    rows={}
    for scope,result in pair.items():
        current=decode_records(result)
        need(all(mid in BUSES[scope] for mid,_ in current),'Cross-bus response')
        need(not rows.keys()&current.keys(),'Duplicate cross-bus response')
        rows.update(current)
    return rows


def rows_from_decoded_pair(pair):
    """Combine feedback already decoded by the two independent bus owners."""
    rows={}
    for scope,(_,current) in pair.items():
        need(all(mid in BUSES[scope] for mid,_ in current),'Cross-bus response')
        need(not rows.keys()&current.keys(),'Duplicate cross-bus response')
        rows.update(current)
    return rows


def _read(workers,parameters,*,label='preflight'):
    return workers.exchange({s:[codec.read_request(i,p) for p in parameters for i in ids]
                             for s,ids in BUSES.items()},label=label)


def preflight(workers,profile,*,firmware_evidence=None,local_characterization=None):
    """All static checks and watchdog readback complete before any enable."""
    epoch=profile.get('motor_power_epoch')
    need(type(epoch) is str and epoch.strip() not in UNKNOWN_EPOCHS,
         'Explicit current motor-power epoch required')
    reviewed=profile.get('watchdog_by_id')
    need(type(reviewed) is dict and set(reviewed)=={str(i) for i in IDS},
         'Twelve tested raw firmware fingerprints required')
    for i in IDS:
        row=reviewed[str(i)]
        fingerprint=row.get('version_bytes_hex') if type(row) is dict else None
        need(type(fingerprint) is str and len(fingerprint)==8 and
             all(c in '0123456789abcdef' for c in fingerprint),f'ID{i} tested raw firmware fingerprint missing/invalid')
    rows=rows_from_pair(_read(workers,[None]))
    for i in IDS:need(rows[i,'identity'][0]['mcu_uid_hex']==profile['axes'][str(i)]['uid'],f'ID{i} UID mismatch')
    stopped=rows_from_pair(workers.exchange({s:[protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=i)
        for i in ids] for s,ids in BUSES.items()}))
    for i in IDS:
        f=stopped[i,'feedback'][0]
        need(f.mode_state==0 and f.fault_bits==0,f'ID{i} initial STOP/fault not clear')
    current_versions=rows_from_pair(workers.exchange(
        {s:[versions.version_request(i) for i in ids] for s,ids in BUSES.items()},
        timeout_ns=250_000_000,label='firmware_version'))
    for i in IDS:
        value=current_versions[i,'version'][0]
        if firmware_evidence is not None:
            firmware_evidence[str(i)]={**value,'uid':rows[i,'identity'][0]['mcu_uid_hex'],
                'expected_version_bytes_hex':reviewed[str(i)]['version_bytes_hex'],
                'matches_tested_firmware':value['version_bytes_hex']==reviewed[str(i)]['version_bytes_hex']}
    for i in IDS:
        need(current_versions[i,'version'][0]['version_bytes_hex']==reviewed[str(i)]['version_bytes_hex'],
             f'ID{i} fresh firmware bytes differ from tested watchdog firmware')
    modes=rows_from_pair(_read(workers,['run_mode','voltage']))
    for i in IDS:
        need(modes[i,'run_mode'][0]['value']==0,f'ID{i} MIT mode is not configured')
        need(profile['voltage_min_v']<=modes[i,'voltage'][0]['value']<=profile['voltage_max_v'],f'ID{i} supply voltage')
    watchdog_acks=rows_from_pair(workers.exchange({s:[protocol.watchdog_setup_request(
        phase=protocol.TrialPhase.WATCHDOG_SETUP,motor_id=i) for i in ids] for s,ids in BUSES.items()},
        label='watchdog_setup'))
    for i in IDS:
        f=watchdog_acks[i,'feedback'][0]
        need(f.mode_state==0 and f.fault_bits==0,f'ID{i} watchdog setup acknowledgement')
    timeouts=rows_from_pair(_read(workers,['can_timeout'],label='watchdog_initial_readback'))
    for i in IDS:need(timeouts[i,'can_timeout'][0]['value']==protocol.WATCHDOG_TICKS,f'ID{i} watchdog readback')
    direct=rows_from_pair(_read(workers,['position','velocity']))
    stopped=rows_from_pair(workers.exchange({s:[protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=i)
        for i in ids] for s,ids in BUSES.items()}))
    offsets={};starts={};turns_by_id={}
    for i in IDS:
        a=profile['axes'][str(i)];raw=direct[i,'position'][0]['value'];f=stopped[i,'feedback'][0]
        need(f.mode_state==0 and f.fault_bits==0,f'ID{i} preflight mode/fault')
        # Local characterization has unknown absolute-zero uncertainty. Its
        # independently bounded encoder/numerical margin only resolves the
        # local branch; it is never an absolute-angle accuracy claim.
        margin=(local_characterization['numerical_position_margin_rad']
                if local_characterization is not None else a['uncertainty_rad'])
        # This branch remains fixed for both telemetry and inverse commands.
        try:
            branch=resolve_unique_numeric_branch(
                raw,sign=a['sign'],offset_rad=a['offset_rad'],
                lower_rad=a['physical_lower_rad'],upper_rad=a['physical_upper_rad'],
                uncertainty_rad=margin)
        except (AngleEvidenceError, KeyError) as error:
            raise RuntimeError(f'ID{i} ambiguous/out-of-range initial encoder branch: {error}') from error
        need(abs(raw-f.protocol_position_rad)<=profile.get('type2_position_tolerance_rad',.02),
             f'ID{i} Type17/Type2 branch or scale mismatch')
        need(abs(direct[i,'velocity'][0]['value'])<=a['max_measured_velocity_rad_s'],f'ID{i} initial velocity')
        turns_by_id[i]=branch['turns']
        offsets[i]=a['offset_rad']-a['sign']*branch['turns']*2*math.pi
        starts[i]=f.protocol_position_rad
    return offsets,starts,stopped,turns_by_id


def feedback_sample(rows,profile,offsets,*,now_ns,previous=None,required_mode=2):
    q=[];v=[];tau=[];temp=[]
    axes=profile['axes'];max_age_ns=profile['max_sample_age_ms']*1e6
    oldest=None
    for i in IDS:
        if (i,'feedback') not in rows:raise RuntimeError(f'ID{i} missing feedback')
        f,start,end=rows[i,'feedback'];a=axes[str(i)]
        if not (0<start<=end<=now_ns and now_ns-start<=max_age_ns):
            raise RuntimeError(f'ID{i} stale feedback')
        if not (f.mode_state==required_mode and f.fault_bits==0):
            raise RuntimeError(f'ID{i} fault/mode')
        if previous is not None:
            old,_,oldend=previous[i,'feedback']
            if not end>oldend:raise RuntimeError(f'ID{i} repeated feedback')
            dt=(end-oldend)/1e9
            if not abs(f.protocol_position_rad-old.protocol_position_rad)<=a['max_measured_velocity_rad_s']*dt+.01:
                raise RuntimeError(f'ID{i} raw position discontinuity')
        q.append(a['sign']*f.protocol_position_rad+offsets[i]);v.append(a['sign']*f.velocity_rad_s)
        tau.append(f.torque_nm);temp.append(f.temperature_c)
        if oldest is None or start<oldest:oldest=start
    return MotionSample(tuple(q),tuple(v),tuple(tau),tuple(temp),oldest/1e9)


def validate_measured(sample,profile,*,initial=None):
    """Hard feedback limits before enable, inference, and command reuse."""
    axes=profile['axes']
    for i in IDS:
        a=axes[str(i)];k=i-1
        if not a['lower_rad']<=sample.q_model_rad[k]<=a['upper_rad']:
            raise RuntimeError(f'ID{i} measured joint limit')
        if not abs(sample.torque_nm[k])<=a['max_measured_torque_nm']:
            raise RuntimeError(f'ID{i} measured torque')
        if not abs(sample.velocity_rad_s[k])<=a['max_measured_velocity_rad_s']:
            raise RuntimeError(f'ID{i} measured velocity')
        if not sample.temperature_c[k]<=a['max_temperature_c']:
            raise RuntimeError(f'ID{i} measured temperature')
        if initial is not None:
            if not abs(sample.q_model_rad[k]-initial.q_model_rad[k])<=a['max_displacement_from_start_rad']:
                raise RuntimeError(f'ID{i} measured trial displacement')


def validate_imu_metadata(value,now,profile,*,previous=0):
    begin=value.get('read_started_monotonic_ns');end=value.get('read_finished_monotonic_ns')
    need(type(begin) is int and type(end) is int and 0<begin<=end<=now and begin>previous,
         'Repeated or noncausal IMU sample')
    need(now-begin<=profile['max_sample_age_ms']*1e6,'Stale IMU')
    for key in ('accel_m_s2','gyro_rad_s'):
        vector=value.get(key)
        need(isinstance(vector,(list,tuple)) and len(vector)==3 and
             all(type(v) in (int,float) and math.isfinite(v) for v in vector),'Invalid IMU vector')
    return begin


def run_supported_policy(profile,sessions,imu_read,policy,*,cancel_io,check=lambda:None,
                         announce=lambda:None,stop_requested=None,clock=time.monotonic_ns,sleep=time.sleep,
                         encode_motion=None,supervision=None,startup_model=None,
                         main_thread_cpu=None,pre_cycle_policy_warmup_calls=None,
                         post_pin_policy_prime_calls=None,defer_gc_during_cycles=False,
                         exclude_policy_cpu_from_workers=False,
                         absolute_epoch_cadence=False,deadline_wait=None,
                         active_timer_slack_ns=None,
                         prepare_voltage_before_feedback_publication=False,
                         native_phase_pair=False,native_feedback_batch_decode=False,
                         unpaired_output_future_notifications=False):
    """Requires a validated profile; caller opens/closes owned resources.

    Normal completion ramps down only while supported. Faults bypass ramps and
    try STOP on both buses. The report never treats a lost USB reply as STOP.
    Raw logs are buffered, then returned after owners have stopped.
    """
    need(type(prepare_voltage_before_feedback_publication) is bool,
         'Prepared voltage publication selection must be a bool')
    if encode_motion is None:
        from .native_active_transport import encode_motion
    need(profile.get('output_allowed') is True,'Reviewed supported output profile required')
    prepared_selection=prepared_voltage_publication_settings(profile)
    need(prepared_selection is prepare_voltage_before_feedback_publication,
         'Prepared voltage publication selection differs from reviewed profile')
    need(not prepared_selection or supervision is None,
         'Prepared voltage publication requires the ordinary box-supported runner')
    need(type(native_phase_pair) is bool and
         native_phase_pair_settings(profile) is native_phase_pair,
         'Native phase pair selection differs from reviewed profile')
    need(not native_phase_pair or (supervision is None and main_thread_cpu==4 and
         pre_cycle_policy_warmup_calls==10 and post_pin_policy_prime_calls==10 and
         exclude_policy_cpu_from_workers and active_timer_slack_ns==1000 and
         absolute_epoch_cadence and callable(deadline_wait)),
         'Native phase pair requires the pinned boxed CPU/timer/startup/epoch settings')
    need(type(native_feedback_batch_decode) is bool and
         native_feedback_batch_decode_settings(profile) is native_feedback_batch_decode,
         'Unpaired feedback codec selection differs from the reviewed profile')
    need(not native_feedback_batch_decode or (native_phase_pair is False and supervision is None),
         'Unpaired feedback codec requires the ordinary independent boxed owners')
    need(type(unpaired_output_future_notifications) is bool and
         unpaired_output_future_notifications_settings(profile) is unpaired_output_future_notifications,
         'Unpaired output notification selection differs from the reviewed profile')
    need(not unpaired_output_future_notifications or (native_phase_pair is False and
         supervision is None and main_thread_cpu==4 and pre_cycle_policy_warmup_calls==10 and
         post_pin_policy_prime_calls==10 and exclude_policy_cpu_from_workers and
         active_timer_slack_ns==1000 and absolute_epoch_cadence and callable(deadline_wait)),
         'Unpaired output notifications require pinned ordinary boxed CPU/timer/startup/epoch settings')
    output_notification_source=None
    if unpaired_output_future_notifications:
        from .unpaired_output_future_notifications import prepare_notifications
        output_notification_source=prepare_notifications(sessions,deadline_wait,
            profile.get('_unpaired_output_notification_selection'))
    local_characterization=local_characterization_settings(profile)
    fixed_position_hold=current_position_hold_only(profile)
    preload_settings=supported_preload_settings(profile)
    preload_path=None;preload_bound=None;preload_wire_audit=None
    if preload_settings is not None:
        from .supported_preload_path import (validate_path, RETURN_COMPLETE_S,
            COMMAND_RETURN_TOLERANCE_RAD, MEASURED_RETURN_TOLERANCE_RAD)
        need(supervision is None and absolute_epoch_cadence,
             'Geometric preload requires supported-only absolute-epoch execution')
        need(callable(getattr(policy,'validate_inputs',None)),
             'Geometric preload must retain full model input validation')
        preload_path=validate_path(preload_settings['path'],profile)
        # Descriptive full-path analysis belongs to setup, before live samples
        # and enable. Fresh anchors use check_origin's extrema checks below.
        preload_wire_audit=preload_path.audit_wire_reference()
    fixed_catch=fixed_catch_current_hold_settings(profile)
    human_supported=human_supported_partial_current_hold_settings(profile)
    if human_supported is not None:
        from .human_supported_hold import HumanSupportedHoldExecution
        need(type(supervision) is HumanSupportedHoldExecution,
             'Human-supported current hold requires its exact terminal supervisor')
        need(absolute_epoch_cadence and fixed_position_hold,
             'Human-supported current hold requires current-hold absolute-epoch execution')
        need(callable(supervision.cancel),
             'Human-supported current hold requires connected emergency cancellation')
        # Recheck the loader-bound finite/audio contract before opening workers
        # or sending Enable. An exact but unbound/previously started object
        # cannot postpone this gate until on_start after hardware is armed.
        supervision.bind_profile(profile,active=True)
    else:
        from .human_supported_hold import HumanSupportedHoldExecution
        need(type(supervision) is not HumanSupportedHoldExecution,
             'Human-supported supervisor requires its reviewed dedicated scope')
    if fixed_catch is not None:
        from .fixed_catch_hold import FixedCatchExecution
        need(type(supervision) is FixedCatchExecution,
             'Fixed-catch current hold requires its exact terminal supervisor')
    else:
        need(supervision is None or fixed_catch is None,
             'No fixed catch supervisor outside its reviewed scope')
    need(not fixed_position_hold or callable(getattr(policy,'validate_inputs',None)),
         'Current-position hold must retain full model input validation')
    post_reply_settings=post_reply_deadline_settings(profile)
    need(post_reply_settings is None or supervision is None,
         'Post-reply deadline policy requires the supported-only runner')
    post_reply_budget=None if post_reply_settings is None else PostReplyDeadlineBudget(post_reply_settings)
    need(local_characterization is None or supervision is None or fixed_catch is not None or human_supported is not None,
         'Local characterization requires the supported-only runner')
    r22=(main_thread_cpu is not None or pre_cycle_policy_warmup_calls is not None or
         post_pin_policy_prime_calls is not None)
    need(type(exclude_policy_cpu_from_workers) is bool,
         'I/O worker CPU exclusion selection must be a bool')
    need(not exclude_policy_cpu_from_workers or r22 and
         (supervision is None or fixed_catch is not None or human_supported is not None),
         'I/O worker CPU exclusion requires explicit R22 supported-only output')
    need(r22 or startup_model is None,'Startup model requires an explicit R22 selection')
    need(not r22 or (type(main_thread_cpu) is int and main_thread_cpu==4 and
         type(pre_cycle_policy_warmup_calls) is int and pre_cycle_policy_warmup_calls==10 and
         (post_pin_policy_prime_calls is None or
          type(post_pin_policy_prime_calls) is int and post_pin_policy_prime_calls==10) and
         startup_model is not None and
         (policy is startup_model or getattr(policy,'model',None) is startup_model) and
         all(callable(getattr(startup_model,name,None)) for name in
             ('pre_pin_warmup','post_pin_prime','finish_startup'))),
         'R22 startup requires pre-pin warmup 10, main CPU4 and an explicit startup model')
    need(type(defer_gc_during_cycles) is bool,'GC deferral selection must be a bool')
    need(type(absolute_epoch_cadence) is bool,
         'Absolute-epoch selection must be a bool')
    need(deadline_wait is None or absolute_epoch_cadence and callable(deadline_wait),
         'Native release wait requires absolute-epoch cadence')
    need(active_timer_slack_ns is None or
         type(active_timer_slack_ns) is int and active_timer_slack_ns==1_000,
         'Active timer slack requires an explicit 1000 ns selection')
    if defer_gc_during_cycles:
        need(gc.isenabled(),'Automatic GC is already disabled before supported output')
    cadence=telemetry_settings(profile)
    execution=execution_settings(profile)
    voltage_pipeline=execution['voltage_pipeline']
    need(type(voltage_pipeline) is bool,'Voltage pipeline selection must be a bool')
    need(not voltage_pipeline or
         profile['schema']==SCHEMA_V3 and execution['voltage_overlap'],
         'Voltage pipeline requires V3 voltage overlap')
    need(not prepare_voltage_before_feedback_publication or voltage_pipeline,
         'Prepared feedback publication requires selected V3 voltage pipeline')
    need(human_supported is None or
         not reviewed_startup_cycle_allowance(profile) and post_reply_settings is None,
         'Human-supported current hold must not permit startup or post-reply deadline allowances')
    validate_cadence_sources(profile)
    native_batch_config=profile.get('native_batch_encoder')
    native_batch_module=None;native_batch_encoder=None;native_batch_sources=None
    if native_batch_config is not None:
        need(profile['schema']==SCHEMA_V3 and type(native_batch_config) is dict and
             set(native_batch_config)=={'path','sha256'} and
             type(profile.get('_native_batch_encoder_path')) is str,
             'Reviewed V3 native batch encoder selection required')
        from .native_policy_batch_encode import PINNED_SOURCE_SHA256, load_verified_module
        native_batch_module=load_verified_module(
            profile['_native_batch_encoder_path'],
            expected_binary_sha256=native_batch_config['sha256'])
        native_batch_sources=dict(PINNED_SOURCE_SHA256)
    workers=BusWorkers(sessions,cancel_io,clock,
        before_emergency_stop=None if human_supported is None else supervision.on_abort,
        prepare_voltage_before_feedback_publication=prepare_voltage_before_feedback_publication,
        native_phase_pair=native_phase_pair,native_feedback_batch_decode=native_feedback_batch_decode,
        native_feedback_codec_selection=profile.get('_unpaired_native_feedback_codec_selection')
            if native_feedback_batch_decode else None,
        unpaired_output_future_notifications=unpaired_output_future_notifications)
    watcher=OutputWatchdog(workers,PERIOD_NS+int(profile['hard_cycle_ms']*1e6),clock)
    imu_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='policy-imu')
    from .active_output_timer_slack import ActiveOutputTimerSlack
    timer_slack=ActiveOutputTimerSlack(active_timer_slack_ns)
    stop_requested=stop_requested or threading.Event()
    report={'status':'ABORTED','errors':[],'cycles':[],'motor_enable_sent':False,'motor_enable_attempted':False,
            'learned_targets_sent':False,'learned_targets_attempted':False,
            'scope':profile.get('scope','supported_characterization_only'),
            'full_controller_50Hz_verified':False,'normal_ramp_completed':False,
            'current_position_hold_only':fixed_position_hold,
            'cyclic_inference_skipped':fixed_position_hold or preload_path is not None,
            'output_kind':'geometric_preload' if preload_path is not None else 'current_hold' if fixed_position_hold else 'learned_policy',
            'preload_targets_attempted':False,'preload_targets_sent':False,
            'preload_return_commanded':False,'preload_return_measured':False,
            'preload_return_max_error_rad':None,
            'preload_wire_reference_audit':preload_wire_audit,
            'preload_wire_reference_audit_origin':'reviewed_capture' if preload_path is not None else None,
            'stop_is_physical_torque_cap':False,'firmware_versions_by_id':{},
            'firmware_versions_match_watchdog_review':False,
            'telemetry_cadence':cadence,
            'execution_settings':execution,
            'prepare_voltage_before_feedback_publication':prepare_voltage_before_feedback_publication,
            'native_phase_pair':{'enabled':native_phase_pair,
                'mode':'persistent_dual_owner.v1' if native_phase_pair else None,
                'paired_phases':'ordinary_exchange_and_output',
                'prepared_feedback_voltage_owners':'existing_python_bus_owners',
                'request_count_per_cycle':26,'active_deadlines_unchanged':True,
                'hardware_timing_improvement_proven':False,'owner_settings':None,
                'settings_history':[],'coordinator_settings':None,
                'coordinator_settings_history':[],'last_phase':None},
            'input_acquisition_wait':('native_ready_poll_200us.v1' if deadline_wait is not None
                                       else 'all_inputs_first_exception.v1'),
            'voltage_join_wait':('native_ready_poll_200us.v1' if deadline_wait is not None
                                 else 'all_ready_first_exception.v1'),
            'output_join_wait':('native_ready_poll_200us_tail50us_1ms.v1' if deadline_wait is not None
                                else 'all_ready_first_exception.v1'),
            'absolute_epoch_cadence':absolute_epoch_cadence,
            'native_release_wait':deadline_wait is not None,
            'timer_slack':timer_slack.report,
            'cadence_source_sha256':dict(profile.get('cadence_source_sha256',{})),
            'after_announcement_watchdog_verified':False,
            'after_announcement_watchdog_readback_by_id':{}}
    if workers.native_pair is not None:
        notifications=getattr(workers.native_pair,'completion_notification_available',False) is True
        report['native_phase_pair'].update(result_publication='original_futures_direct_transform.v1',
            completion_notification_available=notifications)
        if notifications:
            report['output_join_wait']='native_pair_completion_200us_tail50us_1ms.v1'
    feedback_decoders=workers.native_feedback_decoders or {}
    feedback_buses=[scope for scope,decoder in feedback_decoders.items() if decoder.available is True]
    report['native_feedback_batch_decode']={'enabled':bool(feedback_buses),
        'selected_buses':feedback_buses,'mode':'six_fixed_type2_feedback_records.v1' if feedback_buses else None,
        'unsupported_or_invalid_uses_legacy_codec':True,'timestamps_and_deadlines_unchanged':True,
        'hardware_timing_improvement_proven':False}
    if native_feedback_batch_decode:
        report['native_feedback_batch_decode'].update(
            scope='ordinary_unpaired_owners.v1',
            source_binding=workers.unpaired_native_feedback_codec_proof)
    if unpaired_output_future_notifications:
        report['output_join_wait']='unpaired_original_future_notification.v1'
    report['unpaired_output_future_notifications']={'enabled':unpaired_output_future_notifications,
        'source_binding':output_notification_source,'original_futures_required':True,
        'absolute_deadlines_unchanged':True,'hardware_timing_improvement_proven':False}
    if local_characterization is not None:
        report['local_characterization']={**local_characterization,
            'raw_policy_target_limits':'learned_model',
            'blended_target_limits':'reviewed_local_physical_envelope'}
    if voltage_pipeline:report['voltage_pipeline']='feedback_then_voltage.fast_v1'
    report['native_batch_encoder']={'enabled':native_batch_module is not None,
        'binary_sha256':None if native_batch_module is None else native_batch_module.binary_sha256,
        'source_sha256':native_batch_sources}
    v3=profile['schema']==SCHEMA_V3
    voltage_overlap=execution['voltage_overlap']
    # The reviewed R17 diagnostic allows one startup iteration up to 21 ms,
    # with replies still due by 20 ms. Keep that choice tied to its exact V3
    # execution route; later active cycles retain the hard 20 ms deadline.
    startup_20ms_allowance=reviewed_startup_cycle_allowance(profile) or (v3 and
        execution['diagnostic_timing_acceptance']==MEASURED_R17_STARTUP_TIMING and
        execution['model_backend']==SCALAR_BACKEND and voltage_overlap and
        profile['hard_cycle_ms']==20 and profile['max_sample_age_ms']<=20 and
        profile['max_consecutive_20ms_misses']==0)
    report['startup_20ms_allowance_enabled']=startup_20ms_allowance
    report['post_reply_deadline_policy']=post_reply_settings
    voltage_cache={}
    if v3:
        report['voltage_guard']={'maximum_age_ms':V3_VOLTAGE_MAX_AGE_NS/1e6,
            'pre_enable_refresh_by_id':{},'after_enable_refresh_by_id':{},'latest_by_id':{},
            'checks_before_type1':0,'maximum_checked_age_ms':0.,
            'minimum_checked_voltage_v':None}
    original_affinity=None;original_worker_masks=None;worker_restore_required=False
    report['worker_affinity']={'enabled':exclude_policy_cpu_from_workers,
        'excluded_cpu':4 if exclude_policy_cpu_from_workers else None,'target_mask':None,
        'workers_before':None,'workers_during':None,'workers_after':None,
        'restored':None,'restore_errors':[]}
    if r22:
        report['main_thread_affinity']={'requested_cpu':4,'before':None,'during':None,
            'worker_masks_before_pin':None,'worker_masks_after_pin':None,'restored':None}
        report['setup_policy_warmup']={'iterations':10,'begin_ns':None,'end_ns':None,
            'complete':False,'position':'after_worker_startup_before_main_thread_affinity'}
        if post_pin_policy_prime_calls is not None:
            report['setup_policy_prime']={'iterations':10,'begin_ns':None,'end_ns':None,
                'complete':False,'reused_input_buffers':True,'policy_reset_after':False,
                'sensor_cycles':0,'motor_writes':0}
    gc_restore_required=False
    if defer_gc_during_cycles:
        report['cycle_gc_defer']={'before_enabled':None,'during_enabled':None,'after_enabled':None,
            'before_threshold':None,'after_threshold':None,'restored':None,'restore_attempts':0,
            'restore_errors':[]}
    previous=None;last_imu=0;pending_timing=_PendingCycleTiming()
    def safety_check():
        check();need(not workers.aborted.is_set(),workers.reason or 'Output aborted')
        need(human_supported is None or supervision.failed is None,
             getattr(supervision,'failed',None) or 'Human-supported supervision failed')
        need(preload_path is None or not stop_requested.is_set(),
             'Geometric preload cancelled; STOP without a forced return')
    def require_voltage_before_type1():
        if not v3:return
        maximum_age,minimum_voltage=checked_voltage_cache(voltage_cache,profile,clock())
        guard=report['voltage_guard']
        guard['checks_before_type1']+=1
        guard['maximum_checked_age_ms']=max(guard['maximum_checked_age_ms'],maximum_age/1e6)
        old=guard['minimum_checked_voltage_v']
        guard['minimum_checked_voltage_v']=minimum_voltage if old is None else min(old,minimum_voltage)
    def read_imu():
        deadline=clock()+int(profile['max_sample_age_ms']*1e6)
        while clock()<deadline:
            safety_check();sample=imu_read()
            if sample is not None:return sample
            sleep(.0005)
        raise TimeoutError('Fresh IMU deadline')
    def wires_for(command,offsets):
        if native_batch_encoder is not None:return native_batch_encoder(command)
        return _python_motion_wires(command,offsets,profile['axes'],trial_origin_sample.q_model_rad,
                                    encode_motion=encode_motion)
    try:
        timer_slack.apply(workers,imu_pool)
        offsets,starts,initial,turns_by_id=preflight(
            workers,profile,firmware_evidence=report['firmware_versions_by_id'],
            local_characterization=local_characterization)
        report['firmware_versions_match_watchdog_review']=True
        report['fixed_offsets_rad_by_id']=offsets;report['initial_raw_rad_by_id']=starts
        report['fixed_branch_turns_by_id']=turns_by_id
        report['fixed_branch_motor_power_epoch']=profile['motor_power_epoch']
        safety_check();announce();safety_check()
        report['announcement_completed_ns']=clock()
        # V3 intentionally removes rotating timeout-parameter drift polling.
        # Recheck all twelve after speech, before obtaining the final pose/IMU.
        # This is not after-STOP parameter verification or an active re-arm.
        if cadence['after_announcement_all_axis_timeout_readback']:
            timeouts=rows_from_pair(_read(workers,['can_timeout'],label='watchdog_pre_enable_readback'))
            for i in IDS:
                value,started,received=timeouts[i,'can_timeout']
                need(value['value']==protocol.WATCHDOG_TICKS,f'ID{i} pre-enable watchdog readback')
                need(started>=report['announcement_completed_ns'],f'ID{i} watchdog read predates announcement')
                report['after_announcement_watchdog_readback_by_id'][str(i)]={
                    'value_ticks':value['value'],'request_started_ns':started,'received_ns':received}
            report['after_announcement_watchdog_verified']=True
            safety_check()
        if r22:
            if not hasattr(os,'sched_getaffinity') or not hasattr(os,'sched_setaffinity'):
                raise RuntimeError('Main-thread affinity is unavailable')
            original_affinity=set(os.sched_getaffinity(0))
            affinity=report['main_thread_affinity']
            affinity['before']=sorted(original_affinity)
            need(4 in original_affinity and len(original_affinity)>=2,
                 'CPU4 unavailable or main thread already pinned')
            need(not exclude_policy_cpu_from_workers or len(original_affinity-{4})>=3,
                 'I/O worker CPU exclusion requires at least three other available CPUs')
            before=_worker_affinities(workers,imu_pool,safety_check)
            affinity['worker_masks_before_pin']=before
            need(all(set(row['cpus'])==original_affinity for row in before.values()),
                 'I/O workers did not start with the full main-thread CPU mask')
            warmup=report['setup_policy_warmup'];warmup['begin_ns']=clock()
            try:
                safety_check();startup_model.pre_pin_warmup();safety_check()
                warmup['complete']=True
            finally:warmup['end_ns']=clock()
            os.sched_setaffinity(0,{4})
            affinity['during']=sorted(os.sched_getaffinity(0))
            need(affinity['during']==[4],'Main-thread CPU4 affinity was not applied')
            after=_worker_affinities(workers,imu_pool,safety_check)
            affinity['worker_masks_after_pin']=after
            need(all(set(row['cpus'])==original_affinity and
                     row['native_tid']==before[scope]['native_tid']
                     for scope,row in after.items()),
                 'I/O worker affinity changed during main-thread pin')
            if exclude_policy_cpu_from_workers:
                state=report['worker_affinity'];original_worker_masks=before
                target=original_affinity-{4};state['target_mask']=sorted(target)
                state['workers_before']=before;worker_restore_required=True
                safety_check()
                state['workers_during']=_transition_worker_affinities(
                    workers,imu_pool,original_worker_masks,target)
                failures=[row['error'] for row in state['workers_during'].values() if row['error']]
                need(not failures,'I/O worker affinity setup failed: '+str(failures))
                safety_check()
                if workers.native_pair is not None:
                    report['native_phase_pair']['owner_settings']=workers.native_pair.configure_owners(
                        sorted(target),timer_slack_ns=active_timer_slack_ns)
                    report['native_phase_pair']['coordinator_settings']=workers.native_pair.coordinator_settings
                    safety_check()
            if post_pin_policy_prime_calls is not None:
                prime=report['setup_policy_prime'];prime['begin_ns']=clock()
                try:
                    safety_check();startup_model.post_pin_prime();safety_check()
                    prime['complete']=True
                finally:prime['end_ns']=clock()
            startup_model.finish_startup();safety_check()
            if post_pin_policy_prime_calls is not None:
                report['setup_policy_prime']['policy_reset_after']=True
        # Speech may take seconds; recapture before enabling, on the same branch.
        current=rows_from_pair(workers.exchange({s:[protocol.stop_request(phase=protocol.TrialPhase.STOP,motor_id=i)
            for i in ids] for s,ids in BUSES.items()},label='pre_enable_pose'))
        for i in IDS:need(abs(current[i,'feedback'][0].protocol_position_rad-starts[i])<=.02,
                          f'ID{i} moved during preparation; recapture required')
        initial_sample=feedback_sample(current,profile,offsets,now_ns=clock(),required_mode=0)
        validate_measured(initial_sample,profile)
        if preload_path is not None:
            preload_path.check_origin(initial_sample.q_model_rad,
                tuple(current[i,'feedback'][0].protocol_position_rad for i in IDS))
        # Keep the displacement origin fixed across enable and zero-gain
        # startup. The later fresh pose may initialize a jump-free hold target,
        # but must never grant a new displacement budget.
        trial_origin_sample=initial_sample
        report['trial_displacement_origin']='final_pre_enable_feedback'
        report['trial_origin_model_rad_by_id']={str(i):trial_origin_sample.q_model_rad[i-1] for i in IDS}
        report['startup_displacement_checks']=[]
        def check_startup_displacement(reply,stage):
            for (i,_),(f,_,_) in reply.items():
                a=profile['axes'][str(i)]
                q=a['sign']*f.protocol_position_rad+offsets[i]
                delta=q-trial_origin_sample.q_model_rad[i-1]
                report['startup_displacement_checks'].append(
                    {'motor_id':i,'stage':stage,'q_model_rad':q,'displacement_rad':delta})
                need(abs(delta)<=a['max_displacement_from_start_rad'],
                     f'ID{i} startup trial displacement from pre-enable origin')
        pre_enable_imu=read_imu();pre_enable_now=clock()
        last_imu=validate_imu_metadata(pre_enable_imu,pre_enable_now,profile)
        if v3:report['pre_enable_imu']=pre_enable_imu
        need(pre_enable_now/1e9-initial_sample.monotonic_s<=profile['max_sample_age_ms']/1000,
             'Initial motor samples became stale during IMU acquisition')
        if hasattr(policy,'validate_inputs'):policy.validate_inputs(initial_sample,pre_enable_imu,pre_enable_now)
        limits=tuple(AxisLimits(**{key:profile['axes'][str(i)][key] for key in AxisLimits.__dataclass_fields__}) for i in IDS)
        max_stop_s=max(a.max_command_velocity_rad_s/a.max_command_acceleration_rad_s2 for a in limits)+profile['stop_duration_s']
        need(profile['startup_duration_s']+profile['policy_ramp_s']+max_stop_s+.04<profile['duration_s'],
             'Duration must include startup, policy ramp, braking and gain ramp')
        if profile.get('start_pose_bounds'):
            for i in IDS:
                lo,hi=profile['start_pose_bounds'][str(i)]
                need(lo<=initial_sample.q_model_rad[i-1]<=hi,f'ID{i} outside reviewed starting posture')
        if v3:
            # The earlier all-axis voltage read predates speech and model warmup.
            # Refresh after final pose/IMU validation, immediately before enable.
            fresh_voltage=rows_from_pair(_read(workers,['voltage'],label='voltage_pre_enable_refresh'))
            voltage_cache.update(checked_voltage_rows(fresh_voltage,IDS,profile,clock()))
            report['voltage_guard']['pre_enable_refresh_by_id']={str(i):{
                'value_v':voltage_cache[i][0],'received_ns':voltage_cache[i][1]} for i in IDS}
            need(clock()/1e9-initial_sample.monotonic_s<=profile['max_sample_age_ms']/1000,
                 'Initial motor samples became stale during voltage refresh')
        # V3 startup follows the successful one-axis command-loss handshake:
        # enable and check zero gain for one axis before touching the next bus.
        # This is a comparison strategy, not a claim that simultaneous enable
        # caused the observed missing Type3 reply. Legacy schemas keep their
        # original paired-bus startup.
        # Enable is startup work, before any positive gains. A cold Type3 reply
        # can take longer than the 20 ms control period. Give it 30 ms in V3,
        # while keeping zero-gain Type1 and every cyclic deadline unchanged.
        # Bound the entire enable sequence, not just each request. No motion
        # request is retried and a failed check prevents the next axis.
        enable_steps=([{scope:ids[index]} for index in range(6)
                      for scope,ids in BUSES.items()] if v3 else
                      [{scope:ids[index] for scope,ids in BUSES.items()}
                       for index in range(6)])
        transition_begin=clock()
        transition_deadline=transition_begin+120_000_000 if v3 else None
        if v3:
            report['zero_gain_enable_transition']={
                'enable_reply_budget_ms':30., 'total_budget_ms':120.,
                'strategy':'serial_axis_enable_then_zero_gain_v1',
                'ordered_axes':[{'bus':scope,'motor_id':mid}
                    for step in enable_steps for scope,mid in step.items()],
                'completed_axes':[], 'current_axis':None, 'current_stage':None,
                'begin_ns':transition_begin,'deadline_ns':transition_deadline,
                'complete':False,'motion_retry_allowed':False}
        def transition_timeout(desired):
            if transition_deadline is None:return desired
            remaining=transition_deadline-clock()
            need(remaining>=1_000_000,'Zero-gain enable sequence deadline exceeded')
            return min(desired,remaining)
        watcher.kick()
        for step in enable_steps:
            safety_check()
            require_voltage_before_type1()
            if v3:
                scope,mid=next(iter(step.items()))
                report['zero_gain_enable_transition'].update(
                    current_axis={'bus':scope,'motor_id':mid},current_stage='enable')
            report['motor_enable_attempted']=True
            reply=rows_from_pair(workers.exchange({s:[protocol.enable_request(phase=protocol.TrialPhase.ENABLE,motor_id=mid)]
                for s,mid in step.items()},timeout_ns=transition_timeout(
                    30_000_000 if v3 else int(profile['hard_cycle_ms']*1e6)),label='startup_enable'))
            for (i,_),(f,_,_) in reply.items():need(f.mode_state in (0,2) and f.fault_bits==0,'Enable transition failed')
            check_startup_displacement(reply,'enable')
            watcher.kick()
            require_voltage_before_type1()
            if v3:report['zero_gain_enable_transition']['current_stage']='zero_gain'
            reply=rows_from_pair(workers.exchange({s:[encode_motion(mid,starts[mid],0.,0.)]
                for s,mid in step.items()},timeout_ns=transition_timeout(
                    int(profile['hard_cycle_ms']*1e6)),label='startup_zero_gain'))
            for (i,_),(f,_,_) in reply.items():need(f.mode_state==2 and f.fault_bits==0,'Zero-gain transition failed')
            check_startup_displacement(reply,'zero_gain')
            if v3:report['zero_gain_enable_transition']['completed_axes'].extend(step.values())
            watcher.kick()
        if v3:
            transition_end=clock()
            need(transition_end<transition_deadline,'Zero-gain enable sequence deadline exceeded')
            report['zero_gain_enable_transition'].update(
                complete=True,end_ns=transition_end,current_axis=None,current_stage=None)
            # Enabling all twelve motors takes much of the first six-cycle
            # voltage rotation's age budget. Renew the complete cache while
            # gains are still zero, before the final all-axis hold establishes
            # the sample/command timestamps for the first active cycle.
            safety_check()
            refreshed=rows_from_pair(_read(workers,['voltage'],label='voltage_after_enable_refresh'))
            voltage_cache.update(checked_voltage_rows(refreshed,IDS,profile,clock()))
            checked_voltage_cache(voltage_cache,profile,clock())
            report['voltage_guard']['after_enable_refresh_by_id']={str(i):{
                'value_v':voltage_cache[i][0],'received_ns':voltage_cache[i][1]} for i in IDS}
        last_wires={s:[encode_motion(i,starts[i],0.,0.) for i in ids] for s,ids in BUSES.items()}
        require_voltage_before_type1()
        fresh_zero=rows_from_pair(workers.exchange(last_wires,timeout_ns=int(profile['hard_cycle_ms']*1e6)))
        initial_sample=feedback_sample(fresh_zero,profile,offsets,now_ns=clock())
        check_startup_displacement(fresh_zero,'all_axis_zero_gain')
        validate_measured(initial_sample,profile,initial=trial_origin_sample)
        if preload_path is not None:
            preload_bound=preload_path.bind(initial_sample.q_model_rad,
                tuple(fresh_zero[i,'feedback'][0].protocol_position_rad for i in IDS))
            report['preload_path_sha256']=preload_settings['path_sha256']
            report['preload_origin']={'strategy':'fresh_feedback_plus_reviewed_capture_deltas',
                'model_rad':preload_bound.initial_model,'raw_rad':preload_bound.initial_raw,
                'anchor_difference_rad':preload_bound.anchor_difference_rad}
        limits=tuple(replace(a,
            lower_rad=max(a.lower_rad,q-a.max_displacement_from_start_rad),
            upper_rad=min(a.upper_rad,q+a.max_displacement_from_start_rad))
            for a,q in zip(limits,trial_origin_sample.q_model_rad))
        last_command_ns=clock();last_sample_ns=min(row[1] for row in fresh_zero.values())
        envelope=PolicyMotionEnvelope(limits,initial_sample,now_s=last_command_ns/1e9,
            startup_duration_s=profile['startup_duration_s'],stop_duration_s=profile['stop_duration_s'],
            startup_damping_duration_s=profile.get('startup_damping_duration_s'),
            max_sample_age_s=profile['max_sample_age_ms']/1000,
            max_sample_gap_s=profile.get('max_sample_gap_ms',profile['hard_cycle_ms'])/1000)
        report['startup_gain_schedule']={'position_duration_s':envelope.startup_duration_s,
            'damping_duration_s':envelope.startup_damping_duration_s}
        axis_profiles=tuple(profile['axes'][str(i)] for i in IDS)
        if native_batch_module is not None:
            native_batch_encoder=native_batch_module.bind(tuple((
                offsets[i],a['sign'],a['lower_rad'],a['upper_rad'],
                trial_origin_sample.q_model_rad[i-1],a['max_displacement_from_start_rad'],
                a['max_estimated_pd_torque_nm'])
                for i,a in zip(IDS,axis_profiles)))
        watcher.kick();previous=fresh_zero
        if defer_gc_during_cycles:
            state=report['cycle_gc_defer']
            state['before_enabled']=gc.isenabled();state['before_threshold']=tuple(gc.get_threshold())
            need(state['before_enabled'],'Automatic GC is already disabled before active cycles')
            gc_restore_required=True
            gc.disable();state['during_enabled']=gc.isenabled()
            need(not state['during_enabled'],'Automatic GC deferral was not applied')
        safety_check()
        start=clock();release=start;stop_started=False;consecutive=0;previous_release=None
        previous_slot=None
        if supervision is not None:supervision.on_start(start)
        max_run_ns=int((profile['duration_s']+.04)*1e9)
        stop_at_s=profile['duration_s']-max_stop_s-.04
        need(preload_path is None or stop_at_s>RETURN_COMPLETE_S,
             'Geometric preload needs return verification before gain-down reserve')
        while clock()-start<max_run_ns:
            safety_check()
            if absolute_epoch_cadence:
                slot,release=_absolute_epoch_slot(start,previous_slot,previous_release,clock())
                need(slot==len(report['cycles']),
                     'Absolute-epoch cycle slot skipped; STOP before another hold')
                if clock()<release:
                    if deadline_wait is None:sleep(max(0,(release-clock())/1e9))
                    else:deadline_wait(release)
                safety_check()
                begun=clock()
                need(begun>=release,
                     'Absolute-epoch release before scheduled slot; STOP before another hold')
                actual_slot,actual_release=_absolute_epoch_slot(
                    start,previous_slot,previous_release,begun)
                need(actual_slot==slot and actual_release==release,
                     'Absolute-epoch release missed its slot; STOP before another hold')
            else:
                if clock()<release:sleep((release-clock())/1e9)
                safety_check();begun=clock()
                slot=None
            hard_end=begun+int(profile['hard_cycle_ms']*1e6)
            pending_timing.begin(len(report['cycles']),release,begun,last_command_ns,last_sample_ns)
            supervised_stop=False
            if supervision is not None:
                supervised_stop=supervision.before_cycle(begun,stop_requested=stop_requested.is_set())
            # Hold last validated command while obtaining fresh feedback.
            cycle=len(report['cycles']);electric_id={s:ids[cycle%6] for s,ids in BUSES.items()}
            acquisition={s:list(last_wires[s])+([] if voltage_overlap else [codec.read_request(electric_id[s],'voltage')])+
                ([codec.read_request(electric_id[s],'can_timeout')]
                 if cadence['timeout_requests_per_bus_per_cycle'] else []) for s in BUSES}
            # A delayed host wake must STOP before refreshing a stale command.
            # Use the same age/gap limits as feedback_sample and envelope.step;
            # cycle-release jitter itself does not introduce another threshold.
            hold_now=clock();gap_ns=profile.get('max_sample_gap_ms',profile['hard_cycle_ms'])*1e6
            pending_timing.hold_checked_ns=hold_now
            need(hold_now-last_command_ns<=gap_ns and hold_now-last_sample_ns<=gap_ns,
                 'Command/sample gap exceeded before feedback hold')
            # Values were checked after receipt and are immutable; only their
            # age changes while waiting. Avoid rebuilding motion input vectors.
            for i in IDS:
                _,old_start,old_end=previous[i,'feedback']
                need(0<old_start<=old_end<=hold_now and
                     hold_now-old_start<=profile['max_sample_age_ms']*1e6,f'ID{i} stale feedback before hold')
            require_voltage_before_type1()
            pending_timing.stage='input_acquisition'
            voltage_deadline_ns=None;voltage_pending=None
            try:
                if voltage_pipeline:
                    voltage_deadline_ns=[hard_end]
                    incoming,voltage_pending=workers.submit_feedback_then_voltage(
                        acquisition,electric_id,profile,deadline_ns=voltage_deadline_ns,
                        previous=previous)
                else:
                    incoming=workers.submit(acquisition,deadline_ns=hard_end,
                                            label='feedback_hold')
                imu_future=imu_pool.submit(read_imu)
                gathered,imu_value=workers.collect_acquisition(
                    incoming,imu_future,deadline_ns=hard_end,deadline_wait=deadline_wait,timing=pending_timing)
                if voltage_pipeline:
                    decoded_feedback=gathered
                    replies={scope:result for scope,(result,_) in decoded_feedback.items()}
                    rows=rows_from_decoded_pair(decoded_feedback)
                else:
                    replies=gathered
                    rows=rows_from_pair(replies)
                safety_check();acquired=clock()
                pending_timing.acquisition_complete_ns=acquired
                # Raw IMU read stamps are diagnostic evidence, not another guard.
                # Validation below retains its existing clock and failure policy.
                if isinstance(imu_value,dict):
                    begin=imu_value.get('read_started_monotonic_ns')
                    end=imu_value.get('read_finished_monotonic_ns')
                    pending_timing.imu_read_started_ns=begin if type(begin) is int else None
                    pending_timing.imu_read_finished_ns=end if type(end) is int else None
                last_imu=validate_imu_metadata(imu_value,acquired,profile,previous=last_imu)
                first=min(last_imu,*(r.start_ns for result in replies.values() for r in result[0]))
                hard_end=min(hard_end,first+int(profile['max_sample_age_ms']*1e6))
                if voltage_pipeline:
                    need(clock()<hard_end,'Feedback exceeded hard cycle or sample-age deadline')
                    voltage_deadline_ns[0]=hard_end
                elif voltage_overlap:
                    voltage_pending=workers.submit_voltage(electric_id,profile,
                        deadline_ns=hard_end)
                sample=feedback_sample(rows,profile,offsets,now_ns=acquired,previous=previous)
                pending_timing.sample_start_ns=min(row[1] for key,row in rows.items() if key[1]=='feedback')
                # Each owner has already checked mode/fault and continuity before
                # its read-only voltage launch. Revalidate all twelve here.
                validate_measured(sample,profile,initial=trial_origin_sample)
                if v3:
                    # Do not send a learned target if any axis has lost voltage
                    # evidence, even though only two axes are queried this cycle.
                    if not voltage_overlap:
                        voltage_cache.update(checked_voltage_rows(rows,electric_id.values(),profile,acquired))
                    checked_voltage_cache(voltage_cache,profile,acquired)
                for s,i in electric_id.items():
                    if not v3:
                        need(profile['voltage_min_v']<=rows[i,'voltage'][0]['value']<=profile['voltage_max_v'],f'ID{i} voltage')
                    if cadence['timeout_parameter_drift_monitored_during_cycles']:
                        need(rows[i,'can_timeout'][0]['value']==protocol.WATCHDOG_TICKS,f'ID{i} watchdog changed')
                if voltage_pipeline:
                    safety_check()
                    need(clock()<hard_end,'Feedback validation exceeded hard cycle or sample-age deadline')
            except BaseException as error:
                if voltage_pipeline:
                    workers.emergency(type(error).__name__+': '+str(error))
                raise
            wants_stop=(supervised_stop if supervision is not None else
                        stop_requested.is_set() or (begun-start)/1e9>=stop_at_s)
            if supervision is not None and (begun-start)/1e9>=stop_at_s and not (wants_stop or stop_started):
                raise RuntimeError('Ground shutdown reserve reached without fresh re-support confirmation')
            if not stop_started and wants_stop:
                if human_supported is not None:supervision.before_stop(emergency=False)
                if preload_bound is not None:
                    need(report['preload_return_commanded'] and report['preload_return_measured'],
                         'Geometric preload return was not verified before gain-down')
                    need(all(abs(q-q0)<=min(MEASURED_RETURN_TOLERANCE_RAD,a['max_tracking_error_rad'])
                             for q,q0,a in zip(sample.q_model_rad,preload_bound.initial_model,axis_profiles)),
                         'Geometric preload fresh measured return outside tolerance')
                envelope.request_stop();stop_started=True
            weight=0.
            if stop_started:
                # No inference during gain-down, but the fresh IMU limits must
                # still hold. LivePolicyModel validates once inside an active
                # inference call, so do not repeat its assembly on those ticks.
                if hasattr(policy,'validate_inputs'):policy.validate_inputs(sample,imu_value,acquired)
                target=None
            else:
                pending_timing.stage='policy_call';pending_timing.policy_call_begin_ns=clock()
                if preload_bound is not None:
                    policy.validate_inputs(sample,imu_value,acquired)
                    target=preload_bound.target_for_slot(slot)
                elif fixed_position_hold:
                    policy.validate_inputs(sample,imu_value,acquired)
                    target=initial_sample.q_model_rad
                else:
                    target=tuple(policy(sample,imu_value,acquired))
                pending_timing.policy_call_return_ns=clock();pending_timing.stage='target_validation'
                need(len(target)==12 and all(type(v) in (int,float) and math.isfinite(v) for v in target),'Invalid learned target')
                for i,q in enumerate(target,1):
                    a=profile['axes'][str(i)]
                    if local_characterization is not None:
                        lower,upper=MODEL_TARGET_LIMITS_BY_ID[i]
                        need(lower<=q<=upper,f'ID{i} learned target outside model range')
                    else:
                        need(a['lower_rad']<=q<=a['upper_rad'],f'ID{i} learned target outside physical range')
                fraction=max(0.,min(1.,((begun-start)/1e9-profile['startup_duration_s'])/profile['policy_ramp_s']))
                weight=profile['policy_weight']*fraction**3*(10.+fraction*(-15.+6.*fraction))
                if preload_bound is None:
                    target=tuple(q0+weight*(q-q0) for q,q0 in zip(target,initial_sample.q_model_rad))
                if local_characterization is not None:
                    for i,q in enumerate(target,1):
                        a=profile['axes'][str(i)]
                        need(a['lower_rad']<=q<=a['upper_rad'],
                             f'ID{i} blended target outside local physical range')
            safety_check();policy_computed=clock()
            pending_timing.target_ready_ns=policy_computed;pending_timing.stage='voltage_join'
            voltage_validated_ns={}
            if voltage_pending is not None:
                validated=workers.collect_voltage(voltage_pending,deadline_ns=hard_end,
                    deadline_wait=deadline_wait,timing=pending_timing)
                pending_timing.voltage_join_complete_ns=clock()
                for scope,(_,values,stamp) in validated.items():
                    voltage_cache.update(values);voltage_validated_ns[scope]=stamp
                    pending_timing.voltage_owner_validated_ns=max(
                        pending_timing.voltage_owner_validated_ns or 0,stamp)
                # No gain/target frame may be sent until both owner proofs have
                # joined and every cached axis remains in bounds and fresh.
                checked_voltage_cache(voltage_cache,profile,clock())
            safety_check();computed=clock()
            pending_timing.candidate_ns=computed;pending_timing.stage='candidate_deadline'
            need(computed<hard_end,'Inference exceeded hard cycle deadline')
            pending_timing.stage='motion_envelope'
            command=envelope.step(target,sample,now_s=computed/1e9)
            pending_timing.stage='target_encoding'
            outgoing=wires_for(command,offsets)
            safety_check();encoded=clock()
            require_voltage_before_type1()
            need(clock()<hard_end,'Encoded command exceeded hard cycle or sample-age deadline')
            # Mark intent before the first write, including a partial transaction.
            report['learned_targets_attempted']|=weight>0
            report['preload_targets_attempted']|=preload_bound is not None and not stop_started
            pending_timing.output_submit_ns=clock();pending_timing.stage='output_exchange'
            output_futures=workers.submit_decoded(outgoing,deadline_ns=hard_end,
                label='graceful_stop' if stop_started else 'preload_output' if preload_bound is not None else 'policy_output' if weight>0 else 'startup_hold')
            # Store immutable inputs while the owners wait. Diagnostic dicts
            # and unit conversions are deferred until every owner has stopped.
            cycle_row=_DeferredCycleEvidence(cycle,command,begun,release,previous_release,
                slot,absolute_epoch_cadence,acquired,first,computed,policy_computed,
                encoded,weight,imu_value,None,
                voltage_validated_ns,voltage_overlap,pending_timing)
            # Native exchanges retain hard_end for every write/reply. Waiting
            # for their already decoded proofs uses only the existing host
            # bookkeeping allowance and checked-input age, never a new budget.
            output_join_deadline=hard_end
            if post_reply_settings is not None or startup_20ms_allowance and cycle==0:
                bookkeeping_ns=(int(post_reply_settings['max_lateness_ms']*1e6)
                                if post_reply_settings is not None else 1_000_000)
                output_join_deadline=min(first+int(profile['max_sample_age_ms']*1e6),
                                         begun+PERIOD_NS+bookkeeping_ns)
            decoded=workers.collect_output(output_futures,deadline_ns=output_join_deadline,
                deadline_wait=deadline_wait,timing=pending_timing,native_deadline_ns=hard_end)
            reply_return=clock()
            pending_timing.output_return_ns=reply_return;pending_timing.stage='output_validation'
            returned={}
            for _,current,_,_ in decoded.values():
                need(not returned.keys()&current.keys(),'Duplicate cross-bus response')
                returned.update(current)
            # Validate returned limits before the next hold command is reused.
            checked=feedback_sample(returned,profile,offsets,now_ns=reply_return,previous=rows)
            for k,a in enumerate(axis_profiles):
                i=k+1
                need(a['lower_rad']<=checked.q_model_rad[k]<=a['upper_rad'],f'ID{i} joint limit')
                need(abs(checked.torque_nm[k])<=a['max_measured_torque_nm'],f'ID{i} torque')
                need(abs(checked.velocity_rad_s[k])<=a['max_measured_velocity_rad_s'],f'ID{i} velocity')
                need(checked.temperature_c[k]<=a['max_temperature_c'],f'ID{i} temperature')
                need(abs(checked.q_model_rad[k]-command.q_model_rad[k])<=a['max_tracking_error_rad'],f'ID{i} tracking error')
                need(abs(checked.q_model_rad[k]-trial_origin_sample.q_model_rad[k])<=a['max_displacement_from_start_rad'],f'ID{i} trial displacement')
                estimated=command.kp[k]*(command.q_model_rad[k]-checked.q_model_rad[k])-command.kd[k]*checked.velocity_rad_s[k]
                need(abs(estimated)<=a['max_estimated_pd_torque_nm'],f'ID{i} estimated PD torque')
            if preload_bound is not None and preload_bound.return_complete:
                command_error=max(abs(q-q0) for q,q0 in zip(command.q_model_rad,preload_bound.initial_model))
                return_error=max(abs(q-q0) for q,q0 in zip(checked.q_model_rad,preload_bound.initial_model))
                report['preload_return_max_error_rad']=return_error
                report['preload_return_commanded']=command_error<=COMMAND_RETURN_TOLERANCE_RAD
                report['preload_return_measured']=all(
                    abs(q-q0)<=min(MEASURED_RETURN_TOLERANCE_RAD,a['max_tracking_error_rad'])
                    for q,q0,a in zip(checked.q_model_rad,preload_bound.initial_model,axis_profiles))
                need(report['preload_return_commanded'],'Geometric preload command did not return')
                need(report['preload_return_measured'],'Geometric preload measured return outside tolerance')
            final_write=max(entry[2] for entry in decoded.values())
            last_reply=max(entry[3] for entry in decoded.values())
            cycle_row.output_reply_end_ns=last_reply
            cycle_row.output_exchange_return_ns=reply_return
            cycle_row.output_join_begin_ns=pending_timing.output_join_begin_ns
            cycle_row.output_join_ready_ns=pending_timing.output_join_ready_ns
            cycle_row.output_takeout_end_ns=pending_timing.output_takeout_end_ns
            cycle_row.output_join_deadline_ns=output_join_deadline
            cycle_row.output_native_deadline_ns=hard_end
            cycle_row.output_cpu_begin_ns=pending_timing.output_join_cpu_begin_ns
            cycle_row.output_cpu_end_ns=pending_timing.output_join_cpu_end_ns
            cycle_row.final_write_ns=final_write;cycle_row.feedback=checked
            cycle_row.imu_body=getattr(policy,'last_validation',None)
            safety_check();report['cycles'].append(cycle_row)
            # Reply receipt is not cycle completion: decoding, limit checks and
            # raw evidence retention above remain measured control work. Only
            # optional report formatting is removed from the active boundary.
            end=clock()
            pending_timing.cycle_end_ns=end;pending_timing.stage='cycle_deadline'
            elapsed=end-begun;miss=elapsed>PERIOD_NS or final_write-first>PERIOD_NS
            startup_cycle=startup_20ms_allowance and cycle==0
            cycle_row.end_ns=end;cycle_row.deadline20ms_missed=miss
            cycle_row.steady_deadline20ms_missed=miss and not startup_cycle
            # Use a fresh clock even on the final zero-gain cycle. It must not
            # become a successful ramp merely because its reply arrived in time.
            if post_reply_budget is not None:
                admission_ns=clock()
                try:
                    decision=post_reply_budget.admit(index=cycle,begin_ns=begun,oldest_input_ns=first,
                        final_write_ns=final_write,last_reply_ns=last_reply,
                        output_sample_start_ns=min(row[1] for row in returned.values()),
                        checked_ns=admission_ns,sample_age_ns=int(profile['max_sample_age_ms']*1e6),
                        startup_allowed=startup_cycle)
                except RuntimeError as error:
                    cycle_row.post_reply_deadline={'accepted':False,'checked_ns':admission_ns,
                        'allowance_used':False,'rejection':str(error)}
                    raise
                cycle_row.post_reply_deadline=decision
                cycle_row.startup_20ms_allowance_used=decision['startup_allowance_used']
                # Include the same fresh admission boundary. No reply, guard,
                # input-age check or miss budget moves out of the active tick.
                end=decision['checked_ns'];pending_timing.cycle_end_ns=end
                miss=end-begun>PERIOD_NS or final_write-first>PERIOD_NS
                cycle_row.end_ns=end;cycle_row.deadline20ms_missed=miss
                cycle_row.steady_deadline20ms_missed=miss and not startup_cycle
            elif startup_cycle:
                # Only post-reply work gets the one-time allowance. The final
                # host write and all twelve replies must still finish within
                # 20 ms, and the oldest input retains its own age deadline.
                need(final_write-begun<=PERIOD_NS and last_reply-begun<=PERIOD_NS and
                     final_write-first<=PERIOD_NS,
                     'Startup output write/reply exceeded 20ms')
                startup_end=min(begun+PERIOD_NS+1_000_000,
                                first+int(profile['max_sample_age_ms']*1e6))
                need(clock()<=startup_end,'Startup output cycle exceeded 21ms or sample-age deadline')
                cycle_row.startup_20ms_allowance_used=miss
            else:
                need(clock()<hard_end,'Output cycle exceeded hard deadline')
            consecutive=consecutive+1 if cycle_row.steady_deadline20ms_missed else 0
            if post_reply_budget is None:
                need(consecutive<=profile['max_consecutive_20ms_misses'],'Consecutive20ms timing misses')
            watcher.kick();previous=returned;last_wires=outgoing;previous_release=begun
            if absolute_epoch_cadence:previous_slot=slot
            last_command_ns=computed;last_sample_ns=pending_timing.sample_start_ns
            pending_timing.active=False
            if fixed_catch is not None:
                supervision.after_cycle_validated(
                    begun,end,command.phase,stop_requested=stop_requested.is_set())
            elif human_supported is not None:
                full_gain=(command.phase=='active' and command.gain_scale==1. and
                    all(kp==axis['kp'] and kd==axis['kd']
                        for kp,kd,axis in zip(command.kp,command.kd,axis_profiles)))
                pending_timing.active=True;pending_timing.stage='human_hold_supervision'
                supervision.after_cycle_validated(begun,end,command.phase,
                    stop_requested=stop_requested.is_set(),full_gain=full_gain)
                safety_check()
                end=clock();pending_timing.cycle_end_ns=end
                cycle_row.end_ns=end
                cycle_row.deadline20ms_missed=end-begun>PERIOD_NS or final_write-first>PERIOD_NS
                cycle_row.steady_deadline20ms_missed=cycle_row.deadline20ms_missed
                need(end<hard_end,'Human-supported cycle supervision exceeded hard deadline')
                pending_timing.active=False
            if command.phase=='stopped':report['normal_ramp_completed']=True;break
            if not absolute_epoch_cadence:release=max(begun+PERIOD_NS,end)
        need(report['normal_ramp_completed'],'Finite run budget expired before normal stop')
        report['status']='COMPLETE_SUPPORTED_OUTPUT'
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error));workers.emergency(report['errors'][-1])
    finally:
        try:
            try:stops=workers.finish_stops()
            except BaseException as error:
                report['errors'].append('STOP collection: '+type(error).__name__+': '+str(error))
                stops={s:{'confirmed_ids':[],'unconfirmed_ids':list(BUSES[s]),'error':repr(error)} for s in BUSES}
            try:watcher.close()
            except BaseException as error:
                report['errors'].append('Host watchdog close: '+repr(error))
                if report['status']=='COMPLETE_SUPPORTED_OUTPUT':report['status']='ABORTED_WATCHDOG_CLOSE'
        finally:
            try:
                if gc_restore_required:
                    state=report['cycle_gc_defer']
                    for attempt in range(3):
                        state['restore_attempts']=attempt+1
                        try:gc.set_threshold(*state['before_threshold'])
                        except BaseException as error:state['restore_errors'].append('threshold: '+repr(error))
                        try:gc.enable()
                        except BaseException as error:state['restore_errors'].append('enabled state: '+repr(error))
                        try:
                            state['after_enabled']=gc.isenabled();state['after_threshold']=tuple(gc.get_threshold())
                            state['restored']=(state['after_enabled'] is True and
                                state['after_threshold']==state['before_threshold'])
                        except BaseException as error:
                            state['restore_errors'].append('readback: '+repr(error));state['restored']=False
                        if state['restored']:break
                    if not state['restored']:
                        report['errors'].append('Automatic GC state/threshold restoration unconfirmed')
                        if report['status']=='COMPLETE_SUPPORTED_OUTPUT':report['status']='ABORTED_GC_RESTORE'
            finally:
                if worker_restore_required:
                    state=report['worker_affinity']
                    try:
                        state['workers_after']=_transition_worker_affinities(
                            workers,imu_pool,original_worker_masks)
                        state['restore_errors']=[row['error'] for row in state['workers_after'].values()
                                                 if row['error']]
                        state['restored']=not state['restore_errors']
                    except BaseException as error:
                        state['restored']=False;state['restore_errors'].append(type(error).__name__+': '+str(error))
                    if not state['restored']:
                        report['errors'].append('I/O worker affinity restoration unconfirmed: '+str(state['restore_errors']))
                        if report['status']=='COMPLETE_SUPPORTED_OUTPUT':report['status']='ABORTED_WORKER_AFFINITY_RESTORE'
                if original_affinity is not None:
                    affinity=report['main_thread_affinity']
                    try:
                        os.sched_setaffinity(0,original_affinity)
                        affinity['restored']=set(os.sched_getaffinity(0))==original_affinity
                        need(affinity['restored'],'Main-thread CPU affinity restoration differs')
                    except BaseException as error:
                        affinity['restored']=False
                        report['errors'].append(type(error).__name__+': '+str(error))
                        if report['status']=='COMPLETE_SUPPORTED_OUTPUT':report['status']='ABORTED_AFFINITY_RESTORE'
        report['stop_dispatch_errors']=list(workers.emergency_errors)
        if workers.emergency_errors:
            report['errors'].extend('STOP dispatch '+row['stage']+
                (' '+row['bus'] if row['bus'] is not None else '')+': '+row['error']
                for row in workers.emergency_errors)
            if report['status']=='COMPLETE_SUPPORTED_OUTPUT':report['status']='ABORTED_STOP_DISPATCH'
        report['stop_confirmed']=all(set(stops[s].get('confirmed_ids',[]))==set(BUSES[s]) and
            stops[s].get('complete',True) is True and not stops[s].get('unconfirmed_ids') and
            not stops[s].get('ambiguous_ids') for s in BUSES)
        report['stop_reports']={s:{k:v for k,v in stops[s].items() if k not in ('records','stats')} for s in BUSES}
        stop_faults={str(i):bits for s in BUSES for i,bits in stops[s].get('fault_by_id',{}).items() if bits}
        report['stop_faults_by_id']=stop_faults
        if stop_faults:
            report['errors'].append('Fault bits remain in STOP replies')
            if report['status']=='COMPLETE_SUPPORTED_OUTPUT':report['status']='ABORTED_STOP_FAULT'
        if not report['stop_confirmed']:
            report['status']='STOP_UNCONFIRMED_POWER_OFF_REQUIRED';report['errors'].append('All-axis STOP not confirmed; physical power cutoff required')
        try:timer_slack.restore()
        except BaseException as error:
            report['errors'].append('I/O worker timer slack restoration unconfirmed: '+repr(error))
            if report['status']=='COMPLETE_SUPPORTED_OUTPUT':
                report['status']='ABORTED_TIMER_SLACK_RESTORE'
        try:workers.close()
        except BaseException as error:
            report['errors'].append('Native/bus owner close: '+repr(error))
            if report['status']=='COMPLETE_SUPPORTED_OUTPUT':report['status']='ABORTED_NATIVE_PAIR_RESTORE'
        finally:
            imu_pool.shutdown(wait=True,cancel_futures=True)
            if workers.native_pair is not None:
                report['native_phase_pair']['settings_history']=list(workers.native_pair.owner_settings_history)
                report['native_phase_pair']['coordinator_settings']=workers.native_pair.coordinator_settings
                report['native_phase_pair']['coordinator_settings_history']=list(
                    workers.native_pair.coordinator_settings_history)
                report['native_phase_pair']['last_phase']=workers.native_pair.last_phase
        # Convert copies and JSON-ready dictionaries only after all bus owners stop.
        report['acquisition_future_notification']={
            'groups_created':workers.acquisition_notification_groups,
            'wait_calls':workers.acquisition_notification_waits,
            'original_futures_required':True,'absolute_deadlines_unchanged':True,
            'hardware_timing_improvement_proven':False}
        if workers.acquisition_notification_groups:
            report['input_acquisition_wait']='native_future_notification_or_ready_poll_200us.v1'
        report['voltage_future_notification']={
            'groups_created':workers.voltage_notification_groups,
            'wait_calls':workers.voltage_notification_waits,
            'original_futures_required':True,'absolute_deadlines_unchanged':True,
            'hardware_timing_improvement_proven':False}
        if workers.voltage_notification_groups:
            report['voltage_join_wait']='native_future_notification_or_ready_poll_200us.v1'
        report['output_future_notification']={
            'selected':workers.unpaired_output_future_notifications,
            'groups_created':workers.output_notification_groups,
            'wait_calls':workers.output_notification_waits,
            'original_futures_required':True,'absolute_deadlines_unchanged':True,
            'cleanup_within_original_deadline_required':True,
            'hardware_timing_improvement_proven':False}
        if prepare_voltage_before_feedback_publication:
            report['prepared_voltage_publication']={
                'schema':'singularitydog.active-prepared-voltage-publication.v1',
                'mode':'validate_feedback_prepare_voltage_publish_then_native',
                'transport_capability':'singularitydog.active-prepared-exchange.v1',
                'selection_bound_to_reviewed_profile':True,
                'cadence_source_sha256':dict(profile['cadence_source_sha256']),
                'records':[dict(row) for row in workers.prepared_voltage_publications],
                'changes_deadline_or_cancellation_guards':False,
                'hardware_timing_improvement_proven':False}
        if pending_timing.active:
            report['failed_cycle_timing']=pending_timing.snapshot(profile)
        report['cycles']=[cycle.materialize() for cycle in report['cycles']]
        for cycle in report['cycles']:
            cycle['command']=asdict(cycle['command']);cycle['feedback']=asdict(cycle['feedback'])
        report['preload_targets_sent']=any(label=='preload_output' and any(row.written==17 for row in r[0])
            for _,r,_,label in workers.journal)
        report['learned_targets_sent']=any(label=='policy_output' and any(row.written==17 for row in r[0])
            for _,r,_,label in workers.journal)
        sent_frames=[codec.ATParser().feed(bytes(row.tx))[0] for _,r,_,_ in workers.journal
                     for row in r[0] if row.written==17]
        report['motor_enable_sent']=any(frame.kind==3 for frame in sent_frames)
        report['motion_gain_sent']=any(frame.kind==1 and any(frame.data[4:]) for frame in sent_frames)
        report['command_output_sent']=any(frame.kind==1 for frame in sent_frames)
        if v3:
            report['voltage_guard']['latest_by_id']={str(i):{
                'value_v':voltage_cache[i][0],'received_ns':voltage_cache[i][1]}
                for i in sorted(voltage_cache)}
        report['journal']=[{'bus':s,'error':e,'phase':label,**exchange_evidence(*r),
                           'rejected_total':getattr(r[1],'rejected_total',r[1].rejected_size),
                           'rejected_truncated':getattr(r[1],'rejected_total',r[1].rejected_size)>r[1].rejected_size}
                          for s,r,e,label in workers.journal]
        report['deadline20ms_misses']=sum(r['deadline20ms_missed'] for r in report['cycles'])
        report['post_reply_deadline_allowance_uses']=(0 if post_reply_budget is None else
                                                     post_reply_budget.accepted_misses)
        report['post_reply_deadline_rejections']=[{'index':r['index'],**r['post_reply_deadline']}
            for r in report['cycles'] if r.get('post_reply_deadline',{}).get('accepted') is False]
        report['startup_20ms_misses']=sum(r['deadline20ms_missed'] for r in report['cycles']
                                          if startup_20ms_allowance and r['index']==0)
        report['startup_20ms_allowance_uses']=sum(r['startup_20ms_allowance_used']
                                                 for r in report['cycles'])
        report['steady_deadline20ms_misses']=sum(r['steady_deadline20ms_missed'] for r in report['cycles'])
        # Compute start spacing from integer monotonic timestamps after STOP;
        # the existing <=21ms wakeup diagnostic is not strict 50Hz evidence.
        report.update(_start_interval_metrics(report['cycles']))
        if absolute_epoch_cadence:
            slots=[row['cadence_slot'] for row in report['cycles']]
            report['absolute_epoch_schedule']={
                'enabled':True,'epoch_ns':start if 'start' in locals() else None,
                'period_ns':PERIOD_NS,
                'minimum_start_separation_ns':ABSOLUTE_MIN_START_SEPARATION_NS,
                'completed_slots':slots,
                'skipped_slots':sum(right-left-1 for left,right in zip(slots,slots[1:])),
                'strict_20ms_start_interval_verified':report['strict_start_interval_20ms_met']}
        report['host_watchdog_reason']=workers.reason
    if fixed_catch is not None and report['status']=='COMPLETE_SUPPORTED_OUTPUT':
        report['status']='COMPLETE_FIXED_CATCH_HOLD'
    if human_supported is not None and report['status']=='COMPLETE_SUPPORTED_OUTPUT':
        report['status']='COMPLETE_HUMAN_SUPPORTED_PARTIAL_HOLD'
    return report
