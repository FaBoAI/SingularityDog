"""Three concurrent inputs -> real stateful policy -> twelve STOP proxy writes.

This diagnostic never enables motors or sends learned targets. STOP changes
state: run only on an already disabled, independently supported robot. Type17
mode measures acquisition/inference only and sends no STOP. Model/CAN/IMU setup
and warmup are outside the timed cycle; every measured cycle uses new inputs.
"""
import argparse
from array import array
from collections import namedtuple
from concurrent.futures import FIRST_COMPLETED, FIRST_EXCEPTION, Future, ThreadPoolExecutor, wait
from contextlib import ExitStack
import ctypes as C
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import struct
import sys
import threading
import time

from . import native_diagnostic_transport as native
from . import can_readonly as codec
from . import dual_can_pipeline_benchmark as dual
from . import imu
from . import policy_observer as observer
from . import policy_observer_live as live
from . import policy_observer_replay as replay
from . import policy_shadow as shadow
from . import thread_timer_slack
from . import math_thread_startup as math_threads

PERIOD_NS = 20_000_000
ABSOLUTE_MIN_START_SEPARATION_NS = 15_000_000
LIMIT_NS = 100_000_000
WORKER_STARTUP_TIMEOUT_S = .5
_TimingRecord = namedtuple('_TimingRecord','start_ns finish_ns received_ns')
_FeedbackProof = namedtuple('_FeedbackProof','images sample snapshot')
_VoltageProof = namedtuple('_VoltageProof','images values')
_TRACE_SCOPE_INDEX = {scope:index for index,scope in enumerate(dual.SCOPES)}
_READ_WIRES = {(i,p):codec.read_request(i,p) for i in range(1,13)
               for p in ('position','velocity','voltage')}
_STOP_WIRES = {i:native.stop_wire(i) for i in range(1,13)}
# Exact Type2, disabled/fault-free, destination 0xfd, extended-frame flag4,
# DLC8 headers. Matching these bytes is equivalent to decoding flags/can_id.
_STOP_REPLY_HEADERS = {i:b'AT'+((((2<<24)|(i<<8)|0xfd)<<3)|4).to_bytes(4,'big')+b'\x08'
                       for i in range(1,13)}
_OUTPUT_DISPATCH_FIELDS = (
    'infer_end_ns', 'main_check_start_ns', 'main_check_end_ns',
    'front_submit_end_ns', 'rear_submit_end_ns',
    'front_worker_enter_ns', 'front_worker_check_end_ns', 'front_native_begin_ns',
    'front_first_write_ns', 'rear_worker_enter_ns', 'rear_worker_check_end_ns',
    'rear_native_begin_ns', 'rear_first_write_ns',
    'main_infer_thread_cpu_ns', 'main_submits_end_thread_cpu_ns')


def _start_source_provenance(mode, power_epoch):
    """Optional file provenance; no power detection or output authorization.

    Legacy diagnostic invocations remain unbound. A scoped timing record must
    explicitly name its epoch and pin its own source set before opening any
    devices. The caller's epoch string is an assertion, not a sensor reading.
    """
    if mode is None and power_epoch is None:
        return None
    from . import policy_live_profile as profiles
    if mode not in (profiles.SUPPORTED_PRELOAD_5S,
                    profiles.HUMAN_SUPPORTED_PARTIAL_CURRENT_HOLD_8S):
        raise ValueError('Explicit supported --provenance-mode is required with --power-epoch')
    if (type(power_epoch) is not str or not 0 < len(power_epoch) <= 256 or
            power_epoch.strip() != power_epoch or not power_epoch.isprintable()):
        raise ValueError('Scoped source provenance requires an explicit nonempty --power-epoch')
    return {'schema':'singularitydog.diagnostic-source-provenance.v1',
            'mode':mode,'motor_power_epoch':power_epoch,
            'power_epoch_source':'explicit_operator_argument_not_hardware_detected',
            'cadence_source_sha256':profiles.cadence_source_hashes(
                {'diagnostic_timing_acceptance':mode}),
            'source_files_unchanged':None,
            'output_allowed':False,'approved_for_runtime':False}


def _finish_source_provenance(report, provenance):
    """Fail the diagnostic if any pinned file changed during its finite run."""
    if provenance is None:
        return
    from . import policy_live_profile as profiles
    try:
        current = profiles.cadence_source_hashes(
            {'diagnostic_timing_acceptance':provenance['mode']})
        if current != provenance['cadence_source_sha256']:
            raise ValueError('Diagnostic cadence source changed during execution')
        provenance['source_files_unchanged'] = True
    except Exception as error:
        provenance['source_files_unchanged'] = False
        report['status'] = 'ABORTED'
        report.setdefault('errors', []).append(type(error).__name__+': '+str(error))


def _native_record_frame(wire):
    """Decode one complete fixed-size native record without streaming state.

    A valid Type17 or STOP reply has eight data bytes, so its AT frame must
    occupy all 17 bytes. The native ABI stores exactly one 17-byte wire per
    record; incomplete or extra bytes are never accepted as telemetry.
    """
    if wire[:2]!=b'AT' or wire[6]!=8 or wire[15:]!=b'\r\n':
        raise ValueError('Malformed native record')
    encoded=int.from_bytes(wire[2:6],'big')
    return codec.Frame(encoded>>3,encoded&7,wire[7:15],wire)


class _TraceExchange(C.Structure):
    """Fixed capacity for one native exchange; allocated before measured cycles."""
    _fields_ = [('count',C.c_uint32),('records',native.Record*12),('stats',native.Stats)]


class _TraceRow:
    __slots__ = ('storage','cycle_index','metadata','phase_scopes')

    def __init__(self,storage,cycle_index,metadata,phase_scopes):
        self.storage=storage;self.cycle_index=cycle_index;self.metadata=metadata
        self.phase_scopes=phase_scopes

    def serialize(self):
        return {**self.metadata,
            **{phase:{s:self.storage.evidence(self.cycle_index,phase,s)
                       for s in scopes}
               for phase,scopes in self.phase_scopes.items()}}


class _RecordTrace:
    """Bounded native buffers, with no JSON or evidence dicts in the timed cycle."""
    def __init__(self,cycles,mode,*,voltage_overlap=False):
        self.capacity_cycles=cycles
        self.phases=('acquired','voltage','output') if voltage_overlap else (
            ('acquired','output') if mode=='stop-proxy' else ('acquired',))
        self.slots_per_cycle=2*len(self.phases)
        self.slots=(_TraceExchange*(cycles*self.slots_per_cycle))()
        self.allocated_bytes=C.sizeof(self.slots)
        # ctypes gives zeroed virtual storage, but its pages may first fault
        # when capture writes them after a timed reply. Materialize all fixed
        # trace pages now, while setup remains outside the measured schedule.
        C.memset(C.addressof(self.slots),0,self.allocated_bytes)
        self.pretouched_bytes=self.allocated_bytes

    def _slot(self,cycle_index,phase,scope):
        if not 0<=cycle_index<self.capacity_cycles or phase not in self.phases:
            raise ValueError('Invalid trace cycle or phase')
        if scope not in _TRACE_SCOPE_INDEX:raise ValueError('Unknown trace bus')
        return self.slots[cycle_index*self.slots_per_cycle+
                          2*self.phases.index(phase)+_TRACE_SCOPE_INDEX[scope]]

    def capture(self,cycle_index,row):
        scopes={phase:tuple(row[phase]) for phase in self.phases}
        for phase in self.phases:
            for scope,(records,stats) in row[phase].items():
                count=len(records)
                if (not isinstance(records,C.Array) or type(records)._type_ is not native.Record or
                        not 1<=count<=12 or type(stats) is not native.Stats):
                    raise ValueError('Invalid native trace exchange')
                slot=self._slot(cycle_index,phase,scope)
                C.memmove(C.addressof(slot.records),C.addressof(records),C.sizeof(records))
                C.memmove(C.addressof(slot.stats),C.addressof(stats),C.sizeof(native.Stats))
                slot.count=count
        metadata={k:v for k,v in row.items() if k not in self.phases}
        return _TraceRow(self,cycle_index,metadata,scopes)

    def evidence(self,cycle_index,phase,scope):
        slot=self._slot(cycle_index,phase,scope)
        if not 1<=slot.count<=12:raise ValueError('Missing trace exchange')
        return native.exchange_evidence(slot.records[:slot.count],slot.stats)


def distribution(values):
    if not values:return None
    values=sorted(values)
    return {'count':len(values),'median':statistics.median(values),
            'p95':values[math.ceil(.95*len(values))-1],
            'p99':values[math.ceil(.99*len(values))-1],'max':values[-1]}


def _steady_timing_summary(measurements, requested, startup_cycles, complete):
    """An explicit startup allowance never drops evidence or hides later misses."""
    startup=measurements[:startup_cycles]
    steady=measurements[startup_cycles:]
    all_complete=complete and len(measurements)==requested and len(steady)>0
    scheduled=[r for r in steady if 'scheduled_completion_slack_ms' in r]
    pairs=steady[1:]
    iteration_misses=sum(not r['iteration_deadline_met'] for r in steady)
    scheduled_misses=sum(r['scheduled_completion_slack_ms']<0 for r in scheduled)
    strict_misses=sum(r['actual_release_interval_ms']>20. for r in pairs)
    skipped=sum(r.get('skipped_slots_before',0) for r in steady)
    return {'startup_cycle_allowance':startup_cycles,
        'startup_cycles_recorded':len(startup),
        'startup_whole_iteration_ms':[r['whole_iteration_ms'] for r in startup],
        'startup_iteration_deadline_misses':sum(not r['iteration_deadline_met'] for r in startup),
        'startup_to_steady_interval_ms':(steady[0]['actual_release_interval_ms']
                                       if startup_cycles and steady else None),
        'steady_cycles_requested':requested-startup_cycles,
        'steady_cycles_completed':len(steady),
        'steady_whole_iteration_ms':distribution([r['whole_iteration_ms'] for r in steady]),
        'steady_iteration_deadline_misses':iteration_misses,
        'steady_processing_20ms_met':bool(all_complete and iteration_misses==0),
        'steady_scheduled_completion_slack_ms':distribution(
            [r['scheduled_completion_slack_ms'] for r in scheduled]),
        'steady_min_completion_slack_ms':min(
            (r['scheduled_completion_slack_ms'] for r in scheduled),default=None),
        'steady_scheduled_deadline_misses':scheduled_misses if scheduled else None,
        'steady_slots_skipped':skipped,
        'steady_scheduled_deadlines_met':bool(all_complete and len(scheduled)==len(steady)
                                              and scheduled_misses==0 and skipped==0),
        'steady_start_interval_ms':distribution([r['actual_release_interval_ms'] for r in pairs]),
        'steady_start_intervals_over_20ms':strict_misses,
        'strict_steady_start_interval_20ms_met':bool(all_complete and len(pairs)>0
                                                   and strict_misses==0 and skipped==0),
        'all_cycles_retained':bool(all_complete),
        'diagnostic_only':True,'full_controller_50Hz_verified':False}


def _absolute_epoch_slot(epoch_ns, previous_slot, previous_start_ns, now_ns):
    """Choose a fixed 20 ms slot without replaying missed work in a burst.

    A late wake can skip one or more slots. The 15 ms minimum spacing allows
    small scheduler jitter to settle at the next epoch while preventing an
    almost immediate second STOP transaction after a very late first start.
    """
    if now_ns<epoch_ns or (previous_slot is None)!=(previous_start_ns is None):
        raise ValueError('Invalid absolute-epoch schedule state')
    minimum_slot=0 if previous_slot is None else previous_slot+1
    elapsed_slot=(now_ns-epoch_ns)//PERIOD_NS
    if previous_start_ns is None:
        spaced_slot=0
    else:
        earliest=previous_start_ns+ABSOLUTE_MIN_START_SEPARATION_NS
        spaced_slot=max(0,(earliest-epoch_ns+PERIOD_NS-1)//PERIOD_NS)
    slot=max(minimum_slot,elapsed_slot,spaced_slot)
    return slot,epoch_ns+slot*PERIOD_NS


def snapshot_from_records(records_by_bus, sample, tick_ns, *, expected_voltage_by_bus=None):
    """Validate raw replies; one input allocation, no event JSON/copy/hash pass.

    Raw record evidence stays owned by the run, directly or in a bounded
    preallocated trace, and is serialized after it ends.
    The existing observer independently validates source order/freshness/ranges
    and hashes the assembled input before inference.
    """
    motors, seen = [], set()
    oldest=latest=earliest_receive=None
    composite=False;voltages={}
    for scope, records in records_by_bus.items():
        if scope not in dual.SCOPES:raise ValueError('Unknown bus')
        for r in records:
            if not (r.written==r.received==17 and
                    0<r.start_ns<=r.finish_ns<=r.received_ns<r.deadline_ns and r.received_ns<=tick_ns):
                raise ValueError('Incomplete/noncausal native input')
            tx,rx=_native_record_frame(bytes(r.tx)),_native_record_frame(bytes(r.rx))
            mid=tx.destination
            if mid not in dual.SCOPES[scope]:raise ValueError('Cross-bus input')
            if tx.kind==4:
                composite=True
                if (bytes(r.tx)!=_STOP_WIRES[mid] or rx.flags!=4 or
                    rx.can_id!=((2<<24)|(mid<<8)|0xfd) or len(rx.data)!=8 or
                    rx.data[:3]==b'\x00\xc4\x56'):
                    raise ValueError('Invalid STOP composite response')
                p,v,_,_=struct.unpack('>4H',rx.data)
                pairs=(('position',p*(2.*12.57)/65535.-12.57),('velocity',v*100./65535.-50.))
            elif tx.kind==17:
                parameter=next((name for name in ('position','velocity','voltage')
                    if bytes(r.tx)==_READ_WIRES[mid,name]),None)
                if parameter is None:raise ValueError('Invalid Type17 request')
                if bytes(r.tx)!=_READ_WIRES[mid,parameter]:raise ValueError('Invalid Type17 request')
                decoded=codec.decode_reply(rx,mid,parameter)
                if not decoded['ok']:raise ValueError('Rejected Type17 value')
                if parameter=='voltage':
                    if expected_voltage_by_bus is None or scope in voltages or not math.isfinite(decoded['value']):
                        raise ValueError('Unexpected/invalid voltage cadence input')
                    voltages[scope]=(mid,decoded['value'])
                    # This read is timed as part of acquisition, but the
                    # observer's policy-input age/spread covers only the 24
                    # position/velocity values and IMU. Keep those summaries
                    # equal to its independent recomputation.
                    continue
                pairs=((parameter,decoded['value']),)
            else:raise ValueError('Identity is not cycle telemetry')
            for parameter,value in pairs:
                key=(mid,parameter)
                if key in seen:raise ValueError('Duplicate input')
                seen.add(key)
                motors.append({'motor_id':mid,'parameter':parameter,'value':value,
                    'unit':'rad' if parameter=='position' else 'rad_s',
                    'request_ns':r.start_ns,'received_ns':r.received_ns,
                    'age_upper_bound_ns':tick_ns-r.start_ns})
                oldest=r.start_ns if oldest is None else min(oldest,r.start_ns)
                latest=r.received_ns if latest is None else max(latest,r.received_ns)
                earliest_receive=(r.received_ns if earliest_receive is None else
                                  min(earliest_receive,r.received_ns))
    if seen!={(i,p) for i in range(1,13) for p in ('position','velocity')}:
        raise ValueError('Missing full twelve-axis position/velocity inputs')
    if expected_voltage_by_bus is not None and {s:mid for s,(mid,_) in voltages.items()}!=expected_voltage_by_bus:
        raise ValueError('Missing or incorrect rotating voltage input')
    a,b=sample['read_started_monotonic_ns'],sample['read_finished_monotonic_ns']
    if not (0<a<=b<=tick_ns):raise ValueError('Noncausal IMU')
    for name in ('accel_m_s2','gyro_rad_s'):
        if len(sample[name])!=3 or not all(math.isfinite(x) for x in sample[name]):
            raise ValueError('Invalid IMU vector')
    oldest=min(oldest,a);latest=max(latest,b);earliest_receive=min(earliest_receive,b)
    if tick_ns-oldest>LIMIT_NS:raise ValueError('Expired inputs')
    return {'status':'DIAGNOSTIC_READY','output_allowed':False,'blocked_reasons':[],
        'tick_ns':tick_ns,'max_age_ns':LIMIT_NS,'max_spread_ns':LIMIT_NS,'motors':motors,
        'imu':{'frame':'raw_sensor','accel_m_s2':list(sample['accel_m_s2']),
               'gyro_rad_s':list(sample['gyro_rad_s']),'read_started_ns':a,'read_finished_ns':b,
               'age_upper_bound_ns':tick_ns-a},
        'oldest_observation_age_ns':tick_ns-oldest,'acquisition_spread_ns':latest-oldest,
        'receive_spread_ns':latest-earliest_receive,
        'voltage_by_bus':{s:{'motor_id':mid,'value_v':value} for s,(mid,value) in voltages.items()},
        'source_flags':{'native_diagnostic_transport':True,'sensor_type2_candidate':composite,
            'v3_voltage_cadence_proxy':expected_voltage_by_bus is not None,
            'velocity_scale_verified':False,'sensor_internal_sample_time_verified':False,
            'stop_feedback_state_changing':composite,'fresh_identity_match_verified':True,
            'approved_for_runtime':False,'output_allowed':False}}


def timing_row(acquired, sample, output, *, release_ns, gather_end_ns,
               prepare_end_ns, infer_end_ns, cycle_end_ns):
    input_records=[r for records,_ in acquired.values() for r in records]
    first=min(sample['read_started_monotonic_ns'],*(r.start_ns for r in input_records))
    input_end=max(sample['read_finished_monotonic_ns'],*(r.received_ns for r in input_records))
    sent=[r for records,_ in output.values() for r in records]
    final_write=max((r.finish_ns for r in sent),default=0)
    final_reply=max((r.received_ns for r in sent),default=0)
    return _timing_row_from_scalars(first,input_end,final_write,final_reply,bool(sent),
        release_ns=release_ns,gather_end_ns=gather_end_ns,
        prepare_end_ns=prepare_end_ns,infer_end_ns=infer_end_ns,cycle_end_ns=cycle_end_ns)


def _timing_row_from_scalars(first,input_end,final_write,final_reply,has_output,*,release_ns,
                             gather_end_ns,prepare_end_ns,infer_end_ns,cycle_end_ns):
    return {'release_ns':release_ns,'oldest_input_start_ns':first,
        'input_latest_reply_ns':input_end,'gather_end_ns':gather_end_ns,
        'prepare_end_ns':prepare_end_ns,'infer_end_ns':infer_end_ns,
        'final_host_write_ns':final_write or None,'last_proxy_reply_ns':final_reply or None,
        'cycle_end_ns':cycle_end_ns,'acquisition_ms':(gather_end_ns-first)/1e6,
        'prepare_ms':(prepare_end_ns-gather_end_ns)/1e6,'inference_ms':(infer_end_ns-prepare_end_ns)/1e6,
        'oldest_input_to_final_host_write_ms':(final_write-first)/1e6 if has_output else None,
        'oldest_input_to_last_reply_ms':(final_reply-first)/1e6 if has_output else None,
        'whole_iteration_ms':(cycle_end_ns-release_ns)/1e6,
        'host_deadline_met':bool(has_output and final_write-first<=PERIOD_NS),
        'iteration_deadline_met':cycle_end_ns-release_ns<=PERIOD_NS,
        'host_write_is_can_wire_completion':False,'learned_targets_sent':False}


def _timing_scalars(acquired,sample,output):
    """Keep only timestamps and output presence when a trace drops raw results."""
    first=sample['read_started_monotonic_ns']
    input_end=sample['read_finished_monotonic_ns']
    for records,_ in acquired.values():
        for r in records:
            first=min(first,r.start_ns)
            input_end=max(input_end,r.received_ns)
    final_write=final_reply=0;has_output=False
    for records,_ in output.values():
        for r in records:
            has_output=True
            final_write=max(final_write,r.finish_ns)
            final_reply=max(final_reply,r.received_ns)
    return first,input_end,final_write,final_reply,has_output


def _feedback_then_voltage(exchange,scope,feedback_wires,voltage_wire,feedback_ready,
                           proof=None,clock=None,*,publish_before_native=False):
    """One bus owner publishes six replies, then makes its separate voltage read.

    A single worker performs both calls in order; there is never a second task
    racing the same session. Both calls keep their own native Stats and records.
    """
    feedback=None
    def publish_feedback():
        if proof is not None:
            proof.setdefault('feedback_published_ns_by_bus',{})[scope]=clock()
        feedback_ready.set_result(feedback)
    try:
        if proof is not None:proof['feedback_dispatch_ns_by_bus'][scope]=clock()
        feedback=exchange(scope,feedback_wires)
        if proof is not None:
            proof['feedback_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in feedback[0])
            proof['feedback_ready_ns_by_bus'][scope]=clock()
        if not publish_before_native:publish_feedback()
        if proof is not None:proof['voltage_dispatch_ns_by_bus'][scope]=clock()
        if publish_before_native:
            voltage=exchange(scope,(voltage_wire,),before_native=publish_feedback)
            if not feedback_ready.done():
                raise RuntimeError('Native voltage call omitted feedback publication')
        else:voltage=exchange(scope,(voltage_wire,))
        if proof is not None:
            proof['voltage_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in voltage[0])
        return voltage
    except BaseException as error:
        if not feedback_ready.done():
            # FD/check/preparation failures must retain the complete feedback
            # already received; the separate voltage future still fails.
            if feedback is not None:feedback_ready.set_result(feedback)
            else:feedback_ready.set_exception(error)
        raise


def _feedback_then_gated_voltage(exchange,scope,feedback_wires,voltage_wire,
                                 feedback_ready,gate,cancelled,proof,clock):
    """Keep one FD owner across feedback and a coordinator-released voltage read.

    The coordinator validates the complete feedback/IMU snapshot before opening
    the gate. A failed snapshot cancels the waiting owners without issuing a
    voltage request or a subsequent normal STOP-proxy output batch.
    """
    try:
        proof['feedback_dispatch_ns_by_bus'][scope]=clock()
        feedback=exchange(scope,feedback_wires)
        proof['feedback_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in feedback[0])
        proof['feedback_ready_ns_by_bus'][scope]=clock()
        feedback_ready.set_result(feedback)
        gate.wait()
        if cancelled.is_set():raise RuntimeError('Voltage pipeline cancelled before read')
        proof['voltage_dispatch_ns_by_bus'][scope]=clock()
        voltage=exchange(scope,(voltage_wire,))
        proof['voltage_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in voltage[0])
        return voltage
    except BaseException as error:
        if not feedback_ready.done():feedback_ready.set_exception(error)
        raise


def _await_voltage_ready(futures,validation_future,*,deadline_ns,deadline_wait=None,
                         clock=time.monotonic_ns,check=lambda:None,
                         thread_clock=time.thread_time_ns):
    """Wait only for readiness before taking already-owned voltage results.

    The existing native release wait releases the GIL and spins for targets at
    most 200 us apart. It does not read an FD or publish/replace a source time.
    Without that callback, use one bounded all-future condition wait. Result
    and frame/proof validation stay with the existing owners and dispatch gate.
    """
    if set(futures)!=set(dual.SCOPES) or any(not isinstance(f,Future) for f in futures.values()):
        raise ValueError('Exact voltage owner futures required')
    if validation_future is not None and not isinstance(validation_future,Future):
        raise ValueError('Voltage validation future required')
    if type(deadline_ns) is not int or deadline_ns<=0 or (deadline_wait is not None and not callable(deadline_wait)):
        raise ValueError('Absolute voltage join deadline required')
    owners=tuple(futures.values())+(() if validation_future is None else (validation_future,))
    if len({id(future) for future in owners})!=len(owners):
        raise ValueError('Distinct voltage owner/validation futures required')
    begin=clock();cpu_begin=thread_clock();calls=0
    if type(begin) is not int or begin<=0:
        raise ValueError('Causal voltage join clock required')
    while True:
        ready=tuple(f for f in owners if f.done())
        # A ready error wins over an unfinished second owner; never wait on it.
        for future in ready:
            if future.cancelled():raise RuntimeError('Voltage owner future cancelled')
            error=future.exception()
            if error is not None:raise error
        check()
        now=clock()
        if type(now) is not int or now<begin:
            raise ValueError('Noncausal voltage join clock')
        if now>=deadline_ns:
            raise TimeoutError('Voltage pipeline exceeded 20 ms hard deadline at voltage join')
        if len(ready)==len(owners):
            cpu_end=thread_clock();end=clock()
            if type(end) is not int or end<now or cpu_end<cpu_begin:
                raise ValueError('Noncausal voltage join completion clock')
            if end>=deadline_ns:
                raise TimeoutError('Voltage pipeline exceeded 20 ms hard deadline at voltage join')
            return {'mode':'native_readiness_poll_v1' if deadline_wait is not None else 'bounded_future_wait_v1',
                    'native_tick_max_us':200 if deadline_wait is not None else None,
                    'wait_calls':calls,'begin_ns':begin,'end_ns':end,
                    'thread_cpu_begin_ns':cpu_begin,'thread_cpu_end_ns':cpu_end,
                    'future_results_taken_only_after_ready':True}
        if deadline_wait is None:
            wait(owners,timeout=(deadline_ns-now)/1e9,return_when=FIRST_EXCEPTION)
        else:
            wake=min(deadline_ns,now+200_000)
            try:deadline_wait(wake)
            except BaseException:
                # Cancellation can wake the native wait after an owner failed.
                # Retain that original owner error rather than hiding it with
                # the cancellation notification raised by the wait callback.
                for future in owners:
                    if future.done() and not future.cancelled():
                        error=future.exception()
                        if error is not None:raise error
                raise
            returned=clock()
            if type(returned) is not int or returned<wake:
                raise ValueError('Native voltage readiness wait returned before requested wake')
        calls+=1


def _settle_voltage(futures,record):
    """Keep each completed voltage exchange, even if inference failed first."""
    errors=[]
    for scope,future in futures.items():
        try:record['voltage'][scope]=future.result()
        except BaseException as error:
            errors.append(error)
            record.setdefault('voltage_errors_by_bus',{})[scope]=(
                type(error).__name__+': '+str(error))
    return errors


def _verify_final_proxy_stop_records(output,clock):
    """Account for all twelve disabled-only STOP replies in the fast trace."""
    if set(output)!=set(dual.SCOPES):raise ValueError('Incomplete final proxy STOP buses')
    reply_ends={}
    for scope,ids in dual.SCOPES.items():
        records,_=output[scope]
        if len(records)!=len(ids):raise ValueError('Incomplete final proxy STOP replies')
        for mid,row in zip(ids,records):
            if not (bytes(row.tx)==_STOP_WIRES[mid] and row.written==row.received==17 and
                    0<row.start_ns<=row.finish_ns<=row.received_ns<row.deadline_ns):
                raise ValueError('Invalid final proxy STOP record')
            reply=bytes(row.rx)
            if (len(reply)!=17 or reply[:7]!=_STOP_REPLY_HEADERS[mid] or
                    reply[15:]!=b'\r\n' or reply[7:10]==b'\x00\xc4\x56'):
                raise ValueError('Invalid final proxy STOP reply')
        reply_ends[scope]=max(row.received_ns for row in records)
    verified_at=clock()
    if any(end>verified_at for end in reply_ends.values()):
        raise ValueError('Noncausal final proxy STOP reply')
    return reply_ends,verified_at


def _retain_submitted_feedback(feedback_ready,voltage_futures,record):
    """Preserve feedback from accepted tasks after a later submit was rejected."""
    for scope in voltage_futures:
        try:record['acquired'][scope]=feedback_ready[scope].result()
        except BaseException as error:
            record.setdefault('acquired_errors_by_bus',{})[scope]=(
                type(error).__name__+': '+str(error))


def _owned_record_images(owned,count):
    """Freeze complete native input records, including every header/timestamp.

    Each bus has already finished its exchange before these buffers are read.
    Whole-buffer comparison also catches mutations outside decoded SI values;
    no frame is reparsed just to prove the completed buffer is unchanged.
    """
    if set(owned)!=set(dual.SCOPES):raise ValueError('Incomplete voltage validation buses')
    images=[]
    for scope in dual.SCOPES:
        records,_=owned[scope]
        if (not isinstance(records,C.Array) or type(records)._type_ is not native.Record or
                len(records)!=count):
            raise ValueError('Incomplete native records before voltage validation')
        images.append(bytes(records))
    return tuple(images)


def _sample_signature(sample):
    return (sample['read_started_monotonic_ns'],sample['read_finished_monotonic_ns'],
            tuple(sample['accel_m_s2']),tuple(sample['gyro_rad_s']))


def _feedback_snapshot_signature(snapshot):
    # This locally constructed snapshot contains only bounded primitive fields.
    # Tuple ownership freezes its nested numeric values without a JSON hash or
    # recursive deepcopy. Keep units/ages/flags as well as the model SI values.
    imu_value=snapshot['imu']
    return (tuple((k,v) for k,v in snapshot.items()
                  if k not in ('motors','imu','source_flags','blocked_reasons','voltage_by_bus')),
            tuple(tuple(row.items()) for row in snapshot['motors']),
            tuple((k,tuple(v) if isinstance(v,list) else v) for k,v in imu_value.items()),
            tuple(snapshot['source_flags'].items()),tuple(snapshot['blocked_reasons']),
            tuple((s,tuple(v.items())) for s,v in snapshot['voltage_by_bus'].items()))


def _validated_feedback_for_voltage(acquired,sample,tick_ns):
    """Validate feedback once, then seal evidence before worker submission."""
    images=_owned_record_images(acquired,6)
    sample_value=_sample_signature(sample)
    snapshot=snapshot_from_records({s:x[0] for s,x in acquired.items()},sample,tick_ns,
                                   expected_voltage_by_bus=None)
    snapshot['source_flags']['v3_voltage_overlap_pending_at_inference']=True
    if images!=_owned_record_images(acquired,6) or sample_value!=_sample_signature(sample):
        raise ValueError('Feedback/IMU changed during initial voltage validation')
    return snapshot,_FeedbackProof(images,sample_value,_feedback_snapshot_signature(snapshot))


def _check_feedback_proof(acquired,sample,snapshot,proof):
    if (type(proof) is not _FeedbackProof or
            proof.images!=_owned_record_images(acquired,6) or
            proof.sample!=_sample_signature(sample) or
            proof.snapshot!=_feedback_snapshot_signature(snapshot)):
        raise ValueError('Feedback/IMU changed while voltage was pending')


def _voltage_from_records(voltage,expected_voltage_by_bus,tick_ns,voltage_max_v):
    """Decode only the two new voltage replies; retain the same wire checks."""
    if set(expected_voltage_by_bus)!=set(dual.SCOPES):
        raise ValueError('Missing or incorrect rotating voltage input')
    images=_owned_record_images(voltage,1);values={}
    for scope in dual.SCOPES:
        row=voltage[scope][0][0];mid=expected_voltage_by_bus[scope]
        if mid not in dual.SCOPES[scope]:raise ValueError('Cross-bus voltage input')
        if not (row.written==row.received==17 and
                0<row.start_ns<=row.finish_ns<=row.received_ns<row.deadline_ns and
                row.received_ns<=tick_ns):
            raise ValueError('Incomplete/noncausal native voltage input')
        if bytes(row.tx)!=_READ_WIRES[mid,'voltage']:
            raise ValueError('Missing or incorrect rotating voltage input')
        rx=_native_record_frame(bytes(row.rx))
        decoded=codec.decode_reply(rx,mid,'voltage')
        if not decoded['ok']:raise ValueError('Rejected Type17 voltage value')
        value=decoded['value']
        if not math.isfinite(value) or not 35.<=value<=voltage_max_v:
            raise ValueError(f'Voltage outside 35..{voltage_max_v:g} V before proxy STOP')
        values[scope]={'motor_id':mid,'value_v':value}
    if images!=_owned_record_images(voltage,1):
        raise ValueError('Voltage records changed during validation')
    return values,_VoltageProof(images,tuple((s,mid['motor_id'],mid['value_v'])
                                           for s,mid in values.items()))


def _check_voltage_proof(voltage,full):
    proof=full.get('_validated_voltage_proof')
    if (type(proof) is not _VoltageProof or
            proof.images!=_owned_record_images(voltage,1) or
            proof.values!=tuple((s,v['motor_id'],v['value_v'])
                                for s,v in full['voltage_by_bus'].items())):
        raise ValueError('Voltage changed before proxy STOP')


def _verify_joined_voltage_proof(acquired,voltage,sample,snapshot,full,proof,clock):
    """Verify the joined worker result without a second fourteen-record walk."""
    _check_feedback_proof(acquired,sample,snapshot,proof)
    _check_voltage_proof(voltage,full)
    now=clock()
    if (not snapshot['tick_ns']<=full['tick_ns']<=now or
            now-(snapshot['tick_ns']-snapshot['oldest_observation_age_ns'])>LIMIT_NS):
        raise ValueError('Expired feedback/voltage/IMU before proxy STOP')
    return now


def _verify_voltage_with_feedback_proof(acquired,voltage,sample,feedback_snapshot,
                                       expected_voltage_by_bus,clock,voltage_max_v,proof):
    if type(proof) is not _FeedbackProof:raise ValueError('Missing validated feedback proof')
    tick=clock()
    values,voltage_proof=_voltage_from_records(voltage,expected_voltage_by_bus,tick,voltage_max_v)
    # Preserve the complete validation-snapshot schema and actual age fields.
    # Build from the immutable seal, not live mutable snapshot descendants.
    # A concurrent mutation cannot influence this validation result even if
    # it occurs while voltage parsing releases/interleaves the Python thread.
    metadata,motors,imu_fields,flags,blocked,_=proof.snapshot
    full=dict(metadata)
    original_tick=full['tick_ns'];oldest=original_tick-full['oldest_observation_age_ns']
    motor_rows=[dict(row) for row in motors]
    for row in motor_rows:row['age_upper_bound_ns']=tick-row['request_ns']
    imu_value=dict(imu_fields)
    for key in ('accel_m_s2','gyro_rad_s'):imu_value[key]=list(imu_value[key])
    imu_value['age_upper_bound_ns']=tick-imu_value['read_started_ns']
    source_flags=dict(flags);source_flags.pop('v3_voltage_overlap_pending_at_inference',None)
    source_flags['v3_voltage_cadence_proxy']=True
    full.update(tick_ns=tick,motors=motor_rows,imu=imu_value,voltage_by_bus=values,
                source_flags=source_flags,blocked_reasons=list(blocked),
                oldest_observation_age_ns=tick-oldest,_validated_voltage_proof=voltage_proof)
    # Frozen timestamps were causal at feedback validation; both new voltage
    # replies were checked against tick above. Recheck the buffer and actual
    # oldest age here; the coordinator still walks all fourteen timestamps at
    # its final pre-STOP gate. No source timestamp is updated or backdated.
    _check_feedback_proof(acquired,sample,feedback_snapshot,proof)
    verified=clock()
    if verified<tick or tick<original_tick or verified-oldest>LIMIT_NS:
        raise ValueError('Expired/noncausal feedback/voltage/IMU validation')
    return full,verified


def _verify_voltage_after_inference(acquired,voltage,sample,feedback_snapshot,
                                    expected_voltage_by_bus,clock,voltage_max_v=42,feedback_proof=None):
    """Revalidate retained feedback and both late replies before a proxy STOP.

    This second snapshot is a validation copy, never a replacement for the
    feedback-only snapshot whose canonical hash the observer consumed.
    """
    if feedback_proof is not None:
        return _verify_voltage_with_feedback_proof(acquired,voltage,sample,feedback_snapshot,
            expected_voltage_by_bus,clock,voltage_max_v,feedback_proof)
    combined={scope:list(acquired[scope][0])+list(voltage[scope][0])
              for scope in dual.SCOPES}
    validation_tick=clock()
    full=snapshot_from_records(combined,sample,validation_tick,
                               expected_voltage_by_bus=expected_voltage_by_bus)
    fields=('motor_id','parameter','value','request_ns','received_ns')
    if ([tuple(row[key] for key in fields) for row in full['motors']] !=
            [tuple(row[key] for key in fields) for row in feedback_snapshot['motors']]):
        raise ValueError('Feedback changed while voltage was pending')
    for key in ('accel_m_s2','gyro_rad_s','read_started_ns','read_finished_ns'):
        if full['imu'][key]!=feedback_snapshot['imu'][key]:
            raise ValueError('IMU changed while voltage was pending')
    # The upper limit is explicitly selected; 42 V remains the default.
    # This disabled-motor diagnostic screen is never an output approval.
    if any(not 35. <= row['value_v'] <= voltage_max_v
           for row in full['voltage_by_bus'].values()):
        raise ValueError(f'Voltage outside 35..{voltage_max_v:g} V before proxy STOP')
    return full,clock()


def _validate_voltage_during_inference(voltage_futures,acquired,sample,
                                       feedback_snapshot,expected_voltage_by_bus,clock,voltage_max_v=42,
                                       feedback_proof=None):
    """Join both bus-owned voltage reads and validate on the free IMU worker.

    Submitted only after the IMU future has completed, so the three-worker pool
    has a free slot while the two bus workers finish their own serial reads.
    The main thread still joins this result and checks freshness before STOP.
    """
    voltage={scope:future.result() for scope,future in voltage_futures.items()}
    started=clock()
    full,finished=_verify_voltage_after_inference(
        acquired,voltage,sample,feedback_snapshot,expected_voltage_by_bus,clock,voltage_max_v,
        feedback_proof)
    return full,started,finished


def _verify_voltage_final_freshness(acquired,voltage,sample,feedback_snapshot,
                                    full,expected_voltage_by_bus,clock,voltage_max_v=42,
                                    feedback_proof=None):
    """Check the worker proof against the actual post-inference STOP gate time.

    The worker already decoded every frame and compared feedback/IMU with the
    observer snapshot. Owned records stay unchanged until trace capture. This
    final gate checks their 14 native timestamps at the actual STOP gate time,
    without repeating frame parsing or nested equality work.
    """
    now=clock()
    if feedback_proof is not None:
        _check_feedback_proof(acquired,sample,feedback_snapshot,feedback_proof)
        _check_voltage_proof(voltage,full)
    if (full.get('status')!='DIAGNOSTIC_READY' or full.get('output_allowed') is not False or
            not 0<full.get('tick_ns',0)<=now or
            feedback_snapshot.get('source_flags',{}).get('v3_voltage_overlap_pending_at_inference') is not True or
            full.get('source_flags',{}).get('v3_voltage_cadence_proxy') is not True or
            set(acquired)!=set(dual.SCOPES) or set(voltage)!=set(dual.SCOPES)):
        raise ValueError('Voltage validation proof differs before proxy STOP')
    if (len(full.get('motors',()))!=len(feedback_snapshot.get('motors',())) or
            full.get('imu',{}).get('read_started_ns')!=
                feedback_snapshot.get('imu',{}).get('read_started_ns') or
            full.get('imu',{}).get('read_finished_ns')!=
                feedback_snapshot.get('imu',{}).get('read_finished_ns')):
        raise ValueError('Voltage validation snapshot differs before proxy STOP')
    if ({scope:row['motor_id'] for scope,row in full['voltage_by_bus'].items()}!=
            expected_voltage_by_bus or
            any(not 35.<=row['value_v']<=voltage_max_v
                for row in full['voltage_by_bus'].values())):
        raise ValueError('Voltage changed before proxy STOP')
    oldest=latest=earliest_receive=None
    for scope in dual.SCOPES:
        for phase,owned,expected_count in (
                ('feedback',acquired[scope][0],6),('voltage',voltage[scope][0],1)):
            if len(owned)!=expected_count:
                raise ValueError('Incomplete '+phase+' before proxy STOP')
            for row in owned:
                if not (row.written==row.received==17 and
                        0<row.start_ns<=row.finish_ns<=row.received_ns<row.deadline_ns and
                        row.received_ns<=now):
                    raise ValueError('Noncausal '+phase+' before proxy STOP')
                oldest=row.start_ns if oldest is None else min(oldest,row.start_ns)
                latest=row.received_ns if latest is None else max(latest,row.received_ns)
                earliest_receive=(row.received_ns if earliest_receive is None else
                                  min(earliest_receive,row.received_ns))
    imu_start=sample['read_started_monotonic_ns']
    imu_end=sample['read_finished_monotonic_ns']
    if (not 0<imu_start<=imu_end<=now or
            full['imu']['read_started_ns']!=imu_start or
            full['imu']['read_finished_ns']!=imu_end):
        raise ValueError('Noncausal or changed IMU before proxy STOP')
    oldest=min(oldest,imu_start)
    latest=max(latest,imu_end)
    earliest_receive=min(earliest_receive,imu_end)
    if (now-oldest>LIMIT_NS or latest-oldest>LIMIT_NS or
            latest-earliest_receive>LIMIT_NS):
        raise ValueError('Expired feedback/voltage/IMU before proxy STOP')
    return now


def _prestart_workers(pool, check):
    """Start all three workers with bounded, no-I/O tasks before cycle release."""
    until=time.monotonic()+WORKER_STARTUP_TIMEOUT_S
    ready=threading.Barrier(3)
    futures=[]
    def start_worker():
        ready.wait(timeout=max(0.,until-time.monotonic()))
        return threading.get_ident()
    try:
        check()
        for _ in range(3):futures.append(pool.submit(start_worker))
        pending=set(futures);workers=set()
        while pending:
            check()
            remaining=until-time.monotonic()
            if remaining<=0:raise TimeoutError('Diagnostic worker startup deadline exceeded')
            finished,pending=wait(pending,timeout=min(.01,remaining),return_when=FIRST_COMPLETED)
            for future in finished:workers.add(future.result())
        check()
        if len(workers)!=3:raise RuntimeError('Diagnostic startup requires three distinct workers')
    except BaseException as error:
        ready.abort()
        for future in futures:future.cancel()
        if isinstance(error,threading.BrokenBarrierError):
            raise TimeoutError('Diagnostic worker startup deadline exceeded') from error
        raise


def _verify_unpinned_workers(pool, check, expected):
    """Prove the three prestarted I/O workers did not inherit a main-thread pin."""
    barrier=threading.Barrier(3)
    until=time.monotonic()+WORKER_STARTUP_TIMEOUT_S
    def sample():
        barrier.wait(timeout=max(0.,until-time.monotonic()))
        return threading.get_native_id(), sorted(os.sched_getaffinity(0))
    futures=[]
    try:
        check()
        for _ in range(3):futures.append(pool.submit(sample))
        values=[future.result(timeout=max(0.,until-time.monotonic())) for future in futures]
        check()
        if len({tid for tid,_ in values})!=3 or any(set(mask)!=expected for _,mask in values):
            raise RuntimeError('I/O worker affinity changed during main-thread pin')
        return [{'native_tid':tid,'cpus':mask} for tid,mask in values]
    except BaseException:
        barrier.abort()
        for future in futures:future.cancel()
        raise


def _transition_worker_affinity(pool, originals, target):
    """Set or restore all three prestarted workers, identified by native TID.

    Each task waits at a barrier so one executor worker cannot perform two
    transitions while another has not run. Errors are returned as rows, leaving
    the caller able to restore every saved original mask after partial setup.
    """
    barrier=threading.Barrier(3)
    until=time.monotonic()+WORKER_STARTUP_TIMEOUT_S
    def change():
        tid=threading.get_native_id()
        row={'native_tid':tid,'before':None,'after':None,'error':None}
        try:
            barrier.wait(timeout=max(0.,until-time.monotonic()))
            original=originals.get(tid)
            if original is None:raise RuntimeError('Unknown I/O worker TID')
            if target is not None:
                row['before']=sorted(os.sched_getaffinity(0))
                if set(row['before'])!=original:
                    raise RuntimeError('I/O worker original affinity changed before setup')
            desired=original if target is None else target
            os.sched_setaffinity(0,desired)
            row['after']=sorted(os.sched_getaffinity(0))
            if set(row['after'])!=desired:
                raise RuntimeError('I/O worker affinity readback differs')
        except BaseException as error:
            row['error']=type(error).__name__+': '+str(error)
        return row
    futures=[]
    try:
        for _ in range(3):futures.append(pool.submit(change))
        rows=[future.result(timeout=max(0.,until-time.monotonic())) for future in futures]
    except BaseException:
        barrier.abort()
        for future in futures:future.cancel()
        raise
    if len({row['native_tid'] for row in rows})!=3 or set(originals)!={row['native_tid'] for row in rows}:
        raise RuntimeError('I/O worker TID set changed during affinity transition')
    return rows


def _reused_policy_input_tensors(run):
    """Fail closed unless the prime uses all six owned CPU float buffers."""
    sizes=(3,3,3,12,12,12)
    buffers=getattr(run,'_input_buffers',None)
    tensors=getattr(run,'_input_tensors',None)
    if (type(buffers) is not tuple or type(tensors) is not tuple or
            len(buffers)!=6 or len(tensors)!=6 or
            any(type(buf) is not array or len(buf)!=size or
                tuple(tensor.shape)!=(1,size) or
                tensor.data_ptr()!=buf.buffer_info()[0]
                for buf,tensor,size in zip(buffers,tensors,sizes))):
        raise ValueError('Post-pin prime requires six owned reused CPU float input buffers')
    return tensors


def collect(sessions, imu_device, policy_observer, *, mode, cycles, check=lambda:None,
            clock=time.monotonic_ns, sleep=time.sleep, worker_initializer=None,
            record_storage='objects', main_thread_cpu=None, output_dispatch_trace=False,
            defer_gc_during_cycles=False, pre_cycle_policy_prepare=None,
            post_pin_policy_prepare=None,v3_voltage_proxy=False,
            v3_voltage_overlap=False,v3_voltage_validation_overlap=False,
            v3_voltage_pipeline=False,v3_voltage_fast_pipeline=False,
            inference_thread_cpu_trace=False,absolute_epoch_cadence=False,
            exclude_policy_cpu_from_workers=False,startup_cycle_allowance=0,deadline_wait=None,
            voltage_max_v=42):
    """Finite no-catchup benchmark, injectable transports for failure testing."""
    if (mode not in ('type17','stop-proxy') or not 1<=cycles<=3000 or
            record_storage not in ('objects','encoded','trace')):
        raise ValueError('Invalid mode or cycle budget')
    if type(voltage_max_v) not in (int,float) or voltage_max_v not in (42,43):
        raise ValueError('Voltage maximum must be explicitly 42 or 43 V')
    if (type(startup_cycle_allowance) is not int or startup_cycle_allowance not in (0,1) or
            startup_cycle_allowance and (mode!='stop-proxy' or policy_observer is None or
                                         not 2<=cycles<=501)):
        raise ValueError('Startup allowance requires one recorded startup and 1..500 STOP-proxy inference cycles')
    bounded_cycles=500+startup_cycle_allowance
    if deadline_wait is not None and (not callable(deadline_wait) or not absolute_epoch_cadence):
        raise ValueError('Native release wait requires absolute-epoch cadence')
    if type(v3_voltage_proxy) is not bool or (v3_voltage_proxy and
            (mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles)):
        raise ValueError('V3 voltage proxy requires at most 500 STOP-proxy inference cycles')
    if (type(v3_voltage_overlap) is not bool or v3_voltage_overlap and
            (not v3_voltage_proxy or mode!='stop-proxy' or policy_observer is None or
             cycles>bounded_cycles or record_storage!='trace')):
        raise ValueError('Voltage overlap requires bounded V3 STOP-proxy inference with trace storage')
    if (type(v3_voltage_validation_overlap) is not bool or
            v3_voltage_validation_overlap and not v3_voltage_overlap):
        raise ValueError('Voltage validation overlap requires voltage overlap')
    if (type(v3_voltage_pipeline) is not bool or v3_voltage_pipeline and not (
            v3_voltage_proxy and v3_voltage_overlap and v3_voltage_validation_overlap and
            mode=='stop-proxy' and policy_observer is not None and
            record_storage=='trace' and cycles<=bounded_cycles)):
        raise ValueError('Voltage pipeline requires bounded V3 STOP-proxy trace with voltage and validation overlap')
    if (type(v3_voltage_fast_pipeline) is not bool or v3_voltage_fast_pipeline and not (
            v3_voltage_proxy and v3_voltage_overlap and v3_voltage_validation_overlap and
            mode=='stop-proxy' and policy_observer is not None and
            record_storage=='trace' and cycles<=bounded_cycles) or
            v3_voltage_fast_pipeline and v3_voltage_pipeline):
        raise ValueError('Fast voltage pipeline requires bounded V3 STOP-proxy trace and excludes gated pipeline')
    pipeline_key=('voltage_pipeline' if v3_voltage_pipeline else
                  'voltage_fast_pipeline' if v3_voltage_fast_pipeline else None)
    if (type(inference_thread_cpu_trace) is not bool or
            inference_thread_cpu_trace and not v3_voltage_proxy):
        raise ValueError('Inference thread CPU trace requires bounded 26-request STOP-proxy inference')
    if (type(absolute_epoch_cadence) is not bool or absolute_epoch_cadence and
            (mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles)):
        raise ValueError('Absolute-epoch cadence requires at most 500 STOP-proxy inference cycles')
    if main_thread_cpu is not None and (type(main_thread_cpu) is not int or main_thread_cpu<0):
        raise ValueError('Invalid main-thread CPU')
    if (type(exclude_policy_cpu_from_workers) is not bool or
            exclude_policy_cpu_from_workers and not (
                v3_voltage_proxy and main_thread_cpu is not None and mode=='stop-proxy' and
                policy_observer is not None and cycles<=bounded_cycles)):
        raise ValueError('I/O worker CPU exclusion requires at most 500 V3 STOP-proxy cycles and a policy CPU pin')
    if pre_cycle_policy_prepare is not None and (not callable(pre_cycle_policy_prepare) or
            mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles):
        raise ValueError('Pre-cycle policy warmup requires at most 500 STOP-proxy cycles with policy inference')
    if post_pin_policy_prepare is not None and (
            not callable(post_pin_policy_prepare) or pre_cycle_policy_prepare is None or
            main_thread_cpu is None or mode!='stop-proxy' or policy_observer is None or cycles>bounded_cycles):
        raise ValueError('Post-pin policy priming requires pre-cycle warmup, CPU pin and at most 500 STOP-proxy cycles')
    if (type(output_dispatch_trace) is not bool or
            output_dispatch_trace and (mode!='stop-proxy' or policy_observer is None)):
        raise ValueError('Output dispatch trace requires STOP proxy with policy inference')
    if (type(defer_gc_during_cycles) is not bool or
            defer_gc_during_cycles and (mode!='stop-proxy' or policy_observer is None or
                                        cycles>bounded_cycles or record_storage!='trace' or
                                        not output_dispatch_trace)):
        raise ValueError('GC deferral requires at most 500 STOP-proxy cycles, trace storage and output dispatch trace')
    records=[];measurements=[];errors=[]
    dispatch_stride=len(_OUTPUT_DISPATCH_FIELDS)
    # Allocate storage before the measured loop; serialize it afterward.
    dispatch_values=(array('Q',[0])*(cycles*dispatch_stride)
                     if output_dispatch_trace else None)
    inference_cpu_values=(array('Q',[0])*(cycles*2)
                          if inference_thread_cpu_trace else None)
    gc_capacity=max(64,cycles*8) if output_dispatch_trace else 0
    if output_dispatch_trace:
        gc_times=array('Q',[0])*gc_capacity
        gc_tids=array('Q',[0])*gc_capacity
        gc_cycles=array('I',[0])*gc_capacity
        gc_generations=array('b',[0])*gc_capacity
        gc_phases=array('b',[0])*gc_capacity
    gc_count=gc_overflow=gc_errors=active_cycle=0
    gc_probe_installed=False
    gc_restore_required=False
    gc_state={'mode':'defer_automatic_during_cycles','before_enabled':None,
              'during_enabled':None,'after_enabled':None,'before_threshold':None,
              'after_threshold':None,'restored':None,'restore_attempts':0,
              'restore_errors':[]} if output_dispatch_trace else None
    def gc_probe(phase,info):
        nonlocal gc_count,gc_overflow,gc_errors
        if gc_count==gc_capacity:
            gc_overflow+=1
            return
        try:
            index=gc_count
            gc_times[index]=clock()
            gc_tids[index]=threading.get_native_id()
            gc_cycles[index]=active_cycle
            gc_generations[index]=info['generation']
            gc_phases[index]=0 if phase=='start' else 1
            gc_count=index+1
        except BaseException:
            # This diagnostic hook must never alter the collector's behavior.
            gc_errors+=1
    startup={'begin_ns':clock(),'end_ns':None,'duration_ms':None,'worker_count':3,'complete':False}
    last_imu=0;previous_release=None;previous_slot=None;cadence_epoch=None
    pool=None;workers_ready=False;storage_failure=None;trace=None
    policy_armed_before_cycles=False
    original_affinity=None;original_worker_masks=None;worker_restore_required=False
    affinity={'requested_cpu':main_thread_cpu,'before':None,'during':None,
              'worker_masks_after_pin':None,'restored':None}
    worker_affinity={'enabled':exclude_policy_cpu_from_workers,
                     'excluded_cpu':main_thread_cpu if exclude_policy_cpu_from_workers else None,
                     'target_mask':None,'workers_before':None,'workers_during':None,
                     'workers_after':None,'restored':None,'restore_errors':[]}
    def read_imu():
        end=clock()+20_000_000
        while clock()<end:
            check();sample=imu_device.read_sample()
            if sample is not None:return sample
            sleep(.0005)
        raise TimeoutError('No new IMU within20ms')
    def exchange(scope,wires,dispatch_base=None,*,before_native=None):
        if dispatch_base is not None:
            dispatch_values[dispatch_base+(5 if scope=='front' else 9)]=clock()
        check()
        if dispatch_base is not None:
            dispatch_values[dispatch_base+(6 if scope=='front' else 10)]=clock()
        try:
            if before_native is not None:
                return sessions[scope].exchange(wires,before_native=before_native)
            return sessions[scope].exchange(wires)
        except native.ExchangeError as error:
            records.append({'failure_scope':scope,'native_failure':error})
            raise
    wires={scope:([native.stop_wire(i) for i in ids] if mode=='stop-proxy' else
        [codec.read_request(i,p) for p in ('position','velocity') for i in ids])
        for scope,ids in dual.SCOPES.items()}
    voltage_wires=({scope:tuple(tuple(wires[scope])+(_READ_WIRES[ids[phase],'voltage'],)
                                  for phase in range(6))
                    for scope,ids in dual.SCOPES.items()} if v3_voltage_proxy else None)
    try:
        if record_storage=='trace':trace=_RecordTrace(cycles,mode,
                                                     voltage_overlap=v3_voltage_overlap)
        pool_options={'max_workers':3,'thread_name_prefix':'native-bench'}
        if worker_initializer is not None:pool_options['initializer']=worker_initializer
        pool=ThreadPoolExecutor(**pool_options)
        _prestart_workers(pool,check)
        workers_ready=True
        startup['end_ns']=clock();startup['complete']=True
        # Torch may create native helper threads on its first policy call. Warm
        # them while the caller still has the full CPU mask; a thread created
        # after pinning the caller would inherit the single-CPU mask.
        if pre_cycle_policy_prepare is not None:
            check()
            pre_cycle_policy_prepare()
            check()
        if main_thread_cpu is not None:
            if not hasattr(os,'sched_getaffinity') or not hasattr(os,'sched_setaffinity'):
                raise RuntimeError('Main-thread affinity is unavailable')
            original_affinity=set(os.sched_getaffinity(0))
            affinity['before']=sorted(original_affinity)
            if main_thread_cpu not in original_affinity or len(original_affinity)<2:
                raise ValueError('Main-thread CPU unavailable or workers already pinned')
            if exclude_policy_cpu_from_workers and len(original_affinity-{main_thread_cpu})<3:
                raise ValueError('I/O worker CPU exclusion requires at least three other available CPUs')
            os.sched_setaffinity(0,{main_thread_cpu})
            affinity['during']=sorted(os.sched_getaffinity(0))
            if affinity['during']!=[main_thread_cpu]:
                raise RuntimeError('Main-thread affinity was not applied')
            affinity['worker_masks_after_pin']=_verify_unpinned_workers(pool,check,original_affinity)
            if exclude_policy_cpu_from_workers:
                original_worker_masks={row['native_tid']:set(row['cpus'])
                                       for row in affinity['worker_masks_after_pin']}
                worker_affinity['workers_before']=affinity['worker_masks_after_pin']
                target=original_affinity-{main_thread_cpu}
                worker_affinity['target_mask']=sorted(target)
                worker_restore_required=True
                check()
                worker_affinity['workers_during']=_transition_worker_affinity(
                    pool,original_worker_masks,target)
                if any(row['error'] is not None for row in worker_affinity['workers_during']):
                    raise RuntimeError('I/O worker affinity setup failed: '+str(
                        [row['error'] for row in worker_affinity['workers_during'] if row['error']]))
                check()
        if post_pin_policy_prepare is not None:
            check()
            post_pin_policy_prepare()
            check()
        # Measured diagnostic ticks accept the real acquisition-completion
        # timestamp as their tick. Arm their schedule after all policy setup,
        # before the first timed release; all sensor/output cycles remain
        # timed and subject to the same deadline checks.
        if policy_observer is not None and getattr(policy_observer,'_measured_diagnostic_ticks',False) is True:
            check()
            policy_observer.arm_run(clock())
            check()
            policy_armed_before_cycles=True
        if output_dispatch_trace:
            gc.callbacks.append(gc_probe)
            gc_probe_installed=True
        if defer_gc_during_cycles:
            gc_state['before_enabled']=gc.isenabled()
            gc_state['before_threshold']=tuple(gc.get_threshold())
            if not gc_state['before_enabled']:
                raise RuntimeError('Automatic GC is already disabled before the diagnostic')
            gc_restore_required=True
            gc.disable()
            gc_state['during_enabled']=gc.isenabled()
            if gc_state['during_enabled']:
                raise RuntimeError('Automatic GC deferral was not applied')
        # Exclude both selected preparation and optional CPU pin verification
        # from the finite measured schedule.
        release=(clock() if policy_armed_before_cycles or pre_cycle_policy_prepare is not None or main_thread_cpu is not None or
                 post_pin_policy_prepare is not None
                 else startup['end_ns'])
        if absolute_epoch_cadence:cadence_epoch=release
        deadline=release+int(cycles*.12*1e9)+2_000_000_000
        for cycle in range(cycles):
            if output_dispatch_trace:active_cycle=cycle+1
            check()
            if clock()>deadline:raise TimeoutError('Finite overall budget exhausted')
            wait_enter=clock();wait_return=wait_enter;wait_calls=0
            if absolute_epoch_cadence:
                slot,release=_absolute_epoch_slot(cadence_epoch,previous_slot,previous_release,clock())
                while clock()<release:
                    check()
                    if deadline_wait is None:sleep(max(0,release-clock())/1e9)
                    else:deadline_wait(release)
                    wait_return=clock();wait_calls+=1
                check()
            elif clock()<release:sleep((release-clock())/1e9)
            actual_release=clock()
            if absolute_epoch_cadence:
                # A sleep can wake a full slot (or more) late. Attribute the
                # work to its actual slot; never run its missed predecessors.
                slot,release=_absolute_epoch_slot(cadence_epoch,previous_slot,
                                                  previous_release,actual_release)
                skipped=slot if previous_slot is None else slot-previous_slot-1
            acquisition_wires=({scope:voltage_wires[scope][cycle%6]
                                for scope in dual.SCOPES} if v3_voltage_proxy and not v3_voltage_overlap
                               else wires)
            voltage_futures={}
            voltage_gate=voltage_cancelled=None
            if v3_voltage_overlap:
                # Record before dispatch so a partially completed voltage read
                # remains attributable to this cycle on every failure path.
                record={'cycle':cycle+1,'acquired':{},'voltage':{},'imu':None,'output':{},
                        'voltage_overlap':{'status':'PENDING_AT_INFERENCE',
                                           'output_allowed':False}}
                if pipeline_key is not None:
                    if v3_voltage_pipeline:
                        voltage_gate=threading.Event();voltage_cancelled=threading.Event()
                    record[pipeline_key]={
                        'status':'PENDING_AT_FEEDBACK','output_allowed':False,
                        **{name+'_ns_by_bus':{} for name in
                           ('feedback_dispatch','feedback_reply_end','feedback_ready',
                            'voltage_dispatch','voltage_reply_end')}}
                record_index=len(records);records.append(record)
                feedback_ready={scope:Future() for scope in dual.SCOPES}
                try:
                    for scope,ids in dual.SCOPES.items():
                        if v3_voltage_pipeline:
                            voltage_futures[scope]=pool.submit(
                                _feedback_then_gated_voltage,exchange,scope,wires[scope],
                                _READ_WIRES[ids[cycle%6],'voltage'],feedback_ready[scope],
                                voltage_gate,voltage_cancelled,record['voltage_pipeline'],clock)
                        elif v3_voltage_fast_pipeline:
                            voltage_futures[scope]=pool.submit(
                                _feedback_then_voltage,exchange,scope,wires[scope],
                                _READ_WIRES[ids[cycle%6],'voltage'],feedback_ready[scope],
                                record['voltage_fast_pipeline'],clock,publish_before_native=True)
                        else:
                            voltage_futures[scope]=pool.submit(
                                _feedback_then_voltage,exchange,scope,wires[scope],
                                _READ_WIRES[ids[cycle%6],'voltage'],feedback_ready[scope])
                    futures=feedback_ready
                    imu_future=pool.submit(read_imu)
                except BaseException:
                    if v3_voltage_pipeline:voltage_cancelled.set();voltage_gate.set()
                    _settle_voltage(voltage_futures,record)
                    _retain_submitted_feedback(feedback_ready,voltage_futures,record)
                    raise
            else:
                futures={s:pool.submit(exchange,s,w) for s,w in acquisition_wires.items()}
                imu_future=pool.submit(read_imu)
            # Retrieve every future before propagating an error, keeping all completed evidence.
            acquired={};failure=None
            for s,f in futures.items():
                try:acquired[s]=f.result()
                except BaseException as e:failure=failure or e
            try:sample=imu_future.result()
            except BaseException as e:sample=None;failure=failure or e
            if v3_voltage_overlap:
                record['acquired']=acquired;record['imu']=sample
            else:
                record={'cycle':cycle+1,'acquired':acquired,'imu':sample,'output':{}}
                record_index=len(records);records.append(record)
            if failure:
                if v3_voltage_pipeline:voltage_cancelled.set();voltage_gate.set()
                if v3_voltage_overlap:_settle_voltage(voltage_futures,record)
                raise failure
            gather_end=clock()
            if pipeline_key is not None:
                record[pipeline_key]['feedback_join_ns']=gather_end
            validation_future=None
            try:
                if sample['read_started_monotonic_ns']<=last_imu:
                    raise ValueError('IMU sample reused across cycles')
                last_imu=sample['read_started_monotonic_ns']
                expected_voltage=({scope:ids[cycle%6] for scope,ids in dual.SCOPES.items()}
                                  if v3_voltage_proxy else None)
                feedback_proof=None
                if v3_voltage_overlap:
                    snapshot,feedback_proof=_validated_feedback_for_voltage(acquired,sample,gather_end)
                else:
                    snapshot=snapshot_from_records({s:x[0] for s,x in acquired.items()},sample,gather_end,
                                                  expected_voltage_by_bus=expected_voltage)
                if v3_voltage_overlap:
                    snapshot['source_flags']['v3_voltage_overlap_pending_at_inference']=True
                    if pipeline_key is not None:
                        # STOP-proxy feedback and IMU are validated before
                        # inference. The fast path may already be reading
                        # voltage on its bus owners at this point.
                        oldest=min(sample['read_started_monotonic_ns'],
                                   *(r.start_ns for value in acquired.values() for r in value[0]))
                        pipeline_hard_end=min(actual_release+PERIOD_NS,oldest+PERIOD_NS)
                        proof=record[pipeline_key]
                        proof['hard_deadline_ns']=pipeline_hard_end
                        snapshot_validated=clock()
                        if snapshot_validated>=pipeline_hard_end:
                            raise TimeoutError('Feedback exceeded 20 ms pipeline hard deadline')
                        proof['feedback_snapshot_validated_ns']=snapshot_validated
                        proof['status']='PENDING_AT_INFERENCE'
                        if v3_voltage_pipeline:
                            proof['voltage_gate_set_ns']=snapshot_validated
                            voltage_gate.set()
                    if v3_voltage_validation_overlap:
                        # The IMU worker is free after imu_future.result(). Its
                        # task waits on the two bus-owned voltage futures, then
                        # validates their immutable records during inference.
                        validation_future=pool.submit(_validate_voltage_during_inference,
                            voltage_futures,acquired,sample,snapshot,expected_voltage,clock,voltage_max_v,
                            feedback_proof)
                prepared=clock()
                if inference_cpu_values is not None:
                    inference_cpu_base=cycle*2
                    inference_cpu_values[inference_cpu_base]=time.thread_time_ns()
                observed=None
                if policy_observer is not None:
                    if cycle==0 and not policy_armed_before_cycles:policy_observer.arm_run(gather_end)
                    observed=policy_observer.consume(snapshot)
                if inference_cpu_values is not None:
                    inference_cpu_values[inference_cpu_base+1]=time.thread_time_ns()
                inferred=clock()
                if v3_voltage_overlap and output_dispatch_trace:
                    infer_thread_cpu_end=time.thread_time_ns()
            except BaseException:
                if v3_voltage_overlap:
                    if v3_voltage_pipeline:voltage_cancelled.set();voltage_gate.set()
                    _settle_voltage(voltage_futures,record)
                    if validation_future is not None:
                        try:validation_future.result()
                        except BaseException as validation_error:
                            record['voltage_validation_error']=(
                                type(validation_error).__name__+': '+str(validation_error))
                raise
            if v3_voltage_overlap:
                # This scheduling comparison is selected only by the existing
                # native release-wait callback. Legacy no-callback diagnostic
                # sequencing and its failure-stage metadata stay unchanged.
                if pipeline_key is not None and deadline_wait is not None:
                    try:
                        record[pipeline_key]['voltage_join_wait']=_await_voltage_ready(
                            voltage_futures,validation_future,deadline_ns=pipeline_hard_end,
                            deadline_wait=deadline_wait,clock=clock,check=check)
                    except BaseException as join_error:
                        # Preserve every completed owner record on the failure
                        # path. This blocking settlement is cleanup, not an
                        # admission for another proxy STOP output batch.
                        _settle_voltage(voltage_futures,record)
                        if validation_future is not None:
                            try:validation_future.result()
                            except BaseException as validation_error:
                                record['voltage_validation_error']=(
                                    type(validation_error).__name__+': '+str(validation_error))
                        record[pipeline_key].update(
                            status='REJECTED_BEFORE_PROXY_STOP',
                            voltage_join_error=type(join_error).__name__+': '+str(join_error))
                        record.pop('observed',None)
                        raise
                voltage_errors=_settle_voltage(voltage_futures,record)
                voltage_wait_end=clock()
                if pipeline_key is not None:
                    record[pipeline_key]['voltage_join_ns']=voltage_wait_end
                validation_started=validation_finished=final_freshness_checked=None
                if v3_voltage_validation_overlap:
                    try:
                        full_voltage,validation_started,validation_finished=(
                            validation_future.result())
                    except BaseException as validation_error:
                        record['voltage_validation_error']=(
                            type(validation_error).__name__+': '+str(validation_error))
                        if voltage_errors:raise voltage_errors[0]
                        raise
                    if voltage_errors:raise voltage_errors[0]
                    if pipeline_key is None:
                        try:
                            final_freshness_checked=_verify_voltage_final_freshness(
                                acquired,record['voltage'],sample,snapshot,full_voltage,
                                expected_voltage,clock,voltage_max_v,feedback_proof)
                        except BaseException as freshness_error:
                            record['voltage_freshness_error']=(
                                type(freshness_error).__name__+': '+str(freshness_error))
                            raise
                        verified_at=final_freshness_checked
                    else:
                        # The worker completed frame/mutation/age validation.
                        # This route has a second, actual dispatch gate below:
                        # walk fourteen timestamps once there, immediately
                        # before submitting STOP rather than twice in succession.
                        verified_at=_verify_joined_voltage_proof(
                            acquired,record['voltage'],sample,snapshot,full_voltage,feedback_proof,clock)
                else:
                    if voltage_errors:raise voltage_errors[0]
                    full_voltage,verified_at=_verify_voltage_after_inference(
                        acquired,record['voltage'],sample,snapshot,expected_voltage,clock,voltage_max_v,
                        feedback_proof)
                record['voltage_overlap']={
                    'status':'VALIDATED_BEFORE_PROXY_STOP',
                    'feedback_ready_ns':gather_end,'inference_end_ns':inferred,
                    'voltage_wait_end_ns':voltage_wait_end,
                    'voltage_verified_ns':verified_at,
                    'voltage_reply_end_ns_by_bus':{
                        scope:max(r.received_ns for r in value[0])
                        for scope,value in record['voltage'].items()},
                    'voltage_v_by_bus':{
                        scope:row['value_v'] for scope,row in full_voltage['voltage_by_bus'].items()},
                    'range_v':[35.,voltage_max_v], 'observer_snapshot_voltage_pending':True,
                    'output_allowed':False}
                if v3_voltage_validation_overlap:
                    record['voltage_overlap'].update(
                        validation_started_ns=validation_started,
                        validation_finished_ns=validation_finished,
                        final_freshness_checked_ns=final_freshness_checked)
                if pipeline_key is not None:
                    record[pipeline_key].update(
                        status='POST_INFERENCE_VALIDATED',
                        inference_end_ns=inferred,post_inference_verified_ns=verified_at,
                        range_v=[35.,voltage_max_v])
            if policy_observer is not None:record['observed']=observed
            if mode=='stop-proxy' and policy_observer is not None:
                if output_dispatch_trace:
                    dispatch_base=cycle*dispatch_stride
                    dispatch_values[dispatch_base]=inferred
                    dispatch_values[dispatch_base+13]=(
                        infer_thread_cpu_end if v3_voltage_overlap else time.thread_time_ns())
                    dispatch_values[dispatch_base+1]=clock()
                check()
                if pipeline_key is not None:
                    try:
                        # Mirror a pre-Type1 gate at the actual proxy
                        # dispatch point. STOP is the only possible output here.
                        final_gate=_verify_voltage_final_freshness(
                            acquired,record['voltage'],sample,snapshot,full_voltage,
                            expected_voltage,clock,voltage_max_v,feedback_proof)
                        if final_gate>=pipeline_hard_end:
                            raise TimeoutError('Voltage pipeline exceeded 20 ms hard deadline before proxy STOP')
                    except BaseException as gate_error:
                        record[pipeline_key]['status']='REJECTED_BEFORE_PROXY_STOP'
                        record[pipeline_key]['final_gate_error']=(
                            type(gate_error).__name__+': '+str(gate_error))
                        record.pop('observed',None)
                        raise
                    record[pipeline_key].update(
                        status='VALIDATED_BEFORE_PROXY_STOP',voltage_verified_ns=final_gate)
                    record['voltage_overlap']['voltage_verified_ns']=final_gate
                    record['voltage_overlap']['final_freshness_checked_ns']=final_gate
                if output_dispatch_trace:
                    dispatch_values[dispatch_base+2]=clock()
                    futures={}
                    for scope,scope_wires in wires.items():
                        futures[scope]=pool.submit(exchange,scope,scope_wires,dispatch_base)
                        dispatch_values[dispatch_base+(3 if scope=='front' else 4)]=clock()
                    dispatch_values[dispatch_base+14]=time.thread_time_ns()
                else:
                    futures={s:pool.submit(exchange,s,w) for s,w in wires.items()}
                failure=None
                for s,f in futures.items():
                    try:record['output'][s]=f.result()
                    except BaseException as e:failure=failure or e
                if failure:raise failure
                if v3_voltage_fast_pipeline:
                    try:
                        stop_reply_ends,stop_verified=_verify_final_proxy_stop_records(
                            record['output'],clock)
                    except BaseException as stop_error:
                        record['voltage_fast_pipeline']['status']='FINAL_STOP_REPLY_REJECTED'
                        record['voltage_fast_pipeline']['stop_reply_error']=(
                            type(stop_error).__name__+': '+str(stop_error))
                        raise
                    record['voltage_fast_pipeline'].update(
                        stop_reply_end_ns_by_bus=stop_reply_ends,
                        stop_reply_count=12,stop_reply_verified_ns=stop_verified)
                if output_dispatch_trace:
                    for scope,(scope_records,scope_stats) in record['output'].items():
                        offset=7 if scope=='front' else 11
                        dispatch_values[dispatch_base+offset]=scope_stats.begin_ns
                        dispatch_values[dispatch_base+offset+1]=scope_records[0].start_ns
            output=record['output']
            if record_storage=='trace':
                # Copy bounded native buffers into storage allocated before
                # worker startup. The copy and release precede cycle_end_ns.
                try:
                    traced=trace.capture(cycle,record)
                    timing_values=_timing_scalars(acquired,sample,output)
                except BaseException as error:
                    storage_failure={'storage_index':record_index,'cycle':cycle+1,
                        'error':type(error).__name__+': '+str(error)}
                    raise
                records[record_index]=traced
                record=traced=acquired=sample=output=snapshot=imu_future=f=None
                voltage_futures=validation_future=full_voltage=observed=None
                feedback_proof=None
                futures.clear()
            elif record_storage=='encoded':
                # Every output future is settled. Keep the raw in-flight row
                # until encoding succeeds, and include encoding/release in the
                # complete cycle rather than retaining its nested trees.
                try:
                    serialized=_serialize([record])
                    encoded=json.dumps(serialized,allow_nan=False)
                    # Timing needs only scalar timestamps. Drop owned record,
                    # native buffers, futures and observation trees before end.
                    timing_acquired={s:([_TimingRecord(r['start_ns'],r['finish_ns'],r['received_ns'])
                        for r in value['records']],None) for s,value in serialized[0]['acquired'].items()}
                    timing_output={s:([_TimingRecord(r['start_ns'],r['finish_ns'],r['received_ns'])
                        for r in value['records']],None) for s,value in serialized[0]['output'].items()}
                    timing_sample={k:sample[k] for k in
                        ('read_started_monotonic_ns','read_finished_monotonic_ns')}
                except BaseException as error:
                    storage_failure={'storage_index':record_index,'cycle':cycle+1,
                        'error':type(error).__name__+': '+str(error)}
                    raise
                records[record_index]=encoded
                record=serialized=acquired=sample=output=snapshot=imu_future=f=None
                futures.clear()
            else:
                timing_acquired,timing_sample,timing_output=acquired,sample,output
            end=clock()
            if record_storage=='trace':
                timing=_timing_row_from_scalars(*timing_values,release_ns=actual_release,
                    gather_end_ns=gather_end,prepare_end_ns=prepared,infer_end_ns=inferred,cycle_end_ns=end)
            else:
                timing=timing_row(timing_acquired,timing_sample,timing_output,release_ns=actual_release,
                    gather_end_ns=gather_end,prepare_end_ns=prepared,infer_end_ns=inferred,cycle_end_ns=end)
            timing['release_lateness_ms']=max(0,actual_release-release)/1e6
            timing['actual_release_interval_ms']=(actual_release-previous_release)/1e6 if previous_release is not None else None
            timing['timing_phase']='startup' if cycle<startup_cycle_allowance else 'steady'
            if absolute_epoch_cadence:
                timing.update(cadence_slot=slot,scheduled_release_ns=release,
                    skipped_slots_before=skipped,
                    wait_enter_ns=wait_enter,wait_return_ns=wait_return,wait_calls=wait_calls,
                    scheduled_completion_slack_ms=(release+PERIOD_NS-end)/1e6,
                    start_interval_over_20ms=(previous_release is not None and
                                              actual_release-previous_release>PERIOD_NS))
            measurements.append(timing);previous_release=actual_release
            if absolute_epoch_cadence:previous_slot=slot
            if output_dispatch_trace:active_cycle=0
            # Misses are retained; never run a catch-up burst or backdate source times.
            if not absolute_epoch_cadence:release=max(actual_release+PERIOD_NS,end)
    except BaseException as error:
        errors.append(type(error).__name__+': '+str(error))
        if policy_observer is not None:policy_observer.invalidate(errors[-1])
    finally:
        if gc_probe_installed:
            try:gc.callbacks.remove(gc_probe)
            except BaseException as error:
                errors.append('GC probe removal: '+type(error).__name__+': '+str(error))
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if gc_restore_required:
            for attempt in range(3):
                gc_state['restore_attempts']=attempt+1
                try:gc.set_threshold(*gc_state['before_threshold'])
                except BaseException as error:
                    gc_state['restore_errors'].append('threshold: '+repr(error))
                try:
                    if gc_state['before_enabled']:gc.enable()
                    else:gc.disable()
                except BaseException as error:
                    gc_state['restore_errors'].append('enabled state: '+repr(error))
                try:
                    gc_state['after_enabled']=gc.isenabled()
                    gc_state['after_threshold']=tuple(gc.get_threshold())
                    gc_state['restored']=(gc_state['after_enabled']==gc_state['before_enabled'] and
                        gc_state['after_threshold']==gc_state['before_threshold'])
                except BaseException as error:
                    gc_state['restore_errors'].append('readback: '+repr(error))
                    gc_state['restored']=False
                if gc_state['restored']:break
            if not gc_state['restored']:
                errors.append('Automatic GC state/threshold restoration unconfirmed')
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if worker_restore_required:
            try:
                worker_affinity['workers_after']=_transition_worker_affinity(
                    pool,original_worker_masks,None)
                worker_affinity['restore_errors']=[row['error'] for row in worker_affinity['workers_after']
                                                    if row['error'] is not None]
                worker_affinity['restored']=not worker_affinity['restore_errors']
            except BaseException as error:
                worker_affinity['restored']=False
                worker_affinity['restore_errors'].append(type(error).__name__+': '+str(error))
            if not worker_affinity['restored']:
                errors.append('I/O worker affinity restoration unconfirmed')
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if original_affinity is not None:
            try:
                os.sched_setaffinity(0,original_affinity)
                affinity['restored']=set(os.sched_getaffinity(0))==original_affinity
                if not affinity['restored']:raise RuntimeError('Main-thread affinity restoration differs')
            except BaseException as error:
                affinity['restored']=False
                errors.append(type(error).__name__+': '+str(error))
                if policy_observer is not None:policy_observer.invalidate(errors[-1])
        if startup['end_ns'] is None:startup['end_ns']=clock()
        startup['duration_ms']=(startup['end_ns']-startup['begin_ns'])/1e6
        if pool is not None:pool.shutdown(wait=workers_ready,cancel_futures=not workers_ready)
    summary=policy_observer.finish() if policy_observer is not None else None
    report={'status':'COMPLETE_DIAGNOSTIC' if not errors else 'ABORTED',
        'mode':mode,'errors':errors,'cycles_requested':cycles,'cycles_completed':len(measurements),
        'v3_voltage_proxy':v3_voltage_proxy,
        'voltage_max_v':voltage_max_v,'voltage_range_v':[35.,voltage_max_v],
        'motor_enable_sent':False,'learned_targets_sent':False,'approved_for_runtime':False,
        'full_controller_50Hz_verified':False,'worker_startup':startup,
        'main_thread_affinity':affinity,'worker_affinity':worker_affinity,
        'measurements':measurements,'observer':summary,
        'distributions_ms':{k:distribution([r[k] for r in measurements if r[k] is not None]) for k in
            ('acquisition_ms','prepare_ms','inference_ms','oldest_input_to_final_host_write_ms',
             'oldest_input_to_last_reply_ms','whole_iteration_ms','release_lateness_ms','actual_release_interval_ms')},
        'host_deadline_misses':sum(not r['host_deadline_met'] for r in measurements)
            if mode=='stop-proxy' and policy_observer is not None else None,
        'iteration_deadline_misses':sum(not r['iteration_deadline_met'] for r in measurements),
        'release_lateness_over_1ms':sum(r['release_lateness_ms']>1. for r in measurements),
        'release_intervals_over_21ms':sum((r['actual_release_interval_ms'] or 0)>21. for r in measurements)}
    if v3_voltage_overlap:
        report['v3_voltage_overlap']={'enabled':True,'voltage_range_v':[35.,voltage_max_v],
            'validation_overlap_enabled':v3_voltage_validation_overlap,
            'voltage_dispatch_schedule':('after_complete_feedback_imu_snapshot'
                                         if v3_voltage_pipeline else 'after_each_bus_feedback'),
            'observer_snapshot_voltage_pending':True,
            'voltage_verified_before_proxy_stop':report['status']=='COMPLETE_DIAGNOSTIC',
            'timing_fields':{
                'acquisition_ms':'feedback6_per_bus_and_imu_gather; voltage may run concurrently',
                'input_latest_reply_ns':'latest feedback reply or IMU read finish; excludes voltage replies',
                'voltage_verified_ns':'records[].voltage_overlap.voltage_verified_ns',
                'whole_iteration_ms':'actual release through voltage validation, STOP replies, and trace capture'},
            'diagnostic_only':True,'motor_output_allowed':False}
        if v3_voltage_validation_overlap:
            report['v3_voltage_overlap']['timing_fields'].update(
                validation_started_ns='records[].voltage_overlap.validation_started_ns',
                validation_finished_ns='records[].voltage_overlap.validation_finished_ns',
                final_freshness_checked_ns='records[].voltage_overlap.final_freshness_checked_ns')
    if v3_voltage_pipeline:
        report['v3_voltage_pipeline']={
            'enabled':True,'schema':'feedback-then-voltage-proxy-v1',
            'period_ns':PERIOD_NS,'feedback_gate_before_voltage':True,
            'pre_gate_proof':'complete native STOP feedback, finite IMU, causal timestamps and 20 ms freshness',
            'active_feedback_safety_equivalent':False,
            'voltage_verified_before_proxy_stop':report['status']=='COMPLETE_DIAGNOSTIC',
            'timing_fields':{name:'records[].voltage_pipeline.'+name for name in (
                'feedback_dispatch_ns_by_bus','feedback_reply_end_ns_by_bus',
                'feedback_ready_ns_by_bus','feedback_join_ns','hard_deadline_ns',
                'voltage_gate_set_ns',
                'voltage_dispatch_ns_by_bus','voltage_reply_end_ns_by_bus',
                'voltage_join_ns','post_inference_verified_ns','voltage_verified_ns',
                'inference_end_ns')},
            'diagnostic_only':True,'motor_output_allowed':False,
            'learned_targets_sent':False,
            'independent_emergency_stop_available':False,
            'failure_behavior':'Abort normal STOP-proxy batch; no Type1 path; native session errors poison the session'}
    if v3_voltage_fast_pipeline:
        report['v3_voltage_fast_pipeline']={
            'enabled':True,'schema':'immediate-feedback-voltage-proxy-v1',
            'period_ns':PERIOD_NS,'voltage_dispatch_schedule':'after_each_bus_feedback',
            'feedback_publication':'after_voltage_native_preparation',
            'voltage_may_precede_global_feedback_validation':True,
            'active_feedback_safety_equivalent':False,
            'voltage_verified_before_proxy_stop':report['status']=='COMPLETE_DIAGNOSTIC',
            'timing_fields':{name:'records[].voltage_fast_pipeline.'+name for name in (
                'feedback_dispatch_ns_by_bus','feedback_reply_end_ns_by_bus',
                'feedback_ready_ns_by_bus','feedback_published_ns_by_bus',
                'feedback_join_ns','feedback_snapshot_validated_ns',
                'hard_deadline_ns','voltage_dispatch_ns_by_bus','voltage_reply_end_ns_by_bus',
                'voltage_join_ns','post_inference_verified_ns','voltage_verified_ns',
                'inference_end_ns','stop_reply_end_ns_by_bus','stop_reply_count',
                'stop_reply_verified_ns')},
            'diagnostic_only':True,'motor_output_allowed':False,
            'learned_targets_sent':False,'independent_emergency_stop_available':False,
            'failure_behavior':'Abort normal STOP-proxy batch; no Type1 path; native session errors poison the session'}
    report['steady_timing']=_steady_timing_summary(measurements,cycles,startup_cycle_allowance,
                                                report['status']=='COMPLETE_DIAGNOSTIC')
    if absolute_epoch_cadence:
        skipped_total=sum(r['skipped_slots_before'] for r in measurements)
        over_20=sum(r['start_interval_over_20ms'] for r in measurements)
        report['absolute_epoch_schedule']={
            'enabled':True,'epoch_ns':cadence_epoch,'period_ns':PERIOD_NS,
            'minimum_start_separation_ns':ABSOLUTE_MIN_START_SEPARATION_NS,
            'slots_skipped':skipped_total,'start_intervals_over_20ms':over_20,
            'strict_start_interval_20ms_met':bool(
                report['status']=='COMPLETE_DIAGNOSTIC' and len(measurements)==cycles and
                len(measurements)>1 and skipped_total==0 and over_20==0),
            'diagnostic_only':True,'learned_targets_sent':False}
    if output_dispatch_trace:
        report['output_dispatch_trace']={'schema':'native-output-dispatch-v1',
            'main_native_tid':threading.get_native_id(),
            'fields':list(_OUTPUT_DISPATCH_FIELDS),
            'rows':[list(dispatch_values[i*dispatch_stride:(i+1)*dispatch_stride])
                    for i in range(len(measurements))],
            'gc_events':[{'monotonic_ns':gc_times[i], 'native_tid':gc_tids[i],
                          'cycle':gc_cycles[i], 'generation':gc_generations[i],
                          'phase':'start' if gc_phases[i]==0 else 'stop'}
                         for i in range(gc_count)],
            'gc_overflow':gc_overflow,'gc_probe_errors':gc_errors}
    if inference_cpu_values is not None:
        cpu_rows=[]
        for index,measurement in enumerate(measurements):
            begin=inference_cpu_values[index*2]
            finish=inference_cpu_values[index*2+1]
            cpu_ns=finish-begin
            wall_ns=measurement['infer_end_ns']-measurement['prepare_end_ns']
            cpu_rows.append([index+1,begin,finish,cpu_ns,wall_ns,wall_ns-cpu_ns])
        report['inference_thread_cpu_trace']={
            'schema':'native-inference-thread-cpu-v1',
            'clock':'time.thread_time_ns',
            'scope':('collector main thread policy phase; arm_run completed before timed release'
                     if policy_armed_before_cycles else
                     'collector main thread policy phase; cycle 1 includes arm_run'),
            'helper_thread_cpu_included':False,
            'main_native_tid':threading.get_native_id(),
            'fields':['cycle','thread_cpu_begin_ns','thread_cpu_end_ns',
                      'thread_cpu_ns','inference_wall_ns','wall_minus_thread_cpu_ns'],
            'rows':cpu_rows}
    if defer_gc_during_cycles:report['cycle_gc_defer']=gc_state
    if record_storage=='encoded':
        report['record_storage']={'mode':'encoded','encoding_inside_whole_iteration':True,
            'decoded_after_collection':True,'completed_encoded_rows':sum(type(r) is str for r in records)}
        if storage_failure is not None:report['record_storage_failure']=storage_failure
    elif record_storage=='trace':
        report['record_storage']={'mode':'trace','copy_inside_whole_iteration':True,
            'serialized_after_collection':True,'capacity_cycles':cycles,
            'allocated_bytes':trace.allocated_bytes if trace is not None else 0,
            'pretouched_before_release_bytes':trace.pretouched_bytes if trace is not None else 0,
            'completed_trace_rows':sum(type(r) is _TraceRow for r in records)}
        if storage_failure is not None:report['record_storage_failure']=storage_failure
    return report,records


def _serialize(records):
    result=[]
    for row in records:
        if type(row) is _TraceRow:
            result.append(row.serialize());continue
        if type(row) is str:
            # Successful encoded rows are decoded only after collection. An
            # abort can also leave ordinary partial/native-failure rows here.
            decoded=json.loads(row)
            if type(decoded) is not list or len(decoded)!=1 or type(decoded[0]) is not dict:
                raise ValueError('Invalid encoded diagnostic record')
            result.extend(decoded)
            continue
        if 'native_failure' in row:
            e=row['native_failure'];result.append({'failure_scope':row['failure_scope'],
                'error':str(e),**native.exchange_evidence(e.records,e.stats)});continue
        phases=('acquired','voltage','output') if 'voltage' in row else ('acquired','output')
        result.append({**{k:v for k,v in row.items() if k not in phases},
            **{k:{s:native.exchange_evidence(*r) for s,r in row[k].items()} for k in phases}})
    return result


def _storage_invalid_fields(value):
    """Bounded failure inspection; never invoke arbitrary value repr/copy hooks."""
    issues=[];active=set();left=2000;truncated=False
    def typename(item):
        return type.__getattribute__(type(item),'__name__')[:128]
    def representation(item):
        kind=type(item)
        if kind is str:return str.__getitem__(item,slice(0,256))
        if kind is float:return float.__repr__(item)
        if kind is int:return '<integer bits='+str(int.bit_length(item))+'>'
        if kind is bool or item is None:return str(item)
        if kind is bytes:return bytes.hex(item[:64])
        return '<'+typename(item)+'>'
    def add(item,path):
        issues.append({'path':path[:256],'type':typename(item),'representation':representation(item)[:256]})
    def visit(recurse,item,path,depth):
        nonlocal left,truncated
        if left<=0 or depth>24 or len(issues)>=16:
            truncated=True;return
        left-=1
        if item is None or isinstance(item,(str,int,bool)):return
        if isinstance(item,float):
            if not math.isfinite(float.__float__(item)):add(item,path)
            return
        if not isinstance(item,(dict,list,tuple)):
            add(item,path);return
        identity=id(item)
        if identity in active:
            add(item,path);return
        active.add(identity)
        try:
            entries=dict.items(item) if isinstance(item,dict) else enumerate(item)
            for key,child in entries:
                if left<=0 or len(issues)>=16:
                    truncated=True;break
                if isinstance(item,dict):
                    if not (key is None or isinstance(key,(str,int,float,bool))):add(key,path+'.<key>')
                    elif isinstance(key,float) and not math.isfinite(float.__float__(key)):add(key,path+'.<key>')
                    label=(str.__getitem__(key,slice(0,80)) if isinstance(key,str) else representation(key))
                    child_path=path+'.'+label
                else:child_path=path+'['+str(key)+']'
                recurse(recurse,child,child_path,depth+1)
        finally:active.remove(identity)
    visit(visit,value,'$',0)
    return issues,truncated


def _storage_failure_artifact(row,index,error):
    metadata=({k:v for k,v in row.items() if k not in ('acquired','voltage','output','native_failure')}
              if type(row) is dict else row.metadata if type(row) is _TraceRow else row)
    invalid,truncated=_storage_invalid_fields(metadata)
    evidence={}
    if type(row) is _TraceRow:
        for phase,scopes in row.phase_scopes.items():
            evidence[phase]={}
            for scope in scopes:
                try:evidence[phase][scope]=row.storage.evidence(row.cycle_index,phase,scope)
                except Exception as failure:
                    evidence[phase][scope]={'evidence_error':type(failure).__name__+': '+str(failure)[:256]}
    elif type(row) is dict:
        for key in ('acquired','voltage','output'):
            if key not in row:continue
            evidence[key]={}
            for scope,result in row.get(key,{}).items():
                try:
                    if scope not in dual.SCOPES or not 1<=len(result[0])<=12:
                        raise ValueError('Invalid bounded native evidence')
                    evidence[key][scope]=native.exchange_evidence(*result)
                except Exception as failure:
                    evidence[key][scope]={'evidence_error':type(failure).__name__+': '+str(failure)[:256]}
        if isinstance(row.get('native_failure'),native.ExchangeError):
            failure=row['native_failure']
            if 1<=len(failure.records)<=12:
                evidence['native_failure']={'failure_scope':row.get('failure_scope'),
                    'error':str(failure)[:512],**native.exchange_evidence(failure.records,failure.stats)}
    return {'storage_index':index,'cycle':metadata.get('cycle') if type(metadata) is dict else None,
        'error':error[:512],'invalid_json_fields':invalid,'inspection_truncated':truncated,
        'native_evidence':evidence,'record_is_successful':False,'output_allowed':False,
        'approved_for_runtime':False}


def _encoded_records_for_output(records,encoding_failure=None):
    """Post-run conversion with explicit bounded failed-row evidence, never approval."""
    result=[];failures=[]
    for index,row in enumerate(records):
        try:
            value=_serialize([row])[0]
            if type(row) is not str:json.dumps(value,allow_nan=False)
        except Exception as error:
            failure=_storage_failure_artifact(row,index,type(error).__name__+': '+str(error))
            failures.append(failure)
            result.append({'cycle':failure['cycle'],'status':'RECORD_STORAGE_FAILED',
                'record_storage_failure_index':len(failures)-1,'native_evidence':failure['native_evidence'],
                'output_allowed':False,'approved_for_runtime':False})
        else:
            result.append(value)
            if encoding_failure is not None and encoding_failure['storage_index']==index:
                failures.append(_storage_failure_artifact(row,index,encoding_failure['error']))
    return result,failures


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true');p.add_argument('--supported-disabled',action='store_true')
    p.add_argument('--provenance-mode',choices=('supported-geometric-preload-5s-v1',
                   'human-supported-partial-current-hold-audio-8s-v1'),
                   help='Pin current sources for the named supported scope in this disabled diagnostic; requires --power-epoch and grants no output approval')
    p.add_argument('--power-epoch',
                   help='Explicit current motor-power epoch assertion for --provenance-mode; never inferred from an earlier report')
    p.add_argument('--mode',choices=('type17','stop-proxy'),default='type17')
    p.add_argument('--v3-voltage-proxy',action='store_true',
                   help='Disabled-only 26-request proxy: six STOP feedback plus one rotating voltage read per bus, then six STOP; never Type1 or motor enable')
    p.add_argument('--v3-voltage-overlap',action='store_true',
                   help='Diagnostic-only: infer from six feedback replies while each bus reads its separate voltage; verify selected voltage range on both buses before proxy STOP')
    p.add_argument('--voltage-max-v',type=int,choices=(42,43),default=42,
                   help='Explicit voltage upper bound; lower bound remains 35 V, default upper bound 42 V')
    p.add_argument('--v3-voltage-validation-overlap',action='store_true',
                   help='Diagnostic-only: validate completed voltage replies on the released IMU worker during inference; recheck all input timestamps before proxy STOP')
    p.add_argument('--v3-voltage-pipeline',action='store_true',
                   help='Diagnostic-only: keep each bus owner gated after six feedback replies; release separate voltage reads after the feedback/IMU snapshot, then validate before proxy STOP')
    p.add_argument('--v3-voltage-fast-pipeline',action='store_true',
                   help='Diagnostic-only: each bus owner reads voltage immediately after its six feedback replies; retain per-cycle timing proof and validate before proxy STOP')
    p.add_argument('--cycles',type=int,choices=range(1,3001),metavar='1..3000')
    p.add_argument('--startup-cycle-allowance',type=int,choices=(0,1),default=0,
                   help='Record but separately judge first startup cycle; --cycles 501 gives one startup plus 500 steady cycles')
    p.add_argument('--request-window',type=int,choices=(1,2,3),default=3,
                   help='Maximum outstanding telemetry/STOP requests per bus')
    p.add_argument('--request-gap-us',type=int,default=600,metavar='600..5000',
                   help='Minimum gap after each write, in microseconds')
    p.add_argument('--timer-slack-ns',type=int,choices=thread_timer_slack.CHOICES_NS,
                   help='Opt-in per-thread Linux timer slack during three-worker collection only')
    p.add_argument('--main-thread-cpu',type=int,
                   help='Opt-in diagnostic: pin only the policy thread after I/O workers start; restore on exit')
    p.add_argument('--exclude-policy-cpu-from-workers',action='store_true',
                   help='Opt-in V3 STOP-proxy diagnostic: keep all three I/O workers off the pinned policy CPU; restore every worker mask')
    p.add_argument('--output-dispatch-trace',action='store_true',
                   help='Opt-in timestamps for output guard, submits and worker dispatch; collected in memory')
    p.add_argument('--inference-thread-cpu-trace',action='store_true',
                   help='Opt-in per-cycle main-thread CPU versus wall time for bounded 26-request STOP-proxy inference')
    p.add_argument('--absolute-epoch-cadence',action='store_true',
                   help='Diagnostic-only fixed 20 ms start slots; skip missed slots, never catch up in a burst')
    p.add_argument('--release-spin-us',type=int,choices=(200,500),
                   help='Opt-in native absolute wait with bounded final CPU spin; requires absolute-epoch cadence')
    p.add_argument('--defer-gc-during-cycles',action='store_true',
                   help='Opt-in bounded STOP-proxy comparison: defer automatic GC for at most 500 traced cycles')
    p.add_argument('--single-thread-math',action='store_true',
                   help='Opt in to OMP/OPENBLAS/MKL thread counts of 1 before NumPy/Torch import')
    p.add_argument('--require-pinned-fast-model',action='store_true',
                   help='Reject timing runs unless the SHA-pinned scalar C++ model is explicitly selected')
    p.add_argument('--setup-gc',choices=('before-warmup',),
                   help='Opt-in full garbage collection after UID/IMU startup and before policy warmup')
    p.add_argument('--pre-cycle-policy-warmup-calls',type=int,choices=range(10,101),metavar='10..100',
                   help='Opt-in bounded synthetic policy warmup after workers, before optional CPU pin and timed cycles')
    p.add_argument('--post-pin-policy-prime-calls',type=int,choices=range(1,101),metavar='1..100',
                   help='Opt-in synthetic model calls on reused observer input tensors after CPU pin, before timed cycles')
    p.add_argument('--record-storage',choices=('objects','encoded','trace'),default='objects',
                   help='Keep objects, encode rows, or copy into a preallocated native trace within each measured cycle')
    p.add_argument('--acquisition-only',action='store_true')
    p.add_argument('--compare-feedback',action='store_true',
                   help='Separate Type17/STOP-Type2 comparison; no IMU or inference')
    for name in ('front-port','rear-port','expected-uids','library','output','calibration','mount','gyro-bias',
                 'bundle','native-policy-manifest','native-policy-manifest-sha256',
                 'view-cache-manifest','view-cache-manifest-sha256',
                 'scalar-step-manifest','scalar-step-manifest-sha256'):
        p.add_argument('--'+name)
    p.add_argument('--h-hypothesis',type=int,choices=(0,1),default=0)
    args=p.parse_args(argv)
    try:math_startup=math_threads.configure_single_thread_math(args.single_thread_math)
    except math_threads.MathThreadStartupError as error:p.error(str(error))
    if not 600<=args.request_gap_us<=5000:
        p.error('--request-gap-us must be 600..5000')
    view_cache_selected=args.view_cache_manifest is not None or args.view_cache_manifest_sha256 is not None
    scalar_step_selected=args.scalar_step_manifest is not None or args.scalar_step_manifest_sha256 is not None
    if args.require_pinned_fast_model and not (scalar_step_selected and
            args.native_policy_manifest and args.native_policy_manifest_sha256 and
            args.mode=='stop-proxy' and not args.acquisition_only and not args.compare_feedback):
        p.error('--require-pinned-fast-model needs a full STOP-proxy inference run with pinned scalar and baseline manifests')
    if scalar_step_selected:
        if not args.scalar_step_manifest or not args.scalar_step_manifest_sha256:
            p.error('--scalar-step-manifest and --scalar-step-manifest-sha256 must be supplied together')
        if view_cache_selected:
            p.error('Scalar-step and cached-view selections are mutually exclusive')
        if args.mode!='stop-proxy' or args.acquisition_only or args.compare_feedback:
            p.error('Scalar-step diagnostic requires STOP proxy with policy inference')
        if not args.native_policy_manifest or not args.native_policy_manifest_sha256:
            p.error('Scalar-step diagnostic requires the pinned native baseline')
    if view_cache_selected:
        if not args.view_cache_manifest or not args.view_cache_manifest_sha256:
            p.error('--view-cache-manifest and --view-cache-manifest-sha256 must be supplied together')
        if args.acquisition_only or args.compare_feedback:
            p.error('Cached-view diagnostic requires policy inference, without acquisition-only or feedback comparison')
        if not args.native_policy_manifest or not args.native_policy_manifest_sha256:
            p.error('Cached-view diagnostic requires --native-policy-manifest and --native-policy-manifest-sha256')
    if args.setup_gc is not None and (args.acquisition_only or args.compare_feedback):
        p.error('--setup-gc requires policy inference, without acquisition-only or feedback comparison')
    if args.record_storage!='objects' and args.compare_feedback:
        p.error('--record-storage encoded/trace requires the diagnostic collector without feedback comparison')
    if args.output_dispatch_trace and (args.mode!='stop-proxy' or args.acquisition_only or
                                       args.compare_feedback):
        p.error('--output-dispatch-trace requires STOP proxy with policy inference')
    if args.cycles is None:args.cycles=3 if args.compare_feedback else 20
    if args.startup_cycle_allowance and (args.mode!='stop-proxy' or args.acquisition_only or
            args.compare_feedback or not 2<=args.cycles<=501):
        p.error('--startup-cycle-allowance requires one startup and 1..500 STOP-proxy inference cycles')
    bounded_cycles=500+args.startup_cycle_allowance
    if scalar_step_selected and args.cycles>bounded_cycles:
        p.error('Scalar-step diagnostic permits at most 500 steady cycles')
    if args.release_spin_us is not None and not args.absolute_epoch_cadence:
        p.error('--release-spin-us requires --absolute-epoch-cadence')
    if args.pre_cycle_policy_warmup_calls is not None and (
            args.mode!='stop-proxy' or args.acquisition_only or args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--pre-cycle-policy-warmup-calls requires at most 500 STOP-proxy cycles with policy inference')
    if args.post_pin_policy_prime_calls is not None and (
            args.pre_cycle_policy_warmup_calls is None or args.main_thread_cpu is None or
            args.mode!='stop-proxy' or args.acquisition_only or args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--post-pin-policy-prime-calls requires pre-cycle policy warmup, a pinned policy CPU and at most 500 STOP-proxy cycles')
    if args.defer_gc_during_cycles and (args.mode!='stop-proxy' or args.acquisition_only or
                                        args.compare_feedback or args.cycles>bounded_cycles or
                                        args.record_storage!='trace' or not args.output_dispatch_trace):
        p.error('--defer-gc-during-cycles requires at most 500 STOP-proxy cycles, trace storage and --output-dispatch-trace')
    if args.compare_feedback and (args.mode!='stop-proxy' or not args.acquisition_only):
        p.error('Feedback comparison requires --mode stop-proxy --acquisition-only')
    if args.compare_feedback and args.cycles>5:
        p.error('Feedback comparison uses 1..5 cycles')
    if args.compare_feedback and args.timer_slack_ns is not None:
        p.error('--timer-slack-ns requires the three-worker diagnostic collector')
    if args.v3_voltage_proxy and (args.mode!='stop-proxy' or args.acquisition_only or
                                  args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--v3-voltage-proxy requires at most 500 full STOP-proxy inference cycles')
    if args.v3_voltage_overlap and (not args.v3_voltage_proxy or args.mode!='stop-proxy' or
                                    args.acquisition_only or args.compare_feedback or
                                    args.cycles>bounded_cycles or args.record_storage!='trace'):
        p.error('--v3-voltage-overlap requires --v3-voltage-proxy, trace storage and at most 500 full STOP-proxy inference cycles')
    if args.v3_voltage_validation_overlap and not args.v3_voltage_overlap:
        p.error('--v3-voltage-validation-overlap requires --v3-voltage-overlap')
    if args.v3_voltage_pipeline and not (args.v3_voltage_proxy and args.v3_voltage_overlap and
                                         args.v3_voltage_validation_overlap and
                                         args.mode=='stop-proxy' and not args.acquisition_only and
                                         not args.compare_feedback and args.cycles<=bounded_cycles and
                                         args.record_storage=='trace'):
        p.error('--v3-voltage-pipeline requires bounded V3 STOP-proxy trace with --v3-voltage-overlap and --v3-voltage-validation-overlap')
    if args.v3_voltage_fast_pipeline and not (args.v3_voltage_proxy and args.v3_voltage_overlap and
                                              args.v3_voltage_validation_overlap and
                                              args.mode=='stop-proxy' and not args.acquisition_only and
                                              not args.compare_feedback and args.cycles<=bounded_cycles and
                                              args.record_storage=='trace' and not args.v3_voltage_pipeline):
        p.error('--v3-voltage-fast-pipeline requires bounded V3 STOP-proxy trace with overlap, and excludes gated pipeline')
    if args.inference_thread_cpu_trace and not args.v3_voltage_proxy:
        p.error('--inference-thread-cpu-trace requires bounded 26-request STOP-proxy inference with --v3-voltage-proxy')
    if args.absolute_epoch_cadence and (args.mode!='stop-proxy' or args.acquisition_only or
                                        args.compare_feedback or args.cycles>bounded_cycles):
        p.error('--absolute-epoch-cadence requires at most 500 full STOP-proxy inference cycles')
    if args.main_thread_cpu is not None:
        if args.main_thread_cpu<0 or args.compare_feedback or args.acquisition_only:
            p.error('--main-thread-cpu requires a nonnegative CPU and policy inference')
        if not hasattr(os,'sched_getaffinity') or not hasattr(os,'sched_setaffinity'):
            p.error('--main-thread-cpu requires Linux thread affinity')
        if args.main_thread_cpu not in os.sched_getaffinity(0):
            p.error('--main-thread-cpu is outside the current CPU affinity')
    if args.exclude_policy_cpu_from_workers:
        if (not args.v3_voltage_proxy or args.mode!='stop-proxy' or args.acquisition_only or
                args.compare_feedback or args.cycles>bounded_cycles or args.main_thread_cpu is None):
            p.error('--exclude-policy-cpu-from-workers requires at most 500 V3 STOP-proxy inference cycles and --main-thread-cpu')
        if len(set(os.sched_getaffinity(0))-{args.main_thread_cpu})<3:
            p.error('--exclude-policy-cpu-from-workers requires at least three other available CPUs')
    try:source_provenance=_start_source_provenance(args.provenance_mode,args.power_epoch)
    except (ValueError,OSError) as error:p.error(str(error))
    plan={'mode':args.mode,'cycles':args.cycles,'gap_ms':args.request_gap_us/1000,
          'startup_cycle_allowance':args.startup_cycle_allowance,
          'steady_cycles_requested':args.cycles-args.startup_cycle_allowance,
          'release_spin_us':args.release_spin_us,
          'v3_voltage_proxy':args.v3_voltage_proxy,
          'voltage_max_v':args.voltage_max_v,'voltage_range_v':[35.,args.voltage_max_v],
          'v3_voltage_pipeline':args.v3_voltage_pipeline,
          'v3_voltage_fast_pipeline':args.v3_voltage_fast_pipeline,
          'absolute_epoch_cadence':args.absolute_epoch_cadence,
          'absolute_epoch_min_start_separation_ms':(
              ABSOLUTE_MIN_START_SEPARATION_NS/1e6 if args.absolute_epoch_cadence else None),
          'requests_per_cycle':26 if args.v3_voltage_proxy else 24,
          'type1_requests_per_cycle':0,
          'request_gap_us':args.request_gap_us,'window':args.request_window,
          'timer_slack_ns':args.timer_slack_ns,
          'math_thread_startup':math_startup,
          'main_thread_cpu':args.main_thread_cpu,
          'exclude_policy_cpu_from_workers':args.exclude_policy_cpu_from_workers,
          'setup_gc':args.setup_gc,
          'record_storage':args.record_storage,
          'view_cache_variant':view_cache_selected,'view_cache_diagnostic_only':view_cache_selected,
          'policy_backend_requested':('pinned_scalar_cpp' if scalar_step_selected else
                                      'pinned_cached_view' if view_cache_selected else
                                      'pinned_native_baseline' if args.native_policy_manifest else
                                      'reference_bundle'),
          'require_pinned_fast_model':args.require_pinned_fast_model,
          'view_cache_manifest':args.view_cache_manifest,
          'view_cache_manifest_sha256':args.view_cache_manifest_sha256,
          'scalar_step_manifest':args.scalar_step_manifest,
          'scalar_step_manifest_sha256':args.scalar_step_manifest_sha256,
          'reused_policy_input_buffers':not args.acquisition_only,
          'startup_identity_batch_size':6 if args.compare_feedback else 1,
          'startup_identity_window':1,'startup_identity_retry':False,
          'compare_feedback':args.compare_feedback,
          'acquisition_only':args.acquisition_only,'enable_available':False,'learned_targets_sent':False,
          'state_changing_stop':args.mode=='stop-proxy',
          'input_workers':(['front6+voltage1','rear6+voltage1','IMU'] if args.v3_voltage_overlap
                           else ['front7','rear7','IMU'] if args.v3_voltage_proxy
                           else ['front6','rear6','IMU']),
          'disk_io_during_cycles':False,'full_controller_50Hz_verified':False}
    if source_provenance is not None:
        plan['source_provenance']=source_provenance
    if args.v3_voltage_overlap:plan['v3_voltage_overlap']=True
    if args.v3_voltage_validation_overlap:plan['v3_voltage_validation_overlap']=True
    if args.output_dispatch_trace:plan['output_dispatch_trace']=True
    if args.inference_thread_cpu_trace:plan['inference_thread_cpu_trace']=True
    if args.defer_gc_during_cycles:plan['defer_gc_during_cycles']=True
    if args.pre_cycle_policy_warmup_calls is not None:
        plan['pre_cycle_policy_warmup_calls']=args.pre_cycle_policy_warmup_calls
    if args.post_pin_policy_prime_calls is not None:
        plan['post_pin_policy_prime_calls']=args.post_pin_policy_prime_calls
    if not args.execute:
        print(json.dumps(plan,indent=2));return 0
    required=['front_port','rear_port','expected_uids','library','output']
    if not args.acquisition_only:required+=['calibration','mount','bundle']
    if any(not getattr(args,k) for k in required):p.error('Missing execution paths: '+','.join(required))
    if args.mode=='stop-proxy' and not args.supported_disabled:
        p.error('STOP proxy requires independently supported, already disabled robot')
    if args.timer_slack_ns is not None:
        try:thread_timer_slack.require_supported_platform()
        except thread_timer_slack.TimerSlackError as error:p.error(str(error))
    out=Path(args.output).expanduser().resolve()
    if any((parent/'.git').exists() for parent in (out,*out.parents)):
        p.error('Raw diagnostic records must be saved outside Git')
    out.mkdir(mode=0o700,parents=True,exist_ok=False)
    timer_slack=thread_timer_slack.TimerSlack(args.timer_slack_ns)
    setup_gc={'mode':args.setup_gc,'scope':'setup_only',
              'position':'after_identity_and_imu_start_before_warmup' if args.setup_gc is not None else None,
              'generation':2 if args.setup_gc is not None else None,'attempted':False,'complete':False,
              'begin_ns':None,'end_ns':None,'duration_ms':None,'collected_objects':None,
              'changes_gc_settings':False}
    report={'status':'ABORTED','plan':plan,'math_thread_startup':math_startup,
            'timer_slack':timer_slack.report,
            'setup_gc':setup_gc,'errors':[]};saved=[];device=None
    if source_provenance is not None:
        report.update(motor_power_epoch=source_provenance['motor_power_epoch'],
                      cadence_source_sha256=source_provenance['cadence_source_sha256'],
                      source_provenance=source_provenance)
    calibration=None
    cr,cw=os.pipe();handlers={};cancelled=[]
    def cancel(signum,frame):
        cancelled.append(signum)
        if len(cancelled)==1:os.write(cw,b'x')
    try:
        report['input_sha256']={k:hashlib.sha256(Path(getattr(args,k)).read_bytes()).hexdigest()
            for k in ('expected_uids','calibration','mount','gyro_bias') if getattr(args,k)}
        lib=native.load_library(args.library)
        uids=dual.pipeline.validate_uids(shadow._json(Path(args.expected_uids).read_bytes()))
        run=None
        if not args.acquisition_only:
            calibration=shadow._json(Path(args.calibration).read_bytes())
            if calibration['identities']!={str(i):uids[i] for i in range(1,13)}:
                raise ValueError('Calibration UID binding mismatch')
            mount=shadow._json(Path(args.mount).read_bytes())
            bias=shadow._json(Path(args.gyro_bias).read_bytes()) if args.gyro_bias else None
            if args.single_thread_math:
                math_startup['before_torch_import_env']=math_threads.verify_before_math_import()
            else:
                math_startup['before_torch_import_env']=math_threads.effective_math_thread_env()
            import torch
            torch.set_num_threads(1)
            torch.set_num_interop_threads(1)
            if args.native_policy_manifest:
                sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
                if scalar_step_selected:
                    from native_policy_overnight.model_call_fastpath.scalar_loader import load_file_only_verified
                    policy,source=load_file_only_verified(args.scalar_step_manifest,
                        expected_sha256=args.scalar_step_manifest_sha256,
                        baseline_manifest=args.native_policy_manifest,
                        baseline_sha=args.native_policy_manifest_sha256,bundle=args.bundle)
                    report['scalar_step_model_source']=source
                    report['native_baseline_model_source']=source['baseline_provenance']
                elif view_cache_selected:
                    from native_policy_overnight.view_cache.loader import load_file_only_verified
                    policy,source=load_file_only_verified(args.view_cache_manifest,
                        expected_sha256=args.view_cache_manifest_sha256,
                        baseline_manifest=args.native_policy_manifest,
                        baseline_sha=args.native_policy_manifest_sha256,bundle=args.bundle)
                    report['view_cache_model_source']=source
                    report['native_baseline_model_source']=source['baseline_provenance']
                else:
                    from native_policy_overnight import load_verified
                    policy,source=load_verified(args.native_policy_manifest,
                        expected_manifest_sha256=args.native_policy_manifest_sha256,bundle=args.bundle)
                    report['native_baseline_model_source']=source
            else:policy,source=shadow.load_policy(args.bundle)
            report['model_source']=source
            run=observer.StatefulPolicyObserver(policy,calibration,imu_mount_candidate=mount,
                h_hypothesis=args.h_hypothesis,command=[0.,0.,0.],max_ticks=args.cycles,
                max_age_ns=LIMIT_NS,max_spread_ns=LIMIT_NS,torch_module=torch,
                gyro_bias_candidate=bias,profile_consume=True,measured_diagnostic_ticks=True,
                reuse_input_buffers=True)
        bindings=dual.validate_ports(args.front_port,args.rear_port)
        with ExitStack() as stack:
            stack.enter_context(dual.pipeline.ownership_locks())
            if not args.compare_feedback:stack.enter_context(live.imu_ownership_lock())
            guard=dual.BootIdentityGuard();stack.callback(guard.close)
            report['boot_id']=guard.boot_id
            if calibration is not None and calibration.get('source_current_boot_id')!=guard.boot_id:
                raise ValueError('Capture-bound calibration must match the current Jetson boot; run capture again')
            def check():
                if cancelled:raise InterruptedError('Signal cancellation')
                guard.check()
            for sig in (signal.SIGINT,signal.SIGTERM):handlers[sig]=signal.signal(sig,cancel)
            import serial
            sessions={}
            for scope,binding in bindings.items():
                stack.enter_context(dual.port_lock(binding['resolved']))
                port=serial.Serial(port=None,baudrate=921600,timeout=0,write_timeout=.1,exclusive=True)
                port.dtr=port.rts=False;port.port=binding['path'];port.open();stack.callback(port.close)
                if not dual.binding_matches(binding) or os.fstat(port.fileno()).st_rdev!=binding['st_rdev']:
                    raise ValueError('Port binding changed')
                # A separate read-only proc fd per worker; not a shared Python guard lock.
                boot_fd=os.open('/proc/sys/kernel/random/boot_id',os.O_RDONLY|os.O_CLOEXEC);stack.callback(os.close,boot_fd)
                sessions[scope]=native.NativeSession(lib,port.fileno(),first_id=dual.SCOPES[scope][0],
                    cancel_fd=cr,boot_fd=boot_fd,boot_id=guard.boot_id,stop_proxy=args.mode=='stop-proxy',
                    gap_ns=args.request_gap_us*1000,window=args.request_window)
            if args.compare_feedback:
                from .native_feedback_compare import collect_feedback_comparison
                result,saved=collect_feedback_comparison(sessions,uids,boot_id=guard.boot_id,
                    supported_disabled=args.supported_disabled,cycles=args.cycles,check=check)
                report.update(result)
            else:
                for scope,session in sessions.items():
                    captures=report.setdefault('identities',{}).setdefault(scope,[])
                    # Identity preflight is serialized; measured collection keeps the session window.
                    for mid in dual.SCOPES[scope]:
                        check()
                        capture=session.exchange([codec.read_request(mid)])
                        captures.append(native.exchange_evidence(*capture))
                        rows=native.records_as_events(capture[0],cycle=0)
                        if len(rows)!=1 or rows[0]['motor_id']!=mid or rows[0]['result']['mcu_uid_hex']!=uids[mid]:
                            raise ValueError('Fresh UID mismatch: ID'+str(mid))
                device=imu.ICM20948();stack.callback(device.close)
                report['imu_configuration']=device.start()
                prime=None
                if run is not None:
                    if args.setup_gc is not None:
                        import gc
                        setup_gc['begin_ns']=time.monotonic_ns();setup_gc['attempted']=True
                        try:
                            setup_gc['collected_objects']=gc.collect()
                            setup_gc['complete']=True
                        finally:
                            setup_gc['end_ns']=time.monotonic_ns()
                            setup_gc['duration_ms']=(setup_gc['end_ns']-setup_gc['begin_ns'])/1e6
                    pre_cycle_warmup=args.pre_cycle_policy_warmup_calls is not None
                    warmup={'position':('after_worker_startup_before_optional_main_thread_affinity' if pre_cycle_warmup else
                                        'after_identity_and_imu_start_before_worker_startup'),
                            'after_main_thread_affinity':False,
                            'iterations':args.pre_cycle_policy_warmup_calls if pre_cycle_warmup else 10,
                            'begin_ns':None,'end_ns':None,
                            'duration_ms':None,'complete':False}
                    report['setup_policy_warmup']=warmup
                    if args.post_pin_policy_prime_calls is not None:
                        prime={'position':'after_main_thread_affinity_before_timed_cycles',
                               'kind':'synthetic_model_calls_on_reused_input_tensors',
                               'iterations':args.post_pin_policy_prime_calls,
                               'begin_ns':None,'end_ns':None,'duration_ms':None,
                               'complete':False,'observer_reset_after':False,
                               'sensor_cycles':0,'stop_writes':0}
                        report['setup_policy_prime']=prime
                    def prepare_policy():
                        warmup['begin_ns']=time.monotonic_ns()
                        try:
                            replay.warmup_policy(policy,torch,args.h_hypothesis,warmup['iterations'])
                            if prime is None:run.prepare_run(warmup_completed=True)
                            warmup['complete']=True
                        finally:
                            warmup['end_ns']=time.monotonic_ns()
                            warmup['duration_ms']=(warmup['end_ns']-warmup['begin_ns'])/1e6
                    def prime_policy():
                        prime['begin_ns']=time.monotonic_ns()
                        try:
                            replay.warmup_policy(policy,torch,args.h_hypothesis,prime['iterations'],
                                                 input_tensors=_reused_policy_input_tensors(run))
                            run.prepare_run(warmup_completed=True)
                            prime['observer_reset_after']=True
                            prime['complete']=True
                        finally:
                            prime['end_ns']=time.monotonic_ns()
                            prime['duration_ms']=(prime['end_ns']-prime['begin_ns'])/1e6
                    if not pre_cycle_warmup:prepare_policy()
                with timer_slack:
                    options={'mode':args.mode,'cycles':args.cycles,'check':check,
                             'voltage_max_v':args.voltage_max_v}
                    if args.startup_cycle_allowance:
                        options['startup_cycle_allowance']=args.startup_cycle_allowance
                    if args.release_spin_us is not None:
                        options['deadline_wait']=lambda target: native.wait_until(
                            lib,cr,target,spin_us=args.release_spin_us)
                    if args.v3_voltage_proxy:options['v3_voltage_proxy']=True
                    if args.v3_voltage_overlap:options['v3_voltage_overlap']=True
                    if args.v3_voltage_validation_overlap:
                        options['v3_voltage_validation_overlap']=True
                    if args.v3_voltage_pipeline:options['v3_voltage_pipeline']=True
                    if args.v3_voltage_fast_pipeline:options['v3_voltage_fast_pipeline']=True
                    if args.absolute_epoch_cadence:options['absolute_epoch_cadence']=True
                    if args.timer_slack_ns is not None:options['worker_initializer']=timer_slack.worker_initializer
                    if args.main_thread_cpu is not None:options['main_thread_cpu']=args.main_thread_cpu
                    if args.exclude_policy_cpu_from_workers:
                        options['exclude_policy_cpu_from_workers']=True
                    if args.output_dispatch_trace:options['output_dispatch_trace']=True
                    if args.inference_thread_cpu_trace:options['inference_thread_cpu_trace']=True
                    if args.defer_gc_during_cycles:options['defer_gc_during_cycles']=True
                    if run is not None and pre_cycle_warmup:
                        options['pre_cycle_policy_prepare']=prepare_policy
                    if prime is not None:options['post_pin_policy_prepare']=prime_policy
                    if args.record_storage!='objects':options['record_storage']=args.record_storage
                    result,saved=collect(sessions,device,run,**options)
                    report.update(result)
                    if result['status']=='COMPLETE_DIAGNOSTIC':timer_slack.verify_workers()
    except BaseException as error:
        report['status']='ABORTED';report.setdefault('errors',[]).append(type(error).__name__+': '+str(error))
        if isinstance(error,native.ExchangeError):report['startup_failure']=native.exchange_evidence(error.records,error.stats)
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)
        os.close(cr);os.close(cw)
        report['imu_restore_status']=device.restore_status if device is not None else 'not_started'
        if device is not None and device.restore_status not in ('restored','not_needed'):
            report['status']='ABORTED';report['errors'].append('IMU restoration unconfirmed')
        _finish_source_provenance(report,source_provenance)
        extra=[]
        if args.record_storage in ('encoded','trace'):
            values,failures=_encoded_records_for_output(saved,report.get('record_storage_failure'))
            if failures:
                report['status']='ABORTED'
                for failure in failures:
                    if failure['error'] not in report['errors']:report['errors'].append(failure['error'])
                report['record_storage_failure_artifact']='record-storage-failure.json'
                extra.append(('record-storage-failure.json',{
                    'schema':'native-diagnostic-record-storage-failure-v1','status':'ABORTED',
                    'failures':failures,'output_allowed':False,'approved_for_runtime':False}))
        else:values=saved if args.compare_feedback else _serialize(saved)
        for name,value in (*extra,('records.json',values),('report.json',report)):
            with os.fdopen(os.open(out/name,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as f:
                json.dump(value,f,allow_nan=False,ensure_ascii=False);f.write('\n')
            if name=='records.json' and (args.v3_voltage_pipeline or args.v3_voltage_fast_pipeline):
                # Bind the approval report to the exact on-disk evidence, not
                # an in-memory serialization that might differ by one byte.
                key='v3_voltage_pipeline' if args.v3_voltage_pipeline else 'v3_voltage_fast_pipeline'
                report.setdefault(key,{})['records_sha256']=(
                    hashlib.sha256((out/name).read_bytes()).hexdigest())
        print(json.dumps({'status':report['status'],'output':str(out),'errors':report.get('errors',[]),
                          'distributions_ms':report.get('distributions_ms')},ensure_ascii=False))
    return 0 if report['status']=='COMPLETE_DIAGNOSTIC' else 2

if __name__=='__main__':raise SystemExit(main())
