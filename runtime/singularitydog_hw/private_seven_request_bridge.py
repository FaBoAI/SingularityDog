"""PRIVATE unpaired real-observer experiment; no implicit device or Type1 API.

The collector bridge Future is distinct from the native immutable-prefix
Future. The outer bus worker owns each phase from constructor through close;
its original executor Future completes only after the full7 owner and callback
have joined and the session reservation has been released. Both comparison
branches share the same prestarted two-worker native executor.
"""
from concurrent.futures import ThreadPoolExecutor
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import threading
import time

from . import can_readonly as codec
from . import native_active_transport as active
from . import native_diagnostic_transport as diagnostic

MODES=('baseline6plus1','split7')
SCOPES={'front':tuple(range(1,7)),'rear':tuple(range(7,13))}
PERIOD_NS=20_000_000
EXPECTED_SOURCE_COUNT=819
CORE_SHA='dfab5de91e6274d84ef6c1789cd339aeb2d6d08dc45655a61b8b005f43316887'
TARGET_LIBRARY_SHA='8f914f9d3db3aacf37680f07f717b3fea5a9cd2567773514d69f378b5b3e2b0f'
TARGET_QUALIFICATION_SHA='81794540ef49ab2561c7737ed5c723919c9bd1832173fc5eb86980617066128e'
CANDIDATE_SHA='dd0b42914519a6caad8607f3dea5390fe27aaef7d59ed89e7bd76a1e1e1785c2'
SOURCE_NAMES=('singularitydog_hw/native_pipeline_benchmark.py',
              'singularitydog_hw/private_seven_request_bridge.py',
              'singularitydog_hw/private_seven_request_candidate.py',
              'singularitydog_hw/native_active_transport.py',
              'singularitydog_hw/native_diagnostic_transport.py',
              'singularitydog_hw/can_readonly.py',
              'experiments/private_seven_request/transport.cpp')

def need(value,message):
    if not value:raise ValueError(message)

def image(value):return bytes(memoryview(value).cast('B'))

def prepare_plan(args,context):
    """File-only validation; explicit selection never authorizes motion."""
    from . import policy_live_profile as profiles
    mode=args.private_seven_request_experiment
    need(mode in MODES and context is not None,'Private comparison requires exact current real FK profile')
    profile=context['profile']
    need(profile['approved_for_supported_policy_output'] is False and profile['review'] is None and
         bool(profile['blockers']) and profile.get('output_allowed') is False and
         profile.get('native_phase_pair',False) is False,'Fresh blocked unapproved UNPAIRED draft required')
    need(not args.native_phase_pair and not args.native_pair_prime_before_cycles and
         not args.native_future_notification_joins and not args.retain_gil_trace_copy and
         not args.acquisition_only and not args.compare_feedback and args.voltage_max_v==42 and
         args.mode=='stop-proxy' and args.cycles in (5,501) and args.startup_cycle_allowance==1 and
         args.request_gap_us==900 and args.request_window==3 and args.release_spin_us==500 and
         args.timer_slack_ns==1000 and args.main_thread_cpu==4 and args.record_storage=='trace' and
         args.pre_cycle_policy_warmup_calls==10 and args.post_pin_policy_prime_calls==10 and
         args.setup_gc=='before-warmup' and all((args.supported_disabled,args.single_thread_math,
            args.require_pinned_fast_model,args.exclude_policy_cpu_from_workers,args.absolute_epoch_cadence,
            args.defer_gc_during_cycles,args.output_dispatch_trace,args.inference_thread_cpu_trace,
            args.v3_voltage_proxy,args.v3_voltage_overlap,args.v3_voltage_validation_overlap,
            args.v3_voltage_fast_pipeline,args.prepare_voltage_before_feedback_publication)),
         'Private unpaired comparison requires exact canonical5/501900/window3/20ms/realobserver setup')
    inventory_path=Path(args.private_seven_request_source_inventory).absolute()
    inventory,_=profiles._read_json(inventory_path,digest=args.private_seven_request_source_inventory_sha256)
    need(set(inventory)=={'schema','source_count','source_manifest','source_sha256','capability_qualification'} and
         inventory['schema']=='PRIVATE.unpaired-seven-request-source.v2' and
         inventory['source_count']==EXPECTED_SOURCE_COUNT and set(inventory['source_sha256'])==set(SOURCE_NAMES),
         'Exact externally pinned private source inventory required')
    root=Path(__file__).resolve().parents[1];refs={str(inventory_path):args.private_seven_request_source_inventory_sha256}
    for name,digest in inventory['source_sha256'].items():
        path=root/name;profiles._hash(digest,name)
        need(path.is_file() and not any(p.is_symlink() for p in (path,*path.parents)) and
             hashlib.sha256(path.read_bytes()).hexdigest()==digest,'Private selected source differs: '+name)
        refs[str(path)]=digest
    need(inventory['source_sha256']['experiments/private_seven_request/transport.cpp']==CORE_SHA and
         inventory['source_sha256']['singularitydog_hw/private_seven_request_candidate.py']==CANDIDATE_SHA,
         'Frozen optional splitABI2 capability source required')
    manifest_ref=inventory['source_manifest'];manifest,_=profiles._read_json(manifest_ref['path'],digest=manifest_ref['sha256'])
    need(manifest['file_count']==len(manifest['files'])==EXPECTED_SOURCE_COUNT,'Exact private source manifest count required')
    for name,row in manifest['files'].items():
        need(not Path(name).is_absolute() and '..' not in Path(name).parts,'Unsafe private source manifest member')
        path=root.parent/name
        need(path.is_file() and not any(p.is_symlink() for p in (path,*path.parents)) and
             hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256'] and path.stat().st_mode&0o777==row['mode'],
             'Private kit source/mode changed: '+name)
        refs[str(path)]=row['sha256']
    refs[manifest_ref['path']]=manifest_ref['sha256']
    library=Path(args.private_seven_request_library).absolute();digest=profiles._hash(args.private_seven_request_library_sha256,'private candidate library')
    need(library.is_file() and not any(p.is_symlink() for p in (library,*library.parents)) and
         hashlib.sha256(library.read_bytes()).hexdigest()==digest,'Private candidate library SHA differs')
    need(digest==TARGET_LIBRARY_SHA,'Exact qualified ARM ABI2 library required')
    native_source=library.parent/'transport.cpp'
    need(native_source.is_file() and not any(p.is_symlink() for p in (native_source,*native_source.parents)),
         'Regular nonsymlink target native source required before devices')
    need(hashlib.sha256(native_source.read_bytes()).hexdigest()==CORE_SHA,
         'Selected target capability must use the same frozen ABI2 core bytes')
    record_path=library.parent/'build-record.json';record,_=profiles._read_json(record_path)
    need(record['source_sha256']==CORE_SHA and record['binary_sha256']==digest and record['abi']==1,
         'Same genuine active ABI1 plus explicit splitABI2 build required')
    refs[str(library)]=digest;refs[str(record_path)]=hashlib.sha256(record_path.read_bytes()).hexdigest()
    refs[str(native_source)]=CORE_SHA
    qualification_ref=inventory['capability_qualification']
    need(set(qualification_ref)=={'path','sha256'} and qualification_ref['sha256']==TARGET_QUALIFICATION_SHA,
         'Exact independently qualified32 target receipt required')
    qualification,_=profiles._read_json(qualification_ref['path'],digest=TARGET_QUALIFICATION_SHA)
    need(qualification['schema']=='private.seven-prefix-target-socket-pty.v1' and
         qualification['status']=='PASS_32_TARGET_SOCKET_PTY_TESTS' and qualification['returncode']==0 and
         qualification['unit_cases']==32 and qualification['unit_complete'] is True and
         qualification['all_settings_restored'] is True and qualification['source_unchanged'] is True and
         qualification['build']['binary_sha256']==TARGET_LIBRARY_SHA and
         qualification['build']['source_sha256']==CORE_SHA and
         all(qualification[key] is False for key in ('physical_CAN_opened','motor_enable_sent',
             'learned_targets_sent','whole_loop_performance_measured','output_allowed','approved_for_runtime')) and
         qualification['post_trial_physical_observation'] is None,
         'Actual qualified32 ARM capability proof cannot substitute device/model timing')
    refs[qualification_ref['path']]=TARGET_QUALIFICATION_SHA
    return dict(schema='PRIVATE.unpaired-seven-request-plan.v1',branch=mode,
                active_phase_pair_used=False,genuine_unpaired_sessions=True,requests_per_cycle=26,
                request_gap_us=900,request_window=3,period_ns=PERIOD_NS,
                original_absolute_deadlines=True,normal_python_observer=True,real_pinned_fk_model=True,
                original_prestarted_collector_worker_count=3,extra_native_owner_worker_count=0,phase_caller="original_collector_main",
                phase_constructor_start_wait_join_close_inside_whole_cycle=True,
                library={'path':str(library),'sha256':digest},source_refs=refs,
                approved_for_runtime=False,output_allowed=False,active_controller_qualification=False)

def diagnostic_result(records,stats):
    # The extra active rejected-total counter is kept in explicit native
    # evidence. This legacy-view copy never relabels combined Stats as6/1.
    copied=diagnostic.Stats.from_buffer_copy(image(stats)[:C.sizeof(diagnostic.Stats)])
    copied.rejected_total=int(stats.rejected_total)
    return records,copied

def owned_result(future,deadline_ns):
    remaining=(deadline_ns-time.monotonic_ns())/1e9
    if remaining<=0:
        if future.done():
            records,stats=future.result() # native errors retain priority/raw
            raise diagnostic.ExchangeError('Private unpaired completed native exchange but original20ms join expired',records,stats)
        raise TimeoutError('Private unpaired original20ms join expired')
    value=future.result(timeout=remaining)
    if time.monotonic_ns()>=deadline_ns:
        raise diagnostic.ExchangeError('Private unpaired original20ms takeout expired',*value)
    return value

def verify_prefix_full(prefix,full):
    need(prefix.scope=='PARTIAL_COMBINED_SEVEN_REQUESTS' and prefix.native_owner_joined is False,
         'Immutable native partial combined7 prefix required')
    need(full.scope=='FULL_COMBINED_SEVEN_REQUESTS' and prefix.generation==full.generation and
         len(prefix.record_images)==6 and len(full.record_images)==7 and
         prefix.record_images==full.record_images[:6],
         'Exact original six prefix/full7 record bits required before STOP')

class DisabledSession:
    """Exact zero STOP/Type0/17 on original outer bus workers, no nested pool."""
    def __init__(self,runtime,scope,session):
        self.runtime=runtime;self.scope=scope;self.session=session
        ids=SCOPES[scope]
        self.allowed=frozenset([codec.read_request(mid,p) for mid in ids for p in (None,'position','velocity','voltage')]+
                               [diagnostic.stop_wire(mid) for mid in ids])
    def exchange(self,wires,*,timeout_ns=100_000_000,before_native=None):
        wires=tuple(wires)
        need(1<=len(wires)<=12 and all(type(w) is bytes and w in self.allowed for w in wires),
             'Private diagnostic accepts only exact Type0/17/zero STOP')
        need(self.session._phase_pair is None,'Genuine unpaired session required')
        need(all(p._closed for p in self.runtime.phases.values()),'All original main-owned phases must close before STOP/reuse')
        kwargs={'timeout_ns':timeout_ns,'before_native':before_native}
        if self.runtime.deadline_ns is not None:kwargs['deadline_ns']=self.runtime.deadline_ns
        try:
            value=diagnostic_result(*self.session.exchange(wires,**kwargs))
            if self.runtime.deadline_ns is not None and time.monotonic_ns()>=self.runtime.deadline_ns:
                raise diagnostic.ExchangeError('Completed native raw retained; original20ms takeout expired',*value)
            return value
        except active.ExchangeError as error:
            records,stats=diagnostic_result(error.records,error.stats)
            raise diagnostic.ExchangeError(str(error),records,stats) from error

class UnpairedRuntime:
    """Original three workers; immutable prefix takeout and close on main caller."""
    def __init__(self,*,mode,library,fd_by_scope,boot_fd_by_scope,cancel_fd,boot_id,source_refs):
        need(mode in MODES and set(fd_by_scope)==set(boot_fd_by_scope)==set(SCOPES),'Explicit two-bus unpaired mode required')
        self.mode=mode;self.library=library;self.cancel_fd=cancel_fd;self.refs=dict(source_refs)
        self.pool=None;self.deadline_ns=None;self.phases={};self.prefixes={};self.proof=None;self.closed=False
        self.main_caller=threading.current_thread();self.closed_phase_count=0;self.sessions={};self.native_sessions={}
        self.worker_settings=dict(worker_count=3,extra_native_owner_worker_count=0,started=False,
                                  source='original_prestarted_collector_pool',controls='original_collector_timer_affinity_restore')
        try:
            for scope,ids in SCOPES.items():
                session=active.ActiveSession(library,fd_by_scope[scope],first_id=ids[0],cancel_fd=cancel_fd,
                    boot_fd=boot_fd_by_scope[scope],boot_id=boot_id,raw_lower_by_id={i:-12.57 for i in ids},
                    raw_upper_by_id={i:12.57 for i in ids},kp_max_by_id={i:0. for i in ids},
                    kd_max_by_id={i:0. for i in ids},gap_ns=900_000,window=3)
                need(session._phase_pair is None,'No pair borrowing or detach permitted')
                self.native_sessions[scope]=session;self.sessions[scope]=DisabledSession(self,scope,session)
        except BaseException as primary:
            for session in self.native_sessions.values():
                try:session.close()
                except BaseException as cleanup:primary.add_note('Unpaired constructor cleanup: '+repr(cleanup))
            raise
    def main_owner(self):need(threading.current_thread() is self.main_caller,'Original collector main caller required')
    def verify_sources(self):
        for name,digest in self.refs.items():
            path=Path(name);need(path.is_file() and not any(p.is_symlink() for p in (path,*path.parents)) and
                hashlib.sha256(path.read_bytes()).hexdigest()==digest,'Private source/build pin differs: '+name)
    def start_workers(self,pool,initializer):
        from . import thread_timer_slack
        timer=getattr(initializer,'__self__',None)
        need(type(pool) is ThreadPoolExecutor and pool._max_workers==3 and type(timer) is thread_timer_slack.TimerSlack and
             timer.requested_ns==1000 and timer._prctl is not None,'Original prestarted3 pool and1000ns scope required')
        self.pool=pool;self.worker_settings['started']=True
    def configure_owners(self,original_affinity,main_cpu):
        need(self.pool is not None and main_cpu==4 and original_affinity=={0,1,2,3,4},'Original CPU4+0..3 controls required')
        self.worker_settings['main_caller_native_tid']=threading.get_native_id()
    def begin_cycle(self,deadline_ns):
        self.main_owner();need(all(p._closed for p in self.phases.values()),'Previous original native7 not closed; reuse prohibited')
        need(type(deadline_ns) is int and time.monotonic_ns()<deadline_ns,'Original absolute cycle deadline required')
        self.deadline_ns=deadline_ns;self.phases={};self.prefixes={};self.proof=None
    def fd_release_safe(self):
        return all(p._closed for p in self.phases.values()) and all(not s.busy.locked() for s in self.native_sessions.values())
    def start_split(self,voltage_ids,feedback_bridges,proof,generation_base):
        from .private_seven_request_candidate import SevenRequestPhase
        self.main_owner();need(self.mode=='split7' and not self.phases and self.pool is not None,'Fresh main-owned split cycle required')
        self.proof=proof;proof.setdefault('private_seven_request',{'buses':{},'output_allowed':False,'active_controller_qualification':False})
        # BOTH complete constructors precede either native CAN task dispatch.
        for scope in SCOPES:
            row=dict(scope='UNPAIRED_NATIVE_COMBINED_SEVEN',legacy_acquired_stats_scope='PARTIAL_COMBINED_SEVEN_REQUESTS',
                     legacy_voltage_stats_scope='FULL_COMBINED_SEVEN_REQUESTS',generation=generation_base+(1 if scope=='front' else 2),
                     phase_caller_native_tid=threading.get_native_id(),constructor_begin_ns=time.monotonic_ns(),
                     collector_feedback_bridge_future_identity=id(feedback_bridges[scope]),full_owner_joined=False,closed_before_output=False)
            proof['private_seven_request']['buses'][scope]=row
            phase=SevenRequestPhase(self.native_sessions[scope],self.pool,cancel_fd=self.cancel_fd,
                generation=row['generation'],voltage_id=voltage_ids[scope])
            self.phases[scope]=phase;row['constructor_end_ns']=time.monotonic_ns()
        originals={}
        for scope,phase in self.phases.items():
            prefix_future,full_future=phase.start(deadline_ns=self.deadline_ns);originals[scope]=full_future
            row=proof['private_seven_request']['buses'][scope]
            row.update(native_immutable_prefix_future_identity=id(prefix_future),original_native_full_future_identity=id(full_future),
                       native_feedback_distinct_from_collector_bridge=prefix_future is not feedback_bridges[scope],native_start_submitted_ns=time.monotonic_ns())
        return originals
    def pump_feedback(self,bridges):
        self.main_owner();begin=time.monotonic_ns()
        try:
            for scope,phase in self.phases.items():
                prefix=phase.wait_feedback(phase.feedback_future,generation=phase.generation)
                need(phase.feedback_future.result() is prefix,'Exact native immutable prefix Future required')
                self.prefixes[scope]=prefix;row=self.proof['private_seven_request']['buses'][scope]
                value=diagnostic_result(prefix.decoded_records(),prefix.decoded_stats())
                row.update(prefix_scope=prefix.scope,prefix_snapshot_ns=prefix.snapshot_ns,prefix_native_owner_joined=prefix.native_owner_joined,
                           prefix_received_mask=prefix.received_mask,prefix6_records_hex=[r.hex() for r in prefix.record_images],
                           partial_combined7_stats_hex=prefix.stats_image.hex())
                self.proof['feedback_dispatch_ns_by_bus'][scope]=value[1].begin_ns
                self.proof['feedback_reply_end_ns_by_bus'][scope]=max(r.received_ns for r in value[0])
                self.proof['feedback_ready_ns_by_bus'][scope]=prefix.snapshot_ns
                self.proof.setdefault('feedback_published_ns_by_bus',{})[scope]=time.monotonic_ns()
                bridges[scope].set_result(value);row['collector_bridge_published_ns']=time.monotonic_ns()
        except BaseException as primary:
            for future in bridges.values():
                if not future.done():future.set_exception(primary)
            raise
        finally:self.proof['private_seven_request']['main_prefix_pump']=dict(begin_ns=begin,end_ns=time.monotonic_ns(),original_absolute_deadline_ns=self.deadline_ns)
    def voltage_view(self,scope,future):
        """Immutable original-full copy for validator; never join/close/reuse."""
        phase=self.phases[scope];need(future is phase.full_future,'Original native full Future required')
        full=future.result();need(full is phase._full_result,'Native full result identity required')
        prefix=self.prefixes[scope];verify_prefix_full(prefix,full)
        rows=full.decoded_records();stats=full.decoded_stats()
        return diagnostic_result((diagnostic.Record*1).from_buffer_copy(full.record_images[6]),stats)
    def settle_voltage(self,futures,record):
        """Main joins ORIGINAL owners/callback fence and closes before STOP."""
        self.main_owner();errors=[]
        for scope,phase in self.phases.items():
            row=self.proof['private_seven_request']['buses'][scope];primary=None
            try:
                need(futures.get(scope) is phase.full_future,'Original native full owner Future required')
                full=phase.join();row['full_owner_join_ns']=time.monotonic_ns()
                prefix=self.prefixes[scope];verify_prefix_full(prefix,full)
                row.update(full_owner_joined=True,prefix_full_record_bits_exact=True,full_scope=full.scope,
                           full7_records_hex=[r.hex() for r in full.record_images],full_combined7_stats_hex=full.stats_image.hex(),native_return_ns=full.native_return_ns)
                record['voltage'][scope]=self.voltage_view(scope,phase.full_future)
                self.proof['voltage_dispatch_ns_by_bus'][scope]=full.decoded_records()[6].start_ns
                self.proof['voltage_reply_end_ns_by_bus'][scope]=full.decoded_records()[6].received_ns
            except BaseException as error:
                primary=error;errors.append(error);record.setdefault('voltage_errors_by_bus',{})[scope]=type(error).__name__+': '+str(error)
            finally:
                row['close_begin_ns']=time.monotonic_ns()
                try:phase.close()
                except BaseException as cleanup:
                    row['close_error']=repr(cleanup)
                    if primary is not None:primary.add_note('Private main-owned7 cleanup: '+repr(cleanup))
                    else:errors.append(cleanup)
                finally:
                    row['closed_before_output']=phase._closed;row['close_end_ns']=time.monotonic_ns()
                    row.setdefault('full7_records_hex',[image(r).hex() for r in phase.full_records])
                    row.setdefault('full_combined7_stats_hex',image(phase.full_stats).hex())
                    row['original_native_full_future_done']=phase.full_future is not None and phase.full_future.done()
                    if phase._closed:self.closed_phase_count+=1
        return errors
    def cleanup_phases(self):
        self.main_owner();errors=[]
        for scope,phase in self.phases.items():
            if phase._closed:continue
            try:phase.close()
            except BaseException as error:errors.append(error)
            finally:
                if self.proof is not None:
                    row=self.proof['private_seven_request']['buses'][scope]
                    row.setdefault('full7_records_hex',[image(r).hex() for r in phase.full_records]);row.setdefault('full_combined7_stats_hex',image(phase.full_stats).hex())
                    row.update(cleanup_close_end_ns=time.monotonic_ns(),closed_before_output=phase._closed,cleanup_only=True)
        if errors:raise errors[0]
    def close(self):
        if self.closed:return
        self.cleanup_phases();need(self.fd_release_safe(),'Native owner unresolved; retain sessions/FDs/locks')
        for session in self.native_sessions.values():session.close()
        self.verify_sources();self.closed=True
    def evidence(self):
        return dict(schema='PRIVATE.unpaired-real-model-seven-request.v2',branch=self.mode,genuine_unpaired_sessions=True,
                    active_phase_pair_used=False,original_collector_worker_count=3,extra_native_owner_worker_count=0,
                    phase_caller='original_collector_main',owner_workers=self.worker_settings,closed_phase_count=self.closed_phase_count,
                    all_phase_owners_closed=all(p._closed for p in self.phases.values()),fd_release_safe=self.fd_release_safe(),source_refs=self.refs,
                    phase_constructor_join_copy_close_inside_whole_cycle=True,collector_future_is_bridge_not_native_completion=True,
                    active_controller_qualification=False,approved_for_runtime=False,output_allowed=False,post_trial_physical_observation=None)
