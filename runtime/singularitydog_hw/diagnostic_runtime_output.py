"""Default-off STOP-only use of actual BusWorkers on a borrowed collector pool.

No executor thread/session/reader is added. The original collector remains the
sole pool owner and joins it before detaching this adapter. This diagnostic
proves source-specific STOP-proxy publication/waiting, never Type1 admission.
"""
from concurrent.futures import ThreadPoolExecutor
import threading

from . import native_active_transport as active
from . import native_diagnostic_transport as diagnostic
from . import policy_output_runtime as runtime

_BORROWED_COLLECTOR_TOKEN=object()


class _BorrowedDiagnosticOutput(runtime.BusWorkers):
    def __init__(self,pool,sessions,cancel_io,*,token,clock,
                 native_feedback_batch_decode=False,native_feedback_codec_selection=None,
                 unpaired_output_future_notifications=False,notification_waiter=None):
        if (token is not _BORROWED_COLLECTOR_TOKEN or type(pool) is not ThreadPoolExecutor or
                pool._max_workers!=3 or pool._shutdown or len(pool._threads)!=3 or
                type(sessions) is not dict or set(sessions)!=set(runtime.BUSES) or
                sessions['front'] is sessions['rear'] or not callable(cancel_io)):
            raise ValueError('Exact original prestarted three-worker diagnostic pool required')
        for scope,session in sessions.items():
            if (type(session) is not active.ActiveSession or session.first_id!=runtime.BUSES[scope][0] or
                    session._phase_pair is not None or not session._handle or session.poisoned or
                    session.busy.locked() or any(session._limits.kp[i]!=0. or session._limits.kd[i]!=0.
                        for i in range(6))):
                raise ValueError('Genuine idle zero-gain unpaired diagnostic sessions required')
            active.verified_active_source_binding(session.lib)
        self.notification_source_binding=None
        if unpaired_output_future_notifications and notification_waiter is not None:
            from .unpaired_output_future_notifications import prepare_notifications
            self.notification_source_binding=prepare_notifications(sessions,notification_waiter)
        self._borrowed_pool=pool;self._caller=threading.current_thread();self._closed=False
        self._original_futures=[];self._collector_output_transactions=0;self._collector_collect_attempts=0;self._dispatch_trace=None
        self._stop_batches={scope:tuple(diagnostic.stop_wire(mid) for mid in ids)
                            for scope,ids in runtime.BUSES.items()}
        # Ordinary constructor/close behavior remains untouched. Its two lazy
        # executor objects have no threads/tasks yet; discard them during setup.
        super().__init__(sessions,cancel_io,clock=clock,
            native_feedback_batch_decode=native_feedback_batch_decode,
            native_feedback_codec_selection=native_feedback_codec_selection,
            unpaired_output_future_notifications=unpaired_output_future_notifications)
        lazy=self.pools
        if any(pool._threads for pool in lazy.values()):
            super().close();raise RuntimeError('Unexpected active worker started during diagnostic setup')
        for owned in lazy.values():owned.shutdown(wait=True,cancel_futures=False)
        self.pools={scope:pool for scope in runtime.BUSES}

    def _caller_check(self):
        if (threading.current_thread() is not self._caller or self._closed or
                any(value is not self._borrowed_pool for value in self.pools.values())):
            raise RuntimeError('Original main caller and attached borrowed pool required')

    def set_dispatch_trace(self,values,base,check):
        self._caller_check()
        from array import array
        if (type(values) is not array or values.typecode!='Q' or type(base) is not int or
                base<0 or base+14>=len(values) or not callable(check) or
                any(not f.done() for f in self._original_futures)):
            raise ValueError('Bounded original dispatch trace and idle genuine owners required')
        self._dispatch_trace=(values,base,check)

    def _exchange_decoded(self,scope,wires,deadline_ns,label):
        # These are real worker timestamps. Submission stamps are only main
        # upper bounds; raw native begin/write/reply timestamps stay separate.
        journal_begin=len(self.journal)
        try:
            trace=self._dispatch_trace
            if trace is not None:
                values,base,check=trace
                values[base+(5 if scope=='front' else 9)]=self.clock()
                check()
                values[base+(6 if scope=='front' else 10)]=self.clock()
            return super()._exchange_decoded(scope,wires,deadline_ns,label)
        except BaseException as error:
            # _exchange journals the exact raw pair before Python decoding.
            # Preserve that pair on a plain decoder failure as well; do not
            # relabel failure, copy clocks, or substitute cleanup STOP replies.
            if not hasattr(error,'records'):
                for bus,raw,_,raw_label in reversed(self.journal[journal_begin:]):
                    if bus==scope and raw_label==label:
                        error.records,error.stats=raw
                        break
            self.emergency(type(error).__name__+': '+str(error))
            raise

    def submit(self,*args,**kwargs):
        raise RuntimeError('Borrowed diagnostic adapter accepts only exact decoded STOP output')

    def submit_decoded(self,wires,*,deadline_ns,label):
        self._caller_check()
        if (type(wires) is not dict or set(wires)!=set(runtime.BUSES) or
                any(tuple(wires[scope])!=batch for scope,batch in self._stop_batches.items())):
            raise ValueError('Exact all12 zero STOP diagnostic batches required; no Type1 API')
        if self._original_futures and not all(f.done() for f in self._original_futures):
            raise RuntimeError('Previous genuine output owners must finish before diagnostic reuse')
        originals=super().submit_decoded(wires,deadline_ns=deadline_ns,label=label)
        self._original_futures=list(originals.values())
        self._collector_output_transactions+=1
        return originals

    def collect_output(self,futures,**kwargs):
        self._caller_check()
        if (set(futures)!=set(runtime.BUSES) or
                tuple(futures.values())!=tuple(self._original_futures)):
            raise RuntimeError('Exact current original diagnostic output Futures required')
        self._collector_collect_attempts+=1
        return super().collect_output(futures,**kwargs)

    def evidence(self):
        self._caller_check()
        return {'schema':'singularitydog.diagnostic-runtime-output-owner.v1',
            'scope':'disabled_stop_proxy_only','actual_busworkers_submit_decoded':self._collector_output_transactions>0,
            'actual_busworkers_collect_output':self._collector_collect_attempts>0,'genuine_original_output_futures':True,
            'collector_worker_count':3,'extra_worker_or_reader_count':0,
            'borrowed_executor_owner':'original_collector','owns_or_closes_borrowed_executor':False,
            'output_transactions_submitted':self._collector_output_transactions,
            'output_collect_attempts':self._collector_collect_attempts,
            'native_feedback_batch_decode_selected':self.native_feedback_decoders is not None,
            'output_future_notifications_selected':self.unpaired_output_future_notifications,
            'notification_source_binding':self.notification_source_binding,
            'output_notification_groups':self.output_notification_groups,
            'output_notification_waits':self.output_notification_waits,
            'original_absolute_deadlines_unchanged':True,'type1_sent':False,
            'active_controller_qualification':False,'hardware_timing_improvement_proven':False,
            'output_allowed':False,'approved_for_runtime':False}

    def close(self):
        if self._closed:return
        self._caller_check()
        pending=(*self._original_futures,*(self.stop_futures or {}).values())
        if any(not f.done() for f in pending):
            raise RuntimeError('Original collector must join all output/cleanup owners before detach')
        # No shutdown, session.close, motor command or cancel here. Original
        # collector teardown is authoritative for pool and FD lifetimes.
        self.pools={};self._original_futures=[];self._closed=True


def make_borrowed_output(pool,sessions,cancel_io,*,clock,native_feedback_batch_decode=False,
                         native_feedback_codec_selection=None,
                         unpaired_output_future_notifications=False,notification_waiter=None):
    return _BorrowedDiagnosticOutput(pool,sessions,cancel_io,token=_BORROWED_COLLECTOR_TOKEN,
        clock=clock,native_feedback_batch_decode=native_feedback_batch_decode,
        native_feedback_codec_selection=native_feedback_codec_selection,
        unpaired_output_future_notifications=unpaired_output_future_notifications,
        notification_waiter=notification_waiter)
