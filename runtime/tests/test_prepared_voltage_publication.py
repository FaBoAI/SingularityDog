"""Explicit active voltage preparation publication; mocks, no native/device I/O."""
import ctypes as C
from concurrent.futures import Future
from dataclasses import FrozenInstanceError
import inspect
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime
from test_policy_output_runtime import FakeIMU, FakeSession, SimulatedClock, encode_motion, profile


class MockLibrary:
    def __init__(self, clock):
        self.clock=clock;self.calls=[];self.created=[];self.before_call=None
        self.backend=FakeSession(1,clock=clock);self.backend.enabled.update(self.backend.ids)
    def sda_create(self,*args):self.created.append(args);return 123
    def sda_destroy(self,handle):pass
    def sda_exchange(self,handle,raw,count,send_only,deadline,records,stats,error,error_size):
        if self.before_call:self.before_call()
        wires=tuple(bytes(raw[i*17:(i+1)*17]) for i in range(count))
        self.calls.append((wires,send_only,deadline,records,stats,error))
        out,info=self.backend.exchange(wires,deadline_ns=deadline)
        for i,row in enumerate(out):C.memmove(C.addressof(records[i]),C.addressof(row),C.sizeof(row))
        target=C.cast(stats,C.POINTER(native.Stats)).contents
        for name in ('begin_ns','end_ns','waits','reads','bytes','writes'):setattr(target,name,getattr(info,name))
        return 0


def active_session(clock):
    lib=MockLibrary(clock)
    s=native.ActiveSession(lib,123,first_id=1,cancel_fd=124,boot_fd=125,
        boot_id='11111111-2222-3333-4444-555555555555',
        raw_lower_by_id={i:-1. for i in range(1,7)},raw_upper_by_id={i:1. for i in range(1,7)},
        kp_max_by_id={i:3. for i in range(1,7)},kd_max_by_id={i:.15 for i in range(1,7)})
    return s,lib


class ActivePreparationTests(unittest.TestCase):
    def test_buffers_arguments_and_owner_prepared_before_hook_raw_proof_unchanged(self):
        clock=SimulatedClock();s,lib=active_session(clock);events=[]
        wires=[runtime.codec.read_request(1,'voltage')];deadline=clock()+20_000_000
        create=native.C.create_string_buffer
        def allocate(*args):events.append('buffer');return create(*args)
        def hook():
            events.append('publish');self.assertTrue(s.busy.locked());self.assertFalse(lib.calls)
        lib.before_call=lambda:events.append('native')
        with patch.object(native.time,'monotonic_ns',clock),patch.object(native.C,'create_string_buffer',side_effect=allocate):
            records,stats=s.exchange(wires,deadline_ns=deadline,before_native=hook)
        self.assertEqual(events,['buffer','publish','native'])
        self.assertEqual(lib.calls[0][0],tuple(wires));self.assertEqual(lib.calls[0][2],deadline)
        self.assertIs(records,lib.calls[0][3]);self.assertEqual(bytes(records[0].tx),wires[0])
        self.assertEqual(records[0].deadline_ns,deadline);self.assertFalse(s.poisoned);self.assertFalse(s.busy.locked())
        saved=bytes(records)
        with patch.object(native.time,'monotonic_ns',clock):s.exchange(wires,deadline_ns=deadline)
        self.assertEqual(bytes(records),saved,'Later exchange must not reuse owned proof buffers')

    def test_hook_may_only_narrow_original_absolute_deadline(self):
        for value in ('narrow','same','extend','bool','zero','float'):
            with self.subTest(value=value):
                clock=SimulatedClock();s,lib=active_session(clock);original=clock()+20_000_000
                returned={'narrow':original-1_000_000,'same':original,'extend':original+1,'bool':True,'zero':0,'float':float(original)}[value]
                with patch.object(native.time,'monotonic_ns',clock):
                    if value in ('narrow','same'):
                        records,_=s.exchange([runtime.codec.read_request(1,'voltage')],deadline_ns=original,before_native=lambda:returned)
                        self.assertEqual(records[0].deadline_ns,returned)
                    else:
                        with self.assertRaisesRegex(ValueError,'tighten'):
                            s.exchange([runtime.codec.read_request(1,'voltage')],deadline_ns=original,before_native=lambda:returned)
                        self.assertFalse(lib.calls);self.assertTrue(s.poisoned)
                self.assertFalse(s.busy.locked())

    def test_expiry_before_hook_does_not_publish_or_call_native(self):
        clock=SimulatedClock();s,lib=active_session(clock);hook=Mock();deadline=clock()+1000
        with patch.object(native.time,'monotonic_ns',side_effect=(deadline-1,deadline)):
            with self.assertRaisesRegex(TimeoutError,'before prepared'):
                s.exchange([runtime.codec.read_request(1,'voltage')],deadline_ns=deadline,before_native=hook)
        hook.assert_not_called();self.assertFalse(lib.calls);self.assertTrue(s.poisoned);self.assertFalse(s.busy.locked())

    def test_throw_or_late_hook_poisons_releases_owner_and_never_calls_native(self):
        for failure in ('throw','late'):
            clock=SimulatedClock();s,lib=active_session(clock);deadline=clock()+1_000_000
            def hook():
                if failure=='throw':raise KeyboardInterrupt('Injected hook interruption')
                clock.advance_to(deadline)
            with patch.object(native.time,'monotonic_ns',clock):
                with self.assertRaises(KeyboardInterrupt if failure=='throw' else TimeoutError):
                    s.exchange([runtime.codec.read_request(1,'voltage')],deadline_ns=deadline,before_native=hook)
            self.assertFalse(lib.calls);self.assertTrue(s.poisoned);self.assertFalse(s.busy.locked())

    def test_invalid_hook_send_only_hook_and_concurrent_owner_are_rejected(self):
        for value in (True,1,'callback'):
            clock=SimulatedClock();s,lib=active_session(clock)
            with self.assertRaisesRegex(ValueError,'callable'):
                s.exchange([runtime.codec.read_request(1)],before_native=value)
            self.assertFalse(lib.calls);self.assertTrue(s.poisoned)
        clock=SimulatedClock();s,lib=active_session(clock);hook=Mock()
        with self.assertRaisesRegex(ValueError,'acknowledged'):
            s._call([runtime.codec.read_request(1)],10_000_000,True,before_native=hook)
        hook.assert_not_called();self.assertFalse(lib.calls)
        s,lib=active_session(clock);s.busy.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError,'Concurrent'):
                s.exchange([runtime.codec.read_request(1)],before_native=hook)
            self.assertTrue(s.busy.locked());self.assertFalse(lib.calls)
        finally:s.busy.release()

    def test_default_and_send_only_do_not_add_hook_clock_or_return_semantics(self):
        for send_only in (False,True):
            clock=SimulatedClock();s,lib=active_session(clock)
            with patch.object(native.time,'monotonic_ns',return_value=1_000_000_000) as now:
                fn=s.send_only if send_only else s.exchange
                fn([runtime.codec.read_request(1)],timeout_ns=10_000_000)
            now.assert_called_once();self.assertEqual(lib.calls[0][1],int(send_only))
            self.assertEqual(lib.calls[0][2],1_010_000_000)


class PreparedFakeSession(FakeSession):
    prepared_exchange_capability=native.PREPARED_EXCHANGE_CAPABILITY
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.events=[];self.ready=None;self.on_prepare=None;self.call_hook=True;self.double_hook=False
    def exchange(self,wires,*,timeout_ns=100_000_000,deadline_ns=None,before_native=None):
        if len(wires)==1 and runtime.codec.ATParser().feed(wires[0])[0].kind==17:
            self.events.append(('voltage_prepare',self.ready.done()))
            if self.on_prepare:self.on_prepare()
            if before_native is not None and self.call_hook:
                tightened=before_native();self.events.append(('published',self.ready.done()))
                if tightened is not None:deadline_ns=tightened
                if self.double_hook:before_native()
            self.events.append(('voltage_native',self.ready.done()))
        return super().exchange(wires,timeout_ns=timeout_ns,deadline_ns=deadline_ns)


class PreparedOwnerTests(unittest.TestCase):
    def fixture(self,selected=True):
        clock=SimulatedClock();sessions={scope:PreparedFakeSession(first,clock=clock) for scope,first in [('front',1),('rear',7)]}
        for session in sessions.values():session.enabled.update(session.ids)
        cancel=Mock();workers=runtime.BusWorkers(sessions,cancel,clock,
            prepare_voltage_before_feedback_publication=selected)
        self.addCleanup(workers.close);self.addCleanup(workers.finish_stops)
        ready=Future();sessions['front'].ready=ready
        deadline=[clock()+20_000_000]
        wires=[encode_motion(i,0.,0.,0.) for i in sessions['front'].ids]
        return clock,sessions,workers,ready,deadline,wires
    def run_owner(self,f):
        clock,sessions,workers,ready,deadline,wires=f
        return workers._feedback_then_voltage('front',wires,1,profile(),deadline,ready,None)

    def test_explicit_optin_changes_only_publication_order_default_kept(self):
        for selected in (False,True):
            f=self.fixture(selected);_,sessions,_,ready,deadline,_=f
            result=self.run_owner(f)
            self.assertEqual(sessions['front'].events,
                [('voltage_prepare',False),('published',True),('voltage_native',True)] if selected else
                [('voltage_prepare',True),('voltage_native',True)])
            self.assertEqual([call[1] for call in sessions['front'].calls],[1]*6+[17])
            self.assertTrue(ready.done());self.assertEqual(len(ready.result()[0][0]),6)
            self.assertEqual(result[0][0][0].deadline_ns,deadline[0])

    def test_selection_strict_bool_and_capability_checked_before_executor_creation(self):
        for defect in ('nonbool','missing','lookalike'):
            sessions={'front':PreparedFakeSession(1),'rear':PreparedFakeSession(7)}
            selected=True
            if defect=='nonbool':selected=1
            if defect=='missing':sessions['rear'].prepared_exchange_capability=None
            if defect=='lookalike':sessions['rear'].prepared_exchange_capability=native.PreparedExchangeCapability()
            with patch.object(runtime,'ThreadPoolExecutor') as pools:
                with self.assertRaises(RuntimeError):runtime.BusWorkers(sessions,lambda:None,prepare_voltage_before_feedback_publication=selected)
                pools.assert_not_called()
        with self.assertRaises(FrozenInstanceError):native.PREPARED_EXCHANGE_CAPABILITY.schema='other'
        self.assertIs(inspect.signature(runtime.run_supported_policy).parameters['prepare_voltage_before_feedback_publication'].default,False)

    def test_shared_deadline_narrowed_before_and_during_publication_is_forwarded(self):
        f=self.fixture();clock,sessions,workers,ready,deadline,_=f;initial=deadline[0]
        sessions['front'].on_prepare=lambda:deadline.__setitem__(0,initial-1_000_000)
        ready.add_done_callback(lambda _:deadline.__setitem__(0,initial-2_000_000))
        result=self.run_owner(f);self.assertEqual(result[0][0][0].deadline_ns,initial-2_000_000)
        self.assertFalse(workers.aborted.is_set())
        proof=workers.prepared_voltage_publications[0]
        self.assertEqual(proof['status'],'VALIDATED')
        self.assertEqual(proof['submitted_deadline_ns'],initial)
        self.assertEqual(proof['effective_deadline_ns'],initial-2_000_000)

    def test_audit_native_edge_or_deadline_corruption_cannot_qualify_voltage(self):
        for defect in ('noncausal','deadline'):
            with self.subTest(defect=defect):
                f=self.fixture();_,_,workers,ready,_,_=f;original=workers._voltage
                def corrupt(*args,**kwargs):
                    value=original(*args,**kwargs)
                    if defect=='noncausal':value[0][1].begin_ns=1
                    else:value[0][0][0].deadline_ns+=1
                    return value
                with patch.object(workers,'_voltage',side_effect=corrupt):
                    with self.assertRaisesRegex(RuntimeError,'noncausal or has changed deadline'):
                        self.run_owner(f)
                self.assertTrue(ready.done());self.assertTrue(workers.aborted.is_set())
                self.assertIsNotNone(workers.stop_futures)
                proof=workers.prepared_voltage_publications[0]
                self.assertEqual(proof['status'],'FAILED')
                self.assertIn('noncausal or has changed deadline',proof['error'])

    def test_default_publication_has_no_selected_audit_allocation(self):
        f=self.fixture(False);workers=f[2]
        self.run_owner(f)
        self.assertIsNone(workers.prepared_voltage_publications)
        self.assertIsNone(workers.prepared_voltage_counts)

    def test_cancel_before_or_during_publication_blocks_voltage_and_queues_stop(self):
        for at_publish in (False,True):
            f=self.fixture();_,sessions,workers,ready,_,_=f
            if at_publish:ready.add_done_callback(lambda _:workers.aborted.set())
            else:sessions['front'].on_prepare=workers.aborted.set
            with self.assertRaisesRegex(RuntimeError,'cancelled around'):self.run_owner(f)
            if at_publish:self.assertIsNone(ready.exception())
            else:self.assertIsNotNone(ready.exception())
            self.assertFalse(any(c[1]==17 for c in sessions['front'].calls))
            self.assertIsNotNone(workers.stop_futures);workers.cancel_io.assert_called_once()
            proof=workers.prepared_voltage_publications[0]
            self.assertEqual(proof['status'],'FAILED')
            self.assertIsNone(proof['voltage_native_begin_ns'])
            self.assertIsNone(proof['effective_deadline_ns'])
            self.assertIn('cancelled around',proof['error'])

    def test_shared_deadline_expired_or_extended_before_publication_blocks_voltage(self):
        for defect in ('expired','extended','invalid'):
            f=self.fixture();clock,sessions,workers,ready,deadline,_=f;initial=deadline[0]
            def change():
                deadline[0]={'expired':clock()-1,'extended':initial+1,'invalid':True}[defect]
            sessions['front'].on_prepare=change
            with self.assertRaisesRegex(RuntimeError,'deadline'):self.run_owner(f)
            self.assertIsNotNone(ready.exception());self.assertFalse(any(c[1]==17 for c in sessions['front'].calls))
            self.assertTrue(workers.aborted.is_set())

    def test_expiry_during_publication_and_callback_interrupt_are_fail_closed(self):
        for interrupt in (False,True):
            f=self.fixture();clock,sessions,workers,ready,deadline,_=f
            def on_done(_):
                if interrupt:raise KeyboardInterrupt('Injected Future callback interruption')
                clock.advance_to(deadline[0])
            ready.add_done_callback(on_done)
            with self.assertRaises(KeyboardInterrupt if interrupt else RuntimeError):self.run_owner(f)
            self.assertTrue(workers.aborted.is_set());self.assertFalse(any(c[1]==17 for c in sessions['front'].calls))

    def test_bad_feedback_never_published_and_no_voltage_preparation(self):
        f=self.fixture();_,sessions,workers,ready,_,_=f;original=sessions['front'].exchange
        def damaged(*args,**kw):
            records,stats=original(*args,**kw);records[-1].rx[-1]=0;return records,stats
        sessions['front'].exchange=damaged
        with self.assertRaisesRegex(RuntimeError,'Invalid native frame'):self.run_owner(f)
        self.assertIsNotNone(ready.exception());self.assertEqual(sessions['front'].events,[])
        self.assertTrue(workers.aborted.is_set())

    def test_hook_missing_or_duplicate_never_grants_valid_voltage_result(self):
        for duplicate in (False,True):
            f=self.fixture();_,sessions,workers,ready,_,_=f
            sessions['front'].call_hook=duplicate;sessions['front'].double_hook=duplicate
            with self.assertRaisesRegex(RuntimeError,'exactly once|without publishing'):self.run_owner(f)
            self.assertTrue(workers.aborted.is_set())
            if not duplicate:self.assertIsNotNone(ready.exception())

    def test_native_preparation_failure_publishes_error_without_voltage_or_deadlock(self):
        f=self.fixture();_,sessions,workers,ready,_,_=f
        sessions['front'].on_prepare=Mock(side_effect=ValueError('Injected buffer preparation failure'))
        with self.assertRaisesRegex(ValueError,'buffer preparation'):self.run_owner(f)
        self.assertIsInstance(ready.exception(),ValueError);self.assertFalse(any(c[1]==17 for c in sessions['front'].calls))
        self.assertTrue(workers.aborted.is_set())

    def test_buses_progress_independently_without_publishing_unprepared_peer(self):
        f=self.fixture();clock,sessions,workers,_,deadline,_=f
        entered=threading.Event();release=threading.Event();front_published=threading.Event();ready={s:Future() for s in sessions}
        for scope in sessions:sessions[scope].ready=ready[scope]
        ready['front'].add_done_callback(lambda _:front_published.set())
        def block():
            entered.set()
            if not release.wait(1):raise TimeoutError('Synthetic rear preparation stalled')
        sessions['rear'].on_prepare=block
        tasks={}
        try:
            for scope in sessions:
                wires=[encode_motion(i,0.,0.,0.) for i in sessions[scope].ids]
                tasks[scope]=workers.pools[scope].submit(workers._feedback_then_voltage,
                    scope,wires,sessions[scope].ids[0],profile(),deadline,ready[scope],None)
            self.assertTrue(entered.wait(1));self.assertTrue(front_published.wait(1))
            self.assertFalse(ready['rear'].done());self.assertEqual(sessions['rear'].events,[('voltage_prepare',False)])
            release.set()
            values={s:f.result(timeout=1) for s,f in tasks.items()}
            self.assertEqual(set(values),{'front','rear'})
            self.assertTrue(all(f.done() and f.exception() is None for f in ready.values()))
            self.assertFalse(workers.aborted.is_set())
        finally:
            release.set()
            for task in tasks.values():
                try:task.result(timeout=1)
                except BaseException:pass


class PreparedRuntimeTests(unittest.TestCase):
    def test_full_coordinator_selected_path_preserves_count_limits_and_final_stops(self):
        from singularitydog_hw import policy_live_profile as profiles
        class SelectedSession(FakeSession):
            prepared_exchange_capability=native.PREPARED_EXCHANGE_CAPABILITY
            hook_calls=0
            def exchange(self,wires,*,timeout_ns=100_000_000,deadline_ns=None,before_native=None):
                if before_native is not None:
                    self.hook_calls+=1
                    self.assert_voltage_wire(wires)
                    narrowed=before_native()
                    if narrowed is not None:deadline_ns=narrowed
                return super().exchange(wires,timeout_ns=timeout_ns,deadline_ns=deadline_ns)
            def assert_voltage_wire(self,wires):
                if len(wires)!=1 or runtime.codec.ATParser().feed(wires[0])[0].kind!=17:
                    raise AssertionError('Hook used outside rotating voltage phase')
        data=profile();data.update(schema=profiles.SCHEMA_V3,
            telemetry_cadence=profiles.CADENCE_PRE_ENABLE,
            cadence_source_sha256=profiles.cadence_source_hashes(),voltage_overlap=True)
        clock=SimulatedClock();sessions={s:SelectedSession(i,clock=clock) for s,i in [('front',1),('rear',7)]}
        settings=runtime.execution_settings
        with patch.object(runtime,'execution_settings',side_effect=lambda p:{**settings(p),'voltage_pipeline':True}), \
             patch.object(runtime,'prepared_voltage_publication_settings',return_value=True):
            report=runtime.run_supported_policy(data,sessions,FakeIMU(clock=clock),lambda *_:(.04,)*12,
                cancel_io=lambda:None,encode_motion=encode_motion,clock=clock,sleep=clock.sleep,
                prepare_voltage_before_feedback_publication=True)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(report['prepare_voltage_before_feedback_publication'])
        self.assertTrue(report['stop_confirmed']);self.assertTrue(report['normal_ramp_completed'])
        self.assertFalse(report['full_controller_50Hz_verified'])
        proof=report['prepared_voltage_publication']
        self.assertTrue(proof['selection_bound_to_reviewed_profile'])
        self.assertEqual(proof['cadence_source_sha256'],data['cadence_source_sha256'])
        self.assertEqual(len(proof['records']),2*len(report['cycles']))
        self.assertFalse(proof['hardware_timing_improvement_proven'])
        for row in proof['records']:
            self.assertEqual(row['status'],'VALIDATED');self.assertIsNone(row['error'])
            self.assertLessEqual(row['feedback_validated_ns'],row['prepared_before_publish_ns'])
            self.assertLessEqual(row['prepared_before_publish_ns'],row['publication_checked_after_ns'])
            self.assertLessEqual(row['publication_checked_after_ns'],row['voltage_native_begin_ns'])
            self.assertLessEqual(row['voltage_native_begin_ns'],row['voltage_first_request_ns'])
            self.assertLessEqual(row['voltage_first_request_ns'],row['voltage_validated_ns'])
            self.assertLessEqual(row['effective_deadline_ns'],row['submitted_deadline_ns'])
        for scope,session in sessions.items():
            self.assertEqual(session.hook_calls,len(report['cycles']))
            self.assertEqual(len(session.stop_times),1)
            for phases,count in [(('feedback_hold',),6),(('overlapped_voltage',),1),
                                 (('policy_output','startup_hold','graceful_stop'),6)]:
                rows=[r for r in report['journal'] if r['phase'] in phases and r['bus']==scope]
                self.assertEqual(len(rows),len(report['cycles']))
                self.assertTrue(all(len(r['records'])==count for r in rows))
        self.assertTrue(all(c['iteration_ms']<data['hard_cycle_ms'] for c in report['cycles']))

    def test_reviewed_selection_mismatch_rejected_before_workers_or_any_IO(self):
        for selected,reviewed in ((True,False),(False,True)):
            with patch.object(runtime,'prepared_voltage_publication_settings',return_value=reviewed), \
                 patch.object(runtime,'BusWorkers') as workers:
                with self.assertRaisesRegex(RuntimeError,'differs from reviewed profile'):
                    runtime.run_supported_policy(profile(),{},Mock(),Mock(),cancel_io=Mock(),
                        prepare_voltage_before_feedback_publication=selected)
                workers.assert_not_called()

    def test_selected_review_cannot_enter_a_ground_or_human_supervisor(self):
        with patch.object(runtime,'prepared_voltage_publication_settings',return_value=True), \
             patch.object(runtime,'BusWorkers') as workers:
            with self.assertRaisesRegex(RuntimeError,'ordinary box-supported'):
                runtime.run_supported_policy(profile(),{},Mock(),Mock(),cancel_io=Mock(),
                    supervision=Mock(),prepare_voltage_before_feedback_publication=True)
            workers.assert_not_called()

    def test_nonpipeline_and_nonboolean_selection_rejected_before_workers(self):
        for selected in (True,1):
            with patch.object(runtime,'BusWorkers') as workers:
                with self.assertRaisesRegex(RuntimeError,'Prepared.*selection|requires selected V3 voltage pipeline'):
                    runtime.run_supported_policy(profile(),{},lambda:None,lambda:None,
                        cancel_io=lambda:None,prepare_voltage_before_feedback_publication=selected)
                workers.assert_not_called()


if __name__=='__main__':unittest.main()
