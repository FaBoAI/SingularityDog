"""Offline fixed-scope dual-bus tests. Synthetic ports only."""
from contextlib import contextmanager, redirect_stdout, redirect_stderr
import copy
import io
import json
import os
from pathlib import Path
import stat
import signal
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import dual_can_pipeline_benchmark as dual
from singularitydog_hw.can_readonly import read_request
from test_can_pipeline_probe import Clock, Serial, UIDS
from test_pipeline_select_receive import SimulatedDeadlineReader

BINDINGS = {s: {'path':'/dev/serial/by-path/'+s,'resolved':'/dev/'+s,'st_rdev':i}
            for i,s in enumerate(('front','rear'),1)}


def fixture(scope='front', failure=None):
    clock = Clock()
    serial = Serial(clock, failure=failure)
    events=[]
    probe=dual.ScopedPipelineCAN(scope, BINDINGS[scope]['path'], events.append,
        serial_port=serial,clock=clock)
    with probe: report=probe.collect(UIDS)
    return probe,serial,events,report


class ScopeTests(unittest.TestCase):
    def test_select_configured_once_preserves_both_scopes_and_receiver_profile(self):
        for scope in dual.SCOPES:
            with self.subTest(scope=scope):
                clock=Clock();port=Serial(clock)
                probe=dual.ScopedPipelineCAN(scope,'fake',lambda row:None,
                    serial_port=port,clock=clock,receive_mode='select')
                with patch('singularitydog_hw.serial_deadline_reader.DeadlineSerialReader',
                           side_effect=SimulatedDeadlineReader) as reader:
                    with probe:report=probe.collect(UIDS)
                reader.assert_called_once()
                dual.require_complete(report,scope)
                self.assertEqual(report['plan']['receive_mode'],'select')
                self.assertEqual(report['receiver_profile']['mode'],'select')
                self.assertGreater(report['receiver_profile']['deadline_reader']['read_until_calls'],0)
                self.assertEqual([f.kind for f in port.writes],[0]*6+[17]*240)
                self.assertEqual(set(f.destination for f in port.writes),set(dual.SCOPES[scope]))
                self.assertTrue(all(b-a>=500_000 for a,b in zip(port.write_times,port.write_times[1:])))
                self.assertEqual(port.timeout_settings,[0])
                self.assertEqual(port.read_timeouts,[])
                self.assertTrue(port.closed and probe.device_closed)

    def test_select_setup_failure_closes_scoped_port_before_any_write(self):
        clock=Clock();port=Serial(clock)
        probe=dual.ScopedPipelineCAN('front','fake',lambda row:None,
            serial_port=port,clock=clock,receive_mode='select')
        with patch('singularitydog_hw.serial_deadline_reader.DeadlineSerialReader',
                   side_effect=OSError('select setup failed')):
            with self.assertRaisesRegex(OSError,'select setup failed'):
                with probe:self.fail('Failed receiver entered acquisition')
        self.assertTrue(port.closed and probe.device_closed and probe.poisoned)
        self.assertEqual(port.writes,[])

    def test_optional_cycle_gate_preserves_exact_scope_frames_and_source_times(self):
        clock=Clock();port=Serial(clock);events=[];seen=[]
        def gate(number):
            seen.append((number,len(port.writes)))
            arrived=clock();clock.advance(3_000_000)
            return {'arrived_ns':arrived,'released_ns':clock(),'wait_finished_ns':clock(),'wait_ms':3.}
        probe=dual.ScopedPipelineCAN('front','fake',events.append,serial_port=port,clock=clock,cycle_barrier=gate)
        with probe:report=probe.collect(UIDS)
        dual.require_complete(report,'front')
        self.assertEqual(seen,[(n,6+(n-1)*12) for n in range(1,21)])
        self.assertEqual([f.kind for f in port.writes],[0]*6+[17]*240)
        self.assertEqual(len([r for r in events if r['kind']=='dual_cycle_barrier']),20)
        self.assertTrue(all(r['write_started_monotonic_ns']>=report['cycles'][r['cycle']-1]['paired_barrier']['released_ns']
                            for r in report['requests'] if r['cycle']))
        self.assertTrue(port.closed)

    def test_exact_246_canonical_reads_each_scope_and_original_times(self):
        for scope in dual.SCOPES:
            with self.subTest(scope=scope):
                probe,port,events,report=fixture(scope)
                dual.require_complete(report,scope)
                self.assertTrue(port.closed and probe.device_closed)
                self.assertEqual([f.kind for f in port.writes],[0]*6+[17]*240)
                self.assertEqual([f.destination for f in port.writes[:6]],list(dual.SCOPES[scope]))
                self.assertEqual([(f.destination,int.from_bytes(f.data[:2],'little')) for f in port.writes[6:]],
                    [(mid,index) for index in (0x7019,0x701B) for mid in dual.SCOPES[scope]]*20)
                self.assertTrue(all(b-a>=500_000 for a,b in zip(port.write_times,port.write_times[1:])))
                self.assertTrue(all(r['write_started_monotonic_ns']<=r['write_finished_monotonic_ns']<=r['received_monotonic_ns']
                                    for r in report['requests']))
                self.assertTrue(all(not report[k] for k in ('output_allowed','motor_output_available','imu_accessed')))

    def test_actual_write_boundary_rejects_other_scope_and_noncanonical(self):
        for mid,parameter in ((7,None),(7,'position')):
            clock=Clock();port=Serial(clock)
            probe=dual.ScopedPipelineCAN('front','fake',lambda row:None,serial_port=port,clock=clock)
            with probe:
                with self.assertRaisesRegex(ValueError,'scope'):probe._send((mid,parameter),0)
            self.assertFalse(port.writes)
        clock=Clock();port=Serial(clock)
        probe=dual.ScopedPipelineCAN('front','fake',lambda row:None,serial_port=port,clock=clock)
        with probe:
            probe.current_write=(1,'position');probe.pending[(1,'position')]={'deadline_monotonic_ns':clock()+100_000_000}
            with self.assertRaises(ValueError):probe.serial.write(read_request(1,'velocity'))
        self.assertFalse(port.writes)

    def test_partial_timeout_duplicate_status_and_uid_fail_without_retry(self):
        for failure in ('shortwrite','timeout','duplicate','status','uid','noise','nan'):
            with self.subTest(failure=failure):
                probe,port,events,report=fixture(failure=failure)
                self.assertEqual(report['status'],'INCOMPLETE')
                self.assertTrue(report['errors'])
                self.assertLess(len(port.writes),246)
                self.assertTrue(port.closed)
                pairs=[(r['cycle'],r['motor_id'],r['parameter']) for r in report['requests']]
                self.assertEqual(len(pairs),len(set(pairs)))
                with self.assertRaises(ValueError):dual.require_complete(report,'front')

    def test_identity_barrier_precedes_first_parameter(self):
        clock=Clock();port=Serial(clock);seen=[]
        probe=dual.ScopedPipelineCAN('front','fake',lambda row:None,serial_port=port,clock=clock,
             identity_barrier=lambda:seen.append([f.kind for f in port.writes]))
        with probe:report=probe.collect(UIDS)
        self.assertEqual(seen,[[0]*6]);dual.require_complete(report,'front')

    def test_combined_span_includes_drift_and_rejects_incomplete_cycle(self):
        front=fixture('front')[3];rear=fixture('rear')[3]
        for r in rear['requests']:
            r['write_started_monotonic_ns']+=8_000_000
            r['received_monotonic_ns']+=8_000_000
        combined=dual.combined_cycles(front,rear)
        self.assertEqual(len(combined),20)
        for c in combined:
            rows=[r for report in (front,rear) for r in report['requests'] if r['cycle']==c['cycle']]
            self.assertEqual(c['combined_acquisition_span_ms'],(max(r['received_monotonic_ns'] for r in rows)-min(r['write_started_monotonic_ns'] for r in rows))/1e6)
            self.assertEqual(c['bus_start_skew_ms'],8.)
        rear['cycles'][0]['status']='INCOMPLETE'
        with self.assertRaises(ValueError):dual.combined_cycles(front,rear)


class CoordinatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.reports={s:fixture(s)[3] for s in dual.SCOPES}

    def run_coordinator(self, *, failure=None, opened_check=None, binding_check=None, join_grace=.03, hold=None, paired=False,
                        receive_mode='serial'):
        history=[];lock=threading.Lock();closes={};ready=set()
        @contextmanager
        def common():
            history.append('common-enter')
            try:yield
            finally:history.append('common-exit')
        @contextmanager
        def port(path):
            history.append('port-enter-'+path)
            try:yield
            finally:history.append('port-exit-'+path)
        reports=self.reports
        testcase=self
        class Probe:
            def __init__(self,scope,path,emit,**kw):
                self.scope,self.kw=scope,kw;self.device_closed=False
                testcase.assertEqual(kw['receive_mode'],receive_mode)
            def __enter__(self):
                if failure=='enter-close' and self.scope=='front':raise OSError('open then close failure')
                return self
            def __exit__(self,*args):
                if failure=='close' and self.scope=='front':raise OSError('close failure')
                self.device_closed=True;closes[self.scope]=True;history.append('closed-'+self.scope)
            def collect(self,expected):
                if failure=='identity' and self.scope=='front':raise ValueError('identity mismatch')
                with lock:ready.add(self.scope)
                self.kw['identity_barrier']()
                if ready!=set(dual.SCOPES):raise AssertionError('telemetry before both identities')
                history.append('telemetry-'+self.scope)
                if hold is not None and self.scope=='rear' and failure!='cycle-hang':hold.wait(3)
                if failure=='timeout' and self.scope=='front':raise TimeoutError('Missing reply within request deadline')
                if failure in ('timeout','identity'):
                    for _ in range(100):
                        self.kw['check_interrupt']();time.sleep(.001)
                self.kw['check_interrupt']()
                result=copy.deepcopy(reports[self.scope])
                if 'cycle_barrier' in self.kw:
                    for number in range(1,21):
                        if self.scope=='front' and number==3 and failure=='cycle':raise TimeoutError('cycle3 synthetic fail')
                        if self.scope=='front' and number==3 and failure=='cycle-hang':hold.wait(3)
                        gate=self.kw['cycle_barrier'](number)
                        history.append('cycle-'+self.scope+'-'+str(number))
                        result['cycles'][number-1]['paired_barrier']=gate
                        start=time.monotonic_ns()
                        rows=[r for r in result['requests'] if r['cycle']==number]
                        for index,row in enumerate(rows):
                            row['write_started_monotonic_ns']=start+index*1000
                            row['received_monotonic_ns']=start+index*1000+500
                        time.sleep(.001)
                return result
        result=dual.collect_dual(BINDINGS,UIDS,{s:lambda row:None for s in dual.SCOPES},max_seconds=1.,
            pipeline_factory=Probe,common_lock_factory=common,port_lock_factory=port,
            binding_check=binding_check or (lambda b:True),opened_binding_check=opened_check or (lambda p,b:True),
            join_grace_seconds=join_grace,paired_cycle_sync=paired,receive_mode=receive_mode)
        return result,history,closes

    def test_select_mode_reaches_both_workers_with_paired_gate_and_cleanup(self):
        report,history,closes=self.run_coordinator(paired=True,receive_mode='select')
        self.assertEqual(report['status'],'COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT',report['errors'])
        self.assertEqual(report['plan']['receive_mode'],'select')
        self.assertTrue(report['plan']['per_cycle_barrier'])
        self.assertEqual(set(closes),set(dual.SCOPES))
        self.assertTrue(report['producer_threads_exited'] and report['common_lock_released'])

    def test_both_workers_finish_and_close_before_common_release(self):
        report,history,closes=self.run_coordinator()
        self.assertEqual(report['status'],'COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT')
        self.assertTrue(report['producer_threads_exited'] and report['common_lock_released'])
        self.assertEqual(set(closes),set(dual.SCOPES))
        for scope in dual.SCOPES:self.assertLess(history.index('closed-'+scope),history.index('common-exit'))

    def test_paired_twenty_cycles_both_close_and_wait_accounting_is_separate(self):
        report,history,closes=self.run_coordinator(paired=True)
        self.assertEqual(report['status'],'COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT',report['errors'])
        self.assertEqual(len([h for h in history if h.startswith('cycle-')]),40)
        self.assertTrue(report['plan']['per_cycle_barrier'])
        timing=report['paired_cycle_timing']['cycles']
        self.assertEqual(len(timing),20)
        self.assertIsNone(timing[0]['paired_request_start_interval_ms'])
        for row,paired in zip(report['same_cycle'],timing):
            self.assertGreaterEqual(paired['gate_arrival_to_latest_reply_ms'],row['combined_acquisition_span_ms'])
        self.assertEqual(set(closes),set(dual.SCOPES))
        self.assertTrue(report['common_lock_released'])

    def test_cycle_failure_aborts_both_without_next_cycle_or_retry(self):
        report,history,closes=self.run_coordinator(paired=True,failure='cycle')
        self.assertEqual(report['status'],'INCOMPLETE')
        self.assertEqual(set(closes),set(dual.SCOPES))
        self.assertTrue(report['producer_threads_exited'] and report['common_lock_released'])
        self.assertFalse(any(h in history for h in ('cycle-front-3','cycle-rear-3')))

    def test_cycle_wait_timeout_with_hung_peer_retains_lease_until_actual_close(self):
        hold=threading.Event()
        try:
            report,history,closes=self.run_coordinator(paired=True,failure='cycle-hang',hold=hold,join_grace=.001)
            self.assertEqual(report['status'],'INCOMPLETE')
            self.assertFalse(report['common_lock_released'] or report['producer_threads_exited'])
            self.assertNotIn('common-exit',history)
        finally:hold.set()
        until=time.monotonic()+1
        while 'common-exit' not in history and time.monotonic()<until:time.sleep(.001)
        self.assertIn('closed-front',history)
        self.assertGreater(history.index('common-exit'),history.index('closed-front'))

    def test_one_timeout_stops_both_without_retry(self):
        report,history,closes=self.run_coordinator(failure='timeout')
        self.assertEqual(report['status'],'INCOMPLETE')
        self.assertTrue(report['producer_threads_exited'] and report['common_lock_released'])
        self.assertEqual(set(closes),set(dual.SCOPES))
        self.assertTrue(any('Missing reply' in e['error'] for e in report['errors']))

    def test_identity_failure_never_reaches_telemetry_on_either_bus(self):
        report,history,closes=self.run_coordinator(failure='identity')
        self.assertEqual(report['status'],'INCOMPLETE')
        self.assertFalse(any(h.startswith('telemetry-') for h in history))

    def test_enter_or_close_error_retains_common_lock(self):
        for failure in ('enter-close','close'):
            with self.subTest(failure=failure):
                report,history,closes=self.run_coordinator(failure=failure)
                self.assertEqual(report['status'],'INCOMPLETE')
                self.assertFalse(report['common_lock_released'])
                self.assertNotIn('common-exit',history)
                self.assertFalse(report['buses']['front']['device_closed'])

    def test_unjoined_worker_holds_common_lease_until_actual_close(self):
        hold=threading.Event()
        try:
            report,history,closes=self.run_coordinator(hold=hold,join_grace=.001)
            self.assertEqual(report['status'],'INCOMPLETE')
            self.assertFalse(report['producer_threads_exited'] or report['common_lock_released'])
            self.assertNotIn('common-exit',history)
        finally:hold.set()
        until=time.monotonic()+1
        while 'common-exit' not in history and time.monotonic()<until:time.sleep(.001)
        self.assertIn('closed-rear',history)
        self.assertGreater(history.index('common-exit'),history.index('closed-rear'))

    def test_post_open_mapping_mismatch_sends_no_reads(self):
        report,history,closes=self.run_coordinator(opened_check=lambda p,b:False)
        self.assertEqual(report['status'],'INCOMPLETE')
        self.assertFalse(any(h.startswith('telemetry-') for h in history))
        self.assertTrue(report['common_lock_released'])

    def test_same_device_number_rejected_before_lock_or_open(self):
        bindings=copy.deepcopy(BINDINGS);bindings['rear']['st_rdev']=bindings['front']['st_rdev']
        with self.assertRaisesRegex(ValueError,'Duplicate physical'):
            dual.collect_dual(bindings,UIDS,{},common_lock_factory=lambda:self.fail('lock opened'))


class PortAndPlanTests(unittest.TestCase):
    def test_alias_character_nodes_with_same_device_number_rejected(self):
        with patch.object(Path,'resolve',lambda p,strict=False:Path('/dev/'+p.name)),patch.object(Path,'stat',return_value=SimpleNamespace(st_mode=stat.S_IFCHR,st_rdev=81)):
            with self.assertRaisesRegex(ValueError,'same physical'):
                dual.validate_ports('/dev/serial/by-path/front','/dev/serial/by-path/rear')

    def test_actual_fd_device_number_must_match_pin(self):
        probe=SimpleNamespace(raw_port=SimpleNamespace(fileno=lambda:42))
        with patch.object(dual,'binding_matches',return_value=True),patch.object(os,'fstat',return_value=SimpleNamespace(st_mode=stat.S_IFCHR,st_rdev=99)):
            self.assertFalse(dual.opened_binding_matches(probe,BINDINGS['front']))

    def test_plan_is_no_io_fixed_profile_and_rejects_bad_budget(self):
        args=['--expected-uids','/missing','--front-port','/dev/serial/by-path/front','--rear-port','/dev/serial/by-path/rear','--output','/never-created']
        with patch.object(dual,'validate_ports',side_effect=AssertionError('IO')),redirect_stdout(io.StringIO()) as out:
            self.assertEqual(dual.main(args),0)
        plan=json.loads(out.getvalue())
        self.assertEqual((plan['window'],plan['gap_ms'],plan['cycles_per_bus'],plan['requests_per_bus']),(4,.5,20,246))
        self.assertEqual(plan['receive_mode'],'serial')
        self.assertFalse(plan['output_allowed'] or plan['per_cycle_barrier'])
        for invalid in (float('nan'),0,11,True):
            with self.assertRaises(ValueError):dual.make_plan('a','b',invalid)
        with self.assertRaises(ValueError):dual.make_plan('same','same')

    def test_select_plan_requires_explicit_choice_and_preserves_fixed_readonly_limits(self):
        args=['--receive-mode','select','--paired-cycle-sync','--expected-uids','/missing',
              '--front-port','/dev/serial/by-path/front','--rear-port','/dev/serial/by-path/rear',
              '--output','/never-created']
        with patch.object(dual,'validate_ports',side_effect=AssertionError('IO')),redirect_stdout(io.StringIO()) as out:
            self.assertEqual(dual.main(args),0)
        selected=json.loads(out.getvalue())
        baseline=dual.make_plan(BINDINGS['front']['path'],BINDINGS['rear']['path'],paired_cycle_sync=True)
        self.assertEqual(selected,{**baseline,'receive_mode':'select'})
        for mode in ('poll',None,True):
            with self.subTest(mode=mode),self.assertRaises(ValueError):
                dual.collect_dual(BINDINGS,UIDS,{},receive_mode=mode,
                                  common_lock_factory=lambda:self.fail('lock opened'))

    def test_paired_plan_explicit_boolean_only_preserves_limits_and_span_definition(self):
        ordinary=dual.make_plan('a','b');paired=dual.make_plan('a','b',paired_cycle_sync=True)
        for key in ('window','gap_ms','cycles_per_bus','requests_per_bus','total_request_limit',
                    'request_timeout_seconds','same_cycle_measure','allowed_can_types'):
            self.assertEqual(ordinary[key],paired[key])
        self.assertFalse(ordinary['per_cycle_barrier'])
        self.assertTrue(paired['per_cycle_barrier'])
        for bad in (1,None,'paired'):
            with self.assertRaises(ValueError):dual.make_plan('a','b',paired_cycle_sync=bad)


class CycleGateTests(unittest.TestCase):
    def gate(self,check=lambda:None):
        return dual.PairedCycleGate(check,time.monotonic_ns()+2_000_000_000)

    def test_ordered_pair_requires_both_arrivals_and_records_actual_wait(self):
        gate=self.gate();results={};arrived=threading.Event()
        def front():arrived.set();results['front']=gate.wait('front',1)
        t=threading.Thread(target=front);t.start();arrived.wait(1)
        time.sleep(.01)
        self.assertNotIn('front',results)
        results['rear']=gate.wait('rear',1);t.join(.5)
        self.assertFalse(t.is_alive())
        self.assertEqual(results['front']['released_ns'],results['rear']['released_ns'])
        self.assertGreater(results['front']['wait_ms'],5.)
        for row in results.values():self.assertLessEqual(row['arrived_ns'],row['released_ns'])
        with self.assertRaises(ValueError):gate.wait('front',1)

    def test_missing_peer_timeout_is_finite_and_later_peer_cannot_proceed(self):
        gate=self.gate();before=time.monotonic()
        with self.assertRaisesRegex(TimeoutError,'deadline'):gate.wait('front',1)
        self.assertLess(time.monotonic()-before,.7)
        with self.assertRaises(ValueError):gate.wait('rear',1)

    def test_abort_and_external_cancel_wake_waiter_without_peer(self):
        for external in (False,True):
            cancel=threading.Event();errors=[];entered=threading.Event()
            def check():
                entered.set()
                if cancel.is_set():raise InterruptedError('synthetic cancel')
            gate=self.gate(check)
            def worker():
                try:gate.wait('front',1)
                except BaseException as error:errors.append(error)
            t=threading.Thread(target=worker);t.start();entered.wait(1)
            if external:cancel.set()
            else:gate.abort()
            t.join(.2)
            self.assertFalse(t.is_alive());self.assertEqual(len(errors),1)

    def test_blocked_external_check_does_not_hold_gate_mutex(self):
        entered=threading.Event();release=threading.Event();errors=[]
        def check():entered.set();release.wait(1)
        gate=self.gate(check)
        def worker():
            try:gate.wait('front',1)
            except BaseException as error:errors.append(error)
        t=threading.Thread(target=worker);t.start();entered.wait(1)
        try:
            before=time.monotonic();gate.abort()
            self.assertLess(time.monotonic()-before,.1)
        finally:release.set();t.join(1)
        self.assertTrue(errors);self.assertFalse(t.is_alive())

    def test_skip_wrong_scope_bool_index_and_total_deadline_fail_closed(self):
        for scope,index in (('front',2),('front',True),('front',21),('unknown',1)):
            with self.assertRaises(ValueError):self.gate().wait(scope,index)
        gate=dual.PairedCycleGate(lambda:None,time.monotonic_ns()-1)
        with self.assertRaises(TimeoutError):gate.wait('front',1)


class BufferedLoggingTests(unittest.TestCase):
    def test_emit_preserves_original_values_without_json_or_file_io(self):
        buffer=dual.EventBuffer('front')
        row={'kind':'pipeline_reply','monotonic_ns':123,'result':{'value':1.2}}
        with patch.object(json,'dumps',side_effect=AssertionError('hot-path JSON')),patch.object(os,'open',side_effect=AssertionError('hot-path file I/O')):
            buffer.emit(row)
        row['result']['value']=9.;row['monotonic_ns']=999
        self.assertEqual(buffer.rows[0]['monotonic_ns'],123)
        self.assertEqual(buffer.rows[0]['result']['value'],1.2)

    def test_buffer_has_finite_event_bound_and_cannot_append_after_flush(self):
        buffer=dual.EventBuffer('rear')
        with patch.object(dual,'MAX_EVENTS_PER_BUS',2):
            buffer.emit({'kind':'a'});buffer.emit({'kind':'b'})
            with self.assertRaisesRegex(ValueError,'budget'):buffer.emit({'kind':'c'})
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'events.jsonl'
            self.assertEqual(buffer.flush(path),2)
            self.assertEqual([json.loads(x)['kind'] for x in path.read_text().splitlines()],['a','b'])
            self.assertEqual(stat.S_IMODE(path.stat().st_mode),0o600)
            with self.assertRaisesRegex(ValueError,'sealed'):buffer.emit({'kind':'d'})

    def run_main(self, *, alive=False, flush_error=False, signal_during_flush=False,receive_mode=None):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        output=Path(temp.name)/'logs';uids=Path(temp.name)/'uids.json'
        uids.write_text(json.dumps(UIDS))
        history=[]
        class Guard:
            boot_id='00000000-0000-0000-0000-000000000001'
            def check(self):pass
            def close(self):history.append('boot-close')
        def collect(bindings,expected,emitters,*args,**kwargs):
            self.assertEqual(kwargs['receive_mode'],receive_mode or 'serial')
            for scope,emit in emitters.items():
                self.assertFalse((output/('events-'+scope+'.jsonl')).exists())
                emit({'kind':'pipeline_rx_bytes','monotonic_ns':12,'hex':'4154'})
            history.append('workers-alive' if alive else 'both-workers-closed')
            return {'status':'INCOMPLETE' if alive else 'COMPLETE_DUAL_CAN_BENCHMARK_NO_OUTPUT',
                    'producer_threads_exited':not alive,'common_lock_released':not alive}
        original=dual.EventBuffer.flush
        def flush(buffer,path):
            self.assertIn('both-workers-closed',history)
            history.append('flush-'+buffer.scope)
            if flush_error:raise OSError('fsync failed')
            result=original(buffer,path)
            if signal_during_flush:signal.getsignal(signal.SIGTERM)(signal.SIGTERM,None)
            return result
        argv=['--execute-no-output','--expected-uids',str(uids),'--front-port',BINDINGS['front']['path'],
              '--rear-port',BINDINGS['rear']['path'],'--output',str(output)]
        if receive_mode is not None:argv+=['--receive-mode',receive_mode]
        with patch.object(dual,'validate_ports',return_value=BINDINGS),patch.object(dual,'BootIdentityGuard',Guard),patch.object(dual,'collect_dual',side_effect=collect),patch.object(dual.EventBuffer,'flush',flush),redirect_stdout(io.StringIO()):
            status=dual.main(argv)
        return status,json.loads((output/'summary.json').read_text()),output,history

    def test_main_flushes_only_after_workers_exit_then_fsync_and_hash(self):
        code,report,output,history=self.run_main()
        self.assertEqual(code,0)
        self.assertEqual(report['events_written'],{'front':1,'rear':1})
        self.assertEqual(set(report['events_sha256']),set(dual.SCOPES))
        self.assertTrue(report['buffer_captures_final'])
        self.assertLess(history.index('both-workers-closed'),history.index('flush-front'))

    def test_main_passes_select_mode_and_records_reader_source(self):
        code,report,output,history=self.run_main(receive_mode='select')
        self.assertEqual(code,0)
        self.assertEqual(report['plan']['receive_mode'],'select')
        self.assertIn('serial_deadline_reader.py',report['source_sha256'])
        self.assertEqual(len(report['source_sha256']['serial_deadline_reader.py']),64)
        self.assertTrue(all(report['events_flush_confirmed'].values()))

    def test_unjoined_workers_never_flush_or_hash_unstable_buffers(self):
        code,report,output,history=self.run_main(alive=True)
        self.assertEqual(code,2)
        self.assertEqual(report['events_sha256'],{})
        self.assertEqual(report['events_written'],{'front':0,'rear':0})
        self.assertFalse(report['buffer_captures_final'])
        self.assertFalse(list(output.glob('events-*.jsonl')))

    def test_failed_flush_or_cleanup_signal_never_reports_success(self):
        for options in ({'flush_error':True},{'signal_during_flush':True}):
            with self.subTest(options=options):
                code,report,output,history=self.run_main(**options)
                self.assertEqual(code,2);self.assertEqual(report['status'],'INCOMPLETE')
                if options.get('flush_error'):self.assertEqual(report['events_sha256'],{})


if __name__=='__main__':unittest.main()
