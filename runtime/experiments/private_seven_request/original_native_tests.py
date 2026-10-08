"""Genuine private C++ over socket/PTY, no physical ports or network."""
import ctypes as C
from concurrent.futures import Future,ThreadPoolExecutor
from dataclasses import FrozenInstanceError
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import struct
import sys
import tempfile
import threading
import time
import tty
import unittest
from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.native_diagnostic_transport import stop_wire,Record
import seven_request_candidate as candidate

BASE=Path(__file__).parent
BOOT='11111111-2222-3333-4444-555555555555'

def frame(canid,data):return b'AT'+((canid<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
def reply(wire,*,fault=0,mode=0,value=40.):
    f=ATParser().feed(wire)[0];mid=f.destination
    if f.kind==4:return frame((2<<24)|(mode<<22)|(fault<<16)|(mid<<8)|0xfd,struct.pack('>4H',32767,32767,32767,250))
    return frame((17<<24)|(fault<<16)|(mid<<8)|0xfd,f.data[:4]+struct.pack('<f',value))


class SplitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Match the canonical Python switch setting while keeping genuine20ms
        # product deadlines. Ordinary5ms run r4 failures remain archived.
        cls.original_switch=sys.getswitchinterval();sys.setswitchinterval(.0001)
        cls.actual_switch=sys.getswitchinterval()
        assert abs(cls.actual_switch-.0001)<=1e-12
        spec=importlib.util.spec_from_file_location('build_private',BASE/'build.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        cls.path=mod.build();cls.lib=candidate.load_candidate(cls.path)

    @classmethod
    def tearDownClass(cls):
        # CPython stores integer microseconds. A getter such as
        # 9.999999999999999e-05 multiplied on re-set can truncate to 99us.
        # The next representable input rounds to the SAME original integer
        # microseconds; require exact getter equality, never waive restoration.
        sys.setswitchinterval(math.nextafter(cls.original_switch,math.inf))
        assert sys.getswitchinterval()==cls.original_switch

    def setUp(self):
        self.host,self.peer=socket.socketpair();self.host.setblocking(False);self.peer.setblocking(False)
        self.cancel_read,self.cancel_write=os.pipe()
        self.boot=tempfile.TemporaryFile();self.boot.write((BOOT+'\n').encode());self.boot.flush()
        self.sessions=[];self.phases=[];self.peer_threads=[];self.peer_errors=[];self.peer_events=[]
        self.stop_peer=threading.Event();self.pool=ThreadPoolExecutor(max_workers=2)
        # Prestart workers before the measured deadline. Record parser-ready
        # handshake separately; injected protocol delays remain inside20ms.
        barrier=threading.Barrier(3)
        fs=[self.pool.submit(barrier.wait) for _ in range(2)];barrier.wait()
        for f in fs:f.result()
        self.session=self.make_session(self.host.fileno())

    def make_session(self,fd,first=1):
        ids=range(first,first+6)
        s=active.ActiveSession(self.lib,fd,first_id=first,cancel_fd=self.cancel_read,
            boot_fd=self.boot.fileno(),boot_id=BOOT,gap_ns=900_000,window=3,
            raw_lower_by_id={i:-1. for i in ids},raw_upper_by_id={i:1. for i in ids},
            kp_max_by_id={i:3. for i in ids},kd_max_by_id={i:.15 for i in ids})
        self.sessions.append(s);return s

    def phase(self,session=None,generation=1,voltage_id=None):
        session=session or self.session
        p=candidate.SevenRequestPhase(session,self.pool,cancel_fd=self.cancel_read,
            generation=generation,voltage_id=session.first_id if voltage_id is None else voltage_id)
        self.phases.append(p);return p

    def tearDown(self):
        # Bounded cleanup has no new command. Cancel unresolved owners, join,
        # THEN pipe destruction/session release. Never pass after cleanup time.
        try:os.write(self.cancel_write,b'x')
        except OSError:pass
        for p in self.phases:
            if not p._closed:
                try:p.close()
                except BaseException:pass
        self.pool.shutdown(wait=True)
        for s in self.sessions:s.close()
        self.stop_peer.set()
        for t in self.peer_threads:t.join(.2)
        self.host.close();self.peer.close()
        os.close(self.cancel_read);os.close(self.cancel_write);self.boot.close()
        evidence=[]
        for p in self.phases:
            error=None
            if p.full_future is not None and p.full_future.done():
                error=repr(p.full_future.exception())
            evidence.append({'generation':p.generation,'started':p._started,'closed':p._closed,
                'original_deadline_ns':p.deadline_ns,'native_return_ns':p.native_return_ns,
                'prefix_snapshot_ns':p.meta.snapshot_ns,'prefix_received_mask':p.meta.received_mask,
                'native_terminal_ns':p.meta.final_native_ns,'partial_scope':p.meta.scope,
                'original_full_future_done':p.full_future.done() if p.full_future is not None else False,
                'full_publication_fence':p._full_publication.is_set(),'exception':error,
                'observed_mutable_raw_records_hex':[candidate._bytes(r).hex() for r in p.full_records],
                'observed_mutable_stats_hex':candidate._bytes(p.full_stats).hex(),
                'published_prefix_hex':[r.hex() for r in p._published_prefix.record_images] if p._published_prefix else None,
                'published_full_hex':[r.hex() for r in p._full_result.record_images] if p._full_result else None})
        out=BASE/'raw-tests';out.mkdir(exist_ok=True)
        (out/(self._testMethodName+'.json')).write_text(json.dumps({
            'schema':'PRIVATE.seven-request-causal-fixture.v1','hardware':False,
            'qualified_for_output':False,'phases':evidence,'peer_events':self.peer_events,
            'python_switch_interval_s':sys.getswitchinterval(),
            'source_sha256':self.lib._private_split_source_sha256,
            'binary_sha256':self.lib._private_split_binary_sha256},indent=2)+'\n')
        if self.peer_errors:raise self.peer_errors[0]

    def peer_loop(self,*,io=None,delay_ns=2_700_000,delays=None,missing=(),faults=None,modes=None,
                  values=None,fragment=None,total=7):
        io=self.peer.fileno() if io is None else io
        ready=threading.Event();delays=delays or {};faults=faults or {};modes=modes or {};values=values or {}
        def run():
            pending=[];parser=ATParser();count=0;ready.set()
            try:
                while not self.stop_peer.is_set():
                    clock=time.monotonic_ns()
                    for due,data,index in tuple(pending):
                        if due<=clock:
                            os.write(io,data);pending.remove((due,data,index))
                            self.peer_events.append(('reply',index,time.monotonic_ns(),len(data)))
                    if count>=total and not pending:return
                    timeout=min(.005,max(0,(min((x[0] for x in pending),default=clock+5_000_000)-clock)/1e9))
                    if select.select([io],[],[],timeout)[0]:
                        data=os.read(io,1024)
                        if not data:return
                        for f in parser.feed(data):
                            index=count%7;count+=1
                            wire=f.raw if hasattr(f,'raw') else frame(f.can_id,f.data)
                            # Frame API has can_id, confirmed below by framing.
                            self.peer_events.append(('request',index,time.monotonic_ns(),wire.hex()))
                            if index in missing:continue
                            raw=reply(wire,fault=faults.get(index,0),mode=modes.get(index,0),value=values.get(index,40.))
                            if fragment and index==fragment[0]:raw=raw[:fragment[1]]
                            pending.append((time.monotonic_ns()+delays.get(index,delay_ns),raw,index))
            except (BrokenPipeError,ConnectionResetError,OSError) as error:
                if not self.stop_peer.is_set():self.peer_errors.append(error)
            except BaseException as error:self.peer_errors.append(error)
        t=threading.Thread(target=run,daemon=True);self.peer_threads.append(t);t.start()
        self.assertTrue(ready.wait(.1));return t

    def start(self,p):return p.start(deadline_ns=time.monotonic_ns()+20_000_000)

    def test_delayed_voltage_publishes_stable_six_before_full_join(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000})
        feedback,full=self.start(p)
        prefix=p.wait_feedback(feedback,generation=1)
        self.assertFalse(full.done());self.assertTrue(p.session.busy.locked())
        self.assertFalse(prefix.native_owner_joined);self.assertEqual(prefix.scope,'PARTIAL_COMBINED_SEVEN_REQUESTS')
        saved=prefix.record_images
        with self.assertRaises(RuntimeError):self.session.exchange([stop_wire(1)])
        with self.assertRaises(RuntimeError):self.session.emergency_stop()
        fullvalue=p.join()
        self.assertEqual(saved,fullvalue.record_images[:6]);self.assertLess(prefix.snapshot_ns,fullvalue.native_return_ns)
        self.assertEqual(fullvalue.voltage,40.);self.assertEqual(fullvalue.scope,'FULL_COMBINED_SEVEN_REQUESTS')
        rows=fullvalue.decoded_records();self.assertEqual(len(rows),7)
        for left,right in zip(rows,rows[1:]):self.assertGreaterEqual(right.start_ns-left.finish_ns,900_000)
        with self.assertRaises(FrozenInstanceError):prefix.generation=2
        altered=prefix.decoded_records();altered[0].rx[7]^=1
        self.assertEqual(prefix.record_images,saved)
        self.assertEqual(prefix.decoded_stats().writes,7)
        p.close();self.assertFalse(self.session.busy.locked())

    def test_voltage_before_sixth_feedback_cannot_publish_false_prefix(self):
        p=self.phase();self.peer_loop(delays={5:6_000_000,6:500_000})
        f,v=self.start(p)
        prefix=p.wait_feedback(f,generation=1);full=p.join()
        records=full.decoded_records()
        self.assertLess(records[6].received_ns,records[5].received_ns)
        self.assertGreaterEqual(prefix.snapshot_ns,records[5].received_ns)
        self.assertEqual(prefix.received_mask,127)

    def test_missing_feedback_never_publishes_and_retains_seven_raw(self):
        p=self.phase();self.peer_loop(missing={5})
        f,v=self.start(p)
        with self.assertRaises(Exception):p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertTrue(v.done());self.assertEqual(len(v.exception().records),7)
        self.assertEqual(v.exception().records[5].received,0)
        self.assertEqual(p.meta.scope,0);self.assertTrue(self.session.poisoned)

    def test_fault_in_feedback_never_publishes_healthy_prefix(self):
        p=self.phase();self.peer_loop(faults={5:1})
        f,v=self.start(p)
        with self.assertRaises(Exception):p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertFalse(p.meta.scope);self.assertEqual(v.exception().records[5].received,17)
        self.assertTrue(self.session.poisoned)

    def test_missing_voltage_prefix_success_does_not_certify_full(self):
        p=self.phase();self.peer_loop(missing={6})
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1)
        self.assertEqual(len(prefix.record_images),6)
        with self.assertRaises(Exception):p.join()
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertEqual(v.exception().records[6].received,0)
        p.close();self.assertFalse(self.session.busy.locked())

    def test_late_voltage_rejected_at_original_absolute_deadline(self):
        p=self.phase();self.peer_loop(delays={6:22_000_000})
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1)
        self.assertEqual(len(prefix.record_images),6)
        with self.assertRaises(Exception):p.join()
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertEqual(v.exception().records[6].received,0)
        self.assertEqual(v.exception().records[6].deadline_ns,p.deadline_ns)

    def test_fault_voltage_preserves_prefix_and_all7_failure_raw(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000},faults={6:1})
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):p.join()
        with self.assertRaises(Exception):v.result(timeout=.1)
        error=v.exception();self.assertEqual(prefix.record_images,tuple(candidate._bytes(r) for r in error.records[:6]))
        self.assertGreaterEqual(error.stats.rejected_total,17)

    def test_out_of_range_voltage_original35_42_guard_preserved(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000},values={6:34.9})
        f,v=self.start(p);p.wait_feedback(f,generation=1)
        with self.assertRaisesRegex(ValueError,'35..42'):p.join()
        self.assertEqual(v.exception().records[6].received,17)

    def test_cancel_after_prefix_wins_and_owner_join_required(self):
        p=self.phase();self.peer_loop(delays={6:8_000_000})
        f,v=self.start(p);p.wait_feedback(f,generation=1)
        os.write(self.cancel_write,b'x')
        with self.assertRaisesRegex(RuntimeError,'Cancelled'):p.join()
        with self.assertRaises(Exception):v.result(timeout=.1)
        p.close();self.assertFalse(self.session.busy.locked())

    def test_original_future_and_generation_identity_are_required(self):
        p=self.phase();self.peer_loop();f,v=self.start(p)
        with self.assertRaisesRegex(ValueError,'original Future'):p.take_feedback(Future(),generation=1)
        with self.assertRaisesRegex(ValueError,'generation'):p.take_feedback(f,generation=2)
        p.wait_feedback(f,generation=1);p.join()
        with self.assertRaises(ValueError):p.start(deadline_ns=time.monotonic_ns()+20_000_000)

    def test_cpp_generation_mismatch_before_any_writes(self):
        p=self.phase();error=C.create_string_buffer(256)
        status=self.lib.sda_split_take_prefix(p._handle,2,p.prefix_records,C.byref(p.prefix_stats),
                                              C.byref(p.meta),error,256)
        self.assertEqual(status,-1);self.assertIn(b'generation',error.value)
        self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_native_destroy_refuses_inflight_owner(self):
        p=self.phase();self.peer_loop(delays={6:8_000_000});f,v=self.start(p)
        p.wait_feedback(f,generation=1);error=C.create_string_buffer(256)
        self.assertEqual(self.lib.sda_split_destroy(p._handle,error,256),-1)
        self.assertIn(b'Join',error.value);p.join()

    def test_fd_reuse_rejected_and_unrelated_descriptor_not_closed(self):
        p=self.phase();old=p._fds[0];os.close(old)
        newr,neww=os.pipe();os.set_blocking(newr,False)
        if newr!=old:os.dup2(newr,old);os.close(newr)
        try:
            with self.assertRaisesRegex(RuntimeError,'binding'):p.start(deadline_ns=time.monotonic_ns()+20_000_000)
            with self.assertRaisesRegex(RuntimeError,'Reused'):p.close()
            self.assertGreaterEqual(os.fstat(old).st_ino,0)
        finally:os.close(old);os.close(neww)

    def test_noncanonical_voltage_fails_before_write(self):
        p=self.phase();bad=bytearray(p.wires[6]);bad[7]=0x1b
        p._wire_array=(C.c_ubyte*119).from_buffer_copy(b''.join(p.wires[:6])+bytes(bad))
        f,v=self.start(p)
        with self.assertRaisesRegex(active.ExchangeError,'voltage'):v.result(timeout=.1)
        self.assertFalse(select.select([self.peer],[],[],0)[0]);self.assertEqual(p.full_stats.writes,0)

    def test_native_prefix_source_is_immutable_after_full_completion(self):
        p=self.phase();self.peer_loop(delays={6:5_000_000});f,v=self.start(p)
        prefix=p.wait_feedback(f,generation=1);full=p.join()
        p.full_records[0].rx[7]^=1
        rows=(Record*6)();stats=active.Stats();meta=candidate.SplitMeta();error=C.create_string_buffer(256)
        self.assertEqual(self.lib.sda_split_take_prefix(p._handle,1,rows,C.byref(stats),C.byref(meta),error,256),1)
        self.assertEqual(tuple(candidate._bytes(r) for r in rows),prefix.record_images)
        self.assertEqual(prefix.record_images,full.record_images[:6])

    def test_two_buses_have_independent_original_prefix_and_voltage_futures(self):
        h,p=socket.socketpair();h.setblocking(False);p.setblocking(False)
        try:
            rear=self.make_session(h.fileno(),7);first=self.phase();second=self.phase(rear,generation=2,voltage_id=12)
            self.peer_loop(delays={6:6_000_000});self.peer_loop(io=p.fileno(),delays={6:6_000_000})
            deadline=time.monotonic_ns()+20_000_000
            f,v=first.start(deadline_ns=deadline);g,w=second.start(deadline_ns=deadline)
            a=first.wait_feedback(f,generation=1);b=second.wait_feedback(g,generation=2)
            self.assertIsNot(f,g);self.assertIsNot(v,w);self.assertEqual(len(a.record_images)+len(b.record_images),12)
            first.join();second.join();first.close();second.close()
        finally:h.close();p.close()

    def test_genuine_pty_same_raw_protocol_and_owner_join(self):
        master,slave=pty.openpty();tty.setraw(slave);os.set_blocking(slave,False);os.set_blocking(master,False)
        try:
            s=self.make_session(slave);p=self.phase(s,generation=3)
            self.peer_loop(io=master,delays={6:6_000_000});f,v=self.start(p)
            prefix=p.wait_feedback(f,generation=3);full=p.join()
            self.assertEqual(prefix.record_images,full.record_images[:6]);p.close();s.close();self.sessions.remove(s)
        finally:os.close(master);os.close(slave)


    def test_external_feedback_future_success_has_no_native_proof(self):
        p=self.phase();self.peer_loop(delays={6:5_000_000});f,v=self.start(p)
        f.set_result('forged success')
        with self.assertRaisesRegex(RuntimeError,'externally completed'):p.take_feedback(f,generation=1)
        p.join()

    def test_feedback_callback_baseexception_is_not_hidden_by_successful_voltage(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000});f,v=self.start(p)
        def callback(future):raise KeyboardInterrupt('synthetic callback failure')
        f.add_done_callback(callback)
        with self.assertRaisesRegex(KeyboardInterrupt,'synthetic'):p.wait_feedback(f,generation=1)
        with self.assertRaisesRegex(KeyboardInterrupt,'synthetic'):p.join()
        self.assertTrue(v.done());p.close()

    def test_callback_notification_failure_seen_by_original_final_join(self):
        from unittest.mock import patch
        p=self.phase();self.peer_loop(delays={6:6_000_000});f,v=self.start(p)
        p.wait_feedback(f,generation=1)
        original=os.write
        def fail(fd,data):
            if fd==p._fds[1] and data==b'\x02':raise OSError('synthetic final hint failure')
            return original(fd,data)
        with patch.object(candidate.os,'write',side_effect=fail):
            with self.assertRaisesRegex(OSError,'final hint failure'):p.join()
        self.assertTrue(p._full_publication.is_set());p.close()

    def test_cross_thread_getter_and_destroy_rejected_by_same_owner_contract(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000});f,v=self.start(p)
        errors=[]
        def wrong_owner():
            for op in (lambda:p.take_feedback(f,generation=1),p.close):
                try:op()
                except BaseException as error:errors.append(str(error))
        t=threading.Thread(target=wrong_owner);t.start();t.join(.1)
        self.assertEqual(len(errors),2);self.assertTrue(all('main owner' in e for e in errors))
        p.wait_feedback(f,generation=1);p.join();p.close()

    def test_cancel_ready_at_entry_has_priority_over_native_prefix(self):
        p=self.phase();os.write(self.cancel_write,b'x');f,v=self.start(p)
        with self.assertRaisesRegex(RuntimeError,'Cancelled'):p.take_feedback(f,generation=1)
        with self.assertRaisesRegex(active.ExchangeError,'Cancelled'):v.result(timeout=.1)
        self.assertEqual(p.full_stats.writes,0)

    def test_reply_wrong_mode_cannot_become_ready(self):
        p=self.phase();self.peer_loop(modes={5:2});f,v=self.start(p)
        with self.assertRaises(Exception):p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertFalse(p.meta.scope);self.assertGreaterEqual(p.full_stats.rejected_total,17)

    def test_count_extent_invalid_rejected_without_touching_one_record_output(self):
        p=self.phase();one=(Record*1)();one[0].start_ns=123;stats=active.Stats();error=C.create_string_buffer(256)
        status=self.lib.sda_split_exchange(p._handle,p._wire_array,1,time.monotonic_ns()+20_000_000,
                                           one,C.byref(stats),error,256)
        self.assertEqual(status,-1);self.assertEqual(one[0].start_ns,123)
        self.assertIn(b'count7',error.value);self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_partial_seventh_frame_is_retained_and_never_voltage_success(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000},fragment=(6,11));f,v=self.start(p)
        prefix=p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):p.join()
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertEqual(prefix.received_mask,63);self.assertEqual(p.full_records[6].received,0)
        self.assertEqual(p.full_stats.rejected_total,11)

    def test_no_native_split_without_canonical_gap_and_window(self):
        h,q=socket.socketpair();h.setblocking(False)
        try:
            ids=range(1,7)
            s=active.ActiveSession(self.lib,h.fileno(),first_id=1,cancel_fd=self.cancel_read,
                boot_fd=self.boot.fileno(),boot_id=BOOT,gap_ns=890_000,window=3,
                raw_lower_by_id={i:-1. for i in ids},raw_upper_by_id={i:1. for i in ids},
                kp_max_by_id={i:3. for i in ids},kd_max_by_id={i:.15 for i in ids})
            try:
                with self.assertRaisesRegex(RuntimeError,'gap900/window3'):self.phase(s)
                self.assertFalse(s.busy.locked());self.assertFalse(select.select([q],[],[],0)[0])
            finally:s.close()
        finally:h.close();q.close()


    def test_blocked_final_hint_publication_never_extends_original_deadline(self):
        from unittest.mock import patch
        p=self.phase();self.peer_loop(delays={6:5_000_000});f,v=self.start(p)
        entered=threading.Event();release=threading.Event();original=os.write
        def block(fd,data):
            if fd==p._fds[1] and data==b'\x02':
                entered.set();release.wait(.1)
            return original(fd,data)
        with patch.object(candidate.os,'write',side_effect=block):
            try:
                p.wait_feedback(f,generation=1)
                self.assertTrue(entered.wait(.03))
                with self.assertRaisesRegex(TimeoutError,'publication fence'):p.join()
                self.assertFalse(p._full_publication.is_set())
                self.assertTrue(self.session.busy.locked())
            finally:release.set()
        p.close();self.assertTrue(p._full_publication.is_set())


    def test_wrong_cancellation_descriptor_rejected_before_write_and_releases_owner(self):
        r,w=os.pipe()
        try:
            with self.assertRaisesRegex(RuntimeError,'differs from original owner'):
                candidate.SevenRequestPhase(self.session,self.pool,cancel_fd=r,generation=4,voltage_id=1)
            self.assertFalse(self.session.busy.locked());self.assertFalse(select.select([self.peer],[],[],0)[0])
        finally:os.close(r);os.close(w)
        p=self.phase();self.peer_loop();f,v=self.start(p);p.wait_feedback(f,generation=1);p.join()

    def test_invalid_cancellation_fd_does_not_leave_session_reserved(self):
        r,w=os.pipe();os.close(r);os.close(w)
        with self.assertRaises(OSError):
            candidate.SevenRequestPhase(self.session,self.pool,cancel_fd=r,generation=5,voltage_id=1)
        self.assertFalse(self.session.busy.locked());self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_writer_cancellation_endpoint_rejected_before_native_setup(self):
        with self.assertRaisesRegex(ValueError,'Readable'):
            candidate.SevenRequestPhase(self.session,self.pool,cancel_fd=self.cancel_write,generation=6,voltage_id=1)
        self.assertFalse(self.session.busy.locked());self.assertFalse(select.select([self.peer],[],[],0)[0])


    def test_constructor_close_failure_preserves_primary_and_releases_busy(self):
        from unittest.mock import patch
        r,w=os.pipe();allocated=[];realpipe=os.pipe;realclose=os.close
        def make_pipe():
            value=realpipe();allocated.extend(value);return value
        def fail_one(fd):
            if allocated and fd==allocated[0]:raise OSError('synthetic cleanup close failure')
            realclose(fd)
        try:
            with patch.object(candidate.os,'pipe',side_effect=make_pipe),patch.object(candidate.os,'close',side_effect=fail_one):
                with self.assertRaisesRegex(RuntimeError,'differs from original owner') as caught:
                    candidate.SevenRequestPhase(self.session,self.pool,cancel_fd=r,generation=7,voltage_id=1)
            self.assertFalse(self.session.busy.locked())
            self.assertTrue(any('cleanup close failure' in note for note in caught.exception.__notes__))
            self.assertFalse(select.select([self.peer],[],[],0)[0])
        finally:
            for fd in (*allocated,r,w):
                try:realclose(fd)
                except OSError:pass

    def test_constructor_reused_pipe_is_not_closed_during_rollback(self):
        from unittest.mock import patch
        r,w=os.pipe();allocated=[];replacement=[];realpipe=os.pipe;realblocking=os.set_blocking
        def make_pipe():
            value=realpipe();allocated.extend(value);return value
        def replace(fd,blocking):
            if fd==allocated[1]:
                old=allocated[0];os.close(old);newr,neww=realpipe();replacement.extend((newr,neww))
                if newr!=old:os.dup2(newr,old);os.close(newr);replacement[0]=old
                raise OSError('synthetic setup failure after FD reuse')
            realblocking(fd,blocking)
        try:
            with patch.object(candidate.os,'pipe',side_effect=make_pipe),patch.object(candidate.os,'set_blocking',side_effect=replace):
                with self.assertRaisesRegex(OSError,'setup failure') as caught:
                    candidate.SevenRequestPhase(self.session,self.pool,cancel_fd=r,generation=8,voltage_id=1)
            self.assertFalse(self.session.busy.locked());self.assertGreaterEqual(os.fstat(replacement[0]).st_ino,0)
            self.assertTrue(any('unrelated descriptor retained' in note for note in caught.exception.__notes__))
        finally:
            for fd in set((*allocated,*replacement,r,w)):
                try:os.close(fd)
                except OSError:pass

if __name__=='__main__':unittest.main(verbosity=2)
