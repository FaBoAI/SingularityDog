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
import live_seven_request_candidate as candidate

BASE=Path(__file__).parent
BOOT='11111111-2222-3333-4444-555555555555'

def frame(canid,data):return b'AT'+((canid<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
def reply(wire,*,fault=0,mode=0,value=40.):
    f=ATParser().feed(wire)[0];mid=f.destination
    if f.kind in (1,4):return frame((2<<24)|(mode<<22)|(fault<<16)|(mid<<8)|0xfd,struct.pack('>4H',32767,32767,32767,250))
    return frame((17<<24)|(fault<<16)|(mid<<8)|0xfd,f.data[:4]+struct.pack('<f',value))


class LiveTests(unittest.TestCase):
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
        # Genuine frozen STOP ABI2 is a separate source/binary, not a mock.
        spec=importlib.util.spec_from_file_location('build_stop_baseline',BASE/'stop-abi2-baseline/build.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        cls.stop_only_library=mod.build()

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

    def make_session(self,fd,first=1,kp_cap=3.,kd_cap=.15):
        ids=range(first,first+6)
        s=active.ActiveSession(self.lib,fd,first_id=first,cancel_fd=self.cancel_read,
            boot_fd=self.boot.fileno(),boot_id=BOOT,gap_ns=900_000,window=3,
            raw_lower_by_id={i:-1. for i in ids},raw_upper_by_id={i:1. for i in ids},
            kp_max_by_id={i:kp_cap for i in ids},kd_max_by_id={i:kd_cap for i in ids})
        self.sessions.append(s);return s

    def phase(self,session=None,generation=1,voltage_id=None):
        session=session or self.session
        p=candidate.LiveSevenRequestPhase(session,self.pool,cancel_fd=self.cancel_read,
            generation=generation,voltage_id=session.first_id if voltage_id is None else voltage_id,
            held_wires=tuple(active.encode_motion(i,0.,3.,.15) for i in range(session.first_id,session.first_id+6)),
            raw_target_bounds_by_id={i:(-.01,.01) for i in range(session.first_id,session.first_id+6)})
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
        out=BASE/'raw-tests-live';out.mkdir(exist_ok=True)
        (out/(self._testMethodName+'.json')).write_text(json.dumps({
            'schema':'PRIVATE.seven-request-causal-fixture.v1','hardware':False,
            'qualified_for_output':False,'phases':evidence,'peer_events':self.peer_events,
            'python_switch_interval_s':sys.getswitchinterval(),
            'source_sha256':self.lib._private_live_split_source_sha256,
            'binary_sha256':self.lib._private_live_split_binary_sha256},indent=2)+'\n')
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
                            raw=reply(wire,fault=faults.get(index,0),mode=modes.get(index,2 if f.kind==1 else 0),value=values.get(index,40.))
                            if fragment and index==fragment[0]:raw=raw[:fragment[1]]
                            pending.append((time.monotonic_ns()+delays.get(index,delay_ns),raw,index))
            except (BrokenPipeError,ConnectionResetError,OSError) as error:
                if not self.stop_peer.is_set():self.peer_errors.append(error)
            except BaseException as error:self.peer_errors.append(error)
        t=threading.Thread(target=run,daemon=True);self.peer_threads.append(t);t.start()
        self.assertTrue(ready.wait(.1));return t

    def start(self,p):return p.start(deadline_ns=time.monotonic_ns()+20_000_000)

    def custom_phase(self,*,held=None,bounds=None,session=None):
        session=session or self.session
        ids=tuple(range(session.first_id,session.first_id+6))
        p=candidate.LiveSevenRequestPhase(session,self.pool,cancel_fd=self.cancel_read,
            generation=1,voltage_id=session.first_id,
            held_wires=held or tuple(active.encode_motion(i,0.,3.,.15) for i in ids),
            raw_target_bounds_by_id=bounds or {i:(-.01,.01) for i in ids})
        self.phases.append(p);return p

    def test_live_delayed_voltage_prefix_is_immutable_and_not_idle(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000})
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1)
        self.assertFalse(v.done());self.assertFalse(prefix.native_owner_joined)
        self.assertEqual(prefix.scope,'PARTIAL_COMBINED_LIVE_HELD_TYPE1_SEVEN_REQUESTS')
        with self.assertRaises(RuntimeError):self.session.exchange([active.encode_motion(1,.001,3.,.15)])
        with self.assertRaises(RuntimeError):self.session.emergency_stop()
        original=prefix.record_images;decoded=prefix.decoded_records();decoded[0].rx[7]^=1
        self.assertEqual(prefix.record_images,original)
        full=p.join();self.assertEqual(full.record_images[:6],original)
        self.assertEqual(full.scope,'FULL_COMBINED_LIVE_HELD_TYPE1_SEVEN_REQUESTS')
        self.assertEqual(full.voltage,40.)
        p.close();self.assertFalse(self.session.busy.locked())

    def test_next_new_type1_is_only_possible_after_full_join_and_close(self):
        p=self.phase();self.peer_loop(delay_ns=1_000_000,total=13)
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1);full=p.join()
        before_close=time.monotonic_ns();p.close();closed=time.monotonic_ns()
        self.assertLessEqual(full.native_return_ns,before_close)
        self.assertFalse(self.session.busy.locked())
        next_wires=tuple(active.encode_motion(i,.001,3.,.15) for i in range(1,7))
        records,stats=self.session.exchange(next_wires,deadline_ns=time.monotonic_ns()+20_000_000)
        self.assertGreaterEqual(min(r.start_ns for r in records),closed)
        self.assertEqual(tuple(bytes(r.tx) for r in records),next_wires)
        self.assertTrue(all(r.received==17 for r in records))

    def test_seventh_before_sixth_keeps_exact_prefix_scope(self):
        p=self.phase();self.peer_loop(delays={5:6_000_000,6:500_000})
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1);full=p.join()
        rows=full.decoded_records()
        self.assertLess(rows[6].received_ns,rows[5].received_ns)
        self.assertGreaterEqual(prefix.snapshot_ns,rows[5].received_ns)
        self.assertEqual(prefix.received_mask,127)

    def test_mode_zero_feedback_never_publishes_live_prefix(self):
        p=self.phase();self.peer_loop(modes={5:0})
        f,v=self.start(p)
        with self.assertRaises(Exception):p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertIsNone(p._published_prefix);self.assertTrue(self.session.poisoned)
        self.assertEqual(len(v.exception().records),7)

    def test_faulty_feedback_never_publishes_live_prefix(self):
        p=self.phase();self.peer_loop(faults={5:1})
        f,v=self.start(p)
        with self.assertRaises(Exception):p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertIsNone(p._published_prefix);self.assertTrue(self.session.poisoned)
        self.assertGreaterEqual(v.exception().stats.rejected_total,17)

    def test_partial_feedback_is_retained_as_failure(self):
        p=self.phase();self.peer_loop(fragment=(5,11))
        f,v=self.start(p)
        with self.assertRaises(Exception):p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):v.result(timeout=.1)
        self.assertIsNone(p._published_prefix)
        self.assertGreaterEqual(v.exception().stats.rejected_total,11)
        self.assertEqual(v.exception().records[5].received,0)

    def test_missing_seventh_does_not_certify_voltage_or_full_join(self):
        p=self.phase();self.peer_loop(missing=(6,))
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):p.join()
        self.assertFalse(prefix.native_owner_joined)
        self.assertIsNone(p._full_result)
        self.assertEqual(v.exception().records[6].received,0)

    def test_faulty_seventh_after_good_prefix_fails_full(self):
        p=self.phase();self.peer_loop(faults={6:1},delays={6:6_000_000})
        f,v=self.start(p);prefix=p.wait_feedback(f,generation=1)
        with self.assertRaises(Exception):p.join()
        self.assertIsNotNone(prefix);self.assertIsNone(p._full_result)
        self.assertTrue(self.session.poisoned)

    def test_voltage_outside_original_bounds_after_prefix_fails(self):
        p=self.phase();self.peer_loop(values={6:34.9},delays={6:6_000_000})
        f,v=self.start(p);p.wait_feedback(f,generation=1)
        with self.assertRaisesRegex(Exception,'35..42'):p.join()
        self.assertIsNone(p._full_result)

    def test_cancel_after_prefix_wins_over_ready_hint(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000})
        f,v=self.start(p);p.wait_feedback(f,generation=1)
        os.write(self.cancel_write,b'x')
        with self.assertRaisesRegex(Exception,'Cancel'):p.take_feedback(f,generation=1)
        with self.assertRaises(Exception):p.join()

    def test_mixed_stop_wire_rejected_before_any_write(self):
        held=list(active.encode_motion(i,0.,3.,.15) for i in range(1,7));held[2]=stop_wire(3)
        with self.assertRaisesRegex(Exception,'framing/order'):self.custom_phase(held=tuple(held))
        self.assertFalse(self.session.busy.locked());self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_off_current_raw_target_bounds_rejected_before_any_write(self):
        held=tuple(active.encode_motion(i,.02,3.,.15) for i in range(1,7))
        with self.assertRaisesRegex(Exception,'target/gain'):self.custom_phase(held=held)
        self.assertFalse(self.session.busy.locked());self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_gain_above_bounded_caps_rejected_before_any_write(self):
        held=tuple(active.encode_motion(i,0.,3.1,.15) for i in range(1,7))
        # A wider ordinary Session cap must not silently enlarge LIVE3/.15.
        a,b=socket.socketpair();a.setblocking(False);b.setblocking(False)
        try:
            wider=self.make_session(a.fileno(),kp_cap=10.,kd_cap=1.)
            with self.assertRaisesRegex(Exception,'bounded caps'):self.custom_phase(held=held,session=wider)
            self.assertFalse(wider.busy.locked());self.assertFalse(select.select([b],[],[],0)[0])
        finally:
            wider.close();self.sessions.remove(wider);a.close();b.close()
        self.assertFalse(self.session.busy.locked());self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_nonfinite_or_wide_bounds_rejected_before_reservation(self):
        for bad in ((float('nan'),.01),(-.02,.02)):
            with self.assertRaises(ValueError):self.custom_phase(bounds={i:bad for i in range(1,7)})
            self.assertFalse(self.session.busy.locked())
        self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_changed_held_byte_before_start_is_rejected_by_native_binding(self):
        p=self.phase();p._wire_array[8]^=1
        f,v=self.start(p)
        with self.assertRaisesRegex(Exception,'exact bounded previous'):v.result(timeout=.1)
        self.assertEqual(sum(r.written for r in v.exception().records),0)
        self.assertIsNone(p._published_prefix);self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_cancelled_entry_never_writes_held_type1(self):
        p=self.phase();os.write(self.cancel_write,b'x')
        with self.assertRaisesRegex(Exception,'Cancel'):self.start(p)
        self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_next_generation_rejected_until_current_seventh_joined_and_closed(self):
        p=self.phase();self.peer_loop(delays={6:6_000_000})
        f,v=self.start(p);p.wait_feedback(f,generation=1)
        with self.assertRaisesRegex(RuntimeError,'busy|borrowed'):self.phase(generation=2)
        p.join();p.close()
        newer=self.phase(generation=2)
        self.assertEqual(newer.generation,2);newer.close()

    def test_foreign_future_result_is_not_prefix_proof(self):
        p=self.phase();p.feedback_future.set_result(object())
        with self.assertRaisesRegex(RuntimeError,'externally'):self.start(p)
        self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_duplicate_bus_motor_wire_rejected_before_any_write(self):
        held=tuple(active.encode_motion(1,0.,3.,.15) for _ in range(6))
        with self.assertRaisesRegex(Exception,'framing/order'):self.custom_phase(held=held)
        self.assertFalse(select.select([self.peer],[],[],0)[0])

    def test_old_stop_abi2_library_has_no_live_capability(self):
        old=self.stop_only_library
        with self.assertRaisesRegex(ValueError,'complete optional'):candidate.load_candidate(old)

if __name__=='__main__':unittest.main(verbosity=2)
