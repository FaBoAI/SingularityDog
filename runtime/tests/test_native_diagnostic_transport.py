"""Actual C++ I/O against local sockets/PTY, never robot devices."""
import importlib.util
import os
from pathlib import Path
import pty
import select
import socket
import struct
import threading
import time
import tty
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw.can_readonly import ATParser, read_request

ROOT = Path(__file__).resolve().parents[1]/'experiments/native_transport'

def reply(wire, *, fault=0, value=.25):
    f = ATParser().feed(wire)[0]
    mid = f.destination
    kind = 2 if f.kind == 4 else f.kind
    can_id = kind << 24 | fault << 16 | mid << 8 | (0xfe if kind == 0 else 0xfd)
    data = (bytes([mid])*8 if kind == 0 else struct.pack('>4H',32768,32768,32768,250)
            if kind == 2 else f.data[:4]+struct.pack('<f',value))
    return b'AT'+((can_id<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'


class NativeTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('build_native_transport',ROOT/'build.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        cls.lib=native.load_library(mod.build())

    def setUp(self):
        self.host,self.device=socket.socketpair()
        self.host.setblocking(False)
        self.cr,self.cw=os.pipe()
        self.session=native.NativeSession(self.lib,self.host.fileno(),first_id=1,cancel_fd=self.cr,stop_proxy=True)
        self.thread=None

    def tearDown(self):
        self.host.close();self.device.close();os.close(self.cr);os.close(self.cw)
        if self.thread: self.thread.join(timeout=1)

    def device_loop(self, count, mutate=lambda w: reply(w), fragment=False):
        def run():
            parser=ATParser();seen=0
            try:
                while seen<count:
                    if not select.select([self.device],[],[],.5)[0]: return
                    chunk=self.device.recv(4096)
                    if not chunk:return
                    for f in parser.feed(chunk):
                        seen+=1;out=mutate(f.wire)
                        if out:
                            if fragment:
                                self.device.sendall(out[:5]);time.sleep(.0002);self.device.sendall(out[5:])
                            else:self.device.sendall(out)
            except (OSError,ValueError): pass
        self.thread=threading.Thread(target=run);self.thread.start()

    def test_identity_and_type17_exact_reply_mapping(self):
        wires=[read_request(i) for i in range(1,7)]
        self.device_loop(6,fragment=True)
        rows,stats=self.session.exchange(wires)
        self.assertEqual([r['result']['mcu_uid_hex'] for r in native.records_as_events(rows)],
                         [(bytes([i])*8).hex() for i in range(1,7)])
        self.thread.join();self.device_loop(12)
        wires=[read_request(i,p) for p in ('position','velocity') for i in range(1,7)]
        rows,stats=self.session.exchange(wires)
        events=native.records_as_events(rows)
        self.assertEqual(len(events),12)
        self.assertTrue(all(e['result']['value']==.25 for e in events))
        self.assertEqual(stats.writes,12)
        self.assertTrue(all(b.start_ns-a.finish_ns>=600000 for a,b in zip(rows,rows[1:])))

    def test_stop_composite_and_exact_raw_evidence(self):
        self.device_loop(6)
        records,stats=self.session.exchange([native.stop_wire(i) for i in range(1,7)])
        rows=native.records_as_events(records)
        self.assertTrue(all(r['parameter']=='stop_feedback' for r in rows))
        self.assertTrue(all(r['result']['mode_state']==0 for r in rows))
        self.assertEqual(native.exchange_evidence(records,stats)['rejected_hex'],'')
        self.assertTrue(all(r.start_ns<=r.finish_ns<=r.received_ns<r.deadline_ns for r in records))

    def test_stop_proxy_allows_one_voltage_read_but_no_other_new_parameter(self):
        wires=[native.stop_wire(i) for i in range(1,7)]+[read_request(1,'voltage')]
        self.device_loop(7)
        records,stats=self.session.exchange(wires)
        self.assertEqual(stats.writes,7)
        self.assertEqual([row['parameter'] for row in native.records_as_events(records)],
                         ['stop_feedback']*6+['voltage'])
        self.assertEqual(native.records_as_events(records)[-1]['result']['value'],.25)
        for parameter in ('can_timeout','current','run_mode','zero_state'):
            with self.subTest(parameter=parameter):
                session=native.NativeSession(self.lib,self.host.fileno(),first_id=1,
                                             cancel_fd=self.cr,stop_proxy=True)
                with self.assertRaisesRegex(native.ExchangeError,'Disallowed'):
                    session.exchange([read_request(1,parameter)])
                self.assertFalse(select.select([self.device],[],[],0)[0])
        session=native.NativeSession(self.lib,self.host.fileno(),first_id=1,cancel_fd=self.cr)
        with self.assertRaisesRegex(native.ExchangeError,'Disallowed'):
            session.exchange([read_request(1,'voltage')])
        self.assertFalse(select.select([self.device],[],[],0)[0])

    def test_no_motion_configuration_enable_or_batch_writes(self):
        base=read_request(1)
        for kind in (1,3,6,18,24):
            wire=base[:2]+((((kind<<24)|(0xfd<<8)|1)<<3)|4).to_bytes(4,'big')+base[6:]
            session=native.NativeSession(self.lib,self.host.fileno(),first_id=1,cancel_fd=self.cr,stop_proxy=True)
            with self.assertRaisesRegex(native.ExchangeError,'Disallowed'):
                session.exchange([wire])
            self.assertFalse(select.select([self.device],[],[],0)[0])

    def test_stop_requires_explicit_mode(self):
        session=native.NativeSession(self.lib,self.host.fileno(),first_id=1,cancel_fd=self.cr)
        with self.assertRaisesRegex(native.ExchangeError,'Disallowed'):
            session.exchange([native.stop_wire(1)])

    def test_timeout_poison_and_partial_capture(self):
        self.device_loop(1,mutate=lambda w:reply(w)[:8])
        with self.assertRaisesRegex(native.ExchangeError,'[Dd]eadline') as raised:
            self.session.exchange([read_request(1)],timeout_ns=10_000_000)
        self.assertEqual(raised.exception.stats.rejected_size,8)
        with self.assertRaisesRegex(RuntimeError,'poisoned'):
            self.session.exchange([read_request(1)])

    def test_missing_middle_reply_after_all_writes_is_response_deadline(self):
        wires=[read_request(i,'position') for i in range(1,7)];seen=[]
        def drop_id2(wire):
            seen.append(wire)
            return b'' if ATParser().feed(wire)[0].destination==2 else reply(wire)
        self.device_loop(6,mutate=drop_id2)
        with self.assertRaisesRegex(native.ExchangeError,
                r'Response deadline exceeded after all writes \(5/6 replies\); missing motor IDs: 2; no retry') as raised:
            self.session.exchange(wires,timeout_ns=40_000_000)
        error=raised.exception
        self.thread.join(timeout=1)
        self.assertEqual(seen,wires)
        self.assertEqual(error.stats.writes,6)
        self.assertEqual(error.stats.bytes,5*17)
        self.assertEqual([r.written for r in error.records],[17]*6)
        self.assertEqual([r.received for r in error.records],[17,0,17,17,17,17])
        self.assertEqual(bytes(error.records[1].tx),wires[1])
        self.assertEqual(bytes(error.records[1].rx),bytes(17))
        self.assertGreaterEqual(error.stats.end_ns,error.records[1].deadline_ns)
        self.assertTrue(all(r.finish_ns<r.deadline_ns for r in error.records))
        with self.assertRaisesRegex(RuntimeError,'poisoned; no retry'):
            self.session.exchange(wires)
        self.assertEqual(seen,wires)

    def test_response_window_stall_reports_remaining_writes_unsent(self):
        wires=[read_request(i,'position') for i in range(1,7)];seen=[]
        def no_reply(wire):
            seen.append(wire);return b''
        self.device_loop(3,mutate=no_reply)
        with self.assertRaisesRegex(native.ExchangeError,
                r'Diagnostic deadline exceeded before all writes \(3/6 written, 0/6 replies\); no retry') as raised:
            self.session.exchange(wires,timeout_ns=20_000_000)
        error=raised.exception
        self.thread.join(timeout=1)
        self.assertEqual(seen,wires[:3])
        self.assertEqual(error.stats.writes,3)
        self.assertEqual([r.written for r in error.records],[17]*3+[0]*3)
        self.assertEqual([r.received for r in error.records],[0]*6)
        self.assertGreaterEqual(error.stats.end_ns,error.records[0].deadline_ns)

    def test_fault_and_nonfinite_rejected(self):
        self.device_loop(1,mutate=lambda w:reply(w,fault=1))
        with self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
            self.session.exchange([native.stop_wire(1)])
        self.thread.join();self.device_loop(1,mutate=lambda w:reply(w,value=float('nan')))
        s=native.NativeSession(self.lib,self.host.fileno(),first_id=1,cancel_fd=self.cr)
        with self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
            s.exchange([read_request(1,'position')])

    def test_pending_window_and_out_of_order_replies(self):
        pending=[]
        def out(w):
            pending.append(w)
            if len(pending)==3:
                result=b''.join(reply(x) for x in reversed(pending));pending.clear();return result
            return b''
        self.device_loop(6,mutate=out)
        records,_=self.session.exchange([read_request(i,'position') for i in range(1,7)])
        self.assertTrue(all(r.received==17 for r in records))

    def test_identity_only_batch_waits_for_each_reply_without_changing_session_window(self):
        pipelined=[]
        def delayed_reply(wire):
            if select.select([self.device],[],[],.003)[0]:
                pipelined.append(wire)
            return reply(wire)
        self.device_loop(6,mutate=delayed_reply)
        records,stats=self.session.exchange([read_request(i) for i in range(1,7)])
        self.assertEqual(pipelined,[])
        self.assertEqual(stats.writes,6)
        self.assertTrue(all(b.start_ns>=a.received_ns for a,b in zip(records,records[1:])))
        self.assertEqual(self.session.window,3)

    def test_missing_identity_reply_blocks_later_ids_without_retry(self):
        wires=[read_request(i) for i in range(1,7)];seen=[]
        def drop_id2(wire):
            seen.append(wire)
            return reply(wire) if ATParser().feed(wire)[0].destination==1 else b''
        self.device_loop(2,mutate=drop_id2)
        with self.assertRaisesRegex(native.ExchangeError,
                r'Diagnostic deadline exceeded before all writes \(2/6 written, 1/6 replies\); no retry') as raised:
            self.session.exchange(wires,timeout_ns=20_000_000)
        error=raised.exception
        self.thread.join(timeout=1)
        self.assertEqual(seen,wires[:2])
        self.assertEqual(error.stats.writes,2)
        self.assertEqual([r.written for r in error.records],[17,17,0,0,0,0])
        self.assertEqual([r.received for r in error.records],[17,0,0,0,0,0])
        with self.assertRaisesRegex(RuntimeError,'poisoned; no retry'):
            self.session.exchange(wires)

    def test_duplicate_reply_rejected(self):
        self.device_loop(1,mutate=lambda w:reply(w)*2)
        with self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
            self.session.exchange([read_request(1)])

    def test_backlog_not_flushed_or_accepted(self):
        self.device.sendall(reply(read_request(1)))
        with self.assertRaisesRegex(native.ExchangeError,'backlog') as raised:
            self.session.exchange([read_request(1)])
        self.assertEqual(raised.exception.stats.writes,0)
        self.assertEqual(raised.exception.stats.rejected_size,17)

    def test_cancel_before_write(self):
        os.write(self.cw,b'x')
        with self.assertRaisesRegex(native.ExchangeError,'Cancelled') as raised:
            self.session.exchange([read_request(1)])
        self.assertEqual(raised.exception.stats.writes,0)

    def test_cross_bus_id_duplicate_key_rejected(self):
        for wires in ([read_request(7)], [read_request(1)]*2):
            s=native.NativeSession(self.lib,self.host.fileno(),first_id=1,cancel_fd=self.cr)
            with self.assertRaises(native.ExchangeError):s.exchange(wires)

    def test_real_pty_nonblocking_boundary(self):
        master,slave=pty.openpty();tty.setraw(slave);os.set_blocking(slave,False)
        def run():
            if select.select([master],[],[],.5)[0]:
                wire=os.read(master,17);os.write(master,reply(wire))
        worker=threading.Thread(target=run);worker.start()
        try:
            s=native.NativeSession(self.lib,slave,first_id=1,cancel_fd=self.cr)
            records,_=s.exchange([read_request(1)])
            self.assertEqual(records[0].received,17)
        finally:
            worker.join(timeout=1);os.close(master);os.close(slave)

class NativePreparationTests(unittest.TestCase):
    """Python publication boundary, without loading C++ or opening serial ports."""
    def setUp(self):
        self.host,self.peer=socket.socketpair()
        self.host.setblocking(False)
        self.library=SimpleNamespace(sd_exchange=Mock(return_value=0))
        self.session=native.NativeSession(self.library,self.host.fileno(),
            first_id=1,cancel_fd=-1,stop_proxy=True)

    def tearDown(self):
        self.host.close();self.peer.close()

    def test_hook_runs_once_after_fd_checks_and_buffers_before_native(self):
        events=[]
        fstat,get_blocking,create_buffer=os.fstat,os.get_blocking,native.C.create_string_buffer
        def trace(name,fn):
            def call(*args):
                events.append(name)
                return fn(*args)
            return call
        hook=Mock(side_effect=lambda:events.append('publish'))
        self.library.sd_exchange.side_effect=lambda *args:(events.append('native') or 0)
        with patch.object(native.os,'fstat',trace('fstat',fstat)), \
                patch.object(native.os,'get_blocking',trace('blocking',get_blocking)), \
                patch.object(native.C,'create_string_buffer',trace('buffer',create_buffer)):
            self.session.exchange([native.stop_wire(1)],before_native=hook)
        self.assertEqual(events,['fstat','blocking','buffer','publish','native'])
        hook.assert_called_once_with()
        self.assertEqual(bytes(self.library.sd_exchange.call_args.args[4]),native.stop_wire(1))
        self.assertFalse(self.session.poisoned)
        self.assertFalse(self.session.busy.locked())

    def test_fd_or_batch_rejection_never_publishes_or_enters_native(self):
        for failure in ('blocking','binding','batch','hook'):
            with self.subTest(failure=failure):
                self.session.poisoned=False
                hook=Mock()
                st=os.fstat(self.host.fileno())
                bad_st=SimpleNamespace(st_dev=st.st_dev,st_ino=st.st_ino+1,st_rdev=st.st_rdev)
                with patch.object(native.os,'get_blocking',return_value=failure=='blocking'), \
                        patch.object(native.os,'fstat',return_value=bad_st if failure=='binding' else st):
                    with self.assertRaises(ValueError):
                        self.session.exchange([] if failure=='batch' else [native.stop_wire(1)],
                            before_native=0 if failure=='hook' else hook)
                hook.assert_not_called()
                self.library.sd_exchange.assert_not_called()
                self.assertTrue(self.session.poisoned)
                self.assertFalse(self.session.busy.locked())

    def test_hook_exception_poisons_session_without_native_call(self):
        hook=Mock(side_effect=RuntimeError('publication failure'))
        with self.assertRaisesRegex(RuntimeError,'publication failure'):
            self.session.exchange([native.stop_wire(1)],before_native=hook)
        hook.assert_called_once_with()
        self.library.sd_exchange.assert_not_called()
        self.assertTrue(self.session.poisoned)
        self.assertFalse(self.session.busy.locked())

    def test_deadline_is_fixed_before_hook_and_not_rebased_after_it(self):
        now=[1_000_000]
        def delayed_publish():now[0]+=2_000_000
        with patch.object(native.time,'monotonic_ns',side_effect=lambda:now[0]) as clock:
            self.session.exchange([native.stop_wire(1)],timeout_ns=1_000_000,
                                  before_native=delayed_publish)
        self.assertEqual(self.library.sd_exchange.call_args.args[10],2_000_000)
        self.assertLess(self.library.sd_exchange.call_args.args[10],now[0])
        clock.assert_called_once_with()


if __name__=='__main__':unittest.main()
