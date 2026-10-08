"""Actual C++ transport, synthetic RS05 replies over local sockets/PTY only."""
import ctypes as C
import importlib.util
import os
from pathlib import Path
import pty
import select
import socket
import struct
import tempfile
import threading
import time
import tty
import unittest
import hashlib
import json
import subprocess
from unittest.mock import patch

from singularitydog_hw import native_active_transport as native
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.native_diagnostic_transport import stop_wire
from singularitydog_hw.rs05_trial_protocol import (TrialPhase, enable_request,
                                                  watchdog_setup_request)

ROOT = Path(__file__).resolve().parents[1]/'experiments/native_active_transport'
BOOT = '11111111-2222-3333-4444-555555555555'


def frame(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'


def query(mid, index):
    return frame((17 << 24)|(0xfd << 8)|mid, struct.pack('<H',index)+bytes(6))


def enable(mid):
    return enable_request(phase=TrialPhase.ENABLE, motor_id=mid)


def watchdog(mid):
    return watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid)


def reply(wire, *, mode=None, fault=0, value=.25, host=None, source=None):
    f = ATParser().feed(wire)[0]
    mid = f.destination if source is None else source
    if f.kind == 0:
        return frame(mid << 8 | (0xfe if host is None else host), bytes([mid])*8)
    if f.kind in (1,3,4,18):
        state = (0 if f.kind in (4,18) else 2) if mode is None else mode
        data = struct.pack('>4H',32767,32767,32767,250)
        return frame((2 << 24)|(state << 22)|(fault << 16)|(mid << 8)|(0xfd if host is None else host), data)
    index = int.from_bytes(f.data[:2],'little')
    data = (struct.pack('<I',4000) if index == 0x7028 else bytes(4) if index == 0x7005
            else struct.pack('<f',value))
    return frame((17 << 24)|(fault << 16)|(mid << 8)|(0xfd if host is None else host), f.data[:4]+data)


def version_reply(wire, *, mode=0, fault=0, host=0xfd, source=None, prefix=b'\x00\xc4\x56'):
    request=ATParser().feed(wire)[0]
    mid=request.destination if source is None else source
    return frame((2<<24)|(mode<<22)|(fault<<16)|(mid<<8)|host,
                 prefix+bytes.fromhex('05001300')+b'\xa5')


class NativeActiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('build_active_native',ROOT/'build.py')
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        cls.lib = native.load_library(mod.build())

    def setUp(self):
        self.host,self.peer = socket.socketpair(); self.host.setblocking(False)
        self.cancel_read,self.cancel_write = os.pipe()
        self.boot = tempfile.TemporaryFile(); self.boot.write((BOOT+'\n').encode()); self.boot.flush()
        self.sessions=[]; self.thread=None; self.seen=[]; self.peer_error=None
        self.session = self.make_session()

    def make_session(self, fd=None, **overrides):
        args = dict(first_id=1,cancel_fd=self.cancel_read,boot_fd=self.boot.fileno(),boot_id=BOOT,
            raw_lower_by_id={i:-1. for i in range(1,7)},raw_upper_by_id={i:1. for i in range(1,7)},
            kp_max_by_id={i:12. for i in range(1,7)},kd_max_by_id={i:.25 for i in range(1,7)})
        args.update(overrides)
        s=native.ActiveSession(self.lib,self.host.fileno() if fd is None else fd,**args)
        self.sessions.append(s); return s

    def tearDown(self):
        for s in self.sessions:s.close()
        self.host.close();self.peer.close()
        if self.thread:self.thread.join(timeout=1)
        os.close(self.cancel_read);os.close(self.cancel_write);self.boot.close()
        if self.peer_error:raise self.peer_error

    def device_loop(self,count,mutate=reply,fragment=False):
        if self.thread:
            self.thread.join(timeout=1)
            self.assertFalse(self.thread.is_alive(),'Previous synthetic peer still owns the socket')
        ready=threading.Event()
        self.peer_ready_ns=None;self.peer_events=[]
        def run():
            parser=ATParser();seen=0
            # Creating a Python Thread does not prove that its peer parser is
            # running. Exclude that fixture startup from a product deadline;
            # all request wakeups and injected reply delays remain measured.
            self.peer_ready_ns=time.monotonic_ns();ready.set()
            try:
                while seen<count:
                    if not select.select([self.peer],[],[],.7)[0]:return
                    raw=self.peer.recv(4096);read_ns=time.monotonic_ns()
                    if not raw:return
                    for f in parser.feed(raw):
                        event={'motor_id':f.destination,'request_kind':f.kind,
                               'read_ns':read_ns,'mutate_begin_ns':time.monotonic_ns()}
                        self.peer_events.append(event)
                        self.seen.append(f);seen+=1;output=mutate(f.wire)
                        event['mutate_end_ns']=time.monotonic_ns()
                        if output:
                            event['send_begin_ns']=time.monotonic_ns()
                            if fragment:
                                self.peer.sendall(output[:5]);time.sleep(.0001);self.peer.sendall(output[5:])
                            else:self.peer.sendall(output)
                            event['send_end_ns']=time.monotonic_ns()
            except (OSError,ValueError):pass
            except BaseException as error:self.peer_error=error
        self.thread=threading.Thread(target=run);self.thread.start()
        self.assertTrue(ready.wait(timeout=1),'Synthetic peer did not reach its parser before the test')

    def no_write(self):
        self.assertFalse(select.select([self.peer],[],[],0)[0])

    def fresh(self):
        self.session.close();self.session=self.make_session();return self.session

    def test_absolute_deadline_is_not_rebased_after_caller_delay(self):
        deadline_ns=time.monotonic_ns()+70_000_000
        time.sleep(.01)  # Worker preparation/queue delay consumes the same budget.
        self.device_loop(1)
        records,_=self.session.exchange([read_request(1)],deadline_ns=deadline_ns)
        self.assertEqual(records[0].deadline_ns,deadline_ns)
        self.assertLess(records[0].received_ns,deadline_ns)

    def test_receive_gap_hypothesis_late_reply_has_no_second_motion_write(self):
        # Offline socket experiment only. Live runtime still supplies 20 ms.
        # One already-written transaction remains outstanding through a 20 ms
        # soft boundary; receiving it at 25 ms is not a new command's ACK.
        begin=time.monotonic_ns()
        def delayed(wire):
            time.sleep(.025)
            return reply(wire)
        self.device_loop(1,delayed)
        records,_=self.session.exchange([native.encode_motion(1,0.,0.,0.)],
                                       deadline_ns=begin+40_000_000)
        self.assertGreater(records[0].received_ns-begin,20_000_000)
        self.assertLess(records[0].received_ns-begin,40_000_000)
        self.thread.join(timeout=1)
        self.assertEqual(len(self.seen),1)
        self.no_write()

    def test_receive_gap_hypothesis_delayed_crlf_completes_original_frame(self):
        def partial(wire):
            value=reply(wire)
            self.peer.sendall(value[:15])
            time.sleep(.025)
            return value[15:]
        self.device_loop(1,partial)
        begin=time.monotonic_ns()
        records,stats=self.session.exchange([native.encode_motion(1,0.,0.,0.)],
                                           deadline_ns=begin+40_000_000)
        self.assertEqual(records[0].received,17)
        self.assertEqual(stats.bytes,17)
        self.assertGreaterEqual(stats.reads,2)
        self.assertGreater(records[0].received_ns-begin,20_000_000)
        self.thread.join(timeout=1)
        self.assertEqual(len(self.seen),1)
        self.no_write()

    def test_receive_gap_hypothesis_hard_miss_still_poisoned_no_retry(self):
        self.device_loop(1,lambda _:b'')
        with self.assertRaises(native.ExchangeError):
            self.session.exchange([native.encode_motion(1,0.,0.,0.)],timeout_ns=40_000_000)
        self.thread.join(timeout=1)
        with self.assertRaisesRegex(RuntimeError,'poisoned'):
            self.session.exchange([native.encode_motion(1,0.,0.,0.)])
        self.assertEqual(len(self.seen),1)
        self.no_write()

    def test_expired_absolute_deadline_never_writes(self):
        with self.assertRaisesRegex(ValueError,'Expired or invalid absolute'):
            self.session.exchange([read_request(1)],deadline_ns=time.monotonic_ns()-1)
        self.no_write()
        self.assertTrue(self.session.poisoned)

    def test_identity_all_allowed_reads_and_integer_semantics(self):
        self.device_loop(6,fragment=True)
        records,stats=self.session.exchange([read_request(i) for i in range(1,7)])
        self.assertEqual(stats.writes,6)
        self.assertEqual([native.decode_record(r)['uid_hex'] for r in records],[(bytes([i])*8).hex() for i in range(1,7)])
        for index in (0x7019,0x701b,0x701c,0x7028,0x7005):
            self.device_loop(6)
            records,_=self.session.exchange([query(i,index) for i in range(1,7)])
            self.assertEqual([native.decode_record(r)['value'] for r in records],
                             [4000 if index==0x7028 else 0 if index==0x7005 else .25]*6)

    def test_enable_transition_motion_stop_and_wire_gap(self):
        for request,mode in ((enable,0),(enable,2),(lambda i:native.encode_motion(i,.5,12,.25),2),(stop_wire,0)):
            self.device_loop(6,lambda w:reply(w,mode=mode))
            records,_=self.session.exchange([request(i) for i in range(1,7)])
            self.assertEqual([native.decode_record(r)['mode_state'] for r in records],[mode]*6)
            self.assertTrue(all(b.start_ns-a.finish_ns>=600000 for a,b in zip(records,records[1:])))

    def test_custom_session_gap_applies_to_normal_and_emergency_stop(self):
        self.session.close();self.session=self.make_session(gap_ns=5_000_000,window=2)
        self.device_loop(6)
        records,_=self.session.exchange([stop_wire(i) for i in range(1,7)])
        self.assertTrue(all(b.start_ns-a.finish_ns>=5_000_000 for a,b in zip(records,records[1:])))
        self.thread.join(timeout=1)
        self.device_loop(6)
        result=self.session.emergency_stop()
        self.assertTrue(result['complete'])
        self.assertEqual(result['attempted_ids'],list(range(1,7)))
        stop_records=result['evidence']['records']
        self.assertGreaterEqual(stop_records[0]['start_ns']-records[-1].finish_ns,5_000_000)
        self.assertTrue(all(b['start_ns']-a['finish_ns']>=5_000_000
                            for a,b in zip(stop_records,stop_records[1:])))

    def test_missing_third_reply_and_partial_sixth_remain_unconfirmed_after_stop(self):
        # Reproduce the live front-bus trace: six complete Type1 writes,
        # ID3 absent, ID6 with only 15 bytes before the absolute deadline.
        self.session.close()
        self.session=self.make_session(gap_ns=800_000,window=3)
        partial=[]
        def incomplete(wire):
            mid=ATParser().feed(wire)[0].destination
            if mid==3:return b''
            value=reply(wire)
            if mid==6:
                partial.append(value[:15]);return value[:15]
            return value
        self.device_loop(6,incomplete)
        with self.assertRaises(native.ExchangeError) as failed:
            self.session.exchange([native.encode_motion(i,0.,0.,0.) for i in range(1,7)],
                                  timeout_ns=30_000_000)
        records,stats=failed.exception.records,failed.exception.stats
        self.assertEqual([r.written for r in records],[17]*6)
        self.assertEqual([i+1 for i,r in enumerate(records) if not r.received],[3,6])
        self.assertEqual(bytes(stats.rejected[:stats.rejected_size]),partial[0])
        self.assertNotIn('before write',str(failed.exception))
        # Even six later mode0 replies cannot prove which request caused
        # the two outstanding Type2 replies. Preserve the cutoff request.
        self.thread.join(timeout=1)
        self.device_loop(6,lambda wire:reply(wire,mode=0))
        stopped=self.session.emergency_stop()
        self.assertFalse(stopped['complete'])
        self.assertEqual(stopped['unconfirmed_ids'],[3,6])
        self.assertEqual(stopped['ambiguous_ids'],[3,6])
        self.assertEqual(len(stopped['replies']),6)

    def test_watchdog_requires_stopped_ack_before_separate_readback(self):
        self.device_loop(6)
        records,stats=self.session.exchange([watchdog(i) for i in range(1,7)])
        self.assertEqual(stats.writes,6)
        self.assertTrue(all(r.written==r.received==17 for r in records))
        self.assertEqual([native.decode_record(r)['mode_state'] for r in records],[0]*6)
        self.assertTrue(all(b.start_ns>a.received_ns for a,b in zip(records,records[1:])))
        self.thread.join();self.assertEqual(len(self.seen),6)
        self.assertTrue(all(f.kind==18 and f.data==bytes.fromhex('28700000a00f0000') for f in self.seen))
        self.device_loop(6)
        readbacks,_=self.session.exchange([query(i,0x7028) for i in range(1,7)])
        self.assertGreater(min(r.start_ns for r in readbacks),max(r.received_ns for r in records))
        self.assertEqual([native.decode_record(r)['value'] for r in readbacks],[4000]*6)

    def test_send_only_watchdog_is_rejected_before_write(self):
        with self.assertRaises(native.ExchangeError):self.session.send_only([watchdog(1)])
        self.no_write()

    def test_watchdog_ack_split_crlf_waits_before_next_write_on_socket_and_pty(self):
        for use_pty in (False,True):
            with self.subTest(pty=use_pty):
                if use_pty:
                    peer,host=pty.openpty();tty.setraw(host);os.set_blocking(host,False)
                    session=self.make_session(host)
                else:
                    peer,host=self.peer.fileno(),self.host.fileno();session=self.session
                errors=[];seen=[]
                def device():
                    parser=ATParser()
                    try:
                        while len(seen)<4:
                            self.assertTrue(select.select([peer],[],[],.5)[0])
                            for request in parser.feed(os.read(peer,4096)):
                                seen.append(request)
                                answer=reply(request.wire)
                                os.write(peer,answer[:16]) # CR received, final LF withheld.
                                self.assertFalse(select.select([peer],[],[],.002)[0],
                                                 'Next request preceded complete ACK')
                                os.write(peer,answer[16:])
                    except BaseException as error:errors.append(error)
                worker=threading.Thread(target=device);worker.start()
                try:
                    setup,_=session.exchange([watchdog(1),watchdog(2)])
                    reads,_=session.exchange([query(1,0x7028)])
                    more,_=session.exchange([query(2,0x7028)])
                    self.assertTrue(all(r.received==17 for r in (*setup,*reads,*more)))
                    self.assertGreater(setup[1].start_ns,setup[0].received_ns)
                    self.assertGreater(reads[0].start_ns,setup[-1].received_ns)
                finally:
                    worker.join(timeout=1)
                    if use_pty:session.close();os.close(host);os.close(peer)
                if errors:raise errors[0]
                self.assertEqual([r.kind for r in seen],[18,18,17,17])

    def test_watchdog_ack_scope_and_pending_ack_stop_ambiguity(self):
        bad_replies=[]
        for changes in ({'mode':2},{'mode':1},{'fault':1},{'host':0xfe},{'source':7}):
            bad_replies.append(lambda w,changes=changes:reply(w,**changes))
        bad_replies.extend((version_reply,lambda w:reply(query(1,0x7028))))
        for mutation in bad_replies:
            self.device_loop(1,mutation)
            with self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
                self.fresh().exchange([watchdog(1)])
        self.device_loop(1,lambda w:reply(w)[:16])
        with self.assertRaisesRegex(native.ExchangeError,'deadline') as caught:
            self.fresh().exchange([watchdog(1),watchdog(2)],timeout_ns=8_000_000)
        self.assertEqual(caught.exception.stats.writes,1)
        self.assertEqual(caught.exception.stats.rejected_size,16)
        self.device_loop(6)
        result=self.session.emergency_stop()
        self.assertEqual(result['ambiguous_ids'],[1])
        self.assertEqual(result['confirmed_ids'],list(range(2,7)))
        self.assertFalse(result['complete'])

    def test_unsolicited_partial_ack_never_crosses_serialized_write_boundary(self):
        for use_pty in (False,True):
            for second in (watchdog(2),query(2,0x7028)):
                with self.subTest(pty=use_pty,next_kind=ATParser().feed(second)[0].kind):
                    if use_pty:
                        peer,host=pty.openpty();tty.setraw(host);os.set_blocking(host,False)
                        session=self.make_session(host)
                    else:
                        peer,host=self.peer.fileno(),self.host.fileno();session=self.fresh()
                    errors=[];seen=[]
                    def device():
                        try:
                            self.assertTrue(select.select([peer],[],[],.5)[0])
                            first=os.read(peer,17);seen.append(first)
                            unsolicited=reply(second)
                            os.write(peer,reply(first)+unsolicited[:16])
                            if select.select([peer],[],[],.03)[0]:
                                seen.append(os.read(peer,17))
                                os.write(peer,unsolicited[16:])
                        except BaseException as error:errors.append(error)
                    worker=threading.Thread(target=device);worker.start()
                    try:
                        with self.assertRaisesRegex(native.ExchangeError,'Trailing partial') as caught:
                            session.exchange([watchdog(1),second])
                        self.assertEqual(caught.exception.stats.writes,1)
                        self.assertEqual(caught.exception.stats.rejected_size,16)
                        self.assertEqual(bytes(caught.exception.stats.rejected[:16]),reply(second)[:16])
                    finally:
                        worker.join(timeout=1)
                        if use_pty:session.close();os.close(host);os.close(peer)
                    if errors:raise errors[0]
                    self.assertEqual(seen,[watchdog(1)])

    def test_exact_version_request_and_reply_preserve_raw_fingerprint(self):
        self.device_loop(6,version_reply,fragment=True)
        records,stats=self.session.exchange([native.version_request(i) for i in range(1,7)])
        self.assertEqual(stats.writes,6)
        for record in records:
            decoded=native.decode_record(record)
            self.assertEqual(decoded['version_bytes_hex'],'05001300')
            self.assertEqual(decoded['version_bytes'],[5,0,19,0])
            self.assertEqual(decoded['unspecified_byte7'],165)
            self.assertIsNone(decoded['semantic_firmware_version'])
            self.assertNotIn('position_rad_candidate',decoded)
            self.assertEqual(decoded['request_started_monotonic_ns'],record.start_ns)

    def test_version_response_cannot_replace_normal_feedback_or_stop(self):
        for request in (stop_wire(1),enable(1),native.encode_motion(1,.2,1,.1)):
            self.device_loop(1,version_reply)
            with self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
                self.fresh().exchange([request])
        self.device_loop(1,reply)
        with self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
            self.fresh().exchange([native.version_request(1)])

    def test_wrong_version_mode_fault_source_host_prefix_are_rejected(self):
        for change in ({'mode':2},{'fault':1},{'source':7},{'host':0xfe},{'prefix':b'\x00\xc4\x55'}):
            self.device_loop(1,lambda wire:version_reply(wire,**change))
            with self.subTest(change=change),self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
                self.fresh().exchange([native.version_request(1)])

    def test_version_payload_scope_and_combined_type2_keys_remain_restricted(self):
        good=ATParser().feed(native.version_request(1))[0]
        bad=[frame(good.can_id,good.data[:2]+bytes(5)+b'\x01'),
             frame((3<<24)|(0xfd<<8)|1,good.data),
             frame((4<<24)|(0xfe<<8)|1,good.data)]
        for request in bad:
            with self.assertRaises(native.ExchangeError):self.fresh().exchange([request])
            self.no_write()
        for requests in ([native.version_request(1),stop_wire(1)],
                         [native.version_request(1),native.version_request(1)]):
            with self.assertRaisesRegex(native.ExchangeError,'Duplicate'):self.fresh().exchange(requests)
            self.no_write()
        with self.assertRaises(native.ExchangeError):self.fresh().send_only([native.version_request(1)])
        self.no_write()

    def test_version_timeout_never_retries_and_emergency_never_uses_version_as_ack(self):
        self.device_loop(1,lambda w:b'')
        with self.assertRaisesRegex(native.ExchangeError,'deadline'):
            self.session.exchange([native.version_request(1)],timeout_ns=5_000_000)
        self.thread.join();self.assertEqual(len(self.seen),1)
        self.device_loop(6,lambda wire:version_reply(wire) if ATParser().feed(wire)[0].destination==1 else reply(wire))
        result=self.session.emergency_stop(timeout_ns=90_000_000)
        self.assertNotIn(1,result['confirmed_ids'])
        self.assertEqual(result['confirmed_ids'],list(range(2,7)))
        self.assertFalse(result['complete'])

    def test_cxx_rejects_motion_outside_explicit_caps_before_any_write(self):
        wires=[native.encode_motion(1,1.2,12,.25),native.encode_motion(1,.5,13,.25),
               native.encode_motion(1,.5,12,.3)]
        canonical=native.encode_motion(1,.5,12,.25)
        f=ATParser().feed(canonical)[0]
        wires += [frame(f.can_id+(1<<8),f.data),frame(f.can_id,f.data[:2]+b'\x80\x00'+f.data[4:])]
        for wire in wires:
            with self.assertRaisesRegex(native.ExchangeError,'out-of-bounds') as caught:
                self.fresh().exchange([enable(2),wire])
            self.assertEqual(caught.exception.stats.writes,0);self.no_write()

    def test_zero_gain_native_caps_reject_positive_gain_before_any_write(self):
        for kp,kd in ((.1,0.),(0.,.01)):
            self.session.close()
            self.session=self.make_session(kp_max_by_id={i:0. for i in range(1,7)},
                                           kd_max_by_id={i:0. for i in range(1,7)})
            with self.subTest(kp=kp,kd=kd),self.assertRaises(native.ExchangeError) as caught:
                self.session.exchange([enable(2),native.encode_motion(1,.5,kp,kd)])
            self.assertEqual(caught.exception.stats.writes,0);self.no_write()

    def test_quantized_lower_boundary_is_not_silently_clipped(self):
        with self.assertRaises(native.ExchangeError):self.session.exchange([native.encode_motion(1,-1.,0.,0.)])
        self.no_write()

    def test_disallowed_configuration_and_noncanonical_commands(self):
        bad=[frame((k<<24)|(0xfd<<8)|1,bytes(8)) for k in (6,7,12,14,18,24)]
        bad += [query(1,0x700a),frame((4<<24)|(0xfd<<8)|1,b'\x01'+bytes(7)),
                frame((3<<24)|(0xfd<<8)|1,b'\x01'+bytes(7)),native.encode_motion(7,.2,1,.1)]
        for wire in bad:
            with self.assertRaises(native.ExchangeError):self.fresh().exchange([wire])
            self.no_write()
        for wire in (enable(1),stop_wire(1),frame((18<<24)|(0xfd<<8)|1,struct.pack('<H2xI',0x7028,4001))):
            with self.assertRaises(native.ExchangeError):self.fresh().send_only([wire])
            self.no_write()

    def test_watchdog_exact_payload_host_and_shared_type2_keys_only(self):
        good=ATParser().feed(watchdog(1))[0]
        mutations=[frame(good.can_id^(1<<8),good.data)]
        for index in range(8):
            payload=bytearray(good.data);payload[index]^=1
            mutations.append(frame(good.can_id,bytes(payload)))
        for request in mutations:
            with self.assertRaises(native.ExchangeError):self.fresh().exchange([request])
            self.no_write()
        for request in (enable(1),stop_wire(1),native.encode_motion(1,.1,1,.1),watchdog(1)):
            with self.assertRaisesRegex(native.ExchangeError,'Duplicate'):
                self.fresh().exchange([watchdog(1),request])
            self.no_write()

    def test_ambiguous_type2_request_keys_rejected(self):
        for wires in ([enable(1),native.encode_motion(1,.5,1,.1)], [stop_wire(1),enable(1)], [query(1,0x7019)]*2):
            with self.assertRaisesRegex(native.ExchangeError,'Duplicate'):self.fresh().exchange(wires)
            self.no_write()

    def test_wrong_reply_mode_fault_host_and_source_poison(self):
        cases=[dict(mode=0),dict(mode=1),dict(mode=3),dict(fault=1),dict(host=0xfe),dict(source=7)]
        for changes in cases:
            self.device_loop(1,lambda w:reply(w,**changes))
            with self.assertRaisesRegex(native.ExchangeError,'Unmatched'):
                self.fresh().exchange([native.encode_motion(1,.5,1,.1)])
            with self.assertRaisesRegex(RuntimeError,'poisoned'):self.session.exchange([enable(1)])
        self.device_loop(1,lambda w:reply(w,value=float('nan')))
        with self.assertRaises(native.ExchangeError):self.fresh().exchange([query(1,0x7019)])

    def test_faulted_stop_reports_fault_and_keeps_poison(self):
        self.device_loop(1,lambda w:reply(w,fault=17))
        records,_=self.session.exchange([stop_wire(1)])
        self.assertEqual(native.decode_record(records[0])['fault_bits'],17)
        self.assertTrue(self.session.poisoned)
        with self.assertRaisesRegex(RuntimeError,'poisoned'):self.session.exchange([enable(1)])

    def test_timeout_partial_bytes_retained_no_active_retry(self):
        self.device_loop(1,lambda w:reply(w)[:8])
        with self.assertRaisesRegex(native.ExchangeError,'deadline') as caught:
            self.session.exchange([native.encode_motion(1,.1,1,.1)],timeout_ns=8_000_000)
        self.assertEqual(caught.exception.stats.rejected_size,8)
        self.assertEqual(caught.exception.stats.writes,1)
        with self.assertRaises(RuntimeError):self.session.exchange([enable(2)])
        self.no_write()

    def test_duplicate_extra_partial_and_backlog_rejected(self):
        for suffix in ('duplicate','partial'):
            self.device_loop(1,lambda w:reply(w)+(reply(w) if suffix=='duplicate' else b'ATx'))
            with self.assertRaises(native.ExchangeError):self.fresh().exchange([read_request(1)])
        self.fresh();self.peer.sendall(reply(read_request(1)))
        with self.assertRaisesRegex(native.ExchangeError,'backlog') as caught:self.session.exchange([enable(1)])
        self.assertEqual(caught.exception.stats.writes,0)
        self.assertEqual(caught.exception.stats.rejected_size,17)

    def test_cancel_between_writes_and_emergency_ignores_cancel_and_boot(self):
        def cancel(w):
            os.write(self.cancel_write,b'x');return reply(w)
        self.device_loop(1,cancel)
        with self.assertRaisesRegex(native.ExchangeError,'Cancelled') as caught:
            self.session.exchange([native.encode_motion(i,.2,1,.1) for i in range(1,7)])
        self.assertEqual(caught.exception.stats.writes,1)
        self.boot.seek(0);self.boot.write(('0'*36+'\n').encode());self.boot.flush()
        self.device_loop(6)
        result=self.session.emergency_stop()
        self.assertFalse(result['complete'],result)
        self.assertEqual(result['ambiguous_ids'],[1])
        self.assertEqual(result['confirmed_ids'],list(range(2,7)))
        self.assertTrue(all(f.kind==4 for f in self.seen[1:]))
        self.assertTrue(self.session.poisoned)

    def test_cancel_before_write_zero_tx_and_emergency_stop_all_six(self):
        os.write(self.cancel_write,b'x')
        with self.assertRaisesRegex(native.ExchangeError,'Cancelled') as caught:self.session.exchange([enable(1)])
        self.assertEqual(caught.exception.stats.writes,0);self.no_write()
        self.device_loop(6)
        result=self.session.emergency_stop()
        self.assertEqual(result['attempted_ids'],list(range(1,7)))
        self.assertTrue(result['complete'])

    def test_emergency_continues_after_missing_first_reply_and_retains_junk(self):
        self.peer.sendall(b'old-partial')
        def sometimes(w):
            f=ATParser().feed(w)[0]
            return b'' if f.destination==1 else b'junk'+reply(w,fault=4 if f.destination==3 else 0)
        self.device_loop(6,sometimes)
        result=self.session.emergency_stop(timeout_ns=90_000_000)
        self.assertLessEqual(self.peer_ready_ns,result['evidence']['stats']['begin_ns'])
        self.assertEqual(result['attempted_ids'],list(range(1,7)))
        self.assertEqual(result['confirmed_ids'],list(range(2,7)),
                         {'stop':result,'peer_events':self.peer_events})
        self.assertEqual(result['fault_by_id']['3'],4)
        self.assertFalse(result['complete'])
        self.assertTrue(result['evidence']['rejected_hex'].startswith(b'old-partial'.hex()))
        self.assertLess(result['evidence']['stats']['end_ns']-result['evidence']['stats']['begin_ns'],150_000_000)

    def test_emergency_default_budget_handles_observed_26ms_reply_turnaround(self):
        def observed_delay(w):
            time.sleep(.028)
            return reply(w)
        self.device_loop(6,observed_delay)
        result=self.session.emergency_stop()
        self.assertLessEqual(self.peer_ready_ns,result['evidence']['stats']['begin_ns'])
        self.assertTrue(result['complete'],{'stop':result,'peer_events':self.peer_events})
        self.assertEqual(result['timeout_ns'],250_000_000)
        self.assertEqual(result['attempted_ids'],list(range(1,7)))
        self.assertEqual(result['confirmed_ids'],list(range(1,7)))
        rows=result['evidence']['records']
        self.assertTrue(all(r['finish_ns']<r['received_ns']<r['deadline_ns'] for r in rows))
        self.assertTrue(all(b['start_ns']>=a['received_ns'] for a,b in zip(rows,rows[1:])))
        self.assertEqual(rows[-1]['deadline_ns'],result['deadline_monotonic_ns'])
        self.assertGreater(rows[0]['deadline_ns']-rows[0]['start_ns'],35_000_000)

    def test_peer_startup_delay_precedes_the_unchanged_emergency_deadline(self):
        # Reproduce the scheduling cause deliberately: without the fixture
        # ready handshake, 35ms startup + the normal 28ms reply exceeded the
        # first unchanged 250ms/6 STOP slice. Start the peer before the budget,
        # preserving that 28ms delay and every per-axis confirmation assertion.
        original_thread=threading.Thread
        def delayed_thread(*args,**kwargs):
            target=kwargs['target']
            def delayed():time.sleep(.035);target()
            kwargs['target']=delayed
            return original_thread(*args,**kwargs)
        def delayed_reply(wire):
            time.sleep(.028)
            return reply(wire)
        with patch.object(threading,'Thread',side_effect=delayed_thread):
            self.device_loop(6,delayed_reply)
        result=self.session.emergency_stop()
        self.assertTrue(result['complete'],{'stop':result,'peer_events':self.peer_events})
        self.assertEqual(result['timeout_ns'],250_000_000)
        self.assertLessEqual(self.peer_ready_ns,result['evidence']['stats']['begin_ns'])
        self.assertEqual(result['confirmed_ids'],list(range(1,7)))

    def test_emergency_old_short_budget_does_not_reclassify_late_replies(self):
        def observed_delay(w):
            time.sleep(.028)
            return reply(w)
        self.device_loop(6,observed_delay)
        result=self.session.emergency_stop(timeout_ns=150_000_000)
        self.assertFalse(result['complete'])
        self.assertEqual(result['attempted_ids'],list(range(1,7)))
        self.assertNotIn(1,result['confirmed_ids'])
        self.assertNotIn(2,result['confirmed_ids'])
        self.assertIn(reply(stop_wire(1)).hex(),result['evidence']['rejected_hex'])
        self.assertEqual(result['evidence']['records'][0]['received'],0)
        self.assertEqual(result['timeout_ns'],150_000_000)

    def test_emergency_extended_budget_accepts_slow_replies_before_own_deadline(self):
        def slow_reply(w):
            time.sleep(.055)
            return reply(w)
        self.device_loop(6,slow_reply)
        result=self.session.emergency_stop(timeout_ns=500_000_000)
        self.assertTrue(result['complete'],result)
        self.assertEqual(result['confirmed_ids'],list(range(1,7)))
        self.assertTrue(all(f.kind==4 for f in self.seen))
        self.assertTrue(all(r['finish_ns']<r['received_ns']<r['deadline_ns']
                            for r in result['evidence']['records']))

    def test_emergency_default_still_attempts_all_axes_when_none_reply(self):
        self.device_loop(6,lambda w:b'')
        started=time.monotonic_ns()
        result=self.session.emergency_stop()
        self.assertFalse(result['complete'])
        self.assertEqual(result['attempted_ids'],list(range(1,7)))
        self.assertEqual(result['confirmed_ids'],[])
        self.assertEqual(result['unconfirmed_ids'],list(range(1,7)))
        self.assertLessEqual(result['deadline_monotonic_ns']-started,251_000_000)
        self.assertLess(time.monotonic_ns()-started,350_000_000)
        self.assertTrue(all(r['start_ns']<r['deadline_ns'] for r in result['evidence']['records']))

    def test_pending_enable_ack_remains_ambiguous_during_stop(self):
        self.device_loop(1,lambda w:b'')
        with self.assertRaises(native.ExchangeError):self.session.exchange([enable(1)],timeout_ns=5_000_000)
        self.device_loop(6)
        result=self.session.emergency_stop()
        self.assertEqual(result['ambiguous_ids'],[1])
        self.assertNotIn(1,result['confirmed_ids'])
        self.assertEqual(len(result['replies']),6)

    def test_repeated_stop_finishes_after_one_confirmed_round(self):
        self.device_loop(6)
        result=self.session.emergency_stop_repeated()
        self.assertTrue(result['complete'])
        self.assertEqual(len(result['attempts']),1)
        self.assertEqual(result['retry_policy']['attempts_completed'],1)
        self.assertTrue(all(f.kind==4 for f in self.seen))
        with self.assertRaises(RuntimeError):self.session.exchange([enable(1)])
        self.no_write()

    def test_repeated_stop_preserves_pending_motion_ambiguity_and_limits_rounds(self):
        self.device_loop(1,lambda w:b'')
        with self.assertRaises(native.ExchangeError):
            self.session.exchange([native.encode_motion(1,.1,0.,0.)],timeout_ns=5_000_000)
        self.device_loop(18)
        before=len(self.seen)
        result=self.session.emergency_stop_repeated()
        self.assertFalse(result['complete'])
        self.assertEqual(result['ambiguous_ids'],[1])
        self.assertEqual(result['unconfirmed_ids'],[1])
        self.assertEqual(len(result['attempts']),3)
        self.assertEqual(len(self.seen)-before,18)
        self.assertTrue(all(f.kind==4 for f in self.seen[before:]))
        self.assertTrue(all(a['ambiguous_ids']==[1] for a in result['attempts']))
        self.assertTrue(all(a['deadline_monotonic_ns']<=result['retry_policy']['deadline_ns']
                            for a in result['attempts']))

    def test_repeated_stop_cannot_restart_total_deadline_when_nothing_replies(self):
        self.device_loop(18,lambda w:b'')
        begin=time.monotonic_ns()
        result=self.session.emergency_stop_repeated()
        self.assertFalse(result['complete'])
        self.assertTrue(result['retry_policy']['budget_exhausted'])
        self.assertEqual(len(result['attempts']),3)
        self.assertEqual(result['attempts'][0]['timeout_ns'],250_000_000)
        self.assertEqual(result['attempts'][1]['timeout_ns'],500_000_000)
        self.assertLess(result['attempts'][2]['timeout_ns'],250_000_000)
        self.assertLess(time.monotonic_ns()-begin,1_200_000_000)
        self.assertEqual(result['unconfirmed_ids'],list(range(1,7)))
        for attempt in result['attempts']:
            self.assertEqual(attempt['attempted_ids'],list(range(1,7)))
            self.assertLessEqual(attempt['deadline_monotonic_ns'],result['retry_policy']['deadline_ns'])
            for row in attempt['evidence']['records']:
                self.assertLess(row['start_ns'],row['deadline_ns'])

    def test_repeated_stop_short_total_budget_still_bounds_all_rounds(self):
        self.device_loop(12,lambda w:b'')
        result=self.session.emergency_stop_repeated(total_timeout_ns=500_000_000)
        self.assertEqual(len(result['attempts']),2)
        self.assertTrue(result['retry_policy']['budget_exhausted'])
        self.assertTrue(all(a['deadline_monotonic_ns']<=result['retry_policy']['deadline_ns']
                            for a in result['attempts']))
        self.assertFalse(result['complete'])

    def test_repeated_stop_keeps_faults_from_earlier_attempt(self):
        first={'complete':False,'confirmed_ids':list(range(2,7)),'unconfirmed_ids':[1],
               'ambiguous_ids':[],'fault_by_id':{'2':4}}
        second={'complete':True,'confirmed_ids':list(range(1,7)),'unconfirmed_ids':[],
                'ambiguous_ids':[],'fault_by_id':{'2':0}}
        with patch.object(self.session,'emergency_stop',side_effect=[first,second]) as stop:
            result=self.session.emergency_stop_repeated()
        self.assertEqual(stop.call_count,2)
        self.assertTrue(result['complete']);self.assertEqual(result['fault_by_id']['2'],4)
        self.no_write()

    def test_repeated_stop_invalid_budgets_never_write(self):
        for args in ({'max_attempts':0},{'max_attempts':4},{'max_attempts':True},
                     {'total_timeout_ns':20_000_000},{'total_timeout_ns':1_000_000_001}):
            with self.subTest(args=args),self.assertRaises(ValueError):
                self.session.emergency_stop_repeated(**args)
        self.no_write()

    def test_extended_stop_budget_does_not_extend_active_budget(self):
        with self.assertRaises(ValueError):
            self.session.exchange([enable(1)],timeout_ns=250_000_001)
        with self.assertRaises(ValueError):
            self.session.emergency_stop(timeout_ns=500_000_001)
        self.no_write()

    def test_repeated_emergency_does_not_confirm_a_delayed_previous_stop_reply(self):
        pending=[]
        def first_stop(w):
            if ATParser().feed(w)[0].destination==1:
                pending.append(reply(w))
                return b''
            return reply(w)
        self.device_loop(6,first_stop)
        first=self.session.emergency_stop(timeout_ns=90_000_000)
        self.assertEqual(first['unconfirmed_ids'],[1])
        self.assertEqual(first['ambiguous_ids'],[])
        def repeated_stop(w):
            # Deliver only the old ID1 reply after its new STOP was written.
            # The protocol cannot distinguish this from a new STOP reply.
            return pending[0] if ATParser().feed(w)[0].destination==1 else reply(w)
        self.device_loop(6,repeated_stop)
        second=self.session.emergency_stop(timeout_ns=90_000_000)
        self.assertEqual(second['attempted_ids'],list(range(1,7)))
        self.assertEqual(second['ambiguous_ids'],[1])
        self.assertEqual(second['unconfirmed_ids'],[1])
        self.assertEqual(second['confirmed_ids'],list(range(2,7)))
        self.assertEqual(len(second['replies']),6)
        self.assertFalse(second['complete'])
        with self.assertRaises(RuntimeError):self.session.exchange([enable(1)])
        self.no_write()

    def test_pending_motion_mode_zero_is_not_false_stop_confirmation(self):
        self.device_loop(1,lambda w:b'')
        with self.assertRaises(native.ExchangeError):
            self.session.exchange([native.encode_motion(1,.1,1,.1)],timeout_ns=5_000_000)
        # A post-STOP mode0 reply could be the old Type1 after a watchdog disable.
        self.device_loop(6)
        result=self.session.emergency_stop()
        self.assertEqual(result['ambiguous_ids'],[1])
        self.assertEqual(result['confirmed_ids'],list(range(2,7)))
        self.assertFalse(result['complete'])

    def test_native_duplicate_fd_ownership_and_close_releases(self):
        with self.assertRaisesRegex(ValueError,'already owned'):self.make_session()
        duplicate=os.dup(self.host.fileno())
        try:
            with self.assertRaisesRegex(ValueError,'already owned'):self.make_session(duplicate)
        finally:os.close(duplicate)
        self.session.close();self.make_session()
        self.assertGreaterEqual(os.fstat(self.host.fileno()).st_ino,0)

    def test_boot_change_rejects_before_write(self):
        self.boot.seek(0);self.boot.write(('0'*36+'\n').encode());self.boot.flush()
        with self.assertRaisesRegex(native.ExchangeError,'Boot') as caught:self.session.exchange([enable(1)])
        self.assertEqual(caught.exception.stats.writes,0);self.no_write()

    def test_cxx_constructor_enforces_limits_independent_of_python(self):
        self.session.close()
        limits=native.Limits()
        for i in range(6):limits.lower[i]=-1;limits.upper[i]=1;limits.kp[i]=12;limits.kd[i]=.25
        for field,value in (('kp',36.1),('kd',1.1),('lower',float('nan')),('upper',12.58)):
            original=getattr(limits,field)[0];getattr(limits,field)[0]=value
            error=C.create_string_buffer(256)
            handle=self.lib.sda_create(self.host.fileno(),self.cancel_read,self.boot.fileno(),BOOT.encode(),
                1,C.byref(limits),600000,3,error,256)
            self.assertFalse(handle);self.assertIn(b'caps',error.value)
            getattr(limits,field)[0]=original

    def test_cxx_native_poison_cannot_be_cleared_via_python_flag(self):
        self.device_loop(1,lambda w:b'')
        with self.assertRaises(native.ExchangeError):self.session.exchange([enable(1)],timeout_ns=5_000_000)
        self.session.poisoned=False
        with self.assertRaisesRegex(native.ExchangeError,'poisoned'):self.session.exchange([enable(2)])
        self.no_write()

    def test_window_three_and_out_of_order_replies(self):
        pending=[]
        def reverse_three(w):
            pending.append(w)
            if len(pending)==3:
                out=b''.join(reply(x) for x in reversed(pending));pending.clear();return out
            return b''
        self.device_loop(6,reverse_three)
        records,_=self.session.exchange([native.encode_motion(i,.1,1,.1) for i in range(1,7)])
        self.assertTrue(all(r.received==17 for r in records))
        self.assertLess(records[2].received_ns,records[3].start_ns)

    def test_startup_probes_wait_for_each_reply_with_window_three(self):
        for request,respond in ((read_request,reply),(native.version_request,version_reply)):
            with self.subTest(request=request.__name__):
                def delayed(wire):
                    # Longer than the configured 600us write gap. A second
                    # request must not reach the peer while this reply is held.
                    self.assertFalse(select.select([self.peer],[],[],.004)[0])
                    return respond(wire)
                self.device_loop(6,delayed)
                records,stats=self.fresh().exchange([request(i) for i in range(1,7)])
                self.assertEqual(stats.writes,6)
                self.assertTrue(all(b.start_ns>=a.received_ns for a,b in zip(records,records[1:])))
                self.assertTrue(all(b.start_ns-a.finish_ns>=600000 for a,b in zip(records,records[1:])))

    def test_unanswered_startup_probes_do_not_fill_window_three(self):
        cases=((read_request,1),(native.version_request,1),
               (lambda i:native.encode_motion(i,.1,1,.1),3),
               (stop_wire,3),(lambda i:query(i,0x7019),3))
        for request,expected_writes in cases:
            with self.subTest(request=request.__name__,expected_writes=expected_writes):
                self.device_loop(expected_writes,lambda wire:b'')
                with self.assertRaisesRegex(native.ExchangeError,'deadline') as caught:
                    self.fresh().exchange([request(i) for i in range(1,7)],timeout_ns=15_000_000)
                self.assertEqual(caught.exception.stats.writes,expected_writes)
                self.assertEqual([r.written for r in caught.exception.records],
                                 [17]*expected_writes+[0]*(6-expected_writes))
                self.thread.join(timeout=1)
                self.no_write()

    def test_write_failure_is_not_retried_and_partial_prefix_is_preserved(self):
        # Test-only source transformation intercepts exactly the first native write.
        # It actually writes three bytes to the local socket. Production has no injection hook.
        self.session.close()
        with tempfile.TemporaryDirectory() as directory:
            d=Path(directory);source=d/'transport.cpp';binary=d/'libdog_active_transport.so'
            code=(ROOT/'transport.cpp').read_text()
            marker='\nextern "C" {\n'
            injected='''
static bool injected_partial = false;
static ssize_t test_partial_write(int fd, const void *p, size_t n) {
    if (!injected_partial) { injected_partial = true; return ::write(fd, p, 3); }
    return ::write(fd, p, n);
}
'''
            # Only production call sites are replaced; the local shim invokes the OS.
            code=code.replace('write(s->fd,r.tx,17)', 'test_partial_write(s->fd,r.tx,17)')
            code=code.replace(marker, '\n'+injected+marker,1)
            source.write_text(code)
            subprocess.run(['c++','-std=c++17','-O1','-pthread','-shared','-fPIC',str(source),'-o',str(binary)],check=True)
            (d/'build-record.json').write_text(json.dumps({'abi':1,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}))
            lib=native.load_library(binary)
            args=dict(first_id=1,cancel_fd=self.cancel_read,boot_fd=self.boot.fileno(),boot_id=BOOT,
                raw_lower_by_id={i:-1. for i in range(1,7)},raw_upper_by_id={i:1. for i in range(1,7)},
                kp_max_by_id={i:12. for i in range(1,7)},kd_max_by_id={i:.25 for i in range(1,7)})
            s=native.ActiveSession(lib,self.host.fileno(),**args)
            try:
                wires=[native.encode_motion(i,.1,1,.1) for i in range(1,7)]
                with self.assertRaisesRegex(native.ExchangeError,'Partial/failed') as caught:s.exchange(wires)
                self.assertEqual(caught.exception.stats.writes,1)
                self.assertEqual(caught.exception.records[0].written,3)
                self.assertTrue(all(r.written==0 for r in caught.exception.records[1:]))
                self.assertEqual(self.peer.recv(4096),wires[0][:3])
                with self.assertRaisesRegex(RuntimeError,'poisoned'):s.exchange(wires)
                self.no_write()
                self.device_loop(6)
                result=s.emergency_stop()
                self.assertFalse(result['complete'],result)
                self.assertEqual(result['ambiguous_ids'],[1])
                self.assertEqual(result['attempted_ids'],list(range(1,7)))
            finally:s.close()

    def test_replaced_fd_is_rejected_even_for_emergency(self):
        # Same integer descriptor, different local socket; never send STOP to an unrelated FD.
        a,b=socket.socketpair();a.setblocking(False)
        try:
            os.dup2(a.fileno(),self.host.fileno())
            with self.assertRaisesRegex(native.ExchangeError,'binding') as caught:self.session.exchange([enable(1)])
            self.assertEqual(caught.exception.stats.writes,0)
            result=self.session.emergency_stop()
            self.assertEqual(result['attempted_ids'],[])
            self.assertFalse(select.select([b],[],[],0)[0])
        finally:a.close();b.close()

    def test_real_pty_motion_frame_reply(self):
        master,slave=pty.openpty();tty.setraw(slave);os.set_blocking(slave,False)
        def run():
            if select.select([master],[],[],.5)[0]:
                wire=os.read(master,17);os.write(master,reply(wire))
        worker=threading.Thread(target=run);worker.start()
        s=None
        try:
            s=self.make_session(slave)
            records,_=s.exchange([native.encode_motion(1,.1,2,.15)])
            self.assertEqual(native.decode_record(records[0])['mode_state'],2)
        finally:
            worker.join(timeout=1)
            if s:s.close()
            os.close(master);os.close(slave)


class EncodingTests(unittest.TestCase):
    def test_zero_torque_velocity_and_limits(self):
        wire=native.encode_motion(12,.5,36,1)
        parsed=ATParser().feed(wire)[0]
        self.assertEqual((parsed.can_id>>8)&65535,32767)
        self.assertEqual(struct.unpack('>4H',parsed.data)[1],32767)
        for args in ((0,0,1,.1),(True,0,1,.1),(1,float('nan'),1,.1),(1,12.58,1,.1),
                     (1,0,36.1,.1),(1,0,1,1.1),(1,0,-1,.1)):
            with self.assertRaises(ValueError):native.encode_motion(*args)


if __name__=='__main__':unittest.main()
