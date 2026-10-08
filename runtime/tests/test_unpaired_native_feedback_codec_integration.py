"""Genuine two-bus ordinary ABI sockets; no serial/CAN/model/device approval.

Set R21_ACTIVE_LIBRARY to an already built ordinary ABI1 library with adjacent
transport.cpp and build-record.json. No tests compile or use a physical port.
Every active positive/negative phase retains its actual original 20 ms deadline.
"""
import ctypes as C
from concurrent.futures import Future
import copy
from dataclasses import replace
from contextlib import redirect_stdout,redirect_stderr
import io
import hashlib
import math
import os
from pathlib import Path
import select
import socket
import struct
import sys
import tempfile
import threading
import time
import types
import unittest
import uuid
import weakref
from unittest.mock import patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import policy_live_profile as profiles
from singularitydog_hw import unpaired_native_feedback_codec as selection
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.native_diagnostic_transport import stop_wire


def frame(identifier, data):
    return b'AT'+((identifier << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def reply(wire, *, mode=None, fault=0, voltage=38.):
    request=ATParser().feed(wire)[0];mid=request.destination
    if request.kind==0:
        return frame((mid<<8)|0xfe, bytes([mid])*8)
    if request.kind in (1,3,4,18):
        state=(0 if request.kind in (4,18) else 2) if mode is None else mode
        return frame((2<<24)|(state<<22)|(fault<<16)|(mid<<8)|0xfd,
                     struct.pack('>4H',32767,32767,32767,250))
    index=int.from_bytes(request.data[:2], 'little')
    value=(struct.pack('<I',4000) if index==0x7028 else bytes(4) if index==0x7005
           else struct.pack('<f',voltage))
    return frame((17<<24)|(mid<<8)|0xfd,request.data[:4]+value)


def ref(path):
    path=Path(path).absolute()
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


class TwoBusCodecTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        raw=os.environ.get('R21_ACTIVE_LIBRARY')
        if not raw:raise RuntimeError('Explicit R21_ACTIVE_LIBRARY required; no test rebuild/fallback')
        cls.library_path=Path(raw).resolve(strict=True)
        cls.original_switch=sys.getswitchinterval()
        sys.setswitchinterval(.0001)
        if abs(sys.getswitchinterval()-.0001)>1e-12:raise RuntimeError('100us switch readback differs')

    @classmethod
    def tearDownClass(cls):
        sys.setswitchinterval(math.nextafter(cls.original_switch,math.inf))
        if sys.getswitchinterval()!=cls.original_switch:raise RuntimeError('Original switch interval not exactly restored')

    def setUp(self):
        self.lib=active.load_library(self.library_path,expected_sha256=ref(self.library_path)['sha256'])
        self.cancel_read,self.cancel_write=os.pipe()
        self.boot=tempfile.TemporaryFile();self.boot_id=str(uuid.uuid4())
        self.boot.write((self.boot_id+'\n').encode());self.boot.flush()
        self.sockets={};self.sessions={};self.threads=[];self.peer_errors=[]
        self.end_peers=threading.Event();self.seen={s:[] for s in runtime.BUSES}
        self.events={s:[] for s in runtime.BUSES};self.behavior={s:reply for s in runtime.BUSES}
        self.workers=None
        for scope,ids in runtime.BUSES.items():
            host,peer=socket.socketpair();host.setblocking(False)
            self.sockets[scope]=(host,peer)
            self.sessions[scope]=active.ActiveSession(self.lib,host.fileno(),
                first_id=ids[0],cancel_fd=self.cancel_read,boot_fd=self.boot.fileno(),boot_id=self.boot_id,
                raw_lower_by_id={mid:-1. for mid in ids},raw_upper_by_id={mid:1. for mid in ids},
                kp_max_by_id={mid:3. for mid in ids},kd_max_by_id={mid:.15 for mid in ids},
                gap_ns=900_000,window=3)
        self.selected={'schema':selection.SELECTION_SCHEMA,'references':{
            'library':ref(self.library_path),'build_record':ref(self.library_path.parent/'build-record.json'),
            'library_source':ref(self.library_path.parent/'transport.cpp'),
            'runtime':ref(runtime.__file__),'binding':ref(active.__file__),
            'selection_source':ref(selection.__file__)},'output_allowed':False,'approved_for_runtime':False}
        self.profile={'max_sample_age_ms':20,'voltage_min_v':35.,'voltage_max_v':42.,
                      'axes':{str(mid):{'max_measured_velocity_rad_s':1.} for mid in runtime.IDS}}

    def tearDown(self):
        if self.workers is not None:self.workers.close()
        for session in self.sessions.values():session.close()
        self.end_peers.set()
        for host,peer in self.sockets.values():host.close();peer.close()
        for thread in self.threads:
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive(),'Synthetic peer must release socket')
        os.close(self.cancel_read);os.close(self.cancel_write);self.boot.close()
        if self.peer_errors:raise self.peer_errors[0]

    def start_peers(self):
        for scope in runtime.BUSES:
            ready=threading.Event();peer=self.sockets[scope][1]
            def run(scope=scope,peer=peer,ready=ready):
                parser=ATParser();ready.set()
                try:
                    while not self.end_peers.is_set():
                        if not select.select([peer],[],[],.05)[0]:continue
                        raw=peer.recv(4096)
                        if not raw:return
                        for request in parser.feed(raw):
                            self.seen[scope].append(request)
                            stamp=time.monotonic_ns()
                            result=self.behavior[scope](request.wire)
                            self.events[scope].append((request.kind,request.destination,stamp,time.monotonic_ns()))
                            if result:peer.sendall(result)
                except OSError:pass
                except BaseException as error:self.peer_errors.append(error)
            thread=threading.Thread(target=run,name='synthetic-'+scope);thread.start();self.threads.append(thread)
            self.assertTrue(ready.wait(1),'Peer startup must precede product deadline')

    def create_workers(self, *, selected=True, prepared=False):
        self.workers=runtime.BusWorkers(self.sessions,lambda:os.write(self.cancel_write,b'x'),
            prepare_voltage_before_feedback_publication=prepared,
            native_feedback_batch_decode=selected,
            native_feedback_codec_selection=self.selected if selected else None)
        for pool in self.workers.pools.values():pool.submit(lambda:None).result(timeout=1)
        return self.workers

    def motion_wires(self):
        return {s:[active.encode_motion(mid,0.,3.,.15) for mid in ids] for s,ids in runtime.BUSES.items()}

    def assert_raw(self,result,scope,kind):
        rows=runtime.decode_records(result)
        self.assertEqual(set(rows),{(mid,'feedback') for mid in runtime.BUSES[scope]})
        self.assertEqual(rows,self.workers._decode_bus_records(scope,result))
        for record in result[0]:
            self.assertEqual(ATParser().feed(bytes(record.tx))[0].kind,kind)
            self.assertEqual(record.written,17);self.assertEqual(record.received,17)
            self.assertLess(record.received_ns,record.deadline_ns)
        for old,new in zip(result[0],list(result[0])[1:]):
            self.assertGreaterEqual(new.start_ns-old.finish_ns,900_000)
        return rows

    def assert_stop_attempts_and_ambiguity(self,stops):
        for scope,stop in stops.items():
            self.assertEqual(stop['attempted_ids'],list(runtime.BUSES[scope]))
            self.assertEqual(len(stop['evidence']['records']),6)
            self.assertEqual(set(stop['confirmed_ids'])|set(stop['unconfirmed_ids']),set(runtime.BUSES[scope]))
            self.assertFalse(set(stop['confirmed_ids'])&set(stop['ambiguous_ids']))
            for attempt in stop['attempts']:
                for record in attempt['evidence']['records']:
                    self.assertEqual(ATParser().feed(bytes.fromhex(record['tx_hex']))[0].kind,4)

    def test_selected_genuine_two_bus_type1_voltage_full_output_and_all12_raw_stop(self):
        self.start_peers();workers=self.create_workers();begin=time.monotonic_ns();deadline=begin+20_000_000
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline])
        acquired=workers.collect(feedback)
        for scope,(raw,rows) in acquired.items():
            self.assert_raw(raw,scope,1);self.assertTrue(all(row[0].mode_state==2 for row in rows.values()))
        electric=workers.collect_voltage(voltage,deadline_ns=deadline)
        self.assertEqual({s:sorted(row[1]) for s,row in electric.items()},{'front':[1],'rear':[7]})
        outputs=workers.submit_decoded(self.motion_wires(),deadline_ns=deadline,label='policy_output')
        self.assertTrue(all(type(f) is Future for f in outputs.values()))
        result=workers.collect_output(outputs,deadline_ns=deadline)
        for scope,row in result.items():self.assert_raw(row[0],scope,1)
        self.assertLess(time.monotonic_ns(),deadline)
        stops=workers.finish_stops()
        for scope,stop in stops.items():
            self.assertTrue(stop['complete']);self.assertEqual(stop['confirmed_ids'],list(runtime.BUSES[scope]))
            self.assertEqual(len(stop['evidence']['records']),6)
            for record in stop['evidence']['records']:
                rx=ATParser().feed(bytes.fromhex(record['rx_hex']))[0]
                self.assertEqual(rx.kind,2);self.assertEqual((rx.can_id>>22)&3,0)
        self.assertIsNone(workers.native_pair)
        self.assertEqual({s:[f.kind for f in seen[:13]] for s,seen in self.seen.items()},
                         {s:[1]*6+[17]+[1]*6 for s in runtime.BUSES})

    def test_prepared_publication_uses_same_native_busy_owner_and_deadline(self):
        self.start_peers();workers=self.create_workers(prepared=True)
        publication=[];deadline=time.monotonic_ns()+20_000_000
        # Register before the real owner starts: this observes the exact genuine
        # public Future, not a wrapper or a synthetic completed Future.
        gate=threading.Event()
        for pool in workers.pools.values():pool.submit(gate.wait)
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline])
        for scope,future in feedback.items():
            future.add_done_callback(lambda done,scope=scope:publication.append(
                (scope,done is feedback[scope],self.sessions[scope].busy.locked(),voltage[scope].done())))
        gate.set();workers.collect(feedback);workers.collect_voltage(voltage,deadline_ns=deadline)
        self.assertEqual(sorted(publication),[('front',True,True,False),('rear',True,True,False)])
        for proof in workers.prepared_voltage_publications:
            self.assertEqual(proof['effective_deadline_ns'],deadline);self.assertEqual(proof['status'],'VALIDATED')
            self.assertLessEqual(proof['publication_checked_after_ns'],proof['voltage_native_begin_ns'])

    def test_off_constructor_retains_ordinary_legacy_decoder_and_original_owners(self):
        self.start_peers();workers=self.create_workers(selected=False)
        self.assertIsNone(workers.native_feedback_decoders);self.assertIsNone(workers.native_pair)
        self.assertIsNone(workers.unpaired_native_feedback_codec_proof)
        result=workers.exchange(self.motion_wires())
        for scope,raw in result.items():self.assert_raw(raw,scope,1)

    def test_identity_parameter_single_voltage_keep_original_fallback(self):
        self.start_peers();workers=self.create_workers()
        identity=workers.exchange({s:[read_request(mid) for mid in ids] for s,ids in runtime.BUSES.items()})
        for scope,raw in identity.items():
            self.assertEqual(workers._decode_bus_records(scope,raw),runtime.decode_records(raw))
            self.assertIsNone(workers.native_feedback_decoders[scope].decode(raw[0],runtime.BUSES[scope][0]))
        deadline=time.monotonic_ns()+20_000_000
        futures=workers.submit_voltage({'front':1,'rear':7},self.profile,deadline_ns=deadline)
        result=workers.collect_voltage(futures,deadline_ns=deadline)
        self.assertEqual(result['front'][1][1][0],38.)

    def test_original_complete_records_raw_unchanged_and_double_bits_equal(self):
        self.start_peers();workers=self.create_workers()
        result=workers.exchange(self.motion_wires())
        for scope,raw in result.items():
            image=(bytes(raw[0]),bytes(raw[1]));legacy=runtime.decode_records(raw)
            decoded=workers._decode_bus_records(scope,raw)
            self.assertEqual(list(decoded),list(legacy));self.assertEqual(decoded,legacy)
            for key,row in decoded.items():
                a,b=row[0],legacy[key][0]
                fields=lambda x:(x.protocol_position_rad,x.velocity_rad_s,x.torque_nm,x.temperature_c)
                self.assertEqual(struct.pack('>4d',*fields(a)),struct.pack('>4d',*fields(b)))
            self.assertEqual((bytes(raw[0]),bytes(raw[1])),image)

    def test_fault_feedback_fails_before_voltage_and_full_stop_is_preserved(self):
        self.behavior['front']=lambda w:reply(w,fault=1 if ATParser().feed(w)[0].kind==1 else 0)
        self.start_peers();workers=self.create_workers();deadline=time.monotonic_ns()+20_000_000
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline])
        with self.assertRaisesRegex(RuntimeError,'fault/mode'):workers.collect(feedback)
        stops=workers.finish_stops()
        self.assertFalse(any(f.kind==17 for f in self.seen['front']))
        self.assert_stop_attempts_and_ambiguity(stops)
        # A peer still receiving Type1 when its sibling fails may retain sticky
        # attribution ambiguity. Never turn those mode0 frames into success.

    def test_mode0_feedback_cannot_enter_voltage_pipeline(self):
        self.behavior['front']=lambda w:reply(w,mode=0)
        self.start_peers();workers=self.create_workers();deadline=time.monotonic_ns()+20_000_000
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline])
        with self.assertRaisesRegex(RuntimeError,'fault/mode'):workers.collect(feedback)
        self.assertFalse(any(f.kind==17 for f in self.seen['front']))
        self.assert_stop_attempts_and_ambiguity(workers.finish_stops())

    def test_voltage_limits_retained_after_pure_feedback_decode(self):
        self.behavior['front']=lambda w:reply(w,voltage=34.)
        self.start_peers();workers=self.create_workers();deadline=time.monotonic_ns()+20_000_000
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline]);workers.collect(feedback)
        with self.assertRaisesRegex(RuntimeError,'voltage outside'):workers.collect_voltage(voltage,deadline_ns=deadline)
        self.assertTrue(all(s['complete'] for s in workers.finish_stops().values()))

    def test_peer_delayed_reply_original20ms_failure_retains_partial_raw(self):
        def delayed(wire):
            request=ATParser().feed(wire)[0]
            if request.kind==1 and request.destination==6:
                time.sleep(.025);return b''
            return reply(wire)
        self.behavior['front']=delayed;self.start_peers();workers=self.create_workers()
        deadline=time.monotonic_ns()+20_000_000
        outputs=workers.submit_decoded(self.motion_wires(),deadline_ns=deadline,label='policy_output')
        with self.assertRaises((TimeoutError,active.ExchangeError,RuntimeError)):
            workers.collect_output(outputs,deadline_ns=deadline)
        for future in outputs.values():
            try:future.result(timeout=1)
            except BaseException:pass
        failed=[raw for scope,raw,error,label in workers.journal if scope=='front' and error]
        self.assertTrue(failed);self.assertEqual(failed[0][0][-1].written,17)
        self.assertEqual(failed[0][0][-1].received,0)
        self.assertEqual(failed[0][0][-1].deadline_ns,deadline)
        self.assertTrue(workers.aborted.is_set())
        workers.finish_stops() # ambiguous pending Type2 cannot be relabeled healthy STOP

    def test_ready_original_futures_taken_out_after_deadline_still_fail(self):
        self.start_peers();workers=self.create_workers();deadline=time.monotonic_ns()+20_000_000
        outputs=workers.submit_decoded(self.motion_wires(),deadline_ns=deadline,label='policy_output')
        for future in outputs.values():future.result(timeout=1)
        while time.monotonic_ns()<deadline:time.sleep(.0002)
        with self.assertRaisesRegex(TimeoutError,'deadline'):
            workers.collect_output(outputs,deadline_ns=deadline)
        self.assertTrue(workers.aborted.is_set());workers.finish_stops()

    def test_cancel_original_io_never_publishes_partial_feedback_success(self):
        self.behavior={s:lambda w:b'' if ATParser().feed(w)[0].kind==1 else reply(w) for s in runtime.BUSES}
        self.start_peers();workers=self.create_workers();deadline=time.monotonic_ns()+20_000_000
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline])
        os.write(self.cancel_write,b'x')
        with self.assertRaises((active.ExchangeError,RuntimeError)):workers.collect(feedback)
        self.assertTrue(all(f.done() and f.exception() is not None for f in feedback.values()))
        self.assertTrue(all(s['complete'] for s in workers.finish_stops().values()))

    def test_external_feedback_future_cancel_remains_failure_and_owner_stop(self):
        self.start_peers();workers=self.create_workers()
        gate=threading.Event()
        for pool in workers.pools.values():pool.submit(gate.wait)
        deadline=time.monotonic_ns()+20_000_000
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline]);feedback['front'].cancel();gate.set()
        with self.assertRaises(BaseException):workers.collect(feedback)
        self.assertTrue(workers.aborted.is_set());workers.finish_stops()

    def test_changed_native_codec_function_rejected_before_publication(self):
        self.start_peers();workers=self.create_workers()
        original=self.lib.sda_feedback_decode_batch
        self.lib.sda_feedback_decode_batch=lambda *args:0
        try:
            deadline=time.monotonic_ns()+20_000_000
            outputs=workers.submit_decoded(self.motion_wires(),deadline_ns=deadline,label='policy_output')
            with self.assertRaises((ValueError,RuntimeError,active.ExchangeError)):workers.collect(outputs)
            # The other genuine owner can observe cancellation first. Preserve
            # error-first settlement and the actual codec failure on its Future.
            errors=[future.exception() for future in outputs.values()]
            self.assertTrue(any(isinstance(e,ValueError) and 'Exact GIL-releasing' in str(e) for e in errors))
            self.assertTrue(workers.aborted.is_set());workers.finish_stops()
        finally:self.lib.sda_feedback_decode_batch=original

    def test_decoder_contention_uses_exact_legacy_without_raw_mutation(self):
        self.start_peers();workers=self.create_workers();raw=workers.exchange(self.motion_wires())['front']
        decoder=workers.native_feedback_decoders['front'];decoder._busy.acquire()
        try:self.assertEqual(workers._decode_bus_records('front',raw),runtime.decode_records(raw))
        finally:decoder._busy.release()

    def test_malformed_completed_record_preserves_legacy_error(self):
        self.start_peers();workers=self.create_workers();raw=workers.exchange(self.motion_wires())['front']
        raw[0][0].received=16;before=bytes(raw[0])
        with self.assertRaises(RuntimeError) as legacy:runtime.decode_records(raw)
        with self.assertRaises(type(legacy.exception)) as candidate:workers._decode_bus_records('front',raw)
        self.assertEqual(str(candidate.exception),str(legacy.exception));self.assertEqual(bytes(raw[0]),before)

    def test_root_owned_array_alias_falls_back_exactly(self):
        self.start_peers();workers=self.create_workers();raw=workers.exchange(self.motion_wires())['front']
        alias=(active.Record*6).from_address(C.addressof(raw[0]));result=(alias,raw[1])
        self.assertIsNone(workers.native_feedback_decoders['front'].decode(alias,1))
        self.assertEqual(workers._decode_bus_records('front',result),runtime.decode_records(result))

    def test_prepared_feedback_subscriber_consumes_original_deadline_without_voltage_write(self):
        self.start_peers();workers=self.create_workers(prepared=True)
        gate=threading.Event()
        for pool in workers.pools.values():pool.submit(gate.wait)
        deadline=time.monotonic_ns()+20_000_000
        feedback,voltage=workers.submit_feedback_then_voltage(self.motion_wires(),{'front':1,'rear':7},
            self.profile,deadline_ns=[deadline])
        feedback['front'].add_done_callback(lambda done:time.sleep(.025))
        gate.set()
        with self.assertRaises((RuntimeError,TimeoutError)):
            workers.collect_voltage(voltage,deadline_ns=deadline)
        for future in voltage.values():
            try:future.result(timeout=1)
            except BaseException:pass
        self.assertFalse(any(f.kind==17 for f in self.seen['front']))
        self.assertTrue(workers.aborted.is_set())
        self.assert_stop_attempts_and_ambiguity(workers.finish_stops())

    def test_no_overlapping_native_reader_when_same_owner_is_busy(self):
        entered=threading.Event();release=threading.Event()
        def hold(wire):
            if ATParser().feed(wire)[0].kind==1:
                entered.set();release.wait(.03)
            return reply(wire)
        self.behavior['front']=hold;self.start_peers();workers=self.create_workers()
        deadline=time.monotonic_ns()+20_000_000
        outputs=workers.submit_decoded(self.motion_wires(),deadline_ns=deadline,label='policy_output')
        self.assertTrue(entered.wait(1))
        try:
            with self.assertRaisesRegex(RuntimeError,'Concurrent'):
                self.sessions['front'].exchange([stop_wire(1)],deadline_ns=deadline)
        finally:release.set()
        workers.collect_output(outputs,deadline_ns=deadline)
        self.assertEqual(len([f for f in self.seen['front'] if f.kind==4]),0)

    def test_generic_or_session_subclass_cannot_claim_selected_native_ownership(self):
        original=self.sessions['front']
        for fake in (object(),types.SimpleNamespace(first_id=1,lib=self.lib,busy=threading.Lock())):
            self.sessions['front']=fake
            try:
                with self.assertRaisesRegex(ValueError,'Genuine original'):self.create_workers()
            finally:self.sessions['front']=original

    def synthetic_software_validation(self,profile):
        # Negative-admission fixture only. Never serialized or returned as an
        # actual target test/engineering review or used to create an approval.
        return {'schema':'singularitydog.unpaired-native-feedback-codec-source-validation.v1',
            'status':'PASS_CODEC_SOCKET_AND_SAVED_PARITY','source_sha256':profile['cadence_source_sha256'],
            'errors':[],'skips':0,'tests_passed':40,'saved_transactions_compared':1000,
            'hardware_opened':False,'output_allowed':False,'approved_for_runtime':False,
            'whole_loop_timing_qualified':False,'selection':copy.deepcopy(self.selected),
            'review':{'reviewer':'synthetic negative test fixture only',
                'reviewed_at':'2000-01-01T00:00:00Z','decision':'ACCEPT_SOURCE_BOUND_UNPAIRED_FEEDBACK_CODEC',
                'rationale':'Not an actual source review; drives the later rejection branch only.'},
            'test_output':ref(__file__),'checks':dict.fromkeys((
                'numeric_double_bits_exact','raw_bytes_order_timestamps_exact',
                'mode_fault_and_voltage_guards_unchanged','invalid_and_mixed_legacy_fallback',
                'partial_or_changed_abi_rejected','genuine_two_bus_original_futures',
                'prepared_voltage_deadline_cancellation','full_output_then_all12_raw_stop',
                'source_owner_and_busy_rejections'),True)}

    def test_old_ordinary_diagnostic_without_selected_codec_cannot_qualify_new_source(self):
        profile=SourceAdmissionTests().profile();profile['cadence_source_sha256']=profiles.cadence_source_hashes(profile)
        validation=self.synthetic_software_validation(profile)
        old={'cadence_source_sha256':profile['cadence_source_sha256'],
             'native_phase_pair':{'enabled':False}}
        with self.assertRaisesRegex(profiles.ProfileError,'Own current-source disabled codec producer'):
            profiles._unpaired_native_feedback_codec_evidence({
                'native_feedback_codec_source_validation':validation,'pipeline_diagnostic':old},profile)
        self.assertNotIn('_unpaired_native_feedback_codec_selection',profile)

    def test_pair_report_binding_cannot_qualify_independent_owners(self):
        profile=SourceAdmissionTests().profile();profile['cadence_source_sha256']=profiles.cadence_source_hashes(profile)
        validation=self.synthetic_software_validation(profile)
        refs=selection.verify_source_selection(validation['selection'])
        proof={'enabled':True,'selected_buses':['front','rear'],'scope':'ordinary_unpaired_owners.v1',
               'source_binding':{'schema':selection.PROOF_SCHEMA,'references':refs,'feedback_abi':1,
                   'active_abi':1,'adds_owner_or_future_or_request':False,
                   'timestamps_and_deadlines_unchanged':True,'unsupported_or_invalid_uses_legacy_codec':True}}
        report={'cadence_source_sha256':profile['cadence_source_sha256'],
                'native_phase_pair':{'enabled':True},'native_feedback_batch_decode':proof}
        with self.assertRaisesRegex(profiles.ProfileError,'Own current-source disabled codec producer'):
            profiles._unpaired_native_feedback_codec_evidence({
                'native_feedback_codec_source_validation':validation,'pipeline_diagnostic':report},profile)
        self.assertNotIn('_unpaired_native_feedback_codec_selection',profile)

    def test_stale_source_validation_and_positive_software_claim_rejected(self):
        profile=SourceAdmissionTests().profile();profile['cadence_source_sha256']=profiles.cadence_source_hashes(profile)
        for key in ('source_sha256','hardware_opened','whole_loop_timing_qualified'):
            validation=self.synthetic_software_validation(profile)
            validation[key]={} if key=='source_sha256' else True
            with self.assertRaisesRegex(profiles.ProfileError,'Own-source'):
                profiles._unpaired_native_feedback_codec_evidence({
                    'native_feedback_codec_source_validation':validation,'pipeline_diagnostic':{}},profile)

    def test_explicit_codec_rejects_busy_session_before_executor_creation(self):
        self.sessions['front'].busy.acquire()
        try:
            with patch.object(runtime,'ThreadPoolExecutor',side_effect=AssertionError('executor')), \
                 self.assertRaisesRegex(ValueError,'busy'):
                runtime.BusWorkers(self.sessions,lambda:None,native_feedback_batch_decode=True,
                                   native_feedback_codec_selection=self.selected)
        finally:self.sessions['front'].busy.release()
        self.assertFalse(self.sessions['rear'].busy.locked())

    def test_explicit_codec_rejects_phase_pair_owner(self):
        self.sessions['front']._phase_pair=object()
        try:
            with self.assertRaisesRegex(ValueError,'unpaired'):self.create_workers()
        finally:self.sessions['front']._phase_pair=None
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_explicit_codec_rejects_poisoned_or_closed_owner(self):
        for attribute,value in (('poisoned',True),('_handle',None)):
            original=getattr(self.sessions['front'],attribute);setattr(self.sessions['front'],attribute,value)
            try:
                with self.assertRaisesRegex(ValueError,'unpaired'):self.create_workers()
            finally:setattr(self.sessions['front'],attribute,original)
            self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_wrong_session_bus_and_rebound_source_rejected(self):
        self.sessions['front'].first_id=7
        try:
            with self.assertRaisesRegex(ValueError,'unpaired'):self.create_workers()
        finally:self.sessions['front'].first_id=1
        self.selected['references']['runtime']['sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'SHA differs'):self.create_workers()
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_ordinary_unpinned_load_valid_default_but_selected_codec_rejected(self):
        self.sessions['front'].lib=active.load_library(self.library_path)
        with self.assertRaisesRegex(ValueError,'authenticated'):self.create_workers()
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))
        self.sessions['front'].lib=self.lib

    def test_copied_or_cloned_binding_does_not_attest_an_unpinned_library(self):
        unpinned=active.load_library(self.library_path)
        genuine=self.lib._verified_active_source_binding
        unpinned._verified_active_source_binding=genuine
        with self.assertRaisesRegex(ValueError,'authenticated'):
            active.verified_active_source_binding(unpinned)
        # This imitates every public attribute check, including the unpinned
        # library's actual function identities, but cannot register that load.
        unpinned._verified_active_source_binding=replace(genuine,library=weakref.ref(unpinned),
            loaded_name=str(unpinned._name),functions=tuple(
                (getattr(unpinned,name),tuple(getattr(unpinned,name).argtypes or ()),
                 getattr(unpinned,name).restype,getattr(getattr(unpinned,name),'errcheck',None),
                 getattr(unpinned,name)._flags_) for name in active._ACTIVE_SOURCE_FUNCTIONS))
        self.sessions['front'].lib=unpinned
        try:
            with self.assertRaisesRegex(ValueError,'authenticated'):self.create_workers()
        finally:self.sessions['front'].lib=self.lib
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_registered_load_rejects_a_cloned_attached_binding(self):
        genuine=self.lib._verified_active_source_binding
        self.lib._verified_active_source_binding=replace(genuine)
        try:
            with self.assertRaisesRegex(ValueError,'authenticated'):self.create_workers()
        finally:self.lib._verified_active_source_binding=genuine
        self.assertIs(active.verified_active_source_binding(self.lib),genuine)

    def test_foreign_genuine_feedback_function_is_not_the_authenticated_codec(self):
        foreign=active.load_library(self.library_path)
        for name in active._FEEDBACK_SYMBOLS:
            original=getattr(self.lib,name)
            setattr(self.lib,name,getattr(foreign,name))
            try:
                with patch.object(runtime,'ThreadPoolExecutor',side_effect=AssertionError('executor')), \
                     self.assertRaisesRegex(ValueError,'authenticated'):
                    self.create_workers()
            finally:setattr(self.lib,name,original)
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_loaded_snapshot_name_is_not_the_original_path_but_binding_matches(self):
        self.assertNotEqual(self.lib._name,str(self.library_path))
        binding=active.verified_active_source_binding(self.lib)
        self.assertEqual(binding.path,str(self.library_path));self.assertEqual(binding.binary_sha256,ref(self.library_path)['sha256'])
        self.create_workers();self.assertTrue(all(d.available for d in self.workers.native_feedback_decoders.values()))

    def test_binding_or_native_owner_function_tamper_rejected(self):
        original=self.lib.sda_exchange;self.lib.sda_exchange=lambda *args:0
        try:
            with self.assertRaisesRegex(ValueError,'authenticated'):self.create_workers()
        finally:self.lib.sda_exchange=original
        proof=self.lib._verified_active_source_binding;self.lib._verified_active_source_binding=object()
        try:
            with self.assertRaisesRegex(ValueError,'authenticated'):self.create_workers()
        finally:self.lib._verified_active_source_binding=proof
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_original_owner_function_signature_mutation_rejected_before_executor(self):
        function=self.lib.sda_exchange;original=function.restype
        function.restype=C.c_longlong
        try:
            with patch.object(runtime,'ThreadPoolExecutor',side_effect=AssertionError('executor')), \
                 self.assertRaisesRegex(ValueError,'authenticated'):
                runtime.BusWorkers(self.sessions,lambda:None,native_feedback_batch_decode=True,
                                   native_feedback_codec_selection=self.selected)
        finally:function.restype=original
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_absent_or_partial_feedback_abi_explicit_fail_and_default_unchanged(self):
        original={name:getattr(self.lib,name) for name in active._FEEDBACK_SYMBOLS}
        try:
            for name in original:setattr(self.lib,name,None)
            with self.assertRaisesRegex(ValueError,'authenticated'):self.create_workers()
            setattr(self.lib,active._FEEDBACK_SYMBOLS[0],original[active._FEEDBACK_SYMBOLS[0]])
            with self.assertRaisesRegex(ValueError,'authenticated'):self.create_workers()
        finally:
            for name,value in original.items():setattr(self.lib,name,value)
        self.assertFalse(any(s.busy.locked() for s in self.sessions.values()))

    def test_non_bool_or_inactive_selection_or_pair_rejected_before_owner_start(self):
        for flag in (1,'yes',None):
            with self.assertRaisesRegex(RuntimeError,'bool'):
                runtime.BusWorkers(self.sessions,lambda:None,native_feedback_batch_decode=flag)
        with self.assertRaisesRegex(RuntimeError,'Inactive'):
            runtime.BusWorkers(self.sessions,lambda:None,native_feedback_codec_selection=self.selected)
        with self.assertRaisesRegex(RuntimeError,'phase owners'):
            runtime.BusWorkers(self.sessions,lambda:None,native_phase_pair=True,
                native_feedback_batch_decode=True,native_feedback_codec_selection=self.selected)


class SourceAdmissionTests(unittest.TestCase):
    def profile(self):
        return dict(schema=profiles.SCHEMA_V3,native_feedback_batch_decode=True,
            scope='supported_characterization_only',model_backend=profiles.SCALAR_BACKEND,
            local_characterization=profiles.LOCAL_RELATIVE_SUPPORTED,
            watchdog_review_policy=profiles.COMMAND_LOSS_ONLY_SUPPORTED,
            voltage_overlap=True,voltage_pipeline=True,native_phase_pair=False,
            request_gap_us=900,request_window=3,hard_cycle_ms=20,max_sample_age_ms=20,
            max_consecutive_20ms_misses=0,policy_weight=.005,duration_s=2.,
            diagnostic_timing_acceptance=profiles.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            axes={str(mid):{'kp':3.,'kd':.15,'max_displacement_from_start_rad':math.radians(1),
                'max_estimated_pd_torque_nm':.1} for mid in runtime.IDS})

    def test_default_profile_without_new_field_is_unchanged(self):
        self.assertFalse(profiles.native_feedback_batch_decode_settings({}))

    def test_candidate_plan_declares_without_fabricating_runtime_admission(self):
        profile=self.profile()
        self.assertTrue(profiles.native_feedback_batch_decode_settings(profile,require_approved=False))
        with self.assertRaisesRegex(profiles.ProfileError,'own complete'):
            profiles.native_feedback_batch_decode_settings(profile)

    def test_old_approved_profile_or_copied_pair_token_cannot_admit_codec(self):
        profile=self.profile();profile.update(output_allowed=True,
            _unpaired_native_feedback_codec_token=profiles._NATIVE_PHASE_PAIR_TOKEN)
        with self.assertRaisesRegex(profiles.ProfileError,'own complete'):
            profiles.native_feedback_batch_decode_settings(profile)

    def test_extra_caps_duration_pair_or_period_scope_is_rejected(self):
        for key,value in (('duration_s',20.),('native_phase_pair',True),('request_gap_us',890),
                          ('policy_weight',.01),('hard_cycle_ms',21),('max_sample_age_ms',21)):
            p=self.profile();p[key]=value
            with self.assertRaises(profiles.ProfileError):profiles.native_feedback_batch_decode_settings(p,require_approved=False)
        p=self.profile();p['axes']['1']['kp']=3.01
        with self.assertRaises(profiles.ProfileError):profiles.native_feedback_batch_decode_settings(p,require_approved=False)

    def test_profile_structure_and_settings_digest_bind_new_selection_field(self):
        candidate=profiles.template(schema=profiles.SCHEMA_V3)
        candidate.update(self.profile())
        candidate['cadence_source_sha256']=profiles.cadence_source_hashes(candidate)
        for mid in candidate['axes']:
            full=profiles.template(schema=profiles.SCHEMA_V3)['axes'][mid]
            full.update(candidate['axes'][mid]);candidate['axes'][mid]=full
        candidate['artifacts']={name:{'path':None,'sha256':None} for name in profiles.artifact_names(candidate)}
        profiles._structure(candidate)
        self.assertIn('native_feedback_batch_decode',profiles._profile_keys(candidate))
        self.assertIn('native_feedback_codec_source_validation',candidate['artifacts'])
        selected=profiles.reviewed_settings_sha256(candidate)
        candidate['native_feedback_batch_decode']=False
        candidate['cadence_source_sha256']=profiles.cadence_source_hashes(candidate)
        self.assertNotEqual(selected,profiles.reviewed_settings_sha256(candidate))

    def test_own_codec_source_is_pinned_only_when_explicitly_selected(self):
        base=profiles.cadence_source_paths({})
        selected=profiles.cadence_source_paths(self.profile())
        # This producer additionally pins its STOP-only actual BusWorkers seam.
        self.assertEqual(selected,base+(selection.SOURCE_PATH,
            'singularitydog_hw/diagnostic_runtime_output.py'))
        self.assertNotIn(selection.SOURCE_PATH,base)

    def test_missing_own_software_or_current_producer_proof_rejected(self):
        with self.assertRaisesRegex(profiles.ProfileError,'Own-source'):
            profiles._unpaired_native_feedback_codec_evidence({'pipeline_diagnostic':{}},self.profile())

    def test_bool_selection_type_and_legacy_schema_rejected(self):
        for selected in ('yes',1,None):
            p=self.profile();p['native_feedback_batch_decode']=selected
            with self.assertRaises(profiles.ProfileError):profiles.native_feedback_batch_decode_settings(p,require_approved=False)
        p=self.profile();p['schema']=profiles.SCHEMA_V2
        with self.assertRaises(profiles.ProfileError):profiles.native_feedback_batch_decode_settings(p,require_approved=False)

    def test_foreign_proof_on_default_profile_is_rejected(self):
        for name in ('_unpaired_native_feedback_codec_token','_unpaired_native_feedback_codec_selection',
                     '_unpaired_native_feedback_codec_binding'):
            with self.assertRaises(profiles.ProfileError):profiles.native_feedback_batch_decode_settings({name:object()})

class CodecCLITests(unittest.TestCase):
    def candidate(self):
        profile=profiles.template(schema=profiles.SCHEMA_V3)
        profile.update(SourceAdmissionTests().profile(),output_allowed=False,profile_sha256='0'*64)
        profile['cadence_source_sha256']=profiles.cadence_source_hashes(profile)
        return profile

    def test_new_flag_is_visible_without_loading_library_or_opening_devices(self):
        from singularitydog_hw import policy_output as cli
        stream=io.StringIO()
        with redirect_stdout(stream),self.assertRaises(SystemExit) as result:
            cli.main(['--help'])
        self.assertEqual(result.exception.code,0)
        self.assertIn('--native-feedback-batch-decode',stream.getvalue())

    def test_codec_plan_is_unapproved_and_does_not_open_transport(self):
        from singularitydog_hw import policy_output as cli
        candidate=self.candidate();output=io.StringIO()
        with patch.object(cli,'load_profile',return_value=candidate),redirect_stdout(output), \
             patch.object(active,'load_library',side_effect=AssertionError('native load')):
            self.assertEqual(cli.main(['--profile','unused','--native-feedback-batch-decode']),0)
        self.assertIn('"hardware_opened": false',output.getvalue())
        self.assertIn('"output_allowed": false',output.getvalue())

    def test_old_profile_plus_new_flag_rejects_before_device_setup(self):
        from singularitydog_hw import policy_output as cli
        p=self.candidate();p.pop('native_feedback_batch_decode')
        with patch.object(cli,'load_profile',return_value=p),redirect_stderr(io.StringIO()), \
             patch.object(active,'load_library',side_effect=AssertionError('native load')), \
             self.assertRaises(SystemExit) as result:
            cli.main(['--profile','unused','--native-feedback-batch-decode'])
        self.assertEqual(result.exception.code,2)

    def test_active_candidate_without_own_loader_proof_rejects_before_library(self):
        from singularitydog_hw import policy_output as cli
        p=self.candidate();p['output_allowed']=True # negative fixture, no approval file is created
        with patch.object(cli,'load_profile',return_value=p),redirect_stderr(io.StringIO()), \
             patch.object(active,'load_library',side_effect=AssertionError('native load')), \
             self.assertRaises(SystemExit) as result:
            cli.main(['--profile','unused','--execute-supported','--native-feedback-batch-decode'])
        self.assertEqual(result.exception.code,2)

if __name__=='__main__':unittest.main(verbosity=2)
