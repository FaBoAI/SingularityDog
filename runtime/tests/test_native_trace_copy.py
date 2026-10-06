"""Owned local memory and fake ports only; no device or native library load."""
import contextlib
import ctypes as C
import io
import json
import threading
import time
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device
from test_native_voltage_fast_pipeline import PreparedSession
from test_native_voltage_overlap import OverlapObserver


def owned_row():
    row={'cycle':1,'imu':{'proof':'original'},'observed':{'output_allowed':False},
         'voltage_fast_pipeline':{'status':'VALIDATED_BEFORE_PROXY_STOP','tick_ns':50000}}
    for phase,count in [('acquired',6),('voltage',1),('output',6)]:
        row[phase]={}
        for scope,ids in bench.dual.SCOPES.items():
            records=(bench.native.Record*count)()
            for ordinal,record in enumerate(records):
                mid=ids[ordinal]
                tx=bench._READ_WIRES[mid,'voltage'] if phase=='voltage' else bench._STOP_WIRES[mid]
                header=b'AT'+((((17<<24)|(mid<<8)|0xfd)<<3)|4).to_bytes(4,'big')+b'\x08'
                rx=(header+b'\x1c\x70\x00\x00\x00\x00\x14\x42\r\n' if phase=='voltage'
                    else bench._STOP_REPLY_HEADERS[mid]+bytes((ordinal+i)%256 for i in range(8))+b'\r\n')
                record.start_ns=1000+ordinal*100;record.finish_ns=record.start_ns+10
                record.read_start_ns=record.start_ns+12;record.received_ns=record.start_ns+20
                record.deadline_ns=100000;record.written=record.received=17
                record.tx[:]=tx;record.rx[:]=rx
            stats=bench.native.Stats()
            stats.begin_ns=900;stats.end_ns=3000;stats.reads=count;stats.bytes=17*count
            stats.writes=count;stats.waits=2;stats.rejected_size=3;stats.rejected[:3]=b'xyz'
            row[phase][scope]=(records,stats)
    return row


class NativeTraceCopyTests(unittest.TestCase):
    def selected(self):
        return ['--mode','stop-proxy','--supported-disabled','--v3-voltage-proxy',
                '--record-storage','trace','--cycles','5',
                '--provenance-mode','supported-geometric-preload-5s-v1',
                '--power-epoch','synthetic-disabled-test-only','--retain-gil-trace-copy']

    def reject(self,argv):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as caught:
            bench.main(argv)
        self.assertEqual(caught.exception.code,2)

    def test_factory_same_address_exact_flags_abi_and_no_copy(self):
        default=bench.C.memmove
        backend=bench._retained_gil_trace_copy()
        proof=backend.verify()
        self.assertIs(bench.C.memmove,default)
        self.assertEqual(default._flags_,1)
        self.assertEqual(backend._function._flags_,5)
        self.assertEqual(C.cast(default,C.c_void_p).value,C.cast(backend._function,C.c_void_p).value)
        self.assertEqual(backend._function.argtypes,(C.c_void_p,C.c_void_p,C.c_size_t))
        self.assertIs(backend._function.restype,C.c_void_p)
        self.assertEqual(proof['completed_copy_calls'],0)
        self.assertFalse(proof['new_native_library_loaded'])
        self.assertFalse(proof['active_controller_qualification'])

    def test_factory_rejects_wrong_original_or_selected_abi_before_any_copy(self):
        with patch.object(C,'memmove',Mock(_flags_=5)),self.assertRaisesRegex(ValueError,'CFUNCTYPE'):
            bench._retained_gil_trace_copy()
        # CFUNCTYPE has the same 3-argument ABI but releases the GIL, so it must
        # never substitute for the explicitly selected PYFUNCTYPE.
        with patch.object(C,'PYFUNCTYPE',C.CFUNCTYPE),self.assertRaisesRegex(ValueError,'ABI/binding'):
            bench._retained_gil_trace_copy()

    def test_local_same_bytes_including_zero_and_record_stats_lengths(self):
        backend=bench._retained_gil_trace_copy()
        lengths=(0,1,17,88,528,1056,C.sizeof(bench.native.Stats))
        for length in lengths:
            with self.subTest(length=length):
                raw=bytes((i*17+3)%256 for i in range(max(length,1)))
                source=C.create_string_buffer(raw)
                default=C.create_string_buffer(max(length,1));selected=C.create_string_buffer(max(length,1))
                C.memmove(C.addressof(default),C.addressof(source),length)
                self.assertEqual(backend(C.addressof(selected),C.addressof(source),length),C.addressof(selected))
                self.assertEqual(bytes(default),bytes(selected))
        self.assertEqual(backend.provenance()['completed_copy_calls'],len(lengths))
        self.assertEqual(backend.provenance()['completed_copy_bytes'],sum(lengths))

    def test_both_overlap_directions_match_memmove(self):
        backend=bench._retained_gil_trace_copy()
        for destination,source in ((3,0),(0,3)):
            with self.subTest(destination=destination):
                a=C.create_string_buffer(b'abcdefghijklmnop');b=C.create_string_buffer(b'abcdefghijklmnop')
                C.memmove(C.addressof(a)+destination,C.addressof(a)+source,12)
                backend(C.addressof(b)+destination,C.addressof(b)+source,12)
                self.assertEqual(bytes(a),bytes(b))

    def test_bad_pointer_length_or_bool_rejects_before_copy(self):
        backend=bench._retained_gil_trace_copy()
        for args in [(0,1,0),(1,0,0),(-1,1,0),(True,1,0),(1,1,True),
                     (1,1,-1),(1,1,4153),(1,1,1.0),(1,None,0)]:
            with self.subTest(args=args),self.assertRaises(ValueError):backend(*args)
        self.assertEqual(backend.provenance()['completed_copy_calls'],0)

    def test_unvalidated_backend_rejected(self):
        with self.assertRaisesRegex(ValueError,'Validated'):
            bench._RecordTrace(1,'stop-proxy',trace_copy_backend=lambda *args:None)
        backend=bench._retained_gil_trace_copy();backend._token=object()
        with self.assertRaisesRegex(ValueError,'ABI/binding'):backend.verify()

    def test_abi_change_rejects_before_copy(self):
        backend=bench._retained_gil_trace_copy()
        backend._function.argtypes=(C.c_int,C.c_int,C.c_int)
        with self.assertRaisesRegex(ValueError,'ABI/binding'):backend(1,1,0)
        self.assertEqual(backend.provenance()['completed_copy_calls'],0)

    def test_routine_or_source_change_rejects_end_verify(self):
        backend=bench._retained_gil_trace_copy()
        with patch.object(C,'memmove',Mock()),self.assertRaisesRegex(ValueError,'ABI/binding'):backend.verify()
        with patch.object(bench.Path,'read_bytes',return_value=b'changed-source'),self.assertRaisesRegex(ValueError,'routine/source'):
            backend.verify()

    def test_wrong_return_never_counts_a_completed_copy(self):
        backend=bench._retained_gil_trace_copy()
        backend._function=Mock(return_value=2)
        with patch.object(backend,'_verify_abi'),self.assertRaisesRegex(ValueError,'different destination'):
            backend(1,1,0)
        self.assertEqual(backend.provenance()['completed_copy_calls'],0)

    def test_trace_all_raw_bytes_stats_and_metadata_match_default(self):
        row=owned_row();backend=bench._retained_gil_trace_copy()
        a=bench._RecordTrace(1,'stop-proxy',voltage_overlap=True)
        b=bench._RecordTrace(1,'stop-proxy',voltage_overlap=True,trace_copy_backend=backend)
        default=a.capture(0,row);selected=b.capture(0,row)
        self.assertEqual(bytes(a.slots),bytes(b.slots))
        self.assertEqual(default.serialize(),selected.serialize())
        self.assertEqual(selected.metadata['voltage_fast_pipeline'],row['voltage_fast_pipeline'])
        self.assertEqual(backend.provenance()['completed_copy_calls'],12)
        self.assertEqual(backend.provenance()['completed_copy_bytes'],27200)
        frozen=bytes(b.slots)
        row['acquired']['front'][0][0].tx[0]=0;row['output']['rear'][1].rejected[0]=0
        self.assertEqual(bytes(b.slots),frozen)

    def test_default_copy_still_uses_original_ctypes_memmove(self):
        trace=bench._RecordTrace(1,'stop-proxy',voltage_overlap=True)
        with patch.object(C,'memmove',wraps=C.memmove) as copier:
            trace.capture(0,owned_row())
        self.assertIsNone(trace.trace_copy_backend)
        self.assertEqual(copier.call_count,12)

    def test_original_count_type_capacity_checks_precede_selected_call(self):
        for issue in ('count','record_type','stats_type','capacity','scope'):
            row=owned_row();backend=bench._retained_gil_trace_copy()
            trace=bench._RecordTrace(1,'stop-proxy',voltage_overlap=True,trace_copy_backend=backend)
            index=0
            if issue=='count':row['acquired']['front']=((bench.native.Record*13)(),bench.native.Stats())
            elif issue=='record_type':row['acquired']['front']=((C.c_uint64*6)(),bench.native.Stats())
            elif issue=='stats_type':row['acquired']['front']=(row['acquired']['front'][0],object())
            elif issue=='capacity':index=1
            else:row['acquired']={'unknown':row['acquired']['front']}
            with self.subTest(issue=issue),self.assertRaises(ValueError):trace.capture(index,row)
            self.assertEqual(backend.provenance()['completed_copy_calls'],0)

    def test_cli_requires_full_disabled_v3_trace_source_scope(self):
        original=self.selected()
        for flag in ('--supported-disabled','--v3-voltage-proxy','--record-storage',
                     '--provenance-mode','--power-epoch'):
            changed=list(original);index=changed.index(flag)
            del changed[index:index+(2 if flag in ('--record-storage','--provenance-mode','--power-epoch') else 1)]
            with self.subTest(flag=flag):self.reject(changed)
        for flag in ('--acquisition-only','--compare-feedback'):
            with self.subTest(flag=flag):self.reject(original+[flag])
        changed=list(original);changed[changed.index('--mode')+1]='type17';self.reject(changed)

    def test_cli_native_guard_mutually_exclusive_before_guard_plan(self):
        with patch.object(bench,'_retained_gil_trace_copy',side_effect=AssertionError('Do not construct')):
            for flag in ('--native-boot-guard-artifact','--native-boot-guard-artifact-sha256'):
                self.reject(self.selected()+[flag,'a'*64])

    def test_plan_proof_without_copy_or_ports_and_default_absent(self):
        with (patch.object(bench,'_start_source_provenance',return_value={'synthetic':True}),
              patch.object(bench.dual,'validate_ports',side_effect=AssertionError('No ports')),
              patch.object(bench.native,'load_library',side_effect=AssertionError('No library')),
              contextlib.redirect_stdout(io.StringIO()) as stdout):
            self.assertEqual(bench.main(self.selected()),0)
        value=json.loads(stdout.getvalue());proof=value['trace_copy_provenance']
        self.assertEqual(proof['completed_copy_calls'],0)
        self.assertEqual(proof['default_address'],proof['selected_address'])
        self.assertFalse(proof['active_controller_qualification'])
        self.assertFalse(value['enable_available']);self.assertFalse(value['learned_targets_sent'])
        self.assertEqual(value['type1_requests_per_cycle'],0)
        with patch.object(bench,'_start_source_provenance',return_value=None),contextlib.redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(bench.main([]),0)
        self.assertNotIn('trace_copy_provenance',json.loads(stdout.getvalue()))

    def test_factory_error_before_execution_has_no_hardware_or_output(self):
        with (patch.object(bench,'_retained_gil_trace_copy',side_effect=ValueError('Wrong function flags')),
              patch.object(bench.native,'load_library',side_effect=AssertionError('No library')),
              patch.object(bench.dual,'validate_ports',side_effect=AssertionError('No ports'))):
            self.reject(self.selected()+['--execute'])

    def test_collect_wrong_mode_or_storage_rejected_before_worker(self):
        backend=bench._retained_gil_trace_copy()
        for options in ({'mode':'type17','record_storage':'trace','v3_voltage_proxy':True},
                        {'mode':'stop-proxy','record_storage':'objects','v3_voltage_proxy':True},
                        {'mode':'stop-proxy','record_storage':'trace','v3_voltage_proxy':False}):
            with self.subTest(options=options),patch.object(bench,'ThreadPoolExecutor',side_effect=AssertionError('No worker')):
                with self.assertRaisesRegex(ValueError,'Retained-GIL'):
                    bench.collect({},object(),object(),cycles=1,trace_copy_backend=backend,**options)

    def run_fake_collect(self,backend):
        started=(threading.Event(),threading.Event());release=threading.Event()
        sessions={scope:PreparedSession(started[i],release)
                  for i,scope in enumerate(('front','rear'))}
        observer=OverlapObserver(started,release)
        report,raw=bench.collect(sessions,Device(),observer,mode='stop-proxy',cycles=1,
            v3_voltage_proxy=True,v3_voltage_overlap=True,v3_voltage_validation_overlap=True,
            v3_voltage_fast_pipeline=True,record_storage='trace',trace_copy_backend=backend)
        return report,bench._serialize(raw),sessions,observer

    def test_full_26_request_collector_preserves_stop_proof_and_measured_copy_end(self):
        backend=bench._retained_gil_trace_copy();original=bench._RecordTrace.capture
        copy_times=[]
        def capture(trace,*args):
            copy_times.append(time.monotonic_ns())
            result=original(trace,*args)
            time.sleep(.002)
            copy_times.append(time.monotonic_ns())
            return result
        with patch.object(bench._RecordTrace,'capture',capture):
            report,rows,sessions,observer=self.run_fake_collect(backend)
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(report['cycles_completed'],1)
        proof=report['trace_copy_provenance']
        self.assertTrue(proof['source_files_unchanged'])
        self.assertEqual(proof['completed_copy_calls'],12)
        self.assertEqual(proof['completed_copy_bytes'],27200)
        self.assertFalse(proof['active_controller_qualification'])
        self.assertFalse(report['motor_enable_sent']);self.assertFalse(report['learned_targets_sent'])
        row=rows[0];wire_proof=row['voltage_fast_pipeline']
        self.assertEqual(wire_proof['stop_reply_count'],12)
        self.assertEqual(sum(len(row[phase][scope]['records'])
            for phase in ('acquired','voltage','output') for scope in sessions),26)
        self.assertLessEqual(wire_proof['stop_reply_verified_ns'],copy_times[0])
        self.assertLessEqual(copy_times[1],report['measurements'][0]['cycle_end_ns'])
        self.assertGreaterEqual(report['measurements'][0]['whole_iteration_ms'],2.)
        self.assertTrue(all(s.phases==['feedback','voltage','output'] for s in sessions.values()))
        self.assertFalse(observer.invalid)

    def test_copy_failure_retains_original_complete_stop_replies_and_aborts(self):
        backend=bench._retained_gil_trace_copy()
        with patch.object(bench._RetainedGILTraceCopy,'__call__',side_effect=RuntimeError('synthetic copy failure')):
            report,rows,sessions,observer=self.run_fake_collect(backend)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('synthetic copy failure' in error for error in report['errors']))
        self.assertEqual(report['trace_copy_provenance']['completed_copy_calls'],0)
        self.assertFalse(report['approved_for_runtime'])
        self.assertFalse(report['motor_enable_sent']);self.assertFalse(report['learned_targets_sent'])
        self.assertEqual(sum(len(rows[0]['output'][scope]['records']) for scope in sessions),12)
        self.assertTrue(observer.invalid)

    def test_final_source_mismatch_invalidates_completed_collection(self):
        backend=bench._retained_gil_trace_copy();original=bench._RecordTrace.capture
        def capture(trace,*args):
            result=original(trace,*args);backend._source_sha256='0'*64;return result
        with patch.object(bench._RecordTrace,'capture',capture):
            report,rows,_,observer=self.run_fake_collect(backend)
        self.assertEqual(report['status'],'ABORTED')
        self.assertFalse(report['trace_copy_provenance']['source_files_unchanged'])
        self.assertEqual(report['trace_copy_provenance']['completed_copy_calls'],12)
        self.assertTrue(observer.invalid)
        self.assertEqual(rows[0]['voltage_fast_pipeline']['stop_reply_count'],12)


if __name__=='__main__':unittest.main()
