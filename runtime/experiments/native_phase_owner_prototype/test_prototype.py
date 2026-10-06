"""Compile genuine fake backend; exercise pipe/race/deadline/lifetime contracts."""
import ctypes as C
import fcntl
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

from . import prototype as p


class NativePipeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory=tempfile.TemporaryDirectory();cls.root=Path(cls.directory.name).resolve()
        cls.build=p.build(cls.root/'fake.so');cls.bindings=p.Bindings(cls.root/'fake.so',cls.build['library_sha256'],cls.build)

    @classmethod
    def tearDownClass(cls):cls.directory.cleanup()

    def setUp(self):self.buses=[];self.owners=None

    def tearDown(self):
        if self.owners is not None and self.owners.handle is not None:self.owners.cancel();self.owners.close()
        for bus in self.buses:bus.close()

    def setup(self,transforms=(None,None)):
        self.buses=[p.FakeBus(bus,transforms[bus])for bus in range(2)]
        self.start_owner(self.buses[0].request_tx,self.buses[0].reply_rx,self.buses[1].request_tx,self.buses[1].reply_rx)
        return self.owners

    def start_owner(self,*fds):
        self.owners=p.Owners(self.bindings)
        self.owners.start(*fds)
        return self.owners

    def begin(self,generation=1):self.owners.begin(generation,self.bindings.now()+p.BUDGET_NS,seed=1729)

    def pair(self,phase,generation=1):
        pair=None
        while pair is None:pair=self.owners.collect(generation,phase)
        p.validate_pair(pair);return pair

    def acknowledge(self,phase,generation=1):
        pair=self.pair(phase,generation);self.owners.acknowledge(pair,validated=True);return pair

    def finish(self,generation=1):
        self.acknowledge(1,generation);self.acknowledge(2,generation);self.owners.submit_stop(generation,validated=True);self.acknowledge(3,generation)

    def test_two_persistent_owners_two_generations_exact_pipeline_and_gap(self):
        self.setup();threads=[]
        for generation in (1,2):
            self.begin(generation);self.finish(generation)
            status=self.owners.status();self.assertEqual(status.state,3);self.assertEqual(status.ready_mask,status.consumed_mask);self.assertEqual(status.ready_mask,63)
            self.assertEqual(status.exited_mask,0);self.assertEqual(status.stop_admitted,1)
            for row in status.records:self.assertEqual(row.generation,generation);self.assertLess(row.finish_ns,row.deadline_ns)
        closed=self.owners.close();self.assertEqual(closed.joined_mask,3);self.assertEqual(closed.owned_closed_mask,15)
        for bus in self.buses:self.assertEqual([row[2]for row in bus.requests],[1,2,3,1,2,3])

    def test_stop_requires_both_same_generation_validated_pairs(self):
        self.setup();self.begin()
        with self.assertRaisesRegex(ValueError,'explicitly validated'):self.owners.submit_stop(1,validated=True)
        self.acknowledge(1)
        with self.assertRaisesRegex(ValueError,'explicitly validated'):self.owners.submit_stop(1,validated=True)
        self.acknowledge(2)
        for value in (False,1,None):
            with self.assertRaisesRegex(ValueError,'Explicit'):self.owners.submit_stop(1,validated=value)
        self.assertTrue(all(all(row[2]!=3 for row in bus.requests)for bus in self.buses))
        self.owners.submit_stop(1,validated=True);self.acknowledge(3)

    def test_completed_unacknowledged_generation_cannot_be_overwritten(self):
        self.setup();self.begin();self.acknowledge(1);self.acknowledge(2);self.owners.submit_stop(1,validated=True)
        pair=self.pair(3);raw=bytes(pair)
        with self.assertRaisesRegex(ValueError,'fully acknowledged'):self.owners.begin(2,self.bindings.now()+p.BUDGET_NS)
        self.assertEqual(bytes(self.pair(3)),raw)
        self.owners.acknowledge(pair,validated=True)
        with self.assertRaisesRegex(ValueError,'Unconsumed'):self.owners.acknowledge(pair,validated=True)
        self.owners.begin(2,self.bindings.now()+p.BUDGET_NS);self.finish(2)

    def test_ack_snapshot_tamper_poison_and_no_stop(self):
        self.setup();self.begin();pair=self.pair(1);pair.records[0].received_ns+=1
        with self.assertRaisesRegex(ValueError,'changed before'):self.owners.acknowledge(pair,validated=True)
        self.assertEqual(self.owners.status().error_code,4)
        with self.assertRaises(ValueError):self.owners.submit_stop(1,validated=True)

    def test_generation_and_phase_never_accept_stale_or_wrong_ack(self):
        self.setup();self.begin();pair=self.pair(1)
        for value in (0,1,True,-1,2**64,float('nan')):
            if value==1 and type(value)is int:continue
            with self.assertRaises(ValueError):self.owners.begin(value,self.bindings.now()+p.BUDGET_NS)
        with self.assertRaisesRegex(ValueError,'generation'):self.owners.collect(2,1)
        pair.generation=2
        with self.assertRaisesRegex(ValueError,'generation'):self.owners.acknowledge(pair,validated=True)
        self.assertEqual(self.owners.status().generation,1)

    def test_exact_integer_boundaries_reject_bool_nan_and_overwide_wait(self):
        self.setup()
        for value in (True,float('nan'),-1,2**64):
            with self.assertRaises(ValueError):self.owners.begin(1,value)
        self.begin()
        for value in (True,float('nan'),-1,p.WAIT_SLICE_NS+1):
            with self.assertRaises(ValueError):self.owners.collect(1,1,value)
        for value in (0,4,True):
            with self.assertRaises(ValueError):self.owners.collect(1,value)

    def test_native_deadline_cannot_be_extended_beyond_20ms(self):
        self.setup()
        for deadline in (self.bindings.now(),self.bindings.now()+21_000_000):
            with self.assertRaisesRegex(ValueError,'20ms'):self.owners.begin(1,deadline)
        self.assertEqual(self.owners.status().generation,0)

    def test_partial_reply_preserved_and_missing_bytes_are_failure(self):
        self.setup((lambda packet:p._PACKET.pack(p._REPLY,*packet[1:])[:5],None));self.begin()
        with self.assertRaisesRegex(ValueError,'deadline'):self.pair(1)
        status=self.owners.close();row=status.records[0]
        self.assertEqual(row.written,32);self.assertEqual(row.received,5);self.assertNotEqual(row.status,1)
        self.assertEqual(status.stop_admitted,0)
        with self.assertRaisesRegex(ValueError,'Closed'):self.owners.begin(2,self.bindings.now()+p.BUDGET_NS)

    def test_fragmented_genuine_pipe_reply_validates_full_packet(self):
        def fragments(packet):
            raw=p._PACKET.pack(p._REPLY,*packet[1:]);yield raw[:7];time.sleep(.0003);yield raw[7:]
        self.setup((fragments,fragments));self.begin();self.finish()
        self.assertTrue(all(row.received==32 and row.status==1 for row in self.owners.status().records))

    def test_protocol_error_wins_later_timeout_and_cancel(self):
        self.setup((lambda packet:p._PACKET.pack(p._REPLY,packet[1]+1,*packet[2:]),None));self.begin()
        with self.assertRaisesRegex(ValueError,'mismatch'):self.pair(1)
        original=self.owners.status();self.owners.cancel();time.sleep(.021)
        with self.assertRaisesRegex(ValueError,'mismatch'):self.owners.collect(1,1)
        self.assertEqual(self.owners.status().error_code,4);self.assertEqual(bytes(self.owners.status().error),bytes(original.error))

    def test_all_reply_header_fields_reject(self):
        for field in (0,2,3,4):
            def wrong(packet,field=field):
                values=list(packet);values[0]=p._REPLY
                values[field]=b'INVALID!'if field==0 else values[field]+1
                return p._PACKET.pack(*values)
            self.setup((wrong,None));self.begin()
            with self.assertRaisesRegex(ValueError,'mismatch'):self.pair(1)
            self.assertEqual(self.owners.status().error_code,4);self.owners.close()
            for bus in self.buses:bus.close()
            self.buses=[]

    def test_cancel_from_other_thread_wakes_native_collect_and_sticks(self):
        self.setup((lambda packet:None,lambda packet:None));self.begin()
        thread=threading.Thread(target=self.owners.cancel);thread.start();thread.join(.5);self.assertFalse(thread.is_alive())
        with self.assertRaisesRegex(ValueError,'cancellation'):self.owners.collect(1,1)
        self.assertEqual(self.owners.status().cancelled,1)
        with self.assertRaisesRegex(ValueError,'Sticky'):self.owners.begin(2,self.bindings.now()+p.BUDGET_NS)

    def test_error_collect_zeroes_stale_buffers_and_keeps_actual_clock(self):
        self.setup((lambda packet:None,lambda packet:None));self.begin();self.owners.cancel()
        pair=p.Pair();C.memset(C.byref(pair),255,C.sizeof(pair));actual=C.c_uint64(123);error=C.create_string_buffer(160)
        result=self.bindings.lib.sdf_collect(self.owners.handle,1,1,0,C.byref(pair),C.byref(actual),error,len(error))
        self.assertEqual(result,-1);self.assertEqual(bytes(pair),bytes(C.sizeof(pair)));self.assertGreater(actual.value,0)

    def test_nonready_collect_zeroes_scratch_without_fabricated_rows(self):
        self.setup((lambda packet:None,lambda packet:None));self.begin()
        pair=p.Pair();C.memset(C.byref(pair),255,C.sizeof(pair));actual=C.c_uint64(123);error=C.create_string_buffer(160)
        result=self.bindings.lib.sdf_collect(self.owners.handle,1,1,0,C.byref(pair),C.byref(actual),error,len(error))
        self.assertEqual(result,0);self.assertEqual(bytes(pair),bytes(C.sizeof(pair)));self.assertGreater(actual.value,0)

    def test_wrong_thread_python_and_native_rejected_without_state_mutation(self):
        self.setup();errors=[]
        def wrong():
            try:self.owners.status()
            except ValueError as error:errors.append(str(error))
            out=p.Status();message=C.create_string_buffer(160)
            result=self.bindings.lib.sdf_status(self.owners.handle,C.byref(out),message,len(message));errors.append(message.value.decode());self.assertEqual(result,-1)
        thread=threading.Thread(target=wrong);thread.start();thread.join(.5)
        self.assertEqual(len(errors),2);self.assertTrue(all('Wrong coordinator'in value for value in errors));self.assertEqual(self.owners.status().generation,0)

    def test_reentrant_wrapper_call_refuses_before_native_access(self):
        self.setup();self.owners._lock.acquire()
        try:
            with self.assertRaisesRegex(ValueError,'Reentrant'):self.owners.status()
        finally:self.owners._lock.release()
        self.assertEqual(self.owners.status().generation,0)

    def test_closed_supplied_fd_and_duplicate_fd_reject_without_changing_others(self):
        self.buses=[p.FakeBus(bus)for bus in range(2)];front=self.buses[0];rear=self.buses[1]
        with self.assertRaises(ValueError):self.start_owner(front.request_tx,front.reply_rx,front.request_tx,rear.reply_rx)
        temp=os.dup(front.request_tx);os.close(temp)
        with self.assertRaises(ValueError):self.start_owner(temp,front.reply_rx,rear.request_tx,rear.reply_rx)
        self.assertNotEqual(fcntl.fcntl(front.request_tx,fcntl.F_GETFL),-1)

    def test_blocking_pipe_and_regular_file_reject_flags_unchanged(self):
        self.buses=[p.FakeBus(bus)for bus in range(2)];front,rear=self.buses;os.set_blocking(front.request_tx,True)
        before=fcntl.fcntl(front.request_tx,fcntl.F_GETFL)
        with self.assertRaises(ValueError):self.start_owner(front.request_tx,front.reply_rx,rear.request_tx,rear.reply_rx)
        self.assertEqual(fcntl.fcntl(front.request_tx,fcntl.F_GETFL),before)
        with tempfile.TemporaryFile()as regular:
            with self.assertRaises(ValueError):self.start_owner(regular.fileno(),front.reply_rx,rear.request_tx,rear.reply_rx)

    def test_closed_caller_fds_do_not_close_owned_duplicates(self):
        # Darwin adds kernel FWRITTEN after a write; compare caller-settable
        # configuration flags, rather than that kernel activity observation.
        self.setup();mask=os.O_ACCMODE|os.O_NONBLOCK|os.O_APPEND
        flags=[fcntl.fcntl(fd,fcntl.F_GETFL)&mask for bus in self.buses for fd in (bus.request_tx,bus.reply_rx)]
        self.begin();self.finish()
        self.assertEqual(flags,[fcntl.fcntl(fd,fcntl.F_GETFL)&mask for bus in self.buses for fd in (bus.request_tx,bus.reply_rx)])
        for bus in self.buses:os.close(bus.request_tx);os.close(bus.reply_rx)
        self.begin(2);self.finish(2);self.owners.close()

    def test_owned_shared_flag_tamper_poison_before_io(self):
        self.setup();os.set_blocking(self.buses[0].reply_rx,True);self.begin()
        with self.assertRaisesRegex(ValueError,'identity/flags'):self.pair(1)
        self.assertEqual(self.owners.status().error_code,6)

    def test_fake_reply_eof_is_error_not_missing_success(self):
        self.setup();os.close(self.buses[0].reply_tx);self.begin()
        with self.assertRaisesRegex(ValueError,'EOF|read failure'):self.pair(1)
        self.assertEqual(self.owners.status().error_code,5);self.assertEqual(self.owners.status().records[0].received,0)

    def test_closed_request_reader_is_epipe_not_process_sigpipe(self):
        self.setup();self.buses[0].stop.set();self.buses[0].thread.join(.5);os.close(self.buses[0].request_rx);self.begin()
        with self.assertRaisesRegex(ValueError,'write failed'):self.pair(1)
        self.assertEqual(self.owners.status().error_code,5)

    def test_closing_inflight_joins_before_owned_fd_close(self):
        self.setup((lambda packet:None,lambda packet:None));self.begin();status=self.owners.close()
        self.assertEqual(status.state,6);self.assertEqual(status.joined_mask,status.exited_mask);self.assertEqual(status.joined_mask,3)
        self.assertEqual(status.owned_closed_mask,15);self.assertEqual(status.cancelled,1);self.assertEqual(status.error_code,3)
        for bus in self.buses:self.assertGreaterEqual(fcntl.fcntl(bus.request_tx,fcntl.F_GETFD),0)

    def test_invalid_cleanup_deadline_retains_live_context_for_safe_retry(self):
        self.setup();out=p.Status();error=C.create_string_buffer(160)
        result=self.bindings.lib.sdf_close(self.owners.handle,self.bindings.now(),C.byref(out),error,len(error))
        self.assertEqual(result,-1);self.assertIsNotNone(self.owners.handle)
        self.assertEqual(self.owners.status().state,0);self.owners.close()

    def test_cancellation_serializes_with_native_deletion(self):
        self.setup((lambda packet:None,lambda packet:None));self.begin();errors=[]
        self.owners._lifetime_lock.acquire()
        thread=threading.Thread(target=lambda:self._cancel_capture(errors));thread.start()
        status=p.Status();error=C.create_string_buffer(160)
        try:
            result=self.bindings.lib.sdf_close(self.owners.handle,self.bindings.now()+100_000_000,C.byref(status),error,len(error))
            self.assertEqual(result,0);self.owners.handle=None;self.owners.closed_status=status
        finally:self.owners._lifetime_lock.release()
        thread.join(.5);self.assertFalse(thread.is_alive());self.assertEqual(errors,['Closed fake session'])

    def test_interrupt_after_native_close_rejects_dangling_token_without_uaf(self):
        self.setup();original=self.bindings.lib.sdf_close;token=self.owners.handle
        def interrupted(*args):
            result=original(*args);self.assertEqual(result,0)
            raise KeyboardInterrupt('after native close; before wrapper assignment')
        with mock.patch.object(self.bindings.lib,'sdf_close',side_effect=interrupted):
            with self.assertRaises(KeyboardInterrupt):self.owners.close()
        self.assertEqual(self.owners.handle,token)
        self.assertEqual(self.bindings.lib.sdf_cancel(token),-1)
        with self.assertRaisesRegex(ValueError,'Null fake session'):self.owners.status()
        with self.assertRaisesRegex(ValueError,'cleanup evidence unavailable'):self.owners.close()
        self.assertIsNone(self.owners.handle);self.assertIsNone(self.owners.closed_status)

    def test_construction_has_no_native_allocation_and_start_is_single_use(self):
        with mock.patch.object(self.bindings.lib,'sdf_create_into',side_effect=AssertionError('native allocation')):
            self.owners=p.Owners(self.bindings)
        self.assertIsNone(self.owners.handle)
        self.buses=[p.FakeBus(bus)for bus in range(2)]
        fds=(self.buses[0].request_tx,self.buses[0].reply_rx,self.buses[1].request_tx,self.buses[1].reply_rx)
        self.owners.start(*fds);token=self.owners.handle
        with self.assertRaisesRegex(ValueError,'single-use'):self.owners.start(*fds)
        self.assertEqual(self.owners.handle,token);self.owners.close()
        with self.assertRaisesRegex(ValueError,'single-use'):self.owners.start(*fds)

    def test_creation_cell_nonempty_or_missing_rejects_before_resources(self):
        error=C.create_string_buffer(160);cell=C.c_void_p(123)
        for target in (None,C.byref(cell)):
            self.assertEqual(self.bindings.lib.sdf_create_into(-1,-1,-1,-1,target,error,len(error)),-1)
            self.assertIn('Empty caller-owned',error.value.decode());self.assertEqual(cell.value,123)
        cell=C.c_void_p()
        self.assertEqual(self.bindings.lib.sdf_create_into(-1,-1,-1,-1,C.byref(cell),error,len(error)),-1)
        self.assertIsNone(cell.value)
        with self.assertRaises(AttributeError):getattr(self.bindings.lib,'sdf_create')

    def test_interrupt_after_native_creation_retains_token_and_can_reap(self):
        self.buses=[p.FakeBus(bus)for bus in range(2)]
        self.owners=p.Owners(self.bindings);original=self.bindings.lib.sdf_create_into
        def interrupted(*args):
            result=original(*args);self.assertEqual(result,0)
            raise KeyboardInterrupt('after native creation; before start returns')
        with mock.patch.object(self.bindings.lib,'sdf_create_into',side_effect=interrupted):
            with self.assertRaises(KeyboardInterrupt):self.owners.start(self.buses[0].request_tx,self.buses[0].reply_rx,self.buses[1].request_tx,self.buses[1].reply_rx)
        self.assertIsNotNone(self.owners.handle);self.assertEqual(self.owners.status().exited_mask,0)
        closed=self.owners.close();self.assertEqual(closed.joined_mask,3);self.assertEqual(closed.owned_closed_mask,15)

    def test_start_wrong_thread_and_reentry_reject_before_creation(self):
        self.owners=p.Owners(self.bindings);errors=[]
        def wrong():
            try:self.owners.start(0,1,2,3)
            except ValueError as error:errors.append(str(error))
        thread=threading.Thread(target=wrong);thread.start();thread.join(.5)
        self.assertEqual(errors,['Wrong coordinator thread']);self.assertIsNone(self.owners.handle)
        self.owners._lock.acquire()
        try:
            with self.assertRaisesRegex(ValueError,'Reentrant'):self.owners.start(0,1,2,3)
        finally:self.owners._lock.release()
        self.assertFalse(self.owners._creation_attempted)

    def test_collector_failure_actual_is_fresh_and_invalid_call_clears_it(self):
        self.setup((lambda packet:None,lambda packet:None));self.begin();self.owners.cancel()
        self.owners.last_collect_ns=123
        with self.assertRaises(ValueError):self.owners.collect(1,1)
        self.assertGreater(self.owners.last_collect_ns,123)
        with self.assertRaises(ValueError):self.owners.collect(1,True)
        self.assertEqual(self.owners.last_collect_ns,0)

    def test_managed_example_interrupt_after_close_preserves_unknown_and_reaps_fake_buses(self):
        original=self.bindings.lib.sdf_close;created=[];bus_factory=p.FakeBus
        def bus(*args):
            result=bus_factory(*args);created.append(result);return result
        first=[True]
        def interrupted(*args):
            result=original(*args)
            if first[0]:first[0]=False;raise KeyboardInterrupt('after native close')
            return result
        with mock.patch.object(self.bindings.lib,'sdf_close',side_effect=interrupted),mock.patch.object(p,'FakeBus',side_effect=bus):
            result=p.execute_example(self.bindings)
        self.assertEqual(result['status'],'INCOMPLETE_FAKE_PIPE_STATE_MACHINE')
        self.assertIn('KeyboardInterrupt',result['error']);self.assertTrue(result['cleanup_errors'])
        self.assertNotIn('cleanup',result);self.assertFalse(result['native_session_pending'])
        self.assertTrue(all(not value.thread.is_alive()for value in created))

    def test_managed_example_interrupt_after_creation_reaps_native_owners_and_fake_buses(self):
        original=self.bindings.lib.sdf_create_into;created=[];bus_factory=p.FakeBus
        def bus(*args):
            result=bus_factory(*args);created.append(result);return result
        def interrupted(*args):
            result=original(*args);self.assertEqual(result,0)
            raise KeyboardInterrupt('after native creation')
        with mock.patch.object(self.bindings.lib,'sdf_create_into',side_effect=interrupted),mock.patch.object(p,'FakeBus',side_effect=bus):
            result=p.execute_example(self.bindings)
        self.assertEqual(result['status'],'INCOMPLETE_FAKE_PIPE_STATE_MACHINE')
        self.assertIn('KeyboardInterrupt',result['error']);self.assertEqual(result['cleanup_errors'],[])
        self.assertEqual(result['cleanup']['joined_mask'],3);self.assertEqual(result['cleanup']['owned_closed_mask'],15)
        self.assertFalse(result['native_session_pending']);self.assertEqual(result['rows'],[])
        self.assertTrue(all(not value.thread.is_alive()for value in created))

    def _cancel_capture(self,errors):
        try:self.owners.cancel()
        except ValueError as error:errors.append(str(error))

    def test_abi_and_build_pins_are_exact(self):
        self.assertEqual(C.sizeof(p.Record),152);self.assertEqual(C.sizeof(p.Pair),320)
        wrong=dict(self.build,source_sha256='a'*64)
        with self.assertRaises(ValueError):p.Bindings(self.root/'fake.so',self.build['library_sha256'],wrong)
        with self.assertRaises(ValueError):p.Bindings(self.root/'fake.so','a'*64,self.build)


class FileOnlyTests(unittest.TestCase):
    def test_default_plan_never_loads_code_or_opens_pipe(self):
        with tempfile.TemporaryDirectory()as directory:
            output=Path(directory).resolve()/'plan.json'
            with mock.patch.object(p,'Bindings',side_effect=AssertionError('load')),mock.patch.object(p,'build',side_effect=AssertionError('compile')),mock.patch.object(p.os,'pipe',side_effect=AssertionError('pipe')):
                p.main(['--output',str(output)])
            result=json.loads(output.read_bytes());self.assertEqual(result['status'],'FILE_ONLY_PLAN');self.assertFalse(result['native_code_loaded']);self.assertFalse(result['pipes_opened'])

    def test_fresh_output_symlink_git_and_existing_rejected(self):
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory).resolve();out=root/'out.json';out.write_text('keep')
            with self.assertRaises(ValueError):p.fresh(out)
            out.unlink();out.symlink_to(root/'missing')
            with self.assertRaises(ValueError):p.fresh(out)
            out.unlink();(root/'.git').mkdir()
            with self.assertRaises(ValueError):p.fresh(out)

    def test_cli_abbreviation_rejected(self):
        with self.assertRaises(SystemExit):p.main(['--out','anything'])


if __name__=='__main__':unittest.main()
