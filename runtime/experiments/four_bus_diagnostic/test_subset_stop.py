"""Genuine C++ exact-three STOPs over anonymous sockets/PTY; no hardware."""
from concurrent.futures import Future
import ctypes as C
import hashlib
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
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from . import build
from .transport_adapter import Group, ThreeAxisTransport, load_library, verify_library, sha

BOOT = '11111111-2222-3333-4444-555555555555'


def frame(can_id, data):
    return b'AT'+((can_id << 3)|4).to_bytes(4, 'big')+b'\x08'+data+b'\r\n'


def reply(wire, *, fault=0, mode=0):
    value = ATParser().feed(wire)[0]
    if value.kind == 17:
        return frame((17 << 24)|(value.destination << 8)|0xfd,
                     value.data[:4]+struct.pack('<f', 40.))
    return frame((2 << 24)|(mode << 22)|(fault << 16)|(value.destination << 8)|0xfd,
                 struct.pack('>4H', 32767, 32767, 32767, 250))


class SubsetStopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_switch = sys.getswitchinterval()
        sys.setswitchinterval(.0001)
        cls.directory = tempfile.TemporaryDirectory()
        selected = os.environ.get('FOUR_BUS_TEST_LIBRARY')
        cls.library_path = Path(selected).resolve() if selected else build.build(Path(cls.directory.name)/'build')
        record_path = cls.library_path.parent/'build-record.json'
        cls.record = json.loads(record_path.read_bytes())
        cls.lib = load_library(cls.library_path, expected_sha256=sha(cls.library_path),
            ordinary_source_sha256=cls.record['four_bus_subset_stop']['ordinary_source_sha256'],
            extension_source_sha256=cls.record['source_sha256'], build_record_sha256=sha(record_path))

    @classmethod
    def tearDownClass(cls):
        try:
            # CPython getters may round the saved value below a microsecond
            # boundary; nextafter restores its exact original readback.
            sys.setswitchinterval(math.nextafter(cls.original_switch, math.inf))
            assert sys.getswitchinterval() == cls.original_switch
        finally:
            cls.directory.cleanup()

    def setUp(self):
        self.host, self.peer = socket.socketpair(); self.host.setblocking(False)
        self.cancel_read, self.cancel_write = os.pipe()
        self.boot = tempfile.TemporaryFile(); self.boot.write((BOOT+'\n').encode()); self.boot.flush()
        self.adapters = []; self.thread = None; self.seen = []; self.peer_error = None; self.extra_fds = []

    def adapter(self, ids=(1, 2, 3), fd=None):
        group = Group('port0', ids)
        half = range(group.first_id, group.first_id+6)
        result = ThreeAxisTransport.create(self.lib, self.host.fileno() if fd is None else fd,
            group=group, cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
            raw_lower_by_id={mid: -1. for mid in half}, raw_upper_by_id={mid: 1. for mid in half})
        self.adapters.append(result); return result

    def peer_loop(self, count, *, mutate=reply, fragmented=False, fd=None):
        if self.thread:
            self.thread.join(timeout=1)
            self.assertFalse(self.thread.is_alive(), 'Previous synthetic peer remains active')
        ready = threading.Event()
        descriptor = self.peer.fileno() if fd is None else fd
        def run():
            parser = ATParser(); seen = 0; ready.set()
            try:
                while seen < count:
                    if not select.select([descriptor], [], [], 1)[0]: return
                    raw = os.read(descriptor, 4096)
                    if not raw: return
                    for value in parser.feed(raw):
                        self.seen.append(value); seen += 1
                        response = mutate(value.wire)
                        if response:
                            chunks = (response[:5], response[5:11], response[11:]) if fragmented else (response,)
                            for chunk in chunks:
                                os.write(descriptor, chunk)
                                if fragmented: time.sleep(.0001)
            except OSError:
                pass
            except BaseException as error:
                self.peer_error = error
        self.thread = threading.Thread(target=run, name='four-subset-synthetic-peer')
        self.thread.start(); self.assertTrue(ready.wait(timeout=1))

    def tearDown(self):
        for adapter in self.adapters: adapter.close()
        self.host.close(); self.peer.close()
        if self.thread: self.thread.join(timeout=1)
        os.close(self.cancel_read); os.close(self.cancel_write); self.boot.close()
        for fd in self.extra_fds:
            try: os.close(fd)
            except OSError: pass
        if self.peer_error: raise self.peer_error

    def check_group_stop(self, ids):
        adapter = self.adapter(ids); self.peer_loop(3)
        result = adapter.recover_subset()
        self.assertTrue(result['complete'], result)
        self.assertEqual(result['attempted_ids'], list(ids))
        self.assertEqual(result['confirmed_ids'], list(ids))
        self.assertEqual([value.destination for value in self.seen], list(ids))
        self.assertTrue(all(value.wire == stop_wire(value.destination) for value in self.seen))
        self.assertEqual(len(result['raw']['records']), 6)
        untouched = set(range(adapter.group.first_id, adapter.group.first_id+6))-set(ids)
        for mid in untouched:
            row = result['raw']['records'][mid-adapter.group.first_id]
            self.assertEqual(row['written'], 0); self.assertEqual(row['received'], 0)
        self.assertTrue(adapter._session.poisoned)
        with self.assertRaisesRegex(RuntimeError, 'poisoned'):
            adapter.output_stop(deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_subset_1_to_3_uses_only_mask7(self): self.check_group_stop((1, 2, 3))
    def test_subset_4_to_6_uses_only_mask56(self): self.check_group_stop((4, 5, 6))
    def test_subset_10_to_12_uses_half7_mask56(self): self.check_group_stop((10, 11, 12))

    def test_invalid_mask_rejects_before_touching_output_extent_or_writing(self):
        adapter = self.adapter()
        records, stats, result, error = (Record*6)(), active.Stats(), active.StopResult(), C.create_string_buffer(256)
        records[0].written = 123
        for mask in (0, 1, 3, 15, 63, 0xffffffff):
            status = self.lib.sda_emergency_stop_subset(adapter._session._handle, mask,
                time.monotonic_ns()+250_000_000, records, C.byref(stats), C.byref(result), error, len(error))
            self.assertEqual(status, -1); self.assertEqual(records[0].written, 123)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_cancel_and_changed_boot_do_not_block_stop_but_active_retry_stays_poisoned(self):
        adapter = self.adapter((4, 5, 6)); os.write(self.cancel_write, b'x')
        self.boot.seek(0); self.boot.write(b'00000000-0000-0000-0000-000000000000\n'); self.boot.flush()
        self.peer_loop(3)
        result = adapter.recover_subset()
        self.assertTrue(result['complete'], result)
        self.assertEqual(result['confirmed_ids'], [4, 5, 6])

    def test_missing_reply_sticks_ambiguity_and_other_selected_axes_still_attempted(self):
        adapter = self.adapter()
        self.peer_loop(3, mutate=lambda wire: None if ATParser().feed(wire)[0].destination == 2 else reply(wire))
        first = adapter.recover_subset(timeout_ns=90_000_000)
        self.assertFalse(first['complete']); self.assertEqual(first['attempted_ids'], [1, 2, 3])
        self.assertEqual(first['confirmed_ids'], [1, 3]); self.assertEqual(first['ambiguous_ids'], [2])
        self.peer_loop(3)
        second = adapter.recover_subset(timeout_ns=90_000_000)
        self.assertFalse(second['complete']); self.assertEqual(second['ambiguous_ids'], [2])
        self.assertEqual(second['confirmed_ids'], [1, 3])
        self.assertEqual([value.destination for value in self.seen], [1, 2, 3, 1, 2, 3])

    def test_active_partial_timeout_ambiguity_survives_group_recovery(self):
        adapter = self.adapter((10, 11, 12))
        self.peer_loop(3, mutate=lambda wire: None if ATParser().feed(wire)[0].destination == 12 else reply(wire))
        with self.assertRaises(active.ExchangeError) as caught:
            adapter.output_stop(deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)
        self.assertEqual(len(caught.exception.records), 3)
        self.peer_loop(3)
        result = adapter.recover_subset(timeout_ns=90_000_000)
        self.assertFalse(result['complete']); self.assertEqual(result['ambiguous_ids'], [12])
        self.assertEqual(result['confirmed_ids'], [10, 11])
        self.assertEqual(set(value.destination for value in self.seen), {10, 11, 12})

    def test_fragmented_replies_and_faults_are_retained_not_promoted(self):
        adapter = self.adapter()
        self.peer_loop(3, fragmented=True,
                       mutate=lambda wire: reply(wire, fault=4 if ATParser().feed(wire)[0].destination == 3 else 0))
        result = adapter.recover_subset()
        self.assertFalse(result['complete']); self.assertEqual(result['fault_by_id']['3'], 4)
        self.assertEqual(result['raw']['records'][2]['received'], 17)
        self.assertTrue(result['physical_cutoff_required'])

    def test_genuine_three_feedback_prepared_voltage_and_output_preserve_original_counts(self):
        adapter = self.adapter((7, 8, 9)); self.peer_loop(7, fragmented=True)
        future = Future(); future.set_running_or_notify_cancel()
        feedback, voltage = adapter.acquire(8, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)
        self.assertIs(future.result(), feedback)
        self.assertEqual(len(feedback.records), 3); self.assertEqual(len(voltage.records), 1)
        output = adapter.output_stop(deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)
        self.assertEqual(len(output.records), 3)
        self.assertEqual([value.destination for value in self.seen], [7, 8, 9, 8, 7, 8, 9])
        self.assertEqual([value.kind for value in self.seen], [4, 4, 4, 17, 4, 4, 4])

    def test_preexisting_or_foreign_feedback_future_fails_before_new_writes(self):
        adapter = self.adapter(); future = Future(); future.set_result('foreign')
        with self.assertRaisesRegex(ValueError, 'unpublished'):
            adapter.acquire(1, future, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_rebound_fd_recovery_retains_failure_without_writing_new_endpoint(self):
        adapter = self.adapter(); old = self.host.detach(); os.close(old)
        new, other = os.pipe(); self.extra_fds.extend((new, other))
        if new != old:
            os.dup2(new, old); self.extra_fds.append(old)
        result = adapter.recover_subset()
        self.assertFalse(result['complete']); self.assertEqual(result['attempted_ids'], [])
        self.assertIn('FD/budget', result['message'])

    def test_pty_three_axis_subset_uses_same_optional_abi(self):
        master, slave = pty.openpty(); tty.setraw(slave); os.set_blocking(slave, False)
        self.extra_fds.extend((master, slave))
        adapter = self.adapter((4, 5, 6), slave)
        self.peer_loop(3, fd=master, fragmented=True)
        result = adapter.recover_subset()
        self.assertTrue(result['complete'], result)
        self.assertEqual([value.destination for value in self.seen], [4, 5, 6])

    def test_source_and_optional_symbol_identity_cannot_be_replaced(self):
        foreign = C.CDLL(str(self.library_path))
        foreign._four_bus_subset_seal = self.lib._four_bus_subset_seal
        with self.assertRaisesRegex(ValueError, 'authenticated'): verify_library(foreign)
        original = self.lib.sda_emergency_stop_subset
        replacement = foreign.sda_emergency_stop_subset
        replacement.argtypes = original.argtypes; replacement.restype = original.restype
        self.lib.sda_emergency_stop_subset = replacement
        try:
            with self.assertRaisesRegex(ValueError, 'binding changed'): verify_library(self.lib)
        finally:
            self.lib.sda_emergency_stop_subset = original

    def test_whitelist_or_group_replacement_cannot_enable_before_original_call(self):
        adapter = self.adapter()
        original = adapter._stop_wires
        adapter._stop_wires = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in adapter.group.ids)
        try:
            with self.assertRaisesRegex(ValueError, 'binding changed'):
                adapter.output_stop(deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)
        finally:
            adapter._stop_wires = original
        group = adapter.group
        adapter.group = Group('port0', (4, 5, 6))
        try:
            with self.assertRaisesRegex(ValueError, 'binding changed'):
                adapter.recover_subset()
        finally:
            adapter.group = group
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def test_changed_included_source_after_authenticated_load_fails_closed(self):
        from unittest.mock import patch
        path = self.library_path.parent/'ordinary_transport.cpp'
        original = path.read_bytes()
        actual_load = active.load_library
        def changing(*args, **kwargs):
            loaded = actual_load(*args, **kwargs)
            path.write_bytes(original+b'\n// injected source mutation\n')
            return loaded
        try:
            with patch.object(active, 'load_library', side_effect=changing), \
                    self.assertRaisesRegex(ValueError, 'changed across authenticated load'):
                load_library(self.library_path, expected_sha256=sha(self.library_path),
                    ordinary_source_sha256=self.record['four_bus_subset_stop']['ordinary_source_sha256'],
                    extension_source_sha256=self.record['source_sha256'],
                    build_record_sha256=sha(self.library_path.parent/'build-record.json'))
        finally:
            path.write_bytes(original)

    def test_four_genuine_bus_owners_run_full_28_pipeline_without_extra_owner(self):
        from . import pipeline
        from .test_pipeline import Observer, profile, snapshot_builder
        from contextlib import contextmanager
        from unittest.mock import patch
        buses = {}; peers = []; threads = []; errors = []; seen = {}
        groups = tuple(Group(port, ids) for port, ids in zip(('port0','port1','port2','port3'),
                           ((7,8,9),(10,11,12),(4,5,6),(1,2,3))))
        for group in groups:
            host, peer = socket.socketpair(); host.setblocking(False)
            buses[group.port] = host; peers.append(peer); seen[group.port] = []
            ready = threading.Event()
            def serve(port=group.port, sock=peer, event=ready):
                parser = ATParser(); event.set()
                try:
                    while True:
                        if not select.select([sock], [], [], 1)[0]: return
                        raw = sock.recv(4096)
                        if not raw: return
                        for value in parser.feed(raw):
                            seen[port].append(value)
                            sock.sendall(reply(value.wire))
                except OSError: pass
                except BaseException as error: errors.append(error)
            thread = threading.Thread(target=serve); thread.start(); threads.append(thread)
            self.assertTrue(ready.wait(timeout=1))
        made = {}
        def factory(group):
            half = range(group.first_id, group.first_id+6)
            adapter = ThreeAxisTransport.create(self.lib, buses[group.port].fileno(), group=group,
                cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
                raw_lower_by_id={mid:-1. for mid in half}, raw_upper_by_id={mid:1. for mid in half})
            made[group.port] = adapter
            return adapter
        @contextmanager
        def scope(port, mask):
            yield {'native_tid': threading.get_native_id(), 'cpu_mask': list(mask),
                   'timer_slack_ns': 1000, 'file_only_mock_readback': True}
        @contextmanager
        def main_scope(): yield {'file_only_mock_readback': True}
        def imu():
            now = time.monotonic_ns()
            return {'read_started_monotonic_ns':now,'read_finished_monotonic_ns':now,
                    'accel_m_s2':(0.,0.,9.81),'gyro_rad_s':(0.,0.,0.)}
        topology = {'motor_power_epoch':'file-only-test',
                    'ids_by_port':{group.port:list(group.ids) for group in groups}}
        config = pipeline.Config(groups, profile(), {mid:0. for mid in range(1,13)}, topology, 1)
        try:
            with patch('experiments.four_bus_diagnostic.topology.validate_topology', side_effect=lambda x:x):
                result = pipeline.run(config, factory=factory, imu_read=imu, observer=Observer(),
                    snapshot_builder=snapshot_builder, check_current=lambda:None,
                    cancel_io=lambda:os.write(self.cancel_write,b'x'),
                    model_setup=lambda o,**kw:dict(kw,reset_verified=True,file_only_mock_model=True),
                    worker_scope=scope, main_scope=main_scope,
                    backend_usage={'kind':'injected_file_only_mock', 'transport':'genuine_cpp_local_sockets'},
                    release_wait=lambda ns:max(ns,time.monotonic_ns()), execute=True)
            self.assertEqual(result['status'], 'COMPLETE_STOP_PROXY_DIAGNOSTIC', result.get('primary_error'))
            self.assertEqual(result['records'][0]['actual_request_count'],28)
            for group in groups:
                self.assertEqual(len(seen[group.port]),13)  # setup3 + cycle7 + terminal3
                self.assertEqual({f.destination for f in seen[group.port]},set(group.ids))
                self.assertTrue(all(f.kind in (4,17) for f in seen[group.port]))
                self.assertTrue(result['cleanup'][group.port]['complete'])
            self.assertEqual(len({value['native_tid'] for value in result['worker_settings'].values()}),5)
        finally:
            for adapter in made.values(): adapter.close()
            for sock in buses.values(): sock.close()
            for sock in peers: sock.close()
            for thread in threads: thread.join(timeout=1)
        if errors: raise errors[0]


if __name__ == '__main__': unittest.main()
