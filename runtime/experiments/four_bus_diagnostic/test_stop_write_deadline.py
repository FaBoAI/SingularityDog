"""Actual C++ STOP proxy cutoffs over anonymous sockets; no motor hardware."""
import ctypes as C
import json
import math
import os
from pathlib import Path
import select
import sys
import tempfile
import time
import unittest

from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import ATParser, read_request
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from singularitydog_hw.rs05_trial_protocol import TrialPhase, enable_request
from . import build
from . import transport_adapter as T
from . import test_subset_stop as fixtures


def pins_for(path):
    record = json.loads((path.parent / 'build-record.json').read_bytes())
    return dict(expected_sha256=T.sha(path),
                ordinary_source_sha256=record['four_bus_subset_stop']['ordinary_source_sha256'],
                extension_source_sha256=record['source_sha256'],
                build_record_sha256=T.sha(path.parent / 'build-record.json'))


class StopWriteDeadlineTests(unittest.TestCase):
    setUp = fixtures.SubsetStopTests.setUp
    tearDown = fixtures.SubsetStopTests.tearDown
    peer_loop = fixtures.SubsetStopTests.peer_loop

    @classmethod
    def setUpClass(cls):
        cls.original_switch = sys.getswitchinterval()
        sys.setswitchinterval(.0001)
        cls.directory = tempfile.TemporaryDirectory()
        cls.library_path = build.build(Path(cls.directory.name) / 'build')
        cls.lib = T.load_library(cls.library_path, **pins_for(cls.library_path))
        directory = Path(cls.directory.name) / 'third-write-cutoff'
        directory.mkdir()
        source = build.ORDINARY.read_text()
        marker = 'if(sent<count&&sent-done<effective_window&&t>=next) {'
        injected = marker + (
            '\n            if(write_deadline_ns&&sent==2) {'
            'const uint64_t injected_until=write_deadline_ns+1000000ULL;'
            'while(true) {const uint64_t current=now();if(current>=injected_until)break;'
            'const uint64_t remaining=injected_until-current;'
            'timespec injected_wait{time_t(remaining/1000000000ULL),long(remaining%1000000000ULL)};'
            'nanosleep(&injected_wait,nullptr);}}')
        if marker not in source:
            raise AssertionError('Native third-write fixture marker missing')
        ordinary = directory / 'third_write.cpp'
        ordinary.write_text(source.replace(marker, injected, 1))
        cls.cutoff_path = build.build(directory / 'build', ordinary_source=ordinary)
        cls.cutoff_lib = T.load_library(cls.cutoff_path, **pins_for(cls.cutoff_path))

    @classmethod
    def tearDownClass(cls):
        try:
            sys.setswitchinterval(math.nextafter(cls.original_switch, math.inf))
        finally:
            cls.directory.cleanup()

    def adapter(self, ids=(1, 2, 3), *, lib=None, selected=True, fd=None):
        group = T.Group('port0', ids)
        half = range(group.first_id, group.first_id + 6)
        result = T.ThreeAxisTransport.create(self.lib if lib is None else lib,
            self.host.fileno() if fd is None else fd, group=group,
            cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=fixtures.BOOT,
            raw_lower_by_id={mid: -1. for mid in half}, raw_upper_by_id={mid: 1. for mid in half},
            separate_write_deadline=selected)
        self.adapters.append(result)
        return result

    def silent(self):
        self.assertFalse(select.select([self.peer], [], [], 0)[0])

    def direct(self, adapter, wires, *, reply_cap, write_cap, mask=None, send_only=0):
        raw = (C.c_ubyte * (17 * len(wires))).from_buffer_copy(b''.join(wires))
        records, stats, error = (Record * len(wires))(), active.Stats(), C.create_string_buffer(256)
        status = adapter._library.sda_subset_stop_exchange_write_deadline(
            adapter._session._handle, adapter.group.mask if mask is None else mask,
            raw, len(wires), send_only, reply_cap, write_cap,
            records, C.byref(stats), error, len(error))
        return status, error.value.decode(), records, stats

    def test_writes_before_20ms_accept_ack_at_22ms_before_30hz_reply_cap(self):
        adapter = self.adapter()
        start = time.monotonic_ns()
        write_cap, reply_cap = start + 20_000_000, start + 33_333_334
        def response(wire):
            if ATParser().feed(wire)[0].destination == adapter.group.ids[-1]:
                remaining = start + 22_000_000 - time.monotonic_ns()
                if remaining > 0:
                    time.sleep(remaining / 1e9)
            return fixtures.reply(wire)
        self.peer_loop(3, mutate=response)
        batch = adapter.output_stop(deadline_ns=reply_cap, write_deadline_ns=write_cap, check=lambda: None)
        self.assertEqual([r.written for r in batch.records], [17] * 3)
        self.assertEqual([r.received for r in batch.records], [17] * 3)
        self.assertTrue(all(r.start_ns <= r.finish_ns < write_cap for r in batch.records))
        self.assertTrue(all(r.deadline_ns == reply_cap for r in batch.records))
        self.assertGreater(batch.records[-1].received_ns, write_cap)
        self.assertLess(batch.records[-1].received_ns, reply_cap)
        self.assertEqual(adapter.verify_batch(batch, 'output_stop'), batch.rows)
        self.assertEqual(batch.evidence()['separate_write_deadline_monotonic_ns'], write_cap)
        self.assertEqual(adapter.write_deadline_journal[-1][:3], (0, reply_cap, write_cap))
        self.assertLess(adapter.write_deadline_journal[-1][3], write_cap)
        self.thread.join(timeout=1)
        self.assertFalse(adapter._session.poisoned)

    def test_all_four_exact_groups_have_correct_mask_coverage(self):
        for ids in T.GROUPS:
            adapter = self.adapter(ids)
            self.peer_loop(3)
            now = time.monotonic_ns()
            batch = adapter.output_stop(deadline_ns=now + 33_333_334,
                write_deadline_ns=now + 20_000_000, check=lambda: None)
            self.thread.join(timeout=1)
            self.assertEqual([ATParser().feed(bytes(r.tx))[0].destination for r in batch.records], list(ids))
            self.assertEqual([r.received for r in batch.records], [17] * 3)
            adapter.close()
        self.assertTrue(all(value.kind == 4 for value in self.seen))

    def test_late_reply_poisons_and_remains_ambiguous_during_cleanup(self):
        adapter = self.adapter()
        start = time.monotonic_ns()
        def response(wire):
            if ATParser().feed(wire)[0].destination == 3:
                remaining = start + 42_000_000 - time.monotonic_ns()
                if remaining > 0:
                    time.sleep(remaining / 1e9)
            return fixtures.reply(wire)
        self.peer_loop(3, mutate=response)
        with self.assertRaises(active.ExchangeError) as caught:
            adapter.output_stop(deadline_ns=start + 33_333_334,
                write_deadline_ns=start + 20_000_000, check=lambda: None)
        self.thread.join(timeout=1)
        raw = caught.exception
        self.assertEqual([r.written for r in raw.records], [17] * 3)
        self.assertEqual(raw.records[-1].received, 0)
        self.assertIs(adapter.journal[-1][1][0], raw.records)
        self.assertTrue(adapter._session.poisoned)
        self.peer_loop(3)
        result = adapter.recover_subset(timeout_ns=90_000_000)
        self.assertFalse(result['complete'])
        self.assertEqual(result['ambiguous_ids'], [3])
        self.assertEqual(result['confirmed_ids'], [1, 2])
        self.assertGreater(result['raw']['rejected_total_bytes'], 0)
        self.thread.join(timeout=1)

    def test_native_before_third_write_cutoff_refuses_unsent_frame(self):
        adapter = self.adapter(lib=self.cutoff_lib)
        self.peer_loop(2)
        now = time.monotonic_ns()
        with self.assertRaisesRegex(active.ExchangeError, 'write deadline') as caught:
            adapter.output_stop(deadline_ns=now + 33_333_334,
                write_deadline_ns=now + 20_000_000, check=lambda: None)
        records, stats = caught.exception.records, caught.exception.stats
        self.assertEqual(stats.writes, 2)
        self.assertEqual([r.written for r in records], [17, 17, 0])
        self.assertTrue(all(r.finish_ns < now + 20_000_000 for r in records if r.written))
        self.assertEqual(adapter.write_deadline_journal[-1][1:3], (now + 33_333_334, now + 20_000_000))
        self.thread.join(timeout=1)
        self.assertEqual([value.destination for value in self.seen], [1, 2])
        self.silent()
        with self.assertRaisesRegex(RuntimeError, 'poisoned'):
            adapter.output_stop(deadline_ns=time.monotonic_ns() + 33_333_334,
                write_deadline_ns=time.monotonic_ns() + 20_000_000, check=lambda: None)

    def test_native_post_write_crossing_fails_and_stays_poisoned(self):
        directory = Path(self.directory.name) / f'post-write-{time.monotonic_ns()}'
        directory.mkdir()
        source = build.ORDINARY.read_text()
        marker = 'const ssize_t n=write(s->fd,r.tx,17);r.finish_ns=now();s->last_finish=r.finish_ns;++stats->writes;'
        injected = ('const ssize_t n=write(s->fd,r.tx,17);'
                    'timespec injected_delay{0,22000000};nanosleep(&injected_delay,nullptr);'
                    'r.finish_ns=now();s->last_finish=r.finish_ns;++stats->writes;')
        self.assertIn(marker, source)
        ordinary = directory / 'post_write.cpp'
        ordinary.write_text(source.replace(marker, injected, 1))
        path = build.build(directory / 'build', ordinary_source=ordinary)
        adapter = self.adapter(lib=T.load_library(path, **pins_for(path)))
        self.peer_loop(1)
        now = time.monotonic_ns()
        status, message, records, stats = self.direct(adapter, adapter._stop_wires,
            reply_cap=now + 100_000_000, write_cap=now + 20_000_000)
        self.assertEqual(status, -1)
        self.assertIn('Late active write exceeded separate write deadline', message)
        self.assertEqual(stats.writes, 1)
        self.assertEqual([r.written for r in records], [17, 0, 0])
        self.assertLess(records[0].start_ns, now + 20_000_000)
        self.assertGreaterEqual(records[0].finish_ns, now + 20_000_000)
        self.thread.join(timeout=1)
        later = time.monotonic_ns()
        status, message, _, stats = self.direct(adapter, adapter._stop_wires,
            reply_cap=later + 100_000_000, write_cap=later + 20_000_000)
        self.assertEqual(status, -1)
        self.assertIn('poisoned', message)
        self.assertEqual(stats.writes, 0)
        self.peer_loop(3)
        result = adapter.recover_subset(timeout_ns=90_000_000)
        self.assertFalse(result['complete'])
        self.assertEqual(result['ambiguous_ids'], [1])
        self.thread.join(timeout=1)

    def test_native_rejects_noncanonical_kinds_mask_order_and_send_only_before_writes(self):
        stops = tuple(stop_wire(mid) for mid in (1, 2, 3))
        corrupt = bytearray(stops[0]); corrupt[7] = 1
        cases = (
            (tuple(active.encode_motion(mid, 0., 0., 0.) for mid in (1, 2, 3)), 7, 0),
            (tuple(enable_request(phase=TrialPhase.ENABLE, motor_id=mid) for mid in (1, 2, 3)), 7, 0),
            (tuple(read_request(mid, 'voltage') for mid in (1, 2, 3)), 7, 0),
            (tuple(read_request(mid) for mid in (1, 2, 3)), 7, 0),
            ((bytes(corrupt), stops[1], stops[2]), 7, 0),
            ((stops[0], stops[2], stops[1]), 7, 0),
            ((stops[0], stops[0], stops[2]), 7, 0),
            ((stops[0],), 7, 0), (stops, 56, 0), (stops, 63, 0), (stops, 7, 1))
        for wires, mask, send_only in cases:
            with self.subTest(mask=mask, kinds=[w[2] for w in wires], send_only=send_only):
                adapter = self.adapter()
                now = time.monotonic_ns()
                status, _, records, stats = self.direct(adapter, wires, mask=mask, send_only=send_only,
                    reply_cap=now + 33_333_334, write_cap=now + 20_000_000)
                self.assertEqual(status, -1)
                self.assertEqual(stats.writes, 0)
                self.assertTrue(all(r.written == 0 for r in records))
                self.silent(); adapter.close()

    def test_native_rejects_invalid_deadlines_before_writes(self):
        for case in ('zero', 'expired', 'after_reply', 'too_far'):
            adapter = self.adapter()
            now = time.monotonic_ns()
            write_cap = {'zero': 0, 'expired': now - 1, 'after_reply': now + 40_000_000,
                         'too_far': now + 20_000_000}[case]
            reply_cap = now + (300_000_000 if case == 'too_far' else 33_333_334)
            status, message, records, stats = self.direct(adapter, adapter._stop_wires,
                reply_cap=reply_cap, write_cap=write_cap)
            self.assertEqual(status, -1)
            self.assertIn('Separate STOP deadline', message)
            self.assertEqual(stats.writes, 0)
            self.assertTrue(all(r.written == 0 for r in records))
            self.silent(); adapter.close()

    def test_cancel_boot_and_fd_rebinding_still_reject_before_writes(self):
        for case in ('cancel', 'boot', 'fd'):
            adapter = self.adapter()
            if case == 'cancel':
                os.write(self.cancel_write, b'x')
            elif case == 'boot':
                self.boot.seek(0); self.boot.write(b'99999999-2222-3333-4444-555555555555\n'); self.boot.flush()
            else:
                # Keep the caller FD valid but change its original inode binding.
                old = self.host.detach(); os.close(old)
                new, other = os.pipe(); self.extra_fds.extend((new, other))
                if new != old:
                    os.dup2(new, old); self.extra_fds.append(old)
            now = time.monotonic_ns()
            with self.assertRaises(active.ExchangeError) as caught:
                adapter.output_stop(deadline_ns=now + 33_333_334,
                    write_deadline_ns=now + 20_000_000, check=lambda: None)
            self.assertEqual(caught.exception.stats.writes, 0)
            self.assertTrue(adapter._session.poisoned)
            if case != 'fd':
                self.silent()
                self.peer_loop(3)
                self.assertTrue(adapter.recover_subset(timeout_ns=90_000_000)['complete'])
                self.thread.join(timeout=1)
            adapter.close()
            if case == 'cancel':
                os.read(self.cancel_read, 4096)
            elif case == 'boot':
                self.boot.seek(0); self.boot.write((fixtures.BOOT + '\n').encode()); self.boot.flush()

    def test_legacy_output_has_only_its_original_deadline_and_no_new_proof(self):
        adapter = self.adapter(selected=False)
        self.peer_loop(3)
        cap = time.monotonic_ns() + 20_000_000
        batch = adapter.output_stop(deadline_ns=cap, check=lambda: None)
        self.assertTrue(all(r.deadline_ns == cap for r in batch.records))
        self.assertIsNone(batch.write_deadline_ns)
        self.assertNotIn('separate_write_deadline_monotonic_ns', batch.evidence())
        self.assertEqual(adapter.write_deadline_journal, [])
        self.thread.join(timeout=1)


if __name__ == '__main__':
    unittest.main()
