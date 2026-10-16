"""Composite group client tests over local socket peers; never robot hardware."""
import copy
import json
import os
from pathlib import Path
import select
import socket
import tempfile
import threading
import time
import unittest
from unittest import mock

from singularitydog_hw import native_active_transport as active
from singularitydog_hw.can_readonly import ATParser
from experiments.four_bus_diagnostic.transport_adapter import Group, PORTS
from experiments.four_bus_type1 import type1_transport as T
from experiments.four_bus_type1.test_type1_transport import BOOT, bounds, caps, reply
from . import build, client


GROUPS = {'port0': Group('port0', (7, 8, 9)), 'port1': Group('port1', (10, 11, 12)),
          'port2': Group('port2', (4, 5, 6)), 'port3': Group('port3', (1, 2, 3))}


class ClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.library_path = build.build(Path(cls.temporary.name)/'group')
        scope = json.loads((cls.library_path.parent/'build-record.json').read_text())['four_bus_native_group']
        source = scope['included_source_sha256']
        cls.pins = dict(expected_sha256=T.sha(cls.library_path),
            ordinary_source_sha256=source['ordinary'],
            subset_stop_source_sha256=source['subset_stop'],
            subset_active_source_sha256=source['subset_active'],
            group_source_sha256=source['native_group'],
            build_record_sha256=T.sha(cls.library_path.parent/'build-record.json'))
        cls.lib = client.load_group_library(cls.library_path, **cls.pins)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.pairs = {}
        self.peers = []
        self.received = []
        self.stop_peers = threading.Event()
        self.cancel_read, self.cancel_write = os.pipe()
        self.boot = tempfile.TemporaryFile()
        self.boot.write((BOOT+'\n').encode()); self.boot.flush()
        self.adapters = {}
        self.group = None
        for port in PORTS:
            host, peer = socket.socketpair()
            host.setblocking(False)
            self.pairs[port] = (host, peer)
            worker = threading.Thread(target=self.peer_loop, args=(port, peer), daemon=True)
            worker.start(); self.peers.append(worker)
        for port in PORTS:
            group = GROUPS[port]
            self.adapters[port] = T.Type1Transport.create(self.lib, self.pairs[port][0].fileno(),
                group=group, cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
                axis_raw_bounds=bounds(group.ids), kp_cap_by_id=caps(group.ids, 3.),
                kd_cap_by_id=caps(group.ids, .15), cancel_all=self.cancel_all,
                separate_write_deadline=True)

    def tearDown(self):
        if self.group is not None and not self.group.closed:
            self.group.close()
        for adapter in self.adapters.values():
            adapter.close()
        self.stop_peers.set()
        for _, peer in self.pairs.values():
            peer.shutdown(socket.SHUT_RDWR)
        for worker in self.peers:
            worker.join(timeout=1)
        for host, peer in self.pairs.values():
            host.close(); peer.close()
        os.close(self.cancel_read); os.close(self.cancel_write); self.boot.close()

    def cancel_all(self):
        try:
            os.write(self.cancel_write, b'x')
        except BlockingIOError:
            pass

    def peer_loop(self, port, peer):
        parser = ATParser()
        while not self.stop_peers.is_set():
            if not select.select([peer], [], [], .1)[0]:
                continue
            try:
                raw = peer.recv(4096)
                if not raw:
                    return
                for frame in parser.feed(raw):
                    self.received.append((port, frame.wire))
                    peer.sendall(reply(frame.wire))
            except OSError:
                return

    @staticmethod
    def deadline(ms=150):
        return time.monotonic_ns()+ms*1_000_000

    def prepare(self):
        for port in PORTS:
            adapter = self.adapters[port]
            for mid in adapter.group.ids:
                adapter.enable(mid, deadline_ns=self.deadline())
                adapter.zero_gain(mid, 0., deadline_ns=self.deadline())
            wires = tuple(active.encode_motion(mid, 0., 0., 0.) for mid in adapter.group.ids)
            now = time.monotonic_ns()
            adapter.output(wires, deadline_ns=now+100_000_000, write_deadline_ns=now+50_000_000,
                           check=lambda: None)

    def ready(self):
        self.prepare()
        self.group = client.NativeGroup(self.adapters, cpu_masks=(0, 0, 0, 0),
                                        timer_slack_ns=0, socket_fixture=True)
        return self.group

    def wires_and_voltage(self):
        return ({port: self.adapters[port]._last_output for port in PORTS},
                {port: self.adapters[port].group.ids[0] for port in PORTS})

    def test_loader_requires_exact_four_source_pins_and_group_abi_seal(self):
        with self.assertRaisesRegex(ValueError, 'four native group source pins'):
            client.load_group_library(self.library_path,
                **dict(self.pins, subset_active_source_sha256='0'*64))
        self.assertEqual(client.verify_group_library(self.lib).pins,
                         tuple(self.pins[key] for key in (
                             'ordinary_source_sha256', 'subset_stop_source_sha256',
                             'subset_active_source_sha256', 'group_source_sha256')))

    def test_four_socket_owners_hold_voltage_output_and_release_lease(self):
        group = self.ready()
        hold, voltage = self.wires_and_voltage()
        initial = group.initial_hold(hold, self.deadline(), lambda: None)
        self.assertEqual(set(initial), set(PORTS))
        for port in PORTS:
            self.assertEqual(set(self.adapters[port].verify_batch(initial[port], 'output')),
                             {(mid, 'feedback') for mid in self.adapters[port].group.ids})
        with self.assertRaisesRegex(ValueError, 'active operation|original native group buffer'):
            self.adapters['port0'].publish_group_raw(
                'output', initial['port0'].records, initial['port0'].stats,
                lease=group, wires=hold['port0'], reply_deadline_ns=self.deadline())
        ticket = group.start_hold(hold, voltage, self.deadline(), lambda: None)
        self.assertGreater(ticket, 0)
        with self.assertRaisesRegex(ValueError, 'binding changed'):
            self.adapters['port0']._verify()
        feedback = group.wait_hold(ticket)
        self.assertEqual(set(feedback), set(PORTS))
        for port in PORTS:
            self.assertEqual(set(self.adapters[port].verify_batch(feedback[port], 'feedback_hold')),
                             {(mid, 'feedback') for mid in self.adapters[port].group.ids})
        volts = group.wait_voltage(ticket)
        self.assertEqual(set(volts), set(PORTS))
        for port in PORTS:
            self.assertEqual(set(self.adapters[port].verify_batch(volts[port], 'voltage')),
                             {(voltage[port], 'voltage')})
        now = time.monotonic_ns()
        output = group.output(hold, now+120_000_000, now+55_000_000, lambda: None)
        self.assertEqual(set(output), set(PORTS))
        self.assertTrue(all(batch.records[0].written == 17 for batch in output.values()))
        result = group.close()
        self.assertTrue(result['closed'])
        self.assertTrue(result['restored'])
        self.assertTrue(result['snapshot_complete'])
        self.assertIsNone(result['snapshot_error'])
        for adapter in self.adapters.values():
            self.assertTrue(adapter.poisoned)
            self.assertTrue(adapter._session.poisoned)

    def test_second_initial_hold_is_rejected_without_new_write(self):
        group = self.ready()
        hold, _ = self.wires_and_voltage()
        group.initial_hold(hold, self.deadline(), lambda: None)
        old = len(self.received)
        with self.assertRaisesRegex(ValueError, 'once before regular tickets'):
            group.initial_hold(hold, self.deadline(), lambda: None)
        self.assertEqual(len(self.received), old)
        self.assertTrue(group.poisoned)

    def test_external_cancel_and_concurrent_close_release_borrowed_sessions(self):
        group = self.ready()
        hold, voltage = self.wires_and_voltage()
        group.start_hold(hold, voltage, self.deadline(), lambda: None)
        group.cancel()
        self.assertTrue(group.poisoned)
        self.assertTrue(all(adapter.poisoned for adapter in self.adapters.values()))
        reports = []
        workers = [threading.Thread(target=lambda: reports.append(group.close())) for _ in range(2)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(timeout=2)
            self.assertFalse(worker.is_alive())
        self.assertEqual(len(reports), 2)
        self.assertEqual(reports[0], reports[1])
        self.assertTrue(all(adapter._session._phase_pair is None for adapter in self.adapters.values()))

    def test_shallow_clone_is_not_a_registered_native_lease(self):
        group = self.ready()
        impostor = copy.copy(group)
        with self.assertRaisesRegex(ValueError, 'Original native group creation'):
            impostor._bound()
        with self.assertRaisesRegex(ValueError, 'Original native group creation'):
            impostor.cancel()
        with self.assertRaisesRegex(ValueError, 'Original native group creation'):
            impostor.close()
        group._bound()

    def test_close_snapshots_unconsumed_hold_and_voltage_once_each(self):
        group = self.ready()
        hold, voltage = self.wires_and_voltage()
        ticket = group.start_hold(hold, voltage, self.deadline(), lambda: None)
        self.assertFalse(any(row['ticket'] == ticket for row in group.journal))
        report = group.close()
        self.assertTrue(report['snapshot']['complete'])
        self.assertEqual(report['snapshot']['phase_bits'] & 3, 3)
        for label in ('feedback_hold', 'voltage'):
            for port in PORTS:
                matching = [row for row in group.journal if
                            row['ticket'] == ticket and row['label'] == label and row['port'] == port]
                self.assertEqual(len(matching), 1)
                self.assertLessEqual(matching[0]['raw_count'], matching[0]['count'])
        self.assertEqual(group.close(), report)
        self.assertEqual(len(group.journal), 8)

    def test_partial_journal_excludes_untouched_native_record_slots(self):
        group = self.ready()
        result = client.GroupResult()
        result.ticket = 73
        partial = result.ports[0]
        partial.count = 3  # Issued request width, not three actual writes.
        partial.status = -1
        partial.owner_started_ns = 1
        partial.records[0].written = 17
        group._retain_partial(result, 'feedback_hold')
        self.assertEqual(len(group.journal), 1)
        evidence = group.journal[0]
        self.assertEqual(evidence['count'], 3)
        self.assertEqual(evidence['raw_count'], 1)
        self.assertEqual(len(evidence['raw'][0]), 1)
        group._retain_partial(result, 'feedback_hold')
        self.assertEqual(len(group.journal), 1)

    def test_recovered_port_image_may_change_only_collection_cancel_metadata(self):
        group = self.ready()
        result = client.GroupResult()
        result.ticket = 73
        result.collected_ns = 101
        partial = result.ports[0]
        partial.count = 3
        partial.status = -1
        partial.owner_started_ns = 1
        partial.owner_finished_ns = 2
        partial.records[0].written = 17
        group._retain_partial(result, 'feedback_hold')
        original = bytes(group.journal[0]['raw'][0]), bytes(group.journal[0]['raw'][1])
        result.collected_ns = 202
        result.cancel_requested_ns = 203
        group._retain_partial(result, 'feedback_hold')
        self.assertEqual(len(group.journal), 1)
        self.assertEqual(original, (bytes(group.journal[0]['raw'][0]), bytes(group.journal[0]['raw'][1])))
        self.assertEqual(group.raw_snapshot_conflicts, [])

    def test_recovered_raw_status_count_and_owner_clocks_must_match_original(self):
        group = self.ready()
        changes = {
            'tx_byte': lambda row: setattr(row.records[0], 'tx',
                type(row.records[0].tx).from_buffer_copy(b'x'+bytes(row.records[0].tx)[1:])),
            'stats_clock': lambda row: setattr(row.stats, 'begin_ns', 99),
            'status': lambda row: setattr(row, 'status', -2),
            'count': lambda row: setattr(row, 'count', 2),
            'owner_start': lambda row: setattr(row, 'owner_started_ns', 99),
            'owner_end': lambda row: setattr(row, 'owner_finished_ns', 99),
            'native_error': lambda row: setattr(row, 'error', b'changed native error'),
        }
        for ticket, (name, change) in enumerate(changes.items(), 80):
            with self.subTest(field=name):
                result = client.GroupResult()
                result.ticket = ticket
                partial = result.ports[0]
                partial.count = 3
                partial.status = -1
                partial.owner_started_ns = 1
                partial.owner_finished_ns = 2
                partial.records[0].written = 17
                group._retain_partial(result, 'feedback_hold')
                original = group.journal[-1]
                image = bytes(original['raw'][0]), bytes(original['raw'][1])
                before = len(group.journal)
                change(partial)
                with self.assertRaisesRegex(ValueError, 'repeated raw image differs'):
                    group._retain_partial(result, 'feedback_hold')
                self.assertEqual(len(group.journal), before)
                self.assertIs(group.journal[-1], original)
                self.assertEqual(image, (bytes(original['raw'][0]), bytes(original['raw'][1])))
                conflict = group.raw_snapshot_conflicts[-1]
                self.assertFalse(conflict['admission_eligible'])
                self.assertTrue(conflict['original_journal_retained'])
                self.assertNotEqual(conflict['expected'], conflict['actual'])

    def test_close_conflicting_snapshot_retains_originals_joins_and_allows_only_stop(self):
        group = self.ready()
        hold, voltage = self.wires_and_voltage()
        ticket = group.start_hold(hold, voltage, self.deadline(), lambda: None)
        group.wait_hold(ticket)
        group.wait_voltage(ticket)
        original_journal = tuple(group.journal)
        original_images = [(bytes(row['raw'][0]), bytes(row['raw'][1])) for row in original_journal]
        retain = group._retain_partial
        def conflicting_snapshot(result, label):
            if label == 'feedback_hold':
                result.ports[0].records[0].tx[0] ^= 1
            retain(result, label)
        with mock.patch.object(group, '_retain_partial', side_effect=conflicting_snapshot):
            report = group.close()
        self.assertTrue(report['closed'])
        self.assertTrue(report['restored'])
        self.assertFalse(report['snapshot_complete'])
        self.assertIn('repeated raw image differs', report['snapshot_error'])
        self.assertEqual(len(report['raw_snapshot_conflicts']), 1)
        self.assertEqual(tuple(group.journal), original_journal)
        self.assertEqual([(bytes(row['raw'][0]), bytes(row['raw'][1])) for row in group.journal],
                         original_images)
        json.dumps(report, allow_nan=False)
        for adapter in self.adapters.values():
            self.assertTrue(adapter._session.poisoned)
            self.assertIsNone(adapter._session._phase_pair)
            self.assertFalse(adapter._session.busy.locked())
            with self.assertRaisesRegex(RuntimeError, 'poisoned'):
                adapter._session.exchange(adapter._last_output, deadline_ns=self.deadline())
            stop = adapter.stop_repeated()
            self.assertTrue(stop['complete'])
            self.assertEqual(sorted(stop['confirmed_ids']), list(adapter.group.ids))

    def test_close_waits_for_provisional_constructor_and_never_loses_lease(self):
        self.prepare()
        candidate = client.NativeGroup.__new__(client.NativeGroup)
        entered, release = threading.Event(), threading.Event()
        original = client._settings_rows
        def delayed_settings(rows):
            entered.set()
            if not release.wait(2):
                raise TimeoutError('Constructor test was not released')
            return original(rows)
        constructor_errors, reports = [], []
        def construct():
            try:
                client.NativeGroup.__init__(candidate, self.adapters, cpu_masks=(0, 0, 0, 0),
                    timer_slack_ns=0, socket_fixture=True)
            except BaseException as error:
                constructor_errors.append(error)
        def close():
            reports.append(candidate.close())
        with mock.patch.object(client, '_settings_rows', side_effect=delayed_settings):
            create_thread = threading.Thread(target=construct)
            create_thread.start()
            self.assertTrue(entered.wait(2))
            self.assertTrue(all(adapter._session._phase_pair is candidate
                                for adapter in self.adapters.values()))
            with self.assertRaisesRegex(RuntimeError, 'borrowing native pair'):
                self.adapters['port0']._session.close()
            close_thread = threading.Thread(target=close)
            close_thread.start()
            release.set()
            create_thread.join(timeout=2)
            close_thread.join(timeout=2)
        self.group = candidate
        self.assertFalse(create_thread.is_alive())
        self.assertFalse(close_thread.is_alive())
        self.assertFalse(constructor_errors)
        self.assertTrue(reports[0]['closed'])
        self.assertTrue(all(adapter._session._phase_pair is None
                            for adapter in self.adapters.values()))

    def test_constructor_post_create_exception_joins_before_rollback(self):
        self.prepare()
        candidate = client.NativeGroup.__new__(client.NativeGroup)
        with mock.patch.object(client, '_settings_rows', side_effect=RuntimeError('format failure')):
            with self.assertRaisesRegex(RuntimeError, 'format failure'):
                client.NativeGroup.__init__(candidate, self.adapters,
                    cpu_masks=(0, 0, 0, 0), timer_slack_ns=0, socket_fixture=True)
        self.assertTrue(candidate.closed)
        self.assertIsNone(candidate._handle)
        self.assertTrue(all(adapter._session._phase_pair is None
                            for adapter in self.adapters.values()))
        self.assertTrue(all(not adapter._session.busy.locked()
                            for adapter in self.adapters.values()))

    def test_wrong_ticket_poison_and_close_before_original_stop(self):
        group = self.ready()
        hold, voltage = self.wires_and_voltage()
        ticket = group.start_hold(hold, voltage, self.deadline(), lambda: None)
        with self.assertRaisesRegex(ValueError, 'ticket'):
            group.wait_hold(ticket+1)
        self.assertTrue(group.poisoned)
        result = group.close()
        self.assertTrue(result['closed'])
        self.assertTrue(all(adapter._session._phase_pair is None for adapter in self.adapters.values()))

    def test_bad_port_output_rejected_before_any_new_write(self):
        group = self.ready()
        hold, voltage = self.wires_and_voltage()
        ticket = group.start_hold(hold, voltage, self.deadline(), lambda: None)
        group.wait_hold(ticket)
        group.wait_voltage(ticket)
        old = len(self.received)
        bad = dict(hold)
        bad['port1'] = (b'\0'*17,)*3
        now = time.monotonic_ns()
        with self.assertRaisesRegex(ValueError, 'Type1|canonical|outside'):
            group.output(bad, now+120_000_000, now+55_000_000, lambda: None)
        self.assertEqual(len(self.received), old)
        self.assertTrue(group.poisoned)

    def test_expired_output_write_cap_retains_initial_ack_and_original_error(self):
        group = self.ready()
        hold, voltage = self.wires_and_voltage()
        group.initial_hold(hold, self.deadline(), lambda: None)
        original_initial = {port: next(row for row in group.journal if
            row['ticket'] == 0 and row['label'] == 'output' and row['port'] == port)
            for port in PORTS}
        original_images = {port: (bytes(row['raw'][0]), bytes(row['raw'][1]))
            for port, row in original_initial.items()}
        ticket = group.start_hold(hold, voltage, self.deadline(), lambda: None)
        group.wait_hold(ticket)
        group.wait_voltage(ticket)
        now = time.monotonic_ns()
        with self.assertRaisesRegex(RuntimeError, 'now<write_deadline'):
            group.output(hold, now+120_000_000, now-1, lambda: None)
        self.assertTrue(group.poisoned)
        self.assertEqual(len(group.unissued_phase_failures), 1)
        failure = group.unissued_phase_failures[0]
        self.assertEqual(failure['expected_ticket'], ticket)
        self.assertEqual(failure['result_ticket'], ticket)
        self.assertEqual(failure['released_ns'], 0)
        self.assertFalse(failure['issued'])
        self.assertFalse(failure['raw_observed'])
        self.assertIn('now<write_deadline', failure['native_error'])
        self.assertEqual([row for row in group.journal if
            row['ticket'] == ticket and row['label'] == 'output'], [])
        report = group.close()
        self.assertTrue(report['snapshot_complete'])
        self.assertEqual(report['snapshot']['phase_bits'] & 7, 3)
        self.assertEqual(report['raw_snapshot_conflicts'], [])
        self.assertEqual(report['unissued_phase_failures'], [failure])
        for port, original in original_initial.items():
            self.assertIs(next(row for row in group.journal if
                row['ticket'] == 0 and row['label'] == 'output' and row['port'] == port), original)
            self.assertEqual((bytes(original['raw'][0]), bytes(original['raw'][1])), original_images[port])
            self.assertTrue(self.adapters[port].stop_repeated()['complete'])


if __name__ == '__main__':
    unittest.main()
