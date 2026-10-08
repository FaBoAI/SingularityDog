"""Isolated STOP-only candidate on local sockets; placement is explicitly mocked.

No robot, serial device, network or hardware qualification. The Linux-only
placement evidence is exercised as a contract, not claimed by these tests.
"""
import hashlib
import importlib.util
import contextlib
import copy
import gc
from concurrent.futures import Future
import io
import json
import os
from pathlib import Path
import select
import shutil
import socket
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_pipeline_benchmark as benchmark
from singularitydog_hw.can_readonly import ATParser, read_request
from test_native_active_transport import BOOT, reply


class DisabledDiagnosticResultTests(unittest.TestCase):
    def make_result(self, mid, *, tx=None, rx=None, received=17):
        records = (native.Record*1)(); record = records[0]
        wire = native.stop_wire(mid) if tx is None else tx
        record.tx[:] = wire; record.rx[:] = reply(wire) if rx is None else rx
        record.start_ns = 11; record.finish_ns = 12; record.read_start_ns = 13
        record.received_ns = 14 if received else 0; record.deadline_ns = 20
        record.written = 17; record.received = received
        stats = active.Stats(); stats.writes = 1; stats.bytes = received
        stats.rejected[:11] = b'ATpartial11'; stats.rejected_size = 11
        stats.rejected_total = 4111
        return records, stats

    def test_all_twelve_stop_headers_preserve_raw_records_and_diagnostic_stats(self):
        for mid in range(1, 13):
            with self.subTest(mid=mid):
                records, stats = self.make_result(mid)
                before = bytes(records), bytes(stats)
                result, normalized = benchmark._disabled_diagnostic_result(records, stats)
                self.assertIs(result, records); self.assertIs(type(normalized), native.Stats)
                self.assertEqual(bytes(result), before[0]); self.assertEqual(bytes(stats), before[1])
                self.assertEqual(normalized.writes, 1); self.assertEqual(normalized.bytes, 17)
                self.assertEqual(bytes(normalized.rejected[:11]), b'ATpartial11')
                self.assertEqual(normalized.rejected_total, 4111)
                self.assertEqual(bytes(records[0].tx), native.stop_wire(mid))
                again, unchanged_stats = benchmark._disabled_diagnostic_result(records, normalized)
                self.assertIs(again, records); self.assertIs(unchanged_stats, normalized)

    def test_every_motor_rejects_fault_mode_source_destination_and_header_changes(self):
        for mid in range(1, 13):
            wire = native.stop_wire(mid)
            valid = reply(wire)
            header_variants = {'fault':reply(wire, fault=1), 'mode':reply(wire, mode=2),
                'source':reply(wire, source=mid % 12 + 1), 'destination':reply(wire, host=0xfe),
                'type':valid[:2]+bytes([valid[2] ^ 0x80])+valid[3:],
                'flags':valid[:5]+bytes([valid[5] ^ 4])+valid[6:],
                'dlc':valid[:6]+b'\x07'+valid[7:], 'prefix':b'XX'+valid[2:]}
            for name, bad in header_variants.items():
                with self.subTest(mid=mid, mutation=name):
                    records, stats = self.make_result(mid, rx=bad)
                    before = bytes(records)
                    with self.assertRaisesRegex(native.ExchangeError, 'fault/mode/source') as caught:
                        benchmark._disabled_diagnostic_result(records, stats)
                    self.assertIs(caught.exception.records, records)
                    self.assertEqual(bytes(records), before)
                    self.assertEqual(caught.exception.stats.rejected_total, 4111)

    def test_partial_stop_failure_slots_keep_raw_bytes_and_rejected_prefix(self):
        for mid in range(1, 13):
            for written in (0, 5, 17):
                with self.subTest(mid=mid, written=written):
                    records, stats = self.make_result(mid, rx=bytes(17), received=0)
                    records[0].written = written
                    before = bytes(records), bytes(stats)
                    result, normalized = benchmark._disabled_diagnostic_result(records, stats, validate_stop=False)
                    self.assertIs(result, records); self.assertEqual(bytes(result), before[0])
                    self.assertEqual(bytes(stats), before[1]); self.assertEqual(normalized.rejected_total, 4111)
                    self.assertEqual(bytes(normalized.rejected[:11]), b'ATpartial11')
                    with self.assertRaises(native.ExchangeError):
                        benchmark._disabled_diagnostic_result(records, stats)

    def test_nonstop_or_nonexact_tx_keeps_header_only_normalizer_semantics(self):
        for mid in range(1, 13):
            mutated = bytearray(native.stop_wire(mid)); mutated[7] = 1
            for tx in (read_request(mid), read_request(mid, 'position'),
                       read_request(mid, 'voltage'), bytes(mutated)):
                with self.subTest(mid=mid, tx=tx.hex()):
                    records, stats = self.make_result(mid, tx=tx, rx=bytes(17), received=0)
                    result, _ = benchmark._disabled_diagnostic_result(records, stats)
                    self.assertIs(result, records)
            valid = reply(native.stop_wire(mid))
            # Native framing/payload checks remain elsewhere; this adapter's
            # existing contract compares exactly the first seven reply bytes.
            changed_tail = valid[:7] + bytes(10)
            records, stats = self.make_result(mid, rx=changed_tail)
            self.assertIs(benchmark._disabled_diagnostic_result(records, stats)[0], records)


class DisabledNativePairCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        base = Path(cls.directory.name)
        source = Path(active.__file__).resolve().parents[1] / 'experiments/native_active_transport'
        for name in ('transport.cpp', 'build.py'):
            shutil.copyfile(source / name, base / name)
        spec = importlib.util.spec_from_file_location('build_disabled_pair_candidate', base / 'build.py')
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        cls.library_path = builder.build()
        cls.library_sha256 = hashlib.sha256(cls.library_path.read_bytes()).hexdigest()

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.cancel_read, self.cancel_write = os.pipe()
        self.boot = tempfile.TemporaryFile()
        self.boot.write((BOOT + '\n').encode()); self.boot.flush()
        self.hosts = {}; self.peers = {}; self.threads = []; self.errors = []
        for scope in ('front', 'rear'):
            host, peer = socket.socketpair(); host.setblocking(False)
            self.hosts[scope] = host; self.peers[scope] = peer
        self.pins = {name: hashlib.sha256(path.read_bytes()).hexdigest()
                     for name, path in benchmark._disabled_pair_source_paths().items()}
        self.arguments = dict(library_path=self.library_path,
            library_sha256=self.library_sha256, source_sha256=self.pins,
            fd_by_scope={scope: host.fileno() for scope, host in self.hosts.items()},
            boot_fd_by_scope={scope: self.boot.fileno() for scope in self.hosts},
            cancel_fd=self.cancel_read, boot_id=BOOT, motor_power_epoch='local-socket-test-only')
        self.candidate = None
        # Synthetic placement readbacks let macOS exercise the coordinator.
        # They do not establish actual Linux placement or hardware readiness.
        self.placement = {scope: dict(native_tid=index + 100, cpu_mask=15,
            timer_slack_ns=1000, original_cpu_mask=31, original_timer_slack_ns=50_000,
            status=0, configured=1, restored=0)
            for index, scope in enumerate(('front', 'rear'))}
        self.restoration = {scope: {**row, 'cpu_mask': row['original_cpu_mask'],
            'timer_slack_ns': row['original_timer_slack_ns'], 'restored': 1}
            for scope, row in self.placement.items()}
        def configure(pair, cpu_ids, *, timer_slack_ns=1000, restore=False):
            self.assertEqual(tuple(cpu_ids), (0, 1, 2, 3))
            pair._settings_applied = True
            pair.owner_settings_history.append({'restore': False, 'owners': copy.deepcopy(self.placement),
                                                 'status': 0, 'error': ''})
            coordinator = {**copy.deepcopy(self.placement['front']), 'native_tid': 200, 'applied': True}
            # Populate the same public evidence shape as the Linux wrapper.
            pair._coordinator_settings = coordinator
            pair.coordinator_settings_history.append({'restore': False, 'coordinator': coordinator,
                                                       'status': 0, 'error': ''})
            return self.placement
        def restore(pair):
            pair.owner_settings_history.append({'restore': True, 'owners': copy.deepcopy(self.restoration),
                                                 'status': 0, 'error': ''})
            coordinator = {**copy.deepcopy(self.restoration['front']), 'native_tid': 200, 'applied': True}
            pair._coordinator_settings = coordinator
            pair.coordinator_settings_history.append({'restore': True, 'coordinator': coordinator,
                                                       'status': 0, 'error': ''})
            return self.restoration
        self.config_patch = patch.object(active.ActivePhasePair, 'configure_owners',
                                        autospec=True, side_effect=configure)
        self.restore_patch = patch.object(active.ActivePhasePair, 'restore_owners',
                                         autospec=True, side_effect=restore)
        self.configured = self.config_patch.start(); self.restore_patch.start()

    def tearDown(self):
        if self.candidate is not None:
            self.candidate.close()
        self.config_patch.stop(); self.restore_patch.stop()
        for host in self.hosts.values(): host.close()
        for peer in self.peers.values(): peer.close()
        for thread in self.threads: thread.join(timeout=.5)
        os.close(self.cancel_read); os.close(self.cancel_write); self.boot.close()
        if self.errors: raise self.errors[0]

    def create(self):
        self.candidate = benchmark._DisabledNativePairCandidate(**self.arguments)
        return self.candidate

    def batches(self):
        return {scope: tuple(native.stop_wire(mid) for mid in ids)
                for scope, ids in benchmark.dual.SCOPES.items()}

    def device(self, scope, count, mutate=reply):
        def run():
            parser = ATParser(); seen = 0
            try:
                while seen < count:
                    if not select.select([self.peers[scope]], [], [], .2)[0]: return
                    data = self.peers[scope].recv(4096)
                    if not data: return
                    for frame in parser.feed(data):
                        seen += 1
                        outgoing = mutate(frame.wire)
                        if outgoing: self.peers[scope].sendall(outgoing)
            except OSError:
                pass
            except BaseException as error:
                self.errors.append(error)
        thread = threading.Thread(target=run); thread.start(); self.threads.append(thread)

    def no_write(self):
        for peer in self.peers.values():
            self.assertFalse(select.select([peer], [], [], 0)[0])

    def test_bad_external_source_or_library_pin_rejects_before_any_write(self):
        for field in ('source_sha256', 'library_sha256'):
            args = dict(self.arguments)
            if field == 'source_sha256':
                args[field] = {name: 'a' * 64 for name in self.pins}
            else:
                args[field] = 'a' * 64
            with self.subTest(field=field), self.assertRaises(ValueError):
                benchmark._DisabledNativePairCandidate(**args)
            self.no_write()

    def test_unselected_gaps_reject_before_loading_native_or_writing(self):
        for gap in (850,870,879,891,880.,890.,True,None):
            with self.subTest(gap=gap),patch.object(active,'load_library') as load,\
                 self.assertRaisesRegex(ValueError,'exactly 880, 890 or 900'):
                benchmark._DisabledNativePairCandidate(**self.arguments,request_gap_us=gap)
            load.assert_not_called();self.no_write()

    def assert_explicit_gap_socket_spacing(self, gap):
        original_init=active.ActiveSession.__init__
        with patch.object(active.ActiveSession,'__init__',autospec=True,
                          side_effect=original_init) as initialize:
            self.candidate=benchmark._DisabledNativePairCandidate(**self.arguments,request_gap_us=gap)
        self.assertEqual(len(initialize.call_args_list),2)
        for call in initialize.call_args_list:
            self.assertEqual(call.kwargs['gap_ns'],gap*1000)
            self.assertEqual(call.kwargs['window'],3)
            self.assertTrue(all(value==0. for value in call.kwargs['kp_max_by_id'].values()))
            self.assertTrue(all(value==0. for value in call.kwargs['kd_max_by_id'].values()))
        self.candidate.configure_owners()
        for scope in self.peers:self.device(scope,6)
        deadline=time.monotonic_ns()+20_000_000
        for records,_ in self.candidate.exchange_stop_proxy(self.batches(),deadline_ns=deadline).values():
            self.assertTrue(all(b.start_ns-a.finish_ns>=gap*1000 for a,b in zip(records,records[1:])))
            self.assertTrue(all(row.deadline_ns==deadline and row.written==row.received==17 for row in records))
        self.assertEqual(self.candidate.evidence()['request_gap_us'],gap)
        self.assertEqual(self.candidate.evidence()['request_window'],3)

    def test_explicit_880_reaches_both_native_sessions_and_socket_write_spacing(self):
        self.assert_explicit_gap_socket_spacing(880)

    def test_explicit_890_reaches_both_native_sessions_and_socket_write_spacing(self):
        self.assert_explicit_gap_socket_spacing(890)

    def test_no_selection_or_placement_cannot_send_and_has_no_qualification_proof(self):
        candidate = self.create()
        with self.assertRaisesRegex(RuntimeError, 'verified current native owner'):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns() + 20_000_000)
        evidence = candidate.evidence()
        self.assertFalse(evidence['owner_placement_verified'])
        self.assertFalse(evidence['owner_settings_restored'])
        self.assertNotIn('native_phase_pair_proof', evidence)
        self.assertFalse(evidence['output_allowed'])
        self.no_write()

    def test_adapter_rejects_enable_motion_version_and_watchdog_without_writes(self):
        candidate = self.create()
        wire = bytearray(native.stop_wire(1))
        wire[2:6] = ((((3 << 24) | (0xfd << 8) | 1) << 3) | 4).to_bytes(4, 'big')
        version = bytearray(native.stop_wire(1)); version[7:15] = bytes.fromhex('00c4000000000000')
        forbidden = (bytes(wire), active.encode_motion(1, 0., 0., 0.), bytes(version),
                     read_request(1, 'can_timeout'))
        for wire in forbidden:
            with self.subTest(wire=wire.hex()), self.assertRaisesRegex(ValueError, 'exact Type0/17'):
                candidate.sessions['front'].exchange([wire])
            self.no_write()

    def test_invalid_owner_readback_and_nonstop_pair_never_write(self):
        candidate = self.create()
        self.placement['front']['cpu_mask'] = 31
        with self.assertRaisesRegex(RuntimeError, 'read back CPUs'):
            candidate.configure_owners()
        self.no_write()
        self.placement['front']['cpu_mask'] = 15
        with self.assertRaisesRegex(RuntimeError, 'selected once'):
            candidate.configure_owners()

    def test_allowlist_rejects_complete_pair_before_either_bus_writes(self):
        candidate = self.create(); candidate.configure_owners()
        batches = self.batches(); batches['rear'] = (read_request(7, 'position'),) + batches['rear'][1:]
        with self.assertRaisesRegex(ValueError, 'exactly twelve all-zero STOP'):
            candidate.exchange_stop_proxy(batches, deadline_ns=time.monotonic_ns() + 20_000_000)
        self.no_write()

    def test_cached_batches_preserve_exact_order_and_reject_each_motor_mutation(self):
        candidate = self.create(); candidate.configure_owners(); original = self.batches()
        for scope, ids in benchmark.dual.SCOPES.items():
            self.assertEqual(benchmark._DISABLED_PAIR_STOP_BATCHES[scope], original[scope])
            for index, mid in enumerate(ids):
                wire = bytearray(original[scope][index]); wire[7] = 1
                for replacement in (bytes(wire), bytearray(original[scope][index]),
                                    read_request(mid, 'position'), native.stop_wire(7 if scope=='front' else 1)):
                    batches = dict(original); changed = list(batches[scope]); changed[index] = replacement
                    batches[scope] = tuple(changed)
                    with self.subTest(scope=scope, mid=mid, replacement=bytes(replacement).hex()), \
                         patch.object(active.ActivePhasePair, 'submit') as submit, \
                         self.assertRaisesRegex(ValueError, 'exactly twelve all-zero STOP'):
                        candidate.exchange_stop_proxy(batches, deadline_ns=time.monotonic_ns()+20_000_000)
                    submit.assert_not_called(); self.no_write()
            for changed in (original[scope][::-1], original[scope][:-1], original[scope]+original[scope][:1]):
                batches = {**original, scope:changed}
                with self.subTest(scope=scope, count=len(changed)), \
                     patch.object(active.ActivePhasePair, 'submit') as submit, self.assertRaises(ValueError):
                    candidate.exchange_stop_proxy(batches, deadline_ns=time.monotonic_ns()+20_000_000)
                submit.assert_not_called(); self.no_write()

    def test_socket_phase_does_not_regenerate_stop_wires_after_setup(self):
        candidate = self.create(); candidate.configure_owners(); batches = self.batches()
        for scope in self.peers: self.device(scope, 6)
        deadline = time.monotonic_ns()+20_000_000
        with patch.object(native, 'stop_wire', side_effect=AssertionError('STOP regenerated during phase')):
            output = candidate.exchange_stop_proxy(batches, deadline_ns=deadline)
        for scope, (records, _) in output.items():
            self.assertEqual(tuple(bytes(row.tx) for row in records), batches[scope])
            self.assertTrue(all(row.deadline_ns == deadline and row.written == row.received == 17 for row in records))

    def test_expired_or_extended_absolute_deadline_never_writes(self):
        candidate = self.create(); candidate.configure_owners()
        for deadline in (time.monotonic_ns() - 1, time.monotonic_ns() + 100_000_000):
            with self.subTest(deadline=deadline), self.assertRaisesRegex(ValueError, 'original remaining 20ms'):
                candidate.exchange_stop_proxy(self.batches(), deadline_ns=deadline)
        self.no_write()

    def test_completed_publication_is_genuine_current_generation_and_invalidated_before_rejection(self):
        candidate = self.create(); candidate.configure_owners(); published = []
        original = active.ActivePhasePair.submit
        def retain(pair, *args, **kwargs):
            result = original(pair, *args, **kwargs); published.append(result); return result
        with self.assertRaisesRegex(RuntimeError, 'No validated current'):
            candidate.last_completed_futures()
        for scope in self.peers:self.device(scope, 6)
        with patch.object(active.ActivePhasePair, 'submit', retain):
            result = candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns()+20_000_000)
        futures = candidate.last_completed_futures()
        for scope in ('front', 'rear'):
            self.assertIs(futures[scope], published[-1][scope])
            self.assertTrue(futures[scope].done()); self.assertFalse(futures[scope].cancelled())
            self.assertIs(futures[scope].result()[0], result[scope][0])
        with self.assertRaises(TypeError):futures['front'] = Future()
        with self.assertRaises(ValueError):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns()-1)
        with self.assertRaisesRegex(RuntimeError, 'No validated current'):
            candidate.last_completed_futures()
        self.no_write()

    def test_failed_reply_does_not_publish_success_readiness(self):
        candidate = self.create(); candidate.configure_owners()
        self.device('front', 6); self.device('rear', 6, mutate=lambda wire: b'')
        with self.assertRaises(native.ExchangeError):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns()+20_000_000)
        with self.assertRaisesRegex(RuntimeError, 'No validated current'):
            candidate.last_completed_futures()
        self.assertTrue(candidate.evidence()['journal'][-1]['joined'])

    def test_external_pin_dictionary_mutation_does_not_change_frozen_binding(self):
        candidate = self.create()
        name = next(iter(self.pins)); original = self.pins[name]
        self.pins[name] = 'a' * 64
        self.assertTrue(candidate.verify_sources())
        self.assertEqual(candidate.evidence()['source_sha256'][name], original)

    def test_read_adapter_keeps_existing_prepared_hook_without_pair_dispatch(self):
        candidate = self.create(); calls = []
        self.device('front', 1)
        records, _ = candidate.sessions['front'].exchange([read_request(1, 'position')],
            before_native=lambda: calls.append('prepared-before-native'))
        self.assertEqual(calls, ['prepared-before-native'])
        self.assertEqual(records[0].received, 17)
        self.assertEqual(candidate.evidence()['journal'], [])

    def test_real_socket_phase_keeps_absolute_deadline_and_restores_same_owner_contract(self):
        candidate = self.create(); candidate.configure_owners()
        for scope in self.peers: self.device(scope, 6)
        deadline = time.monotonic_ns() + 20_000_000
        results = candidate.exchange_stop_proxy(self.batches(), deadline_ns=deadline)
        for records, _ in results.values():
            self.assertEqual(len(records), 6)
            self.assertTrue(all(b.start_ns-a.finish_ns>=900_000 for a,b in zip(records,records[1:])))
            self.assertTrue(all(row.written == row.received == 17 for row in records))
            self.assertTrue(all(row.deadline_ns == deadline for row in records))
        self.assertEqual(candidate.evidence()['request_gap_us'],900)
        candidate.close()
        evidence = candidate.evidence()
        self.assertTrue(evidence['all_phases_joined'])
        self.assertTrue(evidence['owner_settings_restored'])
        self.assertFalse(evidence['active_controller_qualification'])
        self.assertNotIn('native_phase_pair_proof', evidence)
        self.assertEqual(evidence['journal'][0]['deadline_ns'], deadline)

    def test_missing_reply_retains_both_partial_raw_results_and_poison(self):
        candidate = self.create(); candidate.configure_owners()
        self.device('front', 6); self.device('rear', 6, mutate=lambda wire: b'')
        with self.assertRaises(native.ExchangeError):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns() + 20_000_000)
        evidence = candidate.evidence(); row = evidence['journal'][0]
        self.assertTrue(row['joined'])
        self.assertEqual(set(row['raw_by_scope']), {'front', 'rear'})
        self.assertTrue(any(r['written'] == 17 for r in row['raw_by_scope']['rear']['records']))
        self.assertTrue(row['errors'])
        with self.assertRaises(RuntimeError):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns() + 20_000_000)

    def test_future_wait_exception_keeps_wait_error_and_later_completed_raw(self):
        candidate = self.create(); candidate.configure_owners()
        for scope in self.peers: self.device(scope, 6)
        original = Future.result; calls = []
        def interrupted_result(future, timeout=None):
            if not calls:
                calls.append(True)
                raise TimeoutError('Injected takeout timeout before native publication')
            return original(future, timeout=timeout)
        with patch.object(Future, 'result', interrupted_result), self.assertRaisesRegex(TimeoutError, 'takeout timeout'):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns() + 20_000_000)
        row = candidate.evidence()['journal'][0]
        self.assertIn('takeout timeout', row['errors'][0])
        self.assertEqual(set(row['raw_by_scope']), {'front', 'rear'})
        self.assertTrue(all(record['received'] == 17
            for raw in row['raw_by_scope'].values() for record in raw['records']))

    def test_pair_stop_fault_retains_raw_and_cannot_reach_legacy_decoder(self):
        candidate = self.create(); candidate.configure_owners()
        self.device('front', 6, mutate=lambda wire: reply(wire, fault=1))
        self.device('rear', 6)
        with self.assertRaises(native.ExchangeError):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns() + 20_000_000)
        row = candidate.evidence()['journal'][0]
        self.assertEqual(set(row['raw_by_scope']), {'front', 'rear'})
        self.assertTrue(row['errors'])

    def test_phase_does_not_read_source_files_inside_measured_exchange(self):
        candidate = self.create(); candidate.configure_owners()
        for scope in self.peers: self.device(scope, 6)
        with patch.object(candidate, 'verify_sources', side_effect=AssertionError('disk IO during cycle')):
            candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns() + 20_000_000)

    def test_exception_after_native_submit_retains_filled_raw_slots(self):
        candidate = self.create(); candidate.configure_owners()
        for scope in self.peers: self.device(scope, 6)
        submit = active.ActivePhasePair.submit
        def submit_then_raise(pair, *args, **kwargs):
            futures = submit(pair, *args, **kwargs)
            for future in futures.values(): future.result(timeout=.35)
            raise RuntimeError('Injected exception after both native writes')
        with patch.object(active.ActivePhasePair, 'submit', submit_then_raise):
            with self.assertRaisesRegex(RuntimeError, 'after both native writes'):
                candidate.exchange_stop_proxy(self.batches(), deadline_ns=time.monotonic_ns() + 20_000_000)
        row = candidate.evidence()['journal'][0]
        self.assertEqual(set(row['raw_by_scope']), {'front', 'rear'})
        self.assertTrue(all(record['received'] == 17
            for raw in row['raw_by_scope'].values() for record in raw['records']))

    def test_unjoined_pair_close_preserves_borrowed_sessions(self):
        candidate = self.create()
        with patch.object(active.ActivePhasePair, 'wait_idle', side_effect=TimeoutError('not joined')):
            with self.assertRaisesRegex(TimeoutError, 'not joined'):
                candidate.close()
        self.no_write()
        # A later verified join can clean up; close did not race or release FDs.
        candidate.close()

    def test_changed_restoration_tid_is_rejected_and_not_claimed_restored(self):
        candidate = self.create(); candidate.configure_owners()
        self.restoration['front']['native_tid'] += 10
        with self.assertRaisesRegex(RuntimeError, 'owner TIDs changed'):
            candidate.close()
        self.assertFalse(candidate.evidence()['owner_settings_restored'])

    def test_final_restore_failure_cannot_reuse_earlier_successful_evidence(self):
        candidate = self.create(); candidate.configure_owners()
        with patch.object(active.ActivePhasePair, 'restore_owners', side_effect=RuntimeError('final restore failure')):
            with self.assertRaisesRegex(RuntimeError, 'final restore failure'):
                candidate.close()
        evidence = candidate.evidence()
        self.assertFalse(evidence['owner_settings_restored'])
        self.assertTrue(evidence['errors'])
        self.assertTrue(candidate.fd_release_safe())

    def collect_candidate(self, *, prime=False, mutate=None, observer=None, extra_options=None):
        from test_native_pipeline_benchmark import Device, Observer
        candidate = self.candidate or self.create()
        for scope in self.peers:
            self.device(scope, 5 * 13 + (6 if prime else 0),
                mutate=(mutate or {}).get(scope, lambda wire: reply(wire, value=39.)))
        masks = {}
        def get_affinity(pid):
            return set(masks.setdefault(threading.get_native_id(), {0, 1, 2, 3, 4}))
        def set_affinity(pid, cpus):
            masks[threading.get_native_id()] = set(cpus)
        def until(deadline):
            time.sleep(max(0., (deadline - time.monotonic_ns()) / 1e9))
        options = dict(mode='stop-proxy', cycles=5, native_phase_pair_candidate=candidate,
            record_storage='trace', main_thread_cpu=4, output_dispatch_trace=True,
            defer_gc_during_cycles=True, pre_cycle_policy_prepare=lambda: None,
            post_pin_policy_prepare=lambda: None, v3_voltage_proxy=True,
            v3_voltage_overlap=True, v3_voltage_validation_overlap=True,
            v3_voltage_fast_pipeline=True, prepare_voltage_before_feedback_publication=True,
            inference_thread_cpu_trace=True, absolute_epoch_cadence=True,
            exclude_policy_cpu_from_workers=True, startup_cycle_allowance=1,
            deadline_wait=until, worker_initializer=lambda: None)
        if prime:options['native_pair_prime_before_cycles']=True
        options.update(extra_options or {})
        with patch.object(benchmark.os, 'sched_getaffinity', get_affinity, create=True),\
             patch.object(benchmark.os, 'sched_setaffinity', set_affinity, create=True):
            return benchmark.collect(candidate.sessions, Device(), observer or Observer(), **options)

    def test_collector_uses_diagnostic_stats_trace_and_original_deadline(self):
        report, raw = self.collect_candidate()
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(report['cycles_completed'], 5)
        self.assertTrue(report['native_phase_pair'])
        self.assertFalse(report['active_controller_qualification'])
        self.assertTrue(report['native_phase_pair_proof']['owner_settings_restored'])
        for row in raw:
            saved = row.serialize()
            self.assertGreater(saved['native_phase_pair_phase']['generation'], 0)
            hard_end = saved['voltage_fast_pipeline']['hard_deadline_ns']
            self.assertTrue(all(record['deadline_ns'] == hard_end
                for bus in saved['output'].values() for record in bus['records']))
        self.assertEqual(len(report['native_phase_pair_evidence']['journal']), 5)
        self.assertNotIn('native_phase_pair_prime', report)

    def test_completed_output_raw_survives_publication_getter_failure(self):
        candidate = self.create()
        with patch.object(candidate, 'last_completed_futures',
                          side_effect=RuntimeError('Injected publication metadata failure')):
            report, raw = self.collect_candidate()
        self.assertEqual(report['status'], 'ABORTED'); self.assertEqual(report['cycles_completed'], 0)
        self.assertIn('publication metadata failure', ' '.join(report['errors']))
        self.assertEqual(len(raw), 1)
        output = raw[0]['output'] if isinstance(raw[0], dict) else raw[0].serialize()['output']
        self.assertEqual(set(output), {'front', 'rear'})
        for scope, bus in output.items():
            if not isinstance(bus, dict):bus = native.exchange_evidence(*bus)
            self.assertEqual(len(bus['records']), 6)
            self.assertTrue(all(row['written'] == row['received'] == 17 for row in bus['records']))
        self.assertTrue(report['native_phase_pair_evidence']['all_phases_joined'])

    def test_explicit_prime_retains_twelve_stops_after_setup_and_before_epoch(self):
        from test_native_pipeline_benchmark import Observer
        events = []; candidate = self.create(); exchange = candidate.exchange_stop_proxy
        configure = candidate.configure_owners
        class ArmedObserver(Observer):
            _measured_diagnostic_ticks = True
            def arm_run(self, tick):events.append('armed');super().arm_run(tick)
        observer = ArmedObserver()
        def configure_tracking():
            result=configure();events.append('placed');return result
        def exchange_tracking(wires, *, deadline_ns):
            if not events or events[-1]!='phase':
                self.assertEqual(events, ['warmup', 'policy-prime', 'placed'])
                self.assertFalse(gc.isenabled())
                self.assertEqual(benchmark.os.sched_getaffinity(0), {4})
            events.append('phase')
            return exchange(wires, deadline_ns=deadline_ns)
        # Only inspect the first phase; subsequent phases follow observer arm.
        phase_calls=[]
        def first_exchange(wires, *, deadline_ns):
            if not phase_calls:result=exchange_tracking(wires, deadline_ns=deadline_ns)
            else:result=exchange(wires, deadline_ns=deadline_ns)
            phase_calls.append(True);return result
        with patch.object(candidate, 'configure_owners', configure_tracking),\
             patch.object(candidate, 'exchange_stop_proxy', first_exchange):
            report, raw = self.collect_candidate(prime=True, observer=observer,
                extra_options={'pre_cycle_policy_prepare':lambda:events.append('warmup'),
                    'post_pin_policy_prepare':lambda:events.append('policy-prime')})
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(events, ['warmup', 'policy-prime', 'placed', 'phase', 'armed'])
        self.assertEqual(observer.calls, 5)
        self.assertEqual(len(raw), report['cycles_completed'])
        self.assertEqual(report['cycles_completed'], 5)
        self.assertEqual(len(phase_calls), 6)
        prime = report['native_phase_pair_prime'];journal=report['native_phase_pair_evidence']['journal']
        self.assertTrue(prime['complete']);self.assertTrue(prime['joined'])
        self.assertEqual(prime['deadline_ns']-prime['begin_ns'], benchmark.PERIOD_NS)
        self.assertEqual(prime['raw_by_scope'], journal[0]['raw_by_scope'])
        self.assertEqual(prime['phase'], journal[0]['phase'])
        self.assertLess(prime['end_ns'], observer.first)
        self.assertLess(observer.first, report['absolute_epoch_schedule']['epoch_ns'])
        self.assertFalse(prime['counted_as_measured_cycle'])
        self.assertFalse(prime['active_controller_qualification'])
        for scope, ids in benchmark.dual.SCOPES.items():
            for mid, record in zip(ids, prime['raw_by_scope'][scope]['records']):
                self.assertEqual(record['tx_hex'], native.stop_wire(mid).hex())
                self.assertEqual(bytes.fromhex(record['rx_hex'])[:7], benchmark._STOP_REPLY_HEADERS[mid])
                self.assertEqual(record['deadline_ns'], prime['deadline_ns'])
                self.assertEqual(record['received'], 17)
        self.assertEqual(len(journal), 6)
        self.assertEqual(raw[0].serialize()['native_phase_pair_phase']['generation'],
                         prime['phase']['generation']+1)
        self.assertEqual(report['native_phase_pair_proof']['request_count_per_cycle'], 26)

    def test_prime_timeout_retains_partial_raw_and_aborts_before_any_cycle(self):
        from test_native_pipeline_benchmark import Observer
        observer=Observer()
        report, raw = self.collect_candidate(prime=True, observer=observer,
                                            mutate={'rear':lambda wire:b''})
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(observer.calls, 0);self.assertEqual(raw, [])
        prime=report['native_phase_pair_prime']
        self.assertTrue(prime['attempted']);self.assertFalse(prime['complete'])
        self.assertTrue(prime['joined']);self.assertTrue(prime['errors'])
        self.assertEqual(set(prime['raw_by_scope']), {'front', 'rear'})
        self.assertTrue(any(r['written']==17 for r in prime['raw_by_scope']['rear']['records']))
        self.assertEqual(len(report['native_phase_pair_evidence']['journal']), 1)
        self.assertTrue(report['native_phase_pair_evidence']['owner_settings_restored'])
        self.assertTrue(report['native_phase_pair_evidence']['coordinator_settings_restored'])
        self.assertNotIn('native_phase_pair_proof', report)
        self.assertNotIn('Native pair source/join/owner restoration proof incomplete', report['errors'])
        self.assertIsNone(report['absolute_epoch_schedule']['epoch_ns'])

    def test_prime_fault_reply_is_retained_and_cannot_arm_cadence(self):
        report, raw=self.collect_candidate(prime=True,
            mutate={'front':lambda wire:reply(wire, fault=1)})
        self.assertEqual(report['status'], 'ABORTED');self.assertEqual(raw, [])
        self.assertFalse(report['native_phase_pair_prime']['complete'])
        self.assertTrue(report['native_phase_pair_prime']['raw_by_scope'])
        self.assertIsNone(report['absolute_epoch_schedule']['epoch_ns'])

    def test_context_change_after_prime_retains_phase_but_does_not_arm_run(self):
        from test_native_pipeline_benchmark import Observer
        candidate=self.create();exchange=candidate.exchange_stop_proxy;finished=[]
        class ArmedObserver(Observer):
            _measured_diagnostic_ticks=True
        observer=ArmedObserver()
        def prime_exchange(wires, *, deadline_ns):
            result=exchange(wires, deadline_ns=deadline_ns);finished.append(True);return result
        def check():
            if finished:raise RuntimeError('Injected changed current boot/context')
        with patch.object(candidate, 'exchange_stop_proxy', prime_exchange):
            report, raw=self.collect_candidate(prime=True, observer=observer,
                                               extra_options={'check':check})
        self.assertEqual(report['status'], 'ABORTED');self.assertEqual(raw, [])
        self.assertFalse(hasattr(observer, 'first'))
        prime=report['native_phase_pair_prime']
        self.assertFalse(prime['complete']);self.assertTrue(prime['joined'])
        self.assertIn('changed current boot/context', prime['errors'][0])
        self.assertEqual(sum(r['received']==17 for b in prime['raw_by_scope'].values()
                             for r in b['records']), 12)
        self.assertIsNone(report['absolute_epoch_schedule']['epoch_ns'])

    def test_prime_checks_source_pin_before_any_phase_write(self):
        candidate=self.create();calls=[]
        def sources():
            calls.append(True)
            if len(calls)==2:raise ValueError('Injected changed prime source pin')
            return True
        with patch.object(candidate, 'verify_sources', sources):
            report, raw=self.collect_candidate(prime=True)
        self.assertEqual(report['status'], 'ABORTED');self.assertEqual(raw, [])
        self.assertFalse(report['native_phase_pair_prime']['attempted'])
        self.assertIn('changed prime source pin', report['native_phase_pair_prime']['errors'][0])
        self.assertEqual(report['native_phase_pair_evidence']['journal'], [])
        self.no_write()

    def test_prime_requires_explicit_exact_candidate_before_any_device_read(self):
        with self.assertRaisesRegex(ValueError, 'explicit disabled native pair'):
            benchmark.collect({}, None, None, mode='stop-proxy', cycles=5,
                              native_pair_prime_before_cycles=True)
        self.no_write()


class DisabledNativePairCLITests(unittest.TestCase):
    def setUp(self):
        from test_native_pipeline_active_fk import ActiveFKDiagnosticTests
        self.fixture = ActiveFKDiagnosticTests(); self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.data.update(native_phase_pair=True, request_gap_us=900, request_window=3)
        self.fixture.path = self.fixture.fixture.fixture.seal()
        digest = hashlib.sha256(self.fixture.path.read_bytes()).hexdigest()
        self.argv = self.fixture.argv[:]
        self.argv[self.argv.index('--active-fk-profile-sha256') + 1] = digest
        self.argv[self.argv.index('--request-gap-us') + 1] = '900'
        self.library = self.fixture.base / 'plan-only-library.so'
        self.library.write_bytes(b'SYNTHETIC PLAN-ONLY LIBRARY; NEVER LOAD')
        self.inventory = self.fixture.base / 'pair-sources.json'
        self.inventory.write_text(json.dumps({'schema': 'singularitydog.disabled-native-pair-sources.v1',
            'source_sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest()
                             for name, path in benchmark._disabled_pair_source_paths().items()}}))
        self.argv += ['--native-phase-pair', '--native-pair-active-library', str(self.library),
            '--native-pair-active-library-sha256', hashlib.sha256(self.library.read_bytes()).hexdigest(),
            '--native-pair-source-inventory', str(self.inventory),
            '--native-pair-source-inventory-sha256', hashlib.sha256(self.inventory.read_bytes()).hexdigest(),
            '--main-thread-cpu', '4', '--exclude-policy-cpu-from-workers', '--timer-slack-ns', '1000',
            '--release-spin-us', '500', '--pre-cycle-policy-warmup-calls', '10',
            '--post-pin-policy-prime-calls', '10', '--setup-gc', 'before-warmup',
            '--defer-gc-during-cycles', '--single-thread-math', '--require-pinned-fast-model',
            '--output-dispatch-trace', '--inference-thread-cpu-trace', '--absolute-epoch-cadence']

    def invoke(self, argv):
        output = io.StringIO()
        with patch.object(benchmark.os, 'sched_getaffinity', return_value={0, 1, 2, 3, 4}, create=True),\
             patch.object(benchmark.os, 'sched_setaffinity', create=True),\
             patch.dict(os.environ, {}, clear=True), contextlib.redirect_stdout(output):
            result = benchmark.main(argv)
        return result, json.loads(output.getvalue())

    def replace(self, name, value):
        argv = self.argv[:]; argv[argv.index(name) + 1] = value
        return argv

    def select_gap(self,gap):
        self.fixture.data['request_gap_us']=gap
        self.fixture.path=self.fixture.fixture.fixture.seal()
        self.argv[self.argv.index('--active-fk-profile-sha256')+1]=hashlib.sha256(
            self.fixture.path.read_bytes()).hexdigest()
        self.argv[self.argv.index('--request-gap-us')+1]=str(gap)

    def test_selected_5_and_501_plans_retain_exact_pacing_without_loading_native_model_or_device(self):
        from singularitydog_hw import policy_active_fk
        for gap in (880,890,900):
            self.select_gap(gap)
            for cycles in (5,501):
                with self.subTest(gap=gap,cycles=cycles),patch.object(active,'load_library') as load,\
                     patch.object(benchmark.native,'load_library') as diagnostic,\
                     patch.object(policy_active_fk,'diagnostic_load') as model,\
                     patch.object(benchmark.imu,'ICM20948') as device:
                    result,plan=self.invoke(self.replace('--cycles',str(cycles))+['--native-pair-prime-before-cycles'])
                self.assertEqual(result,0)
                for loader in (load,diagnostic,model,device):loader.assert_not_called()
                self.assertEqual(plan['cycles'],cycles)
                self.assertEqual(plan['request_gap_us'],gap);self.assertEqual(plan['window'],3)
                self.assertEqual(plan['native_phase_pair_plan']['request_gap_us'],gap)
                self.assertEqual(plan['native_phase_pair_plan']['request_window'],3)
                self.assertEqual(plan['native_phase_pair_plan']['pre_cycle_stop_prime']['deadline_budget_ns'],20_000_000)
                self.assertFalse(plan['output_allowed']);self.assertEqual(plan['requests_per_cycle'],26)
                self.assertEqual(plan['type1_requests_per_cycle'],0)

    def test_all_selected_gaps_reject_mismatched_profile_and_cli_before_native_load(self):
        for selected,requested in ((a,b) for a in (880,890,900) for b in (880,890,900) if a!=b):
            self.select_gap(selected)
            with self.subTest(selected=selected,requested=requested),contextlib.redirect_stderr(io.StringIO()),\
                 patch.object(active,'load_library') as load,patch.object(benchmark.imu,'ICM20948') as device,\
                 self.assertRaises(SystemExit):
                self.invoke(self.replace('--request-gap-us',str(requested)))
            load.assert_not_called();device.assert_not_called()

    def test_unselected_gaps_and_window_reject_with_matching_profile_before_native_load(self):
        for gap in (850,870,879,891):
            self.fixture.data['request_gap_us']=gap
            # An invalid scope cannot be sealed by the approved fixture helper.
            # Pin the raw candidate to exercise the CLI's fail-closed loader.
            self.fixture.path.write_text(json.dumps(self.fixture.data))
            self.argv[self.argv.index('--active-fk-profile-sha256')+1]=hashlib.sha256(
                self.fixture.path.read_bytes()).hexdigest()
            self.argv[self.argv.index('--request-gap-us')+1]=str(gap)
            with self.subTest(gap=gap),contextlib.redirect_stderr(io.StringIO()),\
                 patch.object(active,'load_library') as load,patch.object(benchmark.imu,'ICM20948') as device,\
                 self.assertRaises(SystemExit):
                self.invoke(self.argv)
            load.assert_not_called();device.assert_not_called()
        self.select_gap(880);self.fixture.data['request_window']=2
        self.fixture.path.write_text(json.dumps(self.fixture.data))
        self.argv[self.argv.index('--active-fk-profile-sha256')+1]=hashlib.sha256(self.fixture.path.read_bytes()).hexdigest()
        with contextlib.redirect_stderr(io.StringIO()),patch.object(active,'load_library') as load,\
             patch.object(benchmark.imu,'ICM20948') as device,self.assertRaises(SystemExit):
            self.invoke(self.replace('--request-window','2'))
        load.assert_not_called();device.assert_not_called()

    def test_complete_plan_pins_selected_pair_without_loading_any_library_model_or_device(self):
        with patch.object(active, 'load_library') as active_load,\
             patch.object(benchmark.native, 'load_library') as diagnostic_load,\
             patch.object(benchmark.imu, 'ICM20948') as device:
            result, plan = self.invoke(self.argv)
        self.assertEqual(result, 0)
        active_load.assert_not_called(); diagnostic_load.assert_not_called(); device.assert_not_called()
        self.assertTrue(plan['native_phase_pair'])
        self.assertTrue(plan['source_provenance']['native_phase_pair'])
        self.assertFalse(plan['output_allowed'])
        self.assertEqual(plan['native_phase_pair_plan']['paired_phases'], 'output_stop_proxy_only')
        self.assertEqual(plan['native_phase_pair_plan']['source_inventory']['sha256'],
                         hashlib.sha256(self.inventory.read_bytes()).hexdigest())
        self.assertNotIn('native_phase_pair_proof', plan)
        self.assertNotIn('native_pair_prime_before_cycles', plan)

    def test_explicit_prime_plan_is_separate_stop_only_and_file_only(self):
        with patch.object(active, 'load_library') as load,\
             patch.object(benchmark.imu, 'ICM20948') as device:
            result, plan=self.invoke(self.argv+['--native-pair-prime-before-cycles'])
        self.assertEqual(result, 0);load.assert_not_called();device.assert_not_called()
        self.assertTrue(plan['native_pair_prime_before_cycles'])
        prime=plan['native_phase_pair_plan']['pre_cycle_stop_prime']
        self.assertEqual(prime['phase_count'], 1);self.assertEqual(prime['request_count'], 12)
        self.assertEqual(prime['deadline_budget_ns'], benchmark.PERIOD_NS)
        self.assertFalse(prime['counted_as_measured_cycle'])
        self.assertFalse(prime['active_controller_qualification'])
        self.assertEqual(plan['cycles'], 5);self.assertEqual(plan['requests_per_cycle'], 26)

    def test_prime_flag_cannot_implicitly_select_backend_or_loosen_best20(self):
        argv=self.argv+['--native-pair-prime-before-cycles'];argv.remove('--native-phase-pair')
        for variant in (argv, self.replace('--request-gap-us','850')+['--native-pair-prime-before-cycles']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.invoke(variant)

    def test_incomplete_best20_or_bad_external_pins_fail_before_library_or_device(self):
        variants = [self.replace('--cycles', '10'), self.replace('--request-gap-us', '850'),
            self.replace('--release-spin-us', '200'), self.replace('--timer-slack-ns', '50000'),
            self.replace('--pre-cycle-policy-warmup-calls', '11'),
            self.replace('--native-pair-active-library-sha256', 'a' * 64),
            self.replace('--native-pair-source-inventory-sha256', 'a' * 64)]
        for flag in ('--single-thread-math', '--defer-gc-during-cycles', '--output-dispatch-trace'):
            argv = self.argv[:]; argv.remove(flag); variants.append(argv)
        for argv in variants:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()),\
                 patch.object(active, 'load_library') as load,\
                 patch.object(benchmark.imu, 'ICM20948') as device, self.assertRaises(SystemExit) as error:
                self.invoke(argv)
            self.assertEqual(error.exception.code, 2); load.assert_not_called(); device.assert_not_called()

    def test_pair_pins_cannot_implicitly_select_new_backend(self):
        argv = self.argv[:]; argv.remove('--native-phase-pair')
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.invoke(argv)

    def test_legacy_fk_profile_cannot_implicitly_qualify_selected_pair(self):
        self.fixture.data['native_phase_pair'] = False
        self.fixture.path = self.fixture.fixture.fixture.seal()
        argv = self.replace('--active-fk-profile-sha256', hashlib.sha256(self.fixture.path.read_bytes()).hexdigest())
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.invoke(argv)


if __name__ == '__main__':
    unittest.main()
