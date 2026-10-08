"""Explicit default-off trace-copy experiment on owned memory/local sockets."""
import contextlib
import ctypes as C
import gc
import io
import time
import unittest
import weakref
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import policy_live_profile as profiles
from test_native_trace_copy import owned_row
from test_native_trace_destination_cache import row_for
import test_disabled_native_pair_candidate as pair_fixture


class OwnedBufferTraceCopyTests(unittest.TestCase):
    def selected(self, cycles=1):
        backend = bench._OwnedBufferTraceCopy()
        return backend, bench._RecordTrace(cycles, 'stop-proxy', voltage_overlap=True,
                                          trace_copy_backend=backend)

    def test_all_six_slots_match_all_bytes_metadata_and_source_timestamps(self):
        row = owned_row(); before = {phase: {scope: (bytes(records), bytes(stats))
            for scope, (records, stats) in row[phase].items()}
            for phase in ('acquired', 'voltage', 'output')}
        old = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
        backend, new = self.selected()
        a = old.capture(0, row); b = new.capture(0, row)
        self.assertEqual(bytes(old.slots), bytes(new.slots))
        self.assertEqual(a.serialize(), b.serialize())
        for phase, buses in before.items():
            for scope, raw in buses.items():
                records, stats = row[phase][scope]
                self.assertEqual((bytes(records), bytes(stats)), raw)
        self.assertEqual(backend.provenance()['completed_copy_calls'], 12)
        self.assertEqual(backend.provenance()['completed_copy_bytes'], 27200)
        row['output']['front'][0][0].received_ns = 999999
        row['acquired']['rear'][1].rejected[1] = 99
        self.assertEqual(bytes(old.slots), bytes(new.slots))

    def test_padding_rejected_tail_and_every_count_keep_exact_bytes(self):
        for count in range(1, 13):
            backend, selected = self.selected(); old = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
            row = row_for(old, count)
            for phase in old.phases:
                for records, stats in row[phase].values():
                    # Preserve even bytes beyond rejected_size; compacting or
                    # reconstructing Stats is not an equivalent trace copy.
                    stats.rejected[4095] = count
                    (C.c_ubyte * C.sizeof(stats)).from_address(C.addressof(stats))[-1] = 171
            old.capture(0, row); selected.capture(0, row)
            self.assertEqual(bytes(old.slots), bytes(selected.slots))

    def test_unknown_rejected_bytes_and_overflow_counters_are_never_discarded(self):
        for rejected_size in (0, 4096, 4097):
            backend, selected = self.selected()
            old = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
            row = owned_row()
            for phase in old.phases:
                for _, stats in row[phase].values():
                    stats.rejected[:] = bytes((index * 17 + 3) % 256 for index in range(4096))
                    stats.rejected_size = rejected_size
                    stats.reads = 123456; stats.bytes = 987654; stats.waits = 73
            a = old.capture(0, row); b = selected.capture(0, row)
            self.assertEqual(bytes(old.slots), bytes(selected.slots))
            self.assertEqual(a.serialize(), b.serialize())
            # This is preservation of a failed/unknown input artifact, never
            # evidence that its counters are valid for controller admission.
            self.assertFalse(backend.provenance()['active_controller_qualification'])

    def test_selected_path_has_no_memmove_call_or_native_library(self):
        backend, trace = self.selected()
        with patch.object(C, 'memmove', side_effect=AssertionError('C boundary not selected')):
            trace.capture(0, owned_row())
        self.assertFalse(backend.provenance()['new_native_library_loaded'])
        self.assertFalse(backend.provenance()['active_controller_qualification'])
        self.assertFalse(backend.provenance()['output_allowed'])

    def test_default_still_uses_twelve_original_memmove_calls_and_no_views(self):
        trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
        with patch.object(C, 'memmove', wraps=C.memmove) as copier:
            trace.capture(0, owned_row())
        self.assertEqual(copier.call_count, 12); self.assertIsNone(trace._byte_destinations)

    def test_invalid_source_count_type_cycle_or_scope_fails_before_copy(self):
        for issue in ('count', 'record', 'stats', 'cycle', 'scope'):
            backend, trace = self.selected(); row = owned_row(); cycle = 0
            if issue == 'count': row['acquired']['front'] = (bench.native.Record * 13)(), bench.native.Stats()
            elif issue == 'record': row['acquired']['front'] = (C.c_uint64 * 6)(), bench.native.Stats()
            elif issue == 'stats': row['acquired']['front'] = row['acquired']['front'][0], object()
            elif issue == 'cycle': cycle = 1
            else: row['acquired'] = {'unknown': row['acquired']['front']}
            with self.subTest(issue=issue), self.assertRaises(ValueError): trace.capture(cycle, row)
            self.assertEqual(backend.provenance()['completed_copy_calls'], 0)

    def test_spoofed_array_length_cannot_omit_or_overread_source(self):
        for actual, reported in ((1, 12), (12, 1)):
            class Spoofed(bench.native.Record * actual):
                def __len__(self): return reported
            backend, trace = self.selected(); row = owned_row()
            row['acquired']['front'] = Spoofed(), bench.native.Stats()
            with self.assertRaisesRegex(ValueError, 'extent'): trace.capture(0, row)
            self.assertEqual(backend.provenance()['completed_copy_calls'], 0)

    def test_destination_rebind_resize_and_backend_replacement_fail_before_copy(self):
        for issue in ('slots', 'views', 'destinations', 'backend', 'resize'):
            backend, trace = self.selected(); row = owned_row()
            if issue == 'slots': trace.slots = type(trace.slots)()
            elif issue == 'views': trace._byte_destinations = ()
            elif issue == 'destinations': trace._destinations = ()
            elif issue == 'backend': trace.trace_copy_backend = Mock()
            else: C.resize(trace.slots, C.sizeof(trace.slots) + 100)
            with self.subTest(issue=issue), self.assertRaisesRegex(ValueError, 'ownership/ABI'):
                trace.capture(0, row)
            self.assertEqual(backend.provenance()['completed_copy_calls'], 0)

    def test_released_destination_fails_without_mutating_sources_or_claiming_completion(self):
        backend, trace = self.selected(); row = owned_row()
        records, stats = row['acquired']['front']; before = bytes(records), bytes(stats)
        trace._byte_destinations[0][1].release()
        with self.assertRaises(ValueError): trace.capture(0, row)
        self.assertEqual((bytes(records), bytes(stats)), before)
        self.assertEqual(backend.provenance()['completed_copy_calls'], 1)
        self.assertEqual(trace.slots[0].count, 0)

    def test_binding_and_source_changed_fail_closed(self):
        backend, trace = self.selected()
        with patch.object(bench, 'memoryview', Mock(), create=True), \
             self.assertRaisesRegex(ValueError, 'binding changed'): trace.capture(0, owned_row())
        with patch.object(bench.Path, 'read_bytes', return_value=b'changed'), \
             self.assertRaisesRegex(ValueError, 'source changed'): backend.verify()
        self.assertEqual(backend.provenance()['completed_copy_calls'], 0)
        with patch.object(backend, 'copy_exchange', Mock()), \
             self.assertRaisesRegex(ValueError, 'binding changed'): trace.capture(0, owned_row())
        self.assertEqual(backend.provenance()['completed_copy_calls'], 0)

    def test_source_buffers_release_after_capture_and_views_keep_only_destinations(self):
        backend, trace = self.selected(); row = owned_row()
        source = row['acquired']['front'][0]; reference = weakref.ref(source)
        captured = trace.capture(0, row); source = row = None
        self.assertIsNone(reference())
        frozen = bytes(trace.slots); trace = None; gc.collect()
        self.assertEqual(bytes(captured.storage.slots), frozen)

    def test_experimental_trace_provenance_is_still_rejected_for_output_qualification(self):
        backend = bench._OwnedBufferTraceCopy()
        with self.assertRaisesRegex(ValueError, 'Experimental trace-copy'):
            profiles._timing({'trace_copy_provenance': backend.provenance()}, {})

    def test_nonpair_collect_cannot_select_new_backend(self):
        with self.assertRaisesRegex(ValueError, 'explicit native pair'):
            bench.collect({}, None, object(), mode='stop-proxy', cycles=5,
                record_storage='trace', v3_voltage_proxy=True,
                trace_copy_backend=bench._OwnedBufferTraceCopy())


class OwnedBufferTraceCopyPlanTests(unittest.TestCase):
    setUp = pair_fixture.DisabledNativePairCLITests.setUp
    invoke = pair_fixture.DisabledNativePairCLITests.invoke

    def test_explicit_plan_reports_zero_copies_and_cannot_enable_or_load_any_device(self):
        with patch.object(bench.native, 'load_library') as native, \
             patch.object(bench.imu, 'ICM20948') as device:
            code, plan = self.invoke(self.argv + ['--owned-buffer-trace-copy'])
        self.assertEqual(code, 0); native.assert_not_called(); device.assert_not_called()
        proof = plan['trace_copy_provenance']
        self.assertEqual(proof['completed_copy_calls'], 0)
        self.assertEqual(proof['copy_method'], 'builtin_owned_byte_memoryview_assignment')
        self.assertFalse(proof['active_controller_qualification']); self.assertFalse(plan['output_allowed'])
        _, default = self.invoke(self.argv)
        self.assertNotIn('trace_copy_provenance', default)

    def test_nonpair_and_other_experiment_flags_reject_before_factory_or_io(self):
        for change in ('nonpair', '--retain-gil-trace-copy', '--native-boot-guard-artifact'):
            argv = self.argv + ['--owned-buffer-trace-copy']
            if change == 'nonpair': argv.remove('--native-phase-pair')
            elif change == '--retain-gil-trace-copy': argv.append(change)
            else: argv += [change, 'never-load']
            with self.subTest(change=change), patch.object(bench, '_OwnedBufferTraceCopy') as factory, \
                 contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): self.invoke(argv)
            factory.assert_not_called()


class OwnedBufferTraceCopyCollectionTests(unittest.TestCase):
    setUpClass = classmethod(pair_fixture.DisabledNativePairCandidateTests.setUpClass.__func__)
    tearDownClass = classmethod(pair_fixture.DisabledNativePairCandidateTests.tearDownClass.__func__)
    setUp = pair_fixture.DisabledNativePairCandidateTests.setUp
    tearDown = pair_fixture.DisabledNativePairCandidateTests.tearDown
    create = pair_fixture.DisabledNativePairCandidateTests.create
    device = pair_fixture.DisabledNativePairCandidateTests.device
    collect_candidate = pair_fixture.DisabledNativePairCandidateTests.collect_candidate

    def test_genuine_socket_collection_keeps_full_trace_original_deadlines_and_restore(self):
        backend = bench._OwnedBufferTraceCopy()
        capture = bench._RecordTrace.capture; copy_ends = []
        def checked_capture(trace, *args):
            result = capture(trace, *args)
            copy_ends.append(time.monotonic_ns())
            return result
        with patch.object(bench._RecordTrace, 'capture', checked_capture):
            report, rows = self.collect_candidate(extra_options={'trace_copy_backend': backend})
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC', report['errors'])
        self.assertEqual(report['cycles_completed'], 5)
        self.assertEqual(report['trace_copy_provenance']['completed_copy_calls'], 60)
        self.assertTrue(report['trace_copy_provenance']['source_files_unchanged'])
        self.assertTrue(report['native_phase_pair_evidence']['all_phases_joined'])
        self.assertTrue(report['native_phase_pair_evidence']['owner_settings_restored'])
        self.assertEqual(len(copy_ends), 5)
        for row, timing, copy_end in zip(rows, report['measurements'], copy_ends):
            saved = row.serialize(); proof = saved['voltage_fast_pipeline']
            self.assertGreaterEqual(timing['cycle_end_ns'], proof['stop_reply_verified_ns'])
            self.assertGreaterEqual(timing['cycle_end_ns'], copy_end)
            self.assertTrue(all(record['deadline_ns'] == proof['hard_deadline_ns']
                for bus in saved['output'].values() for record in bus['records']))
        self.assertFalse(report['active_controller_qualification'])

    def test_partial_copy_failure_keeps_original_raw_and_restores_without_completed_cycle(self):
        backend = bench._OwnedBufferTraceCopy(); capture = bench._RecordTrace.capture
        def rejected_capture(trace, *args):
            trace._byte_destinations[0][1].release()
            return capture(trace, *args)
        with patch.object(bench._RecordTrace, 'capture', rejected_capture):
            report, rows = self.collect_candidate(extra_options={'trace_copy_backend': backend})
        self.assertEqual(report['status'], 'ABORTED')
        self.assertEqual(report['cycles_completed'], 0)
        self.assertEqual(report['measurements'], [])
        self.assertEqual(report['trace_copy_provenance']['completed_copy_calls'], 1)
        self.assertTrue(report['native_phase_pair_evidence']['all_phases_joined'])
        self.assertTrue(report['native_phase_pair_evidence']['owner_settings_restored'])
        self.assertFalse(report['active_controller_qualification'])
        self.assertFalse(report['motor_enable_sent']); self.assertFalse(report['learned_targets_sent'])
        self.assertEqual(len(rows), 1)
        raw = bench._serialize(rows)[0]
        self.assertEqual(sum(len(bus['records']) for phase in ('acquired', 'voltage', 'output')
                             for bus in raw[phase].values()), 26)
        self.assertTrue(all(record['received'] == 17 for bus in raw['output'].values()
                            for record in bus['records']))
        self.assertEqual(report['record_storage_failure']['cycle'], 1)


if __name__ == '__main__': unittest.main()
