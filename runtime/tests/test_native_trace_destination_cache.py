"""Trace caching parity and lifetime checks using owned local buffers only."""
import ctypes as C
import gc
import unittest
import weakref
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_trace_copy import owned_row


def row_for(trace, count):
    row = owned_row()
    for phase in trace.phases:
        for scope in bench.dual.SCOPES:
            old, stats = row[phase][scope]
            records = (bench.native.Record * count)()
            for index, record in enumerate(records):
                C.memmove(C.addressof(record), C.addressof(old[index % len(old)]), C.sizeof(record))
                record.start_ns += index * 1000
                record.finish_ns += index * 1000
                record.received_ns += index * 1000
            row[phase][scope] = records, stats
    return {key: value for key, value in row.items()
            if key not in ('acquired', 'voltage', 'output') or key in trace.phases}


class TraceDestinationCacheTests(unittest.TestCase):
    def test_full_bytes_and_serialized_evidence_match_all_counts_modes_and_cycles(self):
        for mode, overlap in (('type17', False), ('stop-proxy', False), ('stop-proxy', True)):
            trace = bench._RecordTrace(12, mode, voltage_overlap=overlap)
            for count in range(1, 13):
                row = row_for(trace, count)
                expected = {key: value for key, value in row.items() if key not in trace.phases}
                expected.update({phase: {scope: bench.native.exchange_evidence(list(records), stats)
                    for scope, (records, stats) in row[phase].items()} for phase in trace.phases})
                captured = trace.capture(count - 1, row)
                self.assertEqual(captured.serialize(), expected)
                for phase in trace.phases:
                    for scope, (records, stats) in row[phase].items():
                        slot = trace._slot(count - 1, phase, scope)
                        self.assertEqual(slot.count, count)
                        self.assertEqual(C.string_at(C.addressof(slot.records), C.sizeof(records)), bytes(records))
                        self.assertEqual(bytes(slot.stats), bytes(stats))
                        self.assertEqual(bytes(slot.records)[C.sizeof(records):],
                                         bytes(C.sizeof(slot.records) - C.sizeof(records)))

    def test_replaced_or_resized_owner_is_rejected_before_pointer_use(self):
        for issue in ('replace', 'resize'):
            with self.subTest(issue=issue):
                trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
                row = owned_row()
                if issue == 'replace':
                    trace.slots = type(trace.slots)()
                else:
                    C.resize(trace.slots, trace.allocated_bytes + 4096)
                with patch.object(C, 'memmove') as copier:
                    with self.assertRaisesRegex(ValueError, 'ownership/ABI'):
                        trace.capture(0, row)
                    with self.assertRaisesRegex(ValueError, 'ownership/ABI'):
                        trace.evidence(0, 'acquired', 'front')
                copier.assert_not_called()

    def test_rebound_source_or_destination_abi_rejects_before_any_copy(self):
        for owner, attribute in ((bench.native, 'Record'), (bench.native, 'Stats'),
                                 (bench, '_TraceExchange')):
            with self.subTest(attribute=attribute):
                trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
                row = owned_row()
                with patch.object(owner, attribute, object()), patch.object(C, 'memmove') as copier:
                    with self.assertRaisesRegex(ValueError, 'ownership/ABI'):
                        trace.capture(0, row)
                copier.assert_not_called()

    def test_rebound_source_abi_before_setup_cannot_change_destination_geometry(self):
        class Larger(C.Structure):
            _fields_ = [('extra', C.c_uint64 * 1024)]
        for attribute in ('Record', 'Stats'):
            with self.subTest(attribute=attribute), patch.object(bench.native, attribute, Larger):
                with patch.object(C, 'memset') as toucher:
                    with self.assertRaisesRegex(ValueError, 'ownership/ABI'):
                        bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
                toucher.assert_not_called()

    def test_changed_layout_rejects_before_any_copy(self):
        for field, value in (('capacity_cycles', 2), ('slots_per_cycle', 1),
                             ('phases', ('output', 'acquired', 'voltage'))):
            trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
            row = owned_row()
            setattr(trace, field, value)
            with self.subTest(field=field), patch.object(C, 'memmove') as copier:
                with self.assertRaisesRegex(ValueError, 'ownership/ABI'):
                    trace.capture(0, row)
            copier.assert_not_called()

    def test_compatible_array_subclass_keeps_full_bytes(self):
        class Compatible(bench.native.Record * 6):
            pass
        row = owned_row()
        original, stats = row['acquired']['front']
        records = Compatible.from_buffer_copy(original)
        row['acquired']['front'] = records, stats
        trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
        trace.capture(0, row)
        self.assertEqual(bytes(trace._slot(0, 'acquired', 'front').records)[:C.sizeof(records)],
                         bytes(records))

    def test_spoofed_array_count_cannot_overread_or_omit_raw_bytes(self):
        for actual, reported in ((1, 12), (12, 1)):
            class Spoofed(bench.native.Record * actual):
                def __len__(self):
                    return reported
            row = owned_row()
            row['acquired']['front'] = Spoofed(), bench.native.Stats()
            trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
            with self.subTest(actual=actual, reported=reported), patch.object(C, 'memmove') as copier:
                with self.assertRaisesRegex(ValueError, 'extent'):
                    trace.capture(0, row)
            copier.assert_not_called()

    def test_count_type_cycle_and_scope_checks_still_precede_copy(self):
        for issue in ('zero', 'thirteen', 'record_type', 'stats_type', 'cycle', 'scope'):
            trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
            row = owned_row()
            index = 0
            if issue in ('zero', 'thirteen'):
                row['acquired']['front'] = (bench.native.Record * (0 if issue == 'zero' else 13))(), bench.native.Stats()
            elif issue == 'record_type':
                row['acquired']['front'] = (C.c_uint64 * 6)(), bench.native.Stats()
            elif issue == 'stats_type':
                row['acquired']['front'] = row['acquired']['front'][0], object()
            elif issue == 'cycle':
                index = 1
            else:
                row['acquired'] = {'unknown': row['acquired']['front']}
            with self.subTest(issue=issue), patch.object(C, 'memmove') as copier:
                with self.assertRaises(ValueError):
                    trace.capture(index, row)
            copier.assert_not_called()

    def test_trace_row_owns_destination_without_retaining_borrowed_sources(self):
        trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
        row = owned_row()
        sources = [weakref.ref(item) for phase in trace.phases
                   for pair in row[phase].values() for item in pair]
        owner = weakref.ref(trace.slots)
        captured = trace.capture(0, row)
        expected = captured.serialize()
        del row, trace
        gc.collect()
        self.assertTrue(all(reference() is None for reference in sources))
        self.assertIsNotNone(owner())
        self.assertEqual(captured.serialize(), expected)
        del captured
        gc.collect()
        self.assertIsNone(owner())

    def test_later_source_mutations_cannot_change_retained_raw_copy(self):
        trace = bench._RecordTrace(1, 'stop-proxy', voltage_overlap=True)
        row = owned_row()
        captured = trace.capture(0, row)
        expected_bytes, expected_evidence = bytes(trace.slots), captured.serialize()
        for phase in trace.phases:
            for records, stats in row[phase].values():
                C.memset(C.addressof(records), 0, C.sizeof(records))
                C.memset(C.addressof(stats), 0, C.sizeof(stats))
        self.assertEqual(bytes(trace.slots), expected_bytes)
        self.assertEqual(captured.serialize(), expected_evidence)


if __name__ == '__main__':
    unittest.main()
