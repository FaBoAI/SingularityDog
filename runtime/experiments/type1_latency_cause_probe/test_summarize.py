"""Synthetic pinned file fixtures for strict correlation; never physical evidence."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import struct

from . import summarize as S


def trace_line(tid, ns, kind, payload=''):
    kind = {'restore_entry': 'acquire_entry', 'restore_return': 'acquire_exit'}.get(kind, kind)
    return f' python-{tid} [000] d... {ns / 1e9:.6f}: {kind}: {payload}\n'


def fixture():
    tid, base = 101, 1_000_000_000
    def event(stage, delta, exchange='feedback_hold', **extra):
        stamp = base + delta
        return {'stage': stage, 'wall_ns': stamp, 'sample_started_ns': stamp,
                'sample_finished_ns': stamp + 50, 'tid': tid, 'port': 'port0',
                'cycle': 0, 'exchange': exchange, 'thread_cpu_ns': delta,
                'nvcsw': delta // 1_000_000, 'nivcsw': 0, 'sched_run_delay_ns': delta // 10,
                **extra}
    events = [event('native_call_before', -1_000_000),
        event('native_first_python_statement', 10_000_000, native_stats_end_ns=base, native_status=0),
        event('checked_decode_before', 11_000_000), event('checked_decode_after', 15_000_000),
        event('batch_constructor_before', 19_000_000),
        event('batch_constructor_after', 21_000_000, batch_argument_stamp_ns=base + 20_000_000),
        event('batch_registry_complete', 22_000_000),
        event('voltage_preflight_hook_before', 23_000_000, 'voltage'),
        event('prefix_set_result_before', 24_000_000, 'voltage'),
        event('prefix_set_result_after', 25_000_000, 'voltage'),
        event('voltage_preflight_hook_after', 26_000_000, 'voltage'),
        event('native_call_before', 30_000_000, 'voltage'),
        event('native_first_python_statement', 50_000_000, 'voltage', native_stats_end_ns=base + 40_000_000, native_status=0),
        event('checked_decode_before', 51_000_000, 'voltage'), event('checked_decode_after', 52_000_000, 'voltage'),
        event('batch_constructor_before', 53_000_000, 'voltage'),
        event('batch_constructor_after', 55_000_000, 'voltage', batch_argument_stamp_ns=base + 54_000_000),
        event('batch_registry_complete', 56_000_000, 'voltage'),
        event('outer_set_result_before', 57_000_000, 'voltage'),
        event('outer_set_result_after', 58_000_000, 'voltage')]
    # Counters are monotone and explicitly synthetic; the first call begins
    # before Stats.end, so use positive counters unrelated to wall origin.
    for i, row in enumerate(events):
        row.update(thread_cpu_ns=(i + 1) * 1000, nvcsw=i, sched_run_delay_ns=i * 100)
    placement = {'supported': True, 'restored': True, 'errors': [], 'native_tid': tid,
                 'requested_cpus': [0], 'during_cpus': [0]}
    probe = {'schema': S.PROBE_SCHEMA, 'status': 'COMPLETE_CPU_PROBE', 'errors': [],
        **dict.fromkeys(S.FALSE_FLAGS, False), 'instrumentation_restored': True,
        'original_source_unchanged': True, 'virtual_last_output_seed': True, 'virtual_boot_fixture': True,
        'counter_detail': 'full', 'take_mode': 'exact_original_pipeline_take_with_counted_cpu_only_noop_hot',
        'native_stats_end_is_not_exact_function_return': True, 'gil_or_os_wait_diagnosed_from_wall_alone': False,
        'pid': 500, 'owner_native_tids': [tid], 'ports': 1, 'groups': {'port0': [1, 2, 3]},
        'main_placement': dict(placement, requested_cpus=[4], during_cpus=[4]),
        'owner_placement': {'port0': placement}, 'peer': {'errors': [], 'motor_enable_sent': False,
            'positive_gain_sent': False, 'placement': dict(placement, requested_cpus=[0, 1, 2, 3], during_cpus=[0, 1, 2, 3])},
        'gc_scope': {'restored': True}, 'python_switch_interval': {'restored': True, 'during_s': .0001},
        'failed_cycles': 0, 'completed_cycles': 1, 'requested_cycles': 1,
        'source_sha256': 'a' * 64, 'transport_source_sha256': 'b' * 64,
        'native_pins': {'expected_sha256': 'c' * 64}, 'events': events}
    text = (trace_line(tid, base + 1_000_000, 'restore_entry') +
            trace_line(tid, base + 2_000_000, 'sched_switch', 'prev_pid=101 prev_state=S next_pid=90') +
            trace_line(90, base + 5_000_000, 'sched_wakeup', 'comm=python pid=101 prio=120 target_cpu=000') +
            trace_line(90, base + 6_000_000, 'sched_switch', 'prev_pid=90 prev_state=R next_pid=101') +
            trace_line(tid, base + 9_000_000, 'restore_return') +
            trace_line(tid, base + 41_000_000, 'restore_entry') +
            trace_line(tid, base + 49_000_000, 'restore_return') +
            # An unrelated actor's long path is never aggregated with owner data.
            trace_line(999, base - 1_000_000, 'restore_entry') +
            trace_line(999, base + 200_000_000, 'restore_return'))
    report = {'schema': S.TRACE_SCHEMA, 'status': 'RECORDED_CPU_ONLY_TRACE',
              'motor_hardware_opened': False, 'output_allowed': False, 'restored': True,
              'source_unchanged': True, 'child_reaped': True, 'cleanup_errors': [],
              'child_exit_code': 0, 'child_workload_completed': True, 'child_pid': 500,
              'probe_sha256': 'a' * 64, 'clock': 'mono', 'trace_clock_readback': 'local [mono] global',
              'timestamp_text_precision_ns': 1000, 'trace_buffer_loss_events': 0,
              'per_cpu_stats': {'cpu0': 'overrun: 0\ncommit overrun: 0\ndropped events: 0\n'}}
    offset = 0x18c290
    words = [0xd503233f, 0xa9be7bfd, 0x910003fd, 0xf9000bf3, 0xb4000100, 0xaa0003f3,
             0x94000000, 0xaa1303e0, 0xf9400bf3, 0xa8c27bfd, 0xd50323bf, 0x14000000]
    semantic = 'After PACIASP before frame save, through take_gil, to authenticated pre-tail branch; excludes thread-state swap/return tail'
    points = {'kind': 'arm64_pac_safe_gil_acquisition_path', 'semantics': semantic, 'lr_hijack_used': False,
              'reviewed_executable_sha256': S.TS.ARM64_PAC_EXEC_SHA256,
              'function_entry_file_offset': offset, 'reviewed_instruction_bytes_hex': struct.pack('<12I', *words).hex(),
              'points': [{'kind': 'p', 'name': 'acquire_entry', 'file_offset': offset + 4, 'opcode_hex': 'a9be7bfd'},
                         {'kind': 'p', 'name': 'acquire_exit', 'file_offset': offset + 44, 'opcode_hex': '14000000'}]}
    report.update(probe_points=points, endpoint_semantics=semantic, probe_kind=points['kind'], lr_hijack_used=False,
                  python_sha256=S.TS.ARM64_PAC_EXEC_SHA256, restore_thread_file_offset=offset)
    return probe, report, text


class SummaryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name).resolve()

    def tearDown(self):
        self.tmp.cleanup()

    def save(self, probe, report, trace):
        raw_trace = trace.encode()
        report = dict(report, trace_sha256=S.digest(raw_trace), trace_bytes=len(raw_trace))
        files = (self.root / 'probe.json', self.root / 'trace-report.json', self.root / 'trace.txt')
        values = (json.dumps(probe).encode(), json.dumps(report).encode(), raw_trace)
        for path, raw in zip(files, values):
            path.write_bytes(raw)
        return [value for pair in zip(files, [S.digest(v) for v in values]) for value in pair]

    def summary(self, data=None):
        return S.summarize(*self.save(*(data or fixture())))

    def test_exact_same_tid_windows_and_blocked_witness_without_pure_gil_claim(self):
        result = self.summary()
        self.assertEqual((result['matched_windows'], result['unproven_windows']), (2, 0))
        hold = result['correlations'][0]
        self.assertEqual(hold['restore_interval']['duration_ns'], 8_000_000)
        self.assertEqual(hold['restore_interval']['sleeping_ns'], 3_000_000)
        self.assertEqual(hold['restore_interval']['runnable_wait_ns'], 1_000_000)
        self.assertTrue(hold['restore_path_has_blocked_sleep_witness'])
        self.assertFalse(result['pure_gil_wait_proven'])
        self.assertFalse(result['original_physical_cycle_cause_proven'])
        self.assertFalse(result['output_allowed'])
        self.assertEqual(result['trace_coverage']['owner_restore_intervals_total'], 2)
        stage = next(r for r in result['stage_statistics'] if r['stage'] == 'first_python_to_batch_argument'
                     and r['exchange'] == 'feedback_hold')
        self.assertEqual(stage['p50'], 10_000_000)
        publication = next(r for r in result['stage_statistics'] if r['stage'] == 'batch_argument_to_prefix_publication_after')
        self.assertEqual(publication['max'], 5_000_000)

    def test_no_match_and_multiple_matches_are_unproven(self):
        probe, report, text = fixture()
        wrong_tid = text.replace('python-101', 'python-102')
        result = self.summary((probe, report, wrong_tid))
        self.assertEqual((result['matched_windows'], result['unproven_windows']), (0, 2))
        extra = trace_line(101, 1_000_500_000, 'restore_entry') + trace_line(101, 1_000_900_000, 'restore_return')
        result = self.summary((probe, report, extra + text))
        self.assertEqual(result['correlations'][0]['candidate_count'], 2)
        self.assertEqual(result['correlations'][0]['status'], 'UNPROVEN')

    def test_unmatched_inside_blocks_but_unrelated_outside_does_not(self):
        probe, report, text = fixture()
        outside = trace_line(101, 3_000_000_000, 'restore_return')
        result = self.summary((probe, report, text + outside))
        self.assertEqual(result['matched_windows'], 2)
        self.assertEqual(len(result['trace_coverage']['unmatched_owner_returns']), 1)
        inside = trace_line(101, 1_007_000_000, 'restore_return')
        result = self.summary((probe, report, text + inside))
        self.assertEqual(result['correlations'][0]['status'], 'UNPROVEN')

    def test_loss_unknown_positive_raw_loss_and_nonrestored_scope_rejected(self):
        for change in ({'trace_buffer_loss_events': None}, {'trace_buffer_loss_events': 1},
                       {'restored': False}, {'child_pid': 501}, {'probe_sha256': 'd' * 64},
                       {'per_cpu_stats': {'cpu0': 'overrun: 1\n'}}, {'per_cpu_stats': {}},
                       {'clock': 'local'}):
            probe, report, text = fixture(); report.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.summary((probe, report, text))

    def test_hardware_scope_source_mutation_or_wrong_native_tid_rejected(self):
        for key in S.FALSE_FLAGS:
            probe, report, text = fixture(); probe[key] = True
            with self.subTest(flag=key), self.assertRaises(ValueError):
                self.summary((probe, report, text))
        probe, report, text = fixture(); probe['events'][1]['tid'] = 102
        with self.assertRaisesRegex(ValueError, 'owner TID'):
            self.summary((probe, report, text))
        args = self.save(*fixture())
        args[1] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'SHA differs'):
            S.summarize(*args)

    def test_sampling_counter_windows_not_exact_cpu_and_missing_boundary_rejected(self):
        result = self.summary()
        cpu = next(r for r in result['same_thread_counter_upper_window_statistics']['thread_cpu_ns']
                   if r['stage'] == 'checked_decode' and r['exchange'] == 'feedback_hold')
        self.assertEqual(cpu['max'], 1000)
        self.assertTrue(result['counter_deltas_are_sampling_window_upper_bounds'])
        probe, report, text = fixture()
        probe['events'] = [e for e in probe['events'] if e['stage'] != 'prefix_set_result_after']
        with self.assertRaisesRegex(ValueError, 'Stage starts'):
            self.summary((probe, report, text))

    def test_indexed_scheduler_splitter_matches_original_and_marker_coverage_is_exact(self):
        probe, report, text = fixture()
        direct = S.TA.analyze(text, {101})
        indexed = S.analyze_owner_trace(text, {101})
        self.assertEqual(indexed['intervals'], direct['intervals'])
        probe['events'].append(copy.deepcopy(probe['events'][1]))
        with self.assertRaisesRegex(ValueError, 'marker coverage'):
            self.summary((probe, report, text))

    def test_legacy_endpoint_or_wrong_elf_instruction_semantics_rejected(self):
        for change in ({'probe_kind': 'restore_thread_entry_to_function_return'},
                       {'python_sha256': 'f' * 64}, {'lr_hijack_used': True}):
            probe, report, text = fixture(); report.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.summary((probe, report, text))
        probe, report, text = fixture()
        report['probe_points']['reviewed_instruction_bytes_hex'] = '00' * 48
        with self.assertRaisesRegex(ValueError, 'image differs'):
            self.summary((probe, report, text))

    def test_precision_expanded_edge_and_nearest_rank_summary(self):
        probe, report, text = fixture()
        probe['events'][1]['native_stats_end_ns'] = 1_001_000_500
        result = self.summary((probe, report, text))
        self.assertTrue(result['correlations'][0]['precision_expanded_edges'])
        self.assertEqual(S.percentiles([1, 2, 3, 4])['p95'], 4)
        self.assertEqual(S.percentiles([1, 2, 3, 4])['p50'], 2)


if __name__ == '__main__':
    unittest.main()
