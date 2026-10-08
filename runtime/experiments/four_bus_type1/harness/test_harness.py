"""Motor-free harness tests (quick on macOS; no device, no root, no network).

The end-to-end class builds the current library once and runs two 2 s harness
runs over socketpair peers; it is skipped without a C++ compiler.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from experiments.four_bus_type1.harness import analyze, injection, latency_model, peer

RUNTIME = Path(__file__).resolve().parents[3]


class ParseTest(unittest.TestCase):
    def test_tail_and_stall_specs(self):
        self.assertEqual(injection.parse_tail('port3:70:+1.8', 'hold'),
                         {'stage': 'hold', 'port': 'port3', 'cycle': 70, 'extra_ms': 1.8, 'extra_ns': 1800000})
        self.assertEqual(injection.parse_tail('1:5:+2', 'voltage')['port'], 'port1')
        self.assertEqual(injection.parse_stall('26:2.6'), {'cycle': 26, 'ms': 2.6, 'mode': 'gil'})
        self.assertEqual(injection.parse_stall('26:2.6:sleep')['mode'], 'sleep')
        for bad in ('port3:70:1.8', 'port4:70:+1', 'port3:-1:+1', 'port3:7:+0', 'port3:7:+11', 'port3:7:+nan', 'port3:7'):
            with self.assertRaises(ValueError, msg=bad):
                injection.parse_tail(bad, 'hold')
        for bad in ('26', '26:0', '26:16', '26:1:spin', 'x:1'):
            with self.assertRaises(ValueError, msg=bad):
                injection.parse_stall(bad)
        with self.assertRaisesRegex(ValueError, 'Duplicate tail'):
            injection.parse(['port3:7:+1', 'port3:7:+2'])
        with self.assertRaisesRegex(ValueError, 'Duplicate host stall'):
            injection.parse(stalls=['7:1', '7:2:sleep'])
        self.assertEqual(injection.parse(None, None, None), ([], []))


class FakeClock:
    def __init__(self):
        self.now = 2_000_000_000

    def __call__(self):
        self.now += 1000
        return self.now


class InjectionsTest(unittest.TestCase):
    def test_empty_wraps_nothing(self):
        value = injection.Injections()
        wait = lambda scheduled: scheduled
        self.assertTrue(value.empty)
        self.assertIs(value.wrap_release_wait(wait), wait)
        self.assertIs(value.executor_class(ThreadPoolExecutor), ThreadPoolExecutor)
        with self.assertRaisesRegex(ValueError, 'control pipe'):
            injection.Injections([injection.parse_tail('port0:1:+1', 'hold')])

    def _controller(self, prearmed, clock):
        read, write = os.pipe()
        self.addCleanup(os.close, read); self.addCleanup(os.close, write)
        tails, stalls = injection.parse(['port3:2:+1.8'], ['port1:3:+1.5'], ['2:2.6:sleep'])
        stalled = []
        value = injection.Injections(tails, stalls, prearmed=prearmed, control_fd=write, clock=clock,
                                     stall_functions={'gil': stalled.append, 'sleep': stalled.append})
        return value, read, stalled

    def test_default_path_tails_before_release_and_stall_in_first_hold_submit(self):
        clock = FakeClock()
        value, read, stalled = self._controller(False, clock)
        log = []
        wait = value.wrap_release_wait(lambda scheduled: log.append(('wait', scheduled)) or clock())
        executor = value.executor_class(ThreadPoolExecutor)
        self.assertIsNot(executor, ThreadPoolExecutor)
        pool = executor(max_workers=1)
        self.addCleanup(pool.shutdown)
        releases = [2_000_000_000+k*20_000_000 for k in range(4)]
        for cycle, release in enumerate(releases):
            wait(release)
            if cycle == 2:
                self.assertEqual(os.read(read, 4096), b'hold port3 1800000 2\n')  # Sent before the cycle's wait.
                self.assertEqual(stalled, [])  # Not in the wait: in the first submit after the gates.
            pool.submit(int).result(); pool.submit(int).result()
        self.assertEqual(os.read(read, 4096), b'voltage port1 1500000 3\n')
        self.assertEqual(stalled, [2_600_000])
        performed, = value.performed
        self.assertEqual((performed['cycle'], performed['hook'], performed['release_ns']),
                         (2, 'before_first_hold_submit', releases[2]))
        self.assertGreaterEqual(performed['start_ns'], releases[2]+injection.STALL_AT_NS)
        self.assertEqual([t['cycle'] for t in value.sent], [2, 3])
        report = value.report([{'stage': 'hold', 'port': 'port3', 'cycle': 2, 'extra_ns': 1800000, 'arrived_ns': 1}])
        self.assertEqual([t['stage'] for t in report['tails_not_applied']], ['voltage'])
        self.assertEqual(value.row_fields(2, report['tails_applied_by_peer'])['inject_hold_tail_port3_ms'], 1.8)
        self.assertEqual(value.row_fields(1), {})

    def test_prearmed_path_counts_two_waits_per_cycle_and_stalls_after_the_release_wait(self):
        clock = FakeClock()
        value, read, stalled = self._controller(True, clock)
        self.assertIs(value.executor_class(ThreadPoolExecutor), ThreadPoolExecutor)
        wait = value.wrap_release_wait(lambda scheduled: clock())
        for cycle in range(4):
            release = 2_000_000_000+cycle*20_000_000
            wait(release-1_500_000)
            if cycle == 2:
                self.assertEqual(os.read(read, 4096), b'hold port3 1800000 2\n')
                self.assertEqual(stalled, [])
            wait(release)
            if cycle == 2:
                self.assertEqual(stalled, [2_600_000])
        performed, = value.performed
        self.assertEqual((performed['cycle'], performed['hook']), (2, 'after_actual_release_wait_prearmed'))
        self.assertGreaterEqual(performed['start_after_release_ms'], .16)

    def _progress_during(self, stall, ns):
        stamps, stop = [], threading.Event()

        def spin():
            while not stop.is_set():
                stamps.append(time.monotonic_ns())
                time.sleep(0)
        thread = threading.Thread(target=spin)
        thread.start()
        time.sleep(.01)
        started = time.monotonic_ns()
        stall(ns)
        ended = time.monotonic_ns()
        stop.set(); thread.join()
        self.assertGreaterEqual(ended-started, ns)
        return [s for s in stamps if started+2_000_000 < s < ended-2_000_000]

    def test_gil_stall_holds_the_gil_and_sleep_stall_releases_it(self):
        self.assertEqual(self._progress_during(injection.stall_gil, 20_000_000), [])
        self.assertTrue(self._progress_during(injection.stall_sleep, 20_000_000))


class PeerInjectionTest(unittest.TestCase):
    """The real peer loop on one socketpair: a tail delays only its own burst's replies."""
    def test_hold_and_voltage_tails_apply_to_the_next_matching_burst_only(self):
        from experiments.four_bus_type1 import test_type1_runner as fixture
        from singularitydog_hw import can_readonly as codec
        from singularitydog_hw.can_readonly import ATParser
        host, remote = socket.socketpair()
        remote.setblocking(False)
        stop_r, stop_w = os.pipe()
        control_r, control_w = os.pipe()
        model = latency_model.load()
        sampler = latency_model.Sampler(model, 'fixed', fixed={'type1_first': 1_000_000, 'type1_rest': 2_000_000,
                                                               'single': 1_000_000})
        motors = peer.Motors({'raw_by_id': {str(m): fixture.RAW[m] for m in fixture.IDS},
                              'uid_by_id': {str(m): fixture.UID[m] for m in fixture.IDS},
                              'firmware_by_id': {str(m): fixture.FIRMWARE[m] for m in fixture.IDS}}, 1)
        log, applied = [], []
        switch = sys.getswitchinterval()
        sys.setswitchinterval(.0001)  # The in-process peer spins; keep the GIL hand-off short.
        self.addCleanup(sys.setswitchinterval, switch)
        thread = threading.Thread(target=peer.serve, args=({remote.fileno(): remote}, motors, sampler, log),
                                  kwargs={'stop_fd': stop_r, 'control_fd': control_r,
                                          'port_names': {remote.fileno(): 'port0'}, 'applied': applied})
        thread.start()
        try:
            host.settimeout(1.)

            def burst():
                for mid in (7, 8, 9):
                    host.sendall(peer.frame((1 << 24) | (0x8000 << 8) | mid, bytes(8)))
                    last = time.monotonic_ns()
                    time.sleep(.0009)
                parser, frames, stamps = ATParser(), [], []
                while len(frames) < 3:
                    frames += parser.feed(host.recv(4096))
                    stamps.append(time.monotonic_ns())
                time.sleep(.003)  # Next burst well after BURST_SPLIT.
                return (stamps[-1]-last)/1e6

            def voltage():
                host.sendall(codec.read_request(8, 'voltage'))
                sent = time.monotonic_ns()
                host.recv(4096)
                received = time.monotonic_ns()
                time.sleep(.003)
                return (received-sent)/1e6
            baseline = burst()
            os.write(control_w, b'hold port0 3000000 5\nvoltage port0 2000000 5\n')
            time.sleep(.01)
            delayed, after = burst(), burst()
            single_delayed, single_after = voltage(), voltage()
        finally:
            host.close(); os.close(stop_w); thread.join(5)
            for fd in (stop_r, control_r, control_w):
                os.close(fd)
            remote.close()
        self.assertGreater(delayed-baseline, 2.5)
        self.assertLess(abs(after-baseline), 1.5)
        self.assertGreater(single_delayed-single_after, 1.5)
        self.assertEqual([(a['stage'], a['port'], a['cycle'], a['extra_ns']) for a in applied],
                         [('hold', 'port0', 5, 3000000), ('voltage', 'port0', 5, 2000000)])


def _row(index, *, command, sample, release=None, gate=8.5, cmd=None, first=.1, out=15., it=17., **extra):
    release = 1_000_000_000+index*20_000_000 if release is None else release
    row = {'index': index, 'label': 'zero_gain_timing', 'release_ns': release, 'begin_ns': release,
           'hold_first_write_ns': release+int(first*1e6), 'final_gate_ns': release+int(gate*1e6),
           'output_last_reply_ns': release+int(out*1e6), 'cycle_end_ns': release+int(it*1e6),
           'command_interval_ms': command, 'sample_interval_ms': sample}
    if cmd is not None:
        row.update(natural_gate_ns=row['final_gate_ns'], command_ns=release+int(cmd*1e6))
    row.update(extra)
    return row


def _write(directory, rows, *, options=None, seed=1, status='COMPLETE_FOUR_BUS_TYPE1_ZERO_GAIN_TIMING', **report):
    directory.mkdir(parents=True)
    value = {'argv': ['run_harness.py', '--duration', '20', '--seed', str(seed)], 'envelope_gap_mode': 'measure',
             'observer': {'kind': 'synthetic'}, 'peer': {'latency_mode': 'empirical'},
             'runner': {'status': status, 'completed_cycles': len(rows), 'post_reply_late_cycles': 0,
                        'primary_error': None if status.startswith('COMPLETE') else {'message': 'Output reply deadline'}},
             'cycles': rows, **report}
    if options is not None:
        value['options'] = options
    (directory/'harness-report.json').write_text(json.dumps(value))
    return directory


class AnalyzeTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.root)

    def test_per_run_counts_separate_natural_gate_and_command(self):
        rows = [_row(0, command=None, sample=None)]+[_row(k, command=20., sample=20.) for k in range(1, 6)]
        rows[3]['command_interval_ms'] = 20.9
        rows[4]['command_interval_ms'] = 21.2; rows[4]['sample_interval_ms'] = 21.1
        value = analyze.analyze(_write(self.root/'a', rows))  # Old report: no options -> default.
        self.assertEqual(value['config'].split(' | ')[0], 'default')
        self.assertEqual((value['command_over_20_8'], value['command_over_21'], value['sample_over_20_8'],
                          value['sample_over_21']), (2, 1, 1, 1))
        self.assertAlmostEqual(value['release_to_natural_gate_ms']['max'], 8.5)
        self.assertAlmostEqual(value['release_to_command_ms']['max'], 8.5)
        paced = [_row(k, command=20., sample=20., gate=8.5+k, cmd=11.25) for k in range(1, 4)]
        value = analyze.analyze(_write(self.root/'b', paced, options={
            'pacing_options': {'command_phase_offset_us': 11250}, 'timing_evidence': True}))
        self.assertTrue(value['config'].startswith('F1=11250+F0 | '))
        self.assertAlmostEqual(value['release_to_natural_gate_ms']['max'], 11.5)
        self.assertAlmostEqual(value['release_to_command_ms']['max'], 11.25)

    def test_per_configuration_pools_seeds_counts_aborts_and_evaluates_acceptance(self):
        options = {'pacing_options': {'command_phase_offset_us': 11250}, 'timing_evidence': False}
        # A complete 20 s run: 989 cycles (indices 0..988), 988 steady intervals.
        good = [_row(k, command=20., sample=20., cmd=11.25) for k in range(989)]
        runs = [_write(self.root/f's{seed}', good, options=options, seed=seed) for seed in (101, 102, 103)]
        bad = good[:501]
        runs.append(_write(self.root/'s104', bad, options=options, seed=104, status='ABORTED'))
        config, = analyze.by_config([analyze.analyze(run) for run in runs])
        self.assertEqual((config['seeds'], config['aborts'], config['steady_cycles']), ([101, 102, 103, 104], 1, 3*988+500))
        self.assertEqual(config['abort_errors'][0]['error'], 'Output reply deadline')
        criteria = config['acceptance']['criteria']
        self.assertTrue(config['acceptance']['harness_emulation_only_not_a_timing_qualification'])
        self.assertFalse(criteria['all_runs_complete']['pass'])
        self.assertTrue(criteria['command_within_20_00_pm_0_01_fraction']['pass'])
        self.assertTrue(criteria['natural_gate_p999_ms']['pass'])
        self.assertEqual(criteria['natural_gate_p999_ms']['limit'], 11.)
        config, = analyze.by_config([analyze.analyze(run) for run in runs[:3]])
        self.assertEqual(config['steady_cycles'], analyze.CYCLES_REQUIRED)
        self.assertTrue(config['acceptance']['all_pass'], config['acceptance'])
        config, = analyze.by_config([analyze.analyze(run) for run in runs[:2]])
        self.assertFalse(config['acceptance']['criteria']['steady_cycles']['pass'])

    def test_prearmed_rows_attribute_late_owners_from_the_release(self):
        # F3, lead 2 ms: Boundary 1 ends 1.85 ms before the release; owners begin natively at release+0.1 ms.
        def prearmed(k, owner=.1, **kw):
            row = _row(k, command=20., sample=20., **kw)
            release = row['release_ns']
            row.update(prearm_wake_ns=release-2_000_000, begin_ns=release+400, current_check1_end_ns=release-1_850_000,
                       hold_last_reply_ns=release+5_000_000, acquired_ns=release+5_600_000)
            row.update({f'hold_{p}_owner_begin_ns': release+int(owner*1e6) for p in analyze.PORTS})
            return row
        rows = [prearmed(k) for k in range(6)]
        rows[2] = prearmed(2, first=1.1)  # Late first write, owners on time: not an owner-start stall.
        rows[2].update(command_interval_ms=21.3, sample_interval_ms=21.1)
        rows[4] = prearmed(4, owner=2.6, first=2.7)
        rows[4].update(command_interval_ms=22.5, sample_interval_ms=22.6)
        value = analyze.analyze(_write(self.root/'f3', rows, options={
            'pacing_options': {'prearmed_hold_lead_us': 2000}, 'timing_evidence': False}))
        causes = {item['index']: item for item in value['violations_attributed']}
        self.assertEqual(causes[2]['late_owner_ports'], [])
        self.assertEqual(causes[2]['cause'], 'host_unattributed_begin_to_hold_first_write')
        self.assertEqual(causes[4]['late_owner_ports'], list(analyze.PORTS))
        self.assertEqual(causes[4]['cause'], 'host_owner_start_late_4_of_4_ports')
        self.assertAlmostEqual(value['stage_ms']['release_to_last_hold_owner_begin']['median'], .1)
        self.assertAlmostEqual(value['stage_ms']['check1_end_to_last_hold_owner_begin']['median'], 1.95)
        default = [_row(k, command=20., sample=20.) for k in range(4)]
        for row in default:
            row.update(current_check1_end_ns=row['release_ns']+155_000,
                       **{f'hold_{p}_owner_begin_ns': row['release_ns']+2_000_000 for p in analyze.PORTS})
        default[2].update(command_interval_ms=21.4, hold_first_write_ns=default[2]['release_ns']+2_100_000)
        item, = analyze.analyze(_write(self.root/'d', default))['violations_attributed']
        self.assertEqual(item['cause'], 'host_owner_start_late_4_of_4_ports')

    def test_prearmed_acceptance_requires_the_cycle_end_to_leave_the_next_prearm_window(self):
        from experiments.four_bus_type1 import type1_profile as P
        self.assertEqual(analyze.PREARM_WORK_MS*1000, P.PREARM_WORK_US)
        options = {'pacing_options': {'command_phase_offset_us': 9200, 'prearmed_hold_lead_us': 1500},
                   'timing_evidence': False}
        def rows(late_end):
            value = [_row(k, command=20., sample=20., cmd=9.2, first=.05, out=15., it=17.) for k in range(989)]
            for row in value:
                row['prearm_wake_ns'] = row['release_ns']-1_500_000
            value[500]['cycle_end_ns'] = value[500]['release_ns']+int(late_end*1e6)
            return value
        for late_end, passed in ((18.5, True), (18.6555, False)):  # 18.6555: harness part3/h-all cycle 56.
            runs = [_write(self.root/f'e{late_end}-{seed}', rows(late_end), options=options, seed=seed)
                    for seed in (101, 102, 103)]
            config, = analyze.by_config([analyze.analyze(run) for run in runs])
            item = config['acceptance']['criteria']['cycle_end_to_next_release_min_ms']
            self.assertAlmostEqual(item['value'], 20-late_end)
            self.assertEqual((item['limit'], item['pass']), (1.5, passed), late_end)
        value = analyze.analyze(_write(self.root/'plain', [_row(k, command=20., sample=20.) for k in range(4)]))
        self.assertAlmostEqual(value['cycle_end_to_next_release_ms']['min'], 3.)  # Reported; no criterion by default.
        config, = analyze.by_config([value])
        self.assertNotIn('cycle_end_to_next_release_min_ms', config['acceptance']['criteria'])

    def test_injected_configuration_uses_no_fault_and_labels_the_cause(self):
        rows = [_row(k, command=20., sample=20.) for k in range(1, 6)]
        rows[2].update(command_interval_ms=22.6, sample_interval_ms=21.9, inject_host_stall_ms=2.6,
                       inject_host_stall_mode='gil')
        selected = {'tails': [], 'stalls': [{'cycle': 3, 'ms': 2.6, 'mode': 'gil'}]}
        value = analyze.analyze(_write(self.root/'i', rows, injections_selected=selected))
        self.assertIn('inject=stall@3:2.6:gil', value['config'])
        self.assertEqual(value['cause_counts'], {'injected_host_stall_gil': 1})
        config, = analyze.by_config([value])
        self.assertEqual(list(config['acceptance']['criteria']), ['no_fault'])
        self.assertFalse(config['acceptance']['all_pass'])

    def test_main_writes_json_without_raw_values(self):
        run = _write(self.root/'m', [_row(k, command=20., sample=20.) for k in range(1, 4)])
        out = self.root/'analysis.json'
        with open(os.devnull, 'w') as sink:
            stdout, sys.stdout = sys.stdout, sink
            try:
                analyze.main([str(run), '--json', str(out)])
            finally:
                sys.stdout = stdout
        value = json.loads(out.read_text())
        self.assertEqual(len(value['runs']), 1)
        self.assertNotIn('_values', value['runs'][0])
        self.assertEqual(len(value['configurations']), 1)


class PerCycleRowsTest(unittest.TestCase):
    @staticmethod
    def rows(cycles, calls):
        from types import SimpleNamespace
        from experiments.four_bus_type1.harness import run_harness
        return run_harness.per_cycle_rows({'cycles': cycles}, SimpleNamespace(rows=[]),
                                          SimpleNamespace(calls=[(a, a+100_000, 1) for a in calls]), None,
                                          SimpleNamespace(profiles=[]))

    def test_prearmed_rows_window_boundary1_from_the_wake_and_survive_an_abort_before_release(self):
        r0, r1, r2 = 1_000_000_000, 1_020_000_000, 1_040_000_000
        cycles = [{'index': 0, 'release_ns': r0, 'prearm_wake_ns': r0-1_000_000, 'begin_ns': r0+400},
                  {'index': 1, 'release_ns': r1, 'prearm_wake_ns': r1-1_000_000, 'begin_ns': r1+400},
                  {'index': 2, 'release_ns': r2, 'prearm_wake_ns': r2-1_000_000, 'begin_ns': None}]
        calls = [r0-900_000, r0+5_700_000, r0+8_500_000, r1-900_000, r1+5_700_000, r1+8_500_000, r2-900_000]
        rows = self.rows(cycles, calls)
        for row, release in zip(rows[:2], (r0, r1)):
            self.assertEqual([row[f'current_check{k}_start_ns']-release for k in (1, 2, 3)],
                             [-900_000, 5_700_000, 8_500_000])
        self.assertIsNone(rows[2]['begin_ns'])
        self.assertEqual(rows[2]['current_check1_start_ns'], r2-900_000)
        self.assertNotIn('current_check2_start_ns', rows[2])
        self.assertIsNone(rows[2]['output_last_reply_after_begin_ms'])
        # A row with neither wake nor begin (never produced by the runner) is windowed by its release.
        cycles[2].pop('prearm_wake_ns')
        rows = self.rows(cycles, calls+[r2+100])
        self.assertEqual(rows[1]['current_check3_start_ns'], r1+8_500_000)
        self.assertEqual(rows[2]['current_check1_start_ns'], r2+100)

    def test_default_rows_window_from_begin_as_before(self):
        r0, r1 = 1_000_000_000, 1_020_000_000
        cycles = [{'index': 0, 'release_ns': r0, 'begin_ns': r0+300}, {'index': 1, 'release_ns': r1, 'begin_ns': r1+300}]
        calls = [r0-5_000_000, r0+150_000, r0+6_000_000, r0+9_000_000, r1+150_000, r1+6_000_000, r1+9_000_000]
        rows = self.rows(cycles, calls)
        for row, release in zip(rows, (r0, r1)):
            self.assertEqual([row[f'current_check{k}_start_ns']-release for k in (1, 2, 3)],
                             [150_000, 6_000_000, 9_000_000])
            self.assertNotIn('prearm_wake_ns', row)


class ReceiptTest(unittest.TestCase):
    def test_prearmed_hold_requires_an_exchange_at_receipt(self):
        from experiments.four_bus_type1.harness import run_harness
        root = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, root)
        library = root/'libdog_four_bus_type1_transport.so'
        (root/'build-record.json').write_text(json.dumps({'four_bus_subset_active': {'type1_exchange_abi': 1}}))
        with self.assertRaisesRegex(RuntimeError, 'exchange_at_abi 1'):
            run_harness.exchange_at_receipt(library)
        (root/'build-record.json').write_text(json.dumps({'four_bus_subset_active': {'exchange_at_abi': 1}}))
        self.assertEqual(run_harness.exchange_at_receipt(library)['kind'], 'native_sda_subset_exchange_at')

    def test_parser_forwards_every_option_and_injection(self):
        from experiments.four_bus_type1 import type1_profile as P
        from experiments.four_bus_type1.harness import run_harness
        args = run_harness.parser().parse_args([
            '--output', 'x', '--command-phase-offset-us', '9400', '--decode-once', '--prearmed-hold-lead-us', '1000',
            '--gc-freeze', '--timing-evidence', '--inject-hold-tail', 'port3:70:+1.8', '--inject-voltage-tail',
            'port1:5:+1.5', '--inject-host-stall', '26:2.6', '--inject-host-stall', '27:2.6:sleep'])
        self.assertEqual(P.pacing_options(P.selected_pacing(P.cli_options(args))),
                         {'command_phase_offset_us': 9400, 'decode_once': True, 'prearmed_hold_lead_us': 1000,
                          'gc_freeze': True})
        self.assertTrue(args.timing_evidence)
        tails, stalls = injection.parse(args.inject_hold_tail, args.inject_voltage_tail, args.inject_host_stall)
        self.assertEqual((len(tails), [s['mode'] for s in stalls]), (2, ['gil', 'sleep']))
        combined = run_harness.parser().parse_args(['--output', 'x', '--command-phase-offset-us', '11250',
                                                    '--prearmed-hold-lead-us', '1000'])
        with self.assertRaisesRegex(ValueError, 'pre-armed hold lead'):  # F1-only K with F3: lead-aware check.
            P.selected_pacing(P.cli_options(combined))
        default = run_harness.parser().parse_args(['--output', 'x'])
        self.assertEqual(P.selected_pacing(P.cli_options(default)), P.PACING)
        self.assertEqual(injection.parse(default.inject_hold_tail, default.inject_voltage_tail,
                                         default.inject_host_stall), ([], []))


@unittest.skipUnless(shutil.which(os.environ.get('CXX', 'c++')), 'C++ compiler required for the end-to-end harness')
class EndToEndTest(unittest.TestCase):
    """Two 2 s motor-free runs on the built library (strict mode stays the default and is not used here)."""
    @classmethod
    def setUpClass(cls):
        from experiments.four_bus_type1 import build
        cls.root = Path(tempfile.mkdtemp())
        cls.library = build.build(cls.root/'library')

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root)

    def run_harness(self, name, *extra):
        output = self.root/name
        env = dict(os.environ, PYTHONPATH=str(RUNTIME))
        subprocess.run([sys.executable, '-B', '-m', 'experiments.four_bus_type1.harness.run_harness',
                        '--output', str(output), '--library', str(self.library), '--duration', '2',
                        '--envelope-gap-mode', 'measure', *extra], cwd=RUNTIME, env=env, check=True,
                       stdout=subprocess.DEVNULL)
        return output, json.loads((output/'harness-report.json').read_text())

    def test_default_run_selects_and_wraps_nothing(self):
        output, report = self.run_harness('default')
        self.assertEqual(report['options'], {'pacing_options': {}, 'timing_evidence': False})
        for key in ('injections', 'injections_selected', 'command_wait', 'exchange_at'):
            self.assertNotIn(key, report)
        self.assertNotIn('injections_applied', json.loads((output/'peer-log.json').read_text()))
        self.assertEqual(report['runner']['pacing'], json.loads(json.dumps(
            __import__('experiments.four_bus_type1.type1_profile', fromlist=['PACING']).PACING)))
        rows = [r for r in report['cycles'] if r.get('final_gate_ns') is not None]
        self.assertGreater(len(rows), 10)
        for row in rows:
            self.assertNotIn('command_ns', row)
            self.assertEqual(row['natural_gate_after_release_ms'], row['command_after_release_ms'])
            self.assertFalse(any(key.startswith('inject_') for key in row))
        self.assertIn('over_20_8ms', report['summary']['intervals']['command_interval_ms'])

    def test_all_options_with_every_injection(self):
        output, report = self.run_harness(
            'all', '--command-phase-offset-us', '9200', '--decode-once', '--prearmed-hold-lead-us', '1500',
            '--gc-freeze', '--timing-evidence', '--inject-hold-tail', 'port3:10:+1.8', '--inject-voltage-tail',
            'port1:15:+1.5', '--inject-host-stall', '20:2.6:gil', '--inject-host-stall', '25:2.6:sleep')
        self.assertEqual(report['options']['pacing_options'], {'command_phase_offset_us': 9200, 'decode_once': True,
                                                               'prearmed_hold_lead_us': 1500, 'gc_freeze': True})
        self.assertEqual(report['exchange_at']['kind'], 'native_sda_subset_exchange_at')
        self.assertIn('command_wait', report)
        injected = report['injections']
        self.assertEqual(injected['tails_not_applied'], [])
        self.assertEqual([(s['cycle'], s['hook']) for s in injected['stalls_performed']],
                         [(20, 'after_actual_release_wait_prearmed'), (25, 'after_actual_release_wait_prearmed')])
        rows = {r['index']: r for r in report['cycles']}
        self.assertEqual(rows[10]['inject_hold_tail_port3_ms'], 1.8)
        self.assertEqual(rows[15]['inject_voltage_tail_port1_ms'], 1.5)
        self.assertEqual(rows[20]['inject_host_stall_mode'], 'gil')
        self.assertGreater(rows[10]['hold_port3_modelled_delay_excess_ms'], 1.5)
        paced = [r['command_after_release_ms'] for r in rows.values() if r.get('command_ns') is not None and
                 r['natural_gate_after_release_ms'] < 9.2]
        self.assertTrue(paced and all(9.2 <= value < 9.45 for value in paced))
        self.assertTrue(all('prearm_wake_ns' in r for r in rows.values()))
        for row in rows.values():  # F3: Boundary 1 before the release, Boundaries 2 and 3 after the hold.
            if row.get('current_check3_start_ns') is not None:
                self.assertTrue(row['prearm_wake_ns'] <= row['current_check1_start_ns'] < row['release_ns'] <
                                row['current_check2_start_ns'] < row['current_check3_start_ns'] <= row['final_gate_ns'])
        summary, = analyze.by_config([analyze.analyze(output)])
        self.assertIn('F1=9200+F2b+F3=1500+F4+F0', summary['config'])
        self.assertEqual(list(summary['acceptance']['criteria']), ['no_fault'])


if __name__ == '__main__':
    unittest.main()
