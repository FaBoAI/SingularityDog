"""File/mock admission and lifecycle tests; no native, model or devices."""
from contextlib import ExitStack, contextmanager, redirect_stdout, redirect_stderr
import hashlib
import io
import json
import itertools
from pathlib import Path
import stat
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from . import foreground as fg


def arguments():
    values = []
    for name in ('source-manifest', 'source-binding', 'original-model-profile',
                 'topology', 'events', 'subset-library', 'current-guard-library'):
        values += ['--'+name, '/unused/'+name, '--'+name+'-sha256', 'a'*64]
    return values+['--source-root', '/unused/kit', '--source-count', '840',
        '--subset-build-sha256', 'b'*64, '--subset-source-sha256', 'c'*64,
        '--ordinary-source-sha256', 'd'*64, '--current-guard-build-sha256', 'e'*64,
        '--current-guard-source-sha256', 'f'*64, '--power-epoch', 'user-stated-on',
        '--output', '/unused/fresh']


@contextmanager
def fake_settings(*, slack_exit_error=None, fail_switch=False):
    """All actual CPU/slack/GC/interpreter controls are replaced."""
    state = {'cpus': {0, 1, 2, 3, 4}, 'switch': .005, 'gc': True, 'events': []}
    class Slack:
        def __init__(self, requested):
            self.report = {'parent': {'original_ns': 50_000, 'during_ns': requested}}
        def __enter__(self):
            state['events'].append('slack-enter'); return self.report
        def __exit__(self, *args):
            state['events'].append('slack-exit')
            self.report['parent'].update(after_ns=50_000, restored=slack_exit_error is None)
            if slack_exit_error: raise slack_exit_error
    def set_cpu(pid, cpus):
        state['events'].append(('cpu', set(cpus))); state['cpus'] = set(cpus)
    def set_switch(value):
        state['events'].append(('switch', value))
        if fail_switch and value < .001: raise RuntimeError('switch setup rejected')
        # CPython stores integer microseconds; nextafter avoids downward rounding.
        state['switch'] = int(value*1_000_000)/1_000_000
    with ExitStack() as stack:
        stack.enter_context(patch.object(fg.os, 'sched_getaffinity', create=True,
                                        side_effect=lambda pid: set(state['cpus'])))
        stack.enter_context(patch.object(fg.os, 'sched_setaffinity', create=True, side_effect=set_cpu))
        stack.enter_context(patch.object(fg.os, 'getpriority', return_value=-10))
        stack.enter_context(patch.object(fg.sys, 'getswitchinterval', side_effect=lambda: state['switch']))
        stack.enter_context(patch.object(fg.sys, 'setswitchinterval', side_effect=set_switch))
        stack.enter_context(patch.object(fg.gc, 'isenabled', side_effect=lambda: state['gc']))
        stack.enter_context(patch.object(fg.gc, 'enable', side_effect=lambda: state.update(gc=True)))
        stack.enter_context(patch.object(fg.gc, 'disable', side_effect=lambda: state.update(gc=False)))
        stack.enter_context(patch.object(fg.gc, 'collect', return_value=7))
        stack.enter_context(patch('singularitydog_hw.thread_timer_slack.TimerSlack', Slack))
        yield state


@contextmanager
def fake_guard_files():
    def info(mode, inode, device=7, rdev=0):
        return SimpleNamespace(st_dev=device, st_ino=inode, st_mode=mode, st_rdev=rdev,
                               st_size=13, st_mtime_ns=100, st_ctime_ns=101)
    bindings, descriptors, links, nodes, fd_nodes = {}, {}, {}, {}, {}
    for index, port in enumerate(fg.topology.PORTS):
        path, resolved = '/dev/serial/by-path/usb'+str(index), '/dev/ttyUSB'+str(index)
        target = info(stat.S_IFCHR|0o600, 100+index, rdev=188+index)
        bindings[port] = {'path': path, 'resolved': resolved, 'st_rdev': target.st_rdev}
        descriptors[port] = 10+index
        nodes[path] = info(stat.S_IFLNK|0o777, 200+index)
        nodes[resolved] = target; fd_nodes[10+index] = target
        links[path] = '../../ttyUSB'+str(index)
    for index, path in enumerate(('/dev/serial/by-path', '/dev/serial', '/dev', '/')):
        nodes[path] = info(stat.S_IFDIR|0o755, 300+index)
    fd_nodes[99] = info(stat.S_IFREG|0o444, 400)
    resolved_by_alias = {row['path']: row['resolved'] for row in bindings.values()}
    def followed(path):
        path = str(path)
        if path in resolved_by_alias:
            return nodes[resolved_by_alias[path]]
        return nodes[path]
    clock = itertools.count(1)
    stop = threading.Event()
    with ExitStack() as stack:
        resolve = stack.enter_context(patch.object(fg.topology, 'check_bindings'))
        stack.enter_context(patch.object(fg.os, 'lstat', side_effect=lambda path: nodes[str(path)]))
        stack.enter_context(patch.object(fg.os, 'stat', side_effect=followed))
        stack.enter_context(patch.object(fg.os, 'fstat', side_effect=lambda fd: fd_nodes[fd]))
        readlink = stack.enter_context(patch.object(fg.os, 'readlink', side_effect=lambda path: links[str(path)]))
        pread = stack.enter_context(patch.object(fg.os, 'pread', return_value=b'boot\n'))
        guard = fg.CurrentGuard(bindings, descriptors, 99, 'boot', stop, clock=lambda: next(clock))
        yield SimpleNamespace(guard=guard, bindings=bindings, nodes=nodes, links=links,
            fd_nodes=fd_nodes, stop=stop, pread=pread, readlink=readlink, resolve=resolve, info=info)


class ForegroundTests(unittest.TestCase):
    def test_default_plan_never_calls_execution_or_startup(self):
        plan = {'status': 'PLAN', **fg.FALSE_FLAGS}
        with patch.object(fg, 'prepare', return_value={'plan': plan}) as prepared, \
                patch.object(fg, 'execute') as execute, patch.object(fg, 'startup_readback') as startup, \
                redirect_stdout(io.StringIO()) as output:
            self.assertEqual(fg.main(arguments()), 0)
        self.assertFalse(prepared.call_args.args[0].execute)
        execute.assert_not_called(); startup.assert_not_called()
        self.assertIs(json.loads(output.getvalue())['output_allowed'], False)

    def test_cycles_reject_live_durations_before_prepare(self):
        with patch.object(fg, 'prepare') as prepared, redirect_stderr(io.StringIO()):
            for count in ('2', '10', '20', '30', '500'):
                with self.assertRaises(SystemExit): fg.main(arguments()+['--cycles', count])
        prepared.assert_not_called()

    def test_explicit_failed_execute_retains_false_grants(self):
        prepared = {'output': Path('/unused/fresh')}
        with patch.object(fg, 'prepare', return_value=prepared), \
                patch.object(fg, 'execute', return_value={'status': 'ABORTED', 'failure_retained': True}), \
                redirect_stdout(io.StringIO()) as output:
            self.assertEqual(fg.main(arguments()+['--execute']), 2)
        self.assertIs(json.loads(output.getvalue())['live_type1_qualified'], False)

    def test_pins_reject_changed_bytes_conflicting_pin_and_symlink(self):
        with tempfile.TemporaryDirectory() as value:
            root = Path(value).resolve(); path = root/'input.json'; path.write_bytes(b'{"x":1}')
            digest = hashlib.sha256(path.read_bytes()).hexdigest(); pins = fg.Pins()
            self.assertEqual(pins.json(path, digest), {'x': 1})
            with self.assertRaisesRegex(ValueError, 'Conflicting'): pins.read(path, 'a'*64)
            alias = root/'alias'; alias.symlink_to(path)
            with self.assertRaises(ValueError): fg.Pins().read(alias, digest)
            path.write_bytes(b'{"x":2}')
            with self.assertRaises(ValueError): pins.verify()

    def test_full_inventory_rejects_unlisted_files_and_mode_changes(self):
        with tempfile.TemporaryDirectory() as value:
            root = Path(value).resolve(); path = root/'code.py'; path.write_bytes(b'pass\n'); path.chmod(0o644)
            manifest = {'files': {'code.py': {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'bytes': 5, 'mode': 0o644}}, 'file_count': 1, 'output_allowed': False,
                'approved_for_runtime': False}
            fg.verify_manifest(root, manifest, 1, fg.Pins())
            (root/'extra.py').write_text('pass\n')
            with self.assertRaisesRegex(ValueError, 'Unlisted'): fg.verify_manifest(root, manifest, 1, fg.Pins())
            (root/'extra.py').unlink(); path.chmod(0o600)
            with self.assertRaisesRegex(ValueError, 'mode changed'): fg.verify_manifest(root, manifest, 1, fg.Pins())

    def test_foreign_cached_module_rejected(self):
        module = SimpleNamespace(__file__='/foreign/runtime/singularitydog_hw/policy_observer.py')
        with patch.object(fg.sys, 'modules', {'singularitydog_hw.policy_observer': module}):
            with self.assertRaisesRegex(ValueError, 'origin differs'):
                fg.verify_origins(Path('/selected'), {'files': {}})

    def test_main_scope_exact_success_and_primary_error_cleanup(self):
        with fake_settings() as state:
            scope = fg.MainScope(True, True)
            with scope as during:
                self.assertEqual(during['main_cpu_mask'], [4]); self.assertFalse(state['gc'])
                self.assertEqual(during['timer_slack_ns'], 1000)
            self.assertTrue(scope.report['restored']); self.assertEqual(state['switch'], .005)
            self.assertEqual(state['cpus'], {0, 1, 2, 3, 4}); self.assertTrue(state['gc'])
        primary = RuntimeError('original native failure')
        with fake_settings(slack_exit_error=RuntimeError('slack restore failed')) as state:
            scope = fg.MainScope(True, True); scope.__enter__()
            self.assertFalse(scope.__exit__(RuntimeError, primary, None))
            self.assertFalse(scope.report['restored'])
            self.assertEqual(state['cpus'], {0, 1, 2, 3, 4}); self.assertEqual(state['switch'], .005)
            self.assertTrue(state['gc']); self.assertIn('slack restore failed', primary.__notes__[0])

    def test_scope_enter_failure_rolls_back_prior_changes(self):
        with fake_settings(fail_switch=True) as state:
            scope = fg.MainScope(True, True)
            with self.assertRaisesRegex(RuntimeError, 'switch setup rejected'): scope.__enter__()
            self.assertTrue(scope.report['restored']); self.assertEqual(state['cpus'], {0, 1, 2, 3, 4})
            self.assertEqual(state['switch'], .005); self.assertTrue(state['gc'])

    def test_selected_two_ten_warm_stages_owned_buffers_and_reset(self):
        tensors = (object(),)*6; observer = SimpleNamespace(_input_tensors=tensors, prepare_run=Mock())
        policy, wrapper, torch = object(), object(), object(); calls = []
        with fake_settings() as state, patch('singularitydog_hw.policy_observer_replay.warmup_policy',
                side_effect=lambda *a, **k: calls.append((a, k, set(state['cpus'])))):
            state['cpus'] = {4}
            result = fg.selected_warmup(observer, policy, wrapper, torch, 'h', pre_calls=10, post_calls=10)
        self.assertEqual(len(calls), 2)
        for args, kwargs, cpus in calls:
            self.assertEqual(args, (policy, torch, 'h', 10))
            self.assertIs(kwargs['checked_dispatch_wrapper'], wrapper)
        self.assertEqual(calls[0][2], {0, 1, 2, 3, 4}); self.assertEqual(calls[1][2], {4})
        self.assertNotIn('input_tensors', calls[0][1]); self.assertIs(calls[1][1]['input_tensors'], tensors)
        observer.prepare_run.assert_called_once_with(warmup_completed=True)
        self.assertTrue(result['reset_verified'])

    def test_selected_warm_error_fatal_and_cpu_rollback_no_reset(self):
        observer = SimpleNamespace(_input_tensors=(object(),)*6, prepare_run=Mock())
        with fake_settings() as state, patch('singularitydog_hw.policy_observer_replay.warmup_policy',
                side_effect=ValueError('selected range failure')):
            state['cpus'] = {4}
            with self.assertRaisesRegex(ValueError, 'selected range failure'):
                fg.selected_warmup(observer, object(), object(), object(), 'h', pre_calls=10, post_calls=10)
            self.assertEqual(state['cpus'], {4})
        observer.prepare_run.assert_not_called()

    def test_release_uses_original_wait_500_and_arms_scheduled_epoch_once(self):
        observer = SimpleNamespace(arm_run=Mock()); library = object()
        with patch('singularitydog_hw.native_active_transport.wait_until', side_effect=(123, 456)) as native:
            release = fg.make_release_wait(library, 9, observer)
            self.assertEqual(release(100), 123); self.assertEqual(release(400), 456)
        observer.arm_run.assert_called_once_with(100)
        self.assertEqual(native.call_args_list[0].args, (library, 9, 100))
        self.assertEqual(native.call_args_list[0].kwargs, {'spin_us': 500})

    def test_cancel_binding_cleared_before_fd_close_on_exception(self):
        binding = fg.CancelBinding(); events = []
        with patch.object(fg.os, 'write') as write:
            with self.assertRaisesRegex(RuntimeError, 'original'):
                with ExitStack() as stack:
                    stack.callback(lambda: (events.append(binding.writer), binding.cancel()))
                    binding.bind(stack, 7)
                    binding.cancel(); write.assert_called_once_with(7, b'x')
                    raise RuntimeError('original')
            self.assertEqual(events, [None]); self.assertIsNone(binding.writer)
            binding.cancel(); self.assertEqual(write.call_count, 1); self.assertTrue(binding.stop.is_set())

    def test_cancel_write_error_retained_without_masking_primary(self):
        binding = fg.CancelBinding()
        with ExitStack() as stack, patch.object(fg.os, 'write', side_effect=OSError('bad writer')):
            binding.bind(stack, 7); binding.cancel()
            self.assertEqual(binding.errors, ['bad writer']); self.assertTrue(binding.stop.is_set())

    def test_half_six_raw_bounds_retains_signed_global_transform(self):
        axes = {str(mid): {'global_bounds_rad': [-1., 2.], 'sign': -1, 'fixed_offset_rad': .25}
                for mid in range(1, 7)}
        lower, upper = fg.owner_bounds({'axes': axes}, fg.transport_adapter.Group('port0', (1, 2, 3)))
        self.assertEqual(set(lower), set(range(1, 7))); self.assertEqual(lower[6], -1.75)
        self.assertEqual(upper[6], 1.25)

    def test_optional_root_holder_observation_binds_exact_current_ports(self):
        capture = {'boot_before': 'boot', 'ports': {'port0': {'st_rdev': 1}}, 'finished_monotonic_ns': 100}
        receipt = {'schema': 'singularitydog.four-bus-root-holder-check.v1', 'run_as_uid': 0,
            'boot_id': 'boot', 'ports': capture['ports'], 'holders': [], 'complete_fd_inventory': True,
            'checked_monotonic_ns': 101, 'output_allowed': False}
        self.assertIs(fg.validate_holder_receipt(receipt, capture), receipt)
        for key, value in (('boot_id', 'old'), ('ports', {}), ('holders', [{'pid': 7}]),
                           ('checked_monotonic_ns', 99), ('output_allowed', True)):
            with self.assertRaises(ValueError): fg.validate_holder_receipt({**receipt, key: value}, capture)

    def test_imu_original_deadline_timeout_and_cancellation_preserved(self):
        sample = {'accel_m_s2': [0, 0, 9.8]}; device = SimpleNamespace(read_sample=Mock(side_effect=[None, sample]))
        clock = iter((0, 1, 2)); check = Mock(); sleep = Mock()
        self.assertIs(fg.read_imu(device, check, clock=lambda: next(clock), sleep=sleep), sample)
        self.assertEqual(check.call_count, 2); sleep.assert_called_once_with(.0005)
        with self.assertRaisesRegex(TimeoutError, 'original20ms'):
            values = iter((0, 20_000_000))
            fg.read_imu(device, check, clock=lambda: next(values), sleep=sleep)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            values = iter((0, 1))
            fg.read_imu(device, Mock(side_effect=RuntimeError('cancelled')), clock=lambda: next(values), sleep=sleep)

    def test_hot_guard_uses_pread_original_offset_no_resolve_and_sealed_copy(self):
        with fake_guard_files() as fixture:
            fixture.bindings['port0']['path'] = '/foreign'
            fixture.guard(); fixture.guard()
            self.assertEqual(fixture.resolve.call_count, 1)
            self.assertTrue(all(call.args == (99, 128, 0) for call in fixture.pread.call_args_list))
            self.assertEqual(len(fixture.guard.calls), 2)
            for row in fixture.guard.calls:
                self.assertLess(row['started_ns'], row['boot_checked_ns'])
                self.assertLess(row['boot_checked_ns'], row['ancestors_checked_ns'])
                self.assertLess(row['ancestors_checked_ns'], row['ports_checked_ns'])
                self.assertLess(row['ports_checked_ns'], row['finished_ns'])
                self.assertTrue(row['ok'])

    def test_hot_guard_rejects_link_text_even_same_inode_and_target(self):
        with fake_guard_files() as fixture:
            fixture.links['/dev/serial/by-path/usb0'] = '../../other-link-to-same-device'
            with self.assertRaisesRegex(ValueError, 'device identity changed'): fixture.guard()
            self.assertFalse(fixture.guard.calls[-1]['ok'])
            self.assertIn('finished_ns', fixture.guard.calls[-1])

    def test_hot_guard_rejects_alias_recreation_and_changed_ancestor(self):
        with fake_guard_files() as fixture:
            fixture.nodes['/dev/serial/by-path/usb0'].st_ino += 1
            with self.assertRaisesRegex(ValueError, 'device identity changed'): fixture.guard()
        for changed in ('st_ino', 'st_mode'):
            with fake_guard_files() as fixture:
                node = fixture.nodes['/dev/serial']
                setattr(node, changed, node.st_ino+1 if changed == 'st_ino' else stat.S_IFLNK|0o777)
                with self.assertRaisesRegex(ValueError, 'ancestor identity changed'): fixture.guard()

    def test_hot_guard_rejects_target_inode_rdev_type_and_cross_port_alias(self):
        for key, value in (('st_ino', 900), ('st_rdev', 999), ('st_mode', stat.S_IFREG|0o600)):
            with fake_guard_files() as fixture:
                setattr(fixture.nodes['/dev/ttyUSB0'], key, value)
                with self.assertRaisesRegex(ValueError, 'device identity changed'): fixture.guard()
        with fake_guard_files() as fixture:
            fixture.links['/dev/serial/by-path/usb0'] = '../../ttyUSB1'
            with self.assertRaises(ValueError): fixture.guard()

    def test_hot_guard_boot_changed_partial_read_fd_reuse_cancel_and_end_after_failure(self):
        for data in (b'other-boot\n', b'boo', b''):
            with fake_guard_files() as fixture:
                fixture.pread.return_value = data
                with self.assertRaisesRegex(ValueError, 'boot FD/bytes changed'): fixture.guard()
                self.assertFalse(fixture.guard.calls[-1]['ok'])
        with fake_guard_files() as fixture:
            fixture.fd_nodes[99] = fixture.info(stat.S_IFREG|0o444, 999)
            with self.assertRaisesRegex(ValueError, 'boot FD/bytes changed'): fixture.guard()
        with fake_guard_files() as fixture:
            fixture.pread.side_effect = OSError('closed original boot FD')
            with self.assertRaisesRegex(OSError, 'closed original'): fixture.guard()
            self.assertFalse(fixture.guard.calls[-1]['ok'])
        with fake_guard_files() as fixture:
            fixture.stop.set()
            with self.assertRaisesRegex(ValueError, 'cancelled'): fixture.guard()
            report = fixture.guard.finish()
            self.assertEqual(fixture.resolve.call_count, 2)
            self.assertTrue(report['end_full_path_resolution_verified'])
            self.assertFalse(report['calls'][-1]['cancellation_check_applied'])
            self.assertTrue(fixture.stop.is_set())

    def test_hot_guard_concurrent_original_pread_uses_no_shared_offset(self):
        with fake_guard_files() as fixture:
            threads = [threading.Thread(target=fixture.guard) for _ in range(5)]
            for thread in threads: thread.start()
            for thread in threads: thread.join()
            self.assertEqual(len(fixture.guard.calls), 5)
            self.assertTrue(all(row['ok'] for row in fixture.guard.calls))
            self.assertTrue(all(call.args == (99, 128, 0) for call in fixture.pread.call_args_list))

    def test_projection_timer_retains_failure_and_stage_original_clock_nulls(self):
        rows = []; values = iter((10, 25, 30, 40)); marker = object()
        builder = fg.timed_snapshot_builder(Mock(side_effect=(marker, ValueError('original decode'))), rows,
                                            clock=lambda: next(values))
        self.assertIs(builder({}, {}, 9), marker)
        with self.assertRaisesRegex(ValueError, 'original decode'): builder({}, {}, 20)
        self.assertEqual([row['ok'] for row in rows], [True, False])
        self.assertEqual(rows[0]['finished_ns']-rows[0]['started_ns'], 15)
        stages = fg.stage_timings({'records': [{'cycle': 0, 'release_ns': 100,
            'gather_end_ns': 150, 'infer_end_ns': 180, 'completed': False}, {'cycle': 1, 'release_ns': 200}]})
        self.assertEqual(stages[0]['gather_wall_ns'], 50)
        self.assertEqual(stages[0]['inference_wall_ns'], 30)
        self.assertIsNone(stages[1]['gather_wall_ns']); self.assertIsNone(stages[0]['cycle_end_ns'])

    def test_native_guard_end_failure_still_closes_before_descriptor_teardown(self):
        events = []; errors = []; report = {}
        primary = RuntimeError('original exchange failure')
        def note(error):
            errors.append(error); primary.add_note('cleanup: '+str(error))
        guard = SimpleNamespace(setup={'sealed': True}, calls=[{'ok': False}],
            finish=Mock(side_effect=ValueError('original end identity changed')),
            close=Mock(side_effect=lambda: events.append('native-close')))
        with ExitStack() as stack:
            stack.callback(lambda: events.append('boot-serial-close'))
            stack.callback(fg.finish_current_guard, guard, report, note)
        self.assertEqual(events, ['native-close', 'boot-serial-close'])
        self.assertFalse(report['current_check_profile']['end_full_path_resolution_verified'])
        self.assertTrue(report['current_check_profile']['native_guard_closed'])
        self.assertEqual(report['current_check_profile']['calls'], [{'ok': False}])
        self.assertEqual(len(errors), 1); self.assertEqual(str(primary), 'original exchange failure')

    def test_native_guard_close_failure_retains_completed_end_profile(self):
        report, errors = {}, []
        result = {'setup': {'sealed': True}, 'calls': [{'native_started_ns': 10, 'native_finished_ns': 20}],
                  'end_full_path_resolution_verified': True}
        guard = SimpleNamespace(finish=Mock(return_value=result), close=Mock(side_effect=RuntimeError('close busy')))
        fg.finish_current_guard(guard, report, errors.append)
        self.assertIs(report['current_check_profile'], result)
        self.assertFalse(result['native_guard_closed']); self.assertEqual(str(errors[0]), 'close busy')

    def test_native_guard_plan_authenticates_files_without_import_or_library_load(self):
        with tempfile.TemporaryDirectory() as value:
            root = Path(value).resolve(); library = root/'libdog_current_guard.so'; library.write_bytes(b'not-a-library')
            source = root/'native_current_guard.cpp'; source.write_bytes(b'// exact source\n')
            source_hash, binary_hash = (hashlib.sha256(p.read_bytes()).hexdigest() for p in (source, library))
            record = {'schema': 'singularitydog.four-bus-current-guard-build.v1', 'abi': 1,
                'source_sha256': source_hash, 'source_bytes': len(source.read_bytes()), 'binary_sha256': binary_hash,
                'CAN_IO_available': False, 'output_allowed': False, 'timing_admission_eligible': False}
            build = root/'build-record.json'; build.write_text(json.dumps(record))
            args = SimpleNamespace(current_guard_library=str(library), current_guard_library_sha256=binary_hash,
                current_guard_build_sha256=hashlib.sha256(build.read_bytes()).hexdigest(),
                current_guard_source_sha256=source_hash)
            manifest = {'files': {'runtime/experiments/four_bus_diagnostic/native_current_guard.cpp':
                                 {'sha256': source_hash}}}
            with patch('ctypes.CDLL', side_effect=AssertionError('PLAN loaded native library')):
                result = fg.prepare_native_guard(args, manifest, fg.Pins())
                self.assertFalse(result['loads_library_in_plan'])
                self.assertFalse(result['CAN_IO_available'])
                for key, changed in (('abi', 2), ('CAN_IO_available', True), ('source_bytes', 0),
                                     ('binary_sha256', 'a'*64)):
                    build.write_text(json.dumps({**record, key: changed}))
                    args.current_guard_build_sha256 = hashlib.sha256(build.read_bytes()).hexdigest()
                    with self.assertRaisesRegex(ValueError, 'contract required'):
                        fg.prepare_native_guard(args, manifest, fg.Pins())

    def test_native_guard_plan_rejects_source_membership_and_dirty_bytes(self):
        pins = Mock(); pins.read.side_effect = (b'binary', b'source')
        args = SimpleNamespace(current_guard_library='/fixture/lib.so', current_guard_library_sha256='a'*64,
            current_guard_build_sha256='b'*64, current_guard_source_sha256='c'*64)
        pins.json.return_value = {'schema': 'singularitydog.four-bus-current-guard-build.v1', 'abi': 1,
            'source_sha256': 'c'*64, 'binary_sha256': 'a'*64, 'source_bytes': 6,
            'CAN_IO_available': False, 'output_allowed': False, 'timing_admission_eligible': False}
        with self.assertRaisesRegex(ValueError, 'contract required'):
            fg.prepare_native_guard(args, {'files': {}}, pins)
        pins.read.side_effect = ValueError('SHA mismatch original file')
        with self.assertRaisesRegex(ValueError, 'SHA mismatch'):
            fg.prepare_native_guard(args, {'files': {}}, pins)

    def test_combined_acquisition_cli_is_explicit_default_off(self):
        ordinary = fg.parser().parse_args(arguments())
        selected = fg.parser().parse_args(arguments()+['--combined-acquisition'])
        self.assertIs(ordinary.combined_acquisition, False)
        self.assertIs(selected.combined_acquisition, True)
        self.assertFalse(selected.execute)
        with patch.object(fg, 'prepare', return_value={'plan': {'status': 'PLAN'}}) as prepared, \
                patch.object(fg, 'execute') as execute, redirect_stdout(io.StringIO()):
            self.assertEqual(fg.main(arguments()+['--combined-acquisition']), 0)
        self.assertIs(prepared.call_args.args[0].combined_acquisition, True)
        execute.assert_not_called()

    def test_combined_config_preserves_model_capture_offsets_and_exact_boolean(self):
        profile = {'axes': {'1': {'unchanged': True}}, 'combined_acquisition': True}
        model_plan = {'measured_input_profile': profile,
                      'runtime_offsets_by_id': {str(mid): .25 for mid in range(1, 13)}}
        capture = {'ids_by_port': {port: list(group) for port, group in
            zip(fg.topology.PORTS, ((1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12)))}}
        def config(*args, **kwargs):
            return SimpleNamespace(groups=args[0], profile=args[1], offsets=args[2], topology=args[3],
                                   cycles=args[4], combined_acquisition=kwargs['combined_acquisition'])
        with patch.object(fg.pipeline, 'Config', side_effect=config):
            for selected in (False, True):
                result = fg.make_config(SimpleNamespace(cycles=501, combined_acquisition=selected), model_plan, capture)
                self.assertIs(result.profile, profile); self.assertIs(result.topology, capture)
                self.assertEqual(result.cycles, 501); self.assertIs(result.combined_acquisition, selected)
                self.assertEqual(set(result.offsets), set(range(1, 13)))
                self.assertEqual(tuple(group.ids for group in result.groups),
                                 ((1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12)))
            with self.assertRaisesRegex(ValueError, 'boolean required'):
                fg.make_config(SimpleNamespace(cycles=5, combined_acquisition=1), model_plan, capture)

    def test_snapshot_route_only_explicit_selector_and_legacy_callback_unchanged(self):
        plan = {'unchanged_model': object()}; legacy, combined = object(), object()
        with patch.object(fg.model_bridge, 'batch_snapshot_builder', return_value=legacy) as old, \
                patch.object(fg.model_bridge, 'combined_batch_snapshot_builder', create=True,
                             return_value=combined) as new:
            self.assertIs(fg.select_snapshot_builder(plan, combined_acquisition=False), legacy)
            old.assert_called_once_with(plan); new.assert_not_called()
            self.assertIs(fg.select_snapshot_builder(plan, combined_acquisition=True), combined)
            new.assert_called_once_with(plan); self.assertEqual(old.call_count, 1)
            with self.assertRaisesRegex(ValueError, 'boolean required'):
                fg.select_snapshot_builder(plan, combined_acquisition=1)


if __name__ == '__main__':
    unittest.main()
