"""Mocked CLI startup/JSON and real comparison collector, no device opens."""
from array import array
import contextlib
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import io
import json
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import native_feedback_compare as compare
from test_native_feedback_compare import BOOT, UIDS, FakeSession
from test_thread_timer_slack import FakePrctl


class MockPort:
    """Only a newly created regular temp file; never a tty or network FD."""
    def __init__(self, backing):
        self.backing = backing
        self.fd = None
        self.opened = False
        self.closed = False
        self.dtr = self.rts = None
        self.port = None

    def open(self):
        self.fd = os.open(self.backing, os.O_CREAT | os.O_RDWR, 0o600)
        self.opened = True

    def fileno(self):
        return self.fd

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.closed = True


class NativeBenchmarkCLITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.uid_file = self.root / "uids.json"
        self.uid_file.write_text(json.dumps(UIDS))
        self.boot_file = self.root / "boot-id"
        self.boot_file.write_text(BOOT+"\n")
        self.output = self.root / "result"
        self.ports, self.sessions, self.boot_fds, self.lock_events = [], {}, [], []
        self.fail_at = None
        self.corrupt_uid = False
        self.expected_stop_proxy = True
        self.guard = Mock(boot_id=BOOT)
        self.patches = ExitStack()
        self.addCleanup(self.patches.close)
        # If the test fails before normal CLI cleanup, close only our own temp FDs.
        self.addCleanup(self.close_temp_fds)
        self.real_open = os.open

        def guarded_open(path, flags, *args, **kwargs):
            if os.fspath(path) == "/proc/sys/kernel/random/boot_id":
                fd = self.real_open(self.boot_file, flags, *args, **kwargs)
                self.boot_fds.append(fd)
                return fd
            return self.real_open(path, flags, *args, **kwargs)

        def serial_constructor(**kwargs):
            self.assertIsNone(kwargs["port"])
            port = MockPort(self.root / ("mock-port-"+str(len(self.ports))))
            self.ports.append(port)
            return port

        def session_constructor(library, fd, *, first_id, cancel_fd, boot_fd, boot_id, stop_proxy,
                                gap_ns, window):
            self.assertEqual(boot_id, BOOT)
            self.assertEqual(stop_proxy, self.expected_stop_proxy)
            session = FakeSession(first_id, fail_at=self.fail_at if first_id == 1 else None,
                                  corrupt_uid=self.corrupt_uid if first_id == 1 else False)
            session.boot_fd = boot_fd
            session.stop_proxy = stop_proxy
            session.gap_ns = gap_ns
            session.window = window
            self.sessions[first_id] = session
            return session

        @contextlib.contextmanager
        def lock(name):
            self.lock_events.append((name, "enter"))
            try:
                yield
            finally:
                self.lock_events.append((name, "exit"))

        self.serial_constructor = Mock(side_effect=serial_constructor)
        self.torch = types.SimpleNamespace(set_num_threads=Mock(), set_num_interop_threads=Mock())
        self.patches.enter_context(patch.dict("sys.modules", {
            "serial": types.SimpleNamespace(Serial=self.serial_constructor), "torch": self.torch}))
        self.patches.enter_context(patch.object(bench.os, "open", side_effect=guarded_open))
        self.patches.enter_context(patch.object(bench.signal, "signal", return_value=None))
        self.patches.enter_context(patch.object(bench.native, "load_library", return_value=object()))
        self.native_session = self.patches.enter_context(patch.object(
            bench.native, "NativeSession", side_effect=session_constructor))
        self.patches.enter_context(patch.object(bench.dual, "validate_ports", return_value={
            scope: dict(path="mock-"+scope, resolved="mock-"+scope, st_rdev=0)
            for scope in ("front", "rear")}))
        self.binding_matches = self.patches.enter_context(patch.object(bench.dual, "binding_matches", return_value=True))
        self.patches.enter_context(patch.object(bench.dual, "BootIdentityGuard", return_value=self.guard))
        self.patches.enter_context(patch.object(bench.dual.pipeline, "ownership_locks", side_effect=lambda: lock("common")))
        self.patches.enter_context(patch.object(bench.dual, "port_lock", side_effect=lambda p: lock(p)))
        self.imu_lock = self.patches.enter_context(patch.object(bench.live, "imu_ownership_lock", side_effect=lambda: lock("imu")))
        self.imu_constructor = self.patches.enter_context(patch.object(bench.imu, "ICM20948"))
        self.load_policy = self.patches.enter_context(patch.object(bench.shadow, "load_policy", return_value=(object(), {})))
        self.warmup = self.patches.enter_context(patch.object(bench.replay, "warmup_policy"))
        self.observer = self.patches.enter_context(patch.object(bench.observer, "StatefulPolicyObserver"))
        self.normal_collect = self.patches.enter_context(patch.object(bench, "collect"))
        self.native_baseline_policy, self.cached_policy = object(), object()
        self.baseline_source = {'manifest_sha256': 'b'*64, 'model_sha256': 'c'*64,
                                'library_sha256': 'd'*64, 'hardware_opened': False,
                                'output_allowed': False, 'live_50hz_verified': False}
        self.cached_source = {'schema': 'native-view-cache-file-only-loader-v1',
                              'manifest_sha256': 'a'*64, 'model_sha256': 'e'*64,
                              'baseline_provenance': self.baseline_source, 'diagnostic_only': True,
                              'hardware_opened': False, 'output_allowed': False,
                              'approved_for_runtime': False, 'live_50hz_verified': False}
        self.native_baseline_loader = Mock(return_value=(self.native_baseline_policy, self.baseline_source))
        self.cached_loader = Mock(return_value=(self.cached_policy, self.cached_source))
        package = types.ModuleType('native_policy_overnight'); package.__path__ = []
        package.load_verified = self.native_baseline_loader
        views = types.ModuleType('native_policy_overnight.view_cache'); views.__path__ = []
        loader = types.ModuleType('native_policy_overnight.view_cache.loader')
        loader.load_file_only_verified = self.cached_loader
        self.gc_api = types.ModuleType('gc')
        self.gc_api.collect = Mock(return_value=37)
        self.gc_settings_functions = ('isenabled', 'get_threshold', 'get_count', 'get_stats',
                                      'enable', 'disable', 'set_threshold', 'freeze', 'unfreeze',
                                      'get_freeze_count', 'set_debug', 'get_debug')
        for name in self.gc_settings_functions:
            setattr(self.gc_api, name, Mock())
        self.patches.enter_context(patch.dict('sys.modules', {
            package.__name__: package, views.__name__: views, loader.__name__: loader,
            'gc': self.gc_api}))
        self.patches.enter_context(patch.object(bench.sys, 'path', list(bench.sys.path)))

    def close_temp_fds(self):
        for port in self.ports:
            if port.fd is not None:
                port.close()
        for fd in self.boot_fds:
            try:
                os.close(fd)
            except OSError:
                pass

    def compare_args(self):
        return ["--execute", "--compare-feedback", "--mode", "stop-proxy", "--acquisition-only",
                "--supported-disabled", "--cycles", "1", "--front-port", "mock-front",
                "--rear-port", "mock-rear", "--expected-uids", str(self.uid_file),
                "--library", str(self.root / "unused-library"), "--output", str(self.output)]

    def acquisition_args(self):
        self.expected_stop_proxy = False
        return ["--execute", "--mode", "type17", "--acquisition-only", "--cycles", "1",
                "--front-port", "mock-front", "--rear-port", "mock-rear",
                "--expected-uids", str(self.uid_file), "--library", str(self.root / "unused-library"),
                "--output", str(self.output)]

    def policy_args(self):
        calibration = self.root / "calibration.json"
        calibration.write_text(json.dumps(dict(identities=UIDS, source_current_boot_id=BOOT)))
        mount = self.root / "mount.json"
        mount.write_text("{}")
        return ["--execute", "--mode", "stop-proxy", "--supported-disabled", "--cycles", "1",
                "--front-port", "mock-front", "--rear-port", "mock-rear",
                "--expected-uids", str(self.uid_file), "--library", str(self.root / "unused"),
                "--output", str(self.output), "--calibration", str(calibration),
                "--mount", str(mount), "--bundle", str(self.root / "unused-bundle")]

    def native_baseline_flags(self):
        return ['--native-policy-manifest', str(self.root / 'native-baseline.json'),
                '--native-policy-manifest-sha256', 'b'*64]

    def cached_variant_flags(self):
        return ['--view-cache-manifest', str(self.root / 'cached-variant.json'),
                '--view-cache-manifest-sha256', 'a'*64]

    def mock_timer_collection(self, *, status="COMPLETE_DIAGNOSTIC", measurements=None):
        def collect(sessions, device, policy, *, mode, cycles, check, worker_initializer):
            self.imu_constructor.return_value.start.assert_called_once_with()
            for first in (1, 7):
                self.assertEqual(len(self.sessions[first].calls), 6)
            # Only pure prestart tasks; no mocked sensor is read here.
            with ThreadPoolExecutor(max_workers=3, initializer=worker_initializer) as pool:
                bench._prestart_workers(pool, check)
            return {"status": status, "errors": [] if status == "COMPLETE_DIAGNOSTIC" else ["injected collect failure"],
                    "measurements": measurements or []}, []
        self.normal_collect.side_effect = collect

    def ready_imu(self):
        device = self.imu_constructor.return_value
        device.restore_status = "restored"
        device.start.return_value = {"source": "mock-imu"}
        return device

    def call_main(self, args):
        with contextlib.redirect_stdout(io.StringIO()):
            return bench.main(args)

    def saved(self):
        return json.loads((self.output / "report.json").read_text()), json.loads((self.output / "records.json").read_text())

    def assert_no_model(self):
        for method in (self.load_policy, self.native_baseline_loader, self.cached_loader,
                       self.warmup, self.observer):
            method.assert_not_called()
        self.torch.set_num_threads.assert_not_called()
        self.torch.set_num_interop_threads.assert_not_called()

    def assert_no_model_or_imu(self):
        self.assert_no_model()
        for method in (self.imu_constructor, self.imu_lock, self.normal_collect):
            method.assert_not_called()

    def assert_gc_settings_untouched(self):
        for name in self.gc_settings_functions:
            getattr(self.gc_api, name).assert_not_called()

    def assert_no_gc_activity(self):
        self.gc_api.collect.assert_not_called()
        self.assert_gc_settings_untouched()

    def assert_closed(self):
        self.assertTrue(all(port.closed for port in self.ports))
        self.guard.close.assert_called_once()
        for fd in self.boot_fds:
            with self.assertRaises(OSError):
                os.fstat(fd)
        for name in {name for name, _ in self.lock_events}:
            self.assertEqual([event for n, event in self.lock_events if n == name], ["enter", "exit"])

    def test_compare_cli_saves_replayable_raw_json_without_model_or_imu(self):
        self.assertEqual(self.call_main(self.compare_args()), 0)
        report, evidence = self.saved()
        self.assertEqual(report["status"], "COMPLETE_DIAGNOSTIC")
        self.assertEqual(report["imu_restore_status"], "not_started")
        self.assertEqual(evidence["kind"], "native_feedback_comparison")
        self.assertEqual(len(evidence["cycles"]), 1)
        rebuilt = compare.analyze_feedback_comparison(evidence, UIDS)
        self.assertEqual(report["per_motor"], rebuilt["per_motor"])
        self.assertFalse(report["dynamic_scale_validated"])
        self.assertEqual([len(self.sessions[i].calls) for i in (1, 7)], [4, 4])
        self.assert_no_model_or_imu()
        self.assert_closed()

    def test_compare_failure_retains_both_bus_evidence_and_never_continues(self):
        self.fail_at = 3  # identity, before, then front STOP feedback fails.
        self.assertEqual(self.call_main(self.compare_args()), 2)
        report, evidence = self.saved()
        self.assertEqual(report["status"], "ABORTED")
        self.assertIn("injected failure", report["errors"][0])
        self.assertEqual(evidence["cycles"][0]["after"], {})
        self.assertIn("failed_native_exchange", evidence["cycles"][0]["feedback"]["front"])
        self.assertIn("records", evidence["cycles"][0]["feedback"]["rear"])
        self.assertEqual([len(self.sessions[i].calls) for i in (1, 7)], [3, 3])
        self.assert_no_model_or_imu()
        self.assert_closed()

    def test_uid_mismatch_never_enters_telemetry_or_stop_phase(self):
        self.corrupt_uid = True
        self.assertEqual(self.call_main(self.compare_args()), 2)
        report, evidence = self.saved()
        self.assertEqual(evidence["cycles"], [])
        self.assertIn("UID mismatch", report["errors"][0])
        self.assertEqual([len(self.sessions[i].calls) for i in (1, 7)], [1, 1])
        self.assert_no_model_or_imu()
        self.assert_closed()

    def test_acquisition_startup_serializes_identities_before_imu_and_collect(self):
        device = self.imu_constructor.return_value
        device.restore_status = "restored"
        device.start.return_value = {"source": "mock-imu"}
        checked_calls = []

        def startup_check():
            checked_calls.append(tuple(len(self.sessions[i].calls) for i in (1, 7)))
            self.imu_constructor.assert_not_called()
            self.normal_collect.assert_not_called()

        def complete_collect(sessions, imu_device, policy_observer, *, mode, cycles, check):
            self.assertEqual(set(sessions), {"front", "rear"})
            for scope, first_id in (("front", 1), ("rear", 7)):
                self.assertIs(sessions[scope], self.sessions[first_id])
                self.assertEqual(sessions[scope].calls,
                                 [[bench.codec.read_request(i)] for i in range(first_id, first_id+6)])
            self.assertEqual(checked_calls, [(i, 0) for i in range(6)]+[(6, i) for i in range(6)])
            self.assertIs(imu_device, device)
            self.assertIsNone(policy_observer)
            self.assertEqual((mode, cycles), ("type17", 1))
            self.assertTrue(callable(check))
            self.imu_constructor.assert_called_once_with()
            device.start.assert_called_once_with()
            device.close.assert_not_called()
            return {"status": "COMPLETE_DIAGNOSTIC", "errors": []}, []

        self.guard.check.side_effect = startup_check
        self.normal_collect.side_effect = complete_collect
        self.assertEqual(self.call_main(self.acquisition_args()), 0)
        report, evidence = self.saved()
        self.assertEqual(report["status"], "COMPLETE_DIAGNOSTIC")
        self.assertEqual(report["imu_configuration"], {"source": "mock-imu"})
        self.assertEqual(report["imu_restore_status"], "restored")
        self.assertEqual(evidence, [])
        for scope, first_id in (("front", 1), ("rear", 7)):
            captures = report["identities"][scope]
            self.assertEqual(len(captures), 6)
            for mid, capture in zip(range(first_id, first_id+6), captures):
                self.assertEqual(len(capture["records"]), 1)
                record = capture["records"][0]
                self.assertEqual(record["tx_hex"], bench.codec.read_request(mid).hex())
                self.assertEqual((record["written"], record["received"]), (17, 17))
                reply = bench.codec.ATParser().feed(bytes.fromhex(record["rx_hex"]))[0]
                self.assertEqual(bench.codec.decode_reply(reply, mid, None)["mcu_uid_hex"], UIDS[str(mid)])
                self.assertEqual(capture["stats"]["writes"], 1)
        self.assertEqual(self.native_session.call_count, 2)
        for call in self.native_session.call_args_list:
            self.assertEqual(call.kwargs["gap_ns"], 600_000)
            self.assertEqual(call.kwargs["window"], 3)
        self.assertEqual(report["plan"]["request_gap_us"], 600)
        self.assertEqual(report["plan"]["gap_ms"], .6)
        self.assertEqual(report["plan"]["window"], 3)
        self.assertEqual(report["plan"]["startup_identity_window"], 1)
        self.normal_collect.assert_called_once()
        device.close.assert_called_once_with()
        self.assert_no_model()
        self.assert_closed()

    def test_acquisition_missing_identity_retains_prior_id_and_failed_exchange_without_retry(self):
        self.fail_at = 2
        self.assertEqual(self.call_main(self.acquisition_args()), 2)
        report, evidence = self.saved()
        self.assertEqual(report["status"], "ABORTED")
        self.assertIn("injected failure", report["errors"][0])
        self.assertEqual(evidence, [])
        self.assertEqual(self.sessions[1].calls,
                         [[bench.codec.read_request(1)], [bench.codec.read_request(2)]])
        self.assertEqual(self.sessions[7].calls, [])
        self.assertEqual(set(report["identities"]), {"front"})
        self.assertEqual(len(report["identities"]["front"]), 1)
        completed = report["identities"]["front"][0]["records"]
        self.assertEqual(len(completed), 1)
        self.assertEqual(completed[0]["tx_hex"], bench.codec.read_request(1).hex())
        self.assertEqual(completed[0]["received"], 17)
        failure = report["startup_failure"]
        self.assertEqual(len(failure["records"]), 1)
        self.assertEqual(failure["records"][0]["tx_hex"], bench.codec.read_request(2).hex())
        self.assertEqual(failure["records"][0]["received"], 0)
        self.assertEqual(failure["records"][0]["written"], 17)
        self.assertEqual(len(failure["records"][0]["rx_hex"]), 34)
        self.assertEqual(failure["stats"]["writes"], 1)
        self.assertEqual(report["imu_restore_status"], "not_started")
        self.imu_constructor.assert_not_called()
        self.imu_constructor.return_value.start.assert_not_called()
        self.normal_collect.assert_not_called()
        self.assert_no_model()
        self.assert_closed()

    def test_custom_request_settings_reach_both_acquisition_sessions_and_saved_plan(self):
        device = self.imu_constructor.return_value
        device.restore_status = "restored"
        device.start.return_value = {"source": "mock-imu"}
        self.normal_collect.return_value = ({"status": "COMPLETE_DIAGNOSTIC", "errors": []}, [])
        args = self.acquisition_args()+["--request-window", "1", "--request-gap-us", "5000"]
        self.assertEqual(self.call_main(args), 0)
        report, _ = self.saved()
        self.assertEqual(report["plan"]["window"], 1)
        self.assertEqual(report["plan"]["request_gap_us"], 5000)
        self.assertEqual(report["plan"]["gap_ms"], 5.)
        self.assertEqual(report["plan"]["startup_identity_window"], 1)
        self.assertFalse(report["plan"]["startup_identity_retry"])
        for call in self.native_session.call_args_list:
            self.assertEqual(call.kwargs["gap_ns"], 5_000_000)
            self.assertEqual(call.kwargs["window"], 1)
        sessions = self.normal_collect.call_args.args[0]
        self.assertEqual(set(sessions), {"front", "rear"})
        self.assertTrue(all(s.window == 1 and s.gap_ns == 5_000_000 for s in sessions.values()))
        self.normal_collect.assert_called_once()
        self.assert_no_model()
        self.assert_closed()

    def test_trace_storage_is_opt_in_and_forwarded_to_diagnostic_collector(self):
        self.ready_imu()
        self.normal_collect.return_value = ({"status": "COMPLETE_DIAGNOSTIC", "errors": []}, [])
        self.assertEqual(self.call_main(self.acquisition_args()+['--record-storage','trace']),0)
        report,records=self.saved()
        self.assertEqual(report['plan']['record_storage'],'trace')
        self.assertFalse(report['plan']['disk_io_during_cycles'])
        self.assertEqual(records,[])
        self.assertEqual(self.normal_collect.call_args.kwargs['record_storage'],'trace')
        self.normal_collect.assert_called_once()

    def test_output_dispatch_trace_is_opt_in_and_requires_policy_stop_proxy(self):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as rejected:
            self.call_main(self.acquisition_args()+['--output-dispatch-trace'])
        self.assertEqual(rejected.exception.code,2)
        self.assertFalse(self.output.exists())
        self.normal_collect.assert_not_called()

        self.expected_stop_proxy=True
        self.ready_imu()
        self.normal_collect.return_value=({'status':'COMPLETE_DIAGNOSTIC','errors':[]},[])
        self.assertEqual(self.call_main(self.policy_args()+['--output-dispatch-trace']),0)
        report,_=self.saved()
        self.assertTrue(report['plan']['output_dispatch_trace'])
        self.assertIs(self.normal_collect.call_args.kwargs['output_dispatch_trace'],True)
        self.assertFalse(report['plan']['disk_io_during_cycles'])

    def test_inference_thread_cpu_trace_requires_26_request_mode_and_reaches_collector(self):
        for args in (self.acquisition_args()+['--inference-thread-cpu-trace'],
                     self.policy_args()+['--inference-thread-cpu-trace'],
                     self.policy_args()+['--v3-voltage-proxy','--inference-thread-cpu-trace',
                                         '--cycles','501']):
            with self.subTest(args=args),contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as rejected:
                self.call_main(args)
            self.assertEqual(rejected.exception.code,2)
        self.assertFalse(self.output.exists())
        self.normal_collect.assert_not_called()

        self.expected_stop_proxy=True
        self.ready_imu()
        self.normal_collect.return_value=({'status':'COMPLETE_DIAGNOSTIC','errors':[]},[])
        self.assertEqual(self.call_main(self.policy_args()+[
            '--v3-voltage-proxy','--inference-thread-cpu-trace']),0)
        report,_=self.saved()
        self.assertTrue(report['plan']['inference_thread_cpu_trace'])
        self.assertTrue(report['plan']['v3_voltage_proxy'])
        self.assertEqual(report['plan']['requests_per_cycle'],26)
        self.assertIs(self.normal_collect.call_args.kwargs['inference_thread_cpu_trace'],True)
        self.assertIs(self.normal_collect.call_args.kwargs['v3_voltage_proxy'],True)
        self.assertFalse(report['plan']['disk_io_during_cycles'])

    def test_gc_deferral_requires_bounded_traced_stop_and_is_forwarded_only_when_selected(self):
        invalid=(
            self.acquisition_args()+['--defer-gc-during-cycles','--record-storage','trace',
                                     '--output-dispatch-trace'],
            self.policy_args()+['--defer-gc-during-cycles'],
            self.policy_args()+['--defer-gc-during-cycles','--record-storage','trace',
                                '--output-dispatch-trace','--cycles','501'])
        for args in invalid:
            with self.subTest(args=args),contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as rejected:
                self.call_main(args)
            self.assertEqual(rejected.exception.code,2)
        self.assertFalse(self.output.exists())
        self.normal_collect.assert_not_called()
        self.expected_stop_proxy=True
        self.ready_imu()
        self.normal_collect.return_value=({'status':'COMPLETE_DIAGNOSTIC','errors':[]},[])
        args=self.policy_args()+['--defer-gc-during-cycles','--record-storage','trace',
                                 '--output-dispatch-trace']
        self.assertEqual(self.call_main(args),0)
        report,_=self.saved()
        self.assertTrue(report['plan']['defer_gc_during_cycles'])
        self.assertTrue(report['plan']['output_dispatch_trace'])
        self.assertEqual(report['plan']['record_storage'],'trace')
        self.assertIs(self.normal_collect.call_args.kwargs['defer_gc_during_cycles'],True)

    def test_trace_storage_rejects_feedback_comparison_before_setup(self):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as raised:
            self.call_main(self.compare_args()+['--record-storage','trace'])
        self.assertEqual(raised.exception.code,2)
        self.assertFalse(self.output.exists())
        bench.native.load_library.assert_not_called()
        self.serial_constructor.assert_not_called()
        self.normal_collect.assert_not_called()
        self.assert_no_model()

    def test_custom_request_settings_reach_feedback_comparison_sessions_and_saved_plan(self):
        args = self.compare_args()+["--request-window", "2", "--request-gap-us", "900"]
        self.assertEqual(self.call_main(args), 0)
        report, evidence = self.saved()
        self.assertEqual(report["plan"]["window"], 2)
        self.assertEqual(report["plan"]["request_gap_us"], 900)
        self.assertEqual(report["plan"]["gap_ms"], .9)
        self.assertEqual(report["plan"]["startup_identity_window"], 1)
        self.assertEqual(evidence["kind"], "native_feedback_comparison")
        self.assertEqual(self.native_session.call_count, 2)
        for call in self.native_session.call_args_list:
            self.assertEqual(call.kwargs["gap_ns"], 900_000)
            self.assertEqual(call.kwargs["window"], 2)
        self.assertTrue(all(s.window == 2 and s.gap_ns == 900_000 for s in self.sessions.values()))
        self.assert_no_model_or_imu()
        self.assert_closed()

    def test_timer_slack_plan_is_explicit_without_platform_checks_or_libc(self):
        with patch.object(bench.thread_timer_slack, '_load_prctl') as loader, \
                patch.object(bench.thread_timer_slack, 'require_supported_platform') as supported:
            for value in (None, "1000", "50000"):
                with self.subTest(value=value):
                    output = io.StringIO()
                    args = [] if value is None else ["--timer-slack-ns", value]
                    with contextlib.redirect_stdout(output):
                        self.assertEqual(bench.main(args), 0)
                    plan = json.loads(output.getvalue())
                    self.assertEqual(plan['timer_slack_ns'], int(value) if value is not None else None)
                    self.assertEqual((plan['request_gap_us'], plan['window']), (600, 3))
        loader.assert_not_called(); supported.assert_not_called()
        self.assertFalse(self.output.exists())
        self.assert_no_model_or_imu()
        self.serial_constructor.assert_not_called()

    def test_default_collection_never_loads_libc_or_supplies_initializer(self):
        self.ready_imu()
        self.normal_collect.return_value = ({"status": "COMPLETE_DIAGNOSTIC", "errors": []}, [])
        with patch.object(bench.thread_timer_slack, '_load_prctl') as loader, \
                patch.object(bench.thread_timer_slack, 'require_supported_platform') as supported:
            self.assertEqual(self.call_main(self.acquisition_args()), 0)
        loader.assert_not_called(); supported.assert_not_called()
        self.assertNotIn('worker_initializer', self.normal_collect.call_args.kwargs)
        report, _ = self.saved()
        self.assertIsNone(report['plan']['timer_slack_ns'])
        self.assertFalse(report['timer_slack']['enabled'])
        self.assertEqual(report['timer_slack']['workers'], [])
        self.assertTrue(all(value is None for value in report['timer_slack']['parent'].values()))
        self.assert_closed()

    def test_timer_slack_applies_only_during_collection_and_records_inherited_workers(self):
        self.ready_imu(); self.mock_timer_collection()
        backend = FakePrctl()
        def set_only_after_startup(value):
            self.imu_constructor.return_value.start.assert_called_once_with()
            self.assertTrue(all(len(self.sessions[first].calls) == 6 for first in (1, 7)))
            original_set(value)
        original_set = backend.set
        backend.set = Mock(side_effect=set_only_after_startup)
        args = self.acquisition_args()+['--timer-slack-ns', '1000', '--request-gap-us', '800']
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl', return_value=backend):
            self.assertEqual(self.call_main(args), 0)
        report, _ = self.saved()
        self.assertEqual(report['plan']['timer_slack_ns'], 1000)
        setting = report['timer_slack']
        self.assertEqual(setting['scope'], 'diagnostic_collect_only')
        self.assertEqual(setting['parent'], {'native_tid': backend.parent_tid, 'original_ns': 73_000,
                                            'during_ns': 1000, 'after_ns': 73_000, 'restored': True})
        self.assertTrue(setting['worker_verification_complete'])
        self.assertEqual(len({row['native_tid'] for row in setting['workers']}), 3)
        self.assertTrue(all(row['current_ns'] == 1000 and row['verified'] for row in setting['workers']))
        self.assertEqual([call.args[0] for call in backend.set.call_args_list], [1000, 73_000])
        self.assertEqual((report['plan']['request_gap_us'], report['plan']['window']), (800, 3))
        for call in self.native_session.call_args_list:
            self.assertEqual((call.kwargs['gap_ns'], call.kwargs['window']), (800_000, 3))
        self.assert_closed()

    def test_collect_abort_still_records_workers_and_restores_parent_timer_slack(self):
        self.ready_imu(); self.mock_timer_collection(status='ABORTED')
        backend = FakePrctl()
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl', return_value=backend):
            self.assertEqual(self.call_main(self.acquisition_args()+['--timer-slack-ns', '1000']), 2)
        report, _ = self.saved()
        self.assertEqual(report['errors'], ['injected collect failure'])
        self.assertTrue(report['timer_slack']['worker_verification_complete'])
        self.assertTrue(report['timer_slack']['parent']['restored'])
        self.assertEqual(backend.current, 73_000)
        self.assert_closed()

    def test_unconfirmed_timer_restore_aborts_and_retains_collected_measurements(self):
        self.ready_imu(); self.mock_timer_collection(measurements=[{'saved_before_restore': True}])
        backend = FakePrctl(parent_readbacks=(73_000, 1000, 50_000))
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl', return_value=backend):
            self.assertEqual(self.call_main(self.acquisition_args()+['--timer-slack-ns', '1000']), 2)
        report, _ = self.saved()
        self.assertEqual(report['measurements'], [{'saved_before_restore': True}])
        self.assertIn('restoration unconfirmed', report['errors'][-1])
        self.assertFalse(report['timer_slack']['parent']['restored'])
        self.assertEqual(report['timer_slack']['parent']['after_ns'], 50_000)
        self.assertTrue(report['timer_slack']['worker_verification_complete'])
        self.assert_closed()

    def test_timer_entry_failure_restores_before_any_collection(self):
        self.ready_imu()
        backend = FakePrctl(parent_readbacks=(73_000, 50_000, 73_000))
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl', return_value=backend):
            self.assertEqual(self.call_main(self.acquisition_args()+['--timer-slack-ns', '1000']), 2)
        report, _ = self.saved()
        self.assertIn('parent timer slack readback mismatch', report['errors'][-1])
        self.assertTrue(report['timer_slack']['parent']['restored'])
        self.assertEqual(report['timer_slack']['workers'], [])
        self.normal_collect.assert_not_called()
        self.assert_closed()

    def test_invalid_timer_slack_fails_before_output_or_device_setup(self):
        with patch.object(bench.thread_timer_slack, '_load_prctl') as loader, \
                patch.object(bench.thread_timer_slack, 'require_supported_platform') as supported:
            for value in ('0', '999', '1001', '49999', '50001', '-1', '1000.0', 'true'):
                with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as error:
                        self.call_main(self.acquisition_args()+['--timer-slack-ns', value])
                    self.assertEqual(error.exception.code, 2)
        loader.assert_not_called(); supported.assert_not_called()
        self.assertFalse(self.output.exists())
        bench.native.load_library.assert_not_called()
        self.assert_no_model_or_imu()
        self.serial_constructor.assert_not_called()

    def test_non_linux_timer_execution_fails_before_output_or_devices(self):
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'darwin'), \
                patch.object(bench.thread_timer_slack, '_load_prctl') as loader, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                self.call_main(self.acquisition_args()+['--timer-slack-ns', '1000'])
        self.assertEqual(error.exception.code, 2); loader.assert_not_called()
        self.assertFalse(self.output.exists())
        bench.native.load_library.assert_not_called()
        self.assert_no_model_or_imu()
        self.serial_constructor.assert_not_called()

    def test_feedback_comparison_rejects_timer_slack_before_setup(self):
        with patch.object(bench.thread_timer_slack, '_load_prctl') as loader, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                self.call_main(self.compare_args()+['--timer-slack-ns', '1000'])
        self.assertEqual(error.exception.code, 2); loader.assert_not_called()
        self.assertFalse(self.output.exists())
        bench.native.load_library.assert_not_called()
        self.assert_no_model_or_imu()
        self.serial_constructor.assert_not_called()

    def test_policy_warmup_and_single_prepare_follow_uid_and_imu_start(self):
        device = self.ready_imu()
        run = self.observer.return_value
        policy = self.load_policy.return_value[0]
        order = []
        def imu_started():
            self.assertTrue(all(len(self.sessions[first].calls) == 6 for first in (1, 7)))
            self.warmup.assert_not_called(); run.prepare_run.assert_not_called()
            order.append('imu_start')
            return {'source': 'mock-imu'}
        def warmed(*args):
            self.assertEqual(args, (policy, self.torch, 0, 10))
            self.assertEqual(order, ['imu_start'])
            run.prepare_run.assert_not_called(); self.normal_collect.assert_not_called()
            order.append('warmup')
        def prepared(*, warmup_completed):
            self.assertTrue(warmup_completed)
            self.assertEqual(order, ['imu_start', 'warmup'])
            self.normal_collect.assert_not_called()
            order.append('prepare')
        def collected(sessions, imu_device, observer, **kwargs):
            self.assertIs(imu_device, device); self.assertIs(observer, run)
            self.assertEqual(order, ['imu_start', 'warmup', 'prepare'])
            self.assertNotIn('worker_initializer', kwargs)
            order.append('collect')
            return {'status': 'COMPLETE_DIAGNOSTIC', 'errors': []}, []
        device.start.side_effect = imu_started
        self.warmup.side_effect = warmed
        run.prepare_run.side_effect = prepared
        self.normal_collect.side_effect = collected
        self.assertEqual(self.call_main(self.policy_args()), 0)
        report, _ = self.saved()
        self.assertEqual(order, ['imu_start', 'warmup', 'prepare', 'collect'])
        self.warmup.assert_called_once_with(policy, self.torch, 0, 10)
        run.prepare_run.assert_called_once_with(warmup_completed=True)
        self.normal_collect.assert_called_once()
        setup = report['setup_policy_warmup']
        self.assertEqual(setup['position'], 'after_identity_and_imu_start_before_worker_startup')
        self.assertEqual(setup['iterations'], 10); self.assertTrue(setup['complete'])
        self.assertLessEqual(setup['begin_ns'], setup['end_ns'])
        self.assertAlmostEqual(setup['duration_ms'], (setup['end_ns']-setup['begin_ns'])/1e6)
        self.assert_closed()

    def test_selected_pre_cycle_warmup_is_deferred_and_records_bounded_count(self):
        self.ready_imu()
        run = self.observer.return_value
        order = []
        def warmed(policy, torch_module, h, count):
            self.assertEqual((policy, torch_module, h, count),
                             (self.load_policy.return_value[0], self.torch, 0, 30))
            order.append('warmup')
        def prepared(*, warmup_completed):
            self.assertTrue(warmup_completed)
            self.assertEqual(order, ['warmup'])
            order.append('prepare')
        def collected(sessions, device, observer, *, mode, cycles, check,
                      pre_cycle_policy_prepare):
            self.assertIs(observer, run)
            self.assertEqual((mode, cycles), ('stop-proxy', 1))
            self.assertEqual(order, [])
            self.warmup.assert_not_called(); run.prepare_run.assert_not_called()
            pre_cycle_policy_prepare()
            self.assertEqual(order, ['warmup', 'prepare'])
            return {'status': 'COMPLETE_DIAGNOSTIC', 'errors': []}, []
        self.warmup.side_effect = warmed
        run.prepare_run.side_effect = prepared
        self.normal_collect.side_effect = collected
        self.assertEqual(self.call_main(self.policy_args()+[
            '--pre-cycle-policy-warmup-calls', '30']), 0)
        report, _ = self.saved()
        self.assertEqual(report['plan']['pre_cycle_policy_warmup_calls'], 30)
        setup = report['setup_policy_warmup']
        self.assertEqual(setup['position'], 'after_worker_startup_before_optional_main_thread_affinity')
        self.assertFalse(setup['after_main_thread_affinity'])
        self.assertEqual(setup['iterations'], 30)
        self.assertTrue(setup['complete'])
        self.assertAlmostEqual(setup['duration_ms'], (setup['end_ns']-setup['begin_ns'])/1e6)
        self.warmup.assert_called_once()
        run.prepare_run.assert_called_once_with(warmup_completed=True)
        self.assert_closed()

    def test_post_pin_prime_uses_observer_tensors_then_resets_once(self):
        self.ready_imu()
        run=self.observer.return_value
        run._input_buffers=tuple(array('f',[0.]*n) for n in (3,3,3,12,12,12))
        run._input_tensors=tuple(Mock(shape=(1,len(buf)),
                                  data_ptr=Mock(return_value=buf.buffer_info()[0]))
                                 for buf in run._input_buffers)
        policy=self.load_policy.return_value[0]
        order=[]
        def warmed(*args,**kwargs):
            self.assertEqual(args,(policy,self.torch,0,10 if not kwargs else 3))
            if kwargs:self.assertIs(kwargs['input_tensors'],run._input_tensors)
            order.append('prime' if kwargs else 'warmup')
        def prepared(*,warmup_completed):
            self.assertTrue(warmup_completed)
            self.assertEqual(order,['warmup','prime'])
            order.append('reset')
        def collected(sessions,device,observer,*,mode,cycles,check,main_thread_cpu,
                      pre_cycle_policy_prepare,post_pin_policy_prepare):
            self.assertEqual((mode,cycles,main_thread_cpu),('stop-proxy',1,4))
            self.assertEqual(order,[])
            pre_cycle_policy_prepare()
            self.assertEqual(order,['warmup'])
            run.prepare_run.assert_not_called()
            post_pin_policy_prepare()
            self.assertEqual(order,['warmup','prime','reset'])
            return {'status':'COMPLETE_DIAGNOSTIC','errors':[]},[]
        self.warmup.side_effect=warmed
        run.prepare_run.side_effect=prepared
        self.normal_collect.side_effect=collected
        with patch.object(bench.os,'sched_getaffinity',return_value={0,4},create=True), \
                patch.object(bench.os,'sched_setaffinity',create=True):
            self.assertEqual(self.call_main(self.policy_args()+[
                '--pre-cycle-policy-warmup-calls','10','--main-thread-cpu','4',
                '--post-pin-policy-prime-calls','3']),0)
        report,_=self.saved()
        self.assertEqual(report['plan']['post_pin_policy_prime_calls'],3)
        self.assertEqual(report['setup_policy_warmup']['iterations'],10)
        self.assertTrue(report['setup_policy_warmup']['complete'])
        prime=report['setup_policy_prime']
        self.assertEqual(prime['kind'],'synthetic_model_calls_on_reused_input_tensors')
        self.assertEqual(prime['iterations'],3)
        self.assertTrue(prime['complete'])
        self.assertTrue(prime['observer_reset_after'])
        self.assertEqual((prime['sensor_cycles'],prime['stop_writes']),(0,0))
        self.assertEqual(self.warmup.call_count,2)
        run.prepare_run.assert_called_once_with(warmup_completed=True)
        self.assert_closed()

    def test_invalid_pre_cycle_warmup_rejects_before_hardware_setup(self):
        cases = (
            (self.policy_args()+['--pre-cycle-policy-warmup-calls', '9'], 'invalid choice'),
            (self.policy_args()+['--pre-cycle-policy-warmup-calls', '101'], 'invalid choice'),
            (self.acquisition_args()+['--pre-cycle-policy-warmup-calls', '30'], 'requires at most 500'),
            (self.compare_args()+['--pre-cycle-policy-warmup-calls', '30'], 'requires at most 500'),
            (self.policy_args()+['--cycles', '501', '--pre-cycle-policy-warmup-calls', '30'],
             'requires at most 500'),
            (self.policy_args()+['--post-pin-policy-prime-calls','0'], 'invalid choice'),
            (self.policy_args()+['--post-pin-policy-prime-calls','3'], 'requires pre-cycle'),
            (self.policy_args()+['--pre-cycle-policy-warmup-calls','10',
                                 '--post-pin-policy-prime-calls','3'], 'requires pre-cycle'),
        )
        for args, message in cases:
            with self.subTest(args=args):
                stderr = io.StringIO()
                with contextlib.redirect_stderr(stderr):
                    with self.assertRaises(SystemExit) as error:
                        self.call_main(args)
                self.assertEqual(error.exception.code, 2)
                self.assertIn(message, stderr.getvalue())
                self.assertFalse(self.output.exists())
                bench.native.load_library.assert_not_called()
                self.assert_no_model_or_imu()

    def test_failed_identity_preflight_never_warms_prepares_or_enters_timer_scope(self):
        self.fail_at = 2
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl') as loader:
            self.assertEqual(self.call_main(self.policy_args()+[
                '--timer-slack-ns', '1000', '--setup-gc', 'before-warmup']), 2)
        loader.assert_not_called()
        self.warmup.assert_not_called(); self.observer.return_value.prepare_run.assert_not_called()
        self.imu_constructor.assert_not_called(); self.normal_collect.assert_not_called()
        report, _ = self.saved()
        self.assertNotIn('setup_policy_warmup', report)
        self.assertEqual(report['setup_gc']['mode'], 'before-warmup')
        self.assertFalse(report['setup_gc']['attempted']); self.assert_no_gc_activity()
        self.assertTrue(all(value is None for value in report['timer_slack']['parent'].values()))
        self.assertEqual(report['timer_slack']['workers'], [])
        self.assert_closed()

    def test_failed_setup_warmup_is_retained_without_prepare_collection_or_timer_change(self):
        self.ready_imu()
        self.warmup.side_effect = RuntimeError('warmup failure')
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl') as loader:
            self.assertEqual(self.call_main(self.policy_args()+['--timer-slack-ns', '1000']), 2)
        loader.assert_not_called()
        self.warmup.assert_called_once()
        self.observer.return_value.prepare_run.assert_not_called(); self.normal_collect.assert_not_called()
        report, _ = self.saved()
        self.assertIn('warmup failure', report['errors'][0])
        self.assertFalse(report['setup_policy_warmup']['complete'])
        self.assertIsNotNone(report['setup_policy_warmup']['duration_ms'])
        self.assert_closed()

    def test_cached_variant_plan_is_explicit_and_loads_nothing(self):
        for selected in (False, True):
            with self.subTest(selected=selected):
                args = self.policy_args()
                args.remove('--execute')
                if selected:
                    args += self.native_baseline_flags()+self.cached_variant_flags()
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.assertEqual(bench.main(args), 0)
                plan = json.loads(output.getvalue())
                self.assertIs(plan['view_cache_variant'], selected)
                self.assertIs(plan['view_cache_diagnostic_only'], selected)
                self.assertEqual(plan['view_cache_manifest'], str(self.root/'cached-variant.json') if selected else None)
                self.assertEqual(plan['view_cache_manifest_sha256'], 'a'*64 if selected else None)
                self.assertFalse(plan['enable_available']); self.assertFalse(plan['learned_targets_sent'])
                self.assertFalse(plan['full_controller_50Hz_verified'])
        self.assertFalse(self.output.exists())
        self.assert_no_model_or_imu()
        bench.native.load_library.assert_not_called(); self.serial_constructor.assert_not_called()

    def test_setup_gc_plan_is_explicit_and_reads_or_changes_no_gc_state(self):
        for selected in (None, 'before-warmup'):
            with self.subTest(selected=selected):
                args = self.policy_args()
                args.remove('--execute')
                if selected is not None: args += ['--setup-gc', selected]
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.assertEqual(bench.main(args), 0)
                self.assertEqual(json.loads(output.getvalue())['setup_gc'], selected)
        self.assert_no_gc_activity(); self.assert_no_model_or_imu()
        self.serial_constructor.assert_not_called(); bench.native.load_library.assert_not_called()
        self.assertFalse(self.output.exists())

    def test_default_full_policy_does_not_read_collect_or_change_gc(self):
        self.ready_imu()
        self.normal_collect.return_value = ({'status': 'COMPLETE_DIAGNOSTIC', 'errors': []}, [])
        self.assertEqual(self.call_main(self.policy_args()), 0)
        self.assert_no_gc_activity()
        self.warmup.assert_called_once_with(self.load_policy.return_value[0], self.torch, 0, 10)
        self.observer.return_value.prepare_run.assert_called_once_with(warmup_completed=True)
        self.normal_collect.assert_called_once()
        report, _ = self.saved()
        self.assertIsNone(report['plan']['setup_gc'])
        setup = report['setup_gc']
        self.assertIsNone(setup['mode']); self.assertFalse(setup['attempted']); self.assertFalse(setup['complete'])
        for field in ('position', 'generation', 'begin_ns', 'end_ns', 'duration_ms', 'collected_objects'):
            self.assertIsNone(setup[field])
        self.assertFalse(setup['changes_gc_settings'])
        self.assert_closed()

    def test_invalid_setup_gc_mode_or_value_rejects_before_any_setup(self):
        cases = (
            (self.acquisition_args()+['--setup-gc', 'before-warmup'], 'requires policy inference'),
            (self.compare_args()+['--setup-gc', 'before-warmup'], 'requires policy inference'),
            (self.policy_args()+['--setup-gc', 'before-cycles'], 'invalid choice'),
            (self.policy_args()+['--setup-gc', 'none'], 'invalid choice'),
        )
        for execute in (False, True):
            for base_args, message in cases:
                with self.subTest(execute=execute, args=base_args):
                    args = list(base_args)
                    if not execute: args.remove('--execute')
                    stderr = io.StringIO()
                    with contextlib.redirect_stderr(stderr), patch.object(bench.Path, 'mkdir') as mkdir:
                        with self.assertRaises(SystemExit) as error:
                            self.call_main(args)
                    self.assertEqual(error.exception.code, 2)
                    self.assertIn(message, stderr.getvalue()); mkdir.assert_not_called()
                    self.assertFalse(self.output.exists())
                    bench.native.load_library.assert_not_called(); bench.dual.validate_ports.assert_not_called()
                    self.serial_constructor.assert_not_called(); self.native_session.assert_not_called()
                    self.assert_no_gc_activity(); self.assert_no_model_or_imu()

    def test_v3_voltage_proxy_plan_and_execution_are_disabled_only(self):
        args=self.policy_args()+['--v3-voltage-proxy','--request-gap-us','800']
        self.ready_imu()
        self.normal_collect.return_value=({'status':'COMPLETE_DIAGNOSTIC','errors':[],
                                           'v3_voltage_proxy':True},[])
        self.assertEqual(self.call_main(args),0)
        report,_=self.saved()
        self.assertTrue(report['plan']['v3_voltage_proxy'])
        self.assertEqual(report['plan']['requests_per_cycle'],26)
        self.assertEqual(report['plan']['type1_requests_per_cycle'],0)
        self.assertFalse(report['plan']['enable_available'])
        self.assertFalse(report['plan']['learned_targets_sent'])
        self.assertTrue(self.normal_collect.call_args.kwargs['v3_voltage_proxy'])
        self.assert_closed()

    def test_startup_allowance_explicitly_adds_one_recorded_cycle(self):
        self.ready_imu()
        self.normal_collect.return_value=({'status':'COMPLETE_DIAGNOSTIC','errors':[]},[])
        args=self.policy_args()+['--v3-voltage-proxy','--cycles','501',
                                 '--startup-cycle-allowance','1','--absolute-epoch-cadence']
        self.assertEqual(self.call_main(args),0)
        report,_=self.saved()
        self.assertEqual(report['plan']['steady_cycles_requested'],500)
        self.assertEqual(report['plan']['cycles'],501)
        self.assertEqual(self.normal_collect.call_args.kwargs['startup_cycle_allowance'],1)
        self.assertEqual(self.normal_collect.call_args.kwargs['cycles'],501)
        self.assertFalse(report['plan']['enable_available'])
        self.assertFalse(report['plan']['learned_targets_sent'])
        self.assert_closed()

    def test_startup_allowance_rejects_missing_steady_cycle_or_more_than_one_exception(self):
        for extra in (['--cycles','1','--startup-cycle-allowance','1'],
                      ['--cycles','502','--startup-cycle-allowance','1'],
                      ['--startup-cycle-allowance','2'],
                      ['--release-spin-us','500']):
            with self.subTest(extra=extra),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):self.call_main(self.policy_args()+extra)
                self.serial_constructor.assert_not_called()
                self.assertFalse(self.output.exists())

    def test_v3_voltage_proxy_rejects_incompatible_modes_before_hardware_open(self):
        cases=((self.acquisition_args()+['--v3-voltage-proxy'],'requires at most 500'),
               (self.compare_args()+['--v3-voltage-proxy'],'requires at most 500'),
               (self.policy_args()+['--cycles','501','--v3-voltage-proxy'],'requires at most 500'))
        for args,message in cases:
            with self.subTest(args=args),contextlib.redirect_stderr(io.StringIO()) as error:
                with self.assertRaises(SystemExit) as exit_status:self.call_main(args)
                self.assertEqual(exit_status.exception.code,2)
                self.assertIn(message,error.getvalue())
                self.assertFalse(self.output.exists())
                self.serial_constructor.assert_not_called()
                self.native_session.assert_not_called()
                self.assert_no_model_or_imu()

    def test_worker_cpu_exclusion_is_opt_in_v3_stop_diagnostic_only(self):
        self.ready_imu()
        self.normal_collect.return_value=({'status':'COMPLETE_DIAGNOSTIC','errors':[],
                                           'worker_affinity':{'enabled':True,'restored':True}},[])
        args=self.policy_args()+['--v3-voltage-proxy','--main-thread-cpu','4',
                                 '--exclude-policy-cpu-from-workers']
        with patch.object(bench.os,'sched_getaffinity',return_value={0,1,2,3,4},create=True), \
                patch.object(bench.os,'sched_setaffinity',create=True):
            self.assertEqual(self.call_main(args),0)
        report,_=self.saved()
        self.assertTrue(report['plan']['exclude_policy_cpu_from_workers'])
        self.assertTrue(self.normal_collect.call_args.kwargs['exclude_policy_cpu_from_workers'])
        self.assertTrue(report['worker_affinity']['restored'])
        self.assertFalse(report['plan']['enable_available'])
        self.assertFalse(report['plan']['learned_targets_sent'])
        self.assert_closed()

    def test_worker_cpu_exclusion_rejects_unsupported_plan_before_hardware_open(self):
        cases=(self.policy_args()+['--exclude-policy-cpu-from-workers'],
               self.policy_args()+['--v3-voltage-proxy','--exclude-policy-cpu-from-workers'],
               self.acquisition_args()+['--v3-voltage-proxy','--main-thread-cpu','4',
                                        '--exclude-policy-cpu-from-workers'],
               self.policy_args()+['--cycles','501','--v3-voltage-proxy',
                                   '--main-thread-cpu','4','--exclude-policy-cpu-from-workers'])
        for args in cases:
            with self.subTest(args=args),contextlib.redirect_stderr(io.StringIO()) as error, \
                    patch.object(bench.os,'sched_getaffinity',return_value={0,1,2,3,4},create=True), \
                    patch.object(bench.os,'sched_setaffinity',create=True):
                with self.assertRaises(SystemExit) as exit_status:self.call_main(args)
                self.assertEqual(exit_status.exception.code,2)
                self.assertIn('requires',error.getvalue())
                self.assertFalse(self.output.exists())
                self.serial_constructor.assert_not_called()
                self.native_session.assert_not_called()
                self.assert_no_model_or_imu()

    def test_absolute_epoch_cadence_is_opt_in_disabled_only(self):
        self.ready_imu()
        self.normal_collect.return_value=({'status':'COMPLETE_DIAGNOSTIC','errors':[],
                                           'motor_enable_sent':False,'learned_targets_sent':False},[])
        args=self.policy_args()+['--absolute-epoch-cadence','--v3-voltage-proxy']
        self.assertEqual(self.call_main(args),0)
        report,_=self.saved()
        self.assertTrue(report['plan']['absolute_epoch_cadence'])
        self.assertEqual(report['plan']['absolute_epoch_min_start_separation_ms'],15.)
        self.assertTrue(self.normal_collect.call_args.kwargs['absolute_epoch_cadence'])
        self.assertFalse(report['plan']['enable_available'])
        self.assertFalse(report['plan']['learned_targets_sent'])
        self.assert_closed()

    def test_absolute_epoch_rejects_incompatible_modes_before_hardware_open(self):
        cases=(self.acquisition_args()+['--absolute-epoch-cadence'],
               self.compare_args()+['--absolute-epoch-cadence'],
               self.policy_args()+['--cycles','501','--absolute-epoch-cadence'])
        for args in cases:
            with self.subTest(args=args),contextlib.redirect_stderr(io.StringIO()) as error:
                with self.assertRaises(SystemExit) as exit_status:self.call_main(args)
                self.assertEqual(exit_status.exception.code,2)
                self.assertIn('--absolute-epoch-cadence requires at most 500',error.getvalue())
                self.assertFalse(self.output.exists())
                self.serial_constructor.assert_not_called()
                self.native_session.assert_not_called()
                self.assert_no_model_or_imu()

    def test_setup_gc_failure_retains_metadata_without_warmup_reset_or_collection(self):
        self.ready_imu()
        self.gc_api.collect.side_effect = RuntimeError('Injected setup GC failure')
        args = self.policy_args()+['--setup-gc', 'before-warmup', '--timer-slack-ns', '1000',
                                   '--request-gap-us', '800']
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl') as loader:
            self.assertEqual(self.call_main(args), 2)
        loader.assert_not_called()
        self.gc_api.collect.assert_called_once_with(); self.assert_gc_settings_untouched()
        self.warmup.assert_not_called(); self.observer.return_value.prepare_run.assert_not_called()
        self.normal_collect.assert_not_called()
        self.imu_constructor.return_value.start.assert_called_once_with()
        report, records = self.saved()
        self.assertIn('Injected setup GC failure', report['errors'][0])
        self.assertEqual(records, [])
        self.assertTrue(all(len(report['identities'][scope]) == 6 for scope in ('front', 'rear')))
        self.assertNotIn('setup_policy_warmup', report)
        setup = report['setup_gc']
        self.assertTrue(setup['attempted']); self.assertFalse(setup['complete'])
        self.assertIsNone(setup['collected_objects'])
        self.assertAlmostEqual(setup['duration_ms'], (setup['end_ns']-setup['begin_ns'])/1e6)
        self.assertEqual((report['plan']['request_gap_us'], report['plan']['window']), (800, 3))
        self.assertTrue(all(value is None for value in report['timer_slack']['parent'].values()))
        self.assert_closed()

    def test_cached_variant_uid_imu_setup_gc_late_warmup_and_timer_collection_order(self):
        device = self.ready_imu(); run = self.observer.return_value
        order = []
        backend = FakePrctl()
        def loaded(*args, **kwargs):
            self.serial_constructor.assert_not_called(); self.imu_constructor.assert_not_called()
            self.warmup.assert_not_called(); self.observer.assert_not_called()
            order.append('cached_load')
            return self.cached_policy, self.cached_source
        def imu_started():
            self.assertEqual(order, ['cached_load'])
            self.assertTrue(all(len(self.sessions[first].calls) == 6 for first in (1, 7)))
            self.warmup.assert_not_called(); run.prepare_run.assert_not_called()
            order.append('imu_start')
            return {'source': 'mock-imu'}
        def garbage_collected():
            self.assertEqual(order, ['cached_load', 'imu_start'])
            self.warmup.assert_not_called(); run.prepare_run.assert_not_called()
            self.normal_collect.assert_not_called()
            self.assertEqual(backend.set_count, 0)
            self.assertTrue(all(len(self.sessions[first].calls) == 6 for first in (1, 7)))
            order.append('setup_gc')
            return 37
        def warmed(*args):
            self.assertEqual(args, (self.cached_policy, self.torch, 0, 10))
            self.assertEqual(order, ['cached_load', 'imu_start', 'setup_gc'])
            self.assertEqual(backend.set_count, 0)
            order.append('warmup')
        def prepared(*, warmup_completed):
            self.assertTrue(warmup_completed)
            self.assertEqual(order, ['cached_load', 'imu_start', 'setup_gc', 'warmup'])
            self.assertEqual(backend.set_count, 0)
            order.append('prepare')
        def collected(sessions, imu_device, observer, *, mode, cycles, check, worker_initializer):
            self.assertIs(imu_device, device); self.assertIs(observer, run)
            self.assertEqual((mode, cycles), ('stop-proxy', 1))
            self.assertEqual(order, ['cached_load', 'imu_start', 'setup_gc', 'warmup', 'prepare'])
            self.assertEqual(backend.current, 1000)
            with ThreadPoolExecutor(max_workers=3, initializer=worker_initializer) as pool:
                bench._prestart_workers(pool, check)
            order.append('collect')
            return {'status': 'COMPLETE_DIAGNOSTIC', 'errors': [], 'approved_for_runtime': False,
                    'learned_targets_sent': False, 'full_controller_50Hz_verified': False}, []
        self.cached_loader.side_effect = loaded
        device.start.side_effect = imu_started
        self.gc_api.collect.side_effect = garbage_collected
        self.warmup.side_effect = warmed; run.prepare_run.side_effect = prepared
        self.normal_collect.side_effect = collected
        args = self.policy_args()+self.native_baseline_flags()+self.cached_variant_flags()+[
            '--timer-slack-ns', '1000', '--request-gap-us', '800', '--setup-gc', 'before-warmup']
        with patch.object(bench.thread_timer_slack.sys, 'platform', 'linux'), \
                patch.object(bench.thread_timer_slack, '_load_prctl', return_value=backend):
            self.assertEqual(self.call_main(args), 0)
        self.cached_loader.assert_called_once_with(str(self.root/'cached-variant.json'),
            expected_sha256='a'*64, baseline_manifest=str(self.root/'native-baseline.json'),
            baseline_sha='b'*64, bundle=str(self.root/'unused-bundle'))
        self.native_baseline_loader.assert_not_called(); self.load_policy.assert_not_called()
        self.assertIs(self.observer.call_args.args[0], self.cached_policy)
        self.assertEqual(order, ['cached_load', 'imu_start', 'setup_gc', 'warmup', 'prepare', 'collect'])
        self.gc_api.collect.assert_called_once_with(); self.assert_gc_settings_untouched()
        self.warmup.assert_called_once_with(self.cached_policy, self.torch, 0, 10)
        run.prepare_run.assert_called_once_with(warmup_completed=True)
        report, _ = self.saved()
        self.assertEqual(report['model_source'], self.cached_source)
        self.assertEqual(report['view_cache_model_source'], self.cached_source)
        self.assertEqual(report['native_baseline_model_source'], self.baseline_source)
        self.assertTrue(report['view_cache_model_source']['diagnostic_only'])
        self.assertFalse(report['approved_for_runtime']); self.assertFalse(report['learned_targets_sent'])
        self.assertFalse(report['full_controller_50Hz_verified'])
        self.assertTrue(report['plan']['view_cache_variant'])
        self.assertEqual((report['plan']['request_gap_us'], report['plan']['window']), (800, 3))
        self.assertTrue(report['setup_policy_warmup']['complete'])
        self.assertEqual(report['plan']['setup_gc'], 'before-warmup')
        setup = report['setup_gc']
        self.assertEqual(setup['mode'], 'before-warmup')
        self.assertEqual(setup['scope'], 'setup_only')
        self.assertEqual(setup['position'], 'after_identity_and_imu_start_before_warmup')
        self.assertEqual(setup['generation'], 2)
        self.assertEqual(setup['collected_objects'], 37)
        self.assertTrue(setup['attempted']); self.assertTrue(setup['complete'])
        self.assertFalse(setup['changes_gc_settings'])
        self.assertAlmostEqual(setup['duration_ms'], (setup['end_ns']-setup['begin_ns'])/1e6)
        self.assertLessEqual(setup['end_ns'], report['setup_policy_warmup']['begin_ns'])
        self.assertTrue(report['timer_slack']['worker_verification_complete'])
        self.assertTrue(report['timer_slack']['parent']['restored'])
        self.assert_closed()

    def test_native_baseline_without_cached_flags_retains_original_loader(self):
        self.ready_imu()
        self.normal_collect.return_value = ({'status': 'COMPLETE_DIAGNOSTIC', 'errors': []}, [])
        self.assertEqual(self.call_main(self.policy_args()+self.native_baseline_flags()), 0)
        self.native_baseline_loader.assert_called_once_with(str(self.root/'native-baseline.json'),
            expected_manifest_sha256='b'*64, bundle=str(self.root/'unused-bundle'))
        self.cached_loader.assert_not_called(); self.load_policy.assert_not_called()
        self.assertIs(self.observer.call_args.args[0], self.native_baseline_policy)
        self.warmup.assert_called_once_with(self.native_baseline_policy, self.torch, 0, 10)
        self.observer.return_value.prepare_run.assert_called_once_with(warmup_completed=True)
        report, _ = self.saved()
        self.assertEqual(report['model_source'], self.baseline_source)
        self.assertEqual(report['native_baseline_model_source'], self.baseline_source)
        self.assertNotIn('view_cache_model_source', report)
        self.assertFalse(report['plan']['view_cache_variant'])
        self.assertNotIn('worker_initializer', self.normal_collect.call_args.kwargs)
        self.assert_no_gc_activity()
        self.assert_closed()

    def test_invalid_cached_flags_reject_in_plan_and_execute_before_any_setup(self):
        candidate = self.cached_variant_flags(); baseline = self.native_baseline_flags()
        invalid = (
            (candidate[:2], 'supplied together'),
            (candidate[2:], 'supplied together'),
            (['--view-cache-manifest', '', *candidate[2:]], 'supplied together'),
            ([*candidate[:2], '--view-cache-manifest-sha256', ''], 'supplied together'),
            (candidate, 'requires --native-policy-manifest'),
            (candidate+baseline[:2], 'requires --native-policy-manifest'),
            (candidate+baseline[2:], 'requires --native-policy-manifest'),
            (candidate+baseline+['--acquisition-only'], 'requires policy inference'),
            (candidate+baseline+['--compare-feedback', '--acquisition-only'], 'requires policy inference'),
        )
        for execute in (False, True):
            for flags, message in invalid:
                with self.subTest(execute=execute, flags=flags):
                    args = self.policy_args()
                    if not execute: args.remove('--execute')
                    stderr = io.StringIO()
                    with contextlib.redirect_stderr(stderr), patch.object(bench.Path, 'mkdir') as mkdir:
                        with self.assertRaises(SystemExit) as error:
                            self.call_main(args+flags)
                    self.assertEqual(error.exception.code, 2)
                    self.assertIn(message, stderr.getvalue()); mkdir.assert_not_called()
                    self.assertFalse(self.output.exists())
                    bench.native.load_library.assert_not_called()
                    bench.dual.validate_ports.assert_not_called()
                    self.serial_constructor.assert_not_called(); self.native_session.assert_not_called()
                    self.assert_no_model_or_imu()
                    self.assertEqual(self.ports, []); self.assertEqual(self.boot_fds, [])

    def test_cached_loader_failure_aborts_without_transport_warmup_or_fallback(self):
        self.cached_loader.side_effect = ValueError('Cached-view source SHA mismatch')
        self.assertEqual(self.call_main(self.policy_args()+self.native_baseline_flags()+self.cached_variant_flags()), 2)
        self.cached_loader.assert_called_once()
        self.native_baseline_loader.assert_not_called(); self.load_policy.assert_not_called()
        bench.dual.validate_ports.assert_not_called()
        self.serial_constructor.assert_not_called(); self.native_session.assert_not_called()
        self.imu_constructor.assert_not_called(); self.imu_lock.assert_not_called()
        self.warmup.assert_not_called(); self.observer.assert_not_called(); self.normal_collect.assert_not_called()
        self.guard.close.assert_not_called()
        self.assertEqual(self.ports, []); self.assertEqual(self.boot_fds, [])
        report, records = self.saved()
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('Cached-view source SHA mismatch', report['errors'][0])
        self.assertTrue(report['plan']['view_cache_variant'])
        self.assertNotIn('model_source', report); self.assertNotIn('view_cache_model_source', report)
        self.assertNotIn('setup_policy_warmup', report)
        self.assertEqual(records, [])
        self.assertEqual(report['imu_restore_status'], 'not_started')
        self.assertTrue(all(value is None for value in report['timer_slack']['parent'].values()))

    def test_invalid_request_settings_fail_before_output_or_device_setup(self):
        for option, value in (("--request-window", "0"), ("--request-window", "4"),
                              ("--request-window", "1.5"), ("--request-gap-us", "599"),
                              ("--request-gap-us", "5001"), ("--request-gap-us", "600.0")):
            with self.subTest(option=option, value=value):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                    self.call_main(self.acquisition_args()+[option, value])
                self.assertEqual(raised.exception.code, 2)
                self.assertFalse(self.output.exists())
                bench.native.load_library.assert_not_called()
                self.serial_constructor.assert_not_called()
                self.native_session.assert_not_called()
                self.imu_constructor.assert_not_called()
                self.imu_lock.assert_not_called()
                self.normal_collect.assert_not_called()
                self.assertEqual(self.ports, [])
                self.assertEqual(self.boot_fds, [])

    def test_acquisition_first_uid_mismatch_retains_evidence_and_stops_before_next_id(self):
        self.corrupt_uid = True
        self.assertEqual(self.call_main(self.acquisition_args()), 2)
        report, evidence = self.saved()
        self.assertEqual(report["status"], "ABORTED")
        self.assertIn("UID mismatch", report["errors"][0])
        self.assertEqual(evidence, [])
        self.assertEqual(self.sessions[1].calls, [[bench.codec.read_request(1)]])
        self.assertEqual(self.sessions[7].calls, [])
        self.assertEqual(set(report["identities"]), {"front"})
        self.assertEqual(len(report["identities"]["front"]), 1)
        record = report["identities"]["front"][0]["records"][0]
        self.assertEqual(record["tx_hex"], bench.codec.read_request(1).hex())
        reply = bench.codec.ATParser().feed(bytes.fromhex(record["rx_hex"]))[0]
        self.assertNotEqual(bench.codec.decode_reply(reply, 1, None)["mcu_uid_hex"], UIDS["1"])
        self.assertNotIn("startup_failure", report)
        self.assertEqual(report["imu_restore_status"], "not_started")
        self.imu_constructor.assert_not_called()
        self.imu_constructor.return_value.start.assert_not_called()
        self.normal_collect.assert_not_called()
        self.assert_no_model()
        self.assert_closed()

    def test_stale_calibration_boot_rejected_before_serial_io(self):
        calibration = self.root / "calibration.json"
        calibration.write_text(json.dumps(dict(identities=UIDS, source_current_boot_id="old-boot")))
        mount = self.root / "mount.json"
        mount.write_text("{}")
        args = ["--execute", "--mode", "stop-proxy", "--supported-disabled", "--cycles", "1",
                "--front-port", "mock-front", "--rear-port", "mock-rear", "--expected-uids", str(self.uid_file),
                "--library", str(self.root / "unused"), "--output", str(self.output),
                "--calibration", str(calibration), "--mount", str(mount), "--bundle", str(self.root / "unused-bundle")]
        self.assertEqual(self.call_main(args), 2)
        report, evidence = self.saved()
        self.assertIn("Capture-bound calibration", report["errors"][0])
        self.assertEqual(evidence, [])
        self.serial_constructor.assert_not_called()
        self.native_session.assert_not_called()
        self.imu_constructor.assert_not_called()
        self.normal_collect.assert_not_called()
        self.assertEqual(self.boot_fds, [])
        self.warmup.assert_not_called()
        self.observer.return_value.prepare_run.assert_not_called()
        self.torch.set_num_threads.assert_called_once_with(1)
        self.torch.set_num_interop_threads.assert_called_once_with(1)
        self.assert_closed()

    def test_binding_change_closes_opened_port_without_session_or_fallback(self):
        self.binding_matches.return_value = False
        self.assertEqual(self.call_main(self.compare_args()), 2)
        report, evidence = self.saved()
        self.assertIn("Port binding changed", report["errors"][0])
        self.assertEqual(evidence, [])
        self.assertEqual(len(self.ports), 1)
        self.native_session.assert_not_called()
        self.assert_no_model_or_imu()
        self.assert_closed()


if __name__ == "__main__":
    unittest.main()
