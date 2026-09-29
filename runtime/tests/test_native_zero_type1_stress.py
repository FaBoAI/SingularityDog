"""Native zero-gain diagnostic over local socket pairs; never opens hardware."""

from collections import Counter
from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import select
import shutil
import socket
import struct
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import native_zero_type1_stress as stress
from singularitydog_hw.can_readonly import ATParser


ROOT = Path(__file__).resolve().parents[2]
BOOT = "11111111-2222-3333-4444-555555555555"
EXPECTED = {mid: (bytes([mid])*8).hex() for mid in range(1, 13)}


def frame(cid, data):
    return b"AT"+((cid << 3)|4).to_bytes(4, "big")+b"\x08"+data+b"\r\n"


class RecordingSession(stress.ZeroGainSession):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.owner_calls = []

    def exchange(self, *args, **kwargs):
        self.owner_calls.append(("exchange", threading.get_ident()))
        return super().exchange(*args, **kwargs)

    def emergency_stop_repeated(self, *args, **kwargs):
        self.owner_calls.append(("stop", threading.get_ident()))
        return super().emergency_stop_repeated(*args, **kwargs)


class Rig:
    def __init__(self, library, *, condition="800us-window3", mutate=None, fragment=False):
        self.condition, self.mutate, self.fragment = condition, mutate, fragment
        self.cr, self.cw = os.pipe()
        os.set_blocking(self.cw, False)
        self.boot = tempfile.TemporaryFile()
        self.boot.write((BOOT+"\n").encode()); self.boot.flush()
        self.closed = threading.Event()
        self.hosts, self.peers, self.sessions, self.threads = {}, {}, {}, []
        self.seen, self.errors, self.enabled, self.counts = [], [], set(), Counter()
        self.cancelled_at = None
        for scope, ids in stress.BUSES.items():
            host, peer = socket.socketpair()
            host.setblocking(False); peer.settimeout(.02)
            self.hosts[scope], self.peers[scope] = host, peer
            self.sessions[scope] = RecordingSession(library, host.fileno(), first_id=ids[0],
                cancel_fd=self.cr, boot_fd=self.boot.fileno(), boot_id=BOOT, condition=condition)
            thread = threading.Thread(target=self.emulate, args=(scope,), daemon=True)
            thread.start(); self.threads.append(thread)

    def cancel(self):
        if self.cancelled_at is None:
            self.cancelled_at = time.monotonic_ns()
        try:
            os.write(self.cw, b"x")
        except BlockingIOError:
            pass

    def emulate(self, scope):
        peer, parser = self.peers[scope], ATParser()
        try:
            while not self.closed.is_set():
                try:
                    data = peer.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    return
                for request in parser.feed(data):
                    self.seen.append((scope, request, time.monotonic_ns()))
                    mid = request.destination
                    self.counts[mid, request.kind] += 1
                    if request.kind == 0:
                        response = frame((mid << 8)|0xfe, bytes([mid])*8)
                    elif request.kind in (1, 3, 4, 18):
                        if request.kind == 3:
                            self.enabled.add(mid)
                        elif request.kind == 4:
                            self.enabled.discard(mid)
                        mode = 2 if mid in self.enabled else 0
                        response = frame((2 << 24)|(mode << 22)|(mid << 8)|0xfd,
                                         struct.pack(">4H", 32767, 32767, 32767, 250))
                    elif request.kind == 17:
                        index = int.from_bytes(request.data[:2], "little")
                        value = (struct.pack("<I", 4000) if index == 0x7028 else
                                 bytes(4) if index == 0x7005 else struct.pack("<f", 40.))
                        response = frame((17 << 24)|(mid << 8)|0xfd, request.data[:4]+value)
                    else:
                        raise AssertionError("Unexpected command "+str(request.kind))
                    if self.mutate:
                        response = self.mutate(self, scope, request, response)
                    if response:
                        if self.fragment:
                            peer.sendall(response[:15]); time.sleep(.0001); peer.sendall(response[15:])
                        else:
                            peer.sendall(response)
        except (OSError, ValueError) as error:
            if not self.closed.is_set():
                self.errors.append(error)
        except BaseException as error:
            self.errors.append(error)

    def run(self, **kwargs):
        return stress.run(self.sessions, EXPECTED, cycles=5, condition=self.condition,
                          cancel_io=self.cancel, **kwargs)

    def close(self):
        self.closed.set()
        for peer in self.peers.values():
            peer.shutdown(socket.SHUT_RDWR)
        for thread in self.threads:
            thread.join(timeout=1)
        for session in self.sessions.values():
            session.close()
        for connection in [*self.hosts.values(), *self.peers.values()]:
            connection.close()
        self.boot.close(); os.close(self.cr); os.close(self.cw)


class NativeZeroStressTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Compile a private copy so parallel test jobs cannot replace the
        # production build-record or binary while another loader validates it.
        cls.temp = tempfile.TemporaryDirectory()
        directory = Path(cls.temp.name)
        source = directory/"transport.cpp"
        shutil.copyfile(ROOT/"runtime/experiments/native_active_transport/transport.cpp", source)
        output = directory/"libdog_active_transport.so"
        subprocess.run(["c++", "-std=c++17", "-O2", "-pthread", "-fPIC", "-shared",
                        str(source), "-o", str(output)], check=True)
        (directory/"build-record.json").write_text(json.dumps({"abi": 1,
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "binary_sha256": hashlib.sha256(output.read_bytes()).hexdigest()}))
        cls.library = native.load_library(output)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.rigs = []

    def make_rig(self, **kwargs):
        rig = Rig(self.library, **kwargs)
        self.rigs.append(rig)
        return rig

    def tearDown(self):
        for rig in self.rigs:
            rig.close()
            self.assertFalse(rig.errors, rig.errors)

    def test_default_plan_has_no_library_or_hardware_access(self):
        output = io.StringIO()
        with patch.object(native, "load_library", side_effect=AssertionError("must not load")), \
                patch.object(stress, "ZeroGainSession", side_effect=AssertionError("must not create")), \
                redirect_stdout(output):
            self.assertEqual(stress.main([]), 0)
        result = json.loads(output.getvalue())
        self.assertEqual(result["status"], "PLAN_ONLY")
        self.assertEqual(result["cycles"], 5)
        self.assertFalse(result["hardware_opened"])
        self.assertFalse(result["positive_gain_available"])
        self.assertIsNone(result["timer_slack_ns"])
        with patch.object(stress.timer_slack, "_load_prctl") as load, redirect_stdout(io.StringIO()):
            self.assertEqual(stress.main(["--timer-slack-ns", "1000"]), 0)
        load.assert_not_called()

    def test_exact_condition_and_cycle_allowlists(self):
        self.assertEqual(stress.CONDITIONS, {
            "800us-window3": (800, 3), "800us-window2": (800, 2),
            "900us-window3": (900, 3), "1000us-window3": (1000, 3)})
        for condition, (gap, window) in stress.CONDITIONS.items():
            for cycles in (5, 50, 500):
                result = stress.plan(cycles, condition=condition)
                self.assertEqual((result["request_gap_us"], result["request_window"]), (gap, window))
            output = io.StringIO()
            with patch.object(native, "load_library", side_effect=AssertionError("must not load")), \
                    patch.object(stress, "ZeroGainSession", side_effect=AssertionError("must not create")), \
                    redirect_stdout(output):
                self.assertEqual(stress.main(["--condition", condition]), 0)
            result = json.loads(output.getvalue())
            self.assertEqual(result["status"], "PLAN_ONLY")
            self.assertEqual(result["condition"], condition)
            self.assertEqual((result["request_gap_us"], result["request_window"]), (gap, window))
            self.assertEqual(result["diagnostic_reply_deadline_ms"], 25)
            self.assertFalse(result["hardware_opened"])
        for cycles in (0, 1, 6, 501, True, 5.):
            with self.assertRaises(ValueError):
                stress.plan(cycles)
        for condition in ("", "all", "600us-window3", "1000us-window2", None):
            with self.assertRaises(ValueError):
                stress.plan(condition=condition)
        for value in (0, True, 1000., 50_000):
            with self.assertRaises(ValueError):
                stress.plan(timer_slack_ns=value)

    def test_optional_timer_slack_is_changed_and_restored_only_on_bus_owners(self):
        values, calls = {}, []
        class Backend:
            def get(self):
                return values.get(threading.get_native_id(), 50_000)
            def set(self, value):
                tid = threading.get_native_id()
                calls.append((tid, value)); values[tid] = value
        rig = self.make_rig()
        with patch.object(stress.timer_slack, "require_supported_platform"), \
                patch.object(stress.timer_slack, "_load_prctl", return_value=Backend()) as load:
            result = rig.run(timer_slack_ns=1000)
        self.assertEqual(result["status"], "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC", result["errors"])
        report = result["timer_slack"]
        self.assertTrue(report["apply_verified"])
        self.assertTrue(report["restoration_complete"])
        self.assertEqual(report["status"], "restored")
        self.assertEqual(load.call_count, 2)
        tids = {value["parent"]["native_tid"] for value in report["owners"].values()}
        self.assertEqual(len(tids), 2)
        self.assertNotIn(threading.get_native_id(), tids)
        for tid in tids:
            self.assertEqual([value for owner, value in calls if owner == tid], [1000, 50_000])
        for scope in report["owners"].values():
            self.assertEqual(scope["parent"]["original_ns"], 50_000)
            self.assertEqual(scope["parent"]["during_ns"], 1000)
            self.assertEqual(scope["parent"]["after_ns"], 50_000)
            self.assertTrue(scope["parent"]["restored"])
        with patch.object(stress.timer_slack, "_load_prctl") as load:
            result = self.make_rig().run()
        load.assert_not_called()
        self.assertEqual(result["timer_slack"]["status"], "inactive")

    def test_timer_slack_apply_and_restore_failures_are_not_success(self):
        from test_policy_output_runtime import FakeSession, SimulatedClock

        class LogicalZeroGainSession(FakeSession, stress.ZeroGainSession):
            """ABI replies on the real owners, isolated from host I/O timing."""
            def __init__(self, first_id, clock):
                FakeSession.__init__(self, first_id, clock=clock)
                self.first_id = first_id
                self.condition = "800us-window3"

            def exchange(self, *args, **kwargs):
                records, stats = super().exchange(*args, **kwargs)
                stats.rejected_total = 0
                return records, stats

            def emergency_stop_repeated(self):
                return self.emergency_stop()

        for failure in ("apply", "restore"):
            with self.subTest(failure=failure):
                values = {}
                class Backend:
                    def get(self):
                        return values.get(threading.get_native_id(), 50_000)
                    def set(self, value):
                        if value == 1000 and failure == "apply":
                            return  # Readback remains 50us, causing preflight refusal.
                        if value == 50_000 and failure == "restore":
                            raise RuntimeError("injected restore failure")
                        values[threading.get_native_id()] = value
                # This test covers timer-slack failure classification/restoration,
                # not C++ transport latency. Wall-clock socket scheduling can
                # trigger an unrelated deadline before restoration, correctly
                # leaving status ABORTED and masking the injected failure here.
                # Keep the real owner pools, watchdog and STOP validation; use
                # causal ABI reply time without widening any runtime limits.
                clock = SimulatedClock()
                sessions = {scope: LogicalZeroGainSession(ids[0], clock)
                            for scope, ids in stress.BUSES.items()}
                with patch.object(stress.timer_slack, "require_supported_platform"), \
                        patch.object(stress.timer_slack, "_load_prctl", return_value=Backend()):
                    result = stress.run(sessions, EXPECTED, cycles=5,
                        condition="800us-window3", cancel_io=lambda: None,
                        clock=clock, wait=clock.sleep, timer_slack_ns=1000)
                self.assertTrue(result["stop_confirmed"], result["errors"])
                self.assertNotEqual(result["status"], "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC")
                self.assertEqual(result["timer_slack"]["status"], "failed")
                if failure == "apply":
                    self.assertFalse(result["motor_enable_attempted"])
                    self.assertFalse(result["timer_slack"]["apply_verified"])
                    self.assertTrue(result["timer_slack"]["restoration_complete"])
                else:
                    self.assertEqual(result["cycles_completed"], 5, result["errors"])
                    self.assertEqual(result["status"], "ABORTED_TIMER_SLACK_RESTORE", result["errors"])
                    self.assertFalse(result["timer_slack"]["restoration_complete"])
                    self.assertEqual(len(result["errors"]), 2, result["errors"])
                    self.assertTrue(all("injected restore failure" in error
                                        for error in result["errors"]), result["errors"])

    def test_watchdog_close_failure_still_restores_owner_slack_and_closes_pools(self):
        from singularitydog_hw import policy_output_runtime as runtime
        values, pools_closed = {}, []
        class Backend:
            def get(self):
                return values.get(threading.get_native_id(), 50_000)
            def set(self, value):
                values[threading.get_native_id()] = value
        close_watchdog = runtime.OutputWatchdog.close
        close_workers = runtime.BusWorkers.close
        def failing_close(watcher):
            close_watchdog(watcher)
            raise RuntimeError("injected watchdog close failure")
        def tracked_close(workers):
            close_workers(workers)
            pools_closed.append(True)
        rig = self.make_rig()
        with patch.object(stress.timer_slack, "require_supported_platform"), \
                patch.object(stress.timer_slack, "_load_prctl", return_value=Backend()), \
                patch.object(runtime.OutputWatchdog, "close", failing_close), \
                patch.object(runtime.BusWorkers, "close", tracked_close):
            result = rig.run(timer_slack_ns=1000)
        self.assertEqual(result["status"], "ABORTED_WATCHDOG_CLOSE")
        self.assertTrue(result["stop_confirmed"])
        self.assertTrue(result["timer_slack"]["restoration_complete"])
        self.assertEqual(result["timer_slack"]["status"], "restored")
        self.assertTrue(all(value == 50_000 for value in values.values()))
        self.assertEqual(pools_closed, [True])
        self.assertTrue(any("injected watchdog close failure" in error for error in result["errors"]))

    def test_execute_missing_prerequisites_never_loads_library(self):
        with patch.object(native, "load_library", side_effect=AssertionError("must not load")):
            with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
                stress.main(["--execute-supported-zero-gain"])

    def test_all_four_conditions_exact_26_requests_and_single_owners(self):
        for condition, (gap, _) in stress.CONDITIONS.items():
            with self.subTest(condition=condition):
                rig = self.make_rig(condition=condition)
                announced = []
                def announce():
                    self.assertFalse(rig.enabled)
                    announced.append(True)
                result = rig.run(announce=announce)
                self.assertEqual(result["status"], "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC", result["errors"])
                self.assertEqual(result["cycles_completed"], 5)
                self.assertEqual(len(result["cycles"]), 5)
                self.assertEqual(announced, [True])
                self.assertTrue(result["stop_confirmed"])
                self.assertFalse(result["positive_gain_sent"] or result["learned_targets_sent"])
                for scope, ids in stress.BUSES.items():
                    summary = result["transport_summary"][scope]
                    self.assertEqual(summary["cyclic_writes"], 65)
                    self.assertEqual(summary["cyclic_complete_replies"], 65)
                    self.assertEqual(summary["cyclic_rx_bytes"], 65*17)
                    self.assertGreaterEqual(summary["cyclic_write_gap_us"]["min"], gap)
                    calls = rig.sessions[scope].owner_calls
                    self.assertEqual(len({owner for _, owner in calls}), 1)
                    self.assertEqual(calls[-1][0], "stop")
                    self.assertEqual(result["stop_reports"][scope]["retry_policy"]["attempts_completed"], 1)
                    for cycle in range(1, 6):
                        batches = [row for row in result["journal"] if row["bus"] == scope and
                                   row["phase"].startswith(f"cycle_{cycle}_")]
                        self.assertEqual([row["phase"].rsplit("_", 1)[1] for row in batches],
                                         ["input", "voltage", "output"])
                        self.assertEqual([len(row["records"]) for row in batches], [6, 1, 6])
                        self.assertEqual(len({r["deadline_ns"] for row in batches for r in row["records"]}), 1)
                for _, request, _ in rig.seen:
                    if request.kind == 1:
                        self.assertEqual((request.can_id >> 8)&65535, 32767)
                        self.assertEqual(request.data[2:], b"\x7f\xff"+bytes(4))
                self.assertTrue(all(row["start_interval_ms"] >= 20
                                    for row in result["per_cycle"][1:]))

    def test_native_immutable_caps_reject_positive_gain_before_write(self):
        for gain in ((1., 0.), (0., .1)):
            rig = self.make_rig()
            with self.assertRaisesRegex(native.ExchangeError, "Disallowed"):
                rig.sessions["front"].exchange([native.encode_motion(1, 0., *gain)])
            self.assertFalse(rig.seen)

    def test_feedback_guard_identifies_measurement_and_unchanged_limit(self):
        values = dict(protocol_position_rad=0., velocity_rad_s=0., temperature_c=25.,
                      mode_state=2, fault_bits=0)
        cases = (
            ("protocol_position_rad", float("nan"), "non-finite position_rad=nan", "[-12.57, 12.57]"),
            ("protocol_position_rad", 12.58, "position_rad=12.58", "[-12.57, 12.57]"),
            ("velocity_rad_s", float("nan"), "non-finite velocity_rad_s=nan", "abs <= 0.5"),
            ("velocity_rad_s", .505836575875, "velocity_rad_s=0.505836575875", "abs limit 0.5 rad/s"),
            ("temperature_c", float("nan"), "non-finite temperature_c=nan", "[-10, 60)"),
            ("temperature_c", 60., "temperature_c=60.0", "[-10, 60) C"),
        )
        for field, value, measurement, limit in cases:
            with self.subTest(field=field, value=value):
                feedback = SimpleNamespace(**{**values, field: value})
                with self.assertRaises(RuntimeError) as caught:
                    stress._feedback({(4, "feedback"): (feedback,)}, (4,), mode=2)
                message = str(caught.exception)
                self.assertTrue(message.startswith("ID4 "), message)
                self.assertIn(measurement, message)
                self.assertIn(limit, message)
        for q, velocity, temperature in ((-12.57, -.5, -10.), (12.57, .5, 59.9)):
            feedback = SimpleNamespace(**{**values, "protocol_position_rad": q,
                                          "velocity_rad_s": velocity, "temperature_c": temperature})
            self.assertEqual(stress._feedback({(4, "feedback"): (feedback,)}, (4,), mode=2), {4: q})
        feedback = SimpleNamespace(**{**values, "mode_state": 0, "protocol_position_rad": float("nan")})
        with self.assertRaisesRegex(RuntimeError, "ID4 native feedback mode/fault mismatch"):
            stress._feedback({(4, "feedback"): (feedback,)}, (4,), mode=2)

    def test_unsafe_cyclic_feedback_records_reason_and_attempts_stop_for_both_owners(self):
        for velocity_code, temperature_code, reason in ((33099, 250, "velocity_rad_s=0.505836575875"),
                                                       (32767, 600, "temperature_c=60.0")):
            with self.subTest(reason=reason):
                def mutate(rig, scope, request, response):
                    if request.kind == 1 and request.destination == 4 and rig.counts[4, 1] == 3:
                        return frame((2 << 24)|(2 << 22)|(4 << 8)|0xfd,
                                     struct.pack(">4H", 32767, velocity_code, 32767, temperature_code))
                    return response
                rig = self.make_rig(mutate=mutate)
                result = rig.run()
                expected_status = ("ABORTED" if result["stop_confirmed"] else
                                   "STOP_UNCONFIRMED_POWER_OFF_REQUIRED")
                self.assertEqual(result["status"], expected_status, result["errors"])
                self.assertEqual(result["cycles_completed"], 0)
                self.assertTrue(any("ID4 " + reason in error for error in result["errors"]), result["errors"])
                self.assertEqual(rig.counts[4, 1], 3)
                self.assertIsNotNone(rig.cancelled_at)
                self.assertTrue(all(session.owner_calls[-1][0] == "stop" for session in rig.sessions.values()))
                for scope, ids in stress.BUSES.items():
                    stop = result["stop_reports"][scope]
                    self.assertEqual(stop["attempts"][0]["attempted_ids"], list(ids))
                    self.assertEqual({request.destination for bus, request, timestamp in rig.seen
                                      if bus == scope and request.kind == 4 and timestamp >= rig.cancelled_at},
                                     set(ids))
                    # Cancellation may catch a peer Type1 awaiting its reply.
                    # Every such ID must stay ambiguous through all STOP retries.
                    unresolved = set()
                    for row in result["journal"]:
                        if row["bus"] == scope:
                            for record in row["records"]:
                                if record["written"] and not record["received"]:
                                    request = ATParser().feed(bytes.fromhex(record["tx_hex"]))[0]
                                    if request.kind == 1:
                                        unresolved.add(request.destination)
                    self.assertTrue(unresolved <= set(stop["ambiguous_ids"]), stop)
                    self.assertTrue(set(stop["ambiguous_ids"]) <= set(stop["unconfirmed_ids"]), stop)
                    for attempt in stop["attempts"]:
                        self.assertTrue(unresolved.isdisjoint(attempt["confirmed_ids"]), attempt)
                    if stop["complete"]:
                        self.assertEqual(stop["confirmed_ids"], list(ids))
                        self.assertFalse(stop["ambiguous_ids"] or stop["unconfirmed_ids"])
                    else:
                        self.assertTrue(stop["ambiguous_ids"], stop)
                        self.assertFalse(result["stop_confirmed"])

    def test_wrong_condition_cannot_reuse_other_pacing_session(self):
        rig = self.make_rig()
        with self.assertRaises(ValueError):
            stress.run(rig.sessions, EXPECTED, cycles=5, condition="800us-window2", cancel_io=rig.cancel)
        self.assertFalse(rig.seen)

    def test_uid_mismatch_aborts_before_announcement_and_enable(self):
        def mutate(rig, scope, request, response):
            if request.kind == 0 and request.destination == 4:
                return frame((4 << 8)|0xfe, bytes([99])*8)
            return response
        rig = self.make_rig(mutate=mutate)
        announced = []
        result = rig.run(announce=lambda: announced.append(True))
        self.assertEqual(result["status"], "ABORTED")
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(announced)
        self.assertFalse(any(request.kind in (1, 3) for _, request, _ in rig.seen))

    def test_wrong_watchdog_or_voltage_aborts_before_enable(self):
        for index, data in ((0x7028, struct.pack("<I", 3999)), (0x701c, struct.pack("<f", 34.))):
            with self.subTest(index=index):
                def mutate(rig, scope, request, response):
                    if request.kind == 17 and request.destination == 1 and int.from_bytes(request.data[:2], "little") == index:
                        return frame((17 << 24)|(1 << 8)|0xfd, request.data[:4]+data)
                    return response
                rig = self.make_rig(mutate=mutate)
                result = rig.run()
                self.assertEqual(result["status"], "ABORTED", result["errors"])
                self.assertFalse(any(request.kind in (1, 3) for _, request, _ in rig.seen))
                self.assertTrue(result["stop_confirmed"])

    def test_announcement_failure_and_cancellation_never_enable(self):
        for interrupted in (False, True):
            with self.subTest(interrupted=interrupted):
                rig = self.make_rig()
                def announce():
                    if interrupted:
                        rig.cancel()
                    else:
                        raise RuntimeError("audio failed")
                def check():
                    if rig.cancelled_at is not None:
                        raise InterruptedError("cancelled")
                result = rig.run(announce=announce, check=check)
                self.assertEqual(result["status"], "ABORTED")
                self.assertTrue(result["stop_confirmed"])
                self.assertFalse(any(request.kind in (1, 3) for _, request, _ in rig.seen))

    def test_missing_id4_retains_85_bytes_no_motion_retry_and_sticky_stop(self):
        def mutate(rig, scope, request, response):
            if request.kind == 1 and request.destination == 4 and rig.counts[4, 1] == 2:
                return None
            return response
        rig = self.make_rig(mutate=mutate)
        result = rig.run()
        self.assertEqual(result["status"], "STOP_UNCONFIRMED_POWER_OFF_REQUIRED", result["errors"])
        self.assertEqual(result["cycles_completed"], 0)
        self.assertFalse(result["stop_confirmed"])
        failed = [row for row in result["journal"] if row["bus"] == "front" and row["error"]]
        self.assertEqual(len(failed), 1)
        self.assertEqual(failed[0]["phase"], "cycle_1_input")
        self.assertEqual(failed[0]["stats"]["bytes"], 85)
        self.assertEqual(failed[0]["rejected_hex"], "")
        self.assertEqual(sum(row["written"] == 17 for row in failed[0]["records"]), 6)
        self.assertEqual(rig.counts[4, 1], 2)  # One startup zero, one missing cyclic request.
        stop = result["stop_reports"]["front"]
        self.assertEqual(stop["ambiguous_ids"], [4])
        self.assertEqual(stop["unconfirmed_ids"], [4])
        self.assertEqual(stop["retry_policy"]["attempts_completed"], 3)
        self.assertTrue(all(4 not in attempt["confirmed_ids"] for attempt in stop["attempts"]))
        self.assertTrue(all(kind == "stop" for kind, _ in rig.sessions["front"].owner_calls[-1:]))
        spec = importlib.util.spec_from_file_location("loss_analysis", ROOT/"tools/analyze_can_reply_loss.py")
        analyzer = importlib.util.module_from_spec(spec); spec.loader.exec_module(analyzer)
        analysis = analyzer.analyze(result)
        self.assertEqual(analysis["unresolved_counts_by_id"], {"4": 1})

    def test_bad_cyclic_voltage_prevents_later_type1_and_cancels_peer(self):
        def mutate(rig, scope, request, response):
            # Preflight and post-audio voltage come before any Type1.
            if request.kind == 17 and request.destination == 1 and rig.counts[1, 1] >= 2:
                return frame((17 << 24)|(1 << 8)|0xfd, request.data[:4]+struct.pack("<f", 34.))
            return response
        rig = self.make_rig(mutate=mutate)
        result = rig.run()
        self.assertEqual(result["cycles_completed"], 0)
        self.assertIn("voltage", result["errors"][0])
        self.assertEqual(rig.counts[1, 1], 2)
        self.assertIsNotNone(rig.cancelled_at)
        self.assertTrue(all(session.owner_calls[-1][0] == "stop" for session in rig.sessions.values()))

    def test_native_fragmented_replies_and_release_waiter(self):
        rig = self.make_rig(fragment=True)
        waits = []
        def release(target):
            actual = native.wait_until(self.library, rig.cr, target, spin_us=500)
            waits.append((target, actual))
            return actual
        result = rig.run(deadline_wait=release)
        self.assertEqual(result["status"], "COMPLETE_NATIVE_ZERO_TYPE1_DIAGNOSTIC", result["errors"])
        self.assertTrue(result["native_release_wait"])
        self.assertTrue(waits)
        self.assertTrue(all(actual >= target for target, actual in waits))
        self.assertTrue(all(not row["rejected_hex"] for row in result["journal"]))

    def test_boot_change_after_announcement_stops_before_enable(self):
        rig = self.make_rig()
        def announce():
            rig.boot.seek(0); rig.boot.write(b"22222222-2222-3333-4444-555555555555\n"); rig.boot.flush()
        result = rig.run(announce=announce)
        self.assertEqual(result["status"], "ABORTED")
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(any(request.kind in (1, 3) for _, request, _ in rig.seen))

    def test_window_two_does_not_send_third_request_before_credit(self):
        rig = self.make_rig(condition="800us-window2")
        rig.enabled.update(stress.BUSES["front"])
        held = []
        def mutate(rig, scope, request, response):
            if scope == "front" and request.kind == 1:
                held.append(response)
                if len(held) == 2:
                    # Hold both credits longer than the configured write gap.
                    self.assertFalse(select.select([rig.peers[scope]], [], [], .003)[0])
                    answer = b"".join(reversed(held)); held.clear(); return answer
                return None
            return response
        rig.mutate = mutate
        records, stats = rig.sessions["front"].exchange(
            [native.encode_motion(mid, 0., 0., 0.) for mid in range(1, 7)])
        self.assertEqual(stats.writes, 6)
        self.assertTrue(all(row.received == 17 for row in records))
        self.assertGreaterEqual(records[2].start_ns, records[1].received_ns)


if __name__ == "__main__":
    unittest.main()
