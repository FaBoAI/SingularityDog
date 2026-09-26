"""Firmware-backup protocol and ownership tests; fake clocks and ports only."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import struct
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import firmware_backup as backup
from singularitydog_hw import can_readonly as codec
from singularitydog_hw.serial_deadline_reader import ReceivedChunk


MS = 1_000_000
UIDS = {mid: f"{mid:016x}" for mid in range(1, 13)}
PARAMETERS = ("run_mode", "position", "current", "velocity", "voltage",
              "can_timeout", "zero_state")
VALUES = {"run_mode": 0, "position": -1.25, "current": .03125,
          "velocity": -.125, "voltage": 40., "can_timeout": 4000, "zero_state": 0}
BINDINGS = {scope: {"path": scope, "resolved": scope, "st_rdev": number}
            for scope, number in (("front", 42), ("rear", 43))}


def wire(kind, mid, data, *, destination=0xFD, status=0, flags=4):
    can_id = kind << 24 | status << 16 | mid << 8 | destination
    return (b"AT" + ((can_id << 3) | flags).to_bytes(4, "big")
            + bytes([len(data)]) + data + b"\r\n")


def reply(mid, parameter, *, value=None, status=0):
    if parameter is None:
        return wire(0, mid, bytes.fromhex(UIDS[mid]), destination=0xFE)
    index, fmt, _ = codec.PARAMETERS[parameter]
    data = bytearray(struct.pack("<H", index) + bytes(6))
    struct.pack_into("<" + fmt, data, 4, VALUES[parameter] if value is None else value)
    return wire(17, mid, bytes(data), status=status)


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now


class Port:
    def __init__(self, clock, *, fault=None, target=(1, None), write_ns=50_000,
                 reply_ns=MS, values=None):
        self.clock, self.fault, self.target = clock, fault, target
        self.write_ns, self.reply_ns = write_ns, reply_ns
        self.values = {} if values is None else values
        self.pending, self.writes = [], []
        self.write_timeout = .1

    def write(self, data):
        parser = codec.ATParser()
        frames = parser.feed(data)
        assert len(frames) == 1 and len(data) == 17
        assert not parser.buffer and not parser.discarded_bytes
        frame, = frames
        assert frame.flags == 4 and frame.source == codec.HOST_ID
        assert frame.kind in (0, 17), "Only identity and parameter reads may reach a port"
        mid = frame.destination
        if frame.kind == 0:
            parameter = None
            assert frame.data == bytes(8)
        else:
            index = int.from_bytes(frame.data[:2], "little")
            parameter, = [name for name in PARAMETERS if codec.PARAMETERS[name][0] == index]
            assert frame.data[2:] == bytes(6)
        started = self.clock()
        self.writes.append({"mid": mid, "parameter": parameter, "wire": data,
                            "started": started})
        self.clock.now += self.write_ns
        answer = reply(mid, parameter, value=self.values.get(parameter))
        active = self.fault if (mid, parameter) == self.target else None
        if active == "partial":
            return len(data) - 1
        if active == "exception":
            raise OSError("write failed after uncertain acceptance")
        if active == "boolean_write":
            return True
        if active == "missing":
            return len(data)
        if active == "wrong_uid":
            answer = wire(0, mid, b"\xff" * 8, destination=0xFE)
        elif active == "wrong_id":
            answer = reply(mid + 1, parameter)
        elif active == "wrong_parameter":
            answer = reply(mid, "position")
        elif active == "negative":
            answer = reply(mid, parameter, status=1)
        elif active == "duplicate":
            answer += answer
        elif active == "noise":
            answer = b"noise" + answer
        elif active == "terminator":
            answer = answer[:-2] + b"xx"
        elif active == "partial_tail":
            answer += b"A"
        elif active in ("wrong_flags", "wrong_destination", "wrong_kind"):
            decoded, = codec.ATParser().feed(answer)
            answer = wire(2 if active == "wrong_kind" else decoded.kind, mid,
                          decoded.data, destination=0xFC if active == "wrong_destination"
                          else decoded.destination, flags=0 if active == "wrong_flags" else 4)
        due = started + 250 * MS if active == "late" else self.clock() + self.reply_ns
        if active == "fragmented":
            self.pending.extend(((due, answer[:1]), (due + MS, answer[1:9]),
                                 (due + 2 * MS, answer[9:])))
        else:
            self.pending.append((due, answer))
        if active == "quiet_duplicate":
            self.pending.append((due + 50 * MS, answer))
        return len(data)


class Reader:
    def __init__(self, raw, *, clock, check):
        self.raw, self.clock, self.check = raw, clock, check

    def read_until(self, wake, hard):
        self.check()
        due = min((at for at, _ in self.raw.pending), default=wake)
        self.clock.now = max(self.clock(), min(due, wake, hard))
        self.check()
        chunk = b"".join(data for at, data in self.raw.pending if at <= self.clock())
        self.raw.pending = [(at, data) for at, data in self.raw.pending if at > self.clock()]
        return chunk, self.clock()


class BackupProbeTests(unittest.TestCase):
    def test_reader_rejected_bytes_are_saved_without_identity_or_more_requests(self):
        for timestamp_known in (True, False):
            with self.subTest(timestamp_known=timestamp_known):
                evidence = []
                class RejectedReader(Reader):
                    def read_until(self, wake, hard):
                        chunk, stamp = super().read_until(wake, hard)
                        if chunk:
                            item = ReceivedChunk(chunk, stamp, stamp if timestamp_known else None)
                            evidence.append(item)
                            error = RuntimeError("post-read binding changed")
                            error.serial_read_evidence = item
                            raise error
                        return chunk, stamp
                instance, port, _ = self.build(reader_factory=RejectedReader)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("post-read binding changed", result["failure"])
                self.assertEqual(len(port.writes), 1)
                self.assertNotIn("identity", result["motors"].get("1", {}))
                self.assertEqual(instance.raw_bytes, len(evidence[0].data))
                self.assertEqual(result["rejected_receive_chunks"], 1)
                if timestamp_known:
                    self.assertEqual(len(instance.raw_log), 1)
                    self.assertEqual(instance.raw_log[0][1:], (evidence[0].received_ns, evidence[0].data))
                else:
                    self.assertEqual(instance.raw_log, [])
                    self.assertEqual(result["unclocked_receive_evidence"], [evidence[0].record()])

    def build(self, ids=(1,), *, expected=None, fault=None, target=(1, None),
              values=None, reader_factory=Reader, check=lambda: None,
              deadline_ns=None, write_ns=50_000, reply_ns=MS):
        clock = Clock()
        port = Port(clock, fault=fault, target=target, values=values,
                    write_ns=write_ns, reply_ns=reply_ns)
        probe = backup.BackupProbe(port, ids, UIDS if expected is None else expected,
            clock=clock, check=check, deadline_ns=deadline_ns, reader_factory=reader_factory)
        return probe, port, clock

    def assert_failed_without_further_tx(self, probe, port):
        report = probe.run()
        self.assertEqual(report["status"], "INCOMPLETE")
        before = len(port.writes)
        with self.assertRaises((ValueError, RuntimeError)):
            probe._send(1, None)
        self.assertEqual(len(port.writes), before)
        return report

    def test_success_records_all_raw_parameters_in_exact_per_id_read_order(self):
        probe, port, clock = self.build(ids=(1, 3, 6))
        started = clock()
        report = probe.run()
        self.assertEqual(report["status"], "FIRMWARE_BACKUP_COMPLETE")
        self.assertEqual([(row["mid"], row["parameter"]) for row in port.writes],
            [(mid, name) for mid in (1, 3, 6) for name in (None, *PARAMETERS)])
        self.assertEqual(len(probe.tx_log), 24)
        self.assertEqual(sum(len(raw) for _, _, raw in probe.raw_log), 24 * 17)
        self.assertGreaterEqual(port.writes[0]["started"] - started, 100 * MS)
        self.assertGreaterEqual(clock() - probe.raw_log[-1][1], 100 * MS)
        for mid in (1, 3, 6):
            params = report["motors"][str(mid)]["parameters"]
            self.assertEqual(set(params), set(PARAMETERS))
            for name, value in VALUES.items():
                self.assertTrue(params[name]["ok"])
                self.assertEqual(params[name]["status"], 0)
                self.assertEqual(params[name]["value"], value)
                self.assertEqual(len(params[name]["raw_value_hex"]), 8)

    def test_nonzero_mode_and_zero_state_are_preserved_without_disabled_inference(self):
        probe, port, _ = self.build(values={"run_mode": 3, "zero_state": 7, "can_timeout": 0})
        report = probe.run()
        self.assertEqual(report["status"], "FIRMWARE_BACKUP_COMPLETE")
        params = report["motors"]["1"]["parameters"]
        self.assertEqual(params["run_mode"]["value"], 3)
        self.assertEqual(params["zero_state"]["value"], 7)
        self.assertEqual(params["can_timeout"]["value"], 0)
        self.assertTrue(all(codec.ATParser().feed(row["wire"])[0].kind in (0, 17)
                            for row in port.writes))

    def test_rear_subset_and_fragmented_reply_complete_without_cross_scope_reads(self):
        probe, port, _ = self.build(ids=(7, 12), fault="fragmented", target=(7, None))
        self.assertEqual(probe.run()["status"], "FIRMWARE_BACKUP_COMPLETE")
        self.assertEqual({row["mid"] for row in port.writes}, {7, 12})
        self.assertEqual(len(port.writes), 16)
        self.assertGreater(len(probe.raw_log), 16)

    def test_invalid_ids_or_incomplete_uid_map_rejected_before_io(self):
        for ids in ((), [1], (True,), (0,), (13,), (2, 1), (1, 1), (1, 7)):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                self.build(ids=ids)
        for expected in ({1: UIDS[1]}, {**UIDS, 2: UIDS[1]}, {**UIDS, 1: "bad"}):
            with self.subTest(expected_keys=list(expected)), self.assertRaises(ValueError):
                self.build(expected=expected)

    def test_out_of_schedule_or_unpermitted_send_is_rejected_before_write(self):
        for mid, name in ((2, None), (1, "position"), (1, "period_set"),
                          (1, "report_on"), (1, b"arbitrary")):
            probe, port, _ = self.build()
            # Exercise the schedule guard of an active pass, not the new-session guard.
            probe._ran = True
            with self.subTest(mid=mid, name=name), self.assertRaisesRegex(ValueError, "schedule"):
                probe._send(mid, name)
            self.assertEqual(port.writes, [])

    def test_corrupted_encoder_cannot_cross_the_physical_write_boundary(self):
        stop_id = (4 << 24) | (codec.HOST_ID << 8) | 1
        stop = (b"AT" + ((stop_id << 3) | 4).to_bytes(4, "big")
                + b"\x08" + bytes(8) + b"\r\n")
        other_parameter = codec.read_request(1, "position")
        for encoded in (stop, other_parameter):
            with self.subTest(encoded=encoded.hex()):
                probe, port, _ = self.build()
                with patch.object(backup.codec, "read_request", return_value=encoded):
                    self.assert_failed_without_further_tx(probe, port)
                self.assertEqual(port.writes, [])
                self.assertEqual(probe.tx_log, [])

    def test_completed_probe_cannot_be_reused_or_send_an_extra_read(self):
        probe, port, _ = self.build()
        self.assertEqual(probe.run()["status"], "FIRMWARE_BACKUP_COMPLETE")
        with self.assertRaises(RuntimeError):
            probe.run()
        with self.assertRaises((ValueError, RuntimeError)):
            probe._send(1, None)
        self.assertEqual(len(port.writes), 8)

    def test_wrong_uid_aborts_before_any_parameter_or_later_id(self):
        probe, port, _ = self.build(ids=(1, 2), fault="wrong_uid")
        self.assert_failed_without_further_tx(probe, port)
        self.assertEqual([(r["mid"], r["parameter"]) for r in port.writes], [(1, None)])

    def test_negative_status_retained_and_no_later_parameter_is_sent(self):
        probe, port, _ = self.build(ids=(1, 2), fault="negative", target=(1, "can_timeout"))
        report = self.assert_failed_without_further_tx(probe, port)
        rejected = report["motors"]["1"]["parameters"]["can_timeout"]
        self.assertFalse(rejected["ok"])
        self.assertEqual(rejected["status"], 1)
        self.assertIsNone(rejected["value"])
        self.assertEqual(len(rejected["raw_value_hex"]), 8)
        self.assertEqual(port.writes[-1]["parameter"], "can_timeout")
        self.assertEqual(len(port.writes), 7)

    def test_low_high_or_nonfinite_voltage_cannot_complete(self):
        for voltage in (34.99, 45.01, float("nan"), float("inf")):
            with self.subTest(voltage=voltage):
                probe, port, _ = self.build(values={"voltage": voltage})
                self.assert_failed_without_further_tx(probe, port)
                self.assertEqual(port.writes[-1]["parameter"], "voltage")

    def test_wrong_address_type_parameter_duplicate_and_malformed_replies_abort(self):
        for fault in ("wrong_id", "wrong_flags", "wrong_destination", "wrong_kind",
                      "duplicate", "noise", "terminator", "partial_tail", "wrong_parameter"):
            target = (1, "run_mode") if fault == "wrong_parameter" else (1, None)
            with self.subTest(fault=fault):
                probe, port, _ = self.build(fault=fault, target=target)
                self.assert_failed_without_further_tx(probe, port)
                self.assertEqual(len(port.writes), 2 if fault == "wrong_parameter" else 1)

    def test_partial_exception_and_invalid_write_result_stop_all_followup_io(self):
        for fault in ("partial", "exception", "boolean_write"):
            with self.subTest(fault=fault):
                probe, port, _ = self.build(fault=fault)
                self.assert_failed_without_further_tx(probe, port)
                self.assertEqual(len(port.writes), 1)
                self.assertEqual(probe.tx_log[0]["wire_hex"], port.writes[0]["wire"].hex())

    def test_missing_late_and_slow_full_write_obey_original_query_deadline(self):
        for fault, write_ns in (("missing", 50_000), ("late", 50_000), (None, 250 * MS)):
            with self.subTest(fault=fault, write_ns=write_ns):
                probe, port, clock = self.build(fault=fault, write_ns=write_ns)
                self.assert_failed_without_further_tx(probe, port)
                self.assertEqual(len(port.writes), 1)
                self.assertLessEqual(clock() - port.writes[0]["started"], 250 * MS)

    def test_initial_bytes_and_final_quiet_duplicates_cannot_pass(self):
        probe, port, clock = self.build()
        port.pending.append((clock() + MS, reply(1, None)))
        self.assert_failed_without_further_tx(probe, port)
        self.assertEqual(port.writes, [])
        probe, port, _ = self.build(fault="quiet_duplicate", target=(1, "zero_state"))
        self.assert_failed_without_further_tx(probe, port)
        self.assertEqual(len(port.writes), 8)

    def test_external_deadline_cancellation_and_invalid_clock_stop_before_writing(self):
        probe, port, _ = self.build(deadline_ns=1_000_000_000 + 50 * MS)
        self.assert_failed_without_further_tx(probe, port)
        self.assertEqual(port.writes, [])

        def cancelled():
            raise InterruptedError("cancelled")
        probe, port, _ = self.build(check=cancelled)
        self.assert_failed_without_further_tx(probe, port)
        self.assertEqual(port.writes, [])

        class Backwards(Reader):
            def read_until(self, wake, hard):
                self.clock.now -= 1
                return b"", self.clock()
        probe, port, _ = self.build(reader_factory=Backwards)
        self.assert_failed_without_further_tx(probe, port)
        self.assertEqual(port.writes, [])

    def test_raw_budget_overflow_remains_incomplete_and_prohibits_later_reads(self):
        probe, port, _ = self.build()
        with patch.object(backup, "MAX_RAW_BYTES", 8):
            report = self.assert_failed_without_further_tx(probe, port)
        self.assertTrue(report["raw_log_overflow"])
        self.assertEqual(report["unlogged_chunk_bytes"], 17)
        self.assertEqual(probe.raw_log, [])
        self.assertEqual(len(port.writes), 1)


class FakeBootGuard:
    def __init__(self, events, *, boot_id="boot", fail_check=None):
        self.events, self.boot_id, self.fail_check = events, boot_id, fail_check
        self.checks, self.closes = 0, 0

    def check(self):
        assert not self.closes, "Boot guard closed while an owner was active"
        self.checks += 1
        if self.checks == self.fail_check:
            raise OSError("fresh boot check failed")

    def close(self):
        self.closes += 1
        self.events.append("boot-close")


class BackupOwnershipTests(unittest.TestCase):
    def execute_fake(self, *, fault=None, close_failure=None, boot="boot",
                     fail_check=None, cancelled=False, reply_ns=MS):
        events, ports = [], []
        clock = Clock()
        guard = FakeBootGuard(events, boot_id=boot, fail_check=fail_check)
        @contextlib.contextmanager
        def common_lock():
            events.append("common-open")
            try:
                yield
            finally:
                events.append("common-close")
        @contextlib.contextmanager
        def port_lock(scope):
            events.append("port-lock-" + scope)
            try:
                yield
            finally:
                events.append("port-unlock-" + scope)
        class Raw(Port):
            def __init__(self, **kwargs):
                super().__init__(clock, fault=fault, reply_ns=reply_ns)
                self.is_open = False
                ports.append(self)
            def open(self):
                self.is_open = True
                events.append("open-" + self.port)
            def close(self):
                events.append("close-" + self.port)
                if self.port == close_failure:
                    raise OSError("unconfirmed serial close")
                self.is_open = False
            def fileno(self):
                return 100 if self.port == "front" else 101
        cancel = threading.Event()
        if cancelled:
            cancel.set()
        with patch.object(backup, "ownership_locks", common_lock), \
             patch.object(backup.dual, "port_lock", port_lock), \
             patch.object(backup.dual, "binding_matches", return_value=True), \
             patch.object(backup.os, "fstat", side_effect=lambda fd:
                          SimpleNamespace(st_rdev=42 if fd == 100 else 43)):
            result = backup.execute(BINDINGS, UIDS, "boot", cancelled=cancel,
                serial_factory=Raw, clock=clock, boot_guard_factory=lambda: guard,
                reader_factory=Reader)
        return result, events, ports, guard, clock

    def test_default_cli_is_plan_only_without_boot_port_or_lock_access(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), patch.object(backup, "execute") as execute, \
             patch.object(backup.dual, "validate_ports") as ports, \
             patch.object(backup, "BootIdentityGuard") as guard, \
             patch.object(backup, "ownership_locks") as locks:
            self.assertEqual(backup.main([]), 0)
        execute.assert_not_called()
        ports.assert_not_called()
        guard.assert_not_called()
        locks.assert_not_called()
        self.assertEqual(json.loads(out.getvalue())["allowed_can_types"], [0, 17])

    def test_live_cli_requires_all_pinned_arguments_before_execution(self):
        with contextlib.redirect_stderr(io.StringIO()), patch.object(backup, "execute") as execute:
            with self.assertRaises((ValueError, SystemExit)):
                backup.main(["--execute-read-only"])
        execute.assert_not_called()

    def test_cli_saves_raw_tx_and_hashed_summary_after_complete_or_incomplete_execution(self):
        for status, exit_code in (("FIRMWARE_BACKUP_COMPLETE", 0), ("INCOMPLETE", 2)):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                uid_bytes = json.dumps(UIDS, sort_keys=True).encode()
                uid_file = root / "uids.json"
                uid_file.write_bytes(uid_bytes)
                output = root / "capture"
                tx = [{"motor_id": 1, "parameter": "identity", "wire_hex": "4154",
                       "started_ns": 10, "finished_ns": 20, "returned_bytes": 2}]
                probe = SimpleNamespace(tx_log=tx, raw_log=[(21, 22, b"\x00\xff")])

                def execute(bindings, expected, boot_id, *, cancelled):
                    self.assertEqual(bindings, BINDINGS)
                    self.assertEqual(expected, UIDS)
                    self.assertEqual(boot_id, "boot")
                    self.assertFalse(cancelled.is_set())
                    self.assertEqual(list(output.iterdir()), [])
                    return {"status": status}, {"front": probe}

                with patch.object(backup.dual, "validate_ports", return_value=BINDINGS), \
                     patch.object(backup, "execute", side_effect=execute) as call, \
                     patch.object(backup.signal, "signal", return_value=backup.signal.SIG_DFL), \
                     contextlib.redirect_stdout(io.StringIO()):
                    result = backup.main(["--execute-read-only", "--front-port", "front",
                        "--rear-port", "rear", "--expected-uids", str(uid_file),
                        "--expected-boot-id", "boot", "--output", str(output)])
                self.assertEqual(result, exit_code)
                call.assert_called_once()
                self.assertEqual({p.name for p in output.iterdir()},
                                 {"front-tx.json", "front-raw.json", "summary.json"})
                self.assertEqual(json.loads((output / "front-tx.json").read_text()), tx)
                self.assertEqual(json.loads((output / "front-raw.json").read_text()),
                                 [{"read_started_ns": 21, "received_ns": 22, "hex": "00ff"}])
                summary = json.loads((output / "summary.json").read_text())
                self.assertEqual(summary["status"], status)
                self.assertEqual(summary["expected_uid_file_sha256"], hashlib.sha256(uid_bytes).hexdigest())
                self.assertEqual(summary["source_sha256"], {
                    p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in Path(backup.__file__).parent.glob("*.py")})

    def test_sequential_ports_keep_common_lease_and_boot_guard_until_both_close(self):
        (report, probes), events, ports, guard, _ = self.execute_fake()
        self.assertEqual(report["status"], "FIRMWARE_BACKUP_COMPLETE")
        self.assertEqual(set(probes), {"front", "rear"})
        self.assertEqual([len(p.writes) for p in ports], [48, 48])
        self.assertLess(events.index("close-front"), events.index("open-rear"))
        for scope in ("front", "rear"):
            self.assertLess(events.index("close-" + scope), events.index("port-unlock-" + scope))
            self.assertLess(events.index("port-unlock-" + scope), events.index("common-close"))
            self.assertLess(events.index("close-" + scope), events.index("boot-close"))
        self.assertGreater(guard.checks, 96)
        self.assertEqual(guard.closes, 1)

    def test_front_failure_closes_owner_and_never_opens_rear(self):
        (report, _), events, ports, guard, _ = self.execute_fake(fault="missing")
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertEqual(len(ports), 1)
        self.assertNotIn("open-rear", events)
        self.assertEqual(len(ports[0].writes), 1)
        self.assertLess(events.index("close-front"), events.index("common-close"))
        self.assertEqual(guard.closes, 1)

    def test_fresh_boot_check_failure_prevents_writes_and_closes_monitor(self):
        (report, _), _, ports, guard, _ = self.execute_fake(fail_check=1)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertTrue(all(not p.writes for p in ports))
        self.assertEqual(guard.closes, 1)

    def test_pinned_boot_mismatch_closes_guard_before_any_ownership_or_serial_open(self):
        (report, probes), events, ports, guard, _ = self.execute_fake(boot="different")
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertEqual(probes, {})
        self.assertEqual(ports, [])
        self.assertEqual(events, ["boot-close"])
        self.assertEqual(guard.closes, 1)

    def test_cancel_before_execution_prevents_any_serial_open(self):
        (report, _), events, ports, guard, _ = self.execute_fake(cancelled=True)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertFalse(any(e.startswith("open-") for e in events))
        self.assertTrue(all(not p.writes for p in ports))
        self.assertLessEqual(guard.closes, 1)

    def test_global_twenty_second_deadline_is_shared_across_buses(self):
        (report, _), _, ports, guard, clock = self.execute_fake(reply_ns=240 * MS)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertEqual(len(ports), 2)
        self.assertLess(len(ports[1].writes), 48)
        self.assertLessEqual(clock() - 1_000_000_000, 20_000 * MS)
        self.assertEqual(guard.closes, 1)

    def test_close_failure_retains_port_and_common_locks_and_prevents_rear(self):
        before = len(backup._HELD_LOCKS)
        try:
            (report, _), events, _, guard, _ = self.execute_fake(close_failure="front")
            self.assertEqual(report["status"], "INCOMPLETE")
            self.assertNotIn("open-rear", events)
            self.assertNotIn("port-unlock-front", events)
            self.assertNotIn("common-close", events)
            self.assertGreater(len(backup._HELD_LOCKS), before)
            self.assertEqual(guard.closes, 1)
        finally:
            # These are exclusively fake context managers from execute_fake.
            for held in backup._HELD_LOCKS[before:]:
                if hasattr(held, "close"):
                    held.close()
                elif hasattr(held, "release"):
                    held.release()
            del backup._HELD_LOCKS[before:]


if __name__ == "__main__":
    unittest.main()
