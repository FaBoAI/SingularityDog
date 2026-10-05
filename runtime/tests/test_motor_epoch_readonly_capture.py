import contextlib
import hashlib
import io
import json
import os
import stat
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import motor_epoch_readonly_capture as capture
from singularitydog_hw import can_readonly as codec
from singularitydog_hw.motor_epoch_readonly_capture import (
    _check_quiet, _expected_uids,
)


UIDS = {str(mid): f"{mid:016x}" for mid in range(1, 13)}


class FakeCAN:
    opened = []
    fault = None

    def __init__(self, port, event_sink):
        self.bus = "front" if port.endswith("front") else "rear"
        self.sink, self.calls, self.poisoned, self.closed = event_sink, [], False, False
        self.parser = codec.ATParser()
        self.serial = self
        self.fd = None
        self.opened.append(self)

    def __enter__(self):
        self.fd = os.open("/dev/null", os.O_RDONLY)
        return self

    def __exit__(self, *_):
        os.close(self.fd)
        self.closed = self.poisoned = True
        if self.fault == "cleanup" and self.bus == "front":
            raise OSError("Synthetic CAN close failure")

    def fileno(self):
        return self.fd

    def query(self, mid, parameter=None):
        if self.poisoned:
            raise RuntimeError("Poisoned fake CAN")
        self.calls.append((mid, parameter))
        start = time.monotonic_ns()
        sequence = len(self.calls)
        tx = {"kind": "can_tx", "motor_id": mid, "parameter": parameter or "identity",
              "hex": codec.read_request(mid, parameter).hex(), "sequence": sequence,
              "monotonic_ns": start}
        if self.fault == "guard" and mid == 1:
            tx["parameter"] = "enable"
        try:
            self.sink(tx)
            if self.fault == "timeout" and mid == 2:
                self.sink({"kind": "can_timeout", "motor_id": mid, "parameter": "identity",
                           "monotonic_ns": start + 250_000_000})
                raise TimeoutError("No fresh response: ID2 identity")
            if parameter is None:
                value = "f"*16 if self.fault == "uid" and mid == 1 else UIDS[str(mid)]
                payload, can_id = bytes.fromhex(value), mid << 8 | 0xfe
            else:
                index, fmt, _ = codec.PARAMETERS[parameter]
                value = {"run_mode": 0, "current": 0., "voltage": 40., "position": mid/10}[parameter]
                payload = struct.pack("<H", index) + bytes(2) + struct.pack("<"+fmt, value)
                payload += bytes(8-len(payload))
                can_id = 17 << 24 | mid << 8 | 0xfd
            wire = b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + payload + b"\r\n"
            frame = self.parser.feed(wire)[0]
            self.sink({"kind": "can_rx_bytes", "hex": wire.hex(), "monotonic_ns": start+1})
            self.sink({"kind": "can_rx_frame", **frame.record(), "monotonic_ns": start+1})
            reply = codec.decode_reply(frame, mid, parameter)
            reply.update(kind="motor_parameter", sequence=sequence, request_monotonic_ns=start,
                         monotonic_ns=start+2, round_trip_ms=.000002)
            self.sink(reply)
            return reply
        except BaseException:
            self.poisoned = True
            raise


class FailedWrite:
    def __init__(self, stream, partial=False):
        self.stream, self.partial, self.calls = stream, partial, 0

    def write(self, payload):
        self.calls += 1
        if self.partial and self.calls == 1:
            return self.stream.write(payload[:9])
        raise OSError("Synthetic trace disk failure")

    def __getattr__(self, key):
        return getattr(self.stream, key)


class MotorEpochReadOnlyCaptureTest(unittest.TestCase):
    def setUp(self):
        FakeCAN.opened, FakeCAN.fault = [], None

    def _run(self, root, *, trace=True, execute=True, fault=None, trace_factory=None):
        root = Path(root).resolve()
        source, output, events = root/"uids.json", root/"capture.json", root/"events.jsonl"
        source.write_text(json.dumps(UIDS))
        FakeCAN.fault = fault
        binding = {"resolved": "/dev/null", "st_rdev": os.stat("/dev/null").st_rdev}
        bindings = {bus: {**binding, "path": "/dev/"+bus} for bus in ("front", "rear")}
        argv = ["--front-port", "/dev/front", "--rear-port", "/dev/rear", "--expected-uids", str(source),
                "--output", str(output)]
        if trace:
            argv += ["--trace-events", str(events)]
        if execute:
            argv.append("--execute-readonly")
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(capture, "_boot_id", return_value="synthetic-boot"))
            stack.enter_context(patch.object(capture.dual, "validate_ports", return_value=bindings))
            stack.enter_context(patch.object(capture.dual, "binding_matches", return_value=True))
            stack.enter_context(patch.object(capture.dual, "port_lock", return_value=contextlib.nullcontext()))
            stack.enter_context(patch.object(capture, "ownership_locks", return_value=contextlib.nullcontext()))
            stack.enter_context(patch.object(capture, "ReadOnlyCAN", FakeCAN))
            if trace_factory is not None:
                stack.enter_context(patch.object(capture, "_EventTrace", side_effect=trace_factory))
            stdout = stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            code = capture.main(argv)
        receipt = json.loads(stdout.getvalue()) if stdout.getvalue().strip() else None
        return code, output, events, receipt

    def assert_trace_bytes(self, result, path):
        raw = path.read_bytes()
        meta = result["trace_events"]
        self.assertEqual(meta["sha256"], hashlib.sha256(raw).hexdigest())
        self.assertEqual(meta["byte_count"], len(raw))
        self.assertEqual(meta["event_count"], raw.count(b"\n"))
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def assert_closed(self):
        self.assertEqual(len(FakeCAN.opened), 2)
        for can in FakeCAN.opened:
            self.assertTrue(can.closed and can.poisoned)
            with self.assertRaises(OSError):
                os.fstat(can.fd)

    def test_expected_uid_file_requires_unique_twelve_id_mapping(self):
        values = {str(mid): f"{mid:016x}" for mid in range(1, 13)}
        with TemporaryDirectory() as directory:
            path = Path(directory) / "uids.json"
            path.write_text(json.dumps(values))
            actual, digest = _expected_uids(path)
            self.assertEqual(actual[3], values["3"])
            self.assertEqual(len(digest), 64)
            path.write_text('{"1":"0000000000000001","1":"0000000000000001"}')
            with self.assertRaisesRegex(ValueError, "Duplicate key"):
                _expected_uids(path)

    def test_quiet_check_rejects_motion_and_current(self):
        rows = {str(mid): {"run_mode": 0, "current": 0.0,
                           "position_span_deg": 0.01} for mid in range(1, 13)}
        _check_quiet({"rows": rows})
        rows["7"]["current"] = 0.2
        with self.assertRaisesRegex(RuntimeError, "ID7"):
            _check_quiet({"rows": rows})
        rows["7"]["current"] = 0.0
        rows["7"]["position_span_deg"] = 0.11
        with self.assertRaisesRegex(RuntimeError, "ID7"):
            _check_quiet({"rows": rows})

    def test_optional_plan_creates_neither_trace_capture_nor_devices(self):
        with TemporaryDirectory() as root:
            code, output, events, plan = self._run(root, execute=False)
            self.assertEqual(code, 0)
            self.assertFalse(output.exists() or events.exists() or FakeCAN.opened)
            self.assertEqual(plan["allowed_can_types"], [0, 17])
            self.assertFalse(plan["automatic_retry"] or plan["motor_output_available"])
            self.assertEqual(plan["trace_events"]["max_events"], 4096)
            self.assertEqual(plan["trace_events"]["max_bytes"], 2*1024*1024)

    def test_default_report_and_queries_have_no_trace_side_effect(self):
        with TemporaryDirectory() as root, patch.object(capture, "_EventTrace", side_effect=AssertionError("Trace created")):
            code, output, events, _ = self._run(root, trace=False)
            result = json.loads(output.read_bytes())
            self.assertEqual(code, 0)
            self.assertNotIn("trace_events", result)
            self.assertNotIn("trace_events", result["plan"])
            self.assertFalse(events.exists())
            self.assertEqual(sum(len(can.calls) for can in FakeCAN.opened), 84)
            self.assert_closed()

    def test_complete_trace_replays_exact_raw_events_from_both_owners(self):
        with TemporaryDirectory() as root:
            code, output, events, _ = self._run(root)
            result = json.loads(output.read_bytes())
            self.assertEqual(code, 0)
            self.assert_trace_bytes(result, events)
            rows = [json.loads(row) for row in events.read_bytes().splitlines()]
            self.assertEqual(len(rows), 336)
            self.assertEqual({row["bus"] for row in rows}, {"front", "rear"})
            self.assertTrue(result["trace_events"]["complete"])
            for row in rows:
                if row["kind"] == "can_tx":
                    self.assertIn(row["motor_id"], capture.IDS_BY_BUS[row["bus"]])
                    self.assertEqual(row["hex"], codec.read_request(row["motor_id"], None if row["parameter"]=="identity" else row["parameter"]).hex())
            self.assertFalse(result["approved_for_runtime"] or result["motor_output_allowed"] or result["angle_wrap_applied"])
            self.assert_closed()

    def test_uid_timeout_keeps_tx_and_timeout_without_retry_or_type17(self):
        with TemporaryDirectory() as root:
            code, output, events, _ = self._run(root, fault="timeout")
            result = json.loads(output.read_bytes())
            self.assertEqual(code, 1)
            self.assert_trace_bytes(result, events)
            self.assertTrue(result["trace_events"]["complete"])
            self.assertIn("ID2 identity", result["errors"][0])
            rows = [json.loads(row) for row in events.read_bytes().splitlines()]
            tx = [r for r in rows if r["kind"]=="can_tx" and r["motor_id"]==2]
            timeout = [r for r in rows if r["kind"]=="can_timeout"]
            self.assertEqual(len(tx), 1); self.assertEqual(len(timeout), 1)
            self.assertEqual(timeout[0]["monotonic_ns"]-tx[0]["monotonic_ns"], 250_000_000)
            self.assertTrue(all(p is None for can in FakeCAN.opened for _, p in can.calls))
            self.assertIsNone(result["telemetry"])
            self.assertFalse(result["plan"]["automatic_retry"])
            self.assert_closed()

    def test_wrong_uid_and_rejected_guard_events_are_retained_and_abort(self):
        for fault, message in (("uid", "UID mismatch"), ("guard", "Unexpected CAN transmission")):
            with self.subTest(fault=fault), TemporaryDirectory() as root:
                FakeCAN.opened = []
                code, output, events, _ = self._run(root, fault=fault)
                result = json.loads(output.read_bytes())
                self.assertEqual(code, 1); self.assertIn(message, result["errors"][0])
                self.assert_trace_bytes(result, events)
                self.assertTrue(result["trace_events"]["complete"])
                rows = [json.loads(row) for row in events.read_bytes().splitlines()]
                self.assertTrue(any(r["kind"]=="can_tx" for r in rows))
                if fault == "guard":
                    self.assertTrue(any(r.get("parameter")=="enable" for r in rows))
                self.assert_closed()

    def test_event_and_byte_budget_failure_never_claims_success(self):
        for key, limit in (("MAX_TRACE_EVENTS", 1), ("MAX_TRACE_BYTES", 10)):
            with self.subTest(key=key), TemporaryDirectory() as root, patch.object(capture, key, limit):
                FakeCAN.opened = []
                code, output, events, _ = self._run(root)
                result = json.loads(output.read_bytes())
                self.assertEqual(code, 1); self.assertEqual(result["status"], "INCOMPLETE")
                self.assert_trace_bytes(result, events)
                self.assertFalse(result["trace_events"]["complete"])
                self.assertLessEqual(result["trace_events"]["event_count"], capture.MAX_TRACE_EVENTS)
                self.assertLessEqual(events.stat().st_size, capture.MAX_TRACE_BYTES)
                self.assert_closed()

    def test_partial_write_failure_is_hashed_exactly_and_preserves_guard_exception(self):
        real = capture._EventTrace
        for fault in (None, "guard"):
            with self.subTest(fault=fault), TemporaryDirectory() as root:
                FakeCAN.opened = []; traces=[]
                def factory(path):
                    trace=real(path); traces.append(trace); trace.stream=FailedWrite(trace.stream, partial=True); return trace
                code, output, events, _ = self._run(root, fault=fault, trace_factory=factory)
                result=json.loads(output.read_bytes())
                self.assertEqual(code, 1); self.assert_trace_bytes(result, events)
                self.assertEqual(events.stat().st_size, 9)
                self.assertEqual(result["trace_events"]["event_count"], 0)
                self.assertFalse(result["trace_events"]["complete"])
                self.assertTrue(traces[0].stream.closed)
                if fault=="guard":self.assertIn("Unexpected CAN transmission", result["errors"][0])
                self.assertTrue(any("Synthetic trace disk failure" in e for e in result["errors"]))
                self.assert_closed()

    def test_final_flush_failure_denies_capture_success_and_still_closes(self):
        fsync = os.fsync
        with TemporaryDirectory() as root:
            def fail_trace(fd):
                opened = os.fstat(fd)
                named = (Path(root).resolve()/"events.jsonl").stat()
                if (opened.st_dev, opened.st_ino) == (named.st_dev, named.st_ino):
                    raise OSError("Synthetic final flush failure")
                fsync(fd)
            with patch.object(capture.os, "fsync", side_effect=fail_trace):
                code, output, events, _ = self._run(root)
            result=json.loads(output.read_bytes())
            self.assertEqual(code, 1); self.assertEqual(result["status"], "INCOMPLETE")
            self.assert_trace_bytes(result, events)
            self.assertFalse(result["trace_events"]["complete"])
            self.assertIn("final flush failure", " ".join(result["errors"]))
            self.assert_closed()

    def test_summary_publish_failure_issues_no_success_receipt_despite_complete_json(self):
        fsync = os.fsync
        def fail_directory(fd):
            if stat.S_ISDIR(os.fstat(fd).st_mode):
                raise OSError("Synthetic summary directory sync failure")
            fsync(fd)
        with TemporaryDirectory() as root, patch.object(capture.os, "fsync", side_effect=fail_directory), \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            code, output, events, receipt = self._run(root)
            self.assertEqual(code, 1); self.assertIsNone(receipt)
            result = json.loads(output.read_bytes())
            self.assertEqual(result["status"], "RECORDED_REVIEW_REQUIRED")
            self.assert_trace_bytes(result, events)
            self.assertTrue(result["trace_events"]["complete"])
            self.assertIn("Private output could not be saved", errors.getvalue())
            self.assert_closed()

    def test_can_cleanup_failure_after_all_reads_cannot_report_success(self):
        for enabled in (False,True):
            with self.subTest(trace=enabled), TemporaryDirectory() as root:
                FakeCAN.opened=[]
                code,output,events,_=self._run(root,trace=enabled,fault="cleanup")
                result=json.loads(output.read_bytes())
                self.assertEqual(code,1);self.assertEqual(result["status"],"INCOMPLETE")
                self.assertIn("Synthetic CAN close failure",result["errors"][0])
                self.assertEqual(sum(len(can.calls) for can in FakeCAN.opened),84)
                self.assertFalse(result["approved_for_runtime"] or result["motor_output_allowed"])
                if enabled:
                    self.assert_trace_bytes(result,events)
                    self.assertTrue(result["trace_events"]["complete"])
                self.assert_closed()

    def test_initial_trace_open_failure_does_not_open_either_can(self):
        with TemporaryDirectory() as root:
            code, output, events, _ = self._run(root, trace_factory=lambda _: (_ for _ in ()).throw(OSError("Trace prepare failed")))
            result=json.loads(output.read_bytes())
            self.assertEqual(code, 1); self.assertFalse(FakeCAN.opened or events.exists())
            self.assertFalse(result["trace_events"]["complete"])
            self.assertIn("Trace prepare failed", result["errors"][0])

    def test_private_trace_paths_reject_existing_symlinks_git_and_output_alias(self):
        with TemporaryDirectory() as directory:
            root=Path(directory).resolve(); existing=root/"existing"; existing.write_text("retain")
            link=root/"link"; link.symlink_to(existing)
            linkdir=root/"linkdir"; linkdir.symlink_to(root, target_is_directory=True)
            git=root/"repository"; git.mkdir(); (git/".git").write_text("gitdir: private-worktree")
            for path in (existing,link,linkdir/"new",git/"events"):
                with self.subTest(path=path), self.assertRaises(ValueError):capture._trace_output_path(path)
            with patch.object(capture, "ReadOnlyCAN", side_effect=AssertionError("Opened CAN")), contextlib.redirect_stderr(io.StringIO()):
                source=root/"uids.json";source.write_text(json.dumps(UIDS))
                output=root/"shared.json"
                with self.assertRaises(SystemExit):capture.main(["--front-port","front","--rear-port","rear","--expected-uids",str(source),"--output",str(output),"--trace-events",str(output),"--execute-readonly"])
            self.assertEqual(existing.read_text(),"retain"); self.assertFalse(output.exists())

    def test_shared_stream_lock_and_exact_full_event_budget(self):
        with TemporaryDirectory() as directory:
            path=Path(directory).resolve()/"events.jsonl"; trace=capture._EventTrace(path)
            def worker(bus):
                for i in range(2048):trace.record(bus,{"kind":"can_rx_bytes","hex":"41","index":i})
            threads=[threading.Thread(target=worker,args=(bus,)) for bus in ("front","rear")]
            for thread in threads:thread.start()
            for thread in threads:thread.join()
            with self.assertRaisesRegex(RuntimeError,"budget"):trace.record("front",{"kind":"extra"})
            result=trace.close();raw=path.read_bytes();rows=[json.loads(row) for row in raw.splitlines()]
            self.assertEqual(len(rows),4096);self.assertEqual(result["event_count"],4096)
            self.assertEqual(result["attempted_event_count"],4097)
            self.assertEqual(result["sha256"],hashlib.sha256(raw).hexdigest())
            self.assertEqual({(row["bus"],row["index"]) for row in rows},{(bus,i) for bus in ("front","rear") for i in range(2048)})
            self.assertFalse(result["complete"]);self.assertTrue(trace.stream.closed)

    def test_atomic_create_cannot_overwrite_an_existing_trace(self):
        with TemporaryDirectory() as directory:
            path=Path(directory).resolve()/"events.jsonl"; trace=capture._EventTrace(path)
            trace.record("front",{"kind":"can_rx_bytes","hex":"41"})
            first=trace.close();original=path.read_bytes()
            with self.assertRaises(FileExistsError):capture._EventTrace(path)
            self.assertEqual(path.read_bytes(),original)
            self.assertTrue(first["complete"])
            self.assertEqual(first["sha256"],hashlib.sha256(original).hexdigest())

    def test_original_guard_exception_survives_simultaneous_trace_failure(self):
        with TemporaryDirectory() as directory:
            path=Path(directory).resolve()/"events.jsonl";trace=capture._EventTrace(path)
            trace.stream=FailedWrite(trace.stream)
            original=RuntimeError("Original guard refusal")
            with patch.object(capture,"guard_event",side_effect=original),self.assertRaises(RuntimeError) as raised:
                capture._trace_event(trace,"front",{"kind":"motor_feedback","type":21})
            self.assertIs(raised.exception,original)
            if hasattr(original,"add_note"):
                self.assertTrue(any("Synthetic trace disk failure" in note for note in original.__notes__))
            result=trace.close()
            self.assertFalse(result["complete"])
            self.assertEqual(result["event_count"],0)
            self.assertEqual(result["sha256"],hashlib.sha256(b"").hexdigest())
            self.assertTrue(trace.stream.closed)


if __name__ == "__main__":
    unittest.main()
