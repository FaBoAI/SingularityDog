"""Fixed-scope stop/version protocol tests; no hardware, network or firmware."""
import contextlib
from dataclasses import replace
import io
import json
from pathlib import Path
import signal
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import motor_version_probe as probe
from singularitydog_hw.can_readonly import ATParser

UIDS = {mid: f"{mid:016x}" for mid in range(1, 13)}


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def advance(self, ns): self.now += ns


def wire(kind=2, mid=1, dest=0xFD, *, version=False, mode=0, fault=0, flags=4):
    data = b"\x00\xc4\x56\x01\x23\x45\x67\xab" if version else struct.pack(">4H", 32768, 32768, 32768, 250)
    can_id = (kind << 24) | (mode << 22) | (fault << 16) | (mid << 8) | dest
    return b"AT"+((can_id << 3)|flags).to_bytes(4, "big")+b"\x08"+data+b"\r\n"


class Serial:
    def __init__(self, clock, failure=None):
        self.clock, self.failure = clock, failure
        self.timeout, self.write_timeout = .002, .02
        self.queue, self.buffer, self.writes, self.write_times = [], bytearray(), [], []
        self.closed = False
    def fill(self):
        while self.queue and self.queue[0][0] <= self.clock():
            _, data = self.queue.pop(0)
            self.buffer.extend(data)
    @property
    def in_waiting(self):
        self.fill()
        return len(self.buffer)
    def write(self, data):
        frame = ATParser().feed(data)[0]
        self.writes.append(frame)
        self.write_times.append(self.clock())
        mid = frame.destination
        if self.failure == "shortwrite": return len(data)-1
        if self.failure == "writeerror": raise OSError("fake write failure")
        if self.failure == "signal": signal.raise_signal(signal.SIGTERM)
        delay = 1_000_000
        if frame.kind == 0:
            if self.failure == "identity_timeout": return len(data)
            uid = "f"*16 if self.failure == "uid" else UIDS[mid]
            can_id = (mid << 8)|0xFE
            answer = b"AT"+((can_id << 3)|4).to_bytes(4, "big")+b"\x08"+bytes.fromhex(uid)+b"\r\n"
        elif frame.data == bytes(8):
            if self.failure == "stop_timeout": return len(data)
            answer = wire(mid=mid, version=self.failure == "stop_version_prefix",
                          mode=2 if self.failure == "mode" else 0,
                          fault=1 if self.failure == "fault" else 0)
        else:
            assert frame.data == probe.VERSION_PAYLOAD
            if self.failure == "version_timeout": return len(data)
            answer = wire(mid=6 if self.failure == "wrongid" else mid,
                          dest=0xFC if self.failure == "wrongdest" else 0xFD,
                          version=self.failure != "normal_only",
                          mode=2 if self.failure == "version_mode" else 0,
                          fault=1 if self.failure == "version_fault" else 0,
                          flags=0 if self.failure == "flags" else 4)
            if self.failure == "normal_then_version": answer = wire(mid=mid)+answer
            if self.failure == "version_then_fault": answer += wire(mid=mid, fault=1)
            if self.failure == "duplicate": answer += answer
            if self.failure == "partial_after": answer += b"A"
            if self.failure == "deadline": delay = probe.TIMEOUT_NS
        self.queue.append((self.clock()+delay, answer))
        return len(data)
    def read(self, count):
        self.fill()
        if not self.buffer:
            elapsed = int(self.timeout*1e9)
            if self.queue: elapsed = min(elapsed, max(0, self.queue[0][0]-self.clock()))
            self.clock.advance(elapsed)
            self.fill()
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result
    def close(self): self.closed = True


def fixture(failure=None):
    clock = Clock()
    port = Serial(clock, failure)
    events = []
    instance = probe.VersionProbe(port, events.append, clock=clock)
    return instance, port, events, clock


class VersionTests(unittest.TestCase):
    def test_exact_query_wire_and_no_semantic_firmware_guess(self):
        self.assertEqual(probe.version_request(1).hex(), "41542007e80c0800c40000000000000d0a")
        result = probe.decode_version(ATParser().feed(wire(version=True))[0], 1)
        self.assertEqual(result["version_bytes_hex"], "01234567")
        self.assertEqual(result["version_bytes"], [1, 35, 69, 103])
        self.assertEqual(result["unspecified_byte7"], 0xAB)
        self.assertIsNone(result["semantic_firmware_version"])
        for mid in (True, 0, -1, 13, "1"):
            with self.assertRaises(ValueError): probe.version_request(mid)
        self.assertEqual(probe.IDS, tuple(range(1, 13)))
        self.assertEqual(probe.make_plan()["max_write_attempts"], 36)
        self.assertEqual(probe.make_plan()["max_version_queries"], 12)
        self.assertEqual(ATParser().feed(probe.version_request(12))[0].destination, 12)
        with self.assertRaises(ValueError): probe.validate_uids({mid: UIDS[mid] for mid in (1, 6, 9, 10)})

    def test_strict_version_decoder_rejects_feedback_id_destination_flags_and_wire(self):
        frame = ATParser().feed(wire(version=True))[0]
        invalid = [ATParser().feed(wire())[0], ATParser().feed(wire(mid=6, version=True))[0],
                   ATParser().feed(wire(dest=0xFC, version=True))[0],
                   replace(frame, flags=0), replace(frame, data=frame.data[:-1]),
                   replace(frame, wire=b"bad"), ATParser().feed(wire(version=True, mode=2))[0],
                   ATParser().feed(wire(version=True, fault=1))[0]]
        for item in invalid:
            with self.subTest(item=item), self.assertRaises(ValueError): probe.decode_version(item, 1)

    def test_fixed_twelve_identity_stop_version_sequences_and_minimum_gap(self):
        instance, port, events, _ = fixture()
        result = instance.collect(UIDS)
        self.assertEqual(result["status"], "VERSION_PROBE_COMPLETE")
        self.assertEqual(len(port.writes), 36)
        self.assertEqual([f.kind for f in port.writes], [0, 4, 4]*12)
        self.assertEqual([f.destination for f in port.writes], [mid for mid in probe.IDS for _ in range(3)])
        for i, frame in enumerate(port.writes):
            self.assertEqual(frame.data, probe.VERSION_PAYLOAD if i % 3 == 2 else bytes(8))
        self.assertTrue(all(b-a >= probe.GAP_NS for a, b in zip(port.write_times, port.write_times[1:])))
        self.assertTrue(all(r["identity_verified"] and r["stop_confirmed"] for r in result["motors"]))
        self.assertFalse(result["pure_read_only"])
        self.assertFalse(result["motor_enabling_available"])
        self.assertEqual(len([e for e in events if e["kind"] == "version_tx_write"]), 36)
        with self.assertRaisesRegex(ValueError, "reused"): instance.collect(UIDS)

    def test_normal_feedback_never_counts_as_version_and_timeout_aborts_later_ids(self):
        for failure in ("normal_only", "version_timeout", "wrongid", "wrongdest", "deadline"):
            instance, port, events, _ = fixture(failure)
            result = instance.collect(UIDS)
            self.assertEqual(result["status"], "UNKNOWN", failure)
            self.assertEqual(len(port.writes), 3)
            self.assertEqual(result["motors"][0]["status"], "UNKNOWN")
            self.assertIsNone(result["motors"][0]["version"])
            self.assertTrue(result["motors"][0]["stop_confirmed"])
            self.assertTrue(all(r["status"] == "NOT_ATTEMPTED" for r in result["motors"][1:]))
            self.assertTrue(instance.poisoned)
        instance, port, _, _ = fixture("normal_then_version")
        self.assertEqual(instance.collect(UIDS)["status"], "VERSION_PROBE_COMPLETE")

    def test_version_prefix_cannot_acknowledge_ordinary_stop(self):
        instance, port, _, _ = fixture("stop_version_prefix")
        result = instance.collect(UIDS)
        self.assertEqual(result["status"], "ABORTED")
        self.assertFalse(result["motors"][0]["stop_confirmed"])
        self.assertEqual(len(port.writes), 2)

    def test_rejected_fault_running_mode_short_write_and_uid_do_not_continue(self):
        for failure, expected_writes in (("uid", 1), ("shortwrite", 1), ("writeerror", 1),
            ("identity_timeout", 1), ("stop_timeout", 2), ("mode", 2), ("fault", 2),
            ("flags", 3), ("version_mode", 3), ("version_fault", 3), ("version_then_fault", 3),
            ("duplicate", 3), ("partial_after", 3)):
            with self.subTest(failure=failure):
                instance, port, _, _ = fixture(failure)
                result = instance.collect(UIDS)
                self.assertEqual(result["status"], "ABORTED")
                self.assertEqual(len(port.writes), expected_writes)
                self.assertTrue(instance.poisoned)

    def test_stale_frames_are_discarded_and_partial_or_noise_boundary_aborts(self):
        instance, port, events, _ = fixture("identity_timeout")
        mid = 1
        can_id = (mid << 8)|0xFE
        port.buffer.extend(b"AT"+((can_id << 3)|4).to_bytes(4,"big")+b"\x08"+bytes.fromhex(UIDS[mid])+b"\r\n")
        result = instance.collect(UIDS)
        self.assertEqual(result["status"], "ABORTED")
        self.assertFalse(result["motors"][0]["identity_verified"])
        self.assertTrue(any(e.get("phase") == "pre_send_discard" for e in events))
        for prefix in (b"A", b"noise"):
            instance, port, _, clock = fixture()
            port.buffer.extend(prefix)
            result = instance.collect(UIDS)
            self.assertEqual(result["status"], "ABORTED")
            self.assertEqual(port.writes, [])
            self.assertLessEqual(clock.now-instance.started_ns, probe.DRAIN_NS+2_000_000)

    def test_default_plan_does_not_open_and_existing_git_outputs_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            uids = root/"uids.json"
            uids.write_text(json.dumps(UIDS))
            output = root/"output"
            argv = ["--expected-uids", str(uids), "--output", str(output)]
            with patch.object(probe, "open_serial") as opened, contextlib.redirect_stdout(io.StringIO()) as text:
                self.assertEqual(probe.main(argv), 0)
            opened.assert_not_called()
            self.assertFalse(output.exists())
            self.assertFalse(json.loads(text.getvalue())["pure_read_only"])
            output.mkdir()
            for marker in (None, ".git"):
                if marker:
                    output.rmdir(); (root/marker).write_text("gitdir: test")
                with patch.object(probe, "open_serial") as opened, contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    probe.main(argv+["--execute"])
                opened.assert_not_called()

    def test_cli_signal_and_failure_persist_private_summary_and_raw_events(self):
        for failure in (None, "shortwrite", "signal"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                uids = root/"uids.json"; uids.write_text(json.dumps(UIDS))
                output = root/"output"
                clock = Clock(); port = Serial(clock, failure)
                original = probe.VersionProbe
                def create(serial, emit, **kwargs):
                    return original(serial, emit, clock=clock, **kwargs)
                with patch.object(probe, "open_serial", return_value=port), \
                     patch.object(probe.timing, "ownership_locks", side_effect=contextlib.nullcontext), \
                     patch.object(probe, "VersionProbe", side_effect=create), \
                     patch.object(probe.os, "fsync", wraps=probe.os.fsync) as synced, \
                     contextlib.redirect_stdout(io.StringIO()):
                    code = probe.main(["--execute", "--expected-uids", str(uids), "--output", str(output)])
                self.assertEqual(code, 0 if failure is None else 2)
                self.assertTrue(port.closed)
                summary = json.loads((output/"summary.json").read_text())
                self.assertEqual(output.stat().st_mode & 0o777, 0o700)
                for name in ("events.jsonl", "summary.json"):
                    self.assertEqual((output/name).stat().st_mode & 0o777, 0o600)
                self.assertGreater(synced.call_count, 3)
                self.assertEqual(set(summary["source_sha256"]), {"motor_version_probe.py", "can_readonly.py",
                                                                    "rs05_trial_protocol.py", "can_timing_probe.py"})
                if failure == "signal":
                    self.assertIn(signal.SIGTERM, summary["signals"])
                    self.assertEqual(len(port.writes), 1)


if __name__ == "__main__":
    unittest.main()
