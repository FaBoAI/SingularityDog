"""Offline STOP-batch transport, protocol, timing, and capacity fault tests."""
import dataclasses
import unittest
from unittest.mock import patch

from singularitydog_hw import stop_batch_probe as probe
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.serial_deadline_reader import ReceivedChunk


MS = 1_000_000


def feedback(mid, *, kind=2, mode=0, fault=0, flags=4, destination=0xFD, data=bytes(8)):
    can_id = kind << 24 | mode << 22 | fault << 16 | mid << 8 | destination
    return b"AT" + ((can_id << 3) | flags).to_bytes(4, "big") + bytes([len(data)]) + data + b"\r\n"


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now


class Port:
    def __init__(self, clock, *, failure=None, reply=None, write_ns=50_000):
        self.clock, self.failure, self.reply, self.write_ns = clock, failure, reply, write_ns
        self.pending = []
        self.writes = []
        self.write_timeout = None

    def write(self, data):
        parsed = ATParser().feed(data)
        assert parsed and len(data) == len(parsed) * 17
        assert all(frame.kind == 4 and frame.source == 0xFD and frame.flags == 4
                   and frame.data == bytes(8) for frame in parsed)
        self.writes.append({"bytes": data, "started": self.clock(), "timeout": self.write_timeout})
        self.clock.now += self.write_ns
        self.writes[-1]["finished"] = self.clock()
        ids = tuple(frame.destination for frame in parsed)
        answer = b"".join(feedback(mid) for mid in ids)
        if self.reply is not None:
            answer = self.reply(ids)
        if self.failure == "exception":
            raise OSError("uncertain write")
        if self.failure == "partial":
            return len(data) - 1
        if self.failure == "boolean_write_result":
            return True
        if self.failure == "missing":
            return len(data)
        if self.failure == "late":
            self.pending.append((self.writes[-1]["started"] + probe.GROUP_TIMEOUT_NS, answer))
        elif self.failure == "fragmented":
            self.pending.extend(((self.clock() + MS, answer[:5]),
                                 (self.clock() + 2 * MS, answer[5:20]),
                                 (self.clock() + 3 * MS, answer[20:])))
        else:
            self.pending.append((self.clock() + MS, answer))
        if self.failure == "quiet_duplicate":
            self.pending.append((self.clock() + 50 * MS, feedback(ids[-1])))
        return len(data)


class Reader:
    def __init__(self, raw, *, clock, check):
        self.raw, self.clock, self.check = raw, clock, check

    def read_until(self, wake, hard):
        self.check()
        due = min((at for at, _ in self.raw.pending), default=wake)
        self.clock.now = max(self.clock(), min(due, wake))
        self.check()
        chunk = b"".join(data for at, data in self.raw.pending if at <= self.clock())
        self.raw.pending = [(at, data) for at, data in self.raw.pending if at > self.clock()]
        return chunk, self.clock()


class StopBatchTests(unittest.TestCase):
    def test_reader_deadline_evidence_cannot_complete_group_or_cause_next_write(self):
        for timestamp_known in (True, False):
            with self.subTest(timestamp_known=timestamp_known):
                evidence = []
                class RejectedReader(Reader):
                    def read_until(self, wake, hard):
                        chunk, stamp = super().read_until(wake, hard)
                        if chunk:
                            item = ReceivedChunk(chunk, stamp, stamp if timestamp_known else None)
                            evidence.append(item)
                            error = TimeoutError("post-read deadline")
                            error.serial_read_evidence = item
                            raise error
                        return chunk, stamp
                instance, port, _ = self.build(reader_factory=RejectedReader)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("post-read deadline", result["failure"])
                self.assertEqual(len(port.writes), 1)
                self.assertEqual(result["groups_observed"], 0)
                self.assertEqual(instance.raw_bytes, len(evidence[0].data))
                self.assertEqual(result["rejected_receive_chunks"], 1)
                if timestamp_known:
                    self.assertEqual(len(instance.raw_log), 1)
                    self.assertEqual(instance.raw_log[0][1:], (evidence[0].received_ns, evidence[0].data))
                else:
                    self.assertEqual(instance.raw_log, [])
                    self.assertEqual(result["unclocked_receive_evidence"], [evidence[0].record()])

    def build(self, ids=(1, 2, 3, 4, 5, 6), group_size=1, *, failure=None,
              reply=None, write_ns=50_000, check=lambda: None, reader_factory=Reader):
        clock = Clock()
        port = Port(clock, failure=failure, reply=reply, write_ns=write_ns)
        instance = probe.StopBatchProbe(port, ids, group_size=group_size, clock=clock,
                                       check=check, reader_factory=reader_factory)
        return instance, port, clock

    def test_group_sizes_use_exactly_one_canonical_write_each_and_shared_host_times(self):
        for size in (1, 2, 3, 6):
            with self.subTest(size=size):
                instance, port, _ = self.build(group_size=size)
                result = instance.run()
                self.assertEqual(result["status"], "STOP_BATCH_OBSERVATION_COMPLETE")
                self.assertEqual(len(port.writes), 6 // size)
                self.assertEqual(result["groups_observed"], 6 // size)
                self.assertTrue(result["final_quiet_observed"])
                self.assertGreaterEqual(result["quiet_finished_ns"] - result["quiet_started_ns"], 100 * MS)
                self.assertEqual(result["residual_hex"], "")
                self.assertTrue(result["further_tx_prohibited"])
                observed = []
                for row, write in zip(result["groups"], port.writes):
                    self.assertEqual(len(write["bytes"]), 17 * size)
                    self.assertEqual(row["returned_bytes"], 17 * size)
                    self.assertEqual(row["write_started_ns"], write["started"])
                    self.assertEqual(row["write_finished_ns"], write["finished"])
                    self.assertEqual(row["deadline_ns"], write["started"] + 250 * MS)
                    self.assertEqual(row["ids"], sorted(int(mid) for mid in row["feedback_by_id"]))
                    self.assertLessEqual(write["timeout"], .1)
                    for sample in row["feedback_by_id"].values():
                        observed.append(sample["motor_id"])
                        self.assertEqual(sample["received_ns"], write["finished"] + MS)
                        self.assertGreaterEqual(sample["first_seen_ns"], write["finished"])
                        self.assertNotIn("write_started_ns", sample)
                        self.assertNotIn("velocity_rad_s", sample)
                self.assertEqual(sorted(observed), list(range(1, 7)))
                self.assertFalse(result["per_frame_physical_send_timestamps_available"])
                self.assertFalse(result["physical_stop_time_verified"])

    def test_noncontiguous_rear_ids_and_final_smaller_group_are_explicit(self):
        instance, port, _ = self.build(ids=(7, 9, 12), group_size=2)
        result = instance.run()
        self.assertEqual(result["status"], "STOP_BATCH_OBSERVATION_COMPLETE")
        self.assertEqual(result["groups_planned"], [[7, 9], [12]])
        self.assertEqual([len(row["bytes"]) for row in port.writes], [34, 17])

    def test_invalid_selection_or_codec_is_rejected_before_reader_or_io(self):
        for ids in ((), (True,), (1.0,), (0,), (13,), (2, 1), (1, 1), (1, 7), [1]):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                self.build(ids=ids)
        for size in (True, 0, 4, 7, 2.0, "2"):
            with self.subTest(size=size), self.assertRaises(ValueError):
                self.build(group_size=size)
        with patch.object(probe, "stop_request", return_value=feedback(1)), \
                self.assertRaisesRegex(ValueError, "canonical all-zero"):
            self.build()

    def test_group_plan_is_frozen_and_revalidated_at_write_boundary(self):
        instance, port, _ = self.build()
        with self.assertRaises(dataclasses.FrozenInstanceError):
            instance._groups[0].wire = feedback(1)
        instance._groups = (dataclasses.replace(instance._groups[0], wire=feedback(1)),) + instance._groups[1:]
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("exact prevalidated STOP", result["failure"])
        self.assertEqual(port.writes, [])

    def test_fragmented_and_coalesced_feedback_retains_each_first_read(self):
        instance, port, _ = self.build(ids=(1, 2), group_size=2, failure="fragmented")
        result = instance.run()
        self.assertEqual(result["status"], "STOP_BATCH_OBSERVATION_COMPLETE")
        rows = result["groups"][0]["feedback_by_id"]
        self.assertEqual(len(instance.raw_log), 3)
        self.assertEqual(rows["1"]["first_seen_ns"], instance.raw_log[0][0])
        self.assertEqual(rows["1"]["read_started_ns"], instance.raw_log[1][0])
        self.assertEqual(rows["1"]["received_ns"], instance.raw_log[1][1])
        self.assertEqual(rows["2"]["first_seen_ns"], instance.raw_log[1][0])
        self.assertEqual(rows["2"]["received_ns"], instance.raw_log[2][1])
        self.assertEqual(b"".join(chunk for _, _, chunk in instance.raw_log), feedback(1) + feedback(2))

    def test_partial_exception_or_invalid_write_result_permanently_prohibits_tx(self):
        for failure in ("partial", "exception", "boolean_write_result"):
            with self.subTest(failure=failure):
                instance, port, _ = self.build(group_size=2, failure=failure)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertTrue(result["transport_poisoned"])
                self.assertTrue(instance.tx_prohibited)
                self.assertEqual(len(port.writes), 1)
                with self.assertRaises(RuntimeError):
                    instance.run()
                with self.assertRaises(ValueError):
                    instance._write_next_group()
                self.assertEqual(len(port.writes), 1)
                self.assertIs(instance.report, result)

    def test_successful_pass_also_cannot_repeat(self):
        instance, port, _ = self.build(ids=(1,))
        result = instance.run()
        self.assertEqual(result["status"], "STOP_BATCH_OBSERVATION_COMPLETE")
        with self.assertRaises(RuntimeError):
            instance.run()
        with self.assertRaises(ValueError):
            instance._write_next_group()
        self.assertEqual(len(port.writes), 1)

    def test_missing_late_and_slow_full_write_fail_without_next_group(self):
        for failure, write_ns in (("missing", 50_000), ("late", 50_000), (None, 250 * MS)):
            with self.subTest(failure=failure, write_ns=write_ns):
                instance, port, clock = self.build(failure=failure, write_ns=write_ns)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertFalse(result["transport_poisoned"])
                self.assertEqual(len(port.writes), 1)
                self.assertLessEqual(clock() - result["started_ns"], 250 * MS)
                self.assertFalse(result["final_quiet_observed"])

    def test_wrong_id_duplicate_version_mode_fault_and_malformed_frames_fail(self):
        replies = {
            "other_group": feedback(3),
            "unknown_id": feedback(12),
            "duplicate": feedback(1) * 2 + feedback(2),
            "version": feedback(1, data=b"\x00\xc4\x56" + bytes(5)),
            "mode": feedback(1, mode=2),
            "fault": feedback(1, fault=1),
            "destination": feedback(1, destination=0xFE),
            "kind": feedback(1, kind=24),
            "flags": feedback(1, flags=0),
            "dlc": feedback(1, data=bytes(7)),
            "noise": b"noise" + feedback(1),
            "terminator": feedback(1)[:-2] + b"xx",
        }
        for name, answer in replies.items():
            with self.subTest(name=name):
                instance, port, _ = self.build(group_size=2, reply=lambda ids: answer)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertEqual(len(port.writes), 1)
                self.assertFalse(result["final_quiet_observed"])
                self.assertTrue(instance.tx_prohibited)

    def test_complete_group_plus_partial_tail_cannot_advance_group(self):
        instance, port, _ = self.build(group_size=2,
            reply=lambda ids: b"".join(feedback(mid) for mid in ids) + b"A")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(result["residual_hex"], "41")
        self.assertEqual(len(result["groups"][0]["feedback_by_id"]), 2)
        self.assertEqual(len(port.writes), 1)

    def test_preexisting_bytes_after_external_preflight_block_first_write(self):
        instance, port, clock = self.build()
        port.pending.append((clock(), feedback(1)))
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("group boundary", result["failure"])
        self.assertEqual(port.writes, [])
        self.assertEqual(instance.raw_log[0][2], feedback(1))

    def test_delayed_duplicate_in_final_quiet_is_failure_not_silently_drained(self):
        instance, port, _ = self.build(ids=(1,), failure="quiet_duplicate")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("final quiet", result["failure"])
        self.assertFalse(result["final_quiet_observed"])
        self.assertEqual(len(port.writes), 1)

    def test_reader_setup_failure_and_cancellation_do_not_transmit(self):
        def broken_reader(*args, **kwargs):
            raise OSError("reader setup failed")
        instance, port, _ = self.build(reader_factory=broken_reader)
        self.assertIn("reader setup", instance.run()["failure"])
        self.assertEqual(port.writes, [])
        def cancelled():
            raise InterruptedError("cancelled")
        instance, port, _ = self.build(check=cancelled)
        self.assertIn("cancelled", instance.run()["failure"])
        self.assertEqual(port.writes, [])

    def test_invalid_read_timestamp_and_backwards_clock_fail_closed(self):
        class InvalidReader(Reader):
            def read_until(self, wake, hard):
                chunk, received = super().read_until(wake, hard)
                return chunk, received + 1
        instance, port, _ = self.build(reader_factory=InvalidReader)
        self.assertIn("host read interval", instance.run()["failure"])
        self.assertEqual(port.writes, [])
        class BackwardsReader(Reader):
            def read_until(self, wake, hard):
                self.clock.now -= 1
                return b"", self.clock()
        instance, port, _ = self.build(reader_factory=BackwardsReader)
        self.assertIn("clock moved backward", instance.run()["failure"])
        self.assertEqual(port.writes, [])

    def test_clock_advancing_inside_reader_check_does_not_reject_earlier_receive_stamp(self):
        class CallbackReader(Reader):
            def read_until(self, wake, hard):
                chunk, received = super().read_until(wake, hard)
                self.clock.now += 1
                self.check()
                return chunk, received
        instance, _, _ = self.build(reader_factory=CallbackReader)
        self.assertEqual(instance.run()["status"], "STOP_BATCH_OBSERVATION_COMPLETE")

    def test_overall_deadline_and_raw_log_limits_are_finite(self):
        class SlowSetupReader(Reader):
            def __init__(self, raw, *, clock, check):
                super().__init__(raw, clock=clock, check=check)
                self.clock.now += probe.MAX_RUN_NS
        instance, port, _ = self.build(reader_factory=SlowSetupReader)
        self.assertIn("overall deadline", instance.run()["failure"])
        self.assertEqual(port.writes, [])
        instance, port, _ = self.build(group_size=6)
        with patch.object(probe, "MAX_RAW_BYTES", 50):
            result = instance.run()
        self.assertTrue(result["raw_log_overflow"])
        self.assertEqual(len(instance.raw_log), 0)
        self.assertEqual(result["unlogged_chunk_bytes"], 102)
        self.assertEqual(len(port.writes), 1)
        class StuckReader(Reader):
            def read_until(self, wake, hard):
                return b"", self.clock()
        instance, port, _ = self.build(reader_factory=StuckReader)
        with patch.object(probe, "MAX_READ_CALLS", 5):
            result = instance.run()
        self.assertIn("receive-call budget", result["failure"])
        self.assertEqual(len(port.writes), 1)


if __name__ == "__main__":
    unittest.main()
