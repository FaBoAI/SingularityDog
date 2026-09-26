"""Explicit START-only retry qualification with synthetic bytes and virtual time."""
from collections import Counter
import unittest
from unittest.mock import patch

from singularitydog_hw import rs05_firmware_transfer as ota
from singularitydog_hw.can_readonly import HOST_ID
from test_rs05_firmware_transfer import Clock, FakeIO, IMAGE, MS, UID, decode_request, reply


class StartRetryTests(unittest.TestCase):
    def build(self, *, attempts=3, omit_attempts=False, answers=None,
              write_failure=None, read_failure=None, start_timeout=None):
        clock = Clock()
        port = FakeIO(clock, answers=answers, write_failure=write_failure,
                      read_failure=read_failure)
        options = {} if omit_attempts else {"start_attempts": attempts}
        if start_timeout is not None:
            options["start_ack_timeout_s"] = start_timeout
        with patch.object(ota, "verify_image", return_value=IMAGE):
            engine = ota.FirmwareTransfer(port, IMAGE, 1, UID, clock=clock,
                read_ready=port.ready, read_bytes=port.read, **options)
        return engine, port, clock

    def assert_stopped(self, engine, port, starts, total=None):
        total = starts if total is None else total
        self.assertEqual(engine.report["status"], "INCOMPLETE")
        self.assertEqual(len(port.writes), total)
        self.assertEqual(engine.report["transmit_attempts"], total)
        self.assertEqual(engine.report["start_transmit_attempts"], starts)
        self.assertEqual(sum(decode_request(w)[0] == 11 for w, _ in port.writes), starts)
        self.assertFalse(engine.report["end_acknowledged"])
        self.assertFalse(engine.report["data_retry_available"])
        self.assertFalse(engine.report["cleanup_transmission_available"])
        self.assertIn("failure", engine.report)

    def test_default_and_explicit_one_preserve_one_silent_start(self):
        for omitted in (False, True):
            with self.subTest(omitted=omitted):
                engine, port, _ = self.build(attempts=1, omit_attempts=omitted,
                                             answers=lambda *_: [])
                engine.run()
                self.assert_stopped(engine, port, 1)
                self.assertFalse(engine.report["automatic_retry"])
                self.assertEqual(engine.report["start_attempt_limit"], 1)

    def test_only_strict_integer_one_or_three_are_accepted(self):
        for attempts in (0, 2, 4, -1, True, False, 1.0, 3.0, "3", None):
            with self.subTest(attempts=attempts):
                port = FakeIO(Clock())
                with patch.object(ota, "verify_image", return_value=IMAGE):
                    with self.assertRaisesRegex(ValueError, "START attempts"):
                        ota.FirmwareTransfer(port, IMAGE, 1, UID, start_attempts=attempts)
                self.assertEqual(port.writes, [])

    def test_three_silent_attempts_use_identical_start_and_bounded_deadlines(self):
        engine, port, _ = self.build(answers=lambda *_: [])
        engine.run()
        self.assert_stopped(engine, port, 3)
        self.assertTrue(engine.report["automatic_retry"])
        self.assertEqual(engine.report["start_attempt_limit"], 3)
        self.assertEqual(engine.report["raw_bytes"], 0)
        self.assertEqual(len({wire for wire, _ in port.writes}), 1)
        self.assertEqual(decode_request(port.writes[0][0]), (11, HOST_ID, 1, UID))
        self.assertEqual([row["attempt"] for row in engine.tx_log], [1, 2, 3])
        self.assertEqual([row["ordinal"] for row in engine.tx_log], [0, 0, 0])
        for previous, current in zip(engine.tx_log, engine.tx_log[1:]):
            interval = current["write_started_ns"] - previous["write_started_ns"]
            self.assertGreaterEqual(interval, 2_000 * MS + 50 * MS)
            self.assertLess(interval, 2_000 * MS + 60 * MS)
            self.assertIn("retry_reason", previous)
        self.assertTrue(all(row["returned_bytes"] == 17 and row["ack_timeout"]
                            and row["ack_received_ns"] is None for row in engine.tx_log))
        self.assertGreaterEqual(engine.report["elapsed_s"], 6.2)
        self.assertLess(engine.report["elapsed_s"], 6.3)

    def test_ack_on_third_start_completes_exact_image_with_no_data_retries(self):
        def answers(n, kind, mid):
            return [] if n < 2 else [(100_000, reply(kind, mid=mid))]
        engine, port, _ = self.build(answers=answers)
        engine.run()
        decoded = [decode_request(wire) for wire, _ in port.writes]
        self.assertEqual(engine.report["status"], "TRANSFER_ACK_COMPLETE_PENDING_VERSION")
        self.assertEqual(Counter(row[0] for row in decoded), {11: 3, 12: 1, 13: 11698, 14: 1})
        self.assertEqual(engine.report["start_transmit_attempts"], 3)
        self.assertEqual(engine.report["acknowledged_data_packets"], 11698)
        self.assertTrue(engine.report["end_acknowledged"])
        self.assertFalse(engine.report["version_verified"])
        self.assertFalse(engine.report["data_retry_available"])
        data = [row for row in decoded if row[0] == 13]
        self.assertEqual([row[1] for row in data], list(range(11698)))
        self.assertEqual(b"".join(row[3] for row in data), IMAGE)
        self.assertTrue(all(row[2] == 1 for row in decoded))
        self.assertTrue(all(row["attempt"] == 1 for row in engine.tx_log[3:]))
        self.assertIsNone(engine.tx_log[0]["ack_received_ns"])
        self.assertIsNone(engine.tx_log[1]["ack_received_ns"])
        self.assertGreaterEqual(engine.tx_log[3]["write_started_ns"] -
                                engine.tx_log[2]["ack_received_ns"], 50 * MS)

    def test_retry_mode_requires_fifty_ms_start_to_info_gap_even_first_ack(self):
        for attempts in (1, 3):
            with self.subTest(attempts=attempts):
                engine, port, _ = self.build(attempts=attempts, answers=lambda n, k, m:
                    [(100_000, reply(k, mid=m))] if n == 0 else [])
                engine.run()
                self.assert_stopped(engine, port, 1, 2)
                gap = engine.tx_log[1]["write_started_ns"] - engine.tx_log[0]["ack_received_ns"]
                self.assertGreaterEqual(gap, (50 if attempts == 3 else 1) * MS)
                self.assertLess(gap, (60 if attempts == 3 else 10) * MS)

    def test_any_partial_malformed_wrong_or_failed_start_rx_forbids_retry(self):
        received = [b"A", reply(11)[:9], b"noise", reply(11)[:-2] + b"xx",
                    reply(12), reply(11, mid=2), reply(11, host=0xFE),
                    reply(11, flags=0), reply(11, flags=5), reply(11, data=bytes(7)),
                    reply(11, status=0x0F), reply(11, status=0xF0),
                    reply(11) + reply(11), reply(11) + b"AT"]
        for chunk in received:
            with self.subTest(chunk=chunk.hex()):
                engine, port, _ = self.build(answers=lambda *_: [(100_000, chunk)])
                engine.run()
                self.assert_stopped(engine, port, 1)
                self.assertEqual(engine.report["raw_bytes"], len(chunk))
                self.assertEqual(bytes.fromhex(engine.raw_log[0]["hex"]), chunk)

    def test_late_ack_or_partial_during_retry_quiet_prevents_second_start(self):
        for delay in (2_000 * MS - 50_000, 2_025 * MS):
            for chunk in (reply(11), b"AT"):
                with self.subTest(delay=delay, chunk=chunk.hex()):
                    engine, port, _ = self.build(answers=lambda *_: [(delay, chunk)])
                    engine.run()
                    self.assert_stopped(engine, port, 1)
                    self.assertEqual(engine.report["raw_bytes"], len(chunk))
                    self.assertEqual(engine.raw_log[0]["hex"], chunk.hex())

    def test_uncertain_write_never_qualifies_as_silent_ack_timeout(self):
        for fail_at in (0, 1):
            for outcome in (0, 16, 18, True, 17.0, OSError("uncertain write")):
                with self.subTest(fail_at=fail_at, outcome=outcome):
                    engine, port, _ = self.build(answers=lambda *_: [],
                        write_failure=lambda n: outcome if n == fail_at else None)
                    engine.run()
                    self.assert_stopped(engine, port, fail_at + 1)
                    self.assertIsNotNone(engine.tx_log[-1]["write_finished_ns"])
                    self.assertIsNone(engine.tx_log[-1]["ack_received_ns"])

    def test_guard_timeout_cancel_or_binding_failure_is_not_retryable(self):
        for offset in (2_000 * MS, 2_020 * MS):
            for error in (TimeoutError("guard deadline"), ValueError("binding changed"),
                          KeyboardInterrupt("cancel")):
                with self.subTest(offset=offset, error=error):
                    engine, port, clock = self.build(answers=lambda *_: [])
                    def check():
                        if port.writes and clock() >= port.writes[0][1] + offset:
                            raise error
                    engine.check = check
                    engine.run()
                    self.assert_stopped(engine, port, 1)
                    self.assertIn(str(error), engine.report["failure"])

    def test_read_exception_after_full_start_write_is_not_retryable(self):
        def fail_read():
            raise OSError("read failed")
        engine, port, _ = self.build(read_failure=fail_read)
        engine.run()
        self.assert_stopped(engine, port, 1)
        self.assertIn("read failed", engine.report["failure"])

    def test_write_finishing_at_or_after_ack_deadline_is_not_retryable(self):
        for lateness in (0, MS):
            with self.subTest(lateness=lateness):
                engine, port, clock = self.build(answers=lambda *_: [])
                def slow_write(_):
                    clock.now = port.writes[-1][1] + 2_000 * MS + lateness
                    return 17
                port.write_failure = slow_write
                engine.run()
                self.assert_stopped(engine, port, 1)
                self.assertEqual(engine.tx_log[0]["returned_bytes"], 17)
                self.assertIsNone(engine.tx_log[0]["ack_received_ns"])

    def test_duplicate_after_second_start_ack_stops_before_info(self):
        for extra in (reply(11), b"AT"):
            with self.subTest(extra=extra.hex()):
                def answers(n, kind, mid):
                    if n == 0:
                        return []
                    return [(100_000, reply(kind, mid=mid)), (25 * MS, extra)]
                engine, port, _ = self.build(answers=answers)
                engine.run()
                self.assert_stopped(engine, port, 2)
                self.assertIsNotNone(engine.tx_log[1]["ack_received_ns"])
                self.assertIn("boundary", engine.report["failure"])
                self.assertEqual(engine.report["raw_bytes"], 17 + len(extra))

    def test_info_data_end_timeout_never_retries_with_retry_mode_enabled(self):
        for failed_phase in (12, 13, 14):
            with self.subTest(failed_phase=failed_phase):
                def answers(n, kind, mid):
                    if n == 0 or kind == failed_phase:
                        return []
                    return [(100_000, reply(kind, mid=mid))]
                engine, port, _ = self.build(answers=answers)
                engine.run()
                expected_writes = {12: 3, 13: 4, 14: 11702}[failed_phase]
                self.assert_stopped(engine, port, 2, expected_writes)
                kinds = [decode_request(wire)[0] for wire, _ in port.writes]
                self.assertEqual(kinds[-1], failed_phase)
                self.assertEqual(kinds.count(failed_phase), 1)
                self.assertEqual(engine.tx_log[-1]["attempt"], 1)
                self.assertIn("acknowledgement missing", engine.report["failure"])

    def test_stale_rx_before_first_start_sends_nothing_in_retry_mode(self):
        engine, port, clock = self.build()
        port.pending.append((clock() + MS, reply(11)))
        engine.run()
        self.assert_stopped(engine, port, 0)
        self.assertFalse(engine.report["bootloader_entry_attempted"])

    def test_start_timeout_accepts_only_strict_integer_two_or_three(self):
        for timeout in (0, 1, 4, -1, True, False, 2.0, 3.0, "3", None):
            with self.subTest(timeout=timeout):
                port = FakeIO(Clock())
                with patch.object(ota, "verify_image", return_value=IMAGE):
                    with self.assertRaises(ValueError):
                        ota.FirmwareTransfer(port, IMAGE, 1, UID,
                                             start_ack_timeout_s=timeout)
                self.assertEqual(port.writes, [])

    def test_silent_start_deadline_defaults_to_two_seconds_and_allows_three(self):
        for timeout in (None, 2, 3):
            with self.subTest(timeout=timeout):
                engine, port, clock = self.build(attempts=1, start_timeout=timeout,
                                                 answers=lambda *_: [])
                engine.run()
                self.assert_stopped(engine, port, 1)
                self.assertEqual(clock() - port.writes[0][1],
                                 (2 if timeout is None else timeout) * 1_000 * MS)
                self.assertEqual(engine.report["raw_bytes"], 0)
                self.assertEqual(engine.report["start_ack_timeout_seconds"],
                                 2 if timeout is None else timeout)
                self.assertEqual(engine.report["other_ack_timeout_seconds"], 2)

    def test_start_ack_at_2030_ms_succeeds_with_three_seconds_and_one_attempt(self):
        def answers(n, kind, mid):
            delay = 2_030 * MS - 50_000 if kind == 11 else 100_000
            return [(delay, reply(kind, mid=mid))]
        engine, port, _ = self.build(attempts=1, start_timeout=3, answers=answers)
        engine.run()
        self.assertEqual(engine.report["status"], "TRANSFER_ACK_COMPLETE_PENDING_VERSION")
        self.assertEqual(engine.report["start_transmit_attempts"], 1)
        self.assertFalse(engine.report["automatic_retry"])
        self.assertEqual(Counter(decode_request(wire)[0] for wire, _ in port.writes),
                         {11: 1, 12: 1, 13: 11698, 14: 1})
        self.assertEqual(engine.tx_log[0]["ack_received_ns"] -
                         engine.tx_log[0]["write_started_ns"], 2_030 * MS)
        self.assertEqual(engine.tx_log[0]["ack_timeout_ns"], 3_000 * MS)
        self.assertTrue(all(row["ack_timeout_ns"] == 2_000 * MS
                            for row in engine.tx_log[1:]))
        self.assertTrue(engine.report["end_acknowledged"])
        self.assertFalse(engine.report["version_verified"])

    def test_start_ack_at_2030_ms_does_not_advance_with_two_seconds(self):
        for timeout in (None, 2):
            for attempts in (1, 3):
                with self.subTest(timeout=timeout, attempts=attempts):
                    engine, port, _ = self.build(attempts=attempts, start_timeout=timeout,
                        answers=lambda n, k, m: [(2_030 * MS - 50_000, reply(k, mid=m))])
                    engine.run()
                    self.assert_stopped(engine, port, 1)
                    self.assertIsNone(engine.tx_log[0]["ack_received_ns"])
                    if attempts == 3:
                        self.assertIn("boundary", engine.report["failure"])
                        self.assertEqual(engine.report["raw_bytes"], 17)
                    else:
                        self.assertIn("acknowledgement missing", engine.report["failure"])
                        self.assertEqual(engine.report["raw_bytes"], 0)

    def test_start_ack_exactly_at_three_second_deadline_is_late_and_never_retried(self):
        for attempts in (1, 3):
            with self.subTest(attempts=attempts):
                engine, port, _ = self.build(attempts=attempts, start_timeout=3,
                    answers=lambda n, k, m: [(3_000 * MS - 50_000, reply(k, mid=m))])
                engine.run()
                self.assert_stopped(engine, port, 1)
                self.assertIn("Late OTA bytes", engine.report["failure"])
                self.assertEqual(engine.report["raw_bytes"], 17)
                self.assertEqual(engine.raw_log[0]["received_ns"] -
                                 engine.tx_log[0]["write_started_ns"], 3_000 * MS)
                self.assertIsNone(engine.tx_log[0]["ack_received_ns"])

    def test_three_second_start_does_not_extend_info_data_or_end_deadlines(self):
        for failed_phase in (12, 13, 14):
            for phase_delay in (2_000 * MS, 2_030 * MS):
                with self.subTest(phase=failed_phase, delay=phase_delay):
                    def answers(n, kind, mid):
                        delay = phase_delay - 50_000 if kind == failed_phase else 100_000
                        return [(delay, reply(kind, mid=mid))]
                    engine, port, clock = self.build(attempts=1, start_timeout=3,
                                                     answers=answers)
                    engine.run()
                    expected_writes = {12: 2, 13: 3, 14: 11701}[failed_phase]
                    self.assert_stopped(engine, port, 1, expected_writes)
                    self.assertEqual(engine.tx_log[-1]["kind"], failed_phase)
                    self.assertEqual(clock() - port.writes[-1][1], 2_000 * MS)
                    self.assertIsNone(engine.tx_log[-1]["ack_received_ns"])
                    self.assertIn("Late OTA bytes" if phase_delay == 2_000 * MS else
                                  "acknowledgement missing", engine.report["failure"])


if __name__ == "__main__":
    unittest.main()
