"""Pipeline scheduling/guard integration; real POSIX reader tests live separately."""
import unittest
from unittest.mock import patch

from singularitydog_hw.can_pipeline_probe import PipelineCAN
from singularitydog_hw.can_readonly import read_request
from test_can_pipeline_probe import Clock, Serial, UIDS, wire


class SimulatedDeadlineReader:
    """Deterministic arrival queue, independent of serial's relative timeout."""

    def __init__(self, raw_serial, *, clock, check):
        self.port, self.clock, self.check = raw_serial, clock, check
        self.calls = []
        raw_serial.timeout = 0

    def stats(self):
        return {"read_until_calls": len(self.calls)}

    def read_until(self, wake_ns, hard_ns):
        self.calls.append((self.clock(), wake_ns, hard_ns))
        self.check()
        self.port.fill()
        if not self.port.buffer:
            target = min(wake_ns, hard_ns)
            if self.port.queue:
                target = min(target, self.port.queue[0][0])
            self.clock.advance(max(0, target - self.clock()))
            self.check()
            self.port.fill()
        chunk = bytes(self.port.buffer[:4096])
        del self.port.buffer[:4096]
        return chunk, self.clock()


class PipelineSelectReceiveTests(unittest.TestCase):
    def setUp(self):
        self.reader_patch = patch(
            "singularitydog_hw.serial_deadline_reader.DeadlineSerialReader",
            side_effect=SimulatedDeadlineReader)
        self.reader_factory = self.reader_patch.start()
        self.addCleanup(self.reader_patch.stop)

    def fixture(self, *, receive_mode="select", gap_ms=.5, seconds=10., failure=None):
        clock = Clock()
        port = Serial(clock, failure=failure)
        events = []
        probe = PipelineCAN(events.append, serial_port=port, clock=clock,
                            window=4, gap_ms=gap_ms, cycles=1, max_seconds=seconds,
                            receive_mode=receive_mode)
        return probe, port, events, clock

    def test_absolute_next_send_wake_survives_receive_entry_cost(self):
        probe, port, _, clock = self.fixture()
        original_receive = probe._receive
        injected = []

        def costly_receive(timeout_s, expected_uids):
            if probe._receive_wake_ns is not None and not injected:
                injected.append(probe._receive_wake_ns)
                clock.advance(200_000)  # Guard work before _read_chunk starts.
            return original_receive(timeout_s, expected_uids)

        with probe, patch.object(probe, "_receive", side_effect=costly_receive):
            probe._batch([(1, "position"), (2, "position")],
                         window=4, cycle=1, expected_uids=UIDS)
        self.assertEqual(injected, [port.write_times[0] + 500_000])
        self.assertEqual(probe.deadline_reader.calls[0][1], injected[0])
        self.assertEqual(port.write_times[1] - port.write_times[0], 500_000)
        self.assertEqual(probe.replies, 2)
        self.assertIsNone(probe._receive_wake_ns)
        self.assertTrue(port.closed)

    def test_expired_soft_wake_still_drains_available_reply(self):
        probe, port, _, clock = self.fixture()
        with probe:
            probe._send((1, "position"), 1)
            clock.advance(port.delay_ns)
            received = clock()
            probe._receive_wake_ns = received - 100_000
            probe._receive(0., UIDS)
            self.assertEqual(clock(), received)
            self.assertFalse(probe.pending)
            self.assertEqual(probe.rows[0]["received_monotonic_ns"], received)
            self.assertLess(probe.deadline_reader.calls[0][1], received)
        self.assertEqual(probe.replies, 1)
        self.assertEqual(port.timeout_settings, [0])
        self.assertEqual(port.read_timeouts, [])

    def test_request_and_global_hard_deadlines_bound_reader(self):
        for limit in ("request", "global"):
            with self.subTest(limit=limit):
                probe, port, _, clock = self.fixture(seconds=1.)
                with probe:
                    if limit == "global":
                        clock.advance(999_000_000)
                    probe._send((1, "position"), 1)
                    deadline = probe.pending[(1, "position")]["deadline_monotonic_ns"]
                    if limit == "request":
                        clock.advance(249_000_000)
                    port.queue.clear()
                    with self.assertRaises(TimeoutError):
                        probe._receive(.003, UIDS)
                    entry, wake, hard = probe.deadline_reader.calls[0]
                    self.assertEqual(hard, deadline)
                    self.assertEqual(wake, deadline)
                    self.assertEqual(deadline - entry, 1_000_000)
                    self.assertEqual(clock(), deadline)
                    # A subsequent attempt fails before invoking the reader.
                    with self.assertRaises(TimeoutError):
                        probe._receive(.003, UIDS)
                    self.assertEqual(len(probe.deadline_reader.calls), 1)
                self.assertEqual(probe.replies, 0)
                self.assertEqual(len(port.writes), 1)
                self.assertTrue(port.closed)

    def test_pipeline_rejects_late_chunk_even_if_reader_returns_it(self):
        probe, port, _, clock = self.fixture()
        with probe:
            probe._send((1, "position"), 1)
            deadline = probe.pending[(1, "position")]["deadline_monotonic_ns"]
            response = port.queue[0][1]

            def late_chunk(wake, hard):
                self.assertEqual(hard, deadline)
                clock.advance(deadline - clock())
                return response, deadline

            with patch.object(probe.deadline_reader, "read_until", side_effect=late_chunk):
                with self.assertRaisesRegex(TimeoutError, "Reply arrived after"):
                    probe._receive(.003, UIDS)
            self.assertFalse(probe.rows[0]["ok"])
            self.assertEqual(probe.replies, 0)
        self.assertEqual(len(port.writes), 1)
        self.assertTrue(port.closed)

    def test_select_collection_keeps_identity_matching_and_readonly_counts(self):
        probe, port, _, _ = self.fixture()
        with probe:
            report = probe.collect(UIDS)
        self.assertEqual(report["status"], "READONLY_PIPELINE_COMPLETE")
        self.assertEqual(report["write_attempts"], 36)
        self.assertEqual(report["replies"], 36)
        self.assertTrue(report["identities_verified"])
        self.assertEqual([f.kind for f in port.writes], [0] * 12 + [17] * 24)
        self.assertEqual(report["max_observed_pending"], 4)
        self.assertEqual(report["plan"]["gap_ms"], .5)
        self.assertEqual(port.timeout_settings, [0])
        self.assertEqual(port.read_timeouts, [])
        self.assertEqual(port.write_timeout_settings, [])
        self.assertFalse(report["full_controller_50Hz_verified"])
        self.assertTrue(port.closed)

    def test_serial_baseline_does_not_construct_select_reader(self):
        probe, port, _, _ = self.fixture(receive_mode="serial", gap_ms=0.)
        with probe:
            report = probe.collect(UIDS)
        self.reader_factory.assert_not_called()
        self.assertEqual(report["status"], "READONLY_PIPELINE_COMPLETE")
        self.assertEqual(report["statistics_ms"]["cycle_duration"]["max"], 18.)
        self.assertEqual(report["replies"], 36)
        self.assertEqual(report["receiver_profile"]["mode"], "serial")
        self.assertIsNone(report["receiver_profile"]["deadline_reader"])
        self.assertTrue(port.read_timeouts)
        self.assertTrue(all(0 <= t <= .003 for t in port.read_timeouts))
        self.assertTrue(port.closed)

    def test_select_does_not_expand_public_write_allowlist(self):
        probe, port, _, _ = self.fixture()
        with probe:
            for key in ((0, "position"), (13, "position"), (True, "position"),
                        (1, "voltage"), (1, "enable")):
                with self.subTest(key=key), self.assertRaises(ValueError):
                    probe._send(key, 1)
            with self.assertRaises(ValueError):
                probe.serial.write(read_request(1, "position"))
            probe.current_write = (1, "position")
            probe.pending[probe.current_write] = {"deadline_monotonic_ns": probe.deadline_ns}
            # Control, enable, STOP and parameter-write packets stay forbidden.
            for kind in (1, 3, 4, 18):
                with self.subTest(kind=kind), self.assertRaises(ValueError):
                    probe.serial.write(wire((kind << 24) | (0xFD << 8) | 1, bytes(8)))
            with self.assertRaises(ValueError):
                probe.serial.write(read_request(1, "velocity"))
        self.assertEqual(port.writes, [])
        self.assertEqual(probe.write_attempts, 0)
        self.assertTrue(port.closed)

    def test_reader_setup_failure_closes_port_without_any_transmission(self):
        probe, port, _, _ = self.fixture()
        self.reader_factory.side_effect = OSError("receiver setup failed")
        with self.assertRaisesRegex(OSError, "receiver setup failed"):
            with probe:
                self.fail("A failed receiver must not enter the session")
        self.assertTrue(probe.poisoned)
        self.assertTrue(port.closed)
        self.assertEqual(port.writes, [])

    def test_receive_failure_keeps_partial_report_and_closes_without_retry(self):
        probe, port, _, _ = self.fixture()
        with probe:
            with patch.object(probe.deadline_reader, "read_until",
                              side_effect=OSError("serial disconnected")):
                report = probe.collect(UIDS)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertIn("serial disconnected", str(report["errors"]))
        self.assertEqual(report["write_attempts"], 1)
        self.assertEqual(report["replies"], 0)
        self.assertTrue(probe.poisoned)
        self.assertTrue(port.closed)
        self.assertEqual(len(port.writes), 1)


if __name__ == "__main__":
    unittest.main()
