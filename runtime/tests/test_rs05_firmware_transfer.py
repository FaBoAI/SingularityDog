"""Offline OTA boundary tests: synthetic bytes, virtual time, no serial device."""
from collections import Counter
import hashlib
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import rs05_firmware_transfer as ota
from singularitydog_hw.can_readonly import ATParser, Frame, HOST_ID


IMAGE = bytes(i % 251 for i in range(93584))
UID = bytes.fromhex("0102030405060708")  # Synthetic, never a recorded motor UID.
MS = 1_000_000


def decode_request(wire):
    """Decode the public wire layout independently of frame_for's return value."""
    if len(wire) != 17 or wire[:2] != b"AT" or wire[6] != 8 or wire[-2:] != b"\r\n":
        raise AssertionError("Noncanonical transmission")
    encoded = int.from_bytes(wire[2:6], "big")
    if encoded & 7 != 4:
        raise AssertionError("Non-extended/data transmission")
    cid = encoded >> 3
    return cid >> 24, (cid >> 8) & 0xffff, cid & 0xff, wire[7:15]


def reply(kind, *, mid=1, host=HOST_ID, status=0, flags=4, data=bytes(8)):
    cid = kind << 24 | status << 16 | mid << 8 | host
    return b"AT" + ((cid << 3) | flags).to_bytes(4, "big") + bytes([len(data)]) + data + b"\r\n"


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now


class FakeIO:
    """ACKs become readable at scheduled times; no real waiting or hardware."""
    def __init__(self, clock, *, answers=None, write_failure=None, read_failure=None):
        self.clock = clock
        self.answers = answers
        self.write_failure = write_failure
        self.read_failure = read_failure
        self.pending = []
        self.writes = []
        self.write_timeout = None

    def write(self, wire):
        ordinal = len(self.writes)
        kind, _, mid, _ = decode_request(wire)
        self.writes.append((wire, self.clock()))
        self.clock.now += 50_000
        if self.write_failure:
            outcome = self.write_failure(ordinal)
            if outcome is not None:
                if isinstance(outcome, BaseException):
                    raise outcome
                return outcome
        answer = (self.answers(ordinal, kind, mid) if self.answers
                  else [(100_000, reply(kind, mid=mid))])
        for delay, chunk in answer:
            self.pending.append((self.clock() + delay, chunk))
        self.pending.sort(key=lambda item: item[0])
        return len(wire)

    def ready(self, seconds):
        end = self.clock() + max(1, round(seconds * 1e9))
        if self.pending and self.pending[0][0] <= end:
            self.clock.now = max(self.clock(), self.pending[0][0])
            return True
        self.clock.now = end
        return False

    def read(self):
        if self.read_failure:
            self.read_failure()
        when, chunk = self.pending.pop(0)
        if when > self.clock():
            raise AssertionError("Read before ready")
        return chunk


class ImageAndProtocolTests(unittest.TestCase):
    def test_public_asset_identity_is_fixed(self):
        self.assertEqual(ota.BIN_SIZE, 93584)
        self.assertEqual(ota.PACKETS, 11698)
        self.assertEqual(ota.BIN_SHA256,
                         "3a4dfc1e9ad0ff2116b7b876c1fced3b8d23cf705e5d6366e5a286bac3989047")
        self.assertNotEqual(hashlib.sha256(IMAGE).hexdigest(), ota.BIN_SHA256)

    def test_image_wrong_hash_is_rejected_without_a_device(self):
        with self.assertRaisesRegex(ValueError, "SHA256"):
            ota.verify_image(IMAGE)

    def test_image_wrong_type_or_length_is_rejected(self):
        for image in (b"", IMAGE[:-1], IMAGE + b"x", bytearray(IMAGE), memoryview(IMAGE), None):
            with self.subTest(type=type(image), length=len(image) if image is not None else None):
                with self.assertRaisesRegex(ValueError, "length"):
                    ota.verify_image(image)

    def test_constructor_does_not_accept_unverified_synthetic_image(self):
        port = FakeIO(Clock())
        with self.assertRaisesRegex(ValueError, "SHA256"):
            ota.FirmwareTransfer(port, IMAGE, 1, UID)
        self.assertEqual(port.writes, [])

    def test_start_info_data16bit_and_end_exact_payloads(self):
        cases = [(0, 11, HOST_ID, UID),
                 (1, 12, HOST_ID, struct.pack("<II", 93584, 11698)),
                 (ota.PACKETS + 2, 14, 0, struct.pack("<II", 11698, 0))]
        for index in (0, 255, 256, 257, 4096, 11697):
            cases.append((index + 2, 13, index, IMAGE[index * 8:(index + 1) * 8]))
        for ordinal, kind, data, payload in cases:
            with self.subTest(ordinal=ordinal):
                result_kind, wire = ota.frame_for(IMAGE, 12, UID, ordinal)
                self.assertEqual(result_kind, kind)
                self.assertEqual(decode_request(wire), (kind, data, 12, payload))

    def test_invalid_schedule_inputs_are_rejected(self):
        invalid = [(mid, UID, 0) for mid in (0, 13, True, 1.0)]
        invalid += [(1, uid, 0) for uid in (UID[:-1], UID + b"x", bytearray(UID), "0102030405060708")]
        invalid += [(1, UID, n) for n in (-1, ota.PACKETS + 3, True, 2.0)]
        for mid, uid, ordinal in invalid:
            with self.subTest(mid=mid, uid=uid, ordinal=ordinal):
                with self.assertRaises(ValueError):
                    ota.frame_for(IMAGE, mid, uid, ordinal)

    def test_ack_all_nonzero_statuses_fail(self):
        for status in range(1, 256):
            with self.subTest(status=status):
                frame, = ATParser().feed(reply(13, status=status))
                with self.assertRaisesRegex(ValueError, "failure"):
                    ota.validate_ack(frame, 13, 1)

    def test_ack_identity_kind_flags_and_dlc_are_exact(self):
        bad = [reply(11), reply(13, mid=2), reply(13, host=0xFE),
               reply(13, flags=0), reply(13, flags=5), reply(13, data=bytes(7))]
        for wire in bad:
            with self.subTest(wire=wire.hex()):
                frame, = ATParser().feed(wire)
                with self.assertRaises(ValueError):
                    ota.validate_ack(frame, 13, 1)
        with self.assertRaisesRegex(ValueError, "Noncanonical"):
            ota.validate_ack(Frame(1 << 29, 4, bytes(8), b""), 13, 1)


class TransferTests(unittest.TestCase):
    def build(self, *, answers=None, write_failure=None, read_failure=None, check=None, mid=1):
        clock = Clock()
        port = FakeIO(clock, answers=answers, write_failure=write_failure, read_failure=read_failure)
        with patch.object(ota, "verify_image", return_value=IMAGE) as verify:
            engine = ota.FirmwareTransfer(port, IMAGE, mid, UID, clock=clock,
                read_ready=port.ready, read_bytes=port.read, check=check or (lambda: None))
            verify.assert_called_once_with(IMAGE)
        return engine, port, clock

    def assert_incomplete(self, engine, port, writes):
        self.assertEqual(engine.report["status"], "INCOMPLETE")
        self.assertFalse(engine.report["end_acknowledged"])
        self.assertFalse(engine.report["version_verified"])
        self.assertEqual(len(port.writes), writes)
        self.assertEqual(engine.report["transmit_attempts"], writes)
        self.assertIn("failure", engine.report)
        self.assertFalse(engine.report["automatic_retry"])
        self.assertFalse(engine.report["cleanup_transmission_available"])
        for wire, _ in port.writes:
            self.assertIn(decode_request(wire)[0], (11, 12, 13, 14))

    def test_all_11701_transmissions_and_reassembled_firmware(self):
        engine, port, clock = self.build(mid=12)
        result = engine.run()
        decoded = [decode_request(wire) for wire, _ in port.writes]
        self.assertEqual(len(decoded), 11701)
        self.assertEqual(Counter(x[0] for x in decoded), {11: 1, 12: 1, 13: 11698, 14: 1})
        self.assertEqual(decoded[0], (11, HOST_ID, 12, UID))
        self.assertEqual(decoded[1], (12, HOST_ID, 12, struct.pack("<II", 93584, 11698)))
        self.assertEqual([x[1] for x in decoded[2:-1]], list(range(11698)))
        self.assertTrue(all(x[2] == 12 for x in decoded))
        self.assertEqual(b"".join(x[3] for x in decoded[2:-1]), IMAGE)
        self.assertEqual(decoded[-1], (14, 0, 12, struct.pack("<II", 11698, 0)))
        self.assertEqual(result["status"], "TRANSFER_ACK_COMPLETE_PENDING_VERSION")
        self.assertEqual(result["acknowledged_data_packets"], 11698)
        self.assertTrue(result["end_acknowledged"])
        self.assertFalse(result["version_verified"])
        self.assertLess(clock() - engine.started, ota.MAX_TRANSFER_NS)
        for previous, current in zip(engine.tx_log, engine.tx_log[1:]):
            self.assertGreaterEqual(current["write_started_ns"] - previous["ack_received_ns"], ota.GAP_NS)

    def test_split_ack_is_accepted_but_never_advances_before_completion(self):
        def fragmented(ordinal, kind, mid):
            ack = reply(kind, mid=mid)
            return [(100_000, ack[:1]), (200_000, ack[1:7]), (300_000, ack[7:])]
        engine, port, _ = self.build(answers=fragmented)
        engine.run()
        self.assertEqual(engine.report["status"], "TRANSFER_ACK_COMPLETE_PENDING_VERSION")
        self.assertEqual(len(port.writes), 11701)
        self.assertEqual(len(engine.raw_log), 35103)

    def test_partial_writes_never_retry_or_cleanup_in_any_phase(self):
        for fail_at in (0, 1, 2, 258, ota.PACKETS + 2):
            with self.subTest(fail_at=fail_at):
                engine, port, _ = self.build(write_failure=lambda n: 16 if n == fail_at else None)
                engine.run()
                self.assert_incomplete(engine, port, fail_at + 1)
                self.assertIn("Partial OTA write", engine.report["failure"])

    def test_invalid_write_returns_are_not_success(self):
        for returned in (0, 18, True, 17.0, "17"):
            with self.subTest(returned=returned):
                engine, port, _ = self.build(write_failure=lambda n: returned)
                engine.run()
                self.assert_incomplete(engine, port, 1)

    def test_write_exception_keeps_attempt_evidence_without_followup(self):
        engine, port, _ = self.build(write_failure=lambda n: OSError("uncertain write"))
        engine.run()
        self.assert_incomplete(engine, port, 1)
        self.assertTrue(engine.report["bootloader_entry_attempted"])
        self.assertIsNotNone(engine.tx_log[0]["write_finished_ns"])
        self.assertIsNone(engine.tx_log[0]["returned_bytes"])

    def test_missing_ack_stops_in_each_phase_without_retry(self):
        for fail_at in (0, 1, 2, ota.PACKETS + 2):
            with self.subTest(fail_at=fail_at):
                def answers(ordinal, kind, mid):
                    return [] if ordinal == fail_at else [(100_000, reply(kind, mid=mid))]
                engine, port, _ = self.build(answers=answers)
                engine.run()
                self.assert_incomplete(engine, port, fail_at + 1)
                self.assertIn("acknowledgement missing", engine.report["failure"])

    def test_wrong_or_failed_data_ack_does_not_select_an_alternate_path(self):
        bad = [reply(11), reply(12), reply(14), reply(24), reply(13, mid=2),
               reply(13, host=0xFE), reply(13, flags=0), reply(13, flags=5),
               reply(13, data=bytes(7)), reply(13, status=0x0F), reply(13, status=0xF0)]
        for bad_ack in bad:
            with self.subTest(bad_ack=bad_ack.hex()):
                engine, port, _ = self.build(answers=lambda n, k, m:
                    [(100_000, bad_ack if n == 2 else reply(k, mid=m))])
                engine.run()
                self.assert_incomplete(engine, port, 3)
                self.assertEqual(engine.report["acknowledged_data_packets"], 0)

    def test_duplicate_or_extra_partial_in_same_read_blocks_advance(self):
        for extra in (reply(13), b"A", b"AT", b"AT\x00", b"noise"):
            with self.subTest(extra=extra.hex()):
                engine, port, _ = self.build(answers=lambda n, k, m:
                    [(100_000, reply(k, mid=m) + (extra if n == 2 else b""))])
                engine.run()
                self.assert_incomplete(engine, port, 3)

    def test_delayed_duplicate_or_partial_in_gap_blocks_next_write(self):
        for extra in (reply(13), b"AT"):
            with self.subTest(extra=extra.hex()):
                def answers(n, k, m):
                    return [(100_000, reply(k, mid=m))] + ([(200_000, extra)] if n == 2 else [])
                engine, port, _ = self.build(answers=answers)
                engine.run()
                self.assert_incomplete(engine, port, 3)
                self.assertIn("boundary", engine.report["failure"])

    def test_duplicate_or_partial_with_end_ack_never_marks_end_success(self):
        for extra in (reply(14), b"AT"):
            with self.subTest(extra=extra.hex()):
                engine, port, _ = self.build(answers=lambda n, k, m:
                    [(100_000, reply(k, mid=m) + (extra if k == 14 else b""))])
                engine.run()
                self.assert_incomplete(engine, port, 11701)

    def test_stale_or_partial_bytes_before_start_prevent_all_writes(self):
        for stale in (reply(11), b"AT", b"noise"):
            with self.subTest(stale=stale.hex()):
                engine, port, clock = self.build()
                port.pending.append((clock() + MS, stale))
                engine.run()
                self.assert_incomplete(engine, port, 0)
                self.assertFalse(engine.report["bootloader_entry_attempted"])

    def test_late_ack_is_not_success_even_when_select_returns_ready(self):
        engine, port, _ = self.build(answers=lambda n, k, m:
            [(ota.ACK_TIMEOUT_NS - 50_000, reply(k, mid=m))])
        engine.run()
        self.assert_incomplete(engine, port, 1)
        self.assertIn("Late OTA bytes", engine.report["failure"])
        self.assertEqual(len(engine.raw_log), 1)

    def test_unfinished_ack_times_out_and_retains_partial_evidence(self):
        partial = reply(11)[:9]
        engine, port, _ = self.build(answers=lambda *_: [(100_000, partial)])
        engine.run()
        self.assert_incomplete(engine, port, 1)
        self.assertEqual(engine.report["residual_hex"], partial.hex())
        self.assertIn("acknowledgement missing", engine.report["failure"])

    def test_read_exception_prevents_any_followup(self):
        def bad_read():
            raise OSError("serial read failed")
        engine, port, _ = self.build(read_failure=bad_read)
        engine.run()
        self.assert_incomplete(engine, port, 1)

    def test_bad_read_type_and_malformed_stream_fail(self):
        for bad in (bytearray(reply(11)), reply(11)[:-2] + b"xx", b"noise" + reply(11)):
            with self.subTest(bad=bad):
                engine, port, _ = self.build(answers=lambda n, k, m: [(100_000, bad)])
                engine.run()
                self.assert_incomplete(engine, port, 1)

    def test_total_deadline_exhaustion_stops_before_another_write(self):
        engine, port, clock = self.build()
        def stall(n):
            if n == 2:
                clock.now += ota.MAX_TRANSFER_NS
            return None
        port.write_failure = stall
        engine.run()
        self.assert_incomplete(engine, port, 3)
        self.assertIn("total deadline", engine.report["failure"])

    def test_cancellation_before_start_sends_nothing(self):
        def cancel():
            raise KeyboardInterrupt("cancel")
        engine, port, _ = self.build(check=cancel)
        engine.run()
        self.assert_incomplete(engine, port, 0)

    def test_cancellation_after_data_write_has_no_cleanup_transmission(self):
        engine, port, _ = self.build()
        def cancel():
            if len(port.writes) == 3:
                raise KeyboardInterrupt("cancel after first data")
        engine.check = cancel
        engine.run()
        self.assert_incomplete(engine, port, 3)

    def test_cancellation_after_start_ack_prevents_info(self):
        engine, port, _ = self.build()
        cancelled = False
        def progress(_):
            nonlocal cancelled
            cancelled = True
        def check():
            if cancelled:
                raise KeyboardInterrupt("cancel at acknowledged boundary")
        engine.progress = progress
        engine.check = check
        engine.run()
        self.assert_incomplete(engine, port, 1)
        self.assertIsNotNone(engine.tx_log[0]["ack_received_ns"])

    def test_cancel_during_receive_preserves_raw_evidence(self):
        engine, port, _ = self.build()
        cancelled = False
        def after_read():
            nonlocal cancelled
            cancelled = True
        def check():
            if cancelled:
                raise KeyboardInterrupt("cancel while receiving")
        port.read_failure = after_read
        engine.check = check
        engine.run()
        self.assert_incomplete(engine, port, 1)
        self.assertEqual(engine.raw_log[0]["hex"], reply(11).hex())
        self.assertIsNone(engine.tx_log[0]["ack_received_ns"])

    def test_backward_clock_after_write_prevents_followup(self):
        engine, port, clock = self.build()
        def backwards(n):
            clock.now -= 100 * MS
            return None
        port.write_failure = backwards
        engine.run()
        self.assert_incomplete(engine, port, 1)
        self.assertIn("monotonic clock", engine.report["failure"])

    def test_same_object_cannot_run_twice_after_success_or_failure(self):
        for fail in (False, True):
            with self.subTest(fail=fail):
                engine, port, _ = self.build(answers=(lambda *_: []) if fail else None)
                engine.run()
                previous_count = len(port.writes)
                previous_report = dict(engine.report)
                with self.assertRaisesRegex(ValueError, "single use"):
                    engine.run()
                self.assertEqual(len(port.writes), previous_count)
                self.assertEqual(engine.report, previous_report)


if __name__ == "__main__":
    unittest.main()
