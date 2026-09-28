"""Real native replay with synthetic saved logs and local IPC, no robot ports."""

import copy
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import time
import unittest

spec = importlib.util.spec_from_file_location("replay_native_transport", Path(__file__).with_name("replay_native_transport.py"))
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def synthetic_events(first_id, cycles=1):
    ids = range(first_id, first_id + 6)
    requests = [(0, mid, "identity") for mid in ids]
    requests += [(cycle, mid, parameter) for cycle in range(1, cycles + 1)
                 for parameter in ("position", "velocity") for mid in ids]
    events = []
    for sequence, (cycle, mid, parameter) in enumerate(requests, 1):
        request = replay.read_request(mid, None if parameter == "identity" else parameter)
        frame = replay.single_frame(request)
        can_id = (frame.kind << 24) | (mid << 8) | (0xfe if parameter == "identity" else 0xfd)
        payload = (bytes([mid]) * 8 if parameter == "identity" else
                   frame.data[:4] + struct.pack("<f", mid / 10 + cycle / 100))
        response = b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + payload + b"\r\n"
        received = 1000 * sequence + 20
        reply_frame = replay.single_frame(response)
        decoded = replay.decode_reply(reply_frame, mid, None if parameter == "identity" else parameter)
        events.extend([
            {"kind": "pipeline_tx_intent", "sequence": sequence, "cycle": cycle,
             "motor_id": mid, "parameter": parameter, "wire_hex": request.hex()},
            {"kind": "pipeline_rx_bytes", "monotonic_ns": received, "hex": response[:5].hex()},
            {"kind": "pipeline_rx_bytes", "monotonic_ns": received, "hex": response[5:].hex()},
            {"kind": "pipeline_rx_frame", "monotonic_ns": received, **reply_frame.record()},
            {"kind": "pipeline_reply", "sequence": sequence, "cycle": cycle,
             "motor_id": mid, "parameter": parameter, "ok": True,
             "write_returned_bytes": 17, "received_monotonic_ns": received,
             "write_started_monotonic_ns": received - 20, "result": decoded},
        ])
    return events


class SavedSourceTests(unittest.TestCase):
    def test_raw_byte_fragments_reproduce_every_legacy_value_and_key(self):
        rows = replay.source_records(synthetic_events(7, 2), first_id=7, expected_cycles=2)
        self.assertEqual(len(rows), 30)
        self.assertEqual(rows[0]["legacy_result"]["mcu_uid_hex"], "07" * 8)
        self.assertAlmostEqual(rows[-1]["legacy_result"]["value"], 1.22, places=6)

    def test_changed_raw_chunk_is_rejected_even_if_decoded_metadata_is_unchanged(self):
        events = synthetic_events(1)
        chunk = next(row for row in events if row["kind"] == "pipeline_rx_bytes")
        chunk["hex"] = "00" + chunk["hex"][2:]
        with self.assertRaisesRegex(ValueError, "Raw receive chunks"):
            replay.source_records(events, first_id=1, expected_cycles=1)

    def test_decoded_value_tampering_missing_or_duplicate_request_rejected(self):
        for mutation in ("value", "missing", "duplicate"):
            events = synthetic_events(1)
            if mutation == "value":
                next(row for row in events if row["kind"] == "pipeline_reply"
                     and row["parameter"] == "position")["result"]["value"] += 1
            elif mutation == "missing":
                events = [row for row in events if not (row["kind"] == "pipeline_tx_intent" and row["sequence"] == 1)]
            else:
                events.append(copy.deepcopy(events[0]))
            with self.assertRaises(ValueError):
                replay.source_records(events, first_id=1, expected_cycles=1)

    def test_enable_frame_is_not_replayed(self):
        events = synthetic_events(1)
        row = next(row for row in events if row["kind"] == "pipeline_tx_intent")
        wire = bytes.fromhex(row["wire_hex"])
        can_id = (3 << 24) | (0xfd << 8) | 1
        row["wire_hex"] = (wire[:2] + ((can_id << 3) | 4).to_bytes(4, "big") + wire[6:]).hex()
        with self.assertRaisesRegex(ValueError, "output-capable"):
            replay.source_records(events, first_id=1, expected_cycles=1)


class NativeSavedReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        build_path = replay.DEFAULT_LIBRARY.parent / "build.py"
        spec = importlib.util.spec_from_file_location("build_replay_native", build_path)
        build = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(build)
        cls.path = build.build()
        cls.library = replay.native.load_library(cls.path)

    def test_two_buses_actual_cpp_exchange_all_original_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = []
            for name, first in (("front", 1), ("rear", 7)):
                path = Path(folder) / f"{name}.json"
                path.write_text(json.dumps(synthetic_events(first)))
                paths.append(path)
            report = replay.replay_files(*paths, self.path, expected_cycles=1, fragment=True)
            self.assertEqual(report["requests_replayed"], 36)
            self.assertEqual(report["status"], "EXACT_MATCH_ALL_SAVED_BYTES_AND_VALUES")
            self.assertFalse(report["hardware_accessed"])
            self.assertFalse(report["jetson_latency_verified"])
            self.assertFalse(report["full_controller_50Hz_verified"])
            for bus in report["buses"].values():
                self.assertTrue(all(row["exact_match"] for row in bus["records"]))
                self.assertTrue(all(not row["timing_is_hardware_evidence"] for row in bus["cycles"]))

    def test_native_rejection_joins_emulator_without_hanging(self):
        rows = replay.source_records(synthetic_events(1), first_id=1, expected_cycles=1)
        rows = rows[:1]
        wrong = bytearray.fromhex(rows[0]["rx_hex"])
        # Canonical frame, but wrong motor source ID for the pending request.
        wrong[2:6] = (((2 << 8 | 0xfe) << 3) | 4).to_bytes(4, "big")
        rows[0]["rx_hex"] = wrong.hex()
        began = time.monotonic()
        with self.assertRaises(replay.native.ExchangeError):
            replay.replay_bus(self.library, rows, first_id=1)
        self.assertLess(time.monotonic() - began, 2.)


if __name__ == "__main__":
    unittest.main()
