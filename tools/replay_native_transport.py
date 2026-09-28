#!/usr/bin/env python3
"""Replay historical Type0/17 bytes through real C++ using local socket pairs.

There is no serial-device argument or hardware open.  Saved TX/RX bytes are
validated with the legacy decoder, then exchanged over local IPC.  Emulator
timestamps are newly generated and never count as Jetson/CAN latency evidence.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import select
import socket
import stat
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime"))
from singularitydog_hw import native_diagnostic_transport as native  # noqa: E402
from singularitydog_hw.can_readonly import ATParser, decode_reply, read_request  # noqa: E402

DEFAULT_LIBRARY = Path(__file__).resolve().parents[1] / "runtime/experiments/native_transport/libdog_transport.so"
HISTORY_CAPTURE = "dual-policy-once-current-boot-20260927-r4"


def need(condition, message):
    if not condition:
        raise ValueError(message)


def strict_json(source):
    def pairs(items):
        result = {}
        for key, value in items:
            need(key not in result, "Duplicate JSON key")
            result[key] = value
        return result

    def invalid(value):
        raise ValueError("Nonfinite JSON constant: " + value)

    return json.loads(source, object_pairs_hook=pairs, parse_constant=invalid)


def single_frame(wire):
    need(type(wire) is bytes and len(wire) == 17, "Exactly one 17-byte frame required")
    parser = ATParser()
    frames = parser.feed(wire)
    need(len(frames) == 1 and not parser.buffer and parser.discarded_bytes == 0,
         "Malformed historical AT frame")
    return frames[0]


def source_records(events, *, first_id, expected_cycles):
    """Join original write intents, raw RX frames and decoder results uniquely."""
    need(first_id in (1, 7) and type(expected_cycles) is int and 1 <= expected_cycles <= 100,
         "Invalid replay bus/cycle bound")
    need(type(events) is list and len(events) <= 20000
         and all(type(row) is dict for row in events), "Bounded event list required")
    expected_ids = set(range(first_id, first_id + 6))
    intents, replies, frame_events, byte_frames = {}, {}, [], []
    byte_parser = ATParser()
    for event in events:
        kind = event.get("kind")
        if kind == "pipeline_tx_intent":
            sequence = event.get("sequence")
            need(type(sequence) is int and sequence not in intents, "Duplicate/missing TX sequence")
            intents[sequence] = event
        elif kind == "pipeline_reply":
            sequence = event.get("sequence")
            need(type(sequence) is int and sequence not in replies, "Duplicate/missing reply sequence")
            need(event.get("ok") is True and event.get("write_returned_bytes") == 17,
                 "Incomplete historical reply")
            replies[sequence] = event
        elif kind == "pipeline_rx_frame":
            frame = single_frame(bytes.fromhex(event["wire_hex"]))
            need(all(event.get(key) == value for key, value in frame.record().items()),
                 "Historical frame metadata differs from original wire bytes")
            frame_events.append((event["monotonic_ns"], frame))
        elif kind == "pipeline_rx_bytes":
            byte_frames.extend(byte_parser.feed(bytes.fromhex(event["hex"])))
    count = 6 + 12 * expected_cycles
    need(set(intents) == set(replies) == set(range(1, count + 1)),
         "Source must contain six identities plus every complete twelve-read cycle")
    need(not byte_parser.buffer and byte_parser.discarded_bytes == 0
         and [f.wire for f in byte_frames] == [f.wire for _, f in frame_events]
         and len(frame_events) == count,
         "Raw receive chunks do not reproduce the saved RX frames exactly")
    used, result = set(), []
    for sequence in range(1, count + 1):
        intent, reply = intents[sequence], replies[sequence]
        mid, parameter, cycle = intent.get("motor_id"), intent.get("parameter"), intent.get("cycle")
        need(type(mid) is int and mid in expected_ids and type(cycle) is int
             and 0 <= cycle <= expected_cycles and parameter in ("identity", "position", "velocity"),
             "Unexpected historical read key")
        need(all(reply.get(key) == intent.get(key) for key in ("motor_id", "parameter", "cycle")),
             "Historical intent/reply key mismatch")
        wire = bytes.fromhex(intent["wire_hex"])
        need(wire == read_request(mid, None if parameter == "identity" else parameter),
             "Noncanonical or output-capable command in replay source")
        received = reply.get("received_monotonic_ns")
        matches = []
        for index, (stamp, frame) in enumerate(frame_events):
            if index in used or stamp != received:
                continue
            try:
                decoded = decode_reply(frame, mid, None if parameter == "identity" else parameter)
            except ValueError:
                continue
            if decoded.get("ok") is True and decoded == reply.get("result"):
                matches.append((index, frame, decoded))
        need(len(matches) == 1, "No unique raw RX frame reproduces the legacy reply")
        index, frame, decoded = matches[0]
        used.add(index)
        result.append({"sequence": sequence, "cycle": cycle, "motor_id": mid,
                       "parameter": parameter, "tx_hex": wire.hex(), "rx_hex": frame.wire.hex(),
                       "legacy_result": decoded,
                       "historical_write_started_ns": reply["write_started_monotonic_ns"],
                       "historical_received_ns": received})
    groups = {cycle: [row for row in result if row["cycle"] == cycle]
              for cycle in range(expected_cycles + 1)}
    need([(row["motor_id"], row["parameter"]) for row in groups[0]] ==
         [(mid, "identity") for mid in sorted(expected_ids)], "Identity group/order is incomplete")
    for cycle in range(1, expected_cycles + 1):
        need(len(groups[cycle]) == 12 and
             {(row["motor_id"], row["parameter"]) for row in groups[cycle]} ==
             {(mid, parameter) for mid in expected_ids for parameter in ("position", "velocity")},
             "Incomplete historical telemetry cycle")
    need([row["cycle"] for row in result] == sorted(row["cycle"] for row in result),
         "Historical cycles were interleaved/out of order")
    return result


def load_source(path, *, first_id, expected_cycles):
    path = Path(path).resolve(strict=True)
    need(stat.S_ISREG(path.stat().st_mode), "Replay input must be a regular saved file")
    source = path.read_bytes()
    records = source_records(strict_json(source), first_id=first_id, expected_cycles=expected_cycles)
    return records, hashlib.sha256(source).hexdigest()


def replay_bus(library, records, *, first_id, fragment=False):
    """Run a finite actual NativeSession against an in-process socket emulator."""
    host, device = socket.socketpair()
    host.setblocking(False)
    device.settimeout(.25)
    cancel_read, cancel_write = os.pipe()
    stop = threading.Event()
    errors, seen = [], []
    emulator_deadline = time.monotonic() + 10.

    def emulator():
        parser = ATParser()
        cursor = 0
        try:
            while cursor < len(records) and not stop.is_set():
                if time.monotonic() >= emulator_deadline:
                    raise TimeoutError("Local replay emulator deadline")
                if not select.select([device], [], [], .05)[0]:
                    continue
                chunk = device.recv(4096)
                if not chunk:
                    raise RuntimeError("Local host closed before replay completed")
                frames = parser.feed(chunk)
                need(parser.discarded_bytes == 0, "Native host sent invalid local bytes")
                for frame in frames:
                    need(cursor < len(records), "Unexpected extra native request")
                    expected = records[cursor]
                    need(frame.wire.hex() == expected["tx_hex"], "Native TX differs from historical TX")
                    rx = bytes.fromhex(expected["rx_hex"])
                    if fragment:
                        device.sendall(rx[:5])
                        device.sendall(rx[5:11])
                        device.sendall(rx[11:])
                    else:
                        device.sendall(rx)
                    seen.append(expected["sequence"])
                    cursor += 1
            need(cursor == len(records), "Local replay emulator ended early")
            need(not parser.buffer, "Local replay ended in a partial frame")
        except BaseException as error:
            errors.append(repr(error))
            try:
                os.write(cancel_write, b"x")
            except OSError:
                pass

    worker = threading.Thread(target=emulator, name=f"local-replay-{first_id}", daemon=True)
    results, cycles = [], []
    started = time.monotonic_ns()
    try:
        session = native.NativeSession(library, host.fileno(), first_id=first_id,
                                       cancel_fd=cancel_read, stop_proxy=False,
                                       gap_ns=600_000, window=3)
        worker.start()
        for cycle in sorted({row["cycle"] for row in records}):
            group = [row for row in records if row["cycle"] == cycle]
            native_records, stats = session.exchange([bytes.fromhex(row["tx_hex"]) for row in group],
                                                      timeout_ns=250_000_000)
            decoded = native.records_as_events(native_records, cycle=cycle)
            need(len(decoded) == len(group), "Native reply count differs")
            for expected, actual in zip(group, decoded):
                need(actual["request_wire_hex"] == expected["tx_hex"]
                     and actual["reply_wire_hex"] == expected["rx_hex"],
                     "Native record differs from original TX/RX bytes")
                need(actual["motor_id"] == expected["motor_id"]
                     and actual["parameter"] == expected["parameter"]
                     and actual["result"] == expected["legacy_result"],
                     "Native accepted reply differs from legacy value/ID decoder")
                results.append({"source_sequence": expected["sequence"], "cycle": cycle,
                                "motor_id": actual["motor_id"], "parameter": actual["parameter"],
                                "tx_hex": actual["request_wire_hex"], "rx_hex": actual["reply_wire_hex"],
                                "result": actual["result"], "exact_match": True})
            cycles.append({"cycle": cycle, "local_ipc_elapsed_ms": (stats.end_ns - stats.begin_ns) / 1e6,
                           "requests": len(group), "timing_is_hardware_evidence": False})
        worker.join(timeout=.5)
        need(not worker.is_alive() and not errors and len(seen) == len(records),
             "Emulator did not complete: " + repr(errors))
    finally:
        stop.set()
        try:
            os.write(cancel_write, b"x")
        except OSError:
            pass
        host.close()
        device.close()
        worker.join(timeout=.5) if worker.ident is not None else None
        os.close(cancel_read)
        os.close(cancel_write)
    return {"status": "EXACT_NATIVE_LOCAL_IPC_REPLAY", "records": results, "cycles": cycles,
            "count": len(results), "emulator_errors": errors,
            "elapsed_ms_local_only": (time.monotonic_ns() - started) / 1e6,
            "hardware_accessed": False, "jetson_latency_verified": False}


def replay_files(front, rear, library_path, *, expected_cycles=20, fragment=True):
    sources = {"front": load_source(front, first_id=1, expected_cycles=expected_cycles),
               "rear": load_source(rear, first_id=7, expected_cycles=expected_cycles)}
    library = native.load_library(library_path)
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {name: pool.submit(replay_bus, library, pair[0], first_id=first, fragment=fragment)
                   for name, pair, first in (("front", sources["front"], 1), ("rear", sources["rear"], 7))}
        buses = {name: future.result(timeout=12.) for name, future in futures.items()}
    need(all(bus["count"] == 6 + 12 * expected_cycles for bus in buses.values()),
         "Replay did not cover both complete buses")
    return {"schema": "singularitydog.native-transport-saved-byte-replay.v1",
            "status": "EXACT_MATCH_ALL_SAVED_BYTES_AND_VALUES", "buses": buses,
            "source_sha256": {name: pair[1] for name, pair in sources.items()},
            "native_binary_sha256": hashlib.sha256(Path(library_path).read_bytes()).hexdigest(),
            "requests_replayed": sum(bus["count"] for bus in buses.values()),
            "saved_telemetry_cycles": expected_cycles,
            "local_ipc_only": True, "historical_raw_bytes_changed": False,
            "hardware_accessed": False, "robot_frames_transmitted": 0,
            "timing_note": "New socket-emulator timestamps; historical Jetson/CAN timing is NOT replayed or validated",
            "full_controller_50Hz_verified": False, "jetson_latency_verified": False,
            "approved_for_runtime": False, "motor_output_available": False}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--history-root", type=Path)
    ap.add_argument("--front-events", type=Path)
    ap.add_argument("--rear-events", type=Path)
    ap.add_argument("--library", type=Path, default=DEFAULT_LIBRARY)
    ap.add_argument("--cycles", type=int, default=20)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(argv)
    try:
        if args.history_root:
            need(args.front_events is None and args.rear_events is None, "Choose history root or explicit event files")
            logs = (args.history_root if args.history_root.name == "singularitydog-logs"
                    else args.history_root / "singularitydog-logs")
            front, rear = (logs / HISTORY_CAPTURE / f"events-{bus}.json" for bus in ("front", "rear"))
        else:
            need(args.front_events is not None and args.rear_events is not None, "Both saved event files are required")
            front, rear = args.front_events, args.rear_events
        output = args.output.expanduser()
        need(not output.is_symlink() and not output.exists() and output.parent.is_dir(),
             "Use a fresh result path in an existing private directory")
        report = replay_files(front, rear, args.library, expected_cycles=args.cycles)
        fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
    except (OSError, ValueError, RuntimeError, KeyError, TimeoutError) as error:
        ap.error(str(error))
    print(json.dumps({"output": str(output), "status": report["status"],
                      "requests_replayed": report["requests_replayed"], "hardware_accessed": False,
                      "jetson_latency_verified": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
