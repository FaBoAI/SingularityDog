"""Audit saved STOP-proxy timing against the active output request schedule.

File-only arithmetic. The extra pacing term is a comparison of request counts,
not a prediction of Type1 reply timing or a certification of active control.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct


IDS_BY_BUS = {"front": set(range(1, 7)), "rear": set(range(7, 13))}
ACTIVE_INPUT_REQUESTS_PER_BUS = {"v2": 8, "v3": 7}
STOP_PROXY_REQUESTS_PER_BUS_PER_PHASE = 6
PERIOD_MS = 20.0


def _stop_id(frame):
    if not isinstance(frame, dict) or frame.get("written") != 17 or frame.get("received") != 17:
        raise ValueError("Incomplete STOP-proxy transaction")
    try:
        wire = bytes.fromhex(frame["tx_hex"])
        reply = bytes.fromhex(frame["rx_hex"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Invalid STOP-proxy wire") from error
    if (len(wire) != 17 or wire[:2] != b"AT" or wire[5] & 7 != 4 or
            wire[6] != 8 or wire[-2:] != b"\r\n"):
        raise ValueError("Invalid STOP-proxy framing")
    can_id = int.from_bytes(wire[2:6], "big") >> 3
    if can_id >> 24 != 4 or can_id & 0xFFFF00 != 0xFD00 or wire[7:15] != bytes(8):
        raise ValueError("Expected canonical Type4 STOP")
    mid = can_id & 255
    if (len(reply) != 17 or reply[:2] != b"AT" or reply[5] & 7 != 4 or
            reply[6] != 8 or reply[-2:] != b"\r\n"):
        raise ValueError("Invalid STOP-proxy reply framing")
    reply_id = int.from_bytes(reply[2:6], "big") >> 3
    if (reply_id >> 24 != 2 or (reply_id >> 8) & 255 != mid or
            reply_id & 255 != 0xFD or (reply_id >> 22) & 3 or
            (reply_id >> 16) & 63 or reply[7:10] == b"\x00\xc4\x56"):
        raise ValueError("Expected fault-free mode-zero Type2 STOP reply")
    for key in ("start_ns", "finish_ns", "received_ns", "deadline_ns"):
        if type(frame.get(key)) is not int:
            raise ValueError("Invalid STOP-proxy timestamp")
    if not 0 < frame["start_ns"] <= frame["finish_ns"] <= frame["received_ns"] < frame["deadline_ns"]:
        raise ValueError("Noncausal STOP-proxy transaction")
    return mid


def _voltage_id(frame):
    if not isinstance(frame, dict) or frame.get("written") != 17 or frame.get("received") != 17:
        raise ValueError("Incomplete voltage-proxy transaction")
    try:
        wire = bytes.fromhex(frame["tx_hex"])
        reply = bytes.fromhex(frame["rx_hex"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Invalid voltage-proxy wire") from error
    if (len(wire) != 17 or wire[:2] != b"AT" or wire[5] & 7 != 4 or wire[6] != 8 or
            wire[7:15] != b"\x1c\x70" + bytes(6) or wire[-2:] != b"\r\n"):
        raise ValueError("Expected canonical Type17 voltage request")
    can_id = int.from_bytes(wire[2:6], "big") >> 3
    mid = can_id & 255
    if can_id != (17 << 24) | (0xFD << 8) | mid:
        raise ValueError("Invalid voltage-proxy CAN ID")
    if (len(reply) != 17 or reply[:2] != b"AT" or reply[5] & 7 != 4 or
            reply[6] != 8 or reply[-2:] != b"\r\n" or
            reply[7:11] != b"\x1c\x70\x00\x00" or
            int.from_bytes(reply[2:6], "big") >> 3 != ((17 << 24) | (mid << 8) | 0xFD) or
            not math.isfinite(struct.unpack("<f", reply[11:15])[0])):
        raise ValueError("Invalid voltage-proxy reply")
    for key in ("start_ns", "finish_ns", "received_ns", "deadline_ns"):
        if type(frame.get(key)) is not int:
            raise ValueError("Invalid voltage-proxy timestamp")
    if not 0 < frame["start_ns"] <= frame["finish_ns"] <= frame["received_ns"] < frame["deadline_ns"]:
        raise ValueError("Noncausal voltage-proxy transaction")
    return mid


def summarize(report, records, *, profile="v2"):
    if profile not in ACTIVE_INPUT_REQUESTS_PER_BUS:
        raise ValueError("Select active profile v2 or v3")
    if report.get("mode") != "stop-proxy" or report.get("status") != "COMPLETE_DIAGNOSTIC":
        raise ValueError("Complete STOP-proxy report required")
    if report.get("motor_enable_sent") is not False or report.get("learned_targets_sent") is not False:
        raise ValueError("Report is not disabled STOP-proxy evidence")
    voltage_proxy = report.get("v3_voltage_proxy") is True
    if voltage_proxy and (profile != "v3" or report.get("plan", {}).get("v3_voltage_proxy") is not True or
                          report.get("plan", {}).get("requests_per_cycle") != 26):
        raise ValueError("V3 voltage proxy requires explicit 26-request plan")
    measurements = report.get("measurements")
    if not isinstance(measurements, list) or not isinstance(records, list) or not measurements:
        raise ValueError("Measurements and raw records required")
    if len(records) != len(measurements) or len(records) != report.get("cycles_completed"):
        raise ValueError("Completed cycle count mismatch")
    plan = report.get("plan", {})
    if plan.get("window") != 3 or type(plan.get("request_gap_us")) is not int:
        raise ValueError("Explicit gap and window 3 required")
    gap_us = plan["request_gap_us"]
    if not 600 <= gap_us <= 5000:
        raise ValueError("Invalid measured request gap")

    margins = []
    host_misses = reply_misses = whole_misses = 0
    for index, (cycle, row) in enumerate(zip(records, measurements), 1):
        if cycle.get("cycle") != index:
            raise ValueError("Cycle number mismatch")
        for phase in ("acquired", "output"):
            batches = cycle.get(phase)
            if not isinstance(batches, dict) or set(batches) != set(IDS_BY_BUS):
                raise ValueError("Missing STOP-proxy bus")
            for bus, ids in IDS_BY_BUS.items():
                frames = batches[bus].get("records") if isinstance(batches[bus], dict) else None
                expected_count=7 if voltage_proxy and phase=="acquired" else 6
                if not isinstance(frames, list) or len(frames) != expected_count:
                    raise ValueError("Expected complete STOP/voltage-proxy frame count")
                stop_frames=frames[:6] if voltage_proxy and phase=="acquired" else frames
                if {_stop_id(frame) for frame in stop_frames} != ids:
                    raise ValueError("STOP-proxy motor ID mismatch")
                if voltage_proxy and phase=="acquired" and _voltage_id(frames[6]) != sorted(ids)[(index-1)%6]:
                    raise ValueError("Incorrect rotating voltage-proxy motor ID")
        output_frames = [frame for bus in IDS_BY_BUS for frame in cycle["output"][bus]["records"]]
        if (max(frame["finish_ns"] for frame in output_frames) != row.get("final_host_write_ns") or
                max(frame["received_ns"] for frame in output_frames) != row.get("last_proxy_reply_ns")):
            raise ValueError("Raw output timestamps differ from measured row")
        needed = ("oldest_input_to_final_host_write_ms", "oldest_input_to_last_reply_ms", "whole_iteration_ms")
        if any(type(row.get(key)) not in (int, float) for key in needed):
            raise ValueError("Missing measured timing")
        host_misses += row[needed[0]] > PERIOD_MS
        reply_misses += row[needed[1]] > PERIOD_MS
        whole_misses += row[needed[2]] > PERIOD_MS
        margins.append(PERIOD_MS - row[needed[2]])
    if host_misses != report.get("host_deadline_misses") or whole_misses != report.get("iteration_deadline_misses"):
        raise ValueError("Report deadline counts disagree with measurements")

    extra_per_bus = ACTIVE_INPUT_REQUESTS_PER_BUS[profile] - (7 if voltage_proxy else 6)
    extra_spacing_ms = extra_per_bus * gap_us / 1000
    return {
        "scope": "saved disabled STOP-proxy timestamps and request-count arithmetic only",
        "cycles": len(records),
        "profile": profile,
        "requests_per_cycle": {"stop_proxy": 26 if voltage_proxy else 24,
                               "active": 2 * (ACTIVE_INPUT_REQUESTS_PER_BUS[profile] + 6)},
        "additional_input_requests_per_bus": extra_per_bus,
        "selected_request_gap_us": gap_us,
        "extra_spacing_if_active_uses_same_gap_ms": extra_spacing_ms,
        "measured_stop_proxy_deadline_misses": {"final_host_write": host_misses,
                                                 "last_reply": reply_misses, "whole_iteration": whole_misses},
        "measured_stop_proxy_whole_iteration_headroom_ms": {
            "median": statistics.median(margins), "minimum": min(margins),
            "cycles_below_same_gap_extra_spacing": sum(margin < extra_spacing_ms for margin in margins)},
        "active_profile_gap_checked": False,
        "active_type1_reply_timing_measured": False,
        "active_full_cycle_20ms_verified": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("records", type=Path)
    parser.add_argument("--profile", choices=tuple(ACTIVE_INPUT_REQUESTS_PER_BUS), default="v2")
    args = parser.parse_args()
    report_raw, records_raw = args.report.read_bytes(), args.records.read_bytes()
    result = summarize(json.loads(report_raw), json.loads(records_raw), profile=args.profile)
    result["source_report_sha256"] = hashlib.sha256(report_raw).hexdigest()
    result["source_records_sha256"] = hashlib.sha256(records_raw).hexdigest()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
