"""Assess saved 26-request traces for conditional feedback reuse, without I/O.

The timing of a different 14-request path was not measured in a 26-request log.
This reports original-time reuse failures and independent conditional budget
projections. A projection is neither a replayed 14-request sequence nor a motor
output qualification. It never changes input/request timestamps or raw results.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime"))
from singularitydog_hw import feedback_reuse_cadence as cadence


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def _distribution(values):
    if not values:
        return None
    return {"count": len(values), "minimum": min(values),
            "median": statistics.median(values), "maximum": max(values)}


def _recorded_context(report):
    """Retain recorded identities and source pins, never invent their values.

    transport_generation is one *replay file namespace*, explicitly not a
    recovered current hardware descriptor generation. This function is used
    only for independent historical projections whose output_allowed is false.
    """
    provenance = report["source_provenance"]
    if provenance["source_files_unchanged"] is not True:
        raise ValueError("Recorded source was not stable")
    source = provenance["cadence_source_sha256"]
    if type(source) is not dict or not source:
        raise ValueError("Recorded source pins missing")
    for key, value in source.items():
        if type(key) is not str:
            raise ValueError("Invalid recorded source name")
        cadence._sha(value, "recorded source SHA")
    identities = report["identities"]
    if type(identities) is not dict or set(identities) != set(cadence.BUSES):
        raise ValueError("Recorded identities missing")
    uids = {}
    for bus, ids in cadence.BUSES.items():
        batches = identities[bus]
        if type(batches) is not list:
            raise ValueError("Recorded identity batches missing")
        for batch in batches:
            for row in batch["records"]:
                tx, rx = bytes.fromhex(row["tx_hex"]), bytes.fromhex(row["rx_hex"])
                tx_id, rx_id = cadence._wire_id(tx), cadence._wire_id(rx)
                mid = tx_id & 255
                for name in ("start_ns", "finish_ns", "received_ns", "deadline_ns"):
                    cadence._integer(row[name], "recorded identity " + name, minimum=1)
                if (mid not in ids or mid in uids or tx_id != cadence.HOST_ID << 8 | mid or
                        tx[7:15] != bytes(8) or rx_id != mid << 8 | 0xFE or
                        type(row["written"]) is not int or type(row["received"]) is not int or
                        row["written"] != 17 or row["received"] != 17 or
                        not 0 < row["start_ns"] <= row["finish_ns"] <= row["received_ns"] < row["deadline_ns"]):
                    raise ValueError("Invalid recorded identity pairing")
                uids[mid] = rx[7:15].hex()
    if set(uids) != set(cadence.IDS):
        raise ValueError("Incomplete recorded all-axis UID set")
    return cadence.FeedbackContext(report["boot_id"], report["motor_power_epoch"],
        _digest(source), report["plan"]["active_fk_profile"]["sha256"],
        _digest(identities), tuple((i, uids[i]) for i in cadence.IDS), 1)


def compare(records, report, *, work_to_last_write_budget_ns):
    """Pure evaluation; each previous/current pair is an independent scenario."""
    if type(records) is not list or len(records) < 2 or type(report) is not dict:
        raise ValueError("Original records and measurement report required")
    timings = report["measurements"]
    if type(timings) is not list or not 2 <= len(timings) <= len(records):
        raise ValueError("At least two complete timing measurements required")
    cadence._integer(work_to_last_write_budget_ns, "declared remaining budget",
                     minimum=1, maximum=cadence.PERIOD_NS)
    if report["cycles_completed"] != len(timings):
        raise ValueError("Completed-cycle count differs from original measurements")
    context = _recorded_context(report)
    snapshots = []
    for index, cycle in enumerate(records, 1):
        if cycle["cycle"] != index:
            raise ValueError("Unordered original cycle records")
        if index > len(timings):
            # Keep the original file and its SHA/count. An aborted partial
            # exchange need not decode as a full snapshot and is never promoted
            # into this complete-prefix projection or silently made successful.
            continue
        outputs = cycle["output"]
        snapshots.append(cadence.freeze_type2_snapshot(
            {bus: outputs[bus]["records"] for bus in cadence.BUSES},
            context=context, generation=index,
            published_ns=max(r["received_ns"] for bus in cadence.BUSES
                             for r in outputs[bus]["records"])))
    rows = []
    counts = {"REUSE_14": 0, "REFRESH_26": 0, "BLOCK": 0}
    original_time_misses = 0
    required_mode = 0 if report["mode"] == "stop-proxy" else 2
    for index in range(1, len(timings)):
        original = records[index]
        timing = timings[index]
        sample = original["imu"]
        imu = cadence.IMUInterval(sample["read_started_monotonic_ns"],
            sample["read_finished_monotonic_ns"], sample["sequence"],
            tuple(sample["accel_m_s2"]), tuple(sample["gyro_rad_s"]))
        release = timing["release_ns"]
        cadence._integer(release, "recorded release", minimum=1)
        now = max(release, imu.read_finished_ns)
        previous = snapshots[index - 1]
        actual_write = max(r.write_finished_ns for r in snapshots[index].samples)
        if actual_write != timing["final_host_write_ns"]:
            raise ValueError("Timing final write differs from original output records")
        old_source = min(imu.read_started_ns, *(r.request_ns for r in previous.samples))
        retained_age = actual_write - old_source
        original_time_miss = retained_age > cadence.INPUT_MAX_AGE_NS
        original_time_misses += int(original_time_miss)
        decision = cadence.select_feedback_acquisition(previous, context=context,
            release_ns=release, now_ns=now, imu=imu,
            work_to_last_write_budget_ns=work_to_last_write_budget_ns,
            last_consumed_generation=index - 1,
            previous_imu_sequence=records[index - 1]["imu"]["sequence"],
            bootstrap_complete=True, opt_in=True, required_mode=required_mode)
        counts[decision.mode] += 1
        rows.append({"previous_cycle": index, "current_cycle": index + 1,
            "original_output_feedback_request_ns": min(r.request_ns for r in previous.samples),
            "original_time_input_to_last_write_ms": retained_age / 1e6,
            "original_time_20ms_missed": original_time_miss,
            "conditional_budget_decision": decision.mode,
            "conditional_budget_reasons": list(decision.reasons),
            "conditional_remaining_budget_ns": decision.remaining_budget_ns,
            "conditional_projected_final_write_ns": decision.projected_final_write_ns,
            "original_absolute_reuse_final_write_deadline_ns": min(release + cadence.PERIOD_NS,
                decision.input_source_deadline_ns or release + cadence.PERIOD_NS),
            "conditional_projection_is_hardware_measurement": False})
    return {"schema": "singularitydog.offline-feedback-reuse-comparison.v1",
        "status": "OFFLINE_CONDITIONAL_PROJECTION_ONLY", "output_allowed": False,
        "approved_for_runtime": False, "hardware_opened": False,
        "hardware_14_request_path_measured": False, "motor_enable_sent": False,
        "learned_targets_sent": False, "original_timestamps_changed": False,
        "original_report_status": report["status"],
        "recorded_raw_cycles_retained": len(records), "complete_measurements_retained": len(timings),
        "incomplete_raw_cycles_not_promoted": len(records) - len(timings),
        "incomplete_cycle_numbers_retained_not_evaluated": [r["cycle"] for r in records[len(timings):]],
        "independent_transitions_evaluated": len(rows),
        "original_time_reuse_20ms_misses": original_time_misses,
        "original_time_reuse_input_to_write_ms": _distribution([
            row["original_time_input_to_last_write_ms"] for row in rows]),
        "declared_remaining_work_to_last_write_budget_ns": work_to_last_write_budget_ns,
        "declared_budget_was_measured_on_14_request_path": False,
        "independent_conditional_decision_counts": counts,
        "steady_14_request_50hz_sequence_validated": False,
        "transport_generation_scope": "one historical replay file namespace, not a recovered hardware generation",
        "projection_scope": "independent previous-late/current-earlier transition; not a chained 14-request execution",
        "limitations": ["Host request start and receive intervals do not identify motor sensor sample time.",
            "Each admitted projection assumes the declared remaining work budget; that different path was not executed.",
            "Separate voltage, watchdog, calibrated angle/velocity/torque/temperature and final STOP checks remain mandatory.",
            "A fixed 20ms period with six 0.9ms spaced writes cannot sustain same-phase previous-output reuse indefinitely."],
        "transitions": rows}


def _read_json(path):
    before = path.lstat()
    maximum = 32 * 1024 * 1024
    if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= maximum:
        raise ValueError("Expected a nonempty regular JSON file of at most 32 MiB")
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0))
    try:
        pinned = os.fstat(fd)
        fingerprint = lambda value: (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns)
        if not stat.S_ISREG(pinned.st_mode) or fingerprint(pinned) != fingerprint(before):
            raise ValueError("Original input file changed before reading")
        with os.fdopen(fd, "rb", closefd=False) as stream:
            raw = stream.read(maximum + 1)
        if len(raw) != pinned.st_size or fingerprint(os.fstat(fd)) != fingerprint(pinned):
            raise ValueError("Original input changed while reading")
    finally:
        os.close(fd)
    return raw, json.loads(raw, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--work-to-last-write-budget-ns", type=int, required=True,
                        help="Explicit conditional budget, not an observed shortened-path runtime")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        raw_records, records = _read_json(args.records)
        raw_report, report = _read_json(args.report)
        result = compare(records, report,
                         work_to_last_write_budget_ns=args.work_to_last_write_budget_ns)
        result["source_records_sha256"] = hashlib.sha256(raw_records).hexdigest()
        result["source_report_sha256"] = hashlib.sha256(raw_report).hexdigest()
        rendered = json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
        if args.output:
            with args.output.open("x") as stream:
                stream.write(rendered)
        else:
            print(rendered, end="")
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
