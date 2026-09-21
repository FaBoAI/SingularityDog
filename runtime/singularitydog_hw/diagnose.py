"""Bounded IMU + RS05 telemetry recorder. This CLI cannot enable motors."""
import argparse
from dataclasses import asdict
import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import threading
import time

from .can_readonly import ReadOnlyCAN, PARAMETERS
from .imu import ICM20948
from .safety import GuardConfig, SafetyGuard, SafetyInputs

LEG_IDS = {"FR": [1,2,3], "FL": [4,5,6], "RR": [7,8,9], "RL": [10,11,12]}
OPTIONAL_PARAMETERS = frozenset(("can_timeout", "zero_state"))


def distribution(values):
    if not values:
        return None
    values = sorted(values)
    return {"count": len(values), "mean": statistics.fmean(values),
            "std": statistics.pstdev(values), "min": values[0], "max": values[-1],
            "p50": values[int((len(values)-1)*.50)],
            "p95": values[int((len(values)-1)*.95)],
            "p99": values[int((len(values)-1)*.99)]}


def summarize(records):
    imu = [x for x in records if x.get("kind") == "imu"]
    parameters = [x for x in records if x.get("kind") == "motor_parameter"]
    motors = {}
    for mid in range(1,13):
        subset = [x for x in parameters if x["motor_id"] == mid]
        identities = {x["mcu_uid_hex"] for x in subset if x["parameter"] == "identity" and x["ok"]}
        values = {}
        for name in PARAMETERS:
            good = [x for x in subset if x["parameter"] == name and x["ok"]]
            values[name] = {"last": good[-1]["value"] if good else None,
                            "unit": PARAMETERS[name][2],
                            "statistics": distribution([x["value"] for x in good])}
        motors[str(mid)] = {"identities": sorted(identities), "parameters": values,
                            "query_round_trip_ms": distribution([x["round_trip_ms"] for x in subset]),
                            "error_reply_count": sum(not x["ok"] for x in subset)}
    imu_stats = {"samples": len(imu), "frame": "sensor", "mount_rotation_applied": False,
                 "calibration_applied": False, "magnetometer_measured": False}
    if imu:
        intervals = [(b["monotonic_ns"]-a["monotonic_ns"])/1e6 for a,b in zip(imu,imu[1:])]
        elapsed = (imu[-1]["monotonic_ns"]-imu[0]["monotonic_ns"])/1e9
        imu_stats.update(observed_rate_hz=(len(imu)-1)/elapsed if elapsed else None,
                         sample_interval_ms=distribution(intervals),
                         read_duration_ms=distribution([(x["read_finished_monotonic_ns"]-x["read_started_monotonic_ns"])/1e6 for x in imu]),
                         accel_norm_m_s2=distribution([math.sqrt(sum(v*v for v in x["accel_m_s2"])) for x in imu]),
                         accel_mean_m_s2=[statistics.fmean(x["accel_m_s2"][i] for x in imu) for i in range(3)],
                         gyro_mean_rad_s=[statistics.fmean(x["gyro_rad_s"][i] for x in imu) for i in range(3)],
                         gyro_std_rad_s=[statistics.pstdev(x["gyro_rad_s"][i] for x in imu) for i in range(3)],
                         ready_during_read_count=sum(x["data_ready_during_read"] for x in imu))
    return {"imu": imu_stats, "motors": motors,
            "can_timeout_count": sum(x.get("kind")=="can_timeout" for x in records),
            "can_tx_count": sum(x.get("kind")=="can_tx" for x in records),
            "unsolicited_feedback_count": sum(x.get("kind")=="motor_feedback" for x in records)}


def coverage_errors(summary, rejected_optional=()):
    """A bounded run can end normally before it covers the full robot."""
    errors = []
    if summary["imu"]["samples"] < 2:
        errors.append({"component": "coverage", "error": "Fewer than two IMU samples"})
    identities = []
    for mid, row in summary["motors"].items():
        if len(row["identities"]) != 1:
            errors.append({"component": "coverage", "error": f"ID {mid}: missing or inconsistent identity"})
        identities.extend(row["identities"])
        missing = [name for name, value in row["parameters"].items()
                   if value["last"] is None and (int(mid), name) not in rejected_optional]
        if missing:
            errors.append({"component": "coverage", "error": f"ID {mid}: missing {','.join(missing)}"})
    if len(identities) != len(set(identities)):
        errors.append({"component": "coverage", "error": "Duplicate identity across motor IDs"})
    return errors


def optional_read_rejection(result):
    # Only a correctly correlated negative reply can be reported as unavailable.
    # Malformed, timed-out, nonfinite or missing replies remain hard failures.
    return (result["parameter"] in OPTIONAL_PARAMETERS and
            result.get("error") == "parameter_status_nonzero")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute", action="store_true", help="Open hardware and initialize/restore IMU")
    ap.add_argument("--seconds", type=int, default=60)
    ap.add_argument("--output", type=Path, required=True, help="New private directory for logs")
    args = ap.parse_args(argv)
    if not 1 <= args.seconds <= 120:
        ap.error("seconds must be1..120")
    plan = {"motor_model": "RS05", "model_source": "user_confirmed_all12",
            "ids_bottom_to_top": LEG_IDS, "joint_calibration_verified": False,
            "duration_seconds": args.seconds, "imu_poll_hz": 100,
            "can_sweep_target_hz": 2, "can_uart_baud": 921600,
            "motor_output_available": False, "allowed_can_types": [0,17],
            "imu_configuration": "accel2g/gyro250dps, ~102Hz ODR; restore on normal exit",
            "no_automatic_start_or_background_service": True}
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    args.output = args.output.expanduser().resolve()
    # Raw UIDs and camera material do not belong in source control.
    if Path(__file__).resolve().parent.parent in args.output.parents:
        ap.error("Choose a private log directory outside runtime source")
    if any((p / ".git").exists() for p in (args.output, *args.output.parents)):
        ap.error("Raw logs must be outside a Git checkout")
    args.output.mkdir(mode=0o700, parents=True, exist_ok=False)
    records, errors, warnings = [], [], []
    rejected_optional = set()
    stop = threading.Event()
    lock = threading.Lock()
    imu_device = None
    can_thread = None
    started = datetime.datetime.now().astimezone().isoformat()
    meta = {"started_at": started, "plan": plan, "errors": errors, "warnings": warnings,
            "source_sha256": {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in Path(__file__).parent.glob("*.py")}}
    guard = SafetyGuard(GuardConfig({"imu":0.05,"can":1.0},0.1))
    meta["guard_at_start"] = asdict(guard.evaluate(SafetyInputs()))
    previous_handlers = {}
    def interrupted(signum, _frame):
        errors.append({"component":"process","error":f"signal {signum}"})
        stop.set()
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, interrupted)
    log = (args.output / "events.jsonl").open("x", buffering=1)
    def emit(event):
        with lock:
            record = {"wall_time_ns": time.time_ns(), **event}
            log.write(json.dumps(record,allow_nan=False)+"\n")
            records.append(record)

    # Advisory lock coordinates instances of this collector; it cannot lock out other tools.
    imu_lock = None
    try:
        imu_lock = open("/tmp/singularitydog-imu-i2c7-68.lock", "a+")
        fcntl.flock(imu_lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        imu_device = ICM20948()
        meta["imu_configuration"] = imu_device.start()
        meta["imu_original_registers"] = imu_device.original_registers
        emit({"kind":"imu_configured", "monotonic_ns":time.monotonic_ns(),
              "configuration":imu_device.configuration})
        deadline = time.monotonic() + args.seconds
        def collect_can():
            try:
                with ReadOnlyCAN(event_sink=emit) as can:
                    for mid in range(1,13):
                        for name in (None,"run_mode","can_timeout","zero_state"):
                            if stop.is_set() or time.monotonic() >= deadline:
                                return
                            result=can.query(mid,name)
                            if not result["ok"]:
                                if optional_read_rejection(result):
                                    rejected_optional.add((mid, name))
                                    warnings.append({"component":"can_parameter", "motor_id":mid,
                                                     "parameter":name, "status":result["status"],
                                                     "message":"Read rejected by device; setting is unknown"})
                                else:
                                    raise RuntimeError(f"Invalid parameter reply: {mid} {name}")
                    while not stop.is_set() and time.monotonic()<deadline:
                        sweep_started=time.monotonic()
                        for mid in range(1,13):
                            for name in ("position","velocity","current","voltage"):
                                if stop.is_set() or time.monotonic()>=deadline:
                                    return
                                result=can.query(mid,name)
                                if not result["ok"]:
                                    raise RuntimeError(f"Invalid parameter reply: {mid} {name}")
                        stop.wait(max(0, .5-(time.monotonic()-sweep_started)))
            except BaseException as exc:
                errors.append({"component":"can","error":repr(exc)})
                stop.set()
        can_thread=threading.Thread(target=collect_can,name="can-readonly")
        can_thread.start()
        next_poll=time.monotonic()
        last_sample=time.monotonic()
        while not stop.is_set() and time.monotonic()<deadline:
            sample=imu_device.read_sample()
            if sample is not None:
                last_sample=time.monotonic()
                emit({"kind":"imu",**sample})
            elif time.monotonic()-last_sample > .5:
                raise RuntimeError("No fresh IMU sample for0.5s")
            next_poll+=.01
            wait=next_poll-time.monotonic()
            if wait<-.01:
                emit({"kind":"imu_poll_late","monotonic_ns":time.monotonic_ns(),"late_ms":-wait*1000})
                next_poll=time.monotonic()
            stop.wait(max(0,wait))
    except BaseException as exc:
        errors.append({"component":"main_or_imu","error":repr(exc)})
    finally:
        stop.set()
        if can_thread is not None:
            can_thread.join()  # serial operations have bounded timeout/write_timeout
        if imu_device is not None:
            try:
                imu_device.close()
            except BaseException as exc:
                errors.append({"component":"imu_restore","error":repr(exc)})
            meta["imu_restore_status"]=imu_device.restore_status
        if imu_lock is not None:
            imu_lock.close()
        meta["guard_at_end"]=asdict(guard.close())
        for signum,handler in previous_handlers.items():
            signal.signal(signum,handler)
        log.close()
    summary = summarize(records)
    errors.extend(coverage_errors(summary, rejected_optional))
    meta.update(completed_at=datetime.datetime.now().astimezone().isoformat(),
                summary=summary, status=("INCOMPLETE" if errors else
                                         "COMPLETE_WITH_WARNINGS" if warnings else "COMPLETE"))
    (args.output/"summary.json").write_text(json.dumps(meta,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"output":str(args.output),"status":meta["status"],
                      "imu_samples":meta["summary"]["imu"]["samples"],
                      "imu_rate_hz":meta["summary"]["imu"].get("observed_rate_hz"),
                      "motor_ids_with_voltage":[int(mid) for mid,row in meta["summary"]["motors"].items()
                                                if row["parameters"]["voltage"]["last"] is not None],
                      "errors":errors, "warnings":warnings},indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
