"""Offline six-face calibration candidates; never opens or configures hardware.

Labels x+, x-, y+, y-, z+, z- describe expected sensor-frame specific force:
x+ means sensor +X points upwards at rest, and x- means sensor +X points down.
They are operator assertions, not verified robot/body orientation labels.

API: estimate_six_face({label: sequence_of_imu_dicts}) -> candidate dictionary.
File API: calibrate_jsonl_files({label: path}, new_output_path). JSONL can contain
other diagnostic events; only kind='imu' or untagged IMU records are used.
Bare JSONL is diagnostic input; it does not establish range readback, raw/SI
consistency, complete capture, unchanged trim, or successful restoration.
calibrate_capture_directories audits all those fields in imu_capture directories.
Six fresh --validation-capture-face inputs validate fixed fit coefficients;
none of their samples participate in fitting.

CLI (all six --face arguments are mandatory):
  python3 -m singularitydog_hw.imu_calibration --face x+=xplus.jsonl ... \
      --face z-=zminus.jsonl --output new-candidate.json

Default acceptance heuristics are reported in each candidate: >=100 samples,
>=2 s per face; final25% held out; accel std<=0.03g per axis; gyro std<=0.03rad/s
per axis; mean gyro norm<=0.15rad/s; |accel bias_i|<=0.20g; scale in[0.8,1.25];
face mean angle<=10deg. Held-out norm RMS error<=0.03g, max<=0.10g; full-vector
RMS error<=0.20g and max<=0.30g. These are diagnostic gates, not safety limits.
No constant gyro value can establish absence of constant rotation. No result
is activated, loaded by the live driver, or claimed physically validated.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
from typing import Mapping

from . import imu_fixed_mount_baseline as baseline


GRAVITY = 9.80665
FACES = ("x+", "x-", "y+", "y-", "z+", "z-")
_AXIS = {"x": 0, "y": 1, "z": 2}


class CalibrationError(ValueError):
    """Input or candidate did not satisfy the offline acceptance gates."""


@dataclass(frozen=True)
class CalibrationLimits:
    min_samples: int = 100
    min_duration_s: float = 2.0
    max_gap_s: float = 0.25
    holdout_fraction: float = 0.25
    accel_std_max_g: float = 0.03
    gyro_std_max_rad_s: float = 0.03
    gyro_mean_max_rad_s: float = 0.15
    accel_bias_max_g: float = 0.20
    accel_scale_min: float = 0.80
    accel_scale_max: float = 1.25
    orientation_max_deg: float = 10.0
    norm_rms_error_max_g: float = 0.03
    norm_abs_error_max_g: float = 0.10
    vector_rms_error_max_g: float = 0.20
    vector_abs_error_max_g: float = 0.30
    gyro_residual_mean_max_rad_s: float = 0.03

    def __post_init__(self):
        if type(self.min_samples) is not int or self.min_samples < 40:
            raise ValueError("min_samples must be an integer >=40")
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError("%s must be finite and positive" % name)
        if not 0.1 <= self.holdout_fraction <= 0.4:
            raise ValueError("holdout_fraction must be between0.1 and0.4")
        if not self.accel_scale_min <= 1.0 <= self.accel_scale_max:
            raise ValueError("accel scale bounds must include1.0")
        if self.orientation_max_deg >= 45:
            raise ValueError("orientation_max_deg must be less than45")


def _norm(values):
    return math.hypot(*values)


def _mean(vectors):
    return [statistics.fmean(v[i] for v in vectors) for i in range(3)]


def _std(vectors):
    return [statistics.pstdev(v[i] for v in vectors) for i in range(3)]


def _rms(values):
    return math.sqrt(statistics.fmean(v * v for v in values))


def _digest(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True, allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def _number(value, description):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationError(description + " must be numeric")
    if not math.isfinite(value):
        raise CalibrationError(description + " must be finite")
    return float(value)


def _vector(record, field, label):
    value = record.get(field)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise CalibrationError("%s: %s must contain three values" % (label, field))
    return [_number(x, "%s: %s" % (label, field)) for x in value]


def _validate_face(label, records, limits):
    if not isinstance(records, (list, tuple)) or len(records) < limits.min_samples:
        raise CalibrationError("%s: insufficient samples; require%d" % (label, limits.min_samples))
    rows = []
    for index, record in enumerate(records):
        if not isinstance(record, Mapping):
            raise CalibrationError("%s: sample%d is not an object" % (label, index))
        if record.get("frame") != "sensor":
            raise CalibrationError("%s: original sensor frame is required" % label)
        for field in ("calibration_applied", "orientation_applied", "mount_rotation_applied",
                      "accel_bias_subtracted", "accel_scale_corrected", "gyro_bias_subtracted"):
            if field in record and record[field] is not False:
                raise CalibrationError("%s: %s must be explicitly false when present" % (label, field))
        timestamp = record.get("monotonic_ns")
        if type(timestamp) is not int or timestamp <= 0:
            raise CalibrationError("%s: monotonic_ns must be a positive integer" % label)
        acceleration = _vector(record, "accel_m_s2", label)
        gyro = _vector(record, "gyro_rad_s", label)
        # Reject gross values before squaring/statistical operations can overflow.
        if max(map(abs, acceleration)) > 4 * GRAVITY or max(map(abs, gyro)) > 10:
            raise CalibrationError("%s: unreasonable measured magnitude" % label)
        raw = {}
        for field in ("raw_accel", "raw_gyro"):
            if field in record:
                vector = record[field]
                if (not isinstance(vector, (list, tuple)) or len(vector) != 3 or
                        any(type(v) is not int or not -32768 <= v <= 32767 for v in vector)):
                    raise CalibrationError("%s: invalid signed16 %s" % (label, field))
                raw[field] = list(vector)
        wall = record.get("wall_time_ns")
        if wall is not None and (type(wall) is not int or wall <= 0):
            raise CalibrationError("%s: wall_time_ns must be a positive integer" % label)
        rows.append({"time": timestamp, "wall": wall, "accel": acceleration,
                     "gyro": gyro, **raw})
    gaps = [(b["time"] - a["time"]) / 1e9 for a, b in zip(rows, rows[1:])]
    if min(gaps) <= 0:
        raise CalibrationError("%s: monotonic timing is not strictly increasing" % label)
    if max(gaps) > limits.max_gap_s:
        raise CalibrationError("%s: excessive sample timing gap" % label)
    duration = (rows[-1]["time"] - rows[0]["time"]) / 1e9
    if duration < limits.min_duration_s:
        raise CalibrationError("%s: insufficient duration" % label)
    held_count = max(10, int(len(rows) * limits.holdout_fraction))
    train, holdout = rows[:-held_count], rows[-held_count:]
    if len(train) < 20 or len(holdout) < 10:
        raise CalibrationError("%s: insufficient split sample count" % label)
    if (train[-1]["time"] - train[0]["time"]) / 1e9 < limits.min_duration_s * 0.5:
        raise CalibrationError("%s: insufficient training duration" % label)
    if (holdout[-1]["time"] - holdout[0]["time"]) / 1e9 < limits.min_duration_s * 0.1:
        raise CalibrationError("%s: insufficient held-out duration" % label)
    all_accel, all_gyro = [r["accel"] for r in rows], [r["gyro"] for r in rows]
    accel_std, gyro_std, gyro_mean = _std(all_accel), _std(all_gyro), _mean(all_gyro)
    if max(accel_std) > limits.accel_std_max_g * GRAVITY:
        raise CalibrationError("%s: acceleration variation fails stationary gate" % label)
    if max(gyro_std) > limits.gyro_std_max_rad_s:
        raise CalibrationError("%s: gyro variation fails stationary gate" % label)
    if _norm(gyro_mean) > limits.gyro_mean_max_rad_s:
        raise CalibrationError("%s: gyro mean is too large for a bias candidate" % label)
    for partition, subset in (("training", train), ("held-out", holdout)):
        if max(_std([r["accel"] for r in subset])) > limits.accel_std_max_g * GRAVITY:
            raise CalibrationError("%s: %s acceleration variation fails stationary gate" % (label, partition))
        if max(_std([r["gyro"] for r in subset])) > limits.gyro_std_max_rad_s:
            raise CalibrationError("%s: %s gyro variation fails stationary gate" % (label, partition))
    acceleration_mean = _mean(all_accel)
    axis, sign = _AXIS[label[0]], (1 if label[1] == "+" else -1)
    others = [abs(acceleration_mean[i]) for i in range(3) if i != axis]
    if sign * acceleration_mean[axis] < 0.5 * GRAVITY or sign * acceleration_mean[axis] <= max(others):
        raise CalibrationError("%s: expected axis does not dominate with the labeled sign" % label)
    fingerprint = _digest([
        {key: value for key, value in row.items() if key not in ("time", "wall")}
        for row in rows
    ])
    # A second fingerprint also detects copied SI samples with changed raw fields.
    measurement_fingerprint = _digest([[r["accel"], r["gyro"]] for r in rows])
    stats = {
        "samples": len(rows), "duration_s": duration,
        "training_samples": len(train), "heldout_samples": len(holdout),
        "monotonic_start_ns": rows[0]["time"], "monotonic_end_ns": rows[-1]["time"],
        "maximum_gap_s": max(gaps),
        "accel_mean_m_s2": acceleration_mean, "accel_std_m_s2": accel_std,
        "gyro_mean_rad_s": gyro_mean, "gyro_std_rad_s": gyro_std,
        "measurement_sequence_sha256": measurement_fingerprint,
        "raw_and_measurement_sequence_sha256": fingerprint,
        "raw_counts_present": all("raw_accel" in r and "raw_gyro" in r for r in rows),
    }
    return rows, train, holdout, stats


def _validate_interval_provenance(face_rows):
    # Wall timestamps can distinguish captures spanning different host boots.
    # Otherwise monotonic timestamps must come from one boot/timebase.
    wall_complete = all(r["wall"] is not None for rows in face_rows.values() for r in rows)
    clock = "wall" if wall_complete else "time"
    intervals = []
    for label, rows in face_rows.items():
        timestamps = [r[clock] for r in rows]
        if clock == "wall" and any(b <= a for a, b in zip(timestamps, timestamps[1:])):
            raise CalibrationError("%s: wall timing must increase when used for provenance" % label)
        intervals.append((min(timestamps), max(timestamps), label))
    intervals.sort()
    for previous, following in zip(intervals, intervals[1:]):
        if following[0] <= previous[1]:
            raise CalibrationError("overlapping/reused acquisition intervals: %s and%s" %
                                   (previous[2], following[2]))
    return "wall_time_ns" if wall_complete else "monotonic_ns_single_timebase"


def _evaluate(label, rows, bias, scale, gyro_bias, limits):
    axis, sign = _AXIS[label[0]], (1 if label[1] == "+" else -1)
    expected = [0.0, 0.0, 0.0]
    expected[axis] = sign * GRAVITY
    corrected = [[(r["accel"][i] - bias[i]) * scale[i] for i in range(3)] for r in rows]
    corrected_mean = _mean(corrected)
    cosine = max(-1.0, min(1.0, sign * corrected_mean[axis] / _norm(corrected_mean)))
    angle = math.degrees(math.acos(cosine))
    norm_errors = [_norm(v) - GRAVITY for v in corrected]
    vector_errors = [_norm([v[i] - expected[i] for i in range(3)]) for v in corrected]
    norm_rms, norm_max = _rms(norm_errors), max(map(abs, norm_errors))
    vector_rms, vector_max = _rms(vector_errors), max(vector_errors)
    gyro_residual_mean = [v - b for v, b in zip(_mean([r["gyro"] for r in rows]), gyro_bias)]
    if angle > limits.orientation_max_deg:
        raise CalibrationError("%s: corrected orientation differs from labeled face" % label)
    if norm_rms > limits.norm_rms_error_max_g * GRAVITY or norm_max > limits.norm_abs_error_max_g * GRAVITY:
        raise CalibrationError("%s: corrected gravity norm validation failed" % label)
    if vector_rms > limits.vector_rms_error_max_g * GRAVITY or vector_max > limits.vector_abs_error_max_g * GRAVITY:
        raise CalibrationError("%s: corrected full-vector validation failed" % label)
    if _norm(gyro_residual_mean) > limits.gyro_residual_mean_max_rad_s:
        raise CalibrationError("%s: gyro bias is inconsistent across faces or time" % label)
    return {
        "samples": len(rows), "duration_s": (rows[-1]["time"] - rows[0]["time"]) / 1e9,
        "corrected_accel_mean_m_s2": corrected_mean,
        "expected_accel_m_s2": expected, "orientation_error_deg": angle,
        "corrected_norm_mean_m_s2": statistics.fmean(_norm(v) for v in corrected),
        "norm_rms_error_m_s2": norm_rms, "norm_max_abs_error_m_s2": norm_max,
        "vector_rms_error_m_s2": vector_rms, "vector_max_error_m_s2": vector_max,
        "gyro_residual_mean_rad_s": gyro_residual_mean,
    }


def estimate_six_face(face_datasets, *, provenance=None, limits=None):
    """Fit candidate using first75%, validate the untouched last25% of each face.

    This function returns numbers only. It does not mutate input samples,
    install calibration, normalize live readings, or infer physical poses.
    """
    limits = limits or CalibrationLimits()
    if not isinstance(limits, CalibrationLimits):
        raise TypeError("limits must be CalibrationLimits")
    if not isinstance(face_datasets, Mapping) or set(face_datasets) != set(FACES):
        raise CalibrationError("exactly six distinct faces required: " + ", ".join(FACES))
    provenance = provenance or {}
    rows, train, heldout, stats = {}, {}, {}, {}
    seen_sequences, seen_files = set(), set()
    for label in FACES:
        rows[label], train[label], heldout[label], stats[label] = _validate_face(label, face_datasets[label], limits)
        digest = stats[label]["measurement_sequence_sha256"]
        if digest in seen_sequences:
            raise CalibrationError("%s: reused measurement dataset" % label)
        seen_sequences.add(digest)
        file_hash = provenance.get(label, {}).get("sha256")
        if file_hash:
            if file_hash in seen_files:
                raise CalibrationError("%s: reused input content hash" % label)
            seen_files.add(file_hash)
    interval_clock = _validate_interval_provenance(rows)
    train_means = {label: _mean([r["accel"] for r in train[label]]) for label in FACES}
    bias, scale, spans = [], [], []
    for axis_name, axis in _AXIS.items():
        positive, negative = train_means[axis_name + "+"][axis], train_means[axis_name + "-"][axis]
        span = positive - negative
        if span < 2 * GRAVITY / limits.accel_scale_max or span > 2 * GRAVITY / limits.accel_scale_min:
            raise CalibrationError("%s: poor acceleration span or unreasonable scale correction" % axis_name)
        b, s = (positive + negative) / 2, 2 * GRAVITY / span
        if abs(b) > limits.accel_bias_max_g * GRAVITY:
            raise CalibrationError("%s: unreasonable acceleration bias correction" % axis_name)
        bias.append(b)
        scale.append(s)
        spans.append(span)
    gyro_bias = _mean([r["gyro"] for label in FACES for r in train[label]])
    if _norm(gyro_bias) > limits.gyro_mean_max_rad_s:
        raise CalibrationError("unreasonable gyro bias correction")
    training_validation, heldout_validation = {}, {}
    for label in FACES:
        training_validation[label] = _evaluate(label, train[label], bias, scale, gyro_bias, limits)
        try:
            heldout_validation[label] = _evaluate(label, heldout[label], bias, scale, gyro_bias, limits)
        except CalibrationError as error:
            raise CalibrationError("held-out " + str(error)) from error
    return {
        "schema_version": 1,
        "status": "candidate",
        "requires_physical_validation": True,
        "approved_for_runtime": False,
        "automatically_applied": False,
        "capture_audit_verified": False,
        "frame": "sensor",
        "axis_order": ["x", "y", "z"],
        "gravity_reference_m_s2": GRAVITY,
        "accel": {"bias_m_s2": bias, "scale": scale, "measured_spans_m_s2": spans,
                  "formula": "corrected[i] = (measured[i] - bias_m_s2[i]) * scale[i]"},
        "gyro": {"bias_rad_s": gyro_bias,
                 "formula": "corrected[i] = measured[i] - bias_rad_s[i]",
                 "scale_estimated": False},
        "face_label_convention": "x+ means sensor +X points upward at rest; x- means +X downward; likewise Y/Z. Specific force includes gravity.",
        "fit_partition": "chronological first samples; final holdout_fraction excluded from fit",
        "limits": asdict(limits),
        "faces": stats,
        "provenance": provenance,
        "acquisition_interval_clock": interval_clock,
        "validation": {"training": training_validation, "heldout": heldout_validation,
                       "heldout_is_independent_physical_validation": False},
        "limitations": [
            "Six physical face labels and stationary setup are operator assertions.",
            "Small variance and constant gyro bias cannot prove absence of constant rotation or acceleration.",
            "Chronological held-out samples share captures, hardware and pose errors with training samples.",
            "Only diagonal acceleration scale and bias are estimated; misalignment/cross-axis effects and temperature response are not modeled.",
            "Gate thresholds are diagnostic acceptance heuristics, not validated hardware safety limits.",
            "Hashes detect exact reuse, not fabricated measurements; raw input files remain unchanged.",
        ],
    }


def _load_jsonl(path):
    content = Path(path).read_bytes()
    records = []
    try:
        lines = content.decode("utf-8").splitlines()
    except UnicodeDecodeError as error:
        raise CalibrationError("input JSONL must be UTF-8") from error
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            record = baseline._json(line)
        except (ValueError, RecursionError) as error:
            raise CalibrationError("invalid JSONL at line%d" % line_number) from error
        if not isinstance(record, dict):
            raise CalibrationError("JSONL line%d is not an object" % line_number)
        if record.get("kind") == "imu" or "kind" not in record:
            records.append(record)
        elif record.get("kind") == "imu_configured":
            configuration = record.get("configuration", {})
            if (not isinstance(configuration, dict) or configuration.get("frame") != "sensor" or
                    configuration.get("orientation_applied") is not False or
                    any(field in configuration and configuration[field] is not False
                        for field in ("calibration_applied", "mount_rotation_applied",
                                      "accel_bias_subtracted", "accel_scale_corrected", "gyro_bias_subtracted"))):
                raise CalibrationError("configuration event does not establish original sensor frame")
    return records, {"path": str(Path(path).resolve()), "sha256": hashlib.sha256(content).hexdigest(),
                     "bytes": len(content), "imu_records": len(records)}


def calibrate_jsonl_files(face_paths, output_path, *, limits=None):
    """Read six logs and exclusively create one candidate JSON; never overwrite."""
    if not isinstance(face_paths, Mapping) or set(face_paths) != set(FACES):
        raise CalibrationError("exactly six distinct labeled paths required")
    output = Path(output_path)
    if output.exists() or output.is_symlink():
        raise FileExistsError("output already exists: " + str(output))
    datasets, provenance = {}, {}
    for label in FACES:
        datasets[label], provenance[label] = _load_jsonl(face_paths[label])
    # Check byte-identical files before inspecting their assumed face labels.
    if len({v["sha256"] for v in provenance.values()}) != len(FACES):
        raise CalibrationError("reused input content hash across labeled faces")
    result = estimate_six_face(datasets, provenance=provenance, limits=limits)
    result["input_audit"] = "JSONL measurements only; capture metadata and restoration not audited"
    return _write_candidate(result, output)


def _write_candidate(result, output):
    output = Path(output).expanduser().resolve()
    if any((parent / ".git").exists() for parent in (output.parent, *output.parents)):
        raise CalibrationError("calibration output must be outside Git")
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    serialized = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    # No parent-directory creation, overwrite, auto-install, or git operation.
    descriptor = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                         getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(serialized)
    except BaseException:
        # Remove only the new file created by this call if writing failed.
        output.unlink(missing_ok=True)
        raise
    return result


def calibrate_capture_directories(face_paths, output_path, *, validation_face_paths=None,
                                  operator_confirmed_stationary=False, limits=None):
    output = Path(output_path).expanduser()
    if output.exists() or output.is_symlink():
        raise FileExistsError("output already exists: " + str(output))
    result = estimate_capture_directories(face_paths,
        validation_face_paths=validation_face_paths,
        operator_confirmed_stationary=operator_confirmed_stationary, limits=limits)
    return _write_candidate(result, output)


def estimate_capture_directories(face_paths, *, validation_face_paths=None,
                                 operator_confirmed_stationary=False, limits=None):
    """Fit audited faces, then check six separately acquired faces without refit.

    Face labels and stationarity still require operator assertions. Neither an
    audited fit nor a separate validation set grants runtime approval.
    """
    if operator_confirmed_stationary is not True:
        raise CalibrationError("all face captures require operator-confirmed stationarity")
    if not isinstance(face_paths, Mapping) or set(face_paths) != set(FACES):
        raise CalibrationError("exactly six distinct labeled capture directories required")
    if validation_face_paths is not None and (not isinstance(validation_face_paths, Mapping)
                                              or set(validation_face_paths) != set(FACES)):
        raise CalibrationError("independent validation requires six distinct labeled capture directories")
    limits = limits or CalibrationLimits()
    if not isinstance(limits, CalibrationLimits):
        raise TypeError("limits must be CalibrationLimits")
    datasets, provenance, validation, validation_provenance = {}, {}, {}, {}
    reference = None
    seen_paths, seen_events, seen_sequences = set(), set(), set()
    all_rows = {}
    for partition, paths, destination, origins in (
            ("fit", face_paths, datasets, provenance),
            ("independent", validation_face_paths or {}, validation, validation_provenance)):
        for label in FACES:
            if label not in paths:
                continue
            meta, rows, stats, origin = baseline._load_capture(paths[label], expected_face=label)
            if reference is None:
                reference = meta
            for key in ("source_sha256", "configuration", "register_audit_before"):
                if meta[key] != reference[key]:
                    raise CalibrationError("capture setup mismatch: " + key)
            if any(meta["plan"][key] != reference["plan"][key] for key in ("bus", "address")):
                raise CalibrationError("capture device mismatch")
            for key, seen in (("directory", seen_paths), ("events_sha256", seen_events),
                              ("measurement_sequence_sha256", seen_sequences)):
                if origin[key] in seen:
                    raise CalibrationError("reused capture or measurement sequence: " + key)
                seen.add(origin[key])
            temp = stats["temperature_c"]
            if temp["max"] - temp["min"] > baseline.LIMITS["temperature_span_max_c"]:
                raise CalibrationError("%s %s: temperature not stable" % (partition, label))
            if abs(stats["first_to_last_quarter"]["temperature_mean_change_c"]) > baseline.LIMITS["temperature_drift_max_c"]:
                raise CalibrationError("%s %s: temperature drift" % (partition, label))
            if abs(temp["mean"] - reference["summary"]["temperature_c"]["mean"]) > baseline.LIMITS["temperature_mean_change_max_c"]:
                raise CalibrationError("%s %s: temperature differs from fit" % (partition, label))
            destination[label] = rows
            origins[label] = {**origin, "sha256": origin["events_sha256"],
                              "temperature_c": temp}
            # Fit and independent acquisition intervals must be separate.
            # Timing within each capture has already passed the input audit.
            all_rows[partition + " " + label] = _validate_face(label, rows, limits)[0]
    clock = _validate_interval_provenance(all_rows)
    result = estimate_six_face(datasets, provenance=provenance, limits=limits)
    result["capture_audit_verified"] = True
    result["input_audit"] = "complete imu_capture directories; range readback, raw/SI, trim, timing, source and restoration checked"
    result["operator_confirmed_stationary"] = True
    result["all_capture_interval_clock"] = clock
    result["capture_configuration"] = reference["configuration"]
    if validation_face_paths is not None:
        checked = {}
        for label in FACES:
            # The fixed fit is evaluated against every independent sample.
            # Even a passing chronological split cannot replace this set.
            parsed_rows = all_rows["independent " + label]
            try:
                checked[label] = _evaluate(label, parsed_rows,
                    result["accel"]["bias_m_s2"], result["accel"]["scale"],
                    result["gyro"]["bias_rad_s"], limits)
            except CalibrationError as error:
                raise CalibrationError("independent " + str(error)) from error
        result["validation"]["independent_captures"] = checked
        result["validation"]["independent_provenance"] = validation_provenance
        result["validation"]["independent_capture_gates_passed"] = True
        result["validation"]["independent_captures_used_for_fit"] = False
        result["limitations"].append("Separate captures still share operator face/reference errors; the instrument accuracy and physical mount must be reviewed.")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--face", action="append", metavar="LABEL=JSONL",
                        help="repeat once for each x+,x-,y+,y-,z+,z- sensor face")
    inputs.add_argument("--capture-face", action="append", metavar="LABEL=DIRECTORY",
                        help="six imu_capture directories; audit complete captures before fitting")
    parser.add_argument("--validation-capture-face", action="append", metavar="LABEL=DIRECTORY",
                        help="six fresh captures not used for fitting; requires --capture-face")
    parser.add_argument("--operator-confirmed-stationary", action="store_true",
                        help="operator confirms every fit/validation face was stationary and correctly labeled")
    parser.add_argument("--output", required=True, help="new candidate JSON path; existing paths are rejected")
    args = parser.parse_args(argv)
    def labeled_paths(items):
        result = {}
        for item in items or []:
            label, separator, path = item.partition("=")
            if not separator or label not in FACES or not path or label in result:
                parser.error("each face input must be one distinct LABEL=PATH using " + ", ".join(FACES))
            result[label] = path
        return result
    face_paths = labeled_paths(args.capture_face or args.face)
    validation_paths = labeled_paths(args.validation_capture_face) if args.validation_capture_face else None
    if args.validation_capture_face and not args.capture_face:
        parser.error("--validation-capture-face requires --capture-face")
    try:
        if args.capture_face:
            result = calibrate_capture_directories(face_paths, args.output,
                validation_face_paths=validation_paths,
                operator_confirmed_stationary=args.operator_confirmed_stationary)
        else:
            result = calibrate_jsonl_files(face_paths, args.output)
    except (OSError, CalibrationError, ValueError) as error:
        parser.exit(2, "Calibration rejected: %s\n" % error)
    print(json.dumps({"output": str(Path(args.output).resolve()), "status": result["status"],
                      "capture_audit_verified": result["capture_audit_verified"],
                      "independent_capture_gates_passed": result["validation"].get("independent_capture_gates_passed", False),
                      "requires_physical_validation": True, "approved_for_runtime": False,
                      "automatically_applied": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
