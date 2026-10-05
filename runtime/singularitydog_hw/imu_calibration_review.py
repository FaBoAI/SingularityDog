"""Explicitly reviewed diagonal acceleration input; no device or motor I/O.

The optional accel_calibration_review in a pinned gyro-bias JSON references an
unchanged unapproved fit. Every fit/independent capture is re-audited on load.
A named physical review is required, and callers must explicitly select it.
The CLI creates an UNREVIEWED bias extension; it never fills physical answers.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path

from . import imu_calibration as calibration
from . import imu_fixed_mount_baseline as baseline

SCHEMA = "singularitydog.reviewed-accel-calibration.v1"
DECISION = "ACCEPT_DIAGONAL_ACCEL_CALIBRATION_INPUT"
PHYSICAL_KEYS = ("known_faces_and_stationarity_reviewed", "independent_repositioning_reviewed",
                 "sensor_mount_unchanged", "external_reference_accuracy_reviewed")
SOURCE_PATHS = ("imu.py", "imu_calibration.py", "imu_fixed_mount_baseline.py",
                "imu_calibration_review.py", "policy_observer.py", "policy_output_model.py")


def source_hashes():
    root = Path(__file__).resolve().parent
    result = {}
    for name in SOURCE_PATHS:
        path = root / name
        _need(path.is_file() and not path.is_symlink(), "Missing calibration input source: " + name)
        result["singularitydog_hw/" + name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def _need(condition, message):
    if not condition:
        raise ValueError(message)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _text(value, label):
    _need(type(value) is str and 0 < len(value) <= 1024 and value.strip() == value,
          "Missing " + label)


def audit_candidate(reference):
    """Recompute the fixed candidate and its independent checks in memory."""
    _need(type(reference) is dict and set(reference) == {"path", "sha256"},
          "Pinned acceleration candidate reference required")
    _text(reference["path"], "acceleration candidate path")
    path = Path(reference["path"])
    _need(path.is_absolute() and path.is_file() and not path.is_symlink(),
          "Acceleration candidate requires an absolute regular nonsymlink file")
    raw = path.read_bytes()
    _need(hashlib.sha256(raw).hexdigest() == reference["sha256"], "Acceleration candidate SHA256 mismatch")
    candidate = baseline._json(raw)
    validation = candidate.get("validation")
    _need(type(validation) is dict, "Acceleration candidate validation must be an object")
    _need(candidate.get("approved_for_runtime") is False and candidate.get("automatically_applied") is False
          and candidate.get("capture_audit_verified") is True
          and validation.get("independent_capture_gates_passed") is True
          and validation.get("independent_captures_used_for_fit") is False,
          "An unapproved audited fit plus independent captures is required")
    fit = candidate.get("provenance", {})
    check = candidate["validation"].get("independent_provenance", {})
    _need(type(fit) is dict and type(check) is dict and set(fit) == set(check) == set(calibration.FACES),
          "Complete fit and independent capture provenance required")
    _need(all(type(origin) is dict and type(origin.get("directory")) is str
              for origin in (*fit.values(), *check.values())) and type(candidate.get("limits")) is dict,
          "Invalid acceleration capture or limit provenance")
    recomputed = calibration.estimate_capture_directories(
        {face: fit[face]["directory"] for face in calibration.FACES},
        validation_face_paths={face: check[face]["directory"] for face in calibration.FACES},
        operator_confirmed_stationary=candidate.get("operator_confirmed_stationary"),
        limits=calibration.CalibrationLimits(**candidate.get("limits", {})))
    comparable = {key: value for key, value in candidate.items() if key != "generated_at_utc"}
    # Python value equality equates True with 1 and False with 0. Canonical
    # JSON preserves evidence types as well as values during recomputation.
    _need(_digest(comparable) == _digest(recomputed),
          "Acceleration fit/capture/validation changed; recomputation differs")
    # Detect a concurrent candidate replacement during capture re-audit.
    _need(path.read_bytes() == raw, "Acceleration candidate changed during review load")
    return candidate


@dataclass(frozen=True)
class ReviewedAcceleration:
    bias_m_s2: tuple
    scale: tuple
    norm_min_m_s2: float
    norm_max_m_s2: float
    candidate_sha256: str
    review_sha256: str

    def correct(self, acceleration):
        values = [(acceleration[i] - self.bias_m_s2[i]) * self.scale[i] for i in range(3)]
        norm = math.hypot(*values)
        _need(all(math.isfinite(value) for value in values) and
              self.norm_min_m_s2 <= norm <= self.norm_max_m_s2,
              "Corrected acceleration norm outside reviewed range")
        return values, norm

    def provenance(self):
        return {"kind": SCHEMA, "candidate_sha256": self.candidate_sha256,
                "review_canonical_json_sha256": self.review_sha256,
                "bias_sensor_m_s2": list(self.bias_m_s2), "scale_sensor": list(self.scale),
                "corrected_norm_bounds_m_s2": [self.norm_min_m_s2, self.norm_max_m_s2],
                "fit_and_independent_captures_reaudited": True,
                "grants_motor_output": False}


def reviewed_acceleration(bias_candidate, mount_rotation, *, enabled=False):
    """Default raw; explicit selection requires both reviewed fit and sources."""
    _need(type(enabled) is bool, "Acceleration calibration selection must be boolean")
    if not enabled:
        return None
    _need(type(bias_candidate) is dict, "Pinned gyro-bias document required for acceleration review")
    document = bias_candidate.get("accel_calibration_review")
    keys = {"schema", "candidate", "review", "R_body_from_sensor", *PHYSICAL_KEYS,
            "external_reference_uncertainty_rad", "corrected_norm_min_m_s2",
            "corrected_norm_max_m_s2", "source_sha256"}
    _need(type(document) is dict and set(document) == keys and document["schema"] == SCHEMA,
          "Explicit acceleration calibration review required")
    _need(document["source_sha256"] == source_hashes(), "Acceleration input source SHA256 mismatch")
    review = document["review"]
    _need(type(review) is dict and set(review) == {"reviewer", "reviewed_at", "decision", "rationale"},
          "Named acceleration review required")
    _text(review["reviewer"], "acceleration reviewer")
    _text(review["rationale"], "acceleration review rationale")
    _need(review["decision"] == DECISION, "Acceleration review has not accepted this scope")
    try:
        stamp = datetime.fromisoformat(review["reviewed_at"].replace("Z", "+00:00"))
        _need(stamp.utcoffset() is not None, "Acceleration review time requires timezone")
    except (TypeError, ValueError, AttributeError) as error:
        raise ValueError("Invalid acceleration review date") from error
    _need(all(document[key] is True for key in PHYSICAL_KEYS), "Acceleration physical review incomplete")
    matrix = document["R_body_from_sensor"]
    _need(type(matrix) is list and len(matrix) == 3 and all(type(row) is list and len(row) == 3
          and all(type(value) in (int, float) and math.isfinite(value) for value in row) for row in matrix),
          "Acceleration review mount entries must be finite real numbers")
    _need(_digest(matrix) == _digest([list(row) for row in mount_rotation]),
          "Acceleration review mount differs from selected mount")
    uncertainty = document["external_reference_uncertainty_rad"]
    _need(type(uncertainty) in (int, float) and math.isfinite(uncertainty)
          and 0 < uncertainty <= math.radians(3), "Unknown/zero/excessive external reference uncertainty")
    low, high = document["corrected_norm_min_m_s2"], document["corrected_norm_max_m_s2"]
    _need(all(type(value) in (int, float) and math.isfinite(value) for value in (low, high))
          and 8.8 <= low < calibration.GRAVITY < high <= 11.2 and high - low <= 1.5,
          "Invalid corrected acceleration norm bounds")
    candidate = audit_candidate(document["candidate"])
    # Require the reviewed bounds to contain every independent sample.
    for face, origin in candidate["validation"]["independent_provenance"].items():
        _, rows, _, provenance = baseline._load_capture(origin["directory"], expected_face=face)
        _need(all(provenance[key] == origin[key] for key in ("summary_sha256", "events_sha256")),
              "Independent acceleration source changed during review load")
        for row in rows:
            norm = math.hypot(*[(row["accel_m_s2"][i] - candidate["accel"]["bias_m_s2"][i])
                               * candidate["accel"]["scale"][i] for i in range(3)])
            _need(low <= norm <= high, "Independent corrected sample outside reviewed norm bounds")
    for origins in (candidate["provenance"], candidate["validation"]["independent_provenance"]):
        for origin in origins.values():
            directory = Path(origin["directory"])
            _need(all(hashlib.sha256((directory / filename).read_bytes()).hexdigest() == origin[key]
                      for filename, key in (("summary.json", "summary_sha256"), ("events.jsonl", "events_sha256"))),
                  "Acceleration source changed during final review load")
    reference = document["candidate"]
    _need(hashlib.sha256(Path(reference["path"]).read_bytes()).hexdigest() == reference["sha256"],
          "Acceleration candidate changed during final review load")
    _need(document["source_sha256"] == source_hashes(), "Acceleration input source changed during load")
    return ReviewedAcceleration(tuple(candidate["accel"]["bias_m_s2"]),
        tuple(candidate["accel"]["scale"]), float(low), float(high),
        document["candidate"]["sha256"], _digest(document))


def write_template(bias_path, candidate_path, mount_path, output):
    # This preparation checks evidence, but deliberately leaves physical review
    # unanswered. It cannot produce a usable reviewed acceleration extension.
    from .policy_observer import _bias, _mount
    bias = baseline._json(Path(bias_path).read_bytes())
    _bias(bias)
    candidate_path = Path(candidate_path).expanduser().resolve()
    reference = {"path": str(candidate_path), "sha256": hashlib.sha256(candidate_path.read_bytes()).hexdigest()}
    audit_candidate(reference)
    mount = _mount(baseline._json(Path(mount_path).read_bytes()))
    result = copy.deepcopy(bias)
    _need("accel_calibration_review" not in result, "Bias already contains an acceleration review")
    result["accel_calibration_review"] = {"schema": SCHEMA, "candidate": reference,
        "review": {"reviewer": None, "reviewed_at": None, "decision": "UNREVIEWED", "rationale": None},
        "R_body_from_sensor": mount["R_body_from_sensor"],
        "source_sha256": source_hashes(),
        **{key: False for key in PHYSICAL_KEYS}, "external_reference_uncertainty_rad": None,
        "corrected_norm_min_m_s2": None, "corrected_norm_max_m_s2": None}
    target = Path(output).expanduser()
    _need(not target.exists() and not target.is_symlink(), "Review output already exists")
    target = target.resolve()
    _need(not any((p / ".git").exists() for p in (target.parent, *target.parents)), "Review output must be outside Git")
    with os.fdopen(os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600), "w") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("bias", "candidate", "mount", "output"):
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args(argv)
    try:
        write_template(args.bias, args.candidate, args.mount, args.output)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, "Acceleration review preparation rejected: %s\n" % error)
    print(json.dumps({"status": "UNREVIEWED", "approved_for_runtime": False,
                      "motor_output_allowed": False, "output": str(Path(args.output).resolve())}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
