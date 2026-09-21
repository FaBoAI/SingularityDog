"""Offline, private evidence for ID11 replacement; candidate eligibility only.

No device access, angle correction, motor configuration or runtime approval.
"""
import hashlib
import json
import math
from pathlib import Path
import re

from .joint_observation import _strict_pairs
from .pose_record import load_snapshot


FIELDS = {"schema_version", "motor_id", "eligibility", "previous_capture",
          "previous_events_sha256", "replacement_capture", "replacement_events_sha256"}


def _uid(value):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-fA-F]{16}", value):
        raise ValueError("A motor UID must contain exactly 16 hexadecimal characters")
    return value.lower()


def load_id11_replacement(path):
    """Validate complete snapshots and their pinned hashes, without reusing zeros."""
    path = Path(path).expanduser().resolve()
    data = path.read_bytes()
    config = json.loads(data, object_pairs_hook=_strict_pairs)
    if (not isinstance(config, dict) or set(config) != FIELDS or
            type(config["schema_version"]) is not int or config["schema_version"] != 1 or
            type(config["motor_id"]) is not int or config["motor_id"] != 11 or
            config["eligibility"] != "candidate_only"):
        raise ValueError("Expected schema 1, ID11, candidate_only replacement evidence")
    captures, sources, identities = {}, {}, {}
    for label in ("previous", "replacement"):
        directory, digest = config[label + "_capture"], config[label + "_events_sha256"]
        if not isinstance(directory, str) or not Path(directory).is_absolute():
            raise ValueError("Evidence capture paths must be absolute")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("Evidence must pin a lowercase SHA256")
        capture = load_snapshot(Path(directory))
        if capture["sources"]["events"]["sha256"] != digest:
            raise ValueError("Evidence event hash mismatch: " + label)
        if set(capture["motors"]) != {str(i) for i in range(1, 13)}:
            raise ValueError("Evidence must contain every motor ID1..12")
        identities[label] = {mid: _uid(m["mcu_uid_hex"]) for mid, m in capture["motors"].items()}
        if len(set(identities[label].values())) != 12:
            raise ValueError("Evidence motor UIDs must be unique")
        captures[label] = capture
        sources[label] = {"path": str(Path(directory).resolve()), **capture["sources"]}
    previous, replacement = identities["previous"], identities["replacement"]
    if previous["11"] == replacement["11"]:
        raise ValueError("ID11 has the same UID; replacement is not established")
    if any(previous[str(i)] != replacement[str(i)] for i in range(1, 13) if i != 11):
        raise ValueError("Another motor ID/UID changed; this evidence is only for ID11")
    motor = captures["replacement"]["motors"]["11"]
    position = motor["position"]
    if type(position["samples"]) is not int or position["samples"] < 60:
        raise ValueError("Replacement ID11 requires at least 60 position samples")
    for value, maximum in ((position["peak_to_peak_rad"], .02),
                           (position["max_adjacent_step_rad"], .02),
                           (motor["max_abs_current_A"], .05)):
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= maximum:
            raise ValueError("Replacement stability/current evidence failed")
    # Host monotonic clocks and shaft angles need not survive the replacement.
    end = captures["previous"]["capture_intervals"]["wall_time_ns"]["end"]
    start = captures["replacement"]["capture_intervals"]["wall_time_ns"]["start"]
    if type(end) is not int or type(start) is not int or not 0 < end < start:
        raise ValueError("Replacement evidence must follow previous evidence in wall time")
    return {"schema_version": 1, "motor_id": 11, "eligibility": "candidate_only",
            "retired_uid": previous["11"], "replacement_uid": replacement["11"],
            "sources": {"config": {"path": str(path), "sha256": hashlib.sha256(data).hexdigest()},
                        **sources},
            "approved_for_runtime": False, "motor_power_cycle_continuity_verified": False}


def identity_allowed(mid, uids, policy=None):
    """Require one stable identity; retired hardware remains blocked at any ID."""
    if type(mid) is not int or not 1 <= mid <= 12:
        return False
    if policy is None:
        return mid != 11
    try:
        if (type(policy["schema_version"]) is not int or policy["schema_version"] != 1 or
                type(policy["motor_id"]) is not int or policy["motor_id"] != 11 or
                policy["eligibility"] != "candidate_only" or
                not isinstance(uids, list) or not uids):
            return False
        retired, replacement = _uid(policy["retired_uid"]), _uid(policy["replacement_uid"])
        observed = [_uid(uid) for uid in uids]
        if retired == replacement or len(set(observed)) != 1 or retired in observed:
            return False
        return mid != 11 or observed[0] == replacement
    except (KeyError, TypeError, ValueError):
        return False
