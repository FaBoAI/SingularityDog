"""Explicit experimental preparation adapter; native validation, unchanged hashes.

The helper accepts only the already-owned snapshot values. The reference module,
IMU validation, evidence digests, PreparedCycle and _Motor remain unchanged.
"""
from singularitydog_hw.fast_policy_inputs import (
    _Motor, PreparedCycle, _require, _finite, _stamp, _digest,
    MAX_AGE_NS, MAX_SPREAD_NS,
)
from _event_snapshot_native import snapshot_event
from _native_input_validation import validate_motors


def prepare_cycle(reply_rows, imu_sample, *, cycle=1):
    """Validate a whole captured cycle, own its values, and compute evidence.

    This full preparation cost MUST be reported separately from hot assembly.
    It runs for every new source cycle, even when numbers happen to be equal.
    The saved capture may then be replayed without re-decoding or re-hashing.
    """
    _require(type(cycle) is int and 1 <= cycle <= 20, "Invalid fixed cycle")
    _require(type(reply_rows) in (list, tuple) and len(reply_rows) == 12,
             "Exactly twelve successful same-cycle replies are required")
    rows, imu = snapshot_event(reply_rows), snapshot_event(imu_sample)
    records = {values[0]: (_Motor(*values), raw, row)
               for values, raw, row in validate_motors(rows, cycle)}
    _require(type(imu) is dict and imu.get("kind") == "imu"
             and imu.get("frame") in ("sensor", "raw_sensor"), "Require original sensor IMU event")
    vectors = []
    for key in ("accel_m_s2", "gyro_rad_s"):
        value = imu.get(key)
        _require(type(value) in (list, tuple) and len(value) == 3, "Invalid IMU vector")
        vectors.append(tuple(_finite(x, key) for x in value))
    start = _stamp(imu.get("read_started_monotonic_ns"), "IMU start")
    end = _stamp(imu.get("read_finished_monotonic_ns"), "IMU finish")
    available = _stamp(imu.get("available_monotonic_ns", end), "IMU availability")
    _require(start <= end <= available, "Noncausal IMU interval or availability")
    ordered = tuple(records[i] for i in range(1, 13))
    prepared = PreparedCycle(tuple(x[0] for x in ordered), *vectors, start, end, available, cycle,
        (_digest([x[1] for x in ordered]), _digest([x[2] for x in ordered]), _digest(imu)))
    # Check the source interval without allocating a throwaway snapshot or
    # claiming that any historical timestamp represents the current clock.
    earliest = min(start, *(m.start for m in prepared.motors))
    latest = max(end, *(m.received for m in prepared.motors))
    newest_available = max(available, *(m.available for m in prepared.motors))
    _require(newest_available-earliest <= MAX_AGE_NS and latest-earliest <= MAX_SPREAD_NS,
             "Stale or excessive diagnostic acquisition spread")
    return prepared
