"""Pure, bounded telemetry history for diagnostic shadow replay; no hardware I/O.

Motor timing is the host request-start / serial-read-completion interval, not
the unknown instant when the motor sampled its sensor. IMU timing similarly
covers the host read. A tick sees only entries whose read completed by that
tick. Age uses the interval start conservatively; acquisition spread covers
the earliest interval start through the latest interval end across all selected
motor and IMU entries. Limits are diagnostic and never authorize motor output.

``ingest_pipeline_reply`` accepts successful, already decoded and protocol-
validated PipelineCAN Type 17 reply rows. It does not validate raw CAN bytes.
Alternatively use ``ingest_motor`` with explicit SI units and host timestamps.
Keep raw IMU axes in their sensor frame; mounting transforms/calibration belong
to a later stage. Missing values remain absent, including before retained history.
"""
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass
import math


MOTOR_KEYS = tuple((mid, parameter) for mid in range(1, 13)
                   for parameter in ("position", "velocity"))
_UNITS = {"position": ("rad", "rad_output_shaft"),
          "velocity": ("rad_s", "rad_s_output_shaft")}
_INDICES = {"position": 0x7019, "velocity": 0x701B}


def _ns(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer nanosecond timestamp")
    return value


def _finite(value, name):
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _vector(value, name):
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three sensor-frame SI values")
    return tuple(_finite(v, name) for v in value)


@dataclass(frozen=True)
class MotorObservation:
    motor_id: int
    parameter: str
    value: float
    unit: str
    request_ns: int
    received_ns: int


@dataclass(frozen=True)
class IMUObservation:
    accel_m_s2: tuple
    gyro_rad_s: tuple
    read_started_ns: int
    read_finished_ns: int


@dataclass(frozen=True)
class DiagnosticSnapshot:
    tick_ns: int
    motor_samples: tuple
    imu_sample: IMUObservation | None
    blocked_reasons: tuple
    oldest_observation_age_ns: int | None
    acquisition_spread_ns: int | None
    receive_spread_ns: int | None
    max_age_ns: int
    max_spread_ns: int

    @property
    def status(self):
        return "BLOCKED" if self.blocked_reasons else "DIAGNOSTIC_READY"

    @property
    def output_allowed(self):
        return False

    def as_dict(self):
        """Return an independent diagnostic record; unavailable values are None."""
        selected = {(s.motor_id, s.parameter): s for s in self.motor_samples}
        motors = []
        for mid, parameter in MOTOR_KEYS:
            sample = selected.get((mid, parameter))
            motors.append({"motor_id": mid, "parameter": parameter,
                           "value": sample.value if sample else None,
                           "unit": _UNITS[parameter][0],
                           "request_ns": sample.request_ns if sample else None,
                           "received_ns": sample.received_ns if sample else None,
                           "age_upper_bound_ns": self.tick_ns-sample.request_ns if sample else None})
        imu = self.imu_sample
        return {"status": self.status, "output_allowed": False, "tick_ns": self.tick_ns,
                "blocked_reasons": list(self.blocked_reasons), "motors": motors,
                "imu": None if imu is None else {
                    "frame": "raw_sensor", "accel_m_s2": list(imu.accel_m_s2),
                    "gyro_rad_s": list(imu.gyro_rad_s),
                    "read_started_ns": imu.read_started_ns,
                    "read_finished_ns": imu.read_finished_ns,
                    "age_upper_bound_ns": self.tick_ns-imu.read_started_ns},
                "oldest_observation_age_ns": self.oldest_observation_age_ns,
                "acquisition_spread_ns": self.acquisition_spread_ns,
                "receive_spread_ns": self.receive_spread_ns,
                "max_age_ns": self.max_age_ns, "max_spread_ns": self.max_spread_ns}


class TelemetrySnapshotBuffer:
    """Retain at most ``history_per_key`` immutable observations per source.

    Per-key interval starts and ends must both strictly increase. Different keys
    may have equal completion timestamps, as when one serial read yields several
    replies. Ingestion order across keys is unrestricted. Snapshot ticks may run
    backward for replay; an evicted necessary sample produces ``history_gap``.
    """
    def __init__(self, *, history_per_key=32, max_age_ns=100_000_000,
                 max_spread_ns=50_000_000):
        if type(history_per_key) is not int or not 1 <= history_per_key <= 1024:
            raise ValueError("history_per_key must be an integer in 1..1024")
        self.max_age_ns = _ns(max_age_ns, "max_age_ns")
        self.max_spread_ns = _ns(max_spread_ns, "max_spread_ns")
        self.history_per_key = history_per_key
        self._history = {key: deque(maxlen=history_per_key) for key in (*MOTOR_KEYS, "imu")}
        self._first_finished = {}

    @staticmethod
    def _interval(sample):
        if isinstance(sample, MotorObservation):
            return sample.request_ns, sample.received_ns
        return sample.read_started_ns, sample.read_finished_ns

    def _append(self, key, sample):
        start, finish = self._interval(sample)
        if finish < start:
            raise ValueError("read completion must not precede acquisition start")
        history = self._history[key]
        if history:
            old_start, old_finish = self._interval(history[-1])
            if start <= old_start or finish <= old_finish:
                raise ValueError("duplicate or reordered per-key source/read timestamps")
        self._first_finished.setdefault(key, finish)
        history.append(sample)
        return sample

    def ingest_motor(self, *, can_type, motor_id, parameter, value, unit,
                     request_ns, received_ns):
        if type(can_type) is not int or can_type != 17:
            raise ValueError("only already validated Type 17 motor reads are accepted")
        if type(motor_id) is not int or not 1 <= motor_id <= 12:
            raise ValueError("motor_id must be an integer in 1..12")
        if not isinstance(parameter, str) or parameter not in _UNITS:
            raise ValueError("parameter must be position or velocity")
        if unit not in _UNITS[parameter]:
            raise ValueError("motor data must declare rad or rad_s output-shaft SI units")
        sample = MotorObservation(motor_id, parameter, _finite(value, "value"),
                                  _UNITS[parameter][0], _ns(request_ns, "request_ns"),
                                  _ns(received_ns, "received_ns"))
        return self._append((motor_id, parameter), sample)

    def ingest_pipeline_reply(self, row):
        """Adapter for successful PipelineCAN reply events, excluding identities."""
        if not isinstance(row, Mapping) or row.get("kind") != "pipeline_reply" or row.get("ok") is not True:
            raise ValueError("expected a successful pipeline_reply event")
        result = row.get("result")
        if not isinstance(result, Mapping) or result.get("ok") is not True:
            raise ValueError("expected a successful decoded parameter result")
        parameter = row.get("parameter")
        if not isinstance(parameter, str) or parameter not in _INDICES:
            raise ValueError("identity and other parameters are not telemetry")
        if (type(result.get("motor_id")) is not int
                or result.get("motor_id") != row.get("motor_id")
                or result.get("parameter") != parameter
                or type(result.get("status")) is not int or result["status"] != 0
                or type(result.get("index")) is not int or result["index"] != _INDICES[parameter]):
            raise ValueError("decoded result does not match the successful request")
        started = _ns(row.get("write_started_monotonic_ns"), "write_started_monotonic_ns")
        finished = _ns(row.get("write_finished_monotonic_ns"), "write_finished_monotonic_ns")
        received = _ns(row.get("received_monotonic_ns"), "received_monotonic_ns")
        if not started <= finished <= received:
            raise ValueError("expected request start <= write completion <= receive completion")
        return self.ingest_motor(can_type=17, motor_id=row.get("motor_id"), parameter=parameter,
                                 value=result.get("value"), unit=result.get("unit"),
                                 request_ns=started, received_ns=received)

    def ingest_imu(self, *, accel_m_s2, gyro_rad_s, read_started_ns, read_finished_ns):
        sample = IMUObservation(_vector(accel_m_s2, "accel_m_s2"),
                                _vector(gyro_rad_s, "gyro_rad_s"),
                                _ns(read_started_ns, "read_started_ns"),
                                _ns(read_finished_ns, "read_finished_ns"))
        return self._append("imu", sample)

    def history_sizes(self):
        """Return a copy of retained counts; no sample or internal deque is exposed."""
        return {key: len(history) for key, history in self._history.items()}

    def _at(self, key, tick_ns):
        for sample in reversed(self._history[key]):
            if self._interval(sample)[1] <= tick_ns:
                return sample, None
        if key not in self._first_finished:
            return None, "missing"
        if tick_ns < self._first_finished[key]:
            return None, "not_yet_available"
        return None, "history_gap"

    def snapshot(self, tick_ns):
        tick_ns = _ns(tick_ns, "tick_ns")
        motors, imu, reasons, intervals = [], None, [], []
        for key in (*MOTOR_KEYS, "imu"):
            label = "imu" if key == "imu" else f"ID{key[0]}.{key[1]}"
            sample, unavailable = self._at(key, tick_ns)
            if unavailable:
                reasons.append(f"{unavailable}:{label}")
                continue
            start, finish = self._interval(sample)
            intervals.append((start, finish))
            if tick_ns-start > self.max_age_ns:
                reasons.append(f"stale:{label}")
            if key == "imu":
                imu = sample
            else:
                motors.append(sample)
        oldest_age = tick_ns-min(start for start, _ in intervals) if intervals else None
        spread = max(end for _, end in intervals)-min(start for start, _ in intervals) if intervals else None
        receive_spread = max(end for _, end in intervals)-min(end for _, end in intervals) if intervals else None
        if spread is not None and spread > self.max_spread_ns:
            reasons.append("acquisition_spread_exceeded")
        return DiagnosticSnapshot(tick_ns, tuple(motors), imu, tuple(reasons), oldest_age,
                                  spread, receive_spread, self.max_age_ns, self.max_spread_ns)
