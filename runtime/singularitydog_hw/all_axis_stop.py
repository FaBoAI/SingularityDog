"""Transport-independent, finite STOP coordination for the two fixed CAN buses.

This is preparation for a future drive runner, not an attached device driver,
drive authorization, physical-stop verification, or a hardware watchdog. No
existing runner is changed. A blocked/stalled caller or backend cannot be
preempted here: the finite wait bound depends on the backend contract below.
The last documented all-axis can_timeout readback was zero; this coordinator
neither configures that setting nor substitutes for a tested motor watchdog.

``coordinate_stop(front, rear, timeout_ns=...)`` performs exactly twelve
individual exchanges in front1/rear7/front2/rear8/... order. Each backend call
must attempt the supplied single 17-byte STOP, with no retry or concatenation,
and return or raise before its absolute monotonic deadline. It must own its
port, cancel queued active commands, retain physical pacing, and finish the
reply exchange before the next call. No new thread, worker or polling loop is
created by this module. The sum of backend wait budgets is 12*timeout_ns;
scheduling and coordinator execution time are additional, not hard bounded.

Backend failures do not suppress subsequent STOP requests. In particular an
unclean receive boundary must not prevent an emergency STOP write; instead it
must be reported as unclean so the reply cannot confirm a fresh transaction.
The backend must preserve rejected/residual bytes in its own audit log and
report clean_start/end only when genuinely established, never after silently
discarding evidence. After an unresolved reply it must retain that ambiguity;
timestamps alone cannot disambiguate a late same-key previous STOP reply.
The backend supplies actual write/read timestamps in the clock's domain.
"""
from dataclasses import asdict, dataclass
import time
from typing import Protocol

from .can_readonly import Frame
from .rs05_trial_protocol import TrialPhase, decode_type2, stop_request


@dataclass(frozen=True)
class StopExchange:
    """One owned exchange result; none of these fields is inferred by the caller."""

    write_started_ns: int
    write_finished_ns: int
    write_returned_bytes: int
    received_ns: int
    frame: Frame
    clean_start: bool
    clean_end: bool


class StopBackend(Protocol):
    def stop_exchange(self, wire: bytes, *, deadline_ns: int) -> StopExchange:
        """Attempt one STOP and complete its exchange by deadline, or raise.

        A transport adapter, not this coordinator, enforces port ownership,
        pacing, cancellation of active output, and bounded serial operations.
        No active command or automatic retry is permitted during this call.
        """
        ...


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _stamp(value):
    _require(type(value) is int and 0 <= value < 2**63,
             "Expected integer monotonic nanoseconds below 2**63")
    return value


def _error_text(error):
    # Do not invoke a backend-controlled __str__ (it may raise or block).
    arguments = BaseException.args.__get__(error)
    detail = (arguments[0][:240] if arguments and type(arguments[0]) is str
              else "exception text unavailable")
    return type(error).__name__ + ": " + detail


def _validate(exchange, mid, started, returned, deadline):
    _require(type(exchange) is StopExchange, "Backend did not return StopExchange")
    _require(exchange.clean_start is True and exchange.clean_end is True,
             "Fresh clean receive boundaries were not established")
    _require(type(exchange.write_returned_bytes) is int
             and exchange.write_returned_bytes == 17, "STOP write was not complete")
    a, b, received = map(_stamp, (exchange.write_started_ns,
                                 exchange.write_finished_ns, exchange.received_ns))
    _require(started <= a <= b <= received <= returned < deadline,
             "STOP exchange is stale, noncausal or late")
    _require(type(exchange.frame) is Frame, "Expected a canonical parsed Frame")
    feedback = decode_type2(exchange.frame, motor_id=mid)
    _require(feedback.mode_state == 0 and feedback.fault_bits == 0,
             "STOP reply must report reset mode0 and fault0")
    return feedback


def coordinate_stop(front: StopBackend, rear: StopBackend, *, timeout_ns: int,
                    clock=time.monotonic_ns):
    """Attempt every axis once and report fresh STOP acknowledgements only.

    ``timeout_ns`` is an explicit per-exchange budget in (0, 1 second]. This
    configuration cap is not an acceptable physical-stop-latency specification.
    The injected clock must be a functioning monotonic clock. Backend exceptions
    (including cancellation) are recorded, then the other axes are attempted.
    No device is opened or closed here; the caller owns backend lifetime.
    """
    _require(front is not rear, "Front and rear must have distinct backend owners")
    _require(type(timeout_ns) is int and 0 < timeout_ns <= 1_000_000_000,
             "Explicit per-exchange timeout must be in (0, 1 second]")
    _require(callable(clock) and all(callable(getattr(b, "stop_exchange", None))
                                    for b in (front, rear)), "Invalid clock or backend")
    # Prepare immutable canonical STOP bytes before the first backend call.
    schedule = tuple((scope, mid, backend,
                      stop_request(phase=TrialPhase.STOP, motor_id=mid))
                     for first in range(1, 7)
                     for scope, mid, backend in (("front", first, front),
                                                 ("rear", first+6, rear)))
    started = previous = _stamp(clock())
    _require(started + 12*timeout_ns < 2**63, "Stop budget exceeds timestamp range")
    rows = []
    for scope, mid, backend, wire in schedule:
        call_started = _stamp(clock())
        _require(call_started >= previous, "Coordinator clock moved backwards")
        deadline = call_started + timeout_ns
        _stamp(deadline)
        row = {"scope": scope, "motor_id": mid, "request_attempted": True,
               "request_wire_hex": wire.hex(), "deadline_ns": deadline,
               "call_started_ns": call_started, "call_returned_ns": None,
               "confirmed": False, "exchange": None, "error": None}
        rows.append(row)
        try:
            exchange = backend.stop_exchange(wire, deadline_ns=deadline)
        except BaseException as error:
            row["error"] = _error_text(error)
        else:
            # Retain detached accepted evidence. The backend retains rejected RX.
            try:
                returned = _stamp(clock())
                row["call_returned_ns"] = returned
                _require(returned >= call_started, "Coordinator clock moved backwards")
                feedback = _validate(exchange, mid, call_started, returned, deadline)
                row["exchange"] = {
                    "write_started_monotonic_ns": exchange.write_started_ns,
                    "write_finished_monotonic_ns": exchange.write_finished_ns,
                    "write_returned_bytes": exchange.write_returned_bytes,
                    "received_monotonic_ns": exchange.received_ns,
                    "raw_frame": exchange.frame.record(), "feedback": asdict(feedback),
                    "clean_start": True, "clean_end": True}
                row["confirmed"] = True
            except BaseException as error:
                row["error"] = _error_text(error)
        if row["call_returned_ns"] is None:
            row["call_returned_ns"] = _stamp(clock())
        previous = row["call_returned_ns"]
        if previous < call_started or previous >= deadline:
            row["confirmed"] = False
            row["error"] = row["error"] or "Backend missed its absolute deadline"
    all_confirmed = all(row["confirmed"] for row in rows)
    return {"status": "ALL_STOP_REPLIES_CONFIRMED" if all_confirmed else "STOP_UNCONFIRMED",
            "all_stop_replies_confirmed": all_confirmed, "requests_attempted": len(rows),
            "confirmed_ids": [row["motor_id"] for row in rows if row["confirmed"]],
            "started_ns": started, "finished_ns": previous,
            "per_exchange_timeout_ns": timeout_ns, "backend_wait_budget_ns": 12*timeout_ns,
            "bound_requires_deadline_compliant_backend": True,
            "physical_stop_verified": False, "hardware_watchdog_available": False,
            "drive_authorized": False, "automatic_retry": False, "motors": rows}
