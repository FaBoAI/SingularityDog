"""Finite RS05 Active Reporting experiment; never enables or drives a motor.

Default CLI prints a plan without opening a serial port. Live runs require a
known reporting-OFF baseline, pinned ports/boot/UIDs, and explicit execution.
Only identity/voltage/period reads, an explicit version read, the period setting,
reporting switch and all-zero STOP are available. Preflight-only forbids
reporting switches and period writes at the physical send boundary.
Existing read-only transport guards are unchanged.
State reports during streaming are NEVER counted as STOP acknowledgements.
"""
import argparse
from contextlib import ExitStack
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

from . import can_readonly as codec
from . import dual_can_pipeline_benchmark as dual
from .active_report_protocol import (reporting_request, period_read_request,
                                     period_write_request, decode_period_reply)
from .active_report_receiver import StreamReceiver
from .can_timing_probe import ownership_locks, validate_uids
from .motor_version_probe import version_request, decode_version
from .rs05_trial_protocol import TrialPhase, stop_request, decode_type2
from .serial_deadline_reader import DeadlineSerialReader, record_failed_read
from .sensor_pipeline_benchmark import BootIdentityGuard

FLAGS = dict(motor_enabling_available=False, motion_command_available=False,
             full_controller_50Hz_verified=False, full_pipeline_20ms_verified=False,
             velocity_scale_verified=False, calibration_verified=False,
             motor_internal_sample_time_verified=False, can_wire_completion_verified=False,
             automatic_retry=False, pure_read_only=False)
MAX_RAW_BYTES = 4_000_000
MAX_RAW_CHUNKS = 150_000
QUIET_NS = 100_000_000
QUERY_NS = 250_000_000
CLEANUP_NS = 8_000_000_000
_HELD_LOCKS = []  # Keep ownership if actual port close cannot be confirmed.


def require(condition, message):
    if not condition:
        raise ValueError(message)


def make_plan(stage="one", seconds=10, expected_kind=None, motor_id=1,
              period_policy="set-10ms", activation_type2_prefix=False, *,
              preflight_only=False, read_versions=False, motor_ids=None, off_first_id=None):
    require(stage in ("one", "front", "rear", "both"), "Unknown stage")
    require(type(seconds) in (int, float) and math.isfinite(seconds)
            and seconds in (1, 10, 60), "Select a finite 1, 10 or 60 second run")
    require(expected_kind is None or (type(expected_kind) is int and expected_kind in (2, 24)),
            "Report kind must be discovery, 2 or 24")
    require(type(motor_id) is int and 1 <= motor_id <= 12, "Select an ID in 1..12")
    require(period_policy in ("set-10ms", "observe-current"), "Unknown period policy")
    require(type(activation_type2_prefix) is bool, "Activation prefix option must be boolean")
    require(not activation_type2_prefix or expected_kind == 24,
            "Explicit Type2 activation prefix requires periodic Type24")
    require(type(preflight_only) is bool and type(read_versions) is bool,
            "Preflight and version options must be boolean")
    observe = period_policy == "observe-current"
    scopes = ({"front" if motor_id <= 6 else "rear": (motor_id,)} if stage == "one"
              else {s: ids for s, ids in dual.SCOPES.items() if stage == "both" or stage == s})
    if motor_ids is not None:
        require(stage != "one", "--motor-ids requires stage front, rear, or both")
        require(type(motor_ids) in (tuple, list) and bool(motor_ids)
                and all(type(mid) is int for mid in motor_ids),
                "Motor subset must be nonempty integer IDs")
        require(tuple(motor_ids) == tuple(sorted(set(motor_ids))),
                "Motor subset must be strictly sorted and unique")
        require(set(motor_ids) <= {mid for ids in scopes.values() for mid in ids},
                "Motor subset is outside the selected stage")
        scopes = {scope: tuple(mid for mid in ids if mid in motor_ids)
                  for scope, ids in scopes.items() if set(ids) & set(motor_ids)}
    selected = {mid for ids in scopes.values() for mid in ids}
    require(off_first_id is None or (type(off_first_id) is int and off_first_id in selected),
            "OFF-first ID must be one of the selected IDs")
    return {**FLAGS, "stage": stage, "seconds": seconds, "motor_id": motor_id,
            "motor_ids": list(motor_ids) if motor_ids is not None else None,
            "ids_by_scope": {s: list(ids) for s, ids in scopes.items()},
            "off_first_id": off_first_id,
            "off_order_by_scope": {s: ([off_first_id] if off_first_id in ids else []) +
                [mid for mid in ids if mid != off_first_id] for s, ids in scopes.items()},
            "preflight_only": preflight_only, "read_versions": read_versions,
            "success_status": "PREFLIGHT_COMPLETE" if preflight_only else "CAN_REPORT_HOST_OBSERVATION_COMPLETE",
            "expected_report_kind": expected_kind, "period_policy": period_policy,
            "activation_type2_prefix": activation_type2_prefix,
            "activation_prefix_is_not_stop_ack": True,
            "report_period_ticks": None if observe or preflight_only else 1,
            "report_period_ms": None if observe or preflight_only else 10,
            "allowed_can_types": [0, 4, 17] if preflight_only else
                                 [0, 4, 17, 24] if observe else [0, 4, 17, 18, 24],
            "writable_parameter": None if observe or preflight_only else
                                  "0x7026 only; volatile; restore captured value",
            "known_reporting_off_required": True, "initial_quiet_ms": 100,
            "uart_baudrate": 921600, "streaming_stop_ack_available": False,
            "cleanup_seconds_per_bus": 8, "max_raw_bytes_per_bus": MAX_RAW_BYTES,
            "host_updates_do_not_prove_physical_packet_loss": True,
            "scope": "Disabled-state preflight only; no reporting or period writes" if preflight_only else
                     "CAN-only disabled-state report experiment; no IMU/inference"}


class ActiveProbe:
    """Single port owner; injection points allow end-to-end offline fault tests."""
    def __init__(self, raw, ids, expected, *, seconds, expected_kind=None,
                 period_policy="set-10ms", activation_type2_prefix=False,
                 preflight_only=False, read_versions=False, off_first_id=None,
                 clock=time.monotonic_ns, check=lambda cleaning=False: None, reader_factory=DeadlineSerialReader,
                 identities_ready=lambda: None, stops_ready=lambda: None,
                 reports_ready=lambda: None):
        require(tuple(ids) and all(type(i) is int and 1 <= i <= 12 for i in ids)
                and tuple(ids) == tuple(sorted(set(ids))), "Invalid selected IDs")
        make_plan(seconds=seconds, expected_kind=expected_kind, period_policy=period_policy,
                  activation_type2_prefix=activation_type2_prefix,
                  preflight_only=preflight_only, read_versions=read_versions)
        require(off_first_id is None or (type(off_first_id) is int and off_first_id in ids),
                "OFF-first ID must be one of this port's selected IDs")
        self.period_policy = period_policy
        self.activation_type2_prefix = activation_type2_prefix
        self.preflight_only, self.read_versions = preflight_only, read_versions
        self.off_first_id = off_first_id
        self.raw, self.ids, self.expected = raw, tuple(ids), validate_uids(expected)
        self.seconds, self.clock, self.external_check = seconds, clock, check
        self.identities_ready, self.stops_ready = identities_ready, stops_ready
        self.reports_ready = reports_ready
        self.started = clock()
        self.deadline = self.started + int((seconds + 15) * 1e9)
        self.cleaning = False
        self.reader = reader_factory(raw, clock=clock, check=self.guard)
        self.receiver = StreamReceiver(tuple(ids), expected_kind=expected_kind,
                                       activation_type2_prefix=activation_type2_prefix,
                                       max_frames=int((seconds + 5) * 1000 * len(ids)))
        self.raw_log, self.tx_log = [], []
        self.raw_bytes = 0
        self._reporting_raw_start = self._reporting_raw_bytes_start = None
        self._cleanup_quiet_end = None
        self._cleanup_replay_attempted = False
        self.cleanup_boundary_verified = False
        self.transport_poisoned = False
        self.periods, self.dirty_periods, self.reporting_attempted = {}, set(), set()
        self.stop_observations = []
        self.measure_start = self.measure_end = None
        self.report = {**FLAGS, "status": "INCOMPLETE", "ids": list(ids),
                       "preflight_only": preflight_only, "read_versions": read_versions,
                       "off_first_id": off_first_id,
                       "period_policy": period_policy,
                       "activation_type2_prefix": activation_type2_prefix,
                       "activation_prefix_is_not_stop_ack": True,
                       "known_reporting_off_precondition": True,
                       "reporting_switch_readback_available": False,
                       "initial_quiet_observed": False, "cleanup": {}}

    def guard(self):
        # Cancellation ends acquisition, but does not skip bounded restoration.
        self.external_check(self.cleaning)
        if self.clock() >= self.deadline:
            raise TimeoutError("Active report phase deadline exhausted")

    def _read(self, wake):
        self.guard()
        started = self.clock()
        try:
            chunk, received = self.reader.read_until(min(wake, self.deadline - 1), self.deadline)
        except BaseException as error:
            self.raw_bytes = record_failed_read(error, report=self.report, raw_log=self.raw_log,
                raw_bytes=self.raw_bytes, read_started_ns=started,
                max_raw_bytes=MAX_RAW_BYTES, max_raw_chunks=MAX_RAW_CHUNKS)
            raise
        if chunk:
            self.raw_bytes += len(chunk)
            if self.raw_bytes > MAX_RAW_BYTES or len(self.raw_log) >= MAX_RAW_CHUNKS:
                if not self.cleaning:
                    raise RuntimeError("Bounded raw evidence buffer exhausted")
                self.report["cleanup"]["raw_log_overflow"] = True
            else:
                self.raw_log.append((started, received, bytes(chunk)))
        return chunk, started, received

    def _send(self, mid, action, value=None):
        self.guard()
        require(not self.transport_poisoned, "UART framing uncertain; further writes prohibited")
        require(mid in self.ids, "Wrong scope at write boundary")
        builders = {
            "identity": lambda: codec.read_request(mid),
            "voltage": lambda: codec.read_request(mid, "voltage"),
            "period_read": lambda: period_read_request(mid),
            "version": lambda: version_request(mid),
            "stop": lambda: stop_request(phase=TrialPhase.STOP, motor_id=mid),
            "period_set": lambda: period_write_request(mid, value),
            "report_on": lambda: reporting_request(mid, True),
            "report_off": lambda: reporting_request(mid, False),
        }
        require(action in builders, "Unpermitted command")
        if self.preflight_only:
            require(action in ("identity", "voltage", "period_read", "stop", "version"),
                    "Preflight forbids reporting switches and period writes")
        if action in ("identity", "voltage", "period_read", "period_set", "stop", "version"):
            if self.cleaning and self.reporting_attempted:
                require(self.cleanup_boundary_verified, "Cleanup receive boundary is not verified")
            require(not self.receiver.parser.buffer or (self.cleaning and self.cleanup_boundary_verified),
                    "Residual stream bytes prevent a fresh query")
        if action == "version":
            require(self.read_versions and not self.cleaning and not self.reporting_attempted,
                    "Version reads require explicit selection before any reporting activation")
            require(self.report.get("identities_verified") and any(
                row["motor_id"] == mid and row["action"] == "stop" for row in self.stop_observations),
                "Version read requires verified identities and this ID's stopped feedback")
        if self.cleaning:
            require(action in ("report_off", "period_set", "period_read", "stop"),
                    "Cleanup has a narrow command allowlist")
        if action == "period_set":
            require(self.period_policy == "set-10ms", "Period writes prohibited in observe-current mode")
            require(mid in self.periods and value == (self.periods[mid] if self.cleaning else 1),
                    "Period write must be test value or captured restoration value")
            self.dirty_periods.add(mid)  # Mark BEFORE possible partial write.
        if action == "report_on":
            require(not self.cleaning and mid in self.periods, "No report activation in cleanup")
            if not self.reporting_attempted:
                require(not self.receiver.parser.buffer, "Reporting must start at an empty receive boundary")
                self._reporting_raw_start = len(self.raw_log)
                self._reporting_raw_bytes_start = self.raw_bytes
            self.reporting_attempted.add(mid)
        wire = builders[action]()
        require(len(self.tx_log) < 200, "Finite transmission budget exhausted")
        self.raw.write_timeout = min(.1, max(.001, (self.deadline - self.clock()) / 1e9))
        row = {"motor_id": mid, "action": action, "wire_hex": wire.hex(),
               "cleanup": self.cleaning, "started_ns": self.clock(), "returned_bytes": None}
        self.tx_log.append(row)
        if self.cleaning and action == "report_off":
            self.report["cleanup"].setdefault("reporting_off_order", []).append(mid)
        # Revalidate immediately at the physical write; no arbitrary-byte API.
        self.guard()
        try:
            require(wire == builders[action](), "Command changed before serial write")
            row["returned_bytes"] = self.raw.write(wire)
            if type(row["returned_bytes"]) is not int or row["returned_bytes"] != len(wire):
                raise IOError("Partial serial write; command state is uncertain")
        except BaseException:
            # An incomplete AT frame can consume the next write as its payload.
            # No undocumented resynchronization or cleanup write is permissible.
            self.transport_poisoned = True
            raise
        finally:
            row["finished_ns"] = self.clock()
        self.guard()

    def quiet(self, *, initial=False, feed_stream=False):
        limit = min(self.deadline - 1, self.clock() + 1_000_000_000)
        quiet_until = self.clock() + QUIET_NS
        while self.clock() < quiet_until:
            if self.clock() >= limit:
                raise TimeoutError("Could not establish 100ms receive silence")
            chunk, started, received = self._read(min(quiet_until, limit))
            if chunk:
                if initial:
                    raise RuntimeError("Unsolicited bytes: reporting-OFF baseline not established")
                if feed_stream:
                    self.receiver.feed(chunk, started, received)
                quiet_until = received + QUIET_NS
        if feed_stream:
            require(not self.receiver.parser.buffer, "Partial stream frame at quiet boundary")
        return self.clock()

    def query(self, mid, action, value=None):
        try:
            return self._query(mid, action, value)
        except BaseException:
            # A query has its own parser. Its failed/partial reply invalidates
            # the previously established boundary even if the stream is empty.
            if self.cleaning:
                self.cleanup_boundary_verified = False
                self.report["cleanup"]["cleanup_boundary_verified"] = False
            raise

    def _query(self, mid, action, value=None):
        # Used only while periodic reporting is OFF and boundary is quiet.
        self._send(mid, action, value)
        parser = codec.ATParser()
        end = min(self.deadline - 1, self.clock() + QUERY_NS)
        while self.clock() < end:
            chunk, _, received = self._read(end)
            if chunk and received >= end:
                raise TimeoutError("Query response completed after its deadline")
            frames = parser.feed(chunk)
            require(parser.discarded_bytes == 0, "Malformed query response")
            if frames:
                require(len(frames) == 1 and not parser.buffer, "Ambiguous/coalesced query response")
                frame = frames[0]
                if action in ("identity", "voltage"):
                    result = codec.decode_reply(frame, mid, None if action == "identity" else "voltage")
                    require(result["ok"], "Rejected read")
                    return result
                if action == "period_read":
                    return decode_period_reply(frame, mid,
                        allow_unqualified_zero=self.preflight_only or self.period_policy == "observe-current")
                if action == "version":
                    return {**decode_version(frame, mid), "motor_id": mid, "received_ns": received}
                if action in ("stop", "period_set"):
                    feedback = decode_type2(frame, motor_id=mid)
                    require(feedback.mode_state == 0 and feedback.fault_bits == 0,
                            "Expected disabled mode0/fault0 feedback")
                    self.stop_observations.append({"motor_id": mid, "action": action,
                        "received_ns": received, "wire_hex": frame.wire.hex(),
                        "physical_stop_time_verified": False})
                    return feedback
                raise ValueError("Action is not a query")
        raise TimeoutError(f"No unambiguous {action} reply from ID{mid}")

    def stream_until(self, end, *, feed_stream=True):
        while self.clock() < end:
            chunk, started, received = self._read(min(end, self.clock() + 10_000_000))
            if chunk and feed_stream:
                self.receiver.feed(chunk, started, received)

    def _verify_cleanup_boundary(self):
        """Replay retained reporting bytes once, without repairing the receiver.

        This proves only a canonical, empty host receive boundary after OFF and
        silence. It neither validates the failed observation nor turns Type2
        records into STOP acknowledgements or reporting-switch readback.
        """
        require(not self._cleanup_replay_attempted, "Cleanup raw replay already attempted")
        self._cleanup_replay_attempted = True
        require(self.cleaning and self.report.get("failure") and self.reporting_attempted,
                "Cleanup raw replay requires a failed reporting observation")
        require(not self.transport_poisoned and self._cleanup_quiet_end is not None,
                "Cleanup raw replay requires intact TX and observed quiet")
        start, byte_start = self._reporting_raw_start, self._reporting_raw_bytes_start
        require(type(start) is int and 0 <= start <= len(self.raw_log)
                and type(byte_start) is int, "Missing initial reporting byte boundary")
        require(not any(self.report.get(key) or self.report["cleanup"].get(key) for key in
                ("raw_log_overflow", "unlogged_receive_bytes", "unclocked_receive_evidence",
                 "receive_evidence_storage_failed")), "Incomplete raw receive evidence")
        require(sum(len(row[2]) for row in self.raw_log[:start]) == byte_start and
                sum(len(row[2]) for row in self.raw_log[start:]) == self.raw_bytes - byte_start,
                "Raw receive byte coverage differs")
        off = [row for row in self.tx_log if row["cleanup"] and row["action"] == "report_off"]
        require(len(off) == len(self.reporting_attempted) and
                {row["motor_id"] for row in off} == self.reporting_attempted and
                all(type(row["returned_bytes"]) is int and row["returned_bytes"] == 17 and
                    row["wire_hex"] == reporting_request(row["motor_id"], False).hex()
                    for row in off), "Not every reporting OFF completed a full UART write")
        parser = codec.ATParser()
        last_received = None
        frames_seen = 0
        for begun, received, chunk in self.raw_log[start:]:
            require(type(begun) is int and type(received) is int and 0 <= begun <= received
                    and (last_received is None or begun >= last_received),
                    "Invalid cleanup raw receive timestamps")
            require(type(chunk) is bytes, "Invalid cleanup raw bytes")
            last_received = received
            for frame in parser.feed(chunk):
                require(frame.flags == 4 and len(frame.data) == 8 and frame.source in self.ids
                        and frame.destination == codec.HOST_ID and frame.kind in (2, 24),
                        "Noncanonical cleanup reporting frame")
                require(not (frame.kind == 2 and frame.data.startswith(b"\x00\xc4\x56")),
                        "Version reply is not reporting feedback")
                require((frame.can_id >> 16) & 255 == 0, "Nonzero cleanup mode or fault")
                frames_seen += 1
            require(parser.discarded_bytes == 0, "Malformed cleanup raw evidence")
        require(not parser.buffer, "Partial frame remains in cleanup raw evidence")
        require(last_received is None or self._cleanup_quiet_end - last_received >= QUIET_NS,
                "Cleanup raw evidence lacks 100ms receive silence")
        self.cleanup_boundary_verified = True
        self.report["cleanup"].update(cleanup_boundary_verified=True,
            cleanup_boundary_basis="full reporting raw replay after OFF and100ms quiet",
            cleanup_boundary_replay={"raw_start_index": start, "raw_end_index": len(self.raw_log),
                "raw_bytes": self.raw_bytes - byte_start, "frames": frames_seen,
                "discarded_bytes": 0, "residual_bytes": 0})

    def cleanup(self):
        self.cleaning = True
        self.cleanup_boundary_verified = False
        self.report["cleanup"]["cleanup_boundary_verified"] = False
        self.deadline = self.clock() + CLEANUP_NS
        errors = []
        if self.transport_poisoned:
            self.report["cleanup"].update(ok=False, transport_poisoned=True,
                restoration_unknown=True, errors=["Partial/failed UART write: no more writes permitted"])
            return
        feed_cleanup = not self.report.get("failure")
        off_order = ([self.off_first_id] if self.off_first_id in self.reporting_attempted else []) + [
            mid for mid in sorted(self.reporting_attempted) if mid != self.off_first_id]
        self.report["cleanup"].setdefault("reporting_off_order", [])
        for mid in off_order:
            try:
                if self.activation_type2_prefix and feed_cleanup:
                    self.receiver.expect_deactivation(mid, self.clock(), self.clock() + QUERY_NS)
                self._send(mid, "report_off")
            except BaseException as exc:
                errors.append(f"disable ID{mid}: {exc!r}")
                if self.transport_poisoned:
                    self.report["cleanup"].update(ok=False, transport_poisoned=True,
                        restoration_unknown=True, errors=errors)
                    return
                feed_cleanup = False
            # Do not burst OFF commands into the adapter. Keep receiving during
            # a finite gap so every selected ID gets its own transition window.
            pause_end = self.clock() + 20_000_000
            try:
                self.stream_until(pause_end, feed_stream=feed_cleanup)
            except BaseException as exc:
                errors.append(f"disable ID{mid} receive: {exc!r}")
                feed_cleanup = False
                try:
                    self.stream_until(pause_end, feed_stream=False)
                except BaseException as drain_exc:
                    errors.append(f"disable ID{mid} raw drain: {drain_exc!r}")
                    # No functioning receive wait: do not burst remaining OFFs
                    # or invent a resynchronization. Record them as unattempted.
                    break
        quiet_ok = False
        try:
            # Preserve complete stream evidence when valid; faults remain failures.
            self._cleanup_quiet_end = self.quiet(feed_stream=feed_cleanup)
            if self.activation_type2_prefix and feed_cleanup:
                require(not self.receiver.deactivation_pending and
                        set(self.receiver.deactivation_feedback) == self.reporting_attempted,
                        "Deactivation feedback sequence incomplete")
            quiet_ok = True
        except BaseException as exc:
            errors.append(f"report-off quiet: {exc!r}")
            # Parsing failure must not prevent a raw finite drain for restoration.
            try:
                self._cleanup_quiet_end = self.quiet()
                quiet_ok = True
            except BaseException as drain_exc:
                errors.append(f"raw quiet: {drain_exc!r}")
        restored = {}
        if quiet_ok and not errors:
            if self.report.get("failure") and self.reporting_attempted:
                try:
                    self._verify_cleanup_boundary()
                except BaseException as exc:
                    errors.append(f"cleanup raw boundary: {exc!r}")
            else:
                self.cleanup_boundary_verified = True
                self.report["cleanup"].update(cleanup_boundary_verified=True,
                    cleanup_boundary_basis="live receiver boundary after OFF and100ms quiet")
        if quiet_ok and not errors:
            for mid in sorted(self.dirty_periods):
                try:
                    self.query(mid, "period_set", self.periods[mid])
                    restored[mid] = self.query(mid, "period_read")
                    require(restored[mid] == self.periods[mid], "Period restoration readback differs")
                except BaseException as exc:
                    errors.append(f"restore ID{mid}: {exc!r}")
                    if self.transport_poisoned:
                        quiet_ok = False
                        break
                    if self.reporting_attempted and not self.cleanup_boundary_verified:
                        quiet_ok = False
                        break
                    # Finite drain before best-effort restoration of OTHER IDs.
                    try:
                        self.quiet()
                    except BaseException as drain_exc:
                        errors.append(f"restore boundary: {drain_exc!r}")
                        quiet_ok = False
                        break
        if quiet_ok and not errors and self.reporting_attempted:
            for mid in self.ids:
                try:
                    self.query(mid, "stop")
                except BaseException as exc:
                    errors.append(f"final STOP ID{mid}: {exc!r}")
                    quiet_ok = False
                    break
        if quiet_ok:
            try:
                self.quiet(initial=True)
            except BaseException as exc:
                errors.append(f"final quiet: {exc!r}")
                quiet_ok = False
                self.cleanup_boundary_verified = False
                self.report["cleanup"]["cleanup_boundary_verified"] = False
        self.report["cleanup"].update(errors=errors, quiet_observed=quiet_ok,
            reporting_off_ids_not_attempted=sorted(self.reporting_attempted -
                {row["motor_id"] for row in self.tx_log if row["action"] == "report_off"}),
            periods_captured=self.periods, periods_restored=restored,
            reporting_off_restored=bool(self.reporting_attempted) and quiet_ok and not errors,
            reporting_off_basis="explicit OFF baseline + disable writes +100ms quiet; no switch readback",
            ok=quiet_ok and not errors and not self.report["cleanup"].get("raw_log_overflow", False))

    def _observe_reporting(self):
        if self.period_policy == "set-10ms":
            for mid in self.ids:
                if self.periods[mid] != 1:
                    self.query(mid, "period_set", 1)
                    require(self.query(mid, "period_read") == 1, "Period setting not confirmed")
        # Synchronize BEFORE enabling reports; never block an active RX owner.
        self.reports_ready()
        # Finish every setting/readback before any periodic reports start.
        for mid in self.ids:
            if self.activation_type2_prefix:
                self.receiver.expect_activation(mid, self.clock(), self.clock() + QUERY_NS)
            self._send(mid, "report_on")
            self.stream_until(self.clock() + 20_000_000)
        self.stream_until(self.clock() + 200_000_000)  # warmup outside timed region
        if self.activation_type2_prefix:
            require(not self.receiver.activation_pending and
                    set(self.receiver.activation_feedback) == set(self.ids),
                    "Activation feedback sequence incomplete")
        self.measure_start = self.clock()
        self.measure_end = self.measure_start + int(self.seconds * 1e9)
        self.stream_until(self.measure_end)
        self.report["timed_interval_completed"] = True

    def run(self):
        try:
            self.quiet(initial=True)
            self.report["initial_quiet_observed"] = True
            for mid in self.ids:
                identity = self.query(mid, "identity")
                require(identity["mcu_uid_hex"] == self.expected[mid], "Motor UID mismatch")
            self.report["identities_verified"] = True
            self.identities_ready()
            # Confirm a STOP on every selected ID before checking any battery
            # threshold. A low first reading must not leave the rest untested.
            for mid in self.ids:
                self.query(mid, "stop")
            self.stops_ready()
            low_voltages = {}
            for mid in self.ids:
                self.periods[mid] = self.query(mid, "period_read")
                voltage = self.query(mid, "voltage")["value"]
                self.report.setdefault("voltage_by_id", {})[str(mid)] = voltage
                if not 35 <= voltage <= 45:
                    low_voltages[str(mid)] = voltage
                if self.read_versions:
                    self.report.setdefault("versions_by_id", {})[str(mid)] = self.query(mid, "version")
            require(not low_voltages,
                    "Voltage outside this trial's35..45V range: " + repr(low_voltages))
            self.quiet(initial=True)
            self.report["preflight_completed"] = True
            self.report["preflight_final_quiet_observed"] = True
            if not self.preflight_only:
                self._observe_reporting()
        except BaseException as exc:
            self.report["failure"] = repr(exc)
        finally:
            if self.reporting_attempted or self.dirty_periods:
                try:
                    self.cleanup()
                except BaseException as exc:
                    self.report["cleanup"].update(ok=False, failure=repr(exc))
            else:
                self.report["cleanup"].update(ok=True, no_settings_attempted=True)
        if self.measure_start is not None and self.report.get("timed_interval_completed"):
            self.report["stream"] = self.receiver.summary(self.measure_start, self.measure_end)
        elif self.measure_start is not None:
            self.report["stream"] = {"ok": False, "errors": ["observation_window_incomplete"]}
        self.report.update(measure_start_ns=self.measure_start, measure_end_ns=self.measure_end,
                           activation_feedback=[{**asdict(sample), "wire": sample.wire.hex()}
                               for sample in self.receiver.activation_feedback.values()],
                           deactivation_feedback=[{**asdict(sample), "wire": sample.wire.hex()}
                               for sample in self.receiver.deactivation_feedback.values()],
                           periods_captured_raw=self.periods,
                           period_encoding_qualified_by_id={str(mid): value >= 1
                               for mid, value in self.periods.items()},
                           stop_observations=self.stop_observations,
                           tx_attempts=len(self.tx_log), raw_bytes=self.raw_bytes,
                           transport_poisoned=self.transport_poisoned)
        if (self.preflight_only and not self.report.get("failure")
                and self.report.get("preflight_completed") and self.report["cleanup"].get("ok")):
            self.report["status"] = "PREFLIGHT_COMPLETE"
        elif (not self.preflight_only and not self.report.get("failure") and self.report.get("stream", {}).get("ok")
                and self.report["cleanup"].get("ok")):
            self.report["status"] = "CAN_REPORT_HOST_OBSERVATION_COMPLETE"
        return self.report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stage", choices=("one", "front", "rear", "both"), default="one")
    ap.add_argument("--motor-id", type=int, default=1)
    ap.add_argument("--motor-ids", type=int, nargs="+",
                    help="Sorted unique subset for front, rear, or both; space-separated IDs")
    ap.add_argument("--off-first-id", type=int)
    ap.add_argument("--seconds", type=int, choices=(1, 10, 60), default=10)
    ap.add_argument("--report-kind", choices=("discover", "2", "24"), default="discover")
    ap.add_argument("--period-policy", choices=("set-10ms", "observe-current"), default="set-10ms")
    ap.add_argument("--activation-type2-prefix", action="store_true")
    ap.add_argument("--preflight-only", action="store_true")
    ap.add_argument("--read-versions", action="store_true")
    ap.add_argument("--front-port")
    ap.add_argument("--rear-port")
    ap.add_argument("--expected-uids", type=Path)
    ap.add_argument("--expected-boot-id")
    ap.add_argument("--output", type=Path)
    ap.add_argument("--known-reporting-off", action="store_true")
    ap.add_argument("--execute-no-motion", action="store_true")
    args = ap.parse_args(argv)
    kind = None if args.report_kind == "discover" else int(args.report_kind)
    plan = make_plan(args.stage, args.seconds, kind, args.motor_id, args.period_policy,
                     args.activation_type2_prefix, preflight_only=args.preflight_only,
                     read_versions=args.read_versions, motor_ids=args.motor_ids,
                     off_first_id=args.off_first_id)
    if not args.execute_no_motion:
        print(json.dumps(plan, indent=2)); return 0
    require(args.known_reporting_off, "A known OFF baseline is required; no assumed restoration")
    require(all((args.front_port, args.rear_port, args.expected_uids,
                 args.expected_boot_id, args.output)), "Live run requires ports, UIDs, boot and output")
    bindings = dual.validate_ports(args.front_port, args.rear_port)
    expected = validate_uids(json.loads(args.expected_uids.read_text()))
    output = args.output.expanduser().resolve()
    require(not any((p / ".git").exists() for p in (output, *output.parents)),
            "Private captures must be outside Git")
    output.mkdir(mode=0o700, exist_ok=False)
    signals, handlers, results, probes = [], {}, {}, {}
    cancel = threading.Event()
    scopes = plan["ids_by_scope"]
    identity_gate = threading.Barrier(len(scopes))
    stop_gate = threading.Barrier(len(scopes))
    report_gate = threading.Barrier(len(scopes))
    lease = boot_guard = None
    coordinator_errors = []
    try:
        boot_guard = BootIdentityGuard()
        require(boot_guard.boot_id == args.expected_boot_id, "Boot identity mismatch")
        for number in (signal.SIGINT, signal.SIGTERM):
            handlers[number] = signal.signal(number, lambda n, _: signals.append(n))
        started = time.monotonic_ns()
        lease = dual.CommonLease(ownership_locks)
        def worker(scope):
            raw = None
            port_stack = ExitStack()
            result = {"status": "INCOMPLETE", "port_closed": False}
            try:
                port_stack.enter_context(dual.port_lock(bindings[scope]["resolved"]))
                def check(cleaning=False):
                    if not cleaning and (signals or cancel.is_set()):
                        raise InterruptedError("Run cancelled")
                    require(dual.binding_matches(bindings[scope]), "Port mapping changed")
                    boot_guard.check()
                check()
                import serial
                raw = serial.Serial(port=None, baudrate=921600, bytesize=8, parity="N",
                    stopbits=1, timeout=0, write_timeout=.1, exclusive=True,
                    xonxoff=False, rtscts=False, dsrdtr=False)
                raw.dtr = raw.rts = False
                raw.port = bindings[scope]["path"]
                raw.open()
                require(os.fstat(raw.fileno()).st_rdev == bindings[scope]["st_rdev"],
                        "Opened device mismatch")
                probe = ActiveProbe(raw, scopes[scope], expected, seconds=args.seconds,
                    expected_kind=kind, period_policy=args.period_policy,
                    activation_type2_prefix=args.activation_type2_prefix, check=check,
                    preflight_only=args.preflight_only, read_versions=args.read_versions,
                    off_first_id=args.off_first_id if args.off_first_id in scopes[scope] else None,
                    identities_ready=lambda: identity_gate.wait(timeout=3),
                    stops_ready=lambda: stop_gate.wait(timeout=3),
                    reports_ready=lambda: report_gate.wait(timeout=3))
                probes[scope] = probe
                result.update(probe.run())
                if result["status"] != plan["success_status"]:
                    cancel.set(); identity_gate.abort(); stop_gate.abort(); report_gate.abort()
            except BaseException as exc:
                result.update(status="INCOMPLETE", failure=repr(exc))
                cancel.set(); identity_gate.abort(); stop_gate.abort(); report_gate.abort()
            finally:
                if raw is not None:
                    try:
                        raw.close()
                        result["port_closed"] = not raw.is_open
                    except BaseException as exc:
                        result.update(status="INCOMPLETE", close_failure=repr(exc))
                else:
                    result["port_closed"] = True
                if result["port_closed"]:
                    try:
                        port_stack.close()
                        lease.release()
                    except BaseException as exc:
                        result.update(status="INCOMPLETE", lock_release_failure=repr(exc))
                else:
                    # Never release the per-port/common lease before close.
                    _HELD_LOCKS.extend((port_stack, lease))
                results[scope] = result
        threads = []
        try:
            for scope in scopes:
                thread = threading.Thread(target=worker, args=(scope,), name="active-report-"+scope)
                lease.retain()
                try:
                    thread.start()
                except BaseException:
                    lease.release()
                    raise
                threads.append(thread)
            for thread in threads: thread.join()
        except BaseException as exc:
            coordinator_errors.append(repr(exc))
        finally:
            cancel.set(); identity_gate.abort(); stop_gate.abort(); report_gate.abort()
            # Also cover failure while starting a later worker. Bounded I/O
            # and phase budgets finish before ownership can be released.
            for thread in threads: thread.join()
    finally:
        try:
            if lease is not None:
                lease.release()
        finally:
            # Every started worker has joined before the shared descriptor closes.
            try:
                if boot_guard is not None:
                    boot_guard.close()
            except BaseException as exc:
                coordinator_errors.append("Boot monitor close failed: " + repr(exc))
            finally:
                for number, handler in handlers.items(): signal.signal(number, handler)
    complete = (set(results) == set(scopes) and not signals and not coordinator_errors
                and lease is not None and lease.released and
                all(r["status"] == plan["success_status"] and r["port_closed"]
                    for r in results.values()))
    success_status = "PREFLIGHT_COMPLETE" if args.preflight_only else "COMPLETE"
    report = {**FLAGS, "plan": plan, "status": success_status if complete else "INCOMPLETE",
              "results": results, "signals": signals, "coordinator_errors": coordinator_errors,
              "common_locks_released": lease.released if lease else False,
              "boot_id": args.expected_boot_id,
              "elapsed_s": (time.monotonic_ns()-started)/1e9,
              "host_sample_times_only": True, "configuration_change_attempts_recorded": True}
    for scope, probe in probes.items():
        with (output / (scope + "-raw.jsonl")).open("x") as stream:
            for begin, end, chunk in probe.raw_log:
                stream.write(json.dumps(dict(read_started_ns=begin, received_ns=end, hex=chunk.hex()))+"\n")
        (output / (scope + "-tx.json")).write_text(json.dumps(probe.tx_log, indent=2)+"\n")
        with (output / (scope + "-samples.jsonl")).open("x") as stream:
            for sample in probe.receiver.samples:
                row = asdict(sample); row["wire"] = row["wire"].hex()
                stream.write(json.dumps(row)+"\n")
    report["source_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in sorted(Path(__file__).parent.glob("*.py"))}
    (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"status": report["status"], "output": str(output), **FLAGS}))
    return 0 if complete else 2


if __name__ == "__main__":
    raise SystemExit(main())
