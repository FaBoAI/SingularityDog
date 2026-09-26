"""Finite, deterministic port/reader tests; never import or open real serial I/O."""
import contextlib
import io
import json
import os
from pathlib import Path
import struct
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import active_report_probe as probe
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.serial_deadline_reader import ReceivedChunk


MS = 1_000_000
UIDS = {mid: f"{mid:016x}" for mid in range(1, 13)}


def wire(kind, mid, data, *, destination=0xFD, mode=0, fault=0):
    can_id = kind << 24 | mode << 22 | fault << 16 | mid << 8 | destination
    return b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + bytes([len(data)]) + data + b"\r\n"


def feedback(mid, *, mode=0, kind=2):
    return wire(kind, mid, struct.pack(">4H", 32768, 32768, 32768, 250), mode=mode)


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now


class FakePort:
    """Model query replies, autonomous reporting and uncertain write side effects."""
    def __init__(self, clock, *, period=3, failure=None, report_kind=2,
                 activation_prefix=False):
        self.clock, self.failure = clock, failure
        self.report_kind, self.activation_prefix = report_kind, activation_prefix
        self.periods = dict.fromkeys(range(1, 13), period)
        self.periodic_emitted = dict.fromkeys(range(1, 13), 0)
        self.reporting = {}
        self.pending = []
        self.writes = []
        self.write_timeout = .1
        self.failed_once = False

    def write(self, data):
        frames = ATParser().feed(data)
        if len(frames) != 1 or len(data) != 17:
            raise AssertionError("Only one canonical request is expected")
        frame = frames[0]
        mid, kind = frame.destination, frame.kind
        if kind in (1, 3):
            raise AssertionError("Motion/enable must never reach a serial boundary")
        index = int.from_bytes(frame.data[:2], "little") if kind in (17, 18) else None
        event = {"kind": kind, "mid": mid, "index": index, "wire": data,
                 "at": self.clock(), "period": None}
        self.writes.append(event)
        self.clock.now += 50_000
        answer = None
        if kind == 0:
            answer = wire(0, mid, bytes.fromhex(UIDS[mid]), destination=0xFE)
            if self.failure == "malformed_identity":
                answer = b"bad" + answer
            elif self.failure == "wrong_uid":
                answer = wire(0, mid, b"\xff" * 8, destination=0xFE)
        elif kind == 17 and index == 0x7026:
            answer = wire(17, mid, struct.pack("<HHHH", index, 0, self.periods[mid], 0))
        elif kind == 17 and index == 0x701C:
            answer = wire(17, mid, struct.pack("<H2xf", index,
                          32.8 if self.failure == "low_voltage" and mid == 1 else 40.0))
        elif kind == 18 and index == 0x7026:
            value = int.from_bytes(frame.data[4:6], "little")
            event["period"] = value
            self.periods[mid] = value
            answer = feedback(mid)
        elif kind == 4 and data == probe.version_request(mid):
            answer = wire(2, mid + 1 if self.failure == "wrong_version_id" else mid,
                          b"\x00\xc4\x56\x01\x02\x03\x04\xab",
                          mode=1 if self.failure == "running_version" else 0,
                          fault=1 if self.failure == "fault_version" else 0)
            if self.failure == "normal_version_reply":
                answer = feedback(mid)
            elif self.failure == "malformed_version":
                answer = answer[:-2] + b"xx"
            elif self.failure == "version_timeout":
                answer = None
        elif kind == 4 and frame.data == bytes(8):
            answer = feedback(mid, mode=1 if self.failure == "running_stop" else 0)
            if self.failure == "preflight_final_noise":
                self.pending.append((self.clock() + 2 * MS, b"AT"))
        elif kind == 24:
            enabled = frame.data[6]
            event["enabled"] = enabled
            if enabled:
                self.reporting[mid] = self.clock() + 10 * MS
                if self.activation_prefix:
                    answer = feedback(mid + 1 if self.failure == "wrong_activation_id" else mid,
                                      mode=1 if self.failure == "running_activation" else 0)
            elif self.failure != "off_ignored":
                self.reporting.pop(mid, None)
                if self.activation_prefix and self.failure != "missing_deactivation_prefix":
                    answer = feedback(mid)
                if self.failure == "off_residual":
                    answer = (answer or b"") + feedback(mid, kind=24)[:5]
        else:
            raise AssertionError(f"Unexpected request: {frame.record()}")
        # A partial write may still alter the peer. Do not generate an ACK for
        # this uncertainty: cleanup must first establish a new quiet boundary.
        partial = ((self.failure in ("partial_on", "exception_on") and kind == 24 and frame.data[6] == 1)
                   or (self.failure in ("partial_period", "exception_period") and kind == 18))
        if partial and not self.failed_once:
            self.failed_once = True
            if self.failure.startswith("exception"):
                raise OSError("write failed after uncertain peer acceptance")
            return len(data) - 1
        if answer is not None:
            delay = probe.QUERY_NS + MS if self.failure == "late_version" and data == probe.version_request(mid) else MS
            self.pending.append((self.clock() + delay, answer))
        return len(data)


class FakeReader:
    def __init__(self, raw, *, clock, check):
        self.raw, self.clock, self.check = raw, clock, check

    def read_until(self, wake, hard):
        self.check()
        if self.raw.failure == "persistent_off_read_error" and any(
                row["kind"] == 24 and row["enabled"] == 0 for row in self.raw.writes):
            raise OSError("receive unavailable after OFF")
        if self.raw.failure == "read_error" and self.raw.reporting and not self.raw.failed_once:
            self.raw.failed_once = True
            raise OSError("injected receive failure")
        if self.clock() >= hard:
            raise TimeoutError("hard deadline")
        due = [when for when, _ in self.raw.pending] + list(self.raw.reporting.values())
        next_event = min(due, default=wake)
        self.clock.now = max(self.clock(), min(wake, next_event))
        self.check()
        chunks = []
        remaining = []
        for when, data in self.raw.pending:
            if when <= self.clock():
                chunks.append(data)
            else:
                remaining.append((when, data))
        self.raw.pending = remaining
        for mid, when in list(self.raw.reporting.items()):
            if when <= self.clock():
                chunks.append(feedback(mid, kind=self.raw.report_kind,
                                       mode=1 if self.raw.failure == "running_stream" else 0))
                self.raw.periodic_emitted[mid] += 1
                self.raw.reporting[mid] = when + 10 * MS
        return b"".join(chunks), self.clock()


class ActiveProbeTests(unittest.TestCase):
    def test_reader_failure_evidence_is_saved_once_without_preflight_success(self):
        for timestamp_known in (True, False):
            with self.subTest(timestamp_known=timestamp_known):
                instance, port, _ = self.build(preflight_only=True, period_policy="observe-current",
                                               expected_kind=None)
                old_read = instance.reader.read_until
                error = KeyboardInterrupt("post-read cancellation")
                evidence = []
                def rejected_read(wake, hard):
                    chunk, stamp = old_read(wake, hard)
                    if chunk:
                        item = ReceivedChunk(chunk, stamp, stamp if timestamp_known else None)
                        evidence.append(item)
                        error.serial_read_evidence = item
                        raise error
                    return chunk, stamp
                instance.reader.read_until = rejected_read
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("post-read cancellation", result["failure"])
                self.assertEqual(len(port.writes), 1)
                self.assertEqual(instance.raw_bytes, len(evidence[0].data))
                self.assertEqual(result["rejected_receive_chunks"], 1)
                self.assertFalse(result.get("identities_verified", False))
                self.assertEqual(len(instance.receiver.samples), 0)
                if timestamp_known:
                    self.assertEqual(len(instance.raw_log), 1)
                    self.assertEqual(instance.raw_log[0][1:], (evidence[0].received_ns, evidence[0].data))
                else:
                    self.assertEqual(instance.raw_log, [])
                    self.assertEqual(result["unclocked_receive_evidence"], [evidence[0].record()])

    def build(self, *, ids=(1,), period=3, failure=None, check=None,
              period_policy="set-10ms", expected_kind=2, report_kind=2,
              activation_type2_prefix=False, activation_prefix=False,
              preflight_only=False, read_versions=False, off_first_id=None):
        clock = Clock()
        port = FakePort(clock, period=period, failure=failure, report_kind=report_kind,
                        activation_prefix=activation_prefix)
        kwargs = {"seconds": 1, "clock": clock, "reader_factory": FakeReader,
                  "expected_kind": expected_kind, "period_policy": period_policy,
                  "activation_type2_prefix": activation_type2_prefix,
                  "preflight_only": preflight_only, "read_versions": read_versions,
                  "off_first_id": off_first_id}
        if check is not None:
            kwargs["check"] = check
        instance = probe.ActiveProbe(port, ids, UIDS, **kwargs)
        return instance, port, clock

    def assert_only_safe_writes(self, instance):
        for row in instance.tx_log:
            frame, = ATParser().feed(bytes.fromhex(row["wire_hex"]))
            self.assertNotIn(frame.kind, (1, 3))
            if row["cleanup"]:
                self.assertIn(row["action"], ("report_off", "period_set", "period_read", "stop"))
                self.assertNotEqual(row["action"], "report_on")

    def test_successful_finite_stream_restores_captured_period_and_off(self):
        instance, port, clock = self.build()
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        self.assertTrue(result["timed_interval_completed"])
        self.assertEqual(result["measure_end_ns"] - result["measure_start_ns"], 1_000_000_000)
        self.assertTrue(result["stream"]["ok"])
        self.assertGreaterEqual(result["stream"]["ids"]["1"]["frame_count"], 90)
        self.assertTrue(result["cleanup"]["ok"])
        self.assertEqual(result["cleanup"]["periods_restored"], {1: 3})
        self.assertEqual(port.periods[1], 3)
        self.assertEqual(port.reporting, {})
        self.assertTrue(result["cleanup"]["reporting_off_restored"])
        self.assertLess(clock() - instance.started, 5_000_000_000)
        self.assert_only_safe_writes(instance)

    def test_existing_ten_ms_period_never_writes_parameter(self):
        instance, port, _ = self.build(period=1)
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        self.assertFalse(any(row["kind"] == 18 for row in port.writes))
        self.assertEqual(result["cleanup"]["periods_restored"], {})

    def test_default_period_policy_rejects_raw_zero_after_stop_before_settings(self):
        instance, port, _ = self.build(period=0)
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("failure", result)
        self.assertEqual(len(result["stop_observations"]), 1)
        self.assertFalse(any(row["kind"] in (18, 24) for row in port.writes))
        self.assertEqual(instance.dirty_periods, set())
        self.assertEqual(instance.reporting_attempted, set())
        self.assertEqual(port.periods[1], 0)

    def test_observe_current_accepts_raw_zero_and_nonzero_without_period_writes(self):
        # The fake device supplies an observed10ms stream independently of its
        # captured raw parameter. This test must not infer a raw0 period formula.
        for raw_period in (0, 1, 3, 65535):
            with self.subTest(raw_period=raw_period):
                instance, port, _ = self.build(period=raw_period, period_policy="observe-current")
                result = instance.run()
                self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
                self.assertTrue(result["cleanup"]["ok"])
                self.assertTrue(result["stream"]["ok"])
                self.assertEqual(result["period_policy"], "observe-current")
                self.assertEqual(result["periods_captured_raw"], {1: raw_period})
                self.assertEqual(result["period_encoding_qualified_by_id"], {"1": raw_period >= 1})
                self.assertFalse(any(row["kind"] == 18 for row in port.writes))
                self.assertEqual(port.periods, dict.fromkeys(range(1, 13), raw_period))
                self.assertEqual(instance.dirty_periods, set())
                self.assertEqual(result["cleanup"]["periods_restored"], {})
                self.assertFalse(port.reporting)
                self.assert_only_safe_writes(instance)

    def test_observe_current_keeps_qualification_separate_for_each_id(self):
        instance, port, _ = self.build(ids=(1, 2), period=0, period_policy="observe-current")
        port.periods[2] = 4
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        self.assertEqual(result["periods_captured_raw"], {1: 0, 2: 4})
        self.assertEqual(result["period_encoding_qualified_by_id"], {"1": False, "2": True})
        self.assertEqual((port.periods[1], port.periods[2]), (0, 4))
        self.assertFalse(any(row["kind"] == 18 for row in port.writes))

    def test_observe_current_rejects_direct_period_write_before_io_in_both_phases(self):
        for cleaning, value in ((False, 1), (True, 3)):
            with self.subTest(cleaning=cleaning):
                instance, port, _ = self.build(period_policy="observe-current")
                instance.periods[1] = 3
                instance.cleaning = cleaning
                with self.assertRaises(ValueError):
                    instance._send(1, "period_set", value)
                self.assertEqual(port.writes, [])
                self.assertEqual(instance.tx_log, [])
                self.assertEqual(instance.dirty_periods, set())

    def test_observe_current_plan_has_no_type18_or_proposed_period(self):
        default = probe.make_plan()
        self.assertEqual(default["period_policy"], "set-10ms")
        self.assertIn(18, default["allowed_can_types"])
        observed = probe.make_plan(period_policy="observe-current")
        self.assertEqual(observed["period_policy"], "observe-current")
        self.assertNotIn(18, observed["allowed_can_types"])
        self.assertIsNone(observed.get("report_period_ticks"))
        self.assertIsNone(observed.get("report_period_ms"))

    def test_explicit_activation_prefix_then_type24_observes_raw_zero_without_type18(self):
        instance, port, _ = self.build(period=0, period_policy="observe-current",
            expected_kind=24, report_kind=24, activation_type2_prefix=True, activation_prefix=True)
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        self.assertTrue(result["cleanup"]["ok"])
        self.assertEqual(port.periods[1], 0)
        self.assertFalse(any(row["kind"] == 18 for row in port.writes))
        self.assertFalse(port.reporting)
        self.assertEqual(set(instance.receiver.activation_feedback), {1})
        prefix = instance.receiver.activation_feedback[1]
        self.assertEqual((prefix.kind, prefix.host_sequence), (2, 0))
        self.assertEqual(instance.receiver.activation_pending, {})
        self.assertTrue(all(sample.kind == 24 for sample in instance.receiver.samples))
        self.assertEqual(len(instance.receiver.samples), port.periodic_emitted[1])
        self.assertEqual(instance.receiver.samples[0].host_sequence, 1)
        self.assertEqual(len(result["activation_feedback"]), 1)
        self.assertEqual(result["activation_feedback"][0]["motor_id"], 1)
        self.assertEqual(set(instance.receiver.deactivation_feedback), {1})
        stopped_prefix = instance.receiver.deactivation_feedback[1]
        self.assertEqual((stopped_prefix.kind, stopped_prefix.host_sequence), (2, 0))
        self.assertEqual(instance.receiver.deactivation_pending, {})
        self.assertEqual(len(result["deactivation_feedback"]), 1)
        self.assertEqual(result["deactivation_feedback"][0]["motor_id"], 1)
        self.assertLess(prefix.received_ns, result["measure_start_ns"])
        self.assertGreaterEqual(stopped_prefix.received_ns, result["measure_end_ns"])
        self.assert_only_safe_writes(instance)

    def test_six_activation_prefixes_are_per_id_and_excluded_from_periodic_samples(self):
        ids = tuple(range(1, 7))
        instance, port, _ = self.build(ids=ids, period=0, period_policy="observe-current",
            expected_kind=24, report_kind=24, activation_type2_prefix=True, activation_prefix=True)
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        self.assertEqual(set(instance.receiver.activation_feedback), set(ids))
        self.assertEqual({row["motor_id"] for row in result["activation_feedback"]}, set(ids))
        self.assertEqual({row["motor_id"] for row in result["deactivation_feedback"]}, set(ids))
        self.assertEqual(instance.receiver.activation_pending, {})
        off_writes = [row for row in port.writes if row["kind"] == 24 and row["enabled"] == 0]
        self.assertEqual(len(off_writes), 6)
        for earlier, later in zip(off_writes, off_writes[1:]):
            self.assertGreaterEqual(later["at"] - earlier["at"], 20 * MS)
        self.assertEqual(instance.receiver.deactivation_pending, {})
        self.assertEqual(set(instance.receiver.deactivation_feedback), set(ids))
        self.assertEqual(len(instance.receiver.samples), sum(port.periodic_emitted.values()))
        for mid in ids:
            prefix = instance.receiver.activation_feedback[mid]
            samples = [sample for sample in instance.receiver.samples if sample.motor_id == mid]
            self.assertEqual((prefix.kind, prefix.host_sequence), (2, 0))
            self.assertLess(prefix.received_ns, result["measure_start_ns"])
            self.assertEqual([sample.host_sequence for sample in samples], list(range(1, len(samples) + 1)))
            self.assertTrue(all(sample.kind == 24 for sample in samples))
            self.assertEqual(result["stream"]["detected_kinds"][str(mid)], 24)
            stopped_prefix = instance.receiver.deactivation_feedback[mid]
            self.assertEqual((stopped_prefix.kind, stopped_prefix.host_sequence), (2, 0))
            self.assertGreaterEqual(stopped_prefix.received_ns, result["measure_end_ns"])
        self.assertFalse(any(row["kind"] == 18 for row in port.writes))

    def test_missing_deactivation_prefix_cannot_pass_cleanup_or_reach_final_stop(self):
        instance, port, _ = self.build(period=0, period_policy="observe-current",
            expected_kind=24, report_kind=24, activation_type2_prefix=True,
            activation_prefix=True, failure="missing_deactivation_prefix")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertTrue(result["timed_interval_completed"])
        self.assertFalse(result["cleanup"]["ok"])
        self.assertFalse(any(row["cleanup"] and row["action"] == "stop" for row in instance.tx_log))
        self.assertEqual(instance.receiver.deactivation_feedback, {})
        self.assertFalse(port.reporting)

    def test_persistent_off_read_failure_does_not_burst_remaining_offs(self):
        instance, port, _ = self.build(ids=tuple(range(1, 7)), period=0,
            period_policy="observe-current", expected_kind=24, report_kind=24,
            activation_type2_prefix=True, activation_prefix=True,
            failure="persistent_off_read_error")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertFalse(result["cleanup"]["ok"])
        self.assertEqual(result["cleanup"]["reporting_off_ids_not_attempted"], [2, 3, 4, 5, 6])
        offs = [row for row in port.writes if row["kind"] == 24 and row["enabled"] == 0]
        self.assertEqual([row["mid"] for row in offs], [1])
        self.assertEqual(set(port.reporting), {2, 3, 4, 5, 6})
        self.assertFalse(any(row["kind"] == 18 for row in port.writes))

    def test_activation_prefix_requires_correct_first_feedback_and_still_cleans_up(self):
        for failure, actual_prefix in ((None, False), ("wrong_activation_id", True),
                                       ("running_activation", True)):
            with self.subTest(failure=failure, actual_prefix=actual_prefix):
                instance, port, _ = self.build(period=0, period_policy="observe-current",
                    expected_kind=24, report_kind=24, activation_type2_prefix=True,
                    activation_prefix=actual_prefix, failure=failure)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("failure", result)
                self.assertIsNone(result["measure_start_ns"])
                self.assertEqual(result["cleanup"]["ok"], failure is None)
                self.assertFalse(port.reporting)
                self.assertEqual(port.periods[1], 0)
                cleanup_actions = [row["action"] for row in instance.tx_log if row["cleanup"]]
                self.assertIn("report_off", cleanup_actions)
                self.assertEqual("stop" in cleanup_actions, failure is None)
                self.assertFalse(any(row["kind"] == 18 for row in port.writes))
                self.assert_only_safe_writes(instance)

    def test_type2_prefix_is_not_accepted_without_explicit_option(self):
        instance, port, _ = self.build(period=0, period_policy="observe-current",
            expected_kind=24, report_kind=24, activation_prefix=True)
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("failure", result)
        self.assertIsNone(result["measure_start_ns"])
        self.assertTrue(result["cleanup"]["ok"])
        self.assertFalse(port.reporting)

    def test_activation_prefix_plan_and_constructor_require_explicit_type24(self):
        for expected_kind in (None, 2):
            with self.subTest(expected_kind=expected_kind):
                with self.assertRaises(ValueError):
                    probe.make_plan(expected_kind=expected_kind, activation_type2_prefix=True)
                with self.assertRaises(ValueError):
                    self.build(expected_kind=expected_kind, activation_type2_prefix=True)
        planned = probe.make_plan(expected_kind=24, activation_type2_prefix=True)
        self.assertTrue(planned["activation_type2_prefix"])
        self.assertEqual(planned["expected_report_kind"], 24)
        self.assertFalse(probe.make_plan()["activation_type2_prefix"])

    def test_all_six_periods_are_verified_before_first_reporting_on(self):
        instance, port, _ = self.build(ids=tuple(range(1, 7)))
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        first_on = next(i for i, row in enumerate(instance.tx_log) if row["action"] == "report_on")
        before_on = instance.tx_log[:first_on]
        self.assertEqual({row["motor_id"] for row in before_on if row["action"] == "period_set"}, set(range(1, 7)))
        for mid in range(1, 7):
            actions = [row["action"] for row in before_on if row["motor_id"] == mid]
            self.assertLess(actions.index("period_set"), len(actions) - 1)
            self.assertEqual(actions[-1], "period_read")
        self.assertEqual(port.periods, dict.fromkeys(range(1, 13), 3))

    def test_partial_reporting_on_marks_dirty_and_never_appends_another_wire(self):
        instance, port, _ = self.build(failure="partial_on")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("Partial serial write", result["failure"])
        self.assertEqual(instance.reporting_attempted, {1})
        self.assertEqual(port.writes[-1]["kind"], 24)
        self.assertEqual(port.writes[-1]["enabled"], 1)
        self.assertFalse(any(row["cleanup"] for row in instance.tx_log))
        self.assertFalse(result["cleanup"]["ok"])
        self.assertTrue(port.reporting)
        self.assertEqual(port.periods[1], 1)
        self.assert_only_safe_writes(instance)

    def test_partial_period_write_marks_dirty_and_never_appends_another_wire(self):
        instance, port, _ = self.build(failure="partial_period")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(instance.dirty_periods, {1})
        self.assertFalse(any(row["action"] == "report_on" for row in instance.tx_log))
        self.assertFalse(result["cleanup"]["ok"])
        self.assertEqual(port.periods[1], 1)
        self.assertEqual(port.writes[-1]["kind"], 18)
        self.assertFalse(any(row["cleanup"] for row in instance.tx_log))
        self.assert_only_safe_writes(instance)

    def test_write_exception_with_unknown_peer_state_forbids_all_followup_writes(self):
        for failure, last_kind in (("exception_on", 24), ("exception_period", 18)):
            with self.subTest(failure=failure):
                instance, port, _ = self.build(failure=failure)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("uncertain peer acceptance", result["failure"])
                self.assertEqual(port.writes[-1]["kind"], last_kind)
                self.assertFalse(any(row["cleanup"] for row in instance.tx_log))
                self.assertFalse(result["cleanup"]["ok"])
                self.assertEqual(instance.dirty_periods, {1})
                self.assertEqual(instance.reporting_attempted, {1} if last_kind == 24 else set())

    def test_receive_failure_or_running_stream_still_restores_intact_tx_channel(self):
        for failure in ("read_error", "running_stream"):
            with self.subTest(failure=failure):
                instance, port, _ = self.build(failure=failure)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("failure", result)
                self.assertEqual(result["cleanup"]["ok"], failure == "read_error")
                self.assertFalse(port.reporting)
                self.assertEqual(port.periods[1], 3 if failure == "read_error" else 1)
                self.assertTrue(any(row["cleanup"] and row["action"] == "report_off"
                                    for row in instance.tx_log))
                self.assert_only_safe_writes(instance)

    def test_cancellation_is_latched_but_does_not_skip_bounded_cleanup(self):
        observed = []
        state = {}
        def check(cleaning):
            observed.append(cleaning)
            if not cleaning and state.get("port") and state["port"].reporting:
                raise InterruptedError("cancelled by test")
        instance, port, clock = self.build(check=check)
        state["port"] = port
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("cancelled by test", result["failure"])
        self.assertIn(True, observed)
        self.assertTrue(result["cleanup"]["ok"])
        self.assertFalse(port.reporting)
        self.assertEqual(port.periods[1], 3)
        self.assertLess(clock() - instance.started, 5_000_000_000)
        self.assert_only_safe_writes(instance)

    def test_malformed_identity_wrong_uid_or_running_state_never_configures(self):
        for failure in ("malformed_identity", "wrong_uid", "running_stop"):
            with self.subTest(failure=failure):
                instance, port, _ = self.build(failure=failure)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("failure", result)
                self.assertFalse(any(row["kind"] in (18, 24) for row in port.writes))
                self.assertEqual(port.reporting, {})
                self.assert_only_safe_writes(instance)

    def test_ongoing_stream_after_off_is_not_restored_or_stop_ack(self):
        instance, port, clock = self.build(failure="off_ignored")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertFalse(result["cleanup"]["ok"])
        self.assertFalse(result["cleanup"]["reporting_off_restored"])
        self.assertTrue(result["cleanup"]["errors"])
        self.assertFalse(any(row["cleanup"] and row["action"] in ("stop", "period_set", "period_read")
                             for row in instance.tx_log))
        self.assertEqual(len([row for row in result["stop_observations"] if row["action"] == "stop"]), 1)
        self.assertTrue(port.reporting)
        self.assertLess(clock() - instance.started, 8_000_000_000)

    def test_cleanup_allowlist_rejects_on_identity_voltage_and_wrong_restore(self):
        instance, port, _ = self.build(check=lambda cleaning: None)
        instance.periods[1] = 3
        instance.cleaning = True
        for action, value in (("report_on", None), ("identity", None), ("voltage", None),
                              ("period_set", 1), ("motion", None), ("enable", None)):
            with self.subTest(action=action), self.assertRaises(ValueError):
                instance._send(1, action, value)
        self.assertEqual(port.writes, [])

    def test_partial_off_tail_fails_quiet_boundary_and_prevents_fresh_queries(self):
        for policy in ("observe-current", "set-10ms"):
            with self.subTest(policy=policy):
                instance, port, _ = self.build(failure="off_residual", period_policy=policy,
                    expected_kind=24, report_kind=24, activation_type2_prefix=True,
                    activation_prefix=True)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertTrue(result["timed_interval_completed"])
                self.assertFalse(result["cleanup"]["ok"])
                self.assertTrue(any("Partial stream frame at quiet boundary" in error
                                    for error in result["cleanup"]["errors"]))
                self.assertEqual(bytes(instance.receiver.parser.buffer), feedback(1, kind=24)[:5])
                self.assertFalse(any(row["cleanup"] and row["action"] != "report_off"
                                     for row in instance.tx_log))
                self.assertFalse(port.reporting)
                before = len(port.writes)
                for action in ("identity", "period_read", "period_set", "stop"):
                    with self.subTest(action=action), self.assertRaises(ValueError):
                        instance._send(1, action, 3 if action == "period_set" else None)
                self.assertEqual(len(port.writes), before)

    def test_preflight_only_reads_stops_versions_and_records_unqualified_zero(self):
        instance, port, _ = self.build(ids=(1, 2, 3), period=0,
                                      preflight_only=True, read_versions=True)
        with patch.object(instance, "reports_ready") as ready:
            result = instance.run()
        ready.assert_not_called()
        self.assertEqual(result["status"], "PREFLIGHT_COMPLETE")
        self.assertTrue(result["preflight_completed"])
        self.assertTrue(result["preflight_final_quiet_observed"])
        self.assertEqual(result["periods_captured_raw"], {1: 0, 2: 0, 3: 0})
        self.assertEqual(result["period_encoding_qualified_by_id"], {"1": False, "2": False, "3": False})
        self.assertEqual(set(result["versions_by_id"]), {"1", "2", "3"})
        self.assertEqual(result["versions_by_id"]["3"]["version_bytes_hex"], "01020304")
        self.assertIsNone(result["versions_by_id"]["3"]["semantic_firmware_version"])
        self.assertFalse(any(row["kind"] in (18, 24) for row in port.writes))
        self.assertEqual(instance.reporting_attempted, set())
        self.assertEqual(instance.dirty_periods, set())
        self.assertEqual(instance.receiver.samples, [])
        self.assertIsNone(result["measure_start_ns"])
        self.assertIsNone(result["measure_end_ns"])
        self.assertNotIn("timed_interval_completed", result)
        self.assertNotIn("stream", result)
        for mid in instance.ids:
            actions = [row["action"] for row in instance.tx_log if row["motor_id"] == mid]
            self.assertEqual(actions, ["identity", "stop", "period_read", "voltage", "version"])
        first_voltage = next(i for i, row in enumerate(instance.tx_log)
                             if row["action"] == "voltage")
        self.assertEqual({row["motor_id"] for row in instance.tx_log[:first_voltage]
                          if row["action"] == "stop"}, {1, 2, 3})

    def test_low_voltage_records_other_ids_and_confirms_stops_before_failing(self):
        instance, port, _ = self.build(ids=(1, 2), preflight_only=True,
                                      read_versions=True, failure="low_voltage")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("Voltage outside", result["failure"])
        self.assertEqual(set(result["voltage_by_id"]), {"1", "2"})
        self.assertEqual({row["motor_id"] for row in result["stop_observations"]}, {1, 2})
        self.assertEqual({row["motor_id"] for row in instance.tx_log
                          if row["action"] == "version"}, {1, 2})
        self.assertFalse(any(row["kind"] in (1, 3, 18, 24) for row in port.writes))
        self.assertTrue(result["cleanup"]["ok"])

    def test_preflight_without_version_option_never_queries_version(self):
        instance, port, _ = self.build(preflight_only=True)
        result = instance.run()
        self.assertEqual(result["status"], "PREFLIGHT_COMPLETE")
        self.assertNotIn("versions_by_id", result)
        self.assertFalse(any(row["kind"] in (18, 24) for row in port.writes))
        self.assertTrue(all(ATParser().feed(row["wire"])[0].data == bytes(8)
                            for row in port.writes if row["kind"] == 4))

    def test_preflight_physical_guard_forbids_configuration_and_reporting(self):
        instance, port, _ = self.build(preflight_only=True)
        instance.periods[1] = 3
        for cleaning in (False, True):
            instance.cleaning = cleaning
            for action in ("report_on", "report_off", "period_set"):
                with self.subTest(cleaning=cleaning, action=action), self.assertRaisesRegex(
                        ValueError, "Preflight forbids"):
                    instance._send(1, action, 3 if action == "period_set" else None)
        self.assertEqual(port.writes, [])
        self.assertEqual(instance.tx_log, [])
        self.assertEqual(instance.reporting_attempted, set())
        self.assertEqual(instance.dirty_periods, set())

    def test_preflight_requires_valid_version_and_completed_final_quiet(self):
        for failure in ("wrong_uid", "running_stop", "version_timeout", "late_version",
                        "wrong_version_id", "running_version", "fault_version",
                        "normal_version_reply", "malformed_version", "preflight_final_noise"):
            with self.subTest(failure=failure):
                instance, port, _ = self.build(preflight_only=True, read_versions=True,
                                              failure=failure)
                result = instance.run()
                self.assertEqual(result["status"], "INCOMPLETE")
                self.assertIn("failure", result)
                self.assertFalse(result.get("preflight_completed", False))
                self.assertFalse(result.get("preflight_final_quiet_observed", False))
                self.assertFalse(any(row["kind"] in (18, 24) for row in port.writes))
                self.assertEqual(instance.receiver.samples, [])

    def test_version_reads_are_explicit_after_stop_and_before_any_reporting(self):
        instance, port, _ = self.build(ids=(1, 2), read_versions=True)
        with self.assertRaisesRegex(ValueError, "verified identities"):
            instance._send(1, "version")
        instance.report["identities_verified"] = True
        instance.stop_observations.append({"motor_id": 1, "action": "stop"})
        instance.reporting_attempted.add(2)
        with self.assertRaisesRegex(ValueError, "before any reporting"):
            instance._send(1, "version")
        instance.reporting_attempted.clear()
        instance.cleaning = True
        with self.assertRaisesRegex(ValueError, "before any reporting"):
            instance._send(1, "version")
        self.assertEqual(port.writes, [])
        instance, port, _ = self.build(read_versions=False)
        with self.assertRaisesRegex(ValueError, "explicit selection"):
            instance._send(1, "version")
        self.assertEqual(port.writes, [])

    def test_normal_reporting_can_read_versions_only_in_preflight(self):
        instance, _, _ = self.build(ids=(1, 2), read_versions=True)
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        self.assertEqual(set(result["versions_by_id"]), {"1", "2"})
        first_on = next(i for i, row in enumerate(instance.tx_log) if row["action"] == "report_on")
        self.assertEqual([row["motor_id"] for row in instance.tx_log[:first_on]
                          if row["action"] == "version"], [1, 2])
        self.assertFalse(any(row["action"] == "version" for row in instance.tx_log[first_on:]))

    def test_off_first_orders_actual_attempts_without_changing_selected_ids(self):
        instance, port, _ = self.build(ids=(1, 2, 3), off_first_id=3)
        result = instance.run()
        self.assertEqual(result["status"], "CAN_REPORT_HOST_OBSERVATION_COMPLETE")
        self.assertEqual(result["cleanup"]["reporting_off_order"], [3, 1, 2])
        self.assertEqual([row["mid"] for row in port.writes
                          if row["kind"] == 24 and row["enabled"] == 0], [3, 1, 2])
        self.assertEqual(result["ids"], [1, 2, 3])
        instance, port, _ = self.build(ids=(1, 2, 3), off_first_id=3,
                                      failure="persistent_off_read_error")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(result["cleanup"]["reporting_off_order"], [3])
        self.assertEqual(result["cleanup"]["reporting_off_ids_not_attempted"], [1, 2])
        self.assertEqual(set(port.reporting), {1, 2})
        with self.assertRaises(ValueError):
            self.build(ids=(1, 2, 3), off_first_id=4)
        with self.assertRaises(ValueError):
            self.build(ids=(1, 2, 3), off_first_id=True)

    def test_subset_plans_are_exact_and_omit_empty_bus_workers(self):
        planned = probe.make_plan(stage="front", motor_ids=(1, 2, 3), off_first_id=3)
        self.assertEqual(planned["ids_by_scope"], {"front": [1, 2, 3]})
        self.assertEqual(planned["off_order_by_scope"], {"front": [3, 1, 2]})
        planned = probe.make_plan(stage="both", motor_ids=(7, 9), off_first_id=9)
        self.assertEqual(planned["ids_by_scope"], {"rear": [7, 9]})
        self.assertEqual(planned["off_order_by_scope"], {"rear": [9, 7]})
        planned = probe.make_plan(stage="both", motor_ids=(2, 7), off_first_id=7)
        self.assertEqual(planned["ids_by_scope"], {"front": [2], "rear": [7]})
        self.assertEqual(probe.make_plan(stage="front")["ids_by_scope"], {"front": list(range(1, 7))})
        self.assertEqual(probe.make_plan()["ids_by_scope"], {"front": [1]})
        self.assertEqual(probe.make_plan(motor_id=9, off_first_id=9)["ids_by_scope"], {"rear": [9]})

    def test_subset_and_off_first_validation_rejects_ambiguous_scope(self):
        cases = [("one", (1,)), ("front", ()), ("front", (1, 1)), ("front", (3, 1)),
                 ("front", (1, 7)), ("rear", (6, 7)), ("both", (0, 1)),
                 ("both", (12, 13)), ("front", (True,)), ("front", (1.0,)),
                 ("front", "1,2"), ("front", {1, 2})]
        for stage, ids in cases:
            with self.subTest(stage=stage, ids=ids), self.assertRaises(ValueError):
                probe.make_plan(stage=stage, motor_ids=ids)
        for first in (True, 0, 4, 13, "3"):
            with self.subTest(first=first), self.assertRaises(ValueError):
                probe.make_plan(stage="front", motor_ids=(1, 2, 3), off_first_id=first)

    def test_preflight_subset_cli_is_only_a_plan_with_narrow_physical_types(self):
        output = io.StringIO()
        with patch.object(probe, "ActiveProbe") as active, \
                patch.object(probe.dual, "validate_ports") as ports, \
                patch.object(probe, "ownership_locks") as locks, \
                contextlib.redirect_stdout(output):
            self.assertEqual(probe.main(["--stage", "front", "--motor-ids", "1", "2", "3",
                "--off-first-id", "3", "--preflight-only", "--read-versions"]), 0)
        active.assert_not_called()
        ports.assert_not_called()
        locks.assert_not_called()
        plan = json.loads(output.getvalue())
        self.assertEqual(plan["ids_by_scope"], {"front": [1, 2, 3]})
        self.assertEqual(plan["off_order_by_scope"], {"front": [3, 1, 2]})
        self.assertTrue(plan["preflight_only"])
        self.assertTrue(plan["read_versions"])
        self.assertEqual(plan["allowed_can_types"], [0, 4, 17])
        self.assertEqual(plan["success_status"], "PREFLIGHT_COMPLETE")
        self.assertIsNone(plan["writable_parameter"])
        self.assertIsNone(plan["report_period_ms"])
        for argv in (["--motor-ids", "1"], ["--stage", "front", "--motor-ids", "2", "1"],
                     ["--stage", "rear", "--motor-ids", "1"], ["--off-first-id", "3"]):
            with self.subTest(argv=argv), self.assertRaises(ValueError):
                probe.main(argv)

    def test_default_cli_prints_plan_without_opening_or_claiming_execution(self):
        output = io.StringIO()
        with patch.object(probe, "ActiveProbe") as active, \
                patch.object(probe.dual, "validate_ports") as ports, \
                patch.object(probe, "ownership_locks") as locks, \
                patch.object(probe, "BootIdentityGuard") as guard, \
                contextlib.redirect_stdout(output):
            self.assertEqual(probe.main([]), 0)
        active.assert_not_called()
        ports.assert_not_called()
        locks.assert_not_called()
        guard.assert_not_called()
        plan = json.loads(output.getvalue())
        self.assertEqual(plan["stage"], "one")
        self.assertFalse(plan["motor_enabling_available"])
        self.assertFalse(plan["motion_command_available"])
        self.assertTrue(plan["known_reporting_off_required"])

    def test_observe_current_cli_selects_policy_without_opening_port(self):
        output = io.StringIO()
        with patch.object(probe, "ActiveProbe") as active, \
                patch.object(probe.dual, "validate_ports") as ports, \
                patch.object(probe, "ownership_locks") as locks, \
                contextlib.redirect_stdout(output):
            self.assertEqual(probe.main(["--period-policy", "observe-current", "--seconds", "1"]), 0)
        active.assert_not_called()
        ports.assert_not_called()
        locks.assert_not_called()
        plan = json.loads(output.getvalue())
        self.assertEqual(plan["period_policy"], "observe-current")
        self.assertNotIn(18, plan["allowed_can_types"])
        self.assertIsNone(plan.get("report_period_ticks"))
        self.assertIsNone(plan.get("report_period_ms"))

    def test_activation_prefix_cli_requires_type24_and_remains_a_dry_plan(self):
        output = io.StringIO()
        with patch.object(probe, "ActiveProbe") as active, \
                patch.object(probe.dual, "validate_ports") as ports, \
                contextlib.redirect_stdout(output):
            self.assertEqual(probe.main(["--period-policy", "observe-current", "--seconds", "1",
                                        "--report-kind", "24", "--activation-type2-prefix"]), 0)
        active.assert_not_called()
        ports.assert_not_called()
        plan = json.loads(output.getvalue())
        self.assertTrue(plan["activation_type2_prefix"])
        self.assertEqual(plan["expected_report_kind"], 24)
        for kind in ("discover", "2"):
            with self.subTest(kind=kind), contextlib.redirect_stdout(io.StringIO()), \
                    self.assertRaises(ValueError):
                probe.main(["--report-kind", kind, "--activation-type2-prefix"])


class CleanupRawBoundaryTests(unittest.TestCase):
    build = ActiveProbeTests.build

    @staticmethod
    def receiver_state(receiver):
        return (bytes(receiver.parser.buffer), receiver.poisoned, tuple(receiver.errors),
                tuple(receiver.samples), dict(receiver.activation_pending),
                dict(receiver.activation_feedback), dict(receiver.deactivation_feedback),
                receiver.received_bytes, receiver.unprocessed_bytes, receiver.rejected_wire_hex)

    def failed_split_run(self, *, suffix=b"\r\n", before_replay=None,
                         policy="observe-current", ids=(1,), bad_restore=False):
        instance, port, clock = self.build(ids=ids, period=3, period_policy=policy,
            expected_kind=24, report_kind=24, activation_type2_prefix=True, activation_prefix=True)
        original_read = instance.reader.read_until
        split = False
        def read(wake, hard):
            nonlocal split
            chunk, received = original_read(wake, hard)
            if not split and port.reporting and chunk == feedback(1, kind=24):
                split = True
                port.pending.append((received + MS, suffix))
                return chunk[:-2], received
            return chunk, received
        instance.reader.read_until = read
        def observe():
            if policy == "set-10ms":
                for mid in ids:
                    instance.query(mid, "period_set", 1)
                    instance.query(mid, "period_read")
            instance.receiver.expect_activation(1, clock(), clock() + 5 * MS)
            instance._send(1, "report_on")
            # Missing activation feedback followed by a partial periodic frame
            # reproduces the poisoned 15-byte parser at an expired expectation.
            port.pending.clear()
            chunk, started, received = instance._read(clock() + 20 * MS)
            instance.receiver.feed(chunk, started, received)
            self.fail("Expired activation expectation must abort acquisition")
        instance._observe_reporting = observe
        state = {}
        original_cleanup = instance.cleanup
        def cleanup():
            state["before"] = self.receiver_state(instance.receiver)
            original_cleanup()
        instance.cleanup = cleanup
        original_replay = instance._verify_cleanup_boundary
        def replay():
            if before_replay:
                before_replay(instance)
            return original_replay()
        instance._verify_cleanup_boundary = replay
        if bad_restore:
            original_write = port.write
            def write(data):
                count = original_write(data)
                frame, = ATParser().feed(data)
                if instance.cleaning and frame.kind == 18:
                    when, answer = port.pending[-1]
                    port.pending[-1] = (when, answer[:7])
                return count
            port.write = write
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("activation_prefix_deadline_exceeded", result["failure"])
        self.assertEqual(self.receiver_state(instance.receiver), state["before"])
        self.assertEqual(state["before"][0], feedback(1, kind=24)[:-2])
        self.assertTrue(state["before"][1])
        self.assertIsNone(result["measure_start_ns"])
        return instance, port, result

    def test_failed_partial_frame_completes_in_raw_cleanup_without_repairing_receiver(self):
        instance, _, result = self.failed_split_run()
        self.assertTrue(result["cleanup"]["ok"])
        self.assertTrue(result["cleanup"]["cleanup_boundary_verified"])
        replay = result["cleanup"]["cleanup_boundary_replay"]
        self.assertEqual((replay["raw_bytes"], replay["frames"], replay["residual_bytes"]), (34, 2, 0))
        self.assertEqual([r["action"] for r in instance.tx_log if r["cleanup"]], ["report_off", "stop"])
        self.assertTrue(instance.receiver.activation_pending)
        self.assertEqual(instance.receiver.deactivation_feedback, {})
        self.assertFalse(result["full_pipeline_20ms_verified"])
        with self.assertRaisesRegex(ValueError, "already attempted"):
            instance._verify_cleanup_boundary()

    def test_invalid_or_incomplete_raw_suffix_never_permits_a_new_query(self):
        altered_flags = bytearray(feedback(1, kind=24)); altered_flags[5] ^= 1
        cases = {
            "noise": b"\r\nx", "partial": b"\r", "wrong_id": b"\r\n" + feedback(2),
            "wrong_host": b"\r\n" + wire(24, 1, bytes(8), destination=0xFE),
            "wrong_type": b"\r\n" + wire(17, 1, bytes(8)),
            "flags": b"\r\n" + bytes(altered_flags),
            "dlc": b"\r\n" + wire(24, 1, bytes(7)),
            "running": b"\r\n" + feedback(1, mode=1),
            "fault": b"\r\n" + wire(24, 1, bytes(8), fault=1),
            "version": b"\r\n" + wire(2, 1, b"\x00\xc4\x56" + bytes(5)),
        }
        for name, suffix in cases.items():
            with self.subTest(name=name):
                instance, _, result = self.failed_split_run(suffix=suffix)
                self.assertFalse(result["cleanup"]["ok"])
                self.assertFalse(result["cleanup"]["cleanup_boundary_verified"])
                self.assertEqual([r["action"] for r in instance.tx_log if r["cleanup"]], ["report_off"])

    def test_missing_raw_unknown_clock_overflow_or_incomplete_off_forbids_queries(self):
        def missing(instance):
            instance.raw_log.pop()
        def unknown(instance):
            instance.report["unclocked_receive_evidence"] = [{"received_ns": None, "hex": "0d0a"}]
        def overflow(instance):
            instance.report["cleanup"]["raw_log_overflow"] = True
        def storage_failure(instance):
            instance.report["receive_evidence_storage_failed"] = True
        def incomplete_off(instance):
            instance.tx_log[-1]["returned_bytes"] = None
        def missing_start(instance):
            instance._reporting_raw_start = None
        def backwards(instance):
            _, received, chunk = instance.raw_log[-1]
            instance.raw_log[-1] = (0, received, chunk)
        for fault in (missing, unknown, overflow, storage_failure, incomplete_off, missing_start, backwards):
            with self.subTest(fault=fault.__name__):
                instance, _, result = self.failed_split_run(before_replay=fault)
                self.assertFalse(result["cleanup"]["ok"])
                self.assertFalse(instance.cleanup_boundary_verified)
                self.assertEqual([r["action"] for r in instance.tx_log if r["cleanup"]], ["report_off"])

    def test_partial_new_query_revokes_replayed_boundary_before_next_id_restore(self):
        instance, port, result = self.failed_split_run(policy="set-10ms", ids=(1, 2), bad_restore=True)
        self.assertIn("cleanup_boundary_replay", result["cleanup"])
        self.assertFalse(result["cleanup"]["ok"])
        self.assertFalse(instance.cleanup_boundary_verified)
        self.assertEqual([(r["motor_id"], r["action"]) for r in instance.tx_log if r["cleanup"]],
                         [(1, "report_off"), (1, "period_set")])
        before = len(port.writes)
        with self.assertRaisesRegex(ValueError, "boundary is not verified"):
            instance._send(2, "period_set", 3)
        self.assertEqual(len(port.writes), before)

    def test_pre_on_failure_still_restores_captured_period_without_stream_replay(self):
        instance, port, _ = self.build()
        instance.reports_ready = lambda: (_ for _ in ()).throw(RuntimeError("peer setup failed"))
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertTrue(result["cleanup"]["ok"])
        self.assertEqual(port.periods[1], 3)
        self.assertFalse(instance.reporting_attempted)
        self.assertFalse(instance._cleanup_replay_attempted)

    @unittest.skipUnless(os.environ.get("ACTIVE_REPORT_REGRESSION_CAPTURE"),
                         "Optional private saved-raw regression capture not selected")
    def test_saved_front6_failure_raw_has_a_complete_independent_cleanup_boundary(self):
        # Local offline evidence only; the path and motor UIDs are never emitted.
        capture = Path(os.environ["ACTIVE_REPORT_REGRESSION_CAPTURE"])
        tx = json.loads((capture / "front-tx.json").read_text())
        raw = [(r["read_started_ns"], r["received_ns"], bytes.fromhex(r["hex"]))
               for r in map(json.loads, (capture / "front-raw.jsonl").read_text().splitlines())]
        result = json.loads((capture / "summary.json").read_text())["results"]["front"]
        instance, _, clock = self.build(ids=tuple(range(1, 7)), period=0,
            period_policy="observe-current", expected_kind=24, report_kind=24,
            activation_type2_prefix=True, activation_prefix=True)
        first_on = next(row["started_ns"] for row in tx if row["action"] == "report_on")
        start = next(i for i, row in enumerate(raw) if row[0] >= first_on)
        on = [row for row in tx if row["action"] == "report_on"]
        armed = set()
        for begun, received, chunk in raw[start:]:
            for row in on:
                if row["motor_id"] not in armed and row["started_ns"] <= begun:
                    instance.receiver.expect_activation(row["motor_id"], row["started_ns"],
                                                        row["started_ns"] + probe.QUERY_NS)
                    armed.add(row["motor_id"])
            try:
                instance.receiver.feed(chunk, begun, received)
            except ValueError:
                break
        self.assertEqual(len(instance.receiver.parser.buffer), 15)
        self.assertTrue(instance.receiver.poisoned)
        before = self.receiver_state(instance.receiver)
        instance.raw_log, instance.tx_log = raw, tx
        instance.raw_bytes = sum(len(row[2]) for row in raw)
        instance._reporting_raw_start = start
        instance._reporting_raw_bytes_start = sum(len(row[2]) for row in raw[:start])
        instance.reporting_attempted = set(range(1, 7))
        instance.report["failure"] = result["failure"]
        instance.cleaning = True
        # Empty reads were not archived. This is a test-only completed quiet
        # interval, not a claim inferred from the last raw chunk's timestamp.
        instance._cleanup_quiet_end = clock.now = raw[-1][1] + probe.QUIET_NS
        instance._verify_cleanup_boundary()
        self.assertTrue(instance.cleanup_boundary_verified)
        self.assertEqual(self.receiver_state(instance.receiver), before)
        self.assertEqual(instance.report["status"], "INCOMPLETE")
        self.assertEqual(instance.report["failure"], result["failure"])


class BootGuardLifecycleTests(unittest.TestCase):
    """Exercise live coordinator ownership using fake ports and an owned guard."""
    class Guard:
        boot_id = "boot"

        def __init__(self, *, fail_check=None, fail_close=False):
            self.events = []
            self.checks = self.closes = 0
            self.fail_check, self.fail_close = fail_check, fail_close

        def check(self):
            assert not self.closes, "Check after owned boot FD closed"
            self.checks += 1
            self.events.append("boot-check")
            if self.checks == self.fail_check:
                raise OSError("fresh boot read failed")

        def close(self):
            self.closes += 1
            self.events.append("boot-close")
            if self.fail_close:
                raise OSError("boot close failed")

    def run_fake(self, guard, *, stage="both", fail_second_start=False,
                 setup_failure=None):
        events = guard.events
        bindings = {s: {"path": s, "resolved": s, "st_rdev": 42}
                    for s in ("front", "rear")}
        @contextlib.contextmanager
        def common_lock():
            if setup_failure == "lease":
                raise OSError("lease setup failed")
            events.append("common-open")
            try: yield
            finally: events.append("common-close")
        @contextlib.contextmanager
        def port_lock(scope):
            events.append("port-open-" + scope)
            try: yield
            finally: events.append("port-close-" + scope)
        class Raw:
            def __init__(self, **kwargs): self.is_open = False
            def open(self):
                self.is_open = True
                events.append("open-" + self.port)
            def close(self):
                events.append("close-" + self.port)
                self.is_open = False
            def fileno(self): return 0
        class Preflight:
            def __init__(self, raw, ids, expected, **kwargs):
                self.scope, self.check = raw.port, kwargs["check"]
                self.ready = kwargs["identities_ready"]
                self.raw_log, self.tx_log = [], []
                self.receiver = SimpleNamespace(samples=[])
                assert kwargs["preflight_only"]
            def run(self):
                self.check(False)
                self.ready()
                events.append("cleanup-" + self.scope)
                self.check(True)
                return {"status": "PREFLIGHT_COMPLETE"}
        original_start, starts = threading.Thread.start, 0
        def start(thread):
            nonlocal starts
            starts += 1
            if fail_second_start and starts == 2:
                raise RuntimeError("thread start failed")
            return original_start(thread)
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(probe, "BootIdentityGuard", return_value=guard) as factory, \
             patch.object(probe, "ownership_locks", common_lock), \
             patch.object(probe.dual, "validate_ports", return_value=bindings), \
             patch.object(probe.dual, "port_lock", port_lock), \
             patch.object(probe.dual, "binding_matches", return_value=True), \
             patch.object(probe.os, "fstat", return_value=SimpleNamespace(st_rdev=42)), \
             patch.object(probe.signal, "signal", side_effect=(OSError("signal setup failed")
                 if setup_failure == "signal" else None)), \
             patch.object(probe.threading.Thread, "start", start), \
             patch.object(probe, "ActiveProbe", Preflight), \
             patch.dict("sys.modules", {"serial": SimpleNamespace(Serial=Raw)}), \
             contextlib.redirect_stdout(io.StringIO()):
            uid_path = Path(directory) / "uids.json"
            uid_path.write_text(json.dumps(UIDS))
            output = Path(directory) / "capture"
            rc = probe.main(["--stage", stage, "--preflight-only", "--front-port", "front",
                "--rear-port", "rear", "--expected-uids", str(uid_path),
                "--expected-boot-id", "boot", "--output", str(output),
                "--known-reporting-off", "--execute-no-motion"])
            factory.assert_called_once_with()
            report = json.loads((output / "summary.json").read_text())
        return rc, report

    def test_shared_guard_checks_entry_normal_and_cleanup_until_both_workers_exit(self):
        guard = self.Guard()
        rc, report = self.run_fake(guard)
        self.assertEqual(rc, 0)
        self.assertEqual(report["status"], "PREFLIGHT_COMPLETE")
        self.assertEqual(guard.checks, 6)
        self.assertEqual(guard.closes, 1)
        for scope in ("front", "rear"):
            self.assertLess(guard.events.index("cleanup-" + scope), guard.events.index("boot-close"))
            self.assertLess(guard.events.index("close-" + scope), guard.events.index("boot-close"))

    def test_initial_boot_mismatch_closes_without_opening_serial(self):
        guard = self.Guard()
        guard.boot_id = "different"
        with self.assertRaisesRegex(ValueError, "Boot identity mismatch"):
            self.run_fake(guard)
        self.assertEqual(guard.events, ["boot-close"])
        self.assertEqual(guard.closes, 1)

    def test_fresh_boot_failure_during_cleanup_is_not_success_and_closes(self):
        guard = self.Guard(fail_check=3)
        rc, report = self.run_fake(guard, stage="front")
        self.assertEqual(rc, 2)
        self.assertIn("fresh boot read failed", report["results"]["front"]["failure"])
        self.assertLess(guard.events.index("close-front"), guard.events.index("boot-close"))
        self.assertEqual(guard.closes, 1)

    def test_setup_failures_close_owned_guard(self):
        for failure in ("signal", "lease"):
            guard = self.Guard()
            with self.subTest(failure=failure), self.assertRaises(OSError):
                self.run_fake(guard, setup_failure=failure)
            self.assertEqual(guard.closes, 1)
            self.assertFalse(any(e.startswith("open-") for e in guard.events))

    def test_thread_start_failure_joins_peer_before_boot_close(self):
        guard = self.Guard()
        rc, report = self.run_fake(guard, fail_second_start=True)
        self.assertEqual(rc, 2)
        self.assertTrue(any("thread start failed" in e for e in report["coordinator_errors"]))
        self.assertLess(guard.events.index("port-close-front"), guard.events.index("boot-close"))
        self.assertEqual(guard.closes, 1)

    def test_boot_close_failure_marks_run_incomplete(self):
        guard = self.Guard(fail_close=True)
        rc, report = self.run_fake(guard, stage="front")
        self.assertEqual(rc, 2)
        self.assertEqual(report["status"], "INCOMPLETE")
        self.assertTrue(any("Boot monitor close failed" in e for e in report["coordinator_errors"]))


if __name__ == "__main__":
    unittest.main()
