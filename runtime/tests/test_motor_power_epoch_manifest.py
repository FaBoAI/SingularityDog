"""File-only tests for operator-attested 40 V epoch binding."""

import copy
import hashlib
import json
import math
import unittest

from singularitydog_hw import motor_power_epoch_manifest as epoch


IDS = tuple(str(i) for i in range(1, 13))
UIDS = {mid: f"{int(mid):016x}" for mid in IDS}


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def capture(time_base, *, shifted=False, generic=False):
    identities = {mid: {"mcu_uid_hex": UIDS[mid],
                        "request_monotonic_ns": time_base + 2 * int(mid),
                        "reply_monotonic_ns": time_base + 2 * int(mid) + 1}
                  for mid in IDS}
    rows, direct = {}, {}
    for mid in IDS:
        raw = int(mid) / 10
        if shifted and mid == "3":
            raw += 2 * math.pi + math.radians(2)
        if shifted and mid == "9":
            raw += math.radians(25)  # A physically changed, unreviewed shaft.
        samples = [{"rad": raw + n * .0001,
                    "request_monotonic_ns": time_base + 100 + 10 * int(mid) + n * 2,
                    "reply_monotonic_ns": time_base + 101 + 10 * int(mid) + n * 2}
                   for n in range(3)]
        rows[mid] = {"run_mode": 0, "current": 0.0, "voltage": 40.0,
                     "position_samples": samples,
                     "median_position_rad": raw + .0001,
                     "position_span_deg": math.degrees(.0002)}
        delta = (raw + .0001) - (int(mid) / 10 + .0001)
        direct[mid] = {"direct_delta_rad": delta,
                       "direct_delta_deg": math.degrees(delta)}
    result = {"schema": (epoch.GENERIC_CAPTURE_SCHEMA if generic else epoch.CAPTURE_SCHEMA),
            "status": "RECORDED_REVIEW_REQUIRED", "errors": [],
            "boot_id": "jetson-boot-1",
            "plan": {"allowed_can_types": [0, 17], "automatic_retry": False,
                     "motor_output_available": False},
            "identities": identities,
            "telemetry": {"rows": rows,
                          "started_monotonic_ns": time_base + 100,
                          "ended_monotonic_ns": time_base + 300},
            "angle_wrap_applied": False,
            "stop_state": "UNVERIFIED_BY_READ_ONLY_PROTOCOL",
            "motor_output_allowed": False, "approved_for_runtime": False}
    if not generic:
        result["baseline_sha256"] = "a" * 64
        result["direct_delta_by_id"] = direct
    return result


def events(reference_bytes, current_bytes):
    return {"schema": epoch.EVENT_SCHEMA, "boot_id": "jetson-boot-1",
            "clock_source": "Jetson time.monotonic_ns",
            "motor_output_allowed": False,
            "reference_capture_sha256": hashlib.sha256(reference_bytes).hexdigest(),
            "current_capture_sha256": hashlib.sha256(current_bytes).hexdigest(),
            "events": [
                {"phase": "reference_on", "observed_monotonic_ns": 900,
                 "operator_reported_external_bus_voltage_v": 40.0,
                 "physical_switch_observation": "meter on motor rail, supply on",
                 "operator_confirmed": True, "external_evidence_sha256": "1" * 64},
                {"phase": "supply_off", "observed_monotonic_ns": 1400,
                 "operator_reported_external_bus_voltage_v": .1,
                 "physical_switch_observation": "meter on motor rail, supply off",
                 "operator_confirmed": True, "external_evidence_sha256": "2" * 64},
                {"phase": "current_on", "observed_monotonic_ns": 1600,
                 "operator_reported_external_bus_voltage_v": 40.1,
                 "physical_switch_observation": "meter on motor rail, supply restored",
                 "operator_confirmed": True, "external_evidence_sha256": "3" * 64}],
            "no_full_physical_turn_by_id": {"3": True},
            "physical_pose_observation_by_id": {
                "3": "Continuous ID3 output mark video shows no full revolution"},
            "physical_pose_evidence_sha256_by_id": {"3": "4" * 64},
            "physical_pose_observation_interval_ns_by_id": {
                "3": {"start_ns": 1300, "end_ns": 1650}}}


class EpochManifestTests(unittest.TestCase):
    def setUp(self):
        self.reference = capture(1000)
        self.current = capture(2000, shifted=True)
        self.ref_bytes = encoded(self.reference)
        self.cur_bytes = encoded(self.current)
        self.events = events(self.ref_bytes, self.cur_bytes)

    def build(self, *, reference=None, current=None, operator_events=None):
        return epoch.build_manifest(encoded(reference or self.reference),
                                    encoded(current or self.current),
                                    encoded(operator_events or self.events))

    def test_manifest_binds_two_epochs_and_only_id3_branch(self):
        result = self.build()
        self.assertEqual(result["status"], "OPERATOR_ATTESTED_REVIEW_REQUIRED")
        self.assertNotEqual(result["reference_snapshot"]["motor_power_epoch"],
                            result["current_snapshot"]["motor_power_epoch"])
        self.assertEqual(result["branch_comparison"]["rows"]["3"]
                         ["branch_turns_for_comparison"], 1)
        self.assertFalse(result["branch_comparison"]["rows"]["9"]["branch_reviewed"])
        self.assertFalse(result["motor_power_state_software_sensed"])
        self.assertFalse(result["external_voltage_software_verified"])
        self.assertFalse(result["approved_for_runtime"])
        self.assertFalse(result["motor_output_allowed"])

    def test_generic_same_boot_captures_need_no_historical_baseline(self):
        reference = capture(1000, generic=True)
        current = capture(2000, shifted=True, generic=True)
        proof = events(encoded(reference), encoded(current))
        result = self.build(reference=reference, current=current, operator_events=proof)
        self.assertEqual(result["branch_comparison"]["rows"]["3"]
                         ["branch_turns_for_comparison"], 1)

    def test_malformed_event_and_capture_chronology_rejected(self):
        bad_events = copy.deepcopy(self.events)
        bad_events["events"][1]["observed_monotonic_ns"] = 1250
        with self.assertRaisesRegex(ValueError, "chronology"):
            self.build(operator_events=bad_events)
        bad_capture = copy.deepcopy(self.current)
        bad_capture["identities"]["7"]["request_monotonic_ns"] = 2500
        bad_capture["identities"]["7"]["reply_monotonic_ns"] = 2501
        with self.assertRaisesRegex(ValueError, "chronology"):
            self.build(current=bad_capture)

    def test_incomplete_uid_set_rejected(self):
        incomplete = copy.deepcopy(self.current)
        del incomplete["identities"]["12"]
        with self.assertRaisesRegex(ValueError, "twelve fresh Type0 identities"):
            self.build(current=incomplete)

    def test_capture_hash_mismatch_rejected(self):
        bad = copy.deepcopy(self.events)
        bad["current_capture_sha256"] = "f" * 64
        with self.assertRaisesRegex(ValueError, "capture hash mismatch"):
            self.build(operator_events=bad)

    def test_previous_jetson_boot_cannot_share_monotonic_epoch(self):
        old_boot = copy.deepcopy(self.reference)
        old_boot["boot_id"] = "previous-boot"
        proof = events(encoded(old_boot), self.cur_bytes)
        with self.assertRaisesRegex(ValueError, "capture Jetson boot"):
            self.build(reference=old_boot, operator_events=proof)

    def test_missing_or_noncontinuous_physical_no_turn_evidence_rejected(self):
        bad = copy.deepcopy(self.events)
        del bad["physical_pose_evidence_sha256_by_id"]["3"]
        with self.assertRaisesRegex(ValueError, "no-full-turn evidence"):
            self.build(operator_events=bad)
        bad = copy.deepcopy(self.events)
        bad["physical_pose_observation_interval_ns_by_id"]["3"]["end_ns"] = 1500
        with self.assertRaisesRegex(ValueError, "must span Off/On"):
            self.build(operator_events=bad)


if __name__ == "__main__":
    unittest.main()
