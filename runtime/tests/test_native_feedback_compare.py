"""Replay protocol/time/branch validation; mock sessions never open a device."""
import copy
import math
import struct
import unittest

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_feedback_compare as compare
from singularitydog_hw.can_readonly import ATParser, read_request

BOOT = "12345678-1234-1234-1234-123456789abc"
UIDS = {str(i): (bytes([i])*8).hex() for i in range(1, 13)}


def wire_reply(tx, *, position=.25, velocity=.02, fault=0):
    f = ATParser().feed(tx)[0]
    mid = f.destination
    kind = 2 if f.kind == 4 else f.kind
    if kind == 0:
        payload = bytes([mid])*8
    elif kind == 2:
        quantize = lambda value, bound: round((value+bound)/(2*bound)*65535)
        payload = struct.pack(">4H", quantize(position, 12.57), quantize(velocity, 50), 32768, 250)
    else:
        value = position if f.data[:2] == b"\x19\x70" else velocity
        payload = f.data[:4]+struct.pack("<f", value)
    can_id = kind << 24 | fault << 16 | mid << 8 | (0xfe if kind == 0 else 0xfd)
    return b"AT"+((can_id << 3) | 4).to_bytes(4, "big")+b"\x08"+payload+b"\r\n"


def exchange(wires, start, *, position=.25, velocity=.02, fault=0):
    records = (native.Record*len(wires))()
    for n, tx in enumerate(wires):
        r = records[n]
        for key, offset in (("start_ns", 0), ("finish_ns", 10_000), ("read_start_ns", 200_000),
                            ("received_ns", 300_000), ("deadline_ns", 100_000_000)):
            setattr(r, key, start+n*700_000+offset)
        for key, data in (("tx", tx), ("rx", wire_reply(tx, position=position, velocity=velocity, fault=fault))):
            for i, byte in enumerate(data):
                getattr(r, key)[i] = byte
        r.written = r.received = 17
    stats = native.Stats()
    stats.begin_ns, stats.end_ns = records[0].start_ns, records[-1].received_ns
    stats.writes = len(wires)
    return records, stats


def phase(kind, start, *, position=.25, velocity=.02, fault=0):
    out = {}
    for scope, ids in compare.SCOPES.items():
        wires = ([read_request(i) for i in ids] if kind == "identity" else
                 [native.stop_wire(i) for i in ids] if kind == "feedback" else
                 [read_request(i, p) for p in ("position", "velocity") for i in ids])
        out[scope] = native.exchange_evidence(*exchange(wires, start, position=position, velocity=velocity, fault=fault))
    return out


def evidence():
    return {"kind": "native_feedback_comparison", "boot_id": BOOT, "supported_disabled": True,
            "motor_enable_sent": False, "learned_targets_sent": False,
            "identity": phase("identity", 1_000_000_000), "cycles": [{"cycle": 1,
                "before": phase("before", 2_000_000_000), "feedback": phase("feedback", 2_020_000_000),
                "after": phase("after", 2_040_000_000)}]}


class FeedbackReplayTests(unittest.TestCase):
    def setUp(self):
        self.e = evidence()

    def run_report(self):
        return compare.analyze_feedback_comparison(self.e, UIDS)

    def test_static_candidate_agreement_preserves_raw_and_never_validates_scale(self):
        original = copy.deepcopy(self.e)
        result = self.run_report()
        self.assertEqual(self.e, original)
        self.assertEqual(len(result["rows"]), 12)
        self.assertTrue(all(v["all_direct_static_comparisons_agree"] for v in result["per_motor"].values()))
        self.assertFalse(result["dynamic_scale_validated"])
        self.assertFalse(result["approved_for_runtime"])
        self.assertFalse(result["full_controller_50Hz_verified"])
        for row in result["rows"]:
            self.assertLess(abs(row["mod_2pi_position_error_deg"]), .03)
            self.assertIn("before", row["velocity"])
            self.assertIn("host_interval_fraction_range", row["velocity"])

    def test_360_degree_offset_reported_separately_and_never_promoted(self):
        self.e["cycles"][0]["before"] = phase("before", 2_000_000_000, position=.25+2*math.pi)
        self.e["cycles"][0]["after"] = phase("after", 2_040_000_000, position=.25+2*math.pi)
        r = self.run_report()
        for row in r["rows"]:
            self.assertAlmostEqual(row["direct_position_error_deg"], -360, delta=.03)
            self.assertLess(abs(row["mod_2pi_position_error_deg"]), .03)
            self.assertEqual(row["comparison_result"], "STATIC_MODULO_ONLY_BRANCH_UNRESOLVED")
            self.assertFalse(row["branch_adjustment_applied"])
        self.assertFalse(r["per_motor"]["1"]["all_direct_static_comparisons_agree"])

    def test_moving_or_discontinuous_endpoints_are_inconclusive(self):
        self.e["cycles"][0]["after"] = phase("after", 2_040_000_000, position=.50, velocity=.5)
        r = self.run_report()
        self.assertTrue(all(row["comparison_result"] == "INCONCLUSIVE_MOTION_OR_TIMING" for row in r["rows"]))
        self.assertEqual(r["per_motor"]["1"]["static_bracket_samples"], 0)

    def test_real_candidate_difference_is_not_fit_away(self):
        self.e["cycles"][0]["feedback"] = phase("feedback", 2_020_000_000, position=.5, velocity=.5)
        result = self.run_report()
        self.assertTrue(all(r["comparison_result"] == "STATIC_CANDIDATE_DIFFERS" for r in result["rows"]))
        self.assertGreater(result["per_motor"]["1"]["max_abs_velocity_error_rad_s"], .47)
        self.assertGreater(result["per_motor"]["1"]["max_abs_direct_position_error_deg"], 14.)

    def test_nan_type17_is_rejected_before_comparison(self):
        self.e["cycles"][0]["before"] = phase("before", 2_000_000_000, position=float("nan"))
        with self.assertRaisesRegex(ValueError, "value rejected"):
            self.run_report()

    def test_interpolates_velocity_at_host_time_and_keeps_uncertainty(self):
        self.e["cycles"][0]["before"] = phase("before", 2_000_000_000, velocity=.02)
        self.e["cycles"][0]["after"] = phase("after", 2_040_000_000, velocity=.08)
        row = self.run_report()["rows"][0]
        v = row["velocity"]
        self.assertAlmostEqual(v["interpolated_type17"], .02+.06*v["host_midpoint_fraction"], places=7)
        self.assertGreater(v["host_time_linear_model_spread"], 0)
        self.assertLess(v["host_interval_fraction_range"][0], v["host_midpoint_fraction"])
        self.assertGreater(v["host_interval_fraction_range"][1], v["host_midpoint_fraction"])

    def test_zero_position_change_does_not_clear_unstable_velocity_gate(self):
        self.e["cycles"][0]["before"] = phase("before", 2_000_000_000, velocity=-.06)
        self.e["cycles"][0]["after"] = phase("after", 2_040_000_000, velocity=.06)
        original = copy.deepcopy(self.e)
        result = self.run_report()
        self.assertEqual(self.e, original)
        for row in result["rows"]:
            self.assertEqual(row["failed_diagnostic_gates"], ["speed_endpoints_stable"])
            self.assertEqual(row["comparison_result"], "INCONCLUSIVE_MOTION_OR_TIMING")
            self.assertFalse(row["direct_comparison_agrees"])
            self.assertEqual(row["host_endpoint_diagnostics"]["position_finite_difference_rad_s"], 0.)
        values = result["per_motor"]["1"]["all_sample_statistics"]
        speed = values["type17_velocity_endpoints_rad_s"]
        self.assertEqual(speed["samples"], 2)
        self.assertEqual(speed["mean"], 0.)
        self.assertAlmostEqual(speed["rms"], .06)
        self.assertAlmostEqual(speed["population_std"], .06)
        self.assertFalse(values["affects_comparison_result"])
        self.assertFalse(result["approved_for_runtime"])
        self.assertFalse(result["dynamic_scale_validated"])

    def test_position_finite_difference_uses_position_host_interval_and_keeps_read_offsets(self):
        self.e["cycles"][0]["after"] = phase("after", 2_040_000_000, position=.251, velocity=.08)
        # Delay only the velocity requests, preserving every causal interval.
        for exchange in self.e["cycles"][0]["after"].values():
            for record in exchange["records"][6:]:
                for name in ("start_ns", "finish_ns", "read_start_ns", "received_ns", "deadline_ns"):
                    record[name] += 600_000
        row = self.run_report()["rows"][0]
        diagnostic = row["host_endpoint_diagnostics"]
        self.assertEqual(diagnostic["position_midpoint_delta_ns"], 40_000_000)
        self.assertEqual(diagnostic["velocity_midpoint_delta_ns"], 40_600_000)
        self.assertEqual(diagnostic["velocity_minus_position_before_midpoint_ns"], 4_200_000)
        self.assertEqual(diagnostic["velocity_minus_position_after_midpoint_ns"], 4_800_000)
        self.assertAlmostEqual(diagnostic["position_finite_difference_rad_s"], .001/.04, places=6)
        self.assertFalse(diagnostic["position_and_velocity_read_together"])
        self.assertFalse(diagnostic["sensor_sample_time_verified"])
        self.assertFalse(diagnostic["affects_comparison_result"])
        self.assertIn("not sensor-time velocity", diagnostic["scope"])
        self.assertEqual(row["comparison_result"], "STATIC_CANDIDATE_AGREES")

    def test_all_sample_statistics_include_inconclusive_cycles_and_separate_type2(self):
        self.e["cycles"] = []
        for number, (before_v, feedback_v, after_v) in enumerate(
                ((0., 0., 0.), (-.06, .04, .06), (-.02, -.03, .02)), 1):
            start = 2_000_000_000+(number-1)*100_000_000
            self.e["cycles"].append({"cycle": number,
                "before": phase("before", start, velocity=before_v),
                "feedback": phase("feedback", start+20_000_000, velocity=feedback_v),
                "after": phase("after", start+40_000_000, velocity=after_v)})
        result = self.run_report()
        motor = result["per_motor"]["1"]
        self.assertEqual(motor["static_bracket_samples"], 2)
        self.assertFalse(motor["all_direct_static_comparisons_agree"])
        diagnostics = motor["all_sample_statistics"]
        self.assertTrue(diagnostics["all_cycles_included"])
        self.assertFalse(diagnostics["sample_filtering_applied"])
        self.assertEqual(diagnostics["type17_position_endpoints_rad"]["samples"], 6)
        self.assertEqual(diagnostics["type17_position_endpoints_rad"]["range"], 0.)
        speed = diagnostics["type17_velocity_endpoints_rad_s"]
        self.assertEqual(speed["samples"], 6)
        self.assertEqual(speed["mean"], 0.)
        self.assertAlmostEqual(speed["rms"], math.sqrt((2*.06**2+2*.02**2)/6))
        feedback = diagnostics["type2_feedback_velocity_rad_s_candidate"]
        self.assertEqual(feedback["samples"], 3)
        decoded = [round((v+50)/100*65535)*100/65535-50 for v in (0., .04, -.03)]
        self.assertAlmostEqual(feedback["mean"], sum(decoded)/3)
        self.assertAlmostEqual(feedback["rms"], math.sqrt(sum(v*v for v in decoded)/3))
        self.assertEqual(diagnostics["failed_gate_counts"], {
            "short_host_bracket": 0, "position_endpoints_stable": 0,
            "speed_endpoints_small": 0, "speed_endpoints_stable": 1})

    def test_failed_gate_names_retain_every_failed_existing_condition(self):
        self.e["cycles"][0]["after"] = phase("after", 2_200_000_000, position=.50, velocity=.5)
        result = self.run_report()
        expected = sorted(("short_host_bracket", "position_endpoints_stable",
                           "speed_endpoints_small", "speed_endpoints_stable"))
        for row in result["rows"]:
            self.assertEqual(row["failed_diagnostic_gates"], expected)
            self.assertEqual(row["comparison_result"], "INCONCLUSIVE_MOTION_OR_TIMING")
            self.assertFalse(row["endpoint_stationarity_heuristic_passed"])

    def test_uid_mismatch_and_context_rejected(self):
        bad = dict(UIDS);bad["1"] = "ff"*8
        with self.assertRaisesRegex(ValueError, "UID"):
            compare.analyze_feedback_comparison(self.e, bad)
        self.e["supported_disabled"] = False
        with self.assertRaisesRegex(ValueError, "context"):
            self.run_report()

    def test_missing_duplicate_crossbus_fault_and_malformed_rejected(self):
        for mutation in ("missing", "duplicate", "crossbus", "fault", "malformed"):
            with self.subTest(mutation=mutation):
                self.e = evidence()
                records = self.e["cycles"][0]["feedback"]["front"]["records"]
                if mutation == "missing": records.pop()
                if mutation == "duplicate": records[-1] = copy.deepcopy(records[0])
                if mutation == "crossbus": records[0] = copy.deepcopy(self.e["cycles"][0]["feedback"]["rear"]["records"][0])
                if mutation == "fault": records[0]["rx_hex"] = wire_reply(native.stop_wire(1), fault=1).hex()
                if mutation == "malformed": records[0]["rx_hex"] = "00"+records[0]["rx_hex"][:-2]
                with self.assertRaises(ValueError): self.run_report()

    def test_long_bracket_not_reported_static_agreement(self):
        self.e["cycles"][0]["after"] = phase("after", 2_200_000_000)
        self.assertTrue(all(row["comparison_result"] == "INCONCLUSIVE_MOTION_OR_TIMING" for row in self.run_report()["rows"]))

    def test_reordered_phases_and_noncausal_records_rejected(self):
        self.e["cycles"][0]["feedback"] = phase("feedback", 1_900_000_000)
        with self.assertRaisesRegex(ValueError, "chronology"):
            self.run_report()
        self.e = evidence()
        r = self.e["cycles"][0]["before"]["front"]["records"][0]
        r["received_ns"] = r["start_ns"]-1
        with self.assertRaisesRegex(ValueError, "noncausal"):
            self.run_report()


class FakeSession:
    def __init__(self, first_id, *, fail_at=None, corrupt_uid=False):
        self.first_id, self.stop_proxy, self.boot_id, self.boot_fd = first_id, True, BOOT.encode(), 1
        self.calls, self.fail_at, self.corrupt_uid = [], fail_at, corrupt_uid

    def exchange(self, wires):
        self.calls.append(wires)
        n = len(self.calls)
        if self.fail_at == n:
            r, s = exchange(wires, 1_000_000_000+n*1_000_000_000)
            r[0].received = 0
            raise native.ExchangeError("injected failure", r, s)
        stamp = 1_000_000_000 if n == 1 else 2_000_000_000+(n-2)*20_000_000
        records, stats = exchange(wires, stamp)
        if n == 1 and self.corrupt_uid:
            records[0].rx[7] ^= 1
        return records, stats


class FeedbackCollectTests(unittest.TestCase):
    def sessions(self, **kwargs):
        return {"front": FakeSession(1, **kwargs), "rear": FakeSession(7)}

    def test_three_phases_and_saved_replay_match(self):
        sessions = self.sessions()
        report, raw = compare.collect_feedback_comparison(sessions, UIDS, boot_id=BOOT, supported_disabled=True, cycles=3)
        self.assertEqual(report["status"], "COMPLETE_DIAGNOSTIC")
        self.assertEqual(len(sessions["front"].calls), 10)
        self.assertEqual([len(w) for w in sessions["front"].calls], [6,12,6,12,12,6,12,12,6,12])
        for scope, session in sessions.items():
            ids = compare.SCOPES[scope]
            identity = [read_request(i) for i in ids]
            parameters = [read_request(i, p) for p in ("position", "velocity") for i in ids]
            feedback = [native.stop_wire(i) for i in ids]
            self.assertEqual(session.calls, [identity]+[parameters, feedback, parameters]*3)
        self.assertEqual(report["per_motor"], compare.analyze_feedback_comparison(raw, UIDS)["per_motor"])
        kinds = {ATParser().feed(w)[0].kind for s in sessions.values() for batch in s.calls for w in batch}
        self.assertEqual(kinds, {0,4,17})

    def test_failure_preserves_both_bus_evidence_without_retry_or_next_phase(self):
        sessions = self.sessions(fail_at=3)
        report, raw = compare.collect_feedback_comparison(sessions, UIDS, boot_id=BOOT, supported_disabled=True)
        self.assertEqual(report["status"], "ABORTED")
        self.assertEqual(len(sessions["front"].calls), 3)
        self.assertEqual(len(sessions["rear"].calls), 3)
        self.assertIn("failed_native_exchange", raw["cycles"][0]["feedback"]["front"])
        self.assertIn("records", raw["cycles"][0]["feedback"]["rear"])
        self.assertEqual(raw["cycles"][0]["after"], {})

    def test_identity_barrier_blocks_stop_on_mismatch(self):
        sessions = self.sessions(corrupt_uid=True)
        report, raw = compare.collect_feedback_comparison(sessions, UIDS, boot_id=BOOT, supported_disabled=True)
        self.assertEqual(report["status"], "ABORTED")
        self.assertEqual(raw["cycles"], [])
        self.assertEqual(len(sessions["front"].calls), 1)

    def test_no_io_without_context_or_matching_native_session_boot(self):
        sessions = self.sessions()
        with self.assertRaisesRegex(ValueError, "supported"):
            compare.collect_feedback_comparison(sessions, UIDS, boot_id=BOOT, supported_disabled=False)
        sessions["front"].boot_id = b"wrong"
        with self.assertRaisesRegex(ValueError, "binding"):
            compare.collect_feedback_comparison(sessions, UIDS, boot_id=BOOT, supported_disabled=True)
        self.assertFalse(sessions["front"].calls)


if __name__ == "__main__":
    unittest.main()
