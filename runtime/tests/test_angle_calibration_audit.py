"""Hardware-free branch/calibration regression tests, including power cycles."""

from dataclasses import replace
import copy
import math
import unittest

from singularitydog_hw.angle_calibration_audit import (
    IDS, TAU, AngleEvidenceError, AxisCalibration, EpochAngleMap,
    audit_twelve_axes, equivalent_branch_candidates, fit_reference_observations,
    resolve_current_branch, resolve_unique_numeric_branch, signed_periodic_delta_rad,
)


def axis(mid=1, **changes):
    base = AxisCalibration(mid, f"{mid:016x}", 1, 0., -.5, .5, .001,
                           "a" * 64, True, True, True,
                           "b" * 64, "c" * 64, "d" * 64)
    return replace(base, **changes)


def snapshot(*, epoch="motor-epoch-A", ns=1_000_000_000, raw=0., digest="e"):
    return {"boot_id": "boot-A", "motor_power_epoch": epoch,
            "epoch_evidence_sha256": "f" * 64,
            "uid_read_boot_id": "boot-A", "uid_read_motor_power_epoch": epoch,
            "uid_capture_sha256": "1" * 64, "capture_sha256": digest * 64,
            "monotonic_ns": ns, "raw_rad_by_id": {str(mid): raw for mid in IDS},
            "uids_by_id": {str(mid): f"{mid:016x}" for mid in IDS},
            "motor_output_allowed": False}


def bound(capture=None, contracts=None):
    return EpochAngleMap(contracts or {mid: axis(mid) for mid in IDS},
                         capture or snapshot(), max_speed_rad_s=1.0,
                         noise_margin_rad=.001, max_sample_gap_s=.1)


class PeriodicMathTests(unittest.TestCase):
    def test_logged_plus_361_degree_power_cycle_and_negative_turn_inverse(self):
        old_raw = -0.04319906234741211
        new_raw = 6.262798309326172  # Recorded ID3 post-charge capture.
        branch = resolve_unique_numeric_branch(
            new_raw, sign=1, offset_rad=0., lower_rad=-.5,
            upper_rad=.5, uncertainty_rad=.001)
        self.assertEqual(branch["turns"], 1)
        self.assertAlmostEqual(math.degrees(branch["model_rad"] - old_raw),
                               1.3070350174863, places=9)
        raw_target = branch["model_rad"] + .02 + branch["turns"] * TAU
        self.assertAlmostEqual(raw_target, new_raw + .02)
        negative = resolve_unique_numeric_branch(
            -TAU + .08, sign=-1, offset_rad=0., lower_rad=-.5,
            upper_rad=.5, uncertainty_rad=.001)
        self.assertEqual(negative["turns"], -1)
        self.assertAlmostEqual(negative["model_rad"], -.08)
        self.assertAlmostEqual((-.06 - 0.) / -1 + negative["turns"] * TAU,
                               -TAU + .06)

    def test_numeric_branch_rejects_limit_uncertainty_and_nonunique_nearest(self):
        with self.assertRaisesRegex(AngleEvidenceError, "uncertainty crosses"):
            resolve_unique_numeric_branch(
                .495, sign=1, offset_rad=0., lower_rad=-.5,
                upper_rad=.5, uncertainty_rad=.01)
        with self.assertRaisesRegex(AngleEvidenceError, "got 2"):
            resolve_unique_numeric_branch(
                math.pi, sign=1, offset_rad=0.,
                lower_rad=-math.pi + .001, upper_rad=math.pi - .001,
                uncertainty_rad=.01)
        with self.assertRaisesRegex(AngleEvidenceError, "narrower than one turn"):
            resolve_unique_numeric_branch(
                0., sign=1, offset_rad=0., lower_rad=-math.pi,
                upper_rad=math.pi, uncertainty_rad=.001)

    def test_359_to_1_is_positive_two_degrees(self):
        self.assertAlmostEqual(math.degrees(signed_periodic_delta_rad(
            math.radians(1), math.radians(359))), 2)
        self.assertAlmostEqual(math.degrees(signed_periodic_delta_rad(
            math.radians(359), math.radians(1))), -2)

    def test_180_ambiguity_does_not_choose_arbitrary_direction(self):
        for value in (math.pi, -math.pi, 3 * math.pi):
            with self.assertRaisesRegex(AngleEvidenceError, "Half-turn"):
                signed_periodic_delta_rad(value, 0)
        self.assertAlmostEqual(signed_periodic_delta_rad(math.pi - 1e-8, 0),
                               math.pi - 1e-8)

    def test_179_to_minus179_has_small_positive_orientation_difference(self):
        self.assertAlmostEqual(math.degrees(signed_periodic_delta_rad(
            math.radians(-179), math.radians(179))), 2)

    def test_nonfinite_and_boolean_inputs_rejected(self):
        for value in (True, float("nan"), float("inf"), "0", 10**999):
            with self.assertRaises(AngleEvidenceError):
                signed_periodic_delta_rad(value, 0)

    def test_multiple_turns_are_reported_by_integer_branch(self):
        for turns in (-3, -1, 0, 1, 3):
            row = resolve_current_branch(turns * TAU + .1, axis())
            self.assertEqual(row["turns"], turns)
            self.assertAlmostEqual(row["model_rad"], .1)

    def test_negative_sign_and_nonzero_zero_offset(self):
        row = resolve_current_branch(TAU + .2, axis(sign=-1, offset_rad=.3))
        self.assertEqual(row["turns"], 1)
        self.assertAlmostEqual(row["model_rad"], .1)

    def test_urdf_only_interval_is_not_mechanical_evidence(self):
        candidate = axis(physical_limits_reviewed=False,
                         physical_limits_evidence_sha256=None)
        self.assertEqual(len(equivalent_branch_candidates(TAU + .1, candidate)), 1)
        with self.assertRaisesRegex(AngleEvidenceError, "MECHANICAL_LIMITS"):
            resolve_current_branch(TAU + .1, candidate)

    def test_unreviewed_zero_or_direction_are_independent(self):
        for field in ("zero_reviewed", "direction_reviewed"):
            with self.assertRaises(AngleEvidenceError):
                resolve_current_branch(.1, replace(axis(), **{field: False}))

    def test_reviewed_zero_cannot_bind_with_numeric_only_zero_uncertainty(self):
        with self.assertRaisesRegex(AngleEvidenceError, "nonzero error bound"):
            axis(uncertainty_rad=0.)
        nominal = axis(uncertainty_rad=0., zero_reviewed=False)
        self.assertEqual(len(equivalent_branch_candidates(.1, nominal)), 1)
        with self.assertRaisesRegex(AngleEvidenceError, "ZERO_ACCURACY"):
            resolve_current_branch(.1, nominal)

    def test_wide_interval_and_uncertain_edge_are_rejected(self):
        broad = axis(lower_rad=-TAU, upper_rad=TAU)
        self.assertEqual(len(equivalent_branch_candidates(0, broad)), 3)
        with self.assertRaisesRegex(AngleEvidenceError, "AT_LEAST_ONE_TURN"):
            resolve_current_branch(0, broad)
        with self.assertRaisesRegex(AngleEvidenceError, "uncertainty crosses"):
            resolve_current_branch(.5, axis())
        with self.assertRaisesRegex(AngleEvidenceError, "got 0"):
            resolve_current_branch(1., axis())

    def test_near_full_turn_uncertainty_can_make_two_candidates(self):
        broad = axis(lower_rad=-math.pi + .001, upper_rad=math.pi - .001,
                     uncertainty_rad=.01)
        self.assertEqual(len(equivalent_branch_candidates(math.pi, broad)), 2)
        with self.assertRaisesRegex(AngleEvidenceError, "got 2"):
            resolve_current_branch(math.pi, broad)


class EpochMapTests(unittest.TestCase):
    def test_all_axes_both_signs_rebind_each_power_cycle_without_changing_zero(self):
        # Exercise a return to the old branch as well as both wrap directions.
        # Each power cycle has its own immutable map; no old raw target is reused.
        for sign in (-1, 1):
            contracts = {mid: axis(mid, sign=sign, offset_rad=.007 * mid)
                         for mid in IDS}
            model_pose = {str(mid): .08 + .002 * mid for mid in IDS}
            for cycle, turn_change in enumerate((0, 1, 0, -1, 0)):
                with self.subTest(sign=sign, cycle=cycle):
                    capture = snapshot(epoch=f"power-cycle-{cycle}", digest=str(cycle + 1))
                    turns = {mid: (mid % 3 - 1) + turn_change for mid in IDS}
                    for mid in IDS:
                        capture["raw_rad_by_id"][str(mid)] = (
                            (model_pose[str(mid)] - contracts[mid].offset_rad) / sign
                            + turns[mid] * TAU)
                    saved = copy.deepcopy(capture)
                    mapper = bound(capture, contracts)
                    report = mapper.report()
                    requested = {key: q + .02 for key, q in model_pose.items()}
                    inverse = mapper.raw_target_candidates(requested, max_delta_rad=.03)
                    for mid in IDS:
                        key = str(mid)
                        self.assertEqual(report["turns_by_id"][key], turns[mid])
                        self.assertAlmostEqual(report["model_rad_by_id"][key], model_pose[key])
                        self.assertAlmostEqual(inverse["raw_target_candidates_rad_by_id"][key],
                                               saved["raw_rad_by_id"][key] + sign * .02)
                    self.assertEqual(capture, saved)
                    self.assertFalse(report["approved_for_runtime"])
                    self.assertFalse(inverse["output_allowed"])

    def test_any_axis_live_positive_or_negative_turn_jump_invalidates_inverse(self):
        for mid in IDS:
            for sign in (-1, 1):
                for jump in (-TAU, TAU):
                    with self.subTest(mid=mid, sign=sign, jump=jump):
                        mapper = bound(contracts={i: axis(i, sign=sign) for i in IDS})
                        capture = snapshot(ns=1_020_000_000, digest="2")
                        capture["raw_rad_by_id"][str(mid)] += jump
                        with self.assertRaisesRegex(AngleEvidenceError,
                                                    f"ID{mid}: within-epoch discontinuity"):
                            mapper.observe(capture)
                        with self.assertRaisesRegex(AngleEvidenceError, "invalidated"):
                            mapper.raw_target_candidates({str(i): 0. for i in IDS},
                                                         max_delta_rad=.03)

    def test_current_branch_roundtrip_preserves_raw_turn(self):
        initial = snapshot(raw=TAU + .1)
        mapper = bound(initial)
        report = mapper.report()
        targets = {key: q + .02 for key, q in report["model_rad_by_id"].items()}
        inverse = mapper.raw_target_candidates(targets, max_delta_rad=.03)
        for key, raw in inverse["raw_target_candidates_rad_by_id"].items():
            self.assertAlmostEqual(raw, initial["raw_rad_by_id"][key] + .02)
        self.assertFalse(inverse["output_allowed"])
        self.assertFalse(report["approved_for_runtime"])
        self.assertEqual(initial["raw_rad_by_id"]["1"], TAU + .1)

    def test_new_power_epoch_rebinds_without_repeating_no_turn_statement(self):
        old = bound(snapshot(raw=.1)).report()
        new = bound(snapshot(epoch="motor-epoch-B", raw=TAU + .1,
                             digest="2")).report()
        self.assertEqual(old["turns_by_id"]["1"], 0)
        self.assertEqual(new["turns_by_id"]["1"], 1)
        self.assertAlmostEqual(old["model_rad_by_id"]["1"], new["model_rad_by_id"]["1"])

    def test_same_epoch_full_turn_jump_is_not_modulo_corrected(self):
        mapper = bound()
        jump = snapshot(ns=1_020_000_000, raw=TAU, digest="2")
        with self.assertRaisesRegex(AngleEvidenceError, "within-epoch discontinuity"):
            mapper.observe(jump)
        with self.assertRaisesRegex(AngleEvidenceError, "invalidated"):
            mapper.report()

    def test_same_epoch_crossing_360_uses_continuous_raw(self):
        initial = snapshot(raw=TAU - .005)
        mapper = bound(initial)
        result = mapper.observe(snapshot(ns=1_020_000_000, raw=TAU + .005, digest="2"))
        self.assertAlmostEqual(result["model_rad_by_id"]["1"], .005)
        self.assertEqual(result["turns_by_id"]["1"], 1)

    def test_power_epoch_uid_boot_or_source_change_invalidates_map(self):
        for field, value in (("motor_power_epoch", "motor-epoch-B"),
                             ("boot_id", "boot-B"),
                             ("epoch_evidence_sha256", "2" * 64),
                             ("uid_capture_sha256", "2" * 64)):
            current = snapshot(ns=1_020_000_000, digest="2")
            current[field] = value
            with self.assertRaises(AngleEvidenceError):
                bound().observe(current)
        current = snapshot(ns=1_020_000_000, digest="2")
        current["uids_by_id"]["7"] = "9" * 16
        with self.assertRaisesRegex(AngleEvidenceError, "binding changed"):
            bound().observe(current)

    def test_unknown_epoch_not_inferred_from_same_jetson_boot(self):
        for value in ("UNKNOWN", "", "NOT_INFERRED_FROM_JETSON_BOOT"):
            with self.assertRaisesRegex(AngleEvidenceError, "Explicit motor-power"):
                bound(snapshot(epoch=value))

    def test_stale_duplicate_nonmonotonic_rejected(self):
        for ns in (1_000_000_000, 999_999_999, 1_200_000_000):
            with self.assertRaisesRegex(AngleEvidenceError, "stale"):
                bound().observe(snapshot(ns=ns, digest="2"))
        with self.assertRaisesRegex(AngleEvidenceError, "Repeated angle capture"):
            bound().observe(snapshot(ns=1_020_000_000))

    def test_uid_replacement_and_twelve_axis_completeness(self):
        initial = snapshot()
        initial["uids_by_id"]["11"] = "9" * 16
        with self.assertRaisesRegex(AngleEvidenceError, "ID11: UID changed"):
            bound(initial)
        del initial["raw_rad_by_id"]["10"]
        with self.assertRaisesRegex(AngleEvidenceError, "Exactly twelve"):
            bound(initial)

    def test_target_bounds_delta_and_sign(self):
        contracts = {mid: axis(mid, sign=-1) for mid in IDS}
        mapper = bound(snapshot(raw=TAU + .1), contracts)
        target = {str(mid): -.08 for mid in IDS}
        out = mapper.raw_target_candidates(target, max_delta_rad=.03)
        self.assertAlmostEqual(out["raw_target_candidates_rad_by_id"]["1"], TAU + .08)
        target["1"] = .7
        with self.assertRaisesRegex(AngleEvidenceError, "outside mechanical"):
            mapper.raw_target_candidates(target, max_delta_rad=1.)
        target["1"] = .1
        with self.assertRaisesRegex(AngleEvidenceError, "delta exceeds"):
            mapper.raw_target_candidates(target, max_delta_rad=.03)


def observation(deg, *, sign=1, sequence=0):
    return {"uid": "1" * 16, "boot_id": "boot", "motor_power_epoch": "epoch",
            "source_sha256": f"{sequence:064x}", "relative_output_shaft_observed": True,
            "physical_angle_method": "angle_gauge", "raw_rad": math.radians(deg) / sign + .2,
            "model_rad": math.radians(deg), "uncertainty_rad": math.radians(.5)}


class PhysicalReferenceTests(unittest.TestCase):
    def test_touching_fit_intervals_do_not_claim_perfect_physical_accuracy(self):
        rows = [observation(0), observation(15, sequence=1)]
        # Opposite bounded errors narrow the offset intersection to one point.
        rows[-1]["raw_rad"] -= math.radians(1)
        fit = fit_reference_observations(rows)
        self.assertAlmostEqual(fit["offset_fit_uncertainty_rad"], 0.)
        self.assertAlmostEqual(fit["uncertainty_rad"], math.radians(.5))
        self.assertAlmostEqual(fit["max_abs_residual_rad"], math.radians(.5))
        self.assertAlmostEqual(fit["physical_span_rad"], math.radians(15))
        self.assertAlmostEqual(fit["raw_span_rad"], math.radians(14))
        self.assertAlmostEqual(fit["signed_displacement_scale_candidate"], 15 / 14)
        self.assertTrue(fit["physical_accuracy_review_required"])
        self.assertFalse(fit["dynamic_type2_scale_verified"])

    def test_finite_extreme_sources_cannot_produce_infinite_offset_candidate(self):
        rows=[observation(0),observation(15,sequence=1)]
        for row in rows:row['raw_rad']=1e308
        rows[0]['model_rad']=1e308
        rows[1]['model_rad']=math.nextafter(1e308,math.inf)
        # Previously this returned sign=-1, offset=Inf, uncertainty=NaN.
        with self.assertRaisesRegex(AngleEvidenceError,'Nonfinite'):
            fit_reference_observations(rows)

    def test_known_reference_move_return_fits_both_signs(self):
        for sign in (-1, 1):
            rows = [observation(deg, sign=sign, sequence=n)
                    for n, deg in enumerate((0, 15, 0))]
            result = fit_reference_observations(rows)
            self.assertEqual(result["sign_candidate"], sign)
            self.assertAlmostEqual(result["offset_candidate_rad"], -sign * .2)
            self.assertFalse(result["approved_for_runtime"])

    def test_unknown_accuracy_whole_leg_and_wrong_direction_are_not_accepted(self):
        rows = [observation(0), observation(15, sequence=1)]
        for change in ({"relative_output_shaft_observed": False},
                       {"uncertainty_rad": 0}, {"motor_power_epoch": "other"},
                       {"physical_angle_method": "unknown"}):
            changed = copy.deepcopy(rows)
            changed[-1].update(change)
            with self.assertRaises(AngleEvidenceError):
                fit_reference_observations(changed)
        changed = copy.deepcopy(rows)
        changed[-1]["raw_rad"] = changed[0]["raw_rad"] + .005
        with self.assertRaisesRegex(AngleEvidenceError, "0 candidates"):
            fit_reference_observations(changed)

    def test_wrap_in_observations_requires_new_epoch_instead_of_auto_fix(self):
        rows = [observation(0), observation(15, sequence=1)]
        rows[-1]["raw_rad"] += TAU
        with self.assertRaisesRegex(AngleEvidenceError, "do not wrap"):
            fit_reference_observations(rows)

    def test_too_small_move_or_ambiguous_uncertainty_fails(self):
        with self.assertRaisesRegex(AngleEvidenceError, "five degrees"):
            fit_reference_observations([observation(0), observation(2, sequence=1)])
        rows = [observation(0), observation(15, sequence=1)]
        for row in rows:
            row["uncertainty_rad"] = math.radians(20)
        with self.assertRaisesRegex(AngleEvidenceError, "2 candidates"):
            fit_reference_observations(rows)


class BatchAuditTests(unittest.TestCase):
    def test_replaced_axis_only_loses_identity_eligibility(self):
        contracts = {mid: axis(mid) for mid in IDS}
        sample = snapshot()
        sample["uids_by_id"]["11"] = "9" * 16
        report = audit_twelve_axes(contracts, sample["raw_rad_by_id"], sample["uids_by_id"])
        self.assertEqual(report["uid_changed_ids"], [11])
        self.assertTrue(report["rows_by_id"]["1"]["evidence_ready_for_epoch_binding"])
        self.assertFalse(report["rows_by_id"]["11"]["evidence_ready_for_epoch_binding"])

    def test_modulo_candidate_does_not_mark_physical_evidence_complete(self):
        contracts = {mid: axis(mid, zero_reviewed=False, direction_reviewed=False,
                               physical_limits_reviewed=False) for mid in IDS}
        sample = snapshot(raw=TAU + .1)
        report = audit_twelve_axes(contracts, sample["raw_rad_by_id"], sample["uids_by_id"])
        self.assertEqual(report["needs_zero_review_ids"], list(IDS))
        self.assertEqual(report["rows_by_id"]["1"]["periodic_branch_candidates"][0]["turns"], 1)
        self.assertEqual(report["status"], "INCOMPLETE_PHYSICAL_EVIDENCE")
        self.assertFalse(report["output_allowed"])


if __name__ == "__main__":
    unittest.main()
