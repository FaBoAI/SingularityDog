"""Every-axis validation/failure contracts; optional pinned old-source comparison.

The default suite asserts concrete source freshness and rejection semantics.
OBSERVER_VALIDATION_REFERENCE_SOURCE and OBSERVER_VALIDATION_REFERENCE_SHA256
add comparison with a separately preserved source file, without keeping private
capture paths or historical production code in the repository.
"""
import copy
import hashlib
import math
import os
from pathlib import Path
import types
import unittest

from singularitydog_hw import policy_observer as observer
import test_policy_observer as fixtures
from test_policy_observer_accel_provenance_cache import tree_bits


START = 1_000_000_000
LIMIT = 100_000_000


def update_summaries(source):
    tick = source["tick_ns"]
    for row in source["motors"]:
        row["age_upper_bound_ns"] = tick-row["request_ns"]
    imu = source["imu"]
    imu["age_upper_bound_ns"] = tick-imu["read_started_ns"]
    first = min(imu["read_started_ns"], *(r["request_ns"] for r in source["motors"]))
    last = max(imu["read_finished_ns"], *(r["received_ns"] for r in source["motors"]))
    earliest = min(imu["read_finished_ns"], *(r["received_ns"] for r in source["motors"]))
    source.update(oldest_observation_age_ns=tick-first,
                  acquisition_spread_ns=last-first, receive_spread_ns=last-earliest)


def snapshot(tick=START):
    source = fixtures.snapshot(tick)
    source.update(max_age_ns=LIMIT, max_spread_ns=LIMIT)
    return source


def row_for(source, mid, parameter="position"):
    return next(r for r in source["motors"]
                if r["motor_id"] == mid and r["parameter"] == parameter)


def outcome(module, source, *, previous=None):
    clock = iter(range(100, 10_000, 10))
    run = module.StatefulPolicyObserver(fixtures.Policy(), fixtures.calibration(),
        imu_mount_candidate=fixtures.mount(), command=[0., 0., 0.], h_hypothesis=0,
        max_ticks=2, max_age_ns=LIMIT, max_spread_ns=LIMIT,
        torch_module=fixtures.FakeTorch, profile_consume=True, monotonic_ns=lambda: next(clock))
    run.reset_run(START, warmup_completed=True)
    if previous is not None:
        run.consume(previous)
    before = dict(next_tick=run._next_tick_ns, sources=copy.deepcopy(run._last_sources),
                  branch=copy.deepcopy(run._last_branch_raw), completed=run.ticks_completed,
                  policy_calls=copy.deepcopy(run._policy.calls), state=copy.deepcopy(run._policy.previous))
    original = copy.deepcopy(source)
    try:
        record = run.consume(source)
        error = None
    except Exception as exc:
        record, error = None, (type(exc).__name__, str(exc))
    assert tree_bits(source) == tree_bits(original)
    return dict(record=record, error=error, summary=run.summary(), next_tick=run._next_tick_ns,
                sources=run._last_sources, branch=run._last_branch_raw, before=before,
                policy_calls=run._policy.calls, state=run._policy.previous)


class SourceValidationParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = os.environ.get("OBSERVER_VALIDATION_REFERENCE_SOURCE")
        pin = os.environ.get("OBSERVER_VALIDATION_REFERENCE_SHA256")
        cls.reference = None
        if source is not None or pin is not None:
            if not source or not pin:
                raise AssertionError("Both reference source and SHA256 are required")
            raw = Path(source).read_bytes()
            if hashlib.sha256(raw).hexdigest() != pin:
                raise AssertionError("Pinned observer reference source changed")
            module = types.ModuleType("singularitydog_hw._validation_reference")
            module.__package__ = "singularitydog_hw"
            module.__file__ = str(Path(source))
            exec(compile(raw, str(Path(source)), "exec"), module.__dict__)
            cls.reference = module

    def assert_contract(self, source, message=None, stage="source_validation", previous=None):
        current = outcome(observer, source, previous=previous)
        if self.reference is not None:
            old = outcome(self.reference, source, previous=previous)
            self.assertEqual(tree_bits(current), tree_bits(old))
        if message is None:
            self.assertIsNone(current["error"])
            self.assertIsNotNone(current["record"])
            self.assertEqual(current["summary"]["ticks_completed"], 1 if previous is None else 2)
        else:
            self.assertIsNotNone(current["error"])
            self.assertIn(message, current["error"][1])
            self.assertEqual(current["summary"]["last_consume_profile"]["failed_stage"], stage)
            self.assertEqual(current["summary"]["status"], "INCOMPLETE")
            self.assertEqual(current["summary"]["ticks_completed"], current["before"]["completed"])
            self.assertEqual(current["next_tick"], current["before"]["next_tick"])
            self.assertEqual(current["sources"], current["before"]["sources"])
            self.assertEqual(current["branch"], current["before"]["branch"])
            self.assertEqual(current["state"], current["before"]["state"])
            self.assertEqual(current["policy_calls"], current["before"]["policy_calls"])
        return current

    def test_all_axes_reject_bad_values_and_unreviewed_full_turns_before_policy_call(self):
        for mid in range(1, 13):
            for parameter in ("position", "velocity"):
                for value in (None, True, "1.0"):
                    with self.subTest(mid=mid, parameter=parameter, value=value):
                        source = snapshot(); row_for(source, mid, parameter)["value"] = value
                        self.assert_contract(source, "Invalid motor SI value")
                for value in (math.nan, math.inf, -math.inf):
                    with self.subTest(mid=mid, parameter=parameter, nonfinite=value):
                        source = snapshot(); row_for(source, mid, parameter)["value"] = value
                        # Bounded copying rejects these before source conversion.
                        self.assert_contract(source, "Nonfinite", "snapshot_copy")
            for direction in (-1, 1):
                source = snapshot(); row_for(source, mid)["value"] += direction*2*math.pi
                self.assert_contract(source, "Calibrated position outside registered joint range",
                                     "input_conversion")

    def test_every_axis_missing_fields_duplicate_cross_axis_and_timestamp_failures(self):
        missing = {"motor_id": "Invalid motor key", "parameter": "Invalid motor key",
                   "value": "Invalid motor SI value", "unit": "Invalid motor SI value",
                   "request_ns": "Invalid motor request", "received_ns": "Invalid motor receive",
                   "age_upper_bound_ns": "Invalid motor age"}
        for mid in range(1, 13):
            for parameter in ("position", "velocity"):
                for field, message in missing.items():
                    with self.subTest(mid=mid, parameter=parameter, missing=field):
                        source = snapshot(); del row_for(source, mid, parameter)[field]
                        self.assert_contract(source, message)
                for field, invalid, message in (
                    ("motor_id", True, "Invalid motor key"),
                    ("request_ns", True, "Invalid motor request"),
                    ("request_ns", -1, "Invalid motor request"),
                    ("request_ns", 2**63, "Invalid motor request"),
                    ("received_ns", 2**63, "Invalid motor receive"),
                    ("received_ns", START+1, "Noncausal motor observation"),
                    ("age_upper_bound_ns", True, "Invalid motor age")):
                    with self.subTest(mid=mid, parameter=parameter, field=field, value=invalid):
                        source = snapshot(); row_for(source, mid, parameter)[field] = invalid
                        self.assert_contract(source, message)
                # Substituting another valid ID cannot bypass twelve-axis uniqueness.
                source = snapshot(); row_for(source, mid, parameter)["motor_id"] = 7 if mid <= 6 else 1
                self.assert_contract(source, "Duplicate motor key")

    def test_stale_boundary_and_order_independent_extrema_for_every_axis(self):
        for mid in range(1, 13):
            for parameter in ("position", "velocity"):
                for delta in (0, 1):
                    with self.subTest(mid=mid, parameter=parameter, over_ns=delta):
                        source = snapshot(); row_for(source, mid, parameter)["request_ns"] = START-LIMIT-delta
                        update_summaries(source)
                        self.assert_contract(source, None if delta == 0 else "Stale observation")
                source = snapshot()
                selected = row_for(source, mid, parameter)
                selected["request_ns"] = START-8_000_000
                selected["received_ns"] = START-4_000_000
                update_summaries(source)
                # Arrival order changes neither independent timing checks nor model inputs.
                ordinary = self.assert_contract(source)
                reversed_source = copy.deepcopy(source); reversed_source["motors"].reverse()
                reversed_result = self.assert_contract(reversed_source)
                self.assertEqual(tree_bits(ordinary["record"]["inputs"]),
                                 tree_bits(reversed_result["record"]["inputs"]))
                for key in ("oldest_observation_age_ns", "acquisition_spread_ns", "receive_spread_ns"):
                    self.assertEqual(ordinary["record"]["provenance"][key],
                                     reversed_result["record"]["provenance"][key])

    def test_held_and_partly_repeated_timestamps_keep_exact_failure_and_prior_state(self):
        previous = snapshot()
        for mid in range(1, 13):
            for parameter in ("position", "velocity"):
                old = row_for(previous, mid, parameter)
                for change in ("hold", "changed_value", "start_repeated", "end_repeated"):
                    with self.subTest(mid=mid, parameter=parameter, change=change):
                        source = snapshot(START+observer.DT_NS); current = row_for(source, mid, parameter)
                        if change in ("hold", "changed_value"):
                            current.update(request_ns=old["request_ns"], received_ns=old["received_ns"])
                            if change == "changed_value":
                                current["value"] += .001
                        elif change == "start_repeated":
                            current["request_ns"] = old["request_ns"]
                        else:
                            # Keep this individually causal, but fail progress in the held-source guard.
                            current.update(request_ns=old["request_ns"]+1, received_ns=old["received_ns"])
                        update_summaries(source)
                        message = (None if change == "hold" else "Held source timestamps changed values"
                                   if change == "changed_value" else "timestamps moved backward or partly repeated")
                        self.assert_contract(source, message, previous=previous)

    def test_foreign_row_getters_never_enter_owned_source_validation(self):
        calls = []
        class ForeignRow(dict):
            def get(self, *args):
                calls.append("get")
                raise AssertionError("Getter on caller row")
            def __getitem__(self, key):
                calls.append("getitem")
                raise AssertionError("Getter on caller row")
        source = snapshot(); source["motors"][0] = ForeignRow(source["motors"][0])
        run = fixtures.make(profile_consume=True)
        run.reset_run(START, warmup_completed=True)
        with self.assertRaises(observer.ObserverError):
            run.consume(source)
        self.assertEqual(calls, [])
        self.assertEqual(run.summary()["last_consume_profile"]["failed_stage"], "snapshot_copy")


if __name__ == "__main__":
    unittest.main()
