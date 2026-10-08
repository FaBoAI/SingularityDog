"""Fixed JSON copy plans: exact records and ownership, no sensors or model files."""
import copy
import dis
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_observer as observer
from singularitydog_hw import imu_accel_input_hypothesis as hypotheses
import test_policy_observer as fixtures
from test_policy_observer_accel_provenance_cache import (
    REFERENCE, START, frozen_hypothesis, tree_bits, UncachedCorrection)


def mutable_nodes(value):
    result = set()
    if type(value) in (dict, list, tuple):
        if type(value) in (dict, list):
            result.add(id(value))
        for child in value.values() if type(value) is dict else value:
            result.update(mutable_nodes(child))
    return result


class LiteralFactoryTests(unittest.TestCase):
    def test_literals_match_types_bits_order_and_canonical_sha(self):
        shared = [{"mutable": [1, -0.0, "方向"]}]
        values = [None, True, False, 0, -(2**1023), 1.5, -0.0, "\udfff",
            [], {}, (), {"second": shared, "first": shared,
                "tuple": ("immutable", shared), "float_bits": [5e-324, 1.7976931348623157e308],
                "quoted": "\"__import__('os').system('this is literal data')\"\n"}]
        for value in values:
            with self.subTest(kind=type(value).__name__):
                factory = observer._frozen_json_literal_factory(value)
                self.assertIsNotNone(factory)
                first, second = factory(), factory()
                expected = observer.snapshot_event(value)
                self.assertEqual(tree_bits(first), tree_bits(expected))
                self.assertEqual(observer._digest(first), observer._digest(expected))
                self.assertFalse(mutable_nodes(first) & mutable_nodes(second))
                self.assertFalse(mutable_nodes(first) & mutable_nodes(value))
                self.assertEqual(factory.__code__.co_names, ())
                self.assertEqual(factory.__code__.co_freevars, ())
                self.assertEqual(factory.__globals__["__builtins__"], {})
                self.assertEqual(set(factory.__globals__), {"__builtins__"})
                self.assertFalse(any(i.opname in ("CALL", "LOAD_GLOBAL", "LOAD_ATTR", "IMPORT_NAME")
                                     for i in dis.get_instructions(factory)))

    def test_source_and_previous_return_mutation_cannot_change_plan(self):
        shared = [{"values": [1, 2]}]
        source = {"left": shared, "right": shared, "tuple": (shared,)}
        factory = observer._frozen_json_literal_factory(source)
        expected = tree_bits(observer.snapshot_event(source))
        source["left"][0]["values"].append(100)
        first = factory()
        first["left"][0]["values"].clear()
        self.assertEqual(first["right"][0]["values"], [1, 2])
        self.assertEqual(first["tuple"][0][0]["values"], [1, 2])
        self.assertEqual(tree_bits(factory()), expected)

    def test_nonfinite_cycles_subclasses_and_nonstring_keys_fall_back_without_hooks(self):
        calls = []
        class Foreign:
            def __repr__(self):
                calls.append("repr")
                raise AssertionError("Unexpected hook")
            def __deepcopy__(self, memo):
                calls.append("deepcopy")
                raise AssertionError("Unexpected hook")
        class DictSubclass(dict):
            def items(self):
                calls.append("items")
                raise AssertionError("Unexpected hook")
        cyclic = []; cyclic.append(cyclic)
        for value in (math.nan, math.inf, -math.inf, cyclic, {1: "numeric key"},
                      Foreign(), DictSubclass(a=1), [Foreign()]):
            self.assertIsNone(observer._frozen_json_literal_factory(value))
        self.assertEqual(calls, [])

    def test_explicit_complexity_bounds_keep_original_bounded_copy_accepted(self):
        deep = [1]
        for _ in range(17):
            deep = [deep]
        for value in ("x"*4097, 2**1024, list(range(4096)), deep, ["x"*4000]*8):
            with self.subTest(kind=type(value).__name__):
                observer.snapshot_event(value)  # Existing path still accepts it.
                self.assertIsNone(observer._frozen_json_literal_factory(value))

    def test_compile_receives_only_generated_literal_ast(self):
        import ast
        trees = []
        original = compile
        def observe(tree, *args, **kwargs):
            trees.append(tree)
            return original(tree, *args, **kwargs)
        supplied = {"__class__": "__import__('os').system('false')", "\"\nexec('false')": [1]}
        with patch("builtins.compile", side_effect=observe):
            factory = observer._frozen_json_literal_factory(supplied)
        self.assertEqual(factory(), supplied)
        self.assertEqual(len(trees), 1)
        allowed = (ast.Module, ast.FunctionDef, ast.arguments, ast.Return,
                   ast.Dict, ast.List, ast.Tuple, ast.Constant, ast.Load)
        self.assertTrue(all(isinstance(node, allowed) for node in ast.walk(trees[0])))
        self.assertFalse(any(isinstance(node, (ast.Name, ast.Call, ast.Attribute))
                             for node in ast.walk(trees[0])))

    def test_factory_is_prepared_only_before_ticks(self):
        run = fixtures.make(gyro_bias_candidate=fixtures.bias_candidate())
        self.assertTrue(all(row is not None and row[1] is not None
                            for row in run._static_provenance_factories))
        run.reset_run(START, warmup_completed=True)
        with patch("builtins.compile", side_effect=AssertionError("Compile inside tick")):
            run.consume(fixtures.snapshot())

    def test_complete_records_state_and_profile_clocks_match_original_marshal(self):
        shared = [{"captured": [1.0, -0.0, False, "方向"]}]
        cal = fixtures.calibration()
        cal["metadata"] = {"a": shared, "b": shared, "tuple": (shared,)}
        clocks = [iter(range(100, 1000, 10)) for _ in range(2)]
        runs = [fixtures.make(calibration=cal, gyro_bias_candidate=fixtures.bias_candidate(),
                    profile_consume=True, monotonic_ns=lambda clock=clock: next(clock))
                for clock in clocks]
        runs[0]._static_provenance_factories = (None,)*3
        for run in runs:
            run.reset_run(START, warmup_completed=True)
        for index in range(2):
            source = fixtures.snapshot(START+index*observer.DT_NS)
            source["source_flags"] = {"sequence": index, "own": [{"truth": False}]}
            source["imu"]["gyro_rad_s"][0] += index*.01
            expected, actual = [run.consume(source) for run in runs]
            self.assertEqual(tree_bits(actual), tree_bits(expected))
            self.assertEqual(tree_bits(runs[1]._policy.calls), tree_bits(runs[0]._policy.calls))
            for field in ("_next_tick_ns", "_last_sources", "ticks_completed", "status"):
                self.assertEqual(getattr(runs[1], field), getattr(runs[0], field))
            self.assertFalse(mutable_nodes(actual["provenance"]) & mutable_nodes(expected["provenance"]))
            actual["provenance"]["calibration_source_flags"]["metadata"]["a"].clear()
        self.assertEqual(tree_bits(runs[1].finish()), tree_bits(runs[0].finish()))

    def test_large_bounded_static_metadata_uses_marshal_with_same_records(self):
        cal = fixtures.calibration()
        cal["legacy_large"] = "x"*4097
        run = fixtures.make(calibration=cal)
        self.assertIsNotNone(run._static_provenance_blobs[0])
        self.assertIsNone(run._static_provenance_factories[0][1])
        run.reset_run(START, warmup_completed=True)
        record = run.consume(fixtures.snapshot())
        self.assertEqual(record["provenance"]["calibration_source_flags"]["legacy_large"], "x"*4097)

    def test_replaced_blob_and_copier_do_not_use_stale_factory(self):
        run = fixtures.make()
        factory = run._static_provenance_factories[0][1]
        with patch.object(observer, "marshal") as marshal:
            marshal.loads.return_value = {"new": False}
            original = run._static_provenance_blobs
            replacement = bytes(bytearray(original[0]))
            self.assertIsNot(replacement, original[0])
            run._static_provenance_blobs = (replacement, *original[1:])
            self.assertEqual(run._copy_static_provenance(0, run._calibration_source_flags), {"new": False})
            marshal.loads.assert_called_once_with(replacement)
        run._static_provenance_copiers = (copy.deepcopy,)*3
        with patch.object(observer, "marshal") as marshal:
            self.assertEqual(run._copy_static_provenance(0, run._calibration_source_flags),
                             run._calibration_source_flags)
            marshal.loads.assert_not_called()
        self.assertEqual(factory(), run._calibration_source_flags)

    def test_factory_failure_invalidates_same_provenance_stage_without_tick_commit(self):
        clock = iter(range(100, 1000, 10))
        run = fixtures.make(profile_consume=True, monotonic_ns=lambda: next(clock))
        run.reset_run(START, warmup_completed=True)
        error = MemoryError("Literal allocation failed")
        def fail():
            raise error
        blob = run._static_provenance_blobs[0]
        run._static_provenance_factories = ((blob, fail), *run._static_provenance_factories[1:])
        with self.assertRaises(MemoryError) as caught:
            run.consume(fixtures.snapshot())
        self.assertIs(caught.exception, error)
        self.assertEqual(run.ticks_completed, 0)
        self.assertEqual(run._last_sources, {})
        self.assertEqual(run.status, "INCOMPLETE")
        self.assertEqual(run.summary()["last_consume_profile"]["failed_stage"], "provenance_serialization")

    def test_accel_frozen_identity_fence_and_dynamic_method_calls_remain_exact(self):
        candidate = frozen_hypothesis()
        with patch.object(hypotheses, "load_accel_input_hypothesis", return_value=candidate):
            run = fixtures.make(accel_input_hypothesis=REFERENCE)
        self.assertIsNotNone(run._accel_provenance_cache[4])
        run.reset_run(START, warmup_completed=True)
        calls = []
        old = hypotheses.AccelInputHypothesis.provenance
        def changed(correction):
            calls.append(correction)
            value = old(correction)
            value["changed_method"] = len(calls)
            return value
        with patch.object(hypotheses.AccelInputHypothesis, "provenance", changed):
            for index in range(2):
                record = run.consume(fixtures.snapshot(START+index*observer.DT_NS))
                self.assertEqual(record["provenance"]["accel_input_hypothesis"]["changed_method"], index+1)
        self.assertEqual(calls, [candidate, candidate])

    def test_uncached_custom_correction_keeps_exact_dynamic_provenance(self):
        correction = UncachedCorrection(frozen_hypothesis())
        with patch.object(hypotheses, "load_accel_input_hypothesis", return_value=correction):
            run = fixtures.make(accel_input_hypothesis=REFERENCE)
        self.assertIsNone(run._accel_provenance_cache)
        run.reset_run(START, warmup_completed=True)
        for index in range(2):
            run.consume(fixtures.snapshot(START+index*observer.DT_NS))
        self.assertEqual(correction.provenance_calls, 4)  # reset/arm summaries plus both ticks


if __name__ == "__main__":
    unittest.main()
