"""Exact owned snapshot parity with fake policies; no devices or model weights."""
import copy
import math
import struct
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_observer as observer
from test_policy_observer import Policy, make, snapshot


def native_shape(tick=1_000_000_000):
    value = snapshot(tick)
    value.update(voltage_by_bus={}, source_flags={"pending_voltage": True})
    return value


def mutable_ids(value):
    if type(value) is dict:
        return [id(value)] + [i for v in value.values() for i in mutable_ids(v)]
    if type(value) in (list, tuple):
        return ([id(value)] if type(value) is list else []) + [
            i for v in value for i in mutable_ids(v)]
    return []


class FlatSnapshotTests(unittest.TestCase):
    def test_exact_shape_optional_fields_and_signed_zero_hash_and_ownership(self):
        for voltage in (False, True):
            for flags in (False, True):
                with self.subTest(voltage=voltage, flags=flags):
                    source = snapshot()
                    if voltage: source["voltage_by_bus"] = {}
                    if flags: source["source_flags"] = {"pending": True}
                    source["motors"][0]["value"] = -0.0
                    source["imu"]["gyro_rad_s"][0] = -0.0
                    original = copy.deepcopy(source)
                    actual = observer._flat_feedback_snapshot(source)
                    expected = observer.snapshot_event(source)
                    self.assertIsNotNone(actual)
                    self.assertEqual(actual, expected)
                    self.assertEqual(list(actual), list(expected))
                    self.assertEqual(observer._digest(actual), observer._digest(expected))
                    self.assertEqual(struct.pack('!d', actual["motors"][0]["value"]),
                                     struct.pack('!d', -0.0))
                    ids = mutable_ids(actual)
                    self.assertEqual(len(ids), len(set(ids)))
                    self.assertTrue(set(ids).isdisjoint(mutable_ids(source)))
                    actual["motors"][0]["value"] = 99
                    actual["imu"]["gyro_rad_s"].clear()
                    self.assertEqual(source, original)

    def test_repeated_containers_are_independent_like_original_copier(self):
        source = native_shape()
        source["motors"][1] = source["motors"][0]
        source["imu"]["gyro_rad_s"] = source["imu"]["accel_m_s2"]
        actual = observer._snapshot_copy(source)
        expected = observer.snapshot_event(source)
        self.assertEqual(actual, expected)
        self.assertIsNot(actual["motors"][0], actual["motors"][1])
        self.assertIsNot(actual["imu"]["gyro_rad_s"], actual["imu"]["accel_m_s2"])

    def test_extreme_fast_shape_remains_within_original_bounds(self):
        source = native_shape()
        largest = (1 << 1024) - 1
        for key in observer._FEEDBACK_SCALAR_KEYS: source[key] = largest
        for row in source["motors"]:
            for key in row: row[key] = largest
        for key in ("frame", "read_started_ns", "read_finished_ns", "age_upper_bound_ns"):
            source["imu"][key] = largest
        source["imu"]["accel_m_s2"] = [largest] * 3
        source["imu"]["gyro_rad_s"] = [largest] * 3
        source["source_flags"] = {str(i).rjust(128, 'x'): True for i in range(64)}
        self.assertIsNotNone(observer._flat_feedback_snapshot(source))
        self.assertEqual(observer._snapshot_copy(source), observer.snapshot_event(source))

    def test_unusual_valid_trees_fall_back_without_narrowing_acceptance(self):
        changes = (
            lambda s: s.update(extra={"owned": [1, 2]}),
            lambda s: s["source_flags"].update(nested={"tuple": (1, [2])}),
            lambda s: s["source_flags"].update({str(i): False for i in range(65)}),
            lambda s: s["source_flags"].update({"x" * 129: True}),
            lambda s: s["motors"][0].update(note="accepted metadata"),
            lambda s: s["motors"][0].update(unit="x" * 65),
            lambda s: s["motors"][0].update(request_ns=1 << 1024),
            lambda s: s["imu"].update(gyro_rad_s=(0., 0., 0.)),
            lambda s: s["voltage_by_bus"].update(front={"value": 36.}),
            lambda s: s["blocked_reasons"].append("blocked"),
        )
        for index, change in enumerate(changes):
            with self.subTest(index=index):
                source = native_shape(); change(source)
                self.assertIsNone(observer._flat_feedback_snapshot(source))
                actual = observer._snapshot_copy(source)
                self.assertEqual(actual, observer.snapshot_event(source))
                self.assertEqual(observer._digest(actual), observer._digest(source))
                self.assertTrue(set(mutable_ids(actual)).isdisjoint(mutable_ids(source)))

    def test_rejected_trees_keep_original_error_and_do_not_invoke_copy_hooks(self):
        class Hooks:
            def __deepcopy__(self, memo): raise AssertionError("copy hook")
        class DictSubclass(dict): pass
        changes = (
            lambda s: s["motors"][0].update(value=math.nan),
            lambda s: s["imu"]["gyro_rad_s"].__setitem__(0, math.inf),
            lambda s: s["source_flags"].update(unsupported=Hooks()),
            lambda s: s["source_flags"].update({1: True}),
            lambda s: s["source_flags"].update(cycle=s),
            lambda s: s.update(extra="x" * 90_000),
            lambda s: s.update(imu=DictSubclass(s["imu"])),
        )
        for index, change in enumerate(changes):
            with self.subTest(index=index):
                source = native_shape(); change(source)
                with patch.object(observer, '_flat_feedback_snapshot', return_value=None):
                    with self.assertRaises(observer.ObserverError) as old: observer._snapshot_copy(source)
                with self.assertRaises(observer.ObserverError) as new: observer._snapshot_copy(source)
                self.assertEqual(str(old.exception), str(new.exception))

    def test_schema_key_subclasses_reject_before_equality_hooks(self):
        class Key(str):
            __hash__ = str.__hash__
            def __eq__(self, other): raise AssertionError("key equality hook")
        for mapping, key in ((lambda s: s, 'status'),
                             (lambda s: s['motors'][0], 'motor_id'),
                             (lambda s: s['motors'][23], 'motor_id'),
                             (lambda s: s['imu'], 'frame')):
            source = native_shape()
            target = mapping(source)
            value = target.pop(key)
            target[Key(key)] = value
            self.assertIsNone(observer._flat_feedback_snapshot(source))
            with self.assertRaisesRegex(observer.ObserverError, 'built-in strings'):
                observer._snapshot_copy(source)

    def test_oversized_containers_keep_original_bound_rejection(self):
        cases = (
            lambda s: s.update({str(i): 0 for i in range(10001)}),
            lambda s: s.update(motors=[{}] * 20001),
            lambda s: s['imu'].update({str(i): 0 for i in range(10001)}),
            lambda s: s.update(source_flags={str(i): True for i in range(10001)}),
            lambda s: s['blocked_reasons'].extend([False] * 20001),
            lambda s: s['voltage_by_bus'].update({str(i): 0 for i in range(10001)}),
            lambda s: s['imu'].update(gyro_rad_s=[0.] * 20001),
            lambda s: s['motors'][0].update({str(i): 0 for i in range(10001)}),
        )
        for change in cases:
            source = native_shape(); change(source)
            with self.assertRaises(observer.ObserverError) as old:
                with patch.object(observer, '_flat_feedback_snapshot', return_value=None):
                    observer._snapshot_copy(source)
            with self.assertRaises(observer.ObserverError) as new:
                observer._snapshot_copy(source)
            self.assertEqual(str(old.exception), str(new.exception))

    def test_different_builtin_key_objects_and_order_preserve_exact_copy(self):
        source = native_shape()
        for index, row in enumerate(source['motors']):
            source['motors'][index] = {
                (key + 'x')[:-1]: value for key, value in reversed(tuple(row.items()))
            } if index % 2 else {(key + 'x')[:-1]: value for key, value in row.items()}
        self.assertIsNotNone(observer._flat_feedback_snapshot(source))
        self.assertEqual(observer._snapshot_copy(source), observer.snapshot_event(source))
        self.assertEqual(observer._digest(observer._snapshot_copy(source)), observer._digest(source))

    def test_source_mutation_after_owned_leaf_copy_cannot_inject_new_descendant(self):
        source = native_shape()
        original = observer._flat_snapshot_scalars
        def mutate(values):
            result = original(values)
            if type(values) is type({}.values()):
                source['motors'][0]['value'] = ["later caller mutation"]
            elif type(values) is list and values == source['imu']['gyro_rad_s']:
                source['imu']['gyro_rad_s'][0] = ["later caller mutation"]
            return result
        expected = observer.snapshot_event(source)
        with patch.object(observer, '_flat_snapshot_scalars', side_effect=mutate):
            actual = observer._flat_feedback_snapshot(source)
        self.assertIsNotNone(actual)
        self.assertEqual(actual, expected)

    def test_complete_observer_records_recurrent_history_and_mutation_match(self):
        clocks = [iter(range(100, 5000, 10)) for _ in range(2)]
        runs = [make(max_ticks=8, profile_consume=True,
                     monotonic_ns=lambda c=c: next(c)) for c in clocks]
        for run in runs: run.reset_run(1_000_000_000, warmup_completed=True)
        for index in range(8):
            source = native_shape(1_000_000_000 + index * observer.DT_NS)
            source['imu']['gyro_rad_s'][0] += index * .003
            before = copy.deepcopy(source)
            with patch.object(observer, '_flat_feedback_snapshot', return_value=None):
                expected = runs[0].consume(source)
            actual = runs[1].consume(source)
            self.assertEqual(actual, expected)
            self.assertEqual(source, before)
            self.assertEqual(runs[0]._policy.calls, runs[1]._policy.calls)
            self.assertEqual(runs[0]._last_sources, runs[1]._last_sources)
            self.assertEqual(runs[0]._next_tick_ns, runs[1]._next_tick_ns)
            self.assertTrue(set(mutable_ids(actual)).isdisjoint(mutable_ids(expected)))
            actual['provenance']['snapshot_source_flags'].clear()
            actual['inputs']['gyro_body_rad_s'].clear()
        self.assertEqual(runs[0].finish(), runs[1].finish())

    def test_semantic_and_model_rejections_keep_state_failure_order(self):
        mutations = (
            lambda s: s.update(tick_ns=-1),
            lambda s: s.update(output_allowed=True),
            lambda s: s['motors'][0].update(motor_id=True),
            lambda s: s['motors'][0].update(received_ns=2_000_000_000),
            lambda s: s['motors'][0].update(value=99.),
        )
        for index, change in enumerate(mutations):
            source = native_shape(); change(source)
            runs = [make() for _ in range(2)]
            for run in runs: run.reset_run(1_000_000_000, warmup_completed=True)
            with patch.object(observer, '_flat_feedback_snapshot', return_value=None):
                with self.assertRaises(observer.ObserverError) as old: runs[0].consume(source)
            with self.assertRaises(observer.ObserverError) as new: runs[1].consume(source)
            self.assertEqual(str(old.exception), str(new.exception))
            self.assertEqual(runs[0].summary(), runs[1].summary())
            self.assertEqual(runs[0]._policy.calls, runs[1]._policy.calls)
        for bad in ('actor_nan', 'observation_batch', 'target_shape', 'target_bounds', 'interrupt'):
            runs = [make(Policy(bad=bad)) for _ in range(2)]
            for run in runs: run.reset_run(1_000_000_000, warmup_completed=True)
            source = native_shape()
            errors = []
            for i, run in enumerate(runs):
                try:
                    if i == 0:
                        with patch.object(observer, '_flat_feedback_snapshot', return_value=None):
                            run.consume(source)
                    else: run.consume(source)
                except BaseException as error: errors.append((type(error), str(error)))
            self.assertEqual(len(errors), 2)
            self.assertEqual(errors[0], errors[1])
            self.assertEqual(runs[0].summary(), runs[1].summary())
            self.assertEqual(runs[0]._policy.calls, runs[1]._policy.calls)
            self.assertEqual(runs[1].ticks_completed, 0)


if __name__ == '__main__':
    unittest.main()
