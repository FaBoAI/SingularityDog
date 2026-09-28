"""Ownership and resource limits for strict event snapshots; no device access."""
import copy
import gc
import json
import math
import pathlib
import subprocess
import sys
import unittest
import weakref
from unittest.mock import patch

from singularitydog_hw.event_snapshot import snapshot_event


class EventSnapshotTests(unittest.TestCase):
    def test_content_types_and_json_match_deepcopy_without_serialization(self):
        source = {'kind': 'imu', 'timestamp': 1_000_000_000, 'ok': True,
                  'none': None, 'float': -1.5, 'text': '温度\n😀',
                  'raw': {'hex': '415400'}, 'values': [1, 2., {'x': [0.]}],
                  'tuple': (1, {'y': [2, 3]})}
        with patch.object(json, 'dumps', side_effect=AssertionError('hot JSON')), \
             patch.object(json, 'loads', side_effect=AssertionError('hot JSON')):
            result = snapshot_event(source)
        self.assertEqual(result, copy.deepcopy(source))
        self.assertIs(type(result['tuple']), tuple)
        self.assertEqual(json.dumps(result, allow_nan=False),
                         json.dumps(copy.deepcopy(source), allow_nan=False))
        source['values'][2]['x'].append(99.)
        source['tuple'][1]['y'].clear()
        source['raw']['hex'] = 'changed'
        self.assertEqual(result['values'][2]['x'], [0.])
        self.assertEqual(result['tuple'][1]['y'], [2, 3])
        self.assertEqual(result['raw']['hex'], '415400')

    def test_repeated_mutable_aliases_are_owned_and_bounded_per_occurrence(self):
        shared = {'samples': [1, 2]}
        source = {'a': shared, 'b': [shared]}
        result = snapshot_event(source)
        self.assertIsNot(result['a'], shared)
        self.assertIsNot(result['b'][0], shared)
        self.assertIsNot(result['a'], result['b'][0])
        result['a']['samples'].append(3)
        self.assertEqual(shared['samples'], [1, 2])
        self.assertEqual(result['b'][0]['samples'], [1, 2])
        # Reusing one small container cannot evade the expanded-tree budget.
        with self.assertRaisesRegex(ValueError, 'node'):
            snapshot_event([shared] * 10, max_nodes=20)

    def test_scalar_fast_path_shares_only_exact_immutable_values(self):
        for value in ('plain', '日本語', 17, 0, -18, 1.5, True, False, None):
            with self.subTest(value=value):
                self.assertIs(snapshot_event(value), value)

    def test_nonfinite_values_rejected_at_any_depth(self):
        for value in (math.nan, math.inf, -math.inf):
            for source in (value, {'x': value}, [({'x': [value]},)]):
                with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'Nonfinite'):
                    snapshot_event(source)

    def test_unknown_types_and_subclasses_never_invoke_copy_hooks(self):
        class Hook:
            def __deepcopy__(self, memo):
                raise AssertionError('copy hook invoked')
        class Number(int): pass
        class Text(str): pass
        class Mapping(dict): pass
        for value in (Hook(), object(), b'raw', {1, 2}, complex(1, 2),
                      Number(1), Text('x'), Mapping(x=1)):
            with self.subTest(kind=type(value)), self.assertRaises(TypeError):
                snapshot_event({'nested': value})
        for key in (1, True, None, Text('x')):
            with self.subTest(key=key), self.assertRaisesRegex(TypeError, 'keys'):
                snapshot_event({key: 1})

    def test_cycles_rejected_including_mixed_tuple_container_cycle(self):
        sequence = []; sequence.append(sequence)
        mapping = {}; mapping['self'] = mapping
        bridge = []; cycle = (bridge,); bridge.append(cycle)
        for source in (sequence, mapping, cycle):
            with self.subTest(kind=type(source)), self.assertRaisesRegex(ValueError, 'Cyclic'):
                snapshot_event(source)

    def test_depth_and_node_limits_include_keys_and_empty_containers(self):
        self.assertEqual(snapshot_event({}, max_depth=0, max_nodes=1), {})
        self.assertEqual(snapshot_event({'x': [1]}, max_depth=2, max_nodes=4), {'x': [1]})
        with self.assertRaisesRegex(ValueError, 'depth'):
            snapshot_event({'x': [1]}, max_depth=1)
        with self.assertRaisesRegex(ValueError, 'node'):
            snapshot_event({'x': [1]}, max_nodes=3)
        with self.assertRaisesRegex(ValueError, 'node'):
            snapshot_event([None] * 20_001)

    def test_byte_budget_covers_scalars_keys_escaping_and_repeated_aliases(self):
        for source in ('x' * 10, '\x00' * 10, '😀' * 10,
                       {'long_key': None}, [1.5] * 10, 1 << 200):
            with self.subTest(kind=type(source)), self.assertRaisesRegex(ValueError, 'byte'):
                snapshot_event(source, max_bytes=20)
        self.assertEqual(snapshot_event([], max_bytes=2), [])
        with self.assertRaisesRegex(ValueError, 'byte'):
            snapshot_event([], max_bytes=1)
        shared = {'text': 'z' * 100}
        with self.assertRaisesRegex(ValueError, 'byte'):
            snapshot_event([shared] * 10, max_bytes=5_000)

    def test_invalid_bounds_fail_before_touching_source(self):
        for options in ({'max_depth': True}, {'max_depth': -1}, {'max_depth': 65},
                        {'max_nodes': 0}, {'max_nodes': 1.5}, {'max_bytes': 0},
                        {'max_bytes': True}):
            with self.subTest(options=options), self.assertRaisesRegex(ValueError, 'bounds'):
                snapshot_event(object(), **options)

    def test_recursive_helper_has_no_self_closure_and_dies_without_collection(self):
        visitors = []
        def capture(frame, event, result):
            if event == 'return' and frame.f_code is snapshot_event.__code__:
                visitors.append(frame.f_locals['visit'])
        previous = sys.getprofile()
        enabled = gc.isenabled()
        gc.disable()
        try:
            sys.setprofile(capture)
            self.assertEqual(snapshot_event({'nested': [({'values': [1., None]},)]}),
                             {'nested': [({'values': [1., None]},)]})
            sys.setprofile(previous)
            self.assertEqual(len(visitors), 1)
            visitor = visitors.pop()
            self.assertFalse(any(cell.cell_contents is visitor for cell in visitor.__closure__))
            reference = weakref.ref(visitor)
            del visitor
            # Refcount reclamation must suffice while automatic GC is disabled.
            # No gc.collect() is needed to release this per-clone helper.
            self.assertIsNone(reference())
        finally:
            sys.setprofile(previous)
            if enabled:
                gc.enable()

    def test_gc_disabled_clones_create_no_per_call_unreachable_growth(self):
        # A fresh interpreter excludes unrelated unittest/mock reference cycles
        # from the manual collection counts and restores its own GC in finally.
        script = '''
import gc, importlib.util, json, sys
spec = importlib.util.spec_from_file_location('owned_event_copy', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
shared = {'samples': [1, 2., None, False]}
valid = (0, None, 'text', {}, [], (), {'a': shared, 'b': [shared, ({'deep': [3]},)]})
invalid = ({'bad': float('nan')}, {'bad': object()}, {1: 'bad key'}, {'deep': [[1]]})
enabled = gc.isenabled()
gc.disable()
try:
    gc.collect()
    counts = []
    for calls in (1, 100, 1000):
        for _ in range(calls):
            for source in valid:
                module.snapshot_event(source)
            for source in invalid:
                try:
                    module.snapshot_event(source, max_depth=2)
                except (TypeError, ValueError):
                    pass
                else:
                    raise AssertionError('Invalid event accepted')
        counts.append(gc.collect())
    assert counts == [0, 0, 0], counts
    print(json.dumps({'manual_unreachable_counts': counts, 'calls_each': [1, 100, 1000],
                      'automatic_gc_enabled': gc.isenabled()}))
finally:
    if enabled:
        gc.enable()
'''
        path = pathlib.Path(sys.modules[snapshot_event.__module__].__file__).resolve()
        result = subprocess.run([sys.executable, '-I', '-c', script, str(path)],
                                check=True, capture_output=True, text=True, timeout=20)
        evidence = json.loads(result.stdout)
        self.assertEqual(evidence['manual_unreachable_counts'], [0, 0, 0])
        self.assertFalse(evidence['automatic_gc_enabled'])


if __name__ == '__main__':
    unittest.main()
