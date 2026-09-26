"""Synthetic differential checks; no device, model or private capture needed.

Build event-copy and input-validator extensions first, then run:
    python3 -m unittest test_native_input_validation -v
"""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
RUNTIME = HERE.parents[1]
sys.path[:0] = [str(RUNTIME), str(RUNTIME / 'tests')]
from test_fast_policy_inputs import PreparedInputTests, reply, imu, TICK
from singularitydog_hw import fast_policy_inputs as reference
import native_fast_policy_inputs as native

class ExistingPreparationContracts(PreparedInputTests):
    """Run existing invalid-input/ownership contracts against the native adapter."""
    def setUp(self):
        super().setUp()
        selection = patch.object(reference, 'prepare_cycle', native.prepare_cycle)
        selection.start()
        self.addCleanup(selection.stop)

class DifferentialNativeValidation(unittest.TestCase):
    def test_values_hashes_and_input_order(self):
        rows = [reply(i) for i in range(1, 13)]
        sample = imu()
        for shift in range(12):
            ordered = rows[shift:] + rows[:shift]
            a = reference.prepare_cycle(ordered, sample)
            b = native.prepare_cycle(ordered, sample)
            self.assertEqual(a, b)
            self.assertEqual(a.snapshot(TICK), b.snapshot(TICK))

    def test_every_scalar_field_rejects_with_same_error(self):
        def outcome(function, rows):
            try:
                return ('ok', function(rows, imu()))
            except Exception as error:
                return (type(error).__name__, str(error))
        base = [reply(i) for i in range(1, 13)]
        for group in ('row', 'result', 'raw'):
            source = base[0] if group == 'row' else base[0]['result']
            if group == 'raw': source = source['raw_frame']
            for key in source:
                for value in (None, True, False, -1, 2**1000, float('inf'), 'invalid'):
                    with self.subTest(group=group, key=key, value=value):
                        rows = copy.deepcopy(base)
                        target = rows[0] if group == 'row' else rows[0]['result']
                        if group == 'raw': target = target['raw_frame']
                        target[key] = value
                        self.assertEqual(outcome(reference.prepare_cycle, rows),
                                         outcome(native.prepare_cycle, rows))

if __name__ == '__main__':
    unittest.main()
