"""File-only checks of the batched-scan rewrite; the CPU model run needs the private bundle."""
import os
from pathlib import Path
import unittest

from . import equivalence as E

BUNDLE = Path(__file__).resolve().parents[4]/'RS05_30Hz_Model_20261013'


class RewriteTests(unittest.TestCase):
    def test_plan_reads_nothing(self):
        self.assertEqual(E.main([]), 0)

    def test_rewrite_requires_exact_single_frozen_loop(self):
        frozen = ('x\n'+E.SEQUENTIAL+'            reject=active&(~ok)\n').encode()
        rewritten = E.batched_source(frozen)
        self.assertNotIn('q,ok,_domain=self.feasible_ik(candidate)', rewritten)
        self.assertIn('self.feasible_ik(candidates.reshape(', rewritten)
        self.assertIn('            reject=active&(~ok)\n', rewritten)
        with self.assertRaisesRegex(ValueError, 'exactly once'):
            E.batched_source(b'no loop here')
        with self.assertRaisesRegex(ValueError, 'exactly once'):
            E.batched_source((E.SEQUENTIAL*2).encode())

    def test_pins_reject_changed_files(self):
        path = Path(os.devnull)
        with self.assertRaisesRegex(ValueError, 'Pinned file differs'):
            E._pinned(path, E.SWING_CORE_SHA256)

    @unittest.skipUnless((BUNDLE/'source/swing_core.py').exists(), 'private 30 Hz bundle not present')
    def test_frozen_bundle_short_run_is_bitwise_equal(self):
        import tempfile
        import torch
        torch.set_num_threads(1)
        stats = {'ticks': 0, 'compared_tensors': 0, 'mismatches': 0, 'exceptions_matched': 0,
                 'exception_mismatch': 0, 'step_ns': {'reference': [], 'candidate': []}}
        with tempfile.TemporaryDirectory() as workdir:
            reference, candidate = E.build(BUNDLE, workdir)
            E.synthetic(reference, candidate, 1, stats)
        self.assertEqual(stats['ticks'], 840)
        self.assertEqual(stats['mismatches'], 0)
        self.assertEqual(stats['exception_mismatch'], 0)


if __name__ == '__main__':
    unittest.main()
