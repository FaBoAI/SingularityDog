"""File-only regression for optional F0 setup before the first active hold."""
import unittest

from .test_type1_30hz import Mock30, admitted30
from .test_type1_options import FakeEvidence, OptionHarness


class SlowInitialEvidence(FakeEvidence):
    """Make only diagnostic initialization expensive on a virtual monotonic clock."""

    def __init__(self, clock):
        super().__init__()
        self.clock = clock

    def bind(self, threads):
        self.clock.advance_to(self.clock.now + 20_000_000)
        return super().bind(threads)

    def run_counters(self):
        if self.runs == 0:
            self.clock.advance_to(self.clock.now + 12_000_000)
        return super().run_counters()

    def sample(self):
        if self.samples == 0:
            self.clock.advance_to(self.clock.now + 5_000_000)
        return super().sample()


class InitialTimingEvidenceTests(OptionHarness, unittest.TestCase):
    mock = Mock30

    def test_slow_f0_setup_precedes_fresh_initial_hold_and_does_not_consume_first_gap(self):
        reader = SlowInitialEvidence(self.clock)
        result = self.run_case(value=admitted30(), timing_evidence=reader)
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', result['primary_error'])
        self.assertTrue(result['stop_confirmed'])
        diagnostic = result['timing_evidence']
        self.assertGreaterEqual(diagnostic['setup_ms'], 37)
        first_hold_write = min(record.start_ns for batch in result['initial_hold'].values()
                               for record in batch.records)
        self.assertLess(diagnostic['setup_end_ns'], first_hold_write)
        span = diagnostic['initial_hold_to_epoch']
        self.assertLess(span['last_command_to_epoch_ms'], 34.333334)
        self.assertLess(span['last_sample_to_epoch_ms'], 34.333334)
        self.assertNotIn('failed_pre_hold_gap', diagnostic)
        self.assertEqual(reader.samples, len(result['cycles']) + 1)
        self.assertTrue(reader.closed)


if __name__ == '__main__':
    unittest.main()
