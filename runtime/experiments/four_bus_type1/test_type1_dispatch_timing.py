"""File-only F0 dispatch stamps; no transport, motor, or model is opened."""
import unittest

from .test_type1_30hz import Mock30, admitted30
from .test_type1_options import FakeEvidence, OptionHarness


class OwnerStartStall(Mock30):
    def hold_then_voltage(self, wires, voltage_id, prefix_future, *, deadline_ns, check):
        if self.group.port == 'port2' and self.holds == 0:
            self.h.clock.advance_to(self.h.clock.now + 4_000_000)
        return super().hold_then_voltage(wires, voltage_id, prefix_future,
                                         deadline_ns=deadline_ns, check=check)


class DispatchTimingTests(OptionHarness, unittest.TestCase):
    mock = OwnerStartStall

    def test_port_specific_owner_delay_is_visible_without_changing_admission(self):
        result = self.run_case(value=admitted30(), timing_evidence=FakeEvidence())
        self.assertEqual(result['status'], 'COMPLETE_FOUR_BUS_TYPE1_LEARNED_BOXED', result['primary_error'])
        self.assertTrue(result['stop_confirmed'])
        first = result['timing_evidence']['dispatch_timing_rows'][0]
        self.assertLess(first['guard_begin_ns'], first['guard_end_ns'])
        self.assertEqual(set(first['hold_submit_by_port_ns']), {'port0', 'port1', 'port2', 'port3'})
        self.assertEqual(set(first['owner_entry_ns']), {'port0', 'port1', 'port2', 'port3'})
        self.assertLess(first['hold_submit_by_port_ns']['port2']['begin_ns'],
                        first['hold_submit_by_port_ns']['port2']['end_ns'])
        self.assertGreaterEqual(result['cycles'][0]['hold']['port2'].records[0].start_ns -
                                first['owner_entry_ns']['port2'], 4_000_000)
        self.assertEqual(result['timing_evidence']['cycle_write_observation']
                         ['cycles_without_observed_final_host_write'], 0)


class GuardStallTests(OptionHarness, unittest.TestCase):
    mock = Mock30

    def setUp(self):
        super().setUp()
        self.stall_next_guard = False

    def release_wait(self, when):
        result = super().release_wait(when)
        self.stall_next_guard = True
        return result

    def current(self):
        if self.stall_next_guard:
            self.stall_next_guard = False
            self.clock.advance_to(self.clock.now + 35_000_000)
        super().current()

    def test_failed_guard_retains_actual_times_and_same_gap_stop(self):
        result = self.run_case(value=admitted30(), timing_evidence=FakeEvidence())
        self.assertEqual(result['status'], 'ABORTED')
        self.assertEqual(result['primary_error']['message'],
                         'Command/sample gap exceeded before feedback hold')
        self.assertTrue(result['stop_confirmed'])
        failed = result['timing_evidence']['failed_cycle']
        self.assertEqual(failed['index'], 0)
        self.assertGreaterEqual(failed['guard_end_ns'] - failed['guard_begin_ns'], 35_000_000)
        self.assertIsNone(failed['hold_submit_by_port_ns'])
        self.assertIsNone(failed['owner_entry_ns'])
        self.assertIsNone(failed['output_last_host_write_ns'])
        self.assertIsNone(failed['thread_counter_delta_at_failure'])
        self.assertEqual(result['timing_evidence']['cycle_write_observation'], {
            'command_gap_budget_ms': 34.333334,
            'input_to_tx_window_ms': 20.,
            'cycles_total': 1,
            'cycles_with_actual_final_host_write': 0,
            'cycles_without_observed_final_host_write': 1,
            'observed_write_deadline_overruns': None})


if __name__ == '__main__':
    unittest.main()
