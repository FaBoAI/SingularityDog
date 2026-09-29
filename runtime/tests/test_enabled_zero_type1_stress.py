"""No-hardware tests of the finite enabled, zero-gain transport probe."""

import unittest
from unittest.mock import patch

from singularitydog_hw import enabled_zero_type1_stress as stress
from singularitydog_hw import disabled_type1_stress as disabled
from singularitydog_hw import watchdog_commissioning as watchdog


class FakeChannel:
    def __init__(self, ids, *, fail_active_id=None, stop_complete=True):
        self.ids = ids
        self.fail_active_id = fail_active_id
        self.stop_complete = stop_complete
        self.calls = []
        self.enabled = set()
        self.stops = 0

    def exchange(self, mid, step, *, center=0.):
        self.calls.append((mid, step, center))
        if step == "identity":
            return {"mcu_uid_hex": (bytes([mid])*8).hex()}
        if step == "stop":
            return {"mode_state": 0, "fault_bits": 0, "protocol_position_rad": 0.}
        if step == "run_mode": return {"value": 0}
        if step == "voltage": return {"value": 40.}
        if step == "can_timeout": return {"value": 4000}
        if step == "watchdog_write": return {"mode_state": 0, "fault_bits": 0}
        if step == "enable":
            self.enabled.add(mid)
            return {"mode_state": 2, "fault_bits": 0}
        if step == "zero":
            mode = 2 if mid in self.enabled else 0
            if mode == 2 and mid == self.fail_active_id:
                raise TimeoutError("simulated active Type1 loss")
            return {"mode_state": mode, "fault_bits": 0,
                    "protocol_position_rad": center, "velocity_rad_s": 0.,
                    "temperature_c": 25.}
        raise AssertionError(step)

    def stop_all(self):
        self.stops += 1
        return {"complete": self.stop_complete,
                "unconfirmed_ids": [] if self.stop_complete else [self.ids[0]],
                "errors": []}


class EnabledZeroType1StressTests(unittest.TestCase):
    def make_case(self, **front_kwargs):
        channels = {scope: FakeChannel(ids, **(front_kwargs if scope == "front" else {}))
                    for scope, ids in watchdog.BUSES.items()}
        expected = {mid: (bytes([mid])*8).hex() for mid in watchdog.IDS}
        return channels, expected

    def test_plan_is_bounded_and_has_no_positive_gain(self):
        for count in (0, 501, True, 1.5):
            with self.assertRaises(ValueError): stress.plan(count)
        p = stress.plan(5)
        self.assertFalse(p["hardware_opened"])
        self.assertFalse(p["positive_gain_available"])
        self.assertFalse(p["learned_targets_available"])

    def test_enabled_zero_stream_preflight_and_stop(self):
        channels, expected = self.make_case()
        calls = []
        def checked_batch(owner, ids, centers, check, deadline, *, expected_mode):
            self.assertEqual(expected_mode, 2)
            self.assertEqual(owner.enabled, set(ids))
            self.assertEqual(set(centers), set(watchdog.IDS))
            calls.append((owner, tuple(ids)))
            for mid in ids:
                disabled._check_zero(owner.exchange(mid, "zero", center=centers[mid]),
                                     mid, centers[mid], expected_mode=2)
        with patch.object(disabled, "_zero_batch", side_effect=checked_batch):
            result = stress.run(channels, expected, cycles=2)
        self.assertEqual(result["status"], "COMPLETE_ENABLED_ZERO_TYPE1_DIAGNOSTIC", result)
        self.assertEqual(result["cycles_completed"], 2)
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(len(calls), 8)
        self.assertFalse(result["positive_gain_sent"] or result["learned_targets_sent"])
        for scope, channel in channels.items():
            self.assertEqual(channel.stops, 1)
            self.assertEqual(channel.enabled, set(watchdog.BUSES[scope]))
            self.assertEqual({step for _, step, _ in channel.calls},
                             {"identity", "stop", "run_mode", "voltage", "watchdog_write",
                              "can_timeout", "zero", "enable"})

    def test_active_reply_loss_aborts_and_attempts_both_stops(self):
        channels, expected = self.make_case(fail_active_id=1)
        result = stress.run(channels, expected, cycles=1)
        self.assertEqual(result["status"], "ABORTED")
        self.assertEqual(result["cycles_completed"], 0)
        self.assertTrue(result["motor_enable_attempted"])
        self.assertEqual([c.stops for c in channels.values()], [1, 1])

    def test_unconfirmed_stop_requires_power_cutoff(self):
        channels, expected = self.make_case(stop_complete=False)
        result = stress.run(channels, expected, cycles=1)
        self.assertEqual(result["status"], "STOP_UNCONFIRMED_POWER_OFF_REQUIRED")
        self.assertFalse(result["stop_confirmed"])

    def test_active_mode_not_accepted_as_disabled_and_reverse(self):
        reply = {"mode_state": 2, "fault_bits": 0, "protocol_position_rad": 0.,
                 "velocity_rad_s": 0., "temperature_c": 25.}
        with self.assertRaises(RuntimeError): disabled._check_zero(reply, 1, 0.)
        disabled._check_zero(reply, 1, 0., expected_mode=2)

    def test_evidence_tail_keeps_counts_without_unbounded_raw_events(self):
        channel = object.__new__(stress.EvidenceChannel)
        from collections import Counter, deque
        channel.events = deque(maxlen=stress.EVENT_TAIL)
        channel.event_total = 0
        channel.tx_steps = Counter()
        channel.rx_bytes = 0
        channel.rejected_rx_bytes = 0
        for _ in range(stress.EVENT_TAIL+5):
            channel.event({"kind": "tx", "step": "zero"})
        channel.event({"kind": "rx_bytes", "hex": "4154"})
        channel.event({"kind": "rx_rejected", "hex": "000102"})
        evidence = channel.evidence()
        self.assertEqual(evidence["event_total"], stress.EVENT_TAIL+7)
        self.assertEqual(evidence["tx_steps"]["zero"], stress.EVENT_TAIL+5)
        self.assertEqual(evidence["accepted_rx_bytes"], 2)
        self.assertEqual(evidence["rejected_rx_bytes"], 3)
        self.assertEqual(evidence["rx_bytes"], 5)
        self.assertTrue(evidence["event_tail_truncated"])
        self.assertEqual(evidence["event_tail_count"], stress.EVENT_TAIL)


if __name__ == "__main__": unittest.main()
