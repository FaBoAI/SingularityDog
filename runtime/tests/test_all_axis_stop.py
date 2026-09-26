"""No device access: bounded fake exchanges for the additive stop coordinator."""
from dataclasses import replace
import unittest

from singularitydog_hw.all_axis_stop import StopExchange, coordinate_stop
from singularitydog_hw.can_readonly import ATParser, Frame


class Clock:
    def __init__(self):
        self.now = 1_000_000

    def __call__(self):
        return self.now


def frame(mid, *, kind=2, mode=0, fault=0, destination=0xFD):
    cid = (kind << 24) | (mode << 22) | (fault << 16) | (mid << 8) | destination
    data = bytes.fromhex("7fff7fff7fff012c")
    wire = b"AT" + ((cid << 3) | 4).to_bytes(4, "big") + b"\x08" + data + b"\r\n"
    return Frame(cid, 4, data, wire)


class Backend:
    def __init__(self, scope, clock, calls, failures=None, transform=None):
        self.scope, self.clock, self.calls = scope, clock, calls
        self.failures, self.transform = failures or {}, transform

    def stop_exchange(self, wire, *, deadline_ns):
        parsed = ATParser().feed(wire)
        assert len(wire) == 17 and len(parsed) == 1
        f = parsed[0]
        assert f.kind == 4 and f.data == bytes(8) and f.flags == 4
        assert (f.can_id >> 8) & 0xffff == 0xFD
        mid = f.destination
        assert mid in (range(1, 7) if self.scope == "front" else range(7, 13))
        self.calls.append((self.scope, mid, deadline_ns))
        started = self.clock.now
        self.clock.now += 3
        if mid in self.failures:
            raise self.failures[mid]
        value = StopExchange(started, started+1, 17, started+2, frame(mid), True, True)
        return self.transform(value, mid, deadline_ns) if self.transform else value


class AllAxisStopTests(unittest.TestCase):
    def setup_pair(self, **front_options):
        clock, calls = Clock(), []
        front = Backend("front", clock, calls, **front_options)
        rear = Backend("rear", clock, calls)
        return clock, calls, front, rear

    def check_schedule(self, calls):
        self.assertEqual([(s, i) for s, i, _ in calls],
                         [(s, i) for n in range(1, 7) for s, i in (("front", n), ("rear", n+6))])

    def test_all_twelve_canonical_stops_and_fresh_replies(self):
        clock, calls, front, rear = self.setup_pair()
        report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
        self.check_schedule(calls)
        self.assertEqual(report["status"], "ALL_STOP_REPLIES_CONFIRMED")
        self.assertTrue(report["all_stop_replies_confirmed"])
        self.assertEqual(sorted(report["confirmed_ids"]), list(range(1, 13)))
        self.assertEqual(report["requests_attempted"], 12)
        self.assertEqual(report["backend_wait_budget_ns"], 1200)
        for key in ("physical_stop_verified", "hardware_watchdog_available", "drive_authorized", "automatic_retry"):
            self.assertIs(report[key], False)

    def test_whole_front_bus_failure_still_attempts_every_axis(self):
        clock, calls, front, rear = self.setup_pair(failures={i: OSError("bus down") for i in range(1, 7)})
        report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
        self.check_schedule(calls)
        self.assertFalse(report["all_stop_replies_confirmed"])
        self.assertEqual(report["confirmed_ids"], list(range(7, 13)))

    def test_one_id_failure_and_cancellation_do_not_skip_siblings(self):
        for error in (TimeoutError("missing"), OSError("partial write"), KeyboardInterrupt(), SystemExit(2)):
            with self.subTest(error=type(error).__name__):
                clock, calls, front, rear = self.setup_pair(failures={2: error})
                report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
                self.check_schedule(calls)
                self.assertEqual(len(report["confirmed_ids"]), 11)
                self.assertNotIn(2, report["confirmed_ids"])

    def test_broken_exception_message_cannot_suppress_other_stops(self):
        class BrokenError(Exception):
            def __str__(self):
                raise RuntimeError("formatter failed")
        clock, calls, front, rear = self.setup_pair(failures={1: BrokenError()})
        report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
        self.check_schedule(calls)
        self.assertEqual(report["motors"][0]["error"], "BrokenError: exception text unavailable")
        self.assertEqual(len(report["confirmed_ids"]), 11)

    def test_rejects_partial_missing_mismatched_faulted_and_nonreset(self):
        changes = [lambda e: None,
                   lambda e: replace(e, write_returned_bytes=16),
                   lambda e: replace(e, write_returned_bytes=True),
                   lambda e: replace(e, frame=frame(2)),
                   lambda e: replace(e, frame=frame(1, destination=0xFE)),
                   lambda e: replace(e, frame=frame(1, kind=21)),
                   lambda e: replace(e, frame=frame(1, mode=2)),
                   lambda e: replace(e, frame=frame(1, fault=1)),
                   lambda e: replace(e, frame=replace(e.frame, wire=e.frame.wire[:-1]+b"x")),
                   lambda e: replace(e, frame=replace(e.frame, flags=0)),
                   lambda e: replace(e, clean_start=False),
                   lambda e: replace(e, clean_end=False),
                   lambda e: replace(e, clean_start=1),
                   lambda e: replace(e, received_ns=e.write_started_ns-1),
                   lambda e: replace(e, write_finished_ns=e.received_ns+1),
                   lambda e: replace(e, received_ns=True)]
        for change in changes:
            with self.subTest(change=changes.index(change)):
                clock, calls, front, rear = self.setup_pair(transform=lambda e, mid, d: change(e) if mid == 1 else e)
                report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
                self.check_schedule(calls)
                self.assertFalse(report["all_stop_replies_confirmed"])
                self.assertFalse(report["motors"][0]["confirmed"])
                self.assertEqual(len(report["confirmed_ids"]), 11)

    def test_late_backend_cannot_confirm_even_with_on_time_reply(self):
        for offset in (0, 1):
            clock, calls, front, rear = self.setup_pair()
            def late(value, mid, deadline):
                if mid == 1:
                    clock.now = deadline+offset
                return value
            front.transform = late
            report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
            self.check_schedule(calls)
            self.assertFalse(report["motors"][0]["confirmed"])
            self.assertEqual(len(report["confirmed_ids"]), 11)

    def test_all_timeouts_are_twelve_calls_without_polling_or_retry(self):
        clock, calls, front, rear = self.setup_pair()
        def timeout(value, mid, deadline):
            clock.now = deadline
            raise TimeoutError("absolute deadline")
        front.transform = rear.transform = timeout
        report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
        self.check_schedule(calls)
        self.assertEqual(report["finished_ns"]-report["started_ns"], 1200)
        self.assertEqual(report["confirmed_ids"], [])

    def test_input_rejection_occurs_before_backend_calls(self):
        clock, calls, front, rear = self.setup_pair()
        for timeout in (0, -1, True, 1.0, 1_000_000_001):
            with self.assertRaises(ValueError):
                coordinate_stop(front, rear, timeout_ns=timeout, clock=clock)
        with self.assertRaises(ValueError):
            coordinate_stop(front, front, timeout_ns=100, clock=clock)
        self.assertEqual(calls, [])

    def test_result_has_detached_mutable_evidence(self):
        clock, calls, front, rear = self.setup_pair()
        report = coordinate_stop(front, rear, timeout_ns=100, clock=clock)
        report["motors"][0]["exchange"]["raw_frame"].clear()
        self.assertTrue(report["motors"][1]["exchange"]["raw_frame"])


if __name__ == "__main__":
    unittest.main()
