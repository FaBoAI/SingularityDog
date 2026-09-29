"""Offline finite read-only recorder tests; no serial or hardware access."""

from contextlib import redirect_stdout
import io
import json
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from singularitydog_hw import stationary_readonly_capture as capture


EXPECTED = {mid: f"{mid:016x}" for mid in range(1, 13)}


class Clock:
    def __init__(self):
        self.local = threading.local()
    def __call__(self):
        return getattr(self.local, "now", 1_000_000_000)
    def wait(self, seconds):
        self.local.now = self() + max(1, int(seconds * 1e9))


class FakeCAN:
    def __init__(self, bus, sink, clock, all_calls, *, fail=None, wrong_uid=False, delay_ns=0):
        self.bus, self.sink, self.clock = bus, sink, clock
        self.all_calls, self.fail, self.wrong_uid, self.delay_ns = all_calls, fail, wrong_uid, delay_ns
        self.parser = SimpleNamespace(buffer=b"", discarded_bytes=0)
        self.poisoned = False
        self.tx_count = self.rx_bytes = 0
        self.owners = []
    def __enter__(self):
        self.owners.append(threading.get_ident()); return self
    def __exit__(self, *_):
        self.owners.append(threading.get_ident()); self.poisoned = True
    def query(self, mid, parameter=None):
        self.owners.append(threading.get_ident())
        if self.poisoned:
            raise AssertionError("Poisoned owner reused")
        before = self.clock()
        event = {"kind": "can_tx", "motor_id": mid, "parameter": parameter or "identity",
                 "hex": capture.codec.read_request(mid, parameter).hex(), "monotonic_ns": before}
        self.sink(event)
        self.all_calls.append((mid, parameter, threading.get_ident()))
        self.tx_count += 1
        if (mid, parameter) == self.fail:
            self.poisoned = True
            raise TimeoutError("Injected read timeout")
        self.clock.wait((1000+self.delay_ns)/1e9)
        self.rx_bytes += 17
        reply = {"ok": True, "motor_id": mid, "parameter": parameter or "identity",
                 "request_monotonic_ns": before, "monotonic_ns": self.clock()}
        if parameter is None:
            reply["mcu_uid_hex"] = "f"*16 if self.wrong_uid else EXPECTED[mid]
        else:
            index, _, unit = capture.codec.PARAMETERS[parameter]
            reply.update(index=index, unit=unit, value={"position": mid/10, "velocity": .6,
                         "current": .01, "run_mode": 0, "voltage": 40.}[parameter])
        return reply


class StationaryCaptureTests(unittest.TestCase):
    def run_capture(self, **options):
        clock, calls, cans = Clock(), [], {}
        def factory(bus, sink):
            cans[bus] = FakeCAN(bus, sink, clock, calls, **options)
            return cans[bus]
        result = capture.capture(factory, EXPECTED, clock=clock, wait=clock.wait)
        return result, calls, cans

    def test_plan_only_never_opens_or_reads_inputs(self):
        with patch.object(capture, "capture", side_effect=AssertionError("no capture")), \
                patch.object(capture, "_expected_uids", side_effect=AssertionError("no file read")), \
                patch.object(capture.dual, "validate_ports", side_effect=AssertionError("no ports")), \
                redirect_stdout(io.StringIO()) as output:
            self.assertEqual(capture.main([]), 0)
        plan = json.loads(output.getvalue())
        self.assertEqual(plan["status"], "PLAN_ONLY")
        self.assertEqual((plan["sweeps"], plan["period_ms"], plan["duration_s"]), (50, 200, 10))
        self.assertEqual(plan["allowed_can_types"], [0, 17])
        self.assertFalse(plan["hardware_opened"] or plan["motor_output_allowed"] or plan["stop_sent"])

    def test_fifty_sweeps_follow_all_uids_with_single_bus_owners(self):
        result, calls, cans = self.run_capture()
        self.assertEqual(result["status"], "COMPLETE_STATIONARY_READONLY_CAPTURE", result["errors"])
        self.assertEqual({mid for mid, parameter, _ in calls[:12] if parameter is None}, set(EXPECTED))
        self.assertEqual(len(calls), 3012)
        self.assertEqual(result["queries_sent"], 3012)
        self.assertNotEqual(result["buses"]["front"]["owner_thread_id"], result["buses"]["rear"]["owner_thread_id"])
        for bus, data in result["buses"].items():
            self.assertEqual(cans[bus].tx_count, 1506)
            self.assertEqual(len(set(cans[bus].owners)), 1)
            self.assertEqual(len(data["sweeps"]), 50)
            self.assertGreaterEqual(data["capture_end_ns"] - data["capture_begin_ns"], 10_000_000_000)
            self.assertTrue(all(b["begin_ns"]-a["begin_ns"] >= 200_000_000
                                for a, b in zip(data["sweeps"], data["sweeps"][1:])))
            for row in data["sweeps"]:
                self.assertTrue(row["complete"])
                for sample in row["samples"].values():
                    self.assertEqual(tuple(sample), capture.PARAMETERS)
                    self.assertTrue(all(reply["request_monotonic_ns"] < reply["monotonic_ns"]
                                        for reply in sample.values()))
        self.assertFalse(result["human_stance_verified"] or result["stationarity_verified"] or
                         result["stop_confirmed"] or result["stop_sent"] or result["motor_enable_sent"])

    def test_timeout_poisoning_stops_without_retry_and_retains_partial_evidence(self):
        result, calls, cans = self.run_capture(fail=(4, "velocity"))
        self.assertEqual(result["status"], "ABORTED_READONLY_CAPTURE")
        self.assertEqual(sum((mid, parameter) == (4, "velocity") for mid, parameter, _ in calls), 1)
        self.assertTrue(cans["front"].poisoned)
        self.assertTrue(any("Injected read timeout" in error for error in result["errors"]))
        self.assertTrue(result["buses"]["front"]["events"])
        self.assertFalse(result["buses"]["front"]["sweeps"][-1]["complete"])
        self.assertTrue(all(parameter in (None, *capture.PARAMETERS) for _, parameter, _ in calls))

    def test_uid_failure_prevents_all_parameter_reads(self):
        result, calls, _ = self.run_capture(wrong_uid=True)
        self.assertEqual(result["status"], "ABORTED_READONLY_CAPTURE")
        self.assertTrue(all(parameter is None for _, parameter, _ in calls))
        self.assertTrue(any("UID mismatch" in error for error in result["errors"]))

    def test_final_wait_can_cross_target_during_guard_without_false_timeout(self):
        clock, calls, owners = Clock(), [], {}
        crossed = set()
        def factory(bus, sink):
            can = FakeCAN(bus, sink, clock, calls)
            owners[threading.get_ident()] = can
            return can
        def check():
            can = owners.get(threading.get_ident())
            if can is not None and can.tx_count == 1506 and clock() >= 10_990_000_000:
                # Six 1us UID queries put the common sampling epoch at this
                # offset; the final guard crosses its ten-second target.
                clock.local.now = 11_000_006_000
                crossed.add(threading.get_ident())
        result = capture.capture(factory, EXPECTED, check=check, clock=clock, wait=clock.wait)
        self.assertEqual(result["status"], "COMPLETE_STATIONARY_READONLY_CAPTURE", result["errors"])
        self.assertEqual(len(crossed), 2)

    def test_final_parser_residual_aborts_with_evidence(self):
        clock, calls, owners = Clock(), [], {}
        def factory(bus, sink):
            can = FakeCAN(bus, sink, clock, calls)
            owners[threading.get_ident()] = can
            return can
        def check():
            can = owners.get(threading.get_ident())
            if can is not None and can.tx_count == 1506 and clock() >= 10_990_000_000:
                can.parser.buffer = b"AT"
        result = capture.capture(factory, EXPECTED, check=check, clock=clock, wait=clock.wait)
        self.assertEqual(result["status"], "ABORTED_READONLY_CAPTURE")
        self.assertTrue(any("Final CAN parser" in error for error in result["errors"]))
        self.assertTrue(any(data["parser_residual_hex"] == "4154" for data in result["buses"].values()))

    def test_overrun_never_catches_up_and_stops_at_finite_bound(self):
        result, _, _ = self.run_capture(delay_ns=8_000_000)
        self.assertEqual(result["status"], "ABORTED_READONLY_CAPTURE")
        for data in result["buses"].values():
            rows = data["sweeps"]
            self.assertLess(len(rows), 50)
            self.assertTrue(all(b["begin_ns"]-a["begin_ns"] >= 200_000_000
                                for a, b in zip(rows, rows[1:])))
        self.assertTrue(any("budget" in error or "deadline" in error for error in result["errors"]))

    def test_canonical_guard_rejects_stop_enable_and_cross_bus(self):
        for mid, parameter, wire in ((1, "identity", bytes(17)), (1, "STOP", bytes(17)),
                                     (1, "enable", bytes(17)), (7, "position", capture.codec.read_request(7, "position"))):
            with self.assertRaises(RuntimeError):
                capture.guard_event("front", {"kind": "can_tx", "motor_id": mid,
                    "parameter": parameter, "hex": wire.hex()})


if __name__ == "__main__":
    unittest.main()
