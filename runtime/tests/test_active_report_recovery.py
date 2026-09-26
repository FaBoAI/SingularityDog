"""OFF-only recovery tests; all clocks, port writes and receives are simulated."""
import copy
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from singularitydog_hw import active_report_recovery as recovery
from singularitydog_hw.active_report_protocol import reporting_request
from singularitydog_hw.can_readonly import ATParser
from test_active_report_probe import Clock, FakePort, FakeReader, UIDS, MS


class RecoveryPort(FakePort):
    def write(self, data):
        count = super().write(data)
        frame, = ATParser().feed(data)
        if self.failure == "partial_off" and frame.kind == 24:
            return count - 1
        return count


class RecoveryTests(unittest.TestCase):
    def build(self, failure=None):
        clock = Clock()
        port = RecoveryPort(clock, period=0, failure=failure, report_kind=24,
                            activation_prefix=True)
        # Mimic failed six-ID cleanup: IDs1/2 are quiet;3..6 remain reporting.
        port.reporting = {mid: clock() + 10 * MS for mid in range(3, 7)}
        instance = recovery.RecoveryProbe(port, tuple(range(1, 7)), UIDS,
            clock=clock, reader_factory=FakeReader)
        return instance, port, clock

    def test_recovery_sends_only_paced_off_then_postquiet_identity_and_stop(self):
        instance, port, clock = self.build()
        result = instance.run()
        self.assertEqual(result["status"], "REPORTING_OFF_RECOVERY_COMPLETE")
        self.assertFalse(port.reporting)
        self.assertEqual(len(result["off_observations"]), 6)
        self.assertTrue(all(row["stop_ack"] is False for row in result["off_observations"]))
        self.assertEqual([row["action"] for row in instance.tx_log],
                         ["report_off"] * 6 + ["identity"] * 6 + ["stop"] * 6)
        off = instance.tx_log[:6]
        for previous, following in zip(off, off[1:]):
            self.assertGreaterEqual(following["started_ns"] - previous["finished_ns"], 20 * MS)
        self.assertGreaterEqual(instance.tx_log[6]["started_ns"], result["quiet_verified_ns"])
        self.assertGreaterEqual(result["quiet_verified_ns"] - off[-1]["finished_ns"], 100 * MS)
        self.assertTrue(result["identities_reverified"])
        self.assertEqual(len(result["stop_observations"]), 6)
        self.assertGreater(sum(result["periodic_counts"].values()), 0)
        self.assertTrue(instance.raw_log)
        self.assertLess(clock() - instance.started, recovery.MAX_RUN_NS)
        self.assertTrue(all(row["kind"] in (0, 4, 24) for row in port.writes))
        self.assertTrue(all(row.get("enabled", 0) == 0 for row in port.writes))

    def test_short_off_write_poisons_and_forbids_all_later_writes(self):
        instance, port, _ = self.build("partial_off")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertTrue(result["transport_poisoned"])
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(result["off_attempted"], [1])

    def test_missing_type2_still_offs_all_ids_but_cannot_query_or_succeed(self):
        instance, port, _ = self.build("missing_deactivation_prefix")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(result["off_attempted"], list(range(1, 7)))
        self.assertEqual([row["action"] for row in instance.tx_log], ["report_off"] * 6)
        self.assertFalse(port.reporting)
        self.assertFalse(result["off_observations"])

    def test_continuing_reports_prevent_uid_stop_and_success(self):
        instance, port, clock = self.build("off_ignored")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertTrue(port.reporting)
        self.assertEqual([row["action"] for row in instance.tx_log], ["report_off"] * 6)
        self.assertLess(clock() - instance.started, 3_000_000_000)

    def test_no_on_period_or_stop_before_verified_quiet(self):
        for action, value in (("report_on", None), ("period_set", 1),
                              ("period_read", None), ("identity", None), ("stop", None)):
            with self.subTest(action=action):
                instance, port, _ = self.build()
                with self.assertRaises(ValueError):
                    instance._send(1, action, value)
                self.assertFalse(port.writes)

    def test_uid_mismatch_after_quiet_prevents_stop(self):
        instance, port, _ = self.build("wrong_uid")
        result = instance.run()
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertFalse(port.reporting)
        self.assertNotIn("stop", [row["action"] for row in instance.tx_log])

    def evidence(self):
        summary = {"status": "INCOMPLETE", "boot_id": "boot", "common_locks_released": True,
            "plan": {"period_policy": "observe-current"}, "results": {"front": {
                "identities_verified": True, "port_closed": True, "transport_poisoned": False,
                "ids": list(range(1, 7))}}}
        tx = [{"motor_id": mid, "action": "report_on", "returned_bytes": 17,
               "wire_hex": reporting_request(mid, True).hex()} for mid in range(1, 7)]
        return summary, tx

    def test_evidence_requires_matching_boot_verified_closed_unpoisoned_scope(self):
        summary, tx = self.evidence()
        result = recovery.validate_evidence(summary, tx, scope="front", ids=(1, 2), boot_id="boot")
        self.assertFalse(result["previous_port_binding_recorded"])
        cases = [("boot_id", "other"), ("status", "COMPLETE"), ("common_locks_released", False)]
        for key, value in cases:
            bad = copy.deepcopy(summary); bad[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                recovery.validate_evidence(bad, tx, scope="front", ids=(1,), boot_id="boot")
        for key, value in (("identities_verified", False), ("port_closed", False),
                           ("transport_poisoned", True)):
            bad = copy.deepcopy(summary); bad["results"]["front"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                recovery.validate_evidence(bad, tx, scope="front", ids=(1,), boot_id="boot")

    def test_evidence_requires_prior_canonical_on_and_selected_bus_subset(self):
        summary, tx = self.evidence()
        for ids, rows in (((7,), tx), ((True,), tx), ((1, 1), tx), ((1,), tx[1:])):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                recovery.validate_evidence(summary, rows, scope="front", ids=ids, boot_id="boot")
        tx[0]["returned_bytes"] = 16
        with self.assertRaises(ValueError):
            recovery.validate_evidence(summary, tx, scope="front", ids=(1,), boot_id="boot")

    def test_cli_requires_evidence_but_dry_plan_never_resolves_or_opens_devices(self):
        summary, tx = self.evidence()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "summary.json").write_text(json.dumps(summary))
            (root / "front-tx.json").write_text(json.dumps(tx))
            (root / "uids.json").write_text(json.dumps(UIDS))
            args = ["--failed-summary", str(root / "summary.json"), "--scope", "front",
                    "--ids", "1", "2", "3", "4", "5", "6", "--expected-boot-id", "boot",
                    "--expected-uids", str(root / "uids.json"),
                    "--front-port", "/dev/serial/by-path/front", "--rear-port", "/dev/serial/by-path/rear"]
            output = io.StringIO()
            with patch.object(recovery.dual, "validate_ports") as validate, \
                    patch.object(recovery, "RecoveryProbe") as owner, \
                    patch.object(recovery, "ownership_locks") as locks, \
                    contextlib.redirect_stdout(output):
                self.assertEqual(recovery.main(args), 0)
            validate.assert_not_called(); owner.assert_not_called(); locks.assert_not_called()
            plan = json.loads(output.getvalue())
            self.assertEqual(plan["allowed_can_types"], [0, 4, 24])
            self.assertEqual(plan["off_spacing_ms"], 20)
            self.assertFalse(plan["previous_port_binding_recorded"])
            self.assertEqual(len(plan["failed_summary_sha256"]), 64)

    def test_cli_closes_before_unlocking_and_retains_both_locks_on_close_failure(self):
        for fail_close in (False, True):
            with self.subTest(fail_close=fail_close), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                summary, tx = self.evidence()
                (root / "summary.json").write_text(json.dumps(summary))
                (root / "front-tx.json").write_text(json.dumps(tx))
                (root / "uids.json").write_text(json.dumps(UIDS))
                events = []
                @contextlib.contextmanager
                def lock(name):
                    events.append(name + "_enter")
                    try:
                        yield
                    finally:
                        events.append(name + "_exit")
                class Raw:
                    is_open = False
                    def open(self):
                        events.append("open"); self.is_open = True
                    def close(self):
                        events.append("close")
                        if fail_close:
                            raise OSError("injected close failure")
                        self.is_open = False
                    def fileno(self):
                        return 99
                raw = Raw()
                bindings = {scope: {"path": "/dev/serial/by-path/" + scope,
                            "resolved": "/dev/" + scope, "st_rdev": number}
                            for scope, number in (("front", 1), ("rear", 2))}
                fake_owner = SimpleNamespace(raw_log=[], tx_log=[], run=lambda: {
                    "status": "REPORTING_OFF_RECOVERY_COMPLETE"})
                args = ["--failed-summary", str(root / "summary.json"), "--scope", "front",
                        "--ids", "1", "--expected-boot-id", "boot", "--expected-uids", str(root / "uids.json"),
                        "--front-port", bindings["front"]["path"], "--rear-port", bindings["rear"]["path"],
                        "--output", str(root / "output"), "--execute-no-motion"]
                retained_before = len(recovery._HELD_LOCKS)
                try:
                    with patch.object(recovery.dual, "validate_ports", return_value=bindings), \
                            patch.object(recovery.dual, "binding_matches", return_value=True), \
                            patch.object(recovery, "ownership_locks", side_effect=lambda: lock("common")), \
                            patch.object(recovery.dual, "port_lock", side_effect=lambda _: lock("port")), \
                            patch.object(recovery.Path, "read_text", return_value="boot"), \
                            patch.object(recovery.os, "fstat", return_value=SimpleNamespace(st_rdev=1)), \
                            patch.dict("sys.modules", {"serial": SimpleNamespace(Serial=lambda **_: raw)}), \
                            patch.object(recovery, "RecoveryProbe", return_value=fake_owner), \
                            contextlib.redirect_stdout(io.StringIO()):
                        self.assertEqual(recovery.main(args), 2 if fail_close else 0)
                    expected_events = ["common_enter", "port_enter", "open", "close"]
                    if fail_close:
                        self.assertEqual(len(recovery._HELD_LOCKS), retained_before + 1)
                    else:
                        expected_events += ["port_exit", "common_exit"]
                    self.assertEqual(events, expected_events)
                    saved = json.loads((root / "output" / "summary.json").read_bytes())
                    self.assertEqual(saved["port_closed"], not fail_close)
                    self.assertEqual(saved["locks_released"], not fail_close)
                    self.assertEqual((root / "output" / "summary.json").stat().st_mode & 0o777, 0o600)
                finally:
                    for held in recovery._HELD_LOCKS[retained_before:]:
                        held.close()
                    del recovery._HELD_LOCKS[retained_before:]


if __name__ == "__main__":
    unittest.main()
