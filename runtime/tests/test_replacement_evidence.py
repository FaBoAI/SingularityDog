"""Replacement evidence validation is offline; snapshots are mocked."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw.replacement_evidence import identity_allowed, load_id11_replacement


def snapshot(label, replacement=False):
    motors = {str(i): {"mcu_uid_hex": f"{i:016x}",
                       "position": {"samples": 60 if replacement else 5,
                                    "peak_to_peak_rad": .001, "max_adjacent_step_rad": .001},
                       "max_abs_current_A": 0.} for i in range(1, 13)}
    if replacement:
        motors["11"]["mcu_uid_hex"] = "f00000000000000b"
    return {"motors": motors,
            "capture_intervals": {"wall_time_ns": {"start": 30 if replacement else 10,
                                                     "end": 40 if replacement else 20},
                                  "monotonic_ns": {"start": 1 if replacement else 100}},
            "sources": {"events": {"path": f"/captures/{label}/events.jsonl",
                                   "sha256": ("b" if replacement else "a") * 64}}}


class ReplacementEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "replacement.json"
        self.config = {"schema_version": 1, "motor_id": 11, "eligibility": "candidate_only",
                       "previous_capture": "/captures/previous", "previous_events_sha256": "a" * 64,
                       "replacement_capture": "/captures/replacement", "replacement_events_sha256": "b" * 64}
        self.old, self.new = snapshot("previous"), snapshot("replacement", True)

    def load(self, old=None, new=None):
        self.path.write_text(json.dumps(self.config))
        with patch("singularitydog_hw.replacement_evidence.load_snapshot",
                   side_effect=[self.old if old is None else old, self.new if new is None else new]):
            return load_id11_replacement(self.path)

    def test_valid_replacement_is_candidate_only_and_does_not_compare_monotonic_or_angles(self):
        policy = self.load()
        self.assertEqual(policy["retired_uid"], self.old["motors"]["11"]["mcu_uid_hex"])
        self.assertEqual(policy["replacement_uid"], self.new["motors"]["11"]["mcu_uid_hex"])
        self.assertEqual(policy["sources"]["replacement"]["events"]["sha256"], "b" * 64)
        self.assertEqual(policy["sources"]["config"]["path"], str(self.path.resolve()))
        self.assertFalse(policy["approved_for_runtime"])
        self.assertFalse(policy["motor_power_cycle_continuity_verified"])

    def test_incomplete_snapshot_exception_is_not_suppressed(self):
        self.path.write_text(json.dumps(self.config))
        with patch("singularitydog_hw.replacement_evidence.load_snapshot", side_effect=ValueError("incomplete")):
            with self.assertRaisesRegex(ValueError, "incomplete"):
                load_id11_replacement(self.path)

    def test_config_rejects_missing_wrong_id_and_unpinned_evidence(self):
        for field, value in (("motor_id", 10), ("schema_version", True),
                             ("eligibility", "runtime"), ("replacement_capture", "relative"),
                             ("replacement_events_sha256", "c" * 64),
                             ("previous_events_sha256", "invalid")):
            original = copy.deepcopy(self.config)
            with self.subTest(field=field):
                self.config[field] = value
                with self.assertRaises(ValueError):
                    self.load()
            self.config = original
        del self.config["previous_capture"]
        with self.assertRaises(ValueError):
            self.load()

    def test_rejects_same_uid_other_changed_uid_and_invalid_uid(self):
        for mid, uid in (("11", self.old["motors"]["11"]["mcu_uid_hex"]),
                         ("1", "e000000000000001"), ("11", "bad"),
                         ("11", self.old["motors"]["1"]["mcu_uid_hex"])):
            with self.subTest(mid=mid, uid=uid):
                changed = copy.deepcopy(self.new)
                changed["motors"][mid]["mcu_uid_hex"] = uid
                with self.assertRaises(ValueError):
                    self.load(new=changed)

    def test_rejects_insufficient_unstable_current_and_unordered_evidence(self):
        for field, value in (("samples", 59), ("samples", True), ("peak_to_peak_rad", .81),
                             ("max_adjacent_step_rad", .03), ("peak_to_peak_rad", float("nan"))):
            changed = copy.deepcopy(self.new)
            changed["motors"]["11"]["position"][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.load(new=changed)
        changed = copy.deepcopy(self.new)
        changed["motors"]["11"]["max_abs_current_A"] = .051
        with self.assertRaises(ValueError):
            self.load(new=changed)
        changed = copy.deepcopy(self.new)
        changed["capture_intervals"]["wall_time_ns"]["start"] = 20
        with self.assertRaises(ValueError):
            self.load(new=changed)

    def test_identity_gate_retains_default_and_rejects_mixed_or_retired_motor_anywhere(self):
        policy = self.load()
        old, new = policy["retired_uid"], policy["replacement_uid"]
        self.assertFalse(identity_allowed(11, [new], None))
        self.assertTrue(identity_allowed(1, ["0000000000000001"], None))
        self.assertTrue(identity_allowed(11, [new, new, new], policy))
        self.assertTrue(identity_allowed(1, ["0000000000000001"] * 3, policy))
        for mid, uids in ((11, []), (11, [old]), (11, [new, old]),
                          (11, ["0000000000000002"]), (1, [old]),
                          (1, ["0000000000000001", "0000000000000002"]), (11, ["bad"])):
            with self.subTest(mid=mid, uids=uids):
                self.assertFalse(identity_allowed(mid, uids, policy))
        self.assertFalse(identity_allowed(11, [new], {}))
        self.assertFalse(identity_allowed(True, [new], policy))


if __name__ == "__main__":
    unittest.main()
