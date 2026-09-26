import copy
import unittest

from singularitydog_hw.role_group_summary import markdown, summarize


class RoleGroupSummaryTests(unittest.TestCase):
    def setUp(self):
        self.capture = {
            "status": "READ_ONLY_COMPLETE", "read_only": True,
            "motor_enable_sent": False, "errors": [], "boot_id": "boot",
            "motors": {str(i): {"uid_match": True, "position_last_rad": float(i),
                                "position_range_rad": .01 * i,
                                "current_max_abs_A": 0., "voltage_V": [40., 39.]}
                       for i in range(1, 13)},
        }

    def test_three_groups_cover_each_axis_once_and_keep_id4_in_foot(self):
        report = summarize(self.capture)
        self.assertEqual([[row["motor_id"] for row in g["motors"]]
                          for g in report["groups"]],
                         [[1, 4, 7, 10], [2, 5, 8, 11], [3, 6, 9, 12]])
        self.assertFalse(report["powered_hold_verified"])
        self.assertIn("|足先側|ID1|ID4|ID7|ID10|", markdown(report))

    def test_reject_incomplete_identity_and_nonfinite_capture(self):
        for change in (lambda x: x.update(status="INCOMPLETE"),
                       lambda x: x["motors"]["4"].update(uid_match=False),
                       lambda x: x["motors"]["9"].update(position_last_rad=float("nan")),
                       lambda x: x["motors"].pop("12")):
            capture = copy.deepcopy(self.capture)
            change(capture)
            with self.assertRaises(ValueError):
                summarize(capture)


if __name__ == "__main__":
    unittest.main()
