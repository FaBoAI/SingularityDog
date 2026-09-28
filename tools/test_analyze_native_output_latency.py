"""File-only checks for output timing decomposition."""

import unittest

from analyze_native_output_latency import summarize


class OutputLatencyAnalysisTests(unittest.TestCase):
    def test_separates_send_gap_from_reply_wait(self):
        cycle = {"cycle": 1, "output": {}}
        for bus in ("front", "rear"):
            frames = []
            for index in range(6):
                start = 1_000_000 + index * 800_000
                finish = start + 10_000
                read_start = finish + (2_700_000 if index == 5 else 20_000)
                frames.append({"start_ns": start, "finish_ns": finish,
                               "read_start_ns": read_start,
                               "received_ns": read_start + 2_000,
                               "written": 17, "received": 17})
            cycle["output"][bus] = {"records": frames, "stats": {"reads": 6, "waits": 8}}
        result = summarize([cycle])
        self.assertEqual(result["cycles"], 1)
        for bus in ("front", "rear"):
            fields = result["buses"][bus]
            self.assertEqual(fields["between_write_gap_ms"]["median"], .79)
            self.assertEqual(fields["first_to_last_write_ms"]["median"], 4.)
            self.assertEqual(fields["last_write_to_read_start_ms"]["median"], 2.7)
            self.assertEqual(fields["read_start_to_received_ms"]["median"], .002)

    def test_rejects_missing_reply(self):
        with self.assertRaisesRegex(ValueError, "Missing output bus"):
            summarize([{"cycle": 1, "output": {}}])


if __name__ == "__main__":
    unittest.main()
