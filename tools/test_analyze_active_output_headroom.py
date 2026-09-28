"""Synthetic saved-frame checks; this test never opens a device."""

import unittest
import struct

from analyze_active_output_headroom import summarize


def stop_frame(mid, *, kind=4):
    can_id = (kind << 24) | (0xFD << 8) | mid
    wire = b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + bytes(8) + b"\r\n"
    reply_id = (2 << 24) | (mid << 8) | 0xFD
    reply = b"AT" + ((reply_id << 3) | 4).to_bytes(4, "big") + b"\x08" + bytes(8) + b"\r\n"
    return {"tx_hex": wire.hex(), "rx_hex": reply.hex(), "written": 17, "received": 17,
            "start_ns": 1, "finish_ns": 2, "received_ns": 3, "deadline_ns": 4}


def voltage_frame(mid):
    can_id = (17 << 24) | (0xFD << 8) | mid
    wire = b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08\x1c\x70" + bytes(6) + b"\r\n"
    reply_id = (17 << 24) | (mid << 8) | 0xFD
    reply = (b"AT" + ((reply_id << 3) | 4).to_bytes(4, "big") + b"\x08\x1c\x70\x00\x00" +
             struct.pack("<f", 40.) + b"\r\n")
    return {"tx_hex": wire.hex(), "rx_hex": reply.hex(), "written": 17, "received": 17,
            "start_ns": 1, "finish_ns": 2, "received_ns": 3, "deadline_ns": 4}


def fixture():
    cycle = {"cycle": 1}
    for phase in ("acquired", "output"):
        cycle[phase] = {"front": {"records": [stop_frame(i) for i in range(1, 7)]},
                        "rear": {"records": [stop_frame(i) for i in range(7, 13)]}}
    report = {"status": "COMPLETE_DIAGNOSTIC", "mode": "stop-proxy",
              "motor_enable_sent": False, "learned_targets_sent": False,
              "cycles_completed": 1, "plan": {"window": 3, "request_gap_us": 800},
              "measurements": [{"oldest_input_to_final_host_write_ms": 15.,
                                "oldest_input_to_last_reply_ms": 18.,
                                "whole_iteration_ms": 19.,
                                "final_host_write_ns": 2, "last_proxy_reply_ns": 3}],
              "host_deadline_misses": 0, "iteration_deadline_misses": 0}
    return report, [cycle]


class ActiveOutputHeadroomTest(unittest.TestCase):
    def test_v3_voltage_proxy_verifies_26_frames_and_rotating_voltage(self):
        report, records = fixture()
        report['v3_voltage_proxy'] = True
        report['plan'].update(v3_voltage_proxy=True, requests_per_cycle=26)
        for bus,mid in (('front',1),('rear',7)):
            records[0]['acquired'][bus]['records'].append(voltage_frame(mid))
        result=summarize(report,records,profile='v3')
        self.assertEqual(result['requests_per_cycle'],{'stop_proxy':26,'active':26})
        self.assertEqual(result['additional_input_requests_per_bus'],0)
        self.assertFalse(result['active_type1_reply_timing_measured'])
        records[0]['acquired']['front']['records'][-1]=voltage_frame(2)
        with self.assertRaisesRegex(ValueError,'rotating voltage'):
            summarize(report,records,profile='v3')

    def test_v3_voltage_proxy_rejects_missing_voltage_and_wrong_profile(self):
        report,records=fixture()
        report['v3_voltage_proxy']=True
        report['plan'].update(v3_voltage_proxy=True,requests_per_cycle=26)
        with self.assertRaisesRegex(ValueError,'frame count'):
            summarize(report,records,profile='v3')
        with self.assertRaisesRegex(ValueError,'explicit 26-request'):
            summarize(report,records,profile='v2')

    def test_request_delta_is_only_a_headroom_comparison(self):
        report, records = fixture()
        v2 = summarize(report, records, profile="v2")
        v3 = summarize(report, records, profile="v3")
        self.assertEqual((v2["requests_per_cycle"], v2["extra_spacing_if_active_uses_same_gap_ms"]),
                         ({"stop_proxy": 24, "active": 28}, 1.6))
        self.assertEqual(v3["requests_per_cycle"]["active"], 26)
        self.assertEqual(v3["extra_spacing_if_active_uses_same_gap_ms"], 0.8)
        self.assertEqual(v2["measured_stop_proxy_whole_iteration_headroom_ms"]["cycles_below_same_gap_extra_spacing"], 1)
        self.assertFalse(v2["active_full_cycle_20ms_verified"])

    def test_active_or_incomplete_frame_cannot_pose_as_proxy(self):
        report, records = fixture()
        records[0]["output"]["front"]["records"][0] = stop_frame(1, kind=1)
        with self.assertRaisesRegex(ValueError, "Type4"):
            summarize(report, records)
        records[0]["output"]["front"]["records"][0] = stop_frame(1)
        records[0]["output"]["front"]["records"][0]["received"] = 0
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            summarize(report, records)

    def test_mismatched_cycle_count_rejected(self):
        report, records = fixture()
        report["cycles_completed"] = 2
        with self.assertRaisesRegex(ValueError, "cycle count"):
            summarize(report, records)

    def test_mode_two_reply_cannot_pose_as_stopped_feedback(self):
        report, records = fixture()
        frame = records[0]["output"]["rear"]["records"][0]
        reply = bytearray(bytes.fromhex(frame["rx_hex"]))
        reply_id = (2 << 24) | (2 << 22) | (7 << 8) | 0xFD
        reply[2:6] = ((reply_id << 3) | 4).to_bytes(4, "big")
        frame["rx_hex"] = reply.hex()
        with self.assertRaisesRegex(ValueError, "mode-zero"):
            summarize(report, records)


if __name__ == "__main__":
    unittest.main()
