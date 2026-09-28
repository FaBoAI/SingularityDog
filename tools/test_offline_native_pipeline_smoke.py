"""Emulator contract tests; no private fixtures, device files, or network."""
import json
from pathlib import Path
import socket
import tempfile
import unittest

import offline_native_pipeline_smoke as smoke


class SmokeContracts(unittest.TestCase):
    def setUp(self):
        self.uids = {str(i): (bytes([i])*8).hex() for i in range(1, 13)}
        self.values = {(i, p): .2 if p == "position" else -.1
                       for i in range(1, 13) for p in ("position", "velocity")}

    def wire(self, kind):
        can_id = kind << 24 | 0xfd << 8 | 1
        return b"AT"+((can_id << 3) | 4).to_bytes(4, "big")+b"\x08"+bytes(8)+b"\r\n"

    def test_emulator_rejects_all_motion_and_configuration_frames(self):
        for kind in (1, 3, 6, 17, 18, 24):
            with self.assertRaisesRegex(ValueError, "Simulator rejects"):
                smoke.response(self.wire(kind), tuple(range(1, 7)), self.uids, self.values)

    def test_identity_echo_binds_supplied_calibration_uid(self):
        reply = smoke.response(smoke.codec.read_request(1), tuple(range(1, 7)), self.uids, self.values)
        frame = smoke.codec.ATParser().feed(reply)[0]
        decoded = smoke.codec.decode_reply(frame, 1, None)
        self.assertTrue(decoded["ok"])
        self.assertEqual(decoded["mcu_uid_hex"], self.uids["1"])

    def test_nonzero_stop_and_wrong_bus_rejected(self):
        altered = bytearray(smoke.transport.stop_wire(1))
        altered[7] = 1
        with self.assertRaisesRegex(ValueError, "all-zero STOP"):
            smoke.response(bytes(altered), tuple(range(1, 7)), self.uids, self.values)
        with self.assertRaisesRegex(ValueError, "Cross-bus"):
            smoke.response(smoke.codec.read_request(7), tuple(range(1, 7)), self.uids, self.values)

    def test_quantization_is_bounded_and_never_clips(self):
        for limit in (12.57, 50.):
            for value in (-limit, -.1, 0., .1, limit):
                raw, decoded = smoke.quantize(value, limit)
                self.assertLessEqual(abs(value-decoded), limit/65535.+1e-12)
                self.assertIn(raw, range(65536))
            for value in (-limit-.00001, limit+.00001, float("nan"), float("inf"), True):
                with self.assertRaises(ValueError):
                    smoke.quantize(value, limit)

    def capture(self):
        return dict(status="ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT", output_allowed=False,
                    learned_target_sent=False, observation=dict(snapshot=dict(
                        motors=[dict(motor_id=i, parameter=p, value=v,
                                     unit="rad" if p == "position" else "rad_s")
                                for (i, p), v in self.values.items()],
                        imu=dict(frame="raw_sensor", accel_m_s2=[0., 0., -9.8], gyro_rad_s=[0., 0., 0.]))))

    def test_missing_or_duplicate_saved_axis_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            for duplicate in (False, True):
                data = self.capture()
                motors = data["observation"]["snapshot"]["motors"]
                if duplicate:
                    motors.append(dict(motors[0]))
                else:
                    motors.pop()
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    smoke.load_capture(path)

    def test_saved_values_preserved_and_only_new_imu_times_simulated(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            path.write_text(json.dumps(self.capture()))
            values, sample = smoke.load_capture(path)
            self.assertEqual(values, self.values)
            device = smoke.SimulatedIMU(sample)
            first, second = device.read_sample(), device.read_sample()
            self.assertTrue(first["simulated"])
            self.assertGreater(second["read_started_monotonic_ns"], first["read_started_monotonic_ns"])
            self.assertEqual(first["accel_m_s2"], sample["accel_m_s2"])
            first["accel_m_s2"][0] = 99.
            self.assertEqual(sample["accel_m_s2"][0], 0.)

    def test_socket_peer_rejects_wrong_sequence_and_exits(self):
        host, device = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        peer = smoke.SimulatedCAN(device, tuple(range(1, 7)), self.uids, self.values)
        try:
            peer.thread.start()
            host.sendall(smoke.codec.read_request(2))  # ID1 must be first.
            peer.thread.join(timeout=1.)
            self.assertFalse(peer.thread.is_alive())
            self.assertTrue(peer.errors)
            self.assertEqual(peer.replies, [])
        finally:
            host.close(); device.close()


if __name__ == "__main__":
    unittest.main()
