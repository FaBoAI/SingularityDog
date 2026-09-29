"""Bounded transport diagnostic; fake channels only, with no motor hardware."""

import unittest
import socket
import struct
import threading
import time

from singularitydog_hw import disabled_type1_stress as stress
from singularitydog_hw import watchdog_commissioning as watchdog
from singularitydog_hw import can_readonly as codec


class FakeChannel:
    def __init__(self, ids, *, fail_zero_at=None, stop_complete=True):
        self.ids = ids
        self.fail_zero_at = fail_zero_at
        self.stop_complete = stop_complete
        self.calls = []
        self.zero_count = 0
        self.stop_count = 0

    def exchange(self, mid, step, *, center=0.):
        self.calls.append((mid, step, center))
        if step == "identity":
            return {"mcu_uid_hex": (bytes([mid]) * 8).hex()}
        if step == "stop":
            return {"mode_state": 0, "fault_bits": 0, "protocol_position_rad": 0.}
        if step == "run_mode":
            return {"value": 0}
        if step == "voltage":
            return {"value": 40.}
        if step == "zero":
            self.zero_count += 1
            if self.zero_count == self.fail_zero_at:
                raise TimeoutError("simulated complete Type1 loss")
            return {"mode_state": 0, "fault_bits": 0,
                    "protocol_position_rad": center, "velocity_rad_s": 0.,
                    "temperature_c": 25.}
        raise AssertionError(f"Unexpected command: {step}")

    def stop_all(self):
        self.stop_count += 1
        return {"complete": self.stop_complete,
                "unconfirmed_ids": [] if self.stop_complete else [self.ids[0]],
                "errors": []}


class DisabledType1StressTests(unittest.TestCase):
    def make_case(self, **front_kwargs):
        channels = {scope: FakeChannel(ids, **(front_kwargs if scope == "front" else {}))
                    for scope, ids in watchdog.BUSES.items()}
        expected = {mid: (bytes([mid]) * 8).hex() for mid in watchdog.IDS}
        return channels, expected

    def test_plan_is_bounded_and_does_not_open_hardware(self):
        self.assertFalse(stress.plan(100)["hardware_opened"])
        for cycles in (0, 101, 1.5, True):
            with self.assertRaises(ValueError):
                stress.plan(cycles)
        self.assertEqual(stress.plan(50, dense=True)["type1_requests_per_cycle"], 24)
        self.assertEqual(stress.plan(50, dense=True)["voltage_requests_per_cycle"], 2)
        with self.assertRaises(ValueError):
            stress.plan(51, dense=True)

    def test_all_axes_receive_only_disabled_zero_gain_and_stop(self):
        channels, expected = self.make_case()
        result = stress.run(channels, expected, cycles=3)
        self.assertEqual(result["status"], "COMPLETE_DISABLED_TYPE1_DIAGNOSTIC", result)
        self.assertEqual(result["cycles_completed"], 3)
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(result["motor_enable_sent"])
        self.assertFalse(result["positive_gain_sent"])
        self.assertFalse(result["learned_targets_sent"])
        for scope, channel in channels.items():
            self.assertEqual(channel.zero_count, len(watchdog.BUSES[scope]) * 4)
            self.assertEqual(channel.stop_count, 1)
            self.assertEqual({step for _, step, _ in channel.calls},
                             {"identity", "stop", "run_mode", "voltage", "zero"})

    def test_missing_type1_aborts_and_stops_both_buses(self):
        channels, expected = self.make_case(fail_zero_at=8)
        result = stress.run(channels, expected, cycles=3)
        self.assertEqual(result["status"], "ABORTED")
        self.assertEqual(result["cycles_completed"], 0)
        self.assertTrue(result["stop_confirmed"])
        self.assertIn("Type1 loss", result["errors"][0])
        self.assertEqual([channel.stop_count for channel in channels.values()], [1, 1])

    def test_ambiguous_stop_requires_physical_cutoff(self):
        channels, expected = self.make_case(stop_complete=False)
        result = stress.run(channels, expected, cycles=1)
        self.assertEqual(result["status"], "STOP_UNCONFIRMED_POWER_OFF_REQUIRED")
        self.assertFalse(result["stop_confirmed"])

    def test_dense_two_bus_socket_exchange_uses_only_disabled_zero_type1(self):
        class Port:
            timeout = 0
            write_timeout = .02
            def __init__(self, sock): self.sock = sock
            def fileno(self): return self.sock.fileno()
            def write(self, data): return self.sock.send(data)
            @property
            def in_waiting(self):
                try: return len(self.sock.recv(4096, socket.MSG_PEEK))
                except BlockingIOError: return 0

        def wire(can_id, data):
            return b"AT" + ((can_id << 3) | 4).to_bytes(4, "big") + b"\x08" + data + b"\r\n"

        peers, hosts, threads, seen, errors = [], [], [], [], []
        done = threading.Event()
        for scope, ids in watchdog.BUSES.items():
            host, peer = socket.socketpair()
            host.setblocking(False)
            peer.settimeout(.1)
            hosts.append(host); peers.append(peer)
            def device(peer=peer, ids=ids):
                parser = codec.ATParser()
                try:
                    while not done.is_set():
                        try: data = peer.recv(4096)
                        except socket.timeout: continue
                        if not data: break
                        for frame in parser.feed(data):
                            mid = frame.destination
                            if mid not in ids: raise AssertionError("cross-bus request")
                            seen.append((mid, frame.kind, frame.data))
                            if frame.kind == 0:
                                reply = wire((mid << 8) | 0xfe, bytes([mid]) * 8)
                            elif frame.kind == 17:
                                index = int.from_bytes(frame.data[:2], "little")
                                payload = {0x7005: bytes(4), 0x701c: struct.pack("<f", 40.)}[index]
                                reply = wire((17 << 24) | (mid << 8) | 0xfd, frame.data[:4] + payload)
                            elif frame.kind in (1, 4):
                                if frame.kind == 1:
                                    _, velocity, kp, kd = struct.unpack(">4H", frame.data)
                                    if (velocity, kp, kd) != (32767, 0, 0):
                                        raise AssertionError("nonzero gain or velocity")
                                reply = wire((2 << 24) | (mid << 8) | 0xfd,
                                             struct.pack(">4H", 32767, 32767, 32767, 250))
                            else:
                                raise AssertionError("forbidden motor command")
                            peer.sendall(reply)
                except BaseException as error:
                    if not done.is_set(): errors.append(error)
            thread = threading.Thread(target=device, daemon=True)
            thread.start(); threads.append(thread)
        self.addCleanup(lambda: [sock.close() for sock in peers + hosts])
        def finish():
            done.set()
            for sock in hosts: sock.close()
            for thread in threads: thread.join(timeout=.5)
            self.assertFalse(errors, errors)
        self.addCleanup(finish)
        channels = {scope: watchdog.Channel(Port(hosts[index]), ids)
                    for index, (scope, ids) in enumerate(watchdog.BUSES.items())}
        expected = {mid: (bytes([mid]) * 8).hex() for mid in watchdog.IDS}
        result = stress.run(channels, expected, cycles=2, dense=True)
        self.assertEqual(result["status"], "COMPLETE_DISABLED_TYPE1_DIAGNOSTIC", result)
        self.assertEqual(result["cycles_completed"], 2)
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(any(kind == 3 for _, kind, _ in seen))
        self.assertEqual(sum(kind == 1 for _, kind, _ in seen), 12 + 2 * 24)


if __name__ == "__main__":
    unittest.main()
