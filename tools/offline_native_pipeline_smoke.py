#!/usr/bin/env python3
"""CPU-only 20-cycle integration using two AF_UNIX socketpairs and saved values.

No /dev file, network connection, serial-port discovery, SSH, or physical I/O.
Real C++ transport and real stateful inference run against simulated CAN peers.
All new timestamps refer to this local simulation, never to fresh robot data.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import threading
import time

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "runtime"), str(REPO / "runtime" / "experiments")]

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import native_diagnostic_transport as transport
from singularitydog_hw import native_pipeline_benchmark as pipeline
from singularitydog_hw import policy_observer as observer
from singularitydog_hw import policy_observer_replay as replay
from singularitydog_hw import policy_shadow as shadow
from native_policy_overnight import load_verified
from native_policy_overnight.contracts import strict_json
from native_policy_overnight.verification import compare_state, compare_tensor, saved_inputs

CYCLES = 20
SCOPES = {"front": tuple(range(1, 7)), "rear": tuple(range(7, 13))}


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_capture(path):
    """Keep original motor/IMU values; discard historical time only for simulation."""
    data = strict_json(Path(path).read_bytes())
    require(data.get("status") == "ONE_CAPTURE_POLICY_INFERENCE_NO_OUTPUT", "Incomplete saved capture")
    require(data.get("output_allowed") is False and data.get("learned_target_sent") is False,
            "Capture must be explicitly no-output")
    snapshot = data["observation"]["snapshot"]
    values = {}
    for row in snapshot["motors"]:
        key = row["motor_id"], row["parameter"]
        require(key not in values and type(key[0]) is int and key[0] in range(1, 13)
                and key[1] in ("position", "velocity"), "Invalid or duplicate saved axis")
        value = row["value"]
        require(type(value) in (float, int) and math.isfinite(value), "Nonfinite saved axis")
        require(row["unit"] == ("rad" if key[1] == "position" else "rad_s"), "Wrong saved SI unit")
        values[key] = float(value)
    require(set(values) == {(i, p) for i in range(1, 13) for p in ("position", "velocity")},
            "Missing saved q/dq")
    imu = snapshot["imu"]
    require(imu["frame"] == "raw_sensor", "Saved IMU must be sensor frame")
    sample = {}
    for name in ("accel_m_s2", "gyro_rad_s"):
        vector = imu[name]
        require(type(vector) is list and len(vector) == 3 and
                all(type(x) in (float, int) and math.isfinite(x) for x in vector), "Invalid saved IMU")
        sample[name] = list(vector)
    return values, sample


def quantize(value, limit):
    require(type(value) in (float, int) and math.isfinite(value) and -limit <= value <= limit,
            "Saved value outside Type2 emulation range; clipping prohibited")
    raw = round((value+limit)*65535./(2.*limit))
    require(0 <= raw <= 65535, "Quantization overflow")
    return raw, raw*(2.*limit)/65535.-limit


def response(wire, ids, uids, values):
    parser = codec.ATParser()
    frames = parser.feed(wire)
    require(len(frames) == 1 and not parser.buffer and not parser.discarded_bytes, "Malformed simulator request")
    frame = frames[0]
    mid = frame.destination
    require(mid in ids and frame.flags == 4, "Cross-bus/invalid simulator request")
    if frame.kind == 0:
        require(wire == codec.read_request(mid), "Noncanonical identity request")
        kind, host, data = 0, 0xfe, bytes.fromhex(uids[str(mid)])
        require(len(data) == 8, "Identity must contain eight bytes")
    elif frame.kind == 4:
        require(wire == transport.stop_wire(mid), "Only canonical all-zero STOP accepted")
        p, _ = quantize(values[mid, "position"], 12.57)
        v, _ = quantize(values[mid, "velocity"], 50.)
        kind, host, data = 2, 0xfd, struct.pack(">4H", p, v, 32768, 250)
    else:
        raise ValueError("Simulator rejects enable, learned target, setting, and active-report frames")
    can_id = kind << 24 | mid << 8 | host
    return b"AT"+((can_id << 3) | 4).to_bytes(4, "big")+b"\x08"+data+b"\r\n"


class SimulatedCAN:
    """One strictly ordered, finite peer; socket created locally by this tool."""
    def __init__(self, device, ids, uids, values):
        self.device, self.ids, self.uids, self.values = device, ids, uids, values
        self.requests, self.replies, self.errors = [], [], []
        self.received_bytes = 0
        self.parser = codec.ATParser()
        self.expected = [(0, i) for i in ids]+[(4, i) for _ in range(CYCLES*2) for i in ids]
        self.thread = threading.Thread(target=self.run, name="offline-simulated-can", daemon=True)

    def run(self):
        try:
            self.device.settimeout(.2)
            deadline = time.monotonic()+10.
            while time.monotonic() < deadline:
                try:
                    raw = self.device.recv(4096)
                except socket.timeout:
                    continue
                if not raw:
                    break
                self.received_bytes += len(raw)
                for frame in self.parser.feed(raw):
                    index = len(self.requests)
                    require(index < len(self.expected), "Extra simulator request")
                    require((frame.kind, frame.destination) == self.expected[index], "Simulator order differs")
                    self.requests.append(bytes(frame.wire))
                    wire = response(bytes(frame.wire), self.ids, self.uids, self.values)
                    self.device.sendall(wire)
                    self.replies.append(wire)
                require(not self.parser.discarded_bytes, "Simulator discarded bytes")
            else:
                raise TimeoutError("Finite simulator deadline exceeded")
            require(not self.parser.buffer, "Simulator partial request at EOF")
        except BaseException as error:
            self.errors.append(type(error).__name__+": "+str(error))
            try:
                self.device.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def report(self):
        kinds = Counter(codec.ATParser().feed(w)[0].kind for w in self.requests)
        return dict(simulated=True, request_count=len(self.requests), reply_count=len(self.replies),
                    request_types=dict(kinds), request_bytes=self.received_bytes,
                    reply_bytes=sum(len(w) for w in self.replies),
                    discarded_bytes=self.parser.discarded_bytes, residual_bytes=len(self.parser.buffer),
                    errors=list(self.errors), thread_joined=not self.thread.is_alive())


class SimulatedIMU:
    def __init__(self, sample):
        self.sample, self.calls = sample, 0

    def read_sample(self):
        started = time.monotonic_ns()
        self.calls += 1
        return dict(frame="sensor", simulated=True,
                    accel_m_s2=list(self.sample["accel_m_s2"]), gyro_rad_s=list(self.sample["gyro_rad_s"]),
                    read_started_monotonic_ns=started, read_finished_monotonic_ns=time.monotonic_ns())


def saved_parity(torch, policy, bundle, records):
    """Replay the exact quantized observer inputs, not different original floats."""
    eager, _ = shadow.load_policy(bundle)
    replay.warmup_policy(eager, torch, 0, 10)
    maxima = {}
    with torch.inference_mode():
        eager.reset(torch.tensor([0], dtype=torch.long))
        for row in records:
            observed = row["observed"]
            inputs, _ = saved_inputs(torch, json.dumps({"inputs": observed["inputs"]}, allow_nan=False).encode())
            output = eager(*inputs)
            target = torch.tensor([observed["q_target_rad_diagnostic_only"]], dtype=torch.float32)
            compare_tensor(torch, output, target, "target", maxima)
        compare_state(torch, eager, policy, maxima)
    return dict(status="PASS", frames=len(records), max_errors=maxima,
                input_source="Exact quantized inputs emitted by the simulated native observer",
                final_parameters_and_buffers_compared=True)


def run(args):
    out = Path(args.output).absolute()
    require(not out.exists() and out.parent.is_dir(), "Output must be new with an existing parent")
    require(not any((p / ".git").exists() for p in (out, *out.parents)), "Private records must be outside Git")
    out.mkdir(mode=0o700)
    report = dict(status="INCOMPLETE", simulation_only=True, source_values_are_saved=True,
                  all_new_timestamps_are_simulated=True, new_robot_samples=0, hardware_opened=False,
                  socket_transport="Two local AF_UNIX socketpairs, no network or /dev",
                  output_allowed=False, approved_for_runtime=False, full_controller_50Hz_verified=False,
                  jetson_latency_measurement=False, learned_targets_sent=False, motor_enable_sent=False,
                  timing_scope="Local emulator scheduling and CPU work only; not Jetson/CAN/USB latency",
                  startup_identity_scope="Synthetic replies from the supplied calibration, not fresh hardware identity",
                  warmup_calls=10, measured_real_model_calls=0, errors=[])
    records, startups, peers, hosts, devices = [], {}, {}, [], []
    cr = cw = None
    try:
        report["input_sha256"] = {name: digest(getattr(args, name)) for name in
                                  ("capture", "calibration", "mount", "library", "manifest")}
        report["source_sha256"] = {"smoke": digest(__file__),
            "collector": digest(pipeline.__file__), "transport_wrapper": digest(transport.__file__),
            "observer": digest(observer.__file__)}
        values, imu_sample = load_capture(args.capture)
        calibration = strict_json(Path(args.calibration).read_bytes())
        shadow.validate_calibration(calibration)
        uids = calibration["identities"]
        require(set(uids) == {str(i) for i in range(1, 13)} and
                len(set(uids.values())) == 12, "Require twelve distinct calibration identities")
        mount = strict_json(Path(args.mount).read_bytes())
        report["quantization_only_in_simulated_type2"] = [
            dict(motor_id=i, parameter=p, original=values[i, p],
                 decoded=quantize(values[i, p], limit)[1],
                 absolute_error=abs(quantize(values[i, p], limit)[1]-values[i, p]))
            for i in range(1, 13) for p, limit in (("position", 12.57), ("velocity", 50.))]
        import torch
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        policy, provenance = load_verified(args.manifest,
            expected_manifest_sha256=args.manifest_sha256, bundle=args.bundle)
        report["policy"] = provenance
        replay.warmup_policy(policy, torch, 0, 10)
        run = observer.StatefulPolicyObserver(policy, calibration, imu_mount_candidate=mount,
            h_hypothesis=0, command=[0., 0., 0.], max_ticks=CYCLES, max_age_ns=pipeline.LIMIT_NS,
            max_spread_ns=pipeline.LIMIT_NS, torch_module=torch, profile_consume=True,
            measured_diagnostic_ticks=True)
        run.prepare_run(warmup_completed=True)
        lib = transport.load_library(args.library)
        cr, cw = os.pipe()
        sessions = {}
        for scope, ids in SCOPES.items():
            host, device = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
            hosts.append(host); devices.append(device)
            host.setblocking(False)
            peers[scope] = SimulatedCAN(device, ids, uids, values)
            peers[scope].thread.start()
            sessions[scope] = transport.NativeSession(lib, host.fileno(), first_id=ids[0],
                                                      cancel_fd=cr, stop_proxy=True)
            startups[scope] = sessions[scope].exchange([codec.read_request(i) for i in ids])
            events = transport.records_as_events(startups[scope][0], cycle=0)
            require(all(row["result"]["mcu_uid_hex"] == uids[str(row["motor_id"])] for row in events),
                    "Simulated startup identity mismatch")
        imu_device = SimulatedIMU(imu_sample)
        result, records = pipeline.collect(sessions, imu_device, run, mode="stop-proxy", cycles=CYCLES)
        report["pipeline"] = result
        report["simulated_imu_reads"] = imu_device.calls
        report["measured_real_model_calls"] = result["observer"]["ticks_completed"]
        require(result["status"] == "COMPLETE_DIAGNOSTIC" and result["cycles_completed"] == CYCLES and
                result["observer"]["ticks_completed"] == CYCLES and imu_device.calls == CYCLES,
                "Integration did not finish twenty real policy calls: " + str(result.get("errors")))
        require(all(len(row[phase]) == 2 and all(len(batch[0]) == 6 for batch in row[phase].values())
                    for row in records for phase in ("acquired", "output")), "Unexpected frame count")
        require(all(bytes(rec.tx) == transport.stop_wire(codec.ATParser().feed(bytes(rec.tx))[0].destination)
                    for row in records for phase in ("acquired", "output")
                    for batch in row[phase].values() for rec in batch[0]), "Non-STOP transmit detected")
        report["eager_parity"] = saved_parity(torch, policy, args.bundle, records)
        report["status"] = "COMPLETE_OFFLINE_SIMULATED_NATIVE_PIPELINE"
    except BaseException as error:
        report["errors"].append(type(error).__name__+": "+str(error))
    finally:
        for host in hosts:
            try:
                host.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            host.close()
        for peer in peers.values():
            peer.thread.join(timeout=1.)
        for device in devices:
            device.close()
        for fd in (cr, cw):
            if fd is not None:
                os.close(fd)
        report["peers"] = {scope: peer.report() for scope, peer in peers.items()}
        report["all_local_descriptors_closed"] = all(sock.fileno() == -1 for sock in hosts+devices)
        if any(peer.thread.is_alive() or peer.errors for peer in peers.values()):
            report["status"] = "INCOMPLETE"
            report["errors"].append("Simulator worker failed or did not exit")
        if report["status"] == "COMPLETE_OFFLINE_SIMULATED_NATIVE_PIPELINE":
            if any(len(peer.requests) != 246 or len(peer.replies) != 246 for peer in peers.values()):
                report["status"] = "INCOMPLETE"
                report["errors"].append("Expected total 492 transmitted and 492 received frames")
        report["total_tx_frames"] = sum(len(p.requests) for p in peers.values())
        report["total_rx_frames"] = sum(len(p.replies) for p in peers.values())
        evidence = dict(simulation_only=True, all_new_timestamps_are_simulated=True,
            startup={s: transport.exchange_evidence(*capture) for s, capture in startups.items()},
            cycles=pipeline._serialize(records))
        for name, value in (("report.json", report), ("records.json", evidence)):
            fd = os.open(out / name, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "w") as handle:
                json.dump(value, handle, indent=2, allow_nan=False)
                handle.write("\n")
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "calibration", "mount", "bundle", "library", "manifest", "output"):
        p.add_argument("--"+name, type=Path, required=True)
    p.add_argument("--manifest-sha256", required=True)
    args = p.parse_args(argv)
    report = run(args)
    print(json.dumps({key: report.get(key) for key in ("status", "errors", "measured_real_model_calls",
        "total_tx_frames", "total_rx_frames", "hardware_opened", "jetson_latency_measurement")}))
    return 0 if report["status"] == "COMPLETE_OFFLINE_SIMULATED_NATIVE_PIPELINE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
