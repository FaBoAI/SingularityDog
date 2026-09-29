"""Offline parity and CPU benchmark for a non-integrated batched Type1 encoder.

No transport/session/device API is called. With no --library, measures only the
current Python encode/parse/check path. With --library, compares candidate bytes
and rejection reasons, then measures the ctypes wrapper including 12 bytes
objects per call (the form required by current bus workers).
"""

import argparse
import ctypes as C
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import statistics
import time
from types import SimpleNamespace

from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.native_active_transport import encode_motion


class AxisSpec(C.Structure):
    _fields_ = [(name, C.c_double) for name in
                ('offset', 'sign', 'lower', 'upper', 'initial',
                 'max_displacement', 'max_estimated_pd')]


MESSAGES = {
    1: 'Invalid batch encoder argument',
    2: 'q must be finite numeric',
    3: 'kp must be finite numeric',
    4: 'kd must be finite numeric',
    5: 'Target/gain outside active software caps',
    6: 'quantized target outside physical range',
    7: 'quantized target outside trial displacement',
    8: 'quantized estimated PD torque',
}


def baseline(command, offsets, axes, initial):
    """Exact copy of the calculation/check sequence in runtime.wires_for."""
    raws = {mid: (command.q_model_rad[mid - 1] - offsets[mid]) / axes[mid]['sign']
            for mid in range(1, 13)}
    wires = {bus: [encode_motion(mid, raws[mid], command.kp[mid - 1],
                                 command.kd[mid - 1]) for mid in ids]
             for bus, ids in (('front', range(1, 7)), ('rear', range(7, 13)))}
    for bus_wires in wires.values():
        for wire in bus_wires:
            frame = ATParser().feed(wire)[0]
            mid = frame.destination
            axis = axes[mid]
            raw = int.from_bytes(frame.data[:2], 'big') * 25.14 / 65535 - 12.57
            q = axis['sign'] * raw + offsets[mid]
            if not axis['lower_rad'] <= q <= axis['upper_rad']:
                raise RuntimeError(f'ID{mid} quantized target outside physical range')
            if not abs(q - initial[mid - 1]) <= axis['max_displacement_from_start_rad']:
                raise RuntimeError(f'ID{mid} quantized target outside trial displacement')
            estimated = (command.kp[mid - 1] * (q - command.q_model_rad[mid - 1]) +
                         command.estimated_pd_torque_nm[mid - 1])
            if not abs(estimated) <= axis['max_estimated_pd_torque_nm']:
                raise RuntimeError(f'ID{mid} quantized estimated PD torque')
    return wires


class NativeCandidate:
    def __init__(self, library, offsets, axes, initial):
        lib = C.CDLL(str(Path(library).resolve(strict=True)))
        lib.sdbe_encode.argtypes = [C.POINTER(AxisSpec)] + [C.POINTER(C.c_double)] * 4 + [
            C.POINTER(C.c_uint8), C.POINTER(C.c_int32)]
        lib.sdbe_encode.restype = C.c_int32
        self.encode = lib.sdbe_encode
        self.specs = (AxisSpec * 12)(*(AxisSpec(
            offsets[mid], axes[mid]['sign'], axes[mid]['lower_rad'],
            axes[mid]['upper_rad'], initial[mid - 1],
            axes[mid]['max_displacement_from_start_rad'],
            axes[mid]['max_estimated_pd_torque_nm']) for mid in range(1, 13)))
        self.arrays = tuple((C.c_double * 12)() for _ in range(4))
        self.output = (C.c_uint8 * 204)()
        self.first_id = C.c_int32()

    def __call__(self, command):
        for target, values in zip(self.arrays, (
                command.q_model_rad, command.kp, command.kd,
                command.estimated_pd_torque_nm)):
            for k in range(12):
                target[k] = values[k]
        status = self.encode(self.specs, *self.arrays, self.output,
                             C.byref(self.first_id))
        if status:
            message = MESSAGES.get(status, 'Unknown batch encoder status')
            if status in (2, 3, 4, 5):
                raise ValueError(message)
            raise RuntimeError(f'ID{self.first_id.value} {message}')
        wires = [bytes(self.output[k * 17:(k + 1) * 17]) for k in range(12)]
        return {'front': wires[:6], 'rear': wires[6:]}


def fixture():
    axes = {}
    offsets = {}
    initial = []
    for mid in range(1, 13):
        q = .025 * mid
        sign = -1 if mid % 3 == 0 else 1
        offset = q - sign * q
        offsets[mid] = offset
        initial.append(q)
        axes[mid] = dict(sign=sign, lower_rad=q - .08, upper_rad=q + .08,
                         max_displacement_from_start_rad=.04,
                         max_estimated_pd_torque_nm=.3)
    command = SimpleNamespace(q_model_rad=tuple(initial),
                              kp=(3.,) * 12, kd=(.15,) * 12,
                              estimated_pd_torque_nm=(0.,) * 12)
    return command, offsets, axes, tuple(initial)


def median_us(fn, *, count=10_000, repeats=7):
    for _ in range(100):
        fn()
    rows = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        for _ in range(count):
            fn()
        rows.append((time.perf_counter_ns() - start) / count / 1000)
    return dict(median_us=statistics.median(rows), minimum_us=min(rows),
                maximum_us=max(rows))


def parity(candidate, offsets, axes, initial, *, cases=2000):
    rng = random.Random(92029)
    passed = 0
    for index in range(cases):
        q = tuple(initial[k] + rng.uniform(-.015, .015) for k in range(12))
        kp = tuple(rng.uniform(0., 10.) for _ in range(12))
        kd = tuple(rng.uniform(0., .5) for _ in range(12))
        estimated = tuple(rng.uniform(-.01, .01) for _ in range(12))
        command = SimpleNamespace(q_model_rad=q, kp=kp, kd=kd,
                                  estimated_pd_torque_nm=estimated)
        wanted = baseline(command, offsets, axes, initial)
        actual = candidate(command)
        if wanted != actual:
            raise AssertionError(f'Byte mismatch at seeded case {index}')
        passed += 1
    return passed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path)
    parser.add_argument('--extension', type=Path)
    parser.add_argument('--parity-cases', type=int, default=2000)
    args = parser.parse_args(argv)
    command, offsets, axes, initial = fixture()
    result = dict(schema='singularitydog.offline-native-batch-encode-benchmark.v1',
                  hardware_opened=False, motor_commands_sent=False,
                  baseline=median_us(lambda: baseline(command, offsets, axes, initial)))
    if args.library:
        candidate = NativeCandidate(args.library, offsets, axes, initial)
        result['parity_cases'] = parity(candidate, offsets, axes, initial,
                                        cases=args.parity_cases)
        result['candidate'] = median_us(lambda: candidate(command))
        result['source_sha256'] = hashlib.sha256(
            Path(__file__).with_name('batch_encode.cpp').read_bytes()).hexdigest() \
            if Path(__file__).with_name('batch_encode.cpp').is_file() else None
        result['binary_sha256'] = hashlib.sha256(args.library.read_bytes()).hexdigest()
    if args.extension:
        module_spec = importlib.util.spec_from_file_location('sdbe_native', args.extension)
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
        specs = tuple((offsets[mid], axes[mid]['sign'], axes[mid]['lower_rad'],
                       axes[mid]['upper_rad'], initial[mid - 1],
                       axes[mid]['max_displacement_from_start_rad'],
                       axes[mid]['max_estimated_pd_torque_nm']) for mid in range(1, 13))
        context = module.make_context(specs)
        direct = lambda command: module.encode(context, command)
        result['extension_parity_cases'] = parity(direct, offsets, axes, initial,
                                                   cases=args.parity_cases)
        result['extension_candidate'] = median_us(lambda: direct(command))
        result['extension_binary_sha256'] = hashlib.sha256(args.extension.read_bytes()).hexdigest()
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
