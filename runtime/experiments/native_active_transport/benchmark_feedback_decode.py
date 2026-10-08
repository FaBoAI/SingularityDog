"""File-only comparison of Python and six-record C++ feedback decoding.

Includes the ctypes boundary, Type2Feedback objects, and result dictionaries.
No transport session, device descriptor, socket, or model is opened.
"""
import argparse
import ctypes
import gc
import hashlib
import json
from pathlib import Path
import platform
import statistics
import struct
import time

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import rs05_trial_protocol as protocol


def fixtures():
    """Synthetic actual Type1 requests and their canonical Type2 responses."""
    cases = []
    for variant in range(3):
        buses = []
        for first in (1, 7):
            records = (native.Record * 6)()
            for index, record in enumerate(records):
                mid = first + index
                record.start_ns = 1_000_000 + index * 890_000
                record.finish_ns = record.start_ns + 100
                record.read_start_ns = record.finish_ns + 100
                record.received_ns = record.finish_ns + 200
                record.deadline_ns = 21_000_000
                record.written = record.received = 17
                record.tx[:] = native.encode_motion(mid, .01 * index, 3., .15)
                mode = variant
                fault = (index * 11 + variant) & 63
                payload = struct.pack('>4H', (index * 10921 + variant * 13) & 65535,
                                      (32767 + index * 73) & 65535,
                                      (32767 + variant * 93) & 65535,
                                      250 + index * 10)
                record.rx[:] = protocol._wire(
                    (2 << 24) | (mode << 22) | (fault << 16) | (mid << 8) | 0xfd,
                    payload)
            buses.append((first, records))
        cases.append(buses)
    return cases


def digest_records(cases):
    sha = hashlib.sha256()
    for buses in cases:
        for _, records in buses:
            sha.update(ctypes.string_at(ctypes.addressof(records), ctypes.sizeof(records)))
    return sha.hexdigest()


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]


def summary(values):
    return {'samples': len(values), 'median_us': statistics.median(values) / 1000,
            'p99_us': percentile(values, .99) / 1000, 'max_us': max(values) / 1000}


def run(library, *, samples, trials, warmup):
    decoder = native.NativeFeedbackBatchDecoder(library)
    if not decoder.available:
        raise ValueError('The selected library does not have the feedback batch ABI')
    cases = fixtures()
    before = digest_records(cases)

    def legacy(buses):
        return [runtime.decode_records((records, None)) for _, records in buses]

    def compiled(buses):
        return [decoder.decode(records, first) for first, records in buses]

    # Object, insertion-order, timestamp and IEEE-754 equality are prerequisites
    # to benchmarking. The exhaustive integer-domain tests are separate.
    for buses in cases:
        expected, actual = legacy(buses), compiled(buses)
        if actual != expected or any(list(a) != list(b) for a, b in zip(actual, expected)):
            raise AssertionError('Synthetic Type1 feedback decode differs')
        for a, b in zip(actual, expected):
            for key in a:
                for name in ('protocol_position_rad', 'velocity_rad_s', 'torque_nm', 'temperature_c'):
                    if struct.pack('>d', getattr(a[key][0], name)) != struct.pack('>d', getattr(b[key][0], name)):
                        raise AssertionError(f'Double bits differ: {key} {name}')
    for index in range(warmup):
        buses = cases[index % len(cases)]
        legacy(buses)
        compiled(buses)
    gc_was_enabled = gc.isenabled()
    rows = []
    gc.collect()
    gc.disable()
    try:
        for trial in range(trials):
            measured = {'python': [], 'cpp': []}
            for index in range(samples):
                buses = cases[index % len(cases)]
                order = ('python', 'cpp') if (trial + index) % 2 == 0 else ('cpp', 'python')
                for name in order:
                    method = legacy if name == 'python' else compiled
                    begin = time.perf_counter_ns()
                    result = method(buses)
                    elapsed = time.perf_counter_ns() - begin
                    measured[name].append(elapsed)
                    # Keep every call observable without charging either method
                    # for correctness checks inside the measured interval.
                    if len(result) != 2 or any(len(value) != 6 for value in result):
                        raise AssertionError('Decoder did not return twelve rows')
            rows.append({'trial': trial, **{name: summary(values) for name, values in measured.items()},
                         'median_saved_us': (statistics.median(measured['python']) -
                                             statistics.median(measured['cpp'])) / 1000,
                         'samples_ns': measured})
    finally:
        if gc_was_enabled:
            gc.enable()
    if digest_records(cases) != before:
        raise AssertionError('The decoder modified a source record')
    return {'schema': 'singularitydog.feedback-batch-decode-benchmark.v1',
            'platform': platform.platform(), 'machine': platform.machine(),
            'scope': 'synthetic Type1 -> Type2; twelve rows including Python objects',
            'measurement_order': 'alternate methods each sample; reverse first method each trial',
            'raw_records_sha256': before, 'source_records_unchanged': True,
            'gc_restored': gc.isenabled() == gc_was_enabled,
            'hardware_opened': False, 'robot_commands_sent': False,
            'jetson_measured': False, 'whole_cycle_measured': False,
            'trials': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=2000)
    parser.add_argument('--trials', type=int, default=4)
    parser.add_argument('--warmup', type=int, default=200)
    args = parser.parse_args()
    if not 1 <= args.samples <= 10000 or not 1 <= args.trials <= 10 or not 0 <= args.warmup <= 10000:
        parser.error('Bounded positive sample/trial counts required')
    if args.output.exists():
        parser.error('A fresh output file is required')
    library = native.load_library(args.library)
    report = run(library, samples=args.samples, trials=args.trials, warmup=args.warmup)
    report['library_sha256'] = hashlib.sha256(args.library.read_bytes()).hexdigest()
    report['source_sha256'] = {name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                             for name, path in (('transport.cpp', args.library.parent / 'transport.cpp'),
                                                ('native_active_transport.py', native.__file__),
                                                ('policy_output_runtime.py', runtime.__file__),
                                                ('benchmark_feedback_decode.py', __file__))}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'output': str(args.output.resolve()), 'trials': [
        {k: v for k, v in row.items() if k != 'samples_ns'} for row in report['trials']],
        'jetson_measured': False, 'whole_cycle_measured': False}))


if __name__ == '__main__':
    main()
