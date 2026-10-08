"""File-only balanced comparison of the verified twelve-axis encoder wrapper.

Measures the existing Python encode/parse/quantized-limit path against the
existing verified CPython wrapper, including twelve returned 17-byte wires.
No transport, model, profile, approval, session or motor descriptor is opened.
"""
import argparse
import gc
import hashlib
import json
from pathlib import Path
import platform
import random
import statistics
import time

from singularitydog_hw import native_policy_batch_encode as batch


def command_cases(specs, *, count, seed=890):
    rng = random.Random(seed)
    initial = tuple(row[4] for row in specs)
    return tuple(batch._command(
        (q + rng.uniform(-.015, .015) for q in initial),
        (rng.uniform(0., 10.) for _ in range(12)),
        (rng.uniform(0., .5) for _ in range(12)),
        (rng.uniform(-.01, .01) for _ in range(12))) for _ in range(count))


def same_wires(actual, expected):
    if (actual != expected or list(actual) != list(expected) or
            list(actual) != ['front', 'rear'] or
            any(type(wire) is not bytes or len(wire) != 17
                for wires in actual.values() for wire in wires) or
            any(type(wires) is not list or len(wires) != 6 for wires in actual.values())):
        raise AssertionError('Verified encoder changed twelve bytes wires or bus ordering')


def summary(values):
    ordered = sorted(values)
    return {'samples': len(values), 'median_us': statistics.median(values) / 1000,
            'p99_us': ordered[int((len(ordered) - 1) * .99)] / 1000,
            'max_us': max(values) / 1000}


def run(library, *, binary_sha256, samples=2000, trials=4, warmup=200, parity_cases=2000):
    if (type(samples) is not int or not 1 <= samples <= 10000 or
            type(trials) is not int or not 1 <= trials <= 10 or
            type(warmup) is not int or not 0 <= warmup <= 10000 or
            type(parity_cases) is not int or not 1 <= parity_cases <= 10000):
        raise ValueError('Bounded positive sample, trial and parity counts required')
    # This loader verifies source and binary pins and runs its 96 seeded byte
    # canaries and 72 rejection canaries before exposing the existing wrapper.
    verified = batch.load_verified_module(library, expected_binary_sha256=binary_sha256)
    specs = batch._test_specs()
    encoder = verified.bind(specs)
    baseline = lambda command: batch._reference_wires(command, specs)
    cases = command_cases(specs, count=parity_cases)
    digest = hashlib.sha256()
    for command in cases:
        expected = baseline(command)
        same_wires(encoder(command), expected)
        for wires in expected.values():
            for wire in wires:
                digest.update(wire)
    for index in range(warmup):
        command = cases[index % len(cases)]
        baseline(command)
        encoder(command)

    gc_was_enabled = gc.isenabled()
    rows = []
    gc.collect()
    gc.disable()
    try:
        for trial in range(trials):
            measured = {'python': [], 'verified_cpp': []}
            for index in range(samples):
                command = cases[index % len(cases)]
                order = ('python', 'verified_cpp') if (trial + index) % 2 == 0 else ('verified_cpp', 'python')
                results = {}
                for name in order:
                    method = baseline if name == 'python' else encoder
                    begin = time.perf_counter_ns()
                    result = method(command)
                    elapsed = time.perf_counter_ns() - begin
                    measured[name].append(elapsed)
                    results[name] = result
                # Compare every measured result outside both timed intervals.
                same_wires(results['verified_cpp'], results['python'])
            saved = statistics.median(measured['python']) - statistics.median(measured['verified_cpp'])
            rows.append({'trial': trial,
                         **{name: summary(values) for name, values in measured.items()},
                         'median_saved_us': saved / 1000,
                         'median_saved_percent': 100 * saved / statistics.median(measured['python']),
                         'samples_ns': measured})
    finally:
        if gc_was_enabled:
            gc.enable()
    return {'schema': 'singularitydog.verified-batch-encoder-benchmark.v1',
            'platform': platform.platform(), 'machine': platform.machine(),
            'scope': 'twelve 17-byte Type1 wires; verified wrapper and quantized limit checks',
            'measurement_order': 'alternate methods each sample; reverse first method each trial',
            'source_sha256': dict(batch.PINNED_SOURCE_SHA256),
            'binary_sha256': binary_sha256, 'parity_cases': parity_cases,
            'loader_byte_canaries': 96, 'loader_rejection_canaries': 72,
            'fresh_profile_binding_canary': True,
            'reference_wires_sha256': digest.hexdigest(),
            'gc_restored': gc.isenabled() == gc_was_enabled,
            'hardware_opened': False, 'motor_commands_sent': False,
            'model_loaded': False, 'profile_changed': False, 'output_approved': False,
            'jetson_measured': False, 'whole_cycle_measured': False,
            'trials': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--binary-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=2000)
    parser.add_argument('--trials', type=int, default=4)
    parser.add_argument('--warmup', type=int, default=200)
    parser.add_argument('--parity-cases', type=int, default=2000)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('A fresh output file is required')
    try:
        report = run(args.library, binary_sha256=args.binary_sha256,
                     samples=args.samples, trials=args.trials,
                     warmup=args.warmup, parity_cases=args.parity_cases)
    except (ValueError, RuntimeError) as error:
        parser.error(str(error))
    report['benchmark_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['loader_source_sha256'] = hashlib.sha256(Path(batch.__file__).read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'output': str(args.output.resolve()),
                      'trials': [{key: value for key, value in row.items() if key != 'samples_ns'}
                                 for row in report['trials']],
                      'parity_cases': report['parity_cases'],
                      'jetson_measured': False, 'whole_cycle_measured': False}))


if __name__ == '__main__':
    main()
