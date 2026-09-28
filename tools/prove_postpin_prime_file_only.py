#!/usr/bin/env python3
"""R22 file-only proof: replay one saved snapshot with prime 0 and prime N.

No native library, serial port, CAN exchange, IMU or motor output is opened.
The source-only kit and its private saved input are read and verified first.
"""
import argparse
from array import array
from contextlib import redirect_stdout
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import time


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _verify_kit(root):
    manifest_path = root/'kit-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('schema') != 'private-overnight-kit-v1' or not manifest.get('files'):
        raise ValueError('Missing kit manifest')
    for item in root.rglob('*'):
        if item.is_symlink():
            raise ValueError('Symlink in kit: '+str(item.relative_to(root)))
        if item.is_file() and item != manifest_path and str(item.relative_to(root)) not in manifest['files']:
            raise ValueError('Unlisted kit file: '+str(item.relative_to(root)))
    for name, digest in manifest['files'].items():
        rel = Path(name)
        if rel.is_absolute() or '..' in rel.parts or not (root/rel).is_file():
            raise ValueError('Invalid kit manifest path: '+name)
        if _sha(root/rel) != digest:
            raise ValueError('Kit file changed: '+name)
    return manifest


def _model_state(policy, torch):
    return {name: tensor.detach().clone() for name, tensor in policy.state_dict().items()}


def _same_state(left, right, torch):
    return left.keys() == right.keys() and all(torch.equal(left[k], right[k]) for k in left)


def _plan(benchmark, *, prime_calls):
    args = ['--mode', 'stop-proxy', '--cycles', '500', '--record-storage', 'trace',
            '--output-dispatch-trace', '--defer-gc-during-cycles',
            '--pre-cycle-policy-warmup-calls', '10', '--main-thread-cpu', '4']
    if prime_calls:
        args += ['--post-pin-policy-prime-calls', str(prime_calls)]
    output = io.StringIO()
    # Plan mode reads only CLI settings. Simulated affinity allows the same
    # CPU4 plan check on a non-Jetson host; it is not a hardware availability claim.
    original = getattr(benchmark.os, 'sched_getaffinity', None)
    original_set = getattr(benchmark.os, 'sched_setaffinity', None)
    benchmark.os.sched_getaffinity = lambda _pid: {0, 4}
    benchmark.os.sched_setaffinity = lambda _pid, _mask: None
    try:
        with redirect_stdout(output):
            if benchmark.main(args) != 0:
                raise AssertionError('CLI plan failed')
    finally:
        if original is None:
            delattr(benchmark.os, 'sched_getaffinity')
        else:
            benchmark.os.sched_getaffinity = original
        if original_set is None:
            delattr(benchmark.os, 'sched_setaffinity')
        else:
            benchmark.os.sched_setaffinity = original_set
    plan = json.loads(output.getvalue())
    if (plan['mode'] != 'stop-proxy' or plan['cycles'] != 500 or
            plan.get('post_pin_policy_prime_calls') != (prime_calls or None)):
        raise AssertionError('Unexpected R22 CLI plan')
    return plan


def _saved_sequence(rows):
    names = ('gyro_body_rad_s', 'gravity_body_unit', 'command', 'q_model_rad',
             'dq_model_rad_s', 'h_hypothesis12')
    lengths = (3, 3, 3, 12, 12, 12)
    if type(rows) is not list or len(rows) != 500:
        raise ValueError('Require all 500 R21 saved cycles')
    result = []
    for cycle, row in enumerate(rows, 1):
        values = row.get('observed', {}).get('inputs')
        if row.get('cycle') != cycle or type(values) is not dict or set(values) != set(names):
            raise ValueError('Invalid saved model input sequence')
        vectors = tuple(values[name] for name in names)
        if any(type(v) is not list or len(v) != n or any(type(x) not in (float, int)
                   or not math.isfinite(x) for x in v) for v, n in zip(vectors, lengths)):
            raise ValueError('Invalid saved model vector')
        result.append(vectors)
    return result


def _sequence_trial(root, config, calibration, mount, torch, observer, replay, shadow,
                    vectors, prime_calls):
    policy, _ = shadow.load_policy(root/config['bundle'])
    run = observer.StatefulPolicyObserver(
        policy, calibration, imu_mount_candidate=mount, h_hypothesis=0,
        command=[0., 0., 0.], max_ticks=1, max_age_ns=100_000_000,
        max_spread_ns=100_000_000, torch_module=torch, reuse_input_buffers=True)
    replay.warmup_policy(policy, torch, 0, 10)
    if prime_calls:
        replay.warmup_policy(policy, torch, 0, prime_calls,
                             input_tensors=run._input_tensors)
    run.prepare_run(warmup_completed=True)
    reset_state = _model_state(policy, torch)
    durations = []
    outputs = []
    with torch.inference_mode():
        for values in vectors:
            for tensor, value in zip(run._input_tensors, values):
                tensor.copy_(torch.tensor([value], dtype=torch.float32))
            begin = time.perf_counter_ns()
            target = policy(*run._input_tensors)
            duration = time.perf_counter_ns()-begin
            durations.append(duration)
            outputs.append((target.detach().cpu().tolist(),
                            policy.last_actor_output.detach().cpu().tolist(),
                            policy.last_observation.detach().cpu().tolist()))
    return {'reset_count': run.reset_count, 'reset_state': reset_state,
            'final_state': _model_state(policy, torch), 'outputs': outputs,
            'durations_ns': durations}


def _timings(durations):
    ordered = sorted(durations)
    return {'first_three_ms': [x/1e6 for x in durations[:3]],
            'p99_ms': ordered[math.ceil(.99*len(ordered))-1]/1e6,
            'max_ms': ordered[-1]/1e6}


def prove(root, prime_calls, *, saved_records=None, saved_records_sha256=None):
    if type(prime_calls) is not int or not 1 <= prime_calls <= 100:
        raise ValueError('Prime calls must be 1..100')
    root = Path(root).expanduser().resolve(strict=True)
    original_open = os.open
    serial_attempts = []
    def guarded_open(path, *args, **kwargs):
        value = os.fspath(path)
        if isinstance(value, bytes):
            value = os.fsdecode(value)
        if value.startswith(('/dev/serial/', '/dev/tty', '/dev/cu.')):
            serial_attempts.append(value)
            raise AssertionError('Serial port open attempted in file-only proof')
        return original_open(path, *args, **kwargs)
    os.open = guarded_open
    try:
        manifest = _verify_kit(root)
        config = json.loads((root/'kit-config.json').read_text())
        if config.get('revision') != '20260928-r22-postpin-buffer-prime-diagnostic':
            raise ValueError('Require R22 post-pin prime kit')
        sys.path.insert(0, str(root/'runtime'))
        import torch
        from singularitydog_hw import native_pipeline_benchmark as benchmark
        from singularitydog_hw import policy_observer as observer
        from singularitydog_hw import policy_observer_replay as replay
        from singularitydog_hw import policy_shadow as shadow
        _plan(benchmark, prime_calls=0)
        _plan(benchmark, prime_calls=prime_calls)
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        calibration = json.loads((root/config['calibration']).read_text())
        mount = json.loads((root/config['mount']).read_text())
        saved_path = root/'inputs/saved-policy-report.json'
        saved = json.loads(saved_path.read_text())['observation']
        snapshot = saved['snapshot']
        expected_inputs = saved['observer_tick']['inputs']
        outcomes = []
        for calls in (0, prime_calls):
            policy, _ = shadow.load_policy(root/config['bundle'])
            run = observer.StatefulPolicyObserver(
                policy, calibration, imu_mount_candidate=mount,
                h_hypothesis=0, command=[0., 0., 0.], max_ticks=1,
                max_age_ns=snapshot['max_age_ns'], max_spread_ns=snapshot['max_spread_ns'],
                torch_module=torch, measured_diagnostic_ticks=True, reuse_input_buffers=True)
            replay.warmup_policy(policy, torch, 0, 10)
            if calls:
                replay.warmup_policy(policy, torch, 0, calls,
                                     input_tensors=run._input_tensors)
            run.prepare_run(warmup_completed=True)
            reset_state = _model_state(policy, torch)
            run.arm_run(snapshot['tick_ns'])
            row = run.consume(snapshot)
            if row['inputs'] != expected_inputs or run.reset_count != 1 or run.ticks_completed != 1:
                raise AssertionError('Saved input replay or observer reset differs')
            outcomes.append((row, reset_state, _model_state(policy, torch)))
        a, b = outcomes
        output_fields = ('inputs', 'observation74', 'actor_residual12',
                         'q_target_rad_diagnostic_only')
        if (any(a[0][key] != b[0][key] for key in output_fields) or
                not _same_state(a[1], b[1], torch) or
                not _same_state(a[2], b[2], torch)):
            raise AssertionError('Post-pin prime changed model output or state after reset')
        if serial_attempts:
            raise AssertionError('Serial port open attempted')
        sequence_proof = None
        if saved_records is not None or saved_records_sha256 is not None:
            if saved_records is None or saved_records_sha256 is None:
                raise ValueError('Saved records and expected SHA256 are required together')
            records_path = Path(saved_records).expanduser().resolve(strict=True)
            if _sha(records_path) != saved_records_sha256:
                raise ValueError('Saved R21 records SHA256 differs')
            vectors = _saved_sequence(json.loads(records_path.read_text()))
            baseline = _sequence_trial(root, config, calibration, mount, torch, observer,
                                       replay, shadow, vectors, 0)
            candidate = _sequence_trial(root, config, calibration, mount, torch, observer,
                                        replay, shadow, vectors, prime_calls)
            if (baseline['reset_count'] != 1 or candidate['reset_count'] != 1 or
                    baseline['outputs'] != candidate['outputs'] or
                    not _same_state(baseline['reset_state'], candidate['reset_state'], torch) or
                    not _same_state(baseline['final_state'], candidate['final_state'], torch)):
                raise AssertionError('Saved-sequence model action/state parity differs')
            sequence_proof = {'source': 'R21 saved 500 model-input vectors',
                              'records_sha256': saved_records_sha256,
                              'model_scope': 'R18 bundled legacy policy; R21 cached-view model not replayed',
                              'calls_after_final_reset': len(vectors),
                              'first_three_after_reset_compared': True,
                              'exact_action_and_observation_parity': True,
                              'reset_state_equal': True, 'final_state_equal': True,
                              'baseline_model_call_local_ms': _timings(baseline['durations_ns']),
                              'candidate_model_call_local_ms': _timings(candidate['durations_ns']),
                              'timing_scope': 'local file-only model call; not Jetson cycle timing'}
        return {'schema': 'r22-postpin-prime-file-only-proof-v1',
                'status': 'PASS_FILE_ONLY_REPLAY', 'kit_revision': config['revision'],
                'base_manifest_sha256': manifest['ancestry']['base_manifest_sha256'],
                'saved_report_sha256': _sha(saved_path), 'saved_input_matches': True,
                'prime_calls': prime_calls, 'reset_count_per_run': 1,
                'outputs_equal': True, 'state_after_reset_equal': True,
                'state_after_tick_equal': True, 'serial_port_open_attempts': 0,
                'hardware_accessed': False, 'timing_claim': False,
                'saved_sequence_proof': sequence_proof}
    finally:
        os.open = original_open


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kit-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--prime-calls', type=int, default=10)
    parser.add_argument('--saved-records', type=Path)
    parser.add_argument('--saved-records-sha256')
    args = parser.parse_args(argv)
    print(json.dumps(prove(args.kit_root, args.prime_calls,
                           saved_records=args.saved_records,
                           saved_records_sha256=args.saved_records_sha256), sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
