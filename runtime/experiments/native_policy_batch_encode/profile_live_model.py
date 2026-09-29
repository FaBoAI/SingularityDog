"""Read-only LivePolicyModel timing with one previously saved model input.

The saved report supplies only numeric sample/IMU values. Timestamps are
synthetically advanced to satisfy the adapter's freshness and monotonicity
checks. No transport, serial, IMU, or motor API is imported or called here.
"""

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import time
from types import SimpleNamespace

import torch

from singularitydog_hw import policy_output_model as model_module
from singularitydog_hw.policy_live_profile import load_profile
from singularitydog_hw.policy_shadow import CAN_ORDER


def _summarize(rows):
    return {'median_us': round(statistics.median(rows) / 1000, 3),
            'p95_us': round(sorted(rows)[int(.95 * (len(rows) - 1))] / 1000, 3),
            'maximum_us': round(max(rows) / 1000, 3)}


def _saved_values(path):
    observed = json.loads(path.read_text())['observation']
    vectors = observed['observer_tick']['inputs']
    raw_imu = observed['snapshot']['imu']
    q = [0.] * 12
    dq = [0.] * 12
    for index, mid in enumerate(CAN_ORDER):
        q[mid - 1] = vectors['q_model_rad'][index]
        dq[mid - 1] = vectors['dq_model_rad_s'][index]
    sample = SimpleNamespace(q_model_rad=tuple(q), velocity_rad_s=tuple(dq))
    imu = {'frame': 'sensor', 'accel_m_s2': tuple(raw_imu['accel_m_s2']),
           'gyro_rad_s': tuple(raw_imu['gyro_rad_s'])}
    return sample, imu


def run(profile_path, saved_path, *, cycles=300, warmup=30):
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    profile = load_profile(profile_path)
    sample, saved_imu = _saved_values(saved_path)
    model = model_module.LivePolicyModel(profile, torch_module=torch)
    buckets = {name: [] for name in ('total', 'input_validation', 'policy_call',
                                     'target_row', 'actor_observation_finite',
                                     'other_including_buffer_fill')}
    current = {}
    original_validate = model.validate_inputs
    original_policy = model.policy
    original_row = model_module._tensor_row
    original_finite = model_module._finite_cpu_float32_row

    def timed_validate(*args):
        start = time.perf_counter_ns()
        result = original_validate(*args)
        current['input_validation'] += time.perf_counter_ns() - start
        return result

    class TimedPolicy:
        def __call__(self, *args):
            start = time.perf_counter_ns()
            result = original_policy(*args)
            current['policy_call'] += time.perf_counter_ns() - start
            return result

        def __getattr__(self, name):
            return getattr(original_policy, name)

    def timed_row(*args):
        start = time.perf_counter_ns()
        result = original_row(*args)
        current['target_row'] += time.perf_counter_ns() - start
        return result

    def timed_finite(*args):
        start = time.perf_counter_ns()
        result = original_finite(*args)
        current['actor_observation_finite'] += time.perf_counter_ns() - start
        return result

    model.validate_inputs = timed_validate
    model.policy = TimedPolicy()
    model_module._tensor_row = timed_row
    model_module._finite_cpu_float32_row = timed_finite
    try:
        for index in range(warmup + cycles):
            now_ns = 1_000_000_000 + index * 20_000_000
            imu = {**saved_imu, 'read_started_monotonic_ns': now_ns - 1_000_000,
                   'read_finished_monotonic_ns': now_ns - 500_000}
            current = {name: 0 for name in ('input_validation', 'policy_call',
                                           'target_row', 'actor_observation_finite')}
            start = time.perf_counter_ns()
            model(sample, imu, now_ns)
            total = time.perf_counter_ns() - start
            if index >= warmup:
                for name, value in current.items():
                    buckets[name].append(value)
                buckets['total'].append(total)
                buckets['other_including_buffer_fill'].append(total - sum(current.values()))
    finally:
        model_module._tensor_row = original_row
        model_module._finite_cpu_float32_row = original_finite
        model.policy = original_policy
        model.validate_inputs = original_validate
    return {'schema': 'singularitydog.offline-live-model-profile.v1',
            'hardware_opened': False, 'motor_commands_sent': False,
            'saved_input_sha256': hashlib.sha256(saved_path.read_bytes()).hexdigest(),
            'cycles': cycles, 'warmup': warmup,
            'timings': {name: _summarize(rows) for name, rows in buckets.items()}}


def compare_threads(profile_path, saved_path, *, cycles=300, warmup=30):
    original_threads = torch.get_num_threads()
    profile = load_profile(profile_path)
    sample, saved_imu = _saved_values(saved_path)
    model = model_module.LivePolicyModel(profile, torch_module=torch)
    desired = list(dict.fromkeys((original_threads, 1, 2, 4)))
    reference_outputs = None
    reference_state = None
    reference_reset_state = None
    results = {}
    try:
        for threads in desired:
            torch.set_num_threads(threads)
            with torch.inference_mode():
                model.policy.reset(torch.tensor([0], dtype=torch.long))
            reset_state = {name: value.detach().clone()
                           for name, value in model.policy.state_dict().items()}
            if reference_reset_state is None:
                reference_reset_state = reset_state
            elif not (reset_state.keys() == reference_reset_state.keys() and
                      all(torch.equal(reset_state[name], reference_reset_state[name])
                          for name in reset_state)):
                raise AssertionError('Model reset state differs between thread settings')
            model.last_imu_ns = 0
            model.last_tick_ns = 0
            model.calls = 0
            outputs = []
            durations = []
            for index in range(warmup + cycles):
                now_ns = 1_000_000_000 + index * 20_000_000
                imu = {**saved_imu, 'read_started_monotonic_ns': now_ns - 1_000_000,
                       'read_finished_monotonic_ns': now_ns - 500_000}
                start = time.perf_counter_ns()
                target = model(sample, imu, now_ns)
                elapsed = time.perf_counter_ns() - start
                if index >= warmup:
                    durations.append(elapsed)
                    outputs.append((target, model.policy.last_actor_output.detach().clone(),
                                    model.policy.last_observation.detach().clone()))
            state = {name: value.detach().clone()
                     for name, value in model.policy.state_dict().items()}
            if reference_outputs is None:
                reference_outputs = outputs
                reference_state = state
            elif not (len(outputs) == len(reference_outputs) and
                      all(a[0] == b[0] and torch.equal(a[1], b[1]) and
                          torch.equal(a[2], b[2])
                          for a, b in zip(outputs, reference_outputs)) and
                      state.keys() == reference_state.keys() and
                      all(torch.equal(state[name], reference_state[name]) for name in state)):
                raise AssertionError('Model target, actor, observation or state differs')
            results[str(threads)] = _summarize(durations)
    finally:
        torch.set_num_threads(original_threads)
    return {'schema': 'singularitydog.offline-live-model-thread-compare.v1',
            'hardware_opened': False, 'motor_commands_sent': False,
            'saved_input_sha256': hashlib.sha256(saved_path.read_bytes()).hexdigest(),
            'cycles': cycles, 'warmup': warmup, 'original_torch_threads': original_threads,
            'restored_torch_threads': torch.get_num_threads(),
            'exact_target_actor_observation_and_state_parity': True,
            'timings_by_torch_threads': results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--saved-report', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=300)
    parser.add_argument('--compare-threads', action='store_true')
    args = parser.parse_args()
    func = compare_threads if args.compare_threads else run
    print(json.dumps(func(args.profile, args.saved_report, cycles=args.cycles),
                     sort_keys=True))


if __name__ == '__main__':
    main()
