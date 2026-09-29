"""Offline CPU cost breakdown of the twelve-axis post-reply path.

Run with PYTHONPATH=runtime python3 runtime/tests/bench_policy_post_reply_cpu.py.
This constructs decoded Type2 values in memory; it opens no motor, IMU, or USB
resource. The optional boot check reads only /proc/sys/kernel/random/boot_id.
The figures are microbenchmarks, not a motor-cycle deadline proof.
"""

import ctypes
import gc
import json
import math
import statistics
import time
from types import SimpleNamespace

from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import policy_motion_envelope as motion
from singularitydog_hw.policy_post_reply_timing import PostReplyDeadlineBudget
from singularitydog_hw.rs05_trial_protocol import Type2Feedback
from singularitydog_hw.sensor_pipeline_benchmark import BootIdentityGuard


COUNT = 20_000
REPEATS = 7
NOW_NS = 1_000_000_000


def fixture():
    axes = {}
    offsets = {}
    recent = {}
    previous = {}
    for mid in runtime.IDS:
        q = mid * .02
        sign = -1 if mid % 2 else 1
        offsets[mid] = q - sign * q
        axes[str(mid)] = dict(sign=sign, lower_rad=q - .1, upper_rad=q + .1,
                              max_measured_velocity_rad_s=.25,
                              max_measured_torque_nm=1., max_temperature_c=45.,
                              max_tracking_error_rad=.04,
                              max_displacement_from_start_rad=.02,
                              max_estimated_pd_torque_nm=.1)
        feedback = Type2Feedback(2, 0, 32768 + mid, q, .001, .002, 30.)
        recent[mid, 'feedback'] = (feedback, NOW_NS - 7_000_000, NOW_NS - 2_000_000)
        previous[mid, 'feedback'] = (feedback, NOW_NS - 17_000_000, NOW_NS - 12_000_000)
    profile = dict(axes=axes, max_sample_age_ms=20.)
    checked = runtime.feedback_sample(recent, profile, offsets,
                                      now_ns=NOW_NS, previous=previous)
    command = SimpleNamespace(q_model_rad=checked.q_model_rad,
                              kp=(3.,) * 12, kd=(.15,) * 12)
    return profile, offsets, recent, previous, checked, command


def check_limits(checked, command, initial, axis_profiles):
    # Keep the branch/order/messages in policy_output_runtime's post-output
    # loop so this measures its Python cost. This function is never used live.
    for k, a in enumerate(axis_profiles):
        mid = k + 1
        runtime.need(a['lower_rad'] <= checked.q_model_rad[k] <= a['upper_rad'],
                     f'ID{mid} joint limit')
        runtime.need(abs(checked.torque_nm[k]) <= a['max_measured_torque_nm'],
                     f'ID{mid} torque')
        runtime.need(abs(checked.velocity_rad_s[k]) <= a['max_measured_velocity_rad_s'],
                     f'ID{mid} velocity')
        runtime.need(checked.temperature_c[k] <= a['max_temperature_c'],
                     f'ID{mid} temperature')
        runtime.need(abs(checked.q_model_rad[k] - command.q_model_rad[k]) <=
                     a['max_tracking_error_rad'], f'ID{mid} tracking error')
        runtime.need(abs(checked.q_model_rad[k] - initial.q_model_rad[k]) <=
                     a['max_displacement_from_start_rad'], f'ID{mid} trial displacement')
        estimated = (command.kp[k] * (command.q_model_rad[k] - checked.q_model_rad[k]) -
                     command.kd[k] * checked.velocity_rad_s[k])
        runtime.need(abs(estimated) <= a['max_estimated_pd_torque_nm'],
                     f'ID{mid} estimated PD torque')


def measure(fn):
    for _ in range(100):
        fn()
    timings = []
    for _ in range(REPEATS):
        start = time.perf_counter_ns()
        for _ in range(COUNT):
            fn()
        timings.append((time.perf_counter_ns() - start) / COUNT / 1000)
    return dict(median_us=statistics.median(timings), minimum_us=min(timings),
                maximum_us=max(timings))


def main():
    profile, offsets, recent, previous, checked, command = fixture()
    axes = tuple(profile['axes'][str(mid)] for mid in runtime.IDS)
    front = {key: row for key, row in recent.items() if key[0] <= 6}
    rear = {key: row for key, row in recent.items() if key[0] >= 7}
    halves = (front, rear)
    double12 = ctypes.c_double * 12

    def merge():
        returned = {}
        for current in halves:
            runtime.need(not returned.keys() & current.keys(), 'Duplicate cross-bus response')
            returned.update(current)
        return returned

    def sample():
        return runtime.feedback_sample(recent, profile, offsets,
                                       now_ns=NOW_NS, previous=previous)

    def limits():
        check_limits(checked, command, checked, axes)

    def scalar_row():
        return dict(index=0, phase='starting', begin_ns=NOW_NS - 18_000_000,
                    output_reply_end_ns=NOW_NS - 2_000_000,
                    output_exchange_return_ns=NOW_NS,
                    oldest_input_to_final_host_write_ms=14., feedback=checked,
                    imu_body=None, release_lateness_ms=0., release_interval_ms=20.,
                    effective_policy_weight=0., command=command)

    def ctypes_copy():
        # Minimum Python-side copy for four sample vectors alone. A separate
        # C++ validator would also need command and every per-axis limit.
        return tuple(double12(*values) for values in
                     (checked.q_model_rad, checked.velocity_rad_s,
                      checked.torque_nm, checked.temperature_c))

    def ctypes_full_copy():
        # A separate native validator also needs these command and limit
        # values. This excludes dispatch and result-object construction.
        vectors = [checked.q_model_rad, checked.velocity_rad_s,
                   checked.torque_nm, checked.temperature_c,
                   command.q_model_rad, command.kp, command.kd,
                   checked.q_model_rad]
        vectors.extend(tuple(a[key] for a in axes) for key in (
            'lower_rad', 'upper_rad', 'max_measured_velocity_rad_s',
            'max_measured_torque_nm', 'max_temperature_c',
            'max_tracking_error_rad', 'max_displacement_from_start_rad',
            'max_estimated_pd_torque_nm'))
        return tuple(double12(*values) for values in vectors)

    def post_reply():
        returned = merge()
        current = runtime.feedback_sample(returned, profile, offsets,
                                          now_ns=NOW_NS, previous=previous)
        check_limits(current, command, checked, axes)
        row = scalar_row()
        row['feedback'] = current
        return row

    enabled = gc.isenabled()
    gc.disable()
    try:
        results = {name: measure(fn) for name, fn in (
            ('merge', merge), ('feedback_sample', sample),
            ('axis_limits', limits), ('scalar_row', scalar_row),
            ('ctypes_four_vectors_copy', ctypes_copy),
            ('ctypes_full_vectors_copy', ctypes_full_copy),
            ('combined_no_boot_check', post_reply))}
        original_vector = motion._vector
        def finite_float_tuple(values, name):
            if (type(values) is tuple and len(values) == 12 and
                    all(type(value) is float for value in values) and
                    all(map(math.isfinite, values))):
                return values
            return original_vector(values, name)
        try:
            motion._vector = finite_float_tuple
            assert sample() == checked
            results['feedback_sample_exact_tuple_fastpath'] = measure(sample)
            results['combined_exact_tuple_fastpath'] = measure(post_reply)
        finally:
            motion._vector = original_vector
        settings = dict(max_lateness_ms=1., max_consecutive_misses=1,
                        rolling_window_cycles=100, max_misses_per_window=1)
        def admission():
            budget = PostReplyDeadlineBudget(settings)
            return budget.admit(index=0, begin_ns=NOW_NS - 18_000_000,
                                oldest_input_ns=NOW_NS - 17_000_000,
                                final_write_ns=NOW_NS - 3_000_000,
                                last_reply_ns=NOW_NS - 2_000_000,
                                output_sample_start_ns=NOW_NS - 7_000_000,
                                checked_ns=NOW_NS, sample_age_ns=20_000_000)
        results['admission_with_new_budget'] = measure(admission)
        try:
            guard = BootIdentityGuard()
        except (OSError, RuntimeError, ValueError):
            results['boot_check'] = None
        else:
            try:
                results['boot_check'] = measure(guard.check)
            finally:
                guard.close()
    finally:
        if enabled:
            gc.enable()
    print(json.dumps(dict(schema='singularitydog.offline-post-reply-cpu-benchmark.v1',
                          iterations_per_repeat=COUNT, repeats=REPEATS,
                          hardware_opened=False, motor_commands_sent=False,
                          results=results), sort_keys=True))


if __name__ == '__main__':
    main()
