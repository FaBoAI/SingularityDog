"""Pinned real-model CPU replay. Default PLAN; no device or live selector."""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import sys
import time

if __package__:
    from .file_inputs import need, pinned, reference, source_inventory, verify_pins, fresh_output, distribution
else:
    from file_inputs import need, pinned, reference, source_inventory, verify_pins, fresh_output, distribution

HERE = Path(__file__).absolute().parent
KEYS = ('gyro_body_rad_s', 'gravity_body_unit', 'command', 'q_model_rad',
        'dq_model_rad_s', 'h_hypothesis12')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--inputs', required=True, type=Path)
    parser.add_argument('--inputs-sha256', required=True)
    parser.add_argument('--build-record', required=True, type=Path)
    parser.add_argument('--build-record-sha256', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--require-saved-reference-bit-parity', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    need(not sys.flags.optimize and sys.dont_write_bytecode, 'Unsuppressed python -B required')
    pins = {}
    config_ref = {'path': str(args.inputs.absolute()), 'sha256': args.inputs_sha256}
    config = json.loads(reference(config_ref, pins))
    need(type(config) is dict and set(config) == {
        'schema', 'runtime_root', 'source_manifest', 'original_live_source',
        'original_model', 'candidate_model', 'libraries', 'records'} and
        config['schema'] == 'experimental.live-checked-dispatch-file-inputs.v1',
        'Exact explicit file-only inputs schema required')
    runtime = Path(config['runtime_root'])
    manifest = source_inventory(runtime, config['source_manifest'], pins)
    original_source = runtime / 'singularitydog_hw/policy_output_model.py'
    need(config['original_live_source']['path'] == str(original_source), 'Selected original module path differs')
    reference(config['original_live_source'], pins)
    original_bytes = reference(config['original_model'], pins)
    candidate_bytes = (reference(config['candidate_model'], pins)
                       if config['candidate_model'] is not None else None)
    need(type(config['libraries']) is list and len(config['libraries']) > 0, 'Explicit operator library pins required')
    for ref in config['libraries']:
        reference(ref, pins)
    rows = json.loads(reference(config['records'], pins))
    need(type(rows) is list and len(rows) == 501, 'Original saved501 input rows required')
    frames = []
    for index, row in enumerate(rows):
        need(row['cycle'] == index + 1 and set(row['observed']['inputs']) == set(KEYS),
             'Original row order/input fields differ')
        values = tuple(row['observed']['inputs'][name] for name in KEYS)
        need(all(type(value) is list and len(value) == count for value, count in
                 zip(values, (3, 3, 3, 12, 12, 12))), 'Six exact input extents required')
        need(all(type(value) in (int, float) and math.isfinite(value)
                 for row_values in values for value in row_values), 'Finite numeric saved inputs required')
        json.dumps(values, allow_nan=False)
        frames.append(values)
    record_ref = {'path': str(args.build_record.absolute()), 'sha256': args.build_record_sha256}
    record = json.loads(reference(record_ref, pins))
    reference({'path': str(HERE / 'checked_dispatch.cpp'), 'sha256': record['source_sha256']}, pins)
    library = args.build_record.absolute().parent / 'live_checked_dispatch.so'
    reference({'path': str(library), 'sha256': record['library_sha256']}, pins)
    for name in ('compare.py', 'file_inputs.py', 'adapter.py', 'cpu_kernel.py'):
        path = HERE / name
        reference({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}, pins)
    names = ['original', 'checked_original']
    if candidate_bytes is not None:
        names += ['r11', 'checked_r11']
    plan = {'schema': 'experimental.live-checked-dispatch-replay.v1', 'status': 'PLAN_ONLY',
            'input_sha256': pins, 'source_manifest_schema': manifest['schema'],
            'source_file_count': manifest['file_count'], 'variants': names,
            'hardware_opened': False, 'output_allowed': False, 'approved_for_runtime': False,
            'active_controller_qualification': False, 'timing_admission_eligible': False,
            'live_type1_qualified': False, 'physical_future_observations': None,
            'model_sensor_clock_or_profile_admission_supplied': False}
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    output = fresh_output(args.output)
    output.mkdir(mode=0o700)
    (output / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    old_path = sys.path[:]
    primary = None
    try:
        sys.path[:0] = [str(HERE), str(runtime)]
        import torch
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        need(torch.__version__ == record['torch_version'] and
             bool(torch._C._GLIBCXX_USE_CXX11_ABI) == record['cxx11_abi'], 'Build/load Torch ABI differs')
        from cpu_kernel import GuardedCpuPolicyKernel, compare_named_state, verify_methods
        from adapter import CheckedDispatchCpuKernel, wrap_policy
        from singularitydog_hw import policy_output_model as original_module
        from singularitydog_hw.policy_shadow import CAN_ORDER
        need(Path(original_module.__file__) == original_source, 'Imported original live source origin differs')
        for ref in config['libraries']:
            torch.ops.load_library(ref['path'])
        torch.ops.load_library(str(library))
        models, kernels, checked_serialized = [], [], []
        original_sha = config['original_live_source']['sha256']
        for data in ([original_bytes] if candidate_bytes is None else [original_bytes, candidate_bytes]):
            model = torch.jit.load(io.BytesIO(data), map_location='cpu').eval()
            if models:
                verify_methods(models[0], model)
            wrapper = wrap_policy(torch.jit.load(io.BytesIO(data), map_location='cpu').eval())
            stream = io.BytesIO()
            torch.jit.save(wrapper, stream)
            checked_serialized.append(hashlib.sha256(stream.getvalue()).hexdigest())
            wrapper = torch.jit.load(io.BytesIO(stream.getvalue()), map_location='cpu').eval()
            models.extend((model, wrapper.inner))
            kernels.extend((GuardedCpuPolicyKernel(model, torch, original_source_sha256=original_sha),
                            CheckedDispatchCpuKernel(wrapper, torch, original_source_sha256=original_sha)))
        for kernel in kernels:
            kernel.pre_pin_warmup()
            kernel.post_pin_prime()
            kernel.finish_startup()
        for model in models[1:]:
            compare_named_state(torch, models[0], model, independent=True)
        def check(targets):
            for target in targets[1:]:
                need(struct.pack('>12d', *targets[0]) == struct.pack('>12d', *target), 'CAN target bits differ')
            for model in models[1:]:
                compare_named_state(torch, models[0], model)
        saved = dict(actor=0, observation=0, target=0)
        tensor_bits = lambda value: value.contiguous().reshape(-1).view(torch.uint8).numpy().tobytes()
        for index, values in enumerate(frames):
            before = json.dumps(values, allow_nan=False)
            targets = [kernel(values) for kernel in kernels]
            check(targets)
            need(json.dumps(values, allow_nan=False) == before, 'Saved input snapshot mutated')
            observed = rows[index]['observed']
            for key, value, field in (('actor', models[0].last_actor_output, 'actor_residual12'),
                                      ('observation', models[0].last_observation, 'observation74')):
                wanted = torch.tensor([observed[field]], dtype=torch.float32)
                saved[key] += int(tensor_bits(value) == tensor_bits(wanted))
            wanted = observed['q_target_rad_diagnostic_only']
            saved['target'] += int(struct.pack('>12d', *targets[0]) ==
                struct.pack('>12d', *[wanted[CAN_ORDER.index(can)] for can in range(1, 13)]))
        if args.require_saved_reference_bit_parity:
            need(all(value == 501 for value in saved.values()), 'Selected reference differs from original saved501 bits')
        times = {name: {'wall_ns': [], 'thread_cpu_ns': []} for name in names}
        blocks = []
        for block, reverse in enumerate((False, True, True, False)):
            for kernel in kernels:
                kernel.reset()
            for index, values in enumerate(frames):
                order = list(range(len(names)))
                offset = index % len(names)
                order = order[offset:] + order[:offset]
                if reverse:
                    order.reverse()
                targets = [None] * len(names)
                for slot in order:
                    cpu, wall = time.thread_time_ns(), time.perf_counter_ns()
                    targets[slot] = kernels[slot](values)
                    end, cpu_end = time.perf_counter_ns(), time.thread_time_ns()
                    times[names[slot]]['wall_ns'].append(end - wall)
                    times[names[slot]]['thread_cpu_ns'].append(cpu_end - cpu)
                check(targets)
            blocks.append({'block': block + 1, 'reverse': reverse, 'calls_per_variant': 501})
        report = {**plan, 'status': 'PASS_REAL_MODEL_CPU_COMPONENT_501',
                  'all501_actor_observation_target_named_state_bits_exact': True,
                  'saved_reference_bit_match_counts': saved, 'input_snapshots_mutated': False,
                  'warmup_original_ten_ten_reset': True, 'balanced_blocks': blocks,
                  'checked_serialized_sha256': checked_serialized, 'raw_times': times,
                  'timing': {name: {clock: distribution(values) for clock, values in clocks.items()}
                             for name, clocks in times.items()},
                  'timing_scope': 'CPU six-buffer fill, policy forward, original output guards and CAN conversion; no input sensor validation or whole cycle',
                  'torch_version': torch.__version__, 'source_and_input_pins_unchanged': True}
        verify_pins(pins)
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({key: value for key, value in report.items()
                          if key not in ('input_sha256', 'raw_times')}, indent=2))
    except BaseException as exc:
        primary = exc
        (output / 'failure.json').write_text(json.dumps({'status': 'CPU_COMPONENT_FAILED',
            'error': type(exc).__name__ + ': ' + str(exc), 'output_allowed': False,
            'approved_for_runtime': False, 'live_type1_qualified': False}, indent=2) + '\n')
        raise
    finally:
        sys.path[:] = old_path
        try:
            verify_pins(pins)
        except BaseException as changed:
            if primary is None:
                raise
            primary.add_note('Input verification after failure: ' + str(changed))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
