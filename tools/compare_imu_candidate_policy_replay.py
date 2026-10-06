#!/usr/bin/env python3
"""Compare raw and norm-candidate gravity on a saved complete diagnose capture.

File-only, four independent stateful models (raw/corrected times h=0/1).
Original timestamps and angle branches remain unchanged. Only the gravity
tensor is substituted after the existing observer validates the raw snapshot.
This offline diagnostic cannot grant runtime calibration or motor approval.
Run with PYTHONPATH=runtime. Outputs contain private paths and measurements.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import analyze_imu_pose_tilt as norm_fit
from analyze_stationary_velocity_probe import exact, need, read_file, strict_json
from singularitydog_hw import policy_observer as observer
from singularitydog_hw import policy_observer_replay as replay
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw import telemetry_snapshot as telemetry

SCHEMA = 'singularitydog.imu-candidate-policy-replay.v1'
INPUT_NAMES = ('gyro_body_rad_s', 'gravity_body_unit', 'command', 'q_model_rad',
               'dq_model_rad_s', 'h_hypothesis12')
WIDTHS = (3, 3, 3, 12, 12, 12)
FILE_MAX = 64*1024*1024
FLAGS = dict.fromkeys(('approved_for_runtime', 'automatically_applied',
    'output_allowed', 'motor_output_available', 'hardware_opened',
    'live_50hz_verified', 'live_20ms_verified', 'physical_accuracy_verified',
    'calibration_verified', 'angle_wrap_applied', 'profile_changed',
    'thresholds_changed', 'heldout_used_for_fit', 'replay_used_for_fit'), False)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def verify_bindings(pins):
    for pin in pins:
        exact(read_file(pin['path'], FILE_MAX)[1], pin, 'Pinned file changed')


def source_bindings():
    runtime = Path(observer.__file__).resolve().parent
    names = ('__init__.py', 'policy_shadow.py', 'policy_observer.py',
             'policy_observer_replay.py', 'telemetry_snapshot.py', 'event_snapshot.py',
             'angle_branch_comparison.py', 'imu_calibration_review.py',
             'imu_calibration.py', 'imu_fixed_mount_baseline.py', 'can_readonly.py')
    import analyze_stationary_velocity_probe as reader
    return [read_file(path)[1] for path in
            (Path(__file__), Path(norm_fit.__file__), Path(reader.__file__),
             *(runtime/name for name in names))]


def audit_norm_candidate(input_path, report_path):
    """Re-audit pinned raw faces and recompute; no replay/holdout refitting."""
    raw, pin = read_file(report_path)
    report = strict_json(raw)
    need(type(report) is dict, 'Norm diagnostic must be an object')
    comparable = {k: v for k, v in report.items() if k != 'generated_at_utc'}
    recomputed = norm_fit.analyze_file(input_path)
    exact(comparable, recomputed, 'Norm diagnostic recomputation')
    exact(report.get('schema'), norm_fit.SCHEMA, 'Norm diagnostic schema')
    exact(report.get('status'), 'UNAPPROVED_NORM_ELLIPSE_DIAGNOSTIC', 'Norm diagnostic status')
    for flag in ('approved_for_runtime', 'automatically_applied', 'heldout_used_for_fit',
                 'independent_captures_used_for_fit'):
        exact(report.get(flag), False, 'Norm diagnostic '+flag)
    pins = [pin]+recomputed['input_bindings']+recomputed['source_bindings']
    verify_bindings(pins)
    return report['accel_diagnostic_candidate'], pins


def no_accel_review(bias):
    need(bias is None or type(bias) is dict, 'Gyro candidate must be an object')
    need(bias is None or 'accel_calibration_review' not in bias,
         'Reviewed acceleration extension is not supported in this raw comparison')


class GravitySubstitutionPolicy:
    """Delegate state; substitute one gravity tensor with one-shot raw context."""
    def __init__(self, policy, candidate, rotation, torch_module):
        self.policy, self.torch = policy, torch_module
        self.correcting = candidate is not None
        self.bias = tuple(candidate['bias_m_s2']) if self.correcting else (0., 0., 0.)
        self.scale = tuple(candidate['scale']) if self.correcting else (1., 1., 1.)
        self.rotation = tuple(tuple(row) for row in rotation)
        need(len(self.bias) == len(self.scale) == 3 and
             all(type(x) in (int, float) and math.isfinite(x) for x in self.bias+self.scale)
             and all(x > 0 for x in self.scale), 'Finite positive diagonal candidate required')
        self._warming, self._pending, self.actual = True, None, None

    @property
    def last_actor_output(self):
        return self.policy.last_actor_output

    @property
    def last_observation(self):
        return self.policy.last_observation

    def reset(self, ids):
        need(self._pending is None, 'Cannot reset armed substitution')
        result = self.policy.reset(ids)
        self._warming = False
        return result

    def arm(self, snapshot):
        need(not self._warming and self._pending is None, 'Substitution context already armed or warming')
        imu = snapshot['imu']
        observer._raw_imu_corrections(imu)
        raw = tuple(imu['accel_m_s2'])
        need(len(raw) == 3 and all(type(x) in (int, float) and math.isfinite(x) for x in raw),
             'Finite original raw acceleration required')
        self.actual = None
        self._pending = (raw, snapshot['tick_ns'], digest(snapshot))

    def clear(self):
        self._pending = None

    def __call__(self, *inputs):
        need(len(inputs) == 6, 'Six ordered policy inputs required')
        if self._warming:
            need(self._pending is None, 'Warmup cannot use captured context')
            return self.policy(*inputs)
        need(self._pending is not None, 'Unarmed or reused substitution context')
        raw, tick, snapshot_sha = self._pending
        self._pending = None  # Consume even when a subsequent gate fails.
        if not self.correcting:
            self.actual = {'actual_model_inputs': {
                name: observer._tensor_row(tensor, width, 'actual '+name)
                for name, tensor, width in zip(INPUT_NAMES, inputs, WIDTHS)},
                'substitution': None, 'model_input_snapshot_binding': {
                    'tick_ns': tick, 'snapshot_canonical_json_sha256': snapshot_sha}}
            return self.policy(*inputs)
        corrected = [(a-b)*s for a, b, s in zip(raw, self.bias, self.scale)]
        norm = math.hypot(*corrected)
        need(all(math.isfinite(x) for x in corrected) and math.isfinite(norm) and norm > 1e-9,
             'Corrected acceleration has invalid/zero norm')
        body = [math.fsum(r*x for r, x in zip(row, corrected)) for row in self.rotation]
        gravity = [-x/norm for x in body]
        substituted = list(inputs)
        substituted[1] = self.torch.tensor([gravity], dtype=self.torch.float32)
        actual = {name: observer._tensor_row(tensor, width, 'actual '+name)
                  for name, tensor, width in zip(INPUT_NAMES, substituted, WIDTHS)}
        original = observer._tensor_row(inputs[1], 3, 'original gravity')
        cosine = math.fsum(a*b for a, b in zip(original, actual['gravity_body_unit']))/(
            math.hypot(*original)*math.hypot(*actual['gravity_body_unit']))
        self.actual = {'actual_model_inputs': actual, 'model_input_snapshot_binding': {
            'tick_ns': tick, 'snapshot_canonical_json_sha256': snapshot_sha},
            'substitution': {'kind': 'unapproved norm-based diagonal offline gravity hypothesis',
                'tick_ns': tick, 'snapshot_canonical_json_sha256': snapshot_sha,
                'raw_accel_sensor_m_s2': list(raw), 'raw_accel_norm_m_s2': math.hypot(*raw),
                'corrected_accel_sensor_m_s2': corrected, 'corrected_accel_body_m_s2': body,
                'corrected_accel_norm_m_s2': norm, 'gravity_before_float32': gravity,
                'gravity_angle_difference_deg_descriptive': math.degrees(math.acos(max(-1., min(1., cosine)))),
                'gravity_only_substituted': True, 'other_five_tensors_preserved': True,
                'bias_sensor_m_s2': list(self.bias), 'scale_sensor': list(self.scale),
                'R_body_from_sensor': [list(row) for row in self.rotation], **FLAGS}}
        return self.policy(*substituted)


def consume_corrected(run, policy, snapshot):
    """Use exactly the armed snapshot; clear context on every success/failure."""
    try:
        policy.arm(snapshot)
        result = run.consume(snapshot)
        exact(policy.actual['model_input_snapshot_binding']['snapshot_canonical_json_sha256'],
              result['provenance']['snapshot_canonical_json_sha256'], 'Substitution snapshot binding')
        exact(policy.actual['model_input_snapshot_binding']['tick_ns'], result['tick_ns'], 'Substitution tick binding')
        return result
    except BaseException as error:
        run.invalidate(type(error).__name__+': '+str(error))
        raise
    finally:
        policy.clear()


def compare_records(records, calibration, policies, *, candidate, imu_mount_candidate,
                    max_ticks, max_age_ns, max_spread_ns, torch_module,
                    gyro_bias_candidate=None, command=(0., 0., 0.), warmup_ticks=3, emit=None):
    """Inputs are already wire-audited; synthetic tests must label their data."""
    need(type(policies) in (list, tuple) and len(policies) == 4 and
         len({id(p) for p in policies}) == 4, 'Four independent policy models required')
    no_accel_review(gyro_bias_candidate)
    mount = shadow.validate_imu_mount_candidate(imu_mount_candidate)
    items, timeline = replay.translate_records(records, calibration)
    emit = emit or (lambda row: None)
    results, observations = [], {}
    for model, (h, condition) in zip(policies, ((0, 'raw'), (0, 'corrected'), (1, 'raw'), (1, 'corrected'))):
        policy = GravitySubstitutionPolicy(model, candidate if condition == 'corrected' else None,
                                           mount['R_body_from_sensor'], torch_module)
        run = observer.StatefulPolicyObserver(policy, calibration, imu_mount_candidate=mount,
            h_hypothesis=h, command=list(command), max_ticks=max_ticks,
            max_age_ns=max_age_ns, max_spread_ns=max_spread_ns, torch_module=torch_module,
            gyro_bias_candidate=gyro_bias_candidate, apply_reviewed_accel_calibration=False)
        buffer = telemetry.TelemetrySnapshotBuffer(history_per_key=2, max_age_ns=max_age_ns,
                                                    max_spread_ns=max_spread_ns)
        failure, cursor, targets = None, 0, {}
        try:
            replay.warmup_policy(policy, torch_module, h, warmup_ticks)
            run.reset_run(timeline['first_tick_ns'], warmup_completed=True)
        except Exception as error:
            run.invalidate('Warmup/reset failed: '+type(error).__name__+': '+str(error))
            failure = {'phase': 'warmup_or_reset', 'tick_index': None, 'tick_ns': None, 'reason': run.failure}
        if failure is None:
            for index in range(max_ticks):
                tick = timeline['first_tick_ns']+index*observer.DT_NS
                while cursor < len(items) and items[cursor]['available_ns'] <= tick:
                    replay._ingest(buffer, items[cursor]); cursor += 1
                snapshot = buffer.snapshot(tick).as_dict()
                snapshot['source_flags'] = {'capture_identity_match_verified': True,
                    'fresh_identity_match_verified': False, 'original_timestamps_preserved': True,
                    'source': 'complete diagnose capture; original read completion availability',
                    'live_50hz_verified': False}
                reason = ('capture_exhausted' if tick > timeline['last_available_ns'] else
                          '; '.join(snapshot['blocked_reasons']))
                if reason:
                    run.invalidate(reason)
                else:
                    try:
                        result = consume_corrected(run, policy, snapshot)
                        result['observer_validated_inputs'] = result.pop('inputs')
                        result.update(policy.actual)
                        targets[tick] = (result['q_target_rad_diagnostic_only'],
                                         result['provenance']['snapshot_canonical_json_sha256'])
                        emit({'kind': 'offline_candidate_policy_tick', 'condition': condition, **result, **FLAGS})
                    except Exception as error:
                        reason = type(error).__name__+': '+str(error)
                        run.invalidate(reason)
                if reason:
                    failure = {'phase': 'replay', 'tick_index': index, 'tick_ns': tick,
                               'reason': reason, 'snapshot': snapshot}
                    emit({'kind': 'offline_candidate_policy_blocked', 'h_hypothesis': h,
                          'condition': condition, **failure, **FLAGS})
                    break
        observations[h, condition] = targets
        results.append({**run.finish(), 'condition': condition, 'first_blocked_tick': failure,
                        'warmup_ticks_requested': warmup_ticks,
                        'warmup_source': 'synthetic inputs, discarded by independent reset'})
    pairs = []
    for h in (0, 1):
        raw, corrected = observations[h, 'raw'], observations[h, 'corrected']
        common = sorted(set(raw) & set(corrected))
        differences = [[] for _ in range(12)]
        for tick in common:
            exact(raw[tick][1], corrected[tick][1], 'Paired original snapshot SHA')
            delta = [c-r for r, c in zip(raw[tick][0], corrected[tick][0])]
            for axis, value in zip(differences, delta): axis.append(value)
            emit({'kind': 'offline_candidate_target_difference', 'h_hypothesis': h,
                  'tick_ns': tick, 'snapshot_canonical_json_sha256': raw[tick][1],
                  'corrected_minus_raw_target_rad': delta, **FLAGS})
        pairs.append({'h_hypothesis': h, 'common_successful_ticks': len(common),
            'raw_successful_ticks': len(raw), 'corrected_successful_ticks': len(corrected),
            'common_tick_scope_only': True, 'can_order': list(shadow.CAN_ORDER),
            'target_delta_rms_rad_by_axis': [math.sqrt(math.fsum(x*x for x in row)/len(row))
                                            if row else None for row in differences],
            'target_delta_max_absolute_rad_by_axis': [max(map(abs, row)) if row else None for row in differences]})
    complete = all(row['status'] == 'COMPLETE_NO_OUTPUT_DIAGNOSTIC' for row in results)
    return {'schema': SCHEMA, 'status': ('OFFLINE_COMPARISON_COMPLETE_UNAPPROVED' if complete else
                                        'OFFLINE_COMPARISON_BLOCKED'), **FLAGS,
        'timeline': timeline, 'max_age_ns': max_age_ns, 'max_spread_ns': max_spread_ns,
        'limits_are_explicit_offline_conditions_only': True, 'command': list(command),
        'hypotheses': results, 'paired_comparisons': pairs,
        'limitations': ['Only complete standard diagnose captures are supported; no native/epoch capture substitution.',
            'An offline 20ms grid does not verify live execution, simultaneous acquisition or current-epoch freshness.',
            'Original q/dq/gyro and angle branches are retained. Out-of-range or unavailable sources block; no clipping or filling.',
            'Four model states and resets are independent. Warmup uses synthetic inputs only.',
            'The fixed norm candidate was fitted from prior first75% only; holdout and replay never refit it.',
            'Candidate, mount, gyro bias and h are unapproved hypotheses. Target differences do not prove physical accuracy or safe actuation.',
            'Paired metrics describe only common successful original ticks; every blocked condition is separately retained.']}


def write_json(path, value):
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        stream.flush(); os.fsync(stream.fileno())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('capture', 'calibration', 'bundle', 'imu-mount-candidate', 'output',
                 'accel-diagnostic-input', 'accel-diagnostic-report'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--gyro-bias-candidate', type=Path)
    parser.add_argument('--max-age-ms', type=replay._milliseconds, required=True)
    parser.add_argument('--max-spread-ms', type=replay._milliseconds, required=True)
    parser.add_argument('--max-ticks', type=int, required=True)
    parser.add_argument('--warmup-ticks', type=int, default=3)
    parser.add_argument('--command', type=float, nargs=3, default=[0., 0., 0.])
    args = parser.parse_args(argv)
    output, output_created, phase, pins = None, False, 'output', []
    try:
        need(not any(p.is_symlink() for p in (args.output.absolute(), *args.output.absolute().parents)),
             'Output symlink refused')
        output = replay._output_path(args.output)
        output.mkdir(mode=0o700, exist_ok=False)
        output_created = True
        phase = 'input_and_source_pins'
        pins = source_bindings()
        for path in (args.capture/'summary.json', args.capture/'events.jsonl', args.calibration,
                     args.imu_mount_candidate, args.accel_diagnostic_input, args.accel_diagnostic_report,
                     *([args.gyro_bias_candidate] if args.gyro_bias_candidate else [])):
            pins.append(read_file(path, FILE_MAX)[1])
        for name, expected in shadow.SOURCE_HASHES.items():
            pin = read_file(args.bundle/name, FILE_MAX)[1]
            exact(pin['sha256'], expected, 'Policy bundle SHA'); pins.append(pin)
        phase = 'capture_wire_validation'
        records, capture_source = shadow.load_capture(args.capture)
        verify_bindings(pins)
        phase = 'norm_candidate_recomputation'
        candidate, norm_pins = audit_norm_candidate(args.accel_diagnostic_input, args.accel_diagnostic_report)
        pins.extend(norm_pins)
        calibration = strict_json(read_file(args.calibration)[0])
        mount = shadow.validate_imu_mount_candidate(strict_json(read_file(args.imu_mount_candidate)[0]))
        bias = strict_json(read_file(args.gyro_bias_candidate)[0]) if args.gyro_bias_candidate else None
        no_accel_review(bias)
        replay.translate_records(records, calibration)
        need(type(args.max_ticks) is int and 1 <= args.max_ticks <= 30_000 and
             1 <= args.warmup_ticks <= 100, 'Finite tick/warmup caps required')
        verify_bindings(pins)
        write_json(output/'file-manifest.json', {'schema': SCHEMA+'.file-manifest',
            'scope': 'Exact files used for this offline comparison; no deployment or live approval',
            'bindings': pins, **FLAGS})
        phase = 'model_loading'
        policies, model_sources = zip(*(shadow.load_policy(args.bundle) for _ in range(4)))
        need(all(source['sha256'] == shadow.SOURCE_HASHES for source in model_sources), 'Model load binding mismatch')
        verify_bindings(pins)
        import torch
        phase = 'offline_replay'
        descriptor = os.open(output/'events.jsonl', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            def emit(row):
                stream.write(json.dumps(row, allow_nan=False)+'\n')
            try:
                result = compare_records(records, calibration, policies, candidate=candidate,
                    imu_mount_candidate=mount, max_ticks=args.max_ticks, max_age_ns=args.max_age_ms,
                    max_spread_ns=args.max_spread_ms, torch_module=torch, gyro_bias_candidate=bias,
                    command=args.command, warmup_ticks=args.warmup_ticks, emit=emit)
            finally:
                stream.flush(); os.fsync(stream.fileno())
        phase = 'final_file_binding_verification'
        verify_bindings(pins)
        result['provenance'] = {'capture': capture_source, 'models': list(model_sources),
            'torch_version': getattr(torch, '__version__', None), 'file_manifest_sha256': shadow.sha(output/'file-manifest.json'),
            'events_sha256': shadow.sha(output/'events.jsonl'), 'all_input_source_files_unchanged': True,
            'accel_candidate_recomputed_from_pinned_inputs': True}
        write_json(output/'summary.json', result)
        print(json.dumps({'output': str(output), 'status': result['status'], **FLAGS}))
        return 0 if result['status'] == 'OFFLINE_COMPARISON_COMPLETE_UNAPPROVED' else 2
    except (Exception, KeyboardInterrupt) as error:
        if output_created and not (output/'summary.json').exists():
            write_json(output/'summary.json', {'schema': SCHEMA, 'status': 'OFFLINE_COMPARISON_REJECTED',
                'failure_phase': phase, 'failure': type(error).__name__+': '+str(error),
                'partial_events_preserved': (output/'events.jsonl').exists(),
                'input_binding_verification_completed': False, 'bindings_observed': pins, **FLAGS})
        parser.exit(2, 'Offline comparison rejected: '+str(error)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
