"""File-only periodic encoder branch screen; no hardware, model call or approval.

The nominal sign and zero come from a separately pinned candidate. A fresh quiet
capture selects exactly one integer-turn branch inside the unchanged model
interval. This is a numerical diagnostic, never a physical calibration claim.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import uuid

from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.angle_calibration_audit import resolve_unique_numeric_branch
from singularitydog_hw.can_timing_probe import validate_uids

SCHEMA = 'singularitydog.capture-bound-diagnostic-branch-derivation.v1'
IDS = {str(i) for i in range(1, 13)}


def need(value, message):
    if not value:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def pairs(items):
    result = {}
    for key, value in items:
        need(key not in result, 'Duplicate JSON key')
        result[key] = value
    return result


def read_pinned(path, expected):
    p = Path(path).expanduser()
    need(p.is_absolute() and p.is_file() and not p.is_symlink(), 'Pinned regular absolute file required')
    need(type(expected) is str and len(expected) == 64 and
         all(c in '0123456789abcdef' for c in expected), 'Canonical SHA256 required')
    need(0 < p.stat().st_size <= 64*1024*1024, 'Bounded nonempty JSON file required')
    raw = p.read_bytes()
    need(sha(raw) == expected, 'Pinned JSON changed: ' + p.name)
    value = json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
    need(type(value) is dict, 'JSON object required')
    return value, {'path': str(p), 'sha256': expected}


def derive_document(base, capture, expected_uids, *, base_pin, capture_pin,
                    uid_pin, boot_id, power_epoch):
    """Pure derivation, preserving inputs; caller binds real operator statements."""
    need(str(uuid.UUID(boot_id)) == boot_id, 'Canonical current boot UUID required')
    need(type(power_epoch) is str and power_epoch.strip() == power_epoch and
         0 < len(power_epoch) <= 1024 and power_epoch != 'NOT_INFERRED_FROM_JETSON_BOOT',
         'Explicit operator-declared new power epoch required')
    base_rows = shadow.validate_calibration(base)
    uids = {str(k): v for k, v in validate_uids(expected_uids).items()}
    need(base.get('approved_for_runtime') is False and
         base.get('motor_output_allowed') is False, 'Base must remain an unapproved no-output candidate')
    need(capture.get('schema') == 'singularitydog.readonly-12-angle-capture.v1' and
         capture.get('status') == 'RECORDED_REVIEW_REQUIRED' and capture.get('errors') == [] and
         capture.get('boot_id') == boot_id and capture.get('approved_for_runtime') is False and
         capture.get('motor_output_allowed') is False and capture.get('angle_wrap_applied') is False and
         capture.get('stop_state') == 'UNVERIFIED_BY_READ_ONLY_PROTOCOL',
         'Fresh unchanged quiet read-only capture required')
    need(capture.get('motor_power_epoch') in (power_epoch, 'NOT_INFERRED_FROM_JETSON_BOOT'),
         'Capture power label differs from the current explicit epoch')
    need(capture.get('expected_uids_sha256') == uid_pin['sha256'] and
         base['identities'] == uids, 'UID file and nominal calibration mismatch')
    identities = capture.get('identities', {})
    telemetry = capture.get('telemetry', {}).get('rows', {})
    need(set(identities) == set(telemetry) == IDS, 'All twelve fresh identities and axes required')
    result = copy.deepcopy(base)
    derived = {row['motor_id']: row for row in result['candidates']}
    audit, raw_by_id, q_by_id = {}, {}, {}
    bounds = {mid: (lo, hi) for mid, lo, hi in zip(shadow.CAN_ORDER, shadow.LOWER, shadow.UPPER)}
    for mid in range(1, 13):
        key = str(mid); row = telemetry[key]; identity = identities[key]
        need(identity.get('mcu_uid_hex') == uids[key], f'ID{mid} current UID mismatch')
        need(type(row.get('run_mode')) is int and row['run_mode'] == 0 and
             type(row.get('current')) in (int,float) and row['current'] == 0,
             f'ID{mid} capture was not quiet; no STOP proof inferred')
        samples = row.get('position_samples')
        need(type(samples) is list and len(samples) == 3, f'ID{mid} needs three position samples')
        values = [sample.get('rad') for sample in samples]
        need(all(type(v) in (int,float) and shadow.finite(v) for v in values),
             f'ID{mid} nonfinite or nonnumeric position')
        span_deg = math.degrees(max(values)-min(values))
        need(span_deg <= .1 and type(row.get('position_span_deg')) in (int,float) and
             shadow.finite(row['position_span_deg']) and
             abs(row['position_span_deg']-span_deg) <= 1e-9 and
             type(row.get('median_position_rad')) in (int,float) and
             row.get('median_position_rad') == statistics.median(values),
             f'ID{mid} position median/span inconsistent or not static')
        previous = identity.get('reply_monotonic_ns')
        start = identity.get('request_monotonic_ns')
        need(type(start) is int and type(previous) is int and 0 < start <= previous,
             f'ID{mid} UID causality invalid')
        for sample in samples:
            begin, end = sample.get('request_monotonic_ns'), sample.get('reply_monotonic_ns')
            need(type(begin) is int and type(end) is int and 0 < previous <= begin <= end and
                 end-begin <= 30_000_000, f'ID{mid} position sample causality invalid')
            previous = end
        nominal = base_rows[mid]; sign = nominal['sign_candidate']
        prior_turn = nominal.get('diagnostic_branch_turns_embedded_in_offset', 0)
        need(type(prior_turn) is int and abs(prior_turn) <= 20,
             f'ID{mid} prior embedded turn is invalid')
        base_offset = nominal['offset_candidate_rad'] + sign*prior_turn*2*math.pi
        raw = row['median_position_rad']; lower, upper = bounds[mid]
        try:
            branch = resolve_unique_numeric_branch(raw, sign=sign, offset_rad=base_offset,
                lower_rad=lower, upper_rad=upper, uncertainty_rad=0.)
        except ValueError as error:
            raise ValueError(f'ID{mid} no unique numeric branch: raw={raw!r}, sign={sign}, '
                f'base_offset={base_offset!r}, q_unadjusted={sign*raw+base_offset!r}, '
                f'model_limits=[{lower!r},{upper!r}]: {error}') from error
        turn = branch['turns']; need(abs(turn) <= 20, f'ID{mid} branch turn exceeds diagnostic bound')
        offset = base_offset-sign*turn*2*math.pi
        q = sign*raw+offset
        need(lower <= q <= upper, f'ID{mid} derived angle outside unchanged model limits')
        derived[mid].update(offset_candidate_rad=offset,
            diagnostic_branch_turns_embedded_in_offset=turn,
            physical_angle_accuracy_verified=False, sign_revalidated_for_runtime=False,
            approved_for_runtime=False)
        raw_by_id[key] = raw; q_by_id[key] = q
        audit[key] = dict(motor_id=mid, raw_rad=raw, sign=sign,
            previous_embedded_turn=prior_turn, previous_offset_rad=nominal['offset_candidate_rad'],
            nominal_zero_offset_rad=base_offset, selected_integer_turn=turn,
            derived_offset_rad=offset, unadjusted_model_rad=sign*raw+base_offset,
            derived_model_rad=q, unchanged_model_lower_rad=lower, unchanged_model_upper_rad=upper,
            numerical_selection_uncertainty_rad=0., absolute_calibration_error_rad=None,
            physical_branch_or_motion_proven=False, raw_angles_modified=False)
    need(all(derived[mid]['sign_candidate'] == base_rows[mid]['sign_candidate'] for mid in range(1,13)),
         'Sign changes are forbidden')
    result.update(candidate_subtype='UNIQUE_PERIODIC_BRANCH_NO_OUTPUT_DIAGNOSTIC_ONLY',
        source_current_boot_id=boot_id, source_capture_sha256=capture_pin['sha256'],
        source_current_motor_power_epoch_label=capture['motor_power_epoch'],
        source_capture_power_epoch_label_preserved=capture['motor_power_epoch'],
        source_raw_rad_by_id=raw_by_id, model_rad_at_source_capture_by_id=q_by_id,
        motor_power_epoch=power_epoch, operator_declared_motor_power_epoch=power_epoch,
        power_epoch_source='explicit operator argument; capture does not infer the power epoch',
        diagnostic_inference_only=True, epoch_binding_created=False,
        physical_joint_limits_verified=False, calibration_verified=False,
        cross_boot_angle_continuity_verified=False, motor_supply_off_on_evidence_complete=False,
        raw_angles_modified=False, current_capture_unchanged=True, live_50hz_verified=False,
        motor_targets_generated=False, approved_for_runtime=False, motor_output_available=False,
        motor_output_allowed=False, output_allowed=False,
        diagnostic_branch_derivation=dict(schema=SCHEMA, base_calibration=copy.deepcopy(base_pin),
            fresh_capture=copy.deepcopy(capture_pin), expected_uids=copy.deepcopy(uid_pin),
            boot_id=boot_id, operator_declared_motor_power_epoch=power_epoch,
            physical_statement_verified_by_helper=False, stop_state_not_verified_by_capture=True,
            scope='no_output_inference_only', model_intervals_unchanged=True,
            sign_and_nominal_zero_unchanged=True, raw_capture_unchanged=True, axes=audit))
    shadow.validate_calibration(result)
    return result


def derive_diagnostic_calibration(*, base_calibration_path, base_calibration_sha256,
        capture_path, capture_sha256, expected_uids_path, expected_uids_sha256,
        boot_id, power_epoch, output_path):
    base, base_pin = read_pinned(base_calibration_path, base_calibration_sha256)
    capture, capture_pin = read_pinned(capture_path, capture_sha256)
    expected, uid_pin = read_pinned(expected_uids_path, expected_uids_sha256)
    result = derive_document(base, capture, expected, base_pin=base_pin,
        capture_pin=capture_pin, uid_pin=uid_pin, boot_id=boot_id, power_epoch=power_epoch)
    out = Path(output_path).expanduser()
    need(out.is_absolute() and out.parent.is_dir() and not out.exists(), 'Fresh absolute output file required')
    need(all(not p.is_symlink() and not (p/'.git').exists() for p in (out.parent,*out.parent.parents)),
         'Symlink/Git output base is forbidden')
    raw = (json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
    with os.fdopen(os.open(out,os.O_WRONLY|os.O_CREAT|os.O_EXCL|getattr(os,'O_NOFOLLOW',0),0o600),'wb') as f:
        f.write(raw)
    return dict(path=str(out),sha256=sha(raw),
        turns_by_id={str(r['motor_id']):r['diagnostic_branch_turns_embedded_in_offset'] for r in result['candidates']},
        output_allowed=False,approved_for_runtime=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('base-calibration-path','base-calibration-sha256','capture-path','capture-sha256',
                 'expected-uids-path','expected-uids-sha256','boot-id','power-epoch','output-path'):
        parser.add_argument('--'+name,required=True)
    try:
        print(json.dumps(derive_diagnostic_calibration(**vars(parser.parse_args(argv))),ensure_ascii=False))
        return 0
    except (Exception,KeyboardInterrupt) as error:
        print(json.dumps(dict(status='ABORTED_FILE_ONLY_DERIVATION',errors=[type(error).__name__+': '+str(error)],
            output_allowed=False,approved_for_runtime=False,hardware_opened=False),ensure_ascii=False))
        return 2


if __name__=='__main__':
    raise SystemExit(main())
