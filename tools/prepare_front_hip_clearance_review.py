"""Create an unauthorised front-hip 10-degree physical-review draft.

The twelve reference angles and summary hash come from one exact disabled
preflight. No motor is opened, and all physical approval flags remain false.
An operator must inspect the full 10-degree path plus a +/-3-degree start
envelope and complete the draft before an active package can be built.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import build_role_group_step as role


def prepare(disabled_package: Path, preflight_log: Path, output: Path) -> dict:
    role.active_base.validate_frozen(disabled_package, 'prepared_fullbody.py', 'manifest.json')
    role.require(not output.exists() and output.parent.is_dir(),
                 'Use a fresh physical-review draft path')
    review = role.read_json(disabled_package / 'step2-review.json')
    role.require(review.get('role_group') == 'front-hip'
                 and review.get('amplitude_deg') == 10.
                 and review.get('direction_profile') == 'front-hip-mirrored',
                 'Only the reviewed front-hip 10-degree disabled package is supported')
    summary_path = preflight_log / 'summary.json'
    events_path = preflight_log / 'events.jsonl'
    role.validate_role_group_preflight(disabled_package, summary_path, events_path,
                                       review['boot_id'])
    reference = role.front_hip_step10_preflight_centers(summary_path)
    draft = {
        'schema': 'singularitydog.role-group-step1-physical-review.v1',
        'scope': 'supported-front-hip-current-raw-mirrored-10deg-diagnostic-only',
        'boot_id': review['boot_id'],
        'motor_uids': review['motor_uids'],
        'role_group': 'front-hip',
        'moving_motor_ids': [3, 6],
        'direction_profile': 'front-hip-mirrored',
        'raw_direction_by_id': review['raw_direction_by_id'],
        'source_disabled_review_sha256': role.sha(disabled_package / 'step2-review.json'),
        'clearance_reference_preflight_summary_sha256': role.sha(summary_path),
        'clearance_reference_raw_rad_by_id': reference,
        'start_tolerance_clearance_verified_deg': role.CLEARANCE_START_TOLERANCE_DEG,
        'start_tolerance_clearance_note': '',
        'requires_fresh_pre_run_confirmation': True,
        'old_raw_target_reused': False,
        'calibration_verified': False,
        'model_mapping_verified': False,
        'standing_allowed': False,
        'learned_policy_allowed': False,
        'automatic_retry_allowed': False,
        **{flag: False for flag in role.FLAGS},
        'confirmation_note': '',
    }
    role.write_json(output, draft)
    return {'draft': str(output), 'boot_id': review['boot_id'],
            'reference_axes': len(reference), 'preflight_summary_sha256': role.sha(summary_path),
            'physical_approval_complete': False, 'active_output_authorized': False}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--disabled-package', type=Path, required=True)
    parser.add_argument('--preflight-log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(prepare(args.disabled_package, args.preflight_log, args.output), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
