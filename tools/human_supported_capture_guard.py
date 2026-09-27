"""File-only evidence gate for a new, fully human-supported raw-pose hold.

This module is copied into a frozen private package and used by both its
builder and execution wrapper. It imports no CAN or serial implementation.
"""

from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path


IDS = {str(i) for i in range(1, 13)}
PARAMETERS = ('position', 'velocity', 'current', 'voltage', 'run_mode')
PARAMETER_INDEX = {'position': 0x7019, 'velocity': 0x701b,
                   'current': 0x701a, 'voltage': 0x701c, 'run_mode': 0x7005}
REVIEWED_CAPTURE_SOURCE_SHA256 = 'e417c938dac703617383195550dfc0fa8196131bccb1436c8a356abc8f650577'
# The operator later clarified that both recordings were made on the center
# support stand. Neither may be relabeled as a stand-removed capture.
DISQUALIFIED_STAND_SUPPORTED_CAPTURE_SHA256 = {
    '347b681b1d00f1270f3f1a63ece5912aeea15f6ea65039f8e0c25232aa8d0c3e',
    'b43b279ee44d3d13f8b962863b77a8b44567d267f95fe7c08bc727e4665ccf60',
}
PHYSICAL_FLAGS = (
    'stand_removed_under_40v_off',
    'support_stand_absent_during_capture',
    'torso_fully_human_supported_during_capture',
    'two_operators_continuous_full_support',
    'four_paws_floor',
    'no_slip_sink_or_clamp_contact',
    'cutoff_operator_ready',
    'no_load_easing_planned',
    'full_support_through_stop_planned',
    'box_returned_40v_off_no_anomaly',
    'reviewed_for_two_second_human_supported_hold',
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), 'Missing or symlinked evidence: ' + str(path))
    return json.loads(path.read_text())


def validate_capture(summary_path, events_path, draft_path, source_path,
                     physical_path, *, boot, uids, can_readonly_sha256):
    """Reject incomplete/unstable Type0/17 capture and unreviewed physical state."""
    capture = read(summary_path)
    require(sha(summary_path) not in DISQUALIFIED_STAND_SUPPORTED_CAPTURE_SHA256,
            'Known stand-supported capture cannot qualify as human-supported')
    draft = read(draft_path)
    physical = read(physical_path)
    require(capture.get('schema') == 'singularitydog.fixed-stance-readonly-capture.v1'
            and capture.get('status') == 'RECORDED_REVIEW_REQUIRED'
            and capture.get('boot_id') == boot and capture.get('errors') == []
            and capture.get('output_allowed') is False
            and capture.get('approved_for_runtime') is False
            and capture.get('stop_state') == 'UNVERIFIED_BY_READ_ONLY_PROTOCOL'
            and capture.get('plan', {}).get('allowed_can_types') == [0, 17]
            and capture.get('plan', {}).get('motor_output_available') is False
            and capture.get('plan', {}).get('stop_command_available') is False
            and capture.get('plan', {}).get('sweeps') == 3
            and capture.get('source_sha256', {}).get('fixed_stance_readonly_capture.py')
                == sha(source_path) == REVIEWED_CAPTURE_SOURCE_SHA256
            and capture.get('source_sha256', {}).get('can_readonly.py') == can_readonly_sha256,
            'Capture is not a same-boot, reviewed motor-disabled Type0/17 recording')
    identities = capture.get('identities') or {}
    require(set(identities) == IDS
            and {mid: row.get('mcu_uid_hex') for mid, row in identities.items()} == uids
            and len(set(uids.values())) == 12,
            'Capture twelve motor identities differ from reviewed source')
    pose = capture.get('pose') or {}
    raw = pose.get('raw_rad_by_id') or {}
    samples = pose.get('samples') or {}
    enter_ns = capture.get('operator_enter_monotonic_ns')
    start_ns, end_ns = pose.get('started_monotonic_ns'), pose.get('ended_monotonic_ns')
    require(set(raw) == IDS and set(samples) == IDS
            and pose.get('sampling_issues') == []
            and pose.get('sampling_stability_heuristic_passed') is True
            and pose.get('stationarity_verified') is False
            and type(enter_ns) is int and type(start_ns) is int and type(end_ns) is int
            and 0 < enter_ns <= start_ns < end_ns
            and end_ns - start_ns <= 15_000_000_000,
            'Capture is incomplete or its strict sampling stability gate failed')
    for mid in IDS:
        center, rows = raw[mid], samples[mid]
        require(type(center) in (int, float) and math.isfinite(center)
                and abs(center) <= 12.57 and type(rows) is list and len(rows) == 3,
                f'ID{mid} capture center/sweeps invalid')
        positions = []
        for row in rows:
            require(type(row) is dict and set(row) == set(PARAMETERS),
                    f'ID{mid} missing Type17 parameter')
            values = {}
            for key in PARAMETERS:
                sample = row[key]
                require(type(sample) is dict and set(sample) ==
                        {'value', 'request_monotonic_ns', 'reply_monotonic_ns'}
                        and type(sample['request_monotonic_ns']) is int
                        and type(sample['reply_monotonic_ns']) is int
                        and enter_ns <= sample['request_monotonic_ns']
                            < sample['reply_monotonic_ns'] <= end_ns
                        and type(sample['value']) in (int, float)
                        and math.isfinite(sample['value']),
                        f'ID{mid} {key} stale or invalid')
                values[key] = sample['value']
            require(values['run_mode'] == 0 and abs(values['current']) <= .05
                    and abs(values['velocity']) <= .1
                    and 35. <= values['voltage'] <= 43.,
                    f'ID{mid} read-only electrical/motion envelope failed')
            positions.append(values['position'])
        require(max(positions) - min(positions) <= .02
                and min(positions) <= center <= max(positions),
                f'ID{mid} capture position span or median invalid')
    require(draft.get('schema') == 'singularitydog.fixed-stance-capture-draft.v1'
            and draft.get('boot_id') == boot and draft.get('motor_uids') == uids
            and draft.get('raw_rad_by_id') == raw
            and draft.get('stance_capture_sha256') == sha(summary_path)
            and draft.get('output_allowed') is False
            and draft.get('approved_for_runtime') is False,
            'Read-only draft differs from capture')
    require(physical.get('schema') == 'singularitydog.human-supported-physical-review.v1'
            and physical.get('boot_id') == boot
            and physical.get('capture_summary_sha256') == sha(summary_path)
            and physical.get('capture_events_sha256') == sha(events_path)
            and physical.get('capture_draft_sha256') == sha(draft_path)
            and all(physical.get(flag) is True for flag in PHYSICAL_FLAGS)
            and type(physical.get('operator_note')) is str
            and bool(physical['operator_note'].strip()),
            'Fresh full-human-support physical review is missing')
    events = Path(events_path)
    require(events.is_file() and not events.is_symlink() and events.stat().st_size > 0,
            'Capture events are missing')
    tx = Counter()
    rx = Counter()
    for line in events.read_text().splitlines():
        require(bool(line.strip()), 'Blank capture event')
        event = json.loads(line)
        kind = event.get('kind')
        if kind == 'can_tx':
            mid, parameter = event.get('motor_id'), event.get('parameter')
            require(type(mid) is int and str(mid) in IDS
                    and parameter in ('identity', *PARAMETERS)
                    and event.get('bus') == ('front' if mid <= 6 else 'rear'),
                    'Capture transmitted an unexpected CAN request')
            try:
                wire = bytes.fromhex(event['hex'])
            except (KeyError, TypeError, ValueError):
                raise ValueError('Capture CAN request wire is invalid') from None
            require(len(wire) == 17 and wire[:2] == b'AT' and wire[6] == 8
                    and wire[-2:] == b'\r\n',
                    'Capture CAN request frame is malformed')
            can_id = int.from_bytes(wire[2:6], 'big')
            request_type = (can_id >> 27) & 31
            require((can_id >> 3) & 255 == mid
                    and request_type == (0 if parameter == 'identity' else 17)
                    and wire[9:15] == bytes(6)
                    and (wire[7:9] == bytes(2) if parameter == 'identity' else
                         wire[7:9] == PARAMETER_INDEX[parameter].to_bytes(2, 'little')),
                    'Capture transmitted a nonread-only CAN request')
            tx[(mid, parameter)] += 1
        elif kind == 'can_rx_frame':
            require(event.get('type') in (0, 17),
                    'Capture received a non-Type0/17 frame')
            rx[event['type']] += 1
    require(all(tx[(mid, 'identity')] == 1 and
                all(tx[(mid, key)] == 3 for key in PARAMETERS)
                for mid in range(1, 13))
            and sum(tx.values()) == 192 and rx == {0: 12, 17: 180},
            'Capture Type0/17 event counts are incomplete')
    return raw


def verify_package_capture(base, review, manifest, boot):
    """Re-run the same gate after the frozen package hash/file-set check."""
    evidence = Path(base) / 'evidence'
    raw = validate_capture(
        evidence / 'human-capture-summary.json',
        evidence / 'human-capture-events.jsonl',
        evidence / 'human-capture-draft.json',
        evidence / 'human-capture-source.py',
        evidence / 'human-physical-review.json',
        boot=boot, uids=review['motor_uids'],
        can_readonly_sha256=manifest['singularitydog_hw/can_readonly.py'])
    require(review.get('sha256') == manifest['evidence/human-capture-summary.json']
            and review.get('supported_floor_start_raw_rad_by_id') == raw
            and review.get('human_capture_summary_sha256') ==
                manifest['evidence/human-capture-summary.json']
            and review.get('human_capture_events_sha256') ==
                manifest['evidence/human-capture-events.jsonl']
            and review.get('human_capture_draft_sha256') ==
                manifest['evidence/human-capture-draft.json']
            and review.get('human_physical_review_sha256') ==
                manifest['evidence/human-physical-review.json'],
            'Frozen review differs from human-supported capture')
    return raw


def verify_disabled_result(base, review, active_manifest, boot):
    """Check the frozen, new-pose disabled 2 s result before any active use."""
    evidence = Path(base) / 'evidence'
    disabled_manifest = read(evidence / 'human-disabled-manifest.json')
    summary = read(evidence / 'human-disabled-summary.json')
    events_path = evidence / 'human-disabled-events.jsonl'
    require(review.get('human_disabled_manifest_sha256') ==
                active_manifest['evidence/human-disabled-manifest.json']
            and review.get('human_disabled_summary_sha256') ==
                active_manifest['evidence/human-disabled-summary.json']
            and review.get('human_disabled_events_sha256') ==
                active_manifest['evidence/human-disabled-events.jsonl'],
            'New-pose disabled evidence hashes differ from frozen review')
    active_source = (Path(base) / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()
    require(active_source.count('LIVE_OUTPUT_ENABLED = True') == 1,
            'Active runtime source gate must occur exactly once')
    disabled_source = active_source.replace('LIVE_OUTPUT_ENABLED = True',
                                             'LIVE_OUTPUT_ENABLED = False')
    require(hashlib.sha256(disabled_source.encode()).hexdigest() ==
                disabled_manifest.get('singularitydog_hw/rs05_load_transfer_hold.py')
            and all(active_manifest.get('singularitydog_hw/' + name) ==
                    disabled_manifest.get('singularitydog_hw/' + name)
                    for name in ('__init__.py', 'bounded_pose_plan.py', 'can_readonly.py',
                                 'current_hold_review.py', 'position_response_evidence.py',
                                 'rs05_bus_transport.py', 'rs05_joint_trial.py',
                                 'rs05_leg_trial.py', 'rs05_trial_protocol.py')),
            'Disabled runtime differs from exact active runtime beyond source gate')
    result = summary.get('result') or {}
    workers = result.get('workers') or {}
    require(summary.get('boot_id') == boot
            and summary.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and summary.get('errors') == [] and summary.get('signals') == []
            and summary.get('motor_enable_sent') is False
            and summary.get('motion_gain_sent') is False
            and summary.get('trial_device_closed') is True
            and summary.get('locks_released') is True
            and summary.get('wrapper_sha256') ==
                disabled_manifest.get('prepared_load_transfer.py')
            and summary.get('events_sha256') == sha(events_path)
            and result.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and result.get('stop_confirmed') is True
            and result.get('review', {}).get('motor_uids') == review.get('motor_uids')
            and result.get('review', {}).get('supported_floor_start_raw_rad_by_id') ==
                review.get('supported_floor_start_raw_rad_by_id')
            and set(workers) == {'front', 'rear'}
            and all(workers[bus].get('cycle_count') == 25
                    and len(workers[bus].get('electrical_samples') or []) == 25
                    and len(workers[bus].get('stop_reports') or {}) == 6
                    and all(row.get('confirmed') is True for row in
                            workers[bus]['stop_reports'].values())
                    for bus in ('front', 'rear')),
            'Same-boot new-pose disabled 2 s preflight did not pass all12 STOP')
    return summary
