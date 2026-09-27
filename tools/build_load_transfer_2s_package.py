"""Freeze current-boot floor-hold evidence into a local audit-only 2 s bundle."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys

from load_transfer_2s_launcher import (BOOT, EVIDENCE, FILE_NAMES, SCHEMA,
                                        SOURCES, _read, _sha, verify_package)

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'runtime' / 'singularitydog_hw'
IDS = tuple(range(1, 13))
BUS_IDS = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _private_new_directory(path):
    path = Path(path).expanduser()
    _require(not path.is_symlink() and not path.exists() and path.parent.is_dir(),
             'Use a fresh private output directory')
    output = path.resolve()
    _require(not any(parent.name == '.git' or (parent / '.git').exists()
                     for parent in output.parents), 'Private motor identities must stay outside Git')
    return output


def _validate_floor(summary, events_path, readonly):
    result = summary.get('result', {})
    review = result.get('review', {})
    uids = review.get('motor_uids')
    _require(type(uids) is dict and set(uids) == {str(i) for i in IDS}
             and all(type(uid) is str and len(uid) == 16
                     and all(c in '0123456789abcdef' for c in uid) for uid in uids.values())
             and len(set(uids.values())) == 12, 'Floor hold lacks exact unique 12 UIDs')
    _require(summary.get('boot_id') == BOOT
             and summary.get('status') == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
             and summary.get('errors') == [] and summary.get('signals') == []
             and summary.get('preflight_only') is False
             and summary.get('supported_current_hold_only') is True
             and summary.get('learned_policy_allowed') is False
             and summary.get('standing_allowed') is False
             and summary.get('motor_enable_sent') is True
             and summary.get('motion_gain_sent') is True
             and summary.get('trial_device_closed') is True
             and summary.get('locks_released') is True
             and summary.get('port_closes') == {'front': True, 'rear': True}
             and summary.get('events_sha256') == _sha(events_path)
             and result.get('status') == 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
             and result.get('errors') == [] and result.get('motion_completed') is True
             and result.get('stop_confirmed') is True
             and result.get('gain_profile') == 'id4-id10-kp4'
             and result.get('learned_policy_allowed') is False
             and result.get('standing_allowed') is False
             and result.get('l_target_replay_allowed') is False
             and result.get('automatic_retry') is False
             and review.get('boot_id') == BOOT and review.get('firmware') == '0.5.0.13',
             'Floor-contact supported 5 s hold is not a complete scoped success')
    centers = {}
    for bus, ids in BUS_IDS.items():
        worker = result.get('workers', {}).get(bus, {})
        expected = {str(mid) for mid in ids}
        stops = worker.get('stop_reports', {})
        _require(worker.get('completed') is True and worker.get('cycle_count') == 100
                 and set(worker.get('centers', {})) == expected
                 and set(stops) == expected
                 and all(stops[mid].get('confirmed') is True for mid in expected),
                 'Floor hold bus cycles or six STOP replies missing: ' + bus)
        centers.update(worker['centers'])
    _require(all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 12.57
                 for v in centers.values()), 'Floor hold raw centers malformed')
    _require(readonly.get('boot_id') == BOOT and readonly.get('status') == 'READ_ONLY_COMPLETE'
             and readonly.get('read_only') is True
             and readonly.get('motor_enable_sent') is False
             and readonly.get('errors') == []
             and type(readonly.get('motors')) is dict
             and set(readonly['motors']) == {str(mid) for mid in IDS}
             and all(row.get('uid_match') is True for row in readonly['motors'].values()),
             'Current-boot read-only identities do not corroborate all twelve')
    cycles, mode2, tx = Counter(), Counter(), Counter()
    peak_torque = {str(mid): 0. for mid in IDS}
    with Path(events_path).open() as stream:
        for line in stream:
            event = json.loads(line)
            bus = event.get('bus')
            if event.get('kind') == 'fullbody_cycle':
                _require(bus in BUS_IDS and event.get('preflight_only') is False,
                         'Floor hold cycle is malformed')
                cycles[bus, event.get('tick')] += 1
            elif event.get('kind') == 'bus_trial_feedback':
                mid = event.get('motor_id')
                _require(type(mid) is int and mid in BUS_IDS.get(bus, ())
                         and event.get('fault_bits') == 0
                         and event.get('mode_state') in (0, 2)
                         and all(type(event.get(key)) in (int, float)
                                 and math.isfinite(event[key]) for key in
                                 ('protocol_position_rad', 'velocity_rad_s',
                                  'torque_nm', 'temperature_c')),
                         'Floor hold Type2 feedback malformed or faulted')
                if event['mode_state'] == 2:
                    mode2[mid] += 1
                    peak_torque[str(mid)] = max(peak_torque[str(mid)], abs(event['torque_nm']))
            elif event.get('kind') == 'can_tx':
                _require(bus in BUS_IDS and type(event.get('motor_id')) is int
                         and event['motor_id'] in BUS_IDS[bus]
                         and event.get('type') in (0, 1, 3, 4, 17, 18),
                         'Floor hold TX type or bus differs from scoped protocol')
                tx[str(event['type'])] += 1
    _require(all(cycles[bus, tick] == 1 for bus in BUS_IDS for tick in range(100))
             and sum(cycles.values()) == 200
             and all(mode2[mid] >= 100 for mid in IDS)
             and dict(tx) == summary.get('typed_tx_counts'),
             'Floor hold event stream does not match 100 cycles/12 active feedbacks/TX counts')
    return uids, centers, peak_torque


def _validate_operator(report):
    _require(report.get('schema') == 'rs05-off-power-two-person-transfer-report-v1'
             and report.get('boot_id') == BOOT
             and report.get('power_40v_off') is True
             and report.get('off_power_transfer_rehearsal_reported') is True
             and report.get('stand_removed_and_replaced') is True
             and report.get('stand_restored') is True
             and report.get('four_paws_floor_contact') is True
             and report.get('two_operators_present') is True
             and report.get('no_slip_reported') is True
             and report.get('no_obstruction_reported') is True
             and type(report.get('source_note')) is str and bool(report['source_note'].strip()),
             'Two-person 40 V Off physical rehearsal report is incomplete')


def build(floor_summary, floor_events, readonly_summary, operator_report, output):
    output = _private_new_directory(output)
    inputs = tuple(Path(p) for p in (floor_summary, floor_events, readonly_summary, operator_report))
    _require(all(p.is_file() and not p.is_symlink() for p in inputs),
             'Every input must be an existing regular file, not a symlink')
    floor, readonly, operator = (_read(inputs[0]), _read(inputs[2]), _read(inputs[3]))
    uids, centers, peaks = _validate_floor(floor, inputs[1], readonly)
    _validate_operator(operator)
    source_paths = [RUNTIME / name for name in SOURCES]
    _require(all(p.is_file() and not p.is_symlink() for p in source_paths)
             and 'LIVE_OUTPUT_ENABLED = False' in (RUNTIME / 'rs05_load_transfer_hold.py').read_text(),
             'Runtime source set or disabled live gate missing')
    review = {
        'schema': 'rs05-load-transfer-current-hold-review-v1',
        'scope': 'supervised-load-transfer-current-position-2-15s',
        'boot_id': BOOT, 'motor_ids': list(IDS), 'motor_uids': uids,
        'reviewed_motor_ids': [5, 6, 8], 'motor_reviews': floor['result']['review']['motor_reviews'],
        'firmware': '0.5.0.13', 'sha256': _sha(inputs[0]),
        'duration_s': 2., 'gain_profile': 'id4-id10-kp4',
        'supported_floor_start_raw_rad_by_id': centers,
        'supported_floor_start_tolerance_rad': math.radians(3.),
        'feedback_torque_abort_nm_by_motor': {str(mid): 1.5 for mid in IDS},
        'floor_hold_summary_sha256': _sha(inputs[0]),
        'floor_hold_events_sha256': _sha(inputs[1]),
        'readonly_summary_sha256': _sha(inputs[2]),
        'operator_rehearsal_sha256': _sha(inputs[3]),
        'supported_floor_type2_peak_abs_torque_nm_by_motor': peaks,
        'source_files_verified': True,
        'supported_stance_passed': True,
        'off_power_transfer_rehearsal_reported': True,
        # The user report confirms an Off-power rehearsal, not a timed catch
        # at STOP or a measured loaded motor envelope. Keep every live gate off.
        'off_power_transfer_rehearsal_reviewed': False,
        'physical_catch_reviewed': False,
        'power_cutoff_operator_reviewed': False,
        'load_specific_limits_reviewed': False,
        'review_complete': False, 'load_transfer_hold_authorized': False,
        'calibration_verified': False, 'learned_policy_allowed': False,
        'standing_allowed': False, 'l_target_replay_allowed': False,
        'automatic_retry_allowed': False,
        'serial_write_timeout_verified': False,
        'two_usb_50ms_workload_verified': False,
        'physical_torque_cap_verified': False,
    }
    os.umask(0o077)
    output.mkdir(mode=0o700)
    (output / 'source').mkdir(mode=0o700)
    (output / 'evidence').mkdir(mode=0o700)
    for source in source_paths:
        shutil.copyfile(source, output / 'source' / source.name)
    for name, path in zip(EVIDENCE, inputs):
        shutil.copyfile(path, output / 'evidence' / name)
    shutil.copyfile(ROOT / 'tools' / 'load_transfer_2s_launcher.py', output / 'launcher.py')
    (output / 'review.json').write_text(json.dumps(review, indent=2, allow_nan=False) + '\n')
    manifest = {'schema': SCHEMA, 'boot_id': BOOT, 'duration_s': 2.,
                'file_sha256': {name: _sha(output / name) for name in sorted(FILE_NAMES)}}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    for path in output.rglob('*'):
        if path.is_file():
            path.chmod(0o600)
    manifest_sha = _sha(output / 'manifest.json')
    verified = verify_package(output, manifest_sha, BOOT)
    return {**verified, 'package': str(output),
            'missing_live_evidence': ['two_usb_50ms_workload', 'timed_STOP_catch',
                                      'loaded_torque_margin', 'bounded_serial_write_timeout',
                                      'independent_live_wrapper_review']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--floor-summary', type=Path, required=True)
    parser.add_argument('--floor-events', type=Path, required=True)
    parser.add_argument('--readonly-summary', type=Path, required=True)
    parser.add_argument('--operator-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.floor_summary, args.floor_events,
                           args.readonly_summary, args.operator_report,
                           args.output), indent=2), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
