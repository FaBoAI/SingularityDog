"""Read-only, same-boot evidence board for the bundled pre-stand tests.

The five-second all-axis hold already checks the twelve motor identities,
initial STOP, disabled parameters and voltage, watchdog readback, fault-free
feedback, active hold, and final STOP. This tool reports those checks from the
frozen hold evidence instead of requesting a redundant motor read. It never
opens a port, transfers a package, or executes a frozen launcher.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import front_hip_test_session as session


SCHEMA = 'singularitydog.prestand-batch-status.v1'
PASS, MISSING, FAIL = 'PASS', 'MISSING', 'FAIL'
BUSES = {'front': range(1, 7), 'rear': range(7, 13)}
EXPECTED_IDS = {str(mid) for mid in range(1, 13)}


def row(state: str, detail: str, **extra) -> dict:
    return {'state': state, 'detail': detail, **extra}


def _read(path: Path) -> dict:
    value = json.loads(path.read_text())
    if type(value) is not dict:
        raise ValueError(f'Expected JSON object: {path}')
    return value


def _fault_and_stop(hold: dict) -> tuple[dict, dict]:
    workers = hold['result']['workers']
    seen = {'initial': {}, 'final': {}}
    for bus, ids in BUSES.items():
        worker = workers[bus]
        for stage, key in (('initial', 'initial_stop'), ('final', 'stop_reports')):
            values = worker.get(key)
            if type(values) is not dict or set(values) != {str(mid) for mid in ids}:
                return (row(MISSING, f'ID {bus} {stage} STOP evidence is incomplete'),
                        row(MISSING, f'ID {bus} {stage} feedback is incomplete'))
            seen[stage].update(values)
    stop_ok = (hold['result'].get('stop_confirmed') is True
               and all(type(item) is dict and item.get('confirmed') is True
                       for values in seen.values() for item in values.values()))
    fault_values = [item.get('feedback', {}).get('fault_bits')
                    if type(item) is dict and type(item.get('feedback')) is dict else None
                    for values in seen.values() for item in values.values()]
    if any(value is None for value in fault_values):
        faults = row(MISSING, 'Initial/final STOP feedback does not include all fault bits')
    elif all(type(value) is int and value == 0 for value in fault_values):
        faults = row(PASS, 'Initial and final STOP feedback: fault bits zero on all 12 axes')
    else:
        faults = row(FAIL, 'Nonzero or malformed fault bits in STOP feedback')
    stops = row(PASS if stop_ok else FAIL,
                'Initial and final STOP confirmed on all 12 axes' if stop_ok else
                'One or more initial/final STOP confirmations failed')
    return stops, faults


def _voltage(hold: dict) -> dict:
    values = {}
    for bus, ids in BUSES.items():
        motors = hold['result']['workers'][bus].get('motors')
        if type(motors) is not dict or set(motors) != {str(mid) for mid in ids}:
            return row(MISSING, f'ID {bus} disabled motor parameter records are incomplete')
        for mid in ids:
            record = motors[str(mid)]
            if type(record) is not dict or 'voltage' not in record:
                return row(MISSING, f'ID{mid} voltage record is missing')
            value = record['voltage']
            if type(value) not in (int, float) or not math.isfinite(value):
                return row(FAIL, f'ID{mid} voltage is invalid')
            values[str(mid)] = value
    low, high = min(values.values()), max(values.values())
    bad = [mid for mid, value in values.items() if not 35 <= value <= 43]
    return row(FAIL if bad else PASS,
               f'12 disabled voltage readings: {low:.2f}–{high:.2f} V; '
               + (f'out of 35–43 V: IDs {", ".join(bad)}' if bad else 'inside 35–43 V'),
               minimum_v=low, maximum_v=high)


def _hold_bundle(disabled: Path, boot: str) -> tuple[dict, dict, dict, dict, dict]:
    hold_path = disabled / 'current-hold-summary.json'
    if not hold_path.is_file():
        missing = row(MISSING, 'Frozen five-second all-axis hold summary is missing')
        return missing, missing, missing, missing, missing
    try:
        hold = _read(hold_path)
        review = _read(disabled / 'step2-review.json')
        uids = review['motor_uids']
        if type(uids) is not dict or set(uids) != EXPECTED_IDS:
            raise ValueError('Frozen review does not name all twelve UIDs')
        session.role.disabled_base.validate_hold(hold, session.digest(hold_path), boot, uids)
    except (OSError, ValueError, TypeError, KeyError) as error:
        failed = row(FAIL, f'Five-second same-boot hold validation failed: {error}')
        return failed, failed, failed, failed, failed
    uid = row(PASS, 'All 12 IDs were compared with reviewed UIDs by the completed hold; '
              'firmware version is a review declaration, not a live firmware query',
              declared_firmware=review.get('firmware'), firmware_live_verified=False)
    voltage = _voltage(hold)
    stops, faults = _fault_and_stop(hold)
    states = {voltage['state'], faults['state'], stops['state']}
    composite = FAIL if FAIL in states else MISSING if MISSING in states else PASS
    hold_row = row(composite, 'Both six-axis workers completed 100 cycles (5 s); '
                   'the composite also requires recorded voltage, zero faults, and initial/final STOP',
                   nominal_duration_s=5.0)
    return hold_row, uid, voltage, faults, stops


def _preflight(root: Path) -> dict:
    receipt = root / 'preflight-receipt.json'
    if not receipt.is_file():
        return row(MISSING, 'Continuous front-hip disabled preflight has no imported receipt')
    try:
        ledger, saved, _, preflight = session.check_preflight(root)
        if saved.get('preflight_passed') is not True:
            raise ValueError('Preflight receipt does not report a pass')
        centers = session.role.front_hip_step10_preflight_centers(preflight / 'summary.json')
        if set(centers) != EXPECTED_IDS:
            raise ValueError('Preflight does not have all twelve start angles')
        if ledger.get('continuous_profile') != session.role.CONTINUOUS_FRONT_HIP_PROFILE:
            raise ValueError('Session is not the continuous 1 s / 5° / 10° profile')
    except (OSError, ValueError, TypeError, KeyError) as error:
        return row(FAIL, f'Preflight receipt or same-boot evidence failed: {error}')
    return row(PASS, 'Disabled preflight, 12 start angles, and STOP verified in the same boot')


def _active_package(root: Path, boot: str, preflight_state: str) -> dict:
    receipt = root / 'active-receipt.json'
    if not receipt.is_file():
        return row(MISSING, 'Reviewed continuous active package has not been frozen')
    if preflight_state != PASS:
        return row(FAIL, 'Active package exists without a valid same-boot preflight')
    try:
        status = session.status(root)
        if status.get('boot_id') != boot or status.get('active_package_ready') is not True:
            raise ValueError('Active package is not verified for this boot')
        active = root / session.ACTIVE
        review = _read(active / 'step2-active-review.json')
        physical = _read(active / 'physical-review.json')
        if (review.get('continuous_profile') != session.role.CONTINUOUS_FRONT_HIP_PROFILE
                or review.get('continuous_waypoints_deg') != [5.0, 10.0]
                or physical.get('continuous_19s_reviewed') is not True
                or physical.get('boot_id') != boot
                or not all(physical.get(flag) is True for flag in session.role.FLAGS)):
            raise ValueError('19-second continuous route lacks exact physical review')
    except (OSError, ValueError, TypeError, KeyError) as error:
        return row(FAIL, f'Active package validation failed: {error}')
    return row(PASS, 'Frozen active package and 19-second physical clearance review verified; '
               'this is preparation, not a completed motor trial')


def _trial(root: Path, boot: str, active_state: str) -> dict:
    attempts = root / 'attempts'
    if not attempts.is_dir() or not any(attempts.iterdir()):
        return row(MISSING, 'No imported continuous 5°→10° motor-trial result')
    if active_state != PASS:
        return row(FAIL, 'Trial files exist without a verified active package')
    try:
        session.status(root)  # Recheck every attempt's immutable summary/events hashes.
        receipts = []
        active = root / session.ACTIVE
        active_review = _read(active / 'step2-active-review.json')
        wrapper_sha = session.digest(active / 'prepared_current_hold.py')
        for item in sorted(attempts.iterdir()):
            if not item.is_dir() or item.is_symlink():
                raise ValueError('Attempt directory is malformed')
            receipt = _read(item / 'receipt.json')
            if receipt.get('boot_id') != boot:
                raise ValueError(f'{item.name} belongs to another boot')
            summary_path, events_path = item / 'summary.json', item / 'events.jsonl'
            summary = _read(summary_path)
            result = summary.get('result', {})
            if type(result) is not dict:
                raise ValueError(f'{item.name} result is malformed')
            workers = result.get('workers', {})
            all_stops = (type(workers) is dict and set(workers) == set(BUSES)
                         and all(type(workers[bus]) is dict
                                 and type(workers[bus].get('stop_reports')) is dict
                                 and set(workers[bus]['stop_reports'])
                                     == {str(mid) for mid in ids}
                                 and all(type(workers[bus]['stop_reports'][str(mid)]) is dict
                                         and workers[bus]['stop_reports'][str(mid)]
                                             .get('confirmed') is True for mid in ids)
                                 for bus, ids in BUSES.items()))
            actual_stop = result.get('stop_confirmed') is True and all_stops
            if (receipt.get('summary_sha256') != session.digest(summary_path)
                    or receipt.get('events_sha256') != session.digest(events_path)
                    or summary.get('boot_id') != boot
                    or summary.get('wrapper_sha256') != wrapper_sha
                    or summary.get('events_sha256') != session.digest(events_path)
                    or (result.get('review') != active_review
                        and not (summary.get('status') == 'INCOMPLETE'
                                 and 'review' not in result))
                    or receipt.get('status') != summary.get('status')
                    or receipt.get('motion_completed')
                        != (result.get('motion_completed') is True)
                    or receipt.get('all_twelve_stop_confirmed') != actual_stop
                    or receipt.get('errors') != summary.get('errors', [])):
                raise ValueError(f'{item.name} receipt does not match frozen trial evidence')
            receipts.append((item.name, receipt))
        if any(r.get('all_twelve_stop_confirmed') is not True for _, r in receipts):
            return row(FAIL, 'At least one attempt lacks all-12 STOP confirmation; '
                       'inspect the robot before another command',
                       attempts=[name for name, _ in receipts])
        name, latest = receipts[-1]
        passed = (latest.get('status') == 'RAW_FRONT_HIP_CONTINUOUS_COMPLETED_RESET_CONFIRMED'
                  and latest.get('motion_completed') is True
                  and latest.get('continuous_profile')
                      == session.role.CONTINUOUS_FRONT_HIP_PROFILE
                  and latest.get('errors') == [])
    except (OSError, ValueError, TypeError, KeyError) as error:
        return row(FAIL, f'Continuous trial result validation failed: {error}')
    return row(PASS if passed else FAIL,
               f'{name}: 5°→10° completed with all-12 STOP' if passed else
               f'{name}: STOP confirmed but continuous movement did not complete',
               attempts=[item for item, _ in receipts], latest_attempt=name)


def collect(root: Path) -> dict:
    checks = {}
    ledger_path = root / 'session.json'
    if not ledger_path.is_file():
        missing = row(MISSING, 'No same-boot front-hip session receipt')
        checks = {name: missing for name in ('session', 'hold', 'uid', 'voltage',
                                            'fault', 'stop', 'preflight',
                                            'active_package', 'continuous_trial')}
        return {'schema': SCHEMA, 'boot_id': None, 'checks': checks,
                'standing_authorized': False,
                'outside_this_board': ['remaining joint-group route tests',
                                       'IMU/model-angle calibration',
                                       'fixed-standing trajectory and policy timing'],
                'next': 'Create a same-boot session from the completed all-axis hold'}
    try:
        ledger, disabled, _ = session.check_session(root)
    except (OSError, ValueError, TypeError, KeyError) as error:
        failed = row(FAIL, f'Session integrity failed: {error}')
        checks = {name: failed for name in ('session', 'hold', 'uid', 'voltage',
                                           'fault', 'stop', 'preflight',
                                           'active_package', 'continuous_trial')}
        return {'schema': SCHEMA, 'boot_id': None, 'checks': checks,
                'standing_authorized': False,
                'outside_this_board': ['remaining joint-group route tests',
                                       'IMU/model-angle calibration',
                                       'fixed-standing trajectory and policy timing'],
                'next': 'Inspect the frozen session and evidence before any test'}
    boot = ledger['boot_id']
    checks['session'] = row(PASS, 'Frozen session and boot-provenance receipts match')
    checks['hold'], checks['uid'], checks['voltage'], checks['fault'], checks['stop'] = (
        _hold_bundle(disabled, boot))
    checks['preflight'] = _preflight(root)
    checks['active_package'] = _active_package(root, boot, checks['preflight']['state'])
    checks['continuous_trial'] = _trial(root, boot, checks['active_package']['state'])
    if checks['stop']['state'] == FAIL or (
            checks['continuous_trial']['state'] == FAIL
            and 'STOP' in checks['continuous_trial']['detail']):
        next_step = 'STOP or fault evidence is not clean; inspect the robot and logs'
    else:
        next_step = next((f'{name}: {value["detail"]}' for name, value in checks.items()
                          if value['state'] != PASS),
                         'Front-hip checks complete; review the remaining standing-route tests')
    return {'schema': SCHEMA, 'boot_id': boot, 'checks': checks,
            'standing_authorized': False,
            'outside_this_board': ['remaining joint-group route tests',
                                   'IMU/model-angle calibration',
                                   'fixed-standing trajectory and policy timing'],
            'next': next_step}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path,
                        help='Local same-boot directory made by front_hip_test_session.py')
    parser.add_argument('--table', action='store_true', help='Print a compact human table')
    args = parser.parse_args(argv)
    report = collect(args.session)
    if args.table:
        print(f'boot_id: {report["boot_id"] or "unavailable"}')
        for name, value in report['checks'].items():
            print(f'{name:19} {value["state"]:7} {value["detail"]}')
        print(f'next: {report["next"]}')
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 1 if any(value['state'] == FAIL for value in report['checks'].values()) else 0


if __name__ == '__main__':
    raise SystemExit(main())
