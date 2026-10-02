"""Offline ground-stage evidence review. This module has no device or network API.

A hash records provenance, not physical truth. PASS requires named human review
of a synchronized source video as well as the raw native CAN records. No contact,
load percentage, travelled distance, or true sensor time is inferred from torque,
commands, host timestamps, or a successful software return code.
"""
import argparse
from bisect import bisect_right
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path

from . import can_readonly as codec
from . import rs05_trial_protocol as protocol
from .motor_version_probe import version_request, decode_version
from .native_feedback_compare import _frame
from .native_active_transport import encode_motion
from .policy_observer import _mount, _bias
from .policy_shadow import _json
from .policy_live_profile import SCHEMA_V3, telemetry_settings

SCHEMA = 'singularitydog.ground-trial-evaluation.v1'
ASSOCIATION_SCHEMA = 'singularitydog.ground-video-association.v1'
REVIEW_SCHEMA = 'singularitydog.ground-physical-review.v1'
STAGES = ('supported_stance', 'partial_load', 'stand', 'walk')
IDS = tuple(str(i) for i in range(1, 13))
BUSES = {'front': tuple(range(1, 7)), 'rear': tuple(range(7, 13))}
V3_VOLTAGE_MAX_AGE_NS = 126_000_000


class EvidenceError(ValueError):
    pass


def need(ok, message):
    if not ok:
        raise EvidenceError(message)


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def document(raw):
    need(type(raw) is bytes, 'Evidence must be the original file bytes')
    value = _json(raw.decode('utf-8'))
    need(type(value) is dict, 'Evidence JSON must be an object')
    return value


def vector(value, count, name):
    need(type(value) is list and len(value) == count and all(finite(x) for x in value),
         'Nonfinite/missing '+name)
    return value


def is_hash(value):
    return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def stamp(value):
    need(type(value) is str, 'Missing review timestamp')
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as error:
        raise EvidenceError('Invalid review timestamp') from error
    need(parsed.utcoffset() is not None, 'Review timestamp must include timezone')


def _record(row, bus):
    need(type(row) is dict and bus in BUSES, 'Malformed native record')
    names = ('start_ns', 'finish_ns', 'deadline_ns')
    need(all(type(row.get(n)) is int and row[n] > 0 for n in names), 'Missing native request times')
    need(row['start_ns'] <= row['finish_ns'] < row['deadline_ns'] and row.get('written') == 17,
         'Incomplete native write or deadline violation')
    tx = _frame(row.get('tx_hex'))
    need(tx.destination in BUSES[bus], 'Cross-bus request')
    need(all(type(row.get(n)) is int and row[n] > 0 for n in ('read_start_ns', 'received_ns')),
         'Missing receive times')
    need(row['finish_ns'] <= row['read_start_ns'] <= row['received_ns'] < row['deadline_ns']
         and row.get('received') == 17, 'Incomplete or noncausal native response')
    return tx, _frame(row.get('rx_hex'))


def _journal(runtime, profile, *, tested_firmware=None):
    journal = runtime.get('journal')
    need(type(journal) is list and journal, 'Missing raw native journal')
    identities = {}; voltages = []; feedback = []; mode0 = set(); watchdog = set(); versions = {}
    version_ends = []; output_starts = []
    timeout_writes = {}; initial_timeouts = {}; final_timeouts = {}; final_pose = {}
    voltage_refresh = {}; voltage_refresh_batches = set()
    expected_cadence = telemetry_settings(profile)
    v3 = profile['schema'] == SCHEMA_V3
    if v3:
        need(runtime.get('telemetry_cadence') == expected_cadence, 'Runtime cadence differs from reviewed profile')
        need(runtime.get('cadence_source_sha256') == profile['cadence_source_sha256'],
             'Runtime cadence source pins differ from reviewed profile')
    tested_firmware = tested_firmware if tested_firmware is not None else profile.get('watchdog_by_id')
    need(type(tested_firmware) is dict and set(tested_firmware) == set(IDS), 'Missing twelve tested firmware fingerprints')
    for batch in journal:
        need(batch.get('error') is None and batch.get('rejected_total', 0) == 0
             and not batch.get('rejected_hex'), 'CAN exchange had an error/rejected response')
        bus = batch.get('bus'); records = batch.get('records')
        need(type(records) is list and records, 'Empty native exchange')
        if v3 and batch.get('phase') == 'voltage_pre_enable_refresh':
            need(bus in BUSES and bus not in voltage_refresh_batches and len(records) == len(BUSES[bus]),
                 'V3 pre-enable voltage refresh batch missing/duplicated')
            voltage_refresh_batches.add(bus)
        for row in records:
            tx = _frame(row.get('tx_hex')); mid = tx.destination
            tx, rx = _record(row, bus)
            if tx.kind == 18:
                # The only setting written by this runner is the canonical watchdog.
                need(tx.wire == protocol.watchdog_setup_request(
                    phase=protocol.TrialPhase.WATCHDOG_SETUP,motor_id=mid), 'Unexpected configuration write')
                value = protocol.decode_type2(rx,motor_id=mid)
                need(value.mode_state == 0 and value.fault_bits == 0,
                     'Watchdog setup acknowledgement must be stopped and fault-free')
                need(mid not in timeout_writes, 'Duplicate watchdog setup')
                timeout_writes[mid] = row
                continue
            if tx.kind == 0:
                value = codec.decode_reply(rx, mid, None)['mcu_uid_hex']
                need(value == profile['axes'][str(mid)]['uid'], 'UID mismatch')
                need(mid not in identities, 'Duplicate identity capture')
                identities[mid] = value
            elif tx.kind == 17:
                name = next((n for n, spec in codec.PARAMETERS.items()
                             if spec[0] == int.from_bytes(tx.data[:2], 'little')), None)
                need(name is not None and tx.wire == codec.read_request(mid, name), 'Unknown parameter read')
                value = codec.decode_reply(rx, mid, name)
                need(value['ok'], 'Parameter read failed')
                if name == 'voltage':
                    need(profile['voltage_min_v'] <= value['value'] <= profile['voltage_max_v'], 'Voltage outside profile')
                    voltages.append((mid, row['received_ns'], value['value']))
                    if v3 and batch.get('phase') == 'voltage_pre_enable_refresh':
                        need(mid not in voltage_refresh, 'Duplicate V3 pre-enable voltage refresh')
                        voltage_refresh[mid] = (row, value['value'])
                elif v3 and batch.get('phase') == 'voltage_pre_enable_refresh':
                    need(False, 'V3 pre-enable voltage refresh contains another parameter')
                elif name == 'run_mode':
                    need(value['value'] == 0, 'Mode 0 was not confirmed')
                    mode0.add(mid)
                elif name == 'can_timeout':
                    need(value['value'] == protocol.WATCHDOG_TICKS, 'Device watchdog readback mismatch')
                    need(mid in timeout_writes and timeout_writes[mid]['received_ns'] < row['start_ns'],
                         'Watchdog readback preceded setup acknowledgement')
                    watchdog.add(mid)
                    if v3:
                        phase = batch.get('phase')
                        need(phase in ('watchdog_initial_readback','watchdog_pre_enable_readback'),
                             'V3 unexpected timeout-parameter readback phase')
                        target = initial_timeouts if phase == 'watchdog_initial_readback' else final_timeouts
                        need(mid not in target, 'Duplicate watchdog readback within phase')
                        target[mid] = row
            elif tx.kind == 4 and tx.wire == version_request(mid):
                value = decode_version(rx, mid)
                need(mid not in versions, 'Duplicate firmware version capture')
                need(value['version_bytes_hex'] == tested_firmware[str(mid)].get('version_bytes_hex'),
                     'Fresh firmware differs from tested watchdog firmware')
                versions[mid] = value['version_bytes_hex']
                version_ends.append(row['received_ns'])
            else:
                need(not (v3 and batch.get('phase') == 'voltage_pre_enable_refresh'),
                     'V3 pre-enable voltage refresh contains a non-voltage request')
                need(tx.kind in (1, 3, 4), 'Unexpected command type')
                if tx.kind in (1, 3): output_starts.append(row['start_ns'])
                value = protocol.decode_type2(rx, motor_id=mid)
                need(value.fault_bits == 0, 'QDD fault bits are nonzero')
                need(value.mode_state in ((2,) if tx.kind == 1 else (0,) if tx.kind == 4 else (0, 2)),
                     'QDD mode state mismatch')
                if v3 and batch.get('phase') == 'pre_enable_pose':
                    need(tx.kind == 4 and mid not in final_pose, 'Invalid final pre-enable pose capture')
                    final_pose[mid] = row
                a = profile['axes'][str(mid)]
                need(abs(value.torque_nm) <= a['max_measured_torque_nm']
                     and abs(value.velocity_rad_s) <= a['max_measured_velocity_rad_s']
                     and value.temperature_c <= a['max_temperature_c'], 'QDD physical telemetry limit')
                feedback.append({'mid': mid, 'value': value, 'tx': tx, 'row': row,
                                 'phase': batch.get('phase'), 'bus': bus})
    expected = set(range(1, 13))
    need(set(identities) == mode0 == watchdog == set(timeout_writes) == set(versions) == expected,
         'Incomplete UID/mode/watchdog/firmware preflight')
    need({mid for mid, _, _ in voltages} == expected, 'Missing voltage evidence for one or more axes')
    need(output_starts and max(version_ends) < min(output_starts), 'Firmware check did not precede enable/targets')
    need(max(r['received_ns'] for r in timeout_writes.values()) < min(output_starts),
         'Watchdog setup acknowledgement did not precede enable/targets')
    if v3:
        need(set(timeout_writes) == set(initial_timeouts) == set(final_timeouts) == set(final_pose) == expected,
             'Incomplete V3 all-axis timeout setup/rechecks or final pose capture')
        need(voltage_refresh_batches == set(BUSES) and set(voltage_refresh) == expected,
             'Missing V3 all-axis pre-enable voltage refresh')
        announce_end = runtime.get('announcement_completed_ns')
        need(type(announce_end) is int and announce_end > 0, 'Missing announcement completion time')
        need(max(r['received_ns'] for r in timeout_writes.values()) <
             min(r['start_ns'] for r in initial_timeouts.values()), 'Watchdog initial read preceded setup completion')
        need(max(r['received_ns'] for r in initial_timeouts.values()) <= announce_end <=
             min(r['start_ns'] for r in final_timeouts.values()), 'Watchdog recheck did not follow announcement')
        need(max(r['received_ns'] for r in final_timeouts.values()) <
             min(r['start_ns'] for r in final_pose.values()) and
             max(r['received_ns'] for r in final_pose.values()) < min(output_starts),
             'Watchdog recheck/final pose did not precede enable/targets')
        imu = runtime.get('pre_enable_imu')
        need(type(imu) is dict and type(imu.get('read_started_monotonic_ns')) is int and
             type(imu.get('read_finished_monotonic_ns')) is int and
             max(r['received_ns'] for r in final_pose.values()) < imu['read_started_monotonic_ns'] <=
             imu['read_finished_monotonic_ns'] <
             min(row['start_ns'] for row, _ in voltage_refresh.values()) and
             max(row['received_ns'] for row, _ in voltage_refresh.values()) < min(output_starts),
             'V3 voltage refresh must follow final pose/IMU and precede enable')
        guard = runtime.get('voltage_guard')
        need(type(guard) is dict and guard.get('maximum_age_ms') == 126. and
             type(guard.get('pre_enable_refresh_by_id')) is dict and
             type(guard.get('latest_by_id')) is dict and
             set(guard['pre_enable_refresh_by_id']) == set(IDS) and
             set(guard['latest_by_id']) == set(IDS),
             'Missing or incompatible V3 all-axis voltage guard')
        for mid in expected:
            row, value = voltage_refresh[mid]
            need(guard['pre_enable_refresh_by_id'][str(mid)] ==
                 {'value_v': value, 'received_ns': row['received_ns']},
                 'V3 pre-enable voltage guard differs from raw CAN')
            latest = max((t, voltage) for i, t, voltage in voltages if i == mid)
            need(guard['latest_by_id'][str(mid)] ==
                 {'value_v': latest[1], 'received_ns': latest[0]},
                 'V3 latest voltage guard differs from raw CAN')
        need(guard.get('checks_before_type1') == 13 + 2 * len(runtime.get('cycles', [])) and
             finite(guard.get('maximum_checked_age_ms')) and
             0 <= guard['maximum_checked_age_ms'] <= 126. and
             finite(guard.get('minimum_checked_voltage_v')) and
             profile['voltage_min_v'] <= guard['minimum_checked_voltage_v'] <= profile['voltage_max_v'],
             'V3 voltage guard check count/range invalid')
        first_refresh = min(row['received_ns'] for row, _ in voltage_refresh.values())
        need(guard['minimum_checked_voltage_v'] ==
             min(value for _, t, value in voltages if t >= first_refresh),
             'V3 minimum voltage guard differs from raw CAN')
        voltage_times = {mid: sorted(t for i, t, _ in voltages if i == mid) for mid in expected}
        for batch in journal:
            for row in batch['records']:
                tx = _frame(row['tx_hex'])
                if tx.kind not in (1, 3):
                    continue
                start = row['start_ns']
                for mid in expected:
                    times = voltage_times[mid];index = bisect_right(times, start) - 1
                    need(index >= 0 and start - times[index] <= V3_VOLTAGE_MAX_AGE_NS,
                         'V3 Type1/enable had missing or stale all-axis voltage')
        acquisitions = [batch for batch in journal if batch.get('phase') == 'feedback_hold']
        need(acquisitions and len(acquisitions) % 2 == 0, 'Missing V3 acquisition batches')
        for bus, ids in BUSES.items():
            batches = [batch for batch in acquisitions if batch['bus'] == bus]
            need(len(batches) == len(runtime.get('cycles',[])), 'V3 acquisition cycle count mismatch')
            for index, batch in enumerate(batches):
                frames = [_frame(row['tx_hex']) for row in batch['records']]
                wanted = [ids[index % 6]]
                feedback_ids = [frame.destination for frame in frames if frame.kind == 1]
                voltage_ids = [frame.destination for frame in frames if frame.kind == 17 and
                    frame.data[:2] == codec.PARAMETERS['voltage'][0].to_bytes(2,'little')]
                need(len(frames) == 7 and feedback_ids == list(ids) and voltage_ids == wanted,
                     'V3 acquisition must be six Type1 feedback replies plus one rotating voltage')
    return feedback, voltages


def _stops(runtime, *, after_ns):
    need(runtime.get('stop_confirmed') is True and not runtime.get('stop_faults_by_id'), 'All-axis STOP is unconfirmed/faulted')
    reports = runtime.get('stop_reports')
    need(type(reports) is dict and set(reports) == set(BUSES), 'Missing final raw STOP evidence')
    for bus, ids in BUSES.items():
        report = reports[bus]
        need(type(report) is dict, 'Malformed final STOP report')
        confirmed = report.get('confirmed_ids')
        need(type(confirmed) is list and len(confirmed) == len(ids) and
             all(type(mid) is int for mid in confirmed) and set(confirmed) == set(ids),
             'Final STOP confirmed IDs incomplete/duplicated')
        need(report.get('complete') is True and
             not report.get('unconfirmed_ids') and not report.get('ambiguous_ids') and
             not report.get('errors') and not report.get('error'),
             'Final STOP report is incomplete/ambiguous/errored')
        # Mode0 records prove an observed reset state only. They cannot erase
        # unresolved same-key replies recorded by the session owner, even when
        # the outer runtime summary incorrectly says stop_confirmed=True.
        need(report.get('sticky_boundary_uncertain', False) is False,
             'Final STOP receive boundary remains uncertain')
    evidence = {bus: value.get('evidence') for bus,value in reports.items()}
    need(set(evidence) == set(BUSES) and all(type(v) is dict for v in evidence.values()), 'Missing final raw STOP evidence')
    last = 0
    for bus, ids in BUSES.items():
        seen = set()
        for row in evidence[bus].get('records', []):
            tx, rx = _record(row, bus)
            need(tx.wire == protocol.stop_request(phase=protocol.TrialPhase.STOP,
                 motor_id=tx.destination) and tx.destination not in seen,
                 'Final STOP request mismatch/duplicate')
            # A fresh reply on one bus/axis cannot make an older STOP from
            # another axis evidence of the final state. No enable or Type1
            # transaction may occur after any of these final stop requests.
            need(row['start_ns'] >= after_ns,
                 'STOP preceded final control/output completion')
            value = protocol.decode_type2(rx, motor_id=tx.destination)
            need(value.mode_state == 0 and value.fault_bits == 0, 'Final STOP mode/fault mismatch')
            seen.add(tx.destination); last = max(last, row['received_ns'])
        need(seen == set(ids), 'Final STOP incomplete')
    return last


def _cycles(runtime, profile, feedback, voltages, rotation, bias):
    v3 = profile['schema'] == SCHEMA_V3
    cycles = runtime.get('cycles'); offsets = runtime.get('fixed_offsets_rad_by_id', {})
    need(type(cycles) is list and cycles and set(offsets) == set(IDS), 'Missing cycles/fixed calibration branch')
    need(all(finite(v) for v in offsets.values()), 'Nonfinite angle branch')
    for mid in IDS:
        turns = (profile['axes'][mid]['offset_rad']-offsets[mid])/(2*math.pi)
        need(abs(turns-round(turns)) < 1e-8, 'Fixed offset is not a calibration-equivalent whole turn')
    previous_end = 0; previous_begin = None; previous_imu = 0; previous_sample = 0.; misses = 0; consecutive = 0
    full_policy_cycles = 0; max_tilt = 0.; max_age_ms = 0.; minimum_voltage = math.inf
    max_cycle_ms = 0.; max_latency_ms = 0.; max_voltage_age_ms = 0.; initial_q = None; previous_q = None
    previous_command_time = None; previous_command_velocity = None; previous_target = None
    max_release_interval_ms = 0.; release_intervals_over_20ms = 0; release_intervals_over_21ms = 0
    for index, cycle in enumerate(cycles):
        begin, end = cycle.get('begin_ns'), cycle.get('end_ns')
        need(type(begin) is int and type(end) is int and previous_end <= begin < end and begin > 0,
             'Overlapping/noncausal control cycles')
        need(cycle.get('index') == index, 'Cycle index discontinuity')
        interval = None if previous_begin is None else (begin-previous_begin)/1e6
        if 'release_interval_ms' in cycle:
            logged = cycle['release_interval_ms']
            need(logged is None if interval is None else
                 finite(logged) and abs(logged-interval) < .000001,
                 'Reported release interval disagrees with cycle timestamps')
        if interval is not None:
            max_release_interval_ms = max(max_release_interval_ms, interval)
            # Existing scheduling allowance: 20ms period + 1ms wakeup jitter.
            # This does not relax either 20ms measured-work criterion below.
            release_intervals_over_20ms += int(interval > 20.)
            release_intervals_over_21ms += int(interval > 21.)
        previous_begin = begin
        previous_end = end
        need((end-begin)/1e6 <= profile['hard_cycle_ms'], 'Hard cycle deadline exceeded')
        outgoing = [r for r in feedback if r['tx'].kind == 1 and r['phase'] in
                    ('policy_output', 'startup_hold', 'graceful_stop') and
                    begin <= r['row']['start_ns'] <= r['row']['received_ns'] <= end]
        need(len(outgoing) == 12 and {r['mid'] for r in outgoing} == set(range(1, 13)),
             'Cycle lacks twelve actual output/feedback transactions')
        sample = cycle.get('feedback', {}); command = cycle.get('command', {})
        q = vector(sample.get('q_model_rad'), 12, 'joint positions')
        v = vector(sample.get('velocity_rad_s'), 12, 'joint velocities')
        torque = vector(sample.get('torque_nm'), 12, 'joint torque')
        temp = vector(sample.get('temperature_c'), 12, 'joint temperature')
        target = vector(command.get('q_model_rad'), 12, 'targets')
        kp = vector(command.get('kp'), 12, 'Kp'); kd = vector(command.get('kd'), 12, 'Kd')
        command_velocity = vector(command.get('command_velocity_rad_s'),12,'command velocity')
        command_time = command.get('monotonic_s')
        need(finite(command_time) and begin <= command_time*1e9 <= end, 'Invalid command timestamp')
        for name in ('velocity_reference_rad_s','feedforward_torque_nm'):
            need(all(x == 0. for x in vector(command.get(name),12,name)), 'Nonzero unreviewed feedforward/reference')
        need(finite(command.get('gain_scale')) and 0 <= command['gain_scale'] <= 1., 'Invalid gain scale')
        need(finite(sample.get('monotonic_s')) and sample['monotonic_s'] > previous_sample,
             'Repeated/missing actual feedback timestamp')
        if previous_sample:
            need((sample['monotonic_s']-previous_sample)*1000 <= profile['max_sample_gap_ms'], 'Feedback sample gap exceeded')
        sample_dt = sample['monotonic_s']-previous_sample
        previous_sample = sample['monotonic_s']
        if initial_q is None: initial_q = list(q)
        need(abs(sample['monotonic_s']*1e9-min(r['row']['start_ns'] for r in outgoing)) < 1.,
             'Feedback timestamp disagrees with raw records')
        for r in outgoing:
            mid = str(r['mid']); k = r['mid']-1; a = profile['axes'][mid]; raw = r['value']
            expected = (a['sign']*raw.protocol_position_rad+offsets[mid], a['sign']*raw.velocity_rad_s,
                        raw.torque_nm, raw.temperature_c)
            need(all(abs(x-y) < 1e-8 for x,y in zip((q[k],v[k],torque[k],temp[k]), expected)),
                 'Decoded feedback disagrees with raw CAN')
            lower = a['physical_lower_rad']+a['uncertainty_rad']; upper = a['physical_upper_rad']-a['uncertainty_rad']
            need(lower <= q[k] <= upper and lower <= target[k] <= upper, 'Joint/target physical limit exceeded')
            need(r['tx'].wire == encode_motion(int(mid),(target[k]-offsets[mid])/a['sign'],kp[k],kd[k]), 'Actual command differs from logged target/gains')
            need(abs(q[k]-initial_q[k]) <= a['max_displacement_from_start_rad'], 'Displacement budget exceeded')
            if previous_q is not None:
                need(abs(q[k]-previous_q[k]) <= a['max_measured_velocity_rad_s']*sample_dt+.01, 'Position discontinuity')
            need(0 <= kp[k] <= a['kp'] and 0 <= kd[k] <= a['kd'], 'Command gains outside review')
            need(abs(target[k]-q[k]) <= a['max_tracking_error_rad'], 'Tracking error exceeded')
            need(abs(command_velocity[k]) <= a['max_command_velocity_rad_s'], 'Command velocity budget exceeded')
            if previous_command_time is not None:
                dt = command_time-previous_command_time
                need(dt > 0 and abs(command_velocity[k]-previous_command_velocity[k]) <= a['max_command_acceleration_rad_s2']*dt+1e-9, 'Command acceleration budget exceeded')
                # The logged derivative alone does not prove the nominal
                # position reference obeyed its slew limit. Its encoding was
                # matched byte-for-byte above. Check the original float target,
                # not decoded wire steps: crossing one CAN quantization bin is
                # not a measured velocity. Instantaneous envelope velocity can
                # also differ from the interval average during acceleration.
                need(abs(target[k]-previous_target[k]) <= a['max_command_velocity_rad_s']*dt+1e-9,
                     'Actual target position slew exceeded command velocity budget')
            pd = kp[k]*(target[k]-q[k])-kd[k]*v[k]
            need(abs(pd) <= a['max_estimated_pd_torque_nm'], 'Estimated PD budget exceeded')
        previous_q = list(q)
        previous_command_time = command_time; previous_command_velocity = command_velocity
        previous_target = list(target)
        imu = cycle.get('imu', {})
        imu_start, imu_end = imu.get('read_started_monotonic_ns'), imu.get('read_finished_monotonic_ns')
        need(type(imu_start) is int and type(imu_end) is int and previous_imu < imu_start <= imu_end <= end,
             'IMU timestamp repeated/noncausal')
        previous_imu = imu_start
        need(imu.get('frame') == 'sensor' and all(imu.get(k, False) is False for k in
             ('mount_correction_applied','gyro_bias_correction_applied','mount_rotation_applied','gyro_bias_subtracted')),
             'IMU raw frame/correction state unknown')
        accel = vector(imu.get('accel_m_s2'),3,'IMU acceleration'); gyro = vector(imu.get('gyro_rad_s'),3,'IMU gyro')
        body = [sum(rotation[i][j]*accel[j] for j in range(3)) for i in range(3)]
        body_gyro = [sum(rotation[i][j]*(gyro[j]-bias[j]) for j in range(3)) for i in range(3)]
        norm = math.hypot(*body)
        need(profile['imu_accel_norm_min_m_s2'] <= norm <= profile['imu_accel_norm_max_m_s2'], 'IMU acceleration norm limit')
        tilt = math.acos(max(-1., min(1., body[2]/norm)))
        need(tilt <= profile['imu_tilt_limit_rad'] and math.hypot(*body_gyro) <= profile['imu_gyro_limit_rad_s'],
             'IMU tilt/angular velocity limit')
        max_tilt = max(max_tilt, tilt)
        if 'imu_body' in cycle:
            logged = cycle['imu_body']
            need(logged.get('frame') == 'body' and logged.get('source_monotonic_ns') == imu_start, 'IMU body source mismatch')
            for key,expected in (('accel_m_s2',body),('gyro_rad_s',body_gyro)):
                actual = vector(logged.get(key),3,'corrected IMU '+key)
                need(all(abs(x-y) < 1e-8 for x,y in zip(actual,expected)), 'IMU transform/bias mismatch')
            need(finite(logged.get('tilt_rad')) and finite(logged.get('accel_norm_m_s2')) and
                 abs(logged['tilt_rad']-tilt) < 1e-8 and abs(logged['accel_norm_m_s2']-norm) < 1e-8, 'IMU angle/norm mismatch')
        acquisition = [r for r in feedback if r['phase'] == 'feedback_hold' and
                       begin <= r['row']['start_ns'] <= r['row']['received_ns'] <= end]
        need(len(acquisition) == 12 and {r['mid'] for r in acquisition} == set(range(1,13)), 'Missing fresh parallel input records')
        first = min(imu_start, *(r['row']['start_ns'] for r in acquisition))
        last_write = max(r['row']['finish_ns'] for r in outgoing)
        age = (end-first)/1e6; max_age_ms = max(max_age_ms, age)
        need(age <= profile['max_sample_age_ms'], 'Stale input in output cycle')
        latency = (last_write-first)/1e6
        max_cycle_ms = max(max_cycle_ms,(end-begin)/1e6); max_latency_ms = max(max_latency_ms,latency)
        need(finite(cycle.get('oldest_input_to_final_host_write_ms')) and
             abs(latency-cycle['oldest_input_to_final_host_write_ms']) < .000001, 'Reported pipeline time mismatch')
        missed = end-begin > 20_000_000 or last_write-first > 20_000_000
        need(cycle.get('deadline20ms_missed') is missed, 'Incorrect 20ms miss classification')
        misses += int(missed); consecutive = consecutive+1 if missed else 0
        need(consecutive <= profile['max_consecutive_20ms_misses'], '20ms miss budget exceeded')
        for mid in range(1,13):
            candidates = [(t,val) for i,t,val in voltages if i == mid and t <= end]
            need(candidates, 'No prior voltage sample')
            t,val = max(candidates); minimum_voltage = min(minimum_voltage, val)
            max_voltage_age_ms = max(max_voltage_age_ms,(end-t)/1e6)
            if v3:
                need(end-t <= V3_VOLTAGE_MAX_AGE_NS,
                     'V3 all-axis voltage evidence too old')
            elif index >= 6:
                need(end-t <= 6*(profile['hard_cycle_ms']+1)*1e6, 'Rotating voltage evidence too old')
        weight = cycle.get('effective_policy_weight')
        need(finite(weight) and 0 <= weight <= 1., 'Invalid learned output weight')
        if weight == 1. and command['gain_scale'] == 1. and all(k > 0 for k in kp) and all(r['phase'] == 'policy_output' for r in outgoing):
            full_policy_cycles += 1
    # normal_ramp_completed is a summary flag. Establish normal completion from
    # the last transmitted command as well; an emergency STOP after an active
    # positive-gain cycle is stopped, but is not a completed planned ramp.
    need(cycles[-1].get('phase') == command.get('phase') == 'stopped' and
         command.get('stop_stage') == 'complete' and command['gain_scale'] == 0. and
         all(x == 0. for x in (*kp,*kd,*command_velocity)) and
         all(r['phase'] == 'graceful_stop' for r in outgoing),
         'Final command did not complete zero-gain ramp')
    return {'cycles':len(cycles), 'full_learned_output_cycles':full_policy_cycles,
            'deadline20ms_misses':misses, 'max_tilt_rad':max_tilt, 'max_input_age_ms':max_age_ms,
            'minimum_observed_voltage_v':minimum_voltage, 'tilt_is_acceleration_direction_proxy':True,
            'max_iteration_ms':max_cycle_ms,
            'max_release_interval_ms':max_release_interval_ms,
            'release_intervals_over_20ms':release_intervals_over_20ms,
            'release_intervals_over_21ms':release_intervals_over_21ms,
            'strict_start_interval_20ms_met':len(cycles)>1 and release_intervals_over_20ms==0,
            'max_oldest_input_to_final_host_write_ms':max_latency_ms,
            'max_voltage_evidence_age_ms':max_voltage_age_ms,
            'first_cycle_ns':cycles[0]['begin_ns'],'last_cycle_ns':cycles[-1]['end_ns']}


def _association(value, hashes, start, end):
    need(value.get('schema') == ASSOCIATION_SCHEMA, 'Unsupported video association schema')
    refs = value.get('references', {})
    need(set(refs) == set(hashes), 'Four source references are required')
    for name, sha in hashes.items():
        ref = refs[name]
        need(type(ref) is dict and set(ref) == {'path','sha256'} and type(ref['path']) is str and ref['path']
             and ref['sha256'] == sha, 'Source association mismatch: '+name)
    sync = value.get('sync', {})
    need(set(sync) == {'trial_start_ns','trial_end_ns','video_start_s','video_end_s','uncertainty_ms'}, 'Missing explicit video synchronization')
    need(type(sync['trial_start_ns']) is int and type(sync['trial_end_ns']) is int and
         0 < sync['trial_start_ns'] <= start <= end <= sync['trial_end_ns'], 'Video/log window does not cover trial and STOP')
    need(all(finite(sync[k]) for k in ('video_start_s','video_end_s','uncertainty_ms')) and
         0 <= sync['video_start_s'] < sync['video_end_s'] and 0 <= sync['uncertainty_ms'] <= 100,
         'Invalid video interval/synchronization uncertainty')
    duration = (sync['trial_end_ns']-sync['trial_start_ns'])/1e9
    need(abs((sync['video_end_s']-sync['video_start_s'])-duration) <= max(.001,2*sync['uncertainty_ms']/1000),
         'Video and monotonic interval durations disagree')
    need(type(value.get('synchronization_method')) is str and value['synchronization_method'].strip(),
         'Describe visible/audible synchronization cue')


def _cues(report, plan, metrics, stop_end):
    from .ground_trial_trajectory import GroundTimeline
    need(report.get('planned_trajectory') == plan['trajectory'], 'Reported trajectory differs from plan')
    need(report.get('early_stop_requested') is False, 'Early-stopped run is not completion of the planned stage')
    timeline = GroundTimeline(**plan['trajectory'])
    events = report.get('cues')
    need(type(events) is list and events, 'Stage cues missing')
    for event in events:
        need(type(event.get('monotonic_ns')) is int and event['monotonic_ns'] > 0, 'Invalid event timestamp')
    # A terminal reader and control thread append concurrently; sort actual times.
    starts = [e for e in events if e.get('key') == 'GROUND_TIMELINE_STARTED']
    need(len(starts) == 1, 'Exactly one timeline start is required')
    start = starts[0]['monotonic_ns']
    need(start <= metrics['first_cycle_ns'] <= start+100_000_000, 'First cycle not tied to timeline start')
    timing = timeline.timing
    need(report.get('effective_timing') == timing, 'Effective timing differs from the complete reviewed plan')
    required = {'initial_hold':0., 'active_window_open':timing['active_start_s'],
                'active_window_close':timing['active_end_s'],
                'resupport_window_open':timing['resupport_window_open_s']}
    found = {}
    for key, expected_s in required.items():
        matching = [e for e in events if e.get('key') == key]
        need(len(matching) == 1, 'Missing/repeated stage cue: '+key)
        event = matching[0]; actual_s = (event['monotonic_ns']-start)/1e9
        need(finite(event.get('scheduled_elapsed_s')) and abs(event['scheduled_elapsed_s']-expected_s) < 1e-9
             and expected_s-1e-9 <= actual_s <= expected_s+.061,
             'Stage cue time mismatch/delay: '+key)
        emitted = event.get('emitted_ns')
        need(type(emitted) is int and event['monotonic_ns'] <= emitted <= event['monotonic_ns']+61_000_000, 'Cue emission time missing/noncausal')
        found[key] = emitted
    need(metrics['last_cycle_ns'] >= found['resupport_window_open'], 'Trial did not cover planned active/stationary windows')
    need(stop_end <= start+int((plan['trajectory']['duration_s']+.25)*1e9), 'STOP exceeded finite trial budget')
    if report['stage'] != 'supported_stance':
        acks = [e for e in events if e.get('key') == 'OPERATOR_RESUPPORT_ACK' and
                found['resupport_window_open'] < e['monotonic_ns'] <= start+timing['latest_stop_start_s']*1e9 and
                e.get('cue_emitted_ns') == found['resupport_window_open'] and
                type(e.get('cue_step')) is int and type(e.get('accepted_step')) is int and 0 <= e['cue_step'] < e['accepted_step'] and
                type(e.get('accepted_ns')) is int and e['monotonic_ns'] <= e['accepted_ns'] <= stop_end]
        need(acks, 'No fresh explicit re-support acknowledgement within actual cue/deadline')
        need(report.get('resupport_ack',{}).get('monotonic_ns') in {e['monotonic_ns'] for e in acks}, 'Accepted re-support summary mismatch')
        stops = [c for c in report['runtime_report']['cycles'] if c.get('phase') in ('stopping','stopped')]
        if stops:
            need(any(e['monotonic_ns'] <= stops[0]['begin_ns'] for e in acks), 'Gain-down began before fresh re-support')
    need(not any(e.get('key') in ('OPERATOR_EMERGENCY','TERMINAL_CLOSED') for e in events), 'Emergency/closed terminal event')


def physical_review_template():
    return {'schema':REVIEW_SCHEMA,'decision':'UNREVIEWED','reviewed_by':None,'reviewed_at':None,
            'association_sha256':None,'observations':{name:None for name in (
                'actual_robot_seen','entire_trial_and_stop_visible','all_four_feet_contact_before',
                'all_four_feet_contact_after','feet_contact_consistent_with_stage','slip_observed',
                'body_sinking_observed','overload_observed','unexpected_contact_observed',
                'unplanned_catch_required','planned_support_behavior_verified','physical_stop_verified',
                'independent_body_catch_used','planned_load_transfer_observed',
                'body_support_removed_during_active_window','catch_non_load_bearing_during_standing')},'measured_distance':None,'notes':''}


def _physical(review, association_raw, stage, plan):
    need(review.get('schema') == REVIEW_SCHEMA and review.get('association_sha256') == digest(association_raw),
         'Physical review is not bound to this video/log association')
    need(review.get('decision') == 'APPROVE_THIS_RECORDED_STAGE', 'Human review has not approved this recorded stage')
    need(type(review.get('reviewed_by')) is str and review['reviewed_by'].strip(), 'Named physical reviewer required')
    stamp(review.get('reviewed_at'))
    observations = review.get('observations', {})
    required = set(physical_review_template()['observations'])
    need(set(observations) == required and all(type(v) is bool for v in observations.values()), 'Physical observations remain unknown')
    for name in ('actual_robot_seen','entire_trial_and_stop_visible','all_four_feet_contact_before',
                 'all_four_feet_contact_after','feet_contact_consistent_with_stage',
                 'planned_support_behavior_verified','physical_stop_verified'):
        need(observations[name], 'Physical review did not verify '+name)
    for name in ('slip_observed','body_sinking_observed','overload_observed','unexpected_contact_observed','unplanned_catch_required'):
        need(not observations[name], 'Physical anomaly: '+name)
    if stage != 'supported_stance':
        need(observations['independent_body_catch_used'], 'Independent body catch not observed')
    if stage == 'supported_stance':
        need(not observations['body_support_removed_during_active_window'], 'Support was removed during supported stage')
    else:
        need(observations['planned_load_transfer_observed'], 'No actual load transfer was observed')
    if stage in ('stand','walk'):
        need(observations['body_support_removed_during_active_window'] and
             observations['catch_non_load_bearing_during_standing'], 'Self-support was not physically observed')
    measured = review.get('measured_distance')
    if stage == 'walk':
        need(type(measured) is dict and set(measured) == {'distance_m','uncertainty_m','method','reference'},
             'Walking requires a video-measured distance')
        need(finite(measured['distance_m']) and finite(measured['uncertainty_m']) and
             0 <= measured['uncertainty_m'] < measured['distance_m'], 'No resolved positive walking distance')
        need(measured['method'] == 'video_with_measured_reference' and type(measured['reference']) is str
             and measured['reference'].strip(), 'Distance must use visible measured reference; command integration is not measurement')
        limit = plan.get('maximum_measured_distance_m', plan.get('max_distance_m'))
        need(finite(limit) and measured['distance_m']+measured['uncertainty_m'] <= limit,
             'Measured distance bound is missing/exceeded')
    return observations, measured


def evaluate_bytes(report_raw, plan_raw, profile_raw, association_raw, *, video_sha256,
                   physical_review_raw=None, mount_raw=None, bias_raw=None, hardware_review_raw=None):
    """Pure analysis of exact JSON bytes. video_sha256 is streamed by evaluate_files.

    No output permission is created. A PASS is evidence for one recorded stage,
    under this exact assembly/profile, never a permission to repeat the motion.
    """
    result = {'schema':SCHEMA,'status':'RECORDED_REVIEW_REQUIRED','decision':'RECORDED_REVIEW_REQUIRED',
        'dependency_eligible':False,'errors':[],'review_required':[],
        'genuine_hardware_capture':False,'simulated':False,'replayed':False,
        'learned_policy_used':False,'fault_free':False,'all_axis_stop_confirmed':False,
        'physical_review_complete':False,'independent_body_catch_used':False,
        'load_percentage':None,'contact_inferred_from_torque':False,
        'true_sensor_timestamps_verified':False,'full_controller_50Hz_verified':False,
        'actual_controller_20ms_pass':False,'strict_controller_20ms_pass':False,
        'max_active_cycle_ms':None,
        'reviewed_by':None,'reviewed_at':None,'video_sha256':video_sha256,
        'report_sha256':digest(report_raw),'stage_plan_sha256':digest(plan_raw),
        'base_profile_sha256':digest(profile_raw)}
    try:
        report,plan,profile,association = map(document,(report_raw,plan_raw,profile_raw,association_raw))
        need(profile.get('watchdog_review_policy') is None,
             'Supported-only command-loss acceptance cannot become ground-stage evidence')
        result['association'] = association
        stage = report.get('stage'); result['stage'] = stage
        need(stage in STAGES and plan.get('stage') == stage, 'Unknown/mismatched stage')
        result.update(assembly_id=report.get('assembly_id'), uids_by_id={mid:profile['axes'][mid]['uid'] for mid in IDS})
        need(report.get('stage_plan_sha256') == digest(plan_raw) and report.get('profile_sha256') == digest(profile_raw), 'Runtime plan/profile hash mismatch')
        need(report.get('assembly_id') == profile['assembly_id'] and report.get('boot_id') == profile['boot_id']
             and report.get('motor_power_epoch') == profile['motor_power_epoch'], 'Runtime assembly/boot/power epoch mismatch')
        need(report.get('schema') == 'singularitydog.ground-trial-report.v1', 'Unsupported report schema')
        need(report.get('scope') == 'bounded_ground_characterization', 'Ground scope missing/mismatched')
        need(profile.get('approved_for_supported_policy_output') is True, 'Base profile was unreviewed')
        need(is_hash(video_sha256), 'Missing source video hash')
        result['simulated'] = report.get('execution_kind') == 'simulation' or report.get('simulated') is True or report.get('simulation_only') is True
        result['replayed'] = report.get('execution_kind') == 'replay' or report.get('replayed') is True
        if report.get('execution_kind') != 'hardware' or result['simulated'] or result['replayed']:
            result['review_required'].append('Simulation/replay is never hardware stage evidence')
        runtime = report.get('runtime_report', {})
        need(report.get('status') == 'COMPLETE_BOUNDED_GROUND_TRIAL' and
             runtime.get('status') == 'COMPLETE_SUPPORTED_OUTPUT' and not report.get('errors') and not runtime.get('errors'),
             'Trial aborted or incomplete')
        need(runtime.get('normal_ramp_completed') is True and runtime.get('motor_enable_sent') is True
             and runtime.get('command_output_sent') is True, 'No completed actual output run')
        need(type(runtime.get('actual_model_calls')) is int and runtime['actual_model_calls'] > 0,
             'No actual learned model calls')
        provenance = runtime.get('model_provenance', {})
        need(provenance.get('injected_model') is not True, 'Injected test model is not hardware evidence')
        need(provenance.get('manifest_sha256') == profile['artifacts']['model_manifest']['sha256'], 'Learned model manifest mismatch')
        need(mount_raw is not None and bias_raw is not None, 'Pinned IMU mount/bias bytes required')
        for name,raw in (('mount',mount_raw),('bias',bias_raw)):
            need(profile['artifacts'][name]['sha256'] == digest(raw), 'IMU artifact hash mismatch')
        rotation = _mount(document(mount_raw))['R_body_from_sensor']; bias = _bias(document(bias_raw))['bias_sensor_rad_s']
        need(hardware_review_raw is not None and
             profile['artifacts']['hardware_review']['sha256'] == digest(hardware_review_raw),
             'Pinned hardware/watchdog review bytes required')
        tested_firmware = document(hardware_review_raw).get('device_watchdog')
        feedback,voltages = _journal(runtime,profile,tested_firmware=tested_firmware)
        metrics = _cycles(runtime,profile,feedback,voltages,rotation,bias); result['metrics'] = metrics
        last_output = max(r['row']['received_ns'] for r in feedback if r['tx'].kind in (1,3))
        end = _stops(runtime,after_ns=max(metrics['last_cycle_ns'],last_output))
        result['all_axis_stop_confirmed'] = True
        need(runtime['actual_model_calls'] >= metrics['full_learned_output_cycles'], 'Insufficient actual model calls')
        result['fault_free'] = True
        result['actual_controller_20ms_pass'] = (metrics['deadline20ms_misses'] == 0 and
            metrics['max_iteration_ms'] <= 20. and metrics['max_oldest_input_to_final_host_write_ms'] <= 20. and
            metrics['release_intervals_over_21ms'] == 0)
        # The existing acceptance criterion permits up to 1ms wakeup jitter.
        # Keep strict 20ms start cadence separate so it cannot be inferred from
        # a 20ms work budget or the legacy acceptance result.
        result['strict_controller_20ms_pass'] = (result['actual_controller_20ms_pass'] and
            metrics['strict_start_interval_20ms_met'])
        result['max_active_cycle_ms'] = metrics['max_iteration_ms']
        result['learned_policy_used'] = metrics['full_learned_output_cycles'] > 0
        if stage == 'walk':
            need(metrics['deadline20ms_misses'] == 0 and metrics['max_iteration_ms'] <= 20. and
                 metrics['max_oldest_input_to_final_host_write_ms'] <= 20. and
                 metrics['release_intervals_over_21ms'] == 0,
                 'Walk requires actual complete 20ms cycles and release intervals <=21ms')
        if stage in ('stand','walk'):
            need(result['learned_policy_used'], 'Standing/walking needs a full learned output cycle (weight 1)')
        _association(association,{'report':digest(report_raw),'plan':digest(plan_raw),
            'profile':digest(profile_raw),'video':video_sha256},metrics['first_cycle_ns'],end)
        _cues(report,plan,metrics,end)
        if physical_review_raw is None:
            result['review_required'].append('Named video/physical review is missing')
        else:
            review = document(physical_review_raw)
            observations, measured = _physical(review,association_raw,stage,plan)
            result.update(physical_review_complete=True,independent_body_catch_used=observations['independent_body_catch_used'],
                reviewed_by=review['reviewed_by'],reviewed_at=review['reviewed_at'],
                physical_review_sha256=digest(physical_review_raw), measured_distance=measured)
        if not result['review_required']:
            result.update(status='PASS_REVIEWED_STAGE',decision='PASS_REVIEWED_HARDWARE_STAGE',
                          dependency_eligible=True,genuine_hardware_capture=True)
    except (ValueError,TypeError,KeyError,OverflowError,UnicodeError,AttributeError,IndexError,ZeroDivisionError) as error:
        result['errors'].append(type(error).__name__+': '+str(error))
        result.update(status='FAIL',decision='FAIL')
    return result


def _read(path):
    path = Path(path).expanduser()
    need(path.is_file() and not path.is_symlink(), 'Regular evidence file required: '+str(path))
    return path.read_bytes()


def _file_hash(path):
    path = Path(path).expanduser()
    need(path.is_file() and not path.is_symlink(), 'Regular source video required')
    h=hashlib.sha256()
    with path.open('rb') as f:
        for part in iter(lambda:f.read(1024*1024),b''):h.update(part)
    return h.hexdigest()


def evaluate_files(report_path,plan_path,profile_path,association_path,*,video_path,physical_review_path=None):
    from .policy_live_profile import load_profile
    from .ground_trial_plan import validate_ground_plan
    profile_raw = _read(profile_path); profile = document(profile_raw)
    validated_profile = load_profile(profile_path,require_approved=True)
    need(validated_profile['profile_sha256'] == digest(profile_raw), 'Base profile changed during review')
    plan_raw = _read(plan_path); plan = document(plan_raw)
    plan_base = Path(plan_path).expanduser().parent
    priors = {stage:_read(plan_base/ref['path']) for stage,ref in plan.get('prior_evaluations',{}).items()}
    validate_ground_plan(plan,validated_profile,priors,require_approved=True)
    base = Path(profile_path).expanduser().parent
    artifacts = {}
    for name in ('mount','bias','hardware_review'):
        path = Path(profile['artifacts'][name]['path']).expanduser()
        artifacts[name] = _read(path if path.is_absolute() else base/path)
    association_raw = _read(association_path); association = document(association_raw)
    supplied = {'report':report_path,'plan':plan_path,'profile':profile_path,'video':video_path}
    association_base = Path(association_path).expanduser().parent
    for name,path in supplied.items():
        referenced = Path(association['references'][name]['path']).expanduser()
        if not referenced.is_absolute():referenced = association_base/referenced
        need(referenced.resolve() == Path(path).expanduser().resolve(), 'Association path mismatch: '+name)
    return evaluate_bytes(_read(report_path),plan_raw,profile_raw,association_raw,
        video_sha256=_file_hash(video_path),physical_review_raw=_read(physical_review_path) if physical_review_path else None,
        mount_raw=artifacts['mount'],bias_raw=artifacts['bias'],hardware_review_raw=artifacts['hardware_review'])


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('report','plan','profile','association','video','output'):p.add_argument('--'+name,required=True)
    p.add_argument('--physical-review')
    a=p.parse_args(argv)
    try:
        result=evaluate_files(a.report,a.plan,a.profile,a.association,video_path=a.video,physical_review_path=a.physical_review)
    except (OSError,ValueError,TypeError,KeyError,AttributeError) as error:
        result={'schema':SCHEMA,'status':'FAIL','decision':'FAIL','dependency_eligible':False,
                'errors':[type(error).__name__+': '+str(error)],'review_required':[],
                'failure_scope':'file_validation','genuine_hardware_capture':False}
    out=Path(a.output).expanduser()
    need(not any((parent/'.git').exists() for parent in (out.parent,*out.parents)), 'Keep raw associated evidence outside Git')
    out.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    with os.fdopen(os.open(out,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600),'w') as f:
        json.dump(result,f,ensure_ascii=False,allow_nan=False,indent=2);f.write('\n')
    print(json.dumps({'status':result['status'],'dependency_eligible':result['dependency_eligible'],
                      'errors':result['errors'],'review_required':result['review_required']},ensure_ascii=False))
    return 0 if result['dependency_eligible'] else 2


if __name__=='__main__':raise SystemExit(main())
