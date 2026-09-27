"""Future supervised load-transfer rehearsal: finite raw-position hold only.

The default is a disabled, twenty-tick preflight. Live output requires both an
explicit boolean and a duration/boot/identity-bound review. Active duration is
2-15 seconds with 80 ms static-hold cycles. Targets are fresh Type-2 motor positions;
there is no policy, pose, load-transfer command, gain tuning, or retry.

Each bus has one UART owner and attempts all six STOPs in its finally block.
Python deadlines and the 200 ms device watchdog do not replace a physical
catch/support, bounded serial I/O, and an operator at the 40 V cutoff.
"""
from copy import deepcopy
from dataclasses import asdict
import math
from pathlib import Path
import threading
import time

from .bounded_pose_plan import (ARRIVAL_ERROR_CANDIDATE_RAD, MAX_HOLD_GAP_NS,
                                MIN_OBSERVED_HOLD_NS)
from .current_hold_review import PROFILE, _validate_motor_review
from .position_response_evidence import _digest, _expected_identities
from .rs05_bus_transport import BUS_IDS
from .rs05_joint_trial import check_feedback
from .rs05_leg_trial import (LEGS, MAX_DRIFT_RAD, SETTLED_COUNT, SETTLED_LIMITS,
                             SETTLED_PERIOD_S, FULLBODY_POSITION_PROFILE,
                             evaluate_settled_window)
from .rs05_trial_protocol import (TrialPhase, enable_request, motion_request,
                                  stop_request, watchdog_setup_request)

ALL_IDS = tuple(range(1, 13))
REVIEWED_IDS = (5, 6, 8)
REVIEW_SCHEMA = 'rs05-load-transfer-current-hold-review-v1'
REVIEW_SCOPE = 'supervised-load-transfer-current-position-2-15s'
KP4_IDS_BY_PROFILE = {
    'kp3': frozenset(),
    'id4-kp4-only': frozenset((4,)),
    'id10-kp4-only': frozenset((10,)),
    'id4-id10-kp4': frozenset((4, 10)),
}
GAIN_PROFILES = tuple(KP4_IDS_BY_PROFILE)
CYCLE_S, PREFLIGHT_TICKS = .08, 25
MIN_HOLD_TICKS, MAX_HOLD_TICKS = 25, 187
LOAD_TRANSFER_MAX_HOLD_GAP_NS = 100_000_000
PREPARATION_BUDGET_S, BARRIER_TIMEOUT_S = 30., 5.
ACTIVE_BUDGET_MARGIN_S = 1.
VOLTAGE_MIN_V, VOLTAGE_MAX_V = 35., 43.
WATCHDOG_TICKS = 4000
# A future reviewed release must deliberately change this source gate. There
# is no active launcher and no current load-transfer authorization artifact.
LIVE_OUTPUT_ENABLED = False
MAX_REVIEWED_TORQUE_MONITOR_NM = 1.5
SUPPORTED_FLOOR_START_TOLERANCE_RAD = math.radians(3.)
WRAP_EQUIVALENT_IDS = (3, 9)
# On the reviewed ID4+ID10 Kp4 profile, ID4 and ID9 have measured stable
# gravity biases (~1.16° and ~-1.04°). Only their late current-hold tolerance
# is 1.5°; other axes retain 1°, and live guards remain intact.
ID4_HOLD_STATIC_ERROR_RAD = math.radians(1.5)
ID9_HOLD_STATIC_ERROR_RAD = math.radians(1.5)


def static_hold_error_limit(mid, gain_profile):
    if gain_profile == 'id4-id10-kp4':
        if mid == 4:
            return ID4_HOLD_STATIC_ERROR_RAD
        if mid == 9:
            return ID9_HOLD_STATIC_ERROR_RAD
    return ARRIVAL_ERROR_CANDIDATE_RAD


def floor_start_equivalent(current_raw_rad, reference_raw_rad, mid, review):
    """Accept one encoder branch turn only for reviewed ID3/ID9.

    This is a start-envelope comparison, never a motor target transform. The
    motor target remains the newly read raw position on its current branch.
    """
    if (type(mid) is not int or type(current_raw_rad) not in (int, float)
            or type(reference_raw_rad) not in (int, float)
            or not math.isfinite(current_raw_rad) or not math.isfinite(reference_raw_rad)):
        return False
    delta = current_raw_rad - reference_raw_rad
    if abs(delta) <= SUPPORTED_FLOOR_START_TOLERANCE_RAD:
        return True
    return (mid in WRAP_EQUIVALENT_IDS
            and review.get('wrap_equivalence_motor_ids') == list(WRAP_EQUIVALENT_IDS)
            and review.get('physical_full_turn_excluded') is True
            and abs(abs(delta) - 2. * math.pi) <= SUPPORTED_FLOOR_START_TOLERANCE_RAD)


def _duration_ticks(duration_s):
    if type(duration_s) not in (int, float) or not math.isfinite(duration_s):
        raise ValueError('Hold duration must be a finite number of seconds')
    if not 2. <= duration_s <= 15.:
        raise ValueError('Hold duration must be 2-15 seconds')
    ticks = min(MAX_HOLD_TICKS, math.floor(duration_s / CYCLE_S + 1e-9))
    if ticks < MIN_HOLD_TICKS:
        raise ValueError('Hold duration must include at least 25 cycles')
    return ticks


def _review(expected_uids, reviewed_motor_ids, value, preflight_only, gain_profile,
            duration_s, live_output):
    """Require a distinct, boot-bound load-transfer review before any I/O."""
    expected = _expected_identities(expected_uids, ALL_IDS)
    ticks = _duration_ticks(duration_s)
    if (type(preflight_only) is not bool or type(live_output) is not bool
            or type(reviewed_motor_ids) not in (tuple, list)
            or any(type(i) is not int for i in reviewed_motor_ids)
            or tuple(reviewed_motor_ids) != REVIEWED_IDS):
        raise ValueError('Explicit boolean modes and exactly reviewed IDs5/6/8 are required')
    if not preflight_only and not live_output:
        raise ValueError('Live motor output is disabled by default')
    if not preflight_only and not LIVE_OUTPUT_ENABLED:
        raise ValueError('Load-transfer live output source gate is disabled')
    value = deepcopy(value)
    if (type(value) is not dict or value.get('schema') != REVIEW_SCHEMA
            or value.get('scope') != REVIEW_SCOPE
            or value.get('motor_ids') != list(ALL_IDS)
            or any(type(i) is not int for i in value.get('motor_ids', []))
            or value.get('reviewed_motor_ids') != list(REVIEWED_IDS)
            or any(type(i) is not int for i in value.get('reviewed_motor_ids', []))
            or value.get('firmware') != '0.5.0.13' or not _digest(value.get('sha256'))
            or type(value.get('duration_s')) not in (int, float)
            or not math.isfinite(value['duration_s'])
            or abs(value['duration_s'] - duration_s) > 1e-9
            or (value.get('review_complete') is not True if not preflight_only
                else value.get('preflight_review_complete') is not True)
            or value.get('source_files_verified') is not True
            or (not preflight_only and value.get('load_specific_limits_reviewed') is not True)
            or (not preflight_only and any(value.get(flag) is not True for flag in
                ('supported_stance_passed', 'physical_catch_reviewed',
                 'power_cutoff_operator_reviewed', 'off_power_transfer_rehearsal_reviewed')))
            or any(value.get(flag) is not False for flag in
                   ('calibration_verified', 'learned_policy_allowed', 'standing_allowed', 'l_target_replay_allowed'))
            or type(value.get('load_transfer_hold_authorized')) is not bool
            or (not preflight_only and value['load_transfer_hold_authorized'] is not True)):
        raise ValueError('A separate validated load-transfer current-hold review is required')
    if (type(gain_profile) is not str or gain_profile not in GAIN_PROFILES
            or value.get('gain_profile', 'kp3') != gain_profile):
        raise ValueError('The fixed gain profile requires matching explicit review authorization')
    if _expected_identities(value.get('motor_uids'), ALL_IDS) != expected:
        raise ValueError('Fullbody review identities differ from the supplied twelve identities')
    torque_limits = value.get('feedback_torque_abort_nm_by_motor')
    if (type(torque_limits) is not dict or set(torque_limits) != {str(i) for i in ALL_IDS}
            or any(type(limit) not in (int, float) or not math.isfinite(limit)
                   or not 0 < limit <= MAX_REVIEWED_TORQUE_MONITOR_NM
                   for limit in torque_limits.values())):
        raise ValueError('Review requires twelve load-specific Type2 torque abort monitors')
    floor_starts = value.get('supported_floor_start_raw_rad_by_id')
    if (type(floor_starts) is not dict or set(floor_starts) != {str(i) for i in ALL_IDS}
            or any(type(position) not in (int, float) or not math.isfinite(position)
                   or abs(position) > 12.57 for position in floor_starts.values())
            or value.get('supported_floor_start_tolerance_rad') != SUPPORTED_FLOOR_START_TOLERANCE_RAD):
        raise ValueError('Review requires twelve bounded supported-floor raw starting positions')
    if (value.get('wrap_equivalence_motor_ids') != list(WRAP_EQUIVALENT_IDS)
            or value.get('physical_full_turn_excluded') is not True):
        raise ValueError('ID3/ID9 branch equivalence requires explicit no-full-turn physical review')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if not boot or value.get('boot_id') != boot:
        raise ValueError('Fullbody review boot mismatch')
    reviews = value.get('motor_reviews')
    if type(reviews) is not dict or set(reviews) != {'5', '6', '8'}:
        raise ValueError('Fullbody review requires all three individual response reviews')
    for mid in REVIEWED_IDS:
        _validate_motor_review(reviews[str(mid)], expected, mid, boot)
    return {int(i): uid for i, uid in expected.items()}, value


def run_load_transfer_hold(transports, expected_uids, check_interrupt, emit, *, validated_review,
                           reviewed_motor_ids=REVIEWED_IDS, preflight_only=True,
                           live_output=False, duration_s=5., gain_profile,
                           clock=time.monotonic, wait=time.sleep, active_start_signal=None):
    """Run a disabled preflight by default, or an explicitly reviewed finite hold.

    ``transports`` is exactly {'front': ..., 'rear': ...}, with distinct single
    serial owners. ``validated_review`` follows REVIEW_SCHEMA/REVIEW_SCOPE,
    binds the chosen duration and current boot, and asserts source verification.
    The frozen, independent wrapper must make that assertion from source files.
    Only the fixed default Kp3 and explicitly reviewed ID4/ID10 Kp4 profiles
    exist. No input target, numeric gain, leg substitution, or retry is available.
    Callbacks must be bounded and thread-safe; emit is serialized here and all
    its time is included in deadlines. A blocked OS write cannot be cancelled
    safely by having the parent touch another worker's UART.
    """
    hold_ticks = _duration_ticks(duration_s)
    if active_start_signal is not None and type(active_start_signal) is not threading.Event:
        raise ValueError('Active-start signal must be a threading.Event')
    expected, review = _review(expected_uids, reviewed_motor_ids, validated_review,
                               preflight_only, gain_profile, duration_s, live_output)

    def motion_phase(mid):
        return (TrialPhase.POSITION_STEP5_KP4 if mid in KP4_IDS_BY_PROFILE[gain_profile]
                else TrialPhase.POSITION_STEP5)
    if type(transports) is not dict or set(transports) != set(BUS_IDS):
        raise ValueError('Supply exactly independent front and rear transports')
    for name, ids in BUS_IDS.items():
        t = transports[name]
        if (type(t.ids) is not tuple or any(type(i) is not int for i in t.ids)
                or t.ids != ids or getattr(t, 'bus_name', None) != name):
            raise ValueError('Each worker requires its exact six-axis bus transport')
        if not preflight_only and getattr(t, 'interleave_feedback', None) is not True:
            raise ValueError('Active hold requires reviewed interleaved six-axis feedback')
    front, rear = transports['front'], transports['rear']
    if front is rear or front.serial is rear.serial:
        raise ValueError('The buses must have independent UART owners')
    ports = [getattr(t.serial, 'port', None) for t in (front, rear)]
    if ports[0] is not None and ports[0] == ports[1]:
        raise ValueError('The same UART cannot serve both buses')

    abort = threading.Event()
    state_lock, emit_lock = threading.Lock(), threading.Lock()
    shared = {'latest': {}, 'centers': {}, 'disabled': {}, 'stages': {}, 'times': {},
              'epoch': None, 'validated_tick0': set()}
    reports = {name: {'stage': 'not_started', 'errors': [], 'cycle_count': 0,
                     'centers': {}, 'motors': {}, 'settled_windows': {}, 'stop_reports': {},
                     'completed': False} for name in BUS_IDS}
    parent_errors = []

    def release():
        with state_lock:
            if len(set(shared['stages'].values())) != 1:
                raise RuntimeError('Workers reached different coordination stages')
            shared['epoch'] = max(shared['times'].values())

    barrier = threading.Barrier(2, action=release)

    def request_abort():
        abort.set()
        barrier.abort()

    def interrupted():
        if abort.is_set():
            raise InterruptedError('Peer or parent requested all-bus abort')
        check_interrupt()

    def log(event):
        with emit_lock:
            emit(event)

    def worker(name):
        t, ids, report = transports[name], BUS_IDS[name], reports[name]
        centers, final_rows = {}, {}
        report['electrical_samples'] = []
        prep_deadline = clock() + PREPARATION_BUDGET_S
        active_budget = None
        strict_hold = False

        def check():
            interrupted()
            if active_budget is None and clock() > prep_deadline:
                raise RuntimeError('Fullbody preparation deadline exceeded')
            if active_budget is not None and clock() >= active_budget:
                raise RuntimeError('Fullbody active budget exceeded')

        def until(due):
            while clock() < due:
                check()
                remaining = due - clock()
                if remaining <= 0:
                    break
                wait(min(remaining, .01))
            check()

        def sync(stage):
            check()
            report['stage'] = stage
            with state_lock:
                shared['stages'][name], shared['times'][name] = stage, clock()
            barrier.wait(timeout=BARRIER_TIMEOUT_S)
            check()
            with state_lock:
                epoch = shared['epoch']
            until(epoch)
            return epoch

        def forbid_output():
            raise RuntimeError('Enable and nonzero gain are forbidden during disabled preparation/preflight')

        def arm_disabled():
            t.pre_enable_guard = t.pre_send_guard = forbid_output

        def guard(value, received, mid, mode, *, settled=False):
            numeric = (value.protocol_position_rad, value.velocity_rad_s, value.torque_nm,
                       value.temperature_c, received)
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in numeric):
                raise RuntimeError(f'ID{mid} malformed or nonfinite feedback')
            # An Enable frame is acknowledged while the actuator is still in
            # reset mode on some RS05 firmware.  The following neutral Type-1
            # frame is the mode transition into Motor mode.  During that
            # transition accept only the two known safe states (reset/motor),
            # while every command after the transition requires mode 2.
            if mode is None:
                if value.mode_state not in (0, 2):
                    raise RuntimeError(f'ID{mid} unexpected transition mode')
                required_mode = value.mode_state
            else:
                required_mode = mode
            check_feedback(value, centers[mid], received, clock(), required_mode=required_mode,
                           max_drift_rad=SETTLED_LIMITS['maximum_center_drift_rad'] if settled else MAX_DRIFT_RAD)
            if value.mode_state == 2 and abs(value.torque_nm) > review['feedback_torque_abort_nm_by_motor'][str(mid)]:
                raise RuntimeError(f'ID{mid} Type2 torque feedback monitor tripped')
            limit = static_hold_error_limit(mid, gain_profile)
            if strict_hold and abs(value.protocol_position_rad - centers[mid]) > limit:
                raise RuntimeError(f'ID{mid} final hold error exceeds{math.degrees(limit):g}-degree candidate')

        def publish(found):
            t.latest.update(found)
            with state_lock:
                shared['latest'].update(found)

        def batch(wires, mode, *, settled=False, expected_ids=None):
            expected_ids = ids if expected_ids is None else tuple(expected_ids)
            if not expected_ids or not set(expected_ids) <= set(ids):
                raise ValueError('Feedback subset must belong to this six-axis bus')
            check()
            requested = clock()
            found = t.feedback_many(wires, expected_ids)
            if set(found) != set(expected_ids) or any(type(i) is not int for i in found):
                raise RuntimeError('Missing or malformed feedback batch')
            for mid, (value, received) in found.items():
                if not requested <= received <= clock():
                    raise RuntimeError(f'ID{mid} response predates its request or is in the future')
                guard(value, received, mid, mode, settled=settled)
            publish(found)
            return found

        def neutral(mid):
            return motion_request(phase=TrialPhase.ZERO_GAIN, center_rad=centers[mid], motor_id=mid)

        def parameters(mid):
            check()
            values = {p: t.parameter(mid, p)['value']
                      for p in ('run_mode', 'position', 'current', 'velocity', 'voltage')}
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in values.values()):
                raise RuntimeError(f'ID{mid} nonfinite or malformed initial parameters')
            if (values['run_mode'] != 0 or abs(values['current']) > .05
                    or abs(values['velocity']) > .5
                    or not VOLTAGE_MIN_V <= values['voltage'] <= VOLTAGE_MAX_V):
                raise RuntimeError(f'ID{mid} disabled parameter envelope failed')
            motion_request(phase=motion_phase(mid), center_rad=values['position'], motor_id=mid)
            return values

        def all_guard(mode, *, disabled=False):
            check()
            with state_lock:
                source = dict(shared['disabled'] if disabled else shared['latest'])
                all_centers = dict(shared['centers'])
            if set(source) != set(ALL_IDS) or set(all_centers) != set(ALL_IDS):
                raise RuntimeError('No complete twelve-axis snapshot before output')
            now = clock()
            for mid, (value, received) in source.items():
                if mode is None:
                    if value.mode_state not in (0, 2):
                        raise RuntimeError(f'ID{mid} unexpected transition mode')
                    required_mode = value.mode_state
                else:
                    required_mode = mode
                check_feedback(value, all_centers[mid], received, now, required_mode=required_mode,
                               max_drift_rad=SETTLED_LIMITS['maximum_center_drift_rad'] if disabled else MAX_DRIFT_RAD)
                if value.mode_state == 2 and abs(value.torque_nm) > review['feedback_torque_abort_nm_by_motor'][str(mid)]:
                    raise RuntimeError(f'ID{mid} Type2 torque feedback monitor tripped')
                limit = static_hold_error_limit(mid, gain_profile)
                if strict_hold and abs(value.protocol_position_rad - all_centers[mid]) > limit:
                    raise RuntimeError(f'ID{mid} final hold error exceeds{math.degrees(limit):g}-degree candidate')

        try:
            t.check_interrupt, t.emit = check, log
            arm_disabled()
            report['stage'] = 'identities'
            for mid in ids:
                check()
                if t.parameter(mid).get('mcu_uid_hex') != expected[mid]:
                    raise RuntimeError(f'ID{mid} identity mismatch')
            sync('all_identities_checked')
            report['initial_stop'] = t.stop_all(ids)
            arm_disabled()
            if (set(report['initial_stop']) != set(ids)
                    or any(not report['initial_stop'][mid]['confirmed'] for mid in ids)):
                raise RuntimeError('Initial six-axis STOP not fully confirmed')
            for mid in ids:
                values = parameters(mid)
                centers[mid] = values['position']
                if abs(centers[mid] - report['initial_stop'][mid]['feedback']['protocol_position_rad']) > .02:
                    raise RuntimeError(f'ID{mid} Type17/Type2 position mismatch; no wrap')
                report['motors'][mid] = dict(values)
            t.feedback_guard = lambda value, received, mid: guard(value, received, mid, 0, settled=True)
            batch([neutral(mid) for mid in ids], 0, settled=True)
            for mid in ids:
                previous = t.parameter(mid, 'can_timeout')['value']
                if type(previous) is not int or not 0 <= previous <= 0xFFFFFFFF:
                    raise RuntimeError(f'ID{mid} invalid previous watchdog value')
                report['motors'][mid]['watchdog_previous_ticks'] = previous
            sync('all_previous_watchdogs_checked')
            for mid in ids:
                t.fresh_boundary()
                t.send(watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid))
            until(clock() + .015)
            for mid in ids:
                if t.parameter(mid, 'can_timeout')['value'] != WATCHDOG_TICKS:
                    raise RuntimeError(f'ID{mid} watchdog readback mismatch')
                report['motors'][mid]['watchdog_readback_ticks'] = WATCHDOG_TICKS
            for mid in ids:
                values = parameters(mid)
                if abs(values['position'] - centers[mid]) > .02:
                    raise RuntimeError(f'ID{mid} moved during disabled preparation')
                centers[mid] = values['position']
            window_start = sync('settled_window_start')
            rows = {mid: [] for mid in ids}
            for sample_index in range(SETTLED_COUNT):
                due = window_start + sample_index * SETTLED_PERIOD_S
                until(due)
                if clock() > due + CYCLE_S:
                    raise RuntimeError('Settled-window schedule missed50ms')
                found = batch([neutral(mid) for mid in ids], 0, settled=True)
                for mid, (value, received) in found.items():
                    rows[mid].append({'sample_index': sample_index, 'received_monotonic_s': received,
                                      'checked_monotonic_s': clock(), 'feedback': asdict(value)})
            for leg, members in LEGS.items():
                if not set(members) <= set(ids):
                    continue
                # Type-2 velocity is quantized/noisy while a disabled RS05 is
                # stationary. For the all-body gate, use the stricter
                # position/tail/mean-velocity conjunction for every axis and
                # retain velocity RMS as a warning. A real angle change still
                # fails position range or slope before any enable packet.
                evaluation = evaluate_settled_window(
                    {mid: rows[mid] for mid in members},
                    {mid: centers[mid] for mid in members},
                    profile=FULLBODY_POSITION_PROFILE)
                report['settled_windows'][leg] = evaluation
                if not evaluation['passed']:
                    raise RuntimeError(f'{leg} settled window rejected: ' + '; '.join(evaluation['errors']))
            final_rows = {mid: (t.latest[mid][0], t.latest[mid][1]) for mid in ids}
            for mid in ids:
                if (asdict(final_rows[mid][0]) != rows[mid][-1]['feedback']
                        or final_rows[mid][1] != rows[mid][-1]['received_monotonic_s']):
                    raise RuntimeError('Latest feedback changed after fixed settled window')
                centers[mid] = final_rows[mid][0].protocol_position_rad
                motion_request(phase=motion_phase(mid), center_rad=centers[mid], motor_id=mid)
            report['centers'] = dict(centers)
            for mid in ids:
                if not floor_start_equivalent(
                        centers[mid], review['supported_floor_start_raw_rad_by_id'][str(mid)],
                        mid, review):
                    raise RuntimeError(f'ID{mid} fresh raw start is outside supported floor envelope')
                difference = centers[mid] - review['supported_floor_start_raw_rad_by_id'][str(mid)]
                if abs(difference) > SUPPORTED_FLOOR_START_TOLERANCE_RAD:
                    report.setdefault('wrap_equivalence_used', {})[mid] = (
                        1 if difference > 0 else -1)
            with state_lock:
                shared['centers'].update(centers)
                shared['disabled'].update(final_rows)
            log({'kind': 'load_transfer_bus_ready', 'bus': name, 'centers': dict(centers)})
            sync('both_workers_ready')
            all_guard(0, disabled=True)
            if preflight_only:
                start = sync('disabled_benchmark_start')
                ticks, mode = PREFLIGHT_TICKS, 0
            else:
                active_budget = clock() + hold_ticks * CYCLE_S + ACTIVE_BUDGET_MARGIN_S
                t.active_deadline = active_budget
                # Enable is a complete mode transition on the current RS05
                # firmware: its fresh Type-2 replies report mode2.  Confirm
                # that twelve-axis snapshot before sending any nonzero gain.
                # A zero-gain frame is not needed here and would add another
                # USB2CAN reply burst to the critical path.
                t.pre_enable_guard = lambda: all_guard(None, disabled=True)
                t.feedback_guard = lambda value, received, mid: guard(value, received, mid, None)
                # The six-frame Enable burst can lose the last reply on a
                # CH340/USB2CAN path.  Serialize it as well; no later command
                # is permitted until every individual mode2 reply is fresh.
                for index, mid in enumerate(ids):
                    batch([enable_request(phase=TrialPhase.ENABLE, motor_id=mid)],
                          None, expected_ids=(mid,))
                    # Confirm the same pair on both buses before either bus
                    # may enable its next motor. A fault in the first reply
                    # cannot leave the peer free to enable five more motors.
                    sync(f'enable_pair_{index}_confirmed')
                t.pre_enable_guard = None
                sync('all_enables_confirmed')
                t.feedback_guard = lambda value, received, mid: guard(value, received, mid, 2)
                all_guard(2)
                # A full twelve-axis snapshot is checked at each cycle entry.
                # During a six-frame interleaved exchange, the transport
                # validates each reply and checks abort/deadline before every
                # next write. Rechecking shared.latest before every write
                # would read last tick's reply for motors already refreshed
                # inside this still-in-progress batch, causing a false stale
                # abort. The whole batch must finish before publish().
                t.pre_send_guard = None
                start = sync('active_position_hold_start')
                ticks, mode = hold_ticks, 2
            report['start_monotonic_s'] = start
            final_hold = {mid: [] for mid in ids}
            for tick in range(ticks):
                due, deadline = start + tick * CYCLE_S, start + (tick + 1) * CYCLE_S
                until(due)
                if clock() >= deadline:
                    raise RuntimeError('Load-transfer cycle missed80ms before batch')
                strict_hold = not preflight_only and tick >= hold_ticks - 20
                all_guard(mode)
                t.active_deadline = min(deadline, active_budget) if active_budget is not None else deadline
                if tick == 0:
                    # Benchmark the exact six single-axis reply schedule in
                    # disabled mode too. Active tick zero uses the same
                    # schedule with reviewed gains; every axis must answer
                    # before the next command, inside the 80 ms deadline.
                    found = {}
                    for mid in ids:
                        command = (neutral(mid) if preflight_only else motion_request(
                            phase=motion_phase(mid), center_rad=centers[mid], motor_id=mid))
                        found.update(batch([command], mode, expected_ids=(mid,)))
                else:
                    found = batch([neutral(mid) if preflight_only else motion_request(
                        phase=motion_phase(mid), center_rad=centers[mid], motor_id=mid) for mid in ids], mode)
                # Match the Type17 voltage/watchdog workload in the disabled
                # benchmark and active hold. One of six motors per bus is
                # sampled each tick; every motor is revisited every 480 ms.
                # Both reads share the same 80 ms deadline as CAN feedback.
                electrical_id = ids[tick % len(ids)]
                voltage = t.parameter(electrical_id, 'voltage')['value']
                watchdog = t.parameter(electrical_id, 'can_timeout')['value']
                if (type(voltage) not in (int, float) or not math.isfinite(voltage)
                        or not VOLTAGE_MIN_V <= voltage <= VOLTAGE_MAX_V):
                    raise RuntimeError(f'ID{electrical_id} voltage envelope failed')
                if type(watchdog) is not int or watchdog != WATCHDOG_TICKS:
                    raise RuntimeError(f'ID{electrical_id} watchdog readback mismatch')
                report['electrical_samples'].append({'tick': tick, 'motor_id': electrical_id,
                    'voltage_v': voltage, 'watchdog_ticks': watchdog,
                    'checked_monotonic_s': clock()})
                log({'kind': 'load_transfer_cycle', 'bus': name, 'tick': tick,
                     'preflight_only': preflight_only, 'due_monotonic_s': due,
                     'completed_monotonic_s': clock(), 'deadline_monotonic_s': deadline})
                if clock() > deadline:
                    raise RuntimeError('Load-transfer cycle missed80ms including replies/logging')
                report['cycle_count'] += 1
                if strict_hold:
                    for mid, (value, received) in found.items():
                        final_hold[mid].append((received, value.protocol_position_rad))
                sync(f'cycle_{tick}_complete')
                if clock() > deadline:
                    raise RuntimeError('Peer cycle/barrier missed80ms')
                if tick == 0 and not preflight_only and active_start_signal is not None:
                    # The barrier action precedes each worker's deadline
                    # check. Signal only after both workers have passed that
                    # check, so a late barrier cannot cue load transfer.
                    with state_lock:
                        shared['validated_tick0'].add(name)
                        if shared['validated_tick0'] == set(BUS_IDS) and not abort.is_set():
                            active_start_signal.set()
            until(start + duration_s)
            if not preflight_only:
                for mid, samples in final_hold.items():
                    times = [row[0] for row in samples]
                    if (len(times) != 20 or times[-1] - times[0] < MIN_OBSERVED_HOLD_NS / 1e9
                            or any(not 0 < b - a <= LOAD_TRANSFER_MAX_HOLD_GAP_NS / 1e9 for a, b in zip(times, times[1:]))):
                        raise RuntimeError(f'ID{mid} final hold observation coverage failed')
                report['final_hold_samples'] = final_hold
            check()
            report['completed'] = True
            report['stage'] = 'completed'
        except BaseException as error:
            report['errors'].append(repr(error))
            request_abort()
        finally:
            try:
                report['stop_reports'] = t.stop_all(ids)
            except BaseException as error:
                report['errors'].append('Final STOP failed: ' + repr(error))
                request_abort()
                # An unexpected transport-level failure must not prevent the
                # remaining fixed IDs from receiving individual STOP attempts.
                for mid in ids:
                    error_text = 'No confirmed final STOP'
                    try:
                        t.send(stop_request(phase=TrialPhase.STOP, motor_id=mid))
                    except BaseException as stop_error:
                        error_text += ': ' + repr(stop_error)
                    report['stop_reports'][mid] = {'confirmed': False, 'feedback': None, 'error': error_text}
            if (set(report['stop_reports']) != set(ids)
                    or any(not report['stop_reports'][mid]['confirmed'] for mid in ids)):
                report['errors'].append('Final all-six STOP not confirmed')
                request_abort()

    threads = [threading.Thread(target=worker, args=(name,), name=f'load-transfer-{name}') for name in BUS_IDS]
    started = []
    for thread in threads:
        try:
            thread.start()
            started.append(thread)
        except BaseException as error:
            parent_errors.append('Worker startup failed: ' + repr(error))
            request_abort()
            break
    while any(thread.is_alive() for thread in started):
        try:
            check_interrupt()
        except BaseException as error:
            if not parent_errors:
                parent_errors.append(repr(error))
            request_abort()
        for thread in started:
            try:
                thread.join(timeout=.01)
            except BaseException as error:
                if not parent_errors:
                    parent_errors.append(repr(error))
                request_abort()
    errors = parent_errors + [f'{name}: {error}' for name, report in reports.items() for error in report['errors']]
    stopped = all(set(report['stop_reports']) == set(BUS_IDS[name])
                  and all(row['confirmed'] for row in report['stop_reports'].values())
                  for name, report in reports.items())
    complete = not errors and not abort.is_set() and stopped and all(r['completed'] for r in reports.values())
    return {'status': ('PREFLIGHT_PASSED_RESET_CONFIRMED' if preflight_only else
                       'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED') if complete else 'ABORTED',
            'preflight_only': preflight_only, 'preflight_completed': complete and preflight_only,
            'motion_completed': complete and not preflight_only, 'stop_confirmed': stopped,
            'workers': reports, 'errors': errors, 'review': review, 'reviewed_motor_ids': list(REVIEWED_IDS),
            'calibration_verified': False, 'learned_policy_allowed': False, 'standing_allowed': False,
            'l_target_replay_allowed': False, 'continuous_hold_proven': False, 'automatic_retry': False,
            'load_transfer_proven': False, 'self_supported_stance_proven': False,
            'torque_monitor_semantics': 'Type2 feedback abort only; no physical torque cap',
            'gain_profile': gain_profile,
            'duration_s': hold_ticks * CYCLE_S, 'requested_ticks': hold_ticks,
            'live_output': live_output and not preflight_only,
            'Kp': 0. if preflight_only else (3. if gain_profile == 'kp3' else None),
            'Kp_by_motor_id': {mid: 0. if preflight_only else (
                4. if mid in KP4_IDS_BY_PROFILE[gain_profile]
                else 3.) for mid in ALL_IDS},
            'Kd': 0. if preflight_only else .15,
            'torque_feedforward_nm': 0., 'cycle_deadline_s': CYCLE_S}
