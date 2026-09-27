"""Disabled-by-default 2 mm box-supported rise and return trial.

The box stays beneath the torso and all four paws stay on the floor. This
dedicated two-UART worker does not claim load transfer or standing. The source
gate is closed; changing it requires a separate frozen physical/runtime review.
"""
from copy import deepcopy
from dataclasses import asdict
import math
from pathlib import Path
import threading
import time

from .bounded_pose_plan import MIN_OBSERVED_HOLD_NS
from .box_rise_candidate import (SCHEMA as BOX_RISE_SCHEMA,
                                 TICKS as BOX_RISE_TICKS,
                                 PERIOD_S as BOX_RISE_PERIOD_S,
                                 prepare_candidate as prepare_box_rise_candidate,
                                 verify_package_files)
from .can_readonly import ATParser
from .position_response_evidence import _expected_identities
from .rs05_bus_transport import BUS_IDS
from .rs05_joint_trial import MAX_FEEDBACK_AGE_S, check_feedback
from .rs05_leg_trial import (LEGS, SETTLED_COUNT, SETTLED_LIMITS,
                             SETTLED_PERIOD_S, FULLBODY_POSITION_PROFILE,
                             evaluate_settled_window)
from .rs05_trial_protocol import (TrialPhase, enable_request, motion_request,
                                  stop_request, watchdog_setup_request)

ALL_IDS = tuple(range(1, 13))
CYCLE_S, END_HOLD_TICKS, PREFLIGHT_TICKS = BOX_RISE_PERIOD_S, 20, 25
PREPARATION_BUDGET_S, BARRIER_TIMEOUT_S = 30., 5.
RAW_POSITION_LSB_RAD = 25.14 / 65535.
BOX_RISE_MAX_HOLD_GAP_NS = 100_000_000
VOLTAGE_MIN_V, VOLTAGE_MAX_V = 35., 43.
WATCHDOG_TICKS = 4000
# No active launcher or current hardware authorization exists.
LIVE_OUTPUT_ENABLED = False
BOX_RISE_GAIN_PROFILE = 'box-rise-kp3'
BOX_RISE_TRACKING_ERROR_RAD = math.radians(2.)
BOX_RISE_FINAL_ERROR_RAD = math.radians(1.5)
BOX_RISE_MAX_TOTAL_EXCURSION_RAD = math.radians(12.)
BOX_RISE_FEEDBACK_SPEED_RAD_S = .2
# Diagnostic feedback-abort threshold, not a physical output torque cap.
BOX_RISE_FEEDBACK_TORQUE_NM = 1.2
CONTINUOUS_HOLD_END_TICKS = (BOX_RISE_TICKS + END_HOLD_TICKS - 1,)


def _diagnostic(review):
    if type(review) is not dict or review.get('schema') != BOX_RISE_SCHEMA:
        raise ValueError('Box-rise review schema required')
    return 'box-rise'


def _gain_profile(review):
    _diagnostic(review)
    return BOX_RISE_GAIN_PROFILE


def _interleaved_feedback_enabled(review, preflight_only):
    return not preflight_only


def _continuous_profile(review):
    _diagnostic(review)
    return True


def _trajectory_limits(review):
    ticks = BOX_RISE_TICKS + END_HOLD_TICKS
    return ticks, ticks * CYCLE_S + 1.5


def _motion_phase(mid):
    if type(mid) is not int or mid not in ALL_IDS:
        raise ValueError('Box rise requires one of the twelve motor IDs')
    return TrialPhase.POSITION_STEP5


def _speed_limit(review):
    return BOX_RISE_FEEDBACK_SPEED_RAD_S


def _torque_limit(review, mid):
    return BOX_RISE_FEEDBACK_TORQUE_NM


def _drift_limit(review, mid):
    return BOX_RISE_MAX_TOTAL_EXCURSION_RAD


def _limits(review, mid):
    return (BOX_RISE_TRACKING_ERROR_RAD, BOX_RISE_MAX_TOTAL_EXCURSION_RAD,
            BOX_RISE_FINAL_ERROR_RAD)


def _check_measured_raw_corridor(review, mid, position):
    bounds = review['reviewed_raw_corridor_by_id'][str(mid)]
    if not bounds['min_rad'] <= position <= bounds['max_rad']:
        raise RuntimeError(f'ID{mid} measured position left reviewed raw corridor')


def _check_reviewed_start_envelope(centers, review):
    prepare_box_rise_candidate(review, current_boot_id=review['boot_id'],
                               current_motor_uids=review['motor_uids'],
                               fresh_raw_rad_by_id={str(mid): centers[mid] for mid in ALL_IDS})


def _plan(centers, review):
    candidate = prepare_box_rise_candidate(
        review, current_boot_id=review['boot_id'], current_motor_uids=review['motor_uids'],
        fresh_raw_rad_by_id={str(mid): centers[mid] for mid in ALL_IDS})
    rise = tuple({int(mid): value for mid, value in sample['raw_rad_by_id'].items()}
                 for sample in candidate['samples'])
    plan = rise + tuple(dict(rise[-1]) for _ in range(END_HOLD_TICKS))
    # The selected legacy Type-1 codec requires +/-5deg headroom even though
    # this command uses an absolute center with zero offset. Reject the entire
    # path before any Enable, never after a partial movement.
    for sample in plan:
        for mid, target in sample.items():
            motion_request(phase=TrialPhase.POSITION_STEP5,
                           center_rad=target, motor_id=mid)
    return plan


def _review(expected_uids, value, preflight_only, package_dir):
    expected = _expected_identities(expected_uids, ALL_IDS)
    if type(preflight_only) is not bool:
        raise ValueError('Explicit boolean preflight required')
    value = deepcopy(value)
    _diagnostic(value)
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if not boot or value.get('boot_id') != boot:
        raise ValueError('Box-rise review does not match the current boot')
    if _expected_identities(value.get('motor_uids'), ALL_IDS) != expected:
        raise ValueError('Box-rise motor identities differ')
    if value.get('source_files_verified') is not True:
        raise ValueError('Frozen box-rise source review required')
    if not preflight_only and (not LIVE_OUTPUT_ENABLED
                               or value.get('box_rise_authorized') is not True):
        raise ValueError('Live box-rise output is disabled')
    if not preflight_only:
        if package_dir is None:
            raise ValueError('An exact frozen box-rise package is required for active output')
        verify_package_files(package_dir, value)
    prepare_box_rise_candidate(value, current_boot_id=boot,
                               current_motor_uids=value['motor_uids'],
                               fresh_raw_rad_by_id=value['start_raw_rad_by_id'])
    return {int(mid): uid for mid, uid in expected.items()}, value


def run_box_rise_trial(transports, expected_uids, check_interrupt, emit, *, validated_review,
                       preflight_only=True, package_dir=None,
                       clock=time.monotonic, wait=time.sleep):
    """Run disabled preflight by default; active path exists for virtual tests.

    ``transports`` is exactly {'front': ..., 'rear': ...}, with distinct single
    serial owners. The frozen wrapper must prove source and evidence digests;
    assertions in the review alone do not establish the physical facts.
    The box remains under the torso through final STOP.
    Callbacks must be bounded and thread-safe; emit is serialized here and all
    its time is included in deadlines. A blocked OS write cannot be cancelled
    safely by having the parent touch another worker's UART.
    """
    expected, review = _review(expected_uids, validated_review, preflight_only, package_dir)
    diagnostic = _diagnostic(review)
    continuous = _continuous_profile(review)
    active_ticks, active_budget_s = _trajectory_limits(review)

    def motion_phase(mid):
        return _motion_phase(mid)
    if type(transports) is not dict or set(transports) != set(BUS_IDS):
        raise ValueError('Supply exactly independent front and rear transports')
    for name, ids in BUS_IDS.items():
        t = transports[name]
        if (type(t.ids) is not tuple or any(type(i) is not int for i in t.ids)
                or t.ids != ids or getattr(t, 'bus_name', None) != name):
            raise ValueError('Each worker requires its exact six-axis bus transport')
    front, rear = transports['front'], transports['rear']
    if front is rear or front.serial is rear.serial:
        raise ValueError('The buses must have independent UART owners')
    ports = [getattr(t.serial, 'port', None) for t in (front, rear)]
    if ports[0] is not None and ports[0] == ports[1]:
        raise ValueError('The same UART cannot serve both buses')
    interleaved_feedback = _interleaved_feedback_enabled(review, preflight_only)
    for transport in transports.values():
        transport.interleave_feedback = interleaved_feedback

    abort = threading.Event()
    state_lock, emit_lock = threading.Lock(), threading.Lock()
    shared = {'latest': {}, 'centers': {}, 'disabled': {}, 'stages': {}, 'times': {},
              'epoch': None, 'plan': None, 'last_target': None,
              'last_completed_tick': None}
    reports = {name: {'stage': 'not_started', 'errors': [], 'cycle_count': 0,
                     'centers': {}, 'motors': {}, 'settled_windows': {}, 'stop_reports': {},
                     'completed': False} for name in BUS_IDS}
    parent_errors = []

    def release():
        with state_lock:
            if len(set(shared['stages'].values())) != 1:
                raise RuntimeError('Workers reached different coordination stages')
            shared['epoch'] = max(shared['times'].values())
            stage = next(iter(shared['stages'].values()))
            if stage.startswith('cycle_') and stage.endswith('_complete') and shared['plan'] is not None:
                tick = int(stage.split('_')[1])
                shared['last_target'] = shared['plan'][tick]
                shared['last_completed_tick'] = tick

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
        raw_send = t.send
        expected_active_wires = None
        active_wires_sent = set()
        enable_wires_sent = set()
        enable_neutrals_sent = set()
        wire_stage = 'disabled'
        active_previous = {}

        def monitor_motion(value, received, mid, center, target, *, update_previous):
            _check_measured_raw_corridor(review, mid, value.protocol_position_rad)
            track_limit, excursion_limit, _ = _limits(review, mid)
            if abs(value.protocol_position_rad - target) > track_limit:
                raise RuntimeError(f'ID{mid} raw step tracking error exceeded limit')
            if abs(value.protocol_position_rad - center) > excursion_limit:
                raise RuntimeError(f'ID{mid} raw step excursion exceeded limit')
            if abs(value.torque_nm) > _torque_limit(review, mid):
                raise RuntimeError(f'ID{mid} raw step feedback torque monitor tripped')
            moving_guard = True  # All twelve axes participate in this finite transition.
            if moving_guard:
                if abs(value.velocity_rad_s) > _speed_limit(review):
                    raise RuntimeError(f'ID{mid} raw step feedback velocity exceeded limit')
                previous = active_previous.get(mid)
                if update_previous and previous is not None:
                    prior_position, prior_received = previous
                    duration = received - prior_received
                    if duration < 0 or (duration == 0 and abs(
                            value.protocol_position_rad-prior_position) > RAW_POSITION_LSB_RAD):
                        raise RuntimeError('ID7 raw step feedback chronology changed')
                    if duration > 0 and abs(value.protocol_position_rad - prior_position) / duration \
                            > _speed_limit(review):
                        raise RuntimeError(f'ID{mid} raw step measured position rate exceeded limit')
                if update_previous:
                    active_previous[mid] = (value.protocol_position_rad, received)

        def exact_wire_send(wire):
            """Reject extra, duplicate or changed nonzero-gain Type1 frames."""
            frames = ATParser().feed(wire)
            if len(frames) != 1 or frames[0].wire != wire:
                raise RuntimeError('Noncanonical raw step wire')
            frame = frames[0]
            mid = frame.destination
            if frame.kind == 1:
                if frame.data[4:8] == bytes(4):
                    if (mid not in centers or
                            wire != motion_request(phase=TrialPhase.ZERO_GAIN,
                                                   center_rad=centers[mid], motor_id=mid) or
                            (wire_stage != 'disabled' and
                             (wire_stage != 'enabling' or mid not in enable_wires_sent or
                              mid in enable_neutrals_sent))):
                        raise RuntimeError('Unexpected zero-gain raw step wire')
                    if wire_stage == 'enabling':
                        enable_neutrals_sent.add(mid)
                else:
                    if (wire_stage != 'trajectory' or expected_active_wires is None
                            or mid not in expected_active_wires or mid in active_wires_sent
                            or wire != expected_active_wires[mid]):
                        raise RuntimeError('Unexpected raw step Type1 wire')
                    active_wires_sent.add(mid)
            elif frame.kind == 3:
                if (wire_stage != 'enabling' or mid not in ids or mid in enable_wires_sent
                        or wire != enable_request(phase=TrialPhase.ENABLE, motor_id=mid)):
                    raise RuntimeError('Unexpected raw step Enable wire')
                enable_wires_sent.add(mid)
            elif frame.kind == 4:
                if mid not in ids or wire != stop_request(phase=TrialPhase.STOP, motor_id=mid):
                    raise RuntimeError('Unexpected raw step STOP wire')
            elif frame.kind == 18:
                if (wire_stage != 'disabled' or mid not in ids
                        or wire != watchdog_setup_request(phase=TrialPhase.WATCHDOG_SETUP, motor_id=mid)):
                    raise RuntimeError('Unexpected raw step watchdog wire')
            elif wire_stage != 'disabled':
                raise RuntimeError('Unexpected raw step command kind')
            return raw_send(wire)

        t.send = exact_wire_send

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
            _check_measured_raw_corridor(review, mid, value.protocol_position_rad)
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
                           max_drift_rad=(SETTLED_LIMITS['maximum_center_drift_rad']
                                          if settled else _drift_limit(review, mid)))
            if (wire_stage in ('enabling', 'trajectory')
                    and abs(value.velocity_rad_s) > _speed_limit(review)):
                raise RuntimeError(f'ID{mid} raw step feedback velocity exceeded limit')
            if not settled and active_budget is not None:
                with state_lock:
                    last_target = shared['last_target']
                if last_target is not None:
                    monitor_motion(value, received, mid, centers[mid], last_target[mid],
                                   update_previous=True)

        def publish(found):
            t.latest.update(found)
            with state_lock:
                shared['latest'].update(found)

        def guard_and_publish_active(value, received, mid):
            guard(value, received, mid, 2)
            # Replies can arrive while the peer is still sending its six-frame
            # batch. Make each checked sample visible before that peer's next
            # all-twelve freshness guard, rather than waiting for this entire
            # batch and its quiet interval to finish.
            with state_lock:
                shared['latest'][mid] = (value, received)

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

        def all_guard(mode, *, disabled=False, first_cycle=False):
            check()
            with state_lock:
                source = dict(shared['disabled'] if disabled else shared['latest'])
                all_centers = dict(shared['centers'])
                last_target = shared['last_target']
                plan = shared['plan']
                last_completed_tick = shared['last_completed_tick']
            if set(source) != set(ALL_IDS) or set(all_centers) != set(ALL_IDS):
                raise RuntimeError('No complete twelve-axis snapshot before output')
            now = clock()
            for mid, (value, received) in source.items():
                _check_measured_raw_corridor(review, mid, value.protocol_position_rad)
                if mode is None:
                    if value.mode_state not in (0, 2):
                        raise RuntimeError(f'ID{mid} unexpected transition mode')
                    required_mode = value.mode_state
                else:
                    required_mode = mode
                check_feedback(value, all_centers[mid], received, now, required_mode=required_mode,
                               max_drift_rad=(SETTLED_LIMITS['maximum_center_drift_rad']
                                              if disabled else _drift_limit(review, mid)),
                               max_age_s=(.125 if (wire_stage == 'trajectory' or first_cycle)
                                          and last_completed_tick is None
                                          else MAX_FEEDBACK_AGE_S))
                if (not disabled
                        and wire_stage in ('enabling', 'trajectory')
                        and abs(value.velocity_rad_s) > _speed_limit(review)):
                    raise RuntimeError(f'ID{mid} raw step feedback velocity exceeded limit')
                if not disabled and plan is not None and last_target is not None:
                    monitor_motion(value, received, mid, all_centers[mid], last_target[mid],
                                   update_previous=False)

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
                    raise RuntimeError('Settled-window schedule missed80ms')
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
            with state_lock:
                shared['centers'].update(centers)
                shared['disabled'].update(final_rows)
            log({'kind': 'fullbody_bus_ready', 'bus': name, 'centers': dict(centers)})
            sync('both_workers_ready')
            all_guard(0, disabled=True)
            with state_lock:
                fresh_centers = dict(shared['centers'])
            if not preflight_only:
                _check_reviewed_start_envelope(fresh_centers, review)
            plan = _plan(fresh_centers, review)
            with state_lock:
                if shared['plan'] is None:
                    shared['plan'] = plan
                    shared['last_target'] = dict(fresh_centers)
                elif shared['plan'] != plan:
                    raise RuntimeError('Two workers calculated different raw step plans')
            sync('both_paths_validated')
            if preflight_only:
                start = sync('disabled_benchmark_start')
                ticks, mode = PREFLIGHT_TICKS, 0
            else:
                # Enabling twelve devices is bounded by the preparation
                # deadline, not charged against the finite active trajectory.
                # Both workers start the active budget at their shared epoch.
                t.active_deadline = prep_deadline
                # Each Enable is followed by a center-bound zero-gain frame.
                # This supplies a second safe reply opportunity when the
                # USB2CAN adapter drops the Enable reply.  Confirm mode2 from
                # the final fresh reply before any nonzero gain.
                t.pre_enable_guard = lambda: all_guard(None, disabled=True)
                t.feedback_guard = lambda value, received, mid: guard(value, received, mid, None)
                wire_stage = 'enabling'
                # Serialize each two-frame transition; no later command is
                # permitted until this ID's fresh mode2 reply is confirmed.
                for mid in ids:
                    batch([enable_request(phase=TrialPhase.ENABLE, motor_id=mid), neutral(mid)],
                          2, expected_ids=(mid,))
                if enable_wires_sent != set(ids) or enable_neutrals_sent != set(ids):
                    raise RuntimeError('Exact six-pair Enable stage was incomplete')
                t.pre_enable_guard = None
                sync('all_enables_confirmed')
                t.feedback_guard = guard_and_publish_active
                # The first-cycle bound also covers the initial mode2 gate:
                # serialized Enable/neutral pairs can age the earliest reply
                # before the first active frame. All other gates are unchanged.
                all_guard(2, first_cycle=interleaved_feedback)
                t.pre_send_guard = lambda: all_guard(2)
                wire_stage = 'trajectory'
                start = sync('active_raw_step_start')
                active_budget = start + active_budget_s
                active_previous.update({mid: (value.protocol_position_rad, received)
                                        for mid, (value, received) in t.latest.items() if mid in ids})
                t.active_deadline = active_budget
                ticks, mode = active_ticks, 2
            report['start_monotonic_s'] = start
            final_hold = {mid: [] for mid in ids}
            hold_checks = []

            def check_hold(samples, target, label):
                for mid, observations in samples.items():
                    times = [row[0] for row in observations]
                    if (len(times) != END_HOLD_TICKS or times[-1] - times[0] < MIN_OBSERVED_HOLD_NS / 1e9
                            or any(not 0 < b-a <= BOX_RISE_MAX_HOLD_GAP_NS / 1e9
                                   for a, b in zip(times, times[1:]))):
                        raise RuntimeError(f'ID{mid} {label} hold observation coverage failed')
                    if abs(observations[-1][1] - target[mid]) > _limits(review, mid)[2]:
                        raise RuntimeError(f'ID{mid} {label} raw step error exceeds diagnostic limit')

            for tick in range(ticks):
                due, deadline = start + tick * CYCLE_S, start + (tick + 1) * CYCLE_S
                until(due)
                if clock() >= deadline:
                    raise RuntimeError('Box-rise cycle missed80ms before batch')
                all_guard(mode)
                t.active_deadline = min(deadline, active_budget) if active_budget is not None else deadline
                if not preflight_only:
                    sample = plan[tick]
                    expected_active_wires = {mid: motion_request(
                        phase=motion_phase(mid), center_rad=sample[mid],
                        motor_id=mid) for mid in ids}
                    active_wires_sent.clear()
                # The six selected-bus writes share one receive window even
                # on tick zero. Each raw_send still performs the all-twelve
                # snapshot guard and exact-wire check before that UART write.
                found = batch([neutral(mid) if preflight_only else expected_active_wires[mid]
                               for mid in ids], mode)
                if not preflight_only and active_wires_sent != set(ids):
                    raise RuntimeError('Exact six-wire raw step batch was incomplete')
                electrical_id = ids[tick % len(ids)]
                voltage = t.parameter(electrical_id, 'voltage')['value']
                watchdog = t.parameter(electrical_id, 'can_timeout')['value']
                if (type(voltage) not in (int, float) or not math.isfinite(voltage)
                        or not VOLTAGE_MIN_V <= voltage <= VOLTAGE_MAX_V):
                    raise RuntimeError(f'ID{electrical_id} voltage envelope failed')
                if type(watchdog) is not int or watchdog != WATCHDOG_TICKS:
                    raise RuntimeError(f'ID{electrical_id} watchdog readback mismatch')
                report['electrical_samples'].append({
                    'tick': tick, 'motor_id': electrical_id, 'voltage_v': voltage,
                    'watchdog_ticks': watchdog, 'checked_monotonic_s': clock()})
                log({'kind': 'fullbody_raw_step_cycle', 'bus': name, 'tick': tick,
                     'preflight_only': preflight_only, 'due_monotonic_s': due,
                     'completed_monotonic_s': clock(), 'deadline_monotonic_s': deadline})
                if clock() > deadline:
                    raise RuntimeError('Box-rise cycle missed80ms including replies/logging')
                report['cycle_count'] += 1
                in_hold = any(end-END_HOLD_TICKS < tick <= end
                              for end in CONTINUOUS_HOLD_END_TICKS)
                if not preflight_only and in_hold:
                    for mid, (value, received) in found.items():
                        final_hold[mid].append((received, value.protocol_position_rad))
                if not preflight_only and continuous and tick in CONTINUOUS_HOLD_END_TICKS:
                    # Both workers must pass their six-axis endpoint check
                    # before the common cycle barrier permits the next stage.
                    hold_check = {'end_tick': tick, 'samples': final_hold, 'confirmed': False}
                    hold_checks.append(hold_check)
                    report['continuous_hold_checks'] = hold_checks
                    check_hold(final_hold, plan[tick], f'continuous tick{tick}')
                    hold_check['confirmed'] = True
                    if tick != active_ticks - 1:
                        final_hold = {mid: [] for mid in ids}
                sync(f'cycle_{tick}_complete')
                if clock() > deadline:
                    raise RuntimeError('Peer cycle/barrier missed80ms')
            until(start + ticks * CYCLE_S)
            if not preflight_only:
                report['final_hold_samples'] = final_hold
                check_hold(final_hold, plan[-1], 'final')
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
            t.send = raw_send

    threads = [threading.Thread(target=worker, args=(name,), name=f'fullbody-{name}') for name in BUS_IDS]
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
    completed_label = 'BOX_SUPPORTED_2MM_RISE_RETURN_COMPLETED_RESET_CONFIRMED'
    return {'status': ('PREFLIGHT_PASSED_RESET_CONFIRMED' if preflight_only else
                       completed_label) if complete else 'ABORTED',
            'preflight_only': preflight_only, 'preflight_completed': complete and preflight_only,
            'motion_completed': complete and not preflight_only, 'stop_confirmed': stopped,
            'workers': reports, 'errors': errors, 'review': review,
            'calibration_verified': False, 'learned_policy_allowed': False, 'standing_allowed': False,
            'self_supported_standing_verified': False, 'support_required': True,
            'box_must_remain_under_torso': True, 'load_transfer_proven': False,
            'fixed_hip_raw_targets': True, 'nominal_body_rise_mm': 2.,
            'model_mapping_verified': False, 'l_target_replay_allowed': False,
            'continuous_hold_proven': False, 'automatic_retry': False,
            'gain_profile': _gain_profile(review), 'raw_diagnostic_only': True,
            'box_supported_rise_candidate_only': True,
            'interleaved_feedback': interleaved_feedback,
            'continuous_profile': 'reviewed-2mm-box-rise-return-v1',
            'reviewed_segment_count': 1,
            'active_ticks': active_ticks, 'active_duration_s': active_ticks*CYCLE_S,
            'active_budget_s': active_budget_s,
            'initial_hold_s': 0.,
            'diagnostic': diagnostic, 'moving_motor_ids': list(ALL_IDS),
            'ramp_s': BOX_RISE_TICKS*CYCLE_S,
            'endpoint_hold_s': END_HOLD_TICKS*CYCLE_S,
            'Kp': 0. if preflight_only else None,
            'Kp_by_motor_id': {mid: 0. if preflight_only else 3. for mid in ALL_IDS},
            'Kd': 0. if preflight_only else .15,
            'torque_feedforward_nm': 0., 'cycle_deadline_s': CYCLE_S}
