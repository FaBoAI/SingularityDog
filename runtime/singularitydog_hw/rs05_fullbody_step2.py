"""Finite, supported twelve-axis raw diagnostic; never policy standing.

This separate runner is intentionally unavailable until an explicit same-boot
review checks the diagnostic direction and swept clearance. Targets are formed
only from fresh raw angles on that boot; old raw/model offsets are never used.
Model-angle conversion is intentionally unverified, so no standing claim can
be made.
The current-position hold is a prerequisite, not a motion authorization. A
worker alone owns each UART, and every exit attempts all twelve STOPs. No
thread, USB serial timeout or 200ms motor watchdog is a physical safety device.
The caller must provide a supported robot and an attended 40V cutoff.
"""
from copy import deepcopy
from dataclasses import asdict
import math
from pathlib import Path
import threading
import time

from .bounded_pose_plan import (ARRIVAL_ERROR_CANDIDATE_RAD, MAX_HOLD_GAP_NS,
                                MIN_OBSERVED_HOLD_NS)
from .can_readonly import ATParser
from .fullbody_step10_plan import quintic_fraction
from .position_response_evidence import _digest, _expected_identities
from .rs05_bus_transport import BUS_IDS
from .rs05_joint_trial import MAX_FEEDBACK_AGE_S, check_feedback
from .rs05_leg_trial import (LEGS, MAX_DRIFT_RAD, SETTLED_COUNT, SETTLED_LIMITS,
                             SETTLED_PERIOD_S, FULLBODY_POSITION_PROFILE,
                             evaluate_settled_window)
from .rs05_trial_protocol import (TrialPhase, enable_request, motion_request,
                                  stop_request, watchdog_setup_request)

ALL_IDS = tuple(range(1, 13))
REVIEW_SCHEMA = 'rs05-fullbody-step2-review-v1'
REVIEW_SCOPE = 'supported-fullbody-raw-2deg-50ms-diagnostic'
ID7_REVIEW_SCHEMA = 'rs05-id7-step1-review-v1'
ID7_REVIEW_SCOPE = 'supported-id7-raw-1deg-other11-hold-50ms-diagnostic'
ROLE_GROUP_REVIEW_SCHEMA = 'rs05-role-group-step1-review-v1'
ROLE_GROUP_REVIEW_SCOPE = 'supported-four-axis-raw-bounded-other8-hold-50ms-diagnostic'
FRONT_THIGH_REVIEW_SCOPE = 'supported-front-thigh-two-axis-raw-bounded-other10-hold-50ms-diagnostic'
FRONT_HIP_REVIEW_SCOPE = 'supported-front-hip-two-axis-raw-bounded-other10-hold-50ms-diagnostic'
ROLE_GROUP_IDS = {
    'toe': frozenset((1, 4, 7, 10)),
    'thigh': frozenset((2, 5, 8, 11)),
    'front-thigh': frozenset((2, 5)),
    'front-hip': frozenset((3, 6)),
    'hip': frozenset((3, 6, 9, 12)),
}
ID7_MOTOR_ID = 7
KP4_IDS = frozenset((4, 10))
GAIN_PROFILE = 'id4-id10-kp4'
ROLE_THIGH_GAIN_PROFILE = 'role-thigh-kp12-all4-hip-kp12-all4-id4-id10-kp4'
ROLE_FRONT_THIGH_GAIN_PROFILE = 'role-front-thigh-kp12-front2-hip-kp12-all4-id4-id10-kp4'
ROLE_FRONT_HIP_GAIN_PROFILE = 'role-front-hip-kp12-front2-id4-id10-kp4'
CYCLE_S, RAMP_TICKS, END_HOLD_TICKS, PREFLIGHT_TICKS = .05, 160, 20, 20
ACTIVE_TICKS = RAMP_TICKS + END_HOLD_TICKS
MAX_AMPLITUDE_DEG = 2.
MAX_START_MISMATCH_RAD = math.radians(.5)
ROLE_GROUP_MAX_START_MISMATCH_RAD = math.radians(60.)
MAX_TRACKING_ERROR_RAD = math.radians(2.)
MAX_EXCURSION_RAD = math.radians(2.5)
MAX_FEEDBACK_TORQUE_NM = .8
ID7_MAX_FEEDBACK_SPEED_RAD_S = .2
ID7_MAX_FINITE_DIFFERENCE_RAD_S = .2
RAW_POSITION_LSB_RAD = 25.14 / 65535.
ID7_MAX_EXCURSION_RAD = math.radians(1.5)
ID7_MAX_TRACKING_ERROR_RAD = math.radians(1.5)
ID7_FINAL_ERROR_RAD = math.radians(.5)
ROLE_THIGH_TRACKING_ERROR_RAD = math.radians(10.5)
ROLE_THIGH_EXCURSION_RAD = math.radians(12.)
ROLE_THIGH_FINAL_ERROR_RAD = math.radians(4.)
ID4_ROLE_THIGH_HELD_FINAL_ERROR_RAD = math.radians(1.5)
ID9_FRONT_HIP_HELD_FINAL_ERROR_RAD = math.radians(1.5)
ROLE_THIGH_SPEED_RAD_S = .5
ROLE_FRONT_HIP_SPEED_RAD_S = .25
ROLE_THIGH_MAX_FEEDBACK_TORQUE_NM = 1.8
FRONT_HIP_MAX_EXCURSION_RAD = math.radians(6.)
FRONT_HIP_STEP10_MAX_EXCURSION_RAD = math.radians(11.)
FRONT_HIP_FINAL_ERROR_RAD = math.radians(1.5)
PREPARATION_BUDGET_S, ACTIVE_BUDGET_S, BARRIER_TIMEOUT_S = 30., 10.5, 5.


def _real(value, label):
    if type(value) not in (int, float):
        raise ValueError(f'{label} must be a finite real number')
    try:
        number = float(value)
    except OverflowError:
        raise ValueError(f'{label} must be finite') from None
    if not math.isfinite(number):
        raise ValueError(f'{label} must be finite')
    return number


def _exact_id_map(value, label, convert):
    if type(value) is not dict or set(value) != {str(i) for i in ALL_IDS}:
        raise ValueError(f'{label} must contain exactly string IDs 1..12')
    return {i: convert(value[str(i)], f'{label}[{i}]') for i in ALL_IDS}


def _sign(value, label):
    if type(value) is not int or value not in (-1, 1):
        raise ValueError(f'{label} must be exact integer -1 or +1')
    return value


def _diagnostic(review):
    if type(review) is not dict:
        raise ValueError('A raw diagnostic review dictionary is required')
    identity = (review.get('schema'), review.get('scope'))
    if identity == (REVIEW_SCHEMA, REVIEW_SCOPE):
        return 'fullbody-step2'
    if identity == (ID7_REVIEW_SCHEMA, ID7_REVIEW_SCOPE):
        return 'id7-step1'
    if identity in ((ROLE_GROUP_REVIEW_SCHEMA, ROLE_GROUP_REVIEW_SCOPE),
                    (ROLE_GROUP_REVIEW_SCHEMA, FRONT_THIGH_REVIEW_SCOPE),
                    (ROLE_GROUP_REVIEW_SCHEMA, FRONT_HIP_REVIEW_SCOPE)):
        return 'role-group-step1'
    raise ValueError('Unknown or mismatched raw diagnostic schema/scope')


def _gain_profile(review):
    if _diagnostic(review) == 'role-group-step1':
        if review.get('role_group') == 'thigh':
            return ROLE_THIGH_GAIN_PROFILE
        if review.get('role_group') == 'front-thigh':
            return ROLE_FRONT_THIGH_GAIN_PROFILE
        if review.get('role_group') == 'front-hip':
            return ROLE_FRONT_HIP_GAIN_PROFILE
    return GAIN_PROFILE


def _interleaved_feedback_enabled(review, preflight_only):
    """Limit the transport experiment to the reviewed front-hip 10 degree path."""
    return (preflight_only is False
            and _diagnostic(review) == 'role-group-step1'
            and review.get('scope') == FRONT_HIP_REVIEW_SCOPE
            and review.get('role_group') == 'front-hip'
            and review.get('amplitude_deg') == 10.
            and review.get('gain_profile') == ROLE_FRONT_HIP_GAIN_PROFILE)


def _motion_phase(review, mid):
    if (_diagnostic(review) == 'role-group-step1'
            and review.get('role_group') == 'front-hip'
            and mid in ROLE_GROUP_IDS['front-hip']):
        return (TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10
                if review.get('amplitude_deg') == 10.
                else TrialPhase.POSITION_ROLE_FRONT_HIP_KP12)
    if (_diagnostic(review) == 'role-group-step1'
            and review.get('role_group') in ('thigh', 'front-thigh')):
        if mid in ROLE_GROUP_IDS[review['role_group']]:
            return TrialPhase.POSITION_ROLE_THIGH_KP12
        if mid in ROLE_GROUP_IDS['hip']:
            return TrialPhase.POSITION_ROLE_HIP_HOLD_KP12
    return TrialPhase.POSITION_STEP5_KP4 if mid in KP4_IDS else TrialPhase.POSITION_STEP5


def _speed_limit(review):
    if _gain_profile(review) == ROLE_FRONT_HIP_GAIN_PROFILE:
        return ROLE_FRONT_HIP_SPEED_RAD_S
    return (ROLE_THIGH_SPEED_RAD_S if _gain_profile(review) in (
                ROLE_THIGH_GAIN_PROFILE, ROLE_FRONT_THIGH_GAIN_PROFILE)
            else ID7_MAX_FEEDBACK_SPEED_RAD_S)


def _torque_limit(review, mid):
    if (_gain_profile(review) in (ROLE_THIGH_GAIN_PROFILE, ROLE_FRONT_THIGH_GAIN_PROFILE)
            and mid in ROLE_GROUP_IDS[review['role_group']]):
        return ROLE_THIGH_MAX_FEEDBACK_TORQUE_NM
    if (_gain_profile(review) in (ROLE_THIGH_GAIN_PROFILE, ROLE_FRONT_THIGH_GAIN_PROFILE)
            and mid in ROLE_GROUP_IDS['hip']):
        return 1.3
    return MAX_FEEDBACK_TORQUE_NM


def _drift_limit(review, mid):
    # The shared leg-trial default is 7 degrees. It would prematurely abort
    # the explicitly reviewed ten-degree upper-leg path. Keep the original
    # bound on every held joint and on every other diagnostic.
    if (_gain_profile(review) in (ROLE_THIGH_GAIN_PROFILE, ROLE_FRONT_THIGH_GAIN_PROFILE)
            and mid in ROLE_GROUP_IDS[review['role_group']]):
        return ROLE_THIGH_EXCURSION_RAD
    if (_diagnostic(review) == 'role-group-step1'
            and review.get('role_group') == 'front-hip'
            and review.get('amplitude_deg') == 10.
            and mid in ROLE_GROUP_IDS['front-hip']):
        return FRONT_HIP_STEP10_MAX_EXCURSION_RAD
    return MAX_DRIFT_RAD


def _limits(review, mid):
    diagnostic = _diagnostic(review)
    if diagnostic == 'id7-step1' and mid == ID7_MOTOR_ID:
        return ID7_MAX_TRACKING_ERROR_RAD, ID7_MAX_EXCURSION_RAD, ID7_FINAL_ERROR_RAD
    if diagnostic == 'role-group-step1' and mid in ROLE_GROUP_IDS[review['role_group']]:
        if review['role_group'] in ('thigh', 'front-thigh'):
            return (ROLE_THIGH_TRACKING_ERROR_RAD, ROLE_THIGH_EXCURSION_RAD,
                    ROLE_THIGH_FINAL_ERROR_RAD)
        if review['role_group'] == 'front-hip':
            return (MAX_TRACKING_ERROR_RAD,
                    FRONT_HIP_STEP10_MAX_EXCURSION_RAD if review.get('amplitude_deg') == 10.
                    else FRONT_HIP_MAX_EXCURSION_RAD,
                    FRONT_HIP_FINAL_ERROR_RAD)
        return ID7_MAX_TRACKING_ERROR_RAD, ID7_MAX_EXCURSION_RAD, ID7_FINAL_ERROR_RAD
    # Match the reviewed dual-Kp4 hold's ID4 static bias only at this held-axis
    # endpoint. Live tracking and excursion limits remain unchanged.
    if (mid == 4 and diagnostic == 'role-group-step1'
            and review.get('role_group') in ('thigh', 'front-thigh')
            and review.get('gain_profile') in (
                ROLE_THIGH_GAIN_PROFILE, ROLE_FRONT_THIGH_GAIN_PROFILE)):
        return MAX_TRACKING_ERROR_RAD, MAX_EXCURSION_RAD, ID4_ROLE_THIGH_HELD_FINAL_ERROR_RAD
    # The reviewed front-hip profile holds ID9 at its current position. Its
    # measured ~-1.04° static bias changes only the late endpoint check.
    if (mid == 9 and diagnostic == 'role-group-step1'
            and review.get('role_group') == 'front-hip'
            and review.get('gain_profile') == ROLE_FRONT_HIP_GAIN_PROFILE):
        return MAX_TRACKING_ERROR_RAD, MAX_EXCURSION_RAD, ID9_FRONT_HIP_HELD_FINAL_ERROR_RAD
    return MAX_TRACKING_ERROR_RAD, MAX_EXCURSION_RAD, ARRIVAL_ERROR_CANDIDATE_RAD


def _plan(centers, review):
    """Validate fresh Type17/raw starts and every sample, with no angle wrapping.

    This proves only a bounded raw-motor excursion. It does not claim a model
    coordinate or that the direction reaches a valid standing pose.
    """
    starts = _exact_id_map(review.get('start_raw_rad_by_id'), 'start_raw_rad_by_id', _real)
    diagnostic = _diagnostic(review)
    if diagnostic in ('id7-step1', 'role-group-step1'):
        def hold_or_sign(value, label):
            if type(value) is not int or value not in (-1, 0, 1):
                raise ValueError(f'{label} must be exact integer -1, 0 or +1')
            return value
        directions = _exact_id_map(review.get('raw_direction_by_id'),
                                   'raw_direction_by_id', hold_or_sign)
        moving = (ROLE_GROUP_IDS.get(review.get('role_group'))
                  if diagnostic == 'role-group-step1' else frozenset((ID7_MOTOR_ID,)))
        if moving is None or any((directions[mid] in (-1, 1)) != (mid in moving)
                                 for mid in ALL_IDS):
            raise ValueError('Only the four selected role joints may move')
    else:
        directions = _exact_id_map(review.get('raw_direction_by_id'),
                                   'raw_direction_by_id', _sign)
    amplitude = _real(review.get('amplitude_deg'), 'amplitude_deg')
    maximum = (10. if diagnostic == 'role-group-step1'
               and review.get('role_group') in ('thigh', 'front-thigh')
               else 10. if diagnostic == 'role-group-step1'
               and review.get('role_group') == 'front-hip'
               else 1. if diagnostic in ('id7-step1', 'role-group-step1')
               else MAX_AMPLITUDE_DEG)
    if not 0. < amplitude <= maximum:
        raise ValueError(f'Step amplitude must be positive and at most {maximum:g} degree(s)')
    if set(centers) != set(ALL_IDS):
        raise ValueError('No complete twelve-axis fresh start')
    targets = {}
    for mid in ALL_IDS:
        center = _real(centers[mid], f'ID{mid} fresh center')
        # A disabled leg can sag substantially after STOP (the measured toe
        # drift reached 49 degrees). For a role-group diagnostic, targets are
        # rebuilt from the fresh settled twelve-axis snapshot, never from the
        # old hold snapshot. Reject a larger change or any full-turn wrap;
        # still bound the actual commanded path by its reviewed amplitude.
        start_limit = (ROLE_GROUP_MAX_START_MISMATCH_RAD
                       if diagnostic == 'role-group-step1' else MAX_START_MISMATCH_RAD)
        if abs(center - starts[mid]) > start_limit:
            raise ValueError(f'ID{mid} fresh raw start differs from reviewed start; no wrap')
        # The existing STEP5 wire phase has a strict <=5-degree offset check.
        # Leave a sub-nanoradian floating-point margin for center+offset and
        # its later subtraction; this does not alter the physical 5-degree plan.
        offset = math.radians(amplitude)
        if diagnostic == 'role-group-step1' and review.get('role_group') == 'front-hip':
            offset = min(offset, math.radians(amplitude) - 1e-12)
        end = center + directions[mid]*offset
        motion_request(phase=_motion_phase(review, mid),
                       center_rad=center, offset_rad=end-center, motor_id=mid)
        targets[mid] = end
    samples = []
    for tick in range(ACTIVE_TICKS):
        fraction = quintic_fraction(min(tick/RAMP_TICKS, 1.))
        sample = (dict(centers) if tick == 0 else dict(targets) if fraction == 1.
                  else {mid: centers[mid] + (targets[mid]-centers[mid])*fraction for mid in ALL_IDS})
        samples.append(sample)
    if samples[0] != centers or samples[-1] != targets:
        raise AssertionError('Finite trajectory endpoints changed')
    return tuple(samples)


def _review(expected_uids, value, preflight_only):
    """Require an independently reviewed, same-boot path before any I/O."""
    expected = _expected_identities(expected_uids, ALL_IDS)
    if type(preflight_only) is not bool:
        raise ValueError('Explicit boolean preflight is required')
    value = deepcopy(value)
    diagnostic = _diagnostic(value)
    if (type(value) is not dict
            or value.get('motor_ids') != list(ALL_IDS)
            or any(type(i) is not int for i in value.get('motor_ids', []))
            or value.get('firmware') != '0.5.0.13' or not _digest(value.get('sha256'))
            or value.get('review_complete') is not True
            or value.get('source_files_verified') is not True
            or value.get('gain_profile') != _gain_profile(value)
            or value.get('old_raw_target_reused') is not False
            or any(value.get(flag) is not False for flag in
                   ('calibration_verified', 'model_mapping_verified',
                    'learned_policy_allowed', 'standing_allowed', 'automatic_retry_allowed'))
            or value.get('current_hold_status') != 'CURRENT_HOLD_COMPLETED_RESET_CONFIRMED'
            or value.get('current_hold_stop_confirmed') is not True
            or not _digest(value.get('current_hold_summary_sha256'))
            or value.get('current_hold_gain_profile') != GAIN_PROFILE
            or type(value.get('supported_step_authorized')) is not bool):
        raise ValueError('An explicit current-boot raw path review is required')
    if diagnostic == 'id7-step1' and (
            value.get('source_active_step2_abort_sha256') is None
            or not _digest(value.get('source_active_step2_abort_sha256'))
            or type(value.get('id7_joint_inspection_verified')) is not bool
            or (not preflight_only and value['id7_joint_inspection_verified'] is not True)):
        raise ValueError('ID7-only diagnostic requires pinned aborted trial and inspection')
    if diagnostic == 'role-group-step1' and value.get('role_group') not in ROLE_GROUP_IDS:
        raise ValueError('A single known role group is required')
    if diagnostic == 'role-group-step1':
        front_thigh = value['role_group'] == 'front-thigh'
        front_hip = value['role_group'] == 'front-hip'
        if (value['scope'] == FRONT_THIGH_REVIEW_SCOPE) != front_thigh:
            raise ValueError('Role-group scope does not match the selected joints')
        if (value['scope'] == FRONT_HIP_REVIEW_SCOPE) != front_hip:
            raise ValueError('Role-group scope does not match the selected joints')
        if front_thigh and value.get('raw_direction_by_id') != {
                str(mid): (-1 if mid == 2 else 1 if mid == 5 else 0)
                for mid in ALL_IDS}:
            raise ValueError('Front thighs require exact toward-face raw directions')
        if front_hip and (value.get('direction_profile') != 'front-hip-mirrored'
                          or value.get('raw_direction_by_id') != {
                              str(mid): (1 if mid == 3 else -1 if mid == 6 else 0)
                              for mid in ALL_IDS}):
            raise ValueError('Front hips require exact mirrored raw directions')
    physical_flags = ('raw_direction_reviewed_for_diagnostic',
                      'swept_clearance_verified', 'support_stand_verified',
                      'feet_clear_verified', 'hands_clear_verified', 'physical_cutoff_ready')
    if (any(type(value.get(flag)) is not bool for flag in physical_flags)
            or (preflight_only and value['supported_step_authorized'] is not False)
            or (not preflight_only and (value['supported_step_authorized'] is not True
                                       or any(value[flag] is not True for flag in physical_flags)))):
        raise ValueError('No active raw step without every current physical/path gate')
    if _expected_identities(value.get('motor_uids'), ALL_IDS) != expected:
        raise ValueError('Step review identities differ from the supplied twelve identities')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if not boot or value.get('boot_id') != boot or value.get('current_hold_boot_id') != boot:
        raise ValueError('Step review and passing hold must belong to current boot')
    # Reject malformed or stale geometry before opening a UART. Actual fresh
    # starts are checked after the disabled settled window and before Enable.
    reviewed_starts = _exact_id_map(value.get('start_raw_rad_by_id'), 'start_raw_rad_by_id', _real)
    _plan(reviewed_starts, value)
    return {int(i): uid for i, uid in expected.items()}, value


def run_fullbody_step2(transports, expected_uids, check_interrupt, emit, *, validated_review,
                       preflight_only=True, clock=time.monotonic, wait=time.sleep):
    """Run disabled preflight by default, or one finite raw diagnostic.

    ``transports`` is exactly {'front': ..., 'rear': ...}, with distinct single
    serial owners. ``validated_review`` follows REVIEW_SCHEMA/REVIEW_SCOPE;
    its source_files_verified assertion is the frozen wrapper's responsibility.
    The reviewed role selects its fixed gain profile. This is not an automatic
    standing or policy handoff.
    Callbacks must be bounded and thread-safe; emit is serialized here and all
    its time is included in deadlines. A blocked OS write cannot be cancelled
    safely by having the parent touch another worker's UART.
    """
    expected, review = _review(expected_uids, validated_review, preflight_only)
    diagnostic = _diagnostic(review)

    def motion_phase(mid):
        return _motion_phase(review, mid)
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
            track_limit, excursion_limit, _ = _limits(review, mid)
            if abs(value.protocol_position_rad - target) > track_limit:
                raise RuntimeError(f'ID{mid} raw step tracking error exceeded limit')
            if abs(value.protocol_position_rad - center) > excursion_limit:
                raise RuntimeError(f'ID{mid} raw step excursion exceeded limit')
            if abs(value.torque_nm) > _torque_limit(review, mid):
                raise RuntimeError(f'ID{mid} raw step feedback torque monitor tripped')
            moving_guard = ((diagnostic == 'id7-step1' and mid == ID7_MOTOR_ID)
                            or (diagnostic == 'role-group-step1'
                                and mid in ROLE_GROUP_IDS[review['role_group']]))
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
            if (((diagnostic == 'id7-step1' and mid == ID7_MOTOR_ID)
                 or (diagnostic == 'role-group-step1'
                     and mid in ROLE_GROUP_IDS[review['role_group']]))
                    and wire_stage in ('enabling', 'trajectory')
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
                    or abs(values['velocity']) > .5 or not 35 <= values['voltage'] <= 43):
                raise RuntimeError(f'ID{mid} disabled parameter envelope failed')
            motion_request(phase=motion_phase(mid), center_rad=values['position'], motor_id=mid)
            return values

        def all_guard(mode, *, disabled=False):
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
                if mode is None:
                    if value.mode_state not in (0, 2):
                        raise RuntimeError(f'ID{mid} unexpected transition mode')
                    required_mode = value.mode_state
                else:
                    required_mode = mode
                check_feedback(value, all_centers[mid], received, now, required_mode=required_mode,
                               max_drift_rad=(SETTLED_LIMITS['maximum_center_drift_rad']
                                              if disabled else _drift_limit(review, mid)),
                               max_age_s=(.125 if wire_stage == 'trajectory'
                                          and last_completed_tick is None
                                          else MAX_FEEDBACK_AGE_S))
                if (((diagnostic == 'id7-step1' and mid == ID7_MOTOR_ID)
                     or (diagnostic == 'role-group-step1'
                         and mid in ROLE_GROUP_IDS[review['role_group']])) and not disabled
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
                if t.parameter(mid, 'can_timeout')['value'] != 4000:
                    raise RuntimeError(f'ID{mid} watchdog readback mismatch')
                report['motors'][mid]['watchdog_readback_ticks'] = 4000
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
            with state_lock:
                shared['centers'].update(centers)
                shared['disabled'].update(final_rows)
            log({'kind': 'fullbody_bus_ready', 'bus': name, 'centers': dict(centers)})
            sync('both_workers_ready')
            all_guard(0, disabled=True)
            with state_lock:
                fresh_centers = dict(shared['centers'])
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
                # deadline, not charged against the nine-second trajectory.
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
                all_guard(2)
                t.pre_send_guard = lambda: all_guard(2)
                wire_stage = 'trajectory'
                start = sync('active_raw_step_start')
                active_budget = start + ACTIVE_BUDGET_S
                active_previous.update({mid: (value.protocol_position_rad, received)
                                        for mid, (value, received) in t.latest.items() if mid in ids})
                t.active_deadline = active_budget
                ticks, mode = ACTIVE_TICKS, 2
            report['start_monotonic_s'] = start
            final_hold = {mid: [] for mid in ids}
            for tick in range(ticks):
                due, deadline = start + tick * CYCLE_S, start + (tick + 1) * CYCLE_S
                until(due)
                if clock() >= deadline:
                    raise RuntimeError('Fullbody cycle missed50ms before batch')
                all_guard(mode)
                t.active_deadline = min(deadline, active_budget) if active_budget is not None else deadline
                if not preflight_only:
                    sample = plan[tick]
                    expected_active_wires = {mid: motion_request(
                        phase=motion_phase(mid), center_rad=centers[mid],
                        offset_rad=sample[mid]-centers[mid], motor_id=mid) for mid in ids}
                    active_wires_sent.clear()
                # The six selected-bus writes share one receive window even
                # on tick zero. Each raw_send still performs the all-twelve
                # snapshot guard and exact-wire check before that UART write.
                found = batch([neutral(mid) if preflight_only else expected_active_wires[mid]
                               for mid in ids], mode)
                if not preflight_only and active_wires_sent != set(ids):
                    raise RuntimeError('Exact six-wire raw step batch was incomplete')
                log({'kind': 'fullbody_raw_step_cycle', 'bus': name, 'tick': tick,
                     'preflight_only': preflight_only, 'due_monotonic_s': due,
                     'completed_monotonic_s': clock(), 'deadline_monotonic_s': deadline})
                if clock() > deadline:
                    raise RuntimeError('Fullbody cycle missed50ms including replies/logging')
                report['cycle_count'] += 1
                if not preflight_only and tick >= RAMP_TICKS:
                    for mid, (value, received) in found.items():
                        final_hold[mid].append((received, value.protocol_position_rad))
                sync(f'cycle_{tick}_complete')
                if clock() > deadline:
                    raise RuntimeError('Peer cycle/barrier missed50ms')
            until(start + ticks * CYCLE_S)
            if not preflight_only:
                for mid, samples in final_hold.items():
                    times = [row[0] for row in samples]
                    if (len(times) != END_HOLD_TICKS or times[-1] - times[0] < MIN_OBSERVED_HOLD_NS / 1e9
                            or any(not 0 < b - a <= MAX_HOLD_GAP_NS / 1e9 for a, b in zip(times, times[1:]))):
                        raise RuntimeError(f'ID{mid} final hold observation coverage failed')
                report['final_hold_samples'] = final_hold
                for mid, samples in final_hold.items():
                    if abs(samples[-1][1] - plan[-1][mid]) > _limits(review, mid)[2]:
                        raise RuntimeError(f'ID{mid} final raw step error exceeds diagnostic limit')
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
    completed_label = {'id7-step1': 'RAW_ID7_STEP1_COMPLETED_RESET_CONFIRMED',
                       'role-group-step1': 'RAW_ROLE_GROUP_STEP1_COMPLETED_RESET_CONFIRMED',
                       'fullbody-step2': 'RAW_STEP2_COMPLETED_RESET_CONFIRMED'}[diagnostic]
    return {'status': ('PREFLIGHT_PASSED_RESET_CONFIRMED' if preflight_only else
                       completed_label) if complete else 'ABORTED',
            'preflight_only': preflight_only, 'preflight_completed': complete and preflight_only,
            'motion_completed': complete and not preflight_only, 'stop_confirmed': stopped,
            'workers': reports, 'errors': errors, 'review': review,
            'calibration_verified': False, 'learned_policy_allowed': False, 'standing_allowed': False,
            'model_mapping_verified': False, 'l_target_replay_allowed': False,
            'continuous_hold_proven': False, 'automatic_retry': False,
            'gain_profile': _gain_profile(review), 'raw_diagnostic_only': True,
            'interleaved_feedback': interleaved_feedback,
            'diagnostic': diagnostic, 'moving_motor_ids': (
                [ID7_MOTOR_ID] if diagnostic == 'id7-step1' else
                sorted(ROLE_GROUP_IDS[review['role_group']]) if diagnostic == 'role-group-step1'
                else list(ALL_IDS)),
            'amplitude_deg': review['amplitude_deg'], 'ramp_s': RAMP_TICKS*CYCLE_S,
            'endpoint_hold_s': END_HOLD_TICKS*CYCLE_S,
            'Kp': 0. if preflight_only else None,
            'Kp_by_motor_id': {mid: 0. if preflight_only else (
                12. if _motion_phase(review, mid) in (
                    TrialPhase.POSITION_ROLE_THIGH_KP12,
                    TrialPhase.POSITION_ROLE_HIP_HOLD_KP12,
                    TrialPhase.POSITION_ROLE_FRONT_HIP_KP12,
                    TrialPhase.POSITION_ROLE_FRONT_HIP_KP12_STEP10)
                else 6. if _motion_phase(review, mid) in (
                    TrialPhase.POSITION_ROLE_THIGH_KP6,
                    TrialPhase.POSITION_ROLE_FRONT_HIP_KP6,
                    TrialPhase.POSITION_STEP5_RR_HIP_KP6)
                else 4. if mid in KP4_IDS else 3.) for mid in ALL_IDS},
            'Kd': 0. if preflight_only else .15,
            'torque_feedforward_nm': 0., 'cycle_deadline_s': CYCLE_S}
