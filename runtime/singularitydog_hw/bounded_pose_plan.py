"""Pure, diagnostic one-leg absolute target planning; no transport or actuation.

The existing trial envelope is preserved: at most5 degrees from fresh raw
centers, a4s cosine ramp and1s target hold, ending at5s. Stored L references are
targets, not fresh centers or proof of calibration/turn continuity. Nothing is
wrapped, segmented, re-zeroed, retried, or automatically approved.

The1-degree arrival tolerance below is a NEW diagnostic candidate, not a proven
physical accuracy limit. The nominal1s hold interval requires20 new observations
(4.00..4.95s), with at least0.90s of actual source coverage per axis. Passing is
not evidence of1.000s continuous physical holding. Elapsed time cannot fill a
missing sample or coverage gap, and never grants motor permission.
"""
from dataclasses import dataclass
import math

LEGS = {'FR': (1, 2, 3), 'FL': (4, 5, 6), 'RR': (7, 8, 9), 'RL': (10, 11, 12)}
NS = 1_000_000_000
RAMP_NS, HOLD_NS, DURATION_NS, ACTIVE_BUDGET_NS = 4*NS, NS, 5*NS, 6*NS
CYCLE_NS, MAX_LATENESS_NS, MAX_AGE_NS = 50_000_000, 20_000_000, 100_000_000
MAX_HOLD_GAP_NS, HOLD_SAMPLES = CYCLE_NS+MAX_LATENESS_NS, 20
MIN_OBSERVED_HOLD_NS = 900_000_000  # Diagnostic candidate within the nominal1s hold.
MAX_DELTA_RAD, MAX_DRIFT_RAD = math.radians(5), math.radians(7)
ARRIVAL_ERROR_CANDIDATE_RAD = math.radians(1)
MATCHED_START_TOLERANCE_RAD = math.radians(.5)  # Fixed diagnostic comparison candidate.
SETTLING_MAX_ERROR_RAD = math.radians(2)  # Observation abort limit, never arrival success.
SETTLING_MAX_WORSENING_RAD = math.radians(.25)
POSITION_MIN, POSITION_MAX = -12.57, 12.57
FLAGS = {'output_allowed': False, 'motor_output_available': False,
         'approved_for_runtime': False, 'absolute_accuracy_verified': False,
         'continuous_hold_proven': False, 'calibration_modified': False,
         'turn_continuity_verified': False}


def _require(value, message):
    if not value:
        raise ValueError(message)


def _real(value, label):
    _require(type(value) in (float, int), label+' must be a finite real number')
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ValueError(label+' must be finite') from None
    _require(math.isfinite(result), label+' must be finite')
    return result


def _timestamp(value, label):
    _require(type(value) is int and value >= 0, label+' must be nonnegative integer nanoseconds')
    return value


def _mapping(value, ids, label):
    _require(type(value) is dict, label+' must be an object keyed by the selected three IDs')
    _require(all(type(k) in (int, str) and str(k) in {str(i) for i in ids} for k in value),
             label+' contains an ID outside the selected leg')
    result = {int(k): v for k, v in value.items()}
    _require(len(result) == len(value) and set(result) == set(ids),
             label+' must contain the exact three IDs without duplicates')
    return result


@dataclass(frozen=True)
class Center:
    position_rad: float
    request_ns: int
    received_ns: int

    def validate(self):
        _real(self.position_rad, 'center position')
        _timestamp(self.request_ns, 'center request_ns')
        _timestamp(self.received_ns, 'center received_ns')
        _require(self.request_ns < self.received_ns, 'Center response must strictly follow request')


@dataclass(frozen=True)
class PosePlan:
    leg: str
    ids: tuple
    centers: tuple
    targets_rad: tuple
    planned_at_ns: int

    def validate(self):
        _require(type(self.leg) is str and self.leg in LEGS, 'Select exactly one named leg')
        _require(type(self.ids) is tuple and self.ids == LEGS[self.leg]
                 and all(type(i) is int for i in self.ids), 'Wrong leg ID order')
        _require(type(self.centers) is tuple and len(self.centers) == 3
                 and all(type(c) is Center for c in self.centers), 'Three immutable centers required')
        _require(type(self.targets_rad) is tuple and len(self.targets_rad) == 3,
                 'Three explicit absolute raw targets required')
        _timestamp(self.planned_at_ns, 'planned_at_ns')
        for mid, center, target in zip(self.ids, self.centers, self.targets_rad):
            center.validate()
            target = _real(target, f'ID{mid} target')
            _require(0 <= self.planned_at_ns-center.received_ns <= MAX_AGE_NS,
                     f'ID{mid} center is stale or from the future')
            _require(0 <= self.planned_at_ns-center.request_ns <= MAX_AGE_NS,
                     f'ID{mid} center request is stale or from the future')
            _require(POSITION_MIN+MAX_DELTA_RAD <= center.position_rad <= POSITION_MAX-MAX_DELTA_RAD,
                     f'ID{mid} center lacks existing5-degree codec headroom')
            _require(POSITION_MIN <= target <= POSITION_MAX, f'ID{mid} target outside raw encoding range')
            _require(center.position_rad-MAX_DELTA_RAD <= target <= center.position_rad+MAX_DELTA_RAD,
                     f'ID{mid} absolute delta exceeds5 degrees; no wrap or automatic segmentation')

    def as_dict(self):
        self.validate()
        return {**FLAGS, 'schema_version': 1, 'leg': self.leg, 'ids': list(self.ids),
                'coordinate_frame': 'raw motor radians; no wrapping',
                'planned_at_ns': self.planned_at_ns,
                'centers': {str(i): {'position_rad': c.position_rad, 'request_ns': c.request_ns,
                                    'received_ns': c.received_ns} for i,c in zip(self.ids,self.centers)},
                'absolute_targets_rad': dict(zip(map(str,self.ids),self.targets_rad)),
                'ramp_ns': RAMP_NS, 'hold_ns': HOLD_NS, 'duration_ns': DURATION_NS,
                'active_budget_ns': ACTIVE_BUDGET_NS, 'cycle_ns': CYCLE_NS,
                'arrival_error_candidate_rad': ARRIVAL_ERROR_CANDIDATE_RAD,
                'arrival_threshold_status': 'diagnostic candidate; not physical accuracy evidence',
                'hold_samples_required': HOLD_SAMPLES,
                'hold_observed_span_required_ns': MIN_OBSERVED_HOLD_NS,
                'hold_coverage_status': 'nominal1s interval; candidate minimum0.90s observed, not1.000s proven',
                'max_hold_gap_ns': MAX_HOLD_GAP_NS, 'max_feedback_age_ns': MAX_AGE_NS,
                'automatic_retry': False, 'automatic_segmentation': False, 'persistent_hold': False}


def normalize_absolute_targets(leg, absolute_targets):
    """Validate target structure/range before any caller accesses a transport."""
    _require(type(leg) is str and leg in LEGS, 'Select exactly one named leg')
    targets = _mapping(absolute_targets, LEGS[leg], 'absolute_targets')
    result = {mid: _real(targets[mid], f'ID{mid} target') for mid in LEGS[leg]}
    _require(all(POSITION_MIN <= value <= POSITION_MAX for value in result.values()),
             'Absolute target outside raw encoding range')
    return result


def normalize_matched_start_positions(leg, positions):
    """Copy exact raw references; these are not commands or calibrated joint zeros."""
    _require(type(leg) is str and leg in LEGS, 'Select exactly one named leg')
    positions = _mapping(positions, LEGS[leg], 'matched_start_positions')
    result = {mid: _real(positions[mid], f'ID{mid} matched start') for mid in LEGS[leg]}
    _require(all(POSITION_MIN <= value <= POSITION_MAX for value in result.values()),
             'Matched start reference outside raw encoding range')
    return result


def validate_matched_start_tolerance(tolerance_rad):
    """This candidate has one fixed tolerance, with no caller-selected relaxation."""
    tolerance = _real(tolerance_rad, 'matched_start_tolerance_rad')
    _require(tolerance == MATCHED_START_TOLERANCE_RAD,
             'Matched start candidate requires the fixed0.5-degree tolerance')
    return tolerance


def evaluate_matched_start(plan, positions, *, now_ns,
                           tolerance_rad=MATCHED_START_TOLERANCE_RAD):
    """Compare final fresh disabled raw centers, without wrapping or moving them.

    All selected axes must lie within reference +/- the fixed0.5-degree candidate.
    Endpoint comparisons have no numerical epsilon. A match is diagnostic only;
    identity, boot/turn continuity and the reference provenance belong to the caller.
    """
    _require(type(plan) is PosePlan, 'Validated PosePlan required')
    plan.validate()
    references = normalize_matched_start_positions(plan.leg, positions)
    tolerance = validate_matched_start_tolerance(tolerance_rad)
    checked = _timestamp(now_ns, 'matched start now_ns')
    report = {**FLAGS, 'status': 'BLOCKED', 'passed': False, 'checked_ns': checked,
              'tolerance_rad': tolerance, 'threshold_status': 'fixed diagnostic candidate; not calibration evidence',
              'coordinate_frame': 'absolute raw motor radians; no wrapping',
              'reference_positions_rad': {str(i): references[i] for i in plan.ids},
              'automatic_repositioning': False, 'motors': {}, 'errors': []}
    for mid, center in zip(plan.ids, plan.centers):
        reference = references[mid]
        fresh = (checked >= plan.planned_at_ns
                 and 0 <= checked-center.request_ns <= MAX_AGE_NS
                 and 0 <= checked-center.received_ns <= MAX_AGE_NS)
        within = reference-tolerance <= center.position_rad <= reference+tolerance
        report['motors'][str(mid)] = {
            'reference_position_rad': reference, 'observed_position_rad': center.position_rad,
            'delta_rad': center.position_rad-reference, 'within_tolerance': within,
            'source_fresh': fresh, 'request_ns': center.request_ns, 'received_ns': center.received_ns}
        if not fresh:report['errors'].append(f'ID{mid} matched start center is stale or from the future')
        if not within:report['errors'].append(f'ID{mid} matched start differs by more than0.5-degree candidate; no repositioning')
    report['passed'] = not report['errors']
    if report['passed']:report['status'] = 'DIAGNOSTIC_START_MATCH_CANDIDATE_MET'
    return report


def build_plan(leg, centers, absolute_targets, *, now_ns):
    """Build from exact-ID maps; centers require position_rad/request_ns/received_ns.

    Freshness is checked at the explicit planning instant only. A future runner
    must repeat freshness, identity, turn/pose evidence and every hardware gate
    before enable; this plan contains no permission to send any frame.
    """
    _require(type(leg) is str and leg in LEGS, 'Select exactly one named leg')
    ids = LEGS[leg]
    centers = _mapping(centers, ids, 'centers')
    targets = normalize_absolute_targets(leg, absolute_targets)
    rows = []
    for mid in ids:
        row = centers[mid]
        _require(type(row) is dict and set(row) == {'position_rad','request_ns','received_ns'},
                 f'ID{mid} center requires exact position/request/receive fields')
        rows.append(Center(_real(row['position_rad'], 'center position'), row['request_ns'], row['received_ns']))
    plan = PosePlan(leg, ids, tuple(rows), tuple(_real(targets[i], 'target') for i in ids), now_ns)
    plan.validate()
    return plan


def bounded_offset(center, target):
    """Endpoint comparison avoids cancellation without expanding the5deg bound."""
    center, target = _real(center, 'center'), _real(target, 'target')
    low, high = center-MAX_DELTA_RAD, center+MAX_DELTA_RAD
    _require(low <= target <= high, 'Absolute delta exceeds5 degrees')
    if target == low:
        return -MAX_DELTA_RAD
    if target == high:
        return MAX_DELTA_RAD
    return target-center


def offsets_at(plan, elapsed_ns):
    """Codec offsets relative to the exact fresh plan centers; no recentering."""
    _require(type(plan) is PosePlan, 'Validated PosePlan required')
    plan.validate()
    _timestamp(elapsed_ns, 'elapsed_ns')
    _require(elapsed_ns <= DURATION_NS, 'Finite5s plan ended')
    fraction = .5*(1-math.cos(math.pi*min(elapsed_ns, RAMP_NS)/RAMP_NS))
    return {mid: bounded_offset(c.position_rad,t)*fraction
            for mid,c,t in zip(plan.ids,plan.centers,plan.targets_rad)}


def target_at(plan, elapsed_ns):
    """Absolute targets for one finite4s ramp+1s hold; no continuation after5s."""
    offsets = offsets_at(plan, elapsed_ns)
    return {mid: c.position_rad+offsets[mid] for mid,c in zip(plan.ids,plan.centers)}


def evaluate_hold(plan, samples, *, run_start_ns, ended_ns):
    """The unchanged strict1-degree arrival/hold candidate evaluation."""
    return _evaluate_hold_samples(plan, samples, run_start_ns=run_start_ns, ended_ns=ended_ns,
                                  settling_observation=False)


def evaluate_settling_observation(plan, samples, *, run_start_ns, ended_ns):
    """Evaluate finite RR observation data independently of strict arrival success.

    Shares source/index/freshness/coverage validation with evaluate_hold without
    modifying samples or targets. The2-degree bound is an observation abort
    limit, not a replacement arrival tolerance. This grants no output permission.
    """
    if type(plan) is not PosePlan or plan.leg != 'RR' or plan.ids != (7,8,9):
        raise ValueError('Settling observation requires an explicit RR plan')
    return _evaluate_settling_observation(plan, samples, run_start_ns=run_start_ns, ended_ns=ended_ns)


def evaluate_fr_settling_observation(plan, samples, *, run_start_ns, ended_ns):
    """Evaluate a finite FR diagnostic without treating 2-degree error as arrival."""
    if type(plan) is not PosePlan or plan.leg != 'FR' or plan.ids != (1,2,3):
        raise ValueError('FR settling observation requires an explicit FR plan')
    return _evaluate_settling_observation(plan, samples, run_start_ns=run_start_ns, ended_ns=ended_ns)


def _evaluate_settling_observation(plan, samples, *, run_start_ns, ended_ns):
    checked = _evaluate_hold_samples(plan, samples, run_start_ns=run_start_ns, ended_ns=ended_ns,
                                     settling_observation=True)
    errors = list(checked['errors'])
    baseline, worsening = {}, {}
    if not errors:
        # All20 original rows already passed the full source and shape checks.
        # This baseline is the first COMPLETE row, never a rolling minimum.
        for mid, target in zip(plan.ids,plan.targets_rad):
            absolute_errors = [abs(_mapping(row['joints'],plan.ids,'sample joints')[mid]['position_rad']-target)
                               for row in samples]
            baseline[str(mid)] = absolute_errors[0]
            worsening[str(mid)] = max(absolute_errors)-absolute_errors[0]
            if any(error > absolute_errors[0]+SETTLING_MAX_WORSENING_RAD for error in absolute_errors):
                errors.append(f'ID{mid} target error worsened over0.25-degree observation limit')
    complete = not errors and checked['elapsed_completed']
    return {**FLAGS, 'status': 'SETTLING_OBSERVATION_DATA_COMPLETE' if complete else 'BLOCKED',
            'data_complete': complete, 'errors': errors, 'elapsed_completed': checked['elapsed_completed'],
            'observation_only': True, 'arrival_threshold_unchanged_rad': ARRIVAL_ERROR_CANDIDATE_RAD,
            'max_abs_target_error_observation_rad': SETTLING_MAX_ERROR_RAD,
            'max_abs_error_worsening_observation_rad': SETTLING_MAX_WORSENING_RAD,
            'first_hold_abs_error_rad': baseline, 'maximum_abs_error_worsening_rad': worsening,
            'nominal_hold_ns': HOLD_NS, 'minimum_observed_hold_candidate_ns': MIN_OBSERVED_HOLD_NS,
            'motors': checked['motors']}


def _evaluate_hold_samples(plan, samples, *, run_start_ns, ended_ns, settling_observation):
    """Evaluate diagnostic evidence; never send, retry, fill, wrap or approve.

    Each of20 rows: sample_index(0..19), checked_ns, joints(exact selected IDs).
    Each joint row: position_rad, request_ns, received_ns. Nominal observations
    requests are scheduled at4s+n*50ms, with at most20ms start lateness. Receive
    completion may take longer, subject to100ms source freshness; every source must
    lie in4..5s and be new. Require each axis's actual observed span >=0.90s,
    explicitly a candidate coverage threshold for the nominal1s hold interval.
    Missing evidence is rejected, not inferred from elapsed time.
    """
    report = {**FLAGS, 'status': 'BLOCKED', 'errors': [], 'elapsed_completed': False,
              'arrival_candidate_met': False, 'hold_candidate_met': False,
              'nominal_hold_ns': HOLD_NS, 'minimum_observed_hold_candidate_ns': MIN_OBSERVED_HOLD_NS,
              'arrival_error_candidate_rad': ARRIVAL_ERROR_CANDIDATE_RAD,
              'threshold_status': 'diagnostic candidate; not actuation authorization', 'motors': {}}
    try:
        _require(type(plan) is PosePlan, 'Validated PosePlan required')
        plan.validate()
        _timestamp(run_start_ns, 'run_start_ns'); _timestamp(ended_ns, 'ended_ns')
        _require(run_start_ns >= plan.planned_at_ns and ended_ns >= run_start_ns,
                 'Run timing predates plan or is reversed')
        _require(all(0 <= run_start_ns-c.request_ns <= MAX_AGE_NS
                     and 0 <= run_start_ns-c.received_ns <= MAX_AGE_NS for c in plan.centers),
                 'Centers are no longer fresh at run start')
        report['elapsed_completed'] = ended_ns-run_start_ns >= DURATION_NS
        _require(ended_ns-run_start_ns <= ACTIVE_BUDGET_NS, 'Existing6s active budget exceeded')
        _require(type(samples) in (list, tuple), 'Explicit sample list required')
    except (ValueError, TypeError, KeyError) as error:
        report['errors'].append(str(error)); return report
    if len(samples) != HOLD_SAMPLES:
        report['errors'].append('Exactly20 hold observations required; missing data is not filled')
    times = {i: [] for i in plan.ids}; requests = {i: [] for i in plan.ids}
    errors = {i: [] for i in plan.ids}; checked_previous = None
    first_valid = False
    # Excess input is rejected above and never creates unbounded diagnostic work.
    for index, row in enumerate(samples[:HOLD_SAMPLES]):
        try:
            _require(type(row) is dict and set(row) == {'sample_index','checked_ns','joints'},
                     'Each observation requires exact index/check/joints fields')
            _require(type(row['sample_index']) is int and row['sample_index'] == index,
                     'Missing, duplicate or reordered observation index')
            checked = _timestamp(row['checked_ns'], 'checked_ns')
            due = run_start_ns+RAMP_NS+index*CYCLE_NS
            _require(due <= checked <= ended_ns,
                     'Observation outside scheduled interval or beyond run end')
            _require(checked_previous is None or checked > checked_previous,
                     'Non-increasing observation check time')
            joints = _mapping(row['joints'], plan.ids, 'sample joints')
            updates = []
            for mid, center, target in zip(plan.ids,plan.centers,plan.targets_rad):
                item = joints[mid]
                _require(type(item) is dict and set(item) == {'position_rad','request_ns','received_ns'},
                         f'ID{mid} sample requires exact position/request/receive fields')
                position = _real(item['position_rad'], f'ID{mid} position')
                requested = _timestamp(item['request_ns'], f'ID{mid} request_ns')
                received = _timestamp(item['received_ns'], f'ID{mid} received_ns')
                _require(run_start_ns+RAMP_NS <= requested < received <= run_start_ns+DURATION_NS,
                         f'ID{mid} source outside actual4..5s hold interval')
                _require(due <= requested <= due+MAX_LATENESS_NS,
                         f'ID{mid} request start missed scheduled20ms lateness limit')
                _require(0 <= checked-received <= MAX_AGE_NS, f'ID{mid} stale or future sample')
                _require(0 <= checked-requested <= MAX_AGE_NS, f'ID{mid} stale source request')
                _require(not times[mid] or (received > times[mid][-1] and requested > requests[mid][-1]),
                         f'ID{mid} duplicate/reordered source timestamps')
                _require(not times[mid] or received-times[mid][-1] <= MAX_HOLD_GAP_NS,
                         f'ID{mid} source gap exceeds70ms')
                _require(POSITION_MIN <= position <= POSITION_MAX and abs(position-center.position_rad) <= MAX_DRIFT_RAD,
                         f'ID{mid} raw position outside existing7-degree trial envelope')
                error = position-target
                if settling_observation:
                    _require(target-SETTLING_MAX_ERROR_RAD <= position <= target+SETTLING_MAX_ERROR_RAD,
                             f'ID{mid} target error exceeds2-degree observation limit')
                else:
                    _require(abs(error) <= ARRIVAL_ERROR_CANDIDATE_RAD,
                             f'ID{mid} target error exceeds diagnostic1-degree candidate')
                updates.append((mid, requested, received, error))
            # Atomic row acceptance prevents partial axes from inflating coverage.
            for mid, requested, received, error in updates:
                requests[mid].append(requested); times[mid].append(received); errors[mid].append(error)
            checked_previous = checked
            if index == 0: first_valid = True
        except (ValueError, TypeError, KeyError) as error:
            report['errors'].append(f'sample{index}: {error}')
    for mid in plan.ids:
        span = times[mid][-1]-times[mid][0] if len(times[mid]) >= 2 else 0
        report['motors'][str(mid)] = {'accepted_samples': len(times[mid]), 'observed_span_ns': span,
            'maximum_source_gap_ns': max((b-a for a,b in zip(times[mid],times[mid][1:])), default=None),
            'max_abs_target_error_rad': max(map(abs,errors[mid]), default=None),
            'final_target_error_rad': errors[mid][-1] if errors[mid] else None,
            'received_ns': times[mid], 'request_ns': requests[mid]}
        if span < MIN_OBSERVED_HOLD_NS:
            report['errors'].append(f'ID{mid} actual observation span below candidate0.90s; elapsed clock is not evidence')
    report['arrival_candidate_met'] = first_valid
    if not report['elapsed_completed']:
        report['errors'].append('Finite5s plan has not elapsed')
    report['hold_candidate_met'] = first_valid and not report['errors']
    if report['hold_candidate_met']:
        report['status'] = 'DIAGNOSTIC_TARGET_HOLD_CANDIDATE_MET'
    return report
