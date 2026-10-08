"""Pure four-bus model-input planning; no files, devices, Torch or models opened.

``prepare_model_plan`` accepts pinned JSON envelopes, not file locators to read.
An envelope is ``{reference: {path, sha256}, raw_json: str}``; its exact UTF-8
bytes are checked here.  Actual source files and model dependencies must still
be authenticated by the caller before invoking its explicit loader/factory.
The old unapproved profile supplies model/IMU dependencies only. Its ports,
capture, power epoch, posture bounds, timing result and permissions are not used.

The actual topology capture and its sealed events.jsonl supply a fresh identity,
run_mode, current, voltage and three positions per ID. Values are decoded from
the original wires. ReadOnlyCAN records no write-finish clock; none is invented
here. No branch is changed after planning and no physical clearance/STOP is
inferred.
"""
import copy
import hashlib
import json
import math
from pathlib import PurePosixPath
import statistics
import struct

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.angle_calibration_audit import resolve_unique_numeric_branch
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire

from .topology import PORTS, validate_topology

SOURCE_SCHEMA = 'singularitydog.four-bus-model-source-binding.v1'
PLAN_SCHEMA = 'singularitydog.four-bus-model-input-plan.v1'
IDS = tuple(range(1, 13))
ID_KEYS = {str(i) for i in IDS}
MARGIN_RAD = 2 * 25.14 / 65535
LOCAL_RADIUS_RAD = math.radians(3)
PERIOD_NS = 20_000_000
OBSERVER_LIMIT_NS = 100_000_000  # Existing diagnostic observer; final20ms gate is separate.
MODEL_SOURCES = ('singularitydog_hw/policy_checked_dispatch.py',
    'singularitydog_hw/policy_output_model.py', 'singularitydog_hw/policy_observer.py',
    'singularitydog_hw/policy_observer_replay.py',
    'experiments/private_checked_policy_dispatch/adapter.py',
    'experiments/private_checked_policy_dispatch/cpu_kernel.py',
    'experiments/private_checked_policy_dispatch/checked_dispatch.cpp')
NO_GRANTS = {'output_allowed': False, 'approved_for_runtime': False,
    'active_controller_qualification': False, 'whole_loop_timing_qualified': False,
    'physical_branch_or_motion_proven': False, 'physical_clearance_verified': False}


def _need(value, message):
    if not value:
        raise ValueError(message)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _sha(value):
    _need(type(value) is str and len(value) == 64 and
          all(c in '0123456789abcdef' for c in value), 'Exact lowercase SHA256 required')
    return value


def _reference(value):
    _need(type(value) is dict and set(value) == {'path', 'sha256'}, 'Exact pinned reference required')
    path = value['path']
    _need(type(path) is str and PurePosixPath(path).is_absolute() and
          '..' not in PurePosixPath(path).parts, 'Absolute pinned locator required')
    _sha(value['sha256'])
    return dict(value)


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        _need(key not in result, 'Duplicate JSON key')
        result[key] = value
    return result


def _bad_constant(value):
    raise ValueError('Nonfinite JSON constant: '+value)


def _envelope(value):
    _need(type(value) is dict and set(value) == {'reference', 'raw_json'},
          'Pinned raw-JSON envelope required')
    reference = _reference(value['reference'])
    raw = value['raw_json']
    _need(type(raw) is str and len(raw.encode('utf-8')) <= 8 * 1024 * 1024,
          'Bounded UTF-8 JSON document required')
    _need(hashlib.sha256(raw.encode('utf-8')).hexdigest() == reference['sha256'],
          'Pinned document bytes differ')
    document = json.loads(raw, object_pairs_hook=_pairs, parse_constant=_bad_constant)
    _need(type(document) is dict, 'JSON object required')
    _canonical(document)  # Reject finite-looking overflow floats as well as NaN.
    return document, reference


def _finite(value, name):
    _need(type(value) in (int, float) and math.isfinite(value), 'Finite '+name+' required')
    return value


def _stamp(value, name):
    _need(type(value) is int and value > 0, 'Positive original '+name+' required')
    return value


def _frame(hex_wire):
    _need(type(hex_wire) is str and len(hex_wire) == 34 and
          all(c in '0123456789abcdef' for c in hex_wire), 'One original17-byte frame required')
    wire = bytes.fromhex(hex_wire)
    _need(wire[:2] == b'AT' and wire[6] == 8 and wire[-2:] == b'\r\n',
          'Canonical DLC8 AT frame required')
    encoded = int.from_bytes(wire[2:6], 'big')
    _need(encoded & 7 == 4, 'Extended frame required')
    return codec.Frame(encoded >> 3, encoded & 7, wire[7:15], wire)


def _frame_wire(wire):
    """Same canonical-frame checks as _frame for an owned 17-byte image."""
    _need(type(wire) is bytes and len(wire) == 17, 'One original17-byte frame required')
    _need(wire[:2] == b'AT' and wire[6] == 8 and wire[-2:] == b'\r\n',
          'Canonical DLC8 AT frame required')
    encoded = int.from_bytes(wire[2:6], 'big')
    _need(encoded & 7 == 4, 'Extended frame required')
    return codec.Frame(encoded >> 3, encoded & 7, wire[7:15], wire)


def _events(envelope, topology):
    _need(type(envelope) is dict and set(envelope) == {'reference', 'raw_jsonl'},
          'Pinned original JSONL event envelope required')
    reference = _reference(envelope['reference'])
    raw = envelope['raw_jsonl']
    _need(type(raw) is str and raw.endswith('\n') and len(raw.encode('utf-8')) <= 8 * 1024 * 1024,
          'Bounded complete original JSONL trace required')
    encoded = raw.encode('utf-8')
    _need(hashlib.sha256(encoded).hexdigest() == reference['sha256'], 'Pinned event bytes differ')
    trace = topology.get('trace', {})
    _need(trace.get('path') == reference['path'] and trace.get('sha256') == reference['sha256']
          and trace.get('complete') is True and trace.get('errors') == []
          and trace.get('bytes') == len(encoded) and trace.get('events') == raw.count('\n'),
          'Actual topology trace receipt differs')
    events = [json.loads(line, object_pairs_hook=_pairs, parse_constant=_bad_constant)
              for line in raw.splitlines()]
    _need(len(events) <= 20_000 and all(type(e) is dict for e in events), 'Bounded original events required')
    _canonical(events)
    selected = [e for e in events if e.get('phase') == 'telemetry']
    tx = [e for e in selected if e.get('kind') == 'can_tx']
    results = [e for e in selected if e.get('kind') == 'motor_parameter']
    _need(len(tx) == len(results) == 84, 'Exactly84 original telemetry requests/results required')
    discovery = [e for e in events if e.get('phase') == 'discovery']
    _need(sum(e.get('kind') == 'can_tx' for e in discovery) == 48,
          'All48 original one-shot discovery requests required')
    return events, reference


def _query(query, port, mid, parameter, capture_begin, capture_end, events, used, *, phase='telemetry'):
    _need(type(query) is dict, 'Original query result required')
    name = parameter or 'identity'
    start = _stamp(query.get('request_monotonic_ns'), 'query request')
    end = _stamp(query.get('monotonic_ns'), 'query return')
    sequence = query.get('sequence')
    _need(type(sequence) is int and sequence > 0 and capture_begin <= start <= end <= capture_end,
          'Read-only query causality/capture interval differs')
    _need(end - start <= (30_000_000 if parameter == 'position' else 250_000_000),
          'Read-only query exceeded original capture bound')
    scope = lambda e: e.get('phase') == phase and e.get('port') == port and e.get('requested_id') == mid and e.get('requested_parameter') == name
    txs = [(i, e) for i, e in enumerate(events) if scope(e) and e.get('kind') == 'can_tx'
           and e.get('monotonic_ns') == start and e.get('sequence') == sequence]
    _need(len(txs) == 1 and bytes.fromhex(txs[0][1].get('hex', '')) == codec.read_request(mid, parameter)
          and txs[0][1].get('motor_id') == mid and txs[0][1].get('parameter') == name,
          'One canonical original query write required')
    outcomes = [(i, e) for i, e in enumerate(events) if scope(e) and e.get('kind') == 'motor_parameter'
                and e.get('request_monotonic_ns') == start and e.get('monotonic_ns') == end]
    _need(len(outcomes) == 1 and all(outcomes[0][1].get(k) == v for k, v in query.items()),
          'Original query return event differs')
    frames = []
    for i, event in enumerate(events):
        if not (scope(event) and event.get('kind') == 'can_rx_frame' and
                type(event.get('monotonic_ns')) is int and start <= event['monotonic_ns'] <= end):
            continue
        frame = _frame(event.get('wire_hex'))
        _need(all(event.get(k) == v for k, v in frame.record().items()), 'Raw frame metadata differs')
        if codec.matches(frame, mid, parameter):
            frames.append((i, frame))
    _need(len(frames) == 1, 'One matching fresh original reply required')
    result = codec.decode_reply(frames[0][1], mid, parameter)
    _need(result.get('ok') is True and all(query.get(k) == v for k, v in result.items()),
          'Raw decoded reply differs from original query result')
    _need(query.get('kind') == 'motor_parameter' and query.get('round_trip_ms') == (end - start) / 1e6,
          'Original query metadata/timing differs')
    indices = {txs[0][0], outcomes[0][0], frames[0][0]}
    _need(not used.intersection(indices), 'Original query evidence reused')
    used.update(indices)
    return result, start, end


def _discovery_queries(topology, events, begin, end):
    used = set()
    for port in PORTS:
        rows = topology['discovery'][port]
        _need(type(rows) is list and len(rows) == 12 and
              [row.get('motor_id') for row in rows] == list(IDS), 'Original48 discovery ordering required')
        for row in rows:
            mid = row['motor_id']
            first, last = row.get('started_ns'), row.get('finished_ns')
            _need(type(first) is int and type(last) is int and begin <= first <= last <= end,
                  'Original discovery interval differs')
            if row.get('status') == 'FRESH_IDENTITY':
                reply, _, _ = _query(row.get('raw_result'), port, mid, None, first, last,
                                     events, used, phase='discovery')
                _need(reply['mcu_uid_hex'] == row.get('mcu_uid_hex'), 'Discovery UID/raw reply differs')
                continue
            _need(row.get('status') == 'NO_FRESH_RESPONSE', 'Failed/unfinished discovery cannot qualify')
            txs = [(i, e) for i, e in enumerate(events) if e.get('kind') == 'can_tx'
                   and e.get('phase') == 'discovery' and e.get('port') == port and
                   e.get('requested_id') == mid and first <= e.get('monotonic_ns', -1) <= last]
            timeouts = [(i, e) for i, e in enumerate(events) if e.get('kind') == 'can_timeout'
                        and e.get('phase') == 'discovery' and e.get('port') == port and
                        e.get('requested_id') == mid and first <= e.get('monotonic_ns', -1) <= last]
            _need(len(txs) == len(timeouts) == 1 and
                  txs[0][1].get('hex') == codec.read_request(mid).hex() and
                  txs[0][1]['monotonic_ns'] <= timeouts[0][1]['monotonic_ns'],
                  'Original no-fresh-response transaction evidence missing')
            _need(not used.intersection((txs[0][0], timeouts[0][0])), 'Discovery evidence reused')
            used.update((txs[0][0], timeouts[0][0]))
    _need(len(used) == 108, 'All48 original discovery boundaries must be retained')


def _source_binding(source, profile, profile_ref):
    _need(source.get('schema') == SOURCE_SCHEMA and
          source.get('original_model_profile') == profile_ref and
          source.get('old_transport_qualification_reused') is False and
          source.get('old_pose_approval_reused') is False and
          all(source.get(k) is v for k, v in NO_GRANTS.items()),
          'Separate model-only source binding required; no inherited grants')
    frozen = _reference(source.get('frozen_model_source_manifest'))
    current = _reference(source.get('four_bus_source_manifest'))
    _need(frozen != current and frozen['sha256'] != current['sha256'],
          'New four-bus source proof must be distinct from frozen model source')
    pins = source.get('frozen_model_source_sha256')
    _need(type(pins) is dict and set(pins) == set(MODEL_SOURCES),
          'All seven selected/warmed model source pins required')
    cadence = profile.get('cadence_source_sha256')
    _need(type(cadence) is dict and all(_sha(pins[p]) == cadence.get(p) for p in MODEL_SOURCES),
          'Original profile/model source pins differ')
    return {'frozen_model_source_manifest': frozen, 'four_bus_source_manifest': current,
            'frozen_model_source_sha256': dict(pins),
            'actual_source_file_bytes_verified_here': False,
            'caller_must_verify_source_and_model_files_before_and_after': True}


def prepare_model_plan(*, topology, events, calibration, model_profile, source_binding,
                       mount, bias, accel_hypothesis=None):
    """Default PLAN: derive an unapproved current4bus input map from raw bytes.

    No factory or loader is called. Model-profile input is unchanged and usable
    only with the caller's ordinary frozen diagnostic_load/checked-load proof.
    Its historical transport/angles/boot/power are intentionally never copied.
    """
    topo, topo_ref = _envelope(topology)
    topo = validate_topology(topo)
    event_rows, events_ref = _events(events, topo)
    nominal, nominal_ref = _envelope(calibration)
    profile, profile_ref = _envelope(model_profile)
    source, source_ref = _envelope(source_binding)
    mount_doc, mount_ref = _envelope(mount)
    bias_doc, bias_ref = _envelope(bias)
    _need(profile.get('approved_for_supported_policy_output') is False and
          profile.get('native_target_fk_cache') is True and
          profile.get('native_checked_policy_dispatch') is True,
          'Original unapproved selected FK/checked model profile required')
    binding = _source_binding(source, profile, profile_ref)
    artifacts = profile.get('artifacts', {})
    for name, ref in (('calibration', nominal_ref), ('mount', mount_ref), ('bias', bias_ref)):
        _need(artifacts.get(name) == ref, 'Original '+name+' artifact pin differs')
    shadow.validate_imu_mount_candidate(mount_doc)
    from singularitydog_hw.policy_observer import _bias
    _bias(bias_doc)  # Original unverified A-fit validation; dict path opens nothing.
    accel_selected = profile.get('accel_input_hypothesis', False)
    _need(type(accel_selected) is bool and accel_selected == (accel_hypothesis is not None),
          'Original explicit acceleration hypothesis selection must be preserved')
    accel_ref = None
    if accel_selected:
        accel_doc, accel_ref = _envelope(accel_hypothesis)
        _need(artifacts.get('accel_input_hypothesis') == accel_ref and
              accel_doc.get('grants_motor_output') is False and
              accel_doc.get('formal_calibration_approved') is False,
              'Original unapproved acceleration hypothesis pin required')
    h = _finite(profile.get('h_hypothesis'), 'original h hypothesis')
    command = profile.get('command')
    _need(h == 0 and type(command) is list and len(command) == 3 and
          all(type(x) in (int, float) and math.isfinite(x) and x == 0 for x in command),
          'Original h0 and zero command retained for STOP-only observation')
    _need(topo.get('angle_wrap_applied') is False and topo.get('stop_state') == 'NOT_PROVEN_BY_READONLY',
          'Original four-bus raw capture has no wrapping or STOP proof')
    boot, epoch = topo.get('boot_before'), topo.get('motor_power_epoch')
    _need(type(epoch) is str and epoch.strip() and epoch not in
          ('UNKNOWN', 'NOT_INFERRED_FROM_JETSON_BOOT') and epoch == topo['motor_power_epoch'],
          'Explicit current topology/capture power label required; no continuity inferred')
    begin = _stamp(topo.get('started_monotonic_ns'), 'capture start')
    end = _stamp(topo.get('finished_monotonic_ns'), 'capture finish')
    _need(begin < end, 'Capture interval required')
    _discovery_queries(topo, event_rows, begin, end)
    rows = topo.get('motor_rows')
    _need(type(rows) is dict and set(rows) == ID_KEYS, 'All twelve actual capture rows required')
    _need(nominal.get('approved_for_runtime') is False, 'Unapproved nominal calibration required')
    nominal_rows = shadow.validate_calibration(nominal)
    _need(nominal['identities'] == topo['expected_uids'], 'Current UID/nominal identities differ')
    derived = copy.deepcopy(nominal)
    derived_rows = shadow.validate_calibration(derived)
    axes, raw_by_id, q_by_id, turns, final_ns = {}, {}, {}, {}, {}
    profile_axes = profile.get('axes')
    _need(type(profile_axes) is dict and set(profile_axes) == ID_KEYS, 'Original twelve model axes required')
    used = set()
    for mid in IDS:
        key, row, nominal_axis = str(mid), rows[str(mid)], nominal_rows[mid]
        port = row.get('port')
        _need(port in PORTS and mid in topo['ids_by_port'][port], 'Capture physical ID topology differs')
        reads = row['reads']
        uid, uid_start, uid_end = _query(reads.get('identity'), port, mid, None, begin, end, event_rows, used)
        _need(uid['mcu_uid_hex'] == topo['expected_uids'][key], 'Fresh capture UID differs')
        old_axis = profile_axes[key]
        sign, offset = nominal_axis['sign_candidate'], nominal_axis['offset_candidate_rad']
        _need(old_axis.get('uid') == uid['mcu_uid_hex'] and old_axis.get('sign') == sign and
              old_axis.get('offset_rad') == offset, 'Original model UID/sign/nominal offset differs')
        for parameter in ('run_mode', 'current', 'voltage'):
            result, start, finish = _query(reads.get(parameter), port, mid, parameter, begin, end, event_rows, used)
            _need(uid_end <= start, 'Current parameter predates fresh UID')
            value = result['value']
            _need((35. <= value <= 42.) if parameter == 'voltage' else value == 0,
                  'Current mode/current/strict35..42V rejected: ID'+key)
        samples = reads.get('position')
        _need(type(samples) is list and len(samples) == 3, 'Exactly three current position samples required')
        values, previous = [], uid_end
        for sample in samples:
            result, start, finish = _query(sample, port, mid, 'position', begin, end, event_rows, used)
            _need(previous <= start, 'Position samples reordered or reused')
            values.append(result['value']); previous = finish
        raw = statistics.median(values)
        _need(math.degrees(max(values) - min(values)) <= .1, 'Current position span exceeds0.1degree')
        index = shadow.CAN_ORDER.index(mid)
        lower, upper = shadow.LOWER[index], shadow.UPPER[index]
        branch = resolve_unique_numeric_branch(raw, sign=sign, offset_rad=offset,
            lower_rad=lower, upper_rad=upper, uncertainty_rad=MARGIN_RAD)
        turn = branch['turns']
        q = sign * (raw - turn * 2 * math.pi) + offset
        fixed = offset - sign * turn * 2 * math.pi
        local = [max(lower, q - LOCAL_RADIUS_RAD), min(upper, q + LOCAL_RADIUS_RAD)]
        derived_rows[mid]['offset_candidate_rad'] = fixed
        axes[key] = {'uid': uid['mcu_uid_hex'], 'physical_port': port, 'sign': sign,
            'nominal_offset_rad': offset, 'fixed_offset_rad': fixed,
            'reference_turns': turn, 'global_bounds_rad': [lower, upper],
            'local_bounds_rad': local, 'capture_last_position_reply_ns': previous,
            'numerical_margin_rad': MARGIN_RAD}
        raw_by_id[key] = raw; q_by_id[key] = q; turns[key] = turn; final_ns[key] = previous
    _need(len(used) == 84 * 3, 'All original telemetry evidence must be bound once')
    provenance = {'schema': 'singularitydog.four-bus-local-numeric-branch.v1',
        'topology_capture': topo_ref, 'raw_events': events_ref, 'nominal_calibration': nominal_ref,
        'original_model_profile_for_dependencies_only': profile_ref,
        'source_binding': source_ref, **binding, 'boot_id': boot, 'motor_power_epoch': epoch,
        'power_cycle_count_or_continuity_verified': False, 'uids_by_id': dict(topo['expected_uids']),
        'raw_rad_by_id': raw_by_id, 'model_rad_by_id': q_by_id, 'reference_turns_by_id': turns,
        'capture_last_position_reply_ns_by_id': final_ns,
        'formula': 'q_model = sign * raw + (nominal_offset - sign * turns * 2*pi)',
        'raw_angles_modified': False, 'original_nominal_artifact_modified': False,
        'old_profile_posture_or_transport_permission_used': False,
        'read_only_write_finish_clock_recorded': False,
        'absolute_calibration_error_rad': None, **NO_GRANTS}
    # Keep the original object pinned, including historical metadata. The new
    # numerical copy has its own explicit current provenance and no permission.
    historical_fields = ('source_current_boot_id', 'source_current_motor_power_epoch_label',
        'source_capture_sha256', 'source_raw_rad_by_id', 'model_rad_at_source_capture_by_id',
        'current_operator_power_statement', 'diagnostic_branch_derivation', 'diagnostic_local_reference_branch')
    derived['four_bus_original_nominal_metadata'] = {
        key: copy.deepcopy(derived.pop(key)) for key in historical_fields if key in derived}
    derived['four_bus_numeric_branch'] = copy.deepcopy(provenance)
    derived.update(approved_for_runtime=False, output_allowed=False, motor_output_available=False)
    age = _finite(profile.get('max_sample_age_ms'), 'original sample age')
    _need(0 < age <= 20, 'Original sample age may not exceed20ms')
    measured_axes = {}
    offsets_by_id = {}
    preserved_limits = ('max_measured_torque_nm', 'max_measured_velocity_rad_s',
                        'max_temperature_c', 'max_displacement_from_start_rad')
    for mid in IDS:
        key, current = str(mid), axes[str(mid)]
        original = profile_axes[key]
        for name in preserved_limits:
            _need(_finite(original.get(name), 'original '+name) > 0,
                  'Original positive measured limit required')
        measured_axes[key] = {name: original[name] for name in preserved_limits}
        measured_axes[key].update(uid=current['uid'], sign=current['sign'],
            lower_rad=current['global_bounds_rad'][0], upper_rad=current['global_bounds_rad'][1],
            physical_lower_rad=current['local_bounds_rad'][0], physical_upper_rad=current['local_bounds_rad'][1])
        offsets_by_id[key] = current['fixed_offset_rad']
    measured_profile = {'schema': 'singularitydog.four-bus-measured-input-limits.v1',
        'max_sample_age_ms': age, 'axes': measured_axes,
        'boot_id': boot, 'motor_power_epoch': epoch,
        'original_model_profile_for_numerical_limits_only': profile_ref,
        'old_initial_posture_or_transport_approval_reused': False, **NO_GRANTS}
    plan = {'schema': PLAN_SCHEMA, 'status': 'PURE_PLAN_NO_MODEL_OR_DEVICE_OPENED',
        'topology_by_port': copy.deepcopy(topo['ids_by_port']), 'axes': axes,
        'calibration': derived, 'provenance': provenance, 'model_profile': copy.deepcopy(profile),
        'model_profile_reference': profile_ref, 'source_binding': binding,
        'measured_input_profile': measured_profile, 'runtime_offsets_by_id': offsets_by_id,
        'observer_kwargs': {'imu_mount_candidate': copy.deepcopy(mount_doc),
            'gyro_bias_candidate': copy.deepcopy(bias_doc), 'h_hypothesis': h,
            'command': [0., 0., 0.], 'max_age_ns': OBSERVER_LIMIT_NS,
            'max_spread_ns': OBSERVER_LIMIT_NS, 'profile_consume': True,
            'measured_diagnostic_ticks': True, 'reuse_input_buffers': True,
            'apply_reviewed_accel_calibration': False, 'accel_input_hypothesis': accel_ref},
        'required_selected_method_warmup_before_and_after_placement': 10,
        'required_reset_after_warmup_before_tick_zero': True,
        'required_final_input_to_all_write_and_reply_deadline_ns': PERIOD_NS,
        'model_only_loader': 'policy_active_fk.diagnostic_load; checked.load(active=False)',
        'model_cadence_or_dependency_files_verified_here': False, **NO_GRANTS}
    plan['plan_canonical_sha256'] = hashlib.sha256(_canonical(plan).encode()).hexdigest()
    return plan


def _plan_axes(plan):
    _need(type(plan) is dict and plan.get('schema') == PLAN_SCHEMA and
          all(plan.get(k) is v for k, v in NO_GRANTS.items()), 'Pure unapproved model plan required')
    payload = {key: value for key, value in plan.items() if key != 'plan_canonical_sha256'}
    _need(hashlib.sha256(_canonical(payload).encode()).hexdigest() == plan.get('plan_canonical_sha256'),
          'Prepared model plan was mutated')
    mapping, axes = plan.get('topology_by_port'), plan.get('axes')
    _need(type(mapping) is dict and set(mapping) == set(PORTS) and
          all(type(mapping[p]) is list and len(mapping[p]) == 3 and
              all(type(mid) is int for mid in mapping[p]) for p in PORTS) and
          sorted(mid for p in PORTS for mid in mapping[p]) == list(IDS) and
          type(axes) is dict and set(axes) == ID_KEYS, 'Exact physical4/full12 plan required')
    result = {}
    for mid in IDS:
        row = axes[str(mid)]
        port = row.get('physical_port')
        _need(port in PORTS and mid in mapping[port], 'Plan physical topology differs')
        sign = row.get('sign')
        _need(type(sign) is int and sign in (-1, 1), 'Fixed branch sign required')
        offset = _finite(row.get('fixed_offset_rad'), 'fixed branch offset')
        lo, hi = row.get('local_bounds_rad', (None, None))
        index = shadow.CAN_ORDER.index(mid)
        _need(shadow.LOWER[index] <= _finite(lo, 'local lower') <
              _finite(hi, 'local upper') <= shadow.UPPER[index], 'Unchanged model bounds required')
        final_ns = _stamp(row.get('capture_last_position_reply_ns'), 'last capture reply')
        result[mid] = (port, sign, offset, lo, hi, final_ns)
    return result


def _check_positions(axes, snapshot):
    _need(type(snapshot) is dict and snapshot.get('output_allowed') is False,
          'No-output snapshot required')
    positions = {}
    for row in snapshot.get('motors', []):
        if row.get('parameter') != 'position':
            continue
        mid = row.get('motor_id')
        _need(type(mid) is int and mid in axes and mid not in positions, 'Unique full12 current position required')
        _need(_stamp(row.get('request_ns'), 'position request') > axes[mid][5],
              'Runtime position must follow its actual current capture')
        raw = _finite(row.get('value'), 'current raw position')
        _, sign, offset, lo, hi, _ = axes[mid]
        q = sign * raw + offset
        _need(math.isfinite(q) and lo <= q <= hi, 'Outside immutable local branch/range: ID'+str(mid))
        positions[mid] = raw
    _need(set(positions) == set(IDS), 'Missing full12 current positions; no branch reselection')


def snapshot_from_four_records(plan, records_by_port, sample, tick_ns):
    """Parse complete physical3+3+3+3 original feedback; never synthesize Stats.

    The caller retains each physical Batch/raw journal, checks Stats and owns
    original Future readiness, voltage/full16 input-record finalgate
    and actual20ms deadline. This function projects exactly24 scalar values for
    the ordinary observer and does not confer transport/freshness qualification.
    """
    return _snapshot(_plan_axes(plan), records_by_port, sample, tick_ns)


def _snapshot(axes, records_by_port, sample, tick_ns, *, raw_frames=False):
    _need(type(records_by_port) is dict and set(records_by_port) == set(PORTS),
          'All four original feedback sets required')
    tick_ns = _stamp(tick_ns, 'observer tick')
    motors = []
    oldest = latest = earliest_receive = None
    for port in PORTS:
        records = records_by_port[port]
        _need(type(records) is Record * 3, 'Original owned Record*3 required per physical port')
        seen = set()
        for record in records:
            _need(record.written == record.received == 17 and
                  0 < record.start_ns <= record.finish_ns <= record.read_start_ns <=
                  record.received_ns < record.deadline_ns and record.received_ns <= tick_ns,
                  'Incomplete/noncausal original feedback record')
            if raw_frames:
                tx, rx = _frame_wire(bytes(record.tx)), _frame_wire(bytes(record.rx))
            else:
                tx, rx = _frame(bytes(record.tx).hex()), _frame(bytes(record.rx).hex())
            mid = tx.destination
            _need(mid in axes and axes[mid][0] == port and mid not in seen,
                  'Duplicate/missing/cross-physical-port feedback')
            _need(tx.wire == stop_wire(mid) and rx.kind == 2 and rx.source == mid and
                  rx.destination == codec.HOST_ID and
                  (rx.can_id >> 22) & 3 == 0 and (rx.can_id >> 16) & 63 == 0 and
                  rx.can_id == (2 << 24) | (mid << 8) | codec.HOST_ID and
                  rx.data[:3] != b'\x00\xc4\x56', 'Healthy original mode0 STOP feedback required')
            seen.add(mid)
            p, v, _, _ = struct.unpack('>4H', rx.data)
            # Preserve original parser's operation order/quantization math.
            for parameter, value, unit in (('position', p * (2. * 12.57) / 65535. - 12.57, 'rad'),
                    ('velocity', v * 100. / 65535. - 50., 'rad_s')):
                motors.append({'motor_id': mid, 'parameter': parameter, 'value': value,
                    'unit': unit, 'request_ns': record.start_ns,
                    'received_ns': record.received_ns, 'age_upper_bound_ns': tick_ns - record.start_ns})
            oldest = record.start_ns if oldest is None else min(oldest, record.start_ns)
            latest = record.received_ns if latest is None else max(latest, record.received_ns)
            earliest_receive = record.received_ns if earliest_receive is None else min(earliest_receive, record.received_ns)
    _need(type(sample) is dict, 'Original IMU sample required')
    a = _stamp(sample.get('read_started_monotonic_ns'), 'IMU read start')
    b = _stamp(sample.get('read_finished_monotonic_ns'), 'IMU read finish')
    _need(a <= b <= tick_ns, 'Noncausal original IMU interval')
    vectors = {}
    for name in ('accel_m_s2', 'gyro_rad_s'):
        vector = sample.get(name)
        _need(type(vector) is list and len(vector) == 3, 'Three original IMU components required')
        vectors[name] = [_finite(value, 'IMU component') for value in vector]
    oldest, latest, earliest_receive = min(oldest, a), max(latest, b), min(earliest_receive, b)
    _need(tick_ns - oldest <= PERIOD_NS and latest - oldest <= PERIOD_NS,
          'Original four-bus input age/spread exceeds20ms')
    snapshot = {'status': 'DIAGNOSTIC_READY', 'output_allowed': False, 'blocked_reasons': [],
        'tick_ns': tick_ns, 'max_age_ns': OBSERVER_LIMIT_NS, 'max_spread_ns': OBSERVER_LIMIT_NS,
        'motors': motors, 'imu': {'frame': 'raw_sensor', **vectors,
            'read_started_ns': a, 'read_finished_ns': b, 'age_upper_bound_ns': tick_ns - a},
        'oldest_observation_age_ns': tick_ns - oldest, 'acquisition_spread_ns': latest - oldest,
        'receive_spread_ns': latest - earliest_receive, 'voltage_by_bus': {},
        'source_flags': {'native_diagnostic_transport': True, 'sensor_type2_candidate': True,
            'v3_voltage_cadence_proxy': False, 'v3_voltage_overlap_pending_at_inference': True,
            'velocity_scale_verified': False, 'sensor_internal_sample_time_verified': False,
            'stop_feedback_state_changing': True, 'fresh_identity_match_verified': True,
            'four_physical_buses_projected_to_original_full12_model': True,
            'physical_buses': list(PORTS), 'physical_record_count_by_port': {p: 3 for p in PORTS},
            'synthetic_six_axis_stats_created': False, 'full16_input_final_gate_verified_here': False,
            'approved_for_runtime': False, 'output_allowed': False}}
    _check_positions(axes, snapshot)
    return snapshot


def batch_snapshot_builder(plan, *, decode_once=False):
    """Setup-only immutable plan copy; accept exact current Batchs at takeout.

    Explicit decode_once keeps the same frame/value checks and math, parses the
    owned byte images directly and compares them instead of re-decoding rows.
    """
    _need(type(decode_once) is bool, 'Explicit decode-once boolean required')
    owned = copy.deepcopy(plan)
    axes = _plan_axes(owned)  # Static plan hashing remains outside measured ticks.
    def build(feedback_by_port, sample, tick_ns):
        from .transport_adapter import Batch
        _need(type(feedback_by_port) is dict and set(feedback_by_port) == set(PORTS),
              'Four original physical Batchs required')
        records = {}
        for port in PORTS:
            batch = feedback_by_port[port]
            _need(type(batch) is Batch and batch.group.port == port and batch.label == 'feedback'
                  and list(batch.group.ids) == owned['topology_by_port'][port],
                  'Genuine same-physical-group feedback Batch required')
            if decode_once:
                batch.verify_images()  # Frames below are parsed from these bytes.
            else:
                batch.verify()
            records[port] = batch.records
        return _snapshot(axes, records, sample, tick_ns, raw_frames=decode_once)
    return build


def _combined_feedback_view(batch, port, ids, tick_ns):
    """A direct first-three view of one owned, healthy mixed Record*4 batch.

    No new Stats or record image is produced. The caller retains the complete
    acquisition and its original Future; this view cannot establish owner
    readiness, cancellation, the all12 voltage cache or final20ms admission.
    """
    import ctypes as C
    from .transport_adapter import Batch
    from singularitydog_hw.native_active_transport import Stats
    _need(type(batch) is Batch and batch.group.port == port and batch.group.ids == ids
          and batch.label == 'acquisition_combined4',
          'Explicit same-physical-group combined4 Batch required')
    records, stats = batch.records, batch.stats
    _need(type(records) is Record * 4 and records._b_base_ is None and
          records._b_needsfree_ == 1 and type(stats) is Stats and stats._b_base_ is None
          and stats._b_needsfree_ == 1,
          'Original owned Record*4 and original Stats required')
    rows = batch.verify()
    _need(type(tick_ns) is int and type(batch.completed_ns) is int and
          0 < stats.begin_ns <= stats.end_ns <=
          batch.completed_ns <= tick_ns and stats.writes == 4 and stats.bytes == 4 * 17
          and stats.rejected_size == stats.rejected_total == 0,
          'Complete original mixed4 Stats and current owner clocks required')
    for i, record in enumerate(records):
        _need(record.written == record.received == 17 and
              0 < stats.begin_ns <= record.start_ns <= record.finish_ns <=
              record.read_start_ns <= record.received_ns < record.deadline_ns and
              record.received_ns <= stats.end_ns,
              'All four mixed acquisition records must be complete and causal')
        if i < 3:
            _need(bytes(record.tx) == stop_wire(ids[i]),
                  'Original ordered three STOP feedback slots required')
    voltage_record = records[3]
    tx, rx = _frame(bytes(voltage_record.tx).hex()), _frame(bytes(voltage_record.rx).hex())
    mid = tx.destination
    _need(mid in ids and bytes(voltage_record.tx) == codec.read_request(mid, 'voltage'),
          'Original fourth same-group voltage request required')
    value = codec.decode_reply(rx, mid, 'voltage')
    _need(value.get('ok') is True and 35. <= _finite(value.get('value'), 'combined voltage') <= 42.,
          'Healthy current combined voltage must remain within35..42V')
    _need(set(rows) == {(axis, 'feedback') for axis in ids} | {(mid, 'voltage')} and
          rows[mid, 'voltage'] == (value, voltage_record.start_ns, voltage_record.received_ns),
          'Exact three feedback plus one original voltage row required')
    # from_buffer aliases the original native array. It neither copies bytes
    # nor manufactures a three-record transaction or independently sliced Stats.
    view = (Record * 3).from_buffer(records)
    _need(C.addressof(view) == C.addressof(records) and view._b_needsfree_ == 0 and
          all(C.addressof(view[i]) == C.addressof(records[i]) for i in range(3)),
          'Feedback projection must directly reference original mixed4 slots')
    return view


def combined_batch_snapshot_builder(plan):
    """Explicit full4 acquisition projection; the old default builder is intact.

    Every physical batch's fourth voltage row is checked before the ordinary
    input projection. The caller must join its original executor Future before
    inference and retain all four raw records/Stats for the later full16 gate.
    No earlier two-phase schedule or transport qualification is transferred.
    """
    owned = copy.deepcopy(plan)
    axes = _plan_axes(owned)
    def build(acquisition_by_port, sample, tick_ns):
        _need(type(acquisition_by_port) is dict and set(acquisition_by_port) == set(PORTS),
              'All four complete physical combined acquisitions required')
        views = {port: _combined_feedback_view(acquisition_by_port[port], port,
                    tuple(owned['topology_by_port'][port]), tick_ns) for port in PORTS}
        snapshot = _snapshot(axes, views, sample, tick_ns)
        snapshot['source_flags'].update(
            v3_voltage_overlap_pending_at_inference=False,
            physical_record_count_by_port={port: 4 for port in PORTS},
            projected_feedback_record_count_by_port={port: 3 for port in PORTS},
            rotating_voltage_record_index_by_port={port: 3 for port in PORTS},
            original_combined_stats_retained=True,
            acquisition_schedule='ordered_three_STOP_plus_one_voltage_single_native_exchange',
            combined_acquisition_projection=True)
        return snapshot
    return build


class _GuardedObserver:
    """Freeze the numerical branch; ordinary observer retains all own checks."""
    def __init__(self, delegate, plan):
        self._delegate = delegate
        self._axes = _plan_axes(copy.deepcopy(plan))

    def __getattr__(self, name):
        return getattr(self._delegate, name)

    def consume(self, snapshot):
        _check_positions(self._axes, snapshot)
        return self._delegate.consume(snapshot)


def create_guarded_observer(plan, observer_factory, *, policy, max_ticks,
                            torch_module=None, checked_dispatch_wrapper=None):
    """Explicit runtime-only factory seam, no built-in model loading or grants.

    Caller must authenticate actual frozen source/model files, diagnostic_load
    FK once, checked.load(active=False), and warm the SELECTED wrapper10+10 with
    its ordinary reset before tick0. This seam cannot perform/attest those steps.
    """
    owned = copy.deepcopy(plan)
    _plan_axes(owned)
    _need(callable(observer_factory) and type(max_ticks) is int and max_ticks in (5, 501),
          'Explicit observer factory and finite5/501 diagnostic scope required')
    _need(checked_dispatch_wrapper is not None, 'Selected checked wrapper must remain explicit')
    kwargs = copy.deepcopy(owned['observer_kwargs'])
    kwargs.update(max_ticks=max_ticks, torch_module=torch_module,
                  checked_dispatch_wrapper=checked_dispatch_wrapper)
    delegate = observer_factory(policy, copy.deepcopy(owned['calibration']), **kwargs)
    _need(callable(getattr(delegate, 'consume', None)), 'Ordinary observer consume required')
    return _GuardedObserver(delegate, owned)
