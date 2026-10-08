"""Default-PLAN four-port topology discovery using only Type0/Type17 reads.

Pass four explicit /dev/serial/by-path locators and pinned expected UID and
can_readonly.py files. --execute-readonly is separate. Discovery uses a fresh
session for every port/ID; a timeout is a no-fresh-response observation, never
permission to reuse a poisoned session or accept a late reply. After exact
12-UID/3-per-port discovery, fresh sessions recheck UID and read run_mode,
position and voltage. No enable, Type1, parameter write, STOP or zero request
exists here. Mapping/telemetry is a private unapproved candidate, not a physical
clearance, STOP, power-cycle or performance proof. No native library is used.
"""
from contextlib import ExitStack, contextmanager
import argparse
import copy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import statistics
import struct
import sys
import time
import types


SCHEMA = 'singularitydog.four-bus-readonly-topology.v1'
PORTS = ('port0', 'port1', 'port2', 'port3')
IDS = tuple(range(1, 13))
READS = ('run_mode', 'current', 'voltage', 'position')
TELEMETRY_READS = ('run_mode', 'current', 'voltage', 'position', 'position', 'position')
INDEX = {'run_mode': 0x7005, 'current': 0x701A, 'position': 0x7019, 'voltage': 0x701C}
UNITS = {'run_mode': 'enum', 'current': 'A', 'position': 'rad_output_shaft', 'voltage': 'V'}
MAX_EVENTS = 20000
MAX_TRACE_BYTES = 8 * 1024 * 1024


def need(value, message):
    if not value:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def strict_pairs(pairs):
    value = {}
    for key, item in pairs:
        need(key not in value, 'Duplicate JSON key: '+key)
        value[key] = item
    return value


def bad_constant(value):
    raise ValueError('Nonfinite JSON constant: '+value)


def read_pinned(path, expected_sha256):
    path = Path(path)
    need(path.is_absolute() and not any(p.is_symlink() for p in (path, *path.parents)),
         'Pinned input must be an absolute nonsymlink path: '+str(path))
    need(path.is_file(), 'Regular pinned input required: '+str(path))
    need(type(expected_sha256) is str and len(expected_sha256) == 64 and
         all(c in '0123456789abcdef' for c in expected_sha256), 'Invalid source SHA256')
    raw = path.read_bytes()
    need(digest(raw) == expected_sha256, 'Pinned input SHA256 mismatch: '+str(path))
    return raw


def validate_uids(value):
    need(type(value) is dict and set(value) == {str(i) for i in IDS},
         'Exactly expected UID keys1..12 required')
    need(all(type(uid) is str and len(uid) == 16 and
             all(c in '0123456789abcdef' for c in uid) for uid in value.values()),
         'Expected UID must be eight bytes of lowercase hexadecimal')
    need(len(set(value.values())) == 12, 'Expected UID identities must be unique')
    return dict(value)


def expected_document(value):
    if type(value) is dict and value.get('schema') == 'PRIVATE.four-bus-expected-identities.v1':
        return validate_uids(value['identities'])
    return validate_uids(value)


def port_paths(values):
    need(len(values) == 4, 'Exactly four explicit by-path ports required')
    paths = [Path(value) for value in values]
    need(all(p.is_absolute() and p.parent == Path('/dev/serial/by-path') for p in paths),
         'Use explicit absolute /dev/serial/by-path locators')
    need(len(set(paths)) == 4, 'Four distinct by-path locators required')
    return {name: str(path) for name, path in zip(PORTS, paths)}


def bind_ports(paths):
    bindings = {}
    for name in PORTS:
        path = Path(paths[name])
        resolved = path.resolve(strict=True)
        info = resolved.stat()
        need(stat.S_ISCHR(info.st_mode), 'Port must resolve to a character device: '+str(path))
        bindings[name] = {'path': str(path), 'resolved': str(resolved), 'st_rdev': info.st_rdev}
    need(len({v['resolved'] for v in bindings.values()}) == 4 and
         len({v['st_rdev'] for v in bindings.values()}) == 4,
         'Port aliases do not describe four independent character devices')
    return bindings


def check_bindings(bindings):
    current = bind_ports({name: value['path'] for name, value in bindings.items()})
    need(current == bindings, 'Physical by-path device binding changed')


def canonical_request(motor_id, parameter=None):
    need(type(motor_id) is int and motor_id in IDS, 'Read-only ID1..12 required')
    need(parameter is None or parameter in READS, 'Only identity/mode/position/voltage reads allowed')
    kind = 0 if parameter is None else 17
    data = bytes(8) if parameter is None else struct.pack('<H', INDEX[parameter])+bytes(6)
    can_id = (kind << 24) | (0xFD << 8) | motor_id
    return b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+bytes([8])+data+b'\r\n'


class _SerialGuard:
    def __init__(self, serial):
        self._serial = serial
        self.allowed = None

    def write(self, wire):
        need(self.allowed is not None and type(wire) is bytes and wire == self.allowed,
             'Noncanonical or out-of-transaction CAN write rejected')
        return self._serial.write(wire)

    def __getattr__(self, name):
        return getattr(self._serial, name)


def _uid(result):
    need(type(result) is dict and result.get('ok') is True and
         result.get('parameter') == 'identity', 'Malformed identity read result')
    value = result.get('mcu_uid_hex')
    need(type(value) is str and len(value) == 16 and all(c in '0123456789abcdef' for c in value),
         'Malformed identity UID')
    return value


def _parameter(result, mid, name):
    need(type(result) is dict and result.get('ok') is True and
         result.get('motor_id') == mid and result.get('parameter') == name and
         result.get('index') == INDEX[name] and result.get('unit') == UNITS[name] and
         result.get('status') == 0,
         'Invalid read-only parameter result: ID'+str(mid)+' '+name)
    value = result.get('value')
    need(type(value) in (int, float) and math.isfinite(value), 'Nonfinite read-only parameter')
    if name == 'run_mode':
        need(type(value) is int, 'run_mode is an integer enum, not a STOP/disabled proof')
    raw = result.get('raw_value_hex')
    need(type(raw) is str and len(raw) == 8 and all(c in '0123456789abcdef' for c in raw),
         'Original four-byte parameter value required')
    decoded = struct.unpack_from('<B' if name == 'run_mode' else '<f', bytes.fromhex(raw))[0]
    need(type(decoded) is type(value) and
         (decoded == value if name == 'run_mode' else struct.pack('<d', decoded) == struct.pack('<d', value)),
         'Parameter value differs from original raw bytes')
    request, reply = result.get('request_monotonic_ns'), result.get('monotonic_ns')
    need(type(request) is int and type(reply) is int and 0 < request <= reply,
         'Original causal request/reply nanoseconds required')
    return value


def _mapping(discovery, expected):
    ids_by_port = {name: [] for name in PORTS}
    errors = []
    for mid in IDS:
        seen = [(name, row) for name in PORTS for row in discovery[name]
                if row['motor_id'] == mid and row['status'] == 'FRESH_IDENTITY']
        if len(seen) != 1:
            errors.append('ID'+str(mid)+': expected exactly one fresh bus, observed '+str(len(seen)))
        for name, row in seen:
            if row['mcu_uid_hex'] != expected[str(mid)]:
                errors.append('ID'+str(mid)+': expected UID mismatch on '+name)
        if len(seen) == 1 and seen[0][1]['mcu_uid_hex'] == expected[str(mid)]:
            ids_by_port[seen[0][0]].append(mid)
    if any(len(ids) != 3 for ids in ids_by_port.values()):
        errors.append('Expected exactly three identified motors per port')
    return ids_by_port, errors


def _raw_identity_conflicts(observations, mapping, expected):
    errors = []
    for row in observations:
        mid = str(row['motor_id'])
        if (row['mcu_uid_hex'] != expected[mid] or
                int(mid) not in mapping[row['port']]):
            errors.append('Uncorrelated/contradictory raw identity: ID'+mid+' '+row['port'])
    return errors


def collect_topology(expected, ports, session_factory, emit, *, boot_before,
                     motor_power_epoch=None, clock=time.monotonic_ns, raw_identities=None):
    """Injectable discovery; the real factory owns port/source/write boundaries.

    Timeout is normal only during one-shot identity discovery. Any other fault
    aborts; no poisoned session is continued and no telemetry failure is hidden.
    Caller-provided epoch is recorded verbatim, never detected or inferred.
    """
    expected = validate_uids(expected)
    raw_identities = raw_identities if raw_identities is not None else []
    need(set(ports) == set(PORTS), 'Exactly four named ports required')
    report = {'schema': SCHEMA, 'status': 'INCOMPLETE', 'errors': [],
        'started_monotonic_ns': clock(), 'boot_before': boot_before, 'boot_after': None,
        'motor_power_epoch': motor_power_epoch, 'ports': copy.deepcopy(ports),
        'expected_uids': expected, 'discovery': {p: [] for p in PORTS},
        'ids_by_port': None, 'motor_rows': {}, 'device_closed_after_every_discovery': True,
        'automatic_retry': False, 'allowed_can_types': [0, 17],
        'angle_wrap_applied': False, 'leg_names': None,
        'stop_state': 'NOT_PROVEN_BY_READONLY', 'physical_clearance_verified': False,
        'run_mode_semantics': 'Type17 integer enum observation; not Type2 mode or disabled/STOP proof',
        'motor_power_epoch_inferred': False, 'output_allowed': False,
        'approved_for_runtime': False, 'live_activation': False,
        'whole_loop_timing_qualified': False}
    try:
        for port in PORTS:
            for mid in IDS:
                row = {'motor_id': mid, 'status': 'PENDING', 'started_ns': clock()}
                report['discovery'][port].append(row)
                try:
                    # Fresh object and serial open for EACH ID, including after a timeout.
                    with session_factory(port, (mid,), 'discovery') as session:
                        result = session.query(mid, None)
                        need(result.get('motor_id') == mid, 'Identity source ID mismatch')
                        row.update(status='FRESH_IDENTITY', mcu_uid_hex=_uid(result), raw_result=result)
                except TimeoutError as error:
                    row.update(status='NO_FRESH_RESPONSE', error=repr(error))
                finally:
                    row['finished_ns'] = clock()
                emit({'kind': 'topology_discovery_outcome', 'port': port, **row})
        report['ids_by_port'], errors = _mapping(report['discovery'], expected)
        errors.extend(_raw_identity_conflicts(raw_identities, report['ids_by_port'], expected))
        report['errors'].extend(errors)
        if errors:
            report['status'] = 'REJECTED_TOPOLOGY'
            return report
        telemetry_begin = clock()
        for port in PORTS:
            ids = tuple(report['ids_by_port'][port])
            with session_factory(port, ids, 'telemetry') as session:
                for mid in ids:
                    row = {'port': port, 'motor_id': mid, 'reads': {}}
                    report['motor_rows'][str(mid)] = row
                    identity = session.query(mid, None)
                    row['reads']['identity'] = identity
                    row['mcu_uid_hex'] = _uid(identity)
                    need(identity.get('motor_id') == mid and row['mcu_uid_hex'] == expected[str(mid)],
                         'Fresh telemetry UID changed: ID'+str(mid))
                for mid in ids:
                    row = report['motor_rows'][str(mid)]
                    for name in TELEMETRY_READS:
                        result = session.query(mid, name)
                        if name == 'position':
                            row['reads'].setdefault('position', []).append(result)
                        else:
                            row['reads'][name] = result
                        value = _parameter(result, mid, name)
                        if name == 'position':
                            row.setdefault('position_samples', []).append({'rad': value,
                                'request_monotonic_ns': result['request_monotonic_ns'],
                                'reply_monotonic_ns': result['monotonic_ns']})
                        else:
                            row[{'run_mode': 'run_mode', 'current': 'current',
                                 'voltage': 'voltage_v'}[name]] = value
                    positions = [sample['rad'] for sample in row['position_samples']]
                    row['median_position_rad'] = row['position_rad'] = statistics.median(positions)
                    row['position_span_deg'] = math.degrees(max(positions)-min(positions))
                    need(row['run_mode'] == 0, 'Observed nonzero run_mode: ID'+str(mid))
                    emit({'kind': 'topology_telemetry', **row})
        report['telemetry'] = {'rows': report['motor_rows'], 'started_monotonic_ns': telemetry_begin,
            'ended_monotonic_ns': clock(), 'position_samples_per_id': 3,
            'capture_span_ms': (clock()-telemetry_begin)/1e6,
            'span_limit_deg_for_later_numerical_review': 0.1,
            'branch_or_physical_clearance_inferred': False}
        report['errors'].extend(_raw_identity_conflicts(raw_identities, report['ids_by_port'], expected))
        report['status'] = ('REJECTED_TOPOLOGY' if report['errors'] else
                            'COMPLETE_READONLY_TOPOLOGY_CANDIDATE')
    except BaseException as error:
        report['status'] = 'FAILED_READONLY_CAPTURE'
        report['errors'].append(type(error).__name__+': '+str(error))
        report['interrupted'] = isinstance(error, (KeyboardInterrupt, SystemExit))
    finally:
        report['finished_monotonic_ns'] = clock()
        report['raw_identity_observations'] = copy.deepcopy(raw_identities)
    return report


def validate_topology(document, *, expected_boot=None, expected_power_epoch=None):
    need(type(document) is dict and document.get('schema') == SCHEMA and
         document.get('status') == 'COMPLETE_READONLY_TOPOLOGY_CANDIDATE' and
         document.get('errors') == [], 'Complete read-only topology candidate required')
    need(all(document.get(k) is False for k in ('output_allowed', 'approved_for_runtime',
         'live_activation', 'whole_loop_timing_qualified', 'physical_clearance_verified',
         'motor_power_epoch_inferred')), 'Topology grants no output or physical qualification')
    boot = document.get('boot_before')
    need(type(boot) is str and boot and boot == document.get('boot_after'), 'Stable capture boot required')
    if expected_boot is not None:
        need(boot == expected_boot, 'Topology boot differs from current caller')
    if expected_power_epoch is not None:
        need(document.get('motor_power_epoch') == expected_power_epoch,
             'Topology epoch differs from explicit current caller')
    expected = validate_uids(document['expected_uids'])
    ports = document.get('ports')
    need(type(ports) is dict and set(ports) == set(PORTS), 'Four actual port bindings required')
    port_paths([ports[p]['path'] for p in PORTS])
    need(all(type(ports[p].get('resolved')) is str and Path(ports[p]['resolved']).is_absolute() and
             type(ports[p].get('st_rdev')) is int for p in PORTS) and
         len({ports[p]['resolved'] for p in PORTS}) == 4 and
         len({ports[p]['st_rdev'] for p in PORTS}) == 4, 'Four distinct actual device identities required')
    mapping = document.get('ids_by_port')
    need(type(mapping) is dict and set(mapping) == set(PORTS) and
         all(type(mapping[p]) is list and len(mapping[p]) == 3 and
             all(type(i) is int and i in IDS for i in mapping[p]) for p in PORTS) and
         sorted(i for p in PORTS for i in mapping[p]) == list(IDS), 'Exact3/3/3/3 ID topology required')
    discovered, errors = _mapping(document['discovery'], expected)
    need(not errors and discovered == mapping, 'Raw discovery differs from proposed topology')
    rows = document.get('motor_rows')
    need(type(rows) is dict and set(rows) == set(expected), 'All twelve current telemetry rows required')
    for mid, row in rows.items():
        need(row.get('port') in PORTS and int(mid) in mapping[row['port']] and
             row.get('mcu_uid_hex') == expected[mid] and row.get('run_mode') == 0,
             'Telemetry identity/mode differs: ID'+mid)
        for name in ('position_rad', 'voltage_v'):
            need(type(row.get(name)) in (int, float) and math.isfinite(row[name]),
                 'Finite current angle/voltage required: ID'+mid)
        need(_uid(row['reads']['identity']) == expected[mid], 'Raw telemetry identity differs')
        for name, target in (('run_mode', 'run_mode'), ('current', 'current'), ('voltage', 'voltage_v')):
            need(_parameter(row['reads'][name], int(mid), name) == row[target], 'Raw telemetry value differs')
        raw_positions = row['reads']['position']
        need(type(raw_positions) is list and len(raw_positions) == 3 and
             len(row['position_samples']) == 3, 'Three original position replies required')
        positions = []
        for result, sample in zip(raw_positions, row['position_samples']):
            value = _parameter(result, int(mid), 'position'); positions.append(value)
            need(sample == {'rad': value, 'request_monotonic_ns': result['request_monotonic_ns'],
                            'reply_monotonic_ns': result['monotonic_ns']}, 'Position raw clocks differ')
        need(row['position_rad'] == row['median_position_rad'] == statistics.median(positions) and
             row['position_span_deg'] == math.degrees(max(positions)-min(positions)),
             'Original median/span differs')
    need(not _raw_identity_conflicts(document.get('raw_identity_observations', []), mapping, expected),
         'Contradictory/unmatched raw identity cannot admit a topology')
    return copy.deepcopy(document)


def read_topology(path, sha256, **kwargs):
    return validate_topology(json.loads(read_pinned(path, sha256),
        object_pairs_hook=strict_pairs, parse_constant=bad_constant), **kwargs)


class _Trace:
    def __init__(self, path):
        self.path = path
        self.count = self.size = 0
        self.errors = []
        self.stream = open(path, 'xb', buffering=0)
        os.chmod(path, 0o600)

    def emit(self, event):
        raw = (json.dumps(event, sort_keys=True, allow_nan=False)+'\n').encode()
        need(self.count < MAX_EVENTS and self.size+len(raw) <= MAX_TRACE_BYTES,
             'Raw trace budget exhausted; no complete candidate can be published')
        need(self.stream.write(raw) == len(raw), 'Partial raw trace write')
        self.count += 1; self.size += len(raw)

    def close(self):
        try:
            self.stream.flush(); os.fsync(self.stream.fileno())
        except BaseException as error:
            self.errors.append(repr(error))
        finally:
            self.stream.close()
        raw = self.path.read_bytes()
        return {'path': str(self.path), 'sha256': digest(raw), 'bytes': len(raw),
                'events': raw.count(b'\n'), 'errors': self.errors,
                'complete': not self.errors and len(raw) == self.size and raw.count(b'\n') == self.count}


def _load_readonly(path, sha256):
    raw = read_pinned(path, sha256)
    name = '_four_bus_pinned_readonly_'+sha256
    need(name not in sys.modules, 'Fresh pinned readonly source namespace required')
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    try:
        # Compile authenticated source bytes, not an independently cached pyc.
        exec(compile(raw, str(path), 'exec'), module.__dict__)
        for mid in IDS:
            for parameter in (None, *READS):
                need(module.read_request(mid, parameter) == canonical_request(mid, parameter),
                     'Pinned readonly protocol differs from canonical Type0/Type17 reads')
        return module
    except BaseException:
        del sys.modules[name]
        raise


@contextmanager
def _ownership(bindings):
    import fcntl
    directory = Path.home()/'.cache'/'singularitydog'
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    with ExitStack() as stack:
        paths = [directory/'manual-calibration.lock', directory/'can-readonly.lock']
        paths += [Path('/tmp')/('singularitydog-can-port-'+digest(v['resolved'].encode())[:24]+'.lock')
                  for v in bindings.values()]
        for path in paths:
            handle = stack.enter_context(open(path, 'a+'))
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def _boot():
    value = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    need(bool(value), 'Current kernel boot ID required')
    return value


def _output_path(value, source):
    path = Path(value)
    need(path.is_absolute() and not path.exists() and
         not any(p.is_symlink() for p in (path, *path.parents)), 'Fresh absolute private output required')
    need(not any((p/'.git').exists() for p in path.parents) and
         Path(__file__).absolute().parents[2] not in path.parents and
         Path(source).parent.parent not in path.parents, 'Output must be outside Git/source/runtime')
    return path


def make_plan(args):
    need(type(args.timeout_ms) is int and 10 <= args.timeout_ms <= 500, 'timeout-ms must be10..500')
    if args.port_locators is not None:
        locators = json.loads(read_pinned(args.port_locators, args.port_locators_sha256),
            object_pairs_hook=strict_pairs, parse_constant=bad_constant)
        need(type(locators) is dict and type(locators.get('ports')) is list and len(locators['ports']) == 4,
             'Pinned port locator document must contain exactly four ports')
        paths = port_paths([row['path'] for row in locators['ports']])
    else:
        paths = port_paths(args.port)
    expected = expected_document(json.loads(read_pinned(args.expected_uids, args.expected_uids_sha256),
        object_pairs_hook=strict_pairs, parse_constant=bad_constant))
    read_pinned(args.readonly_source, args.readonly_source_sha256)
    output = _output_path(args.output, args.readonly_source)
    need(args.power_epoch is None or (type(args.power_epoch) is str and args.power_epoch.strip() == args.power_epoch
         and 0 < len(args.power_epoch) <= 1024), 'Explicit power epoch must be nonblank')
    return {'schema': SCHEMA, 'status': 'PLAN_ONLY', 'ports': paths,
        'expected_uids_sha256': args.expected_uids_sha256,
        'readonly_source': {'path': str(args.readonly_source), 'sha256': args.readonly_source_sha256},
        'topology_source_sha256': digest(Path(__file__).read_bytes()),
        'output': str(output), 'timeout_ms': args.timeout_ms, 'allowed_can_types': [0, 17],
        'discovery_requests': 48, 'maximum_total_requests': 132,
        'fresh_session_per_discovery_id': True, 'automatic_retry': False,
        'telemetry_after_exact_uid_topology': ['identity', *TELEMETRY_READS], 'motor_power_epoch': args.power_epoch,
        'port_locators_sha256': args.port_locators_sha256 if args.port_locators is not None else None,
        'device_opened': False, 'output_allowed': False, 'approved_for_runtime': False}, expected


def execute_readonly(args, plan, expected):
    output = Path(plan['output'])
    bindings = bind_ports(plan['ports'])
    source = _load_readonly(args.readonly_source, args.readonly_source_sha256)
    boot_before = _boot()
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    trace = _Trace(output/'events.jsonl')
    report = None
    raw_identities = []
    @contextmanager
    def factory(port, ids, phase):
        current = {'motor_id': None, 'parameter': None}
        def emit(event):
            trace.emit({'port': port, 'phase': phase, 'requested_id': current['motor_id'],
                        'requested_parameter': current['parameter'], **event})
            if (event.get('kind') == 'can_rx_frame' and event.get('type') == 0 and
                    event.get('flags') == 4 and event.get('destination_id') == 0xFE and
                    event.get('source_id') in IDS and type(event.get('data_hex')) is str and
                    len(event['data_hex']) == 16):
                raw_identities.append({'port': port, 'phase': phase, 'motor_id': event['source_id'],
                    'mcu_uid_hex': event['data_hex'], 'monotonic_ns': event['monotonic_ns'],
                    'requested_id': current['motor_id'], 'wire_hex': event['wire_hex']})
        check_bindings(bindings)
        with source.ReadOnlyCAN(bindings[port]['path'], timeout_s=args.timeout_ms/1000,
                                event_sink=emit) as session:
            info = os.fstat(session.serial.fileno())
            need(stat.S_ISCHR(info.st_mode) and info.st_rdev == bindings[port]['st_rdev'],
                 'Opened serial FD differs from actual by-path character device')
            guard = _SerialGuard(session.serial)
            session.serial = guard
            class Bound:
                def query(self, mid, parameter=None):
                    need(mid in ids and (phase != 'discovery' or parameter is None),
                         'Query outside explicit readonly discovery/telemetry scope')
                    current.update(motor_id=mid, parameter=parameter or 'identity')
                    check_bindings(bindings)
                    guard.allowed = canonical_request(mid, parameter)
                    try:
                        return session.query(mid, parameter)
                    finally:
                        guard.allowed = None
                        check_bindings(bindings)
            try:
                yield Bound()
            finally:
                emit({'kind': 'readonly_session_closing', 'poisoned': session.poisoned})
        check_bindings(bindings)
    try:
        with _ownership(bindings):
            report = collect_topology(expected, bindings, factory, trace.emit,
                boot_before=boot_before, motor_power_epoch=args.power_epoch, raw_identities=raw_identities)
    except BaseException as error:
        if report is None:
            report = {'schema': SCHEMA, 'status': 'FAILED_READONLY_CAPTURE', 'errors': [],
                'boot_before': boot_before, 'motor_power_epoch': args.power_epoch,
                'ports': bindings, 'output_allowed': False, 'approved_for_runtime': False}
        report['errors'].append(type(error).__name__+': '+str(error))
        report['status'] = 'FAILED_READONLY_CAPTURE'
    finally:
        if report is None:
            report = {'schema': SCHEMA, 'status': 'FAILED_READONLY_CAPTURE', 'errors': ['No report'],
                      'output_allowed': False, 'approved_for_runtime': False}
        for operation in (lambda: read_pinned(args.readonly_source, args.readonly_source_sha256),
                          lambda: read_pinned(args.expected_uids, args.expected_uids_sha256),
                          lambda: check_bindings(bindings)):
            try:
                operation()
            except BaseException as error:
                report['errors'].append(repr(error)); report['status'] = 'FAILED_READONLY_CAPTURE'
        if args.port_locators is not None:
            try:
                read_pinned(args.port_locators, args.port_locators_sha256)
            except BaseException as error:
                report['errors'].append(repr(error)); report['status'] = 'FAILED_READONLY_CAPTURE'
        try:
            report['boot_after'] = _boot()
            need(report['boot_after'] == boot_before, 'Kernel boot changed during capture')
        except BaseException as error:
            report['errors'].append(repr(error)); report['status'] = 'FAILED_READONLY_CAPTURE'
        report['trace'] = trace.close()
        if not report['trace']['complete']:
            report['errors'].append('Raw trace incomplete'); report['status'] = 'FAILED_READONLY_CAPTURE'
        report['source_sha256'] = {'topology': plan['topology_source_sha256'],
                                  'can_readonly': args.readonly_source_sha256}
        if digest(Path(__file__).read_bytes()) != plan['topology_source_sha256']:
            report['errors'].append('Topology source changed'); report['status'] = 'FAILED_READONLY_CAPTURE'
        report['expected_uids_sha256'] = args.expected_uids_sha256
        report['plan'] = plan
        report['finished_at'] = datetime.datetime.now().astimezone().isoformat()
        try:
            if report['status'] == 'COMPLETE_READONLY_TOPOLOGY_CANDIDATE':
                validate_topology(report)
        except BaseException as error:
            report['errors'].append(repr(error)); report['status'] = 'FAILED_READONLY_CAPTURE'
        with open(output/'capture.json', 'x') as handle:
            os.chmod(output/'capture.json', 0o600)
            json.dump(report, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write('\n'); handle.flush(); os.fsync(handle.fileno())
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--port', action='append', help='Repeat exactly four actual by-path locators')
    selection.add_argument('--port-locators', type=Path, help='Pinned JSON containing ports[].path for four devices')
    parser.add_argument('--port-locators-sha256')
    parser.add_argument('--expected-uids', type=Path, required=True)
    parser.add_argument('--expected-uids-sha256', required=True)
    parser.add_argument('--readonly-source', type=Path, required=True)
    parser.add_argument('--readonly-source-sha256', required=True)
    parser.add_argument('--timeout-ms', type=int, default=100)
    parser.add_argument('--power-epoch', help='Optional root-provided explicit epoch; never inferred')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute-readonly', action='store_true')
    args = parser.parse_args(argv)
    try:
        plan, expected = make_plan(args)
        report = execute_readonly(args, plan, expected) if args.execute_readonly else plan
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0 if report['status'] in ('PLAN_ONLY', 'COMPLETE_READONLY_TOPOLOGY_CANDIDATE') else 2


if __name__ == '__main__':
    raise SystemExit(main())
