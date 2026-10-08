"""File/synthetic tests only: no real serial port, native library or model."""
from contextlib import contextmanager, redirect_stdout
import copy
import io
import json
from pathlib import Path
import stat
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from experiments.four_bus_diagnostic import topology as t


EXPECTED = {str(i): i.to_bytes(8, 'little').hex() for i in t.IDS}
PATHS = {p: '/dev/serial/by-path/test-'+str(i) for i, p in enumerate(t.PORTS)}
BINDINGS = {p: {'path': PATHS[p], 'resolved': '/dev/ttyUSB'+str(i), 'st_rdev': i+1}
            for i, p in enumerate(t.PORTS)}
GROUPS = {p: tuple(range(i*3+1, i*3+4)) for i, p in enumerate(t.PORTS)}


class Fixture:
    def __init__(self, groups=None):
        self.groups = copy.deepcopy(groups if groups is not None else GROUPS)
        self.calls, self.opens, self.closes, self.events = [], [], [], []
        self.ticks = 1000
        self.uid_changes = {}
        self.failures = {}
        self.values = {}

    def clock(self):
        self.ticks += 1000
        return self.ticks

    @contextmanager
    def session(self, port, ids, phase):
        token = (port, ids, phase, len(self.opens))
        self.opens.append(token)
        fixture = self
        class Probe:
            poisoned = False
            def query(self, mid, parameter=None):
                if self.poisoned:
                    raise AssertionError('Poisoned session reused')
                fixture.calls.append((port, mid, parameter, phase))
                fixture.events.append({'kind': 'can_tx', 'hex': t.canonical_request(mid, parameter).hex()})
                if (port, mid, parameter, phase) in fixture.failures:
                    self.poisoned = True
                    raise fixture.failures[port, mid, parameter, phase]
                if mid not in fixture.groups[port]:
                    self.poisoned = True
                    fixture.events.append({'kind': 'can_timeout', 'motor_id': mid})
                    raise TimeoutError('No fresh identity')
                if parameter is None:
                    return {'motor_id': mid, 'parameter': 'identity', 'ok': True,
                        'mcu_uid_hex': fixture.uid_changes.get((port, mid, phase), EXPECTED[str(mid)]),
                        'request_monotonic_ns': fixture.clock(), 'monotonic_ns': fixture.clock()}
                value = fixture.values.get((mid, parameter), 0 if parameter == 'run_mode'
                    else 0.0 if parameter == 'current' else 37.5 if parameter == 'voltage' else mid*.1)
                raw = bytes([value])+bytes(3) if parameter == 'run_mode' else struct.pack('<f', value)
                value = struct.unpack_from('<B' if parameter == 'run_mode' else '<f', raw)[0]
                return {'motor_id': mid, 'parameter': parameter, 'ok': True,
                    'index': t.INDEX[parameter], 'unit': t.UNITS[parameter], 'status': 0,
                    'value': value, 'raw_value_hex': raw.hex(),
                    'request_monotonic_ns': fixture.clock(), 'monotonic_ns': fixture.clock()}
        try:
            yield Probe()
        finally:
            self.closes.append(token)

    def capture(self, **kwargs):
        doc = t.collect_topology(EXPECTED, BINDINGS, self.session, self.events.append,
            boot_before='actual-test-boot', motor_power_epoch='explicit-test-epoch',
            clock=self.clock, **kwargs)
        doc['boot_after'] = 'actual-test-boot'
        return doc


class TopologyTests(unittest.TestCase):
    def test_all48_discovery_ids_use_fresh_sessions_then_four_fresh_telemetry_sessions(self):
        fixture = Fixture(); doc = fixture.capture()
        self.assertEqual(doc['status'], 'COMPLETE_READONLY_TOPOLOGY_CANDIDATE')
        self.assertEqual(len(fixture.opens), 52)
        self.assertEqual(fixture.opens, fixture.closes)
        discovery = [x for x in fixture.opens if x[2] == 'discovery']
        self.assertEqual(len(discovery), 48)
        self.assertTrue(all(len(x[1]) == 1 for x in discovery))
        self.assertEqual(sum(r['status'] == 'NO_FRESH_RESPONSE' for rows in doc['discovery'].values() for r in rows), 36)
        self.assertEqual(doc['ids_by_port'], {p: list(ids) for p, ids in GROUPS.items()})
        self.assertEqual(len(fixture.calls), 132)
        self.assertEqual(t.validate_topology(doc, expected_boot='actual-test-boot',
            expected_power_epoch='explicit-test-epoch'), doc)

    def test_three_uids_are_rechecked_before_any_parameter_on_each_port(self):
        fixture = Fixture(); fixture.capture()
        for port in t.PORTS:
            calls = [c for c in fixture.calls if c[0] == port and c[3] == 'telemetry']
            self.assertEqual([c[2] for c in calls[:3]], [None]*3)
            self.assertEqual([c[1] for c in calls[:3]], list(GROUPS[port]))

    def test_only_canonical_type0_and_type17_allowlisted_parameter_wires(self):
        fixture = Fixture(); fixture.capture()
        wires = [bytes.fromhex(e['hex']) for e in fixture.events if e.get('kind') == 'can_tx']
        self.assertEqual({(int.from_bytes(w[2:6], 'big') >> 27) & 31 for w in wires}, {0, 17})
        self.assertTrue(all(w[:2] == b'AT' and w[-2:] == b'\r\n' for w in wires))
        indexes = {int.from_bytes(w[7:9], 'little') for w in wires
                   if ((int.from_bytes(w[2:6], 'big') >> 27) & 31) == 17}
        self.assertEqual(indexes, set(t.INDEX.values()))

    def test_missing_motor_has_full_discovery_raw_and_no_telemetry(self):
        groups = copy.deepcopy(GROUPS); groups['port3'] = (10, 11)
        fixture = Fixture(groups); doc = fixture.capture()
        self.assertEqual(doc['status'], 'REJECTED_TOPOLOGY')
        self.assertEqual(len(fixture.calls), 48)
        self.assertEqual(sum(e['kind'] == 'can_tx' for e in fixture.events), 48)
        self.assertEqual(sum(e['kind'] == 'can_timeout' for e in fixture.events), 37)
        self.assertEqual(sum(e['kind'] == 'topology_discovery_outcome' for e in fixture.events), 48)
        self.assertTrue(any('ID12' in e for e in doc['errors']))
        self.assertEqual(fixture.opens, fixture.closes)

    def test_duplicate_id_on_two_buses_is_rejected_without_parameter_reads(self):
        groups = copy.deepcopy(GROUPS); groups['port1'] = (*groups['port1'], 1)
        fixture = Fixture(groups); doc = fixture.capture()
        self.assertEqual(doc['status'], 'REJECTED_TOPOLOGY')
        self.assertTrue(any('ID1' in e for e in doc['errors']))
        self.assertTrue(all(c[2] is None for c in fixture.calls))

    def test_swapped_uid_is_rejected_and_original_mismatch_retained(self):
        fixture = Fixture(); fixture.uid_changes['port0', 1, 'discovery'] = EXPECTED['2']
        doc = fixture.capture()
        self.assertEqual(doc['status'], 'REJECTED_TOPOLOGY')
        self.assertEqual(doc['discovery']['port0'][0]['raw_result']['mcu_uid_hex'], EXPECTED['2'])
        self.assertEqual(len(fixture.calls), 48)

    def test_late_unmatched_identity_can_reject_but_cannot_replace_fresh_discovery(self):
        fixture = Fixture()
        late = [{'port': 'port1', 'motor_id': 1, 'mcu_uid_hex': EXPECTED['1'],
                 'requested_id': 4, 'monotonic_ns': 10000, 'wire_hex': '00'}]
        doc = fixture.capture(raw_identities=late)
        self.assertEqual(doc['status'], 'REJECTED_TOPOLOGY')
        self.assertEqual(doc['raw_identity_observations'], late)
        self.assertEqual(len(fixture.calls), 48)

    def test_non_timeout_communication_fault_aborts_and_closes_without_retry(self):
        fixture = Fixture(); fixture.failures['port0', 2, None, 'discovery'] = OSError('partial write')
        doc = fixture.capture()
        self.assertEqual(doc['status'], 'FAILED_READONLY_CAPTURE')
        self.assertEqual(len(fixture.calls), 2)
        self.assertEqual(fixture.opens, fixture.closes)
        self.assertTrue(any('partial write' in e for e in doc['errors']))

    def test_telemetry_timeout_is_failure_not_normal_absence(self):
        fixture = Fixture(); fixture.failures['port0', 1, 'voltage', 'telemetry'] = TimeoutError('voltage late')
        doc = fixture.capture()
        self.assertEqual(doc['status'], 'FAILED_READONLY_CAPTURE')
        self.assertIn('current', doc['motor_rows']['1']['reads'])
        self.assertEqual(fixture.opens, fixture.closes)
        self.assertTrue(any('voltage late' in e for e in doc['errors']))

    def test_rechecked_uid_change_stops_before_that_ports_parameter_reads(self):
        fixture = Fixture(); fixture.uid_changes['port0', 2, 'telemetry'] = EXPECTED['3']
        doc = fixture.capture()
        self.assertEqual(doc['status'], 'FAILED_READONLY_CAPTURE')
        self.assertTrue(all(c[2] is None for c in fixture.calls))

    def test_position_triplet_raw_clocks_median_and_span_are_preserved_without_wrapping(self):
        doc = Fixture().capture()
        for row in doc['motor_rows'].values():
            self.assertEqual(len(row['position_samples']), 3)
            self.assertEqual(len(row['reads']['position']), 3)
            self.assertEqual(row['median_position_rad'], row['position_rad'])
            self.assertEqual(row['position_span_deg'], 0.0)
            self.assertTrue(all(s['request_monotonic_ns'] < s['reply_monotonic_ns'] for s in row['position_samples']))
        changed = copy.deepcopy(doc); changed['motor_rows']['1']['position_samples'][0]['rad'] += .1
        with self.assertRaises(ValueError):
            t.validate_topology(changed)
        self.assertFalse(doc['angle_wrap_applied'])
        self.assertEqual(doc['stop_state'], 'NOT_PROVEN_BY_READONLY')

    def test_source_candidate_cannot_grant_output_or_current_boot_epoch(self):
        doc = Fixture().capture()
        for key in ('output_allowed', 'approved_for_runtime', 'physical_clearance_verified'):
            changed = copy.deepcopy(doc); changed[key] = True
            with self.assertRaises(ValueError):
                t.validate_topology(changed)
        with self.assertRaises(ValueError):
            t.validate_topology(doc, expected_boot='other')
        with self.assertRaises(ValueError):
            t.validate_topology(doc, expected_power_epoch='other')

    def test_aliases_and_non_character_devices_are_rejected_before_open(self):
        with (patch.object(Path, 'resolve', autospec=True, return_value=Path('/dev/one')),
              patch.object(Path, 'stat', autospec=True, return_value=types.SimpleNamespace(st_mode=stat.S_IFCHR, st_rdev=1))):
            with self.assertRaisesRegex(ValueError, 'aliases'):
                t.bind_ports(PATHS)
        with (patch.object(Path, 'resolve', autospec=True, return_value=Path('/dev/one')),
              patch.object(Path, 'stat', autospec=True, return_value=types.SimpleNamespace(st_mode=stat.S_IFREG, st_rdev=1))):
            with self.assertRaisesRegex(ValueError, 'character'):
                t.bind_ports(PATHS)

    def test_serial_write_guard_rejects_all_noncanonical_or_out_of_transaction_bytes(self):
        serial = types.SimpleNamespace(write=lambda wire: len(wire))
        guard = t._SerialGuard(serial)
        with self.assertRaises(ValueError):
            guard.write(t.canonical_request(1))
        guard.allowed = t.canonical_request(1, 'current')
        self.assertEqual(guard.write(guard.allowed), 17)
        with self.assertRaises(ValueError):
            guard.write(t.canonical_request(1, 'position'))
        for name in ('can_timeout', 'zero_state', 'write', 'enable', 'stop'):
            with self.assertRaises(ValueError):
                t.canonical_request(1, name)

    def test_default_plan_pins_wrapper_inputs_without_loading_readonly_or_opening_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            uids = root/'uids.json'; uids.write_text(json.dumps({'schema': 'PRIVATE.four-bus-expected-identities.v1', 'identities': EXPECTED}))
            locators = root/'ports.json'; locators.write_text(json.dumps({'ports': [{'label': p, 'path': PATHS[p]} for p in t.PORTS]}))
            source = Path(__file__).resolve().parents[2]/'singularitydog_hw'/'can_readonly.py'
            argv = ['--port-locators', str(locators), '--port-locators-sha256', t.digest(locators.read_bytes()),
                '--expected-uids', str(uids), '--expected-uids-sha256', t.digest(uids.read_bytes()),
                '--readonly-source', str(source), '--readonly-source-sha256', t.digest(source.read_bytes()),
                '--output', str(root/'new'), '--timeout-ms', '500']
            stream = io.StringIO()
            with (patch.object(t, '_load_readonly', side_effect=AssertionError('loaded')),
                  patch.object(t, 'bind_ports', side_effect=AssertionError('device binding inspected')),
                  redirect_stdout(stream)):
                self.assertEqual(t.main(argv), 0)
            result = json.loads(stream.getvalue())
            self.assertFalse(result['device_opened'])
            self.assertEqual(result['maximum_total_requests'], 132)
            self.assertFalse((root/'new').exists())
            for bound in ('9', '501'):
                with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
                    t.main(argv[:-1]+[bound])

    def test_existing_readonly_timeout_really_poisons_session_and_closes(self):
        source = Path(__file__).resolve().parents[2]/'singularitydog_hw'/'can_readonly.py'
        module = t._load_readonly(source, t.digest(source.read_bytes()))
        class Serial:
            in_waiting = 0
            closed = False
            writes = []
            def read(self, count):
                return b''
            def write(self, wire):
                self.writes.append(wire); return len(wire)
            def close(self):
                self.closed = True
        serial = Serial(); ticks = [0]
        def clock():
            ticks[0] += 1_000_000; return ticks[0]
        events = []
        try:
            with module.ReadOnlyCAN(timeout_s=.01, serial_port=serial, clock=clock, event_sink=events.append) as session:
                with self.assertRaises(TimeoutError):
                    session.query(1)
                self.assertTrue(session.poisoned)
                with self.assertRaises(RuntimeError):
                    session.query(2)
            self.assertTrue(serial.closed)
            self.assertEqual(serial.writes, [t.canonical_request(1)])
            self.assertTrue(any(e['kind'] == 'can_timeout' for e in events))
        finally:
            sys.modules.pop(module.__name__, None)


if __name__ == '__main__':
    unittest.main()
