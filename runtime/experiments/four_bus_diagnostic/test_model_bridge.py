"""Pure fixtures: no robot/device/model/Torch/native-library operation."""
from contextlib import contextmanager
import copy
import hashlib
import json
import math
import struct
import unittest
from unittest.mock import patch

from experiments.four_bus_diagnostic import model_bridge as m
from experiments.four_bus_diagnostic import topology as t
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire


def envelope(document, name):
    raw = json.dumps(document, indent=2, sort_keys=True, allow_nan=False)+'\n'
    return {'reference': {'path': '/fixture/'+name,
                         'sha256': hashlib.sha256(raw.encode()).hexdigest()}, 'raw_json': raw}


def wire(can_id, payload):
    return b'AT'+((can_id << 3) | 4).to_bytes(4, 'big')+b'\x08'+payload+b'\r\n'


class Capture:
    def __init__(self):
        self.clock_ns = 1_000_000
        self.events = []
        self.groups = {p: tuple(range(i*3+1, i*3+4)) for i, p in enumerate(t.PORTS)}
        self.uids = {str(i): i.to_bytes(8, 'little').hex() for i in m.IDS}
        self.raws = {i: (shadow.LOWER[shadow.CAN_ORDER.index(i)] +
                         shadow.UPPER[shadow.CAN_ORDER.index(i)])/2 for i in m.IDS}
        self.values = {}
        self.position_variation = {}

    def clock(self):
        self.clock_ns += 1000
        return self.clock_ns

    @contextmanager
    def session(self, port, ids, phase):
        source = self
        class Session:
            sequence = 0
            position_counts = {}
            def query(self, mid, parameter=None):
                self.sequence += 1
                start = source.clock()
                tag = {'port': port, 'phase': phase, 'requested_id': mid,
                       'requested_parameter': parameter or 'identity'}
                source.events.append({**tag, 'kind': 'can_tx', 'monotonic_ns': start,
                    'sequence': self.sequence, 'motor_id': mid, 'parameter': parameter or 'identity',
                    'hex': codec.read_request(mid, parameter).hex()})
                if mid not in source.groups[port]:
                    source.events.append({**tag, 'kind': 'can_timeout', 'monotonic_ns': source.clock(),
                                          'motor_id': mid, 'parameter': 'identity'})
                    raise TimeoutError('No fresh fixture response')
                if parameter is None:
                    reply = wire((mid << 8) | 0xFE, bytes.fromhex(source.uids[str(mid)]))
                else:
                    value = source.values.get((mid, parameter), source.raws[mid] if parameter == 'position'
                                              else 38. if parameter == 'voltage' else 0.)
                    if parameter == 'position' and mid in source.position_variation:
                        index = self.position_counts.get(mid, 0)
                        self.position_counts[mid] = index + 1
                        value += source.position_variation[mid][index]
                    index, fmt, _ = codec.PARAMETERS[parameter]
                    payload = struct.pack('<H', index)+bytes(2)+struct.pack('<'+fmt, int(value) if fmt == 'B' else value)
                    payload += bytes(8-len(payload))
                    reply = wire((17 << 24) | (mid << 8) | codec.HOST_ID, payload)
                rx_clock = source.clock()
                frame = m._frame(reply.hex())
                source.events.append({**tag, 'kind': 'can_rx_bytes', 'monotonic_ns': rx_clock, 'hex': reply.hex()})
                source.events.append({**tag, 'kind': 'can_rx_frame', 'monotonic_ns': rx_clock, **frame.record()})
                end = source.clock()
                result = codec.decode_reply(frame, mid, parameter)
                result.update(kind='motor_parameter', sequence=self.sequence, request_monotonic_ns=start,
                              monotonic_ns=end, round_trip_ms=(end-start)/1e6)
                source.events.append({**tag, **result})
                return result
        yield Session()

    def document(self):
        ports = {p: {'path': '/dev/serial/by-path/fixture-'+str(i),
                     'resolved': '/dev/ttyUSB'+str(i), 'st_rdev': i+1}
                 for i, p in enumerate(t.PORTS)}
        result = t.collect_topology(self.uids, ports, self.session, self.events.append,
                                   boot_before='current-boot', motor_power_epoch='explicit-current-label', clock=self.clock)
        result['boot_after'] = 'current-boot'
        return result


class Fixture:
    def __init__(self, capture=None):
        self.capture = capture or Capture()
        self.topology = self.capture.document()
        self.nominal = {'status': 'MANUAL_NOMINAL_CANDIDATES_ONLY',
            'formula': 'q_model = sign * raw + offset; rad; no wrapping',
            'model_can_order_candidate': list(shadow.CAN_ORDER),
            'identities': copy.deepcopy(self.capture.uids), 'approved_for_runtime': False,
            'source_current_boot_id': 'historical-boot',
            'source_capture_sha256': 'e'*64,
            'candidates': [{'motor_id': i, 'sign_candidate': 1, 'offset_candidate_rad': 0.} for i in m.IDS]}
        self.mount = {'schema_version': 1, 'status': 'IMU_MOUNT_CANDIDATE_ONLY',
            'input_frame': 'sensor', 'output_frame': 'body_x_forward_y_left_z_up',
            'R_body_from_sensor': [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
            'raw_driver_axes_verified': False, 'approved_for_runtime': False,
            'provenance': {'fixture': 'unverified'}}
        self.bias = {'schema_version': 1, 'kind': 'fixed_mount_baseline',
            'status': 'GYRO_BIAS_CANDIDATE', 'frame': 'sensor', 'axis_order': ['x','y','z'],
            'gyro_bias_candidate_eligible': True, 'operator_confirmed_stationary': True,
            'approved_for_runtime': False, 'automatically_applied': False, 'mount_rotation_applied': False,
            'gyro_bias_candidate_rad_s': [0.,0.,0.], 'captures': {'a': {'gyro_mean_rad_s': [0.,0.,0.]}},
            'provenance': {k: {'summary_sha256': c*64, 'events_sha256': d*64}
                           for k, c, d in (('a','a','b'),('b','c','d'))}}
        self.profile = {'approved_for_supported_policy_output': False,
            'native_target_fk_cache': True, 'native_checked_policy_dispatch': True,
            'accel_input_hypothesis': False, 'h_hypothesis': 0., 'command': [0.,0.,0.],
            'max_sample_age_ms': 20.,
            'boot_id': 'historical-profile-boot', 'motor_power_epoch': 'historical-profile-power',
            'axes': {str(i): {'uid': self.capture.uids[str(i)], 'sign': 1, 'offset_rad': 0.,
                             'physical_lower_rad': -999., 'physical_upper_rad': 999.,
                             'max_measured_torque_nm': 1., 'max_measured_velocity_rad_s': .35,
                             'max_temperature_c': 45., 'max_displacement_from_start_rad': math.radians(1)} for i in m.IDS},
            'artifacts': {}, 'cadence_source_sha256': {p: 'f'*64 for p in m.MODEL_SOURCES}}
        self.source = {'schema': m.SOURCE_SCHEMA,
            'old_transport_qualification_reused': False, 'old_pose_approval_reused': False,
            'frozen_model_source_manifest': {'path': '/fixture/frozen-manifest.json','sha256': 'a'*64},
            'four_bus_source_manifest': {'path': '/fixture/four-manifest.json','sha256': 'b'*64},
            'frozen_model_source_sha256': {p: 'f'*64 for p in m.MODEL_SOURCES}, **m.NO_GRANTS}
        self.accel = None

    def inputs(self):
        raw_events = ''.join(json.dumps(e, sort_keys=True, allow_nan=False)+'\n' for e in self.capture.events)
        event_ref = {'path': '/fixture/events.jsonl', 'sha256': hashlib.sha256(raw_events.encode()).hexdigest()}
        self.topology['trace'] = {**event_ref, 'bytes': len(raw_events.encode()), 'events': raw_events.count('\n'),
                                  'complete': True, 'errors': []}
        documents = {name: envelope(getattr(self, name), name+'.json')
                     for name in ('topology','mount','bias')}
        documents['calibration'] = envelope(self.nominal, 'calibration.json')
        self.profile['artifacts'].update({name: documents[name]['reference'] for name in ('calibration','mount','bias')})
        if self.accel is not None:
            documents['accel_hypothesis'] = envelope(self.accel, 'accel.json')
            self.profile['artifacts']['accel_input_hypothesis'] = documents['accel_hypothesis']['reference']
        documents['model_profile'] = envelope(self.profile, 'profile.json')
        self.source['original_model_profile'] = documents['model_profile']['reference']
        documents['source_binding'] = envelope(self.source, 'source-binding.json')
        documents['events'] = {'reference': event_ref, 'raw_jsonl': raw_events}
        return documents

    def plan(self):
        return m.prepare_model_plan(**self.inputs())


def records(plan):
    result = {}
    begin = max(a['capture_last_position_reply_ns'] for a in plan['axes'].values()) + 1_000_000
    for port in t.PORTS:
        group = plan['topology_by_port'][port]
        values = (Record*3)()
        for i, mid in enumerate(group):
            row = values[i]
            row.start_ns = begin + mid*10_000
            row.finish_ns = row.start_ns + 1000
            row.read_start_ns = row.finish_ns + 1000
            row.received_ns = row.read_start_ns + 1000
            row.deadline_ns = begin + 20_000_000
            row.written = row.received = 17
            raw = plan['provenance']['raw_rad_by_id'][str(mid)]
            p = round((raw + 12.57) * 65535. / (2. * 12.57))
            rx = wire((2 << 24) | (mid << 8) | codec.HOST_ID, struct.pack('>4H', p, 32768, 32768, 250))
            row.tx[:] = stop_wire(mid); row.rx[:] = rx
        result[port] = values
    tick = begin + 1_000_000
    imu = {'read_started_monotonic_ns': begin + 200_000, 'read_finished_monotonic_ns': begin + 300_000,
           'accel_m_s2': [0.,0.,9.80665], 'gyro_rad_s': [0.,0.,0.]}
    return result, imu, tick


class ModelPlanTests(unittest.TestCase):
    def test_plan_pure_no_open_or_loader_with_exact84_and48_raw_boundaries(self):
        fixture = Fixture(); inputs = fixture.inputs(); before = copy.deepcopy(inputs)
        with patch('builtins.open', side_effect=AssertionError('PLAN opened file')):
            plan = m.prepare_model_plan(**inputs)
        self.assertEqual(inputs, before)
        self.assertEqual(plan['status'], 'PURE_PLAN_NO_MODEL_OR_DEVICE_OPENED')
        self.assertTrue(all(plan[k] is False for k in m.NO_GRANTS))
        self.assertFalse(plan['model_cadence_or_dependency_files_verified_here'])
        self.assertFalse(plan['provenance']['read_only_write_finish_clock_recorded'])
        self.assertEqual(plan['provenance']['boot_id'], 'current-boot')
        self.assertEqual(plan['model_profile']['boot_id'], 'historical-profile-boot')
        self.assertNotIn('source_current_boot_id', plan['calibration'])
        self.assertEqual(plan['calibration']['four_bus_original_nominal_metadata']['source_current_boot_id'], 'historical-boot')
        self.assertEqual(m._plan_axes(plan).keys(), set(m.IDS))

    def test_local_bounds_derived_from_current_raw_and_unchanged_global_order(self):
        plan = Fixture().plan()
        for mid in m.IDS:
            index = shadow.CAN_ORDER.index(mid); axis = plan['axes'][str(mid)]
            q = plan['provenance']['model_rad_by_id'][str(mid)]
            self.assertEqual(axis['global_bounds_rad'], [shadow.LOWER[index], shadow.UPPER[index]])
            self.assertEqual(axis['local_bounds_rad'], [max(shadow.LOWER[index],q-m.LOCAL_RADIUS_RAD),
                                                         min(shadow.UPPER[index],q+m.LOCAL_RADIUS_RAD)])

    def test_measured_limits_and_offsets_preserved_without_old_posture_binding(self):
        fixture=Fixture();plan=fixture.plan();limits=plan['measured_input_profile']
        self.assertEqual(limits['schema'],'singularitydog.four-bus-measured-input-limits.v1')
        self.assertEqual(limits['max_sample_age_ms'],fixture.profile['max_sample_age_ms'])
        for mid in m.IDS:
            key=str(mid);a=limits['axes'][key];old=fixture.profile['axes'][key]
            for name in ('max_measured_torque_nm','max_measured_velocity_rad_s',
                         'max_temperature_c','max_displacement_from_start_rad'):
                self.assertEqual(a[name],old[name])
            self.assertEqual([a['lower_rad'],a['upper_rad']],plan['axes'][key]['global_bounds_rad'])
            self.assertEqual([a['physical_lower_rad'],a['physical_upper_rad']],plan['axes'][key]['local_bounds_rad'])
            self.assertEqual(plan['runtime_offsets_by_id'][key],plan['axes'][key]['fixed_offset_rad'])
        self.assertFalse(limits['old_initial_posture_or_transport_approval_reused'])

    def test_arbitrary_physical_port_permutation_preserves_model_can_order(self):
        capture=Capture();capture.groups=dict(zip(t.PORTS,((7,8,9),(10,11,12),(4,5,6),(1,2,3))))
        plan=Fixture(capture).plan()
        self.assertEqual(plan['topology_by_port'],{p:list(ids) for p,ids in capture.groups.items()})
        self.assertEqual(plan['calibration']['model_can_order_candidate'],shadow.CAN_ORDER)

    def test_unique_two_pi_branch_and_negative_sign_without_raw_mutation(self):
        capture = Capture(); capture.raws[6] = 2*math.pi + .1
        fixture = Fixture(capture); fixture.nominal['candidates'][5]['sign_candidate'] = -1
        fixture.profile['axes']['6']['sign'] = -1
        inputs = fixture.inputs(); plan = m.prepare_model_plan(**inputs)
        self.assertEqual(plan['axes']['6']['reference_turns'], 1)
        self.assertEqual(plan['axes']['6']['fixed_offset_rad'], 2*math.pi)
        self.assertEqual(inputs, fixture.inputs())
        self.assertFalse(plan['provenance']['physical_branch_or_motion_proven'])

    def test_out_of_global_range_and_uncertainty_at_limit_rejected(self):
        for raw in (.7, .5):
            with self.subTest(raw=raw):
                capture = Capture(); capture.raws[6] = raw
                with self.assertRaises(ValueError): Fixture(capture).plan()

    def test_nonzero_current_mode_and_voltage_outside35_42_rejected(self):
        for parameter, value in (('current', .01), ('run_mode',1),('voltage',34.99),('voltage',42.01)):
            with self.subTest(parameter=parameter, value=value):
                capture = Capture(); capture.values[1, parameter] = value
                with self.assertRaises(ValueError): Fixture(capture).plan()

    def test_positions_require_three_original_fresh_rows_and_span_limit(self):
        fixture = Fixture(); fixture.topology['motor_rows']['1']['reads']['position'].pop()
        with self.assertRaises(ValueError): fixture.plan()
        fixture = Fixture(); query = fixture.topology['motor_rows']['1']['reads']['position'][1]
        query['request_monotonic_ns'] = fixture.topology['motor_rows']['1']['reads']['position'][0]['request_monotonic_ns']
        with self.assertRaises(ValueError): fixture.plan()
        capture=Capture();capture.position_variation[1]=[0.,math.radians(.2),0.]
        with self.assertRaisesRegex(ValueError,'span'):Fixture(capture).plan()

    def test_local_interval_clips_to_global_without_clipping_current_value(self):
        capture=Capture();capture.raws[6]=.49
        plan=Fixture(capture).plan()
        self.assertEqual(plan['axes']['6']['local_bounds_rad'][1],.5)
        self.assertGreater(plan['provenance']['model_rad_by_id']['6'],.489)

    def test_current_epoch_or_boot_not_inferred_from_old_model_profile(self):
        fixture = Fixture(); fixture.topology['motor_power_epoch'] = 'NOT_INFERRED_FROM_JETSON_BOOT'
        with self.assertRaisesRegex(ValueError, 'Explicit current'): fixture.plan()
        fixture = Fixture(); fixture.topology['boot_after'] = 'other'
        with self.assertRaises(ValueError): fixture.plan()

    def test_missing_raw_tx_rx_or_parameter_return_is_not_fabricated(self):
        for kind in ('can_tx','can_rx_frame','motor_parameter'):
            with self.subTest(kind=kind):
                fixture = Fixture(); index = next(i for i,e in enumerate(fixture.capture.events)
                    if e.get('phase') == 'telemetry' and e.get('kind') == kind)
                fixture.capture.events.pop(index)
                with self.assertRaises(ValueError): fixture.plan()

    def test_duplicate_matching_reply_wrong_raw_value_or_uid_rejected(self):
        fixture = Fixture(); frame = next(e for e in fixture.capture.events
            if e.get('phase') == 'telemetry' and e.get('kind') == 'can_rx_frame')
        fixture.capture.events.append(copy.deepcopy(frame))
        with self.assertRaises(ValueError): fixture.plan()
        fixture = Fixture(); frame = next(e for e in fixture.capture.events
            if e.get('phase') == 'telemetry' and e.get('kind') == 'can_rx_frame')
        wrong = wire(0x1FE, bytes(8)); frame.update(m._frame(wrong.hex()).record())
        with self.assertRaises(ValueError): fixture.plan()

    def test_discovery_timeout_requires_original_boundary_not_absence_assertion(self):
        fixture = Fixture(); fixture.capture.events = [e for e in fixture.capture.events
            if not (e.get('phase') == 'discovery' and e.get('kind') == 'can_timeout')]
        with self.assertRaisesRegex(ValueError, 'no-fresh-response'): fixture.plan()

    def test_trace_bytes_count_sha_and_complete_status_bound(self):
        inputs = Fixture().inputs()
        for mutation in ('raw', 'ref', 'receipt'):
            item = copy.deepcopy(inputs)
            if mutation == 'raw': item['events']['raw_jsonl'] += '\n'
            elif mutation == 'ref': item['events']['reference']['sha256'] = 'a'*64
            else:
                doc = json.loads(item['topology']['raw_json']); doc['trace']['complete'] = False
                item['topology'] = envelope(doc,'topology.json')
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): m.prepare_model_plan(**item)

    def test_nominal_identity_sign_offset_and_artifact_pin_must_match(self):
        for key in ('uid', 'sign', 'offset_rad'):
            fixture = Fixture(); fixture.profile['axes']['1'][key] = '0'*16 if key == 'uid' else -1 if key == 'sign' else .1
            with self.subTest(key=key), self.assertRaises(ValueError): fixture.plan()
        inputs = Fixture().inputs(); inputs['calibration']['raw_json'] += ' '
        with self.assertRaises(ValueError): m.prepare_model_plan(**inputs)

    def test_source_proofs_must_be_distinct_and_keep_all_seven_warm_sources(self):
        for mutation in ('alias', 'missing_warm', 'wrong_digest', 'grant', 'reuse'):
            fixture = Fixture()
            if mutation == 'alias': fixture.source['four_bus_source_manifest'] = fixture.source['frozen_model_source_manifest']
            elif mutation == 'missing_warm': del fixture.source['frozen_model_source_sha256'][m.MODEL_SOURCES[3]]
            elif mutation == 'wrong_digest': fixture.source['frozen_model_source_sha256'][m.MODEL_SOURCES[0]] = '1'*64
            elif mutation == 'grant': fixture.source['output_allowed'] = True
            else: fixture.source['old_pose_approval_reused'] = True
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): fixture.plan()

    def test_acceleration_hypothesis_selection_and_pins_preserved(self):
        fixture = Fixture(); fixture.profile['accel_input_hypothesis'] = True
        with self.assertRaises(ValueError): fixture.plan()
        fixture.accel = {'formal_calibration_approved': False, 'grants_motor_output': False}
        plan = fixture.plan()
        self.assertEqual(plan['observer_kwargs']['accel_input_hypothesis'], fixture.profile['artifacts']['accel_input_hypothesis'])
        self.assertFalse(plan['observer_kwargs']['apply_reviewed_accel_calibration'])

    def test_invalid_json_duplicate_nonfinite_and_relative_refs(self):
        inputs = Fixture().inputs()
        for raw in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":1e10000}'):
            item = copy.deepcopy(inputs); item['bias'] = {'raw_json':raw,
                'reference': {'path':'/fixture/bias.json','sha256':hashlib.sha256(raw.encode()).hexdigest()}}
            with self.subTest(raw=raw), self.assertRaises(ValueError): m.prepare_model_plan(**item)
        item = copy.deepcopy(inputs); item['bias']['reference']['path'] = 'relative.json'
        with self.assertRaises(ValueError): m.prepare_model_plan(**item)


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.plan = Fixture().plan()
        self.raw, self.imu, self.tick = records(self.plan)

    def test_full12_projection_original_math_clocks_order_and_raw_preserved(self):
        images = {p:bytes(r) for p,r in self.raw.items()}
        imu = copy.deepcopy(self.imu)
        snapshot = m.snapshot_from_four_records(self.plan,self.raw,self.imu,self.tick)
        self.assertEqual({p:bytes(r) for p,r in self.raw.items()}, images)
        self.assertEqual(self.imu, imu)
        self.assertEqual(len(snapshot['motors']),24)
        self.assertEqual([(r['motor_id'],r['parameter']) for r in snapshot['motors']],
                         [(i,p) for i in m.IDS for p in ('position','velocity')])
        for port in t.PORTS:
            for record, mid in zip(self.raw[port],self.plan['topology_by_port'][port]):
                p,v,_,_ = struct.unpack('>4H',bytes(record.rx)[7:15])
                row = next(r for r in snapshot['motors'] if r['motor_id']==mid and r['parameter']=='position')
                self.assertEqual(struct.pack('d',row['value']),struct.pack('d',p*(2.*12.57)/65535.-12.57))
                self.assertEqual(row['request_ns'], record.start_ns)
        self.assertFalse(snapshot['source_flags']['synthetic_six_axis_stats_created'])
        self.assertFalse(snapshot['source_flags']['full16_input_final_gate_verified_here'])

    def test_wrong_physical_port_duplicate_id_partial_and_non_owned_buffers_rejected(self):
        for kind in ('cross', 'duplicate','partial','list'):
            raw = {p:type(r).from_buffer_copy(bytes(r)) for p,r in self.raw.items()}
            if kind=='cross': raw['port0'],raw['port1']=raw['port1'],raw['port0']
            elif kind=='duplicate': raw['port0'][1] = raw['port0'][0]
            elif kind=='partial': raw['port0'][0].received=11
            else: raw['port0']=list(raw['port0'])
            with self.subTest(kind=kind),self.assertRaises(ValueError): m.snapshot_from_four_records(self.plan,raw,self.imu,self.tick)

    def test_type1_noncanonical_stop_mode_fault_and_bad_id_rejected(self):
        for mutation in ('tx','mode','fault','bad_id'):
            raw = {p:type(r).from_buffer_copy(bytes(r)) for p,r in self.raw.items()}
            r=raw['port0'][0]
            if mutation=='tx': r.tx[7]=1
            else:
                frame=m._frame(bytes(r.rx).hex()); cid=frame.can_id
                if mutation=='mode': cid |= 2 <<22
                elif mutation=='fault': cid |= 1 <<16
                else: cid ^= 1 <<8
                r.rx[:]=wire(cid,frame.data)
            with self.subTest(mutation=mutation),self.assertRaises(ValueError): m.snapshot_from_four_records(self.plan,raw,self.imu,self.tick)

    def test_deadline_equal_reply_future_imu_and_stale_input_rejected(self):
        raw={p:type(r).from_buffer_copy(bytes(r)) for p,r in self.raw.items()}
        raw['port0'][0].deadline_ns=raw['port0'][0].received_ns
        with self.assertRaises(ValueError): m.snapshot_from_four_records(self.plan,raw,self.imu,self.tick)
        imu=copy.deepcopy(self.imu); imu['read_finished_monotonic_ns']=self.tick+1
        with self.assertRaises(ValueError): m.snapshot_from_four_records(self.plan,self.raw,imu,self.tick)
        with self.assertRaises(ValueError): m.snapshot_from_four_records(self.plan,self.raw,self.imu,self.tick+20_000_000)

    def test_pre_capture_request_and_changed_local_branch_rejected(self):
        raw={p:type(r).from_buffer_copy(bytes(r)) for p,r in self.raw.items()}
        raw['port0'][0].start_ns=self.plan['axes']['1']['capture_last_position_reply_ns']
        with self.assertRaisesRegex(ValueError,'follow'): m.snapshot_from_four_records(self.plan,raw,self.imu,self.tick)
        raw={p:type(r).from_buffer_copy(bytes(r)) for p,r in self.raw.items()}
        r=raw['port0'][0]; frame=m._frame(bytes(r.rx).hex())
        r.rx[:]=wire(frame.can_id,struct.pack('>4H',32768,32768,32768,250))
        with self.assertRaisesRegex(ValueError,'immutable'): m.snapshot_from_four_records(self.plan,raw,self.imu,self.tick)

    def test_plan_mutation_not_accepted(self):
        plan=copy.deepcopy(self.plan);plan['axes']['1']['fixed_offset_rad']+=.1
        with self.assertRaisesRegex(ValueError,'mutated'): m.snapshot_from_four_records(plan,self.raw,self.imu,self.tick)

    def test_observer_factory_preserves_selected_wrapper_and_all_original_kwargs(self):
        calls=[]
        class Observer:
            def consume(self,snapshot): calls.append(snapshot);return 'ordinary_result'
            def prepare_run(self,**kwargs):return kwargs
        def factory(policy,calibration,**kwargs):
            calls.append((policy,calibration,kwargs));return Observer()
        policy,wrapper,torch=object(),object(),object()
        guarded=m.create_guarded_observer(self.plan,factory,policy=policy,max_ticks=5,
                                          torch_module=torch,checked_dispatch_wrapper=wrapper)
        received=calls[0];self.assertIs(received[0],policy)
        self.assertIs(received[2]['checked_dispatch_wrapper'],wrapper)
        self.assertIs(received[2]['torch_module'],torch)
        self.assertEqual(received[2]['h_hypothesis'],0.)
        self.assertEqual(received[2]['max_age_ns'],100_000_000)
        self.assertEqual(guarded.prepare_run(warmup_completed=True),{'warmup_completed':True})
        snapshot=m.snapshot_from_four_records(self.plan,self.raw,self.imu,self.tick)
        self.assertEqual(guarded.consume(snapshot),'ordinary_result')

    def test_guard_freezes_axes_and_invalid_position_never_calls_original(self):
        calls=[]
        class Observer:
            def consume(self,snapshot):calls.append(snapshot);return True
        plan=copy.deepcopy(self.plan)
        guarded=m.create_guarded_observer(plan,lambda *args,**kwargs:Observer(),
                                          policy=object(),max_ticks=501,checked_dispatch_wrapper=object())
        plan['axes']['1']['local_bounds_rad']=[-999,999]
        snapshot=m.snapshot_from_four_records(self.plan,self.raw,self.imu,self.tick)
        snapshot['motors'][0]['value']+=1
        with self.assertRaises(ValueError):guarded.consume(snapshot)
        self.assertEqual(calls,[])

    def test_no_implicit_wrapper_or_nonfinite_scope_factory(self):
        factory=lambda *args,**kwargs:None
        with self.assertRaises(ValueError):m.create_guarded_observer(self.plan,factory,policy=object(),max_ticks=5)
        with self.assertRaises(ValueError):m.create_guarded_observer(self.plan,factory,policy=object(),max_ticks=60,checked_dispatch_wrapper=object())

    def test_batch_builder_freezes_plan_and_keeps_four_physical_raw_images(self):
        from experiments.four_bus_diagnostic.transport_adapter import Batch,Group
        from singularitydog_hw.native_active_transport import Stats
        from singularitydog_hw.policy_output_runtime import decode_records
        batches={}
        for port, raw in self.raw.items():
            stats=Stats();stats.begin_ns=min(r.start_ns for r in raw)
            stats.end_ns=max(r.received_ns for r in raw)
            batches[port]=Batch(Group(port,tuple(self.plan['topology_by_port'][port])), 'feedback',
                raw,stats,decode_records((raw,stats)),bytes(raw),bytes(stats),stats.end_ns)
        plan=copy.deepcopy(self.plan);build=m.batch_snapshot_builder(plan)
        plan['axes']['1']['fixed_offset_rad']+=100
        actual=build(batches,self.imu,self.tick)
        expected=m.snapshot_from_four_records(self.plan,self.raw,self.imu,self.tick)
        self.assertEqual(actual,expected)
        self.assertEqual([len(b.records) for b in batches.values()],[3]*4)
        batches['port0'].records[0].rx[7]^=1
        with self.assertRaisesRegex(ValueError,'changed'):build(batches,self.imu,self.tick)

    def test_batch_builder_rejects_noncurrent_label_foreign_type_or_port(self):
        from experiments.four_bus_diagnostic.transport_adapter import Batch,Group
        from singularitydog_hw.native_active_transport import Stats
        from singularitydog_hw.policy_output_runtime import decode_records
        build=m.batch_snapshot_builder(self.plan)
        batches={}
        for port, raw in self.raw.items():
            stats=Stats()
            batches[port]=Batch(Group(port,tuple(self.plan['topology_by_port'][port])), 'output_stop',
                raw,stats,decode_records((raw,stats)),bytes(raw),bytes(stats),self.tick)
        with self.assertRaises(ValueError):build(batches,self.imu,self.tick)
        with self.assertRaises(ValueError):build(self.raw,self.imu,self.tick)


class CombinedSnapshotTests(unittest.TestCase):
    """Owned mixed4 fixtures; the three feedback slots are direct views."""
    def setUp(self):
        self.plan = Fixture().plan()
        self.raw, self.imu, self.tick = records(self.plan)

    def batches(self, *, voltage=39., rotate=0):
        from experiments.four_bus_diagnostic.transport_adapter import Batch, Group
        from singularitydog_hw.native_active_transport import Stats
        from singularitydog_hw.policy_output_runtime import decode_records
        result = {}
        for port, original in self.raw.items():
            raw = (Record * 4)()
            for i, record in enumerate(original): raw[i] = record
            ids = tuple(self.plan['topology_by_port'][port])
            mid = ids[rotate]
            row = raw[3]
            row.start_ns = max(record.received_ns for record in original) + 10_000
            row.finish_ns = row.start_ns + 1_000
            row.read_start_ns = row.finish_ns + 1_000
            row.received_ns = row.read_start_ns + 1_000
            row.deadline_ns = original[0].deadline_ns
            row.written = row.received = 17
            row.tx[:] = codec.read_request(mid, 'voltage')
            row.rx[:] = wire((17 << 24) | (mid << 8) | codec.HOST_ID,
                             codec.PARAMETERS['voltage'][0].to_bytes(2, 'little') +
                             bytes(2) + struct.pack('<f', voltage))
            stats = Stats()
            stats.begin_ns = min(record.start_ns for record in raw) - 100
            stats.end_ns = max(record.received_ns for record in raw) + 100
            stats.writes = 4; stats.bytes = 68; stats.waits = stats.reads = 4
            result[port] = Batch(Group(port, ids), 'acquisition_combined4', raw, stats,
                decode_records((raw, stats)), bytes(raw), bytes(stats), stats.end_ns + 100)
        return result

    @staticmethod
    def reseal(batch, *, decode=True, **changes):
        from dataclasses import replace
        from singularitydog_hw.policy_output_runtime import decode_records
        return replace(batch, record_image=bytes(batch.records), stats_image=bytes(batch.stats),
                       rows=decode_records((batch.records, batch.stats)) if decode else batch.rows,
                       **changes)

    def test_same_motor_and_imu_bits_with_truthful_four_record_provenance(self):
        build = m.combined_batch_snapshot_builder(self.plan)
        for rotate in range(3):
            batches = self.batches(rotate=rotate)
            images = {port: (bytes(batch.records), bytes(batch.stats), copy.deepcopy(batch.rows))
                      for port, batch in batches.items()}
            with patch('builtins.open', side_effect=AssertionError('Combined projection opened file')):
                actual = build(batches, self.imu, self.tick)
            expected = m.snapshot_from_four_records(self.plan, self.raw, self.imu, self.tick)
            for key in expected.keys() - {'source_flags'}:
                self.assertEqual(actual[key], expected[key])
            for got, before in zip(actual['motors'], expected['motors']):
                self.assertEqual(struct.pack('d', got['value']), struct.pack('d', before['value']))
            flags = actual['source_flags']
            self.assertEqual(flags['physical_record_count_by_port'], {port: 4 for port in t.PORTS})
            self.assertEqual(flags['projected_feedback_record_count_by_port'], {port: 3 for port in t.PORTS})
            self.assertEqual(flags['rotating_voltage_record_index_by_port'], {port: 3 for port in t.PORTS})
            self.assertFalse(flags['v3_voltage_overlap_pending_at_inference'])
            self.assertTrue(flags['combined_acquisition_projection'])
            self.assertFalse(flags['synthetic_six_axis_stats_created'])
            self.assertFalse(flags['full16_input_final_gate_verified_here'])
            self.assertFalse(flags['approved_for_runtime'])
            self.assertFalse(actual['output_allowed'])
            self.assertEqual(images, {port: (bytes(batch.records), bytes(batch.stats), batch.rows)
                                      for port, batch in batches.items()})

    def test_projection_aliases_original_slots_and_never_creates_Stats(self):
        import ctypes as C
        batches = self.batches()
        for port, batch in batches.items():
            view = m._combined_feedback_view(batch, port, batch.group.ids, self.tick)
            self.assertEqual(C.addressof(view), C.addressof(batch.records))
            self.assertEqual(view._b_needsfree_, 0)
            for i in range(3):
                self.assertEqual(C.addressof(view[i]), C.addressof(batch.records[i]))
            original = batch.records[0].rx[7]
            view[0].rx[7] ^= 1
            self.assertEqual(batch.records[0].rx[7], original ^ 1)
            with self.assertRaisesRegex(ValueError, 'changed'):
                batch.verify()
            batch.records[0].rx[7] = original
            self.assertEqual(len(batch.records), 4)

    def test_old_default_and_combined_scope_are_explicitly_disjoint(self):
        batches = self.batches()
        with self.assertRaises(ValueError):
            m.batch_snapshot_builder(self.plan)(batches, self.imu, self.tick)
        from dataclasses import replace
        batches['port0'] = replace(batches['port0'], label='feedback')
        with self.assertRaisesRegex(ValueError, 'combined4'):
            m.combined_batch_snapshot_builder(self.plan)(batches, self.imu, self.tick)
        with self.assertRaises(ValueError):
            m.combined_batch_snapshot_builder(self.plan)(self.raw, self.imu, self.tick)

    def test_unhealthy_fourth_voltage_is_rejected_before_observer(self):
        for voltage in (34.99, 42.01):
            with self.subTest(voltage=voltage), self.assertRaisesRegex(ValueError, '35..42'):
                m.combined_batch_snapshot_builder(self.plan)(self.batches(voltage=voltage), self.imu, self.tick)
        batches = self.batches(); row = batches['port0'].records[3]
        row.rx[11:15] = struct.pack('<f', float('nan'))
        with self.assertRaises(ValueError):
            m.combined_batch_snapshot_builder(self.plan)(batches, self.imu, self.tick)

    def test_fourth_reply_partial_late_future_and_bad_stats_reject(self):
        for kind in ('partial', 'late', 'read_start', 'future_owner', 'writes', 'bytes', 'rejected', 'stats_end'):
            batches = self.batches(); batch = batches['port0']; row = batch.records[3]
            if kind == 'partial': row.received = 11
            elif kind == 'late': row.deadline_ns = row.received_ns
            elif kind == 'read_start': row.read_start_ns = row.finish_ns - 1
            elif kind == 'future_owner': batches['port0'] = self.reseal(batch, completed_ns=self.tick + 1)
            elif kind == 'writes': batch.stats.writes = 3
            elif kind == 'bytes': batch.stats.bytes = 51
            elif kind == 'rejected': batch.stats.rejected_size = batch.stats.rejected_total = 1
            elif kind == 'stats_end': batch.stats.end_ns = row.received_ns - 1
            if kind != 'future_owner': batches['port0'] = self.reseal(batch, decode=False)
            with self.subTest(kind=kind), self.assertRaises((ValueError, RuntimeError)):
                m.combined_batch_snapshot_builder(self.plan)(batches, self.imu, self.tick)

    def test_mixed_same_id_matches_kind_and_parameter_not_just_motor_id(self):
        for kind in ('wrong_parameter', 'wrong_motor', 'feedback_in_voltage_slot', 'duplicate_feedback', 'wrong_order'):
            batches = self.batches(); batch = batches['port0']; ids = batch.group.ids
            if kind == 'wrong_parameter':
                batch.records[3].tx[:] = codec.read_request(ids[0], 'current')
                batch.records[3].rx[7:9] = codec.PARAMETERS['current'][0].to_bytes(2, 'little')
            elif kind == 'wrong_motor':
                mid = self.plan['topology_by_port']['port1'][0]
                batch.records[3].tx[:] = codec.read_request(mid, 'voltage')
                batch.records[3].rx[:] = wire((17 << 24) | (mid << 8) | codec.HOST_ID,
                                             bytes(batch.records[3].rx)[7:15])
            elif kind == 'feedback_in_voltage_slot': batch.records[3] = batch.records[0]
            elif kind == 'duplicate_feedback': batch.records[1] = batch.records[0]
            else:
                before = Record.from_buffer_copy(bytes(batch.records[0]))
                batch.records[0] = batch.records[1]; batch.records[1] = before
            batches['port0'] = self.reseal(batch, decode=False)
            with self.subTest(kind=kind), self.assertRaises((ValueError, RuntimeError)):
                m.combined_batch_snapshot_builder(self.plan)(batches, self.imu, self.tick)

    def test_fourth_status_or_reserved_bytes_and_feedback_mode_fault_reject(self):
        for kind in ('status', 'reserved', 'mode', 'fault'):
            batches = self.batches(); batch = batches['port0']
            if kind == 'reserved': batch.records[3].rx[9] = 1
            else:
                index = 3 if kind == 'status' else 0
                frame = m._frame(bytes(batch.records[index].rx).hex())
                bit = (1 << 16) if kind in ('status', 'fault') else (2 << 22)
                batch.records[index].rx[:] = wire(frame.can_id | bit, frame.data)
            batches['port0'] = self.reseal(batch, decode=False)
            with self.subTest(kind=kind), self.assertRaises((ValueError, RuntimeError)):
                m.combined_batch_snapshot_builder(self.plan)(batches, self.imu, self.tick)

    def test_complete_four_port_set_wrong_port_and_borrowed_buffers_reject(self):
        from dataclasses import replace
        for kind in ('missing', 'wrong_port', 'records_view', 'stats_view'):
            batches = self.batches(); batch = batches['port0']
            if kind == 'missing': batches.pop('port3')
            elif kind == 'wrong_port': batches['port0'] = batches['port1']
            elif kind == 'records_view':
                batches['port0'] = replace(batch, records=(Record * 4).from_buffer(batch.records))
            else:
                batches['port0'] = replace(batch, stats=type(batch.stats).from_buffer(batch.stats))
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                m.combined_batch_snapshot_builder(self.plan)(batches, self.imu, self.tick)

    def test_stale_imu_local_branch_and_plan_mutation_keep_original_guards(self):
        plan = copy.deepcopy(self.plan); build = m.combined_batch_snapshot_builder(plan)
        plan['axes']['1']['fixed_offset_rad'] += 100.
        actual = build(self.batches(), self.imu, self.tick)
        self.assertEqual(actual['motors'][0]['value'],
                         m.snapshot_from_four_records(self.plan, self.raw, self.imu, self.tick)['motors'][0]['value'])
        imu = copy.deepcopy(self.imu); imu['read_finished_monotonic_ns'] = self.tick + 1
        with self.assertRaises(ValueError): build(self.batches(), imu, self.tick)
        with self.assertRaises(ValueError): build(self.batches(), self.imu, self.tick + 20_000_000)
        batches = self.batches(); batch = batches['port0']; row = batch.records[0]
        frame = m._frame(bytes(row.rx).hex())
        row.rx[:] = wire(frame.can_id, struct.pack('>4H', 32768, 32768, 32768, 250))
        batches['port0'] = self.reseal(batch)
        with self.assertRaisesRegex(ValueError, 'immutable'):
            build(batches, self.imu, self.tick)

    def test_no_model_consume_when_combined_projection_fails(self):
        calls = []
        class Observer:
            def consume(self, snapshot): calls.append(snapshot); return True
        guarded = m.create_guarded_observer(self.plan, lambda *a, **kw: Observer(),
                    policy=object(), max_ticks=5, checked_dispatch_wrapper=object())
        batches = self.batches(); batches['port3'].records[3].received = 0
        with self.assertRaises(ValueError):
            guarded.consume(m.combined_batch_snapshot_builder(self.plan)(batches, self.imu, self.tick))
        self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
