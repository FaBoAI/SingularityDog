"""File-only, cold-start Trial28A policy diagnostics; never opens robot hardware.

Every complete telemetry frame is an independent reset, with explicit h=0/h=1
sensitivity hypotheses. This is neither a live50Hz controller nor motor output.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys

MODEL_SHA256 = '7b06b035894c2696381a028d150609c67fef3a9a421cb001799152777e179434'
SOURCE_HASHES = {'model_149.pt': MODEL_SHA256,
    'swing_core.py': '7a4f6ee4d1f4ae165356d91ab160fef5ac67d2958e8d717846ebc10be9d4811a',
    'swing_deployment.py': '8ea25946b3e2799d509afc32490a4efafd752d04a40b6b94a9839a576da59bfa'}
OPTIONS = dict(use_plane=True, period=.56, duty=.60, swing_height=.035, residual_tau=0.,
               stance_widen_m=.02, heading_gain=.8, forward_command_limit=.46, hip_residual_scale=.27)
CAN_ORDER = [6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7]
PARAMETERS = {'run_mode':(0x7005, 'B', 'enum'), 'position':(0x7019, 'f', 'rad_output_shaft'),
    'current':(0x701A, 'f', 'A'), 'velocity':(0x701B, 'f', 'rad_s_output_shaft'),
    'voltage':(0x701C, 'f', 'V'), 'can_timeout':(0x7028, 'I', '50_us_ticks'), 'zero_state':(0x7029, 'B', 'enum')}
LOWER, UPPER = [-.5, -.9, -2.2]*4, [.5, 1.2, -.08]*4
IMU_ROTATION_TOLERANCE = 1e-6


def _pairs(pairs):
    result = {}
    for k, v in pairs:
        if k in result:
            raise ValueError('Duplicate JSON key')
        result[k] = v
    return result


def _json(text):
    def invalid(value):
        raise ValueError('Nonfinite JSON number: '+value)
    return json.loads(text, object_pairs_hook=_pairs, parse_constant=invalid)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_imu_mount_candidate(data):
    """Accept only an explicitly unverified, proper sensor-to-body rotation."""
    fields = {'schema_version', 'status', 'input_frame', 'output_frame',
              'R_body_from_sensor', 'raw_driver_axes_verified',
              'approved_for_runtime', 'provenance'}
    if (not isinstance(data, dict) or set(data) != fields
            or type(data.get('schema_version')) is not int or data['schema_version'] != 1
            or data.get('status') != 'IMU_MOUNT_CANDIDATE_ONLY'
            or data.get('input_frame') != 'sensor'
            or data.get('output_frame') != 'body_x_forward_y_left_z_up'
            or data.get('raw_driver_axes_verified') is not False
            or data.get('approved_for_runtime') is not False):
        raise ValueError('Require an unverified IMU mount candidate schema')
    provenance = data['provenance']
    if (not isinstance(provenance, dict) or not provenance
            or any(not isinstance(k, str) or not k.strip()
                   or not isinstance(v, str) or not v.strip() for k, v in provenance.items())):
        raise ValueError('IMU mount provenance must contain nonempty string fields')
    rotation = data['R_body_from_sensor']
    if (not isinstance(rotation, list) or len(rotation) != 3
            or any(not isinstance(row, list) or len(row) != 3 for row in rotation)):
        raise ValueError('IMU rotation must have exactly three rows of three numbers')
    tolerance = IMU_ROTATION_TOLERANCE
    # Bound entries before products, so malformed enormous values cannot overflow.
    if any(type(x) not in (int, float) or not -1-tolerance <= x <= 1+tolerance
           or not finite(x) for row in rotation for x in row):
        raise ValueError('IMU rotation entries must be finite real numbers')
    for i in range(3):
        for j in range(3):
            product = sum(rotation[i][k] * rotation[j][k] for k in range(3))
            if abs(product - (1. if i == j else 0.)) > tolerance:
                raise ValueError('IMU rotation must be orthonormal; no scale or shear')
    a, b, c = rotation
    determinant = (a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0])
                   + a[2]*(b[0]*c[1]-b[1]*c[0]))
    if abs(determinant - 1.) > tolerance:
        raise ValueError('IMU rotation must have determinant +1; reflections are rejected')
    return data


def load_imu_mount_candidate(path):
    path = Path(path).expanduser().resolve()
    raw = path.read_bytes()
    candidate = validate_imu_mount_candidate(_json(raw.decode('utf-8')))
    return {**candidate, 'mode': 'explicit_mount_candidate',
            'source': {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()},
            'sensor_alignment_verified': False}


def _imu_mount_hypothesis(assume_sensor_aligned, candidate_path):
    if type(assume_sensor_aligned) is not bool:
        raise ValueError('Sensor alignment assumption must be an explicit boolean')
    if assume_sensor_aligned == (candidate_path is not None):
        raise ValueError('Choose exactly one IMU hypothesis: aligned or mount candidate')
    if candidate_path is not None:
        return load_imu_mount_candidate(candidate_path)
    return {'mode': 'identity_hypothesis', 'R_body_from_sensor': [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
            'input_frame': 'sensor', 'output_frame': 'body_x_forward_y_left_z_up',
            'source': None, 'provenance': {'source': 'Explicit --assume-sensor-aligned hypothesis'},
            'raw_driver_axes_verified': False, 'sensor_alignment_verified': False,
            'approved_for_runtime': False}


def validate_calibration(data):
    if (data.get('status') != 'MANUAL_NOMINAL_CANDIDATES_ONLY'
            or data.get('formula') != 'q_model = sign * raw + offset; rad; no wrapping'
            or data.get('model_can_order_candidate') != CAN_ORDER):
        raise ValueError('Unsupported calibration candidate schema or model order')
    identities = data.get('identities', {})
    if set(identities) != {str(i) for i in range(1,13)}:
        raise ValueError('Calibration needs twelve identities')
    values = list(identities.values())
    if (any(not isinstance(v,str) or len(v)!=16 or any(c not in '0123456789abcdef' for c in v) for v in values)
            or len(set(values)) != 12):
        raise ValueError('Malformed or duplicate calibration identity')
    rows = data.get('candidates', [])
    if len(rows) != 12 or sorted(r.get('motor_id',0) for r in rows) != list(range(1,13)):
        raise ValueError('Calibration needs exactly twelve unique candidate rows')
    out = {}
    for row in rows:
        i = row['motor_id']
        if type(i) is not int or type(row.get('sign_candidate')) is not int or row['sign_candidate'] not in (-1,1):
            raise ValueError('Invalid candidate ID/sign')
        if not finite(row.get('offset_candidate_rad')):
            raise ValueError('Nonfinite candidate offset')
        out[i] = row
    return out


def _wire_record(event):
    raw = bytes.fromhex(event['hex'] if event['kind']=='can_tx' else event['wire_hex'])
    if len(raw)!=17 or raw[:2]!=b'AT' or raw[6]!=8 or raw[-2:]!=b'\r\n':
        raise ValueError('Malformed canonical eight-byte AT wire')
    encoded = int.from_bytes(raw[2:6],'big')
    if encoded & 7 != 4:
        raise ValueError('Nonextended wire')
    can_id = encoded >> 3
    return (can_id>>24)&31, (can_id>>8)&255, can_id&255, raw[7:15], can_id


def load_capture(directory, *, allow_incomplete_capture=False):
    directory = Path(directory)
    summary = _json((directory/'summary.json').read_text())
    plan = summary.get('plan',{})
    incomplete = summary.get('status') == 'INCOMPLETE'
    accepted_status = (summary.get('status') in ('COMPLETE','COMPLETE_WITH_WARNINGS') and summary.get('errors') == [])
    prefix_allowed = (incomplete and allow_incomplete_capture is True and summary.get('errors')
        and all(e.get('component')=='can' and e.get('error','').startswith("TimeoutError('No fresh response:") for e in summary['errors']))
    if (not (accepted_status or prefix_allowed)
            or plan.get('motor_output_available') is not False or plan.get('allowed_can_types') != [0,17]
            or plan.get('ids_bottom_to_top') != {'FR':[1,2,3],'FL':[4,5,6],'RR':[7,8,9],'RL':[10,11,12]}):
        raise ValueError('Require a completed, error-free, all-twelve read-only diagnose capture')
    records = [_json(line) for line in (directory/'events.jsonl').read_text().splitlines() if line.strip()]
    original_record_count = len(records)
    prefix_cutoff_ns = None
    if prefix_allowed:
        timeout_indexes = [i for i,e in enumerate(records) if e.get('kind')=='can_timeout']
        if not timeout_indexes:
            raise ValueError('Incomplete capture has no explicit logged timeout boundary')
        cutoff = timeout_indexes[0]
        replies = [i for i,e in enumerate(records[:cutoff]) if e.get('kind')=='motor_parameter']
        if not replies:
            raise ValueError('No completed pre-timeout telemetry prefix')
        records = records[:replies[-1]+1]
        prefix_cutoff_ns = records[-1]['monotonic_ns']
    outstanding, received, count = None, [], 0
    for event in records:
        kind = event.get('kind')
        if kind == 'can_timeout':
            raise ValueError('Capture has CAN timeout')
        if kind == 'can_tx':
            if outstanding is not None:
                raise ValueError('Overlapping CAN requests')
            t, source, i, payload, _ = _wire_record(event)
            name = event.get('parameter')
            if t not in (0,17) or source!=0xFD or i not in range(1,13) or event.get('motor_id')!=i:
                raise ValueError('Capture contains non-read-only or wrong-ID request')
            if name=='identity':
                expected_type, expected_payload = 0, bytes(8)
            elif name in PARAMETERS:
                expected_type, expected_payload = 17, struct.pack('<H',PARAMETERS[name][0])+bytes(6)
            else:
                raise ValueError('Unknown requested parameter')
            if t!=expected_type or payload!=expected_payload:
                raise ValueError('Request bytes disagree with parameter')
            outstanding, received = event, []
        elif kind == 'can_rx_frame' and outstanding is not None:
            received.append(event)
        elif kind == 'motor_parameter':
            if outstanding is None:
                raise ValueError('Parameter without preceding request')
            i, name = event.get('motor_id'), event.get('parameter')
            if (i!=outstanding['motor_id'] or name!=outstanding['parameter']
                    or event.get('sequence')!=outstanding.get('sequence')
                    or event.get('request_monotonic_ns')!=outstanding.get('monotonic_ns')
                    or event.get('monotonic_ns',0)<=outstanding['monotonic_ns']):
                raise ValueError('Parameter chronology/correlation mismatch')
            match = None
            for frame in received:
                t, source, dest, payload, can_id = _wire_record(frame)
                if source!=i or t!=(0 if name=='identity' else 17) or dest!=(0xFE if name=='identity' else 0xFD):
                    continue
                if name!='identity' and int.from_bytes(payload[:2],'little')!=PARAMETERS[name][0]:
                    continue
                match = payload, can_id
            if match is None:
                raise ValueError('No matching raw received frame')
            payload, can_id = match
            if name=='identity':
                if event.get('ok') is not True or payload.hex()!=event.get('mcu_uid_hex'):
                    raise ValueError('Identity does not match raw bytes')
            else:
                status = (can_id>>16)&255
                if status or payload[2:4]!=bytes(2):
                    if not (name in ('can_timeout','zero_state') and status and payload[2:4]==bytes(2)
                            and event.get('ok') is False and event.get('status')==status):
                        raise ValueError('Rejected/malformed required parameter')
                else:
                    value = struct.unpack_from('<'+PARAMETERS[name][1],payload,4)[0]
                    if (event.get('ok') is not True or not finite(value) or event.get('value')!=value
                            or event.get('unit')!=PARAMETERS[name][2]):
                        raise ValueError('Parameter value/unit disagrees with raw bytes')
            count += 1
            outstanding = None
    if outstanding is not None:
        raise ValueError('Unanswered final CAN request')
    return records, {'directory':str(directory.resolve()), 'raw_redecoded_parameter_count':count,
        'summary_sha256':sha(directory/'summary.json'),'events_sha256':sha(directory/'events.jsonl'),
        'source_status':summary['status'], 'source_errors':summary.get('errors',[]),
        'incomplete_source_accepted_for_diagnostics_only':bool(prefix_allowed),
        'prefix_cutoff_monotonic_ns':prefix_cutoff_ns,
        'excluded_tail_record_count':original_record_count-len(records)}


def build_samples(records, calibration, *, assume_sensor_aligned=False,
                  imu_mount_candidate=None, max_frames=10):
    mount = _imu_mount_hypothesis(assume_sensor_aligned, imu_mount_candidate)
    if type(max_frames) is not int or not 1<=max_frames<=10:
        raise ValueError('max_frames must be1..10')
    rows = validate_calibration(calibration)
    identities = {}
    imu, parameters = [], []
    for event in records:
        if event.get('kind')=='imu':
            if event.get('frame')!='sensor':
                raise ValueError('IMU frame is not original sensor coordinates')
            accel, gyro = event.get('accel_m_s2'), event.get('gyro_rad_s')
            if any(not isinstance(v,list) or len(v)!=3 or not all(finite(x) for x in v) for v in (accel,gyro)):
                raise ValueError('Nonfinite/malformed IMU vector')
            start, stamp, finish = (event.get(k) for k in ('read_started_monotonic_ns','monotonic_ns','read_finished_monotonic_ns'))
            if any(type(t) is not int or t<=0 for t in (start,stamp,finish)) or not start<=stamp<=finish:
                raise ValueError('Invalid IMU times')
            if not finite(math.hypot(*accel)) or math.hypot(*accel)<=1e-9:
                raise ValueError('Zero gravity proxy norm')
            imu.append(event)
        elif event.get('kind')=='motor_parameter':
            i, name = event.get('motor_id'), event.get('parameter')
            if type(i) is not int or i not in rows:
                raise ValueError('Invalid motor source')
            if name=='identity':
                if event.get('ok') is not True or event.get('mcu_uid_hex')!=calibration['identities'][str(i)]:
                    raise ValueError('Capture/calibration identity mismatch')
                identities[i] = event['mcu_uid_hex']
            if name in ('position','velocity'):
                if (event.get('ok') is not True or event.get('status')!=0 or event.get('unit')!=PARAMETERS[name][2]
                        or not finite(event.get('value')) or type(event.get('monotonic_ns')) is not int
                        or event['monotonic_ns']<=0):
                    raise ValueError('Malformed/nonfinite position or velocity')
                parameters.append(event)
    if set(identities)!=set(range(1,13)) or len(set(identities.values()))!=12:
        raise ValueError('Missing or duplicate motor identities')
    if not imu:
        raise ValueError('No IMU samples')
    imu.sort(key=lambda e:e['read_finished_monotonic_ns'])
    parameters.sort(key=lambda e:e['monotonic_ns'])
    if len({e['monotonic_ns'] for e in parameters})!=len(parameters):
        raise ValueError('Duplicate parameter receive timestamp')
    expected = [(i,n) for i in range(1,13) for n in ('position','velocity')]
    complete, pending = [], []
    for event in parameters:
        if (event['motor_id'],event['parameter']) != expected[len(pending)]:
            raise ValueError('Position/velocity sweep has missing, duplicate or reordered IDs')
        pending.append(event)
        if len(pending)!=24:
            continue
        end = pending[-1]['monotonic_ns']
        past = [x for x in imu if x['read_finished_monotonic_ns']<=end]
        if not past:
            raise ValueError('No causal preceding IMU sample')
        chosen = past[-1]
        values = {(e['motor_id'],e['parameter']):e['value'] for e in pending}
        q = [rows[i]['sign_candidate']*values[(i,'position')]+rows[i]['offset_candidate_rad'] for i in CAN_ORDER]
        dq = [rows[i]['sign_candidate']*values[(i,'velocity')] for i in CAN_ORDER]
        if not all(finite(v) for v in q+dq):
            raise ValueError('Nonfinite calibrated input')
        norm = math.hypot(*chosen['accel_m_s2'])
        rotation = mount['R_body_from_sensor']
        accel_body = [sum(r[j]*chosen['accel_m_s2'][j] for j in range(3)) for r in rotation]
        gyro_body = [sum(r[j]*chosen['gyro_rad_s'][j] for j in range(3)) for r in rotation]
        if not all(finite(v) for v in accel_body+gyro_body):
            raise ValueError('Nonfinite rotated IMU vector')
        gravity_body = [-v/norm for v in accel_body]
        identity_hypothesis = mount['mode'] == 'identity_hypothesis'
        times = [e['monotonic_ns'] for e in pending]
        complete.append({'frame_monotonic_ns':end,'can_sample_times_ns':times,
            'can_skew_s':(max(times)-min(times))/1e9,'imu_monotonic_ns':chosen['monotonic_ns'],
            'imu_read_finished_monotonic_ns':chosen['read_finished_monotonic_ns'],
            'imu_age_s':(end-chosen['monotonic_ns'])/1e9,'q_model_rad':q,'dq_model_rad_s':dq,
            'gyro_rad_s':gyro_body,'gravity_body_unit':gravity_body,
            'accel_body_candidate_m_s2':accel_body,
            'raw_accel_m_s2':list(chosen['accel_m_s2']),'raw_accel_norm_m_s2':norm,
            'raw_gyro_rad_s':list(chosen['gyro_rad_s']),
            'raw_accel_norm_reference_m_s2':9.80665,
            'raw_accel_norm_relative_deviation':norm/9.80665-1.,
            'projected_gravity_z_positive':gravity_body[2]>0,
            'identity_hypothesis_projected_gravity_z_positive':gravity_body[2]>0 if identity_hypothesis else None,
            'imu_mount':mount,
            'violations':[{'motor_id':i,'q_rad':v,'lower_rad':lo,'upper_rad':hi} for i,v,lo,hi in zip(CAN_ORDER,q,LOWER,UPPER) if not lo<=v<=hi],
            'assumptions':{'sensor_to_body_rotation':('identity hypothesis only' if identity_hypothesis
                else 'explicit mount candidate; raw driver axes unverified'),'gyro_bias_subtracted':False,
                'gyro_bias_rad_s_assumed':[0.,0.,0.], 'sensor_alignment_verified':False,
                'raw_driver_axes_verified':False, 'accel_bias_subtracted':False, 'accel_scale_corrected':False,
                'gravity_fusion_verified':False,'calibration_verified':False,'exposure_h_available':False},
            'motor_output_available':False})
        pending=[]
    if not complete:
        raise ValueError('No complete all-twelve position/velocity frame')
    for sample in complete:
        sample['complete_frames_available'] = len(complete)
        sample['discarded_incomplete_tail_position_velocity_records'] = len(pending)
    indexes = sorted({round(i*(len(complete)-1)/(max_frames-1)) for i in range(max_frames)}) if max_frames>1 else [0]
    return [complete[i] for i in indexes]


def load_policy(bundle):
    bundle = Path(bundle).resolve()
    actual = {name:sha(bundle/name) for name in SOURCE_HASHES}
    if actual != SOURCE_HASHES:
        raise ValueError('Model/controller bundle hash mismatch')
    import torch
    cp = torch.load(bundle/'model_149.pt',map_location='cpu',weights_only=True)
    if type(cp.get('iter')) is not int or cp['iter']!=149:
        raise ValueError('Wrong Trial28A iteration')
    shapes = {f'mlp.{index}.{suffix}':shape for index,ins,outs in ((0,74,128),(2,128,128),(4,128,64),(6,64,12))
              for suffix,shape in (('weight',(outs,ins)),('bias',(outs,)))}
    shapes['distribution.std_param']=(12,)
    state = cp['actor_state_dict']
    if set(state)!=set(shapes) or any(tuple(state[k].shape)!=shape or state[k].dtype!=torch.float32
            or not bool(torch.isfinite(state[k]).all()) for k,shape in shapes.items()):
        raise ValueError('Wrong/nonfinite actor architecture')
    actor=torch.nn.Sequential(torch.nn.Linear(74,128),torch.nn.ELU(),torch.nn.Linear(128,128),
                             torch.nn.ELU(),torch.nn.Linear(128,64),torch.nn.ELU(),torch.nn.Linear(64,12))
    actor.load_state_dict({k.removeprefix('mlp.'):v for k,v in state.items() if k.startswith('mlp.')},strict=True)
    previous = sys.modules.get('swing_core')
    try:
        modules={}
        for name in ('swing_core','swing_deployment'):
            spec=importlib.util.spec_from_file_location(name,bundle/(name+'.py'))
            module=importlib.util.module_from_spec(spec)
            if name=='swing_core':sys.modules[name]=module
            spec.loader.exec_module(module)
            modules[name]=module
        policy=modules['swing_deployment'].DeployableSwingPolicy(actor,**OPTIONS).eval()
    finally:
        if previous is None:sys.modules.pop('swing_core',None)
        else:sys.modules['swing_core']=previous
    torch.set_num_threads(1)
    return policy, {'bundle':str(bundle),'sha256':actual,'controller_options':OPTIONS,
                    'actor_architecture':'74-128ELU-128ELU-64ELU-12 deterministic mean'}


def evaluate_samples(samples, policy, torch_module=None):
    if torch_module is None:
        import torch as torch_module
    torch=torch_module
    results=[]
    with torch.inference_mode():
        for sample in samples:
            scenarios=[]
            for h_value in (0.,1.):
                policy.reset(torch.tensor([0],dtype=torch.long))
                tensor=lambda x:torch.tensor([x],dtype=torch.float32)
                output=policy(tensor(sample['gyro_rad_s']),tensor(sample['gravity_body_unit']),
                    tensor([0.,0.,0.]),tensor(sample['q_model_rad']),tensor(sample['dq_model_rad_s']),tensor([h_value]*12))
                values=output.detach().cpu().tolist()[0]
                raw=policy.last_actor_output.detach().cpu().tolist()[0]
                obs=policy.last_observation.detach().cpu().tolist()[0]
                if len(values)!=12 or len(raw)!=12 or len(obs)!=74 or not all(finite(x) for x in values+raw+obs):
                    raise ValueError('Nonfinite or wrong-shaped policy result')
                scenarios.append({'h_hypothesis':h_value,'h_measured':False,'independent_cold_reset':True,
                    'q_target_rad_diagnostic_only':values,'actor_mean':raw,'observation74':obs,
                    'max_target_current_gap_rad':max(abs(a-b) for a,b in zip(values,sample['q_model_rad']))})
            results.append({**sample,'scenarios':scenarios,
                'h_extreme_target_difference_rad':max(abs(a-b) for a,b in zip(scenarios[0]['q_target_rad_diagnostic_only'],scenarios[1]['q_target_rad_diagnostic_only']))})
    return results


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('bundle','capture','calibration','output'):
        ap.add_argument('--'+name,type=Path,required=True)
    alignment=ap.add_mutually_exclusive_group(required=True)
    alignment.add_argument('--assume-sensor-aligned',action='store_true')
    alignment.add_argument('--imu-mount-candidate',type=Path,
                           help='Apply a file-only unverified proper sensor-to-body rotation candidate')
    ap.add_argument('--allow-incomplete-capture',action='store_true',help='Use only audited completed prefix before first CAN timeout; no motor output')
    args=ap.parse_args(argv)
    output=args.output.expanduser().resolve()
    if any((p/'.git').exists() for p in (output,*output.parents)):
        ap.error('Use a new private output directory outside Git')
    if output.exists():ap.error('Output must be new')
    records,capture_source=load_capture(args.capture,allow_incomplete_capture=args.allow_incomplete_capture)
    calibration=_json(args.calibration.read_text())
    samples=build_samples(records,calibration,assume_sensor_aligned=args.assume_sensor_aligned,
                          imu_mount_candidate=args.imu_mount_candidate)
    policy,model_source=load_policy(args.bundle)
    results=evaluate_samples(samples,policy)
    report={'status':'OFFLINE_COLD_START_SHADOW_ONLY','motor_output_available':False,'hardware_opened':False,
        'live_50hz_verified':False,'approved_for_runtime':False,'sensor_alignment_verified':False,
        'raw_driver_axes_verified':False,'imu_mount':samples[0]['imu_mount'],
        'gyro_bias_verified':False,'gravity_fusion_verified':False,'exposure_h_available':False,
        'calibration_verified':False,'power_cycle_continuity_verified':False,'command':[0.,0.,0.],
        'model_can_order':CAN_ORDER,'capture':capture_source,'model':model_source,
        'calibration_sha256':sha(args.calibration),'runner_sha256':sha(__file__),
        'frame_count':len(results),'max_can_skew_s':max(r['can_skew_s'] for r in results),
        'complete_frames_available':results[0]['complete_frames_available'],
        'discarded_incomplete_tail_position_velocity_records':results[0]['discarded_incomplete_tail_position_velocity_records'],
        'max_imu_age_s':max(r['imu_age_s'] for r in results),
        'max_target_current_gap_rad':max(s['max_target_current_gap_rad'] for r in results for s in r['scenarios']),
        'out_of_model_range_ids':sorted({v['motor_id'] for r in results for v in r['violations']}),
        'identity_hypothesis_projects_gravity_upward':(any(r['identity_hypothesis_projected_gravity_z_positive'] for r in results)
            if args.assume_sensor_aligned else None),
        'mount_hypothesis_projects_gravity_upward':any(r['projected_gravity_z_positive'] for r in results),
        'mount_warning':('Positive projected gravity Z under identity is inconsistent with a level upright body; physical sensor orientation is unverified. No rotation was inferred or applied.'
            if args.assume_sensor_aligned else 'An explicit mounting candidate was applied to acceleration and angular velocity. Raw IC/printed-board axes and rotation signs remain unverified; no bias or scale correction was applied.'),
        'warning':'Targets start near learned nominal, not current pose; never send these diagnostics to motors.',
        'frames':results}
    output.mkdir(mode=0o700,parents=True,exist_ok=False)
    (output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('frames','model','capture')},indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
