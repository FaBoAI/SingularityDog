"""Pinned ctypes bridge for a CPU-only snapshot parser; no transport access."""
import ctypes as C
import hashlib
import math
from pathlib import Path

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import dual_can_pipeline_benchmark as dual


class Motor(C.Structure):
    _fields_ = [('motor_id',C.c_uint32),('parameter',C.c_uint32),('value',C.c_double),
                ('request_ns',C.c_uint64),('received_ns',C.c_uint64),
                ('age_upper_bound_ns',C.c_uint64)]


class Result(C.Structure):
    _fields_ = [('motors',Motor*24),('count',C.c_uint32),('composite',C.c_uint32),
                ('oldest',C.c_uint64),('latest',C.c_uint64),
                ('earliest_receive',C.c_uint64)]


ERRORS = {
    1:'Incomplete/noncausal native input',
    2:'Malformed native record',
    3:'Cross-bus input',
    4:'Invalid STOP composite response',
    5:'Invalid Type17 request',
    6:'Reply does not match request',
    7:'Rejected Type17 value',
    8:'Identity is not cycle telemetry',
    9:'Duplicate input',
    10:'Missing full twelve-axis position/velocity inputs',
}
LIMIT_NS=100_000_000


def load_parser(library, *, expected_sha256):
    path=Path(library)
    if not path.is_file() or path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected_sha256:
        raise ValueError('Pinned parser library changed or missing')
    lib=C.CDLL(str(path.resolve()))
    lib.sd_snapshot_abi.argtypes=[]
    lib.sd_snapshot_abi.restype=C.c_uint32
    if lib.sd_snapshot_abi()!=1 or C.sizeof(native.Record)!=88 or C.sizeof(Result)!=992:
        raise ValueError('Snapshot parser ABI differs')
    lib.sd_snapshot_parse.argtypes=[C.POINTER(native.Record),C.c_uint32,C.c_uint32,
        C.POINTER(native.Record),C.c_uint32,C.c_uint32,C.c_uint64,C.POINTER(Result)]
    lib.sd_snapshot_parse.restype=C.c_int
    return lib


def _record_pointer(records):
    if isinstance(records,C.Array) and type(records)._type_ is native.Record:
        return C.cast(records,C.POINTER(native.Record)),len(records),records
    if isinstance(records,(list,tuple)) and all(type(r) is native.Record for r in records):
        owned=(native.Record*len(records))(*records)
        return C.cast(owned,C.POINTER(native.Record)),len(records),owned
    return None


def snapshot_from_records_native(parser,records_by_bus,sample,tick_ns):
    """Exactly the original snapshot schema; fixed-record work goes to C++.

    Unsupported Python inputs fall back to the reference path to preserve its
    exceptions. Normal native Record arrays and copied trace lists use C++.
    """
    def reference():
        from singularitydog_hw.native_pipeline_benchmark import snapshot_from_records
        return snapshot_from_records(records_by_bus,sample,tick_ns)

    if type(tick_ns) is not int or tick_ns<0 or tick_ns>2**64-1:
        return reference()
    try:
        items=list(records_by_bus.items())
    except AttributeError:
        return reference()
    for scope,_ in items:
        if scope not in dual.SCOPES:
            raise ValueError('Unknown bus')
    if len(items)>2:
        return reference()
    pointer=[]
    owners=[]
    for scope,records in items:
        adapted=_record_pointer(records)
        if adapted is None:return reference()
        p,count,owner=adapted
        pointer.append((p,count,0 if scope=='front' else 1))
        owners.append(owner)
    while len(pointer)<2:
        pointer.append((C.POINTER(native.Record)(),0,0))
    result=Result()
    first,second=pointer
    code=parser.sd_snapshot_parse(first[0],first[1],first[2],second[0],second[1],second[2],
                                  tick_ns,C.byref(result))
    if code:
        raise ValueError(ERRORS.get(code,'Native snapshot parser failed'))
    motors=[]
    for m in result.motors[:result.count]:
        parameter='position' if m.parameter==0 else 'velocity'
        motors.append({'motor_id':m.motor_id,'parameter':parameter,'value':m.value,
            'unit':'rad' if m.parameter==0 else 'rad_s',
            'request_ns':m.request_ns,'received_ns':m.received_ns,
            'age_upper_bound_ns':m.age_upper_bound_ns})
    a,b=sample['read_started_monotonic_ns'],sample['read_finished_monotonic_ns']
    if not (0<a<=b<=tick_ns):raise ValueError('Noncausal IMU')
    for name in ('accel_m_s2','gyro_rad_s'):
        if len(sample[name])!=3 or not all(math.isfinite(x) for x in sample[name]):
            raise ValueError('Invalid IMU vector')
    oldest=min(result.oldest,a)
    latest=max(result.latest,b)
    earliest_receive=min(result.earliest_receive,b)
    if tick_ns-oldest>LIMIT_NS:raise ValueError('Expired inputs')
    composite=bool(result.composite)
    return {'status':'DIAGNOSTIC_READY','output_allowed':False,'blocked_reasons':[],
        'tick_ns':tick_ns,'max_age_ns':LIMIT_NS,'max_spread_ns':LIMIT_NS,'motors':motors,
        'imu':{'frame':'raw_sensor','accel_m_s2':list(sample['accel_m_s2']),
               'gyro_rad_s':list(sample['gyro_rad_s']),'read_started_ns':a,'read_finished_ns':b,
               'age_upper_bound_ns':tick_ns-a},
        'oldest_observation_age_ns':tick_ns-oldest,'acquisition_spread_ns':latest-oldest,
        'receive_spread_ns':latest-earliest_receive,
        'source_flags':{'native_diagnostic_transport':True,'sensor_type2_candidate':composite,
            'velocity_scale_verified':False,'sensor_internal_sample_time_verified':False,
            'stop_feedback_state_changing':composite,'fresh_identity_match_verified':True,
            'approved_for_runtime':False,'output_allowed':False}}
