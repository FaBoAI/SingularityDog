"""Saved Type17/STOP exact comparison, rejection audit, and paired Mac timing."""
import argparse
import copy
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct
import time

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_pipeline_benchmark as bench
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import dual_can_pipeline_benchmark as dual
from .loader import load_parser,snapshot_from_records_native


def sha(raw):return hashlib.sha256(raw).hexdigest()


def pinned_json(path,digest):
    path=Path(path)
    if not path.is_file() or path.is_symlink() or sha(path.read_bytes())!=digest:
        raise ValueError('Saved records changed or missing')
    return json.loads(path.read_bytes())


def restore(row,*,saved_tick=True):
    acquired={}
    for scope,value in row['acquired'].items():
        saved=value['records']
        records=(native.Record*len(saved))()
        for record,source in zip(records,saved):
            for key in ('start_ns','finish_ns','read_start_ns','received_ns','deadline_ns',
                        'written','received'):
                setattr(record,key,source[key])
            record.tx[:]=bytes.fromhex(source['tx_hex'])
            record.rx[:]=bytes.fromhex(source['rx_hex'])
        acquired[scope]=records
    sample=copy.deepcopy(row['imu'])
    tick=(row['observed']['tick_ns'] if saved_tick else
          max(sample['read_finished_monotonic_ns'],
              *(record.received_ns for records in acquired.values() for record in records))+1000)
    return acquired,sample,tick


def clone(acquired):
    copied={}
    for scope,records in acquired.items():
        duplicate=(native.Record*len(records))()
        C.memmove(C.addressof(duplicate),C.addressof(records),C.sizeof(records))
        copied[scope]=duplicate
    return copied


def outcomes(parser,acquired,sample,tick):
    results=[]
    for fn in (bench.snapshot_from_records,
               lambda a,b,c:snapshot_from_records_native(parser,a,b,c)):
        try:results.append(('accepted',fn(acquired,sample,tick)))
        except Exception as error:results.append((type(error).__name__,str(error)))
    return results


def assert_same(parser,acquired,sample,tick,label,*,reject=False):
    left,right=outcomes(parser,acquired,sample,tick)
    if left!=right or (reject and left[0]=='accepted'):
        raise AssertionError(label+': '+repr((left,right))[:1000])
    return left


def rejection_cases(stop,type17):
    cases=[]
    def add(name,base,mutator):
        acquired,sample,tick=base
        acquired,sample=clone(acquired),copy.deepcopy(sample)
        tick=mutator(acquired,sample,tick)
        cases.append((name,acquired,sample,tick))
    add('stop_written_short',stop,lambda a,s,t:(setattr(a['front'][0],'written',16),t)[1])
    add('stop_received_short',stop,lambda a,s,t:(setattr(a['front'][0],'received',16),t)[1])
    add('zero_start',stop,lambda a,s,t:(setattr(a['front'][0],'start_ns',0),t)[1])
    add('finish_before_start',stop,lambda a,s,t:(setattr(a['front'][0],'finish_ns',a['front'][0].start_ns-1),t)[1])
    add('receive_before_finish',stop,lambda a,s,t:(setattr(a['front'][0],'received_ns',a['front'][0].finish_ns-1),t)[1])
    add('receive_at_deadline',stop,lambda a,s,t:(setattr(a['front'][0],'deadline_ns',a['front'][0].received_ns),t)[1])
    add('tick_before_reply',stop,lambda a,s,t:a['front'][0].received_ns-1)
    for wire in ('tx','rx'):
        for offset,value in ((0,ord('X')),(6,7),(15,ord('X'))):
            add('malformed_'+wire+'_'+str(offset),stop,
                lambda a,s,t,wire=wire,offset=offset,value=value:
                (getattr(a['front'][0],wire).__setitem__(offset,value),t)[1])
    add('cross_bus',stop,lambda a,s,t:(a.update(front=a['rear'],rear=a['front']),t)[1])
    add('unknown_bus',stop,lambda a,s,t:(a.__setitem__('other',a.pop('front')),t)[1])
    add('stop_request_payload',stop,lambda a,s,t:(a['front'][0].tx.__setitem__(7,1),t)[1])
    add('stop_response_flags',stop,lambda a,s,t:(a['front'][0].rx.__setitem__(5,a['front'][0].rx[5]^1),t)[1])
    add('stop_forbidden_payload',stop,lambda a,s,t:(a['front'][0].rx.__setitem__(slice(7,10),b'\x00\xc4\x56'),t)[1])
    add('type17_request_payload',type17,lambda a,s,t:(a['front'][0].tx.__setitem__(9,1),t)[1])
    add('type17_reply_index',type17,lambda a,s,t:(a['front'][0].rx.__setitem__(7,0x1b),t)[1])
    add('type17_reserved',type17,lambda a,s,t:(a['front'][0].rx.__setitem__(9,1),t)[1])
    def status(a,s,t):
        r=a['front'][0];encoded=int.from_bytes(bytes(r.rx[2:6]),'big')
        can_id=(encoded>>3)|(1<<16)
        r.rx[2:6]=((can_id<<3)|(encoded&7)).to_bytes(4,'big')
        return t
    add('type17_status',type17,status)
    add('type17_nan',type17,lambda a,s,t:(a['front'][0].rx.__setitem__(slice(11,15),struct.pack('<f',float('nan'))),t)[1])
    def identity(a,s,t):
        r=a['front'][0];encoded=int.from_bytes(bytes(r.tx[2:6]),'big')
        can_id=(encoded>>3)&~(31<<24)
        r.tx[2:6]=((can_id<<3)|(encoded&7)).to_bytes(4,'big')
        return t
    add('identity_not_telemetry',type17,identity)
    add('duplicate_stop',stop,lambda a,s,t:(C.memmove(C.addressof(a['front'][1]),C.addressof(a['front'][0]),C.sizeof(native.Record)),t)[1])
    add('duplicate_type17',type17,lambda a,s,t:(C.memmove(C.addressof(a['front'][1]),C.addressof(a['front'][0]),C.sizeof(native.Record)),t)[1])
    add('missing_bus',stop,lambda a,s,t:(a.pop('rear'),t)[1])
    add('imu_zero_start',stop,lambda a,s,t:(s.__setitem__('read_started_monotonic_ns',0),t)[1])
    add('imu_nan',stop,lambda a,s,t:(s['gyro_rad_s'].__setitem__(0,float('nan')),t)[1])
    add('expired',stop,lambda a,s,t:min(r.start_ns for rows in a.values() for r in rows)+100_000_001)
    return cases


def byte_mutation_sweep(parser,stop,type17):
    checked=0
    for label,(base,sample,tick) in (('STOP',stop),('Type17',type17)):
        for wire in ('tx','rx'):
            for offset in range(17):
                original=getattr(base['front'][0],wire)[offset]
                for value in range(256):
                    if value==original:continue
                    acquired=clone(base)
                    getattr(acquired['front'][0],wire)[offset]=value
                    assert_same(parser,acquired,sample,tick,
                                label+' '+wire+' '+str(offset)+' '+str(value))
                    checked+=1
    return checked


def distribution(values):
    values=sorted(values);n=len(values)
    return {'count':n,'median_ms':statistics.median(values)/1e6,
            'p95_ms':values[math.ceil(.95*n)-1]/1e6,
            'p99_ms':values[math.ceil(.99*n)-1]/1e6,
            'max_ms':values[-1]/1e6}


def benchmark(parser,rows):
    # Full snapshot assembly is inside each measurement. Alternating order
    # reduces drift; saved objects are built before the timed region.
    wall=[[],[]];cpu=[[],[]]
    functions=(bench.snapshot_from_records,
               lambda a,b,c:snapshot_from_records_native(parser,a,b,c))
    for args in rows[:10]:
        for fn in functions:fn(*args)
    for index,args in enumerate(rows):
        result=[None,None]
        for offset in range(2):
            slot=(index+offset)%2
            w=time.perf_counter_ns();c=time.thread_time_ns()
            result[slot]=functions[slot](*args)
            cpu[slot].append(time.thread_time_ns()-c)
            wall[slot].append(time.perf_counter_ns()-w)
        if result[0]!=result[1]:raise AssertionError('Timed snapshot mismatch')
    return {name:{'wall':distribution(wall[i]),'thread_cpu':distribution(cpu[i])}
            for i,name in enumerate(('python','native'))}


def run(args):
    stop=pinned_json(args.stop_records,args.stop_sha)
    type17=pinned_json(args.type17_records,args.type17_sha)
    if type(stop) is not list or len(stop)!=500 or type(type17) is not list or not type17:
        raise ValueError('Expected 500 STOP and nonempty Type17 saved rows')
    parser=load_parser(args.library,expected_sha256=args.library_sha)
    stop_rows=[restore(row) for row in stop]
    type17_rows=[restore(row,saved_tick=False) for row in type17]
    for index,row in enumerate(stop_rows):assert_same(parser,*row,'STOP '+str(index))
    for index,row in enumerate(type17_rows):assert_same(parser,*row,'Type17 '+str(index))
    rejected=[]
    for name,acquired,sample,tick in rejection_cases(stop_rows[0],type17_rows[0]):
        outcome=assert_same(parser,acquired,sample,tick,name,reject=True)
        rejected.append({'case':name,'exception':outcome[0],'reason':outcome[1]})
    byte_mutations=byte_mutation_sweep(parser,stop_rows[0],type17_rows[0])
    report={'schema':'native-snapshot-fastpath-validation-v1','status':'PASS_EXACT_FILE_ONLY',
            'hardware_opened':False,'output_allowed':False,'approved_for_runtime':False,
            'adoption_decision':'REJECT_SMALL_ABSOLUTE_GAIN',
            'jetson_verified':False,'full_cycle_verified':False,
            'deployed_default_changed':False,
            'saved_stop_frames':len(stop_rows),'saved_type17_frames':len(type17_rows),
            'all_snapshot_values_exact':True,'rejection_count':len(rejected),
            'single_byte_mutations_exact':byte_mutations,
            'rejections':rejected,'timing_stop':benchmark(parser,stop_rows),
            'timing_type17':benchmark(parser,(type17_rows*math.ceil(500/len(type17_rows)))[:500]),
            'source_hashes':{name:sha((Path(__file__).resolve().parent/name).read_bytes())
                             for name in ('snapshot.cpp','loader.py','build.py','verify.py')},
            'reference_source_sha256':{
                'native_pipeline_benchmark.py':sha(Path(bench.__file__).read_bytes()),
                'can_readonly.py':sha(Path(codec.__file__).read_bytes()),
                'native_diagnostic_transport.py':sha(Path(native.__file__).read_bytes()),
                'dual_can_pipeline_benchmark.py':sha(Path(dual.__file__).read_bytes())},
            'stop_records_sha256':args.stop_sha,'type17_records_sha256':args.type17_sha,
            'library_sha256':args.library_sha,
            'scope':'CPU snapshot assembly from saved fixed records only; no live I/O'}
    output=Path(args.output)
    if not output.parent.is_dir() or output.exists():raise ValueError('Require new output report')
    with output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True);p.add_argument('--library-sha',required=True)
    p.add_argument('--stop-records',required=True);p.add_argument('--stop-sha',required=True)
    p.add_argument('--type17-records',required=True);p.add_argument('--type17-sha',required=True)
    p.add_argument('--output',required=True)
    result=run(p.parse_args())
    print(json.dumps({'status':result['status'],'rejections':result['rejection_count'],
                      'timing_stop':result['timing_stop'],'timing_type17':result['timing_type17']},indent=2))


if __name__=='__main__':main()
