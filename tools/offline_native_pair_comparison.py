"""Bounded local-socket comparison; never opens robot, serial, SSH or network.

The default is a file-only plan. --execute-offline builds a private copy of the
real active C++ transport and sends only STOP/voltage requests to fake QDDs.
These host measurements cannot qualify hardware, Type1 output or 20 ms control.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import select
import socket
import struct
import sys
import tempfile
import threading
import time

REPOSITORY=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPOSITORY/'runtime'))
from singularitydog_hw import native_active_transport as native
from singularitydog_hw.can_readonly import ATParser
from singularitydog_hw.native_diagnostic_transport import stop_wire, exchange_evidence

BOOT='11111111-2222-3333-4444-555555555555'
SCOPES=('front','rear')


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frame(can_id,data):
    return b'AT'+((can_id<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'


def voltage_wire(mid):
    return frame((17<<24)|(0xfd<<8)|mid,struct.pack('<H',0x701c)+bytes(6))


def fake_reply(wire):
    if (len(wire)!=17 or wire[:2]!=b'AT' or wire[-2:]!=b'\r\n' or wire[6]!=8 or wire[5]&7!=4):
        raise ValueError('Fake backend accepts exact individual AT frames only')
    can_id=int.from_bytes(wire[2:6],'big')>>3; mid=can_id&255; kind=can_id>>24
    if not 1<=mid<=12: raise ValueError('Fake QDD motor ID must be 1..12')
    if kind==4 and wire[7:15]==bytes(8):
        return frame((2<<24)|(mid<<8)|0xfd,struct.pack('>4H',32767,32767,32767,250))
    if kind==17 and wire[7:15]==struct.pack('<H',0x701c)+bytes(6):
        return frame((17<<24)|(mid<<8)|0xfd,wire[7:11]+struct.pack('<f',38.0))
    raise ValueError('Fake comparison prohibits enable, Type1 and all other commands')


class FakeBus:
    def __init__(self):
        self.host,self.peer=socket.socketpair();self.host.setblocking(False)
        self.cancel_read,self.cancel_write=os.pipe();self.done=threading.Event()
        self.requests=[];self.error=None
        self.thread=threading.Thread(target=self._device,name='fake-qdd',daemon=True)
        self.thread.start()

    def _device(self):
        parser=ATParser()
        try:
            while not self.done.is_set():
                if not select.select([self.peer],[],[],.05)[0]:continue
                raw=self.peer.recv(4096)
                if not raw:return
                for item in parser.feed(raw):
                    self.requests.append(item.wire.hex());self.peer.sendall(fake_reply(item.wire))
        except OSError:
            if not self.done.is_set():self.error='Fake socket failed before cleanup'
        except BaseException as error:self.error=repr(error)

    def close(self):
        self.done.set();self.host.close();self.peer.close();self.thread.join(timeout=.5)
        os.close(self.cancel_read);os.close(self.cancel_write)
        if self.thread.is_alive():raise RuntimeError('Fake device thread did not join')


def percentile(values,fraction):
    if not values:return None
    ordered=sorted(values);index=(len(ordered)-1)*fraction
    low=int(index);high=min(low+1,len(ordered)-1)
    return ordered[low]+(ordered[high]-ordered[low])*(index-low)


def summary(rows,requested_cycles):
    complete=[row for row in rows if row['status']=='COMPLETE_FAKE_CYCLE']
    latencies=[row['elapsed_ns']/1e6 for row in complete]
    offsets=[]
    for row in rows:
        for phase in row['phases']:
            if phase.get('native_pair'):
                starts=phase['native_pair']['owner_started_ns']
                if all(starts):offsets.append(abs(starts[1]-starts[0])/1000.)
    return {'requested_cycles':requested_cycles,'attempted_cycles':len(rows),'complete_cycles':len(complete),
        'p50_ms':percentile(latencies,.5),'p99_ms':percentile(latencies,.99),
        'max_ms':max(latencies,default=None),
        'complete_host_cycles_over_20ms':sum(value>20 for value in latencies),
        'incomplete_cycles':len(rows)-len(complete),
        'native_owner_start_delta_us':{'p50':percentile(offsets,.5),'p99':percentile(offsets,.99),
                                      'max':max(offsets,default=None)}}


def run_case(library,mode,cycles):
    buses={scope:FakeBus() for scope in SCOPES};sessions={};pair=None;rows=[]
    with tempfile.TemporaryFile() as boot:
        boot.write((BOOT+'\n').encode());boot.flush()
        pools={scope:ThreadPoolExecutor(max_workers=1,thread_name_prefix='fake-legacy-'+scope)
               for scope in SCOPES}
        try:
            for scope,first in (('front',1),('rear',7)):
                ids=range(first,first+6);bus=buses[scope]
                sessions[scope]=native.ActiveSession(library,bus.host.fileno(),first_id=first,
                    cancel_fd=bus.cancel_read,boot_fd=boot.fileno(),boot_id=BOOT,
                    raw_lower_by_id={mid:-1. for mid in ids},raw_upper_by_id={mid:1. for mid in ids},
                    kp_max_by_id={mid:3. for mid in ids},kd_max_by_id={mid:.15 for mid in ids},
                    gap_ns=900_000,window=3)
            if mode!='legacy_two_pools':pair=native.ActivePhasePair(sessions['front'],sessions['rear'])
            for index in range(cycles):
                begin=time.monotonic_ns();deadline=begin+20_000_000
                row={'cycle':index+1,'begin_ns':begin,'deadline_ns':deadline,'phases':[],
                     'status':'INCOMPLETE_FAKE_CYCLE','error':None}
                failed=False
                for label in ('acquisition_stop','voltage','output_stop_proxy'):
                    batches={scope:([voltage_wire((1 if scope=='front' else 7)+index%6)]
                        if label=='voltage' else [stop_wire(mid) for mid in
                            range(1 if scope=='front' else 7,7 if scope=='front' else 13)])
                        for scope in SCOPES}
                    use_pair=pair is not None and (mode=='native_pair_all_ordinary_phases' or
                                                   label=='output_stop_proxy')
                    phase={'label':label,'begin_ns':time.monotonic_ns(),'buses':{},'native_pair':None}
                    try:
                        if use_pair:futures=pair.submit(batches,deadline_ns=deadline)
                        else:futures={scope:pools[scope].submit(sessions[scope].exchange,batches[scope],
                                                               deadline_ns=deadline) for scope in SCOPES}
                        errors=[]
                        for scope,future in futures.items():
                            try:
                                records,stats=future.result(timeout=.35)
                                phase['buses'][scope]={'status':'COMPLETE_FAKE_EXCHANGE',
                                    'raw':exchange_evidence(records,stats),
                                    'rejected_total_bytes':int(stats.rejected_total)}
                            except BaseException as error:
                                errors.append(type(error).__name__+': '+str(error))
                                phase['buses'][scope]={'status':'INCOMPLETE_FAKE_EXCHANGE','error':repr(error)}
                                if hasattr(error,'records'):
                                    phase['buses'][scope]['raw']=exchange_evidence(error.records,error.stats)
                        if use_pair:phase['native_pair']=pair.last_phase
                        if errors:raise RuntimeError('; '.join(errors))
                    except BaseException as error:
                        row['error']=type(error).__name__+': '+str(error);failed=True
                    phase['end_ns']=time.monotonic_ns();row['phases'].append(phase)
                    if failed:break
                row['end_ns']=time.monotonic_ns();row['elapsed_ns']=row['end_ns']-begin
                if not failed:row['status']='COMPLETE_FAKE_CYCLE'
                rows.append(row)
                if failed:break # Poisoned sessions never resume or retransmit.
            return {'mode':mode,'summary':summary(rows,cycles),'cycles':rows,
                    'fake_received_requests':{scope:len(bus.requests) for scope,bus in buses.items()},
                    'fake_device_errors':{scope:bus.error for scope,bus in buses.items()}}
        finally:
            if pair is not None:pair.close()
            for pool in pools.values():pool.shutdown(wait=True,cancel_futures=False)
            for session in sessions.values():session.close()
            for bus in buses.values():bus.close()


def plan(cycles):
    if type(cycles)is not int or not 1<=cycles<=500:raise ValueError('Offline cycles must be 1..500')
    source=REPOSITORY/'runtime/experiments/native_active_transport/transport.cpp'
    return {'schema':'singularitydog.offline-native-pair-comparison.v1','status':'PLAN_ONLY',
        'cycles_per_case':cycles,'cases':['legacy_two_pools','native_pair_output_only',
                                        'native_pair_all_ordinary_phases'],
        'request_gap_ns':900_000,'request_window':3,'requests_per_cycle':26,
        'phase_contract':'ordinary acquisition 12 -> voltage 2 -> STOP proxy 12',
        'prepared_feedback_voltage_overlap_emulated':False,'period_ns':20_000_000,
        'hardware_opened':False,'network_used':False,'motor_enable_sent':False,
        'learned_targets_sent':False,'hardware_timing_qualified':False,
        'source_sha256':digest(source),'wrapper_sha256':digest(REPOSITORY/'runtime/singularitydog_hw/native_active_transport.py'),
        'tool_sha256':digest(Path(__file__)),
        'interpretation':'Fast synthetic replies measure this host implementation only; no Jetson or robot 20ms claim.'}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycles',type=int,default=20)
    parser.add_argument('--execute-offline',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv);report=plan(args.cycles)
    if not args.execute_offline:
        print(json.dumps(report,ensure_ascii=False));return 0
    if args.output is None:parser.error('--execute-offline requires a fresh private --output directory')
    output=args.output.expanduser().resolve()
    if not output.is_absolute() or output.exists() or any((parent/'.git').exists() for parent in (output,*output.parents)):
        parser.error('Fresh private directory outside Git required')
    output.mkdir(parents=True,mode=0o700)
    frozen=output/'used-sources';frozen.mkdir(mode=0o700)
    source_paths={'transport.cpp':REPOSITORY/'runtime/experiments/native_active_transport/transport.cpp',
        'build.py':REPOSITORY/'runtime/experiments/native_active_transport/build.py',
        'native_active_transport.py':REPOSITORY/'runtime/singularitydog_hw/native_active_transport.py',
        'offline_native_pair_comparison.py':Path(__file__)}
    source_pins={}
    for name,path in source_paths.items():
        (frozen/name).write_bytes(path.read_bytes());source_pins[name]=digest(frozen/name)
    if (source_pins['transport.cpp']!=report['source_sha256'] or
        source_pins['native_active_transport.py']!=report['wrapper_sha256'] or
        source_pins['offline_native_pair_comparison.py']!=report['tool_sha256']):
        raise RuntimeError('Sources changed while creating the offline comparison receipt')
    report['used_source_sha256']=source_pins
    with tempfile.TemporaryDirectory(prefix='dog-offline-pair-build-') as temporary:
        stage=Path(temporary)
        for name in ('transport.cpp','build.py'):(stage/name).write_bytes((frozen/name).read_bytes())
        spec=importlib.util.spec_from_file_location('private_offline_pair_build',stage/'build.py')
        builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
        library_path=builder.build();library=native.load_library(library_path,expected_sha256=digest(library_path))
        report['native_build']=json.loads((stage/'build-record.json').read_text())
        # Private build command paths are kept only in this private report.
        results=[run_case(library,mode,args.cycles) for mode in report['cases']]
    raw_path=output/'raw-phases.json'
    raw_path.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    report.update(status='OFFLINE_COMPARISON_RECORDED',results=[{'mode':row['mode'],**row['summary']} for row in results],
                  raw_sha256=digest(raw_path),raw_bytes=raw_path.stat().st_size,
                  sources_unchanged_after_comparison=all(digest(source_paths[name])==pin for name,pin in source_pins.items()))
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'output':str(output),'results':report['results'],
                      'hardware_timing_qualified':False},ensure_ascii=False));return 0


if __name__=='__main__':raise SystemExit(main())
