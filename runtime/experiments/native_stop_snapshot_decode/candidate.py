"""File-only synthetic STOP snapshot decode experiment; never imports runtime.

The frozen source is parsed into isolated pure helpers. It cannot open CAN/IMU,
load a library/model, infer live sensor time or qualify any controller.
Only canonical STOP acquisition snapshots are in the candidate domain.
"""
import argparse
import ast
import ctypes as C
import gc
import os
import stat
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
import time
from types import SimpleNamespace

DEFAULT_ROOT=Path(__file__).resolve().parent/'fixtures/frozen-source'
PINS={
 'runtime/singularitydog_hw/native_pipeline_benchmark.py':'1f22e29b549e15b7860d9194cecd35864119eba78ed79cfd804a264c273efb0a',
 'runtime/singularitydog_hw/native_diagnostic_transport.py':'acf5e174aea398352f3e79d8b71d38ddfd6f024be00adef257a4bfa8a722b228',
 'runtime/singularitydog_hw/can_readonly.py':'a8a9a2241d9e32cb9ffad6fc855bb3a7d05b32e4e1fd4b2605cca7be0b74f9cc',
 'runtime/singularitydog_hw/dual_can_pipeline_benchmark.py':'b31307264ec7e4986520f2d6e0701238d38c627d26222401aa1507c716a0ca3d',
 }

_DIRECT_STOP_BRANCH = r'''            tx_wire,rx_wire=bytes(r.tx),bytes(r.rx)
            mid=_STOP_IDS.get(tx_wire)
            if mid is None:
                # Invalid/non-STOP input is not a measured fast path. Retain
                # original framing and bus/error checks without accepting it.
                tx,rx=_native_record_frame(tx_wire),_native_record_frame(rx_wire)
                if tx.destination not in dual.SCOPES[scope]:raise ValueError('Cross-bus input')
                if tx.kind==4:raise ValueError('Invalid STOP composite response')
                if tx.kind==17:raise ValueError('Candidate requires exact canonical STOP requests; Type17 unsupported')
                raise ValueError('Identity is not cycle telemetry')
            if rx_wire[:2]!=b'AT' or rx_wire[6]!=8 or rx_wire[15:]!=b'\r\n':
                raise ValueError('Malformed native record')
            if mid not in dual.SCOPES[scope]:raise ValueError('Cross-bus input')
            composite=True
            if (rx_wire[:7]!=_STOP_REPLY_HEADERS[mid] or
                    rx_wire[7:10]==b'\x00\xc4\x56'):
                raise ValueError('Invalid STOP composite response')
            p,v,_,_=struct.unpack('>4H',rx_wire[7:15])
            pairs=(('position',p*(2.*12.57)/65535.-12.57),('velocity',v*100./65535.-50.))
'''


def _source_bytes(root,name):
    path=Path(root)/name
    if not path.is_absolute() or '..' in path.parts or any(p.is_symlink() for p in (path,*path.parents)):
        raise ValueError('Absolute regular source paths without symlinks required')
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    with os.fdopen(fd,'rb') as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size>1024*1024:
            raise ValueError('Bounded regular source required')
        raw=stream.read(1024*1024+1)
    if len(raw)>1024*1024 or hashlib.sha256(raw).hexdigest()!=PINS[name]:
        raise ValueError('Frozen source changed: '+name)
    return raw


def verify_sources(source_root=None):
    root=DEFAULT_ROOT if source_root is None else Path(source_root)
    for name in PINS:_source_bytes(root,name)
    return dict(PINS)


def _select(source,names):
    tree=ast.parse(source)
    selected=[]
    for node in tree.body:
        if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name in names:selected.append(node)
        elif isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id in names for t in node.targets):selected.append(node)
    if len(selected)!=len(names):raise ValueError('Isolated source members absent')
    return ast.Module(body=selected,type_ignores=[])


def load_engine(source_root=None):
    root=DEFAULT_ROOT if source_root is None else Path(source_root)
    verify_sources(root)
    codec_source=_source_bytes(root,'runtime/singularitydog_hw/can_readonly.py').decode()
    codec_ns={'dataclass':dataclass,'math':math,'struct':struct,'__name__':__name__}
    exec(compile(_select(codec_source,{'HOST_ID','PARAMETERS','Frame','read_request','matches','decode_reply'}),'<pinned pure codec>','exec'),codec_ns)
    codec=SimpleNamespace(**codec_ns)
    native_source=_source_bytes(root,'runtime/singularitydog_hw/native_diagnostic_transport.py').decode()
    native_ns={'C':C,'read_request':codec.read_request}
    exec(compile(_select(native_source,{'Record','stop_wire'}),'<pinned Record/STOP encoder>','exec'),native_ns)
    dual_ns={}
    exec(compile(_select(_source_bytes(root,'runtime/singularitydog_hw/dual_can_pipeline_benchmark.py').decode(),{'SCOPES'}),'<pinned scopes>','exec'),dual_ns)
    source=_source_bytes(root,'runtime/singularitydog_hw/native_pipeline_benchmark.py').decode()
    ns={'C':C,'math':math,'struct':struct,'codec':codec,'native':SimpleNamespace(**native_ns),'dual':SimpleNamespace(**dual_ns)}
    names={'LIMIT_NS','_READ_WIRES','_STOP_WIRES','_STOP_REPLY_HEADERS','_native_record_frame','snapshot_from_records'}
    exec(compile(_select(source,names),'<pinned pure snapshot>','exec'),ns)
    function=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='snapshot_from_records')
    text=ast.get_source_segment(source,function)
    start=text.index('            tx,rx=_native_record_frame(')
    end=text.index('            for parameter,value in pairs:',start)
    direct=_DIRECT_STOP_BRANCH
    text=text[:start]+direct+text[end:]
    text=text.replace('def snapshot_from_records(','def direct_stop_snapshot(',1)
    ns['_STOP_IDS']={wire:mid for mid,wire in ns['_STOP_WIRES'].items()}
    ns['source_root']=root
    exec(compile(text,'<experimental direct STOP snapshot>','exec'),ns)
    return SimpleNamespace(**ns)


def synthetic_input(engine):
    """Synthetic bytes/times only; no capture/physical measurement claim."""
    owned={};tick=1_000_000_000
    for scope,ids in engine.dual.SCOPES.items():
        rows=(engine.native.Record*6)()
        for index,mid in enumerate(ids):
            row=rows[index];tx=engine._STOP_WIRES[mid]
            rx=engine._STOP_REPLY_HEADERS[mid]+struct.pack('>4H',28000+mid,32000+mid,30000,230)+b'\r\n'
            row.tx[:]=tx;row.rx[:]=rx;row.written=row.received=17
            row.start_ns=tick-12_000_000+index*870_000;row.finish_ns=row.start_ns+20_000
            row.read_start_ns=row.start_ns+500_000;row.received_ns=row.read_start_ns+8_000
            row.deadline_ns=tick+10_000_000
        owned[scope]=rows
    sample={'read_started_monotonic_ns':tick-8_000_000,'read_finished_monotonic_ns':tick-7_500_000,
            'accel_m_s2':[0.1,-0.2,9.8],'gyro_rad_s':[0.001,0.002,-0.003]}
    return owned,sample,tick


def profile_synthetic(engine,iterations):
    if type(iterations) is not int or not 1<=iterations<=10000:raise ValueError('Bounded iteration count required')
    owned,sample,tick=synthetic_input(engine)
    baseline,candidate=engine.snapshot_from_records,engine.direct_stop_snapshot
    a=baseline(owned,sample,tick);b=candidate(owned,sample,tick)
    if a!=b:raise ValueError('Snapshot parity failed')
    gc_before={'enabled':gc.isenabled(),'stats':gc.get_stats()}
    results=[]
    for block,order in enumerate((('baseline','candidate'),('candidate','baseline')),1):
        for name in order:
            function=baseline if name=='baseline' else candidate
            for _ in range(20):function(owned,sample,tick)
            wall=[];cpu=[]
            for _ in range(iterations):
                start=time.perf_counter_ns();cstart=time.thread_time_ns()
                function(owned,sample,tick)
                cpu.append(time.thread_time_ns()-cstart);wall.append(time.perf_counter_ns()-start)
            results.append({'block':block,'method':name,'wall_ns':wall,'thread_cpu_ns':cpu})
    verify_sources(engine.source_root)
    gc_after={'enabled':gc.isenabled(),'stats':gc.get_stats()}
    return {'schema':'singularitydog.private-offline-decode-experiment.v1','status':'SYNTHETIC_FILE_ONLY',
            'source_sha256':dict(PINS),'gc_before':gc_before,'gc_after':gc_after,'gc_scope_changed':False,'raw_samples':results,'iterations_per_case':iterations,
            'snapshot_parity':True,'candidate_domain':'canonical STOP acquisition only; Type17 not supported',
            'excludes':['native exchange/device waits','IMU device read','scheduler/GIL contention with actual runtime','model/observer/final output validation'],
            'complete_snapshot_created':True,'timestamps_unchanged':True,'live_performance_verified':False,
            'active_output_eligible':False,'formal_calibration_approved':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--source-root',type=Path,default=DEFAULT_ROOT,help='Repo or frozen kit root containing the exact pinned Python sources')
    parser.add_argument('--profile-synthetic',action='store_true')
    parser.add_argument('--iterations',type=int,default=1000)
    args=parser.parse_args()
    if not args.profile_synthetic:
        print(json.dumps({'status':'PLAN_FILE_ONLY','source_sha256':verify_sources(args.source_root),'active_output_eligible':False,'native_or_model_load_available':False}))
        return
    print(json.dumps(profile_synthetic(load_engine(args.source_root),args.iterations),allow_nan=False))

if __name__=='__main__':main()
