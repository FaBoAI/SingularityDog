"""Explicit file/pipe-only prototype; default CLI is a no-load PLAN.

Fake packet stages are feedback, voltage, STOP_ONLY; they are NOT CAN messages.
No sd_exchange, motor API, model, boot device or runtime selector is imported.
"""
import argparse
import ctypes as C
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import struct
import subprocess
import threading
import time

HERE=Path(__file__).resolve().parent
CPP=HERE/'phase_owner.cpp'
BUDGET_NS=20_000_000
GAP_NS=900_000
WAIT_SLICE_NS=200_000
_QUERY=b'SDFAKEQ1';_REPLY=b'SDFKREP1'
_PACKET=struct.Struct('=8sQIIQ')


def require(value,message):
    if not value:raise ValueError(message)


def digest(value):
    require(type(value)is str and re.fullmatch('[0-9a-f]{64}',value),'Explicit SHA256 required')
    return value


def read(path,pin=None):
    if pin is not None:digest(pin)
    path=Path(path);require(path.is_absolute()and not any(p.is_symlink()for p in (path,*path.parents)),'Absolute non-symlink input required')
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|os.O_NOFOLLOW)
    try:
        before=os.fstat(fd);require(stat.S_ISREG(before.st_mode)and before.st_size<=64*1024*1024,'Bounded regular file required')
        with os.fdopen(fd,'rb',closefd=False)as stream:raw=stream.read(64*1024*1024+1)
        after=os.fstat(fd);require(len(raw)<=64*1024*1024 and (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)==(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns),'Input changed while reading')
    finally:os.close(fd)
    if pin is not None:require(hashlib.sha256(raw).hexdigest()==pin,'Pinned bytes differ')
    return raw


def fresh(path):
    path=Path(path);require(path.is_absolute()and path.parent.is_dir(),'Existing absolute parent required')
    require(not path.exists()and not path.is_symlink()and not any(p.is_symlink()for p in path.parents),'Fresh non-symlink output required')
    require(not any((p/'.git').exists()for p in (path.parent,*path.parents)),'Output outside Git required')
    return path


def build(output):
    output=fresh(output);source=read(CPP);compiler=shutil.which('c++');require(compiler is not None,'C++ compiler required')
    command=[compiler,'-std=c++20','-O2','-pthread','-fPIC','-shared',str(CPP),'-o',str(output)]
    env=dict(os.environ);env.pop('LD_PRELOAD',None)
    run=subprocess.run(command,capture_output=True,text=True,timeout=30,env=env)
    require(run.returncode==0,'Fake prototype compilation failed: '+run.stderr[-2000:])
    require(read(CPP)==source,'Source changed during compile')
    return {'schema':'private.fake-phase-owner-build.v1','source_sha256':hashlib.sha256(source).hexdigest(),
        'library_path':str(output),'library_sha256':hashlib.sha256(read(output)).hexdigest(),'command':command,
        'fake_backend_only':True,'sd_exchange_linked':False,'hardware_opened':False,'output_allowed':False}


class Record(C.Structure):
    _fields_=[(name,C.c_uint64)for name in ('generation','start_ns','finish_ns','deadline_ns','read_start_ns','received_ns','prior_finish_ns','nonce')]+[(name,C.c_uint32)for name in ('bus','phase','written','received','status','reserved')]+[('tx',C.c_ubyte*32),('rx',C.c_ubyte*32)]


class Pair(C.Structure):
    _fields_=[('generation',C.c_uint64),('phase',C.c_uint32),('reserved',C.c_uint32),('records',Record*2)]


class Status(C.Structure):
    _fields_=[(name,C.c_uint32)for name in ('abi','state','error_code','error_bus','error_phase','cancelled','stop_admitted','exited_mask','joined_mask','owned_closed_mask','ready_mask','consumed_mask')]+[(name,C.c_uint64)for name in ('generation','deadline_ns','notifications')]+[('last_finish_ns',C.c_uint64*2),('error',C.c_char*160),('records',Record*6)]


def exact_int(value,low,high,name):
    require(type(value)is int and low<=value<=high,'Exact bounded integer '+name+' required');return value


def validate_pair(pair):
    require(type(pair)is Pair and pair.generation>0 and 1<=pair.phase<=3 and pair.reserved==0,'Complete fake pair required')
    for bus,row in enumerate(pair.records):
        require(row.generation==pair.generation and row.phase==pair.phase and row.bus==bus and row.status==1 and row.written==row.received==32 and row.reserved==0,'Complete matching fake row required')
        require(0<row.start_ns<=row.read_start_ns<=row.received_ns<=row.finish_ns<row.deadline_ns,'Original fake timestamps/deadline mismatch')
        require(not row.prior_finish_ns or row.start_ns>=row.prior_finish_ns+GAP_NS,'Original fake inter-stage gap mismatch')
        require(_PACKET.unpack(bytes(row.tx))==(_QUERY,pair.generation,pair.phase,bus,row.nonce)and _PACKET.unpack(bytes(row.rx))==(_REPLY,pair.generation,pair.phase,bus,row.nonce),'Fake packet semantic validation failed')
    return True


def record_dict(row):
    values={name:int(getattr(row,name))for name,_ in Record._fields_ if name not in ('tx','rx')}
    values.update(tx_hex=bytes(row.tx).hex(),rx_hex=bytes(row.rx).hex());return values


def status_dict(status):
    values={name:int(getattr(status,name))for name,_ in Status._fields_ if name not in ('records','error','last_finish_ns')}
    values.update(error=bytes(status.error).decode(errors='replace'),last_finish_ns=list(status.last_finish_ns),records=[record_dict(row)for row in status.records]);return values


class Bindings:
    def __init__(self,library,pin,build_record):
        digest(pin);library=Path(library);raw=read(library,pin)
        require(type(build_record)is dict and build_record.get('schema')=='private.fake-phase-owner-build.v1'and build_record.get('library_path')==str(library)and build_record.get('library_sha256')==pin and build_record.get('source_sha256')==hashlib.sha256(read(CPP)).hexdigest()and build_record.get('fake_backend_only')is True and build_record.get('sd_exchange_linked')is False,'Exact fake-only build record required')
        self.library=library;self.pin=pin;self.build_record=build_record;self.lib=C.CDLL(str(library))
        for name in ('sdf_abi','sdf_record_size','sdf_pair_size','sdf_status_size'):
            f=getattr(self.lib,name);f.argtypes=[];f.restype=C.c_uint32
        require(self.lib.sdf_abi()==1 and (self.lib.sdf_record_size(),self.lib.sdf_pair_size(),self.lib.sdf_status_size())==(C.sizeof(Record),C.sizeof(Pair),C.sizeof(Status)),'Exact isolated fake ABI required')
        self.lib.sdf_now_ns.argtypes=[];self.lib.sdf_now_ns.restype=C.c_uint64
        error=[C.c_char_p,C.c_uint32];ptr=C.c_void_p;u64=C.c_uint64;u32=C.c_uint32
        signatures={'sdf_create_into':([C.c_int]*4+[C.POINTER(ptr)]+error,C.c_int),'sdf_begin':([ptr,u64,u64,u64]+error,C.c_int),
            'sdf_collect':([ptr,u64,u32,u64,C.POINTER(Pair),C.POINTER(u64)]+error,C.c_int),
            'sdf_ack':([ptr,C.POINTER(Pair)]+error,C.c_int),'sdf_submit_stop':([ptr,u64,u32]+error,C.c_int),
            'sdf_cancel':([ptr],C.c_int),'sdf_status':([ptr,C.POINTER(Status)]+error,C.c_int),
            'sdf_close':([ptr,u64,C.POINTER(Status)]+error,C.c_int)}
        for name,(args,result)in signatures.items():f=getattr(self.lib,name);f.argtypes=args;f.restype=result
        require(self.lib.sdf_collect._flags_==1,'CDECL/CDLL GIL-releasing entry required')
        require(read(library,pin)==raw,'Library changed during load')

    def verify(self):
        read(self.library,self.pin);require(self.build_record['source_sha256']==hashlib.sha256(read(CPP)).hexdigest(),'Prototype source changed')

    def now(self):return int(self.lib.sdf_now_ns())


class Owners:
    def __init__(self,bindings):
        # Construction acquires no native resource. The caller must retain this
        # object before start(), so even an interrupted create return is reaped.
        self.bindings=bindings;self.owner=threading.get_ident();self._lock=threading.Lock();self._lifetime_lock=threading.Lock();self._handle_cell=C.c_void_p();self._creation_attempted=False;self.closed_status=None;self.last_collect_ns=0

    @property
    def handle(self):return self._handle_cell.value

    @handle.setter
    def handle(self,value):self._handle_cell.value=value

    def start(self,front_tx,front_rx,rear_tx,rear_rx):
        fds=[exact_int(fd,0,2**31-1,'fd')for fd in (front_tx,front_rx,rear_tx,rear_rx)]
        require(threading.get_ident()==self.owner,'Wrong coordinator thread')
        require(self._lock.acquire(False),'Reentrant coordinator call')
        try:
            with self._lifetime_lock:
                require(not self._creation_attempted and self.handle is None,'Fake owner start is single-use')
                self._creation_attempted=True;error=C.create_string_buffer(160)
                result=self.bindings.lib.sdf_create_into(*fds,C.byref(self._handle_cell),error,len(error))
                require(result==0 and self.handle is not None,error.value.decode())
        finally:self._lock.release()

    def _call(self,name,*values):
        require(threading.get_ident()==self.owner,'Wrong coordinator thread')
        require(self.handle is not None,'Closed fake session')
        require(self._lock.acquire(False),'Reentrant coordinator call')
        try:
            error=C.create_string_buffer(160);result=getattr(self.bindings.lib,name)(self.handle,*values,error,len(error))
            require(result>=0,error.value.decode());return result
        finally:self._lock.release()

    def begin(self,generation,deadline_ns,seed=1):
        generation=exact_int(generation,1,2**64-1,'generation');deadline_ns=exact_int(deadline_ns,1,2**64-1,'deadline');seed=exact_int(seed,0,2**64-1,'seed')
        self._call('sdf_begin',generation,deadline_ns,seed)

    def collect(self,generation,phase,wait_ns=WAIT_SLICE_NS):
        self.last_collect_ns=0
        exact_int(generation,1,2**64-1,'generation');exact_int(phase,1,3,'phase');exact_int(wait_ns,0,WAIT_SLICE_NS,'wait slice')
        pair=Pair();actual=C.c_uint64()
        try:result=self._call('sdf_collect',generation,phase,wait_ns,C.byref(pair),C.byref(actual))
        finally:self.last_collect_ns=int(actual.value)
        return pair if result==1 else None

    def acknowledge(self,pair,validated=False):
        require(type(pair)is Pair and type(validated)is bool and validated,'Explicit main fake semantic validation required')
        self._call('sdf_ack',C.byref(pair))

    def submit_stop(self,generation,validated=False):
        exact_int(generation,1,2**64-1,'generation');require(type(validated)is bool and validated,'Explicit STOP_ONLY validation required')
        self._call('sdf_submit_stop',generation,1)

    def cancel(self):
        # Cancellation may run during collect, but cannot race native deletion.
        with self._lifetime_lock:
            require(self.handle is not None,'Closed fake session');require(self.bindings.lib.sdf_cancel(self.handle)==0,'Fake cancel failed')

    def status(self):
        value=Status();self._call('sdf_status',C.byref(value));return value

    def close(self,timeout_ns=100_000_000):
        exact_int(timeout_ns,1,250_000_000,'cleanup timeout')
        require(threading.get_ident()==self.owner,'Wrong coordinator thread');require(self.handle is not None,'Closed fake session');require(self._lock.acquire(False),'Reentrant coordinator call')
        try:
            status=Status();error=C.create_string_buffer(160)
            with self._lifetime_lock:
                result=self.bindings.lib.sdf_close(self.handle,self.bindings.now()+timeout_ns,C.byref(status),error,len(error))
                if result>=0:
                    self.handle=None
                    self.closed_status=status if result in (0,1) else None
            require(result==0,error.value.decode());self.bindings.verify();return status
        finally:self._lock.release()


class FakeBus:
    """Test responder on private pipes, with bounded native-thread cleanup."""
    def __init__(self,bus,transform=None):
        self.bus=bus;self.transform=transform;self.request_rx,self.request_tx=os.pipe();self.reply_rx,self.reply_tx=os.pipe()
        self.fds=(self.request_rx,self.request_tx,self.reply_rx,self.reply_tx)
        for fd in self.fds:os.set_blocking(fd,False)
        self.stop=threading.Event();self.requests=[];self.errors=[]
        self.thread=threading.Thread(target=self._run,name='fake-responder-'+str(bus));self.thread.start()

    def _run(self):
        import select
        scratch=b''
        try:
            while not self.stop.is_set():
                ready,_,_=select.select([self.request_rx],[],[],.01)
                if not ready:continue
                part=os.read(self.request_rx,32-len(scratch))
                if not part:return
                scratch+=part
                if len(scratch)!=32:continue
                packet=_PACKET.unpack(scratch);self.requests.append(packet);scratch=b''
                if self.transform:reply=self.transform(packet)
                else:reply=_PACKET.pack(_REPLY,*packet[1:])
                if reply is None:continue
                if type(reply)is bytes:reply=(reply,)
                for chunk in reply:
                    offset=0
                    while offset<len(chunk)and not self.stop.is_set():
                        try:offset+=os.write(self.reply_tx,chunk[offset:])
                        except BlockingIOError:select.select([], [self.reply_tx], [], .01)
        except OSError as error:
            if not self.stop.is_set():self.errors.append(str(error))

    def close(self):
        self.stop.set();self.thread.join(.5);require(not self.thread.is_alive(),'Fake responder did not join')
        for fd in self.fds:
            try:os.close(fd)
            except OSError:pass


def execute_example(bindings):
    buses=[];owners=None;rows=[]
    result={'schema':'private.fake-native-phase-owner-result.v1','status':'INCOMPLETE_FAKE_PIPE_STATE_MACHINE','rows':rows,
        'two_persistent_native_owners':True,'sd_exchange_integrated':False,'core_parity_verified':False,
        'native_exchange_performance_verified':False,'python_future_publication_used':False,'main_gil_and_semantic_validation_still_present':True,
        'hardware_opened':False,'motor_commands_sent':False,'output_allowed':False,'active_controller_qualification':False,'latency_improvement_proven':False}
    try:
        for bus in range(2):buses.append(FakeBus(bus))
        owners=Owners(bindings)
        owners.start(buses[0].request_tx,buses[0].reply_rx,buses[1].request_tx,buses[1].reply_rx)
        for generation in (1,2):
            owners.begin(generation,bindings.now()+BUDGET_NS,seed=1729)
            for phase in (1,2):
                pair=None
                while pair is None:pair=owners.collect(generation,phase)
                validate_pair(pair);owners.acknowledge(pair,validated=True);rows.append({'generation':generation,'phase':phase,'records':[record_dict(r)for r in pair.records],'collected_actual_ns':owners.last_collect_ns})
            owners.submit_stop(generation,validated=True);pair=None
            while pair is None:pair=owners.collect(generation,3)
            validate_pair(pair);owners.acknowledge(pair,validated=True);rows.append({'generation':generation,'phase':3,'records':[record_dict(r)for r in pair.records],'collected_actual_ns':owners.last_collect_ns})
        status=status_dict(owners.status());closed=status_dict(owners.close())
        require(all([row[2]for row in bus.requests]==[1,2,3,1,2,3]for bus in buses),'Unexpected fake stage command')
        result.update(status='PASS_FAKE_PIPE_STATE_MACHINE',status_before_close=status,cleanup=closed)
    except BaseException as error:
        result['error']=type(error).__name__+': '+str(error)
        if owners is not None and owners.handle is not None:
            try:result['status_at_failure']=status_dict(owners.status())
            except BaseException as error:result['status_inspection_error']=type(error).__name__+': '+str(error)
    finally:
        errors=[]
        if owners is not None and owners.handle is not None:
            for action in (owners.cancel,owners.close):
                try:action()
                except BaseException as error:errors.append(type(error).__name__+': '+str(error))
        if owners is not None and owners.closed_status is not None:
            result.setdefault('cleanup',status_dict(owners.closed_status))
        for bus in buses:
            try:bus.close()
            except BaseException as error:errors.append(type(error).__name__+': '+str(error))
        result['cleanup_errors']=errors
        result['native_session_pending']=bool(owners is not None and owners.handle is not None)
        if errors or result['native_session_pending']:result['status']='INCOMPLETE_FAKE_PIPE_STATE_MACHINE'
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);p.add_argument('--execute-file-only',action='store_true');p.add_argument('--output',required=True)
    args=p.parse_args(argv);output=fresh(args.output)
    if not args.execute_file_only:
        result={'status':'FILE_ONLY_PLAN','source_sha256':hashlib.sha256(read(CPP)).hexdigest(),'fake_backend_only':True,'native_code_loaded':False,'pipes_opened':False,'hardware_opened':False,'output_allowed':False}
    else:
        library=fresh(output.with_name(output.stem+'-fake.so'));record=build(library);bindings=Bindings(library,record['library_sha256'],record)
        result=execute_example(bindings);result['build']=record
    with output.open('x')as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({'status':result['status'],'output_allowed':False}))
    if result['status']=='INCOMPLETE_FAKE_PIPE_STATE_MACHINE':raise SystemExit(1)


if __name__=='__main__':main()
