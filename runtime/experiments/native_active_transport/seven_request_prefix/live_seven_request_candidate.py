"""PRIVATE LIVE held-Type1 transport prototype; NOT qualified for any live execution. No open/configure of CAN devices.

Six immutable mode2 held-Type1 feedback records can be published while the SAME native
session owner awaits its seventh voltage reply. A prefix is scoped partial
combined-seven evidence; it never certifies native idle, voltage, a completed
phase, or eligibility for live policy output. The caller-supplied held bytes and raw
bounds are transport checks only; genuine prior successful Type1/profile/source/
boot/power/PD/freshness admission remains REQUIRED and is not implemented here. Full raw7 remains authoritative.
"""
import ctypes as C
from concurrent.futures import Future
from dataclasses import dataclass
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import stat
import struct
import threading
import time
import weakref
from singularitydog_hw import native_active_transport as active
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire


class LiveHoldLimits(C.Structure):
    _fields_=[('lower',C.c_double*6),('upper',C.c_double*6)]


class SplitMeta(C.Structure):
    _fields_ = [(k,C.c_uint64) for k in ('generation','snapshot_ns','final_native_ns')]+[
        ('received_mask',C.c_uint32),('scope',C.c_uint32)]


_SIGNATURES = {
    'sda_live_split_feedback_voltage_abi': ((),C.c_uint32),
    'sda_live_split_create': ((C.c_void_p,C.c_uint64,C.c_int,C.c_int,C.c_int,C.POINTER(C.c_ubyte),C.c_uint32,C.POINTER(LiveHoldLimits),C.POINTER(C.c_char),C.c_uint32),C.c_void_p),
    'sda_live_split_exchange': ((C.c_void_p,C.POINTER(C.c_ubyte),C.c_uint32,C.c_uint64,
        C.POINTER(Record),C.POINTER(active.Stats),C.POINTER(C.c_char),C.c_uint32),C.c_int),
    'sda_live_split_take_prefix': ((C.c_void_p,C.c_uint64,C.POINTER(Record),C.POINTER(active.Stats),
        C.POINTER(SplitMeta),C.POINTER(C.c_char),C.c_uint32),C.c_int),
    'sda_live_split_destroy': ((C.c_void_p,C.POINTER(C.c_char),C.c_uint32),C.c_int),
}


def load_candidate(path):
    path=Path(path).resolve(strict=True)
    record=json.loads((path.parent/'build-record.json').read_text())
    lib=active.load_library(path,expected_sha256=record['binary_sha256'])
    functions={}
    for name,(args,result) in _SIGNATURES.items():
        function=getattr(lib,name,None)
        if not isinstance(function,C._CFuncPtr):
            raise ValueError('Explicit candidate requires complete optional split ABI')
        function.argtypes=list(args);function.restype=result;functions[name]=function
    if functions['sda_live_split_feedback_voltage_abi']()!=1:
        raise ValueError('Split ABI/layout mismatch')
    lib._private_live_split_functions=functions
    lib._private_live_split_source_sha256=record['source_sha256']
    lib._private_live_split_binary_sha256=record['binary_sha256']
    return lib


def _verify(lib):
    expected=getattr(lib,'_private_live_split_functions',{})
    for name,(args,result) in _SIGNATURES.items():
        actual=getattr(lib,name,None)
        if (actual is not expected.get(name) or not isinstance(actual,C._CFuncPtr) or
            actual._flags_!=C._FUNCFLAG_CDECL or tuple(actual.argtypes or ())!=args or
            actual.restype is not result or getattr(actual,'errcheck',None) is not None):
            raise ValueError('Split selected capability/function/signature changed')
    if expected['sda_live_split_feedback_voltage_abi']()!=1:
        raise ValueError('Split ABI changed')


def _binding(fd,access):
    st=os.fstat(fd);flags=fcntl.fcntl(fd,fcntl.F_GETFL)
    if not stat.S_ISFIFO(st.st_mode) or flags&os.O_ACCMODE!=access or not flags&os.O_NONBLOCK:
        raise RuntimeError('Split notification endpoint/mode changed')
    return st.st_dev,st.st_ino,st.st_rdev,stat.S_IFMT(st.st_mode),flags&(os.O_ACCMODE|os.O_NONBLOCK)


def _allocated_identity(fd):
    row=os.fstat(fd)
    return row.st_dev,row.st_ino,row.st_rdev,stat.S_IFMT(row.st_mode)


def _bytes(value):
    return bytes(memoryview(value).cast('B'))


def _frame_id(raw):
    if len(raw)!=17 or raw[:2]!=b'AT' or raw[6]!=8 or raw[-2:]!=b'\r\n':
        raise ValueError('Malformed split raw frame')
    encoded=int.from_bytes(raw[2:6],'big')
    if encoded&7!=4:raise ValueError('Split AT framing mismatch')
    return encoded>>3


def _query(mid):
    value=(((17<<24)|(0xfd<<8)|mid)<<3)|4
    return b'AT'+value.to_bytes(4,'big')+b'\x08'+struct.pack('<H',0x701c)+bytes(6)+b'\r\n'


@dataclass(frozen=True)
class Prefix:
    record_images: tuple
    stats_image: bytes
    generation: int
    snapshot_ns: int
    received_mask: int
    scope: str='PARTIAL_COMBINED_LIVE_HELD_TYPE1_SEVEN_REQUESTS'
    native_owner_joined: bool=False

    def decoded_records(self):
        return (Record*6).from_buffer_copy(b''.join(self.record_images))

    def decoded_stats(self):
        return active.Stats.from_buffer_copy(self.stats_image)


@dataclass(frozen=True)
class Full:
    record_images: tuple
    stats_image: bytes
    generation: int
    native_return_ns: int
    published_ns: int
    voltage: float
    scope: str='FULL_COMBINED_LIVE_HELD_TYPE1_SEVEN_REQUESTS'

    def decoded_records(self):
        return (Record*7).from_buffer_copy(b''.join(self.record_images))

    def decoded_stats(self):
        return active.Stats.from_buffer_copy(self.stats_image)


class LiveSevenRequestPhase:
    """Charge constructor through close inside the original absolute20ms cycle.

    Caller supplies a prestarted persistent executor and exact ActiveSession,
    cancellation descriptor. No descriptor is opened for serial/motor input.
    This object owns one bounded nonblocking notification pipe until the full
    native owner has joined, then close frees its reservation. It is single-use.
    """
    def __init__(self,session,executor,*,cancel_fd,generation,voltage_id,held_wires,raw_target_bounds_by_id):
        if type(session) is not active.ActiveSession:
            raise TypeError('Exact genuine ActiveSession required')
        if session._phase_pair is not None or not session._handle or session.poisoned:
            raise RuntimeError('Fresh unborrowed session required')
        if type(generation) is not int or not 0<generation<2**64:
            raise ValueError('Nonzero exact generation required')
        if type(voltage_id) is not int or not session.first_id<=voltage_id<session.first_id+6:
            raise ValueError('Voltage must use same bus motor')
        _verify(session.lib)
        if type(cancel_fd) is not int or not 0<=cancel_fd<2**31:
            raise ValueError('Exact readable original cancellation FD required')
        cancel_stat=os.fstat(cancel_fd)
        if fcntl.fcntl(cancel_fd,fcntl.F_GETFL)&os.O_ACCMODE==os.O_WRONLY:
            raise ValueError('Readable original cancellation FD required')
        self.session=session;self.executor=executor;self.cancel_fd=cancel_fd;self.generation=generation
        self._cancel_binding=(cancel_stat.st_dev,cancel_stat.st_ino,cancel_stat.st_rdev,stat.S_IFMT(cancel_stat.st_mode))
        self._caller=threading.current_thread();self._lock=threading.Lock();self._handle=None
        self._fds=();self._bindings=();self._allocated_ids=();self._closed=False;self._started=False;self._hint_error=None
        self.full_future=None;self.feedback_future=Future();self.feedback_future.set_running_or_notify_cancel()
        self._published_prefix=None;self._full_result=None;self._publication_error=None
        self._full_publication=threading.Event()
        self.full_records=(Record*7)();self.full_stats=active.Stats()
        self.prefix_records=(Record*6)();self.prefix_stats=active.Stats();self.meta=SplitMeta()
        self.error=C.create_string_buffer(256);self.take_error=C.create_string_buffer(256)
        if type(held_wires) is not tuple or len(held_wires)!=6 or any(type(w) is not bytes or len(w)!=17 for w in held_wires):
            raise ValueError('Exact six immutable previously validated held Type1 wires required')
        ids=tuple(range(session.first_id,session.first_id+6))
        if type(raw_target_bounds_by_id) is not dict or set(raw_target_bounds_by_id)!=set(ids):
            raise ValueError('Exact current six-axis raw target bounds required')
        self.live_limits=LiveHoldLimits()
        for index,mid in enumerate(ids):
            bounds=raw_target_bounds_by_id[mid]
            if (type(bounds) is not tuple or len(bounds)!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in bounds)):
                raise ValueError('Finite immutable current raw target bounds required')
            lo,hi=bounds
            if not -12.57<=lo<hi<=12.57 or hi-lo>math.radians(2):
                raise ValueError('LIVE current target bounds exceed original one-degree scope')
            self.live_limits.lower[index]=lo;self.live_limits.upper[index]=hi
        self.wires=held_wires+(_query(voltage_id),)
        self._held_array=(C.c_ubyte*102).from_buffer_copy(b''.join(held_wires))
        self._wire_array=(C.c_ubyte*119).from_buffer_copy(b''.join(self.wires))
        self.native_return_ns=0;self.deadline_ns=0;self.hints=0
        if not session.busy.acquire(False):raise RuntimeError('Native owner already busy')
        try:
            self._fds=os.pipe()
            self._allocated_ids=tuple(_allocated_identity(fd) for fd in self._fds)
            for fd in self._fds:os.set_blocking(fd,False)
            self._bindings=tuple(_binding(fd,mode) for fd,mode in zip(self._fds,(os.O_RDONLY,os.O_WRONLY)))
            self._handle=session.lib.sda_live_split_create(session._handle,generation,cancel_fd,*self._fds,self._held_array,6,C.byref(self.live_limits),self.error,256)
            if not self._handle:raise RuntimeError(self.error.value.decode())
        except BaseException as primary:
            cleanup=[]
            try:
                for index,fd in enumerate(self._fds):
                    try:
                        if index>=len(self._allocated_ids) or _allocated_identity(fd)!=self._allocated_ids[index]:
                            raise RuntimeError('Split rollback endpoint unbound/reused; unrelated descriptor retained')
                        os.close(fd)
                    except BaseException as error:cleanup.append(error)
            finally:session.busy.release()
            for error in cleanup:primary.add_note('Split constructor cleanup: '+repr(error))
            raise

    def _owner(self):
        if threading.current_thread() is not self._caller:raise RuntimeError('Split main owner changed')
        if self._closed:raise RuntimeError('Split phase closed')
        _verify(self.session.lib)

    def _check_cancel(self):
        current=os.fstat(self.cancel_fd)
        binding=(current.st_dev,current.st_ino,current.st_rdev,stat.S_IFMT(current.st_mode))
        if binding!=self._cancel_binding:raise RuntimeError('Split cancellation FD reused')
        if select.select([self.cancel_fd],[],[],0)[0]:raise RuntimeError('Cancelled split publication/takeout')

    def _verify_pipes(self):
        if tuple(_binding(fd,mode) for fd,mode in zip(self._fds,(os.O_RDONLY,os.O_WRONLY)))!=self._bindings:
            raise RuntimeError('Split pipe binding changed/reused')

    def start(self,*,deadline_ns):
        self._owner();self._verify_pipes();self._check_cancel()
        if self.feedback_future.done():
            raise RuntimeError('Original feedback Future externally completed before LIVE start')
        current=time.monotonic_ns()
        if self._started or type(deadline_ns) is not int or not current<deadline_ns<=current+20_000_000:
            raise ValueError('Single generation original absolute20ms deadline required')
        self._started=True;self.deadline_ns=deadline_ns
        try:
            self.full_future=self.executor.submit(self._native_owner)
            ref=weakref.ref(self)
            def publication_hint(future):
                owner=ref()
                if owner is None:return
                # Exact ORIGINAL executor Future; no result()/exception()/STOP
                # under the callback lock. After it is genuine-done only hint.
                with owner._lock:
                    if owner._closed or future is not owner.full_future:return
                    try:
                        owner._verify_pipes()
                        if os.write(owner._fds[1],b'\x02')!=1:raise RuntimeError('Split final hint partial write')
                    except BaseException as error:owner._hint_error=error
                    finally:owner._full_publication.set()
            self.full_future.add_done_callback(publication_hint)
        except BaseException as error:
            if not self.feedback_future.done():self.feedback_future.set_exception(error)
            raise
        return self.feedback_future,self.full_future

    def _validate(self,records,count):
        for i,record in enumerate(records):
            if (record.written!=17 or record.received!=17 or bytes(record.tx)!=self.wires[i] or
                record.deadline_ns!=self.deadline_ns or not record.start_ns or
                not record.start_ns<=record.finish_ns<=record.read_start_ns<=record.received_ns<self.deadline_ns):
                raise ValueError('Split incomplete/mutated/noncausal original transaction')
            kind=_frame_id(bytes(record.rx));mid=self.session.first_id+i
            if i<6:
                if kind!=((2<<24)|(2<<22)|(mid<<8)|0xfd) or bytes(record.rx)[7:10]==b'\x00\xc4\x56':
                    raise ValueError('Split feedback fault/mode/version rejected')
            else:
                dest=_frame_id(self.wires[6])&255
                if kind!=((17<<24)|(dest<<8)|0xfd):raise ValueError('Split voltage header mismatch')
                if bytes(record.rx)[7:11]!=b'\x1c\x70\x00\x00':raise ValueError('Split voltage parameter mismatch')
                voltage=struct.unpack('<f',bytes(record.rx)[11:15])[0]
                if not math.isfinite(voltage) or not 35<=voltage<=42:raise ValueError('Split voltage outside original35..42')
        if count==7:return voltage

    def _native_owner(self):
        try:
            _verify(self.session.lib)
            status=self.session.lib.sda_live_split_exchange(self._handle,self._wire_array,7,self.deadline_ns,
                self.full_records,C.byref(self.full_stats),self.error,256)
            self.native_return_ns=time.monotonic_ns()
            if status:raise active.ExchangeError(self.error.value.decode(),self.full_records,self.full_stats)
            if self.error.value:raise RuntimeError('Split success with native error')
            voltage=self._validate(self.full_records,7)
            if self.full_stats.end_ns>=self.deadline_ns or time.monotonic_ns()>=self.deadline_ns:
                raise TimeoutError('Split full publication missed original20ms deadline')
            self._full_result=Full(tuple(_bytes(r) for r in self.full_records),_bytes(self.full_stats),
                        self.generation,self.native_return_ns,time.monotonic_ns(),voltage)
            return self._full_result
        except BaseException as error:
            self.session.poisoned=True
            if not hasattr(error,'records'):error.records=self.full_records;error.stats=self.full_stats
            raise

    def take_feedback(self,future,*,generation):
        self._owner()
        if future is not self.feedback_future or generation!=self.generation:
            raise ValueError('Exact current original Future and generation required')
        self._verify_pipes();self._check_cancel()
        if self._hint_error is not None:raise self._hint_error
        if self.full_future is not None and self.full_future.done():
            # Known owner failure has priority even if old prefix became ready.
            error=self.full_future.exception()
            if error is not None:
                if not future.done():future.set_exception(error)
                raise error
        if time.monotonic_ns()>=self.deadline_ns:raise TimeoutError('Split feedback original deadline expired')
        if self._publication_error is not None:raise self._publication_error
        if future.done():
            value=future.result()
            if self._published_prefix is None or value is not self._published_prefix:
                raise RuntimeError('Original feedback Future was externally completed without native proof')
            self._check_cancel()
            if time.monotonic_ns()>=self.deadline_ns:raise TimeoutError('Split ready takeout deadline expired')
            return value
        self.take_error.value=b''
        status=self.session.lib.sda_live_split_take_prefix(self._handle,generation,self.prefix_records,
            C.byref(self.prefix_stats),C.byref(self.meta),self.take_error,256)
        if status<0:
            error=RuntimeError(self.take_error.value.decode());error.prefix_records=self.prefix_records
            error.prefix_stats=self.prefix_stats
            if not future.done():future.set_exception(error)
            raise error
        if self.take_error.value:raise RuntimeError('Split takeout error on success')
        if not status:return None
        if self.meta.generation!=generation or self.meta.scope!=1 or self.meta.received_mask&63!=63:
            raise RuntimeError('Split prefix publication proof malformed')
        self._validate(self.prefix_records,6)
        if (not self.prefix_stats.begin_ns<=self.meta.snapshot_ns==self.prefix_stats.end_ns<self.deadline_ns or
            time.monotonic_ns()>=self.deadline_ns):raise TimeoutError('Split prefix original deadline expired')
        value=Prefix(tuple(_bytes(r) for r in self.prefix_records),_bytes(self.prefix_stats),
                     generation,self.meta.snapshot_ns,self.meta.received_mask)
        # Genuine original Future; stable native snapshot copied first.
        self._check_cancel()
        if time.monotonic_ns()>=self.deadline_ns:raise TimeoutError("Split publication deadline expired")
        self._published_prefix=value
        try:future.set_result(value)
        except BaseException as error:
            self._publication_error=error;raise
        if time.monotonic_ns()>=self.deadline_ns:raise TimeoutError('Split callback publication deadline expired')
        return value

    def wait_feedback(self,future,*,generation):
        while True:
            value=self.take_feedback(future,generation=generation)
            if value is not None:return value
            self._verify_pipes();before=time.monotonic_ns();actual=C.c_uint64();error=C.create_string_buffer(256)
            status=self.session.lib.sda_wait_future_ready(self.cancel_fd,self._fds[0],self.deadline_ns,
                                                         C.byref(actual),error,256)
            after=time.monotonic_ns()
            if status<0:raise RuntimeError(error.value.decode())
            if not before<=actual.value<=after or error.value:raise RuntimeError('Split hint clock/error invalid')
            if status==1:raise TimeoutError('Split prefix wait reached original20ms deadline')
            self._verify_pipes()
            try:
                queued=os.read(self._fds[0],16)
                if not queued:raise RuntimeError('Split hint EOF')
                self.hints+=len(queued)
            except BlockingIOError:pass # raced hint is only a hint; originals rechecked

    def join(self):
        self._owner();self._check_cancel()
        if self.full_future is None:raise RuntimeError('Split not started')
        remaining=(self.deadline_ns-time.monotonic_ns())/1e9
        if remaining<=0:
            if self.full_future.done():self.full_future.result() # prioritize known native owner error
            raise TimeoutError('Split full join deadline expired')
        value=self.full_future.result(timeout=remaining)
        remaining=(self.deadline_ns-time.monotonic_ns())/1e9
        if remaining<=0 or not self._full_publication.wait(remaining):
            raise TimeoutError('Split final callback publication fence deadline expired')
        if value is not self._full_result or self._full_result is None:
            raise RuntimeError('Original full Future externally completed without native proof')
        if self._publication_error is not None:raise self._publication_error
        if self._hint_error is not None:raise self._hint_error
        self._check_cancel()
        if time.monotonic_ns()>=self.deadline_ns:raise TimeoutError('Split full takeout deadline expired')
        return value

    def close(self):
        if self._closed:return
        self._owner()
        # This cleanup join is not passing qualification or extending the20ms
        # deadline. Original error/raw remain saved; no STOP/reuse before join.
        primary=None
        if self.full_future is not None:
            try:self.full_future.result(timeout=.1)
            except BaseException as error:primary=error
            if not self.full_future.done():raise RuntimeError('Native owner not joined; pipe/session retained') from primary
            if not self._full_publication.wait(.1):raise RuntimeError('Full callback still active; pipe/session retained')
        with self._lock:
            error=C.create_string_buffer(256)
            status=self.session.lib.sda_live_split_destroy(self._handle,error,256)
            if status:raise RuntimeError(error.value.decode())
            self._handle=None;self._closed=True
            errors=[]
            for fd,expected,mode in zip(self._fds,self._bindings,(os.O_RDONLY,os.O_WRONLY)):
                try:
                    if _binding(fd,mode)!=expected:raise RuntimeError('Reused split FD retained, not closed')
                    os.close(fd)
                except BaseException as error:errors.append(error)
            self.session.busy.release()
            if not self.feedback_future.done():
                self.feedback_future.set_exception(primary or RuntimeError('Split closed before prefix publication'))
            if errors:raise errors[0]

    def __enter__(self):return self
    def __exit__(self,kind,value,trace):
        try:self.close()
        except BaseException as cleanup:
            if value is None:raise
            value.add_note('Split cleanup: '+repr(cleanup))
        return False
