"""CPU-only experiment preserving original row/fallback and guard ordering."""
import ast
import hashlib
import math
import os
from pathlib import Path
import platform
import re
import stat
import struct
import subprocess
import time
import types

HERE=Path(__file__).resolve().parent
OBSERVER=HERE.parents[1]/'singularitydog_hw/policy_observer.py'
SHADOW=HERE.parents[1]/'singularitydog_hw/policy_shadow.py'
OBSERVER_SHA='21a18c24b19fc9eb38f2d9f172556f4928dec07a36eedc2d47a41b9cd990b0ad'
SHADOW_SHA='d46181303822563f24572f15ab8af6d3c7ebd82be4d5529b7aa4d38b683a8016'
BUILD_HELPER_SHA='daa624f1235b4cbb9953e4a33fea8720614f057da4d416aed0956e8693046df0'
BUILD_HELPER=HERE.parent/'native_policy_overnight/model_call_fastpath/run.py'
_LOADED={}

def require(value,message):
    if not value:raise ValueError(message)
def sha(raw):return hashlib.sha256(raw).hexdigest()
def read(path,pin=None):
    if pin is not None:require(type(pin)is str and re.fullmatch('[0-9a-f]{64}',pin),'Explicit SHA256 required')
    path=Path(path);require(path.is_absolute() and not any(p.is_symlink()for p in (path,*path.parents)),'Absolute plain input required')
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|os.O_NOFOLLOW)
    try:
        before=os.fstat(fd);require(stat.S_ISREG(before.st_mode) and 0<=before.st_size<=64*1024*1024,'Bounded regular file required')
        with os.fdopen(fd,'rb',closefd=False)as stream:raw=stream.read(before.st_size+1)
        after=os.fstat(fd);require(len(raw)==before.st_size and (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)==(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns),'Stable complete input required')
    finally:os.close(fd)
    if pin is not None:require(sha(raw)==pin,'Pinned source/artifact differs')
    return raw
def fresh(path):
    path=Path(path);require(path.is_absolute() and path.parent.is_dir() and not path.exists() and not any(p.is_symlink()for p in (path,*path.parents)),'Fresh plain output required')
    require(not any((p/'.git').exists()for p in (path,*path.parents)),'Output outside Git required');return path

class Reference:
    def __init__(self,observer=OBSERVER,shadow=SHADOW):
        self.observer=Path(observer);self.shadow=Path(shadow)
        original=ast.parse(read(self.observer,OBSERVER_SHA));base=ast.parse(read(self.shadow,SHADOW_SHA))
        nodes=[node for node in original.body if isinstance(node,(ast.ClassDef,ast.FunctionDef)) and node.name in ('ObserverError','_require','_vector','_tensor_row')]
        finite=[node for node in base.body if isinstance(node,ast.FunctionDef) and node.name=='finite']
        require(len(nodes)==4 and len(finite)==1,'Exact original row/finite functions required')
        context={'math':math};exec(compile(ast.Module(body=finite,type_ignores=[]),str(self.shadow),'exec'),context)
        context['shadow']=types.SimpleNamespace(finite=context['finite'])
        exec(compile(ast.Module(body=nodes,type_ignores=[]),str(self.observer),'exec'),context)
        self.row=context['_tensor_row'];self.require=context['_require'];self.error=context['ObserverError']
        # The original range uses these same representable float32 endpoints.
        self.lower=[struct.unpack('<f',struct.pack('<f',x))[0]for x in [-.5,-.9,-2.2]*4]
        self.upper=[struct.unpack('<f',struct.pack('<f',x))[0]for x in [.5,1.2,-.08]*4]
    def verify(self):read(self.observer,OBSERVER_SHA);read(self.shadow,SHADOW_SHA)
    def outputs(self,target,actor_getter,observation_getter,row=None):
        # Lazy getters retain original model-attribute access/guard ordering.
        row=self.row if row is None else row
        target=row(target,12,'target')
        actor=row(actor_getter(),12,'actor output')
        observation=row(observation_getter(),74,'observation')
        self.require(all(lo<=q<=hi for q,lo,hi in zip(target,self.lower,self.upper)),
                     'Policy target outside registered joint range')
        return target,actor,observation

def build(torch,path,helper=BUILD_HELPER):
    path=fresh(path);cpp=read(HERE/'rows.cpp');helper=Path(helper)
    nodes=[node for node in ast.parse(read(helper,BUILD_HELPER_SHA)).body
           if isinstance(node,ast.FunctionDef) and node.name=='compile_actor']
    require(len(nodes)==1,'Exact isolated compile function required')
    context={'require':require,'platform':platform,'subprocess':subprocess,'time':time,'HERE':helper.parent}
    # Execute the pinned compiler function only, without importing model code.
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(helper),'exec'),context)
    elapsed=context['compile_actor'](torch,path,HERE/'rows.cpp')
    require(read(HERE/'rows.cpp')==cpp,'C++ source changed during compile');read(helper,BUILD_HELPER_SHA)
    return {'schema':'private.output-row-cpu-build.v1','source_sha256':sha(cpp),'library_path':str(path),
            'library_sha256':sha(read(path)),'build_helper_path':str(helper),'build_helper_sha256':BUILD_HELPER_SHA,'compile_ms':elapsed,
            'hardware_opened':False,'output_allowed':False}

class NativeRows:
    def __init__(self,torch,library,pin,build_record,reference):
        self.torch=torch;self.reference=reference;self.library=Path(library);self.pin=pin;self.source=read(HERE/'rows.cpp');self.module_source=read(Path(__file__).absolute())
        read(self.library,pin);require(build_record['schema']=='private.output-row-cpu-build.v1' and build_record['library_sha256']==pin and build_record['library_path']==str(self.library) and build_record['source_sha256']==sha(self.source) and build_record['build_helper_sha256']==BUILD_HELPER_SHA,'Exact private build record required')
        key=(str(self.library),pin,sha(self.source))
        existing=getattr(torch.ops.sd_output_row_fileonly_r1,'row',None)
        require(existing is None or key in _LOADED,'Unmanaged output-row namespace already loaded')
        if existing is None:torch.ops.load_library(str(self.library));_LOADED[key]=torch.ops.sd_output_row_fileonly_r1.row
        self.op=_LOADED[key];self.tensor_type=torch.Tensor
        self.attributes={name:getattr(torch.Tensor,name)for name in ('__getattribute__','detach','cpu','tolist','is_contiguous','is_neg','is_conj','device','dtype','layout','shape')}
        from torch.utils._python_dispatch import _get_current_dispatch_mode
        self.dispatch_mode=_get_current_dispatch_mode;self.function_mode=torch.overrides._get_current_function_mode
        self.native_calls=0;self.fallback_calls=0;self.verify()
    def eligible(self,value,count):
        return (type(count)is int and count in (12,74) and self.torch.Tensor is self.tensor_type
            and all(getattr(self.tensor_type,name)is method for name,method in self.attributes.items())
            and type(value)is self.tensor_type and not value.__dict__ and self.dispatch_mode()is None and self.function_mode()is None
            and value.device.type=='cpu' and value.dtype is self.torch.float32 and value.layout is self.torch.strided
            and tuple(value.shape)==(1,count) and value.is_contiguous() and not value.is_neg() and not value.is_conj())
    def row(self,value,count,label):
        if not self.eligible(value,count):self.fallback_calls+=1;return self.reference.row(value,count,label)
        code,result=self.op(value,count);self.native_calls+=1
        if code==99:self.fallback_calls+=1;return self.reference.row(value,count,label)
        self.reference.require(code in (0,2),'Unexpected output-row status')
        self.reference.require(code==0,'Invalid '+label)
        self.reference.require(type(result)is list and len(result)==count,'Invalid '+label)
        return result
    def verify(self):
        read(self.library,self.pin);require(read(HERE/'rows.cpp')==self.source and read(Path(__file__).absolute())==self.module_source,'Experiment source changed');self.reference.verify()
