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
import functools
import importlib.util
import sys
import sysconfig

HERE=Path(__file__).resolve().parent
OBSERVER=HERE.parents[1]/'singularitydog_hw/policy_observer.py'
SHADOW=HERE.parents[1]/'singularitydog_hw/policy_shadow.py'
OBSERVER_SHA='21a18c24b19fc9eb38f2d9f172556f4928dec07a36eedc2d47a41b9cd990b0ad'
SHADOW_SHA='d46181303822563f24572f15ab8af6d3c7ebd82be4d5529b7aa4d38b683a8016'
BUILD_HELPER_SHA='daa624f1235b4cbb9953e4a33fea8720614f057da4d416aed0956e8693046df0'
BUILD_HELPER=HERE.parent/'native_policy_overnight/model_call_fastpath/run.py'
_LOADED={}
MODULE_NAME='_sd_output_rows_bridge_r48'
GUARD_NAMES=('__getattribute__','detach','cpu','tolist','is_contiguous','is_neg','is_conj','device','dtype','layout','shape')

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
    # A private Python extension eliminates the Torch operator dispatcher.
    # The original helper stays pinned as provenance, but this build also needs
    # the current interpreter headers and libtorch_python's Tensor binding.
    from torch.utils.cpp_extension import include_paths,library_paths
    path=fresh(path);cpp=read(HERE/'rows.cpp');read(helper,BUILD_HELPER_SHA)
    require(platform.system()in ('Darwin','Linux'),'CPU-only Darwin/Linux build required')
    command=['c++','-std=c++20','-O3','-ffp-contract=off','-shared','-fPIC',
             '-D_GLIBCXX_USE_CXX11_ABI='+str(int(torch._C._GLIBCXX_USE_CXX11_ABI)),
             str(HERE/'rows.cpp'),'-o',str(path),'-I',sysconfig.get_paths()['include']]
    for include in include_paths():command+=['-I',include]
    for library in library_paths():command+=['-L',library,'-Wl,-rpath,'+library]
    command+=['-ltorch_python','-ltorch_cpu','-lc10']
    if platform.system()=='Darwin':command+=['-undefined','dynamic_lookup']
    start=time.perf_counter_ns();result=subprocess.run(command,capture_output=True,text=True,timeout=180)
    require(result.returncode==0,'Output row Python bridge build failed: '+result.stderr[-4000:])
    require(read(HERE/'rows.cpp')==cpp,'C++ source changed during compile');read(helper,BUILD_HELPER_SHA)
    return {'schema':'private.output-row-python-bridge-build.r48.v1','source_sha256':sha(cpp),
            'library_path':str(path),'library_sha256':sha(read(path)),
            'build_helper_path':str(helper),'build_helper_sha256':BUILD_HELPER_SHA,
            'compile_ms':(time.perf_counter_ns()-start)/1e6,'compiler_argv':command,
            'compiler_stdout':result.stdout,'compiler_stderr':result.stderr,
            'python_version':sys.version,'torch_version':torch.__version__,
            'hardware_opened':False,'output_allowed':False}

class NativeRows:
    def __init__(self,torch,library,pin,build_record,reference):
        self.torch=torch;self.reference=reference;self.library=Path(library);self.pin=pin
        self.source=read(HERE/'rows.cpp');self.module_source=read(Path(__file__).absolute())
        read(self.library,pin)
        require(build_record['schema']=='private.output-row-python-bridge-build.r48.v1'
                and build_record['library_sha256']==pin and build_record['library_path']==str(self.library)
                and build_record['source_sha256']==sha(self.source)
                and build_record['build_helper_sha256']==BUILD_HELPER_SHA
                and build_record['python_version']==sys.version and build_record['torch_version']==torch.__version__,
                'Exact private build record required')
        key=(str(self.library),pin,sha(self.source))
        require(MODULE_NAME not in sys.modules or key in _LOADED,'Unmanaged Python bridge namespace already loaded')
        if key not in _LOADED:
            spec=importlib.util.spec_from_file_location(MODULE_NAME,self.library)
            require(spec is not None and spec.loader is not None,'Python bridge loader required')
            module=importlib.util.module_from_spec(spec);sys.modules[MODULE_NAME]=module
            try:spec.loader.exec_module(module)
            except BaseException:
                sys.modules.pop(MODULE_NAME,None);raise
            _LOADED[key]=module
        module=_LOADED[key]
        require(sys.modules.get(MODULE_NAME)is module,'Python bridge module identity differs')
        self.tensor_type=torch.Tensor
        # Capture pristine TensorBase descriptors. A patch already present at
        # construction also falls back, instead of trusting that patched method.
        self.attributes={name:getattr(torch._C.TensorBase,name)for name in GUARD_NAMES}
        # The ordinary helpers return None exactly when these native TLS
        # stacks are empty. Avoid any Python helper/global callback in guards.
        self.dispatch_mode=torch._C._len_torch_dispatch_stack
        self.function_mode=torch._C._len_torch_function_stack
        self.context=module.context(torch,self.tensor_type,GUARD_NAMES,tuple(self.attributes.values()),
                                    self.dispatch_mode,self.function_mode,torch.float32,torch.strided)
        self.op=functools.partial(module.row,self.context)
        self._eligible=functools.partial(module.eligible,self.context)
        self.native_calls=0;self.fallback_calls=0;self.verify()
    def eligible(self,value,count):return self._eligible(value,count)
    def row(self,value,count,label):
        code,result=self.op(value,count)
        if code==99:self.fallback_calls+=1;return self.reference.row(value,count,label)
        self.native_calls+=1
        self.reference.require(code in (0,2),'Unexpected output-row status')
        self.reference.require(code==0,'Invalid '+label)
        self.reference.require(type(result)is list and len(result)==count,'Invalid '+label)
        return result
    def verify(self):
        read(self.library,self.pin)
        require(read(HERE/'rows.cpp')==self.source and read(Path(__file__).absolute())==self.module_source,'Experiment source changed')
        self.reference.verify()
