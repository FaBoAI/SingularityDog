"""Genuine compiled CPU kernel parity, fallback and original failure ordering."""
from pathlib import Path
import struct,tempfile,unittest
from unittest import mock
import torch
from . import conversion as c

def bits(values):return tuple(struct.pack('<d',value)if type(value)is float else(type(value),value)for value in values)

class ConversionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name).resolve();cls.record=c.build(torch,cls.root/'row.so')
        cls.reference=c.Reference();cls.native=c.NativeRows(torch,cls.root/'row.so',cls.record['library_sha256'],cls.record,cls.reference)
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def inputs(self):return (torch.tensor([[0.,0.,-1.]*4]),torch.linspace(-1,1,12).reshape(1,12),torch.linspace(-5,5,74).reshape(1,74))
    def invoke(self,values,native=False):return self.reference.outputs(values[0],lambda:values[1],lambda:values[2],self.native.row if native else None)
    def same(self,values):
        results=[]
        for native in (False,True):
            try:results.append(('OK',tuple(bits(row)for row in self.invoke(values,native))))
            except BaseException as error:results.append((type(error),str(error)))
        self.assertEqual(*results);return results[0]

    def test_all98_float64_bits_signedzero_and_owned_lists(self):
        values=self.inputs();values[0][0,0]=-0.;values[1][0,0]=-0.;values[2][0,0]=-0.
        before=[v.view(torch.int32).clone()for v in values];old=self.native.native_calls;self.same(values)
        self.assertEqual(self.native.native_calls-old,3)
        result=self.invoke(values,True);self.assertEqual([len(row)for row in result],[12,12,74]);self.assertEqual(bits(result[0])[0],struct.pack('<d',-0.))
        result[0][0]=999;result[1].clear();result[2][0]=999
        self.same(values);self.assertTrue(all(torch.equal(a,v.view(torch.int32))for a,v in zip(before,values)))

    def test_nonfinite_each_output_same_reason_and_priority(self):
        for slot in range(3):
            for value in (float('nan'),float('inf'),-float('inf')):
                values=list(self.inputs());values[slot][0,0]=value
                error=self.same(values);self.assertEqual(error,(self.reference.error,'Invalid '+('target','actor output','observation')[slot]))
        values=list(self.inputs());values[0][0,0]=99;values[1][0,0]=float('nan');values[2][0,0]=float('nan')
        self.assertEqual(self.same(values),(self.reference.error,'Invalid actor output'))

    def test_lazy_getter_access_order_and_target_range_last(self):
        values=self.inputs();events=[]
        def actor():events.append('actor');return values[1]
        def observation():events.append('observation');return values[2]
        values[0][0,0]=float('nan')
        with self.assertRaisesRegex(self.reference.error,'Invalid target'):self.reference.outputs(values[0],actor,observation,self.native.row)
        self.assertEqual(events,[])
        values[0][0,0]=99
        with self.assertRaisesRegex(self.reference.error,'joint range'):self.reference.outputs(values[0],actor,observation,self.native.row)
        self.assertEqual(events,['actor','observation'])

    def test_actor_invalid_does_not_read_observation(self):
        values=self.inputs();values[1][0,0]=float('nan')
        with self.assertRaisesRegex(self.reference.error,'Invalid actor output'):
            self.reference.outputs(values[0],lambda:values[1],lambda:(_ for _ in ()).throw(AssertionError('read too early')),self.native.row)

    def test_shape_and_batch_fallbacks_same_reason(self):
        for slot,size in enumerate((12,12,74)):
            for shape in ((size,),(2,size),(1,size+1),(1,0),()):
                values=list(self.inputs());values[slot]=torch.zeros(shape)
                old=self.native.fallback_calls;self.same(values);self.assertGreater(self.native.fallback_calls,old)

    def test_dtype_bool_integer_float16_float64_keep_original_types(self):
        for dtype in (torch.bool,torch.int64,torch.float16,torch.float64):
            for slot in range(3):
                values=list(self.inputs());values[slot]=values[slot].to(dtype)
                old=self.native.fallback_calls;self.same(values);self.assertGreater(self.native.fallback_calls,old)
        value=torch.tensor([[.1]*12],dtype=torch.float64)
        self.assertEqual(bits(self.native.row(value,12,'actor output')),bits(self.reference.row(value,12,'actor output')))

    def test_noncontiguous_and_storage_offset(self):
        values=list(self.inputs());values[1]=torch.arange(24,dtype=torch.float32).reshape(1,24)[:,::2]
        self.assertFalse(values[1].is_contiguous());self.same(values)
        values[1]=torch.arange(24,dtype=torch.float32)[6:18].reshape(1,12);self.assertGreater(values[1].storage_offset(),0)
        self.assertTrue(self.native.eligible(values[1],12));self.same(values)

    def test_negative_view_never_reads_unresolved_storage(self):
        values=list(self.inputs());values[1]=torch._neg_view(values[1]);self.assertTrue(values[1].is_neg())
        self.assertFalse(self.native.eligible(values[1],12));self.same(values)
        self.assertEqual(self.native.op(values[1],12),(99,[]))

    def test_exact_subclass_uses_original_methods(self):
        class Custom(torch.Tensor):pass
        values=list(self.inputs());values[1]=values[1].as_subclass(Custom)
        self.assertFalse(self.native.eligible(values[1],12));self.same(values)

    def test_instance_method_override_and_class_method_patch_fall_back(self):
        values=list(self.inputs());values[1].tolist=lambda:[[7.]*12]
        self.assertFalse(self.native.eligible(values[1],12));self.same(values)
        with mock.patch.object(torch.Tensor,'tolist',lambda self:[[7.]*self.shape[-1]]):
            self.assertFalse(self.native.eligible(values[0],12));self.same(values)

    def test_indirect_class_attribute_override_uses_original_row(self):
        value=torch.zeros(1,12);original=torch.Tensor.__getattribute__
        def indirect(tensor,name):
            if name=='tolist':return lambda:[[.125]*12]
            return original(tensor,name)
        with mock.patch.object(torch.Tensor,'__getattribute__',indirect):
            self.assertFalse(self.native.eligible(value,12))
            self.assertEqual(self.native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
            self.assertEqual(self.native.row(value,12,'actor output'),[.125]*12)

    def test_duck_methods_and_tuple_row_retain_original_behavior(self):
        class Duck:
            def __init__(self,row):self.row=row;self.calls=[]
            def detach(self):self.calls.append('detach');return self
            def cpu(self):self.calls.append('cpu');return self
            def tolist(self):self.calls.append('tolist');return self.row
        good=Duck([[1.]*12]);self.assertEqual(self.native.row(good,12,'actor output'),[1.]*12);self.assertEqual(good.calls,['detach','cpu','tolist'])
        for rows in ((),[[True]*12],[(1.,)*12],[[1.]*11]):
            value=Duck(rows)
            def run(row):
                try:return row(value,12,'actor output')
                except BaseException as error:return type(error),str(error)
            self.assertEqual(run(self.reference.row),run(self.native.row))

    def test_function_and_dispatch_modes_fall_back(self):
        from torch.overrides import TorchFunctionMode
        from torch.utils._python_dispatch import TorchDispatchMode
        class Function(TorchFunctionMode):
            def __torch_function__(self,func,types,args=(),kwargs=None):return func(*args,**(kwargs or {}))
        class Dispatch(TorchDispatchMode):
            def __torch_dispatch__(self,func,types,args=(),kwargs=None):return func(*args,**(kwargs or {}))
        for mode in (Function(),Dispatch()):
            values=self.inputs()
            with mode:self.assertFalse(self.native.eligible(values[0],12));self.same(values)

    def test_float32_joint_endpoints_and_next_values(self):
        for lower in (True,False):
            values=list(self.inputs());caps=self.reference.lower if lower else self.reference.upper
            values[0]=torch.tensor([caps]);self.same(values)
            direction=torch.full((1,12),-float('inf')if lower else float('inf'))
            values[0]=torch.nextafter(values[0],direction)
            self.assertEqual(self.same(values),(self.reference.error,'Policy target outside registered joint range'))

    def test_native_metadata_guard_bounds_before_pointer_access(self):
        for value,count in ((torch.zeros(12),12),(torch.zeros(1,12,dtype=torch.float64),12),(torch.zeros(1,74),75),(torch.zeros(1,12),-1),(torch.zeros(1,12),2**30)):
            self.assertEqual(self.native.op(value,count),(99,[]))

    def test_requires_grad_no_input_or_gradient_mutation(self):
        values=[v.requires_grad_()for v in self.inputs()];self.same(values)
        self.assertTrue(all(v.grad is None and v.requires_grad for v in values))

    def test_build_source_and_binary_pins_reject_tamper(self):
        wrong=dict(self.record,source_sha256='a'*64)
        with self.assertRaises(ValueError):c.NativeRows(torch,self.root/'row.so',self.record['library_sha256'],wrong,self.reference)
        with self.assertRaises(ValueError):c.NativeRows(torch,self.root/'row.so','a'*64,self.record,self.reference)
        self.native.verify()

    def test_each_guarded_class_descriptor_patch_falls_back(self):
        value=torch.zeros(1,12)
        for name in c.GUARD_NAMES:
            with self.subTest(name=name),mock.patch.object(torch.Tensor,name,object()):
                self.assertFalse(self.native.eligible(value,12))
                self.assertEqual(self.native.op(value,12),(99,[]))

    def test_lazy_getter_method_patch_rechecked_before_actor(self):
        values=self.inputs();events=[]
        original=torch.Tensor.tolist
        def actor():
            events.append('actor');torch.Tensor.tolist=lambda self:[[.125]*self.shape[-1]]
            return values[1]
        def observation():events.append('observation');return values[2]
        try:
            result=self.reference.outputs(values[0],actor,observation,self.native.row)
            self.assertEqual(result[1],[.125]*12);self.assertEqual(result[2],[.125]*74)
            self.assertEqual(events,['actor','observation'])
        finally:torch.Tensor.tolist=original

    def test_lazy_getter_instance_override_rechecked_before_actor(self):
        values=self.inputs()
        def actor():
            values[1].detach=lambda:values[1];values[1].cpu=lambda:values[1]
            values[1].tolist=lambda:[[.25]*12];return values[1]
        result=self.reference.outputs(values[0],actor,lambda:values[2],self.native.row)
        self.assertEqual(result[1],[.25]*12)

    def test_modes_rechecked_after_lazy_getter_and_original_events_preserved(self):
        from torch.overrides import TorchFunctionMode
        values=self.inputs()
        class Mode(TorchFunctionMode):
            def __init__(self):self.events=[]
            def __torch_function__(self,func,types,args=(),kwargs=None):
                self.events.append(func.__name__);return func(*args,**(kwargs or {}))
        def invoke(row):
            mode=Mode()
            def actor():mode.__enter__();return values[1]
            try:result=self.reference.outputs(values[0],actor,lambda:values[2],row)
            finally:mode.__exit__(None,None,None)
            return tuple(bits(r)for r in result),mode.events
        self.assertEqual(invoke(self.native.row),invoke(self.reference.row))

    def test_boolean_and_huge_counts_use_original_fallback(self):
        value=torch.zeros(1,12)
        for count in (True,False,12.,2**100,-2**100):
            self.assertEqual(self.native.op(value,count),(99,[]))
            def run(row):
                try:return row(value,count,'actor output')
                except BaseException as error:return type(error),str(error)
            self.assertEqual(run(self.native.row),run(self.reference.row))

    def test_torch_module_type_and_constants_patch_fall_back(self):
        value=torch.zeros(1,12)
        for name in ('Tensor','float32','strided'):
            with self.subTest(name=name),mock.patch.object(torch,name,object()):
                self.assertFalse(self.native.eligible(value,12))
                self.assertEqual(self.native.op(value,12),(99,[]))

    def test_dtype_conjugate_and_parameter_direct_bridge_metadata_fallback(self):
        for value in (torch.zeros(1,12,dtype=torch.complex64).conj(),torch.nn.Parameter(torch.zeros(1,12))):
            self.assertFalse(self.native.eligible(value,12));self.assertEqual(self.native.op(value,12),(99,[]))
            values=list(self.inputs());values[1]=value;self.same(values)

    def test_torch_and_python_build_versions_must_match(self):
        for name in ('torch_version','python_version'):
            wrong=dict(self.record);wrong[name]='wrong'
            with self.subTest(name=name),self.assertRaises(ValueError):
                c.NativeRows(torch,self.root/'row.so',self.record['library_sha256'],wrong,self.reference)

    def another(self):return c.NativeRows(torch,self.root/'row.so',self.record['library_sha256'],self.record,self.reference)

    def test_constants_patched_before_construction_require_original_fallback(self):
        value=torch.zeros(1,12)
        for name,replacement in (('float32',torch.float64),('float32',object()),('strided',torch.sparse_coo),('strided',object())):
            with self.subTest(name=name,replacement=str(replacement)),mock.patch.object(torch,name,replacement):
                native=self.another();self.assertFalse(native.eligible(value,12));self.assertEqual(native.op(value,12),(99,[]))
                self.assertEqual(native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
                self.assertEqual(native.native_calls,0)

    def test_class_methods_patched_before_construction_require_original_fallback(self):
        value=torch.zeros(1,12)
        with mock.patch.object(torch.Tensor,'tolist',lambda self:[[.125]*12]):
            native=self.another();self.assertFalse(native.eligible(value,12));self.assertEqual(native.op(value,12),(99,[]))
            self.assertEqual(native.row(value,12,'actor output'),[.125]*12)

    def test_custom_dict_descriptor_never_invokes_guard_callback(self):
        events=[]
        class Custom(torch.Tensor):
            @property
            def __dict__(self):events.append('dict');raise AssertionError('Custom dict getter must not run')
        value=torch.zeros(1,12).as_subclass(Custom)
        # Tensor's own class __dict__ cannot be replaced through setattr. A
        # subclass can supply this property, including before construction.
        with mock.patch.object(torch,'Tensor',Custom):
            for native in (self.native,self.another()):
                self.assertFalse(native.eligible(value,12));self.assertEqual(native.op(value,12),(99,[]))
                self.assertEqual(native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
        self.assertEqual(events,[])

    def test_custom_dict_subclass_never_invokes_boolean_callback(self):
        value=torch.zeros(1,12);events=[]
        class Custom(dict):
            def __bool__(self):events.append('bool');raise AssertionError('Custom dict bool must not run')
        value.__dict__=Custom()
        self.assertFalse(self.native.eligible(value,12));self.assertEqual(self.native.op(value,12),(99,[]))
        self.assertEqual(self.native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
        self.assertEqual(events,[])

    def test_patched_tls_getters_never_invoked_and_fall_back(self):
        value=torch.zeros(1,12);events=[]
        def custom():events.append('mode');raise AssertionError('Custom TLS getter must not run')
        for name in ('_len_torch_dispatch_stack','_len_torch_function_stack'):
            with self.subTest(name=name),mock.patch.object(torch._C,name,custom):
                for native in (self.native,self.another()):
                    self.assertFalse(native.eligible(value,12));self.assertEqual(native.op(value,12),(99,[]))
                    self.assertEqual(native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
        self.assertEqual(events,[])

    def test_python_mode_helper_side_effects_not_called(self):
        value=torch.zeros(1,12);events=[]
        def custom():events.append('helper');raise AssertionError('Python helper must not run')
        with mock.patch('torch.utils._python_dispatch._get_current_dispatch_mode',custom),mock.patch.object(torch.overrides,'_get_current_function_mode',custom):
            native=self.another();self.assertTrue(native.eligible(value,12))
            self.assertEqual(native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
        self.assertEqual(events,[])

    def deleted_module_guard(self,name):
        value=torch.tensor([[.125]*12]);original=vars(torch)[name]
        try:
            delattr(torch,name)
            self.assertFalse(self.native.eligible(value,12));self.assertEqual(self.native.op(value,12),(99,[]))
            self.assertEqual(self.native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
        finally:setattr(torch,name,original)
    def test_deleted_torch_float32_falls_back_without_extra_error(self):self.deleted_module_guard('float32')
    def test_deleted_torch_strided_falls_back_without_extra_error(self):self.deleted_module_guard('strided')
    def test_deleted_torch_tensor_falls_back_without_extra_error(self):self.deleted_module_guard('Tensor')

    def descriptor_guard(self,name):
        value=torch.tensor([[.125]*12]);events=[]
        class Descriptor:
            def __get__(self,instance,owner):events.append('get');raise AssertionError('Guard must not execute descriptors')
        with mock.patch.object(torch.Tensor,name,Descriptor()):
            self.assertFalse(self.native.eligible(value,12));self.assertEqual(self.native.op(value,12),(99,[]))
            self.assertEqual(self.native.row(value,12,'actor output'),self.reference.row(value,12,'actor output'))
        self.assertEqual(events,[])
    def test_device_descriptor_falls_back_without_extra_callback(self):self.descriptor_guard('device')
    def test_shape_descriptor_falls_back_without_extra_callback(self):self.descriptor_guard('shape')

    def test_type_dictionary_references_do_not_grow(self):
        import gc,sys
        dictionaries=[gc.get_referents(vars(cls))[0]for cls in (torch.Tensor,torch._C.TensorBase,object)]
        self.assertTrue(all(type(value)is dict for value in dictionaries))
        counts=lambda:tuple(sys.getrefcount(value)for value in dictionaries)
        value=torch.zeros(1,12)
        for _ in range(10):self.native.eligible(value,12);self.native.row(value,12,'actor output')
        before=counts()
        for _ in range(1000):self.native.eligible(value,12);self.native.row(value,12,'actor output')
        self.assertEqual(counts(),before)
        # Exercise short-circuit mismatch and context creation/destruction too.
        with mock.patch.object(torch.Tensor,'shape',object()):
            before=counts()
            for _ in range(1000):self.native.eligible(value,12);self.native.row(value,12,'actor output')
            self.assertEqual(counts(),before)
        before=counts()
        for _ in range(100):
            another=self.another();another.eligible(value,12);del another
        self.assertEqual(counts(),before)

if __name__=='__main__':unittest.main()
