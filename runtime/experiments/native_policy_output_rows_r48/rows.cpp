// Isolated CPython bridge. No dispatcher, model, hardware or runtime selector.
#include <Python.h>
#include <ATen/ATen.h>
#include <torch/csrc/DynamicTypes.h>
#include <torch/csrc/autograd/python_variable.h>
#include <cmath>
#include <cstring>
#include <new>

namespace {
constexpr const char* capsule_name="sd.output.rows.r48.context";
struct Context {
    PyObject *torch,*tensor_type,*names,*attributes,*dispatch_mode,*function_mode,*dtype,*layout;
    PyObject *torch_c,*dict_descriptor;
    bool canonical_modes;
};
struct OwnedRef {
    PyObject* value;
    explicit OwnedRef(PyObject* object):value(object){}
    ~OwnedRef(){Py_XDECREF(value);}
    OwnedRef(const OwnedRef&)=delete;
    OwnedRef& operator=(const OwnedRef&)=delete;
};
void release_context(Context* c) {
    Py_DECREF(c->torch);Py_DECREF(c->tensor_type);Py_DECREF(c->names);Py_DECREF(c->attributes);
    Py_DECREF(c->dispatch_mode);Py_DECREF(c->function_mode);Py_DECREF(c->dtype);Py_DECREF(c->layout);
    Py_DECREF(c->torch_c);Py_DECREF(c->dict_descriptor);delete c;
}
void destroy_context(PyObject* capsule) {
    auto* c=static_cast<Context*>(PyCapsule_GetPointer(capsule,capsule_name));
    if(!c) {PyErr_Clear();return;}
    release_context(c);
}
bool canonical_builtin(PyObject* function,PyObject* module,const char* name) {
    if(!PyCFunction_Check(function)||PyCFunction_GetSelf(function)!=module)return false;
    // PyCFunction descriptors have a fixed C name, with no Python callback.
    return std::strcmp(reinterpret_cast<PyCFunctionObject*>(function)->m_ml->ml_name,name)==0;
}
PyObject* module_item(PyObject* module,const char* name) {
    // Identity selection must neither invoke module __getattr__ nor raise
    // AttributeError for a removed optional fast-path guard.
    if(!PyModule_CheckExact(module))return nullptr;
    return PyDict_GetItemString(PyModule_GetDict(module),name);
}
PyObject* type_dict(PyTypeObject* type) {
#if PY_VERSION_HEX >= 0x030C0000
    // PyType_GetDict returns a new reference. Since 3.12 static builtins keep
    // this dictionary on interpreter state. Both branches own one reference.
    return PyType_GetDict(type);
#else
    return Py_XNewRef(type->tp_dict);
#endif
}
PyObject* type_item(PyObject* type,PyObject* name) {
    // Read the raw descriptor through MRO dictionaries. PyObject_GetAttr
    // would execute a custom descriptor even for a class-level guard lookup.
    if(!PyType_Check(type))return nullptr;
    PyObject* mro=reinterpret_cast<PyTypeObject*>(type)->tp_mro;
    if(!mro||!PyTuple_CheckExact(mro))return nullptr;
    for(Py_ssize_t i=0;i<PyTuple_GET_SIZE(mro);++i) {
        PyObject* base=PyTuple_GET_ITEM(mro,i);if(!PyType_Check(base))return nullptr;
        OwnedRef dict(type_dict(reinterpret_cast<PyTypeObject*>(base)));
        if(!dict.value||!PyDict_CheckExact(dict.value))return nullptr;
        PyObject* current=PyDict_GetItemWithError(dict.value,name);
        if(current||PyErr_Occurred())return Py_XNewRef(current);
    }
    return nullptr;
}
PyObject* context(PyObject*,PyObject* args) {
    PyObject *torch,*tensor_type,*names,*attributes,*dispatch_mode,*function_mode,*dtype,*layout;
    if(!PyArg_ParseTuple(args,"OOOOOOOO",&torch,&tensor_type,&names,&attributes,&dispatch_mode,&function_mode,&dtype,&layout))return nullptr;
    if(!PyTuple_CheckExact(names)||!PyTuple_CheckExact(attributes)||PyTuple_GET_SIZE(names)!=11||PyTuple_GET_SIZE(attributes)!=11||
       !PyType_Check(tensor_type)||!PyCallable_Check(dispatch_mode)||!PyCallable_Check(function_mode)) {
        PyErr_SetString(PyExc_ValueError,"Exact bridge context required");return nullptr;
    }
    for(Py_ssize_t i=0;i<11;++i)if(!PyUnicode_CheckExact(PyTuple_GET_ITEM(names,i))) {
        PyErr_SetString(PyExc_ValueError,"Exact guard names required");return nullptr;
    }
    PyObject* torch_c=module_item(torch,"_C");
    if(!torch_c){PyErr_SetString(PyExc_ValueError,"Ordinary Torch module binding required");return nullptr;}
    Py_INCREF(torch_c);
    OwnedRef class_dict(type_dict(reinterpret_cast<PyTypeObject*>(tensor_type)));
    if(!class_dict.value){Py_DECREF(torch_c);return nullptr;}
    PyObject* descriptor=PyDict_GetItemString(class_dict.value,"__dict__");
    // A custom __dict__ property would introduce a callback before conversion.
    // A genuine getset descriptor must belong to this exact Tensor class.
    if(!descriptor||Py_TYPE(descriptor)!=&PyGetSetDescr_Type||PyDescr_TYPE(descriptor)!=reinterpret_cast<PyTypeObject*>(tensor_type)||
       PyUnicode_CompareWithASCIIString(PyDescr_NAME(descriptor),"__dict__")!=0)descriptor=Py_None;
    auto* c=new(std::nothrow) Context{Py_NewRef(torch),Py_NewRef(tensor_type),Py_NewRef(names),Py_NewRef(attributes),
                        Py_NewRef(dispatch_mode),Py_NewRef(function_mode),Py_NewRef(dtype),Py_NewRef(layout),
                        torch_c,Py_NewRef(descriptor),
                        canonical_builtin(dispatch_mode,torch_c,"_len_torch_dispatch_stack")&&
                        canonical_builtin(function_mode,torch_c,"_len_torch_function_stack")};
    // new(nothrow) does not evaluate the initializer when allocation fails.
    if(!c){Py_DECREF(torch_c);return PyErr_NoMemory();}
    PyObject* result=PyCapsule_New(c,capsule_name,destroy_context);
    if(!result)release_context(c);
    return result;
}
// Return -1 for a Python exception, 0 for fallback, and 1 for ordinary Tensor.
int matching_attribute(PyObject* obj,PyObject* name,PyObject* expected) {
    OwnedRef current(type_item(obj,name));if(PyErr_Occurred())return -1;
    return current.value&&current.value==expected;
}
int matching_attribute(PyObject* obj,const char* name,PyObject* expected) {
    PyObject* current=module_item(obj,name);if(PyErr_Occurred())return -1;
    return current&&current==expected;
}
int eligible(Context* c,PyObject* value,PyObject* count_object,int64_t& count) {
    // Preserve count/class/instance/mode guard ordering, with no cached result
    // spanning a lazy getter or a row conversion.
    if(!PyLong_CheckExact(count_object))return 0;
    int overflow=0;count=PyLong_AsLongLongAndOverflow(count_object,&overflow);
    if(PyErr_Occurred())return -1;
    if(overflow|| (count!=12&&count!=74))return 0;
    int match=matching_attribute(c->torch,"Tensor",c->tensor_type);if(match!=1)return match;
    for(Py_ssize_t i=0;i<11;++i) {
        match=matching_attribute(c->tensor_type,PyTuple_GET_ITEM(c->names,i),PyTuple_GET_ITEM(c->attributes,i));
        if(match!=1)return match;
    }
    // THPVariableClass is the actual Python binding exported by this Torch
    // build. Exclude Parameter and all subclasses before unpacking cdata.
    if(c->tensor_type!=THPVariableClass||reinterpret_cast<PyObject*>(Py_TYPE(value))!=c->tensor_type)return 0;
    if(c->dict_descriptor==Py_None)return 0;
    {
        OwnedRef class_dict(type_dict(reinterpret_cast<PyTypeObject*>(c->tensor_type)));
        if(!class_dict.value){if(PyErr_Occurred())return -1;return 0;}
        OwnedRef descriptor(Py_XNewRef(PyDict_GetItemString(class_dict.value,"__dict__")));
        if(descriptor.value!=c->dict_descriptor)return 0;
    }
    // The checked canonical descriptor supplies the actual instance dict;
    // bypass Python attribute dispatch so no selection callback is introduced.
    auto* dict_getset=reinterpret_cast<PyGetSetDescrObject*>(c->dict_descriptor)->d_getset;
    if(!dict_getset->get)return 0;
    PyObject* dict=dict_getset->get(value,dict_getset->closure);if(!dict)return -1;
    bool ordinary_empty=PyDict_CheckExact(dict)&&PyDict_Size(dict)==0;Py_DECREF(dict);if(!ordinary_empty)return 0;
    // A constructor run while module constants are patched cannot establish
    // canonical dtype/layout identity from the supplied Python objects.
    if(c->dtype!=reinterpret_cast<PyObject*>(torch::getTHPDtype(at::kFloat))||
       c->layout!=reinterpret_cast<PyObject*>(torch::getTHPLayout(at::kStrided)))return 0;
    if(!c->canonical_modes)return 0;
    match=matching_attribute(c->torch,"_C",c->torch_c);if(match!=1)return match;
    match=matching_attribute(c->torch_c,"_len_torch_dispatch_stack",c->dispatch_mode);if(match!=1)return match;
    match=matching_attribute(c->torch_c,"_len_torch_function_stack",c->function_mode);if(match!=1)return match;
    for(PyObject* mode:{c->dispatch_mode,c->function_mode}) {
        PyObject* current=PyObject_CallNoArgs(mode);if(!current)return -1;
        bool empty=PyLong_CheckExact(current)&&PyLong_AsLong(current)==0;Py_DECREF(current);
        if(PyErr_Occurred())return -1;if(!empty)return 0;
    }
    // Check module constants too. Metadata descriptors were checked above;
    // direct ATen metadata performs no Python method or dispatcher call.
    match=matching_attribute(c->torch,"float32",c->dtype);if(match!=1)return match;
    match=matching_attribute(c->torch,"strided",c->layout);if(match!=1)return match;
    const at::Tensor& tensor=THPVariable_Unpack(value);
    return tensor.device().is_cpu()&&tensor.layout()==at::kStrided&&tensor.scalar_type()==at::kFloat&&
           tensor.dim()==2&&tensor.size(0)==1&&tensor.size(1)==count&&tensor.is_contiguous()&&!tensor.is_neg()&&!tensor.is_conj();
}
PyObject* status(int code,PyObject* list) {
    if(!list)return nullptr;
    PyObject* number=PyLong_FromLong(code);if(!number){Py_DECREF(list);return nullptr;}
    PyObject* result=PyTuple_New(2);if(!result){Py_DECREF(number);Py_DECREF(list);return nullptr;}
    PyTuple_SET_ITEM(result,0,number);PyTuple_SET_ITEM(result,1,list);return result;
}
PyObject* row(PyObject*,PyObject* args) {
    PyObject *capsule,*value,*count_object;
    if(!PyArg_ParseTuple(args,"OOO",&capsule,&value,&count_object))return nullptr;
    auto* c=static_cast<Context*>(PyCapsule_GetPointer(capsule,capsule_name));if(!c)return nullptr;
    try {
        int64_t count=0;int ok=eligible(c,value,count_object,count);
        if(ok<0)return nullptr;if(!ok)return status(99,PyList_New(0));
        // All Python guards and native metadata checks finish before storage.
        // Keep the GIL; no callback occurs between metadata and pointer access.
        const at::Tensor& tensor=THPVariable_Unpack(value);
        const float* data=tensor.const_data_ptr<float>();
        PyObject* list=PyList_New(count);if(!list)return nullptr;
        for(int64_t i=0;i<count;++i) {
            double item=static_cast<double>(data[i]);
            if(!std::isfinite(item)){Py_DECREF(list);return status(2,PyList_New(0));}
            PyObject* number=PyFloat_FromDouble(item);
            if(!number){Py_DECREF(list);return nullptr;}
            PyList_SET_ITEM(list,i,number);
        }
        return status(0,list);
    }catch(const std::exception& error){PyErr_SetString(PyExc_RuntimeError,error.what());return nullptr;}
}
PyObject* is_eligible(PyObject*,PyObject* args) {
    PyObject *capsule,*value,*count_object;
    if(!PyArg_ParseTuple(args,"OOO",&capsule,&value,&count_object))return nullptr;
    auto* c=static_cast<Context*>(PyCapsule_GetPointer(capsule,capsule_name));if(!c)return nullptr;
    try {int64_t count=0;int ok=eligible(c,value,count_object,count);if(ok<0)return nullptr;return PyBool_FromLong(ok);}
    catch(const std::exception& error){PyErr_SetString(PyExc_RuntimeError,error.what());return nullptr;}
}
PyMethodDef methods[]={{"context",context,METH_VARARGS,nullptr},{"row",row,METH_VARARGS,nullptr},
                       {"eligible",is_eligible,METH_VARARGS,nullptr},{nullptr,nullptr,0,nullptr}};
PyModuleDef module={PyModuleDef_HEAD_INIT,"_sd_output_rows_bridge_r48",nullptr,-1,methods};
}
PyMODINIT_FUNC PyInit__sd_output_rows_bridge_r48(){return PyModule_Create(&module);}
