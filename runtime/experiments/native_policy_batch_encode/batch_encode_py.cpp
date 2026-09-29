// Offline CPython wrapper for batch_encode.cpp. It owns no device descriptor.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cmath>
#include <new>

#include "batch_encode.cpp"

namespace {
struct Context { AxisSpec axes[12]; };
constexpr const char* CAPSULE_NAME = "singularitydog.offline_batch_encoder_context.v1";

double number(PyObject* value, const char* name, bool allow_nonfinite = false) {
    if (PyBool_Check(value) || !(PyFloat_Check(value) || PyLong_Check(value))) {
        PyErr_Format(PyExc_ValueError, "%s must be finite numeric", name);
        return 0.0;
    }
    const double result = PyFloat_AsDouble(value);
    if (PyErr_Occurred()) return 0.0;
    if (!allow_nonfinite && !std::isfinite(result)) {
        PyErr_Format(PyExc_ValueError, "%s must be finite numeric", name);
        return 0.0;
    }
    return result;
}

void destroy_context(PyObject* capsule) {
    auto* context = static_cast<Context*>(PyCapsule_GetPointer(capsule, CAPSULE_NAME));
    if (!context) { PyErr_Clear(); return; }
    delete context;
}

PyObject* make_context(PyObject*, PyObject* args) {
    PyObject* specs = nullptr;
    if (!PyArg_ParseTuple(args, "O", &specs)) return nullptr;
    PyObject* rows = PySequence_Fast(specs, "Exactly twelve axis specs required");
    if (!rows) return nullptr;
    if (PySequence_Fast_GET_SIZE(rows) != 12) {
        Py_DECREF(rows);
        PyErr_SetString(PyExc_ValueError, "Exactly twelve axis specs required");
        return nullptr;
    }
    auto* context = new (std::nothrow) Context{};
    if (!context) { Py_DECREF(rows); return PyErr_NoMemory(); }
    for (int k = 0; k < 12; ++k) {
        PyObject* fields = PySequence_Fast(PySequence_Fast_GET_ITEM(rows, k),
                                           "Axis spec needs seven finite numbers");
        if (!fields) { delete context; Py_DECREF(rows); return nullptr; }
        if (PySequence_Fast_GET_SIZE(fields) != 7) {
            Py_DECREF(fields); delete context; Py_DECREF(rows);
            PyErr_SetString(PyExc_ValueError, "Axis spec needs seven finite numbers");
            return nullptr;
        }
        double values[7];
        for (int j = 0; j < 7; ++j) {
            values[j] = number(PySequence_Fast_GET_ITEM(fields, j), "Axis spec");
            if (PyErr_Occurred()) {
                Py_DECREF(fields); delete context; Py_DECREF(rows); return nullptr;
            }
        }
        Py_DECREF(fields);
        if (!(values[1] == 1.0 || values[1] == -1.0) ||
            !(values[2] < values[3]) || values[5] < 0.0 || values[6] < 0.0) {
            delete context; Py_DECREF(rows);
            PyErr_SetString(PyExc_ValueError, "Invalid axis spec range/sign/limits");
            return nullptr;
        }
        context->axes[k] = AxisSpec{values[0], values[1], values[2], values[3],
                                    values[4], values[5], values[6]};
    }
    Py_DECREF(rows);
    PyObject* capsule = PyCapsule_New(context, CAPSULE_NAME, destroy_context);
    if (!capsule) delete context;
    return capsule;
}

PyObject* encode(PyObject*, PyObject* args) {
    PyObject* capsule = nullptr;
    PyObject* command = nullptr;
    if (!PyArg_ParseTuple(args, "OO", &capsule, &command)) return nullptr;
    auto* context = static_cast<Context*>(PyCapsule_GetPointer(capsule, CAPSULE_NAME));
    if (!context) return nullptr;
    const char* names[4] = {"q_model_rad", "kp", "kd", "estimated_pd_torque_nm"};
    const char* numeric_names[4] = {"q", "kp", "kd", "estimated_pd_torque_nm"};
    PyObject* tuples[4] = {nullptr, nullptr, nullptr, nullptr};
    double values[4][12];
    for (int j = 0; j < 4; ++j) {
        tuples[j] = PyObject_GetAttrString(command, names[j]);
        if (!tuples[j]) goto failure;
        if (!PyTuple_CheckExact(tuples[j]) || PyTuple_GET_SIZE(tuples[j]) != 12) {
            PyErr_Format(PyExc_ValueError, "%s must be a twelve-value tuple", names[j]);
            goto failure;
        }
        for (int k = 0; k < 12; ++k) {
            values[j][k] = number(PyTuple_GET_ITEM(tuples[j], k), numeric_names[j], j == 3);
            if (PyErr_Occurred()) goto failure;
        }
    }
    {
        uint8_t wires[204];
        int32_t failed_id = 0;
        const int32_t status = sdbe_encode(context->axes, values[0], values[1],
                                           values[2], values[3], wires, &failed_id);
        if (status != OK) {
            const char* detail = status == INVALID_ARGUMENT ? "Invalid batch encoder argument" :
                status == NONFINITE_Q ? "q must be finite numeric" :
                status == NONFINITE_KP ? "kp must be finite numeric" :
                status == NONFINITE_KD ? "kd must be finite numeric" :
                status == SOFTWARE_CAP ? "Target/gain outside active software caps" :
                status == QUANTIZED_PHYSICAL_RANGE ? "quantized target outside physical range" :
                status == DISPLACEMENT ? "quantized target outside trial displacement" :
                status == ESTIMATED_PD ? "quantized estimated PD torque" :
                "Unknown batch encoder status";
            if (status <= SOFTWARE_CAP) PyErr_SetString(PyExc_ValueError, detail);
            else PyErr_Format(PyExc_RuntimeError, "ID%d %s", failed_id, detail);
            goto failure;
        }
        PyObject* front = PyList_New(6);
        PyObject* rear = PyList_New(6);
        if (!front || !rear) { Py_XDECREF(front); Py_XDECREF(rear); goto failure; }
        for (int k = 0; k < 12; ++k) {
            PyObject* wire = PyBytes_FromStringAndSize(
                reinterpret_cast<const char*>(wires + 17 * k), 17);
            if (!wire) { Py_DECREF(front); Py_DECREF(rear); goto failure; }
            if (k < 6) PyList_SET_ITEM(front, k, wire);
            else PyList_SET_ITEM(rear, k - 6, wire);
        }
        PyObject* result = PyDict_New();
        if (!result) { Py_DECREF(front); Py_DECREF(rear); goto failure; }
        const int front_ok = PyDict_SetItemString(result, "front", front);
        const int rear_ok = PyDict_SetItemString(result, "rear", rear);
        Py_DECREF(front); Py_DECREF(rear);
        if (front_ok || rear_ok) { Py_DECREF(result); goto failure; }
        for (auto* tuple : tuples) Py_DECREF(tuple);
        return result;
    }
failure:
    for (auto* tuple : tuples) Py_XDECREF(tuple);
    return nullptr;
}

PyMethodDef methods[] = {
    {"make_context", make_context, METH_VARARGS, "Validate immutable axis specs."},
    {"encode", encode, METH_VARARGS, "Encode and validate all twelve Type1 wires."},
    {nullptr, nullptr, 0, nullptr}
};
PyModuleDef module = {PyModuleDef_HEAD_INIT, "sdbe_native", nullptr, -1, methods};
}  // namespace

PyMODINIT_FUNC PyInit_sdbe_native(void) { return PyModule_Create(&module); }
